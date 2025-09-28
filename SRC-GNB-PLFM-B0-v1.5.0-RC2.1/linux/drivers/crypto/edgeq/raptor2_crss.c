// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 EdgeQ Inc.
 */

#include <crypto/internal/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/authenc.h>
#include <crypto/internal/des.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/skcipher.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rtnetlink.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <asm/cacheflush.h>
#include <crypto/gcm.h>
#include <crypto/scatterwalk.h>

#include "raptor2_crss.h"
#include "raptor2_crss_qau_regs.h"
#include "raptor2_crss_hw_ring.h"


static inline void crss_key_table_init(struct raptor2_crss_engine *engine)
{
	int i;

	for (i = 0; i < MAX_KEY_ENTRIES; i++)
		atomic_set(&engine->key_table[i], 0);
}

static inline int get_crss_key_idx(struct raptor2_crss_engine *engine)
{
	int i;

	for (i = 0; i < MAX_KEY_ENTRIES; i++) {
		if (!atomic_read(&engine->key_table[i])) {
			atomic_set(&engine->key_table[i], 1);
			break;
		}
	}

	return i;
}


static inline void release_crss_key_idx(struct raptor2_crss_engine *engine, int idx)
{
	atomic_set(&engine->key_table[idx], 0);
}


static inline struct raptor2_crss_alg *to_raptor2_crss_skcipher(struct skcipher_alg *alg)
{
	return alg ? container_of(alg, struct raptor2_crss_alg, alg) : NULL;
}

static inline struct raptor2_crss_aead *to_raptor2_crss_aead(struct aead_alg *alg)
{
	return container_of(alg, struct raptor2_crss_aead, alg);
}


static inline int get_crss_queue(struct raptor2_crss_engine *engine)
{
	if (engine->curr_queue_idx == engine->num_queues)
		engine->curr_queue_idx = 0;

	return engine->curr_queue_idx++;
}

static inline void release_crss_queue(struct raptor2_crss_engine *engine, int queue)
{
	/* Currently, queues are used in a round robbin way
	 * without being uniquely mapped to a context. Hence,
	 * no need to release the queue
	 */
}

static int raptor2_crss_ctx_init(struct raptor2_crss_engine *engine,
					struct raptor2_crss_generic_ctx *ctx)
{
	int curr_q, key_idx;

	curr_q = get_crss_queue(engine);

	key_idx = get_crss_key_idx(engine);
	if (key_idx == MAX_KEY_ENTRIES)
		return -ENOSPC;

	ctx->curr_q = curr_q;
	ctx->new_key = 0;
	ctx->key_idx = key_idx;
	ctx->engine = engine;

	return 0;
}

static inline int get_packet_index(struct crss_queue_pair_ctx *queue)
{
	if (queue->current_pkt_idx == CRSS_STSQ_RING_SIZE)
		queue->current_pkt_idx = 0;

	return queue->current_pkt_idx++;
}

static inline void *get_next_cmd_ptr(char *ptr, int size, int sg_nents)
{
	return (ptr + size + (sg_nents * sizeof(struct crss_dma_scatter)));
}

static inline uint32_t *crss_cmd_queue_alloc(struct crss_hw_ring_ctx *cmdq, int cmd_size)
{
	uint32_t *write_ptr = NULL;

	if (crss_get_free_cmd_buff(cmdq))
		write_ptr = crss_hw_ring_alloc_cmd_buf(cmdq, cmd_size);

	return write_ptr;
}

static inline int poll_status_ring(struct crss_hw_ring_ctx *status_ring, int timeout)
{
	int num_status_buff = 0;

	do {
		num_status_buff = crss_get_num_status(status_ring);
	} while (!num_status_buff && timeout--);

	return num_status_buff;
}

static inline phys_addr_t crss_va_to_pa(void *va, void *va_base_addr, void *pa_base_addr)
{
	return (phys_addr_t)(pa_base_addr + (va - va_base_addr));
}

size_t crss_sgl_copy_sgl(struct scatterlist *d_sgl, unsigned int d_nents,
	struct scatterlist *s_sgl, unsigned int s_nents, size_t n_bytes)
{
	size_t len;
	size_t offset = 0;
	struct sg_mapping_iter d_iter, s_iter;

	if (!n_bytes)
		return 0;

	sg_miter_start(&s_iter, s_sgl, s_nents, SG_MITER_ATOMIC | SG_MITER_FROM_SG);
	sg_miter_start(&d_iter, d_sgl, d_nents, SG_MITER_ATOMIC | SG_MITER_TO_SG);

	while ((offset < n_bytes) && sg_miter_next(&s_iter) &&
		sg_miter_next(&d_iter)) {

		len = min3(d_iter.length, s_iter.length, n_bytes - offset);
		memcpy(d_iter.addr, s_iter.addr, len);
		offset += len;

		d_iter.consumed = len;
		sg_miter_stop(&d_iter);
		s_iter.consumed = len;
		sg_miter_stop(&s_iter);
	}

	return offset;
}

static inline void crss_request_complete(struct raptor2_crss_engine *engine,
						struct raptor2_crss_req *req, int err)
{
	dma_unmap_sg(engine->dev, req->src, req->mapped_src_ents, DMA_TO_DEVICE);
	dma_unmap_sg(engine->dev, req->dst, req->mapped_dst_ents, DMA_FROM_DEVICE);
	dma_unmap_single(engine->dev, req->hdr_dma_addr, req->hdr_len, DMA_TO_DEVICE);

	req->areq->complete(req->areq, err);
}


static inline int check_return_code(struct raptor2_crss_engine *engine, int ret)
{
	switch (ret) {
	case CRSS_STATUS_ICV_FAIL:
		dev_dbg(engine->dev, "Status ICV Mismatch\n");
		ret = -EBADMSG;
		break;

	case CRSS_STATUS_MEMORY_ERROR:
		dev_warn(engine->dev, "QAU: memory error triggered\n");
		ret = -EFAULT;
		break;

	case CRSS_STATUS_BLOCK_ERROR:
		dev_warn(engine->dev, "QAU: block error triggered\n");
		ret = -EIO;
		break;
	}

	return ret;
}


static irqreturn_t raptor2_crss_irq(int irq, void *dev)
{
	struct crss_queue_pair_ctx *queue = (struct crss_queue_pair_ctx *)dev;
	struct raptor2_crss_engine *engine = queue->engine;
	union qau_event_clear_reg event_clear = {0};

	tasklet_schedule(&queue->status_task);

	if (engine->is_b0) {
		event_clear.bf.sts_ptr_match_evt_clr = 1;
		writel(event_clear.bits, queue->qau_base + CRSS_QAU_EVENT_CLEAR__ADDR);
	}

	return IRQ_HANDLED;
}

static void raptor2_crss_status_task(unsigned long data)
{
	struct crss_queue_pair_ctx *queue = (struct crss_queue_pair_ctx *)data;
	struct raptor2_crss_engine *engine = queue->engine;
	struct crss_hw_ring_ctx	*status_ring;
	int num_status;
	struct crypto_status *status = NULL;
	struct raptor2_crss_req *req;
	unsigned long flags;
	int pkt_idx, err = -EINVAL;

	disable_hardirq(queue->irq_no);

	while (atomic_read(&queue->in_flight)) {
		status_ring = &queue->status_ring_ctx;
		num_status = poll_status_ring(status_ring, RAPTOR2_CRSS_TIMEOUT);
		if (!num_status) {
			dev_err(engine->dev, "QAU Stall: Crypto status timeout\n");
			goto error_status;
		}

		while (num_status--) {

			status = crss_hw_ring_deq_sts_buf(status_ring);
			if (unlikely(!status || (status->pkt_idx >= CRSS_STSQ_RING_SIZE))) {
				err = -EINVAL;
				goto err_skip_check_code;
			}

			pkt_idx = status->pkt_idx;
			err = check_return_code(engine, status->ret_code);

err_skip_check_code:
			spin_lock_irqsave(&queue->lock, flags);
			req = list_first_entry(&queue->in_progress, struct raptor2_crss_req, list);
			list_del(&req->list);

			if (unlikely(!req)) {
				spin_unlock_irqrestore(&queue->lock, flags);
				dev_err(engine->dev, "NULL req at pkt idx = %d\n", pkt_idx);
				goto error_status;
			}

			atomic_dec(&queue->in_flight);
			spin_unlock_irqrestore(&queue->lock, flags);

			crss_request_complete(engine, req, err);

		}

		crss_status_queue_submit(status_ring);
	}

error_status:
	enable_irq(queue->irq_no);
}

static int raptor2_crss_dma_map(struct raptor2_crss_engine *engine, struct raptor2_crss_req *creq)
{
	int src_nents, dst_nents, mapped_src_ents, mapped_dst_ents;
	struct scatterlist *src = creq->src;
	struct scatterlist *dst = creq->dst;
	unsigned int assoclen = creq->assoclen;
	int src_len, dst_len, temp;
	int total_hw_scatters = 0;

	if (creq->hdr_len) {
		creq->hdr_dma_addr = dma_map_single(engine->dev, creq->crss_hdr,
							creq->hdr_len, DMA_TO_DEVICE);

		if (dma_mapping_error(engine->dev, creq->hdr_dma_addr))
			return -ENOMEM;
		total_hw_scatters++;
	}

	src_len = creq->cryptlen + (creq->is_rfc4106 ? 0 : creq->assoclen);

	src_nents = sg_nents_for_len(src, src_len);
	if (src_nents < 0) {
		dev_err(engine->dev, "Invalid numbers of Source SG.\n");
		goto err;
	}


	/* There is a HW bug because of which if AAD_CPY bit is set,
	 * the destination packet is getting corrupted. So, the AAD_CPY
	 * bit is kept 0 and thus AAD will not be copied in the dst buffer
	 * by the Hardware.
	 * Workaround:	Keep the AAD as it is in the destination buffer.
			Hence not including the AAD size in destination.
	 */

	if (creq->is_encrypt)
		dst_len = creq->cryptlen + creq->authsize;
	else
		dst_len = creq->cryptlen - creq->authsize;


	if ((src != dst) && !creq->is_rfc4106) {
		/* Out-of-place transformation
		 * Copy AAD to DST Scatterlist
		 */

		temp = sg_nents_for_len(dst, dst_len + assoclen);
		if (assoclen != crss_sgl_copy_sgl(dst, temp, src, src_nents, assoclen)) {
			dev_err(engine->dev, "Invalid numbers of Destination SG.\n");
			goto err;
		}
	}

	if (!creq->is_rfc4106)
		dst = scatterwalk_ffwd(creq->_dst, dst, assoclen);

	dst_nents = sg_nents_for_len(dst, dst_len);
	if (dst_nents < 0) {
		dev_err(engine->dev, "Invalid numbers of Destination SG.\n");
		goto err;
	}

	mapped_src_ents = dma_map_sg(engine->dev, src, src_nents, DMA_TO_DEVICE);
	if (!mapped_src_ents)
		goto err;

	mapped_dst_ents = dma_map_sg(engine->dev, dst, dst_nents, DMA_FROM_DEVICE);
	if (!mapped_dst_ents)
		goto err_map_sg;

	total_hw_scatters += src_nents + dst_nents;

	creq->dst = dst;
	creq->src_len = src_len;
	creq->dst_len = dst_len;
	creq->src_nents = src_nents;
	creq->dst_nents = dst_nents;
	creq->mapped_src_ents = mapped_src_ents;
	creq->mapped_dst_ents = mapped_dst_ents;
	creq->cmd_size = sizeof(struct crss_command) +
			(total_hw_scatters * sizeof(struct crss_dma_scatter));

	return 0;

err_map_sg:
	dma_unmap_sg(engine->dev, src, src_nents, DMA_TO_DEVICE);

err:
	if (creq->hdr_len)
		dma_unmap_single(engine->dev, creq->hdr_dma_addr, creq->hdr_len, DMA_TO_DEVICE);

	return -ENOMEM;
}

static inline void crss_build_cmd_descriptor(struct raptor2_crss_req *creq,
			uint32_t *ptr, int pkt_idx, bool split, struct crss_hw_ring_ctx *ring)
{
	struct crss_command_0 *c0_ptr = (struct crss_command_0 *)ptr;
	struct crss_command_1 *c1_ptr;
	struct crss_command_2 *c2_ptr;
	struct scatterlist *sg;
	uint32_t src_length = creq->hdr_len + creq->src_len;
	uint32_t dst_length = creq->dst_len;
	uint32_t nbytes = creq->src_len;
	uint64_t val;
	int sg_len;
	int i = 0, j;
	uint16_t key_info = GET_KEY_CMD(creq->new_key, creq->key_idx);
	uint32_t cmd_len_type = 0x60002;
	uint32_t proc_len = creq->cryptlen - (creq->is_encrypt ? 0 : creq->authsize);


	if (likely(split == false)) {
		c0_ptr->pkt_idx = pkt_idx;
		c0_ptr->key_info = key_info;
		c0_ptr->src_buf_len = src_length;


		if (likely(creq->hdr_len)) {
			c0_ptr->src_dma[i].buff_addr_low = lower_32_bits(creq->hdr_dma_addr);
			c0_ptr->src_dma[i].buff_addr_high = upper_32_bits(creq->hdr_dma_addr);
			c0_ptr->src_dma[i].buff_len = creq->hdr_len;
			i = 1;
		}

		for_each_sg(creq->src, sg, creq->mapped_src_ents, j) {
			c0_ptr->src_dma[i + j].buff_addr_low = lower_32_bits(sg_dma_address(sg));
			c0_ptr->src_dma[i + j].buff_addr_high = upper_32_bits(sg_dma_address(sg));
			sg_len = sg_dma_len(sg);
			c0_ptr->src_dma[i + j].buff_len = (nbytes < sg_len) ? nbytes : sg_len;
			nbytes -= sg_len;
		}

		c1_ptr = get_next_cmd_ptr((char *)c0_ptr, sizeof(*c0_ptr),
							creq->mapped_src_ents + i);

		c1_ptr->dst_buf_len = dst_length;

		nbytes = creq->dst_len;
		for_each_sg(creq->dst, sg, creq->mapped_dst_ents, j) {
			c1_ptr->dst_dma[j].buff_addr_low = lower_32_bits(sg_dma_address(sg));
			c1_ptr->dst_dma[j].buff_addr_high = upper_32_bits(sg_dma_address(sg));
			sg_len = sg_dma_len(sg);
			c1_ptr->dst_dma[j].buff_len = (nbytes < sg_len) ? nbytes : sg_len;
			nbytes -= sg_len;
		}

		c2_ptr = get_next_cmd_ptr((char *)c1_ptr, sizeof(*c1_ptr), creq->mapped_dst_ents);

		c2_ptr->cmd_len_type = cmd_len_type;
		c2_ptr->cmd_ctrl = creq->ctrl;
		c2_ptr->proc_len = proc_len;
		c2_ptr->iv_length = creq->ivsize;
		c2_ptr->assoclen = creq->assoclen;
		c2_ptr->icv_len = creq->authsize;
		c2_ptr->key_size = creq->key_size;


	} else {
		/* cmd_size bytes are not available at the end of the CMD queue.
		 * Command descriptor is to be split whereever the roll-over occurs
		 */

		val = U_WORD(src_length) | L_WORD(U_HALF_WORD(key_info) | L_HALF_WORD(pkt_idx));
		ptr = cmd_split_write(ring, ptr, val);

		if (likely(creq->hdr_len)) {
			val = ((uint64_t)((creq->hdr_len) & 0xFFFF) << 48) | creq->hdr_dma_addr;
			ptr = cmd_split_write(ring, ptr, val);
		}

		for_each_sg(creq->src, sg, creq->mapped_src_ents, j) {
			sg_len = (nbytes < sg_dma_len(sg)) ? nbytes : sg_dma_len(sg);
			val = ((uint64_t)(sg_len & 0xFFFF) << 48) | sg_dma_address(sg);
			ptr = cmd_split_write(ring, ptr, val);
			nbytes -= sg_len;
		}

		RAPTOR2_WRITEL(dst_length, check_roll_over(ptr, ring));
		ptr = check_roll_over(ptr + 1, ring);

		nbytes = creq->dst_len;
		for_each_sg(creq->dst, sg, creq->mapped_dst_ents, j) {
			sg_len = (nbytes < sg_dma_len(sg)) ? nbytes : sg_dma_len(sg);
			val = ((uint64_t)(sg_len & 0xFFFF) << 48) | sg_dma_address(sg);
			ptr = cmd_split_write(ring, ptr, val);
			nbytes -= sg_len;
		}

		ptr = cmd_split_write(ring, ptr, U_WORD(creq->ctrl) | L_WORD(cmd_len_type));
		ptr = cmd_split_write(ring, ptr, U_WORD(creq->ivsize) | L_WORD(proc_len));
		ptr = cmd_split_write(ring, ptr, U_WORD(creq->authsize) | L_WORD(creq->assoclen));

		RAPTOR2_WRITEL(creq->key_size, check_roll_over(ptr, ring));
	}
}

static int crss_submit_request(struct raptor2_crss_engine *engine,
				struct raptor2_crss_req *crss_req, int queue_idx)
{
	struct crss_queue_pair_ctx *crss_queue;
	struct crss_hw_ring_ctx *cmdq_ring_ctxt;
	unsigned long flags;
	int pkt_idx;
	uint32_t *write_ptr;
	bool split_cmd = false;
	struct device	*dev = engine->dev;

	crss_queue = &engine->queue_pair_ctx[queue_idx];
	cmdq_ring_ctxt = &crss_queue->command_ring_ctx;

	spin_lock_irqsave(&crss_queue->cmdq_lock, flags);

	write_ptr = crss_cmd_queue_alloc(cmdq_ring_ctxt, crss_req->cmd_size);
	if (!write_ptr) {
		spin_unlock_irqrestore(&crss_queue->cmdq_lock, flags);
		dev_err(dev, "Command Buffer Allocation failed\n");
		goto err_cmd_alloc;
	}

	pkt_idx = get_packet_index(crss_queue);

	/* Check to see if the command descriptor can be written
	 * into the queue's remaining last elements without
	 * rolling over. In case of rollover, the descriptor has
	 * to be split.
	 */

	if (unlikely(bytes_at_tail(cmdq_ring_ctxt, write_ptr) < crss_req->cmd_size))
		split_cmd = true;

	crss_build_cmd_descriptor(crss_req, write_ptr, pkt_idx, split_cmd, cmdq_ring_ctxt);

	spin_lock_irqsave(&crss_queue->lock, flags);
	list_add_tail(&crss_req->list, &crss_queue->in_progress);
	atomic_inc(&crss_queue->in_flight);
	spin_unlock_irqrestore(&crss_queue->lock, flags);

	crss_cmd_queue_submit(cmdq_ring_ctxt);

	spin_unlock_irqrestore(&crss_queue->cmdq_lock, flags);

	return -EINPROGRESS;

err_cmd_alloc:
	dma_unmap_sg(dev, crss_req->src, crss_req->mapped_src_ents, DMA_TO_DEVICE);
	dma_unmap_sg(dev, crss_req->dst, crss_req->mapped_dst_ents, DMA_FROM_DEVICE);
	dma_unmap_single(dev, crss_req->hdr_dma_addr, crss_req->hdr_len, DMA_TO_DEVICE);

	return -ENOMEM;
}


static int raptor2_crss_setkey(struct raptor2_crss_generic_ctx *ctx, const u8 *key, uint32_t len)
{
	struct raptor2_crss_engine *engine = ctx->engine;
	struct crss_key_table *key_table = NULL;

	ctx->new_key = 1;
	key_table = engine->memss_keytbl_base_addr;
	memcpy_toio(key_table[ctx->key_idx].cipher_key, key, len);

	return 0;
}

static int raptor2_crss_gcm_crypt(struct aead_request *req, bool is_encrypt)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	struct raptor2_crss_req *crss_req = aead_request_ctx(req);
	struct raptor2_crss_aead *crss_aead_alg = to_raptor2_crss_aead(crypto_aead_alg(aead));
	int ret;

	((__le32 *)crss_req->crss_hdr)[3] = cpu_to_be32(1);

	crss_req->areq = &req->base;
	crss_req->new_key = ctx->generic.new_key;
	crss_req->key_idx = ctx->generic.key_idx;
	crss_req->is_encrypt = is_encrypt;
	crss_req->ivsize = AES_BLOCK_SIZE;
	crss_req->cryptlen = req->cryptlen;
	crss_req->authsize = ctx->authsize;
	crss_req->key_size = ctx->cipher_key_len;
	crss_req->hdr_len += crss_req->ivsize;

	crss_req->ctrl = crss_aead_alg->ctrl_default |
			(is_encrypt << CRSS_CTRL_ENCRYPT_IDX) | (1 << CRSS_CTRL_MSG_BEGIN) |
			(1 << CRSS_CTRL_MSG_END) | (1 << CRSS_CTRL_ICV_APPEND);

	ret = raptor2_crss_dma_map(ctx->generic.engine, crss_req);
	if (ret)
		return ret;

	ctx->generic.new_key = 0;

	return crss_submit_request(ctx->generic.engine, crss_req, ctx->generic.curr_q);
}


static int raptor2_crss_aes_crypt(struct skcipher_request *req, bool is_encrypt)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct raptor2_crss_ablk_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct raptor2_crss_req *crss_req = skcipher_request_ctx(req);
	struct raptor2_crss_alg *crss_alg = to_raptor2_crss_skcipher(crypto_skcipher_alg(tfm));
	int ret;

	crss_req->areq = &req->base;
	crss_req->src = req->src;
	crss_req->dst = req->dst;
	crss_req->new_key = ctx->generic.new_key;
	crss_req->key_idx = ctx->generic.key_idx;
	crss_req->is_encrypt = is_encrypt;
	crss_req->ivsize = crypto_skcipher_ivsize(tfm);
	crss_req->cryptlen = req->cryptlen;
	crss_req->assoclen = 0;
	crss_req->authsize = 0;
	crss_req->key_size = ctx->key_len;
	crss_req->is_rfc4106 = 0;
	crss_req->hdr_len = crss_req->ivsize;

	/* The IV we are handed may be allocated from the stack so
	 * we must copy it to a DMAable buffer before use.
	 */
	memcpy(crss_req->crss_hdr, req->iv, crss_req->ivsize);

	crss_req->ctrl = crss_alg->ctrl_default | (is_encrypt << CRSS_CTRL_ENCRYPT_IDX) |
				(1 << CRSS_CTRL_MSG_BEGIN) | (1 << CRSS_CTRL_MSG_END);

	ret = raptor2_crss_dma_map(ctx->generic.engine, crss_req);
	if (ret)
		return ret;

	ctx->generic.new_key = 0;

	return crss_submit_request(ctx->generic.engine, crss_req, ctx->generic.curr_q);
}


static inline int crss_rfc4106_set_sg_lists(struct aead_request *areq)
{
	struct raptor2_crss_req *crss_req = aead_request_ctx(areq);
	int assoclen = areq->assoclen;
	int sg_nents = sg_nents_for_len(areq->dst, areq->cryptlen + assoclen);
	int ret;

	if (assoclen != 16 && assoclen != 20)
		return -EINVAL;

	if (areq->src != areq->dst) {
		/* Out-of-place transformation : Copy AAD to DST Scatterlist */
		ret = crss_sgl_copy_sgl(areq->dst, sg_nents, areq->src, sg_nents, assoclen);
		if (assoclen != ret)
			return -EINVAL;
	}

	assoclen -= GCM_RFC4106_IV_SIZE;

	scatterwalk_map_and_copy(&crss_req->crss_hdr[AES_BLOCK_SIZE], areq->src, 0, assoclen, 0);

	/* Jump to offset. */
	crss_req->src = scatterwalk_ffwd(crss_req->rfc_src, areq->src, areq->assoclen);
	crss_req->dst = ((areq->src == areq->dst) ? crss_req->src :
	       scatterwalk_ffwd(crss_req->rfc_dst, areq->dst, areq->assoclen));

	crss_req->is_rfc4106 = 1;
	crss_req->assoclen = assoclen;
	crss_req->hdr_len = crss_req->assoclen;

	return 0;

}

int raptor2_crss_rfc4106_encrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct raptor2_crss_req *crss_req = aead_request_ctx(req);

	if (crss_rfc4106_set_sg_lists(req))
		return -EINVAL;

	memcpy(crss_req->crss_hdr, ctx->salt, GCM_RFC4106_SALT_SIZE);
	memcpy(&crss_req->crss_hdr[GCM_RFC4106_SALT_SIZE], req->iv, ivsize);
	return raptor2_crss_gcm_crypt(req, true);
}

int raptor2_crss_rfc4106_decrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct raptor2_crss_req *crss_req = aead_request_ctx(req);

	if (crss_rfc4106_set_sg_lists(req))
		return -EINVAL;

	memcpy(crss_req->crss_hdr, ctx->salt, GCM_RFC4106_SALT_SIZE);
	memcpy(&crss_req->crss_hdr[GCM_RFC4106_SALT_SIZE], req->iv, ivsize);
	return raptor2_crss_gcm_crypt(req, false);
}


int raptor2_crss_rfc4106_setkey(struct crypto_aead *aead, const u8 *key, unsigned int keylen)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	int err;

	err = aes_check_keylen(keylen - GCM_RFC4106_SALT_SIZE);
	if (err)
		return err;

	keylen -= GCM_RFC4106_SALT_SIZE;

	ctx->cipher_key_len = keylen;

	memcpy(ctx->salt, key + keylen, GCM_RFC4106_SALT_SIZE);

	raptor2_crss_setkey(&ctx->generic, key, keylen);
	return 0;
}

int raptor2_crss_rfc4106_setauthsize(struct crypto_aead *aead, unsigned int authsize)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);

	ctx->authsize = authsize;
	return crypto_rfc4106_check_authsize(authsize);
}

int raptor2_crss_gcm_encrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct raptor2_crss_req *crss_req = aead_request_ctx(req);

	crss_req->src = req->src;
	crss_req->dst = req->dst;
	crss_req->assoclen = req->assoclen;
	crss_req->hdr_len = 0;
	crss_req->is_rfc4106 = 0;

	memcpy(crss_req->crss_hdr, req->iv, ivsize);
	return raptor2_crss_gcm_crypt(req, true);
}

int raptor2_crss_gcm_decrypt(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct raptor2_crss_req *crss_req = aead_request_ctx(req);

	crss_req->src = req->src;
	crss_req->dst = req->dst;
	crss_req->assoclen = req->assoclen;
	crss_req->hdr_len = 0;
	crss_req->is_rfc4106 = 0;

	memcpy(crss_req->crss_hdr, req->iv, ivsize);
	return raptor2_crss_gcm_crypt(req, false);
}


int raptor2_crss_gcm_setkey(struct crypto_aead *aead, const u8 *key, unsigned int keylen)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	int err;

	err = aes_check_keylen(keylen);
	if (err)
		return err;

	ctx->cipher_key_len = keylen;

	raptor2_crss_setkey(&ctx->generic, key, keylen);
	return 0;
}

int raptor2_crss_gcm_setauthsize(struct crypto_aead *aead, unsigned int authsize)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);

	ctx->authsize = authsize;
	return crypto_gcm_check_authsize(authsize);
}

int raptor2_crss_aead_cra_init(struct crypto_aead *aead)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct raptor2_crss_aead *crss_aead_alg = to_raptor2_crss_aead(alg);

	crypto_aead_set_reqsize(aead, sizeof(struct raptor2_crss_req));

	return raptor2_crss_ctx_init(crss_aead_alg->engine, &ctx->generic);
}


void raptor2_crss_aead_cra_exit(struct crypto_aead *tfm)
{
	struct raptor2_crss_aead_ctx *ctx = crypto_aead_ctx(tfm);
	struct raptor2_crss_engine *engine = ctx->generic.engine;

	release_crss_queue(engine, ctx->generic.curr_q);
	release_crss_key_idx(engine, ctx->generic.key_idx);
	ctx->generic.new_key = 0;
}

int raptor2_crss_ablk_encrypt(struct skcipher_request *req)
{
	return raptor2_crss_aes_crypt(req, true);
}

int raptor2_crss_ablk_decrypt(struct skcipher_request *req)
{
	return raptor2_crss_aes_crypt(req, false);
}


int raptor2_crss_aes_setkey(struct crypto_skcipher *cipher, const u8 *key, unsigned int len)
{
	struct raptor2_crss_ablk_ctx *ctx = crypto_skcipher_ctx(cipher);

	if (len != AES_KEYSIZE_256 && len != AES_KEYSIZE_192 && len != AES_KEYSIZE_128)
		return -EINVAL;

	ctx->key_len = len;

	raptor2_crss_setkey(&ctx->generic, key, len);
	return 0;
}

int raptor2_crss_ablk_init_tfm(struct crypto_skcipher *tfm)
{
	struct raptor2_crss_ablk_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);
	struct raptor2_crss_alg *crss_alg = to_raptor2_crss_skcipher(alg);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct raptor2_crss_req));

	return raptor2_crss_ctx_init(crss_alg->engine, &ctx->generic);
}

void raptor2_crss_ablk_exit_tfm(struct crypto_skcipher *tfm)
{
	struct raptor2_crss_ablk_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct raptor2_crss_engine *engine = ctx->generic.engine;

	ctx->generic.new_key = 0;
	release_crss_queue(engine, ctx->generic.curr_q);
	release_crss_key_idx(engine, ctx->generic.key_idx);
}

static void raptor2_crss_qau_hw_init(struct raptor2_crss_engine *engine, int  cmdq_id)
{
	union qau_cmd_rdr_cfg_1_reg cmd_rdr_cfg_1;
	union qau_cmd_rdr_cfg_2_reg cmd_rdr_cfg_2;
	union qau_cmd_rdr_cfg_3_reg cmd_rdr_cfg_3;
	union qau_cmd_rdr_cfg_4_reg cmd_rdr_cfg_4;
	union qau_sts_wtr_cfg_1_reg sts_wtr_cfg_1;
	union qau_sts_wtr_cfg_2_reg sts_wtr_cfg_2;
	union qau_crypto_sts_len_reg crypto_sts_len;
	union qau_sts_wtr_cfg_3_reg sts_wtr_cfg_3;
	union qau_sts_wtr_cfg_4_reg sts_wtr_cfg_4;
	union qau_trigger_reg qau_trigger;
	union qau_intr_cfg_reg intr_cfg;

	struct crss_queue_pair_ctx *crss_queue  = NULL;
	phys_addr_t         cmdq_addr = 0;
	phys_addr_t         stsq_addr = 0;

	void __iomem	*pool_addr_va = engine->q_pool_addr;
	phys_addr_t	pool_addr_pa = engine->q_pool_addr_pa;

	crss_queue = &engine->queue_pair_ctx[cmdq_id];

	cmdq_addr = crss_va_to_pa(crss_queue->cmdq_base_addr, pool_addr_va, (void *)pool_addr_pa);
	stsq_addr = crss_va_to_pa(crss_queue->stsq_base_addr, pool_addr_va, (void *)pool_addr_pa);

	cmd_rdr_cfg_1.bits = 0;
	cmd_rdr_cfg_1.bf.outs_en = 1;
	cmd_rdr_cfg_1.bf.mode = STORAGE_MODE_LINEAR_CIRC_BUFFER;
	cmd_rdr_cfg_1.bf.elem_size = CRSS_ELEMT_SIZE; /* in bits */
	cmd_rdr_cfg_1.bf.timer_en = 1;
	writel(cmd_rdr_cfg_1.bits, crss_queue->qau_base + CRSS_QAU_CMD_RDR_CFG_1__ADDR);

	cmd_rdr_cfg_2.bits = 0;
	/* size of buffer in units of "elem_size" */
	cmd_rdr_cfg_2.bf.buff_size = (uint16_t)((CRSS_CMDQ_SIZE * 8) / cmd_rdr_cfg_1.bf.elem_size);
	writel(cmd_rdr_cfg_2.bits, crss_queue->qau_base + CRSS_QAU_CMD_RDR_CFG_2__ADDR);

	cmd_rdr_cfg_3.bits = 0;
	cmd_rdr_cfg_3.bf.base_addr_lo = (uint32_t) cmdq_addr & 0xFFFFFFFF;

	/* lower 32 bits of the start address of the queue */
	writel(cmd_rdr_cfg_3.bits, crss_queue->qau_base + CRSS_QAU_CMD_RDR_CFG_3__ADDR);

	cmd_rdr_cfg_4.bits = 0;
	cmd_rdr_cfg_4.bf.base_addr_hi = (cmdq_addr >> 0x20) & 0xFF;
	cmd_rdr_cfg_4.bf.timer_config = 0x200;

	/* higher 4 bits of the start address of the queue */
	writel(cmd_rdr_cfg_4.bits, crss_queue->qau_base + CRSS_QAU_CMD_RDR_CFG_4__ADDR);

	sts_wtr_cfg_1.bits = 0;
	sts_wtr_cfg_1.bf.outs_en = 1;
	sts_wtr_cfg_1.bf.mode = STORAGE_MODE_LINEAR_CIRC_BUFFER;
	sts_wtr_cfg_1.bf.elem_size = CRSS_ELEMT_SIZE;
	sts_wtr_cfg_1.bf.timer_en = 1;
	writel(sts_wtr_cfg_1.bits, crss_queue->qau_base + CRSS_QAU_STS_WTR_CFG_1__ADDR);

	sts_wtr_cfg_2.bits = 0;
	/* Size of buffer in units of "elem_size". elemt_size is in bits */
	sts_wtr_cfg_2.bf.buff_size = (uint16_t)((CRSS_STSQ_SIZE * 8) / sts_wtr_cfg_1.bf.elem_size);
	writel(sts_wtr_cfg_2.bits, crss_queue->qau_base + CRSS_QAU_STS_WTR_CFG_2__ADDR);

	crypto_sts_len.bits = 0;
	crypto_sts_len.bf.len = (sizeof(struct crypto_status) / 4);
	writel(crypto_sts_len.bits, crss_queue->qau_base + CRSS_QAU_CRYPTO_STS_LEN__ADDR);

	sts_wtr_cfg_3.bits = 0;
	sts_wtr_cfg_3.bf.base_addr_lo = (uint32_t) stsq_addr & 0xFFFFFFFF;

	/* lower 32 bits of the start address of the queue */
	writel(sts_wtr_cfg_3.bits, crss_queue->qau_base + CRSS_QAU_STS_WTR_CFG_3__ADDR);

	sts_wtr_cfg_4.bits = 0;
	sts_wtr_cfg_4.bf.base_addr_hi = (stsq_addr >> 0x20) & 0xFF;
	sts_wtr_cfg_4.bf.timer_config = 0x200;

	/* higher 4 bits of the start address of the queue */
	writel(sts_wtr_cfg_4.bits, crss_queue->qau_base + CRSS_QAU_STS_WTR_CFG_4__ADDR);

	if (engine->is_b0) {
		/* Setting Match Pointer to 0 (CRSS_STSQ_ELEM_SIZE)*/
		writel(0, crss_queue->qau_base + CRSS_QAU_STS_PTR_MATCH_INT__ADDR);

		intr_cfg.bits = 0;
		intr_cfg.bf.en_sts_ptr_match_int = 1;
		writel(intr_cfg.bits, crss_queue->qau_base + CRSS_QAU_INTR_CFG__ADDR);
	}

	qau_trigger.bits = 0;
	qau_trigger.bf.inp_cmd_rd = 1;
	qau_trigger.bf.out_sts_wr = 1;
	writel(qau_trigger.bits, crss_queue->qau_base + CRSS_QAU_TRIGGER__ADDR);

	dev_info(engine->dev, "Queue%d registered\n", cmdq_id);
}


static int raptor2_crss_prepare_qau_ctx(struct raptor2_crss_engine *engine)
{
	uint8_t cmdq_id;
	struct crss_queue_pair_ctx *queue;
	struct crss_hw_ring_ctx *cmdq_ring;
	struct crss_hw_ring_ctx *stsq_ring;
	struct resource *irq;
	struct device *dev = engine->dev;
	struct platform_device *pdev = to_platform_device(dev);

	for (cmdq_id = 0; cmdq_id < engine->num_queues; cmdq_id++) {
		queue = &engine->queue_pair_ctx[cmdq_id];
		queue->engine = engine;
		queue->cmdq_base_addr = engine->q_pool_addr + (cmdq_id * CRSS_CMDQ_STSQ_SIZE);
		queue->stsq_base_addr = queue->cmdq_base_addr + CRSS_CMDQ_SIZE;
		queue->qau_base = GET_QAU_BASEADDR(engine->regs, cmdq_id);
		queue->current_pkt_idx = 0;
		queue->queue_num = cmdq_id;
		atomic_set(&queue->in_flight, 0);
		spin_lock_init(&queue->cmdq_lock);
		spin_lock_init(&queue->lock);
		INIT_LIST_HEAD(&queue->in_progress);
		tasklet_init(&queue->status_task, raptor2_crss_status_task, (unsigned long)queue);

		irq = platform_get_resource(pdev, IORESOURCE_IRQ, cmdq_id);
		if (!irq) {
			dev_err(dev, "no memory/irq resource for crss queue %d\n", cmdq_id);
			return -ENXIO;
		}

		queue->irq_no = irq->start;

		if (devm_request_irq(dev, irq->start, raptor2_crss_irq, 0, engine->name, queue)) {
			dev_err(dev, "failed to request IRQ for crss queue %d\n", cmdq_id);
			return -EBUSY;
		}

		cmdq_ring   = &queue->command_ring_ctx;
		cmdq_ring->ring_id         = cmdq_id;
		cmdq_ring->base_addr  = (uint8_t *)queue->cmdq_base_addr;
		cmdq_ring->hw_write_addr   = queue->qau_base + CRSS_QAU_CMD_RDR_CFG_5__ADDR;
		cmdq_ring->hw_read_addr    = queue->qau_base + CRSS_QAU_CMD_RDR_RD_PTR__ADDR;
		cmdq_ring->elem_size       = CRSS_CMDQ_ELEM_SIZE;
		cmdq_ring->size            = CRSS_CMDQ_RING_SIZE;
		cmdq_ring->type            = QUEUE_TYPE_COMMAND;
		cmdq_ring->end_addr  = (uint8_t *)(cmdq_ring->base_addr + CRSS_CMDQ_SIZE);

		stsq_ring  = &queue->status_ring_ctx;
		stsq_ring->ring_id        = cmdq_id;
		stsq_ring->base_addr = (uint8_t *)queue->stsq_base_addr;
		stsq_ring->hw_write_addr  = queue->qau_base + CRSS_QAU_STS_WTR_RD_PTR__ADDR;
		stsq_ring->hw_read_addr   = queue->qau_base + CRSS_QAU_STS_WTR_CFG_5__ADDR;
		stsq_ring->elem_size      = CRSS_STSQ_ELEM_SIZE;
		stsq_ring->size           = CRSS_STSQ_RING_SIZE;
		stsq_ring->type           = QUEUE_TYPE_STATUS;
		stsq_ring->end_addr  = (uint8_t *)(stsq_ring->base_addr + CRSS_STSQ_SIZE);

		raptor2_crss_qau_hw_init(engine, cmdq_id);
	}

	return 0;
}

static int raptor2_crss_register_alg(struct raptor2_crss_engine *engine)
{
	unsigned int i, k;
	int err = 0;

	for (i = 0; i < engine->num_algs; ++i) {
		engine->algs[i].engine = engine;
		err = crypto_register_skcipher(&engine->algs[i].alg);

		if (err) {
			dev_err(engine->dev, "failed to register alg \"%s\"\n",
							engine->algs[i].alg.base.cra_name);
			goto err_cipher_algs;
		}
	}

	for (i = 0; i < engine->num_aeads; ++i) {
		engine->aeads[i].engine = engine;
		err = crypto_register_aead(&engine->aeads[i].alg);

		if (err) {
			dev_err(engine->dev, "failed to register alg \"%s\"\n",
							engine->aeads[i].alg.base.cra_name);
			goto err_aead_algs;
		}
	}

	return 0;

err_aead_algs:
	for (k = 0; k < i; k++)
		crypto_unregister_aead(&engine->aeads[k].alg);

	i = engine->num_algs;

err_cipher_algs:
	for (k = 0; k < i; k++)
		crypto_unregister_skcipher(&engine->algs[k].alg);

	return err;
}

static void raptor2_crss_unregister_alg(struct raptor2_crss_engine *engine)
{
	unsigned int i;

	for (i = 0; i < engine->num_algs; ++i)
		crypto_unregister_skcipher(&engine->algs[i].alg);

	for (i = 0; i < engine->num_aeads; ++i)
		crypto_unregister_aead(&engine->aeads[i].alg);
}


static int raptor2_crss_probe(struct platform_device *pdev)
{
	int err;
	struct resource res;
	struct device_node *np = pdev->dev.of_node;
	struct crss_queue_pair_ctx *queue_pair_ctx;
	dma_addr_t tmp = 0;
	struct device_node *memnp;
	int size;

	struct raptor2_crss_engine *engine = devm_kzalloc(&pdev->dev, sizeof(*engine), GFP_KERNEL);

	if (of_property_read_u32(np, "num-queues", &engine->num_queues)) {
		dev_err(&pdev->dev, "could not find num-queues\n");
		return  -ENXIO;
	}

	queue_pair_ctx = devm_kzalloc(&pdev->dev, engine->num_queues * sizeof(*queue_pair_ctx),
			   GFP_KERNEL);

	if (!engine || !queue_pair_ctx)
		return -ENOMEM;

	if (of_device_is_compatible(np, "edgeq,crss")) {
		engine->queue_pair_ctx	= queue_pair_ctx;
		engine->algs		= crss_engine_algs;
		engine->num_algs	= raptor2_crss_num_algs;
		engine->aeads		= crss_engine_aeads;
		engine->num_aeads	= raptor2_crss_num_aeads;
		engine->dev		= &pdev->dev;
		engine->name		= dev_name(&pdev->dev);
		engine->curr_queue_idx	= 0;
	} else {

		return -EINVAL;
	}

	engine->regs = devm_platform_ioremap_resource_byname(pdev, "qua_reg");
	if (IS_ERR(engine->regs))
		return PTR_ERR(engine->regs);

	engine->memss_keytbl_base_addr       = devm_platform_ioremap_resource_byname(pdev, "memss");
	if (IS_ERR(engine->memss_keytbl_base_addr))
		return PTR_ERR(engine->memss_keytbl_base_addr);

	/* CRSS is Coherent in B0 */
	engine->is_b0 = of_property_read_bool(np, "dma-coherent");

	if (engine->is_b0) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
		if (err) {
			dev_err(&pdev->dev, "Failed to set DMA 64Bit mask\n");
			return err;
		}

		if (of_property_read_u32(np, "cmdq-pool-size", &size)) {
			dev_err(&pdev->dev, "could not find cmdq-pool-size\n");
			return  -ENXIO;
		}

		engine->q_pool_addr = dma_alloc_coherent(&pdev->dev, size, &tmp, GFP_KERNEL);
		engine->q_pool_addr_pa = tmp;
		engine->q_pool_size = size;

	} else {

		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
		if (err) {
			dev_err(&pdev->dev, "Failed to set DMA 64Bit mask\n");
			return err;
		}

		memnp = of_parse_phandle(np, "memory-region", 0);
		if (!memnp) {
			dev_err(&pdev->dev, "No %s specified\n", "memory-region");
			return -ENXIO;
		}

		err = of_address_to_resource(memnp, 0, &res);
		if (err) {
			dev_err(&pdev->dev, "No memory address assigned to the region\n");
			return err;
		}

		engine->q_pool_addr_pa = res.start;
		engine->q_pool_addr   = devm_ioremap_resource_wc(&pdev->dev, &res);
	}

	if (IS_ERR(engine->q_pool_addr))
		return PTR_ERR(engine->q_pool_addr);

	err = raptor2_crss_prepare_qau_ctx(engine);
	if (err)
		return err;

	crss_key_table_init(engine);

	err = raptor2_crss_register_alg(engine);
	if (err)
		return err;

	platform_set_drvdata(pdev, engine);

	dev_info(&pdev->dev, "***** %s engine registered\n", engine->name);

	return 0;
}

static int raptor2_crss_remove(struct platform_device *pdev)
{
	int i;
	struct raptor2_crss_engine *engine = platform_get_drvdata(pdev);

	raptor2_crss_unregister_alg(engine);
	for (i = 0; i < engine->num_queues; i++)
		tasklet_kill(&engine->queue_pair_ctx[i].status_task);

	if (engine->is_b0)
		dma_free_coherent(&pdev->dev, engine->q_pool_size,
			engine->q_pool_addr, engine->q_pool_addr_pa);

	dev_info(&pdev->dev, "***** engine removed");
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id crss_of_id_table[] = {
	{ .compatible = "edgeq,crss" },
	{}
};
MODULE_DEVICE_TABLE(of, crss_of_id_table);
#endif /* CONFIG_OF */


static struct platform_driver raptor2_crss_driver = {
	.probe  = raptor2_crss_probe,
	.remove = raptor2_crss_remove,
	.driver = {
	.name = "edgeq,crss",
	.of_match_table = of_match_ptr(crss_of_id_table),
	},
};


module_platform_driver(raptor2_crss_driver);

MODULE_AUTHOR("Abdul Ahad <abdul.ahad@edgeq.io>");
MODULE_DESCRIPTION("Support for EdgeQ's cryptographic engine");
MODULE_LICENSE("GPL");
