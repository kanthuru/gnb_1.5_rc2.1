// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Raptor2 Kernel Driver for BSS CMD Q
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Pravin Bathija <bathija@edgeq.io>
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <soc/edgeq/raptor2_bss_api.h>
#include "../include/raptor2_bss_su.h"
#include "raptor2_bss_cmdq.h"

/* phys addresses of shm chunks */
static uint64_t chbase_addr, chbase_sz;		/* channel base */
static uint64_t reset_addr, reset_sz;		/* efuse/reset chan */
static uint64_t suinfra_addr, suinfra_sz;	/* su infra chan */
static uint64_t suimg_addr, suimg_sz;		/* su img loading */
static uint32_t suimg_addr_bss;

static void *pchbase;
static void *psuimg;
static void *pchtbl[BSSRT_CHAN_MAX];	/* virt addr of ch list as per enum seq */

static spinlock_t chlock[BSSRT_CHAN_MAX];

static const struct of_device_id raptor2_bsscmdq_of_match[] = {
	{
		.compatible = "edgeq,raptor2-bsscmdq",
	},
	{ /* sentinel */ }
};

static inline unsigned long claim_chan_lock(bss_chan_type_t ch)
{
	unsigned long flags;

	spin_lock_irqsave(&(chlock[ch]), flags);
	return flags;
}

static inline void release_chan_lock(bss_chan_type_t ch, unsigned long flags)
{
	spin_unlock_irqrestore(&(chlock[ch]), flags);
}

static uint32_t get_checksum(MSG_BSS *msg_ptr)
{
	uint32_t checksum, sum = 0;
	int id;

	sum += msg_ptr->rsv[0];
	sum += msg_ptr->rsv[1];
	sum += msg_ptr->id;
	sum += msg_ptr->type;
	sum += msg_ptr->info;
	sum += msg_ptr->size;

	for (id = 0; id < BSS_MSG_DATA_MAX_SIZE; id++)
		sum += msg_ptr->data[id];

	checksum = ~sum;

	return checksum;
}

static int send_msg(bss_chan_type_t ch, uint8_t msg_id, uint8_t msg_type, uint16_t msg_info,
		u8 *buf, u32 buf_size)
{
	MSG_BSS msg;
	uint32_t chk;
	MSG_BSS *tx_msg;
	uint64_t *cmd_msg_ready, *resp_msg_ready;

	if ((buf_size > 0) && (buf == NULL)) {
		pr_err("%s: buf is invalid\n", __func__);
		return -EINVAL;
	}

	tx_msg = (MSG_BSS *)(pchtbl[ch] + BSS_CMD_MSG_OFFSET);
	cmd_msg_ready = (uint64_t *)(pchtbl[ch] + BSS_CMD_MSG_RDY_OFFSET);
	resp_msg_ready = (uint64_t *)(pchtbl[ch] + BSS_RSP_MSG_RDY_OFFSET);

	memset(&msg, 0, sizeof(MSG_BSS));
	msg.rsv[0] = 0;
	msg.rsv[1] = 0;
	msg.id = msg_id;
	msg.type = msg_type;
	msg.info = msg_info;

	if ((buf != NULL) && (buf_size > 0)) {
		memcpy(msg.data, buf, buf_size);
		msg.size = buf_size;
	}

	chk = get_checksum(&msg);
	msg.checksum = chk;
	memcpy_toio((void *)tx_msg, (void *)&msg, sizeof(MSG_BSS));
	__iowmb();

	*cmd_msg_ready = (uint64_t)CMD_MSG_READY_MAGIC_NUM;

	/* clear previous resp ready message */
	*resp_msg_ready = 0x0;

	return 0;
}

static int get_response_msg(bss_chan_type_t ch, MSG_BSS *buf, uint32_t timeout)
{
	u8 cmd;
	u32 msg_status;
	MSG_BSS *rx_msg;
	uint32_t iter, max_retry;
	uint64_t *resp_msg_ready;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	rx_msg = (MSG_BSS *)(pchtbl[ch] + BSS_RSP_MSG_OFFSET);
	resp_msg_ready = (uint64_t *)(pchtbl[ch] + BSS_RSP_MSG_RDY_OFFSET);

	max_retry = timeout / MSG_WAIT_TIME;
	if (max_retry == 0)
		max_retry = 1; /* wait for minimum timeout value */

	if ((timeout % MSG_WAIT_TIME) != 0)
		max_retry++;

	for (iter = 0; iter <  max_retry; iter++) {
		msg_status = (u32)(*resp_msg_ready);
		if (msg_status ==  RESP_MSG_READY_MAGIC_NUM)
			break;

		udelay(MSG_WAIT_TIME);
	}

	cmd = rx_msg->id;
	if (iter >= max_retry) {
		pr_err("%s: Cmd timed out for %s\n", __func__, cmdstr[cmd]);
		return -ETIMEDOUT;
	}

	buf->id = rx_msg->id;
	buf->type = rx_msg->type;
	buf->info = rx_msg->info;
	buf->size = rx_msg->size;
	buf->checksum = rx_msg->checksum;

	if (buf->size > 0)
		memcpy_fromio((void *)buf->data, (void *)rx_msg->data, buf->size);

	return 0;
}

static void bss_clr_shm(bss_chan_type_t ch)
{
	uint64_t *cmd_msg_ready, *resp_msg_ready;
	uint8_t *cmd_msg_addr, *resp_msg_addr;

	cmd_msg_ready = (uint64_t *)(pchtbl[ch] + BSS_CMD_MSG_RDY_OFFSET);
	resp_msg_ready = (uint64_t *)(pchtbl[ch] + BSS_RSP_MSG_RDY_OFFSET);
	*cmd_msg_ready = 0;
	*resp_msg_ready = 0;

	cmd_msg_addr = pchtbl[ch] + BSS_CMD_MSG_OFFSET;
	resp_msg_addr = pchtbl[ch] + BSS_RSP_MSG_OFFSET;
	memset_io((void *)cmd_msg_addr, 0, sizeof(MSG_BSS));
	memset_io((void *)resp_msg_addr, 0, sizeof(MSG_BSS));
}

int bss_cmd_setup(bss_chan_type_t ch)
{
	uint64_t *rdy_ptr;
	uint32_t bss_rt_ready;

	rdy_ptr = (uint64_t *)(pchbase + BSSRT_READY_OFFS);
	bss_rt_ready = (u32)(*rdy_ptr);
	if (bss_rt_ready != BST_RT_READY_MAGIC_NUM) {
		pr_err("%s: RT image is not running on BSS,read magic num: 0x%x",
				__func__, bss_rt_ready);
		return -ESRCH;
	}

	bss_clr_shm(ch);

	return 0;
}
EXPORT_SYMBOL(bss_cmd_setup);

int send_recv_bss_msg(bss_chan_type_t ch, uint8_t cmd, uint8_t msg_type, uint16_t msg_info,
							void *txbuf, uint32_t txbufsz,
							void *rxbuf, uint32_t *rxbufsz)
{
	int ret;
	MSG_BSS *msg;
	unsigned long flags;

	if (msg_type == RAPTOR2_MSGTYPE_READ) {
		if ((rxbuf == NULL) || (rxbufsz == NULL)) {
			pr_err("%s rxbuf%s is NULL\n", __func__, (!rxbufsz) ? "sz" : "");
			return -EINVAL;
		}
	}

	msg = kzalloc(sizeof(MSG_BSS), GFP_KERNEL);
	if (msg == NULL)
		return -ENOMEM;

	flags = claim_chan_lock(ch);

	ret = bss_cmd_setup(ch);
	if (ret < 0) {
		pr_err("%s: bss_cmd_setup failed for ch %d cmd %s\n",
			__func__, ch, cmdstr[cmd]);
		goto out;
	}

	ret = 0;

	ret = send_msg(ch, cmd, msg_type, msg_info, (uint8_t *)txbuf, txbufsz);
	if (ret < 0) {
		pr_err("%s: Failed to send msg %s to bss", __func__, cmdstr[cmd]);
		goto out;
	}

	ret = get_response_msg(ch, msg, MAX_RESP_MSG_DELAY);
	if (ret < 0) {
		pr_err("%s: get_response_msg for %s failed with err %d\n",
							__func__, cmdstr[cmd], ret);
		goto out;
	}

	if ((msg->id == cmd) &&
			(msg->type !=  (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
		pr_err("%s: ERROR: BSS failed to process command %s, status: 0x%02x",
					__func__, cmdstr[cmd], msg->type);
		ret = -EIO;
		goto out;
	}

	if (msg_type == RAPTOR2_MSGTYPE_READ) {
		memcpy(rxbuf, msg->data, msg->size);
		*rxbufsz = msg->size;
	}

out:
	release_chan_lock(ch, flags);
	kfree(msg);
	return ret;
}
EXPORT_SYMBOL(send_recv_bss_msg);

int bss_l1_reset(void)
{
	int ret;

	ret = send_recv_bss_msg(BSSRT_CHAN_SUINFRA, RAPTOR2_MSGID_L1_RESET,
			RAPTOR2_MSGTYPE_WRITE, 0, 0, 0, 0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed", __func__);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(bss_l1_reset);

int bss_read_data(char *buf, int *count, bss_data_type_t type)
{
	int ret, sz;
	uint16_t msgid;
	uint8_t *tmpbuf;
	bss_sumem_t bss_info;

	switch (type) {
	case BSS_PUBLIC_KEY:
		msgid = RAPTOR2_MSGID_PUBLIC_KEY_READ;
		break;
	case BSS_LICENSE_KEY:
		msgid = RAPTOR2_MSGID_LICENSE_KEY_READ;
		break;
	default:
		pr_err("%s: Incorrect data type %d\n", __func__, type);
		return -EINVAL;
	}

	ret = send_recv_bss_msg(BSSRT_CHAN_SUINFRA, msgid,
			RAPTOR2_MSGTYPE_READ, 0, 0, 0, &bss_info, &sz);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed for read %s", __func__, bdstr[type]);
		return ret;
	}

	tmpbuf = (uint8_t *)(psuimg + bss_info.offs);
	memcpy_fromio(buf, tmpbuf, bss_info.size);
	*count = bss_info.size;

	return 0;
}
EXPORT_SYMBOL(bss_read_data);

int bss_write_data(const char *buf, int bufsz, bss_data_type_t type)
{
	int ret;
	uint32_t offs;
	uint16_t msgid;
	bss_sumem_t bss_info;

	if ((buf == NULL) || (bufsz == 0)) {
		pr_err("%s: buf is NULL or bufsz is 0\n", __func__);
		return -EINVAL;
	}

	offs = BSS_MEM_TEMP_KEY_OFFS;
	switch (type) {
	case BSS_PUBLIC_KEY:
		msgid = RAPTOR2_MSGID_PUBLIC_KEY_WRITE;
		break;
	case BSS_LICENSE_KEY:
		msgid = RAPTOR2_MSGID_LICENSE_KEY_WRITE;
		break;
	default:
		pr_err("%s: Incorrect data type %d\n", __func__, type);
		return -EINVAL;
	}

	memset(&bss_info, 0, sizeof(bss_sumem_t));
	memcpy_toio(psuimg + offs, buf, bufsz);
	__iowmb();
	bss_info.addr = suimg_addr_bss + offs;
	bss_info.size = bufsz;
	bss_info.offs = 0x0;

	ret = send_recv_bss_msg(BSSRT_CHAN_SUINFRA, msgid,
			RAPTOR2_MSGTYPE_WRITE, 0, &bss_info, sizeof(bss_sumem_t),
			0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed for write %s\n", __func__, bdstr[type]);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(bss_write_data);

int bss_su_reset(bss_suinfo_t *suinfo)
{
	int ret;

	if (suinfo == NULL) {
		pr_err("%s: suinfo is NULL\n", __func__);
		return -EINVAL;
	}

	ret = send_recv_bss_msg(BSSRT_CHAN_SUINFRA, RAPTOR2_MSGID_SU_RESET,
			RAPTOR2_MSGTYPE_WRITE, 0, (uint8_t *)suinfo, sizeof(bss_suinfo_t), 0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed", __func__);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(bss_su_reset);

int bss_load_su_firmware(bss_suinfo_t *suinfo, uint8_t *hexbuf, uint32_t bufsz)
{
	int ret;

	if (suinfo == NULL) {
		pr_err("%s: suinfo is NULL\n", __func__);
		return -EINVAL;
	}

	if ((hexbuf == NULL) || (bufsz == 0)) {
		pr_err("%s: hexbuf is NULL or bufsz is 0\n", __func__);
		return -EINVAL;
	}

	memcpy_toio(psuimg, hexbuf, bufsz);
	__iowmb();
	suinfo->bss.addr = suimg_addr_bss;
	suinfo->bss.size = bufsz;
	suinfo->bss.offs = 0x0;

	ret = send_recv_bss_msg(BSSRT_CHAN_SUINFRA, RAPTOR2_MSGID_LOAD_SU_FIRMWARE,
			RAPTOR2_MSGTYPE_WRITE, 0, (uint8_t *)suinfo, sizeof(bss_suinfo_t), 0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed", __func__);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(bss_load_su_firmware);

static void init_spinlocks(void)
{
	int i;

	for (i = 0; i < BSSRT_CHAN_MAX; i++)
		spin_lock_init(&chlock[i]);
}

static int raptor2_bsscmdq_probe(struct platform_device *pdev)
{
	void *ptr;
	uint64_t upper;
	struct resource *res;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;

	match = of_match_device(raptor2_bsscmdq_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find bsscmdq node\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Memory resource for idx 0 suimg not found\n");
		return -ENODEV;
	}

	suimg_addr = (uint64_t)(res->start);
	suimg_sz = (uint64_t)(resource_size(res));
	psuimg = devm_memremap(dev, suimg_addr, suimg_sz, MEMREMAP_WT);
	if (!psuimg) {
		pr_err("%s: devm_memremap failed for suimg addr 0x%llx\n", __func__, suimg_addr);
		return -ENOMEM;
	}

	upper = (suimg_addr & 0x0000000F00000000) >> 4;
	suimg_addr_bss = (uint32_t) (suimg_addr | upper);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Memory resource for channel base idx 1 not found\n");
		return -ENODEV;
	}

	chbase_addr = (uint64_t)(res->start);
	chbase_sz = (uint64_t)(resource_size(res));
	pchbase = devm_memremap(dev, chbase_addr, chbase_sz, MEMREMAP_WT);
	if (!pchbase) {
		pr_err("%s: devm_memremap failed for suimg addr 0x%llx\n", __func__, chbase_addr);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "Memory resource for reset addr idx 2 not found\n");
		return -ENODEV;
	}

	reset_addr = (uint64_t)(res->start);
	reset_sz = (uint64_t)(resource_size(res));
	ptr = devm_memremap(dev, reset_addr, reset_sz, MEMREMAP_WT);
	if (!ptr) {
		pr_err("%s: devm_memremap failed for reset/efuse chan addr 0x%llx\n",
				__func__, reset_addr);
		return -ENOMEM;
	}
	pchtbl[BSSRT_CHAN_EFUSE] = ptr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 5);
	if (!res) {
		dev_err(&pdev->dev, "Memory resource for suinfra idx 5 not found\n");
		return -ENODEV;
	}

	suinfra_addr = (uint64_t)(res->start);
	suinfra_sz = (uint64_t)(resource_size(res));
	ptr = devm_memremap(dev, suinfra_addr, suinfra_sz, MEMREMAP_WT);
	if (!ptr) {
		pr_err("%s: memremap failed for suinfra chan addr 0x%llx\n",
				__func__, suinfra_addr);
		return -ENOMEM;
	}
	pchtbl[BSSRT_CHAN_SUINFRA] = ptr;

	/* init per channel spinlock */
	init_spinlocks();

	dev_info(&pdev->dev, "Probe for Platform Driver bsscmdq Successful\n");

	return 0;
}

static int raptor2_bsscmdq_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	devm_memunmap(dev, psuimg);
	devm_memunmap(dev, pchbase);
	devm_memunmap(dev, pchtbl[BSSRT_CHAN_EFUSE]);
	devm_memunmap(dev, pchtbl[BSSRT_CHAN_SUINFRA]);

	return 0;
}

static struct platform_driver raptor2_bsscmdq_driver = {
	.driver         = {
		.owner = THIS_MODULE,
		.name = "raptor2-bsscmdq",
		.of_match_table = raptor2_bsscmdq_of_match,
	},
	.probe = raptor2_bsscmdq_probe,
	.remove = raptor2_bsscmdq_remove,
};

builtin_platform_driver(raptor2_bsscmdq_driver);

MODULE_AUTHOR("Pravin Bathija <bathija@edgeq.io>");
MODULE_DESCRIPTION("Raptor BSS RT Cmd Q driver");
MODULE_LICENSE("GPL");
