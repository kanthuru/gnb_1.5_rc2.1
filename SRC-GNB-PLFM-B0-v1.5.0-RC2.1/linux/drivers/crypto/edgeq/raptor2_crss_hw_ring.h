/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (c) 2022 EdgeQ Inc.
 */

#ifndef RAPTOR2_CRSS_HW_RING_H
#define RAPTOR2_CRSS_HW_RING_H

					/* Cmd descriptor is around 18 times
					 * bigger than the status descriptor.
					 */
#define CRSS_CMDQ_RING_SIZE		(512 * 18)
#define CRSS_CMDQ_ELEM_SIZE		4
#define CRSS_STSQ_RING_SIZE		512
#define CRSS_STSQ_ELEM_SIZE		sizeof(struct crypto_status)
#define CRSS_CMDQ_SIZE			(CRSS_CMDQ_RING_SIZE * CRSS_CMDQ_ELEM_SIZE)
#define CRSS_STSQ_SIZE			(CRSS_STSQ_RING_SIZE * CRSS_STSQ_ELEM_SIZE)
#define CRSS_CMDQ_STSQ_SIZE		(CRSS_CMDQ_SIZE + CRSS_STSQ_SIZE)
#define CRSS_ELEMT_SIZE			0x20

#define QUEUE_TYPE_STATUS		0
#define QUEUE_TYPE_COMMAND		1

#define CRSS_QAU_ADDR_RANGE		0x40000

#define GET_QAU_BASEADDR(base_addr, cmdq_id) (base_addr + (cmdq_id * CRSS_QAU_ADDR_RANGE))
#define GET_RING_BYTE_SIZE(ring_ctxt) ((ring_ctxt)->size *  (ring_ctxt)->elem_size)

#define RAPTOR2_WRITEQ(val, ptr) (*(uint64_t *)ptr = val)
#define RAPTOR2_WRITEL(val, ptr) (*(uint32_t *)ptr = val)


#define L_HALF_WORD(x) ((x) & HALF_WORD_MASK)
#define U_HALF_WORD(x) (((x) & HALF_WORD_MASK) << 16)

#define L_WORD(x)      ((x) & WORD_MASK)
#define U_WORD(x)      ((((uint64_t)x) & WORD_MASK) << 32)


struct crss_hw_ring_ctx {
	uint8_t		*base_addr;
	uint8_t		*end_addr;
	void __iomem	*hw_write_addr;
	void __iomem	*hw_read_addr;
	uint32_t	ring_id;
	uint32_t	sw_write_offset;
	uint32_t	sw_read_offset;
	uint32_t	size;
	uint32_t	elem_size;
	uint32_t	type;
};

static inline int bytes_at_tail(struct crss_hw_ring_ctx *ring, uint32_t *ptr)
{
	return ((uint64_t)ring->end_addr - (uint64_t)(ptr));
}

static inline uint32_t *check_roll_over(uint32_t *ptr, struct crss_hw_ring_ctx *ring)
{
	return ((ptr >= (uint32_t *)ring->end_addr) ?
		(uint32_t *)(((uint64_t)ptr - (uint64_t)ring->end_addr) +
						(uint64_t)ring->base_addr) : ptr);
}

static inline uint32_t *cmd_split_write(struct crss_hw_ring_ctx *ring, uint32_t *ptr, uint64_t val)
{
	if (bytes_at_tail(ring, ptr) > 4) {
		RAPTOR2_WRITEQ(val, ptr);
	} else {
		RAPTOR2_WRITEL(val & 0xFFFFFFFF, check_roll_over(ptr, ring));
		RAPTOR2_WRITEL((val>>32) & 0xFFFFFFFF, check_roll_over(ptr + 1, ring));
	}

	return ptr+2;
}


void *crss_hw_ring_alloc_cmd_buf(struct crss_hw_ring_ctx *ring, uint32_t command_size);
inline void *crss_hw_ring_deq_sts_buf(struct crss_hw_ring_ctx *ring);
inline uint32_t crss_get_free_cmd_buff(struct crss_hw_ring_ctx *ring);
inline uint32_t crss_get_num_status(struct crss_hw_ring_ctx *ring);
inline void crss_cmd_queue_submit(struct crss_hw_ring_ctx *ring);
inline void crss_status_queue_submit(struct crss_hw_ring_ctx *ring);


#endif /* RAPTOR2_CRSS_HW_RING_H */
