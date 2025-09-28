// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 EdgeQ Inc.
 */

#include <linux/io.h>
#include "raptor2_crss_hw_ring.h"


static inline uint32_t hw_ring_get_num_write_buf_avail(struct crss_hw_ring_ctx *ring)
{
	uint32_t read_offset  = ring->sw_read_offset;
	uint32_t write_offset = ring->sw_write_offset;

	if (read_offset > write_offset)
		return (read_offset - write_offset)/ring->elem_size;
	else
		return (((GET_RING_BYTE_SIZE(ring) - write_offset) + read_offset) /
									ring->elem_size) - 1;
}

static inline uint32_t hw_ring_get_num_read_buf_avail(struct crss_hw_ring_ctx *ring)
{
	uint32_t read_offset  = ring->sw_read_offset;
	uint32_t write_offset = ring->sw_write_offset;

	if (write_offset >= read_offset)
		return (write_offset - read_offset)/ring->elem_size;
	else
		return ((GET_RING_BYTE_SIZE(ring) - read_offset) + write_offset) / ring->elem_size;
}


void *crss_hw_ring_alloc_cmd_buf(struct crss_hw_ring_ctx *ring, uint32_t command_size)
{
	uint32_t num_write_buf_avail = hw_ring_get_num_write_buf_avail(ring);
	uint8_t *new_write_addr;

	if (num_write_buf_avail > (command_size/ring->elem_size)) {
		new_write_addr = ring->base_addr + ring->sw_write_offset;

		ring->sw_write_offset = (ring->sw_write_offset + command_size)
							% GET_RING_BYTE_SIZE(ring);

		return new_write_addr;
	}

	return NULL;
}


inline void *crss_hw_ring_deq_sts_buf(struct crss_hw_ring_ctx *ring)
{
	uint32_t num_read_buf_avail = hw_ring_get_num_read_buf_avail(ring);
	uint8_t *new_read_addr;

	if (num_read_buf_avail > 0) {
		ring->sw_read_offset = ring->sw_read_offset % GET_RING_BYTE_SIZE(ring);
		new_read_addr = ring->base_addr + ring->sw_read_offset;
		ring->sw_read_offset = (ring->sw_read_offset + ring->elem_size);
		return new_read_addr;
	}

	return NULL;
}

inline uint32_t crss_get_free_cmd_buff(struct crss_hw_ring_ctx *ring)
{
	ring->sw_read_offset = readl(ring->hw_read_addr);
	return hw_ring_get_num_write_buf_avail(ring);
}

inline uint32_t crss_get_num_status(struct crss_hw_ring_ctx *ring)
{
	ring->sw_write_offset = readl(ring->hw_write_addr);

	if (ring->sw_write_offset == GET_RING_BYTE_SIZE(ring))
		ring->sw_write_offset = 0;

	return hw_ring_get_num_read_buf_avail(ring);
}


inline void crss_cmd_queue_submit(struct crss_hw_ring_ctx *ring)
{
	writel(ring->sw_write_offset, ring->hw_write_addr); /* Update write index */
}

inline void crss_status_queue_submit(struct crss_hw_ring_ctx *ring)
{
	writel(ring->sw_read_offset, ring->hw_read_addr); /* Update read index */
}

