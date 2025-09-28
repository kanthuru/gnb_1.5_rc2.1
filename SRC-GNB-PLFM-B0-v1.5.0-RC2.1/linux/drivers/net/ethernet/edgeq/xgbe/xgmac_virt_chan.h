/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 EdgeQ, Inc. and/or its affiliates.
 */

#ifndef __XGMAC_VIRT_CHAN_H__
#define __XGMAC_VIRT_CHAN_H__

#include "common.h"
#include "dwxgmac.h"

/* RX Descriptor Definitions
 * Unless defined, XGMAC macros maybe used.
 */
#define XGMAC_VIRT_CHAN_RDES3_ES BIT(16)
#define XGMAC_VIRT_CHAN_RDES3_PL GENMASK(15, 0)

static inline int virt_chan_get_rx_status(struct dma_desc *p)
{
	unsigned int rdes3 = le32_to_cpu(p->des3);

	if (unlikely(rdes3 & XGMAC_RDES3_OWN))
		return dma_own;
	if (likely(!(rdes3 & XGMAC_RDES3_LD)))
		return rx_not_ls;
	if (unlikely((rdes3 & XGMAC_VIRT_CHAN_RDES3_ES) && (rdes3 & XGMAC_RDES3_LD)))
		return discard_frame;

	return good_frame;
}


static inline unsigned int virt_chan_get_rx_frame_len(struct dma_desc *p)
{
	return (le32_to_cpu(p->des3) & XGMAC_VIRT_CHAN_RDES3_PL);
}

#endif /* __XGMAC_VIRT_CHAN_H__ */
