/* SPDX-License-Identifier: GPL-2.0 */
/*
 * UIO virtual driver header file
 *
 * Copyright (C) 2023 EdgeQ Inc.
 *
 * Author: Aakash Verma <aakash.verma@edgeq.io>
 */

#ifndef __EDGEQ_VIRT_UIO_H__
#define __EDGEQ_VIRT_UIO_H__

#define MAX_VIRT_FUNC		8
#define MAX_PCIE_RES		1
#define MAX_XGMAX_RES		3
#define MAX_VDEV_VIRT_RES	1
#define PCIE_WA_NAME		"PCIE_DMA_WA"
#define XGMAC_VIRT_NAME		"XGMAC_VIRT"
#define NLK_EDGEQ_VIRT		28
#define MEM_REGION_COUNT	4
#define DMA_WA_SIZE		0x400000			/* 4 MB */
#define DMA_CHAN_SIZE		0x1000				/* 4 KB */
#define DMA_WROFF(x)		((x*2)*DMA_CHAN_SIZE)
#define DMA_RDOFF(x)		(((x*2)+1)*DMA_CHAN_SIZE)
#define DMA_WAOFF(x)		(x*DMA_WA_SIZE)
#define INVALID_DMA_CH_ID	0xFF

enum dma_ch_type {
	DMA_READ_CHAN = 1,
	DMA_WRITE_CHAN,
};

enum res_node {
	VDEV_PCIE_WA = 1,
	VDEV_XGMAC,
};

/* Global Capability Structure */
struct pf_dev {
	uint64_t	vf_count;
} __packed __aligned(8);

struct vf_dev {
	uint64_t	dev_id;
	uint64_t	dev_name;
	uint64_t	cap_flg;
} __packed __aligned(8);

struct virt_reg {
	struct pf_dev	pf0;
	struct vf_dev	vfn[MAX_VIRT_FUNC];
} __packed __aligned(8);

/* Netlink data structure */
enum nlk_request_code {
	REQ_INIT_VDEV = 1,
	REQ_DEINIT_VDEV,
	REQ_EXPOSE_UIO,
};

enum nlk_response_code {
	VDEV_REQ_FAIL = 1,
	VDEV_REQ_DONE,
	VDEV_REQ_NOACTION,
	VDEV_REQ_INUSE,
	VDEV_REQ_UNCAP_DONE,
};

struct nlk_request {
	/* REQ_INIT/DEINIT */
	uint32_t	vdev_id;
	uint32_t	vdev_cap;

	/* REQ_EXPOSE_UIO */
	uint64_t	start_addr;
	uint64_t	size;
	uint64_t	mem_type;

	uint64_t	req_type;
} __packed __aligned(8);

struct nlk_response {
	uint64_t status_code;
	uint64_t ul_dma_chan_id;
	uint64_t dl_dma_chan_id;
} __packed __aligned(8);

/* Driver data structures */
enum virt_uio_map {
	MAP0 = 0,
	MAP1,
	MAP2,
	MAP3,
	MAP4,
};

enum vdev_capibility {
	MEMCPY_CAP = 1,
	DMA_CAP_BOTH,
	DMA_CAP_RD,
	DMA_CAP_WR,
	INVALID_CAP,
};

struct dma_cap {
	uint32_t	dma_rd_ch_id;
	uint32_t	dma_wr_ch_id;
	phys_addr_t	dma_wa_paddr;	/* dma work-area = LL + Scratchpad */
	phys_addr_t	dma_chan_paddr;
};

struct vdev_cap {
	uint32_t	cap_flag;
	struct dma_cap	dma_info;
};

struct virtmem {
	phys_addr_t paddr;
	phys_addr_t vaddr;
	phys_addr_t size;
};

struct vx_res {
	struct virtmem reg[MAX_XGMAX_RES];
};

struct vdev_sz {
	uint32_t reg_size;
	uint32_t rx_size;
	uint32_t tx_size;
};

struct eq_dev {
	uint32_t	device_id;
	uint32_t	in_use;
	char		*device_name;
	struct vx_res	xgmac;
	struct uio_info	*be_uio;
	struct uio_info	*fe_uio;
	struct vdev_cap	cap;
};

struct eq_virt {
	struct device	*dev;

	struct resource *dma_wa_res;
	struct resource *dma_chan;

	uint32_t	dma_ch_count;
	uint32_t	dma_wr_inuse_bmap;
	uint32_t	dma_rd_inuse_bmap;

	bool		use_nlk;
	struct sock	*netlink_sk;

	uint32_t	vdev_count;
	struct vdev_sz	mem_sz;
	struct resource *vdev_res[MAX_XGMAX_RES];
	struct eq_dev	*pdev;		/* Future use */
	struct eq_dev	*vdev;
};
#endif /* __EDGEQ_VIRT_UIO_H__ */
