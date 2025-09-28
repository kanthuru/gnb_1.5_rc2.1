/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCIe HDMA Register Space for Raptor2 B0.
 *
 */

#define CREATE_MASK64(end, start)	((~((uint64_t)(0)) - (1UL << (start)) + 1) &\
					(~((uint64_t)(0)) >> (BITS_PER_LONG - 1 - (end))))

#define LINK4_HDMA_V0_MAX_NR_CH		16
#define LINK4_DMA_RD_NR_CH			16
#define LINK4_DMA_WR_NR_CH			16

#define LINK3_HDMA_V0_MAX_NR_CH		8
#define LINK3_DMA_RD_NR_CH			8
#define LINK3_DMA_WR_NR_CH			8

/* HDMA Channel Enable Register */
#define HDMA_CH_ENABLE			(1UL<<0)

/* HDMA Status Register */
#define HDMA_CH_STS_RSVD	0x0
#define HDMA_CH_STS_RUN	0x1
#define HDMA_CH_STS_HALT	0x2
#define HDMA_CH_STS_STOP	0x3
#define HDMA_CH_STATUS_MASK	CREATE_MASK64(2, 0)

/* HDMA PCCS register */
#define HDMA_PCCS_CCS_BIT		(1UL<<1)
#define HDMA_PCCS_CB_BIT		(1UL<<0)

/* HDMA Control1 register */
#define HDMA_CONTROL_LLE_BIT		(1UL<<0)

/* Doorbell register */
#define HDB_STOP_BIT			(1UL<<1)
#define HDB_START_BIT			(1UL<<0)

/* Prefetch register */
#define HDMA_ELE_PFETCH_MASK		CREATE_MASK64(6, 0)

/* Interrupt Status register */
#define HDMA_INT_STS_ABORT		(1UL<<2)
#define HDMA_INT_STS_WM			(1UL<<1)
#define HDMA_INT_STS_STOP		(1UL<<0)
#define HDMA_INT_STA_ERR_MASK		CREATE_MASK64(6, 3)

/* Interrupt Setup register */
#define HDMA_INT_SETUP_LAIE		(1UL<<6)
#define HDMA_INT_SETUP_RAIE		(1UL<<5)
#define HDMA_INT_SETUP_LSIE		(1UL<<4)
#define HDMA_INT_SETUP_RSIE		(1UL<<3)
#define HDMA_INT_SETUP_ABORT_MASK	(1UL<<2)
#define HDMA_INT_SETUP_WM_MASK		(1UL<<1)
#define HDMA_INT_SETUP_STOP_MASK	(1UL<<0)

/* Interrupt Clear register */
#define HDMA_INT_CLR_ABORT		(1UL<<2)
#define HDMA_INT_CLR_WM			(1UL<<1)
#define HDMA_INT_CLR_STOP		(1UL<<0)

/* Data Element */
#define DATA_ELEMENT_RIE_BIT		(1UL<<4)
#define DATA_ELEMENT_LIE_BIT		(1UL<<3)
#define DATA_ELEMENT_LLP_BIT		(1UL<<2)
#define DATA_ELEMENT_CB_BIT		(1UL<<0)

/* Link Element */
#define LINK_ELEMENT_LLP_MASK		(1UL<<2)
#define LINK_ELEMENT_TCB_MASK		(1UL<<1)
#define LINK_ELEMENT_CB_MASK		(1UL<<0)

typedef struct dw_hdma_ch_regs {
	uint32_t ch_en;				/* 0x0000 */
	uint32_t doorbell;			/* 0x0004 */
	uint32_t prefetch;			/* 0x0008 */
	uint32_t handshake;			/* 0x000c */
	uint32_t llp_low;			/* 0x0010 */
	uint32_t llp_high;			/* 0x0014 */
	uint32_t cycle_sync;			/* 0x0018 */
	uint32_t transfer_size;			/* 0x001c */
	uint32_t sar_low;			/* 0x0020 */
	uint32_t sar_high;			/* 0x0024 */
	uint32_t dar_low;			/* 0x0028 */
	uint32_t dar_high;			/* 0x002c */
	uint32_t watermark_en;			/* 0x0030 */
	uint32_t control1;			/* 0x0034 */
	uint32_t func_num;			/* 0x0038 */
	uint32_t qos;				/* 0x003c */
	uint32_t reserved[16];			/* 0x0040..0x007c */
	uint32_t ch_stat;			/* 0x0080 */
	uint32_t int_stat;			/* 0x0084 */
	uint32_t int_setup;			/* 0x0088 */
	uint32_t int_clear;			/* 0x008c */
	uint32_t msi_stop_low;			/* 0x0090 */
	uint32_t msi_stop_high;			/* 0x0094 */
	uint32_t msi_watermark_low;		/* 0x0098 */
	uint32_t msi_watermark_high;		/* 0x009c */
	uint32_t msi_abort_low;			/* 0x00a0 */
	uint32_t msi_abort_high;		/* 0x00a4 */
	uint32_t msi_msgdata;			/* 0x00a8 */
	uint32_t pad;
} dw_hdma_ch_regs_t;

struct L4_dw_hdma_v0_ch {
	dw_hdma_ch_regs_t wr;		/* 0x0000 */
	uint32_t pad[212];				/* 0x00b0--0x03fc rsvd Padding */
	dw_hdma_ch_regs_t rd;		/* 0x0400 */
	uint32_t pad2[212];				/* 0x04b0--0x07fc rsvd Padding */
};

struct L3_dw_hdma_v0_ch {
	dw_hdma_ch_regs_t wr;		/* 0x0000 */
	uint32_t pad[468];				/* 0x00b0--0x03fc rsvd Padding */
	dw_hdma_ch_regs_t rd;		/* 0x0800 */
	uint32_t pad2[468];				/* 0x04b0--0x07fc rsvd Padding */
};

typedef struct L4_dw_hdma_regs {
	struct L4_dw_hdma_v0_ch ch[LINK4_HDMA_V0_MAX_NR_CH];	/* 0x0000..0x0fa8 */
} L4_dw_hdma_regs_t;

typedef struct L3_dw_hdma_regs {
	struct L3_dw_hdma_v0_ch ch[LINK3_HDMA_V0_MAX_NR_CH];	/* 0x0000..0x0fa8 */
} L3_dw_hdma_regs_t;
