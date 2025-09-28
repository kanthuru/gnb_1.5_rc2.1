// PCIe eDMA Registers

#define EDMA_DIR_WRITE                                  0
#define EDMA_V0_MAX_NR_CH                               8
#define EDMA_V0_VIEWPORT_MASK                           CREATE_MASK(2, 0)
#define EDMA_V0_DONE_INT_MASK                           CREATE_MASK(7, 0)
#define EDMA_V0_ABORT_INT_MASK                          CREATE_MASK(23, 16)
#define EDMA_V0_WRITE_CH_COUNT_MASK                     CREATE_MASK(3, 0)
#define EDMA_V0_READ_CH_COUNT_MASK                      CREATE_MASK(19, 16)
#define EDMA_V0_CH_STATUS_MASK                          CREATE_MASK(6, 5)
#define EDMA_V0_DOORBELL_CH_MASK                        CREATE_MASK(2, 0)
#define EDMA_V0_LINKED_LIST_ERR_MASK                    CREATE_MASK(7, 0)

#define EDMA_V0_CH_ODD_MSI_DATA_MASK                    CREATE_MASK(31, 16)
#define EDMA_V0_CH_EVEN_MSI_DATA_MASK                   CREATE_MASK(15, 0)

struct dw_edma_v0_ch_regs {
	uint32_t ch_control1;                           /* 0x000 */
	uint32_t ch_control2;                           /* 0x004 */
	uint32_t transfer_size;                         /* 0x008 */
	uint32_t sar_low;                               /* 0x00c */
	uint32_t sar_high;                              /* 0x010 */
	uint32_t dar_low;                               /* 0x014 */
	uint32_t dar_high;                              /* 0x018 */
	uint32_t llp_low;                               /* 0x01c */
	uint32_t llp_high;                              /* 0x020 */
};

struct dw_edma_v0_ch {
	struct dw_edma_v0_ch_regs wr;                   /* 0x200 */
	uint32_t padding_1[55];                         /* [0x224..0x2fc] */
	struct dw_edma_v0_ch_regs rd;                   /* 0x300 */
	uint32_t padding_2[55];                         /* [0x324..0x3fc] */
};

struct dw_edma_v0_unroll {
	uint32_t padding_1;                             /* 0x0f8 */
	uint32_t wr_engine_chgroup;                     /* 0x100 */
	uint32_t rd_engine_chgroup;                     /* 0x104 */
	uint32_t wr_engine_hshake_cnt_low;              /* 0x108 */
	uint32_t wr_engine_hshake_cnt_high;             /* 0x10c */
	uint32_t padding_2[2];                          /* [0x110..0x114] */
	uint32_t rd_engine_hshake_cnt_low;              /* 0x118 */
	uint32_t rd_engine_hshake_cnt_high;             /* 0x11c */
	uint32_t padding_3[2];                          /* [0x120..0x124] */
	uint32_t wr_ch0_pwr_en;                         /* 0x128 */
	uint32_t wr_ch1_pwr_en;                         /* 0x12c */
	uint32_t wr_ch2_pwr_en;                         /* 0x130 */
	uint32_t wr_ch3_pwr_en;                         /* 0x134 */
	uint32_t wr_ch4_pwr_en;                         /* 0x138 */
	uint32_t wr_ch5_pwr_en;                         /* 0x13c */
	uint32_t wr_ch6_pwr_en;                         /* 0x140 */
	uint32_t wr_ch7_pwr_en;                         /* 0x144 */
	uint32_t padding_4[8];                          /* [0x148..0x164] */
	uint32_t rd_ch0_pwr_en;                         /* 0x168 */
	uint32_t rd_ch1_pwr_en;                         /* 0x16c */
	uint32_t rd_ch2_pwr_en;                         /* 0x170 */
	uint32_t rd_ch3_pwr_en;                         /* 0x174 */
	uint32_t rd_ch4_pwr_en;                         /* 0x178 */
	uint32_t rd_ch5_pwr_en;                         /* 0x18c */
	uint32_t rd_ch6_pwr_en;                         /* 0x180 */
	uint32_t rd_ch7_pwr_en;                         /* 0x184 */
	uint32_t padding_5[30];                         /* [0x188..0x1fc] */
	struct dw_edma_v0_ch ch[EDMA_V0_MAX_NR_CH];     /* [0x200..0x1120] */
};

struct dw_edma_v0_regs {
	/* eDMA global registers */
	uint32_t ctrl_data_arb_prior;                        /* 0x000 */
	uint32_t padding_1;                                  /* 0x004 */
	uint32_t ctrl;                                       /* 0x008 */
	uint32_t wr_engine_en;                               /* 0x00c */
	uint32_t wr_doorbell;                                /* 0x010 */
	uint32_t padding_2;                                  /* 0x014 */
	uint32_t wr_ch_arb_weight_low;                       /* 0x018 */
	uint32_t wr_ch_arb_weight_high;                      /* 0x01c */
	uint32_t padding_3[3];                               /* [0x020..0x028] */
	uint32_t rd_engine_en;                               /* 0x02c */
	uint32_t rd_doorbell;                                /* 0x030 */
	uint32_t padding_4;                                  /* 0x034 */
	uint32_t rd_ch_arb_weight_low;                       /* 0x038 */
	uint32_t rd_ch_arb_weight_high;                      /* 0x03c */
	uint32_t padding_5[3];                               /* [0x040..0x048] */
	/* eDMA interrupts registers */
	uint32_t wr_int_status;                              /* 0x04c */
	uint32_t padding_6;                                  /* 0x050 */
	uint32_t wr_int_mask;                                /* 0x054 */
	uint32_t wr_int_clear;                               /* 0x058 */
	uint32_t wr_err_status;                              /* 0x05c */
	uint32_t wr_done_imwr_low;                           /* 0x060 */
	uint32_t wr_done_imwr_high;                          /* 0x064 */
	uint32_t wr_abort_imwr_low;                          /* 0x068 */
	uint32_t wr_abort_imwr_high;                         /* 0x06c */
	uint32_t wr_ch01_imwr_data;                          /* 0x070 */
	uint32_t wr_ch23_imwr_data;                          /* 0x074 */
	uint32_t wr_ch45_imwr_data;                          /* 0x078 */
	uint32_t wr_ch67_imwr_data;                          /* 0x07c */
	uint32_t padding_7[4];                               /* [0x080..0x08c] */
	uint32_t wr_linked_list_err_en;                      /* 0x090 */
	uint32_t padding_8[3];                               /* [0x094..0x09c] */
	uint32_t rd_int_status;                              /* 0x0a0 */
	uint32_t padding_9;                                  /* 0x0a4 */
	uint32_t rd_int_mask;                                /* 0x0a8 */
	uint32_t rd_int_clear;                               /* 0x0ac */
	uint32_t padding_10;                                 /* 0x0b0 */
	uint32_t rd_err_status_low;                          /* 0x0b4 */
	uint32_t rd_err_status_high;                         /* 0x0b8 */
	uint32_t padding_11[2];                              /* [0x0bc..0x0c0] */
	uint32_t rd_linked_list_err_en;                      /* 0x0c4 */
	uint32_t padding_12;                                 /* 0x0c8 */
	uint32_t rd_done_imwr_low;                           /* 0x0cc */
	uint32_t rd_done_imwr_high;                          /* 0x0d0 */
	uint32_t rd_abort_imwr_low;                          /* 0x0d4 */
	uint32_t rd_abort_imwr_high;                         /* 0x0d8 */
	uint32_t rd_ch01_imwr_data;                          /* 0x0dc */
	uint32_t rd_ch23_imwr_data;                          /* 0x0e0 */
	uint32_t rd_ch45_imwr_data;                          /* 0x0e4 */
	uint32_t rd_ch67_imwr_data;                          /* 0x0e8 */
	uint32_t padding_13[4];                              /* [0x0ec..0x0f8] */

	/* eDMA channel context grouping */
	struct dw_edma_v0_unroll unroll;                /* [0x0f8..0x1120] */
};