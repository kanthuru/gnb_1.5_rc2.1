/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (c) 2022 EdgeQ Inc.
 */

#ifndef __RAPTOR2_CRSS_QAU_REGS_H_
#define __RAPTOR2_CRSS_QAU_REGS_H_


#define STORAGE_MODE_LINEAR		0x0
#define STORAGE_MODE_LINEAR_CIRC_BUFFER 0x1


/* CRSS QAU Registers */

#define CRSS_QAU_TRIGGER__ADDR 0x0
union qau_trigger_reg {
	struct {
		uint32_t init: 1;
		uint32_t crypto_cmd_cnt_decr: 1;
		uint32_t crypto_sts_cnt_incr: 1;
		uint32_t inp_cmd_rd: 1;
		uint32_t out_sts_wr: 1;
		uint32_t post_hdrproc_cmd_wr: 1;
		uint32_t post_hdrproc_cmd_rd: 1;
		uint32_t reserved : 25;
	} bf;
	uint32_t bits;
} __packed;

#define CRSS_QAU_EVENT_CLEAR__ADDR 0x10
union qau_event_clear_reg {
	struct {
		uint32_t crypto_eng_0_evt_clr: 1;
		uint32_t crypto_eng_1_evt_clr: 1;
		uint32_t crypto_eng_2_evt_clr: 1;
		uint32_t crypto_eng_3_evt_clr: 1;
		uint32_t crypto_eng_4_evt_clr: 1;
		uint32_t crypto_eng_5_evt_clr: 1;
		uint32_t cmd_ptr_match_evt_clr: 1;
		uint32_t sts_ptr_match_evt_clr: 1;
		uint32_t cmd_q_1_ptr_match_evt_clr: 1;
		uint32_t sts_q_1_ptr_match_evt_clr: 1;
		uint32_t reserved1 : 6;
		uint32_t rip_addr_err_evt_clr: 1;
		uint32_t rip_ext_addr_err_evt: 1;
		uint32_t reserved2 : 14;
	} bf;
	uint32_t bits;
} __packed;

#define CRSS_QAU_INTR_CFG__ADDR 0x28
union qau_intr_cfg_reg {
	struct {
		uint32_t en_cmd_ptr_match_int: 1;
		uint32_t en_sts_ptr_match_int: 1;
		uint32_t en_cmd_q_1_ptr_match_int: 1;
		uint32_t en_sts_q_1_ptr_match_int: 1;
		uint32_t reserved : 28;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CMD_RDR_CFG_1__ADDR 0x30
union qau_cmd_rdr_cfg_1_reg {
	struct {
		uint32_t outs_en: 1;
		uint32_t dim: 4;
		uint32_t mode: 4;
		uint32_t elem_size: 8;
		uint32_t reserved1: 7;
		uint32_t timer_en: 1;
		uint32_t reserved2: 7;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CMD_RDR_CFG_2__ADDR 0x34
union qau_cmd_rdr_cfg_2_reg {
	struct {
		uint32_t buff_size: 16;
		uint32_t offset: 16;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CMD_RDR_CFG_3__ADDR 0x38
union qau_cmd_rdr_cfg_3_reg {
	struct {
		uint32_t base_addr_lo: 32;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CMD_RDR_CFG_4__ADDR 0x3c
union qau_cmd_rdr_cfg_4_reg {
	struct {
		uint32_t base_addr_hi: 8;
		uint32_t flush_threshold: 8;
		uint32_t timer_config: 16;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CMD_RDR_CFG_5__ADDR 0x40

#define CRSS_QAU_STS_WTR_CFG_1__ADDR 0x4c
union qau_sts_wtr_cfg_1_reg {
	struct {
		uint32_t outs_en: 1;
		uint32_t dim: 4;
		uint32_t mode: 4;
		uint32_t elem_size: 8;
		uint32_t reserved1: 7;
		uint32_t timer_en: 1;
		uint32_t reserved2: 7;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_STS_WTR_CFG_2__ADDR 0x50
union qau_sts_wtr_cfg_2_reg {
	struct {
		uint32_t buff_size: 16;
		uint32_t offset: 16;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_CRYPTO_STS_LEN__ADDR 0x80
union qau_crypto_sts_len_reg {
	struct {
		uint32_t len: 16;
		uint32_t reserved : 16;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_STS_WTR_CFG_3__ADDR 0x54
union qau_sts_wtr_cfg_3_reg {
	struct {
		uint32_t base_addr_lo: 32;
	} bf;
	uint32_t bits;
} __packed;


#define CRSS_QAU_STS_WTR_CFG_4__ADDR 0x58
union qau_sts_wtr_cfg_4_reg {
	struct {
		uint32_t base_addr_hi: 8;
		uint32_t flush_threshold: 8;
		uint32_t timer_config: 16;
	} bf;
	uint32_t bits;
} __packed;

#define CRSS_QAU_MEMSS_SRC_BASE__ADDR 0x64
#define CRSS_QAU_MEMSS_SRC_BUF_ROOM__ADDR 0x6c
#define CRSS_QAU_CMD_RDR_RD_PTR__ADDR 0xb0
#define CRSS_QAU_STS_WTR_RD_PTR__ADDR 0xb8
#define CRSS_QAU_STS_WTR_CFG_5__ADDR 0x5c
#define CRSS_QAU_STS_PTR_MATCH_INT__ADDR 0x60


#endif /* __RAPTOR2_CRSS_QAU_REGS_H_ */
