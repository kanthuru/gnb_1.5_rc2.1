/* SPDX-License-Identifier: GPL-2.0-or-later
 * Raptor2 Header file for BSS SU Firmware Loading
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Pravin Bathija <bathija@edgeq.io>
 */

typedef struct bss_sumem {
	uint32_t addr;
	uint32_t offs;
	uint32_t size;
} __packed bss_sumem_t;

typedef struct bss_suinfo {
	uint32_t count;
	uint8_t etype;
	uint8_t suid;
	uint8_t rstval;
	uint32_t membm;
	uint32_t sudev_base;
	uint32_t cmuoffs;
	bss_sumem_t bss;
	bss_sumem_t ilm;
	bss_sumem_t dlm;
	bss_sumem_t lmem_wb;
	bss_sumem_t lmem_nc;
	bss_sumem_t lmem_wt;
	bss_sumem_t lmem_cnoc;
	bss_sumem_t lmem_cnoc_b2;
	bss_sumem_t clilm_local;
	bss_sumem_t clilm_cnoc;
	uint8_t res[117];
} __packed bss_suinfo_t;

typedef enum bss_data_type {
	BSS_NONE,
	BSS_PUBLIC_KEY = 0x1,
	BSS_LICENSE_KEY,
	BSS_DATA_UNKNOWN,
} bss_data_type_t;

static char *bdstr[] = {
	/* efuse section */
	[BSS_NONE] =		"No Command",
	[BSS_PUBLIC_KEY] =	"BSS PUBLIC KEY",
	[BSS_LICENSE_KEY] =	"BSS LICENSE KEY",
	[BSS_DATA_UNKNOWN] =	"BSS DATA UNKNOWN",
};

int bss_load_su_firmware(bss_suinfo_t *suinfo, uint8_t *hexbuf, uint32_t bufsz);
int bss_su_reset(bss_suinfo_t *suinfo);
int bss_l1_reset(void);
int bss_read_data(char *buf, int *count, bss_data_type_t type);
int bss_write_data(const char *buf, int count, bss_data_type_t type);
