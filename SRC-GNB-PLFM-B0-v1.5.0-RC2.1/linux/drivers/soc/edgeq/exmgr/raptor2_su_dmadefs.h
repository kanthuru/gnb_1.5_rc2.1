/* SPDX-License-Identifier: GPL-2.0
 * Copyright (c) 2023 EdgeQ, Inc.
 * Raptor2 defines for Exmgr
 * SU DMA Memory Buffers
 */

#define R2_MAX_ILM_SIZE		(64*1024)
#define R2_MAX_DLM_SIZE		(128*1024)
#define R2_MAX_CLILM_SIZE	(256*1024)
#define R2_MAX_LMEM_SIZE	(1408*1024)
#define R2_MAX_LMEM_B2_SIZE	(512*1024)

typedef struct su_dma_buf {
	uint64_t ilmsz;
	uint64_t dlmsz;
	uint64_t lmemsz;
	uint64_t lmemb2sz;
	uint64_t clilmsz;
	uint8_t reserved[PAGE_SIZE - 5 * sizeof(uint64_t)];
	uint8_t ilm[R2_MAX_ILM_SIZE];
	uint8_t dlm[R2_MAX_DLM_SIZE];
	uint8_t lmem[R2_MAX_LMEM_SIZE];
	uint8_t lmem_b2[R2_MAX_LMEM_B2_SIZE];
	uint8_t clilm[R2_MAX_CLILM_SIZE];
} su_dma_buf_t;

struct eqsu_membuf {
	int suid;
	int mbufsize;
	r2_engine_t etype;
	su_dma_buf_t *mbuf;
	struct r2_engine_su *engsu;
};

struct exmgr_mapping_access {
	int size;
	u64 virtaddr;
	uint32_t nprocs;
	spinlock_t lock;
	void *mptr;
};
