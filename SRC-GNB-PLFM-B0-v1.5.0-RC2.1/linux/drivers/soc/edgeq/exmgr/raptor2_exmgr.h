/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2023 EdgeQ, Inc.
 * Raptor2 Include file for EXMGR Ops
 */

#include "../include/raptor2_su_common.h"
#include "raptor2_su_dmadefs.h"
#include "cci.h"

#define EXMGR_MAX_PROCS			1
#define EXMGR_DEV_NAME			"r2devexmgr"
#define EXMGRDEV(m)			EXMGR_DEV_NAME #m
#define EXMGR_WAIT_FOR_EXCEPTION	0x54
#define EXMGR_NOWAIT_FOR_EXCEPTION	0x55
#define EXMGR_DUMP_MEMS_AND_EXIT	0x56
#define EXMGR_DUMP_RTOSMEM_AND_EXIT	0x57
#define EXMGR_FREESU_DMABUFS		0x58

#define RAPTOR2_EXMGR_IRQ		240

typedef enum memtype {
	MEMTYPE_ILM	= 0,
	MEMTYPE_DLM,
	MEMTYPE_LMEM_WB,
	MEMTYPE_LMEM_NC,
	MEMTYPE_LMEM_WT,
	MEMTYPE_LMEM_CNOC,
	MEMTYPE_LMEM_CNOC_B2,
	MEMTYPE_CLUSTER_ILM_LOCAL,
	MEMTYPE_CLUSTER_ILM_CNOC,
	MEMTYPE_DDR,
	MEMTYPE_UNKNOWN	= 31,
} memtype_t;

#define  __MASK(X)	(1UL<<(X))

#define MEMTYPE_ILM_BM				__MASK(MEMTYPE_ILM)
#define MEMTYPE_DLM_BM				__MASK(MEMTYPE_DLM)
#define MEMTYPE_LMEM_WB_BM			__MASK(MEMTYPE_LMEM_WB)
#define MEMTYPE_LMEM_NC_BM			__MASK(MEMTYPE_LMEM_NC)
#define MEMTYPE_LMEM_WT_BM			__MASK(MEMTYPE_LMEM_WT)
#define MEMTYPE_LMEM_CNOC_BM			__MASK(MEMTYPE_LMEM_CNOC)
#define MEMTYPE_LMEM_CNOC_B2_BM			__MASK(MEMTYPE_LMEM_CNOC_B2)
#define MEMTYPE_CLUSTER_ILM_LOCAL_BM		__MASK(MEMTYPE_CLUSTER_ILM_LOCAL)
#define MEMTYPE_CLUSTER_ILM_CNOC_BM		__MASK(MEMTYPE_CLUSTER_ILM_CNOC)
#define MEMTYPE_CLUSTER_ILM_BM			(MEMTYPE_CLUSTER_ILM_LOCAL_BM | \
						MEMTYPE_CLUSTER_ILM_CNOC_BM)
#define MEMTYPE_DDR_BM				__MASK(MEMTYPE_DDR)
#define MEMTYPE_LMEM_BM				(MEMTYPE_LMEM_WB_BM | MEMTYPE_LMEM_NC_BM | \
						MEMTYPE_LMEM_WT_BM | MEMTYPE_LMEM_CNOC_BM | \
						MEMTYPE_LMEM_CNOC_B2_BM)

typedef enum dumpmap {
	DUMP_SUMEM	= 0,
	DUMP_RTOSMEM	= 1,
} dumpmap_t;

#define DUMPTYPE_SUMEM		__MASK(DUMP_SUMEM)
#define DUMPTYPE_RTOSMEM	__MASK(DUMP_RTOSMEM)

typedef struct mapping {
	void __iomem *ilm;
	void __iomem *dlm;
	void __iomem *lmem;
	void __iomem *clilm;
	void __iomem *lmem_b2;
	void __iomem *ddr;
} mapping_t;

struct su_device {
	u64 ilm_offs;
	u32 ilmsz;
	u64 dlm_offs;
	u32 dlmsz;
	u64 cmu_offs;
	u32 cmusz;
	u64 cmu1_offs;
	u32 cmu1sz;
	u64 stmr_offs;
	u32 stmrsz;
	u64 dbg_offs;
	u32 dbgsz;
	u64 tmr_offs;
	u32 tmrsz;
	u64 per_offs;
	u32 persz;
	u64 eplic_offs;
	u32 eplicsz;
	u64 splic_offs;
	u32 splicsz;
};

struct su_memory {
	u64 loc_lmem_wb_addr;
	u32 loc_lmem_wb_sz;
	u64 loc_lmem_wt_addr;
	u32 loc_lmem_wt_sz;
	u64 loc_lmem_u_addr;
	u32 loc_lmem_u_sz;
	u64 lmem_addr;
	u32 lmem_sz;
	u64 edev_addr;
	u32 edev_sz;
};

struct r2_engine_su {
	char name[32];
	struct list_head node;
	r2_engine_t etype;
	struct su_device sudev; 
	struct su_memory sumem; 
	int su_index;
	u64 sudevaddr;
	u64 sudevsz;
	u64 cpuregaddr;
	u64 cpuregsz;
	u64 dcaddr;
	u64 dcsz;
	u64 icaddr;
	u64 icsz;
	u64 suinfoaddr;
	u64 suinfosz;
	u64 suconfaddr;
	u64 suconfsz;
	u64 suauaddr;
	u64 suausz;
	u64 sustatsaddr;
	u64 sustatssz;
	u64 sudbgaddr;
	u64 sudbgsz;
	u64 ddraddr;
	u64 ddrsz;
	u64 lmemaddr_wb_local;
	u64 lmemsz_wb_local;
	u64 lmemaddr_nc_local;
	u64 lmemsz_nc_local;
	u64 lmemaddr_wt_local;
	u64 lmemsz_wt_local;
	u64 lmemaddr_cpu;
	u64 lmemsz_cpu;
	u64 lmemb2_addr_cpu;
	u64 lmemb2_sz_cpu;
	u64 clilmaddr_local;
	u64 clilmsz_local;
	u64 clilmaddr_cnoc;
	u64 clilmsz_cnoc;
	u32 membm;
	mapping_t map;
	u32 enabled;
};

#define	ZEPHYR_OCM_START_ADDR		0x180008000
#define	ZEPHYR_OCM_SIZE			0x00028000
#define	ZEPHYR_OCM_UNCACHED_START_ADDR	0x180000000
#define	ZEPHYR_OCM_UNCACHED_SIZE	0x00008000
#define ZEPHYR_LMEM_ADDR		0x1C800000
#define ZEPHYR_LMEM_SIZE		0x00080000

#define UIO_DEV_NAME_LEN		64
#define UIO_DEV_NUM_MAPS		1
#define UIO_MAX_MAPS			5

struct uio_map {
	char name[UIO_DEV_NAME_LEN];
	uint64_t paddr;
	uint32_t size;
	uint64_t offset; /* vaddr - paddr */
};

struct device_uio_map {
	struct uio_map uio_map;
	void *vaddr;
	uint64_t offset; /* vaddr - paddr */
};

struct device_uio {
	char name[UIO_DEV_NAME_LEN];
	int uio_dev_num;
	int uio_fd;
	struct device_uio_map addr_map[UIO_DEV_NUM_MAPS];
};

typedef struct meminfo {
	uint64_t startaddr;
	uint64_t size;
	uint8_t *vptr;
} meminfo_t;

typedef struct zephyr_crash_info {
	cci_crash_dump_info_t crashinfo;
	meminfo_t ocm_info;
	uint8_t ocmbuf[ZEPHYR_OCM_SIZE];
	meminfo_t ocm_uc_info;
	uint8_t ocmbuf_uc[ZEPHYR_OCM_UNCACHED_SIZE];
	meminfo_t lmem_info;
	uint8_t lmembuf[ZEPHYR_LMEM_SIZE];
} zephyr_crash_info_t;

#define ZEPHYR_CRASHBUF_SIZE		0x100000
#define	ZEPHYR_CRASHBUF_REMAINING	(ZEPHYR_CRASHBUF_SIZE - sizeof(meminfo_t) -\
				sizeof(zephyr_crash_info_t) - 2*sizeof(struct device_uio))

struct r2_rtos_info {
	meminfo_t info;
	zephyr_crash_info_t z;
	struct device_uio ocm_uc_uio;
	struct device_uio ocm_uio;
	uint8_t reserved[ZEPHYR_CRASHBUF_REMAINING];
};
