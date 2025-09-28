/*
 * EdgeQ Inc.
 *
 * Raptor2 Data Structures for OTRX, TXU, PPU, SPU Debug/Logging
 */

#include "../include/raptor2_su_common.h"

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
	MEMTYPE_CLUSTER_ILM = MEMTYPE_CLUSTER_ILM_LOCAL | MEMTYPE_CLUSTER_ILM_CNOC,
	MEMTYPE_DDR,
	MEMTYPE_UNKNOWN = 31,
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
#define MEMTYPE_CLUSTER_ILM_BM			__MASK(MEMTYPE_CLUSTER_ILM)
#define MEMTYPE_DDR_BM				__MASK(MEMTYPE_DDR)
#define MEMTYPE_LMEM_BM				(MEMTYPE_LMEM_WB_BM | MEMTYPE_LMEM_NC_BM | \
						MEMTYPE_LMEM_WT_BM | MEMTYPE_LMEM_CNOC_BM | \
						MEMTYPE_LMEM_CNOC_B2_BM)

#define CMU1_OFFSET				0x8000

typedef struct membuf {
	bool in_use;
	unsigned char *buffer;
	u32 total_count;
	u32 written;
	loff_t moffs;
	void __iomem *ilm;
	void __iomem *dlm;
	void __iomem *lmem;
	void __iomem *lmem_b2;
	void __iomem *ddr;
	void __iomem *cluster_ilm;
} membuf_t;

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
	struct kobject *kobj_su;
	struct r2_kobj_entry *kobj_entry;
	u32 image_size;
	u32 membm;
	membuf_t membuf;
	u32 enabled;
};

struct r2_engine_info {
	r2_engine_t etype;
	const char *ename;
	int num_sus;
	struct kobject *kobj_eng;
	struct r2_engine_su *enghead;
};
