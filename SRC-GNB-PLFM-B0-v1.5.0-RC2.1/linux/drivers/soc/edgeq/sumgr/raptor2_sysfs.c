/*
 * EdgeQ Inc.
 *
 * Raptor2 Create sysfs for Micro-engines
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/stat.h>
#include "raptor2_sysfs.h"
#include "raptor2.h"
#include "raptor2_sysfs_ops.h"
#include "raptor2_riscv.h"
#include "../include/raptor2_bss_su.h"
#include <soc/edgeq/raptor2_rf_spi.h>

struct kobject *kobj_raptor2;
extern struct r2_engine_info r2_engine_info_table[];
static r2_chip_version_t chip_version;
EXPORT_SYMBOL_GPL(kobj_raptor2);

static uint32_t l1reset_in_progress;
static uint32_t secure_load_fmw;

const char *engstr[] = {
	[ENGINE_PPU] =          "ppu",
	[ENGINE_PPU_MXL] =      "ppu_mxl",
	[ENGINE_PPU_CRSS] =     "ppu_crss",
	[ENGINE_OTRX_HSU] =     "otrx_hsu",
	[ENGINE_OTRX_QSU] =     "otrx_qsu",
	[ENGINE_SPU] =          "spu",
	[ENGINE_ECPRI] =        "ecpri",
	[ENGINE_TXU] =          "txu",
	[ENGINE_IRING] =        "iring",
	//Always the last, all new entries before
	[ENGINE_NONE] =         "none",
};

static struct r2_engine_su *get_suinfo(struct kobject *kobj_su)
{
	const char *sp;
	int i, ret, found;
	unsigned long sunum;
	struct list_head *cur;
	struct r2_engine_info *enginfo;
	struct r2_engine_su *engsu, *suhead;

	found = 0;
	sp = kobj_su->name + 2;

	ret = kstrtoul(sp, 10, &sunum);
	if (ret != 0) {
		pr_err("%s: kstrtoul fails with err code %d\n", __func__, ret);
		return NULL;
	}

	for (i = 0; i < ENGINE_NONE; i++) {
		enginfo = &r2_engine_info_table[i];
		if (strcmp(kobj_su->parent->name, enginfo->ename) == 0) {
			suhead = enginfo->enghead;
			found = 1;
			break;
		}
	}

	if (found) {
		cur = &suhead->node;
		while (cur != NULL) {
			engsu = list_entry(cur, struct r2_engine_su, node);
			if (engsu->su_index == sunum)
				return engsu;

			if (cur->next == &suhead->node)
				return NULL;

			cur = cur->next;
		}
	}

	return NULL;
}

static memtype_t get_memtype(struct r2_engine_su *engsu, ulong addr)
{
	memtype_t mtype;

	if ((addr >= engsu->sudev.ilm_offs) &&
			(addr < (engsu->sudev.ilm_offs + engsu->sudev.ilmsz)))
		mtype = MEMTYPE_ILM;
	else if ((addr >= engsu->sudev.dlm_offs) &&
			(addr < (engsu->sudev.dlm_offs + engsu->sudev.dlmsz)))
		mtype = MEMTYPE_DLM;
	else if ((addr >= engsu->lmemaddr_wb_local) &&
			(addr < (engsu->lmemaddr_wb_local + engsu->lmemsz_wb_local)))
		mtype = MEMTYPE_LMEM_WB;
	else if ((addr >= engsu->lmemaddr_nc_local) &&
			(addr < (engsu->lmemaddr_nc_local + engsu->lmemsz_nc_local)))
		mtype = MEMTYPE_LMEM_NC;
	else if ((addr >= engsu->lmemaddr_wt_local) &&
			(addr < (engsu->lmemaddr_wt_local + engsu->lmemsz_wt_local)))
		mtype = MEMTYPE_LMEM_WT;
	else if ((addr >= engsu->lmemaddr_cpu) &&
			(addr < (engsu->lmemaddr_cpu + engsu->lmemsz_cpu)))
		mtype = MEMTYPE_LMEM_CNOC;
	else if ((addr >= engsu->lmemb2_addr_cpu) &&
			(addr < (engsu->lmemb2_addr_cpu + engsu->lmemb2_sz_cpu)))
		mtype = MEMTYPE_LMEM_CNOC_B2;
	else if ((addr >= engsu->clilmaddr_local) &&
			(addr < (engsu->clilmaddr_local + engsu->clilmsz_local)))
		mtype = MEMTYPE_CLUSTER_ILM_LOCAL;
	else if ((addr >= engsu->clilmaddr_cnoc) &&
			(addr < (engsu->clilmaddr_cnoc + engsu->clilmsz_cnoc)))
		mtype = MEMTYPE_CLUSTER_ILM_CNOC;
	else
		mtype = MEMTYPE_UNKNOWN;

	if (((mtype == MEMTYPE_CLUSTER_ILM_LOCAL) || (mtype == MEMTYPE_CLUSTER_ILM_CNOC)) &&
				(chip_version == R2_VERSION_A0))
		mtype = MEMTYPE_UNKNOWN;

	return mtype;
}

static char *get_lmem_addr_local(struct r2_engine_su *engsu, u64 baseaddr,
							u64 start_addr, u64 *moffs)
{
	u64 b2offs;
	unsigned char *mptr;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return NULL;
	}

	/* LMEM is contiguous via local access in B0 */
	if (chip_version == R2_VERSION_A0)
		b2offs = engsu->lmemb2_addr_cpu - engsu->lmemaddr_cpu;
	else
		b2offs = engsu->lmemsz_cpu;

	if ((engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM) && (baseaddr  >= (start_addr + b2offs))) {
		mptr = (unsigned char *)(engsu->membuf.lmem_b2);
		*moffs = baseaddr - (start_addr + b2offs);
	} else {
		mptr = (unsigned char *)(engsu->membuf.lmem);
		*moffs = baseaddr - start_addr;
	}

	return mptr;
}

static int writeval(struct r2_engine_su *engsu, unsigned int val, u64 baseaddr,
							int idx, memtype_t mtype)
{
	u64 moffs;
	unsigned char *mptr, byte;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	switch (mtype) {
	default:
		pr_err("%s: Unknown mem type\n", __func__);
		return -EBADTYPE;
	case MEMTYPE_ILM:
		mptr = (unsigned char *)(engsu->membuf.ilm);
		moffs = baseaddr - engsu->sudev.ilm_offs;
		break;
	case MEMTYPE_DLM:
		mptr = (unsigned char *)(engsu->membuf.dlm);
		moffs = baseaddr - engsu->sudev.dlm_offs;
		break;
	case MEMTYPE_LMEM_WB:
		mptr = get_lmem_addr_local(engsu, baseaddr, engsu->lmemaddr_wb_local, &moffs);
		break;
	case MEMTYPE_LMEM_NC:
		mptr = get_lmem_addr_local(engsu, baseaddr, engsu->lmemaddr_nc_local, &moffs);
		break;
	case MEMTYPE_LMEM_WT:
		mptr = get_lmem_addr_local(engsu, baseaddr, engsu->lmemaddr_wt_local, &moffs);
		break;
	case MEMTYPE_LMEM_CNOC:
		mptr = (unsigned char *)(engsu->membuf.lmem);
		moffs = baseaddr - engsu->lmemaddr_cpu;
		break;
	case MEMTYPE_LMEM_CNOC_B2:
		mptr = (unsigned char *)(engsu->membuf.lmem_b2);
		moffs = baseaddr - engsu->lmemb2_addr_cpu;
		break;
	case MEMTYPE_CLUSTER_ILM_LOCAL:
		mptr = (unsigned char *)(engsu->membuf.cluster_ilm);
		moffs = baseaddr - engsu->clilmaddr_local;
		break;
	case MEMTYPE_CLUSTER_ILM_CNOC:
		mptr = (unsigned char *)(engsu->membuf.cluster_ilm);
		moffs = baseaddr - engsu->clilmaddr_cnoc;
		break;
	}

	if (mptr == NULL) {
		pr_err("%s: mptr is NULL for mtype %d\n", __func__, mtype);
		return -ENOMEM;
	}

	if ((mtype == MEMTYPE_CLUSTER_ILM_LOCAL) ||
			(mtype == MEMTYPE_CLUSTER_ILM_CNOC)) {
		iowrite32be(val, mptr + moffs + idx);
	} else {
		byte = (unsigned char)(val & 0xff);
		writeb(byte, mptr + moffs + idx);
	}

	return 0;
}

static int get_bytestr_val(unsigned char *buf, unsigned char *bufend,
						memtype_t mtype, int *count)
{
	unsigned int val;
	int i = 0, ret, j = 0;
	unsigned char bytestr[9];

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	memset(bytestr, 0, sizeof(bytestr));

	switch (mtype) {
	default:
		pr_err("%s: Unknown mem type %d\n", __func__, mtype);
		return -EBADTYPE;
	case MEMTYPE_ILM:
	case MEMTYPE_DLM:
	case MEMTYPE_LMEM_WB:
	case MEMTYPE_LMEM_NC:
	case MEMTYPE_LMEM_WT:
	case MEMTYPE_LMEM_CNOC:
	case MEMTYPE_LMEM_CNOC_B2:
		memcpy(bytestr, buf, 2);
		*count = 2;
		break;
	case MEMTYPE_CLUSTER_ILM_LOCAL:
	case MEMTYPE_CLUSTER_ILM_CNOC:
		memset(bytestr, '0', sizeof(bytestr) - 1);
		while (j < 8) {
			if ((buf + i) >= bufend)
				break;

			if ((buf[i] == 0xA) || (buf[i] == 0xD) ||
				(buf[i] == 0x20) || (buf[i] == 0x0)) {

				++i;
				continue;
			}

			*(bytestr + j) = buf[i];
			++j;
			++i;
		}
		*count = i;
		break;
	}

	ret = kstrtou32(bytestr, 16, &val);
	if (ret < 0) {
		pr_err("%s: kstrtou32 fails for %s err code %d\n",
			__func__, bytestr, ret);
		return ret;
	}

	return val;
}

static int get_bcount(memtype_t mtype)
{
	int bcount;

	switch (mtype) {
	default:
	case MEMTYPE_ILM:
	case MEMTYPE_DLM:
	case MEMTYPE_LMEM_WB:
	case MEMTYPE_LMEM_NC:
	case MEMTYPE_LMEM_WT:
	case MEMTYPE_LMEM_CNOC:
	case MEMTYPE_LMEM_CNOC_B2:
		bcount = 1;
		break;
	case MEMTYPE_CLUSTER_ILM_LOCAL:
	case MEMTYPE_CLUSTER_ILM_CNOC:
		bcount = 4;
		break;
	}

	return bcount;
}

static int parse_codebuf(struct r2_engine_su *engsu)
{
	u64 baseaddr;
	memtype_t mtype;
	unsigned int val;
	unsigned char addrstr[16];
	unsigned char *buf, *bufend;
	int i, j, found, count, ret, bcount;

	if (engsu == NULL) {
		pr_err("%s: %s SU%d engsu is NULL\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		return -ENXIO;
	}

	buf = engsu->membuf.buffer;
	bufend = buf + engsu->membuf.total_count;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -ENXIO;
	}

	memset(addrstr, 0, sizeof(addrstr));

	i = 0;	// keep track of count from buf
	j = 0;	// keep track of byte within addr section
	found = 0;
	mtype = MEMTYPE_UNKNOWN;

	while (i < engsu->membuf.total_count) {
		if (buf[i] == '@') {
			memcpy(addrstr, buf + i + 1, 8);
			ret = kstrtoull(addrstr, 16, &baseaddr);
			if (ret != 0) {
				pr_err("%s: %s SU%d kstrtoull fails with err code %d\n", __func__,
					engstr[engsu->etype], engsu->su_index, ret);
				return ret;
			}

			mtype = get_memtype(engsu, baseaddr);
			if (mtype == MEMTYPE_UNKNOWN) {
				pr_err("%s: baseaddr: 0x%llx not found for %s SU%d\n",
					__func__, baseaddr,
					engstr[engsu->etype],
					engsu->su_index);
				return -EBADTYPE;
			}
			bcount = get_bcount(mtype);
			i += 9;
			j = 0;
			found = 1;
			//pr_info("found base addr 0x%llx\n", baseaddr);
		} else if ((buf[i] == 0xA) || (buf[i] == 0xD) ||
				(buf[i] == 0x20) || (buf[i] == 0x0))
			++i;
		else {
			count = 0;
			val = get_bytestr_val(buf + i, bufend, mtype, &count);
			if (val < 0) {
				pr_err("%s: %s SU%d get_bytestr_val failed baseaddr 0x%llx\n",
				__func__, engstr[engsu->etype], engsu->su_index, baseaddr);
				return val;
			}

			ret = writeval(engsu, val, baseaddr, j, mtype);
			if (ret < 0) {
				pr_err("%s: %s SU%d writeval failed for %x baseaddr 0x%llx\n",
				__func__, engstr[engsu->etype], engsu->su_index, val, baseaddr);
				return ret;
			}
			engsu->membuf.written += bcount;

			i += count;
			j += bcount;
		}
	}

	if (engsu->membuf.written == 0) {
		pr_err("%s: %s SU%d Code not written to ILM/DLM/LMEM.\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		return -EBADTYPE;
	}

	return 0;
}

static void unmap_su_clilm(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	iounmap(engsu->membuf.cluster_ilm);
	engsu->membuf.cluster_ilm = NULL;
}

static void unmap_su_ddr(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	iounmap(engsu->membuf.ddr);
	engsu->membuf.ddr = NULL;
}

static void unmap_su_lmem(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	if (!(engsu->membm & MEMTYPE_LMEM_CNOC_BM)) {
		pr_err("%s LMEM not supported for %s SU%d", __func__,
			engstr[engsu->etype], engsu->su_index);
		return;
	}

	iounmap(engsu->membuf.lmem);
	engsu->membuf.lmem = NULL;

	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM) {
		iounmap(engsu->membuf.lmem_b2);
		engsu->membuf.lmem_b2 = NULL;
	}
}

static void unmap_su_dlm(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	iounmap(engsu->membuf.dlm);
	engsu->membuf.dlm = NULL;
}

static void unmap_su_ilm(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	iounmap(engsu->membuf.ilm);
	engsu->membuf.ilm = NULL;
}

static void unmap_sumems(struct r2_engine_su *engsu)
{
	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return;
	}

	/* UnMap ILM */
	if (engsu->membm & MEMTYPE_ILM_BM)
		unmap_su_ilm(engsu);

	/* UnMap DLM */
	if (engsu->membm & MEMTYPE_DLM_BM)
		unmap_su_dlm(engsu);

	/* UnMap LMEM */
	if (engsu->membm & MEMTYPE_LMEM_CNOC_BM)
		unmap_su_lmem(engsu);

	/* UnMap DDR */
	if (engsu->membm & MEMTYPE_DDR_BM)
		unmap_su_ddr(engsu);

	/* UnMap Cluster ILM */
	if (engsu->membm & MEMTYPE_CLUSTER_ILM_BM)
		unmap_su_clilm(engsu);
}

static int map_su_clilm(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;

	mapaddr = engsu->clilmaddr_cnoc;
	mapsz = engsu->clilmsz_cnoc;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_CLUSTER_ILM_BM)) {
		pr_err("%s: %s SU%d Cluster ILM not supported\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return -EOPNOTSUPP;
	}

	engsu->membuf.cluster_ilm = ioremap(mapaddr, mapsz);
	if (engsu->membuf.cluster_ilm == NULL) {
		pr_err("%s: %s SU%d Unable to map LMEM addr 0x%llx, size 0x%llx\n", __func__,
		engstr[engsu->etype], engsu->su_index, engsu->clilmaddr_cnoc, engsu->clilmsz_cnoc);
		return -ENOMEM;
	}

	return 0;
}

static int map_su_lmem(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;

	mapaddr = engsu->lmemaddr_cpu;
	mapsz = engsu->lmemsz_cpu;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_LMEM_CNOC_BM)) {
		pr_err("%s: LMEM not supported for %s SU%d\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return -EOPNOTSUPP;
	}

	engsu->membuf.lmem = ioremap(mapaddr, mapsz);
	if (engsu->membuf.lmem == NULL) {
		pr_err("%s: %s SU%d Unable to map LMEM addr 0x%llx, size 0x%llx\n", __func__,
		engstr[engsu->etype], engsu->su_index, engsu->lmemaddr_cpu, engsu->lmemsz_cpu);
		return -ENOMEM;
	}

	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM) {
		mapaddr = engsu->lmemb2_addr_cpu;
		mapsz = engsu->lmemb2_sz_cpu;
		engsu->membuf.lmem_b2 = ioremap(mapaddr, mapsz);
		if (engsu->membuf.lmem_b2 == NULL) {
			pr_err("%s: %s SU%d Unable to map LMEM_B2 addr 0x%llx, size 0x%llx\n",
			__func__, engstr[engsu->etype], engsu->su_index, engsu->lmemb2_addr_cpu,
			engsu->lmemb2_sz_cpu);
			return -ENOMEM;
		}
	}

	return 0;
}

static int map_su_dlm(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_DLM_BM)) {
		pr_err("%s: %s SU%d DLM not supported\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		return -EOPNOTSUPP;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.dlm_offs;
	mapsz = engsu->sudev.dlmsz;

	engsu->membuf.dlm = ioremap(mapaddr, mapsz);
	if (engsu->membuf.dlm == NULL) {
		pr_err("%s: %s SU%d DLM map fail base=0x%llx, offs=0x%llx, msz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, engsu->sudevaddr,
			engsu->sudev.dlm_offs, engsu->sudev.dlmsz);
		return -ENOMEM;
	}

	return 0;
}

static int map_su_ilm(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_ILM_BM)) {
		pr_err("%s: %s SU%d ILM not supported\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return -EOPNOTSUPP;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.ilm_offs;
	mapsz = engsu->sudev.ilmsz;

	engsu->membuf.ilm = ioremap(mapaddr, mapsz);
	if (engsu->membuf.ilm == NULL) {
		pr_err("%s: %s SU%d ILM map fail base=0x%llx, offs=0x%llx, sz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, engsu->sudevaddr,
			engsu->sudev.ilm_offs, engsu->sudev.ilmsz);
		return -ENOMEM;
	}

	return 0;
}

static int map_sumems(struct r2_engine_su *engsu)
{
	int ret;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	/* Map ILM */
	if (engsu->membm & MEMTYPE_ILM_BM) {
		ret = map_su_ilm(engsu);
		if (ret < 0)
			return ret;
	}

	/* Map DLM */
	if (engsu->membm & MEMTYPE_DLM_BM) {
		ret = map_su_dlm(engsu);
		if (ret < 0)
			return ret;
	}

	/* Map LMEM */
	if (engsu->membm & MEMTYPE_LMEM_CNOC_BM) {
		ret = map_su_lmem(engsu);
		if (ret < 0)
			return ret;
	}

	/* Map Cluster ILM */
	if (engsu->membm & MEMTYPE_CLUSTER_ILM_BM) {
		ret = map_su_clilm(engsu);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int deinit_membuf(membuf_t *membuf)
{
	if (membuf == NULL) {
		pr_err("%s: membuf is NULL\n", __func__);
		return -ENXIO;
	}

	membuf->in_use = 0;
	membuf->total_count = 0;
	membuf->written = 0;

	if (membuf->buffer) {
		kfree(membuf->buffer);
		membuf->buffer = NULL;
	}

	return 0;
}

static int init_membuf(membuf_t *membuf, size_t size)
{
	if (membuf == NULL) {
		pr_err("%s: membuf is NULL\n", __func__);
		return -ENXIO;
	}

	if (size == 0) {
		pr_err("%s: Invalid allocation size for membuf\n", __func__);
		return -ENOPARAM;
	}

	membuf->buffer = (unsigned char *)kmalloc(size, GFP_KERNEL);
	if (membuf->buffer == NULL) {
		//pr_err("%s: kmalloc failed for membuf\n", __func__);
		return -ENOMEM;
	}
	memset(membuf->buffer, 0, size);
	membuf->in_use = 1;
	membuf->total_count = 0;
	membuf->written = 0;

	return 0;
}

static int check_su_enable(struct r2_engine_su *engsu)
{
	int ret;
	u64 mapaddr;
	u32 ccrval, mapsz;
	void __iomem *cmu;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -EINVAL;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	ccrval = readl(cmu + CMU_CORE_CONFIG_REG);

	if (ccrval & SU_ASSERT_BIT)
		ret = 0;
	else
		ret = 1;

	iounmap(cmu);
	return ret;
}

static int enable_su_memory(struct r2_engine_su *engsu)
{
	u32 mapsz;
	u64 mapaddr;
	void __iomem *cmu;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -ENXIO;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	/* Enable Memories for SU */
	if (engsu->etype == ENGINE_TXU)
		writel(0xC0008815, cmu + CMU_CORE_CONFIG_REG);
	else
		writel(0x00008BDA, cmu + CMU_CORE_CONFIG_REG);

	iounmap(cmu);

	return 0;
}

static int populate_bss_suinfo(bss_suinfo_t *info, struct r2_engine_su *engsu)
{
	if (info == NULL) {
		pr_err("%s: info is NULL\n", __func__);
		return -EINVAL;
	}

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -EINVAL;
	}

	memset(info, 0, sizeof(bss_suinfo_t));

	info->etype = engsu->etype;
	info->suid = engsu->su_index;
	info->membm = engsu->membm;
	info->sudev_base = engsu->sudevaddr;
	info->cmuoffs =  engsu->sudev.cmu_offs;

	if (engsu->membm & MEMTYPE_ILM_BM) {
		info->ilm.addr = engsu->sudev.ilm_offs;
		info->ilm.size = engsu->sudev.ilmsz;
		info->ilm.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_DLM_BM) {
		info->dlm.addr = engsu->sudev.dlm_offs;
		info->dlm.size = engsu->sudev.dlmsz;
		info->dlm.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_LMEM_WB_BM) {
		info->lmem_wb.addr = engsu->lmemaddr_wb_local;
		info->lmem_wb.size = engsu->lmemsz_wb_local;
		info->lmem_wb.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_LMEM_NC_BM) {
		info->lmem_nc.addr = engsu->lmemaddr_nc_local;
		info->lmem_nc.size = engsu->lmemsz_nc_local;
		info->lmem_nc.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_LMEM_WT_BM) {
		info->lmem_wt.addr = engsu->lmemaddr_wt_local;
		info->lmem_wt.size = engsu->lmemsz_wt_local;
		info->lmem_wt.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_LMEM_CNOC_BM) {
		info->lmem_cnoc.addr = engsu->lmemaddr_cpu;
		info->lmem_cnoc.size = engsu->lmemsz_cpu;
		info->lmem_cnoc.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM) {
		info->lmem_cnoc_b2.addr = engsu->lmemb2_addr_cpu;
		info->lmem_cnoc_b2.size = engsu->lmemb2_sz_cpu;
		info->lmem_cnoc_b2.offs = 0x0;
	}

	if (engsu->membm & MEMTYPE_CLUSTER_ILM_BM) {
		info->clilm_local.addr = engsu->clilmaddr_local;
		info->clilm_local.size = engsu->clilmsz_local;
		info->clilm_local.offs = 0x0;

		info->clilm_cnoc.addr = engsu->clilmaddr_cnoc;
		info->clilm_cnoc.size = engsu->clilmsz_cnoc;
		info->clilm_cnoc.offs = 0x0;
	}

	return 0;
}

static ssize_t raptor2_sysfs_image_write(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	int ret;
	bss_suinfo_t *bss_suinfo;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	/* stash in temporary buffer */
	if (engsu->membuf.in_use == 0) {
		ret = init_membuf(&engsu->membuf, MAX_IMAGE_SIZE);
		if (ret < 0)
			return ret;
	}

	memmove(engsu->membuf.buffer + offs, buf, count);
	engsu->membuf.total_count += count;

	if (engsu->image_size) {
		if (engsu->membuf.total_count < engsu->image_size)
			return count;
	} else {
		/* return if more data expected from upper layer */
		if (f->remaining > count)
			return count;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (!secure_load_fmw) {
		ret = map_sumems(engsu);
		if (ret < 0)
			goto out;

		ret = parse_codebuf(engsu);
		if (ret < 0)
			goto out;

		unmap_sumems(engsu);
	} else {

		bss_suinfo = kmalloc(sizeof(bss_suinfo_t), GFP_KERNEL);
		if (bss_suinfo == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		populate_bss_suinfo(bss_suinfo, engsu);

		ret = bss_load_su_firmware(bss_suinfo, engsu->membuf.buffer,
				engsu->membuf.total_count);
		if (ret < 0) {
			pr_err("%s: bss_load_su_firmware failed for %s SU%d\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out1;
		}
	}

	ret = count;
out1:
	if (secure_load_fmw)
		kfree(bss_suinfo);
out:
	deinit_membuf(&engsu->membuf);
	engsu->image_size = 0;

	return ret;
}

static ssize_t raptor2_sysfs_image_size_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	sprintf(buf, "%d", engsu->image_size);

	return strlen(buf);
}

static ssize_t raptor2_sysfs_image_size_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);

	ret = kstrtoul(buf, 10, &val);
	if (ret != 0) {
		pr_err("%s: %s SU%d kstrtoul fails with err code %d\n",
			__func__, engstr[engsu->etype], engsu->su_index, ret);
		return ret;
	}

	engsu->image_size = val;

	return count;
}

static ssize_t raptor2_sysfs_reset_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	u64 mapaddr;
	u32 mapsz, ccrval;
	void __iomem *cmu;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	ccrval = readl(cmu + CMU_CORE_CONFIG_REG);
	sprintf(buf, "CORE_RESET_ENABLE:%d\n", (ccrval & 0x00000800) ? 1 : 0);

	iounmap(cmu);
	return strlen(buf);
}

static int reset_su_legacy(struct r2_engine_su *engsu, unsigned long val)
{
	u64 mapaddr;
	u32 mapsz, ccrval;
	void __iomem *cmu;

	if (engsu == NULL) {
		pr_err("%s: engsu is NULL\n", __func__);
		return -EINVAL;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	if (engsu->etype == ENGINE_TXU) {
		writel(0xC0008015, cmu + CMU_CORE_CONFIG_REG);
		ccrval = 0xC0008815;
	} else {
		writel(0x000083DA, cmu + CMU_CORE_CONFIG_REG);
		ccrval = 0x00008BDA;
	}

	if (!val) {
		/* Write to reset vector register */
		writel(0x0, cmu + CMU_RESET_VECTOR_BASE_ADDR);
		if (engsu->etype == ENGINE_TXU) {
			writel(0xC0008015, cmu + CMU_CORE_CONFIG_REG);
			ccrval = 0xC0008415;
		} else {
			writel(0x000083D5, cmu + CMU_CORE_CONFIG_REG);
			ccrval = 0x000087D5;
		}
	}
	/* else the SU is in reset */

	writel(ccrval, cmu + CMU_CORE_CONFIG_REG);

	iounmap(cmu);

	return 0;
}

static ssize_t raptor2_sysfs_reset_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	bss_suinfo_t *bss_suinfo;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);

	ret = kstrtoul(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: %s SU%d kstrtoul fails with err code %d\n",
				__func__, engstr[engsu->etype], engsu->su_index, ret);
		return ret;
	}

	if (!secure_load_fmw) {
		ret = reset_su_legacy(engsu, val);
		if (ret < 0) {
			pr_err("%s: reset_su_legacy failed with %d\n",
					__func__, ret);
			goto out;
		}

	} else {
		bss_suinfo = kmalloc(sizeof(bss_suinfo_t), GFP_KERNEL);
		if (bss_suinfo == NULL) {
			ret = -ENOMEM;
			goto out;
		}

		memset(bss_suinfo, 0, sizeof(bss_suinfo_t));
		bss_suinfo->suid = engsu->su_index;
		bss_suinfo->etype = engsu->etype;
		bss_suinfo->sudev_base = engsu->sudevaddr;
		bss_suinfo->cmuoffs =  engsu->sudev.cmu_offs;
		bss_suinfo->rstval = (uint8_t)val;

		ret = bss_su_reset(bss_suinfo);
		if (ret < 0) {
			pr_err("%s %s SU%d bss_su_reset failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out1;
		}
	}

	ret = count;

out1:
	if (secure_load_fmw)
		kfree(bss_suinfo);
out:
	return ret;
}

static ssize_t raptor2_sysfs_info_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	int ret;
	struct r2_engine_su *engsu;
	char info[SU_INFO_SIZE + 1];

	if (chip_version == R2_VERSION_A0) {
		pr_err("%s: Call not supported for version A0\n", __func__);
		return -EOPNOTSUPP;
	}

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_DLM_BM)) {
		pr_err("%s: DLM not supported for SU\n", __func__);
		return -EOPNOTSUPP;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	memset(info, 0, sizeof(info));
	memcpy_fromio(info, engsu->membuf.dlm + SU_DLM_INFO_OFFS, SU_INFO_SIZE);
	sprintf(buf, "%s\n", info);
	unmap_su_dlm(engsu);
	ret = strlen(buf);
out:
	return ret;
}

static ssize_t raptor2_sysfs_info_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_conf_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	int ret;
	struct r2_engine_su *engsu;
	char conf[SU_CONF_SIZE + 1];

	if (chip_version == R2_VERSION_A0) {
		pr_err("%s: Call not supported for version A0\n", __func__);
		return -EOPNOTSUPP;
	}

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_DLM_BM)) {
		pr_err("%s: %s SU%d DLM not supported\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		return -EOPNOTSUPP;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	memset(conf, 0, sizeof(conf));
	memcpy_fromio(conf, engsu->membuf.dlm + SU_DLM_CONF_OFFS, SU_CONF_SIZE);
	sprintf(buf, "%s\n", conf);
	unmap_su_dlm(engsu);
	ret = strlen(buf);
out:
	return ret;
}

static ssize_t raptor2_sysfs_conf_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}

static void add_sprintf_regval(void __iomem *regbase, char *buf, char *regstr,
								u64 regoffs, rtype_t rt)
{
	if (regbase == NULL) {
		pr_err("%s: cmux is NULL\n", __func__);
		return;
	}

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return;
	}

	if (regstr == NULL) {
		pr_err("%s: regstr is NULL\n", __func__);
		return;
	}

	sprintf(buf + strlen(buf), regstr);
	switch (rt) {
	case rt_b:
		sprintf(buf + strlen(buf), "(0x%x):0x%02x\n",
				(u32)(regoffs), (u32)(readb(regbase + regoffs)));
		break;
	case rt_w:
		sprintf(buf + strlen(buf), "(0x%x):0x%x\n",
				(u32)(regoffs), (u32)(readw(regbase + regoffs)));
		break;
	case rt_l:
		sprintf(buf + strlen(buf), "(0x%x):0x%x\n",
				(u32)(regoffs), (u32)(readl(regbase + regoffs)));
		break;
	case rt_q:
		sprintf(buf + strlen(buf), "(0x%llx):0x%llx\n",
				regoffs, readq(regbase + regoffs));
		break;
	}
}

static ssize_t raptor2_sysfs_regs_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	u32 mapsz;
	u64 mapaddr;
	struct r2_engine_su *engsu;
	void __iomem *cmu, *cmu1, *stmr, *tmr;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
				__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	sprintf(buf, "CMU0 Regs\n");
	add_sprintf_regval(cmu, buf, "CMU0 TR", CMU_TRIGGER_REG, rt_q);
	add_sprintf_regval(cmu, buf, "CMU0 Config", CMU_CORE_CONFIG_REG, rt_q);
	add_sprintf_regval(cmu, buf, "CMU0 Status", CMU_CORE_STATUS_REG, rt_q);

	if (chip_version == R2_VERSION_B0) {
		sprintf(buf + strlen(buf), "\n\n");
		cmu1 = cmu + CMU1_OFFSET;
		sprintf(buf + strlen(buf), "CMU1 Regs\n");
		add_sprintf_regval(cmu1, buf, "CMU1 TR", CMU1_TRIGGER_REG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 CCR", CMU1_CORE_CONFIG_REG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 EV", CMU1_EVENT_REG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 WC_ADDR1_CFG", CMU1_WC_ADDR1_CFG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 WC_ADDR2_CFG", CMU1_WC_ADDR2_CFG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 WC_ADDR3_CFG", CMU1_WC_ADDR3_CFG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 WC_ADDR4_CFG", CMU1_WC_ADDR4_CFG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG1", CMU1_ADDRM_CFG1, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG2", CMU1_ADDRM_CFG2, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG3", CMU1_ADDRM_CFG3, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG4", CMU1_ADDRM_CFG4, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG5", CMU1_ADDRM_CFG5, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 ADDRM_CFG6", CMU1_ADDRM_CFG6, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1 GPIO CFG", CMU1_GPIO_CFG, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_HW_STMR_SNP0_ST", CMU1_HW_STMR_SNP0_ST, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_HW_STMR_SNP1_ST", CMU1_HW_STMR_SNP1_ST, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_STATUS", CMU1_STATUS, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_WR_COL_ST", CMU1_WR_COL_ST, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_STMR_SNP0_ST", CMU1_STMR_SNP0_ST, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_STMR_SNP1_ST", CMU1_STMR_SNP1_ST, rt_q);
		add_sprintf_regval(cmu1, buf, "CMU1_ADDR_MISM_ST", CMU1_ADDR_MISM_ST, rt_q);
	}

	iounmap(cmu);

	if (chip_version == R2_VERSION_A0) {
		mapaddr = engsu->sudevaddr + engsu->sudev.stmr_offs;
		mapsz = engsu->sudev.stmrsz;
		stmr = ioremap(mapaddr, mapsz);
		if (stmr == NULL) {
			pr_err("%s:%s SU%d stmr ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
				__func__, engstr[engsu->etype], engsu->su_index,
				engsu->sudevaddr + engsu->sudev.stmr_offs, engsu->sudev.stmrsz);
			return -ENOMEM;
		}

		sprintf(buf + strlen(buf), "\n");
		sprintf(buf + strlen(buf), "STMR Regs\n");
		add_sprintf_regval(cmu1, buf, "RSIT0", STMR_REQ_INT0_TIME, rt_l);
		add_sprintf_regval(cmu1, buf, "RSIT1", STMR_REQ_INT1_TIME, rt_l);
		add_sprintf_regval(cmu1, buf, "RSIT2", STMR_REQ_INT2_TIME, rt_l);
		add_sprintf_regval(cmu1, buf, "RSIT3", STMR_REQ_INT3_TIME, rt_l);
		add_sprintf_regval(cmu1, buf, "CST", STMR_CURR_SYS_TIME, rt_l);

		iounmap(stmr);

		mapaddr = engsu->sudevaddr + engsu->sudev.tmr_offs;
		mapsz = engsu->sudev.tmrsz;
		tmr = ioremap(mapaddr, mapsz);
		if (tmr == NULL) {
			pr_err("%s: %s SU%d tmr ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
					__func__, engstr[engsu->etype], engsu->su_index,
					engsu->sudevaddr + engsu->sudev.tmr_offs, mapsz);
			return -ENOMEM;
		}

		sprintf(buf + strlen(buf), "\n");
		sprintf(buf + strlen(buf), "TMR Regs\n");
		add_sprintf_regval(cmu1, buf, "MTIME0", TMR_MTIME_REG0, rt_l);
		add_sprintf_regval(cmu1, buf, "MTIME1", TMR_MTIME_REG1, rt_l);
		add_sprintf_regval(cmu1, buf, "MCR0", TMR_MTIME_CMP_REG0, rt_l);
		add_sprintf_regval(cmu1, buf, "MCR1", TMR_MTIME_CMP_REG1, rt_l);
		iounmap(tmr);
	}

	return strlen(buf);
}

static ssize_t raptor2_sysfs_regs_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_au_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_au_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}


static ssize_t raptor2_sysfs_mem_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	return count;
}

static ssize_t raptor2_sysfs_mem_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	u32 mapsz;
	u64 mapaddr;
	void __iomem *cmu;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
				__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	sprintf(buf, "CCR:0x%x\n", readl(cmu + CMU_CORE_CONFIG_REG));
	iounmap(cmu);
	return strlen(buf);
}

static ssize_t raptor2_sysfs_ilm_read(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (offs >= engsu->sudev.ilmsz) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds ilmsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, engsu->sudev.ilmsz);
		return -ERANGE;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_ilm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_ilm failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	ret = count;
	if (offs + count > engsu->sudev.ilmsz)
		ret = engsu->sudev.ilmsz - offs;

	memcpy_fromio(buf, engsu->membuf.ilm + offs, ret);

	unmap_su_ilm(engsu);
out:
	return ret;
}

static ssize_t raptor2_sysfs_ilm_write(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	int ret;
	u32 writecount;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (offs >= engsu->sudev.ilmsz) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds ilmsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, engsu->sudev.ilmsz);
		deinit_membuf(&engsu->membuf);
		return -ERANGE;
	}

	if (engsu->membuf.in_use == 0) {	//stash in temporary buffer
		ret = init_membuf(&engsu->membuf, engsu->sudev.ilmsz);
		if (ret != 0)
			return ret;

		engsu->membuf.moffs = offs;
	}

	memmove(engsu->membuf.buffer + offs, buf, count);
	engsu->membuf.total_count += count;

	if (engsu->image_size) {
		if (engsu->membuf.total_count < engsu->image_size)
			return count;
	} else {
		/* return if more data expected from upper layer */
		if (f->remaining > count)
			return count;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_ilm(engsu);
	if (ret < 0) {
		pr_err("%s: map_su_ilm failed for SU\n", __func__);
		goto out;
	}

	if (engsu->membuf.total_count > engsu->sudev.ilmsz)
		writecount = engsu->sudev.ilmsz;
	else
		writecount = engsu->membuf.total_count;

	memcpy_toio(engsu->membuf.ilm + engsu->membuf.moffs,
			engsu->membuf.buffer + engsu->membuf.moffs, writecount);
	pr_info("%s: %s SU%d Wrote %d bytes to ILM\n",
		__func__, engstr[engsu->etype], engsu->su_index, writecount);

	ret = count;
	unmap_su_ilm(engsu);

out:
	deinit_membuf(&engsu->membuf);
	engsu->image_size = 0;

	return ret;
}

static ssize_t raptor2_sysfs_ilm0_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_ilm(engsu);
	if (ret < 0) {
		pr_err("%s: map_su_dlm failed for SU\n", __func__);
		goto out;
	}

	memset_io(engsu->membuf.ilm, 0, engsu->sudev.ilmsz);
	ret = count;
	unmap_su_ilm(engsu);

out:
	return ret;
}

static ssize_t raptor2_sysfs_dlm_read(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (offs >= engsu->sudev.dlmsz) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds dlmsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, engsu->sudev.dlmsz);
		return -ERANGE;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	ret = count;
	if (offs + count > engsu->sudev.dlmsz)
		ret = engsu->sudev.dlmsz - offs;

	memcpy_fromio(buf, engsu->membuf.dlm + offs, ret);
	unmap_su_dlm(engsu);
out:
	return ret;
}

static ssize_t raptor2_sysfs_dlm_write(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	int ret;
	u32 writecount;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (offs + count - 1 >= engsu->sudev.dlmsz) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds dlmsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, engsu->sudev.dlmsz);
		deinit_membuf(&engsu->membuf);
		return -ERANGE;
	}

	if (engsu->membuf.in_use == 0) {	//stash in temporary buffer
		ret = init_membuf(&engsu->membuf, engsu->sudev.dlmsz);
		if (ret != 0)
			return ret;

		engsu->membuf.moffs = offs;
	}

	memmove(engsu->membuf.buffer + offs, buf, count);
	engsu->membuf.total_count += count;

	if (f->remaining > count)	//return if more data expected from upper layer
		return count;

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out;
		}
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	writecount = engsu->membuf.total_count;
	memcpy_toio(engsu->membuf.dlm + engsu->membuf.moffs,
			engsu->membuf.buffer + engsu->membuf.moffs, writecount);
	ret = count;
	unmap_su_dlm(engsu);

out:
	deinit_membuf(&engsu->membuf);

	return ret;
}

static ssize_t raptor2_sysfs_dlm0_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	memset_io(engsu->membuf.dlm, 0, engsu->sudev.dlmsz);
	ret = count;
	unmap_su_dlm(engsu);

out:
	engsu->image_size = 0;
	return ret;
}

static ssize_t raptor2_sysfs_lmem_read(struct file *f, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	u32 b2offs;
	int ret, b1sz, b2sz;
	u64  max_lmem_range;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	max_lmem_range = engsu->lmemsz_cpu;
	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM)
		max_lmem_range += engsu->lmemb2_sz_cpu;

	if (offs >= max_lmem_range) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds max_lmem_range=0x%llx\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, max_lmem_range);
		return -ERANGE;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_lmem(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_lmem failed\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	b2sz = 0;

	if (offs < engsu->lmemsz_cpu) {
		b1sz = engsu->lmemsz_cpu - offs;
		if (count < b1sz)
			b1sz = count;
		else
			b2sz = count - b1sz;
		b2offs = 0;
		if (b2sz > engsu->lmemb2_sz_cpu)
			b2sz = engsu->lmemb2_sz_cpu;
	} else {
		b1sz = 0;
		b2offs = offs - engsu->lmemsz_cpu;
		b2sz = engsu->lmemb2_sz_cpu - b2offs;
		if (count < b2sz)
			b2sz = count;
	}

	if (b1sz > 0) {
		memcpy_fromio(buf, engsu->membuf.lmem + offs, b1sz);
		ret = b1sz;
	}

	if ((b2sz > 0) && (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM))  {
		memcpy_fromio(buf + ret, engsu->membuf.lmem_b2 + b2offs, b2sz);
		ret += b2sz;
	}

	unmap_su_lmem(engsu);
out:
	return ret;
}

static ssize_t raptor2_sysfs_lmem_write(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	loff_t moffs;
	u32 b2offs, totcnt;
	u64 max_lmem_range;
	int ret, b1sz, b2sz;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	max_lmem_range = engsu->lmemsz_cpu;
	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM)
		max_lmem_range += engsu->lmemb2_sz_cpu;

	if (offs + count - 1 >= max_lmem_range) {
		pr_err("%s:%s SU%d offs 0x%llx out of bounds lmemsz=0x%llx\n",
				__func__, engstr[engsu->etype], engsu->su_index,
				offs, max_lmem_range);
		deinit_membuf(&engsu->membuf);
		return -ERANGE;
	}

	if (engsu->membuf.in_use == 0) {	//stash in temporary buffer
		ret = init_membuf(&engsu->membuf, max_lmem_range);
		if (ret != 0)
			return ret;

		engsu->membuf.moffs = offs;
	}

	memmove(engsu->membuf.buffer + offs, buf, count);
	engsu->membuf.total_count += count;

	if (f->remaining > count)	//return if more data expected from upper layer
		return count;

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out;
		}
	}

	ret = map_su_lmem(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_lmem failed\n", __func__,
				engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	b2sz = 0;
	moffs = engsu->membuf.moffs;
	totcnt = engsu->membuf.total_count;

	if (moffs < engsu->lmemsz_cpu) {
		b1sz = engsu->lmemsz_cpu - moffs;
		if (totcnt < b1sz)
			b1sz = totcnt;
		else
			b2sz = totcnt - b1sz;
		b2offs = 0;
		if (b2sz > engsu->lmemb2_sz_cpu)
			b2sz = engsu->lmemb2_sz_cpu;
	} else {
		b1sz = 0;
		b2offs = moffs - engsu->lmemsz_cpu;
		b2sz = engsu->lmemb2_sz_cpu - b2offs;
		if (totcnt < b2sz)
			b2sz = totcnt;
	}

	if (b1sz > 0) {
		memcpy_toio(engsu->membuf.lmem + moffs,
				engsu->membuf.buffer + moffs, b1sz);
		ret = b1sz;
	}

	if ((b2sz > 0) && (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM))  {
		memcpy_toio(engsu->membuf.lmem_b2 + b2offs,
				engsu->membuf.buffer + moffs + ret, b2sz);
	}

	ret = count;
	unmap_su_lmem(engsu);
out:
	deinit_membuf(&engsu->membuf);

	return ret;
}

static ssize_t raptor2_sysfs_lmem0_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_lmem(engsu);
	if (ret < 0) {
		pr_err("%s: map_su_lmem failed for SU\n", __func__);
		goto out;
	}

	memset_io(engsu->membuf.lmem, 0, engsu->lmemsz_cpu);

	if (engsu->membm & MEMTYPE_LMEM_CNOC_B2_BM)
		memset_io(engsu->membuf.lmem_b2, 0, engsu->lmemb2_sz_cpu);

	unmap_su_lmem(engsu);
	ret = count;
out:
	engsu->image_size = 0;
	return ret;
}

static ssize_t raptor2_sysfs_clilm_read(struct file *f, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	u32 val;
	int i, ret;
	u64 max_clilm_range;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	max_clilm_range = engsu->clilmsz_cnoc;

	if (offs + count - 1 >= max_clilm_range) {
		pr_err("%s:%s SU%d offs 0x%llx + count 0x%lx oob max_clilm_range=0x%llx\n",
			__func__, engstr[engsu->etype], engsu->su_index,
			offs, count, max_clilm_range);
		return -ERANGE;
	}

	if (count % 4) {
		pr_err("%s: number of bytes to be read should be multiple of 4, count=%ld\n",
				__func__, count);
		return -EINVAL;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_clilm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_clilm failed\n", __func__,
			engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	ret = count;
	for (i = 0; i < ret/4; i += 4) {
		val = readl(engsu->membuf.cluster_ilm + offs + i);
		memcpy(buf + i, (u8 *)(&val), sizeof(val));
	}

	unmap_su_lmem(engsu);
out:
	return ret;
}

static ssize_t raptor2_sysfs_clilm_write(struct file *f, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t offs, size_t count)
{
	u32 val;
	int i, ret;
	size_t totcnt;
	loff_t moffs;
	u64 max_clilm_range;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	max_clilm_range = engsu->clilmsz_cnoc;

	if (offs + count - 1 >= max_clilm_range) {
		pr_err("%s:%s SU%d offs 0x%llx + count 0x%lx oob clilmsz=0x%llx\n",
				__func__, engstr[engsu->etype], engsu->su_index,
				offs, count, max_clilm_range);
		deinit_membuf(&engsu->membuf);
		return -ERANGE;
	}

	if (engsu->membuf.in_use == 0) {	//stash in temporary buffer
		ret = init_membuf(&engsu->membuf, max_clilm_range);
		if (ret != 0)
			return ret;

		engsu->membuf.moffs = offs;
	}

	memmove(engsu->membuf.buffer + offs, buf, count);
	engsu->membuf.total_count += count;

	if (f->remaining > count)	//return if more data expected from upper layer
		return count;

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out;
		}
	}

	ret = map_su_clilm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_clilm failed\n", __func__,
				engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	moffs = engsu->membuf.moffs;
	totcnt = engsu->membuf.total_count;

	if (totcnt % 4) {
		pr_err("%s: number of bytes to be written should be multiple of 4, count=%ld\n",
				__func__, totcnt);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < totcnt/4; i += 4) {
		memcpy((u8 *)(&val), engsu->membuf.buffer + moffs + i, sizeof(val));
		writel(val, engsu->membuf.cluster_ilm + moffs + i);
	}

	ret = count;
	unmap_su_lmem(engsu);
out:
	deinit_membuf(&engsu->membuf);

	return ret;
}

static int clilm_clear(void __iomem *mbuf, ssize_t sz)
{
	int i = 0;

	if (mbuf == NULL) {
		pr_err("%s: mbuf is NULL\n", __func__);
		return -1;
	}

	if ((sz <= 0) || (sz % 4)) {
		pr_err("%s: invalid value of sz=%ld\n", __func__, sz);
		return -1;
	}

	for (i = 0; i < sz/4; i += 4)
		writel(0x0, mbuf + i);

	return 0;
}

static ssize_t raptor2_sysfs_clilm0_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_clilm(engsu);
	if (ret < 0) {
		pr_err("%s: map_su_clilm failed for SU\n", __func__);
		return ret;
	}

	ret = clilm_clear(engsu->membuf.cluster_ilm, engsu->clilmsz_cnoc);
	if (ret != 0) {
		pr_err("%s: clilm_clear failed\n", __func__);
		goto out;
	}

	ret = count;
out:
	unmap_su_clilm(engsu);
	return ret;
}

static ssize_t raptor2_sysfs_splic_int_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	u32 mapsz;
	u64 mapaddr;
	void __iomem *splic;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.splic_offs;
	mapsz = engsu->sudev.splicsz;

	splic = ioremap(mapaddr, mapsz);
	if (splic == NULL) {
		pr_err("%s: %s SU%d sPLIC ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
				__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	writel((1 << 30), splic + PLIC_PENDING_OFFSET); //trigger an interrupt to SU on line 30

	iounmap(splic);
	return count;
}

static ssize_t raptor2_sysfs_stats_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	int ret;
	struct r2_engine_su *engsu;
	char stats[SU_STATS_SIZE + 1];

	if (chip_version == R2_VERSION_A0) {
		pr_err("%s: Call not supported for version A0\n", __func__);
		return -EOPNOTSUPP;
	}

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("%s: engsu not found\n", __func__);
		return -ENXIO;
	}

	if (!(engsu->membm & MEMTYPE_DLM_BM)) {
		pr_err("%s: DLM not supported for SU\n", __func__);
		return -EOPNOTSUPP;
	}

	ret = enable_su_memory(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d enable_su_memory failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		return ret;
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	memset(stats, 0, sizeof(stats));
	memcpy_fromio(stats, engsu->membuf.dlm + SU_DLM_STATS_OFFS, SU_STATS_SIZE);
	sprintf(buf, "%s\n", stats);
	unmap_su_dlm(engsu);
	ret = strlen(buf);
out:
	return ret;
}

static ssize_t raptor2_sysfs_stats_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_dbg_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_dbg_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_hartid_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	u64 mapaddr;
	u32 hartid, mapsz;
	void __iomem *cmu, *cmu1;
	struct r2_engine_su *engsu;

	if (chip_version == R2_VERSION_A0) {
		pr_err("%s: Call not supported for version A0\n", __func__);
		return -EOPNOTSUPP;
	}

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	cmu1 = cmu + CMU1_OFFSET;
	hartid = (readl(cmu1 + CMU1_CORE_CONFIG_REG) >> HARTID_SHIFT) & HARTID_MASK;

	iounmap(cmu);

	sprintf(buf, "HART ID:0x%x\n", hartid);

	return strlen(buf);
}

static ssize_t raptor2_sysfs_hartid_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	u64 mapaddr;
	u32 mapsz, hartid;
	unsigned long ccr;
	void __iomem *cmu, *cmu1;
	struct r2_engine_su *engsu;

	if (chip_version == R2_VERSION_A0) {
		pr_err("%s: Call not supported for version A0\n", __func__);
		return -EOPNOTSUPP;
	}

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	ret = kstrtou32(buf, 10, &hartid);
	if (ret != 0) {
		pr_err("%s: %s SU%d kstrtou32 fails with err code %d\n",
				__func__, engstr[engsu->etype], engsu->su_index, ret);
		return ret;
	}

	mapaddr = engsu->sudevaddr + engsu->sudev.cmu_offs;
	mapsz = engsu->sudev.cmusz;

	cmu = ioremap(mapaddr, mapsz);
	if (cmu == NULL) {
		pr_err("%s: %s SU%d CMU ioremap fail mapaddr=0x%llx, mapsz=0x%x\n",
			__func__, engstr[engsu->etype], engsu->su_index, mapaddr, mapsz);
		return -ENOMEM;
	}

	pr_info("Writing hart id: 0x%x\n", hartid);
	cmu1 = cmu + CMU1_OFFSET;
	ccr = readl(cmu1 + CMU1_CORE_CONFIG_REG);

	hartid <<= HARTID_SHIFT;
	ccr = (ccr & ~HARTID_REG_MASK) | hartid;
	writel(ccr, cmu1 + CMU1_CORE_CONFIG_REG);

	iounmap(cmu);

	return count;
}

static ssize_t raptor2_sysfs_heartbeat_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	int ret;
	u32 heartbeat;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			return ret;
		}
	}

	ret = map_su_dlm(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_dlm failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		ret = -ENOMEM;
		goto out;
	}

	memcpy_fromio(&heartbeat, engsu->membuf.dlm + SU_DLM_HEARTBEAT_OFFSET, sizeof(u32));
	sprintf(buf, "0x%x\n", heartbeat);

	unmap_su_dlm(engsu);
out:

	return strlen(buf);
}

static ssize_t raptor2_sysfs_heartbeat_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
		return -EOPNOTSUPP;
}

static ssize_t raptor2_sysfs_trigger_show(struct kobject *kobj, struct kobj_attribute *attr,
										char *buf)
{
	int ret;
	u32 trigger;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out;
		}
	}

	ret = map_su_lmem(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_lmem failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	memcpy_fromio(&trigger, engsu->membuf.lmem + OTRX_LMEM_TRIGGER_OFFSET, sizeof(u32));
	sprintf(buf, "0x%x\n", trigger);

	unmap_su_lmem(engsu);

	ret = strlen(buf);
out:
	return ret;
}

static ssize_t raptor2_sysfs_trigger_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	u32 trigger;
	struct r2_engine_su *engsu;

	engsu = get_suinfo(kobj);
	if (engsu == NULL) {
		pr_err("Unable to find engsu for kobj %p\n", kobj);
		return -EINVAL;
	}

	if (buf == NULL) {
		pr_err("buf is NULL\n");
		return -EINVAL;
	}

	ret = kstrtou32(buf, 10, &trigger);
	if (ret != 0) {
		pr_err("%s: %s SU%d kstrtou32 fails with err code %d\n",
				__func__, engstr[engsu->etype], engsu->su_index, ret);
		return ret;
	}

	ret = check_su_enable(engsu);
	if (ret < 0) {
		pr_err("%s %s SU%d check_su_enable failed\n",
			__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (ret == 0) {
		ret = enable_su_memory(engsu);
		if (ret < 0) {
			pr_err("%s %s SU%d enable_su_memory failed\n",
					__func__, engstr[engsu->etype], engsu->su_index);
			goto out;
		}
	}

	ret = map_su_lmem(engsu);
	if (ret < 0) {
		pr_err("%s: %s SU%d map_su_lmem failed\n",
				__func__, engstr[engsu->etype], engsu->su_index);
		goto out;
	}

	if (trigger)
		trigger = TRIGGER_PATTERN;
	else
		trigger = 0x0;

	memcpy_toio(engsu->membuf.lmem + OTRX_LMEM_TRIGGER_OFFSET, &trigger, sizeof(u32));

	unmap_su_lmem(engsu);

	ret = count;
out:

	return ret;
}

static ssize_t raptor2_sysfs_l1reset_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d: %s\n", l1reset_in_progress,
			(l1reset_in_progress) ? "L1 Reset in progress" :
			"L1 Reset not in progress");

	return ret;
}

static ssize_t raptor2_sysfs_l1reset_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;
	uint32_t val;

	ret = count;

	ret = kstrtou32(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: kstrtou32 fails with err code %d\n", __func__, ret);
		ret = 0;
		goto out;
	}

	if (val && l1reset_in_progress) {
		pr_err("%s: Unable to perform operation. L1 Reset in progress", __func__);
		ret = 0;
		goto out;
	}

	if (val == 0) {
		ret = 0;
		goto out;
	}

	l1reset_in_progress = 1;

	pr_info("%s: calling bss_l1_reset\n", __func__);
	ret = bss_l1_reset();
	if (ret < 0) {
		pr_err("%s: bss_l1_reset failed\n", __func__);
		ret = 0;
	}

	pr_info("%s: calling raptor2_reconf_rfspidev\n", __func__);
	ret = raptor2_reconf_rfspidev();
	if (ret < 0) {
		pr_err("%s: raptor2_reconf_rfspidev failed\n", __func__);
		ret = 0;
	} else
		ret = count;

	l1reset_in_progress = 0;

out:
	return ret;
}

static ssize_t raptor2_sysfs_public_key_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret, count;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	ret = bss_read_data(buf, &count, BSS_PUBLIC_KEY);
	if (ret < 0) {
		pr_err("%s: bss_read_data failed with err %d for %s\n",
				__func__, ret, bdstr[BSS_PUBLIC_KEY]);
		return 0;
	}

	return count;
}

static ssize_t raptor2_sysfs_public_key_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	ret = bss_write_data(buf, count, BSS_PUBLIC_KEY);
	if (ret < 0) {
		pr_err("%s: bss_write_data failed with err %d for %s\n",
				__func__, ret, bdstr[BSS_PUBLIC_KEY]);
		return 0;
	}

	pr_info("%s: Success\n", __func__);

	return count;
}

static ssize_t raptor2_sysfs_license_key_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret, count;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	ret = bss_read_data(buf, &count, BSS_LICENSE_KEY);
	if (ret < 0) {
		pr_err("%s: bss_read_data failed with err %d for %s\n",
				__func__, ret, bdstr[BSS_LICENSE_KEY]);
		return 0;
	}

	return count;
}

static ssize_t raptor2_sysfs_license_key_store(struct kobject *kobj, struct kobj_attribute *attr,
								const char *buf, size_t count)
{
	int ret;

	if (buf == NULL) {
		pr_err("%s: buf is NULL\n", __func__);
		return -EINVAL;
	}

	ret = bss_write_data(buf, count, BSS_LICENSE_KEY);
	if (ret < 0) {
		pr_err("%s: bss_write_data failed with err %d for %s\n",
				__func__, ret, bdstr[BSS_LICENSE_KEY]);
		return 0;
	}

	pr_info("%s: Success\n", __func__);
	return count;
}

static ssize_t raptor2_sysfs_secure_load_fmw_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret;


	ret = sprintf(buf, "%d: %s\n", secure_load_fmw,
			(secure_load_fmw) ? "Secure Loading of SU firmware enabled" :
			"Secure Loading of SU firmware disabled");

	return ret;
}

static ssize_t raptor2_sysfs_secure_load_fmw_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint32_t val;

	ret = count;

	ret = kstrtou32(buf, 10, &val);
	if (ret < 0) {
		pr_err("%s: kstrtou32 fails with err code %d\n", __func__, ret);
		return 0;
	}

	if (val)
		secure_load_fmw = 1;
	else
		secure_load_fmw = 0;

	return count;
}

/*
static void raptor2_sysfs_release(struct kobject *kobj)
{
	pr_debug("kobject: (%p): %s\n", kobj, __func__);
	kfree(kobj);
}
*/

/* Iterate over sysfs entries for a certain SU */
bool raptor2_create_su_kobj_entries(struct kobject *kobj_su,
			struct r2_kobj_entry *r2kobj, r2_engine_t engtype)
{
	int i, ret, fail;

	if (r2kobj == NULL) {
		pr_err("r2kobj is NULL\n");
		return false;
	}

	fail = 0;
	i = 0;
	while (r2_sysfs_entry_table[i].stype != SU_ENTRY_NONE) {

		struct r2_sysfs_entry *en;

		en = &r2_sysfs_entry_table[i];
		switch (en->stype) {
		default:
			pr_err("Incorrect stype %d\n", en->stype);
			continue;
		case SU_ENTRY_EXEC_IMAGE:
			sysfs_attr_init(&r2kobj->image_attr.attr);
			r2kobj->image_attr.attr.name = "exec_image";
			r2kobj->image_attr.attr.mode = S_IWUGO;
			r2kobj->image_attr.size = MAX_IMAGE_SIZE;
			//r2kobj->image_attr.read = raptor2_sysfs_image_read;
			r2kobj->image_attr.write = raptor2_sysfs_image_write;
			//ret = sysfs_create_file(kobj_su, &r2kobj->image_attr.attr);
			ret = sysfs_create_bin_file(kobj_su, &r2kobj->image_attr);
			break;
		case SU_ENTRY_IMAGE_SIZE:
			sysfs_attr_init(&r2kobj->image_size_attr.attr);
			r2kobj->image_size_attr.attr.name = "image_size";
			r2kobj->image_size_attr.attr.mode = S_IRWXUGO;
			r2kobj->image_size_attr.show = raptor2_sysfs_image_size_show;
			r2kobj->image_size_attr.store = raptor2_sysfs_image_size_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->image_size_attr.attr);
			break;
		case SU_ENTRY_RESET:
			sysfs_attr_init(&r2kobj->reset_attr.attr);
			r2kobj->reset_attr.attr.name = "reset";
			r2kobj->reset_attr.attr.mode = S_IRWXUGO;
			r2kobj->reset_attr.show = raptor2_sysfs_reset_show;
			r2kobj->reset_attr.store = raptor2_sysfs_reset_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->reset_attr.attr);
			break;
		case SU_ENTRY_INFO:
			sysfs_attr_init(&r2kobj->info_attr.attr);
			r2kobj->info_attr.attr.name = "info";
			//r2kobj->info_attr.attr.mode = 0644;
			r2kobj->info_attr.attr.mode = S_IRWXUGO;
			r2kobj->info_attr.show = raptor2_sysfs_info_show;
			r2kobj->info_attr.store = raptor2_sysfs_info_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->info_attr.attr);
			break;
		case SU_ENTRY_CONF:
			sysfs_attr_init(&r2kobj->conf_attr.attr);
			r2kobj->conf_attr.attr.name = "conf";
			//r2kobj->conf_attr.attr.mode = 0444;
			r2kobj->conf_attr.attr.mode = S_IRUGO;
			r2kobj->conf_attr.show = raptor2_sysfs_conf_show;
			r2kobj->conf_attr.store = raptor2_sysfs_conf_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->conf_attr.attr);
			break;
		case SU_ENTRY_REGS:
			sysfs_attr_init(&r2kobj->regs_attr.attr);
			r2kobj->regs_attr.attr.name = "regs";
			r2kobj->regs_attr.attr.mode = S_IRUGO;
			r2kobj->regs_attr.show = raptor2_sysfs_regs_show;
			r2kobj->regs_attr.store = raptor2_sysfs_regs_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->regs_attr.attr);
			break;
		case SU_ENTRY_AU:
			sysfs_attr_init(&r2kobj->au_attr.attr);
			r2kobj->au_attr.attr.name = "au";
			r2kobj->au_attr.attr.mode = S_IRUGO;
			r2kobj->au_attr.show = raptor2_sysfs_au_show;
			r2kobj->au_attr.store = raptor2_sysfs_au_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->au_attr.attr);
			break;
		case SU_ENTRY_ILM:
			sysfs_attr_init(&r2kobj->ilm_attr.attr);
			r2kobj->ilm_attr.attr.name = "ilm";
			r2kobj->ilm_attr.attr.mode = S_IRWXUGO;
			r2kobj->ilm_attr.size = MAX_ILMBUF_SIZE;
			r2kobj->ilm_attr.read = raptor2_sysfs_ilm_read;
			r2kobj->ilm_attr.write = raptor2_sysfs_ilm_write;
			ret = sysfs_create_bin_file(kobj_su, &r2kobj->ilm_attr);
			break;
		case SU_ENTRY_ILM_0:
			sysfs_attr_init(&r2kobj->ilm0_attr.attr);
			r2kobj->ilm0_attr.attr.name = "ilm_0";
			r2kobj->ilm0_attr.attr.mode = S_IRWXUGO;
			r2kobj->ilm0_attr.store = raptor2_sysfs_ilm0_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->ilm0_attr.attr);
			break;
		case SU_ENTRY_DLM:
			sysfs_attr_init(&r2kobj->dlm_attr.attr);
			r2kobj->dlm_attr.attr.name = "dlm";
			r2kobj->dlm_attr.attr.mode = S_IRWXUGO;
			r2kobj->dlm_attr.size = MAX_DLMBUF_SIZE;
			r2kobj->dlm_attr.read = raptor2_sysfs_dlm_read;
			r2kobj->dlm_attr.write = raptor2_sysfs_dlm_write;
			ret = sysfs_create_bin_file(kobj_su, &r2kobj->dlm_attr);
			break;
		case SU_ENTRY_DLM_0:
			sysfs_attr_init(&r2kobj->dlm0_attr.attr);
			r2kobj->dlm0_attr.attr.name = "dlm_0";
			r2kobj->dlm0_attr.attr.mode = S_IRWXUGO;
			r2kobj->dlm0_attr.store = raptor2_sysfs_dlm0_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->dlm0_attr.attr);
			break;
		case SU_ENTRY_LMEM:
			if ((engtype == ENGINE_SPU) || (engtype == ENGINE_IRING))
				goto loop;
			sysfs_attr_init(&r2kobj->lmem_attr.attr);
			r2kobj->lmem_attr.attr.name = "lmem";
			r2kobj->lmem_attr.attr.mode = S_IRWXUGO;
			r2kobj->lmem_attr.size = MAX_LMEMBUF_SIZE;
			r2kobj->lmem_attr.read = raptor2_sysfs_lmem_read;
			r2kobj->lmem_attr.write = raptor2_sysfs_lmem_write;
			ret = sysfs_create_bin_file(kobj_su, &r2kobj->lmem_attr);
			break;
		case SU_ENTRY_LMEM_0:
			if ((engtype == ENGINE_SPU) || (engtype == ENGINE_IRING))
				goto loop;
			sysfs_attr_init(&r2kobj->lmem0_attr.attr);
			r2kobj->lmem0_attr.attr.name = "lmem_0";
			r2kobj->lmem0_attr.attr.mode = S_IRWXUGO;
			r2kobj->lmem0_attr.store = raptor2_sysfs_lmem0_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->lmem0_attr.attr);
			break;
		case SU_ENTRY_CLILM:
			if ((chip_version == R2_VERSION_B0) &&
					(engtype == ENGINE_TXU)) {
				sysfs_attr_init(&r2kobj->clilm0_attr.attr);
				r2kobj->clilm_attr.attr.name = "clilm";
				//r2kobj->clilm_attr.attr.mode = S_IRWXUGO;
				r2kobj->clilm_attr.attr.mode = 0777;
				r2kobj->clilm_attr.size = MAX_CLILMBUF_SIZE;
				r2kobj->clilm_attr.read = raptor2_sysfs_clilm_read;
				r2kobj->clilm_attr.write = raptor2_sysfs_clilm_write;
				ret = sysfs_create_bin_file(kobj_su, &r2kobj->clilm_attr);
			}
			break;
		case SU_ENTRY_CLILM_0:
			if ((chip_version == R2_VERSION_B0) &&
					(engtype == ENGINE_TXU)) {
				sysfs_attr_init(&r2kobj->clilm0_attr.attr);
				r2kobj->clilm0_attr.attr.name = "clilm_0";
				r2kobj->clilm0_attr.attr.mode = S_IRWXUGO;
				r2kobj->clilm0_attr.store = raptor2_sysfs_clilm0_store;
				ret = sysfs_create_file(kobj_su, &r2kobj->clilm0_attr.attr);
			}
			break;
		case SU_ENTRY_sPLIC_INT:
			sysfs_attr_init(&r2kobj->splic_int_attr.attr);
			r2kobj->splic_int_attr.attr.name = "sPLIC_int";
			r2kobj->splic_int_attr.attr.mode = S_IRWXUGO;
			r2kobj->splic_int_attr.store = raptor2_sysfs_splic_int_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->splic_int_attr.attr);
			break;
		case SU_ENTRY_STATS:
			sysfs_attr_init(&r2kobj->stats_attr.attr);
			r2kobj->stats_attr.attr.name = "stats";
			r2kobj->stats_attr.attr.mode = S_IRUGO;
			r2kobj->stats_attr.show = raptor2_sysfs_stats_show;
			r2kobj->stats_attr.store = raptor2_sysfs_stats_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->stats_attr.attr);
			break;
		case SU_ENTRY_DBG:
			sysfs_attr_init(&r2kobj->dbg_attr.attr);
			r2kobj->dbg_attr.attr.name = "dbg";
			r2kobj->dbg_attr.attr.mode = S_IRUGO;
			r2kobj->dbg_attr.show = raptor2_sysfs_dbg_show;
			r2kobj->dbg_attr.store = raptor2_sysfs_dbg_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->dbg_attr.attr);
			break;
		case SU_ENTRY_MEM_ENABLE:
			sysfs_attr_init(&r2kobj->su_mem_enable_attr.attr);
			r2kobj->su_mem_enable_attr.attr.name = "mem_enable";
			r2kobj->su_mem_enable_attr.attr.mode = S_IRWXUGO;
			r2kobj->su_mem_enable_attr.show = raptor2_sysfs_mem_enable_show;
			r2kobj->su_mem_enable_attr.store = raptor2_sysfs_mem_enable_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->su_mem_enable_attr.attr);
			break;
		case SU_ENTRY_HART_ID:
			if (chip_version == R2_VERSION_B0) {
				sysfs_attr_init(&r2kobj->hartid_attr.attr);
				r2kobj->hartid_attr.attr.name = "hart_id";
				r2kobj->hartid_attr.attr.mode = S_IRWXUGO;
				r2kobj->hartid_attr.show = raptor2_sysfs_hartid_show;
				r2kobj->hartid_attr.store = raptor2_sysfs_hartid_store;
				ret = sysfs_create_file(kobj_su, &r2kobj->hartid_attr.attr);
			}
			break;
		case SU_ENTRY_HEART_BEAT:
			sysfs_attr_init(&r2kobj->heartbeat_attr.attr);
			r2kobj->heartbeat_attr.attr.name = "heart_beat";
			//r2kobj->heartbeat_attr.attr.mode = S_IRWXUGO;
			r2kobj->heartbeat_attr.attr.mode = 0777;
			r2kobj->heartbeat_attr.show = raptor2_sysfs_heartbeat_show;
			r2kobj->heartbeat_attr.store = raptor2_sysfs_heartbeat_store;
			ret = sysfs_create_file(kobj_su, &r2kobj->heartbeat_attr.attr);
			break;
		case SU_ENTRY_TRIGGER:
			if (engtype == ENGINE_OTRX_QSU) {
				sysfs_attr_init(&r2kobj->trigger_attr.attr);
				r2kobj->trigger_attr.attr.name = "trigger";
				//r2kobj->trigger_attr.attr.mode = S_IRWXUGO;
				r2kobj->trigger_attr.attr.mode = 0777;
				r2kobj->trigger_attr.show = raptor2_sysfs_trigger_show;
				r2kobj->trigger_attr.store = raptor2_sysfs_trigger_store;
				ret = sysfs_create_file(kobj_su, &r2kobj->trigger_attr.attr);
			}
			break;
		}

		if (ret) {
			pr_err("sysfs_create_file failed for %s, ret code %d\n", en->name, ret);
			fail = 1;
			goto loop;
		}
		//pr_debug("created entry %s\n", en->name);
loop:
		++i;
	}

	if (fail == 0)
		return true;

	return false;
}

/* Iterate over SUs for a certain engine */
bool raptor2_init_kobj_sus(struct r2_engine_info *info)
{
	int ret;
	char suname[8];
	struct list_head *cur;
	struct kobject *kobj_su, *kobj_eng;
	struct r2_engine_su *engsu, *suhead;
	struct r2_kobj_entry *entry;

	if (info == NULL) {
		pr_err("info is NULL\n");
		return false;
	}

	if (info->enghead == NULL) {
		pr_err("info->enghead is NULL\n");
		return false;
	}

	memset(suname, 0, sizeof(suname));

	suhead = info->enghead;
	cur = &suhead->node;
	while (cur != NULL) {

		engsu = list_entry(cur, struct r2_engine_su, node);
		if (!engsu->enabled)
			goto loop;

		sprintf(suname, "%s%d", "SU", engsu->su_index);
		kobj_su = kobject_create_and_add(suname, info->kobj_eng);
		if (kobj_su == NULL) {
			pr_err("%s: kobject_create_and_add() failed for %s\n",
								__func__, suname);
			return false;
		}
		engsu->kobj_su = kobj_su;
		kobj_eng = info->kobj_eng;
		entry = kzalloc(sizeof(struct r2_kobj_entry), GFP_KERNEL);
		if (entry == NULL) {
			pr_err("kzalloc() failed\n");
			return false;
		}

		engsu->kobj_entry = entry;
		ret = raptor2_create_su_kobj_entries(engsu->kobj_su, engsu->kobj_entry,
										info->etype);
		if (ret == false) {
			pr_err("raptor2_create_kobj_entries() failed\n");
			goto loop;
			//return false;
		}
loop:
		if (cur->next == &suhead->node)
			break;

		cur = cur->next;
	}

	return true;
}

/* All sysfs entries under /raptor2 under here */
static struct kobj_attribute l1_reset_attr = __ATTR(l1_reset, 0660,
	raptor2_sysfs_l1reset_show, raptor2_sysfs_l1reset_store);
static struct kobj_attribute public_key_attr = __ATTR(public_key, 0660,
	raptor2_sysfs_public_key_show, raptor2_sysfs_public_key_store);
static struct kobj_attribute license_key_attr = __ATTR(license_key, 0660,
	raptor2_sysfs_license_key_show, raptor2_sysfs_license_key_store);
static struct kobj_attribute secure_load_fmw_attr = __ATTR(secure_load_fmw, 0660,
	raptor2_sysfs_secure_load_fmw_show, raptor2_sysfs_secure_load_fmw_store);

static int create_hl_kobj_entries(void)
{
	int ret;

	if (kobj_raptor2 == NULL) {
		pr_err("%s: kobj for /raptor2 is NULL\n", __func__);
		return -EINVAL;
	}

	/* L1 reset entry point */
	ret = sysfs_create_file(kobj_raptor2, &l1_reset_attr.attr);
	if (ret < 0) {
		pr_err("%s: failed to create the l1_reset file in /sys/raptor2\n", __func__);
		return ret;
	}

	/* Public Key entry point */
	ret = sysfs_create_file(kobj_raptor2, &public_key_attr.attr);
	if (ret < 0) {
		pr_err("%s: failed to create the public_key file in /sys/raptor2\n", __func__);
		return ret;
	}

	/* License Key entry point */
	ret = sysfs_create_file(kobj_raptor2, &license_key_attr.attr);
	if (ret < 0) {
		pr_err("%s: failed to create the license_key file in /sys/raptor2\n", __func__);
		return ret;
	}

	/* Secure Load fmw entry point */
	ret = sysfs_create_file(kobj_raptor2, &secure_load_fmw_attr.attr);
	if (ret < 0) {
		pr_err("%s: failed to create the secure_load_fmw file in /sys/raptor2\n", __func__);
		return ret;
	}

	return 0;
}

bool raptor2_init_sysfs(struct r2_engine_info r2_engine_info_table[], r2_chip_version_t r2vers)
{
	int i, ret;
	struct r2_engine_info *info;
	struct kobject *kobj;

	l1reset_in_progress = 0;
	secure_load_fmw = 0;

	kobj_raptor2 = kobject_create_and_add("raptor2", NULL);
	if (kobj_raptor2 == NULL) {
		pr_err("kobject_create_and_add() failed\n");
		return false;
	}

	chip_version = r2vers;

	ret = create_hl_kobj_entries();
	if (ret < 0) {
		pr_err("%s: create_hl_kobj_entries failed with\n", __func__);
		return false;
	}

	i = 0;
	while (r2_engine_info_table[i].etype != ENGINE_NONE) {

		if ((chip_version == R2_VERSION_A0) &&
				(r2_engine_info_table[i].etype == ENGINE_IRING))
			goto next;

		info = (struct r2_engine_info *)(&r2_engine_info_table[i]);

		kobj = kobject_create_and_add(info->ename, kobj_raptor2);
		if (kobj == NULL) {
			pr_err("kobject_create_and_add() failed\n");
			ret = false;
			goto out;
		}

		info->kobj_eng = kobj;
		pr_info("%s: calling raptor2_init_kobj_sus() for %s\n", __func__,
				info->ename);
		ret = raptor2_init_kobj_sus(info);
		if (ret == false) {
			pr_err("raptor2_init_kobj_sus() failed\n");
			goto out;
		}
next:
		++i;
	}
out:
	return ret;
}
