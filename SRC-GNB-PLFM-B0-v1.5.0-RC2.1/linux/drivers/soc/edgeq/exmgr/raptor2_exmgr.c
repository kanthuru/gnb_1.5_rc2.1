// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 EdgeQ, Inc.
 * Raptor2 EXMGR Initialization
 */

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include "raptor2_exmgr_ops.h"

static r2_chip_version_t r2vers;

static struct su_device sudev_g;
static struct su_memory sumem_g;

static struct r2_rtos_info *rtos_info;
static struct eqsu_membuf eqsu_mbuf_table[TOTALSUCOUNT + 1];

static r2_chip_version_t get_chip_version(void)
{
	int size;
	const char *cv;
	struct device_node *root;

	root = of_find_node_by_path("/");
	if (root == NULL) {
		pr_err("Root not found. assuming B0\n");
		goto out;
	}

	cv = of_get_property(root, "chip-version", &size);
	if (cv == NULL) {
		pr_err("property chip-version not found. assuming B0\n");
		goto out;
	}

	pr_info("Raptor2 chip version detected %s\n", cv);

	if (strncmp(cv, "A0", strlen("A0")) == 0)
		return R2_VERSION_A0;

out:
	return R2_VERSION_B0;
}

static int parse_su_resource_addr(struct device_node *node, int index, u64 *rscaddr, u64 *rscsize)
{
	u64 sz;
	unsigned int flags;
	const __be32 *addrp;

	addrp = of_get_address(node, index, &sz, &flags);
	if (addrp) {
		*rscaddr = ((__be64)be32_to_cpu(*addrp)) << 32 |
							(__be64)be32_to_cpu(*(addrp+1));
		*rscsize = ((__be64)be32_to_cpu(*(addrp+2))) << 32 |
							(__be64)be32_to_cpu(*(addrp+3));
		return 0;
	}

	return -1;
}

static u32 getmembm(r2_engine_t etype, int suidx)
{
	u32 mask;

	mask = MEMTYPE_ILM_BM | MEMTYPE_DLM_BM | MEMTYPE_LMEM_WB_BM |
		MEMTYPE_LMEM_NC_BM | MEMTYPE_LMEM_WT_BM | MEMTYPE_LMEM_CNOC_BM;

	switch (etype) {
	default:
		return 0;
	case ENGINE_PPU:
	case ENGINE_PPU_MXL:
	case ENGINE_PPU_CRSS:
	case ENGINE_OTRX_QSU:
		break;
	case ENGINE_OTRX_HSU:
		if ((suidx == 7) || (suidx == 8) || (suidx == 9))
			mask |= MEMTYPE_LMEM_CNOC_B2_BM;
		break;
	case ENGINE_IRING:
	case ENGINE_SPU:
		mask = MEMTYPE_ILM_BM | MEMTYPE_DLM_BM;
		break;
	case ENGINE_ECPRI:
		mask |= MEMTYPE_LMEM_CNOC_B2_BM;
		break;
	case ENGINE_TXU:
		mask |= MEMTYPE_LMEM_CNOC_B2_BM;
		if (r2vers == R2_VERSION_B0)
			mask |= MEMTYPE_CLUSTER_ILM_BM;
		break;
	}

	return mask;
}

static int get_mbuftbl_idx(r2_engine_t etype, int suidx)
{
	if (suidx < 0) {
		pr_err("Invalid SU index %d\n", suidx);
		return -EINVAL;
	}

	switch (etype) {
	default:
		pr_err("Invalid etype %d\n", etype);
		return -EINVAL;
	case ENGINE_PPU:
		return (PPU0 + suidx);
	case ENGINE_PPU_MXL:
		return (PPU_MXL0 + suidx);
	case ENGINE_PPU_CRSS:
		return (PPU_CRSS0 + suidx);
	case ENGINE_OTRX_HSU:
		return (OTRX_HSU0 + suidx);
	case ENGINE_OTRX_QSU:
		return (OTRX_QSU0 + suidx);
	case ENGINE_SPU:
		return (SPU0 + suidx);
	case ENGINE_ECPRI:
		return (ECPRI0 + suidx);
	case ENGINE_TXU:
		return (TXU0 + suidx);
	case ENGINE_IRING:
		return (IRING0 + suidx);
	}

	return 0; // never reached
}

static bool parse_raptor2_engine_sus(struct device_node *enode, r2_engine_t etype,
						struct eqsu_membuf eqsu_mbuf_table[])
{
	u32 vals[4];
	char compat[32];
	int j, rc, suidx;
	struct device_node *cn;
	struct r2_engine_su *engsu;

	memset(compat, 0, sizeof(compat));
	memset(vals, 0, sizeof(vals));

	if (enode == NULL) {
		pr_err("enode is NULL\n");
		return false;
	}

	if (eqsu_mbuf_table == NULL) {
		pr_err("eqsu_mbuf_table is NULL\n");
		return false;
	}


	cn = NULL;
	suidx = -1;

	while ((cn = of_get_next_child(enode, cn))) {

		const char *status;
		u64 rscaddr, rscsize;

		if (strncmp(cn->name, "aliases", strlen("aliases")) == 0)
			continue;

		engsu = kzalloc(sizeof(struct r2_engine_su), GFP_KERNEL);
		if (engsu == NULL) {
			pr_err("engsu kzalloc failed for cn->name=%s\n", cn->name);
			return false;
		}

		engsu->etype = etype;
		engsu->su_index = ++suidx;
		j = get_mbuftbl_idx(etype, suidx);
		eqsu_mbuf_table[j].engsu = engsu;
		engsu->membm = getmembm(engsu->etype, suidx);
		memcpy(&engsu->sudev, &sudev_g, sizeof(struct su_device));
		memcpy(&engsu->sumem, &sumem_g, sizeof(struct su_memory));
		strncpy(engsu->name, cn->name,
			(strlen(cn->name) >= sizeof(engsu->name)) ? (sizeof(engsu->name) - 1) :
			strlen(cn->name));

		rc = of_property_read_string(cn, "status", &status);
		if (rc < 0) {
			pr_err("failed to read status for %s\n", cn->name);
			continue;
		}

		if (strncmp(status, "okay", strlen("okay")) == 0)
			engsu->enabled = 1;
		else
			engsu->enabled = 0;

		rc = parse_su_resource_addr(cn, 0, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get sudev resource address for %s\n", cn->name);
			continue;
		}
		engsu->sudevaddr = rscaddr;
		engsu->sudevsz = rscsize;

		rc = parse_su_resource_addr(cn, 1, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get cpuregs resource address for %s\n", cn->name);
			continue;
		}
		engsu->cpuregaddr = rscaddr;
		engsu->cpuregsz = rscsize;

		rc = parse_su_resource_addr(cn, 2, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_debug("failed to get d-cache resource address for %s\n", cn->name);
			continue;
		}
		engsu->dcaddr = rscaddr;
		engsu->dcsz = rscsize;

		rc = parse_su_resource_addr(cn, 3, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get i-cache resource address for %s\n", cn->name);
			continue;
		}
		engsu->icaddr = rscaddr;
		engsu->icsz = rscsize;

		rc = parse_su_resource_addr(cn, 4, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_debug("failed to get suinfo resource address for %s\n", cn->name);
			continue;
		}
		engsu->suinfoaddr = rscaddr;
		engsu->suinfosz = rscsize;

		rc = parse_su_resource_addr(cn, 5, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_debug("failed to get suconf resource address for %s\n", cn->name);
			continue;
		}
		engsu->suconfaddr = rscaddr;
		engsu->suconfsz = rscsize;

		rc = parse_su_resource_addr(cn, 6, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_debug("failed to get suau resource address for %s\n", cn->name);
			continue;
		}
		engsu->suauaddr = rscaddr;
		engsu->suausz = rscsize;

		rc = parse_su_resource_addr(cn, 7, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get sustats resource address for %s\n", cn->name);
			continue;
		}
		engsu->sustatsaddr = rscaddr;
		engsu->sustatssz = rscsize;

		rc = parse_su_resource_addr(cn, 8, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get sudbg resource address for %s\n", cn->name);
			continue;
		}
		engsu->sudbgaddr = rscaddr;
		engsu->sudbgsz = rscsize;

		rc = parse_su_resource_addr(cn, 9, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmemaddr_wb_local resource address for %s\n",
					cn->name);
			continue;
		}
		engsu->lmemaddr_wb_local = rscaddr;
		engsu->lmemsz_wb_local = rscsize;

		rc = parse_su_resource_addr(cn, 10, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmemaddr_nc_local resource address for %s\n",
					cn->name);
			continue;
		}
		engsu->lmemaddr_nc_local = rscaddr;
		engsu->lmemsz_nc_local = rscsize;

		rc = parse_su_resource_addr(cn, 11, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmemaddr_wt_local resource address for %s\n",
					cn->name);
			continue;
		}
		engsu->lmemaddr_wt_local = rscaddr;
		engsu->lmemsz_wt_local = rscsize;

		rc = parse_su_resource_addr(cn, 12, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmemaddr_cpu resource address for %s\n", cn->name);
			continue;
		}
		engsu->lmemaddr_cpu = rscaddr;
		engsu->lmemsz_cpu = rscsize;

		rc = parse_su_resource_addr(cn, 13, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to LMEM_B2 address for %s\n", cn->name);
			continue;
		}
		engsu->lmemb2_addr_cpu = rscaddr;
		engsu->lmemb2_sz_cpu = rscsize;

		rc = parse_su_resource_addr(cn, 14, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get ILM offs resource address for %s\n", cn->name);
			continue;
		}
		engsu->sudev.ilm_offs = rscaddr;
		engsu->sudev.ilmsz = rscsize;

		rc = parse_su_resource_addr(cn, 15, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get DLM offs resource address for %s\n", cn->name);
			continue;
		}
		engsu->sudev.dlm_offs = rscaddr;
		engsu->sudev.dlmsz = rscsize;

		if ((r2vers == R2_VERSION_B0) && (engsu->etype == ENGINE_TXU)) {
			rc = parse_su_resource_addr(cn, 16, &rscaddr, &rscsize);
			if (rc == -1) {
				pr_err("failed to get ILM Cluster Local resource address for %s\n",
						cn->name);
				continue;
			}
			engsu->clilmaddr_local = rscaddr;
			engsu->clilmsz_local = rscsize;

			rc = parse_su_resource_addr(cn, 17, &rscaddr, &rscsize);
			if (rc == -1) {
				pr_err("failed to get ILM Cluster CNOC resource address for %s\n",
						cn->name);
				continue;
			}
			engsu->clilmaddr_cnoc = rscaddr;
			engsu->clilmsz_cnoc = rscsize;
		}
	}

	return true;
}

static bool parse_su_resource(struct device_node *rn, const char *name,
							u64 *rscaddr, u64 *rscsize)
{
	int rc;
	struct device_node *cn;

	if (rn == NULL) {
		pr_err("rn is NULL\n");
		return false;
	}

	if ((name == NULL) || (strlen(name) == 0)) {
		pr_err("name is NULL or empty string\n");
		return false;
	}

	if (rscaddr == NULL) {
		pr_err("rscaddr is NULL\n");
		return false;
	}

	if (rscsize == NULL) {
		pr_err("rscsize is NULL\n");
		return false;
	}

	cn = of_get_child_by_name(rn, name);
	if (!cn) {
		pr_err("No %s information found in DT for SUDEV\n", name);
		return false;
	}

	rc = parse_su_resource_addr(cn, 0, rscaddr, rscsize);
	if (rc == -1) {
		pr_err("failed to get sudev resource address for %s\n", cn->name);
		return false;
	}

	return true;
}

#if defined(WLAN)
static bool parse_rtos_info_dt(void)
{
	uint8_t *mptr;
	u64 sz, addr, size;
	unsigned int flags;
	const __be32 *addrp;
	struct device_node *rn;

	rn = of_find_node_by_name(NULL, "coredump-mem");
	if (rn == NULL) {
		pr_err("Parsing coredump-mem failed\n");
		return false;
	}

	addrp = of_get_address(rn, 0, &sz, &flags);
	if (addrp) {
		addr = ((__be64)be32_to_cpu(*addrp)) << 32 |
							(__be64)be32_to_cpu(*(addrp+1));
		size = ((__be64)be32_to_cpu(*(addrp+2))) << 32 |
							(__be64)be32_to_cpu(*(addrp+3));
		eqsu_mbuf_table[MRTOS].mbufsize = size;
		pr_info("%s: Found RTOS core dump memory %llx size %llx\n",
					__func__, addr, size);
	}
	else {
		pr_err("%s: Finding RTOS core dump memory address failed\n", __func__);
		return false;
	}

	mptr = memremap(addr, size, MEMREMAP_WB);
	if (mptr == NULL) {
		pr_err("%s: memremap failed for RTOS Memory %llx size %llx\n",
							__func__, addr, size);
		return false;
	}

	rtos_info = (struct r2_rtos_info *)mptr;
	memset(rtos_info, 0, sizeof(struct r2_rtos_info));
	eqsu_mbuf_table[MRTOS].mbuf = mptr;
	rtos_info->info.startaddr = addr;
	rtos_info->info.size = size;

	return true;
}
#endif	// WLAN

static bool parse_su_device_dt(void)
{
	bool ret;
	struct device_node *rn;
	u64 rscaddr, rscsize;

	rn = of_find_node_by_name(NULL, "su_device");
	if (rn == NULL) {
		pr_err("Parsing su_device failed\n");
		return false;
	}

	ret = parse_su_resource(rn, "cmu", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for cmu\n");
		return false;
	}
	sudev_g.cmu_offs = rscaddr;
	sudev_g.cmusz = rscsize;

	ret = parse_su_resource(rn, "stmr", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for stmr\n");
		return false;
	}

	sudev_g.stmr_offs = rscaddr;
	sudev_g.stmrsz = rscsize;

	ret = parse_su_resource(rn, "dbg", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for stmr\n");
		return false;
	}

	sudev_g.dbg_offs = rscaddr;
	sudev_g.dbgsz = rscsize;

	ret = parse_su_resource(rn, "tmr", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for tmr\n");
		return false;
	}

	sudev_g.tmr_offs = rscaddr;
	sudev_g.tmrsz = rscsize;

	ret = parse_su_resource(rn, "peripherals", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for peripherals\n");
		return false;
	}

	sudev_g.per_offs = rscaddr;
	sudev_g.persz = rscsize;

	ret = parse_su_resource(rn, "eplic", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for eplic\n");
		return false;
	}

	sudev_g.eplic_offs = rscaddr;
	sudev_g.eplicsz = rscsize;

	ret = parse_su_resource(rn, "splic", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for splic\n");
		return false;
	}

	sudev_g.splic_offs = rscaddr;
	sudev_g.splicsz = rscsize;

	return true;
}

static bool parse_su_memory_dt(void)
{
	bool ret;
	struct device_node *rn;
	u64 rscaddr, rscsize;

	rn = of_find_node_by_name(NULL, "su_memory");
	if (rn == NULL) {
		pr_err("Parsing su_memory failed\n");
		return false;
	}

	ret = parse_su_resource(rn, "loc_lmem_wb", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for loc_lmem_wb\n");
		return false;
	}

	sumem_g.loc_lmem_wb_addr = rscaddr;
	sumem_g.loc_lmem_wb_sz = rscsize;

	ret = parse_su_resource(rn, "loc_lmem_wt", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for loc_lmem_wt\n");
		return false;
	}

	sumem_g.loc_lmem_wt_addr = rscaddr;
	sumem_g.loc_lmem_wt_sz = rscsize;

	ret = parse_su_resource(rn, "loc_lmem_u", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for loc_lmem_u\n");
		return false;
	}

	sumem_g.loc_lmem_u_addr = rscaddr;
	sumem_g.loc_lmem_u_sz = rscsize;

	ret = parse_su_resource(rn, "lmem", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for lmem\n");
		return false;
	}

	sumem_g.lmem_addr = rscaddr;
	sumem_g.lmem_sz = rscsize;

	ret = parse_su_resource(rn, "edev", &rscaddr, &rscsize);
	if (!ret) {
		pr_err("parse_su_resource failed for edev\n");
		return false;
	}

	sumem_g.edev_addr = rscaddr;
	sumem_g.edev_sz = rscsize;

	return true;
}

static int init_eqmbuf_table(struct eqsu_membuf eqsu_mbuf_table[])
{
	int num_sus;
	r2_suidx_t i;

	if (eqsu_mbuf_table == NULL) {
		pr_err("%s: eqsu_mbuf_table is NULL\n", __func__);
		return -EINVAL;
	}

	if (r2vers == R2_VERSION_A0)
		num_sus = TOTALSUCOUNT_A0;
	else
		num_sus = TOTALSUCOUNT;

	for (i = PPU0; i <= num_sus; i++) {

		eqsu_mbuf_table[i].mbuf = NULL;
		eqsu_mbuf_table[i].mbufsize = sizeof(su_dma_buf_t);

		if ((i >= PPU0) && (i < (PPU0 + PPU_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_PPU;
			eqsu_mbuf_table[i].suid = i - PPU0;
			continue;
		}

		if ((i >= PPU_MXL0) && (i < (PPU_MXL0 + PPU_MXL_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_PPU_MXL;
			eqsu_mbuf_table[i].suid = i - PPU_MXL0;
			continue;
		}

		if ((i >= PPU_CRSS0) && (i < (PPU_CRSS0 + PPU_CRSS_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_PPU_CRSS;
			eqsu_mbuf_table[i].suid = i - PPU_CRSS0;
			continue;
		}

		if ((i >= OTRX_HSU0) && (i < (OTRX_HSU0 + OTRX_HSU_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_OTRX_HSU;
			eqsu_mbuf_table[i].suid = i - OTRX_HSU0;
			continue;
		}

		if ((i >= OTRX_QSU0) && (i < (OTRX_QSU0 + OTRX_QSU_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_OTRX_QSU;
			eqsu_mbuf_table[i].suid = i - OTRX_QSU0;
			continue;
		}

		if ((i >= SPU0) && (i < (SPU0 + SPU_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_SPU;
			eqsu_mbuf_table[i].suid = i - SPU0;
			continue;
		}

		if ((i >= ECPRI0) && (i < (ECPRI0 + ECPRI_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_ECPRI;
			eqsu_mbuf_table[i].suid = i - ECPRI0;
			continue;
		}

		if ((i >= TXU0) && (i < (TXU0 + TXU_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_TXU;
			eqsu_mbuf_table[i].suid = i - TXU0;
			continue;
		}

		if ((i >= IRING0) && (i < (IRING0 + IRING_NUM_SUS))) {
			eqsu_mbuf_table[i].etype = ENGINE_IRING;
			eqsu_mbuf_table[i].suid = i - IRING0;
		}
	}

	return 0;
}

static bool raptor2_init_dt(struct eqsu_membuf eqsu_mbuf_table[])
{
	bool ret;
	r2_engine_t etype;
	struct device_node *en, *np;

	np = of_find_compatible_node(NULL, NULL, "risc-v,edgeq-raptor2");
	if (!np) {
		pr_err("%s:of_find_compatible_node for risc-v,edgeq-raptor2 failed\n",
									__func__);
		return false;
	}

	memset(&sudev_g, 0, sizeof(struct su_device));
	memset(&sumem_g, 0, sizeof(struct su_device));

	ret = parse_su_device_dt();
	if (ret == false) {
		pr_err("%s: parse_su_device_dt failed\n", __func__);
		return false;
	}

	ret = parse_su_memory_dt();
	if (ret == false) {
		pr_err("%s: parse_su_memory_dt failed\n", __func__);
		return false;
	}

	etype = ENGINE_PPU;
	while (etype <= ENGINE_IRING) {

		if ((r2vers == R2_VERSION_A0) && (etype == ENGINE_IRING))
			goto next;

		en = of_find_node_by_name(NULL, engstr[etype]);
		if (en == NULL) {
			pr_err("of_find_node_by_name for %s failed\n", engstr[etype]);
			goto next;
		}

		ret = parse_raptor2_engine_sus(en, etype, eqsu_mbuf_table);
		if (ret == false)
			pr_err("parse_raptor2_engine_sus for %s failed\n", engstr[etype]);
next:
		++etype;
	}

#if defined(WLAN)
	ret = parse_rtos_info_dt();
#endif

	return true;
}

static int __init raptor2_exmgr_init(void)
{
	int ret;

	r2vers = get_chip_version();
	pr_info("%s: Initializing Exmgr for Raptor2 %s\n", __func__, engstr[r2vers]);

	ret = init_eqmbuf_table(eqsu_mbuf_table);
	if (ret < 0) {
		pr_err("%s: init_eqmbuf_table failed, ret:%d\n", __func__, ret);
		return ret;
	}

	ret = raptor2_init_dt(eqsu_mbuf_table);
	if (ret == false) {
		pr_err("%s: raptor2 init dt() failed\n", __func__);
		return ret;
	}

	ret = raptor2_exmgr_ops_init(eqsu_mbuf_table, rtos_info, r2vers);
	if (ret) {
		pr_err("%s: raptor2 exmgr ops init() failed\n", __func__);
		return ret;
	}

	pr_info("%s: Exmgr Init complete\n\n", __func__);

	return ret;
}

static void __init raptor2_exmgr_cleanup(void)
{
	r2_suidx_t idx;
	struct r2_engine_su *engsu;

	raptor2_exmgr_ops_cleanup();

	for (idx = PPU0; idx < SU_END; idx++) {
		engsu = eqsu_mbuf_table[idx].engsu;
		kfree(engsu);
	}
}

module_init(raptor2_exmgr_init);
module_exit(raptor2_exmgr_cleanup);
