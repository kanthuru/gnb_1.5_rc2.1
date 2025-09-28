/*
 * EdgeQ Inc.
 *
 * Raptor2 Parse Device Tree for Micro-engines
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
#include "raptor2.h"
#include "raptor2_sysfs_ops.h"

static r2_chip_version_t r2vers;

static struct su_device sudev_g;
static struct su_memory sumem_g;

struct r2_engine_info r2_engine_info_table [] = {
	{ENGINE_PPU,		"ppu",		PPU_NUM_SUS,		NULL,	NULL},
	{ENGINE_PPU_MXL,	"ppu_mxl",	PPU_MXL_NUM_SUS,	NULL,	NULL},
	{ENGINE_PPU_CRSS,	"ppu_crss",	PPU_CRSS_NUM_SUS,	NULL,	NULL},
	{ENGINE_OTRX_HSU,	"otrx_hsu",	OTRX_HSU_NUM_SUS,	NULL,	NULL},
	{ENGINE_OTRX_QSU,	"otrx_qsu",	OTRX_QSU_NUM_SUS,	NULL,	NULL},
	{ENGINE_SPU,		"spu",		SPU_NUM_SUS,		NULL,	NULL},
	{ENGINE_ECPRI,		"ecpri",	ECPRI_NUM_SUS,		NULL,	NULL},
	{ENGINE_TXU,		"txu",		TXU_NUM_SUS,		NULL,	NULL},
	{ENGINE_IRING,		"iring",	IRING_NUM_SUS,		NULL,	NULL},
	/* Always the last, all new entries before */
	{ENGINE_NONE,		NULL,		0,			NULL,	NULL},
};

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

u32 get_membm(r2_engine_t etype, int suidx)
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

static bool parse_raptor2_engine_sus(struct device_node *enode, struct r2_engine_info *info)
{
	int rc, suidx;
	u32 vals[4];
	char compat[32];
	struct list_head *cur, *head;
	struct device_node *cn;
	struct r2_engine_su *engsu;

	memset(compat, 0, sizeof(compat));
	memset(vals, 0, sizeof(vals));

	if (enode == NULL) {
		pr_err("enode is NULL\n");
		return false;
	}

	if (info == NULL) {
		pr_err("info is NULL\n");
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

		rc = of_property_read_string(cn, "status", &status);
		if (rc < 0) {
			pr_err("failed to read status for %s\n", cn->name);
			continue;
		}

		if (strncmp(status, "okay", strlen("okay")) == 0)
			engsu->enabled = 1;
		else
			engsu->enabled = 0;

		engsu->etype = info->etype;
		engsu->su_index = ++suidx;
		engsu->membm = get_membm(info->etype, suidx);
		memcpy(&engsu->sudev, &sudev_g, sizeof(struct su_device));
		memcpy(&engsu->sumem, &sumem_g, sizeof(struct su_memory));
		strncpy(engsu->name, cn->name,
			(strlen(cn->name) >= sizeof(engsu->name)) ?
					(sizeof(engsu->name) - 1) : strlen(cn->name));

		pr_info("%s: %s SU%d\n", __func__, r2_engine_info_table[engsu->etype].ename,
									engsu->su_index);

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
		if (rc < 0) {
			pr_err("failed to get suinfo resource address for %s\n", cn->name);
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
			pr_err("failed to get lmem_wb_local resource address for %s\n",
										cn->name);
			continue;
		}
		engsu->lmemaddr_wb_local = rscaddr;
		engsu->lmemsz_wb_local = rscsize;

		rc = parse_su_resource_addr(cn, 10, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmem_nc_local resource address for %s\n",
										cn->name);
			continue;
		}
		engsu->lmemaddr_nc_local = rscaddr;
		engsu->lmemsz_nc_local = rscsize;
		
		rc = parse_su_resource_addr(cn, 11, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmem_wt_local resource address for %s\n", cn->name);
			continue;
		}
		engsu->lmemaddr_wt_local = rscaddr;
		engsu->lmemsz_wt_local = rscsize;
		
		rc = parse_su_resource_addr(cn, 12, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to get lmem_cpu resource address for %s\n", cn->name);
			continue;
		}
		engsu->lmemaddr_cpu = rscaddr;
		engsu->lmemsz_cpu = rscsize;

		rc = parse_su_resource_addr(cn, 13, &rscaddr, &rscsize);
		if (rc == -1) {
			pr_err("failed to LMEM_B2 CPU address for %s\n",
					cn->name);
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
				pr_err("failed to get CLILM Local resource address for %s\n",
						cn->name);
				continue;
			}
			engsu->clilmaddr_local = rscaddr;
			engsu->clilmsz_local = rscsize;
			pr_debug("CLILM local addr is 0x%llx and size is 0x%llx\n",
					engsu->clilmaddr_local, engsu->clilmsz_local);

			rc = parse_su_resource_addr(cn, 17, &rscaddr, &rscsize);
			if (rc == -1) {
				pr_err("failed to get CLILM CNOC resource address for %s\n",
						cn->name);
				continue;
			}
			engsu->clilmaddr_cnoc = rscaddr;
			engsu->clilmsz_cnoc = rscsize;
			pr_debug("CLILM CNOC addr is 0x%llx and size is 0x%llx\n",
					engsu->clilmaddr_cnoc, engsu->clilmsz_cnoc);
		}

		if (info->enghead == NULL) {
			info->enghead = engsu;
			INIT_LIST_HEAD(&engsu->node);
		}
		else {
			list_add(&engsu->node, &(info->enghead->node));
		}
	}

	cur = &info->enghead->node;
	head = &info->enghead->node;
	while (cur != NULL) {
		engsu = container_of(cur, struct r2_engine_su, node);
		if (cur->next == head)
			break;
		cur = cur->next;
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

static bool parse_su_device_dt(void)
{
	bool ret;
	struct device_node *rn;
	u64 rscaddr, rscsize;

	pr_info("%s\n", __FUNCTION__);
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

	sudev_g.cmu1_offs = sudev_g.cmu_offs + CMU1_OFFSET;
	sudev_g.cmu1sz = sudev_g.cmusz - CMU1_OFFSET;

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

	printk(KERN_NOTICE "%s\n", __FUNCTION__);
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

static bool raptor2_init_dt(void)
{
	int i;
	bool ret;
	struct device_node *en, *np;
	struct r2_engine_info *info;

	r2vers = get_chip_version();

	np = of_find_compatible_node(NULL, NULL, "risc-v,edgeq-raptor2");
	if (!np)
		return false;

	memset(&sudev_g, 0, sizeof(struct su_device));
	memset(&sumem_g, 0, sizeof(struct su_device));
	parse_su_device_dt();
	parse_su_memory_dt();

	i = 0;
	while (r2_engine_info_table[i].etype != ENGINE_NONE) {

		info = (struct r2_engine_info *)(&(r2_engine_info_table[i]));

		if ((r2vers == R2_VERSION_A0) &&
				(r2_engine_info_table[i].etype == ENGINE_IRING))
			goto next;

		en = of_find_node_by_name(NULL, info->ename);
		if (en == NULL) {
			pr_err("of_find_node_by_name for %s failed\n", info->ename);
			goto next;
		}

		ret = parse_raptor2_engine_sus(en, info);
		if (ret == false)
			pr_err("parse_raptor2_engine_sus for %s failed\n", info->ename);
next:
		++i;
	}

	return true;
}

static int __init raptor2_init(void)
{
	int ret;

	pr_info("Initializing SU Infra Kernel support start\n");
	printk(KERN_NOTICE "calling raptor2_init_dt\n");
	ret = raptor2_init_dt();
	if (ret == false) {
		pr_err("raptor2_init_dt() failed\n");
		return ret;
	}

	printk(KERN_NOTICE "calling raptor2_init_sysfs\n");
	ret = raptor2_init_sysfs(r2_engine_info_table, r2vers);
	if (ret == false)
		pr_err("raptor2_init_sysfs() failed\n");

	pr_info("Initializing SU Infra Kernel support complete\n\n");

	return ret;
}

static void __init raptor2_cleanup(void)
{
	int i;
	struct r2_engine_su *engsu;
	struct r2_kobj_entry *entry;

	for (i = 0; r2_engine_info_table[i].etype != ENGINE_NONE; i++) {
		engsu = r2_engine_info_table[i].enghead;
		entry = engsu->kobj_entry;
		kfree(entry);
		kfree(engsu);
	}
}

module_init(raptor2_init);
module_exit(raptor2_cleanup);

MODULE_AUTHOR("Pravin Bathija <bathija@edgeq.io>");
MODULE_DESCRIPTION("Raptor Wireless Infra driver");
MODULE_LICENSE("GPL");
