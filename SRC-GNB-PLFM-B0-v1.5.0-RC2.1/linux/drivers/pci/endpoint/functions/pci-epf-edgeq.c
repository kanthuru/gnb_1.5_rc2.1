// SPDX-License-Identifier: GPL-2.0
/**
 * Test driver to edgeq endpoint functionality
 *
 * Copyright (C) 2021 EdgeQ Inc.
 * Author: Ankit Jindal <ankit.jindal@edgeq.io>
 * Author: Pranavkumar Sawargaonkar <pranav.sawargaonkar@edgeq.io>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/uio_driver.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include "pci-epf-edgeq.h"
#include "hdma_reg.h"

typedef struct {
	L3_dw_hdma_regs_t *l3_reg;
	L4_dw_hdma_regs_t *l4_reg;
} dw_hdma_regs_t;

#define HDMA_V0_MAX_NR_CH LINK4_HDMA_V0_MAX_NR_CH
#define DRV_VERSION			"0.1"
#define RAPTOR2_EARLY_INIT		0

/* BAR mappings */
#define EDGEQ_PCIE_EP_BAR		0
#define EDGEQ_DDR_BAR			2

int link3_active;

/* PCIe Global Variables */
static struct workqueue_struct	*kpciedgeq_workqueue;
struct pci_epf			*edev;		/* Global pci epf dev */
void				*reg_base_addr;
void				*dma_reg_base_addr;
struct raptor2_prv_data		*r2dev;


/* Enable B0 HDMA Read and Write Channels */
static void dma_enable_rd_wr(phys_addr_t dma_register_addr, int dma_ch, bool link3_active)
{
	/* Use dw_hdma_regs_t to accommodate both L3 and L4 structures */
	dw_hdma_regs_t reg;
	int i = 0;

	if (link3_active)
		reg.l3_reg	= (L3_dw_hdma_regs_t *)dma_register_addr;
	else
		reg.l4_reg	= (L4_dw_hdma_regs_t *)dma_register_addr;

	for (i = 0; i < dma_ch; i++) {
		if (link3_active) {
			/* Use L3 register for Link3 */
			writel(HDMA_CH_ENABLE, &(reg.l3_reg->ch[i].wr.ch_en));
			writel(HDMA_CH_ENABLE, &(reg.l3_reg->ch[i].rd.ch_en));
			/* Logging for Link3 */
			R2_ANALYZE("Enabled Link3 HDMA WR Addr: %llx CH:%d VAL:%d\n",
				(uint64_t)&(reg.l3_reg->ch[i].wr.ch_en), i,
				readl(&(reg.l3_reg->ch[i].wr.ch_en)));
			R2_ANALYZE("Enabled Link3 HDMA RD Addr: %llx CH:%d VAL:%d\n",
				(uint64_t)&(reg.l3_reg->ch[i].rd.ch_en), i,
				readl(&(reg.l3_reg->ch[i].rd.ch_en)));
		} else {
			/* Use L4 register for Link4 */
			writel(HDMA_CH_ENABLE, &(reg.l4_reg->ch[i].wr.ch_en));
			writel(HDMA_CH_ENABLE, &(reg.l4_reg->ch[i].rd.ch_en));
			/* Logging for Link4 */
			R2_ANALYZE("Enabled Link4 HDMA WR Addr: %llx CH:%d VAL:%d\n",
				(uint64_t)&(reg.l4_reg->ch[i].wr.ch_en), i,
				readl(&(reg.l4_reg->ch[i].wr.ch_en)));
			R2_ANALYZE("Enabled Link4 HDMA RD Addr: %llx CH:%d VAL:%d\n",
				(uint64_t)&(reg.l4_reg->ch[i].rd.ch_en), i,
				readl(&(reg.l4_reg->ch[i].rd.ch_en)));
		}
	}
}

/* UIO specific code */
static void fill_uio_map(struct uio_info *info, phys_addr_t paddr, phys_addr_t vaddr,
			size_t size, char *name, int map_num, int mem_type)
{
		info->mem[map_num].addr			= paddr;
		info->mem[map_num].internal_addr		= (void *)vaddr;
		info->mem[map_num].size			= size;
		info->mem[map_num].memtype			= mem_type;
		info->mem[map_num].name			= kasprintf(GFP_KERNEL, "%s", name);

		R2_ANALYZE("UIO_MAP: paddr=%llx vaddr=%p size=%llx name=%s\n",
			info->mem[map_num].addr, info->mem[map_num].internal_addr,
			info->mem[map_num].size, info->mem[map_num].name);

		if (strcmp(name, "DMA_LL") != 0)
			test_read(vaddr, 8*3);
}

static int check_flag(void *data)
{
	struct nlmsghdr *nlh;
	struct edgeq_user_data *udata;
	struct wait_flags *in_wflg;
	struct wait_flags *out_wflg;
	struct sk_buff *skb_out;
	int pid;
	int ret;
	int dev_id;
	bool ready_flg = 0;

	if (data == NULL) {
		R2_LOG("DEBUG: Data for check flag is NULL\n");
		return -1;
	}

	nlh = (struct nlmsghdr *)data;
	pid = nlh->nlmsg_pid;

	udata		= nlmsg_data(nlh);
	in_wflg	= (struct wait_flags *)udata->data;
	dev_id		= in_wflg->dev_id;
	out_wflg	= devm_kzalloc(r2dev->dev, sizeof(struct wait_flags), GFP_KERNEL);
	print_recv_flags(in_wflg);
	if (in_wflg->linkup_ready)
		out_wflg->linkup_ready				= ready_flg
		= (r2dev->linkup_ready == true)?true:false;
	else if (in_wflg->is_local_global_mem_mapped)
		out_wflg->is_local_global_mem_mapped		= ready_flg
		= (r2dev->exposed_local_glob_region == true)?true:false;
	else if (in_wflg->is_remote_global_mem_mapped)
		out_wflg->is_remote_global_mem_mapped		= ready_flg
		= (r2dev->exposed_remote_glob_region == true)?true:false;
	else if (in_wflg->is_local_device_ready)
		out_wflg->is_local_device_ready			= ready_flg
		= (r2dev->exposed_local_device[dev_id] == true)?true:false;
	else {
		R2_ANALYZE("DEBUG: No flag requested\n");
		ret = 0;
		goto ntlnk_end;
	}

	out_wflg->dev_id = dev_id;

	if (!ready_flg) {
		R2_LOG("DEBUG: Flag not ready. Rescheduling\n");
		r2dev->edgeq_work->req_type = POLL_FLAG;
		queue_delayed_work(r2dev->r2_wrkq, &r2dev->edgeq_work->cmd_handler,
				msecs_to_jiffies(1000));
		ret = -1;
	} else {
		R2_LOG("DEBUG: Flag ready, sending message to userspace\n");
		skb_out = nlmsg_new(sizeof(struct wait_flags), GFP_KERNEL);
		if (!skb_out) {
			R2PR_LOG_ERR("EDGEQ: Failed to allocate new skb\n");
			ret = -2;
			goto ntlnk_end;
		}
		nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, sizeof(struct wait_flags), 0);
		NETLINK_CB(skb_out).dst_group = 0;
		memset(nlmsg_data(nlh), 0, sizeof(struct wait_flags));
		memcpy(nlmsg_data(nlh), out_wflg, sizeof(struct wait_flags));
		ret = nlmsg_unicast(r2dev->netlink_sk, skb_out, pid);
		R2_ANALYZE("DEBUG: Send message. ret=%d", ret);
		devm_kfree(r2dev->dev, r2dev->edgeq_work->data);
	}

ntlnk_end:
	devm_kfree(r2dev->dev, out_wflg);
	return ret;
}

static void ftl_edgeq_cmd_handler(struct work_struct *work)
{
	struct edgeq_ucmd_work *eq_work;
	int ret = 0;

	/* 1. Get the edgeq_ucmd_work and r2dev from the work */
	eq_work = container_of((struct delayed_work *)work, struct edgeq_ucmd_work,
				cmd_handler);

	/* 2. Do work as per request */
	switch (eq_work->req_type) {
	case NO_OP:
		R2_VERBOSE("EDGEQ: No Operation Request\n");
		break;
	case POLL_FLAG:
		R2_LOG("EDGEQ: Poll flag Operation Request\n");
		ret = check_flag(eq_work->data);
		break;
	default:
		R2_VERBOSE("EDGEQ: Unknown request type %d\n", eq_work->req_type);
		break;
	}

	if (ret == -1)
		R2_LOG("EDGEQ: Rescheduling work\n");
	else if (ret < -1)
		R2_LOG("EDGEQ: Failed to respond to message\n");
	else
		R2_LOG("EDGEQ: Message sent\n");
}

struct netlink_kernel_cfg netlink_cfg = {
	.input = edgeq_netlink_recv,
};

static void print_fdesc(struct ftl_descriptor *fdesc)
{
	R2_ANALYZE("########FTL Descriptor########\n");
	R2_ANALYZE("num deviceid	=%lld\n", fdesc->num_devices);
	R2_ANALYZE("queue addr	=0x%llx\n", fdesc->queue_region_addr);
	R2_ANALYZE("queue size	=%lld\n", fdesc->queue_region_size);
	R2_ANALYZE("dma_read ch	=%lld\n", fdesc->num_dma_read_ch);
	R2_ANALYZE("dma write ch	=%lld\n", fdesc->num_dma_write_ch);
	R2_ANALYZE("DMA addr	=0x%llx\n", fdesc->dma_region_addr);
	R2_ANALYZE("DMA size	=%lld\n", fdesc->dma_region_size);
	R2_ANALYZE("SEQT Enable	=%lld\n", fdesc->seqt_enable);
	R2_ANALYZE("##############################\n");
}

static void fill_glob(struct ftl_descriptor *fdesc, global_t *glb)
{
	glb->num_devices	= fdesc->num_devices;

	glb->queue_region_addr	= fdesc->queue_region_addr;
	glb->queue_region_size	= fdesc->queue_region_size;
	memset(&(glb->queue_map), 0, sizeof(uint64_t));

	glb->num_dma_read_ch	= fdesc->num_dma_read_ch;
	glb->num_dma_write_ch	= fdesc->num_dma_write_ch;
	memset(&(glb->dma_read_chan_map), 0, sizeof(uint64_t));
	memset(&(glb->dma_write_chan_map), 0, sizeof(uint64_t));

	glb->dma_region_addr	= 0;
	glb->dma_region_size	= 0;

	glb->dma_fault		= 0;
	glb->seqt_enable	= fdesc->seqt_enable;
}

static int expose_global_uio(struct uio_info *info, struct raptor2_plat_pcie *raptor2_plat_pcie)
{
	int ret		= 0;

	info->name	= "LOCAL_GLOB";
	info->version	= "0.1";
	fill_uio_map(info, raptor2_plat_pcie->ddr_addr + raptor2_plat_pcie->ep_off_addr
		, (phys_addr_t)reg_base_addr, GLOB_TOTAL_SIZE, "global", 0, UIO_MEM_IOVA);

	ret = uio_register_device(r2dev->dev, info);
	if (ret >= 0)
		pr_info("Created local global UIO node - %s\n",
			info->name);
	else
		R2PR_LOG_ERR("Failed to create local global UIO node - %s\n",
			info->name);
	return ret;
}

static int set_global_params(char *data)
{
	int ret					= 0;
	struct ftl_descriptor *fdesc			= (struct ftl_descriptor *)data;
	struct pci_epf_edgeq *epf_edgeq		= epf_get_drvdata(edev);
	const struct pci_epc_features *epc_features	= epf_edgeq->epc_features;
	struct raptor2_plat_pcie *raptor2_plat_pcie	= (struct raptor2_plat_pcie *)
		epc_features->private;
	print_fdesc(fdesc);

	/* 1.a. Check if linkup is done or not. */
	if (r2dev->linkup_ready == false) {
		R2PR_LOG_ERR("Link up not done yet\n");
		return 0;
	}

	/* 1.b. Check if the global has been initialized before. */
	if (r2dev->exposed_local_glob_region == true) {
		R2_LOG("Local GLOB already initialized, reinitializing only parameters\n");
		r2dev->exposed_local_glob_region		= false;
		r2dev->local_gbl->glob->num_queues		= 16;
		r2dev->local_gbl->glob->available_queues	= 16;
		memset(&r2dev->local_gbl->glob->queue_map, 0x0, sizeof(uint64_t));
		r2dev->local_gbl->glob->num_dma_read_ch	= raptor2_plat_pcie->dma_ch;
		r2dev->local_gbl->glob->available_read_ch	= raptor2_plat_pcie->dma_ch;
		memset(&r2dev->local_gbl->glob->dma_read_chan_map, 0x0, sizeof(uint64_t));
		r2dev->local_gbl->glob->num_dma_write_ch	= raptor2_plat_pcie->dma_ch;
		r2dev->local_gbl->glob->available_write_ch	= raptor2_plat_pcie->dma_ch;
		memset(&r2dev->local_gbl->glob->dma_write_chan_map, 0x0, sizeof(uint64_t));
		r2dev->local_gbl->glob->seqt_enable		= fdesc->seqt_enable;
		r2dev->local_gbl->glob->dma_fault		= 0;
		r2dev->exposed_local_glob_region		= true;
		return 0;
	}

	/* 2. Save the queue region information passed */
	r2dev->local_gbl->queue_region.paddr	= fdesc->queue_region_addr;
	r2dev->local_gbl->queue_region.size	= fdesc->queue_region_size;

	/* LL Region as allocated from kernel reserved region */
	r2dev->local_gbl->ll_region.paddr	= raptor2_plat_pcie->dma_ll_addr;
	r2dev->local_gbl->ll_region.size	= (raptor2_plat_pcie->max_ch_mem/2);

	/* Remote LL region */
	r2dev->remote_gbl->ll_region.paddr	= raptor2_plat_pcie->dma_ll_addr
		+ (raptor2_plat_pcie->max_ch_mem/2);
	r2dev->remote_gbl->ll_region.size	= (raptor2_plat_pcie->max_ch_mem/2);

	/* 3. Fill the global structure on the local side */
	fill_glob(fdesc, r2dev->local_gbl->glob);

	/* 4. Expose the Local global structure through uio
	 * Set global mappped = true - This will unblock the work item.
	 */
	ret = expose_global_uio(r2dev->local_gbl->gdata_info, raptor2_plat_pcie);
	if (ret < 0) {
		R2PR_LOG_ERR("Failed to initialize local glb register space\n");
		return -1;
	}
	r2dev->local_gbl->is_global_mem_mapped		= true;

	/* 5. Set status as map in progress */
	r2dev->local_gbl->glob->status			|= STATUS_EP_MAP_IN_PROGRESS;

	r2dev->local_gbl->glob->num_queues		= 16;
	r2dev->local_gbl->glob->available_queues	= 16;
	memset(&r2dev->local_gbl->glob->queue_map, 0x0, sizeof(uint64_t));

	r2dev->local_gbl->glob->num_dma_read_ch	= raptor2_plat_pcie->dma_ch;
	r2dev->local_gbl->glob->available_read_ch	= raptor2_plat_pcie->dma_ch;
	memset(&r2dev->local_gbl->glob->dma_read_chan_map, 0x0, sizeof(uint64_t));

	r2dev->local_gbl->glob->num_dma_write_ch	= raptor2_plat_pcie->dma_ch;
	r2dev->local_gbl->glob->available_write_ch	= raptor2_plat_pcie->dma_ch;
	memset(&r2dev->local_gbl->glob->dma_write_chan_map, 0x0, sizeof(uint64_t));

	r2dev->local_gbl->dma_reg_region.paddr		= raptor2_plat_pcie->hdma_reg_addr;
	r2dev->local_gbl->dma_reg_region.vaddr		= (phys_addr_t) dma_reg_base_addr;
	r2dev->local_gbl->dma_reg_region.size		= raptor2_plat_pcie->hdma_reg_size;

	/* 6. Schedule the work for the first time. */
	queue_delayed_work(kpciedgeq_workqueue, &epf_edgeq->cmd_handler,
				msecs_to_jiffies(1000));

	r2dev->exposed_local_glob_region = true;
	R2_ANALYZE("%s:%d: Completed local initialization\n", __func__, __LINE__);
	return 0;
}

/***************** Device init request ****************/
static void print_devdesc(struct device_descriptor *desc)
{
	R2_ANALYZE("############ DEVICE DESC ###########\n");
	R2_ANALYZE("device id	= %lld\n", desc->device_id);
	R2_ANALYZE("num_queues	= %lld\n", desc->num_queues);
	R2_ANALYZE("dma_read_chan	= %lld\n", desc->num_dma_read_chan);
	R2_ANALYZE("dma_write_chan	= %lld\n", desc->num_dma_write_chan);
	R2_ANALYZE("dma_cpu id	= %lld\n", desc->cpu_id);
	R2_ANALYZE("####################################\n");
}

static int init_devices(char *data)
{
	struct raptor2_device *dev_r2;
	glob_device_t *lgdev;
	global_t *rglob;
	global_t *lglob;
	struct device_descriptor *dev_desc = (struct device_descriptor *)data;
	phys_addr_t reg_paddr, reg_vaddr, reg_size;
	uint64_t i, req_queues, ret;
	uint64_t req_rd_chan, req_wr_chan;
	uint16_t max_dma_ch;

	max_dma_ch = ((link3_active)?(LINK3_HDMA_V0_MAX_NR_CH):(LINK4_HDMA_V0_MAX_NR_CH));

	/* Check if LOCAL GLOB REGION is ready */
	if (r2dev->exposed_local_glob_region) {
		R2_LOG("%s:LOCAL GLOB region ready!!!!\n", __func__);
	}  else {
		R2_ANALYZE("%s:LOCAL GLOB region not ready yet\n", __func__);
		return 0;
	}

	if (r2dev->exposed_remote_glob_region) {
		R2_LOG("%s:Remote GLOB region ready!!!!\n", __func__);
	} else {
		R2_ANALYZE("%s:Remote GLOB region not ready!!!!\n", __func__);
		return 0;
	}

	print_devdesc(dev_desc);

	/* 1. Check if the device is available */
	if ((r2dev->exposed_local_device[dev_desc->device_id] == true) ||
	 (r2dev->raptor2_ldev[dev_desc->device_id] != NULL)) {
		R2_LOG("Device already registered. Reinitializing parameters\n");
		lglob = r2dev->local_gbl->glob;
		lgdev = (glob_device_t *)((phys_addr_t)r2dev->local_gbl->glob
			+ GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id));

		/* 0. Unset device ready */
		r2dev->exposed_local_device[dev_desc->device_id] = false;

		/* 1. Reconfigure Queues */
		memset(&lgdev->dev_qm.queue_map, 0x0, sizeof(uint64_t));
		req_queues = dev_desc->num_queues;
		for (i = 0; i < MAX_QUEUES; i++) {
			if (!((lglob->queue_map >> i)&0x01) && (req_queues > 0)) {
				req_queues--;
				lglob->queue_map		|= (1<<i);
				lgdev->dev_qm.queue_map	|= (1<<i);
			}
		}
		if ((lglob->available_queues-dev_desc->num_queues) < 0)
			R2PR_LOG("Error num_queues parameter for device %lld\n",
				dev_desc->device_id);
		else {
			lglob->available_queues		-= dev_desc->num_queues;
			lgdev->dev_qm.num_queues		= dev_desc->num_queues;
		}

		/* 2. Reconfigure DMA Channel */
		memset(&lgdev->dev_dma.dma_read_chan_map, 0x0, sizeof(uint64_t));
		req_rd_chan = dev_desc->num_dma_read_chan;
		req_wr_chan = dev_desc->num_dma_write_chan;

		for (i = 0; i < max_dma_ch; i++) {
			if (!((lglob->dma_read_chan_map >> i)&0x01) && (req_rd_chan > 0)) {
				req_rd_chan--;
				lglob->dma_read_chan_map		|= (1<<i);
				lgdev->dev_dma.dma_read_chan_map	|= (1<<i);
			}

			if (!((lglob->dma_write_chan_map >> i)&0x01) && (req_wr_chan > 0)) {
				req_wr_chan--;
				lglob->dma_write_chan_map		|= (1<<i);
				lgdev->dev_dma.dma_write_chan_map	|= (1<<i);
			}
		}

		if (((lglob->available_read_ch - dev_desc->num_dma_read_chan) < 0) ||
			((lglob->available_write_ch - dev_desc->num_dma_write_chan) < 0))
			R2PR_LOG("Error num of dma channels parameter for device %lld\n",
				dev_desc->device_id);
		else {
			lglob->available_read_ch		-= dev_desc->num_dma_read_chan;
			lglob->available_write_ch		-= dev_desc->num_dma_write_chan;
			lgdev->dev_dma.num_read_channel	+= dev_desc->num_dma_read_chan;
			lgdev->dev_dma.num_write_channel	+= dev_desc->num_dma_write_chan;
		}

		/* 3. Reconfigure CPU ID */
		lgdev->dev_dma.dma_cpu_id				= dev_desc->cpu_id;

		/* 4. Set device ready */
		r2dev->exposed_local_device[dev_desc->device_id]	= true;
		return 0;
	}

	/* 2. Allocate the device structure */
	r2dev->raptor2_ldev[dev_desc->device_id] = devm_kzalloc(r2dev->dev,
						sizeof(struct raptor2_device), GFP_KERNEL);
	if (!r2dev->raptor2_ldev[dev_desc->device_id]) {
		R2PR_LOG_ERR("Failed to allocate device context structure\n");
		return -1;
	}

	dev_r2			= r2dev->raptor2_ldev[dev_desc->device_id];
	dev_r2->is_claimed	= true;

	dev_r2->dev_info	= devm_kzalloc(r2dev->dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!dev_r2->dev_info) {
		devm_kfree(r2dev->dev, dev_r2);
		R2PR_LOG_ERR("Failed to allocate device info structure\n");
		return -1;
	}
	dev_r2->dev_info->name	= kasprintf(GFP_KERNEL, "DEVICE_%lld", dev_desc->device_id);
	dev_r2->dev_info->version = "0.1";

	/* MAP 0: LOCAL DEV REG SPACE */
	reg_paddr		= r2dev->local_gbl->glob_paddr +
		GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id);

	reg_vaddr		= (phys_addr_t)r2dev->local_gbl->glob +
		GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id);

	lgdev			= (glob_device_t *)reg_vaddr;
	lgdev->device_id	= dev_desc->device_id;

	lglob			= r2dev->local_gbl->glob;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			GLOB_DEVICE_SIZE, "LOCAL_DEV", 0, UIO_MEM_IOVA);
	dev_r2->is_local_dreg_ready = true;

	/* MAP 1: REMOTE DEV REG SPACE */
	reg_paddr = r2dev->remote_gbl->glob_paddr
	 + (GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id));

	reg_vaddr = (phys_addr_t)r2dev->remote_gbl->glob
	 + (GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id));

	rglob = r2dev->remote_gbl->glob;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			GLOB_DEVICE_SIZE, "REMOTE_DEV", 1, UIO_MEM_PHYS_PFETCH);
	dev_r2->is_remote_dreg_ready = true;

	/* MAP 2: REMOTE QUEUE SPACE */
	/* Setup queue bitmap */
	lgdev->dev_qm.queue_map = 0x00;
	req_queues = dev_desc->num_queues;
	for (i = 0; i < MAX_QUEUES; i++) {
		if (!((lglob->queue_map >> i)&0x01) && (req_queues > 0)) {
			req_queues--;
			lglob->queue_map |= (1<<i);
			lgdev->dev_qm.queue_map |= (1<<i);
		}
	}
	if ((lglob->available_queues-dev_desc->num_queues) < 0)
		R2PR_LOG_ERR("Error num_queues parameter for device %lld\n", dev_desc->device_id);
	else {
		lglob->available_queues -= dev_desc->num_queues;
		lgdev->dev_qm.num_queues = dev_desc->num_queues;
	}
	R2_LOG("LOCAL: GLOB QBMP:%llx DEV QBMP:%llx\n",
				lglob->queue_map, lgdev->dev_qm.queue_map);

	reg_paddr = r2dev->remote_gbl->queue_region.paddr;
	reg_vaddr = r2dev->remote_gbl->queue_region.vaddr;
	reg_size  = r2dev->remote_gbl->queue_region.size;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			reg_size, "REMOTE_QUEUE", 2, UIO_MEM_PHYS_PFETCH);
	dev_r2->is_remote_queue_ready = true;

	/* MAP 3: DMA REG SPACE */
	/* Setup dma read and write channel bitmap */
	lgdev->dev_dma.dma_read_chan_map			= 0x0;
	req_rd_chan						= dev_desc->num_dma_read_chan;
	req_wr_chan						= dev_desc->num_dma_write_chan;

	for (i = 0; i < max_dma_ch; i++) {
		if (!((lglob->dma_read_chan_map >> i)&0x01) && (req_rd_chan > 0)) {
			req_rd_chan--;
			lglob->dma_read_chan_map		|= (1<<i);
			lgdev->dev_dma.dma_read_chan_map	|= (1<<i);
		}

		if (!((lglob->dma_write_chan_map >> i)&0x01) && (req_wr_chan > 0)) {
			req_wr_chan--;
			lglob->dma_write_chan_map		|= (1<<i);
			lgdev->dev_dma.dma_write_chan_map	|= (1<<i);
		}
	}

	R2_LOG("LOCAL: GLOB DMA_READ_BMAP:%llx DEV GLOB DMA_READ_BMAP:%llx\n",
		lglob->dma_read_chan_map, lgdev->dev_dma.dma_read_chan_map);
	R2_LOG("LOCAL: GLOB DMA_WRITE_BMAP:%llx DEV GLOB DMA_WRITE_BMAP:%llx\n",
		lglob->dma_write_chan_map, lgdev->dev_dma.dma_write_chan_map);

	if (((lglob->available_read_ch - dev_desc->num_dma_read_chan) < 0) ||
		((lglob->available_write_ch - dev_desc->num_dma_write_chan) < 0))
		R2PR_LOG_ERR("Error num of dma channels parameter for device %lld\n",
			dev_desc->device_id);
	else {
		lglob->available_read_ch		-= dev_desc->num_dma_read_chan;
		lglob->available_write_ch		-= dev_desc->num_dma_write_chan;
		lgdev->dev_dma.num_read_channel		+= dev_desc->num_dma_read_chan;
		lgdev->dev_dma.num_write_channel	+= dev_desc->num_dma_write_chan;
	}

	lgdev->dev_dma.dma_cpu_id = dev_desc->cpu_id;
	R2_ANALYZE("Reserved CPU#%lld for device %lld\n",
		lgdev->dev_dma.dma_cpu_id, dev_desc->device_id);


	reg_paddr = r2dev->local_gbl->dma_reg_region.paddr;
	reg_vaddr = r2dev->local_gbl->dma_reg_region.vaddr;
	reg_size  = r2dev->local_gbl->dma_reg_region.size;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			reg_size, "DMA_REG", 3, UIO_MEM_PHYS);
	dev_r2->is_dma_reg_ready = true;

	/* MAP 4: DMA LL REGION */
	reg_paddr = r2dev->local_gbl->ll_region.paddr;
	reg_vaddr = (phys_addr_t)memremap((phys_addr_t)r2dev->local_gbl->ll_region.paddr,
					r2dev->local_gbl->ll_region.size, MEMREMAP_WB);
	if (!reg_vaddr) {
		R2PR_LOG_ERR("Failed to map DMA LL region\n");
		devm_kfree(r2dev->dev, dev_r2->dev_info);
		devm_kfree(r2dev->dev, dev_r2);
		return -1;
	}
	reg_size  = r2dev->local_gbl->ll_region.size;

	fill_uio_map(dev_r2->dev_info, reg_vaddr, 0,
			reg_size, "DMA_LL", 4, UIO_MEM_LOGICAL);
	dev_r2->is_dma_reg_ready = true;

	/* UIO register */
	ret = uio_register_device(r2dev->dev, dev_r2->dev_info);
	if (ret >= 0)
		R2_ANALYZE("Created UIO node - %s\n",
			dev_r2->dev_info->name);
	else {
		R2PR_LOG_ERR("Failed to create UIO node - %s\n",
			dev_r2->dev_info->name);
		iounmap((void *)reg_vaddr);
		devm_kfree(r2dev->dev, dev_r2->dev_info);
		devm_kfree(r2dev->dev, dev_r2);
		return -1;
	}
	dev_r2->is_device_init_done				= true;
	r2dev->exposed_local_device[dev_desc->device_id]	= true;
	return 0;
}

/********************* Check Flag request *************/
static void check_for_flag(struct sk_buff *sbuff)
{
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *)sbuff->data;

	/* Queue work item to check for flags */
	r2dev->edgeq_work->req_type = POLL_FLAG;
	r2dev->edgeq_work->data = devm_kzalloc(r2dev->dev, nlh->nlmsg_len, GFP_KERNEL);
	memcpy(r2dev->edgeq_work->data, nlh, nlh->nlmsg_len);

	queue_delayed_work(r2dev->r2_wrkq, &r2dev->edgeq_work->cmd_handler,
			msecs_to_jiffies(1000));
}

static void edgeq_netlink_recv(struct sk_buff *skb)
{
	int ret = 0;
	struct nlmsghdr *nlh;
	struct edgeq_user_data *udata;
	int pid;

	nlh = (struct nlmsghdr *)skb->data;
	pid = nlh->nlmsg_pid;

	udata = nlmsg_data(nlh);
	if (!udata) {
		R2PR_LOG_ERR("EdegQ Netlink No netlink data\n");
		return;
	}

	R2_LOG("EdgeQ Netlink Request type = %d\n", udata->req_type);
	switch (udata->req_type) {
	case REQ_SET_FTL_DESC:			/* First request from userspace */
		set_global_params(udata->data);
		break;

	case REQ_INIT_DEV:
		ret = init_devices(udata->data);/* Second request from userspace */
		break;

	case REQ_WAIT_DESC:			/* Check flags request */
		check_for_flag(skb);
		break;
	default:
		R2PR_LOG_ERR("Unknown Request type");
		break;
	}

	if (ret < 0)
		R2PR_LOG_ERR("Failed to respond to request%d\n", udata->req_type);
	else
		R2PR_LOG_ERR("Responded to request%d\n", udata->req_type);
}

/****************************************************
 * @desc: Allocating Raptor2 specific data structure
 *
 * @returns: < 0 in case of error, else 0
 ***************************************************/
static int init_infra(struct device *dev)
{
	/* 0. Allocate the r2dev structure */
	r2dev = devm_kzalloc(dev, sizeof(struct raptor2_prv_data), GFP_KERNEL);
	if (!r2dev) {
		R2PR_LOG_ERR("%s: Failed to alloc_workqueue\n", __func__);
		return -1;
	}

	/* 1. Allocate the work queue for Raptor2 use case. */
	r2dev->r2_wrkq = alloc_workqueue("r2_workq", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!r2dev->r2_wrkq) {
		devm_kfree(r2dev->dev, r2dev);
		R2PR_LOG_ERR("%s: Failed to alloc_workqueue\n", __func__);
		return -2;
	}

	/* 2. Allocate the work item */
	r2dev->edgeq_work = devm_kzalloc(dev, sizeof(struct edgeq_ucmd_work), GFP_KERNEL);
	if (!r2dev->edgeq_work) {
		R2PR_LOG_ERR("Unable to allocate structure for edgeq work.\n");
		destroy_workqueue(r2dev->r2_wrkq);
		devm_kfree(r2dev->dev, r2dev);
		return -3;
	}
	INIT_DELAYED_WORK(&r2dev->edgeq_work->cmd_handler, ftl_edgeq_cmd_handler);

	/* 3. Setup netlink socket */
	R2_ANALYZE("%s: Netlink %d", __func__, NETLINK_EDGEQ_PCI(link3_active));
	r2dev->netlink_sk = netlink_kernel_create(&init_net,
	NETLINK_EDGEQ_PCI(link3_active), &netlink_cfg);
	if (r2dev->netlink_sk == NULL) {
		R2PR_LOG_ERR("Failed to create netlink socket.\n");
		devm_kfree(r2dev->dev, r2dev->edgeq_work);
		destroy_workqueue(r2dev->r2_wrkq);
		devm_kfree(r2dev->dev, r2dev);
		return -4;
	}
	return 0;
}

static int raptor2_pci_init(void *private)
{
	struct raptor2_plat_pcie *raptor2_plat_pcie = (struct raptor2_plat_pcie *)private;
	/* Raptor2 Init code */
	/* 1. Allocate the local and remote glb structure */
	r2dev->local_gbl = devm_kzalloc(r2dev->dev, sizeof(struct raptor2_global), GFP_KERNEL);
	if (!r2dev->local_gbl) {
		R2_LOG("Failed to allocate local context structure.\n");
		return -3;
	}

	r2dev->remote_gbl = devm_kzalloc(r2dev->dev, sizeof(struct raptor2_global), GFP_KERNEL);
	if (!r2dev->remote_gbl) {
		devm_kfree(r2dev->dev, r2dev->local_gbl);
		R2_LOG("Failed to allocate remote context structure.\n");
		return -3;
	}

	r2dev->local_gbl->gdata_info = devm_kzalloc(r2dev->dev, sizeof(struct uio_info), GFP_KERNEL);
	if (!r2dev->local_gbl->gdata_info) {
		R2_LOG("Failed to allocate global uio structure.\n");
		devm_kfree(r2dev->dev, r2dev->local_gbl);
		devm_kfree(r2dev->dev, r2dev->remote_gbl);
		return -3;
	}

	/* 2. Initialize the global structure */
	r2dev->local_gbl->glob				= (global_t *) reg_base_addr;
	r2dev->local_gbl->glob_paddr			= raptor2_plat_pcie->ddr_addr
	+ raptor2_plat_pcie->ep_off_addr;
	r2dev->local_gbl->glob->status			= STATUS_EP_NOT_READY;
	r2dev->local_gbl->glob->num_devices		= MAX_EP_FUNC_DEVS;
	r2dev->local_gbl->glob->ep_bar_ddr_paddr	= raptor2_plat_pcie->ddr_addr;

	pr_info("Completed Raptor2 pci Init\n");
	return 0;
}

static struct pci_epf_header edgeq_header = {
	.vendorid		= PCI_VENDOR_ID_EDGEQ,
	.deviceid		= PCI_ANY_ID,
	.cache_line_size	= 64,
	.baseclass_code	= PCI_CLASS_OTHERS,
	.interrupt_pin		= PCI_INTERRUPT_INTA,
};

static int edgeq_map_host_mem(struct pci_epf_edgeq *epf_edgeq,
				phys_addr_t pci_addr, size_t size,
				phys_addr_t *map_paddr, phys_addr_t *map_addr)
{
	int ret;
	phys_addr_t phys_addr;
	struct pci_epf *epf	= epf_edgeq->epf;
	struct device *dev	= &epf->dev;
	struct pci_epc *epc	= epf->epc;

	*map_addr = (phys_addr_t)pci_epc_mem_alloc_addr(epc, &phys_addr, size);
	if (!map_addr) {
		dev_err(dev, "Failed to allocate address\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr,
				pci_addr, size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		goto err_addr;
	}

	*map_paddr = phys_addr;

	return 0;
err:
err_addr:
	return -1;
}

static void pci_epf_edgeq_cmd_handler(struct work_struct *work)
{
	struct pci_epf_edgeq *epf_edgeq = container_of(work, struct pci_epf_edgeq, cmd_handler.work);
	struct raptor2_plat_pcie *raptor2_plat_pcie = (struct raptor2_plat_pcie *)
		epf_edgeq->epc_features->private;
	int ret;

	/* Wait for userspace to send the info for the GLOBAL structure*/
	if (!r2dev->local_gbl->is_global_mem_mapped) {
		R2PR_LOG_ERR("Global not mapped yet\n");
		goto reset_handler;	/* Reschedule this work item */
	}

	/* Check if the remote memory regions are already present */
	if (r2dev->remote_gbl->is_global_mem_mapped &&
		r2dev->remote_gbl->is_qm_mem_mapped)
		goto completed;		/* Exit */

	/* Initialize the remote global mem region and expose the uio device */
	if (r2dev->remote_gbl->is_global_mem_mapped == false) {
		/* Wait for the host driver to fill this field in the EP global structure. */
		/* Read Local GBL */
		R2_VERBOSE("Trying to map the remote global region");

		if (!r2dev->local_gbl->glob->host_global_paddr)
			goto reset_handler;
		R2_LOG("Got host side paddr in local GLOB");

		if (!r2dev->local_gbl->glob->host_global_size) {
			R2PR_LOG_ERR("No size specified for host global data\n");
			goto reset_handler;
		}
		R2_LOG("Got host side size in local GLOB");

		r2dev->local_gbl->glob->status |= STATUS_EP_MAP_IN_PROGRESS;

		ret = edgeq_map_host_mem(epf_edgeq,
					r2dev->local_gbl->glob->host_global_paddr,
					r2dev->local_gbl->glob->host_global_size,
					&r2dev->remote_gbl->ddr_region.paddr,
					&r2dev->remote_gbl->ddr_region.vaddr);

		if (ret) {
			R2PR_LOG_ERR("Failed to map Host global paddr\n");
			r2dev->local_gbl->glob->host_global_paddr	= 0;
			r2dev->local_gbl->glob->host_global_size	= 0;
			r2dev->local_gbl->glob->status			|= STATUS_EP_MAP_FAIL;
			goto reset_handler;
		} else {
			R2_ANALYZE(" Global paddr remote pci address 0x%llx",
				r2dev->local_gbl->glob->host_global_paddr);
			R2_ANALYZE(" mapped to local ob paddress:virtaddr 0x%llx:0x%llx\n",
				 r2dev->remote_gbl->ddr_region.paddr,
				 r2dev->remote_gbl->ddr_region.vaddr);

			r2dev->local_gbl->glob->remote_glob_lpaddr =
				r2dev->remote_gbl->ddr_region.paddr;

			r2dev->remote_gbl->glob_paddr =
				r2dev->remote_gbl->ddr_region.paddr;
			r2dev->remote_gbl->glob =
				(global_t *) r2dev->remote_gbl->ddr_region.vaddr;
			R2_ANALYZE("Remote GLOBAL Region = %llx\n",
				(phys_addr_t)r2dev->remote_gbl->glob);

			r2dev->remote_gbl->gdata_info =
				devm_kzalloc(r2dev->dev,
					sizeof(struct uio_info), GFP_KERNEL);
			r2dev->remote_gbl->gdata_info->name = "REMOTE_GLOB";
			r2dev->remote_gbl->gdata_info->version = "0.0.1";

			fill_uio_map(r2dev->remote_gbl->gdata_info, r2dev->remote_gbl->ddr_region.paddr,
				(phys_addr_t)r2dev->remote_gbl->glob, GLOB_TOTAL_SIZE, "GLOBAL",
					0, UIO_MEM_PHYS_PFETCH);

			ret = uio_register_device(r2dev->dev,
				r2dev->remote_gbl->gdata_info);
			if (ret >= 0) {
				R2_ANALYZE("Created global UIO node - %s\n",
					r2dev->remote_gbl->gdata_info->name);
				r2dev->remote_gbl->is_global_mem_mapped = true;
			} else {
				R2PR_LOG_ERR("Failed to create global UIO node - %s\n",
					r2dev->remote_gbl->gdata_info->name);
				devm_kfree(r2dev->dev, r2dev->remote_gbl->gdata_info);
				goto reset_handler;
			}
		}
	}

	/* Initialize the remote queue mem region */
	if (r2dev->remote_gbl->is_qm_mem_mapped == false) {
		if (!r2dev->remote_gbl->glob)
			goto reset_handler;

		if (!r2dev->remote_gbl->glob->queue_region_addr)
			goto reset_handler;

		if (!r2dev->remote_gbl->glob->queue_region_size)
			goto reset_handler;

		R2_VERBOSE("Trying to map queue region\n");

		ret = edgeq_map_host_mem(epf_edgeq,
				r2dev->remote_gbl->glob->queue_region_addr,
				r2dev->remote_gbl->glob->queue_region_size,
				&r2dev->remote_gbl->queue_region.paddr,
				&r2dev->remote_gbl->queue_region.vaddr);

		r2dev->remote_gbl->queue_region.size = r2dev->remote_gbl->glob->queue_region_size;

		if (ret) {
			R2PR_LOG_ERR("Failed to map Host QM paddr\n");
			goto reset_handler;
		} else {
			R2_ANALYZE("Host QM paddr remote pci address 0x%llx",
			 r2dev->remote_gbl->glob->queue_region_addr);
			R2_ANALYZE("mapped to local ob paddress:virtaddr 0x%llx:0x%llx\n",
				r2dev->remote_gbl->queue_region.paddr,
				r2dev->remote_gbl->queue_region.vaddr);
			r2dev->remote_gbl->is_qm_mem_mapped = true;
		}

		/* Fill up the ll region info in remote global reg space*/
		r2dev->remote_gbl->glob->dma_region_addr = r2dev->remote_gbl->ll_region.paddr;
		r2dev->remote_gbl->glob->dma_region_size = r2dev->remote_gbl->ll_region.size;
		R2_ANALYZE("Remote LL Region: Addr-%llx Size-%llx\n",
			r2dev->remote_gbl->glob->dma_region_addr, r2dev->remote_gbl->glob->dma_region_size);

		dma_enable_rd_wr((phys_addr_t)dma_reg_base_addr,
			raptor2_plat_pcie->dma_ch, link3_active);

		r2dev->local_gbl->glob->status		|= STATUS_EP_MAP_DONE;
		r2dev->remote_gbl->is_qm_mem_mapped	= true;
		r2dev->exposed_remote_glob_region	= true;
	}

completed:
	return;

reset_handler:
	queue_delayed_work(kpciedgeq_workqueue, &epf_edgeq->cmd_handler,
			   msecs_to_jiffies(1000));
}

static void pci_epf_edgeq_unbind(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	struct pci_epc *epc		= epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;
	int i = 0;

	cancel_delayed_work(&epf_edgeq->cmd_handler);

	if (r2dev != NULL) {
		/* 1. Destroy Netlink socket */
		if (r2dev->netlink_sk)
			netlink_kernel_release(r2dev->netlink_sk);
		/* 2. Work item */
		if (r2dev->edgeq_work->data)
			devm_kfree(r2dev->dev, r2dev->edgeq_work->data);

		if (r2dev->edgeq_work) {
			cancel_delayed_work(&r2dev->edgeq_work->cmd_handler);
			devm_kfree(r2dev->dev, r2dev->edgeq_work);
		}

		/* 3. Workqueue */
		if (r2dev->r2_wrkq)
			destroy_workqueue(r2dev->r2_wrkq);

		/* 4. gdata_info */
		if (r2dev->local_gbl->glob)
			dma_free_coherent(r2dev->dev, GLOB_TOTAL_SIZE,
				r2dev->local_gbl->glob,
					r2dev->local_gbl->glob_paddr);

		if (r2dev->local_gbl->gdata_info)
			devm_kfree(r2dev->dev, r2dev->local_gbl->gdata_info);

		if (r2dev->remote_gbl->gdata_info)
			devm_kfree(r2dev->dev, r2dev->remote_gbl->gdata_info);

		/* 5. local_gbl */
		if (r2dev->local_gbl)
			devm_kfree(r2dev->dev, r2dev->local_gbl);

		/* 6. remote_glb */
		if (r2dev->remote_gbl)
			devm_kfree(r2dev->dev, r2dev->remote_gbl);

		/* 7. device structure */
		for (i = 0; i < MAX_DEVICE; i++) {
			if (r2dev->raptor2_ldev[i]) {
				if (r2dev->raptor2_ldev[i]->dev_info)
					devm_kfree(r2dev->dev, r2dev->raptor2_ldev[i]->dev_info);

				devm_kfree(r2dev->dev, r2dev->raptor2_ldev[i]);
			}
		}

		/* 7. r2dev */
		devm_kfree(r2dev->dev, r2dev);
	}

	pci_epc_stop(epc);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar	= &epf->bar[bar];

		if (epf_edgeq->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf_bar);
			if (epf_edgeq->reg[bar])
				iounmap(epf_edgeq->reg[bar]);
		}
	}
}

#if RAPTOR2_EARLY_INIT
static int pci_epf_edgeq_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc		= epf->epc;
	struct device *dev		= &epf->dev;
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;

	epc_features = epf_edgeq->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		if (!epf_bar->size)
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf_bar);
		if (ret) {
			if (epf_edgeq->reg[bar])
				iounmap(epf_edgeq->reg[bar]);
			dev_err(dev, "Failed to set BAR%d\n", bar);
		}
	}

	return 0;
}
#endif

static int pci_epf_init_reg(void *reg_base, void *private)
{
	struct raptor2_plat_pcie *raptor2_plat_pcie = (struct raptor2_plat_pcie *)private;

	if (!reg_base)
		return -EINVAL;

	memset(reg_base, 0, raptor2_plat_pcie->ep_off_size);
	return 0;
}

static int pci_epf_edgeq_core_init(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq	= epf_get_drvdata(epf);
#if RAPTOR2_EARLY_INIT
	struct pci_epf_header *header	= epf->header;
#endif
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc		= epf->epc;
	int ret;
	bool msi_capable = true;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	epf_edgeq->epc_features = epc_features;
	if (epc_features)
		msi_capable = epc_features->msi_capable;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}
	}

#if RAPTOR2_EARLY_INIT
	ret = pci_epc_write_header(epc, epf->func_no, header);
	if (ret) {
		dev_err(&epf->dev, "Configuration header write failed\n");
		return ret;
	}

	ret = pci_epf_edgeq_set_bar(epf);
	if (ret)
		return ret;
#endif
	ret = pci_epf_init_reg(reg_base_addr, epf_edgeq->epc_features->private);
	if (ret)
		return ret;

	return 0;
}

static int pci_epf_edgeq_notifier(struct notifier_block *nb, unsigned long val,
				 void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_edgeq_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		r2dev->linkup_ready = true;
		break;

	default:
		dev_err(&epf->dev, "Invalid EPF edgeq notifier event\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_edgeq_configure_bar(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq	= epf_get_drvdata(epf);
	struct device *dev			= &epf->dev;
	struct pci_epf_bar *epf_bar;
	void *base;
	int bar, add;
	const struct pci_epc_features *epc_features;
	struct raptor2_plat_pcie *raptor2_plat_pcie;
	dma_addr_t bar_phys_addr;
	size_t bar_size;
	int ret				= 0;
	int bar_no;

	epc_features = epf_edgeq->epc_features;
	raptor2_plat_pcie = (struct raptor2_plat_pcie *)epc_features->private;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];

		bar_no = -1;
		switch (bar) {

		case EDGEQ_PCIE_EP_BAR:
			add	 = 1;
			base = ioremap((phys_addr_t)raptor2_plat_pcie->hdma_reg_addr,
						(size_t)raptor2_plat_pcie->hdma_reg_size);
			bar_phys_addr		= raptor2_plat_pcie->hdma_reg_addr;
			epf_bar->flags		|= (PCI_BASE_ADDRESS_MEM_TYPE_64 |
				PCI_BASE_ADDRESS_MEM_PREFETCH);
			bar_size		= raptor2_plat_pcie->hdma_reg_size;
			bar_no			= EDGEQ_PCIE_EP_BAR;
			dma_reg_base_addr	= base;
			break;
		case EDGEQ_DDR_BAR:
			add	= 2;
			base	= ioremap_cache((phys_addr_t)(raptor2_plat_pcie->ddr_addr
					+ raptor2_plat_pcie->ep_off_addr),
						raptor2_plat_pcie->ep_off_size);
			if (!base)
				dev_err(dev, "Failed to allocate space for BAR%d\n",
					bar);
			epf_bar->flags	|= (PCI_BASE_ADDRESS_MEM_TYPE_64 |
					PCI_BASE_ADDRESS_MEM_PREFETCH);
			/* Address and size should be aligned */
			bar_phys_addr	= raptor2_plat_pcie->ddr_addr;
			bar_size	= raptor2_plat_pcie->ddr_size;
			bar_no		= EDGEQ_DDR_BAR;
			reg_base_addr	= base;
			break;
		default:
			break;

		};
		if (bar_no != -1) {
			epf_bar->addr		= epf_edgeq->reg[bar] = base;
			epf_bar->phys_addr	= bar_phys_addr;
			epf_bar->size		= bar_size;
			epf_bar->barno		= bar_no;
			pr_info("Bar %d Addr %llx size %lx flags %x base %llx\n",
				epf_bar->barno, epf_bar->phys_addr,
				epf_bar->size, epf_bar->flags,
				(unsigned long long) base);
		}

	}

	return ret;
}

static int pci_epf_edgeq_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_edgeq *epf_edgeq	= epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc			= epf->epc;
	bool linkup_notifier			= false;
	bool core_init_notifier		= false;
	struct raptor2_plat_pcie *raptor2_plat_pcie;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features			= pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		linkup_notifier	= epc_features->linkup_notifier;
		core_init_notifier	= epc_features->core_init_notifier;
	}

	epf_edgeq->reg_bar		= EDGEQ_DDR_BAR;
	epf_edgeq->epc_features	= epc_features;
	raptor2_plat_pcie		= (struct raptor2_plat_pcie *)epf_edgeq->epc_features;
	ret				= pci_epf_edgeq_configure_bar(epf);
	if (ret)
		return ret;

	if (!core_init_notifier) {
		ret = pci_epf_edgeq_core_init(epf);
		if (ret)
			return ret;
	}

	memset(reg_base_addr, 0, raptor2_plat_pcie->ep_off_size);

	/* Initialize the local and remote context data stucture */
	r2dev->dev = &epf_edgeq->epf->dev;
	ret = raptor2_pci_init(epf_edgeq->epc_features->private);
	if (ret < 0) {
		R2PR_LOG_ERR("%s: Failed raptor2_pci_init: ret=%d\n", __func__, ret);
		return -ENOMEM;
	}

	pr_info("%s: Raptor2 Init Complete\n", __func__);

	if (linkup_notifier) {
		epf->nb.notifier_call = pci_epf_edgeq_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	}

	return 0;
}

static const struct pci_epf_device_id pci_epf_edgeq_ids[] = {
	{
		.name = "pci_epf_edgeq_link3",
	},
	{
		.name = "pci_epf_edgeq_link4",
	},
	{},
};

static int pci_epf_edgeq_probe(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq;
	struct device *dev	= &epf->dev;
	int ret = 0;

	link3_active = false;

	if (strcmp(epf->name, "pci_epf_edgeq_link3") == 0)
		link3_active = true;

	R2_ANALYZE("%s: epf->name %s", __func__, epf->name);
	epf_edgeq		= devm_kzalloc(dev, sizeof(*epf_edgeq), GFP_KERNEL);
	if (!epf_edgeq)
		return -ENOMEM;

	epf->header	= &edgeq_header;
	epf_edgeq->epf	= epf;

	INIT_DELAYED_WORK(&epf_edgeq->cmd_handler, pci_epf_edgeq_cmd_handler);

	ret = init_infra(dev);
	if (!ret) {
		pr_info("FTL infrastructure Ready\n");
	} else {
		R2PR_LOG_ERR("Failed to initialize FTL infrastructure ret=%d\n", ret);
		return -ENOMEM;
	}

	epf_set_drvdata(epf, epf_edgeq);
	edev = epf;

	return 0;
}

static struct pci_epf_ops ops = {
	.unbind	= pci_epf_edgeq_unbind,
	.bind	= pci_epf_edgeq_bind,
};

static struct pci_epf_driver edgeq_driver = {
	.driver.name	= "pci_epf_edgeq",
	.probe		= pci_epf_edgeq_probe,
	.id_table	= pci_epf_edgeq_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_edgeq_init(void)
{
	int ret;

	kpciedgeq_workqueue = alloc_workqueue("kpciedgeq", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpciedgeq_workqueue) {
		R2PR_LOG_ERR("Failed to allocate the kpciedgeq work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&edgeq_driver);
	if (ret) {
		R2PR_LOG_ERR("Failed to register pci epf edgeq driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_edgeq_init);

static void __exit pci_epf_edgeq_exit(void)
{
	pci_epf_unregister_driver(&edgeq_driver);
}
module_exit(pci_epf_edgeq_exit);

MODULE_DESCRIPTION("EDGEQ PCI EPF DRIVER");
MODULE_AUTHOR("Ankit Jindal <ankit.jindal@edgeq.io>");
MODULE_AUTHOR("Pranavkumar Sawargaonkar <pranav.sawargaonkar@edgeq.io>");
MODULE_AUTHOR("Aakash Verma <aakash.verma@edgeq.io>");
MODULE_AUTHOR("Aman Sharma <aman.sharma@edgeq.io>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
