// SPDX-License-Identifier: GPL-2.0
/*
 * UIO driver for EDGEQ Raptor 2 PCIe boards.
 *
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/dma-mapping.h>
#include <linux/pci_ids.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/jiffies.h>

#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/uio_driver.h>
#include <linux/workqueue.h>
#include "uio_pci_edgeq.h"

#define UIO_MEM_PHYS_PFETCH			5

/* Global Variables */
bool device_own = true;
bool ccix_active;
struct raptor2_prv_data *rap2dev;

/* EDGEQ UIO MMAP Support */
static const struct vm_operations_struct uio_physical_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys,
#endif
};

/* UIO specific code */
static void fill_uio_map(struct uio_info *info, phys_addr_t paddr, phys_addr_t vaddr,
			size_t size, char *name, int map_num, int mem_type)
{
		info->mem[map_num].addr = paddr;
		if (vaddr != 0)
			info->mem[map_num].internal_addr = (phys_addr_t *)vaddr;
		info->mem[map_num].size = size;
		info->mem[map_num].memtype = mem_type;
		info->mem[map_num].name = kasprintf(GFP_KERNEL, "%s", name);

		pr_info("UIO_MAP: paddr=%llx vaddr=%llx size=%llx name=%s\n",
			info->mem[map_num].addr, (uint64_t)info->mem[map_num].internal_addr,
			info->mem[map_num].size, info->mem[map_num].name);
}

/****************** Workqueue specific code *********/

/* WORK ITEM 1: POPULATE REMOTE */
static inline void map_ep_to_glob(void)
{
	rap2dev->remote_gbl->glob_paddr =
		(uint64_t)(rap2dev->remote_gbl->ddr_region.paddr+GLOB_START_OFFSET(ccix_active));
	rap2dev->remote_gbl->glob =
		(global_t *)(rap2dev->remote_gbl->ddr_region.vaddr+GLOB_START_OFFSET(ccix_active));
	pr_info("Remote GLOBAL Region = %llx\n", (uint64_t)rap2dev->remote_gbl->glob);
}

/* #2. UIO NODE: REMOTE GLOBAL REG SPACE */
static int expose_remote_glob(struct uio_info *info)
{
	int ret = 0;
	phys_addr_t paddr, vaddr;

	paddr = rap2dev->remote_gbl->ddr_region.paddr+GLOB_START_OFFSET(ccix_active);
	vaddr = rap2dev->remote_gbl->ddr_region.vaddr+GLOB_START_OFFSET(ccix_active);

	info->name = "REMOTE_GLOB";
	info->version = "0.1";

	fill_uio_map(info, paddr, vaddr,
			GLOB_TOTAL_SIZE, "GLOBAL", 0, UIO_MEM_PHYS_PFETCH);

	ret = uio_register_device(rap2dev->dev, info);
	if (ret >= 0) {
		pr_info("Created global UIO node - %s\n", info->name);
		rap2dev->remote_gbl->is_global_mem_mapped = true;
	} else {
		R2PR_LOG_ERR("Failed to create global UIO node - %s\n", info->name);
		return -1;
	}
	return 0;
}

static inline void set_hostpaddr(void)
{
	rap2dev->remote_gbl->glob->host_global_paddr = virt_to_phys(rap2dev->local_gbl->glob);
	rap2dev->remote_gbl->glob->host_global_size = GLOB_TOTAL_SIZE;
	pr_info("Host paddr = %llx\t Host size  = %llx\n",
		rap2dev->remote_gbl->glob->host_global_paddr
		, rap2dev->remote_gbl->glob->host_global_size);
}

static void populate_rqueue_reg(phys_addr_t start_offset)
{
	/* Queue Address Maps */
	rap2dev->remote_gbl->queue_region.paddr =
	 start_offset+rap2dev->remote_gbl->ddr_region.paddr;
	rap2dev->remote_gbl->queue_region.vaddr =
	 start_offset+rap2dev->remote_gbl->ddr_region.vaddr;
	rap2dev->remote_gbl->queue_region.size =
	 rap2dev->remote_gbl->glob->queue_region_size;

	pr_info("Host QM paddr remote pci address 0x%llx",
		rap2dev->remote_gbl->glob->queue_region_addr);
	pr_info("mapped to local ob paddress:virtaddr 0x%llx:0x%llx\n",
		rap2dev->remote_gbl->queue_region.paddr,
		rap2dev->remote_gbl->queue_region.vaddr);
}

static void populate_ll_reg(phys_addr_t start_offset)
{
	/* ll Address Maps */
	rap2dev->remote_gbl->ll_region.paddr =
		start_offset + rap2dev->remote_gbl->ddr_region.paddr;
	rap2dev->remote_gbl->ll_region.vaddr =
		start_offset + rap2dev->remote_gbl->ddr_region.vaddr;
	rap2dev->remote_gbl->ll_region.size =
		rap2dev->local_gbl->glob->dma_region_size;

	pr_info("Host ll region paddr remote pci address 0x%llx",
		rap2dev->local_gbl->glob->dma_region_addr);
	pr_info("mapped to local ob paddress:virtaddr 0x%llx:0x%llx\n",
		rap2dev->remote_gbl->ll_region.paddr,
		rap2dev->remote_gbl->ll_region.vaddr);
}

/* Maps and saves DMA reg space in local raptor2 priv */
static int map_dma_reg_space(void)
{
	int ret = 0;
	struct raptor2_global *lgbl = rap2dev->local_gbl;
	mem_region_t *dma_reg = &(lgbl->dma_reg_region);

	dma_reg->paddr = pci_resource_start(rap2dev->pdev, EDGEQ_DMA_BAR);
	if (!dma_reg->paddr) {
		R2PR_LOG_ERR("Unable to get PCIe Bar 0 paddress\n");
		ret = -1;
		goto err_dma_map;
	}

	dma_reg->vaddr = (uint64_t)pcim_iomap_table(rap2dev->pdev)[EDGEQ_DMA_BAR];
	if (!dma_reg->vaddr) {
		R2PR_LOG_ERR("Unable to get PCIe Bar 0 vaddr\n");
		ret = -2;
		goto err_dma_map;
	}

	dma_reg->size = pci_resource_len(rap2dev->pdev, EDGEQ_DMA_BAR);
	if (!dma_reg->size) {
		R2PR_LOG_ERR("Unable to get PCIe Bar 0 size\n");
		ret = -3;
		goto err_dma_map;
	}

	R2PR_LOG("DMA Reg mapped to local paddress:virtaddr:size 0x%llx",
		dma_reg->paddr);
	R2PR_LOG(" : 0x%llx : 0x%llx\n",
		dma_reg->vaddr, dma_reg->size);

err_dma_map:
	return ret;
}

static void populate_remote(void)
{
	phys_addr_t ep_ddr_paddr = 0;
	phys_addr_t offset = 0;

	/* Wait for userspace to send the info for the GLOBAL structure*/
	if (!rap2dev->local_gbl->is_global_mem_mapped)
		goto reset_handler;     /* Reschedule this work item */

	/* Check if the remote memory regions are already present */
	if (rap2dev->remote_gbl->is_global_mem_mapped &&
		rap2dev->remote_gbl->is_qm_mem_mapped)
		goto completed;         /* Exit */

	/* Initialize the remote global mem region and expose the uio device */
	if (rap2dev->remote_gbl->is_global_mem_mapped == false) {
		/* Wait for the host driver to fill this field in the EP global structure. */
		R2PR_LOG("Trying to map the remote global region using BAR information\n");

		/* Try to map BAR 2 */
		rap2dev->remote_gbl->ddr_region.paddr =
			pci_resource_start(rap2dev->pdev, EDGEQ_DDR_BAR);
		if (!rap2dev->remote_gbl->ddr_region.paddr) {
			R2PR_LOG_ERR("Unable to get PCIe Bar 2 paddress\n");
			goto reset_handler;
		}

		rap2dev->remote_gbl->ddr_region.size =
			pci_resource_len(rap2dev->pdev, EDGEQ_DDR_BAR);
		if (!rap2dev->remote_gbl->ddr_region.size) {
			R2PR_LOG_ERR("Unable to get PCIe Bar 2 size\n");
			goto reset_handler;
		}

		/* Mapping the PCIe BAR 2 as write combine for better performance */
		rap2dev->remote_gbl->ddr_region.vaddr =
			(uint64_t)pci_ioremap_wc_bar(rap2dev->pdev, 2);
		if (!rap2dev->remote_gbl->ddr_region.vaddr) {
			R2PR_LOG_ERR("Unable to get PCIe Bar 2 vaddr\n");
			goto reset_handler;
		}

		pr_info("Global paddr remote ddr address mapped to");
		pr_info("local paddress:virtaddr 0x%llx:0x%llx\n",
			 rap2dev->remote_gbl->ddr_region.paddr,
			 rap2dev->remote_gbl->ddr_region.vaddr);

		/* Try to map BAR 0 */
		if (map_dma_reg_space() < 0) {
			R2PR_LOG_ERR("Unable to map BAR 0 DMA reg space\n");
			goto reset_handler;
		}

		/* Map the EP side global structure */
		map_ep_to_glob();
		rap2dev->local_gbl->glob->status |= STATUS_EP_MAP_IN_PROGRESS;

		rap2dev->remote_gbl->gdata_info =
			devm_kzalloc(rap2dev->dev, sizeof(struct uio_info), GFP_KERNEL);
		if (!rap2dev->remote_gbl->gdata_info) {
			R2PR_LOG_ERR("Failed to expose allocate uio structure for remote glob.\n");
			goto reset_handler;
		}

		if (expose_remote_glob(rap2dev->remote_gbl->gdata_info) < 0) {
			kfree(rap2dev->remote_gbl->gdata_info);
			goto reset_handler;
		}

		/* Populate the host_global_paddr in the EP side Glob structure */
		set_hostpaddr();
	}

	/* Initialize the remote queue mem region */
	if (rap2dev->remote_gbl->is_qm_mem_mapped == false) {
		if (!rap2dev->remote_gbl->glob)
			goto reset_handler;

		if (!rap2dev->remote_gbl->glob->queue_region_addr)
			goto reset_handler;

		if (!rap2dev->remote_gbl->glob->queue_region_size)
			goto reset_handler;

		if (!rap2dev->remote_gbl->glob->ep_bar_ddr_paddr)
			goto reset_handler;

		if (!rap2dev->local_gbl->glob->dma_region_addr)
			goto reset_handler;

		if (!rap2dev->local_gbl->glob->dma_region_size)
			goto reset_handler;

		/* Queue region */
		ep_ddr_paddr =  rap2dev->remote_gbl->glob->ep_bar_ddr_paddr;
		offset = rap2dev->remote_gbl->glob->queue_region_addr - ep_ddr_paddr;
		populate_rqueue_reg(offset);

		/* LL region */
		ep_ddr_paddr =  rap2dev->remote_gbl->glob->ep_bar_ddr_paddr;
		offset = rap2dev->local_gbl->glob->dma_region_addr - ep_ddr_paddr;
		populate_ll_reg(offset);

		rap2dev->remote_gbl->is_qm_mem_mapped = true;
		rap2dev->local_gbl->glob->status |= STATUS_EP_MAP_DONE;

		rap2dev->exposed_remote_glob_region = true;
	}

completed:
	return;

reset_handler:
	rap2dev->edgeq_work->req_type = POLL_FOR_REMOTE;
	rap2dev->edgeq_work->data = 0;
	queue_delayed_work(rap2dev->r2_wrkq, &rap2dev->edgeq_work->cmd_handler,
			msecs_to_jiffies(1000));
}

/* WORK ITEM 2: CHECK FLAG */
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

	nlh = (struct nlmsghdr *)data;
	pid = nlh->nlmsg_pid;

	udata = nlmsg_data(nlh);
	in_wflg = (struct wait_flags *)udata->data;
	dev_id = in_wflg->dev_id;
	out_wflg = kzalloc(sizeof(struct wait_flags), GFP_KERNEL);

	if (in_wflg->linkup_ready)
		out_wflg->linkup_ready = ready_flg = (rap2dev->linkup_ready == true)?true:false;
	else if (in_wflg->is_local_global_mem_mapped)
		out_wflg->is_local_global_mem_mapped = ready_flg
			= (rap2dev->exposed_local_glob_region == true)?true:false;
	else if (in_wflg->is_remote_global_mem_mapped)
		out_wflg->is_remote_global_mem_mapped = ready_flg
			= (rap2dev->exposed_remote_glob_region == true)?true:false;
	else if (in_wflg->is_local_device_ready)
		out_wflg->is_local_device_ready = ready_flg =
			(rap2dev->exposed_local_device[dev_id] == true)?true:false;
	else
		R2PR_LOG("No flag requested\n");

	out_wflg->dev_id = dev_id;

	if (!ready_flg) {
		rap2dev->edgeq_work2->req_type = POLL_FLAG;
		kfree(out_wflg);
		queue_delayed_work(rap2dev->r2_wrkq, &rap2dev->edgeq_work2->cmd_handler,
				msecs_to_jiffies(1000));
		ret = 0;
	} else {
		skb_out = nlmsg_new(sizeof(struct wait_flags), GFP_KERNEL);
		if (!skb_out) {
			R2PR_LOG_ERR("Failed to allocate new skb\n");
			return -1;
		}
		nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, sizeof(struct wait_flags), 0);
		NETLINK_CB(skb_out).dst_group = 0;
		memset(nlmsg_data(nlh), 0, sizeof(struct wait_flags));
		memcpy(nlmsg_data(nlh), out_wflg, sizeof(struct wait_flags));
		ret = nlmsg_unicast(rap2dev->netlink_sk, skb_out, pid);
		kfree(rap2dev->edgeq_work2->data);
	}
	return ret;
}

/* Work Command Handler */
static void ftl_edgeq_cmd_handler(struct work_struct *work)
{
	struct edgeq_ucmd_work *eq_work;
	int ret = 0;

	/* 1. Get the edgeq_ucmd_work and rap2dev from the work */
	eq_work = container_of((struct delayed_work *)work, struct edgeq_ucmd_work,
				cmd_handler);

	/* 2. Do work as per request */
	switch (eq_work->req_type) {
	case NO_OP:
		R2PR_LOG("%s:%d No Operation Request\n", __func__, __LINE__);
		break;
	case POLL_FLAG:
		R2PR_LOG("%s:%d Poll flag Operation Request\n", __func__, __LINE__);
		ret = check_flag(eq_work->data);
		break;
	case POLL_FOR_REMOTE:
		pr_debug("%s:%d Poll for Remote Request\n", __func__, __LINE__);
		populate_remote();
		break;
	default:
		R2PR_LOG_ERR("Unknown request type %d\n", eq_work->req_type);
		break;
	}

	if (ret == -1)
		R2PR_LOG_ERR("Rescheduling work\n");
	else if (ret < -1)
		R2PR_LOG_ERR("Failed to respond to message\n");
	else
		R2PR_LOG("Message sent\n");
}

/******************* Netlink specific code **************************/
struct netlink_kernel_cfg uio_netlink_cfg = {
	.input = edgeq_netlink_recv,
};

/******************* global region **********************************/
static void print_fdesc(struct ftl_descriptor *fdesc)
{
	pr_info("########FTL Descriptor########\n");
	pr_info("num device =%lld\n", fdesc->num_devices);
	pr_info("queue addr 0x%llx\n", fdesc->queue_region_addr);
	pr_info("queue size =%lld\n", fdesc->queue_region_size);
	pr_info("dma_read ch =%lld\n", fdesc->num_dma_read_ch);
	pr_info("dma write ch =%lld\n", fdesc->num_dma_write_ch);
	pr_info("SEQT =%lld\n", fdesc->seqt_enable);
	pr_info("##############################\n");
}

static void fill_glob(struct ftl_descriptor *fdesc, global_t *glb)
{
	glb->num_devices        = fdesc->num_devices;

	glb->queue_region_addr  = fdesc->queue_region_addr;
	glb->queue_region_size  = fdesc->queue_region_size;
	memset(&(glb->queue_map), 0, sizeof(uint64_t));

	glb->num_dma_read_ch = fdesc->num_dma_read_ch;
	glb->num_dma_write_ch = fdesc->num_dma_write_ch;
	memset(&(glb->dma_read_chan_map), 0, sizeof(uint64_t));
}

static void expose_global_uio(struct uio_info *info)
{
	int ret = 0;

	info->name = "LOCAL_GLOB";
	info->version = "0.1";
	fill_uio_map(info, rap2dev->local_gbl->glob_paddr,
			(phys_addr_t)rap2dev->local_gbl->glob,
			GLOB_TOTAL_SIZE, "global", 0, UIO_MEM_PHYS);

	ret = uio_register_device(rap2dev->dev, info);
	if (ret >= 0)
		pr_info("Created local global UIO node - %s\n",
			info->name);
	else
		R2PR_LOG_ERR("Failed to create local global UIO node - %s\n",
			info->name);
}

static void set_global_params(char *data)
{
	struct ftl_descriptor *fdesc = (struct ftl_descriptor *)data;

	print_fdesc(fdesc);

	/* 1.a. Check if linkup is done or not */
	if (rap2dev->linkup_ready == false) {
		R2PR_LOG("Link up not done yet\n");
		return;
	}

	/* 1.b. Check if the global has been initialized before if yes then only initialize the
	 * parameters.
	 */
	if (rap2dev->exposed_local_glob_region == true) {
		R2PR_LOG("Local GLOB already initialized, reinitializing only parameters\n");
		rap2dev->exposed_local_glob_region = false;
		rap2dev->local_gbl->glob->num_queues = 16;
		rap2dev->local_gbl->glob->available_queues = 16;
		memset(&rap2dev->local_gbl->glob->queue_map, 0x0, sizeof(uint64_t));
		rap2dev->local_gbl->glob->num_dma_read_ch = ((ccix_active)?(8):(16));
		rap2dev->local_gbl->glob->available_read_ch = ((ccix_active)?(8):(16));
		memset(&rap2dev->local_gbl->glob->dma_read_chan_map, 0x0, sizeof(uint64_t));
		rap2dev->local_gbl->glob->num_dma_write_ch = ((ccix_active)?(8):(16));
		rap2dev->local_gbl->glob->available_write_ch = ((ccix_active)?(8):(16));
		memset(&rap2dev->local_gbl->glob->dma_write_chan_map, 0x0, sizeof(uint64_t));
		rap2dev->local_gbl->glob->seqt_enable = fdesc->seqt_enable;
		rap2dev->local_gbl->glob->dma_fault = 0;
		rap2dev->exposed_local_glob_region = true;
		return;
	}

	/* 2. Save the queue region information passed */
	rap2dev->local_gbl->queue_region.paddr    = fdesc->queue_region_addr;
	rap2dev->local_gbl->queue_region.size     = fdesc->queue_region_size;

	/* 3. Fill the global structure on the local side */
	fill_glob(fdesc, rap2dev->local_gbl->glob);

	/* 4. Expose the Local global structure through uio
	 * Set global mappped = true - This will unblock the work item.
	 */
	expose_global_uio(rap2dev->local_gbl->gdata_info);
	rap2dev->local_gbl->is_global_mem_mapped = true;
	rap2dev->exposed_local_glob_region = true;

	/* 5. Set status as map in progress */
	rap2dev->local_gbl->glob->status |= STATUS_EP_MAP_IN_PROGRESS;

	rap2dev->local_gbl->glob->num_queues = 16;
	rap2dev->local_gbl->glob->available_queues = 16;
	memset(&rap2dev->local_gbl->glob->queue_map, 0x0, sizeof(uint64_t));

	rap2dev->local_gbl->glob->num_dma_read_ch = ((ccix_active)?(8):(16));
	rap2dev->local_gbl->glob->available_read_ch = ((ccix_active)?(8):(16));
	memset(&rap2dev->local_gbl->glob->dma_read_chan_map, 0x0, sizeof(uint64_t));

	rap2dev->local_gbl->glob->num_dma_write_ch = ((ccix_active)?(8):(16));
	rap2dev->local_gbl->glob->available_write_ch = ((ccix_active)?(8):(16));
	memset(&rap2dev->local_gbl->glob->dma_write_chan_map, 0x0, sizeof(uint64_t));

	R2PR_LOG("SEQT - %lld\n", fdesc->seqt_enable);
	rap2dev->local_gbl->glob->seqt_enable = fdesc->seqt_enable;
	rap2dev->local_gbl->glob->dma_fault = 0;

	/* 6. Schedule the work for the first time. */
	rap2dev->edgeq_work->req_type = POLL_FOR_REMOTE;
	rap2dev->edgeq_work->data = 0;
	queue_delayed_work(rap2dev->r2_wrkq, &rap2dev->edgeq_work->cmd_handler,
			msecs_to_jiffies(1000));
	R2PR_LOG("%s:%d: Completed local initialization\n", __func__, __LINE__);
}

/***************************** Device Region *******************************/
static void print_devdesc(struct device_descriptor *desc)
{
	R2PR_LOG("############ DEVICE DESC ###########\n");
	R2PR_LOG("device id = %lld\n", desc->device_id);
	R2PR_LOG("num_queues = %lld\n", desc->num_queues);
	R2PR_LOG("dma_read_chan = %lld\n", desc->num_dma_read_chan);
	R2PR_LOG("dma_write_chan = %lld\n", desc->num_dma_write_chan);
	R2PR_LOG("dma cpu id = %lld\n", desc->cpu_id);
	R2PR_LOG("####################################\n");
}

static void init_devices(char *data)
{
	struct raptor2_device *dev_r2;
	glob_device_t *lgdev;
	global_t *rglob;
	global_t *lglob;
	struct device_descriptor *dev_desc = (struct device_descriptor *)data;
	phys_addr_t reg_paddr, reg_vaddr, reg_size;
	uint64_t i, req_queues, ret;
	uint64_t req_rd_chan, req_wr_chan;
	int loop = ((ccix_active) ? (8):(16));

	print_devdesc(dev_desc);

	/* Check if LOCAL GLOB REGION is ready */
	if (rap2dev->exposed_local_glob_region) {
		R2PR_LOG("%s:LOCAL GLOB region ready!!!!\n", __func__);
	} else {
		R2PR_LOG("%s:LOCAL GLOB region not ready yet\n", __func__);
		return;
	}

	if (rap2dev->exposed_remote_glob_region) {
		R2PR_LOG("%s:Remote GLOB region ready!!!!\n", __func__);
	} else {
		R2PR_LOG("%s:Remote GLOB region not ready!!!!\n", __func__);
		return;
	}

	/* 1. Check if the device is available if yes then only initialize the
	 * device parameters
	 */
	if ((rap2dev->exposed_local_device[dev_desc->device_id] == true)
			 || (rap2dev->raptor2_ldev[dev_desc->device_id] != NULL)) {
		R2PR_LOG("Device already registered and expoosed. Reconfiguring parameters\n");
		lglob = rap2dev->local_gbl->glob;
		lgdev = (glob_device_t *)((phys_addr_t)rap2dev->local_gbl->glob
				 + GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id));

		/* 0. Unset device ready */
		rap2dev->exposed_local_device[dev_desc->device_id] = false;

		/* 1. Reconfigure Queues */
		memset(&lgdev->dev_qm.queue_map, 0x0, sizeof(uint64_t));
		req_queues = dev_desc->num_queues;
		for (i = 0; i < MAX_QUEUES; i++) {
			if (!((lglob->queue_map >> i)&0x01) && (req_queues > 0)) {
				req_queues--;
				lglob->queue_map |= (1<<i);
				lgdev->dev_qm.queue_map |= (1<<i);
			}
		}
		if ((lglob->available_queues-dev_desc->num_queues) < 0)
			R2PR_LOG_ERR("Error num_queues parameter for device %lld\n",
				 dev_desc->device_id);
		else {
			lglob->available_queues -= dev_desc->num_queues;
			lgdev->dev_qm.num_queues = dev_desc->num_queues;
		}

		/* 2. Reconfigure DMA Channel */
		memset(&lgdev->dev_dma.dma_read_chan_map, 0x0, sizeof(uint64_t));
		req_rd_chan = dev_desc->num_dma_read_chan;
		req_wr_chan = dev_desc->num_dma_write_chan;

		for (i = 0; i < loop; i++) {
			if (!((lglob->dma_read_chan_map >> i)&0x01) && (req_rd_chan > 0)) {
				req_rd_chan--;
				lglob->dma_read_chan_map |= (1<<i);
				lgdev->dev_dma.dma_read_chan_map |= (1<<i);
			}

			if (!((lglob->dma_write_chan_map >> i)&0x01) && (req_wr_chan > 0)) {
				req_wr_chan--;
				lglob->dma_write_chan_map |= (1<<i);
				lgdev->dev_dma.dma_write_chan_map |= (1<<i);
			}
		}
		if (((lglob->available_read_ch - dev_desc->num_dma_read_chan) < 0) ||
			((lglob->available_write_ch - dev_desc->num_dma_write_chan) < 0))
			R2PR_LOG_ERR("Error num of dma channels parameter for device %lld\n",
				 dev_desc->device_id);
		else {
			lglob->available_read_ch -= dev_desc->num_dma_read_chan;
			lglob->available_write_ch -= dev_desc->num_dma_write_chan;
			lgdev->dev_dma.num_read_channel += dev_desc->num_dma_read_chan;
			lgdev->dev_dma.num_write_channel += dev_desc->num_dma_write_chan;
		}

		/* 3. Reconfigure CPU ID */
		lgdev->dev_dma.dma_cpu_id = dev_desc->cpu_id;

		/* 4. Set device ready */
		rap2dev->exposed_local_device[dev_desc->device_id] = true;
		return;
	}

	/* 2. Allocate the device structure */
	rap2dev->raptor2_ldev[dev_desc->device_id] =
		kzalloc(sizeof(struct raptor2_device), GFP_KERNEL);
	dev_r2 = rap2dev->raptor2_ldev[dev_desc->device_id];
	dev_r2->is_claimed = true;

	dev_r2->dev_info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	dev_r2->dev_info->name = kasprintf(GFP_KERNEL, "DEVICE_%lld", dev_desc->device_id);
	dev_r2->dev_info->version = "0.1";
	/*	dev_r2->dev_info->mmap = edgq_uio_mmap;
	 *	Registering custom UIO mmap callback.
	 */

	/* MAP 0: LOCAL DEV REG SPACE */
	reg_paddr = rap2dev->local_gbl->glob_paddr +
		GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id);
	reg_vaddr = (phys_addr_t)rap2dev->local_gbl->glob +
		GLOB_STRUCT_SIZE + (GLOB_DEVICE_SIZE*dev_desc->device_id);
	lgdev = (glob_device_t *)reg_vaddr;
	lgdev->device_id = dev_desc->device_id;

	lglob = rap2dev->local_gbl->glob;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			GLOB_DEVICE_SIZE, "LOCAL_DEV", 0, UIO_MEM_PHYS);
	dev_r2->is_local_dreg_ready = true;

	/* MAP 1: REMOTE DEV REG SPACE */
	reg_paddr = rap2dev->remote_gbl->glob_paddr
		+ (GLOB_STRUCT_SIZE+(GLOB_DEVICE_SIZE*dev_desc->device_id));
	reg_vaddr = (phys_addr_t)rap2dev->remote_gbl->glob
		+ (GLOB_STRUCT_SIZE+(GLOB_DEVICE_SIZE*dev_desc->device_id));
	rglob = rap2dev->remote_gbl->glob;
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
	R2PR_LOG("LOCAL: GLOB QBMP:%llx DEV QBMP:%llx\n",
				lglob->queue_map, lgdev->dev_qm.queue_map);

	reg_paddr = rap2dev->remote_gbl->queue_region.paddr;
	reg_vaddr = rap2dev->remote_gbl->queue_region.vaddr;
	reg_size  = rap2dev->remote_gbl->queue_region.size;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			reg_size, "REMOTE_QUEUE", 2, UIO_MEM_PHYS_PFETCH);
	dev_r2->is_remote_queue_ready = true;

	/* MAP 3: DMA REG SPACE */
	/* Setup dma read and write channel bitmap */
	lgdev->dev_dma.dma_read_chan_map = 0x0;
	req_rd_chan = dev_desc->num_dma_read_chan;
	req_wr_chan = dev_desc->num_dma_write_chan;

	for (i = 0; i < loop; i++) {
		if (!((lglob->dma_read_chan_map >> i)&0x01) && (req_rd_chan > 0)) {
			req_rd_chan--;
			lglob->dma_read_chan_map |= (1<<i);
			lgdev->dev_dma.dma_read_chan_map |= (1<<i);
		}

		if (!((lglob->dma_write_chan_map >> i)&0x01) && (req_wr_chan > 0)) {
			req_wr_chan--;
			lglob->dma_write_chan_map |= (1<<i);
			lgdev->dev_dma.dma_write_chan_map |= (1<<i);
		}
	}

	R2PR_LOG("LOCAL: GLOB DMA_READ_BMAP:%llx DEV GLOB DMA_READ_BMAP:%llx\n",
		lglob->dma_read_chan_map, lgdev->dev_dma.dma_read_chan_map);
	R2PR_LOG("LOCAL: GLOB DMA_WRITE_BMAP:%llx DEV GLOB DMA_WRITE_BMAP:%llx\n",
		lglob->dma_write_chan_map, lgdev->dev_dma.dma_write_chan_map);

	if (((lglob->available_read_ch - dev_desc->num_dma_read_chan) < 0) ||
		((lglob->available_write_ch - dev_desc->num_dma_write_chan) < 0))
		R2PR_LOG_ERR("Error num of dma channels parameter for device %lld\n",
					 dev_desc->device_id);
	else {
		lglob->available_read_ch -= dev_desc->num_dma_read_chan;
		lglob->available_write_ch -= dev_desc->num_dma_write_chan;
		lgdev->dev_dma.num_read_channel += dev_desc->num_dma_read_chan;
		lgdev->dev_dma.num_write_channel += dev_desc->num_dma_write_chan;
	}

	lgdev->dev_dma.dma_cpu_id = dev_desc->cpu_id;

	reg_paddr = rap2dev->local_gbl->dma_reg_region.paddr;
	reg_vaddr = rap2dev->local_gbl->dma_reg_region.vaddr;
	reg_size  = rap2dev->local_gbl->dma_reg_region.size;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			reg_size, "DMA_REG", 3, UIO_MEM_PHYS_PFETCH);

	/* MAP 4: LL region */
	reg_paddr = rap2dev->remote_gbl->ll_region.paddr;
	reg_vaddr = rap2dev->remote_gbl->ll_region.vaddr;
	reg_size  = rap2dev->remote_gbl->ll_region.size;
	fill_uio_map(dev_r2->dev_info, reg_paddr, reg_vaddr,
			reg_size, "DMA_LL", 4, UIO_MEM_PHYS_PFETCH);

	dev_r2->is_dma_reg_ready = true;

	/* UIO register */
	ret = uio_register_device(rap2dev->dev, dev_r2->dev_info);
	if (ret >= 0)
		R2PR_LOG("Created UIO node - %s\n",
			dev_r2->dev_info->name);
	else
		R2PR_LOG_ERR("Failed to create UIO node - %s\n",
			dev_r2->dev_info->name);
	dev_r2->is_device_init_done = true;
	rap2dev->exposed_local_device[dev_desc->device_id] = true;
}

/****************************** Check Flag Function ***************************/
static void check_for_flag(struct sk_buff *sbuff)
{
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *)sbuff->data;
	R2PR_LOG("%s:%d: nlmsg pid=%d msg len=%d\n", __func__, __LINE__,
		 nlh->nlmsg_pid, nlh->nlmsg_len);

	/* Queue work item to check for flags */
	rap2dev->edgeq_work2->req_type = POLL_FLAG;
	rap2dev->edgeq_work2->data = kzalloc(nlh->nlmsg_len, GFP_KERNEL);
	memcpy(rap2dev->edgeq_work2->data, nlh, nlh->nlmsg_len);
	queue_delayed_work(rap2dev->r2_wrkq, &rap2dev->edgeq_work2->cmd_handler,
			msecs_to_jiffies(1000));
}

/****************** Netlink Receive *****************/
static void edgeq_netlink_recv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;
	struct edgeq_user_data *udata;
	int pid;

	nlh = (struct nlmsghdr *)skb->data;
	pid = nlh->nlmsg_pid;

	udata = nlmsg_data(nlh);
	if (!udata)
		return;

	R2PR_LOG("%s: Request type = %d\n", __func__, udata->req_type);
	switch (udata->req_type) {
	case REQ_SET_FTL_DESC:                  /* First request from userspace */
		set_global_params(udata->data);
		break;

	case REQ_INIT_DEV:
		init_devices(udata->data);      /* Second request from userspace */
		break;

	case REQ_WAIT_DESC:                     /* Check flags request */
		check_for_flag(skb);
		break;
	default:
		R2PR_LOG("Unknown Request type");
		break;
	}
}

/*********************** Init Sequence ******************/
static int init_infra(struct device *dev, bool ccix_active)
{
	/* Raptor2 Init code */
	/* 0. Allocate the rap2dev structure */
	rap2dev = devm_kzalloc(dev, sizeof(struct raptor2_prv_data), GFP_KERNEL);

	/* 1. Allocate the work queue */
	rap2dev->r2_wrkq = alloc_workqueue("r2_workq", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!rap2dev->r2_wrkq) {
		R2PR_LOG_ERR("%s: Failed to alloc_workqueue\n", __func__);
		return -1;
	}

	/* 2. Allocate the work item */
	rap2dev->edgeq_work = devm_kzalloc(dev, sizeof(struct edgeq_ucmd_work), GFP_KERNEL);
	if (!rap2dev->edgeq_work) {
		R2PR_LOG_ERR("Unable to allocate structure for edgeq work.\n");
		return -2;
	}
	INIT_DELAYED_WORK(&rap2dev->edgeq_work->cmd_handler, ftl_edgeq_cmd_handler);

	rap2dev->edgeq_work2 = devm_kzalloc(dev, sizeof(struct edgeq_ucmd_work), GFP_KERNEL);
	if (!rap2dev->edgeq_work2) {
		R2PR_LOG_ERR("Unable to allocate structure for edgeq work2.\n");
		return -3;
	}
	INIT_DELAYED_WORK(&rap2dev->edgeq_work2->cmd_handler, ftl_edgeq_cmd_handler);
	pr_debug("%s: CCIX_state: %d NETLINK %d",
		__func__, ccix_active,
			NETLINK_EDGEQ_PCI(ccix_active));
	/* 3. Setup netlink socket */
	rap2dev->netlink_sk =
		netlink_kernel_create(&init_net,
			NETLINK_EDGEQ_PCI(ccix_active), &uio_netlink_cfg);
	if (!rap2dev->netlink_sk) {
		R2PR_LOG_ERR("Failed to create netlink socket.\n");
		return -4;
	}

	return 0;
}

static int raptor2_uio_pci_init(void)
{
	phys_addr_t paddr;
	/* Raptor2 Init code */
	/* 1. Allocate the local and remote glb structure */
	rap2dev->local_gbl = devm_kzalloc(rap2dev->dev, sizeof(struct raptor2_global), GFP_KERNEL);
	if (!rap2dev->local_gbl) {
		R2PR_LOG_ERR("Allocate local gbl structure failed.\n");
		return -5;
	}

	rap2dev->remote_gbl = devm_kzalloc(rap2dev->dev, sizeof(struct raptor2_global), GFP_KERNEL);
	if (!rap2dev->remote_gbl) {
		R2PR_LOG_ERR("Allocate local gbl structure failed.\n");
		return -6;
	}

	/* 2. Allocate the local GLOBAL uio_info structure */
	rap2dev->local_gbl->gdata_info = devm_kzalloc(rap2dev->dev,
		 sizeof(struct uio_info), GFP_KERNEL);
	if (!rap2dev->local_gbl->gdata_info) {
		R2PR_LOG_ERR("Allocate global uio structure.\n");
		return -7;
	}

	/* 3. Remap the local glob region and initialize the global structure */
	rap2dev->local_gbl->glob = dma_alloc_coherent(rap2dev->dev,
		 GLOB_TOTAL_SIZE, &paddr, GFP_KERNEL);
	if (!rap2dev->local_gbl->glob) {
		R2PR_LOG_ERR("Mapping global region failed.\n");
		return -8;
	}
	rap2dev->local_gbl->glob_paddr = paddr;

	memset(rap2dev->local_gbl->glob, 0, GLOB_TOTAL_SIZE);
	rap2dev->local_gbl->glob->status = STATUS_EP_NOT_READY;
	rap2dev->local_gbl->glob->host_global_paddr = paddr;
	rap2dev->local_gbl->glob->host_global_size  = GLOB_TOTAL_SIZE;

	pr_info("Completed Raptor2 Init\n");

	return 0;
}

static int edgeq_pci_dma_set_mask(struct pci_dev *dev)
{
	int ret = -EINVAL;

	ret = dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64));
	if (ret) {
		R2PR_LOG_ERR("Failed to set DMA 64Bit mask\n");
		ret = dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32));
		if (ret)
			R2PR_LOG_ERR("Failed to set DMA 32Bit mask\n");
	}

	return ret;
}

static int edgeq_pci_probe(struct pci_dev *dev,
			const struct pci_device_id *id)
{
	int err;
	int ret = 0;

	if (id->device == PCI_DEVICE_ID_LINK3) {
	/* Enable property specific to LINK3 EP */
		pr_debug("%s: %d: Enabling PCIe: Link3 device parameters\n",
			__func__, __LINE__);
		ccix_active = true;
	} else if (id->device == PCI_DEVICE_ID_LINK4) {
	/* Enable property specific to LINK4 EP */
		pr_debug("%s: %d: Enabling PCIe: Link4 device parameters\n",
			__func__, __LINE__);
		ccix_active = false;
	} else {
		pr_info("%s: %d: Un-supported Device\n", __func__, __LINE__);
		return 0;
	}

	pr_info("%s: %d: Bringing up FTL Infra.\n", __func__, __LINE__);
	ret = init_infra(&dev->dev, ccix_active);
	if (ret < 0) {
		R2PR_LOG_ERR("%s: Failed raptor2_uio_pci_init: ret=%d\n", __func__, ret);
		goto err_state;
	}

	if (!pci_is_enabled(dev)) {
		/* Enable PCI device */
		err = pcim_enable_device(dev);
		if (err) {
			pci_err(dev, "enabling device failed\n");
			return err;
		}

		/* Mapping PCI BAR regions */
		err = pcim_iomap_regions(dev, 0x1, pci_name(dev));
		if (err) {
			pci_err(dev, "BAR I/O remapping failed\n");
			pci_disable_device(dev);
			return err;
		}

		if (edgeq_pci_dma_set_mask(dev)) {
			R2PR_LOG_ERR("Failed to set dma mask\n");
			pci_disable_device(dev);
			return -EINVAL;
		}
		pci_set_master(dev);
		/* Since we have enabled device we own it */
		device_own = false;
	}
	rap2dev->linkup_ready = 1;

	rap2dev->dev = &dev->dev;
	rap2dev->pdev = dev;

	ret = raptor2_uio_pci_init();
	if (ret < 0) {
		R2PR_LOG_ERR("%s: Failed raptor2_uio_pci_init: ret=%d\n", __func__, ret);
		goto err_state;
	}

	return 0;

err_state:
	if (ret <= -8 && rap2dev->local_gbl && rap2dev->local_gbl->gdata_info)
		kfree(rap2dev->local_gbl->gdata_info);
	if (ret <= -7 && rap2dev->remote_gbl)
		kfree(rap2dev->remote_gbl);
	if (ret <= -6 && rap2dev->local_gbl)
		kfree(rap2dev->local_gbl);
	if (ret <= -5 && rap2dev->netlink_sk)
		netlink_kernel_release(rap2dev->netlink_sk);
	if (ret <= -4 && rap2dev->edgeq_work2)
		kfree(rap2dev->edgeq_work2);
	if (ret <= -3 && rap2dev->edgeq_work)
		kfree(rap2dev->edgeq_work);
	if (ret <= -2 && rap2dev->r2_wrkq)
		destroy_workqueue(rap2dev->r2_wrkq);
	if (ret <= -1) {
		if (pci_is_enabled(dev) && (device_own == false)) {
			pcim_iounmap_regions(dev, 0x7);
			pci_clear_master(dev);
			pci_disable_device(dev);
			device_own = true;
		}
		kfree(rap2dev);
		return -ENODEV;
	}

	return 0;
}

/*********************** DEINIT Code Region ************/

static void destroy_uio(struct pci_dev *dev, struct raptor2_global *gbl)
{
	struct uio_info *p = gbl->gdata_info;

		uio_unregister_device(p);
		devm_kfree(&dev->dev, gbl->gdata_info);
		devm_kfree(&dev->dev, gbl);

}

static void destroy_edgeq_work(void)
{
	if (rap2dev->edgeq_work2) {
		cancel_delayed_work(&rap2dev->edgeq_work2->cmd_handler);
		kfree(rap2dev->edgeq_work2);

	}
	if (rap2dev->edgeq_work) {
		cancel_delayed_work(&rap2dev->edgeq_work->cmd_handler);
		kfree(rap2dev->edgeq_work);

	}
	if (rap2dev->r2_wrkq)
		destroy_workqueue(rap2dev->r2_wrkq);

	R2PR_LOG("Cancelled all work items and Freed the work queue\n");
}

static void destroy_local(struct pci_dev *dev, struct raptor2_global *gbl)
{
	if (gbl->glob) {
		memunmap(gbl->glob);
		R2PR_LOG("Unmapped global region\n");
	}
	destroy_uio(dev, gbl);
	R2PR_LOG("Freed Global structure");
}

static void destroy_remote(struct pci_dev *dev, struct raptor2_global *gbl)
{
	destroy_uio(dev, gbl);
	R2PR_LOG("Freed Remote Global structure\n");
}

void edgeq_pci_uio_remove(struct pci_dev *dev)
{
	destroy_edgeq_work();
	dma_free_coherent(rap2dev->dev, GLOB_TOTAL_SIZE,
		rap2dev->local_gbl->glob, rap2dev->local_gbl->glob_paddr);
	destroy_local(dev, rap2dev->local_gbl);
	destroy_remote(dev, rap2dev->remote_gbl);

	if (rap2dev->netlink_sk) {
		netlink_kernel_release(rap2dev->netlink_sk);
		R2PR_LOG("Released Netlink socket\n");
	}

	if (pci_is_enabled(dev) && (device_own == false)) {
		pcim_iounmap_regions(dev, 0x7);
		pci_clear_master(dev);
		pci_disable_device(dev);
		pci_disable_device(dev);
		device_own = true;
	}
	kfree(rap2dev);
	pr_info("%s: Unregistered PCIe devices\n", __func__);
}

static void edgeq_pci_remove(struct pci_dev *pdev)
{
	/* struct uio_info *info = pci_get_drvdata(dev); */
	if (pdev)
		edgeq_pci_uio_remove(pdev);
	pdev = NULL;

	/* TODO */
}

static struct pci_device_id edgeq_pci_ids[] = {
	{
		.vendor =	PCI_VENDOR_ID_EDGEQ,
		.device =	PCI_DEVICE_ID_LINK3,
		.subvendor =	0,
		.subdevice =	0,
	},
	{
		.vendor =	PCI_VENDOR_ID_EDGEQ,
		.device =	PCI_DEVICE_ID_LINK4,
		.subvendor =	0,
		.subdevice =	0,
	},
	{ 0, }
};

static struct pci_driver edgeq_pci_driver = {
	.name = "edgeq-pci",
	.id_table = edgeq_pci_ids,
	.probe = edgeq_pci_probe,
	.remove = edgeq_pci_remove,
};

module_pci_driver(edgeq_pci_driver);
MODULE_DEVICE_TABLE(pci, edgeq_pci_ids);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ankit Jindal <ankit.jindal@edgeq.io>");
MODULE_AUTHOR("Pranavkumar Sawargaonkar <pranav.swargaonkar@edgeq.io>");
MODULE_AUTHOR("Aakash Verma <aakash.verma@edgeq.io>");
MODULE_AUTHOR("Aman Sharma <aman.sharma@edgeq.io>");
