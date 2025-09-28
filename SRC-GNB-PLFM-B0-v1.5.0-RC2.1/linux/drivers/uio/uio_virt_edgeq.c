// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeQ Virt Driver for FE-BE.
 *
 * Copyright (C) 2023 EdgeQ Inc.
 *
 * Author: Aakash Verma <aakash.verma@edgeq.io>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/of_reserved_mem.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include "uio_virt_edgeq.h"

#define DRV_VERSION	"0.1"

/* Global structures */
struct eq_virt *geqv;

/* Destroy Netlink */
static inline void edgeq_virt_destroy_nlk(void)
{
	if (geqv->netlink_sk) {
		netlink_kernel_release(geqv->netlink_sk);
		dev_info(geqv->dev, "Released Netlink socket\n");
	}
}

/* Destroy UIOs */
static inline void edgeq_virt_destroy_uio(struct eq_dev	*vdev)
{
	/* FE */
	if (vdev->fe_uio != NULL) {
		uio_unregister_device(vdev->fe_uio);
		devm_kfree(geqv->dev, vdev->fe_uio);
	}

	/* BE */
	if (vdev->be_uio != NULL) {
		uio_unregister_device(vdev->be_uio);
		devm_kfree(geqv->dev, vdev->be_uio);
	}
}

/* Free up DMA channels */
static inline void edgeq_virt_free_dma(struct eq_dev *vdev)
{

	if (vdev->cap.cap_flag == DMA_CAP_RD) {
		geqv->dma_rd_inuse_bmap &=
			(~(1<<vdev->cap.dma_info.dma_rd_ch_id));
	}
	if (vdev->cap.cap_flag == DMA_CAP_WR) {
		geqv->dma_wr_inuse_bmap &=
			(~(1<<vdev->cap.dma_info.dma_wr_ch_id));
	}
	if (vdev->cap.cap_flag == DMA_CAP_BOTH) {
		geqv->dma_rd_inuse_bmap &=
			(~(1<<vdev->cap.dma_info.dma_rd_ch_id));
		geqv->dma_wr_inuse_bmap &=
			(~(1<<vdev->cap.dma_info.dma_wr_ch_id));
	}
}

/* Cleanup everything */
static void edgeq_virt_cleanup(struct eq_virt *eqv)
{
	int vrt_itr = 0;
	int res = 0;

	if (eqv->netlink_sk != NULL)
		edgeq_virt_destroy_nlk();

	if (eqv->dma_wa_res != NULL)
		devm_kfree(eqv->dev, eqv->dma_wa_res);

	for (res = 0; res < MAX_XGMAX_RES; res++)
		if (eqv->vdev_res[res] != NULL)
			devm_kfree(eqv->dev, eqv->vdev_res[res]);

	if (eqv->vdev == NULL)
		return;

	for (vrt_itr = 0; vrt_itr < eqv->vdev_count; vrt_itr++) {
		edgeq_virt_destroy_uio(&eqv->vdev[vrt_itr]);
		edgeq_virt_free_dma(&eqv->vdev[vrt_itr]);
	}

	devm_kfree(eqv->dev, eqv->vdev);
}

/* UIO specific code */
static inline void edgeq_virt_fill_uio_map(struct uio_info *info,
		phys_addr_t paddr, phys_addr_t vaddr, size_t size,
		char *name, int map_num, int mem_type)
{
		info->mem[map_num].addr = paddr;
		info->mem[map_num].internal_addr = (void *)vaddr;
		info->mem[map_num].size = size;
		info->mem[map_num].memtype = mem_type;
		info->mem[map_num].name = kasprintf(GFP_KERNEL, "%s", name);

		dev_dbg(geqv->dev, "UIO_MAP%d: paddr=%llx size=%llx name=%s\n",
			map_num, info->mem[map_num].addr, info->mem[map_num].size,
			info->mem[map_num].name);
}

static int edgeq_virt_setup_dma(struct dma_cap *dma_info, int dma_cap_type, int vdev_id)
{
	int dma_ch_id[2] = { INVALID_DMA_CH_ID, INVALID_DMA_CH_ID };
	int map_itr = 0;
	uint32_t dma_bmap[2];
	int ret;

	if (geqv->dma_ch_count == 0) {
		dev_err(geqv->dev, "No DMA channel found\n");
		return -ENODEV;
	}

	dma_bmap[0] = geqv->dma_rd_inuse_bmap;	/* Read channel */
	dma_bmap[1] = geqv->dma_wr_inuse_bmap;	/* Write channel */

	for (map_itr = 0; map_itr < geqv->dma_ch_count; map_itr++) {
		/* Get a read channel */
		if (((dma_bmap[0]&(1<<map_itr)) == 0) &&
					(dma_ch_id[0] == INVALID_DMA_CH_ID))
			dma_ch_id[0] = map_itr;

		/* Get a write channel */
		if (((dma_bmap[1]&(1<<map_itr)) == 0) &&
					(dma_ch_id[1] == INVALID_DMA_CH_ID))
			dma_ch_id[1] = map_itr;
	}

	if (((dma_ch_id[0] > geqv->dma_ch_count) ||
			(dma_ch_id[1] > geqv->dma_ch_count)) &&
			(dma_cap_type == DMA_CAP_BOTH))
		goto dma_chan_alloc_fail;
	else if ((dma_ch_id[0] > geqv->dma_ch_count) &&
		(dma_cap_type == DMA_CAP_RD))
		goto dma_chan_alloc_fail;
	else if ((dma_ch_id[1] > geqv->dma_ch_count) &&
		(dma_cap_type == DMA_CAP_WR))
		goto dma_chan_alloc_fail;
	else {
		dma_info->dma_chan_paddr = geqv->dma_chan->start;
		dma_info->dma_wa_paddr = geqv->dma_wa_res->start +
					DMA_WAOFF(vdev_id*2);

		if (dma_cap_type == DMA_CAP_RD) {
			dma_info->dma_rd_ch_id = dma_ch_id[0];
			dma_info->dma_wr_ch_id = INVALID_DMA_CH_ID;
			geqv->dma_rd_inuse_bmap |= (1<<dma_ch_id[0]);
			ret = 1;
		} else if (dma_cap_type == DMA_CAP_WR) {
			dma_info->dma_rd_ch_id = INVALID_DMA_CH_ID;
			dma_info->dma_wr_ch_id = dma_ch_id[1];
			geqv->dma_wr_inuse_bmap |= (1<<dma_ch_id[1]);
			ret = 1;
		} else {	/* DMA_CAP_BOTH */
			dma_info->dma_rd_ch_id = dma_ch_id[0];
			dma_info->dma_wr_ch_id = dma_ch_id[1];
			geqv->dma_rd_inuse_bmap |= (1<<dma_ch_id[0]);
			geqv->dma_wr_inuse_bmap |= (1<<dma_ch_id[1]);
			ret = 2;
		}
	}

	return ret;

dma_chan_alloc_fail:
	dev_err(geqv->dev, "Failed to allocate DMA channel(s)\n");
	return -ENODEV;
}

static int edgeq_virt_init_virtdev(uint32_t vdev_id, uint32_t vdev_cap,
					struct nlk_response *out)
{
	struct eq_dev *vdev;
	int ret, cap_ret;
	int chan_ret;

	if ((vdev_id >= MAX_VIRT_FUNC) || (vdev_id >= geqv->vdev_count)) {
		dev_err(geqv->dev, "Invalid virt device id\n");
		return -ENODEV;
	}

	if (vdev_cap >= INVALID_CAP) {
		dev_err(geqv->dev, "Invalid virt dev capability requested\n");
		return -ENODEV;
	}

	vdev = &geqv->vdev[vdev_id];

	if (vdev->in_use != 0x0) {
		if (out != NULL) {
			out->ul_dma_chan_id = vdev->cap.dma_info.dma_rd_ch_id;
			out->dl_dma_chan_id = vdev->cap.dma_info.dma_wr_ch_id;
		}

		if (vdev->cap.cap_flag == vdev_cap) {
			dev_info(geqv->dev, "VIRT%d already configured\n", vdev_id);
			cap_ret = VDEV_REQ_NOACTION;
			goto early_vdevinit_done;
		} else {
			dev_err(geqv->dev, "VIRT%d BE-FE in use\n", vdev_id);
			cap_ret = VDEV_REQ_INUSE;
			goto early_vdevinit_done;
		}
	}

	vdev->be_uio = devm_kzalloc(geqv->dev, sizeof(struct uio_info),
								GFP_KERNEL);
	if (!vdev->be_uio)
		return -ENOMEM;

	vdev->fe_uio = devm_kzalloc(geqv->dev, sizeof(struct uio_info),
								GFP_KERNEL);
	if (!vdev->fe_uio) {
		devm_kfree(geqv->dev, vdev->be_uio);
		return -ENOMEM;
	}

	if (vdev_cap != MEMCPY_CAP) {
		chan_ret = edgeq_virt_setup_dma(&vdev->cap.dma_info, vdev_cap, vdev_id);
		if (chan_ret < 0) {
			dev_err(geqv->dev, "No DMA for %s setup memcpy\n",
				vdev->device_name);
			cap_ret = VDEV_REQ_UNCAP_DONE;
			vdev->cap.cap_flag = MEMCPY_CAP;
		} else {
			cap_ret = VDEV_REQ_DONE;
			vdev->cap.cap_flag = vdev_cap;
			if (out != NULL) {
				out->ul_dma_chan_id = vdev->cap.dma_info.dma_rd_ch_id;
				out->dl_dma_chan_id = vdev->cap.dma_info.dma_wr_ch_id;
			}
		}
	} else {
		cap_ret = VDEV_REQ_DONE;
		vdev->cap.cap_flag = MEMCPY_CAP;
		vdev->cap.dma_info.dma_rd_ch_id = INVALID_DMA_CH_ID;
		vdev->cap.dma_info.dma_wr_ch_id = INVALID_DMA_CH_ID;
	}

	/* BE device node */
	vdev->be_uio->name = kasprintf(GFP_KERNEL, "XGMAC_BE_VIRT%d", vdev_id);
	vdev->be_uio->version = DRV_VERSION;
	edgeq_virt_fill_uio_map(vdev->be_uio, vdev->xgmac.reg[0].paddr,
				0x0, vdev->xgmac.reg[0].size,
				"XGMAC_REG", MAP0, UIO_MEM_IOVA);
	edgeq_virt_fill_uio_map(vdev->be_uio, vdev->xgmac.reg[1].paddr,
				0x0, vdev->xgmac.reg[1].size,
				"XGMAC_RX", MAP1, UIO_MEM_IOVA);
	edgeq_virt_fill_uio_map(vdev->be_uio, vdev->xgmac.reg[2].paddr,
				0x0, vdev->xgmac.reg[2].size,
				"XGMAC_TX", MAP2, UIO_MEM_IOVA);

	if (vdev->cap.cap_flag != MEMCPY_CAP) {
		edgeq_virt_fill_uio_map(vdev->be_uio,
			vdev->cap.dma_info.dma_wa_paddr, 0x0,
			(DMA_WA_SIZE*chan_ret), "DMA_WA", MAP3,
			UIO_MEM_IOVA);
		edgeq_virt_fill_uio_map(vdev->be_uio,
				vdev->cap.dma_info.dma_chan_paddr, 0x0,
				resource_size(geqv->dma_chan),
				"DMA_REG", MAP4, UIO_MEM_PHYS);
	}

	ret = uio_register_device(geqv->dev, vdev->be_uio);
	if (ret < 0) {
		dev_err(geqv->dev, "Failed UIO create for %s. ret=%d\n",
			vdev->device_name, ret);
		devm_kfree(geqv->dev, vdev->be_uio);
		devm_kfree(geqv->dev, vdev->fe_uio);
		edgeq_virt_free_dma(vdev);
		return -EIO;
	}

	/* FE device node */
	vdev->fe_uio->name = kasprintf(GFP_KERNEL, "XGMAC_VIRT%d", vdev_id);
	vdev->fe_uio->version = DRV_VERSION;
	edgeq_virt_fill_uio_map(vdev->fe_uio, vdev->xgmac.reg[0].paddr,
				0x0, vdev->xgmac.reg[0].size,
				"XGMAC_REG", MAP0, UIO_MEM_IOVA);
	edgeq_virt_fill_uio_map(vdev->fe_uio, vdev->xgmac.reg[1].paddr,
				0x0, vdev->xgmac.reg[1].size,
				"XGMAC_RX", MAP1, UIO_MEM_IOVA);
	edgeq_virt_fill_uio_map(vdev->fe_uio, vdev->xgmac.reg[2].paddr,
				0x0, vdev->xgmac.reg[2].size,
				"XGMAC_TX", MAP2, UIO_MEM_IOVA);

	ret = uio_register_device(geqv->dev, vdev->fe_uio);
	if (ret < 0) {
		dev_err(geqv->dev, "Failed UIO create for %s. ret=%d\n",
			vdev->device_name, ret);
		devm_kfree(geqv->dev, vdev->fe_uio);
		edgeq_virt_destroy_uio(vdev);
		edgeq_virt_free_dma(vdev);
		return -EIO;
	}
	vdev->in_use = 0x1;
	dev_info(geqv->dev, "%s FE-BE configured with %s\n", vdev->device_name,
		((vdev->cap.cap_flag == MEMCPY_CAP)?"MEMCPY":"DMA"));

early_vdevinit_done:
	return cap_ret;
}

static int edgeq_virt_deinit_virtdev(uint32_t vdev_id)
{
	struct eq_dev *vdev;

	if ((vdev_id >= MAX_VIRT_FUNC) || (vdev_id >= geqv->vdev_count)) {
		dev_err(geqv->dev, "Invalid virt device id\n");
		return -ENODEV;
	}

	vdev = &geqv->vdev[vdev_id];

	if (vdev->in_use != 0x0) {
		edgeq_virt_free_dma(vdev);
		edgeq_virt_destroy_uio(vdev);
		vdev->in_use = 0x0;
	} else {
		dev_info(geqv->dev, "VIRT%d not found\n", vdev_id);
		return VDEV_REQ_NOACTION;
	}

	dev_info(geqv->dev, "VIRT%d Removed\n", vdev_id);
	return VDEV_REQ_DONE;
}

static int edgeq_virt_netlink_send(struct nlmsghdr *nlh, struct nlk_response *out)
{
	int ret = 0;
	int pid = 0;
	struct sk_buff *skb_out = NULL;

	pid = nlh->nlmsg_pid;

	skb_out = nlmsg_new(sizeof(struct nlk_response), GFP_KERNEL);
	if (!skb_out) {
		dev_err(geqv->dev, "Failed to allocate new skb\n");
		devm_kfree(geqv->dev, out);
		return -1;
	}
	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE,
			sizeof(struct nlk_response), 0);
	NETLINK_CB(skb_out).dst_group = 0;
	memset(nlmsg_data(nlh), 0, sizeof(struct nlk_response));
	memcpy(nlmsg_data(nlh), out, sizeof(struct nlk_response));
	ret = nlmsg_unicast(geqv->netlink_sk, skb_out, pid);

	return ret;
}

static void edgeq_virt_netlink_recv(struct sk_buff *skb)
{
	struct nlmsghdr	*nlh;
	struct nlk_request	*udata;
	struct nlk_response	out;
	int			ret = 0;

	nlh = (struct nlmsghdr *) skb->data;
	udata = (struct nlk_request *) nlmsg_data(nlh);
	if (!udata) {
		dev_err(geqv->dev, "Empty netlink message\n");
		return;
	}

	out.status_code = out.ul_dma_chan_id = out.dl_dma_chan_id = INVALID_DMA_CH_ID;

	dev_info(geqv->dev, "Virt request type = %lld\n", udata->req_type);
	switch (udata->req_type) {
	case REQ_INIT_VDEV:
		ret = edgeq_virt_init_virtdev(udata->vdev_id, udata->vdev_cap,
						&out);
		break;
	case REQ_DEINIT_VDEV:
		ret = edgeq_virt_deinit_virtdev(udata->vdev_id);
		break;
	default:
		dev_err(geqv->dev, "Unknown Virt request type\n");
		break;
	}

	if (ret < 0) {
		out.status_code = VDEV_REQ_FAIL;
		out.ul_dma_chan_id = INVALID_DMA_CH_ID;
		out.dl_dma_chan_id = INVALID_DMA_CH_ID;
	} else
		out.status_code = ret;

	if (edgeq_virt_netlink_send(nlh, &out) < 0)
		dev_err(geqv->dev, "Failed to respond to the request\n");
}

struct netlink_kernel_cfg virt_netlink_cfg = {
	.input = edgeq_virt_netlink_recv,
};

static int edgeq_virt_init_netlink(struct platform_device *pdev)
{
	struct eq_virt *eqv = platform_get_drvdata(pdev);

	/* Create netlink socket */
	eqv->netlink_sk = netlink_kernel_create(&init_net, NLK_EDGEQ_VIRT,
						&virt_netlink_cfg);
	if (eqv->netlink_sk == NULL) {
		dev_info(&pdev->dev, "Failed to create netlink socket\n");
		return -ENOMEM;
	}

	return 0;
}

static void edgeq_virt_print_capability(struct platform_device *pdev,
							struct eq_virt *eqv)
{
	int i = 0;

	dev_info(&pdev->dev, "CONFIG: DMA CHs:%d VDEVs:%d\n",
			eqv->dma_ch_count, eqv->vdev_count);

	dev_info(&pdev->dev, "use_netlink property is %s\n",
		(eqv->use_nlk?"set":"not set"));

	if (eqv->dma_chan != NULL)
		dev_info(&pdev->dev, "DMA CH resource: addr:%llx sz:%llx\n",
			eqv->dma_chan->start, resource_size(eqv->dma_chan));
	else
		dev_info(&pdev->dev, "DMA resource not found, %s\n",
			(eqv->dma_ch_count > 0?"Conflicting configuration":""));

	if (eqv->dma_wa_res != NULL)
		dev_info(&pdev->dev, "DMA WA resource: addr:%llx sz:%llx\n",
			eqv->dma_wa_res->start, resource_size(eqv->dma_wa_res));
	else
		dev_info(&pdev->dev, "DMA WA resource not found, %s\n",
			(eqv->dma_ch_count > 0?"Conflicting configuration":""));

	dev_info(&pdev->dev, "Virtual device resources:\n");
	for (i = 0; i < eqv->vdev_count; i++) {
		dev_info(&pdev->dev, "VIRT%d: REG:addr:%llx sz:%llx",
			i, eqv->vdev[i].xgmac.reg[0].paddr,
			eqv->vdev->xgmac.reg[0].size);

		dev_info(&pdev->dev, "VIRT%d: RX :addr:%llx sz:%llx",
			i, eqv->vdev[i].xgmac.reg[1].paddr,
			eqv->vdev->xgmac.reg[1].size);

		dev_info(&pdev->dev, "VIRT%d: TX :addr:%llx sz:%llx",
			i, eqv->vdev[i].xgmac.reg[2].paddr,
			eqv->vdev->xgmac.reg[2].size);
	}
}

static int edgeq_virt_verify_config(struct platform_device *pdev,
					struct eq_virt *eqv)
{
	uint32_t vdev_cnt = 0;
	uint32_t sup_vdevs = 0;
	int vitr = 0;

	sup_vdevs = (resource_size(eqv->vdev_res[0])/eqv->mem_sz.reg_size);
	vdev_cnt = (resource_size(eqv->vdev_res[1])/eqv->mem_sz.rx_size);
	if (sup_vdevs > vdev_cnt)
		sup_vdevs = vdev_cnt;
	vdev_cnt = (resource_size(eqv->vdev_res[2])/eqv->mem_sz.tx_size);
	if (sup_vdevs > vdev_cnt)
		sup_vdevs = vdev_cnt;
	if (sup_vdevs == 0) {
		dev_err(&pdev->dev, "Supported vdevs %d\n", sup_vdevs);
		return -ENOMEM;
	} else if (sup_vdevs < eqv->vdev_count) {
		dev_err(&pdev->dev, "Not enough memory for %d vdevs\n",
							eqv->vdev_count);
		dev_err(&pdev->dev, "Can only support %d, reconfiguring\n",
							sup_vdevs);
		eqv->vdev_count = sup_vdevs;
	} else {
	}

	eqv->vdev = devm_kzalloc(&pdev->dev, sizeof(struct eq_dev)*sup_vdevs,
								GFP_KERNEL);
	if (!eqv->vdev)
		return -ENOMEM;

	for (vitr = 0; vitr < sup_vdevs; vitr++) {
		eqv->vdev[vitr].device_id = vitr;
		eqv->vdev[vitr].in_use = 0x0;
		eqv->vdev[vitr].device_name = kasprintf(GFP_KERNEL,
							"VIRT_%d", vitr);

		eqv->vdev[vitr].xgmac.reg[0].paddr =
			(eqv->mem_sz.reg_size*vitr) + eqv->vdev_res[0]->start;
		eqv->vdev[vitr].xgmac.reg[0].size = eqv->mem_sz.reg_size;

		eqv->vdev[vitr].xgmac.reg[1].paddr =
			(eqv->mem_sz.rx_size*vitr) + eqv->vdev_res[1]->start;
		eqv->vdev[vitr].xgmac.reg[1].size = eqv->mem_sz.rx_size;

		eqv->vdev[vitr].xgmac.reg[2].paddr =
			(eqv->mem_sz.tx_size*vitr) + eqv->vdev_res[2]->start;
		eqv->vdev[vitr].xgmac.reg[2].size = eqv->mem_sz.tx_size;
	}

	return 0;
}

static int edgeq_virt_parse_dt(struct platform_device *pdev)
{
	struct eq_virt *eqv = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *memph = NULL;
	struct resource *mem_res = NULL;
	int num_mem_hndl = 0;
	int num = 0;
	int ret = 0;
	int num_regs = 0;
	int ritr = 0;
	const char *mem_name = NULL;
	uint8_t virt_flag = 0;

	/* Get the structure */
	eqv = platform_get_drvdata(pdev);

	if (of_node == NULL) {
		dev_err(&pdev->dev, "device of_node is NULL\n");
		return -ENODEV;
	}

	/* Get the pcie dma register space */
	eqv->dma_chan = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, "pcie_dma");
	if (eqv->dma_chan == NULL)
		dev_warn(&pdev->dev, "No \"pcie_dma\" register entry found\n");

	/* Get DMA channel parameters */
	ret = of_property_read_u32(of_node, "dma-channels",
						&eqv->dma_ch_count);
	if (ret)
		dev_warn(&pdev->dev, "Failed to find dma-channels. ret:%d\n",
									ret);

	ret = of_property_read_u32(of_node, "virt_reg_size",
						&eqv->mem_sz.reg_size);
	if (ret) {
		dev_err(&pdev->dev, "Failed to find virt_reg_size. ret:%d\n",
									ret);
		return -EINVAL;
	}
	ret = of_property_read_u32(of_node, "virt_rx_size",
						&eqv->mem_sz.rx_size);
	if (ret) {
		dev_err(&pdev->dev, "Failed to find virt_rx_size. ret:%d\n",
									ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(of_node, "virt_tx_size",
						&eqv->mem_sz.tx_size);
	if (ret) {
		dev_err(&pdev->dev, "Failed to find virt_tx_size. ret:%d\n",
									ret);
		return -EINVAL;
	}

	/* Get vdevs parameter */
	ret = of_property_read_u32(of_node, "vdevs", &eqv->vdev_count);
	if (ret) {
		dev_err(&pdev->dev, "Invalid vdevs parameter. ret:%d\n", ret);
		return -EINVAL;
	}

	/* Get netlink parameter */
	eqv->use_nlk = of_property_read_bool(of_node, "use_netlink");
	if (!eqv->use_nlk)
		dev_warn(&pdev->dev, "\"use_netlink\" not set. ret:%d\n", ret);

	/* Get the reserved memory region handles */
	num_mem_hndl = of_count_phandle_with_args(of_node, "memory-region",
									NULL);
	if (num_mem_hndl != 2) {
		dev_err(&pdev->dev, "Missing reserved entries\n");
		return -EINVAL;
	}

	for (num = 0; num < num_mem_hndl; num++) {
		memph = of_parse_phandle(of_node, "memory-region", num);
		if (!memph)
			dev_err(&pdev->dev,
				"No memory node found for the device\n");

		ret = of_property_read_string_index(of_node, "mem-region-name",
						num, &mem_name);

		if (!strcmp(mem_name, PCIE_WA_NAME)) {
			num_regs = MAX_PCIE_RES;
			virt_flag = VDEV_PCIE_WA;
		} else if (!strcmp(mem_name, XGMAC_VIRT_NAME)) {
			num_regs = MAX_XGMAX_RES;
			virt_flag = VDEV_XGMAC;
		} else {
			dev_err(&pdev->dev, "Undesired node %s\n", mem_name);
			continue;
		}

		for (ritr = 0; ritr < num_regs; ritr++) {
			mem_res = devm_kzalloc(dev,
					sizeof(struct resource), GFP_KERNEL);
			if (!mem_res)
				break;

			ret = of_address_to_resource(memph, ritr, mem_res);
			if (ret == 0) {
				if (virt_flag == VDEV_PCIE_WA)
					eqv->dma_wa_res = mem_res;
				else
					eqv->vdev_res[ritr] = mem_res;
			} else {
				devm_kfree(&pdev->dev, mem_res);
				if (virt_flag == VDEV_XGMAC)
					return -ENOMEM;
				if (eqv->dma_ch_count <= 0)
					break;
			}
		}
		of_node_put(memph);
	}

	return 0;
}

static int edgeq_virt_probe(struct platform_device *pdev)
{
	struct eq_virt *eqv = NULL;
	int ret = 0;
	int vdev_itr = 0;

	/* Allocate and init the structures */
	eqv = devm_kzalloc(&pdev->dev, sizeof(struct eq_virt), GFP_KERNEL);
	if (!eqv)
		return -ENOMEM;

	geqv = eqv;
	eqv->dev = &pdev->dev;
	platform_set_drvdata(pdev, eqv);

	/* Get the device tree entries */
	ret = edgeq_virt_parse_dt(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to parse device tree\n");
		goto edgeq_virt_fail;
	}

	ret = edgeq_virt_verify_config(pdev, eqv);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't support vdevs with config\n");
		goto edgeq_virt_fail;
	}
	edgeq_virt_print_capability(pdev, eqv);

	/* If netlink flag is set then defer the virt device initialization */
	if (eqv->use_nlk == true) {
		ret = edgeq_virt_init_netlink(pdev);	/* Setup netlink */
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to initialize netlink\n");
			goto edgeq_virt_nlk_fail;
		}
		dev_info(&pdev->dev, "Configured netlink..\n");
	} else
		goto edgeq_virt_nlk_fail;

	dev_info(&pdev->dev, "EdgeQ virt driver probe complete\n");
	return 0;

edgeq_virt_nlk_fail:
	dev_info(&pdev->dev, "Netlink disabled. Exposing all virt devices\n");
	for (vdev_itr = 0; vdev_itr < eqv->vdev_count; vdev_itr++) {
		ret = edgeq_virt_init_virtdev(vdev_itr, MEMCPY_CAP, NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to setup VIRT%d\n",
								vdev_itr);
			goto edgeq_virt_fail;
		}
	}

	return 0;

edgeq_virt_fail:
	edgeq_virt_cleanup(eqv);
	devm_kfree(&pdev->dev, eqv);
	geqv = NULL;
	platform_set_drvdata(pdev, NULL);
	dev_info(&pdev->dev, "Cleaning up. Probe failed\n");

	return ret;
}

int edgeq_virt_uio_init(u32 vdev_id)
{
	int ret;

	ret = edgeq_virt_init_virtdev(vdev_id, MEMCPY_CAP, NULL);
	if (ret == VDEV_REQ_DONE)
		return 0;
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(edgeq_virt_uio_init);

int edgeq_virt_uio_addr(u32 vdev_id, phys_addr_t *base, phys_addr_t *rx,
		   u32 *rx_size, phys_addr_t *tx, u32 *tx_size)
{
	struct eq_dev *vdev;

	if (vdev_id >= geqv->vdev_count) {
		dev_err(geqv->dev, "Invalid virt device id %d\n", vdev_id);
		return -ENODEV;
	}

	vdev = &geqv->vdev[vdev_id];
	*base = vdev->xgmac.reg[0].paddr;
	/* Region 2 is for VPP Tx and is Rx for Linux driver */
	*rx = vdev->xgmac.reg[2].paddr;
	*rx_size = vdev->xgmac.reg[2].size;
	/* Region 1 is for VPP Rx and is Tx for Linux driver */
	*tx = vdev->xgmac.reg[1].paddr;
	*tx_size = vdev->xgmac.reg[1].size;

	return 0;
}
EXPORT_SYMBOL_GPL(edgeq_virt_uio_addr);

static int edgeq_virt_remove(struct platform_device *pdev)
{
	struct eq_virt *eqv = platform_get_drvdata(pdev);

	if (eqv != NULL) {
		edgeq_virt_cleanup(eqv);
		devm_kfree(&pdev->dev, eqv);
		geqv = NULL;
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static const struct of_device_id edgeq_virt_of_match[] = {
	{ .compatible = "edgeq,virt" },
	{ }
};

MODULE_DEVICE_TABLE(of, edgeq_virt_of_match);

static struct platform_driver edgeq_virt_driver = {

	.probe = edgeq_virt_probe,
	.remove = edgeq_virt_remove,
	.driver = {
		.name = "virt-edgeq",
		.of_match_table = of_match_ptr(edgeq_virt_of_match),
	},
};

module_platform_driver(edgeq_virt_driver);

MODULE_DESCRIPTION("EdgeQ Virt Driver for FE-BE");
MODULE_AUTHOR("Aakash Verma <aakash.verma@edgeq.io>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
