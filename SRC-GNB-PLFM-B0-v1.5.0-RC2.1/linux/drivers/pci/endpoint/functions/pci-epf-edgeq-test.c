// SPDX-License-Identifier: GPL-2.0
/**
 * Test driver to edgeq endpoint functionality
 *
 * Copyright (C) 2021 EdgeQ Inc.
 * Author: Ankit Jindal <ankit.jindal@edgeq.io>
 * Author: Pranavkumar Sawargaonkar <pranav.sawargaonkar@edgeq.io>
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>

#define DRV_VERSION			"0.1"
#define IRQ_TYPE_LEGACY			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

#define COMMAND_RAISE_LEGACY_IRQ	BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ			BIT(3)
#define COMMAND_WRITE			BIT(4)
#define COMMAND_COPY			BIT(5)

#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)

#define FLAG_USE_DMA			BIT(0)

#define TIMER_RESOLUTION		1

/* BAR mappings */
#define EDGEQ_PCIE_EP_BAR		0
#define EDGEQ_PCIE_EP_BAR_ADDR		0x3C980000	/* PCIe L4 EDMA */
#define EDGEQ_PCIE_EP_BAR_SIZE		0x00010000	/* 64KB */

#define EDGEQ_BSS_BAR			1
#define EDGEQ_BSS_BAR_ADDR		0x0
#define EDGEQ_BSS_BAR_SIZE		0x02000000	/* 32MB */

#define EDGEQ_DDR_BAR			2
#define EDGEQ_DDR_BAR_ADDR		0x440000000ULL
#define EDGEQ_DDR_BAR_SIZE		0x80000000ULL	/* 2GB */

#define EDGEQ_DDR_BAR_ADDR_ALIGNED	0x400000000ULL
#define EDGEQ_DDR_BAR_SIZE_ALIGNED	0x200000000ULL	/* 8GB */

static struct workqueue_struct *kpciedgeq_workqueue;

struct pci_epf_edgeq {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		test_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	struct dma_chan		*dma_chan;
	struct completion	transfer_complete;
	bool			dma_supported;
	const struct pci_epc_features *epc_features;
};

struct pci_epf_edgeq_test_reg {
	u32	magic;
	u32	command;
	u32	status;
	u64	src_addr;
	u64	dst_addr;
	u32	size;
	u32	checksum;
	u32	irq_type;
	u32	irq_number;
	u32	flags;
} __packed;

static struct pci_epf_header edgeq_header = {
	.vendorid	= PCI_VENDOR_ID_EDGEQ,
	.deviceid	= PCI_ANY_ID,
	.cache_line_size = 64,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static void pci_epf_edgeq_dma_callback(void *param)
{
	struct pci_epf_edgeq *epf_edgeq = param;

	complete(&epf_edgeq->transfer_complete);
}

/**
 * pci_epf_edgeq_data_transfer() - Function that uses dmaengine API to transfer
 *				  data between PCIe EP and remote PCIe RC
 * @epf_edgeq: the EPF edgeq device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 *
 * Function that uses dmaengine API to transfer data between PCIe EP and remote
 * PCIe RC. The source and destination address can be a physical address given
 * by pci_epc_mem_alloc_addr or the one obtained using DMA mapping APIs.
 *
 * The function returns '0' on success and negative value on failure.
 */
static int pci_epf_edgeq_data_transfer(struct pci_epf_edgeq *epf_edgeq,
				      dma_addr_t dma_dst, dma_addr_t dma_src,
				      size_t len)
{
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct dma_chan *chan = epf_edgeq->dma_chan;
	struct pci_epf *epf = epf_edgeq->epf;
	struct dma_async_tx_descriptor *tx;
	struct device *dev = &epf->dev;
	dma_cookie_t cookie;
	int ret;

	if (IS_ERR_OR_NULL(chan)) {
		dev_err(dev, "Invalid DMA memcpy channel\n");
		return -EINVAL;
	}

	tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len, flags);
	if (!tx) {
		dev_err(dev, "Failed to prepare DMA memcpy\n");
		return -EIO;
	}

	tx->callback = pci_epf_edgeq_dma_callback;
	tx->callback_param = epf_edgeq;
	cookie = tx->tx_submit(tx);
	reinit_completion(&epf_edgeq->transfer_complete);

	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "Failed to do DMA tx_submit %d\n", cookie);
		return -EIO;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_interruptible(&epf_edgeq->transfer_complete);
	if (ret < 0) {
		dmaengine_terminate_sync(chan);
		dev_err(dev, "DMA wait_for_completion_timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * pci_epf_edgeq_init_dma_chan() - Function to initialize EPF edgeq DMA channel
 * @epf_edgeq: the EPF edgeq device that performs data transfer operation
 *
 * Function to initialize EPF edgeq DMA channel.
 */
static int pci_epf_edgeq_init_dma_chan(struct pci_epf_edgeq *epf_edgeq)
{
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf_edgeq->transfer_complete);

	epf_edgeq->dma_chan = dma_chan;

	return 0;
}

/**
 * pci_epf_edgeq_clean_dma_chan() - Function to cleanup EPF edgeq DMA channel
 * @epf_edgeq: the EPF edgeq device that performs data transfer operation
 *
 * Helper to cleanup EPF edgeq DMA channel.
 */
static void pci_epf_edgeq_clean_dma_chan(struct pci_epf_edgeq *epf_edgeq)
{
	if (!epf_edgeq->dma_supported)
		return;

	dma_release_channel(epf_edgeq->dma_chan);
	epf_edgeq->dma_chan = NULL;
}

static void pci_epf_edgeq_print_rate(const char *ops, u64 size,
				    struct timespec64 *start,
				    struct timespec64 *end, bool dma)
{
	struct timespec64 ts;
	u64 rate, ns;

	ts = timespec64_sub(*end, *start);

	/* convert both size (stored in 'rate') and time in terms of 'ns' */
	ns = timespec64_to_ns(&ts);
	rate = size * NSEC_PER_SEC;

	/* Divide both size (stored in 'rate') and ns by a common factor */
	while (ns > UINT_MAX) {
		rate >>= 1;
		ns >>= 1;
	}

	if (!ns)
		return;

	/* calculate the rate */
	do_div(rate, (uint32_t)ns);

	pr_info("\n%s => Size: %llu bytes\t DMA: %s\t Time: %llu.%09u seconds\t"
		"Rate: %llu KB/s\n", ops, size, dma ? "YES" : "NO",
		(u64)ts.tv_sec, (u32)ts.tv_nsec, rate / 1024);
}

static int pci_epf_edgeq_copy(struct pci_epf_edgeq *epf_edgeq)
{
	int ret;
	bool use_dma;
	void __iomem *src_addr;
	void __iomem *dst_addr;
	phys_addr_t src_phys_addr;
	phys_addr_t dst_phys_addr;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_bar = epf_edgeq->test_bar;
	struct pci_epf_edgeq_test_reg *reg = epf_edgeq->reg[test_bar];

	src_addr = pci_epc_mem_alloc_addr(epc, &src_phys_addr, reg->size);
	if (!src_addr) {
		dev_err(dev, "Failed to allocate source address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, src_phys_addr, reg->src_addr,
			       reg->size);
	if (ret) {
		dev_err(dev, "Failed to map source address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		goto err_src_addr;
	}

	dst_addr = pci_epc_mem_alloc_addr(epc, &dst_phys_addr, reg->size);
	if (!dst_addr) {
		dev_err(dev, "Failed to allocate destination address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		ret = -ENOMEM;
		goto err_src_map_addr;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, dst_phys_addr, reg->dst_addr,
			       reg->size);
	if (ret) {
		dev_err(dev, "Failed to map destination address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		goto err_dst_addr;
	}

	ktime_get_ts64(&start);
	use_dma = !!(reg->flags & FLAG_USE_DMA);
	if (use_dma) {
		if (!epf_edgeq->dma_supported) {
			dev_err(dev, "Cannot transfer data using DMA\n");
			ret = -EINVAL;
			goto err_map_addr;
		}

		ret = pci_epf_edgeq_data_transfer(epf_edgeq, dst_phys_addr,
						 src_phys_addr, reg->size);
		if (ret)
			dev_err(dev, "Data transfer failed\n");
	} else {
		memcpy(dst_addr, src_addr, reg->size);
	}
	ktime_get_ts64(&end);
	pci_epf_edgeq_print_rate("COPY", reg->size, &start, &end, use_dma);

err_map_addr:
	pci_epc_unmap_addr(epc, epf->func_no, dst_phys_addr);

err_dst_addr:
	pci_epc_mem_free_addr(epc, dst_phys_addr, dst_addr, reg->size);

err_src_map_addr:
	pci_epc_unmap_addr(epc, epf->func_no, src_phys_addr);

err_src_addr:
	pci_epc_mem_free_addr(epc, src_phys_addr, src_addr, reg->size);

err:
	return ret;
}

static int pci_epf_edgeq_read(struct pci_epf_edgeq *epf_edgeq)
{
	int ret;
	void __iomem *src_addr;
	void *buf;
	u32 crc32;
	bool use_dma;
	phys_addr_t phys_addr;
	phys_addr_t dst_phys_addr;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	struct device *dma_dev = epf->epc->dev.parent;
	enum pci_barno test_bar = epf_edgeq->test_bar;
	struct pci_epf_edgeq_test_reg *reg = epf_edgeq->reg[test_bar];

	src_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->size);
	if (!src_addr) {
		dev_err(dev, "Failed to allocate address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr, reg->src_addr,
			       reg->size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		reg->status = STATUS_SRC_ADDR_INVALID;
		goto err_addr;
	}

	buf = kzalloc(reg->size, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_map_addr;
	}

	use_dma = !!(reg->flags & FLAG_USE_DMA);
	if (use_dma) {
		if (!epf_edgeq->dma_supported) {
			dev_err(dev, "Cannot transfer data using DMA\n");
			ret = -EINVAL;
			goto err_dma_map;
		}

		dst_phys_addr = dma_map_single(dma_dev, buf, reg->size,
					       DMA_FROM_DEVICE);
		if (dma_mapping_error(dma_dev, dst_phys_addr)) {
			dev_err(dev, "Failed to map destination buffer addr\n");
			ret = -ENOMEM;
			goto err_dma_map;
		}

		ktime_get_ts64(&start);
		ret = pci_epf_edgeq_data_transfer(epf_edgeq, dst_phys_addr,
						 phys_addr, reg->size);
		if (ret)
			dev_err(dev, "Data transfer failed\n");
		ktime_get_ts64(&end);

		dma_unmap_single(dma_dev, dst_phys_addr, reg->size,
				 DMA_FROM_DEVICE);
	} else {
		ktime_get_ts64(&start);
		memcpy_fromio(buf, src_addr, reg->size);
		ktime_get_ts64(&end);
	}

	pci_epf_edgeq_print_rate("READ", reg->size, &start, &end, use_dma);

	crc32 = crc32_le(~0, buf, reg->size);
	if (crc32 != reg->checksum)
		ret = -EIO;

err_dma_map:
	kfree(buf);

err_map_addr:
	pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, src_addr, reg->size);

err:
	return ret;
}

static int pci_epf_edgeq_write(struct pci_epf_edgeq *epf_edgeq)
{
	int ret;
	void __iomem *dst_addr;
	void *buf;
	bool use_dma;
	phys_addr_t phys_addr;
	phys_addr_t src_phys_addr;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	struct device *dma_dev = epf->epc->dev.parent;
	enum pci_barno test_bar = epf_edgeq->test_bar;
	struct pci_epf_edgeq_test_reg *reg = epf_edgeq->reg[test_bar];

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, reg->size);
	if (!dst_addr) {
		dev_err(dev, "Failed to allocate address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		ret = -ENOMEM;
		goto err;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, phys_addr, reg->dst_addr,
			       reg->size);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		reg->status = STATUS_DST_ADDR_INVALID;
		goto err_addr;
	}

	buf = kzalloc(reg->size, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err_map_addr;
	}

	get_random_bytes(buf, reg->size);
	reg->checksum = crc32_le(~0, buf, reg->size);

	use_dma = !!(reg->flags & FLAG_USE_DMA);
	if (use_dma) {
		if (!epf_edgeq->dma_supported) {
			dev_err(dev, "Cannot transfer data using DMA\n");
			ret = -EINVAL;
			goto err_map_addr;
		}

		src_phys_addr = dma_map_single(dma_dev, buf, reg->size,
					       DMA_TO_DEVICE);
		if (dma_mapping_error(dma_dev, src_phys_addr)) {
			dev_err(dev, "Failed to map source buffer addr\n");
			ret = -ENOMEM;
			goto err_dma_map;
		}

		ktime_get_ts64(&start);
		ret = pci_epf_edgeq_data_transfer(epf_edgeq, phys_addr,
						 src_phys_addr, reg->size);
		if (ret)
			dev_err(dev, "Data transfer failed\n");
		ktime_get_ts64(&end);

		dma_unmap_single(dma_dev, src_phys_addr, reg->size,
				 DMA_TO_DEVICE);
	} else {
		ktime_get_ts64(&start);
		memcpy_toio(dst_addr, buf, reg->size);
		ktime_get_ts64(&end);
	}

	pci_epf_edgeq_print_rate("WRITE", reg->size, &start, &end, use_dma);

	/*
	 * wait 1ms inorder for the write to complete. Without this delay L3
	 * error in observed in the host system.
	 */
	usleep_range(1000, 2000);

err_dma_map:
	kfree(buf);

err_map_addr:
	pci_epc_unmap_addr(epc, epf->func_no, phys_addr);

err_addr:
	pci_epc_mem_free_addr(epc, phys_addr, dst_addr, reg->size);

err:
	return ret;
}

static void pci_epf_edgeq_raise_irq(struct pci_epf_edgeq *epf_edgeq, u8 irq_type,
				   u16 irq)
{
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_bar = epf_edgeq->test_bar;
	struct pci_epf_edgeq_test_reg *reg = epf_edgeq->reg[test_bar];

	reg->status |= STATUS_IRQ_RAISED;

	switch (irq_type) {
	case IRQ_TYPE_LEGACY:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		break;
	case IRQ_TYPE_MSI:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI, irq);
		break;
	case IRQ_TYPE_MSIX:
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX, irq);
		break;
	default:
		dev_err(dev, "Failed to raise IRQ, unknown type\n");
		break;
	}
}

static void pci_epf_edgeq_cmd_handler(struct work_struct *work)
{
	int ret;
	int count;
	u32 command;
	struct pci_epf_edgeq *epf_edgeq = container_of(work, struct pci_epf_edgeq,
						     cmd_handler.work);
	struct pci_epf *epf = epf_edgeq->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno test_bar = epf_edgeq->test_bar;
	struct pci_epf_edgeq_test_reg *reg = epf_edgeq->reg[test_bar];

	command = reg->command;
	if (!command)
		goto reset_handler;

	reg->command = 0;
	reg->status = 0;

	if (reg->irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_LEGACY_IRQ) {
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_LEGACY, 0);
		goto reset_handler;
	}

	if (command & COMMAND_WRITE) {
		ret = pci_epf_edgeq_write(epf_edgeq);
		if (ret)
			reg->status |= STATUS_WRITE_FAIL;
		else
			reg->status |= STATUS_WRITE_SUCCESS;
		pci_epf_edgeq_raise_irq(epf_edgeq, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_READ) {
		ret = pci_epf_edgeq_read(epf_edgeq);
		if (!ret)
			reg->status |= STATUS_READ_SUCCESS;
		else
			reg->status |= STATUS_READ_FAIL;
		pci_epf_edgeq_raise_irq(epf_edgeq, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_COPY) {
		ret = pci_epf_edgeq_copy(epf_edgeq);
		if (!ret)
			reg->status |= STATUS_COPY_SUCCESS;
		else
			reg->status |= STATUS_COPY_FAIL;
		pci_epf_edgeq_raise_irq(epf_edgeq, reg->irq_type,
				       reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSI_IRQ) {
		count = pci_epc_get_msi(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSI,
				  reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSIX_IRQ) {
		count = pci_epc_get_msix(epc, epf->func_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, PCI_EPC_IRQ_MSIX,
				  reg->irq_number);
		goto reset_handler;
	}

reset_handler:
	queue_delayed_work(kpciedgeq_workqueue, &epf_edgeq->cmd_handler,
			   msecs_to_jiffies(1));
}

static void pci_epf_edgeq_unbind(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	cancel_delayed_work(&epf_edgeq->cmd_handler);
	pci_epf_edgeq_clean_dma_chan(epf_edgeq);
	pci_epc_stop(epc);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (epf_edgeq->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf_bar);
			if (epf_edgeq->reg[bar])
				iounmap(epf_edgeq->reg[bar]);
		}
	}
}

static int pci_epf_edgeq_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
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

		printk("Programming Bar %d Addr %llx size %lx flags %x\n",
					epf_bar->barno, epf_bar->phys_addr,
					epf_bar->size, epf_bar->flags);

		ret = pci_epc_set_bar(epc, epf->func_no, epf_bar);
		if (ret) {
			if (epf_edgeq->reg[bar])
				iounmap(epf_edgeq->reg[bar]);
			dev_err(dev, "Failed to set BAR%d\n", bar);
		}
	}

	return 0;
}

static int pci_epf_edgeq_core_init(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool msix_capable = false;
	bool msi_capable = true;
	int ret;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
	}

	ret = pci_epc_write_header(epc, epf->func_no, header);
	if (ret) {
		dev_err(dev, "Configuration header write failed\n");
		return ret;
	}

	ret = pci_epf_edgeq_set_bar(epf);
	if (ret)
		return ret;

	if (msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->msix_interrupts,
				       epf_edgeq->test_bar,
				       epf_edgeq->msix_table_offset);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	return 0;
}

static int pci_epf_edgeq_notifier(struct notifier_block *nb, unsigned long val,
				 void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_edgeq_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(kpciedgeq_workqueue, &epf_edgeq->cmd_handler,
				   msecs_to_jiffies(1));
		break;

	default:
		dev_err(&epf->dev, "Invalid EPF edgeq notifier event\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_edgeq_configure_bar(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	size_t msix_table_size = 0;
	size_t test_bar_size;
	bool msix_capable;
	void *base;
	int bar, add;
	const struct pci_epc_features *epc_features;
	size_t edgeq_reg_size;
	dma_addr_t bar_phys_addr;
	size_t bar_size;
	int ret = 0;
	int bar_no;

	epc_features = epf_edgeq->epc_features;

	test_bar_size = ALIGN(sizeof(struct pci_epf_edgeq_test_reg), 128);

	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_edgeq->msix_table_offset = test_bar_size;
	}
	edgeq_reg_size = EDGEQ_DDR_BAR_SIZE;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];

		bar_no = -1;
		switch (bar) {

		case EDGEQ_PCIE_EP_BAR:
			add = 1;
			base = ioremap((phys_addr_t)EDGEQ_PCIE_EP_BAR_ADDR,
					(size_t)EDGEQ_PCIE_EP_BAR_SIZE);
			bar_phys_addr = EDGEQ_PCIE_EP_BAR_ADDR;
			bar_size = EDGEQ_PCIE_EP_BAR_SIZE;
			bar_no = EDGEQ_PCIE_EP_BAR;
			break;
		case EDGEQ_BSS_BAR:
			add = 1;
			base = ioremap((phys_addr_t)EDGEQ_BSS_BAR_ADDR,
					(size_t)EDGEQ_BSS_BAR_SIZE);
			bar_phys_addr = EDGEQ_BSS_BAR_ADDR;
			bar_size = EDGEQ_BSS_BAR_SIZE;
			bar_no = EDGEQ_BSS_BAR;
			break;
		case EDGEQ_DDR_BAR:
			add = 2;
			base = ioremap_cache((phys_addr_t)EDGEQ_DDR_BAR_ADDR,
					(size_t)edgeq_reg_size);
			if (!base)
				dev_err(dev, "Failed to allocate space for BAR%d\n",
					bar);
			epf_bar->flags |= (PCI_BASE_ADDRESS_MEM_TYPE_64 |
					PCI_BASE_ADDRESS_MEM_PREFETCH);
			/* Address and size should be aligned */
			bar_phys_addr = EDGEQ_DDR_BAR_ADDR_ALIGNED;
			bar_size = EDGEQ_DDR_BAR_SIZE_ALIGNED;
			bar_no = EDGEQ_DDR_BAR;
			memset(base, 0, 256);
			break;
		default:
			printk ("Wrong BAR Config\n");
			break;

		};
		if (bar_no != -1) {
			epf_bar->addr = epf_edgeq->reg[bar] = base;
			epf_bar->phys_addr = bar_phys_addr;
			epf_bar->size = bar_size;
			epf_bar->barno = bar_no;
			printk("Bar %d Addr %llx size %lx flags %x base %llx\n",
					epf_bar->barno, epf_bar->phys_addr,
					epf_bar->size, epf_bar->flags, (unsigned long long) base);
		}

	}

	return ret;
}

static int pci_epf_edgeq_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_edgeq *epf_edgeq = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no);
	if (epc_features) {
		linkup_notifier = epc_features->linkup_notifier;
		core_init_notifier = epc_features->core_init_notifier;
	}

	epf_edgeq->test_bar = EDGEQ_DDR_BAR;
	epf_edgeq->epc_features = epc_features;

	ret = pci_epf_edgeq_configure_bar(epf);
	if (ret)
		return ret;

	if (!core_init_notifier) {
		ret = pci_epf_edgeq_core_init(epf);
		if (ret)
			return ret;
	}

	/* TODO: start with no dma */
	epf_edgeq->dma_supported = true;

	ret = pci_epf_edgeq_init_dma_chan(epf_edgeq);
	if (ret)
		epf_edgeq->dma_supported = false;

	if (linkup_notifier) {
		epf->nb.notifier_call = pci_epf_edgeq_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(kpciedgeq_workqueue, &epf_edgeq->cmd_handler.work);
	}

	return 0;
}

static const struct pci_epf_device_id pci_epf_edgeq_ids[] = {
	{
		.name = "pci_epf_edgeq",
	},
	{},
};

static int pci_epf_edgeq_probe(struct pci_epf *epf)
{
	struct pci_epf_edgeq *epf_edgeq;
	struct device *dev = &epf->dev;

	epf_edgeq = devm_kzalloc(dev, sizeof(*epf_edgeq), GFP_KERNEL);
	if (!epf_edgeq)
		return -ENOMEM;

	epf->header = &edgeq_header;
	epf_edgeq->epf = epf;

	INIT_DELAYED_WORK(&epf_edgeq->cmd_handler, pci_epf_edgeq_cmd_handler);

	epf_set_drvdata(epf, epf_edgeq);
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

	kpciedgeq_workqueue = alloc_workqueue("kpciedgeq",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpciedgeq_workqueue) {
		pr_err("Failed to allocate the kpciedgeq work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&edgeq_driver);
	if (ret) {
		pr_err("Failed to register pci epf edgeq driver --> %d\n", ret);
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
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
