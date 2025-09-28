// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for EdgeQ Raptor2 SoC
 *
 * Copyright (C) 2020-2021 EdgeQ, Inc.
 *
 * Authors: Pranavkumar Sawargaonkar <pranav.sawargaonkar@edgeq.io>
 *	    Ankit Jindal <ankit.jindal@edgeq.io>
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include "pcie-designware.h"

//#define CONFIG_RAPTOR2_EP_EARLY

#define PCIE_USB_NOC_RST_N_LO		0x40
#define PCIE_USB_NOC_RST_N_HI		0x44
#define PCIE_USB_NOC_QR_CLK_LO		0x48
#define PCIE_USB_NOC_QR_CLK_HI		0x4C
#define IOSS_TOP_SHUTDOWN_LO		0x90
#define IOSS_TOP_SHUTDOWN_HI		0x94
#define PCIE_PORT_SHUTDOWN_REM		0x20

#define LINK4_E2_SERD_SHUTDOWN		0x120020
#define LINK4_E2_SRAM_BYPASS		0x120024

#define LINK3_E2_SERD_SHUTDOWN		0x60020
#define LINK3_E2_SRAM_BYPASS		0x60024
#define PORT_BRUP			0x40020

#define LTSSM_LINK_LO			0x00020
#define LTSSM_LINK_HI			0x00024
#define	LTSSM_LINK_STATUS		0x002D8

#define	LTSSM_CCIX_LINK_LO		0x00020
#define	LTSSM_CCIX_LINK_HI		0x00024
#define	LTSSM_CCIX_LINK_STATUS		0x001A0


#define SYS_INT_EVENT_STATUS		(0x08)
#define SYS_INT_EVENT_CLEAR		(0x10)
#define SYS_CTRL_REG			(0x40)
#define PCI_INTERRUPT_PIN_L4		(0x3C)
#define INTERRUPT_PIN_MASK		(0xFF)

#define R2_DEBUG			0	/* Set to 1 to enable logs */

#define R2PR_LOG_ERR(...)			pr_err(__VA_ARGS__)
#if R2_DEBUG
#define R2PR_LOG(...)				pr_info(__VA_ARGS__)
#else
#define R2PR_LOG(...)
#endif

struct raptor2_plat_int {
	struct resource			*sif_ctrl_res;
	void __iomem			*sif_ctrl_base;
};
struct raptor2_plat_int r2_plat_int;
struct raptor2_plat_pcie {
	struct dw_pcie                  *pci;
	struct regmap                   *regmap;
	struct resource			*brup_cfg_res;
	void __iomem			*brup_cfg_base;
	struct resource			*lk_reg_res;
	void __iomem			*lk_reg_base;
	enum dw_pcie_device_mode        mode;
	bool				ccix_port;
	uint32_t link_mode;		/* PCIe Link Information */;
	uint32_t dma_ch;		/* PCIe HDMA CH count */
	uint32_t max_ch_mem;		/* Max Channel Size */
	uint32_t num_windows;		/* ATU windows */
	uint64_t dma_ll_addr;		/* LL Region Addr */
	uint32_t dma_ll_size;		/* LL Region Size */
	uint32_t hdma_reg_addr;		/* BAR mapped PCIe HDMA Addr */
	uint32_t hdma_reg_size;		/* BAR mapped PCIe HDMA Size */
	uint64_t ddr_addr;		/* BAR mapped DDR Addr */
	uint64_t ddr_size;		/* BAR mapped DDR Size */
	uint64_t ep_off_addr;		/* REG/Global Address Space */
	uint32_t ep_off_size;		/* REG/Global Size */
};
struct raptor2_plat_pcie r2_plat_pcie;

struct raptor2_plat_pcie_of_data {
	enum dw_pcie_device_mode        mode;
};

#define	to_raptor2_pcie(x)		dev_get_drvdata((x)->dev)

static const struct of_device_id raptor2_plat_pcie_of_match[];
static const struct dw_pcie_host_ops raptor2_plat_pcie_host_ops = {
};


static int raptor2_pcie_establish_link(struct dw_pcie *pci)
{
	struct raptor2_plat_pcie *raptor2_pcie = to_raptor2_pcie(pci);
	void __iomem *cfg_base = NULL;

	cfg_base = raptor2_pcie->lk_reg_base;

	if (pci->early_init) {
		dw_pcie_ep_linkup(&pci->ep);
		return 0;
	}

	if (raptor2_pcie->mode == DW_PCIE_EP_TYPE) {
		if (!raptor2_pcie->ccix_port) {
			writel(0x1, cfg_base + LTSSM_LINK_LO);
			writel(0x0, cfg_base + LTSSM_LINK_HI);
		} else {
			writel(0x1, cfg_base + LTSSM_CCIX_LINK_LO);
			writel(0x0, cfg_base + LTSSM_CCIX_LINK_HI);
		}
	} else {
		/* RC Linkup */
		/* Non CCIX PCIe Link : Link4 */
		if (!raptor2_pcie->ccix_port) {
			writel(0x101, cfg_base + LTSSM_LINK_LO);
			writel(0x0, cfg_base + LTSSM_LINK_HI);
		} else {
			writel(0x101, cfg_base + LTSSM_CCIX_LINK_LO);
			writel(0x0, cfg_base + LTSSM_CCIX_LINK_HI);
		}
	}
	udelay(200);
	dw_pcie_ep_linkup(&pci->ep);

	return 0;
}

static void raptor2_pcie_stop_link(struct dw_pcie *pci)
{
	struct raptor2_plat_pcie *raptor2_pcie = to_raptor2_pcie(pci);
	void __iomem *cfg_base = NULL;
	unsigned int status;

	cfg_base = raptor2_pcie->lk_reg_base;

	if (pci->early_init)
		return;

	/* Non CCIX PCIe Link : Link4 */
	if (!raptor2_pcie->ccix_port) {
		writel(0x0, cfg_base + LTSSM_LINK_LO);
		writel(0x0, cfg_base + LTSSM_LINK_HI);
		udelay(200);

		status = readl(cfg_base + LTSSM_LINK_STATUS);
	} else {
		writel(0x0, cfg_base + LTSSM_CCIX_LINK_LO);
		writel(0x0, cfg_base + LTSSM_CCIX_LINK_HI);
		udelay(200);
	}

}

static u64 raptor2_pcie_cpu_addr(struct dw_pcie *pci, u64 cpu_addr)
{
		return cpu_addr;
}

static const struct dw_pcie_ops raptor2_pcie_ops = {
	.start_link = raptor2_pcie_establish_link,
	.stop_link = raptor2_pcie_stop_link,
	.cpu_addr_fixup = raptor2_pcie_cpu_addr,
};

static void raptor2_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	if (pci->early_init)
		return;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
}

static irqreturn_t r2_pcie_irq_handler(int irq, void *arg)
{
	struct raptor2_plat_pcie *ep = arg;
	struct raptor2_plat_int *r2_int = &r2_plat_int;

	pr_debug("IRQ for r2-pcie-intx Invoked");

	/* If INTx Interrupt*/
	if (readl(r2_int->sif_ctrl_base + SYS_INT_EVENT_STATUS)) {
		/*Clear INTx interrupt*/
		writel(readl(r2_int->sif_ctrl_base + SYS_INT_EVENT_STATUS),
		r2_int->sif_ctrl_base + SYS_INT_EVENT_CLEAR);

		switch ((dw_pcie_readl_dbi(ep->pci, PCI_INTERRUPT_PIN_L4)>>8)
		&(INTERRUPT_PIN_MASK)) {
		case 1:
		pr_debug("INTxA Asserted\n");
			break;
		case 2:
		pr_debug("INTxB Asserted\n");
			break;
		case 3:
		pr_debug("INTxC Asserted\n");
			break;
		case 4:
		pr_debug("INTxD Asserted\n");
			break;
		}
	}
	return IRQ_HANDLED;
}


int r2_pcie_ep_raise_legacy_irq(struct dw_pcie_ep *ep, u8 func_no)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct pcie_port *pp = &pci->pp;

	return r2_pcie_irq_handler(pp->irq, ep);
}

static int raptor2_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return r2_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features raptor2_plat_pcie_epc_features = {
	.linkup_notifier = true,
	.msi_capable = true,
	.msix_capable = true,
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_4 | 1 << BAR_5,
	.bar_fixed_64bit = 1 << BAR_2 | 1 << BAR_0,
	.bar_fixed_size[0] = SZ_512K,
	.bar_fixed_size[2] = SZ_8G,
	.private = &r2_plat_pcie,
};

static const struct pci_epc_features raptor2_plat_pcie_epc_features_2 = {
	.linkup_notifier = true,
	.msi_capable = true,
	.msix_capable = true,
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_4 | 1 << BAR_5,
	.bar_fixed_64bit = 1 << BAR_2 | 1 << BAR_0,
	.bar_fixed_size[0] = SZ_256K,
	.bar_fixed_size[2] = SZ_8G,
	.private = &r2_plat_pcie,
};

static const struct pci_epc_features*
raptor2_plat_pcie_get_features(struct dw_pcie_ep *ep)
{
	if (!r2_plat_pcie.ccix_port)
		return &raptor2_plat_pcie_epc_features;

	return &raptor2_plat_pcie_epc_features_2;
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = raptor2_pcie_ep_init,
	.raise_irq = raptor2_pcie_ep_raise_irq,
	.get_features = raptor2_plat_pcie_get_features,
};

static int raptor2_add_pcie_port_rc(struct raptor2_plat_pcie *raptor2_pcie,
				    struct platform_device *pdev)
{
	struct dw_pcie *pci = raptor2_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;
	struct raptor2_plat_int *raptor2_int_pcie;

	raptor2_int_pcie = &r2_plat_int;

	pp->irq = platform_get_irq(pdev, 0);
	if (pp->irq < 0)
		return pp->irq;

	ret = devm_request_irq(dev, pp->irq, r2_pcie_irq_handler,
				IRQF_SHARED, "r2-pcie-intx", raptor2_pcie);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		return ret;
	}

	pp->num_vectors = MSI_DEF_NUM_VECTORS;
	pp->ops = &raptor2_plat_pcie_host_ops;

	ret = of_property_read_u32(dev->of_node, "num-windows", &raptor2_pcie->num_windows);
	if (ret) {
		dev_err(dev, "Failed to find \"num-windows\" region\n");
		return ret;
	}

	if (pci->early_init == true) {
		pci->num_ob_windows = raptor2_pcie->num_windows;
		pci->num_ib_windows = raptor2_pcie->num_windows;
	} else {
		pci->num_ob_windows = 8;
		pci->num_ib_windows = 8;
	}

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int raptor2_pcie_bring_up_config(struct raptor2_plat_pcie *raptor2_pcie,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *br_up_base = NULL;

	raptor2_pcie->lk_reg_res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "lk_reg");
	if (!raptor2_pcie->lk_reg_res) {
		dev_err(dev, "Failed to find \"lk_reg\" region\n");
		return -ENODEV;
	}

	raptor2_pcie->lk_reg_base = devm_ioremap_resource(dev,
					raptor2_pcie->lk_reg_res);
	if (IS_ERR(raptor2_pcie->lk_reg_base))
		return PTR_ERR(raptor2_pcie->lk_reg_base);
	raptor2_pcie->brup_cfg_res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "br_up");
	if (!raptor2_pcie->brup_cfg_res) {
		dev_err(dev, "Failed to find \"br_up\" region\n");
		return  -ENXIO;
	}

	raptor2_pcie->brup_cfg_base = devm_ioremap(&pdev->dev, raptor2_pcie->brup_cfg_res->start,
				       resource_size(raptor2_pcie->brup_cfg_res));
	if (IS_ERR(raptor2_pcie->brup_cfg_res))
		return PTR_ERR(raptor2_pcie->brup_cfg_res);

	br_up_base = raptor2_pcie->brup_cfg_base;


	if (!raptor2_pcie->ccix_port) {
		writel(0x0, br_up_base + 0x70);
		writel(0x1, br_up_base + 0x98);
		pr_debug("%s: ccix is disabled\n", __func__);
	} else {
		writel(0x10000, br_up_base + 0x70);
		writel(0x1, br_up_base + 0x98);
		writel(0x12, raptor2_pcie->brup_cfg_base + 0xC0);
		pr_debug("%s: non-ccix is disabled\n", __func__);
	}
	devm_iounmap(dev, raptor2_pcie->brup_cfg_base);

	return 0;
}

#ifdef CONFIG_RAPTOR2_EP_EARLY

static struct pci_epf_header edgeq_header = {
	.vendorid       = PCI_VENDOR_ID_EDGEQ,
	.deviceid       = PCI_ANY_ID,
	.cache_line_size = 64,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin  = PCI_INTERRUPT_INTA,
};

#define EDGEQ_BSS_BAR                   0
#define EDGEQ_BSS_BAR_ADDR              0x80000000
#define EDGEQ_BSS_BAR_SIZE              0x20000000      /* 16MB */

#define EDGEQ_TXU_BAR                   1
#define EDGEQ_TXU_BAR_ADDR              0x1C200000
#define EDGEQ_TXU_BAR_SIZE              0x02000000      /* 128MB */

#define EDGEQ_DDR_BAR                   2
#define EDGEQ_DDR_BAR_ADDR              0x440000000ULL
#define EDGEQ_DDR_BAR_SIZE              0x80000000      /* 2GB */

#define EDGEQ_DNOC_BAR                  4
#define EDGEQ_DNOC_BAR_ADDR             0x0
#define EDGEQ_DNOC_BAR_SIZE             0x40000000      /* 1GB */

int raptor2_pcie_configure_ep(struct dw_pcie *pci)
{
	struct dw_pcie_ep *ep;
	struct pci_epc *epc;
	const struct dw_pcie_ep_ops *ops;
	const struct pci_epc_features *epc_features;
	int ret;
	int bar;
	struct pci_epf_bar epf_bar;
	bool resv_bar = false;

	ep = &pci->ep;
	epc = ep->epc;
	ops = ep->ops;

	epc_features = pci_epc_get_features(epc, 0);

	/* EdgeQ vendor and Device id */
	ret = pci_epc_write_header(epc, 0, &edgeq_header);
	if (ret) {
		R2PR_LOG_ERR("%s: %d: Configuration header write failed\n",
				__func__, __LINE__);
		return ret;
	}

	/* Bar Programming */
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {

		memset(&epf_bar, 0, sizeof(struct pci_epf_bar));
		resv_bar = false;

		switch (bar) {
		case EDGEQ_BSS_BAR:
			epf_bar.phys_addr = EDGEQ_BSS_BAR_ADDR;
			epf_bar.size = EDGEQ_BSS_BAR_SIZE;
			epf_bar.barno = EDGEQ_BSS_BAR;
			break;
		case EDGEQ_TXU_BAR:
			epf_bar.phys_addr = EDGEQ_TXU_BAR_ADDR;
			epf_bar.size = EDGEQ_TXU_BAR_SIZE;
			epf_bar.barno = EDGEQ_TXU_BAR;
			break;
		case EDGEQ_DDR_BAR:
			epf_bar.phys_addr = EDGEQ_DDR_BAR_ADDR;
			epf_bar.size = EDGEQ_DDR_BAR_SIZE;
			epf_bar.flags |= (PCI_BASE_ADDRESS_MEM_TYPE_64 |
					PCI_BASE_ADDRESS_MEM_PREFETCH);
			epf_bar.barno = EDGEQ_DDR_BAR;
			break;
		case EDGEQ_DNOC_BAR:
			epf_bar.phys_addr = EDGEQ_DNOC_BAR_ADDR;
			epf_bar.size = EDGEQ_DNOC_BAR_SIZE;
			epf_bar.flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
			epf_bar.barno = EDGEQ_DNOC_BAR;
			resv_bar = true;
			break;
		default:
			resv_bar = true;
			break;

		};

		if (!resv_bar) {
			pr_info("Programming Bar %d Addr %llx size %lx flags %x\n",
					epf_bar.barno, epf_bar.phys_addr,
					epf_bar.size, epf_bar.flags);
			ret = pci_epc_set_bar(epc, 0, &epf_bar);
			if (ret)
				R2PR_LOG_ERR("#### Failed to program bar %d\n", epf_bar.barno);
		}
	}

	/* Link Up */
	ret = pci_epc_start(epc);
	if (ret)
		R2PR_LOG_ERR("#### Failed to do link up ...\n");

	return 0;
}

#endif /* CONFIG_RAPTOR2_EP_EARLY */

static int raptor2_add_pcie_ep(struct raptor2_plat_pcie *raptor2_pcie,
			       struct platform_device *pdev)
{
	int ret;
	struct dw_pcie_ep *ep;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = raptor2_pcie->pci;

	ep = &pci->ep;

	ret = of_property_read_u32(dev->of_node, "dma-ch", &raptor2_pcie->dma_ch);
	if (ret) {
		dev_err(dev, "Failed to find \"dma-ch\" region\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "max-ch-mem", &raptor2_pcie->max_ch_mem);
	if (ret) {
		dev_err(dev, "Failed to find \"max-ch-mem\" region\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "num-windows", &raptor2_pcie->num_windows);
	if (ret) {
		dev_err(dev, "Failed to find \"num-windows\" region\n");
		return ret;
	}

	ret = of_property_read_u64(dev->of_node, "dma-ll-addr", &raptor2_pcie->dma_ll_addr);
	if (ret) {
		dev_err(dev, "Failed to find \"dma-ll-addr\" region\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "dma-ll-size", &raptor2_pcie->dma_ll_size);
	if (ret) {
		dev_err(dev, "Failed to find \"dma-ll-size\" region\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "hdma-reg-addr", &raptor2_pcie->hdma_reg_addr);
	if (ret) {
		dev_err(dev, "Failed to find \"hdma-reg-addr\" region\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "hdma-reg-size", &raptor2_pcie->hdma_reg_size);
	if (ret) {
		dev_err(dev, "Failed to find \"hdma-reg-size\" region\n");
		return ret;
	}

	ret = of_property_read_u64(dev->of_node, "ddr-addr", &raptor2_pcie->ddr_addr);
	if (ret) {
		dev_err(dev, "Failed to find \"ddr-addr\" region\n");
		return ret;
	}

	ret = of_property_read_u64(dev->of_node, "ddr-size", &raptor2_pcie->ddr_size);
	if (ret) {
		dev_err(dev, "Failed to find \"ddr-size\" region\n");
		return ret;
	}
	ret = of_property_read_u64(dev->of_node, "ep-off-addr", &raptor2_pcie->ep_off_addr);
	if (ret) {
		dev_err(dev, "Failed to find \"ep-off-addr\" region\n");
		return ret;
	}
	ret = of_property_read_u32(dev->of_node, "ep-off-size", &raptor2_pcie->ep_off_size);
	if (ret) {
		dev_err(dev, "Failed to find \"ep-off-size\" region\n");
		return ret;
	}

	memcpy(&r2_plat_pcie, raptor2_pcie, sizeof(struct raptor2_plat_pcie));

	ep->ops = &pcie_ep_ops;

	if (pci->early_init == true) {
		pci->num_ob_windows = raptor2_pcie->num_windows;
		pci->num_ib_windows = raptor2_pcie->num_windows;
	} else {
		pci->num_ob_windows = 8;
		pci->num_ib_windows = 8;
	}
	ep->page_size = SZ_64K;

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "Failed to initialize endpoint\n");
		return ret;
	}

#ifdef CONFIG_RAPTOR2_EP_EARLY

	ret = raptor2_pcie_configure_ep(pci);
	if (ret)
		dev_err(dev, "Failed to configure endpoint\n");

#endif

	return 0;
}

static int raptor2_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct raptor2_plat_pcie *raptor2_plat_pcie;
	struct raptor2_plat_int *r2_int_pcie = &r2_plat_int;
	struct dw_pcie *pci;
	int ret;
	const struct of_device_id *match;
	const struct raptor2_plat_pcie_of_data *data;
	enum dw_pcie_device_mode mode;

	match = of_match_device(raptor2_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = (struct raptor2_plat_pcie_of_data *)match->data;
	mode = (enum dw_pcie_device_mode)data->mode;

	raptor2_plat_pcie = devm_kzalloc(dev, sizeof(*raptor2_plat_pcie),
					 GFP_KERNEL);
	if (!raptor2_plat_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &raptor2_pcie_ops;

	raptor2_plat_pcie->pci = pci;
	raptor2_plat_pcie->mode = mode;

	pci->early_init = false;
	raptor2_plat_pcie->ccix_port = false;

	if (of_get_property(dev->of_node, "ccix_port", NULL)) {
		pr_info("%s: PCIe: Link 3 CCIX PORT\n", __func__);
		raptor2_plat_pcie->ccix_port = true;
	} else {
		pr_info("%s: PCIe: Link 4 Non CCIX Port\n", __func__);
	}

	if (of_get_property(dev->of_node, "raptor2_early_init", NULL)) {
		pr_info("%s: PCIe: Early init enabled\n", __func__);
		pci->early_init = true;
	} else {
		pr_info("%s: PCIe: Early init disabled\n", __func__);

	}
	ret = raptor2_pcie_bring_up_config(raptor2_plat_pcie, pdev);
	if (ret) {
		dev_err(dev, "Failed to initialize bridge configuration\n");
		return ret;
	}


	r2_int_pcie->sif_ctrl_res = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "sif_ctrl");
	if (!r2_int_pcie->sif_ctrl_res) {
		dev_err(dev, "Failed to find \"sif_ctrl\" region\n");
		return -ENODEV;
	}

	r2_int_pcie->sif_ctrl_base = devm_ioremap(&pdev->dev, r2_int_pcie->sif_ctrl_res->start,
				       resource_size(r2_int_pcie->sif_ctrl_res));
	if (IS_ERR(r2_int_pcie->sif_ctrl_res))
		return PTR_ERR(r2_int_pcie->sif_ctrl_res);

	memcpy(&r2_plat_pcie, raptor2_plat_pcie, sizeof(*raptor2_plat_pcie));

	platform_set_drvdata(pdev, raptor2_plat_pcie);

	switch (raptor2_plat_pcie->mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_RAPTOR2_HOST))
			return -ENODEV;
		pr_info("%s: PCIe: RC Port\n", __func__);
		ret = raptor2_add_pcie_port_rc(raptor2_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_RAPTOR2_EP))
			return -ENODEV;
		pr_info("%s: PCIe: EP Port\n", __func__);
		pci->ep.ops = &pcie_ep_ops;
		ret = raptor2_add_pcie_ep(raptor2_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n",
			raptor2_plat_pcie->mode);
	}

	return 0;
}

static const struct raptor2_plat_pcie_of_data raptor2_plat_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct raptor2_plat_pcie_of_data raptor2_plat_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id raptor2_plat_pcie_of_match[] = {
	{
		.compatible = "raptor2-pcie-rc",
		.data = &raptor2_plat_pcie_rc_of_data,
	},
	{
		.compatible = "raptor2-pcie-ep",
		.data = &raptor2_plat_pcie_ep_of_data,
	},
	{},
};

static struct platform_driver raptor2_plat_pcie_driver = {
	.driver = {
		.name	= "raptor2-pcie",
		.of_match_table = raptor2_plat_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = raptor2_plat_pcie_probe,
};
builtin_platform_driver(raptor2_plat_pcie_driver);
