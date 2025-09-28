// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This contains the functions to handle the platform driver.

*******************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_irq.h>

#include "xgbe.h"
#include "xgmac_platform.h"
#include "xgbe_misc.h"
#include "xgbe_ecpri.h"
#ifdef CONFIG_MV88X7120_PHY
#include "xgbe_phy.h"
#endif

#ifdef CONFIG_OF

const char *soc_clk_src_str[] = {"VCTCXO", "DCTCXO", "DPLL", "LOCAL_CLK", "INVALID CLOCK"};

/**
 * xgmac_axi_setup - parse DT parameters for programming the AXI register
 * @pdev: platform device
 * Description:
 * if required, from device-tree the AXI internal register can be tuned
 * by using platform parameters.
 */
static struct xgmac_axi *xgmac_axi_setup(struct platform_device *pdev)
{
	struct device_node *np;
	struct xgmac_axi *axi;

	np = of_parse_phandle(pdev->dev.of_node, "snps,axi-config", 0);
	if (!np)
		return NULL;

	axi = devm_kzalloc(&pdev->dev, sizeof(*axi), GFP_KERNEL);
	if (!axi) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}

	/* lpi is not supported */
	axi->axi_lpi_en = false;
	axi->axi_xit_frm = of_property_read_bool(np, "snps,xit_frm");
	axi->axi_kbbe = of_property_read_bool(np, "snps,axi_kbbe");
	axi->axi_fb = of_property_read_bool(np, "snps,axi_fb");
	axi->axi_mb = of_property_read_bool(np, "snps,axi_mb");
	axi->axi_rb =  of_property_read_bool(np, "snps,axi_rb");

	if (of_property_read_u32(np, "snps,wr_osr_lmt", &axi->axi_wr_osr_lmt))
		axi->axi_wr_osr_lmt = 31;
	if (of_property_read_u32(np, "snps,rd_osr_lmt", &axi->axi_rd_osr_lmt))
		axi->axi_rd_osr_lmt = 31;
	of_property_read_u32_array(np, "snps,blen", axi->axi_blen, AXI_BLEN);
	of_node_put(np);

	return axi;
}

/**
 * xgmac_mtl_setup - parse DT parameters for multiple queues configuration
 * @pdev: platform device
 */
static int xgmac_mtl_setup(struct platform_device *pdev,
			    struct plat_xgmacenet_data *plat)
{
	struct device_node *q_node;
	struct device_node *rx_node;
	struct device_node *tx_node;
	struct device_node *ia_node;
	u8 queue;
	u8 chan;
	int ret = 0;

	/* For backwards-compatibility with device trees that don't have any
	 * snps,mtl-rx-config or snps,mtl-tx-config properties, we fall back
	 * to one RX and TX queues each.
	 */
	plat->rx_queues_to_use = 1;
	plat->tx_queues_to_use = 1;
	plat->rx_chans_to_use = 1;
	plat->tx_chans_to_use = 1;

	/* First Queue must always be in DCB mode. As MTL_QUEUE_DCB = 1 we need
	 * to always set this, otherwise Queue will be classified as AVB
	 * (because MTL_QUEUE_AVB = 0).
	 */
	plat->rx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;
	plat->tx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;

	rx_node = of_parse_phandle(pdev->dev.of_node, "snps,mtl-rx-config", 0);
	if (!rx_node)
		return ret;

	/* Processing RX queues common config */
	if (of_property_read_u32(rx_node, "snps,rx-queues-to-use",
				 &plat->rx_queues_to_use))
		plat->rx_queues_to_use = MTL_MIN_RX_QUEUES;

	for (queue = 0; queue < plat->rx_queues_to_use; queue++) {
		plat->rx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;
		/* If static mapping, map queue number to chan number */
		plat->rx_queues_cfg[queue].chan = queue;
		plat->rx_queues_cfg[queue].pkt_route = 0x0;
		plat->rx_queues_cfg[queue].prio = 0;
		plat->rx_queues_cfg[queue].use_prio = false;
	}

	rx_node = of_parse_phandle(pdev->dev.of_node, "snps,dma-rx-config", 0);
	if (!rx_node)
		return ret;

	/* Processing RX DMA common config */
	if (of_property_read_u32(rx_node, "snps,rx-chans-to-use",
				 &plat->rx_chans_to_use))
		plat->rx_chans_to_use = MAX_DMA_CHANS;
	if (plat->ecpri_en && plat->rx_chans_to_use < MIN_RX_CHANS_WITH_ECPRI)
		plat->rx_chans_to_use = MIN_RX_CHANS_WITH_ECPRI;
	if (plat->rx_chans_to_use > MAX_DMA_CHANS)
		plat->rx_chans_to_use = MAX_DMA_CHANS;

	if (of_property_read_bool(rx_node, "snps,rx-sched-sp"))
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	else if (of_property_read_bool(rx_node, "snps,rx-sched-wsp"))
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_WSP;
	else
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	/* Use per channel interrupt by default */
	plat->per_ch_irq = true;
	/* Use dynamic dma mapping by default */
	plat->dynamic_dma_map = true;

	/* Processing individual RX chan config */
	chan = 0;
	for_each_child_of_node(rx_node, q_node) {
		if (chan >= plat->rx_chans_to_use)
			break;

		plat->rx_chans_cfg[chan].ch_irq = of_irq_get(q_node, 0);
		if (plat->rx_chans_cfg[chan].ch_irq < 0)
			return plat->rx_chans_cfg[chan].ch_irq;

		ia_node = of_parse_phandle(q_node, "interrupt-affinity", 0);
		if (ia_node) {
			int cpu = of_cpu_node_to_id(ia_node);

			if (cpu >= 0)
				plat->rx_chans_cfg[chan].ch_irq_cpu = cpu;
			else
				plat->rx_chans_cfg[chan].ch_irq_cpu = 1;
		} else {
			/* Default to cpu 1 */
			plat->rx_chans_cfg[chan].ch_irq_cpu = 1;
		}

		chan++;
	}
	if (chan != plat->rx_chans_to_use) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Not all RX chans were configured\n");
		goto out;
	}

	tx_node = of_parse_phandle(pdev->dev.of_node, "snps,dma-tx-config", 0);
	if (!tx_node) {
		of_node_put(rx_node);
		return ret;
	}

	/* Processing TX DMA common config */
	if (of_property_read_u32(tx_node, "snps,tx-chans-to-use",
				 &plat->tx_chans_to_use))
		plat->tx_chans_to_use = MAX_DMA_CHANS;
	if (plat->ecpri_en) {
		if (plat->eth_link == 1 &&
		    plat->tx_chans_to_use < MIN_TX_CHANS_WITH_ECPRI_L1)
			plat->tx_chans_to_use = MIN_TX_CHANS_WITH_ECPRI_L1;
		if (plat->eth_link == 2 &&
		    plat->tx_chans_to_use < MIN_TX_CHANS_WITH_ECPRI)
			plat->tx_chans_to_use = MIN_TX_CHANS_WITH_ECPRI;
	}
	if (plat->tx_chans_to_use > MAX_DMA_CHANS)
		plat->tx_chans_to_use = MAX_DMA_CHANS;
	/* For Tx, number of queues and chans are equal */
	plat->tx_queues_to_use = plat->tx_chans_to_use;

	for (queue = 0; queue < plat->tx_queues_to_use; queue++) {
		plat->tx_queues_cfg[queue].weight = 0x10 + queue;
		plat->tx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;
		plat->tx_queues_cfg[queue].prio = 0;
		plat->tx_queues_cfg[queue].use_prio = false;
	}

	if (of_property_read_bool(tx_node, "snps,tx-sched-wrr"))
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	else if (of_property_read_bool(tx_node, "snps,tx-sched-wfq"))
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WFQ;
	else if (of_property_read_bool(tx_node, "snps,tx-sched-dwrr"))
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_DWRR;
	else
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	/* Processing individual TX DMA config */
	chan = 0;
	for_each_child_of_node(tx_node, q_node) {
		if (chan >= plat->tx_chans_to_use)
			break;

		plat->tx_chans_cfg[chan].ch_irq = of_irq_get(q_node, 0);
		if (plat->tx_chans_cfg[chan].ch_irq < 0)
			return plat->tx_chans_cfg[chan].ch_irq;

		ia_node = of_parse_phandle(q_node, "interrupt-affinity", 0);
		if (ia_node) {
			int cpu = of_cpu_node_to_id(ia_node);

			if (cpu >= 0)
				plat->tx_chans_cfg[chan].ch_irq_cpu = cpu;
			else
				plat->tx_chans_cfg[chan].ch_irq_cpu = 1;
		} else {
			/* Default to cpu 1 */
			plat->tx_chans_cfg[chan].ch_irq_cpu = 1;
		}

		chan++;
	}
	if (chan != plat->tx_chans_to_use) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Not all TX chans were configured\n");
		goto out;
	}

out:
	of_node_put(rx_node);
	of_node_put(tx_node);
	of_node_put(q_node);

	return ret;
}

/**
 * xgmac_dt_phy - parse device-tree driver parameters to allocate PHY resources
 * @plat: driver data platform structure
 * @np: device tree node
 * @dev: device pointer
 * Description:
 * The mdio bus will be allocated in case of a phy transceiver is on board;
 * it will be NULL if the fixed-link is configured.
 * If there is the "snps,dwmac-mdio" sub-node the mdio will be allocated
 * in any case (for DSA, mdio must be registered even if fixed-link).
 * The table below sums the supported configurations:
 *	-------------------------------
 *	snps,phy-addr	|     Y
 *	-------------------------------
 *	phy-handle	|     Y
 *	-------------------------------
 *	fixed-link	|     N
 *	-------------------------------
 *	snps,dwmac-mdio	|
 *	  even if	|     Y
 *	fixed-link	|
 *	-------------------------------
 *
 * It returns 0 in case of success otherwise -ENODEV.
 */
static int xgmac_dt_phy(struct plat_xgmacenet_data *plat,
			 struct device_node *np, struct device *dev)
{
	bool mdio = !of_phy_is_fixed_link(np);

	/**
	 * If snps,dwmac-mdio is passed from DT, always register
	 * the MDIO
	 */
	for_each_child_of_node(np, plat->mdio_node) {
		if (of_device_is_compatible(plat->mdio_node,
					    "snps,dwmac-mdio")) {
			break;
		}
	}

	if (plat->mdio_node) {
		dev_dbg(dev, "Found MDIO subnode\n");
		mdio = true;
	}

	if (mdio) {
		plat->mdio_bus_data =
			devm_kzalloc(dev, sizeof(struct xgmac_mdio_bus_data),
				     GFP_KERNEL);
		if (!plat->mdio_bus_data)
			return -ENOMEM;

		plat->mdio_bus_data->needs_reset = false;
	}

	return 0;
}

/**
 * xgmac_of_get_mac_mode - retrieves the interface of the MAC
 * @np - device-tree node
 * Description:
 * Similar to `of_get_phy_mode()`, this function will retrieve (from
 * the device-tree) the interface mode on the MAC side. This assumes
 * that there is mode converter in-between the MAC & PHY
 * (e.g. GMII-to-RGMII).
 */
static int xgmac_of_get_mac_mode(struct device_node *np)
{
	const char *pm;
	int err, i;

	err = of_property_read_string(np, "mac-mode", &pm);
	if (err < 0)
		return err;

	for (i = 0; i < PHY_INTERFACE_MODE_MAX; i++) {
		if (!strcasecmp(pm, phy_modes(i)))
			return i;
	}

	return -ENODEV;
}

/**
 * xgmac_probe_config_dt - parse device-tree driver parameters
 * @pdev: platform_device structure
 * @mac: MAC address to use
 * Description:
 * this function is to read the driver parameters from device-tree and
 * set some private fields that will be used by the main at runtime.
 */
struct plat_xgmacenet_data *
xgmac_probe_config_dt(struct platform_device *pdev, const char **mac)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ia_node;
	struct plat_xgmacenet_data *plat;
	struct xgmac_dma_cfg *dma_cfg;
	int rc;
	int cpu;
	u32 dac_base;
	u32 sif_base;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	*mac = of_get_mac_address(np);
	if (IS_ERR(*mac)) {
		if (PTR_ERR(*mac) == -EPROBE_DEFER)
			return ERR_CAST(*mac);

		*mac = NULL;
	}

	ia_node = of_parse_phandle(np, "interrupt-affinity", 0);
	if (ia_node) {
		cpu = of_cpu_node_to_id(ia_node);
		if (cpu >= 0)
			plat->irq_cpu = cpu;
		else
			plat->irq_cpu = 1;
	} else {
		/* Default to cpu 1 */
		plat->irq_cpu = 1;
	}

	plat->phy_interface = device_get_phy_mode(&pdev->dev);
	if (plat->phy_interface < 0)
		return ERR_PTR(plat->phy_interface);

	plat->interface = xgmac_of_get_mac_mode(np);
	if (plat->interface < 0)
		plat->interface = plat->phy_interface;

	/* Some wrapper drivers still rely on phy_node. Let's save it while
	 * they are not converted to phylink. */
	plat->phy_node = of_parse_phandle(np, "phy-handle", 0);

	/* PHYLINK automatically parses the phy-handle property */
	plat->phylink_node = np;

	/* Get max speed of operation from device tree */
	if (of_property_read_u32(np, "max-speed", &plat->max_speed))
		plat->max_speed = -1;

	plat->bus_id = of_alias_get_id(np, "ethernet");
	if (plat->bus_id < 0)
		plat->bus_id = 0;

	/* Default to phy auto-detection */
	plat->phy_addr = -1;

	/* Default to get clk_csr from xgmac_clk_csr_set(),
	 * or get clk_csr from device tree.
	 */
	plat->clk_csr = -1;
	of_property_read_u32(np, "clk_csr", &plat->clk_csr);

	if (of_property_read_u32(np, "phy-addr", &plat->phy_addr))
		plat->phy_addr = -1;
	if (plat->phy_addr >= PHY_MAX_ADDR)
		plat->phy_addr = -1;

	if (of_property_read_u32(np, "phy-lane", &plat->phy_lane))
		return ERR_PTR(-EINVAL);

	if (of_property_read_u32(np, "eth-link", &plat->eth_link))
		return ERR_PTR(-EINVAL);

	/* To Configure PHY by using all device-tree supported properties */
	rc = xgmac_dt_phy(plat, np, &pdev->dev);
	if (rc)
		return ERR_PTR(rc);

	of_property_read_u32(np, "tx-fifo-depth", &plat->tx_fifo_size);

	of_property_read_u32(np, "rx-fifo-depth", &plat->rx_fifo_size);

	plat->force_sf_dma_mode =
		of_property_read_bool(np, "snps,force_sf_dma_mode");

	/* lpi not supported */
	plat->en_tx_lpi_clockgating = false;

	/* Set the maxmtu to a default of JUMBO_LEN in case the
	 * parameter is not present in the device tree.
	 */
	plat->maxmtu = JUMBO_LEN;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	if (of_device_is_compatible(np, "edgeq,xgbe")) {
		plat->has_xgmac = 1;
		plat->pmt = 0;
		plat->tso_en = of_property_read_bool(np, "snps,tso");
	}

	plat->has_xpcs = of_property_read_bool(np, "edgeq,xpcs");
	plat->ecpri_en = of_property_read_bool(np, "edgeq,ecpri-offload");
	if (plat->ecpri_en) {
		if (plat->pmd_enabled) {
			dev_warn(&pdev->dev,
				 "Cannot enable PMD and eCPRI at same time\n");
			return ERR_PTR(-EINVAL);
		}
		dev_warn(&pdev->dev, "eCPRI enabled\n");
	}
	/* Read phy rate matching from dts */
	plat->phy_rate_matching = of_property_read_bool(np, "phy-rate-match");
	/* Read linux RPF enable flag */
	plat->linux_rpf_en = of_property_read_bool(np, "linux-rpf-en");

	/* Get Rx watchdog timer from dts */
	plat->riwt_en = of_property_read_bool(np, "snps,riwt_en");

	dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*dma_cfg),
			       GFP_KERNEL);
	if (!dma_cfg) {
		xgmac_remove_config_dt(pdev, plat);
		return ERR_PTR(-ENOMEM);
	}
	plat->dma_cfg = dma_cfg;

	of_property_read_u32(np, "snps,pbl", &dma_cfg->pbl);
	if (!dma_cfg->pbl)
		dma_cfg->pbl = DEFAULT_DMA_PBL;
	of_property_read_u32(np, "snps,txpbl", &dma_cfg->txpbl);
	of_property_read_u32(np, "snps,rxpbl", &dma_cfg->rxpbl);
	dma_cfg->pblx8 = !of_property_read_bool(np, "snps,no-pbl-x8");

	dma_cfg->aal = of_property_read_bool(np, "snps,aal");
	dma_cfg->fixed_burst = of_property_read_bool(np, "snps,fixed-burst");
	dma_cfg->mixed_burst = of_property_read_bool(np, "snps,mixed-burst");

	plat->force_thresh_dma_mode = of_property_read_bool(np, "snps,force_thresh_dma_mode");
	if (plat->force_thresh_dma_mode) {
		plat->force_sf_dma_mode = 0;
		dev_warn(&pdev->dev,
			 "force_sf_dma_mode is ignored if force_thresh_dma_mode is set.\n");
	}

	of_property_read_u32(np, "snps,ps-speed", &plat->mac_port_sel_speed);

	/* Read clock source */
	if (of_property_read_u8(np, CLOCK_SRC_PROP, &plat->soc_clk_src)) {
		dev_info(&pdev->dev, "%s not found. Default to VCTCXO!\n", CLOCK_SRC_PROP);
		plat->soc_clk_src = SoC_CLK_DEFAULT; /* default to VCTCXO */
	} else {
		dev_info(&pdev->dev, "clk src: %s\n", soc_clk_src_str[plat->soc_clk_src]);
	}

	plat->axi = xgmac_axi_setup(pdev);

	rc = xgmac_mtl_setup(pdev, plat);
	if (rc) {
		xgmac_remove_config_dt(pdev, plat);
		return ERR_PTR(rc);
	}
	if (of_property_read_u32(np, "edgeq,pmd-num-chans",
				 &plat->pmd_num_chans))
		plat->pmd_num_chans = 0;
	if (plat->pmd_num_chans >= plat->rx_chans_to_use) {
		plat->pmd_num_chans = plat->rx_chans_to_use - 1;
		dev_warn(&pdev->dev, "Not enough Rx channel for PMD\n");
		dev_warn(&pdev->dev, "Number of PMD channel is reduced to %d\n",
			 plat->pmd_num_chans);
	}
	if (plat->pmd_num_chans >= plat->tx_chans_to_use) {
		plat->pmd_num_chans = plat->tx_chans_to_use - 1;
		dev_warn(&pdev->dev, "Not enough Tx channel for PMD\n");
		dev_warn(&pdev->dev, "Number of PMD channel is reduced to %d\n",
			 plat->pmd_num_chans);
	}
	if (plat->pmd_num_chans > 0) {
		if (of_property_read_u32(np, "edgeq,pmd-start-chan",
					 &plat->pmd_start_chan))
			return ERR_PTR(-EINVAL);
		plat->pmd_enabled = true;
		dev_info(&pdev->dev, "PMD support enabled\n");
		/* Read virtual channel enable flag */
		plat->virt_chan_en = false;
		if (of_property_read_u32(np, "edgeq,virt-chan-uio-id",
					 &plat->virt_chan_uio_id))
			return ERR_PTR(-EINVAL);
	} else {
		plat->pmd_start_chan = 0;
		plat->pmd_enabled = false;
		plat->virt_chan_en = false;
	}
	if ((plat->pmd_start_chan + plat->pmd_num_chans) > plat->rx_chans_to_use ||
	    (plat->pmd_start_chan + plat->pmd_num_chans) > plat->tx_chans_to_use) {
		dev_warn(&pdev->dev,
			 "PMD channel exceeds total number of channels.\n");
		return ERR_PTR(-EINVAL);
	}

	/* clock setup */
	plat->xgmac_clk = devm_clk_get(&pdev->dev, "xgmaceth");
	if (IS_ERR(plat->xgmac_clk)) {
		dev_warn(&pdev->dev, "Cannot get CSR clock\n");
		plat->xgmac_clk = NULL;
	}
	clk_prepare_enable(plat->xgmac_clk);

	plat->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(plat->pclk)) {
		if (PTR_ERR(plat->pclk) == -EPROBE_DEFER)
			goto error_pclk_get;

		plat->pclk = NULL;
	}
	clk_prepare_enable(plat->pclk);

	/* Fall-back to main clock in case of no PTP ref is passed */
	plat->clk_ptp_ref = devm_clk_get(&pdev->dev, "ptp_ref");
	if (IS_ERR(plat->clk_ptp_ref)) {
		plat->clk_ptp_rate = clk_get_rate(plat->xgmac_clk);
		plat->clk_ptp_ref = NULL;
		dev_info(&pdev->dev, "PTP uses main clock\n");
	} else {
		plat->clk_ptp_rate = clk_get_rate(plat->clk_ptp_ref);
		dev_dbg(&pdev->dev, "PTP rate %d\n", plat->clk_ptp_rate);
	}

	if (of_property_read_u32(np, "dac-base", &dac_base))
		return ERR_PTR(-EINVAL);
	plat->dac_base = ioremap(dac_base, sizeof(unsigned long));
	if (of_property_read_u32(np, "sif-base", &sif_base))
		return ERR_PTR(-EINVAL);
	plat->sif_base = ioremap(sif_base, sizeof(unsigned long));

	/* Enable RSS by default */
	plat->rss_en = 1;

	return plat;

error_pclk_get:
	clk_disable_unprepare(plat->xgmac_clk);

	return ERR_PTR(-EPROBE_DEFER);
}

/**
 * xgmac_remove_config_dt - undo the effects of xgmac_probe_config_dt()
 * @pdev: platform_device structure
 * @plat: driver data platform structure
 *
 * Release resources claimed by xgmac_probe_config_dt().
 */
void xgmac_remove_config_dt(struct platform_device *pdev,
			     struct plat_xgmacenet_data *plat)
{
	of_node_put(plat->phy_node);
	of_node_put(plat->mdio_node);
}
#else
struct plat_xgmacenet_data *
xgmac_probe_config_dt(struct platform_device *pdev, const char **mac)
{
	return ERR_PTR(-EINVAL);
}

void xgmac_remove_config_dt(struct platform_device *pdev,
			     struct plat_xgmacenet_data *plat)
{
}
#endif /* CONFIG_OF */
EXPORT_SYMBOL_GPL(xgmac_probe_config_dt);
EXPORT_SYMBOL_GPL(xgmac_remove_config_dt);

int xgmac_get_platform_resources(struct platform_device *pdev,
				  struct xgmac_resources *xgmac_res)
{
 	struct resource *res;
	s32 err = 0;

	memset(xgmac_res, 0, sizeof(*xgmac_res));

	/* Get IRQ information early to have an ability to ask for deferred
	 * probe if needed before we went too far with resource allocation.
	 */
	xgmac_res->irq = platform_get_irq_byname(pdev, "macirq");
	if (xgmac_res->irq < 0)
		return xgmac_res->irq;

	xgmac_res->pps_irq = platform_get_irq_byname(pdev, "ppsirq");
	if (xgmac_res->pps_irq < 0)
		return xgmac_res->pps_irq;

	/* wol_irq and lpi_irq are not supported */
	xgmac_res->wol_irq = -ENXIO;
	xgmac_res->lpi_irq = -ENXIO;

	xgmac_res->xgmac_base = devm_platform_ioremap_resource_byname(pdev, "xgmac");
	err = PTR_ERR_OR_ZERO(xgmac_res->xgmac_base);
	if (err)
		return err;
	xgmac_res->xpcs_base = devm_platform_ioremap_resource_byname(pdev, "xpcs");
	err = PTR_ERR_OR_ZERO(xgmac_res->xpcs_base);
	if (err)
		return err;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ecpri");
	xgmac_res->ecpri_base = devm_ioremap_resource(&pdev->dev, res);
	xgmac_res->ecpri_phy_addr = (dma_addr_t)res->start;
	err = PTR_ERR_OR_ZERO(xgmac_res->ecpri_base);
	if (err)
		return err;
	if (pdev->num_resources > 5) {
		if (!xgbe_is_misc_addr_set()) {
			xgmac_res->misc_base =
				devm_platform_ioremap_resource_byname(pdev, "misc");
			/* misc_base is for first link only */
			err = PTR_ERR_OR_ZERO(xgmac_res->misc_base);
			if (err)
				return err;
		}
		if (pdev->num_resources > 6) {
			xgmac_res->e56_base =
				devm_platform_ioremap_resource_byname(pdev, "e56");
			/* e56_base is for link 1 only */
			err = PTR_ERR_OR_ZERO(xgmac_res->e56_base);
			if (err)
				return err;
		}
	} else {
		xgmac_res->misc_base = NULL;
		xgmac_res->e56_base = NULL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xgmac_get_platform_resources);

/**
 * xgmac_pltfr_remove
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and calls the platforms hook and release the resources (e.g. mem).
 */
int xgmac_pltfr_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	struct plat_xgmacenet_data *plat = priv->plat;
	int ret = xgmac_dvr_remove(&pdev->dev);

	if (plat->exit)
		plat->exit(pdev, plat->bsp_priv);

	xgmac_remove_config_dt(pdev, plat);

	return ret;
}
EXPORT_SYMBOL_GPL(xgmac_pltfr_remove);

#ifdef CONFIG_PM_SLEEP
/**
 * xgmac_pltfr_suspend
 * @dev: device pointer
 * Description: this function is invoked when suspend the driver and it direcly
 * call the main suspend function and then, if required, on some platform, it
 * can call an exit helper.
 */
static int xgmac_pltfr_suspend(struct device *dev)
{
	int ret;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	ret = xgmac_suspend(dev);
	if (priv->plat->exit)
		priv->plat->exit(pdev, priv->plat->bsp_priv);

	return ret;
}

/**
 * xgmac_pltfr_resume
 * @dev: device pointer
 * Description: this function is invoked when resume the driver before calling
 * the main resume function, on some platforms, it can call own init helper
 * if required.
 */
static int xgmac_pltfr_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	struct platform_device *pdev = to_platform_device(dev);

	if (priv->plat->init)
		priv->plat->init(pdev, priv->plat->bsp_priv);

	return xgmac_resume(dev);
}
#endif /* CONFIG_PM_SLEEP */

SIMPLE_DEV_PM_OPS(xgmac_pltfr_pm_ops, xgmac_pltfr_suspend,
				       xgmac_pltfr_resume);
EXPORT_SYMBOL_GPL(xgmac_pltfr_pm_ops);

MODULE_DESCRIPTION("EdgeQ 10Gb Ethernet device support");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
