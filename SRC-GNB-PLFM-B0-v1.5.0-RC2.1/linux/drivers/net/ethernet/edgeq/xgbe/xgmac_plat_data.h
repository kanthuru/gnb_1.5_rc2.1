/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
  Header file for xgmac platform data

*******************************************************************************/

#ifndef __XGMAC_PLATFORM_DATA
#define __XGMAC_PLATFORM_DATA

#include <linux/platform_device.h>
#include <linux/phy.h>

#define MTL_MAX_RX_QUEUES	16
#define MTL_MAX_TX_QUEUES	16
#define MTL_MIN_RX_QUEUES	2
#define MAX_DMA_CHANS		16

#define XGMAC_RX_COE_NONE	0
#define XGMAC_RX_COE_EN	1

/* Define the macros for CSR clock range parameters to be passed by
 * platform code.
 * This could also be configured at run time using CPU freq framework. */

/* MDC Clock Selection define*/
#define	XGMAC_CSR_60_100M	0x0	/* MDC = clk_scr_i/42 */
#define	XGMAC_CSR_100_150M	0x1	/* MDC = clk_scr_i/62 */
#define	XGMAC_CSR_20_35M	0x2	/* MDC = clk_scr_i/16 */
#define	XGMAC_CSR_35_60M	0x3	/* MDC = clk_scr_i/26 */
#define	XGMAC_CSR_150_250M	0x4	/* MDC = clk_scr_i/102 */
#define	XGMAC_CSR_250_300M	0x5	/* MDC = clk_scr_i/122 */

/* MTL algorithms identifiers */
#define MTL_TX_ALGORITHM_WRR	0x0
#define MTL_TX_ALGORITHM_WFQ	0x1
#define MTL_TX_ALGORITHM_DWRR	0x2
#define MTL_TX_ALGORITHM_SP	0x3
#define MTL_RX_ALGORITHM_SP	0x4
#define MTL_RX_ALGORITHM_WSP	0x5

/* RX/TX Queue Mode */
#define MTL_QUEUE_AVB		0x0
#define MTL_QUEUE_DCB		0x1

/* The MDC clock could be set higher than the IEEE 802.3
 * specified frequency limit 0f 2.5 MHz, by programming a clock divider
 * of value different than the above defined values. The resultant MDIO
 * clock frequency of 12.5 MHz is applicable for the interfacing chips
 * supporting higher MDC clocks.
 * The MDC clock selection macros need to be defined for MDC clock rate
 * of 12.5 MHz, corresponding to the following selection.
 */
#define XGMAC_CSR_I_4		0x8	/* clk_csr_i/4 */
#define XGMAC_CSR_I_6		0x9	/* clk_csr_i/6 */
#define XGMAC_CSR_I_8		0xA	/* clk_csr_i/8 */
#define XGMAC_CSR_I_10		0xB	/* clk_csr_i/10 */
#define XGMAC_CSR_I_12		0xC	/* clk_csr_i/12 */
#define XGMAC_CSR_I_14		0xD	/* clk_csr_i/14 */
#define XGMAC_CSR_I_16		0xE	/* clk_csr_i/16 */
#define XGMAC_CSR_I_18		0xF	/* clk_csr_i/18 */

/* AXI DMA Burst length supported */
#define DMA_AXI_BLEN_4		(1 << 1)
#define DMA_AXI_BLEN_8		(1 << 2)
#define DMA_AXI_BLEN_16		(1 << 3)
#define DMA_AXI_BLEN_32		(1 << 4)
#define DMA_AXI_BLEN_64		(1 << 5)
#define DMA_AXI_BLEN_128	(1 << 6)
#define DMA_AXI_BLEN_256	(1 << 7)
#define DMA_AXI_BLEN_ALL (DMA_AXI_BLEN_4 | DMA_AXI_BLEN_8 | DMA_AXI_BLEN_16 \
			| DMA_AXI_BLEN_32 | DMA_AXI_BLEN_64 \
			| DMA_AXI_BLEN_128 | DMA_AXI_BLEN_256)

/* Platfrom data for platform device structure's platform_data field */

struct xgmac_mdio_bus_data {
	unsigned int phy_mask;
	unsigned int has_xpcs;
	int *irqs;
	int probed_phy_irq;
	bool needs_reset;
};

struct xgmac_dma_cfg {
	int pbl;
	int txpbl;
	int rxpbl;
	bool pblx8;
	int fixed_burst;
	int mixed_burst;
	bool aal;
	bool eame;
};

#define AXI_BLEN	7
struct xgmac_axi {
	bool axi_lpi_en;
	bool axi_xit_frm;
	u32 axi_wr_osr_lmt;
	u32 axi_rd_osr_lmt;
	bool axi_kbbe;
	u32 axi_blen[AXI_BLEN];
	bool axi_fb;
	bool axi_mb;
	bool axi_rb;
};

#define EST_GCL		1024
struct xgmac_est {
	int enable;
	u32 btr_offset[2];
	u32 btr[2];
	u32 ctr[2];
	u32 ter;
	u32 gcl_unaligned[EST_GCL];
	u32 gcl[EST_GCL];
	u32 gcl_size;
};

struct xgmac_rxq_cfg {
	u8 mode_to_use;
	u32 chan;
	u8 pkt_route;
	bool use_prio;
	u32 prio;
};

struct xgmac_rx_ch_cfg {
	u32 ch_irq;
	u32 ch_irq_cpu;
};

struct xgmac_txq_cfg {
	u32 weight;
	u8 mode_to_use;
	/* Credit Base Shaper parameters */
	u32 send_slope;
	u32 idle_slope;
	u32 high_credit;
	u32 low_credit;
	bool use_prio;
	u32 prio;
};

struct xgmac_tx_ch_cfg {
	int tbs_en;
	u32 ch_irq;
	u32 ch_irq_cpu;
};

struct plat_xgmacenet_data {
	int bus_id;
	int phy_addr;
	int interface;
	phy_interface_t phy_interface;
	struct xgmac_mdio_bus_data *mdio_bus_data;
	struct device_node *phy_node;
	struct device_node *phylink_node;
	struct device_node *mdio_node;
	struct xgmac_dma_cfg *dma_cfg;
	struct xgmac_est *est;
	int clk_csr;
	int has_gmac;
	int enh_desc;
	int tx_coe;
	int rx_coe;
	int bugged_jumbo;
	int pmt;
	int force_sf_dma_mode;
	int force_thresh_dma_mode;
	int riwt_en;
	int max_speed;
	int maxmtu;
	int multicast_filter_bins;
	int unicast_filter_entries;
	int tx_fifo_size;
	int rx_fifo_size;
	u32 rx_queues_to_use;
	u32 tx_queues_to_use;
	u32 rx_chans_to_use;
	u32 tx_chans_to_use;
	u8 rx_sched_algorithm;
	u8 tx_sched_algorithm;
	struct xgmac_rxq_cfg rx_queues_cfg[MTL_MAX_RX_QUEUES];
	struct xgmac_txq_cfg tx_queues_cfg[MTL_MAX_TX_QUEUES];
	struct xgmac_rx_ch_cfg rx_chans_cfg[MAX_DMA_CHANS];
	struct xgmac_tx_ch_cfg tx_chans_cfg[MAX_DMA_CHANS];
	void (*fix_mac_speed)(void *priv, unsigned int speed);
	int (*serdes_powerup)(struct net_device *ndev, void *priv);
	void (*serdes_powerdown)(struct net_device *ndev, void *priv);
	int (*init)(struct platform_device *pdev, void *priv);
	void (*exit)(struct platform_device *pdev, void *priv);
	struct mac_device_info *(*setup)(void *priv);
	void *bsp_priv;
	struct clk *xgmac_clk;
	struct clk *pclk;
	struct clk *clk_ptp_ref;
	unsigned int clk_ptp_rate;
	unsigned int clk_ref_rate;
	s32 ptp_max_adj;
	struct reset_control *xgmac_rst;
	struct xgmac_axi *axi;
	int has_gmac4;
	bool has_sun8i;
	bool tso_en;
	int rss_en;
	int mac_port_sel_speed;
	bool en_tx_lpi_clockgating;
	int has_xgmac;
	int has_xpcs;
	bool ecpri_en;
	bool per_ch_irq;
	bool dynamic_dma_map;
	int phy_lane;
	int eth_link;
	u8 soc_clk_src; /* SoC Clock Type */
	int sph_en;
	bool phy_rate_matching;
	bool pmd_enabled;
	u32 irq_cpu;
	bool linux_rpf_en;
	u32 pmd_num_chans;
	u32 pmd_start_chan;
	void __iomem *dac_base;
	void __iomem *sif_base;
	bool virt_chan_en;
	u32 virt_chan_uio_id;
};
#endif
