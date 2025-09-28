/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 EdgeQ Inc
 */

#ifndef __XGBE_XGMAC_H__
#define __XGBE_XGMAC_H__

#ifndef CONFIG_DM_ETH
# error "Please define CONFIG_DM_ETH"
#endif

#include <asm/cache.h>
#include <net.h>
#include <linux/bitops.h>

#if CONFIG_IS_ENABLED(DM_GPIO)
#include <asm-generic/gpio.h>
#endif

#define TX_DESC_NUM		16
#define RX_DESC_NUM		16
#define ACTUAL__DESC_SIZE	16
#define ETH_BUF_SIZE		2048
#define TX_TOTAL_BUF_SIZE	(ETH_BUF_SIZE * TX_DESC_NUM)
#define RX_TOTAL_BUF_SIZE	(ETH_BUF_SIZE * RX_DESC_NUM)

#define MAC_RESET_TIMEOUT	(3 * CONFIG_SYS_HZ)
#define MDIO_TIMEOUT		(3 * CONFIG_SYS_HZ)

/* MAC Registers */
#define MAC_TX_CONFIG		0x00000000
#define XGMAC_TX_SS_MASK	GENMASK(31, 29)
#define XGMAC_TX_SS_SHIFT	29
#define XGMAC_TX_SS_10G		(0 << XGMAC_TX_SS_SHIFT)
#define XGMAC_TX_SS_2G5_GMII	(2 << XGMAC_TX_SS_SHIFT)
#define XGMAC_TX_SS_1G_GMII	(3 << XGMAC_TX_SS_SHIFT)
#define XGMAC_TX_SS_5G_XGMII	(5 << XGMAC_TX_SS_SHIFT)
#define XGMAC_TX_SS_2G5_XGMII	(6 << XGMAC_TX_SS_SHIFT)
#define XLGMAC_TX_SS_MASK	GENMASK(30, 28)
#define XLGMAC_TX_SS_SHIFT	28
#define XLGMAC_TX_SS_25G	(0x1 << XLGMAC_TX_SS_SHIFT)
#define XLGMAC_TX_SS_50G	(0x2 << XLGMAC_TX_SS_SHIFT)
#define XLGMAC_TX_SS_10G	(0x4 << XLGMAC_TX_SS_SHIFT)
#define MAC_CONFIG_JD		BIT(16)
#define MAC_CONFIG_TE		BIT(0)
#define MAC_RX_CONFIG		0x00000004
#define MAC_CONFIG_LM		BIT(10)
#define MAC_CONFIG_IPC		BIT(9)
#define MAC_CONFIG_JE		BIT(8)
#define MAC_CONFIG_RE		BIT(0)
#define MAC_CORE_INIT_RX	(MAC_CONFIG_IPC | MAC_CONFIG_JE)
#define MAC_PACKET_FILTER	0x00000008
#define MAC_FILTER_RA		BIT(31)
#define MAC_FILTER_IPFE		BIT(20)
#define MAC_FILTER_VTFE		BIT(16)
#define MAC_FILTER_HPF		BIT(10)
#define MAC_FILTER_PR		BIT(0)

#define XGMAC_RXQ_CTRL0		0x000000a0
#define XLGMAC_RXQ_CTRL0	0x00000140
#define MAC_RXQEN_MASK(x)	GENMASK((x) * 2 + 1, (x) * 2)
#define MAC_RXQEN_SHIFT(x)	((x) * 2)
#define MAC_RXQEN		2

#define MAC_VERSION		0x00000110
#define MAC_VID_MASK		GENMASK(15, 8)
#define MAC_VID_SHIFT		8
#define MAC_VID_XGMAC		0x76
#define MAC_VID_XLGMAC		0x27

#define MAC_HW_FEATURE0		0x0000011c
#define MAC_HW_FEATURE1		0x00000120
#define MAC_HW_FEATURE2		0x00000124
#define MAC_HW_FEATURE3		0x00000128

/* MII registers */
#define MAC_MDIO_ADDR		0x00000200
#define MII_DA_MASK		GENMASK(25, 21)
#define MII_DA_SHIFT		21
#define MII_ADDR_MASK		GENMASK(20, 16)
#define MII_ADDR_SHIFT		16
#define MII_REG_MASK		GENMASK(15, 0)
#define MII_REG_SHIFT		0
#define MAC_MDIO_DATA		0x00000204
#define MII_CRS			BIT(31)
#define MII_BUSY		BIT(22)
#define MII_CLK_MASK		GENMASK(21, 19)
#define MII_CLK_SHIFT		19
#define MII_CLK_100_150M	0
#define MII_CLK_150_250M	1
#define MII_CLK_250_300M	2
#define MII_CLK_300_350M	3
#define MII_CLK_350_400M	4
#define MII_CLK_400_450M	5
#define MII_CMD_MASK		GENMASK(17, 16)
#define MII_CMD_SHIFT		16
#define MII_CMD_WRITE		1
#define MII_CMD_READ		3
#define MII_DATA_MASK		GENMASK(15, 0)
#define MII_DATA_SHIFT		0
#define MAC_MDIO_C22P		0x00000220
#define MAC_ADDR0_HIGH		0x00000300
#define MAC_ADDR0_LOW		0x00000304

#define MAC_TX_PKT_CNT_LOW	0x0000081c
#define MAC_RX_PKT_CNT_LOW	0x00000900

#define MAC_TIMESTAMP_CTRL	0x00000d00
#define MAC_TSCTRLSSR		BIT(9)
#define MAC_TSINIT		BIT(2)
#define MAC_TSENA		BIT(0)
#define MAC_SUB_SEC_INC		0x00000d04
#define MAC_PPS_CONTROL		0x00000d70

/* MTL Registers */
#define MTL_RXQ_DMA_MAP0	0x00001030
#define MTL_QxMDMACH_MASK(x)	GENMASK((x) * 8 + 7, (x) * 8)
#define MTL_QxMDMACH_SHIFT(x)	((x) * 8)
#define MTL_TXQ_OPMODE(x)	(0x00001100 + (0x80 * (x)))
#define MTL_TQS_MASK		GENMASK(25, 16)
#define MTL_TQS_SHIFT		16
#define MTL_TXQEN_MASK		GENMASK(3, 2)
#define MTL_TXQEN_SHIFT		2
#define MTL_TXQEN		2
#define MTL_TSF			BIT(1)
#define MTL_FTQ			BIT(0)
#define MTL_RXQ_OPMODE(x)	(0x00001140 + (0x80 * (x)))
#define MTL_RQS_MASK		GENMASK(25, 16)
#define MTL_RQS_SHIFT		16
#define MTL_DIS_TCP_EF		BIT(6)
#define MTL_RSF			BIT(5)
#define MTL_FEF			BIT(4)

#define TX_FIFO_SIZE		8192
#define RX_FIFO_SIZE		8192

/* DMA Registers */
#define DMA_MODE		0x00003000
#define DMA_SWR			BIT(0)
#define DMA_SYSBUS_MODE		0x00003004
#define DMA_WR_OSR_LMT		GENMASK(29, 24)
#define DMA_WR_OSR_LMT_SHIFT	24
#define DMA_RD_OSR_LMT		GENMASK(21, 16)
#define DMA_RD_OSR_LMT_SHIFT	16
#define DMA_OSR_LMT_32		31  // Actual OSR is LMT + 1
#define DMA_AAL			BIT(12)
#define DMA_EAME		BIT(11)
#define DMA_BLEN_MASK		GENMASK(7, 1)
#define DMA_BLEN8		BIT(2)
#define DMA_UNDEF		BIT(0)

#define DMA_CH_CONTROL(x)	(0x00003100 + (0x80 * (x)))
#define DMA_DSL_MASK		GENMASK(20, 18)
#define DMA_DSL_SHIFT		18
#define DMA_CH_TX_CONTROL(x)	(0x00003104 + (0x80 * (x)))
#define DMA_TxPBL_MASK		GENMASK(21, 16)
#define DMA_TxPBL_SHIFT		16
#define DEFAULT_DMA_PBL		8
#define DMA_TXST		BIT(0)
#define DMA_CH_RX_CONTROL(x)	(0x00003108 + (0x80 * (x)))
#define DMA_RxPBL_MASK		GENMASK(21, 16)
#define DMA_RxPBL_SHIFT		16
#define DMA_RBSZ_MASK		GENMASK(14, 1)
#define DMA_RBSZ_SHIFT		1
#define DMA_RXST		BIT(0)
#define DMA_CH_TxDESC_HADDR(x)	(0x00003110 + (0x80 * (x)))
#define DMA_CH_TxDESC_LADDR(x)	(0x00003114 + (0x80 * (x)))
#define DMA_CH_RxDESC_HADDR(x)	(0x00003118 + (0x80 * (x)))
#define DMA_CH_RxDESC_LADDR(x)	(0x0000311c + (0x80 * (x)))
#define DMA_CH_TxDESC_TAIL_LPTR(x)	(0x00003124 + (0x80 * (x)))
#define DMA_CH_RxDESC_TAIL_LPTR(x)	(0x0000312c + (0x80 * (x)))
#define DMA_CH_TxDESC_RING_LEN(x)	(0x00003130 + (0x80 * (x)))
#define DMA_CH_RxDESC_RING_LEN(x)	(0x00003134 + (0x80 * (x)))

/* Descriptors */
#define TDES2_B1L		GENMASK(13, 0)
#define TDES3_OWN		BIT(31)
#define TDES3_CTXT		BIT(30)
#define TDES3_FD		BIT(29)
#define TDES3_LD		BIT(28)
#define TDES3_CPC		GENMASK(27, 26)
#define TDES3_CPC_SHIFT		26
#define TDES3_TCMSSV		BIT(26)
#define TDES3_SAIC		GENMASK(25, 23)
#define TDES3_SAIC_SHIFT	23
#define TDES3_TBSV		BIT(24)
#define TDES3_THL		GENMASK(22, 19)
#define TDES3_THL_SHIFT		19
#define TDES3_IVTIR		GENMASK(19, 18)
#define TDES3_IVTIR_SHIFT	18
#define TDES3_TSE		BIT(18)
#define TDES3_IVLTV		BIT(17)
#define TDES3_CIC		GENMASK(17, 16)
#define TDES3_CIC_SHIFT		16
#define TDES3_TPL		GENMASK(17, 0)
#define TDES3_VLTV		BIT(16)
#define TDES3_VT		GENMASK(15, 0)
#define TDES3_FL		GENMASK(14, 0)
#define RDES2_HL		GENMASK(9, 0)
#define RDES3_OWN		BIT(31)
#define RDES3_CTXT		BIT(30)
#define RDES3_IOC		BIT(30)
#define RDES3_LD		BIT(28)
#define RDES3_CDA		BIT(27)
#define RDES3_RSV		BIT(26)
#define RDES3_L34T		GENMASK(23, 20)
#define RDES3_L34T_SHIFT	20
#define RDES3_L34T_NONIP	0x0
#define RDES3_L34T_IP4TCP	0x1
#define RDES3_L34T_IP4UDP	0x2
#define RDES3_L34T_IP4ICMP	0x3
#define RDES3_L34T_IP4IGMP	0x4
#define RDES3_L34T_IP6TCP	0x9
#define RDES3_L34T_IP6UDP	0xA
#define RDES3_L34T_IP6ICMP	0xB
#define RDES3_ET		GENMASK(19, 16)
#define RDES3_ET_SHIFT		16
#define RDES3_ET_CRC		0x3
#define RDES3_ET_IP_HDR		0x5
#define RDES3_ET_CHKSUM		0x6
#define RDES3_ES		BIT(15)
#define RDES3_PL		GENMASK(13, 0)
#define RDES3_TSD		BIT(6)
#define RDES3_TSA		BIT(4)

#define OF_NULL_ADDR		(-1ULL)

#define PTN_E32_REG_BASE	0x3C2D0000
#define IOSS_PLL_CFG_BASE	0x3CB00000
#define IOSS_CCIX_CFG_BASE	0x3CB40000
#define IOSS_SHTDWN		0x90
#define IOMMU_SHTDWN		0x94

#define NO_PHY_RESET_GPIO	0xFFFFFFFF

#define NO_LOOPBACK		0
#define MAC_LOOPBACK		1
#define SERDES_LOOPBACK		2
#define PHY_HOST_LOOPBACK	3
#define PHY_LINE_LOOPBACK	4
#define PHY_SERDES_LOOPBACK	5

struct dma_desc {
	u32 des0;
	u32 des1;
	u32 des2;
	u32 des3;
} __aligned(ARCH_DMA_MINALIGN);

struct dw_eth_dev {
	struct dma_desc tx_desc[TX_DESC_NUM] __aligned(ARCH_DMA_MINALIGN);
	struct dma_desc rx_desc[RX_DESC_NUM] __aligned(ARCH_DMA_MINALIGN);
	u8 tx_buff[TX_TOTAL_BUF_SIZE] __aligned(ARCH_DMA_MINALIGN);
	u8 rx_buff[RX_TOTAL_BUF_SIZE] __aligned(ARCH_DMA_MINALIGN);

	struct udevice *dev;
	u32 interface;
	u32 max_speed;
	s32 phy_addr;
	s32 phy_lane;
	u32 eth_link;
	u32 phy_reset_gpio;
	u32 tx_currdescnum;
	u32 rx_currdescnum;

	phys_addr_t mac_base;
#ifdef CONFIG_RAPTOR_FAMILY
	phys_addr_t misc_base;
#endif
#ifdef CONFIG_EQXPCS
	phys_addr_t xpcs_base;
	phys_addr_t e56_base;
	struct xpcs_ops *xpcs;
#endif
#if CONFIG_IS_ENABLED(DM_GPIO)
	struct gpio_desc reset_gpio;
#endif
#ifdef CONFIG_CLK
	struct clk *clocks;	/* clock list */
	int clock_count;	/* number of clock in clock list */
#endif
	struct phy_device *phydev;
	struct mii_dev *bus;
	u8 is_xlgmac;
	bool use_i2c;
	bool phy_rate_matching;
	u16 curr_speed;
};

int dwxgmac_eth_ofdata_to_platdata(struct udevice *dev);
int dwxgmac_eth_probe(struct udevice *dev);
extern const struct eth_ops dwxgmac_eth_ops;
extern u16 config_eth_speed[];
extern u16 config_eth_link;
extern u16 loopback_test;

struct dw_eth_pdata {
	struct eth_pdata eth_pdata;
	u32 reset_delays[3];
};

int dwxgmac_eth_init(struct dw_eth_dev *priv, u8 *enetaddr);
int dwxgmac_eth_enable(struct dw_eth_dev *priv);
int dwxgmac_eth_send(struct udevice *dev, void *packet, int length);
int dwxgmac_eth_recv(struct udevice *dev, int flags, uchar **packetp);
int dwxgmac_eth_free_pkt(struct udevice *dev, uchar *packet, int length);
void dwxgmac_eth_stop(struct udevice *dev);
int dwxgmac_eth_write_hwaddr(struct udevice *dev);

#endif /* __XGBE_XGMAC_H__ */
