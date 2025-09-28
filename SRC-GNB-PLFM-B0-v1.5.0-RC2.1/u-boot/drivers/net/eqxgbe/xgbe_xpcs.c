// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 EdgeQ Inc.
 *
 * XPCS/XLGPCS/E56 APB3 API. E32 indirect access API.
 *
 */

#include <common.h>
#include <errno.h>
#include <miiphy.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/io.h>
#include "xgbe_xgmac.h"
#include "xgbe_misc.h"
#include "xgbe_xpcs.h"
#include "xgbe_b0_e32_1g_2g5_10g_25g.h"

/* Note: using 32-bit write because of the address translation in IOSS HW */

static phys_addr_t master_lane_ioaddr = 0;
static bool master_is_xpcs = true;

#define xpcs_warn(__device, __link, __args...) \
({ \
	if (__link) \
		dev_warn(__device, ##__args); \
})

static int ioss_pvt_mon_read(struct dw_eth_dev *priv)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 value;
	u32 status ;
	int retry = 20;

	value = 0x0;
	writel(value, misc_addr + XGBE_PVT_MON_CFG);
	udelay(10);
	value = PVT_MON_DEFAULT_SETTING;
	writel(value, misc_addr + XGBE_PVT_MON_CFG);
	udelay(10);
	value |= PVT_MON_CLK_EN;
	writel(value, misc_addr + XGBE_PVT_MON_CFG);
	udelay(10);
	value |= PVT_MON_EN;
	writel(value, misc_addr + XGBE_PVT_MON_CFG);
	udelay(200);

	status = readl(misc_addr + XGBE_PVT_MON_STATUS);
	while ((status & PVT_MON_DATA_VALID) != PVT_MON_DATA_VALID && --retry > 0) {
		udelay(10);
		status = readl(misc_addr + XGBE_PVT_MON_STATUS);
	};

	if ((status & PVT_MON_DATA_VALID) != PVT_MON_DATA_VALID)
		return -1;

	return (status >> 1);
}

static int xpcs_read_fault_10g(struct udevice *dev, phys_addr_t ioaddr,
			       int link)
{
	u16 value;

#if 0
	value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
	if (value & DW_RX_STS1_FAULT) {
		xpcs_warn(dev, link, "Link fault condition detected!\n");
		return -EFAULT;
	}

	value = readw(ioaddr + DW_XPCS_SR_PCS_STS2);
	if (value & DW_RX_STS2_FAULT)
		xpcs_warn(dev, link, "Receiver fault detected!\n");
	if (value & DW_TX_STS2_FAULT)
		xpcs_warn(dev, link, "Transmitter fault detected!\n");
#endif

	value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		xpcs_warn(dev, link, "FIFO fault condition detected!\n");
		return -EFAULT;
	}

#if 0
	value = readw(ioaddr + DW_XPCS_SR_PCS_KR_STS1);
	if (!(value & DW_RPCS_BKLK))
		xpcs_warn(dev, link, "Link is not locked!\n");

	value = readw(ioaddr + DW_XPCS_SR_PCS_KR_STS2);
	if (value & DW_ERR_BLK) {
		xpcs_warn(dev, link, "Link has errors!\n");
		return -EFAULT;
	}
#endif

	return 0;
}

static int xpcs_read_fault_25g(struct udevice *dev, phys_addr_t ioaddr,
			       int link)
{
	u16 value;

#if 0
	value = readw(ioaddr + DW_25G_SR_PCS_STS1);
	if (value & DW_RX_STS1_FAULT) {
		xpcs_warn(dev, link, "Link fault condition detected!\n");
		return -EFAULT;
	}

	value = readw(ioaddr + DW_25G_SR_PCS_STS2);
	if (value & DW_RX_STS2_FAULT)
		xpcs_warn(dev, link, "Receiver fault detected!\n");
	if (value & DW_TX_STS2_FAULT)
		xpcs_warn(dev, link, "Transmitter fault detected!\n");
#endif

	value = readw(ioaddr + DW_25G_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		value = readw(ioaddr + DW_25G_VR_PCS_DIG_STS);
		if (value & DW_RXFIFO_ERR) {
			xpcs_warn(dev, link, "FIFO fault condition detected!\n");
			return -EFAULT;
		}
	}

#if 0
	value = readw(ioaddr + DW_25G_SR_PCS_BASER_STS1);
	if (!(value & DW_RPCS_BKLK))
		xpcs_warn(dev, link, "Link is not locked!\n");

	value = readw(ioaddr + DW_25G_SR_PCS_BASER_STS2);
	if (value & DW_ERR_BLK) {
		xpcs_warn(dev, link, "Link has errors!\n");
		return -EFAULT;
	}
#endif

	return 0;
}

static int xpcs_read_fault_50g(struct udevice *dev, phys_addr_t ioaddr,
			       int link)
{
	u16 value;

#if 0
	value = readw(ioaddr + DW_50G_SR_PCS_STS1);
	if (value & DW_RX_STS1_FAULT) {
		xpcs_warn(dev, link, "Link fault condition detected!\n");
		return -EFAULT;
	}

	value = readw(ioaddr + DW_50G_SR_PCS_STS2);
	if (value & DW_RX_STS2_FAULT)
		xpcs_warn(dev, link, "Receiver fault detected!\n");
	if (value & DW_TX_STS2_FAULT)
		xpcs_warn(dev, link, "Transmitter fault detected!\n");
#endif

	value = readw(ioaddr + DW_50G_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		xpcs_warn(dev, link, "FIFO fault condition detected!\n");
		return -EFAULT;
	}

#if 0
	value = readw(ioaddr + DW_50G_SR_PCS_BASER_STS1);
	if (!(value & DW_RPCS_BKLK))
		xpcs_warn(dev, link, "Link is not locked!\n");

	value = readw(ioaddr + DW_50G_SR_PCS_BASER_STS2);
	if (value & DW_ERR_BLK) {
		xpcs_warn(dev, link, "Link has errors!\n");
		return -EFAULT;
	}
#endif

	return 0;
}

static u32 xpcs_get_id(phys_addr_t ioaddr_1, phys_addr_t ioaddr_2)
{
	u32 xpcs_id;

	xpcs_id = (u32)readw(ioaddr_1);
	xpcs_id = (xpcs_id << 16) + readw(ioaddr_2);

	return xpcs_id;
}

static int xpcs_poll_clear(phys_addr_t ioaddr, u16 mon_bit, u16 usec)
{
	/* Poll until the bit clears (1ms per retry == 0.5 sec) */
	u16 retries = 500;
	u16 value;
	int ret;

	do {
		udelay(usec);
		value = readw(ioaddr);
	} while ((value & mon_bit) && --retries);

	ret = (value & mon_bit) ? -ETIMEDOUT : 0;

	return ret;
}

static int xpcs_poll_set(phys_addr_t ioaddr, u16 mon_bit)
{
	/* Poll until the bit sets (1ms per retry == 0.1 sec) */
	u16 retries = 100;
	u16 value;
	int ret;

	do {
		udelay(1000);
		value = readw(ioaddr);
	} while (!(value & mon_bit) && --retries);

	ret = !(value & mon_bit) ? -ETIMEDOUT : 0;

	return ret;
}

static int xpcs_soft_reset(phys_addr_t ioaddr)
{
/* Soft reset could cause u-boot hang. Not reset for now. */
#if 0
	u16 value;

	value = readw(ioaddr);
	value |= DW_CTRL1_RST;
	writel(value, ioaddr);

	return xpcs_poll_clear(ioaddr, DW_CTRL1_RST, 1000);
#endif
	return 0;
}

static u16 read_e32_cr_10g(phys_addr_t ioaddr, u16 addr)
{
	u16 value;
	int ret;

	ret = xpcs_poll_clear(ioaddr+DW_XPCS_VR_PMA_SNPS_CR_CTRL,
			      DW_START_BUSY, 1);
	if (ret)
		return ret;

	writel(addr, ioaddr + DW_XPCS_VR_PMA_SNPS_CR_ADDR);
	value = readw(ioaddr + DW_XPCS_VR_PMA_SNPS_CR_CTRL);
	value &= ~DW_WR_RDN;
	value |= DW_START_BUSY;
	writel(value, ioaddr + DW_XPCS_VR_PMA_SNPS_CR_CTRL);

	ret = xpcs_poll_clear(ioaddr+DW_XPCS_VR_PMA_SNPS_CR_CTRL,
			      DW_START_BUSY, 1);
	if (ret)
		return ret;

	value = readw(ioaddr + DW_XPCS_VR_PMA_SNPS_CR_DATA);
	return value;
}

static int write_e32_cr_10g(phys_addr_t ioaddr, u16 *data_pair)
{
	u16 value;
	int ret;

	ret = xpcs_poll_clear(ioaddr+DW_XPCS_VR_PMA_SNPS_CR_CTRL,
			      DW_START_BUSY, 1);
	if (ret)
		return ret;

	writel(*data_pair, ioaddr + DW_XPCS_VR_PMA_SNPS_CR_ADDR);
	writel(*(data_pair+1), ioaddr + DW_XPCS_VR_PMA_SNPS_CR_DATA);
	value = readw(ioaddr + DW_XPCS_VR_PMA_SNPS_CR_CTRL);
	value |= (DW_WR_RDN | DW_START_BUSY);
	writel(value, ioaddr + DW_XPCS_VR_PMA_SNPS_CR_CTRL);

	return 0;
}

static int xpcs_prog_e32_10g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 data_pair[2];
	u16 value;
	int array_size;
	int i;

	/* Program 4-lane configuration */
	data_pair[0] = 0x6054;
	data_pair[1] = 0x280C;
	write_e32_cr_10g(ioaddr, data_pair);
	data_pair[0] = 0x6052;
	data_pair[1] = 0x2528;
	write_e32_cr_10g(ioaddr, data_pair);

	array_size = sizeof(image_1g_2g5_10g_25g) /
		     sizeof(image_1g_2g5_10g_25g[0]);
	for (i = 0; i < array_size; i++)
		write_e32_cr_10g(ioaddr, image_1g_2g5_10g_25g[i]);

	/* Programming SRAM done */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_SRAM);
	value |= DW_EXT_LD_DN;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_SRAM);

	return xpcs_poll_clear(ioaddr+DW_XPCS_SR_PCS_CTRL1, DW_CTRL1_RST, 1000);
}

static int xpcs_probe_10g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 data_pair[2];
	u32 xpcs_id;
	u16 value;
	int ret;

	if (priv->phy_lane == 0) {
		master_lane_ioaddr = ioaddr;
		master_is_xpcs = true;

		/* Use default for USXGMII */
		if (priv->interface == PHY_INTERFACE_MODE_SGMII) {
			/* Override Tx and Rx width */
			data_pair[0] = 0x1021;
			data_pair[1] = 0x9000;
			write_e32_cr_10g(ioaddr, data_pair);
			data_pair[0] = 0x1002;
			data_pair[1] = 0x90;
			write_e32_cr_10g(ioaddr, data_pair);
		}

		ret = xpcs_prog_e32_10g(priv);
		if (ret)
			return ret;
	}

	xpcs_id = xpcs_get_id(ioaddr + DW_XPCS_SR_PCS_DEV_ID1,
			      ioaddr + DW_XPCS_SR_PCS_DEV_ID2);

	if (xpcs_id != DW_XPCS_ID)
		return -ENODEV;

	/* Set PCS speed */
	value = readw(ioaddr + DW_XPCS_SR_PCS_CTRL2);
	value &= ~DW_XPCS_TYPE_SEL;
	if (priv->interface == PHY_INTERFACE_MODE_SGMII)
		/* SGMII selects 10GBaseX */
		value |= DW_XPCS_TYPE_10GBASE_X;
	else
		/* USXGMII selects 10GBaseR */
		value |= DW_XPCS_TYPE_10GBASE_R;
	writel(value, ioaddr + DW_XPCS_SR_PCS_CTRL2);

	/* Select mode */
	value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
	if (priv->interface == PHY_INTERFACE_MODE_SGMII) {
		if (priv->max_speed == SPEED_2500) {
			value |= DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		}
		else if (priv->max_speed == SPEED_1000) {
			value &= ~DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		}
	}
	else {
		/* Enable USXGMII */
		value &= ~DW_EN_2_5G_MODE;
		value |= DW_USXG_EN;
	}
	writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);

	if (priv->interface == PHY_INTERFACE_MODE_SGMII) {
		value = readw(ioaddr + DW_XPCS_VR_MII_AN_CTRL);
		value &= DW_PCS_MODE;
		value |= DW_PCS_MODE_SGMII;
		writel(value, ioaddr + DW_XPCS_VR_MII_AN_CTRL);
	}
	else {
		value = readw(ioaddr + DW_XPCS_VR_PCS_KR_CTRL);
		value &= ~DW_USXG_MODE;
		value |= DW_USXG_MODE_10G_SXGMII;
		writel(value, ioaddr + DW_XPCS_VR_PCS_KR_CTRL);

		/* Configure PHY for 32-bit */
		value = readw(ioaddr + DW_XPCS_VR_PMA_25G_RX_WIDTH_CTRL);
		value &= ~DW_RX0_WIDTH;
		value |= DW_RX0_WIDTH_32BIT;
		writel(value, ioaddr + DW_XPCS_VR_PMA_25G_RX_WIDTH_CTRL);
		value = readw(ioaddr + DW_XPCS_VR_PMA_25G_TX_WIDTH_CTRL);
		value &= ~DW_TX0_WIDTH;
		value |= DW_TX0_WIDTH_32BIT;
		writel(value, ioaddr + DW_XPCS_VR_PMA_25G_TX_WIDTH_CTRL);
	}
	udelay(1000);

	/* Configure MII speed */
	value = readw(ioaddr + DW_XPCS_SR_MII_CTRL);
	value &= ~DW_USXGMII_SS;
	if (priv->interface == PHY_INTERFACE_MODE_SGMII)
		value |= DW_SGMII_1000;
	else if (priv->max_speed == SPEED_5000)
		value |= DW_USXGMII_5000 | DW_USXGMII_FULL;
	else if (priv->max_speed == SPEED_2500)
		value |= DW_USXGMII_2500 | DW_USXGMII_FULL;
	else if (priv->max_speed == SPEED_1000)
		value |= DW_USXGMII_1000 | DW_USXGMII_FULL;
	else
		value |= DW_USXGMII_10000 | DW_USXGMII_FULL;
	writel(value, ioaddr + DW_XPCS_SR_MII_CTRL);
	udelay(1);	// Wait for 1us for XGMII clock to stablize

	if (master_lane_ioaddr == 0)
		return -ENODEV;

	/* LOS from PHY can be unreliable, supress it */
	value = readw(ioaddr + DW_XPCS_VR_PCS_DEBUG_CTRL);
	value |= (DW_SUPRESS_LOS_DET | DW_RX_DT_EN_CTL);
	writel(value, ioaddr + DW_XPCS_VR_PCS_DEBUG_CTRL);

	ret = xpcs_poll_set(ioaddr + DW_XPCS_VR_PMA_RX_LSTS,
			    DW_XPCS_RX_VALID_0);
	if (ret)
		return ret;

	value = readw(ioaddr + DW_XPCS_VR_PMA_32G_RX_EQ_CTRL4);
	value &= ~DW_RX_AD_REQ;
	writel(value, ioaddr + DW_XPCS_VR_PMA_32G_RX_EQ_CTRL4);

	if (priv->interface == PHY_INTERFACE_MODE_USXGMII) {
		/* USXGMII rate adaptor reset */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		value |= DW_USRA_RST;
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);

		ret =  xpcs_poll_clear(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1,
				       DW_USRA_RST, 1000);
		if (ret)
			printf("DW_USRA_RST not cleared\n");
	}

	return 0;
}

static int xpcs_rx_reset_10g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;

	/* Rx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	value |= DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	udelay(50000);

	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	value &= ~DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	udelay(1000);

	return 0;
}

static int xpcs_config_10g(struct dw_eth_dev *priv)
{
	static bool rx_reset_done_10g = false;
	phys_addr_t ioaddr = priv->xpcs_base;
	int ret;

	/* Do Rx reset only once. Otherwise it may cause local fault */
	if (!rx_reset_done_10g) {
		xpcs_rx_reset_10g(priv);
		rx_reset_done_10g = true;
	}

	ret = xpcs_poll_clear(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_STS,
			      DW_RX_ACK_0, 1000);
	if (ret)
		return ret;

	return ret;
}

static int xpcs_get_state_10g(struct dw_eth_dev *priv, int *link)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;
	int ret, i;

	/* Check link first */
	*link = 0;
	for (i = 0; i < 1000; i++) {
		value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
		if (value & DW_RX_LINK_UP) {
			*link = 1;
			break;
		}
		udelay(1);
	}
	if (*link == 0) {
		xpcs_rx_reset_10g(priv);
		for (i = 0; i < 1000; i++) {
			value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
			if (value & DW_RX_LINK_UP) {
				*link = 1;
				break;
			}
			udelay(10);
		}
	}
	if (loopback_test != NO_LOOPBACK) {
		if (*link == 0) {
			*link = 1;
			printf("Fake link up in loopback\n");
		}
		return 0; // Not check link fault for loopback
	}

	/* Then check the faults */
	ret = xpcs_read_fault_10g(priv->dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_XPCS_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_10g(struct dw_eth_dev *priv, int speed)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;
	int ret;

	/* Tx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_TX_GENCTRL0);
	value |= DW_XPCS_TX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_TX_GENCTRL0);
	/* Rx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	value |= DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	udelay(1000);

	if (priv->interface == PHY_INTERFACE_MODE_SGMII) {
		/* Set PCS speed */
		value = readw(ioaddr + DW_XPCS_SR_PCS_CTRL2);
		value &= ~DW_XPCS_TYPE_SEL;
		/* SGMII selects 10GBaseX */
		value |= DW_XPCS_TYPE_10GBASE_X;
		writel(value, ioaddr + DW_XPCS_SR_PCS_CTRL2);

		/* Select mode */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		if (speed == SPEED_2500) {
			value |= DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		}
		else if (speed == SPEED_1000) {
			value &= ~DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		}
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
	}

	/* Configure MII speed */
	value = readw(ioaddr + DW_XPCS_SR_MII_CTRL);
	value &= ~DW_USXGMII_SS;
	if (priv->interface == PHY_INTERFACE_MODE_SGMII)
		value |= DW_SGMII_1000;
	else if (speed == SPEED_5000)
		value |= DW_USXGMII_5000 | DW_USXGMII_FULL;
	else if (speed == SPEED_2500)
		value |= DW_USXGMII_2500 | DW_USXGMII_FULL;
	else if (speed == SPEED_1000)
		value |= DW_USXGMII_1000 | DW_USXGMII_FULL;
	else
		value |= DW_USXGMII_10000 | DW_USXGMII_FULL;
	writel(value, ioaddr + DW_XPCS_SR_MII_CTRL);
	udelay(1);	// Wait for 1us for XGMII clock to stablize

	if (priv->interface == PHY_INTERFACE_MODE_USXGMII) {
		/* USXGMII rate adaptor reset */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		value |= DW_USRA_RST;
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
	}

	/* Tx reset clear */
	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_TX_GENCTRL0);
	value &= ~DW_XPCS_TX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_TX_GENCTRL0);
	/* Rx reset clear */
	value = readw(ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	value &= ~DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_MP_12G_16G_25G_RX_GENCTRL1);
	udelay(50000);

	if (priv->interface == PHY_INTERFACE_MODE_USXGMII) {
		ret =  xpcs_poll_clear(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1,
				       DW_USRA_RST, 1000);
		if (ret)
			printf("DW_USRA_RST not cleared\n");
	}

	return 0;
}

static int write_e32_cr_25g(phys_addr_t ioaddr, u16 *data_pair)
{
	u16 value;
	int ret;

	ret = xpcs_poll_clear(ioaddr+DW_25G_VR_PMA_25G_16G_CR_CTRL,
			      DW_START_BUSY, 1);
	if (ret)
		return ret;

	writel(*data_pair, ioaddr + DW_25G_VR_PMA_25G_16G_CR_ADDR);
	writel(*(data_pair+1), ioaddr + DW_25G_VR_PMA_25G_16G_CR_DATA);
	value = readw(ioaddr + DW_25G_VR_PMA_25G_16G_CR_CTRL);
	value |= (DW_WR_RDN | DW_START_BUSY);
	writel(value, ioaddr + DW_25G_VR_PMA_25G_16G_CR_CTRL);

	return 0;
}

static int xpcs_prog_e32_25g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 data_pair[2];
	u16 value;
	int array_size;
	int i;

	/* Program 4-lane configuration */
	data_pair[0] = 0x6054;
	data_pair[1] = 0x280C;
	write_e32_cr_25g(ioaddr, data_pair);
	data_pair[0] = 0x6052;
	data_pair[1] = 0x2528;
	write_e32_cr_25g(ioaddr, data_pair);

	array_size = sizeof(image_1g_2g5_10g_25g) /
		     sizeof(image_1g_2g5_10g_25g[0]);
	for (i = 0; i < array_size; i++)
		write_e32_cr_25g(ioaddr, image_1g_2g5_10g_25g[i]);

	/* Programming SRAM done */
	value = readw(ioaddr + DW_25G_VR_PMA_25G_16G_SRAM);
	value |= DW_EXT_LD_DN;
	writel(value, ioaddr + DW_25G_VR_PMA_25G_16G_SRAM);

	return xpcs_poll_clear(ioaddr+DW_25G_SR_PCS_CTRL1, DW_CTRL1_RST, 1000);
}

static int xpcs_probe_25g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u32 xpcs_id;
	u16 value;
	int ret;

	if (priv->phy_lane == 0) {
		ret = xpcs_prog_e32_25g(priv);
		if (ret)
			return ret;
	}

	if (priv->phy_lane == 0) {
		master_lane_ioaddr = ioaddr;
		master_is_xpcs = false;
	}

	xpcs_id = xpcs_get_id(ioaddr + DW_25G_SR_PCS_DEV_ID1,
			      ioaddr + DW_25G_SR_PCS_DEV_ID2);

	if (xpcs_id != DW_XLGPCS_25G_ID)
		return -ENODEV;

	if (priv->max_speed == SPEED_10000) {
		/* Powerup bypass */
		value = readw(ioaddr + DW_25G_VR_PCS_DIG_CTRL1);
		value = DW_BYP_PWRUP;
		writel(value, ioaddr + DW_25G_VR_PCS_DIG_CTRL1);

		/* Enable four lane PMD and disable FEC */
		/* Disable FEC */
		value = readw(ioaddr + DW_25G_SR_PMA_RS_FEC_CTRL);
		//value |= DW_FLP_25G_X_2;
		value &= ~DW_RSFEC_EN;
		writel(value, ioaddr + DW_25G_SR_PMA_RS_FEC_CTRL);

		/* Set speed to 10G */
		value = readw(ioaddr + DW_25G_SR_PCS_CTRL1);
		value &= ~DW_XLGPCS_SS_5_2;
		value |= DW_XLGPCS_SS_10G;
		writel(value, ioaddr + DW_25G_SR_PCS_CTRL1);
		value = readw(ioaddr + DW_25G_SR_PCS_CTRL2);
		value &= ~DW_XLGPCS_TYPE_SEL;
		value |= DW_XLGPCS_TYPE_10GBASE_R;
		writel(value, ioaddr + DW_25G_SR_PCS_CTRL2);
		value = readw(ioaddr + DW_25G_SR_PMA_CTRL2);
		value &= ~DW_XLGPCS_PMA_TYPE;
		value |= DW_XLGPCS_PMA_TYPE_10GBASE_KR;
		writel(value, ioaddr + DW_25G_SR_PMA_CTRL2);

		/* Disable consortium */
		value = readw(ioaddr + DW_25G_VR_PCS_DIG_CTRL3);
		value &= ~DW_CNS_50G;
		value &= ~DW_CNS_EN;
		writel(value, ioaddr + DW_25G_VR_PCS_DIG_CTRL3);

		/* Vendor specific soft reset */
		value = readw(ioaddr + DW_25G_VR_PCS_DIG_CTRL1);
		value |= DW_VR_RST;
		writel(value, ioaddr + DW_25G_VR_PCS_DIG_CTRL1);
		ret = xpcs_poll_clear(ioaddr + DW_25G_VR_PCS_DIG_CTRL1, DW_VR_RST,
				      1000);
		if (ret)
			return ret;
	}
	else {
		/* For 25G, use default */
	}

	return 0;
}

static int xpcs_rx_reset_25g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;

	/* Rx reset */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value |= DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	udelay(50000);

	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value &= ~DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	udelay(1000);

	return 0;
}

static int xpcs_config_25g(struct dw_eth_dev *priv)
{
	static bool rx_reset_done_25g = false;

	/* Do Rx reset only once. Otherwise it may cause local fault */
	if (!rx_reset_done_25g) {
		xpcs_rx_reset_25g(priv);
		rx_reset_done_25g = true;
	}

	return 0;
}

static int xpcs_get_state_25g(struct dw_eth_dev *priv, int *link)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;
	int ret, i;

	/* Check link first */
	*link = 0;
	for (i = 0; i < 1000; i++) {
		value = readw(ioaddr + DW_25G_SR_PCS_STS1);
		if (value & DW_RX_LINK_UP) {
			*link = 1;
			break;
		}
		udelay(1);
	}
	if (*link == 0) {
		xpcs_rx_reset_25g(priv);
		for (i = 0; i < 1000; i++) {
			value = readw(ioaddr + DW_25G_SR_PCS_STS1);
			if (value & DW_RX_LINK_UP) {
				*link = 1;
				break;
			}
			udelay(10);
		}
	}
	if (loopback_test != NO_LOOPBACK) {
		if (*link == 0) {
			*link = 1;
			printf("Fake link up in loopback\n");
		}
		return 0; // Not check link fault for loopback
	}

	/* Then check the faults */
	ret = xpcs_read_fault_25g(priv->dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_25G_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_25g(struct dw_eth_dev *priv, int speed)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;

	/* Tx reset */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_TX_GENCTRL0);
	value |= DW_XPCS_TX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_TX_GENCTRL0);
	/* Rx reset */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value |= DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	udelay(1000);

	/* Do nothing for now */

	/* Tx reset clear */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_TX_GENCTRL0);
	value &= ~DW_XPCS_TX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_TX_GENCTRL0);
	/* Rx reset clear */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value &= ~DW_XPCS_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	udelay(50000);

	return 0;
}

static int xpcs_probe_50g(struct dw_eth_dev *priv)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u32 xpcs_id;
	u32 value;
	int ret;

	xpcs_id = xpcs_get_id(ioaddr + DW_50G_SR_PCS_DEV_ID1,
			      ioaddr + DW_50G_SR_PCS_DEV_ID2);
	if (xpcs_id != DW_XLGPCS_50G_ID)
		return -ENODEV;

	/* Set PCS type to 50G or 25G or 10G */
	if (priv->max_speed == SPEED_50000)
		value = DW_XLGPCS_TYPE_50GBASE_R;
	else if (priv->max_speed == SPEED_25000)
		value = DW_XLGPCS_TYPE_25GBASE_R;
	else
		value = DW_XLGPCS_TYPE_10GBASE_R;
	writel(value, ioaddr + DW_50G_SR_PCS_CTRL2);

	/* Set PMA type to 50G or 25G or 10G */
	value = readw(ioaddr + DW_50G_SR_PMA_CTRL2);
	value &= ~DW_XLGPCS_PMA_TYPE;
	if (priv->max_speed == SPEED_50000)
		value |= DW_XLGPCS_PMA_TYPE_50GBASE_KR;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_XLGPCS_PMA_TYPE_25GBASE_KR;
	else
		value |= DW_XLGPCS_PMA_TYPE_10GBASE_KR;
	writel(value, ioaddr + DW_50G_SR_PMA_CTRL2);

	value = readw(ioaddr + DW_50G_SR_PCS_CTRL1);
	value &= ~DW_XLGPCS_SS_5_2;
	if (priv->max_speed == SPEED_50000)
		value |= DW_XLGPCS_SS_50G;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_XLGPCS_SS_25G;
	else
		value |= DW_XLGPCS_SS_10G;
	writel(value, ioaddr + DW_50G_SR_PCS_CTRL1);

	if (priv->max_speed == SPEED_50000) {
		value = DW_PCS_AM_CNT_50G;
		writel(value, ioaddr + DW_50G_VR_PCS_AM_CNT);
		value = DW_AM_CNTX40_MOD66_50G;
		writel(value, ioaddr + DW_50G_SR_PMA_CTRL3);
		value = DW_AM_CNTX40_DIV66_50G;
		writel(value, ioaddr + DW_50G_SR_PMA_CTRL4);
	}

	/* Enable RS_FEC */
	value = readw(ioaddr + DW_50G_SR_PMA_RS_FEC_CTRL);
	if (priv->max_speed == SPEED_50000) {
		value &= ~DW_FLP_25G_X_2; // Select 50G_X_1
		value |= DW_RSFEC_EN;
	}
	else if (priv->max_speed == SPEED_25000) {
		value |= DW_FLP_25G_X_2; // Select 25G_X_2
		value &= ~DW_RSFEC_EN;
	}
	else {
		value &= ~DW_FLP_25G_X_2;
		value &= ~DW_RSFEC_EN;
	}
	writel(value, ioaddr + DW_50G_SR_PMA_RS_FEC_CTRL);

	/* Enable PMA layer bit-multiplexing */
	value = readw(ioaddr + DW_50G_VR_PCS_DIG_CTRL3);
	if (priv->max_speed == SPEED_50000) {
		value |= DW_BMX_544;
	}
	else if (priv->max_speed == SPEED_25000) {
		value &= ~DW_BMX_544;
	}
	else {
		value &= ~DW_BMX_544;
		value |= DW_MSK_PHY_RST;
	}
	writel(value, ioaddr + DW_50G_VR_PCS_DIG_CTRL3);

	value = readw(ioaddr +	DW_50G_SR_PMA_CTRL1);
	value &= ~DW_PMA_SS_5_2;
	if (priv->max_speed == SPEED_50000)
		value |= DW_PMA_SS_50G;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_PMA_SS_25G;
	else
		value |= DW_PMA_SS_10G;
	writel(value, ioaddr + DW_50G_SR_PMA_CTRL1);

	/* Vendor specific soft reset */
	value = readw(ioaddr + DW_50G_VR_PCS_DIG_CTRL1);
	value |= DW_VR_RST;
	value |= (DW_TX_INIT | DW_RX_INIT);
	writel(value, ioaddr + DW_50G_VR_PCS_DIG_CTRL1);

	ret = xpcs_poll_clear(ioaddr + DW_50G_VR_PCS_DIG_CTRL1, DW_VR_RST,
			      1000);
	if (ret)
		return ret;

	return 0;
}

static int xpcs_config_50g(struct dw_eth_dev *priv)
{
	/* Nothing to do */
	return 0;
}

static int xpcs_get_state_50g(struct dw_eth_dev *priv, int *link)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 value;
	int ret, i;

	/* Check link first */
	*link = 0;
	for (i = 0; i < 1000; i++) {
		value = readw(ioaddr + DW_50G_SR_PCS_STS1);
		if (value & DW_RX_LINK_UP) {
			*link = 1;
			break;
		}
		udelay(1);
	}
	if (loopback_test != NO_LOOPBACK) {
		if (*link == 0) {
			*link = 1;
			printf("Fake link up in loopback\n");
		}
		return 0; // Not check link fault for loopback
	}

	/* Then check the faults */
	ret = xpcs_read_fault_50g(priv->dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_50G_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_50g(struct dw_eth_dev *priv, int speed)
{
	/* Do nothing for now */
	return 0;
}

static struct xpcs_ops xpcs_ops_10g = {
	.config = xpcs_config_10g,
	.get_state = xpcs_get_state_10g,
	.probe = xpcs_probe_10g,
	.update_speed = xpcs_update_speed_10g,
};

static struct xpcs_ops xpcs_ops_25g = {
	.config = xpcs_config_25g,
	.get_state = xpcs_get_state_25g,
	.probe = xpcs_probe_25g,
	.update_speed = xpcs_update_speed_25g,
};

static struct xpcs_ops xpcs_ops_50g = {
	.config = xpcs_config_50g,
	.get_state = xpcs_get_state_50g,
	.probe = xpcs_probe_50g,
	.update_speed = xpcs_update_speed_50g,
};

struct xpcs_ops *xpcs_get_ops(struct dw_eth_dev *priv)
{
	struct xpcs_ops *ops;

	switch (priv->eth_link) {
	case 1:
		ops = &xpcs_ops_50g;
		break;
	case 2:
		if (priv->phy_lane == 0)
			ops = &xpcs_ops_10g;
		else
			ops = &xpcs_ops_25g;
		break;
	default:
		ops = NULL;
		break;
	}

	return ops;
}

int xgbe_e56_serdes_probe(struct dw_eth_dev *priv)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;
	int ret;

	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value |= DW_E56_PMD_SOFT_RESET;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	ret = xpcs_poll_clear(e56_addr + DW_E56_PMD_CFG0,
			      DW_E56_PMD_SOFT_RESET, 1000);
	if (ret)
		return ret;

	value = readl(e56_addr + DW_E56_PMD_CFG0);
	if ((value & DW_E56_PMD_CFG0_MASK) != DW_E56_PMD_CFG0_DEFAULT)
		return -ENODEV;

	/* E56 Tx config should be done after PHY config */
	xgbe_e56_serdes_tx_config(priv);

	return 0;
}

static int e56_serdes_tx_config_cms_10g(struct dw_eth_dev *priv)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;

	/* CMS */
	value = readl(e56_addr + DW_E56_PLL0_DIV_CFG0);
	value &= ~(DW_E56_PLL0_PREDIV | DW_E56_PLL0_FBDIV_INT);
	value |= (DW_E56_PLL0_PREDIV_8 | DW_E56_PLL0_FBDIV_INT_294);
	value |= DW_E56_PLL0_DIV_SPARE;
	writel(value, e56_addr + DW_E56_PLL0_DIV_CFG0);

	value = readl(e56_addr + DW_E56_PLL0_CFG0);
	value |= (DW_E56_CLK_TX_TL_EN | DW_E56_CLK_TX_TR_EN);
	writel(value, e56_addr + DW_E56_PLL0_CFG0);

	value = readl(e56_addr + DW_E56_PLL1_CFG0);
	value |= (DW_E56_CLK_TX_TL_EN | DW_E56_CLK_TX_TR_EN);
	writel(value, e56_addr + DW_E56_PLL1_CFG0);

	value = readl(e56_addr + DW_E56_PLL1_CFG2);
	value &= ~DW_E56_PLL1_POSTDIV;
	if (priv->max_speed == SPEED_10000)
		value |= DW_E56_PLL1_POSTDIV_8;
	else
		value |= DW_E56_PLL1_POSTDIV_A;
	writel(value, e56_addr + DW_E56_PLL1_CFG2);

	value = readl(e56_addr + DW_E56_PLL1_DIV_CFG0);
	value &= ~(DW_E56_PLL1_PREDIV | DW_E56_PLL1_FBDIV_INT);
	value |= (DW_E56_PLL1_PREDIV_8 | DW_E56_PLL1_FBDIV_INT_108);
	writel(value, e56_addr + DW_E56_PLL1_DIV_CFG0);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
	value |= DW_E56_OVRD_EN_ANA_LCPLL_LF_VCO_CTRL;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL7);
	value &= ~DW_E56_ANA_LCPLL_LF_VCO_CTRL;
	value |= DW_E56_ANA_LCPLL_LF_VCO_CTRL_F;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL7);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
	value |= DW_E56_OVRD_EN_ANA_LCPLL_LF_TST;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL9);
	value &= ~DW_E56_ANA_LCPLL_LF_TEST;
	value |= DW_E56_ANA_LCPLL_LF_TEST_VAL;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL9);

	return 0;
}

static int e56_serdes_tx_config_cms_25g_50g(struct dw_eth_dev *priv)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;

	/* CMS */
	value = readl(e56_addr + DW_E56_PLL0_CFG0);
	value |= (DW_E56_CLK_TX_TL_EN | DW_E56_CLK_TX_TR_EN);
	writel(value, e56_addr + DW_E56_PLL0_CFG0);

	value = readl(e56_addr + DW_E56_PLL0_CFG2);
	value &= ~DW_E56_PLL0_POSTDIV;
	value |= DW_E56_PLL0_POSTDIV_4;
	writel(value, e56_addr + DW_E56_PLL0_CFG2);

	value = readl(e56_addr + DW_E56_PLL0_DIV_CFG0);
	value &= ~(DW_E56_PLL0_PREDIV | DW_E56_PLL0_FBDIV_INT);
	value |= DW_E56_PLL0_PREDIV_8;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_PLL0_FBDIV_INT_2A8;
	else
		value |= DW_E56_PLL0_FBDIV_INT_294;
	writel(value, e56_addr + DW_E56_PLL0_DIV_CFG0);

	value = readl(e56_addr + DW_E56_PIN_OVRDVAL0);
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_INT_PLL0_TX_SIG_TYPE;
	else
		value &= ~DW_E56_INT_PLL0_TX_SIG_TYPE;
	writel(value, e56_addr + DW_E56_PIN_OVRDVAL0);

	value = readl(e56_addr + DW_E56_PIN_OVRDEN0);
	value |= DW_E56_OVRD_EN_PLL0_TX_SIG_TYPE;
	writel(value, e56_addr + DW_E56_PIN_OVRDEN0);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL2);
	value &= ~DW_E56_ANA_LCPLL_HF_VCO;
	value |= DW_E56_ANA_LCPLL_HF_VCO_F;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL2);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_LCPLL_HF_VCO;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL4);
	value &= ~DW_E56_ANA_LCPLL_HF_TEST;
	value |= DW_E56_ANA_LCPLL_HF_TEST_VAL;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL4);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
	value |= DW_E56_OVRD_EN_ANA_LCPLL_HF_TST;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	return 0;
}

#define USE_EXT_CLOCK	1
#define LINK1_ALT_REF_CLK_MASK	GENMASK(3, 0)
#define LINK1_ALT_REF_CLK_VAL	0x0
#define PVT_MON_TEMP_CODE_40C	323
#define PVT_MON_TEMP_CODE_70C	462
int xgbe_e56_serdes_tx_config(struct dw_eth_dev *priv)
{
	phys_addr_t e56_addr = priv->e56_base;
	phys_addr_t misc_addr = priv->misc_base;
	u32 value;
	u32 temp_code;

	// char_3
	/* Ref clock enable */
#if (USE_EXT_CLOCK == 1)
	value = readl(e56_addr + DW_E56_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_REFCLK;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL0);
	value |= DW_E56_ANA_REFCLK_BUF_PAD_EN;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL0);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
	value |= DW_E56_OVRD_EN_ANA_TEST_SLICER;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL5);
	value |= DW_E56_ANA_TEST_SLICER;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL5);

	value = readl(e56_addr + DW_E56_PLL0_CFG0);
	value &= ~DW_E56_PLL0_REFCLK_SEL;
	value |= DW_E56_PLL0_REFCLK_FROM_PAD;
	writel(value, e56_addr + DW_E56_PLL0_CFG0);

	value = readl(e56_addr + DW_E56_PLL1_CFG0);
	value &= ~DW_E56_PLL1_REFCLK_SEL;
	value |= DW_E56_PLL1_REFCLK_FROM_PAD;
	writel(value, e56_addr + DW_E56_PLL1_CFG0);

	/* Switch off the IOSS PLL0 Clock to Link1 */
	udelay(10000);
	value = readl(IOSS_CCIX_CFG_BASE + 0xB0);
	value &= ~LINK1_ALT_REF_CLK_MASK;
	value |= LINK1_ALT_REF_CLK_VAL;
	writel(value, IOSS_CCIX_CFG_BASE + 0xB0);

	udelay(100);
	/* Switch the APB clock mux - its safe to switch now as the other clock is off */
	value = readl(misc_addr + XGBE_E56_CFG1);
	value |= XGBE_E56_CLK_SEL; // 1: from ext pll
	writel(value, misc_addr + XGBE_E56_CFG1);


#else
	value = readl(e56_addr + DW_E56_ANA_OVRDEN0);
	value &= ~DW_E56_OVRD_EN_ANA_REFCLK;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL0);
	value &= ~DW_E56_ANA_REFCLK_BUF_PAD_EN;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL0);

	value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
	value &= ~DW_E56_OVRD_EN_ANA_TEST_SLICER;
	writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL5);
	value &= ~DW_E56_ANA_TEST_SLICER;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL5);

	value = readl(e56_addr + DW_E56_PLL0_CFG0);
	value &= ~DW_E56_PLL0_REFCLK_SEL;
	value |= DW_E56_PLL0_REFCLK_FROM_INT;
	writel(value, e56_addr + DW_E56_PLL0_CFG0);

	value = readl(e56_addr + DW_E56_PLL1_CFG0);
	value &= ~DW_E56_PLL1_REFCLK_SEL;
	value |= DW_E56_PLL1_REFCLK_FROM_INT;
	writel(value, e56_addr + DW_E56_PLL1_CFG0);

	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value |= DW_E56_PLL_REF_CLK_INT;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
#endif

	// char_4
	/* CMS */
	if (priv->max_speed == SPEED_10000)
		e56_serdes_tx_config_cms_10g(priv);
	else
		e56_serdes_tx_config_cms_25g_50g(priv);

	/* TXS */
	value = readl(e56_addr + DW_E56_TXS_CFG1);
	value &= ~DW_E56_ADAPTATION_WAIT_CNT;
	value |= DW_E56_ADAPTATION_WAIT_CNT_F;
	writel(value, e56_addr + DW_E56_TXS_CFG1);

	value = readl(e56_addr + DW_E56_TXS_WKUP_CNT);
	value &= ~(DW_E56_LDO_WKUP_CNT | DW_E56_DCC_WKUP_CNT);
	value |= (DW_E56_LDO_WKUP_CNT_FF | DW_E56_DCC_WKUP_CNT_FF);
	writel(value, e56_addr + DW_E56_TXS_WKUP_CNT);

	value = readl(e56_addr + DW_E56_TXS_PIN_OVRDVAL6);
	if (priv->max_speed == SPEED_50000) {
		value &= ~DW_E56_INT_TX0_EFUSE_BITS_15_12;
		value |= DW_E56_INT_TX0_EFUSE_PAM4;
	}
	else if (priv->max_speed == SPEED_25000) {
		value &= ~DW_E56_INT_TX0_EFUSE_BITS_11_8;
		value |= DW_E56_INT_TX0_EFUSE_NRZ;
	}
	else {
		value &= ~DW_E56_INT_TX0_EFUSE_BITS_3_0;
		value |= DW_E56_INT_TX0_EFUSE_10G;
	}
	writel(value, e56_addr + DW_E56_TXS_PIN_OVRDVAL6);

	value = readl(e56_addr + DW_E56_TXS_PIN_OVRDEN0);
	value |= DW_E56_OVRD_EN_TX0_EFUSE_BITS;
	writel(value, e56_addr + DW_E56_TXS_PIN_OVRDEN0);

	value = readl(e56_addr + DW_E56_TXS_ANA_OVRDVAL1);
	value &= ~DW_E56_ANA_TEST_DAC;
	value |= DW_E56_ANA_TEST_DAC_1;
	writel(value, e56_addr + DW_E56_TXS_ANA_OVRDVAL1);

	value = readl(e56_addr + DW_E56_TXS_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_TEST_DAC;
	writel(value, e56_addr + DW_E56_TXS_ANA_OVRDEN0);


	/* RXS */
	value = readl(e56_addr + DW_E56_RXS_CFG0);
	value &= ~DW_E56_DSER_DATA_SEL;
	writel(value, e56_addr + DW_E56_RXS_CFG0);

	value = readl(e56_addr + DW_E56_RXS_CFG0);
	value &= ~DW_E56_TRAIN_CLK_GATE_BYPASS;
	value |= DW_E56_TRAIN_CLK_GATE_BYP_1FFF;
	writel(value, e56_addr + DW_E56_RXS_CFG0);

	if (priv->max_speed == SPEED_50000) {
		value = (DW_E56_PREDIV2_700 | DW_E56_TARGET_CNT2_2530);
		writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR2);
	}
	else if (priv->max_speed == SPEED_25000) {
		value = (DW_E56_PREDIV1_700 | DW_E56_TARGET_CNT1_2418);
		writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR1);
	}
	else {
		value = (DW_E56_PREDIV0_FA0 | DW_E56_TARGET_CNT0_203A);
		writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR0);
	}

	value = readl(e56_addr + DW_E56_OSC_CAL_N_CDR4);
	if (priv->max_speed == SPEED_50000) {
		value &= ~(DW_E56_OSC_RANGE_SEL2 | DW_E56_VCO_CODE_INIT);
		value |= (DW_E56_OSC_SEL2_26_DOT_56G | DW_E56_VCO_CODE_7FB);
		value &= ~(DW_E56_OSC_CURRENT_BOOST_EN2 | DW_E56_BBCDR_CURRENT_BOOST2);
	}
	else if (priv->max_speed == SPEED_25000) {
		value &= ~(DW_E56_OSC_RANGE_SEL0 | DW_E56_OSC_RANGE_SEL1);
		value |= (DW_E56_OSC_SEL0_25_DOT_78G | DW_E56_OSC_SEL1_25_DOT_78G);
		value &= ~(DW_E56_OSC_RANGE_SEL2 | DW_E56_VCO_CODE_INIT);
		value |= (DW_E56_OSC_SEL2_25_DOT_78G | DW_E56_VCO_CODE_7FB);
		value &= ~(DW_E56_OSC_CURRENT_BOOST_EN1 | DW_E56_BBCDR_CURRENT_BOOST1);
	}
	else {
		value &= ~(DW_E56_OSC_RANGE_SEL0 | DW_E56_VCO_CODE_INIT);
		value |= (DW_E56_OSC_SEL0_20G | DW_E56_VCO_CODE_7FF);
		value |= DW_E56_OSC_CURRENT_BOOST_EN0;
		value &= ~DW_E56_BBCDR_CURRENT_BOOST0;
	}
	writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR4);

	value = readl(e56_addr + DW_E56_OSC_CAL_N_CDR5);
	value &= ~(DW_E56_SDM_WIDTH | DW_E56_BBCDR_PROP_STEP_PRELOCK |
		   DW_E56_BBCDR_PROP_STEP_POSTLOCK | DW_E56_BBCDR_GAIN_CTRL_POSTLOCK |
		   DW_E56_BBCDR_GAIN_CTRL_PRELOCK | DW_E56_BBCDR_RDY_CNT );
	if (priv->max_speed == SPEED_50000)
		value |= (DW_E56_SDM_WIDTH_5BITS | DW_E56_CDR_PROP_PRELOCK_450MHZ |
			  DW_E56_CDR_PROP_POSTLOCK_90MHZ | DW_E56_CDR_GAIN_POSTLOCK_13BIT |
			  DW_E56_CDR_GAIN_PRELOCK_8BIT | DW_E56_CDR_RDY_CNT_64K );
	else if (priv->max_speed == SPEED_25000)
		value |= (DW_E56_SDM_WIDTH_5BITS | DW_E56_CDR_PROP_PRELOCK_450MHZ |
			  DW_E56_CDR_PROP_POSTLOCK_90MHZ | DW_E56_CDR_GAIN_POSTLOCK_10BIT |
			  DW_E56_CDR_GAIN_PRELOCK_7BIT | DW_E56_CDR_RDY_CNT_64K);
	else
		value |= (DW_E56_SDM_WIDTH_5BITS | DW_E56_CDR_PROP_PRELOCK_450MHZ |
			  DW_E56_CDR_PROP_POSTLOCK_MAX | DW_E56_CDR_GAIN_POSTLOCK_7BIT |
			  DW_E56_CDR_GAIN_PRELOCK_7BIT | DW_E56_CDR_RDY_CNT_64K);
	writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR5);

	value = readl(e56_addr + DW_E56_OSC_CAL_N_CDR6);
	value &= ~(DW_E56_PI_GAIN_CTRL_PRELOCK | DW_E56_PI_GAIN_CTRL_POSTLOCK);
	value |= DW_E56_PI_GAIN_PRELOCK_6BIT;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_PI_GAIN_POSTLOCK_12BIT;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_E56_PI_GAIN_POSTLOCK_11BIT;
	else
		value |= DW_E56_PI_GAIN_POSTLOCK_8BIT;
	writel(value, e56_addr + DW_E56_OSC_CAL_N_CDR6);


	if (priv->max_speed == SPEED_50000) {
		value = readl(e56_addr + DW_E56_INTL_CONFIG1);
		value &= ~DW_E56_ADC_INTL2SLICE_DELAY2;
		value |= DW_E56_ADC_DELAY2_DDDD;
		writel(value, e56_addr + DW_E56_INTL_CONFIG1);
	}
	else if (priv->max_speed == SPEED_25000) {
		value = readl(e56_addr + DW_E56_INTL_CONFIG0);
		value &= ~DW_E56_ADC_INTL2SLICE_DELAY1;
		value |= DW_E56_ADC_DELAY1_3333;
		writel(value, e56_addr + DW_E56_INTL_CONFIG0);
	}
	else {
		value = readl(e56_addr + DW_E56_INTL_CONFIG0);
		value &= ~DW_E56_ADC_INTL2SLICE_DELAY0;
		value |= DW_E56_ADC_DELAY0_5555;
		writel(value, e56_addr + DW_E56_INTL_CONFIG0);
	}

	value = readl(e56_addr + DW_E56_INTL_CONFIG2);
	if (priv->max_speed == SPEED_50000)
		value &= ~DW_E56_INTL_HBW_DISABLED2;
	else if (priv->max_speed == SPEED_25000)
		value &= ~DW_E56_INTL_HBW_DISABLED1;
	else
		value |= DW_E56_INTL_HBW_DISABLED0;
	writel(value, e56_addr + DW_E56_INTL_CONFIG2);

	value = readl(e56_addr + DW_E56_TXFFE_TRAINING0);
	value &= ~(DW_E56_ADC_DATA_PEAK_LTH | DW_E56_ADC_DATA_PEAK_UTH);
	if (priv->max_speed == SPEED_50000)
		value |= (DW_E56_ADC_DATA_PEAK_LTH_56 | DW_E56_ADC_DATA_PEAK_UTH_70);
	else
		value |= (DW_E56_ADC_DATA_PEAK_LTH_56 | DW_E56_ADC_DATA_PEAK_UTH_6A);
	writel(value, e56_addr + DW_E56_TXFFE_TRAINING0);

	value = readl(e56_addr + DW_E56_TXFFE_TRAINING1);
	value &= ~(DW_E56_C1_LTH | DW_E56_C1_UTH);
	if (priv->max_speed == SPEED_10000)
		value |= (DW_E56_C1_LTH_1E8 | DW_E56_C1_UTH_78);
	else
		value |= (DW_E56_C1_LTH_1F8 | DW_E56_C1_UTH_F0);
	writel(value, e56_addr + DW_E56_TXFFE_TRAINING1);

	value = readl(e56_addr + DW_E56_TXFFE_TRAINING2);
	value &= ~(DW_E56_CM1_LTH | DW_E56_CM1_UTH);
	value |= (DW_E56_CM1_LTH_100 | DW_E56_CM1_UTH_FF);
	writel(value, e56_addr + DW_E56_TXFFE_TRAINING2);

	value = readl(e56_addr + DW_E56_TXFFE_TRAINING3);
	value &= ~(DW_E56_CM2_LTH | DW_E56_CM2_UTH | DW_E56_TXFFE_TRAIN_MODE_TYPE);
	value |= (DW_E56_CM2_LTH_4 | DW_E56_CM2_UTH_37 | DW_E56_TRAIN_MODE_TYPE_38);
	writel(value, e56_addr + DW_E56_TXFFE_TRAINING3);

	value = readl(e56_addr + DW_E56_VGA_TRAINING0);
	value &= ~DW_E56_VGA_TARGET;
	value |= DW_E56_VGA_TARGET_34;
	writel(value, e56_addr + DW_E56_VGA_TRAINING0);

	value = readl(e56_addr + DW_E56_VGA_TRAINING1);
	value &= ~(DW_E56_VGA1_CODE_INIT0 | DW_E56_VGA1_CODE_INIT123 |
		   DW_E56_VGA2_CODE_INIT0 | DW_E56_VGA2_CODE_INIT123);
	value |= (DW_E56_VGA1_CODE_INIT0_A | DW_E56_VGA1_CODE_INIT123_A |
		  DW_E56_VGA2_CODE_INIT0_A | DW_E56_VGA2_CODE_INIT123_A);
	writel(value, e56_addr + DW_E56_VGA_TRAINING1);

	value = readl(e56_addr + DW_E56_CTLE_TRAINING0);
	value &= ~(DW_E56_CTLE_CODE_INIT0 | DW_E56_CTLE_CODE_INIT123);
	value |= (DW_E56_CTLE_CODE_INIT0_9 | DW_E56_CTLE_CODE_INIT123_9);
	writel(value, e56_addr + DW_E56_CTLE_TRAINING0);

	if (priv->max_speed == SPEED_50000)
		value = DW_E56_CTLE_LFEQ_LUT_VAL;
	else
		value = DW_E56_CTLE_LFEQ_LUT_VAL2;
	writel(value, e56_addr + DW_E56_CTLE_TRAINING1);

	value = readl(e56_addr + DW_E56_CTLE_TRAINING2);
	value &= ~(DW_E56_ISI_TH_FRAC_P1 | DW_E56_ISI_TH_FRAC_P2 |
		   DW_E56_ISI_TH_FRAC_P3);
	value |= DW_E56_ISI_TH_FRAC_P1_18;
	writel(value, e56_addr + DW_E56_CTLE_TRAINING2);

	value = readl(e56_addr + DW_E56_CTLE_TRAINING3);
	value &= ~(DW_E56_TAP_WEIGHT_P1 | DW_E56_TAP_WEIGHT_P2 |
		   DW_E56_TAP_WEIGHT_P3);
	value |= DW_E56_TAP_WEIGHT_P1_1;
	writel(value, e56_addr + DW_E56_CTLE_TRAINING3);

	value = readl(e56_addr + DW_E56_OFFSET_N_GAIN_CAL0);
	value &= ~(DW_E56_ADC_SLICE_DATA_AVG_CNT | DW_E56_ADC_DATA_AVG_CNT |
		   DW_E56_FE_OFFSET_DAC_CLK_CNT);
	value |= (DW_E56_ADC_SLICE_DATA_AVG_16K | DW_E56_ADC_DATA_AVG_16K |
		  DW_E56_FE_OFFSET_DAC_CLK_CNT_C);
	writel(value, e56_addr + DW_E56_OFFSET_N_GAIN_CAL0);

	value = readl(e56_addr + DW_E56_OFFSET_N_GAIN_CAL1);
	value &= ~DW_E56_SAMP_ADAPT_CFG;
	value |= DW_E56_SAMP_ADAPT_CFG_5;
	writel(value, e56_addr + DW_E56_OFFSET_N_GAIN_CAL1);

	value = readl(e56_addr + DW_E56_FFE_TRAINING0);
	value &= ~DW_E56_FFE_TAP_EN;
	value |= DW_E56_FFE_TAP_EN_F9FF;
	writel(value, e56_addr + DW_E56_FFE_TRAINING0);

	value = readl(e56_addr + DW_E56_IDLE_DETECT1);
	value &= ~(DW_E56_IDLE_TH_ADC_PEAK_MAX | DW_E56_IDLE_TH_ADC_PEAK_MIN);
	value |= (DW_E56_IDLE_TH_ADC_PEAK_MAX_A | DW_E56_IDLE_TH_ADC_PEAK_MIN_5);
	writel(value, e56_addr + DW_E56_IDLE_DETECT1);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL0);
	value |= DW_E56_ANA_EN_RTERM;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL0);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_EN_RTERM;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL6);
	value &= ~DW_E56_ANA_TEST_BBCDR_BIT_2_0;
	if (priv->max_speed == SPEED_10000)
		value |= DW_E56_ANA_TEST_BBCDR_6;
	value &= ~DW_E56_ANA_TEST_BBCDR_BIT_3;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_ANA_TEST_BBCDR_BIT_4;
	else
		value &= ~DW_E56_ANA_TEST_BBCDR_BIT_4;
	value &= ~DW_E56_ANA_TEST_BBCDR_BIT_13;
	if (priv->max_speed == SPEED_10000)
		value |= DW_E56_ANA_TEST_BBCDR_BIT_14;
	else
		value &= ~DW_E56_ANA_TEST_BBCDR_BIT_14;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL6);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN1);
	value |= DW_E56_OVRD_EN_ANA_TEST_BBCRD;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN1);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL15);
	value &= ~DW_E56_ANA_ANABS_CONFIG;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_ANA_ANABS_CONFIG_7;
	else
		value |= DW_E56_ANA_ANABS_CONFIG_1;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL15);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN3);
	value |= DW_E56_OVRD_EN_ANA_ANABS_CONFIG;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN3);

	value = readl(e56_addr + DW_E56_ANA_OVRDVAL17);
	value &= ~DW_E56_ANA_VGA2_BOOST_CSTM;
	writel(value, e56_addr + DW_E56_ANA_OVRDVAL17);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN3);
	value |= DW_E56_OVRD_EN_ANA_VGA2_BOOST_CSTM;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN3);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL14);
	if (priv->max_speed == SPEED_10000)
		value |= DW_E56_ANA_SPARE_BIT13;
	else
		value &= ~DW_E56_ANA_SPARE_BIT13;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL14);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN4);
	value |= DW_E56_OVRD_EN_ANA_SPARE_BIT13;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN4);

	value = DW_E56_EYE_SCAN_TIMER_400;
	writel(value, e56_addr + DW_E56_EYE_SCAN_TIMER);

	value = readl(e56_addr + DW_E56_RXS_RINGO0);
	value &= ~DW_E56_RINGO0_SPARE0;
	value &= ~DW_E56_RINGO0_SPARE3;
	value |= DW_E56_RINGO0_SPARE1;
	value |= DW_E56_RINGO0_SPARE2;
	value &= ~DW_E56_RINGO0_SPARE4_TO_9;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_RINGO0_SPARE4_TO_9_38;
	else
		value |= DW_E56_RINGO0_SPARE4_TO_9_36;
	writel(value, e56_addr + DW_E56_RXS_RINGO0);

	/* PMD */
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_PMD_MODE;	// 0 for ETHML mode
	writel(value, e56_addr + DW_E56_PMD_CFG0);

	value = readl(e56_addr + DW_E56_PMD_CFG3);
	value &= ~DW_E56_CTRL_FSM_TIMEOUT;
	value |= DW_E56_CTRL_FSM_TIMEOUT_80;
	writel(value, e56_addr + DW_E56_PMD_CFG3);

	value = readl(e56_addr + DW_E56_PMD_CFG4);
	value &= ~(DW_E56_TRAIN_DC_ON_PERIOD | DW_E56_TRAIN_DC_PERIOD);
	value |= (DW_E56_TRAIN_DC_ON_PERIOD_18 | DW_E56_TRAIN_DC_PERIOD_3E);
	writel(value, e56_addr + DW_E56_PMD_CFG4);

	value = readl(e56_addr + DW_E56_PMD_CFG5);
	value |= DW_E56_USE_RECENT_MARKER_OFFSET;
	writel(value, e56_addr + DW_E56_PMD_CFG5);

	if (priv->max_speed == SPEED_50000)
		value = DW_E56_DATAPATH_CFG0_50G;
	else if (priv->max_speed == SPEED_25000)
		value = DW_E56_DATAPATH_CFG0_25G;
	else
		value = DW_E56_DATAPATH_CFG0_10G;
	writel(value, e56_addr + DW_E56_DATAPATH_CFG0);

	if (priv->max_speed == SPEED_10000)
		value = DW_E56_DATAPATH_CFG1_10G;
	else
		value = DW_E56_DATAPATH_CFG1_DEFAULT;
	writel(value, e56_addr + DW_E56_DATAPATH_CFG1);

	value = readl(e56_addr + DW_E56_AN_CFG1);
	value &= ~(DW_E56_AN0_RATE_SELECT_REG | DW_E56_AN1_RATE_SELECT_REG |
		   DW_E56_AN2_RATE_SELECT_REG | DW_E56_AN3_RATE_SELECT_REG);
	if (priv->max_speed == SPEED_50000)
		value |= (DW_E56_AN0_RATE_SELECT_REG_D | DW_E56_AN1_RATE_SELECT_REG_D |
			  DW_E56_AN2_RATE_SELECT_REG_D | DW_E56_AN3_RATE_SELECT_REG_D);
	else if (priv->max_speed == SPEED_25000)
		value |= (DW_E56_AN0_RATE_SELECT_REG_A | DW_E56_AN1_RATE_SELECT_REG_A |
			  DW_E56_AN2_RATE_SELECT_REG_A | DW_E56_AN3_RATE_SELECT_REG_A);
	else
		value |= (DW_E56_AN0_RATE_SELECT_REG_2 | DW_E56_AN1_RATE_SELECT_REG_2 |
			  DW_E56_AN2_RATE_SELECT_REG_2 | DW_E56_AN3_RATE_SELECT_REG_2);
	writel(value, e56_addr + DW_E56_AN_CFG1);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG0);
	value &= ~(DW_E56_DO_RX_ADC_OFST_CAL | DW_E56_RX_ERR_ACTION_EN);
	value |= (DW_E56_CONT_ON_ADC_GAIN_CAL_ERR | DW_E56_DO_RX_ADC_OFST_CAL_3);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG0);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG1);
	value &= ~(DW_E56_TRAIN_ST0_WAIT_CNT | DW_E56_TRAIN_ST1_WAIT_CNT |
		   DW_E56_TRAIN_ST2_WAIT_CNT | DW_E56_TRAIN_ST3_WAIT_CNT);
	value |= (DW_E56_TRAIN_ST0_WAIT_CNT_FF | DW_E56_TRAIN_ST1_WAIT_CNT_FF |
		  DW_E56_TRAIN_ST2_WAIT_CNT_FF | DW_E56_TRAIN_ST3_WAIT_CNT_FF);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG1);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG2);
	value &= ~(DW_E56_TRAIN_ST4_WAIT_CNT | DW_E56_TRAIN_ST5_WAIT_CNT |
		   DW_E56_TRAIN_ST6_WAIT_CNT | DW_E56_TRAIN_ST7_WAIT_CNT);
	value |= (DW_E56_TRAIN_ST4_WAIT_CNT_1 | DW_E56_TRAIN_ST5_WAIT_CNT_4 |
		  DW_E56_TRAIN_ST6_WAIT_CNT_4 | DW_E56_TRAIN_ST7_WAIT_CNT_4);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG2);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG3);
	value &= ~(DW_E56_TRAIN_ST8_WAIT_CNT | DW_E56_TRAIN_ST9_WAIT_CNT |
		   DW_E56_TRAIN_ST10_WAIT_CNT | DW_E56_TRAIN_ST11_WAIT_CNT);
	value |= (DW_E56_TRAIN_ST8_WAIT_CNT_4 | DW_E56_TRAIN_ST9_WAIT_CNT_4 |
		  DW_E56_TRAIN_ST10_WAIT_CNT_4 | DW_E56_TRAIN_ST11_WAIT_CNT_4);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG3);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG4);
	value &= ~(DW_E56_TRAIN_ST12_WAIT_CNT | DW_E56_TRAIN_ST13_WAIT_CNT |
		   DW_E56_TRAIN_ST14_WAIT_CNT | DW_E56_TRAIN_ST15_WAIT_CNT);
	value |= (DW_E56_TRAIN_ST12_WAIT_CNT_4 | DW_E56_TRAIN_ST13_WAIT_CNT_4 |
		  DW_E56_TRAIN_ST14_WAIT_CNT_4 | DW_E56_TRAIN_ST15_WAIT_CNT_4);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG4);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG7);
	value &= ~(DW_E56_TRAIN_ST4_EN | DW_E56_TRAIN_ST5_EN);
	value |= (DW_E56_TRAIN_ST4_EN_4BF | DW_E56_TRAIN_ST5_EN_C4BF);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG7);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG8);
	value &= ~DW_E56_TRAIN_ST7_EN;
	value |= DW_E56_TRAIN_ST7_EN_47FF;
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG8);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG12);
	value &= ~DW_E56_TRAIN_ST15_EN;
	value |= DW_E56_TRAIN_ST15_EN_67FF;
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG12);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG13);
	value &= ~(DW_E56_TRAIN_ST0_DONE_EN | DW_E56_TRAIN_ST1_DONE_EN);
	value |= (DW_E56_TRAIN_ST0_DONE_EN_8001 | DW_E56_TRAIN_ST1_DONE_EN_8002);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG13);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG14);
	value &= ~DW_E56_TRAIN_ST3_DONE_EN;
	value |= DW_E56_TRAIN_ST3_DONE_EN_8008;
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG14);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG15);
	value &= ~DW_E56_TRAIN_ST4_DONE_EN;
	value |= DW_E56_TRAIN_ST4_DONE_EN_8004;
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG15);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG29);
	value &= ~DW_E56_TRAIN_ST15_DC_EN;
	value |= DW_E56_TRAIN_ST15_DC_EN_3F6D;
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG29);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG33);
	value &= ~(DW_E56_TRAIN0_RATE_SEL | DW_E56_TRAIN1_RATE_SEL);
	value |= (DW_E56_TRAIN0_RATE_SEL_8000 | DW_E56_TRAIN1_RATE_SEL_8000);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG33);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CFG34);
	value &= ~(DW_E56_TRAIN2_RATE_SEL | DW_E56_TRAIN3_RATE_SEL);
	value |= (DW_E56_TRAIN2_RATE_SEL_8000 | DW_E56_TRAIN3_RATE_SEL_8000);
	writel(value, e56_addr + DW_E56_CTRL_FSM_CFG34);

	value = readl(e56_addr + DW_E56_KRT_TFSM_CFG);
	value &= ~(DW_E56_TFSM_MAX_WAIT_TIMER_X1M | DW_E56_TFSM_MAX_WAIT_TIMER_X8M |
		   DW_E56_TFSM_HOLDOFF_TIMER_X256K);
	value |= (DW_E56_MAX_WAIT_TIMER_X1M_49 | DW_E56_MAX_WAIT_TIMER_X8M_37 |
		  DW_E56_HOLDOFF_TIMER_X245K_2F);
	writel(value, e56_addr + DW_E56_KRT_TFSM_CFG);

	if (priv->max_speed == SPEED_50000) {
		value = readl(e56_addr + DW_E56_PMD_SPARE4);
		value &= ~DW_E56_SPARE4;
		value |= DW_E56_SPARE4_3F;
		writel(value, e56_addr + DW_E56_PMD_SPARE4);

		value = readl(e56_addr + DW_E56_PMD_SPARE52);
		value &= ~DW_E56_SPARE52;
		value |= DW_E56_SPARE52_F0;
		writel(value, e56_addr + DW_E56_PMD_SPARE52);
	}

	temp_code = ioss_pvt_mon_read(priv);
	if (temp_code < PVT_MON_TEMP_CODE_40C) {
		// char_5
		/* T < 40deg C tracking range enabled */
		//printf("T < 40deg C tracking range enabled\n");
		if (priv->max_speed != SPEED_10000) {
			value = readl(e56_addr + DW_E56_ANA_OVRDVAL2);
			value &= ~DW_E56_ANA_LCPLL_HF_CALIB;
			value |= DW_E56_ANA_LCPLL_HF_CALIB_1;
			writel(value, e56_addr + DW_E56_ANA_OVRDVAL2);

			value = readl(e56_addr + DW_E56_ANA_OVRDEN0);
			value |= DW_E56_OVRD_EN_ANA_LCPLL_HF_CALIB;
			writel(value, e56_addr + DW_E56_ANA_OVRDEN0);
		} else {
			value = readl(e56_addr + DW_E56_ANA_OVRDVAL7);
			value &= ~DW_E56_ANA_LCPLL_LF_CALIB;
			value |= DW_E56_ANA_LCPLL_LF_CALIB_1;
			writel(value, e56_addr + DW_E56_ANA_OVRDVAL7);

			value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
			value |= DW_E56_OVRD_EN_ANA_LCPLL_LF_CALIB;
			writel(value, e56_addr + DW_E56_ANA_OVRDEN1);
		}
	} else if (temp_code > PVT_MON_TEMP_CODE_70C) {
		//printf("T > 70 tracking range enabled\n");
		if (priv->max_speed != SPEED_10000) {
			value = readl(e56_addr + DW_E56_ANA_OVRDVAL2);
			value &= ~DW_E56_ANA_LCPLL_HF_CALIB;
			value |= DW_E56_ANA_LCPLL_HF_CALIB_3;
			writel(value, e56_addr + DW_E56_ANA_OVRDVAL2);

			value = readl(e56_addr + DW_E56_ANA_OVRDEN0);
			value |= DW_E56_OVRD_EN_ANA_LCPLL_HF_CALIB;
			writel(value, e56_addr + DW_E56_ANA_OVRDEN0);
		} else {
			value = readl(e56_addr + DW_E56_ANA_OVRDVAL7);
			value &= ~DW_E56_ANA_LCPLL_LF_CALIB;
			value |= DW_E56_ANA_LCPLL_LF_CALIB_3;
			writel(value, e56_addr + DW_E56_ANA_OVRDVAL7);

			value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
			value |= DW_E56_OVRD_EN_ANA_LCPLL_LF_CALIB;
			writel(value, e56_addr + DW_E56_ANA_OVRDEN1);
		}
	}

        /* 2.3.1 - Step 4 Apply following setting at any value of T, including 40 <= T <= 70	*/
	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_ANA_OVRDVAL5);
		value &= ~DW_E56_ANA_LCPLL_HF_BIT33_32;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL5);

		value = readl(e56_addr + DW_E56_ANA_OVRDVAL4);
		value &= ~DW_E56_ANA_LCPLL_HF_BIT31_29;
		value |= DW_E56_ANA_LCPLL_HF_BIT31_29_4;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL4);
		value |= DW_E56_ANA_LCPLL_HF_BIT24;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL4);

		value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
		value |= DW_E56_OVRD_EN_ANA_LCPLL_HF_TST;
		writel(value, e56_addr + DW_E56_ANA_OVRDEN1);

	} else {
		value = readl(e56_addr + DW_E56_ANA_OVRDVAL9);
		value |= DW_E56_ANA_LCPLL_LF_BIT24;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL9);
		value &= ~DW_E56_ANA_LCPLL_LF_BIT31_29;
		value |= DW_E56_ANA_LCPLL_LF_BIT31_29_4;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL9);

		value = readl(e56_addr + DW_E56_ANA_OVRDVAL10);
		value &= ~DW_E56_ANA_LCPLL_LF_BIT33_32;
		writel(value, e56_addr + DW_E56_ANA_OVRDVAL10);

		value = readl(e56_addr + DW_E56_ANA_OVRDEN1);
		value |= DW_E56_OVRD_EN_ANA_LCPLL_LF_TST;
		writel(value, e56_addr + DW_E56_ANA_OVRDEN1);
	}

	udelay(100);

	/* Precode en */
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_PAM4_PRECODE_NO_KRT_EN; // PAM4 only
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_PLL_EN_CFG;
	/* Enable PLL based on speed */
	if (priv->max_speed == SPEED_10000) {
		value |= DW_E56_PLL_EN_PLL1;
	} else {
		value |= DW_E56_PLL_EN_PLL0;
	}
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_PMD_EN;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(100);
	value |= DW_E56_PMD_EN;
	writel(value, e56_addr + DW_E56_PMD_CFG0);

	// char_6
	/* Enable TXS only. RXS will be enabled after link is up */
	value &= ~DW_E56_TX_EN_CFG;
	value |= DW_E56_TX_EN_LANE0;
	writel(value, e56_addr + DW_E56_PMD_CFG0);

	value = readl(e56_addr + DW_E56_CTRL_FSM_CM_STAT0);
	while ((value & DW_E56_FSM_CM_ST) != DW_E56_FSM_CM_ST_PLL_RDY ) {
		udelay(100);
		value = readl(e56_addr + DW_E56_CTRL_FSM_CM_STAT0);
	};
	udelay(10000);
	//printf("CMS Ready status read is (0x%08x)\n", value);


	value = readl(e56_addr + DW_E56_CTRL_FSM_TX_STAT0);
	while ((value & DW_E56_FSM_TX0_ST) != DW_E56_FSM_TX0_ST_TX_RDY ) {
		udelay(100);
		value = readl(e56_addr + DW_E56_CTRL_FSM_TX_STAT0);
	};
	udelay(10000);
	//printf("Tx Ready status read is (0x%08x)\n", value);

	return 0;
}

#define PVT_MON_TEMP_CODE_M5C	144
#define PVT_MON_TEMP_CODE_30C	281
#define PVT_MON_TEMP_CODE_65C	436
#define PVT_MON_TEMP_CODE_100C	618
int xgbe_e56_serdes_rx_config(struct dw_eth_dev *priv)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;
	u32 coarse;
	s32 offset_centre_range_l;
	s32 offset_centre_range_h;
	u32 rx_coarse_mid_td = 8;
	u32 intr_rx_osc_freq_err_h = 0;
	u32 intr_rx_osc_freq_err_l = 0;
	int loop, i;

	// char_14
	/* OSC config Directly select RANGE_L as RANGE_FINAL for 50G and RANGE_H for 25/10G */

	value = ioss_pvt_mon_read(priv);
	if (value >= PVT_MON_TEMP_CODE_M5C && value < PVT_MON_TEMP_CODE_30C)
		rx_coarse_mid_td = 9;
	else if (value >= PVT_MON_TEMP_CODE_30C && value < PVT_MON_TEMP_CODE_65C)
		rx_coarse_mid_td = 8;
	else if (value >= PVT_MON_TEMP_CODE_65C && value < PVT_MON_TEMP_CODE_100C)
		rx_coarse_mid_td = 7;

	/* Set as RANGE_H */
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_50G;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_25G;
	else
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_10G;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL0);
	value &= ~DW_E56_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL0);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN0);
	value |= DW_E56_OVDR_EN_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN0);

	/* Enable RXS */
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_RX_EN_CFG;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(1000);
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value |= DW_E56_RX_EN_LANE0;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(100000);

	/* For debug */
	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("Range H  : 0x3c2f14fc is (0x%08x)\n", value);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	coarse = (value & DW_E56_ANA_BBCDR_COARSE) >> 4;

	value = readl(e56_addr + DW_E56_PMD_INTR0);
	if (value & DW_E56_INTR_RX_OSC_FREQ_ERR) {
		intr_rx_osc_freq_err_h = 1 ;
		//printf("Range H 0x3c2f15ec bit 8 should be 0 (0x%08x)\n", value);
	}

	offset_centre_range_h = coarse - rx_coarse_mid_td;
	//printf("Range H Coarse is (0x%08x)\n", coarse);
	//printf("offset_centre_range_h (0x%08x)\n", offset_centre_range_h);

	/* Disable RXS */
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
	value &= ~DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_RXS0_OVRDEN0);
	value &= ~DW_E56_OVDR_EN_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN0);

	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_RX_EN_CFG;
	writel(value, e56_addr + DW_E56_PMD_CFG0);

	value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	while ((value & DW_E56_FSM_RX0_ST) != DW_E56_FSM_RX0_ST_POWERDN ) {
		udelay(10);
		value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	};
	/* For debug */
	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("\n Range H : 0x3c2f14fc is (0x%08x)\n", value);

	value = readl(e56_addr + DW_E56_PMD_INTR0);
	value |= DW_E56_INTR_RX_OSC_FREQ_ERR;
	writel(value, e56_addr + DW_E56_PMD_INTR0);
	value = readl(e56_addr + DW_E56_PMD_INTR2);
	value |= DW_E56_INTR_CTRL_FSM_RX_ERR;
	writel(value, e56_addr + DW_E56_PMD_INTR2);

	/* Set as RANGE_L */
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
	if (priv->max_speed == SPEED_50000)
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_50G;
	else if (priv->max_speed == SPEED_25000)
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_25G;
	else
		value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_10G;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
	value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL0);
	value &= ~DW_E56_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL0);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN0);
	value |= DW_E56_OVDR_EN_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN0);

	/* Enable RXS */
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_RX_EN_CFG;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(1000);
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value |= DW_E56_RX_EN_LANE0;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(100000);

	/* For debug */
	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("Range L  : 0x3c2f14fc is (0x%08x)\n", value);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	coarse = (value & DW_E56_ANA_BBCDR_COARSE) >> 4;

	value = readl(e56_addr + DW_E56_PMD_INTR0);
	if (value & DW_E56_INTR_RX_OSC_FREQ_ERR) {
		intr_rx_osc_freq_err_l = 1 ;
		//printf("Range L 0x3c2f15ec bit 8 should be 0 (0x%08x)\n", value);
	}

	offset_centre_range_l = coarse - rx_coarse_mid_td;
	//printf("Range L Coarse is (0x%08x)\n", coarse);
	//printf("offset_centre_range_l (0x%08x)\n", offset_centre_range_l);

	/* Disable RXS */
	/* Remove Overrides */
	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
	value &= ~DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
	writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);

	value = readl(e56_addr + DW_E56_RXS0_OVRDEN0);
	value &= ~DW_E56_OVDR_EN_RXS_SAMP_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN0);

	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_RX_EN_CFG;
	writel(value, e56_addr + DW_E56_PMD_CFG0);

	value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	while ((value & DW_E56_FSM_RX0_ST) != DW_E56_FSM_RX0_ST_POWERDN ) {
		udelay(10);
		value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	};
	/* For debug */
	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("Range L : 0x3c2f14fc is (0x%08x)\n", value);

	value = readl(e56_addr + DW_E56_PMD_INTR0);
	value |= DW_E56_INTR_RX_OSC_FREQ_ERR;
	writel(value, e56_addr + DW_E56_PMD_INTR0);
	value = readl(e56_addr + DW_E56_PMD_INTR2);
	value |= DW_E56_INTR_CTRL_FSM_RX_ERR;
	writel(value, e56_addr + DW_E56_PMD_INTR2);
	if (offset_centre_range_l < 0)
		offset_centre_range_l = -offset_centre_range_l;
	if (offset_centre_range_h < 0)
		offset_centre_range_h = -offset_centre_range_h;
	//printf("offset_centre_range_h (0x%08x)\n", offset_centre_range_h);
	//printf("offset_centre_range_l (0x%08x)\n", offset_centre_range_l);

	if (intr_rx_osc_freq_err_h) {
		/* Set as RANGE_L */
		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
		value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
		if (priv->max_speed == SPEED_50000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_50G;
		else if (priv->max_speed == SPEED_25000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_25G;
		else
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_10G;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);

		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		//printf("Final Range is L as error is set in H\n");
	} else if (intr_rx_osc_freq_err_l) {
		/* Set as RANGE_H */
		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
		value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
		if (priv->max_speed == SPEED_50000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_50G;
		else if (priv->max_speed == SPEED_25000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_25G;
		else
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_10G;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);

		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		//printf("Final Range is H as error is set in L\n");
	} else if ((offset_centre_range_l) < (offset_centre_range_h)) {
		/* Set as RANGE_L */
		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
		value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
		if (priv->max_speed == SPEED_50000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_50G;
		else if (priv->max_speed == SPEED_25000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_25G;
		else
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_LSEL_10G;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);

		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		//printf("Final Range is L\n");
	} else 	{
		/* Set as RANGE_H */
		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
		value &= ~DW_E56_ANA_BBCDR_OSC_RANGE_SEL;
		if (priv->max_speed == SPEED_50000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_50G;
		else if (priv->max_speed == SPEED_25000)
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_25G;
		else
			value |= DW_E56_ANA_BBCDR_OSC_RANGE_HSEL_10G;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDVAL5);

		value = readl(e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		value |= DW_E56_OVRD_EN_ANA_BBCDR_OSC_RANGE_SEL;
		writel(value, e56_addr + DW_E56_RXS_ANA_OVRDEN0);
		//printf("Final Range is H\n");
        }

	/* Enable RXS */
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value &= ~DW_E56_RX_EN_CFG;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(1000);
	value = readl(e56_addr + DW_E56_PMD_CFG0);
	value |= DW_E56_RX_EN_LANE0;
	writel(value, e56_addr + DW_E56_PMD_CFG0);
	udelay(100000);
	/* For debug */
	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("Enable Range Final : 0x3c2f14fc is (0x%08x)\n", value);
	value = readl(e56_addr + DW_E56_PMD_INTR0);
	if (value & DW_E56_INTR_RX_OSC_FREQ_ERR)
		printf("Range Final 0x3c2f15ec bit 8 should be 0 (0x%08x)\n", value);

	// char_13
	/* Program following before enabling RXS. Purpose is to disable
	 * power-up FSM control on ADC offset adaptation, ADC gain
	 * adaptation and ADC interleaver calibration and adaptation.
	 * These will be controlled by software later in the sequence below.
	 */
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_OFST_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
	value |= DW_E56_OVRD_EN_RX0_ADC_OFST_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_OFST_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_GAIN_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
	value |= DW_E56_OVRD_EN_RX0_ADC_GAIN_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_GAIN_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_INTL_CAL_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_CAL_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_INTL_CAL_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value |= DW_E56_RX0_ADC_INTL_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value |= DW_E56_RX0_ADC_INTL_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	if (value & DW_E56_RXS0_OVRDVAL1_ERR_MASK1)
		printf("0x3c2f1544 bit 30 27 18 0 should be 0 (0x%08x)\n", value);

	//value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	//printf("Before Enable CDR : 0x3c2f14fc is (0x%08x)\n", value);

	// char_15
	/* Enable CRD */
	//value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	//value &= DW_E56_RX0_CDR_EN;
	//value = value >> 11;
	//printf("CDR Enable is (0x%08x)\n", value);
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_CDR_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_CRD_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value |= DW_E56_RX0_CDR_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value &= ~DW_E56_OVRD_EN_RX0_CRD_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_CDR_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_RXS_ANA_OVRDVAL5);
	if ((value & DW_E56_ANA_BBCDR_INT_CSTM) == 0 ||
	    (value & DW_E56_ANA_BBCDR_INT_CSTM) == DW_E56_ANA_BBCDR_INT_CSTM_1F)
		printf("0x3c2f00b4 bit 24:20 should not be 0x0 or 0x1f (0x%08x)\n", value);

	// char_16
	/* VGA training */
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_VGA_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_VGA_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_VGA_TRAIN_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	}

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_CTLE_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_CTLE_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_CTLE_TRAIN_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	}
	udelay(100);

	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_ADC_INTL_CAL_DONE;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	}
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value &= ~DW_E56_OVRD_EN_RX0_ADC_INTL_CAL_DONE;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value |= DW_E56_RX0_ADC_INTL_CAL_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_CAL_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);

	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value |= DW_E56_RX0_ADC_INTL_CAL_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	}

	for (i = 0; i < 1000; i++) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		if (value & DW_E56_RX0_ADC_INTL_CAL_DONE)
			break;
		udelay(100);
	}
	if ((value & DW_E56_RX0_ADC_INTL_CAL_DONE) == 0)
		printf("E56 serdes adc intl calibration not done\n");

	if (priv->max_speed != SPEED_10000) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_ADC_INTL_CAL_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);
	}

	/* Following is not in B0 sequence */
	//value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	//value &= ~DW_E56_OVRD_EN_RX0_ADC_INTL_CAL_EN;
	//writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	udelay(1000);

	// char_17
	/* ADC offset adaptation and ADC gain adaptation */
	for (loop = 0; loop < 16; loop++) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value |= DW_E56_RX0_ADC_OFST_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

		value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
		value |= DW_E56_OVRD_EN_RX0_ADC_OFST_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);

		for (i = 0; i < 1000; i++) {
			value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
			if ((value & DW_E56_RX0_ADC_OFST_ADAPT_DONE) && i > 100)
				break;
			udelay(100);
		}
		if ((value & DW_E56_RX0_ADC_OFST_ADAPT_DONE) == 0)
			printf("E56 serdes adc offset adapt not done (loop %d)\n", loop);

		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_ADC_OFST_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

		value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
		value |= DW_E56_OVRD_EN_RX0_ADC_OFST_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);
		udelay(10000);

		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value |= DW_E56_RX0_ADC_GAIN_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

		value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
		value |= DW_E56_OVRD_EN_RX0_ADC_GAIN_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);

		for (i = 0; i < 1000; i++) {
			value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
			if ((value & DW_E56_RX0_ADC_GAIN_ADAPT_DONE) && i > 100)
				break;
			udelay(100);
		}
		if ((value & DW_E56_RX0_ADC_GAIN_ADAPT_DONE) == 0)
			printf("E56 serdes adc gain adapt not done (loop %d)\n", loop);

		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
		value &= ~DW_E56_RX0_ADC_GAIN_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

		value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
		value |= DW_E56_OVRD_EN_RX0_ADC_GAIN_ADAPT_EN;
		writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);
		udelay(10000);
	}
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	if (value & DW_E56_RXS0_OVRDVAL1_ERR_MASK2)
		printf("0x3c2f1544 bit 27 18 0 should be 0 (0x%08x)\n", value);

	// char_18
	/* ADC interleave adaptation for 10ms or greater, then disable it */
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value |= DW_E56_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);

	for (i = 0; i < 10000; i++) {
		value = readl(e56_addr + DW_E56_RXS0_OVRDVAL2);
		if (value & DW_E56_RX0_ADC_INTL_ADAPT_DONE)
			break;
		udelay(100);
	}
	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL2);
	if (value & DW_E56_RX0_ADC_INTL_ADAPT_ERR)
		printf("RX0 ADC intl adapt error\n");

	value = readl(e56_addr + DW_E56_RXS0_OVRDVAL1);
	value &= ~DW_E56_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDVAL1);

	value = readl(e56_addr + DW_E56_RXS0_OVRDEN2);
	value |= DW_E56_OVRD_EN_RX0_ADC_INTL_ADAPT_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN2);

	/* All speed remove VGA , CTLE disable override */
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value &= ~DW_E56_OVRD_EN_RX0_VGA_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	value = readl(e56_addr + DW_E56_RXS0_OVRDEN1);
	value &= ~DW_E56_OVRD_EN_RX0_CTLE_TRAIN_EN;
	writel(value, e56_addr + DW_E56_RXS0_OVRDEN1);
	udelay(1000);

	value = readl(e56_addr + DW_E56_CTRL_FSM_RX_STAT0);
	if ((value & DW_E56_FSM_RX0_ST) != DW_E56_FSM_RX0_ST_RX_RDY)
		printf("0x3c2f14fc bit 5:0 should be 0x1B (0x%08x)\n", value);

	return 0;
}

// e32 loopback caused PHY host side link down */
#if 0
static int xgbe_e32_serdes_rx2tx_lb(struct dw_eth_dev *priv, u8 enable)
{
	u16 data_pair[2];

	if (master_lane_ioaddr == 0)
		return 0;

	/* rx2tx loopback */
	if (enable) {
		data_pair[0] = DIG_PCS_XF_LANE_OVRD_BCAST;
		data_pair[1] = RX2TX_LB_EN_OVRD | RX2TX_LB_EN;
	}
	else {
		data_pair[0] = DIG_PCS_XF_LANE_OVRD_BCAST;
		data_pair[1] = RX2TX_LB_EN_OVRD;
	}
	if (master_is_xpcs)
		write_e32_cr_10g(master_lane_ioaddr, data_pair);
	else
		write_e32_cr_25g(master_lane_ioaddr, data_pair);

	return 0;
}
#endif

static int write_e32_cr(phys_addr_t ioaddr, u16 *data_pair)
{
	int ret;

	if (master_is_xpcs)
		ret = write_e32_cr_10g(master_lane_ioaddr, data_pair);
	else
		ret = write_e32_cr_25g(master_lane_ioaddr, data_pair);

	return ret;
}

static int xgbe_e32_serdes_prbs(struct dw_eth_dev *priv, u8 enable)
{
	u16 data_pair[2];

	if (master_lane_ioaddr == 0)
		return 0;

	if (enable) {
		/* Enable e32 prbs transmit */
		data_pair[0] = DIG_TX_LBERT_CTL +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = BERT_MODE_LFSR31;
		write_e32_cr(master_lane_ioaddr, data_pair);
	}
	else {
		/* Set e32 prbs pattern */
		data_pair[0] = DIG_RX_LBERT_CTL +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = BERT_MODE_LFSR31 | BERT_ERR_CNT_CLR_N;
		write_e32_cr(master_lane_ioaddr, data_pair);
		/* Start e32 prbs Rx match and reset error count */
		data_pair[1] |= BERT_SYNC;
		write_e32_cr(master_lane_ioaddr, data_pair);
		data_pair[0] = DIG_RX_LBERT_ERR +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = read_e32_cr_10g(master_lane_ioaddr,
					       data_pair[0]);
		data_pair[1] = read_e32_cr_10g(master_lane_ioaddr,
					       data_pair[0]);
		printf("SoC prbs error_count 0x%04X\n", data_pair[1]);
		udelay(1000000);

		/* Need two reads for error counter */
		data_pair[0] = DIG_RX_LBERT_ERR +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = read_e32_cr_10g(master_lane_ioaddr,
					       data_pair[0]);
		data_pair[1] = read_e32_cr_10g(master_lane_ioaddr,
					       data_pair[0]);
		printf("SoC prbs error_count 0x%04X\n", data_pair[1]);

		/* Disable e32 prbs */
		data_pair[0] = DIG_TX_LBERT_CTL +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = BERT_MODE_DISABLE;
		write_e32_cr(master_lane_ioaddr, data_pair);
		data_pair[0] = DIG_RX_LBERT_CTL +
			       priv->phy_lane * DW_E32_LANE_OFFSET;
		data_pair[1] = BERT_MODE_DISABLE;
		write_e32_cr(master_lane_ioaddr, data_pair);
	}

	return 0;
}

#if 1
static int xgbe_e56_serdes_rx2tx_lb(struct dw_eth_dev *priv, u8 enable)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;

	value = readl(e56_addr + DW_E56_PMD_CFG5);
	if (enable)
		value |= DW_E56_RX_TO_TX_LOOPBACK;
	else
		value &= ~DW_E56_RX_TO_TX_LOOPBACK;
	writel(value, e56_addr + DW_E56_PMD_CFG5);

	return 0;
}
#endif

#if 0
static int xgbe_e56_serdes_prbs(struct dw_eth_dev *priv, u8 enable)
{
	phys_addr_t e56_addr = priv->e56_base;
	u32 value;

	if (enable) {
		/* Set e56 tx prbs pattern */
		value = readl(e56_addr + DW_E56_TXS_TST_CFG0);
		value &= ~DW_E56_TXS_PATTERN_SEL;
		value |= DW_E56_TXS_PATTERN_PRBS31;
		value |= DW_E56_TXS_PATTERN_EN;
		writel(value, e56_addr + DW_E56_TXS_TST_CFG0);
	}
	else {
		/* Set e56 rx prbs pattern */
		value = readl(e56_addr + DW_E56_RXS_DFT1);
		value &= ~DW_E56_RXS_PATTERN_SEL;
		value &= ~DW_E56_RXS_BER_READ_MODE_EN;
		value |= DW_E56_RXS_PATTERN_PRBS31;
		value |= DW_E56_RXS_BER_EN;
		writel(value, e56_addr + DW_E56_RXS_DFT1);

		/* Read prbs error count */
		value = readl(e56_addr + DW_E56_RXS_DFT1);
		value |= DW_E56_RXS_BER_READ_MODE_EN;
		writel(value, e56_addr + DW_E56_RXS_DFT1);
		value = readl(e56_addr + DW_E56_RXS_DFT3);
		printf("SoC prbs error_count 0x%04X\n", value);
		udelay(1000000);

		/* Read prbs error count */
		value = readl(e56_addr + DW_E56_RXS_DFT1);
		value &= ~DW_E56_RXS_BER_READ_MODE_EN;
		writel(value, e56_addr + DW_E56_RXS_DFT1);
		value |= DW_E56_RXS_BER_READ_MODE_EN;
		writel(value, e56_addr + DW_E56_RXS_DFT1);
		value = readl(e56_addr + DW_E56_RXS_DFT3);
		printf("SoC prbs error_count 0x%04X\n", value);

		/* Disable e56 rx prbs */
		value = readl(e56_addr + DW_E56_RXS_DFT1);
		value &= ~DW_E56_RXS_BER_EN;
		writel(value, e56_addr + DW_E56_RXS_DFT1);

		/* Disable e56 tx prbs */
		value = readl(e56_addr + DW_E56_TXS_TST_CFG0);
		value &= ~DW_E56_TXS_PATTERN_EN;
		writel(value, e56_addr + DW_E56_TXS_TST_CFG0);
	}

	return 0;
}
#endif

int xgbe_serdes_rx2tx_lb(struct dw_eth_dev *priv, u8 enable)
{
	if (priv->eth_link == 1)
		xgbe_e56_serdes_rx2tx_lb(priv, enable);
		//xgbe_e56_serdes_prbs(priv, enable);
	else
		/* e32 rx2tx lb not working */
		//xgbe_e32_serdes_rx2tx_lb(priv, enable);
		xgbe_e32_serdes_prbs(priv, enable);

	return 0;
}

static int xgbe_e32_serdes_tx2rx_lb(struct dw_eth_dev *priv, u8 enable)
{
	phys_addr_t ioaddr = priv->xpcs_base;
	u16 data_pair[2];

	if (priv->phy_lane != 0)
		return 0;

	/* tx2rx 4 lanes loopback pcs override */
	data_pair[0] = DIG_PCS_XF_LANE_OVRD_BCAST;
	data_pair[1] = TX2RX_LB_EN_OVRD;
	if (enable)
		data_pair[1] |= TX2RX_LB_EN;
	else
		data_pair[1] &= ~TX2RX_LB_EN;
	write_e32_cr_10g(ioaddr, data_pair);

	return 0;
}

int xgbe_serdes_tx2rx_lb(struct dw_eth_dev *priv, u8 enable)
{
	if (priv->eth_link == 1) {
		printf("E56 serdes loopback not supported\n");
		return -EINVAL;
	}
	else if (priv->eth_link == 2) {
		xgbe_e32_serdes_tx2rx_lb(priv, enable);
	}

	return 0;
}
