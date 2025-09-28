// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 EdgeQ, Inc.
 *
 * Synopsys DesignWare XPCS/XLGPCS/E56 APB3 API
 *
 */

#include "xgbe.h"
#include "xgbe_misc.h"
#include "xgbe_xpcs.h"


#define xpcs_warn(__device, __link, __args...) \
({ \
	if (__link) \
		dev_warn(__device, ##__args); \
})

static int xpcs_read_fault_10g(struct device *dev, void __iomem *ioaddr,
			       int link)
{
	u16 value;

	//value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
	//if (value & DW_RX_STS1_FAULT) {
	//	xpcs_warn(dev, link, "Link fault condition detected!\n");
	//	return -EFAULT;
	//}

	//value = readw(ioaddr + DW_XPCS_SR_PCS_STS2);
	//if (value & DW_RX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Receiver fault detected!\n");
	//if (value & DW_TX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Transmitter fault detected!\n");

	value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		xpcs_warn(dev, link, "FIFO fault condition detected!\n");
		return -EFAULT;
	}

	//value = readw(ioaddr + DW_XPCS_SR_PCS_KR_STS1);
	//if (!(value & DW_RPCS_BKLK))
	//	xpcs_warn(dev, link, "Link is not locked!\n");

	//value = readw(ioaddr + DW_XPCS_SR_PCS_KR_STS2);
	//if (value & DW_ERR_BLK) {
	//	xpcs_warn(dev, link, "Link has errors!\n");
	//	return -EFAULT;
	//}

	return 0;
}

static int xpcs_read_fault_25g(struct device *dev, void __iomem *ioaddr,
			       int link)
{
	u16 value;

	//value = readw(ioaddr + DW_25G_SR_PCS_STS1);
	//if (value & DW_RX_STS1_FAULT) {
	//	xpcs_warn(dev, link, "Link fault condition detected!\n");
	//	return -EFAULT;
	//}

	//value = readw(ioaddr + DW_25G_SR_PCS_STS2);
	//if (value & DW_RX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Receiver fault detected!\n");
	//if (value & DW_TX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Transmitter fault detected!\n");

	value = readw(ioaddr + DW_25G_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		value = readw(ioaddr + DW_25G_VR_PCS_DIG_STS);
		if (value & DW_RXFIFO_ERR) {
			xpcs_warn(dev, link, "FIFO fault condition detected!\n");
			return -EFAULT;
		}
	}

	//value = readw(ioaddr + DW_25G_SR_PCS_BASER_STS1);
	//if (!(value & DW_RPCS_BKLK))
	//	xpcs_warn(dev, link, "Link is not locked!\n");

	//value = readw(ioaddr + DW_25G_SR_PCS_BASER_STS2);
	//if (value & DW_ERR_BLK) {
	//	xpcs_warn(dev, link, "Link has errors!\n");
	//	return -EFAULT;
	//}

	return 0;
}

static int xpcs_read_fault_50g(struct device *dev, void __iomem *ioaddr,
			       int link)
{
	u16 value;

	//value = readw(ioaddr + DW_50G_SR_PCS_STS1);
	//if (value & DW_RX_STS1_FAULT) {
	//	xpcs_warn(dev, link, "Link fault condition detected!\n");
	//	return -EFAULT;
	//}

	//value = readw(ioaddr + DW_50G_SR_PCS_STS2);
	//if (value & DW_RX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Receiver fault detected!\n");
	//if (value & DW_TX_STS2_FAULT)
	//	xpcs_warn(dev, link, "Transmitter fault detected!\n");

	value = readw(ioaddr + DW_50G_VR_PCS_DIG_STS);
	if (value & DW_RXFIFO_ERR) {
		xpcs_warn(dev, link, "FIFO fault condition detected!\n");
		return -EFAULT;
	}

	//value = readw(ioaddr + DW_50G_SR_PCS_BASER_STS1);
	//if (!(value & DW_RPCS_BKLK))
	//	xpcs_warn(dev, link, "Link is not locked!\n");

	//value = readw(ioaddr + DW_50G_SR_PCS_BASER_STS2);
	//if (value & DW_ERR_BLK) {
	//	xpcs_warn(dev, link, "Link has errors!\n");
	//	return -EFAULT;
	//}

	return 0;
}

static u32 xpcs_get_id(void __iomem *ioaddr_1, void __iomem *ioaddr_2)
{
	u32 xpcs_id;

	xpcs_id = (u32)readw(ioaddr_1);
	xpcs_id = (xpcs_id << 16) + readw(ioaddr_2);

	return xpcs_id;
}

static int xpcs_poll_clear(void __iomem *ioaddr, u16 mon_bit)
{
	/* Poll until the bit clears (1ms per retry == 0.5 sec) */
	u16 retries = 500;
	u16 value;
	int ret;

	do {
		msleep(1);
		value = readw(ioaddr);
	} while ((value & mon_bit) && --retries);

	ret = (value & mon_bit) ? -ETIMEDOUT : 0;

	return ret;
}

//static int xpcs_poll_set(void __iomem *ioaddr, u16 mon_bit)
//{
//	/* Poll until the bit sets (1ms per retry == 0.1 sec) */
//	u16 retries = 100;
//	u16 value;
//	int ret;
//
//	do {
//		msleep(1);
//		value = readw(ioaddr);
//	} while (!(value & mon_bit) && --retries);
//
//	ret = !(value & mon_bit) ? -ETIMEDOUT : 0;
//
//	return ret;
//}

static int xpcs_soft_reset(void __iomem *ioaddr)
{
	u16 value;

	/* Reset may cause some issue */
	return 0;

	value = readw(ioaddr);
	value |= DW_CTRL1_RST;
	writel(value, ioaddr);

	return xpcs_poll_clear(ioaddr, DW_CTRL1_RST);
}

static int xpcs_probe_10g(void __iomem *ioaddr)
{
	u32 xpcs_id;

	xpcs_id = xpcs_get_id(ioaddr + DW_XPCS_SR_PCS_DEV_ID1,
			      ioaddr + DW_XPCS_SR_PCS_DEV_ID2);

	if (xpcs_id != DW_XPCS_ID)
		return -ENODEV;

	return 0;
}

static int xpcs_rx_reset_10g(void __iomem *ioaddr)
{
	u16 value;

	/* Rx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	value |= DW_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	usleep_range(5000, 5500);

	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	value &= ~DW_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	usleep_range(1000, 1100);

	return 0;
}

static int xpcs_config_10g(void __iomem *ioaddr, int interface)
{
	u16 value;
	int ret;

	/* Config is done in u-boot */
	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		/* USXGMII rate adaptor reset */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		value |= DW_USRA_RST;
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);

		ret = xpcs_poll_clear(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1,
				      DW_USRA_RST);
		if (ret)
			return ret;
	}

	/* Do Rx reset to get link up */
	xpcs_rx_reset_10g(ioaddr);
	ret = xpcs_poll_clear(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_STS,
			      DW_RX_ACK_0);
	return ret;
}

static int xpcs_get_state_10g(struct device *dev, void __iomem *ioaddr,
			      int *link)
{
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
		xpcs_rx_reset_10g(ioaddr);
		for (i = 0; i < 1000; i++) {
			value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
			if (value & DW_RX_LINK_UP) {
				*link = 1;
				break;
			}
			usleep_range(5, 10);
		}
	}

	/* Then check the faults */
	ret = xpcs_read_fault_10g(dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_XPCS_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_10g(void __iomem *ioaddr, int speed, int interface)
{
	u16 value;

	/* Tx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_TX_GENCTRL0);
	value |= DW_TX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_TX_GENCTRL0);
	/* Rx reset */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	value |= DW_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	usleep_range(800, 1000);

	if (interface == PHY_INTERFACE_MODE_SGMII) {
		/* Set PCS speed */
		value = readw(ioaddr + DW_XPCS_SR_PCS_CTRL2);
		value &= ~DW_XPCS_TYPE_SEL;
		value |= DW_XPCS_TYPE_10GBASE_X;
		writel(value, ioaddr + DW_XPCS_SR_PCS_CTRL2);

		/* Select mode */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		if (speed == SPEED_2500) {
			value |= DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		} else {
			value &= ~DW_EN_2_5G_MODE;
			value &= ~DW_USXG_EN;
		}
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
	}

	/* Configure MII speed */
	value = readw(ioaddr + DW_XPCS_SR_MII_CTRL);
	value &= ~DW_USXGMII_SS;
	if (interface == PHY_INTERFACE_MODE_SGMII)
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
	udelay(1);      // Wait for 1us for XGMII clock to stablize

	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		/* USXGMII rate adaptor reset */
		value = readw(ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
		value |= DW_USRA_RST;
		writel(value, ioaddr + DW_XPCS_VR_PCS_DIG_CTRL1);
	}

	/* Tx reset clear */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_TX_GENCTRL0);
	value &= ~DW_TX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_TX_GENCTRL0);
	/* Rx reset clear */
	value = readw(ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	value &= ~DW_RX_RST_0;
	writel(value, ioaddr + DW_XPCS_VR_PMA_12G_16G_25G_RX_GENCTRL1);
	usleep_range(10000, 20000);

	/* May need Rx reset twice */
	xpcs_rx_reset_10g(ioaddr);
	value = readw(ioaddr + DW_XPCS_SR_PCS_STS1);
	if (!(value & DW_RX_LINK_UP)) {
		usleep_range(30000, 50000);
		xpcs_rx_reset_10g(ioaddr);
	}

	return 0;
}

static int xpcs_probe_25g(void __iomem *ioaddr)
{
	u32 xpcs_id;

	xpcs_id = xpcs_get_id(ioaddr + DW_25G_SR_PCS_DEV_ID1,
			      ioaddr + DW_25G_SR_PCS_DEV_ID2);

	if (xpcs_id != DW_XLGPCS_25G_ID)
		return -ENODEV;

	return 0;
}

static int xpcs_config_25g(void __iomem *ioaddr, int interface)
{
	/* XLGPCS config is done in u-boot */
	return 0;
}

static int xpcs_rx_reset_25g(void __iomem *ioaddr)
{
	u16 value;

	/* Rx reset */
	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value |= DW_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	usleep_range(5000, 5500);

	value = readw(ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	value &= ~DW_RX_RST_0;
	writel(value, ioaddr + DW_25G_VR_PMA_MP_RX_GENCTRL1);
	usleep_range(1000, 1100);

	return 0;
}

static int xpcs_get_state_25g(struct device *dev, void __iomem *ioaddr,
			      int *link)
{
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
		xpcs_rx_reset_25g(ioaddr);
		for (i = 0; i < 1000; i++) {
			value = readw(ioaddr + DW_25G_SR_PCS_STS1);
			if (value & DW_RX_LINK_UP) {
				*link = 1;
				break;
			}
			usleep_range(5, 10);
		}
	}

	/* Then check the faults */
	ret = xpcs_read_fault_25g(dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_25G_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_25g(void __iomem *ioaddr, int speed, int interface)
{
	/* Do nothing for now */
	return 0;
}

static int xpcs_probe_50g(void __iomem *ioaddr)
{
	u32 xpcs_id;

	xpcs_id = xpcs_get_id(ioaddr + DW_50G_SR_PCS_DEV_ID1,
			      ioaddr + DW_50G_SR_PCS_DEV_ID2);

	if (xpcs_id != DW_XLGPCS_50G_ID)
		return -ENODEV;

	return 0;
}

static int xpcs_config_50g(void __iomem *ioaddr, int interface)
{
// Configured in u-boot
#if 0
	u16 value;
	int ret;

	/* Vendor specific soft reset */
	value = readw(ioaddr + DW_50G_VR_PCS_DIG_CTRL1);
	value |= DW_VR_RST;
	writel(value, ioaddr + DW_50G_VR_PCS_DIG_CTRL1);

	ret = xpcs_poll_clear(ioaddr + DW_50G_VR_PCS_DIG_CTRL1, DW_VR_RST);
	if (ret)
		return ret;

	/* Set speed to 50G PAM4 */
	value = readw(ioaddr + DW_50G_SR_PCS_CTRL2);
	value &= ~DW_XLGPCS_TYPE_SEL;
	value |= DW_XLGPCS_TYPE_50GBASE_R;
	writel(value, ioaddr + DW_50G_SR_PCS_CTRL2);
	value = readw(ioaddr + DW_50G_SR_PMA_CTRL2);
	value &= ~DW_XLGPCS_PMA_TYPE;
	value |= DW_XLGPCS_PMA_TYPE_50GBASE_KR;
	writel(value, ioaddr + DW_50G_SR_PMA_CTRL2);
	value = readw(ioaddr + DW_50G_SR_PCS_CTRL1);
	value &= ~DW_XLGPCS_SS_5_2;
	value |= DW_XLGPCS_SS_50G;
	writel(value, ioaddr + DW_50G_SR_PCS_CTRL1);

	/* Enable RS_FEC */
	value = readw(ioaddr + DW_50G_SR_PMA_RS_FEC_CTRL);
	value &= ~DW_FLP_25G_X_2; // Select 50G_X_1
	value |= DW_RSFEC_EN;
	writel(value, ioaddr + DW_50G_SR_PMA_RS_FEC_CTRL);
#endif

	return 0;
}

static int xpcs_get_state_50g(struct device *dev, void __iomem *ioaddr,
			      int *link)
{
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

	/* Then check the faults */
	ret = xpcs_read_fault_50g(dev, ioaddr, *link);
	if (ret) {
		ret = xpcs_soft_reset(ioaddr + DW_50G_SR_PCS_CTRL1);
		if (ret)
			return ret;

		*link = 0;
	}

	return 0;
}

static int xpcs_update_speed_50g(void __iomem *ioaddr, int speed, int interface)
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

static struct xpcs_ops xpcs_ops_null = {
	.config = NULL,
	.get_state = NULL,
	.probe = NULL,
};

struct xpcs_ops *xpcs_get_ops(struct xgmac_priv *priv)
{
	struct xpcs_ops *ops;

	if (!priv)
		return &xpcs_ops_null;

	switch (priv->plat->eth_link) {
	case 1:
		ops = &xpcs_ops_50g;
		break;
	case 2:
		if (priv->plat->phy_lane == 0)
			ops = &xpcs_ops_10g;
		else
			ops = &xpcs_ops_25g;
		break;
	default:
		ops = &xpcs_ops_null;
		break;
	}

	return ops;
}

static int e56_rx_temp_track(void __iomem *e56_base, int speed)
{
	u32 value;
	u32 sec_code;
	u32 ufine_code;
	u32 fine_code;
	u32 coarse_code;
	bool do_update = false;

	value = readl(e56_base + DW_E56_RXS_ANA_OVRDVAL5);
	sec_code = value & DW_E56_ANA_BBCDR_INT_CSTM;
	ufine_code = value & DW_E56_ANA_BBCDR_UFINE;
	fine_code = value & DW_E56_ANA_BBCDR_FINE;
	coarse_code = value & DW_E56_ANA_BBCDR_COARSE;
	if (sec_code <= DW_E56_ANA_BBCDR_INT_CSTM_SEC_LOW_TH) {
		if ((speed == SPEED_50000 && ufine_code < DW_E56_ANA_BBCDR_UFINE_MAX_50G) ||
		    (speed == SPEED_25000 && ufine_code < DW_E56_ANA_BBCDR_UFINE_MAX_25G) ||
		    (speed == SPEED_10000 && ufine_code < DW_E56_ANA_BBCDR_UFINE_MAX_10G)) {
			ufine_code >>= DW_E56_ANA_BBCDR_UFINE_SHIFT;
			ufine_code += 1;
			ufine_code <<= DW_E56_ANA_BBCDR_UFINE_SHIFT;
			do_update = true;
		} else if (fine_code < DW_E56_ANA_BBCDR_FINE_MAX) {
			if (speed == SPEED_50000)
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMAX_WRAP_50G;
			else if (speed == SPEED_25000)
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMAX_WRAP_25G;
			else
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMAX_WRAP_10G;
			fine_code >>= DW_E56_ANA_BBCDR_FINE_SHIFT;
			fine_code += 1;
			fine_code <<= DW_E56_ANA_BBCDR_FINE_SHIFT;
			do_update = true;
		} else if (coarse_code < DW_E56_ANA_BBCDR_COARSE_MAX) {
			if (speed == SPEED_50000) {
				ufine_code = DW_E56_ANA_BBCDR_UFINE_FMAX_WRAP_50G;
				fine_code = DW_E56_ANA_BBCDR_FINE_FMAX_WRAP_50G;
			} else if (speed == SPEED_25000) {
				ufine_code = DW_E56_ANA_BBCDR_UFINE_FMAX_WRAP_25G;
				fine_code = DW_E56_ANA_BBCDR_FINE_FMAX_WRAP_25G;
			} else {
				ufine_code = DW_E56_ANA_BBCDR_UFINE_FMAX_WRAP_10G;
				fine_code = DW_E56_ANA_BBCDR_FINE_FMAX_WRAP_10G;
			}
			coarse_code >>= DW_E56_ANA_BBCDR_COARSE_SHIFT;
			coarse_code += 1;
			coarse_code <<= DW_E56_ANA_BBCDR_COARSE_SHIFT;
			do_update = true;
		}
	} else if (sec_code >= DW_E56_ANA_BBCDR_INT_CSTM_SEC_HIGH_TH) {
		if (ufine_code > DW_E56_ANA_BBCDR_UFINE_MIN) {
			ufine_code >>= DW_E56_ANA_BBCDR_UFINE_SHIFT;
			ufine_code -= 1;
			ufine_code <<= DW_E56_ANA_BBCDR_UFINE_SHIFT;
			do_update = true;
		} else if (fine_code > DW_E56_ANA_BBCDR_FINE_MIN) {
			if (speed == SPEED_50000)
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMIN_WRAP_50G;
			else if (speed == SPEED_25000)
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMIN_WRAP_25G;
			else
				ufine_code = DW_E56_ANA_BBCDR_UFINE_UMIN_WRAP_10G;
			fine_code >>= DW_E56_ANA_BBCDR_FINE_SHIFT;
			fine_code -= 1;
			fine_code <<= DW_E56_ANA_BBCDR_FINE_SHIFT;
			do_update = true;
		} else if ((speed == SPEED_50000 &&
			    coarse_code > DW_E56_ANA_BBCDR_COARSE_MIN_50G) ||
			   (speed == SPEED_25000 &&
			    coarse_code < DW_E56_ANA_BBCDR_COARSE_MIN_25G) ||
			   (speed == SPEED_10000 &&
			    coarse_code < DW_E56_ANA_BBCDR_COARSE_MIN_10G)) {
			ufine_code = DW_E56_ANA_BBCDR_UFINE_FMIN_WRAP;
			if (speed == SPEED_50000)
				fine_code = DW_E56_ANA_BBCDR_FINE_FMIN_WRAP_50G;
			else if (speed == SPEED_25000)
				fine_code = DW_E56_ANA_BBCDR_FINE_FMIN_WRAP_25G;
			else
				fine_code = DW_E56_ANA_BBCDR_FINE_FMIN_WRAP_10G;
			coarse_code >>= DW_E56_ANA_BBCDR_COARSE_SHIFT;
			coarse_code -= 1;
			coarse_code <<= DW_E56_ANA_BBCDR_COARSE_SHIFT;
			do_update = true;
		}
	}
	if (do_update) {
		value &= ~DW_E56_ANA_BBCDR_UFINE;
		value &= ~DW_E56_ANA_BBCDR_FINE;
		value &= ~DW_E56_ANA_BBCDR_COARSE;
		value |= (ufine_code | fine_code | coarse_code);
		writel(value, e56_base + DW_E56_RXS_ANA_OVRDVAL5);
		value = readl(e56_base + DW_E56_RXS_ANA_OVRDEN1);
		value |= DW_E56_OVRD_EN_ANA_BBCRD_COARSE;
		value |= DW_E56_OVRD_EN_ANA_BBCRD_FINE;
		value |= DW_E56_OVRD_EN_ANA_BBCRD_UFINE;
		writel(value, e56_base + DW_E56_RXS_ANA_OVRDEN1);
	}

	return false;
}

#define DO_CALIB_COUNT	10
static u32 prev_temp_code;
static u32 calib_count;
int xgbe_e56_calibration(struct xgmac_priv *priv, int speed)
{
	int temp_code;

	temp_code = xgbe_ioss_pvt_mon_read();
	if (temp_code < 0)
		return -1;

	calib_count++;
	/* When temperature code changes 20, temperature changes about 3-5 degeeC. */
	if (calib_count >= DO_CALIB_COUNT ||
	    temp_code > prev_temp_code + 20 ||
	    temp_code < prev_temp_code - 20) {
		e56_rx_temp_track(priv->e56_base, speed);
		prev_temp_code = temp_code;
		calib_count = 0;
	}

	return 0;
}

