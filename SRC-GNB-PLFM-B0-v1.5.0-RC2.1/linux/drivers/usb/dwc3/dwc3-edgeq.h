/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  EdgeQ USB driver header file
 *
 *  Copyright (C) 2023 EdgeQ Inc.
 *
 * Authors: Aman Sharma <aman.sharma@edgeq.io>,
 *	    Aakash Verma <aakash.verma@edgeq.io>
 */

#ifndef __DRIVERS_USB_DWC3_EDGEQ_H
#define __DRIVERS_USB_DWC3_EDGEQ_H

#include "../drivers/soc/edgeq/eeprom/eeprom_data.h"

#define TIMEOUT_JIFFIES		(msecs_to_jiffies(500))  // Set the timeout to 0.5 seconds
/* IOSS Specific and PLL Configuration */

/* Config 6 */
#define USB_RST_N		BIT(3)
#define PCIE_USB_NOC_RST_N	BIT(4)

/* Config 7 */
#define PCIE_USB_QR_CLK_EN	BIT(8)

/* Config 16*/
#define IOSS_MST_MAINSHDN	BIT(0)

/* IOSS Specific and PLL Configuration Registers */
#define IOSS_RST_CONFIG_6	0x40
#define IOSS_CLK_CONFIG_7	0x48
#define IOSS_MST_CONFIG_16	0x90


/* Bringup Configuration Constants */
#define NUM_USB2_PORT		1
#define NUM_USB3_PORT		1

/* Bringup Configuration Registers */
#define BRINGUP_CONFIG_0	0x20
#define VCC_RESET_N		BIT(0)
#define VCC_RESET_N_MASK	BIT(0)

#define BRINGUP_CONFIG_2_L	0x30
#define HOST_NUM_U2_PORT(n)	((n) << 16)
#define HOST_NUM_U3_PORT(n)	((n) << 20)
#define HOST_NUM_U2_PORT_MASK	(0xF << 16)
#define HOST_NUM_U3_PORT_MASK	(0xF << 20)
#define DISABLE_SS		(0x1 << 28)

#define BRINGUP_CONFIG_2_H	0x34
#define XHC_BME			BIT(16)
#define XHC_BME_MASK		BIT(16)

#define BRINGUP_CONFIG_4	0x40
#define SHUTDOWN_BIT		BIT(0)
#define SHUTDOWN_BIT_MASK	BIT(0)


/* Phy Misc Constants */
#define REF_CLK_SEL_VAL		2
#define FREQ_SEL_VAL		0x27

/* Phy Misc Registers */
#define PHY_MISC_CONFIG_0	0x20
#define REF_CLK_SEL(n)		((n) << 12)
#define REF_CLK_SEL_MASK	(0x3 << 12)

#define PHY_MISC_CONFIG_0_1	0x24

#define PHY_MISC_CONFIG_1	0x28
#define FREQ_SEL(n)		((n) << 4)
#define FREQ_SEL_MASK		(0x3F << 4)

#define PHY_MISC_CONFIG_1_1	0x2c

#define PHY_MISC_CONFIG_2_L	0x30
#define PHY_RESET		BIT(0)
#define PHY_RESET_MASK		BIT(0)

#define PHY_MISC_CONFIG_2_H	0x34
#define REF_SSP_EN		BIT(0)
#define REF_SSP_EN_MASK		BIT(0)

#define PHY_MISC_CONFIG_3	0x38
#define REF_USE_PAD		BIT(0)
#define ALT_CLK_EN		BIT(24)
#define REF_USE_PAD_MASK	BIT(0)
#define ALT_CLK_EN_MASK		BIT(24)

struct dwc3_edgeq {
	struct device		*dev;
	void __iomem		*ioss_pll_base;
	void __iomem		*bringup_config_base;
	void __iomem		*phy_misc_base;
	struct platform_device	*dwc3;

	/* Deferred to dwc3 core layer for mode switch*/
	struct extcon_dev	*edev;
	enum usb_dr_mode	mode;
};

#endif /* __DRIVERS_USB_DWC3_EDGEQ_H */
