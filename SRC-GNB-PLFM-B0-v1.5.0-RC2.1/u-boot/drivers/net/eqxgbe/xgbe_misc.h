/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
  Copyright (c) 2020  EdgeQ Inc.
*******************************************************************************/

#ifndef __XGBE_MISC_H__
#define __XGBE_MISC_H__

/* The following are three Ethernet misc registers */
/* ETHERNET_MISC_CFG register */
#define XGBE_MISC_CFG			0x0028
#define XGBE_MUX_PCS_SEL_LINK2_25G	BIT(0)
#define XGBE_MUX_MDIO_SEL_LINK1_XLGMAC	BIT(1)
#define XGBE_XLGPCS_25G_ACTIVE		BIT(4)  /* Write 1 to be active */
#define XGBE_XLGPCS_50G_ACTIVE		BIT(5)  /* Write 1 to be active */
#ifdef CONFIG_TARGET_EQ_RAPTOR2_B0
#define XGBE_XLGPCS_E56_10G_MODE	BIT(6)  /* 0 for 25G/50G */
#endif
#define XGBE_XPCS_ACTIVE		GENMASK(11, 8)  /* Write 1 to be active */
#define XGBE_XGMAC_MEM_ACTIVE		BIT(22) /* 1: out of shutdown */
#define XGBE_XLGMAC_MEM_ACTIVE		BIT(23) /* 1: out of shutdown */
#define XGBE_MISC_CFG1			0x002C
#ifdef CONFIG_TARGET_EQ_RAPTOR2_B0
#define XGBE_LANE_CFG_OVERRIDE		0x00C0
#define XGBE_LANE_25G_MODE		BIT(0) /*0 for 10G */
#define XGBE_REFA_CLK_DIV2_EN		BIT(4)
#define XGBE_REFA_CLK_DIV2_LANE0_25G	BIT(5)  /* 0 for 10G */
#define XGBE_CTXT_ID_EN			BIT(8)
#define XGBE_PHY_RESET_EN		BIT(9)
#define XGBE_PHY_RESET			BIT(10)
#define XGBE_LANE_LINK_NUM_CFG		0x00D8
#define XGBE_LANE_0_LINK_NUM		(0 << 0)
#define XGBE_LANE_1_LINK_NUM		(1 << 2)
#define XGBE_LANE_2_LINK_NUM		(2 << 4)
#define XGBE_LANE_3_LINK_NUM		(3 << 8)
#define XGBE_LANE_FW_CTXT_CFG		0x00F0
#define XGBE_LANE_CTXT_ID		GENMASK(1, 0)
/* CTXT_ID value depends on e32 fw */
#define XGBE_LANE_CTXT_SPEED_1		2 /* 1st speed: 1G */
#define XGBE_LANE_CTXT_SPEED_2		0 /* 2nd speed: 2.5G */
#define XGBE_LANE_CTXT_SPEED_3		3 /* 3rd speed: USXGMII */
#define XGBE_LANE_CTXT_SPEED_4		1 /* 4th speed: 25G */
#endif
/* SERDES_E56_CFG register */
#define XGBE_E56_CFG0			0x0128
#define XGBE_E56_AN0_RATE_SELECT	GENMASK(4, 0)
#define XGBE_E56_AN0_RATE_10G		0x02
#define XGBE_E56_AN0_RATE_25G		0x0A
#define XGBE_E56_AN0_RATE_50G		0x0D
#define XGBE_E56_CFG1			0x012C
#define XGBE_E56_CLK_SEL		BIT(0) /* 0: from IOSS pll0 */

#define XGBE_PVT_MON_CFG		0x00B8
#define PVT_MON_EN			BIT(0)
#define PVT_MON_CLK_EN			BIT(1)
#define PVT_MON_DEFAULT_SETTING		0x3401E0
#define XGBE_PVT_MON_STATUS		0x01E0
#define PVT_MON_DATA_VALID		BIT(0)


void xgbe_pcs_mux_config(struct dw_eth_dev *priv);
void xgbe_mdio_mux_config(struct dw_eth_dev *priv);
int xgbe_set_mac_mem_active(struct dw_eth_dev *priv);
void xgbe_misc_init(struct dw_eth_dev *priv);
void xgbe_misc_phy_reset(struct dw_eth_dev *priv);
void xgbe_misc_update_speed(struct dw_eth_dev *priv, int speed);

#endif /* __XGBE_MISC_H__ */
