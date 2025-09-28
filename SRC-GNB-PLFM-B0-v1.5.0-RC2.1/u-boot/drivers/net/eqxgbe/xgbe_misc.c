// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 EdgeQ Inc.
 */

#include <common.h>
#include <errno.h>
#include <miiphy.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/io.h>
#include "xgbe_xgmac.h"
#include "xgbe_xpcs.h"
#include "xgbe_misc.h"


void xgbe_pcs_mux_config(struct dw_eth_dev *priv)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;

	if (priv->phy_lane != 0)
		return;

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
        if (priv->eth_link == 2 && priv->max_speed == SPEED_25000)
		misc_cfg |= XGBE_MUX_PCS_SEL_LINK2_25G;
	else
		misc_cfg &= ~XGBE_MUX_PCS_SEL_LINK2_25G;
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);
}

void xgbe_mdio_mux_config(struct dw_eth_dev *priv)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
	if (priv->eth_link == 1)
		misc_cfg |= XGBE_MUX_MDIO_SEL_LINK1_XLGMAC;
	else
		misc_cfg &= ~XGBE_MUX_MDIO_SEL_LINK1_XLGMAC;
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);
}

int xgbe_set_mac_mem_active(struct dw_eth_dev *priv)
{
	static bool reset_done = false;
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;

	/* pcs reset assertion. Only once. */
	if (priv->phy_lane == 0 && !reset_done) {
		writel(0, misc_addr + XGBE_MISC_CFG);
		writel(0, misc_addr + XGBE_MISC_CFG1);
		reset_done = true;
		udelay(100);
	}

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
	if (priv->eth_link == 1) {
		misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
		misc_cfg |= XGBE_XLGPCS_50G_ACTIVE;
		if (priv->max_speed == SPEED_10000)
			misc_cfg |= XGBE_XLGPCS_E56_10G_MODE;
		else
			misc_cfg &= ~XGBE_XLGPCS_E56_10G_MODE;
	}
	else {
		if (priv->phy_lane == 0) {
			if (priv->max_speed == SPEED_25000) {
				misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
				misc_cfg |= XGBE_XLGPCS_25G_ACTIVE;
			}
			else {
				misc_cfg |= XGBE_XGMAC_MEM_ACTIVE;
				misc_cfg |= XGBE_XPCS_ACTIVE;
			}
		}
		else {
			misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
			misc_cfg |= XGBE_XLGPCS_25G_ACTIVE;
		}
	}
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);

	return 0;
}

#ifdef CONFIG_TARGET_EQ_RAPTOR2_B0
void xgbe_misc_init(struct dw_eth_dev *priv)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;

	if (priv->eth_link == 1)
		return;

	misc_cfg = readl(misc_addr + XGBE_LANE_CFG_OVERRIDE);
	if (priv->max_speed == SPEED_25000) {
		misc_cfg |= (XGBE_LANE_25G_MODE << priv->phy_lane);
		if (priv->phy_lane == 0)
			misc_cfg |= XGBE_REFA_CLK_DIV2_LANE0_25G;
	}
	misc_cfg |= XGBE_REFA_CLK_DIV2_EN;
	misc_cfg |= XGBE_CTXT_ID_EN;
	writel(misc_cfg, misc_addr + XGBE_LANE_CFG_OVERRIDE);

	misc_cfg = XGBE_LANE_0_LINK_NUM |
		   XGBE_LANE_1_LINK_NUM |
		   XGBE_LANE_2_LINK_NUM |
		   XGBE_LANE_3_LINK_NUM;
	writel(misc_cfg, misc_addr + XGBE_LANE_LINK_NUM_CFG);

	/* Config e32 fw context number */
	xgbe_misc_update_speed(priv, priv->max_speed);
}

void xgbe_misc_update_speed(struct dw_eth_dev *priv, int speed)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;
	u32 ctxt_mask;
	u32 ctxt_val;
	int shift;

	if (priv->phy_lane < NUM_E32_LANES - 1)
		shift = 2 * priv->phy_lane;
	else
		shift = 2 * (priv->phy_lane + 1);
	if (speed == SPEED_25000)
		ctxt_val = (XGBE_LANE_CTXT_SPEED_4 << shift);
	else if (priv->interface == PHY_INTERFACE_MODE_USXGMII)
		ctxt_val = (XGBE_LANE_CTXT_SPEED_3 << shift);
	else if (speed == SPEED_2500)
		ctxt_val = (XGBE_LANE_CTXT_SPEED_2 << shift);
	else
		ctxt_val = (XGBE_LANE_CTXT_SPEED_1 << shift);
	ctxt_mask = XGBE_LANE_CTXT_ID << shift;
	misc_cfg = readl(misc_addr + XGBE_LANE_FW_CTXT_CFG);
	misc_cfg &= ~ctxt_mask;
	misc_cfg |= ctxt_val;
	writel(misc_cfg, misc_addr + XGBE_LANE_FW_CTXT_CFG);
}

void xgbe_misc_phy_reset(struct dw_eth_dev *priv)
{
	phys_addr_t misc_addr = priv->misc_base;
	u32 misc_cfg;

	if (priv->phy_lane != 0)
		return;

        if (priv->eth_link == 2) {
		misc_cfg = readl(misc_addr + XGBE_LANE_CFG_OVERRIDE);
		misc_cfg |= (XGBE_PHY_RESET_EN | XGBE_PHY_RESET);
		writel(misc_cfg, misc_addr + XGBE_LANE_CFG_OVERRIDE);
		udelay(100);

		misc_cfg = readl(misc_addr + XGBE_LANE_CFG_OVERRIDE);
		misc_cfg &= ~XGBE_PHY_RESET;
		writel(misc_cfg, misc_addr + XGBE_LANE_CFG_OVERRIDE);
		udelay(1000);
	}
}
#endif
