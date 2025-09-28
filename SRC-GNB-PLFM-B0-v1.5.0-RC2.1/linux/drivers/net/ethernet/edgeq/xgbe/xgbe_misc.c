// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 EdgeQ, Inc.
 */

#include "xgbe.h"
#include "xgbe_misc.h"

static void __iomem *misc_addr;

void xgbe_pcs_mux_config(struct xgmac_priv *priv)
{
	int speed = priv->plat->mac_port_sel_speed;
	u32 misc_cfg;

	if (priv->plat->phy_lane != 0)
		return;

	if (!misc_addr)
		return;

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
	if (speed == SPEED_25000 && priv->plat->eth_link == 2)
		misc_cfg |= XGBE_MUX_PCS_SEL_LINK2_25G;
	else
		misc_cfg &= ~XGBE_MUX_PCS_SEL_LINK2_25G;
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);
}

void xgbe_mdio_mux_config(struct xgmac_priv *priv)
{
	u32 misc_cfg;

	if (!misc_addr)
		return;

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
	if (priv->plat->eth_link == 1)
		misc_cfg |= XGBE_MUX_MDIO_SEL_LINK1_XLGMAC;
	else
		misc_cfg &= ~XGBE_MUX_MDIO_SEL_LINK1_XLGMAC;
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);
}

int xgbe_set_mac_mem_active(struct xgmac_priv *priv)
{
	int speed = priv->plat->mac_port_sel_speed;
	u32 misc_cfg;

	if (!misc_addr)
		return 0;

	misc_cfg = readl(misc_addr + XGBE_MISC_CFG);
	if (priv->plat->eth_link == 1) {
		misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
		misc_cfg |= XGBE_XLGPCS_50G_ACTIVE;
		if (speed == SPEED_10000)
			misc_cfg |= XGBE_XLGPCS_E56_10G_MODE;
		else
			misc_cfg &= ~XGBE_XLGPCS_E56_10G_MODE;
	} else {
		if (priv->plat->phy_lane == 0) {
			if (speed == SPEED_25000) {
				misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
				misc_cfg |= XGBE_XLGPCS_25G_ACTIVE;
			} else {
				misc_cfg |= XGBE_XGMAC_MEM_ACTIVE;
				misc_cfg |= XGBE_XPCS_ACTIVE;
			}
		} else {
			misc_cfg |= XGBE_XLGMAC_MEM_ACTIVE;
			misc_cfg |= XGBE_XLGPCS_25G_ACTIVE;
		}
	}
	writel(misc_cfg, misc_addr + XGBE_MISC_CFG);

	return 0;
}

void xgbe_set_misc_addr(void __iomem *misc_base)
{
	misc_addr = misc_base;
}

bool xgbe_is_misc_addr_set(void)
{
	return misc_addr;
}

void xgbe_aux_dac_write(struct xgmac_priv *priv, u32 val)
{
	u32 reg;

	reg = readl(priv->plat->dac_base);
	reg &= XGBE_AUX_DAC_TOGGLE;	/* Keel toggle bit unchanged */
	reg |= XGBE_AUX_DAC_OPM_EN;
	reg |= (val << XGBE_AUX_DAC_VAL_SHIFT) & XGBE_AUX_DAC_VAL;
	writel(reg, priv->plat->dac_base);
	reg ^= XGBE_AUX_DAC_TOGGLE;	/* Toggle the bit */
	writel(reg, priv->plat->dac_base);
}

void xgbe_misc_update_speed(struct xgmac_priv *priv, int speed)
{
	u32 misc_cfg;
	u32 ctxt_mask;
	u32 ctxt_val;
	int shift;

	if (priv->plat->eth_link == 1 || priv->plat->phy_lane != 0)
		return;

	if (!misc_addr)
		return;

	if (priv->plat->phy_lane < NUM_E32_LANES - 1)
		shift = 2 * priv->plat->phy_lane;
	else
		shift = 2 * (priv->plat->phy_lane + 1);
	if (speed == SPEED_25000)
		ctxt_val = (XGBE_LANE_CTXT_SPEED_4 << shift);
	else if (priv->plat->interface == PHY_INTERFACE_MODE_USXGMII)
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

int xgbe_ioss_pvt_mon_read(void)
{
	u32 value;
	u32 status;
	int retry = 20;

	if (!misc_addr)
		return -1;

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
	}

	if ((status & PVT_MON_DATA_VALID) != PVT_MON_DATA_VALID)
		return -1;

	return (status >> 1);
}

