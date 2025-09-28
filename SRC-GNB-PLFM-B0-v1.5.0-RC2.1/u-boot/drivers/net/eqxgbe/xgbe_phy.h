// SPDX-License-Identifier: GPL-2.0-only
/*
 * (c) Copyright 2021 EdgeQ Inc
 */

#ifndef _XGBE_PHY_H__
#define _XGBE_PHY_H__

#ifdef CONFIG_TARGET_EQ_RAPTOR2_A0
#define BSS_PERIPH_BASE		0x69200000
#endif
#ifdef CONFIG_TARGET_EQ_RAPTOR2_B0
#define BSS_PERIPH_BASE		0x6E400000
#endif
#define GPIO_OFFSET		0x40020

int xgbe_phy_init(struct dw_eth_dev *priv);
u32 xgbe_phy_supported_features(struct dw_eth_dev *priv);
void xgbe_phy_post_config(struct dw_eth_dev *priv);

#endif // _XGBE_PHY_H__

