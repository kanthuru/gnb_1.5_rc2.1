/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
  Copyright (C) 2020  EdgeQ Inc.
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
#define XGBE_XLGPCS_E56_10G_MODE	BIT(6)  /* 0 for 25G/50G */
#define XGBE_XPCS_ACTIVE		GENMASK(11, 8)  /* Write 1 to be active */
#define XGBE_XGMAC_MEM_ACTIVE		BIT(22) /* 1: out of shutdown */
#define XGBE_XLGMAC_MEM_ACTIVE		BIT(23) /* 1: out of shutdown */
#define XGBE_LANE_FW_CTXT_CFG		0x00F0
#define XGBE_LANE_CTXT_ID		GENMASK(1, 0)
/* CTXT_ID value depends on e32 fw */
#define XGBE_LANE_CTXT_SPEED_1		2 /* 1st speed: 1G */
#define XGBE_LANE_CTXT_SPEED_2		0 /* 2nd speed: 2.5G */
#define XGBE_LANE_CTXT_SPEED_3		3 /* 3rd speed: USXGMII */
#define XGBE_LANE_CTXT_SPEED_4		1 /* 4th speed: 25G */
/* SERDES_E56_CFG register */
#define XGBE_E56_CFG0			0x0128
#define XGBE_E56_CFG1			0x012C
#define XGBE_E56_CLK_SEL		BIT(0) /* 0: from IOSS pll0 */

#define XGBE_AUX_DAC_OPM		GENMASK(2, 0)
#define XGBE_AUX_DAC_OPM_EN		0x3
#define XGBE_AUX_DAC_SELIF		BIT(3)	// 0: unsigned; 1: 2's comp
#define XGBE_AUX_DAC_VAL		GENMASK(27, 16)
#define XGBE_AUX_DAC_VAL_SHIFT		16
#define XGBE_AUX_DAC_TOGGLE		BIT(28)	// Toggle to make val effective

#define XGBE_DAC_MIN_VAL		0
#define XGBE_DAC_MID_VAL		1024
#define XGBE_DAC_MAX_VAL		2047

#define NUM_E32_LANES			4

#define XGBE_PVT_MON_CFG		0x00B8
#define PVT_MON_EN			BIT(0)
#define PVT_MON_CLK_EN			BIT(1)
#define PVT_MON_DEFAULT_SETTING		0x3401E0
#define XGBE_PVT_MON_STATUS		0x01E0
#define PVT_MON_DATA_VALID		BIT(0)

void xgbe_pcs_mux_config(struct xgmac_priv *priv);
void xgbe_mdio_mux_config(struct xgmac_priv *priv);
int xgbe_set_mac_mem_active(struct xgmac_priv *priv);
void xgbe_set_misc_addr(void __iomem *misc_base);
bool xgbe_is_misc_addr_set(void);
void xgbe_aux_dac_write(struct xgmac_priv *priv, u32 val);
void xgbe_misc_update_speed(struct xgmac_priv *priv, int speed);
int xgbe_ioss_pvt_mon_read(void);

#endif /* __XGBE_MISC_H__ */
