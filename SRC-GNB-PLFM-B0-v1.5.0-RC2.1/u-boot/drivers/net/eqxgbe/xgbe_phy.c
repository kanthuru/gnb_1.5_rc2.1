// SPDX-License-Identifier: GPL-2.0+
/*
 * (c) Copyright 2021 EdgeQ Inc
 */

/*
 * PHY driver for mv88x7120 PHY.
 */

#include <common.h>
#include <log.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <bitfield.h>
#include <errno.h>
#include <malloc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/io.h>
#include <command.h>
#include <i2c.h>

#include "xgbe_xgmac.h"
#include "xgbe_xpcs.h"
#include "xgbe_misc.h"
#include "xgbe_phy.h"


static const u32 phy_1g_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause |
	SUPPORTED_1000baseT_Full;

static const u32 phy_2g5_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause |
	SUPPORTED_2500baseX_Full;

static const u32 phy_5g_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause;

static const u32 phy_10g_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause |
	SUPPORTED_10000baseKR_Full;

static const int phy_25g_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause;

static const int phy_50g_supported =
	SUPPORTED_Autoneg |
	SUPPORTED_Pause |
	SUPPORTED_Asym_Pause;


#define GPIO_15		15
#define GPIO_20		20

u16 loopback_test = NO_LOOPBACK;

/* Only XGMAC0 or XLGMAC MDIO is connected MDIO bus */
phys_addr_t link2_mdio_mac_base = 0;

#if defined CONFIG_PHY_MV88X7121 || defined CONFIG_PHY_MV88X3310
static int xgbe_xgmac_mdio_read(phys_addr_t iobase, u16 mdio_port,
				u16 mmd, u16 reg, u16 *data)
{
	u32 mii_data = MII_BUSY;
	u32 mii_addr = 0;
	ulong start;

	if (iobase == 0)
		return -EFAULT;

	mii_addr = ((mmd << MII_DA_SHIFT) & MII_DA_MASK) |
		   ((mdio_port << MII_ADDR_SHIFT) & MII_ADDR_MASK) |
		   ((reg << MII_REG_SHIFT) & MII_REG_MASK);
	mii_data |= ((MII_CLK_400_450M << MII_CLK_SHIFT) & MII_CLK_MASK) |
		    ((MII_CMD_READ << MII_CMD_SHIFT) & MII_CMD_MASK);

	writel(mii_addr, iobase + MAC_MDIO_ADDR);
	writel(mii_data, iobase + MAC_MDIO_DATA);

	start = get_timer(0);
	while (get_timer(start) < MDIO_TIMEOUT) {
		if (!(readl(iobase + MAC_MDIO_DATA) & MII_BUSY)) {
			*data = (u16)readl(iobase + MAC_MDIO_DATA);
			return 0;
		}
		udelay(10);
	};

	return -EFAULT;
}

static int xgbe_xgmac_mdio_write(phys_addr_t iobase, u16 mdio_port,
				 u16 mmd, u16 reg, u16 data)
{
	u32 mii_data = data | MII_BUSY;
	u32 mii_addr = 0;
	ulong start;

	if (iobase == 0)
		return -EFAULT;

	mii_addr = ((mmd << MII_DA_SHIFT) & MII_DA_MASK) |
		   ((mdio_port << MII_ADDR_SHIFT) & MII_ADDR_MASK) |
		   ((reg << MII_REG_SHIFT) & MII_REG_MASK);
	mii_data |= ((MII_CLK_400_450M << MII_CLK_SHIFT) & MII_CLK_MASK) |
		    ((MII_CMD_WRITE << MII_CMD_SHIFT) & MII_CMD_MASK);

	writel(mii_data, iobase + MAC_MDIO_DATA);
	writel(mii_addr, iobase + MAC_MDIO_ADDR);

	start = get_timer(0);
	while (get_timer(start) < MDIO_TIMEOUT) {
		if (!(readl(iobase + MAC_MDIO_DATA) & MII_BUSY))
			return 0;
		udelay(10);
	};

	return -EFAULT;
}
#endif

/******************************************************************************
 Initialize PHY driver
*******************************************************************************/
int xgbe_phy_init(struct dw_eth_dev *priv)
{
	u32 reg_reset;
	u32 reg_clear;
	u32 val;
	phys_addr_t gpio_offset;

	/* Only XGMAC-0 or XLGMAC can init PHY */
	if (priv->phy_lane == 0) {
		/* Set dirve strength of the pad to max */
		writel(0x0001, IOSS_PLL_CFG_BASE + 0x80);
		udelay(1000);
#if defined CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK
		/* Hawk V3 needs lower drive strength */
		writel(0x3030, IOSS_PLL_CFG_BASE + 0x88);
#else
		writel(0xF0F0, IOSS_PLL_CFG_BASE + 0x88);
#endif
		writel(0x0000, IOSS_PLL_CFG_BASE + 0x8C);
	}

	if (priv->phy_reset_gpio != NO_PHY_RESET_GPIO) {
		/* Reset phy by toggle GPIO pin. Each GPIO uses 2 bytes */
		gpio_offset = GPIO_OFFSET + (priv->phy_reset_gpio & 0xFE) * 2;
		val = readl(BSS_PERIPH_BASE + gpio_offset);
		/* Even GPIO number use lower two bytes */
		if ((priv->phy_reset_gpio & 0x1) == 0) {
			val &= 0xFFFF0000;
			reg_reset = val | 0x00001EE0;
			reg_clear = val | 0x00001EF0;
		}
		/* Odd GPIO number uses upper two bytes */
		else { // odd GPIO number
			val &= 0x0000FFFF;
			reg_reset = val | 0x1EE00000;
			reg_clear = val | 0x1EF00000;
		}
		writel(reg_reset, BSS_PERIPH_BASE + gpio_offset);
		udelay(100000);
		writel(reg_clear, BSS_PERIPH_BASE + gpio_offset);
		udelay(100000);
	}

	if (priv->eth_link == 2 && priv->phy_lane == 0)
		link2_mdio_mac_base = priv->mac_base;

	return 0;
}


#if defined CONFIG_PHY_MV88X3310
#define MV_X_UNIT		3
#define MV_H_UNIT		4
#define MV_M_UNIT		31

#define GPIO_DATA               0xF012
#define GPIO_CONTROL            0xF013
#endif

void xgbe_phy_post_config(struct dw_eth_dev *priv)
{
#if defined CONFIG_PHY_MV88X3310
	phys_addr_t iobase;
	phys_addr_t gpio_offset;
	u32 reg;
#if defined CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK
	u16 reg_val;
#endif

	if (priv->eth_link == 1)
		iobase = priv->mac_base;
	else
		iobase = link2_mdio_mac_base;

	xgbe_mdio_mux_config(priv);

	/* Enable Tx on SFP */
#if defined CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK
	/* Using PHY GPIO-0 on Hawk board */
	/* Set GPIO pin to output */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			MV_M_UNIT, GPIO_CONTROL, &reg_val);
	reg_val |= 0x1;
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			MV_M_UNIT, GPIO_CONTROL, reg_val);
	/* Set GPIO pin state to 0 to enable Tx, 1 to disable Tx */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			MV_M_UNIT, GPIO_DATA, &reg_val);
	reg_val &= 0xFE;
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			MV_M_UNIT, GPIO_DATA, reg_val);
	/* Set SoC GPIO-15 on Hawk board to enable SFP power */
	gpio_offset = GPIO_OFFSET + (GPIO_15 & 0xFE) * 2;
	reg = readl(BSS_PERIPH_BASE + gpio_offset);
	reg &= 0x0000FFFF;
	reg |= 0x00500000;
	writel(reg, BSS_PERIPH_BASE + gpio_offset);

#elif defined CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS
	/* Using SoC GPIO-20 on Pegasus board */
	gpio_offset = GPIO_OFFSET + (GPIO_20 & 0xFE) * 2;
	reg = readl(BSS_PERIPH_BASE + gpio_offset);
	reg &= 0xFFFF0000;
	reg |= 0x00001E40;
	writel(reg, BSS_PERIPH_BASE + gpio_offset);
#endif
#endif
}


/******************************************************************************
 Get supported features
*******************************************************************************/
u32 xgbe_phy_supported_features(struct dw_eth_dev *priv)
{
	int max_speed = priv->max_speed;
	u32 supported = 0;

	if (max_speed == SPEED_50000)
		supported = phy_50g_supported;
	else if (max_speed == SPEED_25000)
		supported = phy_25g_supported;
	else if (max_speed == SPEED_5000)
		supported = phy_5g_supported;
	else if (max_speed == SPEED_2500)
		supported = phy_2g5_supported;
	else if (max_speed == SPEED_1000)
		supported = phy_1g_supported;
	else
		supported = phy_10g_supported;

	return supported;
}

#define TYPE_PRBS31	0
#define TYPE_PRBS9	1

#if defined CONFIG_PHY_MV88X7121
typedef enum
{
	/* NRZ signal supported PRBS patterns starts here */
	MV_PRBS31,   /* 0000b */
	MV_PRBS7,    /* 0001b */
	MV_PRBS9,    /* 0010b */
	MV_PRBS23,   /* 0011b */
	MV_IPRBS31,  /* 0100b */
	MV_IPRBS7,   /* 0101b */
	MV_IPRBS9,   /* 0110b */
	MV_IPRBS23,  /* 0111b */
	MV_PRBS15,   /* 1000b */
	MV_IPRBS15,  /* 1001b */
	MV_PRBS58,   /* 1010b */
	MV_IPRBS58,  /* 1011b */
	MV_PRBS13,   /* 1100b */
	MV_IPRBS13,  /* 1101b */
	MV_JP03AB,   /* 1110b */
	MV_GEN_TX,   /* 1111b */

	/* PAM-4 50G/100G supported PRBS patterns start here */
	MV_PAM4_PRBS31Q = 100,
	MV_PAM4_PRBS13Q,
	MV_PAM4_PRBS15Q,
	MV_PAM4_JP03A,
	MV_PAM4_JP03B,
	MV_PAM4_SSPRQ	/* Short Stress Pattern Random - Quaternary;
			   PAM-4 50G PCS only */
} prbs_selector_type;

#define MV_PMA_DEV		1
#define MV_LINE_SIDE		3
#define MV_HOST_SIDE		4
#define MV_CHIP_REG		31

#define MV_PRBS_NONE		0
#define PRBS_CONTROL		0xF200
#define PRBS0_TEST_PATTA	0xF20A
#define PRBS0_TEST_PATTB	0xF20B
#define PRBS_LANE_OFFSET	0x10
#define IEEE_LINE_PRBS_CONTROL	0x05DD
#define IEEE_HOST_PRBS_CONTROL	0x15DD
#define PRBS_PAM4_LANE_OFFSET	0x2000
#define PRBS_SYM_TXCTR1		0xF201
#define PRBS_SYM_RXCTR1		0xF204
#define PRBS_SYM_ERRCTR1	0xF207

#define MV_P10LN		10  // 1 lane PCS mode
#define MV_P25LN		18  // 1 lane PCS mode
#define MV_P25LR		20  // 1 lane PCS mode
#define MV_P50UP		60  // 1 lane PCS mode

int mv88x7121_prbs_start(struct dw_eth_dev *priv, u16 host_or_line,
			 u16 prbs_type, long duration)
{
	u16 host_mode;
	u16 line_mode;
	int wait_loop;
	u16 sig_detect = 0;
	u16 dsp_lock = 0;
	u64 tx_bit_count;
	u64 rx_bit_count;
	u64 rx_bit_error_count;
	long loop_count;
	u16 reg;
	u16 reg_val;
	char prbs_str[16];
	phys_addr_t iobase;
	int i;

	if (prbs_type == TYPE_PRBS31)
		prbs_type = MV_PRBS31;
	else if (prbs_type == TYPE_PRBS9)
		prbs_type = MV_PRBS9;

	if (priv->eth_link == 1) {
		if (priv->max_speed == SPEED_50000) {
			host_mode = MV_P50UP;
			line_mode = MV_P50UP;
			prbs_type = MV_PAM4_PRBS31Q;
			printf("Link 1 50G\n");
		}
		else if (priv->max_speed == SPEED_25000) {
			host_mode = MV_P25LN;
			line_mode = MV_P25LR;
			printf("Link 1 25G\n");
		}
		else if (priv->max_speed == SPEED_10000) {
			host_mode = MV_P10LN;
			line_mode = MV_P10LN;
			printf("Link 1 10G\n");
		}
		else {
			printf("Invalid link/speed combination\n");
			return -1;
		}
		iobase = priv->mac_base;
	}
	else if (priv->eth_link == 2) {
		if (priv->max_speed == SPEED_25000) {
			host_mode = MV_P25LN;
			line_mode = MV_P25LR;
			printf("Link 2 Lane %d 25G\n", priv->phy_lane);
		}
		else if (priv->max_speed == SPEED_10000) {
			host_mode = MV_P10LN;
			line_mode = MV_P10LN;
			printf("Link 2 Lane %d 10G\n", priv->phy_lane);
		}
		else {
			printf ("Invalid link/speed combination\n");
			return -1;
		}
		iobase = link2_mdio_mac_base;
	}
	else {
		printf("Invalid link %d\n", priv->eth_link);
		return -EINVAL;
	}

	xgbe_mdio_mux_config(priv);

	/* Set the Tx and Rx to use the matching PRBS pattern. */
	if (prbs_type == MV_PRBS31)
		strcpy(prbs_str, "prbs31");
	else if (prbs_type == MV_PRBS9)
		strcpy(prbs_str, "prbs9");
	else if (prbs_type == MV_PAM4_PRBS31Q)
		strcpy(prbs_str, "pam4-prbs31q");
	else
		strcpy(prbs_str, "Unknown");
	printf("Setting %s side %s test\n",
	       (host_or_line == MV_HOST_SIDE) ? "host" : "line", prbs_str);

	if (prbs_type <= MV_GEN_TX) {
		/* Following prbs types needs subtype */
		switch (prbs_type) {
		case MV_JP03AB:
		case MV_GEN_TX:
			printf("Pattern not supported (0x%x)\n", prbs_type);
			return -EINVAL;
		default:
			/* Nothing to do for other types */
			break;
		}
	}
	else {
		/* Clear PRBS13Q pattern setting */
		if (host_or_line == MV_HOST_SIDE)
			reg = 0x15E8;
		else
			reg = 0x05E8 + (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, 0x0);

		/* Clear all others PRBS patterns setting */
		if (host_or_line == MV_HOST_SIDE)
			reg = IEEE_HOST_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		else
			reg = IEEE_LINE_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, 0x0);

		switch (prbs_type) {
		case MV_PAM4_PRBS31Q:
			xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
					MV_PMA_DEV, reg, 0x2000);
			break;
		default:
			printf("Pattern not supported (0x%x)\n", prbs_type);
			return -EINVAL;
		}
	}

	/* Set to wait for locking before counting */
	reg = PRBS_CONTROL + (priv->phy_lane * PRBS_LANE_OFFSET);
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			host_or_line, reg, &reg_val);
	reg_val &= ~BIT(7);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			host_or_line, reg, reg_val);

	/* Enable sending PRBS pattern/receives on line side */
	if (prbs_type <= MV_GEN_TX) {
		/* mv88x7121 PRBS31 does not match Viavi tester PRBS31,
		 * but the Inverted PRBS31 (IPRBS31) matches Viavi PRBS31.
		 * For now, use inverted pattern on line side. */
		if (prbs_type == MV_PRBS31 && host_or_line == MV_LINE_SIDE)
			prbs_type = MV_IPRBS31;
		else if (prbs_type == MV_PRBS9 && host_or_line == MV_LINE_SIDE)
			prbs_type = MV_IPRBS9;

		/* Must enable Rx/Tx and set prbs pattern at same time */
		reg = PRBS_CONTROL + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				host_or_line, reg, &reg_val);
		reg_val &= ~GENMASK(3, 0);
		reg_val |= prbs_type;
		reg_val |= BIT(4);
		reg_val |= BIT(5);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				host_or_line, reg, reg_val);
	}
	else if (prbs_type == MV_PAM4_PRBS31Q) {
		if (host_or_line == MV_HOST_SIDE)
			reg = IEEE_HOST_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		else
			reg = IEEE_LINE_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, &reg_val);
		reg_val |= BIT(0);
		reg_val |= BIT(3);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, reg_val);
	}
	else {
		printf("Pattern not supported (0x%x)\n", prbs_type);
		return -EINVAL;
	}

	/* Wait for getting PRBS lock */
	wait_loop = 100;
	for (i = 0; i < wait_loop; i++) {
		reg = PRBS_CONTROL + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				host_or_line, reg, &reg_val);
		if (reg_val & BIT(8)) {
			printf("PRBS locked in %u milliseconds\n", i*100);
			break;
		}
		udelay(100000);
	}
	if (i >= wait_loop) {
		printf("PRBS locking failed\n");
	}

	reg = 0xF180 + (priv->phy_lane * 0x20);
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			host_or_line, reg, &reg_val);
	if (reg_val & BIT(8))
		sig_detect = 1;
	if (reg_val & BIT(6))
		dsp_lock = 1;
	printf ("Signal Detect:%d DSP Lock:%d\n", sig_detect, dsp_lock);

	/* reset the PRBS counter */
	reg = PRBS_CONTROL + (priv->phy_lane * PRBS_LANE_OFFSET);
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			host_or_line, reg, &reg_val);
	reg_val |= BIT(6);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			host_or_line, reg, reg_val);

	/* Run the PRBS test for 10 secs; sample only. The actual PRBS test
	   should be much longer to gather more accurate data */
	printf("Running PRBS pattern test. Total duration %ld seconds ...\n",
	        duration);

	loop_count = duration;
	while (loop_count > 0) {
		/* Get Tx bit count */
		reg = PRBS_SYM_TXCTR1 + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg, &reg_val);
		tx_bit_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+1, &reg_val);
		tx_bit_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+2, &reg_val);
		tx_bit_count += ((u64)reg_val << 32);

		/* Get Rx bit count */
		reg = PRBS_SYM_RXCTR1 + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg, &reg_val);
		rx_bit_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+1, &reg_val);
		rx_bit_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+2, &reg_val);
		rx_bit_count += ((u64)reg_val << 32);

		/* Get Rx bit error count */
		reg = PRBS_SYM_ERRCTR1 + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg, &reg_val);
		rx_bit_error_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+1, &reg_val);
		rx_bit_error_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, reg+2, &reg_val);
		rx_bit_error_count += ((u64)reg_val << 32);

		printf("%ld tx_bit_count:%llu rx_bit_count:%llu rx_bit_error_count:%llu\n",
		       duration-loop_count+1, tx_bit_count, rx_bit_count,
		       rx_bit_error_count);

		udelay(1000000);
		loop_count--;
	}

	return 0;
}

int mv88x7121_prbs_stop(struct dw_eth_dev *priv, u16 host_or_line,
			u16 prbs_type)
{
	phys_addr_t iobase;
	u16 reg;
	u16 reg_val;

	if (prbs_type == TYPE_PRBS31)
		prbs_type = MV_PRBS31;
	else if (prbs_type == TYPE_PRBS9)
		prbs_type = MV_PRBS9;

	if (priv->eth_link == 1) {
		if (priv->max_speed == SPEED_50000) {
			prbs_type = MV_PAM4_PRBS31Q;
		}
		else if (priv->max_speed == SPEED_25000) {
			/* Use specified type */
		}
		else if (priv->max_speed == SPEED_10000) {
			/* Use specified type */
		}
		else {
			printf ("Invalid link/speed combination\n");
			return -1;
		}
		iobase = priv->mac_base;
	}
	else if (priv->eth_link == 2) {
		/* Use specified type */
		iobase = link2_mdio_mac_base;
	}
	else {
		printf("Invalid link %d\n", priv->eth_link);
		return -EINVAL;
	}

	xgbe_mdio_mux_config(priv);

	if (prbs_type <= MV_GEN_TX) {
		reg = PRBS_CONTROL + (priv->phy_lane * PRBS_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				host_or_line, reg, &reg_val);
		reg_val &= ~BIT(4);
		reg_val &= ~BIT(5);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				host_or_line, reg, reg_val);
	}
	else if (prbs_type == MV_PAM4_PRBS31Q) {
		if (host_or_line == MV_HOST_SIDE)
			reg = IEEE_HOST_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		else
			reg = IEEE_LINE_PRBS_CONTROL +
				 (priv->phy_lane * PRBS_PAM4_LANE_OFFSET);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, &reg_val);
		reg_val &= ~BIT(0);
		reg_val &=~ BIT(3);
		xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
				MV_PMA_DEV, reg, reg_val);
	}
	else {
		printf("Pattern not supported (0x%x)\n", prbs_type);
		return -EINVAL;
	}

	return 0;
}

#elif defined CONFIG_PHY_MV88X3310
#define MV_PRBS31		0x0000
#define MV_PRBS9		0x0002
#define MV_IPRBS31		0x0004
#define MV_IPRBS9		0x0006
#define PRBS_CONTROL		0xF030
#define PRBS_SYM_TXCTR1		0xF031
#define PRBS_SYM_TXCTR2		0xF032
#define PRBS_SYM_TXCTR3		0xF033
#define PRBS_SYM_RXCTR1		0xF034
#define PRBS_SYM_RXCTR2		0xF035
#define PRBS_SYM_RXCTR3		0xF036
#define PRBS_SYM_ERRCTR1	0xF037
#define PRBS_SYM_ERRCTR2	0xF038
#define PRBS_SYM_ERRCTR3	0xF039
#define PRBS_RXTX_CTRL		0xF074
int mv88x3310_prbs_start(struct dw_eth_dev *priv, u16 host_or_line,
			 u16 prbs_type, long duration)
{
	u16 reg_val;
	int wait_loop;
	u64 tx_bit_count;
	u64 rx_bit_count;
	u64 rx_bit_error_count;
	long loop_count;
	char prbs_str[16];
	phys_addr_t iobase;
	int i;

	if (prbs_type == TYPE_PRBS31)
		prbs_type = MV_PRBS31;
	else if (prbs_type == TYPE_PRBS9)
		prbs_type = MV_PRBS9;

	if (host_or_line == MV_H_UNIT) {
		printf("Host side PRBS is not supported\n");
		return 0;
	}
	printf("Link %d Lane %d\n", priv->eth_link, priv->phy_lane);

	/* Set the Tx and Rx to use the matching PRBS pattern. */
	if (prbs_type == MV_PRBS31)
		strcpy(prbs_str, "prbs31");
	else if (prbs_type == MV_PRBS9)
		strcpy(prbs_str, "prbs9");
	else
		strcpy(prbs_str, "Unknown");
	printf("Setting %s side %s test\n",
	       (host_or_line == MV_H_UNIT) ? "host" : "line", prbs_str);

	if (priv->eth_link == 1)
		iobase = priv->mac_base;
	else
		iobase = link2_mdio_mac_base;

	xgbe_mdio_mux_config(priv);

	/* Disable bit-13 of 0xF074 as stated in errata */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			 host_or_line, PRBS_RXTX_CTRL, &reg_val);
	reg_val &= ~BIT(13);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			 host_or_line, PRBS_RXTX_CTRL, reg_val);

	if (prbs_type == MV_PRBS31 && host_or_line == MV_H_UNIT)
		prbs_type = MV_IPRBS31;

	/* Must enable Rx/Tx and set prbs pattern at same time */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, &reg_val);
	reg_val &= ~GENMASK(3, 0);
	/* PRBS pattern */
	reg_val |= prbs_type;
	/* No disable_wait_for_lock */
	reg_val &= ~BIT(7);
	/* Enable Rx and Tx */
	reg_val |= BIT(4);
	reg_val |= BIT(5);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, reg_val);

	/* Wait for getting PRBS lock */
	wait_loop = 100;
	for (i = 0; i < wait_loop; i++) {
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_CONTROL, &reg_val);
		if (reg_val & BIT(8)) {
			printf("PRBS locked in %u milliseconds\n", i*100);
			break;
		}
		udelay(100000);
	}
	if (i >= wait_loop) {
		printf("PRBS locking failed\n");
	}

	/* reset the PRBS counter */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, &reg_val);
	reg_val |= BIT(6);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, reg_val);

	/* Run the PRBS test for 10 secs; sample only. The actual PRBS test
	   should be much longer to gather more accurate data */
	printf("Running PRBS pattern test. Total duration %ld seconds ...\n",
	       duration);

	loop_count = duration;
	while (loop_count > 0) {
		/* Get Tx bit count */
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_TXCTR1, &reg_val);
		tx_bit_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_TXCTR2, &reg_val);
		tx_bit_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_TXCTR3, &reg_val);
		tx_bit_count += ((u64)reg_val << 32);

		/* Get Rx bit count */
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_RXCTR1, &reg_val);
		rx_bit_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_RXCTR2, &reg_val);
		rx_bit_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_RXCTR3, &reg_val);
		rx_bit_count += ((u64)reg_val << 32);

		/* Get Rx bit error count */
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_ERRCTR1, &reg_val);
		rx_bit_error_count = reg_val;
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_ERRCTR2, &reg_val);
		rx_bit_error_count += ((u64)reg_val << 16);
		xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
				 host_or_line, PRBS_SYM_ERRCTR3, &reg_val);
		rx_bit_error_count += ((u64)reg_val << 32);

		printf("%ld tx_bit_count:%llu rx_bit_count:%llu rx_bit_error_count:%llu\n",
		       duration-loop_count+1, tx_bit_count, rx_bit_count,
		       rx_bit_error_count);

		udelay(1000000);
		loop_count--;
	}

	return 0;
}

int mv88x3310_prbs_stop(struct dw_eth_dev *priv, u16 host_or_line,
			u16 prbs_type)
{
	phys_addr_t iobase;
	u16 reg_val;

	if (priv->eth_link == 1)
		iobase = priv->mac_base;
	else
		iobase = link2_mdio_mac_base;

	xgbe_mdio_mux_config(priv);

	/* Enable bit-13 of 0xF074 as stated in errata */
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			 host_or_line, PRBS_RXTX_CTRL, &reg_val);
	reg_val |= BIT(13);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			 host_or_line, PRBS_RXTX_CTRL, reg_val);
	xgbe_xgmac_mdio_read(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, &reg_val);
	reg_val &= ~BIT(4);
	reg_val &= ~BIT(5);
	xgbe_xgmac_mdio_write(iobase, priv->phy_addr,
			 host_or_line, PRBS_CONTROL, reg_val);

	return 0;
}
#endif

#if defined CONFIG_PHY_TIDS250DF230
#define RTM_ADDR		0x18
#define I2C_MUX_ADDR		0x70
#define RTM_MUX_CHAN		0x10
#define RTM_HOST		0
#define RTM_LINE		1
#define RTM_PRBS9		0x01
#define RTM_PRBS31		0x11

static int rtm_select_ch(struct udevice *dev, u8 chan, u8 en_ch_smb)
{
	u8 value;

	if (dm_i2c_read(dev, 0xFC, &value, 1))
		return -EFAULT;
	value &= ~GENMASK(7, 0);
	value |= (1 << chan);
	if (dm_i2c_write(dev, 0xFC, &value, 1))
		return -EFAULT;

	if (dm_i2c_read(dev, 0xFF, &value, 1))
		return -EFAULT;
	value &= ~BIT(0);
	value |= en_ch_smb;
	if (dm_i2c_write(dev, 0xFF, &value, 1))
		return -EFAULT;

	return 0;
}

static int rtm_prbs_set_pattern(struct udevice *dev, u8 prbs_type)
{
	u8 value;

	if (dm_i2c_read(dev, 0x2E, &value, 1))
		return -EFAULT;
	value &= ~BIT(2);
	value |= ((prbs_type >> 2) & BIT(2));
	if (dm_i2c_write(dev, 0x2E, &value, 1))
		return -EFAULT;

	if (dm_i2c_read(dev, 0x30, &value, 1))
		return -EFAULT;
	value &= ~GENMASK(1, 0);
	value |= (prbs_type & GENMASK(1, 0));
	if (dm_i2c_write(dev, 0x30, &value, 1))
		return -EFAULT;

	return 0;
}

static int rtm_prbs_checker_enable(struct udevice *dev, bool enable)
{
	u8 value;

	if (enable) {
		if (dm_i2c_read(dev, 0x0D, &value, 1))
			return -EFAULT;
		value &= ~BIT(7);
		if (dm_i2c_write(dev, 0x0D, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x79, &value, 1))
			return -EFAULT;
		value |= BIT(6);
		if (dm_i2c_write(dev, 0x79, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x30, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;
		value |= BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x30, &value, 1))
			return -EFAULT;
		value |= BIT(4);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;
		value &= ~BIT(4);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x82, &value, 1))
			return -EFAULT;
		value |= BIT(6);
		if (dm_i2c_write(dev, 0x82, &value, 1))
			return -EFAULT;
		value &= ~BIT(6);
		if (dm_i2c_write(dev, 0x82, &value, 1))
			return -EFAULT;
	}
	else {
		if (dm_i2c_read(dev, 0x0D, &value, 1))
			return -EFAULT;
		value |= BIT(7);
		if (dm_i2c_write(dev, 0x0D, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x79, &value, 1))
			return -EFAULT;
		value &= ~BIT(6);
		if (dm_i2c_write(dev, 0x79, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x30, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;
	}

	return 0;
}

static int rtm_prbs_gen_enable(struct udevice *dev, bool enable)
{
	u8 value;

	if (enable) {
		if (dm_i2c_read(dev, 0x1E, &value, 1))
			return -EFAULT;
		value |= BIT(4);
		if (dm_i2c_write(dev, 0x1E, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x79, &value, 1))
			return -EFAULT;
		value &= ~BIT(5);
		if (dm_i2c_write(dev, 0x79, &value, 1))
			return -EFAULT;
		value |= BIT(5);
		if (dm_i2c_write(dev, 0x79, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x30, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;
		value |= BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0xA5, &value, 1))
			return -EFAULT;
		value &= ~GENMASK(7, 5);
		value |= (0x4 << 5);
		if (dm_i2c_write(dev, 0xA5, &value, 1))
			return -EFAULT;
	}
	else {
		if (dm_i2c_read(dev, 0x1E, &value, 1))
			return -EFAULT;
		value &= ~BIT(4);
		if (dm_i2c_write(dev, 0x1E, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x79, &value, 1))
			return -EFAULT;
		value &= ~BIT(5);
		if (dm_i2c_write(dev, 0x79, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0x30, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		if (dm_i2c_write(dev, 0x30, &value, 1))
			return -EFAULT;

		if (dm_i2c_read(dev, 0xA5, &value, 1))
			return -EFAULT;
		value &= ~GENMASK(7, 5);
		value |= (0x1 << 5);
		if (dm_i2c_write(dev, 0xA5, &value, 1))
			return -EFAULT;
	}

	return 0;
}

static int rtm_prbs_clear_err_cnt(struct udevice *dev)
{
	u8 value;

	if (dm_i2c_read(dev, 0x82, &value, 1))
		return -EFAULT;
	value |= BIT(6);
	if (dm_i2c_write(dev, 0x82, &value, 1))
		return -EFAULT;
	value &= ~BIT(6);
	if (dm_i2c_write(dev, 0x82, &value, 1))
		return -EFAULT;

	return 0;
}

static int rtm_prbs_read_err_cnt(struct udevice *dev, u16 *err_cnt)
{
	u8 value;
	u8 value2;

	*err_cnt = 0;

	if (dm_i2c_read(dev, 0x82, &value, 1))
		return -EFAULT;
	value |= BIT(7);
	if (dm_i2c_write(dev, 0x82, &value, 1))
		return -EFAULT;

	if (dm_i2c_read(dev, 0x83, &value, 1))
		return -EFAULT;
	if (dm_i2c_read(dev, 0x84, &value2, 1))
		return -EFAULT;
	*err_cnt = (((u16)value & 0x3) << 8) + value2;

	return 0;
}

int tids250df230_prbs_start(struct dw_eth_dev *priv, u16 host_or_line,
			    u16 prbs_type, long duration)
{
	struct udevice *i2c_bus;
	struct udevice *mux_dev;
	struct udevice *rtm_dev;
	u8 saved_mux_chan;
	u8 mux_chan = RTM_MUX_CHAN;
	u8 reg_val;
	int wait_loop;
	u16 bit_err_cnt;
	long loop_count;
	char prbs_str[16];
	int i;

	if (prbs_type == TYPE_PRBS31)
		prbs_type = RTM_PRBS31;
	else if (prbs_type == TYPE_PRBS9)
		prbs_type = RTM_PRBS9;

	printf("Link %d Lane %d\n", priv->eth_link, priv->phy_lane);

	/* Set the Tx and Rx to use the matching PRBS pattern. */
	if (prbs_type == RTM_PRBS31)
		strcpy(prbs_str, "prbs31");
	else if (prbs_type == RTM_PRBS9)
		strcpy(prbs_str, "prbs9");
	else
		strcpy(prbs_str, "Unknown");
	printf("Setting %s side %s test\n",
	       (host_or_line == RTM_HOST) ? "host" : "line", prbs_str);

	if (uclass_get_device(UCLASS_I2C, 0, &i2c_bus))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, I2C_MUX_ADDR, 1, &mux_dev))
		return -EFAULT;

	if (dm_i2c_read(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	if (dm_i2c_write(mux_dev, 0, &mux_chan, 1))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, RTM_ADDR, 1, &rtm_dev))
		return -EFAULT;

	if (host_or_line == RTM_HOST) {
		/* Host side use channel 1 for Tx */
		if (rtm_select_ch(rtm_dev, 1, 1))
			return -EFAULT;
	}
	else {
		/* Line side use channel 0 for Tx */
		if (rtm_select_ch(rtm_dev, 0, 1))
			return -EFAULT;
	}

	if (rtm_prbs_set_pattern(rtm_dev, prbs_type))
		return -EFAULT;

	if (rtm_prbs_gen_enable(rtm_dev, true))
		return -EFAULT;

	if (host_or_line == RTM_HOST) {
		/* Host side use channel 0 for Rx */
		if (rtm_select_ch(rtm_dev, 0, 1))
			return -EFAULT;
	}
	else {
		/* Line side use channel 1 for Rx */
		if (rtm_select_ch(rtm_dev, 1, 1))
			return -EFAULT;
	}

	if (rtm_prbs_checker_enable(rtm_dev, true))
		return -EFAULT;

	/* Wait for getting PRBS lock */
	wait_loop = 100;
	for (i = 0; i < wait_loop; i++) {
		if (dm_i2c_read(rtm_dev, 0x01, &reg_val, 1))
			return -EFAULT;
		if (reg_val & BIT(4)) {
			printf("PRBS locked in %u milliseconds\n", i*100);
			break;
		}
		udelay(100000);
	}
	if (i >= wait_loop) {
		printf("PRBS locking failed\n");
	}

	/* Reset the PRBS counter */
	rtm_prbs_read_err_cnt(rtm_dev, &bit_err_cnt);
	if (rtm_prbs_clear_err_cnt(rtm_dev))
		return -EFAULT;

	/* Run the PRBS test for 10 secs; sample only. The actual PRBS test
	   should be much longer to gather more accurate data */
	printf("Running PRBS pattern test. Total duration %ld seconds ...\n",
	       duration);

	loop_count = duration;
	while (loop_count > 0) {
		/* Get Rx bit error count */
		if (rtm_prbs_read_err_cnt(rtm_dev, &bit_err_cnt))
			return -EFAULT;
		printf("rx_bit_error_count: %u\n", bit_err_cnt);

		udelay(1000000);
		loop_count--;
	}

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}

int tids250df230_prbs_stop(struct dw_eth_dev *priv, u16 host_or_line,
			   u16 prbs_type)
{
	struct udevice *i2c_bus;
	struct udevice *mux_dev;
	struct udevice *rtm_dev;
	u8 saved_mux_chan;
	u8 mux_chan = RTM_MUX_CHAN;

	if (uclass_get_device(UCLASS_I2C, 0, &i2c_bus))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, I2C_MUX_ADDR, 1, &mux_dev))
		return -EFAULT;

	if (dm_i2c_read(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	if (dm_i2c_write(mux_dev, 0, &mux_chan, 1))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, RTM_ADDR, 1, &rtm_dev))
		return -EFAULT;

	if (host_or_line == RTM_HOST) {
		if (rtm_select_ch(rtm_dev, 0, 1))
			return -EFAULT;
	}
	else {
		if (rtm_select_ch(rtm_dev, 1, 1))
			return -EFAULT;
	}

	if (rtm_prbs_checker_enable(rtm_dev, false))
		return -EFAULT;

	if (host_or_line == RTM_HOST) {
		if (rtm_select_ch(rtm_dev, 1, 1))
			return -EFAULT;
	}
	else {
		if (rtm_select_ch(rtm_dev, 0, 1))
			return -EFAULT;
	}
	if (rtm_prbs_gen_enable(rtm_dev, false))
		return -EFAULT;

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}
#endif

static int do_prbs(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
#ifdef CONFIG_DM_ETH_PHY
	const char* ethact_name;
	struct udevice *dev;
	struct dw_eth_dev *priv;
	u16 host_prbs;
	u16 prbs_type;
	long test_duration;

	if (argc < 4) {
		cmd_usage(cmdtp);
		return 1;
	}

	ethact_name = env_get("ethact");
	if (ethact_name == NULL)
		ethact_name = eth_get_name();
	if (ethact_name != NULL) {
		dev = eth_get_dev_by_name(ethact_name);
		if (dev == NULL) {
			printf("Cannot get device for %s\n", ethact_name);
			return 1;
		}
	}
	else {
		printf("Cannot get device for ethact\n");
		return 1;
	}
	priv = dev_get_priv(dev);

	if (strcmp("line", argv[1]) == 0)
		host_prbs = 0;
	else if (strcmp("host", argv[1]) == 0)
		host_prbs = 1;
	else {
		printf("Unsupported %s side\n", argv[1]);
		cmd_usage(cmdtp);
		return 1;
	}

	if (strcmp("prbs31", argv[2]) == 0 ||
	    strcmp("PRBS31", argv[2]) == 0) {
		prbs_type = TYPE_PRBS31;
	}
	else if (strcmp("prbs9", argv[2]) == 0 ||
	         strcmp("PRBS9", argv[2]) == 0) {
		if (host_prbs == 1) {
			printf("Host side only support prbs31\n");
			return 1;
		}
		prbs_type = TYPE_PRBS9;
	}
	else {
		printf("Unsupported prbs pattern %s\n", argv[2]);
		cmd_usage(cmdtp);
		return 1;
	}

	test_duration = simple_strtol(argv[3], NULL, 10);
	if (test_duration < 1) {
		printf("Invalid duration %s\n", argv[3]);
		cmd_usage(cmdtp);
		return 1;
	}

#if defined CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS
	/* On Pegasus, RealTek PHY does not support prbs */
	if (priv->phy_lane == 0) {
		printf("PRBS is not supported on lane 0\n");
		return 0;
	}
#endif

#if defined CONFIG_PHY_MV88X7121 || defined CONFIG_PHY_MV88X3310 || \
    defined CONFIG_PHY_TIDS250DF230
	/* Sequence to run e32 prbs and PHY host side prbs:
	 * 1) Enable e32 prbs
	 * 2) Start PHY host side prbs and collects error counter
	 * 3) Collect e32 prbs error for 1 second and disable e32 prbs
	 * 4) Stop PHY host side prbs */
	if (host_prbs) {
#if defined CONFIG_PHY_MV88X3310
		if (!priv->use_i2c) {
			printf("Host side PRBS is not supported\n");
			return 0;
		}
#endif
		/* Enable rx2tx loopback for host side prbs */
		xgbe_serdes_rx2tx_lb(priv, 1);
		udelay(100);
#if defined CONFIG_PHY_MV88X7121
		mv88x7121_prbs_start(priv, MV_HOST_SIDE, prbs_type,
				     test_duration);
#elif defined CONFIG_PHY_MV88X3310 && defined CONFIG_PHY_TIDS250DF230
		if (priv->use_i2c)
			tids250df230_prbs_start(priv, RTM_HOST, prbs_type,
						test_duration);
		else
			mv88x3310_prbs_start(priv, MV_H_UNIT, prbs_type,
					     test_duration);
#elif defined CONFIG_PHY_MV88X3310
		mv88x3310_prbs_start(priv, MV_H_UNIT, prbs_type,
				     test_duration);
#endif
		xgbe_serdes_rx2tx_lb(priv, 0);

#if defined CONFIG_PHY_MV88X7121
		mv88x7121_prbs_stop(priv, MV_HOST_SIDE, prbs_type);
#elif defined CONFIG_PHY_MV88X3310 && defined CONFIG_PHY_TIDS250DF230
		if (priv->use_i2c)
			tids250df230_prbs_stop(priv, RTM_HOST, prbs_type);
		else
			mv88x3310_prbs_stop(priv, MV_H_UNIT, prbs_type);
#elif defined CONFIG_PHY_MV88X3310
		mv88x3310_prbs_stop(priv, MV_H_UNIT, prbs_type);
#endif
	}
	else {
#if defined CONFIG_PHY_MV88X7121
		mv88x7121_prbs_start(priv, MV_LINE_SIDE, prbs_type,
				     test_duration);
		mv88x7121_prbs_stop(priv, MV_LINE_SIDE, prbs_type);
#elif defined CONFIG_PHY_MV88X3310 && defined CONFIG_PHY_TIDS250DF230
		if (priv->use_i2c) {
			tids250df230_prbs_start(priv, RTM_LINE, prbs_type,
						test_duration);
			tids250df230_prbs_stop(priv, RTM_LINE, prbs_type);
		}
		else {
			mv88x3310_prbs_start(priv, MV_X_UNIT, prbs_type,
					     test_duration);
			mv88x3310_prbs_stop(priv, MV_X_UNIT, prbs_type);
		}
#elif defined CONFIG_PHY_MV88X3310
		mv88x3310_prbs_start(priv, MV_X_UNIT, prbs_type,
				     test_duration);
		mv88x3310_prbs_stop(priv, MV_X_UNIT, prbs_type);
#endif
	}
#else
	printf("PRBS is not supported\n");
#endif
#endif

	return 0;
}

static int (*phy_lb_func)(struct dw_eth_dev *, u8) = NULL;

#if defined CONFIG_PHY_MV88X7121
#define MV_HOST_SODE		4
#define PORT_PCS_CNTL		0xF010
#define LOOPBACK_POS		12
int mv88x7121_lb(struct dw_eth_dev *priv, u8 enable)
{
	struct phy_device *phydev = (struct phy_device *)priv->phydev;
	u16 reg_val;

	if (!phydev) {
		printf("PHY device not found on lane %d\n", priv->phy_lane);
		return -EINVAL;
	}

	if (enable) {
		reg_val = phy_read(phydev, MV_HOST_SIDE, PORT_PCS_CNTL);
		reg_val |= (0x1 << (priv->phy_lane + LOOPBACK_POS));
		phy_write(phydev, MV_HOST_SIDE, PORT_PCS_CNTL, reg_val);
	}
	else {
		reg_val = phy_read(phydev, MV_HOST_SIDE, PORT_PCS_CNTL);
		reg_val &= ~(0x1 << (priv->phy_lane + LOOPBACK_POS));
		phy_write(phydev, MV_HOST_SIDE, PORT_PCS_CNTL, reg_val);
	}

	return 0;
}
#endif

#if defined CONFIG_PHY_MV88X3310
#define MV_X_UNIT		3
#define MV10GBR_PCS_CONTROL	0x1000
#define SERDES_CONTROL1		0xF003
int mv88x3310_lb(struct dw_eth_dev *priv, u8 enable)
{
	struct phy_device *phydev = (struct phy_device *)priv->phydev;
	u16 reg_val;

	if (!phydev) {
		printf("PHY device not found on lane %d\n", priv->phy_lane);
		return -EINVAL;
	}

	if (enable) {
		/* Enable deep MAC loopback */
		reg_val = phy_read(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL);
		reg_val |= BIT(14);
		phy_write(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL, reg_val);

		/* Enable no-passthrough */
		reg_val = phy_read(phydev, MV_X_UNIT, SERDES_CONTROL1);
		reg_val |= BIT(6);
		phy_write(phydev, MV_X_UNIT, SERDES_CONTROL1, reg_val);
	}
	else {
		/* Disable deep MAC loopback */
		reg_val = phy_read(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL);
		reg_val &= ~BIT(14);
		phy_write(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL, reg_val);

		/* Disable no-passthrough */
		reg_val = phy_read(phydev, MV_X_UNIT, SERDES_CONTROL1);
		reg_val &= ~BIT(6);
		phy_write(phydev, MV_X_UNIT, SERDES_CONTROL1, reg_val);
	}

	return 0;
}
#endif

#if defined CONFIG_PHY_TIDS250DF230
#define DS250_ADDR	      0x18
#define I2C_MUX_ADDR	      0x70
#define RTM_MUX_CHAN	      0x10
static int select_ch(struct udevice *dev, u8 chan, u8 en_ch_smb)
{
	u8 value;

	if (dm_i2c_read(dev, 0xFC, &value, 1))
		return -EFAULT;
	value &= ~GENMASK(7, 0);
	value |= (1 << chan);
	if (dm_i2c_write(dev, 0xFC, &value, 1))
		return -EFAULT;

	if (dm_i2c_read(dev, 0xFF, &value, 1))
		return -EFAULT;
	value &= ~BIT(0);
	value |= en_ch_smb;
	if (dm_i2c_write(dev, 0xFF, &value, 1))
		return -EFAULT;

	return 0;
}

static int cross_point_ch(struct udevice *dev, u8 chan, u8 enable)
{
	u8 value;

	if (enable) {
		if (dm_i2c_read(dev, 0x95, &value, 1))
			return -EFAULT;
		value |= BIT(3);
		if (dm_i2c_write(dev, 0x95, &value, 1))
			return -EFAULT;
		if (dm_i2c_read(dev, 0x96, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		value |= (BIT(2) | BIT(1) | BIT(0));
		if (dm_i2c_write(dev, 0x96, &value, 1))
			return -EFAULT;
	}
	else {
		if (dm_i2c_read(dev, 0x95, &value, 1))
			return -EFAULT;
		value &= ~BIT(3);
		if (dm_i2c_write(dev, 0x95, &value, 1))
			return -EFAULT;
		if (dm_i2c_read(dev, 0x96, &value, 1))
			return -EFAULT;
		value |= BIT(3);
		value &= ~(BIT(2) | BIT(1) | BIT(0));
		if (dm_i2c_write(dev, 0x96, &value, 1))
			return -EFAULT;
	}

	return 0;
}

int tids250df230_lb(struct dw_eth_dev *priv, u8 enable)
{
	struct udevice *i2c_bus;
	struct udevice *mux_dev;
	struct udevice *rtm_dev;
	u8 saved_mux_chan;
	u8 mux_chan = RTM_MUX_CHAN;

	if (uclass_get_device(UCLASS_I2C, 0, &i2c_bus))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, I2C_MUX_ADDR, 1, &mux_dev))
		return -EFAULT;

	if (dm_i2c_read(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	if (dm_i2c_write(mux_dev, 0, &mux_chan, 1))
		return -EFAULT;

	if (i2c_get_chip(i2c_bus, DS250_ADDR, 1, &rtm_dev))
		return -EFAULT;

	/* Host side */
	select_ch(rtm_dev, 0, 1);
	cross_point_ch(rtm_dev, 0, enable);
	/* Line side */
	select_ch(rtm_dev, 1, 1);
	cross_point_ch(rtm_dev, 1, enable);

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}
#endif

int phy_host_lb(struct dw_eth_dev *priv, u8 enable)
{
	int ret = 0;

#if defined CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS
	/* On Pegasus, RealTek PHY does not support loopback */
	if (priv->phy_lane == 0) {
		printf("PHY loopback not supported on lane 0\n");
		return -EINVAL;
	}
#endif

	if (enable) {
#if defined CONFIG_PHY_MV88X7121
		phy_lb_func = mv88x7121_lb;
#elif defined CONFIG_PHY_MV88X3310 && defined CONFIG_PHY_TIDS250DF230
		if (priv->use_i2c)
			phy_lb_func = tids250df230_lb;
		else
			phy_lb_func = mv88x3310_lb;
#elif defined CONFIG_PHY_MV88X3310
		phy_lb_func = mv88x3310_lb;
#else
		printf("Configure PHY loopback manually\n");
#endif
	}
	if (phy_lb_func)
		phy_lb_func(priv, enable);

	if (!enable)
		phy_lb_func = NULL;

	return ret;
}

static int do_loopback(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	const char *link1_name = "ethernet@3c000000";
	const char *link2_name = "ethernet@3c040000";
	const char *ethact_name;
	struct udevice *dev;
	struct dw_eth_dev *link1_priv = NULL;
	struct dw_eth_dev *link2_priv = NULL;
	struct dw_eth_dev *act_priv = NULL;
	u16 new_loopback_test;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	if (strcmp("disable", argv[1]) == 0) {
		if (loopback_test == NO_LOOPBACK) {
			printf("Loopback test not enabled\n");
			return 1;
		}
		new_loopback_test = NO_LOOPBACK;
	}
	else {
		if (loopback_test != NO_LOOPBACK) {
			printf("Stop current loopback test first\n");
			return 1;
		}
		if (strcmp("mac", argv[1]) == 0)
			new_loopback_test = MAC_LOOPBACK;
		else if (strcmp("serdes", argv[1]) == 0)
			new_loopback_test = SERDES_LOOPBACK;
		else if (strcmp("phy-host", argv[1]) == 0)
			new_loopback_test = PHY_HOST_LOOPBACK;
		else {
			printf("Unsupported loopback type %s\n", argv[1]);
			cmd_usage(cmdtp);
			return 1;
		}
	}

	/* MAC, SERDES and PHY loopback use ethact port */
	ethact_name = env_get("ethact");
	if (ethact_name == NULL)
		ethact_name = eth_get_name();
	if (ethact_name != NULL) {
		dev = eth_get_dev_by_name(ethact_name);
		if (dev == NULL) {
			printf("Cannot get device for %s\n", ethact_name);
			return 1;
		}
		act_priv = dev_get_priv(dev);
	}
	else {
		printf("Cannot get device for ethact\n");
		return 1;
	}

	/* SERDES loopback uses link2-lane0 or link1 for config */
	dev = eth_get_dev_by_name(link2_name);
	if (dev)
		link2_priv = dev_get_priv(dev);
	dev = eth_get_dev_by_name(link1_name);
	if (dev)
		link1_priv = dev_get_priv(dev);

	if (new_loopback_test == NO_LOOPBACK) {
		if (loopback_test == SERDES_LOOPBACK) {
			if (link2_priv)
				xgbe_serdes_tx2rx_lb(link2_priv, false);
			if (link1_priv)
				xgbe_serdes_tx2rx_lb(link1_priv, false);
		}
		else if (loopback_test == PHY_HOST_LOOPBACK) {
			phy_host_lb(act_priv, false);
		}
		else if (loopback_test == MAC_LOOPBACK) {
			/* MAC loopback disalbed at eth_stop */
		}
	}
	else if (new_loopback_test == SERDES_LOOPBACK) {
		if (link2_priv) {
			if (xgbe_serdes_tx2rx_lb(link2_priv, true))
				return 1;
		}
		if (link1_priv)
			if (xgbe_serdes_tx2rx_lb(link1_priv, true))
				return 1;
	}
	else if (new_loopback_test == PHY_HOST_LOOPBACK) {
		if (phy_host_lb(act_priv, true))
			return 1;
	}
	else if (new_loopback_test == MAC_LOOPBACK) {
		/* MAC loopback enabled at eth start */
	}
	loopback_test = new_loopback_test;

	return 0;
}

#if defined CONFIG_PHY_MV88X7121
#define SERDES_LINE		0x04
#define SERDES_HOST		0x05

typedef enum {
    C112GX4_TXEQ_EM_PRE3_CTRL,  /* Exists Only in R1.2 */
    C112GX4_TXEQ_EM_PRE2_CTRL,
    C112GX4_TXEQ_EM_PRE_CTRL,
    C112GX4_TXEQ_EM_MAIN_CTRL,
    C112GX4_TXEQ_EM_POST_CTRL,
    C112GX4_TXEQ_EM_NA,         /* Exists Only in R1.2 */
} ffe_type;

typedef struct _serdes_field
{
    u32 reg;
    u16 hi_bit;
    u16 lo_bit;
    u16 total_bits;
    u32 mask;
    u32 retain_mask;
} serdes_field;

#define FIELD_DEFINE(reg, hiBit, loBit) { \
    reg, \
    hiBit, \
    loBit, \
    (hiBit-loBit) + 1, \
    (u32) ((((u64) 1 << ((hiBit-loBit) + 1)) - 1) << loBit), \
    (u32) ~((((u64) 1 << ((hiBit-loBit) + 1)) - 1) << loBit)}

static serdes_field to_ana_tx_fir_c0 = FIELD_DEFINE(0x2058, 29, 24);
static serdes_field to_ana_tx_fir_c1 = FIELD_DEFINE(0x2058, 21, 16);
static serdes_field to_ana_tx_fir_c2 = FIELD_DEFINE(0x2058, 13, 8);
static serdes_field to_ana_tx_fir_c3 = FIELD_DEFINE(0x2058, 5, 0);
static serdes_field to_ana_tx_fir_c4 = FIELD_DEFINE(0x2098, 29, 24);
static serdes_field to_ana_tx_fir_c5 = FIELD_DEFINE(0x2098, 21, 16);

static serdes_field tx_fir_update   = FIELD_DEFINE(0x2054, 8, 8);
static serdes_field tx_fir_c0       = FIELD_DEFINE(0x2050, 22, 17);
static serdes_field tx_fir_c0_force = FIELD_DEFINE(0x2050, 16, 16);
static serdes_field tx_fir_c1       = FIELD_DEFINE(0x2050, 14, 9);
static serdes_field tx_fir_c1_force = FIELD_DEFINE(0x2050, 8, 8);
static serdes_field tx_fir_c2       = FIELD_DEFINE(0x2050, 5, 0);
static serdes_field tx_fir_c2_force = FIELD_DEFINE(0x2050, 6, 6);
static serdes_field tx_fir_c3       = FIELD_DEFINE(0x2054, 29, 24);
static serdes_field tx_fir_c3_force = FIELD_DEFINE(0x2054, 31, 31);
static serdes_field tx_fir_c4       = FIELD_DEFINE(0x2054, 21, 16);
static serdes_field tx_fir_c4_force = FIELD_DEFINE(0x2054, 30, 30);
static serdes_field tx_fir_c5       = FIELD_DEFINE(0x2054, 15, 10);
static serdes_field tx_fir_c5_force = FIELD_DEFINE(0x2054, 9, 9);

static serdes_field lane_sel        = FIELD_DEFINE(0xA314, 31, 29);

static u32 read_serdes_reg(phys_addr_t iobase, u16 phy_addr, u32 reg_addr)
{
	u32 value;
	u16 val;
	int wait_count = 0;

	/* Read command */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AB, 0);
	/* High addr */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AF, (reg_addr >> 16));
	/* Low addr */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AC, (reg_addr & 0xFFFF));
	/* Wait for read complete */
	val = 0;
	do {
		if (wait_count++ > 1000)
			break;
		xgbe_xgmac_mdio_read(iobase, phy_addr,
				MV_CHIP_REG, 0xF0AB, &val);
	} while ((val & 0x2) != 0x2);
	if (wait_count > 100) {
		printf("Serdes read failed\n");
		return 0;
	}
	/* High data */
	xgbe_xgmac_mdio_read(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AD, &val);
	value = val;
	/* Low data */
	xgbe_xgmac_mdio_read(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AE, &val);
	value = (value << 16) + val;

	return value;
}

static void write_serdes_reg(phys_addr_t iobase, u16 phy_addr, u32 reg_addr, u32 value)
{
	/* Write command */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AB, 1);
	/* High addr */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AF, (reg_addr >> 16));
	/* Low addr */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AC, (reg_addr & 0xFFFF));
	/* High data */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AD, (value >> 16));
	/* Low data */
	xgbe_xgmac_mdio_write(iobase, phy_addr,
			MV_CHIP_REG, 0xF0AE, (value & 0xFFFF));
}

static void write_serdes_field_direct(phys_addr_t iobase, u16 phy_addr,
				      u8 host_or_line, serdes_field *field, u32 value)
{
	u32 apb_addr;
	u32 val;

	apb_addr = (host_or_line << 20) + field->reg;

	/* Read reg */
	val = read_serdes_reg(iobase, phy_addr, apb_addr);
	/* Modify value */
	val = (val & field->retain_mask) | (value << field->lo_bit);
	/* Write value back */
	write_serdes_reg(iobase, phy_addr, apb_addr, val);
}

static u32 read_serdes_field(phys_addr_t iobase, u16 phy_addr,
			     u8 host_or_line, serdes_field *field)
{
	u32 apb_addr;
	u32 value;

	if (field->reg < 0x8000) {
		/* Set lane 0 */
		serdes_field *field_lane_sel = &lane_sel;
		write_serdes_field_direct(iobase, phy_addr,
					  host_or_line, field_lane_sel, 0);
	}

	apb_addr = (host_or_line << 20) + field->reg;

	value = read_serdes_reg(iobase, phy_addr, apb_addr);

	/* Get field value */
	value = (value & field->mask) >> field->lo_bit;

	return value;
}

static void write_serdes_field(phys_addr_t iobase, u16 phy_addr,
			       u8 host_or_line, serdes_field *field, u32 value)
{
	u32 apb_addr;
	u32 val;

	if (field->reg < 0x8000) {
		/* Set lane 0 */
		serdes_field *field_lane_sel = &lane_sel;
		write_serdes_field_direct(iobase, phy_addr,
					  host_or_line, field_lane_sel, 0);
	}

	apb_addr = (host_or_line << 20) + field->reg;

	/* Read reg */
	val = read_serdes_reg(iobase, phy_addr, apb_addr);
	/* Modify value */
	val = (val & field->retain_mask) | (value << field->lo_bit);
	/* Write value back */
	write_serdes_reg(iobase, phy_addr, apb_addr, val);
}

static int get_tx_ffe(struct dw_eth_dev *priv, int type, u32 *val)
{
	serdes_field *field;

	switch (type) {
	case C112GX4_TXEQ_EM_PRE3_CTRL:
		field = &to_ana_tx_fir_c0;
		break;
	case C112GX4_TXEQ_EM_PRE2_CTRL:
		field = &to_ana_tx_fir_c1;
		break;
	case C112GX4_TXEQ_EM_PRE_CTRL:
		field = &to_ana_tx_fir_c2;
		break;
	case C112GX4_TXEQ_EM_MAIN_CTRL:
		field = &to_ana_tx_fir_c3;
		break;
	case C112GX4_TXEQ_EM_POST_CTRL:
		field = &to_ana_tx_fir_c4;
		break;
	case C112GX4_TXEQ_EM_NA:
		field = &to_ana_tx_fir_c5;
		break;
	default:
		printf("Invalid ffe type %d\n", type);
		return -1;
	}

	xgbe_mdio_mux_config(priv);

	*val = read_serdes_field(priv->mac_base, priv->phy_addr,
				 SERDES_HOST, field);

	return 0;
}

void check_tx_ffe(struct dw_eth_dev *priv)
{
	u32 c0_val, c1_val, c2_val, c3_val, c4_val, c5_val;
	int ret = 0;

	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_PRE3_CTRL, &c0_val);
	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_PRE2_CTRL, &c1_val);
	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_PRE_CTRL, &c2_val);
	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_MAIN_CTRL, &c3_val);
	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_POST_CTRL, &c4_val);
	ret |= get_tx_ffe(priv, C112GX4_TXEQ_EM_NA, &c5_val);
	if (ret < 0)
		printf("%s failed\n", __func__);
	else
		printf("Serdes Tx FFE: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",
		       c0_val, c1_val, c2_val, c3_val, c4_val, c5_val);
}

static int do_get_tx_ffe(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	const char *link1_name = "ethernet@3c000000";
	struct udevice *dev;
	struct dw_eth_dev *link1_priv = NULL;
	int type;
	u32 val;
	int ret;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	type = simple_strtol(argv[1], NULL, 10);

	dev = eth_get_dev_by_name(link1_name);
	if (dev)
		link1_priv = dev_get_priv(dev);
	else
		return 1;

	ret = get_tx_ffe(link1_priv, type, &val);
	if (ret < 0)
		return 1;

	printf("Host side FFE type %d value 0x%x\n", type, val);

	return 0;
}

#define GLOBAL_RESET	0xF404
static int set_tx_ffe(struct dw_eth_dev *priv, int type, u32 val)
{
	serdes_field *field, *field_force;

	field = &tx_fir_update;
	write_serdes_field(priv->mac_base, priv->phy_addr,
			   SERDES_HOST, field, 0);

	switch (type) {
	case C112GX4_TXEQ_EM_PRE3_CTRL:
		field = &tx_fir_c0;
		field_force = &tx_fir_c0_force;
		break;
	case C112GX4_TXEQ_EM_PRE2_CTRL:
		field = &tx_fir_c1;
		field_force = &tx_fir_c1_force;
		break;
	case C112GX4_TXEQ_EM_PRE_CTRL:
		field = &tx_fir_c2;
		field_force = &tx_fir_c2_force;
		break;
	case C112GX4_TXEQ_EM_MAIN_CTRL:
		field = &tx_fir_c3;
		field_force = &tx_fir_c3_force;
		break;
	case C112GX4_TXEQ_EM_POST_CTRL:
		field = &tx_fir_c4;
		field_force = &tx_fir_c4_force;
		break;
	case C112GX4_TXEQ_EM_NA:
		field = &tx_fir_c5;
		field_force = &tx_fir_c5_force;
		break;
	default:
		printf("Invalid ffe type %d\n", type);
		return -1;
	}

	xgbe_mdio_mux_config(priv);

	write_serdes_field(priv->mac_base, priv->phy_addr,
			   SERDES_HOST, field_force, 1);
	write_serdes_field(priv->mac_base, priv->phy_addr,
			   SERDES_HOST, field, val);

	field = &tx_fir_update;
	write_serdes_field(priv->mac_base, priv->phy_addr,
			   SERDES_HOST, field, 1);
	udelay(1000);
	write_serdes_field(priv->mac_base, priv->phy_addr,
			   SERDES_HOST, field, 0);

	/* The TX emphasis main is automatically calculated and updated
	 * unless user explicitly wants to manually set it.
	 */
	if (type != C112GX4_TXEQ_EM_MAIN_CTRL) {
		u32 pre3_data, pre2_data, pre_data, post_data, na_data;
		u32 full_swing = 63;
		u32 sum, main_data;

		field = &to_ana_tx_fir_c0;
		pre3_data = read_serdes_field(priv->mac_base,
					 priv->phy_addr, SERDES_HOST, field);
		field = &to_ana_tx_fir_c1;
		pre2_data = read_serdes_field(priv->mac_base,
					 priv->phy_addr, SERDES_HOST, field);
		field = &to_ana_tx_fir_c2;
		pre_data = read_serdes_field(priv->mac_base,
					 priv->phy_addr, SERDES_HOST, field);
		field = &to_ana_tx_fir_c4;
		post_data = read_serdes_field(priv->mac_base,
					 priv->phy_addr, SERDES_HOST, field);
		field = &to_ana_tx_fir_c5;
		na_data = read_serdes_field(priv->mac_base,
					 priv->phy_addr, SERDES_HOST, field);
		sum = pre3_data + pre2_data + pre_data + post_data + na_data;
		if (full_swing >= sum)
			main_data = full_swing - sum;
		else
			main_data = 0;

		field = &tx_fir_c3;
		field_force = &tx_fir_c3_force;
		write_serdes_field(priv->mac_base, priv->phy_addr,
				   SERDES_HOST, field_force, 1);
		write_serdes_field(priv->mac_base, priv->phy_addr,
				   SERDES_HOST, field, main_data);

		field = &tx_fir_update;
		write_serdes_field(priv->mac_base, priv->phy_addr,
				   SERDES_HOST, field, 1);
		udelay(1000);
		write_serdes_field(priv->mac_base, priv->phy_addr,
				   SERDES_HOST, field, 0);
	}

	return 0;
}

int init_tx_ffe(struct dw_eth_dev *priv)
{
	int ret = 0;

	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_PRE3_CTRL, 0x0);
	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_PRE2_CTRL, 0x0);
	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_PRE_CTRL, 0x10);
	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_POST_CTRL, 0x10);
	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_NA, 0x0);
	ret |= set_tx_ffe(priv, C112GX4_TXEQ_EM_MAIN_CTRL, 0x1f);

	return ret;
}

static int do_set_tx_ffe(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	const char *link1_name = "ethernet@3c000000";
	struct udevice *dev;
	struct dw_eth_dev *link1_priv = NULL;
	int type;
	u32 val;
	int ret;

	if (argc < 3) {
		cmd_usage(cmdtp);
		return 1;
	}

	type = simple_strtol(argv[1], NULL, 10);
	val = simple_strtol(argv[2], NULL, 16);
	if (val > 0x3f) {
		printf("Invalid value\n");
		cmd_usage(cmdtp);
		return 1;
	}

	dev = eth_get_dev_by_name(link1_name);
	if (dev)
		link1_priv = dev_get_priv(dev);
	else
		return 1;

	ret = set_tx_ffe(link1_priv, type, val);
	if (ret != 0)
		return 1;

	printf("Host side FFE type %d set to 0x%x\n", type, val);

	return 0;
}
#endif

U_BOOT_CMD(prbs, 4, 0, do_prbs,
	   "eqxgbe phy prbs test",
	   "<host | line> <pattern> <duration>");

U_BOOT_CMD(loopback, 2, 0, do_loopback,
	   "mac/serdes/phy loopback test",
	   "<mac | serdes | phy-host | disable>");

#if defined CONFIG_PHY_MV88X7121
U_BOOT_CMD(get_tx_ffe, 2, 0, do_get_tx_ffe,
	   "get phy serdes Tx ffe of link1 host side",
	   "<type 0 - 5>");

U_BOOT_CMD(set_tx_ffe, 3, 0, do_set_tx_ffe,
	   "set phy serdes Tx ffe of link1 host side",
	   "<type 0 - 5> <value 0x0 - 0x3f>");
#endif
