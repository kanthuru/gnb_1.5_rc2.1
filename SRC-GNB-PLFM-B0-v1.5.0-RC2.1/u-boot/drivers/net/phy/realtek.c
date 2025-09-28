// SPDX-License-Identifier: GPL-2.0+
/*
 * RealTek PHY drivers
 *
 * Copyright 2010-2011, 2015 Freescale Semiconductor, Inc.
 * author Andy Fleming
 * Copyright 2016 Karsten Merker <merker@debian.org>
 */
#include <common.h>
#include <linux/bitops.h>
#include <phy.h>
#include <linux/delay.h>

#define PHY_RTL8211x_FORCE_MASTER BIT(1)
#define PHY_RTL8211E_PINE64_GIGABIT_FIX BIT(2)
#define PHY_RTL8211F_FORCE_EEE_RXC_ON BIT(3)

#define PHY_AUTONEGOTIATE_TIMEOUT 5000

/* RTL8211x 1000BASE-T Control Register */
#define MIIM_RTL8211x_CTRL1000T_MSCE BIT(12);
#define MIIM_RTL8211x_CTRL1000T_MASTER BIT(11);

/* RTL8211x PHY Status Register */
#define MIIM_RTL8211x_PHY_STATUS       0x11
#define MIIM_RTL8211x_PHYSTAT_SPEED    0xc000
#define MIIM_RTL8211x_PHYSTAT_GBIT     0x8000
#define MIIM_RTL8211x_PHYSTAT_100      0x4000
#define MIIM_RTL8211x_PHYSTAT_DUPLEX   0x2000
#define MIIM_RTL8211x_PHYSTAT_SPDDONE  0x0800
#define MIIM_RTL8211x_PHYSTAT_LINK     0x0400

/* RTL8211x PHY Interrupt Enable Register */
#define MIIM_RTL8211x_PHY_INER         0x12
#define MIIM_RTL8211x_PHY_INTR_ENA     0x9f01
#define MIIM_RTL8211x_PHY_INTR_DIS     0x0000

/* RTL8211x PHY Interrupt Status Register */
#define MIIM_RTL8211x_PHY_INSR         0x13

/* RTL8211F PHY Status Register */
#define MIIM_RTL8211F_PHY_STATUS       0x1a
#define MIIM_RTL8211F_AUTONEG_ENABLE   0x1000
#define MIIM_RTL8211F_PHYSTAT_SPEED    0x0030
#define MIIM_RTL8211F_PHYSTAT_GBIT     0x0020
#define MIIM_RTL8211F_PHYSTAT_100      0x0010
#define MIIM_RTL8211F_PHYSTAT_DUPLEX   0x0008
#define MIIM_RTL8211F_PHYSTAT_SPDDONE  0x0800
#define MIIM_RTL8211F_PHYSTAT_LINK     0x0004

#define MIIM_RTL8211E_CONFREG           0x1c
#define MIIM_RTL8211E_CONFREG_TXD		0x0002
#define MIIM_RTL8211E_CONFREG_RXD		0x0004
#define MIIM_RTL8211E_CONFREG_MAGIC		0xb400	/* Undocumented */

#define MIIM_RTL8211E_EXT_PAGE_SELECT  0x1e

#define MIIM_RTL8211F_PAGE_SELECT      0x1f
#define MIIM_RTL8211F_TX_DELAY		0x100
#define MIIM_RTL8211F_RX_DELAY		0x8
#define MIIM_RTL8211F_LCR		0x10

/* RTL8221B PHY Registers */
//#define RTL8221B_OPT_3                 1
#undef  RTL8221B_OPT_3
#define MIIM_RTL8221B_SERDES_OPT       0x697a  /* MDIO_MMD_VEND1 */
#define MIIM_RTL8221B_OPT_MASK         0x3f
#define MIIM_RTL8221B_OPT_1            0x1
#define MIIM_RTL8221B_OPT_3            0x3
#define MIIM_RTL8221B_SERDES_CTRL1     0x6a04  /* MDIO_MMD_VEND1 */
#define MIIM_RTL8221B_MDIO_SEL_MASK    0x700
#define MIIM_RTL8221B_MDIO_SEL_HISGMII 0x503
#define MIIM_RTL8221B_SERDES_CTRL3     0x7580  /* MDIO_MMD_VEND1 */
#define MIIM_RTL8221B_SDS_MODE_MASK    0x1f
#define MIIM_RTL8221B_SDS_MODE_2500    0x16
#define MIIM_RTL8221B_SERDES_CTRL5     0x7582  /* MDIO_MMD_VEND1 */
#define MIIM_RTL8221B_SERDES_1000      0x0002
#define MIIM_RTL8221B_SERDES_2500      0x1001
#define MIIM_RTL8221B_PHY_FEDCR        0xa400  /* MDIO_MMD_VEND2 reg */
#define MIIM_RTL8221B_PCS_LPB          0x4000
#define MIIM_RTL8221B_PHY_STATUS       0xa434  /* MDIO_MMD_VEND2 */
#define MIIM_RTL8221B_LINK_UP          0x4
#define MIIM_RTL8221B_FULL_DUPLEX      0x8
#define MIIM_RTL8221B_SPEED_MASK       0x630
#define MIIM_RTL8221B_SPEED_2500       0x210
#define MIIM_RTL8221B_SPEED_1000       0x020
#define MIIM_RTL8221B_SPEED_100        0x010
#define MIIM_RTL8221B_PHY_INER         0xa4d2  /* MDIO_MMD_VEND2 */

static int rtl8211f_phy_extread(struct phy_device *phydev, int addr,
				int devaddr, int regnum)
{
	int oldpage = phy_read(phydev, MDIO_DEVAD_NONE,
			       MIIM_RTL8211F_PAGE_SELECT);
	int val;

	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT, devaddr);
	val = phy_read(phydev, MDIO_DEVAD_NONE, regnum);
	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT, oldpage);

	return val;
}

static int rtl8211f_phy_extwrite(struct phy_device *phydev, int addr,
				 int devaddr, int regnum, u16 val)
{
	int oldpage = phy_read(phydev, MDIO_DEVAD_NONE,
			       MIIM_RTL8211F_PAGE_SELECT);

	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT, devaddr);
	phy_write(phydev, MDIO_DEVAD_NONE, regnum, val);
	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT, oldpage);

	return 0;
}

static int rtl8211b_probe(struct phy_device *phydev)
{
#ifdef CONFIG_RTL8211X_PHY_FORCE_MASTER
	phydev->flags |= PHY_RTL8211x_FORCE_MASTER;
#endif

	return 0;
}

static int rtl8211e_probe(struct phy_device *phydev)
{
#ifdef CONFIG_RTL8211E_PINE64_GIGABIT_FIX
	phydev->flags |= PHY_RTL8211E_PINE64_GIGABIT_FIX;
#endif

	return 0;
}

static int rtl8211f_probe(struct phy_device *phydev)
{
#ifdef CONFIG_RTL8211F_PHY_FORCE_EEE_RXC_ON
	phydev->flags |= PHY_RTL8211F_FORCE_EEE_RXC_ON;
#endif

	return 0;
}

static int rtl8221b_probe(struct phy_device *phydev)
{
	return 0;
}

/* RealTek RTL8211x */
static int rtl8211x_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR, BMCR_RESET);

	/* mask interrupt at init; if the interrupt is
	 * needed indeed, it should be explicitly enabled
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211x_PHY_INER,
		  MIIM_RTL8211x_PHY_INTR_DIS);

	if (phydev->flags & PHY_RTL8211x_FORCE_MASTER) {
		unsigned int reg;

		reg = phy_read(phydev, MDIO_DEVAD_NONE, MII_CTRL1000);
		/* force manual master/slave configuration */
		reg |= MIIM_RTL8211x_CTRL1000T_MSCE;
		/* force master mode */
		reg |= MIIM_RTL8211x_CTRL1000T_MASTER;
		phy_write(phydev, MDIO_DEVAD_NONE, MII_CTRL1000, reg);
	}
	if (phydev->flags & PHY_RTL8211E_PINE64_GIGABIT_FIX) {
		unsigned int reg;

		phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT,
			  7);
		phy_write(phydev, MDIO_DEVAD_NONE,
			  MIIM_RTL8211E_EXT_PAGE_SELECT, 0xa4);
		reg = phy_read(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211E_CONFREG);
		/* Ensure both internal delays are turned off */
		reg &= ~(MIIM_RTL8211E_CONFREG_TXD | MIIM_RTL8211E_CONFREG_RXD);
		/* Flip the magic undocumented bits */
		reg |= MIIM_RTL8211E_CONFREG_MAGIC;
		phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211E_CONFREG, reg);
		phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT,
			  0);
	}
	/* read interrupt status just to clear it */
	phy_read(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211x_PHY_INER);

	genphy_config_aneg(phydev);

	return 0;
}

static int rtl8211f_config(struct phy_device *phydev)
{
	u16 reg;

	if (phydev->flags & PHY_RTL8211F_FORCE_EEE_RXC_ON) {
		unsigned int reg;

		reg = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
		reg &= ~MDIO_PCS_CTRL1_CLKSTOP_EN;
		phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1, reg);
	}

	phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR, BMCR_RESET);

	phy_write(phydev, MDIO_DEVAD_NONE,
		  MIIM_RTL8211F_PAGE_SELECT, 0xd08);
	reg = phy_read(phydev, MDIO_DEVAD_NONE, 0x11);

	/* enable TX-delay for rgmii-id and rgmii-txid, otherwise disable it */
	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
	    phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
		reg |= MIIM_RTL8211F_TX_DELAY;
	else
		reg &= ~MIIM_RTL8211F_TX_DELAY;

	phy_write(phydev, MDIO_DEVAD_NONE, 0x11, reg);

	/* enable RX-delay for rgmii-id and rgmii-rxid, otherwise disable it */
	reg = phy_read(phydev, MDIO_DEVAD_NONE, 0x15);
	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID ||
	    phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID)
		reg |= MIIM_RTL8211F_RX_DELAY;
	else
		reg &= ~MIIM_RTL8211F_RX_DELAY;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x15, reg);

	/* restore to default page 0 */
	phy_write(phydev, MDIO_DEVAD_NONE,
		  MIIM_RTL8211F_PAGE_SELECT, 0x0);

	/* Set green LED for Link, yellow LED for Active */
	phy_write(phydev, MDIO_DEVAD_NONE,
		  MIIM_RTL8211F_PAGE_SELECT, 0xd04);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x10, 0x617f);
	phy_write(phydev, MDIO_DEVAD_NONE,
		  MIIM_RTL8211F_PAGE_SELECT, 0x0);

	genphy_config_aneg(phydev);

	return 0;
}

/* RealTek RTL8221B */
static int rtl8221b_config(struct phy_device *phydev)
{
	u16 reg;
	int loop_cnt = 60;
	int i;

	phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR, BMCR_RESET);
	udelay(20000);	/* 20 ms */

	/* Disable interrupt */
	phy_write(phydev, MDIO_MMD_VEND2, MIIM_RTL8221B_PHY_INER, 0);

	/* Set MMD 30.0x75F3.0 = 0 as stated on App Note */
	reg = phy_read(phydev, MDIO_MMD_VEND1, 0x75F3);
	reg &= 0xFFFE;
	phy_write(phydev, MDIO_MMD_VEND1, 0x75F3, reg);

	/* Serdes option 2.5G mode */
	reg = phy_read(phydev, MDIO_MMD_VEND1, MIIM_RTL8221B_SERDES_OPT);
	reg &= ~MIIM_RTL8221B_OPT_MASK;
#ifdef RTL8221B_OPT_3
	reg |= MIIM_RTL8221B_OPT_3;
#else
	reg |= MIIM_RTL8221B_OPT_1;
#endif
	phy_write(phydev, MDIO_MMD_VEND1, MIIM_RTL8221B_SERDES_OPT, reg);

	/* MDIO select 2.5G mode */
	reg = phy_read(phydev, MDIO_MMD_VEND1, MIIM_RTL8221B_SERDES_CTRL1);
	reg &= ~MIIM_RTL8221B_MDIO_SEL_MASK;
	reg |= MIIM_RTL8221B_MDIO_SEL_HISGMII; // For both opt 1 and 3
	phy_write(phydev, MDIO_MMD_VEND1, MIIM_RTL8221B_SERDES_CTRL1, reg);

	/* Set MMD 30.0x6F10.15:0 = 0xD455 as stated on App Note */
	reg = 0xD433; // For both opt 1 and 3
	phy_write(phydev, MDIO_MMD_VEND1, 0x6F10, reg);

	/* Set MMD 30.0x6F11.15:0 = 0x8020 as stated on App Note */
	reg = 0x8020;
	phy_write(phydev, MDIO_MMD_VEND1, 0x6F11, reg);

	/* HISGMII related config */
	reg = 0x2;
	phy_write(phydev, MDIO_MMD_VEND1, 0x7588, reg);
	reg = 0x71D0;
	phy_write(phydev, MDIO_MMD_VEND1, 0x7589, reg);
	reg = 0x3;
	phy_write(phydev, MDIO_MMD_VEND1, 0x7587, reg);
	udelay(100);

	reg = 0x5040;
	phy_write(phydev, MDIO_MMD_VEND2, MIIM_RTL8221B_PHY_FEDCR, reg);
	udelay(20000);	/* 20 ms */

	reg = phy_read(phydev, MDIO_MMD_VEND2, MIIM_RTL8221B_PHY_STATUS);

	reg = 0x1040;
	phy_write(phydev, MDIO_MMD_VEND2, MIIM_RTL8221B_PHY_FEDCR, reg);

	/* Restart line side auto-neg after host side link up */
	for (i = 0; i < loop_cnt; i++) {
		reg = phy_read(phydev, MDIO_MMD_VEND1,
			       MIIM_RTL8221B_SERDES_CTRL5);
#ifdef RTL8221B_OPT_3
		if ((reg & MIIM_RTL8221B_SERDES_2500) ==
		    MIIM_RTL8221B_SERDES_2500)
#else
		if ((reg & MIIM_RTL8221B_SERDES_2500) ==
		    MIIM_RTL8221B_SERDES_2500 ||
		    (reg & MIIM_RTL8221B_SERDES_1000) ==
		    MIIM_RTL8221B_SERDES_1000)
#endif
			break;
		udelay(100000);
	}
	if (i >= loop_cnt)
		puts("Serdes speed not up correctly\n");

	reg = phy_read(phydev, MDIO_MMD_AN, MDIO_CTRL1);
	reg |= 0x200;
	phy_write(phydev, MDIO_MMD_AN, MDIO_CTRL1, reg);
	udelay(1000);

	return 0;
}

static int rtl8211x_parse_status(struct phy_device *phydev)
{
	unsigned int speed;
	unsigned int mii_reg;

	mii_reg = phy_read(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211x_PHY_STATUS);

	if (!(mii_reg & MIIM_RTL8211x_PHYSTAT_SPDDONE)) {
		int i = 0;

		/* in case of timeout ->link is cleared */
		phydev->link = 1;
		puts("Waiting for PHY realtime link");
		while (!(mii_reg & MIIM_RTL8211x_PHYSTAT_SPDDONE)) {
			/* Timeout reached ? */
			if (i > PHY_AUTONEGOTIATE_TIMEOUT) {
				puts(" TIMEOUT !\n");
				phydev->link = 0;
				break;
			}

			if ((i++ % 1000) == 0)
				putc('.');
			udelay(1000);	/* 1 ms */
			mii_reg = phy_read(phydev, MDIO_DEVAD_NONE,
					MIIM_RTL8211x_PHY_STATUS);
		}
		puts(" done\n");
		udelay(500000);	/* another 500 ms (results in faster booting) */
	} else {
		if (mii_reg & MIIM_RTL8211x_PHYSTAT_LINK)
			phydev->link = 1;
		else
			phydev->link = 0;
	}

	if (mii_reg & MIIM_RTL8211x_PHYSTAT_DUPLEX)
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;

	speed = (mii_reg & MIIM_RTL8211x_PHYSTAT_SPEED);

	switch (speed) {
	case MIIM_RTL8211x_PHYSTAT_GBIT:
		phydev->speed = SPEED_1000;
		break;
	case MIIM_RTL8211x_PHYSTAT_100:
		phydev->speed = SPEED_100;
		break;
	default:
		phydev->speed = SPEED_10;
	}

	return 0;
}

static int rtl8211f_parse_status(struct phy_device *phydev)
{
	unsigned int speed;
	unsigned int mii_reg;
	int i = 0;

	phy_write(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PAGE_SELECT, 0xa43);
	mii_reg = phy_read(phydev, MDIO_DEVAD_NONE, MIIM_RTL8211F_PHY_STATUS);

	phydev->link = 1;
	while (!(mii_reg & MIIM_RTL8211F_PHYSTAT_LINK)) {
		if (i > PHY_AUTONEGOTIATE_TIMEOUT) {
			puts(" TIMEOUT !\n");
			phydev->link = 0;
			break;
		}

		if ((i++ % 1000) == 0)
			putc('.');
		udelay(1000);
		mii_reg = phy_read(phydev, MDIO_DEVAD_NONE,
				   MIIM_RTL8211F_PHY_STATUS);
	}

	if (mii_reg & MIIM_RTL8211F_PHYSTAT_DUPLEX)
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;

	speed = (mii_reg & MIIM_RTL8211F_PHYSTAT_SPEED);

	switch (speed) {
	case MIIM_RTL8211F_PHYSTAT_GBIT:
		phydev->speed = SPEED_1000;
		break;
	case MIIM_RTL8211F_PHYSTAT_100:
		phydev->speed = SPEED_100;
		break;
	default:
		phydev->speed = SPEED_10;
	}

	return 0;
}

static int rtl8221b_parse_status(struct phy_device *phydev)
{
	u16 reg;
	int i = 0;

	phydev->link = 1;
	reg = phy_read(phydev, MDIO_MMD_VEND2, MIIM_RTL8221B_PHY_STATUS);
	if (!(reg & MIIM_RTL8221B_LINK_UP))
		puts("Waiting for PHY link up.");
	while (!(reg & MIIM_RTL8221B_LINK_UP)) {
		if (i > PHY_AUTONEGOTIATE_TIMEOUT) {
			puts(" TIMEOUT !\n");
			phydev->link = 0;
			break;
		}

		if ((i++ % 1000) == 0)
			putc('.');
		udelay(1000);
		reg = phy_read(phydev, MDIO_MMD_VEND2,
			       MIIM_RTL8221B_PHY_STATUS);
	}

	if (phydev->link == 1) {
		u16 speed;
		speed = (reg & MIIM_RTL8221B_SPEED_MASK);
		if (speed == MIIM_RTL8221B_SPEED_2500)
			phydev->speed = SPEED_2500;
		else if (speed == MIIM_RTL8221B_SPEED_1000)
			phydev->speed = SPEED_1000;
		else if (speed == MIIM_RTL8221B_SPEED_100)
			phydev->speed = SPEED_100;
		else
			phydev->speed = SPEED_10;

		if (reg & MIIM_RTL8221B_FULL_DUPLEX)
			phydev->duplex = DUPLEX_FULL;
		else
			phydev->duplex = DUPLEX_HALF;
	}
	else {
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_HALF;
		printf("PHY eth link status reg 0x%x\n", reg);
		reg = phy_read(phydev, MDIO_MMD_PCS, MDIO_STAT1);
		udelay(100);
		reg = phy_read(phydev, MDIO_MMD_PCS, MDIO_STAT1);
		printf("PHY pcs link status reg 0x%x\n", reg);
		reg = phy_read(phydev, MDIO_MMD_VEND1, 0x758d);
		printf("PHY serdes link status reg 0x%x\n", reg);
	}

	return 0;
}

static int rtl8211x_startup(struct phy_device *phydev)
{
	int ret;

	/* Read the Status (2x to make sure link is right) */
	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return rtl8211x_parse_status(phydev);
}

static int rtl8211e_startup(struct phy_device *phydev)
{
	int ret;

	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return genphy_parse_link(phydev);
}

static int rtl8211f_startup(struct phy_device *phydev)
{
	int ret;

	/* Read the Status (2x to make sure link is right) */
	ret = genphy_update_link(phydev);
	if (ret)
		return ret;
	/* Read the Status (2x to make sure link is right) */

	return rtl8211f_parse_status(phydev);
}

static int rtl8221b_startup(struct phy_device *phydev)
{
	int ret;

	/* Read the Status (2x to make sure link is right) */
	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return rtl8221b_parse_status(phydev);
}

/* Support for RTL8211B PHY */
static struct phy_driver RTL8211B_driver = {
	.name = "RealTek RTL8211B",
	.uid = 0x1cc912,
	.mask = 0xffffff,
	.features = PHY_GBIT_FEATURES,
	.probe = &rtl8211b_probe,
	.config = &rtl8211x_config,
	.startup = &rtl8211x_startup,
	.shutdown = &genphy_shutdown,
};

/* Support for RTL8211E-VB-CG, RTL8211E-VL-CG and RTL8211EG-VB-CG PHYs */
static struct phy_driver RTL8211E_driver = {
	.name = "RealTek RTL8211E",
	.uid = 0x1cc915,
	.mask = 0xffffff,
	.features = PHY_GBIT_FEATURES,
	.probe = &rtl8211e_probe,
	.config = &rtl8211x_config,
	.startup = &rtl8211e_startup,
	.shutdown = &genphy_shutdown,
};

/* Support for RTL8211DN PHY */
static struct phy_driver RTL8211DN_driver = {
	.name = "RealTek RTL8211DN",
	.uid = 0x1cc914,
	.mask = 0xffffff,
	.features = PHY_GBIT_FEATURES,
	.config = &rtl8211x_config,
	.startup = &rtl8211x_startup,
	.shutdown = &genphy_shutdown,
};

/* Support for RTL8211F PHY */
static struct phy_driver RTL8211F_driver = {
	.name = "RealTek RTL8211F",
	.uid = 0x1cc916,
	.mask = 0xffffff,
	.features = PHY_GBIT_FEATURES,
	.probe = &rtl8211f_probe,
	.config = &rtl8211f_config,
	.startup = &rtl8211f_startup,
	.shutdown = &genphy_shutdown,
	.readext = &rtl8211f_phy_extread,
	.writeext = &rtl8211f_phy_extwrite,
};

/* Support for RTL8221B PHY */
static struct phy_driver RTL8221B_driver = {
	.name = "RealTek RTL8221B",
	.uid = 0x1cc849,
	.mask = 0xffffff,
	.features = PHY_2500_FEATURES,
	.probe = &rtl8221b_probe,
	.config = &rtl8221b_config,
	.startup = &rtl8221b_startup,
	.shutdown = &genphy_shutdown,
};

int phy_realtek_init(void)
{
	phy_register(&RTL8211B_driver);
	phy_register(&RTL8211E_driver);
	phy_register(&RTL8211F_driver);
	phy_register(&RTL8211DN_driver);
	phy_register(&RTL8221B_driver);

	return 0;
}
