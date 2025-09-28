// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for TI DS250DF230 Retimer
 *
 * Copyright 2023 EdgeQ Inc.
 */
#include <common.h>
#include <linux/bitops.h>
#include <phy.h>
#include <i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include "../eqxgbe/xgbe_xgmac.h"


#define DS250_ADDR	      0x18
#define I2C_MUX_ADDR	      0x70
#define RTM_MUX_CHAN	      0x10

/* To calculate the calibration clock setting:
 * setting = Floor{(DataRate × 1024) / (32 × ClockFreq)} + 0x8000
 * To support 10G ethernet on EdgeQ SoC:
 *     DataRate = 10.315Gbps
 *     ClockFreq = 19.44Mhz
 */
#define DS250_CALIB_CLK_10G	0xC24F // 19.44MHz ref clock
#define DS250_CALIB_CLK_25G	0xFFFF // 19.44MHz not supported 25G
#define SPEED_UNKNOWN		-1

static int ds250df230_probe(struct phy_device *phydev)
{
	/* Nothing to do */
	return 0;
}

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

static int check_cal_clk(struct udevice *dev)
{
	u8 value;

	if (dm_i2c_read(dev, 0x0B, &value, 1))
		return -EFAULT;
	/* 0 is to enable CAL_CLK_N detection */
	value &= ~BIT(3);
	if (dm_i2c_write(dev, 0x0B, &value, 1))
		return -EFAULT;
	udelay(100);
	if (dm_i2c_read(dev, 0x0B, &value, 1))
		return -EFAULT;
	if (!(value & BIT(6)))
		printf("Ref clock not detected (0x%x)\n", value);

	return 0;
}

static int reset_ch(struct udevice *dev)
{
	u8 value;

	if (dm_i2c_read(dev, 0x00, &value, 1))
		return -EFAULT;
	value |= GENMASK(3, 0);
	if (dm_i2c_write(dev, 0x00, &value, 1))
		return -EFAULT;
	udelay(100);
	value &= ~GENMASK(3, 0);
	if (dm_i2c_write(dev, 0x00, &value, 1))
		return -EFAULT;

	return 0;
}

static int check_signal_detect(struct udevice *dev, u8 *cdr_lock)
{
	u8 value;

	/* For debugging */
	//if (dm_i2c_read(dev, 0x78, &value, 1))
	//	return -EFAULT;
	//printf("Signal detect 0x78 %02x\n", (value >> 5) & 0x1);
	//if (dm_i2c_read(dev, 0x01, &value, 1))
	//	return -EFAULT;
	//printf("Signal detect 0x01 %02x\n", (value >> 7) & 0x1);

	if (dm_i2c_read(dev, 0x78, &value, 1))
		return -EFAULT;
	*cdr_lock = (value >> 4) & 0x1;

	return 0;
}

#if 0
/* For debugging */
static int read_eye(struct udevice *dev)
{
	u8 heo;
	u8 veo;
	u32 H, V;

	if (dm_i2c_read(dev, 0x27, &heo, 1))
		return -EFAULT;
	if (dm_i2c_read(dev, 0x28, &veo, 1))
		return -EFAULT;
	H = (u32)heo * 1000 / 32;
	V = (u32)veo * 3125 / 1000;
	printf("Eye H=%d/1000 UI V=%d mV\n", H, V);

	return 0;
}
#endif

#define DS250_SPEED_CFG		GENMASK(7, 4)
#define DS250_SPEED_25000	(0x5 << 4)
#define DS250_SPEED_10000	(0x6 << 4)
static int enable_ch(struct udevice *dev, u8 chan, u16 speed)
{
	u8 clk_lo;
	u8 clk_hi;
	u8 value;

	if (select_ch(dev, chan, 1))
		return -EFAULT;

	/* Set speed */
	if (speed == SPEED_25000) {
		if (DS250_CALIB_CLK_25G == 0xFFFF)
			return -EFAULT;

		clk_lo = DS250_CALIB_CLK_25G & 0xFF;
		clk_hi = (DS250_CALIB_CLK_25G >> 8) & 0xFF;
	}
	else {
		clk_lo = DS250_CALIB_CLK_10G & 0xFF;
		clk_hi = (DS250_CALIB_CLK_10G >> 8) & 0xFF;
	}
	if (dm_i2c_write(dev, 0x60, &clk_lo, 1))
		return -EFAULT;
	if (dm_i2c_write(dev, 0x61, &clk_hi, 1))
		return -EFAULT;
	if (dm_i2c_write(dev, 0x62, &clk_lo, 1))
		return -EFAULT;
	if (dm_i2c_write(dev, 0x63, &clk_hi, 1))
		return -EFAULT;
	value = 0xFF;
	if (dm_i2c_write(dev, 0x64, &value, 1))
		return -EFAULT;
	if (dm_i2c_read(dev, 0x09, &value, 1))
		return -EFAULT;
	value |= 0x4;
	if (dm_i2c_write(dev, 0x09, &value, 1))
		return -EFAULT;
	if (dm_i2c_read(dev, 0x18, &value, 1))
		return -EFAULT;
	value &= 0x8F;
	if (speed == SPEED_10000)
		value |= 0x10;
	if (dm_i2c_write(dev, 0x18, &value, 1))
		return -EFAULT;

	/* CDR reset */
	if (dm_i2c_read(dev, 0x0A, &value, 1))
		return -EFAULT;
	value |= GENMASK(3, 2);
	if (dm_i2c_write(dev, 0x0A, &value, 1))
		return -EFAULT;
	value &= ~GENMASK(3, 2);
	if (dm_i2c_write(dev, 0x0A, &value, 1))
		return -EFAULT;
	udelay(1000000);

	return 0;
}

static int ds250df230_config(struct phy_device *phydev)
{
	struct dw_eth_dev *priv = (struct dw_eth_dev *)phydev->priv;
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

	/* Select channel 0 and disable smb */
	if (select_ch(rtm_dev, 0, 0))
		return -EFAULT;

	/* Check cal clock */
	if (check_cal_clk(rtm_dev))
		return -EFAULT;

	/* Select channel 0 and enable smb */
	if (select_ch(rtm_dev, 0, 1))
		return -EFAULT;

	/* Reset channel */
	if (reset_ch(rtm_dev))
		return -EFAULT;

	/* Select channel 1 and enable smb */
	if (select_ch(rtm_dev, 1, 1))
		return -EFAULT;

	/* Reset channel */
	if (reset_ch(rtm_dev))
		return -EFAULT;

	/* Enable channel 1 */
	if (enable_ch(rtm_dev, 1, priv->max_speed))
		return -EFAULT;

	/* Enable channel 0 */
	if (enable_ch(rtm_dev, 0, priv->max_speed))
		return -EFAULT;

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}

static int ds250df230_startup(struct phy_device *phydev)
{
	struct udevice *i2c_bus;
	struct udevice *mux_dev;
	struct udevice *rtm_dev;
	u8 saved_mux_chan;
	u8 mux_chan = RTM_MUX_CHAN;
	u8 value;
	u8 host_link = 0;;
	u8 line_link = 0;;
        int retries;
        int i;

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

        /* Wait upto 1 second for link to come up */
        phydev->link = 0;
        retries = 20;
        for (i = 0; i < retries; i++) {
		/* Host side */
		if (select_ch(rtm_dev, 0, 1))
			break;
		if (check_signal_detect(rtm_dev, &host_link))
			break;
		/* For debugging */
		//read_eye(rtm_dev);
		/* Line side */
		if (select_ch(rtm_dev, 1, 1))
			break;
		if (check_signal_detect(rtm_dev, &line_link))
			break;
		/* For debugging */
		//read_eye(rtm_dev);

		if (host_link && line_link) {
			phydev->link = 1;
			break;
		}
		udelay(100);
	}

	if (phydev->link == 1) {
		u8 value2;
		u16 clk;

		dm_i2c_read(rtm_dev, 0x60, &value, 1);
		dm_i2c_read(rtm_dev, 0x61, &value2, 1);
		clk = (value2 << 8) + value;
		if (clk == DS250_CALIB_CLK_25G) {
			phydev->speed = SPEED_25000;
			phydev->duplex = DUPLEX_FULL;
		}
		else if (clk == DS250_CALIB_CLK_10G) {
			phydev->speed = SPEED_10000;
			phydev->duplex = DUPLEX_FULL;
		}
		else {
			phydev->speed = SPEED_UNKNOWN;
			phydev->duplex = DUPLEX_HALF;
		}
	}
	else {
                if (!line_link)
			printf("PHY line side link down\n");
		if (!host_link)
			printf("PHY host side link down\n");
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_HALF;
	}

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}

static int ds250df230_shutdown(struct phy_device *phydev)
{
	return 0;
}

/* Support for TI DS250DF230 Retimer */
static struct phy_driver TIDS250DF230_driver = {
	.name = "TI DS250DF230",
	.uid = 0x15010E,
	.mask = 0xffffff,
	.features = PHY_10G_FEATURES,
	.probe = &ds250df230_probe,
	.config = &ds250df230_config,
	.startup = &ds250df230_startup,
	.shutdown = &ds250df230_shutdown,
};

int phy_tids250df230_init(void)
{
	phy_register(&TIDS250DF230_driver);

	return 0;
}

/* Default uses mdio, same as weak get_phy_id */
static int def_get_phy_id(struct mii_dev *bus, int addr, int devad, u32 *phy_id)
{
	int phy_reg;

	/*
	* Grab the bits from PHYIR1, and put them
	* in the upper half
	*/
	phy_reg = bus->read(bus, addr, devad, MII_PHYSID1);

	if (phy_reg < 0)
		return -EIO;

	*phy_id = (phy_reg & 0xffff) << 16;

	/* Grab the bits from PHYIR2, and put them in the lower half */
	phy_reg = bus->read(bus, addr, devad, MII_PHYSID2);

	if (phy_reg < 0)
		return -EIO;

	*phy_id |= (phy_reg & 0xffff);

	return 0;
}

/* Use i2c interface to get phy id */
static int i2c_get_phy_id(struct mii_dev *bus, int addr, int devad, u32 *phy_id)
{
	struct udevice *i2c_bus;
	struct udevice *mux_dev;
	struct udevice *rtm_dev;
	u8 saved_mux_chan;
	u8 mux_chan = RTM_MUX_CHAN;
	u8 dev_id[3];

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

	if (dm_i2c_read(rtm_dev, 0xEF, dev_id, 3))
		return -EFAULT;

	*phy_id = (dev_id[2] << 16) | (dev_id[1] << 8) | dev_id[0];

	if (dm_i2c_write(mux_dev, 0, &saved_mux_chan, 1))
		return -EFAULT;

	return 0;
}

int get_phy_id(struct mii_dev *bus, int addr, int devad, u32 *phy_id)
{
	struct udevice *dev = bus->priv;
	struct dw_eth_dev *priv = dev_get_priv(dev);

	if (priv->use_i2c)
		return i2c_get_phy_id(bus, addr, devad, phy_id);
	else
		return def_get_phy_id(bus, addr, devad, phy_id);
}

