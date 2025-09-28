// SPDX-License-Identifier: GPL-2.0+
/*
 * Andestech ATCIIC100 I2C controller driver.
 * Copyright 2021 EdgeQ Inc.
 * Author: Nilesh Waghmare (nilesh.waghmare@edgeq.io)
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <i2c.h>
#include <log.h>
#include <asm/io.h>
#include <clk.h>
#include <reset.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/bitops.h>

DECLARE_GLOBAL_DATA_PTR;

/* ID and Revision Register */
#define IDREV   0x00
#define ID_OFF  12
#define ID_MSK  0xfffff
#define ID      0x02021
/* Configuration Register */
#define CFG     0x10
#define FIFOSIZE_MASK   0x3

/* INterrupt Enable Register */
#define INTEN   0x14
// Do not enable ByteRecv INTR 
// else we need to manually ACK each received byte in SW
#define INTR_EN (0x2FF) 

/* Completion and Fifo Full */
#define INTR_EN_RD (0x202)
/* Completion and Fifo Empty */
#define INTR_EN_WR (0x201)

/* Status Register */
#define STA     0x18

#define LINESDA BIT(14)
#define LINESCL BIT(13)
#define GENCALL BIT(12)
#define BBUSY   BIT(11)
#define ACK     BIT(10)
#define COMPL   BIT(9)
#define BRECV   BIT(8)
#define BTRAN   BIT(7)
#define START   BIT(6)
#define STOP    BIT(5)
#define ALOSE   BIT(4)
#define ADDRHIT BIT(3)
#define FIFOHAF BIT(2)
#define FIFOFUL BIT(1)
#define FIFOETY BIT(0)
#define STAW1C  (COMPL | BRECV | BTRAN | START | STOP | ALOSE | ADDRHIT)
#define STA_CLR (0x1FF)

#define INTR_EN_COMPL_FIFOETY (COMPL | FIFOETY)
#define INTR_EN_COMPL (COMPL)
#define INTR_ALL (0x3FF)

/* Address Register */
#define ADR     0x1c

/* Data Register */
#define DAT     0x20

/* Control Register */
#define CTL     0x24
#define PSTART  BIT(12)
#define PADDR   BIT(11)
#define PDATA   BIT(10)
#define PSTOP   BIT(9)
#define DIR_RX  BIT(8)
#define DCNTMSK 0xff

/* Command Register */
#define CMD             0x28
#define ISSUE_DATA      0x1
#define RESPOND_ACK     0x2
#define RESPOND_NACK    0x3
#define CLEAR_FIFO      0x4
#define RESET_IIC       0x5

/* Setup Register */
#define SETUP           0x2c
#define MASTER          BIT(2)
#define ADDR10          BIT(1)
#define IICEN           BIT(0)
#define IICEN_set       (0)
#define MASTER_set      (2)
#define T_SUDAT         (24)
#define T_SP            (21)
#define T_HDDAT         (16)
#define T_SCLRATIO      (13)
#define T_SCLHI         (4)

/* I2C Timing Parameter Multiplier Register */
#define TPM             0x30

#define XFER_TIMEOUT    0xf000

#ifdef CONFIG_ATCIIC100_DEBUG
#define PRINTK printk
#else
#define PRINTK(x...)
#endif

/* Data setup time in nano seconds */
#define SUDAT_MIN_STANDARD_MODE         250
#define SUDAT_MIN_FAST_MODE             100
#define SUDAT_MIN_FAST_MODE_PLUS        50

/* Data hold time in nano seconds */
#define HDDAT_MIN_STANDARD_MODE         300
#define HDDAT_MIN_FAST_MODE             300
#define HDDAT_MIN_FAST_MODE_PLUS        0

/* SCL LOW period  in ns*/
#define T_LOW_STANDARD_MODE             4700
#define T_LOW_FAST_MODE                 1300
#define T_LOW_FAST_MODE_PLUS            500

/* Spike Suppression Width */
#define T_SSW_MAX                       50

#define T_SP_MIN                        0x1
#define T_SCLHI_MAX                     0x200

/* I2C Frequency Modes */
#define I2C_MAX_STANDARD_MODE_FREQ      100000
#define I2C_MAX_FAST_MODE_FREQ          400000
#define I2C_MAX_FAST_MODE_PLUS_FREQ     1000000
#define I2C_MAX_TURBO_MODE_FREQ         1400000
#define I2C_MAX_HIGH_SPEED_MODE_FREQ    3400000
#define I2C_MAX_ULTRA_FAST_MODE_FREQ    5000000

/* Information about i2c controller */
struct raptor2_i2c_priv {
	int			id;
	struct clk		clk;
	int			speed;
	unsigned int bus_clk_rate;
	void			*base;
	struct udevice		*bus;
	int			reg_shift;      /* reg offset shift */
	int                     addr_hit;
	int                     cmpl;
	struct i2c_msg          *pmsg;
	u8                      *ptr;
	int                     readnum;
	int                     writenum;
	int                     result;
	u8                      fifolen;
};

static inline void raptor2_i2c_writel(struct raptor2_i2c_priv *priv, u32 val, int reg)
{
	writel(val, priv->base + (reg << (priv->reg_shift - 1)));
}

static inline u32 raptor2_i2c_readl(struct raptor2_i2c_priv *priv, int reg)
{
	return readl(priv->base + (reg << (priv->reg_shift - 1)));
}

static int raptor_i2c_set_bus_speed(struct udevice *dev, unsigned int speed)
{
	struct raptor2_i2c_priv *i2c_bus = dev_get_priv(dev);

	i2c_bus->speed = speed;

	return 0;
}

static void raptor2_i2c_init(struct udevice *dev)
{
        
	//Initialize I2C
	int t_sp = 0, tpm = 0, t_sclhi = 0, t_sudat =0, t_hddat = 0;
	u32 wdata, cfgdata;
	struct raptor2_i2c_priv *i2c_bus = dev_get_priv(dev);

	/* reset controller */
	raptor2_i2c_writel(i2c_bus, RESET_IIC, CMD);

	/* interrupt status clear */
	raptor2_i2c_writel(i2c_bus, STA_CLR, STA);

	/* I2C Timing Setup */
	tpm = dev_read_u32_default(dev, "tpm", tpm);
	if(!tpm)
		dev_dbg(&bus->dev,"Unable to find tpm\n");

	t_sp = dev_read_u32_default(dev, "t_sp", t_sp);
	if(!t_sp)
		dev_dbg(&bus->dev,"Unable to find t_sp\n");

	t_sudat = dev_read_u32_default(dev, "t_sudat", t_sudat);
	if(!t_sudat)
		dev_dbg(&bus->dev,"Unable to find t_sudat\n");

	t_hddat = dev_read_u32_default(dev, "t_hddat", t_hddat);
	if(!t_hddat)
		dev_dbg(&bus->dev,"Unable to find t_hddat\n");

	t_sclhi = dev_read_u32_default(dev, "t_sclhi", t_sclhi);
	if(!t_sclhi)
		dev_dbg(&bus->dev,"Unable to find t_sclhi\n");

	raptor2_i2c_writel(i2c_bus, tpm, TPM);
	
	/*
	 * Timing parameters for standard mode
	 * The LOW period is equal to HIGH period for std mode hence ratio is 1
	 * */
	//wdata = raptor2_i2c_readl(i2c_bus, SETUP);
	//wdata &= ~(1 << T_SCLRATIO);
	wdata = (t_sp << T_SP) | (t_sudat << T_SUDAT) | (t_hddat << T_HDDAT) | (t_sclhi << T_SCLHI);
	raptor2_i2c_writel(i2c_bus, wdata, SETUP);
	
	wdata = raptor2_i2c_readl(i2c_bus, TPM);
	wdata |= tpm;
	raptor2_i2c_writel(i2c_bus, wdata, TPM);

	cfgdata = raptor2_i2c_readl(i2c_bus, CFG);
	i2c_bus->fifolen = (1 << ((cfgdata & FIFOSIZE_MASK) + 1));
	wdata = raptor2_i2c_readl(i2c_bus, SETUP);
	wdata |= (1 << MASTER_set) | (1 << IICEN_set);
	raptor2_i2c_writel(i2c_bus, wdata, SETUP);
}

static int raptor_i2c_probe(struct udevice *dev)
{
	struct raptor2_i2c_priv *i2c_bus = dev_get_priv(dev);
	int ret;

	i2c_bus->base = map_physmem(devfdt_get_addr(dev),
				    sizeof(ulong),
				 MAP_NOCACHE);
	if (!i2c_bus->base) {
		dev_dbg(&bus->dev,"%s: could not map device address\n", __func__);
		return -EINVAL;
	}


	ret = clk_get_by_index(dev, 0, &i2c_bus->clk);
	if (ret) {
		dev_dbg(&bus->dev,"%s: Failed to get i2c clk\n", __func__);
		return ret;
	}

	i2c_bus->bus_clk_rate = dev_read_u32_default(dev,
			"clock-frequency", I2C_SPEED_STANDARD_RATE);
	
	if (!i2c_bus->bus_clk_rate) {
		dev_dbg(&bus->dev,"%s: Failed to get i2c clock-frequency\n", __func__);
		return ret;
	}

	i2c_bus->reg_shift = 1;
	i2c_bus->reg_shift = dev_read_u32_default(dev, "reg-shift",
						  i2c_bus->reg_shift);
	if (!i2c_bus->reg_shift)
		dev_dbg(&dev->dev, "Unable to find reg-shift\n");

	i2c_bus->id = dev->seq;
	i2c_bus->bus = dev;

	raptor2_i2c_init(dev);

	return 0;
}


static int i2c_pio_transfer(struct raptor2_i2c_priv *i2c_bus, uint flags, u8 *buf, uint len)
{
	int cnt = XFER_TIMEOUT;
	int i = 0;

	if (flags & I2C_M_RD) {
		while (len--) {
			while ((raptor2_i2c_readl(i2c_bus, STA) & FIFOETY) == FIFOETY) {
				if (!(cnt--)) {
					dev_dbg(&bus->dev,"wait %s error\n", "fifo empty");
					return -EIO;
				}
			}
			buf[i++] = (u8)raptor2_i2c_readl(i2c_bus, DAT);
			cnt = XFER_TIMEOUT;
		}
		while ((raptor2_i2c_readl(i2c_bus, STA) & COMPL) != COMPL) {
			if (!(cnt--)) {
				dev_dbg(&bus->dev,"wait %s error\n", "cmpl");
				return -EIO;
			}
		}
	} else {
		while ((raptor2_i2c_readl(i2c_bus, STA) & ADDRHIT) != ADDRHIT) {
			if (!(cnt--)) {
				dev_dbg(&bus->dev,"wait %s error\n", "addrhit");
				return -EIO;
			}
		}
		cnt = XFER_TIMEOUT;
		while (len--) {
			while ((raptor2_i2c_readl(i2c_bus, STA) & FIFOFUL) == FIFOFUL) {
				if (!(cnt--)) {
					dev_dbg(&bus->dev,"wait %s error\n", "fifo full");
					return -EIO;
				}
			}
			raptor2_i2c_writel(i2c_bus, buf[i], DAT);
			cnt = XFER_TIMEOUT;
			i++;
		}
		while ((raptor2_i2c_readl(i2c_bus, STA) & COMPL) != COMPL) {
			if (!(cnt--)) {
				dev_dbg(&bus->dev,"wait %s error\n", "cmpl");
				return -EIO;
			}
		}
	}
	return 0;
}

/*
 * Generic i2c master receive.
 *
 */
static int i2c_read(struct udevice *bus, unsigned int chipaddr, u8 *buf, uint len, u8 stop)
{
	struct raptor2_i2c_priv *i2c_bus = dev_get_priv(bus);
	int rc = 0, ctl_bits = 0;

	/* Control reg - start=1, addr=1, data = 1, stop = 1, dir = 1 rx, cnt = 1 */
	ctl_bits = ((len & 0xff) | PSTART | PADDR | PDATA | DIR_RX);
	if (stop)
		ctl_bits |= PSTOP;
	raptor2_i2c_writel(i2c_bus, ctl_bits, CTL);

	raptor2_i2c_writel(i2c_bus, chipaddr, ADR);
	raptor2_i2c_writel(i2c_bus, INTR_EN_RD, INTEN);

	raptor2_i2c_writel(i2c_bus, ISSUE_DATA, CMD);

	rc = i2c_pio_transfer(i2c_bus, I2C_M_RD, buf, len);
	raptor2_i2c_writel(i2c_bus, 0, INTEN);
	raptor2_i2c_writel(i2c_bus, STAW1C, STA);
	return rc;
}

/*
 * Generic i2c master transmit.
 *
 */
static int i2c_write(struct udevice *bus, unsigned int chipaddr, u8 *buf, uint len, u8 stop)
{
	struct raptor2_i2c_priv *i2c_bus = dev_get_priv(bus);
	int rc = 0, ctl_bits = 0;

	/* Control reg - start=1, addr=1, data = 1, stop = 0, dir = 0 tx, cnt = 1 */
	ctl_bits = ((len & 0xff) | PSTART | PADDR | PDATA);
	if (stop)
		ctl_bits |= PSTOP;
	raptor2_i2c_writel(i2c_bus, ctl_bits, CTL);

	raptor2_i2c_writel(i2c_bus, chipaddr, ADR);

	raptor2_i2c_writel(i2c_bus, INTR_EN_WR, INTEN);

	/* Must have 5ms delay - from Linux port */
	mdelay(5);
	raptor2_i2c_writel(i2c_bus, ISSUE_DATA, CMD);

	rc = i2c_pio_transfer(i2c_bus, !I2C_M_RD, buf, len);
	raptor2_i2c_writel(i2c_bus, 0, INTEN);
	raptor2_i2c_writel(i2c_bus, STAW1C, STA);
	return rc;
}

static int raptor_i2c_xfer(struct udevice *bus, struct i2c_msg *msg,
			   int nmsgs)
{
	int ret=0;
	u8 stop = 0;
	
	for (; nmsgs > 0; nmsgs--, msg++) {
		dev_dbg(&bus->dev,"i2c_xfer : %sing %d byte%s %s 0x%02x\n",
			msg->flags & I2C_M_RD ? "read" : "writ",
			msg->len, msg->len > 1 ? "s" : "",
			msg->flags & I2C_M_RD ? "from" : "to", msg->addr);

		if (nmsgs == 1)
			stop = 1;
		if (msg->flags & I2C_M_RD)
			ret = i2c_read(bus, msg->addr, msg->buf, msg->len, stop);
		else
			ret = i2c_write(bus, msg->addr, msg->buf, msg->len, stop);

		if (ret){
			debug("i2c_xfer: error sending\n");
			return -EREMOTEIO;
		}
	}

	return ret;
}

/* Probe to see if a chip is present. */
static int raptor_i2c_probe_chip(struct udevice *bus, uint chip_addr,
				 uint chip_flags)
{
	int ret = 0;
	unsigned char buf;
	ret = i2c_read(bus, chip_addr, &buf, 1, 1);
	if (ret)
		raptor2_i2c_init(bus);
	
	return ret;
}

static int raptor_i2c_deblock(struct udevice *dev)
{
	int ret = 0;

	/*
	 * Attempt to reinitialize controller
	 */
	raptor2_i2c_init(dev);
	return ret;
}

static const struct dm_i2c_ops raptor_i2c_ops = {
	.xfer		= raptor_i2c_xfer,
	.probe_chip	= raptor_i2c_probe_chip,
	.set_bus_speed	= raptor_i2c_set_bus_speed,
	.deblock	= raptor_i2c_deblock,
};

static const struct udevice_id raptor_i2c_ids[] = {
	{ .compatible = "edgeq,raptor2-i2c", },
	{ }
};

U_BOOT_DRIVER(i2c_raptor) = {
	.name		= "i2c_raptor",
	.id		= UCLASS_I2C,
	.of_match	= raptor_i2c_ids,
	.probe		= raptor_i2c_probe,
	.priv_auto_alloc_size = sizeof(struct raptor2_i2c_priv),
	.ops		= &raptor_i2c_ops,
};
