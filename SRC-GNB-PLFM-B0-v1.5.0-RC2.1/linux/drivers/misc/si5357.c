// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for DCTCXO SI5357 control through I2C bus
 *
 * Copyright (c) 2022 Timothy Huang, EdgeQ, Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define DEV_CNT     1
#define DEVICE_NAME "si5357"

/* I2C Bus */
#define SI5357_I2C_BUS_ADDR               0x60

/* I2C Reg Address */
#define SI5357_I2C_FREQ_CTRL_LSW_ADDR     0x00
#define SI5357_I2C_FREQ_CTRL_MSW_ADDR     0x01
#define SI5357_I2C_PULL_RNG_CTRL_ADDR     0x02

/* I2C Reg Offset & Mask */
#define SI5357_I2C_FREQ_CTRL_LSW_OFFSET   0
#define SI5357_I2C_FREQ_CTRL_LSW_MASK     0x0000FFFF

#define SI5357_I2C_FREQ_CTRL_MSW_OFFSET   16
#define SI5357_I2C_FREQ_CTRL_MSW_MASK     0x03FF0000

#define SI5357_I2C_FREQ_CTRL_OE_OFFSET    10
#define SI5357_I2C_FREQ_CTRL_OE_MASK      (0x0001 << SI5357_I2C_FREQ_CTRL_OE_OFFSET)

#define SI5357_I2C_PULL_RNG_CTRL_OFFSET   0
#define SI5357_I2C_PULL_RNG_CTRL_MASK     (0x000F << SI5357_I2C_PULL_RNG_CTRL_MASK)

/* Pull Range */
#define SI5357_I2C_PULL_RNG_LOOKUP_TBL_SZ 16
#define SI5357_I2C_PULL_RNG_6_25_MAX      1250000
#define SI5357_I2C_PULL_RNG_200_MAX       33554431

/* Default values */
#define SI5357_I2C_PULL_RNG_FULL_SCALE    33554431
#define SI5357_I2C_PULL_RNG_6_25          0 /* 0: +/-6.25ppm w/ 5x10^-12 resolution */
#define SI5357_I2C_PULL_RNG_DFLT          SI5357_I2C_PULL_RNG_6_25
#define SI5357_I2C_PULL_RNG_200           9 /* 9: +/-200ppm w/ 5x10^-12 resolution */

#define SwapTwoBytes(data) ((((data) >> 8) & 0x00FF) | (((data) << 8) & 0xFF00))

static uint32_t si5356ax_pull_range_tbl[SI5357_I2C_PULL_RNG_LOOKUP_TBL_SZ] = {
	6250,	10000,	12500,	25000,	50000,	80000,	 100000,  125000,
	150000, 200000, 400000, 600000, 800000, 1200000, 1600000, 3200000
};
static const struct file_operations si5357_fops = { .owner = THIS_MODULE,
						    .open = NULL,
						    .release = NULL,
						    .unlocked_ioctl = NULL,
						    .read = NULL,
						    .write = NULL };
struct si5357_device_data {
	struct cdev cdev;
	struct device *dev;
	struct i2c_adapter *i2c_adap; /* I2C bus adapter */
	uint32_t pull_range_ppb;
	int64_t pull_range_max;
	uint32_t bus; /* I2C bus line */
	uint32_t addr; /* I2C slave address */
};

static int dev_major;
static struct si5357_device_data si5357_data;

/**
 * @brief Set the frequency pull range for SI5357
 *
 * @param pull_range
 * @return int
 */
int si5357_set_pull_range(uint8_t pull_range)
{
	static struct i2c_adapter *i2c_adap;
	union i2c_smbus_data val;

	if (i2c_adap == NULL) {
		i2c_adap = i2c_get_adapter(si5357_data.bus);
		if (i2c_adap == NULL) {
			dev_err(si5357_data.dev,
				"Failed to get I2C bus %u adapter\n",
				si5357_data.bus);
			return -EFAULT;
		}
	}

	memset(&val, 0, sizeof(val));
	val.byte = pull_range;
	if (i2c_smbus_xfer(i2c_adap, si5357_data.addr, si5357_data.bus,
			   I2C_SMBUS_WRITE, SI5357_I2C_PULL_RNG_CTRL_ADDR,
			   I2C_SMBUS_BYTE_DATA, &val)) {
		dev_err(si5357_data.dev,
			"I2C failed to transfer pull range to slave device 0x%02X on bus %u\n",
			si5357_data.addr, si5357_data.bus);
		return -EAGAIN;
	}

	si5357_data.pull_range_ppb = si5356ax_pull_range_tbl[pull_range];
	si5357_data.pull_range_max = SI5357_I2C_PULL_RNG_FULL_SCALE;

	return 0;
}
EXPORT_SYMBOL_GPL(si5357_set_pull_range);

/**
 * @brief Get current frequency pull range setting
 *
 * @return int
 */
int si5357_get_pull_range(void)
{
	return si5357_data.pull_range_ppb;
}
EXPORT_SYMBOL_GPL(si5357_get_pull_range);

/**
 * @brief Get output enable pin status (not implemented yet)
 *
 * @return int
 */
int si5357_get_oe_ctrl(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(si5357_get_oe_ctrl);

/**
 * @brief Set SI5357's frequency in ppb
 *
 * @param ppb
 * @return int
 */
int si5357_set_freq(int ppb)
{
	int64_t ctrl_word = 0;
	uint16_t ctrl_lsw = 0, ctrl_msw = 0;
	union i2c_smbus_data val, val1;
	static struct i2c_adapter *i2c_adap;
	int ret = 0;

	if (i2c_adap == NULL) {
		i2c_adap = i2c_get_adapter(si5357_data.bus);
		if (i2c_adap == NULL) {
			dev_err(si5357_data.dev,
				"Failed to get I2C bus %u adapter\n",
				si5357_data.bus);
			return -EFAULT;
		}
	}

	dev_dbg(si5357_data.dev, "[%s] ppb: %d\n", __func__, ppb);

	if (si5357_data.pull_range_ppb == 0) {
		ret = si5357_set_pull_range(SI5357_I2C_PULL_RNG_DFLT);
		if (ret < 0) {
			dev_err(si5357_data.dev,
				"Failed to SI5357 pull range\n");
			return ret;
		}
	}

	ctrl_word = (int64_t)ppb * si5357_data.pull_range_max /
		    si5357_data.pull_range_ppb;
	ctrl_lsw = (uint16_t)((int)ctrl_word &
			      SI5357_I2C_FREQ_CTRL_LSW_MASK); // lower 16 bits
	ctrl_msw =
		(uint16_t)(((int)ctrl_word & SI5357_I2C_FREQ_CTRL_MSW_MASK) >>
			   SI5357_I2C_FREQ_CTRL_MSW_OFFSET); // higher 10 bits
	dev_dbg(si5357_data.dev,
		"[%s][%d] pull_range_ppb: %u, pull_range_max: %lld, ctrl_word: %lld\n",
		__func__, __LINE__, si5357_data.pull_range_ppb,
		si5357_data.pull_range_max, ctrl_word);
	dev_dbg(si5357_data.dev,
		"[%s][%d] ctrl_word: 0x%08X, ctrl_lsw: 0x%04X, ctrl_msw: 0x%04X\n",
		__func__, __LINE__, (int)ctrl_word, ctrl_lsw, ctrl_msw);
	ctrl_msw |= SI5357_I2C_FREQ_CTRL_OE_MASK; // OE should always be enabled

	/* Wrtie LSW first */
	memset(&val, 0, sizeof(val));
	val.word = SwapTwoBytes(ctrl_lsw);
	dev_dbg(si5357_data.dev,
		"[%s][%d] ctrl_lsw: 0x%04X, ctrl_msw: 0x%04X, word: 0x%08X\n",
		__func__, __LINE__, ctrl_lsw, ctrl_msw, val.word);
	ret = i2c_smbus_xfer(i2c_adap, si5357_data.addr, si5357_data.bus,
			     I2C_SMBUS_WRITE, SI5357_I2C_FREQ_CTRL_LSW_ADDR,
			     I2C_SMBUS_WORD_DATA, &val);
	if (ret) {
		dev_err(si5357_data.dev,
			"I2C failed to transfer LSW to slave device 0x%02X on bus %u\n",
			si5357_data.addr, si5357_data.bus);
		return ret;
	}
	dev_dbg(si5357_data.dev, "[%s][%d]\n", __func__, __LINE__);

	/* Write MSW */
	memset(&val1, 0, sizeof(val1));
	val1.word = SwapTwoBytes(ctrl_msw);
	dev_dbg(si5357_data.dev,
		"[%s][%d] ctrl_lsw: 0x%04X, ctrl_msw: 0x%04X, word: 0x%08X\n",
		__func__, __LINE__, ctrl_lsw, ctrl_msw, val1.word);
	ret = i2c_smbus_xfer(i2c_adap, si5357_data.addr, si5357_data.bus,
			     I2C_SMBUS_WRITE, SI5357_I2C_FREQ_CTRL_MSW_ADDR,
			     I2C_SMBUS_WORD_DATA, &val1);
	if (ret) {
		dev_err(si5357_data.dev,
			"I2C failed to transfer MSW to slave device 0x%02X on bus %u\n",
			si5357_data.addr, si5357_data.bus);
		return ret;
	}
	dev_dbg(si5357_data.dev, "[%s][%d]\n", __func__, __LINE__);

	return ret;
}
EXPORT_SYMBOL_GPL(si5357_set_freq);

static int si5357_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	if (of_property_read_u32(node, "bus", &si5357_data.bus))
		return -EINVAL;
	dev_info(&pdev->dev, "Bus line: %u\n", si5357_data.bus);

	if (of_property_read_u32(node, "addr", &si5357_data.addr))
		return -EINVAL;
	dev_info(&pdev->dev, "I2C salve addr: 0x%02X\n", si5357_data.addr);

	return 0;
}

static int si5357_probe(struct platform_device *pdev)
{
	dev_t dev;
	int ret = 0;

	memset(&si5357_data, 0, sizeof(si5357_data));

	si5357_data.dev = &pdev->dev;

	/* Init character device */
	ret = alloc_chrdev_region(&dev, 0, DEV_CNT, "si5357");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate device number\n");
		return ret;
	}

	dev_major = MAJOR(dev);

	cdev_init(&si5357_data.cdev, &si5357_fops);
	si5357_data.cdev.owner = THIS_MODULE;
	cdev_add(&si5357_data.cdev, MKDEV(dev_major, 0), 1);

	/* Parse device tree - get I2C bus line and slave address */
	ret = si5357_parse_dt(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse SI5357 device node\n");
		unregister_chrdev_region(MKDEV(dev_major, 0), DEV_CNT);
		return ret;
	}

	dev_info(&pdev->dev, "Successfully probed Si5357 driver\n");

	return ret;
}

static int si5357_remove(struct platform_device *pdev)
{
	cdev_del(&si5357_data.cdev);
	unregister_chrdev_region(MKDEV(dev_major, 0), DEV_CNT);

	return 0;
}
static const struct of_device_id si5357_match_tbl[] = {
	{
		.compatible = "sitime,si5357",
	}, {
		/* sentinel */
	}
};
static struct platform_driver si5357_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = si5357_match_tbl,
	},
	.probe = si5357_probe,
	.remove = si5357_remove,
};

module_platform_driver(si5357_driver);

MODULE_AUTHOR("Timothy Huang <tim.huang@edgeq.io>");
MODULE_DESCRIPTION("DCTCXO driver for SI5357");
MODULE_LICENSE("GPL");
