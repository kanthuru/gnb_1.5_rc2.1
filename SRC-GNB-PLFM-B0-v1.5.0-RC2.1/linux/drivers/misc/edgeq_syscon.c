// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for EdgeQ system controller
 *
 * Copyright (c) 2023 Timothy Huang, EdgeQ, Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define DEVICE_NAME "eq_syscon"
#define EQ_SYSCON_DT_REG_CONF_LEN		2
#define EQ_SYSCON_DT_REG_CONF_ADDR_OFFSET_IDX	0
#define EQ_SYSCON_DT_REG_CONF_REG_VAL_IDX	1

struct edgeq_syscon {
	struct regmap *gpio;
	struct regmap *sif;
};

static struct edgeq_syscon eq_syscon;

static int eq_syscon_configure_sif(struct device *dev,
				   struct regmap *sif)
{
	struct device_node *node = dev->of_node;
	const void *prop = NULL;
	u32 sif_conf[EQ_SYSCON_DT_REG_CONF_LEN] = { 0 };
	int sif_conf_bytes, sif_conf_num;
	int i, j, offset;
	int ret = 0;

	prop = of_get_property(node, "sif_conf", &sif_conf_bytes);
	if ((prop == NULL) || (sif_conf_bytes == 0))
		return 0;

	/* sif_conf_bytes represents number of bytes in this array,
	 * and sif_conf_num represents number of entries in sif_conf.
	 * Since each entry has X 4-byte elements, sif_conf_num is
	 * derived from sif_conf_bytes / (4bytes * X)
	 */
	sif_conf_num =
		sif_conf_bytes / (sizeof(u32) * EQ_SYSCON_DT_REG_CONF_LEN);
	dev_dbg(dev, "SIF conf num: %d\n", sif_conf_num);

	for (i = 0; i < sif_conf_num; i++) {
		offset = i * EQ_SYSCON_DT_REG_CONF_LEN;

		for (j = 0; j < EQ_SYSCON_DT_REG_CONF_LEN; j++) {
			ret = of_property_read_u32_index(node, "sif_conf",
							 offset + j, &sif_conf[j]);
			if (ret) {
				dev_err(dev, "Failed to read sif_conf[%d]\n", j);
				return ret;
			}

			dev_info(dev, "sif_conf[%d][%d]: 0x%08X\n", i, j, sif_conf[j]);
		}

		ret = regmap_write(sif,
				   sif_conf[EQ_SYSCON_DT_REG_CONF_ADDR_OFFSET_IDX],
				   sif_conf[EQ_SYSCON_DT_REG_CONF_REG_VAL_IDX]);
		if (ret) {
			dev_err(dev, "Failed to write to sif\n");
			return ret;
		}
	}

	return ret;
}

static int eq_syscon_configure_gpio(struct device *dev,
				    struct regmap *gpio)
{
	struct device_node *node = dev->of_node;
	const void *prop = NULL;
	u32 gpio_conf[EQ_SYSCON_DT_REG_CONF_LEN] = { 0 };
	uint32_t first_gpio = 0xFFFF0000;
	uint32_t second_gpio = 0x0000FFFF;
	uint32_t value_read, value_conf;
	int gpio_conf_bytes, gpio_conf_num;
	int i, j, offset;
	int ret = 0;

	prop = of_get_property(node, "gpio_conf", &gpio_conf_bytes);
	if ((prop == NULL) || (gpio_conf_bytes == 0))
		return 0;

	/* gpio_conf_bytes represents number of bytes in this array,
	 * and gpio_conf_num represents number of entries in gpio_conf.
	 * Since each entry has X 4-byte elements, gpio_conf_num is
	 * derived from gpio_conf_bytes / (4bytes * X)
	 */
	gpio_conf_num =
		gpio_conf_bytes / (sizeof(u32) * EQ_SYSCON_DT_REG_CONF_LEN);
	dev_dbg(dev, "GPIO conf num: %d\n", gpio_conf_num);

	for (i = 0; i < gpio_conf_num; i++) {
		offset = i * EQ_SYSCON_DT_REG_CONF_LEN;

		for (j = 0; j < EQ_SYSCON_DT_REG_CONF_LEN; j++) {
			ret = of_property_read_u32_index(node, "gpio_conf",
							 offset + j, &gpio_conf[j]);
			if (ret) {
				dev_err(dev, "Failed to read gpio_conf[%d]\n", j);
				return ret;
			}
			dev_info(dev, "gpio_conf[%d][%d]: 0x%08X", i, j, gpio_conf[j]);
		}
		ret = regmap_read(gpio,
				gpio_conf[EQ_SYSCON_DT_REG_CONF_ADDR_OFFSET_IDX],
				&value_read);
		if (ret) {
			dev_err(dev, "Failed to read from gpio\n");
			return ret;
		}

		value_conf = gpio_conf[EQ_SYSCON_DT_REG_CONF_REG_VAL_IDX];
		/*checking which gpio is to be configured*/
		if ((value_conf & first_gpio) == first_gpio) {
			/* Clear the first GPIO value in original setting */
			value_read &= first_gpio;
			/* Keep the first GPIO value in the new setting */
			value_conf &= second_gpio;
		} else if ((value_conf & second_gpio) == second_gpio) {
			/* Clear the second GPIO value in original setting */
			value_read &= second_gpio;
			/* Keep the second GPIO value in the new setting */
			value_conf &= first_gpio;
		} else
			/* Clear both the GPIO values in original setting */
			value_read &= (first_gpio & second_gpio);
		value_read |= value_conf;//configure a GPIO pin
		ret = regmap_write(gpio,
				gpio_conf[EQ_SYSCON_DT_REG_CONF_ADDR_OFFSET_IDX],
				value_read);
		if (ret) {
			dev_err(dev, "Failed to write to gpio\n");
			return ret;
		}
	}

	return ret;
}

static int eq_syscon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int sif_ret = 0, gpio_ret = 0;

	eq_syscon.gpio =
		syscon_regmap_lookup_by_phandle(node, "eq_gpio_syscon");
	eq_syscon.sif = syscon_regmap_lookup_by_phandle(node, "eq_sif_syscon");

	if (IS_ERR(eq_syscon.gpio)) {
		dev_err(dev, "Failed to get gpio syscon %ld\n",
			PTR_ERR(eq_syscon.gpio));
	} else {
		dev_dbg(dev, "Successfully get gpio syscon\n");
		gpio_ret = eq_syscon_configure_gpio(dev, eq_syscon.gpio);
		if (gpio_ret)
			dev_err(dev, "Failed to configure GPIO %d\n", gpio_ret);
	}

	if (IS_ERR(eq_syscon.sif)) {
		dev_err(dev, "Failed to get sif syscon %ld\n",
			PTR_ERR(eq_syscon.sif));
	} else {
		dev_dbg(dev, "Successfully get sif syscon\n");
		sif_ret = eq_syscon_configure_sif(dev, eq_syscon.sif);
		if (sif_ret)
			dev_err(dev, "Failed to configure SIF %d\n", sif_ret);
	}

	return ((gpio_ret == 0) && (sif_ret == 0)) ? 0 : -EIO;
}

static int eq_syscon_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id eq_syscon_match_tbl[] = {
	{
		.compatible = "edgeq,syscon",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, eq_syscon_match_tbl);

static struct platform_driver eq_syscon_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = eq_syscon_match_tbl,
	},
	.probe = eq_syscon_probe,
	.remove = eq_syscon_remove,
};

module_platform_driver(eq_syscon_driver);

MODULE_AUTHOR("Timothy Huang <timothyhuang514@gmail.com>");
MODULE_DESCRIPTION("System controller for EdgeQ");
MODULE_LICENSE("GPL");
