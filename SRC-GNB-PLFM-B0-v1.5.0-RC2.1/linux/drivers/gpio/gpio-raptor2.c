// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EdgeQ SoC GPIO driver
 *
 * Copyright (C) 2022 EdgeQ Inc.
 * Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include "gpiolib.h"

#define GPIO_LINE_DIRECTION_IN  1
#define GPIO_LINE_DIRECTION_OUT 0
#define RAPTOR2_GPIO_SOC_VARIANT_A0 1
#define RAPTOR2_GPIO_SOC_VARIANT_B0 2

#define GPIO_OUT_0	0xD8
#define GPIO_OE_0	0xC8
/* Input pin status register */
#define GPIO_IN_0	0x370

struct raptor2_gpio {
	void __iomem *reg_base;
	spinlock_t lock;
	struct gpio_chip gpio_chip;
	struct platform_device *pdev;
};

static const struct of_device_id raptor2_gpio_of_match[] = {
	{
		.compatible = "raptor2-a0-gpio",
		.data       = (void *) RAPTOR2_GPIO_SOC_VARIANT_A0,
	},
	{
		.compatible = "raptor2-b0-rfgpio",
		.data       = (void *) RAPTOR2_GPIO_SOC_VARIANT_B0,
	},
	{ /* sentinel */ }
};

/*
 * Returns direction for signal "offset/gpio", 0=out, 1=in,
 *      (same as GPIO_LINE_DIRECTION_OUT / GPIO_LINE_DIRECTION_IN),
 */
static int raptor2_gpio_get_dir(struct gpio_chip *chip, unsigned int gpio)
{
	struct raptor2_gpio *r2_gpio = gpiochip_get_data(chip);
	void __iomem *reg_base = r2_gpio->reg_base;
	u32 val;

	val = readl(reg_base + GPIO_OE_0) & (1 << gpio);
	val = val >> gpio;

	return val ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

/*
 * Assigns output value for signal "offset/gpio"
 */
static void raptor2_gpio_set(struct gpio_chip *chip, unsigned int gpio, int value)
{
	struct raptor2_gpio *r2_gpio;
	void __iomem *reg_base;
	struct device *dev;
	u32 val;
	unsigned long flags;

	r2_gpio = gpiochip_get_data(chip);
	reg_base = r2_gpio->reg_base;
	dev = &r2_gpio->pdev->dev;

	spin_lock_irqsave(&r2_gpio->lock, flags);
	/* this function only applies to output pin */
	if (raptor2_gpio_get_dir(chip, gpio) == GPIO_LINE_DIRECTION_IN) {
		dev_err(dev, "This is input pin, can not assign value to it.\n");
		goto out;
	} else {
		val = readl(reg_base + GPIO_OUT_0) & ~(1 << gpio);
		writel(val | (value << gpio), reg_base + GPIO_OUT_0);
	}

out:
	spin_unlock_irqrestore(&r2_gpio->lock, flags);
}

/*
 * Returns value for signal "offset/gpio", 0=low, 1=high
 */
static int raptor2_gpio_get(struct gpio_chip *chip, unsigned int gpio)
{
	struct raptor2_gpio *r2_gpio = gpiochip_get_data(chip);
	void __iomem *reg_base = r2_gpio->reg_base;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&r2_gpio->lock, flags);

	if (readl(reg_base + GPIO_OE_0) & (1 << gpio)) {
		/* Read gpio value in output mode
		 * NOTE: This is just reg value
		 */
		val = readl(reg_base + GPIO_OUT_0) & ~(0 << gpio);
		spin_unlock_irqrestore(&r2_gpio->lock, flags);
		return (val >> gpio) & 0x1;
	} else {
		/* Read gpio value in input mode */
		val = readl(reg_base + GPIO_IN_0) & ~(0 << gpio);
		spin_unlock_irqrestore(&r2_gpio->lock, flags);
		return (val >> gpio) & 0x1;

	}
}

/*
 * Configures signal "offset/gpio" as input
 */
static int raptor2_gpio_direction_input(struct gpio_chip *chip, unsigned int gpio)
{
	struct raptor2_gpio *r2_gpio = gpiochip_get_data(chip);
	void __iomem *reg_base = r2_gpio->reg_base;
	u32 val = 0;
	unsigned long flags;

	spin_lock_irqsave(&r2_gpio->lock, flags);
	/* Disable OE */
	val = readl(reg_base + GPIO_OE_0) | ~(1 << gpio);
	writel(val, reg_base + GPIO_OE_0);
	spin_unlock_irqrestore(&r2_gpio->lock, flags);

	return 0;
}

static void gpio_rmw(void __iomem *addr, u32 mask, u32 val)
{
	u32 rdat = readl(addr);

	rdat = rdat & (~mask);
	rdat = rdat | val;
	writel(rdat, addr);
}

/*
 * Configures signal "offset/gpio" as output
 */
static int raptor2_gpio_direction_output(struct gpio_chip *chip,
					  unsigned int gpio, int value)
{
	struct raptor2_gpio *r2_gpio = gpiochip_get_data(chip);
	void __iomem *reg_base = r2_gpio->reg_base;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&r2_gpio->lock, flags);

	val = readl(reg_base + GPIO_OUT_0) & ~(1 << gpio);
	writel(val | (value << gpio), reg_base + GPIO_OUT_0);

	val = 1 << gpio;
	/* Program OE(“1”) to set gpio as output */
	gpio_rmw(reg_base + GPIO_OE_0, val, val);
	spin_unlock_irqrestore(&r2_gpio->lock, flags);

	return 0;
}

/*
 * Optional hook for all kinds of settings
 */
static int raptor2_gpio_set_config(struct gpio_chip *chip, unsigned int gpio,
				    unsigned long config)
{
	return 0;
}

/*
 * Optional hook for chip-specific activation, such as
 *      enabling module power and clock
 */
static int raptor2_gpio_request(struct gpio_chip *chip, unsigned int gpio)
{
	return 0;
}

/*
 * Optional hook for chip-specific deactivation, such as
 *      disabling module power and clock
 */
static void raptor2_gpio_free(struct gpio_chip *chip, unsigned int gpio)
{
}

static const struct gpio_chip template_chip = {
	.label = "raptor2-rf-gpio",
	.owner = THIS_MODULE,
	.request = raptor2_gpio_request,
	.free = raptor2_gpio_free,
	.get_direction = raptor2_gpio_get_dir,
	.direction_input = raptor2_gpio_direction_input,
	.get = raptor2_gpio_get,
	.direction_output = raptor2_gpio_direction_output,
	.set = raptor2_gpio_set,
	.set_config = raptor2_gpio_set_config,
	.base = -1,
};

static int raptor2_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct raptor2_gpio *r2_gpio;
	struct gpio_chip *chip;
	int ret, ngpio;

	match = of_match_device(raptor2_gpio_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find gpio controller\n");
		return -ENODEV;
	} else
		dev_info(dev, "Found gpio controller !\n");

	r2_gpio = devm_kzalloc(dev, sizeof(*r2_gpio), GFP_KERNEL);
	if (!r2_gpio)
		return -ENOMEM;

	r2_gpio->gpio_chip = template_chip;
	chip = &r2_gpio->gpio_chip;

	if (of_property_read_u32(pdev->dev.of_node, "ngpios", &ngpio)) {
		dev_err(&pdev->dev, "Missing ngpios OF property\n");
		ret = -ENODEV;
		goto free_r2_gpio;
	}

	chip->ngpio = ngpio;
	r2_gpio->pdev = pdev;
	platform_set_drvdata(pdev, r2_gpio);
	chip->of_node = dev->of_node;

	r2_gpio->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(r2_gpio->reg_base)) {
		ret = PTR_ERR(r2_gpio->reg_base);
		goto free_r2_gpio;
	}

	dev_info(&pdev->dev, "Setting up Raptor2 GPIO\n");

	ret = devm_gpiochip_add_data(dev, chip, r2_gpio);
	if (ret < 0) {
		dev_err(dev, "Couldn't add GPIO chip -- %d\n", ret);
	}

	spin_lock_init(&r2_gpio->lock);

	return 0;

free_r2_gpio:
	devm_kfree(dev, r2_gpio);

	return ret;
}

static int raptor2_gpio_remove(struct platform_device *pdev)
{
	struct raptor2_gpio *r2_gpio = platform_get_drvdata(pdev);
	struct gpio_chip *gc = &r2_gpio->gpio_chip;

	gpiochip_remove(gc);
	free_bucket_spinlocks(&r2_gpio->lock);
	devm_kfree(&pdev->dev, r2_gpio);

	return 0;
}

static struct platform_driver raptor2_gpio_driver = {
	.driver         = {
		.owner = THIS_MODULE,
		.name = "raptor2-rfgpio",
		.of_match_table = raptor2_gpio_of_match,
	},
	.probe = raptor2_gpio_probe,
	.remove = raptor2_gpio_remove,
};

MODULE_AUTHOR("Nilesh Waghmare <nilesh.waghmare@edgeq.io>");
MODULE_DESCRIPTION("EdgeQ SoC RFGPIO driver");
MODULE_LICENSE("GPL");
builtin_platform_driver(raptor2_gpio_driver);
