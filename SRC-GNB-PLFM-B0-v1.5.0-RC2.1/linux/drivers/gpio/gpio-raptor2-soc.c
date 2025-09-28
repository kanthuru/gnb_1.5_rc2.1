// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EdgeQ SoC GPIO driver
 *
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Akhilesh Kadam <c_sanjaykadam@edgeq.io>
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
#define RAPTOR2_GPIO_SOC_VARIANT_B0 2

#define GPIO_OUT_0          4
#define GPIO_OE_0           6
/* Input pin status register */
#define GPIO_IE_0           5
#define GPIO_OFFSET         0x20
#define DS_MASK             15     /* Set drive strength to max */
#define DS_MASK_OFFSET_1    9
#define DS_MASK_OFFSET_2    25
#define NEXT_GPIO_OFFSET    16

struct raptor2_soc_gpio {
	void __iomem *reg_base;
	struct gpio_chip gpio_chip;
	struct platform_device *pdev;
};

void __iomem *soc_gpio_base;

static const struct of_device_id raptor2_soc_gpio_of_match[] = {
	{
		.compatible = "raptor2-b0-socgpio",
		.data       = (void *) RAPTOR2_GPIO_SOC_VARIANT_B0,
	},
	{ /* sentinel */ }
};

static void soc_gpio_rmw(void __iomem *addr, unsigned int gpio)
{
	u32 rdat = readl(addr);

	if (gpio % 2 == 0)
		rdat = rdat | (DS_MASK << DS_MASK_OFFSET_1);
	else
		rdat = rdat | (DS_MASK << DS_MASK_OFFSET_2);

	writel(rdat, addr);
}

static int raptor2_soc_gpio_get_dir(struct gpio_chip *chip1, unsigned int gpio)
{
	struct raptor2_soc_gpio *r2_soc_gpio = gpiochip_get_data(chip1);
	void __iomem *reg_base = r2_soc_gpio->reg_base;
	u32 val;

	if (gpio % 2 == 0) {
		val = readl(reg_base + GPIO_OFFSET + (gpio * 2));
		val = val >> GPIO_OE_0;
	} else {
		val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2));
		val = val >> (GPIO_OE_0 + NEXT_GPIO_OFFSET);
	}
	return val ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static void raptor2_soc_gpio_set(struct gpio_chip *chip1, unsigned int gpio, int value)
{
	struct raptor2_soc_gpio *r2_soc_gpio;
	void __iomem *reg_base;
	struct device *dev;
	u32 val;

	r2_soc_gpio = gpiochip_get_data(chip1);
	reg_base = r2_soc_gpio->reg_base;
	dev = &r2_soc_gpio->pdev->dev;

	if (raptor2_soc_gpio_get_dir(chip1, gpio) == GPIO_LINE_DIRECTION_IN) {
		dev_err(dev, "This is input pin, can not assign value to it.\n");

	} else {
		if (gpio % 2 == 0) {
			val = readl(reg_base + GPIO_OFFSET + (gpio * 2)) & ~(1 << GPIO_OUT_0);
			writel(val | (value << GPIO_OUT_0), reg_base + GPIO_OFFSET + (gpio * 2));
		} else {
			val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				~(1 << (GPIO_OUT_0 + NEXT_GPIO_OFFSET));
			writel(val | (value << (GPIO_OUT_0 + NEXT_GPIO_OFFSET)),
					reg_base + GPIO_OFFSET + ((gpio - 1) * 2));
		}
	}

}

static int raptor2_soc_gpio_get(struct gpio_chip *chip1, unsigned int gpio)
{
	struct raptor2_soc_gpio *r2_soc_gpio = gpiochip_get_data(chip1);
	void __iomem *reg_base = r2_soc_gpio->reg_base;
	u32 val;

	if (gpio % 2 == 0) {
		val = readl(reg_base + GPIO_OFFSET + (gpio * 2));
		if (readl(reg_base + GPIO_OFFSET + (gpio * 2)) & (1 << GPIO_OE_0))
			/* Read gpio value in output mode
			 * NOTE: This is just reg value
			 */
			return (val >> GPIO_OUT_0) & 0x1;
		else
			/* Read gpio value in input mode */
			return (val >> GPIO_OUT_0) & 0x1;
	} else {
		val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2));
		if (readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				(1 << (GPIO_OE_0 + NEXT_GPIO_OFFSET)))
			/* Read gpio value in output mode
			 * NOTE: This is just reg value
			 */
			return (val >> (GPIO_OUT_0 + NEXT_GPIO_OFFSET)) & 0x1;
		else
			/* Read gpio value in input mode */
			return (val >> (GPIO_OUT_0 + NEXT_GPIO_OFFSET)) & 0x1;

	}
	return -1;
}

static int raptor2_soc_gpio_direction_input(struct gpio_chip *chip1, unsigned int gpio)
{
	struct raptor2_soc_gpio *r2_soc_gpio = gpiochip_get_data(chip1);
	void __iomem *reg_base = r2_soc_gpio->reg_base;
	u32 val = 0;

	/* Disable OE */
	if (gpio % 2 == 0) {
		if (readl(reg_base + GPIO_OFFSET + (gpio * 2)) & (1 << GPIO_OE_0)) {
			val = readl(reg_base + GPIO_OFFSET + (gpio * 2)) & ~(1 << GPIO_OE_0);
			writel(val, reg_base + GPIO_OFFSET + (gpio * 2));
		}
		val = readl(reg_base + GPIO_OFFSET + (gpio * 2)) | (1 << GPIO_IE_0);
		writel(val, (reg_base + GPIO_OFFSET + (gpio * 2)));

	} else {
		if (readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				(1 << (GPIO_OE_0 + NEXT_GPIO_OFFSET))) {
			val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				~(1 << (GPIO_OE_0 + NEXT_GPIO_OFFSET));
			writel(val, (reg_base + GPIO_OFFSET + ((gpio - 1) * 2)));
		}
		val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) |
			(1 << (GPIO_IE_0 + NEXT_GPIO_OFFSET));
		writel(val, (reg_base + GPIO_OFFSET + ((gpio - 1) * 2)));
	}
	return 0;
}

static int raptor2_soc_gpio_direction_output(struct gpio_chip *chip1,
		unsigned int gpio, int value)
{
	struct raptor2_soc_gpio *r2_soc_gpio = gpiochip_get_data(chip1);
	void __iomem *reg_base = r2_soc_gpio->reg_base;
	u32 val;

	if (gpio % 2 == 0) {
		soc_gpio_rmw(reg_base + GPIO_OFFSET + (gpio * 2), gpio);
		val = readl(reg_base + GPIO_OFFSET + (gpio * 2));
		if (readl(reg_base + GPIO_OFFSET + (gpio * 2)) & (1 << GPIO_IE_0)) {
			val = readl(reg_base + (GPIO_OFFSET + (gpio * 2))) & ~(1 << GPIO_IE_0);
			writel(val, reg_base + GPIO_OFFSET + (gpio * 2));
		}
		/* Program OE(“1”) as output*/
		val = readl(reg_base + GPIO_OFFSET + (gpio * 2)) | (1 << GPIO_OE_0);
		val = val & ~(1 << GPIO_OUT_0);
		writel(val | (value << GPIO_OUT_0), (reg_base + GPIO_OFFSET + (gpio * 2)));
	} else {
		soc_gpio_rmw(reg_base + GPIO_OFFSET + ((gpio - 1) * 2), gpio);
		val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2));
		if (readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				(1 << (GPIO_IE_0 + NEXT_GPIO_OFFSET))) {
			val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)) &
				~(1 << (GPIO_IE_0 + NEXT_GPIO_OFFSET));
			writel(val, reg_base + GPIO_OFFSET + ((gpio - 1) * 2));
		}
		/* Program OE(“1”) as output */
		val = readl(reg_base + GPIO_OFFSET + ((gpio - 1) * 2))
			| (1 << (GPIO_OE_0 + NEXT_GPIO_OFFSET));
		val = val & ~(1 << (GPIO_OUT_0 + NEXT_GPIO_OFFSET));
		writel(val | (value << (GPIO_OUT_0 + NEXT_GPIO_OFFSET)),
			(reg_base + GPIO_OFFSET + ((gpio - 1) * 2)));
	}
	return 0;
}


static int raptor2_soc_gpio_set_config(struct gpio_chip *chip1, unsigned int gpio,
		unsigned long config)
{
	return 0;
}

static int raptor2_soc_gpio_request(struct gpio_chip *chip1, unsigned int gpio)
{
	return 0;
}

static void raptor2_soc_gpio_free(struct gpio_chip *chip1, unsigned int gpio)
{
}

static const struct gpio_chip template_chip = {
	.label = "raptor2-soc-gpio",
	.owner = THIS_MODULE,
	.request = raptor2_soc_gpio_request,
	.free = raptor2_soc_gpio_free,
	.get_direction = raptor2_soc_gpio_get_dir,
	.direction_input = raptor2_soc_gpio_direction_input,
	.get = raptor2_soc_gpio_get,
	.direction_output = raptor2_soc_gpio_direction_output,
	.set = raptor2_soc_gpio_set,
	.set_config = raptor2_soc_gpio_set_config,
	.base = -1,
};

/* Function to retrieve the remapped virtual address */
void __iomem *get_ioremapped_gpio_base(void)
{
	return soc_gpio_base;
}
EXPORT_SYMBOL_GPL(get_ioremapped_gpio_base);

static int raptor2_soc_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct raptor2_soc_gpio *r2_soc_gpio;
	struct gpio_chip *chip1;
	int ret, ngpio;

	match = of_match_device(raptor2_soc_gpio_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find gpio controller\n");
		return -ENODEV;
	} else
		dev_info(dev, "Found controller\n");

	r2_soc_gpio = devm_kzalloc(dev, sizeof(*r2_soc_gpio), GFP_KERNEL);
	if (!r2_soc_gpio)
		return -ENOMEM;

	r2_soc_gpio->gpio_chip = template_chip;
	chip1 = &r2_soc_gpio->gpio_chip;

	if (of_property_read_u32(pdev->dev.of_node, "ngpios", &ngpio)) {
		dev_err(&pdev->dev, "Missing ngpios OF property\n");
		ret = -ENODEV;
		goto free_r2_gpio;
	}

	chip1->ngpio = ngpio;
	r2_soc_gpio->pdev = pdev;
	platform_set_drvdata(pdev, r2_soc_gpio);
	chip1->of_node = dev->of_node;

	r2_soc_gpio->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(r2_soc_gpio->reg_base)) {
		ret = PTR_ERR(r2_soc_gpio->reg_base);
		goto free_r2_gpio;
	}

	soc_gpio_base = r2_soc_gpio->reg_base;

	dev_info(&pdev->dev, "Setting up Raptor2 GPIO\n");

	ret = devm_gpiochip_add_data(dev, chip1, r2_soc_gpio);
	if (ret < 0) {
		dev_err(dev, "Couldn't add GPIO chip -- %d\n", ret);
	}

	return 0;

free_r2_gpio:
	devm_kfree(dev, r2_soc_gpio);

	return ret;
}

static int raptor2_soc_gpio_remove(struct platform_device *pdev)
{
	struct raptor2_soc_gpio *r2_soc_gpio = platform_get_drvdata(pdev);
	struct gpio_chip *gc = &r2_soc_gpio->gpio_chip;

	gpiochip_remove(gc);
	devm_kfree(&pdev->dev, r2_soc_gpio);

	return 0;
}

static struct platform_driver raptor2_soc_gpio_driver = {
	.driver         = {
		.owner = THIS_MODULE,
		.name = "raptor2-soc-gpio",
		.of_match_table = raptor2_soc_gpio_of_match,
	},
	.probe = raptor2_soc_gpio_probe,
	.remove = raptor2_soc_gpio_remove,
};

MODULE_AUTHOR("Akhilesh Kadam <c_sanjaykadam@edgeq.io>");
MODULE_AUTHOR("Nilesh Waghmare <nilesh.waghmare@edgeq.io>");
MODULE_DESCRIPTION("EdgeQ SoC GPIO driver");
MODULE_LICENSE("GPL");
builtin_platform_driver(raptor2_soc_gpio_driver);
