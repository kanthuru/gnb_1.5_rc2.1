/*
 * EGDEQ Raptor2 UIO driver (uio_raptor2)
 *
 * This driver exports EDGEQ raptor2 various maps
 * to user space for applications interacting with EDGEQ firmware
 *
 * Copyright (C) 2021-22 EdgeQ Inc - http://www.edgeq.io/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/of_reserved_mem.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/string.h>

#define DRV_NAME "edgeq_raptor2_uio"
#define DRV_VERSION "1.0"

#define UIO_MAX_MAP	64

#define CACHEABLE 1
#define NON_CACHEABLE 2
#define NON_CACHEABLE_PFETCH 3

#define KB(x)	((x) * 1024)
#define MB(x)	(KB (KB (x)))
#define GB(x)	(MB (KB (x)))

#define MAX_ADDR_MAP	5
#define MAX_SIZE_MAP	5

enum raptor2_uio_type {
	RAPTOR2_UIO_DU = BIT(0),
	RAPTOR2_UIO_GNB = BIT(1),
	RAPTOR2_UIO_WLAN = BIT(2),
	RAPTOR2_UIO_GNB_WLAN = (RAPTOR2_UIO_GNB | RAPTOR2_UIO_WLAN),
	RAPTOR2_UIO_GNB_DU = (RAPTOR2_UIO_DU | RAPTOR2_UIO_GNB),
	RAPTOR2_UIO_ALL = (RAPTOR2_UIO_DU | RAPTOR2_UIO_GNB | RAPTOR2_UIO_WLAN)
};

struct uio_raptor2_dev {
	struct uio_info *info;
	enum raptor2_uio_type deploy_type;
};

struct uio_raptor2_map {
	const char *name;
	phys_addr_t addr[MAX_ADDR_MAP];
	size_t size[MAX_SIZE_MAP];
	int memtype;
	int num_map;
	int irq_flag;
	struct uio_info *info;
	enum raptor2_uio_type deploy_type;
};

static struct uio_raptor2_map raptor2_map[UIO_MAX_MAP] = {
	{}
};

static const struct of_device_id raptor2_uio_of_match[] = {
	{.compatible = "edgeq,raptor2-uio-all",    .data = (void *)RAPTOR2_UIO_ALL},
	{.compatible = "edgeq,raptor2-uio-wlan", .data = (void *)RAPTOR2_UIO_WLAN},
	{.compatible = "edgeq,raptor2-uio-gnb", .data = (void *)RAPTOR2_UIO_GNB},
	{.compatible = "edgeq,raptor2-uio-du", .data = (void *)RAPTOR2_UIO_DU},
	{},
};

MODULE_DEVICE_TABLE(of, raptor2_uio_of_match);


static irqreturn_t raptor2_handler(int irq, struct uio_info *info)
{
	/* TODO */
	return IRQ_HANDLED;
}

static int uio_genirq_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	return 0;
}

static void raptor2_cleanup(struct device *dev, struct uio_raptor2_dev *gdev)
{
	int cnt;
	struct uio_info *p = gdev->info;

	for (cnt = 0; cnt < UIO_MAX_MAP; cnt++, p++)
		uio_unregister_device(p);

	devm_kfree(dev, gdev->info);
	devm_kfree(dev, gdev);
}

static int parse_uio_dt_memory(struct platform_device *pdev)
{
	int i = 0, num_reg = 0, tmp, j = 0;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *memnp;
	struct resource *rmem = NULL;
	int num_mem_hndl, num, rc, num_args = 3, num_reg_args = 4, offset;

	num_mem_hndl = of_count_phandle_with_args(of_node, "memory-region", NULL);
	num_mem_hndl = num_mem_hndl / num_args;
	for (num = 0; num < num_mem_hndl; num++) {
		offset = num * num_args;
		memnp = of_parse_phandle(of_node, "memory-region", offset);
		if (!memnp) {
			dev_err(dev, "no mem node\n");
			rc = -ENXIO;
		}
		if (!of_get_property(memnp, "reg", &tmp))
			return -ENXIO;
		num_reg = tmp / (sizeof(u32) * num_reg_args);
		rmem = kzalloc(num_reg * sizeof(*rmem), GFP_KERNEL);
		do {
			rc = of_address_to_resource(memnp, j, &rmem[j]);
			if (rc) {
				pr_err("failed to translate memory-region \
						to a resource");
				of_node_put(memnp);
				rc = -EINVAL;
				return rc;
			}
			raptor2_map[i].num_map = num_reg;
			raptor2_map[i].addr[j] = rmem[j].start;
			raptor2_map[i].size[j] = resource_size(&rmem[j]);
			j++;
		} while (j < num_reg);
		rc = of_property_read_u32_index(of_node,
				"memory-region", offset+1,
				&raptor2_map[i].memtype);
		rc = of_property_read_string_index(of_node,
				"mem-region-name", num,
				&(raptor2_map[i].name));
		rc = of_property_read_u32_index(of_node,
				"memory-region", offset+2,
				&raptor2_map[i].irq_flag);

		i++;
		j = 0;
		kfree(rmem);
		of_node_put(memnp);
	}
	return 0 ;
}

static int raptor2_probe(struct platform_device *pdev)
{
	struct uio_info *p;
	struct device *dev = &pdev->dev;
	struct uio_raptor2_dev *gdev;
	int ret, cnt, i, num_map, j = 0, irq_num = 0;
	const struct of_device_id *of_id;
	gdev = devm_kzalloc(dev, sizeof(struct uio_raptor2_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	gdev->info = devm_kcalloc(dev, UIO_MAX_MAP, sizeof(*p), GFP_KERNEL);
	if (!gdev->info) {
		devm_kfree(dev, gdev);
		return -ENOMEM;
	}

	of_id = of_match_device(raptor2_uio_of_match, &pdev->dev);
	if (of_id) {
		gdev->deploy_type = (enum raptor2_uio_type)of_id->data;
	}

	if (!gdev->deploy_type) {
		ret = -ENODEV;
		goto err;
	}

	ret = parse_uio_dt_memory(pdev);
	if (ret) {
		pr_err("Unable to parse memory");
		goto err;
	}
	p = gdev->info;
	while (cnt < UIO_MAX_MAP) {
		/* return if this is last device */
		if (!raptor2_map[cnt].addr[0])
			break;
		num_map = raptor2_map[cnt].num_map;
		p->name = raptor2_map[cnt].name;
		if(raptor2_map[cnt].irq_flag) {
			ret = platform_get_irq(pdev, irq_num);
			if (ret == -ENXIO)
				p->irq = UIO_IRQ_NONE;
			else if (ret == -EPROBE_DEFER)
				return ret;
			else if (ret < 0)
				return ret;
			else
				p->irq = ret;
			irq_num++;
		}
		for (j = 0; j < num_map; j++) {

			p->mem[j].addr = raptor2_map[cnt].addr[j];
			p->mem[j].size = raptor2_map[cnt].size[j];

			if (raptor2_map[cnt].memtype == CACHEABLE)
				p->mem[j].memtype = UIO_MEM_IOVA;
			else if (raptor2_map[cnt].memtype == NON_CACHEABLE)
				p->mem[j].memtype = UIO_MEM_PHYS;
			else if (raptor2_map[cnt].memtype == NON_CACHEABLE_PFETCH)
				p->mem[j].memtype = UIO_MEM_PHYS_PFETCH;
		}
		cnt++;

		p->version = DRV_VERSION;
		p->handler = raptor2_handler;
		p->irqcontrol = uio_genirq_irqcontrol;
		p->priv = gdev;
		ret = uio_register_device(dev, p);
		if (ret < 0)
			goto err_unloop;

		raptor2_map[cnt].info = p;
		p++;
	}

	platform_set_drvdata(pdev, gdev);
	return 0;

err_unloop:
	for (i = 0, p = gdev->info; i < cnt; i++, p++) {
		uio_unregister_device(p);
	}

err:
	devm_kfree(dev, gdev->info);
	devm_kfree(dev, gdev);

	return ret;
}

static int raptor2_remove(struct platform_device *dev)
{
	struct uio_raptor2_dev *gdev = platform_get_drvdata(dev);

	raptor2_cleanup(&dev->dev, gdev);
	return 0;
}

static struct platform_driver raptor2_driver = {
	.probe = raptor2_probe,
	.remove = raptor2_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(raptor2_uio_of_match),
	},
};

module_platform_driver(raptor2_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Ankit Jindal <ankit.jindal@edgeq.io>");
MODULE_AUTHOR("Pranavkumar Sawargaonkar <pranav.swargaonkar@edgeq.io>");
