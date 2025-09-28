// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2020 EdgeQ
 *
 * Adopted from dwmac-generic.c
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "xgbe.h"
#include "xgmac_platform.h"

static int xgbe_dwmac_probe(struct platform_device *pdev)
{
	struct plat_xgmacenet_data *plat_dat;
	struct xgmac_resources xgmac_res;
	int ret;

	ret = xgmac_get_platform_resources(pdev, &xgmac_res);
	if (ret)
		return ret;

	plat_dat = xgmac_probe_config_dt(pdev, &xgmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	/* Custom initialisation (if needed) */

	if (plat_dat->init) {
		ret = plat_dat->init(pdev, plat_dat->bsp_priv);
		if (ret)
			goto err_remove_config_dt;
	}

	ret = xgmac_dvr_probe(&pdev->dev, plat_dat, &xgmac_res);
	if (ret)
		goto err_remove_config_dt;

	return 0;

err_remove_config_dt:
	xgmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id xgbe_dwmac_match[] = {
	{ .compatible = "edgeq,xgbe" },
	{}
};
MODULE_DEVICE_TABLE(of, xgbe_dwmac_match);

static struct platform_driver xgbe_dwmac_driver = {
	.probe  = xgbe_dwmac_probe,
	.remove = xgmac_pltfr_remove,
	.driver = {
		.name           = "xgbe",
		.pm		= &xgmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(xgbe_dwmac_match),
	},
};
//module_platform_driver(xgbe_dwmac_driver);

int xgbe_driver_init(void)
{
	mdio_lock_init_done = false;
	return platform_driver_register(&xgbe_dwmac_driver);
}

void xgbe_driver_exit(void)
{
        platform_driver_unregister(&xgbe_dwmac_driver);
}

MODULE_LICENSE("GPL v2");
