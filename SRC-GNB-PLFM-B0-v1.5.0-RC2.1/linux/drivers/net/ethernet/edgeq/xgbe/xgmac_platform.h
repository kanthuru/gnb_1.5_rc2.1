/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
  Platfrom driver header file.

*******************************************************************************/

#ifndef __XGMAC_PLATFORM_H__
#define __XGMAC_PLATFORM_H__

#include "xgbe.h"

struct plat_xgmacenet_data *
xgmac_probe_config_dt(struct platform_device *pdev, const char **mac);
void xgmac_remove_config_dt(struct platform_device *pdev,
			     struct plat_xgmacenet_data *plat);

int xgmac_get_platform_resources(struct platform_device *pdev,
				  struct xgmac_resources *xgmac_res);

int xgmac_pltfr_remove(struct platform_device *pdev);
extern const struct dev_pm_ops xgmac_pltfr_pm_ops;

static inline void *get_xgmac_bsp_priv(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);

	return priv->plat->bsp_priv;
}

#endif /* __XGMAC_PLATFORM_H__ */
