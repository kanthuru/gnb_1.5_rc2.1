// SPDX-License-Identifier: GPL-2.0-only
#ifndef _XGBE_PHY_H__
#define _XGBE_PHY_H__

int xgmac_mdio_register(struct net_device *ndev);
int xgmac_mdio_unregister(struct net_device *ndev);

#endif // _XGBE_PHY_H__
