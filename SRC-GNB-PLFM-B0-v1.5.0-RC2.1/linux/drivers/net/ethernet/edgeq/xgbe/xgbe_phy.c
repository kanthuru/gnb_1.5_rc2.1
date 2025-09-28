// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  Provides interface to the external PHY

  Copyright (c) 2021  EdgeQ Inc
*******************************************************************************/

#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/slab.h>

#include "dwxgmac.h"
#include "xgbe.h"
#include "xgbe_misc.h"
#include "xgbe_phy.h"

void __iomem *link2_mdio_mac_base;

/* Link1 and Link2 share the same MDIO. All 4 ports on Link2 share
 * the same MDIO. Use single MDIO lock to protect access to MDIO.
 */
static struct mutex mdio_lock;
bool mdio_lock_init_done;

static int xgbe_xgmac_c45_format(struct xgmac_priv *priv, int phyaddr,
				 int phyreg, u32 *hw_addr)
{
	void __iomem *ioaddr;
	u32 tmp;

	if (priv->plat->eth_link == 1)
		ioaddr = priv->ioaddr;
	else
		ioaddr = link2_mdio_mac_base;
	if (ioaddr == 0)
		return -EFAULT;

	/* Set port as Clause 45 */
	tmp = readl(ioaddr + XGMAC_MDIO_C22P);
	tmp &= ~BIT(phyaddr);
	writel(tmp, ioaddr + XGMAC_MDIO_C22P);

	*hw_addr = (phyaddr << XGMAC_PA_SHIFT) | (phyreg & 0xffff);
	*hw_addr |= (phyreg >> MII_DEVADDR_C45_SHIFT) << XGMAC_DA_SHIFT;
	return 0;
}

static int xgbe_xgmac_c22_format(struct xgmac_priv *priv, int phyaddr,
				 int phyreg, u32 *hw_addr)
{
	void __iomem *ioaddr;
	u32 tmp;

	if (priv->plat->eth_link == 1)
		ioaddr = priv->ioaddr;
	else
		ioaddr = link2_mdio_mac_base;
	if (ioaddr == 0)
		return -EFAULT;

	/* HW does not support C22 addr >= 4 */
	if (phyaddr > XGMAC_MAX_C22ADDR)
		return -ENODEV;

	/* Set port as Clause 22 */
	tmp = readl(ioaddr + XGMAC_MDIO_C22P);
	tmp &= ~XGMAC_C22P_MASK;
	tmp |= BIT(phyaddr);
	writel(tmp, ioaddr + XGMAC_MDIO_C22P);

	*hw_addr = (phyaddr << XGMAC_PA_SHIFT) | (phyreg & 0x1f);
	return 0;
}

static int xgbe_xgmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	struct net_device *ndev = bus->priv;
	struct xgmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	void __iomem *ioaddr;
	u32 tmp, addr;
	u32 value = XGMAC_BUSY;
	int mmd;
	int ret;

	if (priv->plat->eth_link == 1)
		ioaddr = priv->ioaddr;
	else
		ioaddr = link2_mdio_mac_base;
	if (ioaddr == 0)
		return -EFAULT;

	mutex_lock(&mdio_lock);

	xgbe_mdio_mux_config(priv);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(ioaddr + mii_data, tmp,
			       !(tmp & XGMAC_BUSY), 100, 10000)) {
		mutex_unlock(&mdio_lock);
		return -EBUSY;
	}

	if (phyreg & MII_ADDR_C45) {
		/* Linux phy_c45_probe_present() function reads VNED1 STAT2
		 * register. That doesn't go through the phy driver. Reading
		 * this register of RTL8221B-VB phy can cause the phy to
		 * loss the link under certain conditions. Thus block reading
		 * this register.
		 */
		mmd = (phyreg >> MII_DEVADDR_C45_SHIFT) & 0x1f;
		if (mmd == MDIO_MMD_VEND1 && (phyreg & 0xffff) == MDIO_STAT2) {
			mutex_unlock(&mdio_lock);
			return 0;
		}

		phyreg &= ~MII_ADDR_C45;
		ret = xgbe_xgmac_c45_format(priv, phyaddr, phyreg, &addr);
		if (ret) {
			mutex_unlock(&mdio_lock);
			return ret;
		}
	} else {
		ret = xgbe_xgmac_c22_format(priv, phyaddr, phyreg, &addr);
		if (ret) {
			mutex_unlock(&mdio_lock);
			return ret;
		}
		value |= XGMAC_SADDR;
	}

	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	value |= ((XGMAC_CMD_READ << XGMAC_CMD_SHIFT) & XGMAC_CMD_MASK);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(ioaddr + mii_data, tmp,
			       !(tmp & XGMAC_BUSY), 100, 10000)) {
		mutex_unlock(&mdio_lock);
		return -EBUSY;
	}

	/* Set the MII address register to read */
	writel(addr, ioaddr + mii_address);
	writel(value, ioaddr + mii_data);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(ioaddr + mii_data, tmp,
			       !(tmp & XGMAC_BUSY), 100, 10000)) {
		mutex_unlock(&mdio_lock);
		return -EBUSY;
	}

	/* Read the data from the MII data register */
	ret = readl(ioaddr + mii_data) & GENMASK(15, 0);

	mutex_unlock(&mdio_lock);

	return ret;
}

static int xgbe_xgmac_mdio_write(struct mii_bus *bus, int phyaddr,
				 int phyreg, u16 phydata)
{
	struct net_device *ndev = bus->priv;
	struct xgmac_priv *priv = netdev_priv(ndev);
	unsigned int mii_address = priv->hw->mii.addr;
	unsigned int mii_data = priv->hw->mii.data;
	void __iomem *ioaddr;
	u32 addr, tmp;
	u32 value = XGMAC_BUSY;
	int ret;

	if (priv->plat->eth_link == 1)
		ioaddr = priv->ioaddr;
	else
		ioaddr = link2_mdio_mac_base;
	if (ioaddr == 0)
		return -EFAULT;

	mutex_lock(&mdio_lock);

	xgbe_mdio_mux_config(priv);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(ioaddr + mii_data, tmp,
			       !(tmp & XGMAC_BUSY), 100, 10000)) {
		mutex_unlock(&mdio_lock);
		return -EBUSY;
	}

	if (phyreg & MII_ADDR_C45) {
		phyreg &= ~MII_ADDR_C45;
		ret = xgbe_xgmac_c45_format(priv, phyaddr, phyreg, &addr);
		if (ret) {
			mutex_unlock(&mdio_lock);
			return ret;
		}
	} else {
		ret = xgbe_xgmac_c22_format(priv, phyaddr, phyreg, &addr);
		if (ret) {
			mutex_unlock(&mdio_lock);
			return ret;
		}
		value |= XGMAC_SADDR;
	}

	value |= (priv->clk_csr << priv->hw->mii.clk_csr_shift)
		& priv->hw->mii.clk_csr_mask;
	value |= phydata;
	value |= ((XGMAC_CMD_WRITE << XGMAC_CMD_SHIFT) & XGMAC_CMD_MASK);

	/* Wait until any existing MII operation is complete */
	if (readl_poll_timeout(ioaddr + mii_data, tmp,
			       !(tmp & XGMAC_BUSY), 100, 10000)) {
		mutex_unlock(&mdio_lock);
		return -EBUSY;
	}

	/* Set the MII address register to write */
	writel(addr, ioaddr + mii_address);
	writel(value, ioaddr + mii_data);

	/* Wait until any existing MII operation is complete */
	ret = readl_poll_timeout(ioaddr + mii_data, tmp,
				 !(tmp & XGMAC_BUSY), 100, 10000);

	mutex_unlock(&mdio_lock);

	return ret;
}

int xgmac_mdio_reset(struct mii_bus *bus)
{
	return 0;
}

int xgmac_mdio_register(struct net_device *ndev)
{
	int err = 0;
	struct mii_bus *new_bus;
	struct xgmac_priv *priv = netdev_priv(ndev);
	struct xgmac_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;
	struct device_node *mdio_node = priv->plat->mdio_node;
	struct device *dev = ndev->dev.parent;
	int addr, found, max_addr;

	if (!mdio_bus_data)
		return 0;

	new_bus = mdiobus_alloc();
	if (!new_bus)
		return -ENOMEM;

	if (mdio_bus_data->irqs)
		memcpy(new_bus->irq, mdio_bus_data->irqs, sizeof(new_bus->irq));

	new_bus->name = "xgmac";

	if (priv->plat->eth_link == 2 && priv->plat->phy_lane == 0)
		link2_mdio_mac_base = priv->ioaddr;

	if (!mdio_lock_init_done) {
		mutex_init(&mdio_lock);
		mdio_lock_init_done = true;
	}

	new_bus->read = &xgbe_xgmac_mdio_read;
	new_bus->write = &xgbe_xgmac_mdio_write;

	if (mdio_bus_data->needs_reset)
		new_bus->reset = &xgmac_mdio_reset;

	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 new_bus->name, priv->plat->bus_id);
	new_bus->priv = ndev;
	new_bus->phy_mask = mdio_bus_data->phy_mask;
	new_bus->parent = priv->device;

	err = of_mdiobus_register(new_bus, mdio_node);
	if (err != 0) {
		dev_err(dev, "Cannot register the MDIO bus\n");
		goto bus_register_fail;
	}

	/* Looks like we need a dummy read for XGMAC only and C45 PHYs */
	xgbe_xgmac_mdio_read(new_bus, 0, MII_ADDR_C45);

	if (priv->plat->phy_node || mdio_node)
		goto bus_register_done;

	found = 0;
	max_addr = PHY_MAX_ADDR;
	for (addr = 0; addr < max_addr; addr++) {
		struct phy_device *phydev = mdiobus_get_phy(new_bus, addr);

		if (!phydev)
			continue;

		/* If an IRQ was provided to be assigned after
		 * the bus probe, do it here.
		 */
		if (!mdio_bus_data->irqs &&
		    mdio_bus_data->probed_phy_irq > 0) {
			new_bus->irq[addr] = mdio_bus_data->probed_phy_irq;
			phydev->irq = mdio_bus_data->probed_phy_irq;
		}

		/* If we're going to bind the MAC to this PHY bus,
		 * and no PHY number was provided to the MAC,
		 * use the one probed here.
		 */
		if (priv->plat->phy_addr == -1)
			priv->plat->phy_addr = addr;

		phy_attached_info(phydev);
		found = 1;
	}

	if (!found && !mdio_node) {
		dev_warn(dev, "No PHY found\n");
		mdiobus_unregister(new_bus);
		mdiobus_free(new_bus);
		return -ENODEV;
	}

bus_register_done:
	priv->mii = new_bus;
	priv->phydev = mdiobus_get_phy(new_bus, priv->plat->phy_addr);
	((struct phy_device *)priv->phydev)->interface =
						priv->plat->phy_interface;

	return 0;

bus_register_fail:
	mdiobus_free(new_bus);
	return err;
}

int xgmac_mdio_unregister(struct net_device *ndev)
{
	struct xgmac_priv *priv = netdev_priv(ndev);

	if (!priv->mii)
		return 0;

	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	mdiobus_free(priv->mii);
	priv->mii = NULL;

	return 0;
}
