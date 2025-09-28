// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2020 EdgeQ Inc
 */

/*
 * Designware XGMAC/XLGMAC ethernet driver for U-Boot
 */

#include <common.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <miiphy.h>
#include <malloc.h>
#include <net.h>
#include <reset.h>
#include <asm/cache.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include "xgbe_xgmac.h"
#ifdef CONFIG_RAPTOR_FAMILY
#include "xgbe_misc.h"
#endif
#ifdef CONFIG_EQXPCS
#include "xgbe_xpcs.h"
#endif
#ifdef CONFIG_DM_ETH_PHY
#include "xgbe_phy.h"
#endif

extern phys_addr_t link2_mdio_mac_base;
extern u32 env_board_type;

#define BOARD_HAWK	1
static void set_mac_speed(struct dw_eth_dev *priv, int speed);

/**
 * rmwritel - Perform a read-modify-write operation on a 32-bit register.
 * @reg_addr: Address of the register to modify
 * @mask: Bitmask to apply on the register value
 * @val: Value to write to the register after applying bitmask
 * @brief:
 * This function performs a read-modify-write operation on the specified 32-bit
 * register. It first reads the current value of the register, then applies the
 * provided bitmask on it to selectively modify the required bits. Finally, the
 * modified value is written back to the register.	*/
static inline void rmwritel(phys_addr_t reg_addr, uint32_t mask, uint32_t val)
{
	uint32_t reg_val;

	/* Read the current value of the register	*/
	reg_val = readl(reg_addr);

	/* Modify the register value using bitwise operations	*/
	val &= mask;
	reg_val &= ~mask;
	reg_val |= val;

	/* Write the modified value back to the register	*/
	writel(reg_val, reg_addr);
}

#if CONFIG_IS_ENABLED(DM_MDIO)
static int dw_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	struct udevice *dev = bus->priv;
	struct dw_eth_dev *priv = dev_get_priv(dev);
	phys_addr_t iobase;
	u32 mii_data = MII_BUSY;
	u32 mii_addr = 0;
	ulong start;

	if (devad == MDIO_DEVAD_NONE)
		return -EOPNOTSUPP;

	if (priv->eth_link == 1)
		iobase = priv->mac_base;
	else
		iobase = link2_mdio_mac_base;

	if (iobase == 0)
		return -EFAULT;

	xgbe_mdio_mux_config(priv);

	mii_addr |= ((devad << MII_DA_SHIFT) & MII_DA_MASK);
	mii_addr |= ((addr << MII_ADDR_SHIFT) & MII_ADDR_MASK);
	mii_addr |= ((reg << MII_REG_SHIFT) & MII_REG_MASK);
	mii_data |= ((MII_CLK_400_450M << MII_CLK_SHIFT) & MII_CLK_MASK);
	mii_data |= ((MII_CMD_READ << MII_CMD_SHIFT) & MII_CMD_MASK);

	writel(mii_addr, iobase + MAC_MDIO_ADDR);
	writel(mii_data, iobase + MAC_MDIO_DATA);

	start = get_timer(0);
	while (get_timer(start) < MDIO_TIMEOUT) {
		if (!(readl(iobase + MAC_MDIO_DATA) & MII_BUSY))
			return readl(iobase + MAC_MDIO_DATA);
		udelay(10);
	};

	return -ETIMEDOUT;
}

static int dw_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			u16 val)
{
	struct udevice *dev = bus->priv;
	struct dw_eth_dev *priv = dev_get_priv(dev);
	phys_addr_t iobase;
	u32 mii_data = val | MII_BUSY;
	u32 mii_addr = 0;
	ulong start;

	if (devad == MDIO_DEVAD_NONE)
		return -EOPNOTSUPP;

	if (priv->eth_link == 1)
		iobase = priv->mac_base;
	else
		iobase = link2_mdio_mac_base;

	if (iobase == 0)
		return -EFAULT;

	xgbe_mdio_mux_config(priv);

	mii_addr |= ((devad << MII_DA_SHIFT) & MII_DA_MASK);
	mii_addr |= ((addr << MII_ADDR_SHIFT) & MII_ADDR_MASK);
	mii_addr |= ((reg << MII_REG_SHIFT) & MII_REG_MASK);
	mii_data |= ((MII_CLK_400_450M << MII_CLK_SHIFT) & MII_CLK_MASK);
	mii_data |= ((MII_CMD_WRITE << MII_CMD_SHIFT) & MII_CMD_MASK);

	writel(mii_addr, iobase + MAC_MDIO_ADDR);
	writel(mii_data, iobase + MAC_MDIO_DATA);

	start = get_timer(0);
	while (get_timer(start) < MDIO_TIMEOUT) {
		if (!(readl(iobase + MAC_MDIO_DATA) & MII_BUSY))
			return 0;
		udelay(10);
	};

	return -ETIMEDOUT;
}

#if CONFIG_IS_ENABLED(DM_GPIO)
static int dw_mdio_reset(struct mii_dev *bus)
{
	struct udevice *dev = bus->priv;
	struct dw_eth_dev *priv = dev_get_priv(dev);
	struct dw_eth_pdata *pdata = dev_get_platdata(dev);
	int ret;

	if (!dm_gpio_is_valid(&priv->reset_gpio))
		return 0;

	/* reset the phy */
	ret = dm_gpio_set_value(&priv->reset_gpio, 0);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[0]);

	ret = dm_gpio_set_value(&priv->reset_gpio, 1);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[1]);

	ret = dm_gpio_set_value(&priv->reset_gpio, 0);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[2]);

	return 0;
}
#endif

static int dw_mdio_init(const char *name, void *priv)
{
	struct mii_dev *bus = mdio_alloc();

	if (!bus) {
		printf("Failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->read = dw_mdio_read;
	bus->write = dw_mdio_write;
	snprintf(bus->name, sizeof(bus->name), "%s", name);
#if CONFIG_IS_ENABLED(DM_GPIO)
	bus->reset = dw_mdio_reset;
#endif

	bus->priv = priv;

	return mdio_register(bus);
}
#endif

static void tx_desc_init(struct dw_eth_dev *priv)
{
	struct dma_desc *desc_table_p = &priv->tx_desc[0];
	u8 *tx_buff = &priv->tx_buff[0];
	struct dma_desc *desc_p;
	phys_addr_t iobase = priv->mac_base;
	u64 addr;
	u32 idx;

	for (idx = 0; idx < TX_DESC_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		addr = (u64)&tx_buff[idx * ETH_BUF_SIZE];
		desc_p->des0 = (u32)addr;
		desc_p->des1 = (u32)(addr >> 32);
		desc_p->des2 = 0;
		desc_p->des3 = 0;
	}

	/* Flush all Tx buffer descriptors at once */
	flush_dcache_range((u64)priv->tx_desc,
			   (u64)priv->tx_desc + sizeof(priv->tx_desc));

	addr = (u64)&desc_table_p[0];
	writel((u32)(addr >> 32), iobase + DMA_CH_TxDESC_HADDR(0));
	writel((u32)addr, iobase + DMA_CH_TxDESC_LADDR(0));
	/* The actual ring size is setting value plus 1 */
	writel(TX_DESC_NUM-1, iobase + DMA_CH_TxDESC_RING_LEN(0));
	/* Update tail pointer. No packet to transmit */
	writel((u32)addr, iobase + DMA_CH_TxDESC_TAIL_LPTR(0));

	priv->tx_currdescnum = 0;
}

static void rx_desc_init(struct dw_eth_dev *priv)
{
	struct dma_desc *desc_table_p = &priv->rx_desc[0];
	u8 *rx_buff = &priv->rx_buff[0];
	struct dma_desc *desc_p;
	phys_addr_t iobase = priv->mac_base;
	u64 addr;
	u32 idx;

	/* Before passing buffers to XGMAC we need to make sure zeros
	 * written there right after "priv" structure allocation were
	 * flushed into RAM.
	 * Otherwise there's a chance to get some of them flushed in RAM when
	 * XGMAC is already pushing data to RAM via DMA. This way incoming from
	 * XGMAC data will be corrupted. */
	flush_dcache_range((u64)rx_buff, (u64)(rx_buff + RX_TOTAL_BUF_SIZE));

	for (idx = 0; idx < RX_DESC_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		addr = (u64)&rx_buff[idx * ETH_BUF_SIZE];
		desc_p->des0 = (u32)addr;
		desc_p->des1 = (u32)(addr >> 32);
		desc_p->des2 = 0;
		desc_p->des3 = RDES3_OWN;
	}

	/* Flush all Rx buffer descriptors at once */
	flush_dcache_range((u64)priv->rx_desc,
			   (u64)priv->rx_desc + sizeof(priv->rx_desc));

	addr = (u64)&desc_table_p[0];
	writel((u32)(addr >> 32), iobase + DMA_CH_RxDESC_HADDR(0));
	writel((u32)addr, iobase + DMA_CH_RxDESC_LADDR(0));
	/* The actual ring size is setting value plus 1 */
	writel(RX_DESC_NUM-1, iobase + DMA_CH_RxDESC_RING_LEN(0));
	/* Update tail poiter. All descriptors are ready to use */
	addr = (u64)&desc_table_p[RX_DESC_NUM-1];
	writel((u32)addr, iobase + DMA_CH_RxDESC_TAIL_LPTR(0));

	priv->rx_currdescnum = 0;
}

static int dw_write_hwaddr(struct dw_eth_dev *priv, u8 *mac_id)
{
	phys_addr_t iobase = priv->mac_base;
	u32 macid_lo;
	u32 macid_hi;

	macid_lo = mac_id[0] + (mac_id[1] << 8) + (mac_id[2] << 16) +
		   (mac_id[3] << 24);
	macid_hi = mac_id[4] + (mac_id[5] << 8);

	writel(macid_hi, iobase + MAC_ADDR0_HIGH);
	writel(macid_lo, iobase + MAC_ADDR0_LOW);

	return 0;
}

#ifdef CONFIG_DM_ETH_PHY
static int dw_adjust_link(struct dw_eth_dev *priv, struct phy_device *phydev)
{
	/* Set PHY link related settings when link is up */
	/* Update link speed setting when link is up and no rate matching */
	if (priv->phy_rate_matching || !phydev->link ||
	    priv->eth_link == 1 || priv->phy_lane != 0) {
		/* Do nothing */
	}
	else if (phydev->speed != priv->curr_speed) {
		if (priv->interface != PHY_INTERFACE_MODE_USXGMII) {
			xgbe_misc_update_speed(priv, phydev->speed);
		}
		priv->xpcs->update_speed(priv, phydev->speed);
		set_mac_speed(priv, phydev->speed);
		priv->curr_speed = phydev->speed;
	}

	if (loopback_test != NO_LOOPBACK) {
		/* In loopback mode, fake link up se we can do loopback */
		if (phydev->link == 0) {
			printf("Fake link status for loopback test\n");
			phydev->link = 1;
			phydev->speed = priv->max_speed;
			phydev->duplex = 1;
		}
	}

	printf("Link: %s, Speed: %d, %s duplex%s\n",
	       phydev->link ? "up" : "down",
	       phydev->speed, (phydev->duplex) ? "full" : "half",
	       (phydev->port == PORT_FIBRE) ? ", fiber mode" : "");

	return 0;
}
#endif

static void dw_eth_halt(struct dw_eth_dev *priv)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;

	/* Disable Rx MAC */
	reg = readl(iobase + MAC_RX_CONFIG);
	reg &= ~MAC_CONFIG_RE;
	reg &= ~MAC_CONFIG_LM;
	writel(reg, iobase + MAC_RX_CONFIG);
	/* Disable Rx DMA */
	reg = readl(iobase + DMA_CH_RX_CONTROL(0));
	reg &= ~DMA_RXST;
	writel(reg, iobase + DMA_CH_RX_CONTROL(0));

	/* Disable Tx DMA */
	reg = readl(iobase + DMA_CH_TX_CONTROL(0));
	reg &= ~DMA_TXST;
	writel(reg, iobase + DMA_CH_TX_CONTROL(0));
	/* Disable Tx MAC */
	reg = readl(iobase + MAC_TX_CONFIG);
	reg &= ~MAC_CONFIG_TE;
	writel(reg, iobase + MAC_TX_CONFIG);

#ifdef CONFIG_DM_ETH_PHY
	phy_shutdown(priv->phydev);
#endif
}

static int mac_reset(struct dw_eth_dev *priv)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;
	u32 start;

	reg = readl(iobase + DMA_MODE);
	reg |= DMA_SWR;
	writel(reg, iobase + DMA_MODE);

	start = get_timer(0);
	while (get_timer(start) < MAC_RESET_TIMEOUT) {
		reg = readl(iobase + DMA_MODE);
		if (!(reg & DMA_SWR))
			return 0;
		udelay(10);
	};

	printf("Lane %d soft reset timeout\n", priv->phy_lane);
	return -1;
}

void mac_init(struct dw_eth_dev *priv, u8 *enetaddr)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;

	if (priv->curr_speed == SPEED_50000)
		reg = XLGMAC_TX_SS_50G;
	else if (priv->curr_speed == SPEED_25000)
		reg = XLGMAC_TX_SS_25G;
	else if (priv->curr_speed == SPEED_5000)
		reg = XGMAC_TX_SS_5G_XGMII;
	else if (priv->curr_speed == SPEED_1000)
		reg = XGMAC_TX_SS_1G_GMII;
	else if (priv->curr_speed == SPEED_2500) {
		if (priv->interface == PHY_INTERFACE_MODE_GMII ||
		    priv->interface == PHY_INTERFACE_MODE_SGMII)
			reg = XGMAC_TX_SS_2G5_GMII;
		else
			reg = XGMAC_TX_SS_2G5_XGMII;
	}
	else {
		if (priv->is_xlgmac)
			reg = XLGMAC_TX_SS_10G;
		else
			reg = XGMAC_TX_SS_10G;
	}
	reg |= MAC_CONFIG_JD;
	writel(reg, iobase + MAC_TX_CONFIG);

	reg = MAC_CONFIG_IPC | MAC_CONFIG_JE;
	if (loopback_test ==  MAC_LOOPBACK) {
		reg |= MAC_CONFIG_LM;
		printf("XGMAC/XLGMAC loopback enabled\n");
	}
	writel(reg, iobase + MAC_RX_CONFIG);

	reg = MAC_FILTER_RA;
	writel(reg, iobase + MAC_PACKET_FILTER);

	reg = MAC_RXQEN << MAC_RXQEN_SHIFT(0);
	if (priv->is_xlgmac)
		writel(reg, iobase + XLGMAC_RXQ_CTRL0);
	else
		writel(reg, iobase + XGMAC_RXQ_CTRL0);

	/* MAC soft reset clears HW address. Set again. */
	dw_write_hwaddr(priv, enetaddr);

	/* Map queue 0 to DMA channel 0 */
	reg = 0 << MTL_QxMDMACH_SHIFT(0);
	writel(reg, iobase + MTL_RXQ_DMA_MAP0);

	reg = ((TX_FIFO_SIZE / 256) -1) << MTL_TQS_SHIFT;
	reg |= (MTL_TXQEN << MTL_TXQEN_SHIFT) | MTL_TSF | MTL_FTQ;
	writel(reg, iobase + MTL_TXQ_OPMODE(0));

	reg = ((RX_FIFO_SIZE / 256) -1) << MTL_RQS_SHIFT;
	reg |= MTL_RSF;
	/* Disable dropping packet on checksum error */
	reg |= MTL_DIS_TCP_EF;
	/* Forward error packets for 50G or 25G */
	if (priv->eth_link == 1)
		reg |= MTL_FEF;
	writel(reg, iobase + MTL_RXQ_OPMODE(0));
}

static void set_mac_speed(struct dw_eth_dev *priv, int speed)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;

	if (speed == SPEED_50000)
		reg = XLGMAC_TX_SS_50G;
	else if (speed == SPEED_25000)
		reg = XLGMAC_TX_SS_25G;
	else if (speed == SPEED_5000)
		reg = XGMAC_TX_SS_5G_XGMII;
	else if (speed == SPEED_1000)
		reg = XGMAC_TX_SS_1G_GMII;
	else if (speed == SPEED_2500) {
		if (priv->interface == PHY_INTERFACE_MODE_GMII ||
		    priv->interface == PHY_INTERFACE_MODE_SGMII)
			reg = XGMAC_TX_SS_2G5_GMII;
		else
			reg = XGMAC_TX_SS_2G5_XGMII;
	}
	else {
		if (priv->is_xlgmac)
			reg = XLGMAC_TX_SS_10G;
		else
			reg = XGMAC_TX_SS_10G;
	}
	writel(reg, iobase + MAC_TX_CONFIG);
}

void dma_init(struct dw_eth_dev *priv)
{
	phys_addr_t iobase = priv->mac_base;
	u32 dsl_value;
	u32 reg;

	reg = DMA_UNDEF | DMA_AAL | DMA_EAME;
	reg |= (DMA_OSR_LMT_32 << DMA_RD_OSR_LMT_SHIFT);
	reg |= (DMA_OSR_LMT_32 << DMA_WR_OSR_LMT_SHIFT);
	writel(reg, iobase + DMA_SYSBUS_MODE);

	reg = DEFAULT_DMA_PBL << DMA_TxPBL_SHIFT;
	writel(reg, iobase + DMA_CH_TX_CONTROL(0));

	reg = DEFAULT_DMA_PBL << DMA_RxPBL_SHIFT;
	reg |= ETH_BUF_SIZE << DMA_RBSZ_SHIFT;
	writel(reg, iobase + DMA_CH_RX_CONTROL(0));

	/* Set Descriptor Skip Length */
	if (ARCH_DMA_MINALIGN >= ACTUAL__DESC_SIZE) {
		dsl_value = (ARCH_DMA_MINALIGN - ACTUAL__DESC_SIZE) / 16;
	}
	else
		dsl_value = 0;
	reg = dsl_value << DMA_DSL_SHIFT;
	writel(reg, iobase + DMA_CH_CONTROL(0));

	rx_desc_init(priv);
	tx_desc_init(priv);
}

int dwxgmac_eth_init(struct dw_eth_dev *priv, u8 *enetaddr)
{
#if defined CONFIG_EQXPCS || defined CONFIG_DM_ETH_PHY
	int ret;
#endif

	mac_init(priv, enetaddr);
	dma_init(priv);

#ifdef CONFIG_DM_ETH_PHY
	/* Start up the PHY */
	ret = phy_startup(priv->phydev);
	if (ret) {
		printf("Could not initialize PHY %s\n",
		       priv->phydev->dev->name);
		return ret;
	}

	ret = dw_adjust_link(priv, priv->phydev);
	if (ret)
		return ret;
#endif

#ifdef CONFIG_EQXPCS
	ret = priv->xpcs->config(priv);
	if (ret)
		return ret;
#endif

	return 0;
}

int dwxgmac_eth_enable(struct dw_eth_dev *priv)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;

#ifdef CONFIG_EQXPCS
	int linkup;
	int ret;
	ret = priv->xpcs->get_state(priv, &linkup);
	if (ret)
		return ret;
	else if (!linkup) {
		printf("XPCS link down\n");
		return -ENOLINK;
        }
#endif

#ifdef CONFIG_DM_ETH_PHY
	if (!priv->phydev->link)
		return -EIO;
#endif

	/* Enable Tx MAC */
	reg = readl(iobase + MAC_TX_CONFIG);
	reg |= MAC_CONFIG_TE;
	writel(reg, iobase + MAC_TX_CONFIG);
	/* Enable Tx DMA */
	reg = readl(iobase + DMA_CH_TX_CONTROL(0));
	reg |= DMA_TXST;
	writel(reg, iobase + DMA_CH_TX_CONTROL(0));

	/* Enable Rx DMA */
	reg = readl(iobase + DMA_CH_RX_CONTROL(0));
	reg |= DMA_RXST;
	writel(reg, iobase + DMA_CH_RX_CONTROL(0));
	/* Enable Rx MAC */
	reg = readl(iobase + MAC_RX_CONFIG);
	reg |= MAC_CONFIG_RE;
	writel(reg, iobase + MAC_RX_CONFIG);

	return 0;
}

static int dw_eth_send(struct dw_eth_dev *priv, void *packet, int length)
{
	u32 desc_num = priv->tx_currdescnum;
	struct dma_desc *desc_p = &priv->tx_desc[desc_num];
	u64 desc_start;
	u64 desc_end;
	u64 data_start;
	u64 data_end;

	desc_start = (u64)desc_p;
	desc_end = desc_start + roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	data_start = (u64)&priv->tx_buff[desc_num * ETH_BUF_SIZE];
	data_end = data_start + roundup(length, ARCH_DMA_MINALIGN);

	/*
	 * Strictly we only need to invalidate the "txrx_status" field
	 * for the following check, but on some platforms we cannot
	 * invalidate only 4 bytes, so we flush the entire descriptor,
	 * which is 16 bytes in total. This is safe because the
	 * individual descriptors in the array are each aligned to
	 * ARCH_DMA_MINALIGN and padded appropriately.
	 */
	invalidate_dcache_range(desc_start, desc_end);

	/* Check if the descriptor is owned by CPU */
	if (desc_p->des3 & TDES3_OWN) {
		printf("CPU not owner of tx frame\n");
		return -EAGAIN;
	}

	if (length > ETH_BUF_SIZE) {
		printf("Packet size is too big %d\n", length);
		return -EPERM;
	}
	/* If length < 60, XGMAC inserts pad automatically */
	memcpy((void *)data_start, packet, length);

	/* Flush data to be sent */
	flush_dcache_range(data_start, data_end);

	/* XGMAC does not modify des0 and des1 */
	desc_p->des2 = length & TDES2_B1L;
	desc_p->des3 = TDES3_OWN | TDES3_FD | TDES3_LD | (length & TDES3_FL);

	/* Flush modified buffer descriptor */
	flush_dcache_range(desc_start, desc_end);

	/* Test the wrap-around condition. */
	if (++desc_num >= TX_DESC_NUM)
		desc_num = 0;

	priv->tx_currdescnum = desc_num;

	/* Start the transmission */
	desc_p = &priv->tx_desc[desc_num];
	desc_start = (u64)desc_p;
	writel((u32)desc_start, priv->mac_base + DMA_CH_TxDESC_TAIL_LPTR(0));

	return 0;
}

static int dw_eth_recv(struct dw_eth_dev *priv, uchar **packetp)
{
	u32 desc_num = priv->rx_currdescnum;
	struct dma_desc *desc_p = &priv->rx_desc[desc_num];
	u32 status;
	int length = -EAGAIN;
	u64 desc_start;
	u64 desc_end;
	u64 data_start;
	u64 data_end;

	desc_start = (ulong)desc_p;
	desc_end = desc_start + roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);

	/* Invalidate entire buffer descriptor */
	invalidate_dcache_range(desc_start, desc_end);

	status = desc_p->des3;

	/* Check  if the owner is the CPU */
	if (!(status & RDES3_OWN)) {
		length = (status & RDES3_PL);
		data_start = (u64)&priv->rx_buff[desc_num * ETH_BUF_SIZE];
		data_end = data_start + roundup(length, ARCH_DMA_MINALIGN);

		/* Invalidate received data */
		invalidate_dcache_range(data_start, data_end);
		*packetp = (uchar *)data_start;
		if (loopback_test != NO_LOOPBACK)
			printf("Received loopback pkt. Length %d\n", length);

		/* Check packet error */
		if ((status & RDES3_LD) && (status & RDES3_ES)) {
			char l34_type[10];
			char err_type[10];

			switch ((status & RDES3_L34T) >> RDES3_L34T_SHIFT) {
			case RDES3_L34T_NONIP:
				snprintf(l34_type, 10, "Non-IP");
				break;
			case RDES3_L34T_IP4TCP:
				snprintf(l34_type, 10, "IP4 TCP");
				break;
			case RDES3_L34T_IP4UDP:
				snprintf(l34_type, 10, "IP4 UDP");
				break;
			case RDES3_L34T_IP4ICMP:
				snprintf(l34_type, 10, "IP4 ICMP");
				break;
			case RDES3_L34T_IP6TCP:
				snprintf(l34_type, 10, "IP6 TCP");
				break;
			case RDES3_L34T_IP6UDP:
				snprintf(l34_type, 10, "IP6 UDP");
				break;
			case RDES3_L34T_IP6ICMP:
				snprintf(l34_type, 10, "IP6 ICMP");
				break;
			default:
				snprintf(l34_type, 10, "Unknown");
				break;
			}

			switch ((status & RDES3_ET) >> RDES3_ET_SHIFT) {
			case RDES3_ET_CRC:
				snprintf(err_type, 10, "CRC");
				break;
			case RDES3_ET_IP_HDR:
				snprintf(err_type, 10, "IP hdr");
				break;
			case RDES3_ET_CHKSUM:
				snprintf(err_type, 10, "chksum");
				break;
			default:
				snprintf(err_type, 10, "0x%lx",
					 (status & RDES3_ET) >> RDES3_ET_SHIFT);
				break;
			}
			printf("Recv %s packet %s error\n", l34_type, err_type);
		}
	}

	return length;
}

static int dw_free_pkt(struct dw_eth_dev *priv)
{
	u32 desc_num = priv->rx_currdescnum;
	struct dma_desc *desc_p = &priv->rx_desc[desc_num];
	u64 addr;
	u64 desc_start;
	u64 desc_end;

	desc_start = (u64)desc_p;
	desc_end = desc_start + roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	/*
	 * Make the current descriptor valid again and go to
	 * the next one
	 */
	addr = (u64)&priv->rx_buff[desc_num * ETH_BUF_SIZE];
	desc_p->des0 = (u32)addr;
	desc_p->des1 = (u32)(addr >> 32);
	desc_p->des2 = 0;
	desc_p->des3 = RDES3_OWN;

	/* Flush only status field - others weren't changed */
	flush_dcache_range(desc_start, desc_end);

	/* Move tail pointer */
	writel((u32)desc_start, priv->mac_base + DMA_CH_RxDESC_TAIL_LPTR(0));

	/* Test the wrap-around condition. */
	if (++desc_num >= RX_DESC_NUM)
		desc_num = 0;
	priv->rx_currdescnum = desc_num;

	return 0;
}

#ifdef CONFIG_DM_ETH_PHY
static void xgbe_xgmac_set_cl45(struct dw_eth_dev *priv)
{
	/* Set port for MDIO Clause 45 */
	writel(0, priv->mac_base + MAC_MDIO_C22P);
}

static int dw_phy_init(struct dw_eth_dev *priv, void *dev)
{
	struct phy_device *phydev;
	int ret;

	xgbe_xgmac_set_cl45(priv);

	ret = xgbe_phy_init(priv);
	if (ret)
		return -ENODEV;

	phydev = phy_connect(priv->bus, priv->phy_addr, dev, priv->interface);
	if (!phydev)
		return -ENODEV;

	/* u-boot phy framework supports upto 10G */
	if (priv->max_speed <= SPEED_10000 && priv->max_speed != SPEED_5000) {
		ret = phy_set_supported(phydev, priv->max_speed);
		if (ret)
			return ret;
	}
	phydev->supported &= xgbe_phy_supported_features(priv);

	phydev->advertising = phydev->supported;
	phydev->priv = priv;

	priv->phydev = phydev;
	phy_config(phydev);
	xgbe_phy_post_config(priv);

	return 0;
}
#endif

static int dwxgmac_eth_start(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	int ret;

	ret = dwxgmac_eth_init(priv, pdata->enetaddr);
	if (ret)
		return ret;

	ret = dwxgmac_eth_enable(priv);
	return ret;
}

int dwxgmac_eth_send(struct udevice *dev, void *packet, int length)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);

	return dw_eth_send(priv, packet, length);
}

int dwxgmac_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);

	return dw_eth_recv(priv, packetp);
}

int dwxgmac_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);

	return dw_free_pkt(priv);
}

void dwxgmac_eth_stop(struct udevice *dev)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);

	return dw_eth_halt(priv);
}

int dwxgmac_eth_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);

	return dw_write_hwaddr(priv, pdata->enetaddr);
}

static int dwxgmac_eth_bind(struct udevice *dev)
{
	return 0;
}

static void mac_fixed_pps_config(struct dw_eth_dev *priv)
{
	phys_addr_t iobase = priv->mac_base;
	u32 reg;

	/* PTP clock 750 MHz */
	writel(0x15500, iobase + MAC_SUB_SEC_INC);

	reg = readl(iobase + MAC_TIMESTAMP_CTRL);
	reg |= (MAC_TSCTRLSSR | MAC_TSINIT | MAC_TSENA);
	writel(reg, iobase + MAC_TIMESTAMP_CTRL);

	/* PPS 1 hz */
	writel(0x1, iobase + MAC_PPS_CONTROL);
}

int dwxgmac_eth_probe(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	phys_addr_t misc_addr = priv->misc_base;
	u32 reg;
	u8 vid;
	int err;
	int ret;

	debug("%s, iobase=%llx, priv=%p\n", __func__, pdata->iobase, priv);
	priv->interface = pdata->phy_interface;

	/* Check if it is XGMAC or XLGMAC */
	reg = readl(priv->mac_base + MAC_VERSION);
	vid = (u8)((reg & MAC_VID_MASK) >> MAC_VID_SHIFT);
	if (vid == MAC_VID_XLGMAC)
		priv->is_xlgmac = 1;
	else if (vid == MAC_VID_XGMAC)
		priv->is_xlgmac = 0;
	else
		return -ENODEV;

#ifdef CONFIG_RAPTOR_FAMILY
	if (priv->phy_lane == 0) {
		if (priv->eth_link == 1) {
			/* NOC reset removal */
			rmwritel(IOSS_PLL_CFG_BASE + 0x40, 0x3FF00, 0x10);
			/* Ethernet reset removal */
			rmwritel(IOSS_PLL_CFG_BASE + 0x44, 0x1, 0x1);
			udelay(1000);

			/* Enable clocks */
			writel(0x01010101, IOSS_PLL_CFG_BASE + 0x48);
			writel(0x00000000, IOSS_PLL_CFG_BASE + 0x4C);
			udelay(1000);

			/* memory shutdown removal ioss top memories
			 * clearing ioss_mst_i_I_mainshdn
			 * register [bit 0] & iommu_shdn[bit 48-55]
			 * would release the IOSS subsystem
			 * from shutdown mode,allowing it to resume
			 * normal operation */
			writel(0x0, IOSS_PLL_CFG_BASE + 0x90);
			writel(0x0, IOSS_PLL_CFG_BASE + 0x94);

			/* PLL0 bringup for 156.25 MHz.
			 * Using ref clock as 122.8 MHz */
			writel(0x200A000F, IOSS_CCIX_CFG_BASE + 0xC8);
			udelay(1000);
			writel(0x00585555, IOSS_CCIX_CFG_BASE + 0xD0);
			writel(0x000A000E, IOSS_CCIX_CFG_BASE + 0xC8);
			udelay(1000);

			/* e56 rate select. Same for all speed on link1 */
			reg = readl(misc_addr + XGBE_E56_CFG0);
			reg &= ~XGBE_E56_AN0_RATE_SELECT;
			reg |= XGBE_E56_AN0_RATE_50G;
			writel(reg, misc_addr + XGBE_E56_CFG0);

			/* e56 apb clk is from ioss pll0 */
			reg = readl(misc_addr + XGBE_E56_CFG1);
			reg &= ~XGBE_E56_CLK_SEL; // 0: from ioss pll
			writel(reg, misc_addr + XGBE_E56_CFG1);
		} else {
			/* link2_ref_clk selection */
			writel(0x000A000F, IOSS_CCIX_CFG_BASE + 0xC8);
			udelay(1000);
			writel(0x00585555, IOSS_CCIX_CFG_BASE + 0xD0);
			writel(0x000A000E, IOSS_CCIX_CFG_BASE + 0xC8);
			udelay(1000);

			/* memory shutdown removal ioss top memories */
			writel(0x0, IOSS_PLL_CFG_BASE + 0x90);
			writel(0x0, IOSS_PLL_CFG_BASE + 0x94);
			/* memory shutdown removal link2 fw sram */
			writel(0x0, PTN_E32_REG_BASE + 0x8020);
			writel(0x0, PTN_E32_REG_BASE + 0x8024);
		}

		/* Set lvds to output mode */
		rmwritel(IOSS_PLL_CFG_BASE + 0x50, 0xFFFFF, 0x000FE001);
		udelay(1000);

		/* Set pll mux to enable both clocks */
		/* Also routing pll0 to lvds */
		rmwritel(IOSS_CCIX_CFG_BASE + 0xB0, 0xF0000FF, 0x01000011);
	}

	xgbe_misc_init(priv);

	/* Bring mac memory out of shutdown */
	ret = xgbe_set_mac_mem_active(priv);
	if (ret)
		return -ENODEV;
#endif

#if CONFIG_IS_ENABLED(DM_MDIO)
	ret = dw_mdio_init(dev->name, dev);
	if (ret) {
		err = ret;
		goto mdio_err;
	}
#endif

#ifdef CONFIG_EQXPCS
	xgbe_pcs_mux_config(priv);

	priv->xpcs = xpcs_get_ops(priv);
	if (!priv->xpcs)
		return -ENODEV;

	if (priv->eth_link == 2 && priv->phy_lane != 0)
		priv->xpcs->update_speed(priv, priv->max_speed);
	priv->curr_speed = priv->max_speed;

	ret = priv->xpcs->probe(priv);
	if (ret)
		goto cfg_err;

	if (priv->eth_link == 1) {
		ret = xgbe_e56_serdes_probe(priv);
		if (ret)
			goto cfg_err;
	}
#endif

	priv->bus = miiphy_get_dev_by_name(dev->name);

#ifdef CONFIG_DM_ETH_PHY
	ret = dw_phy_init(priv, dev);
	if (ret)
		goto cfg_err;
#endif

#ifdef CONFIG_EQXPCS
	/* Do e56 Rx calibration after PHY init */
	if (priv->eth_link == 1) {
		udelay(100000);
		ret = xgbe_e56_serdes_rx_config(priv);
		if (ret)
			goto cfg_err;
	}
#endif
	ret = mac_reset(priv);
	if (ret)
		goto cfg_err;

	/* This is temporary for development */
	mac_fixed_pps_config(priv);

#ifdef CONFIG_EQXPCS
#ifdef CONFIG_PHY_MV88X3310
	/* Add delay to clear local fault */
	udelay(500000);
	set_mac_speed(priv, priv->max_speed);
	ret = priv->xpcs->config(priv);
	if (ret)
		return ret;
#endif
#endif
	return 0;

#if defined(CONFIG_DM_ETH_PHY) || defined(CONFIG_EQXPCS)
cfg_err:
#endif
	/* continue here for cleanup if no PHY found */
	err = ret;
#if CONFIG_IS_ENABLED(DM_MDIO)
	mdio_unregister(priv->bus);
	mdio_free(priv->bus);

mdio_err:
#endif
	return err;
}

static int dwxgmac_eth_remove(struct udevice *dev)
{
#ifdef CONFIG_DM_ETH_PHY
	struct dw_eth_dev *priv = dev_get_priv(dev);

	free(priv->phydev);
#endif
#if CONFIG_IS_ENABLED(DM_MDIO)
	mdio_unregister(priv->bus);
	mdio_free(priv->bus);
#endif

	return 0;
}

const struct eth_ops dwxgmac_eth_ops = {
	.start		= dwxgmac_eth_start,
	.send		= dwxgmac_eth_send,
	.recv		= dwxgmac_eth_recv,
	.free_pkt	= dwxgmac_eth_free_pkt,
	.stop		= dwxgmac_eth_stop,
	.write_hwaddr	= dwxgmac_eth_write_hwaddr,
};

int dwxgmac_eth_ofdata_to_platdata(struct udevice *dev)
{
	struct dw_eth_pdata *dw_pdata = dev_get_platdata(dev);
	struct eth_pdata *pdata = &dw_pdata->eth_pdata;
	const char *phy_mode;
	struct dw_eth_dev *priv = dev_get_priv(dev);
#if CONFIG_IS_ENABLED(DM_GPIO)
	int reset_flags = GPIOD_IS_OUT;
#endif
	int ret = 0;

	priv->dev = dev;
	priv->mac_base = dev_read_addr_name(dev, "xgmac");
	if (priv->mac_base == OF_NULL_ADDR) {
		dev_err(dev, "No reg property for xgmac\n");
		return -EINVAL;
	}
#ifdef CONFIG_RAPTOR_FAMILY
	priv->misc_base = dev_read_addr_name(dev, "misc");
#endif
#ifdef CONFIG_EQXPCS
	priv->xpcs_base = dev_read_addr_name(dev, "xpcs");
	if (priv->mac_base == OF_NULL_ADDR) {
		dev_err(dev, "No reg property for xpcs\n");
		return -EINVAL;
	}
	priv->e56_base = dev_read_addr_name(dev, "e56");
#endif
	pdata->iobase = priv->mac_base;
	pdata->phy_interface = -1;
	phy_mode = dev_read_string(dev, "phy-mode");
	if (phy_mode)
		pdata->phy_interface = phy_get_interface_by_name(phy_mode);
	if (pdata->phy_interface == -1) {
		debug("%s: Invalid PHY interface '%s'\n", __func__, phy_mode);
		//return -EINVAL;
	}
	ret = dev_read_s32(dev, "phy-addr", &priv->phy_addr);
	if (ret)
		priv->phy_addr = -1;
	ret = dev_read_s32(dev, "phy-lane", &priv->phy_lane);
	if (ret)
		return -EINVAL;
	ret = dev_read_u32(dev, "max-speed", &priv->max_speed);
	if (ret)
		return -EINVAL;
	ret = dev_read_u32(dev, "eth-link", &priv->eth_link);
	if (ret)
		return -EINVAL;
	if ((priv->eth_link & config_eth_link) != priv->eth_link)
		return -ENODEV;
	if (priv->eth_link == 1 && priv->e56_base == OF_NULL_ADDR) {
		dev_err(dev, "No reg property for e56\n");
		return -EINVAL;
	}
	if (priv->eth_link == 1) {
		if (config_eth_speed[4] > 0) {
			if (config_eth_speed[4] < SPEED_10000)
				return -ENODEV;
			priv->max_speed = config_eth_speed[4];
		}
	}
	else {
		if (config_eth_speed[priv->phy_lane] > 0) {
			if (config_eth_speed[priv->phy_lane] > SPEED_25000)
				return -ENODEV;
			priv->max_speed = config_eth_speed[priv->phy_lane];
		}
	}
	ret = dev_read_u32(dev, "phy-rst-gpio", &priv->phy_reset_gpio);
	if (ret) {
		priv->phy_reset_gpio = NO_PHY_RESET_GPIO;
		ret = 0;
	}
	priv->use_i2c = dev_read_bool(dev, "edgeq,use-i2c");
	/* Hawk V1 does not support retimer */
	if (env_board_type == BOARD_HAWK && priv->use_i2c)
		return -ENODEV;
	priv->phy_rate_matching = dev_read_bool(dev, "phy-rate-match");

#if CONFIG_IS_ENABLED(DM_GPIO)
	if (dev_read_bool(dev, "snps,reset-active-low"))
		reset_flags |= GPIOD_ACTIVE_LOW;

	ret = gpio_request_by_name(dev, "snps,reset-gpio", 0,
		&priv->reset_gpio, reset_flags);
	if (ret == 0) {
		ret = dev_read_u32_array(dev, "snps,reset-delays-us",
					 dw_pdata->reset_delays, 3);
	} else if (ret == -ENOENT) {
		ret = 0;
	}
#endif

	return ret;
}

static const struct udevice_id dwxgmac_eth_ids[] = {
	{ .compatible = "edgeq,xgbe" },
	{ }
};

U_BOOT_DRIVER(eth_dwxgmac) = {
	.name	= "eth_dwxgmac",
	.id	= UCLASS_ETH,
	.of_match = dwxgmac_eth_ids,
	.ofdata_to_platdata = dwxgmac_eth_ofdata_to_platdata,
	.bind	= dwxgmac_eth_bind,
	.probe	= dwxgmac_eth_probe,
	.remove	= dwxgmac_eth_remove,
	.ops	= &dwxgmac_eth_ops,
	.priv_auto_alloc_size = sizeof(struct dw_eth_dev),
	.platdata_auto_alloc_size = sizeof(struct dw_eth_pdata),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};
