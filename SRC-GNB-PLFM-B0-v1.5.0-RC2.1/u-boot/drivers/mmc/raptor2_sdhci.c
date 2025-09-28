// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare Multimedia Card Interface driver
 * for EdgeQ's Raptor2 SoC.
 * Copyright 2021 EdgeQ Inc.
 * Author: Nilesh Waghmare (nilesh.waghmare@edgeq.io)
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <mmc.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <sdhci.h>
#include <clk.h>

DECLARE_GLOBAL_DATA_PTR;

#define F_MIN			400000
#define F_MAX			52000000

#define MSHC_VER_ID_R                   0x500
#define MSHC_VER_TYPE_R                 0x504
#define MSHC_EMMC_CTRL_R                0x52c
#define MSHC_EMMC_CTRL_EMMC_RST_N       BIT(2)
#define MSHC_EMMC_CTRL_CARD_IS_EMMC     BIT(0)

struct raptor2_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct raptor2_sdhci_priv {
	struct sdhci_host host;
};

static int raptor2_sdhci_bind(struct udevice *dev)
{
	struct raptor2_sdhci_plat *plat = dev_get_platdata(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static inline void raptor2_sdhci_raw_writel(struct sdhci_host *host, u32 val,
					    int reg)
{
	writel(val, host->ioaddr + (reg << (host->reg_shift - 1)));
}

static inline u32 raptor2_sdhci_raw_readl(struct sdhci_host *host, int reg)
{
	return readl(host->ioaddr + (reg << (host->reg_shift - 1)));
}

static void raptor2_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	raptor2_sdhci_raw_writel(host, val, reg);
}

static void raptor2_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	static u32 shadow;
	u32 oldval = (reg == SDHCI_COMMAND) ? shadow :
		raptor2_sdhci_raw_readl(host, reg & ~3);
	u32 word_num = (reg >> 1) & 1;
	u32 word_shift = word_num * 16;
	u32 mask = 0xffff << word_shift;
	u32 newval = (oldval & ~mask) | (val << word_shift);

	if (reg == SDHCI_TRANSFER_MODE)
		shadow = newval;
	else
		raptor2_sdhci_raw_writel(host, newval, reg & ~3);
}

static void raptor2_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 oldval = raptor2_sdhci_raw_readl(host, reg & ~3);
	u32 byte_num = reg & 3;
	u32 byte_shift = byte_num * 8;
	u32 mask = 0xff << byte_shift;
	u32 newval = (oldval & ~mask) | (val << byte_shift);

	raptor2_sdhci_raw_writel(host, newval, reg & ~3);
}

static u32 raptor2_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val = raptor2_sdhci_raw_readl(host, reg);

	return val;
}

static u16 raptor2_sdhci_readw(struct sdhci_host *host, int reg)
{
	u32 val = raptor2_sdhci_raw_readl(host, (reg & ~3));
	u32 word_num = (reg >> 1) & 1;
	u32 word_shift = word_num * 16;
	u32 word = (val >> word_shift) & 0xffff;

	return word;
}

static u8 raptor2_sdhci_readb(struct sdhci_host *host, int reg)
{
	u32 val = raptor2_sdhci_raw_readl(host, (reg & ~3));
	u32 byte_num = reg & 3;
	u32 byte_shift = byte_num * 8;
	u32 byte = (val >> byte_shift) & 0xff;

	return byte;
}

static const struct sdhci_ops raptor2_ops = {
	.write_l = raptor2_sdhci_writel,
	.write_w = raptor2_sdhci_writew,
	.write_b = raptor2_sdhci_writeb,
	.read_l = raptor2_sdhci_readl,
	.read_w = raptor2_sdhci_readw,
	.read_b = raptor2_sdhci_readb,
};

static int raptor2_sdhci_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct raptor2_sdhci_plat *plat = dev_get_platdata(dev);
	struct raptor2_sdhci_priv *priv = dev_get_priv(dev);
	struct sdhci_host *host = &priv->host;
	struct clk clk;
	int ret, bus_hz;

	host->name = dev->name;
	host->ioaddr = (void *)devfdt_get_addr(dev);
	if (IS_ERR(host->ioaddr))
		return PTR_ERR(host->ioaddr);

	host->reg_shift = 1;
	host->reg_shift = dev_read_u32_default(dev, "reg-shift",
		host->reg_shift);
	if (!host->reg_shift)
		debug("%s: Unable to find reg-shift\n", __func__);

	plat->cfg.host_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;

	/* Clock Setup */
	ret = clk_get_by_name(dev, "ciu", &clk);
	if (ret)
		goto clk_err;

	bus_hz = clk_get_rate(&clk);
	if (bus_hz < F_MIN) {
		ret = -EINVAL;
		goto clk_err_ciu;
	}

	host->quirks = SDHCI_QUIRK_WAIT_SEND_CMD | SDHCI_QUIRK_CLOCK_DIV_ZERO_BROKEN;

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		return ret;

	host->max_clk = bus_hz;
	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	host->ops = &raptor2_ops;

	ret = sdhci_setup_cfg(&plat->cfg, host, plat->cfg.f_max,
		F_MIN);
	if (ret)
		return ret;

	upriv->mmc = &plat->mmc;
	host->mmc->priv = host;

	debug("%s: SDHC version: 0x%08x\n",
		dev->name, sdhci_readw(host, SDHCI_HOST_VERSION));
	debug("%s: SDHC version: 0x%08x\n",
		dev->name, sdhci_readl(host, MSHC_VER_ID_R));
	debug("%s: SDHC type:    0x%08x\n",
		dev->name, sdhci_readl(host, MSHC_VER_TYPE_R));

	return sdhci_probe(dev);

clk_err_ciu:
	clk_free(&clk);
clk_err:
	dev_err(dev, "failed to setup clocks, ret %d\n", ret);

	return ret;
}

static const struct udevice_id raptor2_sdhci_ids[] = {
	{ .compatible = "edgeq,raptor2-sdhci" },
	{ }
};

U_BOOT_DRIVER(raptor2_sdhci) = {
	.name		= "raptor2_sdhci",
	.id		= UCLASS_MMC,
	.of_match	= raptor2_sdhci_ids,
	.bind		= raptor2_sdhci_bind,
	.probe		= raptor2_sdhci_probe,
	.ops		= &sdhci_ops,
	.platdata_auto_alloc_size = sizeof(struct raptor2_sdhci_plat),
	.priv_auto_alloc_size = sizeof(struct raptor2_sdhci_priv),
};
