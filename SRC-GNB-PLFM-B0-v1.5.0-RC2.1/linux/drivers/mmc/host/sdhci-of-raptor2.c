// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 *
 * Copyright (C) 2021 EdgeQ Inc.
 *
 * Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sizes.h>
#include <linux/delay.h>
#include "sdhci-pltfm.h"

#define SDHCI_DWCMSHC_ARG2_STUFF	GENMASK(31, 16)
#define	MSHC_VER_ID_R			0x500
#define	MSHC_VER_TYPE_R			0x504
#define	MSHC_EMMC_CTRL_R		0x52c
#define	MSHC_EMMC_CTRL_EMMC_RST_N	BIT(2)
#define	MSHC_EMMC_CTRL_CARD_IS_EMMC	BIT(0)

/* DWCMSHC specific Mode Select value */
#define DWCMSHC_CTRL_HS400		0x7

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

#define REG_OFFSET_IN_BITS(reg) ((reg) << 3 & 0x18)

struct r2_sdhci_priv {
	struct clk	*bus_clk;
	u32		shadow_cmd;
	u32		shadow_blk;
	bool		is_cmd_shadowed;
	bool		is_blk_shadowed;
};

static inline void raptor2_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	pr_debug("%s %d: writel [0x%02x] 0x%08x\n",
		 mmc_hostname(host->mmc), __LINE__, reg, val);

	writel(val, host->ioaddr + (reg << (host->reg_shift - 1)));

	if (host->clock <= 400000) {
		/* Round up to micro-second four SD clock delay */
		if (host->clock)
			udelay((4 * 1000000 + host->clock - 1) / host->clock);
		else
			udelay(10);
	}
}

static inline u32 raptor2_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val = readl(host->ioaddr + (reg << (host->reg_shift - 1)));

	pr_debug("%s %d: readl [0x%02x] 0x%08x\n",
		 mmc_hostname(host->mmc), __LINE__, reg, val);
	return val;
}

static inline void raptor2_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct r2_sdhci_priv *priv = sdhci_pltfm_priv(pltfm_host);
	u32 word_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xffff << word_shift;
	u32 oldval, newval;

	if (reg == SDHCI_COMMAND) {
		/* Write the block now as we are issuing a command */
		if (priv->is_blk_shadowed) {
			raptor2_sdhci_writel(host, priv->shadow_blk,
				SDHCI_BLOCK_SIZE);
			priv->is_blk_shadowed = false;
		}
		oldval = priv->shadow_cmd;
		priv->is_cmd_shadowed = false;
	} else if ((reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) &&
		   priv->is_blk_shadowed) {
		/* Block size and count are stored in shadow reg */
		oldval = priv->shadow_blk;
	} else {
		/* Read reg, all other registers are not shadowed */
		oldval = raptor2_sdhci_readl(host, (reg & ~3));
	}

	newval = (oldval & ~mask) | (val << word_shift);

	if (reg == SDHCI_TRANSFER_MODE) {
		/* Save the transfer mode until the command is issued */
		priv->shadow_cmd = newval;
		priv->is_cmd_shadowed = true;
	} else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
		/* Save the block info until the command is issued */
		priv->shadow_blk = newval;
		priv->is_blk_shadowed = true;
	} else {
		/* Command or other regular 32-bit write */
		raptor2_sdhci_writel(host, newval, reg & ~3);
	}
}

static inline void raptor2_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 oldval = raptor2_sdhci_readl(host, (reg & ~3));
	u32 byte_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xff << byte_shift;
	u32 newval = (oldval & ~mask) | (val << byte_shift);

	raptor2_sdhci_writel(host, newval, reg & ~3);
}

static inline u16 raptor2_sdhci_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct r2_sdhci_priv *priv = sdhci_pltfm_priv(pltfm_host);
	u32 val;
	u16 word;

	if ((reg == SDHCI_TRANSFER_MODE) && priv->is_cmd_shadowed) {
		/* Get the saved transfer mode */
		val = priv->shadow_cmd;
	} else if ((reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) &&
		   priv->is_blk_shadowed) {
		/* Get the saved block info */
		val = priv->shadow_blk;
	} else {
		val = raptor2_sdhci_readl(host, (reg & ~3));
	}
	word = val >> REG_OFFSET_IN_BITS(reg) & 0xffff;
	return word;
}

static inline u8 raptor2_sdhci_readb(struct sdhci_host *host, int reg)
{
	u32 val = raptor2_sdhci_readl(host, (reg & ~3));
	u8 byte = val >> REG_OFFSET_IN_BITS(reg) & 0xff;
	return byte;
}

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void raptor2_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void raptor2_check_auto_cmd23(struct mmc_host *mmc,
				     struct mmc_request *mrq)
{
	struct sdhci_host *host = mmc_priv(mmc);

	/*
	 * No matter V4 is enabled or not, ARGUMENT2 register is 32-bit
	 * block count register which doesn't support stuff bits of
	 * CMD23 argument on dwcmsch host controller.
	 */
	if (mrq->sbc && (mrq->sbc->arg & SDHCI_DWCMSHC_ARG2_STUFF))
		host->flags &= ~SDHCI_AUTO_CMD23;
	else
		host->flags |= SDHCI_AUTO_CMD23;
}

static void raptor2_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	raptor2_check_auto_cmd23(mmc, mrq);
	sdhci_request(mmc, mrq);
}

static void raptor2_set_uhs_signaling(struct sdhci_host *host,
				      unsigned int timing)
{
	u16 ctrl_2;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if ((timing == MMC_TIMING_MMC_HS200) ||
	    (timing == MMC_TIMING_UHS_SDR104))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (timing == MMC_TIMING_UHS_SDR12)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
	else if ((timing == MMC_TIMING_UHS_SDR25) ||
		 (timing == MMC_TIMING_MMC_HS))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
	else if (timing == MMC_TIMING_UHS_SDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
	else if ((timing == MMC_TIMING_UHS_DDR50) ||
		 (timing == MMC_TIMING_MMC_DDR52))
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
	else if (timing == MMC_TIMING_MMC_HS400)
		ctrl_2 |= DWCMSHC_CTRL_HS400;
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
}

static void sdhci_raptor2_set_emmc(struct sdhci_host *host)
{
	if (!mmc_card_is_removable(host->mmc)) {
		u8 value;

		value = sdhci_readb(host, MSHC_EMMC_CTRL_R);
		if (!(value & MSHC_EMMC_CTRL_CARD_IS_EMMC)) {
			value |= MSHC_EMMC_CTRL_CARD_IS_EMMC;
			pr_debug("%s: Set EMMC_CTRL_R: 0x%08x\n",
					mmc_hostname(host->mmc), value);
			sdhci_writeb(host, value, MSHC_EMMC_CTRL_R);
		}
	}
}

static void sdhci_raptor2_reset(struct sdhci_host *host, u8 mask)
{
	sdhci_reset(host, mask);
	pr_debug("%s %d Reset done with mask %x\n", __func__, __LINE__, mask);
	/* Be sure CARD_IS_EMMC stays set */
	sdhci_raptor2_set_emmc(host);
}

static const struct sdhci_ops sdhci_raptor2_ops = {
	.write_l		= raptor2_sdhci_writel,
	.write_w		= raptor2_sdhci_writew,
	.write_b		= raptor2_sdhci_writeb,
	.read_l			= raptor2_sdhci_readl,
	.read_w			= raptor2_sdhci_readw,
	.read_b			= raptor2_sdhci_readb,
	.set_clock		= sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= raptor2_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.reset			= sdhci_raptor2_reset,
	.adma_write_desc	= raptor2_adma_write_desc,
};

static const struct sdhci_pltfm_data sdhci_raptor2_pdata = {
	.ops = &sdhci_raptor2_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
};

static int raptor2_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct r2_sdhci_priv *priv;
	struct device_node *memnp;
	struct resource mem_res;
	struct device *dev;
	int rc;
	int err;
	u32 extra;
	u8 sd_card = 0;
	u8 raptor2_a0 = 0;
	u8 raptor2_b0 = 0;

	host = sdhci_pltfm_init(pdev, &sdhci_raptor2_pdata,
				sizeof(struct r2_sdhci_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	host->reg_shift = 1;

	if (of_property_read_u32(pdev->dev.of_node, "reg-shift", &host->reg_shift) != 0)
		dev_dbg(&pdev->dev, "Unable to find reg-shift\n");

	dev = host->mmc->parent;

	if (device_property_read_bool(dev, "sd-card"))
		sd_card = 1;

	if (device_property_read_bool(dev, "raptor2_a0"))
		raptor2_a0 = 1;

	if (device_property_read_bool(dev, "raptor2_b0"))
		raptor2_b0 = 1;

	if (device_property_read_bool(dev, "mmc_noncoherent")) {
		/* Get reserved memory region from Device-tree */
		memnp = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
		if (!memnp) {
			dev_err(&pdev->dev, "No %s specified\n", "memory-region");
			goto free_pltfm;
		}

		rc = of_address_to_resource(memnp, 0, &mem_res);
		if (rc) {
			dev_err(&pdev->dev, "No memory address assigned to the region\n");
			goto free_pltfm;
		}

		host->paddr = mem_res.start;
		host->vaddr = ioremap_cache(host->paddr, resource_size(&mem_res));
		dev_info(&pdev->dev, "Allocated reserved memory, vaddr: 0x%0llX, paddr: 0x%0llX\n",
				(u64)host->vaddr, host->paddr);

		host->quirks |= SDHCI_QUIRK_BROKEN_ADMA,
		host->quirks2 |= SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN |
			SDHCI_QUIRK2_RAPTOR2_NON_COHERENT;
		if (sd_card)
			host->quirks2 |= SDHCI_QUIRK2_PRESET_VALUE_BROKEN;

		if (raptor2_a0)
			host->quirks2 |= SDHCI_QUIRK2_RAPTOR2_A0;
		else if (raptor2_b0)
			host->quirks2 |= SDHCI_QUIRK2_RAPTOR2_B0;
	} else if (device_property_read_bool(dev, "mmc_coherent")) {
		host->quirks2 |= SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN | SDHCI_QUIRK2_RAPTOR2_COHERENT;
		if (sd_card)
			host->quirks2 |= SDHCI_QUIRK2_PRESET_VALUE_BROKEN;

		if (raptor2_b0)
			host->quirks2 |= SDHCI_QUIRK2_RAPTOR2_B0;
	}
	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
		clk_prepare_enable(priv->bus_clk);

	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	sdhci_get_of_property(pdev);

	host->mmc_host_ops.request = raptor2_request;

	if (!mmc_card_is_removable(host->mmc)) {
		/* Update EMMC_CTRL */
		sdhci_raptor2_set_emmc(host);
		/* If eMMC, disable SD and SDIO */
		if (sd_card)
			host->mmc->caps2 |= (MMC_CAP2_NO_SDIO | MMC_CAP2_NO_MMC);
		else
			host->mmc->caps2 |= (MMC_CAP2_NO_SDIO | MMC_CAP2_NO_SD);
	}

	pr_info("Version:   0x%08x | Present:  0x%08x\n",
	    sdhci_readl(host, SDHCI_SLOT_INT_STATUS) >> 0x10,
	    sdhci_readl(host, SDHCI_PRESENT_STATE));
	pr_info("Caps:      0x%08x | Caps_1:   0x%08x\n",
	    sdhci_readl(host, SDHCI_CAPABILITIES),
	    sdhci_readl(host, SDHCI_CAPABILITIES_1));

	sdhci_enable_v4_mode(host);

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	pr_debug("%s: SDHC version: 0x%08x\n",
			mmc_hostname(host->mmc), sdhci_readl(host, MSHC_VER_ID_R));
	pr_debug("%s: SDHC type:    0x%08x\n",
			mmc_hostname(host->mmc), sdhci_readl(host, MSHC_VER_TYPE_R));

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int raptor2_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct r2_sdhci_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int raptor2_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct r2_sdhci_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable_unprepare(pltfm_host->clk);
	if (!IS_ERR(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);

	return ret;
}

static int raptor2_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct r2_sdhci_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		return ret;

	if (!IS_ERR(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret)
			return ret;
	}

	return sdhci_resume_host(host);
}
#endif

static SIMPLE_DEV_PM_OPS(raptor2_pmops, raptor2_suspend, raptor2_resume);

static const struct of_device_id sdhci_raptor2_dt_ids[] = {
	{ .compatible = "edgeq,raptor2-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_raptor2_dt_ids);

static struct platform_driver sdhci_raptor2_driver = {
	.driver	= {
		.name	= "raptor2-sdhci",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_raptor2_dt_ids,
		.pm = &raptor2_pmops,
	},
	.probe	= raptor2_probe,
	.remove	= raptor2_remove,
};
module_platform_driver(sdhci_raptor2_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Synopsys DWC MSHC");
MODULE_AUTHOR("Nilesh Waghmare <nilesh.waghmare@edgeq.io>");
MODULE_LICENSE("GPL v2");
