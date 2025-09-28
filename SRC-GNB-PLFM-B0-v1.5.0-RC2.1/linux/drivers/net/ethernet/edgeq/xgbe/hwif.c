// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
 * XGMAC HW Interface Handling
 */

#include "common.h"
#include "xgbe.h"
#include "xgmac_ptp.h"

static u32 xgmac_get_id(struct xgmac_priv *priv, u32 id_reg)
{
	u32 reg = readl(priv->ioaddr + id_reg);

	if (!reg) {
		dev_info(priv->device, "Version ID not available\n");
		return 0x0;
	}

	dev_info(priv->device, "User ID: 0x%x, Synopsys ID: 0x%x\n",
			(unsigned int)(reg & GENMASK(15, 8)) >> 8,
			(unsigned int)(reg & GENMASK(7, 0)));
	return reg & GENMASK(7, 0);
}

static u32 xgmac_get_dev_id(struct xgmac_priv *priv, u32 id_reg)
{
	u32 reg = readl(priv->ioaddr + id_reg);

	if (!reg) {
		dev_info(priv->device, "Version ID not available\n");
		return 0x0;
	}

	return (reg & GENMASK(15, 8)) >> 8;
}

static int xgmac_dwxlgmac_quirks(struct xgmac_priv *priv)
{
	priv->hw->xlgmac = true;
	return 0;
}

static const struct xgmac_hwif_entry {
	bool gmac;
	bool gmac4;
	bool xgmac;
	u32 min_id;
	u32 dev_id;
	const struct xgmac_regs_off regs;
	const void *desc;
	const void *dma;
	const void *mac;
	const void *hwtimestamp;
	const void *mode;
	const void *tc;
	const void *mmc;
	int (*setup)(struct xgmac_priv *priv);
	int (*quirks)(struct xgmac_priv *priv);
} xgmac_hw[] = {
	/* NOTE: New HW versions shall go to the end of this table */
	{
		.gmac = false,
		.gmac4 = false,
		.xgmac = true,
		.min_id = DWXGMAC_CORE_2_10,
		.dev_id = DWXGMAC_ID,
		.regs = {
			.ptp_off = PTP_XGMAC_OFFSET,
			.mmc_off = MMC_XGMAC_OFFSET,
		},
		.desc = &dwxgmac210_desc_ops,
		.dma = &dwxgmac210_dma_ops,
		.mac = &dwxgmac210_ops,
		.hwtimestamp = &xgmac_ptp,
		.mode = NULL,
		.tc = &dwxgmac210_tc_ops,
		.mmc = &dwxgmac_mmc_ops,
		.setup = dwxgmac2_setup,
		.quirks = NULL,
	}, {
		.gmac = false,
		.gmac4 = false,
		.xgmac = true,
		.min_id = DWXLGMAC_CORE_2_00,
		.dev_id = DWXLGMAC_ID,
		.regs = {
			.ptp_off = PTP_XGMAC_OFFSET,
			.mmc_off = MMC_XGMAC_OFFSET,
		},
		.desc = &dwxgmac210_desc_ops,
		.dma = &dwxlgmac2_dma_ops,
		.mac = &dwxlgmac2_ops,
		.hwtimestamp = &xgmac_ptp,
		.mode = NULL,
		.tc = &dwxgmac210_tc_ops,
		.mmc = &dwxgmac_mmc_ops,
		.setup = dwxlgmac2_setup,
		.quirks = xgmac_dwxlgmac_quirks,
	},
};

int xgmac_hwif_init(struct xgmac_priv *priv)
{
	bool needs_xgmac = priv->plat->has_xgmac;
	const struct xgmac_hwif_entry *entry;
	struct mac_device_info *mac;
	bool needs_setup = true;
	u32 id, dev_id = 0;
	int i, ret;

	if (needs_xgmac) {
		id = xgmac_get_id(priv, XGMAC_VERSION);
		dev_id = xgmac_get_dev_id(priv, XGMAC_VERSION);
	} else {
		id = 0;
	}

	/* Save ID for later use */
	priv->synopsys_id = id;
	priv->dev_id = dev_id;

	/* Lets assume some safe values first */
	priv->ptpaddr = priv->ioaddr + PTP_XGMAC_OFFSET;
	priv->mmcaddr = priv->ioaddr + MMC_XGMAC_OFFSET;

	/* Check for HW specific setup first */
	if (priv->plat->setup) {
		mac = priv->plat->setup(priv);
		needs_setup = false;
	} else {
		mac = devm_kzalloc(priv->device, sizeof(*mac), GFP_KERNEL);
	}

	if (!mac)
		return -ENOMEM;

	/* Fallback to generic HW */
	for (i = ARRAY_SIZE(xgmac_hw) - 1; i >= 0; i--) {
		entry = &xgmac_hw[i];

		if (needs_xgmac ^ entry->xgmac)
			continue;
		/* Use synopsys_id var because some setups can override this */
		if (priv->synopsys_id < entry->min_id)
			continue;
		if (needs_xgmac && (dev_id ^ entry->dev_id))
			continue;

		/* Only use generic HW helpers if needed */
		mac->desc = mac->desc ? : entry->desc;
		mac->dma = mac->dma ? : entry->dma;
		mac->mac = mac->mac ? : entry->mac;
		mac->ptp = mac->ptp ? : entry->hwtimestamp;
		mac->mode = mac->mode ? : entry->mode;
		mac->tc = mac->tc ? : entry->tc;
		mac->mmc = mac->mmc ? : entry->mmc;

		priv->hw = mac;
		priv->ptpaddr = priv->ioaddr + entry->regs.ptp_off;
		priv->mmcaddr = priv->ioaddr + entry->regs.mmc_off;

		/* Entry found */
		if (needs_setup) {
			ret = entry->setup(priv);
			if (ret)
				return ret;
		}

		/* Save quirks, if needed for posterior use */
		priv->hwif_quirks = entry->quirks;
		return 0;
	}

	dev_err(priv->device, "Failed to find HW IF (id=0x%x)\n", id);
	return -EINVAL;
}
