// SPDX-License-Identifier: GPL-2.0
/* EdgeQ USB Device Driver
 *
 * Authors: Aman Sharma <aman.sharma@edgeq.io>,
 * Aakash Verma <aakash.verma@edgeq.io>
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/of_clk.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/interconnect.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/usb/of.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include "../drivers/soc/edgeq/eeprom/eeprom_data.h"
#include "core.h"
#include "dwc3-edgeq.h"
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>

#define EQUSB_DBG_EN		0

#if EQUSB_DBG_EN
#define EQUSB_DBG(...)		pr_debug(fmt, ##__VA_ARGS__)
#else
#define EQUSB_DBG(...)
#endif

static inline void dwc3_edgeq_writel(void __iomem *base, uint32_t offset, uint64_t mask, uint32_t val)
{
	uint32_t reg_val;

	reg_val = readl(base + offset);
	reg_val &= (~mask);
	reg_val |= val;
	writel(reg_val, (base + offset));

	/* ensure that above write is through */
	reg_val = readl(base + offset);

	EQUSB_DBG("%s:%d mask=%llx value=%x reg_loc=%x readback_reg_val=%x\n",
			__func__, __LINE__, mask, val, offset, reg_val);
}

static inline void dwc3_edgeq_writel_raw(void __iomem *base, uint32_t offset, uint32_t val)
{	uint32_t reg_val;

	writel(val, (base + offset));
	/* ensure that above write is through */
	reg_val = readl(base + offset);
	EQUSB_DBG("%s:%d value=%x reg_loc=%x readback_reg_val=%x\n",
			__func__, __LINE__, val, offset, reg_val);
}

static inline uint32_t dwc3_edgeq_readl(void __iomem *base, uint32_t offset)
{
	uint32_t read_val = 0;

	read_val = readl(base+offset);
	EQUSB_DBG("%s:%d reg_loc=%p reg_val=%x\n",
			__func__, __LINE__, (base + offset), read_val);
	return read_val;
}

/* Bringup Configuration */
static void dwc3_edgeq_shutdown_signal(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_writel(edgeq->bringup_config_base,
				BRINGUP_CONFIG_4, SHUTDOWN_BIT_MASK, 0x0);
}

static void dwc3_edgeq_vcc_reset_signal(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_writel(edgeq->bringup_config_base,
				BRINGUP_CONFIG_0, VCC_RESET_N_MASK, VCC_RESET_N);
}

static void dwc3_edgeq_host_set_u2_u3_num(struct dwc3_edgeq *edgeq, int u2, int u3)
{
	dwc3_edgeq_writel(edgeq->bringup_config_base, BRINGUP_CONFIG_2_L,
				(HOST_NUM_U2_PORT_MASK|HOST_NUM_U3_PORT_MASK),
				(HOST_NUM_U2_PORT(u2)|HOST_NUM_U3_PORT(u3)));
}

static void dwc3_edgeq_host_xhc_bme(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_writel(edgeq->bringup_config_base, BRINGUP_CONFIG_2_H,
				XHC_BME_MASK, XHC_BME);
}

static void dwc3_edgeq_bringup_init(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_shutdown_signal(edgeq);
	dwc3_edgeq_vcc_reset_signal(edgeq);

	/* Set number of U2/U3 ports for host mode and XHC BME*/
	if (edgeq->mode == USB_DR_MODE_HOST) {
		dwc3_edgeq_host_set_u2_u3_num(edgeq, NUM_USB2_PORT,
						NUM_USB3_PORT);
		dwc3_edgeq_host_xhc_bme(edgeq);
	}
	/*#Setting the fladg_30mhz_reg to its default value
	 *#Setting 33:28 bits to 'd32('h20 or 'b10_0000)
	 *Frame Jitter Error Resolution
	 */
	dwc3_edgeq_writel(edgeq->bringup_config_base, PHY_MISC_CONFIG_0,
				0xF0000000, 0x00000000);
	dwc3_edgeq_writel(edgeq->bringup_config_base, PHY_MISC_CONFIG_0_1,
				0x00000003, 0x00000002);

	dwc3_edgeq_vcc_reset_signal(edgeq);
}

/* USB Phy Initialization */
static void dwc3_edgeq_refclk_sel(struct dwc3_edgeq *edgeq, int clksel)
{
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				REF_CLK_SEL_MASK, REF_CLK_SEL(clksel));
}

static void dwc3_edgeq_freq_sel(struct dwc3_edgeq *edgeq, int freq_sel)
{
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1,
				FREQ_SEL_MASK, FREQ_SEL(freq_sel));
}

static void dwc3_edgeq_ssp_en(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_2_H,
				REF_SSP_EN_MASK, REF_SSP_EN);
}

static void dwc3_edgeq_phy_reset(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_2_L,
				PHY_RESET_MASK, 0x0);
}

static void dwc3_edgeq_alt_clk_and_pad_en(struct dwc3_edgeq *edgeq, int int_clk)
{
	if (int_clk)
		dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_3,
					(REF_USE_PAD_MASK|ALT_CLK_EN_MASK),
					(ALT_CLK_EN));
	else
		dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_3,
					(REF_USE_PAD_MASK|ALT_CLK_EN_MASK),
					(ALT_CLK_EN|REF_USE_PAD));
}

static void dwc3_edgeq_phy_init(struct dwc3_edgeq *edgeq, int int_clk)
{
	uint32_t ssc_range = 0;

	dwc3_edgeq_refclk_sel(edgeq, REF_CLK_SEL_VAL);
	dwc3_edgeq_freq_sel(edgeq, FREQ_SEL_VAL);
	dwc3_edgeq_ssp_en(edgeq);
	/* Setting RX0LOSLFPSEN = 1*/
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0x10000, 0x10000);
	/* echo "added RX0LOSLFPSEN = 1"
	 * COMPDISTUNE set to 100
	 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0x4, 0x4);
	/* OTGTUNE0 set to 100 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0x400, 0x400);
	/* SQRXTUNE0 set to 011 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0x700000, 0x300000);
	/* TXFSLSTUNE0 set to 0011 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0xF000000, 0x3000000);
	/* TXHSHVTUNE0 set to 11 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0,
				0x30000000, 0x30000000);
	/* TXRESTUNE0 set to 01 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0_1,
				0x300, 0x100);
	/* TXRISETUNE set to 10 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0_1,
				0x3000, 0x2000);
	/* TXVREFTUNE0 set to 1000 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_0_1,
				0xF0000, 0x80000);
	/* VDATREFTUNE set to 01 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1,
				0x3, 0x1);
	/* los_bias set to 101 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1,
				0x700000, 0x500000);
	/* los_level set 01001 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1,
				0x1F000000, 0x09000000);
	/* pcs_tx_deemph_3p5db set to 'd21 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1_1,
				0x3F000, 0x28000);
	/* pcs_tx_deemph_6db set to 'd32 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_1_1,
				0x3F00000, 0x2800000);
	/* pcs_tx_swing_full set to 7'b1111111 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_2_L,
				0x7F00, 0x7F00);
	/* tx_vboost_lvl set to 100 */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_2_H,
				0x7000000, 0x7000000);

	/* ssc_range */
	dwc3_edgeq_writel(edgeq->phy_misc_base, PHY_MISC_CONFIG_2_H,
				0x0000700, (ssc_range << 8));

	dwc3_edgeq_alt_clk_and_pad_en(edgeq, int_clk);
	dwc3_edgeq_phy_reset(edgeq);
}

static void dwc3_edgeq_read_detect_rx(struct dwc3_edgeq *edgeq, uint64_t addr)
{
	uint64_t cr_addr = 0x10000|addr;
	uint64_t data = 0x0;
	static unsigned long timeout_jiffies;

	dwc3_edgeq_writel_raw(edgeq->phy_misc_base, 0x44, 0x0);
	dwc3_edgeq_writel_raw(edgeq->phy_misc_base, 0x40, cr_addr);
	pr_debug("USB RX Detect Read: Polling 1\n");

	/* Initialize timeout value */
	timeout_jiffies = jiffies + TIMEOUT_JIFFIES;

	/* Replace the do-while loop with a while loop and timeout */
	while (!(dwc3_edgeq_readl(edgeq->phy_misc_base, 0x108) & 0x1)) {

		/* Check for timeout */
		if (time_after(jiffies, timeout_jiffies)) {
			pr_debug("USB RX Detect Read: Timeout occurred!\n");
			/* Exit the loop on timeout */
			break;
		}

		/* Sleep for a short period to avoid busy-waiting */
		msleep(100);
	}
	dwc3_edgeq_writel_raw(edgeq->phy_misc_base, 0x40, 0x0);
	dwc3_edgeq_writel_raw(edgeq->phy_misc_base, 0x44, 0x1);
	pr_debug("USB RX Detect Read: Polling 2\n");

	/* Replace the do-while loop with a while loop and timeout */
	while (!(dwc3_edgeq_readl(edgeq->phy_misc_base, 0x108) & 0x1)) {

		/* Check for timeout */
		if (time_after(jiffies, timeout_jiffies)) {
			pr_debug("USB RX Detect Read: Timeout occurred!\n");
			/* Exit the loop on timeout */
			break;
		}

		/* Sleep for a short period to avoid busy-waiting */
		msleep(100);
	}
	data = dwc3_edgeq_readl(edgeq->phy_misc_base, 0x108);
	EQUSB_DBG("%s:Read data 1 - %llx\n", __func__, data);
	dwc3_edgeq_writel_raw(edgeq->phy_misc_base, 0x44, 0x0);

	data = dwc3_edgeq_readl(edgeq->phy_misc_base, 0x108);
	EQUSB_DBG("%s:Read data 2 - %llx\n", __func__,
				((data & 0xffff00) >> 8));

}

static void dwc3_edgeq_phy_config(struct dwc3_edgeq *edgeq)
{
	dwc3_edgeq_read_detect_rx(edgeq, 0x0);
}

static int dwc3_edgeq_core_populate(struct platform_device *pdev)
{
	struct dwc3_edgeq	*edgeq = platform_get_drvdata(pdev);
	struct device_node	*np = pdev->dev.of_node, *dwc3_np;
	struct device		*dev = &pdev->dev;
	int			ret;

	dwc3_np = of_get_child_by_name(np, "dwc3");
	if (!dwc3_np) {
		dev_err(dev, "failed to find dwc3 core child\n");
		return -ENODEV;
	}

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to register dwc3 core - %d\n", ret);
		return ret;
	}

	edgeq->dwc3 = of_find_device_by_node(dwc3_np);
	if (!edgeq->dwc3) {
		dev_err(dev, "failed to get dwc3 platform device\n");
		return -ENODEV;
	}

	return 0;
}

static int dwc3_edgeq_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;
	struct device_node	*eeprom_np;
	struct dwc3_edgeq	*edgeq;
	struct device_node	*child;
	int			ret;
	const char		*mode;
	uint32_t		int_clk;
	int			rc;
	struct device_node	*mem;
	struct resource		mem_res;
	void			*vaddr;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_vbus;
	struct gpio_desc *vbus_gpio;


	dev_info(dev, "Probing EdgeQ DWC3\n");
	mode = devm_kzalloc(&pdev->dev, sizeof(char)*64, GFP_KERNEL);
	if (!mode)
		return -ENOMEM;

	edgeq = devm_kzalloc(&pdev->dev, sizeof(*edgeq), GFP_KERNEL);
	if (!edgeq)
		return -ENOMEM;

	platform_set_drvdata(pdev, edgeq);
	edgeq->dev = &pdev->dev;

	/* Get Base address from DT */
	edgeq->bringup_config_base = devm_platform_ioremap_resource_byname(pdev, "bringup");
	if (IS_ERR(edgeq->bringup_config_base))
		return PTR_ERR(edgeq->bringup_config_base);

	edgeq->phy_misc_base = devm_platform_ioremap_resource_byname(pdev, "phy_misc");
	if (IS_ERR(edgeq->phy_misc_base))
		return PTR_ERR(edgeq->phy_misc_base);

	/* Reading Board-type property from EEPROM*/
	eeprom_np = of_find_node_by_name(NULL, "eeprom");
	if (!eeprom_np) {
		pr_err("Device not found in the device tree\n");
		return -ENODEV;
	}

	mem = of_parse_phandle(eeprom_np, "memory-region", 0);
	if (!mem) {
		pr_err("Failed to get phandle from np\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(mem, 0, &mem_res);
	if (rc) {
		pr_err("Failed to get address from np\n");
		return -EINVAL;
	}

	vaddr = ioremap_cache(mem_res.start, resource_size(&mem_res));
	if (!vaddr) {
		dev_err(&pdev->dev, "Failed to read EEPROM address\n");
		return -ENOMEM;
	}

	/* Read the "int-clk" property */
	if (of_property_read_u32(np, "int-clk", &int_clk) != 0) {
		dev_err(&pdev->dev, "Failed to read Internal Clock property\n");
		return -EINVAL;
	}

	/* Get the child node */
	child = of_get_child_by_name(np, "dwc3");
	if (!child) {
		dev_err(&pdev->dev, "failed to find dwc3 core node\n");
		ret = -ENODEV;
		goto child_node_put;
	}
	/* Get USB mode from DT */
	if (of_property_read_string(child, "dr_mode", &mode) != 0)
		dev_err(&pdev->dev, "failed to find dwc3 mode\n");

	if (!strcmp(mode, "host"))
		edgeq->mode = USB_DR_MODE_HOST;
	else
		edgeq->mode = USB_DR_MODE_PERIPHERAL;

	dev_info(&pdev->dev, "USB mode as per DT - %s %d\n", mode, edgeq->mode);
	of_node_put(child);

	dwc3_edgeq_phy_init(edgeq, int_clk);
	dwc3_edgeq_phy_config(edgeq);
	dwc3_edgeq_bringup_init(edgeq);

	/* Get the pinctrl handle from the device */
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	if (edgeq->mode == USB_DR_MODE_HOST) {
		/* Lookup the pin state */
		pinctrl_state_vbus = pinctrl_lookup_state(pinctrl, "vbus");
		if (IS_ERR(pinctrl_state_vbus)) {
			dev_err(&pdev->dev, "Failed to lookup default pinctrl vbus state\n");
			return PTR_ERR(pinctrl_state_vbus);
		}

		/* Select the pin state */
		ret = pinctrl_select_state(pinctrl, pinctrl_state_vbus);
		if (ret) {
			dev_err(&pdev->dev, "Failed to select default pinctrl vbus state\n");
			return ret;
		}

		/* Request the GPIO */
		vbus_gpio = devm_gpiod_get(&pdev->dev, "vbus", GPIOD_OUT_HIGH);
		if (IS_ERR(vbus_gpio)) {
			dev_err(&pdev->dev, "Failed to get vbus GPIO\n");
			return PTR_ERR(vbus_gpio);
		}
		/* Set the GPIO dir to output */
		gpiod_direction_output(vbus_gpio, 1);
	}

	/* Populate dwc3 core */
	ret = dwc3_edgeq_core_populate(pdev);
	if (ret) {
		dev_err(dev, "failed to register DWC3 Core, err=%d\n", ret);
		goto depopulate;
	}

	dev_alert(dev, "Probe EdgeQ DWC3 Complete\n");
	return 0;

child_node_put:
	of_node_put(child);
depopulate:
	if (np)
		of_platform_depopulate(&pdev->dev);
	else
		platform_device_put(pdev);

	return ret;
}

static int dwc3_edgeq_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	of_platform_depopulate(dev);

	return 0;
}

static const struct of_device_id dwc3_edgeq_of_match[] = {
	{ .compatible = "edgeq,dwc3" },
	{ }
};

MODULE_DEVICE_TABLE(of, dwc3_edgeq_of_match);

static struct platform_driver dwc3_edgeq_driver = {
	.probe		= dwc3_edgeq_probe,
	.remove		= dwc3_edgeq_remove,
	.driver		= {
	.name		= "dwc3-edgeq",
	.of_match_table = dwc3_edgeq_of_match,
	},
};
module_platform_driver(dwc3_edgeq_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare DWC3 EDGEQ Glue Driver");
