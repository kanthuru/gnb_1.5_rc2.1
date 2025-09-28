// SPDX-License-Identifier: GPL-2.0-only
/*
 *  EdgeQ Raptor2 Pin Control Driver
 *  Copyright (C) 2024 EdgeQ Inc.
 *  Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "pinctrl-edgeq.h"

/* Macros to help us access the registers */

#define MFP_R2(a, f0, f1, f2, f3, f4, f5, f6)	\
{				\
	.name = #a,		\
	.pin = a,		\
	.func = {		\
		R2_MUX_##f0,	\
		R2_MUX_##f1,	\
		R2_MUX_##f2,	\
		R2_MUX_##f3,	\
		R2_MUX_##f4,	\
		R2_MUX_##f5,	\
		R2_MUX_##f6,	\
	},			\
}

/* These are the offsets to our controller registers */
#define GPIO_OFFSET		0x20
#define EVEN_PIN(x)		(GPIO_OFFSET + (x * 2))
#define ODD_PIN(x)		(GPIO_OFFSET + ((x - 1) * 2))
#define SEL_PIN(pin)		(((pin) % 2 == 0) ? EVEN_PIN(pin) : ODD_PIN(pin))

#define GPIO_BASE(p)		(GPIO_OFFSET * (p * 2))

#define PADMUX_OFFSET		1	/* 3:1 for mux control */
#define PINCTRL_OUT_0		4
#define PINCTRL_OE_0		6
#define PINCTRL_IE_0		5
#define PINCTRL_PD_OFFSET       7
#define PINCTRL_PU_OFFSET       8
#define PINCTRL_DS_OFFSET	9
#define PADMUX_MASK		7
#define PINCTRL_DS_MASK         0xF
#define CURRENT_GPIO_OFFSET	0
#define NEXT_GPIO_OFFSET	16
#define GPIO_ALT(f)		(f << PADMUX_OFFSET)
#define MAX_DS_VALUE		0xF
#define EVEN_MUX               1
#define ODD_MUX                        16
#define GIC_SPI_OFFSET         32
#define SIF_IN                 1
#define INPUT_REGISTER_OFFSET  0xC0

/* Macros to help us access the registers */
#define r2_gpio_getbit(m, r, p)		(!!(__raw_readl(m + r) & (BIT(p))))

#define r2_gpio_setbit(m, r, p)		\
	(__raw_writel(((__raw_readl(m + r) & (~(0))) | (BIT(p))), (m + r)))

#define r2_gpio_clearbit(m, r, p)	\
	(__raw_writel((__raw_readl(m + r) & ~(BIT(p))) | (0), (m + r)))

#define r2_gpio_rmw(base, offset, val, odd, mask, bitoffset) \
	(__raw_writel((((__raw_readl(base + offset)) & ~(mask << (bitoffset + odd))) \
		       | (val << (bitoffset + odd))), (base + offset)))

#define r2_gpio_getmux(m, r, p)                (!!(__raw_readl(m + r) & (7 << p)))

enum r2_mux {
	R2_MUX_GPIO = 0,
	R2_MUX_BSS_SPIM_1,
	R2_MUX_BSS_SPIM_2,
	R2_MUX_SIF_EVENT_IN,
	R2_MUX_BSS_TIM_SIG,
	R2_MUX_BSS_UART1,
	R2_MUX_BSS_UART2,
	R2_MUX_BSS_UART3,
	R2_MUX_USB0_UART,
	R2_MUX_USB1_UART,
	R2_MUX_EFUSE,
	R2_MUX_SIF_EVENT_OUT,
	R2_MUX_UART2,
	R2_MUX_USIM,
	R2_MUX_BSS_I2C2,
	R2_MUX_BSS_I2C3,
	R2_MUX_SD_CARD,
	R2_MUX_BSS_WD_RESET_N,
	R2_MUX_BSS_NMI_INT,
	R2_MUX_SOC_DIV_PLL_VCO,
	R2_MUX_IOSS_DIV_PLL0_VCO,
	R2_MUX_IOSS_DIV_PLL1_VCO,
	R2_MUX_OTRX_DIV_PLL_VCO,
	R2_MUX_SPU_DIV_DAC_VCO,
	R2_MUX_SPU_DIV_ADC_VCO,
	R2_MUX_DDR_DIV_PLL_VCO,
	R2_MUX_DDR_DTO,
	R2_MUX_CPUSS_DIV_PLL_VCO,
	R2_MUX_CRYPTO_DIV_PLL_VCO,
	R2_MUX_CPUCORE_DIV_PLL_VCO,
	R2_MUX_TCIP_DIV_PLL_VCO,
	R2_MUX_TXU_DIV_PLL_VCO,
	R2_MUX_IOSS_USB_PLL_LOCK,
	R2_MUX_IOSS_LINK1_PLL_LOCK,
	R2_MUX_IOSS_LINK2_PLL_LOCK,
	R2_MUX_IOSS_LINK3_PLL_LOCK,
	R2_MUX_IOSS_LINK4_PLL_LOCK,
	R2_MUX_CPU0_UART,
	R2_MUX_CPU1_UART,
	R2_MUX_NONE = 0xffff,
};


struct gpio_lookup_irq {
	int gpio_num;
	int sif_in;
	int sif_out;
	char *irq_name;
};

struct gpio_lookup_irq gpio_irq[] = {
	{0, 190, 102, "gpio0"},
	{1, 191, 103, "gpio1"},
	{2, 192, 104, "gpio2"},
	{3, 193, 105, "gpio3"},
	{4, 194, 106, "gpio4"},
	{5, 195, 107, "gpio5"},
	{6, 196, 108, "gpio6"},
	{7, 197, 109, "gpio7"},
	{8, 198, 110, "gpio8"},
	{9, 199, 111, "gpio9"},
	{10, 200, 112, "gpio10"},
	{11, 201, 113, "gpio11"},
	{12, 202, 114, "gpio12"},
	{13, 203, 115, "gpio13"},
	{14, 204, 116, "gpio14"},
	{15, 205, 117, "gpio15"},
	{16, 206, 118, "gpio16"},
	{17, 207, 119, "gpio17"},
	{18, 208, 120, "gpio18"},
	{19, 209, 121, "gpio19"},
	{20, 210, 122, "gpio20"},
	{21, 211, 123, "gpio21"},
	{22, 212, 124, "gpio22"},
	{23, 213, 125, "gpio23"},
	{24, 214, 126, "gpio24"},
	{25, 215, 127, "gpio25"},
	{26, 216, 128, "gpio26"},
	{27, 217, 129, "gpio27"},
	{28, 218, 130, "gpio28"},
	{29, 219, 131, "gpio29"},
	{30, 220, 132, "gpio30"},
	{31, 221, 133, "gpio31"},
	{32, 222, 134, "gpio32"},
	{33, 223, 135, "gpio33"},
	{34, 224, 136, "gpio34"},
	{35, 225, 137, "gpio35"},
	{36, 226, 138, "gpio36"},
	{37, 227, 139, "gpio37"},
	{38, 228, 140, "gpio38"},
	{39, 229, 141, "gpio39"},
	{40, 230, 142, "gpio40"},
	{41, 231, 143, "gpio41"},
	{42, 232, 144, "gpio42"},
	{43, 233, 145, "gpio43"},
	{44, 234, 146, "gpio44"},
	{45, 235, 147, "gpio45"},
	{46, 236, 148, "gpio46"},
	{47, 237, 149, "gpio47"},
	{48, 238, 150, "gpio48"},
	{49, 239, 151, "gpio49"},
	{50, 240, 152, "gpio50"},
	{51, 241, 153, "gpio51"},
	{52, 242, 154, "gpio52"},
	{53, 243, 155, "gpio53"},
};

#define GRP_MUX(a, m, p)		\
	{ .name = a, .mux = R2_MUX_##m, .pins = p, .npins = ARRAY_SIZE(p), }

/* Raptor2 SoC gpio related code */
#define R2_SOC_MAX_PIN		64
#define PINS			64

static const struct eq_mfp_pin r2_socgpio_mfp[] = {
/*     pin    f0    f1	          f2	       f3	  f4               f5     f6      */
MFP_R2(GPIO0, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, SOC_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO1, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, SOC_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO2, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, IOSS_DIV_PLL0_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO3, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, IOSS_DIV_PLL0_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO4, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, IOSS_DIV_PLL1_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO5, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, IOSS_DIV_PLL1_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO6, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART1, OTRX_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO7, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, OTRX_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO8, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, SPU_DIV_DAC_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO9, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, SPU_DIV_DAC_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO10, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, SPU_DIV_ADC_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO11, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, SPU_DIV_ADC_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO12, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, DDR_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO13, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART2, DDR_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO14, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, DDR_DTO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO15, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CRYPTO_DIV_PLL_VCO, EFUSE,
		SIF_EVENT_OUT),
MFP_R2(GPIO16, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CRYPTO_DIV_PLL_VCO, EFUSE,
		SIF_EVENT_OUT),
MFP_R2(GPIO17, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CPUSS_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO18, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CPUSS_DIV_PLL_VCO, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO19, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CPUCORE_DIV_PLL_VCO, EFUSE,
		SIF_EVENT_OUT),
MFP_R2(GPIO20, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_UART3, CPUCORE_DIV_PLL_VCO, EFUSE,
		SIF_EVENT_OUT),
MFP_R2(GPIO21, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, NONE, IOSS_LINK1_PLL_LOCK, EFUSE, SIF_EVENT_OUT),
MFP_R2(GPIO22, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_WD_RESET_N, IOSS_LINK2_PLL_LOCK, EFUSE,
		SIF_EVENT_OUT),
MFP_R2(GPIO23, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_NMI_INT, IOSS_LINK3_PLL_LOCK, NONE,
		SIF_EVENT_OUT),
MFP_R2(GPIO24, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, IOSS_LINK4_PLL_LOCK, NONE,
		SIF_EVENT_OUT),
MFP_R2(GPIO25, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, IOSS_USB_PLL_LOCK, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO26, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, TCIP_DIV_PLL_VCO, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO27, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, TCIP_DIV_PLL_VCO, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO28, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, TXU_DIV_PLL_VCO, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO29, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, TXU_DIV_PLL_VCO, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO30, GPIO, NONE,	    NONE,        NONE,            NONE, NONE, NONE),
MFP_R2(GPIO31, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO32, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO33, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO34, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO35, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO36, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO37, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU1_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO38, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, CPU0_UART, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO39, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, NONE, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO40, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, SD_CARD, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO41, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, SD_CARD, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO42, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_I2C2, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO43, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_I2C2, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO44, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_I2C3, NONE, BSS_UART1, SIF_EVENT_OUT),
MFP_R2(GPIO45, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_I2C3, NONE, BSS_UART1, SIF_EVENT_OUT),
MFP_R2(GPIO46, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_1, NONE, BSS_UART2, SIF_EVENT_OUT),
MFP_R2(GPIO47, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_1, NONE, BSS_UART2, SIF_EVENT_OUT),
MFP_R2(GPIO48, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_1, NONE, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO49, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_1, USIM, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO50, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_2, USIM, BSS_UART3, SIF_EVENT_OUT),
MFP_R2(GPIO51, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_2, USIM, BSS_UART3, SIF_EVENT_OUT),
MFP_R2(GPIO52, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_2, USIM, NONE, SIF_EVENT_OUT),
MFP_R2(GPIO53, GPIO, SIF_EVENT_IN, BSS_TIM_SIG, BSS_SPIM_2, USIM, NONE, SIF_EVENT_OUT),
};

static const unsigned int r2_pins_spi_1_cs_n[] = {GPIO47};
static const unsigned int r2_pins_gpio40[] = {GPIO40};
static const unsigned int r2_pins_gpio41[] = {GPIO41};
static const unsigned int r2_pins_gpio47[] = {GPIO47};
static const unsigned int r2_pins_spi_grp[] = {GPIO46, GPIO48, GPIO49};
static const unsigned int r2_pins_sif[] = {GPIO21};

static const struct eq_pin_group r2_socgpio_grps[] = {
	GRP_MUX("bss_spim_1_cs_n", BSS_SPIM_1, r2_pins_spi_1_cs_n),
	GRP_MUX("spim_1_clk_miso_mosi", BSS_SPIM_1, r2_pins_spi_grp),
	GRP_MUX("gpio40", GPIO, r2_pins_gpio40),
	GRP_MUX("gpio41", GPIO, r2_pins_gpio41),
	GRP_MUX("gpio47", GPIO, r2_pins_gpio47),
	GRP_MUX("sif_in", SIF_EVENT_IN, r2_pins_sif),
};

static const char * const eq_spi_grps[] = {"bss_spim_1_clk", "bss_spim_1_cs_n", "bss_spim_1_miso",
					"bss_spim_1_mosi", "bss_spim_2_clk", "bss_spim_2_cs_n",
					"bss_spim_2_miso", "bss_spim_2_mosi",
					"spim_1_clk_miso_mosi"};
static const char * const eq_socgpio_grps[] = {"gpio40", "gpio41", "gpio47"};
static const char * const eq_sif_grps[] = {"sif_in"};

/* Functions which needs to be specified in the dts */
static const struct eq_pmx_func r2_socgpio_funcs[] = {
	{"spi",		eq_spi_grps, ARRAY_SIZE(eq_spi_grps)},
	{"gpio",	eq_socgpio_grps, ARRAY_SIZE(eq_socgpio_grps)},
	{"sif_input",   eq_sif_grps, ARRAY_SIZE(eq_sif_grps)},
};

static const unsigned int r2_exin_pin_map[] = {GPIO0, GPIO1, GPIO2};

/* Register the pinctrl layer */
struct r2_soc_pinctrl {
	struct platform_device *p_dev;
	int pin_count;
	const struct eq_mfp_pin *mfp;
	const struct eq_pin_group *grps;
	unsigned int num_grps;
	const struct eq_pmx_func *funcs;
	unsigned int num_funcs;
	const unsigned int *exin;
	unsigned int num_exin;
};

/* EdgeQ Raptor2 Family */
static struct r2_soc_pinctrl rapto2_soc_pinctrl = {
	.pin_count = R2_SOC_MAX_PIN,
	.mfp = r2_socgpio_mfp,
	.grps = r2_socgpio_grps,
	.num_grps = ARRAY_SIZE(r2_socgpio_grps),
	.funcs = r2_socgpio_funcs,
	.num_funcs = ARRAY_SIZE(r2_socgpio_funcs),
	.exin = r2_exin_pin_map,
	.num_exin = 3
};

/* Pinconf related code */
static int r2_pinconf_get(struct pinctrl_dev *pctrldev,
		unsigned int pin,
		unsigned long *config)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	enum pin_config_param param = pinconf_to_config_param(*config);

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(pin % 2)) {
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						EVEN_PIN(pin), PINCTRL_PU_OFFSET));
		} else
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_PU_OFFSET));

		dev_dbg(pctrldev->dev, "%d pin %d *config %lx param %d \r\n", __LINE__,
				pin, *config, param);
	break;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!(pin % 2)) {
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						EVEN_PIN(pin), PINCTRL_PD_OFFSET));
		} else
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_PD_OFFSET));
		dev_dbg(pctrldev->dev, "%d pin %d *config %lx param %d \r\n", __LINE__, pin,
				*config, param);
	break;

	case PIN_CONFIG_OUTPUT:
		if (!(pin % 2)) {
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						EVEN_PIN(pin), PINCTRL_OUT_0));
		} else
			*config = pinconf_to_config_packed(param,
					r2_gpio_getbit(info->membase,
						ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_OUT_0));
		dev_dbg(pctrldev->dev, "%d pin %d *config %lx param %d \r\n", __LINE__, pin,
				*config, param);
	break;

	default:
		dev_err(pctrldev->dev, "Invalid config param %04x\n", param);
		return -EOPNOTSUPP;
	}
	return 0;
}

static int r2_pinconf_set(struct pinctrl_dev *pctrldev,
				unsigned int pin,
				unsigned long *configs,
				unsigned int num_configs)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	enum pin_config_param param;
	int arg;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_STRENGTH:
			if (arg && (arg <= MAX_DS_VALUE)) {
				if (!(pin % 2))
					r2_gpio_rmw(info->membase,
							EVEN_PIN(pin), arg,
							CURRENT_GPIO_OFFSET,
							PINCTRL_DS_MASK,
							PINCTRL_DS_OFFSET);
				else
					r2_gpio_rmw(info->membase,
							ODD_PIN(pin), arg,
							NEXT_GPIO_OFFSET,
							PINCTRL_DS_MASK,
							PINCTRL_DS_OFFSET);
			} else
				dev_err(pctrldev->dev,
						"Invalid drive strength value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));

		break;

		case PIN_CONFIG_BIAS_PULL_UP:
			if (!(pin % 2))
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_PD_OFFSET);
			else
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_PD_OFFSET);

			if (arg == 1) {
				if (!(pin % 2))
					r2_gpio_setbit(info->membase,
							EVEN_PIN(pin), PINCTRL_PU_OFFSET);
				else
					r2_gpio_setbit(info->membase,
							ODD_PIN(pin),
							NEXT_GPIO_OFFSET +
							PINCTRL_PU_OFFSET);
			} else if (arg == 0) {
				if (!(pin % 2))
					r2_gpio_clearbit(info->membase,
							EVEN_PIN(pin),
							PINCTRL_PU_OFFSET);
				else
					r2_gpio_clearbit(info->membase,
							ODD_PIN(pin),
							NEXT_GPIO_OFFSET +
							PINCTRL_PU_OFFSET);
			} else
				dev_err(pctrldev->dev,
						"Invalid pull value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));

			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (!(pin % 2))
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_PU_OFFSET);
			else
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET
						+ PINCTRL_PU_OFFSET);

			if (arg == 1) {
				if (!(pin % 2))
					r2_gpio_setbit(info->membase,
							EVEN_PIN(pin), PINCTRL_PD_OFFSET);
				else
					r2_gpio_setbit(info->membase,
							ODD_PIN(pin),
							NEXT_GPIO_OFFSET +
							PINCTRL_PD_OFFSET);
			} else if (arg == 0) {
				if (!(pin % 2))
					r2_gpio_clearbit(info->membase,
							EVEN_PIN(pin),
							PINCTRL_PD_OFFSET);
				else
					r2_gpio_clearbit(info->membase,
							ODD_PIN(pin),
							NEXT_GPIO_OFFSET +
							PINCTRL_PD_OFFSET);
			} else
				dev_err(pctrldev->dev,
						"Invalid pull value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));
			break;

		case PIN_CONFIG_OUTPUT:
			if (!(pin % 2)) {
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_IE_0);
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OE_0);
			} else {
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_IE_0);
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_OE_0);
			}

			if (arg == 1) {
				if (!(pin % 2))
					r2_gpio_setbit(info->membase, EVEN_PIN(pin),
							PINCTRL_OUT_0);
				else
					r2_gpio_setbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_OUT_0);
			} else if (arg == 0) {
				if (!(pin % 2))
					r2_gpio_clearbit(info->membase, EVEN_PIN(pin),
							PINCTRL_OUT_0);
				else
					r2_gpio_clearbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_OUT_0);
			} else
				dev_err(pctrldev->dev,
						"Invalid pull value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));
			break;

		case PIN_CONFIG_OUTPUT_ENABLE:
			if (!(pin % 2)) {
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_IE_0);
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
			} else {
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET
						+ PINCTRL_IE_0);
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET
						+ PINCTRL_OUT_0);
			}

			if (arg == 1) {
				if (!(pin % 2))
					r2_gpio_setbit(info->membase, EVEN_PIN(pin),
							PINCTRL_OE_0);
				else
					r2_gpio_setbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_OE_0);
			} else if (arg == 0) {
				if (!(pin % 2))
					r2_gpio_clearbit(info->membase, EVEN_PIN(pin),
							PINCTRL_OE_0);
				else
					r2_gpio_clearbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_OE_0);
			} else
				dev_err(pctrldev->dev,
						"Invalid pull value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			if (!(pin % 2)) {
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OE_0);
				r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
			} else {
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_OE_0);
				r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
						PINCTRL_OUT_0);
			}

			if (arg == 1) {
				if (!(pin % 2))
					r2_gpio_setbit(info->membase, EVEN_PIN(pin),
							PINCTRL_IE_0);
				else
					r2_gpio_setbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_IE_0);
			} else if (arg == 0) {
				if (!(pin % 2))
					r2_gpio_clearbit(info->membase, EVEN_PIN(pin),
							PINCTRL_IE_0);
				else
					r2_gpio_clearbit(info->membase, ODD_PIN(pin),
							NEXT_GPIO_OFFSET + PINCTRL_IE_0);
			} else
				dev_err(pctrldev->dev,
						"Invalid pull value %d\n", arg);

			dev_dbg(pctrldev->dev, "%d pin %d *config 0x%lx num_configs %d \r\n",
					__LINE__, pin, *configs, num_configs);
			dev_dbg(pctrldev->dev, "param %d arg %d Reg value %x \r\n",
					param, arg, readl(info->membase + SEL_PIN(pin)));
			break;


		default:
			dev_err(pctrldev->dev,
					"Invalid config param %04x\n", param);
			return -EOPNOTSUPP;
	}
	} /* for each config */

	return 0;
}

int r2_pinconf_group_set(struct pinctrl_dev *pctrldev,
		unsigned int selector,
		unsigned long *configs,
		unsigned int num_configs)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	int i, ret = 0;

	dev_dbg(pctrldev->dev, "%d selector %d  *configs %lx num_configs %d \r\n", __LINE__,
			selector, *configs, num_configs);

	for (i = 0; i < info->grps[selector].npins && !ret; i++)
		ret = r2_pinconf_set(pctrldev,
				info->grps[selector].pins[i],
				configs,
				num_configs);

	return ret;
}

static const struct pinconf_ops r2_pinconf_ops = {
	.pin_config_get	= r2_pinconf_get,
	.pin_config_set	= r2_pinconf_set,
	.pin_config_group_set = r2_pinconf_group_set,
};

static struct pinctrl_desc r2_pctrl_desc = {
	.owner		= THIS_MODULE,
	.confops	= &r2_pinconf_ops,
};

static inline int r2_mux_apply(struct pinctrl_dev *pctrldev,
		int mux, int pin_num)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	dev_dbg(pctrldev->dev, "%d reg value %x mux(alt func) %x pin_num %d \r\n", __LINE__,
			readl(info->membase + SEL_PIN(pin_num)), mux, pin_num);
	if (!(pin_num % 2))
		r2_gpio_rmw(info->membase, EVEN_PIN(pin_num), mux, CURRENT_GPIO_OFFSET,
				PADMUX_MASK, PADMUX_OFFSET);
	else
		r2_gpio_rmw(info->membase, ODD_PIN(pin_num), mux, NEXT_GPIO_OFFSET,
				PADMUX_MASK, PADMUX_OFFSET);

	dev_dbg(pctrldev->dev, "%d reg value %x \r\n", __LINE__, readl(info->membase +
				SEL_PIN(pin_num)));

	return 0;
}

static const struct eq_cfg_param r2_cfg_params[] = {
	{"edgeq,pullup",	PIN_CONFIG_BIAS_PULL_UP},
	{"edgeq,pulldown",	PIN_CONFIG_BIAS_PULL_DOWN},
	{"edgeq,drive-strength", PIN_CONFIG_DRIVE_STRENGTH},
	{"edgeq,output",	PIN_CONFIG_OUTPUT},
	{"edgeq,oe",		PIN_CONFIG_OUTPUT_ENABLE},
	{"edgeq,ie",		PIN_CONFIG_INPUT_ENABLE},
};

static struct eq_pinmux_info r2_pinmux = {
	.desc		= &r2_pctrl_desc,
	.apply_mux	= r2_mux_apply,
	.params		= r2_cfg_params,
	.num_params	= ARRAY_SIZE(r2_cfg_params),
};

/* gpio_chip related code */
static void r2_socgpio_set(struct gpio_chip *chip, unsigned int pin, int val)
{
	struct eq_pinmux_info *info = dev_get_drvdata(chip->parent);

	if (val) {
		if (!(pin % 2))
			r2_gpio_setbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
		else
			r2_gpio_setbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
					PINCTRL_OUT_0);
	} else {
		if (!(pin % 2))
			r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
		else
			r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
					PINCTRL_OUT_0);
	}
}

static int r2_socgpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct eq_pinmux_info *info = dev_get_drvdata(chip->parent);

	if (!(pin % 2)) {
		if (!!r2_gpio_getbit(info->membase, EVEN_PIN(pin), PINCTRL_OE_0))
			/* Read gpio value in output mode
			 * NOTE: This is just reg value
			 */
			return !!r2_gpio_getbit(info->membase, EVEN_PIN(pin),
					PINCTRL_OUT_0);
		else {
			/* Read gpio value in input mode */
			if (pin < 32)
				return !!r2_gpio_getbit(info->membase,
						(INPUT_REGISTER_OFFSET), pin);
			else
				return !!r2_gpio_getbit(info->membase,
						(INPUT_REGISTER_OFFSET + 4), (pin - 32));
		}
	} else {
		if (!!r2_gpio_getbit(info->membase, ODD_PIN(pin), (PINCTRL_OE_0 +
				    NEXT_GPIO_OFFSET)))
			/* Read gpio value in output mode
			 * NOTE: This is just reg value
			 */
			return !!r2_gpio_getbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
				PINCTRL_OUT_0);
		else {
			/* Read gpio value in input mode */
			if (pin < 32)
				return !!r2_gpio_getbit(info->membase, INPUT_REGISTER_OFFSET,
						pin);
			else
				return !!r2_gpio_getbit(info->membase, (INPUT_REGISTER_OFFSET + 4),
						(pin - 32));
		}
	}
}

static int r2_socgpio_dir_in(struct gpio_chip *chip, unsigned int pin)
{
	struct eq_pinmux_info *info = dev_get_drvdata(chip->parent);

	if (!(pin % 2)) {
		r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OE_0);
		r2_gpio_setbit(info->membase, EVEN_PIN(pin), PINCTRL_IE_0);
	} else {
		r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET + PINCTRL_OE_0);
		r2_gpio_setbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET + PINCTRL_IE_0);
	}

	return 0;
}

static int r2_socgpio_dir_out(struct gpio_chip *chip, unsigned int pin, int val)
{
	struct eq_pinmux_info *info = dev_get_drvdata(chip->parent);

	if (!(pin % 2)) {
		r2_gpio_rmw(info->membase, EVEN_PIN(pin), PINCTRL_DS_MASK, CURRENT_GPIO_OFFSET,
				PINCTRL_DS_MASK, PINCTRL_DS_OFFSET);
		r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_IE_0);
		r2_gpio_setbit(info->membase, EVEN_PIN(pin), PINCTRL_OE_0);
	} else {
		r2_gpio_rmw(info->membase, ODD_PIN(pin), PINCTRL_DS_MASK, NEXT_GPIO_OFFSET,
				PINCTRL_DS_MASK, PINCTRL_DS_OFFSET);
		r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET + PINCTRL_IE_0);
		r2_gpio_setbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET + PINCTRL_OE_0);
	}

	if (val) {
		if (!(pin % 2))
			r2_gpio_setbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
		else
			r2_gpio_setbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
					PINCTRL_OUT_0);
	} else {
		if (!(pin % 2))
			r2_gpio_clearbit(info->membase, EVEN_PIN(pin), PINCTRL_OUT_0);
		else
			r2_gpio_clearbit(info->membase, ODD_PIN(pin), NEXT_GPIO_OFFSET +
					PINCTRL_OUT_0);
	}

	return 0;
}

/*
 * gpiolib gpiod_to_irq callback function.
 * Returns the mapped IRQ (external interrupt) number for a given GPIO pin.
 */
static int r2_socgpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	int irq;
	int state;
	struct eq_pinmux_info *info = dev_get_drvdata(chip->parent);

	if (offset >= 64)
		return -ENXIO;

	if (!(offset % 2))
		state = r2_gpio_getmux(info->membase, EVEN_PIN(offset), EVEN_MUX);
	else
		state = r2_gpio_getmux(info->membase, ODD_PIN(offset), ODD_MUX);

	if (state != SIF_IN) {
		pr_err("%s:Pin is not set as an interrupt pin\n", __func__);
		return -EINVAL;
	}

	irq = platform_get_irq_byname(rapto2_soc_pinctrl.p_dev, gpio_irq[offset].irq_name);
	if (!irq && (irq != (gpio_irq[offset].sif_in + GIC_SPI_OFFSET))) {
		pr_err("%s:Failed to map hardware IRQ %d to virtual IRQ\n", __func__, irq);
		return -EINVAL;
	}

	return irq;
}

static struct gpio_chip r2_socgpio_chip = {
	.label = "r2-socgpio-pinctrl",
	.direction_input = r2_socgpio_dir_in,
	.direction_output = r2_socgpio_dir_out,
	.get = r2_socgpio_get,
	.set = r2_socgpio_set,
	.request = gpiochip_generic_request,
	.free = gpiochip_generic_free,
	.to_irq = r2_socgpio_to_irq,
	.base = -1,
};

static const struct of_device_id raptor2_pinctrl_of_match[] = {
	{ .compatible = "r2-socgpio-pinctrl", .data = &rapto2_soc_pinctrl},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, raptor2_pinctrl_of_match);

static int raptor2_pinctrl_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct r2_soc_pinctrl *r2_soc;
	int ret, i;

	/* get and remap our register range */
	r2_pinmux.membase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(r2_pinmux.membase))
		return PTR_ERR(r2_pinmux.membase);

	match = of_match_device(raptor2_pinctrl_of_match, &pdev->dev);
	if (match)
		r2_soc = (const struct r2_soc_pinctrl *) match->data;
	else {
		r2_soc = &rapto2_soc_pinctrl;
		dev_err(&pdev->dev, "Could not find matching pinctrl device\r\n");
		dev_err(&pdev->dev, "Setting up the default configuration\r\n");
	}

	/* find out how many pads we have */
	r2_socgpio_chip.ngpio = r2_soc->pin_count;

	/* load our pad descriptors */
	r2_pinmux.pads = devm_kcalloc(&pdev->dev, r2_socgpio_chip.ngpio,
			sizeof(struct pinctrl_pin_desc),
			GFP_KERNEL);
	if (!r2_pinmux.pads)
		return -ENOMEM;

	for (i = 0; i < r2_socgpio_chip.ngpio; i++) {
		char *name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "io%d", i);

		if (!name)
			return -ENOMEM;

		r2_pinmux.pads[i].number = GPIO0 + i;
		r2_pinmux.pads[i].name = name;
	}
	r2_pctrl_desc.pins = r2_pinmux.pads;

	/* setup the data needed by pinctrl */
	r2_pctrl_desc.name	= dev_name(&pdev->dev);
	r2_pctrl_desc.npins	= r2_socgpio_chip.ngpio;

	r2_pinmux.num_pads	= r2_socgpio_chip.ngpio;
	r2_pinmux.num_mfp	= r2_socgpio_chip.ngpio;
	r2_pinmux.mfp		= r2_soc->mfp;
	r2_pinmux.grps		= r2_soc->grps;
	r2_pinmux.num_grps	= r2_soc->num_grps;
	r2_pinmux.funcs		= r2_soc->funcs;
	r2_pinmux.num_funcs	= r2_soc->num_funcs;
	r2_pinmux.exin		= r2_soc->exin;
	r2_pinmux.num_exin	= r2_soc->num_exin;

	/* register with the generic EdgeQ layer */
	ret = eq_pinctrl_register(pdev, &r2_pinmux);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register pinctrl driver\n");
		return ret;
	}

	/* register the gpio chip */
	r2_socgpio_chip.parent = &pdev->dev;
	r2_socgpio_chip.owner = THIS_MODULE;
	r2_socgpio_chip.of_node = pdev->dev.of_node;
	ret = devm_gpiochip_add_data(&pdev->dev, &r2_socgpio_chip, NULL);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register gpio chip\n");
		return ret;
	}

	rapto2_soc_pinctrl.p_dev = pdev;
	if (!rapto2_soc_pinctrl.p_dev)
		return -ENOMEM;

	dev_info(&pdev->dev, "Raptor2 pinctrl driver probed\n");
	return 0;
}

static struct platform_driver raptor2_pinctrl_driver = {
	.driver = {
		.name = "raptor2_pinctrl",
		.of_match_table = raptor2_pinctrl_of_match,
	},
	.probe = raptor2_pinctrl_probe,
};
module_platform_driver(raptor2_pinctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nilesh Waghmare");
MODULE_DESCRIPTION("Raptor2 Pin Control Driver");
