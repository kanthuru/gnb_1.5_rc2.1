/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  EdgeQ Pin Control Driver
 *  Copyright (C) 2024 EdgeQ Inc.
 *  Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

#ifndef __PINCTRL_EDGEQ_H
#define __PINCTRL_EDGEQ_H

#include <linux/clkdev.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>

#include "core.h"

#define EQ_MAX_MUX		7

struct eq_cfg_param {
	const char *property;
	enum pin_config_param param;
};

struct eq_mfp_pin {
	const char *name;
	const unsigned int pin;
	const unsigned short func[EQ_MAX_MUX];
};

struct eq_pin_group {
	const char *name;
	const unsigned int mux;
	const unsigned int *pins;
	const unsigned int npins;
};

struct eq_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned int num_groups;
};

struct eq_pinmux_info {
	struct device *dev;
	struct pinctrl_dev *pctrl;

	void __iomem *membase;

	/* The descriptor for the subsystem */
	struct pinctrl_desc *desc;

	/* Expose pads to the subsystem */
	struct pinctrl_pin_desc *pads;

	/* The number of pads. this varies between socs */
	unsigned int num_pads;

	/* These are multifunction pins */
	const struct eq_mfp_pin *mfp;
	unsigned int num_mfp;

	/* A number of multifunction pins can be grouped together */
	const struct eq_pin_group *grps;
	unsigned int num_grps;

	/* A mapping between function string and id */
	const struct eq_pmx_func *funcs;
	unsigned int num_funcs;

	/* The pinconf options that we are able to read from the DT */
	const struct eq_cfg_param *params;
	unsigned int num_params;

	/* The pad controller can have a irq mapping  */
	const unsigned int *exin;
	unsigned int num_exin;

	/* Soc specific callback used to apply muxing */
	int (*apply_mux)(struct pinctrl_dev *pctrldev, int mux_num, int pin_num);
};

enum eq_pin {
	GPIO0 = 0,
	GPIO1,
	GPIO2,
	GPIO3,
	GPIO4,
	GPIO5,
	GPIO6,
	GPIO7,
	GPIO8,
	GPIO9,
	GPIO10, /* 10 */
	GPIO11,
	GPIO12,
	GPIO13,
	GPIO14,
	GPIO15,
	GPIO16,
	GPIO17,
	GPIO18,
	GPIO19,
	GPIO20, /* 20 */
	GPIO21,
	GPIO22,
	GPIO23,
	GPIO24,
	GPIO25,
	GPIO26,
	GPIO27,
	GPIO28,
	GPIO29,
	GPIO30, /* 30 */
	GPIO31,
	GPIO32,
	GPIO33,
	GPIO34,
	GPIO35,
	GPIO36,
	GPIO37,
	GPIO38,
	GPIO39,
	GPIO40, /* 40 */
	GPIO41,
	GPIO42,
	GPIO43,
	GPIO44,
	GPIO45,
	GPIO46,
	GPIO47,
	GPIO48,
	GPIO49,
	GPIO50, /* 50 */
	GPIO51,
	GPIO52,
	GPIO53,
	GPIO54,
	GPIO55,
	GPIO56,
	GPIO57,
	GPIO58,
	GPIO59,
	GPIO60, /* 60 */
	GPIO61,
	GPIO62,
	GPIO63,
};

int eq_pinctrl_register(struct platform_device *pdev,
		struct eq_pinmux_info *info);
int eq_pinctrl_unregister(struct platform_device *pdev);
#endif	/* __PINCTRL_EDGEQ_H */
