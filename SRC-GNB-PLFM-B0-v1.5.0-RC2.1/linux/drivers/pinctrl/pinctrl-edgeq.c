// SPDX-License-Identifier: GPL-2.0-only
/*
 *  EdgeQ Pin Control Driver
 *  Copyright (C) 2024 EdgeQ Inc.
 *  Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf-generic.h>

#include "pinctrl-edgeq.h"

static int eq_get_group_count(struct pinctrl_dev *pctrldev)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	dev_dbg(pctrldev->dev, "%d info->num_grps %d \r\n", __LINE__, info->num_grps);

	return info->num_grps;
}

static const char *eq_get_group_name(struct pinctrl_dev *pctrldev,
		unsigned int selector)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	dev_dbg(pctrldev->dev, "%d selector %d grp name %s \r\n",
			__LINE__, selector, info->grps[selector].name);
	if (selector >= info->num_grps)
		return NULL;

	return info->grps[selector].name;
}

/* Here selector is referred to index of a group */
static int eq_get_group_pins(struct pinctrl_dev *pctrldev,
		unsigned int selector,
		const unsigned int **pins,
		unsigned int *num_pins)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	if (selector >= info->num_grps)
		return -EINVAL;
	*pins = info->grps[selector].pins;
	*num_pins = info->grps[selector].npins;

	dev_dbg(pctrldev->dev, "%d pins %d num_pins %d selector %d \r\n",
			__LINE__, **pins, *num_pins, selector);

	return 0;
}

static void eq_pinctrl_dt_free_map(struct pinctrl_dev *pctrldev,
				    struct pinctrl_map *map, unsigned int num_maps)
{
	int i;

	for (i = 0; i < num_maps; i++)
		if (map[i].type == PIN_MAP_TYPE_CONFIGS_PIN ||
		    map[i].type == PIN_MAP_TYPE_CONFIGS_GROUP)
			kfree(map[i].data.configs.configs);
	kfree(map);
}

static void eq_pinctrl_pin_dbg_show(struct pinctrl_dev *pctrldev,
					struct seq_file *s,
					unsigned int offset)
{
	seq_printf(s, " %s", dev_name(pctrldev->dev));
}

/* Parse subnodes of child nodes and retrieve information */
static void eq_pinctrl_dt_subnode_to_map(struct pinctrl_dev *pctrldev,
				struct device_node *np,
				struct pinctrl_map **map)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	struct property *pins = of_find_property(np, "edgeq,pins", NULL);
	struct property *groups = of_find_property(np, "edgeq,groups", NULL);
	unsigned long configs[3];
	unsigned int num_configs = 0;
	struct property *prop;
	const char *group, *pin;
	const char *function;
	int ret, i;

	if (!pins && !groups) {
		dev_err(pctrldev->dev, "%pOFn defines neither pins nor groups\n",
				np);
		return;
	}

	if (pins && groups) {
		dev_err(pctrldev->dev, "%pOFn defines both pins and groups\n",
				np);
		return;
	}

	ret = of_property_read_string(np, "edgeq,function", &function);
	if (groups && !ret) {
		of_property_for_each_string(np, "edgeq,groups", prop, group) {
			(*map)->type = PIN_MAP_TYPE_MUX_GROUP;
			(*map)->name = function;
			(*map)->data.mux.group = group;
			(*map)->data.mux.function = function;
			(*map)++;
		}
	}

	for (i = 0; i < info->num_params; i++) {
		u32 val;
		int ret = of_property_read_u32(np,
				info->params[i].property, &val);
		if (!ret)
			configs[num_configs++] =
				pinconf_to_config_packed(info->params[i].param,
						val);
	}

	if (!num_configs)
		return;

	of_property_for_each_string(np, "edgeq,pins", prop, pin) {
		(*map)->data.configs.configs = kmemdup(configs,
				num_configs * sizeof(unsigned long),
				GFP_KERNEL);
		(*map)->type = PIN_MAP_TYPE_CONFIGS_PIN;
		(*map)->name = pin;
		(*map)->data.configs.group_or_pin = pin;
		(*map)->data.configs.num_configs = num_configs;
		(*map)++;
	}

	of_property_for_each_string(np, "edgeq,groups", prop, group) {
		(*map)->data.configs.configs = kmemdup(configs,
				num_configs * sizeof(unsigned long),
				GFP_KERNEL);
		(*map)->type = PIN_MAP_TYPE_CONFIGS_GROUP;
		(*map)->name = group;
		(*map)->data.configs.group_or_pin = group;
		(*map)->data.configs.num_configs = num_configs;
		(*map)++;
	}
}

static int eq_pinctrl_dt_subnode_size(struct device_node *np)
{
	int ret;

	ret = of_property_count_strings(np, "edgeq,groups");
	if (ret < 0)
		ret = of_property_count_strings(np, "edgeq,pins");

	return ret;
}

/* Parse child nodes and retrieve information */
static int eq_pinctrl_dt_node_to_map(struct pinctrl_dev *pctrldev,
		struct device_node *np_config,
		struct pinctrl_map **map,
		unsigned int *num_maps)
{
	struct pinctrl_map *tmp;
	struct device_node *np;
	int max_maps = 0;

	for_each_child_of_node(np_config, np)
		max_maps += eq_pinctrl_dt_subnode_size(np);
	*map = kzalloc(array3_size(max_maps, sizeof(struct pinctrl_map), 2),
			GFP_KERNEL);
	if (!*map)
		return -ENOMEM;
	tmp = *map;

	for_each_child_of_node(np_config, np)
		eq_pinctrl_dt_subnode_to_map(pctrldev, np, &tmp);
	*num_maps = ((int)(tmp - *map));

	dev_dbg(pctrldev->dev, "%d num_maps %d max_maps %d \r\n", __LINE__, *num_maps, max_maps);

	return 0;
}

/* These callback gets called when function: devm_pinctrl_get(dev)
 * is called to get the pinctrl handle from the device
 */
static const struct pinctrl_ops eq_pctrl_ops = {
	.get_groups_count	= eq_get_group_count,
	.get_group_name		= eq_get_group_name,
	.get_group_pins		= eq_get_group_pins,
	.pin_dbg_show		= eq_pinctrl_pin_dbg_show,
	.dt_node_to_map		= eq_pinctrl_dt_node_to_map,
	.dt_free_map		= eq_pinctrl_dt_free_map,
};

static int eq_pmx_func_count(struct pinctrl_dev *pctrldev)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	dev_dbg(pctrldev->dev, "%d info->num_funcs %d \r\n", __LINE__, info->num_funcs);

	return info->num_funcs;
}

static const char *eq_pmx_func_name(struct pinctrl_dev *pctrldev,
		unsigned int selector)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	dev_dbg(pctrldev->dev, "%d selector %d function name %s \r\n",
			__LINE__, selector, info->funcs[selector].name);

	if (selector >= info->num_funcs)
		return NULL;

	return info->funcs[selector].name;
}

static int eq_pmx_get_groups(struct pinctrl_dev *pctrldev,
		unsigned int func,
		const char * const **groups,
		unsigned * const num_groups)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);

	*groups = info->funcs[func].groups;
	*num_groups = info->funcs[func].num_groups;

	dev_dbg(pctrldev->dev, "%d num_groups %d func %d \r\n", __LINE__, *num_groups, func);

	return 0;
}

/*
 * Return function number(index) of one of the mfp from array of struct eq_mfp_pin.
 * If failure, return negative value.
 */
static int match_mux(const struct eq_mfp_pin *mfp, unsigned int mux)
{
	int i;

	for (i = 0; i < EQ_MAX_MUX; i++) {
		if (mfp->func[i] == mux)
			break;
	}

	if (i >= EQ_MAX_MUX)
		return -EINVAL;

	return i;
}

/* As .mfp is not linearly mapped. find the mfp with the correct .pin */
static int match_mfp(const struct eq_pinmux_info *info, int pin)
{
	int i;

	for (i = 0; i < info->num_mfp; i++) {
		/* find index of mfp from array of eq_mfp_pin structure */
		if (info->mfp[i].pin == pin)
			return i;
	}
	return -1;
}

static int eq_pmx_set(struct pinctrl_dev *pctrldev,
		unsigned int func,
		unsigned int group)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	/*One group from array of eq_pin_group struct which has bunch of GRP_MUX */
	const struct eq_pin_group *pin_grp = &info->grps[group];
	int i, pin_index, pin_func_index, ret;

	dev_dbg(pctrldev->dev, "%d group %d func %d \r\n", __LINE__, group, func);

	for (i = 0; i < pin_grp->npins; i++) {
		/* find index of mfp from array of eq_mfp_pin structure */
		pin_index = match_mfp(info, pin_grp->pins[i]);
		if (pin_index < 0) {
			dev_err(info->dev, "could not find mfp for pin %d\n",
					pin_grp->pins[i]);
			return -EINVAL;
		}
		/* find index of function of one of the mfp from array of eq_mfp_pin structure */
		pin_func_index = match_mux(&info->mfp[pin_index], pin_grp->mux);
		ret = info->apply_mux(pctrldev, pin_func_index, info->mfp[pin_index].pin);
		if (ret) {
			dev_err(info->dev,
					"failed to apply mux %d for pin_index %d\n",
					pin_func_index, pin_index);
			return ret;
		}
	}
	return 0;
}

static int eq_pmx_gpio_request_enable(struct pinctrl_dev *pctrldev,
		struct pinctrl_gpio_range *range,
		unsigned int pin)
{
	struct eq_pinmux_info *info = pinctrl_dev_get_drvdata(pctrldev);
	int mfp = match_mfp(info, pin);
	int pin_func;

	if (mfp < 0) {
		dev_err(info->dev, "could not find mfp for pin %d\n", pin);
		return -EINVAL;
	}

	pin_func = match_mux(&info->mfp[mfp], 0);
	if (pin_func < 0) {
		dev_err(info->dev, "No GPIO function on pin%d\n", mfp);
		return -EINVAL;
	}

	return info->apply_mux(pctrldev, pin_func, info->mfp[mfp].pin);
}

static const struct pinmux_ops eq_pmx_ops = {
	.get_functions_count	= eq_pmx_func_count,
	.get_function_name	= eq_pmx_func_name,
	.get_function_groups	= eq_pmx_get_groups,
	.set_mux		= eq_pmx_set,
	.gpio_request_enable	= eq_pmx_gpio_request_enable,
};

/*
 * Allow different SoCs to register with the generic part of the edgeq
 * pinctrl code
 */
int eq_pinctrl_register(struct platform_device *pdev,
		struct eq_pinmux_info *info)
{
	struct pinctrl_desc *desc;

	if (!info)
		return -EINVAL;
	desc = info->desc;
	desc->pctlops = &eq_pctrl_ops;
	desc->pmxops = &eq_pmx_ops;
	info->dev = &pdev->dev;

	info->pctrl = devm_pinctrl_register(&pdev->dev, desc, info);
	if (IS_ERR(info->pctrl)) {
		dev_err(&pdev->dev, "failed to register EdgeQ pinmux driver\n");
		return PTR_ERR(info->pctrl);
	}
	platform_set_drvdata(pdev, info);
	return 0;
}
