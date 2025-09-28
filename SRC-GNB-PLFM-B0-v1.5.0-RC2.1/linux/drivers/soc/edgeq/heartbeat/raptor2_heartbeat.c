// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EdgeQ HEARTBEAT sysfs driver
 *
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Adhikari Venkata Yuktha <adhikari.yuktha@edgeq.io>
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <soc/edgeq/raptor2_bss_api.h>

extern struct kobject *kobj_raptor2;
static int flag = -1;
static int Config = -1;
#define LINUX 3

static const struct of_device_id raptor2_heartbeat_of_match[] = {
	{
		.compatible = "edgeq,raptor2-heartbeat",
	},
	{ /* sentinel */ }
};

static ssize_t heartbeat_show(struct kobject *kobj_raptor2, struct kobj_attribute *attr,
		char *buf)
{
	int ret;

	if (flag == 0)
		ret = sprintf(buf, "%s\n", "Heartbeat is disabled");
	else if (flag == 1)
		ret = sprintf(buf, "%s\n", "Heartbeat is enabled");
	else if ((flag == 2) && ((Config < 16) || (Config > 0)))
		ret = sprintf(buf, "%s%d%s\n", "Heartbeat is configured for ", Config, " seconds");
	else
		ret = sprintf(buf, "%s\n", "Heartbeat message is not yet sent");
	return ret;
}

ssize_t heartbeat_store(struct kobject *kobj_raptor2, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val, config;
	uint16_t msg_info;
	uint8_t state, msg_type;

	val = 0;
	config = 0;

	ret = sscanf(buf, "%ld %ld", &val, &config);
	if ((ret != 1) && (ret != 2)) {
		flag = -1;
		Config = -1;
		pr_err("%s: sscanf failed for %s\n", __func__, buf);
		return count;
	}

	if ((val < 0) || (val > 2) || ((val != 2) && (config))) {
		flag = -1;
		Config = -1;
		pr_debug("Incorrect value sent to heartbeat sysfs driver\n");
		pr_debug("0 - to disable, 1 - to enable, 2 - configure\n");
		return count;
	}

	if ((val == 2) && ((config != 3) && (config != 5) && (config != 10) && (config != 15))) {
		flag = -1;
		Config = -1;
		pr_debug("Incorrect value sent to heartbeat sysfs driver\n");
		pr_debug("Watchdog configuration supported values are: 3, 5, 10, 15\n");
		return count;
	}

	flag = val;
	Config = config;

	ret = bss_cmd_setup(BSSRT_CHAN_EFUSE);
	if (ret < 0) {
		pr_err("%s: bss_cmd_setup failed\n", __func__);
		return count;
	}
  
	msg_type = RAPTOR2_MSGTYPE_WRITE;
	msg_info = 0x0;

	msg_info = LINUX;
	if (val == 1) {
		state = 1;
		msg_info |= (state << 3);
	} else if (val == 2)
		msg_info |= (config << 4);

	ret = send_recv_bss_msg(BSSRT_CHAN_EFUSE,
			RAPTOR2_MSGID_SOC_HEARTBEAT, msg_type, msg_info, 0, 0, 0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed for %s",
				__func__, cmdstr[RAPTOR2_MSGID_SOC_HEARTBEAT]);
		return count;
	}

	pr_debug("%s: Heartbeat message received by BSS\n", __func__);

	return count;
}

static struct kobj_attribute heartbeat_attribute = __ATTR(heartbeat, 0660, heartbeat_show,
		heartbeat_store);

static int raptor2_heartbeat_probe(struct platform_device *pdev)
{
	int ret;

	if (!kobj_raptor2) {
		pr_err("%s: kobj_raptor2 is NULL\n", __func__);
		return -EINVAL;
	}

	ret = sysfs_create_file(kobj_raptor2, &heartbeat_attribute.attr);
	if (ret < 0) {
		pr_err("%s: failed to create the heartbeat file in /sys/raptor2\n", __func__);
		return ret;
	}

	dev_info(&pdev->dev, "Probe for Platform Driver Heartbeat Successful\n");
	return 0;
}

static struct platform_driver raptor2_heartbeat_driver = {
	.driver         = {
		.owner = THIS_MODULE,
		.name = "raptor2-heartbeat",
		.of_match_table = raptor2_heartbeat_of_match,
	},
	.probe = raptor2_heartbeat_probe,
};

builtin_platform_driver(raptor2_heartbeat_driver);

MODULE_AUTHOR("Adhikari Venkata Yuktha <adhikari.yuktha@edgeq.io>");
MODULE_DESCRIPTION("EdgeQ HEARTBEAT sysfs driver");
MODULE_LICENSE("GPL");
