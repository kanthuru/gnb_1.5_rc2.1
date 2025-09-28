// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EdgeQ EEPROM sysfs driver
 *
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Akhilesh Kadam <c_sanjaykadam@edgeq.io>
 * Author: Abdul Ahad <abdul.ahad@edgeq.io>
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
#include <linux/of_address.h>
#include "eeprom_data.h"
#include "eeprom_field.h"

struct eeprom {
	void *vaddr;
	char pu[50];
	int number_of_fields;
};

extern struct kobject *kobj_raptor2;
struct eeprom eeprom_t;

static int eeprom_output_format(struct eeprom_field *field, bool reverse, int index)
{
	int i;

	int from = reverse ? field[index].size - 1 : 0;
	int to = reverse ? 0 : field[index].size - 1;

	for (i = from; i != to; reverse ? i-- : i++) {
		if ((unsigned char)field[index].buf[i] > 0x0)
			break;
	}
	return i;
}

static ssize_t eeprom_field_print(struct eeprom_field *field, char *chara,
		char *delimeter, bool reverse, int index, int start_index)
{
	int i;
	int j = 0;
	size_t length;
	int to, from;

	if (field[index].format) {
		length = strlen(field[index].buf);
		if (length > field[index].size)
			length = field[index].size;
		to = reverse ? 0 : length - 1;
	} else
		to = reverse ? 0 : field[index].size - 1;

	from = start_index;
	j += sprintf(chara, "%s: ", field[index].name);
	for (i = from; i != to; reverse ? i-- : i++)
		j += sprintf(chara+j, field[index].format ? "%c%s" : "%02x%s",
				field[index].buf[i], delimeter);

	j += sprintf(chara+j, field[index].format ? "%c\n" : "%02x\n",
			field[index].buf[i]);

	return strlen(chara);
}

static ssize_t eeprom_show(struct kobject *kobj_raptor2, struct kobj_attribute *attr,
		char *buf)
{
	int i, j = 0;
	int ret = 0;
	int ret1 = 0;

	for (i = 0; i < eeprom_t.number_of_fields; i++) {
		if ((strcmp(eeprom_t.pu, field[i].name) == 0) ||
				(strcmp(eeprom_t.pu, "all") == 0)) {
			if ((strcmp(field[i].name, "RESERVED_FIELDS") == 0))
				continue;
			buf = buf+ret;

			if ((strncmp(field[i].name, "ethernet_configs.mac", 20) == 0)) {
				j = eeprom_output_format(field, false, i);
				ret = eeprom_field_print(field, buf, ":", false, i, j);
			} else if ((strncmp(field[i].name,
						"board_configs.board_serial_number", 33) == 0)) {
				j = eeprom_output_format(field, false, i);
				ret = eeprom_field_print(field, buf, "", false, i, j);
			} else {
				j = eeprom_output_format(field, true, i);
				ret = eeprom_field_print(field, buf, "", true, i, j);
			}
			ret1 += ret;

		} else {
			if (!ret1)
				ret1 = sprintf(buf, "%s\n", "incorrect field");
		}
	}
	buf = (char *)eeprom_t.vaddr;

	return ret1;
}


ssize_t eeprom_store(struct kobject *kobj_raptor2, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int i;

	memset(eeprom_t.pu, '\0', strlen(eeprom_t.pu));
	i = sscanf(buf, "%s", eeprom_t.pu);

	return count;
}

static struct kobj_attribute eeprom_attribute = __ATTR(eeprom, 0660, eeprom_show,
		eeprom_store);

static int __init raptor2_eeprom_init(void)
{
	u64  paddr;
	int i, rc, size = 0;
	struct device_node *np;
	struct device_node *mem;
	struct resource mem_res;
	int ret = 0;

	eeprom_t.number_of_fields = sizeof(field)/sizeof(field[1]);
	np = of_find_node_by_name(NULL, "eeprom");
	if (!np) {
		pr_err("Device not found in the device tree\n");
		return -ENODEV;
	}

	mem = of_parse_phandle(np, "memory-region", 0);
	if (!mem) {
		pr_err("Failed to get phandle from np\n");
		return -EINVAL;
	}

	rc = of_address_to_resource(mem, 0, &mem_res);
	if (rc) {
		pr_err("Failed to get address from np\n");
		return -EINVAL;
	}

	paddr = mem_res.start;
	eeprom_t.vaddr = ioremap_cache(paddr, resource_size(&mem_res));

	for (i = 0; i < eeprom_t.number_of_fields; i++) {
		field[i].buf = (unsigned char *)(eeprom_t.vaddr+size);
		size += field[i].size;
	}

	if (!kobj_raptor2) {
		pr_err("kobj_raptor2 is NULL\n");
		return false;
	}

	ret = sysfs_create_file(kobj_raptor2, &eeprom_attribute.attr);
	if (ret < 0) {
		pr_err("failed to create the eeprom file in /sys/raptor2\n");
		return ret;
	}

	pr_info("EEPROM sysfs module Init successful\n");

	return ret;
}

module_init(raptor2_eeprom_init);

MODULE_AUTHOR("Akhilesh Kadam <c_sanjaykadam@edgeq.io>");
MODULE_AUTHOR("Abdul Ahad <abdul.ahad@edgeq.io>");
MODULE_DESCRIPTION("EdgeQ EEPROM sysfs driver");
MODULE_LICENSE("GPL");
