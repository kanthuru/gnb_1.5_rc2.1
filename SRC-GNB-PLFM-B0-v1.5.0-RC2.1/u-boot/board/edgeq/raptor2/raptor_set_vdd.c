/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
#include <malloc.h>
#include <linux/ctype.h>
#include <dm.h>
#include "raptor_set_vdd.h"

int set_buck2_vdd(int vdd)
{
	int raw_vdd;
	struct udevice *bus, *dev;
	u8 reply[1];
	u8 raw;

	if (uclass_get_device(UCLASS_I2C, 0, &bus))
		goto error;

	if (i2c_get_chip(bus, 0x5E, 1, &dev))
		goto error;

	raw_vdd = VDD2RAW(vdd);
	if (raw_vdd < 0)
		raw_vdd = 0;

	/* Bit 0 represents Decay Bit*/
	raw = raw_vdd << 1;

	if (dm_i2c_write(dev, 0x21, &raw, 1))
		goto error;

	if (dm_i2c_read(dev, 0x21, reply, 1))
		goto error;

	if (raw != *reply) {
		printf("I2C write failed. Read and VDD value not matched\n");
		goto error;
	}

	return 0;

error:
	return CMD_RET_FAILURE;
}

static int is_valid_num(const char *str)
{
	int j = 0;

	if (!str)
		return 0;

	while (isdigit(str[j]))
		j++;

	return ((strlen(str) == j) && (j != 0));
}

int parse_vdd_value(const char *str)
{
	int value;
	char temp[5] = {0};

	memset(temp, '0', 4);

	if (strlen(str) > 4 || str[1] != '.')
		goto error;

	temp[0] = str[0];

	memcpy(&temp[1], &str[2], strlen(&str[2]));

	if (!is_valid_num(temp))
		goto error;

	value = simple_strtol(temp, NULL, 10);

	/* VDD ranges from VDD_MIN to VDD_MAX*/
	if (value < VDD_MIN || value > VDD_MAX)
		goto error;

	return value;
error:
	return -1;
}

static int do_set_vdd(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char *str;
	int vdd_val, ret = 0;

	if (argc != 2)
		return CMD_RET_USAGE;

	str = strdup(argv[1]);
	if (!str)
		return -ENOMEM;

	vdd_val = parse_vdd_value(str);
	if (vdd_val == -1) {
		printf("Invalid VDD value: %s\n", str);
		ret =  CMD_RET_USAGE;
		goto error;
	}

	ret = set_buck2_vdd(vdd_val - 20);

	if (ret)
		printf("I2C Error: Set VDD Failed\n");
	else
		printf("Vdd updated with %smV\n", str);

error:
	free(str);
	return ret;
}

U_BOOT_CMD(
	setvdd,    2,    0,     do_set_vdd,
	"set vdd for buck2 of pmic",
	"<value>\n"
	"<value>	- vdd value to set\n"
	"Max 2 digits after decimal are allowed\n"
	"The value should be in between 0.8mV to 0.9mV\n"
);

