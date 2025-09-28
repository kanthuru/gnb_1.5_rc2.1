/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _EEPROM_LAYOUT_VERSIONS_H_
#define _EEPROM_LAYOUT_VERSIONS_H_

#define LAYOUT_VERSION_VER1 1

struct eeprom_field {
	char *name;
	int size;
	unsigned char *buf;
	int offset;
	int format;

	void (*print)(const struct eeprom_field *eeprom_field);
	int (*update)(struct eeprom_field *eeprom_field, char *value);
};

struct eeprom_field field[] = {
	{"eeprom_info.crc32",                       4, NULL, 0, 0, NULL},
	{"eeprom_info.signature",                   4, NULL, 0, 0, NULL},
	{"eeprom_info.size",                        4, NULL, 0, 0, NULL},
	{"eeprom_info.version",                     2, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                         18, NULL, 0, 0, NULL},
	{"board_configs.crc32",                     4, NULL, 0, 0, NULL},
	{"board_configs.board_serial_number",       16, NULL, 0, 1, NULL},
	{"board_configs.board_target_usecase",      4, NULL, 0, 0, NULL},
	{"board_configs.board_version",             4, NULL, 0, 0, NULL},
	{"board_configs.soc_chip_id",               8, NULL, 0, 0, NULL},
	{"board_configs.soc_chip_silicon_corner",   4, NULL, 0, 0, NULL},
	{"board_configs.soc_package_marking_lot",   4, NULL, 0, 0, NULL},
	{"board_configs.boot_mode_setting",         4, NULL, 0, 0, NULL},
	{"board_configs.device_uid0",               8, NULL, 0, 0, NULL},
	{"board_configs.device_uid1",               8, NULL, 0, 0, NULL},
	{"board_configs.device_uid2",               8, NULL, 0, 0, NULL},
	{"board_configs.device_uid3",               8, NULL, 0, 0, NULL},
	{"board_configs.product_config",            64, NULL, 0, 0, NULL},
	{"board_configs.ddr_config",                4, NULL, 0, 0, NULL},
	{"board_configs.link1_status",                1, NULL, 0, 0, NULL},
	{"board_configs.link2_status",                1, NULL, 0, 0, NULL},
	{"board_configs.link3_status",                1, NULL, 0, 0, NULL},
	{"board_configs.link4_status",                1, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                           104, NULL, 0, 0, NULL},
	{"pcie_usb_configs.crc32",                  4, NULL, 0, 0, NULL},
	{"pcie_usb_configs.active_pcie_interfaces", 2, NULL, 0, 0, NULL},
	{"pcie_usb_configs.pcie_id",                8, NULL, 0, 0, NULL},
	{"pcie_usb_configs.usb_id",                 8, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                           42, NULL, 0, 0, NULL},
	{"rfic_configs.crc32",                      4, NULL, 0, 0, NULL},
	{"rfic_configs.status",                     4, NULL, 0, 0, NULL},
	{"rfic_configs.serial",                     4, NULL, 0, 0, NULL},
	{"rfic_configs.version",                    4, NULL, 0, 0, NULL},
	{"rfic_configs.assembly_option",            4, NULL, 0, 0, NULL},
	{"rfic_configs.rf1_card_type",              1, NULL, 0, 0, NULL},
	{"rfic_configs.rf2_card_type",              1, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                           10, NULL, 0, 0, NULL},
	{"ethernet_configs.crc32",                  4, NULL, 0, 0, NULL},
	{"ethernet_configs.max_interfaces",         2, NULL, 0, 0, NULL},
	{"ethernet_configs.active_interfaces",      2, NULL, 0, 0, NULL},
	{"ethernet_configs.ethlink",                2, NULL, 0, 0, NULL},
	{"ethernet_configs.ethact",                 2, NULL, 0, 0, NULL},
	{"ethernet_configs.ethspeeds",              6, NULL, 0, 0, NULL},
	{"ethernet_configs.mac0",                   6, NULL, 0, 0, NULL},
	{"ethernet_configs.mac1",                   6, NULL, 0, 0, NULL},
	{"ethernet_configs.mac2",                   6, NULL, 0, 0, NULL},
	{"ethernet_configs.mac3",                   6, NULL, 0, 0, NULL},
	{"ethernet_configs.mac4",                   6, NULL, 0, 0, NULL},
	{"ethernet_configs.ecpri_bitmap",           1, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                         79, NULL, 0, 0, NULL},
	{"misc_configs.crc32",                      4, NULL, 0, 0, NULL},
	{"misc_configs.active_misc_interfaces",     4, NULL, 0, 0, NULL},
	{"misc_configs.pmic_configuration_version", 4, NULL, 0, 0, NULL},
	{"misc_configs.pcb_assembly_option",        4, NULL, 0, 0, NULL},
	{"misc_configs.pcb_rework_serivce_code",    4, NULL, 0, 0, NULL},
	{"misc_configs.dctxo_initial_value",        4, NULL, 0, 0, NULL},
	{"misc_configs.pvt_mon_cal",                16, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                         88, NULL, 0, 0, NULL},
	{"jesd_configs.crc32",                      4, NULL, 0, 0, NULL},
	{"jesd_configs.link3_lane_select",          2, NULL, 0, 0, NULL},
	{"jesd_configs.link4_lane_select",          2, NULL, 0, 0, NULL},
	{"jesd_configs.link3_lane_polarity",        1, NULL, 0, 0, NULL},
	{"jesd_configs.link4_lane_polarity",        1, NULL, 0, 0, NULL},
	{"jesd_configs.link_speed",                 1, NULL, 0, 0, NULL},
	{"RESERVED_FIELDS",                         373, NULL, 0, 0, NULL},
};

#endif /* _EEPROM_LAYOUT_VERSIONS_H_*/
