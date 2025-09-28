/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#ifndef _EEPROM_LAYOUT_VERSIONS_H_
#define _EEPROM_LAYOUT_VERSIONS_H_

#define LAYOUT_VERSION_VER1 1

#define DEFINE_PRINT_UPDATE_BIN_REV(x)	eeprom_field_print_bin_rev, x##_fields_update_bin_rev
#define DEFINE_PRINT_UPDATE_ASCII(x)	eeprom_field_print_ascii, x##_fields_update_ascii
#define DEFINE_PRINT_UPDATE_MAC(x)		eeprom_field_print_mac, x##_fields_update_mac
#define DEFINE_PRINT_UPDATE_CRC		eeprom_field_print_bin_rev, eeprom_field_update_dummy
#define DEFINE_PRINT_UPDATE_RESERVED	eeprom_field_print_reserved, eeprom_field_update_reserved

int eeprom_info_fields_update_bin_rev(struct eeprom_field *field, char *value);
int board_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int pcie_usb_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int rfic_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int ethernet_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int ethernet_configs_fields_update_mac(struct eeprom_field *field, char *value);
int misc_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int jesd_configs_fields_update_bin_rev(struct eeprom_field *field, char *value);
int eeprom_field_update_dummy(struct eeprom_field *field, char *value);
int board_configs_fields_update_ascii(struct eeprom_field *field, char *value);

struct eeprom_field layout_v1[] = {
{"eeprom_info.crc32",                       4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"eeprom_info.signature",                   4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(eeprom_info)},
{"eeprom_info.size",                        4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(eeprom_info)},
{"eeprom_info.version",                     2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(eeprom_info)},
{RESERVED_FIELDS,                           18, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"board_configs.crc32",                     4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"board_configs.board_serial_number",       16, NULL, DEFINE_PRINT_UPDATE_ASCII(board_configs)},
{"board_configs.board_target_usecase",      4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.board_version",             4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.soc_chip_id",               8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.soc_chip_silicon_corner",   4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.soc_package_marking_lot",   4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.boot_mode_setting",         4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.device_uid0",               8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.device_uid1",               8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.device_uid2",               8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.device_uid3",               8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.product_config",            64, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"board_configs.ddr_config",                4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.link1_status",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.link2_status",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.link3_status",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{"board_configs.link4_status",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(board_configs)},
{RESERVED_FIELDS,                           104, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"pcie_usb_configs.crc32",                  4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"pcie_usb_configs.active_pcie_interfaces", 2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(pcie_usb_configs)},
{"pcie_usb_configs.pcie_id",                8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(pcie_usb_configs)},
{"pcie_usb_configs.usb_id",                 8, NULL, DEFINE_PRINT_UPDATE_BIN_REV(pcie_usb_configs)},
{RESERVED_FIELDS,                           42, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"rfic_configs.crc32",                      4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"rfic_configs.status",                     4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{"rfic_configs.serial",                     4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{"rfic_configs.version",                    4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{"rfic_configs.assembly_option",            4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{"rfic_configs.rf1_card_type",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{"rfic_configs.rf2_card_type",              1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(rfic_configs)},
{RESERVED_FIELDS,                           10, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"ethernet_configs.crc32",                  4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"ethernet_configs.max_interfaces",         2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{"ethernet_configs.active_interfaces",      2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{"ethernet_configs.ethlink",                2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{"ethernet_configs.ethact",                 2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{"ethernet_configs.ethspeeds",              6, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{"ethernet_configs.mac0",                   6, NULL, DEFINE_PRINT_UPDATE_MAC(ethernet_configs)},
{"ethernet_configs.mac1",                   6, NULL, DEFINE_PRINT_UPDATE_MAC(ethernet_configs)},
{"ethernet_configs.mac2",                   6, NULL, DEFINE_PRINT_UPDATE_MAC(ethernet_configs)},
{"ethernet_configs.mac3",                   6, NULL, DEFINE_PRINT_UPDATE_MAC(ethernet_configs)},
{"ethernet_configs.mac4",                   6, NULL, DEFINE_PRINT_UPDATE_MAC(ethernet_configs)},
{"ethernet_configs.ecpri_bitmap",           1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(ethernet_configs)},
{RESERVED_FIELDS,                           79, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"misc_configs.crc32",                      4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"misc_configs.active_misc_interfaces",     4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{"misc_configs.pmic_configuration_version", 4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{"misc_configs.pcb_assembly_option",        4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{"misc_configs.pcb_rework_serivce_code",    4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{"misc_configs.dctxo_initial_value",        4, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{"misc_configs.pvt_mon_cal",                16, NULL, DEFINE_PRINT_UPDATE_BIN_REV(misc_configs)},
{RESERVED_FIELDS,                           88, NULL, DEFINE_PRINT_UPDATE_RESERVED},
{"jesd_configs.crc32",                      4, NULL, DEFINE_PRINT_UPDATE_CRC},
{"jesd_configs.link3_lane_select",          2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(jesd_configs)},
{"jesd_configs.link4_lane_select",          2, NULL, DEFINE_PRINT_UPDATE_BIN_REV(jesd_configs)},
{"jesd_configs.link3_lane_polarity",        1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(jesd_configs)},
{"jesd_configs.link4_lane_polarity",        1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(jesd_configs)},
{"jesd_configs.link_speed",                 1, NULL, DEFINE_PRINT_UPDATE_BIN_REV(jesd_configs)},
{RESERVED_FIELDS,                           373, NULL, DEFINE_PRINT_UPDATE_RESERVED},
};

#endif /* _EEPROM_LAYOUT_VERSIONS_H_*/
