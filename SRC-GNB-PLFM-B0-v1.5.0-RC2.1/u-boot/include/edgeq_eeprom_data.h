/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2023 EdgeQ
 *
 */

#ifndef _EDGEQ_EEPROM_DATA_H_
#define _EDGEQ_EEPROM_DATA_H_

struct eeprom_info {
	uint32_t crc32;
	uint32_t signature;
	uint32_t size;
	uint16_t version;
	uint8_t reserved[18];
} __packed; /*struct eeprom_info*/

struct board_configs {
	uint32_t crc32;
	uint8_t board_serial_number[16];
	uint32_t board_target_usecase;
	uint32_t board_version;
	uint64_t soc_chip_id;
	uint32_t soc_chip_silicon_corner;
	uint32_t soc_package_marking_lot;
	uint32_t boot_mode_setting;
	uint64_t device_uid0;
	uint64_t device_uid1;
	uint64_t device_uid2;
	uint64_t device_uid3;
	uint8_t product_config[64];
	uint32_t ddr_config;
	uint8_t link1_status;
	uint8_t link2_status;
	uint8_t link3_status;
	uint8_t link4_status;
	uint8_t reserved[104];
} __packed; /*struct board_configs*/

/* EEPROM main structure */
struct eeprom_data {
	struct eeprom_info eeprom_info;
	struct board_configs board_configs;
} __packed; /*struct eeprom_data */

#endif /* _EDGEQ_EEPROM_DATA_H_*/

