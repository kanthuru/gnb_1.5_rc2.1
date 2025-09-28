/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#ifndef _RAPTOR2_EEPROM_H_
#define _RAPTOR2_EEPROM_H_

#define EEPROM_BUFFER_OFFSET	0xFD0000
#define EEPROM_BUFFER_ADDR	(CONFIG_SYS_SDRAM_BASE + EEPROM_BUFFER_OFFSET)
#define EEPROM_SIGNATURE	0xED0E0FAB
#define EEPROM_CRC_VERIFIED	0x1
#define MAX_ETH_INTERFACES	5
#define MAX_NUM_ETHSPEED	6
#define MAX_ETHLINK		3
#define BYTES2MAC(x, y) (sprintf((x), "%02x:%02x:%02x:%02x:%02x:%02x", \
				(y)[0], (y)[1], (y)[2], (y)[3], (y)[4], (y)[5]))

#define MAX_VENDORS		10
#define MAX_BOARDS		8
#define SOC_CHIP_B0		2
#define CHIP_NAME_MASK		0xf
#define GET_VENDOR_IDX(x)	(((x) >> 16) & 0xffff)
#define GET_BOARD_IDX(x)	(((x) >> 8) & 0xff)

enum e_board_names {
	BOARD_TITAN = 0,
	BOARD_HAWK,
	BOARD_PEGASUS,
	BOARD_HAWKV2,
	BOARD_HAWKV3,
	BOARD_HAWKV4,
	BOARD_HAWKV5,
	BOARD_UNKNOWN
};

enum e_link {
	STS_LINK1 = 1,
	STS_LINK2,
	STS_LINK3,
	STS_LINK4
};

enum e_link_status {
	LINK_UNUSED = 0,
	LINK_PCIE,
	LINK_JESD,
	LINK_ETH,
};

uint32_t get_eeprom_signature(void);
void set_mac_interfaces(void);
void get_board_configs(const char **chip, const char **vendor, const char **board);
u8 get_pci_active_interface_type(void);
u8 get_rf1_card_type(void);
u8 get_rf2_card_type(void);
u8 get_link_status(enum e_link link);
u8* get_product_config(void);
u32 get_board_name_value(void);
u32 get_ddr_config(void);
int copy_rfic_eeprom_data(struct rfic_info *dest);
void pcie_add_bootargs(void *blob);
void add_params_bootargs(void * input_blob,const char *param);
#endif /* _RAPTOR2_EEPROM_H_*/
