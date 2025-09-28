/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#include <common.h>
#include <linux/kernel.h>
#include <u-boot/crc.h>
#include <eeprom.h>
#include <eeprom_layout.h>
#include <eeprom_field.h>
#include <env.h>
#include <log.h>

#include "jesdinit/jesdinit.h"
#include "eeprom_data.h"
#include "raptor2_eeprom.h"
#include "eeprom_layout_versions.h"

#ifdef CONFIG_CMD_EEPROM_LAYOUT
struct eeprom_layout *current_layout;
#endif

static const char *eth_speed[MAX_NUM_ETHSPEED] = {"1g", "2g5", "5g",
						"10g", "25g", "50g"};

static const char *vendors[MAX_VENDORS] = {"EdgeQ",
						"WNC",
						"Wistron"};

static const char *board_names[MAX_VENDORS][MAX_BOARDS] = {
						{"EVB-V2", "Titan", "Hawk", "PCIe Acc Card"},
						{"Pegasus"},
						{"Pheonix"} };

int verify_eeprom_crc(void *eeprom, int g_count, size_t *g_sizes)
{
	unsigned char *buff;
	uint32_t crc;
	uint32_t calc_crc;
	int i;
	size_t crc32_size = sizeof(CRC32_TYPE);

	buff = (unsigned char *)eeprom;
	for (i = 0; i < g_count; i++) {
		crc = *(CRC32_TYPE *)buff;

		if (crc == EEPROM_CRC_VERIFIED) {
			buff += g_sizes[i];
			continue;
		}

		calc_crc = crc32(0, buff + crc32_size, g_sizes[i] - crc32_size);
		buff += g_sizes[i];

		if (crc != calc_crc)
			return i;
	}

	return i;
}

uint32_t get_eeprom_signature(void)
{
	return ((struct eeprom_data *)EEPROM_BUFFER_ADDR)->eeprom_info.signature;
}

void set_mac_interfaces(void)
{
	int ret, i;
	char mac_string[20] = {0};
	char ethlink_env[2] = {0};
	char ethaddr_env[10] = {0};
	char ethact_env[6] = {0};
	char ethspeed_env[11] = {0};
	char ecpri_env[9] = {0};
	int num_groups = 1;
	struct ethernet_configs *ethernet_configs;

	ethernet_configs = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->ethernet_configs);

	ret = verify_eeprom_crc(ethernet_configs, num_groups, &group_sizes[GROUP_ETHERNET_CONFIGS]);
	if (ret != num_groups)
		return;

	if (ethernet_configs->max_interfaces > MAX_ETH_INTERFACES) {
		printf("Ethernet Configs: Invalid number of max interfaces\n");
		return;
	}

	if (ethernet_configs->active_interfaces > ethernet_configs->max_interfaces) {
		printf("Ethernet Configs: Invalid number of active interfaces\n");
		return;
	}

	if (ethernet_configs->ethlink && ethernet_configs->ethlink <= MAX_ETHLINK) {
		sprintf(ethlink_env, "%d", ethernet_configs->ethlink);
		env_set("ethlink", ethlink_env);

		if (ethernet_configs->ethact < ethernet_configs->active_interfaces) {
			sprintf(ethact_env, "eth%d", ethernet_configs->ethact);
			env_set("ethact", ethact_env);
		}
	}

	if (get_link_status(STS_LINK1) == LINK_ETH && get_link_status(STS_LINK2) != LINK_ETH)
		env_set("ethlink", "1");
	else if (get_link_status(STS_LINK1) != LINK_ETH && get_link_status(STS_LINK2) == LINK_ETH)
		env_set("ethlink", "2");
	else if (get_link_status(STS_LINK1) != LINK_ETH && get_link_status(STS_LINK2) != LINK_ETH)
		printf("Invalid Config for Link1 and Link2\n");

	for (i = 0; i < ethernet_configs->active_interfaces; i++) {
		BYTES2MAC(mac_string, (&ethernet_configs->mac0)[i]);

		if (!i) {
			env_set("ethaddr", mac_string);

			if (ethernet_configs->ethspeeds[i] && ethernet_configs->ethspeeds[i] <= MAX_NUM_ETHSPEED)
				env_set("ethspeed", eth_speed[ethernet_configs->ethspeeds[i] - 1]);
		} else {
			sprintf(ethaddr_env, "eth%daddr", i);
			env_set(ethaddr_env, mac_string);

			if (ethernet_configs->ethspeeds[i] && ethernet_configs->ethspeeds[i] <= MAX_NUM_ETHSPEED) {
				sprintf(ethspeed_env, "eth%dspeed", i);
				env_set(ethspeed_env, eth_speed[ethernet_configs->ethspeeds[i] - 1]);
			}
		}
	}

	sprintf(ecpri_env, "%d%d%d%d", (ethernet_configs->ecpri_bitmap & 0x08) >> 3,
									(ethernet_configs->ecpri_bitmap & 0x04) >> 2, 
									(ethernet_configs->ecpri_bitmap & 0x02) >> 1,
									ethernet_configs->ecpri_bitmap & 0x01);
	env_set("ecpri_en", ecpri_env);

}

void get_board_configs(const char **chip, const char **vendor, const char **board)
{
	struct board_configs *board_configs;
	int num_groups = 1;
	int ret, vendor_idx, board_idx;

	board_configs = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs);
	ret = verify_eeprom_crc(board_configs, num_groups, &group_sizes[GROUP_BOARD_CONFIGS]);
	if (ret != num_groups)
		return;

	if ((board_configs->soc_chip_id & CHIP_NAME_MASK) == SOC_CHIP_B0)
		*chip = "EQ2542";
	else
		*chip = "EQ2540";

	vendor_idx = GET_VENDOR_IDX(board_configs->board_version);
	board_idx = GET_BOARD_IDX(board_configs->board_version);

	if (vendor_idx < MAX_VENDORS) {
		*vendor = vendors[vendor_idx];

		if (board_idx < MAX_BOARDS) {
			*board = board_names[vendor_idx][board_idx];

			if (*board == NULL)
				*board = "Invalid";
		} else {
			*board = "Invalid";
		}
	} else {
		*vendor = "Invalid";
		*board = "Invalid";
	}
}

u8 get_pci_active_interface_type(void)
{
	return (((struct eeprom_data *)EEPROM_BUFFER_ADDR)->pcie_usb_configs.active_pcie_interfaces);
}

u8 get_rf1_card_type(void)
{
	return (((struct eeprom_data *)EEPROM_BUFFER_ADDR)->rfic_configs.rf1_card_type
			& RF_CARD_TYPE_MASK);
}

u8 get_rf2_card_type(void)
{
	return (((struct eeprom_data *)EEPROM_BUFFER_ADDR)->rfic_configs.rf2_card_type
			& RF_CARD_TYPE_MASK);
}

u32 get_board_version(void)
{
	return ((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs.board_version;
}

u8* get_product_config(void)
{
	return ((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs.product_config;
}

u32 get_board_name_value(void)
{
	u32 board = 0xFFFFFFFF;
	u32 board_version = get_board_version();

	if (board_version == 0x00000100 || board_version == 0x00000101 )
		board = BOARD_TITAN;
	else if (board_version == 0x00000200)
		board = BOARD_HAWK;
	else if (board_version == 0x00000201)
		board = BOARD_HAWKV2;
	else if (board_version == 0x00000202)
		board = BOARD_HAWKV3;
	else if (board_version == 0x00000203)
		board = BOARD_HAWKV4;
	else if (board_version == 0x00000204)
		board = BOARD_HAWKV5;
	else if (board_version == 0x00010000)
		board = BOARD_PEGASUS;
	else
		board = BOARD_UNKNOWN;

	return board;
}

u8 get_link_status(enum e_link link)
{
	struct board_configs *board_config = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs);
	u8 link_status = 0;

	switch (link) {
	case STS_LINK1:
		link_status = board_config->link1_status;
		break;
	case STS_LINK2:
		link_status = board_config->link2_status;
		break;
	case STS_LINK3:
		link_status = board_config->link3_status;
		break;
	case STS_LINK4:
		link_status = board_config->link4_status;
		break;
	default:
		link_status = 0;
		break;
	}

	printf("u-boot: Link 0x%X value is 0x%X\n", link, link_status);

	return link_status;
}

u32 get_ddr_config(void)
{
	return ((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs.ddr_config;
}

int copy_rfic_eeprom_data(struct rfic_info *dest)
{
	struct rfic_configs *rfic = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->rfic_configs);
	struct jesd_configs *jesd = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->jesd_configs);

	if (!dest) {
		printf("u-boot:JESD: invalid pointer provided for copying RFIC configs\r\n");
		return -1;
	}

	debug("u-boot:JESD: Copying all EEPROM data into the local variables\r\n");
	dest->rf1_card_type = rfic->rf1_card_type;
	dest->rf2_card_type = rfic->rf2_card_type;
	dest->link3_lane_select = jesd->link3_lane_select;
	dest->link4_lane_select = jesd->link4_lane_select;
	dest->link3_lane_polarity = jesd->link3_lane_polarity;
	dest->link4_lane_polarity = jesd->link4_lane_polarity;
	dest->link_speed = jesd->link_speed;

	debug("u-boot:JESD: EEPROM DATA:\r\n"
			"rf1_card_type:0x%X\r\n"
			"rf2_card_type:0x%X\r\n"
			"link3_lane_select:0x%X\r\n"
			"link4_lane_select:0x%X\r\n"
			"link3_lane_polarity:0x%X\r\n"
			"link4_lane_polarity:0x%X\r\n"
			"link_speed:0x%X\r\n",
			rfic->rf1_card_type, rfic->rf2_card_type,
			jesd->link3_lane_select, jesd->link4_lane_select,
			jesd->link3_lane_polarity, jesd->link4_lane_polarity,
			jesd->link_speed);

	return 0;
}

#ifdef CONFIG_CMD_EEPROM_LAYOUT
static int eeprom_update_crc(int offset, int size)
{
	unsigned char *buff = NULL;
	uint32_t crc;

	if (!current_layout || !current_layout->data || !size)
		return -1;

	buff = current_layout->data + offset;

	crc = crc32(0, buff + sizeof(uint32_t), size - sizeof(uint32_t));
	*(uint32_t *)buff = crc;

	return 0;
}

int eeprom_info_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int eeprom_info_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		eeprom_info_offset = offsetof(struct eeprom_data, eeprom_info);
		ret = eeprom_update_crc(eeprom_info_offset, sizeof(struct eeprom_info));
	}

	return ret;
}

int board_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int board_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		board_configs_offset = offsetof(struct eeprom_data, board_configs);
		ret = eeprom_update_crc(board_configs_offset, sizeof(struct board_configs));
	}

	return ret;
}

int pcie_usb_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int pcie_usb_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		pcie_usb_configs_offset = offsetof(struct eeprom_data, pcie_usb_configs);
		ret = eeprom_update_crc(pcie_usb_configs_offset, sizeof(struct pcie_usb_configs));
	}

	return ret;
}

int rfic_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int rfic_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		rfic_configs_offset = offsetof(struct eeprom_data, rfic_configs);
		ret = eeprom_update_crc(rfic_configs_offset, sizeof(struct rfic_configs));
	}

	return ret;
}

int ethernet_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int ethernet_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		ethernet_configs_offset = offsetof(struct eeprom_data, ethernet_configs);
		ret = eeprom_update_crc(ethernet_configs_offset, sizeof(struct ethernet_configs));
	}

	return ret;
}

int ethernet_configs_fields_update_mac(struct eeprom_field *field, char *value)
{
	int ret;
	int ethernet_configs_offset;

	ret = eeprom_field_update_mac(field, value);

	if (!ret) {
		ethernet_configs_offset = offsetof(struct eeprom_data, ethernet_configs);
		ret = eeprom_update_crc(ethernet_configs_offset, sizeof(struct ethernet_configs));
	}

	return ret;
}

int misc_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int misc_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		misc_configs_offset = offsetof(struct eeprom_data, misc_configs);
		ret = eeprom_update_crc(misc_configs_offset, sizeof(struct misc_configs));
	}

	return ret;
}

int jesd_configs_fields_update_bin_rev(struct eeprom_field *field, char *value)
{
	int ret;
	int jesd_configs_offset;

	ret = eeprom_field_update_bin_rev(field, value);

	if (!ret) {
		jesd_configs_offset = offsetof(struct eeprom_data, jesd_configs);
		ret = eeprom_update_crc(jesd_configs_offset, sizeof(struct jesd_configs));
	}

	return ret;
}

int board_configs_fields_update_ascii(struct eeprom_field *field, char *value)
{
	int ret;
	int board_configs_offset;

	ret = eeprom_field_update_ascii(field, value);

	if (!ret) {
		board_configs_offset = offsetof(struct eeprom_data, board_configs);
		ret = eeprom_update_crc(board_configs_offset, sizeof(struct board_configs));
	}

	return ret;
}

int eeprom_field_update_dummy(struct eeprom_field *field, char *value)
{
	/* Dummy update function for crc32*/
	return 0;
}

int eeprom_parse_layout_version(char *str)
{
	if (!strcmp(str, "v1"))
		return LAYOUT_VERSION_VER1;
	else
		return LAYOUT_VERSION_UNRECOGNIZED;
}

int eeprom_layout_detect(unsigned char *data)
{
	struct eeprom_data *eeprom = (struct eeprom_data *)data;

	switch (eeprom->eeprom_info.version) {
	case 1:
		return LAYOUT_VERSION_VER1;
	default:
		return LAYOUT_VERSION_UNRECOGNIZED;
	}
}

void eeprom_layout_assign(struct eeprom_layout *layout, int layout_version)
{
	layout->fields = layout_v1;
	layout->num_of_fields = ARRAY_SIZE(layout_v1);

	current_layout = layout;
}

#endif /* CONFIG_CMD_EEPROM_LAYOUT */
