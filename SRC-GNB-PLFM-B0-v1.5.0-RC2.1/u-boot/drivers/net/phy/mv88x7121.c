// SPDX-License-Identifier: GPL-2.0+
/*
 * (c) Copyright 2021 EdgeQ Inc
 */

/*
 * Driver for mv88x7121 PHY.
 */

#include <common.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <errno.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>

#include "../eqxgbe/xgbe_xgmac.h"
#include "mv88x7121_image/88x7121_fw.h"


#define SPEED_UNKNOWN		-1

/* Follow the loopback definition in xgbe_xgmac.h */
#define PHY_HOST_LOOPBACK	3
#define PHYAD_MASK		0x1C

#define MV_PMA_DEV		1
#define MV_LINE_SIDE		3
#define MV_HOST_SIDE		4
#define MV_AUTONEG		7
#define MV_CHIP_REG		31

#define MV_MAX_SECTIONS		5
#define MV_HEADER_SIZE		32
#define MV_CKSUM_OFFSET		16
#define MV_RESET_TIMEOUT	5000	// msec
#define GLOBAL_RESET		0xF404
#define RAM_ADDR_LOW		0xF8F0
#define RAM_ADDR_HIGH		0xF8F1
#define RAM_DATA		0xF8F2
#define RAM_CHECKSUM		0xF8F3
#define UPC_STATUS0		0xF80F
#define PHY_EXT_STAT1		0xF098
#define SERDES_LOADER_FAIL	0xBFA
#define SERDES_LOADER_DONE	0xBFF
#define MV_BOOT_INIT		0x00
#define MV_WAIT_DL		0x01
#define MV_CHECKSUM_STARTED	0x02
#define MV_JUMPING_TO_APP	0x03
#define MV_RAM_HEADER_ADDR	0x28000000
#define APP_CODE_START_TIMEOUT	600
#define EXECUTABLE_TIMEOUT	500
#define FLASH_CTRL		0xF418
#define MV_NUM_LANES		4
#define POWER_DOWN_REG		0xF0EF
#define PCS_CONTROL		0x0000
#define MODE_SELECTION		0xF000
#define AUTONEG_CNTL_LINE	0x0000
#define AUTONEG_ADV2_LINE	0x1011
#define AUTONEG_ADV3_LINE	0x0012
#define AUTONEG_LANE_CTRL20_LINE	0x8014
#define AUTONEG_LANE_CTRL21_LINE	0x8015
#define AUTONEG_CNTL_HOST	0x1000
#define AUTONEG_ADV2_HOST	0x1011
#define AUTONEG_ADV3_HOST	0x1012
#define AUTONEG_LANE_CTRL20_HOST	0x9014
#define AUTONEG_LANE_CTRL21_HOST	0x9015
#define POWER_CNTL		0xF0EF
#define PORT_PCS_CNTL		0xF010
#define LOOPBACK_POS		12
#define PCS_40G_STATUS		0x1001
#define PCS_25G_STATUS		0x2001
#define MV_P10LN		10  // 1 lane PCS mode
#define MV_P25LN		18  // 1 lane PCS mode
#define MV_P25LR		20  // 1 lane PCS mode
#define MV_P50UP		60  // 1 lane PCS mode


typedef struct _file_header_type
{
	u32 data_length;
	u32 data_destination;
	u16 sec_checksum;
	u16 data_only;
	u16 port_skip;
	u32 next_hdr_offset;
} file_header_type;

extern u16 loopback_test;

static int chip_hw_reset(struct phy_device *phydev)
{
	u16 reg_val;
	bool reset_done = false;
	int i;

	/* Do hw reset */
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val |= BIT(10);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val |= BIT(9);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val &= ~BIT(10);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val &= ~BIT(9);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val |= BIT(14);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);
	reg_val = phy_read(phydev, MV_CHIP_REG, GLOBAL_RESET);
	reg_val &= ~BIT(12);
	phy_write(phydev, MV_CHIP_REG, GLOBAL_RESET, reg_val);

	/* Wait for reset to complete */
	for (i = 0; i < MV_RESET_TIMEOUT; i++) {
		reg_val = phy_read(phydev, MV_PMA_DEV, 0x0);
		if (reg_val != 0 && reg_val != 0xFFFF) {
			reset_done = true;
			break;
		}
		udelay(1000);
	}
	if (!reset_done) {
		printf("HW reset failed\n");
		return -EFAULT;
	}

	return 0;
}

void get_u32_from_buf(u32 *num, u8 *buf)
{
	unsigned counter;

	*num = 0;
	for (counter = 0; counter < 4; counter++) {
		*num += (((u32)(buf[counter])) << (counter * 8));
	}
}

void get_u16_from_buf(u16 *num, u8 *buf)
{
	unsigned counter;

	*num = 0;
	for (counter = 0; counter < 2; counter++) {
		*num += (((u16)(buf[counter])) << (counter * 8));
	}
}

int get_header(file_header_type *hdr, u8 *buf)
{
	u16 calcd_checksum = 0;
	u16 hdr_checksum;
	u16 hdr_size = MV_HEADER_SIZE - MV_CKSUM_OFFSET;
	u8 *buf_ptr = buf;

	/* First make sure it's a good header */
	while (hdr_size--) {
		calcd_checksum += *buf_ptr++;
	}
	calcd_checksum = ~calcd_checksum;
	get_u16_from_buf(&hdr_checksum, &buf[MV_CKSUM_OFFSET]);

	if (hdr_checksum != calcd_checksum) {
		return -EFAULT;
	}

	/* Now fill in the header structure */
	get_u32_from_buf(&(hdr->data_length), buf);
	buf += 4;
	get_u32_from_buf(&(hdr->data_destination), buf);
	buf += 4;
	get_u16_from_buf(&(hdr->sec_checksum), buf);
	buf += 2;
	hdr->data_only = ((*buf++) & 0x40) >> 6;
	hdr->port_skip = *buf++;
	get_u32_from_buf(&(hdr->next_hdr_offset), buf);

	return 0;
}

int write_to_ram_calc_chksums(struct phy_device *phydev, u8 data[], u32 size,
			      u32 ram_address, u16 *ram_checksum)
{
	u16 reg_val;
	u32 buff_count;
	u8  low_byte;
	u8  high_byte;
	u16 byte_checksum;

	/* This read clears the RAM checksum register */
	reg_val = phy_read(phydev, MV_CHIP_REG, RAM_CHECKSUM);

	/* Set starting address in RAM */
	reg_val = (u16)ram_address;
	phy_write(phydev, MV_CHIP_REG, RAM_ADDR_LOW, reg_val);
	reg_val = (u16)(ram_address >> 16);
	phy_write(phydev, MV_CHIP_REG, RAM_ADDR_HIGH, reg_val);

	/* Copy the code to the phy's internal RAM, calculating checksum
	 * as we go */
	buff_count = 0;
	byte_checksum = 0;
	while (buff_count < size) {
		low_byte = data[buff_count++];
		high_byte = data[buff_count++];

		/* HDR file and RAM interface calc checksum this way*/
		byte_checksum += low_byte;
		byte_checksum += high_byte;

		/* Write to RAM */
		reg_val = ((u16)high_byte << 8) | low_byte;
		phy_write(phydev, MV_CHIP_REG, RAM_DATA, reg_val);
	}
	*ram_checksum = byte_checksum;

	/* Read hardware checksum register and see if it matches the locally
	* computed checksum. This will indicate all the data got there ok. */
	reg_val = phy_read(phydev, MV_CHIP_REG, RAM_CHECKSUM);
	if (byte_checksum != reg_val) {
		printf("Error downloading code.\n");
		printf("Expected checksum 0x%04X HW checksum 0x%04X",
		       byte_checksum, reg_val);
		return -EFAULT;
	}

	return 0;
}

static void poll_phy_app_code_started(struct phy_device *phydev, u16 tmo,
				      bool *app_started)
{
	u16 reg_val;
	int i;

	*app_started = false;
	for (i = 0; i < tmo; i++) {
		reg_val = phy_read(phydev, MV_CHIP_REG, UPC_STATUS0);
		if (reg_val & BIT(4)) {
			*app_started = true;
			break;
		}
		udelay(1000);
	}
}

static int write_hdr_to_ram(struct phy_device *phydev, u8 fw_image[],
			    u32 fw_size)
{
	u16 reg_val;
	bool in_dl_mode = false;
	u16 byteChecksum, sectionCount = 0, executableCount = 0;
	u32 ramAddress, imageIndex = 0;
	file_header_type f_header;
	bool appStarted, hasNextSection = false;
	int status;
	int i;

	/* fw_size must be an even number of bytes */
	if (fw_image == NULL || fw_size == 0 || (fw_size % 2)) {
		printf("Invalid fw image\n");
		return -EINVAL;
	}

	/* Make sure the devices are in the download mode */
	for (i = 0; i < MV_RESET_TIMEOUT/10; i++) {
		reg_val = phy_read(phydev, MV_CHIP_REG, UPC_STATUS0);
		if (((reg_val & GENMASK(2, 1)) >> 1) == MV_WAIT_DL) {
			in_dl_mode = true;
			break;
		}
		udelay(1000);
	}
	if (!in_dl_mode) {
		printf("Device not in fw download mode\n");
		return -EFAULT;
	}
		
	do {
		if (imageIndex > fw_size) {
			printf("Image size and image data do not match.\n");
			return -EINVAL;
		}

		/* Now process one or more headers */
		if (get_header(&f_header, &fw_image[imageIndex]) != 0) {
			printf("Header corrupted\n");
			return -EFAULT;
		}
		else {
			hasNextSection = (f_header.next_hdr_offset != 0 &&
				f_header.next_hdr_offset != 0xFFFFFFFF) ?
				true : false;
		}

		/* if the section is not available, data_length is 0 */
		if (f_header.data_length != 0) {
			ramAddress = MV_RAM_HEADER_ADDR + MV_HEADER_SIZE *
				     sectionCount;
			status = write_to_ram_calc_chksums(phydev,
						&fw_image[imageIndex],
						MV_HEADER_SIZE,
						ramAddress, &byteChecksum);
			if (status != 0) {
				printf("Failed writing to ram\n");
				return -EFAULT;
			}
			imageIndex += MV_HEADER_SIZE; /* Skip over the header */
			ramAddress = f_header.data_destination;
			status = write_to_ram_calc_chksums(phydev,
						&fw_image[imageIndex],
						f_header.data_length,
						ramAddress, &byteChecksum);
			if (status != 0) {
				printf("Failed writing to ram\n");
				return -EFAULT;
			}

			byteChecksum = (~byteChecksum);
			if (byteChecksum != f_header.sec_checksum) {
				printf("Error downloading code\n");
				printf("Expected HDR checksum 0x%04X actual 0x%04X on section %d",
                                       (int)f_header.sec_checksum,
                                       (int)byteChecksum,
                                       (int)sectionCount);
                                return -EFAULT;
			}

			if (!f_header.data_only) {
				reg_val = phy_read(phydev, MV_CHIP_REG,
						   UPC_STATUS0);
				reg_val |= BIT(6);
				phy_write(phydev, MV_CHIP_REG, UPC_STATUS0,
						   reg_val);

				/* Just delay. No Error code return if failed */
				udelay(10000);

				/* Check all the devices code started */
				poll_phy_app_code_started(phydev,
							APP_CODE_START_TIMEOUT,
							&appStarted);
				if (appStarted == false) {
					printf("App code did not start.\n");
					return -EFAULT;
				}

				if (executableCount == 0) {
					/* wait serdes load done.
					 * Need about 31 msec. */
					udelay(100000);
				}
				executableCount++;
			}
		}

		/* Go to the next section if more are coming */
		if (hasNextSection) {
			sectionCount++;
			/* sections don't need to be sequential using
			 * this approach */
			imageIndex = f_header.next_hdr_offset;
		}
	} while (sectionCount < MV_MAX_SECTIONS && hasNextSection);

	if (sectionCount >= MV_MAX_SECTIONS && hasNextSection) {
		/* there's something wrong with the file, maximum section
		 * number reached but more sections are coming */
		printf("Max sections reached but more data is coming.\n");
		return -EFAULT;
	}

	return 0;
}

static int fw_download(struct phy_device *phydev, u8 fw_image[], u32 fw_size)
{
	u16 reg_val;
	int status;

	/* Disable flash loading */
	reg_val = phy_read(phydev, MV_CHIP_REG, FLASH_CTRL);
	reg_val |= BIT(0);
	phy_write(phydev, MV_CHIP_REG, FLASH_CTRL, reg_val);

	/* Do chip hw reset */
	chip_hw_reset(phydev);

	status = write_hdr_to_ram(phydev, fw_image, fw_size);
	if (status != 0)
		return status;

	return 0;
}

static int mv88x7121_probe(struct phy_device *phydev)
{
	static bool load_fw_done = false;
	u16 reg_val;
	u16 phy_addr_sav;
	int i;
	int status;

	if (load_fw_done)
		return 0;

	/* mv88x7121 uses base mdio port for fw download */
	phy_addr_sav = phydev->addr;
	phydev->addr = (phy_addr_sav & PHYAD_MASK);
	status = fw_download(phydev, x7121_fw_image, sizeof(x7121_fw_image));
	phydev->addr = phy_addr_sav;
	if (status != 0)
		return status;

	/* Power down line side lanes first, then host side */
	for (i = 0; i < MV_NUM_LANES; i++) {
		reg_val = phy_read(phydev, MV_LINE_SIDE, POWER_DOWN_REG);
		reg_val |= (0x3 << (i * 2));
		phy_write(phydev, MV_LINE_SIDE, POWER_DOWN_REG, reg_val);
	}
	for (i = 0; i < MV_NUM_LANES; i++) {
		reg_val = phy_read(phydev, MV_HOST_SIDE, POWER_DOWN_REG);
		reg_val |= (0x3 << (i * 2));
		phy_write(phydev, MV_HOST_SIDE, POWER_DOWN_REG, reg_val);
	}

	load_fw_done = true;

	return 0;
}

static int port_power_down(struct phy_device *phydev, u16 host_or_line,
			   u16 phy_lane)
{
	u16 reg;
	u16 reg_val;

	/* Power down the port */
	reg = PCS_CONTROL + (phy_lane * 0x200);
	reg_val = phy_read(phydev, host_or_line, reg);
	reg_val |= BIT(11);
	phy_write(phydev, host_or_line, reg, reg_val);
	reg = MODE_SELECTION + phy_lane;
	reg_val = phy_read(phydev, host_or_line, reg);
	reg_val |= 0xA000;
	phy_write(phydev, host_or_line, reg, reg_val);

	return 0;
}

static int restore_start(struct phy_device *phydev, u16 host_or_line,
			 u16 phy_lane)
{
	u16 reg;
	u16 reg_val;

	if (host_or_line == MV_HOST_SIDE) {
		reg = AUTONEG_CNTL_HOST + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= ~BIT(12);
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV3_HOST + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= ~GENMASK(11, 0);
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL21_HOST + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0xFFFA;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV2_HOST + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0x4C5F;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL20_HOST + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0xFC4F;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV3_HOST + (phy_lane * 0x200);
		reg_val = 0x0;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL21_HOST + (phy_lane * 0x200);
		reg_val = 0x0;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);
	}
	else {
		reg = AUTONEG_CNTL_LINE + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= ~BIT(12);
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV3_LINE + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= ~GENMASK(11, 0);
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL21_LINE + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0xFFFA;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV2_LINE + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0x4C5F;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL20_LINE + (phy_lane * 0x200);
		reg_val = phy_read(phydev, MV_AUTONEG, reg);
		reg_val &= 0xFC4F;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_ADV3_LINE + (phy_lane * 0x200);
		reg_val = 0x0;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);

		reg = AUTONEG_LANE_CTRL21_LINE + (phy_lane * 0x200);
		reg_val = 0x0;
		phy_write(phydev, MV_AUTONEG, reg, reg_val);
	}

	return 0;
}

static int restore_start_mod(struct phy_device *phydev, u16 host_or_line,
			     u16 phy_lane, u16 op_mode)
{
	u16 mode_lane_count;
	u16 reg;
	u16 reg_val;

	/* If other mode is used, lane cound may change. */
	if (op_mode == MV_P50UP || op_mode == MV_P25LN ||
	    op_mode == MV_P25LR || op_mode == MV_P10LN) {
		mode_lane_count = 1;
	}
	else {
		printf("Unsupported op mode (%d)\n", op_mode);
		return -EINVAL;
	}

	if (phy_lane == 0) {
		reg_val = phy_read(phydev, host_or_line, 0xF052);
		reg_val &= ~GENMASK(9, 8);
		phy_write(phydev, host_or_line, 0xF052, reg_val);
	}
	else if (phy_lane == 2) {
		reg_val = phy_read(phydev, host_or_line, 0xF052);
		reg_val &= ~BIT(11);
		phy_write(phydev, host_or_line, 0xF052, reg_val);
	}

	if (mode_lane_count == 1) {
		reg = 0xF0A0 + phy_lane;
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(14);
		phy_write(phydev, host_or_line, reg, reg_val);

		reg = 0xF18C + (phy_lane * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(11);
		phy_write(phydev, host_or_line, reg, reg_val);
	}
	else if (mode_lane_count == 2) {
		reg = 0xF0A0 + phy_lane;
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(14);
		phy_write(phydev, host_or_line, reg, reg_val);

		reg = 0xF0A0 + phy_lane + 1;
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(14);
		phy_write(phydev, host_or_line, reg, reg_val);

		reg = 0xF18C + (phy_lane * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(11);
		phy_write(phydev, host_or_line, reg, reg_val);

		reg = 0xF18C + ((phy_lane + 1) * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(11);
		phy_write(phydev, host_or_line, reg, reg_val);
	}

	reg_val = phy_read(phydev, host_or_line, 0xF0C0);
	reg_val &= ~GENMASK(3, 1);
	phy_write(phydev, host_or_line, 0xF0C0, reg_val);
	reg_val = phy_read(phydev, host_or_line, 0xF0C1);
	reg_val &= ~GENMASK(15, 13);
	phy_write(phydev, host_or_line, 0xF0C1, reg_val);

	return 0;
}

static int fec_an_config(struct phy_device *phydev, u16 host_or_line,
			 u16 phy_lane, u16 op_mode)
{
	/* If other mode is used, lane cound may change. */
	if (op_mode == MV_P50UP || op_mode == MV_P25LN ||
	    op_mode == MV_P25LR || op_mode == MV_P10LN) {
		/* Do nothing */
	}
	else {
		printf("Unsupported op mode (%d)\n", op_mode);
		return -EINVAL;
	}

	return 0;
}

static int mode_workaround(struct phy_device *phydev, u16 host_or_line,
			   u16 phy_lane, u16 op_mode)
{
	u16 mode_lane_count;
	u16 reg;
	u16 reg_val;

	/* If other mode is used, lane cound may change. */
	if (op_mode == MV_P50UP || op_mode == MV_P25LN ||
	    op_mode == MV_P25LR || op_mode == MV_P10LN) {
		mode_lane_count = 1;
	}
	else {
		printf("Unsupported op mode (%d)\n", op_mode);
		return -EINVAL;
	}

	if (mode_lane_count == 1) {
		reg = 0xF193 + (phy_lane * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(1);
		phy_write(phydev, host_or_line, reg, reg_val);
	}
	else if (phy_lane == 2) {
		reg = 0xF193 + (phy_lane * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(1);
		phy_write(phydev, host_or_line, reg, reg_val);

		reg = 0xF193 + ((phy_lane + 1) * 0x20);
		reg_val = phy_read(phydev, host_or_line, reg);
		reg_val &= ~BIT(1);
		phy_write(phydev, host_or_line, reg, reg_val);
	}

	return 0;
}

static int port_power_down_other_lane(struct phy_device *phydev,
				 u16 host_or_line, u16 phy_lane, u16 op_mode)
{
	u16 mode_lane_count;

	/* If other mode is used, lane cound may change. */
	if (op_mode == MV_P50UP || op_mode == MV_P25LN ||
	    op_mode == MV_P25LR || op_mode == MV_P10LN) {
		mode_lane_count = 1;
	}
	else {
		printf("Unsupported op mode (%d)\n", op_mode);
		return -EINVAL;
	}

	/* Only needed if mode_line_count > 1 */
	if (mode_lane_count == 1) {
		/* Nothing to do */
		return 0;
	}

	return 0;
}

static int set_fixed_mode(struct phy_device *phydev, u16 host_or_line,
			  u16 phy_lane, u16 op_mode)

{
	u16 reg;
	u16 reg_val;
	int status;

	status = port_power_down(phydev, host_or_line, phy_lane);
	if (status != 0)
		return status;

	status = restore_start(phydev, host_or_line, phy_lane);
	if (status != 0)
		return status;

	status = restore_start_mod(phydev, host_or_line, phy_lane, op_mode);
	if (status != 0)
		return status;

	status = fec_an_config(phydev, host_or_line, phy_lane, op_mode);
	if (status != 0)
		return status;

	status = mode_workaround(phydev, host_or_line, phy_lane, op_mode);
	if (status != 0)
		return status;

	status = port_power_down_other_lane(phydev, host_or_line, phy_lane,
					    op_mode);
	if (status != 0)
		return status;

	/* Set mode register */
	reg = MODE_SELECTION + phy_lane;
	reg_val = phy_read(phydev, host_or_line, reg);
	if (op_mode == MV_P10LN)
		reg_val = 0x8023;
	else if (op_mode == MV_P25LN)
		reg_val = 0x8024;
	else if (op_mode == MV_P25LR)
		reg_val = 0x80A4;
	else if (op_mode == MV_P50UP)
		reg_val = 0x81E6;
	else
		return -EINVAL;
	phy_write(phydev, host_or_line, reg, reg_val);

	/* PMA power up */
	reg = PCS_CONTROL + (phy_lane * 0x200);
	reg_val = phy_read(phydev, host_or_line, reg);
	reg_val &= ~BIT(11);
	phy_write(phydev, host_or_line, reg, reg_val);

	return 0;
}

static int mv88x7121_phy_config(struct phy_device *phydev)
{
	struct dw_eth_dev *priv = (struct dw_eth_dev *)phydev->priv;
	u16 host_mode;
	u16 line_mode;
	u16 reg_val;
	int status;

	if (priv->max_speed == SPEED_50000) {
		host_mode = MV_P50UP; // No auto-neg on host side
		line_mode = MV_P50UP;
	}
	else if (priv->max_speed == SPEED_25000) {
		host_mode = MV_P25LN; // No auto-neg on host side
		line_mode = MV_P25LR;
	}
	else {
		host_mode = MV_P10LN; // No auto-neg on host side
		line_mode = MV_P10LN;
	}

	status = set_fixed_mode(phydev, MV_LINE_SIDE, priv->phy_lane,
				line_mode);
	if (status != 0)
		return status;

	status = set_fixed_mode(phydev, MV_HOST_SIDE, priv->phy_lane,
				host_mode);
	if (status != 0)
		return status;

	reg_val = phy_read(phydev, MV_HOST_SIDE, POWER_CNTL);
	reg_val &= ~(0x3 << (priv->phy_lane * 2));
	phy_write(phydev, MV_HOST_SIDE, POWER_CNTL, reg_val);

	reg_val = phy_read(phydev, MV_LINE_SIDE, POWER_CNTL);
	reg_val &= ~(0x3 << (priv->phy_lane * 2));
	phy_write(phydev, MV_LINE_SIDE, POWER_CNTL, reg_val);

	return 0;
}

static int mv88x7121_phy_startup(struct phy_device *phydev)
{
	struct dw_eth_dev *priv = (struct dw_eth_dev *)phydev->priv;
	u16 reg;
	u16 reg_val;
	u16 host_link_status;
	u16 line_link_status;
	int retries;
	int i;

	/* Wait upto 0.2 second for link to come up */
	phydev->link = 0;
	retries = 2000;
	if (priv->max_speed == SPEED_50000)
		reg = PCS_40G_STATUS + (priv->phy_lane * 0x200);
	else
		reg = PCS_25G_STATUS + (priv->phy_lane * 0x200);
	/* First read is latched status */
	reg_val = phy_read(phydev, MV_HOST_SIDE, reg);
	reg_val = phy_read(phydev, MV_LINE_SIDE, reg);
	for (i = 0; i < retries; i++) {
		/* Need two reads for current status */
		reg_val = phy_read(phydev, MV_HOST_SIDE, reg);
		host_link_status = (reg_val & BIT(2));
		reg_val = phy_read(phydev, MV_LINE_SIDE, reg);
		line_link_status = (reg_val & BIT(2));
		if (host_link_status && line_link_status) {
			phydev->link = 1;
			break;
		}
		udelay(100);
	}

	if (phydev->link == 0) {
		if (!host_link_status)
			printf("PHY host side link down\n");
		if (!line_link_status)
			printf("PHY line side link down\n");
	}

	/* For loopback test, fake link status in case link is not up */
	if (loopback_test == PHY_HOST_LOOPBACK) {
		if (phydev->link == 0) {
			phydev->link = 1;
			printf("Fake link status in loopback test\n");
		}
	}

	if (phydev->link) {
		phydev->speed = priv->max_speed;
		phydev->duplex = DUPLEX_FULL;
	}
	else {
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_HALF;
	}

	return 0;
}

static struct phy_driver mv88x7121m_driver = {
	.name = "Marvell MV88X7121M",
	.uid = 0x2b0bbe,
	.mask = 0xffffff,
	.features = PHY_10G_FEATURES,
	.probe = mv88x7121_probe,
	.config = mv88x7121_phy_config,
	.startup = mv88x7121_phy_startup,
	.shutdown = &genphy_shutdown,
};

int phy_mv88x7121_init(void)
{
	phy_register(&mv88x7121m_driver);

	return 0;
}

