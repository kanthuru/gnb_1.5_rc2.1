// SPDX-License-Identifier: GPL-2.0+
/*
 * (c) Copyright 2021 EdgeQ Inc
 */

/*
 * Driver for mv88x3310 PHY.
 */

#include <common.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <errno.h>
#include <miiphy.h>
#include <netdev.h>
#include <phy.h>

#include "mv88x3310_image/88x3310_fw.h"


#define SPEED_UNKNOWN		-1

/* Follow the loopback definition in xgbe_xgmac.h */
#define PHY_HOST_LOOPBACK	3

#define MV_T_UNIT_PMA_PMD	1
#define MV_X_UNIT		3
#define MV_H_UNIT		4
#define MV_C_UNIT		31
#define MV_M_UNIT		31

#define MV_MAX_HEADERS		2
#define MV_HEADER_SIZE		32
#define MV_CKSUM_OFFSET		16
#define MV_MS_CU_ONLY		0
#define MV_MS_FBR_ONLY		1
#define MV_MS_AUTO_PREFER_CU	2
#define MV_MS_AUTO_PREFER_FBR	3
#define MV_MS_AUTO_FIRST_LINK	7
#define MV_FT_SGMII		1
#define MV_FT_10GBASER		3
#define MV_MAC_TYPE_SGMII_ANOFF	0x5
#define MV_MAC_TYPE_USXGMII	0x7
#define MV_MAC_SNOOP_OFF	0x0
#define MV_MAX_MAC_SPEED_10G	0x0

#define CUNIT_PORT_CTRL		0xF001
#define BOOT_STATUS		0xC050
#define ADDR_LOW		0xD0F0
#define ADDR_HIGH		0xD0F1
#define RAM_DATA		0xD0F2
#define RAM_CHECKSUM		0xD0F3
#define EXEC_STATUS		0xC001
#define EXEC_DONE		0xBFF
#define EXEC_SKIP		0xBFE
#define DOWNLOAD_MODE		0xF008
#define BOOT_PROGRESS_POS	1
#define BOOT_PROGRESS_MASK	(0x3 << BOOT_PROGRESS_POS)
#define WAIT_DL			0x01
#define DEVICE_INFO		0xD00D
#define PMA_CTRL1		0x0000
#define BIST_STATUS		0xC00C
#define CUNIT_MODE_CONFIG	0xF000
#define USXGMII_AN_CNTL		0xC049
#define COMPHY_ACCESS_CTRL	0xF0A8
#define MV10GBR_PCS_CONTROL	0x1000
#define SERDES_CONTROL1		0xF003
#define GPIO_DATA		0xF012
#define GPIO_CONTROL		0xF013
#define USXGMII_AUTO_NEG_BASE	0xF0A6
#define USXGMII_LINK_UP		0x8000
#define USXGMII_DUPLEX		0x1000
#define USXGMII_SPEED		0x0E00
#define USXGMII_SPEED_10G	0x0600
#define USXGMII_SPEED_5G	0x0A00
#define USXGMII_SPEED_2G5	0x0800
#define USXGMII_SPEED_1G	0x0400
#define MV10GBR_PCS_STATUS1	0x1001
#define MV10GBR_PCS_FAULT	0x1008
#define COPPER_STATUS		0x8008
#define COPPER_LINK_UP		BIT(10)
#define COPPER_SPEED_DUP	BIT(11)
#define COPPER_SPEED_MASK	GENMASK(15, 14)
#define COPPER_HI_SPEED		COPPER_SPEED_MASK
#define COPPER_HI_SPEED_MASK	GENMASK(3, 2)
#define COPPER_10G		(0x0 << 2)
#define COPPER_5G		(0x2 << 2)
#define COPPER_2G5		(0x1 << 2)
#define COPPER_1G		(0x2 << 14)
#define SGMII_STATUS		0x2001
#define SGMII_LINK_UP		0x4

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

int ram_dnld_calc_checksums(struct phy_device *phydev, u8 data[], u32 size,
			    u32 ram_address, u16 *ram_checksum)
{
	u16 reg_val;
	u32 buff_count;
	u8  low_byte;
	u8  high_byte;
	u16 byte_checksum;

	/* This read clears the RAM checksum register */
	reg_val = phy_read(phydev, 3, RAM_CHECKSUM);

	/* Set starting address in RAM */
	reg_val = (u16)ram_address;
	phy_write(phydev, 3, ADDR_LOW, reg_val);
	reg_val = (u16)(ram_address >> 16);
	phy_write(phydev, 3, ADDR_HIGH, reg_val);

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
		phy_write(phydev, 3, RAM_DATA, reg_val);
	}
	*ram_checksum = byte_checksum;

	/* Read hardware checksum register and see if it matches the locally
	 * computed checksum. This will indicate all the data got there ok. */
	reg_val = phy_read(phydev, 3, RAM_CHECKSUM);
	if (byte_checksum != reg_val) {
		printf("Error downloading code.\n");
		printf("Expected checksum 0x%04X HW checksum 0x%04X",
		       byte_checksum, reg_val);
		return -EFAULT;
	}

	return 0;
}

int wait_for_exec_done(struct phy_device *phydev, u16 tmo_ms)
{
	u16 reg_val;
	u16 tmo_cnt = 0;
	int status = -EFAULT;

	while (tmo_cnt++ < tmo_ms) {
		reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, EXEC_STATUS);
		reg_val &= GENMASK(11, 0);
		if (reg_val == EXEC_DONE || reg_val == EXEC_SKIP) {
			status = 0;
			break;
		}
		udelay(1000);
	}

	return status;
}

int put_phy_in_dl_mode(struct phy_device *phydev)
{
	u16 reg_val;

	/* Put PHY in download mode */
	reg_val = phy_read(phydev, MV_C_UNIT, DOWNLOAD_MODE);
	reg_val |= BIT(5);
	phy_write(phydev, MV_C_UNIT, DOWNLOAD_MODE, reg_val);

	/* Reset PHY */
	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_PORT_CTRL);
	reg_val |= BIT(12);
	phy_write(phydev, MV_C_UNIT, CUNIT_PORT_CTRL, reg_val);

	/* Allow reset to complete */
	udelay(50000);

	/* Check if in download mode */
	reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, BOOT_STATUS);
	if (((reg_val & BOOT_PROGRESS_MASK) >> BOOT_PROGRESS_POS) != WAIT_DL) {
		printf("PHY is not in download mode\n");
		return -EFAULT;
	}

	return 0;
}

int download_fw_to_ram(struct phy_device *phydev, u8 data[], u32 size)
{
	u16 reg_val;
	u16 num_ports_per_dev;
	u16 byte_checksum;
	u16 section_count = 0;
	u16 max_sections;
	u16 relative_port;
	u32 image_index = 0;
	file_header_type f_header;
	int status;

	max_sections = MV_MAX_HEADERS;

	/* Size must be an even number of bytes */
	if (size % 2) {
		printf("Fw size must be even number of bytes\n");
		return -EFAULT;
	}

	/* Put PHY in download mode */
	status = put_phy_in_dl_mode(phydev);
	if (status != 0) {
		printf("Failed to download mode\n");
		return status;
	}

	/* Get number of ports of the PHY */
	reg_val = phy_read(phydev, 3, DEVICE_INFO);
	num_ports_per_dev = ((reg_val & GENMASK(9, 7)) >> 7) + 1;
	if (num_ports_per_dev == 1) {
		relative_port = 0;
	}
	else {
		printf("Incorrect num ports per device\n");
		return -EFAULT;
	}

	memset(&f_header, 0, sizeof(file_header_type));
	do {
		if (image_index > size) {
			printf("Image size and image data do not match.\n");
			return -EFAULT;
		}
		/* Now process one or more headers */
		if (get_header(&f_header, &data[image_index]) != 0) {
			printf("Header corrupted\n");
			return -EFAULT;
		}
		image_index += MV_HEADER_SIZE;

		if (!(f_header.port_skip & (1 << relative_port))) {
			ram_dnld_calc_checksums(phydev, &data[image_index],
					f_header.data_length,
					f_header.data_destination,
					&byte_checksum);

			byte_checksum = (~byte_checksum);
			if (byte_checksum != f_header.sec_checksum) {
				printf("Error downloading code\n");
				printf("Expected HDR checksum 0x%04X actual 0x%04X on section %d",
				       (int)f_header.sec_checksum,
				       (int)byte_checksum,
				       (int)section_count);
				return -EFAULT;
			}

			/* if this is an executable section, execute the code */
			if (!f_header.data_only) {
				/* Now start code which was downloaded */
				reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD,
						   BOOT_STATUS);
				reg_val |= BIT(6);
				phy_write(phydev, MV_T_UNIT_PMA_PMD, BOOT_STATUS,
					  reg_val);
				/* Give app code time to start */
				udelay(200000);

				reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD,
						   BOOT_STATUS);
				if ((reg_val & BIT(6)) == 0) {
					printf("Download app did not start.\n");
					return -EFAULT;
				}

				/* If more data to download, need to wait for
				 * current one to complete. */
				if ((f_header.next_hdr_offset != 0 &&
				     f_header.next_hdr_offset != 0xFFFFFFFF) &&
				     ((section_count+1) < max_sections)) {
					/* There's another section to process,
					 * wait until the executable is done.
					 * The code assumes the first executable
					 * is the serdes loader which sets this
					 * status when done or skipped. */
					status = wait_for_exec_done(phydev,
								    3000);
					if (status != 0) {
						printf("Exec time out\n");
						return -EFAULT;
					}

					/* reset Tunit processor and put in
					 * download mode to clear memory to
					 * prepare for next sections */
					status = put_phy_in_dl_mode(phydev);
					if (status != 0) {
						printf("Download failed\n");
						return -EFAULT;
					}
				}
			}
		}
		/* Go to the next section if there are more sections */
		if (f_header.next_hdr_offset != 0 &&
		    f_header.next_hdr_offset != 0xFFFFFFFF) {
			/* sections don't need to be sequential using
			 * this approach */
			section_count++;
			image_index = f_header.next_hdr_offset;
		}
	} while ((f_header.next_hdr_offset != 0 &&
		  f_header.next_hdr_offset != 0xFFFFFFFF) &&
		  (section_count < max_sections));

	if ((section_count >= max_sections) && (f_header.next_hdr_offset != 0 &&
	    f_header.next_hdr_offset != 0xFFFFFFFF)) {
		/* there's something wrong with the file, maximum section
		 * number reached but more sections are coming */
		printf("Max sections reached but more data is coming.\n");
		return -EFAULT;
	}

	return 0;
}

static int mv88x3310_probe(struct phy_device *phydev)
{
	int status;

	status = download_fw_to_ram(phydev, x3310_fw_image,
				    sizeof(x3310_fw_image));
	if (status != 0) {
		printf("Update image failed\n");
		return -EFAULT;
	}

	return 0;
}

int do_cunit_soft_reset(struct phy_device *phydev)
{
	u16 reg_val;
	u16 media_sel;
	u16 tunitinswreset;
	u16 tunitinwaitpowerdown;

	/* Do Cunit sw reset */
	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_PORT_CTRL);
	reg_val |= BIT(15);
	phy_write(phydev, MV_C_UNIT, CUNIT_PORT_CTRL, reg_val);

	/* Get media selection */
	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_MODE_CONFIG);
	media_sel = (reg_val & GENMASK(2, 0));
	if (media_sel == MV_MS_FBR_ONLY) {
		/* Check if Tunit is already in fiber only mode and
		 * T unit software reset is already in process */
		reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, PMA_CTRL1);
		tunitinswreset = (reg_val & BIT(15));
		reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, BIST_STATUS);
		tunitinwaitpowerdown = (reg_val & BIT(7));

		if ((tunitinswreset == 1 && tunitinwaitpowerdown == 0) ||
		    (tunitinswreset == 0 && tunitinwaitpowerdown == 1)) {
			/* Either a T unit software reset is already in
			 * progress, or T unit is in a strange state */
			return -EFAULT;
		}

		if ((tunitinswreset == 0 && tunitinwaitpowerdown == 0)) {
			/* Tunit is not in fiber only mode yet and
			 * no sw reset is in progress */
			udelay(10000);
			/* Tunit soft reset */
			reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, PMA_CTRL1);
			reg_val |= BIT(15);
			phy_write(phydev, MV_T_UNIT_PMA_PMD, PMA_CTRL1, reg_val);
			udelay(10000);
		}
		else {
			/* Tunit already in fiber only mode, i.e. both are 1 */
		}
	}

	return 0;
}

static int mv88x3310_phy_config(struct phy_device *phydev)
{
	u16 reg_val;
	int status;

	/* Delay for PHY to settle */
	udelay(300000);

	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_MODE_CONFIG);
	/* F2R_OFf */
	reg_val &= ~GENMASK(5, 4);
	/* Use the media type selected by hw pins */
	//reg_val &= ~GENMASK(2, 0);
	//reg_val |= MV_MS_AUTO_PREFER_FBR;
	phy_write(phydev, MV_C_UNIT, CUNIT_MODE_CONFIG, reg_val);

	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_PORT_CTRL);
	/* No npMediaEnergyDetect */
	reg_val &= ~BIT(10);
	/* No maxPowerTunitAMDetect */
	reg_val &= ~BIT(9);
	/* Fiber type */
	reg_val &= ~GENMASK(5, 3);
	if (phydev->interface == PHY_INTERFACE_MODE_USXGMII)
		reg_val |= (MV_FT_10GBASER << 3);
	else
		reg_val |= (MV_FT_SGMII << 3);
	phy_write(phydev, MV_C_UNIT, CUNIT_PORT_CTRL, reg_val);
	/* Do Cunit soft reset after next config */

	/* MAC type select */
	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_PORT_CTRL);
	reg_val &= ~GENMASK(2, 0);
	if (phydev->interface == PHY_INTERFACE_MODE_USXGMII)
		reg_val |= MV_MAC_TYPE_USXGMII;
	else
		reg_val |= MV_MAC_TYPE_SGMII_ANOFF;
	phy_write(phydev, MV_C_UNIT, CUNIT_PORT_CTRL, reg_val);

	/* MAC mode config */
	reg_val = phy_read(phydev, MV_C_UNIT, CUNIT_MODE_CONFIG);
	/* MAC intf power down */
	reg_val |= BIT(3);
	/* Snoop off */
	reg_val &= ~GENMASK(9, 8);
	reg_val |= (MV_MAC_SNOOP_OFF << 8);
	/* Select active lane 0 */
	reg_val &= ~BIT(10);
	/* MAC link down speed set to 10G */
	reg_val &= ~GENMASK(7, 6);
	reg_val |= MV_MAX_MAC_SPEED_10G;
	phy_write(phydev, MV_C_UNIT, CUNIT_MODE_CONFIG, reg_val);

	/* Select max speed */
	reg_val = phy_read(phydev, MV_M_UNIT, COMPHY_ACCESS_CTRL);
	reg_val &= ~GENMASK(1, 0);
	reg_val |= MV_MAX_MAC_SPEED_10G;
	phy_write(phydev, MV_M_UNIT, COMPHY_ACCESS_CTRL, reg_val);

	/* Do sw reset */
	status = do_cunit_soft_reset(phydev);
	if (status != 0)
		return status;
	/* Do second sw reset if setting link down speed */
	status = do_cunit_soft_reset(phydev);
	if (status != 0)
		return status;

	/* Disalbe USXGMII AN by setting the bit to 1 */
	reg_val = phy_read(phydev, MV_T_UNIT_PMA_PMD, USXGMII_AN_CNTL);
	reg_val |= BIT(0);
	phy_write(phydev, MV_T_UNIT_PMA_PMD, USXGMII_AN_CNTL, reg_val);

	/* Soft reset H-unit */
	reg_val = phy_read(phydev, MV_H_UNIT, MV10GBR_PCS_CONTROL);
	reg_val |= BIT(15);
	phy_write(phydev, MV_H_UNIT, MV10GBR_PCS_CONTROL, reg_val);

	/* Soft reset X-unit */
	reg_val = phy_read(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL);
	reg_val |= BIT(15);
	phy_write(phydev, MV_X_UNIT, MV10GBR_PCS_CONTROL, reg_val);

	return 0;
}

static int mv88x3310_phy_startup(struct phy_device *phydev)
{
	bool h_link_status = false;
	bool l_link_status = false;
	u16 reg_val;
	int retries;
	u16 speed;
	u16 duplex;
	int i;

	phydev->link = 0;
	speed = SPEED_UNKNOWN;
	duplex = DUPLEX_HALF;

	/* First read is latched status */
	reg_val = phy_read(phydev, MV_X_UNIT, MV10GBR_PCS_STATUS1);
	retries = 2000;
	for (i = 0; i < retries; i++) {
		reg_val = phy_read(phydev, MV_X_UNIT, MV10GBR_PCS_STATUS1);
		if (reg_val & BIT(2)) {
			l_link_status = true;
			break;
		}
		udelay(100);
	}

	/* If fiber link is down, check copper link status */
	if (l_link_status) {
		printf("Use fiber link\n");
		speed = SPEED_10000;
		duplex = DUPLEX_FULL;
	}
	else {
		/* First read is latched status */
		reg_val = phy_read(phydev, MV_X_UNIT, COPPER_STATUS);
		retries = 2000;
		for (i = 0; i < retries; i++) {
			reg_val = phy_read(phydev, MV_X_UNIT, COPPER_STATUS);
			if ((reg_val & COPPER_LINK_UP) &&
			    (reg_val & COPPER_SPEED_DUP)) {
				l_link_status = true;
				break;
			}
			udelay(100);
		}
		if (l_link_status) {
			printf("Use copper link\n");
			if ((reg_val & COPPER_SPEED_MASK) == COPPER_HI_SPEED) {
				u16 hi_speed = (reg_val & COPPER_HI_SPEED_MASK);
				if (hi_speed == COPPER_10G) {
					speed = SPEED_10000;
					duplex = DUPLEX_FULL;
				}
				else if (hi_speed == COPPER_2G5) {
					speed = SPEED_2500;
					duplex = DUPLEX_FULL;
				}
				else if (hi_speed == COPPER_5G) {
					speed = SPEED_5000;
					duplex = DUPLEX_FULL;
				}
				else {
					printf("Unsupported speed\n");
				}
			}
			else if ((reg_val & COPPER_HI_SPEED) == COPPER_1G) {
				speed = SPEED_1000;
				duplex = DUPLEX_FULL;
			}
			else {
				printf("Unsupported speed\n");
			}
		}
		else {
			printf("PHY line side link down\n");
		}
	}

	/* Check host side link status */
	if (phydev->interface == PHY_INTERFACE_MODE_USXGMII) {
		/* First read is latched status */
		reg_val = phy_read(phydev, MV_H_UNIT, USXGMII_AUTO_NEG_BASE);
		reg_val = phy_read(phydev, MV_H_UNIT, USXGMII_AUTO_NEG_BASE);
		if ((reg_val & USXGMII_LINK_UP) == 0) {
			printf("PHY host side link down\n");
			h_link_status = false;
		}
		else {
			h_link_status = true;
		}
	}
	else {
		/* For SGMII mode, when the speed is switched between 1G and
		 * 2.5G, SoC serdes speed must be updated before PHY host
		 * side link can come up. So don't check host side link.
		 */
		h_link_status = true;
	}

	if (h_link_status == true && l_link_status == true)
		phydev->link = 1;

	/* For loopback test, fake link status in case link is not up */
	if (loopback_test == PHY_HOST_LOOPBACK) {
		if (phydev->link == 0) {
			phydev->link = 1;
			printf("Fake link status in loopback test\n");
		}
	}

	if (phydev->link) {
		phydev->speed = speed;
		phydev->duplex = duplex;
	}
	else {
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_HALF;
	}

	return 0;
}

static struct phy_driver mv88x3310p_b0_driver = {
	.name = "Marvell MV88X3310P_B0",
	.uid = 0x2b09ae,
	.mask = 0xffffff,
	.features = PHY_10G_FEATURES,
	.probe = mv88x3310_probe,
	.config = mv88x3310_phy_config,
	.startup = mv88x3310_phy_startup,
	.shutdown = &genphy_shutdown,
};

static struct phy_driver mv88x3310p_a1_driver = {
	.name = "Marvell MV88X3310P_A1",
	.uid = 0x2b09ab,
	.mask = 0xffffff,
	.features = PHY_10G_FEATURES,
	.probe = mv88x3310_probe,
	.config = mv88x3310_phy_config,
	.startup = mv88x3310_phy_startup,
	.shutdown = &genphy_shutdown,
};

int phy_mv88x3310_init(void)
{
	phy_register(&mv88x3310p_b0_driver);
	phy_register(&mv88x3310p_a1_driver);

	return 0;
}

