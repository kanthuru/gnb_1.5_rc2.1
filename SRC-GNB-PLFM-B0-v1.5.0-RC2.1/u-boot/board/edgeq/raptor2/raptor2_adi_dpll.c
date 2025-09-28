/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#include <common.h>
#include <asm/io.h>
#include <spi.h>
#include "jesdinit/jesdinit.h"
#include "raptor2_eeprom.h"

void adi_dpll_spi_write(struct spi_slave *slave, const uint8_t tx_data[])
{
	int ret = 0;

	ret = spi_xfer(slave, 3 * 8, tx_data, NULL, SPI_XFER_ONCE);
	if (ret)
		printf("ADI:DPLL: spi_xfer fail. Return code %d\n", ret);
}

void adi_dpll_spi_read(struct spi_slave *slave, const uint8_t tx_data[], uint8_t rx_data[])
{
	int ret = 0;

	ret = spi_write_then_read(slave, tx_data, 2, NULL, rx_data, 1);
	if (ret)
		printf("ADI:DPLL: spi_xfer fail. Return code %d\n", ret);
}

int adi_dpll_spi_init(struct spi_slave **slave, struct udevice **dev, int bus)
{
	int ret = 0;

	ret = spi_get_bus_and_cs(bus, 0, 500000, 0, "spi_generic_drv",
			"atcspi200_spi", dev, slave);
	if (ret) {
		printf("ADI:DPLL: Unable to get bus & cs. Return code %d\n", ret);
		goto error;
	}

	ret = spi_claim_bus(*slave);
	if (ret) {
		printf("ADI:DPLL: Unable to claim bus. Return code %d\n", ret);
		spi_release_bus(*slave);
		goto error;
	}
error:
	return ret;
}

void adi_dpll_spi_deinit(struct spi_slave *slave, struct udevice *dev)
{
	spi_release_bus(slave);
	spi_free_slave(slave);
}

void adi_dpll_program(struct spi_slave *slave)
{
	adi_dpll_spi_write(slave, (u8[]){ 0x00, 0x00, 0x3C });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x00, 0x02 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x01, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x02, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x03, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x04, 0x02 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x05, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x06, 0x0A });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x07, 0x03 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x08, 0x28 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x09, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x0A, 0x02 });
	adi_dpll_spi_write(slave, (u8[]){ 0x01, 0x0B, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x00, 0xE6 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x01, 0x87 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x02, 0x03 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x04, 0x03 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x05, 0x2A });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x06, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x08, 0x09 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x09, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x07, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x02, 0x03, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x00, 0x40 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x01, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x02, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x03, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x04, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x05, 0x04 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x06, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x07, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x08, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x09, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0A, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0B, 0x04 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0C, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0D, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0E, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x0F, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x10, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x11, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x12, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x13, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x14, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x15, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x16, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x17, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x18, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x19, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1A, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1B, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1C, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1D, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1E, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x1F, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x20, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x21, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x22, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x23, 0x13 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x24, 0x40 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x25, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x26, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x27, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x28, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x29, 0x04 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x2A, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x2B, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x2C, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x2D, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x03, 0x2E, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x04, 0x00, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x04, 0x01, 0x02 });
	adi_dpll_spi_write(slave, (u8[]){ 0x04, 0x02, 0x00 });
	adi_dpll_spi_write(slave, (u8[]){ 0x04, 0x03, 0x91 });
	adi_dpll_spi_write(slave, (u8[]){ 0x04, 0x04, 0x04 });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x00, 0x10 });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x01, 0xF4 });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x02, 0x0F });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x03, 0xFF });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x04, 0xFF });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x05, 0x07 });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x06, 0x01 });
	adi_dpll_spi_write(slave, (u8[]){ 0x05, 0x07, 0x0C });
	adi_dpll_spi_write(slave, (u8[]){ 0x00, 0x0F, 0x01 });
}

