/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#ifndef _RAPTOR2_ADI_DPLL_H_
#define _RAPTOR2_ADI_DPLL_H_

int adi_dpll_spi_init(struct spi_slave **slave, struct udevice **dev, int bus);
void adi_dpll_spi_deinit(struct spi_slave *slave, struct udevice *dev);
void adi_dpll_spi_write(struct spi_slave *slave, const uint8_t tx_data[]);
void adi_dpll_spi_read(struct spi_slave *slave, const uint8_t tx_data[], uint8_t rx_data[]);
void adi_dpll_program(struct spi_slave *slave);

#endif
