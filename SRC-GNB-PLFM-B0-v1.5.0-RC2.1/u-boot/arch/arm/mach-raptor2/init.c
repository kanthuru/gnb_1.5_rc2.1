/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright 2020 EdgeQ 
 */

#include <common.h>
#include <dm.h>
#include <spl.h>
#include <init.h>
#include <asm/armv8/mmu.h>
#include <edgeq_eeprom_data.h>

DECLARE_GLOBAL_DATA_PTR;
#define EEPROM_BUFFER_OFFSET    0xFD0000
#define EEPROM_BUFFER_ADDR      (CONFIG_SYS_SDRAM_BASE + EEPROM_BUFFER_OFFSET)

#define DDR_SIZE_8GB	8
#define DDR_SIZE_16GB	16
#define DDR_SIZE_32GB	32

int print_cpuinfo(void)
{
        return 0;
}

int dram_init(void)
{
	struct board_configs *board_configs;
	board_configs = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs);

	if(((board_configs->ddr_config & 0x3F) == 0x0) || ((board_configs->ddr_config & 0x3F) == DDR_SIZE_8GB))
		gd->ram_size = SYS_SDRAM_SIZE_8GB;
	else if ((board_configs->ddr_config & 0x3F) == DDR_SIZE_16GB)
		gd->ram_size = SYS_SDRAM_SIZE_16GB;
	else if ((board_configs->ddr_config & 0x3F) == DDR_SIZE_32GB)
		gd->ram_size = SYS_SDRAM_SIZE_32GB;

        return 0;
}

int dram_init_banksize(void)
{
	struct board_configs *board_configs;
	board_configs = &(((struct eeprom_data *)EEPROM_BUFFER_ADDR)->board_configs);

	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;

	if (((board_configs->ddr_config & 0x3F) == 0x0) || ((board_configs->ddr_config & 0x3F) == DDR_SIZE_8GB))
		gd->bd->bi_dram[0].size = SYS_SDRAM_SIZE_8GB;
	else if ((board_configs->ddr_config & 0x3F) == DDR_SIZE_16GB)
		gd->bd->bi_dram[0].size = SYS_SDRAM_SIZE_16GB;
	else if ((board_configs->ddr_config & 0x3F) == DDR_SIZE_32GB)
		gd->bd->bi_dram[0].size = SYS_SDRAM_SIZE_32GB;

	return 0;
}
