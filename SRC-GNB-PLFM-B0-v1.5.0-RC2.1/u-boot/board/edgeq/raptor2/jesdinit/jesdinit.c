/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */
#include <linux/delay.h>
#include <asm/io.h>
#include <log.h>
#include "jesdinit.h"
#include "../raptor2_eeprom.h"

/* Global variables */
static u32 link_sel = 0x3;
static u32 NUM_LANES_L3L4 = 0x11;
static u32 divider_value;
static const u32 tx_clk_serdes_clk_src = 0x10;
static const u32 lane_mux_sel_val = 0x76543210;
static const u32 ILPBK;
static const u32 term_cfg;
static u32 link3_tx_clk_sel_word;
static u32 link4_tx_clk_sel_word;

static u32 link3_lane0_pol;
static u32 link3_lane1_pol;
static u32 link3_lane2_pol;
static u32 link3_lane3_pol;
static u32 link4_lane0_pol;
static u32 link4_lane1_pol;
static u32 link4_lane2_pol;
static u32 link4_lane3_pol;
//link_speed - 0x2 for 16.22 lane rate, 0x0 for 8.11 lane rate for link3
static u32 link3_lane_rate = 0x2;
//link_speed - 0x2 for 16.22 lane rate, 0x0 for 8.11 lane rate for link4
static u32 link4_lane_rate = 0x2;
struct rfic_info rf_info;

void cpu_wr_w(u32 addr, u32 wdata)
{
	writel(wdata, (u64)addr);
}

u32 cpu_rd_w(u32 addr)
{
	u32 data;

	data = readl((u64)addr);
	return data;
}

u64 cpu_rd(u32 cpu_id, u32 addr)
{
	u64 data;

	data = readq((u64)addr);
	return data;
}

void cpu_rmw_w(u32 addr, u32 mask, u32 wdata)
{
	u32 data;

	data = readl((u64)addr);
	wdata = (wdata & mask) | (data & ~mask);
	writel(wdata, (u64)addr);
}

void link34_write1(u32 link_sel, u32 offset, u32 data)
{
	u32 addr;

	if (link_sel == JESD_LINK_3) {
		addr = JESD_LINK3_IO_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
	if (link_sel == JESD_LINK_4) {
		addr = JESD_LINK4_IO_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
	if (link_sel == JESD_LINK_34) {
		addr = JESD_LINK3_IO_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
		addr = JESD_LINK4_IO_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
}

void link34_write2(u32 link_sel, u32 offset, u32 data)
{
	u32 addr;

	if (link_sel == JESD_LINK_3) {
		addr = JESD_LINK3_SERDES_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
	if (link_sel == JESD_LINK_4) {
		addr = JESD_LINK4_SERDES_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
	if (link_sel == JESD_LINK_34) {
		addr = JESD_LINK3_SERDES_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
		addr = JESD_LINK4_SERDES_CTRL_BASE + offset;
		cpu_wr_w(addr, data);
	}
}

void jesd_phy_pwrup_iossinit(void)
{
	ndelay(100);
	cpu_rmw_w(JESD_IOSS_TOP_MISC_13, 0x0000FFFF, 0x00004440);
	cpu_wr_w(JESD_IOSS_TOP_MISC_13, 0x4440);
	ndelay(100);
	cpu_rmw_w(JESD_IOSS_TOP_CFG_13, 0xFFFFFFFF, 0x05050503);
	ndelay(100);
	cpu_rmw_w(JESD_IOSS_TOP_CFG_9, 0x00000FFF, 0xC01);
	ndelay(100);
	cpu_rmw_w(JESD_IOSS_TOP_CFG_6, 0x01000017, 0x01000017);
	cpu_rmw_w(JESD_IOSS_TOP_CFG_7, 0x01010101, 0x01010101);
	cpu_rmw_w(JESD_IOSS_TOP_CFG_16, 0xFFFFFFFF, 0x0);
	cpu_rmw_w(JESD_IOSS_TOP_MISC_16, 0xFFFFFFFF, 0x0);

	if (link_sel == JESD_LINK_3)
		cpu_rmw_w(JESD_IOSS_TOP_CFG_12, 0x01010101, 0x01000001);
	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)
		cpu_rmw_w(JESD_IOSS_TOP_CFG_12, 0x01010101, 0x01000101);
	cpu_rmw_w(JESD_IOSS_TOP_MISC_12, 0x00000001, 0x00000001);
	ndelay(100);
	ndelay(100);

	if (link_sel == JESD_LINK_3)
		cpu_rmw_w(JESD_CCIX_BRINGUP_CFG_18, 0x00F00F00, 0x00400000);
	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)
		cpu_rmw_w(JESD_CCIX_BRINGUP_CFG_18, 0x00F0F000, 0x00403000);
	ndelay(100);
	cpu_wr_w(JESD_TX_CFG_0, 0x11000000);
	cpu_wr_w(JESD_RX_CFG_0, 0x0);
	cpu_rmw_w(JESD_TX_CLK_CHAR_DIV_RATIO, 0x00000033, divider_value);
	cpu_rmw_w(JESD_TX_SERDES_CLK_SRC_SELECT, 0x00000011, tx_clk_serdes_clk_src);

	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34)
		link34_write2(JESD_LINK_3, 0x0328, link3_tx_clk_sel_word);
	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)
		link34_write2(JESD_LINK_4, 0x0328, link4_tx_clk_sel_word);
	cpu_rmw_w(JESD_TX_SERDES_LANE_MUX_CFG, 0xFFFFFFFF, lane_mux_sel_val);
	cpu_rmw_w(JESD_TX_SERDES_MISC_0, 0xFFFFFFFF, 0x00000000);
	cpu_rmw_w(JESD_RX_SERDES_LANE_MUX_CFG, 0xFFFFFFFF, lane_mux_sel_val);
	cpu_rmw_w(JESD_RX_MISC_0, 0xFFFFFFFF, 0x00000000);

	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
		link34_write1(link_sel, 0x0078, 0x4000101);
		ndelay(100);
		link34_write1(link_sel, 0x0080, 0x0101);
		ndelay(100);
	}

	if (link_sel == JESD_LINK_3) {
		link34_write1(link_sel, 0x0078, 0x4001101);
		ndelay(100);
		link34_write1(link_sel, 0x0080, 0x1101);
		ndelay(100);
	}
	printf("JESD_INIT: ioss setup done, phy power up sequence starts\r\n");
}

void jesd_phy_pwrup_part1(void)
{
	u32 rdata;

	link34_write2(link_sel, 0x02C4, 0x5);
	link34_write2(link_sel, 0x0020, 0x0);
	link34_write2(link_sel, 0x0024, 0x01000100);
	link34_write1(link_sel, 0x0028, 0x00010001);
	link34_write1(link_sel, 0x002C, 0x00010001);
	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
		link34_write1(link_sel, 0x0128, (0x01400130 | (link3_lane0_pol << 12)));
		link34_write1(link_sel, 0x0148, (0x01400130 | (link3_lane1_pol << 12)));
		link34_write1(link_sel, 0x0168, (0x01400130 | (link3_lane2_pol << 12)));
		link34_write1(link_sel, 0x0188, (0x01400130 | (link3_lane3_pol << 12)));
		link34_write1(JESD_LINK_3, 0x00A0, (0x01500413 + (link3_lane_rate << 12)));
		link34_write1(JESD_LINK_3, 0x00C8, (0x01500413 + (link3_lane_rate << 12)));
		link34_write1(JESD_LINK_3, 0x00F0, (0x01500413 + (link3_lane_rate << 12)));
		link34_write1(JESD_LINK_3, 0x0118, (0x01500413 + (link3_lane_rate << 12)));
		link34_write1(JESD_LINK_3, 0x012C, (0x00088000 + (link3_lane_rate << 4)));
		link34_write1(JESD_LINK_3, 0x014C, (0x00088000 + (link3_lane_rate << 4)));
		link34_write1(JESD_LINK_3, 0x016C, (0x00088000 + (link3_lane_rate << 4)));
		link34_write1(JESD_LINK_3, 0x018C, (0x00088000 + (link3_lane_rate << 4)));
		link34_write1(JESD_LINK_3, 0x01A8, (link3_lane_rate * 0x01010101));
	}
	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
		link34_write1(link_sel, 0x0128, (0x01400130 | (link4_lane0_pol << 12)));
		link34_write1(link_sel, 0x0148, (0x01400130 | (link4_lane1_pol << 12)));
		link34_write1(link_sel, 0x0168, (0x01400130 | (link4_lane2_pol << 12)));
		link34_write1(link_sel, 0x0188, (0x01400130 | (link4_lane3_pol << 12)));
		link34_write1(JESD_LINK_4, 0x00A0, (0x01500413 + (link4_lane_rate << 12)));
		link34_write1(JESD_LINK_4, 0x00C8, (0x01500413 + (link4_lane_rate << 12)));
		link34_write1(JESD_LINK_4, 0x00F0, (0x01500413 + (link4_lane_rate << 12)));
		link34_write1(JESD_LINK_4, 0x0118, (0x01500413 + (link4_lane_rate << 12)));
		link34_write1(JESD_LINK_4, 0x012C, (0x00088000 + (link4_lane_rate << 4)));
		link34_write1(JESD_LINK_4, 0x014C, (0x00088000 + (link4_lane_rate << 4)));
		link34_write1(JESD_LINK_4, 0x016C, (0x00088000 + (link4_lane_rate << 4)));
		link34_write1(JESD_LINK_4, 0x018C, (0x00088000 + (link4_lane_rate << 4)));
		link34_write1(JESD_LINK_4, 0x01A8, (link4_lane_rate * 0x01010101));
	}
	link34_write1(link_sel, 0x0020, 0x11111110);
	link34_write1(link_sel, 0x0024, 0x00000001);

	//TODO: Add for link4 etc
	do {
		ndelay(100);
		printf("JESD_INIT: Waiting for sram init to finish for link\r\n");
		rdata = cpu_rd_w(0x3cb60400);
	} while ((rdata & 0x1) == 0);
	ndelay(100);
}

void jesd_phy_pwrup_part2(void)
{
	u32 l3_lanes = ((NUM_LANES_L3L4 & 0xF0) >> 4);
	u32 l4_lanes = ((NUM_LANES_L3L4 & 0x0F));

	if (ILPBK == 0x1) {
		link34_write1(link_sel, 0x0028, 0x01010081);
		link34_write1(link_sel, 0x002C, 0x00810081);
	}

	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_17, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_22, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_27, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_32, 0x01700000, ((1 << 24) + (term_cfg << 20)));
	}
	if (link_sel == 0x4 || link_sel == 0x34) {
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_17, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_22, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_27, 0x01700000, ((1 << 24) + (term_cfg << 20)));
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_32, 0x01700000, ((1 << 24) + (term_cfg << 20)));
	}

	ndelay(100);
	link34_write2(link_sel, 0x0020, 0x0);
	link34_write2(link_sel, 0x0024, 0x1010100);
	ndelay(100);
	link34_write1(link_sel, 0x0020, 0x00000000);
	link34_write1(link_sel, 0x0024, 0x0);
	ndelay(100);

	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
		cpu_wr_w(JESD_LINK3_IO_CTRL_CFG_50, 0x01010101);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_16, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_21, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_26, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_31, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_33, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_37, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_41, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_45, 0x00010030, 0x00010000);
		ndelay(200);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_16, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_21, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_26, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_31, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_33, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_37, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_41, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK3_IO_CTRL_CFG_45, 0x00010030, 0x00000000);
		ndelay(200);
		cpu_wr_w(JESD_LINK3_IO_CTRL_CFG_36, 0x01233014);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_36, 0x01000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_CFG_40, 0x01233014);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_40, 0x01000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_CFG_44, 0x01233014);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_44, 0x01000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_CFG_48, 0x01233014);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_48, 0x01000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_16, 0x10000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_21, 0x10000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_26, 0x10000000);
		cpu_wr_w(JESD_LINK3_IO_CTRL_MISC_31, 0x10000000);
		ndelay(200);
		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
	}

	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
		cpu_wr_w(JESD_LINK4_IO_CTRL_CFG_50, 0x01010101);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_16, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_21, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_26, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_31, 0x10000003, 0x10000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_33, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_37, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_41, 0x00010030, 0x00010000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_45, 0x00010030, 0x00010000);
		ndelay(200);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_16, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_21, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_26, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_31, 0x10000003, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_33, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_37, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_41, 0x00010030, 0x00000000);
		cpu_rmw_w(JESD_LINK4_IO_CTRL_CFG_45, 0x00010030, 0x00000000);
		ndelay(200);
		cpu_wr_w(JESD_LINK4_IO_CTRL_CFG_36, 0x01233014);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_36, 0x01000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_CFG_40, 0x01233014);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_40, 0x01000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_CFG_44, 0x01233014);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_44, 0x01000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_CFG_48, 0x01233014);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_48, 0x01000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_16, 0x10000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_21, 0x10000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_26, 0x10000000);
		cpu_wr_w(JESD_LINK4_IO_CTRL_MISC_31, 0x10000000);
		ndelay(200);
		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
	}

	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
		if (l3_lanes == 1) {
			link34_write1(JESD_LINK_3, 0x0020, 0x11011100);
			link34_write1(JESD_LINK_3, 0x0024, 0x00000001);
		}
		if (l3_lanes == 2) {
			link34_write1(JESD_LINK_3, 0x0020, 0x10011000);
			link34_write1(JESD_LINK_3, 0x0024, 0x00000001);
		}
		if (l3_lanes == 3) {
			link34_write1(JESD_LINK_3, 0x0020, 0x0010000);
			link34_write1(JESD_LINK_3, 0x0024, 0x00000001);
		}
		if (l3_lanes == 4) {
			link34_write1(JESD_LINK_3, 0x0020, 0x00000000);
			link34_write1(JESD_LINK_3, 0x0024, 0x00000000);
		}
	}

	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
		if (l4_lanes == 1) {
			link34_write1(JESD_LINK_4, 0x0020, 0x11011100);
			link34_write1(JESD_LINK_4, 0x0024, 0x00000001);
		}
		if (l4_lanes == 2) {
			link34_write1(JESD_LINK_4, 0x0020, 0x10011000);
			link34_write1(JESD_LINK_4, 0x0024, 0x00000001);
		}
		if (l4_lanes == 3) {
			link34_write1(JESD_LINK_4, 0x0020, 0x0010000);
			link34_write1(JESD_LINK_4, 0x0024, 0x00000001);
		}
		if (l4_lanes == 4) {
			link34_write1(JESD_LINK_4, 0x0020, 0x00000000);
			link34_write1(JESD_LINK_4, 0x0024, 0x00000000);
		}
	}

	ndelay(200);

	if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c45103c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c45123c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c45143c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK3_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c45163c)\r\n");
		ndelay(200);
	}
	if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c46103c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c46123c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c46143c)\r\n");
		ndelay(200);

		cpu_wr_w(JESD_LINK4_SERDES_CTRL_CFG_85, 0x00001000);
		ndelay(100);
		//printf(" Cheking rx0_Ack to de-asserted & rx_valid to go high (mdw 0x3c46163c)\r\n");
		ndelay(200);
	}
	printf("phy powerup ends\r\n");
}

void cr_para_wr(u32 addr, u32 data, u32 cr_para_offset, u32 cr_para_lsb)
{
	u32 wr_addr;
	u32 addr_lsb;
	u32 wdata_lsb;

	wr_addr = cr_para_offset + (addr & 0xFFFC);
	addr_lsb = addr & 0x3;
	wdata_lsb = 0x1000 | (addr_lsb << 8);

	//printf("**INFO**:: Writing Addr=0x%08x, Data=0x%08x\r\n", cr_para_lsb, wdata_lsb);
	cpu_wr_w(cr_para_lsb, wdata_lsb);
	//printf("**INFO**:: Writing Addr=0x%08x, Data=0x%08x\r\n", wr_addr, data);
	cpu_wr_w(wr_addr    , data     );
	//printf("**INFO**:: CR-PARA Write :: Addr=0x%04x, Data=0x%04x\r\n",addr,data);
}

void jesd_fw_load(void)
{
	u32 i = 0;
	u32 rdata;

	printf("u-boot:JESD_INIT: ******** L4 E32 FW load Start ********\r\n");

	for (i = 0; i < (FW_SIZE / 2 + 1); i++) {

		rdata = top_fw[i];

		if (i == (FW_SIZE / 2)) {
			if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34)
				cr_para_wr(0x0101, rdata & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
			if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)
				cr_para_wr(0x0101, rdata & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
		} else {
			if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
				cr_para_wr(FW_START + 2 * i, rdata & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
				cr_para_wr(FW_START + 2 * i + 1, (rdata >> 16) & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
			}
			if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
				cr_para_wr(FW_START + 2 * i, rdata & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
				cr_para_wr(FW_START + 2 * i + 1, (rdata >> 16) & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
			}
		}
	}

	for (i = 0; i < (FW_SIZE / 2 + 1); i++) {

		rdata = bot_fw[i];

		if (i == (FW_SIZE / 2)) {
			if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34)
				cr_para_wr(0x0101, rdata & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
			if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)
				cr_para_wr(0x0101, rdata & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
		} else {
			if (link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) {
				cr_para_wr(FW_START + 2 * i, rdata & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
				cr_para_wr(FW_START + 2 * i + 1, (rdata >> 16) & 0xFFFF,
						JESD_LINK3_CR_PARA_OFFSET, JESD_LINK3_CR_PARA_LSB);
			}
			if (link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) {
				cr_para_wr(FW_START + 2 * i, rdata & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
				cr_para_wr(FW_START + 2 * i + 1, (rdata >> 16) & 0xFFFF,
						JESD_LINK4_CR_PARA_OFFSET, JESD_LINK4_CR_PARA_LSB);
			}
		}
	}

	printf("u-boot:JESD_INIT: ******** L4 E32 FW load Done ********\r\n");
}

void jesd_phy_pwrup(void)
{
	u8 num_lanes_3, num_lanes_4;
	u32 board_type = 0;

	if ((rf_info.rf1_card_type & RF_JESDINIT_BYP_MASK) ||
	    (rf_info.rf2_card_type & RF_JESDINIT_BYP_MASK)) {
		printf("u-boot:SKIPPING JESD_INIT based on EEPROM config \r\n");
		return;
	}

	/**
	 * link_sel:
	 * link_sel is 0x3 if RF1 card type is 1 and RF2 card type is 0;
	 * link_sel is 0x4 if RF1 card type is 0 and RF2 card type is 1;
	 * link_sel is 0x34 if RF1 and RF2 card type both are 1
	 */
	if (((rf_info.rf1_card_type & RF_CARD_TYPE_MASK) == RF_CARD_ADI) &&
	    ((rf_info.rf2_card_type & RF_CARD_TYPE_MASK) == RF_CARD_METANOIA)) {
		link_sel = JESD_LINK_3;
	} else if (((rf_info.rf1_card_type & RF_CARD_TYPE_MASK) == RF_CARD_METANOIA) &&
		((rf_info.rf2_card_type & RF_CARD_TYPE_MASK) == RF_CARD_ADI)) {
		link_sel = JESD_LINK_4;
	} else if (((rf_info.rf1_card_type & RF_CARD_TYPE_MASK) == RF_CARD_METANOIA) &&
		((rf_info.rf2_card_type & RF_CARD_TYPE_MASK) == RF_CARD_METANOIA)) {
		link_sel = JESD_LINK_34;
	} else if (((rf_info.rf1_card_type & RF_CARD_TYPE_MASK) == RF_CARD_ADI) &&
		((rf_info.rf2_card_type & RF_CARD_TYPE_MASK) == RF_CARD_ADI)) {
		link_sel = JESD_LINK_34;
	} else {
		debug("u-boot:JESD: opting to default Link3\r\n");
		link_sel = JESD_LINK_3;
	}

	board_type = get_board_name_value();
	if ((board_type == BOARD_HAWK) || (board_type == BOARD_HAWKV2)
			|| (board_type == BOARD_HAWKV3) || (board_type == BOARD_HAWKV4) || (board_type == BOARD_HAWKV5)) {
		if (link_sel != JESD_LINK_3)
			debug("u-boot:JESD: overriding link select\r\n");
		link_sel = JESD_LINK_3;
	}

	debug("u-boot:JESD: link select is 0x%X\r\n", link_sel);

	/**
	 *  lane_rate:
	 *  lane_rate = 0x2 if link speed is 16G and 0x0 if link speed is 8G
	 *  lane_rate is to be mapped with Link Speed's 2 LSB bits
	 */
	link3_lane_rate = rf_info.link_speed & 0x03;
	link4_lane_rate = (rf_info.link_speed >> 4) & 0x03;
	debug("u-boot:JESD: Link3 Lane rate:%x\r\nLink4 Lane rate:%x\r\n",
			link3_lane_rate, link4_lane_rate);

	/**
	 * Take number of lanes from links_speed's 2 MSB bits
	 */
	num_lanes_3 = ((rf_info.link_speed & 0x0C) >> 2) + 1;
	num_lanes_4 = (((rf_info.link_speed >> 4) & 0x0C) >> 2) + 1;

	NUM_LANES_L3L4 = (num_lanes_3 << 4) | num_lanes_4;

	/**
	 * NUM_LANES_L3L4:
	 * lower 4 bits is 1 if link_sel is 0x4 or 0x34 and lane_rate is 0x2;
	 * lower 4 bits is 2 if link_sel is 0x4 or 0x34 and lane_rate is 0x0
	 * next 4 bits is 1 if link_sel is 0x3 or 0x34 and lane_rate is 0x2;
	 * next 4 bits is 2 if link_sel is 0x3 or 0x34 and lane_rate is 0x0
	 * so it can take following values: 0x11, 0x22, 0x21 or 0x12
	 */
	if ((link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) &&
			link4_lane_rate == JESD_LINK_SPEED_16G) {
		divider_value = 0x0;
		link3_tx_clk_sel_word = 0x0;
		link4_tx_clk_sel_word = 0x0;
	} else if ((link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34) &&
			link4_lane_rate == JESD_LINK_SPEED_8G) {
		divider_value = 0x11;
		link3_tx_clk_sel_word = 0x02;
		link4_tx_clk_sel_word = 0x02;
	} else if ((link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) &&
			link3_lane_rate == JESD_LINK_SPEED_16G) {
		divider_value = 0x10;
		link3_tx_clk_sel_word = 0x0;
		link4_tx_clk_sel_word = 0x02;
	} else if ((link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34) &&
			link3_lane_rate == JESD_LINK_SPEED_8G) {
		divider_value = 0x01;
		link3_tx_clk_sel_word = 0x02;
		link4_tx_clk_sel_word = 0x0;
	} else {
		debug("u-boot:JESD: switching clock configs to default values\r\n");
		divider_value = 0x0;
		link3_tx_clk_sel_word = 0x0;
		link4_tx_clk_sel_word = 0x0;
	}
	debug("u-boot:JESD: NUM_LANES_L3L4 is 0x%X\r\n"
		"u-boot:JESD: divider_value is 0x%X\r\n"
		"u-boot:JESD: link3_tx_clk_sel_word 0x%X\r\n"
		"u-boot:JESD: link4_tx_clk_sel_word 0x%X\r\n",
		NUM_LANES_L3L4, divider_value, link3_tx_clk_sel_word, link4_tx_clk_sel_word);

	/**
	 * try to set lane polarities
	 **/
	if ((link_sel == JESD_LINK_3 || link_sel == JESD_LINK_34)) {
		// take first nibble for u-boot
		link3_lane0_pol = rf_info.link3_lane_polarity & 0x1; // 0
		link3_lane1_pol = (rf_info.link3_lane_polarity >> 1) & 0x1; // 1
		link3_lane2_pol = (rf_info.link3_lane_polarity >> 2) & 0x1; // 0
		link3_lane3_pol = (rf_info.link3_lane_polarity >> 3) & 0x1; // 1
		debug("u-boot:JESD: Link3 polarities are:\r\n"
				"link3_lane0_pol:0x%x\r\n"
				"link3_lane1_pol:0x%x\r\n"
				"link3_lane2_pol:0x%x\r\n"
				"link3_lane3_pol:0x%x\r\n",
				link3_lane0_pol, link3_lane1_pol,
				link3_lane2_pol, link3_lane3_pol);
	}
	if ((link_sel == JESD_LINK_4 || link_sel == JESD_LINK_34)) {
		// take first nibble for u-boot
		link4_lane0_pol = rf_info.link4_lane_polarity & 0x1; // 0
		link4_lane1_pol = (rf_info.link4_lane_polarity >> 1) & 0x1; // 1
		link4_lane2_pol = (rf_info.link4_lane_polarity >> 2) & 0x1; // 0
		link4_lane3_pol = (rf_info.link4_lane_polarity >> 3) & 0x1; // 1
		debug("u-boot:JESD: Link4 polarities are:\r\n"
				"link4_lane0_pol:0x%x\r\n"
				"link4_lane1_pol:0x%x\r\n"
				"link4_lane2_pol:0x%x\r\n"
				"link4_lane3_pol:0x%x\r\n",
				link4_lane0_pol, link4_lane1_pol,
				link4_lane2_pol, link4_lane3_pol);
	}
	jesd_phy_pwrup_iossinit();
	jesd_phy_pwrup_part1();
	jesd_fw_load();
	jesd_phy_pwrup_part2();
}
