/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#include <common.h>
#include <command.h>
#include <i2c.h>
#include <malloc.h>
#include <linux/ctype.h>
#include <dm.h>
#include <linux/delay.h>
#include <log.h>

#include "jesdinit/jesdinit.h"
#include "raptor2_eeprom.h"
#include "raptor2_avs.h"

#define AVS_MAX_VIDS		41
#define AVS_MAX_VIDS_ALLOWED    38

/**
 * AVS related defines
 **/
#define TITAN_I2CMUX_ADDR	0x71
#define TITAN_I2CMUX_VAL        0x02
#define TITAN_VDDAVS_ADDR	0x0D
#define TITAN_VDDAVS_REG	0x21
#define TITAN_VDDAVS_RD_REG	0x8B
#define TITAN_VDDFXD_ADDR	0x0E
#define TITAN_VDDFXD_REG	0x21
#define TITAN_VDDFXD_RD_REG	0x8B

#define HAWK_I2CMUX_ADDR	0x70
#define HAWK_I2CMUX_VAL         0x40
#define HAWK_VDDAVS_ADDR	0x57
#define HAWK_VDDAVS_REG		0x21
#define HAWK_VDDAVS_RD_REG	0x8B
#define HAWK_VDDFXD_ADDR	0x58
#define HAWK_VDDFXD_REG		0x21
#define HAWK_VDDFXD_RD_REG	0x8B

#define PEGASUS_VDDAVS_ADDR	0x5E
#define PEGASUS_VDDAVS_REG	0x21
#define PEGASUS_VDDFXD_ADDR	0x5E
#define PEGASUS_VDDFXD_REG	0x20

#define PVT_MON_BASE_ADDR	0x60480000

enum e_pmic_names {
	PMIC_VDDFXD = 0,
	PMIC_VDDAVS
};

u32 vid_hawk[AVS_MAX_VIDS] = 
{0x0133,0x0138,0x013D,0x0142,0x0147,0x014C,0x0151,0x0157,0x015C,0x0161,0x0166,0x016B,0x0170,0x0175,0x017A,0x0180,0x0185,0x018A,0x018F,0x0194,0x0199,0x019E,0x01A3,0x01A8,0x01AE,0x01B3,0x01B8,0x01BD,0x01C2,0x01C7,0x01CC,0x01D1,0x01D7,0x01DC,0x01E1,0x01E6,0x01EB,0x01F0,0x01F5,0x01FA,0x0200}; 

u32 vid_pegasus[AVS_MAX_VIDS] = 
{0x0027,0x0029,0x002B,0x002E,0x0030,0x0032,0x0034,0x0036,0x0038,0x0039,0x003B,0x003D,0x003F,0x0041,0x0044,0x0046,0x0048,0x004A,0x004C,0x004E,0x0050,0x0052,0x0053,0x0055,0x0057,0x0059,0x005C,0x005E,0x0060,0x0062,0x0064,0x0066,0x0068,0x006A,0x006B,0x006D,0x006F,0x0071,0x0073,0x0076,0x0078}; 

u32 vid_titan_avs[AVS_MAX_VIDS] = 
{0x1333,0x1385,0x13D7,0x1428,0x147A,0x14CC,0x151E,0x1570,0x15C2,0x1614,0x1666,0x16B8,0x170A,0x175C,0x17AE,0x1800,0x1851,0x18A3,0x18F5,0x1947,0x1999,0x19EB,0x1A3D,0x1A8F,0x1AE1,0x1B33,0x1B85,0x1BD7,0x1C28,0x1C7A,0x1CCC,0x1D1E,0x1D70,0x1DC2,0x1E14,0x1E66,0x1EB8,0x1F0A,0x1F5C,0x1FAE,0x2000};

u32 vid_titan_fxd[AVS_MAX_VIDS] = 
{0x0999,0x09C2,0x09EB,0x0A14,0x0A3D,0x0A66,0x0A8F,0x0AB8,0x0AE1,0x0B0A,0x0B33,0x0B5C,0x0B85,0x0BAE,0x0BD7,0x0C00,0x0C28,0x0C51,0x0C7A,0x0CA3,0x0CCC,0x0CF5,0x0D1E,0x0D47,0x0D70,0x0D99,0x0DC2,0x0DEB,0x0E14,0x0E3D,0x0E66,0x0E8F,0x0EB8,0x0EE1,0x0F0A,0x0F33,0x0F5C,0x0F85,0x0FAE,0x0FD7,0x1000}; 

int get_pvtmon_ulvt(u32 pvtmon_base)
{
	int rdata;

	cpu_wr_w(pvtmon_base + 0x30, 0);
	cpu_wr_w(pvtmon_base + 0x20, 0);
	cpu_wr_w(pvtmon_base + 0x28, 0x36007A);
	cpu_wr_w(pvtmon_base + 0x20, 1);
	cpu_wr_w(pvtmon_base + 0x30, 1);

	while (1) {
		udelay(10);
		rdata = cpu_rd_w(pvtmon_base + 0x50);
		if (rdata != 0)
			break;
	}

	udelay(10);
	rdata = rdata >> 1;
	cpu_wr_w(pvtmon_base + 0x30, 0);

	return rdata;
}

int get_pvtmon_volt(u32 pvtmon_base)
{
	int rdata;

	cpu_wr_w(pvtmon_base + 0x30, 0);
	cpu_wr_w(pvtmon_base + 0x20, 0);
	cpu_wr_w(pvtmon_base + 0x28, 0x36007C);
	cpu_wr_w(pvtmon_base + 0x20, 1);
	cpu_wr_w(pvtmon_base + 0x30, 1);

	while (1) {
		udelay(10);
		rdata = cpu_rd_w(pvtmon_base + 0x50);
		if (rdata != 0)
			break;
	}

	udelay(10);
	rdata = rdata >> 1;
	cpu_wr_w(pvtmon_base + 0x30, 0);

	return rdata;
}

/**
 * VID/CFGIDX look up table
 * board - hawk/pegasus
 * cfgidx - 0 to AVS_MAX_VIDS_ALLOWED for 0.6V to 1.0V in steps of 10mV
 * pmicid - 0:fxd, 1:avs
 */

u32 get_vid(u32 board, u32 cfgidx, u32 pmicid)
{
	u32 vid = 0;

	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		if (pmicid == PMIC_VDDFXD || pmicid == PMIC_VDDAVS)
			vid = vid_hawk[cfgidx];
	} else if (board == BOARD_PEGASUS) {
		if (pmicid == PMIC_VDDFXD || pmicid == PMIC_VDDAVS)
			vid = vid_pegasus[cfgidx];
	} else if (board == BOARD_TITAN) {
		if (pmicid == PMIC_VDDFXD)
			vid = vid_titan_fxd[cfgidx];
		if (pmicid == PMIC_VDDAVS)
			vid = vid_titan_avs[cfgidx];
	}

	return vid;
}

u32 get_cfgidx(u32 board, u32 vid, u32 pmicid)
{
	u32 cfgidx = 0;

	for (cfgidx = 0; cfgidx < AVS_MAX_VIDS; cfgidx++) {
		if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
			if (pmicid == PMIC_VDDFXD || pmicid == PMIC_VDDAVS) {
				if (vid <= vid_hawk[cfgidx])
					break;
			}
		} else if (board == BOARD_PEGASUS) {
			if (pmicid == PMIC_VDDFXD || pmicid == PMIC_VDDAVS) {
				if (vid <= vid_pegasus[cfgidx])
					break;
			}
		} else if (board == BOARD_TITAN) {
			if (pmicid == PMIC_VDDFXD) {
				if (vid <= vid_titan_fxd[cfgidx])
					break;
			}
			if (pmicid == PMIC_VDDAVS) {
				if (vid <= vid_titan_avs[cfgidx])
					break;
			}
		}
	}

	return cfgidx;
}

u32 get_pmic_addr(u32 board, u32 pmicid)
{
	u32 addr = 0;

	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		if (pmicid == PMIC_VDDFXD) {
			addr = HAWK_VDDFXD_ADDR;
			if (board == BOARD_HAWKV4 || board == BOARD_HAWKV5)
				addr = HAWK_VDDAVS_ADDR;
		}
		else if (pmicid == PMIC_VDDAVS)
			addr = HAWK_VDDAVS_ADDR;
	} else if (board == BOARD_PEGASUS) {
		if (pmicid == PMIC_VDDFXD)
			addr = PEGASUS_VDDFXD_ADDR;
		else if (pmicid == PMIC_VDDAVS)
			addr = PEGASUS_VDDAVS_ADDR;
	} else if (board == BOARD_TITAN) {
		if (pmicid == PMIC_VDDFXD)
			addr = TITAN_VDDFXD_ADDR;
		else if (pmicid == PMIC_VDDAVS)
			addr = TITAN_VDDAVS_ADDR;
	}
	return addr;
}

u32 get_pmic_offset_len(u32 board, u32 pmicid)
{
	u32 len = 0;

	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) 
		len = 2;
	else if (board == BOARD_PEGASUS)
		len = 1;
	else if (board == BOARD_TITAN)
		len = 2;
	return len;
}

u32 get_pmic_reg_addr(u32 board, u32 pmicid)
{
	u32 addr = 0;

	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		if (pmicid == PMIC_VDDFXD)
			addr = HAWK_VDDFXD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = HAWK_VDDAVS_REG;
	} else if (board == BOARD_PEGASUS) {
		if (pmicid == PMIC_VDDFXD)
			addr = PEGASUS_VDDFXD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = PEGASUS_VDDAVS_REG;
	} else if (board == BOARD_TITAN) {
		if (pmicid == PMIC_VDDFXD)
			addr = TITAN_VDDFXD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = TITAN_VDDAVS_REG;
	}
	return addr;
}

u32 get_pmic_rdreg_addr(u32 board, u32 pmicid)
{
	u32 addr = 0;

	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		if (pmicid == PMIC_VDDFXD)
			addr = HAWK_VDDFXD_RD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = HAWK_VDDAVS_RD_REG;
	} else if (board == BOARD_PEGASUS) {
		if (pmicid == PMIC_VDDFXD)
			addr = PEGASUS_VDDFXD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = PEGASUS_VDDAVS_REG;
	} else if (board == BOARD_TITAN) {
		if (pmicid == PMIC_VDDFXD)
			addr = TITAN_VDDFXD_RD_REG;
		else if (pmicid == PMIC_VDDAVS)
			addr = TITAN_VDDAVS_RD_REG;
	}
	return addr;
}

/**
 * Actual PMIC configuration I2C/PMBUS Write
 */
int set_pmic_cfg(struct udevice *dev, u32 regoffset, u32 board, u32 cfgidx, u32 pmicid)
{
	u32 vid;
	int i, len, ret = 0;
	u8 wd[2];

	vid = get_vid(board, cfgidx, pmicid);
	len = get_pmic_offset_len(board, pmicid);
	wd[0] = (u8)(vid & 0xFF);
	wd[1] = (u8)((vid & 0xFF00) >> 8);

	debug("uboot:avs: set_cfg: pmicid %d, cfgidx %d, vid %04X dbg ", pmicid, cfgidx, vid);
	for (i = 0; i < len; i++)
		debug("%02X ", wd[i]);
	debug("\r\n");
	ret = dm_i2c_write(dev, regoffset, wd, len);

	return ret;
}

int get_pmic_cfg(struct udevice *dev, u32 regoffset, u32 board, u32 *cfgidx, u32 pmicid)
{
	u32 vid;
	int len, i, ret = 0;
	u8 rd[2] = {0, 0};

	len = get_pmic_offset_len(board, pmicid);
	ret = dm_i2c_read(dev, regoffset, rd, len);
	//TITAN reads to need it twice !!
	if (board == BOARD_TITAN)
		ret = dm_i2c_read(dev, regoffset, rd, len);
	vid = (rd[0] + (rd[1] << 8));
	*cfgidx = get_cfgidx(board, vid, pmicid);
	debug("uboot:avs: get_cfg: pmicid %d, cfgidx %d, vid %04X dbg ", pmicid, *cfgidx, vid);
	for (i = 0; i < len; i++)
		debug("%02X ", rd[i]);
	debug("\r\n");

	return ret;
}

int reconfig_pmics(struct udevice *pmic_fxd, struct udevice *pmic_avs, u32 board, u32 volt_idx)
{
	u32 pmic_reg_off;
	int ret = 0;

	pmic_reg_off = get_pmic_reg_addr(board, PMIC_VDDFXD);
	ret = set_pmic_cfg(pmic_fxd, pmic_reg_off, board, volt_idx, PMIC_VDDFXD);
	if (ret)
		return ret;

	pmic_reg_off = get_pmic_reg_addr(board, PMIC_VDDAVS);
	ret = set_pmic_cfg(pmic_avs, pmic_reg_off, board, volt_idx, PMIC_VDDAVS);
	if (ret)
		return ret;

	return ret;
}

int cfg_avs_pmic(u32 avs_idx)
{
	struct udevice *bus, *i2cmux, *pmic_avs, *pmic_fxd;
	/* PPU PVTMON used for pvt */
	u32 pvtmon_base = PVT_MON_BASE_ADDR;
	u32 voltfin;
	u32 voltstart;
	u32 pvtfin;
	u32 pvtstart;
	u32 board;
	u32 pmic_addr;
	u32 pmic_reg_off;
	u32 i2cmux_addr;
	u32 curr_fxd_idx;
	u32 curr_avs_idx;
	u32 fin_fxd_idx;
	u32 fin_avs_idx;
	u32 idx;
	bool volt_step_dir;
	int ret = 0;
	u8 i2cmux_val;

	/* Get board Type */
	board = get_board_name_value();
	if (board == BOARD_UNKNOWN) {
		printf("u-boot: Skipping AVS, board type unsupported\r\n");
		return ret;
	}

	/* Get current PVT values */
	voltstart = get_pvtmon_volt(pvtmon_base);
	pvtstart = get_pvtmon_ulvt(pvtmon_base);

	/* Get the I2C bus */
	ret = uclass_get_device(UCLASS_I2C, 0, &bus);
	if (ret) {
		printf("u-boot:avs:err Unable to get the i2c device\r\n");
		goto error;
	}

	/* Board specific i2c mux configuration */
	i2cmux_addr =   (board == BOARD_HAWKV2 || board == BOARD_HAWKV3 || board == BOARD_HAWKV4 || board == BOARD_HAWKV5 ) ? HAWK_I2CMUX_ADDR : TITAN_I2CMUX_ADDR;
	i2cmux_val =  (board == BOARD_HAWKV2 || board == BOARD_HAWKV3 || board == BOARD_HAWKV4 || board == BOARD_HAWKV5 ) ? HAWK_I2CMUX_VAL : TITAN_I2CMUX_VAL;

	if (board == BOARD_HAWKV2 || board == BOARD_HAWKV3 || board == BOARD_HAWKV4 || board == BOARD_HAWKV5 || board == BOARD_TITAN) {
		/* Probe the MUX */
		ret = i2c_get_chip(bus, i2cmux_addr, 1, &i2cmux);
		if (ret) {
			printf("u-boot:avs:err Unable probing i2c mux slave addr:%02X\n",
					i2cmux_addr);
			goto error;
		}

		ret = dm_i2c_write(i2cmux, 0x00, &i2cmux_val, 1);
		if (ret) {
			printf("u-boot:avs:err Unable writing i2c mux slave addr:%02X,val:%02X\r\n",
					i2cmux_addr, i2cmux_val);
			goto error;
		}
	}

	/* PMIC Configuration for FXD PMIC */
	pmic_addr = get_pmic_addr(board, PMIC_VDDFXD);
	ret = i2c_get_chip(bus, pmic_addr, 1, &pmic_fxd);
	if (ret) {
		printf("u-boot:avs:err Unable probing PMIC slave addr:%02X\r\n", pmic_addr);
		goto error;
	} else {
		debug("u-boot:avs: Success in probing PMIC slave addr:%02X\r\n", pmic_addr);
	}

	/* PMIC Configuration for AVS PMIC */
	pmic_addr = get_pmic_addr(board, PMIC_VDDAVS);
	ret = i2c_get_chip(bus, pmic_addr, 1, &pmic_avs);
	if (ret) {
		printf("u-boot:avs:err Unable probing PMIC slave addr:%02X\r\n", pmic_addr);
		goto error;
	} else {
		debug("u-boot:avs: Success in probing PMIC slave addr:%02X\r\n", pmic_addr);
	}

	/* Get the current configured voltage idx */
	pmic_reg_off = get_pmic_rdreg_addr(board, PMIC_VDDFXD);
	ret = get_pmic_cfg(pmic_fxd, pmic_reg_off, board, &curr_fxd_idx, PMIC_VDDFXD);
	if (ret)
		goto error;

	pmic_reg_off = get_pmic_rdreg_addr(board, PMIC_VDDAVS);
	ret = get_pmic_cfg(pmic_avs, pmic_reg_off, board, &curr_avs_idx, PMIC_VDDAVS);
	if (ret)
		goto error;

	/**
	 * For hawk & pegasus- both PMICs are configured at same value from start and target
	 * So using only curr_fxd_idx for further computation.
	 * Get the target avs_idx & curr_fxd_idx- check if we want to step up or step down voltage
	 * Perform the voltage reconfig in steps of 10mV (i.e. 1 idx) and
	 * 20uS wait after each reconfig.
	 */
	volt_step_dir = (curr_fxd_idx >= avs_idx) ? 0 /*down*/ : 1 /*up*/;
	if (volt_step_dir == 0) { /* step down loop */
		for (idx = curr_fxd_idx; idx >= avs_idx; idx--) {
			debug("Step down: initial_idx=%d, target_idx=%d, now setting index=%d\n",
					curr_fxd_idx, avs_idx, idx);
			/* Configure the PMICs */
			ret = reconfig_pmics(pmic_fxd, pmic_avs, board, idx);
			if (ret)
				goto error;

			/* delay for stabilizing voltage */
			udelay(20);

			if (idx == 0)
				break;
		}
	} else { /* step up loop */
		for (idx = curr_fxd_idx; idx <= avs_idx; idx++) {
			debug("Step up: initial_idx=%d, target_idx=%d, now setting index=%d\n",
					curr_fxd_idx, avs_idx, idx);
			/* Configure the PMICs */
			ret = reconfig_pmics(pmic_fxd, pmic_avs, board, idx);
			if (ret)
				goto error;

			/* delay for stabilizing voltage */
			udelay(20);

			if (idx == AVS_MAX_VIDS_ALLOWED)
				break;
		}
	}

	/* Meaure pvtmon/volt */
	mdelay(10);
	pmic_reg_off = get_pmic_rdreg_addr(board, PMIC_VDDFXD);
	ret = get_pmic_cfg(pmic_fxd, pmic_reg_off, board, &fin_fxd_idx, PMIC_VDDFXD);
	if (ret)
		goto error;
	pmic_reg_off = get_pmic_rdreg_addr(board, PMIC_VDDAVS);
	ret = get_pmic_cfg(pmic_avs, pmic_reg_off, board, &fin_avs_idx, PMIC_VDDAVS);
	if (ret)
		goto error;
	/* Get the current configured voltage idx */
	voltfin = get_pvtmon_volt(pvtmon_base);
	pvtfin = get_pvtmon_ulvt(pvtmon_base);

	printf("AVS Info: Status Pass: board = %d, tgt_idx = %d !\r\n"
	       "AVS Info: Initial: pvt = %d, volt = %d, "
	       "fxd_idx = %d, avs_idx = %d \r\n"
	       "AVS Info: Final : pvt = %d, volt = %d, "
	       "fxd_idx = %d, avs_idx = %d \r\n",
	       board, avs_idx,
	       pvtstart, voltstart,
	       curr_fxd_idx, curr_avs_idx, pvtfin, voltfin,
	       fin_fxd_idx, fin_avs_idx);

	return ret;

error:
	voltfin = get_pvtmon_volt(pvtmon_base);
	pvtfin = get_pvtmon_ulvt(pvtmon_base);
	printf("AVS Info: Status Fail: board = %d, tgt_idx = %d !\r\n"
	       "AVS Info: Final  : pvt = %d, volt = %d, ",
	       board, avs_idx, pvtfin, voltfin);

	/**
	 * TODO: Stay here as PMIC config is failed
	 * WDT based recovery should reset device once activated.
	 **/
	while (1)
		;

	return ret;
}

static int is_valid_index(const char *str)
{
	int j = 0;

	if (!str)
		return 0;

	while (isdigit(str[j]))
		j++;

	return ((strlen(str) == j) && (j != 0));
}

static int do_set_avs(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	u32 index = 0;
	char *str;

	if (argc != 2)
		return CMD_RET_USAGE;

	if (!argv[1]) {
		printf("AVS: please provide proper index value\n");
		goto error;
	}

	str = strdup(argv[1]);

	if (!is_valid_index(str))
		goto error;

	index = simple_strtol(str, NULL, 10);

	if (index < 0 || index > AVS_MAX_VIDS_ALLOWED)
		goto error;

	cfg_avs_pmic(index);

	return 0;
error:
	return -1;
}

U_BOOT_CMD(set_avs,    2,    0,     do_set_avs,
	"set avs for pmic",
	"<index>\n"
	"<index>	- integer IDX value for selecting step values 0.6V=0, there after steps of 10mV per index\n"
	"The value should be in between 0 to 38\n"
);
