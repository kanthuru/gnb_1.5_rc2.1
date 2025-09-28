/* SPDX-License-Identifier: GPL-2.0-or-later
 * Raptor2 Header file for BSS CMDQ
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Pravin Bathija <bathija@edgeq.io>
 */

#ifndef _RAPTOR2_BSS_API_H
#define _RAPTOR2_BSS_API_H

typedef enum bss_chan_type {
	BSSRT_CHAN_EFUSE,
	BSSRT_CHAN_ONEPPS,
	BSSRT_CHAN_IMGVF,
	BSSRT_CHAN_SUINFRA,
	/* All new chan entries here */
	BSSRT_CHAN_MAX,
} bss_chan_type_t;

/* msg identifier for bss */
enum msgId_bss {
	NONE	= 0x0,
	/* reset section */
	RAPTOR2_MSGID_SOC_RESET = 0x1C,
	RAPTOR2_MSGID_SOC_HEARTBEAT = 0x20,
	/* suinfra section */
	RAPTOR2_MSGID_L1_RESET,
	RAPTOR2_MSGID_LOAD_SU_FIRMWARE,
	RAPTOR2_MSGID_SU_RESET,
	RAPTOR2_MSGID_PUBLIC_KEY_READ,
	RAPTOR2_MSGID_PUBLIC_KEY_WRITE,
	RAPTOR2_MSGID_LICENSE_KEY_READ,
	RAPTOR2_MSGID_LICENSE_KEY_WRITE,
	/* all cmd codes before this */
	RAPTOR2_MSGID_BSS_INVALID
};

/* msg type */
enum msgType_bss {
	RAPTOR2_MSGTYPE_READ = 0x1,
	RAPTOR2_MSGTYPE_WRITE,
	RAPTOR2_MSGTYPE_ACK,
	RAPTOR2_MSGTYPE_NACK,
	RAPTOR2_MSGTYPE_CHECKSUM_ERROR,
	RAPTOR2_MSGTYPE_OP_INVALID,
	RAPTOR2_MSGTYPE_UNKNOWN_CMD
};

static char *cmdstr[] = {
	[NONE] =					"No Command",
	/* reset section */
	[RAPTOR2_MSGID_SOC_RESET] =			"SOC Reset",
	[RAPTOR2_MSGID_SOC_HEARTBEAT] =			"SOC Heartbeat",
	/* suinfra section */
	[RAPTOR2_MSGID_L1_RESET] =			"L1 Reset",
	[RAPTOR2_MSGID_LOAD_SU_FIRMWARE] =		"Load SU Firmware",
	[RAPTOR2_MSGID_SU_RESET] =			"SU Reset",
	[RAPTOR2_MSGID_PUBLIC_KEY_READ] =		"Public Key Read",
	[RAPTOR2_MSGID_PUBLIC_KEY_WRITE] =		"Public Key Write",
	[RAPTOR2_MSGID_LICENSE_KEY_READ] =		"License Key Read",
	[RAPTOR2_MSGID_LICENSE_KEY_WRITE] =		"License Key Write",
	/* all cmd codes before this */
	[RAPTOR2_MSGID_BSS_INVALID] =			"Invalid Message ID",
};

int bss_cmd_setup(bss_chan_type_t ch);
int send_recv_bss_msg(bss_chan_type_t ch, uint8_t cmd, uint8_t msg_type, uint16_t msg_info,
							void *txbuf, uint32_t txbufsz,
							void *rxbuf, uint32_t *rxbufsz);
#endif
