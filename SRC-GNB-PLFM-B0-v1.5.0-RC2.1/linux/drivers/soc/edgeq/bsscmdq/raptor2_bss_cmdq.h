/* SPDX-License-Identifier: GPL-2.0-or-later
 * Raptor2 Header file for BSS CMDQ
 * Copyright (C) 2023 EdgeQ Inc.
 * Author: Pravin Bathija <bathija@edgeq.io>
 */

#ifndef _RAPTOR2_BSS_CMDQ_H
#define _RAPTOR2_BSS_CMDQ_H

#define RAPTOR2_BSS_CMDQ_DEV_NAME	"r2bsscmdq"
#define BSS_MSG_DATA_MAX_SIZE		256

#define BSSRT_READY_OFFS		0x0
#define BSS_CMD_MSG_RDY_OFFSET		0x8
#define BSS_RSP_MSG_RDY_OFFSET		0x10
#define BSS_CMD_MSG_OFFSET		0x80
#define BSS_RSP_MSG_OFFSET		0x200

#define BSS_MEM_TEMP_KEY_OFFS		0xF0000

#define BST_RT_READY_MAGIC_NUM		(0xA5A5A5A5)
#define CMD_MSG_READY_MAGIC_NUM		(0x55555555)
#define RESP_MSG_READY_MAGIC_NUM	(0xAAAAAAAA)
#define MSG_WAIT_TIME			1000 //us
#define MAX_RESP_MSG_DELAY		100000000

typedef struct msg_bss {
	uint8_t rsv[2];
	uint8_t id;	/* msg id */
	uint8_t type;	/* msg type */
	uint16_t info;	/* addl info like heartbeat */
	uint8_t size;
	uint8_t data[BSS_MSG_DATA_MAX_SIZE];
	uint32_t checksum;
} MSG_BSS;

#endif
