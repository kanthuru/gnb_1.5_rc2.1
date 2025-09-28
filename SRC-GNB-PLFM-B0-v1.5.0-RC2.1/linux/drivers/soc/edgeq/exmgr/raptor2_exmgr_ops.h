/* SPDX-License-Identifier: GPL-2.0
 * Copyright (c) 2023 EdgeQ, Inc.
 * Raptor2 Include file for EXMGR Ops
 */

#include "raptor2_exmgr.h"

extern int raptor2_exmgr_ops_init(struct eqsu_membuf eqsu_mbuf_table[],
				struct r2_rtos_info *rtosinfo, r2_chip_version_t r2vers);
extern void raptor2_exmgr_ops_cleanup(void);
