/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#ifndef _RAPTOR_SET_VDD_H
#define _RAPTOR_SET_VDD_H

#ifdef PMIC_STEP_SIZE_25

#define VDD_START	425
#define STEP_SIZE	25

#else

#define VDD_START	410
#define STEP_SIZE	10

#endif

#define VDD_MAX			900 //mVolt
#define VDD_MIN			800
#define DEFAULT_PMIC_VDD        860 //mVolt
#define VDD2RAW(x) (((x) - VDD_START + STEP_SIZE) / STEP_SIZE)

int set_buck2_vdd(int vdd);
int parse_vdd_value(const char *str);

#endif /*_RAPTOR_SET_VDD_H*/
