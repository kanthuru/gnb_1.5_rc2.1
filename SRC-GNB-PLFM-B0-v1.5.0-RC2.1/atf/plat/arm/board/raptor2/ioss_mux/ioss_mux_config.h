/*
 * IOSS MUX Programming functionalities.
 * This implementation is based on SIF_R2_SPEC document.
 */

#ifndef __IOSS_MUX_CONFIG_H__
#define __IOSS_MUX_CONFIG_H__

#include <stdint.h>

#if B0_BUILD
/* IOSS Mux Programming address */
#define IOSS_MUX_PRG_BASE_ADDR	0x3CB20020UL
#define IOSS_MUX_PRG_MAX_ADDR	0x3CB2021CUL

/* Maximum events */
#define MAX_IOSS_INPUT_EVENTS	512
#define MAX_IOSS_OUTPUT_EVENTS	128
#else
/* IOSS Mux Programming address */
#define IOSS_MUX_PRG_BASE_ADDR	0x3CB20020UL
#define IOSS_MUX_PRG_MAX_ADDR	0x3CB2011CUL

/* Maximum events */
#define MAX_IOSS_INPUT_EVENTS	256
#define MAX_IOSS_OUTPUT_EVENTS	64
#endif

/* Used to specify unused IOSS output event */
#define IOSS_EVENT_UNUSED	0xFFFF

/* Structure to map output and input events */
struct ioss_muxing_info_t
{
	uint8_t output;
	uint16_t input;
};

/* Function Declaration */
void ioss_mux_init(void);

#endif  /* __IOSS_MUX_CONFIG_H__ */
