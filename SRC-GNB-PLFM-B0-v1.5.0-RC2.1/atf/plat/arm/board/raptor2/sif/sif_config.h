/*
 * SIF Programming functionalities.
 * This implementation is based on SIF_R2_SPEC and SIF_R2B0_SPEC documents.
 */

#ifndef __SIF_CONFIG_H__
#define __SIF_CONFIG_H__

#include <stdint.h>

#if B0_BUILD
/*
 * SIF Programming Base addresses
 * Further definitions of individual base addresses are defined below.
 */
#define SIF_PRG_BASE_ADDR			0x69080000UL

#define SIF_ENABLE_REG_EVENT_TO_SU_BASE		0x69080020UL
#define SIF_REG_EVENT_TO_SU_BASE		0x69080060UL
#define SIF_ENABLE_CPU_EVENT_TO_SU_BASE		0x690800A0UL
#define SIF_CPU_IN_EVENT_SEL_SU_BASE		0x690800E0UL
#define SIF_SU_IN_EVENT_MID_SEL_SU_BASE		0x690802E0UL
#define SIF_SU_IN_EVENT_HI_SEL_SU_BASE		0x69080320UL
#define SIF_SU_IN_EVENT_SEL_SU_BASE		0x69080360UL

/*
 * SIF Routing types
 * Note: To reduce memory usage, declared as macro instead of enum.
 */
#define SIF_ROUTING_UNUSED		0
#define SIF_ROUTING_SU_TO_SU		1
#define SIF_ROUTING_E1_TO_SU		2
#define SIF_ROUTING_REG_EVT_TO_SU	3

/* Event is considered as 2 byte value for internal storage. */
#define SIF_INVALID_EVENT	0xFFFF

#define MAX_SIF_TO_SU_EVENTS	512
#else
/*
 * SIF Programming Base addresses
 * Further definitions of individual base addresses are defined below.
 */
#define SIF_PRG_BASE_ADDR			0x69080000UL

#define SIF_ENABLE_REG_EVENT_TO_SU_BASE		0x69080020UL
#define SIF_REG_EVENT_TO_SU_BASE		0x69080060UL
#define SIF_ENABLE_CPU_EVENT_TO_SU_BASE		0x690800A0UL
#define SIF_SU_IN_EVENT_HI_SEL_SU_BASE		0x690800E0UL
#define SIF_CPU_IN_EVENT_SEL_SU_BASE		0x69080120UL
#define SIF_SU_IN_EVENT_SEL_SU_BASE		0x69080320UL
#define SIF_ENABLE_REG_EVENT_TO_CPU_BASE	0x69080520UL
#define SIF_REG_EVENT_TO_CPU_BASE		0x69080540UL
#define SIF_ENABLE_CPU_EVENT_TO_CPU_BASE	0x69080560UL
#define SIF_SU_IN_EVENT_HI_SEL_CPU_BASE		0x69080580UL
#define SIF_CPU_IN_EVENT_SEL_CPU_BASE		0x690805A0UL
#define SIF_SU_IN_EVENT_SEL_CPU_BASE		0x690806A0UL

/*
 * SIF Routing types
 * Note: To reduce memory usage, declared as macro instead of enum.
 */
#define SIF_ROUTING_UNUSED		0
#define SIF_ROUTING_SU_TO_SU		1
#define SIF_ROUTING_SU_TO_E1		2
#define SIF_ROUTING_E1_TO_SU		3
#define SIF_ROUTING_E1_TO_E1		4
#define SIF_ROUTING_REG_EVT_TO_SU	5
#define SIF_ROUTING_REG_EVT_TO_E1	6

/* Event is considered as 2 byte value for internal storage. */
#define SIF_INVALID_EVENT	0xFFFF

#define MAX_SIF_TO_SU_EVENTS	512
#define MAX_SIF_TO_E1_EVENTS	256

#define MAX_SIF_OUTPUT_EVENTS	(MAX_SIF_TO_SU_EVENTS + MAX_SIF_TO_E1_EVENTS)
#endif

struct sif_routing_info_t
{
	uint8_t  routing_type;
	uint16_t in_evt;
	uint16_t out_evt;
};

void sif_init(void);

#endif  /* __SIF_CONFIG_H__ */
