#include <lib/mmio.h>
#include <lib/utils_def.h>
#include "sif_config.h"

#if B0_BUILD
extern struct sif_routing_info_t sif_su_routing_table[MAX_SIF_TO_SU_EVENTS];
#else
extern struct sif_routing_info_t sif_routing_table[MAX_SIF_OUTPUT_EVENTS];
#endif

static inline int sif_configure_bit_position(uintptr_t base_addr,
					     uint16_t event, uint32_t value)
{
	uint32_t bit_pos = (event % 32);
	uint32_t bit_base_offset = (event / 32);
	uint32_t mask = (1 << bit_pos);
	/* Multiply by 4 to account for pointer arithmetic */
	uintptr_t reg_addr = base_addr + (bit_base_offset * 4);

	/* Read current value and update respective bit position. */
	mmio_clrsetbits_32(reg_addr, mask, (value << bit_pos));

	return 0;
}

#define SIF_CONFIGURE_ENABLE_REG_EVENT_TO_SU(_event, _value) \
    sif_configure_bit_position(SIF_ENABLE_REG_EVENT_TO_SU_BASE, _event, _value)
#define SIF_CONFIGURE_REG_EVENT_TO_SU(_event, _value) \
    sif_configure_bit_position(SIF_REG_EVENT_TO_SU_BASE, _event, _value)
#define SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_SU(_event, _value) \
    sif_configure_bit_position(SIF_ENABLE_CPU_EVENT_TO_SU_BASE, _event, _value)
#define SIF_CONFIGURE_SU_IN_EVENT_HI_SEL_SU(_event, _value) \
    sif_configure_bit_position(SIF_SU_IN_EVENT_HI_SEL_SU_BASE, _event, _value)

#if B0_BUILD
#define SIF_CONFIGURE_SU_IN_EVENT_MID_SEL_SU(_event, _value) \
    sif_configure_bit_position(SIF_SU_IN_EVENT_MID_SEL_SU_BASE, _event, _value)
#else
#define SIF_CONFIGURE_ENABLE_REG_EVENT_TO_CPU(_event, _value) \
    sif_configure_bit_position(SIF_ENABLE_REG_EVENT_TO_CPU_BASE, _event, _value)
#define SIF_CONFIGURE_REG_EVENT_TO_CPU(_event, _value) \
    sif_configure_bit_position(SIF_REG_EVENT_TO_CPU_BASE, _event, _value)
#define SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_CPU(_event, _value) \
    sif_configure_bit_position(SIF_ENABLE_CPU_EVENT_TO_CPU_BASE, _event, _value)
#define SIF_CONFIGURE_SU_IN_EVENT_HI_SEL_CPU(_event, _value) \
    sif_configure_bit_position(SIF_SU_IN_EVENT_HI_SEL_CPU_BASE, _event, _value)
#endif

static inline int sif_configure_byte_position(uintptr_t base_addr,
					      uint16_t input_evt,
					      uint16_t output_evt)
{
	uint32_t byte_pos = (output_evt % 4);
	uint32_t byte_base_offset = (output_evt / 4);
	uint32_t shift = (byte_pos * 8);
	uint32_t mask = (0xFF << shift);
	/* Multiply by 4 to account for pointer arithmetic */
	uintptr_t reg_addr = base_addr + (byte_base_offset * 4);

	/* Read current value and update respective byte position. */
	mmio_clrsetbits_32(reg_addr, mask, (uint32_t)(input_evt << shift));

	return 0;
}

#define SIF_CONFIGURE_CPU_IN_EVENT_SEL_SU(_in_evt, _out_evt) \
   sif_configure_byte_position(SIF_CPU_IN_EVENT_SEL_SU_BASE, _in_evt, _out_evt)
#define SIF_CONFIGURE_SU_IN_EVENT_SEL_SU(_in_evt, _out_evt) \
   sif_configure_byte_position(SIF_SU_IN_EVENT_SEL_SU_BASE, _in_evt, _out_evt)

#if !B0_BUILD
#define SIF_CONFIGURE_CPU_IN_EVENT_SEL_CPU(_in_evt, _out_evt) \
   sif_configure_byte_position(SIF_CPU_IN_EVENT_SEL_CPU_BASE, _in_evt, _out_evt)
#define SIF_CONFIGURE_SU_IN_EVENT_SEL_CPU(_in_evt, _out_evt) \
   sif_configure_byte_position(SIF_SU_IN_EVENT_SEL_CPU_BASE, _in_evt, _out_evt)
#endif

/*
 * Program SIF for SU to SU routing.
 *    1. enable_reg_event_to_su<output_evt> = 0
 *    2. enable_cpu_event_to_su<output_evt> = 0
 *    3. su_in_event_sel_su<output_evt>[8:0] = <input_evt>	(A0_BUILD)
 *    OR
 *    3. su_in_event_sel_su<output_evt>[9:0] = <input_evt>	(B0_BUILD)
 */
static int sif_routing_su_to_su(uint16_t input_evt, uint16_t output_evt)
{
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_SU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_SU(output_evt, 0);
#if B0_BUILD
	SIF_CONFIGURE_SU_IN_EVENT_HI_SEL_SU(output_evt,
					   ((input_evt >> 9) & 0x1));
	SIF_CONFIGURE_SU_IN_EVENT_MID_SEL_SU(output_evt,
					    ((input_evt >> 8) & 0x1));
#else
	SIF_CONFIGURE_SU_IN_EVENT_HI_SEL_SU(output_evt,
					   ((input_evt >> 8) & 0x1));
#endif
	SIF_CONFIGURE_SU_IN_EVENT_SEL_SU((input_evt & 0xFF), output_evt);

	return 0;
}

#if !B0_BUILD
/*
 * Program SIF for SU to E1 routing.
 *    1. enable_reg_event_to_cpu<output_evt> = 0
 *    2. enable_cpu_event_to_cpu<output_evt> = 0
 *    3. su_in_event_sel_cpu<output_evt>[8:0] = <input_evt>
 */
static int sif_routing_su_to_e1(uint16_t input_evt, uint16_t output_evt)
{
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_CPU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_CPU(output_evt, 0);
	SIF_CONFIGURE_SU_IN_EVENT_HI_SEL_CPU(output_evt,
					    ((input_evt >> 8) & 0x1));
	SIF_CONFIGURE_SU_IN_EVENT_SEL_CPU((input_evt & 0xFF), output_evt);

	return 0;
}
#endif

/*
 * Program SIF for E1 to SU routing.
 *    1. enable_reg_event_to_su<output_evt> = 0
 *    2. enable_cpu_event_to_su<output_evt> = 1
 *    3. cpu_in_event_sel_su<output_evt>[7:0] = <input_evt>
 */
static int sif_routing_e1_to_su(uint16_t input_evt, uint16_t output_evt)
{
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_SU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_SU(output_evt, 1);
	SIF_CONFIGURE_CPU_IN_EVENT_SEL_SU(input_evt, output_evt);

	return 0;
}

#if !B0_BUILD
/*
 * Program SIF for E1 to E1 routing
 *    1. enable_reg_event_to_cpu<output_evt> = 0
 *    2. enable_cpu_event_to_cpu<output_evt> = 1
 *    3. cpu_in_event_sel_cpu<output_evt>[7:0] = <input_evt>
 */
static int sif_routing_e1_to_e1(uint16_t input_evt, uint16_t output_evt)
{
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_CPU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_CPU(output_evt, 1);
	SIF_CONFIGURE_CPU_IN_EVENT_SEL_CPU(input_evt, output_evt);

	return 0;
}
#endif

/*
 * Program SIF to SU reg events
 *    1. reg_event_to_su<output_evt> = 0
 *    2. enable_reg_event_to_su<output_evt> = 1
 *    3. enable_cpu_event_to_su<output_evt> = 0
 */
static int sif_routing_reg_evt_to_su(uint16_t output_evt)
{
	/* Make sure reg event output is 0 before enabling reg based events */
	SIF_CONFIGURE_REG_EVENT_TO_SU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_SU(output_evt, 1);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_SU(output_evt, 0);

	return 0;
}

#if !B0_BUILD
/*
 * Program SIF to E1 reg events
 *    1. reg_event_to_cpu<output_evt> = 0
 *    2. enable_reg_event_to_cpu<output_evt> = 1
 *    3. enable_cpu_event_to_cpu<output_evt> = 0
 */
static int sif_routing_reg_evt_to_e1(uint16_t output_evt)
{
	/* Make sure reg event output is 0 before enabling reg based events */
	SIF_CONFIGURE_REG_EVENT_TO_CPU(output_evt, 0);
	SIF_CONFIGURE_ENABLE_REG_EVENT_TO_CPU(output_evt, 1);
	SIF_CONFIGURE_ENABLE_CPU_EVENT_TO_CPU(output_evt, 0);

	return 0;
}
#endif

int sif_routing_programming()
{
	uint32_t index = 0;
	int ret_val = 0;

#if B0_BUILD
	for (index = 0; index < MAX_SIF_TO_SU_EVENTS; index++) {
		switch (sif_su_routing_table[index].routing_type) {
		case SIF_ROUTING_SU_TO_SU:
			ret_val = sif_routing_su_to_su(
				  sif_su_routing_table[index].in_evt,
				  sif_su_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_E1_TO_SU:
			ret_val = sif_routing_e1_to_su(
				  sif_su_routing_table[index].in_evt,
				  sif_su_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_REG_EVT_TO_SU:
			ret_val = sif_routing_reg_evt_to_su(
				  sif_su_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_UNUSED:
			break;
		default:
			ret_val = -1;
			break;
		}

		if (ret_val < 0)
			break;
	}
#else
	for (index = 0; index < MAX_SIF_OUTPUT_EVENTS; index++) {
		switch (sif_routing_table[index].routing_type) {
		case SIF_ROUTING_SU_TO_SU:
			ret_val = sif_routing_su_to_su(
				  sif_routing_table[index].in_evt,
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_SU_TO_E1:
			ret_val = sif_routing_su_to_e1(
				  sif_routing_table[index].in_evt,
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_E1_TO_SU:
			ret_val = sif_routing_e1_to_su(
				  sif_routing_table[index].in_evt,
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_E1_TO_E1:
			ret_val = sif_routing_e1_to_e1(
				  sif_routing_table[index].in_evt,
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_REG_EVT_TO_SU:
			ret_val = sif_routing_reg_evt_to_su(
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_REG_EVT_TO_E1:
			ret_val = sif_routing_reg_evt_to_e1(
				  sif_routing_table[index].out_evt);
			break;
		case SIF_ROUTING_UNUSED:
			break;
		default:
			ret_val = -1;
			break;
		}

		if (ret_val < 0)
			break;
	}
#endif

	return ret_val;
}
/*
 * SIF Module Initialization.
 *   - Program SIF module based on routing table.
 */
void sif_init(void)
{
	sif_routing_programming();
}

