/*
 * Copyright (c) 2016-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/bl_common.h>
#include <common/desc_image_load.h>
#include <drivers/generic_delay_timer.h>
#include <plat/arm/common/plat_arm.h>
#include <platform_def.h>
#include <assert.h>

extern struct dram_timing_info dram_timing;

/*******************************************************************************
 * Following descriptor provides BL image/ep information that gets used
 * by BL2 to load the images and also subset of this information is
 * passed to next BL image. The image loading sequence is managed by
 * populating the images in required loading order. The image execution
 * sequence is managed by populating the `next_handoff_image_id` with
 * the next executable image id.
 ******************************************************************************/
static bl_mem_params_node_t bl2_mem_params_descs[] = {
    /* Fill BL31 related information */
    {
	    .image_id = BL31_IMAGE_ID,

	    SET_STATIC_PARAM_HEAD(ep_info, PARAM_EP,
		    VERSION_2, entry_point_info_t,
		    SECURE | EXECUTABLE | EP_FIRST_EXE),
	    .ep_info.pc = BL31_BASE,
	    .ep_info.spsr = SPSR_64(MODE_EL3, MODE_SP_ELX,
		    DISABLE_ALL_EXCEPTIONS),
#if DEBUG
	    .ep_info.args.arg3 = ARM_BL31_PLAT_PARAM_VAL,
#endif

#if R2_PRELOADED_BL31_BASE
	    SET_STATIC_PARAM_HEAD(image_info, PARAM_EP,
			VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
#else
	    SET_STATIC_PARAM_HEAD(image_info, PARAM_EP,
		    VERSION_2, image_info_t, IMAGE_ATTRIB_PLAT_SETUP),
#endif
	    .image_info.image_base = BL31_BASE,
	    .image_info.image_max_size = BL31_LIMIT - BL31_BASE,
	    .next_handoff_image_id = BL33_IMAGE_ID,
    },
    /* Fill HW_CONFIG related information */
    {
	    .image_id = HW_CONFIG_ID,
	    SET_STATIC_PARAM_HEAD(ep_info, PARAM_IMAGE_BINARY,
		    VERSION_2, entry_point_info_t, NON_SECURE | NON_EXECUTABLE),
	    SET_STATIC_PARAM_HEAD(image_info, PARAM_IMAGE_BINARY,
		    VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
	    .next_handoff_image_id = INVALID_IMAGE_ID,
    },
    /* Fill SOC_FW_CONFIG related information */
    {
            .image_id = SOC_FW_CONFIG_ID,
            SET_STATIC_PARAM_HEAD(ep_info, PARAM_IMAGE_BINARY,
                    VERSION_2, entry_point_info_t, SECURE | NON_EXECUTABLE),
            SET_STATIC_PARAM_HEAD(image_info, PARAM_IMAGE_BINARY,
                    VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
            .next_handoff_image_id = INVALID_IMAGE_ID,
    },
    /* Fill BL33 related information */
    {
	    .image_id = BL33_IMAGE_ID,
	    SET_STATIC_PARAM_HEAD(ep_info, PARAM_EP,
		    VERSION_2, entry_point_info_t, NON_SECURE | EXECUTABLE),
# ifdef PRELOADED_BL33_BASE
	    .ep_info.pc = PRELOADED_BL33_BASE,

	    SET_STATIC_PARAM_HEAD(image_info, PARAM_EP,
		    VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
#else
#if R2_PRELOADED_BL33_BASE
	    .ep_info.pc = (PLAT_ARM_NS_IMAGE_BASE + 0xC00000),

	    SET_STATIC_PARAM_HEAD(image_info, PARAM_EP,
		    VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
	    .image_info.image_base = (PLAT_ARM_NS_IMAGE_BASE + 0xC00000),
	    .image_info.image_max_size = R2_BL33_SIZE,
#else
	    .ep_info.pc = (PLAT_ARM_NS_IMAGE_BASE + 0xC00000),

	    SET_STATIC_PARAM_HEAD(image_info, PARAM_EP,
		    VERSION_2, image_info_t, 0),
	    .image_info.image_base = (PLAT_ARM_NS_IMAGE_BASE + 0xC00000),
	    .image_info.image_max_size = R2_BL33_SIZE,
#endif /* R2_PRELOADED_BL33_BASE */
#endif /* PRELOADED_BL33_BASE */
	    .next_handoff_image_id = INVALID_IMAGE_ID,
    },
    /* Fill NT_FW_CONFIG related information */
    {
            .image_id = NT_FW_CONFIG_ID,
            SET_STATIC_PARAM_HEAD(ep_info, PARAM_IMAGE_BINARY,
                    VERSION_2, entry_point_info_t, NON_SECURE | NON_EXECUTABLE),
            SET_STATIC_PARAM_HEAD(image_info, PARAM_IMAGE_BINARY,
                    VERSION_2, image_info_t, IMAGE_ATTRIB_SKIP_LOADING),
            .next_handoff_image_id = INVALID_IMAGE_ID,
    },
};

void bl2_el3_early_platform_setup(u_register_t arg0 __unused,
                                  u_register_t arg1 __unused,
                                  u_register_t arg2 __unused,
                                  u_register_t arg3 __unused)
{
	arm_bl2_el3_early_platform_setup();
        //printf("ATF Early Platform Initialization done.\n");

        /*
	 * Initialize Interconnect for this cluster during cold boot.
	 * No need for locks as no other CPU is active.
	 */
        plat_arm_interconnect_init();
        /*
	 * Enable Interconnect coherency for the primary CPU's cluster.
	 */
        plat_arm_interconnect_enter_coherency();

        generic_delay_timer_init();

	uint64_t *mailbox = (uint64_t *)PLAT_ARM_TRUSTED_MAILBOX_BASE;
	uint8_t ind;

	for(ind = 0; ind < 32; ind++)
	{
		*mailbox++ = 0;
	}

	/* make sure write are complete and send the event */
	dsb();
	sev();
}

REGISTER_BL_IMAGE_DESCS(bl2_mem_params_descs)
