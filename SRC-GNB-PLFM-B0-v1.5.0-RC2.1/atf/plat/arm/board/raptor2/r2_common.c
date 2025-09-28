/*
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <common/debug.h>
#include <lib/mmio.h>
#include <platform_def.h>
#include <plat/arm/common/arm_config.h>
#include <plat/arm/common/plat_arm.h>

/*
 *  TODO: First 3 mappings are device memory if OCM is non-coherent.
 *         BL2 RO mapping is duplicate here. Already defined in MAP_BL2_EL3_TOTAL.
 *         Need to correct this.
 */
#define  R2_MAP_BL2_RO         MAP_REGION_FLAT(BL2_RO_BASE,  \
                                              (BL2_RO_LIMIT - BL2_RO_BASE), \
                                              MT_CODE | MT_SECURE)

#define  R2_MAP_OCM		MAP_REGION_FLAT(BL31_BASE,  \
				(BL31_LIMIT - BL31_BASE), \
				MT_RW | MT_EXECUTE)

#define  R2_MAP_BL2_RW         MAP_REGION_FLAT(BL2_RW_BASE,  \
                                              (BL2_RW_LIMIT - BL2_RW_BASE), \
                                              MT_RW_DATA | MT_SECURE)

#define  R2_MAP_CONFIG_RW      MAP_REGION_FLAT(R2_CONFIG_RAM_BASE, \
                                               R2_CONFIG_RAM_SIZE, \
                                               MT_RW_DATA | MT_SECURE)

#define  R2_MAP_TRUSTED_DRAM   MAP_REGION_FLAT(R2_TRUSTED_DRAM_BASE, \
                                               R2_TRUSTED_DRAM_SIZE, \
                                               MT_MEMORY | MT_RW | MT_EXECUTE | MT_SECURE)

/*
 * TODO: Need to correct flash mapping as per SoC map.
 */
#define  R2_MAP_FLASH_EMULATED       MAP_REGION_FLAT(R2_FLASH_BASE_EMULATED,\
                                              V2M_FLASH0_SIZE,        \
                                              MT_DEVICE | MT_RO | MT_NS)

#if IOSS_MUX_CONFIG
#define  R2_MAP_IOSS_MUX        MAP_REGION_FLAT(R2_IOSS_MUX_BASE,\
                                                R2_IOSS_MUX_SIZE,\
                                                MT_DEVICE | MT_RW | MT_NS)
#endif

#if SIF_CONFIG
#define  R2_MAP_SIF             MAP_REGION_FLAT(R2_SIF_BASE,\
                                                R2_SIF_SIZE,\
                                                MT_DEVICE | MT_RW | MT_NS)
#endif

#define  R2_MAP_PERIPHERAL_DEV  MAP_REGION_FLAT(R2_PERIPHERAL_BASE,\
                                                R2_PERIPHERAL_SIZE,\
                                                MT_DEVICE | MT_RW | MT_SECURE)

#define  R2_MAP_SHARED_DRAM     MAP_REGION_FLAT(R2_SHARED_RAM_BASE, \
                                                R2_SHARED_RAM_SIZE, \
                                                MT_MEMORY | MT_RW | MT_SECURE)

#define  R2_MAP_NS_DRAM         MAP_REGION_FLAT(R2_NS_DRAM_BASE, \
		                                R2_NS_DRAM_SIZE, \
						MT_MEMORY | MT_RW | MT_NS)

#define  R2_MAP_GIC             MAP_REGION_FLAT(R2_GIC_BASE, \
		                                R2_GIC_SIZE, \
						MT_DEVICE | MT_RW | MT_EXECUTE_NEVER | MT_SECURE)

#if IMAGE_BL2
const mmap_region_t plat_arm_mmap[] = {
	R2_MAP_OCM,
        R2_MAP_CONFIG_RW,
        R2_MAP_BL2_RO,
        R2_MAP_BL2_RW,
        R2_MAP_PERIPHERAL_DEV,
	//R2_MAP_TRUSTED_DRAM,  /* This is mapped by MAP_BL2_EL3_TOTAL */
#if EMULATED_FLASH
        R2_MAP_FLASH_EMULATED,
#endif
	R2_MAP_NS_DRAM,
        {0}
};
#endif

#if IMAGE_BL31
const mmap_region_t plat_arm_mmap[] = {
        //R2_MAP_CONFIG_RW,
	R2_MAP_TRUSTED_DRAM,
#if IOSS_MUX_CONFIG
	R2_MAP_IOSS_MUX,
#endif
#if SIF_CONFIG
	R2_MAP_SIF,
#endif
        R2_MAP_PERIPHERAL_DEV,
	R2_MAP_NS_DRAM,
	R2_MAP_GIC,
#if EMULATED_FLASH
	R2_MAP_FLASH_EMULATED,
#endif
        {0}
};
#endif


ARM_CASSERT_MMAP


unsigned int plat_get_syscnt_freq2(void)
{
        /* TODO: need to fix for real board */
#if A0_BUILD
#if A0_PEGASUS_BUILD
	return 40960000;	//40.96MHz for Pegasus board
#else
	return 40000000;
#endif
#elif PROTIUM_BUILD /* Same for SINGLE_PPU_CLUSTER build */
        return 3680000;
#elif (ZEBU_BUILD || B0_BUILD)
        return 61440000;
#else
        return 10000000;
#endif
}
