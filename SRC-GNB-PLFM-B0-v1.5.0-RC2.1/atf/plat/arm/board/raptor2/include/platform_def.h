/*
 * Copyright (c) 2018-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Raptor specific platform implemenation. Implemented as per docs/getting_started/porting-guide.rst
 */

#ifndef PLATFORM_DEF_H
#define PLATFORM_DEF_H

#include <arch.h>
#include <lib/xlat_tables/xlat_tables_defs.h>
#include <common/tbbr/tbbr_img_def.h>
#include <plat/arm/common/smccc_def.h>
#include <r2_mem_description.h>


#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*
 * Note: Trusted boot has recursive functions.
 * Increasing stack size to safe value of 20 KB.
 */
#define PLATFORM_STACK_SIZE             0X5000
#define BL31_STACK_SIZE			0x400
#define CACHE_WRITEBACK_GRANULE         64

#define RAPTOR2_CHIP_COUNT              U(1)
#define PLAT_ARM_CLUSTER_COUNT		U(2)
#define PLAT_MAX_CPUS_PER_CLUSTER	U(6)
#define PLAT_MAX_PE_PER_CPU		U(2)
#define PLATFORM_CORE_COUNT             ( (PLAT_ARM_CLUSTER_COUNT) * \
                                               (PLAT_MAX_CPUS_PER_CLUSTER * PLAT_MAX_PE_PER_CPU) )

/*
 * Root of trust key hash lengths
 */
#define ARM_ROTPK_HEADER_LEN            19
#define ARM_ROTPK_HASH_LEN              32

/* These defined for compilation. Should never be accessed. */
#define TZ_PUB_KEY_HASH_BASE            0xdeadbeef
#define TFW_NVCTR_BASE                  0xdeadbeef
#define NTFW_CTR_BASE                   0xdeadbeef

/*
 * Special value used to verify platform parameters from BL2 to BL31
 * Note: Used for debug builds.
 */
#define ARM_BL31_PLAT_PARAM_VAL         ULL(0x0f1e2d3c4b5a6978)

/* Power domain level corresponds to affinity level. */
#define RAPTOR2_PWR_LVL0                MPIDR_AFFLVL0
#define RAPTOR2_PWR_LVL1                MPIDR_AFFLVL1
#define RAPTOR2_PWR_LVL2                MPIDR_AFFLVL2

#define PLAT_MAX_PWR_LVL                RAPTOR2_PWR_LVL2
#define PLAT_MAX_OFF_STATE              RAPTOR2_PWR_LVL2
#define PLAT_MAX_RET_STATE              RAPTOR2_PWR_LVL1

#define BL2_RO_BASE                     R2_BL2_RO_BASE
#define BL2_RO_LIMIT                    R2_BL2_RO_LIMIT
#define BL2_RW_BASE                     R2_BL2_RW_BASE
#define BL2_RW_LIMIT                    R2_BL2_RW_LIMIT

/*
 * When using BL2_IN_XIP_MEM, BL2_BASE is not used.
 * However due to compilation assert() a specific value is being used.
 */
#define BL2_BASE                        0xf000000000

#define ARM_BL2_MEM_DESC_BASE           R2_MEM_DESC_BASE

#define ARM_SHARED_RAM_BASE             R2_SHARED_RAM_BASE
#define ARM_SHARED_RAM_SIZE             R2_SHARED_RAM_SIZE
#define PLAT_ARM_TRUSTED_MAILBOX_BASE   R2_MAILBOX_BASE

#define ARM_TB_FW_CONFIG_BASE           R2_FW_CONFIG_BASE
#define ARM_TB_FW_CONFIG_LIMIT          R2_FW_CONFIG_LIMIT

#define BL31_BASE                       R2_BL31_BASE
#define BL31_LIMIT                      (R2_BL31_BASE + R2_BL31_SIZE)

#define ARM_BL_RAM_BASE                 R2_TRUSTED_DRAM_BASE
#define ARM_BL_RAM_SIZE                 R2_TRUSTED_DRAM_SIZE

/* GIC related constants */
#if SINGLE_PPU_CLUSTER
#define PLAT_ARM_GICD_BASE		R2_GIC_BASE
#define PLAT_ARM_GICR_BASE		(R2_GIC_BASE + 0x1C0000)
#else
#define PLAT_ARM_GICD_BASE		R2_GIC_BASE
#define PLAT_ARM_GICR_BASE		(R2_GIC_BASE + 0x40000)
#endif

/* Non secure image (u-boot) load info */
#define PLAT_ARM_NS_IMAGE_BASE          (R2_DDR_BASE + UL(0x1000000))

/* UART Definitions */
#if RAPTOR2_FASTMODEL

/* SOCFM */
#define RAPTOR2_UART_BASE               UL(0x70400000)
#define RAPTOR2_UART_CLK_IN_HZ          7372800
#define RAPTOR2_BAUD_RATE               115200

#elif SINGLE_PPU_CLUSTER & !A0_BUILD

/* Protium Single PPU BSS UART */
#define RAPTOR2_UART_BASE               UL(0x70434000)
#define RAPTOR2_UART_CLK_IN_HZ		1843200
#define RAPTOR2_BAUD_RATE               19200

#elif A0_BUILD

/* A0 Silicon BSS UART */
#define RAPTOR2_UART_BASE               UL(0x70434000)
#define RAPTOR2_UART_CLK_IN_HZ          1500000000
#define RAPTOR2_BAUD_RATE               115200

#elif ZEBU_BUILD

#define RAPTOR2_UART_BASE               UL(0x6E450000)
#define RAPTOR2_UART_CLK_IN_HZ          1843200
#define RAPTOR2_BAUD_RATE               19200

#elif B0_BUILD

#define RAPTOR2_UART_BASE               UL(0x6E450000)
#define RAPTOR2_UART_CLK_IN_HZ          1500000000
#define RAPTOR2_BAUD_RATE               115200

#else
/* Protium 8core build */
#define RAPTOR2_UART_BASE               UL(0x70380000)
#define RAPTOR2_UART_CLK_IN_HZ          1843200
#define RAPTOR2_BAUD_RATE               19200

#endif

#define PLAT_ARM_BOOT_UART_BASE         RAPTOR2_UART_BASE
#define PLAT_ARM_BOOT_UART_CLK_IN_HZ    RAPTOR2_UART_CLK_IN_HZ
#define ARM_CONSOLE_BAUDRATE            RAPTOR2_BAUD_RATE

#define PLAT_ARM_RUN_UART_BASE          RAPTOR2_UART_BASE
#define PLAT_ARM_RUN_UART_CLK_IN_HZ     RAPTOR2_UART_CLK_IN_HZ

#define PLAT_ARM_CRASH_UART_BASE        RAPTOR2_UART_BASE
#define PLAT_ARM_CRASH_UART_CLK_IN_HZ   RAPTOR2_UART_CLK_IN_HZ

/* NOR Flash */
#define V2M_FLASH0_BASE                 R2_FLASH_BASE
#define V2M_FLASH0_SIZE                 R2_FLASH_SIZE
#define V2M_FLASH_BLOCK_SIZE            R2_FLASH_BLOCK_SIZE

/* Reserve the last block of flash for PSCI MEM PROTECT flag */
#define PLAT_ARM_FIP_BASE               V2M_FLASH0_BASE
#define PLAT_ARM_FIP_MAX_SIZE           (V2M_FLASH0_SIZE - V2M_FLASH_BLOCK_SIZE)

/*
 * TODO: Trusted Board Boot definitions
 */

/*
 * Physical and virtual address space limits for MMU in AARCH64 & AARCH32 modes
 */
#define PLAT_PHY_ADDR_SPACE_SIZE	(1ULL << 36)
#define PLAT_VIRT_ADDR_SPACE_SIZE	(1ULL << 36)

#define MAX_IO_DEVICES                  2
#define MAX_IO_BLOCK_DEVICES            2
#define MAX_IO_HANDLES                  4

/*
 *  * PLAT_ARM_MMAP_ENTRIES depends on the number of entries in the
 *   * plat_arm_mmap array defined for each BL stage.
 *    */
#if defined(IMAGE_BL32)
# define PLAT_ARM_MMAP_ENTRIES          8
# define MAX_XLAT_TABLES                6
#else
#if A0_BUILD
# define PLAT_ARM_MMAP_ENTRIES          16
# define MAX_XLAT_TABLES                10
#else
# define PLAT_ARM_MMAP_ENTRIES          12
# define MAX_XLAT_TABLES                6
#endif
#endif


/*
 * If SEPARATE_CODE_AND_RODATA=1 we define a region for each section
 * otherwise one region is defined containing both.
 */
#if SEPARATE_CODE_AND_RODATA
#define ARM_MAP_BL_RO                   MAP_REGION_FLAT(                        \
                                                BL_CODE_BASE,                   \
                                                BL_CODE_END - BL_CODE_BASE,     \
                                                MT_CODE | MT_SECURE),           \
                                        MAP_REGION_FLAT(                        \
                                                BL_RO_DATA_BASE,                \
                                                BL_RO_DATA_END                  \
                                                        - BL_RO_DATA_BASE,      \
                                                MT_RO_DATA | MT_SECURE)
#else
#define ARM_MAP_BL_RO                   MAP_REGION_FLAT(                        \
                                                BL_CODE_BASE,                   \
                                                BL_CODE_END - BL_CODE_BASE,     \
                                                MT_CODE | MT_SECURE)
#endif

/*
 * The max number of regions like RO(code), coherent and data required by
 * different BL stages which need to be mapped in the MMU.
 */
#define ARM_BL_REGIONS                  5

#define MAX_MMAP_REGIONS                (PLAT_ARM_MMAP_ENTRIES +        \
                                         ARM_BL_REGIONS)


#define ARM_IRQ_SEC_PHY_TIMER           29

#define ARM_IRQ_SEC_SGI_0               8
#define ARM_IRQ_SEC_SGI_1               9
#define ARM_IRQ_SEC_SGI_2               10
#define ARM_IRQ_SEC_SGI_3               11
#define ARM_IRQ_SEC_SGI_4               12
#define ARM_IRQ_SEC_SGI_5               13
#define ARM_IRQ_SEC_SGI_6               14
#define ARM_IRQ_SEC_SGI_7               15

#define PLAT_SDEI_NORMAL_PRI            0x70

/*
 * Define a list of Group 1 Secure and Group 0 interrupt properties as per GICv3
 * terminology. On a GICv2 system or mode, the lists will be merged and treated
 * as Group 0 interrupts.
 */
#define ARM_G1S_IRQ_PROPS(grp) \
        INTR_PROP_DESC(ARM_IRQ_SEC_PHY_TIMER, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_LEVEL), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_1, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_2, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_3, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_4, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_5, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_7, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE)

#define ARM_G0_IRQ_PROPS(grp) \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_0, PLAT_SDEI_NORMAL_PRI, (grp), \
                        GIC_INTR_CFG_EDGE), \
        INTR_PROP_DESC(ARM_IRQ_SEC_SGI_6, GIC_HIGHEST_SEC_PRIORITY, (grp), \
                        GIC_INTR_CFG_EDGE)


#define PLAT_ARM_G1S_IRQ_PROPS(grp)     ARM_G1S_IRQ_PROPS(grp)

#define PLAT_ARM_G0_IRQ_PROPS(grp)      ARM_G0_IRQ_PROPS(grp)

#endif /* PLATFORM_DEF_H */
