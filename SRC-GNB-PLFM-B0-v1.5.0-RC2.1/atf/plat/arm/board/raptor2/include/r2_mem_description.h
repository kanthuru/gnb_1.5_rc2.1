#ifndef R2_MEM_DESCRIPTION_H
#define R2_MEM_DESCRIPTION_H

/*
 *  This file describes the memory layout and defines the required memory areas.
 */

/*
 * Physical memory layout. <xx> MB OCM memory and <xx> MB DDR memory.

                   DDR
    0xffffffff +----------+  <<<< 2 GB
               |          |
    0x40000000 +----------+

                   OCM
    0x180080000 +----------+  <<<< 512 KB
                |          |
    0x180000000 +----------+

    		   GIC
    0x150000000 +----------+  <<<< 256 MB
                |          |
    0x140000000 +----------+
*/

#if A0_BUILD
#define DDR_BASE	ULL(0x400000000)
#elif B0_BUILD
#define DDR_BASE	ULL(0x800000000)
#endif

#if  RAPTOR2_FASTMODEL
#define  R2_OCM_BASE       ULL(0x1800000000)
#define  R2_OCM_SIZE       0x00080000   /* 512 KB */
#define  R2_OCM_LIMIT      (R2_OCM_BASE + R2_OCM_SIZE)

#define  R2_DDR_BASE       ULL(0x400000000)
#define  R2_DDR_SIZE       ULL(0x80000000)   /* 2GB */
#define  R2_DDR_LIMIT      (R2_DDR_BASE + R2_DDR_SIZE)
#else

#if HTG_BUILD
#define  R2_OCM_BASE       ULL(0x400000000)
#endif
#if E1_DDR_BOOT
#define  R2_OCM_BASE        (DDR_BASE + ULL(0x1000000))
#else
#define  R2_OCM_BASE       ULL(0x180000000)
#endif
#define  R2_OCM_SIZE       0x00080000   /* 512 KB */
#define  R2_OCM_LIMIT      (R2_OCM_BASE + R2_OCM_SIZE)

#if HTG_BUILD
#define  R2_DDR_BASE       ULL(0x400200000)
#define  R2_DDR_SIZE       ULL(0x40000000 - 0x1000000)
#else
#define  R2_DDR_BASE       DDR_BASE
#define  R2_DDR_SIZE       ULL(0x4000000)
#endif
#define  R2_DDR_LIMIT      (R2_DDR_BASE + R2_DDR_SIZE)
#endif

#define  R2_GIC_BASE       ULL(0x140000000)
#define  R2_GIC_SIZE       0x10000000

#define  R2_OCM_BL31_BASE       ULL(0x180000000)
#define	 BSS_RUNTIME_OFFSET_OCM  0x1C000
#define  BSS_RUNTIME_OFFSET_DDR  0xF00000

/*
 * Logical memory layout.
 *
 * ATF memory binaries use three different memory areas to run binaries.
 *     1. Trusted ROM
 *     2. Trusted SRAM
 *     3. Trusted DRAM
 *     4. Non Secure DRAM
 *
 * We shall define the available memory into these areas. 

                   DRAM
    0xffffffff +----------+
               :          : (non-secure) <<<<  2037 MB
    0x40a00000 |----------|  
               |          | (secure) <<<<  10 MB
    0x40000000 +----------+

                Trusted SRAM
    0x40400000  +----------+  <<<< 512 KB
                |          |
    0x180000000 +----------+

                Trusted ROM
    0x04000000 +----------+
               |          |
    0x00000000 +----------+

 */

/* ROM area is not used R2 implementation. It is defined for understanding and completion purpose */
#define  R2_TRUSTED_ROM_BASE    0x00000000
#define  R2_TRUSTED_ROM_SIZE    0x00200000  /* 2MB */
#define  R2_TRUSTED_ROM_LIMIT   0x00200000 

#define  R2_TRUSTED_SRAM_BASE     R2_OCM_BASE
#define  R2_TRUSTED_SRAM_SIZE     R2_OCM_SIZE
#define  R2_TRUSTED_SRAM_LIMIT    (R2_TRUSTED_SRAM_BASE + R2_TRUSTED_SRAM_SIZE)

#define  R2_TRUSTED_DRAM_BASE     R2_DDR_BASE
#define  R2_TRUSTED_DRAM_SIZE     0x01000000       /* 16MB */
#define  R2_TRUSTED_DRAM_LIMIT    (R2_TRUSTED_DRAM_BASE + R2_TRUSTED_DRAM_SIZE)

/* Starting from 17 GB. */
#define  R2_NS_DRAM_BASE          R2_TRUSTED_DRAM_LIMIT
#define  R2_NS_DRAM_SIZE          (R2_DDR_SIZE - R2_TRUSTED_DRAM_SIZE)
#define  R2_NS_DRAM_LIMIT         (R2_NS_DRAM_BASE + R2_NS_DRAM_SIZE)

/* TODO: Any CASSERT() required to catch memory overwrite? */


/*
 * BLxx image memory description.
 * BL2 & BL31 images require the base and limit address to be defined.
 * Note: Raptor platform does not use BL1.
 *       BL2 & BL31 may not use the complete available SRAM/DRAM memory.
 *       Respective sizes are defined for the purpose.

 * CONFIG section in memory layout contains:
    +--------------------+
    |bl2_mem_params_descs|
    |--------------------|
    |     fw_configs     |
    +--------------------+

 * We shall define the memory layout for each of the images.

                   DRAM
    0xffffffff +----------+
               :          :
               |----------|
               |HW_CONFIG |
    0x80a00000 |----------|  (non-secure)
               :          :
               :          :  loaded by BL2  +----------------+  9 MB
               |          |  <<<<<<<<<<<<<  |  BL31 NOBITS   |
               |          |  <<<<<<<<<<<<<  |----------------|
               |          |  <<<<<<<<<<<<<  | BL31 PROGBITS  |
               |          |  <<<<<<<<<<<<<  +----------------+
    0x80001000 +----------+
               |  Shared  |  <<<< 4 KB
    0x80000000 +----------+

                Trusted SRAM
    0x180024000 +----------+
                |  CONFIG  |  <<<< 4 KB
    0x180023000 +----------+
                |   BL2 RW |  <<<< 60 KB
    0x180014000 :----------:
                |   BL2 RO |  <<<< 80 KB
    0x180000000 +----------+

 *
 */

/* BL2 is XIP. RO & RW parameters are to be defined separately. */
#define  R2_BL2_RO_BASE        R2_TRUSTED_SRAM_BASE
#define  R2_BL2_RO_SIZE        0x00014000          /* 80 KB trusted boot */
#define  R2_BL2_RO_LIMIT       (R2_BL2_RO_BASE + R2_BL2_RO_SIZE)

#define  R2_BL2_RW_BASE        R2_BL2_RO_LIMIT
#define  R2_BL2_RW_SIZE        0x00014000          /* 80 KB trusted boot */
#define  R2_BL2_RW_LIMIT       (R2_BL2_RW_BASE + R2_BL2_RW_SIZE)

/* CONFIG Section used only by BL2. */
#define  R2_CONFIG_RAM_BASE    R2_BL2_RW_LIMIT
#define  R2_CONFIG_RAM_SIZE    0x00001000          /* 4 KB */
#define  R2_CONFIG_RAM_LIMIT   (R2_CONFIG_RAM_BASE + R2_CONFIG_RAM_SIZE)

#define  R2_FW_CONFIG_BASE     R2_CONFIG_RAM_BASE
#define  R2_FW_CONFIG_SIZE     0x00000800          /* 2 KB */
#define  R2_FW_CONFIG_LIMIT    (R2_FW_CONFIG_BASE + R2_FW_CONFIG_SIZE)

#define  R2_MEM_DESC_BASE      R2_FW_CONFIG_LIMIT
#define  R2_MEM_DESC_SIZE      0x00000800          /* 2 KB */
#define  R2_MEM_DESC_LIMIT     (R2_MEM_DESC_BASE + R2_MEM_DESC_SIZE)

#define  R2_SHARED_RAM_BASE    R2_TRUSTED_DRAM_BASE
#define  R2_SHARED_RAM_SIZE    0x00001000          /* 4 KB */
#define  R2_SHARED_RAM_LIMIT   (R2_SHARED_RAM_BASE + R2_SHARED_RAM_SIZE)


/*
 * TODO: Define contents of Shared memory.
 *       Mailbox for each of the CPU are to be defined here.
 */
#define  R2_MAILBOX_BASE       R2_SHARED_RAM_BASE
#define  R2_MAILBOX_CPU_0      (R2_SHARED_RAM_BASE + 0)
#define  R2_MAILBOX_CPU_1      (R2_SHARED_RAM_BASE + 8)
 
#define  R2_BL31_BASE          R2_OCM_BL31_BASE
#define  R2_BL31_SIZE          0x0001C000          /* 112KB */
#define  R2_BL31_LIMIT         (R2_BL31_BASE + R2_BL31_SIZE)

#define  R2_BL33_SIZE          0x00400000         /* TODO: Assuming 4 MB u-boot size. */

/* SPI NOR Flash memory description */
#if RAPTOR2_FASTMODEL
/* In Fastmodel, we use memory mapped driver. */
#define  R2_FLASH_BASE                 UL(0x18000000)
#define  R2_FLASH_SIZE                 UL(0x08000000)
#define  R2_FLASH_BLOCK_SIZE           UL(0x00040000)  /* 256 KB */

#else
/* SoC shall use SPI NOR driver. Definitions related to it. */
//#define  R2_FLASH_BASE                 UL(0x00780000)
//#define  R2_FLASH_SIZE                 UL(0x00080000)  /* TODO: This needs to be verified on SoC */
//#define  R2_FLASH_BLOCK_SIZE           UL(0x00040000)  /* 256 KB */

/* FPGA implementation for now. */
#if HTG_BUILD
#define  R2_FLASH_BASE                 UL(0x400100000)
#define  R2_FLASH_SIZE                 UL(0x00080000)  /* TODO: This needs to be verified on SoC */
#elif A0_BUILD
#if EMULATED_FLASH
#define  R2_FLASH_BASE                 UL(0x00000000)  /* TODO: Update naming convention to offset */
#else
#define  R2_FLASH_BASE                 UL(0x00100000)  /* Load fip at 1MB offset */
#endif
#define  R2_FLASH_BASE_EMULATED	       UL(0x1C880000)
#define  R2_FLASH_SIZE                 UL(0x00100000)	/* 1MB */
#elif (ZEBU_BUILD || B0_BUILD)
#define  R2_FLASH_BASE_EMULATED        UL(0x1C880000)  /*TBD if to use emulated flash or fip mem driver, if emulated flash is used
                                                        R2_FLASH_BASE would be 0x00000000 & EMULATED_FLASH=1*/
#if EMULATED_FLASH
#define  R2_FLASH_BASE                 UL(0x00000000)  /* Zebu build */
#else
#define  R2_FLASH_BASE                 UL(0x00380000)  /* Load fip at 3.5MB offset, based on active partition it would get changed */
#endif
#define  R2_FLASH_SIZE                 UL(0x00180000)	/* 1.5MB */

#else   /*Protium build*/
#define  R2_FLASH_BASE                 UL(0x68000000)  /* SPU SU memory : doc ref : R2A0_SU_Config*/
#define  R2_FLASH_SIZE                 UL(0x00100000)  /* TODO: This needs to be verified on SoC */
#endif

#define  R2_FLASH_BLOCK_SIZE           UL(0x00000200)  /* 512 Bytes */

/* Cache memory for block driver. */
#define  R2_FLASH_CACHE_DATA_SIZE      0x00100000      /* 1MB at end of 14th MB of secure memory */
#define  R2_FLASH_CACHE_DATA_BASE      (R2_TRUSTED_DRAM_LIMIT - (2 * R2_FLASH_CACHE_DATA_SIZE))
#define  R2_FLASH_CACHE_DATA_LIMIT     (R2_FLASH_CACHE_DATA_BASE + R2_FLASH_CACHE_DATA_SIZE)
#endif

#if IOSS_MUX_CONFIG
/* IOSS Mux range */
#define R2_IOSS_MUX_BASE		UL(0x3CB20000)
#define R2_IOSS_MUX_SIZE		UL(0x20000)
#endif

#if SIF_CONFIG
/* SIF memory range */
#define R2_SIF_BASE			UL(0x69080000)
#define R2_SIF_SIZE			UL(0x80000)
#endif

/* Peripheral memory range */
#if A0_BUILD
#define  R2_PERIPHERAL_BASE            UL(0x70000000)
#elif B0_BUILD
#define  R2_PERIPHERAL_BASE            UL(0x6E000000)
#endif
#define  R2_PERIPHERAL_SIZE            UL(0x10000000)
#define  R2_PERIPHERAL_LIMIT           (R2_PERIPHERAL_BASE + R2_PERIPHERAL_SIZE)

/*
 * Raptor2 specific image load details.
 * TODO: Images placed at higher address secure DRAM. To be corrected.
 *       Max size set to 20 KB. To be corrected.
 */

#define  R2_DDR_CTRL_FW_IMAGE1_BASE    UL(0x400900000)
#define  R2_DDR_CTRL_FW_IMAGE2_BASE    UL(0x400905000)
#define  R2_DDR_CTRL_FW_IMAGE3_BASE    UL(0x40090a000)
#define  R2_DDR_CTRL_FW_IMAGE4_BASE    UL(0x40090f000)
#if A0_BUILD
#define  R2_RUNTIME_BSS_IMAGE_BASE     (R2_OCM_BL31_BASE + BSS_RUNTIME_OFFSET_OCM)
#elif B0_BUILD
#define  R2_RUNTIME_BSS_IMAGE_BASE     (R2_OCM_BL31_BASE + BSS_RUNTIME_OFFSET_OCM)
#endif

#define  R2_DDR_CTRL_FW_IMAGE1_SIZE    UL(0x5000)
#define  R2_DDR_CTRL_FW_IMAGE2_SIZE    UL(0x5000)
#define  R2_DDR_CTRL_FW_IMAGE3_SIZE    UL(0x5000)
#define  R2_DDR_CTRL_FW_IMAGE4_SIZE    UL(0x5000)

#define R2_EEPROM_BUFFER_OFFSET		0xFD0000
#define R2_EEPROM_BUFFER_ADDR			(R2_DDR_BASE + R2_EEPROM_BUFFER_OFFSET)
#define R2_EEPROM_BUFFER_BOOTMODE_OFFSET	(R2_EEPROM_BUFFER_ADDR + 0x4C)

#define NONSECURE_MODE	0
#define SECURE_MODE	1

#endif  /* R2_MEM_DESCRIPTION_H */
