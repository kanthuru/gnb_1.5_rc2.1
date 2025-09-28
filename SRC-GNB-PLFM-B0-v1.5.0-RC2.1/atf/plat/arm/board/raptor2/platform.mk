#
# Copyright (c) 2018-2020, Arm Limited. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Raptor2 platform build options:
#    1. Non-secure boot
#       make PLAT=raptor2 all
#       Required build options for BL2/BL31 are set in this makefile.
#    2. Secure boot
#       make PLAT=raptor2 all fip TRUSTED_BOARD_BOOT=1 GENERATE_COT=1 ARM_ROTPK_LOCATION=<devel_rsa|prod_rsa>
#       Required build options are set in this makefile.
#       Override  ROT_KEY & ARM_ROTPK_HASH if different key/hash is to be used.
#    3. Prebuilt Secure images
#     make PLAT=raptor2 all fip LOG_LEVEL=40 A0_BUILD=1 TRUSTED_BOARD_BOOT=1 ARM_ROTPK_LOCATION=customer_rsa MBEDTLS_DIR=<mbedtls path>
#
#

# Firmware Configuration Framework sources
include lib/fconf/fconf.mk

# FDT for device tree processing
include lib/libfdt/libfdt.mk

# Memory mapping and translation table support
include lib/xlat_tables_v2/xlat_tables.mk

# PSCI run time service support
include lib/psci/psci_lib.mk

# BL1 image not required. Enforce BL1 not be built.
BL1_SOURCES =

# General environment override.
CTX_INCLUDE_AARCH32_REGS	:= 0
ARM_DISABLE_TRUSTED_WDOG        := 1
ARM_PLAT_MT                     := 1
HW_ASSISTED_COHERENCY           := 1
USE_COHERENT_MEM                := 0

# BL2 environment override
BL2_AT_EL3                      := 1
ENABLE_PIE                      := 1
BL2_IN_XIP_MEM                  := 1

# BL3 environment override
GICV3_IMPL                      := GIC600
ARM_BL31_IN_DRAM                := 1
NEOVERSE_E1_EXTERNAL_LLC        := 1

# Include GIC
include drivers/arm/gic/v3/gicv3.mk

# Sources and includes definition
RAPTOR2_BASE		 =	plat/arm/board/raptor2

PLAT_INCLUDES		+=	-I${RAPTOR2_BASE}/include/

INTERCONNECT_SOURCES    :=      ${RAPTOR2_BASE}/r2_interconnect.c

DYN_CFG_SOURCES         +=      plat/arm/common/arm_dyn_cfg.c               \
                                plat/arm/common/arm_dyn_cfg_helpers.c       \
                                common/fdt_wrappers.c

CPU_SOURCES		:=	lib/cpus/aarch64/neoverse_e1.S

# Raptor common sources for both BL2 & BL31
RAPTOR2_COMMON_SOURCES  :=      ${RAPTOR2_BASE}/r2_helpers.S                \
				${RAPTOR2_BASE}/r2_plat.c	            \
				${RAPTOR2_BASE}/r2_common.c                 \
				${INTERCONNECT_SOURCES}

# Common source for both BL2 & BL31
RAPTOR2_BL_COMMON_SOURCES       :=      ${CPU_SOURCES}                              \
				        ${RAPTOR2_COMMON_SOURCES}                   \
        				lib/utils/mem_region.c			    \
        				${RAPTOR2_BASE}/r2_console.c                \
        				plat/arm/common/arm_common.c                \
        				${XLAT_TABLES_LIB_SRCS}

ifeq (${RAPTOR2_FASTMODEL},1)
RAPTOR2_BL_COMMON_SOURCES += drivers/arm/pl011/${ARCH}/pl011_console.S
else
RAPTOR2_BL_COMMON_SOURCES += ${RAPTOR2_BASE}/r2_16550_console.S
endif

SPI_DRIVER_BASE         :=      ${RAPTOR2_BASE}/spi_nor_driver
PLAT_INCLUDES           +=      -I${SPI_DRIVER_BASE}
SPI_DRIVER_SOURCES      :=      ${SPI_DRIVER_BASE}/spi_driver.c

BL2_SOURCES		+=	${RAPTOR2_BL_COMMON_SOURCES}                \
                                ${RAPTOR2_BASE}/r2_security.c	            \
				${RAPTOR2_BASE}/r2_err.c                    \
				${RAPTOR2_BASE}/r2_bl2_mem_params_desc.c    \
				${RAPTOR2_BASE}/r2_io_storage.c             \
                                plat/arm/common/arm_image_load.c            \
				common/desc_image_load.c                    \
				drivers/io/io_fip.c                         \
				drivers/io/io_memmap.c                      \
				drivers/io/io_storage.c                     \
				drivers/io/io_block.c                       \
				plat/arm/common/arm_bl2_setup.c             \
				plat/arm/common/arm_bl2_el3_setup.c         \
				drivers/delay_timer/delay_timer.c           \
				drivers/delay_timer/generic_delay_timer.c   \
				${DYN_CFG_SOURCES}                          \
				${SPI_DRIVER_SOURCES}			    \

RAPTOR2_GIC_SOURCES         :=  ${GICV3_SOURCES}                            \
                                plat/common/plat_gicv3.c                    \
                                plat/arm/common/arm_gicv3.c


BL31_SOURCES		+=	${RAPTOR2_BL_COMMON_SOURCES}                \
                                ${RAPTOR2_BASE}/r2_security.c	            \
				${RAPTOR2_BASE}/r2_topology.c	            \
				${RAPTOR2_BASE}/r2_pm.c	                    \
				${RAPTOR2_BASE}/r2_bl31_setup.c             \
				drivers/cfi/v2m/v2m_flash.c		    \
				plat/arm/common/arm_topology.c              \
				plat/common/plat_psci_common.c              \
				${PSCI_LIB_SOURCES}                         \
                                ${RAPTOR2_GIC_SOURCES}

ifeq (${TRUSTED_BOARD_BOOT}, 1)

include drivers/auth/mbedtls/mbedtls_crypto.mk
include drivers/auth/mbedtls/mbedtls_x509.mk

AUTH_SOURCES        :=      drivers/auth/auth_mod.c                         \
                            drivers/auth/crypto_mod.c                       \
                            drivers/auth/img_parser_mod.c                   \
                            lib/fconf/fconf_tbbr_getter.c                   \
			    plat/common/tbbr/plat_tbbr.c                    \
			    drivers/auth/tbbr/tbbr_cot.c                    \
			    plat/arm/board/common/board_arm_trusted_boot.c

ifeq (${ARM_ROTPK_LOCATION}, devel_rsa)
AUTH_SOURCES	+=	plat/arm/board/raptor2/include/r2_dev_rotpk.S
endif

BL2_SOURCES	    +=	    ${RAPTOR2_BASE}/r2_trusted_boot.c           \
                            ${AUTH_SOURCES}


# Below particular configuration is for mbedtls.
MBEDTLS_NO_UDBL_DIVISION := 1


# ROTPK hash location
ifeq (${ARM_ROTPK_LOCATION}, devel_rsa)
	# Secure boot specific compilation options enabled by default for Raptor2
        COT                  := tbbr
        GENERATE_COT         := 1
        SAVE_KEYS            := 1
        CREATE_KEYS          := 1
        TRUSTED_WORLD_KEY    := twk
        NON_TRUSTED_WORLD_KEY := ntwk
        CRYPTO_ALG              := rsa
        ARM_ROTPK_LOCATION_ID   := ARM_ROTPK_DEVEL_RSA_ID
	# Root certificate and the hash value available in ATF source code will be used.
	# Override these values to support new root certificates.
        ROT_KEY                 := plat/arm/board/common/rotpk/arm_rotprivk_rsa.pem
        ARM_ROTPK_HASH          := plat/arm/board/common/rotpk/arm_rotpk_rsa_sha256.bin
    # Add BL2 CoT (image cert)
        $(eval $(call TOOL_ADD_PAYLOAD,${BUILD_PLAT}/tb_fw.crt,--tb-fw-cert))

        $(warning Development keys support only for dev purpose. Not for production. Use `prod_rsa` option instead)
else ifeq (${ARM_ROTPK_LOCATION}, prod_rsa)
# Secure boot specific compilation options enabled by default for Raptor2
        COT                  := tbbr
        GENERATE_COT         := 1
        #SAVE_KEYS            := 1
        #CREATE_KEYS          := 1
        TRUSTED_WORLD_KEY    := twk
        NON_TRUSTED_WORLD_KEY := ntwk

        ifeq (${ROT_KEY},)
            $(error no ROT_KEY defined)
        endif
        # Add BL2 CoT (image cert)
            $(eval $(call TOOL_ADD_PAYLOAD,${BUILD_PLAT}/tb_fw.crt,--tb-fw-cert))
else ifeq (${ARM_ROTPK_LOCATION}, customer_rsa)
		COT                  := tbbr
endif

# Include common TBB sources
$(eval $(call add_define,ARM_ROTPK_LOCATION_ID))

$(eval $(call add_define_val,ARM_ROTPK_HASH,'"$(ARM_ROTPK_HASH)"'))
$(BUILD_PLAT)/bl2/r2_dev_rotpk.o : $(ARM_ROTPK_HASH)

# TBBR based COT to be supported.
$(eval $(call add_define,ARM_COT_${COT}))

$(eval $(call add_define,MBEDTLS_NO_UDBL_DIVISION))

endif # End of TRUSTED_BOARD_BOOT

# Add the FDT_SOURCES and options for Dynamic Config
FDT_SOURCES		+=	${RAPTOR2_BASE}/fdts/r2_fw_config.dts
TB_FW_CONFIG		:=	${BUILD_PLAT}/fdts/r2_fw_config.dtb

# Add the TB_FW_CONFIG to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_PAYLOAD,${TB_FW_CONFIG},--tb-fw-config))

FDT_SOURCES		+=	${RAPTOR2_BASE}/fdts/r2_nt_fw_config.dts
NT_FW_CONFIG		:=	${BUILD_PLAT}/fdts/r2_nt_fw_config.dtb

# Add the NT_FW_CONFIG to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_PAYLOAD,${NT_FW_CONFIG},--nt-fw-config))

# Raptor specific compilation flags
$(eval $(call assert_boolean,NEOVERSE_E1_EXTERNAL_LLC))
$(eval $(call add_define,NEOVERSE_E1_EXTERNAL_LLC))

$(eval $(call assert_boolean,ARM_PLAT_MT))
$(eval $(call add_define,ARM_PLAT_MT))

R2_PRELOADED_BL31_BASE := 0
R2_PRELOADED_BL33_BASE := 0

RAPTOR2_FASTMODEL := 0
PROTIUM_BUILD     := 0
ZEBU_BUILD := 0
B0_BUILD := 0
HTG_BUILD         := 0
SINGLE_PPU_CLUSTER := 0
A0_BUILD	:= 0
A0_PEGASUS_BUILD:= 0
SIF_CONFIG	:= 0
IOSS_MUX_CONFIG	:= 0
EMULATED_FLASH	:= 0
E1_DDR_BOOT := 1

$(eval $(call assert_boolean,R2_PRELOADED_BL31_BASE))
$(eval $(call add_define,R2_PRELOADED_BL31_BASE))
$(eval $(call assert_boolean,R2_PRELOADED_BL33_BASE))
$(eval $(call add_define,R2_PRELOADED_BL33_BASE))
$(eval $(call assert_boolean,RAPTOR2_FASTMODEL))
$(eval $(call add_define,RAPTOR2_FASTMODEL))
$(eval $(call assert_boolean,PROTIUM_BUILD))
$(eval $(call add_define,PROTIUM_BUILD))
$(eval $(call assert_boolean,ZEBU_BUILD))
$(eval $(call add_define,ZEBU_BUILD))
$(eval $(call assert_boolean,B0_BUILD))
$(eval $(call add_define,B0_BUILD))
$(eval $(call assert_boolean,HTG_BUILD))
$(eval $(call add_define,HTG_BUILD))
$(eval $(call assert_boolean,SINGLE_PPU_CLUSTER))
$(eval $(call add_define,SINGLE_PPU_CLUSTER))
$(eval $(call assert_boolean,A0_BUILD))
$(eval $(call add_define,A0_BUILD))
$(eval $(call assert_boolean,A0_PEGASUS_BUILD))
$(eval $(call add_define,A0_PEGASUS_BUILD))
$(eval $(call assert_boolean,EMULATED_FLASH))
$(eval $(call add_define,EMULATED_FLASH))
$(eval $(call assert_boolean,E1_DDR_BOOT))
$(eval $(call add_define,E1_DDR_BOOT))

ifeq (${PROTIUM_BUILD}, 1)
SIF_CONFIG :=1
endif

ifeq (${SINGLE_PPU_CLUSTER}, 1)
SIF_CONFIG :=1
IOSS_MUX_CONFIG	:= 1
endif

ifeq (${A0_BUILD}, 1)
SIF_CONFIG :=1
IOSS_MUX_CONFIG	:= 1
endif

ifeq (${B0_BUILD}, 1)
SIF_CONFIG :=1
IOSS_MUX_CONFIG	:= 1
endif

ifeq (${EMULATED_FLASH}, 1)
	FIP_NAME := fip_mem.bin
else
	FIP_NAME := fip.bin
endif

$(eval $(call assert_boolean,SIF_CONFIG))
$(eval $(call add_define,SIF_CONFIG))

$(eval $(call assert_boolean,IOSS_MUX_CONFIG))
$(eval $(call add_define,IOSS_MUX_CONFIG))

ifeq (${SIF_CONFIG}, 1)
PLAT_INCLUDES		+=	-I${RAPTOR2_BASE}/sif/
BL31_SOURCES		+=	${RAPTOR2_BASE}/sif/sif_config.c	\
				${RAPTOR2_BASE}/sif/sif_routing.c
endif

ifeq (${IOSS_MUX_CONFIG}, 1)
PLAT_INCLUDES		+=	-I${RAPTOR2_BASE}/ioss_mux/
BL31_SOURCES		+=	${RAPTOR2_BASE}/ioss_mux/ioss_mux_config.c
endif

# Add BL2 image to FIP
FIP_BL2_ARGS += tb-fw
