
#include <assert.h>
#include <errno.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/io/io_block.h>
#include <drivers/io/io_driver.h>
#include <drivers/io/io_fip.h>
#include <drivers/io/io_memmap.h>
#include <drivers/io/io_storage.h>
#include <lib/mmio.h>
#include <tools_share/firmware_image_package.h>
#include <spi_driver.h>

#if ((A0_BUILD == 0) && (ZEBU_BUILD == 0) && (B0_BUILD == 0))
/* FPGA Hack to use fastmodel based IO (memmap()) */
#if !RAPTOR2_FASTMODEL
#undef RAPTOR2_FASTMODEL
#define RAPTOR2_FASTMODEL 1
#endif
#endif

#if B0_BUILD
#define MBR_HEADER		0xAA55AA55
#define MBR_FOOTER		0x55AA55AA
#define FAILSAFE_PARTITION_ID	1
#define PARTITION_A_ID		2
#define PARTITION_B_ID		3
#define FLASH_BASE_OFFSET	0
#define FIP_OFFSET		0x100000

#define R2_ACTIVE_PARTITION_OFFSET   0x200	/* to store active partition */

typedef struct __RAPTOR2_MBR {
    uint32_t header;		/* 0xAA55AA55 */
    uint32_t active_partition;	/* 1=failsafe, 2=part_A 3=part_B */
    uint32_t failSafe_present;	/* 1= present */
    uint32_t failSafe_offset;
    uint32_t failsafe_size;
    uint32_t partA_offset;
    uint32_t partA_size;
    uint32_t partB_offset;
    uint32_t partB_size;
    uint32_t flash_addr_len;	/* 3 for standard(3-byte) address, 4 for for extended(4-byte) address */
    uint32_t rsv[53];
    uint32_t footer;		/* footer = 0x55AA55AA */
} RAPTOR2_MBR;

RAPTOR2_MBR mbr __attribute__((aligned(4)));
#endif

/* IO devices */
static const io_dev_connector_t *fip_dev_con;
uintptr_t fip_dev_handle;

struct plat_io_policy {
	uintptr_t *dev_handle;
	uintptr_t image_spec;
	int (*check)(const uintptr_t spec);
};

int open_fip(const uintptr_t spec);

#if RAPTOR2_FASTMODEL
static const io_dev_connector_t *memmap_dev_con;
uintptr_t memmap_dev_handle;

const io_block_spec_t r2_fip_block_spec = {
	.offset = PLAT_ARM_FIP_BASE,
	.length = PLAT_ARM_FIP_MAX_SIZE
};

int open_memmap(const uintptr_t spec);

#else
static const io_dev_connector_t *r2_spinor_dev_con;
static uintptr_t r2_spinor_dev_handle;

/* Fip base needs to be updated by reading MBR for B0 Titan */
static io_block_spec_t r2_spinor_fip_block_spec = {
	.offset = PLAT_ARM_FIP_BASE,
	.length = PLAT_ARM_FIP_MAX_SIZE
};


static int open_r2_spinor(const uintptr_t spec);
size_t r2_spinor_read_blks(int lba, uintptr_t buf, size_t size);
size_t r2_spinor_write_blks(int lba, const uintptr_t buf, size_t size);

static const io_block_dev_spec_t r2_spinor_dev_spec = {
        /* It's used as temp buffer in block driver. */
	.buffer         = {
		.offset = R2_FLASH_CACHE_DATA_BASE,
		.length = R2_FLASH_CACHE_DATA_SIZE,
        },
        .ops            = {
		.read   = r2_spinor_read_blks,
        },
        .block_size     = R2_FLASH_BLOCK_SIZE,
};

#endif

const io_uuid_spec_t r2_uuid_spec[MAX_NUMBER_IDS] = {
	[BL2_IMAGE_ID] = {UUID_TRUSTED_BOOT_FIRMWARE_BL2},
	[TB_FW_CONFIG_ID] = {UUID_TB_FW_CONFIG},
	[BL31_IMAGE_ID] = {UUID_EL3_RUNTIME_FIRMWARE_BL31},
	[BL33_IMAGE_ID] = {UUID_NON_TRUSTED_FIRMWARE_BL33},
	[HW_CONFIG_ID] = {UUID_HW_CONFIG},
	[SOC_FW_CONFIG_ID] = {UUID_SOC_FW_CONFIG},
	[TOS_FW_CONFIG_ID] = {UUID_TOS_FW_CONFIG},
	[NT_FW_CONFIG_ID] = {UUID_NT_FW_CONFIG},
#if TRUSTED_BOARD_BOOT
	[TRUSTED_BOOT_FW_CERT_ID] = {UUID_TRUSTED_BOOT_FW_CERT},
	[TRUSTED_KEY_CERT_ID] = {UUID_TRUSTED_KEY_CERT},
	[SOC_FW_KEY_CERT_ID] = {UUID_SOC_FW_KEY_CERT},
	[NON_TRUSTED_FW_KEY_CERT_ID] = {UUID_NON_TRUSTED_FW_KEY_CERT},
	[SOC_FW_CONTENT_CERT_ID] = {UUID_SOC_FW_CONTENT_CERT},
	[NON_TRUSTED_FW_CONTENT_CERT_ID] = {UUID_NON_TRUSTED_FW_CONTENT_CERT},
#endif /* TRUSTED_BOARD_BOOT */
	/* Raptor2 private images */
	[DDR_CTRL_FW_IMAGE1_ID] = {UUID_DDR_CTRL_FIRMWARE_IMAGE_1},
	[DDR_CTRL_FW_IMAGE2_ID] = {UUID_DDR_CTRL_FIRMWARE_IMAGE_2},
	[DDR_CTRL_FW_IMAGE3_ID] = {UUID_DDR_CTRL_FIRMWARE_IMAGE_3},
	[DDR_CTRL_FW_IMAGE4_ID] = {UUID_DDR_CTRL_FIRMWARE_IMAGE_4},
};

/* By default, ARM platforms load images from the FIP */
struct plat_io_policy policies[MAX_NUMBER_IDS] = {
	[FIP_IMAGE_ID] = {
#if RAPTOR2_FASTMODEL
		&memmap_dev_handle,
		(uintptr_t)&r2_fip_block_spec,
		open_memmap
#else
		&r2_spinor_dev_handle,
		(uintptr_t)&r2_spinor_fip_block_spec,
		open_r2_spinor
#endif
	},
	[BL2_IMAGE_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[BL2_IMAGE_ID],
		open_fip
	},
	[TB_FW_CONFIG_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[TB_FW_CONFIG_ID],
		open_fip
	},
	[BL31_IMAGE_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[BL31_IMAGE_ID],
		open_fip
	},
	[BL33_IMAGE_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[BL33_IMAGE_ID],
		open_fip
	},
	[HW_CONFIG_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[HW_CONFIG_ID],
		open_fip
	},
	[SOC_FW_CONFIG_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[SOC_FW_CONFIG_ID],
		open_fip
	},
	[TOS_FW_CONFIG_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[TOS_FW_CONFIG_ID],
		open_fip
	},
	[NT_FW_CONFIG_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[NT_FW_CONFIG_ID],
		open_fip
	},
#if TRUSTED_BOARD_BOOT
	[TRUSTED_BOOT_FW_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[TRUSTED_BOOT_FW_CERT_ID],
		open_fip
	},
	[TRUSTED_KEY_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[TRUSTED_KEY_CERT_ID],
		open_fip
	},
	[SOC_FW_KEY_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[SOC_FW_KEY_CERT_ID],
		open_fip
	},
	[NON_TRUSTED_FW_KEY_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[NON_TRUSTED_FW_KEY_CERT_ID],
		open_fip
	},
	[SOC_FW_CONTENT_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[SOC_FW_CONTENT_CERT_ID],
		open_fip
	},
	[NON_TRUSTED_FW_CONTENT_CERT_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[NON_TRUSTED_FW_CONTENT_CERT_ID],
		open_fip
	},
#endif /* TRUSTED_BOARD_BOOT */
	[DDR_CTRL_FW_IMAGE1_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[DDR_CTRL_FW_IMAGE1_ID],
		open_fip
	},
	[DDR_CTRL_FW_IMAGE2_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[DDR_CTRL_FW_IMAGE2_ID],
		open_fip
	},
	[DDR_CTRL_FW_IMAGE3_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[DDR_CTRL_FW_IMAGE3_ID],
		open_fip
	},
	[DDR_CTRL_FW_IMAGE4_ID] = {
		&fip_dev_handle,
		(uintptr_t)&r2_uuid_spec[DDR_CTRL_FW_IMAGE4_ID],
		open_fip
	},
};


int open_fip(const uintptr_t spec)
{
	int result;
	uintptr_t local_image_handle;

	/* See if a Firmware Image Package is available */
	result = io_dev_init(fip_dev_handle, (uintptr_t)FIP_IMAGE_ID);
	if (result == 0) {
		result = io_open(fip_dev_handle, spec, &local_image_handle);
		if (result == 0) {
			VERBOSE("Using FIP\n");
			io_close(local_image_handle);
		}
	}
	return result;
}

#if RAPTOR2_FASTMODEL
int open_memmap(const uintptr_t spec)
{
	int result;
	uintptr_t local_image_handle;

	result = io_dev_init(memmap_dev_handle, (uintptr_t)NULL);
	if (result == 0) {
		result = io_open(memmap_dev_handle, spec, &local_image_handle);
		if (result == 0) {
			VERBOSE("Using Memmap\n");
			io_close(local_image_handle);
		}
	}
	return result;
}
#else
static int open_r2_spinor(const uintptr_t spec)
{
	int result;
	uintptr_t local_image_handle;

	result = io_dev_init(r2_spinor_dev_handle, (uintptr_t)NULL);
	if (result == 0) {
		result = io_open(r2_spinor_dev_handle, spec, &local_image_handle);
		if (result == 0) {
			VERBOSE("Using R2 SPINOR\n");
			io_close(local_image_handle);
		}
	}
	return result;
}
#endif

int arm_io_setup(void)
{
	int io_result;
	int active_partition;

	io_result = register_io_dev_fip(&fip_dev_con);
	if (io_result < 0) {
		return io_result;
	}

	/* Open connections to devices and cache the handles */
	io_result = io_dev_open(fip_dev_con, (uintptr_t)NULL,
				&fip_dev_handle);
	if (io_result < 0) {
		return io_result;
	}

#if RAPTOR2_FASTMODEL
	io_result = register_io_dev_memmap(&memmap_dev_con);
	if (io_result < 0) {
		return io_result;
	}

	io_result = io_dev_open(memmap_dev_con, (uintptr_t)NULL,
				&memmap_dev_handle);
#else

#if EMULATED_FLASH
	printf("Using Emulated flash\n");
#else
	spi_init();
#endif

#if B0_BUILD
	spi_read((uint32_t *)(&mbr), FLASH_BASE_OFFSET, sizeof(RAPTOR2_MBR));
	if ((mbr.header != MBR_HEADER) || (mbr.footer != MBR_FOOTER)) {
		printf(" SPL: failed to read MBR \r\n");
		return -1;
	}

	active_partition = (*(volatile uint64_t *)(DDR_BASE + R2_ACTIVE_PARTITION_OFFSET));

	if (active_partition == FAILSAFE_PARTITION_ID)
		r2_spinor_fip_block_spec.offset = mbr.failSafe_offset + FIP_OFFSET;
	else if (active_partition == PARTITION_A_ID)
		r2_spinor_fip_block_spec.offset = mbr.partA_offset + FIP_OFFSET;
	else if (active_partition == PARTITION_B_ID)
		r2_spinor_fip_block_spec.offset = mbr.partB_offset + FIP_OFFSET;
#endif
	io_result = register_io_dev_block(&r2_spinor_dev_con);
	if( io_result < 0 )
	{
		return io_result;
	}

	io_result = io_dev_open(r2_spinor_dev_con, (uintptr_t)&r2_spinor_dev_spec,
				&r2_spinor_dev_handle);
	if( io_result < 0 )
	{
		return io_result;
	}
#endif

	return io_result;
}

void plat_arm_io_setup(void)
{
	int err;

	err = arm_io_setup();
	if (err < 0) {
		panic();
	}
}

int plat_arm_get_alt_image_source(
	unsigned int image_id __unused,
	uintptr_t *dev_handle __unused,
	uintptr_t *image_spec __unused)
{
	/* By default do not try an alternative */
	return -ENOENT;
}

/* Return an IO device handle and specification which can be used to access
 * an image. Use this to enforce platform load policy */
int plat_get_image_source(unsigned int image_id, uintptr_t *dev_handle,
			  uintptr_t *image_spec)
{
	int result;
	const struct plat_io_policy *policy;

	assert(image_id < ARRAY_SIZE(policies));

	policy = &policies[image_id];

	result = policy->check(policy->image_spec);
	if (result == 0) {
		*image_spec = policy->image_spec;
		*dev_handle = *(policy->dev_handle);
        }

        return result;
}

/*
 * See if a Firmware Image Package is available,
 * by checking if TOC is valid or not.
 */
bool arm_io_is_toc_valid(void)
{
	return (io_dev_init(fip_dev_handle, (uintptr_t)FIP_IMAGE_ID) == 0);
}

#if !RAPTOR2_FASTMODEL
/*
 * Read function called by generic block driver.
 * lba - Starting block size.
 * buf - cache buffer to read into.
 * size - number of bytes.
 *
 * This function will inturn call the SPI-NOR driver specific read function.
 * Note, src/dstAddr are determined as per SPI-NOR driver.
 */
size_t r2_spinor_read_blks(int lba, uintptr_t buf, size_t size)
{
	uint32_t  *srcAddr;
	uint32_t  offset;
	uint32_t  *dstAddr;

	offset = lba * r2_spinor_dev_spec.block_size;

#if (EMULATED_FLASH == 0)
	if( offset > (r2_spinor_fip_block_spec.offset + r2_spinor_fip_block_spec.length))
	{
		printf("%s %d Error: Fip size out of bound\n", __func__, __LINE__);
		panic();
	}
#endif
	srcAddr = (uint32_t *)(uintptr_t)offset;
	dstAddr = (uint32_t *)buf;

	/* if flash is emulated in some other memmory */
#if EMULATED_FLASH
	srcAddr = (uint32_t *)(R2_FLASH_BASE_EMULATED + offset);
	memcpy(dstAddr, srcAddr, size);

	return size;
#else
        return spi_read( dstAddr, srcAddr, size);
#endif
}
#endif
