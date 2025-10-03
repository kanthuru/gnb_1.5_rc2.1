/* SPDX-License-Identifier: GPL-2.0+
 *
 * Copyright (c) 2020 EdgeQ
 */

#include <common.h>
#include <i2c.h>
#include <dm.h>
#include <spl.h>
#include <init.h>
#include <miiphy.h>
#include <fdt_support.h>
#include <asm/armv8/mmu.h>
#include <asm/io.h>
#include <spi.h>
#include <malloc.h>
#include <linux/delay.h>
#include <mmc.h>
#include <part.h>
#include <log.h>
#include <configs/edgeq_raptor2.h>
#include "../../../drivers/net/eqxgbe/xgbe_xgmac.h"
#include "dpll_app_config.h"
#include "raptor_set_vdd.h"
#include "jesdinit/jesdinit.h"
#include "raptor2_eeprom.h"
#include "raptor2_avs.h"
#include "raptor2_adi_dpll.h"
//#define PCIE_ENV_NOR

#define ENABLE_PADMUX_SPI(addr) writel(0x00060006, addr)

#define SPI_COMMAND 0xC0
#define SPI_REPLY 0xD0
#define SPI_CMD_BUFF_SIZE 512
#define DPLL_TIMEOUT	125

#define READ_REPLY	0x00
#define SIO_TEST	0x01
#define SIO_INFO	0x02
#define HOST_LOAD	0x05
#define BOOT		0x07
#define RESTART		0xF0

#define CLOCK_NODE_PATH		"/virt_46mhz"

#define RFGPIO_BASE	0x68494000
#define RFGPIO_OUT	0xC8
#define RFGPIO_IN	0xD8

#define CLOCK_SRC_PROP		"soc-clk-src" /** SoC clock source property name */

#define MMC_CLOCK_46MHZ		46000000
#define MMC_CLOCK_23MHZ		23000000

/* NOR FLASH partition details */
#define FAILSAFE_PARTITION_ID   1
#define PARTITION_A_ID          2
#define PARTITION_B_ID          3
#define R2_ACTIVE_PARTITION_OFFSET	0x800000200

#define FIRMWARE_FS_OFFSET      "0x80000"
#define FIRMWARE_A_OFFSET	"0x1000000"
#define FIRMWARE_B_OFFSET	"0x1800000"

/* ENV VARIABLES */
#define FIT_BOOT		"bootcmd_fit"
#define ROOTFS_PARTITION_A	"rootfs_a"
#define ROOTFS_PARTITION_B	"rootfs_b"
#define TITAN_FIT_CONFIG	"b0_titan"
#define HAWK_FIT_CONFIG		"b0_hawk"
#define PEGASUS_FIT_CONFIG	"b0_pegasus"

/* Use the board type, when patching a dtb name for tftp boot */
#define BOARD_NAME_TITAN	"titan"
#define BOARD_NAME_HAWK		"hawk"
#define BOARD_NAME_PEGASUS	"pegasus"

#define PCI_PORT_TYPE_SIZE 3
#define LINK_MASK 0x03
#define LINK3_OFF 0		// Bit mask for Link4 modes
#define LINK4_OFF 2		// Bit mask for Link4 modes

#define DDR_SIZE_8GB	8
#define DDR_SIZE_16GB	16
#define DDR_SIZE_32GB	32

#define SHM_BSSRT_START_OFFS	0x0
#define SHM_BSSRT_SIZE		0x1000000	/* First 16MB for BSS RT */

enum pci_mode {
	NA = 0,
	LB,
	EP,
	RC
};

#if 0
#define RAPTOR2_DBG(fmt, ...) printf("[%s][%s][%d] " fmt, __FILE__, __func__, \
							  __LINE__, ##__VA_ARGS__);
#else
#define RAPTOR2_DBG(fmt, ...)
#endif

enum e_board_version {
	INVALID_BOARD = 0,
	BOARD_V1,
	BOARD_V2
};

/**
 * @brief SoC clock source
 * @note Should there be any changes to this enum, make sure
 * 		 to update Linux driver's enum as well!
 */
enum soc_clk_src {
	SoC_CLK_DEFAULT,
	SoC_CLK_VCTCXO = 0,
	SoC_CLK_DCTCXO,
	SoC_CLK_DPLL,
	SoC_CLK_LOCAL_CLK,
	SoC_CLK_SRC_NUM,
};

static uchar            din_reply[64];
static uchar            dout_args[100] = {0x01, 0x02, 0x03, 0x04, 0x05};

static struct mm_region raptor2_mem_map[] = {
	{
		/* SHM for BSS RT */
		.virt = CONFIG_SYS_SDRAM_BASE + SHM_BSSRT_START_OFFS,
		.phys = CONFIG_SYS_SDRAM_BASE + SHM_BSSRT_START_OFFS,
		.size = SHM_BSSRT_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL_NC) | PTE_BLOCK_INNER_SHARE,
	}, {
		/* RAM */
		.virt = CONFIG_SYS_SDRAM_BASE + SHM_BSSRT_SIZE,
		.phys = CONFIG_SYS_SDRAM_BASE + SHM_BSSRT_SIZE,
		.size = SYS_SDRAM_SIZE_32GB - SHM_BSSRT_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) | PTE_BLOCK_INNER_SHARE,
	}, {
		/* Peripherials */
		.virt = 0x000000000000UL,
		.phys = 0x000000000000UL,
		.size = 0x000400000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			PTE_BLOCK_NON_SHARE,
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = raptor2_mem_map;
#define NUM_CONF_SPEED	5
u16 config_eth_speed[NUM_CONF_SPEED] = {0, 0, 0, 0, 0};
u16 config_eth_link = 2;
u32 emmc_clock_f;
u32 env_board_type = 0;
extern int raptor_config_txu_pll(void);

#if (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_A0_EVB) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_TITAN) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS))
typedef enum {
	RESET_N    =   0,	/* RG_GPIO PIN 0*/
	TX_OFF,			/* RG_GPIO PIN 1*/
	RX_OFF,			/* RG_GPIO PIN 2*/
	FEM2_DPD_SW_EN,		/* RG_GPIO PIN 3*/
	FEM4_TX_SW_EN,		/* RG_GPIO PIN 4*/
	FEM4_LNA_EN,		/* RG_GPIO PIN 5*/
	GPIO_PIN_6,		/* RG_GPIO PIN 6*/
	GPIO_PIN_7,		/* RG_GPIO PIN 7*/
	FEM3_DPD_SW_EN,		/* RG_GPIO PIN 8*/
	FEM3_LNA_EN,		/* RG_GPIO PIN 9*/
	GPIO_PIN_10,		/* RG_GPIO PIN 10*/
	FEM4_PAEN,		/* RG_GPIO PIN 11*/
	GPIO_PIN_12,		/* RG_GPIO PIN 12*/
	FEM3_PAEN,		/* RG_GPIO PIN 13*/
	FEM1_LNA_EN,		/* RG_GPIO PIN 14*/
	ADR0,			/* RG_GPIO PIN 15*/
	GPIO_PIN_16,		/* RG_GPIO PIN 16*/
	GPIO_PIN_17,		/* RG_GPIO PIN 17*/
	GPIO_PIN_18,		/* RG_GPIO PIN 18*/
	GPIO_PIN_19,		/* RG_GPIO PIN 19*/
	GPIO_PIN_20,		/* RG_GPIO PIN 20*/
	LDO_EN,			/* RG_GPIO PIN 21*/
	FEM1_DPD_SW_EN,		/* RG_GPIO PIN 22*/
	FEM1_PAEN,		/* RG_GPIO PIN 23*/
	FEM2_TX_SW_EN,		/* RG_GPIO PIN 24*/
	FEM2_LNA_EN,		/* RG_GPIO PIN 25*/
	FEM2_PAEN,		/* RG_GPIO PIN 26*/
	FEM1_TX_SW_EN,		/* RG_GPIO PIN 27*/
	GPIO_PIN_28,		/* RG_GPIO PIN 28*/
	ADR1,			/* RG_GPIO PIN 29*/
	FEM4_DPD_SW_EN,		/* RG_GPIO PIN 30*/
	FEM3_TX_SW_EN		/* RG_GPIO PIN 31*/
} rfgpio_e;

#elif (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_A0_PEGASUS))
typedef enum {
	RFIC1_RST_N = 0,
	RFIC1_TX1_PA_EN,
	RFIC1_TX1_DA_DIS,
	RFIC1_TRX1,
	RFIC1_ENB = 5,
	RFIC1_IDENT,
	RFIC2_RST_N = 8,
	RFIC2_TX3_PA_EN,
	RFIC2_TRX3,
	RFIC2_TRX4,
	RFIC1_RX2_DPD,
	RFIC2_RX4_DPD,
	RFIC1_TRX2,
	RFIC2_ENB,
	RFIC1_TX2_DA_DIS = 17,
	RFIC1_TX2_PA_EN = 19,
	RFIC2_TX4_PA_EN,
	RFIC2_IDENT,
	RFIC2_TX3_DA_DIS,
	RFIC2_TX4_DA_DIS,
	RFIC1_RX1_DPD = 28,
	RFIC2_RX3_DPD,
} rfgpio_e;
#endif

const char *ether_path[] = {"/ethernet@3C040000",
				"/ethernet@3C080000",
				"/ethernet@3C0C0000",
				"/ethernet@3C100000",
				"/ethernet@3C000000"};


static void dpll_spi_command(struct spi_slave *slave, char command, uchar *args, int cmd_size)
{
	uchar *data_out = NULL;
	int i, ret = 0;

	data_out = calloc(cmd_size, 1);
	if (!data_out) {
		printf("Calloc Failed\n");
		return;
	}

	data_out[0] = SPI_COMMAND;
	data_out[1] = command;

	for (i = 2; i < cmd_size; i++)
		data_out[i] = args[i - 2];

	ret = spi_xfer(slave, (cmd_size) * 8, data_out, NULL, SPI_XFER_ONCE);

	if (ret)
		printf("DPLL: spi_xfer fail. Return code %d\n", ret);

	free(data_out);
}

static void dpll_spi_reply(struct spi_slave *slave, uchar *buffer, int count, char command)
{
	int ret = 0;

	buffer[0] = SPI_REPLY;

	ret = spi_write_then_read(slave, buffer, 1, NULL, buffer, count);

	if (ret) {
		printf("DPLL: Error during SPI reply: Return code = %d\n", ret);
		return;
	}

	// HW BUG: READ_REPLY always returning API ERROR. Thus suppressing check
	// for READ_REPLY
	if (((buffer[0] & 0x70) != 0) && command != READ_REPLY)
		printf("DPLL: Error Status %02X for command %02X\n", buffer[0], command);
}

static void dpll_send_cmd(struct spi_slave *slave, char command, uchar *args, uchar *read_buf
	, int cmd_size, int reply_len)
{
	dpll_spi_command(slave, command, args, cmd_size);
	mdelay(15);
	dpll_spi_reply(slave, read_buf, reply_len, command);
}

static int wait_cts(struct spi_slave *slave)
{
	uchar reply[3] = {0};
	int timeout = DPLL_TIMEOUT;

	do {
		dpll_send_cmd(slave, READ_REPLY, NULL, reply, 2, 3);
		mdelay(1);
	} while (((reply[0] & 0x80) == 0) && --timeout);

	if (!timeout)
		printf("DPLL: CTS Timeout\n");

	return timeout;
}

static void dpll_print_reply(uchar *buf, int size)
{
	int i;

	for (i = 0; i < size; i++)
		printf("%02X", buf[i]);
	printf("\n");
}

static void dpll_app_load(struct spi_slave *slave, int data_to_load, uchar *data_ptr, int block_size)
{
	int write_len = 0;

	while (data_to_load) {
		data_ptr += write_len;
		if (data_to_load > block_size) {
			write_len = block_size;
			data_to_load -= block_size;
		} else {
			write_len = data_to_load;
			data_to_load = 0;
		}

		dpll_send_cmd(slave, HOST_LOAD, data_ptr, din_reply, write_len + 2, 1);
		if (din_reply[0] != 0x80)
			printf("DPLL: HOST LOAD failed with status %02X\n", din_reply[0]);
	}
	wait_cts(slave);
}

static void dpll_load_config(void)
{
#ifdef CONFIG_DM_SPI
	struct spi_slave *slave;
	int ret = 0;
	int cmd_buf_size, reply_buf_size;
	int load_size = SPI_CMD_BUFF_SIZE - 2;

	struct udevice *dev;

	ret = spi_get_bus_and_cs(1/*bus*/, 0/*cs*/, 15000000/*freq*/, 0/*mode*/, "spi_generic_drv",
				 "atcspi200_spi", &dev, &slave);
	if (ret) {
		printf("DPLL: Unable to get bus & cs. Return code %d\n", ret);
		return;
	}

	ret = spi_claim_bus(slave);
	if (ret) {
		printf("DPLL: Unable to claim bus. Return code %d\n", ret);
		goto done;
	}

	/* DPLL Sequence Start*/

	// Test  Command
	dpll_send_cmd(slave, SIO_TEST, dout_args, din_reply, 7, 7);

	if (memcmp(dout_args, &din_reply[2], 5)) {
		printf("DPLL: Test Command failed:\n");
		dpll_print_reply(din_reply, 7);
	}

	// Reading Device Info
	dpll_send_cmd(slave, SIO_INFO, NULL, din_reply, 2, 5);
	cmd_buf_size = (din_reply[2] << 8) | din_reply[1];
	reply_buf_size = (din_reply[4] << 8) | din_reply[3];

	if (!wait_cts(slave))
		goto done;

	// Restart DPLL
	dpll_send_cmd(slave, RESTART, &dout_args[0], din_reply, 3, 1);
	if (din_reply[0] != 0x80)
		printf("DPLL: Restart failed with status %02X\n", din_reply[0]);

	if (!wait_cts(slave))
		goto done;

	/// Loading Configuration to the device
	if ((load_size + 2) > cmd_buf_size)
		load_size = cmd_buf_size - 2;

	dpll_app_load(slave, dpll_app_prod_fw_bin_len, dpll_app_prod_fw_bin, load_size);
	dpll_app_load(slave, dpll_app_user_config_bin_len, dpll_app_user_config_bin, load_size);

	if (!wait_cts(slave))
		goto done;

	// BOOTING DPLL with updated configuration
	dpll_send_cmd(slave, BOOT, NULL, din_reply, 2, 1);

	if (din_reply[0] != 0x80)
		printf("DPLL: BOOT failed with status: %02X\n", din_reply[0]);


done:
	spi_release_bus(slave);
	spi_free_slave(slave);
#endif
}

static void gpio_rmw(phys_addr_t addr, u32 mask, u32 val)
{
	u32 rdat = readl(addr);

	rdat = rdat & (~mask);
	rdat = rdat | val;
	writel(rdat, addr);
}

static void rfgpio_update(u32 pin, u32 in)
{
	u32 val = readl(RFGPIO_BASE + RFGPIO_IN);

	val  = val & (~(1 << pin));
	val  = val | (in << pin);
	writel(val, RFGPIO_BASE + RFGPIO_IN);
	val = 1 << pin;
	gpio_rmw(RFGPIO_BASE + RFGPIO_OUT, val, val);
}

#if (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_A0_EVB) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_TITAN) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_HAWK) || \
	IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_B0_PEGASUS))
static void rfic_reset(void)
{
	rfgpio_update(RESET_N, 1);
	rfgpio_update(ADR1, 1);
	rfgpio_update(FEM1_TX_SW_EN, 0);
	rfgpio_update(FEM2_TX_SW_EN, 0);
	rfgpio_update(FEM1_LNA_EN, 0);
	rfgpio_update(FEM2_LNA_EN, 0);
	rfgpio_update(FEM3_LNA_EN, 0);
	rfgpio_update(FEM4_LNA_EN, 0);
	rfgpio_update(FEM1_DPD_SW_EN, 0);
	rfgpio_update(FEM2_DPD_SW_EN, 0);
	rfgpio_update(FEM3_DPD_SW_EN, 0);
	rfgpio_update(FEM4_DPD_SW_EN, 0);
	rfgpio_update(FEM1_PAEN, 0);
	rfgpio_update(FEM2_PAEN, 0);
	rfgpio_update(FEM3_PAEN, 0);
	rfgpio_update(FEM4_PAEN, 0);
	rfgpio_update(RESET_N, 0);
	rfgpio_update(ADR1, 0);
	rfgpio_update(ADR1, 1);
	rfgpio_update(RESET_N, 1);
}
#elif (IS_ENABLED(CONFIG_TARGET_EQ_RAPTOR2_A0_PEGASUS))
static void rfic_reset(void)
{
	rfgpio_update(RFIC1_RST_N, 1);
	rfgpio_update(RFIC1_ENB, 1);
	rfgpio_update(RFIC1_IDENT, 0);
	rfgpio_update(RFIC1_TRX1, 1);
	rfgpio_update(RFIC1_TRX2, 1);
	rfgpio_update(RFIC1_TX1_PA_EN, 0);
	rfgpio_update(RFIC1_TX2_PA_EN, 0);
	rfgpio_update(RFIC1_TX1_DA_DIS, 1);
	rfgpio_update(RFIC1_TX2_DA_DIS, 1);
	rfgpio_update(RFIC1_RX1_DPD, 1);
	rfgpio_update(RFIC1_RX2_DPD, 1);
	rfgpio_update(RFIC1_RST_N, 0);
	rfgpio_update(RFIC1_ENB, 0);
	rfgpio_update(RFIC1_ENB, 1);
	rfgpio_update(RFIC1_RST_N, 1);
  }
#endif

int adi_dpll_config(void)
{
	struct spi_slave *slave;
	struct udevice *dev;
	int ret = 0;
	u32 board;
	u8 rdat;

	/* 1. Lift ADI DPLL reset */
	rfgpio_update(GPIO_PIN_28, 1);

	/* 2. SPI Init */
	board = get_board_name_value();
	if (board == BOARD_TITAN) {
		ret = adi_dpll_spi_init(&slave, &dev, 3);
	} else if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		ret = adi_dpll_spi_init(&slave, &dev, 4);
	} else {
		ret = -1;
		printf("ADI:DPLL: Error! wrong board detected\r\n");
		goto error;
	}

	if (board == BOARD_TITAN) {
		/* Drive RFGPIO31 drive to 1 */
		rfgpio_update(FEM3_TX_SW_EN, 1); /* RG_GPIO PIN 31*/
	}
	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		/* drive SOC_GPIO35 to 1 */
		cpu_rmw_w(0x6E440064, 0xFFFF0000, 0x1E500000);
	}

	/* 3. Check DPLL status post reset */
	adi_dpll_spi_read(slave, (u8[]){0x05 | 0x80, 0x08, 0x00}, &rdat);
	printf("ADI:DPLL: status pre reset = 0x%X\n", rdat);
	adi_dpll_spi_write(slave, (u8[]){0x00, 0x00, 0xBD});
	mdelay(100);
	adi_dpll_spi_read(slave, (u8[]){0x05 | 0x80, 0x08, 0x00}, &rdat);
	printf("ADI:DPLL: status post reset = 0x%X\n", rdat);
	adi_dpll_spi_read(slave, (u8[]){0x00 | 0x80, 0x0C, 0x00}, &rdat);
	printf("ADI:DPLL: ID = 0x%X\n", rdat);

	adi_dpll_program(slave);
	mdelay(1000);

	adi_dpll_spi_read(slave, (u8[]){0x05 | 0x80, 0x08, 0x00}, &rdat);
	printf("ADI:DPLL: status 0x%x\r\n", rdat);
	if (rdat != 0xe7) {
		ret = -1;
		printf("ERROR: ADI DPLL not locked\r\n");
	} else {
		ret = 0;
		printf("ADI:DPLL: locked\r\n");
	}

	if (board == BOARD_TITAN) {
		/* Drive RFGPIO31 drive to 0 */
		rfgpio_update(FEM3_TX_SW_EN, 0); /* RG_GPIO PIN 31*/
	}
	if ((board == BOARD_HAWK) || (board == BOARD_HAWKV2) || (board == BOARD_HAWKV3) || (board == BOARD_HAWKV4) || (board == BOARD_HAWKV5)) {
		/* drive SOC_GPIO35 to 0 */
		cpu_rmw_w(0x6E440064, 0xFFFF0000, 0x1E400000);
	}

	adi_dpll_spi_deinit(slave, dev);
error:
	return ret;
}

static int detect_board(void)
{
	int ret = 0;
	static int board_version;
#ifdef CONFIG_DM_I2C
	struct udevice *bus, *udev;

	if (board_version == 0) {
		ret = uclass_get_device(UCLASS_I2C, 0, &bus);
		if (ret)
			return INVALID_BOARD;

		i2c_deblock(bus);
		ret = i2c_get_chip_for_busnum(0, 0x50, 2, &udev);
		if (ret)
			board_version = BOARD_V1;
		else
			board_version = BOARD_V2;
	}
#endif
	return board_version;
}

static void set_pmic_vdd(char *vdd_ptr)
{
#ifdef CONFIG_CMD_SET_VDD
	int vdd_val;

	vdd_val = parse_vdd_value(vdd_ptr);
	if (vdd_val == -1) {
		printf("Invalid vdd_0v8 env: %s\n"
				"Max 2 digits after decimal are allowed\n"
				"The value should be in between 0.8V to 0.9V\n", vdd_ptr);
	} else {
		if (set_buck2_vdd(vdd_val - 20))
			printf("I2C Error: Set PMIC VDD Failed\n");
	}
#endif
}

#ifdef CONFIG_SPL
u32 spl_boot_device(void)
{
	return BOOT_DEVICE_RAM;
}

void board_boot_order(u32 *spl_boot_list)
{
	u8 i;
	u32 boot_devices[] = {
#ifdef CONFIG_SPL_RAM_SUPPORT
		BOOT_DEVICE_RAM,
#endif
#ifdef CONFIG_SPL_MMC_SUPPORT
		BOOT_DEVICE_MMC1,
#endif
	};

	for (i = 0; i < ARRAY_SIZE(boot_devices); i++)
		spl_boot_list[i] = boot_devices[i];
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* boot using first FIT config */
	return 0;
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	return 0;
}
#endif

int detect_emmc_boot_partition(int active_partition)
{
	int dev = 0;
	int part_num;
	struct mmc *mmc;
	struct blk_desc *dev_desc;
	struct disk_partition part_info;
	int ret;

	/* Initialize the MMC devices */
	ret = mmc_initialize(NULL);

	/* Find the eMMC device using eMMC device number */
	mmc = find_mmc_device(dev);
	if (!mmc) {
		printf("Failed to find eMMC device %d\n", dev);
		return -1;
	}

	/* Get the block descriptor for the device */
	dev_desc = blk_get_dev("mmc", dev);
	if (!dev_desc) {
		printf("Failed to get block device descriptor for eMMC %d\n", dev);
		return -1;
	}

	/* Iterate through the partitions */
	for (part_num = 1; part_num <= MAX_SEARCH_PARTITIONS; part_num++) {
		if (part_get_info(dev_desc, part_num, &part_info) != 0) {
			printf("Failed to get partition %d information\n", part_num);
			return -1;
		}

		/* Check if the partition label matches "boot_a" or "boot_b" */
		if (active_partition == PARTITION_A_ID) {
			if (strcmp((const char *)part_info.name, "boot_a") == 0) {
				printf("Found 'boot_a' partition at eMMC %d, \
						partition %d\n", dev, part_num);
				return part_num;
			}
		} else if (active_partition == PARTITION_B_ID) {
			if (strcmp((const char *)part_info.name, "boot_b") == 0) {
				printf("Found 'boot_b' partition at eMMC %d, \
						partition %d\n", dev, part_num);
				return part_num;
			}
		}
	}

	printf("No 'boot' partition found at eMMC %d\n", dev);
	return -1;
}

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	enum e_board_version brd_ver;
	char *dpll_flag = NULL;
	char *vdd_ptr = NULL;
	uint32_t sig;
	u32 *shared_data_addr = (u32 *)SHARED_MEM_ADDR;
	u32 avs_idx;
	int active_partition;
	int partition;

	const char *chip = "EQ2540";
	const char *vendor = "EdgeQ";
	const char *board_name = "EVB-V1";

	brd_ver = detect_board();

	if (brd_ver == INVALID_BOARD) {
		printf("Board: Unable to detect board version.\n");
	} else {

		if (brd_ver == BOARD_V2) {

			board_name = "EVB-V2";
			dpll_flag = env_get("dpll_enable");
			if (dpll_flag) {
				if (strcmp(dpll_flag, "1") == 0) {
					ENABLE_PADMUX_SPI(0x7044007C); //GPIO_46 & GPIO 47
					ENABLE_PADMUX_SPI(0x70440080); //GPIO_48 & GPIO 49

					dpll_load_config();
				}
			}

			vdd_ptr = env_get("vdd_0v8");
			if (vdd_ptr)
				set_pmic_vdd(vdd_ptr);

			sig = get_eeprom_signature();
			if (sig == EEPROM_SIGNATURE) {
				get_board_configs(&chip, &vendor, &board_name);
				set_mac_interfaces();
			} else {
				printf("Warning: EEPROM signature not matched\n");
			}
		}
		printf("Chip: %s, Board: %s-%s\n", chip, vendor, board_name);
	}

	active_partition = (*(uint64_t *)(R2_ACTIVE_PARTITION_OFFSET));

	if (active_partition != FAILSAFE_PARTITION_ID) {
		partition = detect_emmc_boot_partition(active_partition);
		if (partition < 0)
			printf("Failed to get boot partition number !\n");
	}

	if (active_partition == PARTITION_A_ID) {
		env_set("boot_cmd", FIT_BOOT);
		env_set("linux_partition", simple_itoa(partition));
		env_set("rfs_partition", ROOTFS_PARTITION_A);
		env_set("firmware_addr_f", FIRMWARE_A_OFFSET);
	} else if (active_partition == PARTITION_B_ID) {
		env_set("boot_cmd", FIT_BOOT);
		env_set("linux_partition", simple_itoa(partition));
		env_set("rfs_partition", ROOTFS_PARTITION_B);
		env_set("firmware_addr_f", FIRMWARE_B_OFFSET);
	} else if (active_partition == FAILSAFE_PARTITION_ID) {
		env_set("firmware_addr_f", FIRMWARE_FS_OFFSET);
	}

	if (!(strcmp(board_name, "Titan"))) {
		env_set("b0_board", TITAN_FIT_CONFIG);
		env_set("board_name", BOARD_NAME_TITAN);
	} else if (!(strcmp(board_name, "Hawk"))) {
		env_set("b0_board", HAWK_FIT_CONFIG);
		env_set("board_name", BOARD_NAME_HAWK);
	} else if (!(strcmp(board_name, "Pegasus"))) {
		env_set("b0_board", PEGASUS_FIT_CONFIG);
		env_set("board_name", BOARD_NAME_PEGASUS);
	}

	rfic_reset();

	avs_idx = *shared_data_addr;
	//cfg_avs_pmic(avs_idx);

	return 0;
}
#endif

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	char *env_ethlink;
	char *env_ethspeed;
	char speed_env_name[16];
	int i;
	int active_partition;
	u8 rf1_card_type;
	u8 rf2_card_type;

	env_ethlink = env_get("ethlink");
	if (env_ethlink != NULL) {
		if (strcmp(env_ethlink, "1") == 0)
			config_eth_link = 1;
		else if (strcmp(env_ethlink, "3") == 0)
			config_eth_link = 3;
		else
			config_eth_link = 2;
	}

	for (i = 0; i < NUM_CONF_SPEED; i++) {
		if (i == 0)
			sprintf(speed_env_name, "ethspeed");
		else
			sprintf(speed_env_name, "eth%dspeed", i);
		env_ethspeed = env_get(speed_env_name);
		if (env_ethspeed != NULL) {
			if (strcmp(env_ethspeed, "50g") == 0 ||
			    strcmp(env_ethspeed, "50G") == 0)
				config_eth_speed[i] = SPEED_50000;
			else if (strcmp(env_ethspeed, "25g") == 0 ||
			         strcmp(env_ethspeed, "25G") == 0)
				config_eth_speed[i] = SPEED_25000;
			else if (strcmp(env_ethspeed, "10g") == 0 ||
			         strcmp(env_ethspeed, "10G") == 0)
				config_eth_speed[i] = SPEED_10000;
			else if (strcmp(env_ethspeed, "5g") == 0 ||
			         strcmp(env_ethspeed, "5G") == 0)
				config_eth_speed[i] = SPEED_5000;
			else if (strcmp(env_ethspeed, "2g5") == 0 ||
			         strcmp(env_ethspeed, "2G5") == 0)
				config_eth_speed[i] = SPEED_2500;
			else if (strcmp(env_ethspeed, "1g") == 0 ||
			         strcmp(env_ethspeed, "1G") == 0)
				config_eth_speed[i] = SPEED_1000;
		}
	}

	env_board_type = get_board_name_value();

	active_partition = (*(uint64_t *)(R2_ACTIVE_PARTITION_OFFSET));

	if (active_partition != FAILSAFE_PARTITION_ID) {
		if ((get_link_status(STS_LINK3) != LINK_JESD) &&
			(get_link_status(STS_LINK4) != LINK_JESD)) {
			debug("u-boot:JESD: Link3 and Link4 are not configured for JESD");
			goto done;
		}

		rf1_card_type = get_rf1_card_type();
		rf2_card_type = get_rf2_card_type();

		/**
		 * TODO: Right now check only for ADI card based on SPL changes
		 */
		if (rf1_card_type == RF_CARD_ADI || rf2_card_type == RF_CARD_ADI) {
			/* Store eeprom information into intermediate structure */
			if (copy_rfic_eeprom_data(&rf_info)) {
				debug("u-boot:JESD: unable to copy the eeprom data\r\n");
				goto done;
			}

			if (adi_dpll_config()) {
				printf("u-boot:JESD: skip training for ADI card!\r\n");
				goto done;
			} else {
				jesd_phy_pwrup();
			}
		} else {
			debug("u-boot:JESD: no JESD card detected!\r\n");
		}
	}
done:
	return 0;
}

#endif

void fdt_fixup_board_sus(void *blob)
{
	uint8_t *product_config;
	uint8_t otrx_ldpc_cfg, otrx_bcc_cfg;
	uint8_t txu_cluster01_cfg0, txu_cluster01_cfg1, txu_cluster01_cfg;
	uint8_t txu_cluster23_cfg0, txu_cluster23_cfg1, txu_cluster23_cfg;

	product_config = get_product_config();

	//Extract TXU configs from HW Resource map
	txu_cluster01_cfg0 = product_config[0];
	txu_cluster23_cfg0 = product_config[1];
	txu_cluster01_cfg1 = product_config[2];
	txu_cluster23_cfg1 = product_config[3];

	//ORing SU_AU & LMEM bitfield from HWResource map
	txu_cluster01_cfg = (txu_cluster01_cfg0|txu_cluster01_cfg1);
	txu_cluster23_cfg = (txu_cluster23_cfg0|txu_cluster23_cfg1);

	//Extract OTRX config from HW Resource map
	otrx_ldpc_cfg = product_config[5];
	otrx_bcc_cfg = product_config[6];

	if (txu_cluster01_cfg & 0xF0) {
		fdt_status_okay_by_alias(blob, "txu4");
		fdt_status_okay_by_alias(blob, "txu5");
		fdt_status_okay_by_alias(blob, "txu6");
		fdt_status_okay_by_alias(blob, "txu7");
		fdt_status_okay_by_alias(blob, "iring1");
		printf("Enabled txu cluster 1\n");
	}

	if (txu_cluster23_cfg & 0x0F) {
		fdt_status_okay_by_alias(blob, "txu8");
		fdt_status_okay_by_alias(blob, "txu9");
		fdt_status_okay_by_alias(blob, "txu10");
		fdt_status_okay_by_alias(blob, "txu11");
		fdt_status_okay_by_alias(blob, "iring2");
		printf("Enabled txu cluster 2\n");
	}

	if ((otrx_ldpc_cfg & 0x30) || (otrx_bcc_cfg & 0x03)) {
		fdt_status_okay_by_alias(blob, "orxd_ldpc2");
		fdt_status_okay_by_alias(blob, "orxd_bcc0");
		fdt_status_okay_by_alias(blob, "orxd_bcc1");
		printf("Enabled orxd_ldpc2\n");
		printf("Enabled orxd_bcc 0 and 1\n");
	}
}

#ifdef CONFIG_OF_BOARD_SETUP
#define ECPRI_ARRAY_LEN		5
#define ECPRI_REG_OFFSET	32
#define ECPRI_LMEM_ADDR		0x14A00000
#define ECPRI_LMEM_SIZE		0x40000
void fdt_fixup_board_enet(void *blob)
{
	char *env_ecpri_en;
	u8 ecpri_en[ECPRI_ARRAY_LEN] = {0, 0, 0, 0, 0};
	int ecpri_en_len;
	int offs;
	int i;

	env_ecpri_en = env_get("ecpri_en");
	if (env_ecpri_en != NULL) {
		ecpri_en_len = strlen(env_ecpri_en);
		if (ecpri_en_len > ECPRI_ARRAY_LEN)
			ecpri_en_len = ECPRI_ARRAY_LEN;
		for (i = 0; i < ecpri_en_len; i++) {
			/* Allow only one port to enable eCPRI */
			if (*(env_ecpri_en+i) ==  '1') {
				ecpri_en[ecpri_en_len-1-i] = 1;
				break;
			}
		}
	}

	switch (config_eth_link) {
	case 1:
		fdt_status_disabled_by_alias(blob, "ethernet0"); // Link2 Lane0
		fdt_status_disabled_by_alias(blob, "ethernet1"); // Link2 Lane1
		fdt_status_disabled_by_alias(blob, "ethernet2"); // Link2 Lane2
		fdt_status_disabled_by_alias(blob, "ethernet3"); // Link2 Lane3
		i = NUM_CONF_SPEED - 1;
		if (config_eth_speed[i] >= SPEED_10000 &&
		    config_eth_speed[i] <= SPEED_50000) {
			do_fixup_by_path_u32(blob, ether_path[i],
				"snps,ps-speed", config_eth_speed[i], 0);
			do_fixup_by_path_u32(blob, ether_path[i],
				"max-speed", config_eth_speed[i], 0);
		}
		/* Enable eCPRI for link1 */
		if (ecpri_en[i]) {
			fdt_find_and_setprop(blob, ether_path[i],
					"edgeq,ecpri-offload", NULL, 0, 1);
			/* Disable pmd */
			do_fixup_by_path_u32(blob, ether_path[i],
					"edgeq,pmd-num-chans", 0, 0);
		}
		break;
	case 2:
		fdt_status_disabled_by_alias(blob, "ethernet4"); // Link1
		if (config_eth_speed[0] >= SPEED_1000 &&
		    config_eth_speed[0] <= SPEED_25000) {
			do_fixup_by_path_u32(blob, ether_path[0],
				"snps,ps-speed", config_eth_speed[0], 0);
			do_fixup_by_path_u32(blob, ether_path[0],
				"max-speed", config_eth_speed[0], 0);
		}
		for (i = 1; i < NUM_CONF_SPEED-1; i++) {
			if (config_eth_speed[i] >= SPEED_10000 &&
			    config_eth_speed[i] <= SPEED_25000) {
				do_fixup_by_path_u32(blob, ether_path[i],
					"snps,ps-speed", config_eth_speed[i], 0);
				do_fixup_by_path_u32(blob, ether_path[i],
					"max-speed", config_eth_speed[i], 0);
			}
		}
		/* Hawk V1 does not support retimer on lane1 */
		if (env_board_type == BOARD_HAWK)
			fdt_status_disabled_by_alias(blob, "ethernet1");
		/* Enable eCPRI for link2 ports */
		for (i = 0; i < ECPRI_ARRAY_LEN-1; i++) {
			const void *prop;
			int len;
			u64 *loc;

			if (ecpri_en[i]) {
				fdt_find_and_setprop(blob, ether_path[i],
					 "edgeq,ecpri-offload", NULL, 0, 1);

				/* One port uses all eCPRI LMEM */
				offs = fdt_path_offset(blob, ether_path[i]);
				prop = fdt_getprop(blob, offs, "reg", &len);
				loc = (u64 *)(prop + ECPRI_REG_OFFSET);
				*loc = cpu_to_fdt64(ECPRI_LMEM_ADDR);
				loc = (u64 *)(prop + ECPRI_REG_OFFSET + 8);
				*loc = cpu_to_fdt64(ECPRI_LMEM_SIZE);

				/* Disable pmd */
				do_fixup_by_path_u32(blob, ether_path[i],
						"edgeq,pmd-num-chans", 0, 0);
			}
		}
		break;
	case 3:
		if (config_eth_speed[0] >= SPEED_1000 &&
		    config_eth_speed[0] <= SPEED_10000) {
			do_fixup_by_path_u32(blob, ether_path[0],
				"snps,ps-speed", config_eth_speed[0], 0);
			do_fixup_by_path_u32(blob, ether_path[0],
				"max-speed", config_eth_speed[0], 0);
		}
		else if (config_eth_speed[0] == SPEED_25000) {
			do_fixup_by_path_u32(blob, ether_path[0],
				"snps,ps-speed", SPEED_10000, 0);
			do_fixup_by_path_u32(blob, ether_path[0],
				"max-speed", SPEED_10000, 0);
		}
		for (i = 1; i < NUM_CONF_SPEED-1; i++) {
			if (config_eth_speed[i] >= SPEED_10000 &&
			    config_eth_speed[i] <= SPEED_25000) {
				do_fixup_by_path_u32(blob, ether_path[i],
					"snps,ps-speed", config_eth_speed[i], 0);
				do_fixup_by_path_u32(blob, ether_path[i],
					"max-speed", config_eth_speed[i], 0);
			}
		}
		if (config_eth_speed[i] >= SPEED_10000 &&
		    config_eth_speed[i] <= SPEED_50000) {
			do_fixup_by_path_u32(blob, ether_path[i],
				"snps,ps-speed", config_eth_speed[i], 0);
			do_fixup_by_path_u32(blob, ether_path[i],
				"max-speed", config_eth_speed[i], 0);
		}
		/* Enable eCPRI for link2 ports */
		for (i = 0; i < ECPRI_ARRAY_LEN; i++) {
			const void *prop;
			int len;
			u64 *loc;

			if (ecpri_en[i]) {
				fdt_find_and_setprop(blob, ether_path[i],
					 "edgeq,ecpri-offload", NULL, 0, 1);

				/* One port uses all eCPRI LMEM */
				offs = fdt_path_offset(blob, ether_path[i]);
				prop = fdt_getprop(blob, offs, "reg", &len);
				loc = (u64 *)(prop + ECPRI_REG_OFFSET);
				*loc = cpu_to_fdt64(ECPRI_LMEM_ADDR);
				loc = (u64 *)(prop + ECPRI_REG_OFFSET + 8);
				*loc = cpu_to_fdt64(ECPRI_LMEM_SIZE);

				/* Disable pmd */
				do_fixup_by_path_u32(blob, ether_path[i],
						"edgeq,pmd-num-chans", 0, 0);
			}
		}
		break;
	default:
		break;
	}
}

void fdt_fixup_board_emmc(void *blob)
{
	char *emmc_freq;

	emmc_freq = env_get("emmc_freq");
	if (emmc_freq != NULL) {
		if (strcmp(emmc_freq, "46") == 0)
			emmc_clock_f = MMC_CLOCK_46MHZ;
		else if (strcmp(emmc_freq, "23") == 0)
			emmc_clock_f = MMC_CLOCK_23MHZ;
	}

	if (emmc_clock_f) {
		do_fixup_by_path_u32(blob, CLOCK_NODE_PATH, "clock-frequency", emmc_clock_f, 1);
		printf("Emmc clock frequency set to %d Hz\n", emmc_clock_f);
	}
}

void fdt_fixup_board_ddr(void *blob)
{
	uint64_t ddr_size = CONFIG_SYS_SDRAM_SIZE;
	uint32_t ddr_config;
	int offset;

	ddr_config = get_ddr_config();

	if(((ddr_config & 0x3F) == 0x0) || ((ddr_config & 0x3F) == DDR_SIZE_8GB))
		ddr_size = SYS_SDRAM_SIZE_8GB;
	else if ((ddr_config & 0x3F) == DDR_SIZE_16GB)
		ddr_size = SYS_SDRAM_SIZE_16GB;
	else if ((ddr_config & 0x3F) == DDR_SIZE_32GB)
		ddr_size = SYS_SDRAM_SIZE_32GB;

	/* Find the memory node in the DTB */
	offset = fdt_path_offset(blob, "/memory");
	if (offset < 0) {
		printf("Failed to find memory node in DTB\n");
	}

	/* Get the existing 'reg' property values */
	int *reg_prop = (int *)fdt_getprop(blob, offset, "reg", NULL);

	/* Update the specific values in the 'reg' property */
	reg_prop[2] = cpu_to_fdt32((uint32_t)(ddr_size >> 32)); /* High 32 bits */
	reg_prop[3] = cpu_to_fdt32((uint32_t)(ddr_size & 0xFFFFFFFF)); /* Lower 32 bits */
}

static void set_property(void *blob, const char *node_path, const char *prop_name,
			 uint8_t card_type)
{
	int node_offset = fdt_path_offset(blob, node_path); /* get the offset of the node */

	if (node_offset < 0)
		printf("Failed to find the node offset of %s node with err %d, "
			"card_type is 0x%x\n", node_path, node_offset, card_type);
	/* add the flag property to the node */
	int prop_offset = fdt_setprop_empty(blob, node_offset, prop_name);

	if (prop_offset < 0)
		printf("Failed to set %s peoperty with err %d, "
			"card_type is 0x%x\n", prop_name, prop_offset, card_type);
}

static void delete_property(void *blob, const char *node_path, const char *prop_name,
			    uint8_t card_type)
{
	int node_offset = fdt_path_offset(blob, node_path);

	if (node_offset < 0)
		printf("Failed to find the node offset of %s with err %d, "
			"card_type is 0x%x\n", node_path, node_offset, card_type);
	/* delete the flag property to the node */
	int prop_offset = fdt_delprop(blob, node_offset, prop_name);

	if (prop_offset == -1)
		printf("%s is already not present in %s, "
			"card_type is 0x%x\n", prop_name, node_path, card_type);
}

void fdt_fixup_board_rfic(void *blob)
{
	u8 rf1_card_type;
	u8 rf2_card_type;
	const char *rf1_node_path = "/spi@68490000";
	const char *rf2_node_path = "/spi@68491000";
	const char *prop_name = "read_cs_toggle_quirk";

	rf1_card_type = get_rf1_card_type();

	if (rf1_card_type == RF_CARD_METANOIA)
		set_property(blob, rf1_node_path, prop_name, rf1_card_type);
	else if (rf1_card_type == RF_CARD_ADI)
		delete_property(blob, rf1_node_path, prop_name, rf1_card_type);
	else
		printf("Invalid RF card type\n");

	rf2_card_type = get_rf2_card_type();

	if (rf2_card_type == RF_CARD_METANOIA)
		set_property(blob, rf2_node_path, prop_name, rf2_card_type);
	else if (rf2_card_type == RF_CARD_ADI)
		delete_property(blob, rf2_node_path, prop_name, rf2_card_type);
	else
		printf("Invalid RF card type\n");
}

#ifdef PCIE_ENV_NOR
/**
 * @brief Initializes PCIe interface for both link 4 and link 3 modes
 * This function initializes the PCIe interface for both link 4 and link 3 modes
 * based on the specified mode of operation for each link.
 * @param mode pci_env is a variable whose lower nibble decides Link3 & Link 4 Modes
 * @param port in string form sets Link4_mode : Link3_mode
 * @return None
 */
static unsigned char  pci_port_mode_type_env(char *port)
{
	unsigned char  port_id = NA;

	if (!strncmp(port, "RC", 2))
		port_id = RC;
	else if (!strncmp(port, "EP", 2))
		port_id = EP;
	else if (!strncmp(port, "LB", 2))
		port_id = LB;

	return port_id;
}

int get_pci_ports_mode_env(char *pci_env, u8 link3_sts, u8 link4_sts, unsigned char *port0_mode, unsigned char *port1_mode)
{
	char port0[PCI_PORT_TYPE_SIZE], port1[PCI_PORT_TYPE_SIZE];
	size_t colon_index = strcspn(pci_env, ":");

	/* FORMAT ex: setenv pci EP:NA */
	if (colon_index != 2 || strlen(pci_env) != 5){
		printf("## Error: Invalid pci env values");
		return -1;
	}

	// Copy the substring before the ':' to port0
	strncpy(port0, pci_env, colon_index);
	port0[colon_index] = '\0';

	/* Copy the substring after the ':' to port1 */
	strncpy(port1, pci_env + colon_index + 1,PCI_PORT_TYPE_SIZE);

	*port0_mode = (link3_sts == LINK_PCIE) ? pci_port_mode_type_env(port0) : NA;
	*port1_mode = (link4_sts == LINK_PCIE) ? pci_port_mode_type_env(port1) : NA;
	return 0;
}
#endif

/**
 * @brief Initializes PCIe interface for both link 4 and link 3 modes
 * This function initializes the PCIe interface for both link 4 and link 3 modes
 * based on the specified mode of operation for each link.
 * @param mode pci_env is a variable whose lower nibble decides Link3 & Link 4 Modes
 * @param mode_port0_mode of operation for PCIe link 3 (NA LP EP RC) bit 0:1
 * @param mode_port1_mode of operation for PCIe link 4 (NA LP EP RC) bit 2:3
 * @return None
 */
void get_pci_ports_mode(int pci_env, u8 link3_sts, u8 link4_sts, unsigned char *port0_mode, unsigned char *port1_mode)
{
	pci_env = pci_env & 0xFF;

	*port0_mode = (link3_sts == LINK_PCIE) ? ((pci_env>>LINK3_OFF) & LINK_MASK): NA;
	*port1_mode = (link4_sts == LINK_PCIE) ? ((pci_env>>LINK4_OFF) & LINK_MASK): NA;
}

/**
 * @brief Changes bootargs property in Flattened Device Tree
 * Adds param character string to the end of present bootargs
 * ex: const char *paramas = "pci=nomis\0"
 *
 * @param input_blob (fllattened device tree) , parameter to add
 * @see add_params_bootargs
 */
void add_params_bootargs(void *input_blob, const char *param){
	  // Modify the FDT to add the pci=nomsi parameter
    int offset = fdt_path_offset(input_blob, "/chosen");
    if (offset < 0) {
        fprintf(stderr, "Failed to find /chosen node in the FDT\n");
        free(input_blob);
        return;
    }

    int bootargs_len = strlen(param) + 1;
    int old_bootargs_len = 0;

    const char *old_bootargs = fdt_getprop(input_blob, offset, "bootargs", &old_bootargs_len);
    if (old_bootargs && old_bootargs_len > 0) {
        bootargs_len += old_bootargs_len;
    }

    char *new_bootargs = malloc(bootargs_len);
    if (!new_bootargs) {
        fprintf(stderr, "Memory allocation error\n");
        free(input_blob);
        return;
    }

    if (old_bootargs) {
        snprintf(new_bootargs, bootargs_len, "%s %s", old_bootargs, param);
    } else {
        snprintf(new_bootargs, bootargs_len, "%s", param);
    }

    fdt_setprop(input_blob, offset, "bootargs", new_bootargs, bootargs_len);
}

/**
 * @brief Changes bootargs property in fdt
 * to add pcie properties
 *
 * @param blob (fllattened device tree)
 * @see pcie_add_bootargs
 */
void pcie_add_bootargs(void *blob)
{
	const char *pci_param = "default_hugepagesz=32M \0";
	add_params_bootargs(blob, pci_param);
}


/**
 * @brief Determine which pcie Link mode is connected
 *               to Raptor2 SoC and populate the corresponding
 *               Link property in device tree
 *
 * @param blob
 * @see fdt_fixup_board_pcie
 */
void fdt_fixup_board_pcie(void *blob)
{
#ifdef PCIE_ENV_NOR
	char *env_pci_mode;
	int pcie_en_len;
#endif
	int link3_sts = LINK_UNUSED;
	int link4_sts = LINK_UNUSED;
	int pcie_env = 0;
	unsigned char link4_mode=NA, link3_mode=NA;
	int lb_noff = 0;
	int active_partition;

	pcie_env = get_pci_active_interface_type();
	active_partition = (*(uint64_t *)(R2_ACTIVE_PARTITION_OFFSET));
	/*if we boot into failsafe partition disable PCIe*/
	if (active_partition != FAILSAFE_PARTITION_ID) {
		link3_sts = get_link_status(STS_LINK3);
		link4_sts = get_link_status(STS_LINK4);
	}
	else{
		printf("\r\n Failsafe: PCIe Disabled!!\r\n");
	}
	/* if both pcie link are active together
	 * or none of the pcie links are active
	 * disabling PCIe Controllers node
	 */
	if ((link3_sts == LINK_PCIE && link4_sts == LINK_PCIE)
		|| (link3_sts != LINK_PCIE && link4_sts != LINK_PCIE)) {
		/*Delete these pci-based reserve memory if pcie link not detected*/
		printf("Both PCIe controllers activation are not supported simultaneously\n");
		printf("Disabling Link-3 & Link-4 PCIe\n");
		fdt_del_node_and_alias(blob, "pci-dmall0");
		fdt_del_node_and_alias(blob, "pci-glb0");
		fdt_status_disabled_by_alias(blob, "pcie_rc_link4");
		fdt_status_disabled_by_alias(blob, "pcie_ep_link4");
		fdt_status_disabled_by_alias(blob, "pcie_rc_link3");
		fdt_status_disabled_by_alias(blob, "pcie_ep_link3");
		lb_noff = fdt_node_offset_by_compatible(blob, -1, "edgeq,virt");
		if(lb_noff < 0) {
			printf("Failed to find virtdma node\n");
		}
		else {
			printf("Link 4 DMA unavailable for virtdma. ret:%d\n",
				fdt_delprop(blob, lb_noff, \
				"dma-channels"));
		}
		return;
	}

	get_pci_ports_mode(pcie_env, link3_sts, link4_sts, &link3_mode, &link4_mode);
	if (!pcie_env)
	{
		printf("PCI variable set as 0x%x\n", pcie_env);
	}

#ifdef PCIE_ENV_NOR
	/* EEPROM PCIe says both link disabled try NOR FLASH*/
	if((link3_mode==NA) && (link4_mode==NA))
	{
		env_pci_mode = env_get("pci");
		if (env_pci_mode != NULL)
		{
			printf("PCI variable set as %s\n", env_pci_mode);
			pcie_en_len = strlen(env_pci_mode);
			if(pcie_en_len > 6){
				printf("PCIe variable not correct\n");
				return;
			}
		}
	get_pci_ports_mode_env(env_pci_mode,  link3_sts, link4_sts, &link4_mode, &link3_mode);
	}
#endif

	/*Selectively add bootargs needed for pcie app*/
	if( link4_mode || link3_mode ){
		/*Add PCIe related bootargs if any controller is detected*/
		pcie_add_bootargs(blob);
	} else {
		/*Delete these pci-based reserve memory if pcie link not detected*/
		fdt_del_node_and_alias(blob, "pci-dmall0");
		fdt_del_node_and_alias(blob, "pci-glb0");
	}

	if(link4_mode != LB) {
		lb_noff = fdt_node_offset_by_compatible(blob, -1, "edgeq,virt");
		if(lb_noff < 0) {
			printf("Failed to find virtdma node\n");
		}
		else {
			printf("Link 4 DMA unavailable for virtdma. ret:%d\n",
				fdt_delprop(blob, lb_noff, \
				"dma-channels"));
		}
	}

	switch(link4_mode)
	{
	case EP:	 fdt_status_okay_by_alias(blob, "pcie_ep_link4");
			  printf("Enabling L4 EP\n");
			  break;
	case RC:	 fdt_status_okay_by_alias(blob, "pcie_rc_link4");
			  printf("Enabling L4 RC\n");
			  break;
	case LB:	  printf("Enabling L4 LB\n");
			  break;
	default:	  fdt_status_disabled_by_alias(blob, "pcie_rc_link4");
			  fdt_status_disabled_by_alias(blob, "pcie_ep_link4");
			  printf("Disabling L4\n");
	 }

	switch(link3_mode)
	{

	case EP:	fdt_status_okay_by_alias(blob, "pcie_ep_link3");
			printf("Enabling L3 EP\n");
			break;
	case RC:	fdt_status_okay_by_alias(blob, "pcie_rc_link3");
			printf("Enabling L3 RC\n");
			break;
	case LB:	fdt_status_okay_by_alias(blob, "pcie_lb_link3");
			printf("Enabling L3 LB\n");
			break;
	default:	printf("Link 3 is Not Supported\n");
			fdt_status_disabled_by_alias(blob, "pcie_rc_link3");
			fdt_status_disabled_by_alias(blob, "pcie_ep_link3");
			fdt_status_disabled_by_alias(blob, "pcie_lb_link3");
			printf("Disabling L3\n");
	}
}

/**
 * @brief Determine which clock source is connected
 * 		  to Raptor2 SoC and populate the corresponding
 * 		  XGMAC property in device tree
 *
 * @details Please note that this function populates properties
 * 			for all the XGMAC interfaces regardless of its status
 * 			(ok/disabled).
 * 			This function is independent of @fdt_fixup_board_enet()
 * 			because in the future it might also need to populate
 * 			property for other device nodes.
 * @param blob
 * @see fdt_fixup_board_enet
 */
void fdt_fixup_board_clock_src(void *blob)
{
	char *dpll_flag = NULL;
	u8 clk_src = SoC_CLK_DEFAULT; /* default to VCTCXO */
	int nodeoffset = -1; /* offset for DCTCXO device node */
	u32 i = 0, eth_path_cnt = 0;

	/* Detect clock source - DPLL/DCTCXO/VCTCXO
	 * DPLL has the highest precedence. If it's
	 * not present, it will check DCTCXO and
	 * VCTCXO. */
	RAPTOR2_DBG("\n");
	dpll_flag = env_get("dpll_enable");
	if (dpll_flag && strcmp(dpll_flag, "1") == 0) {
		printf("Use DPLL as the SoC ref clk\n");
		clk_src = SoC_CLK_DPLL;
	} else {
#ifdef CONFIG_I2C_DCTCXO
		struct udevice *bus, *udev;
		u32 chip_flags = 0;

		if (uclass_get_device(UCLASS_I2C, R2_A0_DCTCXO_I2C_BUS, &bus)) {
			printf("Failed to get udevice for I2C bus %d\n", R2_A0_DCTCXO_I2C_BUS);
			return;
		}

		if (i2c_deblock(bus)) {
			printf("Failed to recover I2C bus %d from an unknown state\n", R2_A0_DCTCXO_I2C_BUS);
			return;
		}

		if (dm_i2c_probe(bus, R2_A0_DCTCXO_I2C_SLAVE_ADDR, chip_flags, &udev)) {
			printf("Failed to probe DCTCXO on I2C bus %d @ 0x%02X\n", R2_A0_DCTCXO_I2C_BUS, R2_A0_DCTCXO_I2C_SLAVE_ADDR);
			clk_src = SoC_CLK_VCTCXO;
		} else {
			printf("Successfully probed DCTCXO on I2C bus %d @ 0x%02X\n", R2_A0_DCTCXO_I2C_BUS, R2_A0_DCTCXO_I2C_SLAVE_ADDR);
			clk_src = SoC_CLK_DCTCXO;
		}
#else
		printf("CONFIG_I2C_DCTCXO not defined. Select VCTCXO as the SoC reference clock\n");
		clk_src = SoC_CLK_DEFAULT;
#endif /* CONFIG_I2C_DCTCXO */
	}
	/* Populate the corresponding property for XGMAC */
	eth_path_cnt = sizeof(ether_path) / sizeof(char *);
	RAPTOR2_DBG("eth_path_cnt: %u, clk_src: %u\n", eth_path_cnt, clk_src);
	for (i = 0; i < eth_path_cnt; i++) {
		RAPTOR2_DBG("i: %u, ether path: %s\n", i, ether_path[i]);
		if (fdt_find_and_setprop(blob, ether_path[i],
			CLOCK_SRC_PROP, (const void *)&clk_src, sizeof(clk_src), 1)) {
			printf("Failed to set %s property\n", CLOCK_SRC_PROP);
		}
	}

	/* If using DCTCXO, set DCTCXO node status "okay" */
	if (clk_src == SoC_CLK_DCTCXO) {
		RAPTOR2_DBG("Setting DCTCXO node's status to 'okay'...\n");
		if ((nodeoffset = fdt_find_or_add_subnode(blob, 0, "dctcxo")) < 0) {
			printf("Failed to find/add DCTCXO node\n");
			return;
		}

		if (fdt_set_node_status(blob, nodeoffset, FDT_STATUS_OKAY, 0)) {
			printf("Failed to set status 'okay' for DCTCXO\n");
			return;
		} else {
			RAPTOR2_DBG("Successfully set DCTCXO node's status to okay\n");
		}
	}
}

int ft_board_setup(void *blob, bd_t *bd)
{
	fdt_fixup_board_enet(blob);

	/* Change linux emmc clock as per "emmc_freq" env variable */
	fdt_fixup_board_emmc(blob);

	fdt_fixup_board_pcie(blob);

	fdt_fixup_board_clock_src(blob);

	fdt_fixup_board_rfic(blob);

	fdt_fixup_board_sus(blob);

	fdt_fixup_board_ddr(blob);

	return 0;
}
#endif

int board_init(void)
{
	/* For now nothing to do here. */
	return 0;
}

