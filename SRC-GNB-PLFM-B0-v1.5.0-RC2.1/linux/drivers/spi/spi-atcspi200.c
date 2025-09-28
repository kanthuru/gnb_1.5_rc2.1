// SPDX-License-Identifier: GPL-2.0
/*
 * Andestech SPI controller driver
 *
 * Author: Nylon Chen
 *	nylon7@andestech.com
 *
 * 2020 (c) Andes Technology Corporation

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <soc/edgeq/raptor2_rf_spi.h>
#include <linux/pinctrl/consumer.h>

#define SPI_XFER_BEGIN		(1 << 0)
#define SPI_XFER_END		(1 << 1)
#define SPI_XFER_ONCE		(SPI_XFER_BEGIN | SPI_XFER_END)

#define SPI_NAME		"atcspi200"
#define SPI_MAX_HZ		50000000
#define MAX_TRANSFER_LEN	512
#define CHUNK_SIZE		1
#define SPI_NOR_TIMEOUT		0x20000000
#define SPI_TIMEOUT		0x9000
#define NSPI_MAX_CS_NUM		1
#define DATA_LENGTH(x)		((x-1)<<8)

#define NUM_INIT_RFDEVS		0
#define MAX_RAPTOR2_RFDEVS	2

/* SPI Transfer Control Register  */
#define ATCSPI200_TRANSFMT_OFFSET		24
#define ATCSPI200_TRANSFMT_MASK			(0x0F<<ATCSPI200_TRANSFMT_OFFSET)
#define ATCSPI200_TRANSMODE_WR_SYNC		(0<<ATCSPI200_TRANSFMT_OFFSET)
#define ATCSPI200_TRANSMODE_W_ONLY	(1<<ATCSPI200_TRANSFMT_OFFSET)
#define ATCSPI200_TRANSMODE_R_ONLY	(2<<ATCSPI200_TRANSFMT_OFFSET)
#define ATCSPI200_TRANSMODE_WR		(3<<ATCSPI200_TRANSFMT_OFFSET)
#define ATCSPI200_TRANSCTRL_WRTRANCNT_OFFSET	12
#define ATCSPI200_TRANSCTRL_WRTRANCNT_MASK	(0x1FF<<ATCSPI200_TRANSCTRL_WRTRANCNT_OFFSET)
#define ATCSPI200_TRANSCTRL_RDTRANCNT_OFFSET	0
#define ATCSPI200_TRANSCTRL_RDTRANCNT_MASK	(0x1FF<<ATCSPI200_TRANSCTRL_RDTRANCNT_OFFSET)
#define ADDR_EN         29
#define CMD_EN          30
#define TRAMODE_ADDREN  BIT(ADDR_EN)
#define TRAMODE_CMDEN   BIT(CMD_EN)

/* SPI Control Register */
#define ATCSPI200_CTRL_TXFIFORST_MASK		(1<<2)
#define ATCSPI200_CTRL_RXFIFORST_MASK		(1<<1)
#define ATCSPI200_CTRL_SPIRST_MASK		(1<<0)

/* SPI Transfer Format Register */
#define ATCSPI200_TRANSFMT_CPHA_MASK		(1UL << 0)
#define ATCSPI200_TRANSFMT_CPOL_MASK		(1UL << 1)
#define ATCSPI200_TRANSFMT_DATA_MERGE_MASK		(1UL << 7)

/* SPI Status Register */
#define ATCSPI200_STATUS_TXEMPTY_OFFSET		(1<<22)
#define ATCSPI200_STATUS_RXEMPTY_OFFSET		(1<<14)
#define ATCSPI200_STATUS_TXNUM_LOWER_OFFSET	(16)
#define ATCSPI200_STATUS_TXNUM_LOWER_MASK	(0x1F<<ATCSPI200_STATUS_TXNUM_LOWER_OFFSET)
#define ATCSPI200_STATUS_RXNUM_LOWER_OFFSET	(8)
#define ATCSPI200_STATUS_RXNUM_LOWER_MASK	(0x1F<<ATCSPI200_STATUS_RXNUM_LOWER_OFFSET)
#define ATCSPI200_STATUS_SPIACTIVE_MASK		(1<<0)

/* SPI Interface timing Setting */
#define ATCSPI200_TIMING_SCLK_DIV_MASK		0xFF
#define READ_CS_TOGGLE_QUIRK			1
#define NOR_FLASH_QUIRK				2
#define RFI_DEVICE_QUIRK			20
#define SCLK_DIV				100
#define ADI_TIMING				0x22C8
#define MT_TIMING				0x22C8
#define NOR_FLASH_TIMING			0x1	/* NOR FLASH at 15.36MHz */

/* SPI active bit setting */
#define ATCSPI200_ACTIVE_LOW_USLEEP_MIN 6
#define ATCSPI200_ACTIVE_LOW_USLEEP_MAX 8

/* For 4 bytes of data, the while loop will usually take 4 cycles (normally
 * less but just to be safe here) till the active bit goes low, and the maximum
 * bytes SPI can transfer is 1024 bytes. Therefore, in the worst case where
 * SPI master is transferring 1024 bytes, it should take at most 1024
 * cycles for the active bit to go low.
 */
#define ATCSPI200_ACTIVE_LOW_TIMEOUT	1024

/* ATCSPI200 registers  */
#define IDRev		0x00	// ID and Revision Register
#define TransFmt	0x10    // SPI Transfer Format Register
#define DirectIO	0x14	// SPI Direct IO Control Register
#define TransCtrl	0x20	// SPI Transfer Control Register
#define Cmd		0x24	// SPI Command Register
#define Addr		0x28	// SPI Address Register
#define Data		0x2C	// SPI Data Register
#define Ctrl		0x30	// SPI Control Register
#define Status		0x34	// SPI Status Register
#define IntrEn		0x38	// SPI Interrupt Enable Register
#define IntrSt		0x3C	// SPI Interrupt Status Registe
#define Timing		0x40	// SPI Interface timing Register

/* SPI FSM workaround-related constants */
#define SPI_ACTIVE_RETRY_MAX   		50
#define SPI_MAX_READ_BYTES 	   		4
#define SPI_CS_LINE_LOW				0
#define SPI_CS_LINE_HIGH		    1

struct atcspi200_spi {
	struct device *dev; /* parent device */
	struct device *pltdev; /* platform device */
	void __iomem	*regs;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pctrl_state;
	struct pinctrl_state *gpio_line;
	struct pinctrl_state *spim1_cs_line;
	struct clk	*clk;
	size_t		trans_len;
	size_t		data_len;
	size_t		cmd_len;
	u32		clk_rate;
	u8		cmd_buf[16];
	u8		*din;
	u8		*dout;
	unsigned int	max_transfer_length;
	unsigned int	freq;
	unsigned int	mode;
	unsigned int	mtiming;
	int		timeout;
	spinlock_t	lock;
	struct mutex	transfer_lock;
	u32             reg_shift;	/* reg offset shift */
	u32             quirks;
	u8  			data_merge;
	u8 				spi_hw_workaround;
};

static uint32_t nrfdevs = NUM_INIT_RFDEVS;
static struct atcspi200_spi *raptor2_rfdevs[MAX_RAPTOR2_RFDEVS] = {NULL, NULL};

static void atcspi200_spi_write(struct atcspi200_spi *spi, int offset, u32 value)
{
	dev_dbg(spi->dev, "[%s] offset: 0x%08X, val: 0x%08X, shift: 0x%X\n", __func__,
				offset, value, spi->reg_shift);
	iowrite32(value, spi->regs + (offset << (spi->reg_shift - 1)));
}

static u32 atcspi200_spi_read(struct atcspi200_spi *spi, int offset)
{
	u32 val = ioread32(spi->regs + (offset << (spi->reg_shift - 1)));
	dev_dbg(spi->dev, "[%s] offset: 0x%08X, val: 0x%08X, shift: 0x%X\n", __func__,
				offset, val, spi->reg_shift);
	return val;
}

static int atcspi200_spi_setup(struct atcspi200_spi *spi)
{
	unsigned int	format_val;
	u32	timing;
	u8	div;
	int ctrl_val = atcspi200_spi_read(spi, Ctrl);

	ctrl_val |= (ATCSPI200_CTRL_TXFIFORST_MASK|ATCSPI200_CTRL_RXFIFORST_MASK|
			ATCSPI200_CTRL_SPIRST_MASK);
	atcspi200_spi_write(spi, Ctrl, ctrl_val);
	while (((atcspi200_spi_read(spi, Ctrl))&(ATCSPI200_CTRL_TXFIFORST_MASK|
			ATCSPI200_CTRL_RXFIFORST_MASK|ATCSPI200_CTRL_SPIRST_MASK))
			&& (spi->timeout--))
		if (!spi->timeout) {
			if (spi->quirks & NOR_FLASH_QUIRK)
				spi->timeout = SPI_NOR_TIMEOUT;
			else {
				usleep_range(20, 30);
				spi->timeout = SPI_TIMEOUT;
			}
			return -EINVAL;
		}

	spi->cmd_len = 0;
	format_val = spi->mode|DATA_LENGTH(8);
	if (spi->mode & SPI_CPHA)
		format_val |= ATCSPI200_TRANSFMT_CPHA_MASK;
	if (spi->mode & SPI_CPOL)
		format_val |= ATCSPI200_TRANSFMT_CPOL_MASK;
	if (spi->data_merge)
		format_val |= ATCSPI200_TRANSFMT_DATA_MERGE_MASK;

	atcspi200_spi_write(spi, TransFmt, format_val);

	timing = atcspi200_spi_read(spi, Timing);
	timing &= ~ATCSPI200_TIMING_SCLK_DIV_MASK;

	if (spi->freq >= spi->clk_rate)
		div = ATCSPI200_TIMING_SCLK_DIV_MASK;
	else {
		for (div = 0; div < ATCSPI200_TIMING_SCLK_DIV_MASK; div++) {
			if (spi->freq >= spi->clk_rate / (2 * (div + 1)))
				break;
		}
	}
	timing |= div;
	if (spi->quirks & READ_CS_TOGGLE_QUIRK)
		atcspi200_spi_write(spi, Timing, MT_TIMING);
	else if (spi->data_merge)
		atcspi200_spi_write(spi, Timing, 0x24a);
	else if (spi->quirks & NOR_FLASH_QUIRK)
		atcspi200_spi_write(spi, Timing, NOR_FLASH_TIMING);
	else
		atcspi200_spi_write(spi, Timing, ADI_TIMING);

	return 0;
}

int raptor2_reconf_rfspidev(void)
{
	int i, ret;
	struct atcspi200_spi *spi;

	if (!nrfdevs) {
		pr_err("%s: No Raptor2 SPI RF Devs available\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < nrfdevs; i++) {
		spi = raptor2_rfdevs[i];
		ret = atcspi200_spi_setup(spi);
		if (ret < 0) {
			pr_err("%s: atcspi200_spi_setup failed for %s\n",
					__func__, spi->dev->of_node->name);
			return ret;
		}
	}

	pr_info("%s: Successfully reconfigured SPI RF interface for Raptor\n", __func__);

	return 0;
}
EXPORT_SYMBOL(raptor2_reconf_rfspidev);

static int atcspi200_spi_stop(struct atcspi200_spi *spi)
{
	dev_dbg(spi->dev, "[%s][%d]\n", __func__, __LINE__);
	while ((atcspi200_spi_read(spi, Status) & ATCSPI200_STATUS_SPIACTIVE_MASK)
			&& (spi->timeout--))
		if (!spi->timeout) {
			if (spi->quirks & NOR_FLASH_QUIRK)
				spi->timeout = SPI_NOR_TIMEOUT;
			else {
				usleep_range(20, 30);
				spi->timeout = SPI_TIMEOUT;
			}
			return -EINVAL;
		}
	return 0;
}

static int atcspi200_spi_start(struct atcspi200_spi *spi, struct spi_transfer *t)
{
	int i, trans_len = 0;
	int tc = atcspi200_spi_read(spi, TransCtrl);
	int rflag = 0;
	int spi_active = 1;

	tc &= ~(ATCSPI200_TRANSCTRL_WRTRANCNT_MASK|ATCSPI200_TRANSCTRL_RDTRANCNT_MASK
			|ATCSPI200_TRANSFMT_MASK | TRAMODE_ADDREN | TRAMODE_CMDEN);
	if ((spi->dout) && (spi->cmd_len)) {
		if (spi->quirks & READ_CS_TOGGLE_QUIRK) {
			tc |= ATCSPI200_TRANSMODE_W_ONLY;
			rflag = 1;
		}
		else
			tc |= ATCSPI200_TRANSMODE_WR;
	}
	else if (spi->dout)
		tc |= ATCSPI200_TRANSMODE_R_ONLY;
	else
		tc |= ATCSPI200_TRANSMODE_W_ONLY;
	if (spi->din)
		trans_len = spi->trans_len;
	tc |= (spi->cmd_len+trans_len-1) << ATCSPI200_TRANSCTRL_WRTRANCNT_OFFSET;

	if (spi->dout)
		tc |= (spi->trans_len-1) << ATCSPI200_TRANSCTRL_RDTRANCNT_OFFSET;

	atcspi200_spi_write(spi, TransCtrl, tc);

	for (i = 0; i < spi->cmd_len; i++)
		atcspi200_spi_write(spi, Data, spi->cmd_buf[i]);
	atcspi200_spi_write(spi, Cmd, 0);

	if(rflag)
	{
		while(spi_active)
		{
			spi_active = atcspi200_spi_read(spi, Status) & ATCSPI200_STATUS_SPIACTIVE_MASK;
		}
		tc &= ~(ATCSPI200_TRANSFMT_MASK);
		atcspi200_spi_write(spi, TransCtrl, (tc | ATCSPI200_TRANSMODE_R_ONLY));
		atcspi200_spi_write(spi, Cmd, 0);
	}
	return 0;
}

static void atcspi200_spi_tx(struct atcspi200_spi *spi, const u8 *dout)
{
	atcspi200_spi_write(spi, Data, *dout);
}

static int atcspi200_spi_rx(struct atcspi200_spi *spi, u8 *din, unsigned int bytes)
{
	u32 tmp_data = atcspi200_spi_read(spi, Data);
	*din = tmp_data;
	return bytes;
}

static u32 atcspi200_spi_set_cs_line(struct atcspi200_spi *spi, u8 cs_val)
{
	int ret;

	if (cs_val) {
		/* set cs pin as alt function 3 */
		ret = pinctrl_select_state(spi->pinctrl, spi->spim1_cs_line);
		if (ret) {
			dev_err(spi->pltdev, "Failed to select spim1_cs_line pinctrl state\n");
			return ret;
		}
		dev_dbg(spi->pltdev, "Selected spim1_cs_line pinctrl state\n");
	} else {
		/* set cs pin as gpio */
		ret = pinctrl_select_state(spi->pinctrl, spi->gpio_line);
		if (ret) {
			dev_err(spi->pltdev, "Failed to select gpio_line pinctrl state\n");
			return ret;
		}
		dev_dbg(spi->pltdev, "Selected gpio_line pinctrl state\n");
	}
	return 0;
}

static int atcspi200_spi_wait_for_active_low_timeout(struct atcspi200_spi *spi)
{
	int timeout_remain = ATCSPI200_ACTIVE_LOW_TIMEOUT;
	int active = 0;

	active = atcspi200_spi_read(spi, Status) & ATCSPI200_STATUS_SPIACTIVE_MASK;
	while (active && --timeout_remain) {
		udelay(ATCSPI200_ACTIVE_LOW_USLEEP_MAX);
		active = atcspi200_spi_read(spi, Status) & ATCSPI200_STATUS_SPIACTIVE_MASK;
	}

	return (timeout_remain) ? 0 : -ETIMEDOUT;
}

static int atcspi200_spi_get_rx_cnt(struct atcspi200_spi *spi)
{
	return (atcspi200_spi_read(spi, Status) & ATCSPI200_STATUS_RXNUM_LOWER_MASK)
				>> ATCSPI200_STATUS_RXNUM_LOWER_OFFSET;
}

static int atcspi200_spi_transfer_workaround(struct spi_device *atcspi200_spi,
		struct spi_transfer *t, unsigned long flags)
{
	struct atcspi200_spi *spi = spi_master_get_devdata(atcspi200_spi->master);
	u8 *rx_buf = (u8 *)t->rx_buf;
	u32 *tx_buf = (u32 *)t->tx_buf;
	unsigned long data_len = t->len;
	u32 tc = 0, data = 0, rx_cnt = 0, i = 0, bytes_written = 0, read_num = 0;
	int ret = 0;

	/* Data merge should be enabled for HW workaround */
	WARN_ON(spi->data_merge == 0);
	WARN_ON(t->rx_buf == NULL && t->tx_buf == NULL);
	/* Write and Read at the same time transaction - Instead of
	 * waiting for active low after every write transaction, the driver
	 * waits for active low before every write and every read transaction
	 * begins in order to save some CPU cycles. The same CPU cycles
	 * busy-waiting for active low after each transaction could be used
	 * by the next transaction or user-space program that made this ioctl call.
	 * As long as SPI active bit is low (idle) before each transaction,
	 * the SPI bus will function as expected.
	 */
	if (rx_buf && tx_buf) {
		/* We always write and read a constant 4 bytes at a time.
		 * Although the hw bug affects only the read portion,
		 * since we do write and read simultaneously, the lengths
		 * of the read and write must be the same.
		 */
		tc |= ((SPI_MAX_READ_BYTES - 1) << ATCSPI200_TRANSCTRL_WRTRANCNT_OFFSET);
		tc |= ((SPI_MAX_READ_BYTES - 1) << ATCSPI200_TRANSCTRL_RDTRANCNT_OFFSET);
		tc |= ATCSPI200_TRANSMODE_WR_SYNC;
		while (bytes_written < data_len) {
			/* Blocking function with timeout to wait for SPI
			 * active bit to go low. This is required because
			 * write transaction should not begin before previous
			 * one completes.
			 */
			ret = atcspi200_spi_wait_for_active_low_timeout(spi);
			if (ret)
				return ret;

			/* Start writing to SPI regs */
			atcspi200_spi_write(spi, TransCtrl, tc);
			atcspi200_spi_write(spi, Cmd, 0x1);
			dev_dbg(spi->dev, "[%s][%d] data_len: %lu, bytes written: %u\n",
					__func__, __LINE__, data_len, bytes_written);
			dev_dbg(spi->dev, "[%s][%d] tx_buf: 0x%08X\n",
					__func__, __LINE__, *tx_buf);
			atcspi200_spi_write(spi, Data, *tx_buf);
			tx_buf++;

			ret = atcspi200_spi_wait_for_active_low_timeout(spi);
			if (ret)
				return ret;

			data = atcspi200_spi_read(spi, Data);
			memcpy(rx_buf, &data, sizeof(data));
			dev_dbg(spi->dev, "[%s][%d] data: 0x%08X, rx_buf: 0x%X\n",
						__func__, __LINE__, data, *(u32 *)rx_buf);
			rx_buf += sizeof(data);

			bytes_written += sizeof(*tx_buf);
		}
	}
	/* Write only transaction - The if section below takes care of the
	 * write only transaction if tx_buf is valid. Instead of waiting
	 * for active low after every write transaction, the driver waits
	 * for active low before every write transaction begins in order
	 * to save some CPU cycles. The same CPU cycles busy-waiting for
	 * active low after each transaction could be used by the next
	 * transaction or user-space program that made this ioctl call.
	 * As long as SPI active bit is low (idle) before each transaction,
	 * the SPI bus will function as expected.
	 */
	else if (tx_buf) {
		tc |= ((data_len - 1) << ATCSPI200_TRANSCTRL_WRTRANCNT_OFFSET);
		tc |= ATCSPI200_TRANSMODE_W_ONLY;

		/* Blocking function with timeout to wait for SPI
		 * active bit to go low. This is required because
		 * write transaction should not begin before previous
		 * one completes.
		 */
		ret = atcspi200_spi_wait_for_active_low_timeout(spi);
		if (ret)
			return ret;

		/* Start writing to SPI regs */
		atcspi200_spi_write(spi, TransCtrl, tc);
		atcspi200_spi_write(spi, Cmd, 0x1);

		/* Write data to Data register if any */
		while (bytes_written < data_len) {
			dev_dbg(spi->dev, "[%s][%d] data_len: %lu, bytes written: %u\n",
					__func__, __LINE__, data_len, bytes_written);
			dev_dbg(spi->dev, "[%s][%d] tx_buf: 0x%08X\n",
					__func__, __LINE__, *tx_buf);
			atcspi200_spi_write(spi, Data, *tx_buf);
			tx_buf++;
			bytes_written += sizeof(*tx_buf);
		}
	}
	/* Read only transaction - The if section below takes care of the read-only
	 * transaction. For the same reason, the driver waits for active low before
	 * each transaction instead of after in order to make better use of the
	 * CPU. Additionally, due to a HW bug in SPI, it can only read at most
	 * 4 bytes at a time. Therefore, any read transaction reading more than
	 * 4 bytes will be split into multiple 4-byte chunks to read.
	 */
	else if (rx_buf) {
		/* Only read out 4 bytes at a time */
		tc = 0;
		tc |= ((SPI_MAX_READ_BYTES - 1) << ATCSPI200_TRANSCTRL_RDTRANCNT_OFFSET);
		tc |= ATCSPI200_TRANSMODE_R_ONLY;
		read_num = data_len / SPI_MAX_READ_BYTES;
		read_num += (data_len % SPI_MAX_READ_BYTES) ? 1 : 0;

		/* Ensure active bit is low before proceding to read transaction */
		ret = atcspi200_spi_wait_for_active_low_timeout(spi);

		for (i = 0; i < read_num; i++) {
			/* Request to read 4 bytes from slave device */
			atcspi200_spi_write(spi, TransCtrl, tc);
			atcspi200_spi_write(spi, Cmd, 0x1);

			/* Wait for data to arrive at the master side */
			ret = atcspi200_spi_wait_for_active_low_timeout(spi);

			/* Get rx fifo count. Skip read if 0 */
			rx_cnt = atcspi200_spi_get_rx_cnt(spi);
			if (rx_cnt == 0) {
				dev_info(spi->dev, "[%s] rx empty\n", __func__);
				continue;
			}
			data = atcspi200_spi_read(spi, Data);
			memcpy(rx_buf, &data, sizeof(data));
			dev_dbg(spi->dev, "[%s][%d] i: %u, data: 0x%08X, rx_buf: 0x%X\n",
					__func__, __LINE__, i, data, *(u32 *)rx_buf);
			rx_buf += sizeof(data);
		}
	}

	return ret;
}

static int atcspi200_spi_transfer(struct spi_device *atcspi200_spi,
		struct spi_transfer *t, unsigned long flags)
{
	struct atcspi200_spi *spi = spi_master_get_devdata(atcspi200_spi->master);

	unsigned int event, rx_bytes;
	const u8 *dout = NULL;
	u8 *din = NULL;
	int num_blks, num_chunks, max_trans_len, trans_len;
	int num_bytes;
	u8 *cmd_buf = spi->cmd_buf;
	size_t cmd_len = spi->cmd_len;
	unsigned long data_len = t->len;
	int rf_cnt;
	int ret = 0, timeout = 0;
	u32 addr = 0;
	dev_dbg(&atcspi200_spi->dev, "[%s][%d]\n", __func__, __LINE__);
	max_trans_len = spi->max_transfer_length;

	switch (flags) {
	case SPI_XFER_BEGIN:
		cmd_len = spi->cmd_len = data_len;
		memcpy(cmd_buf, t->tx_buf, cmd_len);
		return 0;
	case 0:
		if (data_len == 0)
			return 0;
		cmd_len = spi->cmd_len;
		memcpy(cmd_buf + cmd_len, t->tx_buf, t->len);
		spi->cmd_len += t->len;
		return 0;

	case SPI_XFER_END:
		if (data_len == 0)
			return 0;
		spi->data_len = data_len;
		spi->din = (u8 *)t->tx_buf;
		spi->dout = (u8 *)t->rx_buf;
		break;

	case SPI_XFER_BEGIN | SPI_XFER_END:
		dev_dbg(&atcspi200_spi->dev, "[%s][%d]\n", __func__, __LINE__);
		spi->data_len = 0;
		spi->din = 0;
		spi->dout = 0;
		cmd_len = spi->cmd_len = data_len;
		if (t->tx_buf)
			memcpy(cmd_buf, t->tx_buf, cmd_len);
		else
			WARN_ON(1);
		t->tx_buf = 0;
		data_len = 0;
		atcspi200_spi_start(spi, t);
		break;
	}
	num_chunks = DIV_ROUND_UP(data_len, max_trans_len);
	din = t->rx_buf;
	dout = t->tx_buf;
	dev_dbg(&atcspi200_spi->dev, "[%s][%d] num_chunks: %d, data_len: %lu, t->len: %u, max_trans_len: %d\n", __func__,
				__LINE__, num_chunks, data_len, t->len, max_trans_len);
	while (num_chunks--) {
		trans_len = min((size_t)data_len, (size_t)max_trans_len);
		spi->trans_len = trans_len;
		num_blks = DIV_ROUND_UP(trans_len, CHUNK_SIZE);
		num_bytes = (trans_len) % CHUNK_SIZE;

		if (spi->quirks & NOR_FLASH_QUIRK)
			timeout = SPI_NOR_TIMEOUT;
		else
			timeout = SPI_TIMEOUT;

		if (num_bytes == 0)
			num_bytes = CHUNK_SIZE;
		atcspi200_spi_start(spi, t);
		dev_dbg(&atcspi200_spi->dev, "[%s][%d] num_blks: %d, timeout: %d\n", __func__, __LINE__,
					num_blks, timeout);
		while (num_blks && (timeout--)) {
			event = atcspi200_spi_read(spi, Status);
			dev_dbg(&atcspi200_spi->dev, "[%s][%d] event: %u\n", __func__, __LINE__, event);
			if ((event & ATCSPI200_STATUS_TXEMPTY_OFFSET) &&
					(t->tx_buf)) {
				atcspi200_spi_tx(spi, dout);
				num_blks -= CHUNK_SIZE;
				dout += CHUNK_SIZE;
			}
			if ((event&ATCSPI200_STATUS_RXNUM_LOWER_MASK) && (t->rx_buf)) {
				rf_cnt = ((event & ATCSPI200_STATUS_RXNUM_LOWER_MASK) >>
						ATCSPI200_STATUS_RXNUM_LOWER_OFFSET);
				if (rf_cnt >= CHUNK_SIZE)
					rx_bytes = CHUNK_SIZE;
				else if (num_blks == 1 && rf_cnt == num_bytes)
					rx_bytes = num_bytes;
				else
					continue;

				if (atcspi200_spi_rx(spi, din, rx_bytes) == rx_bytes) {
					num_blks -= CHUNK_SIZE;
					din = (unsigned char *)din + rx_bytes;
				}
			}
			if (!timeout) {
				printk("spi_xfer: %s() timeout\n", __func__);
				break;
			}
			if (!(spi->quirks & NOR_FLASH_QUIRK))
				usleep_range(20, 30);
		}
		data_len -= trans_len;
		if (data_len) {
			addr = ((spi->cmd_buf[1]<<24) | (spi->cmd_buf[2]<<16) |
					(spi->cmd_buf[3]<<8) | (spi->cmd_buf[4]));
			addr += trans_len;
			spi->cmd_buf[1] = ((addr>>24)&0xff);
			spi->cmd_buf[2] = ((addr>>16)&0xff);
			spi->cmd_buf[3] = ((addr>>8)&0xff);
			spi->cmd_buf[4] = ((addr)&0xff);
			spi->data_len = data_len;
		}
		ret = atcspi200_spi_stop(spi);
	}
	ret = atcspi200_spi_stop(spi);
	return ret;
}
static int atcspi200_spi_transfer_one_message(struct spi_master *master, struct spi_message *m)
{
	struct atcspi200_spi *spi = spi_master_get_devdata(master);
	struct spi_transfer *t;
	unsigned long spi_flags;
	int ret = 0, i = 0;

	m->actual_length = 0;

	spi_flags = SPI_XFER_BEGIN;
	if (spi->spi_hw_workaround)
		atcspi200_spi_set_cs_line(spi, SPI_CS_LINE_LOW);
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (!t->tx_buf && !t->rx_buf)
			spi_flags |= SPI_XFER_ONCE;

		if (list_is_last(&t->transfer_list, &m->transfers))
			spi_flags |= SPI_XFER_END;

		dev_dbg(&master->dev, "[%s] i: %d t->len: %u\n", __func__, i++, t->len);

		/* A workaround function is used here if SPI HW workaround is enabled.
		 * Instead of using spin_lock_irqsave() which disables all IRQs on the
		 * same CPU, the workaround uses mutex lock to achieve the same purpose
		 * of preventing multiple threads controlling SPI HW at the same time.
		 * Since spin_lock_irqsave() is part of the original source code provided
		 * by the SPI vendor, it is kept here as it is.
		 */
		if (spi->spi_hw_workaround) {
			if (ret)
				break;
			ret = atcspi200_spi_transfer_workaround(m->spi, t, spi_flags);
		} else {
			ret = atcspi200_spi_transfer(m->spi, t, spi_flags);
		}

		if (ret)
			break;
		m->actual_length += t->len;
		spi_flags = 0;
	}

	if (spi->spi_hw_workaround)
		atcspi200_spi_set_cs_line(spi, SPI_CS_LINE_HIGH);

	m->status = ret;
	spi_finalize_current_message(master);

	return 0;
}

static int atcspi200_spi_set_gpio_padmux(struct atcspi200_spi *spi)
{
	/*
	 * Here in this function only GPIO47 is marked as GPIO and rest are
	 * marked as SPI
	 */
	u32 ret;

	/* Get the pinctrl handle from the device */
	spi->pinctrl = devm_pinctrl_get(spi->pltdev);
	if (IS_ERR(spi->pinctrl)) {
		dev_err(spi->pltdev, "Failed to get pinctrl\n");
		return PTR_ERR(spi->pinctrl);
	}

	/* Lookup the default pin state */
	spi->pctrl_state = pinctrl_lookup_state(spi->pinctrl, "default");
	if (IS_ERR(spi->pctrl_state)) {
		dev_err(spi->pltdev, "Failed to lookup default pinctrl state\n");
		return PTR_ERR(spi->pctrl_state);
	}

	/* Select the default pin state */
	ret = pinctrl_select_state(spi->pinctrl, spi->pctrl_state);
	if (ret) {
		dev_err(spi->pltdev, "Failed to select default pinctrl state\n");
		return ret;
	}

	spi->gpio_line = pinctrl_lookup_state(spi->pinctrl, "gpio_line");
	if (IS_ERR(spi->gpio_line)) {
		dev_err(spi->pltdev, "Failed to lookup gpio_line pinctrl state\n");
		return PTR_ERR(spi->gpio_line);
	}

	ret = pinctrl_select_state(spi->pinctrl, spi->gpio_line);
	if (ret) {
		dev_err(spi->pltdev, "Failed to select gpio_line pinctrl state\n");
		return ret;
	}

	spi->spim1_cs_line = pinctrl_lookup_state(spi->pinctrl, "spim1_cs_line");
	if (IS_ERR(spi->spim1_cs_line)) {
		dev_err(spi->pltdev, "Failed to lookup spim1_cs_line pinctrl state\n");
		return PTR_ERR(spi->spim1_cs_line);
	}

	return 0;
}

/* Declaration of the wrapper function exported by GPIO Driver */
extern void __iomem *get_ioremapped_gpio_base(void);

static int atcspi200_spi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct atcspi200_spi *spi;
	int ret;
	u32 num_cs = NSPI_MAX_CS_NUM;

	master = spi_alloc_master(&pdev->dev, sizeof(struct atcspi200_spi));
	if (!master) {
		dev_err(&pdev->dev, "spi_allooc_master error\n");
		return -ENOMEM;
	}

	spi = spi_master_get_devdata(master);
	spi->dev = &master->dev;
	platform_set_drvdata(pdev, master);

	/* get base addr  */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	spi->regs = devm_ioremap_resource(&pdev->dev, res);
	
	/* Regs sanity check */
	if (IS_ERR(spi->regs)) {
		dev_err(&pdev->dev, "Unable to map IO resources\n");
		ret = PTR_ERR(spi->regs);
		goto put_master;
	}

	/* init mutex lock for transfer */
	mutex_init(&spi->transfer_lock);

	spi->pltdev = &pdev->dev;
	/* Get GPIO46 for HW workaround */
	spi->spi_hw_workaround = of_property_read_bool(pdev->dev.of_node, "spi-hw-workaround");
	if (spi->spi_hw_workaround) {
		spi->data_merge = 1; /* data merge mode is needed for this workaround */
		atcspi200_spi_set_gpio_padmux(spi);
	} else {
		spi->data_merge = 0;
	}

	spi->timeout = SPI_TIMEOUT;
	spi->max_transfer_length = MAX_TRANSFER_LEN;
	spi->mtiming = atcspi200_spi_read(spi, Timing);

	/* get clock */
	spi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(spi->clk)) {
		dev_err(&pdev->dev, "Unable to find bus clock\n");
		ret = PTR_ERR(spi->clk);
		goto put_master;
	}
	
	/* Optional parameters */
	ret = of_property_read_u32(pdev->dev.of_node, "spi-max-frequency", &master->max_speed_hz);
	if (ret) {
		master->max_speed_hz = SPI_MAX_HZ; /* 50MHz */
	}
	spi->freq = master->max_speed_hz;

	if (of_property_read_bool(pdev->dev.of_node, "spi-cpha"))
		spi->mode |= SPI_CPHA;
	if (of_property_read_bool(pdev->dev.of_node, "spi-cpol"))
		spi->mode |= SPI_CPOL;
	
	if (of_property_read_bool(pdev->dev.of_node, "read_cs_toggle_quirk"))
		spi->quirks |= READ_CS_TOGGLE_QUIRK;
	if (of_property_read_bool(pdev->dev.of_node, "nor_flash_quirk"))
		spi->quirks |= NOR_FLASH_QUIRK;
	if (of_property_read_bool(pdev->dev.of_node, "rfi_dev_quirk"))
		spi->quirks |= RFI_DEVICE_QUIRK;

	if (spi->quirks & NOR_FLASH_QUIRK)
		spi->timeout = SPI_NOR_TIMEOUT;

	/* Stash away Raptor2 RF devs */
	if (spi->quirks & RFI_DEVICE_QUIRK)
		raptor2_rfdevs[nrfdevs++] = spi;

	/* Spin up the bus clock before hitting registers */
	ret = clk_prepare_enable(spi->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable bus clock\n");
		goto put_master;
	}
	spi->clk_rate = clk_get_rate(spi->clk);
	if (!spi->clk_rate) {
		dev_err(&pdev->dev, "clk rate = 0\n");
		ret = -EINVAL;
		goto put_master;
	}
	/* probe the number of CS lines */
	ret = of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs);
	if (ret) {
		dev_err(&pdev->dev, "could not find num-cs\n");
		ret = -ENXIO;
		goto put_master;
	}
	if (num_cs > NSPI_MAX_CS_NUM) {
		dev_warn(&pdev->dev, "unsupported number of cs (%i), reducing to 1\n", num_cs);
		num_cs = NSPI_MAX_CS_NUM;
	}

        spi->reg_shift = 1;

        if (of_property_read_u32(pdev->dev.of_node, "reg-shift", &spi->reg_shift) != 0)
                dev_dbg(&pdev->dev, "Unable to find reg-shift\n");
	
	/* Define our master */
	master->bus_num = pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->dev.of_node = pdev->dev.of_node;
	master->num_chipselect = num_cs;
	master->transfer_one_message = atcspi200_spi_transfer_one_message;

	/* Configure the SPI master hardware */
	atcspi200_spi_setup(spi);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		goto put_master;
	}
	dev_info(&pdev->dev, "Andes SPI controller driver probed successfully.\n");

	return 0;

put_master:
	spi_master_put(master);

	return ret;
}

static const struct of_device_id atcspi200_spi_of_match[] = {
	{ .compatible = "andestech,atcspi200", },
	{}
};
MODULE_DEVICE_TABLE(of, atcspi200_spi_of_match);

static struct platform_driver atcspi200_spi_driver = {
	.probe = atcspi200_spi_probe,
	.driver = {
		.name = SPI_NAME,
		.of_match_table = atcspi200_spi_of_match,
	},
};
module_platform_driver(atcspi200_spi_driver);

MODULE_AUTHOR("Nylon Chen. <nylon7@andestech.com>");
MODULE_AUTHOR("Nilesh Waghmare <nilesh.waghmare@edgeq.io>");
MODULE_DESCRIPTION("Andes SPI driver");
MODULE_LICENSE("GPL");
