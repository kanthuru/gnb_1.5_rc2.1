// SPDX-License-Identifier: GPL-2.0+
/*
 * Andestech ATCSPI200 SPI controller driver.
 *
 * Copyright 2017 Andes Technology, Inc.
 * Author: Rick Chen (rick@andestech.com)
 */

#include <common.h>
#include <clk.h>
#include <log.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>
#include <dm.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define MAX_TRANSFER_LEN	512
#define CHUNK_SIZE		(ns->data_merge ? 4 : 1)
#define SPI_TIMEOUT		0x100000
#define SPI0_BUS		0
#define SPI1_BUS		1
#define SPI0_BASE		0xf0b00000
#define SPI1_BASE		0xf0f00000
#define NSPI_MAX_CS_NUM		1
#define CMD_BUF_SIZE		512

/*Andes SPI Controller Registers*/
#define SPI_ID_REV		0x00
#define SPI_XFER_FORMAT		0x10
#define DATA_LENGTH(x)		((x - 1) << 8)
#define SPI_IO_CTRL		0x14
#define SPI_XFER_CTRL		0x20
#define  ADDR_EN		29
#define  CMD_EN			30
#define  TRAMODE_OFFSET		24
#define  TRAMODE_MASK		(0x0F << TRAMODE_OFFSET)
#define  TRAMODE_WR_SYNC	(0 << TRAMODE_OFFSET)
#define  TRAMODE_WO		BIT(TRAMODE_OFFSET)
#define  TRAMODE_RO		(2 << TRAMODE_OFFSET)
#define  TRAMODE_WR		(3 << TRAMODE_OFFSET)
#define  TRAMODE_RW		(4 << TRAMODE_OFFSET)
#define  TRAMODE_WDR		(5 << TRAMODE_OFFSET)
#define  TRAMODE_RDW		(6 << TRAMODE_OFFSET)
#define  TRAMODE_NONE		(7 << TRAMODE_OFFSET)
#define  TRAMODE_DW		(8 << TRAMODE_OFFSET)
#define  TRAMODE_DR		(9 << TRAMODE_OFFSET)
#define  TRAMODE_ADDREN		BIT(ADDR_EN)
#define  TRAMODE_CMDEN		BIT(CMD_EN)
#define DATA_PHASE_FORMAT	22 /* SPI Data Phase Format. Bits 23:22 */
#define REGULAR_IO_MODE_MASK	(0x0 << DATA_PHASE_FORMAT)
#define QUAD_IO_MODE_MASK	(0x2 << DATA_PHASE_FORMAT)
#define  WCNT_OFFSET		12
#define  WCNT_MASK		(0x1FF << WCNT_OFFSET)
#define  RCNT_OFFSET		0
#define  RCNT_MASK		(0x1FF << RCNT_OFFSET)
#define  ADDR_FMT		28
#define SPI_COMMAND		0x24
#define SPI_ADDR		0x28
#define SPI_DATA		0x2C
#define SPI_CTRL		0x30
#define  TXFTH_OFFSET		16
#define  RXFTH_OFFSET		8
#define  TXDMAEN		BIT(4)
#define  RXDMAEN		BIT(3)
#define  TXFRST			BIT(2)
#define  RXFRST			BIT(1)
#define  SPIRST			BIT(0)
#define SPI_STATUS		0x34
#define  TXFFL			BIT(23)
#define  TXEPTY			BIT(22)
#define  TXFVE_MASK		(0x1F << 16)
#define  RXFEM			BIT(14)
#define  RXFVE_OFFSET		(8)
#define  RXFVE_MASK		(0x1F << RXFVE_OFFSET)
#define  SPIBSY			BIT(0)
#define SPI_INTERRUPT_EN	0x38
#define SPI_INTERRUPT_STATUS	0x3C
#define SPI_INTERRUPT_TIMING	0x40
#define  SCLK_DIV_MASK		0xFF
#define SPI_MEMCTRL		0x50
#define SPI_SLAVE_STATUS	0x60
#define SPI_SLAVE_DATA_CNT	0x64
#define SPI_CONFIG		0x7C
#define DATA_MERGE		BIT(7)
#define ADDR_3BYTE_MODE		0x2
#define ADDR_4BYTE_MODE		0x3

/* SPI flags */
#define SPI_WRITE_DELAY_QUIRK	BIT(0)

/*
 * Use spi-use-cmd-addr-reg in DT to use command & address registers
 * to send commands & addresses respectively, instead of using only data
 * register to achieve the same. It is required to use this flag to operate SPI
 * slaves in any mode other than regular SPI mode.
 */
#define SPI_USE_CMD_ADDR_REG	BIT(1)

struct nds_spi_slave {
	volatile struct atcspi200_spi_regs *regs;
	int		to;
	unsigned int	freq;
	ulong		clock;
	unsigned int	mode;
	u8 		num_cs;
	unsigned int	mtiming;
	size_t		cmd_len;
	union {
		u8	cmd_buf[CMD_BUF_SIZE];
		u32	cmd_buf32[CMD_BUF_SIZE / 4];
	} cmd_buff_u;
	size_t		data_len;
	size_t		tran_len;
	u8		*din;
	u8		*dout;
	unsigned int    max_transfer_length;
	u32		reg_shift;      /* reg offset shift */
	void		*ioaddr;
	u32 data_merge;
	u32		addr_mode; 	/* 4 byte mode or 3 byte mode */
	u32		flags;	/* To enable features & quirks */
};

static void atcspi200_spi_writel(struct nds_spi_slave *ns, u32 val, int reg)
{
	writel(val, ns->ioaddr + (reg << (ns->reg_shift - 1)));
	if (ns->flags & SPI_WRITE_DELAY_QUIRK)
		udelay(20);
}

static inline u32 atcspi200_spi_readl(struct nds_spi_slave *ns, int reg)
{
	return readl(ns->ioaddr + (reg << (ns->reg_shift - 1)));
}

static int __atcspi200_spi_set_speed(struct nds_spi_slave *ns)
{
	u32 tm;
	u8 div;
	tm = atcspi200_spi_readl(ns, SPI_INTERRUPT_TIMING);
	tm &= ~SCLK_DIV_MASK;

	if(ns->freq >= ns->clock)
		div =0xff;
	else{
		for (div = 0; div < 0xff; div++) {
			if (ns->freq >= ns->clock / (2 * (div + 1)))
				break;
		}
	}

	tm |= div;
	atcspi200_spi_writel(ns, tm, SPI_INTERRUPT_TIMING);

	return 0;

}

static int __atcspi200_spi_claim_bus(struct nds_spi_slave *ns)
{
		unsigned int format=0;
		unsigned int rdata;

		if (ns->data_merge)
			format = DATA_MERGE;

		rdata = atcspi200_spi_readl(ns, SPI_CTRL);
		atcspi200_spi_writel(ns, rdata | TXFRST | RXFRST | SPIRST, SPI_CTRL);
		while((atcspi200_spi_readl(ns, SPI_CTRL) & (TXFRST | RXFRST | SPIRST)) && (ns->to--))
			if(!ns->to)
				return -EINVAL;

		ns->cmd_len = 0;
		format |= ((ns->addr_mode << TXFTH_OFFSET) | ns->mode | DATA_LENGTH(8));
		atcspi200_spi_writel(ns, format, SPI_XFER_FORMAT);
		__atcspi200_spi_set_speed(ns);

		return 0;
}

static int __atcspi200_spi_release_bus(struct nds_spi_slave *ns)
{
	/* do nothing */
	return 0;
}

/**
 * Start a SPI transaction. Use only data register.
 */
static int __atcspi200_spi_start_data_reg_only(struct nds_spi_slave *ns)
{
	int i,olen=0, clen = 0;
	int tc = atcspi200_spi_readl(ns, SPI_XFER_CTRL);

	tc &= ~(WCNT_MASK | RCNT_MASK | TRAMODE_MASK | TRAMODE_ADDREN | TRAMODE_CMDEN);
	if ((ns->din)&&(ns->cmd_len))
		tc |= TRAMODE_WR;
	else if (ns->din)
		tc |= TRAMODE_RO;
	else
		tc |= TRAMODE_WO;

	if(ns->dout)
		olen = ns->tran_len;

	clen = ns->cmd_len-1;
	tc |= (clen+olen) << WCNT_OFFSET;

	if(ns->din)
		tc |= (ns->tran_len-1) << RCNT_OFFSET;

	__atcspi200_spi_set_speed(ns);
	atcspi200_spi_writel(ns, tc, SPI_XFER_CTRL);
	atcspi200_spi_writel(ns, 1, SPI_COMMAND);

	for (i = 0; i < DIV_ROUND_UP(ns->cmd_len, CHUNK_SIZE); i++) {
		if((ns->addr_mode == ADDR_3BYTE_MODE) && (i == 4))
			break;

		atcspi200_spi_writel(ns, ns->data_merge ? ns->cmd_buff_u.cmd_buf32[i] :
			ns->cmd_buff_u.cmd_buf[i], SPI_DATA);

		if (ns->data_merge)
			udelay(10);
	}

	return 0;
}

/**
 * Just to start a SPI start transaction, no data is being sent.
 */
static int __atcspi200_spi_start(struct nds_spi_slave *ns)
{
	/*
	 * Command buffer -> [ opcode | addr | dummy ]
	 * opcode is the 0th byte, address is 3/4 bytes long or 0 if no address
	 * is provided, dummy is 0 or more bytes
	 */

	u8 *cmd_buf = ns->cmd_buff_u.cmd_buf;
	u8 opcode = cmd_buf[0];
	uint addr = 0;
	uint tc = TRAMODE_CMDEN;

	if (ns->cmd_len > 1)
		tc |= TRAMODE_ADDREN;

	if ((ns->din) && !(ns->dout))
		tc |= TRAMODE_RO;
	else if (!(ns->din) && (ns->dout))
		tc |= TRAMODE_WO;
	else if (!(ns->din) && !(ns->dout))
		tc |= TRAMODE_NONE;
	
	tc |= REGULAR_IO_MODE_MASK;

	/*
	 * For transferring/recieving x bytes of data, x-1 has to be written to 
	 * WCNT offset or RCNT offset respectively
	 */

	/* Assumption: Read & Write won't be requested simultaneously */
	if (ns->dout)
		tc |= (ns->tran_len-1) << WCNT_OFFSET;
	else if (ns->din)
		tc |= (ns->tran_len-1) << RCNT_OFFSET;

	__atcspi200_spi_set_speed(ns);

	atcspi200_spi_writel(ns, tc, SPI_XFER_CTRL);

	if (ns->cmd_len > 1) {
		if (ns->addr_mode == ADDR_3BYTE_MODE)
			addr = ((cmd_buf[1] << 16) | (cmd_buf[2] << 8) |
				(cmd_buf[3]));
		else
			addr = ((cmd_buf[1] << 24) | (cmd_buf[2] << 16) |
				(cmd_buf[3] << 8) | (cmd_buf[4]));

		atcspi200_spi_writel(ns, addr, SPI_ADDR);	
	}

	/*
	 * Writing operations on command register will trigger SPI transfers.
	 */
	atcspi200_spi_writel(ns, opcode, SPI_COMMAND);

	return 0;
}

static int __atcspi200_spi_stop(struct nds_spi_slave *ns)
{
	atcspi200_spi_writel(ns, ns->mtiming, SPI_INTERRUPT_TIMING);
	while ((atcspi200_spi_readl(ns, SPI_STATUS) & SPIBSY)&&(ns->to--))
		if (!ns->to)
			return -EINVAL;

	return 0;
}

static void __nspi_espi_tx(struct nds_spi_slave *ns, const void *dout)
{
	if (ns->data_merge)
		atcspi200_spi_writel(ns, *(u32 *)dout, SPI_DATA);
	else
		atcspi200_spi_writel(ns, *(u8 *)dout, SPI_DATA);
}

static int __nspi_espi_rx(struct nds_spi_slave *ns, void *din, unsigned int bytes)
{
	if (ns->data_merge)
		*(u32 *)din = atcspi200_spi_readl(ns, SPI_DATA);
	else
		*(u8 *)din = atcspi200_spi_readl(ns, SPI_DATA);
	return bytes;
}


static int __atcspi200_spi_xfer(struct nds_spi_slave *ns,
		unsigned int bitlen,  const void *data_out, void *data_in,
		unsigned long flags)
{
		unsigned int event, rx_bytes;
		const void *dout = NULL;
		void *din = NULL;
		int num_blks, num_chunks, max_tran_len, tran_len;
		int num_bytes;
		u8 *cmd_buf = ns->cmd_buff_u.cmd_buf;
		size_t cmd_len = ns->cmd_len;
		unsigned long data_len = bitlen / 8;
		int rf_cnt;
		int ret = 0, timeout = 0;
		u32 addr = 0;
		u32 rdata;

		max_tran_len = ns->max_transfer_length;
		switch (flags) {
		case SPI_XFER_BEGIN:
			/* BEGIN flag: opcode + address + dummy cycles */
			cmd_len = ns->cmd_len = data_len;
			memset(cmd_buf, 0x0, CMD_BUF_SIZE);
			memcpy(cmd_buf, data_out, cmd_len);
			return 0;

		case SPI_XFER_END:
			/* END flag: rx or tx data path */
			if (bitlen == 0) {
				return 0;
			}
			ns->data_len = data_len;
			ns->din = (u8 *)data_in;
			ns->dout = (u8 *)data_out;
			break;

		case SPI_XFER_ONCE:
			/* ONCE = BEGIN & END flag together, indicates no rx/tx data */
			ns->data_len = 0;
			ns->din = 0;
			ns->dout = 0;
			cmd_len = ns->cmd_len = data_len;
			memset(cmd_buf, 0x0, CMD_BUF_SIZE);
			memcpy(cmd_buf, data_out, cmd_len);
			data_out = 0;
			data_len = 0;
			if (ns->flags & SPI_USE_CMD_ADDR_REG)
				__atcspi200_spi_start(ns);
			else
				__atcspi200_spi_start_data_reg_only(ns);
			break;
		}

		num_chunks = DIV_ROUND_UP(data_len, max_tran_len);
		din = data_in;
		dout = data_out;
		while (num_chunks--) {
			tran_len = min((size_t)data_len, (size_t)max_tran_len);
			ns->tran_len = tran_len;
			num_blks = DIV_ROUND_UP(tran_len , CHUNK_SIZE);
			num_bytes = (tran_len) % CHUNK_SIZE;
			timeout = SPI_TIMEOUT;
			if (num_bytes == 0)
				num_bytes = CHUNK_SIZE;

			if (ns->flags & SPI_USE_CMD_ADDR_REG)
				__atcspi200_spi_start(ns);
			else
				__atcspi200_spi_start_data_reg_only(ns);

			while (num_blks && (timeout--)) {
				rdata = atcspi200_spi_readl(ns, SPI_STATUS);
				event = in_le32(&rdata);
				if ((event & TXEPTY) && (data_out)) {
					__nspi_espi_tx(ns, dout);
					num_blks -= 1;
					dout += CHUNK_SIZE;
				}

				if ((event & RXFVE_MASK) && (data_in)) {
					rf_cnt = ((event & RXFVE_MASK)>> RXFVE_OFFSET);
					if (rf_cnt >= 1)
						rx_bytes = CHUNK_SIZE;
					else if (num_blks == 1 && rf_cnt == num_bytes)
						rx_bytes = num_bytes;
					else
						continue;

					if (__nspi_espi_rx(ns, din, rx_bytes) == rx_bytes) {
						num_blks -= 1;
						din = (unsigned char *)din + rx_bytes;
					}
				}
				if (!timeout) {
					debug("spi_xfer: %s() timeout\n", __func__);
					break;
				}
			}

			data_len -= tran_len;
			if(data_len)
			{
				if(ns->addr_mode == ADDR_4BYTE_MODE)
				{
					addr = ((ns->cmd_buff_u.cmd_buf[1] << 24) |
						(ns->cmd_buff_u.cmd_buf[2] << 16) |
						(ns->cmd_buff_u.cmd_buf[3] << 8) |
						(ns->cmd_buff_u.cmd_buf[4]));
					addr += tran_len;
					ns->cmd_buff_u.cmd_buf[1] = ((addr >> 24) & 0xff);
					ns->cmd_buff_u.cmd_buf[2] = ((addr >> 16) & 0xff);
					ns->cmd_buff_u.cmd_buf[3] = ((addr >> 8) & 0xff);
					ns->cmd_buff_u.cmd_buf[4] = ((addr) & 0xff);
					ns->data_len = data_len;
				}
				else	/* ADDR_3BYTE_MODE */
				{
					addr = ((ns->cmd_buff_u.cmd_buf[1] << 16) |
						(ns->cmd_buff_u.cmd_buf[2] << 8) |
						(ns->cmd_buff_u.cmd_buf[3]));
					addr += tran_len;
					ns->cmd_buff_u.cmd_buf[1] = ((addr >> 16) & 0xff);
					ns->cmd_buff_u.cmd_buf[2] = ((addr >> 8) & 0xff);
					ns->cmd_buff_u.cmd_buf[3] = ((addr) & 0xff);
					ns->data_len = data_len;
				}

			}
			ret = __atcspi200_spi_stop(ns);
		}
		ret = __atcspi200_spi_stop(ns);

		return ret;
}

static int atcspi200_spi_set_speed(struct udevice *bus, uint max_hz)
{
	struct nds_spi_slave *ns = dev_get_priv(bus);

	debug("%s speed %u\n", __func__, max_hz);

	ns->freq = max_hz;
	__atcspi200_spi_set_speed(ns);

	return 0;
}

static int atcspi200_spi_set_mode(struct udevice *bus, uint mode)
{
	struct nds_spi_slave *ns = dev_get_priv(bus);

	debug("%s mode %u\n", __func__, mode);
	ns->mode = mode;

	return 0;
}

static int atcspi200_spi_claim_bus(struct udevice *dev)
{
	struct dm_spi_slave_platdata *slave_plat =
		dev_get_parent_platdata(dev);
	struct udevice *bus = dev->parent;
	struct nds_spi_slave *ns = dev_get_priv(bus);

	if (slave_plat->cs >= ns->num_cs) {
		printf("Invalid SPI chipselect\n");
		return -EINVAL;
	}

	return __atcspi200_spi_claim_bus(ns);
}

static int atcspi200_spi_release_bus(struct udevice *dev)
{
	struct nds_spi_slave *ns = dev_get_priv(dev->parent);

	return __atcspi200_spi_release_bus(ns);
}

static int atcspi200_spi_xfer(struct udevice *dev, unsigned int bitlen,
			    const void *dout, void *din,
			    unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct nds_spi_slave *ns = dev_get_priv(bus);

	return __atcspi200_spi_xfer(ns, bitlen, dout, din, flags);
}

static int atcspi200_spi_get_clk(struct udevice *bus)
{
	struct nds_spi_slave *ns = dev_get_priv(bus);
	struct clk clk;
	ulong clk_rate;
	int ret;

	ret = clk_get_by_index(bus, 0, &clk);
	if (ret)
		return -EINVAL;

	clk_rate = clk_get_rate(&clk);
	if (!clk_rate)
		return -EINVAL;

	ns->clock = clk_rate;
	clk_free(&clk);

	return 0;
}

static int atcspi200_spi_probe(struct udevice *bus)
{
	struct nds_spi_slave *ns = dev_get_priv(bus);

	ns->to = SPI_TIMEOUT;
	ns->max_transfer_length = MAX_TRANSFER_LEN;
	ns->mtiming = atcspi200_spi_readl(ns, SPI_INTERRUPT_TIMING);;
	atcspi200_spi_get_clk(bus);

	return 0;
}

static int atcspi200_ofdata_to_platadata(struct udevice *bus)
{
	struct nds_spi_slave *ns = dev_get_priv(bus);
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(bus);

	ns->reg_shift = 1;
	ns->reg_shift = dev_read_u32_default(bus, "reg-shift",
			ns->reg_shift);
	if (!ns->reg_shift)
		debug("%s: Unable to find reg-shift\n", __func__);

	ns->ioaddr = (void *)devfdt_get_addr(bus);
	if (IS_ERR(ns->ioaddr))
		return PTR_ERR(ns->ioaddr);

	ns->num_cs = fdtdec_get_int(blob, node, "num-cs", 4);
	ns->data_merge = dev_read_bool(bus, "data-merge");
	if (!ns->data_merge)
		debug("%s: Unable to find data_merge\n", __func__);

	if (!dev_read_bool(bus, "addr_3byte")){
		debug("%s: Unable to find address length\n", __func__);
		ns->addr_mode = ADDR_4BYTE_MODE;
	}
	else
		ns->addr_mode = ADDR_3BYTE_MODE;

	debug("Address mode = %x\n", ns->addr_mode);

	if (!dev_read_bool(bus, "spi-write-delay-quirk")) {
		debug("%s: Unable to find spi-write-delay-quirk\n", __func__);
	} else {
		ns->flags |= SPI_WRITE_DELAY_QUIRK;
		debug("%s: found spi-write-delay-quirk %d \n", __func__, ns->flags);
	}

	if (!dev_read_bool(bus, "spi-use-cmd-addr-reg")) {
		debug("%s: Unable to find spi-use-cmd-addr-reg\n", __func__);
	} else {
		ns->flags |= SPI_USE_CMD_ADDR_REG;
		debug("%s: Found spi-use-cmd-addr-reg\n", __func__);
	}

	return 0;
}

static const struct dm_spi_ops atcspi200_spi_ops = {
	.claim_bus	= atcspi200_spi_claim_bus,
	.release_bus	= atcspi200_spi_release_bus,
	.xfer		= atcspi200_spi_xfer,
	.set_speed	= atcspi200_spi_set_speed,
	.set_mode	= atcspi200_spi_set_mode,
};

static const struct udevice_id atcspi200_spi_ids[] = {
	{ .compatible = "andestech,atcspi200" },
	{ }
};

U_BOOT_DRIVER(atcspi200_spi) = {
	.name = "atcspi200_spi",
	.id = UCLASS_SPI,
	.of_match = atcspi200_spi_ids,
	.ops = &atcspi200_spi_ops,
	.ofdata_to_platdata = atcspi200_ofdata_to_platadata,
	.priv_auto_alloc_size = sizeof(struct nds_spi_slave),
	.probe = atcspi200_spi_probe,
};
