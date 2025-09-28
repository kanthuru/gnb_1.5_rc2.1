
#include <stdint.h>
#include <spi_driver.h>
#include <arch_helpers.h>
#include <lib/mmio.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>

#define BSS_RAM_START 0x80000000
#define E1_RAM_START 0x800000000ULL
#define E1_TO_BSS(addr) (uint32_t)((addr - E1_RAM_START) + BSS_RAM_START)

/* SPI Control Register */
#define ATCSPI200_CTRL_TXFIFORST_MASK           (1<<2)
#define ATCSPI200_CTRL_RXFIFORST_MASK           (1<<1)
#define ATCSPI200_CTRL_SPIRST_MASK              (1<<0)

/*
 * addr variable in below functions is type casted to (long) and then casted to INT PTR.
 * This is because of the strict compilatiotion checks in ATF.
 * In order to avoid [-Werror=int-to-pointer-cast] one extra conversion performed.
 */
void cpu_wr_w(volatile unsigned long addr, volatile unsigned int wdata)
{
	*((volatile unsigned int *)addr) = wdata;
	dsb();
}

volatile unsigned int cpu_rd_w (volatile unsigned long addr)
{
	volatile unsigned int data;
	data = *(volatile unsigned int *)addr;
	dsb();
	return data;
}

const struct flash_info flash_device [] =
/* Device Name, JEDEC ID, Size */
{
        FLASH_ID("issi is25wp256d", 0x0019709d, 0x2000000),
        FLASH_ID("win w25q128fv", 0x001860ef, 0x1000000),
        FLASH_ID(NULL, 0, 0)
};

static uint8_t mode_4byte_enable = 0;

uint32_t read_jedec(void)
{
	uint32_t data;

	// Reset SPI, RX and TX FIFO
	cpu_wr_w(SPI_CNTRL_REG, 0x7);

	// Setting DataLen to 1 Byte + Data Merge
	cpu_wr_w(SPI_TXFR_FMT_REG, 0x20780);

	// TCTRL Register
	data = 0x42000002;
	cpu_wr_w(SPI_TXFR_CNTRL_REG, data);

	// SPI Command for JEDEC ID
	cpu_wr_w(SPI_CMD_REG, 0x9F);
	while((cpu_rd_w(SPI_CNTRL_REG) & (1<<14)) != 0);
	data = cpu_rd_w(SPI_DATA_REG);
	data &= 0xFFFFFF;
	INFO("Found Flash %x\n", data);

	return data;
}

uint32_t find_flash_mode(void)
{
	uint32_t jedec_id = 0x0;
	int i = 0;

	jedec_id = read_jedec();

	while(1)
	{
		if(jedec_id == flash_device[i].device_id)
			break;

		if(flash_device[i].device_id == 0)
			break;

		i++;
	}
	return ((flash_device[i].size_in_bytes==0x1000000)?3:4);
}

void reset_spi(void)
{
	unsigned int spib_ctrl = cpu_rd_w(SPI_CNTRL_REG);
	spib_ctrl |= (ATCSPI200_CTRL_TXFIFORST_MASK | \
			ATCSPI200_CTRL_RXFIFORST_MASK|ATCSPI200_CTRL_SPIRST_MASK);
	cpu_wr_w(SPI_CNTRL_REG, spib_ctrl);
	mdelay(1);
}

void spi_init(void)
{
	uint32_t data;

	if(find_flash_mode() != 3)
	{
		//--------------SPI CFG-----------//
		// SPI Command for enabling ADDR4_MODE
		// enabling cmd_en for B7 to the control reg
		data = cpu_rd_w(SPI_TXFR_CNTRL_REG);
		data |= 0x47000000;
		cpu_wr_w(SPI_TXFR_CNTRL_REG, data);

		// SPI Command for
		// cmd reg: 0x24
		// cmd : b7 - 4 byte address
		cpu_wr_w(SPI_CMD_REG, 0xb7);
		mode_4byte_enable = 1;
	}

	//SPI INTERRUPT enable control Register - 0x38
	cpu_wr_w(SPI_INTR_ENABLE_REG, 0x00000010);

	//SPI control Register - 0x30
	// 2 : TXFIFORST
	// 1 : RXFIFORST
	// 0 : SPIRST
	//clear TXFIFO,RXFIFO Buffers
	data = cpu_rd_w(SPI_CNTRL_REG);
	data |= 0x7;
	cpu_wr_w(SPI_CNTRL_REG, data);

	/* SPI interface timing Register - 0x40
	 * SCLK period =((SCLK_DIV+1)*2)((Period of SPI clock source)
	 */
#if A0_BUILD
	/* SPI source clk=25ns
	 * SCLK=800ns
	 * SCLK_DIV=0xF
	 */
	cpu_wr_w(SPI_TIMING_REG, 0x0000000F);
#elif B0_BUILD
	/* SPI source clk=16.27ns
	 * SCLK=800ns
	 * SCLK_DIV=0x1
	 * SPI clock source 61440000
	 * 15.36MHz clock to NOR
	 */
	cpu_wr_w(SPI_TIMING_REG, 0x00000001);
#endif
	//--------------SPI CFG-----------//

}

/*
 * spi_read(): Read xfercnt bytes from the baseAddr into dstAddr.
 *             The addresses to the SPI DMA Master is at uint32 boundary
 */

// xfercnt is in bytes.
// The addresses to the SPI DMA Master is at uint32 boundary
int spi_read(uint32_t *dstAddr, uint32_t *baseAddr, uint32_t xfercnt)
{
	uint32_t bytelen;
	uint32_t loopcnt;
	uint32_t blksize, waddr;
	uint32_t rdata;
	uint32_t rdata1;
	uint32_t wdata;
	uint32_t rem;
	uint32_t *srcAddr;
	uint32_t wordlen;
	uint32_t bss_dstAddr = E1_TO_BSS((unsigned long long)dstAddr);

	/* Clear interrupts */
	cpu_wr_w(SPI_INTR_STATUS_REG, 0x0000003f);
	cpu_wr_w(SPI_DMA_INTR_STATUS_REG, 0x00ffffff);

	if(mode_4byte_enable == 0)
		reset_spi();

	srcAddr = baseAddr;
	blksize = 0x200;
	rem = xfercnt % blksize;
	if (rem == 0)
		loopcnt = xfercnt/blksize ; // xfercnt mod maxbytes
	else
		loopcnt = (xfercnt/blksize) + 1;

	for (uint32_t i = 0; i < loopcnt; i++)
	{
		// Loop Starts Here //---------
		// Channel 1
		// transSize = 0x44 + 0* 20
		cpu_wr_w(SPI_DMA_CH_TXFRSIZE_REG(0), (blksize/4)); // transfer size in words

		// srcaddl = 0x48 + 0* 20
		cpu_wr_w(SPI_DMA_CH_SRCADDR_REG(0), SPI_DATA_REG_BSS); // constant as Data reg
		// destaddl = 0x50 + 0* 20
		waddr = (bss_dstAddr)+(blksize * i);

		cpu_wr_w(SPI_DMA_CH_DSTADDR_REG(0), waddr); // dest addr
		// ch2llptr = 0x58 + 0* 20
		// DMA Controller : Channel 0 Linked List pointer lower register
		cpu_wr_w(SPI_DMA_CH_LLPR_REG(0), 0x00000000);

		// DMA Controller : Channel 0 control register
		wdata = 0x204A8001;
		cpu_wr_w(SPI_DMA_CH_CNTRL_REG(0), wdata);

		//SPI Configuration Register - 0x7C
		//[7:4] : Depth of TXFIFO
		//[3:0] : Depth of RXFIFO
		rdata = cpu_rd_w(SPI_CFG_REG);
		do {
			//SPI Status Register - 0x34
			// 0 : SPIActive
			rdata = cpu_rd_w(SPI_STATUS_REG);
			rdata1 = rdata & 0x00000001;
		} while (rdata1 == 1);

		//SPI TRANSFER format Register - 0x10
		// 17:16 : AddrLen (4 byte address)
		// [12:8]: DataLen (data length-1)
		// 7     : DataMerge
		if(mode_4byte_enable == 0)
			cpu_wr_w(SPI_TXFR_FMT_REG, 0x00020780);
		else
			cpu_wr_w(SPI_TXFR_FMT_REG, 0x00030780);

		//cpu_wr_w(SPI_TIMING_REG, 0x4);

		if (i < loopcnt - 1) {
			if (xfercnt >= blksize) {
				bytelen = blksize - 1;
				cpu_wr_w(SPI_DMA_CH_TXFRSIZE_REG(0), (blksize/4)); // transfer size in words
			} else {
				bytelen = xfercnt - 1;
				wordlen = (bytelen + 1) / 4 ;
				if ( ( (bytelen + 1 ) % 4) != 0 )
					wordlen = wordlen + 1;
				cpu_wr_w(SPI_DMA_CH_TXFRSIZE_REG(0), wordlen); // transfer size in words
			}

		} else {
			if (rem == 0) {
				bytelen = blksize - 1;
				cpu_wr_w(SPI_DMA_CH_TXFRSIZE_REG(0), (blksize/4)); // transfer size in words
			} else {
				bytelen = rem - 1;
				wordlen = (bytelen + 1) / 4 ;
				if ( ( (bytelen + 1 ) % 4) != 0 )
					wordlen = wordlen + 1;
				cpu_wr_w(SPI_DMA_CH_TXFRSIZE_REG(0), wordlen); // transfer size in words
			}
		}
		//SPI TRANSFER control Register - 0x20
		// 30 : CmdEn
		// 29 : AddrLen
		// 28 : AddrFmt
		// [27:24]: Transmode (read only 02)
		// [23:22]: DualQuad
		// [8:0]: RdTranCnt (total transmission count - 1)
		wdata = 0x62000000 | bytelen; // (0x62000000 + 511);
		cpu_wr_w(SPI_TXFR_CNTRL_REG, wdata); //Readonly without dummy


		//SPI control Register - 0x30
		// [23:16] : TXTHRES
		// [15:8] : RXTHRES
		// 4 : TXDMAen
		// 3 : RXDMAen
		// 2 : TXFIFORST
		// 1 : RXFIFORST
		// 0 : SPIRST
		// Rx Mode enablement for receiving from nor flash
		cpu_wr_w(SPI_CNTRL_REG, 0x00080108);


		//SPI Address Register - 0x28
		waddr = (unsigned long)srcAddr + (blksize * i);
		cpu_wr_w(SPI_ADDR_REG, waddr);
/*
		printf("%s %d: srcAddr %x destAddr %llx bssdestAddr %x\n", \
			__func__,__LINE__, waddr, (unsigned long long)dstAddr, bss_dstAddr);
*/
		//SPI Command Register - 0x24
		// [7:0] : SPI command
		// 02 : Page-Program command
		// 03 : Read
		// 0b : fast_Read
		// 65 : rdar
		// 13 : 4-BYTE READ
		if(mode_4byte_enable == 0)
			cpu_wr_w(SPI_CMD_REG, 0x00000003);
		else
			cpu_wr_w(SPI_CMD_REG, 0x00000013);

		//SPI Interrupt status Register - 0x3C
		// 4 : EndInt
		do {
			rdata = cpu_rd_w(SPI_INTR_STATUS_REG);
			rdata1 = rdata & 0x00000010;
		} while (rdata1 == 0x0);
		cpu_wr_w(SPI_INTR_STATUS_REG, 0x00000010);

		do {
			// DMAC Intr Status Register - 0x30
			rdata = cpu_rd_w(SPI_DMA_INTR_STATUS_REG);
		} while (rdata == 0x00000000);
#if A0_BUILD
		mdelay(1);
#endif
		cpu_wr_w(SPI_DMA_INTR_STATUS_REG, rdata);

	} // for

	/* Since dma is non coherent need to invalidate the addr range */
	inv_dcache_range((uintptr_t)dstAddr, xfercnt);

	return xfercnt;
}
