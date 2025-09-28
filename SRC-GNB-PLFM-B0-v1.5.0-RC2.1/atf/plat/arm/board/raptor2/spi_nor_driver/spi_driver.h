#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

/*
 * SPI NOR flash driver module.
 * This implements bare minimum functionality of reading from flash.
 *
 * Note: This code is ported from ROM code.
 */
#if (ZEBU_BUILD || B0_BUILD)
#define   SPI_BASE 		(0x6E4D0000)
#else
#define   SPI_BASE 		(0x704D0000)
#endif
#define   SPI_BASE_BSS 		(0x4D0000)
#define   SPI_TXFR_FMT_REG 	(SPI_BASE + (2 * 0x10))
#define   SPI_TXFR_CNTRL_REG	(SPI_BASE + (2 * 0x20))
#define   SPI_CMD_REG 		(SPI_BASE + (2 * 0x24))
#define   SPI_ADDR_REG 		(SPI_BASE + (2 * 0x28))
#define   SPI_DATA_REG 		(SPI_BASE + (2 * 0x2C))
#define   SPI_DATA_REG_BSS 	(SPI_BASE_BSS + (2 * 0x2C))
#define   SPI_CNTRL_REG 	(SPI_BASE + (2 * 0x30))
#define   SPI_STATUS_REG 	(SPI_BASE + (2 * 0x34))
#define   SPI_INTR_ENABLE_REG 	(SPI_BASE + (2 * 0x38))
#define   SPI_INTR_STATUS_REG 	(SPI_BASE + (2 * 0x3C))
#define   SPI_TIMING_REG 	(SPI_BASE + (2 * 0x40))
#define   SPI_CFG_REG 		(SPI_BASE + (2 * 0x7C))

#if (ZEBU_BUILD || B0_BUILD)
#define   SPI_DMA_BASE 			(0x6E4C0000)
#else
#define   SPI_DMA_BASE 			(0x704C0000)
#endif
#define   SPI_DMA_INTR_STATUS_REG 	(SPI_DMA_BASE + (2 * 0x30))
#define   SPI_DMA_CH_CNTRL_REG(n) 	(SPI_DMA_BASE + (2 * (0x40 + (n * 0x20))))
#define   SPI_DMA_CH_TXFRSIZE_REG(n) 	(SPI_DMA_BASE + (2 * (0x44 + (n * 0x20))))
#define   SPI_DMA_CH_SRCADDR_REG(n) 	(SPI_DMA_BASE + (2 * (0x48 + (n * 0x20))))
#define   SPI_DMA_CH_DSTADDR_REG(n) 	(SPI_DMA_BASE + (2 * (0x50 + (n * 0x20))))
#define   SPI_DMA_CH_LLPR_REG(n) 	(SPI_DMA_BASE + (2 *(0x58 + (n * 0x20))))


#define   READ_BYTE(addr) 		(*(volatile uint8_t *)(addr))
#define   WRITE_BYTE(addr, val) 	((*(volatile uint8_t *)(addr)) = val)
#define   READ_WORD(addr) 		(*(volatile uint32_t *)(addr))
#define   WRITE_WORD(addr, val) 	((*(volatile uint32_t *)(addr)) = val)

struct flash_info {
        const char *name;
        uint32_t device_id;
        uint32_t size_in_bytes;
};

#define FLASH_ID(n, id, size)           \
{                                       \
        .name = n,                      \
        .device_id = id,                \
        .size_in_bytes = size,          \
}

/*
 *  spi_init(): Initialize SPI controller.
 *  spi_read(): Read from SPI NOR Flash, using DMA functionality.
 *       Note: Does not implement any interrupt functionality. Only reads the status register.
 *
 */
void  spi_init(void);
int   spi_read(uint32_t *dstAddr, uint32_t *baseAddr, uint32_t xfercnt);

#endif  /* __SPI_DRIVER_H__ */

