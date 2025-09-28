/*
 * EdgeQ Inc.
 *
 * Raptor2 Register Offsets for RISC-V Scalar Units
 */


/* CMU0 Register Offsets */
#define	CMU_TRIGGER_REG			0x0
#define	CMU_CORE_CONFIG_REG		0x20
#define CMU_RESET_VECTOR_BASE_ADDR	0x24
#define	CMU_CORE_STATUS_REG		0x28

/* CMU1 Register Offsets */
#define	CMU1_TRIGGER_REG		0x0
#define	CMU1_EVENT_REG			0x8
#define	CMU1_CORE_CONFIG_REG		0x78
#define	CMU1_WC_ADDR1_CFG		0x70
#define	CMU1_WC_ADDR2_CFG		0x68
#define	CMU1_WC_ADDR3_CFG		0x60
#define	CMU1_WC_ADDR4_CFG		0x58
#define	CMU1_ADDRM_CFG1			0x50
#define	CMU1_ADDRM_CFG2			0x48
#define	CMU1_ADDRM_CFG3			0x40
#define	CMU1_ADDRM_CFG4			0x38
#define	CMU1_ADDRM_CFG5			0x30
#define	CMU1_ADDRM_CFG6			0x28
#define	CMU1_GPIO_CFG			0x20
#define CMU1_HW_STMR_SNP0_ST		0xB0
#define CMU1_HW_STMR_SNP1_ST		0xA8
#define CMU1_STATUS			0xA0
#define CMU1_WR_COL_ST			0x98
#define CMU1_STMR_SNP0_ST		0x90
#define CMU1_STMR_SNP1_ST		0x88
#define CMU1_ADDR_MISM_ST		0x80

/* STMR Register Offsets */
#define	STMR_REQ_INT0_TIME		0x20
#define	STMR_REQ_INT1_TIME		0x24
#define	STMR_REQ_INT2_TIME		0x28
#define	STMR_REQ_INT3_TIME		0x2C
#define	STMR_HYST_VAL			0x30
#define	STMR_CURR_SYS_TIME		0x34

/* TMR Register Offsets */
#define TMR_MTIME_REG0			0x0
#define TMR_MTIME_REG1			0x4
#define TMR_MTIME_CMP_REG0		0x8
#define TMR_MTIME_CMP_REG1		0xC

/*sPLIC Register Offsets*/
#define PLIC_PENDING_OFFSET 		0x00001000 

#define HARTID_SHIFT			19
#define HARTID_MASK			0x3F
#define HARTID_REG_MASK			(HARTID_MASK << HARTID_SHIFT)

#define OTRX_LMEM_TRIGGER_OFFSET	0x000BFFFC
#define TRIGGER_PATTERN			0xFEEDFEED

struct suregs {
	void __iomem *cmu;
	void __iomem *stmr;
	void __iomem *tmr;
	void __iomem *plic;
};
