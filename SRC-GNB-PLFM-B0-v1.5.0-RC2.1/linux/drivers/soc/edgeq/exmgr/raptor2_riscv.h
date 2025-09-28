/*
 * EdgeQ Inc.
 *
 * Raptor2 Register Offsets for RISC-V Scalar Units
 */


/* CMU Register Offsets */
#define	CMU_TRIGGER_REG			0x0
#define	CMU_CORE_CONFIG_REG		0x20
#define	CMU_RESET_VECTOR_BASE_ADDR	0x24
#define	CMU_CORE_STATUS_REG		0x28

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

struct suregs {
	void __iomem *cmu;
	void __iomem *stmr;
	void __iomem *tmr;
	void __iomem *plic;
};
