/*
 * Copyright (c) 2018-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <arch.h>
#include <asm_macros.S>
#include <common/bl_common.h>
#include <common/debug.h>
#include <neoverse_e1.h>
#include <cpu_macros.S>
#include <plat_macros.S>

/* Hardware handled coherency */
#if HW_ASSISTED_COHERENCY == 0
#error "Neoverse E1 must be compiled with HW_ASSISTED_COHERENCY enabled"
#endif

/* 64-bit only core */
#if CTX_INCLUDE_AARCH32_REGS == 1
#error "Neoverse-E1 supports only AArch64. Compile with CTX_INCLUDE_AARCH32_REGS=0"
#endif

	/* -------------------------------------------------
	 * The CPU Ops reset function for Neoverse-E1.
	 * Shall clobber: x0-x19
	 * -------------------------------------------------
	 */
func neoverse_e1_reset_func
	mov	x19, x30

#if ERRATA_DSU_936184
	bl	errata_dsu_936184_wa
#endif

#if NEOVERSE_E1_EXTERNAL_LLC
	/* Some system may have External LLC, core needs to be made aware */
	mrs     x0, NEOVERSE_E1_CPUECTLR_EL1
	orr     x0, x0, NEOVERSE_E1_CPUECTLR_EL1_EXTLLC_BIT
	bic     x0, x0, #0x3F000000		/* Lowest Streaming */
	/* Best Prefetch */
	orr	x0, x0, #0xE000
	orr	x0, x0, #0x0400
	msr     NEOVERSE_E1_CPUECTLR_EL1, x0
#endif

	/* Make sure accesses from EL0/EL1 and EL2 are not trapped to EL3 */
	mov	x0, #0x1CA3
	msr     actlr_el3, x0
	msr     actlr_el2, x0
	isb

	ret	x19
endfunc neoverse_e1_reset_func

func neoverse_e1_cpu_pwr_dwn
	mrs	x0, NEOVERSE_E1_CPUPWRCTLR_EL1
	orr	x0, x0, #NEOVERSE_E1_CPUPWRCTLR_EL1_CORE_PWRDN_BIT
	msr	NEOVERSE_E1_CPUPWRCTLR_EL1, x0
	isb
	ret
endfunc neoverse_e1_cpu_pwr_dwn

#if REPORT_ERRATA
/*
 * Errata printing function for Neoverse N1. Must follow AAPCS.
 */
func neoverse_e1_errata_report
	stp	x8, x30, [sp, #-16]!

	bl	cpu_get_rev_var
	mov	x8, x0

	/*
	 * Report all errata. The revision-variant information is passed to
	 * checking functions of each errata.
	 */
	report_errata ERRATA_DSU_936184, neoverse_e1, dsu_936184

	ldp	x8, x30, [sp], #16
	ret
endfunc neoverse_e1_errata_report
#endif


.section .rodata.neoverse_e1_regs, "aS"
neoverse_e1_regs:  /* The ascii list of register names to be reported */
	.asciz	"cpuectlr_el1", ""

func neoverse_e1_cpu_reg_dump
	adr	x6, neoverse_e1_regs
	mrs	x8, NEOVERSE_E1_ECTLR_EL1
	ret
endfunc neoverse_e1_cpu_reg_dump

declare_cpu_ops neoverse_e1, NEOVERSE_E1_MIDR, \
	neoverse_e1_reset_func, \
	neoverse_e1_cpu_pwr_dwn
