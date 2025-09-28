/*
 * Copyright (c) 2013-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <asm_macros.S>
#include <cpu_macros.S>
#include <platform_def.h>

	.globl	plat_secondary_cold_boot_setup
	.globl	plat_secondary_cold_boot_bl3
	.globl	plat_get_my_entrypoint
	.globl	plat_is_my_cpu_primary
	.globl  plat_arm_calc_core_pos
	.globl  plat_crash_console_init
	.globl  plat_crash_console_putc
	.globl  plat_crash_console_flush
	.globl  platform_mem_init
	.globl  plat_my_core_pos

	/* ---------------------------------------------------------------------
	 * void plat_secondary_cold_boot_setup(void);
	 *
	 * In the normal boot flow, cold-booting secondary CPUs is not yet
	 * implemented and they panic.
	 *
	 * This routine makes secondary jump to bl3 and loop there in the mailbox.
	 * ---------------------------------------------------------------------
	 */
func plat_secondary_cold_boot_setup
	wfe
	mov_imm x0, PLAT_ARM_TRUSTED_MAILBOX_BASE

	/* Wait until the mailbox gets populated */
poll_bl3_entry:
	ldr	x1, [x0]
	cbz	x1, 1f
	br	x1
1:
	wfe
	b	poll_bl3_entry
endfunc plat_secondary_cold_boot_setup

	/* ---------------------------------------------------------------------
	 * void plat_secondary_cold_boot_bl3(void);
	 *
	 * In the normal boot flow, cold-booting secondary CPUs is not yet
	 * implemented and they panic.
	 *
	 * When booting an EL3 payload, secondary CPUs are placed in a holding
	 * pen, waiting for their mailbox to be populated. Note that all CPUs
	 * share the same mailbox ; therefore, populating it will release all
	 * CPUs from their holding pen. If finer-grained control is needed then
	 * this should be handled in the code that secondary CPUs jump to.
	 * ---------------------------------------------------------------------
	 */
func plat_secondary_cold_boot_bl3
	bl      plat_my_core_pos
	mov     x4, x0
	mov_imm x5, 0x8
	mul     x4, x4, x5
	mov_imm x0, PLAT_ARM_TRUSTED_MAILBOX_BASE
	add     x0, x0, x4

	/* Wait until the mailbox gets populated */
poll_mailbox:
	ldr	x1, [x0]
	cbz	x1, 1f
	br	x1
1:
	wfe
	b	poll_mailbox
endfunc plat_secondary_cold_boot_bl3

	/* ---------------------------------------------------------------------
	 * uintptr_t plat_get_my_entrypoint (void);
	 *
	 * Main job of this routine is to distinguish between a cold and a warm
	 * boot. On CSS platforms, this distinction is based on the contents of
	 * the Trusted Mailbox. It is initialised to zero by the SCP before the
	 * AP cores are released from reset. Therefore, a zero mailbox means
	 * it's a cold reset.
	 *
	 * This functions returns the contents of the mailbox, i.e.:
	 *  - 0 for a cold boot;
	 *  - the warm boot entrypoint for a warm boot.
	 * ---------------------------------------------------------------------
	 */
func plat_get_my_entrypoint
	//mov_imm	x0, PLAT_ARM_TRUSTED_MAILBOX_BASE
	//ldr	x0, [x0]
	mov_imm     x0, 0
	ret
endfunc plat_get_my_entrypoint

	/* -----------------------------------------------------
	 * unsigned int plat_is_my_cpu_primary (void);
	 *
	 * Find out whether the current cpu is the primary
	 * cpu (applicable ony after a cold boot)
	 * -----------------------------------------------------
	 */
func plat_is_my_cpu_primary
	mov	x9, x30
	bl	plat_my_core_pos
	mov	x4, x0
        /* CPU zero is primary CPU */
#if SINGLE_PPU_CLUSTER
	mov     x0, #0x0000000C
#else
	mov     x0, #0x00000000
#endif
	/* Check for error */
	mov	x1, #0xffffffff
	cmp	x0, x1
	b.eq	1f
	cmp	x0, x4
	cset	w0, eq
	ret	x9
1:
	no_ret	plat_panic_handler
endfunc plat_is_my_cpu_primary

        /* -----------------------------------------------------
         * unsigned int plat_arm_calc_core_pos(u_register_t mpidr)
         *
	 * Ported from N1SDP for the Neoverse E1.
	 *
         * (((ChipId * PLAT_MAX_CLUSTERS_PER_CHIP + ClusterId) *
         * PLAT_MAX_CPUS_PER_CLUSTER + CPUId) * PLAT_MAX_PE_PER_CPU) +
         * ThreadId
         * ------------------------------------------------------
         */

func plat_arm_calc_core_pos
        mov     x4, x0

        /*
         * The MT bit in MPIDR is always set for n1sdp and the
         * affinity level 0 corresponds to thread affinity level.
         */

        /* Extract individual affinity fields from MPIDR */
        ubfx    x0, x4, #MPIDR_AFF0_SHIFT, #MPIDR_AFFINITY_BITS
        ubfx    x1, x4, #MPIDR_AFF1_SHIFT, #MPIDR_AFFINITY_BITS
        ubfx    x2, x4, #MPIDR_AFF2_SHIFT, #MPIDR_AFFINITY_BITS
        ubfx    x3, x4, #MPIDR_AFF3_SHIFT, #MPIDR_AFFINITY_BITS

        /* Compute linear position */
        mov     x4, #PLAT_ARM_CLUSTER_COUNT
        madd    x2, x3, x4, x2
        mov     x4, #PLAT_MAX_CPUS_PER_CLUSTER
        madd    x1, x2, x4, x1
        mov     x4, #PLAT_MAX_PE_PER_CPU
        madd    x0, x1, x4, x0
        ret
endfunc plat_arm_calc_core_pos

	/* -----------------------------------------------------
	*  unsigned int plat_my_core_pos(void)
	*  This function uses the plat_arm_calc_core_pos()
	*  definition to get the index of the calling CPU.
	* -----------------------------------------------------
	*/
func plat_my_core_pos
	mrs     x0, mpidr_el1
	b       plat_arm_calc_core_pos
endfunc plat_my_core_pos

	/* ---------------------------------------------
	* int plat_crash_console_init(void)
	* Function to initialize the crash console
	* without a C Runtime to print crash report.
	* Clobber list : x0 - x4
	* ---------------------------------------------
	*/
func plat_crash_console_init
	mov_imm x0, PLAT_ARM_CRASH_UART_BASE
	mov_imm x1, PLAT_ARM_CRASH_UART_CLK_IN_HZ
	mov_imm x2, ARM_CONSOLE_BAUDRATE

#if RAPTOR2_FASTMODEL
	b       console_pl011_core_init
#else
	b	console_16550_core_init
#endif

endfunc plat_crash_console_init

	/* ---------------------------------------------
	* int plat_crash_console_putc(int c)
	* Function to print a character on the crash
	* console without a C Runtime.
	* Clobber list : x1, x2
	* ---------------------------------------------
	*/
func plat_crash_console_putc
	mov_imm x1, PLAT_ARM_CRASH_UART_BASE

#if RAPTOR2_FASTMODEL
	b       console_pl011_core_putc
#else
	b	console_16550_putc
#endif
	
endfunc plat_crash_console_putc

	/* ---------------------------------------------
	* int plat_crash_console_flush()
	* Function to force a write of all buffered
	* data that hasn't been output.
	* Out : return -1 on error else return 0.
	* Clobber list : r0
	* ---------------------------------------------
	*/
func plat_crash_console_flush
	mov_imm x0, PLAT_ARM_CRASH_UART_BASE

#if RAPTOR2_FASTMODEL
	b       console_pl011_core_flush
#else
	b	console_16550_getc
#endif

endfunc plat_crash_console_flush

	/* ---------------------------------------------------------------------
	* We don't need to carry out any memory initialization on ARM
	* platforms. The Secure RAM is accessible straight away.
	* ---------------------------------------------------------------------
	*/
func platform_mem_init
	ret
endfunc platform_mem_init

