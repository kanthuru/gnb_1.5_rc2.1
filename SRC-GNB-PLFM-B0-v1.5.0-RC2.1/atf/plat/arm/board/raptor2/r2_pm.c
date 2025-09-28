/*
 * Copyright (c) 2019, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <assert.h>
#include <drivers/arm/gicv2.h>
#include <lib/psci/psci.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/platform.h>

uintptr_t  secondary_entrypoint;

/*
 * Platform handler called when a power domain is about to be turned on. 
 * mpidr contains the target CPU id.
 */
int r2_pwr_domain_on(u_register_t mpidr)
{
	unsigned int pos = plat_core_pos_by_mpidr(mpidr);
	uintptr_t    *mailbox_base = (void *) PLAT_ARM_TRUSTED_MAILBOX_BASE;

	/*
	 * Populate CPU ID specific mailbox.
	 */
	mailbox_base[pos] = secondary_entrypoint;

	dsb();
	sev();

	return PSCI_E_SUCCESS;
}

/*
 * Platform handler called when a power domain has just been powered on.
 * The target_state encodes the low power state that each level has woken up from.
 */
void r2_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	/* Program GIC per-cpu distributor or re-distributor interface */
	plat_arm_gic_pcpu_init();

	/* Enable GIC CPU interface */
	plat_arm_gic_cpuif_enable();

	//arm_configure_sys_timer();
}

/*
 * Platform handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 */
void r2_pwr_domain_off(const psci_power_state_t *target_state)
{
	return;
}

/*
 * Export the platform handlers via r2_psci_pm_ops. The ARM Standard
 * platform layer will take care of registering the handlers with PSCI.
 */
plat_psci_ops_t r2_psci_pm_ops = {
	.pwr_domain_on = r2_pwr_domain_on,
	.pwr_domain_on_finish = r2_pwr_domain_on_finish,
	.pwr_domain_off = r2_pwr_domain_off
};

int __init plat_setup_psci_ops(uintptr_t sec_entrypoint,
				const plat_psci_ops_t **psci_ops)
{
	*psci_ops = &r2_psci_pm_ops;

	/*
	 * Secondary CPU entrypoint is saved.
	 * This will be used during boot through PSCI power on.
	 */
	secondary_entrypoint = sec_entrypoint;

	return 0;
}
