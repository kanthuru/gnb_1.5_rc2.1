/*
 * Copyright (c) 2019-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <plat/arm/common/plat_arm.h>

/******************************************************************************
 * FKAN: Need to revisit this as this was written for RD-E1-Edge platform
 *       We need to fix it for Raptor platform
 * The power domain tree descriptor. RD-E1-Edge platform consists of two
 * clusters with eight CPUs in each cluster. The CPUs are multi-threaded with
 * two threads per CPU.
 ******************************************************************************/
static const unsigned char r2_pd_tree_desc[] = {
	RAPTOR2_CHIP_COUNT,
	PLAT_ARM_CLUSTER_COUNT,
	PLAT_MAX_CPUS_PER_CLUSTER * PLAT_MAX_PE_PER_CPU,
        PLAT_MAX_CPUS_PER_CLUSTER * PLAT_MAX_PE_PER_CPU
};

/******************************************************************************
 * This function returns the topology tree information.
 ******************************************************************************/
const unsigned char *plat_get_power_domain_tree_desc(void)
{
	return r2_pd_tree_desc;
}


/******************************************************************************
 * This function implements a part of the critical interface between the psci
 * generic layer and the platform that allows the former to query the platform
 * to convert an MPIDR to a unique linear index. An error code (-1) is
 * returned in case the MPIDR is invalid.
 *****************************************************************************/
int plat_core_pos_by_mpidr(u_register_t mpidr)
{
        if (arm_check_mpidr(mpidr) == 0) {
#if ARM_PLAT_MT
                assert((read_mpidr_el1() & MPIDR_MT_MASK) != 0);

                /*
                 * The DTB files don't provide the MT bit in the mpidr argument
                 * so set it manually before calculating core position
                 */
                mpidr |= MPIDR_MT_MASK;
#endif
                return plat_arm_calc_core_pos(mpidr);
        }
        return -1;
}

/*
 * Common topology related methods for SGI and RD based platforms
 */
/*******************************************************************************
 * This function returns the core count within the cluster corresponding to
 * `mpidr`.
 ******************************************************************************/
unsigned int plat_arm_get_cluster_core_count(u_register_t mpidr)
{
        return PLAT_MAX_CPUS_PER_CLUSTER;
}

#if ARM_PLAT_MT
/******************************************************************************
 * Return the number of PE's supported by the CPU.
 *****************************************************************************/
unsigned int plat_arm_get_cpu_pe_count(u_register_t mpidr)
{
        return PLAT_MAX_PE_PER_CPU;
}
#endif
