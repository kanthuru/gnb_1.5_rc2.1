/*
 * Copyright (c) 2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <plat/common/platform.h>
#include <plat/arm/common/plat_arm.h>

#define BL2_ROTK_ADDR (R2_DDR_BASE + 0xF18000)
#define PUBLIC_KEY_DER_SIZE 294
/*
 * Return the ROTPK hash in the following ASN.1 structure in DER format:
 *
 * AlgorithmIdentifier  ::=  SEQUENCE  {
 *     algorithm         OBJECT IDENTIFIER,
 *     parameters        ANY DEFINED BY algorithm OPTIONAL
 * }
 *
 * DigestInfo ::= SEQUENCE {
 *     digestAlgorithm   AlgorithmIdentifier,
 *     digest            OCTET STRING
 * }
 */
int plat_get_rotpk_info(void *cookie, void **key_ptr, unsigned int *key_len,
			unsigned int *flags)
{
#if (ARM_ROTPK_LOCATION_ID == ARM_ROTPK_DEVEL_RSA_ID) || \
	(ARM_ROTPK_LOCATION_ID == ARM_ROTPK_DEVEL_ECDSA_ID)

	return arm_get_rotpk_info(cookie, key_ptr, key_len, flags);
#else
	*key_ptr = (void *)BL2_ROTK_ADDR;
	*key_len = PUBLIC_KEY_DER_SIZE;
	*flags = 0;

	return 0;
#endif
}

/*
 * Raptor2 platform does not use NV Counter functionality.
 * Both get/set functions are for ATF AUTH module.
 * Note:  get() function should make sure value is zero initialized.
 */
int plat_get_nv_ctr(void *cookie, unsigned int *nv_ctr)
{
	*nv_ctr = 0;

        return 0;
}

int plat_set_nv_ctr(void *cookie, unsigned int nv_ctr)
{
        return 0;
}

/*
 * Heap wrapper API for mbedtls. Used by ATF/mbedtls Auth module.
 */
int plat_get_mbedtls_heap(void **heap_addr, size_t *heap_size)
{
        return get_mbedtls_heap_helper(heap_addr, heap_size);
}
