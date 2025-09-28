// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 EdgeQ Inc.
 */

#include <crypto/internal/aead.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/authenc.h>
#include <crypto/internal/des.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/skcipher.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <crypto/gcm.h>

#include "raptor2_crss.h"


struct raptor2_crss_alg crss_engine_algs[] = {
	{
		.ctrl_default = CRSS_CTRL_CIPH_ALG_AES | CRSS_CTRL_CIPH_MODE_CBC,
		.alg = {
			.base.cra_name		= "cbc(aes)",
			.base.cra_driver_name	= "cbc-aes-edgeq",
			.base.cra_priority	= CRSS_CRYPTO_ALG_PRIORITY,
			.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC |
							CRYPTO_ALG_NEED_FALLBACK,
			.base.cra_blocksize	= AES_BLOCK_SIZE,
			.base.cra_ctxsize	= sizeof(struct raptor2_crss_ablk_ctx),
			.base.cra_module	= THIS_MODULE,

			.setkey			= raptor2_crss_aes_setkey,
			.encrypt		= raptor2_crss_ablk_encrypt,
			.decrypt		= raptor2_crss_ablk_decrypt,
			.min_keysize		= AES_MIN_KEY_SIZE,
			.max_keysize		= AES_MAX_KEY_SIZE,
			.ivsize			= AES_BLOCK_SIZE,
			.init			= raptor2_crss_ablk_init_tfm,
			.exit			= raptor2_crss_ablk_exit_tfm,
		},
	},
	{
		.ctrl_default = CRSS_CTRL_CIPH_ALG_AES | CRSS_CTRL_CIPH_MODE_CTR,
		.alg = {
			.base.cra_name		= "ctr(aes)",
			.base.cra_driver_name	= "ctr-aes-edgeq",
			.base.cra_priority	= CRSS_CRYPTO_ALG_PRIORITY,
			.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY |
						  CRYPTO_ALG_ASYNC |
						  CRYPTO_ALG_NEED_FALLBACK,
			.base.cra_blocksize	= AES_BLOCK_SIZE,
			.base.cra_ctxsize	= sizeof(struct raptor2_crss_ablk_ctx),
			.base.cra_module	= THIS_MODULE,

			.setkey			= raptor2_crss_aes_setkey,
			.encrypt		= raptor2_crss_ablk_encrypt,
			.decrypt		= raptor2_crss_ablk_decrypt,
			.min_keysize		= AES_MIN_KEY_SIZE,
			.max_keysize		= AES_MAX_KEY_SIZE,
			.ivsize			= AES_BLOCK_SIZE,
			.init			= raptor2_crss_ablk_init_tfm,
			.exit			= raptor2_crss_ablk_exit_tfm,
		},
	},
};

struct raptor2_crss_aead crss_engine_aeads[] = {

	{
		.ctrl_default = CRSS_CTRL_CIPH_ALG_AES | CRSS_CTRL_CIPH_MODE_GCM,
		.alg = {
			.base = {
				.cra_name = "gcm(aes)",
				.cra_driver_name = "gcm-aes-edgeq",
				.cra_priority = CRSS_CRYPTO_ALG_PRIORITY,
				.cra_flags = CRYPTO_ALG_ASYNC |
						CRYPTO_ALG_NEED_FALLBACK |
						CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = 1,
				.cra_ctxsize = sizeof(struct raptor2_crss_aead_ctx),
				.cra_module = THIS_MODULE,
			},
			.setkey = raptor2_crss_gcm_setkey,
			.setauthsize = raptor2_crss_gcm_setauthsize,
			.encrypt = raptor2_crss_gcm_encrypt,
			.decrypt = raptor2_crss_gcm_decrypt,
			.ivsize = GCM_AES_IV_SIZE,
			.maxauthsize = AES_BLOCK_SIZE,
			.init = raptor2_crss_aead_cra_init,
			.exit = raptor2_crss_aead_cra_exit,
		},
	},

	{
		.ctrl_default = CRSS_CTRL_CIPH_ALG_AES | CRSS_CTRL_CIPH_MODE_GCM,
		.alg = {
			.base = {
				.cra_name = "rfc4106(gcm(aes))",
				.cra_driver_name = "rfc4106-gcm-aes-edgeq",
				.cra_priority = CRSS_CRYPTO_ALG_PRIORITY,
				.cra_flags = CRYPTO_ALG_ASYNC |
					     CRYPTO_ALG_NEED_FALLBACK |
					     CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = 1,
				.cra_ctxsize = sizeof(struct raptor2_crss_aead_ctx),
				.cra_module = THIS_MODULE,
			},
			.setkey = raptor2_crss_rfc4106_setkey,
			.setauthsize = raptor2_crss_rfc4106_setauthsize,
			.encrypt = raptor2_crss_rfc4106_encrypt,
			.decrypt = raptor2_crss_rfc4106_decrypt,
			.ivsize = GCM_RFC4106_IV_SIZE,
			.maxauthsize = AES_BLOCK_SIZE,
			.init = raptor2_crss_aead_cra_init,
			.exit = raptor2_crss_aead_cra_exit,
		},
	},
};

unsigned int raptor2_crss_num_algs = ARRAY_SIZE(crss_engine_algs);
unsigned int raptor2_crss_num_aeads = ARRAY_SIZE(crss_engine_aeads);

