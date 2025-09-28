/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2020 EdgeQ Inc.
 */
#ifndef __RAPTOR2_CRSS_H__
#define __RAPTOR2_CRSS_H__

#include <linux/scatterlist.h>
#include "raptor2_crss_hw_ring.h"


#define CRSS_STATUS_OK                   0
#define CRSS_STATUS_ICV_FAIL             1
#define CRSS_STATUS_MEMORY_ERROR         2
#define CRSS_STATUS_BLOCK_ERROR          3
#define CRSS_STATUS_SECURITY_ERROR       4

#define CRSS_CTRL_CIPH_MODE_IDX          4
#define CRSS_CTRL_HASH_ALG_IDX           8
#define CRSS_CTRL_HASH_MODE_IDX          13
#define CRSS_CTRL_ENCRYPT_IDX            15
#define CRSS_CTRL_CTX_IDX                16
#define CRSS_CTRL_AAD_COPY               25
#define CRSS_CTRL_ICV_PT                 26
#define CRSS_CTRL_ICV_ENC                27
#define CRSS_CTRL_ICV_APPEND             28
#define CRSS_CTRL_KEY_EXP                29
#define CRSS_CTRL_MSG_BEGIN              30
#define CRSS_CTRL_MSG_END                31

#define CRSS_CTRL_CIPH_ALG_NULL          0x00
#define CRSS_CTRL_CIPH_ALG_DES           0x01
#define CRSS_CTRL_CIPH_ALG_AES           0x02
#define CRSS_CTRL_CIPH_ALG_RC4           0x03
#define CRSS_CTRL_CIPH_ALG_MULTI2        0x04
#define CRSS_CTRL_CIPH_ALG_KASUMI        0x05

#define CRSS_CTRL_CIPH_MODE_ECB          (0x00 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_CBC          (0x01 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_CTR          (0x02 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_CCM          (0x03 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_GCM          (0x05 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_OFB          (0x07 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_CFB          (0x08 << CRSS_CTRL_CIPH_MODE_IDX)
#define CRSS_CTRL_CIPH_MODE_F8           (0x09 << CRSS_CTRL_CIPH_MODE_IDX)

#define CRSS_CTRL_HASH_ALG_NULL          (0x00 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_MD5           (0x01 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_SHA           (0x02 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_SHA224        (0x03 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_SHA256        (0x04 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_SHA384        (0x05 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_SHA512        (0x06 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_AESMAC        (0x07 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_AESCMAC       (0x08 << CRSS_CTRL_HASH_ALG_IDX)
#define CRSS_CTRL_HASH_ALG_KASF9         (0x09 << CRSS_CTRL_HASH_ALG_IDX)

#define CRSS_CTRL_HASH_MODE_RAW          (0x00 << CRSS_CTRL_HASH_MODE_IDX)
#define CRSS_CTRL_HASH_MODE_SSLMAC       (0x01 << CRSS_CTRL_HASH_MODE_IDX)
#define CRSS_CTRL_HASH_MODE_HMAC         (0x02 << CRSS_CTRL_HASH_MODE_IDX)

/* The priority to register each algorithm with. */
#define CRSS_CRYPTO_ALG_PRIORITY  10000

#define RAPTOR2_CRSS_TIMEOUT 100000

#define HALF_WORD_MASK     0xFFFF
#define WORD_MASK          0xFFFFFFFF

#define MAX_KEY_ENTRIES	16
#define KEY_OFFSET		48

#define GET_KEY_CMD(newKeyBit, key_idx) ((newKeyBit << 15) | (key_idx + KEY_OFFSET))

#define GCM_RFC4106_SALT_SIZE 4
#define GCM_RFC4106_AAD_MAX 16

struct crss_dma_scatter {
	uint32_t buff_addr_low;  /* B31-B0  */
	uint16_t buff_addr_high; /* B47-B32 */
	uint16_t buff_len;
};

struct crypto_status {
	uint16_t pkt_idx;
	uint8_t ret_code:3;
	uint8_t res1:5;
	uint8_t res2;
};

#define  MAX_KEY_SIZE 64
struct crss_key_table {
	uint8_t cipher_key[MAX_KEY_SIZE];
	uint8_t hash_key[MAX_KEY_SIZE];
};


	/*		CRSS command descriptor format
	 *
	 *	___________________________________________
	 *	| NK | FC | R | KEY IDX |     PKT IDX       |
	 *	|____|____|___|_________|___________________|
	 *	|          SRC & DST ADDR-LEN PAIR          |
	 *	|                     :                     |
	 *	|                     :                     |
	 *	|___________________________________________|
	 *	|  CMD LEN    |    R    |      CMD TYPE     |
	 *	|_____________|_________|___________________|
	 *	|                CMD_CTRL                   |
	 *	|___________________________________________|
	 *	|		PROC LEN		    |
	 *	|___________________________________________|
	 *	|                 IV LEN                    |
	 *	|___________________________________________|
	 *	|		Pre-AAD Len		    |
	 *	|___________________________________________|
	 *	|		ICV Len			    |
	 *	|___________________________________________|
	 *	|                KEY_SIZE                   |
	 *	|___________________________________________|
	 *
	 *
	 *
	 *		SRC Packet Format
	 *	____________________________________
	 *	|    IV   |   Pre-AAD   |  Payload  |
	 *	|_________|_____________|___________|
	 *
	 *
	 *
	 *		DST Packet Format
	 *	____________________________________
	 *	| Pre-AAD |  Payload    |    ICV    |
	 *	|_________|_____________|___________|
	 */


/* This structure is only for reference and cannnot be used directly as
 * size of src_dma and dst_dma will be dynamic and can only be calculated
 * as per the size of scatterlists passed on the fly.
 */
struct crss_command {
	uint16_t pkt_idx;
	uint16_t key_info;
	uint32_t src_buf_len;
	struct crss_dma_scatter src_dma[0]; /* dynamic, change as per src scatterlist */
	uint32_t dst_buf_len;
	struct crss_dma_scatter dst_dma[0]; /* dynamic, change as per dest scatterlist */
	uint32_t cmd_len_type;
	uint32_t cmd_ctrl;
	uint32_t proc_len;
	uint32_t iv_length;
	uint32_t assoclen;	/* Pre-AAD len*/
	uint32_t icv_len;	/* Auth tag size*/
	uint32_t key_size;
} __packed;


struct crss_command_0 {
	uint16_t pkt_idx;
	uint16_t key_info;
	uint32_t src_buf_len;
	struct crss_dma_scatter src_dma[0]; /* dynamic, change as per src scatterlist */
} __packed;

struct crss_command_1 {
	uint32_t dst_buf_len;
	struct crss_dma_scatter dst_dma[0]; /* dynamic, change as per dest scatterlist */
} __packed;

struct crss_command_2 {
	uint32_t cmd_len_type;
	uint32_t cmd_ctrl;
	uint32_t proc_len;
	uint32_t iv_length;
	uint32_t assoclen;	/* Pre-AAD len*/
	uint32_t icv_len;	/* Auth tag size*/
	uint32_t key_size;
} __packed;


struct crss_queue_pair_ctx {
	struct raptor2_crss_engine *engine;
	void __iomem		*cmdq_base_addr;
	void __iomem		*stsq_base_addr;
	void __iomem		*qau_base;
	spinlock_t		cmdq_lock;
	spinlock_t		lock;
	atomic_t		in_flight;
	int			current_pkt_idx;
	struct crss_hw_ring_ctx	command_ring_ctx;
	struct crss_hw_ring_ctx	status_ring_ctx;
	struct list_head	in_progress;
	struct tasklet_struct	status_task;
	int			irq_no;
	int			queue_num;
};


struct raptor2_crss_engine {
	struct crss_queue_pair_ctx	*queue_pair_ctx;
	void __iomem			*q_pool_addr;
	void __iomem			*memss_keytbl_base_addr;
	phys_addr_t			q_pool_addr_pa;
	void __iomem			*regs;
	struct raptor2_crss_alg		*algs;
	unsigned int			num_algs;
	struct raptor2_crss_aead	*aeads;
	unsigned int			num_aeads;
	const char			*name;
	struct device			*dev;
	int				num_queues;
	int				curr_queue_idx;
	int				q_pool_size;
	atomic_t			key_table[MAX_KEY_ENTRIES];
	bool				is_b0;
};

struct raptor2_crss_req {
	struct list_head		list;
	struct crypto_async_request	*areq;

	unsigned int	src_len;
	unsigned int	dst_len;
	unsigned int	assoclen;
	unsigned int	cryptlen;
	unsigned int	authsize;
	unsigned int	ivsize;
	unsigned int	key_size;
	unsigned int	cmd_size;

	unsigned int	src_nents;
	unsigned int	dst_nents;
	unsigned int	mapped_src_ents;
	unsigned int	mapped_dst_ents;
	unsigned int	new_key;
	unsigned int	key_idx;
	unsigned int	hdr_len;

	dma_addr_t	hdr_dma_addr;
	u8		crss_hdr[AES_BLOCK_SIZE + GCM_RFC4106_AAD_MAX];

	bool		is_encrypt;
	uint32_t	ctrl;

	struct scatterlist	*src;
	struct scatterlist	*dst;
	struct scatterlist	_dst[2];

	/* RFC4106 specifics */
	bool			is_rfc4106;
	struct scatterlist	rfc_src[3];
	struct scatterlist	rfc_dst[3];
};


struct raptor2_crss_aead {
	unsigned long			ctrl_default;
	struct aead_alg			alg;
	struct raptor2_crss_engine	*engine;
};

/* RAPTOR2 CRSS definition of a crypto algorithm. */
struct raptor2_crss_alg {
	unsigned long			ctrl_default;
	struct skcipher_alg		alg;
	struct raptor2_crss_engine	*engine;
};

/* Generic context structure for any algorithm type. */
struct raptor2_crss_generic_ctx {
	struct raptor2_crss_engine    *engine;
	int curr_q;
	int new_key;
	int key_idx;
};

/* Block cipher context. */
struct raptor2_crss_ablk_ctx {
	struct raptor2_crss_generic_ctx  generic;
	int	key_len;
};

/* AEAD cipher context. */
struct raptor2_crss_aead_ctx {
	struct raptor2_crss_generic_ctx  generic;
	u8		cipher_key_len;
	u8		hash_key_len;
	int		authsize;
	u8	salt[GCM_RFC4106_SALT_SIZE];
};

extern struct raptor2_crss_alg crss_engine_algs[];
extern struct raptor2_crss_aead crss_engine_aeads[];
extern unsigned int raptor2_crss_num_algs;
extern unsigned int raptor2_crss_num_aeads;


int raptor2_crss_aead_cra_init(struct crypto_aead *tfm);
void raptor2_crss_aead_cra_exit(struct crypto_aead *tfm);
int raptor2_crss_ablk_encrypt(struct skcipher_request *req);
int raptor2_crss_ablk_decrypt(struct skcipher_request *req);
int raptor2_crss_ablk_init_tfm(struct crypto_skcipher *tfm);
void raptor2_crss_ablk_exit_tfm(struct crypto_skcipher *tfm);
int raptor2_crss_aead_setkey(struct crypto_aead *tfm, const u8 *key, unsigned int len);
int raptor2_crss_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize);
int raptor2_crss_aes_setkey(struct crypto_skcipher *cipher, const u8 *key, unsigned int len);
int raptor2_crss_des_setkey(struct crypto_skcipher *cipher, const u8 *key, unsigned int len);
int raptor2_crss_des3_setkey(struct crypto_skcipher *cipher, const u8 *key, unsigned int len);

int raptor2_crss_gcm_setkey(struct crypto_aead *aead, const u8 *key, unsigned int keylen);
int raptor2_crss_gcm_setauthsize(struct crypto_aead *tfm, unsigned int authsize);
int raptor2_crss_gcm_encrypt(struct aead_request *req);
int raptor2_crss_gcm_decrypt(struct aead_request *req);

int raptor2_crss_rfc4106_setkey(struct crypto_aead *aead, const u8 *key, unsigned int keylen);
int raptor2_crss_rfc4106_setauthsize(struct crypto_aead *tfm, unsigned int authsize);
int raptor2_crss_rfc4106_encrypt(struct aead_request *req);
int raptor2_crss_rfc4106_decrypt(struct aead_request *req);


int raptor2_crss_seqiv_rfc4106_encrypt(struct aead_request *req);
int raptor2_crss_seqiv_rfc4106_decrypt(struct aead_request *req);


#endif /* __RAPTOR2_CRSS_H__ */
