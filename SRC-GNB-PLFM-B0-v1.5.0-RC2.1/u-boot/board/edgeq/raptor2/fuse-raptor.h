#ifndef _FUSE_RAPTOR_H
#define _FUSE_RAPTOR_H
#include <configs/edgeq_raptor2.h>

/* 14MB-15MB is reserved for SU Image loading
 * 15MB-16MB is for bss runtime app + shared memory msg
 * first 64K is for bssruntime
 * shared memory starts after 64K
 */
#define BSS_RUNTIME_SHM_BASE		(CONFIG_SYS_SDRAM_BASE + 0x00E00000)
#define BSS_RUNTIME_CHAN_BASE		(BSS_RUNTIME_SHM_BASE + 0x00100000)
#define BSS_RT_READY_ADDR		BSS_RUNTIME_CHAN_BASE
#define BSS_RUNTIME_EFUSE_BASE		(BSS_RUNTIME_CHAN_BASE + 0x10000)
#define CMD_MSG_READY_ADDR		(BSS_RUNTIME_EFUSE_BASE + 0x08)
#define RESP_MSG_READY_ADDR		(BSS_RUNTIME_EFUSE_BASE + 0x10)
#define CMD_MSG_ADDR			(BSS_RUNTIME_EFUSE_BASE + 0x80)
#define RESP_MSG_ADDR			(BSS_RUNTIME_EFUSE_BASE + 0x200)
#define BST_RT_READY_MAGIC_NUM		(0xA5A5A5A5)
#define CMD_MSG_READY_MAGIC_NUM		(0x55555555)
#define RESP_MSG_READY_MAGIC_NUM	(0xAAAAAAAA)
#define MSG_WAIT_TIME			1000 //us
#define MAX_RESP_MSG_DELAY		5000000

#define EFUSE_MEMORY_SIZE		512
#define EFUSE_MSG_DATA_MAX_SIZE		256
#define EFUSE_READ_OP			0
#define EFUSE_WRITE_OP			1

#define MAX_RETRY_COUNT                 10

/* error codes */
#define ERROR_RESPONSE			-2
#define ERROR_TIMEOUT			-3
#define ERROR_INVALID_CERT_ID		-4
#define ERROR_UNSUPPORTED_CMD		-5
#define ERROR_INVALID_CMD		-6

typedef struct _msg_efuse {
	uint8_t rsv[2];
	uint8_t id;	/* msg id */
	uint8_t type;	/* msg type */
	uint16_t info;	/* addl info like heartbeat */
	uint8_t size;
	uint8_t data[EFUSE_MSG_DATA_MAX_SIZE];
	uint32_t checksum;
} MSG_EFUSE;

/* msg identifier */
enum msgId_efuse {
	/* efuse section */
	RAPTOR2_MSGID_EFUSE_LOCK = 0x1,
	RAPTOR2_MSGID_EFUSE_BOARD_TYPE,
	RAPTOR2_MSGID_EFUSE_SECUREMODE,
	RAPTOR2_MSGID_EFUSE_SHA_TYPE,
	RAPTOR2_MSGID_EFUSE_KAK1_BURN_STATUS,
	RAPTOR2_MSGID_EFUSE_KAK2_BURN_STATUS,
	RAPTOR2_MSGID_EFUSE_KAK1_REVOKE,
	RAPTOR2_MSGID_EFUSE_KAK2_REVOKE,
	RAPTOR2_MSGID_EFUSE_CSK_REVOKE,
	RAPTOR2_MSGID_EFUSE_KAK1_HASH,
	RAPTOR2_MSGID_EFUSE_KAK2_HASH,
	RAPTOR2_MSGID_EFUSE_MEMORY_READ,
	RAPTOR2_MSGID_EFUSE_JTAG_DISABLE_BIT,
	RAPTOR2_MSGID_EFUSE_BOOT_SPI_DISABLE_BIT,
	RAPTOR2_MSGID_EFUSE_CSHA_TYPE,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_BURN_STATUS,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_BURN_STATUS,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_REVOKE,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_REVOKE,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_HASH,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_HASH,
	RAPTOR2_MSGID_EFUSE_CUSTOMER_CSK_REVOKE,
	RAPTOR2_MSGID_EFUSE_SOC_ID,
	RAPTOR2_MSGID_EFUSE_WDT_BIT,
	RAPTOR2_MSGID_EFUSE_PROD_BIT,
	RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET,
	RAPTOR2_MSGID_EFUSE_SPI_ADDR_MODE_BIT,
	/* soc and other operations */
	RAPTOR2_MSGID_SOC_RESET = 0x1C,
	RAPTOR2_MSGID_TXU_PLL_CFG,
	RAPTOR2_MSGID_REG_READ,
	RAPTOR2_MSGID_REG_WRITE,
	RAPTOR2_MSGID_SOC_HEARTBEAT,
	/* All new ids before this */
	RAPTOR2_MSGID_EFUSE_INVALID
};

/* msg type */
enum msgType_efuse {
	RAPTOR2_MSGTYPE_READ = 0x1,
	RAPTOR2_MSGTYPE_WRITE,
	RAPTOR2_MSGTYPE_ACK,
	RAPTOR2_MSGTYPE_NACK,
	RAPTOR2_MSGTYPE_CHECKSUM_ERROR,
	RAPTOR2_MSGTYPE_OP_INVALID,
	RAPTOR2_MSGTYPE_DATA_MISMATCH,
	RAPTOR2_MSGTYPE_UNKNOWN_CMD
};

enum HeartBeat {
	STAGE_UBOOT = 1,
	STAGE_INITRD,
	STAGE_LINUX
};

static int get_efuse_bit(uint8_t msg_id);
static int set_efuse_bit(uint8_t msg_id);
static int get_csk_revoke_status(u32 cert_id, u32 csk_revoke_type);
static int set_csk_revoke_status(u32 cert_id, u32 csk_revoke_type);
static int efuse_bit(const char *bit_type, const char *cert_id, int op_mode);
static int efuse_show_memory(const char *str_offset, const char *str_size);
static int efuse_show_hash(const char *str_kak_id, const char *str_kak_size);
static int efuse_write_hash(int argc, char *const argv[]);
static int send_msg(uint8_t msg_id, uint8_t msg_type, uint16_t msg_info, u8 *buf, u8 buf_size);
static int get_response_msg(MSG_EFUSE *buf, int timeout);
#endif
