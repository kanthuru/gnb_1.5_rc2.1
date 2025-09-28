#include <common.h>
#include <command.h>
#include <console.h>
#include <cpu_func.h>
#include <errno.h>
#include <asm/cache.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include "fuse-raptor.h"

static void efuse_clr_shm(void);

static int send_and_get_response_msg(u8 msg_id, uint8_t msg_type, uint16_t msg_info, u8 *buf,
				     u8 buf_size, MSG_EFUSE *msg)
{
	int ret, retry_count = 0;

	ret = send_msg(msg_id, msg_type, msg_info, buf, buf_size);
	while ((ret < 0) && (retry_count < MAX_RETRY_COUNT)) {
		ret = send_msg(msg_id, msg_type, msg_info, buf, buf_size);
		retry_count++;
	}

	if ((ret < 0) || (retry_count == MAX_RETRY_COUNT)) {
		printf(" Failed to send msg to bss\n");
		return ret;
	}

	retry_count = 0;
	ret = get_response_msg(msg, MAX_RESP_MSG_DELAY);
	while ((ret == ERROR_TIMEOUT) && (retry_count < MAX_RETRY_COUNT)) {
		ret = get_response_msg(msg, MAX_RESP_MSG_DELAY);
		retry_count++;
	}

	return ret;
}

static int get_efuse_bit(uint8_t msg_id)
{
	int ret;
	MSG_EFUSE msg;

	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_READ, 0, 0, 0, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf(" bit status:%d\n", msg.data[0]);
		} else {
			printf("response error:%d for msg_id:%d\n", msg.type, msg_id);
			return ERROR_RESPONSE;
		}
	}
	return ret;
}

static int get_efuse_field(uint8_t msg_id)
{
	int ret;
	MSG_EFUSE msg;
	int count = 0;

	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_READ, 0, 0, 0, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf("Field data: ");
			while (count < (msg.size)) {
				printf((msg_id == RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET) ? "%u"
						: "%c", msg.data[count]);
				count++;
			}
			printf("\r\n");
		} else {
			printf("response error:%d for msg_id:%d\n", msg.type, msg_id);
			return ERROR_RESPONSE;
		}
	}
	return ret;
}

static int set_efuse_bit(uint8_t msg_id)
{
	int ret;
	MSG_EFUSE msg;

	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_WRITE, 0, 0, 0, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf(" bit is set\n");
		} else {
			printf("!!ERROR:efuse bit burn failed, response error:%d for msg_id:%d\n",
					msg.type, msg_id);
			return ERROR_RESPONSE;
		}
	}

	return ret;
}

static int set_efuse_field(uint8_t msg_id, const char *data)
{
	MSG_EFUSE msg;
	int size = 0;
	u8 buf[32];
	bool isEmpty = false;
	int ret, i, j, data_size;

	if (data == NULL) {
		printf("Please enter value\n");
		isEmpty = true;
		return -EINVAL;
	}

	data_size = strlen(data);
	if ((msg_id == RAPTOR2_MSGID_EFUSE_SOC_ID) && (data_size > 16)) {
		printf("SOC ID can not be more than 16 character long\n");
		return CMD_RET_FAILURE;
	}

	for (i = 0; data[i] != '\0'; i++) {
		if (msg_id == RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET) {
			buf[i] = data[i] - '0';
			if ((buf[i] < 0) || (buf[i] > 3)) {
				printf("Incorrect value passed\n");
				printf("The wdt-timer configuration should be 0-3\n");
				return CMD_RET_FAILURE;
			}
		} else
			buf[i] = data[i];
	}

	size = i;

	printf("The field data in bytes is: ");
	for (j = 0; j < i; j++)
		printf((msg_id == RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET) ? "%d" : "%c", buf[j]);

	printf("\r\n");

	if (!isEmpty) {
		printf("\n!!WARNING: Programming fuses is an irreversible operation!\n"
				"\n proceed with fuse programming? <y/n>\n");

		if (!confirm_yesno()) {
			printf("Fuse programming aborted\n");
			return CMD_RET_FAILURE;
		}
	}
	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_WRITE, 0, buf, size, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf("input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf("field is programmed\n");
		} else if ((msg.id == msg_id) &&
				(msg.type == RAPTOR2_MSGTYPE_DATA_MISMATCH)) {
			printf("!!ERROR: Field program failed due to field data mismatch\n");
			printf("msg_id %d response error:%d\n", msg.id, msg.type);
			ret =  ERROR_RESPONSE;
		} else {
			printf("!!ERROR:efuse field burn failed, response error:%d for msg_id:%d\n",
					msg.type, msg_id);
			return ERROR_RESPONSE;
		}
	}
	return ret;
}

static int get_csk_revoke_status(u32 cert_id, u32 csk_revoke_type)
{
	MSG_EFUSE msg;
	u8 buf;
	int ret;
	uint8_t msg_id;

	if (csk_revoke_type)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_CSK_REVOKE;	/* Customer csk revokation */
	else
		msg_id = RAPTOR2_MSGID_EFUSE_CSK_REVOKE;

	buf = (u8)cert_id;
	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_READ, 0, &buf, 1, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf(" bit status:%d\n", msg.data[0]);
		} else {
			printf("response error:%d for msg_id:%d\n", msg.type, msg.id);
			ret =  ERROR_RESPONSE;
		}
	}

	return ret;
}

static int set_csk_revoke_status(u32 cert_id, u32 csk_revoke_type)
{
	MSG_EFUSE msg;
	u8 buf;
	int ret;
	uint8_t msg_id;

	if (csk_revoke_type)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_CSK_REVOKE;	/* Customer csk revokation */
	else
		msg_id = RAPTOR2_MSGID_EFUSE_CSK_REVOKE;

	buf = (u8)cert_id;
	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_WRITE, 0, &buf, 1, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf(" bit is set\n");
		} else {
			printf("!!ERROR: efuse bit burn failed,response error:%d for msg_id:%d\n",
					msg.type, msg.id);
			ret =  ERROR_RESPONSE;
		}
	}

	return ret;
}

static int efuse_bit(const char *bit_type,
		const char *cert_id, int op_mode)
{
	int ret;
	char *ep;
	u32 cert_number;
	u32 csk_revoke_type = 0;

	if (!strcmp(bit_type, "board")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_BOARD_TYPE);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_BOARD_TYPE);
	} else if (!strcmp(bit_type, "securemode")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_SECUREMODE);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_SECUREMODE);
	} else if (!strcmp(bit_type, "sha-type")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_SHA_TYPE);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_SHA_TYPE);
	} else if (!strcmp(bit_type, "csha-type")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_CSHA_TYPE);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_CSHA_TYPE);
	} else if (!strcmp(bit_type, "jtag_disable")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_JTAG_DISABLE_BIT);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_JTAG_DISABLE_BIT);
	} else if (!strcmp(bit_type, "boot_spi_disable")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_BOOT_SPI_DISABLE_BIT);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_BOOT_SPI_DISABLE_BIT);
	} else if (!strcmp(bit_type, "wdt_enable")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_WDT_BIT);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_WDT_BIT);
	} else if (!strcmp(bit_type, "prod_bit")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_PROD_BIT);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_PROD_BIT);
	} else if (!strcmp(bit_type, "spi_addr_mode")) {
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_SPI_ADDR_MODE_BIT);
		else
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_SPI_ADDR_MODE_BIT);
	} else if (!strcmp(bit_type, "hash-burn-status")) {

		if (op_mode == EFUSE_WRITE_OP) {
			printf("write operation is not permitted\n");
			printf(" Bit is set automatically after burning HASH\n");
			return CMD_RET_FAILURE;
		}

		if (cert_id == NULL ) {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		cert_number = simple_strtoul(cert_id, &ep, 10);
		if (ep == cert_id || *ep != '\0') {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		if ((cert_number > 3) || (cert_number < 0)) {
			printf("entered id is %d, but valid KAK certificate id are 0,1,2,3\n",
					cert_number);
			return ERROR_INVALID_CERT_ID;
		}

		if (cert_number == 0)
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK1_BURN_STATUS);
		else if (cert_number == 1)
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK2_BURN_STATUS);
		else if (cert_number == 2)
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_BURN_STATUS);
		else if (cert_number == 3)
			ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_BURN_STATUS);

	} else if (!strcmp(bit_type, "kak-revoke")) {
		if (cert_id == NULL ) {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		cert_number = simple_strtoul(cert_id, &ep, 10);
		if (ep == cert_id || *ep != '\0') {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		if ((cert_number > 3) || (cert_number < 0)) {
			printf("entered id is %d, but valid KAK certificate id are 0,1,2,3\n",
					cert_number);
			return ERROR_INVALID_CERT_ID;
		}
		if (cert_number == 0) {
			if (op_mode == EFUSE_WRITE_OP)
				ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK1_REVOKE);
			else
				ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK1_REVOKE);
		} else if (cert_number == 1) {
			if (op_mode == EFUSE_WRITE_OP)
				ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK2_REVOKE);
			else
				ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_KAK2_REVOKE);
		} else if (cert_number == 2) {
			if (op_mode == EFUSE_WRITE_OP)
				ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_REVOKE);
			else
				ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_REVOKE);
		} else if (cert_number == 3) {
			if (op_mode == EFUSE_WRITE_OP)
				ret = set_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_REVOKE);
			else
				ret = get_efuse_bit(RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_REVOKE);
		}
	} else if (!strcmp(bit_type, "edgeq-csk-revoke")) {
		if (!cert_id) {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		cert_number = simple_strtoul(cert_id, &ep, 10);
		if (ep == cert_id || *ep != '\0') {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		if ((cert_number > 31) || (cert_number < 0)) {
			printf("entered id is %d, but valid CSK certificate range is 0-31\n",
					cert_number);
			return ERROR_INVALID_CERT_ID;
		}
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_csk_revoke_status(cert_number, csk_revoke_type);
		else
			ret = get_csk_revoke_status(cert_number, csk_revoke_type);

	} else if (!strcmp(bit_type, "customer-csk-revoke")) {
		csk_revoke_type = 1;

		if (cert_id == NULL ) {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		cert_number = simple_strtoul(cert_id, &ep, 10);
		if (ep == cert_id || *ep != '\0') {
			printf("Pls enter certificate number\n");
			return -EINVAL;
		}
		if ((cert_number > 31) || (cert_number < 0)) {
			printf("entered id is %d, but valid CSK certificate range is 0-31\n",
					cert_number);
			return ERROR_INVALID_CERT_ID;
		}
		if (op_mode == EFUSE_WRITE_OP)
			ret = set_csk_revoke_status(cert_number, csk_revoke_type);
		else
			ret = get_csk_revoke_status(cert_number, csk_revoke_type);

	} else if (!strcmp(bit_type, "soc_id")) {
		if (op_mode == EFUSE_READ_OP)
			ret = get_efuse_field(RAPTOR2_MSGID_EFUSE_SOC_ID);
		else
			ret = set_efuse_field(RAPTOR2_MSGID_EFUSE_SOC_ID, cert_id);
	} else if (!strcmp(bit_type, "wdt-timer")) {
		if (op_mode == EFUSE_READ_OP)
			ret = get_efuse_field(RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET);
		else
			ret = set_efuse_field(RAPTOR2_MSGID_EFUSE_WDT_TIMER_OFFSET, cert_id);
	} else {

		printf("unknown bit-type:%s\n", bit_type);
		printf("supported bit-type strings are :"
				"board,securemode,sha-type,hash-burn-status,kak-revoke,csk-revoke\n"
				"jtag_disable,boot_spi_disable,soc_id,wdt_enable,"
				"prod_bit,spi_addr_mode\n");
		ret = -1;
	}
	return ret;
}

static int efuse_show_memory(const char *str_offset, const char *str_size)
{
	MSG_EFUSE msg;
	int ret, i;
	u8 buf[5];
	char *ep;
	u32 data, offset, size;

	offset  = (u32)simple_strtoul(str_offset, &ep, 16);
	if (ep == str_offset || *ep != '\0') {
		printf("Failed to get offset addr\n");
		return -EINVAL;
	}
	size  = (u32)simple_strtoul(str_size, &ep, 10);
	if (ep == str_size || *ep != '\0') {
		printf("Failed to get  size\n");
		return -EINVAL;
	}
	if (offset > EFUSE_MEMORY_SIZE) {
		printf("offset addr 0x%x is higher than efuse memory area:0x%x\n",
				offset, EFUSE_MEMORY_SIZE);
		return CMD_RET_FAILURE;
	}
	if (size > EFUSE_MSG_DATA_MAX_SIZE) {
		printf(" max. size can read for an instance is %d\n",
				EFUSE_MSG_DATA_MAX_SIZE);
		return CMD_RET_FAILURE;
	}
	if (((offset % 4) != 0) || ((size % 4) != 0)) {
		printf("offset addr, size should be 4-byte aligned \n");
		return -EINVAL;
	}

	buf[0] = (u8)(offset & 0xFF);
	buf[1] = (u8)((offset >> 8) & 0xFF);
	buf[2] = (u8)((offset >> 16) & 0xFF);
	buf[3] = (u8)((offset >> 24) & 0xFF);
	buf[4] = (u8)(size & 0xFF);

	ret = send_and_get_response_msg(RAPTOR2_MSGID_EFUSE_MEMORY_READ, RAPTOR2_MSGTYPE_READ,
			0, buf, 5, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_EFUSE_MEMORY_READ)
					&& (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			for (i = 0; i < msg.size; i = i + 4) {
				if ((i % 16) == 0)
					printf("\n0x%08x: ", offset + i);
				data = (u32)msg.data[i] | ((u32)(msg.data[i + 1]) << 8) | \
				((u32)(msg.data[i + 2]) << 16) | ((u32)(msg.data[i + 3]) << 24);
				printf("%08x ", data);
			}
			printf("\n");
		} else {
			printf("response error:%d for msg id:%d\n", msg.type, msg.id);
			ret =  ERROR_RESPONSE;
		}
	}
	return ret;
}

static int efuse_show_hash(const char *str_kak_id,
		const char *str_kak_size)
{
	MSG_EFUSE msg;
	int ret, i;
	uint8_t msg_id;
	u8 buf;
	char *ep;
	u32 cert_number, size;

	cert_number = simple_strtoul(str_kak_id, &ep, 10);
	if (ep == str_kak_id || *ep != '\0') {
		printf("Pls enter valid certificate number\n");
		return -EINVAL;
	}
	size  = simple_strtoul(str_kak_size, &ep, 10);
	if (ep == str_kak_size || *ep != '\0') {
		printf("Failed to get hash size\n");
		return -EINVAL;
	}
	if ((size != 32) && (size != 64)) {
		printf(" invalid hash size, hash size should be 32 or 64 bytes only\n");
		return -EINVAL;
	}
	if ((cert_number > 3) || (cert_number < 0)) {
		printf("entered id is %d, but valid KAK certificate id are 0,1,2,3\n",
				cert_number);
		return ERROR_INVALID_CERT_ID;
	}
	if (size > EFUSE_MSG_DATA_MAX_SIZE) {
		printf(" max. hash size is %d\n", EFUSE_MSG_DATA_MAX_SIZE);
		return CMD_RET_FAILURE;
	}

	if (cert_number == 0)
		msg_id = RAPTOR2_MSGID_EFUSE_KAK1_HASH;
	else if (cert_number == 1)
		msg_id = RAPTOR2_MSGID_EFUSE_KAK2_HASH;
	else if (cert_number == 2)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_HASH;
	else if (cert_number == 3)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_HASH;

	buf = (u8)size;
	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_READ, 0, &buf, 1, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			for (i = 0; i < msg.size; i++) {
				printf("%02x", msg.data[i]);
			}
			printf("\n");
		} else {
			printf("response error:%d for msg id:%d\n", msg.type, msg.id);
			ret = ERROR_RESPONSE;
		}
	}
	return ret;
}

static int efuse_write_hash(int argc, char *const argv[])
{
	MSG_EFUSE msg;
	u32 kak_id;
	u8 buf[EFUSE_MSG_DATA_MAX_SIZE] = {'\0'};
	uint8_t msg_id;
	u8 value;
	int confirmed = argc >= 4 && !strcmp(argv[3], "-y");
	int ret, i, j, hash_size, data_start_index;
	char *ep;
	char *sha_str;
	char c;

	if (confirmed) {
		kak_id = simple_strtoul(argv[4], &ep, 10);
		if (ep == argv[4] || *ep != '\0')
			return CMD_RET_USAGE;
		data_start_index = 5;
	} else {
		kak_id = simple_strtoul(argv[3], &ep, 10);
		if (ep == argv[3] || *ep != '\0')
			return CMD_RET_USAGE;
		data_start_index = 4;
	}
	if ((kak_id > 3) || (kak_id < 0)) {
		printf("entered id is %d, but valid KAK certificate id are 0,1,2,3\n",
				kak_id);
		return ERROR_INVALID_CERT_ID;
	}

	sha_str = argv[data_start_index];

	for (i = 0; sha_str[i] != '\0'; i++) {
		c  = sha_str[i];
		if(c >= '0' && c <= '9') {
			value = (c - '0');
		} else if (c >= 'A' && c <= 'F') {
			value = (10 + (c - 'A'));
		} else if (c >= 'a' && c <= 'f') {
			value = (10 + (c - 'a'));
		} else {
			printf ("invalid character in HASH string \n");
			return CMD_RET_FAILURE;
		}

		buf[(i/2)] += value << (((i + 1) % 2) * 4);
	}
	hash_size = i/2;
	printf(" the hash data entered in bytes is:\n");
	for (j = 0; j < hash_size; j++)
		printf("%02x ", buf[j]);
	printf("\n");

	if (!confirmed) {
		printf("!!WARNING: Programming fuses is an irreversible operation!\n"
				"\n proceed with fuse programming? <y/n>\n");

		if (!confirm_yesno()) {
			printf("Fuse programming aborted\n");
			return CMD_RET_FAILURE;
		}
	}

	if (kak_id == 0)
		msg_id = RAPTOR2_MSGID_EFUSE_KAK1_HASH;
	else if (kak_id == 1)
		msg_id = RAPTOR2_MSGID_EFUSE_KAK2_HASH;
	else if (kak_id == 2)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_HASH;
	else if (kak_id == 3)
		msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_HASH;

	ret = send_and_get_response_msg(msg_id, RAPTOR2_MSGTYPE_WRITE, 0, buf, (u8)hash_size, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == msg_id) && (msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf(" HASH is programmed successfully\n");

			/* set hash burn status bit */
			if (kak_id == 0)
				msg_id = RAPTOR2_MSGID_EFUSE_KAK1_BURN_STATUS;
			else if (kak_id == 1)
				msg_id = RAPTOR2_MSGID_EFUSE_KAK2_BURN_STATUS;
			else if (kak_id == 2)
				msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK1_BURN_STATUS;
			else if (kak_id == 3)
				msg_id = RAPTOR2_MSGID_EFUSE_CUSTOMER_KAK2_BURN_STATUS;
			ret = set_efuse_bit(msg_id);
		} else if ((msg.id == msg_id) &&
				(msg.type == RAPTOR2_MSGTYPE_DATA_MISMATCH)) {
			printf("!!ERROR: HASH program failed due to KAK hash mismatch,");
			printf("msg_id %d response error:%d\n", msg.id, msg.type);
			ret = ERROR_RESPONSE;
		} else {
			printf("!!ERROR: HASH program failed, msg_id %d response error:%d\n",
					msg.id, msg.type);
			ret =  ERROR_RESPONSE;
		}
	}

	return ret;
}

uint32_t get_checksum(MSG_EFUSE *msg_ptr)
{
	uint32_t checksum, sum = 0;
	int id;

	sum += msg_ptr->rsv[0];
	sum += msg_ptr->rsv[1];
	sum += msg_ptr->id;
	sum += msg_ptr->type;
	sum += msg_ptr->info;
	sum += msg_ptr->size;

	for (id = 0; id < EFUSE_MSG_DATA_MAX_SIZE; id++)
		sum += msg_ptr->data[id];

	checksum = ~sum;

	return checksum;
}

static int send_msg(uint8_t msg_id, uint8_t msg_type, uint16_t msg_info, u8 *buf, u8 buf_size)
{
	MSG_EFUSE msg;
	volatile MSG_EFUSE *tx_msg = (volatile MSG_EFUSE *)CMD_MSG_ADDR;
	volatile u64 *cmd_msg_ready = (volatile u64*)CMD_MSG_READY_ADDR;
	volatile u64 *resp_msg_ready = (volatile u64*)RESP_MSG_READY_ADDR;
	int i;
	uint32_t chk;

	if ((buf_size > 0) && (buf == NULL))
		return -1;

	msg.rsv[0] = 0;
	msg.rsv[1] = 0;
	msg.id = msg_id;
	msg.type = msg_type;
	msg.info = msg_info;
	if ((buf != NULL) && (buf_size > 0)) {
		for (i = 0; i < buf_size; i++)
			msg.data[i] = buf[i];
		msg.size = buf_size;
		/* init to zero of unused data */
		for (i = buf_size; i < EFUSE_MSG_DATA_MAX_SIZE; i++)
			msg.data[i] = 0;
	} else {
		msg.size = 0;
		/* init to zero of unused data */
		for (i = 0; i < EFUSE_MSG_DATA_MAX_SIZE; i++)
			msg.data[i] = 0;
	}
	chk = get_checksum(&msg);
	msg.checksum = chk;
	memcpy( (void *)tx_msg, (void *)&msg, sizeof(MSG_EFUSE));

	*cmd_msg_ready = (u64)CMD_MSG_READY_MAGIC_NUM;

	/* clear previous resp ready message */
	*resp_msg_ready = 0x0;

	return 0;
}

/* timeout in us */
static int get_response_msg(MSG_EFUSE *buf, int timeout)
{
	u32 msg_status;
	int iter, max_retry;
	MSG_EFUSE *rx_msg = (MSG_EFUSE *)RESP_MSG_ADDR;
	u64 *resp_msg_ready = (u64 *)RESP_MSG_READY_ADDR;

	if (buf == NULL)
		return -1;

	if (timeout < 0)
		return -1;

	max_retry = timeout / MSG_WAIT_TIME;
	if (max_retry == 0)
		max_retry = 1; //wait for minimum timeout value
	if ((timeout % MSG_WAIT_TIME) != 0)
		max_retry++;

	for (iter = 0; iter <  max_retry; iter++) {
		msg_status = (u32)(*resp_msg_ready);
		if (msg_status ==  RESP_MSG_READY_MAGIC_NUM)
			break;
		else
			udelay(MSG_WAIT_TIME);
	}

	if (iter == max_retry)
		return ERROR_TIMEOUT;

	buf->id = rx_msg->id;
	buf->type = rx_msg->type;
	buf->info = rx_msg->info;
	buf->size = rx_msg->size;
	buf->checksum = rx_msg->checksum;

	if (buf->size > 0)
		memcpy((void *)buf->data, (void *)rx_msg->data, buf->size);

	return 0;
}

static void efuse_clr_shm(void)
{
	volatile u64 *resp_msg_ready = (volatile u64*)RESP_MSG_READY_ADDR;
	volatile u64 *cmd_msg_ready = (volatile u64*)CMD_MSG_READY_ADDR;

	*resp_msg_ready = 0;
	*cmd_msg_ready = 0;
	memset((void *)CMD_MSG_ADDR, 0, sizeof(MSG_EFUSE));
	memset((void *)RESP_MSG_ADDR, 0, sizeof(MSG_EFUSE));
}


static int do_efuse_raptor(struct cmd_tbl *cmdtp, int flag, int argc,
		char *const argv[])
{
	u32 bss_rt_ready;
	volatile u64 *bss_rt_ptr = (volatile u64*)BSS_RT_READY_ADDR;
	const char *op = argc >= 2 ? argv[1] : NULL;
	const char *cert_id = argc == 5 ? argv[4] : NULL;
	int ret;

	if (argc < 4)
		return CMD_RET_USAGE;

	bss_rt_ready = (u32)(*bss_rt_ptr);
	if (bss_rt_ready != BST_RT_READY_MAGIC_NUM) {
		printf("runtime image is not running on BSS,"
				"read magic num: 0x%x\n", bss_rt_ready);
		return CMD_RET_FAILURE;
	}
	efuse_clr_shm();

	if (!strcmp(op, "read")) {
		if (!strcmp(argv[2], "bit")) {
			ret = efuse_bit(argv[3], cert_id, EFUSE_READ_OP);
			if (ret <  0)
				return CMD_RET_FAILURE;

		} else if (!strcmp(argv[2], "memory")) {
			if (argc < 5) {
				printf("usage: efuse read memory offset_addr size\n");
				printf("offset_addr in hex, size in decimal\n");
				return CMD_RET_FAILURE;
			}
			ret = efuse_show_memory(argv[3], argv[4]);
			if (ret <  0)
				return CMD_RET_FAILURE;

		} else if (!strcmp(argv[2], "hash")) {
			if (argc < 5) {
				printf("usage: efuse read hash hash-id size\n");
				return CMD_RET_FAILURE;
			}
			ret = efuse_show_hash(argv[3], argv[4]);
			if (ret <  0)
				return CMD_RET_FAILURE;
		} else if (!strcmp(argv[2], "field")) {
			ret = efuse_bit(argv[3], cert_id, EFUSE_READ_OP);
			if (ret <  0)
				return CMD_RET_FAILURE;
		} else {
			printf("unknown read operation command:%s\n", argv[2]);
			return CMD_RET_FAILURE;
		}

	} else if (!strcmp(op, "program")) {
		if (!strcmp(argv[2], "bit")) {
			ret = efuse_bit(argv[3], cert_id, EFUSE_WRITE_OP);
			if (ret <  0)
				return CMD_RET_FAILURE;
		} else if (!strcmp(argv[2], "memory")) {
			printf("memory write operation is not permitted\n");
			printf("Pls use bit operatin to write\n");
			return CMD_RET_FAILURE;

		} else if (!strcmp(argv[2], "hash")) {
			if (argc < 5) {
				printf("usage: efuse write hash hash-id hash-string \n");
				return CMD_RET_FAILURE;
			}
			ret = efuse_write_hash(argc, argv);
			if (ret <  0)
				return CMD_RET_FAILURE;
		} else if (!strcmp(argv[2], "field")) {
			ret = efuse_bit(argv[3], cert_id, EFUSE_WRITE_OP);
			if (ret <  0)
				return CMD_RET_FAILURE;
		} else {
			printf("unknown program operation command:%s\n", argv[2]);
			return CMD_RET_FAILURE;
		}

	} else {
		return CMD_RET_USAGE;

	}
	return CMD_RET_SUCCESS;
}

static int raptor_soc_reset(void)
{
	MSG_EFUSE msg;
	u32 bss_rt_ready;
	volatile u64 *bss_rt_ptr = (volatile u64*)BSS_RT_READY_ADDR;
	int ret;
	int i;

	bss_rt_ready = (u32)(*bss_rt_ptr);
	if (bss_rt_ready != BST_RT_READY_MAGIC_NUM) {
		printf("runtime image is not running on BSS,"
				"read magic num: 0x%x\n", bss_rt_ready);
		return CMD_RET_FAILURE;
	}
	efuse_clr_shm();

	ret = send_msg(RAPTOR2_MSGID_SOC_RESET, RAPTOR2_MSGTYPE_WRITE, 0, 0, 0);
	if (ret < 0) {
		printf(" Failed to send msg to bss \n");
		for (i = 0; i <= MAX_RETRY_COUNT; i++) {
			ret = send_msg(RAPTOR2_MSGID_SOC_RESET, RAPTOR2_MSGTYPE_WRITE, 0, 0, 0);
			if (ret < 0)
				printf("retrying to send reset message to bss runtime\n");
			else if ((ret < 0) && (i == MAX_RETRY_COUNT))
				return ret;
		}
	}

	ret = get_response_msg(&msg, MAX_RESP_MSG_DELAY);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_SOC_RESET) &&
				(msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf("The soc reset command is received by BSS\n");
		} else {
			printf("!!ERROR: Failed to process the command resp err:%d\n", msg.type);
			return ERROR_RESPONSE;
		}
	}	
	return CMD_RET_SUCCESS;
}

int raptor_soc_heartbeat(u8 wdt_state, u8 config)
{
	int ret;
	MSG_EFUSE msg;
	u32 bss_rt_ready;
	uint8_t msg_type;
	uint16_t msg_info;

	volatile u64 *bss_rt_ptr = (volatile u64 *)BSS_RT_READY_ADDR;

	bss_rt_ready = (u32)(*bss_rt_ptr);
	if (bss_rt_ready != BST_RT_READY_MAGIC_NUM) {
		printf("runtime image is not running on BSS,read magic num: 0x%x\n",
				bss_rt_ready);
		return CMD_RET_FAILURE;
	}
	efuse_clr_shm();
	msg_type = RAPTOR2_MSGTYPE_WRITE;
	msg_info = STAGE_UBOOT;

	if (wdt_state == 1)
		msg_info |= (wdt_state << 3);
	else if ((wdt_state == 2) && ((config > 1) || (config < 15)))
		msg_info |= (config << 4);

	ret = send_and_get_response_msg(RAPTOR2_MSGID_SOC_HEARTBEAT, msg_type, msg_info,
			0, 0, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf("Input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_SOC_HEARTBEAT) &&
				(msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			if (wdt_state == 1)
				printf("heartbeat is enabled\n");
			else if (wdt_state == 2)
				printf("heartbeat timer is configured for %u seconds\n", config);
			else
				printf("heartbeat is disabled\n");
		} else {
			printf("!!ERROR: Failed to process the command. resp err:%d\n", msg.type);
			return ERROR_RESPONSE;
		}
	}
	return CMD_RET_SUCCESS;
}

int raptor_config_txu_pll(void)
{
	MSG_EFUSE msg;
	int ret;

	ret = send_and_get_response_msg(RAPTOR2_MSGID_TXU_PLL_CFG, RAPTOR2_MSGTYPE_WRITE, 0, 0,
			0, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf(" input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_TXU_PLL_CFG) &&
				(msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			//printf("  The txu pll cfg command is received by BSS\n");
		} else {
			printf("!!ERROR: Failed to process the command\n");
			return ERROR_RESPONSE;
		}
	}
	return CMD_RET_SUCCESS;
}

int bss_reg_read(int argc, char *const argv[])
{
	char *ep;
	u32 addr;
	u32 data;
	u8 buf[4];
	u8 ret;
	MSG_EFUSE msg;

	if (argc < 3)
		return CMD_RET_USAGE;

	addr = simple_strtoul(argv[2], &ep, 16);
	if (ep == argv[2] || *ep != '\0') {
		printf("Failed to get offset addr\n");
		return -EINVAL;
	}

	buf[0] = (u8)(addr & 0xFF);
	buf[1] = (u8)((addr >> 8) & 0xFF);
	buf[2] = (u8)((addr >> 16) & 0xFF);
	buf[3] = (u8)((addr >> 24) & 0xFF);

	ret = send_and_get_response_msg(RAPTOR2_MSGID_REG_READ, RAPTOR2_MSGTYPE_READ, 0, buf,
			4, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf("Input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_REG_READ) &&
				(msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			data = (u32)msg.data[0] | ((u32)(msg.data[1]) << 8) |
				((u32)(msg.data[2]) << 16) | ((u32)(msg.data[3]) << 24);
			printf("%08x\n", data);
		} else {
			printf("ERROR: Failed to read BSS Reg, response error:%d\n", msg.type);
			ret =  ERROR_RESPONSE;
		}
	}
	return CMD_RET_SUCCESS;
}

int bss_reg_write(int argc, char *const argv[])
{
	char *ep;
	u32 addr;
	u32 value;
	u8 buf[8] = {'\0'};
	u8 ret;
	MSG_EFUSE msg;

	if (argc < 4)
		return CMD_RET_USAGE;

	addr = simple_strtoul(argv[2], &ep, 16);
	if (ep == argv[2] || *ep != '\0') {
		printf("Failed to get offset addr\n");
		return -EINVAL;
	}

	value = simple_strtoul(argv[3], &ep, 16);
	if (ep == argv[3] || *ep != '\0') {
		printf("Failed to get offset addr\n");
		return -EINVAL;
	}

	for (int i = 0; i < 4; i++)
		buf[i] = (addr >> i * 8);
	for (int i = 0; i < 4; i++)
		buf[i + 4] = (value >> i * 8);

	ret = send_and_get_response_msg(RAPTOR2_MSGID_REG_WRITE, RAPTOR2_MSGTYPE_WRITE, 0, buf,
			8, &msg);
	if (ret == ERROR_TIMEOUT) {
		printf("Timeout error: %d\n", ret);
	} else if (ret == -1) {
		printf("Input msg buffer is null\n");
	} else {
		if ((msg.id == (uint8_t)RAPTOR2_MSGID_REG_WRITE) &&
				(msg.type == (uint8_t)RAPTOR2_MSGTYPE_ACK)) {
			printf("Value is written on BSS Reg\n");
		} else {
			printf("ERROR: Failed to write value on BSS Reg, response error:%d\n",
											msg.type);
			ret =  ERROR_RESPONSE;
		}
	}
	return CMD_RET_SUCCESS;
}

static int do_soc_heartbeat_disable(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = 0;
	char *ep;
	u8 wdt_state, config = 0;
	u32 length;
	const char *str;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (strcmp(argv[0], "heartbeat"))
		goto heartbeat_command_error;

	length = strlen(argv[1]);
	str = argv[1];

	if ((length > 1) || !((str[0] == '0') || (str[0] == '1') || (str[0] == '2')))
		goto heartbeat_value_error;

	wdt_state = simple_strtoul(argv[1], &ep, 10);

	if ((argc == 3) && (wdt_state == 2)) {
		config = simple_strtoul(argv[2], &ep, 10);
		length = strlen(argv[2]);
		if ((length > 2) || ((config != 3) && (config != 5) && (config != 10) &&
					(config != 15)))
			goto heartbeat_configuration_value_error;
	} else if ((argc == 3) && !(wdt_state == 2))
		goto heartbeat_configuration_error;
	else if ((argc != 3) && (wdt_state == 2))
		goto heartbeat_configuration_value_error;
	else
		config = 0;

	ret = raptor_soc_heartbeat(wdt_state, config);

	return ret;

heartbeat_configuration_value_error:
	printf("Incorrect heartbeat configuration value passed\n");
	printf("Watchdog configuration supported values are: 3, 5, 10, 15\n");
	return CMD_RET_USAGE;

heartbeat_value_error:
	printf("Incorrect value passed\n");
	return CMD_RET_USAGE;

heartbeat_command_error:
	printf("Incorrect command passed\n");
	return CMD_RET_USAGE;

heartbeat_configuration_error:
	printf(" Heartbeat configuration value can be set only in configure mode[2]\n");
	return CMD_RET_USAGE;
}

static int do_bss_reg_access(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret = 0;

	if (argc < 3)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "read"))
		ret = bss_reg_read(argc, argv);
	if (!strcmp(argv[1], "write"))
		ret = bss_reg_write(argc, argv);

	return ret;
}

U_BOOT_CMD(
		efuse, 21, 0, do_efuse_raptor,
		"efuse command",
		"efuse [read/program] [bit/memory/hash] "
			"[bit_type/offset_addr/cert_id] [size]\n"
		"efuse read/program bit "
			"[board/securemode/sha-type/csha-type/hash-burn-status/kak-revoke/csk-revoke]\n"
			"[jtag_disable/boot_spi_disable/wdt_enable/prod_bit/spi_addr_mode]\n"
		"efuse read bit hash-burn-status kak_id[0-3]\n"
		"efuse read/program bit kak-revoke kak_id[0-3]\n"
		"efuse read/program bit edgeq-csk-revoke csk_id[0-31]\n"
		"efuse read/program bit customer-csk-revoke csk_id[0-31]\n"
		"efuse read memory offset_addr(hex) size(decimal)\n"
		"efuse read hash kak_id[0-3] size[32/64]\n"
		"efuse program hash kak_id[0-3] hash-string\n"
		"kak_id[0-1] edgeq specific kak ids\n"
		"kak_id[2-3] customer specific kak ids\n"
		"efuse read field [soc_id/wdt-timer]\n"
		"efuse program field soc_id [soc_id]\n"
		"efuse program field wdt-timer [timer_value]"
	  );

U_BOOT_CMD(
		bss_reg,    4,    0,	do_bss_reg_access,
		"Read/Write value on bss register",
		"bss_reg <read> <addr>\n"
		"bss_reg <write> <addr> <value>\n"
	  );

U_BOOT_CMD(
	heartbeat, 3, 0, do_soc_heartbeat_disable,
	"Enable/Disable the watchdog timer",
	"0 - to disable, 1 - to enable, 2 - to configure\n"
	"heartbeat state[0-1]\n"
	"heartbeat state[2] config[3/5/10/15]\n"
);

void reset_cpu(ulong addr)
{
	raptor_soc_reset();
	wfi();
}
