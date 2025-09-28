/*
 * EdgeQ Inc.
 *
 * Raptor2 Data Structures for Logger
 */

#include "../include/raptor2_su_common.h"

#define LOGGER_DEV_NAME			"r2logdev"
#define PPBUFSIZE			64*1024
#define PPBUF_AVBL			0x00000001
#define LOGGER_MEM_BASE_PHYS		0x401000000
#define LOGGER_MAX_PROCS		1
#define MAX_LOG_MSG_STR_LEN		80
#define MAX_NUM_LOG_MSG_PARAMS		9

enum log_rat {
	NR_LTE_LOG,
	WLAN_LOG
};


typedef struct st_logger_msg {
	/* Word aligned */
	uint32_t stmr_ts;	// STMR Timestamp

	union
	{
		struct
		{
			/* SFN (0 - 1023) and slot number (0 - 319) of the message.
			   Set to OTA SFN/slot number if not relevant */
			uint32_t sfn:10;
			uint32_t slot_num:9;
			/* Symbol number (0 - 13). Set to 0xF if not applicable */
			uint32_t symbol_num:4;
			/* Carrier index */
			uint32_t carrier_id:4;
			/* Module ID : OTX_CTRL, ORXC_CTRL, etc. */
			uint32_t module_id:5;
		} nr_lte;

		struct
		{
			uint32_t rsvd_1:5;
			uint32_t slot_num:9;
			/* Symbol number (0 - 1280). Set to 0xFFF if not applicable */
			uint32_t symbol_num:11;
			uint32_t wlan_instance:2;
			/* Module ID : OTX_CTRL, ORXC_CTRL, etc. */
			uint32_t module_id:5;
		} wlan;
	} logger_msg_word_2;

	union
	{
		struct
		{
			/* Currently ongoing SFN (0 - 1023) and slot number (0 - 319) */
			uint32_t ota_SFN:10;
			uint32_t ota_slot_num:9;
			/* Number of valid parameters */
			uint32_t num_params:4;
			/* SU Core ID */
			uint32_t core_id:6;
			uint32_t log_str:1;	/* Set if string based log */
			uint32_t rat:2;	/* RAT : NR, WLAN or LTE   */
		} nr_lte;

		struct
		{
			uint32_t rsvd_2:8;
			uint32_t ota_symbol_num:11;
			/* Number of valid parameters */
			uint32_t num_params:4;
			/* SU Core ID */
			uint32_t core_id:6;
			uint32_t log_str:1;	/* Set if string based log */
			uint32_t rat:2;	/* RAT : NR, WLAN or LTE   */
		} wlan;
	} logger_msg_word_3;

	/* Word aligned */
	union
	{
		uint8_t  	log_msg_str[MAX_LOG_MSG_STR_LEN];	// Log string
		uint32_t	log_msg_id;
	} log_msg;

	/* Word aligned */
	uint32_t params[MAX_NUM_LOG_MSG_PARAMS];	// Log params
} st_logger_msg_t;


#define NUM_LOG_MSGS	PPBUFSIZE/sizeof(st_logger_msg_t)


struct ppbuf {
	uint32_t flag0;
	uint32_t flag1;
	st_logger_msg_t buf0[NUM_LOG_MSGS];
	st_logger_msg_t buf1[NUM_LOG_MSGS];
};

struct engine_ppbuf_mapping {
	struct ppbuf ppu[PPU_NUM_SUS];
	struct ppbuf ppu_mxl[PPU_MXL_NUM_SUS];
	struct ppbuf ppu_crss[PPU_CRSS_NUM_SUS];
	struct ppbuf otrx_hsu[OTRX_HSU_NUM_SUS];
	struct ppbuf otrx_qsu[OTRX_QSU_NUM_SUS];
	struct ppbuf spu[SPU_NUM_SUS];
	struct ppbuf ecpri[ECPRI_NUM_SUS];
	struct ppbuf txu[TXU_NUM_SUS];
};

struct engine_ppbuf_mapping_all {
	struct ppbuf allbufs[TOTALSUCOUNT];
};

#ifdef __KERNEL__
struct log_mapping_access {
	spinlock_t lock;
	struct engine_ppbuf_mapping_all *epmap;
	u64 virtaddr;
	int size;
	uint32_t nprocs;
};
#endif
