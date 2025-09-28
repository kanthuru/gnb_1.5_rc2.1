/*
 *
 * EdgeQ Inc.
 *
 * Raptor2 TX/RX Ring Data Structures &
 * Function Prototypes for WLAN Firmware
 *
 */

#include <net/mac80211.h>


typedef struct {
	uint64_t    ppdu_start_time_us;
	uint32_t    ppdu_id;
	uint32_t    status : 3, /** ok, ttl, xretry, qtaildrop, sta_inv etc */
		    mpdu_id : 8, /* MPDU ID on successful attempt */
		    num_tries : 5,
		    reserved_1 : 16;
} __packed tx_status_t;


typedef struct {
	uint8_t  *buf_vaddr;
	uint32_t buf_len;
	uint32_t desc_id;         /** MSB 8b: additional signaling possible */
	uint32_t sta_id;          /** STA this is destined to               */
	uint32_t mpdu        : 1, /** MPDU desc if set, MSDU desc otherwise */
		 mpdu_first  : 1, /** Add/modify MPDU hdr or mutable fields */
		 mpdu_last   : 1, /** Append MIC, FCS if offloaded          */
		 cpu_intr_id : 3, /** Tx completion interrupt cpu           */
		 reserved_1  : 26;
	uint32_t mpdufrags   : 4, /* number of fragments in mpdu */
		 fragnum     : 4, /* fragment number */
		 reserved    : 24;
	uint32_t tx_params_valid_mask;
#define TX_PARAMS_VALID_TID   BIT(0)
#define TX_PARAMS_VALID_PROTO BIT(1)
#define TX_PARAMS_VALID_AGGR  BIT(2)
#define TX_PARAMS_VALID_ENCR  BIT(3)
#define TX_PARAMS_VALID_RATE  BIT(4)
#define TX_PARAMS_VALID_TCLAS BIT(5)
	uint32_t tid     : 5,   /** QoS: 0-15, non-QoS: 16, Mgmt: 17         */
		 tcp     : 1,   /** proto:tcp if set                         */
		 udp     : 1,   /** proto:udp if set                         */
		 rtp     : 1,   /** port:rtp if set                          */
		 eapol   : 1,   /** proto:eapol if set                       */
		 no_aggr : 1,   /** singleton (mpdu or msdu desc)            */
		 no_encr : 1,   /** encr performed or not needed (mpdu desc) */
		 no_encap : 1,  /** no mpdu hdr encapsulation needed         */
		 num_rates : 2, /** valid only when no_aggr set              */
		 reserved_2: 18;
	/** Traffic class identifier. Ignore if 0, act when non-zero         */
	uint32_t classifier_id;
	/** Rates for a singleton MPDU/MMPDU                                 */
	uint32_t rates[4];
#define TX_PARAMS_RATE_MCS_MASK   0x0000F
#define TX_PARAMS_RATE_NSS_MASK   0x000F0
#define TX_PARAMS_RATE_PREAM_MASK 0x00F00
#define TX_PARAMS_RATE_TRIES_MASK 0x1F000
	union {
		/** WLAN firmware descriptor, opaque to host                     */
		tx_fw_desc_t    fw;
		tx_status_t     tx_status;
	};
} __packed tx_desc_t;

typedef struct {
	struct hlist_node hlist; /* hash linkage */
	uint32_t       desc_id;
	struct sk_buff *skb;
} tx_host_desc_t;


typedef struct ppdu_desc_cmn {
	uint32_t          radio_id;
	ppdu_class_t      ppdu_class;
	ppdu_preamble_t   ppdu_preamble;
	ppdu_gi_t         ppdu_gi;
	ppdu_sr_t         ppdu_sr;
	ppdu_txbf_t       ppdu_txbf;
	ppdu_bw_t         ppdu_bw;
	uint32_t          num_users;
	uint32_t          flags;        /* To determine FES success or failure */
#define PPDU_EXPECTED_RESP_NONE   0x01
#define PPDU_EXPECTED_RESP_CTS    0x02
#define PPDU_EXPECTED_RESP_ACK_BA 0x04
#define PPDU_EXPECTED_RESP_CBF    0x08
#define PPDU_EXPECTED_RESP_TBPPDU 0x10
#define PPDU_EXPECTED_RESP_MBA    0x20  /* Only in STA mode */
} ppdu_desc_cmn_t;

/* XXX Require convergence */
typedef struct psdu_desc {
	uint32_t    num_mpdus;
	uint32_t    num_bytes;       /* A-MPDU size (not counting delimiters) */
	uint32_t    num_null_delims; /* For min MPDU start spacing            */
	uint32_t    num_total_bytes; /* PSDU size (not including EOF pad)     */
	float       psdu_txtime_us;
	uint32_t    flags;
#define PSDU_FLAG_AMPDU         0x1
#define PSDU_FLAG_MULTI_TID
#define PSDU_FLAG_VHT_SINGLETON
#define PSDU_FLAG_EXPECTED_ACK
#define PSDU_FLAG_EXPECTED_BA

	list_head_t mpdu_q;
} psdu_desc_t;
/* The starting part of this struct and that of txop_sched_user_t must match
 * so that this struct can be easily populated from txop_sched_user_t
 */
typedef struct psdu_usr_desc {
	user_rate_t usr_rate;
	ru_size_t   ru_size;
	uint32_t    ru_idx;
	psdu_desc_t psdu;
} psdu_usr_desc_t;

/* MAX_TXBF_USERS is typically greater to support sound-only sequences */
PREPROCESSOR_ASSERT(MAX_MIMO_USERS <= MAX_TXBF_USERS);
typedef struct ppdu_desc {
	ppdu_desc_cmn_t   cmn;
	psdu_usr_desc_t   usr_desc[0];
} ppdu_desc_t;

typedef struct tx_psdu_usr_status {
	user_rate_t       usr_rate;
	ru_size_t         ru_size;
	uint32_t          ru_idx;
	uint32_t          num_mpdus_tried;
	uint32_t          num_bytes_tried;
	uint32_t          num_bytes_total;  /* including delimiters overhead */
	uint32_t          num_tcp_bytes_tried;
	uint32_t          num_udp_bytes_tried;
	/* ok fields valid only if ack_rcvd        */
	uint32_t          num_mpdus_ok;
	uint32_t          num_bytes_ok;
	uint32_t          num_tcp_bytes_ok;
	uint32_t          num_udp_bytes_ok;
	uint32_t          num_delims;
	uint32_t          ack_rcvd : 1,
			  num_tids : 4,
			  reserved : 27;
} tx_psdu_usr_status_t;

/* ppdu_status_t is generated from ppdu_desc_t */
typedef struct tx_ppdu_status {
	uint32_t          radio_id;
	ppdu_class_t      ppdu_class;
	ppdu_preamble_t   ppdu_preamble;
	ppdu_gi_t         ppdu_gi;
	ppdu_sr_t         ppdu_sr;
	ppdu_txbf_t       ppdu_txbf;
	ppdu_bw_t         ppdu_bw;
	uint32_t          ppdu_duration_us;
	uint32_t          fes_duration_us;
	int32_t           ack_rssi;

	uint32_t          num_users;
	tx_psdu_usr_status_t *usr_status;	// variable array depends on num_users
} tx_ppdu_status_t;

typedef struct {
	/* TXOP wait time = transmission time – schedule time */
	uint32_t wait_duration_us;
	/* ACQ encoded in MSB */
	uint32_t txop_id;
	uint32_t num_data_ppdus;
	uint32_t txop_duration_us;
	uint32_t status;
	uint32_t flags;
} __packed txop_status_t;


typedef struct {
	struct hlist_node hlist; /* hash linkage */
	uint32_t       desc_id;
	struct sk_buff *skb;
} rx_host_desc_t;

typdef struct ieee80211_rx_status rx_status_t;
typedef struct {
	uint8_t  *buf_vaddr;
	uint32_t buf_len;
	uint32_t desc_id;         /** MSB 8b: additional signaling possible */
	uint32_t mpdu : 1,
		 mpdu_first : 1,
		 mpdu_last  : 1,
		 msdu_frags  : 4,
		 msdu_fragnum  : 4,
		 union {
			 /** WLAN firmware descriptor, opaque to host */
			 rx_fw_desc_t    fw;
			 rx_status_t     rx_status;
		 };
} __packed rx_desc_t;



typedef struct cci_cmd_hdr {
	uint32_t cmd_id     : 16,
		 group_id   :  8,
		 reserved_0 :  6,
		 radio_id   :  2;  /* Will be deprecated after code maturity */
	uint32_t length     : 10,  /* In units of sizeof(uint32_t)           */
		 reserved_1 : 22;

} cci_cmd_hdr_t;


typedef struct cci_cmd_generic {
	cci_cmd_hdr_t cmd_hdr;
	uint32_t      value[1];
} cci_cmd_generic;


enum CCI_CMD_GROUP_SYSTEM {
	CCI_CMD_ID_SYSTEM_BOOT,
	CCI_CMD_ID_SYSTEM_RESET,
	CCI_CMD_ID_SYSTEM_HALT,
	CCI_CMD_ID_SYSTEM_MEM_DUMP,
	/* Configuration and Event Rings setup */
	CCI_CMD_ID_CFG_RINGS,
};

enum CCI_CMD_GROUP_RADIO{
	/* Channel and channel-width */
	CCI_CMD_ID_RADIO_CHANNEL,
	CCI_CMD_ID_RADIO_TX_CHAINMASK,
	CCI_CMD_ID_RADIO_RX_CHAINMASK,
	CCI_CMD_ID_RADIO_POWER_TABLE,
	CCI_CMD_ID_RADIO_POWER_LIMIT,
	CCI_CMD_ID_RADIO_CTL_TABLE,
	CCI_CMD_ID_RADIO_ANTENNA_GAIN,
	CCI_CMD_ID_DFS,
	/* Rx sensitivity */
	CCI_CMD_ID_RADIO_CCA_THRESHOLD,
	CCI_CMD_ID_RADIO_BSS_COLOR,
	/* Enable/disable Non-SRG OBSS PD Spatial Reuse */
	CCI_CMD_ID_RADIO_NON_SRG_OBSS_PD_SR,
	/* Non-SRG OBSS PD Max Threshold (-62 dBm to -82 dBm) */
	CCI_CMD_ID_RADIO_NON_SRG_OBSS_PD_MAX,
	CCI_CMD_ID_RADIO_SRG_OBSS_PD_MAX,
	CCI_CMD_ID_RADIO_SRG_OBSS_PD_MIN,
	/* Data path rings */
	CCI_CMD_ID_RADIO_TX_DATA_RINGS,
	CCI_CMD_ID_RADIO_RX_DATA_RINGS,
	/* Separate Mgmt path rings to prevent head-of-line blocking */
	CCI_CMD_ID_RADIO_TX_MGMT_RINGS,
	CCI_CMD_ID_RADIO_RX_MGMT_RINGS,
	CCI_CMD_ID_RADIO_MON_RINGS,
	/* AP WMM parameters */
	CCI_CMD_ID_RADIO_WMM,
	/* Tx/Rx frame types to be duplicated and forwarded to mon ring */
	CCI_CMD_ID_RADIO_MON_FILTER,
};

enum CCI_CMD_GROUP_VIF {
	/* Create VIF instance          */
	CCI_CMD_ID_VIF_CREATE,
	/* Start a VIF instance         */
	CCI_CMD_ID_VIF_START,
	/* Restart VIF on a new channel */
	CCI_CMD_ID_VIF_RESTART,
	/* Stop a VIF instance          */
	CCI_CMD_ID_VIF_STOP,
	/* Delete a VIF instance        */
	CCI_CMD_ID_VIF_DELETE,
	/* Shared key (WEP), Group key (WPA), IGTK (PMF) */
	CCI_CMD_ID_VIF_PARAM_KEY,
};

enum CCI_CMD_GROUP_STA {
	/* Add STA                      */
	CCI_CMD_ID_STA_ADD,
	/* Delete STA                   */
	CCI_CMD_ID_STA_DELETE,
	/* Pause TIDs on power-save entry */
	CCI_CMD_ID_STA_PAUSE,
	/* Resume TIDs on power-save exit */
	CCI_CMD_ID_STA_RESUME,
	/* Unique per-STA key (WEP/WPA)   */
	CCI_CMD_ID_STA_PARAM_KEY,
	/* BA state enable/disable, BA negotiated parameters for TID */
	CCI_CMD_ID_STA_PARAM_BA,
	/* Rate info. Num rates SU: 1, RU_SIZES: 6, MUMIMO: 7               */
	CCI_CMD_ID_STA_DATA_RATE,
	/* Operation mode change affecting rate/nss/bw */
	CCI_CMD_ID_STA_OP_MODE,
};

typedef struct cci_evt_hdr {
	uint32_t evt_id        : 16,
		 group_id      :  8,
		 reserved_0    :  8;
	uint32_t length        : 10,  /* In units of sizeof(uint32_t) */
		 reserved_1    : 22;
} cci_evt_hdr_t;

typedef struct cci_evt_generic {
	cci_evt_hdr_t evt_hdr;
	uint32_t      value[1];
} cci_evt_generic;



typedef void (*ring_notifier_fn_t)(uint32_t ring_id, uint32_t num_entries);
typedef enum {
	RING_ROLE_PRODUCER,
	RING_ROLE_CONSUMER,
} ring_role_t;


typedef struct {
	/** Ring size + 1                                 */
	uint32_t num_entries;
	/** Ring entry size                               */
	uint32_t entry_sz;
	/** Producer index                                */
	uint32_t head_idx;
	/** Consumer index                                */
	uint32_t tail_idx;
	/** Memory address of the ring                    */
	uint8_t  *base_addr;
	/** Aligned offset of the first ring entry        */
	uint32_t start_offset;
	/** To notify consumer that head has been updated */
	ring_notifier_fn_t consumer_notifier;
	/** To notify producer that tail has been updated */
	ring_notifier_fn_t producer_notifier;
	/** Used only when ring view is not part of ring  */
	ring_role_t role;
	/* CPU on which ring is located for Interrupt load balancing purposes */
	int cpuid;
} ring_t;

/*
 *
 *  Generic Ring Interface
 *
 */

/* Returns a free entry at head for producer and increments head (or NULL
 * If ring is full). Returns tail entry for consumer and increments tail
 * (or NULL if ring is empty). Send ring update notification if requested.
 */
void *ring_get_entry(ring_t *ring, ring_role_t role, bool notify);

/* Allocate ring in shared memory. The ring mgmt can optionally be allocated
 * in shared memory. Return base_addr or pointer to ring_t.
 */
void *ring_init(uint32_t ring_sz, uint32_t entry_sz, uint32_t align_sz, bool sm_ring_mgmt, int cpuid);

/*
struct tx_enqueue_ring {
	tx_desc_t *ring;
	int num_entries;
};

struct tx_status_ring {
	tx_desc_t *ring;
	int num_entries;
};

struct tx_acctg_ring {
	tx_ppdu_status_t *ring;
	int num_entries;
};

struct rx_enqueue_ring {
	rx_desc_t *ring;
	int num_entries;
};

struct rx_status_ring {
	rx_desc_t *ring;
	int num_entries;
};

struct rx_acctg_ring {
	rx_ppdu_status_t *ring;
	int num_entries;
};
*/

/*
 *  Firmware Specific Data Structures
 */
#define NUM_BUFPOOLS	5

typedef enum {
	BUFPOOL_SZ_2K	= 0,
	BUFPOOL_SZ_4K	= 1,
	BUFPOOL_SZ_8K	= 2,
	BUFPOOL_SZ_12K	= 3,
	BUFPOOL_SZ_16K	= 4,
} bufpool_sz_t;


struct r2wlan_tx {
	ring_t *tx_enqueue_ring;
	ring_t *tx_status_ring;
	ring_t *tx_acctg_ring;
};

struct r2wlan_rx {
	ring_t *rx_enqueue_ring;
	ring_t *rx_packet_ring;
	ring_t *rx_acctg_ring;
};

struct r2wlan_cci {
	ring_t *cci_cmd_ring;
	ring_t *cci_event_enqueue_ring;
	ring_t *cci_event_ring;
};

struct r2wlan {
	struct ieee80211_hw *hw;
	struct device *dev;

	struct r2wlan_tx tx;
	struct r2wlan_rx rx;
};

/*
 *
 *  Firmware Specific Ring Interface
 *
 */

/* Just one set of rings common systm-wide & radio-wide */
int cci_cmd_ring_init(struct r2wlan_cci *r2wcci, uint32_t ring_sz);
int cci_event_enqueue_ring_init(struct r2wlan_cci *r2wcci, uint32_t ring_sz);
int cci_event_ring_init(struct r2wlan_cci *r2wcci, uint32_t ring_sz);

/* TX Rings init: Will need to be called for each radio */
int tx_desc_enqueue_ring_init(struct r2wlan_tx *r2wtx, uint32_t ring_sz);
int tx_status_ring_init(struct r2wlan_tx *r2wtx, uint32_t ring_sz, int cpuid);
int tx_acctg_ring_init(struct r2wlan_tx *r2wtx, uint32_t ring_sz, int cpuid);

/* TX Rings init: Will need to be called for each radio */
int rx_desc_enqueue_ring_init(struct r2wlan_rx *r2wrx, uint32_t ring_sz);
int rx_packet_ring_init(struct r2wlan_rx *r2wrx, uint32_t ring_sz);
int rx_acctg_ring_init(struct r2wlan_rx *r2wrx, uint32_t ring_sz);

/* Host driver is Producer. Firmware is Consumer */
tx_desc_t *tx_enqueue_ring_get_entry(struct r2wlan_tx *r2wtx, bool notify);

/* Firmware is Producer. Host driver is Consumer */
int tx_status_ring_push_entry(struct r2wlan_tx *r2wtx, tx_desc_t *txdesc, bool notify);

/* Firmware is Producer. Host driver is Consumer */
int tx_acctg_ring_push_entry(struct r2wlan_tx *r2wtx, tx_ppdu_status_t *statusdesc, bool notify);

/* Host driver is Producer. Firmware is Consumer */
rx_desc_t *rx_enqueue_ring_get_entry(struct r2wlan_rx *r2wrx, bool notify);

/* Firmware is Producer. Host driver is Consumer */
int rx_packet_ring_push_entry(struct r2wlan_rx *r2wrx, rx_desc_t *rxdesc, bool notify);

/* Firmware is Producer. Host driver is Consumer */
int rx_acctg_ring_push_entry(struct r2wlan_rx *r2wrx, rx_ppdu_status_t *statusdesc, bool notify);

/* Host driver is Producer. Firmware is Consumer */
cci_cmd_generic *cci_cmd_ring_get_entry(struct r2wlan_cci *r2wcci, bool notify);

/* Host driver is Producer. Firmware is Consumer */
cci_evt_generic *cci_event_enqueue_ring_get_entry(struct r2wlan_cci *r2wcci, bool notify);

/* Firmware is Producer. Host driver is Consumer */
int *cci_event_ring_push_entry(struct r2wlan_cci *r2wcci, cci_evt_generic *cci_evt, bool notify);
