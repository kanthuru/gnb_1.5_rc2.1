/*
* Copyright (C) 2019-2022 EDGECOMPUTE Corporation.  All rights reserved.
*
* This source code and/or documentation ("Licensed Deliverables") are
* subject to EDGECOMPUTE intellectual property rights under U.S. and
* international Copyright laws.
*
* See file <LICENSE> under the project root for full license details.
*/

#ifndef _CCI_H
#define _CCI_H

//#if !defined(__LINUX__)
//#include <stdint.h>
//#endif
//#include "eq_ring.h"
//#include "hif_tx.h"
//#include "ppdu_types.h"

/*
 * @brief CCI version
 */
#define CCI_VER_MAJOR               (0x01)      /* Major 1(~99) */
#define CCI_VER_MINOR               (0x01)      /* Minor 1(~99) */
#define CCI_VERSION                 ((CCI_VER_MAJOR << 8) | CCI_VER_MINOR)
#define CCI_CB_MAGIC                (0xC1CB0000 | CCI_VERSION)

/*=== Portability */
#define CCI_SYS_DDR_ADDR            0x400000000
#define CCI_SYS_DDR_SIZE            0x200000000
#define CCI_CB_BOOT_ADDR            0x41C000000 /* 4KB shared memory */

#define CCI_IRQ_NUM_START           205          /* assigned IRQ(GIC SPI) number */
#define CCI_IRQ_NUM_END             220          /* reserved IRQ(GIC SPI) number */

/* Checking the structure that the size is fixed by xxx_SIZE
   Usage: CCI_CHK_STRUCT_SIZE(sizeof(xxx_t) % xxx_SIZE);
          If got compile error then check the structure again */
#define CCI_CHK_STRUCT_SIZE(sizeof_t_mod_size) ((void)sizeof(char[1 - 2*!!(sizeof_t_mod_size)]))
/*=== Portability - End */

/*=== System Level Config */
#define CCI_MAX_RADIO               3
#define CCI_MAX_VIF                 16
#define CCI_MAX_STA                 512
#define CCI_MAX_HIF_RING            48

/* Soft register size */
#define CCI_CB_CFG_SIZE             1024
#define CCI_CB_STS_SIZE             4096

/* CCI command and event ring constants */
#define CCI_RING_BUF_ELEM_SIZE      128     /* elem_sz: Can be optimized to 64B */
#define CCI_RING_ENTRY_NUM          32      /* each ring size 4KB (32 * 128) */

/* CCI constants */
#define CCI_TLV_TYPE_LEN_SIZE       4       /* uint16_t type and len in cci_comm_tlv_t */

#define CCI_MAX_HECAP_PHY_SIZE      3
#define CCI_MAX_CMD_RING_ENTRIES    10

#define CCI_MAX_SSID_LEN            32      /* For beacon generation, IEEE80211_MAX_SSID_LEN<ieee80211.h> */
#define CCI_ETH_ALEN                6       /* ETH_ALEN<if_ether.h> */  // shall remove ADDR_LENGTH in common_ds.h

#define CCI_MAX_TKIP_KEY            4
#define CCI_MAX_CCMP_KEY            4
#define CCI_MAX_GCMP_KEY            4
#define CCI_CCMP_PN_LEN             6
#define CCI_CCMP_HDR_LEN            8
#define CCI_CCMP_MIC_LEN            8

typedef enum Ecci_irq_num {
    CCI_IRQ_RADIO_0_RX_DESC_CPU_0 = CCI_IRQ_NUM_START, /* 205 */
    CCI_IRQ_RADIO_0_RX_DESC_CPU_1,
    CCI_IRQ_RADIO_0_RX_DESC_CPU_2,
    CCI_IRQ_RADIO_0_RX_DESC_CPU_3,
    CCI_IRQ_RADIO_0_RX_ACCT,
    CCI_IRQ_RADIO_0_TX_DESC,        /* From host */
    CCI_IRQ_RADIO_0_TX_STS_CPU_0,
    CCI_IRQ_RADIO_0_TX_STS_CPU_1,
    CCI_IRQ_RADIO_0_TX_STS_CPU_2,
    CCI_IRQ_RADIO_0_TX_STS_CPU_3,
    CCI_IRQ_RADIO_0_TX_ACCT,
    CCI_IRQ_CMD,                    /* From host */
    CCI_IRQ_EVT,
    CCI_IRQ_FATAL_ERR,
    CCI_IRQ_MAX = CCI_IRQ_NUM_END
}Ecci_irq_num_t;


typedef enum
{
    ACQ_BEACON,
    ACQ_MCAST,
    ACQ_MGMT,
    ACQ_VO,
    ACQ_VI,
    ACQ_BE,
    ACQ_BK,
    MAX_NUM_ACQ,
}e_wlan_acq_t;

/*
 * @enum Ecci_hif_ring_id_t
 * @brief HIF ring ID to index eq_ring_ctxt_t array
 * This ID is defined for finding the corresponding eq_ring context by:
 *     cci_cb_config_t.hif_ring_base + CCI_HIF_RING_XXX
 * 
 */
typedef enum Ecci_hif_ring_id {
    CCI_HIF_RING_RX_BUF_POOL_0,             /* Free Rx buffer pool ID 0-3 per size */
    CCI_HIF_RING_RADIO_0_RX_DESC_POOL,      /* Free Rx descriptor pool */
    CCI_HIF_RING_RADIO_0_RX_DESC_CPU_0,
    CCI_HIF_RING_RADIO_0_RX_DESC_CPU_1,
    CCI_HIF_RING_RADIO_0_RX_DESC_CPU_2,
    CCI_HIF_RING_RADIO_0_RX_DESC_CPU_3,
    CCI_HIF_RING_RADIO_0_RX_ACCT,
    CCI_HIF_RING_RADIO_0_TX_DESC,        /* From host */
    CCI_HIF_RING_RADIO_0_TX_STS_CPU_0,
    CCI_HIF_RING_RADIO_0_TX_STS_CPU_1,
    CCI_HIF_RING_RADIO_0_TX_STS_CPU_2,
    CCI_HIF_RING_RADIO_0_TX_STS_CPU_3,
    CCI_HIF_RING_RADIO_0_TX_ACCT,
    CCI_HIF_RING_CMD,               /* From host */
    CCI_HIF_RING_EVT,
    CCI_HIF_RING_ID_MAX = CCI_MAX_HIF_RING
}Ecci_hif_ring_id_t;

typedef enum Ecci_service_bitmap {
    CCI_MAX_SERVICE,
}Ecci_service_bitmap_t;

typedef enum Ecci_cipher_suite 
{
    CCI_CIPHER_SUITE_NULL,
    CCI_CIPHER_SUITE_WEP40,
    CCI_CIPHER_SUITE_WEP104,
    CCI_CIPHER_SUITE_TKIP,
    CCI_CIPHER_SUITE_CCMP, 
    CCI_CIPHER_SUITE_GCMP 
} Ecci_cipher_suite_t;

/* Security key length */
typedef enum Ecci_key_len {
	CCI_KEY_LEN_WEP40 = 5,
	CCI_KEY_LEN_WEP104 = 13,
	CCI_KEY_LEN_CCMP = 16,
	CCI_KEY_LEN_CCMP_256 = 32,
	CCI_KEY_LEN_TKIP = 32,
	CCI_KEY_LEN_AES_CMAC = 16,
	CCI_KEY_LEN_SMS4 = 32,
	CCI_KEY_LEN_GCMP = 16,
	CCI_KEY_LEN_GCMP_256 = 32,
	CCI_KEY_LEN_BIP_CMAC_256 = 32,
	CCI_KEY_LEN_BIP_GMAC_128 = 16,
	CCI_KEY_LEN_BIP_GMAC_256 = 32,
	CCI_KEY_LEN_MAX = 32
} Ecci_key_len_t;

/*=== System Level Config - End */

#define MAX_NUM_GEN_PURPOSE_REGS 31
#define MAX_FW_VERSION_SIZE      100
#define MAX_THREAD_NAME_SIZE     100
#define MAX_FILE_NAME_SIZE       100
#define MAX_REASON_STR_SIZE      100

typedef enum Ecci_fatal_error_reason {
    /** Generic CPU exception, not covered by other codes */
    CCI_FATAL_ERR_CPU_EXCEPTION,
    /** Unhandled hardware interrupt */
    CCI_FATAL_ERR_SPURIOUS_IRQ,
    /** Faulting context overflowed its stack buffer */
    CCI_FATAL_ERR_STACK_CHK_FAIL,
    /** Moderate severity software error */
    CCI_FATAL_ERR_KERNEL_OOPS,
    /** High severity software error */
    CCI_FATAL_ERR_KERNEL_PANIC
} Ecci_fatal_error_reason_t;

typedef struct fw_thread_info {
    char thread_name[MAX_THREAD_NAME_SIZE];
    char file_name[MAX_FILE_NAME_SIZE];
    uint32_t line_num;
    uint64_t stack_dump_start_addr;
    uint64_t stack_dump_end_addr;
} fw_thread_info_t;

typedef struct fw_fatal_error_info {
    Ecci_fatal_error_reason_t reason;
    char reason_str[MAX_REASON_STR_SIZE];
    uint32_t cpu_id;	
} fw_fatal_error_info_t;

typedef struct cpu_reg_info {
    uint64_t reg_dump[MAX_NUM_GEN_PURPOSE_REGS];    
    uint64_t sp;
    uint64_t pc;
    uint64_t cpsr;
} cpu_reg_info_t;

typedef struct cci_crash_dump_info {
    Ecci_irq_num_t irq_num;
    char fw_version[MAX_FW_VERSION_SIZE];
    fw_thread_info_t thread_info;
    fw_fatal_error_info_t error_msg;
    cpu_reg_info_t cpu_reg;
} cci_crash_dump_info_t;

#if 0
/*=== CCI Control Block */
/*
 * @struct cci_cb_boot_t
 * @brief CCI bootstrap information
 * This structure is defined for passing the loacation of CCI Control Block.
 * Host shall fill up the CCI command and event ring information first,
 * then write CCI_CB_MAGIC to let MAC start self-initialization.
 * The information is located at CCI_CB_BOOT_ADDR.
 * 
 */

typedef struct cci_cb_boot_t
{
    uint32_t    magic;              /* Shall match with CCI_CB_MAGIC[31:16] */
    uint32_t    cb_sz;              /* CCICB size in bytes */
    uint64_t    cb_base_addr;       /* CCICB base physical address(PA) pointing to cci_cb_config_t */
} cci_cb_boot_t;

/*
 * @struct cci_cb_config_t
 * @brief WLAN MAC global configuration
 * This structure is defined for setting the top-level configuration of WLAN MAC.
 * The information is located at the beginning of cci_cb_boot_t.cb_base_addr.
 * 
 */
typedef struct cci_cb_config_t
{
    eq_ring_ctxt_t *hif_ring_base;  /* base PA of hif_ring[CCI_MAX_HIF_RING] */
    uint32_t op_mode;               /* System Level Operation Mode set by TBD definitions */
    uint32_t tx_stop;               /* 1 bit per [bss_id] to stop all TX frames for the corresponding VIF */

    uint32_t bcn_stop;              /* Same bit control map to stop TX beacons when tx_stop = 0 */
    uint16_t mgmt_mac_tx;           /* TX Management frame generation (1 << mgmt_Subtype) by 0: Host, 1: MAC */
    uint16_t mgmt_mac_rx;           /* RX Management frame processing (1 << mgmt_Subtype) by 0: Host, 1: MAC */
    uint32_t rsvd_cfg_1[2];         /* Make 16B QW */
    
    uint8_t rsvd[CCI_CB_CFG_SIZE - (16 * 2)];
} cci_cb_config_t;

/**
 * @struct cci_cb_status_t
 * @brief WLAN MAC global status
 * This structure is defined for reporting the top-level status of WLAN MAC.
 */
typedef struct cci_cb_status_t
{
    volatile uint32_t  heartbeat;   /* alive */
    uint32_t op_sts;                /* System Level Operation Status set by TBD definitions */
    uint32_t svc_bm;                /* service bitmap */
    uint32_t rsvd0;                 /* reserved in QWORD[0] */

    uint32_t ver_info[4];           // report [module] version control

    /* Important structure pointers */
    uint8_t  num_radios;            /** Max number of radios */
    uint8_t  num_vifs;              /** Max number of vifs that can be supported */
    uint16_t num_stas;              /** Max number of stations that can be connected at the same time */
    uint8_t  radio_en_bm;           /** Bitmap for enabed radios. b0(radio 0) ~ bn(radio max-1) */
    uint8_t  num_activated_vifs;    /** Number of created vifs */
    uint16_t num_activated_stas;    /** Number of associated stations */
    uint32_t rsvd2[2];              /* reserved in QWORD[2] */
    
    uint64_t vif_base_addr;         // TBD: Do we need full 64b addr?
    uint64_t sta_base_addr;

    /* Capability Info */

    /* Statistics */
    /* FW Crash dump info */
    cci_crash_dump_info_t crash_dump;
  
    uint8_t rsvd[CCI_CB_STS_SIZE - ((16 * 4) + sizeof(cci_crash_dump_info_t))];
} cci_cb_status_t;

/**
 * @struct cci_cb_t
 * @brief WLAN MAC global Control and Configuration Interface Control Block
 * The size of MAC reserved: cci_cb_boot_t.cb_sz - (CCI_CB_CFG_SIZE + CCI_CB_STS_SIZE)
 */
typedef struct cci_cb_t
{
    cci_cb_config_t cfg;
    cci_cb_status_t sts;

    uint32_t mac_rsvd[4];           /* for starting pointer of &mac_rsvd[0] */
} cci_cb_t;

/* Put below 2 macros in init function to check the integrity of CCI stuructures */
#define CCI_CHK_STRUCT_CB_CFG() CCI_CHK_STRUCT_SIZE(sizeof(cci_cb_config_t) % CCI_CB_CFG_SIZE)
#define CCI_CHK_STRUCT_CB_STS() CCI_CHK_STRUCT_SIZE(sizeof(cci_cb_status_t) % CCI_CB_STS_SIZE)

/*=== CCI Control Block - End */

/*=== CCI Common */

/**
 * @struct cci_comm_hdr_t
 * @brief WLAN MAC cci command and event common header
 * Host sends cci command to mac and mac sends cci event to host. 
 * This header will have common details like group_id, id 
 * and length of the message, this can be used to decode and invoke 
 * corresponding handler.
 */

typedef struct cci_comm_hdr{
    uint32_t id         : 16;  /* CCI_CMD_ID_xxx or CCI_EVT_ID_xxx */
    uint32_t group_id   :  8;
    uint32_t tr_id      :  8;  /* transaction ID for matching ACK event */

    uint32_t length     : 10;  /* msg size in bytes not including this header */
    uint32_t reserved_0 :  6;
    uint32_t radio_id   :  2;  
    uint32_t reserved_1 : 14;
} cci_comm_hdr_t;
#define CCI_HDR_LEN     sizeof(cci_comm_hdr_t)
#define cmd_id id /* Temp: remove in the next check-in */

/**
 * @struct cci_generic_t
 * @brief WLAN MAC cci command and event generic format
 * This structure is generic format of cci cmd and evt, 
 * used to cast any received message using this stcuct
 * and parse the header to understand what command or event is
 * received.
 */
typedef struct cci_generic{
    cci_comm_hdr_t hdr;      
    uint32_t       msg[2];   /* for casting  pMsg =(MSG_t *)&pCmd->msg[0]; */
} cci_generic_t;
#define cci_cmd_hdr hdr /* Temp: remove in the next check-in */

/**
 * @struct cci_comm_tlv_t
 * @brief WLAN MAC cci command common TLV format
 * This structure is used for following messages:
 *  1. All CCI_CMD_ID_xxx_SET[GET]_PARAM
 *  2. All CCI_EVT_ID_xxx_SET[GET]_PARAM
 *  3. To be extended...
 */
typedef struct cci_comm_tlv {
    uint16_t type;
    uint16_t len;           /* b[1:0]= 0: Multiples of 4B, not include type and len */
    uint32_t value[1];      /* for the start of parameter value or structure casting */
} cci_comm_tlv_t;

/** IEEE 802.11 Capability definitions ******************/
/** 9.4.1.4 Capability Information field -2B */
typedef struct cci_ieee_legacy_cap {
    uint16_t ess:1;             /* WLAN_CAPABILITY_ESS              (1<<0) */
    uint16_t ibss:1;            /* WLAN_CAPABILITY_IBSS             (1<<1) */
    uint16_t cf_pollable:1;     /* WLAN_CAPABILITY_CF_POLLABLE      (1<<2) */
    uint16_t cf_poll_req:1;     /* WLAN_CAPABILITY_CF_POLL_REQUEST  (1<<3) */
    uint16_t privacy:1;         /* WLAN_CAPABILITY_PRIVACY          (1<<4) */
    uint16_t short_preamble:1;  /* WLAN_CAPABILITY_SHORT_PREAMBLE   (1<<5) */
    uint16_t rsvd:2;            /* WLAN_CAPABILITY_PBCC(1<<6) _CHANNEL_AGILITY(1<<7) */
    uint16_t spectrum_mgmt:1;   /* WLAN_CAPABILITY_SPECTRUM_MGMT    (1<<8) */
    uint16_t qos:1;             /* WLAN_CAPABILITY_QOS              (1<<9) */
    uint16_t short_slot_time:1; /* WLAN_CAPABILITY_SHORT_SLOT_TIME  (1<<10) */
    uint16_t apsd:1;            /* WLAN_CAPABILITY_APSD             (1<<11) */
    uint16_t radio_measure:1;   /* WLAN_CAPABILITY_RADIO_MEASURE    (1<<12) */
    uint16_t rsvd2:1;           /* WLAN_CAPABILITY_DSSS_OFDM        (1<<13) */
    uint16_t delayed_ba:1;      /* WLAN_CAPABILITY_DEL_BACK         (1<<14) */
    uint16_t immed_ba:1;        /* WLAN_CAPABILITY_IMM_BACK         (1<<15) */
} cci_ieee_legacy_cap_t;

/** 9.4.2.56 HT Capabilities element - 2B */
typedef struct cci_ieee_ht_cap {
    uint16_t ldpc:1;
    uint16_t sup_ch_width_20_40:1;
    uint16_t sm_ps:2;           /* 0:static, 1:dyn, 2:invalid, 3:diabled */
    uint16_t green_field:1;
    uint16_t sgi_20:1;
    uint16_t sgi_40:1;
    uint16_t tx_stbc:1;
    uint16_t rx_stbc:2;
    uint16_t delayed_ba:1;
    uint16_t max_amsdu:1;       /* 0:3839, 1:7935 bytes */
    uint16_t dsss_cck_40:1;
    uint16_t rsvd:1;
    uint16_t intolerant_40:1;
    uint16_t lsig_txop_prot:1;
} cci_ieee_ht_cap_t;

/** 9.4.2.56.3 A-MPDU Parameters field - 1B
 *  HT ampdu_factor is always 3: 2^(13+3) - 1 = 65535B
 *     ampdu_density will be common from cci_cap_mac */

/** 9.4.2.56.4 Supported MCS Set field - 16B */
typedef struct cci_ieee_ht_mcs_set {
    uint8_t  rx_mask[10];       /* 77 + 3-bit reserved */
    uint16_t rx_highest_rate;   /* 10 + 6-bit reserved, 0-1023 in units of 1Mbps */

    uint8_t  tx_set_defined:1;
    uint8_t  tx_rx_not_eq:1;
    uint8_t  tx_max_nss:2;
    uint8_t  tx_uneq_modulation:1;
    uint8_t  tx_rsvd:3;
    uint8_t  reserved[3];
} cci_ieee_ht_mcs_set_t;

/** 9.4.2.158 VHT Capabilities element - 4B */
typedef struct cci_ieee_vht_cap {
    uint32_t max_mpdu_len:2;    /* 0:3895, 1:7991, 2:11454 */
    uint32_t supp_ch_widtht:2;  /* 0:80, 1:160, 2:80+80 MHz */
    uint32_t rx_ldpc:1;
    uint32_t short_gi_80:1;
    uint32_t short_gi_160:1;
    uint32_t tx_stbc:1;
    uint32_t rx_stbc:3;
    uint32_t su_bfmer_cap:1;
    uint32_t su_bfmee_cap:1;
    uint32_t bfmee_sts:3;
    uint32_t sounding_dimensions:3;
    uint32_t mu_bfmer_cap:1;
    uint32_t mu_bfmee_cap:1;
    uint32_t vht_txop_ps:1;
    uint32_t htc_vht:1;
    uint32_t ampdu_len_exponent:3;  /* Max 2^(13+7) - 1 = 1MB */
    uint32_t vht_la_unsol_mfb:1;    /* VHT Link Adaptation Capable */
    uint32_t vht_la_mrq_mfb:1;      /* VHT Link Adaptation Capable */
    uint32_t rx_antenna_pattern_consistency:1;
    uint32_t tx_antenna_pattern_consistency:1;
    uint32_t ext_nss_bw:2;
} cci_ieee_vht_cap_t;

/** 9.4.2.158.3 Supported VHT-MCS and NSS Set field - 8B */
typedef struct cci_ieee_vht_mcs_nss_set {
    uint16_t rx_mcs_map;        /* 2-bit per SS */

    uint16_t rx_highest:13;     /* Long GI data rate */
    uint16_t max_nsts_total:3;

    uint16_t tx_mcs_map;        /* 2-bit per SS */

    uint16_t tx_highest:13;     /* Long GI data rate */
    uint16_t vht_ext_nss_bw_cap:1;
    uint16_t reserved:2;
} cci_ieee_vht_mcs_nss_set_t;
#define CCI_IEEE_VHT_MCS_0_7            0
#define CCI_IEEE_VHT_MCS_0_8            1
#define CCI_IEEE_VHT_MCS_0_9            2
#define CCI_IEEE_VHT_MCS_NOT_SUPPORTED  3

/** 9.4.2.248.2 HE MAC Capabilities Information field - 6B + 2B padding */
typedef struct cci_ieee_he_mac_cap {
    uint32_t htc_he:1;
    uint32_t twt_req:1;
    uint32_t twt_rsp:1;
    uint32_t dynamic_frag:2;    /* 0:Not support, 1-3:Level */
    uint32_t max_frag_msdus_exponent:3; /* 0:1, 1:2, ... 6:64, 7:Unlmited */

    uint32_t min_frag_size:2;
    uint32_t tf_mac_pad_duration:2;     /* 0:0, 1:8, 2:16us */
    uint32_t mtid_aggr_rx:3;            /* number of TIDs minus 1 of QoS Data frames */
    uint32_t link_adaptation:2;
    uint32_t all_ack:1;
    uint32_t trs:1;
    uint32_t bsr:1;
    uint32_t bcast_twt:1;
    uint32_t ba_bitmap_32bit:1;
    uint32_t mu_cascading:1;
    uint32_t ack_enabled_aggr:1;

    uint32_t rsvd_bit_24:1;
    uint32_t omi_control:1;
    uint32_t ofmda_ra:1;
    uint32_t max_ampdu_len_exponent:2;  /* min((2^(20+3) - 1), 6500631) */
    uint32_t amsdu_frag:1;
    uint32_t flex_twt_sched:1;
    uint32_t rx_ctrl_frame_to_multibss:1;

    uint16_t bsrp_bqrp_ampdu_aggr:1;
    uint16_t qtp:1;
    uint16_t bqr:1;
    uint16_t psr_responder:1;
    uint16_t ndp_feedback_report:1;
    uint16_t ops:1;
    uint16_t amsdu_in_ampdu:1;
    uint16_t mtid_aggr_tx:3;
    uint16_t subch_selective_tx:1;
    uint16_t ul_2x996_tone_ru:1;
    uint16_t om_ctrl_ul_mu_data_disable_rx:1;
    uint16_t he_dynamic_sm_ps:1;
    uint16_t punctured_sounding:1;
    uint16_t ht_vht_tf_rx:1;

    uint16_t pad;
} cci_ieee_he_mac_cap_t;

/** 9.4.2.248.3 HE PHY Capabilities Information field - 11B + 1B padding */
typedef struct cci_ieee_he_phy_cap {
    uint32_t rsvd_bit_0:1;
    uint32_t ch_width_set:7;    /* 5G 0x01:rsvd, 02:40_80M, 04:160M, 08:80p80M, 10:242_RU
                                   2G 0x01:40M,  02~08:rsvd, 10:242_RU 20:all rsvd*/
    uint32_t punctured_preamble_rx:4;   /* B0-1:80, B2-3:160 or 80p80 MHz */
    uint32_t device_class_a:1;  /* 0:class B, 1:class A, for non AP STA only */
    uint32_t ldpc_coding_in_payload:1;
    uint32_t su_ppdu_1x_ltf_08_gi:1;
    uint32_t midamble_tx_rx_max_nsts:2; /* if doppler_tx rx set */
    uint32_t ndp_4x_ltf_32_gi:1;
    uint32_t stbc_tx_le_80mhz:1;
    uint32_t stbc_rx_le_80mhz:1;
    uint32_t doppler_tx:1;
    uint32_t doppler_rx:1;
    uint32_t full_bw_ul_mu_mimo:1;
    uint32_t partial_bw_ul_mu_mimo:1;
    uint32_t dcm_max_constellation_tx:2;
    uint32_t dcm_max_nss_tx:1;
    uint32_t dcm_max_constellation_rx:2;
    uint32_t dcm_max_nss_rx:1;
    uint32_t rx_he_mu_ppdu_from_nonap_sta:1;
    uint32_t su_bfmer:1;                /* b31 */

    uint32_t su_bfmee:1;
    uint32_t mu_bfmer:1;
    uint32_t bfmee_sts_le_80mhz:3;
    uint32_t bfmee_sts_gt_80mhz:3;
    uint32_t sounding_dimensions_le_80mhz:3;
    uint32_t sounding_dimensions_gt_80mhz:3;
    uint32_t ng_16_su_feedback:1;
    uint32_t ng_16_mu_feedback:1;

    uint32_t codebook_size_4_2_su:1;
    uint32_t codebook_size_7_5_mu:1;
    uint32_t trig_su_bf_feedback:1;
    uint32_t trig_mu_bf_partial_bw_feedback:1;
    uint32_t trig_cqi_feedback:1;
    uint32_t partial_bw_ext_range:1;
    uint32_t partial_bw_dl_mu_mimo:1;
    uint32_t ppe_threshold_present:1;   /* 1:cci_ieee_he_ppe_thres_t, b55 */

    uint32_t psr_based_sr:1;
    uint32_t power_boost_factor:1;
    uint32_t su_mu_ppdu_4x_ltf_08_gi:1;
    uint32_t max_nc:3;
    uint32_t stbc_tx_gt_80mhz:1;
    uint32_t stbc_rx_gt_80mhz:1;        /* b63 */

    uint32_t er_su_ppdu_4x_ltf_08_gi:1;
    uint32_t ppdu_20_in_40mhz_in_2g:1;
    uint32_t ppdu_20_in_160mhz:1;
    uint32_t ppdu_80_in_160mhz:1;
    uint32_t er_su_ppdu_1x_ltf_08_gi:1;
    uint32_t midamble_tx_rx_2x_n_1x_ltf:1;
    uint32_t dcm_max_ru:2;
    uint32_t longer_than_16_sigb_ofdm_sym:1; /* >= 16 users, b72 */
    uint32_t non_trig_cqi_feedback:1;
    uint32_t tx_1024qam_lt_242tone_ru:1;
    uint32_t rx_1024qam_lt_242tone_ru:1;
    uint32_t rx_full_su_using_mu_with_comp_sigb:1;
    uint32_t rx_full_su_using_mu_with_non_comp_sigb:1;
    uint32_t nominal_packet_padding:2;  /* 0,8,16us when ppe_threshold_present=0, b78-9 */
    uint32_t n_he_ltf_gt_one_ru_mu:1;
    uint32_t rsvd_bit_81_87:7;          /* b87 - 11B */

    uint32_t pad:8;
} cci_ieee_he_phy_cap_t;


/** 9.4.2.248.4 Supported HE-MCS And NSS Set field - Fixed max 12B */
typedef struct cci_ieee_he_mcs_nss_set {
    uint16_t rx_mcs_80;        /* 2-bit per SS */
    uint16_t tx_mcs_80;
    uint16_t rx_mcs_160;
    uint16_t tx_mcs_160;
    uint16_t rx_mcs_80p80;
    uint16_t tx_mcs_80p80;
} cci_ieee_he_mcs_nss_set_t;
#define CCI_IEEE_HE_MCS_0_7             0
#define CCI_IEEE_HE_MCS_0_9             1
#define CCI_IEEE_HE_MCS_0_11            2
#define CCI_IEEE_HE_MCS_NOT_SUPPORTED   3

/** 9.4.2.248.5 PPE Thresholds field - max 25B + 3B padding */
typedef struct cci_ieee_he_ppe_thres {
    uint8_t  ppe_thres_tbd[25];

    uint8_t  pad[3];
} cci_ieee_he_ppe_thres_t;

/** 9.4.2.263 HE 6 GHz Band Capabilities element - 2B + 2B padding */
typedef struct cci_ieee_he_6ghz_cap {
    uint16_t min_mpdu_start:3;
    uint16_t max_ampdu_len_exponent:3;  /* 2^(13+7) - 1 = 1MB */
    uint16_t max_mpdu_len:2;            /* 0:3895, 1:7991, 2:11454 */
    uint16_t rsvd_bit_8:1;
    uint16_t sm_ps:2;
    uint16_t rd_responder:1;
    uint32_t rx_antenna_pattern_consistency:1;
    uint32_t tx_antenna_pattern_consistency:1;
    uint16_t rsvd_bit_14_15:2;

    uint16_t pad;
} cci_ieee_he_6ghz_cap_t;

/** CCI IEEE80211 Capabilities - 96B per band */
typedef struct cci_ieee_caps {
    cci_ieee_legacy_cap_t      legacy_cap;  /* 2B */
    cci_ieee_ht_cap_t          ht_cap;      /* 2B */
    cci_ieee_ht_mcs_set_t      ht_mcs;      /* 16B */
    cci_ieee_vht_cap_t         vht_cap;     /* 4B */
    cci_ieee_vht_mcs_nss_set_t vht_mcs_nss; /* 8B */
    cci_ieee_he_mac_cap_t      he_mac_cap;  /* 8B */
    cci_ieee_he_phy_cap_t      he_phy_cap;  /* 12B */
    cci_ieee_he_mcs_nss_set_t  he_mcs_nss;  /* 12B */
    cci_ieee_he_ppe_thres_t    he_ppe_thres;/* 28B */
    cci_ieee_he_6ghz_cap_t     he_6ghz_cap; /* 4B */
} cci_ieee_caps_t;

/**
 * @struct cci_cap_sys_t
 * @brief WLAN MAC system capabilities that can be supported
 */
typedef struct cci_cap_sys {
    uint8_t  num_radios;    /** Max number of radios */
    uint8_t  num_vifs;      /** Max number of vif entries that can be allocated in MAC */
    uint16_t num_stas;      /** Max number of sta entries that can be allocated in MAC */

    uint32_t rsvd[3];       /* TBD: Make 16B */
} cci_cap_sys_t;

/* Supported wireless mode bitmap
 */
#define CCI_WLAN_MODE_B     (1 << 0)
#define CCI_WLAN_MODE_G     (1 << 1)
#define CCI_WLAN_MODE_A     (1 << 2)
#define CCI_WLAN_MODE_N     (1 << 3)
#define CCI_WLAN_MODE_AC    (1 << 4)
#define CCI_WLAN_MODE_AX    (1 << 5)

#define CCI_SERVICE_BM_SIZE ((CCI_MAX_SERVICE + sizeof(uint32_t) - 1) / sizeof(uint32_t))
/**
 * @struct cci_cap_mac_t
 * @brief WLAN MAC feature capabilities that can be supported
 */
typedef struct cci_cap_mac {
    uint8_t  wlan_mode_bm;  /** CCI_WLAN_MODE_x */
    uint8_t  ampdu_density; /** 3:1, 4:2, 5:4, 6:8, 7:16 us - IEEE encoding for mmss */
    uint8_t  mmss;          /** MPDU density: 1, 2, 4, 8, 16us - obsolete soon */
    uint8_t  rsvd1[1];

    uint32_t service_bm[CCI_SERVICE_BM_SIZE];   /* TBD: */

    uint32_t rsvd[2];       /* TBD: Make 16B */
} cci_cap_mac_t;


/**
 * @brief copy from enum nl80211_band except 60GHZ, 900MHz(S1G)
 * @CCI_NL80211_BAND_2GHZ: 2.4 GHz ISM band
 * @CCI_NL80211_BAND_5GHZ: around 5 GHz band (4.9 - 5.7 GHz)
 * @CCI_NL80211_BAND_6GHZ: around 6 GHz band (5.9 - 7.2 GHz)
 * @CCI_NUM_NL80211_BANDS: number of bands
 */
#define	CCI_NL80211_BAND_2GHZ   0
#define	CCI_NL80211_BAND_5GHZ   1
#define	CCI_NL80211_BAND_6GHZ   2
#define	CCI_NUM_NL80211_BANDS   3

/**
 * @struct cci_cap_phy_t
 * @brief WLAN PHY capabilities that can be supported per radio
 */
typedef struct cci_cap_phy {
    /** Tx/Rx antenna and spatial stream capa */
    uint8_t  num_antennas_tx:4;         /** No. of transmit antennas */
    uint8_t  num_spatial_streams_tx:4;  /** No. of transmit spatial streams */
    uint8_t  num_antennas_rx:4;         /** No. of receive antennas */
    uint8_t  num_spatial_streams_rx:4;  /** No. of receive spatial streams */
    uint8_t  pad1; 
    uint8_t  band_support_bm;           /** BIT(CCI_NL80211_BAND_xxx) for band[] */

    struct cci_cap_band {
        uint32_t max_bw:4;              /** Max channel bandwidth 20/40/80/160/320 MHz */
        uint32_t rsvd1:28;
    } band[CCI_NUM_NL80211_BANDS];      /* 4B * 3 */

    uint32_t phy_cap[2];    /* TBD: */
} cci_cap_phy_t;


/**
 * @struct cci_radio_init_cfg_t
 * @brief radio basic configuration parameters
 * This structure has various parameters to be configured for a radio.
 * Host can send various config params to firmware for each radio
 * based on user configuration. First time after radio init, these
 * group of parameters should be configured for each radio.
 */

typedef struct cci_radio_init_cfg {
    /** Configured bandwidth of the radio */
    uint32_t channel_band:3;      /** Channel band  : 2.4 GHz, 5 GHz. enum e80211_band */
    uint32_t channel_bw:4;        /** Channel bandwidth (not including the punctured BWs) */
    uint32_t wlan_mode:1;         /** 0 - Full WLAN radio, 1 - Monitor only mode (Remove and statically allocate to third instance) */
    uint32_t dss_enable:1;        /** Specifies whether the channel supports CCK detection or not */
    uint32_t resvd1:23;

    /** Channel specification */
    uint32_t center_freq1:16;      /** Centre frequency of first segment specified band in MHz
                                  Specifies the center frequency of the entire channel in case of 20/40/80/160/320 MHz bands
                                  Specifies the segment that contains the primary channel in case of 80+80/160+160 */
    uint32_t center_freq2:16;      /** Centre frequency of second segment specified band in MHz. For 80+80 MHz/160+160 MHz */


    uint32_t primary_channel_num:4;   /** Channel number if primary within the band (0 to 15 - 320 MHz )*/
    uint32_t ht_forty_plus_minus:2;   /** Specifies the position of the secondary 20 MHz channel (Might not be required in L1) */
    uint32_t is_dfs_channel:1;        /** Indicates the channel is a DFS channel and will require radar detection */

    uint32_t resvd2:13;

    /** Tx/Rx antenna and spatial stream config */
    uint32_t num_tx_antennas:4;           /** No. of transmit antennas */
    uint32_t num_spatial_streams_tx:4;    /** No. of transmit spatial streams */
    uint32_t tx_spatial_stream_mapping:8; /** Bitmap specifying the tx spatial streams mapped to the radio. Bit 0 : Antenna 0 (J0) */
    uint32_t num_rx_antennas:4;           /** No. of receive antennas */
    uint32_t num_spatial_streams_rx:4;    /** No. of receive spatial streams */
    uint32_t rx_spatial_stream_mapping:8; /** Bitmap specifying the rx spatial streams mapped to the radio. Bit 0 : Antenna 0 (J0) */

    uint32_t radio_rsvd[4];
} cci_radio_init_cfg_t;

/** TLV definitions ******************/
/** Channel specification */
typedef struct cci_param_channel {
    uint32_t band:3;                /** Channel band  : 2.4 GHz, 5 GHz. enum e80211_band */
    uint32_t bw:4;                  /** Channel bandwidth (not including the punctured BWs) */
    uint32_t rsvd_monitor:1;
    uint32_t dss_enable:1;          /** Specifies whether the channel supports CCK detection or not */
    uint32_t resvd1:16;
    uint32_t primary_channel_num:4; /** Channel number if primary within the band (0 to 15 - 320 MHz )*/
    uint32_t ht_forty_plus_minus:2; /** Specifies the position of the secondary 20 MHz channel (Might not be required in L1) */
    uint32_t is_dfs_channel:1;      /** Indicates the channel is a DFS channel and will require radar detection */


    uint32_t center_freq1:16;      /** Centre frequency of first segment specified band in MHz
                                    Specifies the center frequency of the entire channel in case of 20/40/80/160/320 MHz bands
                                    Specifies the segment that contains the primary channel in case of 80+80/160+160 */
    uint32_t center_freq2:16;      /** Centre frequency of second segment specified band in MHz. For 80+80 MHz/160+160 MHz */
} cci_param_channel_t;

/** Tx/Rx antenna and spatial stream config */
typedef struct cci_param_nss {
    uint32_t num_tx_antennas:4;           /** No. of transmit antennas */
    uint32_t num_spatial_streams_tx:4;    /** No. of transmit spatial streams */
    uint32_t tx_spatial_stream_mapping:8; /** Bitmap specifying the tx spatial streams mapped to the radio. Bit 0 : Antenna 0 (J0) */
    uint32_t num_rx_antennas:4;           /** No. of receive antennas */
    uint32_t num_spatial_streams_rx:4;    /** No. of receive spatial streams */
    uint32_t rx_spatial_stream_mapping:8; /** Bitmap specifying the rx spatial streams mapped to the radio. Bit 0 : Antenna 0 (J0) */
} cci_param_nss_t;


/*=== CCI Common - End */

/*=== CCI Command */

enum Ecci_cmd_groups {
    CCI_CMD_GRP_SYSTEM,
    CCI_CMD_GRP_RADIO,
    CCI_CMD_GRP_VIF,
    CCI_CMD_GRP_STA,
    CCI_CMD_GRP_FWDBG_STATS,
    CCI_CMD_GRP_VENDOR_CMD,
    CCI_CMD_GRP_MAX,
};

/** SYSTEM Commands ******************/
enum Ecci_cmd_group_system {    
    CCI_CMD_ID_SYSTEM_BOOT,
    CCI_CMD_ID_SYSTEM_RESET,
    CCI_CMD_ID_SYSTEM_HALT,
    CCI_CMD_ID_SYSTEM_MEM_DUMP,
    CCI_CMD_ID_SYSTEM_INIT,
    /* Configuration and Event Rings setup */
    CCI_CMD_ID_CFG_RINGS,
    CCI_CMD_ID_SYSTEM_MAX,
};

/**
 * @struct cci_cmd_system_init_t
 * @brief cci init command sent from host to mac
 * This structure has various parameters to be configured at the soc level,
 * Includes one time initialization like firmware pool allocation for vaious data
 * structures.
 * Includes Radio/phy level parameter setting.
 *
 */
typedef struct cci_cmd_system_init{
    cci_comm_hdr_t cmd_hdr;

    uint8_t  radio_en_bm;   /** Bitmap for enabling radios. b0(radio 0) ~ bn(radio max-1) */
    uint8_t  num_vifs;      /** Max number of vifs that can be supported */
    uint16_t num_stas;      /** Max number of stations that can be connected at the same time */
    uint8_t  rsvd1[32 - (CCI_HDR_LEN + 4)];

    cci_radio_init_cfg_t  config_radio[CCI_MAX_RADIO]; /** radio band/freq initialization in 2g/5g/6g */
} cci_cmd_system_init_t;


/** RADIO Commands ******************/
enum Ecci_cmd_group_radio {
    CCI_CMD_ID_RADIO_BASIC_CONF,
    CCI_CMD_ID_RADIO_SET_PARAM,
    CCI_CMD_ID_RADIO_GET_PARAM,
    CCI_CMD_ID_RADIOGRP_MAX,
};

/** This enum gives the type information of radio parameter that is used in TLV of CCI_CMD_ID_RADIO_SET_PARAM
    64 max. If required more than 64, create a new CMD. */
typedef enum Ecci_radio_param {
    CCI_RADIO_PARAM_CHANNEL,        /** cci_param_channel_t */
    CCI_RADIO_PARAM_CHAINMASK,
    CCI_RADIO_PARAM_POWER_TABLE,
    CCI_RADIO_PARAM_POWER_LIMIT,
    CCI_RADIO_PARAM_ANTENNA_GAIN,
    CCI_RADIO_PARAM_DFS,
    /* Rx sensitivity */
    CCI_RADIO_PARAM_CCA_THRESHOLD,
    CCI_RADIO_PARAM_BSS_COLOR,
    /* TBD... */
    CCI_RADIO_PARAM_MAX = (sizeof(uint64_t) - 1)
} Ecci_radio_param_t;

/**
 * @struct cci_cmd_radio_set_param_t
 * @brief cci command sent from host to mac to set a particular radio param
 * param1 - 1st parameter TLV [and more] until cmd_hdr.length.
 *
 */
typedef struct cci_cmd_radio_set_param {
    cci_comm_hdr_t cmd_hdr;
    cci_comm_tlv_t param1;          /** 1st TLV from Ecci_radio_param_t */
} cci_cmd_radio_set_param_t;

/**
 * @struct cci_cmd_radio_get_param_t
 * @brief cci command sent from host to mac to get a particular radio param
 * param_req_bm - bitmap of Ecci_radio_param_t to request information
 *
 */
typedef struct cci_cmd_radio_get_param {
    cci_comm_hdr_t cmd_hdr;
    uint64_t       param_req_bm;    /* set BIT(Ecci_radio_param_t) */
} cci_cmd_radio_get_param_t;


/** VIF Commands ******************/
enum Ecci_cmd_group_vif {
    /* Create VIF instance          */
    CCI_CMD_ID_VIF_CREATE,
    /* Start a VIF instance         */
    CCI_CMD_ID_VIF_START,
    /* Configure a VIF using TLV format */
    CCI_CMD_ID_VIF_SET_PARAM,
    /* Get a VIF configuration using TLV format */
    CCI_CMD_ID_VIF_GET_PARAM,
    /* Stop a VIF instance          */
    CCI_CMD_ID_VIF_STOP,
    /* Restart VIF instance */
    CCI_CMD_ID_VIF_RESTART,
    /* Delete a VIF instance        */
    CCI_CMD_ID_VIF_DELETE,
    /* Shared key (WEP), Group key (WPA), IGTK (PMF) */
    CCI_CMD_ID_VIF_PARAM_KEY,
};

typedef enum Ecci_vif_type {
    CCI_VIF_TYPE_AP,
    CCI_VIF_TYPE_MONITOR,
    CCI_VIF_TYPE_STA,
    /* CCI_VIF_TYPE_IBSS, */
} Ecci_vif_type_t;

/**
 * @struct cci_cmd_vif_create_t
 * @brief cci command for creating a VIF
 *
 */
typedef struct cci_cmd_vif_create {
    cci_comm_hdr_t cmd_hdr;

    uint8_t  mac_addr[CCI_ETH_ALEN];/** 6 bytes BSSID */
    uint8_t  vif_id;                /** 0 - (CCI_MAX_VIF - 1) */
    uint8_t  vif_type;              /** Ecci_vif_type_t */

    uint8_t  wlan_mode_bm;          /** CCI_WLAN_MODE_x (B, G, A, N, AC, AX) */
    uint8_t  rsvd[15];

} cci_cmd_vif_create_t;

/**
 * @struct cci_cmd_vif_start_t
 * @brief cci command for starting a VIF
 *
 */
typedef struct cci_cmd_vif_start {
    cci_comm_hdr_t cmd_hdr;

    uint8_t  vif_id;                /** 0 - (CCI_MAX_VIF - 1) */
    uint8_t  ssid_len;              /** Max 32B SSID */
    uint8_t  rsvd[2];
    uint32_t op_flags;              /** dual CTS, No Tx BSSID, QoS, SPP etc */

    uint8_t  ssid[32];              /** Max 32B SSID */

    uint16_t bss_basic_rate;        /** b0: 1M, … b4: 6M, … b11:54M */
    uint16_t rts_threshold;
    uint8_t  short_retry_limit;
    uint8_t  long_retry_limit;
    uint8_t  bcn_rate:2;            /** 1M or 6M */
    uint8_t  bcn_dtim_period:6;
    uint8_t  bcn_interval;          /** TU */

    uint16_t bcn_capa_info;         /** 9.4.1.4 Capability Information field */
    uint16_t bcn_tx_pwr;

    uint32_t region_domain;
    cci_param_nss_t  streams;         /** Max TxRx streams preferred */
} cci_cmd_vif_start_t;


/** VIF parameter that is used in TLV of CCI_CMD_ID_VIF_SET_PARAM
  First 32 TLV types match with enum ieee80211_bss_change */
typedef enum Ecci_vif_param {
    CCI_VIF_PARAM_AID,          /** BSS_CHANGED_ASSOC */
    CCI_VIF_PARAM_PROT_MODE,    /** BSS_CHANGED_ERP_CTS_PROT */
    CCI_VIF_PARAM_PREAMBLE,     /** BSS_CHANGED_ERP_PREAMBLE */
    CCI_VIF_PARAM_SLOT_TIME,    /** BSS_CHANGED_ERP_SLOT */
    CCI_VIF_PARAM_HT,           /** BSS_CHANGED_HT */
    CCI_VIF_PARAM_BASIC_RATES,  /** BSS_CHANGED_BASIC_RATES */
    CCI_VIF_PARAM_BCN_INTERVAL, /** BSS_CHANGED_BEACON_INT */
    CCI_VIF_PARAM_BSSID,        /** BSS_CHANGED_BSSID */
    CCI_VIF_PARAM_BCN_TMPL,     /** BSS_CHANGED_BEACON */
    CCI_VIF_PARAM_BCN_MODE,     /** BSS_CHANGED_BEACON_ENABLED */
    CCI_VIF_PARAM_CQM,          /** BSS_CHANGED_CQM Connection Quality Monitor */
    CCI_VIF_PARAM_IBSS,         /** BSS_CHANGED_IBSS */
    CCI_VIF_PARAM_ARP_FILTER,   /** BSS_CHANGED_ARP_FILTER */
    CCI_VIF_PARAM_QOS,          /** BSS_CHANGED_QOS */
    CCI_VIF_PARAM_IDLE,         /** BSS_CHANGED_IDLE */
    CCI_VIF_PARAM_SSID,         /** BSS_CHANGED_SSID */
    CCI_VIF_PARAM_AP_PRS,       /** BSS_CHANGED_AP_PROBE_RESP */
    CCI_VIF_PARAM_PS,           /** BSS_CHANGED_PS */
    CCI_VIF_PARAM_TX_PWR,       /** BSS_CHANGED_TXPOWER */
    CCI_VIF_PARAM_P2P_PS,       /** BSS_CHANGED_P2P_PS */
    CCI_VIF_PARAM_BCN_INFO,     /** BSS_CHANGED_BEACON_INFO currently dtim_period only */
    CCI_VIF_PARAM_BW,           /** BSS_CHANGED_BANDWIDTH */
    CCI_VIF_PARAM_OCB,          /** BSS_CHANGED_OCB */
    CCI_VIF_PARAM_MU_GRPS,      /** BSS_CHANGED_MU_GROUPS */
    CCI_VIF_PARAM_KEEP_ALIVE,   /** BSS_CHANGED_KEEP_ALIVE */
    CCI_VIF_PARAM_MCAST_RATE,   /** BSS_CHANGED_MCAST_RATE */
    CCI_VIF_PARAM_FTM_RESPONDER,/** BSS_CHANGED_FTM_RESPONDER */
    CCI_VIF_PARAM_TWT,          /** BSS_CHANGED_TWT */
    CCI_VIF_PARAM_HE_OBSS_PD,   /** BSS_CHANGED_HE_OBSS_PD */
    CCI_VIF_PARAM_HE_BSS_COLOR, /** BSS_CHANGED_HE_BSS_COLOR */
    CCI_VIF_PARAM_FILS_DISCOVERY,  /** BSS_CHANGED_FILS_DISCOVERY */
    CCI_VIF_PARAM_UNSOL_BCAST_PRS, /** BSS_CHANGED_UNSOL_BCAST_PROBE_RESP */
    /* TBD... */

    CCI_VIF_PARAM_MAX = (sizeof(uint64_t) - 1)
} Ecci_vif_param_t;

/**
 * @struct cci_cmd_vif_set_param_t
 * @brief cci command sent from host to mac to set a particular vif param
 * param1 - 1st parameter TLV [and more] until cmd_hdr.length.
 * restart - Restart vif instance after applying parameter TLV changes
 *
 */
typedef struct cci_cmd_vif_set_param {
    cci_comm_hdr_t cmd_hdr;
    cci_comm_tlv_t param1;      /** 1st TLV from Ecci_vif_param_t */

    uint8_t  restart;           /** For started VIF */
    uint8_t  rsvd[15];
} cci_cmd_vif_set_param_t;

/**
 * @struct cci_cmd_vif_get_param_t
 * @brief cci command sent from host to mac to get a particular vif param
 * param_req_bm - bitmap of Ecci_vif_param_t to request information
 *
 */
typedef struct cci_cmd_vif_get_param {
    cci_comm_hdr_t cmd_hdr;
    uint64_t       param_req_bm;  /* set BIT(Ecci_vif_param_t) */
} cci_cmd_vif_get_param_t;

/**
 * @struct cci_cmd_vif_change_t
 * @brief cci common format for VIF STOP/RESTART/DELETE commands
 */
typedef struct cci_cmd_vif_change
{
    cci_comm_hdr_t cmd_hdr;

    uint8_t  vif_id;                /** 0 - (CCI_MAX_VIF - 1) */
    uint8_t  rsvd8[3];
    uint32_t option;                /** parameters that can be applied optionally */
} cci_cmd_vif_change_t;


/** STA Commands ******************/
enum Ecci_cmd_group_sta {
    /* Create STA entry when authenticated    */
    CCI_CMD_ID_STA_ADD,
    /* Update STA parameters when [re]associated */
    CCI_CMD_ID_STA_ASSOC,
    /* Change STA ieee80211_sta_state */
    CCI_CMD_ID_STA_STATE,
    /* Configure a VIF using TLV format */
    CCI_CMD_ID_STA_SET_PARAM,
    /* Get a VIF configuration using TLV format */
    CCI_CMD_ID_STA_GET_PARAM,
    /* Delete STA entry */
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


/**
 * @struct cci_cmd_sta_add_t
 * @brief cci command for creating a STA when authenticated
 *
 */
typedef struct cci_cmd_sta_add
{
    cci_comm_hdr_t cmd_hdr;

    uint8_t  mac_addr[CCI_ETH_ALEN];/** 6 bytes STA MAC address */
    uint16_t pad;

    uint16_t vif_id;                /** is belong to [0 - (CCI_MAX_VIF - 1)] */
    uint16_t sta_id;                /** [CCI_MAX_VIF - (CCI_MAX_STA - 1)] */
    uint32_t rsvd32[3];
} cci_cmd_sta_add_t;

typedef struct cci_sta_op_flag
{
    uint32_t qos_sta    : 1;    /* QoS STA */
    uint32_t wds        : 1;    /* WDS 4 address */
    uint32_t mfp        : 1;    /* MGMT Frame Protection enabled, 9.4.2.54.4 RSN capa */
    uint32_t spp        : 1;    /* SPP AMSDU (CCMP AAD include bit 7 of the QCF) */
    uint32_t he_htc     : 1;    /* +HTC HE support */
    uint32_t rsvd       : 27;
} cci_sta_op_flag_t;

/**
 * @struct cci_cmd_sta_assoc_t
 * @brief cci command for updating STA parameters when [re]associated
 * 
 */
typedef struct cci_cmd_sta_assoc
{
    cci_comm_hdr_t cmd_hdr; 

    uint16_t vif_id;             /* for STA DB allocation design flexibility */
    uint16_t sta_id;
    uint16_t aid;                /* b[13:0] 1 - 2007 */
    uint8_t  sta_mode;           /* B ONLY, G ONLY, B+G MIXED, A ONLY, N ONLY, AC, HE */
    uint8_t  tx_phy_mode;

    cci_sta_op_flag_t op_flags;   /* MFP, WDS, QoS, SPP etc */
    uint32_t ack_policy_bm;       /* 2bits per TID */

    cci_param_nss_t  streams;     /* STA TxRx streams */

    uint8_t  priority;      /* Optional STA service priority on top of AC priority */
    uint8_t  qosInfo;       /* 9.4.1.17 Figure 9-83 in QoS Capability IE */

    /* Processed information of all STA capabilities and rates in plane format
     *   ...
     */
    uint32_t rate_info;     // legacy

    /* max_mpdu_bytes, mmss, tx_pwr… */
} cci_cmd_sta_assoc_t;

/**
 * @struct cci_cmd_sta_change_t
 * @brief cci common format for STA STATE/PAUSE/RESUME/DELETE commands
 */
typedef struct cci_cmd_sta_change_t
{
    cci_comm_hdr_t cmd_hdr;

    uint16_t sta_id;
    uint16_t bm_tid;                /** TID bitmap: 0xFF(all TIDs), reserved for STA_STATE & DELETE */
    uint32_t params;                /** parameters that can be applied optionally, i.e. state chnage */
} cci_cmd_sta_change_t;

/** STA parameter that is used in TLV of CCI_CMD_ID_STA_SET_PARAM
    64 max. If required more than 64, create a new CMD. */
typedef enum Ecci_sta_param {
    CCI_STA_PARAM_ACK_POLICY,
    CCI_STA_PARAM_BW,
    CCI_STA_PARAM_NSS,
    CCI_STA_PARAM_TX_PWR,
    /* TBD... */
    
    CCI_STA_PARAM_MAX = (sizeof(uint64_t) - 1)
} Ecci_sta_param_t;

/**
 * @struct cci_cmd_sta_set_param_t
 * @brief cci command sent from host to mac to set a particular sta param
 * param1 - 1st parameter TLV [and more] until cmd_hdr.length.
 *
 */
typedef struct cci_cmd_sta_set_param {
    cci_comm_hdr_t cmd_hdr;
    cci_comm_tlv_t param1;      /** 1st TLV from Ecci_sta_param_t */
} cci_cmd_sta_set_param_t;

/**
 * @struct cci_cmd_sta_get_param_t
 * @brief cci command sent from host to mac to get a particular sta param
 * param_req_bm - bitmap of Ecci_sta_param_t to request information
 *
 */
typedef struct cci_cmd_sta_get_param {
    cci_comm_hdr_t cmd_hdr;
    uint64_t       param_req_bm;  /* set BIT(Ecci_vif_param_t) */
} cci_cmd_sta_get_param_t;

/*
 * @struct cci_cmd_sta_key_t
 * @brief WLAN STA key configuration
 * This structure is defined for setting the security mode and key configuration
 * of WLAN STA.
 * This structure is associated with STA_PARAM_KEY cci cmd during [Re]Association
 * sequence. 
 */
typedef struct cci_cmd_sta_key_t
{
    /**< TODO: this is commented for now. need to be enabled
          during contorl module integration. TLV??? */
    //cci_comm_hdr_t cmd_hdr; 

    uint16_t sta_id;                /* 0 - 527 (TBD: set vif sta boundary vs */
                                    /* b[15] 0: sta_id, 1: vif_sta_id) */
    uint16_t action: 2;             /* 0: set, 1: delete, 2:get */
    uint16_t key_type: 2;           /* 0: PTK, 1: GTK, 2: IGTK */
    uint16_t key_id: 3;             /* PTK:0,1 WEP: 0,1,2,3 */
    uint16_t wep_defult_key: 1;     /* For WEP TX only */
    uint16_t key_len:6;             /* WEP(5 or 13), TKIP(16), */
                                    /* CCMP-GCMP-CMAC-GMAC(16 or 32) */
    uint16_t rsvd: 2;               /* Reserved for other TX-RX only flag */

	uint32_t cipher;                /* refer to CCI_CIPHER_SUITE_xxx */
    uint8_t  key[CCI_KEY_LEN_MAX];  /* 32B - 256 max */
} cci_cmd_sta_key_t;

/*
 * @struct cci_cmd_sta_ba_t
 * @brief Block Ack session configuration
 *  ssn      - starting SN of the first frame to be sent after the BA session is negotiated
 *  active   - 1:ADDBA, 0:DELBA state from Block Ack Action
 *  ampdu_tx - 0: IEEE80211_AMPDU_RX_xxx, 1: IEEE80211_AMPDU_TX_xxx
 *  Negotiated BA parameters for TID
 */
typedef struct cci_cmd_sta_ba
{
    cci_comm_hdr_t  cmd_hdr;

    uint16_t sta_id;
    uint16_t ssn: 12;       /* Fragment Number is always 0 */
    uint16_t active: 1;     /* 0:disable, 1:enable  */
    uint16_t ampdu_tx: 1;   /* 0:recipient(RX), 1:originator(TX) */
    uint16_t rsvd: 2;       /* Table 9-265a—HE Fragmentation Operation subfield */

    /* Figure 9-79 Block Ack Parameter Set fixed field */
    uint16_t amsdu_supp: 1; /* IEEE80211_ADDBA_PARAM_AMSDU_MASK 0x0001 */
    uint16_t immed_ba: 1;   /* IEEE80211_ADDBA_PARAM_POLICY_MASK 0x0002 */
    uint16_t tid: 4;        /* #define IEEE80211_ADDBA_PARAM_TID_MASK 0x003C */
    uint16_t buf_size: 10;  /* #define IEEE80211_ADDBA_PARAM_BUF_SIZE_MASK 0xFFC0 */

    uint16_t ba_timeout;    /* duration in TUs, disabled(0), 9.4.1.15 BA Timeout Value field */
} cci_cmd_sta_ba_t;

enum Ecci_cmd_group_fwdbg_stats {
    /* SOC stats                 */
    CCI_CMD_ID_FWDBG_UMAC_TX_STATS,
    CCI_CMD_ID_FWDBG_UMAC_RX_STATS,
    /* lmac stats per ACQ       */
    CCI_CMD_ID_FWDBG_LMAC_TX_STATS,
    CCI_CMD_ID_FWDBG_LMAC1_TX_STATS,
    CCI_CMD_ID_FWDBG_LMAC_RX_STATS,
    CCI_CMD_ID_FWDBG_LMAC1_RX_STATS,
    /* Rate stats               */
    CCI_CMD_ID_FWWDBG_TX_RATE_STATS,
    CCI_CMD_ID_FWWDBG_RX_RATE_STATS,
    /* Throughput stats         */
    CCI_CMD_ID_FWWDBG_TX_THROUGHPUT_STATS,
    CCI_CMD_ID_FWWDBG_RX_THROUGHPUT_STATS,
    /* VIF stats                */
    CCI_CMD_ID_FWDBFG_VIF_STATS,
    /* STA stats                */
    CCI_CMD_ID_FWDBFG_STA_STATS,
    /* Profiler and PMU stats   */
    CCI_CMD_ID_FWWDBG_PROFILER_STATS,
};

enum Ecci_cmd_group_vendor_cmd {
    /* Vendor command */
    CCI_CMD_ID_VENDOR_CMD_RATECFG,
};

/** Vendor cmd parameter that is used in TLV of CCI_CMD_ID_VENDOR_CMD_RATECFG
    64 max. If required more than 64, create a new CMD. */
typedef enum Ecci_rate_param {
    CCI_CMD_ID_VENDOR_PARAM_PPDU_CLASS,
    CCI_CMD_ID_VENDOR_PARAM_PPDU_BW,
    CCI_CMD_ID_VENDOR_PARAM_PPDU_GI,
    CCI_CMD_ID_VENDOR_PARAM_PPDU_LTF,
    CCI_CMD_ID_VENDOR_PARAM_PPDU_MCS,
    CCI_CMD_ID_VENDOR_PARAM_PPDU_NSS,
    /* TBD... */
    
    CCI_CMD_ID_VENDOR_PARAM_MAX = (sizeof(uint64_t) - 1)
} Ecci_rate_param_t;

/**
 * @struct cci_cmd_rate_set_param_t
 * @brief cci command sent from host to mac to set a particular rate param
 * param1 - 1st parameter TLV [and more] until cmd_hdr.length.
 *
 */
typedef struct cci_cmd_rate_set_param {
    cci_comm_hdr_t cmd_hdr;
    cci_comm_tlv_t param1;      /** 1st TLV from Ecci_rate_param_t */
} cci_cmd_rate_set_param_t;

/*=== CCI Command - End */

/*=== CCI Event */

enum Ecci_evt_groups {
    CCI_EVT_GRP_SYSTEM,
    CCI_EVT_GRP_RADIO,
    CCI_EVT_GRP_VIF,
    CCI_EVT_GRP_STA,
    CCI_EVT_GRP_FWDBG_STATS,
    CCI_EVT_GRP_VENDOR_CMD,
    CCI_EVT_GRP_MAX,
};

enum Ecci_evt_group_system {    
    /** CCI service is ready; after this event CCI messages can be sent/received  */
    CCI_EVT_ID_SYSTEM_READY,
    /** CCI is ready; after this event the wlan firmware is initialized and can process commands. */
    CCI_EVT_ID_SYSTEM_INIT_DONE,
    CCI_EVT_ID_SYSTEM_MEM_DUMP,
    CCI_EVT_ID_SYSTEM_MAX,
};

typedef struct cci_cap_phy_channel {
    uint32_t center_freq;
    uint32_t hw_value;
} cci_cap_phy_channel_t;

typedef struct cci_cap_phy_channel_list {
    uint32_t radio_id;
    uint32_t band;           /* CCI_NL80211_BAND_xxx */
    uint32_t num_channels;   /* number of supported channel info following */
    cci_cap_phy_channel_t   info[0];    /* 1st channel info out of [num_channels] */
} cci_cap_phy_channel_list_t;

/**
 * @struct cci_evt_system_ready_t
 * @brief cci service ready event sent to host from mac
 * This event goes from mac to host advertising 
 * radio/phy level capabilities, firmware supported features.
 * 
 */
typedef struct cci_evt_system_ready {
    cci_comm_hdr_t      evt_hdr;
    uint32_t            cci_ver_major : 16,
                        cci_ver_minor :  8,
                        reserved_2    :  8;
    uint32_t            sys_rsvd;

    cci_cap_sys_t       sys_capabilities;
    cci_cap_mac_t       mac_capabilities;
    cci_cap_phy_t       phy_capabilities[CCI_MAX_RADIO];

    cci_ieee_caps_t     ieee_caps_5g;
    cci_ieee_caps_t     ieee_caps_2g;

    uint32_t            num_channel_list;
    cci_cap_phy_channel_list_t channels;	/* 1st channel list with variable length */
} cci_evt_system_ready_t;

/**
 * @struct cci_evt_system_init_done_t
 * @brief cci ready event sent from to host from mac
 * This event is to inform host that firmware initialization is done.
 * 
 */
typedef struct cci_evt_system_init_done {
    cci_comm_hdr_t      evt_hdr;
    uint32_t            init_status;
} cci_evt_system_init_done_t;

/** VIF Events ******************/
enum Ecci_evt_group_vif {
    /* Response to: Get a VIF configuration using TLV format */
    CCI_EVT_ID_VIF_GET_PARAM,
    /* Response to: Delete VIF entry */
    CCI_EVT_ID_VIF_DELETE,
};

/**
 * @struct cci_evt_vif_rsp_t
 * @brief cci common format for simple ACK response to VIF commands
 *        Applied to CCI_EVT_ID_VIF_DELETE,
 */
typedef struct cci_evt_vif_rsp
{
    cci_comm_hdr_t      evt_hdr;

    uint16_t vif_id;
    uint16_t rsvd;
    uint32_t err_code;  /* Only if there's an error */

    uint32_t params[4]; /* small information to return */
} cci_evt_vif_rsp_t;


/** STA Events ******************/
enum Ecci_evt_group_sta {
    /* Response to: Create STA entry */
    CCI_EVT_ID_STA_ADD,
    /* Response to: Get a STA configuration using TLV format */
    CCI_EVT_ID_STA_GET_PARAM,
    /* Response to: Delete STA entry */
    CCI_EVT_ID_STA_DELETE,
};

/**
 * @struct cci_evt_system_sta_add_t
 * @brief cci event sent from to host from mac
 * in response to sta_add command status
 *
 */
typedef struct cci_evt_sta_add {
    cci_comm_hdr_t      evt_hdr;

    uint16_t vif_id;
    uint16_t sta_id;
    uint32_t err_code;  /* Only if there's an error */
} cci_evt_sta_add_t;

/**
 * @struct cci_evt_sta_rsp_t
 * @brief cci common format for simple ACK response to STA commands
 *        Applied to CCI_EVT_ID_STA_DELETE,
 */
typedef struct cci_evt_sta_rsp
{
    cci_comm_hdr_t      evt_hdr;

    uint16_t sta_id;
    uint16_t rsvd;
    uint32_t err_code;  /* Only if there's an error */

    uint32_t params[4]; /* small information to return */
} cci_evt_sta_rsp_t;

/** FWDBG STATS Events ******************/

enum Ecci_evt_group_fwdbg_stats {
    /* SOC stats                 */
    CCI_EVT_ID_FWDBG_UMAC_TX_STATS,
    CCI_EVT_ID_FWDBG_UMAC_RX_STATS,
    /* Radio stats per ACQ       */
    CCI_EVT_ID_FWDBG_LMAC_TX_STATS,
    CCI_EVT_ID_FWDBG_LMAC_RX_STATS,
    /* Rate stats               */
    CCI_EVT_ID_FWWDBG_TX_RATE_STATS,
    CCI_EVT_ID_FWWDBG_RX_RATE_STATS,
    CCI_EVT_ID_FWWDBG_TX_THROUGHPUT_STATS,
    CCI_EVT_ID_FWWDBG_RX_THROUGHPUT_STATS,
    /* VIF stats                */
    CCI_EVT_ID_FWDBFG_VIF_STATS,
    /* STA stats                */
    CCI_EVT_ID_FWDBFG_STA_STATS,
    /* Profiler and PMU stats   */
    CCI_EVT_ID_FWWDBG_PROFILER_STATS,
};

/**
 * @struct cci_evt_fwdbg_umac_txstats
 * @brief cci event sent from to host from mac
 * in response to umac_tx_stats command status
 *
 */
typedef struct cci_evt_fwdbg_umac_txstats {
    cci_comm_hdr_t      evt_hdr;
    uint64_t msdu_enqueued;
    uint64_t msdu_bytes_txed;
    uint64_t mpdugen_req;
    uint64_t mpdugen_req_reaped;
    uint64_t mpdugen_req_completed;
    uint64_t mpdugen_with_no_mpdus; 
    uint64_t msdu_reaped[e_max_msdu_desc_status];
} cci_evt_fwdbg_umac_txstats_t;

typedef struct lmactx_per_radio_stats
{
    uint32_t num_txop_sent;
    uint32_t num_txop_free;
    uint32_t num_txop_success;
    uint32_t num_txop_fail;
    uint32_t num_tx_ppdus_sent;
    uint32_t num_tx_ppdu_fail;
    uint32_t num_tx_data_ppdus_sent;
    uint32_t num_tx_beacon_ppdu;
    uint32_t num_tx_mcast_ppdu;
    uint32_t num_tx_mpdus_tried;
    uint32_t num_tx_tcp_bytes_tried;
    uint32_t num_tx_udp_bytes_tried;
    uint32_t num_tx_ppdu_resp;
    uint32_t num_tx_ack_ba;
    uint32_t num_tx_ul_ba;
    uint32_t num_tx_ul_ppdu;
    uint32_t num_tx_rts;
    uint32_t num_tx_bar;
    uint32_t num_cbf;
    uint32_t num_tx_mpdus_ok;
    uint32_t num_tx_tcp_bytes_ok;
    uint32_t num_tx_udp_bytes_ok;
    uint32_t tx_mxl_fail_cnt;
    uint32_t tx_mxl_lenmismatch_abort_cnt;
    uint32_t tx_mxl_earlyeof_abort_cnt;
    uint32_t tx_mxl_lateeof_abort_cnt;
    uint32_t tx_mxl_hdrparsing_error_cnt;
}lmactx_per_radio_stats_t;

typedef struct cci_evt_fwdbg_lmac_txstats {
    cci_comm_hdr_t      evt_hdr;
    uint32_t radio_id;
    lmactx_per_radio_stats_t lmac_tx_stats[MAX_NUM_ACQ];
    
} cci_evt_fwdbg_lmac_txstats_t;



typedef struct tx_tput_stats{
    uint64_t num_msdus_enqueued_per_second;
    uint64_t num_msdu_bytes_per_second;
    uint64_t num_msdus_freed_per_second;
    uint64_t num_crss_cmds_per_second;
    uint64_t num_mpdus_generated_per_second;

}tx_tput_stats_t;

#define MAX_TPUT_IDX 10
typedef struct cci_evt_fwdbg_tx_tput_stats {
    cci_comm_hdr_t      evt_hdr;
    tx_tput_stats_t     tx_tput[MAX_TPUT_IDX];

} cci_evt_fwdbg_tx_tput_stats_t;

typedef struct rx_tput_stats
{
    uint64_t num_mpdus_received_per_second;
    uint64_t num_decryption_per_second;
    uint64_t num_host_mpdu_indication_per_second;
}rx_tput_stats_t;

typedef struct cci_evt_fwdbg_rx_tput_stats {
    cci_comm_hdr_t      evt_hdr;
    rx_tput_stats_t     rx_tput[MAX_TPUT_IDX];

} cci_evt_fwdbg_rx_tput_stats_t;
/** VENDOR COMMAND Events ******************/

enum Ecci_evt_group_vendor_cmd {
    CCI_EVT_ID_VENDOR_CMD_RATECFG,
};

/*=== CCI Event - End */

#endif

#endif /* #ifndef CCI_H */


