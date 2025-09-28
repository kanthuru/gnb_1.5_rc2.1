/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
  XGMAC Common Header File

*******************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/module.h>
#if IS_ENABLED(CONFIG_VLAN_8021Q)
#define XGMAC_VLAN_TAG_USED
#include <linux/if_vlan.h>
#endif

#include "xgmac_plat_data.h"
#include "descs.h"
#include "hwif.h"
#include "mmc.h"

#define KB(x) ((x) * 1024)
#define MB(x) ((x) * KB(1024))

/* Synopsys Core versions */
#define DWXGMAC_CORE_2_10	0x21
#define DWXLGMAC_CORE_2_00	0x20

/* Device ID */
#define DWXGMAC_ID		0x76
#define DWXLGMAC_ID		0x27

/* These need to be power of two, and >= 4 */
#define DMA_TX_SIZE		512
#define DMA_RX_SIZE		512
#define XGMAC_GET_ENTRY(x, size)	((x + 1) & (size - 1))

/* Extra statistic and debug information exposed by ethtool */
struct xgmac_extra_stats {
	/* Transmit errors */
	unsigned long tx_underflow ____cacheline_aligned;
	unsigned long tx_carrier;
	unsigned long tx_losscarrier;
	unsigned long vlan_tag;
	unsigned long tx_deferred;
	unsigned long tx_vlan;
	unsigned long tx_jabber;
	unsigned long tx_frame_flushed;
	unsigned long tx_payload_error;
	unsigned long tx_ip_header_error;
	/* Receive errors */
	unsigned long rx_desc;
	unsigned long sa_filter_fail;
	unsigned long overflow_error;
	unsigned long ipc_csum_error;
	unsigned long rx_collision;
	unsigned long rx_crc_errors;
	unsigned long dribbling_bit;
	unsigned long rx_length;
	unsigned long rx_mii;
	unsigned long rx_multicast;
	unsigned long rx_gmac_overflow;
	unsigned long rx_watchdog;
	unsigned long da_rx_filter_fail;
	unsigned long sa_rx_filter_fail;
	unsigned long rx_missed_cntr;
	unsigned long rx_overflow_cntr;
	unsigned long rx_vlan;
	unsigned long rx_split_hdr_pkt_n;
	/* Tx/Rx IRQ error info */
	unsigned long tx_undeflow_irq;
	unsigned long tx_process_stopped_irq;
	unsigned long tx_jabber_irq;
	unsigned long rx_overflow_irq;
	unsigned long rx_buf_unav_irq;
	unsigned long rx_process_stopped_irq;
	unsigned long rx_watchdog_irq;
	unsigned long tx_early_irq;
	unsigned long fatal_bus_error_irq;
	/* Tx/Rx IRQ Events */
	unsigned long rx_early_irq;
	unsigned long threshold;
	unsigned long tx_pkt_n;
	unsigned long rx_pkt_n;
	unsigned long normal_irq_n;
	unsigned long rx_normal_irq_n;
	unsigned long napi_poll;
	unsigned long tx_normal_irq_n;
	unsigned long tx_clean;
	unsigned long tx_set_ic_bit;
	unsigned long irq_receive_pmt_irq_n;
	/* MMC info */
	unsigned long mmc_tx_irq_n;
	unsigned long mmc_rx_irq_n;
	unsigned long mmc_rx_csum_offload_irq_n;
	/* EEE */
	unsigned long irq_tx_path_in_lpi_mode_n;
	unsigned long irq_tx_path_exit_lpi_mode_n;
	unsigned long irq_rx_path_in_lpi_mode_n;
	unsigned long irq_rx_path_exit_lpi_mode_n;
	unsigned long phy_eee_wakeup_error_n;
	/* Extended RDES status */
	unsigned long ip_hdr_err;
	unsigned long ip_payload_err;
	unsigned long ip_csum_bypassed;
	unsigned long ipv4_pkt_rcvd;
	unsigned long ipv6_pkt_rcvd;
	unsigned long no_ptp_rx_msg_type_ext;
	unsigned long ptp_rx_msg_type_sync;
	unsigned long ptp_rx_msg_type_follow_up;
	unsigned long ptp_rx_msg_type_delay_req;
	unsigned long ptp_rx_msg_type_delay_resp;
	unsigned long ptp_rx_msg_type_pdelay_req;
	unsigned long ptp_rx_msg_type_pdelay_resp;
	unsigned long ptp_rx_msg_type_pdelay_follow_up;
	unsigned long ptp_rx_msg_type_announce;
	unsigned long ptp_rx_msg_type_management;
	unsigned long ptp_rx_msg_pkt_reserved_type;
	unsigned long ptp_frame_type;
	unsigned long ptp_ver;
	unsigned long timestamp_dropped;
	unsigned long av_pkt_rcvd;
	unsigned long av_tagged_pkt_rcvd;
	unsigned long vlan_tag_priority_val;
	unsigned long l3_filter_match;
	unsigned long l4_filter_match;
	unsigned long l3_l4_filter_no_match;
	/* PCS */
	unsigned long irq_pcs_ane_n;
	unsigned long irq_pcs_link_n;
	unsigned long irq_rgmii_n;
	unsigned long pcs_link;
	unsigned long pcs_duplex;
	unsigned long pcs_speed;
	/* debug register */
	unsigned long mtl_tx_status_fifo_full;
	unsigned long mtl_tx_fifo_not_empty;
	unsigned long mmtl_fifo_ctrl;
	unsigned long mtl_tx_fifo_read_ctrl_write;
	unsigned long mtl_tx_fifo_read_ctrl_wait;
	unsigned long mtl_tx_fifo_read_ctrl_read;
	unsigned long mtl_tx_fifo_read_ctrl_idle;
	unsigned long mac_tx_in_pause;
	unsigned long mac_tx_frame_ctrl_xfer;
	unsigned long mac_tx_frame_ctrl_idle;
	unsigned long mac_tx_frame_ctrl_wait;
	unsigned long mac_tx_frame_ctrl_pause;
	unsigned long mac_gmii_tx_proto_engine;
	unsigned long mtl_rx_fifo_fill_level_full;
	unsigned long mtl_rx_fifo_fill_above_thresh;
	unsigned long mtl_rx_fifo_fill_below_thresh;
	unsigned long mtl_rx_fifo_fill_level_empty;
	unsigned long mtl_rx_fifo_read_ctrl_flush;
	unsigned long mtl_rx_fifo_read_ctrl_read_data;
	unsigned long mtl_rx_fifo_read_ctrl_status;
	unsigned long mtl_rx_fifo_read_ctrl_idle;
	unsigned long mtl_rx_fifo_ctrl_active;
	unsigned long mac_rx_frame_ctrl_fifo;
	unsigned long mac_gmii_rx_proto_engine;
	/* TSO */
	unsigned long tx_tso_frames;
	unsigned long tx_tso_nfrags;
};

/* Safety Feature statistics exposed by ethtool */
struct xgmac_safety_stats {
	unsigned long mac_errors[32];
	unsigned long mtl_errors[32];
	unsigned long dma_errors[32];
};

/* Number of fields in Safety Stats */
#define XGMAC_SAFETY_FEAT_SIZE	\
	(sizeof(struct xgmac_safety_stats) / sizeof(unsigned long))

/* CSR Frequency Access Defines*/
#define CSR_F_35M		35000000
#define CSR_F_60M		60000000
#define CSR_F_100M		100000000
#define CSR_F_150M		150000000
#define CSR_F_250M		250000000
#define CSR_F_300M		300000000
#define CSR_F_350M		350000000
#define CSR_F_400M		400000000

#define	MAC_CSR_H_FRQ_MASK	0x20

#define HASH_TABLE_SIZE		64
#define PAUSE_TIME		0xffff

/* Flow Control defines */
#define FLOW_OFF		0
#define FLOW_RX			1
#define FLOW_TX			2
#define FLOW_AUTO		(FLOW_TX | FLOW_RX)

/* PCS defines */
#define XGMAC_PCS_RGMII		(1 << 0)
#define XGMAC_PCS_SGMII		(1 << 1)
#define XGMAC_PCS_TBI		(1 << 2)
#define XGMAC_PCS_RTBI		(1 << 3)

#define SF_DMA_MODE		1  /* DMA STORE-AND-FORWARD Operation Mode */
#define DEFAULT_DMA_PBL		8

/* Max/Min RI Watchdog Timer count value */
#define MAX_DMA_RIWT		0xff
#define MIN_DMA_RIWT		0x4
#define DEF_DMA_RIWT		0x5E
#define DEF_DMA_RWTU		2  /* (256 << 2) clock cycles per RWT unit */

/* Tx coalesce parameters */
#define XGMAC_COAL_TX_TIMER	1000
#define XGMAC_MAX_COAL_TX_TICK	100000
#define XGMAC_TX_MAX_FRAMES	256
#define XGMAC_TX_FRAMES		25
#define XGMAC_RX_FRAMES		128
#define XGMAC_MAX_RX_FRAMES	1023

/* Packets types */
enum packets_types {
	PACKET_AVCPQ = 0x1, /* AV Untagged Control packets */
	PACKET_PTPQ = 0x2, /* PTP Packets */
	PACKET_DCBCPQ = 0x3, /* DCB Control Packets */
	PACKET_UPQ = 0x4, /* Untagged Packets */
	PACKET_MCBCQ = 0x5, /* Multicast & Broadcast Packets */
};

/* Rx IPC status */
enum rx_frame_status {
	good_frame = 0x0,
	discard_frame = 0x1,
	csum_none = 0x2,
	llc_snap = 0x4,
	dma_own = 0x8,
	rx_not_ls = 0x10,
};

/* Tx status */
enum tx_frame_status {
	tx_done = 0x0,
	tx_not_ls = 0x1,
	tx_err = 0x2,
	tx_dma_own = 0x4,
};

enum dma_irq_status {
	tx_hard_error = 0x1,
	tx_hard_error_bump_tc = 0x2,
	handle_rx = 0x4,
	handle_tx = 0x8,
	handle_rbu = 0x10,
};

/* EEE and LPI defines */
#define	CORE_IRQ_TX_PATH_IN_LPI_MODE	(1 << 0)
#define	CORE_IRQ_TX_PATH_EXIT_LPI_MODE	(1 << 1)
#define	CORE_IRQ_RX_PATH_IN_LPI_MODE	(1 << 2)
#define	CORE_IRQ_RX_PATH_EXIT_LPI_MODE	(1 << 3)
#define	CORE_IRQ_LINK_FAULT		(1 << 4)

#define CORE_IRQ_MTL_RX_OVERFLOW	BIT(8)

/* Physical Coding Sublayer */
struct rgmii_adv {
	unsigned int pause;
	unsigned int duplex;
	unsigned int lp_pause;
	unsigned int lp_duplex;
};

#define XGMAC_PCS_PAUSE		1
#define XGMAC_PCS_ASYM_PAUSE	2

/* DMA HW capabilities */
struct dma_features {
	unsigned int mbps_10_100;
	unsigned int mbps_1000;
	unsigned int half_duplex;
	unsigned int hash_filter;
	unsigned int multi_addr;
	unsigned int pcs;
	unsigned int sma_mdio;
	unsigned int pmt_remote_wake_up;
	unsigned int pmt_magic_frame;
	unsigned int rmon;
	/* IEEE 1588-2008 */
	unsigned int time_stamp;
	/* 802.3az - Energy-Efficient Ethernet (EEE) */
	unsigned int eee;
	unsigned int av;
	unsigned int hash_tb_sz;
	unsigned int tsoen;
	/* TX and RX csum */
	unsigned int tx_coe;
	unsigned int rx_coe;
	unsigned int rx_coe_type1;
	unsigned int rx_coe_type2;
	unsigned int rxfifo_over_2048;
	/* TX and RX number of channels */
	unsigned int number_rx_channel;
	unsigned int number_tx_channel;
	/* TX and RX number of queues */
	unsigned int number_rx_queues;
	unsigned int number_tx_queues;
	/* PPS output */
	unsigned int pps_out_num;
	/* Alternate (enhanced) DESC mode */
	unsigned int enh_desc;
	/* TX and RX FIFO sizes */
	unsigned int tx_fifo_size;
	unsigned int rx_fifo_size;
	/* Automotive Safety Package */
	unsigned int asp;
	/* RX Parser */
	unsigned int frpsel;
	unsigned int frpbs;
	unsigned int frpes;
	unsigned int addr64;
	unsigned int rssen;
	unsigned int vlhash;
	unsigned int sphen;
	unsigned int vlins;
	unsigned int dvlan;
	unsigned int l3l4fnum;
	unsigned int arpoffsel;
	unsigned int numtc;
	/* TSN Features */
	unsigned int estwid;
	unsigned int estdep;
	unsigned int estsel;
	unsigned int fpesel;
	unsigned int tbssel;
	/* PTP timestamping */
	unsigned int tstamp_hword;
	unsigned int ptoen;
	unsigned int osten;
	/* Auxiliary snapshot number */
	unsigned int aux_snap_num;
};

/* RX Buffer size must be multiple of 4/8/16 bytes */
#define BUF_SIZE_16KiB		16368
#define BUF_SIZE_9KiB		9216
#define BUF_SIZE_8KiB		8192
#define BUF_SIZE_4KiB		4096
#define BUF_SIZE_2KiB		2048

/* Power Down and WOL */
#define PMT_NOT_SUPPORTED	0
#define PMT_SUPPORTED		1

/* Default LPI timers */
#define XGMAC_DEFAULT_LIT_LS	0x3E8
#define XGMAC_DEFAULT_TWT_LS	0x1E

#define XGMAC_CHAIN_MODE	0x1
#define XGMAC_RING_MODE		0x2

#define JUMBO_LEN		9000

/* Receive Side Scaling */
#define XGMAC_RSS_HASH_KEY_SIZE	40
#define XGMAC_RSS_MAX_TABLE_SIZE 256
#define XGMAC_RSS_MIN_TABLE_SIZE 32

/* VLAN */
#define XGMAC_VLAN_NONE		0x0
#define XGMAC_VLAN_REMOVE	0x1
#define XGMAC_VLAN_INSERT	0x2
#define XGMAC_VLAN_REPLACE	0x3

extern const struct xgmac_desc_ops enh_desc_ops;
extern const struct xgmac_desc_ops ndesc_ops;

struct mac_device_info;

extern const struct xgmac_hwtimestamp xgmac_ptp;

struct mac_link {
	u32 speed_mask;
	u32 speed10;
	u32 speed100;
	u32 speed1000;
	u32 speed2500;
	u32 duplex;
	struct {
		u32 speed2500;
		u32 speed5000;
		u32 speed10000;
	} xgmii;
	struct {
		u32 speed25000;
		u32 speed40000;
		u32 speed50000;
		u32 speed100000;
	} xlgmii;
};

struct mii_regs {
	unsigned int addr;	/* MII Address */
	unsigned int data;	/* MII Data */
	unsigned int addr_shift;	/* MII address shift */
	unsigned int reg_shift;		/* MII reg shift */
	unsigned int addr_mask;		/* MII address mask */
	unsigned int reg_mask;		/* MII reg mask */
	unsigned int clk_csr_shift;
	unsigned int clk_csr_mask;
};

struct mac_device_info {
	const struct xgmac_ops *mac;
	const struct xgmac_desc_ops *desc;
	const struct xgmac_dma_ops *dma;
	const struct xgmac_mode_ops *mode;
	const struct xgmac_hwtimestamp *ptp;
	const struct xgmac_tc_ops *tc;
	const struct xgmac_mmc_ops *mmc;
	const struct xpcs_ops *xpcs;
	struct mdio_xpcs_args xpcs_args;
	struct mii_regs mii;	/* MII register Addresses */
	struct mac_link link;
	void __iomem *pcsr;     /* vpointer to device CSRs */
	unsigned int multicast_filter_bins;
	unsigned int unicast_filter_entries;
	unsigned int mcast_bits_log2;
	unsigned int rx_csum;
	unsigned int pcs;
	unsigned int pmt;
	unsigned int ps;
	unsigned int xlgmac;
	unsigned int num_vlan;
	u32 vlan_filter[32];
};

struct xgmac_rx_routing {
	u32 reg_mask;
	u32 reg_shift;
};

int dwxgmac2_setup(struct xgmac_priv *priv);
int dwxlgmac2_setup(struct xgmac_priv *priv);
void xgmac_set_mac_addr(void __iomem *ioaddr, u8 addr[6],
			 unsigned int high, unsigned int low);
void xgmac_get_mac_addr(void __iomem *ioaddr, unsigned char *addr,
			 unsigned int high, unsigned int low);
void xgmac_set_mac(void __iomem *ioaddr, bool enable);

#endif /* __COMMON_H__ */
