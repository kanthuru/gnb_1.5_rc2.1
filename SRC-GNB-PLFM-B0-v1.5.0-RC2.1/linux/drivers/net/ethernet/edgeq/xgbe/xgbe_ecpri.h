/* SPDX-License-Identifier: GPL-2.0-only */
/*******************************************************************************
  Copyright (C) 2020  EdgeQ

*******************************************************************************/

#ifndef __XGBE_ECPRI_H__
#define __XGBE_ECPRI_H__

#include <net/snmp.h>

#define MIN_RX_CHANS_WITH_ECPRI		2	// One for ecpri
#define MIN_TX_CHANS_WITH_ECPRI		2	// One for ecpri
#define MIN_TX_CHANS_WITH_ECPRI_L1	3	// 4 ecpri chans for Link1

#define ECPRI_XGMAC_QCNT		4	// Four queues for ECPRI XGMAC
#define ECPRI_RX_DESC_OFFSET		0x100
#define ECPRI_XGMAC_TX_DESC_OFFSET	0x200
#define ECPRI_XLGMAC_TX_DESC_OFFSET	0x500
#define ECPRI_XGMAC_NOR_BUF_SIZE	0x8000	// 32K
#define ECPRI_XGMAC_JUMBO_BUF_SIZE	0x9000	// 36K
#define ECPRI_XLGMAC_NOR_BUF_SIZE	0x20000	// 128K
#define ECPRI_XLGMAC_JUMBO_BUF_SIZE	0x24000	// 144K
#define ECPRI_DMA_STATE_STOP		0		
#define ECPRI_DMA_STATE_START		1		
#define ECPRI_DMA_STATE_RESUME		2		
#define ECPRI_HOST_BUF_RING_SIZE	256
#define XGMAC_HOST_BUF_OFFSET		0x12300
#define XLGMAC_HOST_BUF_OFFSET		0x36900

#define UDP_ADD_STATS(net, field, val)	\
			SNMP_ADD_STATS((net)->mib.udp_statistics, field, val)

struct ecpri_ip_addr {
	u64 addr_hi;		// IPv6 hi addr or 0 for IPv4
	u64 addr_lo;		// IPv6 lo addr or IPv4 addr
};

struct ecpri_mac {
	u8 addr[ETH_ALEN];
};

struct __attribute__((__packed__)) ecpri_ctrl_status {
	u16 rx_desc_offset;	// The size of config and status section (256)
	u16 rx_desc_ring_size;	// 16, 8 or 4 for XGMAC; 64, 32 or 16 for XLGMAC
	u16 rx_buf_size;	// 2K, 4K, 8K or 9K depending on MTU size
	u16 tx_desc_ring_size;	// 16, 8 or 4 for XGMAC or per XLGMAC Tx queue
	u16 tx_buf_size;        // 2K, 4K, 8K or 9K depending on MTU size
	u16 udp_port;		// UDP port for eCPRI; 0 for eCPRI over Ethernet
	u16 vlan_tag[4];	// VLAN tag (0 for non-VLAN); upto 4 for XLGMAC
	struct ecpri_ip_addr ip_addr[4];
				// Dest IP addr for eCPRI over UDP; 4 for XLGMAC
	u8 is_ipv4[4]; 		// IPv4 or IPv6 address
	u32 rx_pkt_cnt;		// Rx eCPRI packet count (stats)
	u32 rx_byte_cnt;	// Rx bytes count (stats)
	u32 rx_drop_cnt;	// Rx eCPRI dropped packet count (stats)
	u32 rx_err_cnt;		// Rx eCPRI error packet count (stats)
	u32 tx_pkt_cnt[4];	// Tx eCPRI packet count (stats); 4 for XLGMAC
	u32 tx_byte_cnt[4];	// Tx bytes count (stats); 4 for XLGMAC
	u32 tx_drop_cnt[4];	// Tx eCPRI dropped packet count; 4 for XLGMAC
	u32 tx_err_cnt[4];	// Tx eCPRI error packet count; 4 for XLGMAC
	struct ecpri_mac dst_mac[4];
				// Dest MAC addr (or gateway MAC); 4 for XLGMAC
	struct ecpri_mac src_mac;
				// Source MAC addr
	u8 tx_dma_ch[4];	// Tx DMA chan for eCPRI; 4 for XLGMAC 
	u8 rx_dma_ch;		// DMA chan for eCPRI
	u8 eth_link;		// 1 for link1 or 2 for link2
	u8 curr_state;		// Tell eCPRI SS to start, stop, resume, etc
	u8 tx_ring_full;	// Indicate no free Tx descriptor available
	u8 num_tx_q;		// Total number Tx queues
	u8 reserve[49];		// Make it total 256 byte for this structure
};

struct __attribute__((__packed__)) ecpri_host_buf_hdr {
	u16 ring_size;		// number of entries
	u16 head;		// head index
	u16 tail;		// tail index
	u8 buff_overflow;	// buffer over flow flag
	u8 reserve[57];		// make it 64 bytes
};

struct ecpri_host_buf {
	struct sk_buff *skb;
	dma_addr_t addr;
};

void ecpri_setup_config(struct xgmac_priv *priv);
int ecpri_alloc_rx_resources(struct xgmac_priv *priv, u32 queue);
int ecpri_init_rx_rings(struct xgmac_priv *priv, u32 queue);
void ecpri_free_rx_resources(struct xgmac_priv *priv, u32 queue);
int ecpri_alloc_tx_resources(struct xgmac_priv *priv, u32 queue);
void ecpri_rx_queue_dma_chan_map(struct xgmac_priv *priv);
void ecpri_rxqueues_routing(struct xgmac_priv *priv);
int ecpri_napi_poll_rx(struct napi_struct *napi, int budget);
int ecpri_napi_poll_tx(struct napi_struct *napi, int budget);
void ecpri_start(struct xgmac_priv *priv);
void ecpri_stop(struct xgmac_priv *priv);
void ecpri_resume(struct xgmac_priv *priv);
int ecpri_set_udp_port(struct net_device *dev, struct ifreq *ifr);
int ecpri_get_udp_port(struct net_device *dev, struct ifreq *ifr);
int ecpri_set_vlan(struct net_device *dev, struct ifreq *ifr);
int ecpri_get_vlan(struct net_device *dev, struct ifreq *ifr);
int ecpri_set_dst_ip(struct net_device *dev, struct ifreq *ifr);
int ecpri_get_dst_ip(struct net_device *dev, struct ifreq *ifr);
int ecpri_set_dst_mac(struct net_device *dev, struct ifreq *ifr);
int ecpri_get_dst_mac(struct net_device *dev, struct ifreq *ifr);

#endif // __XGBE_ECPRI_H__
