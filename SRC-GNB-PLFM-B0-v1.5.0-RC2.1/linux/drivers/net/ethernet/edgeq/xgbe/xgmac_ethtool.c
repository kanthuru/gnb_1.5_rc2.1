// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  XGMAC Ethtool support

*******************************************************************************/

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/phylink.h>
#include <linux/net_tstamp.h>
#include <asm/io.h>

#include "xgbe.h"
#include "dwxgmac.h"

#define XGMAC_ETHTOOL_NAME	"xgbe"

#define ETHTOOL_DMA_OFFSET	55

struct xgmac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
};

#define XGMAC_STAT(m)	\
	{ #m, sizeof_field(struct xgmac_extra_stats, m),	\
	offsetof(struct xgmac_priv, xstats.m)}

static const struct xgmac_stats xgmac_gstrings_stats[] = {
	/* Transmit errors */
	XGMAC_STAT(tx_underflow),
	XGMAC_STAT(tx_carrier),
	XGMAC_STAT(tx_losscarrier),
	XGMAC_STAT(vlan_tag),
	XGMAC_STAT(tx_deferred),
	XGMAC_STAT(tx_vlan),
	XGMAC_STAT(tx_jabber),
	XGMAC_STAT(tx_frame_flushed),
	XGMAC_STAT(tx_payload_error),
	XGMAC_STAT(tx_ip_header_error),
	/* Receive errors */
	XGMAC_STAT(rx_desc),
	XGMAC_STAT(sa_filter_fail),
	XGMAC_STAT(overflow_error),
	XGMAC_STAT(ipc_csum_error),
	XGMAC_STAT(rx_collision),
	XGMAC_STAT(rx_crc_errors),
	XGMAC_STAT(dribbling_bit),
	XGMAC_STAT(rx_length),
	XGMAC_STAT(rx_mii),
	XGMAC_STAT(rx_multicast),
	XGMAC_STAT(rx_gmac_overflow),
	XGMAC_STAT(rx_watchdog),
	XGMAC_STAT(da_rx_filter_fail),
	XGMAC_STAT(sa_rx_filter_fail),
	XGMAC_STAT(rx_missed_cntr),
	XGMAC_STAT(rx_overflow_cntr),
	XGMAC_STAT(rx_vlan),
	XGMAC_STAT(rx_split_hdr_pkt_n),
	/* Tx/Rx IRQ error info */
	XGMAC_STAT(tx_undeflow_irq),
	XGMAC_STAT(tx_process_stopped_irq),
	XGMAC_STAT(tx_jabber_irq),
	XGMAC_STAT(rx_overflow_irq),
	XGMAC_STAT(rx_buf_unav_irq),
	XGMAC_STAT(rx_process_stopped_irq),
	XGMAC_STAT(rx_watchdog_irq),
	XGMAC_STAT(tx_early_irq),
	XGMAC_STAT(fatal_bus_error_irq),
	/* Tx/Rx IRQ Events */
	XGMAC_STAT(rx_early_irq),
	XGMAC_STAT(threshold),
	XGMAC_STAT(tx_pkt_n),
	XGMAC_STAT(rx_pkt_n),
	XGMAC_STAT(normal_irq_n),
	XGMAC_STAT(rx_normal_irq_n),
	XGMAC_STAT(napi_poll),
	XGMAC_STAT(tx_normal_irq_n),
	XGMAC_STAT(tx_clean),
	XGMAC_STAT(tx_set_ic_bit),
	XGMAC_STAT(irq_receive_pmt_irq_n),
	/* MMC info */
	XGMAC_STAT(mmc_tx_irq_n),
	XGMAC_STAT(mmc_rx_irq_n),
	XGMAC_STAT(mmc_rx_csum_offload_irq_n),
	/* EEE */
	XGMAC_STAT(irq_tx_path_in_lpi_mode_n),
	XGMAC_STAT(irq_tx_path_exit_lpi_mode_n),
	XGMAC_STAT(irq_rx_path_in_lpi_mode_n),
	XGMAC_STAT(irq_rx_path_exit_lpi_mode_n),
	XGMAC_STAT(phy_eee_wakeup_error_n),
	/* Extended RDES status */
	XGMAC_STAT(ip_hdr_err),
	XGMAC_STAT(ip_payload_err),
	XGMAC_STAT(ip_csum_bypassed),
	XGMAC_STAT(ipv4_pkt_rcvd),
	XGMAC_STAT(ipv6_pkt_rcvd),
	XGMAC_STAT(no_ptp_rx_msg_type_ext),
	XGMAC_STAT(ptp_rx_msg_type_sync),
	XGMAC_STAT(ptp_rx_msg_type_follow_up),
	XGMAC_STAT(ptp_rx_msg_type_delay_req),
	XGMAC_STAT(ptp_rx_msg_type_delay_resp),
	XGMAC_STAT(ptp_rx_msg_type_pdelay_req),
	XGMAC_STAT(ptp_rx_msg_type_pdelay_resp),
	XGMAC_STAT(ptp_rx_msg_type_pdelay_follow_up),
	XGMAC_STAT(ptp_rx_msg_type_announce),
	XGMAC_STAT(ptp_rx_msg_type_management),
	XGMAC_STAT(ptp_rx_msg_pkt_reserved_type),
	XGMAC_STAT(ptp_frame_type),
	XGMAC_STAT(ptp_ver),
	XGMAC_STAT(timestamp_dropped),
	XGMAC_STAT(av_pkt_rcvd),
	XGMAC_STAT(av_tagged_pkt_rcvd),
	XGMAC_STAT(vlan_tag_priority_val),
	XGMAC_STAT(l3_filter_match),
	XGMAC_STAT(l4_filter_match),
	XGMAC_STAT(l3_l4_filter_no_match),
	/* PCS */
	XGMAC_STAT(irq_pcs_ane_n),
	XGMAC_STAT(irq_pcs_link_n),
	XGMAC_STAT(irq_rgmii_n),
	/* DEBUG */
	XGMAC_STAT(mtl_tx_status_fifo_full),
	XGMAC_STAT(mtl_tx_fifo_not_empty),
	XGMAC_STAT(mmtl_fifo_ctrl),
	XGMAC_STAT(mtl_tx_fifo_read_ctrl_write),
	XGMAC_STAT(mtl_tx_fifo_read_ctrl_wait),
	XGMAC_STAT(mtl_tx_fifo_read_ctrl_read),
	XGMAC_STAT(mtl_tx_fifo_read_ctrl_idle),
	XGMAC_STAT(mac_tx_in_pause),
	XGMAC_STAT(mac_tx_frame_ctrl_xfer),
	XGMAC_STAT(mac_tx_frame_ctrl_idle),
	XGMAC_STAT(mac_tx_frame_ctrl_wait),
	XGMAC_STAT(mac_tx_frame_ctrl_pause),
	XGMAC_STAT(mac_gmii_tx_proto_engine),
	XGMAC_STAT(mtl_rx_fifo_fill_level_full),
	XGMAC_STAT(mtl_rx_fifo_fill_above_thresh),
	XGMAC_STAT(mtl_rx_fifo_fill_below_thresh),
	XGMAC_STAT(mtl_rx_fifo_fill_level_empty),
	XGMAC_STAT(mtl_rx_fifo_read_ctrl_flush),
	XGMAC_STAT(mtl_rx_fifo_read_ctrl_read_data),
	XGMAC_STAT(mtl_rx_fifo_read_ctrl_status),
	XGMAC_STAT(mtl_rx_fifo_read_ctrl_idle),
	XGMAC_STAT(mtl_rx_fifo_ctrl_active),
	XGMAC_STAT(mac_rx_frame_ctrl_fifo),
	XGMAC_STAT(mac_gmii_rx_proto_engine),
	/* TSO */
	XGMAC_STAT(tx_tso_frames),
	XGMAC_STAT(tx_tso_nfrags),
};
#define XGMAC_STATS_LEN ARRAY_SIZE(xgmac_gstrings_stats)

/* HW MAC Management counters (if supported) */
#define XGMAC_MMC_STAT(m)	\
	{ #m, sizeof_field(struct xgmac_counters, m),	\
	offsetof(struct xgmac_priv, mmc.m)}

static const struct xgmac_stats xgmac_mmc[] = {
	XGMAC_MMC_STAT(mmc_tx_octetcount_gb),
	XGMAC_MMC_STAT(mmc_tx_framecount_gb),
	XGMAC_MMC_STAT(mmc_tx_broadcastframe_g),
	XGMAC_MMC_STAT(mmc_tx_multicastframe_g),
	XGMAC_MMC_STAT(mmc_tx_64_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_65_to_127_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_128_to_255_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_256_to_511_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_512_to_1023_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_1024_to_max_octets_gb),
	XGMAC_MMC_STAT(mmc_tx_unicast_gb),
	XGMAC_MMC_STAT(mmc_tx_multicast_gb),
	XGMAC_MMC_STAT(mmc_tx_broadcast_gb),
	XGMAC_MMC_STAT(mmc_tx_underflow_error),
	XGMAC_MMC_STAT(mmc_tx_singlecol_g),
	XGMAC_MMC_STAT(mmc_tx_multicol_g),
	XGMAC_MMC_STAT(mmc_tx_deferred),
	XGMAC_MMC_STAT(mmc_tx_latecol),
	XGMAC_MMC_STAT(mmc_tx_exesscol),
	XGMAC_MMC_STAT(mmc_tx_carrier_error),
	XGMAC_MMC_STAT(mmc_tx_octetcount_g),
	XGMAC_MMC_STAT(mmc_tx_framecount_g),
	XGMAC_MMC_STAT(mmc_tx_excessdef),
	XGMAC_MMC_STAT(mmc_tx_pause_frame),
	XGMAC_MMC_STAT(mmc_tx_vlan_frame_g),
	XGMAC_MMC_STAT(mmc_rx_framecount_gb),
	XGMAC_MMC_STAT(mmc_rx_octetcount_gb),
	XGMAC_MMC_STAT(mmc_rx_octetcount_g),
	XGMAC_MMC_STAT(mmc_rx_broadcastframe_g),
	XGMAC_MMC_STAT(mmc_rx_multicastframe_g),
	XGMAC_MMC_STAT(mmc_rx_crc_error),
	XGMAC_MMC_STAT(mmc_rx_align_error),
	XGMAC_MMC_STAT(mmc_rx_run_error),
	XGMAC_MMC_STAT(mmc_rx_jabber_error),
	XGMAC_MMC_STAT(mmc_rx_undersize_g),
	XGMAC_MMC_STAT(mmc_rx_oversize_g),
	XGMAC_MMC_STAT(mmc_rx_64_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_65_to_127_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_128_to_255_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_256_to_511_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_512_to_1023_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_1024_to_max_octets_gb),
	XGMAC_MMC_STAT(mmc_rx_unicast_g),
	XGMAC_MMC_STAT(mmc_rx_length_error),
	XGMAC_MMC_STAT(mmc_rx_autofrangetype),
	XGMAC_MMC_STAT(mmc_rx_pause_frames),
	XGMAC_MMC_STAT(mmc_rx_fifo_overflow),
	XGMAC_MMC_STAT(mmc_rx_vlan_frames_gb),
	XGMAC_MMC_STAT(mmc_rx_watchdog_error),
	XGMAC_MMC_STAT(mmc_rx_ipc_intr_mask),
	XGMAC_MMC_STAT(mmc_rx_ipc_intr),
	XGMAC_MMC_STAT(mmc_rx_ipv4_gd),
	XGMAC_MMC_STAT(mmc_rx_ipv4_hderr),
	XGMAC_MMC_STAT(mmc_rx_ipv4_nopay),
	XGMAC_MMC_STAT(mmc_rx_ipv4_frag),
	XGMAC_MMC_STAT(mmc_rx_ipv4_udsbl),
	XGMAC_MMC_STAT(mmc_rx_ipv4_gd_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv4_hderr_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv4_nopay_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv4_frag_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv4_udsbl_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv6_gd_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv6_hderr_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv6_nopay_octets),
	XGMAC_MMC_STAT(mmc_rx_ipv6_gd),
	XGMAC_MMC_STAT(mmc_rx_ipv6_hderr),
	XGMAC_MMC_STAT(mmc_rx_ipv6_nopay),
	XGMAC_MMC_STAT(mmc_rx_udp_gd),
	XGMAC_MMC_STAT(mmc_rx_udp_err),
	XGMAC_MMC_STAT(mmc_rx_tcp_gd),
	XGMAC_MMC_STAT(mmc_rx_tcp_err),
	XGMAC_MMC_STAT(mmc_rx_icmp_gd),
	XGMAC_MMC_STAT(mmc_rx_icmp_err),
	XGMAC_MMC_STAT(mmc_rx_udp_gd_octets),
	XGMAC_MMC_STAT(mmc_rx_udp_err_octets),
	XGMAC_MMC_STAT(mmc_rx_tcp_gd_octets),
	XGMAC_MMC_STAT(mmc_rx_tcp_err_octets),
	XGMAC_MMC_STAT(mmc_rx_icmp_gd_octets),
	XGMAC_MMC_STAT(mmc_rx_icmp_err_octets),
	XGMAC_MMC_STAT(mmc_tx_fpe_fragment_cntr),
	XGMAC_MMC_STAT(mmc_tx_hold_req_cntr),
	XGMAC_MMC_STAT(mmc_rx_packet_assembly_err_cntr),
	XGMAC_MMC_STAT(mmc_rx_packet_smd_err_cntr),
	XGMAC_MMC_STAT(mmc_rx_packet_assembly_ok_cntr),
	XGMAC_MMC_STAT(mmc_rx_fpe_fragment_cntr),
};
#define XGMAC_MMC_STATS_LEN ARRAY_SIZE(xgmac_mmc)

static void xgmac_ethtool_getdrvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, XGMAC_ETHTOOL_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static int xgmac_ethtool_get_link_ksettings(struct net_device *dev,
					     struct ethtool_link_ksettings *cmd)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (priv->hw->pcs & XGMAC_PCS_RGMII ||
	    priv->hw->pcs & XGMAC_PCS_SGMII) {
		struct rgmii_adv adv;
		u32 supported, advertising, lp_advertising;

		if (!priv->xstats.pcs_link) {
			cmd->base.speed = SPEED_UNKNOWN;
			cmd->base.duplex = DUPLEX_UNKNOWN;
			return 0;
		}
		cmd->base.duplex = priv->xstats.pcs_duplex;

		cmd->base.speed = priv->xstats.pcs_speed;

		/* Get and convert ADV/LP_ADV from the HW AN registers */
		if (xgmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv))
			return -EOPNOTSUPP;	/* should never happen indeed */

		/* Encoding of PSE bits is defined in 802.3z, 37.2.1.4 */

		ethtool_convert_link_mode_to_legacy_u32(
			&supported, cmd->link_modes.supported);
		ethtool_convert_link_mode_to_legacy_u32(
			&advertising, cmd->link_modes.advertising);
		ethtool_convert_link_mode_to_legacy_u32(
			&lp_advertising, cmd->link_modes.lp_advertising);

		if (adv.pause & XGMAC_PCS_PAUSE)
			advertising |= ADVERTISED_Pause;
		if (adv.pause & XGMAC_PCS_ASYM_PAUSE)
			advertising |= ADVERTISED_Asym_Pause;
		if (adv.lp_pause & XGMAC_PCS_PAUSE)
			lp_advertising |= ADVERTISED_Pause;
		if (adv.lp_pause & XGMAC_PCS_ASYM_PAUSE)
			lp_advertising |= ADVERTISED_Asym_Pause;

		/* Reg49[3] always set because ANE is always supported */
		cmd->base.autoneg = ADVERTISED_Autoneg;
		supported |= SUPPORTED_Autoneg;
		advertising |= ADVERTISED_Autoneg;
		lp_advertising |= ADVERTISED_Autoneg;

		if (adv.duplex) {
			supported |= (SUPPORTED_1000baseT_Full |
				      SUPPORTED_100baseT_Full |
				      SUPPORTED_10baseT_Full);
			advertising |= (ADVERTISED_1000baseT_Full |
					ADVERTISED_100baseT_Full |
					ADVERTISED_10baseT_Full);
		} else {
			supported |= (SUPPORTED_1000baseT_Half |
				      SUPPORTED_100baseT_Half |
				      SUPPORTED_10baseT_Half);
			advertising |= (ADVERTISED_1000baseT_Half |
					ADVERTISED_100baseT_Half |
					ADVERTISED_10baseT_Half);
		}
		if (adv.lp_duplex)
			lp_advertising |= (ADVERTISED_1000baseT_Full |
					   ADVERTISED_100baseT_Full |
					   ADVERTISED_10baseT_Full);
		else
			lp_advertising |= (ADVERTISED_1000baseT_Half |
					   ADVERTISED_100baseT_Half |
					   ADVERTISED_10baseT_Half);
		cmd->base.port = PORT_OTHER;

		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.supported, supported);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.advertising, advertising);
		ethtool_convert_legacy_u32_to_link_mode(
			cmd->link_modes.lp_advertising, lp_advertising);

		return 0;
	}

	return phylink_ethtool_ksettings_get(priv->phylink, cmd);
}

static int
xgmac_ethtool_set_link_ksettings(struct net_device *dev,
				  const struct ethtool_link_ksettings *cmd)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (priv->hw->pcs & XGMAC_PCS_RGMII ||
	    priv->hw->pcs & XGMAC_PCS_SGMII) {
		u32 mask = ADVERTISED_Autoneg | ADVERTISED_Pause;

		/* Only support ANE */
		if (cmd->base.autoneg != AUTONEG_ENABLE)
			return -EINVAL;

		mask &= (ADVERTISED_1000baseT_Half |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full);

		mutex_lock(&priv->lock);
		xgmac_pcs_ctrl_ane(priv, priv->ioaddr, 1, priv->hw->ps, 0);
		mutex_unlock(&priv->lock);

		return 0;
	}

	return phylink_ethtool_ksettings_set(priv->phylink, cmd);
}

static u32 xgmac_ethtool_getmsglevel(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void xgmac_ethtool_setmsglevel(struct net_device *dev, u32 level)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	priv->msg_enable = level;

}

static int xgmac_check_if_running(struct net_device *dev)
{
	if (!netif_running(dev))
		return -EBUSY;
	return 0;
}

static int xgmac_ethtool_get_regs_len(struct net_device *dev)
{
	return XGMAC_REGSIZE * 4;
}

static void xgmac_ethtool_gregs(struct net_device *dev,
			  struct ethtool_regs *regs, void *space)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 *reg_space = (u32 *) space;

	xgmac_dump_mac_regs(priv, priv->hw, reg_space);
	xgmac_dump_dma_regs(priv, priv->ioaddr, reg_space);
}

static int xgmac_nway_reset(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	return phylink_ethtool_nway_reset(priv->phylink);
}

static void
xgmac_get_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct xgmac_priv *priv = netdev_priv(netdev);
	struct rgmii_adv adv_lp;

	if (priv->hw->pcs && !xgmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return;
	} else {
		phylink_ethtool_get_pauseparam(priv->phylink, pause);
	}
}

static int
xgmac_set_pauseparam(struct net_device *netdev,
		      struct ethtool_pauseparam *pause)
{
	struct xgmac_priv *priv = netdev_priv(netdev);
	struct rgmii_adv adv_lp;

	if (priv->hw->pcs && !xgmac_pcs_get_adv_lp(priv, priv->ioaddr, &adv_lp)) {
		pause->autoneg = 1;
		if (!adv_lp.pause)
			return -EOPNOTSUPP;
		return 0;
	} else {
		return phylink_ethtool_set_pauseparam(priv->phylink, pause);
	}
}

static void xgmac_get_ethtool_stats(struct net_device *dev,
				 struct ethtool_stats *dummy, u64 *data)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	unsigned long count;
	int i, j = 0, ret;

	if (priv->dma_cap.asp) {
		for (i = 0; i < XGMAC_SAFETY_FEAT_SIZE; i++) {
			if (!xgmac_safety_feat_dump(priv, &priv->sstats, i,
						&count, NULL))
				data[j++] = count;
		}
	}

	/* Update the DMA HW counters for dwmac10/100 */
	ret = xgmac_dma_diagnostic_fr(priv, &dev->stats, (void *) &priv->xstats,
			priv->ioaddr);
	if (ret) {
		/* If supported, for new GMAC chips expose the MMC counters */
		if (priv->dma_cap.rmon) {
			xgmac_mmc_read(priv, priv->mmcaddr, &priv->mmc);

			for (i = 0; i < XGMAC_MMC_STATS_LEN; i++) {
				char *p;
				p = (char *)priv + xgmac_mmc[i].stat_offset;

				data[j++] = (xgmac_mmc[i].sizeof_stat ==
					     sizeof(u64)) ? (*(u64 *)p) :
					     (*(u32 *)p);
			}
		}
		if (priv->eee_enabled) {
			int val = phylink_get_eee_err(priv->phylink);
			if (val)
				priv->xstats.phy_eee_wakeup_error_n = val;
		}

		xgmac_mac_debug(priv, priv->ioaddr,
				(void *)&priv->xstats,
				rx_queues_count, tx_queues_count);
	}
	for (i = 0; i < XGMAC_STATS_LEN; i++) {
		char *p = (char *)priv + xgmac_gstrings_stats[i].stat_offset;
		data[j++] = (xgmac_gstrings_stats[i].sizeof_stat ==
			     sizeof(u64)) ? (*(u64 *)p) : (*(u32 *)p);
	}
}

static int xgmac_get_sset_count(struct net_device *netdev, int sset)
{
	struct xgmac_priv *priv = netdev_priv(netdev);
	int i, len, safety_len = 0;

	switch (sset) {
	case ETH_SS_STATS:
		len = XGMAC_STATS_LEN;

		if (priv->dma_cap.rmon)
			len += XGMAC_MMC_STATS_LEN;
		if (priv->dma_cap.asp) {
			for (i = 0; i < XGMAC_SAFETY_FEAT_SIZE; i++) {
				if (!xgmac_safety_feat_dump(priv,
							&priv->sstats, i,
							NULL, NULL))
					safety_len++;
			}

			len += safety_len;
		}

		return len;
	case ETH_SS_TEST:
		return xgmac_selftest_get_count(priv);
	default:
		return -EOPNOTSUPP;
	}
}

static void xgmac_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;
	struct xgmac_priv *priv = netdev_priv(dev);

	switch (stringset) {
	case ETH_SS_STATS:
		if (priv->dma_cap.asp) {
			for (i = 0; i < XGMAC_SAFETY_FEAT_SIZE; i++) {
				const char *desc;
				if (!xgmac_safety_feat_dump(priv,
							&priv->sstats, i,
							NULL, &desc)) {
					memcpy(p, desc, ETH_GSTRING_LEN);
					p += ETH_GSTRING_LEN;
				}
			}
		}
		if (priv->dma_cap.rmon)
			for (i = 0; i < XGMAC_MMC_STATS_LEN; i++) {
				memcpy(p, xgmac_mmc[i].stat_string,
				       ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
		for (i = 0; i < XGMAC_STATS_LEN; i++) {
			memcpy(p, xgmac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	case ETH_SS_TEST:
		xgmac_selftest_get_strings(priv, p);
		break;
	default:
		WARN_ON(1);
		break;
	}
}

/* Currently only support WOL through Magic packet. */
static void xgmac_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	mutex_lock(&priv->lock);
	if (device_can_wakeup(priv->device)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST;
		wol->wolopts = priv->wolopts;
	}
	mutex_unlock(&priv->lock);
}

static int xgmac_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 support = WAKE_MAGIC | WAKE_UCAST;

	/* By default almost all GMAC devices support the WoL via
	 * magic frame but we can disable it if the HW capability
	 * register shows no support for pmt_magic_frame. */
	if ((priv->hw_cap_support) && (!priv->dma_cap.pmt_magic_frame))
		wol->wolopts &= ~WAKE_MAGIC;

	if (!device_can_wakeup(priv->device))
		return -EINVAL;

	if (wol->wolopts & ~support)
		return -EINVAL;

	if (wol->wolopts) {
		pr_info("xgmac: wakeup enable\n");
		device_set_wakeup_enable(priv->device, 1);
		enable_irq_wake(priv->wol_irq);
	} else {
		device_set_wakeup_enable(priv->device, 0);
		disable_irq_wake(priv->wol_irq);
	}

	mutex_lock(&priv->lock);
	priv->wolopts = wol->wolopts;
	mutex_unlock(&priv->lock);

	return 0;
}

static int xgmac_ethtool_op_get_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (!priv->dma_cap.eee)
		return -EOPNOTSUPP;

	edata->eee_enabled = priv->eee_enabled;
	edata->eee_active = priv->eee_active;
	edata->tx_lpi_timer = priv->tx_lpi_timer;

	return phylink_ethtool_get_eee(priv->phylink, edata);
}

static int xgmac_ethtool_op_set_eee(struct net_device *dev,
				     struct ethtool_eee *edata)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int ret;

	if (!edata->eee_enabled) {
		xgmac_disable_eee_mode(priv);
	} else {
		/* We are asking for enabling the EEE but it is safe
		 * to verify all by invoking the eee_init function.
		 * In case of failure it will return an error.
		 */
		edata->eee_enabled = xgmac_eee_init(priv);
		if (!edata->eee_enabled)
			return -EOPNOTSUPP;
	}

	ret = phylink_ethtool_set_eee(priv->phylink, edata);
	if (ret)
		return ret;

	priv->eee_enabled = edata->eee_enabled;
	priv->tx_lpi_timer = edata->tx_lpi_timer;
	return 0;
}

static u32 xgmac_usec2riwt(u32 usec, struct xgmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->xgmac_clk);

	if (!clk) {
		clk = priv->plat->clk_ref_rate;
		if (!clk)
			return 0;
	}

	return (usec * (clk / 1000000)) / (256 << DEF_DMA_RWTU);
}

static u32 xgmac_riwt2usec(u32 riwt, struct xgmac_priv *priv)
{
	unsigned long clk = clk_get_rate(priv->plat->xgmac_clk);

	if (!clk) {
		clk = priv->plat->clk_ref_rate;
		if (!clk)
			return 0;
	}

	return (riwt * (256 << DEF_DMA_RWTU)) / (clk / 1000000);
}

static int xgmac_get_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	ec->tx_coalesce_usecs = priv->tx_coal_timer;
	ec->tx_max_coalesced_frames = priv->tx_coal_frames;

	if (priv->use_riwt) {
		ec->rx_max_coalesced_frames = priv->rx_coal_frames;
		ec->rx_coalesce_usecs = xgmac_riwt2usec(priv->rx_riwt, priv);
	}

	return 0;
}

static int xgmac_set_coalesce(struct net_device *dev,
			       struct ethtool_coalesce *ec)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_chans_to_use;
	unsigned int rx_riwt;

	rx_riwt = xgmac_usec2riwt(ec->rx_coalesce_usecs, priv);
	if ((rx_riwt < MIN_DMA_RIWT && rx_riwt != 0) || rx_riwt > MAX_DMA_RIWT)
		return -EINVAL;

	if (ec->rx_max_coalesced_frames > XGMAC_MAX_RX_FRAMES)
		return -EINVAL;

	if (rx_riwt == 0)
		priv->use_riwt = 0;
	else
		priv->use_riwt = 1;

	priv->rx_riwt = rx_riwt;
	priv->rx_coal_frames = ec->rx_max_coalesced_frames;
	xgmac_rx_watchdog(priv, priv->ioaddr, priv->rx_riwt,
			  priv->rx_coal_frames, rx_cnt);

	if ((ec->tx_coalesce_usecs == 0) &&
	    (ec->tx_max_coalesced_frames == 0))
		return -EINVAL;

	if ((ec->tx_coalesce_usecs > XGMAC_MAX_COAL_TX_TICK) ||
	    (ec->tx_max_coalesced_frames > XGMAC_TX_MAX_FRAMES))
		return -EINVAL;

	/* Only copy relevant parameters, ignore all others. */
	priv->tx_coal_frames = ec->tx_max_coalesced_frames;
	priv->tx_coal_timer = ec->tx_coalesce_usecs;
	return 0;
}

static int xgmac_get_rxnfc(struct net_device *dev,
			    struct ethtool_rxnfc *rxnfc, u32 *rule_locs)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	switch (rxnfc->cmd) {
	case ETHTOOL_GRXRINGS:
		rxnfc->data = priv->plat->rx_chans_to_use;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static u32 xgmac_get_rxfh_key_size(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	return sizeof(priv->rss.key);
}

static u32 xgmac_get_rxfh_indir_size(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	return priv->rss.table_size;
}

static int xgmac_get_rxfh(struct net_device *dev, u32 *indir, u8 *key,
			   u8 *hfunc)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int i;

	if (indir) {
		for (i = 0; i < priv->rss.table_size; i++)
			indir[i] = priv->rss.table[i];
	}

	if (key)
		memcpy(key, priv->rss.key, sizeof(priv->rss.key));
	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;

	return 0;
}

static int xgmac_set_rxfh(struct net_device *dev, const u32 *indir,
			   const u8 *key, const u8 hfunc)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int i;

	if ((hfunc != ETH_RSS_HASH_NO_CHANGE) && (hfunc != ETH_RSS_HASH_TOP))
		return -EOPNOTSUPP;

	if (indir) {
		for (i = 0; i < priv->rss.table_size; i++)
			priv->rss.table[i] = indir[i];
	}

	if (key)
		memcpy(priv->rss.key, key, sizeof(priv->rss.key));

	return xgmac_rss_configure(priv, priv->hw, &priv->rss,
				    priv->plat->rx_queues_to_use);
}

static int xgmac_get_ts_info(struct net_device *dev,
			      struct ethtool_ts_info *info)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (priv->dma_cap.time_stamp) {

		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE |
					SOF_TIMESTAMPING_TX_HARDWARE |
					SOF_TIMESTAMPING_RX_SOFTWARE |
					SOF_TIMESTAMPING_RX_HARDWARE |
					SOF_TIMESTAMPING_SOFTWARE |
					SOF_TIMESTAMPING_RAW_HARDWARE;

		if (priv->ptp_clock)
			info->phc_index = ptp_clock_index(priv->ptp_clock);

		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);

		info->rx_filters = ((1 << HWTSTAMP_FILTER_NONE) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_EVENT) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_SYNC) |
				    (1 << HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				    (1 << HWTSTAMP_FILTER_ALL));
		return 0;
	} else
		return ethtool_op_get_ts_info(dev, info);
}

static int xgmac_get_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna, void *data)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		*(u32 *)data = priv->rx_copybreak;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int xgmac_set_tunable(struct net_device *dev,
			      const struct ethtool_tunable *tuna,
			      const void *data)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int ret = 0;

	switch (tuna->id) {
	case ETHTOOL_RX_COPYBREAK:
		priv->rx_copybreak = *(u32 *)data;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct ethtool_ops xgmac_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS |
				     ETHTOOL_COALESCE_MAX_FRAMES,
	.begin = xgmac_check_if_running,
	.get_drvinfo = xgmac_ethtool_getdrvinfo,
	.get_msglevel = xgmac_ethtool_getmsglevel,
	.set_msglevel = xgmac_ethtool_setmsglevel,
	.get_regs = xgmac_ethtool_gregs,
	.get_regs_len = xgmac_ethtool_get_regs_len,
	.get_link = ethtool_op_get_link,
	.nway_reset = xgmac_nway_reset,
	.get_pauseparam = xgmac_get_pauseparam,
	.set_pauseparam = xgmac_set_pauseparam,
	.self_test = xgmac_selftest_run,
	.get_ethtool_stats = xgmac_get_ethtool_stats,
	.get_strings = xgmac_get_strings,
	.get_wol = xgmac_get_wol,
	.set_wol = xgmac_set_wol,
	.get_eee = xgmac_ethtool_op_get_eee,
	.set_eee = xgmac_ethtool_op_set_eee,
	.get_sset_count	= xgmac_get_sset_count,
	.get_rxnfc = xgmac_get_rxnfc,
	.get_rxfh_key_size = xgmac_get_rxfh_key_size,
	.get_rxfh_indir_size = xgmac_get_rxfh_indir_size,
	.get_rxfh = xgmac_get_rxfh,
	.set_rxfh = xgmac_set_rxfh,
	.get_ts_info = xgmac_get_ts_info,
	.get_coalesce = xgmac_get_coalesce,
	.set_coalesce = xgmac_set_coalesce,
	.get_tunable = xgmac_get_tunable,
	.set_tunable = xgmac_set_tunable,
	.get_link_ksettings = xgmac_ethtool_get_link_ksettings,
	.set_link_ksettings = xgmac_ethtool_set_link_ksettings,
};

void xgmac_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &xgmac_ethtool_ops;
}
