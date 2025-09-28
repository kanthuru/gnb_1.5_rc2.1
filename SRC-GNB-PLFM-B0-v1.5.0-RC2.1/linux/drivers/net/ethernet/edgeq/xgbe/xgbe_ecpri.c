#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <net/ip.h>
#include "xgbe.h"
#include "dwxgmac.h"
#include "hwif.h"
#include "xgbe_ecpri.h"

void ecpri_setup_config(struct xgmac_priv *priv)
{
	struct ecpri_ctrl_status *ecpri_cs;
	u32 total_buf_sz;
	int i;

	/* Default to eCPRI over Ethernet */
	priv->ecpri_over_eth = true;
	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	memset_io((void *)ecpri_cs, 0, sizeof(struct ecpri_ctrl_status));
	ecpri_cs->curr_state = ECPRI_DMA_STATE_STOP;
	smp_mb();
	ecpri_cs->rx_desc_offset = ECPRI_RX_DESC_OFFSET;
	smp_mb();
	ecpri_cs->eth_link = priv->plat->eth_link;
	smp_mb();
	ecpri_cs->rx_buf_size = priv->dma_buf_sz;
	smp_mb();
	ecpri_cs->tx_buf_size = priv->dma_buf_sz;
	smp_mb();
	if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2) {
		ecpri_cs->num_tx_q = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		smp_mb();  // Accessing LMEM
		if (priv->dma_buf_sz == BUF_SIZE_9KiB)
			total_buf_sz = ECPRI_XLGMAC_JUMBO_BUF_SIZE;
		else
			total_buf_sz = ECPRI_XLGMAC_NOR_BUF_SIZE;
		ecpri_cs->rx_desc_ring_size = total_buf_sz / priv->dma_buf_sz;
		smp_mb();
		if (priv->plat->eth_link == 1)
			ecpri_cs->tx_desc_ring_size =
				ecpri_cs->rx_desc_ring_size / ecpri_cs->num_tx_q;
		else
			ecpri_cs->tx_desc_ring_size =
				ecpri_cs->rx_desc_ring_size / 4;
		smp_mb();
		priv->h_buf_hdr = (struct ecpri_host_buf_hdr *)(priv->ecpri_base +
				  XLGMAC_HOST_BUF_OFFSET);
		smp_mb();
		for (i = 0; i < ecpri_cs->num_tx_q; i++) {
			ecpri_cs->tx_dma_ch[i] = i;
			smp_mb();
		}
	}
	else {
		ecpri_cs->num_tx_q = 1;
		smp_mb();  // Accessing LMEM
		if (priv->dma_buf_sz == BUF_SIZE_9KiB)
			total_buf_sz = ECPRI_XGMAC_JUMBO_BUF_SIZE;
		else
			total_buf_sz = ECPRI_XGMAC_NOR_BUF_SIZE;
		ecpri_cs->rx_desc_ring_size = total_buf_sz / priv->dma_buf_sz;
		smp_mb();
		ecpri_cs->tx_desc_ring_size = ecpri_cs->rx_desc_ring_size;
		smp_mb();
		priv->h_buf_hdr = (struct ecpri_host_buf_hdr *)(priv->ecpri_base +
				  XGMAC_HOST_BUF_OFFSET);
		smp_mb();
		ecpri_cs->tx_dma_ch[0] = 0;
		smp_mb();
	}
	memset_io(priv->h_buf_hdr, 0, sizeof(struct ecpri_host_buf_hdr));
	priv->h_buf_hdr->ring_size = ECPRI_HOST_BUF_RING_SIZE;
	smp_mb();
	xgmac_get_umac_addr(priv, priv->hw, ecpri_cs->src_mac.addr, 0);
	smp_mb();
}

int ecpri_alloc_rx_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

	rx_ch->chan_index = chan;
	rx_ch->priv_data = priv;
	rx_ch->page_pool = NULL;	// Not used
	rx_ch->buf_pool = NULL;		// Not used

	rx_ch->dma_rx = (struct dma_desc *)((u8 *)priv->ecpri_base + ECPRI_RX_DESC_OFFSET);
	rx_ch->dma_rx_phy = priv->ecpri_phy_addr + ECPRI_RX_DESC_OFFSET;

	/* Host buffer descriptor ring is in LMEM */
	priv->h_desc = (struct dma_desc *)((u8 *)priv->h_buf_hdr + sizeof(struct ecpri_host_buf_hdr));
	priv->h_pkt_buf = kcalloc(ECPRI_HOST_BUF_RING_SIZE, sizeof(*priv->h_pkt_buf), GFP_KERNEL);
	if (priv->h_pkt_buf == NULL) {
		netdev_err(priv->dev, "Error allocating buffer for eCPRI queue\n");
		return -ENOMEM;
	}

	return 0;
}

int ecpri_init_rx_rings(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_channel *ch = &priv->channel[chan];
	struct dma_desc *h_desc = priv->h_desc;
	struct ecpri_host_buf *h_pkt_buf;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	int i;

	for (i = 0; i < ECPRI_HOST_BUF_RING_SIZE; i++) {
		h_pkt_buf = &priv->h_pkt_buf[i];
		skb = napi_alloc_skb(&ch->rx_napi, priv->dma_buf_sz);
		if (skb == NULL)
			goto err_alloc;
		dma_addr = dma_map_single(priv->device, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, dma_addr))
			goto err_alloc;

		h_pkt_buf->skb = skb;
		h_pkt_buf->addr = dma_addr;
		xgmac_set_desc_addr(priv, &h_desc[i], dma_addr);
		smp_mb();
		xgmac_set_rx_owner(priv, &h_desc[i], priv->use_riwt);
		smp_mb();
	}

	priv->h_buf_hdr->head = 0;
	smp_mb();
	priv->h_buf_hdr->tail = 0;
	smp_mb();

	return 0;

err_alloc:
	/* Free allocated skb */
	while (i >= 0) {
		h_pkt_buf = &priv->h_pkt_buf[i];
		if (h_pkt_buf->skb != NULL)
			dev_kfree_skb(h_pkt_buf->skb);
		i--;
	}
	netdev_err(priv->dev, "Error allocating skb for eCPRI chan\n");

	return -ENOMEM;
}

void ecpri_free_rx_resources(struct xgmac_priv *priv, u32 chan)
{
	struct ecpri_host_buf *h_pkt_buf;
	int i;

	for (i = 0; i < ECPRI_HOST_BUF_RING_SIZE; i++) {
		h_pkt_buf = &priv->h_pkt_buf[i];
		if (h_pkt_buf->skb != NULL)
			dev_kfree_skb(h_pkt_buf->skb);
	}
	kfree(priv->h_pkt_buf);
}

int ecpri_alloc_tx_resources(struct xgmac_priv *priv, u32 chan)
{
	struct ecpri_ctrl_status *ecpri_cs;
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	u32 offset;
	u32 desc_size;

	tx_ch->chan_index = chan;
	tx_ch->priv_data = priv;
	tx_ch->tx_skbuff_dma = NULL;	// Not used
	tx_ch->tx_skbuff = NULL;	// Not used

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	desc_size = sizeof(struct dma_desc);
	if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
		offset = ECPRI_XLGMAC_TX_DESC_OFFSET +
			 desc_size * ecpri_cs->tx_desc_ring_size * chan;
	else
		offset = ECPRI_XGMAC_TX_DESC_OFFSET;
	tx_ch->dma_tx = (struct dma_desc *)((u8 *)priv->ecpri_base + offset);
	tx_ch->dma_tx_phy = priv->ecpri_phy_addr + offset;

	return 0;
}

void ecpri_rx_queue_dma_chan_map(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	struct ecpri_ctrl_status *ecpri_cs;
	u32 queue;
	u32 chan;

	/* For eCPRO over Ethernet, do static mapping. */
	if (priv->ecpri_over_eth) {
		for (queue = 0; queue < rx_queues_count; queue++) {
			chan = priv->plat->rx_queues_cfg[queue].chan;
			xgmac_map_mtl_to_dma(priv, priv->hw, queue, chan);
		}
	}
	/* For eCPRO over UDP, do dynamic mapping. */
	else {
		for (queue = 0; queue < rx_queues_count; queue++)
			xgmac_map_mtl_to_dma(priv, priv->hw, queue,
					      XGMAC_QDDMACH);
	}

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	ecpri_cs->rx_dma_ch = priv->plat->rx_queues_cfg[0].chan;
	smp_mb();
}

void ecpri_rxqueues_routing(struct xgmac_priv *priv)
{
	u32 q = priv->plat->rx_queues_to_use - 1;

	/* For eCPRI over Ethernet, route broadcast/multicast packets, PTP
	 * packets to last queue. The remaining packets fall to queue 0.
	 */
	if (priv->ecpri_over_eth) {
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_PTPQ, q, true);
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ, q, true);
	}
	else {
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_PTPQ, q, false);
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ, q, false);
	}
}

static inline void ecpri_rx_refill(struct xgmac_priv *priv, u32 chan,
				   u32 end_entry)
{
	struct xgmac_channel *ch = &priv->channel[chan];
	struct dma_desc *h_desc = priv->h_desc;
	struct ecpri_host_buf *h_pkt_buf;
	struct sk_buff *skb;
	dma_addr_t dma_addr;
	int i;

	if (end_entry == priv->h_buf_hdr->tail)
		return;

	i = priv->h_buf_hdr->tail;
	do {
		h_pkt_buf = &priv->h_pkt_buf[i];
		skb = napi_alloc_skb(&ch->rx_napi, priv->dma_buf_sz);
		if (skb == NULL) {
			netdev_err(priv->dev, "Error refill skb for eCPRI queue\n");
			break;
		}
		dma_addr = dma_map_single(priv->device, skb->data, skb_headlen(skb),
					  DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, dma_addr)) {
			netdev_err(priv->dev, "Error mapping skb for eCPRI queue\n");
			break;
		}

		h_pkt_buf->skb = skb;
		h_pkt_buf->addr = dma_addr;
		xgmac_set_desc_addr(priv, &h_desc[i], dma_addr);
		smp_mb();
		xgmac_set_rx_owner(priv, &h_desc[i], priv->use_riwt);
		smp_mb();

		i = XGMAC_GET_ENTRY(i, ECPRI_HOST_BUF_RING_SIZE);
	} while (i != end_entry);

	priv->h_buf_hdr->tail = i;
	smp_mb();
}

static void skb_drop_list(struct sk_buff **listp)
{
	kfree_skb_list(*listp);
	*listp = NULL;
}

static inline void skb_drop_fraglist(struct sk_buff *skb)
{
	skb_drop_list(&skb_shinfo(skb)->frag_list);
}

static unsigned int ecpri_rx_buf1_len(struct xgmac_priv *priv,
				      struct dma_desc *p, int status)
{
	int coe = priv->hw->rx_csum;
	unsigned int plen = 0;

	/* First descriptor, not last descriptor and not split header */
	if (status & rx_not_ls)
		return priv->dma_buf_sz;

	plen = xgmac_get_rx_frame_len(priv, p, coe);

	/* First descriptor and last descriptor and not split header */
	return min_t(unsigned int, priv->dma_buf_sz, plen);
}

static int ecpri_rx(struct xgmac_priv *priv, int limit, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct xgmac_channel *ch = &priv->channel[chan];
	unsigned int count = 0, error = 0, len = 0;
	int status = 0, coe = priv->hw->rx_csum;
	unsigned int next_entry = priv->h_buf_hdr->tail;
	struct sk_buff *skb = NULL;
	struct sk_buff *head_skb = NULL;
	struct ecpri_ctrl_status *ecpri_cs;
	u32 rx_packets;
	u32 rx_bytes;
	u32 rx_error;
	u32 rx_dropped;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	while (count < limit) {
		unsigned int buf1_len = 0;
		enum pkt_hash_types hash_type;
		struct ecpri_host_buf *h_pkt_buf;
		struct dma_desc *np, *p;
		int entry;
		u32 hash;

		if (!count && rx_ch->state_saved) {
			skb = rx_ch->state.skb;
			len = rx_ch->state.len;
			head_skb = skb;
			if (skb_has_frag_list(skb)) {
				skb = skb_shinfo(skb)->frag_list;
				while (skb->next)
					skb = skb->next;
			}
		} else {
			rx_ch->state_saved = false;
			skb = NULL;
			head_skb = NULL;
			len = 0;
		}

read_again:
		if (next_entry == priv->h_buf_hdr->head)
			break;

		buf1_len = 0;
		entry = next_entry;
		h_pkt_buf = &priv->h_pkt_buf[entry];

		p = priv->h_desc + entry;

		next_entry = XGMAC_GET_ENTRY(next_entry, ECPRI_HOST_BUF_RING_SIZE);
		np = priv->h_desc + next_entry;
		prefetch(np);

		/* read the status of the incoming frame */
		status = xgmac_rx_status(priv, &priv->dev->stats,
				&priv->xstats, p);
		/* check if managed by the DMA otherwise go ahead */
		if (unlikely((status & dma_own) || status == discard_frame)) {
			dev_kfree_skb(h_pkt_buf->skb);
			h_pkt_buf->skb = NULL;
			error = 1;
			if (!priv->hwts_rx_en)
				priv->dev->stats.rx_errors++;
		}

		if (unlikely(error && (status & rx_not_ls)))
			goto read_again;
		if (unlikely(error)) {
			if (head_skb) {
				if (skb_has_frag_list(head_skb))
					skb_drop_fraglist(head_skb);
				dev_kfree_skb(head_skb);
				head_skb = NULL;
				skb = NULL;
			}
			count++;
			continue;
		}

		/* Buffer is good. Go on. */

		buf1_len = ecpri_rx_buf1_len(priv, p, status);
		len += buf1_len;

		/* ACS is set; GMAC core strips PAD/FCS for IEEE 802.3
		* Type frames (LLC/LLC-SNAP)
		*
		* llc_snap is never checked in GMAC >= 4, so this ACS
		* feature is always disabled and packets need to be
		* stripped manually.
		*/
		if (likely(!(status & rx_not_ls)) &&
		    unlikely(status != llc_snap)) {
			buf1_len -= ETH_FCS_LEN;
			len -= ETH_FCS_LEN;
		}

		if (!skb) {
			dma_sync_single_for_cpu(priv->device, h_pkt_buf->addr,
						buf1_len, DMA_FROM_DEVICE);
			skb = h_pkt_buf->skb;
			skb_put(skb, buf1_len);
			head_skb = skb;
			h_pkt_buf->skb = NULL;
		} else if (buf1_len) {
			dma_sync_single_for_cpu(priv->device, h_pkt_buf->addr,
						buf1_len, DMA_FROM_DEVICE);

			if (skb == head_skb)
				skb_shinfo(skb)->frag_list = h_pkt_buf->skb;
			else
				skb->next = h_pkt_buf->skb;
			skb = h_pkt_buf->skb;
			skb_put(skb, buf1_len);
			head_skb->data_len += buf1_len;
			head_skb->len += buf1_len;
			head_skb->truesize += skb->truesize;

			/* Data payload appended into SKB */
			h_pkt_buf->skb = NULL;
		}

		if (likely(status & rx_not_ls))
			goto read_again;
		if (!head_skb)
			continue;

		/* Got entire packet into SKB. Finish it. */

		xgmac_get_rx_hwtstamp(priv, p, np, head_skb);
		xgmac_rx_vlan(priv->dev, head_skb);
		head_skb->protocol = eth_type_trans(head_skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(head_skb);
		else
			head_skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (!xgmac_get_rx_hash(priv, p, &hash, &hash_type))
			skb_set_hash(head_skb, hash, hash_type);

		skb_record_rx_queue(head_skb, chan);
		napi_gro_receive(&ch->rx_napi, head_skb);
		head_skb = NULL;
		skb = NULL;

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += len;
		count++;
	}

	if (status & rx_not_ls || head_skb) {
		rx_ch->state_saved = true;
		rx_ch->state.skb = head_skb;
		rx_ch->state.error = error;
		rx_ch->state.len = len;
	}

	if (next_entry != priv->h_buf_hdr->tail)
		ecpri_rx_refill(priv, chan, next_entry);

	priv->xstats.rx_pkt_n += count;

	/* Update eCPRI packet stats */
	rx_packets = ecpri_cs->rx_pkt_cnt;
	ecpri_cs->rx_pkt_cnt = 0;
	smp_mb();
	rx_bytes = ecpri_cs->rx_byte_cnt;
	ecpri_cs->rx_byte_cnt = 0;
	smp_mb();
	rx_dropped = ecpri_cs->rx_drop_cnt;
	ecpri_cs->rx_drop_cnt = 0;
	smp_mb();
	rx_error = ecpri_cs->rx_err_cnt;
	ecpri_cs->rx_err_cnt = 0;
	smp_mb();

	priv->dev->stats.rx_packets += rx_packets;
	priv->dev->stats.rx_bytes += rx_bytes;
	priv->dev->stats.rx_dropped += rx_dropped;
	priv->dev->stats.rx_errors += rx_error;
	priv->xstats.rx_pkt_n += rx_packets;

	if (!priv->ecpri_over_eth) {
		if (rx_packets > 0) {
			UDP_ADD_STATS(dev_net(priv->dev), UDP_MIB_INDATAGRAMS, rx_packets);
			IP_ADD_STATS(dev_net(priv->dev), IPSTATS_MIB_INPKTS, rx_packets);
		}
		if (rx_bytes > 0)
			IP_ADD_STATS(dev_net(priv->dev), IPSTATS_MIB_INOCTETS, rx_bytes);
		if (rx_error > 0)
			UDP_ADD_STATS(dev_net(priv->dev), UDP_MIB_INERRORS, rx_error);
	}

	if (priv->h_buf_hdr->buff_overflow) {
		netdev_err(priv->dev, "eCPRI host buffer overflow\n");
		priv->h_buf_hdr->buff_overflow = 0;
		smp_mb();
	}

	return count;
}

int ecpri_napi_poll_rx(struct napi_struct *napi, int budget)
{
	struct xgmac_channel *ch =
		container_of(napi, struct xgmac_channel, rx_napi);
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan = ch->index;
	int work_done;

	priv->xstats.napi_poll++;

	work_done = ecpri_rx(priv, budget, chan);
	if (work_done < budget && napi_complete_done(napi, work_done)) {
		unsigned long flags;

		spin_lock_irqsave(&ch->lock, flags);
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 1, 0);
		spin_unlock_irqrestore(&ch->lock, flags);
	}

	return work_done;
}

static int ecpri_tx_clean(struct xgmac_priv *priv, u32 chan)
{
	struct ecpri_ctrl_status *ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	u32 tx_packets;
	u32 tx_bytes;
	u32 tx_dropped;
	u32 tx_error;

	__netif_tx_lock_bh(netdev_get_tx_queue(priv->dev, chan));

	/* Update eCPRI packet stats */
	tx_packets = ecpri_cs->tx_pkt_cnt[chan];
	ecpri_cs->tx_pkt_cnt[chan] = 0;
	smp_mb();
	tx_bytes = ecpri_cs->tx_byte_cnt[chan];
	ecpri_cs->tx_byte_cnt[chan] = 0;
	smp_mb();
	tx_dropped = ecpri_cs->tx_drop_cnt[chan];
	ecpri_cs->tx_drop_cnt[chan] = 0;
	smp_mb();
	tx_error = ecpri_cs->tx_err_cnt[chan];
	ecpri_cs->tx_err_cnt[chan] = 0;
	smp_mb();

	priv->dev->stats.tx_packets += tx_packets;
	priv->dev->stats.tx_bytes += tx_bytes;
	priv->dev->stats.tx_dropped += tx_dropped;
	priv->dev->stats.tx_errors += tx_error;

	if (!priv->ecpri_over_eth) {
		if (tx_packets > 0) {
			UDP_ADD_STATS(dev_net(priv->dev), UDP_MIB_OUTDATAGRAMS, tx_packets);
			IP_ADD_STATS(dev_net(priv->dev), IPSTATS_MIB_OUTPKTS, tx_packets);
		}
		if (tx_bytes > 0)
			IP_ADD_STATS(dev_net(priv->dev), IPSTATS_MIB_OUTOCTETS, tx_bytes);
	}

	priv->xstats.tx_pkt_n += tx_packets;
	priv->xstats.tx_clean++;

	__netif_tx_unlock_bh(netdev_get_tx_queue(priv->dev, chan));

	return tx_packets;
}

int ecpri_napi_poll_tx(struct napi_struct *napi, int budget)
{
	struct xgmac_channel *ch =
		container_of(napi, struct xgmac_channel, tx_napi);
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan = ch->index;
	int work_done;

	priv->xstats.napi_poll++;

	work_done = ecpri_tx_clean(priv, chan);
	work_done = min(work_done, budget);

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		unsigned long flags;
		struct ecpri_ctrl_status *ecpri_cs;

		ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
		if (ecpri_cs->tx_ring_full) {
			if (ecpri_cs->curr_state != ECPRI_DMA_STATE_STOP) {
				ecpri_cs->curr_state = ECPRI_DMA_STATE_STOP;
				smp_mb();
				netdev_err(priv->dev, "eCPRI Tx Ring full\n");
			}
			else {
				/* Resume eCPRI process after a pause */
				ecpri_cs->tx_ring_full = 0;
				ecpri_cs->curr_state = ECPRI_DMA_STATE_RESUME;
				smp_mb();
			}
		}

		spin_lock_irqsave(&ch->lock, flags);
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 0, 1);
		spin_unlock_irqrestore(&ch->lock, flags);
	}

	return work_done;
}

void ecpri_start(struct xgmac_priv *priv)
{
	struct ecpri_ctrl_status *ecpri_cs;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	ecpri_cs->curr_state = ECPRI_DMA_STATE_START;
	smp_mb();
}

void ecpri_stop(struct xgmac_priv *priv)
{
	struct ecpri_ctrl_status *ecpri_cs;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	ecpri_cs->curr_state = ECPRI_DMA_STATE_STOP;
	smp_mb();
}

void ecpri_resume(struct xgmac_priv *priv)
{
	struct ecpri_ctrl_status *ecpri_cs;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	ecpri_cs->curr_state = ECPRI_DMA_STATE_RESUME;
	smp_mb();
}

static int ecpri_change_mode(struct xgmac_priv *priv, bool over_eth)
{
        struct ecpri_ctrl_status *ecpri_cs;
	u16 port;
	u16 dma_ch;
	int idx;
	int ret;

	if (over_eth == priv->ecpri_over_eth)
		return 0;

        ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	port = ecpri_cs->udp_port;
	dma_ch = priv->plat->rx_queues_cfg[0].chan;
	idx = priv->dma_cap.l3l4fnum - 1;
	if (over_eth) {
		ret = xgmac_config_l4_filter(priv, priv->hw, idx, false,
				true, false, false, port, false, 0);
	}
	else {
		ret = xgmac_config_l4_filter(priv, priv->hw, idx, true,
				true, false, false, port, true, dma_ch);
	}

	if (!ret) {
		priv->ecpri_over_eth = over_eth;
	}

	return ret;
}

int ecpri_set_udp_port(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
        struct ecpri_ctrl_status *ecpri_cs;
	u16 udp_port = 0;
	bool over_eth;
	int ret;

	if (copy_from_user(&udp_port, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

        ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (udp_port == ecpri_cs->udp_port)
		return 0;

	ecpri_cs->udp_port = udp_port;
	smp_mb();
	if (udp_port == 0)
		over_eth = true;
	else
		over_eth = false;

	ecpri_stop(priv);
	ret = ecpri_change_mode(priv, over_eth);
	ecpri_rxqueues_routing(priv);
	ecpri_rx_queue_dma_chan_map(priv);
	ecpri_resume(priv);

	return ret;
}

int ecpri_get_udp_port(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct ecpri_ctrl_status *ecpri_cs;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_to_user(ifr->ifr_data, &ecpri_cs->udp_port, sizeof(u16)))
		return -EFAULT;

	return 0;
}

int ecpri_set_vlan(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
        struct ecpri_ctrl_status *ecpri_cs;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

        ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_from_user(&ecpri_cs->vlan_tag[idx], ifr->ifr_data+2, sizeof(u16)))
		return -EFAULT;
	smp_mb();

	return 0;
}

int ecpri_get_vlan(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct ecpri_ctrl_status *ecpri_cs;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_to_user(ifr->ifr_data, &ecpri_cs->vlan_tag[idx], sizeof(u16)))
		return -EFAULT;

	return 0;
}

int ecpri_set_dst_ip(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
        struct ecpri_ctrl_status *ecpri_cs;
	struct ecpri_ip_addr dst_addr;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

        ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_from_user(&ecpri_cs->ip_addr[idx], ifr->ifr_data+2, sizeof(dst_addr)))
		return -EFAULT;
	smp_mb();
	if (copy_from_user(&ecpri_cs->is_ipv4[idx], ifr->ifr_data+18, 1))
		return -EFAULT;
	smp_mb();

	return 0;
}

int ecpri_get_dst_ip(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct ecpri_ctrl_status *ecpri_cs;
	struct ecpri_ip_addr dst_addr;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_to_user(ifr->ifr_data, &ecpri_cs->ip_addr[idx], sizeof(dst_addr)))
		return -EFAULT;
	if (copy_to_user(ifr->ifr_data+16, &ecpri_cs->is_ipv4[idx], 1))
		return -EFAULT;

	return 0;
}

int ecpri_set_dst_mac(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
        struct ecpri_ctrl_status *ecpri_cs;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_from_user(&ecpri_cs->dst_mac[idx], ifr->ifr_data+2, ETH_ALEN))
		return -EFAULT;
	smp_mb();

	return 0;
}

int ecpri_get_dst_mac(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct ecpri_ctrl_status *ecpri_cs;
	u16 idx;

	if (copy_from_user(&idx, ifr->ifr_data, sizeof(u16)))
		return -EFAULT;

	if (idx >= 4)
		return -EINVAL;

	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (copy_to_user(ifr->ifr_data, &ecpri_cs->dst_mac[idx], ETH_ALEN))
		return -EFAULT;

	return 0;
}
