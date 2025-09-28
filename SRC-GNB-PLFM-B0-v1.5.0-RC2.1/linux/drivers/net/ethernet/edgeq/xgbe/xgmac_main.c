// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This is the driver for the Raptor on-chip Ethernet controllers.
  Raptor Ethernet IPs are built around a Synopsys IP Core.

*******************************************************************************/

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <linux/pinctrl/consumer.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif /* CONFIG_DEBUG_FS */
#include <linux/net_tstamp.h>
#include <linux/phylink.h>
#include <linux/phy.h>
#include <linux/udp.h>
#include <net/pkt_cls.h>
#include <net/ip.h>
#include "xgmac_ptp.h"
#include "xgbe.h"
#include <linux/reset.h>
#include <linux/of_mdio.h>
#include "dwxgmac.h"
#include "hwif.h"
#include "xgbe_ecpri.h"
#include "xgbe_xpcs.h"
#include "xgbe_misc.h"
#include "xgbe_phy.h"
#include "xgmac_virt_chan.h"

#define	XGMAC_ALIGN(x)		ALIGN(ALIGN(x, SMP_CACHE_BYTES), 16)
#define	TSO_MAX_BUFF_SIZE	(SZ_16K - 1)

/* Module parameters */
#define TX_TIMEO	5000
static int watchdog = TX_TIMEO;
module_param(watchdog, int, 0644);
MODULE_PARM_DESC(watchdog, "Transmit timeout in milliseconds (default 5s)");

static int debug = -1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static int phyaddr = -1;
module_param(phyaddr, int, 0444);
MODULE_PARM_DESC(phyaddr, "Physical device address");

#define XGMAC_TX_THRESH	(DMA_TX_SIZE / 4)
#define XGMAC_RX_THRESH	(DMA_RX_SIZE / 4)

static int flow_ctrl = FLOW_AUTO;
module_param(flow_ctrl, int, 0644);
MODULE_PARM_DESC(flow_ctrl, "Flow control ability [on/off]");

static int pause = PAUSE_TIME;
module_param(pause, int, 0644);
MODULE_PARM_DESC(pause, "Flow Control Pause Time");

#define TC_DEFAULT 64
static int tc = TC_DEFAULT;
module_param(tc, int, 0644);
MODULE_PARM_DESC(tc, "DMA threshold control value");

#define	DEFAULT_BUFSIZE	BUF_SIZE_2KiB
static int buf_sz = DEFAULT_BUFSIZE;
module_param(buf_sz, int, 0644);
MODULE_PARM_DESC(buf_sz, "DMA buffer size");

#define	XGMAC_RX_COPYBREAK	256

#define USE_FLEXIBLE_PPS	1

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
				      NETIF_MSG_LINK | NETIF_MSG_IFUP |
				      NETIF_MSG_IFDOWN | NETIF_MSG_TIMER);

#define XGMAC_DEFAULT_LPI_TIMER	1000
static int eee_timer = XGMAC_DEFAULT_LPI_TIMER;
module_param(eee_timer, int, 0644);
MODULE_PARM_DESC(eee_timer, "LPI tx expiration time in msec");
#define XGMAC_LPI_T(x) (jiffies + msecs_to_jiffies(x))

/* By default the driver will use the ring mode to manage tx and rx descriptors,
 * but allow user to force to use the chain instead of the ring
 */
static unsigned int chain_mode;
module_param(chain_mode, int, 0444);
MODULE_PARM_DESC(chain_mode, "To use chain instead of ring mode");

static irqreturn_t xgmac_interrupt(int irq, void *data);
static irqreturn_t xgmac_dma_isr(int irq, void *data);
static irqreturn_t xgmac_uio_isr(int irq, void *data);

#ifdef CONFIG_DEBUG_FS
static const struct net_device_ops xgmac_netdev_ops;
static void xgmac_init_fs(struct net_device *dev);
static void xgmac_exit_fs(struct net_device *dev);
#endif

#define XGMAC_TIMER_TICK(x)	(jiffies + usecs_to_jiffies(x))
#define XGMAC_TIMER_MS_TICK(x)	(jiffies + msecs_to_jiffies(x))

#define MAX_TX_TS_FIFO_SIZE	8

#define VIRT_CH_TIMER_INTV	10 // In usec

static bool eth0_defer;
static bool suspend_aux_snap_read;

/* Latency profiling related definitions. The profiling assumes
 * large packets (IP fragmented) are sent and received every 500us.
 * The profiling reads STMR counter on first and last fragments,
 * and calculates the intervals.
 */
#define STMR_500US		30720
#define STMR_100US		(STMR_500US / 5)
#define STMR_1000US		(STMR_500US * 2)
#define STMR_1500US		(STMR_500US * 3)
#define STMR_2000US		(STMR_500US * 4)
#define MAX_PROFILE_PKT		30000
#define PROFILE_RX		0
#define PROFILE_TX		1
/* Rx counters */
static int pf_rx_first_seg_idx;
static int pf_rx_last_seg_idx;
static struct profile_data pf_rx_data[MAX_PROFILE_PKT];
/* Tx counters */
static int pf_tx_first_seg_idx;
static int pf_tx_last_seg_idx;
static struct profile_data pf_tx_data[MAX_PROFILE_PKT];
/* Rx napi counters */
static unsigned long napi_rx_prev_val;
static unsigned long max_napi_rx_intv;
static unsigned long min_napi_rx_intv;
static unsigned long num_napi_rx_call;
static unsigned long sum_napi_rx_intv;
static bool cal_napi_rx_intv;
/* End of latency profiling definition */

static void dma_free_tx_skbufs(struct xgmac_priv *priv, u32 chan);
static inline u32 xgmac_rx_dirty(struct xgmac_priv *priv, u32 chan);
static unsigned int xgmac_rx_buf1_len(struct xgmac_priv *priv,
				      struct dma_desc *p,
				      int status, unsigned int len);

static inline bool virt_chan_avail(struct xgmac_priv *priv)
{
	struct virt_info *vinfo = priv->virt_chan_info;

	if (priv->plat->virt_chan_en && vinfo->fastpath_state == VIRT_CHAN_READY)
		return true;
	else
		return false;
}

/**
 * xgmac_verify_args - verify the driver parameters.
 * Description: it checks the driver parameters and set a default in case of
 * errors.
 */
static void xgmac_verify_args(void)
{
	if (unlikely(watchdog < 0))
		watchdog = TX_TIMEO;
	if (unlikely((buf_sz < DEFAULT_BUFSIZE) || (buf_sz > BUF_SIZE_16KiB)))
		buf_sz = DEFAULT_BUFSIZE;
	if (unlikely(flow_ctrl > 1))
		flow_ctrl = FLOW_AUTO;
	else if (likely(flow_ctrl < 0))
		flow_ctrl = FLOW_OFF;
	if (unlikely((pause < 0) || (pause > 0xffff)))
		pause = PAUSE_TIME;
	if (eee_timer < 0)
		eee_timer = XGMAC_DEFAULT_LPI_TIMER;
}

/**
 * xgmac_disable_all_chans - Disable all channels
 * @priv: driver private structure
 */
static void xgmac_disable_all_chans(struct xgmac_priv *priv)
{
	u32 rx_chans_cnt = priv->plat->rx_chans_to_use;
	u32 tx_chans_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 max_ch = max(rx_chans_cnt, tx_chans_cnt);
	u32 chan;

	for (chan = 0; chan < max_ch; chan++) {
		struct xgmac_channel *ch = &priv->channel[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		if (chan < rx_chans_cnt)
			napi_disable(&ch->rx_napi);
		if (chan < tx_chans_cnt)
			napi_disable(&ch->tx_napi);
	}

	if (priv->plat->virt_chan_en)
		hrtimer_cancel(&priv->virt_ch_timer);
}

static void xgmac_virt_ch_timer_arm(struct xgmac_priv *priv)
{
	ktime_t intv = ns_to_ktime(VIRT_CH_TIMER_INTV * NSEC_PER_USEC);

	hrtimer_start(&priv->virt_ch_timer, intv, HRTIMER_MODE_REL);
}

static void virt_chan_reinit(struct xgmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_chans_to_use;
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[rx_count];
	u32 tx_count = priv->plat->tx_chans_to_use;
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[tx_count];

	rx_ch->dma_rx = priv->virt_chan_rx;
	rx_ch->cur_rx = 0;
	rx_ch->dirty_rx = 0;

	dma_free_tx_skbufs(priv, tx_count);
	tx_ch->dma_tx = priv->virt_chan_tx;
	tx_ch->dirty_tx = 0;
	tx_ch->cur_tx = 0;
	tx_ch->mss = 0;
}

static int virt_chan_update_state(struct xgmac_priv *priv)
{
	struct virt_info *vinfo = priv->virt_chan_info;
	int ret = 1;

	if (vinfo->linux_state == VIRT_CHAN_RESET) {
		vinfo->linux_state = VIRT_CHAN_INIT;
	} else if (vinfo->linux_state == VIRT_CHAN_INIT) {
		if (vinfo->fastpath_state == VIRT_CHAN_INIT) {
			vinfo->linux_state = VIRT_CHAN_ACK;
		}
	} else if (vinfo->linux_state == VIRT_CHAN_ACK) {
		if (vinfo->fastpath_state == VIRT_CHAN_READY) {
			vinfo->linux_state = VIRT_CHAN_READY;
			ret = 0;
		}
	} else if (vinfo->linux_state == VIRT_CHAN_READY) {
		if (vinfo->fastpath_state == VIRT_CHAN_INIT)
			vinfo->linux_state = VIRT_CHAN_INIT;
		else
			ret = 0;
	}

	return ret;
}

static void virt_chan_rx_copy(dma_addr_t dst_addr, dma_addr_t src_addr, u32 len)
{
	void *src = phys_to_virt(src_addr);
	void *dst = phys_to_virt(dst_addr);

	memcpy(dst, src, len);
}

static void reset_virt_chan_rx_owner(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct dma_desc *p;
	unsigned int entry;
	int dirty;

	entry = rx_ch->dirty_rx;
	dirty = xgmac_rx_dirty(priv, chan);

	while (dirty-- > 0) {
		p = rx_ch->dma_rx + entry;
		xgmac_set_rx_owner(priv, p, priv->use_riwt);

		entry = XGMAC_GET_ENTRY(entry, DMA_RX_SIZE);
	}
	rx_ch->dirty_rx = entry;
}

static int xgmac_virt_chan_rx(struct xgmac_priv *priv, int limit, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct xgmac_channel *ch = &priv->channel[chan];
	unsigned int count = 0, error = 0, len = 0;
	int status = 0, coe = priv->hw->rx_csum;
	unsigned int next_entry = rx_ch->cur_rx;
	struct sk_buff *skb = NULL;
	u32 dev_queue = 0;

	priv->virt_chan_rx_running = true;

	if (priv->plat->pmd_start_chan > 0)
		ch = &priv->channel[0];
	else
		ch = &priv->channel[priv->plat->rx_chans_to_use - 1];

	while (count < limit) {
		unsigned int buf1_len = 0;
		enum pkt_hash_types hash_type;
		struct dma_desc *p;
		phys_addr_t buf_phy_addr;
		void *buf_virt_addr;
		int entry;
		u32 hash;
		u32 i;

		if (!count && rx_ch->state_saved) {
			skb = rx_ch->state.skb;
			error = rx_ch->state.error;
			len = rx_ch->state.len;
		} else {
			rx_ch->state_saved = false;
			skb = NULL;
			error = 0;
			len = 0;
		}

		if (count >= limit)
			break;

read_again:
		buf1_len = 0;
		entry = next_entry;

		p = rx_ch->dma_rx + entry;

		/* read the status of the incoming frame */
		status = virt_chan_get_rx_status(p);
		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & dma_own))
			break;

		rx_ch->cur_rx = XGMAC_GET_ENTRY(rx_ch->cur_rx, DMA_RX_SIZE);
		next_entry = rx_ch->cur_rx;

		if (unlikely(status == discard_frame)) {
			error = 1;
			if (!priv->hwts_rx_en)
				priv->dev->stats.rx_errors++;
		}

		if (unlikely(error && (status & rx_not_ls)))
			goto read_again;
		if (unlikely(error)) {
			dev_kfree_skb(skb);
			skb = NULL;
			count++;
			continue;
		}

		buf1_len = virt_chan_get_rx_frame_len(p);
		len += buf1_len;
		if (unlikely(buf1_len > KB(64)))
			goto drain_data;

		buf_phy_addr = ((u64)le32_to_cpu(p->des1) << 32) |
			       le32_to_cpu(p->des0);
		buf_virt_addr = phys_to_virt(buf_phy_addr);
		prefetch(buf_virt_addr);

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
			skb = napi_alloc_skb(&ch->rx_napi, buf1_len);
			if (!skb) {
				priv->dev->stats.rx_dropped++;
				count++;
				goto drain_data;
			}

			dma_sync_single_for_cpu(priv->device, buf_phy_addr,
						buf1_len, DMA_FROM_DEVICE);
			skb_copy_to_linear_data(skb, buf_virt_addr,
						buf1_len);
			skb_put(skb, buf1_len);
		} else if (buf1_len) {
			struct page *buf_page;
			dma_addr_t buf_addr;

			for (i = 0; i < DIV_ROUND_UP(buf1_len, PAGE_SIZE); i++) {
				buf_page = page_pool_dev_alloc_pages(rx_ch->page_pool);
				buf_addr = page_pool_get_dma_addr(buf_page);
				dma_sync_single_for_cpu(priv->device, buf_addr,
							buf1_len, DMA_FROM_DEVICE);
				buf_phy_addr += i*PAGE_SIZE;
				virt_chan_rx_copy(buf_addr, buf_phy_addr,
						min_t(u32, PAGE_SIZE, buf1_len));
				skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags,
						buf_page, 0, buf1_len,
						priv->dma_buf_sz);

				/* Data payload appended into SKB */
				page_pool_release_page(rx_ch->page_pool, buf_page);
				buf1_len -= PAGE_SIZE;
			}
		}

drain_data:
		if (likely(status & rx_not_ls))
			goto read_again;
		if (!skb)
			continue;

		/* Got entire packet into SKB. Finish it. */

		xgmac_rx_vlan(priv->dev, skb);
		skb->protocol = eth_type_trans(skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (!xgmac_get_rx_hash(priv, p, &hash, &hash_type))
			skb_set_hash(skb, hash, hash_type);

		skb_record_rx_queue(skb, dev_queue);
		napi_gro_receive(&ch->rx_napi, skb);
		skb = NULL;

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += len;
		count++;
	}

	if (status & rx_not_ls || skb) {
		rx_ch->state_saved = true;
		rx_ch->state.skb = skb;
		rx_ch->state.error = error;
		rx_ch->state.len = len;
	}

	reset_virt_chan_rx_owner(priv, chan);

	priv->xstats.rx_pkt_n += count;

	priv->virt_chan_rx_running = false;

	return count;
}

static enum hrtimer_restart xgmac_virt_ch_timer(struct hrtimer *t)
{
	struct xgmac_priv *priv = container_of(t, struct xgmac_priv, virt_ch_timer);
	u32 chan;
	struct xgmac_channel *ch;
	unsigned long flags;
	int ret;

	ret = virt_chan_update_state(priv);
	if (ret) { // Not in ready state
		xgmac_virt_ch_timer_arm(priv);
		return HRTIMER_NORESTART;
	}

	if (priv->plat->pmd_start_chan == 0)
		chan = priv->plat->rx_chans_to_use - 1;
	else
		chan = 0;
	ch = &priv->channel[chan];
	if (napi_schedule_prep(&ch->rx_napi)) {
		spin_lock_irqsave(&ch->lock, flags);
		xgmac_disable_dma_irq(priv, priv->ioaddr, chan, 1, 0);
		spin_unlock_irqrestore(&ch->lock, flags);
		__napi_schedule(&ch->rx_napi);
	}
	if (napi_schedule_prep(&ch->tx_napi)) {
		spin_lock_irqsave(&ch->lock, flags);
		xgmac_disable_dma_irq(priv, priv->ioaddr, chan, 0, 1);
		spin_unlock_irqrestore(&ch->lock, flags);
		__napi_schedule(&ch->tx_napi);
	}
	return HRTIMER_NORESTART;
}

/**
 * xgmac_enable_all_chans - Enable all channels
 * @priv: driver private structure
 */
static void xgmac_enable_all_chans(struct xgmac_priv *priv)
{
	u32 rx_chans_cnt = priv->plat->rx_chans_to_use;
	u32 tx_chans_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 max_ch = max(rx_chans_cnt, tx_chans_cnt);
	u32 chan;

	for (chan = 0; chan < max_ch; chan++) {
		struct xgmac_channel *ch = &priv->channel[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		if (chan < rx_chans_cnt)
			napi_enable(&ch->rx_napi);
		if (chan < tx_chans_cnt)
			napi_enable(&ch->tx_napi);
	}

	if (priv->plat->virt_chan_en)
		xgmac_virt_ch_timer_arm(priv);
}

/**
 * xgmac_stop_all_queues - Stop all real queues
 * @priv: driver private structure
 */
static void xgmac_stop_all_queues(struct xgmac_priv *priv)
{
	u32 dev_tx_queues;
	u32 queue;

	if (priv->plat->pmd_enabled)
		dev_tx_queues = priv->plat->tx_chans_to_use -
					priv->plat->pmd_num_chans;
	else
		dev_tx_queues = priv->plat->tx_chans_to_use;
	for (queue = 0; queue < dev_tx_queues; queue++)
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
}

/**
 * xgmac_start_all_queues - Start all real queues
 * @priv: driver private structure
 */
static void xgmac_start_all_queues(struct xgmac_priv *priv)
{
	u32 dev_tx_queues;
	u32 queue;

	/* Start all queues, including ecpri queue. */
	if (priv->plat->pmd_enabled)
		dev_tx_queues = priv->plat->tx_chans_to_use -
					priv->plat->pmd_num_chans;
	else
		dev_tx_queues = priv->plat->tx_chans_to_use;
	for (queue = 0; queue < dev_tx_queues; queue++)
		netif_tx_start_queue(netdev_get_tx_queue(priv->dev, queue));
}

static void xgmac_service_event_schedule(struct xgmac_priv *priv)
{
	if (!test_bit(XGMAC_DOWN, &priv->state) &&
	    !test_and_set_bit(XGMAC_SERVICE_SCHED, &priv->state))
		queue_work(priv->wq, &priv->service_task);
}

static void xgmac_global_err(struct xgmac_priv *priv)
{
	netif_carrier_off(priv->dev);
	set_bit(XGMAC_RESET_REQUESTED, &priv->state);
	xgmac_service_event_schedule(priv);
}

/**
 * xgmac_clk_csr_set - dynamically set the MDC clock
 * @priv: driver private structure
 * Description: this is to dynamically set the MDC clock according to the csr
 * clock input.
 * Note:
 *	If a specific clk_csr value is passed from the platform
 *	this means that the CSR Clock Range selection cannot be
 *	changed at run-time and it is fixed (as reported in the driver
 *	documentation). Viceversa the driver will try to set the MDC
 *	clock dynamically according to the actual clock input.
 */
static void xgmac_clk_csr_set(struct xgmac_priv *priv)
{
	u32 clk_rate;

	clk_rate = clk_get_rate(priv->plat->xgmac_clk);

	priv->clk_csr = 0x0;
	if (priv->plat->has_xgmac) {
		if (clk_rate > CSR_F_400M)
			priv->clk_csr = 0x5;
		else if (clk_rate > CSR_F_350M)
			priv->clk_csr = 0x4;
		else if (clk_rate > CSR_F_300M)
			priv->clk_csr = 0x3;
		else if (clk_rate > CSR_F_250M)
			priv->clk_csr = 0x2;
		else if (clk_rate > CSR_F_150M)
			priv->clk_csr = 0x1;
		else
			priv->clk_csr = 0x0;
	}
}

static void print_pkt(unsigned char *buf, int len)
{
	pr_debug("len = %d byte, buf addr: 0x%p\n", len, buf);
	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buf, len);
}

static inline u32 xgmac_tx_avail(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	u32 avail;

	if (tx_ch->dirty_tx > tx_ch->cur_tx)
		avail = tx_ch->dirty_tx - tx_ch->cur_tx - 1;
	else
		avail = DMA_TX_SIZE - tx_ch->cur_tx + tx_ch->dirty_tx - 1;

	return avail;
}

/**
 * xgmac_rx_dirty - Get RX chan dirty
 * @priv: driver private structure
 * @chan: RX chan index
 */
static inline u32 xgmac_rx_dirty(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	u32 dirty;

	if (rx_ch->dirty_rx <= rx_ch->cur_rx)
		dirty = rx_ch->cur_rx - rx_ch->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_ch->dirty_rx + rx_ch->cur_rx;

	return dirty;
}

/**
 * xgmac_enable_eee_mode - check and enter in LPI mode
 * @priv: driver private structure
 * Description: this function is to verify and enter in LPI mode in case of
 * EEE.
 */
static void xgmac_enable_eee_mode(struct xgmac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_chans_to_use;
	u32 chan;

	/* If eCPRI enabled, eCPRI SS manages eCPRI Tx chan. Skip eee mode */
	if (priv->plat->ecpri_en || priv->plat->pmd_enabled)
		return;

	/* check if all TX chan have the work finished */
	for (chan = 0; chan < tx_cnt; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (tx_ch->dirty_tx != tx_ch->cur_tx)
			return; /* still unfinished work */
	}

	/* Check and enter in LPI mode */
	if (!priv->tx_path_in_lpi_mode)
		xgmac_set_eee_mode(priv, priv->hw,
				priv->plat->en_tx_lpi_clockgating);
}

/**
 * xgmac_disable_eee_mode - disable and exit from LPI mode
 * @priv: driver private structure
 * Description: this function is to exit and disable EEE in case of
 * LPI state is true. This is called by the xmit.
 */
void xgmac_disable_eee_mode(struct xgmac_priv *priv)
{
	xgmac_reset_eee_mode(priv, priv->hw);
	del_timer_sync(&priv->eee_ctrl_timer);
	priv->tx_path_in_lpi_mode = false;
}

/**
 * xgmac_eee_ctrl_timer - EEE TX SW timer.
 * @arg : data hook
 * Description:
 *  if there is no data transfer and if we are not in LPI state,
 *  then MAC Transmitter can be moved to LPI state.
 */
static void xgmac_eee_ctrl_timer(struct timer_list *t)
{
	struct xgmac_priv *priv = from_timer(priv, t, eee_ctrl_timer);

	xgmac_enable_eee_mode(priv);
	mod_timer(&priv->eee_ctrl_timer, XGMAC_LPI_T(eee_timer));
}

/**
 * xgmac_eee_init - init EEE
 * @priv: driver private structure
 * Description:
 *  if the GMAC supports the EEE (from the HW cap reg) and the phy device
 *  can also manage EEE, this function enable the LPI state and start related
 *  timer.
 */
bool xgmac_eee_init(struct xgmac_priv *priv)
{
	int tx_lpi_timer = priv->tx_lpi_timer;

	/* Check if MAC core supports the EEE feature. */
	if (!priv->dma_cap.eee)
		return false;

	mutex_lock(&priv->lock);

	/* Check if it needs to be deactivated */
	if (!priv->eee_active) {
		if (priv->eee_enabled) {
			netdev_dbg(priv->dev, "disable EEE\n");
			del_timer_sync(&priv->eee_ctrl_timer);
			xgmac_set_eee_timer(priv, priv->hw, 0, tx_lpi_timer);
		}
		mutex_unlock(&priv->lock);
		return false;
	}

	if (priv->eee_active && !priv->eee_enabled) {
		timer_setup(&priv->eee_ctrl_timer, xgmac_eee_ctrl_timer, 0);
		mod_timer(&priv->eee_ctrl_timer, XGMAC_LPI_T(eee_timer));
		xgmac_set_eee_timer(priv, priv->hw, XGMAC_DEFAULT_LIT_LS,
				     tx_lpi_timer);
	}

	mutex_unlock(&priv->lock);
	netdev_dbg(priv->dev, "Energy-Efficient Ethernet initialized\n");
	return true;
}

/* xgmac_get_tx_hwtstamp - get HW TX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read timestamp from the descriptor & pass it to stack.
 * and also perform some sanity checks.
 */
static void xgmac_get_tx_hwtstamp(struct xgmac_priv *priv,
				   struct dma_desc *p, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamp;
	bool found = false;
	u64 ns = 0;

	if (!priv->hwts_tx_en)
		return;

	/* exit if skb doesn't support hw tstamp */
	if (likely(!skb || !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		return;

	/* check tx tstamp status */
	if (xgmac_get_tx_timestamp_status(priv, p)) {
		xgmac_get_timestamp(priv, p, &ns);
		found = true;
	} else if (!xgmac_get_mac_tx_timestamp(priv, priv->hw, &ns)) {
		found = true;
	}

	if (found) {
		memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp.hwtstamp = ns_to_ktime(ns);

		netdev_dbg(priv->dev, "get valid TX hw timestamp %llu\n", ns);
		/* pass tstamp to stack */
		skb_tstamp_tx(skb, &shhwtstamp);
	}
}

static void xgmac_clear_tx_hwtstamp_fifo(struct xgmac_priv *priv)
{
	u64 ns;
	int count = 0;

	do {
		if (count++ > MAX_TX_TS_FIFO_SIZE)
			break;
	} while (!xgmac_get_mac_tx_timestamp(priv, priv->hw, &ns));
}

/* xgmac_get_rx_hwtstamp - get HW RX timestamps
 * @priv: driver private structure
 * @p : descriptor pointer
 * @np : next descriptor pointer
 * @skb : the socket buffer
 * Description :
 * This function will read received packet's timestamp from the descriptor
 * and pass it to stack. It also perform some sanity checks.
 */
void xgmac_get_rx_hwtstamp(struct xgmac_priv *priv, struct dma_desc *p,
			    struct dma_desc *np, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	struct dma_desc *desc = p;
	u64 ns = 0;

	if (!priv->hwts_rx_en)
		return;

	/* For XGMAC, the valid timestamp is from CTX next desc. */
	if (priv->plat->has_xgmac)
		desc = np;

	/* Check if timestamp is available */
	if (xgmac_get_rx_timestamp_status(priv, p, np)) {
		xgmac_get_timestamp(priv, desc, &ns);
		netdev_dbg(priv->dev, "get valid RX hw timestamp %llu\n", ns);
		shhwtstamp = skb_hwtstamps(skb);
		memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp->hwtstamp = ns_to_ktime(ns);
	} else  {
		netdev_dbg(priv->dev, "cannot get RX hw timestamp\n");
	}
}

static void xgmac_hwtstamp_config(struct xgmac_priv *priv,
				   struct xgmac_hw_ts_config *cfg)
{
	u64 temp = 0;
	u32 ss_inc = 0;
	u32 value = 0;
	u32 temp32 = 0;
	u32 ts_init = 0;

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		xgmac_config_hw_tstamping(priv, priv->ptpaddr, 0);
	else {
		/* When the prev TSENA is set 0, set the cur TSINIT to 1
		 * This ensures clock continues running
		 */
		temp32 = readl(priv->ptpaddr + PTP_TCR);
		if (temp32 & PTP_TCR_TSENA)
			ts_init = 0;
		else
			ts_init = PTP_TCR_TSINIT;

		value = (PTP_TCR_TSENA | PTP_TCR_TSCFUPDT | PTP_TCR_TSCTRLSSR |
			 PTP_TCR_TSENMACADDR | PTP_TCR_TXTSSTSM |
			 PTP_TCR_TSVER2ENA | cfg->ptp_over_ethernet |
			 cfg->ptp_over_ipv6_udp | cfg->ptp_over_ipv4_udp |
			 cfg->ts_event_en | cfg->ts_master_en |
			 cfg->snap_type_sel | cfg->tstamp_all |
			 ts_init);
		xgmac_config_hw_tstamping(priv, priv->ptpaddr, value);

		/* program Sub Second Increment reg */
		xgmac_config_sub_second_increment(priv, priv->ptpaddr,
				priv->plat->clk_ptp_rate, &ss_inc);

		/* Store sub second increment and flags for later use */
		priv->sub_second_inc = ss_inc;
		priv->systime_flags = value;

		/* calculate default added value:
		 * formula is :
		 * addend = (2^32)/freq_div_ratio;
		 * where, freq_div_ratio = 1e9ns/ss_inc
		 */
		temp = div_u64(1000000000ULL, ss_inc);
		temp = (u64)(temp << 32);
		priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
		xgmac_config_addend(priv, priv->ptpaddr, priv->default_addend);
	}
}

#ifdef USE_FLEXIBLE_PPS
#define PPS_TIMER_INTV		12000      // 12 ms: 2 ms more than pulse width
#define PPS_TIMEOUT_SEC		3100000    // 3.1 seconds timeout
static void xgmac_pps_timer(struct timer_list *t)
{
	struct xgmac_priv *priv = from_timer(priv, t, pps_timer);

	xgmac_gen_flex_pps(priv, priv->ioaddr, 1, false);
}

static void xgmac_pps_tmo_timer(struct timer_list *t)
{
	struct xgmac_priv *priv = from_timer(priv, t, pps_tmo_timer);

	/* Timeout occurred, just start next pps */
	xgmac_gen_flex_pps(priv, priv->ioaddr, 1, false);
}

static irqreturn_t xgmac_pps_interrupt(int irq, void *data)
{
	struct xgmac_priv *priv = (struct xgmac_priv *)data;

	mod_timer(&priv->pps_timer, XGMAC_TIMER_TICK(PPS_TIMER_INTV));
	mod_timer(&priv->pps_tmo_timer, XGMAC_TIMER_TICK(PPS_TIMEOUT_SEC));

	return IRQ_HANDLED;
}

static int init_flex_pps(struct xgmac_priv *priv)
{
	unsigned long irq_flags;
	int ret;

	timer_setup(&priv->pps_timer, xgmac_pps_timer, 0);
	timer_setup(&priv->pps_tmo_timer, xgmac_pps_tmo_timer, 0);

	/* Start first pps */
	xgmac_gen_flex_pps(priv, priv->ioaddr, 1, false);

#ifndef CONFIG_PREEMPT_RT
	irq_flags = IRQF_SHARED | IRQF_TRIGGER_RISING;
#else
	irq_flags = IRQF_SHARED | IRQF_NO_THREAD | IRQF_TRIGGER_RISING;
#endif

	/* Request the PPS IRQ */
	snprintf(priv->pps_irq_name, sizeof(priv->pps_irq_name) - 1, "%s-pps",
		 priv->dev->name);
	ret = request_irq(priv->pps_irq, xgmac_pps_interrupt,
			  irq_flags, priv->pps_irq_name, priv);
	if (unlikely(ret < 0)) {
		netdev_err(priv->dev,
			   "%s: ERROR: PPS IRQ %d (error: %d)\n",
			   __func__, priv->pps_irq, ret);
		return -EINVAL;
	}

	return 0;
}

static void stop_flex_pps(struct xgmac_priv *priv)
{
	xgmac_gen_flex_pps(priv, priv->ioaddr, 1, true);
	del_timer_sync(&priv->pps_timer);
	del_timer_sync(&priv->pps_tmo_timer);
	free_irq(priv->pps_irq, priv);
}
#endif

/**
 *  xgmac_hwtstamp_set - control hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specific structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function configures the MAC to enable/disable both outgoing(TX)
 *  and incoming(RX) packets time stamping based on user input.
 *  Return Value:
 *  0 on success and an appropriate -ve integer on failure.
 */
static int xgmac_hwtstamp_set(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	struct xgmac_hw_ts_config ts_cfg;

	if (!priv->dma_cap.time_stamp) {
		netdev_alert(priv->dev, "No support for HW time stamping\n");
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;

		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(config)))
		return -EFAULT;

	netdev_dbg(priv->dev, "%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
		   __func__, config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	if (config.tx_type != HWTSTAMP_TX_OFF &&
	    config.tx_type != HWTSTAMP_TX_ON) {
		if (config.tx_type != HWTSTAMP_TX_ONESTEP_SYNC &&
		    config.tx_type != HWTSTAMP_TX_ONESTEP_P2P)
			return -ERANGE;
		else if (!priv->dma_cap.osten)
			return -EOPNOTSUPP;
	}

	ts_cfg.ptp_over_ipv4_udp = 0;
	ts_cfg.ptp_over_ipv6_udp = 0;
	ts_cfg.ptp_over_ethernet = 0;
	ts_cfg.snap_type_sel = 0;
	ts_cfg.ts_master_en = 0;
	ts_cfg.ts_event_en = 0;
	ts_cfg.tstamp_all = 0;
	/* TODO - this is temporary workaround for PTP unicast issue */
	/* Change has been removed as this is now handled in the userspace for AccuTime*/
	/* However, due to HW bug still being present this will break PTP4L unicast*/
	//ts_cfg.tstamp_all = PTP_TCR_TSENALL;

	switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			/* time stamp no incoming packet at all */
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
			/* PTP v2, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
			/* take time stamp for all event messages */
			ts_cfg.snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
			/* PTP v2, UDP, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
			/* take time stamp for SYNC messages only */
			ts_cfg.ts_event_en = PTP_TCR_TSEVNTENA;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			/* PTP v2, UDP, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
			/* take time stamp for Delay_Req messages only */
			ts_cfg.ts_master_en = PTP_TCR_TSMSTRENA;
			ts_cfg.ts_event_en = PTP_TCR_TSEVNTENA;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			/* PTP v2/802.AS1 any layer, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			ts_cfg.snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ts_cfg.ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			/* PTP v2/802.AS1, any layer, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			/* take time stamp for SYNC messages only */
			ts_cfg.ts_event_en = PTP_TCR_TSEVNTENA;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ts_cfg.ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			/* PTP v2/802.AS1, any layer, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
			/* take time stamp for Delay_Req messages only */
			ts_cfg.ts_master_en = PTP_TCR_TSMSTRENA;
			ts_cfg.ts_event_en = PTP_TCR_TSEVNTENA;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ts_cfg.ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_NTP_ALL:
		case HWTSTAMP_FILTER_ALL:
			/* time stamp any incoming packet */
			/* time stamp for all packet is not allowed */
			// config.rx_filter = HWTSTAMP_FILTER_ALL;
			// ts_cfg.tstamp_all = PTP_TCR_TSENALL;

			/* Use same config as HWTSTAMP_FILTER_PTP_V2_EVENT */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			ts_cfg.snap_type_sel = PTP_TCR_SNAPTYPSEL_1;

			ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ts_cfg.ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		default:
			return -ERANGE;
	}

	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
	priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

	xgmac_hwtstamp_config(priv, &ts_cfg);
#ifdef USE_FLEXIBLE_PPS
	/* HW clock re-configured, need to re-trigger pps */
	xgmac_gen_flex_pps(priv, priv->ioaddr, 1, false);
#endif

	memcpy(&priv->tstamp_config, &config, sizeof(config));

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(config)) ? -EFAULT : 0;
}

/**
 *  xgmac_hwtstamp_get - read hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specific structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function obtain the current hardware timestamping settings
    as requested.
 */
static int xgmac_hwtstamp_get(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct hwtstamp_config *config = &priv->tstamp_config;

	if (!priv->dma_cap.time_stamp)
		return -EOPNOTSUPP;

	return copy_to_user(ifr->ifr_data, config,
			    sizeof(*config)) ? -EFAULT : 0;
}

/**
 * xgmac_init_ptp - init PTP
 * @priv: driver private structure
 * Description: this is to verify if the HW supports the PTPv1 or PTPv2.
 * This is done by looking at the HW cap. register.
 * This function also registers the ptp driver.
 */
static int xgmac_init_ptp(struct xgmac_priv *priv)
{
	struct xgmac_hw_ts_config ts_cfg;

	if (!priv->dma_cap.time_stamp)
		return -EOPNOTSUPP;
//	else
//		netdev_info(priv->dev,
//			    "IEEE 1588-2008 Timestamp supported\n");

	/* Enable HW timestamping by default */
	priv->hwts_tx_en = 1;
	priv->hwts_rx_en = 1;

	/* Set default HW timestamping configuration */
	memset(&ts_cfg, 0, sizeof(struct xgmac_hw_ts_config));
	ts_cfg.ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
	ts_cfg.ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
	ts_cfg.ptp_over_ethernet = PTP_TCR_TSIPENA;
	ts_cfg.snap_type_sel = PTP_TCR_SNAPTYPSEL_1;
	//ts_cfg.ts_master_en = PTP_TCR_TSMSTRENA;	// default as slave
	//ts_cfg.ts_event_en = PTP_TCR_TSEVNTENA;	// ts for delay req also
	ts_cfg.tstamp_all = 0;
	//ts_cfg.tstamp_all = PTP_TCR_TSENALL;		// For testing only

	xgmac_hwtstamp_config(priv, &ts_cfg);
	xgmac_ptp_register(priv);

	return 0;
}

static void xgmac_release_ptp(struct xgmac_priv *priv)
{
	if (priv->plat->clk_ptp_ref)
		clk_disable_unprepare(priv->plat->clk_ptp_ref);
	xgmac_ptp_unregister(priv);
}

/**
 *  xgmac_mac_flow_ctrl - Configure flow control in all queues
 *  @priv: driver private structure
 *  Description: It is used for configuring the flow control in all queues
 */
static void xgmac_mac_flow_ctrl(struct xgmac_priv *priv, u32 duplex)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;

	xgmac_flow_ctrl(priv, priv->hw, duplex, priv->flow_ctrl,
			priv->pause, tx_cnt);
}

static void xgmac_validate(struct phylink_config *config,
			    unsigned long *supported,
			    struct phylink_link_state *state)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mac_supported) = { 0, };
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };
	int tx_cnt = priv->plat->tx_queues_to_use;
	int max_speed = priv->plat->max_speed;

	phylink_set(mac_supported, 10baseT_Half);
	phylink_set(mac_supported, 10baseT_Full);
	phylink_set(mac_supported, 100baseT_Half);
	phylink_set(mac_supported, 100baseT_Full);
	phylink_set(mac_supported, 1000baseT_Half);
	phylink_set(mac_supported, 1000baseT_Full);
	phylink_set(mac_supported, 1000baseKX_Full);

	phylink_set(mac_supported, Autoneg);
	phylink_set(mac_supported, Pause);
	phylink_set(mac_supported, Asym_Pause);
	phylink_set(mac_supported, FEC_RS);
	phylink_set_port_modes(mac_supported);

	/* Cut down 1G if asked to */
	if ((max_speed > 0) && (max_speed < 1000)) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseX_Full);
	} else if (priv->plat->has_xgmac) {
		if (!max_speed || (max_speed >= 2500)) {
			phylink_set(mac_supported, 2500baseT_Full);
			phylink_set(mac_supported, 2500baseX_Full);
		}
		if (!max_speed || (max_speed >= 5000)) {
			phylink_set(mac_supported, 5000baseT_Full);
		}
		if (!max_speed || (max_speed >= 10000)) {
			phylink_set(mac_supported, 10000baseSR_Full);
			phylink_set(mac_supported, 10000baseLR_Full);
			phylink_set(mac_supported, 10000baseER_Full);
			phylink_set(mac_supported, 10000baseLRM_Full);
			phylink_set(mac_supported, 10000baseT_Full);
			phylink_set(mac_supported, 10000baseKX4_Full);
			phylink_set(mac_supported, 10000baseKR_Full);
		}
		if (!max_speed || (max_speed >= 25000)) {
			phylink_set(mac_supported, 25000baseCR_Full);
			phylink_set(mac_supported, 25000baseKR_Full);
			phylink_set(mac_supported, 25000baseSR_Full);
		}
		if (!max_speed || (max_speed >= 40000)) {
			phylink_set(mac_supported, 40000baseKR4_Full);
			phylink_set(mac_supported, 40000baseCR4_Full);
			phylink_set(mac_supported, 40000baseSR4_Full);
			phylink_set(mac_supported, 40000baseLR4_Full);
		}
		if (!max_speed || (max_speed >= 50000)) {
			phylink_set(mac_supported, 50000baseCR2_Full);
			phylink_set(mac_supported, 50000baseKR2_Full);
			phylink_set(mac_supported, 50000baseSR2_Full);
			phylink_set(mac_supported, 50000baseKR_Full);
			phylink_set(mac_supported, 50000baseSR_Full);
			phylink_set(mac_supported, 50000baseCR_Full);
			phylink_set(mac_supported, 50000baseLR_ER_FR_Full);
			phylink_set(mac_supported, 50000baseDR_Full);
		}
		if (!max_speed || (max_speed >= 100000)) {
			phylink_set(mac_supported, 100000baseKR4_Full);
			phylink_set(mac_supported, 100000baseSR4_Full);
			phylink_set(mac_supported, 100000baseCR4_Full);
			phylink_set(mac_supported, 100000baseLR4_ER4_Full);
			phylink_set(mac_supported, 100000baseKR2_Full);
			phylink_set(mac_supported, 100000baseSR2_Full);
			phylink_set(mac_supported, 100000baseCR2_Full);
			phylink_set(mac_supported, 100000baseLR2_ER2_FR2_Full);
			phylink_set(mac_supported, 100000baseDR2_Full);
		}
	}

	/* Half-Duplex can only work with single queue */
	if (tx_cnt > 1) {
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 1000baseT_Half);
	}

	linkmode_and(supported, supported, mac_supported);
	linkmode_andnot(supported, supported, mask);

	linkmode_and(state->advertising, state->advertising, mac_supported);
	linkmode_andnot(state->advertising, state->advertising, mask);
}

static void mac_get_link_state(struct xgmac_priv *priv,
			       struct phylink_link_state *state)
{
	void __iomem *ioaddr = priv->hw->pcsr;
	u32 status;

	status = readl(ioaddr + XGMAC_INT_STATUS);
	if (status & XGMAC_LINK_FAULT) {
		state->link = 0;
		state->speed = SPEED_UNKNOWN;
		state->duplex = DUPLEX_HALF;
	}
	else {
		state->link = 1;
		state->speed = priv->plat->max_speed;
		state->duplex = DUPLEX_FULL;
	}
}

static void xgmac_mac_pcs_get_state(struct phylink_config *config,
				     struct phylink_link_state *state)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	struct phy_device *phydev = priv->phydev;
	int status;

	state->link = 0;
	if (phydev && phydev->drv) {
		status = phy_read_status(phydev);
		if (status == 0) {
			state->link = phydev->link;
			state->speed = phydev->speed;
			state->duplex = phydev->duplex;
		} else {
			state->link = 0;
			state->speed = SPEED_UNKNOWN;
			state->duplex = DUPLEX_HALF;
		}
	} else {
		mac_get_link_state(priv, state);
	}
}

static void xgmac_mac_config(struct phylink_config *config, unsigned int mode,
			      const struct phylink_link_state *state)
{
	/* Do nothing */
}

static void xgmac_mac_an_restart(struct phylink_config *config)
{
	/* Not Supported */
}

static void xgmac_mac_link_down(struct phylink_config *config,
				 unsigned int mode, phy_interface_t interface)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	xgmac_mac_set(priv, priv->ioaddr, false);
	priv->eee_active = false;
	xgmac_eee_init(priv);
	xgmac_set_eee_pls(priv, priv->hw, false);
	xgmac_clear_tx_hwtstamp_fifo(priv);
}

static void xgmac_mac_link_up(struct phylink_config *config,
			       struct phy_device *phy,
			       unsigned int mode, phy_interface_t interface,
			       int speed, int duplex,
			       bool tx_pause, bool rx_pause)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	u32 tx_cfg;
	int linkup;

	tx_cfg = readl(priv->ioaddr + XGMAC_TX_CONFIG);
	tx_cfg &= ~priv->hw->link.speed_mask;

	/* When PHY does rate matching, XGMAC stays at max speed */
	if (priv->plat->phy_rate_matching || priv->plat->eth_link == 1 ||
	    priv->plat->phy_lane != 0) {
		speed = priv->plat->max_speed;
	} else {
		if (interface != PHY_INTERFACE_MODE_USXGMII)
			xgbe_misc_update_speed(priv, speed);
		xgmac_xpcs_update_speed(priv, priv->xpcs_base, speed,
					interface);
	}
	/* When PHY link is up, needs to update XPCS link status */
	xgmac_xpcs_get_state(priv, priv->device, priv->xpcs_base, &linkup);

	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		switch (speed) {
		case SPEED_10000:
			tx_cfg |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_5000:
			tx_cfg |= priv->hw->link.xgmii.speed5000;
			break;
		case SPEED_2500:
			tx_cfg |= priv->hw->link.xgmii.speed2500;
			break;
		case SPEED_1000:
			tx_cfg |= priv->hw->link.speed1000;
			break;
		default:
			return;
		}
	} else if (interface == PHY_INTERFACE_MODE_XLGMII) {
		switch (speed) {
		case SPEED_100000:
			tx_cfg |= priv->hw->link.xlgmii.speed100000;
			break;
		case SPEED_50000:
			tx_cfg |= priv->hw->link.xlgmii.speed50000;
			break;
		case SPEED_40000:
			tx_cfg |= priv->hw->link.xlgmii.speed40000;
			break;
		case SPEED_25000:
			tx_cfg |= priv->hw->link.xlgmii.speed25000;
			break;
		case SPEED_10000:
			tx_cfg |= priv->hw->link.xgmii.speed10000;
			break;
		default:
			return;
		}
	} else if (interface == PHY_INTERFACE_MODE_XGMII) {
		switch (speed) {
		case SPEED_10000:
			tx_cfg |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_5000:
			tx_cfg |= priv->hw->link.xgmii.speed5000;
			break;
		case SPEED_2500:
			tx_cfg |= priv->hw->link.xgmii.speed2500;
			break;
		default:
			return;
		}
	} else {
		switch (speed) {
		case SPEED_2500:
			tx_cfg |= priv->hw->link.speed2500;
			break;
		case SPEED_1000:
			tx_cfg |= priv->hw->link.speed1000;
			break;
		case SPEED_100:
			tx_cfg |= priv->hw->link.speed100;
			break;
		case SPEED_10:
			tx_cfg |= priv->hw->link.speed10;
			break;
		default:
			return;
		}
	}
	writel(tx_cfg, priv->ioaddr + XGMAC_TX_CONFIG);

	priv->speed = speed;

	/* Flow Control operation */
	if (tx_pause && rx_pause)
		xgmac_mac_flow_ctrl(priv, duplex);


	xgmac_mac_set(priv, priv->ioaddr, true);
	if (phy && priv->dma_cap.eee) {
		priv->eee_active = phy_init_eee(phy, 1) >= 0;
		priv->eee_enabled = xgmac_eee_init(priv);
		xgmac_set_eee_pls(priv, priv->hw, true);
	}

	/* When eth link is disconnected while ptp daemon is running,
	 * Tx timestamp could be left in Tx timestamp fifo. That could
	 * cause ptp timestamp out of sync. Clear Tx timestamp fifo after
	 * eth link is up.
	 */
	readb(priv->ioaddr + XGMAC_TXTIMESTAMP_SEC+3);
}

static const struct phylink_mac_ops xgmac_phylink_mac_ops = {
	.validate = xgmac_validate,
	.mac_pcs_get_state = xgmac_mac_pcs_get_state,
	.mac_config = xgmac_mac_config,
	.mac_an_restart = xgmac_mac_an_restart,
	.mac_link_down = xgmac_mac_link_down,
	.mac_link_up = xgmac_mac_link_up,
};

static int xgmac_phy_setup(struct xgmac_priv *priv)
{
	struct fwnode_handle *fwnode = of_fwnode_handle(priv->plat->phylink_node);
	int mode = priv->plat->phy_interface;
	struct phylink *phylink;

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	priv->phylink_config.pcs_poll = true;

	if (!fwnode)
		fwnode = dev_fwnode(priv->device);

	phylink = phylink_create(&priv->phylink_config, fwnode,
				 mode, &xgmac_phylink_mac_ops);
	if (IS_ERR(phylink))
		return PTR_ERR(phylink);

	priv->phylink = phylink;

	return 0;
}

static void xgmac_display_rx_rings(struct xgmac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	void *head_rx;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* Not display eCPRI ring. eCPRI ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en)
		num_ecpri_ch = 1;

	/* Display RX rings */
	for (chan = num_ecpri_ch; chan < rx_cnt; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		pr_info("\tRX Queue %u rings\n", chan);

		if (priv->extend_desc)
			head_rx = (void *)rx_ch->dma_erx;
		else
			head_rx = (void *)rx_ch->dma_rx;

		/* Display RX ring */
		xgmac_display_ring(priv, head_rx, DMA_RX_SIZE, true);
	}
}

static void xgmac_display_tx_rings(struct xgmac_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	void *head_tx;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* Not display eCPRI ring. eCPRI ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en) {
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_ecpri_ch = 1;
	}

	/* Display TX rings */
	for (chan = num_ecpri_ch; chan < tx_cnt; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		pr_info("\tTX Queue %d rings\n", chan);

		if (priv->extend_desc)
			head_tx = (void *)tx_ch->dma_etx;
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			head_tx = (void *)tx_ch->dma_entx;
		else
			head_tx = (void *)tx_ch->dma_tx;

		xgmac_display_ring(priv, head_tx, DMA_TX_SIZE, false);
	}
}

static void xgmac_display_rings(struct xgmac_priv *priv)
{
	/* Display RX ring */
	xgmac_display_rx_rings(priv);

	/* Display TX ring */
	xgmac_display_tx_rings(priv);
}

static int xgmac_set_bfsize(int mtu, int bufsize)
{
	int ret = bufsize;

	if (mtu+14 > BUF_SIZE_8KiB)
		ret = BUF_SIZE_16KiB;
	else if (mtu+14 > BUF_SIZE_4KiB)
		ret = BUF_SIZE_8KiB;
	else if (mtu+14 > BUF_SIZE_2KiB)
		ret = BUF_SIZE_4KiB;
	else if (mtu+14 > DEFAULT_BUFSIZE)
		ret = BUF_SIZE_2KiB;
	else
		ret = DEFAULT_BUFSIZE;

	return ret;
}

/**
 * xgmac_clear_rx_descriptors - clear RX descriptors
 * @priv: driver private structure
 * @chan: RX chan index
 * Description: this function is called to clear the RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void xgmac_clear_rx_descriptors(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	int i;

	/* Clear the RX descriptors */
	for (i = 0; i < DMA_RX_SIZE; i++)
		if (priv->extend_desc)
			xgmac_init_rx_desc(priv, &rx_ch->dma_erx[i].basic,
					   priv->use_riwt);
		else
			xgmac_init_rx_desc(priv, &rx_ch->dma_rx[i],
					   priv->use_riwt);
}

/**
 * xgmac_clear_tx_descriptors - clear tx descriptors
 * @priv: driver private structure
 * @chan: TX chan index.
 * Description: this function is called to clear the TX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void xgmac_clear_tx_descriptors(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	int i;

	/* Clear the TX descriptors */
	for (i = 0; i < DMA_TX_SIZE; i++) {
		struct dma_desc *p;

		if (priv->extend_desc)
			p = &tx_ch->dma_etx[i].basic;
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			p = &tx_ch->dma_entx[i].basic;
		else
			p = &tx_ch->dma_tx[i];

		xgmac_init_tx_desc(priv, p);
	}
}

/**
 * xgmac_clear_descriptors - clear descriptors
 * @priv: driver private structure
 * Description: this function is called to clear the TX and RX descriptors
 * in case of both basic and extended descriptors are used.
 */
static void xgmac_clear_descriptors(struct xgmac_priv *priv)
{
	u32 rx_chan_cnt = priv->plat->rx_chans_to_use;
	u32 tx_chan_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_rx_ecpri_q = 0;
	u32 num_tx_ecpri_q = 0;

	/* eCPRI desc ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en) {
		num_rx_ecpri_q = 1;
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_tx_ecpri_q = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_tx_ecpri_q = 1;
	}

	/* Clear the RX descriptors */
	for (chan = num_rx_ecpri_q; chan < rx_chan_cnt; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		xgmac_clear_rx_descriptors(priv, chan);
	}

	/* Clear the TX descriptors */
	for (chan = num_tx_ecpri_q; chan < tx_chan_cnt; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		xgmac_clear_tx_descriptors(priv, chan);
	}
}

/**
 * xgmac_init_rx_buffers - init the RX descriptor buffer.
 * @priv: driver private structure
 * @p: descriptor pointer
 * @i: descriptor index
 * @flags: gfp flag
 * @chan: RX chan index
 * Description: this function is called to allocate a receive buffer, perform
 * the DMA mapping and init the descriptor.
 */
static int xgmac_init_rx_buffers(struct xgmac_priv *priv, struct dma_desc *p,
				  int i, gfp_t flags, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct xgmac_rx_buffer *buf = &rx_ch->buf_pool[i];

	buf->page = page_pool_dev_alloc_pages(rx_ch->page_pool);
	if (!buf->page)
		return -ENOMEM;

	if (priv->sph) {
		buf->sec_page = page_pool_dev_alloc_pages(rx_ch->page_pool);
		if (!buf->sec_page)
			return -ENOMEM;

		buf->sec_addr = page_pool_get_dma_addr(buf->sec_page);
		xgmac_set_desc_sec_addr(priv, p, buf->sec_addr);
	} else {
		buf->sec_page = NULL;
	}

	buf->addr = page_pool_get_dma_addr(buf->page);
	xgmac_set_desc_addr(priv, p, buf->addr);

	return 0;
}

/**
 * xgmac_free_rx_buffer - free RX dma buffers
 * @priv: private structure
 * @chan: RX chan index
 * @i: buffer index.
 */
static void xgmac_free_rx_buffer(struct xgmac_priv *priv, u32 chan, int i)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct xgmac_rx_buffer *buf = &rx_ch->buf_pool[i];

	if (buf->page)
		page_pool_put_full_page(rx_ch->page_pool, buf->page, false);
	buf->page = NULL;

	if (buf->sec_page)
		page_pool_put_full_page(rx_ch->page_pool, buf->sec_page, false);
	buf->sec_page = NULL;
}

/**
 * xgmac_free_tx_buffer - free RX dma buffers
 * @priv: private structure
 * @chan: RX chan index
 * @i: buffer index.
 */
static void xgmac_free_tx_buffer(struct xgmac_priv *priv, u32 chan, int i)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

	if (tx_ch->tx_skbuff_dma[i].buf) {
		if (tx_ch->tx_skbuff_dma[i].map_as_page)
			dma_unmap_page(priv->device,
				       tx_ch->tx_skbuff_dma[i].buf,
				       tx_ch->tx_skbuff_dma[i].len,
				       DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device,
					 tx_ch->tx_skbuff_dma[i].buf,
					 tx_ch->tx_skbuff_dma[i].len,
					 DMA_TO_DEVICE);
	}

	if (tx_ch->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_ch->tx_skbuff[i]);
		tx_ch->tx_skbuff[i] = NULL;
		tx_ch->tx_skbuff_dma[i].buf = 0;
		tx_ch->tx_skbuff_dma[i].map_as_page = false;
	}
}

/**
 * init_dma_rx_desc_rings - init the RX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_rx_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	int ret = -ENOMEM;
	int chan;
	u32 num_ecpri_ch = 0;
	int i;

	/* eCPRI Rx desc ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en)
		num_ecpri_ch = 1;

	/* RX INITIALIZATION */
	netif_dbg(priv, probe, priv->dev,
		  "SKB addresses:\nskb\t\tskb data\tdma data\n");

	/* The eCPRI chan points to host buffer ring */
	for (chan = num_ecpri_ch; chan < rx_count; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_rx_phy=0x%08x\n", __func__,
			  (u32)rx_ch->dma_rx_phy);

		xgmac_clear_rx_descriptors(priv, chan);

		for (i = 0; i < DMA_RX_SIZE; i++) {
			struct dma_desc *p;

			if (priv->extend_desc)
				p = &((rx_ch->dma_erx + i)->basic);
			else
				p = rx_ch->dma_rx + i;

			ret = xgmac_init_rx_buffers(priv, p, i, flags,
						     chan);
			if (ret)
				goto err_init_rx_buffers;
		}

		rx_ch->cur_rx = 0;
		rx_ch->dirty_rx = (unsigned int)(i - DMA_RX_SIZE);
	}

	/* Init eCPRI host buffer ring */
	if (priv->plat->ecpri_en) {
		if (ecpri_init_rx_rings(priv, 0))
			goto err_init_rx_buffers;
	}
	/* Init virtual channel desc index */
	if (priv->plat->pmd_enabled) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[rx_count];

		rx_ch->cur_rx = 0;
		rx_ch->dirty_rx = 0;
	}

	return 0;

err_init_rx_buffers:
	while (chan >= 0) {
		while (--i >= 0)
			xgmac_free_rx_buffer(priv, chan, i);

		if (chan == 0)
			break;

		i = DMA_RX_SIZE;
		chan--;
	}

	return ret;
}

/**
 * init_dma_tx_desc_rings - init the TX descriptor rings
 * @dev: net device structure.
 * Description: this function initializes the DMA TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_tx_desc_rings(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 tx_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_ecpri_ch = 0;
	u32 dev_queue = 0;
	int i;

	/* eCPRI Tx desc ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en) {
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_ecpri_ch = 1;
	}

	for (chan = num_ecpri_ch; chan < tx_count; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		netif_dbg(priv, probe, priv->dev,
			  "(%s) dma_tx_phy=0x%08x\n", __func__,
			 (u32)tx_ch->dma_tx_phy);

		for (i = 0; i < DMA_TX_SIZE; i++) {
			struct dma_desc *p;
			if (priv->extend_desc)
				p = &((tx_ch->dma_etx + i)->basic);
			else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
				p = &((tx_ch->dma_entx + i)->basic);
			else
				p = tx_ch->dma_tx + i;

			xgmac_clear_desc(priv, p);

			tx_ch->tx_skbuff_dma[i].buf = 0;
			tx_ch->tx_skbuff_dma[i].map_as_page = false;
			tx_ch->tx_skbuff_dma[i].len = 0;
			tx_ch->tx_skbuff_dma[i].last_segment = false;
			tx_ch->tx_skbuff[i] = NULL;
		}

		tx_ch->dirty_tx = 0;
		tx_ch->cur_tx = 0;
		tx_ch->mss = 0;

		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, dev_queue));
		dev_queue++;
	}
	/* Init virtual channel desc index */
	if (priv->plat->pmd_enabled) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[tx_count];

		tx_ch->dirty_tx = 0;
		tx_ch->cur_tx = 0;
		tx_ch->mss = 0;
	}

	return 0;
}

/**
 * init_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * @flags: gfp flag.
 * Description: this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers. It supports the chained and ring
 * modes.
 */
static int init_dma_desc_rings(struct net_device *dev, gfp_t flags)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int ret;

	ret = init_dma_rx_desc_rings(dev, flags);
	if (ret)
		return ret;

	ret = init_dma_tx_desc_rings(dev);
	if (ret)
		return ret;

	xgmac_clear_descriptors(priv);

	if (netif_msg_hw(priv))
		xgmac_display_rings(priv);

	return 0;
}

/**
 * dma_free_rx_skbufs - free RX dma buffers
 * @priv: private structure
 * @chan: RX chan index
 */
static void dma_free_rx_skbufs(struct xgmac_priv *priv, u32 chan)
{
	int i;

	for (i = 0; i < DMA_RX_SIZE; i++)
		xgmac_free_rx_buffer(priv, chan, i);
}

/**
 * dma_free_tx_skbufs - free TX dma buffers
 * @priv: private structure
 * @chan: TX chan index
 */
static void dma_free_tx_skbufs(struct xgmac_priv *priv, u32 chan)
{
	int i;

	for (i = 0; i < DMA_TX_SIZE; i++)
		xgmac_free_tx_buffer(priv, chan, i);
}

static void free_virt_rx_desc_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

	if (rx_ch->page_pool)
		page_pool_destroy(rx_ch->page_pool);
}

/**
 * free_dma_rx_desc_resources - free RX dma desc resources
 * @priv: private structure
 */
static void free_dma_rx_desc_resources(struct xgmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* eCPRI Rx desc is pre-allocated in LMEM */
	if (priv->plat->ecpri_en)
		num_ecpri_ch = 1;

	/* Free RX chan resources */
	for (chan = num_ecpri_ch; chan < rx_count; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		/* Release the DMA RX socket buffers */
		dma_free_rx_skbufs(priv, chan);

		/* Free DMA regions of consistent memory previously allocated */
		if (!priv->extend_desc)
			dma_free_coherent(priv->device,
					  DMA_RX_SIZE * sizeof(struct dma_desc),
					  rx_ch->dma_rx, rx_ch->dma_rx_phy);
		else
			dma_free_coherent(priv->device, DMA_RX_SIZE *
					  sizeof(struct dma_extended_desc),
					  rx_ch->dma_erx, rx_ch->dma_rx_phy);

		kfree(rx_ch->buf_pool);
		if (rx_ch->page_pool)
			page_pool_destroy(rx_ch->page_pool);
	}

	/* Free eCPRI host buffer */
	if (priv->plat->ecpri_en)
		ecpri_free_rx_resources(priv, 0);
	if (priv->plat->pmd_enabled)
		free_virt_rx_desc_resources(priv, rx_count);
}

static void free_virt_tx_desc_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

	/* Release the DMA TX socket buffers */
	dma_free_tx_skbufs(priv, chan);

	kfree(tx_ch->tx_skbuff_dma);
	kfree(tx_ch->tx_skbuff);
}

/**
 * free_dma_tx_desc_resources - free TX dma desc resources
 * @priv: private structure
 */
static void free_dma_tx_desc_resources(struct xgmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* eCPRI Tx desc is pre-allocated in LMEM */
	if (priv->plat->ecpri_en) {
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_ecpri_ch = 1;
	}

	/* Free TX chan resources */
	for (chan = num_ecpri_ch; chan < tx_count; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
		size_t size;
		void *addr;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		/* Release the DMA TX socket buffers */
		dma_free_tx_skbufs(priv, chan);

		if (priv->extend_desc) {
			size = sizeof(struct dma_extended_desc);
			addr = tx_ch->dma_etx;
		} else if (tx_ch->tbs & XGMAC_TBS_AVAIL) {
			size = sizeof(struct dma_edesc);
			addr = tx_ch->dma_entx;
		} else {
			size = sizeof(struct dma_desc);
			addr = tx_ch->dma_tx;
		}

		size *= DMA_TX_SIZE;

		dma_free_coherent(priv->device, size, addr, tx_ch->dma_tx_phy);

		kfree(tx_ch->tx_skbuff_dma);
		kfree(tx_ch->tx_skbuff);
	}
	if (priv->plat->pmd_enabled)
		free_virt_tx_desc_resources(priv, tx_count);
}

static void init_pmd_rx_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

	rx_ch->chan_index = chan;
	rx_ch->priv_data = priv;
	rx_ch->page_pool = NULL;	// Not used
	rx_ch->buf_pool = NULL;		// Not used
	rx_ch->dma_rx = NULL;		// Not used
	rx_ch->dma_rx_phy = 0;		// Not used
}

static int alloc_virt_rx_desc_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct page_pool_params pp_params = { 0 };
	unsigned int num_pages;
	int ret = -ENOMEM;

	/* RX chan buffers and DMA */
	rx_ch->chan_index = chan;
	rx_ch->priv_data = priv;

	pp_params.flags = PP_FLAG_DMA_MAP;
	pp_params.pool_size = DMA_TX_SIZE;
	num_pages = DIV_ROUND_UP(priv->dma_buf_sz, PAGE_SIZE);
	pp_params.order = ilog2(num_pages);
	pp_params.nid = dev_to_node(priv->device);
	pp_params.dev = priv->device;
	pp_params.dma_dir = DMA_FROM_DEVICE;

	rx_ch->page_pool = page_pool_create(&pp_params);
	if (IS_ERR(rx_ch->page_pool)) {
		ret = PTR_ERR(rx_ch->page_pool);
		rx_ch->page_pool = NULL;
		goto err_dma;
	}

	rx_ch->dma_rx = priv->virt_chan_rx;
	if (!rx_ch->dma_rx)
		goto err_dma;

	return 0;

err_dma:
	return ret;
}

/**
 * alloc_dma_rx_desc_resources - alloc RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_rx_desc_resources(struct xgmac_priv *priv)
{
	u32 rx_count = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	int ret = -ENOMEM;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* eCPRI Rx desc is pre-allocated in LMEM */
	if (priv->plat->ecpri_en)
		num_ecpri_ch = 1;

	/* RX chan buffers and DMA */
	for (chan = num_ecpri_ch; chan < rx_count; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
		struct page_pool_params pp_params = { 0 };
		unsigned int num_pages;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		rx_ch->chan_index = chan;
		rx_ch->priv_data = priv;

		pp_params.flags = PP_FLAG_DMA_MAP;
		pp_params.pool_size = DMA_RX_SIZE;
		num_pages = DIV_ROUND_UP(priv->dma_buf_sz, PAGE_SIZE);
		pp_params.order = ilog2(num_pages);
		pp_params.nid = dev_to_node(priv->device);
		pp_params.dev = priv->device;
		pp_params.dma_dir = DMA_FROM_DEVICE;

		rx_ch->page_pool = page_pool_create(&pp_params);
		if (IS_ERR(rx_ch->page_pool)) {
			ret = PTR_ERR(rx_ch->page_pool);
			rx_ch->page_pool = NULL;
			goto err_dma;
		}

		rx_ch->buf_pool = kcalloc(DMA_RX_SIZE, sizeof(*rx_ch->buf_pool),
					  GFP_KERNEL);
		if (!rx_ch->buf_pool)
			goto err_dma;

		if (priv->extend_desc) {
			rx_ch->dma_erx = dma_alloc_coherent(priv->device,
							    DMA_RX_SIZE *
							    sizeof(struct dma_extended_desc),
							    &rx_ch->dma_rx_phy,
							    GFP_KERNEL);
			if (!rx_ch->dma_erx)
				goto err_dma;

		} else {
			rx_ch->dma_rx = dma_alloc_coherent(priv->device,
							   DMA_RX_SIZE *
							   sizeof(struct dma_desc),
							   &rx_ch->dma_rx_phy,
							   GFP_KERNEL);
			if (!rx_ch->dma_rx)
				goto err_dma;
		}
	}

	/* Assign eCPRI ring address in LMEM and allocate eCPRI host buffer */
	if (priv->plat->ecpri_en) {
		if (ecpri_alloc_rx_resources(priv, 0))
			goto err_dma;
	} else if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++)
			init_pmd_rx_resources(priv, chan);

		alloc_virt_rx_desc_resources(priv, rx_count);
	}

	return 0;

err_dma:
	free_dma_rx_desc_resources(priv);

	return ret;
}

static void init_pmd_tx_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

	tx_ch->chan_index = chan;
	tx_ch->priv_data = priv;
	tx_ch->tx_skbuff_dma = NULL;	// Not used
	tx_ch->tx_skbuff = NULL;	// Not used
	tx_ch->dma_tx = NULL;		// Not used
	tx_ch->dma_tx_phy = 0;		// Not used
}

static int alloc_virt_tx_desc_resources(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	int ret = -ENOMEM;

	/* TX chan buffers and DMA */
	tx_ch->chan_index = chan;
	tx_ch->priv_data = priv;

	tx_ch->tx_skbuff_dma = kcalloc(DMA_TX_SIZE,
				       sizeof(*tx_ch->tx_skbuff_dma),
				       GFP_KERNEL);
	if (!tx_ch->tx_skbuff_dma)
		goto err_dma;

	tx_ch->tx_skbuff = kcalloc(DMA_TX_SIZE,
				   sizeof(struct sk_buff *),
				   GFP_KERNEL);
	if (!tx_ch->tx_skbuff)
		goto err_dma;

	tx_ch->dma_tx = priv->virt_chan_tx;
	if (!tx_ch->dma_tx)
		goto err_dma;

	return 0;

err_dma:
	return ret;
}

/**
 * alloc_dma_tx_desc_resources - alloc TX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_tx_desc_resources(struct xgmac_priv *priv)
{
	u32 tx_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	int ret = -ENOMEM;
	u32 chan;
	u32 num_ecpri_ch = 0;

	/* eCPRI Tx desc is pre-allocated in LMEM */
	if (priv->plat->ecpri_en) {
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_ecpri_ch = 1;
	}

	/* TX chan buffers and DMA */
	for (chan = num_ecpri_ch; chan < tx_count; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
		size_t size;
		void *addr;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		tx_ch->chan_index = chan;
		tx_ch->priv_data = priv;

		tx_ch->tx_skbuff_dma = kcalloc(DMA_TX_SIZE,
					       sizeof(*tx_ch->tx_skbuff_dma),
					       GFP_KERNEL);
		if (!tx_ch->tx_skbuff_dma)
			goto err_dma;

		tx_ch->tx_skbuff = kcalloc(DMA_TX_SIZE,
					   sizeof(struct sk_buff *),
					   GFP_KERNEL);
		if (!tx_ch->tx_skbuff)
			goto err_dma;

		if (priv->extend_desc)
			size = sizeof(struct dma_extended_desc);
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			size = sizeof(struct dma_edesc);
		else
			size = sizeof(struct dma_desc);

		size *= DMA_TX_SIZE;

		addr = dma_alloc_coherent(priv->device, size,
					  &tx_ch->dma_tx_phy, GFP_KERNEL);
		if (!addr)
			goto err_dma;

		if (priv->extend_desc)
			tx_ch->dma_etx = addr;
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			tx_ch->dma_entx = addr;
		else
			tx_ch->dma_tx = addr;
	}

	/* Assign eCPRI ring address in LMEM */
	if (priv->plat->ecpri_en) {
		for (chan = 0; chan < num_ecpri_ch; chan++) {
			if (ecpri_alloc_tx_resources(priv, chan))
				goto err_dma;
		}
	} else if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++)
			init_pmd_tx_resources(priv, chan);

		alloc_virt_tx_desc_resources(priv, tx_count);
	}

	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);
	return ret;
}

/**
 * alloc_dma_desc_resources - alloc TX/RX resources.
 * @priv: private structure
 * Description: according to which descriptor can be used (extend or basic)
 * this function allocates the resources for TX and RX paths. In case of
 * reception, for example, it pre-allocated the RX socket buffer in order to
 * allow zero-copy mechanism.
 */
static int alloc_dma_desc_resources(struct xgmac_priv *priv)
{
	/* RX Allocation */
	int ret = alloc_dma_rx_desc_resources(priv);

	if (ret)
		return ret;

	ret = alloc_dma_tx_desc_resources(priv);

	return ret;
}

/**
 * free_dma_desc_resources - free dma desc resources
 * @priv: private structure
 */
static void free_dma_desc_resources(struct xgmac_priv *priv)
{
	/* Release the DMA RX socket buffers */
	free_dma_rx_desc_resources(priv);

	/* Release the DMA TX socket buffers */
	free_dma_tx_desc_resources(priv);
}

/**
 *  xgmac_mac_enable_rx_queues - Enable MAC rx queues
 *  @priv: driver private structure
 *  Description: It is used for enabling the rx queues in the MAC
 */
static void xgmac_mac_enable_rx_queues(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	int queue;
	u8 mode;

	for (queue = 0; queue < rx_queues_count; queue++) {
		mode = priv->plat->rx_queues_cfg[queue].mode_to_use;
		xgmac_rx_queue_enable(priv, priv->hw, mode, queue);
	}
}

static void xgmac_mac_disable_rx_queues(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	int queue;

	for (queue = 0; queue < rx_queues_count; queue++)
		xgmac_rx_queue_disable(priv, priv->hw, queue);
}

/**
 * xgmac_start_rx_dma - start RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This starts a RX DMA channel
 */
static void xgmac_start_rx_dma(struct xgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes started in channel %d\n", chan);
	xgmac_start_rx(priv, priv->ioaddr, chan);
}

/**
 * xgmac_start_tx_dma - start TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This starts a TX DMA channel
 */
static void xgmac_start_tx_dma(struct xgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes started in channel %d\n", chan);
	xgmac_start_tx(priv, priv->ioaddr, chan);
}

/**
 * xgmac_stop_rx_dma - stop RX DMA channel
 * @priv: driver private structure
 * @chan: RX channel index
 * Description:
 * This stops a RX DMA channel
 */
static void xgmac_stop_rx_dma(struct xgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA RX processes stopped in channel %d\n", chan);
	xgmac_stop_rx(priv, priv->ioaddr, chan);
}

/**
 * xgmac_stop_tx_dma - stop TX DMA channel
 * @priv: driver private structure
 * @chan: TX channel index
 * Description:
 * This stops a TX DMA channel
 */
static void xgmac_stop_tx_dma(struct xgmac_priv *priv, u32 chan)
{
	netdev_dbg(priv->dev, "DMA TX processes stopped in channel %d\n", chan);
	xgmac_stop_tx(priv, priv->ioaddr, chan);
}

/**
 * xgmac_start_all_dma - start all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This starts all the RX and TX DMA channels
 */
static void xgmac_start_all_dma(struct xgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 tx_channels_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;


	for (chan = 0; chan < rx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		/* Enable RPF for linux may cause problem even with SW
		 * workaround. Enable RPF for linux channel if set in dts.
		 */
		if (priv->plat->linux_rpf_en)
			xgmac_enable_rpf(priv, priv->ioaddr, 1, chan);
		xgmac_start_rx_dma(priv, chan);
	}

	for (chan = 0; chan < tx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_start_tx_dma(priv, chan);
	}
}

/**
 * xgmac_stop_all_dma - stop all RX and TX DMA channels
 * @priv: driver private structure
 * Description:
 * This stops the RX and TX DMA channels
 */
static void xgmac_stop_all_dma(struct xgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 tx_channels_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	for (chan = 0; chan < rx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_stop_rx_dma(priv, chan);
	}

	for (chan = 0; chan < tx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_stop_tx_dma(priv, chan);
	}
}

/**
 *  xgmac_dma_operation_mode - HW DMA operation mode
 *  @priv: driver private structure
 *  Description: it is used for configuring the DMA operation mode register in
 *  order to program the tx/rx DMA thresholds or Store-And-Forward mode.
 */
static void xgmac_dma_operation_mode(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;
	u32 txmode = 0;
	u32 rxmode = 0;
	u32 queue = 0;
	u32 chan = 0;
	u8 qmode = 0;

	if (rxfifosz == 0 || rxfifosz > priv->dma_cap.rx_fifo_size)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0 || txfifosz > priv->dma_cap.tx_fifo_size)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_queues_count;
	txfifosz /= tx_queues_count;

	if (priv->plat->force_thresh_dma_mode) {
		txmode = tc;
		rxmode = tc;
	} else if (priv->plat->force_sf_dma_mode || priv->plat->tx_coe) {
		/*
		 * In case of GMAC, SF mode can be enabled
		 * to perform the TX COE in HW. This depends on:
		 * 1) TX COE if actually supported
		 * 2) There is no bugged Jumbo frame support
		 *    that needs to not insert csum in the TDES.
		 */
		txmode = SF_DMA_MODE;
		rxmode = SF_DMA_MODE;
		priv->xstats.threshold = SF_DMA_MODE;
	} else {
		txmode = tc;
		rxmode = SF_DMA_MODE;
	}

	/* configure all queues */
	for (queue = 0; queue < rx_queues_count; queue++) {
		qmode = priv->plat->rx_queues_cfg[queue].mode_to_use;

		xgmac_mtl_rx_mode(priv, priv->ioaddr, rxmode, queue,
				  rxfifosz, qmode);
	}

	for (queue = 0; queue < tx_queues_count; queue++) {
		qmode = priv->plat->tx_queues_cfg[queue].mode_to_use;

		xgmac_mtl_tx_mode(priv, priv->ioaddr, txmode, queue,
				  txfifosz, qmode);
	}

	/* configure all channels */
	for (chan = 0; chan < rx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
			chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_set_dma_bfsize(priv, priv->ioaddr, priv->dma_buf_sz,
				     chan);
	}
}

/**
 * xgmac_tx_clean - to manage the transmission completion
 * @priv: driver private structure
 * @chan: TX chan index
 * Description: it reclaims the transmit resources after transmission completes.
 */
static int xgmac_tx_clean(struct xgmac_priv *priv, int budget, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry, count = 0;
	u32 dev_queue = chan;

	if (priv->plat->pmd_enabled && chan >= priv->plat->pmd_start_chan)
		dev_queue = 0;

	__netif_tx_lock_bh(netdev_get_tx_queue(priv->dev, dev_queue));

	priv->xstats.tx_clean++;

	entry = tx_ch->dirty_tx;
	while ((entry != tx_ch->cur_tx) && (count < budget)) {
		struct sk_buff *skb = tx_ch->tx_skbuff[entry];
		struct dma_desc *p;
		int status;

		if (priv->extend_desc)
			p = (struct dma_desc *)(tx_ch->dma_etx + entry);
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			p = &tx_ch->dma_entx[entry].basic;
		else
			p = tx_ch->dma_tx + entry;

		status = xgmac_tx_status(priv, &priv->dev->stats,
				&priv->xstats, p, priv->ioaddr);
		/* Check if the descriptor is owned by the DMA */
		if (unlikely(status & tx_dma_own))
			break;

		count++;

		/* Make sure descriptor fields are read after reading
		 * the own bit.
		 */
		dma_rmb();

		/* Just consider the last segment and ...*/
		if (likely(!(status & tx_not_ls))) {
			/* ... verify the status error condition */
			if (unlikely(status & tx_err)) {
				priv->dev->stats.tx_errors++;
			} else {
				priv->dev->stats.tx_packets++;
				priv->xstats.tx_pkt_n++;
			}
			if (!virt_chan_avail(priv) ||
			    chan != priv->plat->tx_chans_to_use)
				xgmac_get_tx_hwtstamp(priv, p, skb);
		}

		if (likely(tx_ch->tx_skbuff_dma[entry].buf)) {
			if (tx_ch->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device,
					       tx_ch->tx_skbuff_dma[entry].buf,
					       tx_ch->tx_skbuff_dma[entry].len,
					       DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device,
						 tx_ch->tx_skbuff_dma[entry].buf,
						 tx_ch->tx_skbuff_dma[entry].len,
						 DMA_TO_DEVICE);
			tx_ch->tx_skbuff_dma[entry].buf = 0;
			tx_ch->tx_skbuff_dma[entry].len = 0;
			tx_ch->tx_skbuff_dma[entry].map_as_page = false;
		}

		xgmac_clean_desc3(priv, tx_ch, p);

		tx_ch->tx_skbuff_dma[entry].last_segment = false;
		tx_ch->tx_skbuff_dma[entry].is_jumbo = false;

		if (likely(skb != NULL)) {
			pkts_compl++;
			bytes_compl += skb->len;
			dev_consume_skb_any(skb);
			tx_ch->tx_skbuff[entry] = NULL;
		}

		if (!virt_chan_avail(priv) ||
		    chan != priv->plat->tx_chans_to_use)
			xgmac_release_tx_desc(priv, p, priv->mode);

		entry = XGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	}
	tx_ch->dirty_tx = entry;

	if (!virt_chan_avail(priv) || chan != priv->plat->tx_chans_to_use)
		netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, dev_queue),
					  pkts_compl, bytes_compl);

	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,
								dev_queue))) &&
	    xgmac_tx_avail(priv, chan) > XGMAC_TX_THRESH) {

		netif_dbg(priv, tx_done, priv->dev,
			  "%s: restart transmit\n", __func__);
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, dev_queue));
	}

	if ((priv->eee_enabled) && (!priv->tx_path_in_lpi_mode)) {
		xgmac_enable_eee_mode(priv);
		mod_timer(&priv->eee_ctrl_timer, XGMAC_LPI_T(eee_timer));
	}

	/* We still have pending packets, let's call for a new scheduling */
	if (tx_ch->dirty_tx != tx_ch->cur_tx)
		if (!virt_chan_avail(priv) ||
		    chan != priv->plat->tx_chans_to_use)
			mod_timer(&tx_ch->txtimer,
				  XGMAC_TIMER_TICK(priv->tx_coal_timer));

	__netif_tx_unlock_bh(netdev_get_tx_queue(priv->dev, dev_queue));

	return count;
}

/**
 * xgmac_tx_err - to manage the tx error
 * @priv: driver private structure
 * @chan: channel index
 * Description: it cleans the descriptors and restarts the transmission
 * in case of transmission errors.
 */
static void xgmac_tx_err(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	u32 dev_queue = chan;

	if (priv->plat->pmd_enabled && chan >= priv->plat->pmd_start_chan)
		dev_queue = 0;

	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, dev_queue));

	xgmac_stop_tx_dma(priv, chan);
	dma_free_tx_skbufs(priv, chan);
	xgmac_clear_tx_descriptors(priv, chan);
	tx_ch->dirty_tx = 0;
	tx_ch->cur_tx = 0;
	tx_ch->mss = 0;
	netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, dev_queue));
	xgmac_init_tx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
			    tx_ch->dma_tx_phy, chan);
	xgmac_start_tx_dma(priv, chan);

	priv->dev->stats.tx_errors++;
	netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, dev_queue));
}

/**
 *  xgmac_set_dma_operation_mode - Set DMA operation mode by channel
 *  @priv: driver private structure
 *  @txmode: TX operating mode
 *  @rxmode: RX operating mode
 *  @chan: channel index
 *  Description: it is used for configuring of the DMA operation mode in
 *  runtime in order to program the tx/rx DMA thresholds or Store-And-Forward
 *  mode.
 */
#if 0 // not used for XGMAC/XLGMAC
static void xgmac_set_dma_operation_mode(struct xgmac_priv *priv, u32 txmode,
					  u32 rxmode, u32 queue)
{
	u8 rxqmode = priv->plat->rx_queues_cfg[queue].mode_to_use;
	u8 txqmode = priv->plat->tx_queues_cfg[queue].mode_to_use;
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;

	if (rxfifosz == 0 || rxfifosz > priv->dma_cap.rx_fifo_size)
		rxfifosz = priv->dma_cap.rx_fifo_size;
	if (txfifosz == 0 || txfifosz > priv->dma_cap.tx_fifo_size)
		txfifosz = priv->dma_cap.tx_fifo_size;

	/* Adjust for real per queue fifo size */
	rxfifosz /= rx_queues_count;
	txfifosz /= tx_queues_count;

	xgmac_mtl_rx_mode(priv, priv->ioaddr, rxmode, queue, rxfifosz, rxqmode);
	xgmac_mtl_tx_mode(priv, priv->ioaddr, txmode, queue, txfifosz, txqmode);
}
#endif

static bool xgmac_safety_feat_interrupt(struct xgmac_priv *priv)
{
	int ret;

	ret = xgmac_safety_feat_irq_status(priv, priv->dev,
			priv->ioaddr, priv->dma_cap.asp, &priv->sstats);
	if (ret && (ret != -EINVAL && ret != -ENODEV)) {
		xgmac_global_err(priv);
		return true;
	}

	return false;
}

static int xgmac_napi_check(struct xgmac_priv *priv, u32 chan)
{
	int status = xgmac_dma_interrupt_status(priv, priv->ioaddr,
						 &priv->xstats, chan);
	struct xgmac_channel *ch = &priv->channel[chan];
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 int_en, tail_addr;
	volatile struct xgmac_uio_shm *xgmac_ch_uio_shm = (struct xgmac_uio_shm *)
			((u8 *)(priv->uio_shared_mem) + (XGMAC_UIO_SHARED_MEM_OFFSET * chan));
#ifndef CONFIG_PREEMPT_RT
	unsigned long flags;
#endif

	if (status & handle_rbu && chan < priv->plat->rx_chans_to_use) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan) {
			/* Disable RBU interrupt using AIE */
			int_en = readl(priv->ioaddr + XGMAC_DMA_CH_INT_EN(chan));
			int_en &= ~XGMAC_AIE;
			writel(int_en, priv->ioaddr + XGMAC_DMA_CH_INT_EN(chan));

			/* Disable DMA before setting tail pointer */
			xgmac_stop_rx_dma(priv, chan);

			/* Read current tail address */
			tail_addr = readl(priv->ioaddr + XGMAC_DMA_CH_RxDESC_TAIL_LPTR(chan));
			/* Update tail pointer to handle DMA suspend state */
			xgmac_set_rx_tail_ptr(priv, priv->ioaddr, tail_addr, chan);

			/* Set the flag to indicate that DMA has been stopped */
			xgmac_ch_uio_shm->rbu_interrupt_flag = 1;
			return status;
		} else if (priv->plat->linux_rpf_en) {
			/* Disable DMA before setting tail pointer */
			xgmac_stop_rx_dma(priv, chan);

			tail_addr = rx_ch->rx_tail_addr;
			/* Update tail pointer to handle DMA suspend state */
			xgmac_set_rx_tail_ptr(priv, priv->ioaddr, tail_addr, chan);

			/* Add Rx flag to status to schedule napi thread */
			status |= handle_rx;

			/* Set the flag to indicate that DMA has been stopped */
			priv->channel[chan].rx_dma_stopped = true;
		} else {
			/* Add Rx flag to status to schedule napi thread */
			status |= handle_rx;
		}
	}

	if (priv->plat->pmd_enabled && chan >= pmd_start_chan && chan < pmd_end_chan)
		return status;

	if (status & handle_rx && chan < priv->plat->rx_chans_to_use) {
		if (napi_schedule_prep(&ch->rx_napi)) {
#ifndef CONFIG_PREEMPT_RT
			spin_lock_irqsave(&ch->lock, flags);
#endif
			xgmac_disable_dma_irq(priv, priv->ioaddr, chan, 1, 0);
#ifndef CONFIG_PREEMPT_RT
			spin_unlock_irqrestore(&ch->lock, flags);
			__napi_schedule_irqoff(&ch->rx_napi);
#else
			__napi_schedule(&ch->rx_napi);
#endif
		}
	}

	if (status & handle_tx && chan < priv->plat->tx_chans_to_use) {
		if (napi_schedule_prep(&ch->tx_napi)) {
#ifndef CONFIG_PREEMPT_RT
			spin_lock_irqsave(&ch->lock, flags);
#endif
			xgmac_disable_dma_irq(priv, priv->ioaddr, chan, 0, 1);
#ifndef CONFIG_PREEMPT_RT
			spin_unlock_irqrestore(&ch->lock, flags);
			__napi_schedule_irqoff(&ch->tx_napi);
#else
			__napi_schedule(&ch->tx_napi);
#endif
		}
	}

	return status;
}

/**
 * xgmac_dma_interrupt - DMA ISR
 * @priv: driver private structure
 * Description: this is the DMA ISR. It is called by the main ISR.
 * It calls the dwmac dma routine and schedule poll method in case of some
 * work can be done.
 */
static void xgmac_dma_interrupt(struct xgmac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_chans_to_use;
	u32 rx_channel_count = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 channels_to_check = tx_channel_count > rx_channel_count ?
				tx_channel_count : rx_channel_count;
	u32 chan;
	int status[max_t(u32, MTL_MAX_TX_QUEUES, MTL_MAX_RX_QUEUES)];

	/* Make sure we never check beyond our status buffer. */
	if (WARN_ON_ONCE(channels_to_check > ARRAY_SIZE(status)))
		channels_to_check = ARRAY_SIZE(status);

	for (chan = 0; chan < channels_to_check; chan++)
		status[chan] = xgmac_napi_check(priv, chan);

	for (chan = 0; chan < tx_channel_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		if (unlikely(status[chan] == tx_hard_error)) {
			xgmac_tx_err(priv, chan);
		}
	}
}

/**
 * xgmac_mmc_setup: setup the Mac Management Counters (MMC)
 * @priv: driver private structure
 * Description: this masks the MMC irq, in fact, the counters are managed in SW.
 */
static void xgmac_mmc_setup(struct xgmac_priv *priv)
{
	unsigned int mode = MMC_CNTRL_RESET_ON_READ | MMC_CNTRL_COUNTER_RESET |
			    MMC_CNTRL_PRESET | MMC_CNTRL_FULL_HALF_PRESET;

	xgmac_mmc_intr_all_mask(priv, priv->mmcaddr);

	if (priv->dma_cap.rmon) {
		xgmac_mmc_ctrl(priv, priv->mmcaddr, mode);
		memset(&priv->mmc, 0, sizeof(struct xgmac_counters));
	} else
		netdev_info(priv->dev, "No MAC Management Counters available\n");
}

//TODO
/**
 * xgmac_get_hw_features - get MAC capabilities from the HW cap. register.
 * @priv: driver private structure
 * Description:
 *  new GMAC chip generations have a new register to indicate the
 *  presence of the optional feature/functions.
 *  This can be also used to override the value passed through the
 *  platform and necessary for old MAC10/100 and GMAC chips.
 */
static int xgmac_get_hw_features(struct xgmac_priv *priv)
{
	return xgmac_get_hw_feature(priv, priv->ioaddr, &priv->dma_cap) == 0;
}

/**
 * xgmac_check_ether_addr - check if the MAC addr is valid
 * @priv: driver private structure
 * Description:
 * it is to verify if the MAC address is valid, in case of failures it
 * generates a random MAC address
 */
static void xgmac_check_ether_addr(struct xgmac_priv *priv)
{
	if (!is_valid_ether_addr(priv->dev->dev_addr)) {
		xgmac_get_umac_addr(priv, priv->hw, priv->dev->dev_addr, 0);
		if (!is_valid_ether_addr(priv->dev->dev_addr))
			eth_hw_addr_random(priv->dev);
		dev_info(priv->device, "device MAC address %pM\n",
			 priv->dev->dev_addr);
	}
}

/**
 * xgmac_init_dma_engine - DMA init.
 * @priv: driver private structure
 * Description:
 * It inits the DMA invoking the specific MAC/GMAC callback.
 * Some DMA parameters can be passed from the platform;
 * in case of these are not passed a default is kept for the MAC or GMAC.
 */
static int xgmac_init_dma_engine(struct xgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 tx_channels_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 dma_csr_ch = max(rx_channels_count, tx_channels_count);
	struct xgmac_rx_chan *rx_ch;
	struct xgmac_tx_chan *tx_ch;
	u32 chan = 0;
	int atds = 0;
	int ret = 0;

	if (!priv->plat->dma_cfg || !priv->plat->dma_cfg->pbl) {
		dev_err(priv->device, "Invalid DMA configuration\n");
		return -EINVAL;
	}

	ret = xgmac_reset(priv, priv->ioaddr);
	if (ret) {
		dev_err(priv->device, "Failed to reset the dma\n");
		return ret;
	}

	/* DMA Configuration */
	xgmac_dma_init(priv, priv->ioaddr, priv->plat->dma_cfg, atds);

	if (priv->plat->axi)
		xgmac_axi(priv, priv->ioaddr, priv->plat->axi);

	/* Select DMA per channel interrupt */
	if (priv->plat->per_ch_irq)
		xgmac_set_dma_int_mode(priv, priv->ioaddr, XGMAC_INTM_LEVEL);

	/* DMA CSR Channel configuration */
	for (chan = 0; chan < dma_csr_ch; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		xgmac_init_chan(priv, priv->ioaddr,
				priv->plat->dma_cfg, chan, 1);
	}

	/* DMA RX Channel Configuration */
	for (chan = 0; chan < rx_channels_count; chan++) {
		rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan) {
			rx_ch->rx_tail_addr = 0;
			continue;
		}

		xgmac_init_rx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    rx_ch->dma_rx_phy, chan);

		/* For eCPRI channel, init tail ptr to no-free desc. eCPRI
		 * SS will update tail ptr after it initializes desc ring.
		 */
		if (priv->plat->ecpri_en && chan == 0)
			rx_ch->rx_tail_addr = rx_ch->dma_rx_phy;
		else
			rx_ch->rx_tail_addr = rx_ch->dma_rx_phy +
			    (DMA_RX_SIZE * sizeof(struct dma_desc));
		xgmac_set_rx_tail_ptr(priv, priv->ioaddr,
				       rx_ch->rx_tail_addr, chan);
	}

	/* DMA TX Channel Configuration */
	for (chan = 0; chan < tx_channels_count; chan++) {
		tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan) {
			tx_ch->tx_tail_addr = 0;
			continue;
		}

		xgmac_init_tx_chan(priv, priv->ioaddr, priv->plat->dma_cfg,
				    tx_ch->dma_tx_phy, chan);

		tx_ch->tx_tail_addr = tx_ch->dma_tx_phy;
		xgmac_set_tx_tail_ptr(priv, priv->ioaddr,
				       tx_ch->tx_tail_addr, chan);
	}

	return ret;
}

static void xgmac_tx_timer_arm(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

	mod_timer(&tx_ch->txtimer, XGMAC_TIMER_TICK(priv->tx_coal_timer));
}

/**
 * xgmac_tx_timer - mitigation sw timer for tx.
 * @data: data pointer
 * Description:
 * This is the timer handler to directly invoke the xgmac_tx_clean.
 */
static void xgmac_tx_timer(struct timer_list *t)
{
	struct xgmac_tx_chan *tx_ch = from_timer(tx_ch, t, txtimer);
	struct xgmac_priv *priv = tx_ch->priv_data;
	struct xgmac_channel *ch;

	ch = &priv->channel[tx_ch->chan_index];

	if (likely(napi_schedule_prep(&ch->tx_napi))) {
		unsigned long flags;

		spin_lock_irqsave(&ch->lock, flags);
		xgmac_disable_dma_irq(priv, priv->ioaddr, ch->index, 0, 1);
		spin_unlock_irqrestore(&ch->lock, flags);
		__napi_schedule(&ch->tx_napi);
	}
}

/**
 * xgmac_init_coalesce - init mitigation options.
 * @priv: driver private structure
 * Description:
 * This inits the coalesce parameters: i.e. timer rate,
 * timer handler and default threshold used for enabling the
 * interrupt on completion bit.
 */
static void xgmac_init_coalesce(struct xgmac_priv *priv)
{
	u32 tx_channel_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	priv->tx_coal_frames = XGMAC_TX_FRAMES;
	priv->tx_coal_timer = XGMAC_COAL_TX_TIMER;
	priv->rx_coal_frames = XGMAC_RX_FRAMES;
	priv->rx_riwt = DEF_DMA_RIWT;

	for (chan = 0; chan < tx_channel_count; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		timer_setup(&tx_ch->txtimer, xgmac_tx_timer, 0);
	}
}

static void xgmac_set_rings_length(struct xgmac_priv *priv)
{
	struct ecpri_ctrl_status *ecpri_cs;
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 tx_channels_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_rx_ecpri_ch = 0;
	u32 num_tx_ecpri_ch = 0;

	/* eCPRI ring length is different */
	if (priv->plat->ecpri_en) {
		num_rx_ecpri_ch = 1;
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_tx_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_tx_ecpri_ch = 1;
	}

	/* set TX ring length */
	for (chan = num_tx_ecpri_ch; chan < tx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_set_tx_ring_len(priv, priv->ioaddr,
				(DMA_TX_SIZE - 1), chan);
	}

	/* eCPRI ring length is different */
	ecpri_cs = (struct ecpri_ctrl_status *)priv->ecpri_base;
	if (priv->plat->ecpri_en) {
		for (chan = 0; chan < num_tx_ecpri_ch; chan++) {
			xgmac_set_tx_ring_len(priv, priv->ioaddr,
				(ecpri_cs->tx_desc_ring_size - 1), chan);
		}
	}

	/* set RX ring length */
	for (chan = num_rx_ecpri_ch; chan < rx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_set_rx_ring_len(priv, priv->ioaddr,
				(DMA_RX_SIZE - 1), chan);
	}

	/* eCPRI ring length is different */
	if (priv->plat->ecpri_en) {
		for (chan = 0; chan < num_rx_ecpri_ch; chan++) {
			xgmac_set_rx_ring_len(priv, priv->ioaddr,
				(ecpri_cs->rx_desc_ring_size - 1), chan);
		}
	}
}

/**
 *  xgmac_set_tx_queue_weight - Set TX queue weight
 *  @priv: driver private structure
 *  Description: It is used for setting TX queues weight
 */
static void xgmac_set_tx_queue_weight(struct xgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 weight;
	u32 queue;

	for (queue = 0; queue < tx_queues_count; queue++) {
		weight = priv->plat->tx_queues_cfg[queue].weight;
		xgmac_set_mtl_tx_queue_weight(priv, priv->hw, weight, queue);
	}
}

/**
 *  xgmac_configure_cbs - Configure CBS in TX queue
 *  @priv: driver private structure
 *  Description: It is used for configuring CBS in AVB TX queues
 */
static void xgmac_configure_cbs(struct xgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 mode_to_use;
	u32 queue;

	/* queue 0 is reserved for legacy traffic */
	for (queue = 1; queue < tx_queues_count; queue++) {
		mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
		if (mode_to_use == MTL_QUEUE_DCB)
			continue;

		xgmac_config_cbs(priv, priv->hw,
				priv->plat->tx_queues_cfg[queue].send_slope,
				priv->plat->tx_queues_cfg[queue].idle_slope,
				priv->plat->tx_queues_cfg[queue].high_credit,
				priv->plat->tx_queues_cfg[queue].low_credit,
				queue);
	}
}

/**
 *  xgmac_rx_queue_dma_chan_map - Map RX queue to RX dma channel
 *  @priv: driver private structure
 *  Description: It is used for mapping RX queues to RX dma channels
 */
static void xgmac_rx_queue_dma_chan_map(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 chan;

	/* With eCPRI, DMA mapping is different */
	if (priv->plat->ecpri_en) {
		ecpri_rx_queue_dma_chan_map(priv);
		return;
	}

	for (queue = 0; queue < rx_queues_count; queue++) {
		if (queue == (rx_queues_count - 1) &&
		    priv->plat->pmd_enabled &&
		    priv->plat->pmd_start_chan == 0) {
			chan = priv->plat->rx_chans_to_use - 1;
			xgmac_map_mtl_to_dma(priv, priv->hw, queue, chan);
			continue;
		}

		if (priv->plat->dynamic_dma_map) {
			xgmac_map_mtl_to_dma(priv, priv->hw, queue,
					      XGMAC_QDDMACH);
		}
		else {
			chan = priv->plat->rx_queues_cfg[queue].chan;
			xgmac_map_mtl_to_dma(priv, priv->hw, queue, chan);
		}
	}
}

/**
 *  xgmac_mac_config_rx_queues_prio - Configure RX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX Queue Priority
 */
static void xgmac_mac_config_rx_queues_prio(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 prio;

	for (queue = 0; queue < rx_queues_count; queue++) {
		if (!priv->plat->rx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->rx_queues_cfg[queue].prio;
		xgmac_rx_queue_prio(priv, priv->hw, prio, queue);
	}
}

/**
 *  xgmac_mac_config_tx_queues_prio - Configure TX Queue priority
 *  @priv: driver private structure
 *  Description: It is used for configuring the TX Queue Priority
 */
static void xgmac_mac_config_tx_queues_prio(struct xgmac_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue;
	u32 prio;

	for (queue = 0; queue < tx_queues_count; queue++) {
		if (!priv->plat->tx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->tx_queues_cfg[queue].prio;
		xgmac_tx_queue_prio(priv, priv->hw, prio, queue);
	}
}

/**
 *  pmd_rxqueues_routing - Reserve a L4 filter. PMD configures it.
 *  @priv: driver private structure
 *  Description: It is used for reserving L4 filter for PMD.
 *  L4 Filter 6,7 - Unicast PTP
 *  L4 Filter 5 - DHCPv4
 */
static void pmd_rxqueues_routing(struct xgmac_priv *priv)
{
	struct xgmac_flow_entry *entry0 = &priv->flow_entries[0];
	//int ptp_evt_idx = priv->dma_cap.l3l4fnum - 2;
	//int ptp_gen_idx = priv->dma_cap.l3l4fnum - 1;
	//int ptp_evt_port = 319;
	//int ptp_gen_port = 320;
	//struct xgmac_flow_entry *entry6 = &priv->flow_entries[ptp_evt_idx];
	//struct xgmac_flow_entry *entry7 = &priv->flow_entries[ptp_gen_idx];
	//int linux_ch = priv->plat->rx_chans_to_use - 1;

	entry0->in_use = true;
	entry0->is_l4 = true;
	entry0->ip_proto = IPPROTO_UDP;
	netdev_info(priv->dev, "Reserve L4 filter %d for EGTPU\n",
		    entry0->idx);
	if (priv->plat->pmd_start_chan == 0) {
		/* Route L2 PTP to the last queue */
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_PTPQ,
				       priv->plat->rx_queues_to_use - 1, true);

		/* Route L2 MCBCQ to the last queue */
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ,
				       priv->plat->rx_queues_to_use - 1, true);

		/* Route UDP PTP to last DMA channel */
		/* Need ARP to work first */
		//entry6->in_use = true;
		//entry6->is_l4 = true;
		//entry6->ip_proto = IPPROTO_UDP;
		//xgmac_config_l4_filter(priv, priv->hw, ptp_evt_idx, true,
		//		true, false, false, ptp_evt_port, true, linux_ch);
		//entry7->in_use = true;
		//entry7->is_l4 = true;
		//entry7->ip_proto = IPPROTO_UDP;
		//xgmac_config_l4_filter(priv, priv->hw, ptp_gen_idx, true,
		//		true, false, false, ptp_gen_port, true, linux_ch);
	}
}

/**
 *  xgmac_mac_config_rx_queues_routing - Configure RX Queue Routing
 *  @priv: driver private structure
 *  Description: It is used for configuring the RX queue routing
 */
static void xgmac_mac_config_rx_queues_routing(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u8 packet;

	/* Use eCPRI queue routing */
	if (priv->plat->ecpri_en) {
		ecpri_rxqueues_routing(priv);
		return;
	}

	if (priv->plat->pmd_enabled) {
		pmd_rxqueues_routing(priv);
		return;
	}

	for (queue = 0; queue < rx_queues_count; queue++) {
		/* no specific packet type routing specified for the queue */
		if (priv->plat->rx_queues_cfg[queue].pkt_route == 0x0)
			continue;

		packet = priv->plat->rx_queues_cfg[queue].pkt_route;
		xgmac_rx_queue_routing(priv, priv->hw, packet, queue, true);
	}
}

static void xgmac_mac_config_rss(struct xgmac_priv *priv)
{
	if (!priv->dma_cap.rssen || !priv->plat->rss_en) {
		priv->rss.enable = false;
		return;
	}

	/* No RSS if eCPRI or PMD enabled */
	if (priv->plat->ecpri_en || priv->plat->pmd_enabled)
		priv->rss.enable = false;
	else if (priv->dev->features & NETIF_F_RXHASH)
		priv->rss.enable = true;
	else
		priv->rss.enable = false;

	xgmac_rss_configure(priv, priv->hw, &priv->rss,
			     priv->plat->rx_queues_to_use);
}

/**
 *  xgmac_mtl_configuration - Configure MTL
 *  @priv: driver private structure
 *  Description: It is used for configurring MTL
 */
static void xgmac_mtl_configuration(struct xgmac_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;

	if (tx_queues_count > 1)
		xgmac_set_tx_queue_weight(priv);

	/* Configure MTL RX algorithms */
	if (rx_queues_count > 1)
		xgmac_prog_mtl_rx_algorithms(priv, priv->hw,
				priv->plat->rx_sched_algorithm);

	/* Configure MTL TX algorithms */
	if (tx_queues_count > 1)
		xgmac_prog_mtl_tx_algorithms(priv, priv->hw,
				priv->plat->tx_sched_algorithm);

	/* Configure CBS in AVB TX queues */
	if (tx_queues_count > 1)
		xgmac_configure_cbs(priv);

	/* Map RX MTL to DMA channels */
	xgmac_rx_queue_dma_chan_map(priv);

	/* Set RX priorities */
	if (rx_queues_count > 1)
		xgmac_mac_config_rx_queues_prio(priv);

	/* Set TX priorities */
	if (tx_queues_count > 1)
		xgmac_mac_config_tx_queues_prio(priv);

	/* Set RX routing */
	if (rx_queues_count > 1)
		xgmac_mac_config_rx_queues_routing(priv);

	/* Receive Side Scaling */
	if (rx_queues_count > 1)
		xgmac_mac_config_rss(priv);
}

/**
 * xgmac_hw_setup - setup mac in a usable state.
 *  @dev : pointer to the device structure.
 *  Description:
 *  this is the main function to setup the HW in a usable state because the
 *  dma engine is reset, the core registers are configured (e.g. AXI,
 *  Checksum features, timers). The DMA is ready to start receiving and
 *  transmitting.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int xgmac_hw_setup(struct net_device *dev, bool init_ptp)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_chans_to_use;
	u32 tx_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	int speed = priv->plat->mac_port_sel_speed;
	u32 chan;
	int ret;

	ret = xgmac_xpcs_config(priv, priv->xpcs_base,
				priv->plat->phy_interface);
	if (ret < 0 && ret != -ENODEV) {
		netdev_err(priv->dev, "%s: XPCS config failed\n", __func__);
		return ret;
	}

	/* DMA initialization and SW reset */
	ret = xgmac_init_dma_engine(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA engine initialization failed\n",
			   __func__);
		return ret;
	}

	/* Copy the MAC addr into the HW  */
	xgmac_set_umac_addr(priv, priv->hw, dev->dev_addr, 0);
	/* Divert all traffic matching MAC address to Linux DMA channel */
	if (priv->plat->pmd_enabled && pmd_start_chan == 0)
		xgmac_set_umac_dma_chan(priv, priv->hw, 0, rx_cnt - 1);

	/* PS and related bits will be programmed according to the speed */
	if ((speed == SPEED_10) || (speed == SPEED_100) ||
	    (speed == SPEED_1000) || (speed == SPEED_10000) ||
	    (speed == SPEED_25000) || (speed == SPEED_50000) ||
	    (speed == SPEED_2500) || (speed == SPEED_5000)) {
		priv->hw->ps = speed;
	} else {
		dev_warn(priv->device, "invalid port speed\n");
		priv->hw->ps = 0;
	}

	/* Initialize the MAC Core */
	xgmac_core_init(priv, priv->hw, dev);

	/* Enable jumbo packet since eCPRI could have jumbo packet */
	if (priv->plat->ecpri_en)
		xgmac_set_jumbo_enable(priv, priv->ioaddr, true, true);

	/* Initialize MTL*/
	xgmac_mtl_configuration(priv);

	ret = xgmac_rx_ipc(priv, priv->hw);
	if (!ret) {
		netdev_warn(priv->dev, "RX IPC Checksum Offload disabled\n");
		priv->plat->rx_coe = XGMAC_RX_COE_NONE;
		priv->hw->rx_csum = 0;
	}

	/* Enable the MAC Rx/Tx */
	xgmac_mac_set(priv, priv->ioaddr, true);

	/* Set the HW DMA mode and the COE */
	xgmac_dma_operation_mode(priv);

	xgmac_mmc_setup(priv);

	if (init_ptp) {
		ret = clk_prepare_enable(priv->plat->clk_ptp_ref);
		if (ret < 0)
			netdev_warn(priv->dev,
				"failed to enable PTP reference clock: %d\n", ret);

		ret = xgmac_init_ptp(priv);
		if (ret == -EOPNOTSUPP)
			netdev_warn(priv->dev, "PTP not supported by HW\n");
		else if (ret)
			netdev_warn(priv->dev, "PTP init failed\n");

		xgmac_fixed_pps_config(priv, priv->ioaddr, 0,
				       priv->plat->clk_ptp_rate,
				       XGMAC_PPSCTRL0_1HZ);
	}

	priv->tx_lpi_timer = XGMAC_DEFAULT_TWT_LS;

	if (priv->use_riwt) {
		if (!priv->rx_riwt)
			priv->rx_riwt = DEF_DMA_RIWT;
		if (!priv->rx_coal_frames)
			priv->rx_coal_frames = XGMAC_RX_FRAMES;

		ret = xgmac_rx_watchdog(priv, priv->ioaddr, priv->rx_riwt,
					priv->rx_coal_frames, rx_cnt);
	}

	if (priv->hw->pcs)
		xgmac_pcs_ctrl_ane(priv, priv->ioaddr, 1, priv->hw->ps, 0);

	/* set TX and RX rings length */
	xgmac_set_rings_length(priv);

	/* Enable TSO */
	if (priv->tso) {
		for (chan = 0; chan < tx_cnt; chan++) {
			if (priv->plat->pmd_enabled &&
			    chan >= pmd_start_chan && chan < pmd_end_chan)
				continue;
			xgmac_enable_tso(priv, priv->ioaddr, 1, chan);
		}
	}

	/* Enable Split Header */
	/* eCPRI channel or pmd channel do not do sph */
	if (priv->sph && priv->hw->rx_csum) {
		for (chan = 0; chan < rx_cnt; chan++) {
			if (priv->plat->pmd_enabled &&
			    chan >= pmd_start_chan && chan < pmd_end_chan)
				continue;
			if (priv->plat->ecpri_en && chan == 0)
				xgmac_enable_sph(priv, priv->ioaddr, 0, chan);
			else
				xgmac_enable_sph(priv, priv->ioaddr, 1, chan);
		}
	}

	/* VLAN Tag Insertion */
	if (priv->dma_cap.vlins)
		xgmac_enable_vlan(priv, priv->hw, XGMAC_VLAN_INSERT);

	/* TBS */
	for (chan = 0; chan < tx_cnt; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
		int enable = tx_ch->tbs & XGMAC_TBS_AVAIL;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_enable_tbs(priv, priv->ioaddr, enable, chan);
	}

	/* Start the ball rolling... */
	xgmac_start_all_dma(priv);

	/* Enable MAC RX Queues */
	xgmac_mac_enable_rx_queues(priv);

	return 0;
}

static void xgmac_hw_teardown(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	clk_disable_unprepare(priv->plat->clk_ptp_ref);
}

static int xgmac_request_irqs(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct xgmac_channel *ch;
	u32 rx_chan_cnt;
	u32 tx_chan_cnt;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	int ret;
	unsigned long irq_flags;

#ifndef CONFIG_PREEMPT_RT
	irq_flags = IRQF_SHARED;
#else
	irq_flags = IRQF_SHARED | IRQF_NO_THREAD;
#endif

	/* Request the IRQ lines */
	ret = request_irq(dev->irq, xgmac_interrupt,
			  irq_flags, dev->name, dev);

	if (unlikely(ret < 0)) {
		netdev_err(priv->dev,
			   "%s: ERROR: IRQ %d (error: %d)\n",
			   __func__, dev->irq, ret);
		goto irq_error;
	}

	irq_set_affinity(dev->irq, cpumask_of(priv->plat->irq_cpu));

	if (priv->plat->per_ch_irq) {
		rx_chan_cnt = priv->plat->rx_chans_to_use;
		for (chan = 0; chan < rx_chan_cnt; chan++) {
			if (priv->plat->pmd_enabled &&
			    chan >= pmd_start_chan && chan < pmd_end_chan)
				continue;

			ch = &priv->channel[chan];
			snprintf(ch->rx_irq_name, sizeof(ch->rx_irq_name)-1,
				 "%s-rx-%u", dev->name, ch->index);

			ret = request_irq(ch->rx_irq, xgmac_dma_isr,
					  irq_flags, ch->rx_irq_name, ch);

			if (unlikely(ret < 0)) {
				netdev_err(priv->dev,
					   "%s: ERROR: IRQ %d (error: %d)\n",
					   __func__, ch->rx_irq, ret);
				goto rxirq_error;
			}

			irq_set_affinity(ch->rx_irq,
					 cpumask_of(ch->rx_irq_cpu));
		}

		tx_chan_cnt = priv->plat->tx_chans_to_use;
		for (chan = 0; chan < tx_chan_cnt; chan++) {
			if (priv->plat->pmd_enabled &&
			    chan >= pmd_start_chan && chan < pmd_end_chan)
				continue;

			ch = &priv->channel[chan];
			snprintf(ch->tx_irq_name, sizeof(ch->tx_irq_name)-1,
				 "%s-tx-%u", dev->name, ch->index);

			ret = request_irq(ch->tx_irq, xgmac_dma_isr,
					  irq_flags, ch->tx_irq_name, ch);

			if (unlikely(ret < 0)) {
				netdev_err(priv->dev,
					   "%s: ERROR: IRQ %d (error: %d)\n",
					   __func__, ch->tx_irq, ret);
				goto txirq_error;
			}

			irq_set_affinity(ch->tx_irq,
					 cpumask_of(ch->tx_irq_cpu));
		}
	}

	return 0;

txirq_error:
	for (; chan > 0; chan--) {
		u32 ch_idx = chan - 1;

		if (priv->plat->pmd_enabled &&
		    ch_idx >= pmd_start_chan && ch_idx < pmd_end_chan)
			continue;
		ch = &priv->channel[ch_idx];
		free_irq(ch->tx_irq, ch);
	}
	for (chan = rx_chan_cnt; chan > 0; chan--) {
		u32 ch_idx = chan - 1;

		if (priv->plat->pmd_enabled &&
		    ch_idx >= pmd_start_chan && ch_idx < pmd_end_chan)
			continue;
		ch = &priv->channel[ch_idx];
		free_irq(ch->rx_irq, ch);
	}
rxirq_error:
	for (; chan > 0; chan--) {
		u32 ch_idx = chan - 1;

		if (priv->plat->pmd_enabled &&
		    ch_idx >= pmd_start_chan && ch_idx < pmd_end_chan)
			continue;
		ch = &priv->channel[ch_idx];
		free_irq(ch->rx_irq, ch);
	}
irq_error:
	return ret;
}

static void xgmac_e56_calib_timer(struct timer_list *t)
{
	struct xgmac_priv *priv = from_timer(priv, t, e56_calib_timer);

	xgbe_e56_calibration(priv, priv->plat->max_speed);
	mod_timer(&priv->e56_calib_timer, XGMAC_TIMER_MS_TICK(10000));
}

/**
 *  xgmac_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int xgmac_open(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct timespec64 now;
	int bfsize = 0;
	u32 dev_rx_queues, dev_tx_queues;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	int linkup;
	int ret;

	/* Configure real RX and TX queues */
	if (priv->plat->pmd_enabled) {
		dev_rx_queues = priv->plat->rx_chans_to_use -
					priv->plat->pmd_num_chans;
		dev_tx_queues = priv->plat->tx_chans_to_use -
					priv->plat->pmd_num_chans;
	} else {
		dev_rx_queues = priv->plat->rx_chans_to_use;
		dev_tx_queues = priv->plat->tx_chans_to_use;
	}
	netif_set_real_num_rx_queues(dev, dev_rx_queues);
	netif_set_real_num_tx_queues(dev, dev_tx_queues);

	/* Extra statistics */
	memset(&priv->xstats, 0, sizeof(struct xgmac_extra_stats));
	priv->xstats.threshold = tc;

	bfsize = xgmac_set_bfsize(dev->mtu, priv->dma_buf_sz);
	if (priv->plat->ecpri_en && bfsize > BUF_SIZE_9KiB)
		bfsize = BUF_SIZE_9KiB;
	priv->dma_buf_sz = bfsize;
	buf_sz = bfsize;

	priv->rx_copybreak = XGMAC_RX_COPYBREAK;

	/* eCPRI related config */
	if (priv->plat->ecpri_en)
		ecpri_setup_config(priv);

	/* Earlier check for TBS */
	for (chan = 0; chan < priv->plat->tx_chans_to_use; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
		int tbs_en = priv->plat->tx_chans_cfg[chan].tbs_en;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		tx_ch->tbs |= tbs_en ? XGMAC_TBS_AVAIL : 0;
		if (xgmac_enable_tbs(priv, priv->ioaddr, tbs_en, chan))
			tx_ch->tbs &= ~XGMAC_TBS_AVAIL;
	}

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors allocation failed\n",
			   __func__);
		goto dma_desc_error;
	}

	ret = init_dma_desc_rings(dev, GFP_KERNEL);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: DMA descriptors initialization failed\n",
			   __func__);
		goto init_error;
	}

	xgmac_init_coalesce(priv);

	ret = xgmac_hw_setup(dev, true);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Hw setup failed\n", __func__);
		goto init_error;
	}

	phylink_start(priv->phylink);

	ret = xgmac_xpcs_get_state(priv, priv->device, priv->xpcs_base,
				   &linkup);
	if ((ret < 0 && ret != -ENODEV) || !linkup) {
		netdev_err(priv->dev, "%s: XPCS link down\n", __func__);
	}

	ret = xgmac_request_irqs(dev);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Request IRQs failed\n", __func__);
		goto irq_error;
	}

	if (priv->plat->pmd_enabled) {
		priv->virt_chan_info->fastpath_state = VIRT_CHAN_RESET;
		priv->virt_chan_info->linux_state = VIRT_CHAN_INIT;
		priv->virt_chan_info->ifaceidx = dev->ifindex;
		hrtimer_init(&priv->virt_ch_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->virt_ch_timer.function = xgmac_virt_ch_timer;
		priv->virt_chan_rx_running = false;
		priv->virt_chan_tx_running = false;
	}

	xgmac_enable_all_chans(priv);
	xgmac_start_all_queues(priv);

	if (priv->plat->ecpri_en)
		ecpri_start(priv);

	if (priv->plat->eth_link == 2)
		ptp_priv_list[priv->plat->phy_lane] = priv;
	else
		ptp_priv_list[PTP_MAX_NUM_PORTS - 1] = priv;

	if (priv->plat->eth_link == 1) {
		timer_setup(&priv->e56_calib_timer, xgmac_e56_calib_timer, 0);
		xgbe_e56_calibration(priv, priv->plat->max_speed);
		mod_timer(&priv->e56_calib_timer, XGMAC_TIMER_MS_TICK(10000));
	}

	/* initialize system time */
	ktime_get_real_ts64(&now);
	/* lower 32 bits of tv_sec are safe until y2106 */
	xgmac_init_systime(priv, priv->ptpaddr, (u32)now.tv_sec, now.tv_nsec);

#ifdef USE_FLEXIBLE_PPS
	ret = init_flex_pps(priv);
	if (ret < 0) {
		netdev_err(priv->dev, "%s: Request PPS IRQs failed\n", __func__);
		goto irq_error;
	}
#endif

	return 0;

irq_error:
	phylink_stop(priv->phylink);

	for (chan = 0; chan < priv->plat->tx_chans_to_use; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		del_timer_sync(&priv->tx_chan[chan].txtimer);
	}

	xgmac_hw_teardown(dev);
init_error:
	free_dma_desc_resources(priv);
dma_desc_error:
	phylink_disconnect_phy(priv->phylink);
	return ret;
}

/**
 *  xgmac_release - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int xgmac_release(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	if (priv->plat->ecpri_en)
		ecpri_stop(priv);

	if (priv->eee_enabled)
		del_timer_sync(&priv->eee_ctrl_timer);

	/* Stop and disconnect the PHY */
	phylink_stop(priv->phylink);
	phylink_disconnect_phy(priv->phylink);

	xgmac_stop_all_queues(priv);

	xgmac_disable_all_chans(priv);

	for (chan = 0; chan < priv->plat->tx_chans_to_use; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		del_timer_sync(&priv->tx_chan[chan].txtimer);
	}

	if (priv->plat->virt_chan_en)
		hrtimer_cancel(&priv->virt_ch_timer);

#ifdef USE_FLEXIBLE_PPS
	stop_flex_pps(priv);
#endif

	/* Free the IRQ lines */
	free_irq(dev->irq, dev);
	if (priv->wol_irq != dev->irq)
		free_irq(priv->wol_irq, dev);
	if (priv->lpi_irq > 0)
		free_irq(priv->lpi_irq, dev);

	/* Stop TX/RX DMA and clear the descriptors */
	xgmac_stop_all_dma(priv);

	/* Release and free the Rx/Tx resources */
	free_dma_desc_resources(priv);

	/* Disable the MAC Rx/Tx */
	xgmac_mac_set(priv, priv->ioaddr, false);

	netif_carrier_off(dev);

	xgmac_release_ptp(priv);

	if (priv->plat->eth_link == 1)
		del_timer_sync(&priv->e56_calib_timer);

	return 0;
}

static bool xgmac_vlan_insert(struct xgmac_priv *priv, struct sk_buff *skb,
			       struct xgmac_tx_chan *tx_ch)
{
	u16 tag = 0x0, inner_tag = 0x0;
	u32 inner_type = 0x0;
	struct dma_desc *p;

	if (!priv->dma_cap.vlins)
		return false;
	if (!skb_vlan_tag_present(skb))
		return false;
	if (skb->vlan_proto == htons(ETH_P_8021AD)) {
		inner_tag = skb_vlan_tag_get(skb);
		inner_type = XGMAC_VLAN_INSERT;
	}

	tag = skb_vlan_tag_get(skb);

	if (tx_ch->tbs & XGMAC_TBS_AVAIL)
		p = &tx_ch->dma_entx[tx_ch->cur_tx].basic;
	else
		p = &tx_ch->dma_tx[tx_ch->cur_tx];

	if (xgmac_set_desc_vlan_tag(priv, p, tag, inner_tag, inner_type))
		return false;

	xgmac_set_tx_owner(priv, p);
	tx_ch->cur_tx = XGMAC_GET_ENTRY(tx_ch->cur_tx, DMA_TX_SIZE);
	return true;
}

/**
 *  xgmac_tso_allocator - close entry point of the driver
 *  @priv: driver private structure
 *  @des: buffer start address
 *  @total_len: total length to fill in descriptors
 *  @last_segmant: condition for the last descriptor
 *  @chan: TX chan index
 *  Description:
 *  This function fills descriptor and request new descriptors according to
 *  buffer length to fill
 */
static void xgmac_tso_allocator(struct xgmac_priv *priv, dma_addr_t des,
				 int total_len, bool last_segment, u32 chan)
{
	struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];
	struct dma_desc *desc;
	u32 buff_size;
	int tmp_len;

	tmp_len = total_len;

	while (tmp_len > 0) {
		dma_addr_t curr_addr;

		tx_ch->cur_tx = XGMAC_GET_ENTRY(tx_ch->cur_tx, DMA_TX_SIZE);
		WARN_ON(tx_ch->tx_skbuff[tx_ch->cur_tx]);

		if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			desc = &tx_ch->dma_entx[tx_ch->cur_tx].basic;
		else
			desc = &tx_ch->dma_tx[tx_ch->cur_tx];

		curr_addr = des + (total_len - tmp_len);
		if (priv->dma_cap.addr64 <= 32)
			desc->des0 = cpu_to_le32(curr_addr);
		else
			xgmac_set_desc_addr(priv, desc, curr_addr);

		buff_size = tmp_len >= TSO_MAX_BUFF_SIZE ?
			    TSO_MAX_BUFF_SIZE : tmp_len;

		xgmac_prepare_tso_tx_desc(priv, desc, 0, buff_size,
				0, 1,
				(last_segment) && (tmp_len <= TSO_MAX_BUFF_SIZE),
				0, 0);

		tmp_len -= TSO_MAX_BUFF_SIZE;
	}
}

/**
 *  xgmac_tso_xmit - Tx entry point of the driver for oversized frames (TSO)
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description: this is the transmit function that is called on TSO frames
 *  (support available on GMAC4 and newer chips).
 *  Diagram below show the ring programming in case of TSO frames:
 *
 *  First Descriptor
 *   --------
 *   | DES0 |---> buffer1 = L2/L3/L4 header
 *   | DES1 |---> TCP Payload (can continue on next descr...)
 *   | DES2 |---> buffer 1 and 2 len
 *   | DES3 |---> must set TSE, TCP hdr len-> [22:19]. TCP payload len [17:0]
 *   --------
 *	|
 *     ...
 *	|
 *   --------
 *   | DES0 | --| Split TCP Payload on Buffers 1 and 2
 *   | DES1 | --|
 *   | DES2 | --> buffer 1 and 2 len
 *   | DES3 |
 *   --------
 *
 * mss is fixed when enable tso, so w/o programming the TDES3 ctx field.
 */
static netdev_tx_t xgmac_tso_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dma_desc *desc, *first, *mss_desc = NULL;
	struct xgmac_priv *priv = netdev_priv(dev);
	int desc_size, tmp_pay_len = 0, first_tx;
	int nfrags = skb_shinfo(skb)->nr_frags;
	u32 chan = skb_get_queue_mapping(skb);
	unsigned int first_entry, tx_packets;
	struct xgmac_tx_chan *tx_ch;
	bool has_vlan, set_ic;
	u8 proto_hdr_len, hdr;
	u32 pay_len, mss;
	dma_addr_t des;
	int i;

	tx_ch = &priv->tx_chan[chan];
	first_tx = tx_ch->cur_tx;

	/* Compute header lengths */
	if (skb_shinfo(skb)->gso_type & SKB_GSO_UDP_L4) {
		proto_hdr_len = skb_transport_offset(skb) + sizeof(struct udphdr);
		hdr = sizeof(struct udphdr);
	} else {
		proto_hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		hdr = tcp_hdrlen(skb);
	}

	/* Desc availability based on threshold should be enough safe */
	if (unlikely(xgmac_tx_avail(priv, chan) <
		(((skb->len - proto_hdr_len) / TSO_MAX_BUFF_SIZE + 1)))) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, chan))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								chan));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

	pay_len = skb_headlen(skb) - proto_hdr_len; /* no frags */

	mss = skb_shinfo(skb)->gso_size;

	/* set new MSS value if needed */
	if (mss != tx_ch->mss) {
		if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			mss_desc = &tx_ch->dma_entx[tx_ch->cur_tx].basic;
		else
			mss_desc = &tx_ch->dma_tx[tx_ch->cur_tx];

		xgmac_set_mss(priv, mss_desc, mss);
		tx_ch->mss = mss;
		tx_ch->cur_tx = XGMAC_GET_ENTRY(tx_ch->cur_tx, DMA_TX_SIZE);
		WARN_ON(tx_ch->tx_skbuff[tx_ch->cur_tx]);
	}

	if (netif_msg_tx_queued(priv)) {
		pr_info("%s: hdrlen %d, hdr_len %d, pay_len %d, mss %d\n",
			__func__, hdr, proto_hdr_len, pay_len, mss);
		pr_info("\tskb->len %d, skb->data_len %d\n", skb->len,
			skb->data_len);
	}

	/* Check if VLAN can be inserted by HW */
	has_vlan = xgmac_vlan_insert(priv, skb, tx_ch);

	first_entry = tx_ch->cur_tx;
	WARN_ON(tx_ch->tx_skbuff[first_entry]);

	if (tx_ch->tbs & XGMAC_TBS_AVAIL)
		desc = &tx_ch->dma_entx[first_entry].basic;
	else
		desc = &tx_ch->dma_tx[first_entry];
	first = desc;

	if (has_vlan)
		xgmac_set_desc_vlan(priv, first, XGMAC_VLAN_INSERT);

	/* first descriptor: fill Headers on Buf1 */
	des = dma_map_single(priv->device, skb->data, skb_headlen(skb),
			     DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, des))
		goto dma_map_err;

	tx_ch->tx_skbuff_dma[first_entry].buf = des;
	tx_ch->tx_skbuff_dma[first_entry].len = skb_headlen(skb);

	if (priv->dma_cap.addr64 <= 32) {
		first->des0 = cpu_to_le32(des);

		/* Fill start of payload in buff2 of first descriptor */
		if (pay_len)
			first->des1 = cpu_to_le32(des + proto_hdr_len);

		/* If needed take extra descriptors to fill the remaining payload */
		tmp_pay_len = pay_len - TSO_MAX_BUFF_SIZE;
	} else {
		xgmac_set_desc_addr(priv, first, des);
		tmp_pay_len = pay_len;
		des += proto_hdr_len;
		pay_len = 0;
	}

	xgmac_tso_allocator(priv, des, tmp_pay_len, (nfrags == 0), chan);

	/* Prepare fragments */
	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		des = skb_frag_dma_map(priv->device, frag, 0,
				       skb_frag_size(frag),
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		xgmac_tso_allocator(priv, des, skb_frag_size(frag),
				     (i == nfrags - 1), chan);

		tx_ch->tx_skbuff_dma[tx_ch->cur_tx].buf = des;
		tx_ch->tx_skbuff_dma[tx_ch->cur_tx].len = skb_frag_size(frag);
		tx_ch->tx_skbuff_dma[tx_ch->cur_tx].map_as_page = true;
	}

	tx_ch->tx_skbuff_dma[tx_ch->cur_tx].last_segment = true;

	/* Only the last descriptor gets to point to the skb. */
	tx_ch->tx_skbuff[tx_ch->cur_tx] = skb;

	/* Manage tx mitigation */
	tx_packets = (tx_ch->cur_tx + 1) - first_tx;
	tx_ch->tx_count_frames += tx_packets;

	if ((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) && priv->hwts_tx_en)
		set_ic = true;
	else if (!priv->tx_coal_frames)
		set_ic = false;
	else if (tx_packets > priv->tx_coal_frames)
		set_ic = true;
	else if ((tx_ch->tx_count_frames % priv->tx_coal_frames) < tx_packets)
		set_ic = true;
	else
		set_ic = false;

	if (set_ic) {
		if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			desc = &tx_ch->dma_entx[tx_ch->cur_tx].basic;
		else
			desc = &tx_ch->dma_tx[tx_ch->cur_tx];

		tx_ch->tx_count_frames = 0;
		xgmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and xgmac_tx_clean may clean up to this descriptor.
	 */
	tx_ch->cur_tx = XGMAC_GET_ENTRY(tx_ch->cur_tx, DMA_TX_SIZE);

	if (unlikely(xgmac_tx_avail(priv, chan) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, chan));
	}

	dev->stats.tx_bytes += skb->len;
	priv->xstats.tx_tso_frames++;
	priv->xstats.tx_tso_nfrags += nfrags;

	if (priv->sarc_type)
		xgmac_set_desc_sarc(priv, first, priv->sarc_type);

	skb_tx_timestamp(skb);

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->hwts_tx_en)) {
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		xgmac_enable_tx_timestamp(priv, first);
	}

	/* Complete the first descriptor before granting the DMA */
	xgmac_prepare_tso_tx_desc(priv, first, 1,
			proto_hdr_len,
			pay_len,
			1, tx_ch->tx_skbuff_dma[first_entry].last_segment,
			hdr / 4, (skb->len - proto_hdr_len));

	/* If context desc is used to change MSS */
	if (mss_desc) {
		/* Make sure that first descriptor has been completely
		 * written, including its own bit. This is because MSS is
		 * actually before first descriptor, so we need to make
		 * sure that MSS's own bit is the last thing written.
		 */
		dma_wmb();
		xgmac_set_tx_owner(priv, mss_desc);
	}

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	if (netif_msg_pktdata(priv)) {
		pr_info("%s: curr=%d dirty=%d f=%d, e=%d, f_p=%p, nfrags %d\n",
			__func__, tx_ch->cur_tx, tx_ch->dirty_tx, first_entry,
			tx_ch->cur_tx, first, nfrags);
		pr_info(">>> frame to be transmitted: ");
		print_pkt(skb->data, skb_headlen(skb));
	}

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, chan), skb->len);

	if (tx_ch->tbs & XGMAC_TBS_AVAIL)
		desc_size = sizeof(struct dma_edesc);
	else
		desc_size = sizeof(struct dma_desc);

	tx_ch->tx_tail_addr = tx_ch->dma_tx_phy + (tx_ch->cur_tx * desc_size);
	xgmac_set_tx_tail_ptr(priv, priv->ioaddr, tx_ch->tx_tail_addr, chan);
	xgmac_tx_timer_arm(priv, chan);

	return NETDEV_TX_OK;

dma_map_err:
	dev_err(priv->device, "Tx dma map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static int hw_timestamp_needed(struct sk_buff *skb, struct xgmac_priv *priv)
{
	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     priv->hwts_tx_en))
		return 1;
	else
		return 0;
}

static void virt_chan_tx_copy(struct dma_desc *p, dma_addr_t src_addr, u32 len)
{
	void *src = phys_to_virt(src_addr);
	void *dst;
	phys_addr_t dst_addr;

	dst_addr = ((u64)le32_to_cpu(p->des1) << 32) | le32_to_cpu(p->des0);
	dst = phys_to_virt(dst_addr);
	memcpy(dst, src, len);
}

static netdev_tx_t xgmac_virt_chan_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int first_entry, tx_packets;
	struct xgmac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i, csum_insertion = 0, is_jumbo = 0;
	u32 dev_queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry, first_tx;
	struct dma_desc *desc, *first;
	struct xgmac_tx_chan *tx_ch;
	bool has_vlan, set_ic;
	dma_addr_t des;
	u32 chan = priv->plat->tx_chans_to_use;
	static u32 ring_full_err_cnt;

	priv->virt_chan_tx_running = true;

	tx_ch = &priv->tx_chan[chan];
	first_tx = tx_ch->cur_tx;

	if (priv->tx_path_in_lpi_mode)
		xgmac_disable_eee_mode(priv);

	if (unlikely(xgmac_tx_avail(priv, chan) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, dev_queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								dev_queue));
			/* Log error once */
			if ((ring_full_err_cnt % 100) == 0) {
				netdev_err(priv->dev,
					   "%s: Tx Ring full when queue awake\n",
					   __func__);
				ring_full_err_cnt++;
			}
		}
		priv->virt_chan_tx_running = false;
		return NETDEV_TX_BUSY;
	}
	ring_full_err_cnt = 0;

	/* Check if VLAN can be inserted by HW */
	has_vlan = xgmac_vlan_insert(priv, skb, tx_ch);

	entry = tx_ch->cur_tx;
	first_entry = entry;
	WARN_ON(tx_ch->tx_skbuff[first_entry]);

	csum_insertion = (skb->ip_summed == CHECKSUM_PARTIAL);

	desc = tx_ch->dma_tx + entry;
	first = desc;

	if (has_vlan)
		xgmac_set_desc_vlan(priv, first, XGMAC_VLAN_INSERT);

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = XGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		WARN_ON(tx_ch->tx_skbuff[entry]);
		desc = tx_ch->dma_tx + entry;

		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err; /* should reuse desc w/o issues */

		virt_chan_tx_copy(desc, des, len);

		tx_ch->tx_skbuff_dma[entry].buf = des;
		tx_ch->tx_skbuff_dma[entry].map_as_page = true;
		tx_ch->tx_skbuff_dma[entry].len = len;
		tx_ch->tx_skbuff_dma[entry].last_segment = last_segment;

		/* Prepare the descriptor and set the own bit too */
		xgmac_prepare_tx_desc(priv, desc, 0, len, csum_insertion,
				      priv->mode, 1, last_segment, skb->len);
	}

	/* Only the last descriptor gets to point to the skb. */
	tx_ch->tx_skbuff[entry] = skb;

	/* According to the coalesce parameter the IC bit for the latest
	 * segment is reset and the timer re-started to clean the tx status.
	 * This approach takes care about the fragments: desc is the first
	 * element in case of no SG.
	 */
	tx_packets = (entry + 1) - first_tx;
	tx_ch->tx_count_frames += tx_packets;

	if ((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) && priv->hwts_tx_en)
		set_ic = true;
	else if (!priv->tx_coal_frames)
		set_ic = false;
	else if (tx_packets > priv->tx_coal_frames)
		set_ic = true;
	else if ((tx_ch->tx_count_frames % priv->tx_coal_frames) < tx_packets)
		set_ic = true;
	else
		set_ic = false;

	if (set_ic) {
		desc = &tx_ch->dma_tx[entry];
		tx_ch->tx_count_frames = 0;
		xgmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and xgmac_tx_clean may clean up to this descriptor.
	 */
	entry = XGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_ch->cur_tx = entry;

	if (netif_msg_pktdata(priv)) {
		netdev_dbg(priv->dev,
			   "%s: curr=%d dirty=%d f=%d, e=%d, first=%p, nfrags=%d",
			   __func__, tx_ch->cur_tx, tx_ch->dirty_tx, first_entry,
			   entry, first, nfrags);

		netdev_dbg(priv->dev, ">>> frame to be transmitted: ");
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(xgmac_tx_avail(priv, chan) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, dev_queue));
	}

	dev->stats.tx_bytes += skb->len;
	dev->stats.tx_packets += 1;

	if (priv->sarc_type)
		xgmac_set_desc_sarc(priv, first, priv->sarc_type);

	skb_tx_timestamp(skb);

	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
	if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		virt_chan_tx_copy(first, des, nopaged_len);

		tx_ch->tx_skbuff_dma[first_entry].buf = des;
		tx_ch->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_ch->tx_skbuff_dma[first_entry].last_segment = last_segment;

		/* Prepare the first descriptor setting the OWN bit too */
		xgmac_prepare_tx_desc(priv, first, 1, nopaged_len,
				      csum_insertion, priv->mode, 0,
				      last_segment, skb->len);
	}

	xgmac_set_tx_owner(priv, first);

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, dev_queue), skb->len);

	netdev_tx_completed_queue(netdev_get_tx_queue(dev, dev_queue), 1,
				  skb->len);

	priv->virt_chan_tx_running = false;

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

/**
 *  xgmac_xmit - Tx entry point of the driver
 *  @skb : the socket buffer
 *  @dev : device pointer
 *  Description : this is the tx entry point of the driver.
 *  It programs the chain or the ring and supports oversized frames
 *  and SG feature.
 */
static netdev_tx_t xgmac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned int first_entry, tx_packets, enh_desc;
	struct xgmac_priv *priv = netdev_priv(dev);
	unsigned int nopaged_len = skb_headlen(skb);
	int i, csum_insertion = 0, is_jumbo = 0;
	u32 dev_queue = skb_get_queue_mapping(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int gso = skb_shinfo(skb)->gso_type;
	struct dma_edesc *tbs_desc = NULL;
	int entry, desc_size, first_tx;
	struct dma_desc *desc, *first;
	struct xgmac_tx_chan *tx_ch;
	bool has_vlan, set_ic;
	dma_addr_t des;
	u32 num_ecpri_ch;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan = dev_queue;
	u32 tx_count = priv->plat->tx_chans_to_use;
	unsigned long stmr_val = 0;

	/* Should be no Tx packet to eCPRI chan from stack */
	if (priv->plat->ecpri_en) {
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_ecpri_ch = 1;
		if (chan < num_ecpri_ch)
			chan = num_ecpri_ch;
	}

	/* If PMD is enabled, re-route pmd chan traffic to next chan */
	if (priv->plat->pmd_enabled &&
	    chan >= pmd_start_chan && chan < pmd_end_chan)
		chan = pmd_end_chan;
	if (virt_chan_avail(priv)) {
		/* Packets not requiring timestamp are send to virtual chan */
		if (!hw_timestamp_needed(skb, priv))
			return xgmac_virt_chan_xmit(skb, dev);

		tx_count += 1;
	}
	if (chan >= tx_count) {
		WARN_ON_ONCE(chan);
		dev_kfree_skb(skb);
		priv->dev->stats.tx_dropped++;
		return NET_XMIT_DROP;
	}

	tx_ch = &priv->tx_chan[chan];
	first_tx = tx_ch->cur_tx;

	if (priv->tx_path_in_lpi_mode)
		xgmac_disable_eee_mode(priv);

	/* Manage oversized TCP frames for GMAC4 device */
	if (skb_is_gso(skb) && priv->tso) {
		if (gso & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6))
			return xgmac_tso_xmit(skb, dev);
	}

	if (unlikely(xgmac_tx_avail(priv, chan) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(dev, dev_queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev,
								dev_queue));
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx Ring full when queue awake\n",
				   __func__);
		}
		return NETDEV_TX_BUSY;
	}

	/* Check if VLAN can be inserted by HW */
	has_vlan = xgmac_vlan_insert(priv, skb, tx_ch);

	entry = tx_ch->cur_tx;
	first_entry = entry;
	WARN_ON(tx_ch->tx_skbuff[first_entry]);

	csum_insertion = (skb->ip_summed == CHECKSUM_PARTIAL);

	if (likely(priv->extend_desc))
		desc = (struct dma_desc *)(tx_ch->dma_etx + entry);
	else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
		desc = &tx_ch->dma_entx[entry].basic;
	else
		desc = tx_ch->dma_tx + entry;

	first = desc;

	if (has_vlan)
		xgmac_set_desc_vlan(priv, first, XGMAC_VLAN_INSERT);

	enh_desc = priv->plat->enh_desc;
	/* To program the descriptors according to the size of the frame */
	if (enh_desc)
		is_jumbo = xgmac_is_jumbo_frm(priv, skb->len, enh_desc);

	if (unlikely(is_jumbo)) {
		entry = xgmac_jumbo_frm(priv, tx_ch, skb, csum_insertion);
		if (unlikely(entry < 0) && (entry != -EINVAL))
			goto dma_map_err;
	}

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = XGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		WARN_ON(tx_ch->tx_skbuff[entry]);

		if (likely(priv->extend_desc))
			desc = (struct dma_desc *)(tx_ch->dma_etx + entry);
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			desc = &tx_ch->dma_entx[entry].basic;
		else
			desc = tx_ch->dma_tx + entry;

		des = skb_frag_dma_map(priv->device, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err; /* should reuse desc w/o issues */

		tx_ch->tx_skbuff_dma[entry].buf = des;

		xgmac_set_desc_addr(priv, desc, des);

		tx_ch->tx_skbuff_dma[entry].map_as_page = true;
		tx_ch->tx_skbuff_dma[entry].len = len;
		tx_ch->tx_skbuff_dma[entry].last_segment = last_segment;

		/* Prepare the descriptor and set the own bit too */
		xgmac_prepare_tx_desc(priv, desc, 0, len, csum_insertion,
				priv->mode, 1, last_segment, skb->len);
	}

	/* Only the last descriptor gets to point to the skb. */
	tx_ch->tx_skbuff[entry] = skb;

	/* According to the coalesce parameter the IC bit for the latest
	 * segment is reset and the timer re-started to clean the tx status.
	 * This approach takes care about the fragments: desc is the first
	 * element in case of no SG.
	 */
	tx_packets = (entry + 1) - first_tx;
	tx_ch->tx_count_frames += tx_packets;

	if ((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) && priv->hwts_tx_en)
		set_ic = true;
	else if (!priv->tx_coal_frames)
		set_ic = false;
	else if (tx_packets > priv->tx_coal_frames)
		set_ic = true;
	else if ((tx_ch->tx_count_frames % priv->tx_coal_frames) < tx_packets)
		set_ic = true;
	else
		set_ic = false;

	if (set_ic) {
		if (likely(priv->extend_desc))
			desc = &tx_ch->dma_etx[entry].basic;
		else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
			desc = &tx_ch->dma_entx[entry].basic;
		else
			desc = &tx_ch->dma_tx[entry];

		tx_ch->tx_count_frames = 0;
		xgmac_set_tx_ic(priv, desc);
		priv->xstats.tx_set_ic_bit++;
	}

	/* We've used all descriptors we need for this skb, however,
	 * advance cur_tx so that it references a fresh descriptor.
	 * ndo_start_xmit will fill this descriptor the next time it's
	 * called and xgmac_tx_clean may clean up to this descriptor.
	 */
	entry = XGMAC_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_ch->cur_tx = entry;

	if (netif_msg_pktdata(priv)) {
		netdev_dbg(priv->dev,
			   "%s: curr=%d dirty=%d f=%d, e=%d, first=%p, nfrags=%d",
			   __func__, tx_ch->cur_tx, tx_ch->dirty_tx, first_entry,
			   entry, first, nfrags);

		netdev_dbg(priv->dev, ">>> frame to be transmitted: ");
		print_pkt(skb->data, skb->len);
	}

	if (unlikely(xgmac_tx_avail(priv, chan) <= (MAX_SKB_FRAGS + 1))) {
		netif_dbg(priv, hw, priv->dev, "%s: stop transmitted packets\n",
			  __func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, dev_queue));
	}

	dev->stats.tx_bytes += skb->len;

	if (priv->sarc_type)
		xgmac_set_desc_sarc(priv, first, priv->sarc_type);

	skb_tx_timestamp(skb);

	/* Ready to fill the first descriptor and set the OWN bit w/o any
	 * problems because all the descriptors are actually ready to be
	 * passed to the DMA engine.
	 */
	if (likely(!is_jumbo)) {
		bool last_segment = (nfrags == 0);

		des = dma_map_single(priv->device, skb->data,
				     nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tx_ch->tx_skbuff_dma[first_entry].buf = des;

		xgmac_set_desc_addr(priv, first, des);

		tx_ch->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_ch->tx_skbuff_dma[first_entry].last_segment = last_segment;

		if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
			     priv->hwts_tx_en)) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			xgmac_enable_tx_timestamp(priv, first);
		}

		/* Prepare the first descriptor setting the OWN bit too */
		xgmac_prepare_tx_desc(priv, first, 1, nopaged_len,
				csum_insertion, priv->mode, 0, last_segment,
				skb->len);
	}

	if (priv->profiling_tx) {
		struct iphdr *ip_header = NULL;

		/* Ensure the skb has an Ethernet header */
		if (skb->len >= sizeof(struct ethhdr)) {
			struct ethhdr *eth = eth_hdr(skb);

			if (eth->h_proto == htons(ETH_P_IP))
				ip_header = ip_hdr(skb);
		}

		if (ip_header) {
			u16 frag_off = ntohs(ip_header->frag_off);

			/* Check if this is the first fragment */
			if ((frag_off & IP_MF) && !(frag_off & IP_OFFSET) &&
			    pf_tx_first_seg_idx < MAX_PROFILE_PKT) {
				asm volatile("mrs %0, cntvct_el0" : "=r" (stmr_val));
				pf_tx_data[pf_tx_first_seg_idx].first_seg_stmr = stmr_val;
				pf_tx_first_seg_idx++;
			}

			/* Check if this is the last fragment */
			if (!(frag_off & IP_MF) && (frag_off & IP_OFFSET) &&
			    pf_tx_last_seg_idx < MAX_PROFILE_PKT) {
				asm volatile("mrs %0, cntvct_el0" : "=r" (stmr_val));
				pf_tx_data[pf_tx_last_seg_idx].last_seg_stmr = stmr_val;
				pf_tx_last_seg_idx++;
			}
		}
	}

	if (tx_ch->tbs & XGMAC_TBS_EN) {
		struct timespec64 ts = ns_to_timespec64(skb->tstamp);

		tbs_desc = &tx_ch->dma_entx[first_entry];
		xgmac_set_desc_tbs(priv, tbs_desc, ts.tv_sec, ts.tv_nsec);
	}

	xgmac_set_tx_owner(priv, first);

	/* The own bit must be the latest setting done when prepare the
	 * descriptor and then barrier is needed to make sure that
	 * all is coherent before granting the DMA engine.
	 */
	wmb();

	netdev_tx_sent_queue(netdev_get_tx_queue(dev, dev_queue), skb->len);

	xgmac_enable_dma_transmission(priv, priv->ioaddr);

	if (likely(priv->extend_desc))
		desc_size = sizeof(struct dma_extended_desc);
	else if (tx_ch->tbs & XGMAC_TBS_AVAIL)
		desc_size = sizeof(struct dma_edesc);
	else
		desc_size = sizeof(struct dma_desc);

	tx_ch->tx_tail_addr = tx_ch->dma_tx_phy + (tx_ch->cur_tx * desc_size);
	xgmac_set_tx_tail_ptr(priv, priv->ioaddr, tx_ch->tx_tail_addr, chan);
	xgmac_tx_timer_arm(priv, chan);

	return NETDEV_TX_OK;

dma_map_err:
	netdev_err(priv->dev, "Tx DMA map failed\n");
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

void xgmac_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct vlan_ethhdr *veth;
	__be16 vlan_proto;
	u16 vlanid;

	veth = (struct vlan_ethhdr *)skb->data;
	vlan_proto = veth->h_vlan_proto;

	if ((vlan_proto == htons(ETH_P_8021Q) &&
	     dev->features & NETIF_F_HW_VLAN_CTAG_RX) ||
	    (vlan_proto == htons(ETH_P_8021AD) &&
	     dev->features & NETIF_F_HW_VLAN_STAG_RX)) {
		/* pop the vlan tag */
		vlanid = ntohs(veth->h_vlan_TCI);
		memmove(skb->data + VLAN_HLEN, veth, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, vlan_proto, vlanid);
	}
}


static inline int xgmac_rx_threshold_count(struct xgmac_rx_chan *rx_ch)
{
	if (rx_ch->rx_zeroc_thresh < XGMAC_RX_THRESH)
		return 0;

	return 1;
}

/**
 * xgmac_rx_refill - refill used skb preallocated buffers
 * @priv: driver private structure
 * @chan: RX chan index
 * Description : this is to reallocate the skb for the reception process
 * that is based on zero-copy.
 */
static inline void xgmac_rx_refill(struct xgmac_priv *priv, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	int len, dirty = xgmac_rx_dirty(priv, chan);
	unsigned int entry = rx_ch->dirty_rx;

	len = DIV_ROUND_UP(priv->dma_buf_sz, PAGE_SIZE) * PAGE_SIZE;

	while (dirty-- > 0) {
		struct xgmac_rx_buffer *buf = &rx_ch->buf_pool[entry];
		struct dma_desc *p;
		bool use_rx_wd;

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_ch->dma_erx + entry);
		else
			p = rx_ch->dma_rx + entry;

		if (!buf->page) {
			buf->page = page_pool_dev_alloc_pages(rx_ch->page_pool);
			if (!buf->page)
				break;
		}

		if (priv->sph && !buf->sec_page) {
			buf->sec_page = page_pool_dev_alloc_pages(rx_ch->page_pool);
			if (!buf->sec_page)
				break;

			buf->sec_addr = page_pool_get_dma_addr(buf->sec_page);

			dma_sync_single_for_device(priv->device, buf->sec_addr,
						   len, DMA_FROM_DEVICE);
		}

		buf->addr = page_pool_get_dma_addr(buf->page);

		/* Sync whole allocation to device. This will invalidate old
		 * data.
		 */
		dma_sync_single_for_device(priv->device, buf->addr, len,
					   DMA_FROM_DEVICE);

		xgmac_set_desc_addr(priv, p, buf->addr);
		xgmac_set_desc_sec_addr(priv, p, buf->sec_addr);
		xgmac_refill_desc3(priv, rx_ch, p);

#if 0 // Use HW coalesce
		rx_ch->rx_count_frames++;
		rx_ch->rx_count_frames += priv->rx_coal_frames;
		if (rx_ch->rx_count_frames > priv->rx_coal_frames)
			rx_ch->rx_count_frames = 0;

		use_rx_wd = !priv->rx_coal_frames;
		use_rx_wd |= rx_ch->rx_count_frames > 0;
		if (!priv->use_riwt)
			use_rx_wd = false;
#else
		use_rx_wd = priv->use_riwt;
#endif

		dma_wmb();
		xgmac_set_rx_owner(priv, p, use_rx_wd);

		entry = XGMAC_GET_ENTRY(entry, DMA_RX_SIZE);
	}
	rx_ch->dirty_rx = entry;
	/* Disable update of tail pointer for RPF SW workaround */
	if (!priv->plat->linux_rpf_en) {
		rx_ch->rx_tail_addr = rx_ch->dma_rx_phy +
				    (rx_ch->dirty_rx * sizeof(struct dma_desc));
		xgmac_set_rx_tail_ptr(priv, priv->ioaddr, rx_ch->rx_tail_addr, chan);
	} else {
		/* Check if DMA is stopped and start it if required */
		if (priv->channel[chan].rx_dma_stopped) {
			xgmac_start_rx_dma(priv, chan);

			/* Clear the DMA stopped flag */
			priv->channel[chan].rx_dma_stopped = false;
		}
	}
}

static unsigned int xgmac_rx_buf1_len(struct xgmac_priv *priv,
			        struct dma_desc *p,
			        int status, unsigned int len)
{
	int ret, coe = priv->hw->rx_csum;
	unsigned int plen = 0, hlen = 0;

	/* Not first descriptor, buffer is always zero */
	if (priv->sph && len)
		return 0;

	/* First descriptor, get split header length */
	ret = xgmac_get_rx_header_len(priv, p, &hlen);
	if (priv->sph && hlen) {
		priv->xstats.rx_split_hdr_pkt_n++;
		return hlen;
	}

	/* First descriptor, not last descriptor and not split header */
	if (status & rx_not_ls)
		return priv->dma_buf_sz;

	plen = xgmac_get_rx_frame_len(priv, p, coe);

	/* First descriptor and last descriptor and not split header */
	return min_t(unsigned int, priv->dma_buf_sz, plen);
}

static unsigned int xgmac_rx_buf2_len(struct xgmac_priv *priv,
				       struct dma_desc *p,
				       int status, unsigned int len)
{
	int coe = priv->hw->rx_csum;
	unsigned int plen = 0;

	/* Not split header, buffer is not available */
	if (!priv->sph)
		return 0;

	/* Not last descriptor */
	if (status & rx_not_ls)
		return priv->dma_buf_sz;

	plen = xgmac_get_rx_frame_len(priv, p, coe);

	/* Last descriptor */
	return plen - len;
}

void print_latency_profile(struct xgmac_priv *priv, int rxtx)
{
	struct profile_data *pf_data;
	unsigned long min_diff, max_diff, sum_diff, diff_stmr;
	unsigned int pf_idx, max_diff_idx, min_diff_idx;
	unsigned long sum_diff_last_seg = 0;
	unsigned long max_diff_last_seg = 0, min_diff_last_seg = STMR_2000US;
	unsigned long last_seg_diff;
	unsigned long last_seg_stmr;
	int st_0, st_1, st_5, st_10, st_15, st_20;
	int i, max_last_seg_idx = 0;

	if (rxtx == PROFILE_RX) {
		pf_data = pf_rx_data;
		pf_idx = pf_rx_last_seg_idx;
	} else {
		pf_data = pf_tx_data;
		pf_idx = pf_tx_last_seg_idx;
	}

	st_0 = 0;
	st_1 = 0;
	st_5 = 0;
	st_10 = 0;
	st_15 = 0;
	st_20 = 0;

	last_seg_stmr = pf_data[0].last_seg_stmr;
	if (pf_data[0].last_seg_stmr >= pf_data[0].first_seg_stmr)
		diff_stmr = pf_data[0].last_seg_stmr -
			    pf_data[0].first_seg_stmr;
	else
		diff_stmr = pf_data[0].last_seg_stmr + 0x100000000 -
			    pf_data[0].first_seg_stmr;
	max_diff = diff_stmr;
	min_diff = diff_stmr;
	max_diff_idx = 0;
	min_diff_idx = 0;
	sum_diff = diff_stmr;

	for (i = 1; i < pf_idx; i++) {
		if (pf_data[i].last_seg_stmr >= pf_data[i].first_seg_stmr)
			diff_stmr = pf_data[i].last_seg_stmr -
				    pf_data[i].first_seg_stmr;
		else
			diff_stmr = pf_data[i].last_seg_stmr + 0x100000000 -
				    pf_data[i].first_seg_stmr;
		if (diff_stmr < min_diff) {
			min_diff = diff_stmr;
			min_diff_idx = i;
		}
		if (diff_stmr > max_diff) {
			max_diff = diff_stmr;
			max_diff_idx = i;
		}
		sum_diff += diff_stmr;

		if (pf_data[i].last_seg_stmr >= last_seg_stmr)
			last_seg_diff = pf_data[i].last_seg_stmr -
					last_seg_stmr;
		else
			last_seg_diff = pf_data[i].last_seg_stmr +
					0x100000000 - last_seg_stmr;

		if (last_seg_diff < STMR_100US)
			st_0++;
		else if (last_seg_diff >= STMR_100US && last_seg_diff < STMR_500US)
			st_1++;
		else if (last_seg_diff >= STMR_500US && last_seg_diff < STMR_1000US)
			st_5++;
		else if (last_seg_diff >= STMR_1000US && last_seg_diff < STMR_1500US)
			st_10++;
		else if (last_seg_diff >= STMR_1500US && last_seg_diff < STMR_2000US)
			st_15++;
		else
			st_20++;

		if (last_seg_diff > max_diff_last_seg) {
			max_diff_last_seg = last_seg_diff;
			max_last_seg_idx = i;
		}

		if (last_seg_diff < min_diff_last_seg)
			min_diff_last_seg = last_seg_diff;

		last_seg_stmr = pf_data[i].last_seg_stmr;
		sum_diff_last_seg += last_seg_diff;
	}

	netdev_info(priv->dev, "#####################################################\n");
	netdev_info(priv->dev, "Total pkts %u max_diff_last_seg %lu at idx %u\n",
		    pf_idx, max_diff_last_seg, max_last_seg_idx);
	netdev_info(priv->dev, "min_diff_last_seg %lu avg_diff_last_seg %lu\n",
		    min_diff_last_seg, sum_diff_last_seg / (pf_idx - 1));
	netdev_info(priv->dev, "st_0 %u st_1 %u st_5 %u st_10 %u st_15 %u st_20 %u\n",
		    st_0, st_1, st_5, st_10, st_15, st_20);
	netdev_info(priv->dev, "max_diff %lu max_diff_idx %u min_diff %lu\n",
		    max_diff, max_diff_idx, min_diff);
	netdev_info(priv->dev, "min_diff_idx %u avg_diff %lu\n",
		    min_diff_idx, sum_diff / pf_idx);
	if (rxtx == PROFILE_RX && !priv->poll_mode) {
		netdev_info(priv->dev, "min_napi_rx_intv %lu max_napi_rx_intv %lu\n",
			    min_napi_rx_intv, max_napi_rx_intv);
		netdev_info(priv->dev, "num_napi_rx_call %lu avg_napi_rx_intv %lu\n",
			    num_napi_rx_call, sum_napi_rx_intv / num_napi_rx_call);
	}
	netdev_info(priv->dev, "#####################################################\n");

	if (rxtx == PROFILE_RX) {
		pf_rx_first_seg_idx = 0;
		pf_rx_last_seg_idx = 0;
		pf_rx_data[0].first_seg_stmr = 0;
		pf_rx_data[0].last_seg_stmr = 0;
		napi_rx_prev_val = 0;
		max_napi_rx_intv = 0;
		min_napi_rx_intv = STMR_2000US;
		num_napi_rx_call = 0;
		sum_napi_rx_intv = 0;
		cal_napi_rx_intv = false;
	} else {
		pf_tx_first_seg_idx = 0;
		pf_tx_last_seg_idx = 0;
		pf_tx_data[0].first_seg_stmr = 0;
		pf_tx_data[0].last_seg_stmr = 0;
	}
	max_diff = 0;
	min_diff = 0;
	max_diff_idx = 0;
	min_diff_idx = 0;
	sum_diff = 0;
}

/**
 * xgmac_rx - manage the receive process
 * @priv: driver private structure
 * @limit: napi bugget
 * @chan: RX chan index.
 * Description :  this the function called by the napi poll method.
 * It gets all the frames inside the ring.
 */
static int xgmac_rx(struct xgmac_priv *priv, int limit, u32 chan)
{
	struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];
	struct xgmac_channel *ch = &priv->channel[chan];
	unsigned int count = 0, error = 0, len = 0;
	int status = 0, coe = priv->hw->rx_csum;
	unsigned int next_entry = rx_ch->cur_rx;
	struct sk_buff *skb = NULL;
	u32 dev_queue = chan;
	unsigned long stmr_val = 0;

	if (priv->plat->pmd_enabled && chan >= priv->plat->pmd_start_chan)
		dev_queue = 0;

	if (netif_msg_rx_status(priv)) {
		void *rx_head;

		netdev_dbg(priv->dev, "%s: descriptor ring:\n", __func__);
		if (priv->extend_desc)
			rx_head = (void *)rx_ch->dma_erx;
		else
			rx_head = (void *)rx_ch->dma_rx;

		xgmac_display_ring(priv, rx_head, DMA_RX_SIZE, true);
	}
	while (count < limit) {
		unsigned int buf1_len = 0, buf2_len = 0;
		enum pkt_hash_types hash_type;
		struct xgmac_rx_buffer *buf;
		struct dma_desc *np, *p;
		int entry;
		u32 hash;

		if (!count && rx_ch->state_saved) {
			skb = rx_ch->state.skb;
			error = rx_ch->state.error;
			len = rx_ch->state.len;
		} else {
			rx_ch->state_saved = false;
			skb = NULL;
			error = 0;
			len = 0;
		}

		if (count >= limit)
			break;

read_again:
		buf1_len = 0;
		buf2_len = 0;
		entry = next_entry;
		buf = &rx_ch->buf_pool[entry];

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_ch->dma_erx + entry);
		else
			p = rx_ch->dma_rx + entry;

		/* read the status of the incoming frame */
		status = xgmac_rx_status(priv, &priv->dev->stats,
				&priv->xstats, p);
		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & dma_own))
			break;

		rx_ch->cur_rx = XGMAC_GET_ENTRY(rx_ch->cur_rx, DMA_RX_SIZE);
		next_entry = rx_ch->cur_rx;

		if (priv->extend_desc)
			np = (struct dma_desc *)(rx_ch->dma_erx + next_entry);
		else
			np = rx_ch->dma_rx + next_entry;

		prefetch(np);

		if (priv->extend_desc)
			xgmac_rx_extended_status(priv, &priv->dev->stats,
					&priv->xstats, rx_ch->dma_erx + entry);
		if (unlikely(status == discard_frame)) {
			page_pool_recycle_direct(rx_ch->page_pool, buf->page);
			buf->page = NULL;
			error = 1;
			if (!priv->hwts_rx_en)
				priv->dev->stats.rx_errors++;
		}

		if (unlikely(error && (status & rx_not_ls)))
			goto read_again;
		if (unlikely(error)) {
			dev_kfree_skb(skb);
			skb = NULL;
			count++;
			continue;
		}

		/* Buffer is good. Go on. */

		prefetch(page_address(buf->page));
		if (buf->sec_page)
			prefetch(page_address(buf->sec_page));

		buf1_len = xgmac_rx_buf1_len(priv, p, status, len);
		len += buf1_len;
		buf2_len = xgmac_rx_buf2_len(priv, p, status, len);
		len += buf2_len;
		/* ACS is set; GMAC core strips PAD/FCS for IEEE 802.3
		 * Type frames (LLC/LLC-SNAP)
		 *
		 * llc_snap is never checked in GMAC >= 4, so this ACS
		 * feature is always disabled and packets need to be
		 * stripped manually.
		 */
		if (likely(!(status & rx_not_ls)) &&
		    unlikely(status != llc_snap)) {
			if (buf2_len)
				buf2_len -= ETH_FCS_LEN;
			else
				buf1_len -= ETH_FCS_LEN;

			len -= ETH_FCS_LEN;
		}

		/* When SPH is enabled, and HL in in desc is 0, both buf1
		 * and buf2 are used to store the packet, buf1 can store
		 * upto HDSMS (256) bytes. The rest of the packet is stored
		 * in buf2.
		 */
		if (priv->sph && buf1_len > XGMAC_MAX_SPH_SIZE) {
			buf2_len += (buf1_len - XGMAC_MAX_SPH_SIZE);
			buf1_len = XGMAC_MAX_SPH_SIZE;
		}

		if (!skb) {
			skb = napi_alloc_skb(&ch->rx_napi, buf1_len);
			if (!skb) {
				priv->dev->stats.rx_dropped++;
				count++;
				goto drain_data;
			}

			dma_sync_single_for_cpu(priv->device, buf->addr,
						buf1_len, DMA_FROM_DEVICE);
			skb_copy_to_linear_data(skb, page_address(buf->page),
						buf1_len);
			skb_put(skb, buf1_len);

			/* Data payload copied into SKB, page ready for recycle */
			page_pool_recycle_direct(rx_ch->page_pool, buf->page);
			buf->page = NULL;
		} else if (buf1_len) {
			dma_sync_single_for_cpu(priv->device, buf->addr,
						buf1_len, DMA_FROM_DEVICE);
			skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags,
					buf->page, 0, buf1_len,
					priv->dma_buf_sz);

			/* Data payload appended into SKB */
			page_pool_release_page(rx_ch->page_pool, buf->page);
			buf->page = NULL;
		}

		if (buf2_len) {
			dma_sync_single_for_cpu(priv->device, buf->sec_addr,
						buf2_len, DMA_FROM_DEVICE);
			skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags,
					buf->sec_page, 0, buf2_len,
					priv->dma_buf_sz);

			/* Data payload appended into SKB */
			page_pool_release_page(rx_ch->page_pool, buf->sec_page);
			buf->sec_page = NULL;
		}

drain_data:
		if (likely(status & rx_not_ls))
			goto read_again;
		if (!skb)
			continue;

		/* Got entire packet into SKB. Finish it. */

		xgmac_get_rx_hwtstamp(priv, p, np, skb);
		xgmac_rx_vlan(priv->dev, skb);
		skb->protocol = eth_type_trans(skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		if (!xgmac_get_rx_hash(priv, p, &hash, &hash_type))
			skb_set_hash(skb, hash, hash_type);

		skb_record_rx_queue(skb, dev_queue);

		if (priv->profiling_rx) {
			struct iphdr *ip_header = NULL;

			/* Ensure the skb has an Ethernet header */
			if (skb->len >= sizeof(struct ethhdr)) {
				struct ethhdr *eth = eth_hdr(skb);

				if (eth->h_proto == htons(ETH_P_IP))
					ip_header = (struct iphdr *)((u8 *)eth +
							 sizeof(struct ethhdr));
			}

			if (ip_header) {
				u16 frag_off = ntohs(ip_header->frag_off);

				/* Check if this is the first fragment */
				if ((frag_off & IP_MF) && !(frag_off & IP_OFFSET) &&
				    pf_rx_first_seg_idx < MAX_PROFILE_PKT) {
					asm volatile("mrs %0, cntvct_el0" : "=r" (stmr_val));
					pf_rx_data[pf_rx_first_seg_idx].first_seg_stmr = stmr_val;
					pf_rx_first_seg_idx++;
				}

				/* Check if this is the last fragment */
				if (!(frag_off & IP_MF) && (frag_off & IP_OFFSET) &&
				    pf_rx_last_seg_idx < MAX_PROFILE_PKT) {
					asm volatile("mrs %0, cntvct_el0" : "=r" (stmr_val));
					pf_rx_data[pf_rx_last_seg_idx].last_seg_stmr = stmr_val;
					if (pf_rx_last_seg_idx == 0)
						cal_napi_rx_intv = true;
					pf_rx_last_seg_idx++;
					if (pf_rx_last_seg_idx == MAX_PROFILE_PKT)
						cal_napi_rx_intv = false;
				}
			}
		}

		napi_gro_receive(&ch->rx_napi, skb);
		skb = NULL;

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += len;
		count++;
	}

	if (status & rx_not_ls || skb) {
		rx_ch->state_saved = true;
		rx_ch->state.skb = skb;
		rx_ch->state.error = error;
		rx_ch->state.len = len;
	}

	xgmac_rx_refill(priv, chan);

	priv->xstats.rx_pkt_n += count;

	return count;
}

static int xgmac_napi_poll_rx(struct napi_struct *napi, int budget)
{
	struct xgmac_channel *ch =
		container_of(napi, struct xgmac_channel, rx_napi);
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan = ch->index;
	int work_done;
	unsigned long stmr_val, napi_intv;

	if (priv->profiling_rx && !priv->poll_mode && cal_napi_rx_intv) {
		asm volatile("mrs %0, cntvct_el0" : "=r" (stmr_val));
		if (cal_napi_rx_intv && napi_rx_prev_val != 0) {
			if (stmr_val >= napi_rx_prev_val)
				napi_intv = stmr_val - napi_rx_prev_val;
			else
				napi_intv = stmr_val + 0x100000000 - napi_rx_prev_val;
			if (napi_intv > max_napi_rx_intv)
				max_napi_rx_intv = napi_intv;
			if (napi_intv < min_napi_rx_intv)
				min_napi_rx_intv = napi_intv;

			num_napi_rx_call++;
			sum_napi_rx_intv += napi_intv;
		}
		napi_rx_prev_val = stmr_val;
	}

	priv->xstats.napi_poll++;

	work_done = xgmac_rx(priv, budget, chan);
	if (virt_chan_avail(priv) && work_done < budget) {
		u32 virt_rx_chan = priv->plat->rx_chans_to_use;

		work_done += xgmac_virt_chan_rx(priv,
					 budget - work_done, virt_rx_chan);
	}

	/* If napi poll mode is enabled, don't enable irq. Returning budget
	 * value is necessary, otherwise kernel wouldn't schedule napi again.
	 */
	if (priv->poll_mode)
		return budget;

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		unsigned long flags;

		spin_lock_irqsave(&ch->lock, flags);
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 1, 0);
		spin_unlock_irqrestore(&ch->lock, flags);
		if (virt_chan_avail(priv))
			xgmac_virt_ch_timer_arm(priv);
	}

	return work_done;
}

static int xgmac_napi_poll_tx(struct napi_struct *napi, int budget)
{
	struct xgmac_channel *ch =
		container_of(napi, struct xgmac_channel, tx_napi);
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan = ch->index;
	int work_done;

	priv->xstats.napi_poll++;

	work_done = xgmac_tx_clean(priv, DMA_TX_SIZE, chan);
	work_done = min(work_done, budget);
	if (virt_chan_avail(priv) && work_done < budget) {
		u32 virt_tx_chan = priv->plat->tx_chans_to_use;

		work_done += xgmac_tx_clean(priv, DMA_TX_SIZE,
					    virt_tx_chan);
		work_done = min(work_done, budget);
	}

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		unsigned long flags;

		spin_lock_irqsave(&ch->lock, flags);
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 0, 1);
		spin_unlock_irqrestore(&ch->lock, flags);
		if (virt_chan_avail(priv))
			xgmac_virt_ch_timer_arm(priv);
	}

	return work_done;
}

/**
 *  xgmac_tx_timeout
 *  @dev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable time. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void xgmac_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	xgmac_global_err(priv);
}

/**
 *  xgmac_set_rx_mode - entry point for multicast addressing
 *  @dev : pointer to the device structure
 *  Description:
 *  This function is a driver entry point which gets called by the kernel
 *  whenever multicast addresses must be enabled/disabled.
 *  Return value:
 *  void.
 */
static void xgmac_set_rx_mode(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	xgmac_set_filter(priv, priv->hw, dev);
}

/**
 *  xgmac_change_mtu - entry point to change MTU size for the device.
 *  @dev : device pointer.
 *  @new_mtu : the new MTU size for the device.
 *  Description: the Maximum Transfer Unit (MTU) is used by the network layer
 *  to drive packet transmission. Ethernet has an MTU of 1500 octets
 *  (ETH_DATA_LEN). This value can be changed with ifconfig.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int xgmac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int txfifosz = priv->plat->tx_fifo_size;
	const int mtu = new_mtu;

	if (txfifosz == 0 || txfifosz > priv->dma_cap.tx_fifo_size)
		txfifosz = priv->dma_cap.tx_fifo_size;

	txfifosz /= priv->plat->tx_queues_to_use;

	if (netif_running(dev)) {
		netdev_err(priv->dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	new_mtu = XGMAC_ALIGN(new_mtu+14);

	/* If condition true, FIFO is too small or MTU too large */
	if (priv->plat->ecpri_en) {
		if ((txfifosz < new_mtu) || (new_mtu > BUF_SIZE_9KiB))
			return -EINVAL;
	}
	else {
		if ((txfifosz < new_mtu) || (new_mtu > BUF_SIZE_16KiB))
			return -EINVAL;
	}

	dev->mtu = mtu;

	netdev_update_features(dev);

	return 0;
}

static netdev_features_t xgmac_fix_features(struct net_device *dev,
					     netdev_features_t features)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (priv->plat->rx_coe == XGMAC_RX_COE_NONE)
		features &= ~NETIF_F_RXCSUM;

	if (!priv->plat->tx_coe)
		features &= ~NETIF_F_CSUM_MASK;

	/* Some GMAC devices have a bugged Jumbo frame support that
	 * needs to have the Tx COE disabled for oversized frames
	 * (due to limited buffer sizes). In this case we disable
	 * the TX csum insertion in the TDES and not use SF.
	 */
	if (priv->plat->bugged_jumbo && (dev->mtu > ETH_DATA_LEN))
		features &= ~NETIF_F_CSUM_MASK;

	/* Disable tso if asked by ethtool */
	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		if (features & NETIF_F_TSO)
			priv->tso = true;
		else
			priv->tso = false;
	}

	return features;
}

static int xgmac_set_features(struct net_device *netdev,
			       netdev_features_t features)
{
	struct xgmac_priv *priv = netdev_priv(netdev);
	u32 rx_cnt = priv->plat->rx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	bool sph_en;
	u32 chan;

	/* Keep the COE Type in case of csum is supporting */
	if (features & NETIF_F_RXCSUM)
		priv->hw->rx_csum = priv->plat->rx_coe;
	else
		priv->hw->rx_csum = 0;
	/* No check needed because rx_coe has been set before and it will be
	 * fixed in case of issue.
	 */
	xgmac_rx_ipc(priv, priv->hw);

	/* No header split for eCPRI or pmd channel */
	sph_en = (priv->hw->rx_csum > 0) && priv->plat->sph_en;
	priv->sph = sph_en;
	for (chan = 0; chan < rx_cnt; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		if (priv->plat->ecpri_en && chan == 0)
			xgmac_enable_sph(priv, priv->ioaddr, false, chan);
		else
			xgmac_enable_sph(priv, priv->ioaddr, sph_en, chan);
	}

	return 0;
}

/**
 *  xgmac_interrupt - main ISR
 *  @irq: interrupt number.
 *  @data: to pass the net device pointer (must be valid).
 *  Description: this is the main driver interrupt service routine.
 *  It can call:
 *  o DMA service routine (to manage incoming frame reception and transmission
 *    status)
 *  o Core interrupts to manage: remote wake-up, management counter, LPI
 *    interrupts.
 */
static irqreturn_t xgmac_interrupt(int irq, void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queues_count;
	u32 queue;
	int mtl_status;
	int status;

	queues_count = (rx_cnt > tx_cnt) ? rx_cnt : tx_cnt;

	status = xgmac_host_irq_status(priv, priv->hw, &priv->xstats);
	if (status & CORE_IRQ_LINK_FAULT) {
		//netdev_warn(priv->dev, "link fault\n");
	}

	if (priv->irq_wake)
		pm_wakeup_event(priv->device, 0);

	/* Check if adapter is up */
	if (test_bit(XGMAC_DOWN, &priv->state))
		return IRQ_HANDLED;
	/* Check if a fatal error happened */
	if (xgmac_safety_feat_interrupt(priv))
		return IRQ_HANDLED;

	/* To handle XGMAC own interrupts */
	if (unlikely(status)) {
		/* For LPI we need to save the tx status */
		if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
			priv->tx_path_in_lpi_mode = true;
		if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
			priv->tx_path_in_lpi_mode = false;
	}

	for (queue = 0; queue < queues_count; queue++) {
		mtl_status = xgmac_host_mtl_irq_status(priv, priv->hw, queue);
		if (mtl_status & CORE_IRQ_MTL_RX_OVERFLOW) {
			//netdev_warn(priv->dev, "Queue %d overflow\n", queue);
		}
	}

	/* PCS link status */
	if (priv->hw->pcs) {
		if (priv->xstats.pcs_link)
			netif_carrier_on(dev);
		else
			netif_carrier_off(dev);
	}

	/* To handle DMA interrupts */
	/* When napi runs, it disables channel irq. If any channel error
	 * occurs during that time, it will trigger the main irq. Thus
	 * we need to check channel interrupt here.
	 */
	xgmac_dma_interrupt(priv);

	return IRQ_HANDLED;
}

static irqreturn_t xgmac_dma_isr(int irq, void *data)
{
	struct xgmac_channel *ch = (struct xgmac_channel *)data;
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan;
	int status;

	/* Check if adapter is up */
	if (test_bit(XGMAC_DOWN, &priv->state))
		return IRQ_HANDLED;

	chan = ch->index;
	status = xgmac_napi_check(priv, chan);

	if (unlikely(status == tx_hard_error)) {
		xgmac_tx_err(priv, chan);
	}

	return IRQ_HANDLED;
}

static irqreturn_t xgmac_uio_isr(int irq, void *data)
{
	struct xgmac_channel *ch = (struct xgmac_channel *)data;
	struct xgmac_priv *priv = ch->priv_data;
	u32 chan = ch->index;

	/* Clear Rx interrupt */
	writel(XGMAC_RI, priv->ioaddr + XGMAC_DMA_CH_STATUS(chan));

	/* Disable Rx interrupt */
	xgmac_disable_dma_irq(priv, priv->ioaddr, chan, 1, 0);

	/* Notify userspace program with an event */
	uio_event_notify(&priv->uio_dev[chan].uinfo);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/* Polling receive - used by NETCONSOLE and other diagnostic tools
 * to allow network I/O with interrupts disabled.
 */
static void xgmac_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	xgmac_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif

int xgmac_get_ptp_sec(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 data[3];

	if (!priv->dma_cap.time_stamp)
		return -EOPNOTSUPP;

	data[0] = readl(priv->ioaddr + XGMAC_SYSTEM_TIME_SEC);
	data[1] = readl(priv->ioaddr + XGMAC_SYSTEM_TIME_SEC_HIGH);
	data[2] = readl(priv->ioaddr + XGMAC_SYSTEM_TIME_NSEC);
	if (copy_to_user(ifr->ifr_data, data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static int xgmac_get_pmd_support(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (copy_to_user(ifr->ifr_data, &priv->plat->pmd_enabled, sizeof(int)))
		return -EFAULT;

	return 0;
}

static int xgmac_set_aux_snap_en(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int enable;
	u32 reg;

	if (priv->dma_cap.aux_snap_num == 0)
		return -EOPNOTSUPP;

	if (copy_from_user(&enable, ifr->ifr_data, sizeof(int)))
		return -EFAULT;

	reg = readl(priv->ioaddr + XGMAC_AUX_CTRL);
	if (enable)
		reg |= XGMAC_ATSEN0;
	else
		reg &= ~XGMAC_ATSEN0;
	reg |= XGMAC_ATSFC;	// Clear FIFO
	writel(reg, priv->ioaddr + XGMAC_AUX_CTRL);

	return 0;
}

static int xgmac_get_aux_snap_en(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	int enable;
	u32 reg;

	reg = readl(priv->ioaddr + XGMAC_AUX_CTRL);
	if (reg & XGMAC_ATSEN0)
		enable = 1;
	else
		enable = 0;

	if (copy_to_user(ifr->ifr_data, &enable, sizeof(int)))
		return -EFAULT;

	return 0;
}

int xgmac_get_aux_snap_ts(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 reg;
	u32 snap_cnt;
	u32 sec = 0;
	u32 nsec = 0;

	if (priv->dma_cap.aux_snap_num == 0)
		return -EOPNOTSUPP;

	if (suspend_aux_snap_read) {
		usleep_range(2000, 3000);
		return -EBUSY;
	}

	reg = readl(priv->ioaddr + XGMAC_AUX_CTRL);
	if (reg & XGMAC_ATSEN0) {
		reg = readl(priv->ioaddr + XGMAC_TIMESTAMP_STATUS);
		snap_cnt = (reg & XGMAC_ATSNS) >> XGMAC_ATSNS_SHIFT;
		/* Return the last entry */
		while (snap_cnt > 0) {
			/* Must read nsec first */
			nsec = readl(priv->ioaddr + XGMAC_AUX_SNAP_NSEC);
			sec = readl(priv->ioaddr + XGMAC_AUX_SNAP_SEC);
			snap_cnt--;
		}
		if (copy_to_user(ifr->ifr_data, &sec, sizeof(u32)))
			return -EFAULT;
		if (copy_to_user(ifr->ifr_data+4, &nsec, sizeof(u32)))
			return -EFAULT;
	}
	else
		return -EINVAL;

	return 0;
}

int xgmac_get_link_lane_num(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	if (copy_to_user(ifr->ifr_data, &priv->plat->eth_link, sizeof(u32)))
		return -EFAULT;
	if (copy_to_user(ifr->ifr_data + 4, &priv->plat->phy_lane, sizeof(u32)))
		return -EFAULT;

	return 0;
}

/**
 * L4 Filter to divert L2 unicast DHCPv4 packets to Linux instead of DPDK/VPP.
 * L4 Filter 6,7 - Unicast PTP
 * L4 Filter 5 - DHCPv4
 */
static void configure_l4_dhcpv4_filter(struct xgmac_priv *priv, u8 enable)
{
	int dhcp4_gen_idx = priv->dma_cap.l3l4fnum - 3;
	struct xgmac_flow_entry *entry5 = &priv->flow_entries[dhcp4_gen_idx];
	int dhcp_client_port = 68;
	int linux_ch = priv->plat->rx_chans_to_use - 1;

	entry5->in_use = true;
	if (priv->plat->pmd_start_chan == 0) {
		entry5->is_l4 = true;
		entry5->ip_proto = IPPROTO_UDP;
		if (enable) {
			xgmac_config_l4_filter(priv, priv->hw, dhcp4_gen_idx, true,
				true, false, false, dhcp_client_port, true, linux_ch);
			netdev_info(priv->dev, "Enable L4 filter %d for DHCP\n",
				entry5->idx);
		} else {
			xgmac_config_l4_filter(priv, priv->hw, dhcp4_gen_idx, false,
				false, false, false, dhcp_client_port, false, linux_ch);
			netdev_info(priv->dev, "Disable L4 filter %d for DHCP\n",
				entry5->idx);
		}
	}
}

static void xgmac_enable_irq(struct xgmac_priv *priv)
{
	u32 rx_channels_count = priv->plat->rx_chans_to_use;
	u32 tx_channels_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	for (chan = 0; chan < rx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 1, 0);
	}

	for (chan = 0; chan < tx_channels_count; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		xgmac_enable_dma_irq(priv, priv->ioaddr, chan, 0, 1);
	}
}

static int set_virt_chan(struct xgmac_priv *priv, u8 enable)
{
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	priv->plat->virt_chan_en = false;
	hrtimer_cancel(&priv->virt_ch_timer);
	while (priv->virt_chan_rx_running || priv->virt_chan_tx_running)
		usleep_range(100, 200);

	mutex_lock(&priv->lock);
	xgmac_stop_all_queues(priv);
	xgmac_disable_all_chans(priv);
	for (chan = 0; chan < priv->plat->tx_chans_to_use; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		del_timer_sync(&priv->tx_chan[chan].txtimer);
	}
	xgmac_stop_all_dma(priv);
	xgmac_mac_disable_rx_queues(priv);

	priv->virt_chan_info->linux_state = VIRT_CHAN_INIT;
	virt_chan_reinit(priv);
	priv->plat->virt_chan_en = (enable != 0);
	if (priv->plat->virt_chan_en)
		priv->dev->wanted_features &= ~VIRT_CHAN_HW_FEATRUE_MASK;
	else
		priv->dev->wanted_features |= VIRT_CHAN_HW_FEATRUE_MASK;
	netdev_change_features(priv->dev);

	xgmac_start_all_dma(priv);
	xgmac_enable_all_chans(priv);
	xgmac_start_all_queues(priv);
	xgmac_enable_irq(priv);
	xgmac_mac_enable_rx_queues(priv);
	mutex_unlock(&priv->lock);

	return 0;
}

/**
 * Non LCP Mode with L2 fwd configuration
 * MCast, BCast -> Q1 -> Linux DMA Channel (3) (Static DMA Mapping for Q1)
 * MCast PTP -> Q1 -> Linux DMA Channel (3) (Static DMA Mapping for Q1)
 * MAC0 filter -> Q0 -> Linux DMA Channel (3) (Dynamic DMA Mapping for Q0)
 * Everything else -> Q0 -> DMA Channel 0 (DPDK owns DMA Ch 0,1,2)
 *
 * LCP Mode
 * MCast PTP -> Q1 -> Linux DMA Channel (3) (Static DMA Mapping for Q1)
 * MCast, BCAST queue -> Q1 -> Linux DMA Channel (3) (Static DMA Mapping for Q1)
 *				Queue routing retained as virt channel is available.
 * MAC0 filter -> Q0 -> Linux DMA Channel (0) (Dynamic DMA Mapping for Q0)
 *				Runtime filter config change
 * Everything else -> Q0 -> DMA Channel 0 (DPDK owns DMA Ch 0,1,2)
 *
 */
static int xgmac_lcp_control(struct xgmac_priv *priv, struct fastpath_lcp_config lcp_config)
{
	u8 bcast_filter_kernel_default = false;

	if (lcp_config.mcbc_routing_queue >= priv->dma_cap.number_rx_queues ||
		lcp_config.mcbc_routing_queue >= priv->plat->rx_queues_to_use)
		bcast_filter_kernel_default = true;

	/* Multicast, Broadcast queue filter enable/disable*/
	if (bcast_filter_kernel_default && priv->plat->pmd_enabled
								&& priv->plat->pmd_start_chan == 0)
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ,
				       priv->plat->rx_queues_to_use - 1, true);
	else if (bcast_filter_kernel_default || !lcp_config.mcbc_routing_enable)
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ,
				       0, false);
	else
		xgmac_rx_queue_routing(priv, priv->hw, PACKET_MCBCQ,
				       lcp_config.mcbc_routing_queue, true);

	if (lcp_config.mac0_filter_dma_ch >= priv->dma_cap.number_rx_channel ||
		lcp_config.mac0_filter_dma_ch >= priv->plat->rx_chans_to_use)
		lcp_config.mac0_filter_enable = false;

	/* Divert all traffic matching MAC address 0 to DMA channel or reset to default*/
	if (!lcp_config.mac0_filter_enable && priv->plat->pmd_enabled
								&& priv->plat->pmd_start_chan == 0)
		xgmac_set_umac_dma_chan(priv, priv->hw, 0, priv->plat->rx_chans_to_use - 1);
	else if (!lcp_config.mac0_filter_enable)
		xgmac_set_umac_dma_chan(priv, priv->hw, 0, 0);
	else
		xgmac_set_umac_dma_chan(priv, priv->hw, 0, lcp_config.mac0_filter_dma_ch);

	if (lcp_config.dhcp_filter)
		configure_l4_dhcpv4_filter(priv, true);
	else
		configure_l4_dhcpv4_filter(priv, false);

	return 0;
}

static int xgmac_lcp_ioctl_dispatch(struct net_device *dev, struct ifreq *ifr)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	struct fastpath_lcp_config lcp_config = {0};

	if (copy_from_user(&lcp_config, ifr->ifr_data, sizeof(struct fastpath_lcp_config)))
		return -EFAULT;

	if (lcp_config.message_type == FASTPATH_LCP_CFG)
		return xgmac_lcp_control(priv, lcp_config);
	else if (lcp_config.message_type == FASTPATH_LCP_CFG_VIRT_CH)
		return set_virt_chan(priv, lcp_config.virt_ch_state);
	else
		return -EINVAL;
}

/**
 *  xgmac_ioctl - Entry point for the Ioctl
 *  @dev: Device pointer.
 *  @rq: An IOCTL specefic structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  @cmd: IOCTL command
 *  Description:
 *  Currently it supports the phy_mii_ioctl(...) and HW time stamping.
 */
static int xgmac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct xgmac_priv *priv = netdev_priv (dev);
	int ret = -EOPNOTSUPP;

	if (cmd == SIOCGPMDSUPPORT)
		return xgmac_get_pmd_support(dev, rq);
	else if (cmd == SIOCSPMDSUPPORT)
		return -EOPNOTSUPP;

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		ret = phylink_mii_ioctl(priv->phylink, rq, cmd);
		break;
	case SIOCSHWTSTAMP:
		ret = xgmac_hwtstamp_set(dev, rq);
		break;
	case SIOCGHWTSTAMP:
		ret = xgmac_hwtstamp_get(dev, rq);
		break;
	case SIOCGPTPSEC:
		ret = xgmac_get_ptp_sec(dev, rq);
		break;
	case SIOCSECPRIUDP:
		ret = ecpri_set_udp_port(dev, rq);
		break;
	case SIOCGECPRIUDP:
		ret = ecpri_get_udp_port(dev, rq);
		break;
	case SIOCSECPRIVLAN:
		ret = ecpri_set_vlan(dev, rq);
		break;
	case SIOCGECPRIVLAN:
		ret = ecpri_get_vlan(dev, rq);
		break;
	case SIOCSECPRIDESTIP:
		ret = ecpri_set_dst_ip(dev, rq);
		break;
	case SIOCGECPRIDESTIP:
		ret = ecpri_get_dst_ip(dev, rq);
		break;
	case SIOCSECPRIDESTMAC:
		ret = ecpri_set_dst_mac(dev, rq);
		break;
	case SIOCGECPRIDESTMAC:
		ret = ecpri_get_dst_mac(dev, rq);
		break;
	case SIOCSAUXSNAPEN:
		ret = xgmac_set_aux_snap_en(dev, rq);
		break;
	case SIOCGAUXSNAPEN:
		ret = xgmac_get_aux_snap_en(dev, rq);
		break;
	case SIOCGAUXSNAPTS:
		ret = xgmac_get_aux_snap_ts(dev, rq);
		break;
	case SIOCGLINKLANENUM:
		ret = xgmac_get_link_lane_num(dev, rq);
		break;
	case SIOCSLCPCONFIG:
		ret = xgmac_lcp_ioctl_dispatch(dev, rq);
		break;
	default:
		break;
	}

	return ret;
}

static int xgmac_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
				    void *cb_priv)
{
	struct xgmac_priv *priv = cb_priv;
	int ret = -EOPNOTSUPP;

	if (!tc_cls_can_offload_and_chain0(priv->dev, type_data))
		return ret;

	xgmac_disable_all_chans(priv);

	switch (type) {
	case TC_SETUP_CLSU32:
		ret = xgmac_tc_setup_cls_u32(priv, priv, type_data);
		break;
	case TC_SETUP_CLSFLOWER:
		ret = xgmac_tc_setup_cls(priv, priv, type_data);
		break;
	default:
		break;
	}

	xgmac_enable_all_chans(priv);
	return ret;
}

static LIST_HEAD(xgmac_block_cb_list);

static int xgmac_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			   void *type_data)
{
	struct xgmac_priv *priv = netdev_priv(ndev);

	switch (type) {
	case TC_SETUP_BLOCK:
		return flow_block_cb_setup_simple(type_data,
						  &xgmac_block_cb_list,
						  xgmac_setup_tc_block_cb,
						  priv, priv, true);
	case TC_SETUP_QDISC_CBS:
		return xgmac_tc_setup_cbs(priv, priv, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return xgmac_tc_setup_taprio(priv, priv, type_data);
	case TC_SETUP_QDISC_ETF:
		return xgmac_tc_setup_etf(priv, priv, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static u16 xgmac_select_queue(struct net_device *dev, struct sk_buff *skb,
			       struct net_device *sb_dev)
{
	int gso = skb_shinfo(skb)->gso_type;

	if (gso & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6 | SKB_GSO_UDP_L4)) {
		/*
		 * There is no way to determine the number of TSO/USO
		 * capable Queues. Let's use always the Queue 0
		 * because if TSO/USO is supported then at least this
		 * one will be capable.
		 */
		return 0;
	}

	return netdev_pick_tx(dev, skb, NULL) % dev->real_num_tx_queues;
}

static int xgmac_set_mac_address(struct net_device *ndev, void *addr)
{
	struct xgmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		return ret;

	xgmac_set_umac_addr(priv, priv->hw, ndev->dev_addr, 0);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *xgmac_fs_dir;

static void sysfs_display_ring(void *head, int size, int extend_desc,
			       struct seq_file *seq)
{
	int i;
	struct dma_extended_desc *ep = (struct dma_extended_desc *)head;
	struct dma_desc *p = (struct dma_desc *)head;

	for (i = 0; i < size; i++) {
		if (extend_desc) {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(ep),
				   le32_to_cpu(ep->basic.des0),
				   le32_to_cpu(ep->basic.des1),
				   le32_to_cpu(ep->basic.des2),
				   le32_to_cpu(ep->basic.des3));
			ep++;
		} else {
			seq_printf(seq, "%d [0x%x]: 0x%x 0x%x 0x%x 0x%x\n",
				   i, (unsigned int)virt_to_phys(p),
				   le32_to_cpu(p->des0), le32_to_cpu(p->des1),
				   le32_to_cpu(p->des2), le32_to_cpu(p->des3));
			p++;
		}
		seq_printf(seq, "\n");
	}
}

static int xgmac_rings_status_show(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 rx_count = priv->plat->rx_chans_to_use;
	u32 tx_count = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_rx_ecpri_ch = 0;
	u32 num_tx_ecpri_ch = 0;

	if ((dev->flags & IFF_UP) == 0)
		return 0;

	/* eCPRI ring is managed by eCPRI SS */
	if (priv->plat->ecpri_en) {
		num_rx_ecpri_ch = 1;
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_tx_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_tx_ecpri_ch = 1;
	}

	for (chan = num_rx_ecpri_ch; chan < rx_count; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		seq_printf(seq, "RX Chan %d:\n", chan);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)rx_ch->dma_erx,
					   DMA_RX_SIZE, 1, seq);
		} else {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring((void *)rx_ch->dma_rx,
					   DMA_RX_SIZE, 0, seq);
		}
	}

	for (chan = num_tx_ecpri_ch; chan < tx_count; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		seq_printf(seq, "TX Chan %d:\n", chan);

		if (priv->extend_desc) {
			seq_printf(seq, "Extended descriptor ring:\n");
			sysfs_display_ring((void *)tx_ch->dma_etx,
					   DMA_TX_SIZE, 1, seq);
		} else if (!(tx_ch->tbs & XGMAC_TBS_AVAIL)) {
			seq_printf(seq, "Descriptor ring:\n");
			sysfs_display_ring((void *)tx_ch->dma_tx,
					   DMA_TX_SIZE, 0, seq);
		}
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xgmac_rings_status);

static int xgmac_dma_cap_show(struct seq_file *seq, void *v)
{
	struct net_device *dev = seq->private;
	struct xgmac_priv *priv = netdev_priv(dev);

	if (!priv->hw_cap_support) {
		seq_printf(seq, "DMA HW features not supported\n");
		return 0;
	}

	seq_printf(seq, "==============================\n");
	seq_printf(seq, "\tDMA HW features\n");
	seq_printf(seq, "==============================\n");

	seq_printf(seq, "\t10/100 Mbps: %s\n",
		   (priv->dma_cap.mbps_10_100) ? "Y" : "N");
	seq_printf(seq, "\t1000 Mbps: %s\n",
		   (priv->dma_cap.mbps_1000) ? "Y" : "N");
	seq_printf(seq, "\tHalf duplex: %s\n",
		   (priv->dma_cap.half_duplex) ? "Y" : "N");
	seq_printf(seq, "\tHash Filter: %s\n",
		   (priv->dma_cap.hash_filter) ? "Y" : "N");
	seq_printf(seq, "\tMultiple MAC address registers: %s\n",
		   (priv->dma_cap.multi_addr) ? "Y" : "N");
	seq_printf(seq, "\tPCS (TBI/SGMII/RTBI PHY interfaces): %s\n",
		   (priv->dma_cap.pcs) ? "Y" : "N");
	seq_printf(seq, "\tSMA (MDIO) Interface: %s\n",
		   (priv->dma_cap.sma_mdio) ? "Y" : "N");
	seq_printf(seq, "\tPMT Remote wake up: %s\n",
		   (priv->dma_cap.pmt_remote_wake_up) ? "Y" : "N");
	seq_printf(seq, "\tPMT Magic Frame: %s\n",
		   (priv->dma_cap.pmt_magic_frame) ? "Y" : "N");
	seq_printf(seq, "\tRMON module: %s\n",
		   (priv->dma_cap.rmon) ? "Y" : "N");
	seq_printf(seq, "\tIEEE 1588-2008 Time Stamp: %s\n",
		   (priv->dma_cap.time_stamp) ? "Y" : "N");
	seq_printf(seq, "\t802.3az - Energy-Efficient Ethernet (EEE): %s\n",
		   (priv->dma_cap.eee) ? "Y" : "N");
	seq_printf(seq, "\tAV features: %s\n", (priv->dma_cap.av) ? "Y" : "N");
	seq_printf(seq, "\tChecksum Offload in TX: %s\n",
		   (priv->dma_cap.tx_coe) ? "Y" : "N");
	seq_printf(seq, "\tIP Checksum Offload in RX: %s\n",
			   (priv->dma_cap.rx_coe) ? "Y" : "N");
	seq_printf(seq, "\tRXFIFO > 2048bytes: %s\n",
		   (priv->dma_cap.rxfifo_over_2048) ? "Y" : "N");
	seq_printf(seq, "\tNumber of Additional RX channel: %d\n",
		   priv->dma_cap.number_rx_channel);
	seq_printf(seq, "\tNumber of Additional TX channel: %d\n",
		   priv->dma_cap.number_tx_channel);
	seq_printf(seq, "\tNumber of Additional RX queues: %d\n",
		   priv->dma_cap.number_rx_queues);
	seq_printf(seq, "\tNumber of Additional TX queues: %d\n",
		   priv->dma_cap.number_tx_queues);
	seq_printf(seq, "\tEnhanced descriptors: %s\n",
		   (priv->dma_cap.enh_desc) ? "Y" : "N");
	seq_printf(seq, "\tTX Fifo Size: %d\n", priv->dma_cap.tx_fifo_size);
	seq_printf(seq, "\tRX Fifo Size: %d\n", priv->dma_cap.rx_fifo_size);
	seq_printf(seq, "\tHash Table Size: %d\n", priv->dma_cap.hash_tb_sz);
	seq_printf(seq, "\tTSO: %s\n", priv->dma_cap.tsoen ? "Y" : "N");
	seq_printf(seq, "\tNumber of PPS Outputs: %d\n",
		   priv->dma_cap.pps_out_num);
	seq_printf(seq, "\tSafety Features: %s\n",
		   priv->dma_cap.asp ? "Y" : "N");
	seq_printf(seq, "\tFlexible RX Parser: %s\n",
		   priv->dma_cap.frpsel ? "Y" : "N");
	seq_printf(seq, "\tEnhanced Addressing: %d\n",
		   priv->dma_cap.addr64);
	seq_printf(seq, "\tReceive Side Scaling: %s\n",
		   priv->dma_cap.rssen ? "Y" : "N");
	seq_printf(seq, "\tVLAN Hash Filtering: %s\n",
		   priv->dma_cap.vlhash ? "Y" : "N");
	seq_printf(seq, "\tSplit Header: %s\n",
		   priv->dma_cap.sphen ? "Y" : "N");
	seq_printf(seq, "\tVLAN TX Insertion: %s\n",
		   priv->dma_cap.vlins ? "Y" : "N");
	seq_printf(seq, "\tDouble VLAN: %s\n",
		   priv->dma_cap.dvlan ? "Y" : "N");
	seq_printf(seq, "\tNumber of L3/L4 Filters: %d\n",
		   priv->dma_cap.l3l4fnum);
	seq_printf(seq, "\tARP Offloading: %s\n",
		   priv->dma_cap.arpoffsel ? "Y" : "N");
	seq_printf(seq, "\tEnhancements to Scheduled Traffic (EST): %s\n",
		   priv->dma_cap.estsel ? "Y" : "N");
	seq_printf(seq, "\tFrame Preemption (FPE): %s\n",
		   priv->dma_cap.fpesel ? "Y" : "N");
	seq_printf(seq, "\tTime-Based Scheduling (TBS): %s\n",
		   priv->dma_cap.tbssel ? "Y" : "N");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xgmac_dma_cap);

/* Use network device events to rename debugfs file entries.
 */
static int xgmac_device_event(struct notifier_block *unused,
			       unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct xgmac_priv *priv = netdev_priv(dev);

	if (dev->netdev_ops != &xgmac_netdev_ops)
		goto done;

	switch (event) {
	case NETDEV_CHANGENAME:
		if (priv->dbgfs_dir)
			priv->dbgfs_dir = debugfs_rename(xgmac_fs_dir,
							 priv->dbgfs_dir,
							 xgmac_fs_dir,
							 dev->name);
		break;
	}
done:
	return NOTIFY_DONE;
}

static struct notifier_block xgmac_notifier = {
	.notifier_call = xgmac_device_event,
};

static void xgmac_init_fs(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	rtnl_lock();

	/* Create per netdev entries */
	priv->dbgfs_dir = debugfs_create_dir(dev->name, xgmac_fs_dir);

	/* Entry to report DMA RX/TX rings */
	debugfs_create_file("descriptors_status", 0444, priv->dbgfs_dir, dev,
			    &xgmac_rings_status_fops);

	/* Entry to report the DMA HW features */
	debugfs_create_file("dma_cap", 0444, priv->dbgfs_dir, dev,
			    &xgmac_dma_cap_fops);

	rtnl_unlock();
}

static void xgmac_exit_fs(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);

	debugfs_remove_recursive(priv->dbgfs_dir);
}
#endif /* CONFIG_DEBUG_FS */

static u32 xgmac_vid_crc32_le(__le16 vid_le)
{
	unsigned char *data = (unsigned char *)&vid_le;
	unsigned char data_byte = 0;
	u32 crc = ~0x0;
	u32 temp = 0;
	int i, bits;

	bits = get_bitmask_order(VLAN_VID_MASK);
	for (i = 0; i < bits; i++) {
		if ((i % 8) == 0)
			data_byte = data[i / 8];

		temp = ((crc & 1) ^ data_byte) & 1;
		crc >>= 1;
		data_byte >>= 1;

		if (temp)
			crc ^= 0xedb88320;
	}

	return crc;
}

static int xgmac_vlan_update(struct xgmac_priv *priv, bool is_double)
{
	u32 crc, hash = 0;
	__le16 pmatch = 0;
	int count = 0;
	u16 vid = 0;

	for_each_set_bit(vid, priv->active_vlans, VLAN_N_VID) {
		__le16 vid_le = cpu_to_le16(vid);
		crc = bitrev32(~xgmac_vid_crc32_le(vid_le)) >> 28;
		hash |= (1 << crc);
		count++;
	}

	if (!priv->dma_cap.vlhash) {
		if (count > 2) /* VID = 0 always passes filter */
			return -EOPNOTSUPP;

		pmatch = cpu_to_le16(vid);
		hash = 0;
	}

	return xgmac_update_vlan_hash(priv, priv->hw, hash, pmatch, is_double);
}

static int xgmac_vlan_rx_add_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct xgmac_priv *priv = netdev_priv(ndev);
	bool is_double = false;
	int ret;

	if (be16_to_cpu(proto) == ETH_P_8021AD)
		is_double = true;

	set_bit(vid, priv->active_vlans);
	ret = xgmac_vlan_update(priv, is_double);
	if (ret) {
		clear_bit(vid, priv->active_vlans);
		return ret;
	}

	if (priv->hw->num_vlan) {
		ret = xgmac_add_hw_vlan_rx_fltr(priv, ndev, priv->hw, proto, vid);
		if (ret)
			return ret;
	}

	return 0;
}

static int xgmac_vlan_rx_kill_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct xgmac_priv *priv = netdev_priv(ndev);
	bool is_double = false;
	int ret;

	if (be16_to_cpu(proto) == ETH_P_8021AD)
		is_double = true;

	clear_bit(vid, priv->active_vlans);

	if (priv->hw->num_vlan) {
		ret = xgmac_del_hw_vlan_rx_fltr(priv, ndev, priv->hw, proto, vid);
		if (ret)
			return ret;
	}

	return xgmac_vlan_update(priv, is_double);
}

static const struct net_device_ops xgmac_netdev_ops = {
	.ndo_open = xgmac_open,
	.ndo_start_xmit = xgmac_xmit,
	.ndo_stop = xgmac_release,
	.ndo_change_mtu = xgmac_change_mtu,
	.ndo_fix_features = xgmac_fix_features,
	.ndo_set_features = xgmac_set_features,
	.ndo_set_rx_mode = xgmac_set_rx_mode,
	.ndo_tx_timeout = xgmac_tx_timeout,
	.ndo_do_ioctl = xgmac_ioctl,
	.ndo_setup_tc = xgmac_setup_tc,
	.ndo_select_queue = xgmac_select_queue,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = xgmac_poll_controller,
#endif
	.ndo_set_mac_address = xgmac_set_mac_address,
	.ndo_vlan_rx_add_vid = xgmac_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = xgmac_vlan_rx_kill_vid,
};

static void xgmac_reset_subtask(struct xgmac_priv *priv)
{
	/* No reset */
	return;

	if (!test_and_clear_bit(XGMAC_RESET_REQUESTED, &priv->state))
		return;
	if (test_bit(XGMAC_DOWN, &priv->state))
		return;

	netdev_err(priv->dev, "Reset adapter.\n");

	rtnl_lock();
	netif_trans_update(priv->dev);
	while (test_and_set_bit(XGMAC_RESETING, &priv->state))
		usleep_range(1000, 2000);

	set_bit(XGMAC_DOWN, &priv->state);
	dev_close(priv->dev);
	dev_open(priv->dev, NULL);
	clear_bit(XGMAC_DOWN, &priv->state);
	clear_bit(XGMAC_RESETING, &priv->state);
	rtnl_unlock();
}

static void xgmac_service_task(struct work_struct *work)
{
	struct xgmac_priv *priv = container_of(work, struct xgmac_priv,
			service_task);

	xgmac_reset_subtask(priv);
	clear_bit(XGMAC_SERVICE_SCHED, &priv->state);
}

/**
 *  xgmac_hw_init - Init the MAC device
 *  @priv: driver private structure
 *  Description: this function is to configure the MAC device according to
 *  some platform parameters or the HW capability register. It prepares the
 *  driver to use either ring or chain modes and to setup either enhanced or
 *  normal descriptors.
 */
static int xgmac_hw_init(struct xgmac_priv *priv)
{
	int ret;

	/* ring/chain mode not used for XGMAC */
	priv->chain_mode = chain_mode;

	/* Initialize HW Interface */
	ret = xgmac_hwif_init(priv);
	if (ret)
		return ret;

	/* Get the HW capability (new GMAC newer than 3.50a) */
	priv->hw_cap_support = xgmac_get_hw_features(priv);
	if (priv->hw_cap_support) {
		dev_info(priv->device, "DMA HW capability register supported\n");

		/* We can override some gmac/dma configuration fields: e.g.
		 * enh_desc, tx_coe (e.g. that are passed through the
		 * platform) with the values from the HW capability
		 * register (if supported).
		 */
		priv->plat->enh_desc = priv->dma_cap.enh_desc;
		priv->plat->pmt = priv->dma_cap.pmt_remote_wake_up;
		priv->hw->pmt = priv->plat->pmt;
		if (priv->dma_cap.hash_tb_sz) {
			priv->hw->multicast_filter_bins =
					(BIT(priv->dma_cap.hash_tb_sz) << 5);
			priv->hw->mcast_bits_log2 =
					ilog2(priv->hw->multicast_filter_bins);
		}

		/* TXCOE doesn't work in thresh DMA mode */
		if (priv->plat->force_thresh_dma_mode)
			priv->plat->tx_coe = 0;
		else
			priv->plat->tx_coe = priv->dma_cap.tx_coe;

		/* In case of GMAC4 rx_coe is from HW cap register. */
		priv->plat->rx_coe = priv->dma_cap.rx_coe;

		/* Use the FIFO size from HW feature register */
		priv->plat->tx_fifo_size = priv->dma_cap.tx_fifo_size;
		priv->plat->rx_fifo_size = priv->dma_cap.rx_fifo_size;
	} else {
		dev_info(priv->device, "No HW DMA feature register supported\n");
	}

	if (priv->plat->rx_coe) {
		priv->hw->rx_csum = priv->plat->rx_coe;
		dev_info(priv->device, "RX Checksum Offload Engine supported\n");
		dev_info(priv->device, "COE enabled %d\n", priv->hw->rx_csum);
	}
	if (priv->plat->tx_coe)
		dev_info(priv->device, "TX Checksum insertion supported\n");

	if (priv->plat->pmt) {
		dev_info(priv->device, "Wake-Up On Lan supported\n");
		device_set_wakeup_capable(priv->device, 1);
	}

	if (priv->dma_cap.tsoen)
		dev_info(priv->device, "TSO supported\n");

	/* Run HW quirks, if any */
	if (priv->hwif_quirks) {
		ret = priv->hwif_quirks(priv);
		if (ret)
			return ret;
	}

	/* Rx Watchdog is available in the COREs newer than the 3.40.
	 * In some case, for example on bugged HW this feature
	 * has to be disable and this can be done by passing the
	 * riwt_en field from the platform.
	 */
	if (priv->plat->riwt_en) {
		priv->use_riwt = 1;
		dev_info(priv->device,
			 "Enable RX Mitigation via HW Watchdog Timer\n");
	}

	return 0;
}

static int xgmac_uio_init(struct xgmac_priv *priv, unsigned int chan)
{
	struct device_node *np = priv->device->of_node;
	struct uio_info *uinfo = &priv->uio_dev[chan].uinfo;
	struct resource res;
	int err;
	void *new_uio_shm = NULL;
	volatile struct xgmac_uio_shm *xgmac_ch_uio_shm;

	err = of_address_to_resource(np, 0, &res);
	if (err < 0) {
		dev_err(priv->device, "Failed to get address resource\n");
		return err;
	}

	if (priv->uio_shared_mem == NULL) {
		new_uio_shm = (void *) get_zeroed_page(GFP_KERNEL);
		if (!new_uio_shm) {
			dev_err(priv->device, "Failed to allocate memory for UIO region\n");
		return -ENOMEM;
		}
		priv->uio_shared_mem = new_uio_shm;
	}

	xgmac_ch_uio_shm = (struct xgmac_uio_shm *)
			((u8 *)(priv->uio_shared_mem) + (XGMAC_UIO_SHARED_MEM_OFFSET * chan));
	xgmac_ch_uio_shm->pmd_enabled_flag = XGMAC_UIO_SHM_RX_PMD|XGMAC_UIO_SHM_TX_PMD;

	uinfo->mem[0].addr = res.start;
	uinfo->mem[0].size = resource_size(&res);
	uinfo->mem[0].memtype = UIO_MEM_PHYS;

	uinfo->mem[1].addr = (phys_addr_t)priv->uio_shared_mem;
	uinfo->mem[1].size = PAGE_SIZE;
	uinfo->mem[1].memtype = UIO_MEM_LOGICAL;

	snprintf(priv->uio_dev[chan].name, sizeof(priv->uio_dev[chan].name) - 1,
		 "%s-%u", priv->dev->name, chan);
	uinfo->name = priv->uio_dev[chan].name;
	uinfo->irq = UIO_IRQ_CUSTOM;
	uinfo->version = DRV_MODULE_VERSION;
	uinfo->priv = priv;

	err = uio_register_device(priv->device, uinfo);
	if (err < 0) {
		if (err == -EPROBE_DEFER)
			dev_err(priv->device,
				"UIO class not registered yet, deferring probe\n");
		else
			dev_err(priv->device,
				"UIO device registration failed: %d\n", err);
		free_page((phys_addr_t)new_uio_shm);
	}

	return err;
}

static int xgmac_uio_irq_init(struct xgmac_priv *priv, unsigned int chan)
{
	struct xgmac_channel *ch;
	unsigned int rx_irq, rx_irq_cpu;
	int err;

	ch = &priv->channel[chan];
	rx_irq = priv->plat->rx_chans_cfg[chan].ch_irq;
	rx_irq_cpu = priv->plat->rx_chans_cfg[chan].ch_irq_cpu;

	snprintf(ch->rx_irq_name, sizeof(ch->rx_irq_name) - 1,
		 "%s-uio-rx-%u", priv->dev->name, chan);

	err = request_irq(rx_irq, xgmac_uio_isr,
			  IRQF_SHARED, ch->rx_irq_name, ch);

	if (unlikely(err < 0)) {
		dev_err(priv->device,
			"%s: ERROR: IRQ %d (error: %d)\n",
			__func__, rx_irq, err);
	} else {
		irq_set_affinity(rx_irq, cpumask_of(rx_irq_cpu));
	}

	return err;
}

static void xgmac_napi_del(struct net_device *dev)
{
	struct xgmac_priv *priv = netdev_priv(dev);
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan, max_ch;

	max_ch = max(priv->plat->rx_chans_to_use, priv->plat->tx_chans_to_use);

	for (chan = 0; chan < max_ch; chan++) {
		struct xgmac_channel *ch = &priv->channel[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		if (chan < priv->plat->rx_chans_to_use)
			netif_napi_del(&ch->rx_napi);
		if (chan < priv->plat->tx_chans_to_use)
			netif_napi_del(&ch->tx_napi);
	}
}

#define SIF_IOSS_IN_EVENT_EN	0x3C
#define SIF_IOSS_IN_EVENT	0x7C
/* IOSS input event for auxiliary snapshot:
 * Link2 lane0 trigger0/1/2/3: bit-14/15/16/17
 * Link2 lane1 trigger0/1: bit-18/19
 * Link2 lane2 trigger0/1: bit-20/21
 * Link2 lane3 trigger0/1: bit-22/23
 * Link1 trigger0/1: bit-24/25
 * Use trigger 0 for all lanes.
 */
#define SIF_IOSS_IN_EVENT_BITS	(BIT(14) | BIT(18) | BIT(20) | BIT(22) | BIT(24))
int xgmac_latch_aux_snapshot(struct xgmac_priv *priv_list[], struct timespec64 ts[])
{
	struct xgmac_priv *priv_b = NULL;
	struct xgmac_priv *priv;
	u32 reg;
	u32 snap_cnt;
	int i;

	/* At least one port is up */
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		priv_b = priv_list[i];
		if (priv_b)
			break;
	}
	if (!priv_b)
		return -ENODEV;

	suspend_aux_snap_read = true;

	/* Enable aux snapshot event. This also disables 1PPS input trigger */
	reg = readl(priv_b->plat->sif_base + SIF_IOSS_IN_EVENT_EN);
	reg |= SIF_IOSS_IN_EVENT_BITS;
	writel(reg, priv_b->plat->sif_base + SIF_IOSS_IN_EVENT_EN);

	/* Flush existing aux snapshots */
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		priv = priv_list[i];
		if (!priv)
			continue;
		if (priv->dma_cap.aux_snap_num == 0)
			continue;

		reg = readl(priv->ioaddr + XGMAC_AUX_CTRL);
		if (reg & XGMAC_ATSEN0) {
			reg = readl(priv->ioaddr + XGMAC_TIMESTAMP_STATUS);
			snap_cnt = (reg & XGMAC_ATSNS) >> XGMAC_ATSNS_SHIFT;
			while (snap_cnt > 0) {
				/* Must read nsec first */
				readl(priv->ioaddr + XGMAC_AUX_SNAP_NSEC);
				readl(priv->ioaddr + XGMAC_AUX_SNAP_SEC);
				snap_cnt--;
			}
		} else {
			reg |= XGMAC_ATSEN0;
			writel(reg, priv->ioaddr + XGMAC_AUX_CTRL);
		}
	}

	/* Trigger one aux snapshot on all interfaces */
	reg = readl(priv_b->plat->sif_base + SIF_IOSS_IN_EVENT);
	reg |= SIF_IOSS_IN_EVENT_BITS;
	writel(reg, priv_b->plat->sif_base + SIF_IOSS_IN_EVENT);
	reg &= ~SIF_IOSS_IN_EVENT_BITS;
	writel(reg, priv_b->plat->sif_base + SIF_IOSS_IN_EVENT);

	/* Read aux snapshot */
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		priv = priv_list[i];
		if (!priv)
			continue;
		if (priv->dma_cap.aux_snap_num == 0)
			continue;

		reg = readl(priv->ioaddr + XGMAC_TIMESTAMP_STATUS);
		snap_cnt = (reg & XGMAC_ATSNS) >> XGMAC_ATSNS_SHIFT;
		if (snap_cnt == 1) {
			/* Must read nsec first */
			ts[i].tv_nsec = readl(priv->ioaddr + XGMAC_AUX_SNAP_NSEC);
			ts[i].tv_sec = readl(priv->ioaddr + XGMAC_AUX_SNAP_SEC);
		} else {
			dev_err(priv->device,
				"Incorrect snap_cnt %d intf %d\n", snap_cnt, i);
		}
	}

	/* Disable aux snapshot event. This also enables 1PPS input trigger */
	reg = readl(priv_b->plat->sif_base + SIF_IOSS_IN_EVENT_EN);
	reg &= ~SIF_IOSS_IN_EVENT_BITS;
	writel(reg, priv_b->plat->sif_base + SIF_IOSS_IN_EVENT_EN);

	suspend_aux_snap_read = false;

	return 0;
}

#ifdef CONFIG_SYSFS
static ssize_t poll_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));

	return sprintf(buf, "%d\n", priv->poll_mode);
}

static ssize_t poll_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));
	int mode;

	if (kstrtoint(buf, 10, &mode))
		return -EINVAL;
	priv->poll_mode = mode;

	return len;
}
static DEVICE_ATTR_RW(poll_mode);

static ssize_t latency_profile_rx_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));

	if (priv->profiling_rx)
		print_latency_profile(priv, PROFILE_RX);

	return sprintf(buf, "%d\n", priv->profiling_rx);
}

static ssize_t latency_profile_rx_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));
	int profiling;

	if (kstrtoint(buf, 10, &profiling))
		return -EINVAL;
	priv->profiling_rx = profiling;

	pf_rx_first_seg_idx = 0;
	pf_rx_last_seg_idx = 0;
	pf_rx_data[0].first_seg_stmr = 0;
	pf_rx_data[0].last_seg_stmr = 0;
	napi_rx_prev_val = 0;
	max_napi_rx_intv = 0;
	min_napi_rx_intv = STMR_2000US;
	num_napi_rx_call = 0;
	sum_napi_rx_intv = 0;
	cal_napi_rx_intv = false;

	return len;
}
static DEVICE_ATTR_RW(latency_profile_rx);

static ssize_t latency_profile_tx_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));

	if (priv->profiling_tx)
		print_latency_profile(priv, PROFILE_TX);

	return sprintf(buf, "%d\n", priv->profiling_tx);
}

static ssize_t latency_profile_tx_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct xgmac_priv *priv = netdev_priv(to_net_dev(dev));
	int profiling;

	if (kstrtoint(buf, 10, &profiling))
		return -EINVAL;
	priv->profiling_tx = profiling;

	pf_tx_first_seg_idx = 0;
	pf_tx_last_seg_idx = 0;
	pf_tx_data[0].first_seg_stmr = 0;
	pf_tx_data[0].last_seg_stmr = 0;

	return len;
}
static DEVICE_ATTR_RW(latency_profile_tx);
#endif

/**
 * xgmac_dvr_probe
 * @device: device pointer
 * @plat_dat: platform data pointer
 * @res: xgmac resource pointer
 * Description: this is the main probe function used to
 * call the alloc_etherdev, allocate the priv structure.
 * Return:
 * returns 0 on success, otherwise errno.
 */
int xgmac_dvr_probe(struct device *device,
		     struct plat_xgmacenet_data *plat_dat,
		     struct xgmac_resources *res)
{
	struct net_device *ndev = NULL;
	struct xgmac_priv *priv;
	u32 pmd_start_chan;
	u32 pmd_end_chan;
	u32 chan, rx_ch, max_ch;
	int speed;
	phys_addr_t base;
	phys_addr_t rx_addr;
	phys_addr_t tx_addr;
	u32 rx_size;
	u32 tx_size;
	int i, ret = 0;

	if (eth0_defer && plat_dat->phy_lane != 0) {
		dev_err(device, "eth0 defered, deferring probe\n");
		return -EPROBE_DEFER;
	}

	ndev = devm_alloc_etherdev_mqs(device, sizeof(struct xgmac_priv),
				       MTL_MAX_TX_QUEUES, MTL_MAX_RX_QUEUES);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;
	priv->plat = plat_dat;
	pmd_start_chan = priv->plat->pmd_start_chan;
	pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;

	/* Get next available net dev name for UIO name. */
	ret = dev_alloc_name(ndev, ndev->name);
	if (ret < 0)
		return ret;

	if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++) {
			ret = xgmac_uio_init(priv, chan);
			if (ret >= 0) {
				if (priv->plat->phy_lane == 0)
					eth0_defer = false;
			} else {
				if (ret == -EPROBE_DEFER &&
				    priv->plat->phy_lane == 0)
					eth0_defer = true;
				return ret;
			}
		}

		/* Initialize UIO for virtual chan */
		ret = edgeq_virt_uio_init(priv->plat->virt_chan_uio_id);
		if (ret) {
			dev_err(priv->device,
				"Virt chan uio init failed\n");
			return -ENODEV;
		}
		ret = edgeq_virt_uio_addr(priv->plat->virt_chan_uio_id,
					  &base, &rx_addr, &rx_size,
					  &tx_addr, &tx_size);
		if (ret) {
			dev_err(priv->device,
				"Get virt chan uio addr failed\n");
			return -ENODEV;
		}
		if ((rx_size / sizeof(struct dma_desc)) < DMA_RX_SIZE) {
			dev_err(priv->device,
				"Virt chan uio Rx ring too small\n");
			return -ENODEV;
		}
		if ((tx_size / sizeof(struct dma_desc)) < DMA_TX_SIZE) {
			dev_err(priv->device,
				"Virt chan uio Tx ring too small\n");
			return -ENODEV;
		}
		priv->virt_chan_info =
				 devm_memremap(device, base + VIRT_INFO_OFFSET,
					       sizeof(struct virt_info),
					       MEMREMAP_WB);
		priv->virt_chan_rx = devm_memremap(device, rx_addr,
						   rx_size, MEMREMAP_WB);
		priv->virt_chan_tx = devm_memremap(device, tx_addr,
						   tx_size, MEMREMAP_WB);
	}

	xgmac_set_ethtool_ops(ndev);
	priv->pause = pause;
	priv->ioaddr = res->xgmac_base;
	priv->xpcs_base = res->xpcs_base;
	priv->ecpri_base = res->ecpri_base;
	if (priv->plat->phy_lane == 0) {
		if (!res->misc_base && !xgbe_is_misc_addr_set()) {
			dev_err(priv->device, "Lane0 Ethernet without misc address\n");
			return -ENODEV;
		} else if (res->misc_base) {
			xgbe_set_misc_addr(res->misc_base);
		}
	}
	priv->misc_base = res->misc_base;
	if (priv->plat->eth_link == 1 && !res->e56_base) {
		dev_err(priv->device, "Link1 Ethernet without E56 address\n");
		return -ENODEV;
	}
	priv->e56_base = res->e56_base;
	priv->ecpri_phy_addr = res->ecpri_phy_addr;
	priv->dev->base_addr = (unsigned long)res->xgmac_base;

	priv->dev->irq = res->irq;
	priv->wol_irq = res->wol_irq;
	priv->lpi_irq = res->lpi_irq;
	priv->pps_irq = res->pps_irq;

	if (!IS_ERR_OR_NULL(res->mac))
		memcpy(priv->dev->dev_addr, res->mac, ETH_ALEN);

	dev_set_drvdata(device, priv->dev);

	/* Verify driver arguments */
	xgmac_verify_args();

	/* Allocate workqueue */
	priv->wq = create_singlethread_workqueue("xgmac_wq");
	if (!priv->wq) {
		dev_err(priv->device, "failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_WORK(&priv->service_task, xgmac_service_task);

	/* Override with kernel parameters if supplied XXX CRS XXX
	 * this needs to have multiple instances
	 */
	if ((phyaddr >= 0) && (phyaddr <= 31))
		priv->plat->phy_addr = phyaddr;

#if 0 // This is set in u-boot
	/* Bring mac memory out of shutdown */
	if (priv->misc_base != NULL) {
		ret = xgbe_set_mac_mem_active(priv);
		if (ret) {
			dev_err(priv->device, "MAC memory not active\n");
			goto error_hw_init;
		}
	}
#endif

	/* Init MAC and get the capabilities */
	ret = xgmac_hw_init(priv);
	if (ret)
		goto error_hw_init;

	speed = priv->plat->mac_port_sel_speed;
	if ((speed == SPEED_50000 || speed == SPEED_25000) && !priv->hw->xlgmac) {
		dev_err(priv->device, "speed and device mismatch\n");
		return -ENODEV;
	}

	/* Only first link need to configure mux */
	if (priv->misc_base != NULL) {
		xgbe_pcs_mux_config(priv);
	}

	xgmac_check_ether_addr(priv);

	priv->plat->rx_queues_to_use = min_t(unsigned int,
					     priv->plat->rx_queues_to_use,
					     priv->dma_cap.number_rx_queues);
	priv->plat->rx_chans_to_use = min_t(unsigned int,
					    priv->plat->rx_chans_to_use,
					    priv->dma_cap.number_rx_channel);
	priv->plat->tx_queues_to_use = min_t(unsigned int,
					     priv->plat->tx_queues_to_use,
					     priv->dma_cap.number_tx_queues);
	priv->plat->tx_chans_to_use = min_t(unsigned int,
					    priv->plat->tx_chans_to_use,
					    priv->dma_cap.number_tx_channel);

	/* Configure real RX and TX queues */
	netif_set_real_num_rx_queues(ndev, priv->plat->rx_chans_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_chans_to_use);

	ndev->netdev_ops = &xgmac_netdev_ops;

	ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM;
	if (priv->plat->tx_coe)
		ndev->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;

	ret = xgmac_tc_init(priv, priv);
	if (!ret) {
		ndev->hw_features |= NETIF_F_HW_TC;
	}

	if ((priv->plat->tso_en) && (priv->dma_cap.tsoen)) {
		ndev->hw_features |= NETIF_F_TSO | NETIF_F_TSO6;
		priv->tso = true;
		dev_info(priv->device, "TSO feature enabled\n");
	}

	if (priv->dma_cap.sphen) {
		ndev->hw_features |= NETIF_F_GRO;
		/* SPH default to disabled. Can be enabled via dts */
		priv->plat->sph_en = false;
		dev_info(priv->device, "SPH feature disabled\n");
	} else {
		priv->plat->sph_en = false;
		dev_info(priv->device, "SPH feature not supported\n");
	}
	priv->sph = priv->plat->sph_en;

	if (priv->dma_cap.addr64) {
		ret = dma_set_mask_and_coherent(device,
				DMA_BIT_MASK(priv->dma_cap.addr64));
		if (!ret) {
			dev_info(priv->device, "Using %d bits DMA width\n",
				 priv->dma_cap.addr64);

			/*
			 * If more than 32 bits can be addressed, make sure to
			 * enable enhanced addressing mode.
			 */
			if (IS_ENABLED(CONFIG_ARCH_DMA_ADDR_T_64BIT))
				priv->plat->dma_cfg->eame = true;
		} else {
			ret = dma_set_mask_and_coherent(device, DMA_BIT_MASK(32));
			if (ret) {
				dev_err(priv->device, "Failed to set DMA Mask\n");
				goto error_hw_init;
			}

			priv->dma_cap.addr64 = 32;
		}
	}

	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
#ifdef XGMAC_VLAN_TAG_USED
	/* Both mac100 and gmac support receive VLAN tag detection */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_STAG_RX;
	if (priv->dma_cap.vlhash) {
		ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
		ndev->features |= NETIF_F_HW_VLAN_STAG_FILTER;
	}
	if (priv->dma_cap.vlins) {
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
		if (priv->dma_cap.dvlan)
			ndev->hw_features |= NETIF_F_HW_VLAN_STAG_TX;
	}
#endif
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;

	priv->msg_enable = netif_msg_init(debug, default_msg_level);

	/* Initialize RSS */
	rx_ch = priv->plat->rx_chans_to_use;
	netdev_rss_key_fill(priv->rss.key, sizeof(priv->rss.key));
	for (i = 0; i < ARRAY_SIZE(priv->rss.table); i++)
		priv->rss.table[i] = ethtool_rxfh_indir_default(i, rx_ch);
	if (priv->hw->xlgmac)
		priv->rss.table_size = XGMAC_RSS_MIN_TABLE_SIZE;
	else
		priv->rss.table_size = XGMAC_RSS_MAX_TABLE_SIZE;

	if (priv->dma_cap.rssen && priv->plat->rss_en)
		ndev->features |= NETIF_F_RXHASH;

	/* MTU range: 46 - hw-specific max */
	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;
	if (priv->plat->has_xgmac)
		ndev->max_mtu = XGMAC_JUMBO_LEN;
	else if (priv->plat->enh_desc)
		ndev->max_mtu = JUMBO_LEN;
	else
		ndev->max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);
	/* Will not overwrite ndev->max_mtu if plat->maxmtu > ndev->max_mtu
	 * as well as plat->maxmtu < ndev->min_mtu which is a invalid range.
	 */
	if ((priv->plat->maxmtu < ndev->max_mtu) &&
	    (priv->plat->maxmtu >= ndev->min_mtu))
		ndev->max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < ndev->min_mtu)
		dev_warn(priv->device,
			 "%s: warning: maxmtu having invalid value (%d)\n",
			 __func__, priv->plat->maxmtu);

	if (flow_ctrl)
		priv->flow_ctrl = FLOW_AUTO;	/* RX/TX pause on */

	/* Setup channels NAPI */
	max_ch = max(priv->plat->rx_chans_to_use, priv->plat->tx_chans_to_use);

	for (chan = 0; chan < max_ch; chan++) {
		struct xgmac_channel *ch = &priv->channel[chan];

		spin_lock_init(&ch->lock);
		ch->priv_data = priv;
		ch->index = chan;
		ch->rx_dma_stopped = false;

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		if (chan < priv->plat->rx_chans_to_use) {
			/* Separate NAPI Rx poll function for eCPRI */
			if (priv->plat->ecpri_en && chan == 0) {
				netif_napi_add(ndev, &ch->rx_napi,
					       ecpri_napi_poll_rx,
					       NAPI_POLL_WEIGHT);
			} else {
				netif_napi_add(ndev, &ch->rx_napi,
					       xgmac_napi_poll_rx,
					       NAPI_POLL_WEIGHT);
			}
			ch->rx_irq = priv->plat->rx_chans_cfg[chan].ch_irq;
			ch->rx_irq_cpu = priv->plat->rx_chans_cfg[chan].ch_irq_cpu;
		}
		if (chan < priv->plat->tx_chans_to_use) {
			/* Separate NAPI Tx poll function for eCPRI */
			u32 num_ecpri_ch = 0;
			if ((priv->plat->eth_link == 1 ||
			     priv->plat->eth_link == 2) &&
			    priv->plat->ecpri_en)
				num_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
			else
				num_ecpri_ch = 1;
			if (priv->plat->ecpri_en && chan < num_ecpri_ch) {
				netif_tx_napi_add(ndev, &ch->tx_napi,
						  ecpri_napi_poll_tx,
						  NAPI_POLL_WEIGHT);
			} else {
				netif_tx_napi_add(ndev, &ch->tx_napi,
						  xgmac_napi_poll_tx,
						  NAPI_POLL_WEIGHT);
			}
			ch->tx_irq = priv->plat->tx_chans_cfg[chan].ch_irq;
			ch->tx_irq_cpu = priv->plat->tx_chans_cfg[chan].ch_irq_cpu;
		}
	}

	mutex_init(&priv->lock);

	/* If a specific clk_csr value is passed from the platform
	 * this means that the CSR Clock Range selection cannot be
	 * changed at run-time and it is fixed. Viceversa the driver'll try to
	 * set the MDC clock dynamically according to the csr actual
	 * clock input.
	 */
	if (priv->plat->clk_csr >= 0)
		priv->clk_csr = priv->plat->clk_csr;
	else
		xgmac_clk_csr_set(priv);

	if (priv->plat->has_xpcs)
		priv->hw->xpcs = xpcs_get_ops(priv);
	else
		priv->hw->xpcs = xpcs_get_ops(0);
	if (!priv->hw->xpcs) {
		netdev_err(ndev, "failed to get xpcs ops\n");
		goto error_xpcs;
	}
	ret = xgmac_xpcs_probe(priv, priv->xpcs_base);
	if (ret < 0) {
		if (ret == -ENODEV) {
			netdev_info(priv->dev, "no xpcs device\n");
		}
		else {
			netdev_err(ndev, "failed to probe xpcs device\n");
			goto error_xpcs;
		}
	}

	/* MDIO bus Registration */
	ret = xgmac_mdio_register(ndev);
	if (ret < 0) {
		dev_err(priv->device,
			"%s: MDIO bus (id: %d) registration failed",
			__func__, priv->plat->bus_id);
		goto error_mdio_register;
	}

	ret = xgmac_phy_setup(priv);
	if (ret) {
		dev_err(priv->device, "failed to setup phy (%d)\n", ret);
		goto error_phy_setup;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(priv->device, "%s: ERROR %i registering the device\n",
			__func__, ret);
		goto error_netdev_register;
	}

	if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++) {
			ret = xgmac_uio_irq_init(priv, chan);
			if (ret < 0)
				return ret;
		}
	}

	if (priv->plat->serdes_powerup) {
		ret = priv->plat->serdes_powerup(ndev,
						 priv->plat->bsp_priv);

		if (ret < 0)
			goto error_serdes_powerup;
	}

	priv->poll_mode = 0;
	priv->profiling_rx = 0;
	priv->profiling_tx = 0;
#ifdef CONFIG_SYSFS
	/* Register sysfs attribute */
	ret = device_create_file(&ndev->dev, &dev_attr_poll_mode);
	ret = device_create_file(&ndev->dev, &dev_attr_latency_profile_rx);
	ret = device_create_file(&ndev->dev, &dev_attr_latency_profile_tx);
#endif

#ifdef CONFIG_DEBUG_FS
	xgmac_init_fs(ndev);
#endif
	suspend_aux_snap_read = false;

	return ret;

error_serdes_powerup:
	if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++) {
			free_irq(priv->channel[chan].rx_irq, &priv->channel[chan]);
			uio_unregister_device(&priv->uio_dev[chan].uinfo);
		}
	}
	unregister_netdev(ndev);
error_netdev_register:
	phylink_destroy(priv->phylink);
error_phy_setup:
error_xpcs:
	for (chan = 0; chan < max_ch; chan++) {
		struct xgmac_channel *ch = &priv->channel[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		if (chan < priv->plat->rx_chans_to_use)
			netif_napi_del(&ch->rx_napi);
		if (chan < priv->plat->tx_chans_to_use)
			netif_napi_del(&ch->tx_napi);
	}
error_mdio_register:
	xgmac_napi_del(ndev);
error_hw_init:
	destroy_workqueue(priv->wq);

	return ret;
}
EXPORT_SYMBOL_GPL(xgmac_dvr_probe);

/**
 * xgmac_dvr_remove
 * @dev: device pointer
 * Description: this function resets the TX/RX processes, disables the MAC RX/TX
 * changes the link status, releases the DMA descriptor rings.
 */
int xgmac_dvr_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	netdev_info(priv->dev, "%s: removing driver", __func__);

	xgmac_stop_all_dma(priv);

	if (priv->plat->pmd_enabled) {
		for (chan = pmd_start_chan; chan < pmd_end_chan; chan++) {
			free_irq(priv->channel[chan].rx_irq, &priv->channel[chan]);
			uio_unregister_device(&priv->uio_dev[chan].uinfo);
		}
	}

	if (priv->plat->serdes_powerdown)
		priv->plat->serdes_powerdown(ndev, priv->plat->bsp_priv);

	xgmac_mac_set(priv, priv->ioaddr, false);
	netif_carrier_off(ndev);
#ifdef CONFIG_SYSFS
	device_remove_file(&ndev->dev, &dev_attr_poll_mode);
	device_remove_file(&ndev->dev, &dev_attr_latency_profile_rx);
	device_remove_file(&ndev->dev, &dev_attr_latency_profile_tx);
#endif
	unregister_netdev(ndev);
#ifdef CONFIG_DEBUG_FS
	xgmac_exit_fs(ndev);
#endif
	phylink_destroy(priv->phylink);
	clk_disable_unprepare(priv->plat->pclk);
	clk_disable_unprepare(priv->plat->xgmac_clk);
	destroy_workqueue(priv->wq);
	mutex_destroy(&priv->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(xgmac_dvr_remove);

/**
 * xgmac_suspend - suspend callback
 * @dev: device pointer
 * Description: this is the function to suspend the device and it is called
 * by the platform driver to stop the network queue, release the resources,
 * program the PMT register (for WoL), clean and release driver resources.
 */
int xgmac_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;

	if (!ndev || !netif_running(ndev))
		return 0;

	phylink_mac_change(priv->phylink, false);

	if (priv->plat->ecpri_en)
		ecpri_stop(priv);

	mutex_lock(&priv->lock);

	netif_device_detach(ndev);
	xgmac_stop_all_queues(priv);

	xgmac_disable_all_chans(priv);

	for (chan = 0; chan < priv->plat->tx_chans_to_use; chan++) {
		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;
		del_timer_sync(&priv->tx_chan[chan].txtimer);
	}

	if (priv->plat->virt_chan_en)
		hrtimer_cancel(&priv->virt_ch_timer);

	/* Stop TX/RX DMA */
	xgmac_stop_all_dma(priv);

	if (priv->plat->serdes_powerdown)
		priv->plat->serdes_powerdown(ndev, priv->plat->bsp_priv);

	/* Enable Power down mode by programming the PMT regs */
	if (device_may_wakeup(priv->device)) {
		xgmac_pmt(priv, priv->hw, priv->wolopts);
		priv->irq_wake = 1;
	} else {
		mutex_unlock(&priv->lock);
		rtnl_lock();
		phylink_stop(priv->phylink);
		rtnl_unlock();
		mutex_lock(&priv->lock);

		xgmac_mac_set(priv, priv->ioaddr, false);
		pinctrl_pm_select_sleep_state(priv->device);
		/* Disable clock in case of PWM is off */
		if (priv->plat->clk_ptp_ref)
			clk_disable_unprepare(priv->plat->clk_ptp_ref);
		clk_disable_unprepare(priv->plat->pclk);
		clk_disable_unprepare(priv->plat->xgmac_clk);
	}
	mutex_unlock(&priv->lock);

	priv->speed = SPEED_UNKNOWN;
	return 0;
}
EXPORT_SYMBOL_GPL(xgmac_suspend);

/**
 * xgmac_reset_chans_param - reset channel parameters
 * @dev: device pointer
 */
static void xgmac_reset_chans_param(struct xgmac_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_chans_to_use;
	u32 tx_cnt = priv->plat->tx_chans_to_use;
	u32 pmd_start_chan = priv->plat->pmd_start_chan;
	u32 pmd_end_chan = pmd_start_chan + priv->plat->pmd_num_chans;
	u32 chan;
	u32 num_rx_ecpri_ch = 0;
	u32 num_tx_ecpri_ch = 0;

	/* eCPRI SS manages eCPRI chan */
	if (priv->plat->ecpri_en) {
		num_rx_ecpri_ch = 1;
		if (priv->plat->eth_link == 1 || priv->plat->eth_link == 2)
			num_tx_ecpri_ch = MIN_TX_CHANS_WITH_ECPRI_L1 - 1;
		else
			num_tx_ecpri_ch = 1;
	}

	for (chan = num_rx_ecpri_ch; chan < rx_cnt; chan++) {
		struct xgmac_rx_chan *rx_ch = &priv->rx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		rx_ch->cur_rx = 0;
		rx_ch->dirty_rx = 0;
	}

	for (chan = num_tx_ecpri_ch; chan < tx_cnt; chan++) {
		struct xgmac_tx_chan *tx_ch = &priv->tx_chan[chan];

		if (priv->plat->pmd_enabled &&
		    chan >= pmd_start_chan && chan < pmd_end_chan)
			continue;

		tx_ch->cur_tx = 0;
		tx_ch->dirty_tx = 0;
		tx_ch->mss = 0;
	}
}

/**
 * xgmac_resume - resume callback
 * @dev: device pointer
 * Description: when resume this function is invoked to setup the DMA and CORE
 * in a usable state.
 */
int xgmac_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct xgmac_priv *priv = netdev_priv(ndev);
	int ret;

	if (!netif_running(ndev))
		return 0;

	/* Power Down bit, into the PM register, is cleared
	 * automatically as soon as a magic packet or a Wake-up frame
	 * is received. Anyway, it's better to manually clear
	 * this bit because it can generate problems while resuming
	 * from another devices (e.g. serial console).
	 */
	if (device_may_wakeup(priv->device)) {
		mutex_lock(&priv->lock);
		xgmac_pmt(priv, priv->hw, 0);
		mutex_unlock(&priv->lock);
		priv->irq_wake = 0;
	} else {
		pinctrl_pm_select_default_state(priv->device);
		/* enable the clk previously disabled */
		clk_prepare_enable(priv->plat->xgmac_clk);
		clk_prepare_enable(priv->plat->pclk);
		if (priv->plat->clk_ptp_ref)
			clk_prepare_enable(priv->plat->clk_ptp_ref);
		/* reset the phy so that it's ready */
		if (priv->mii)
			xgmac_mdio_reset(priv->mii);
	}

	if (priv->plat->serdes_powerup) {
		ret = priv->plat->serdes_powerup(ndev,
						 priv->plat->bsp_priv);

		if (ret < 0)
			return ret;
	}

	mutex_lock(&priv->lock);

	xgmac_reset_chans_param(priv);

	xgmac_clear_descriptors(priv);

	xgmac_init_coalesce(priv);
	xgmac_hw_setup(ndev, false);
	xgmac_set_rx_mode(ndev);

	xgmac_restore_hw_vlan_rx_fltr(priv, ndev, priv->hw);

	xgmac_enable_all_chans(priv);
	xgmac_start_all_queues(priv);

	if (priv->plat->ecpri_en)
		ecpri_start(priv);

	mutex_unlock(&priv->lock);

	if (!device_may_wakeup(priv->device)) {
		rtnl_lock();
		phylink_start(priv->phylink);
		rtnl_unlock();
	}

	phylink_mac_change(priv->phylink, true);

	netif_device_attach(ndev);

	return 0;
}
EXPORT_SYMBOL_GPL(xgmac_resume);

#ifndef MODULE
static int __init xgmac_cmdline_opt(char *str)
{
	char *opt;

	if (!str || !*str)
		return -EINVAL;
	while ((opt = strsep(&str, ",")) != NULL) {
		if (!strncmp(opt, "debug:", 6)) {
			if (kstrtoint(opt + 6, 0, &debug))
				goto err;
		} else if (!strncmp(opt, "phyaddr:", 8)) {
			if (kstrtoint(opt + 8, 0, &phyaddr))
				goto err;
		} else if (!strncmp(opt, "buf_sz:", 7)) {
			if (kstrtoint(opt + 7, 0, &buf_sz))
				goto err;
		} else if (!strncmp(opt, "tc:", 3)) {
			if (kstrtoint(opt + 3, 0, &tc))
				goto err;
		} else if (!strncmp(opt, "watchdog:", 9)) {
			if (kstrtoint(opt + 9, 0, &watchdog))
				goto err;
		} else if (!strncmp(opt, "flow_ctrl:", 10)) {
			if (kstrtoint(opt + 10, 0, &flow_ctrl))
				goto err;
		} else if (!strncmp(opt, "pause:", 6)) {
			if (kstrtoint(opt + 6, 0, &pause))
				goto err;
		} else if (!strncmp(opt, "eee_timer:", 10)) {
			if (kstrtoint(opt + 10, 0, &eee_timer))
				goto err;
		} else if (!strncmp(opt, "chain_mode:", 11)) {
			if (kstrtoint(opt + 11, 0, &chain_mode))
				goto err;
		}
	}
	return 0;

err:
	pr_err("%s: ERROR broken module parameter conversion", __func__);
	return -EINVAL;
}

__setup("xgmaceth=", xgmac_cmdline_opt);
#endif /* MODULE */

static int __init xgmac_init(void)
{
	int ret;

#ifdef CONFIG_DEBUG_FS
	/* Create debugfs main directory if it doesn't exist yet */
	if (!xgmac_fs_dir)
		xgmac_fs_dir = debugfs_create_dir(XGMAC_RESOURCE_NAME, NULL);
	register_netdevice_notifier(&xgmac_notifier);
#endif
	eth0_defer = false;

	ret = xgbe_driver_init();
	if (ret)
		goto err_driver_init;

	return 0;

err_driver_init:
#ifdef CONFIG_DEBUG_FS
	unregister_netdevice_notifier(&xgmac_notifier);
	debugfs_remove_recursive(xgmac_fs_dir);
#endif
	return ret;
}

static void __exit xgmac_exit(void)
{
	xgbe_driver_exit();
#ifdef CONFIG_DEBUG_FS
	unregister_netdevice_notifier(&xgmac_notifier);
	debugfs_remove_recursive(xgmac_fs_dir);
#endif
}

module_init(xgmac_init)
module_exit(xgmac_exit)

MODULE_DESCRIPTION("EdgeQ 10Gb Ethernet device driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
