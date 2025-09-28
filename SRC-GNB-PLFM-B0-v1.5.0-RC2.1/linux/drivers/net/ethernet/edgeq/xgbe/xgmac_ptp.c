// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  PTP 1588 clock using the XGMAC.

*******************************************************************************/
#include "xgbe.h"
#include "xgbe_misc.h"
#include "xgmac_ptp.h"

int (*adjust_freq_cb)(struct ptp_clock_info *, s32);
struct xgmac_priv *ptp_priv_list[PTP_MAX_NUM_PORTS] = {
				NULL, NULL, NULL, NULL, NULL};

/**
 * xgmac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 */

/**
 * This function will adjust the frequency of external VCTCXO
 *
 * +/-6.25ppm pull for a control voltage range .5V to 2.5V (center voltage 1.5V)
 * with 11 bit DAC.
 *
 * Also giving Code_Value 0 is fastest and Code_Value 0x7FF is slowest,
 * the equation for calculating DAC is:
 *
 * 	ppm ~= (2.5 - (Code_Value * (2V / 2048)) * 6.25 - 9.375
 * OR:
 * 	ppm ~= (1 - (Code_Value * (2V / 2048)) * 6.25
 *
 * The equation can be simplified to:
 *	Code_Value = 1024 - ppm * 2048 / 12.5
 * OR:
 *	Code_Value = 1024 - ppb * 2048 / 12500
 */
static int xgmac_adjust_freq_vctcxo(struct ptp_clock_info *ptp, s32 ppb)
{
	struct xgmac_priv *priv =
			container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	s64 adj, temp;

	temp = ppb;
	temp = temp * 2048 / 12500;
	adj = XGBE_DAC_MID_VAL - temp;
	if (adj < XGBE_DAC_MIN_VAL)
		adj = XGBE_DAC_MIN_VAL;
	else if (adj > XGBE_DAC_MAX_VAL)
		adj = XGBE_DAC_MAX_VAL;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	xgbe_aux_dac_write(priv, adj);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static int xgmac_adjust_freq_dctcxo(struct ptp_clock_info *ptp, s32 ppb)
{
	struct xgmac_priv *priv =
			container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	dev_dbg(priv->device, "[%s] ppb: %d\n", __func__, ppb);
	return si5357_set_freq(ppb);
}

/**
 * This function will adjust the frequency of hardware clock in XGMAC.
 */
static int xgmac_adjust_freq_local_clk(struct ptp_clock_info *ptp, s32 ppb)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	spin_lock_irqsave(&priv->ptp_lock, flags);
	xgmac_config_addend(priv, priv->ptpaddr, addend);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static int xgmac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	return (adjust_freq_cb == NULL) ? -EFAULT : adjust_freq_cb(ptp, ppb);
}

static int xgmac_init_intf_time(struct ptp_clock_info *ptp,
				const struct timespec64 *ts)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	int i;

	/* Init HW clock of master port */
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		struct xgmac_priv *priv_l = ptp_priv_list[i];

		if (!priv_l || priv_l == priv)
			continue;
		spin_lock_irqsave(&priv_l->ptp_lock, flags);
		xgmac_init_systime(priv_l, priv_l->ptpaddr, ts->tv_sec, ts->tv_nsec);
		spin_unlock_irqrestore(&priv_l->ptp_lock, flags);
	}

	return 0;
}

static int xgmac_sync_intf_time(struct ptp_clock_info *ptp)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	struct timespec64 ts[PTP_MAX_NUM_PORTS];
	int src_ind = 0;
	int i;

	/* Update HW clock of master port */
	xgmac_latch_aux_snapshot(ptp_priv_list, ts);
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		struct xgmac_priv *priv_l = ptp_priv_list[i];

		if (priv_l == priv) {
			src_ind = i;
			break;
		}
	}
	for (i = 0; i < PTP_MAX_NUM_PORTS; i++) {
		struct xgmac_priv *priv_l = ptp_priv_list[i];

		if (!priv_l || priv_l == priv)
			continue;
		spin_lock_irqsave(&priv_l->ptp_lock, flags);
		xgmac_sync_systime(priv_l, priv_l->ptpaddr, &ts[src_ind], &ts[i]);
		spin_unlock_irqrestore(&priv_l->ptp_lock, flags);
	}

	return 0;
}

/**
 * xgmac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int xgmac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	xgmac_adjust_systime(priv, priv->ptpaddr, sec, nsec, neg_adj);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	/* Update HW clock of master port */
	xgmac_sync_intf_time(ptp);

	return 0;
}

/**
 * xgmac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int xgmac_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns = 0;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	xgmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/**
 * xgmac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int xgmac_set_time(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	unsigned long flags;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	xgmac_init_systime(priv, priv->ptpaddr, ts->tv_sec, ts->tv_nsec);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	/* Update HW clock of master port */
	xgmac_init_intf_time(ptp, ts);
	xgmac_sync_intf_time(ptp);

	return 0;
}

static int xgmac_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	struct xgmac_priv *priv =
	    container_of(ptp, struct xgmac_priv, ptp_clock_ops);
	struct xgmac_pps_cfg *cfg;
	int ret = -EOPNOTSUPP;
//	unsigned long flags;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		/* Reject requests with unsupported flags */
		if (rq->perout.flags)
			return -EOPNOTSUPP;

		cfg = &priv->pps[rq->perout.index];

		cfg->start.tv_sec = rq->perout.start.sec;
		cfg->start.tv_nsec = rq->perout.start.nsec;
		cfg->period.tv_sec = rq->perout.period.sec;
		cfg->period.tv_nsec = rq->perout.period.nsec;

#if 0 // fixed pps enabled during xgmac init
		spin_lock_irqsave(&priv->ptp_lock, flags);
		ret = xgmac_flex_pps_config(priv, priv->ioaddr,
					     rq->perout.index, cfg, on,
					     priv->sub_second_inc,
					     priv->systime_flags);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
#endif
		break;
	default:
		break;
	}

	return ret;
}

/* structure describing a PTP hardware clock */
static struct ptp_clock_info xgmac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "xgbe ptp",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0, /* will be overwritten in xgmac_ptp_register */
	.n_pins = 0,
	.pps = 0,
	.adjfreq = xgmac_adjust_freq,
	.adjtime = xgmac_adjust_time,
	.gettime64 = xgmac_get_time,
	.settime64 = xgmac_set_time,
	.enable = xgmac_enable,
};

/**
 * xgmac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void xgmac_ptp_register(struct xgmac_priv *priv)
{
	int i;

	for (i = 0; i < priv->dma_cap.pps_out_num; i++) {
		if (i >= XGMAC_PPS_MAX)
			break;
		priv->pps[i].available = true;
	}

	if (priv->plat->ptp_max_adj)
		xgmac_ptp_clock_ops.max_adj = priv->plat->ptp_max_adj;

	xgmac_ptp_clock_ops.n_per_out = priv->dma_cap.pps_out_num;

	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = xgmac_ptp_clock_ops;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     priv->device);
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->dev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
	}
//	else if (priv->ptp_clock)
//		netdev_info(priv->dev, "registered PTP clock\n");

	/* Register adjust_freq callback */
	netdev_info(priv->dev, "Register adjust_freq callback\n");
	adjust_freq_cb = NULL;
	switch (priv->plat->soc_clk_src) {
	case SoC_CLK_VCTCXO:
		adjust_freq_cb = xgmac_adjust_freq_vctcxo;
		break;
	case SoC_CLK_DCTCXO:
		adjust_freq_cb = xgmac_adjust_freq_dctcxo;
		break;
	case SoC_CLK_LOCAL_CLK:
		adjust_freq_cb = xgmac_adjust_freq_local_clk;
		break;
	default:
		netdev_err(priv->dev,
			"Invalid SoC clock source: %u\n", priv->plat->soc_clk_src);
		break;
	}
}

/**
 * xgmac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void xgmac_ptp_unregister(struct xgmac_priv *priv)
{
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		pr_debug("Removed PTP HW clock successfully on %s\n",
			 priv->dev->name);
	}
}
