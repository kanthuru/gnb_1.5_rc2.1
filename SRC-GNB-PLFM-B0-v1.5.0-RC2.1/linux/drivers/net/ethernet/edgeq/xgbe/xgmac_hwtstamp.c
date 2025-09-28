// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  Copyright (C) 2021

  This implements all the API for managing HW timestamp & PTP.

*******************************************************************************/

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include "common.h"
#include "xgmac_ptp.h"

static void config_hw_tstamping(void __iomem *ioaddr, u32 data)
{
	writel(data, ioaddr + PTP_TCR);
}

static void config_sub_second_increment(void __iomem *ioaddr,
		u32 ptp_clock, u32 *ssinc)
{
	u64 temp;
	u32 reg;

	/* For XGMAC version, in "fine adjustement mode" set sub-second
	 * increment to twice the number of nanoseconds of a clock cycle. The
	 * calculation of the default_addend value by the caller will set it
	 * to mid-range = 2^31 when the remainder of this division is zero,
	 * which will make the accumulator overflow once every 2 ptp_clock
	 * cycles, adding twice the number of nanoseconds of a clock cycle :
	 * 2000000000ULL / ptp_clock.
	 */
	reg = readl(ioaddr + PTP_TCR);
	if (reg & PTP_TCR_TSCFUPDT)
		temp = (2000000000ULL / ptp_clock);
	else
		temp = (1000000000ULL / ptp_clock);

	/* 0.465ns accuracy */
	if (!(reg & PTP_TCR_TSCTRLSSR))
		temp = (temp * 1000) / 465;

	reg = (temp << PTP_SSIR_SSINC_SHIFT) & PTP_SSIR_SSINC_MASK;
	/* SoC fractional PLL_divider = 48.(82587492465973 / 2^24)
	 * SoC_frequency = 122.88 MHz / 4 * PLL_divider
	 * XGMAC_clock = SoC_frequency / 2
	 * In XGMAC setting, we need to select SSINC:SNSINC and ADDEND
	 * to have minimum frequency error respecting to SoC_clock:
	 * XGMAC_clock = 1000 / ((addend / 2^32) * (ssinc + snsinc / 256))
	 * Based on above equations, the ssinc:snsinc value is 0x1:0x6B.
	 */
	reg = 0x16B00;
	writel(reg, ioaddr + PTP_SSIR);

	if (ssinc)
		*ssinc = temp;
}

static int init_systime(void __iomem *ioaddr, u32 sec, u32 nsec)
{
	u32 value;

	writel(sec, ioaddr + PTP_STSUR);
	writel(nsec, ioaddr + PTP_STNSUR);
	/* issue command to initialize the system time value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSINIT;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present system time initialize to complete */
	return readl_poll_timeout(ioaddr + PTP_TCR, value,
				 !(value & PTP_TCR_TSINIT),
				 10000, 100000);
}

static int config_addend(void __iomem *ioaddr, u32 addend)
{
	u32 value;
	int limit;

	/* SoC fractional PLL_divider = 48.(82587492465973 / 2^24)
	 * SoC_frequency = 122.88 MHz / 4 * PLL_divider
	 * XGMAC_clock = SoC_frequency / 2
	 * In XGMAC setting, we need to select SSINC:SNSINC and ADDEND
	 * to have minimum frequency error respecting to SoC_clock:
	 * XGMAC_clock = 1000 / ((addend / 2^32) * (ssinc + snsinc / 256))
	 * Based on above equations, the addend value is 0xF0BB241D.
	 */
	addend = 0xF0BB241D;
	writel(addend, ioaddr + PTP_TAR);
	/* issue command to update the addend value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSADDREG;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present addend update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSADDREG))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static int adjust_systime(void __iomem *ioaddr, u32 sec, u32 nsec, int add_sub)
{
	u32 value;
	int limit;

	if (add_sub) {
		/* If the new sec value needs to be subtracted with
		 * the system time, then MAC_STSUR reg should be
		 * programmed with (2^32 – <new_sec_value>)
		 */
		sec = -sec;

		value = readl(ioaddr + PTP_TCR);
		if (value & PTP_TCR_TSCTRLSSR)
			nsec = (PTP_DIGITAL_ROLLOVER_MODE - nsec);
		else
			nsec = (PTP_BINARY_ROLLOVER_MODE - nsec);
	}

	writel(sec, ioaddr + PTP_STSUR);
	value = (add_sub << PTP_STNSUR_ADDSUB_SHIFT) | nsec;
	writel(value, ioaddr + PTP_STNSUR);

	/* issue command to initialize the system time value */
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSUPDT;
	writel(value, ioaddr + PTP_TCR);

	/* wait for present system time adjust/update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSUPDT))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static void get_systime(void __iomem *ioaddr, u64 *systime)
{
	u64 ns;

	/* Get the TSSS value */
	ns = readl(ioaddr + PTP_STNSR);
	/* Get the TSS and convert sec time value to nanosecond */
	ns += readl(ioaddr + PTP_STSR) * 1000000000ULL;

	if (systime)
		*systime = ns;
}

/* This function is for sycn the time beteen two interfaces, mainly for
 * boundary clock, sycn the time of master port to slave port.
 */
static int sync_systime(void __iomem *dst_ioaddr, struct timespec64 *src_ts,
			struct timespec64 *dst_ts)
{
	u32 sec_diff;
	u32 nsec_diff;
	int neg_adj = 0;

	/* Get the time difference */
	if (src_ts->tv_sec > dst_ts->tv_sec) {
		sec_diff = src_ts->tv_sec - dst_ts->tv_sec;
		if (src_ts->tv_nsec >= dst_ts->tv_nsec) {
			nsec_diff = src_ts->tv_nsec - dst_ts->tv_nsec;
		} else {
			nsec_diff = src_ts->tv_nsec + 1000000000 - dst_ts->tv_nsec;
			sec_diff -= 1;
		}
	} else if (src_ts->tv_sec == dst_ts->tv_sec) {
		sec_diff = 0;
		if (src_ts->tv_nsec >= dst_ts->tv_nsec) {
			nsec_diff = src_ts->tv_nsec - dst_ts->tv_nsec;
		} else {
			nsec_diff = dst_ts->tv_nsec - src_ts->tv_nsec;
			neg_adj = 1;
		}
	} else {
		sec_diff = dst_ts->tv_sec - src_ts->tv_sec;
		if (dst_ts->tv_nsec >= src_ts->tv_nsec) {
			nsec_diff = dst_ts->tv_nsec - src_ts->tv_nsec;
		} else {
			nsec_diff = dst_ts->tv_nsec + 1000000000 - src_ts->tv_nsec;
			sec_diff -= 1;
		}
		neg_adj = 1;
	}

	/* Update the time of destination port */
	return adjust_systime(dst_ioaddr, sec_diff, nsec_diff, neg_adj);
}

const struct xgmac_hwtimestamp xgmac_ptp = {
	.config_hw_tstamping = config_hw_tstamping,
	.init_systime = init_systime,
	.config_sub_second_increment = config_sub_second_increment,
	.config_addend = config_addend,
	.adjust_systime = adjust_systime,
	.get_systime = get_systime,
	.sync_systime = sync_systime,
};
