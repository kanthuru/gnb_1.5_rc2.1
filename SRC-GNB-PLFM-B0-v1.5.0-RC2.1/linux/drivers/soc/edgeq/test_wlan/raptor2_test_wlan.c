/*
 * EdgeQ Inc.
 *
 * Raptor2 Parse Device Tree for Micro-engines
 */
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/mm.h>

static bool raptor2_init_dt(void)
{
	bool ret;
	int i, j, size, irqline;
	struct device_node *np, *rn;
	int rx_pkt_int[4], tx_status_int[4];
	int rx_status_int, tx_pkt_int, tx_acctg_int;
	int cci_cmd_int, cci_evt_int, fatal_err_int;

	np = of_find_compatible_node(NULL, NULL, "arm,edgeq-raptor2,wlan");
	if(!np) {
		pr_err("%s:of_find_compatible_node for arm,edgeq-raptor2,wlan failed\n", __FUNCTION__);
		return false;
	}

	pr_info("%s: Found node %s\n", __FUNCTION__, np->name);

	cci_cmd_int = irq_of_parse_and_map(np, 0);
	if(!cci_cmd_int) {
		pr_err("%s: Unable to map interrupt cci_cmd_int for %s\n", __FUNCTION__, np->name);
		return false;
	}

	pr_info("%s: cci_cmd_int: %d\n",  __FUNCTION__, cci_cmd_int);

	cci_evt_int = irq_of_parse_and_map(np, 1);
	if(!cci_evt_int) {
		pr_err("%s: Unable to map interrupt cci_evt_int for %s\n", __FUNCTION__, np->name);
		return false;
	}

	pr_info("%s: cci_evt_int: %d\n",  __FUNCTION__, cci_evt_int);

	fatal_err_int = irq_of_parse_and_map(np, 2);
	if(!fatal_err_int) {
		pr_err("%s: Unable to map interrupt fatal_err_int for %s\n", __FUNCTION__, np->name);
		return false;
	}

	pr_info("%s: fatal_err_int: %d\n",  __FUNCTION__, fatal_err_int);

	rn = of_get_next_child(np, NULL);
	if(!rn) {
		pr_err("%s:of_get_next_child failed for %s failed\n", __FUNCTION__, np->name);
		return false;
	}

	pr_info("%s: Found child node %s\n", __FUNCTION__, rn->name);

	for(i=0; i<4; i++) {
		irqline = irq_of_parse_and_map(rn, i);
		if(!irqline) {
			pr_err("%s: Unable to map interrupt rx_pkt_int%d for %s\n", __FUNCTION__, i, rn->name);
			return false;
		}
		rx_pkt_int[i] = irqline;
		pr_info("%s: rx_pkt_int%d: %d\n", __FUNCTION__, i, rx_pkt_int[i]);
	}

	rx_status_int = irq_of_parse_and_map(rn, i);
	if(!rx_status_int) {
		pr_err("%s: Unable to map interrupt rx_status_int for %s\n", __FUNCTION__, rn->name);
		return false;
	}

	pr_info("%s: rx_status_int: %d\n",  __FUNCTION__, rx_status_int);

	++i;
	tx_pkt_int = irq_of_parse_and_map(rn, i);
	if(!tx_pkt_int) {
		pr_err("%s: Unable to map interrupt tx_pkt_int for %s\n", __FUNCTION__, rn->name);
		return false;
	}

	pr_info("%s: tx_pkt_int: %d\n",  __FUNCTION__, tx_pkt_int);

	++i;

	j = i;
	for(i=0; i<4; i++) {
		irqline = irq_of_parse_and_map(rn, j+i);
		if(!irqline) {
			pr_err("%s: Unable to map interrupt tx_status_int%d for %s\n", __FUNCTION__, i, rn->name);
			return false;
		}
		tx_status_int[i] = irqline;
		pr_info("%s: tx_status_int%d: %d\n", __FUNCTION__, i, tx_status_int[i]);
	}

	i = j+4;
	tx_acctg_int = irq_of_parse_and_map(rn, i);
	if(!tx_acctg_int) {
		pr_err("%s: Unable to map interrupt tx_acctg_int for %s\n", __FUNCTION__, rn->name);
		return false;
	}

	pr_info("%s: tx_acctg_int: %d\n",  __FUNCTION__, tx_acctg_int);
	return true;
}

static int __init raptor2_test_wlan_init(void)
{
	int ret;
	printk(KERN_NOTICE "%s: calling raptor2_init_dt\n", __FUNCTION__);
	ret = raptor2_init_dt();
	if(ret == false) {
		pr_err("%s: raptor2_init_dt() failed\n", __FUNCTION__);
		return ret;
	}

	return 0;
}

static void __init raptor2_test_wlan_cleanup(void)
{
	return;
}

module_init(raptor2_test_wlan_init);
module_exit(raptor2_test_wlan_cleanup);
