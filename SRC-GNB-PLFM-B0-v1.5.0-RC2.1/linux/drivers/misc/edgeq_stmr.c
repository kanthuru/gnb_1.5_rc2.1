/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2021 EdgeQ, Inc.
 *
 * stmr module will get interrupt from bss and read stmr value and sends the user space
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/cacheflush.h>


#define DEVICE_NAME		"stmr"
#define STMR_MAJOR		55
#define SIOCSSTMRSRC		(SIOCDEVPRIVATE+0)
#define SIOCGSTMRSRC		(SIOCDEVPRIVATE+1)


static int stmr_irq = 0;
static void *stmr_addr = NULL;
static void __iomem *sif_addr = NULL;
static void __iomem *ioss_mux_addr = NULL;
static bool data_ready = false;
static struct cdev stmr_cdev;
static wait_queue_head_t stmr_waitq;
static u32 ioss_mux_pps_index[5] = {123, 121, 119, 117, 115};

typedef enum {
	STMR_TRIGGER_XGMAC0 = 0,
	STMR_TRIGGER_XGMAC1,
	STMR_TRIGGER_XGMAC2,
	STMR_TRIGGER_XGMAC3,
	STMR_TRIGGER_XLGMAC,
	STMR_TRIGGER_MAX
} stmr_trigger_t;

static void stmr_work_func(struct work_struct *work)
{
	wake_up_interruptible(&stmr_waitq);
}

static DECLARE_WORK(stmr_work, stmr_work_func);

static irqreturn_t stmr_irq_handler(int irq, void *dev_id)
{
	/* Notify data ready */
	data_ready = true;
	schedule_work(&stmr_work);

	return IRQ_HANDLED;
}

static int stmr_open(struct inode *ip, struct file *fp)
{
	unsigned long irq_flags;
	int ret;

	data_ready = false;
	init_waitqueue_head(&stmr_waitq);

#ifndef CONFIG_PREEMPT_RT
	irq_flags = IRQF_SHARED;
#else
	irq_flags = IRQF_SHARED | IRQF_NO_THREAD;
#endif

	ret = request_irq(stmr_irq, stmr_irq_handler,
			  irq_flags | IRQF_TRIGGER_RISING, DEVICE_NAME, fp);
	if (ret < 0) {
		pr_err("%s: Failed to request IRQ (%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int stmr_close(struct inode *ip, struct file *fp)
{
	free_irq(stmr_irq, fp);
	return 0;
}

static __poll_t stmr_poll(struct file *fp, poll_table *wait)
{
        poll_wait(fp, &stmr_waitq, wait);
	if (data_ready)
		return (EPOLLIN | EPOLLRDNORM);

        return 0;
}

static ssize_t stmr_read(struct file *fp, char __user *buf, size_t buf_len,
			 loff_t *ppos)
{
	u32 stmr_data;
	size_t data_len;
	int ret;

	data_len = sizeof(stmr_data);
	if (buf_len < data_len)
		return 0;

	if (!data_ready) {
		return 0;
	}

	stmr_data = readl(stmr_addr);
	ret = copy_to_user(buf, &stmr_data, data_len);

	data_ready = false;

	return data_len;
}

static long stmr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u32 trigger_src;
	u32 index;
	int i;

	if (!ioss_mux_addr)
		return -EFAULT;

	switch (cmd) {
	case SIOCSSTMRSRC:
                if (copy_from_user(&trigger_src, argp, sizeof(trigger_src)))
			return -EFAULT;
		if (trigger_src >= STMR_TRIGGER_MAX)
			return -EINVAL;

		index = ioss_mux_pps_index[trigger_src];
		writel(index, ioss_mux_addr);
		break;

	case SIOCGSTMRSRC:
		index = readl(ioss_mux_addr);
		for (i = 0; i < STMR_TRIGGER_MAX; i++) {
			if (ioss_mux_pps_index[i] == index)
				break;
		}
		trigger_src = i;
		if (copy_to_user(argp, &trigger_src, sizeof(trigger_src)))
			return -EFAULT;
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct file_operations stmr_fops = {
	.owner		= THIS_MODULE,
	.open		= stmr_open,
	.release	= stmr_close,
	.poll		= stmr_poll,
	.read		= stmr_read,
	.unlocked_ioctl	= stmr_ioctl,
};

static const struct of_device_id raptor2_stmr_match[] = {
	{
		.compatible = "edgeq,raptor2-stmr",
	},
	{
		/*end of table*/
	}
};

MODULE_DEVICE_TABLE(of, raptor2_stmr_match);

static int stmr_probe(struct platform_device *pdev)
{
	int ret;
	uint64_t addr, sz;
	struct resource *res;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	if (of_device_is_compatible(pdev->dev.of_node,
				"edgeq,raptor2-stmr") == 0)
		return -EINVAL;

	stmr_irq = ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		pr_err("%s: No IRQ %d in DT\n", __func__, ret);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sif_addr = devm_ioremap_resource(&pdev->dev, res);
	if (sif_addr == NULL){
		pr_err("%s: cannot map sif_addr\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ioss_mux_addr = devm_ioremap_resource(&pdev->dev, res);
	if (ioss_mux_addr == NULL){
		pr_err("%s: cannot map ioss_mux_addr\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		pr_err("%s: memory resource for STMR SHM not found\n", __func__);
		return -ENODEV;
	}

	addr = (uint64_t)(res->start);
	sz = (uint64_t)(resource_size(res));
	pr_info("stmr shm addr: 0x%llx sz: 0x%llx\n", addr, sz);
	stmr_addr = devm_memremap(&pdev->dev, addr, sz, MEMREMAP_WT);
	if (!stmr_addr) {
		pr_err("%s: devm_memremap failed for stmr shm addr 0x%llx\n", __func__, addr);
		return -ENOMEM;
	}

	cdev_init(&stmr_cdev, &stmr_fops);
	stmr_cdev.owner = THIS_MODULE;
	ret = cdev_add(&stmr_cdev, MKDEV(STMR_MAJOR, 0), 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not add cdev\n");
		return ret;
	}

	return 0;
}

static int stmr_remove(struct platform_device *pdev)
{
	cdev_del(&stmr_cdev);

	return 0;
}

static struct platform_driver stmr_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = raptor2_stmr_match,
	},
	.probe = stmr_probe,
	.remove = stmr_remove,
};

module_platform_driver(stmr_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("stmr kernel module");
