#include <linux/delay.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <soc/edgeq/raptor2_bss_api.h>

#define MAX_RETRY_COUNT	3

static const struct of_device_id raptor2_reboot_of_match[] = {
	{
		.compatible = "edgeq,raptor2-reboot",
	},
	{ /* sentinel */ }
};

struct raptor_reboot_context {
	struct notifier_block restart_handler;
};

static int raptor_soc_reset(void)
{
	int ret;
	int i;

	ret = bss_cmd_setup(BSSRT_CHAN_EFUSE);
	if (ret < 0) {
		pr_err("%s: bss_cmd_setup failed\n", __func__);
		return ret;
	}

	ret = send_recv_bss_msg(BSSRT_CHAN_EFUSE, RAPTOR2_MSGID_SOC_RESET,
			RAPTOR2_MSGTYPE_WRITE, 0, 0, 0, 0, 0);
	if (ret < 0) {
		pr_err("%s: send_recv_bss_msg failed for %s\n",
					__func__, cmdstr[RAPTOR2_MSGID_SOC_RESET]);
		for (i = 0; i <= MAX_RETRY_COUNT; i++) {
			ret = send_recv_bss_msg(BSSRT_CHAN_EFUSE, RAPTOR2_MSGID_SOC_RESET,
				RAPTOR2_MSGTYPE_WRITE, 0, 0, 0, 0, 0);
			if (ret < 0) {
				pr_err("%s: retrying to send reboot message to bss runtime for %s\n",
						__func__, cmdstr[RAPTOR2_MSGID_SOC_RESET]);
			} else
				return 0;
		}
		return ret;
	}

	return 0;
}

static int raptor_restart_handler(struct notifier_block *this,
                                 unsigned long mode, void *cmd)
{
	int err;
	pr_emerg("%s: Entering Raptor2 restart handler\n", __func__);
	err = raptor_soc_reset();
	mdelay(1000);
	pr_warn("%s: reboot unsucessful", __func__);
	return NOTIFY_DONE;
}

static int raptor2_reboot_probe(struct platform_device *pdev)
{
	int err;
	struct raptor_reboot_context *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->restart_handler.notifier_call = raptor_restart_handler;
	ctx->restart_handler.priority = 128;
	err = register_restart_handler(&ctx->restart_handler);
	if (err)
		pr_warn("%s: cannot register restart handler", __func__);

	dev_info(&pdev->dev, "Probe for Platform Driver Reboot Successful\n");

	return err;
}

static struct platform_driver raptor2_reboot_driver = {
	.driver         = {
		.owner = THIS_MODULE,
		.name = "raptor2-reboot",
		.of_match_table = raptor2_reboot_of_match,
	},
	.probe = raptor2_reboot_probe,
};

builtin_platform_driver(raptor2_reboot_driver);
