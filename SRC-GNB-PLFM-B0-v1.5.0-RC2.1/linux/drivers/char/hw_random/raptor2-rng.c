// SPDX-License-Identifier: GPL-2.0-only
/*
 * EdgeQ TRNG driver
 *
 * Copyright (C) 2024 EdgeQ Inc.
 * Author: Akhilesh Kadam <c_sanjaykadam@edgeq.io>
 * Author: Nilesh Waghmare <nilesh.waghmare@edgeq.io>
 */

 #include <linux/err.h>
 #include <linux/kernel.h>
 #include <linux/hw_random.h>
 #include <linux/delay.h>
 #include <linux/timer.h>
 #include <linux/io.h>
 #include <linux/module.h>
 #include <linux/of.h>
 #include <linux/platform_device.h>
 #include <linux/random.h>

 #define RNG_CTRL_OFFSET         0x0
 #define RNG_MODE_OFFSET         0x8
 #define RNG_IE_OFFSET           0x10
 #define RNG_ISTAT_OFFSET        0x14
 #define RNG_RAND_OFFSET         0x20

 #define RNG_IE_VALUE            0x8000001F
 #define RNG_ISTAT_VALUE         0x1
 #define RNG_MODE_VALUE          0x8
 #define RAND_RDY_VALUE          0x1
 #define CMD_GEN_RAND		0x1
 #define CMD_NOP		0x0

 #define DELAY_TIME		1
 #define RAND32_SIZE		32
 #define RAND16_SIZE		16
 #define RAND_REG_SIZE		4
 #define RNG_RAND_LEN		8
 #define MAX_RETRY		10

 #define to_raptor2_rng(p)	container_of(p, struct raptor2_rng, rng)


struct raptor2_rng {
	void __iomem *base;
	struct hwrng rng;
};

static int raptor2_rng_init(struct hwrng *rng)
{
	struct raptor2_rng *hrng = to_raptor2_rng(rng);
	u32 val;

	/* reading and setting the value of IE register */
	val = readl(hrng->base + RNG_IE_OFFSET);
	writel((val | RNG_IE_VALUE), (hrng->base + RNG_IE_OFFSET));

	pr_info("RNG module init successful\n");

	return 0;
}

static void raptor2_rng_cleanup(struct hwrng *rng)
{
	struct raptor2_rng *hrng = to_raptor2_rng(rng);
	u32 val;

	/*
	 * Reading and overriding the value of ISTAT register from previous call
	 */
	val = readl(hrng->base + RNG_ISTAT_OFFSET);
	writel((val | RNG_ISTAT_VALUE), (hrng->base + RNG_ISTAT_OFFSET));

	/*
	 * Reading and disabling the value of IE register from previous call
	 */
	val = readl(hrng->base + RNG_IE_OFFSET);
	writel((val & ~RNG_IE_VALUE), (hrng->base + RNG_IE_OFFSET));

	/*
	 * Reading and overriding the value of CTRL register from previous call
	 */
	val = readl(hrng->base + RNG_CTRL_OFFSET);
	writel((val & CMD_NOP), (hrng->base + RNG_CTRL_OFFSET));
}

static int raptor2_rng_read(struct hwrng *rng, void *buf, size_t max,
		bool wait)
{
	struct raptor2_rng *hrng = to_raptor2_rng(rng);
	u32 val;
	u32 data[8];
	int i, iter;

	/*
	 * Reading and overriding the value of ISTAT register from previous call
	 */
	val = readl(hrng->base + RNG_ISTAT_OFFSET);
	writel((val | RNG_ISTAT_VALUE), (hrng->base + RNG_ISTAT_OFFSET));

	/*
	 * To send control signal to produce random number in RAND resisters
	 */
	val = readl(hrng->base + RNG_CTRL_OFFSET);
	writel((val | CMD_GEN_RAND), (hrng->base + RNG_CTRL_OFFSET));


	/*
	 * Reading the ISTAT register value to check if the random number is ready
	 */
	val = readl(hrng->base + RNG_ISTAT_OFFSET);

	for (iter = 0; iter <  MAX_RETRY; iter++) {
		val = readl(hrng->base + RNG_ISTAT_OFFSET);
		if (val & RAND_RDY_VALUE)
			break;
		udelay(DELAY_TIME);
	}

	if (iter == MAX_RETRY)
		return -EIO;

	/* Copying the random number from the TRNG to buffer */
	for (i = 0; i < RNG_RAND_LEN; i++)
		data[i] = readl_relaxed(hrng->base + RNG_RAND_OFFSET +
				(i*RAND_REG_SIZE));

	memcpy(buf, data, RAND32_SIZE);

	/* Checking the mode of the output */
	val = readl(hrng->base + RNG_MODE_OFFSET);
	if (val & RNG_MODE_VALUE)
		return RAND32_SIZE;	/* Number of bytes read */
	else
		return RAND16_SIZE;	/* Number of bytes read */
}

static int raptor2_rng_probe(struct platform_device *pdev)
{
	struct raptor2_rng *rng;
	int ret;
	struct resource *res;

	rng = devm_kzalloc(&pdev->dev, sizeof(*rng), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	rng->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rng->base))
		return PTR_ERR(rng->base);

	dev_info(&pdev->dev, "EdgeQ raptor-2 rng device found\n");
	dev_info(&pdev->dev, "Mapped I/O memory address: %pS for rng\n",
			rng->base);

	rng->rng.name = pdev->name;
	rng->rng.init = raptor2_rng_init;
	rng->rng.cleanup = raptor2_rng_cleanup;
	rng->rng.read = raptor2_rng_read;

	ret = devm_hwrng_register(&pdev->dev, &rng->rng);
	if (ret) {
		dev_err(&pdev->dev, "failed to register hwrng\n");
		return ret;
	}

	platform_set_drvdata(pdev, rng);

	dev_info(&pdev->dev, "Successfully registered hwrng\n");

	return 0;
}

static int raptor2_rng_remove(struct platform_device *pdev)
{
	struct raptor2_rng *rng = platform_get_drvdata(pdev);
	u32 val;

	/*
	 * Reading and overriding the value of ISTAT register from previous call
	 */
	val = readl(rng->base + RNG_ISTAT_OFFSET);
	writel((val | RNG_ISTAT_VALUE), (rng->base + RNG_ISTAT_OFFSET));

	/*
	 * Reading and disabling the value of IE register from previous call
	 */
	val = readl(rng->base + RNG_IE_OFFSET);
	writel((val & ~RNG_IE_VALUE), (rng->base + RNG_IE_OFFSET));

	/*
	 * Reading and overriding the value of CTRL register from previous call
	 */
	val = readl(rng->base + RNG_CTRL_OFFSET);
	writel((val & CMD_NOP), (rng->base + RNG_CTRL_OFFSET));

	devm_hwrng_unregister(&pdev->dev, &rng->rng);

	return 0;
}

static const struct of_device_id raptor2_rng_dt_ids[] = {
	{ .compatible = "edgeq,raptor2-rng" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, raptor2_rng_dt_ids);

static struct platform_driver raptor2_rng_driver = {
	.probe		= raptor2_rng_probe,
	.remove		= raptor2_rng_remove,
	.driver		= {
		.owner = THIS_MODULE,
		.name	= "raptor2-rng",
		.of_match_table = of_match_ptr(raptor2_rng_dt_ids),
	},
};

module_platform_driver(raptor2_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Akhilesh Kadam <c_sanjaykadam@edgeq.io>");
MODULE_AUTHOR("Nilesh Waghmare <nilesh.waghmare@edgeq.io>");
MODULE_DESCRIPTION("EdegeQ TRNG driver");
