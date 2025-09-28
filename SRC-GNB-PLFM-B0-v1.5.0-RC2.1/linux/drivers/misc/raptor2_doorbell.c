#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <linux/of.h>
#include <linux/of_device.h>

/* Enable / Disable SIF register based interrupts */
#define SIF_REG_INT	0

#if SIF_REG_INT
#define SIF_REG_EVENT_TO_CPU_BASE	0x69080540UL

#define GIC_SPI_INT_BASE	32
#define NUM_INTR_PER_REG	32
#endif

struct raptor2_doorbell {
	int hwirq;
	int virq;
	int zirq;
#if SIF_REG_INT
	void __iomem *regs_base;
#endif
	struct device *dev;
};

#if SIF_REG_INT
static void doorbell_update_irq_bit(int irq, void __iomem *regs_base, bool value)
{
	void __iomem *reg;
	uint32_t bit_pos;
	uint32_t bit_base_offset;

	bit_pos = irq % NUM_INTR_PER_REG;
	bit_base_offset = irq / NUM_INTR_PER_REG;
	reg = regs_base + (bit_base_offset * 4);

	if (value)
		writel(readl(reg) | (uint32_t)(1 << bit_pos), reg);
	else
		writel(readl(reg) & (uint32_t)(~(1 << bit_pos)), reg);
}
#endif

static irqreturn_t raptor2_doorbell_irq(int irq, void *data)
{
	struct raptor2_doorbell *raptor2_doorbell = data;

#if SIF_REG_INT
	/* GIC SPI interrupt number will be (actual hwirq + GIC_SPI_INT_BASE) */
	doorbell_update_irq_bit(raptor2_doorbell->hwirq - GIC_SPI_INT_BASE,
				raptor2_doorbell->regs_base, 0);

	/* Sample code to trigger irq on remote end (Zephyr) */

	/* SIF: Use actual hwirq number for Zephyr side remote irq */
	//doorbell_update_irq_bit(raptor2_doorbell->zirq,
	//			raptor2_doorbell->regs_base, 1);
#else

	/* GIC-SPI: Set the Zephyr side interrupt as pending inside GIC */
	//irq_set_irqchip_state(raptor2_doorbell->zirq, IRQCHIP_STATE_PENDING, true);
#endif

	printk(KERN_CRIT "Doorbell ISR triggered for virq %d and hwirq %d\n",
			  raptor2_doorbell->virq, raptor2_doorbell->hwirq);

	return IRQ_HANDLED;
}

static int raptor2_doorbell_probe(struct platform_device *pdev)
{
	struct raptor2_doorbell *raptor2_doorbell;
	struct irq_data *irqd;
	int ret;

	raptor2_doorbell = devm_kzalloc(&pdev->dev,
					sizeof(struct raptor2_doorbell),
					GFP_KERNEL);
	if (!raptor2_doorbell) {
		ret = -ENOMEM;
		goto out;
	}

	raptor2_doorbell->dev = &pdev->dev;

#if SIF_REG_INT
	raptor2_doorbell->regs_base = ioremap(SIF_REG_EVENT_TO_CPU_BASE, 0x20);
	if (raptor2_doorbell->regs_base == NULL) {
		dev_err(&pdev->dev, "Can't map IO regs\n");
		ret = -ENXIO;
		goto out;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "zirq",
				   &raptor2_doorbell->zirq);
	if (ret) {
		dev_err(&pdev->dev, "Can't get remote IRQ !!\n");
		goto out;
	}
#else

	raptor2_doorbell->zirq = platform_get_irq(pdev, 1);
	if (raptor2_doorbell->zirq < 0) {
		dev_err(&pdev->dev, "Can't get remote IRQ !!\n");
		ret = raptor2_doorbell->zirq;
		goto out;
	}
#endif

	raptor2_doorbell->virq = platform_get_irq(pdev, 0);
	if (raptor2_doorbell->virq < 0) {
		dev_err(&pdev->dev, "Can't get IRQ !!\n");
		ret = raptor2_doorbell->virq;
		goto out;
	}

	irqd = irq_get_irq_data(raptor2_doorbell->virq);
	if (!irqd) {
		ret = -EINVAL;
		goto out;
	}
	raptor2_doorbell->hwirq = irqd->hwirq;

	ret = request_irq(raptor2_doorbell->virq, raptor2_doorbell_irq, 0,
			  pdev->name, raptor2_doorbell);
	if (ret) {
		dev_err(&pdev->dev, "Can't get IRQ %d !!\n",
			raptor2_doorbell->virq);
	} else {
		platform_set_drvdata(pdev, raptor2_doorbell);
	}

out:
	return ret;
}

static int raptor2_doorbell_remove(struct platform_device *pdev)
{
	struct raptor2_doorbell *raptor2_doorbell = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	free_irq(raptor2_doorbell->virq, raptor2_doorbell);
#if SIF_REG_INT
	iounmap(raptor2_doorbell->regs_base);
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id doorbell_dt_match[] = {
	{ .compatible = "edgeq,raptor2-doorbell" },
	{},
};
MODULE_DEVICE_TABLE(of, doorbell_dt_match);
#endif

static struct platform_driver raptor2_doorbell_platform_driver = {

	.driver		= {
		.name	= "raptor2-doorbell",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(doorbell_dt_match),
#endif
	},
	.probe		= raptor2_doorbell_probe,
	.remove		= raptor2_doorbell_remove,
};

module_platform_driver(raptor2_doorbell_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Saurabh Karajgaonkar");
MODULE_DESCRIPTION("Raptor2 doorbell driver");
