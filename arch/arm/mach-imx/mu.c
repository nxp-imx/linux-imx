/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/busfreq-imx6.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include "common.h"
#include "hardware.h"

#define MU_ATR0_OFFSET	0x0
#define MU_ARR0_OFFSET	0x10
#define MU_ASR		0x20
#define MU_ACR		0x24

#define MU_LPM_HANDSHAKE_INDEX		0
#define MU_LPM_BUS_HIGH_READY_FOR_M4	0xFFFF6666
#define MU_LPM_M4_FREQ_CHANGE_READY	0xFFFF7777
#define MU_LPM_M4_REQUEST_HIGH_BUS	0x2222CCCC
#define MU_LPM_M4_RELEASE_HIGH_BUS	0x2222BBBB
#define MU_LPM_M4_WAKEUP_SRC_VAL	0x55555000
#define MU_LPM_M4_WAKEUP_SRC_MASK	0xFFFFF000
#define MU_LPM_M4_WAKEUP_IRQ_MASK	0xFF0
#define MU_LPM_M4_WAKEUP_IRQ_SHIFT	0x4
#define MU_LPM_M4_WAKEUP_ENABLE_MASK	0xF
#define MU_LPM_M4_WAKEUP_ENABLE_SHIFT	0x0

static void __iomem *mu_base;
static u32 m4_message;
static struct delayed_work mu_work;
static u32 m4_wake_irqs[4];
static bool m4_freq_low;

bool imx_mu_is_m4_in_low_freq(void)
{
	return m4_freq_low;
}

void imx_mu_enable_m4_irqs_in_gic(bool enable)
{
	int i, j;

	for (i = 0; i < 4; i++) {
		if (m4_wake_irqs[i] == 0)
			continue;
		for (j = 0; j < 32; j++) {
			if (m4_wake_irqs[i] & (1 << j)) {
				if (enable)
					enable_irq((i + 1) * 32 + j);
				else
					disable_irq((i + 1) * 32 + j);
			}
		}
	}
}

static irqreturn_t mcc_m4_dummy_isr(int irq, void *param)
{
	return IRQ_HANDLED;
}

static void imx_mu_send_message(unsigned int index, unsigned int data)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	/* wait for transfer buffer empty */
	do {
		val = readl_relaxed(mu_base + MU_ASR);
		if (time_after(jiffies, timeout)) {
			pr_err("Waiting MU transmit buffer empty timeout!\n");
			break;
		}
	} while ((val & (1 << (20 + index))) == 0);

	writel_relaxed(data, mu_base + index * 0x4 + MU_ATR0_OFFSET);
}

static void mu_work_handler(struct work_struct *work)
{
	int ret;
	u32 irq, enable, idx, mask;

	pr_debug("receive M4 message 0x%x\n", m4_message);

	switch (m4_message) {
	case MU_LPM_M4_REQUEST_HIGH_BUS:
		request_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(true);
		imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_BUS_HIGH_READY_FOR_M4);
		m4_freq_low = false;
		break;
	case MU_LPM_M4_RELEASE_HIGH_BUS:
		release_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(false);
		imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_M4_FREQ_CHANGE_READY);
		m4_freq_low = true;
		break;
	default:
		if ((m4_message & MU_LPM_M4_WAKEUP_SRC_MASK) ==
			MU_LPM_M4_WAKEUP_SRC_VAL) {
			irq = (m4_message & MU_LPM_M4_WAKEUP_IRQ_MASK) >>
				MU_LPM_M4_WAKEUP_IRQ_SHIFT;

			enable = (m4_message & MU_LPM_M4_WAKEUP_ENABLE_MASK) >>
				MU_LPM_M4_WAKEUP_ENABLE_SHIFT;

			idx = irq / 32 - 1;
			mask = 1 << irq % 32;

			if (enable && can_request_irq(irq, 0)) {
				ret = request_irq(irq, mcc_m4_dummy_isr,
					IRQF_NO_SUSPEND, "imx-m4-dummy", NULL);
				if (ret) {
					pr_err("%s: register interrupt %d failed, rc %d\n",
						__func__, irq, ret);
					break;
				}
				disable_irq(irq);
				m4_wake_irqs[idx] = m4_wake_irqs[idx] | mask;
			}
			imx_gpc_add_m4_wake_up_irq(irq, enable);
		}
		break;
	}
	m4_message = 0;
	/* enable RIE3 interrupt */
	writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(27),
		mu_base + MU_ACR);
}

static irqreturn_t imx_mu_isr(int irq, void *param)
{
	u32 irqs;

	irqs = readl_relaxed(mu_base + MU_ASR);

	if (irqs & (1 << 27)) {
		/* get message from receive buffer */
		m4_message = readl_relaxed(mu_base + MU_ARR0_OFFSET);
		/* disable RIE3 interrupt */
		writel_relaxed(readl_relaxed(mu_base + MU_ACR) & (~BIT(27)),
			mu_base + MU_ACR);
		schedule_delayed_work(&mu_work, 0);
	}

	return IRQ_HANDLED;
}

static int imx_mu_probe(struct platform_device *pdev)
{
	int ret;
	u32 irq;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-mu");
	mu_base = of_iomap(np, 0);
	WARN_ON(!mu_base);

	irq = platform_get_irq(pdev, 0);

	ret = request_irq(irq, imx_mu_isr,
		IRQF_EARLY_RESUME, "imx-mu", NULL);
	if (ret) {
		pr_err("%s: register interrupt %d failed, rc %d\n",
			__func__, irq, ret);
		return ret;
	}
	INIT_DELAYED_WORK(&mu_work, mu_work_handler);

	/* enable the bit27(RIE3) of MU_ACR */
	writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(27),
		mu_base + MU_ACR);

	/* MU always as a wakeup source for low power mode */
	imx_gpc_add_m4_wake_up_irq(irq, true);

	pr_info("MU is ready for cross core communication!\n");

	return 0;
}

static const struct of_device_id imx_mu_ids[] = {
	{ .compatible = "fsl,imx6sx-mu" },
	{ }
};

static struct platform_driver imx_mu_driver = {
	.driver = {
		.name   = "imx-mu",
		.owner  = THIS_MODULE,
		.of_match_table = imx_mu_ids,
	},
	.probe = imx_mu_probe,
};

static int __init imx6_mu_init(void)
{
	return platform_driver_register(&imx_mu_driver);
}
subsys_initcall(imx6_mu_init);
