// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include "ele_bbsm.h"

/*
 * Get BBSM Tamper status.
 * Returns non-zero value if tamper event has been reported.
 */
int ele_bbsm_get_tamper_status(struct se_if_priv *priv)
{
	int ret = 0;
	struct arm_smccc_res res;

	/* Check if BBSM tamper event reported */
	arm_smccc_smc(IMX_SIP_BBSM, IMX_SIP_BBSM_READ_TAMPER_STATUS,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(priv->dev, "Failed to read tamper status.");
		return ret;
	}

	if (res.a1)
		ret = 1;

	return ret;
}

/* BBSM Tamper Interrupt handler */
static irqreturn_t ele_bbsm_irq_handler(int irq, void *data)
{
	struct arm_smccc_res res;

	pr_info("BBSM Tamper Interrupt: handler evoked.\n");

	/* Send clear interrupt request to ELE */
	arm_smccc_smc(IMX_SIP_BBSM, IMX_SIP_BBSM_CLEAR_INTERRUPT, 0, 0, 0, 0,
		      0, 0, &res);
	if (res.a0) {
		pr_err("BBSM Tamper Interrupt: Clear Interrupt op failed.\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/* Register BBSM Tamper IRQ handlers */
int ele_bbsm_irq_register(struct se_if_priv *priv)
{
	int ret, irq[2];
	struct platform_device *pdev = to_platform_device(priv->dev);

	/* Get IRQ (secure interrupt) */
	irq[0] = platform_get_irq(pdev, 0);
	if (irq[0] < 0) {
		dev_err(priv->dev, "Failed to get IRQ\n");
		return irq[0];
	}

	/* Get IRQ (non-secure interrupt) */
	irq[1] = platform_get_irq(pdev, 1);
	if (irq[1] < 0) {
		dev_err(priv->dev, "Failed to get IRQ\n");
		return irq[1];
	}

	/* Register IRQ handler (secure interrupt) */
	ret = devm_request_irq(priv->dev, irq[0], ele_bbsm_irq_handler, 0,
			       dev_name(priv->dev), NULL);
	if (ret) {
		dev_err(priv->dev, "Failed to request IRQ\n");
		return ret;
	}

	/* Register IRQ handler (non-secure interrupt) */
	ret = devm_request_irq(priv->dev, irq[1], ele_bbsm_irq_handler, 0,
			       dev_name(priv->dev), NULL);
	if (ret) {
		dev_err(priv->dev, "Failed to request IRQ\n");
		return ret;
	}

	return ret;
}
