/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*
 * based on arch/arm/plat-mxc/pwm.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/pmic_status.h>
#include <linux/pmic_external.h>
#include <linux/mfd/mc34708/mc34708.h>
#include <linux/mfd/mc34708/mc34708_pwm.h>

#define PWM1DUTY_SHIFT		0
#define PWM1CLKDIV_SHIFT	6
#define PWM2DUTY_SHIFT		12

#define BASE_CLK_FREQ		(2000000 / 32) /* base clk: 2MHz/32 */
#define MAX_CLK_DIV		64
#define PERIOD_NS_LOW_LIMIT	(1000000000 / BASE_CLK_FREQ)
#define PERIOD_NS_HIGH_LIMIT	(PERIOD_NS_LOW_LIMIT * MAX_CLK_DIV)

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device {
	struct list_head	node;
	struct platform_device *pdev;
	const char	*label;

	unsigned int	use_count;
	unsigned int	pwm_id;
	unsigned int	duty_cycles;
};

int mc34708_pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long long c;
	unsigned long period_cycles;
	u32 offset;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	if (period_ns < PERIOD_NS_LOW_LIMIT) {
		dev_warn(&pwm->pdev->dev,
			"Period time LESSER than the low limit,"
			" forced to %d ns.\n",
			PERIOD_NS_LOW_LIMIT);

		period_cycles = 1;
		period_ns = PERIOD_NS_LOW_LIMIT;
	} else if (period_ns > PERIOD_NS_HIGH_LIMIT) {
		dev_warn(&pwm->pdev->dev,
			"Period time BEYOND the high limit, forced to %d ns.\n",
			PERIOD_NS_HIGH_LIMIT);

		period_cycles = MAX_CLK_DIV;
		period_ns = PERIOD_NS_HIGH_LIMIT;
		if (duty_ns > period_ns)
			duty_ns = period_ns;
	} else {
		c = period_ns;
		do_div(c, 1000);
		c = c * BASE_CLK_FREQ;
		do_div(c, 1000000);
		period_cycles = c;
	}

	offset = pwm->pwm_id * PWM2DUTY_SHIFT + PWM1CLKDIV_SHIFT;
	pmic_write_reg(MC34708_REG_PWM_CTL, (period_cycles - 1) << offset,
			0x3F << offset);

	c = (unsigned long long)duty_ns * 32;
	do_div(c, period_ns);
	pwm->duty_cycles = c;

	/* The following is for debug purpose */
	period_ns = PERIOD_NS_LOW_LIMIT * period_cycles;
	duty_ns = (pwm->duty_cycles == 0) ?
			0 : (period_ns * pwm->duty_cycles / 32);
	dev_dbg(&pwm->pdev->dev, "set duty_ns %d, period_ns %d\n",
		duty_ns, period_ns);

	return 0;
}
EXPORT_SYMBOL(mc34708_pwm_config);

int mc34708_pwm_enable(struct pwm_device *pwm)
{
	int rc = 0;
	u32 offset;

	/* No dedicated {EN|DIS}ABLE bit, it's controlled by duty cycle */
	offset = pwm->pwm_id * PWM2DUTY_SHIFT;
	rc = pmic_write_reg(MC34708_REG_PWM_CTL, pwm->duty_cycles << offset,
			0x3F << offset);
	dev_dbg(&pwm->pdev->dev, "%s: duty_cycles %d\n",
			__func__, pwm->duty_cycles);

	return rc;
}
EXPORT_SYMBOL(mc34708_pwm_enable);

void mc34708_pwm_disable(struct pwm_device *pwm)
{
	u32 offset;

	offset = pwm->pwm_id * PWM2DUTY_SHIFT;
	pmic_write_reg(MC34708_REG_PWM_CTL, 0, 0x3F << offset);

}
EXPORT_SYMBOL(mc34708_pwm_disable);

struct pwm_device *mc34708_pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;

	mutex_lock(&pwm_lock);

	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(mc34708_pwm_request);

void mc34708_pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);

	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		pr_warning("PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(mc34708_pwm_free);

static int __devinit mc34708_pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm;

	if (pdev->id != 0 && pdev->id != 1) {
		dev_err(&pdev->dev, "pdev->id (pwm_id) should be 0 or 1,"
			"instead of %d\n", pdev->id);
		return -EINVAL;
	}

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->use_count = 0;
	pwm->pwm_id = pdev->id;
	pwm->pdev = pdev;

	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);

	platform_set_drvdata(pdev, pwm);

	return 0;
}

static int __devexit mc34708_pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	kfree(pwm);
	return 0;
}

static struct platform_driver mc34708_pwm_driver = {
	.driver		= {
		.name	= "mc34708_pwm",
	},
	.probe		= mc34708_pwm_probe,
	.remove		= __devexit_p(mc34708_pwm_remove),
};

static int __init mc34708_pwm_init(void)
{
	return platform_driver_register(&mc34708_pwm_driver);
}
arch_initcall(mc34708_pwm_init);

static void __exit mc34708_pwm_exit(void)
{
	platform_driver_unregister(&mc34708_pwm_driver);
}
module_exit(mc34708_pwm_exit);

MODULE_DESCRIPTION("MC34708 PWM driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
