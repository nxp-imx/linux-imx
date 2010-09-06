/*
 * Freesclae MXS Touchscreen driver
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>

#include <mach/hardware.h>
#include <mach/lradc.h>
#include <mach/device.h>
#include <mach/regs-lradc.h>

#define TOUCH_DEBOUNCE_TOLERANCE	100

struct mxs_ts_info {
	int touch_irq;
	int device_irq;
	unsigned int base;
	u8 x_plus_chan;
	u8 x_minus_chan;
	u8 y_plus_chan;
	u8 y_minus_chan;

	unsigned int x_plus_val;
	unsigned int x_minus_val;
	unsigned int y_plus_val;
	unsigned int y_minus_val;
	unsigned int x_plus_mask;
	unsigned int x_minus_mask;
	unsigned int y_plus_mask;
	unsigned int y_minus_mask;

	struct input_dev *idev;
	enum {
		TS_STATE_DISABLED,
		TS_STATE_TOUCH_DETECT,
		TS_STATE_TOUCH_VERIFY,
		TS_STATE_X_PLANE,
		TS_STATE_Y_PLANE,
	} state;
	u16 x;
	u16 y;
	int sample_count;
};

static inline void enter_state_touch_detect(struct mxs_ts_info *info)
{
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_plus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_plus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_minus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_minus_chan));

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,
		     info->base + HW_LRADC_CTRL1_CLR);
	/*
	 * turn off the yplus and yminus pullup and pulldown, and turn off touch
	 * detect (enables yminus, and xplus through a resistor.On a press,
	 * xplus is pulled down)
	 */
	__raw_writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_SET);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_TOUCH_DETECT;
	info->sample_count = 0;
}

static inline void enter_state_disabled(struct mxs_ts_info *info)
{
	__raw_writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 0);
	info->state = TS_STATE_DISABLED;
	info->sample_count = 0;
}


static inline void enter_state_x_plane(struct mxs_ts_info *info)
{
	__raw_writel(info->y_plus_val, info->base + HW_LRADC_CTRL0_SET);
	__raw_writel(info->y_minus_val, info->base + HW_LRADC_CTRL0_SET);
	__raw_writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);

	info->state = TS_STATE_X_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_y_plane(struct mxs_ts_info *info)
{
	__raw_writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_plus_val, info->base + HW_LRADC_CTRL0_SET);
	__raw_writel(info->x_minus_val, info->base + HW_LRADC_CTRL0_SET);
	__raw_writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_CLR);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->state = TS_STATE_Y_PLANE;
	info->sample_count = 0;
}

static inline void enter_state_touch_verify(struct mxs_ts_info *info)
{
	__raw_writel(info->y_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->y_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_plus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(info->x_minus_mask, info->base + HW_LRADC_CTRL0_CLR);
	__raw_writel(BM_LRADC_CTRL0_TOUCH_DETECT_ENABLE,
		     info->base + HW_LRADC_CTRL0_SET);
	info->state = TS_STATE_TOUCH_VERIFY;
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
	info->sample_count = 0;
}

static void process_lradc(struct mxs_ts_info *info, u16 x, u16 y,
			int pressure)
{
	switch (info->state) {
	case TS_STATE_X_PLANE:
		pr_debug("%s: x plane state, sample_count %d\n", __func__,
				info->sample_count);
		if (info->sample_count < 2) {
			info->x = x;
			info->sample_count++;
		} else {
			if (abs(info->x - x) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 x_c = info->x * (info->sample_count - 1);
				info->x = (x_c + x) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4)
			enter_state_y_plane(info);
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_Y_PLANE:
		pr_debug("%s: y plane state, sample_count %d\n", __func__,
				info->sample_count);
		if (info->sample_count < 2) {
			info->y = y;
			info->sample_count++;
		} else {
			if (abs(info->y - y) > TOUCH_DEBOUNCE_TOLERANCE)
				info->sample_count = 1;
			else {
				u16 y_c = info->y * (info->sample_count - 1);
				info->y = (y_c + y) / info->sample_count;
				info->sample_count++;
			}
		}
		if (info->sample_count > 4)
			enter_state_touch_verify(info);
		else
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		break;

	case TS_STATE_TOUCH_VERIFY:
		pr_debug("%s: touch verify state, sample_count %d\n", __func__,
				info->sample_count);
		pr_debug("%s: x %d, y %d\n", __func__, info->x, info->y);
		input_report_abs(info->idev, ABS_X, info->x);
		input_report_abs(info->idev, ABS_Y, info->y);
		input_report_abs(info->idev, ABS_PRESSURE, pressure);
		input_sync(info->idev);
		/* fall through */
	case TS_STATE_TOUCH_DETECT:
		pr_debug("%s: touch detect state, sample_count %d\n", __func__,
				info->sample_count);
		if (pressure) {
			input_report_abs(info->idev, ABS_PRESSURE, pressure);
			enter_state_x_plane(info);
			hw_lradc_set_delay_trigger_kick(
					LRADC_DELAY_TRIGGER_TOUCHSCREEN, 1);
		} else
			enter_state_touch_detect(info);
		break;

	default:
		printk(KERN_ERR "%s: unknown touchscreen state %d\n", __func__,
				info->state);
	}
}

static irqreturn_t ts_handler(int irq, void *dev_id)
{
	struct mxs_ts_info *info = dev_id;
	u16 x_plus, y_plus;
	int pressure = 0;

	if (irq == info->touch_irq)
		__raw_writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,
			     info->base + HW_LRADC_CTRL1_CLR);
	else if (irq == info->device_irq)
		__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
			     info->base + HW_LRADC_CTRL1_CLR);

	/* get x, y values */
	x_plus = __raw_readl(info->base + HW_LRADC_CHn(info->x_plus_chan)) &
		BM_LRADC_CHn_VALUE;
	y_plus = __raw_readl(info->base + HW_LRADC_CHn(info->y_plus_chan)) &
		BM_LRADC_CHn_VALUE;

	/* pressed? */
	if (__raw_readl(info->base + HW_LRADC_STATUS) &
	    BM_LRADC_STATUS_TOUCH_DETECT_RAW)
		pressure = 1;

	pr_debug("%s: irq %d, x_plus %d, y_plus %d, pressure %d\n",
			__func__, irq, x_plus, y_plus, pressure);

	process_lradc(info, x_plus, y_plus, pressure);

	return IRQ_HANDLED;
}

static int __devinit mxs_ts_probe(struct platform_device *pdev)
{
	struct input_dev *idev;
	struct mxs_ts_info *info;
	int ret = 0;
	struct resource *res;
	struct mxs_touchscreen_plat_data *plat_data;

	plat_data = (struct mxs_touchscreen_plat_data *)pdev->dev.platform_data;
	if (plat_data == NULL)
		return -ENODEV;

	idev = input_allocate_device();
	if (idev == NULL)
		return -ENOMEM;

	info = kzalloc(sizeof(struct mxs_ts_info), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto out_nomem_info;
	}

	idev->name = "MXS touchscreen";
	idev->evbit[0] = BIT(EV_ABS);
	input_set_abs_params(idev, ABS_X, 0, 0xFFF, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 0xFFF, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);

	ret = input_register_device(idev);
	if (ret)
		goto out_nomem;

	info->idev = idev;
	info->x_plus_chan = plat_data->x_plus_chan;
	info->x_minus_chan = plat_data->x_minus_chan;
	info->y_plus_chan = plat_data->y_plus_chan;
	info->y_minus_chan = plat_data->y_minus_chan;
	info->x_plus_val = plat_data->x_plus_val;
	info->x_minus_val = plat_data->x_minus_val;
	info->y_plus_val = plat_data->y_plus_val;
	info->y_minus_val = plat_data->y_minus_val;
	info->x_plus_mask = plat_data->x_plus_mask;
	info->x_minus_mask = plat_data->x_minus_mask;
	info->y_plus_mask = plat_data->y_plus_mask;
	info->y_minus_mask = plat_data->y_minus_mask;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ERR "%s: couldn't get MEM resource\n", __func__);
		ret = -ENODEV;
		goto out_nodev;
	}
	info->base = (unsigned int)IO_ADDRESS(res->start);
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "%s: couldn't get IRQ resource\n", __func__);
		ret = -ENODEV;
		goto out_nodev;
	}
	info->touch_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		printk(KERN_ERR "%s: couldn't get IRQ resource\n", __func__);
		ret = -ENODEV;
		goto out_nodev;
	}
	info->device_irq = res->start;

	ret = request_irq(info->touch_irq, ts_handler, IRQF_DISABLED,
				"mxs_ts_touch", info);
	if (ret)
		goto out_nodev;

	ret = request_irq(info->device_irq, ts_handler, IRQF_DISABLED,
				"mxs_ts_dev", info);
	if (ret) {
		free_irq(info->touch_irq, info);
		goto out_nodev;
	}
	enter_state_touch_detect(info);

	hw_lradc_use_channel(info->x_plus_chan);
	hw_lradc_use_channel(info->x_minus_chan);
	hw_lradc_use_channel(info->y_plus_chan);
	hw_lradc_use_channel(info->y_minus_chan);
	hw_lradc_configure_channel(info->x_plus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->x_minus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->y_plus_chan, 0, 0, 0);
	hw_lradc_configure_channel(info->y_minus_chan, 0, 0, 0);

	/* Clear the accumulator & NUM_SAMPLES for the channels */
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_plus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->x_minus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_plus_chan));
	__raw_writel(0xFFFFFFFF,
		     info->base + HW_LRADC_CHn_CLR(info->y_minus_chan));

	hw_lradc_set_delay_trigger(LRADC_DELAY_TRIGGER_TOUCHSCREEN,
			0x3c, 0, 0, 8);

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ,
		     info->base + HW_LRADC_CTRL1_CLR);

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_SET);
	__raw_writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ_EN,
		     info->base + HW_LRADC_CTRL1_SET);

	platform_set_drvdata(pdev, info);
	device_init_wakeup(&pdev->dev, 1);
	goto out;

out_nodev:
	input_free_device(idev);
out_nomem:
	kfree(info);
out_nomem_info:
	kfree(idev);
out:
	return ret;
}

static int __devexit mxs_ts_remove(struct platform_device *pdev)
{
	struct mxs_ts_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	hw_lradc_unuse_channel(info->x_plus_chan);
	hw_lradc_unuse_channel(info->x_minus_chan);
	hw_lradc_unuse_channel(info->y_plus_chan);
	hw_lradc_unuse_channel(info->y_minus_chan);

	__raw_writel(BM_LRADC_CTRL1_LRADC0_IRQ_EN << info->y_minus_chan,
		     info->base + HW_LRADC_CTRL1_CLR);
	__raw_writel(BM_LRADC_CTRL1_TOUCH_DETECT_IRQ_EN,
		     info->base + HW_LRADC_CTRL1_CLR);

	free_irq(info->device_irq, info);
	free_irq(info->touch_irq, info);
	input_free_device(info->idev);

	enter_state_disabled(info);
	kfree(info->idev);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int mxs_ts_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct mxs_ts_info *info = platform_get_drvdata(pdev);

	if (!device_may_wakeup(&pdev->dev)) {
		hw_lradc_unuse_channel(info->x_plus_chan);
		hw_lradc_unuse_channel(info->x_minus_chan);
		hw_lradc_unuse_channel(info->y_plus_chan);
		hw_lradc_unuse_channel(info->y_minus_chan);
	}
	return 0;
}

static int mxs_ts_resume(struct platform_device *pdev)
{
	struct mxs_ts_info *info = platform_get_drvdata(pdev);

	if (!device_may_wakeup(&pdev->dev)) {
		hw_lradc_use_channel(info->x_plus_chan);
		hw_lradc_use_channel(info->x_minus_chan);
		hw_lradc_use_channel(info->y_plus_chan);
		hw_lradc_use_channel(info->y_minus_chan);
	}
	return 0;
}
#endif

static struct platform_driver mxs_ts_driver = {
	.probe		= mxs_ts_probe,
	.remove		= __devexit_p(mxs_ts_remove),
#ifdef CONFIG_PM
	.suspend	= mxs_ts_suspend,
	.resume		= mxs_ts_resume,
#endif
	.driver		= {
		.name	= "mxs-ts",
	},
};

static int __init mxs_ts_init(void)
{
	return platform_driver_register(&mxs_ts_driver);
}

static void __exit mxs_ts_exit(void)
{
	platform_driver_unregister(&mxs_ts_driver);
}

module_init(mxs_ts_init);
module_exit(mxs_ts_exit);
