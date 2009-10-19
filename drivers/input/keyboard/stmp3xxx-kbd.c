/*
 * Keypad ladder driver for Freescale STMP37XX/STMP378X boards
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>

#define BUTTON_PRESS_THRESHOLD  3300
#define LRADC_NOISE_MARGIN      100

/* this value represents the the lradc value at 3.3V ( 3.3V / 0.000879 V/b ) */
#define TARGET_VDDIO_LRADC_VALUE 3754

struct stmpkbd_data {
	struct input_dev *input;
	int last_button;
	int irq;
	struct stmpkbd_keypair *keycodes;
};

static int delay1 = 500;
static int delay2 = 200;

static int stmpkbd_open(struct input_dev *dev);
static void stmpkbd_close(struct input_dev *dev);

static struct stmpkbd_data *stmpkbd_data_alloc(struct platform_device *pdev,
		struct stmpkbd_keypair *keys)
{
	struct stmpkbd_data *d = kzalloc(sizeof(*d), GFP_KERNEL);

	if (!d)
		return NULL;

	if (!keys) {
		dev_err(&pdev->dev,
			"No keycodes in platform_data, bailing out.\n");
		kfree(d);
		return NULL;
	}
	d->keycodes = keys;

	d->input = input_allocate_device();
	if (!d->input) {
		kfree(d);
		return NULL;
	}

	d->input->phys = "onboard";
	d->input->uniq = "0000'0000";
	d->input->name = pdev->name;
	d->input->id.bustype = BUS_HOST;
	d->input->open = stmpkbd_open;
	d->input->close = stmpkbd_close;
	d->input->dev.parent = &pdev->dev;

	set_bit(EV_KEY, d->input->evbit);
	set_bit(EV_REL, d->input->evbit);
	set_bit(EV_REP, d->input->evbit);


	d->last_button = -1;

	while (keys->raw >= 0) {
		set_bit(keys->kcode, d->input->keybit);
		keys++;
	}

	return d;
}

static inline struct input_dev *GET_INPUT_DEV(struct stmpkbd_data *d)
{
	BUG_ON(!d);
	return d->input;
}

static void stmpkbd_data_free(struct stmpkbd_data *d)
{
	if (!d)
		return;
	if (d->input)
		input_free_device(d->input);
	kfree(d);
}

static unsigned stmpkbd_decode_button(struct stmpkbd_keypair *codes,
			int raw_button)

{
	pr_debug("Decoding %d\n", raw_button);
	while (codes->raw != -1) {
		if ((raw_button > (codes->raw - LRADC_NOISE_MARGIN)) &&
		    (raw_button < (codes->raw + LRADC_NOISE_MARGIN))) {
			pr_debug("matches code 0x%x = %d\n",
				codes->kcode, codes->kcode);
			return codes->kcode;
		}
		codes++;
	}
	return (unsigned)-1; /* invalid key */
}


static irqreturn_t stmpkbd_irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct stmpkbd_data *devdata = platform_get_drvdata(pdev);
	u16 raw_button, normalized_button, vddio;
	unsigned btn;

	raw_button = __raw_readl(REGS_LRADC_BASE +
			HW_LRADC_CHn(LRADC_CH0)) & BM_LRADC_CHn_VALUE;
	vddio = hw_lradc_vddio();
	BUG_ON(vddio == 0);

	normalized_button = (raw_button * TARGET_VDDIO_LRADC_VALUE) /
		vddio;

	if (normalized_button < BUTTON_PRESS_THRESHOLD &&
	    devdata->last_button < 0) {

		btn = stmpkbd_decode_button(devdata->keycodes,
				normalized_button);

		if (btn < KEY_MAX) {
			devdata->last_button = btn;
			input_report_key(GET_INPUT_DEV(devdata),
				devdata->last_button, !0);
		} else
			dev_err(&pdev->dev, "Invalid button: raw = %d, "
				"normalized = %d, vddio = %d\n",
				raw_button, normalized_button, vddio);
	} else if (devdata->last_button > 0 &&
		normalized_button >= BUTTON_PRESS_THRESHOLD) {

		input_report_key(GET_INPUT_DEV(devdata),
				devdata->last_button, 0);
		devdata->last_button = -1;

	}

	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC0_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	return IRQ_HANDLED;
}

static int stmpkbd_open(struct input_dev *dev)
{
	/* enable clock */
	return 0;
}

static void stmpkbd_close(struct input_dev *dev)
{
	/* disable clock */
}

static void stmpkbd_hwinit(struct platform_device *pdev)
{
	hw_lradc_init_ladder(LRADC_CH0, LRADC_DELAY_TRIGGER_BUTTON, 200);
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC0_IRQ, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmp3xxx_setl(BM_LRADC_CTRL1_LRADC0_IRQ_EN, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, !0);
}

static int stmpkbd_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	struct input_dev *idev = platform_get_drvdata(pdev);

	hw_lradc_stop_ladder(LRADC_CH0, LRADC_DELAY_TRIGGER_BUTTON);
	hw_lradc_set_delay_trigger_kick(LRADC_DELAY_TRIGGER_BUTTON, 0);
	hw_lradc_unuse_channel(LRADC_CH0);
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC0_IRQ_EN, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmpkbd_close(idev);
#endif
	return 0;
}

static int stmpkbd_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	struct input_dev *idev = platform_get_drvdata(pdev);

	stmp3xxx_setl(BM_LRADC_CTRL1_LRADC0_IRQ_EN, REGS_LRADC_BASE + HW_LRADC_CTRL1);
	stmpkbd_open(idev);
	hw_lradc_use_channel(LRADC_CH0);
	stmpkbd_hwinit(pdev);
#endif
	return 0;
}

static int __devinit stmpkbd_probe(struct platform_device *pdev)
{
	int err = 0;
	int irq = platform_get_irq(pdev, 0);
	struct stmpkbd_data *d;

	/* Create and register the input driver. */
	d = stmpkbd_data_alloc(pdev,
		(struct stmpkbd_keypair *)pdev->dev.platform_data);
	if (!d) {
		dev_err(&pdev->dev, "Cannot allocate driver structures\n");
		err = -ENOMEM;
		goto err_out;
	}

	d->irq = irq;
	err = request_irq(irq, stmpkbd_irq_handler,
		IRQF_DISABLED, pdev->name, pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot request keypad IRQ\n");
		goto err_free_dev;
	}

	platform_set_drvdata(pdev, d);

	/* Register the input device */
	err = input_register_device(GET_INPUT_DEV(d));
	if (err)
		goto err_free_irq;

	/* these two have to be set after registering the input device */
	d->input->rep[REP_DELAY] = delay1;
	d->input->rep[REP_PERIOD] = delay2;

	hw_lradc_use_channel(LRADC_CH0);
	stmpkbd_hwinit(pdev);

	return 0;

err_free_irq:
	platform_set_drvdata(pdev, NULL);
	free_irq(irq, pdev);
err_free_dev:
	stmpkbd_data_free(d);
err_out:
	return err;
}

static int __devexit stmpkbd_remove(struct platform_device *pdev)
{
	struct stmpkbd_data *d = platform_get_drvdata(pdev);

	hw_lradc_unuse_channel(LRADC_CH0);
	input_unregister_device(GET_INPUT_DEV(d));
	free_irq(d->irq, pdev);
	stmpkbd_data_free(d);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver stmpkbd_driver = {
	.probe		= stmpkbd_probe,
	.remove		= __devexit_p(stmpkbd_remove),
	.suspend	= stmpkbd_suspend,
	.resume		= stmpkbd_resume,
	.driver		= {
		.name	= "stmp3xxx-keyboard",
	},
};

static int __init stmpkbd_init(void)
{
	return platform_driver_register(&stmpkbd_driver);
}

static void __exit stmpkbd_exit(void)
{
	platform_driver_unregister(&stmpkbd_driver);
}

module_init(stmpkbd_init);
module_exit(stmpkbd_exit);
MODULE_DESCRIPTION("Freescale STMP3xxxx keyboard driver");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_LICENSE("GPL");
