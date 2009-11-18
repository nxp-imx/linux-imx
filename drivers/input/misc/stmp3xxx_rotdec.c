/*
 * Freescale STMP3XXX Rotary Encoder Driver
 *
 * Author: Drew Benedetti <drewb@embeddedalley.com>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <mach/regs-timrot.h>
#include <mach/rotdec.h>
#include <mach/platform.h>

static int relative;
static unsigned int poll_interval = 500;

void stmp3xxx_rotdec_flush(struct input_polled_dev *dev)
{
	/* in relative mode, reading the counter resets it */
	if (relative)
		__raw_readl(REGS_TIMROT_BASE + HW_TIMROT_ROTCOUNT);
}

void stmp3xxx_rotdec_poll(struct input_polled_dev *dev)
{
	s16 cnt = __raw_readl(REGS_TIMROT_BASE + HW_TIMROT_ROTCOUNT) & BM_TIMROT_ROTCOUNT_UPDOWN;
	if (relative)
		input_report_rel(dev->input, REL_WHEEL, cnt);
	else
		input_report_abs(dev->input, ABS_WHEEL, cnt);
}

struct input_polled_dev *rotdec;
static u32 rotctrl;

static int stmp3xxx_rotdec_probe(struct platform_device *pdev)
{
	int rc = 0;

	/* save original state of HW_TIMROT_ROTCTRL */
	rotctrl = __raw_readl(REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL);

	if (!(rotctrl & BM_TIMROT_ROTCTRL_ROTARY_PRESENT)) {
		dev_info(&pdev->dev, "No rotary decoder present\n");
		rc = -ENODEV;
		goto err_rotdec_present;
	} else {
		/* I had to add some extra line breaks in here
		 * to avoid lines >80 chars wide
		 */
		__raw_writel(
		 BF(0x0, TIMROT_ROTCTRL_DIVIDER) | /* 32kHz divider - 1 */
		 BF(BV_TIMROT_ROTCTRL_OVERSAMPLE__2X,
			TIMROT_ROTCTRL_OVERSAMPLE) |
		 BF(BV_TIMROT_ROTCTRL_SELECT_B__ROTARYB,
			TIMROT_ROTCTRL_SELECT_B) |
		 BF(BV_TIMROT_ROTCTRL_SELECT_A__ROTARYA,
			TIMROT_ROTCTRL_SELECT_A)
		, REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL);
		__raw_writel(
		 BM_TIMROT_ROTCTRL_POLARITY_B |
		 BM_TIMROT_ROTCTRL_POLARITY_A
		, REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_CLR);

		if (relative)
			__raw_writel(BM_TIMROT_ROTCTRL_RELATIVE,
				REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_SET);
		else
			__raw_writel(BM_TIMROT_ROTCTRL_RELATIVE,
				REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL_CLR);

		rc = rotdec_pinmux_request();
		if (rc) {
			dev_err(&pdev->dev,
				"Pin request failed (err=%d)\n", rc);
			goto err_pinmux;
		}

		/* set up input_polled_dev */
		rotdec = input_allocate_polled_device();
		if (!rotdec) {
			dev_err(&pdev->dev,
				"Unable to allocate polled device\n");
			rc = -ENOMEM;
			goto err_alloc_polldev;
		}
		rotdec->flush = stmp3xxx_rotdec_flush;
		rotdec->poll = stmp3xxx_rotdec_poll;
		rotdec->poll_interval = poll_interval; /* msec */

		rotdec->input->name = "stmp3xxx-rotdec";
		if (relative)
			input_set_capability(rotdec->input, EV_REL, REL_WHEEL);
		else {
			input_set_capability(rotdec->input, EV_ABS, ABS_WHEEL);
			input_set_abs_params(rotdec->input, ABS_WHEEL,
					-32768, 32767, 0, 0);
		}

		rc = input_register_polled_device(rotdec);
		if (rc) {
			dev_err(&pdev->dev,
				"Unable to register rotary decoder (err=%d)\n",
				rc);
			goto err_reg_polldev;
		}
	}

	return 0;

err_reg_polldev:
	input_free_polled_device(rotdec);
err_alloc_polldev:
	rotdec_pinmux_free();
err_pinmux:
	/* restore original register state */
	__raw_writel(rotctrl, REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL);

err_rotdec_present:
	return rc;
}

static int stmp3xxx_rotdec_remove(struct platform_device *pdev)
{
	input_unregister_polled_device(rotdec);
	input_free_polled_device(rotdec);

	rotdec_pinmux_free();

	/* restore original register state */
	__raw_writel(rotctrl, REGS_TIMROT_BASE + HW_TIMROT_ROTCTRL);

	return 0;
}

static struct platform_driver stmp3xxx_rotdec_driver = {
	.probe	= stmp3xxx_rotdec_probe,
	.remove	= stmp3xxx_rotdec_remove,
	.driver	= {
		.name	= "stmp3xxx-rotdec",
	},
};

static int __init stmp3xxx_rotdec_init(void)
{
	return platform_driver_register(&stmp3xxx_rotdec_driver);
}

static void __exit stmp3xxx_rotdec_exit(void)
{
	platform_driver_unregister(&stmp3xxx_rotdec_driver);
}

module_init(stmp3xxx_rotdec_init);
module_exit(stmp3xxx_rotdec_exit);

module_param(relative, bool, 0600);
module_param(poll_interval, uint, 0600);

MODULE_AUTHOR("Drew Benedetti <drewb@embeddedalley.com>");
MODULE_DESCRIPTION("STMP3xxx rotary decoder driver");
MODULE_LICENSE("GPL");
