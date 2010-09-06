/*
 * Freescale STMP37XX/STMP378X LRADC helper routines
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>

struct lradc_device {
	struct sys_device sys;
	unsigned int base;
	unsigned int vddio_voltage;
	unsigned int battery_voltage;
};

static int channels[8];

static __refdata struct lradc_device mxs_lradc;

int hw_lradc_use_channel(int channel)
{
	if (channel < 0 || channel > 7)
		return -EINVAL;
	channels[channel]++;
	return 0;
}
EXPORT_SYMBOL(hw_lradc_use_channel);

int hw_lradc_unuse_channel(int channel)
{
	if (channel < 0 || channel > 7)
		return -EINVAL;
	channels[channel]--;
	return 0;
}
EXPORT_SYMBOL(hw_lradc_unuse_channel);

void hw_lradc_reinit(int enable_ground_ref, unsigned freq)
{
	__raw_writel(BM_LRADC_CTRL0_SFTRST,
		     mxs_lradc.base + HW_LRADC_CTRL0_SET);
	udelay(1);
	__raw_writel(BM_LRADC_CTRL0_SFTRST,
		     mxs_lradc.base + HW_LRADC_CTRL0_CLR);

	/* Clear the Clock Gate for normal operation */
	__raw_writel(BM_LRADC_CTRL0_CLKGATE,
		     mxs_lradc.base + HW_LRADC_CTRL0_CLR);

	if (enable_ground_ref)
		__raw_writel(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
			     mxs_lradc.base + HW_LRADC_CTRL0_SET);
	else
		__raw_writel(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
			    mxs_lradc.base + HW_LRADC_CTRL0_CLR);

	__raw_writel(BM_LRADC_CTRL3_CYCLE_TIME,
		     mxs_lradc.base + HW_LRADC_CTRL3_CLR);
	__raw_writel(BF_LRADC_CTRL3_CYCLE_TIME(freq),
		     mxs_lradc.base + HW_LRADC_CTRL3_SET);

	__raw_writel(BM_LRADC_CTRL4_LRADC6SELECT | BM_LRADC_CTRL4_LRADC7SELECT,
		     mxs_lradc.base + HW_LRADC_CTRL4_CLR);
	__raw_writel(BF_LRADC_CTRL4_LRADC6SELECT(mxs_lradc.vddio_voltage),
		     mxs_lradc.base + HW_LRADC_CTRL4_SET);
	__raw_writel(BF_LRADC_CTRL4_LRADC7SELECT(mxs_lradc.battery_voltage),
		     mxs_lradc.base + HW_LRADC_CTRL4_SET);
}

int hw_lradc_init_ladder(int channel, int trigger, unsigned sampling)
{
	/*
	 * check if the lradc channel is present in this product
	 */
	if (!hw_lradc_present(channel))
		return -ENODEV;

	hw_lradc_configure_channel(channel, !0 /* div2 */ ,
				   0 /* acc */ ,
				   0 /* num_samples */);

	/* Setup the trigger loop forever */
	hw_lradc_set_delay_trigger(trigger, 1 << channel,
				   1 << trigger, 0, sampling);

	/* Clear the accumulator & NUM_SAMPLES */
	__raw_writel(0xFFFFFFFF, mxs_lradc.base + HW_LRADC_CHn_CLR(channel));
	return 0;
}

EXPORT_SYMBOL(hw_lradc_init_ladder);

int hw_lradc_stop_ladder(int channel, int trigger)
{
	/*
	 * check if the lradc channel is present in this product
	 */
	if (!hw_lradc_present(channel))
		return -ENODEV;
	hw_lradc_clear_delay_trigger(trigger, 1 << channel, 1 << trigger);
	return 0;
}

EXPORT_SYMBOL(hw_lradc_stop_ladder);

int hw_lradc_present(int channel)
{
	if (channel < 0 || channel > 7)
		return 0;
	return __raw_readl(mxs_lradc.base + HW_LRADC_STATUS)
	    & (1 << (16 + channel));
}

EXPORT_SYMBOL(hw_lradc_present);

void hw_lradc_configure_channel(int channel, int enable_div2,
				int enable_acc, int samples)
{
	if (enable_div2)
		__raw_writel(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << channel),
			     mxs_lradc.base + HW_LRADC_CTRL2_SET);
	else
		__raw_writel(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << channel),
			     mxs_lradc.base + HW_LRADC_CTRL2_CLR);

	/* Clear the accumulator & NUM_SAMPLES */
	__raw_writel(0xFFFFFFFF, mxs_lradc.base + HW_LRADC_CHn_CLR(channel));

	/* Sets NUM_SAMPLES bitfield of HW_LRADC_CHn register. */
	__raw_writel(BM_LRADC_CHn_NUM_SAMPLES,
		     mxs_lradc.base + HW_LRADC_CHn_CLR(channel));
	__raw_writel(BF_LRADC_CHn_NUM_SAMPLES(samples),
		     mxs_lradc.base + HW_LRADC_CHn_SET(channel));

	if (enable_acc)
		__raw_writel(BM_LRADC_CHn_ACCUMULATE,
			     mxs_lradc.base + HW_LRADC_CHn_SET(channel));
	else
		__raw_writel(BM_LRADC_CHn_ACCUMULATE,
			     mxs_lradc.base + HW_LRADC_CHn_CLR(channel));
}

EXPORT_SYMBOL(hw_lradc_configure_channel);

void hw_lradc_set_delay_trigger(int trigger, u32 trigger_lradc,
				u32 delay_triggers, u32 loops, u32 delays)
{
	/* set TRIGGER_LRADCS in HW_LRADC_DELAYn */
	__raw_writel(BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc),
		     mxs_lradc.base + HW_LRADC_DELAYn_SET(trigger));
	__raw_writel(BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers),
		     mxs_lradc.base + HW_LRADC_DELAYn_SET(trigger));

	__raw_writel(BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY,
		     mxs_lradc.base + HW_LRADC_DELAYn_CLR(trigger));
	__raw_writel(BF_LRADC_DELAYn_LOOP_COUNT(loops),
		     mxs_lradc.base  + HW_LRADC_DELAYn_SET(trigger));
	__raw_writel(BF_LRADC_DELAYn_DELAY(delays),
		     mxs_lradc.base + HW_LRADC_DELAYn_SET(trigger));
}

EXPORT_SYMBOL(hw_lradc_set_delay_trigger);

void hw_lradc_clear_delay_trigger(int trigger, u32 trigger_lradc,
				  u32 delay_triggers)
{
	__raw_writel(BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc),
		     mxs_lradc.base + HW_LRADC_DELAYn_CLR(trigger));
	__raw_writel(BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers),
		     mxs_lradc.base + HW_LRADC_DELAYn_CLR(trigger));
}

EXPORT_SYMBOL(hw_lradc_clear_delay_trigger);

void hw_lradc_set_delay_trigger_kick(int trigger, int value)
{
	if (value)
		__raw_writel(BM_LRADC_DELAYn_KICK,
			     mxs_lradc.base + HW_LRADC_DELAYn_SET(trigger));
	else
		__raw_writel(BM_LRADC_DELAYn_KICK,
			     mxs_lradc.base + HW_LRADC_DELAYn_CLR(trigger));
}

EXPORT_SYMBOL(hw_lradc_set_delay_trigger_kick);

u32 hw_lradc_vddio(void)
{
	/* Clear the Soft Reset and Clock Gate for normal operation */
	__raw_writel(BM_LRADC_CTRL0_SFTRST | BM_LRADC_CTRL0_CLKGATE,
		     mxs_lradc.base + HW_LRADC_CTRL0_CLR);

	/*
	 * Clear the divide by two for channel 6 since
	 * it has a HW divide-by-two built in.
	 */
	__raw_writel(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << VDDIO_VOLTAGE_CH),
		     mxs_lradc.base + HW_LRADC_CTRL2_CLR);

	/* Clear the accumulator & NUM_SAMPLES */
	__raw_writel(0xFFFFFFFF,
		     mxs_lradc.base + HW_LRADC_CHn_CLR(VDDIO_VOLTAGE_CH));

	/* Clear the interrupt flag */
	__raw_writel(BM_LRADC_CTRL1_LRADC6_IRQ,
		     mxs_lradc.base + HW_LRADC_CTRL1_CLR);

	/*
	 * Get VddIO; this is the max scale value for the button resistor
	 * ladder.
	 * schedule ch 6:
	 */
	__raw_writel(BF_LRADC_CTRL0_SCHEDULE(1 << VDDIO_VOLTAGE_CH),
		     mxs_lradc.base + HW_LRADC_CTRL0_SET);

	/* wait for completion */
	while ((__raw_readl(mxs_lradc.base + HW_LRADC_CTRL1)
		& BM_LRADC_CTRL1_LRADC6_IRQ) != BM_LRADC_CTRL1_LRADC6_IRQ)
		cpu_relax();

	/* Clear the interrupt flag */
	__raw_writel(BM_LRADC_CTRL1_LRADC6_IRQ,
		     mxs_lradc.base + HW_LRADC_CTRL1_CLR);

	/* read ch 6 value. */
	return __raw_readl(mxs_lradc.base + HW_LRADC_CHn(VDDIO_VOLTAGE_CH)) &
			   BM_LRADC_CHn_VALUE;
}

EXPORT_SYMBOL(hw_lradc_vddio);

#ifdef CONFIG_PM
static u32 lradc_registers[0x16];
static int do_gate;

static int hw_lradc_suspend(struct sys_device *dev, pm_message_t state)
{
	int i;

	do_gate = 1;
	for (i = 0; i < ARRAY_SIZE(channels); i++)
		if (channels[i] > 0) {
			do_gate = 0;
			break;
		}

	for (i = 0; i < ARRAY_SIZE(lradc_registers); i++)
		lradc_registers[i] = __raw_readl(mxs_lradc.base + (i << 4));

	if (do_gate)
		__raw_writel(BM_LRADC_CTRL0_CLKGATE,
			     mxs_lradc.base + HW_LRADC_CTRL0_SET);
	return 0;
}

static int hw_lradc_resume(struct sys_device *dev)
{
	int i;

	if (do_gate) {
		__raw_writel(BM_LRADC_CTRL0_SFTRST,
			     mxs_lradc.base + HW_LRADC_CTRL0_SET);
		udelay(10);
		__raw_writel(BM_LRADC_CTRL0_SFTRST |
			     BM_LRADC_CTRL0_CLKGATE,
			     mxs_lradc.base + HW_LRADC_CTRL0_CLR);
	}
	for (i = 0; i < ARRAY_SIZE(lradc_registers); i++)
		__raw_writel(lradc_registers[i], mxs_lradc.base + (i << 4));
	return 0;
}

#endif

static struct sysdev_class mxs_lradc_sysclass = {
	.name = "mxs-lradc",
#ifdef CONFIG_PM
	.suspend = hw_lradc_suspend,
	.resume = hw_lradc_resume,
#endif
};

static int lradc_freq = LRADC_CLOCK_6MHZ;

static int __init lradc_freq_setup(char *str)
{
	long freq;

	if (strict_strtol(str, 0, &freq) < 0)
		return 0;

	if (freq < 0)
		return 0;
	if (freq >= 6)
		lradc_freq = LRADC_CLOCK_6MHZ;
	else if (freq >= 4)
		lradc_freq = LRADC_CLOCK_4MHZ;
	else if (freq >= 3)
		lradc_freq = LRADC_CLOCK_3MHZ;
	else if (freq >= 2)
		lradc_freq = LRADC_CLOCK_2MHZ;
	else
		return 0;
	return 1;
}

__setup("lradc_freq=", lradc_freq_setup);

static int __devinit mxs_lradc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mxs_lradc_plat_data *plat_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	plat_data = (struct mxs_lradc_plat_data *)(pdev->dev.platform_data);
	if (plat_data == NULL)
		return -EFAULT;

	mxs_lradc.base = (unsigned int)IO_ADDRESS(res->start);
	mxs_lradc.sys.id = -1;
	mxs_lradc.sys.cls = &mxs_lradc_sysclass;
	mxs_lradc.vddio_voltage = plat_data->vddio_voltage;
	mxs_lradc.battery_voltage = plat_data->battery_voltage;
	hw_lradc_reinit(0, lradc_freq);
	return sysdev_register(&mxs_lradc.sys);
}

static int __devexit mxs_lradc_remove(struct platform_device *pdev)
{
	sysdev_unregister(&mxs_lradc.sys);
	return 0;
}

static __refdata struct platform_driver mxs_lradc_drv = {
	.probe = mxs_lradc_probe,
	.remove = __devexit_p(mxs_lradc_remove),
	.driver = {
		.name = "mxs-lradc",
		.owner = THIS_MODULE,
	}
};

static int __init hw_lradc_init(void)
{
	sysdev_class_register(&mxs_lradc_sysclass);
	platform_driver_register(&mxs_lradc_drv);
	return 0;
}

static void __exit hw_lradc_exit(void)
{
	platform_driver_unregister(&mxs_lradc_drv);
	sysdev_class_unregister(&mxs_lradc_sysclass);
}

subsys_initcall(hw_lradc_init);
module_exit(hw_lradc_exit);
