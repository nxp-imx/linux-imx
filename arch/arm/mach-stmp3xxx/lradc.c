/*
 * Freescale STMP37XX/STMP378X LRADC helper routines
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <linux/delay.h>
#include <mach/regs-lradc.h>
#include <mach/lradc.h>

static int channels[8];

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
	HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_SFTRST);
	udelay(1);
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_SFTRST);

	/* Clear the Clock Gate for normal operation */
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_CLKGATE);

	if (enable_ground_ref)
		HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_ONCHIP_GROUNDREF);
	else
		HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_ONCHIP_GROUNDREF);

	HW_LRADC_CTRL3_CLR(BM_LRADC_CTRL3_CYCLE_TIME);
	HW_LRADC_CTRL3_SET(BF_LRADC_CTRL3_CYCLE_TIME(freq));

	HW_LRADC_CTRL4_CLR(BM_LRADC_CTRL4_LRADC6SELECT |
			BM_LRADC_CTRL4_LRADC7SELECT);
	HW_LRADC_CTRL4_SET(BF_LRADC_CTRL4_LRADC6SELECT(VDDIO_VOLTAGE_CH));
	HW_LRADC_CTRL4_SET(BF_LRADC_CTRL4_LRADC7SELECT(BATTERY_VOLTAGE_CH));
}

int hw_lradc_init_ladder(int channel, int trigger, unsigned sampling)
{
	/*
	 * check if the lradc channel is present in this product
	 */
	if (!hw_lradc_present(channel))
		return -ENODEV;

	hw_lradc_configure_channel(channel,
			!0 /* div2 */,
			 0 /* acc */,
			 0 /* num_samples */);

	/* Setup the trigger loop forever */
	hw_lradc_set_delay_trigger(trigger, 1<<channel,
			1<<trigger, 0, sampling);

	/* Clear the accumulator & NUM_SAMPLES */
	HW_LRADC_CHn_CLR(channel, 0xFFFFFFFF);
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
	hw_lradc_clear_delay_trigger(trigger, 1<<channel, 1<<trigger);
	return 0;
}
EXPORT_SYMBOL(hw_lradc_stop_ladder);

int hw_lradc_present(int channel)
{
	if (channel < 0 || channel > 7)
		return 0;
	return HW_LRADC_STATUS_RD() & (1<<(16+channel));
}
EXPORT_SYMBOL(hw_lradc_present);

void hw_lradc_configure_channel(int channel, int enable_div2,
		int enable_acc, int samples)
{
	if (enable_div2)
		HW_LRADC_CTRL2_SET(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1<<channel));
	else
		HW_LRADC_CTRL2_CLR(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1<<channel));

	/* Clear the accumulator & NUM_SAMPLES */
	HW_LRADC_CHn_CLR(channel, 0xFFFFFFFF);

	/* Sets NUM_SAMPLES bitfield of HW_LRADC_CHn register. */
	HW_LRADC_CHn_CLR(channel, BM_LRADC_CHn_NUM_SAMPLES);
	HW_LRADC_CHn_SET(channel, BF_LRADC_CHn_NUM_SAMPLES(samples));

	if (enable_acc)
		HW_LRADC_CHn_SET(channel, BM_LRADC_CHn_ACCUMULATE);
	else
		HW_LRADC_CHn_CLR(channel, BM_LRADC_CHn_ACCUMULATE);
}
EXPORT_SYMBOL(hw_lradc_configure_channel);

void hw_lradc_set_delay_trigger(int trigger, u32 trigger_lradc,
		u32 delay_triggers, u32 loops, u32 delays)
{
	/* set TRIGGER_LRADCS in HW_LRADC_DELAYn */
	HW_LRADC_DELAYn_SET(trigger,
			BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc));
	HW_LRADC_DELAYn_SET(trigger,
			BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers));

	HW_LRADC_DELAYn_CLR(trigger,
			BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY);
	HW_LRADC_DELAYn_SET(trigger,
			BF_LRADC_DELAYn_LOOP_COUNT(loops));
	HW_LRADC_DELAYn_SET(trigger,
			BF_LRADC_DELAYn_DELAY(delays));
}
EXPORT_SYMBOL(hw_lradc_set_delay_trigger);

void hw_lradc_clear_delay_trigger(int trigger, u32 trigger_lradc,
		u32 delay_triggers)
{
	HW_LRADC_DELAYn_CLR(trigger,
			BF_LRADC_DELAYn_TRIGGER_LRADCS(trigger_lradc));
	HW_LRADC_DELAYn_CLR(trigger,
			BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_triggers));
}
EXPORT_SYMBOL(hw_lradc_clear_delay_trigger);

void hw_lradc_set_delay_trigger_kick(int trigger, int value)
{
	if (value)
		HW_LRADC_DELAYn_SET(trigger, BM_LRADC_DELAYn_KICK);
	else
		HW_LRADC_DELAYn_CLR(trigger, BM_LRADC_DELAYn_KICK);
}
EXPORT_SYMBOL(hw_lradc_set_delay_trigger_kick);

u32 hw_lradc_vddio(void)
{
	/* Clear the Soft Reset and Clock Gate for normal operation */
	HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_SFTRST | BM_LRADC_CTRL0_CLKGATE);

	/*
	 * Clear the divide by two for channel 6 since
	 * it has a HW divide-by-two built in.
	 */
	HW_LRADC_CTRL2_CLR(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1<<VDDIO_VOLTAGE_CH));

	/* Clear the accumulator & NUM_SAMPLES */
	HW_LRADC_CHn_CLR(VDDIO_VOLTAGE_CH, 0xFFFFFFFF);

	/* Clear the interrupt flag */
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC6_IRQ);

	/*
	 * Get VddIO; this is the max scale value for the button resistor
	 * ladder.
	 * schedule ch 6:
	 */
	HW_LRADC_CTRL0_SET(BF_LRADC_CTRL0_SCHEDULE(1<<VDDIO_VOLTAGE_CH));

	/* wait for completion */
	while ((HW_LRADC_CTRL1_RD() & BM_LRADC_CTRL1_LRADC6_IRQ) !=
			BM_LRADC_CTRL1_LRADC6_IRQ)
		cpu_relax();

	/* Clear the interrupt flag */
	HW_LRADC_CTRL1_CLR(BM_LRADC_CTRL1_LRADC6_IRQ);

	/* read ch 6 value. */
	return HW_LRADC_CHn_RD(6) & BM_LRADC_CHn_VALUE;
}
EXPORT_SYMBOL(hw_lradc_vddio);

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
		lradc_registers[i] = __raw_readl(REGS_LRADC_BASE + (i << 4));

	if (do_gate)
		HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_CLKGATE);
	return 0;
}

static int hw_lradc_resume(struct sys_device *dev)
{
	int i;

	if (do_gate) {
		HW_LRADC_CTRL0_SET(BM_LRADC_CTRL0_SFTRST);
		udelay(10);
		HW_LRADC_CTRL0_CLR(BM_LRADC_CTRL0_SFTRST |
				   BM_LRADC_CTRL0_CLKGATE);
	}
	for (i = 0; i < ARRAY_SIZE(lradc_registers); i++)
		__raw_writel(lradc_registers[i], REGS_LRADC_BASE + (i << 4));
	return 0;
}

static struct sysdev_class stmp3xxx_lradc_sysclass = {
	.name		= "stmp3xxx-lradc",
#ifdef CONFIG_PM
	.suspend	= hw_lradc_suspend,
	.resume	 = hw_lradc_resume,
#endif
};

static struct sys_device stmp3xxx_lradc_device = {
	.id = -1,
	.cls = &stmp3xxx_lradc_sysclass,
};

static int __initdata lradc_freq  = LRADC_CLOCK_6MHZ;

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

static int __init hw_lradc_init(void)
{
	hw_lradc_reinit(0, lradc_freq);
	sysdev_class_register(&stmp3xxx_lradc_sysclass);
	sysdev_register(&stmp3xxx_lradc_device);
	return 0;
}
subsys_initcall(hw_lradc_init);
