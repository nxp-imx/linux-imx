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
#include <mach/platform.h>
#include <mach/stmp3xxx.h>
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
	stmp3xxx_setl(BM_LRADC_CTRL0_SFTRST, REGS_LRADC_BASE + HW_LRADC_CTRL0);
	udelay(1);
	stmp3xxx_clearl(BM_LRADC_CTRL0_SFTRST,
			REGS_LRADC_BASE + HW_LRADC_CTRL0);

	/* Clear the Clock Gate for normal operation */
	stmp3xxx_clearl(BM_LRADC_CTRL0_CLKGATE,
			REGS_LRADC_BASE + HW_LRADC_CTRL0);

	if (enable_ground_ref)
		stmp3xxx_setl(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
			      REGS_LRADC_BASE + HW_LRADC_CTRL0);
	else
		stmp3xxx_clearl(BM_LRADC_CTRL0_ONCHIP_GROUNDREF,
				REGS_LRADC_BASE + HW_LRADC_CTRL0);

	stmp3xxx_clearl(BM_LRADC_CTRL3_CYCLE_TIME,
			REGS_LRADC_BASE + HW_LRADC_CTRL3);
	stmp3xxx_setl(BF(freq, LRADC_CTRL3_CYCLE_TIME),
		      REGS_LRADC_BASE + HW_LRADC_CTRL3);

	stmp3xxx_clearl(BM_LRADC_CTRL4_LRADC6SELECT |
			BM_LRADC_CTRL4_LRADC7SELECT,
			REGS_LRADC_BASE + HW_LRADC_CTRL4);
	stmp3xxx_setl(BF(VDDIO_VOLTAGE_CH, LRADC_CTRL4_LRADC6SELECT),
		      REGS_LRADC_BASE + HW_LRADC_CTRL4);
	stmp3xxx_setl(BF(BATTERY_VOLTAGE_CH, LRADC_CTRL4_LRADC7SELECT),
		      REGS_LRADC_BASE + HW_LRADC_CTRL4);
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
				   0 /* num_samples */ );

	/* Setup the trigger loop forever */
	hw_lradc_set_delay_trigger(trigger, 1 << channel,
				   1 << trigger, 0, sampling);

	/* Clear the accumulator & NUM_SAMPLES */
	stmp3xxx_clearl(0xFFFFFFFF, REGS_LRADC_BASE + HW_LRADC_CHn(channel));
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
	return __raw_readl(REGS_LRADC_BASE + HW_LRADC_STATUS)
	    & (1 << (16 + channel));
}

EXPORT_SYMBOL(hw_lradc_present);

void hw_lradc_configure_channel(int channel, int enable_div2,
				int enable_acc, int samples)
{
	if (enable_div2)
		stmp3xxx_setl(BF(1 << channel, LRADC_CTRL2_DIVIDE_BY_TWO),
			      REGS_LRADC_BASE + HW_LRADC_CTRL2);
	else
		stmp3xxx_clearl(BF(1 << channel, LRADC_CTRL2_DIVIDE_BY_TWO),
				REGS_LRADC_BASE + HW_LRADC_CTRL2);

	/* Clear the accumulator & NUM_SAMPLES */
	stmp3xxx_clearl(0xFFFFFFFF, REGS_LRADC_BASE + HW_LRADC_CHn(channel));

	/* Sets NUM_SAMPLES bitfield of HW_LRADC_CHn register. */
	stmp3xxx_clearl(BM_LRADC_CHn_NUM_SAMPLES,
			REGS_LRADC_BASE + HW_LRADC_CHn(channel));
	stmp3xxx_setl(BF(samples, LRADC_CHn_NUM_SAMPLES),
		      REGS_LRADC_BASE + HW_LRADC_CHn(channel));

	if (enable_acc)
		stmp3xxx_setl(BM_LRADC_CHn_ACCUMULATE,
			      REGS_LRADC_BASE + HW_LRADC_CHn(channel));
	else
		stmp3xxx_clearl(BM_LRADC_CHn_ACCUMULATE,
				REGS_LRADC_BASE + HW_LRADC_CHn(channel));
}

EXPORT_SYMBOL(hw_lradc_configure_channel);

void hw_lradc_set_delay_trigger(int trigger, u32 trigger_lradc,
				u32 delay_triggers, u32 loops, u32 delays)
{
	/* set TRIGGER_LRADCS in HW_LRADC_DELAYn */
	stmp3xxx_setl(BF(trigger_lradc, LRADC_DELAYn_TRIGGER_LRADCS),
		      REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
	stmp3xxx_setl(BF(delay_triggers, LRADC_DELAYn_TRIGGER_DELAYS),
		      REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));

	stmp3xxx_clearl(BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY,
			REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
	stmp3xxx_setl(BF(loops, LRADC_DELAYn_LOOP_COUNT),
		      REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
	stmp3xxx_setl(BF(delays, LRADC_DELAYn_DELAY),
		      REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
}

EXPORT_SYMBOL(hw_lradc_set_delay_trigger);

void hw_lradc_clear_delay_trigger(int trigger, u32 trigger_lradc,
				  u32 delay_triggers)
{
	stmp3xxx_clearl(BF(trigger_lradc, LRADC_DELAYn_TRIGGER_LRADCS),
			REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
	stmp3xxx_clearl(BF(delay_triggers, LRADC_DELAYn_TRIGGER_DELAYS),
			REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
}

EXPORT_SYMBOL(hw_lradc_clear_delay_trigger);

void hw_lradc_set_delay_trigger_kick(int trigger, int value)
{
	if (value)
		stmp3xxx_setl(BM_LRADC_DELAYn_KICK,
			      REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
	else
		stmp3xxx_clearl(BM_LRADC_DELAYn_KICK,
				REGS_LRADC_BASE + HW_LRADC_DELAYn(trigger));
}

EXPORT_SYMBOL(hw_lradc_set_delay_trigger_kick);

u32 hw_lradc_vddio(void)
{
	/* Clear the Soft Reset and Clock Gate for normal operation */
	stmp3xxx_clearl(BM_LRADC_CTRL0_SFTRST | BM_LRADC_CTRL0_CLKGATE,
			REGS_LRADC_BASE + HW_LRADC_CTRL0);

	/*
	 * Clear the divide by two for channel 6 since
	 * it has a HW divide-by-two built in.
	 */
	stmp3xxx_clearl(BF(1 << VDDIO_VOLTAGE_CH, LRADC_CTRL2_DIVIDE_BY_TWO),
			REGS_LRADC_BASE + HW_LRADC_CTRL2);

	/* Clear the accumulator & NUM_SAMPLES */
	stmp3xxx_clearl(0xFFFFFFFF,
			REGS_LRADC_BASE + HW_LRADC_CHn(VDDIO_VOLTAGE_CH));

	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC6_IRQ,
			REGS_LRADC_BASE + HW_LRADC_CTRL1);

	/*
	 * Get VddIO; this is the max scale value for the button resistor
	 * ladder.
	 * schedule ch 6:
	 */
	stmp3xxx_setl(BF(1 << VDDIO_VOLTAGE_CH, LRADC_CTRL0_SCHEDULE),
		      REGS_LRADC_BASE + HW_LRADC_CTRL0);

	/* wait for completion */
	while ((__raw_readl(REGS_LRADC_BASE + HW_LRADC_CTRL1)
		& BM_LRADC_CTRL1_LRADC6_IRQ) != BM_LRADC_CTRL1_LRADC6_IRQ)
		cpu_relax();

	/* Clear the interrupt flag */
	stmp3xxx_clearl(BM_LRADC_CTRL1_LRADC6_IRQ,
			REGS_LRADC_BASE + HW_LRADC_CTRL1);

	/* read ch 6 value. */
	return __raw_readl(REGS_LRADC_BASE + HW_LRADC_CHn(6))
	    & BM_LRADC_CHn_VALUE;
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
		stmp3xxx_setl(BM_LRADC_CTRL0_CLKGATE,
			      REGS_LRADC_BASE + HW_LRADC_CTRL0);
	return 0;
}

static int hw_lradc_resume(struct sys_device *dev)
{
	int i;

	if (do_gate) {
		stmp3xxx_setl(BM_LRADC_CTRL0_SFTRST,
			      REGS_LRADC_BASE + HW_LRADC_CTRL0);
		udelay(10);
		stmp3xxx_clearl(BM_LRADC_CTRL0_SFTRST |
				BM_LRADC_CTRL0_CLKGATE,
				REGS_LRADC_BASE + HW_LRADC_CTRL0);
	}
	for (i = 0; i < ARRAY_SIZE(lradc_registers); i++)
		__raw_writel(lradc_registers[i], REGS_LRADC_BASE + (i << 4));
	return 0;
}

static struct sysdev_class stmp3xxx_lradc_sysclass = {
	.name = "stmp3xxx-lradc",
#ifdef CONFIG_PM
	.suspend = hw_lradc_suspend,
	.resume = hw_lradc_resume,
#endif
};

static struct sys_device stmp3xxx_lradc_device = {
	.id = -1,
	.cls = &stmp3xxx_lradc_sysclass,
};

static int __initdata lradc_freq = LRADC_CLOCK_6MHZ;

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
