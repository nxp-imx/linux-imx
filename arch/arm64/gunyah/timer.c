// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <asm/arch_timer.h>
#include <linux/clocksource.h>
#include <linux/gunyah.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/types.h>

/**
 * struct gunyah_timer_scale - Scaling factors for timer conversion
 * @mult: Multiplier for converting cycles to nanoseconds
 * @shift: Shift value for the multiplier
 *
 * These values are pre-calculated during initialization to efficiently
 * convert hypervisor tick deadlines to kernel time without performing
 * expensive division operations on the hot path.
 */
static struct {
	u32 mult;
	u32 shift;
} gunyah_timer_scale;

/**
 * gunyah_arch_timer_init() - Initialize arch-specific timer scaling factors
 *
 * Calculates the mult/shift factors needed to convert system counter ticks
 * to nanoseconds. This follows the same pattern as clocksource_cyc2ns().
 *
 * Returns: 0 on success, -EINVAL if counter frequency is unavailable
 */
int gunyah_arch_timer_init(void)
{
	u32 cntfrq = arch_timer_get_cntfrq();

	if (unlikely(!cntfrq)) {
		pr_err("gunyah: Failed to get ARM timer frequency\n");
		return -EINVAL;
	}

	/* Calculate mult/shift factors for efficient cycle-to-ns conversion */
	clocks_calc_mult_shift(&gunyah_timer_scale.mult,
			       &gunyah_timer_scale.shift,
			       cntfrq,
			       NSEC_PER_SEC,
			       300);

	return 0;
}
EXPORT_SYMBOL_GPL(gunyah_arch_timer_init);

/**
 * gunyah_arch_deadline_to_ktime() - Convert hypervisor tick deadline to CLOCK_MONOTONIC ktime
 * @timeout_ticks: Absolute deadline in system counter ticks
 * @expires:       Output ktime, set only when the deadline is still in the future
 *
 * Converts a hypervisor-provided absolute deadline (in system counter ticks)
 * to a kernel ktime value suitable for use with hrtimer. Uses pre-calculated
 * scaling factors to efficiently perform the conversion.
 *
 * Returns: true and populates @expires if @timeout_ticks is still in the future.
 *          false if the deadline has already passed or the frequency is unavailable;
 *          the caller should re-enter the hypercall immediately without waiting.
 */
bool gunyah_arch_deadline_to_ktime(u64 timeout_ticks, ktime_t *expires)
{
	u64 now_ticks, delta_ns;

	/* If timer scaling factors are not initialized, fail gracefully */
	if (unlikely(!gunyah_timer_scale.mult))
		return false;

	/* Read the current system counter value */
	now_ticks = arch_timer_read_counter();

	/* Check if the deadline has already passed */
	if (timeout_ticks <= now_ticks)
		return false;

	/* Convert the tick delta to nanoseconds using pre-calculated factors */
	delta_ns = clocksource_cyc2ns(timeout_ticks - now_ticks,
				      gunyah_timer_scale.mult,
				      gunyah_timer_scale.shift);

	/* Convert to absolute ktime */
	*expires = ktime_add_ns(ktime_get(), delta_ns);
	return true;
}
EXPORT_SYMBOL_GPL(gunyah_arch_deadline_to_ktime);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Gunyah arm64 VCPU Timer support");
