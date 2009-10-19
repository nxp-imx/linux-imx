/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * The LED can be used for debugging purpose. To enalbe the LEDs, in the
 * config file, select:
 * CONFIG_LEDS
 * CONFIG_LEDS_TIMER   --- enable the OS tick LED once every 50 ticks (.5sec)
 * CONFIG_LEDS_CPU     --- enable the cpu idle in/out LED (blink fast)
 *
 * The two LEDs can be disabled through user space by issuing:
 *      echo "claim" > /sys/devices/system/leds/leds0/event
 * To release the LEDs back to the normal operation, do:
 *      echo "release" > /sys/devices/system/leds/leds0/event
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/leds.h>

#define LED_STATE_ENABLED	(1 << 0)
#define LED_STATE_CLAIMED	(1 << 1)

static unsigned int led_state;
static unsigned int hw_led_state;

static void mxc_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch (evt) {
	case led_start:
		hw_led_state = MXC_BD_LED1 | MXC_BD_LED2;
		led_state = LED_STATE_ENABLED;
		break;

	case led_stop:
	case led_halted:
		hw_led_state = 0;
		led_state &= ~LED_STATE_ENABLED;
		MXC_BD_LED_OFF(MXC_BD_LED1 | MXC_BD_LED2);
		break;

	case led_claim:
		led_state |= LED_STATE_CLAIMED;
		hw_led_state = 0;
		break;

	case led_release:
		led_state &= ~LED_STATE_CLAIMED;
		hw_led_state = 0;
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state ^= MXC_BD_LED1;
		break;
#endif

#ifdef CONFIG_LEDS_CPU
	case led_idle_start:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state &= ~MXC_BD_LED2;
		break;

	case led_idle_end:
		if (!(led_state & LED_STATE_CLAIMED))
			hw_led_state |= MXC_BD_LED2;
		break;
#endif

	default:
		break;
	}

	if (led_state & LED_STATE_ENABLED) {
		MXC_BD_LED_OFF(~hw_led_state);
		MXC_BD_LED_ON(hw_led_state);
	}

	local_irq_restore(flags);
}

static int __init mxc_leds_init(void)
{
	led_state = LED_STATE_ENABLED;
	leds_event = mxc_leds_event;
	return 0;
}

core_initcall(mxc_leds_init);
