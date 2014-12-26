/*
 * Copyright 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_BUSFREQ_H__
#define __ASM_ARCH_MXC_BUSFREQ_H__

#include <linux/notifier.h>
/*
 * This enumerates busfreq mode.
 */
enum busfreq_event {
	LOW_BUSFREQ_ENTER,
	LOW_BUSFREQ_EXIT,
};

enum bus_freq_mode {
	BUS_FREQ_HIGH,
	BUS_FREQ_MED,
	BUS_FREQ_AUDIO,
	BUS_FREQ_LOW,
};
void request_bus_freq(enum bus_freq_mode mode);
void release_bus_freq(enum bus_freq_mode mode);
int register_busfreq_notifier(struct notifier_block *nb);
int unregister_busfreq_notifier(struct notifier_block *nb);
#endif
