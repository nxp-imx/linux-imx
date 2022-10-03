/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Driver types and constants for pca995x LED bindings
 *
 * Copyright 2011 bct electronic GmbH
 * Copyright 2013 Qtechnology/AS
 * Copyright 2022 NXP
 */

#ifndef __LINUX_PCA995X_H
#define __LINUX_PCA995X_H
#include <linux/leds.h>

enum pca995x_blink_type {
	PCA995X_SW_BLINK,
	PCA995X_HW_BLINK,
};

struct pca995x_platform_data {
	struct led_platform_data leds;
	enum pca995x_blink_type blink_type;
};

#endif /* __LINUX_PCA995X_H*/
