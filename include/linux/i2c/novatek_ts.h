/* Copyright (C) 2012 Freescale Semiconductor, Inc. */

#ifndef NOVATEK_TS_H
#define NOVATEK_TS_H



/**
 * struct novatek_platform_data - platform data for novatek touch screen chip.
 * @reset_gpio: gpio for chip reset pin
 */
struct novatek_platform_data {
	int reset_gpio;
};

#endif
