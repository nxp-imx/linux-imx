/* For mc34708's pmic driver
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc.
 *
 * based on:
 * Copyright 2009-2010 Pengutronix, Uwe Kleine-Koenig
 * <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#ifndef __LINUX_MFD_MC_PMIC_H
#define __LINUX_MFD_MC_PMIC_H

#include <linux/interrupt.h>

struct mc_pmic;

void mc_pmic_lock(struct mc_pmic *mc_pmic);
void mc_pmic_unlock(struct mc_pmic *mc_pmic);

int mc_pmic_reg_read(struct mc_pmic *mc_pmic, unsigned int offset, u32 * val);
int mc_pmic_reg_write(struct mc_pmic *mc_pmic, unsigned int offset, u32 val);
int mc_pmic_reg_rmw(struct mc_pmic *mc_pmic, unsigned int offset,
		    u32 mask, u32 val);

int mc_pmic_get_flags(struct mc_pmic *mc_pmic);

int mc_pmic_irq_request(struct mc_pmic *mc_pmic, int irq,
			irq_handler_t handler, const char *name, void *dev);
int mc_pmic_irq_request_nounmask(struct mc_pmic *mc_pmic, int irq,
				 irq_handler_t handler, const char *name,
				 void *dev);
int mc_pmic_irq_free(struct mc_pmic *mc_pmic, int irq, void *dev);

int mc_pmic_irq_mask(struct mc_pmic *mc_pmic, int irq);
int mc_pmic_irq_unmask(struct mc_pmic *mc_pmic, int irq);
int mc_pmic_irq_status(struct mc_pmic *mc_pmic, int irq,
		       int *enabled, int *pending);
int mc_pmic_irq_ack(struct mc_pmic *mc_pmic, int irq);

int mc_pmic_get_flags(struct mc_pmic *mc_pmic);

#ifdef CONFIG_MFD_MC34708
#define MC34708_SW1A		0
#define MC34708_SW1B		1
#define MC34708_SW2		  2
#define MC34708_SW3		  3
#define MC34708_SW4A		4
#define MC34708_SW4B		5
#define MC34708_SW5		  6
#define MC34708_SWBST		7
#define MC34708_VPLL		8
#define MC34708_VREFDDR	9
#define MC34708_VUSB		10
#define MC34708_VUSB2		11
#define MC34708_VDAC		12
#define MC34708_VGEN1		13
#define MC34708_VGEN2		14
#define MC34708_REGU_NUM	15

#define MC34708_REG_INT_STATUS0 0
#define MC34708_REG_INT_MASK0	1
#define MC34708_REG_INT_STATUS1 3
#define MC34708_REG_INT_MASK1	4
#define MC34708_REG_IDENTIFICATION	7
#define MC_PMIC_REG_INT_MASK0	MC34708_REG_INT_MASK0
#define MC_PMIC_REG_INT_MASK1	MC34708_REG_INT_MASK1
#define MC_PMIC_REG_INT_STATUS0 MC34708_REG_INT_STATUS0
#define MC_PMIC_REG_INT_STATUS1	MC34708_REG_INT_STATUS1
#define MC_PMIC_REG_IDENTIFICATION	MC34708_REG_IDENTIFICATION
#endif

#define MC_PMIC_IRQ_ADCDONE	0
#define MC_PMIC_IRQ_TSDONE	1
#define MC_PMIC_IRQ_TSPENDET	2
#define MC_PMIC_IRQ_USBDET	3
#define MC_PMIC_IRQ_AUXDET	4
#define MC_PMIC_IRQ_USBOVP	5
#define MC_PMIC_IRQ_AUXOVP	6
#define MC_PMIC_IRQ_CHRTIMEEXP	7
#define MC_PMIC_IRQ_BATTOTP	8
#define MC_PMIC_IRQ_BATTOVP	9
#define MC_PMIC_IRQ_CHRCMPL	10
#define MC_PMIC_IRQ_WKVBUSDET	11
#define MC_PMIC_IRQ_WKAUXDET	12
#define MC_PMIC_IRQ_LOWBATT	13
#define MC_PMIC_IRQ_VBUSREGMI	14
#define MC_PMIC_IRQ_ATTACH	15
#define MC_PMIC_IRQ_DETACH	16
#define MC_PMIC_IRQ_KP	17
#define MC_PMIC_IRQ_LKP	18
#define MC_PMIC_IRQ_LKR	19
#define MC_PMIC_IRQ_UKNOW_ATTA	20
#define MC_PMIC_IRQ_ADC_CHANGE	21
#define MC_PMIC_IRQ_STUCK_KEY	22
#define MC_PMIC_IRQ_STUCK_KEY_RCV	23
#define MC_PMIC_IRQ_1HZ	24
#define MC_PMIC_IRQ_TODA 25
#define MC_PMIC_IRQ_UNUSED1	26
#define MC_PMIC_IRQ_PWRON1	27
#define MC_PMIC_IRQ_PWRON2	28
#define MC_PMIC_IRQ_WDIRESET	29
#define MC_PMIC_IRQ_SYSRST	30
#define MC_PMIC_IRQ_RTCRST	31
#define MC_PMIC_IRQ_PCI	32
#define MC_PMIC_IRQ_WARM	33
#define MC_PMIC_IRQ_MEMHLD	34
#define MC_PMIC_IRQ_UNUSED2	35
#define MC_PMIC_IRQ_THWARNL	36
#define MC_PMIC_IRQ_THWARNH	37
#define MC_PMIC_IRQ_CLK	38
#define MC_PMIC_IRQ_UNUSED3	39
#define MC_PMIC_IRQ_SCP	40
#define MC_PMIC_NUMREGS	0x3f
#define MC_PMIC_NUM_IRQ	46

struct regulator_init_data;

struct mc_pmic_regulator_init_data {
	int id;
	struct regulator_init_data *init_data;
};

struct mc_pmic_regulator_platform_data {
	int num_regulators;
	struct mc_pmic_regulator_init_data *regulators;
};

struct mc_pmic_platform_data {
#define MC_PMIC_USE_TOUCHSCREEN (1 << 0)
#define MC_PMIC_USE_CODEC	(1 << 1)
#define MC_PMIC_USE_ADC		(1 << 2)
#define MC_PMIC_USE_RTC		(1 << 3)
#define MC_PMIC_USE_REGULATOR	(1 << 4)
#define MC_PMIC_USE_LED		(1 << 5)
	unsigned int flags;

	int num_regulators;
	struct mc_pmic_regulator_init_data *regulators;
};

#endif				/* ifndef __LINUX_MFD_MC_PMIC_H */
