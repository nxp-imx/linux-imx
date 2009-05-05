/*
 * Freescale STMP37XX/STMP378X internal functions and data declarations
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
#ifndef __ASM_ARCH_MACH_STMP3XXX_COMMON_H
#define __ASM_ARCH_MACH_STMP3XXX_COMMON_H

#include <linux/irq.h>
#include <asm/mach/time.h>
#include "pinmux.h"

extern struct sys_timer stmp3xxx_timer;
int stmp3xxx_add_devices(void);
void stmp3xxx_init_irq(struct irq_chip *chip1,
		      struct irq_chip *chip2,
		      int (*is2)(int));
void stmp3xxx_init(void);
void stmp3xxx_set_mmc_data(struct device *dev);

void stmp37xx_map_io(void);
void stmp37xx_init_irq(void);
int stmp37xx_add_devices(void);

void stmp378x_map_io(void);
void stmp378x_init_irq(void);
void stmp378x_set_mmc_data(struct device *dev);

extern struct pin_group i2c_pins;
extern struct pin_group appuart_pins[];
extern struct pin_group dbguart_pins[];
extern struct pin_group gpmi_pins;
extern struct pin_group stmp37xx_lcd_pins;
extern struct pin_group stmp378x_lcd_pins;
extern unsigned stmp37xx_lcd_spi_pins[];
extern unsigned stmp378x_lcd_spi_pins[];
extern struct pin_group usb_mux_pins;
extern struct pin_group spdif_pins;

/* pm.c */
extern int stmp_s2ram_alloc_sz;
void stmp37xx_cpu_suspend(void);
extern int stmp_standby_alloc_sz;
void stmp37xx_cpu_standby(void);
void stmp3xxx_suspend_timer(void);
void stmp3xxx_resume_timer(void);

/* SPI */
extern int stmp37xx_spi_enc_init(void *);
extern int stmp37xx_spi_enc_release(void *);


#endif /* __ASM_ARCH_MACH_STMP3XXXL_COMMON_H */
