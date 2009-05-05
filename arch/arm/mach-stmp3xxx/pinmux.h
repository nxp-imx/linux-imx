/*
 * Freescale STMP37XX/STMP378X Pin Multiplexing
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
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
#ifndef __PINMUX_H
#define __PINMUX_H

#include <linux/spinlock.h>
#include <linux/types.h>

/* Pin definitions */
#include <mach/pins.h>

/*
 * Each pin may be routed up to four different HW interfaces
 * including GPIO
 */
enum pin_fun {
	PIN_FUN1 = 0,
	PIN_FUN2,
	PIN_FUN3,
	PIN_GPIO,
};

/*
 * Each pin may have different output drive strength in range from
 * 4mA to 20mA. The most common case is 4, 8 and 12 mA strengths.
 */
enum pin_strength {
	PIN_4MA = 0,
	PIN_8MA,
	PIN_12MA,
	PIN_16MA,
	PIN_20MA,
};

/*
 * Each pin can be programmed for 1.8V or 3.3V
 */
enum pin_voltage {
	PIN_1_8V = 0,
	PIN_3_3V,
};

/*
 * Structure to define a group of pins and their parameters
 */
struct pin_desc {
	unsigned id;
	enum pin_fun fun;
	enum pin_strength strength;
	enum pin_voltage voltage;
	unsigned pullup:1;
};

struct pin_group {
	struct pin_desc *pins;
	int nr_pins;
};

/* Set pin drive strength */
void stmp3xxx_pin_strength(unsigned id, enum pin_strength strength,
			   char *label);

/* Set pin voltage */
void stmp3xxx_pin_voltage(unsigned id, enum pin_voltage voltage, char *label);

/* Enable pull-up resisitor for a pin */
void stmp3xxx_pin_pullup(unsigned id, int enable, char *label);

/*
 * Request a pin ownership, only one module (identified by @label)
 * may own a pin.
 */
int stmp3xxx_request_pin(unsigned id, enum pin_fun fun, char *label);

/* Release pin */
void stmp3xxx_release_pin(unsigned id, char *label);

void stmp3xxx_set_pin_type_chklock(unsigned id, enum pin_fun fun, int lock);
#define stmp3xxx_set_pin_type(id, fun) stmp37xx_set_pin_type_chklock(id, fun, 1)

/* Request and configure a group of pins */
int stmp3xxx_request_pin_group(struct pin_group *pin_group, char *label);

/* Release a group of pin for device */
void stmp3xxx_release_pin_group(struct pin_group *pin_group, char *label);

/* Configure pin as interrupt source */
void stmp3xxx_configure_irq(unsigned id, unsigned type);

/* Acknowledge irq in pinctrl registers */
void stmp3xxx_pin_ack_irq(int irq);

/*
 * Each bank is associated with a number of registers to control
 * pin function, drive strength, voltage and pull-up reigster. The
 * number of registers of a given type depends on the number of bits
 * describin particular pin.
 */
#define HW_MUXSEL_NUM		2	/* registers per bank */
#define HW_MUXSEL_PIN_LEN	2	/* bits per pin */
#define HW_MUXSEL_PIN_NUM	16	/* pins per register */
#define HW_MUXSEL_PINFUN_MASK	0x3	/* pin function mask */
#define HW_MUXSEL_PINFUN_NUM	4	/* four options for a pin */

#define HW_DRIVE_NUM		4	/* registers per bank */
#define HW_DRIVE_PIN_LEN	4	/* bits per pin */
#define HW_DRIVE_PIN_NUM	8	/* pins per register */
#define HW_DRIVE_PINDRV_MASK	0x3	/* pin strength mask - 2 bits */
#define HW_DRIVE_PINDRV_NUM	5	/* five possible strength values */
#define HW_DRIVE_PINV_MASK	0x4	/* pin voltage mask - 1 bit */


struct stmp3xxx_pinmux_bank {
	/* Pins allocation map */
	unsigned long pin_map;

	/* Pin owner names */
	char *pin_labels[STMP3XXX_PINMUX_BANK_SIZE];

	/* spin lock to protect an access to pin map */
	spinlock_t lock;

	/* Bank registers */
	u32 hw_muxsel[HW_MUXSEL_NUM];
	u32 hw_drive[HW_DRIVE_NUM];
	u32 hw_pull;

	u32 pin2irq, irqlevel, irqpolarity, irqen, irqstat;

	/* HW MUXSEL register function bit values */
	u8 functions[HW_MUXSEL_PINFUN_NUM];

	/*
	 * HW DRIVE register strength bit values:
	 * 0xff - requested strength is not supported for this bank
	 */
	u8 strengths[HW_DRIVE_PINDRV_NUM];

	/* GPIO things */
	u32 hw_gpio_read, hw_gpio_set, hw_gpio_clr, hw_gpio_doe;
	int irq;
};

struct stmp3xxx_pinmux_bank *stmp_pinmux_banks(int i);

#endif /* __PINMUX_H */
