/*
 * Freescale STMP378X/STMP378X Pin Multiplexing
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/string.h>
#include <linux/bitops.h>
#include <linux/sysdev.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <mach/regs-pinctrl.h>
#include "pinmux.h"


static struct stmp3xxx_pinmux_bank pinmux_banks[STMP3XXX_PINMUX_NR_BANKS] = {
	[0] = {
	       .hw_muxsel = {HW_PINCTRL_MUXSEL0_ADDR, HW_PINCTRL_MUXSEL1_ADDR},
	       .hw_drive = {HW_PINCTRL_DRIVE0_ADDR, HW_PINCTRL_DRIVE1_ADDR,
			    HW_PINCTRL_DRIVE2_ADDR, HW_PINCTRL_DRIVE3_ADDR},
	       .hw_pull = HW_PINCTRL_PULL0_ADDR,
	       .functions = {0x0, 0x1, 0x2, 0x3},
	       .strengths = {0x0, 0x1, 0x2, 0x3, 0xff},
	       .hw_gpio_read = HW_PINCTRL_DIN0_ADDR,
	       .hw_gpio_set = HW_PINCTRL_DOUT0_ADDR + HW_STMP3xxx_SET,
	       .hw_gpio_clr = HW_PINCTRL_DOUT0_ADDR + HW_STMP3xxx_CLR,
	       .hw_gpio_doe = HW_PINCTRL_DOE0_ADDR,
	       .irq = IRQ_GPIO0,

	       .pin2irq = HW_PINCTRL_PIN2IRQ0_ADDR,
	       .irqstat = HW_PINCTRL_IRQSTAT0_ADDR,
	       .irqlevel = HW_PINCTRL_IRQLEVEL0_ADDR,
	       .irqpolarity = HW_PINCTRL_IRQPOL0_ADDR,
	       .irqen = HW_PINCTRL_IRQEN0_ADDR,
	       },
	[1] = {
	       .hw_muxsel = {HW_PINCTRL_MUXSEL2_ADDR, HW_PINCTRL_MUXSEL3_ADDR},
	       .hw_drive = {HW_PINCTRL_DRIVE4_ADDR, HW_PINCTRL_DRIVE5_ADDR,
			    HW_PINCTRL_DRIVE6_ADDR, HW_PINCTRL_DRIVE7_ADDR},
	       .hw_pull = HW_PINCTRL_PULL1_ADDR,
	       .functions = {0x0, 0x1, 0x2, 0x3},
	       .strengths = {0x0, 0x1, 0x2, 0x3, 0xff},
		.hw_gpio_read = HW_PINCTRL_DIN1_ADDR,
		.hw_gpio_set = HW_PINCTRL_DOUT1_ADDR + HW_STMP3xxx_SET,
		.hw_gpio_clr = HW_PINCTRL_DOUT1_ADDR + HW_STMP3xxx_CLR,
		.hw_gpio_doe = HW_PINCTRL_DOE1_ADDR,
		.irq = IRQ_GPIO1,

	       .pin2irq = HW_PINCTRL_PIN2IRQ1_ADDR,
	       .irqstat = HW_PINCTRL_IRQSTAT1_ADDR,
	       .irqlevel = HW_PINCTRL_IRQLEVEL1_ADDR,
	       .irqpolarity = HW_PINCTRL_IRQPOL1_ADDR,
	       .irqen = HW_PINCTRL_IRQEN1_ADDR,

	       },
	[2] = {
	       .hw_muxsel = {HW_PINCTRL_MUXSEL4_ADDR, HW_PINCTRL_MUXSEL5_ADDR},
	       .hw_drive = {HW_PINCTRL_DRIVE8_ADDR, HW_PINCTRL_DRIVE9_ADDR,
			    HW_PINCTRL_DRIVE10_ADDR, HW_PINCTRL_DRIVE11_ADDR},
	       .hw_pull = HW_PINCTRL_PULL2_ADDR,
	       .functions = {0x0, 0x1, 0x2, 0x3},
	       .strengths = {0x0, 0x1, 0x2, 0x1, 0x2},
	       .hw_gpio_read = HW_PINCTRL_DIN2_ADDR,
	       .hw_gpio_set = HW_PINCTRL_DOUT2_ADDR + HW_STMP3xxx_SET,
	       .hw_gpio_clr = HW_PINCTRL_DOUT2_ADDR + HW_STMP3xxx_CLR,
		.hw_gpio_doe = HW_PINCTRL_DOE2_ADDR,
	       .irq = IRQ_GPIO2,

	       .pin2irq = HW_PINCTRL_PIN2IRQ2_ADDR,
	       .irqstat = HW_PINCTRL_IRQSTAT2_ADDR,
	       .irqlevel = HW_PINCTRL_IRQLEVEL2_ADDR,
	       .irqpolarity = HW_PINCTRL_IRQPOL2_ADDR,
	       .irqen = HW_PINCTRL_IRQEN2_ADDR,
	       },
	[3] = {
	       .hw_muxsel = {HW_PINCTRL_MUXSEL6_ADDR, HW_PINCTRL_MUXSEL7_ADDR},
	       .hw_drive = {HW_PINCTRL_DRIVE12_ADDR, HW_PINCTRL_DRIVE13_ADDR,
			    HW_PINCTRL_DRIVE14_ADDR, 0},
	       .hw_pull = HW_PINCTRL_PULL3_ADDR,
	       .functions = {0x0, 0x1, 0x2, 0x3},
	       .strengths = {0x0, 0x1, 0x2, 0x3, 0xff},
	       },
};

struct stmp3xxx_pinmux_bank *stmp_pinmux_banks(int i)
{
	BUG_ON(i < 0 || i > sizeof(pinmux_banks)/sizeof(pinmux_banks[0]));
	return &pinmux_banks[i];
}
/* Compares pin owner and caller labels */
static inline int stmp3xxx_check_owner(u32 bank, u32 pin, char *label)
{
	return label && strcmp(label, pinmux_banks[bank].pin_labels[pin]);
}

/* Check if requested pin is owned by caller */
static int stmp3xxx_check_pin(u32 bank, u32 pin, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank = &pinmux_banks[bank];

	if (!test_bit(pin, &pbank->pin_map)) {
		printk(KERN_WARNING
		       "%s: Accessing free pin %d:%d, caller %s\n",
		       __func__, bank, pin, label);

		return -EINVAL;
	}

	if (stmp3xxx_check_owner(bank, pin, label)) {
		printk(KERN_WARNING
		       "%s: Wrong pin owner %d:%d, caller %s owner %s\n",
		       __func__, bank, pin, label, pbank->pin_labels[pin]);

		return -EINVAL;
	}

	return 0;
}

void stmp3xxx_pin_strength(unsigned id, enum pin_strength strength, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 hwdrive, shift, val;
	u32 bank, pin;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pr_debug("%s: label %s bank %d pin %d strength %d\n", __func__, label,
		 bank, pin, strength);

	pbank = &pinmux_banks[bank];

	hwdrive = pbank->hw_drive[pin / HW_DRIVE_PIN_NUM];
	shift = (pin % HW_DRIVE_PIN_NUM) * HW_DRIVE_PIN_LEN;
	val = pbank->strengths[strength];
	if (val == 0xff) {
		printk(KERN_WARNING
		       "%s: strength is not supported for bank %d, caller %s",
		       __func__, bank, label);
		return;
	}

	spin_lock(&pbank->lock);
	if (stmp3xxx_check_pin(bank, pin, label)) {
		printk(KERN_WARNING "%s: Pin %d:%d access failure\n",
		       __func__, bank, pin);
		goto out;
	}

	pr_debug("%s: writing 0x%x to 0x%x register\n", __func__,
			val << shift, hwdrive);
	__raw_writel(HW_DRIVE_PINDRV_MASK << shift, hwdrive + HW_STMP3xxx_CLR);
	__raw_writel(val << shift, hwdrive + HW_STMP3xxx_SET);

out:
	spin_unlock(&pbank->lock);
}

void stmp3xxx_pin_voltage(unsigned id, enum pin_voltage voltage, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 hwdrive, shift;
	u32 bank, pin;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pr_debug("%s: label %s bank %d pin %d voltage %d\n", __func__, label,
		 bank, pin, voltage);

	pbank = &pinmux_banks[bank];

	hwdrive = pbank->hw_drive[pin / HW_DRIVE_PIN_NUM];
	shift = (pin % HW_DRIVE_PIN_NUM) * HW_DRIVE_PIN_LEN;

	spin_lock(&pbank->lock);
	if (stmp3xxx_check_pin(bank, pin, label)) {
		printk(KERN_WARNING "%s: Pin %d:%d access failure\n",
		       __func__, bank, pin);
		goto out;
	}

	pr_debug("%s: changing 0x%x bit in 0x%x register\n",
			__func__, HW_DRIVE_PINV_MASK << shift, hwdrive);
	if (voltage == PIN_1_8V)
		__raw_writel(HW_DRIVE_PINV_MASK << shift,
			     hwdrive + HW_STMP3xxx_CLR);
	else
		__raw_writel(HW_DRIVE_PINV_MASK << shift,
			     hwdrive + HW_STMP3xxx_SET);
out:
	spin_unlock(&pbank->lock);
}

void stmp3xxx_pin_pullup(unsigned id, int enable, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 hwpull;
	u32 bank, pin;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pr_debug("%s: label %s bank %d pin %d enable %d\n", __func__, label,
		 bank, pin, enable);

	pbank = &pinmux_banks[bank];

	hwpull = pbank->hw_pull;

	spin_lock(&pbank->lock);
	if (stmp3xxx_check_pin(bank, pin, label)) {
		printk(KERN_WARNING "%s: Pin %d:%d access failure\n",
		       __func__, bank, pin);
		goto out;
	}

	pr_debug("%s: changing 0x%x bit in 0x%x register\n",
			__func__, 1 << pin, hwpull);
	__raw_writel(1 << pin,
		     hwpull + (enable ? HW_STMP3xxx_SET : HW_STMP3xxx_CLR));
out:
	spin_unlock(&pbank->lock);
}

int stmp3xxx_request_pin(unsigned id, enum pin_fun fun, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 bank, pin;
	int ret = 0;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pr_debug("%s: label %s bank %d pin %d fun %d\n", __func__, label,
		 bank, pin, fun);

	pbank = &pinmux_banks[bank];

	spin_lock(&pbank->lock);
	if (test_bit(pin, &pbank->pin_map)) {
		printk(KERN_WARNING
		       "%s: CONFLICT DETECTED pin %d:%d caller %s owner %s\n",
		       __func__, bank, pin, label, pbank->pin_labels[pin]);
		ret = -EBUSY;
		goto out;
	}

	set_bit(pin, &pbank->pin_map);
	pbank->pin_labels[pin] = label;

	stmp3xxx_set_pin_type_chklock(id, fun, 0);

out:
	spin_unlock(&pbank->lock);
	return ret;
}

void stmp3xxx_set_pin_type_chklock(unsigned id, enum pin_fun fun, int lock)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 hwmux, shift, val;
	u32 bank, pin;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pbank = &pinmux_banks[bank];
	hwmux = pbank->hw_muxsel[pin / HW_MUXSEL_PIN_NUM];
	shift = (pin % HW_MUXSEL_PIN_NUM) * HW_MUXSEL_PIN_LEN;

	val = pbank->functions[fun];
	if (lock)
		spin_lock(&pbank->lock);
	shift = (pin % HW_MUXSEL_PIN_NUM) * HW_MUXSEL_PIN_LEN;
	pr_debug("%s: writing 0x%x to 0x%x register\n",
			__func__, val << shift, hwmux);
	__raw_writel(HW_MUXSEL_PINFUN_MASK << shift, hwmux + HW_STMP3xxx_CLR);
	__raw_writel(val << shift, hwmux + HW_STMP3xxx_SET);
	if (lock)
		spin_unlock(&pbank->lock);
}

void stmp3xxx_release_pin(unsigned id, char *label)
{
	struct stmp3xxx_pinmux_bank *pbank;
	u32 bank, pin;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);

	pr_debug("%s: label %s bank %d pin %d\n", __func__, label, bank, pin);

	pbank = &pinmux_banks[bank];

	spin_lock(&pbank->lock);
	if (stmp3xxx_check_pin(bank, pin, label)) {
		printk(KERN_WARNING "%s: Pin %d:%d access failure\n",
		       __func__, bank, pin);
		goto out;
	}

	clear_bit(pin, &pbank->pin_map);
	pbank->pin_labels[pin] = NULL;
out:
	spin_unlock(&pbank->lock);
}

int stmp3xxx_request_pin_group(struct pin_group *pin_group, char *label)
{

	struct pin_desc *pin;
	int p;
	int err = 0;

	/* Allocate and configure pins */
	for (p = 0; p < pin_group->nr_pins; p++) {
		pr_debug("%s: #%d\n", __func__, p);
		pin = &pin_group->pins[p];

		err = stmp3xxx_request_pin(pin->id, pin->fun, label);
		if (err)
			goto out_err;

		stmp3xxx_pin_strength(pin->id, pin->strength, label);
		stmp3xxx_pin_voltage(pin->id, pin->voltage, label);
		stmp3xxx_pin_pullup(pin->id, pin->pullup, label);
	}

	return 0;

out_err:
	/* Release allocated pins in case of error */
	while (--p >= 0) {
		pr_debug("%s: releasing #%d\n", __func__, p);
		stmp3xxx_release_pin(pin_group->pins[p].id, label);
	}
	return err;
}

void stmp3xxx_release_pin_group(struct pin_group *pin_group, char *label)
{
	struct pin_desc *pin;
	int p;

	for (p = 0; p < pin_group->nr_pins; p++) {
		pin = &pin_group->pins[p];
		stmp3xxx_release_pin(pin->id, label);
	}
}

/**
 * stmp3xxx_configure_irq
 *
 * Configure the pin as interrupt source
 *
 * id:		pin id
 * type:	interrupt type, as in linux/irq.h; IRQ_TYPE_NONE disables irq
 *
 **/

void stmp3xxx_configure_irq(unsigned id, unsigned type)
{
	struct stmp3xxx_pinmux_bank *pbank;
	unsigned pin, bank;
	unsigned m;
	int l, p;

	bank = STMP3XXX_PINID_TO_BANK(id);
	pin = STMP3XXX_PINID_TO_PINNUM(id);
	BUG_ON(bank >= STMP3XXX_PINMUX_NR_BANKS);
	pbank = &pinmux_banks[bank];

	spin_lock(&pbank->lock);
	if (stmp3xxx_check_pin(bank, pin, NULL)) {
		printk(KERN_WARNING "%s: Pin %d:%d access failure\n",
		       __func__, bank, pin);
		goto out;
	}

	pr_debug("%s: caller configures %d(%d:%d) as IRQ type 0x%x\n",
			__func__, id, bank, pin, type);
	m = 1<<pin;

	if (type == IRQ_TYPE_NONE) {
		pr_debug("%s: cleared\n", __func__);
		__raw_writel(m, pbank->irqen + HW_STMP3xxx_CLR);
		__raw_writel(m, pbank->pin2irq + HW_STMP3xxx_CLR);
		goto out;
	}
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		l = 0; p = 1; break;
	case IRQ_TYPE_EDGE_FALLING:
		l = 0; p = 0; break;
	case IRQ_TYPE_LEVEL_HIGH:
		l = 1; p = 1; break;
	case IRQ_TYPE_LEVEL_LOW:
		l = 1; p = 0; break;
	default:
		pr_debug("%s: Incorrect GPIO interrupt type 0x%x\n",
				__func__, type);
		goto out;
	}

	__raw_writel(m,
		pbank->irqlevel + (l ? HW_STMP3xxx_SET : HW_STMP3xxx_CLR));
	__raw_writel(m,
		pbank->irqpolarity + (p ? HW_STMP3xxx_SET : HW_STMP3xxx_CLR));

	__raw_writel(m, pbank->irqstat + HW_STMP3xxx_CLR);

	__raw_writel(m, pbank->pin2irq + HW_STMP3xxx_SET);

	__raw_writel(m, pbank->irqen + HW_STMP3xxx_SET);
out:
	spin_unlock(&pbank->lock);
	return;
}
EXPORT_SYMBOL(stmp3xxx_configure_irq);

void stmp3xxx_pin_ack_irq(int irq)
{
	int b;
	u32 stat;

	for (b = 0; b < STMP3XXX_PINMUX_NR_BANKS; b++) {
		if (irq != pinmux_banks[b].irq)
			continue;
		stat = __raw_readl(pinmux_banks[b].irqstat);
		if (stat)
			__raw_writel(stat,
				pinmux_banks[b].irqstat + HW_STMP3xxx_CLR);
	}
}
EXPORT_SYMBOL(stmp3xxx_pin_ack_irq);

#ifdef CONFIG_PM
static int pinmux_suspend(struct sys_device *dev, pm_message_t state)
{
	return 0;
}

static int pinmux_resume(struct sys_device *dev)
{
	return 0;
}
#endif

static struct sysdev_class stmp3xxx_pinmux_sysclass = {
	.name		= "stmp3xxx-pinmux",
#ifdef CONFIG_PM
	.suspend        = pinmux_suspend,
	.resume         = pinmux_resume,
#endif
};

static struct sys_device stmp3xxx_pinmux_device = {
	.id = -1,
	.cls = &stmp3xxx_pinmux_sysclass,
};

static char *str_function(unsigned f)
{
	static char s[10];

	if (f != 3)
		sprintf(s, "FUNC_%d", f);
	else
		strcpy(s, "GPIO");
	return s;
}

static char *str_strength(unsigned f)
{
	static char *s[] = {
		"4mA", "8mA", "12mA", "16mA", "20mA"
	};
	return s[f];
}

static char *str_pullup(unsigned f)
{
	return f ? "Pullup" : "";
}

static char *str_voltage(unsigned f)
{
	return f ? "3.3V" : "1.8V";
}

static char *str_irqtype(int level, int pol)
{
	if (level)
		return pol ? "high lvl" : "low lvl";
	return pol ? "raising edge" : "falling edge";
}

static char *str_irq(int v_icfg, int v_ien, int v_ilevel,
			int v_ipol, int v_istat)
{
	static char irqcfg[30];

	if (v_icfg <= 0)
		return "";
	sprintf(irqcfg, "IRQ%c %s %s",
		v_istat ? '!' : ' ',
		str_irqtype(v_ilevel, v_ipol),
		v_ien ? "Enabled" : "Disabled");
	return irqcfg;
}

static ssize_t pinmux_all_list(struct sys_device *dev, 
			       struct sysdev_attribute *attr, char *buf)
{
	int b, pin, n = 0;
	struct stmp3xxx_pinmux_bank *p;
	u32 hwpull, hwmux, hwdrive, hwdrive_v, pm;
	int shift1, shift2;
	unsigned v_strength, v_pull, v_fun, v_voltage;
	int v_ien, v_icfg, v_ilevel, v_ipol, v_istat;

	for (b = 0; b < STMP3XXX_PINMUX_NR_BANKS; b++) {

		p = pinmux_banks + b;

		for (pin = 0, pm = 1; pin < 32; pin++, pm <<= 1) {
			if ((p->pin_map & pm) == 0)
				continue;

			hwmux = p->hw_muxsel[pin / HW_MUXSEL_PIN_NUM];
			shift1 = (pin % HW_MUXSEL_PIN_NUM) * HW_MUXSEL_PIN_LEN;
			hwpull = p->hw_pull;
			hwdrive = p->hw_drive[pin / HW_DRIVE_PIN_NUM];
			shift2 = (pin % HW_DRIVE_PIN_NUM) * HW_DRIVE_PIN_LEN;

			hwdrive_v = __raw_readl(hwdrive) >> shift2;
			v_strength = hwdrive_v & HW_DRIVE_PINDRV_MASK;
			v_voltage = hwdrive_v & HW_DRIVE_PINV_MASK;
			v_pull = __raw_readl(hwpull) & pm;
			v_fun = (__raw_readl(hwmux) >> shift1) &
				HW_MUXSEL_PINFUN_MASK;

			if (p->irqen == 0) {
				v_icfg = -1;
				v_ien = -1;
				v_ilevel = -1;
				v_ipol = -1;
				v_istat = -1;
			} else {
				v_icfg = (__raw_readl(p->pin2irq) & pm)
					>> pin;
				v_ien = (__raw_readl(p->irqen) & pm)
					>> pin;
				v_ilevel = (__raw_readl(p->irqlevel) & pm)
					>> pin;
				v_ipol = (__raw_readl(p->irqpolarity) & pm)
					>> pin;
				v_istat = (__raw_readl(p->irqstat) & pm)
					>> pin;
			}
			n += sprintf(buf + n,
			  "%d:%d\t%-15.15s\t%s\t%s\t%s\t%s\t%s\n",
			  b, pin, p->pin_labels[pin],
			  str_function(v_fun),
			  str_strength(v_strength),
			  str_voltage(v_voltage),
			  str_pullup(v_pull),
			  str_irq(v_icfg, v_ien, v_ilevel, v_ipol, v_istat));
		}
	}
	return n;
}
SYSDEV_ATTR(all, 0444, pinmux_all_list, NULL);

static int __init stmp3xxx_pinmux_init(void)
{
	int b;

	for (b = 0; b < STMP3XXX_PINMUX_NR_BANKS; b++)
		spin_lock_init(&pinmux_banks[b].lock);
	sysdev_class_register(&stmp3xxx_pinmux_sysclass);
	sysdev_register(&stmp3xxx_pinmux_device);
	sysdev_create_file(&stmp3xxx_pinmux_device, &attr_all);

	return 0;
}

arch_initcall(stmp3xxx_pinmux_init);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_LICENSE("GPL");
