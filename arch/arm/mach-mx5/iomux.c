/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup GPIO_MX5 Board GPIO and Muxing Setup
 * @ingroup MSL_MX5
 */
/*!
 * @file mach-mx5/iomux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX5
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include "iomux.h"
#include "mx51_pins.h"

#define MUX_I_START_MX53	0x0020
#define PAD_I_START_MX53	0x348
#define INPUT_CTL_START_MX53	0x730
#define MUX_I_END_MX53		(PAD_I_START_MX53 - 4)

#define PAD_I_START_MX50	0x2CC
#define INPUT_CTL_START_MX50	0x6C4

/*!
 * IOMUX register (base) addressesf
 */
#define IOMUXGPR0		(IO_ADDRESS(IOMUXC_BASE_ADDR))
#define IOMUXGPR1		(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x004)
#define IOMUXSW_MUX_CTL	(IO_ADDRESS(IOMUXC_BASE_ADDR))
#define IOMUXSW_INPUT_CTL	(IO_ADDRESS(IOMUXC_BASE_ADDR))

static u8 iomux_pin_res_table[(0x3F0 / 4) + 1];
static DEFINE_SPINLOCK(gpio_mux_lock);

static inline void *_get_sw_pad(void)
{
	if (cpu_is_mx51())
		return IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_START_MX51;
	else if (cpu_is_mx53())
		return IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_START_MX53;
	else
		return IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_START_MX50;
}

static inline void *_get_mux_reg(iomux_pin_name_t pin)
{
	u32 mux_reg = PIN_TO_IOMUX_MUX(pin);

	return IOMUXSW_MUX_CTL + mux_reg;
}

static inline void *_get_pad_reg(iomux_pin_name_t pin)
{
	u32 pad_reg = PIN_TO_IOMUX_PAD(pin);
	void __iomem *sw_pad_reg = _get_sw_pad();

	return sw_pad_reg + pad_reg;
}

static inline void *_get_mux_end(void)
{
	if (cpu_is_mx50())
		return IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x2C8;

	return IO_ADDRESS(IOMUXC_BASE_ADDR) + (0x3F0 - 4);
}

/*!
 * This function is used to configure a pin through the IOMUX module.
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
static int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 ret = 0;
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	void __iomem *mux_reg = _get_mux_reg(pin);
	u32 mux_data = 0;
	u8 *rp;

	BUG_ON((mux_reg > _get_mux_end()) || (mux_reg < IOMUXSW_MUX_CTL));
	spin_lock(&gpio_mux_lock);

	if (config == IOMUX_CONFIG_GPIO)
		mux_data = PIN_TO_ALT_GPIO(pin);
	else
		mux_data = config;

	__raw_writel(mux_data, mux_reg);

	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = iomux_pin_res_table + pin_index;
	if ((mux_data & *rp) && (*rp != mux_data)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
		       " config changed, reg=%p, "
		       " prev=0x%x new=0x%x\n", mux_reg, *rp, mux_data);
		ret = -EINVAL;
	}
	*rp = mux_data;
	spin_unlock(&gpio_mux_lock);
	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	int ret = iomux_config_mux(pin, config);
	int gpio = IOMUX_TO_GPIO(pin);

	if (!ret && (gpio < MXC_GPIO_IRQS) && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin))))
		ret |= gpio_request(gpio, NULL);

	return ret;
}
EXPORT_SYMBOL(mxc_request_iomux);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u8 *rp = iomux_pin_res_table + pin_index;
	int gpio = IOMUX_TO_GPIO(pin);

	*rp = 0;
	if ((gpio < MXC_GPIO_IRQS)
	    && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin))))
		gpio_free(gpio);

}
EXPORT_SYMBOL(mxc_free_iomux);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	void __iomem *pad_reg = _get_pad_reg(pin);
	void __iomem *sw_pad_reg = _get_sw_pad();

	BUG_ON(pad_reg < sw_pad_reg);
	__raw_writel(config, pad_reg);
}
EXPORT_SYMBOL(mxc_iomux_set_pad);

unsigned int mxc_iomux_get_pad(iomux_pin_name_t pin)
{
	void __iomem *pad_reg = _get_pad_reg(pin);

	return __raw_readl(pad_reg);
}
EXPORT_SYMBOL(mxc_iomux_get_pad);

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 *      */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config)
{
	void __iomem *reg;

	if (cpu_is_mx51()) {
		reg = IOMUXSW_INPUT_CTL + (input << 2) + INPUT_CTL_START_MX51;
	} else if (cpu_is_mx53()) {
		reg = IOMUXSW_INPUT_CTL + (input << 2) + INPUT_CTL_START_MX53;
	} else
		reg = IOMUXSW_INPUT_CTL + (input << 2) + INPUT_CTL_START_MX50;

	BUG_ON(input >= MUX_INPUT_NUM_MUX);
	__raw_writel(config, reg);
}
EXPORT_SYMBOL(mxc_iomux_set_input);
