/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup GPIO_MX31 Board GPIO and Muxing Setup
 * @ingroup MSL_MX31
 */
/*!
 * @file mach-mx3/iomux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX31
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "iomux.h"

/*!
 * 4 control fields per MUX register
 */
#define MUX_CTL_FIELDS          4

/*!
 * 3 control fields per PAD register
 */
#define PAD_CTL_FIELDS          3

/*!
 * Maximum number of MUX pins
 * Number of pins = (highest iomux reg - lowest iomux reg + 1) * (4 pins/reg)
 */
#define MUX_PIN_NUM_MAX \
        (((u32 *)IOMUXSW_MUX_END - (u32 *)IOMUXSW_MUX_CTL + 1) * MUX_CTL_FIELDS)

/*!
 * Number of pad controls =
 *               (highest pad ctl reg - lowest pad ctl reg + 1) * (3 pins/reg)
 */
#define PAD_CTL_NUM_MAX \
        (((u32 *)IOMUXSW_PAD_END - (u32 *)IOMUXSW_PAD_CTL + 1) * PAD_CTL_FIELDS)

#define PIN_TO_IOMUX_INDEX(pin) ((pin >> MUX_I) & ((1 << (MUX_F - MUX_I)) - 1))
#define PIN_TO_IOMUX_FIELD(pin) ((pin >> MUX_F) & ((1 << (PAD_I - MUX_F)) - 1))

/*!
 * 8 bits for each MUX control field
 */
#define MUX_CTL_BIT_LEN         8

/*!
 * 10 bits for each PAD control field
 */
#define MUX_PAD_BIT_LEN         10

/*!
 * IOMUX register (base) addresses
 */
#define IOMUXGPR	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x008)	/*!< General purpose */
#define IOMUXSW_MUX_CTL	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x00C)	/*!< MUX control */
#define IOMUXSW_MUX_END	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x150)	/*!< last MUX control register */
#define IOMUXSW_PAD_CTL	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x154)	/*!< Pad control */
#define IOMUXSW_PAD_END	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x308)	/*!< last Pad control register */
#define IOMUXINT_OBS1	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x000)	/*!< Observe interrupts 1 */
#define IOMUXINT_OBS2	(IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x004)	/*!< Observe interrupts 2 */

/* len - mask bit length; fld - mask bit field. Example, to have the mask:
 * 0xFF000000, use GET_FIELD_MASK(8, 3). Translate in plain language:
 * "set the 3rd (0-based) 8-bit-long field to all 1's */
#define GET_FIELD_MASK(len, fld)    (((1 << len) - 1) << (len * fld))
static DEFINE_SPINLOCK(gpio_mux_lock);
static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];

/*!
 * This function is used to configure a pin through the IOMUX module.
 * FIXED ME: for backward compatible. Will be static function!
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		     iomux_pin_icfg_t in)
{
	void __iomem *reg;
	u32 l, ret = 0;
	u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
	u32 mux_field = PIN_TO_IOMUX_FIELD(pin);
	u32 mux_mask = GET_FIELD_MASK(MUX_CTL_BIT_LEN, mux_field);
	u8 *rp;

	BUG_ON((mux_index > (MUX_PIN_NUM_MAX / MUX_CTL_FIELDS - 1)) ||
	       (mux_field >= MUX_CTL_FIELDS));

	reg = IOMUXSW_MUX_CTL + (mux_index * 4);
	spin_lock(&gpio_mux_lock);
	l = __raw_readl(reg);
	l = (l & (~mux_mask)) |
	    (((out << 4) | in) << (mux_field * MUX_CTL_BIT_LEN));
	__raw_writel(l, reg);
	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = iomux_pin_res_table + mux_index * MUX_CTL_FIELDS + mux_field;
	if (out & *rp && *rp != ((out << 4) | in)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		if (pin != MX31_PIN_CTS1 &&
		    pin != MX31_PIN_RTS1 &&
		    pin != MX31_PIN_DCD_DCE1 &&
		    pin != MX31_PIN_DSR_DTE1 &&
		    pin != MX31_PIN_DTR_DTE1 &&
		    pin != MX31_PIN_RI_DCE1 &&
		    pin != MX31_PIN_DSR_DCE1 &&
		    pin != MX31_PIN_DTR_DCE1 &&
		    pin != MX31_PIN_RXD1 && pin != MX31_PIN_TXD1) {
			printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
			       " config changed, index=%d field=%d, "
			       " prev=0x%x new=0x%x\n", mux_index, mux_field,
			       *rp, (out << 4) | in);
		}
		ret = -EINVAL;
	}
	*rp = (out << 4) | in;
	spin_unlock(&gpio_mux_lock);

	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		      iomux_pin_icfg_t in)
{
	int ret = iomux_config_mux(pin, out, in);
	if (out == OUTPUTCONFIG_GPIO && in == INPUTCONFIG_GPIO) {
		ret |= gpio_request(IOMUX_TO_GPIO(pin), NULL);
	}
	return ret;
}

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		    iomux_pin_icfg_t in)
{
	u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
	u32 mux_field = PIN_TO_IOMUX_FIELD(pin);
	u8 *rp = iomux_pin_res_table + mux_index * MUX_CTL_FIELDS + mux_field;

	BUG_ON((mux_index > (MUX_PIN_NUM_MAX / MUX_CTL_FIELDS - 1)) ||
	       (mux_field >= MUX_CTL_FIELDS));

	*rp = 0;
	if (out == OUTPUTCONFIG_GPIO && in == INPUTCONFIG_GPIO) {
		gpio_free(IOMUX_TO_GPIO(pin));
	}
}

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	void __iomem *reg;
	u32 l;
	u32 pad_index = (pin >> PAD_I) & ((1 << (PAD_F - PAD_I)) - 1);
	u32 pad_field = (pin >> PAD_F) & ((1 << (MUX_IO_I - PAD_F)) - 1);
	u32 pad_mask = GET_FIELD_MASK(MUX_PAD_BIT_LEN, pad_field);

	BUG_ON((pad_index > (PAD_CTL_NUM_MAX / PAD_CTL_FIELDS - 1)) ||
	       (pad_field >= PAD_CTL_FIELDS));

	reg = IOMUXSW_PAD_CTL + (pad_index * 4);
	spin_lock(&gpio_mux_lock);
	l = __raw_readl(reg);
	l = (l & (~pad_mask)) | (config << (pad_field * MUX_PAD_BIT_LEN));
	__raw_writel(l, reg);
	spin_unlock(&gpio_mux_lock);
}

/*
 * FIXED ME: for backward compatible. to be removed!
 */
void iomux_config_pad(iomux_pin_name_t pin, u32 config)
{
	mxc_iomux_set_pad(pin, config);
}

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en)
{
	u32 l;

	spin_lock(&gpio_mux_lock);
	l = __raw_readl(IOMUXGPR);
	if (en) {
		l |= gp;
	} else {
		l &= ~gp;
	}
	__raw_writel(l, IOMUXGPR);
	spin_unlock(&gpio_mux_lock);
}

/*!
 * FIXED ME: for backward compatible. to be removed!
 */
void iomux_config_gpr(iomux_gp_func_t gp, bool en)
{
	mxc_iomux_set_gpr(gp, en);
}

EXPORT_SYMBOL(mxc_request_iomux);
EXPORT_SYMBOL(mxc_free_iomux);
EXPORT_SYMBOL(mxc_iomux_set_pad);
EXPORT_SYMBOL(mxc_iomux_set_gpr);
EXPORT_SYMBOL(iomux_config_pad);
EXPORT_SYMBOL(iomux_config_gpr);
EXPORT_SYMBOL(iomux_config_mux);
