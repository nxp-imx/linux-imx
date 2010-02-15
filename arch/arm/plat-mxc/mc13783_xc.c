/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/usb/fsl_xcvr.h>
#include <mach/pmic_external.h>
#include <mach/pmic_convity.h>
#include <mach/arc_otg.h>

/* Events to be passed to the thread */
#define MC13783_USB_VBUS_ON		0x0001
#define MC13783_USB_VBUS_OFF		0x0002
#define MC13783_USB_DETECT_MINI_A	0x0004
#define MC13783_USB_DETECT_MINI_B	0x0008

extern void otg_set_serial_peripheral(void);
extern void otg_set_serial_host(void);

static unsigned int p_event;
static PMIC_CONVITY_EVENTS g_event;
static PMIC_CONVITY_HANDLE pmic_handle = (PMIC_CONVITY_HANDLE) NULL;

static void xc_workqueue_handler(struct work_struct *work);

DECLARE_WORK(xc_work, xc_workqueue_handler);

DECLARE_MUTEX(pmic_mx);

static void pmic_event_handler(const PMIC_CONVITY_EVENTS event)
{
	if (event & USB_DETECT_4V4_RISE)
		pr_debug("%s: USB_DETECT_4V4_RISE\n", __func__);

	if (event & USB_DETECT_4V4_FALL)
		pr_debug("%s: USB_DETECT_4V4_FALL\n", __func__);

	if (event & USB_DETECT_2V0_RISE)
		pr_debug("%s: USB_DETECT_2V0_RISE\n", __func__);

	if (event & USB_DETECT_2V0_FALL)
		pr_debug("%s: USB_DETECT_2V0_FALL\n", __func__);

	if (event & USB_DETECT_0V8_RISE)
		pr_debug("%s: USB_DETECT_0V8_RISE\n", __func__);

	if (event & USB_DETECT_0V8_FALL)
		pr_debug("%s: USB_DETECT_0V8_FALL\n", __func__);

	if (event & USB_DETECT_MINI_B) {
		pr_debug("%s: USB_DETECT_MINI_B\n", __func__);
		otg_set_serial_peripheral();
		g_event = USB_DETECT_MINI_B;
		p_event = MC13783_USB_DETECT_MINI_B;
		schedule_work(&xc_work);
	}
	if (event & USB_DETECT_MINI_A) {
		pr_debug("%s: USB_DETECT_MINI_A\n", __func__);
		otg_set_serial_host();
		g_event = USB_DETECT_MINI_A;
		p_event = MC13783_USB_DETECT_MINI_A;
		schedule_work(&xc_work);
	}

	/*
	 * Mini-B cable insertion/removal does not generate cable-detect
	 * event, so we rely on the VBUS changes to identify a mini-b cable
	 * connect. This logic is only used if mini-b is the first cable that
	 * is connected after bootup. At all other times, removal of mini-a
	 * cable is used to initialize peripheral.
	 */
	if (g_event != USB_DETECT_MINI_A && g_event != USB_DETECT_MINI_B) {
		if ((event & USB_DETECT_0V8_RISE) &&
		    (event & USB_DETECT_2V0_RISE) &&
		    (event & USB_DETECT_4V4_RISE)) {
			otg_set_serial_peripheral();
			g_event = USB_DETECT_MINI_B;
			p_event = MC13783_USB_DETECT_MINI_B;
			schedule_work(&xc_work);
		}
	}
}

static int usb_pmic_mod_init(void)
{
	PMIC_STATUS rs = PMIC_ERROR;

	init_MUTEX_LOCKED(&pmic_mx);

	rs = pmic_convity_open(&pmic_handle, USB);
	if (rs != PMIC_SUCCESS) {
		printk(KERN_ERR "pmic_convity_open returned error %d\n", rs);
		return rs;
	}

	rs = pmic_convity_set_callback(pmic_handle, pmic_event_handler,
				       USB_DETECT_4V4_RISE | USB_DETECT_4V4_FALL
				       | USB_DETECT_2V0_RISE |
				       USB_DETECT_2V0_FALL | USB_DETECT_0V8_RISE
				       | USB_DETECT_0V8_FALL | USB_DETECT_MINI_A
				       | USB_DETECT_MINI_B);

	if (rs != PMIC_SUCCESS) {
		printk(KERN_ERR
		       "pmic_convity_set_callback returned error %d\n", rs);
		return rs;
	}

	return rs;
}

static void usb_pmic_mod_exit(void)
{
	PMIC_STATUS rs;

	pmic_convity_set_mode(pmic_handle, RS232_1);
	pmic_convity_clear_callback(pmic_handle);

	if (pmic_handle != (PMIC_CONVITY_HANDLE) NULL) {
		rs = pmic_convity_close(pmic_handle);
		if (rs != PMIC_SUCCESS) {
			printk(KERN_ERR
			       "pmic_convity_close() returned error %d", rs);
		} else {
			pmic_handle = (PMIC_CONVITY_HANDLE) NULL;
		}
	}
}

static inline void mc13783_set_host(void)
{
	PMIC_STATUS rs = PMIC_ERROR;

	rs = pmic_convity_usb_otg_clear_config(pmic_handle, USB_OTG_SE0CONN);
	rs |= pmic_convity_usb_otg_clear_config(pmic_handle, USB_PU);

	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_UDM_PD);
	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_UDP_PD);

	if (rs != PMIC_SUCCESS)
		printk(KERN_ERR "mc13783_set_host failed\n");

}

static inline void mc13783_set_peripheral(void)
{
	PMIC_STATUS rs = PMIC_ERROR;

	rs = pmic_convity_usb_otg_clear_config(pmic_handle, USB_UDM_PD);
	rs |= pmic_convity_usb_otg_clear_config(pmic_handle, USB_UDP_PD);

	rs |= pmic_convity_usb_set_speed(pmic_handle, USB_FULL_SPEED);
	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_OTG_SE0CONN);
	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_PU);

	if (rs != PMIC_SUCCESS)
		printk(KERN_ERR "mc13783_set_peripheral failed\n");
}

void mc13783_set_vbus_power(struct fsl_xcvr_ops *this,
			    struct fsl_usb2_platform_data *pdata, int on)
{
	if (on) {
		p_event = MC13783_USB_VBUS_ON;
		schedule_work(&xc_work);
	}
}

static struct fsl_xcvr_ops mc13783_ops_otg = {
	.name = "mc13783",
	.xcvr_type = PORTSC_PTS_SERIAL,
	.set_host = mc13783_set_host,
	.set_device = mc13783_set_peripheral,
	.set_vbus_power = mc13783_set_vbus_power,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);

static void xc_workqueue_handler(struct work_struct *work)
{
	PMIC_STATUS rs = PMIC_ERROR;

	down(&pmic_mx);

	switch (p_event) {
	case MC13783_USB_VBUS_OFF:
		mc13783_set_peripheral();
		break;
	case MC13783_USB_VBUS_ON:
		mc13783_set_host();
		break;
	case MC13783_USB_DETECT_MINI_B:
		rs = pmic_convity_set_output(pmic_handle, true, false);
		rs |=
		    pmic_convity_usb_otg_clear_config(pmic_handle,
						      USB_VBUS_CURRENT_LIMIT_LOW_30MS);

		if (rs != PMIC_SUCCESS)
			printk(KERN_ERR "MC13783_USB_VBUS_OFF failed\n");
		break;
	case MC13783_USB_DETECT_MINI_A:
		rs = pmic_convity_set_output(pmic_handle, true, true);
		rs |=
		    pmic_convity_usb_otg_set_config(pmic_handle,
						    USB_VBUS_CURRENT_LIMIT_LOW_30MS);

		if (rs != PMIC_SUCCESS)
			printk(KERN_ERR "MC13783_USB_VBUS_ON failed\n");
		break;
	default:
		break;
	}
	up(&pmic_mx);
}

int mc13783xc_init(void)
{
	PMIC_STATUS rs = PMIC_ERROR;

#if defined(CONFIG_MXC_USB_SB3)
	int xc_mode = USB_SINGLE_ENDED_BIDIR;
#elif defined(CONFIG_MXC_USB_SU6)
	int xc_mode = USB_SINGLE_ENDED_UNIDIR;
#elif defined(CONFIG_MXC_USB_DB4)
	int xc_mode = USB_DIFFERENTIAL_BIDIR;
#else
	int xc_mode = USB_DIFFERENTIAL_UNIDIR;
#endif

	rs = usb_pmic_mod_init();
	if (rs != PMIC_SUCCESS) {
		usb_pmic_mod_exit();
		printk(KERN_ERR "usb_pmic_mod_init failed\n");
		return rs;
	}

	rs = pmic_convity_usb_set_xcvr(pmic_handle, xc_mode);
	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_OTG_SE0CONN);
	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USBXCVREN);
	rs |= pmic_convity_set_output(pmic_handle, false, true);

	rs |= pmic_convity_usb_otg_set_config(pmic_handle, USB_PULL_OVERRIDE);
	rs |= pmic_convity_usb_otg_clear_config(pmic_handle, USB_USBCNTRL);
	rs |= pmic_convity_usb_otg_clear_config(pmic_handle, USB_DP150K_PU);

	if (rs != PMIC_SUCCESS)
		printk(KERN_ERR "pmic configuration failed\n");

	fsl_usb_xcvr_register(&mc13783_ops_otg);

	mc13783_set_peripheral();

	return rs;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

void mc13783xc_uninit(void)
{
	/* Clear stuff from init */
	pmic_convity_usb_otg_clear_config(pmic_handle, USB_OTG_SE0CONN);
	pmic_convity_usb_otg_clear_config(pmic_handle, USBXCVREN);
	pmic_convity_set_output(pmic_handle, false, false);
	pmic_convity_usb_otg_clear_config(pmic_handle, USB_PULL_OVERRIDE);

	/* Clear host mode */
	pmic_convity_usb_otg_clear_config(pmic_handle, USB_UDP_PD);
	pmic_convity_usb_otg_clear_config(pmic_handle, USB_UDM_PD);

	/* Clear peripheral mode */
	pmic_convity_usb_otg_clear_config(pmic_handle, USB_PU);

	/* Vbus off */
	pmic_convity_set_output(pmic_handle, true, false);
	pmic_convity_usb_otg_clear_config(pmic_handle,
					  USB_VBUS_CURRENT_LIMIT_LOW_30MS);

	usb_pmic_mod_exit();

	fsl_usb_xcvr_unregister(&mc13783_ops_otg);
}

subsys_initcall(mc13783xc_init);
module_exit(mc13783xc_uninit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("mc13783xc");
MODULE_LICENSE("GPL");
