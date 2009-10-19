/*
 * Copyright 2005-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


extern int usbotg_init(struct platform_device *pdev);
extern void usbotg_uninit(struct fsl_usb2_platform_data *pdata);
extern int gpio_usbotg_hs_active(void);
extern void gpio_usbotg_hs_inactive(void);
extern struct platform_device *host_pdev_register(struct resource *res,
		  int n_res, struct fsl_usb2_platform_data *config);

extern int gpio_usbotg_utmi_active(void);
extern void gpio_usbotg_utmi_inactive(void);
static int usbotg_init_ext(struct platform_device *pdev);
static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata);

/*
 * Determine which platform_data struct to use for the DR controller,
 * based on which transceiver is configured.
 * PDATA is a pointer to it.
 */
#if defined(CONFIG_ISP1301_MXC)
static struct fsl_usb2_platform_data __maybe_unused dr_1301_config;
#define PDATA (&dr_1301_config)
#elif defined(CONFIG_MC13783_MXC)
static struct fsl_usb2_platform_data __maybe_unused dr_13783_config;
#define PDATA (&dr_13783_config)
#elif defined(CONFIG_UTMI_MXC)
static struct fsl_usb2_platform_data __maybe_unused dr_utmi_config;
#define PDATA (&dr_utmi_config)
#endif


/*
 * Used to set pdata->operating_mode before registering the platform_device.
 * If OTG is configured, the controller operates in OTG mode,
 * otherwise it's either host or device.
 */
#ifdef CONFIG_USB_OTG
#define DR_UDC_MODE	FSL_USB2_DR_OTG
#define DR_HOST_MODE	FSL_USB2_DR_OTG
#else
#define DR_UDC_MODE	FSL_USB2_DR_DEVICE
#define DR_HOST_MODE	FSL_USB2_DR_HOST
#endif


#ifdef CONFIG_USB_EHCI_ARC_OTG
static inline void dr_register_host(struct resource *r, int rs)
{
	PDATA->operating_mode = DR_HOST_MODE;
	host_pdev_register(r, rs, PDATA);
}
#else
static inline void dr_register_host(struct resource *r, int rs)
{
}
#endif

#ifdef CONFIG_USB_GADGET_ARC
static struct platform_device dr_udc_device;

static inline void dr_register_udc(void)
{
	PDATA->operating_mode = DR_UDC_MODE;
	dr_udc_device.dev.platform_data = PDATA;

	if (platform_device_register(&dr_udc_device))
		printk(KERN_ERR "usb: can't register DR gadget\n");
	else
		printk(KERN_INFO "usb: DR gadget (%s) registered\n",
		       PDATA->transceiver);
}
#else
static inline void dr_register_udc(void)
{
}
#endif

#ifdef CONFIG_USB_OTG
static struct platform_device dr_otg_device;

/*
 * set the proper operating_mode and
 * platform_data pointer, then register the
 * device.
 */
static inline void dr_register_otg(void)
{
	PDATA->operating_mode = FSL_USB2_DR_OTG;
	dr_otg_device.dev.platform_data = PDATA;

	if (platform_device_register(&dr_otg_device))
		printk(KERN_ERR "usb: can't register otg device\n");
	else
		printk(KERN_INFO "usb: DR OTG registered\n");
}
#else
static inline void dr_register_otg(void)
{
}
#endif
