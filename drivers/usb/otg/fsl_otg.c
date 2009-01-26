/*
 * Copyright (C) 2005-2008 Freescale semiconductor, Inc.
 *
 * Author: Li Yang <LeoLi@freescale.com>
 *         Jerry Huang <Chang-Ming.Huang@freescale.com>
 *
 * Initialization based on code from Shlomi Gridish.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/fsl_devices.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>

#include "fsl_otg.h"

#define CONFIG_USB_OTG_DEBUG_FILES
#define DRIVER_VERSION "$Revision: 1.55 $"
#define DRIVER_AUTHOR "Jerry Huang/Li Yang"
#define DRIVER_DESC "Freescale USB OTG Driver"
#define DRIVER_INFO DRIVER_VERSION " " DRIVER_DESC

MODULE_DESCRIPTION("Freescale USB OTG Transceiver Driver");

static const char driver_name[] = "fsl-usb2-otg";

const pm_message_t otg_suspend_state = {
	.event = 1,
};

#define HA_DATA_PULSE 1

volatile static struct usb_dr_mmap *usb_dr_regs;
static struct fsl_otg *fsl_otg_dev;
static int srp_wait_done;

/* FSM timers */
struct fsl_otg_timer *a_wait_vrise_tmr, *a_wait_bcon_tmr, *a_aidl_bdis_tmr,
	*b_ase0_brst_tmr, *b_se0_srp_tmr;

/* Driver specific timers */
struct fsl_otg_timer *b_data_pulse_tmr, *b_vbus_pulse_tmr, *b_srp_fail_tmr,
	*b_srp_wait_tmr, *a_wait_enum_tmr;

static struct list_head active_timers;

static struct fsl_otg_config fsl_otg_initdata = {
	.otg_port = 1,
};

int write_ulpi(u8 addr, u8 data)
{
	u32 temp;
	temp = 0x60000000 | (addr << 16) | data;
	temp = cpu_to_le32(temp);
	usb_dr_regs->ulpiview = temp;
	return 0;
}

/* prototype declaration */
void fsl_otg_add_timer(void *timer);
void fsl_otg_del_timer(void *timer);

/* -------------------------------------------------------------*/
/* Operations that will be called from OTG Finite State Machine */

/* Charge vbus for vbus pulsing in SRP */
void fsl_otg_chrg_vbus(int on)
{
	if (on)
		usb_dr_regs->otgsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) &
				 ~OTGSC_INTSTS_MASK &
				 ~OTGSC_CTRL_VBUS_DISCHARGE) |
				OTGSC_CTRL_VBUS_CHARGE);
	else
		usb_dr_regs->otgsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) &
				 ~OTGSC_INTSTS_MASK & ~OTGSC_CTRL_VBUS_CHARGE));
}

/* Discharge vbus through a resistor to ground */
void fsl_otg_dischrg_vbus(int on)
{
	if (on)
		usb_dr_regs->otgsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) &
				 ~OTGSC_INTSTS_MASK)
				| OTGSC_CTRL_VBUS_DISCHARGE);
	else
		usb_dr_regs->otgsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) &
				 ~OTGSC_INTSTS_MASK &
				 ~OTGSC_CTRL_VBUS_DISCHARGE));
}

/* A-device driver vbus, controlled through PP bit in PORTSC */
void fsl_otg_drv_vbus(int on)
{
/*	if (on)
		usb_dr_regs->portsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->portsc) &
				 ~PORTSC_W1C_BITS) | PORTSC_PORT_POWER);
	else
		usb_dr_regs->portsc =
		    cpu_to_le32(le32_to_cpu(usb_dr_regs->portsc) &
				~PORTSC_W1C_BITS & ~PORTSC_PORT_POWER);
*/
}

/*
 * Pull-up D+, signalling connect by periperal. Also used in
 * data-line pulsing in SRP
 */
void fsl_otg_loc_conn(int on)
{
	if (on)
		usb_dr_regs->otgsc =
		    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) &
				 ~OTGSC_INTSTS_MASK) | OTGSC_CTRL_DATA_PULSING);
	else
		usb_dr_regs->otgsc =
		    cpu_to_le32(le32_to_cpu(usb_dr_regs->otgsc) &
				~OTGSC_INTSTS_MASK & ~OTGSC_CTRL_DATA_PULSING);
}

/* Generate SOF by host.  This is controlled through suspend/resume the
 * port.  In host mode, controller will automatically send SOF.
 * Suspend will block the data on the port.
 */
void fsl_otg_loc_sof(int on)
{
	u32 tmpval;

	tmpval = readl(&fsl_otg_dev->dr_mem_map->portsc) & ~PORTSC_W1C_BITS;
	if (on)
		tmpval |= PORTSC_PORT_FORCE_RESUME;
	else
		tmpval |= PORTSC_PORT_SUSPEND;
	writel(tmpval, &fsl_otg_dev->dr_mem_map->portsc);

}

/* Start SRP pulsing by data-line pulsing, followed with v-bus pulsing. */
void fsl_otg_start_pulse(void)
{
	srp_wait_done = 0;
#ifdef HA_DATA_PULSE
	usb_dr_regs->otgsc =
	    cpu_to_le32((le32_to_cpu(usb_dr_regs->otgsc) & ~OTGSC_INTSTS_MASK)
			| OTGSC_HA_DATA_PULSE);
#else
	fsl_otg_loc_conn(1);
#endif

	fsl_otg_add_timer(b_data_pulse_tmr);
}

void fsl_otg_pulse_vbus(void);

void b_data_pulse_end(unsigned long foo)
{
#ifdef HA_DATA_PULSE
#else
	fsl_otg_loc_conn(0);
#endif

	/* Do VBUS pulse after data pulse */
	fsl_otg_pulse_vbus();
}

void fsl_otg_pulse_vbus(void)
{
	srp_wait_done = 0;
	fsl_otg_chrg_vbus(1);
	/* start the timer to end vbus charge */
	fsl_otg_add_timer(b_vbus_pulse_tmr);
}

void b_vbus_pulse_end(unsigned long foo)
{
	fsl_otg_chrg_vbus(0);

	/* As USB3300 using the same a_sess_vld and b_sess_vld voltage
	 * we need to discharge the bus for a while to distinguish
	 * residual voltage of vbus pulsing and A device pull up */
	fsl_otg_dischrg_vbus(1);
	fsl_otg_add_timer(b_srp_wait_tmr);
}

void b_srp_end(unsigned long foo)
{
	fsl_otg_dischrg_vbus(0);
	srp_wait_done = 1;

	if ((fsl_otg_dev->otg.state == OTG_STATE_B_SRP_INIT) &&
	    fsl_otg_dev->fsm.b_sess_vld)
		fsl_otg_dev->fsm.b_srp_done = 1;
}

/* Workaround for a_host suspending too fast.  When a_bus_req=0,
 * a_host will start by SRP.  It needs to set b_hnp_enable before
 * actually suspending to start HNP
 */
void a_wait_enum(unsigned long foo)
{
	VDBG("a_wait_enum timeout\n");
	if (!fsl_otg_dev->otg.host->b_hnp_enable)
		fsl_otg_add_timer(a_wait_enum_tmr);
	else
		otg_statemachine(&fsl_otg_dev->fsm);
}

/* ------------------------------------------------------*/

/* The timeout callback function to set time out bit */
void set_tmout(unsigned long indicator)
{
	*(int *)indicator = 1;
}

/* Initialize timers */
int fsl_otg_init_timers(struct otg_fsm *fsm)
{
	/* FSM used timers */
	a_wait_vrise_tmr = otg_timer_initializer(&set_tmout, TA_WAIT_VRISE,
				(unsigned long)&fsm->a_wait_vrise_tmout);
	if (a_wait_vrise_tmr == NULL)
		return -ENOMEM;

	a_wait_bcon_tmr = otg_timer_initializer(&set_tmout, TA_WAIT_BCON,
				(unsigned long)&fsm->a_wait_bcon_tmout);
	if (a_wait_bcon_tmr == NULL)
		return -ENOMEM;

	a_aidl_bdis_tmr = otg_timer_initializer(&set_tmout, TA_AIDL_BDIS,
				(unsigned long)&fsm->a_aidl_bdis_tmout);
	if (a_aidl_bdis_tmr == NULL)
		return -ENOMEM;

	b_ase0_brst_tmr = otg_timer_initializer(&set_tmout, TB_ASE0_BRST,
				(unsigned long)&fsm->b_ase0_brst_tmout);
	if (b_ase0_brst_tmr == NULL)
		return -ENOMEM;

	b_se0_srp_tmr = otg_timer_initializer(&set_tmout, TB_SE0_SRP,
				(unsigned long)&fsm->b_se0_srp);
	if (b_se0_srp_tmr == NULL)
		return -ENOMEM;

	b_srp_fail_tmr = otg_timer_initializer(&set_tmout, TB_SRP_FAIL,
				(unsigned long)&fsm->b_srp_done);
	if (b_srp_fail_tmr == NULL)
		return -ENOMEM;

	a_wait_enum_tmr = otg_timer_initializer(&a_wait_enum, 10,
				(unsigned long)&fsm);
	if (a_wait_enum_tmr == NULL)
		return -ENOMEM;

	/* device driver used timers */
	b_srp_wait_tmr = otg_timer_initializer(&b_srp_end, TB_SRP_WAIT, 0);
	if (b_srp_wait_tmr == NULL)
		return -ENOMEM;

	b_data_pulse_tmr = otg_timer_initializer(&b_data_pulse_end,
				TB_DATA_PLS, 0);
	if (b_data_pulse_tmr == NULL)
		return -ENOMEM;

	b_vbus_pulse_tmr = otg_timer_initializer(&b_vbus_pulse_end,
				TB_VBUS_PLS, 0);
	if (b_vbus_pulse_tmr == NULL)
		return -ENOMEM;

	return 0;
}

/* Uninitialize timers */
void fsl_otg_uninit_timers(void)
{
	/* FSM used timers */
	if (a_wait_vrise_tmr != NULL)
		kfree(a_wait_vrise_tmr);
	if (a_wait_bcon_tmr != NULL)
		kfree(a_wait_bcon_tmr);
	if (a_aidl_bdis_tmr != NULL)
		kfree(a_aidl_bdis_tmr);
	if (b_ase0_brst_tmr != NULL)
		kfree(b_ase0_brst_tmr);
	if (b_se0_srp_tmr != NULL)
		kfree(b_se0_srp_tmr);
	if (b_srp_fail_tmr != NULL)
		kfree(b_srp_fail_tmr);
	if (a_wait_enum_tmr != NULL)
		kfree(a_wait_enum_tmr);

	/* device driver used timers */
	if (b_srp_wait_tmr != NULL)
		kfree(b_srp_wait_tmr);
	if (b_data_pulse_tmr != NULL)
		kfree(b_data_pulse_tmr);
	if (b_vbus_pulse_tmr != NULL)
		kfree(b_vbus_pulse_tmr);
}

/* Add timer to timer list */
void fsl_otg_add_timer(void *gtimer)
{
	struct fsl_otg_timer *timer = (struct fsl_otg_timer *)gtimer;
	struct fsl_otg_timer *tmp_timer;

	/* Check if the timer is already in the active list,
	 * if so update timer count
	 */
	list_for_each_entry(tmp_timer, &active_timers, list)
	    if (tmp_timer == timer) {
		timer->count = timer->expires;
		return;
	}
	timer->count = timer->expires;
	list_add_tail(&timer->list, &active_timers);
}

/* Remove timer from the timer list; clear timeout status */
void fsl_otg_del_timer(void *gtimer)
{
	struct fsl_otg_timer *timer = (struct fsl_otg_timer *)gtimer;
	struct fsl_otg_timer *tmp_timer, *del_tmp;

	list_for_each_entry_safe(tmp_timer, del_tmp, &active_timers, list)
		if (tmp_timer == timer)
			list_del(&timer->list);
}

/* Reduce timer count by 1, and find timeout conditions.
 * Called by fsl_otg 1ms timer interrupt
 */
int fsl_otg_tick_timer(void)
{
	struct fsl_otg_timer *tmp_timer, *del_tmp;
	int expired = 0;

	list_for_each_entry_safe(tmp_timer, del_tmp, &active_timers, list) {
		tmp_timer->count--;
		/* check if timer expires */
		if (!tmp_timer->count) {
			list_del(&tmp_timer->list);
			tmp_timer->function(tmp_timer->data);
			expired = 1;
		}
	}

	return expired;
}

/* Reset controller, not reset the bus */
void otg_reset_controller(void)
{
	u32 command;

	command = readl(&usb_dr_regs->usbcmd);
	command |= (1 << 1);
	writel(command, &usb_dr_regs->usbcmd);
	while (readl(&usb_dr_regs->usbcmd) & (1 << 1)) ;
}

/* Call suspend/resume routines in host driver */
int fsl_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct device *dev;
	struct fsl_otg *otg_dev = container_of(xceiv, struct fsl_otg, otg);
	u32 retval = 0;

	if (!xceiv->host)
		return -ENODEV;
	dev = xceiv->host->controller;

	/* Update a_vbus_vld state as a_vbus_vld int is disabled
	 * in device mode
	 */
	fsm->a_vbus_vld =
	    (le32_to_cpu(usb_dr_regs->otgsc) & OTGSC_STS_A_VBUS_VALID) ? 1 : 0;
	if (on) {
		/* start fsl usb host controller */
		if (otg_dev->host_working)
			goto end;
		else {
			otg_reset_controller();
			VDBG("host on......\n");
			if (dev->driver->resume) {
				retval = dev->driver->resume(dev);
				if (fsm->id) {
					/* default-b */
					fsl_otg_drv_vbus(1);
					/* Workaround: b_host can't driver
					 * vbus, but PP in PORTSC needs to
					 * be 1 for host to work.
					 * So we set drv_vbus bit in
					 * transceiver to 0 thru ULPI. */
#if defined(CONFIG_ISP1504_MXC)
					write_ulpi(0x0c, 0x20);
#endif
				}
			}

			otg_dev->host_working = 1;
		}
	} else {
		/* stop fsl usb host controller */
		if (!otg_dev->host_working)
			goto end;
		else {
			VDBG("host off......\n");
			if (dev && dev->driver) {
				retval = dev->driver->suspend(dev,
							otg_suspend_state);
				if (fsm->id)
					/* default-b */
					fsl_otg_drv_vbus(0);
			}
			otg_dev->host_working = 0;
		}
	}
end:
	return retval;
}

/* Call suspend and resume function in udc driver
 * to stop and start udc driver.
 */
int fsl_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct device *dev;

	if (!xceiv->gadget || !xceiv->gadget->dev.parent)
		return -ENODEV;

	VDBG("gadget %s \n", on ? "on" : "off");
	dev = xceiv->gadget->dev.parent;

	if (on)
		dev->driver->resume(dev);
	else
		dev->driver->suspend(dev, otg_suspend_state);

	return 0;
}

/* Called by initialization code of host driver.  Register host controller
 * to the OTG.  Suspend host for OTG role detection.
 */
static int fsl_otg_set_host(struct otg_transceiver *otg_p, struct usb_bus *host)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	otg_p->host = host;

	otg_dev->fsm.a_bus_drop = 0;
	otg_dev->fsm.a_bus_req = 1;

	if (host) {
		VDBG("host off......\n");

		otg_p->host->otg_port = fsl_otg_initdata.otg_port;
		otg_p->host->is_b_host = otg_dev->fsm.id;
		/* must leave time for khubd to finish its thing
		 * before yanking the host driver out from under it,
		 * so suspend the host after a short delay.
		 */
		otg_dev->host_working = 1;
		schedule_delayed_work(&otg_dev->otg_event, 100);
		return 0;
	} else {		/* host driver going away */

		if (!(le32_to_cpu(otg_dev->dr_mem_map->otgsc) &
		      OTGSC_STS_USB_ID)) {
			/* Mini-A cable connected */
			struct otg_fsm *fsm = &otg_dev->fsm;

			otg_p->state = OTG_STATE_UNDEFINED;
			fsm->protocol = PROTO_UNDEF;
		}
	}

	otg_dev->host_working = 0;

	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* Called by initialization code of udc.  Register udc to OTG.*/
static int fsl_otg_set_peripheral(struct otg_transceiver *otg_p,
				  struct usb_gadget *gadget)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	VDBG("otg_dev 0x%x\n", (int)otg_dev);
	VDBG("fsl_otg_dev 0x%x\n", (int)fsl_otg_dev);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	if (!gadget) {
		if (!otg_dev->otg.default_a)
			otg_p->gadget->ops->vbus_draw(otg_p->gadget, 0);
		usb_gadget_vbus_disconnect(otg_dev->otg.gadget);
		otg_dev->otg.gadget = 0;
		otg_dev->fsm.b_bus_req = 0;
		otg_statemachine(&otg_dev->fsm);
		return 0;
	}
#ifdef DEBUG
	/*
	 * debug the initial state of the ID pin when only
	 * the gadget driver is loaded and no cable is connected.
	 * sometimes, we get an ID irq right
	 * after the udc driver's otg_get_transceiver() call
	 * that indicates that IDpin=0, which means a Mini-A
	 * connector is attached.  not good.
	 */
	DBG("before: fsm.id ID pin=%d", otg_dev->fsm.id);
	otg_dev->fsm.id = (otg_dev->dr_mem_map->otgsc & OTGSC_STS_USB_ID) ?
	    1 : 0;
	DBG("after:  fsm.id ID pin=%d", otg_dev->fsm.id);
	/*if (!otg_dev->fsm.id) {
	   printk("OTG Control = 0x%x\n",
	   isp1504_read(ISP1504_OTGCTL,
	   &otg_dev->dr_mem_map->ulpiview));
	   } */
#endif

	otg_p->gadget = gadget;
	otg_p->gadget->is_a_peripheral = !otg_dev->fsm.id;

	otg_dev->fsm.b_bus_req = 1;

	/* start the gadget right away if the ID pin says Mini-B */
	DBG("ID pin=%d\n", otg_dev->fsm.id);
	if (otg_dev->fsm.id == 1) {
		fsl_otg_start_host(&otg_dev->fsm, 0);
		otg_drv_vbus(&otg_dev->fsm, 0);
		fsl_otg_start_gadget(&otg_dev->fsm, 1);
	}

	return 0;
}

/* Set OTG port power, only for B-device */
static int fsl_otg_set_power(struct otg_transceiver *otg_p, unsigned mA)
{
	if (!fsl_otg_dev)
		return -ENODEV;
	if (otg_p->state == OTG_STATE_B_PERIPHERAL)
		printk(KERN_INFO "FSL OTG:Draw %d mA\n", mA);

	return 0;
}

/* Delayed pin detect interrupt processing.
 *
 * When the Mini-A cable is disconnected from the board,
 * the pin-detect interrupt happens before the disconnnect
 * interrupts for the connected device(s).  In order to
 * process the disconnect interrupt(s) prior to switching
 * roles, the pin-detect interrupts are delayed, and handled
 * by this routine.
 */
static void fsl_otg_event(struct work_struct *work)
{
	struct fsl_otg *og = container_of(work, struct fsl_otg, otg_event.work);
	struct otg_fsm *fsm = &og->fsm;

	if (fsm->id) {		/* switch to gadget */
		fsl_otg_start_host(fsm, 0);
		otg_drv_vbus(fsm, 0);
		fsl_otg_start_gadget(fsm, 1);
	}
}

/* B-device start SRP */
static int fsl_otg_start_srp(struct otg_transceiver *otg_p)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev
	    || otg_p->state != OTG_STATE_B_IDLE)
		return -ENODEV;

	otg_dev->fsm.b_bus_req = 1;
	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* A_host suspend will call this function to start hnp */
static int fsl_otg_start_hnp(struct otg_transceiver *otg_p)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	/* printk("start_hnp.............\n"); */
	/* clear a_bus_req to enter a_suspend state */
	otg_dev->fsm.a_bus_req = 0;
	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* Interrupt handler.  OTG/host/peripheral share the same int line.
 * OTG driver clears OTGSC interrupts and leaves USB interrupts
 * intact.  It needs to have knowledge of some USB interrupts
 * such as port change.
 */
irqreturn_t fsl_otg_isr(int irq, void *dev_id)
{
	struct otg_fsm *fsm = &((struct fsl_otg *)dev_id)->fsm;
	struct otg_transceiver *otg = &((struct fsl_otg *)dev_id)->otg;
	u32 otg_int_src, otg_sc;

	otg_sc = le32_to_cpu(usb_dr_regs->otgsc);
	otg_int_src = otg_sc & OTGSC_INTSTS_MASK & (otg_sc >> 8);

	/* Only clear otg interrupts */
	usb_dr_regs->otgsc |= cpu_to_le32(otg_sc & OTGSC_INTSTS_MASK);

	/*FIXME: ID change not generate when init to 0 */
	fsm->id = (otg_sc & OTGSC_STS_USB_ID) ? 1 : 0;
	otg->default_a = (fsm->id == 0);

	/* process OTG interrupts */
	if (otg_int_src) {
		if (otg_int_src & OTGSC_INTSTS_USB_ID) {
			fsm->id = (otg_sc & OTGSC_STS_USB_ID) ? 1 : 0;
			otg->default_a = (fsm->id == 0);
			/* clear conn information */
			if (fsm->id)
				fsm->b_conn = 0;
			else
				fsm->a_conn = 0;

			if (otg->host)
				otg->host->is_b_host = fsm->id;
			if (otg->gadget)
				otg->gadget->is_a_peripheral = !fsm->id;
			VDBG("ID int (ID is %d)\n", fsm->id);

			if (fsm->id) {	/* switch to gadget */
				schedule_delayed_work(&((struct fsl_otg *)
							dev_id)->otg_event,
							100);
			} else {	/* switch to host */
				cancel_delayed_work(&
						    ((struct fsl_otg *)dev_id)->
						    otg_event);
				fsl_otg_start_gadget(fsm, 0);
				otg_drv_vbus(fsm, 1);
				fsl_otg_start_host(fsm, 1);
			}

			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static struct otg_fsm_ops fsl_otg_ops = {
	.chrg_vbus = fsl_otg_chrg_vbus,
	.drv_vbus = fsl_otg_drv_vbus,
	.loc_conn = fsl_otg_loc_conn,
	.loc_sof = fsl_otg_loc_sof,
	.start_pulse = fsl_otg_start_pulse,

	.add_timer = fsl_otg_add_timer,
	.del_timer = fsl_otg_del_timer,

	.start_host = fsl_otg_start_host,
	.start_gadget = fsl_otg_start_gadget,
};

/* Initialize the global variable fsl_otg_dev and request IRQ for OTG */
static int fsl_otg_conf(struct platform_device *pdev)
{
	int status;
	struct fsl_otg *fsl_otg_tc;
	struct fsl_usb2_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	DBG();

	if (fsl_otg_dev)
		return 0;

	/* allocate space to fsl otg device */
	fsl_otg_tc = kzalloc(sizeof(struct fsl_otg), GFP_KERNEL);
	if (!fsl_otg_tc)
		return -ENODEV;

	INIT_DELAYED_WORK(&fsl_otg_tc->otg_event, fsl_otg_event);

	INIT_LIST_HEAD(&active_timers);
	status = fsl_otg_init_timers(&fsl_otg_tc->fsm);
	if (status) {
		printk(KERN_INFO "Couldn't init OTG timers\n");
		fsl_otg_uninit_timers();
		kfree(fsl_otg_tc);
		return status;
	}
	spin_lock_init(&fsl_otg_tc->fsm.lock);

	/* Set OTG state machine operations */
	fsl_otg_tc->fsm.ops = &fsl_otg_ops;

	/* initialize the otg structure */
	fsl_otg_tc->otg.label = DRIVER_DESC;
	fsl_otg_tc->otg.set_host = fsl_otg_set_host;
	fsl_otg_tc->otg.set_peripheral = fsl_otg_set_peripheral;
	fsl_otg_tc->otg.set_power = fsl_otg_set_power;
	fsl_otg_tc->otg.start_hnp = fsl_otg_start_hnp;
	fsl_otg_tc->otg.start_srp = fsl_otg_start_srp;

	fsl_otg_dev = fsl_otg_tc;

	/* Store the otg transceiver */
	status = otg_set_transceiver(&fsl_otg_tc->otg);
	if (status) {
		printk(KERN_WARNING ": unable to register OTG transceiver.\n");
		return status;
	}

	return 0;
}

/* OTG Initialization*/
int usb_otg_start(struct platform_device *pdev)
{
	struct fsl_otg *p_otg;
	struct otg_transceiver *otg_trans = otg_get_transceiver();
	struct otg_fsm *fsm;
	volatile unsigned long *p;
	int status;
	struct resource *res;
	u32 temp;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	p_otg = container_of(otg_trans, struct fsl_otg, otg);
	fsm = &p_otg->fsm;

	/* Initialize the state machine structure with default values */
	SET_OTG_STATE(otg_trans, OTG_STATE_UNDEFINED);
	fsm->transceiver = &p_otg->otg;

	/* We don't require predefined MEM/IRQ resource index */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	/* We don't request_mem_region here to enable resource sharing
	 * with host/device */

	usb_dr_regs = ioremap(res->start, sizeof(struct usb_dr_mmap));
	p_otg->dr_mem_map = (struct usb_dr_mmap *)usb_dr_regs;
	pdata->regs = (void *)usb_dr_regs;

	/* request irq */
	p_otg->irq = platform_get_irq(pdev, 0);
	status = request_irq(p_otg->irq, fsl_otg_isr,
				IRQF_SHARED, driver_name, p_otg);
	if (status) {
		dev_dbg(p_otg->otg.dev, "can't get IRQ %d, error %d\n",
			p_otg->irq, status);
		iounmap(p_otg->dr_mem_map);
		kfree(p_otg);
		return status;
	}

	if (pdata->platform_init && pdata->platform_init(pdev) != 0)
		return -EINVAL;


	/* Export DR controller resources */
	otg_set_resources(pdev->resource);

	/* stop the controller */
	temp = readl(&p_otg->dr_mem_map->usbcmd);
	temp &= ~USB_CMD_RUN_STOP;
	writel(temp, &p_otg->dr_mem_map->usbcmd);

	/* reset the controller */
	temp = readl(&p_otg->dr_mem_map->usbcmd);
	temp |= USB_CMD_CTRL_RESET;
	writel(temp, &p_otg->dr_mem_map->usbcmd);

	/* wait reset completed */
	while (readl(&p_otg->dr_mem_map->usbcmd) & USB_CMD_CTRL_RESET) ;

	/* configure the VBUSHS as IDLE(both host and device) */
	temp = USB_MODE_STREAM_DISABLE | (pdata->es ? USB_MODE_ES : 0);
	writel(temp, &p_otg->dr_mem_map->usbmode);

	/* configure PHY interface */
	temp = readl(&p_otg->dr_mem_map->portsc);
	temp &= ~(PORTSC_PHY_TYPE_SEL | PORTSC_PTW);
	switch (pdata->phy_mode) {
	case FSL_USB2_PHY_ULPI:
		temp |= PORTSC_PTS_ULPI;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		temp |= PORTSC_PTW_16BIT;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		temp |= PORTSC_PTS_UTMI;
		/* fall through */
	default:
		break;
	}
	writel(temp, &p_otg->dr_mem_map->portsc);

	if (pdata->have_sysif_regs) {
		/* configure control enable IO output, big endian register */
		p = (volatile unsigned long *)(&p_otg->dr_mem_map->control);
		temp = *p;
		temp |= USB_CTRL_IOENB;
		*p = temp;
	}

	/* disable all interrupt and clear all OTGSC status */
	temp = readl(&p_otg->dr_mem_map->otgsc);
	temp &= ~OTGSC_INTERRUPT_ENABLE_BITS_MASK;
	temp |= OTGSC_INTERRUPT_STATUS_BITS_MASK | OTGSC_CTRL_VBUS_DISCHARGE;
	writel(temp, &p_otg->dr_mem_map->otgsc);


	/*
	 * The identification (id) input is FALSE when a Mini-A plug is inserted
	 * in the devices Mini-AB receptacle. Otherwise, this input is TRUE.
	 * Also: record initial state of ID pin
	 */
	if (le32_to_cpu(p_otg->dr_mem_map->otgsc) & OTGSC_STS_USB_ID) {
		p_otg->otg.state = OTG_STATE_UNDEFINED;
		p_otg->fsm.id = 1;
	} else {
		p_otg->otg.state = OTG_STATE_A_IDLE;
		p_otg->fsm.id = 0;
	}

	DBG("initial ID pin=%d\n", p_otg->fsm.id);

	/* enable OTG ID pin interrupt */
	temp = readl(&p_otg->dr_mem_map->otgsc);
	temp |= OTGSC_INTR_USB_ID_EN;
	temp &= ~(OTGSC_CTRL_VBUS_DISCHARGE | OTGSC_INTR_1MS_TIMER_EN);
	writel(temp, &p_otg->dr_mem_map->otgsc);

	return 0;
}

/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
#ifdef CONFIG_USB_OTG_DEBUG_FILES

#include <linux/seq_file.h>

static const char proc_filename[] = "driver/isp1504_otg";

static int otg_proc_read(char *page, char **start, off_t off, int count,
			 int *eof, void *_dev)
{
	struct otg_fsm *fsm = &fsl_otg_dev->fsm;
	char *buf = page;
	char *next = buf;
	unsigned size = count;
	unsigned long flags;
	int t;
	u32 tmp_reg;

	if (off != 0)
		return 0;

	spin_lock_irqsave(&fsm->lock, flags);

	/* ------basic driver infomation ---- */
	t = scnprintf(next, size,
		      DRIVER_DESC "\n" "fsl_usb2_otg version: %s\n\n",
		      DRIVER_VERSION);
	size -= t;
	next += t;

	/* ------ Registers ----- */
	tmp_reg = le32_to_cpu(usb_dr_regs->otgsc);
	t = scnprintf(next, size, "OTGSC reg: %08x\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = le32_to_cpu(usb_dr_regs->portsc);
	t = scnprintf(next, size, "PORTSC reg: %08x\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = le32_to_cpu(usb_dr_regs->usbmode);
	t = scnprintf(next, size, "USBMODE reg: %08x\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = le32_to_cpu(usb_dr_regs->usbcmd);
	t = scnprintf(next, size, "USBCMD reg: %08x\n", tmp_reg);
	size -= t;
	next += t;

	tmp_reg = le32_to_cpu(usb_dr_regs->usbsts);
	t = scnprintf(next, size, "USBSTS reg: %08x\n", tmp_reg);
	size -= t;
	next += t;

	/* ------ State ----- */
	t = scnprintf(next, size,
		      "OTG state: %s\n\n",
		      state_string(fsl_otg_dev->otg.state));
	size -= t;
	next += t;

#if 1 || defined DEBUG
	/* ------ State Machine Variables ----- */
	t = scnprintf(next, size, "a_bus_req: %d\n", fsm->a_bus_req);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_bus_req: %d\n", fsm->b_bus_req);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_bus_resume: %d\n", fsm->a_bus_resume);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_bus_suspend: %d\n", fsm->a_bus_suspend);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_conn: %d\n", fsm->a_conn);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_sess_vld: %d\n", fsm->a_sess_vld);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_srp_det: %d\n", fsm->a_srp_det);
	size -= t;
	next += t;

	t = scnprintf(next, size, "a_vbus_vld: %d\n", fsm->a_vbus_vld);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_bus_resume: %d\n", fsm->b_bus_resume);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_bus_suspend: %d\n", fsm->b_bus_suspend);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_conn: %d\n", fsm->b_conn);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_se0_srp: %d\n", fsm->b_se0_srp);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_sess_end: %d\n", fsm->b_sess_end);
	size -= t;
	next += t;

	t = scnprintf(next, size, "b_sess_vld: %d\n", fsm->b_sess_vld);
	size -= t;
	next += t;

	t = scnprintf(next, size, "id: %d\n", fsm->id);
	size -= t;
	next += t;
#endif

	spin_unlock_irqrestore(&fsm->lock, flags);

	*eof = 1;
	return count - size;
}

#define create_proc_file()	create_proc_read_entry(proc_filename, \
				0, NULL, otg_proc_read, NULL)

#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_OTG_DEBUG_FILES */

#define create_proc_file()	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/*CONFIG_USB_OTG_DEBUG_FILES */

/*----------------------------------------------------------*/
/* Char driver interface to control some OTG input */

/* This function handle some ioctl command,such as get otg
 * status and set host suspend
 */
static int fsl_otg_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	u32 retval = 0;

	switch (cmd) {
	case GET_OTG_STATUS:
		retval = fsl_otg_dev->host_working;
		break;

	case SET_A_SUSPEND_REQ:
		fsl_otg_dev->fsm.a_suspend_req = arg;
		break;

	case SET_A_BUS_DROP:
		fsl_otg_dev->fsm.a_bus_drop = arg;
		break;

	case SET_A_BUS_REQ:
		fsl_otg_dev->fsm.a_bus_req = arg;
		break;

	case SET_B_BUS_REQ:
		fsl_otg_dev->fsm.b_bus_req = arg;
		break;

	default:
		break;
	}

	otg_statemachine(&fsl_otg_dev->fsm);

	return retval;
}

static int fsl_otg_open(struct inode *inode, struct file *file)
{

	return 0;
}

static int fsl_otg_release(struct inode *inode, struct file *file)
{

	return 0;
}

static struct file_operations otg_fops = {
	.owner = THIS_MODULE,
	.llseek = NULL,
	.read = NULL,
	.write = NULL,
	.ioctl = fsl_otg_ioctl,
	.open = fsl_otg_open,
	.release = fsl_otg_release,
};

static int __init fsl_otg_probe(struct platform_device *pdev)
{
	int status;
	struct fsl_usb2_platform_data *pdata;

	DBG("pdev=0x%p\n", pdev);

	if (!pdev)
		return -ENODEV;

	if (!pdev->dev.platform_data)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;

	/* configure the OTG */
	status = fsl_otg_conf(pdev);
	if (status) {
		printk(KERN_INFO "Couldn't init OTG module\n");
		return -status;
	}

	/* start OTG */
	status = usb_otg_start(pdev);

	if (register_chrdev(FSL_OTG_MAJOR, FSL_OTG_NAME, &otg_fops)) {
		printk(KERN_WARNING FSL_OTG_NAME
		       ": unable to register FSL OTG device\n");
		return -EIO;
	}

	create_proc_file();
	return status;
}

static int __exit fsl_otg_remove(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	otg_set_transceiver(NULL);
	free_irq(fsl_otg_dev->irq, fsl_otg_dev);

	iounmap((void *)usb_dr_regs);

	kfree(fsl_otg_dev);

	remove_proc_file();

	unregister_chrdev(FSL_OTG_MAJOR, FSL_OTG_NAME);

	if (pdata->platform_uninit)
		pdata->platform_uninit(pdata);

	return 0;
}

struct platform_driver fsl_otg_driver = {
	.probe = fsl_otg_probe,
	.remove = fsl_otg_remove,
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/

static int __init fsl_usb_otg_init(void)
{
	printk(KERN_INFO DRIVER_DESC " loaded, %s\n", DRIVER_VERSION);
	return platform_driver_register(&fsl_otg_driver);
}

static void __exit fsl_usb_otg_exit(void)
{
	platform_driver_unregister(&fsl_otg_driver);
	printk(KERN_INFO DRIVER_DESC " unloaded\n");
}

module_init(fsl_usb_otg_init);
module_exit(fsl_usb_otg_exit);

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
