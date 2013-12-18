/*
 * Copyright (C) 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __PLAT_MXC_EPIT_H
#define __PLAT_MXC_EPIT_H


#define EPITCR		0x00
#define EPITSR		0x04
#define EPITLR		0x08
#define EPITCMPR	0x0c
#define EPITCNR		0x10

#define EPITCR_EN				(1 << 0)
#define EPITCR_ENMOD			(1 << 1)
#define EPITCR_OCIEN			(1 << 2)
#define EPITCR_RLD				(1 << 3)
#define EPITCR_PRESC(x)			(((x) & 0xfff) << 4)
#define EPITCR_SWR				(1 << 16)
#define EPITCR_IOVW				(1 << 17)
#define EPITCR_DBGEN			(1 << 18)
#define EPITCR_WAITEN			(1 << 19)
#define EPITCR_RES				(1 << 20)
#define EPITCR_STOPEN			(1 << 21)
#define EPITCR_OM_DISCON		(0 << 22)
#define EPITCR_OM_TOGGLE		(1 << 22)
#define EPITCR_OM_CLEAR			(2 << 22)
#define EPITCR_OM_SET			(3 << 22)
#define EPITCR_CLKSRC_OFF		(0 << 24)
#define EPITCR_CLKSRC_PERIPHERAL	(1 << 24)
#define EPITCR_CLKSRC_REF_HIGH		(2 << 24)
#define EPITCR_CLKSRC_REF_LOW		(3 << 24)
#define EPIT_FREE_RUN_MODE			(0)
#define EPIT_SET_FORGET_MODE		(1)

#define EPITSR_OCIF					(1 << 0)


struct epit_device;
/*
 * epit_request - request a epit device
 */
struct epit_device *epit_request(int epit_id, const char *label);

/*
 * epit_free - free a epit device
 */
void epit_free(struct epit_device *epit);

/*
 * epit_config - change a epit device configuration
 */
int epit_config(struct epit_device *epit, int mode, void *cb, void *para);

/*
 * epit_start - start a epit output toggling
 */
void  epit_start(struct epit_device *epit, int time_ns);
/*
 * epit_stop - stop a epit output toggling
 */
void  epit_stop(struct epit_device *epit);

#endif /* __LINUX_epit_H */
