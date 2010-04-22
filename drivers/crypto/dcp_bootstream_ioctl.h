/*
 * Freescale DCP driver for bootstream update. Only handles the OTP KEY
 * case and can only encrypt/decrypt.
 *
 * Author: Pantelis Antoniou <pantelis@embeddedalley.com>
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#ifndef DCP_BOOTSTREAM_IOCTL_H
#define DCP_BOOTSTREAM_IOCTL_H

/* remember to have included the proper _IO definition
 * file before hand.
 * For user space it's <sys/ioctl.h>
 */

#define DBS_IOCTL_BASE   'd'

#define DBS_ENC	_IOW(DBS_IOCTL_BASE, 0x00, uint8_t[16])
#define DBS_DEC _IOW(DBS_IOCTL_BASE, 0x01, uint8_t[16])

#endif
