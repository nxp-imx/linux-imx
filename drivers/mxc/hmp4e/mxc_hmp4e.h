/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Encoder device driver (kernel module headers)
 *
 * Copyright (C) 2005  Hantro Products Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */
#ifndef _HMP4ENC_H_
#define _HMP4ENC_H_
#include <linux/ioctl.h>	/* needed for the _IOW etc stuff used later */

/* this is for writing data through ioctl to registers*/
typedef struct {
	unsigned long data;
	unsigned long offset;
} write_t;

/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define HMP4E_IOC_MAGIC  'k'
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
#define HMP4E_IOCGBUFBUSADDRESS	_IOR(HMP4E_IOC_MAGIC,  1, unsigned long *)
#define HMP4E_IOCGBUFSIZE	_IOR(HMP4E_IOC_MAGIC,  2, unsigned int *)
#define HMP4E_IOCGHWOFFSET     	_IOR(HMP4E_IOC_MAGIC,  3, unsigned long *)
#define HMP4E_IOCGHWIOSIZE     	_IOR(HMP4E_IOC_MAGIC,  4, unsigned int *)
#define HMP4E_IOC_CLI          	_IO(HMP4E_IOC_MAGIC,   5)
#define HMP4E_IOC_STI          	_IO(HMP4E_IOC_MAGIC,   6)
#define HMP4E_IOCHARDRESET 	_IO(HMP4E_IOC_MAGIC,   7)
#define HMP4E_IOCSREGWRITE	_IOW(HMP4E_IOC_MAGIC,  8, write_t)
#define HMP4E_IOCXREGREAD	_IOWR(HMP4E_IOC_MAGIC, 9, unsigned long)

#define HMP4E_IOC_MAXNR 9

#endif				/* !_HMP4ENC_H_ */
