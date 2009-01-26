/*
 * linux/drivers/char/mxc_si4702.h
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup Character device driver for SI4702 FM radio
 */

/*
 * @file mxc_si4702.h
 *
 * @brief SI4702 Radio FM driver
 *
 * @ingroup Character
 */

#ifndef _MXC_SI4702_FM_H
#define _MXC_SI4702_FM_H

/* define IOCTL command */
#define SI4702_GETVOLUME	_IOR('S', 0x10, unsigned int)
#define SI4702_SETVOLUME	_IOW('S', 0x11, unsigned int)
#define SI4702_MUTEON		_IO('S', 0x12)
#define SI4702_MUTEOFF		_IO('S', 0x13)
#define SI4702_SELECT		_IOW('S', 0x14, unsigned int)
#define SI4702_SEEK		_IOWR('S', 0x15, unsigned int)

#endif				/* _MXC_SI4702_FM_H */
