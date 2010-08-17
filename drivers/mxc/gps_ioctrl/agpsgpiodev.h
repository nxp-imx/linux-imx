/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file agpsgpiodev.h
 *
 * @brief head file of Simple character device interface for AGPS kernel module.
 *
 * @ingroup
 */

#ifndef AGPSGPIODEV_H
#define AGPSGPIODEV_H

#include <linux/ioctl.h>

#define USE_BLOCKING		/* Test driver with blocking calls */
#undef USE_FASYNC		/* Test driver with async notification */

/* The major device number. We can't rely on dynamic registration any more
   because ioctls need to know it */
#define MAJOR_NUM 100

#define IOCTL_WRITEGPIO   _IOWR(MAJOR_NUM, 1, char *)
#define IOCTL_READGPIO    _IOR(MAJOR_NUM, 2, char *)
#define IOCTL_MAXNUMBER   2

/* The name of the device file */
#define AGPSGPIO_DEVICE_FILE_NAME "agpsgpio"

/* Exported prototypes */
int init_chrdev(struct device *dev);
void cleanup_chrdev(void);
void wakeup(void);

#endif
