/*
 *fs_sdio_api.h - Freescale SDIO glue module API for UniFi.
 *
 * Copyright (C) 2008 Cambridge Silicon Radio Ltd.
 *
 */
/*
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
#ifndef _FS_SDIO_API_H
#define _FS_SDIO_API_H

struct sdio_dev;

struct fs_driver {
	const char *name;
	int (*probe)(struct sdio_dev *fdev);
	void (*remove)(struct sdio_dev *fdev);
	void (*card_int_handler)(struct sdio_dev *fdev);
	void (*suspend)(struct sdio_dev *fdev, pm_message_t state);
	void (*resume)(struct sdio_dev *fdev);
};

int fs_sdio_readb(struct sdio_dev *fdev, int funcnum,
		  unsigned long addr, unsigned char *pdata);
int fs_sdio_writeb(struct sdio_dev *fdev, int funcnum,
		   unsigned long addr, unsigned char data);
int fs_sdio_block_rw(struct sdio_dev *fdev, int funcnum,
		     unsigned long addr, unsigned char *pdata,
		     unsigned int count, int direction);

int fs_sdio_register_driver(struct fs_driver *driver);
void fs_sdio_unregister_driver(struct fs_driver *driver);
int fs_sdio_set_block_size(struct sdio_dev *fdev, int blksz);
int fs_sdio_set_max_clock_speed(struct sdio_dev *fdev, int max_khz);
int fs_sdio_enable_interrupt(struct sdio_dev *fdev, int enable);
int fs_sdio_enable(struct sdio_dev *fdev);
int fs_sdio_hard_reset(struct sdio_dev *fdev);

struct sdio_dev {
	/**< Device driver for this module. */
	struct fs_driver *driver;

	struct sdio_func *func;

	/**< Data private to the device driver. */
	void *drv_data;

	int int_enabled;
	spinlock_t lock;

	uint16_t vendor_id;     /**< Vendor ID of the card. */
	uint16_t device_id;     /**< Device ID of the card. */

	/**< Maximum block size supported. */
	int max_blocksize;
};


#endif	/* #ifndef _FS_SDIO_API_H */
