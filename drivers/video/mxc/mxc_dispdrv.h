/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MXC_DISPDRV_H__
#define __MXC_DISPDRV_H__

struct mxc_dispdrv_entry;

struct mxc_dispdrv_driver {
	const char *name;
	int (*init) (struct mxc_dispdrv_entry *);
	void (*deinit) (struct mxc_dispdrv_entry *);
};

struct mxc_dispdrv_setting {
	/*input-feedback parameter*/
	struct fb_info *fbi;
	int if_fmt;
	int default_bpp;
	char *dft_mode_str;

	/*feedback parameter*/
	int dev_id;
	int disp_id;
};

struct mxc_dispdrv_entry *mxc_dispdrv_register(struct mxc_dispdrv_driver *drv);
int mxc_dispdrv_unregister(struct mxc_dispdrv_entry *entry);
int mxc_dispdrv_init(char *name, struct mxc_dispdrv_setting *setting);
int mxc_dispdrv_setdata(struct mxc_dispdrv_entry *entry, void *data);
void *mxc_dispdrv_getdata(struct mxc_dispdrv_entry *entry);
struct mxc_dispdrv_setting
	*mxc_dispdrv_getsetting(struct mxc_dispdrv_entry *entry);
#endif
