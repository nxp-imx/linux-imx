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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <linux/fsl_devices.h>
#include "mxc_dispdrv.h"

struct mxc_lcdif_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_lcdif;
};

#define DISPDRV_LCD	"lcd"

static struct fb_videomode lcdif_modedb[] = {
	{
	/* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	"CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	"SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
};
static int lcdif_modedb_sz = ARRAY_SIZE(lcdif_modedb);

static int lcdif_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret, i;
	struct mxc_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_lcd_platform_data *plat_data
			= lcdif->pdev->dev.platform_data;
	struct fb_videomode *modedb = lcdif_modedb;
	int modedb_sz = lcdif_modedb_sz;

	/* use platform defined ipu/di */
	setting->dev_id = plat_data->ipu_id;
	setting->disp_id = plat_data->disp_id;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		struct fb_videomode m;
		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &modedb[i])) {
			fb_add_videomode(&modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	return ret;
}

void lcdif_deinit(struct mxc_dispdrv_handle *disp)
{
	/*TODO*/
}

static struct mxc_dispdrv_driver lcdif_drv = {
	.name 	= DISPDRV_LCD,
	.init 	= lcdif_init,
	.deinit	= lcdif_deinit,
};

static int mxc_lcdif_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxc_lcdif_data *lcdif;

	lcdif = kzalloc(sizeof(struct mxc_lcdif_data), GFP_KERNEL);
	if (!lcdif) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	lcdif->pdev = pdev;
	lcdif->disp_lcdif = mxc_dispdrv_register(&lcdif_drv);
	mxc_dispdrv_setdata(lcdif->disp_lcdif, lcdif);

	dev_set_drvdata(&pdev->dev, lcdif);

alloc_failed:
	return ret;
}

static int mxc_lcdif_remove(struct platform_device *pdev)
{
	struct mxc_lcdif_data *lcdif = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(lcdif->disp_lcdif);
	mxc_dispdrv_unregister(lcdif->disp_lcdif);
	kfree(lcdif);
	return 0;
}

static struct platform_driver mxc_lcdif_driver = {
	.driver = {
		   .name = "mxc_lcdif",
		   },
	.probe = mxc_lcdif_probe,
	.remove = mxc_lcdif_remove,
};

static int __init mxc_lcdif_init(void)
{
	return platform_driver_register(&mxc_lcdif_driver);
}

static void __exit mxc_lcdif_exit(void)
{
	platform_driver_unregister(&mxc_lcdif_driver);
}

module_init(mxc_lcdif_init);
module_exit(mxc_lcdif_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ipuv3 LCD extern port driver");
MODULE_LICENSE("GPL");
