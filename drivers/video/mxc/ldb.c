/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file mxc_ldb.c
 *
 * @brief This file contains the LDB driver device interface and fops
 * functions.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include "mxc_dispdrv.h"

#define DISPDRV_LDB	"ldb"

#define LDB_BGREF_RMODE_MASK		0x00008000
#define LDB_BGREF_RMODE_INT		0x00008000
#define LDB_BGREF_RMODE_EXT		0x0

#define LDB_DI1_VS_POL_MASK		0x00000400
#define LDB_DI1_VS_POL_ACT_LOW		0x00000400
#define LDB_DI1_VS_POL_ACT_HIGH		0x0
#define LDB_DI0_VS_POL_MASK		0x00000200
#define LDB_DI0_VS_POL_ACT_LOW		0x00000200
#define LDB_DI0_VS_POL_ACT_HIGH		0x0

#define LDB_BIT_MAP_CH1_MASK		0x00000100
#define LDB_BIT_MAP_CH1_JEIDA		0x00000100
#define LDB_BIT_MAP_CH1_SPWG		0x0
#define LDB_BIT_MAP_CH0_MASK		0x00000040
#define LDB_BIT_MAP_CH0_JEIDA		0x00000040
#define LDB_BIT_MAP_CH0_SPWG		0x0

#define LDB_DATA_WIDTH_CH1_MASK		0x00000080
#define LDB_DATA_WIDTH_CH1_24		0x00000080
#define LDB_DATA_WIDTH_CH1_18		0x0
#define LDB_DATA_WIDTH_CH0_MASK		0x00000020
#define LDB_DATA_WIDTH_CH0_24		0x00000020
#define LDB_DATA_WIDTH_CH0_18		0x0

#define LDB_CH1_MODE_MASK		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI1		0x0000000C
#define LDB_CH1_MODE_EN_TO_DI0		0x00000004
#define LDB_CH1_MODE_DISABLE		0x0
#define LDB_CH0_MODE_MASK		0x00000003
#define LDB_CH0_MODE_EN_TO_DI1		0x00000003
#define LDB_CH0_MODE_EN_TO_DI0		0x00000001
#define LDB_CH0_MODE_DISABLE		0x0

#define LDB_SPLIT_MODE_EN		0x00000010

struct ldb_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_ldb;
	uint32_t *reg;
	uint32_t *control_reg;
	uint32_t *gpr3_reg;
	struct regulator *lvds_bg_reg;
	int mode;
	bool inited;
	struct ldb_setting {
		struct clk *di_clk;
		struct clk *ldb_di_clk;
		bool active;
		int ipu;
		int di;
		uint32_t ch_mask;
		uint32_t ch_val;
	} setting[2];
};

static int g_ldb_mode;
static struct i2c_client *ldb_i2c_client[2];
static u8 g_edid[2][512];

static struct fb_videomode ldb_modedb[] = {
	{
	 "LDB-WXGA", 60, 1280, 800, 14065,
	 40, 40,
	 10, 3,
	 80, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 "LDB-XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 "LDB-1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 FB_MODE_IS_DETAILED,},
};
static int ldb_modedb_sz = ARRAY_SIZE(ldb_modedb);

static int mxc_ldb_edidread(struct i2c_client *client);

static int bits_per_pixel(int pixel_fmt)
{
	switch (pixel_fmt) {
	case IPU_PIX_FMT_BGR24:
	case IPU_PIX_FMT_RGB24:
		return 24;
		break;
	case IPU_PIX_FMT_BGR666:
	case IPU_PIX_FMT_RGB666:
	case IPU_PIX_FMT_LVDS666:
		return 18;
		break;
	default:
		break;
	}
	return 0;
}

static int valid_mode(int pixel_fmt)
{
	return ((pixel_fmt == IPU_PIX_FMT_RGB24) ||
		(pixel_fmt == IPU_PIX_FMT_BGR24) ||
		(pixel_fmt == IPU_PIX_FMT_LVDS666) ||
		(pixel_fmt == IPU_PIX_FMT_RGB666) ||
		(pixel_fmt == IPU_PIX_FMT_BGR666));
}

/*
 *    "ldb=spl0/1"       --      split mode on DI0/1
 *    "ldb=dul0/1"       --      dual mode on DI0/1
 *    "ldb=sin0/1"       --      single mode on LVDS0/1
 *    "ldb=sep0/1" 	 --      separate mode begin from LVDS0/1
 *
 *    there are two LVDS channels(LVDS0 and LVDS1) which can transfer video
 *    datas, there two channels can be used as split/dual/single/separate mode.
 *
 *    split mode means display data from DI0 or DI1 will send to both channels
 *    LVDS0+LVDS1.
 *    dual mode means display data from DI0 or DI1 will be duplicated on LVDS0
 *    and LVDS1, it said, LVDS0 and LVDS1 has the same content.
 *    single mode means only work for DI0/DI1->LVDS0 or DI0/DI1->LVDS1.
 *    separate mode means you can make DI0/DI1->LVDS0 and DI0/DI1->LVDS1 work
 *    at the same time.
 */
static int __init ldb_setup(char *options)
{
	if (!strcmp(options, "spl0"))
		g_ldb_mode = LDB_SPL_DI0;
	else if (!strcmp(options, "spl1"))
		g_ldb_mode = LDB_SPL_DI1;
	else if (!strcmp(options, "dul0"))
		g_ldb_mode = LDB_DUL_DI0;
	else if (!strcmp(options, "dul1"))
		g_ldb_mode = LDB_DUL_DI1;
	else if (!strcmp(options, "sin0"))
		g_ldb_mode = LDB_SIN0;
	else if (!strcmp(options, "sin1"))
		g_ldb_mode = LDB_SIN1;
	else if (!strcmp(options, "sep0"))
		g_ldb_mode = LDB_SEP0;
	else if (!strcmp(options, "sep1"))
		g_ldb_mode = LDB_SEP1;

	return 1;
}
__setup("ldb=", ldb_setup);

static int find_ldb_setting(struct ldb_data *ldb, struct fb_info *fbi)
{
	char *id_di[] = {
		 "DISP3 BG",
		 "DISP3 BG - DI1",
		};
	char id[16];
	int i;

	for (i = 0; i < 2; i++) {
		if (ldb->setting[i].active) {
			memset(id, 0, 16);
			memcpy(id, id_di[ldb->setting[i].di],
				strlen(id_di[ldb->setting[i].di]));
			id[4] += ldb->setting[i].ipu;
			if (!strcmp(id, fbi->fix.id))
				return i;
		}
	}
	return -EINVAL;
}

static int ldb_disp_setup(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	uint32_t reg;
	uint32_t pixel_clk, rounded_pixel_clk;
	struct clk *ldb_clk_parent;
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int setting_idx, di;

	setting_idx = find_ldb_setting(ldb, fbi);
	if (setting_idx < 0)
		return setting_idx;

	di = ldb->setting[setting_idx].di;

	reg = readl(ldb->control_reg);
	/* clear channel mode */
	reg &= ~ldb->setting[setting_idx].ch_mask;
	/* vsync setup */
	if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT) {
		if (di == 0)
			reg = (reg & ~LDB_DI0_VS_POL_MASK)
				| LDB_DI0_VS_POL_ACT_HIGH;
		else
			reg = (reg & ~LDB_DI1_VS_POL_MASK)
				| LDB_DI1_VS_POL_ACT_HIGH;
	} else {
		if (di == 0)
			reg = (reg & ~LDB_DI0_VS_POL_MASK)
				| LDB_DI0_VS_POL_ACT_LOW;
		else
			reg = (reg & ~LDB_DI1_VS_POL_MASK)
				| LDB_DI1_VS_POL_ACT_LOW;
	}
	writel(reg, ldb->control_reg);

	/* clk setup */
	pixel_clk = (PICOS2KHZ(fbi->var.pixclock)) * 1000UL;
	ldb_clk_parent = clk_get_parent(ldb->setting[setting_idx].ldb_di_clk);
	if ((ldb->mode == LDB_SPL_DI0) || (ldb->mode == LDB_SPL_DI1))
		clk_set_rate(ldb_clk_parent, pixel_clk * 7 / 2);
	else
		clk_set_rate(ldb_clk_parent, pixel_clk * 7);
	rounded_pixel_clk = clk_round_rate(ldb->setting[setting_idx].ldb_di_clk,
			pixel_clk);
	clk_set_rate(ldb->setting[setting_idx].ldb_di_clk, rounded_pixel_clk);

	return 0;
}

static int ldb_disp_enable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int index;
	uint32_t reg;

	index = find_ldb_setting(ldb, fbi);
	if (index < 0)
		return index;

	reg = readl(ldb->control_reg);
	reg |= ldb->setting[index].ch_val;
	writel(reg, ldb->control_reg);

	return 0;
}

static void ldb_disp_disable(struct mxc_dispdrv_handle *disp,
			     struct fb_info *fbi)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int index;
	uint32_t reg;

	index = find_ldb_setting(ldb, fbi);
	if (index < 0)
		return;

	reg = readl(ldb->control_reg);
	reg &= ~ldb->setting[index].ch_mask;
	writel(reg, ldb->control_reg);
}

#define LVDS_MUX_CTL_WIDTH	2
#define LVDS_MUX_CTL_MASK	3
#define LVDS0_MUX_CTL_OFFS	6
#define LVDS1_MUX_CTL_OFFS	8
#define LVDS0_MUX_CTL_MASK	(LVDS_MUX_CTL_MASK << 6)
#define LVDS1_MUX_CTL_MASK	(LVDS_MUX_CTL_MASK << 8)
#define ROUTE_IPU_DI(ipu, di)	(((ipu << 1) | di) & LVDS_MUX_CTL_MASK)
static int ldb_ipu_ldb_route(int ipu, int di, struct ldb_data *ldb)
{
	uint32_t reg;
	int channel;
	int shift;
	int mode = ldb->mode;

	reg = readl(ldb->gpr3_reg);
	if (mode < LDB_SIN0) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		reg |= (ROUTE_IPU_DI(ipu, di) << LVDS0_MUX_CTL_OFFS) |
			(ROUTE_IPU_DI(ipu, di) << LVDS1_MUX_CTL_OFFS);
		dev_dbg(&ldb->pdev->dev,
			"Dual/Split mode both channels route to IPU%d-DI%d\n",
			ipu, di);
	} else if ((mode == LDB_SIN0) || (mode == LDB_SIN1)) {
		reg &= ~(LVDS0_MUX_CTL_MASK | LVDS1_MUX_CTL_MASK);
		channel = mode - LDB_SIN0;
		shift = LVDS0_MUX_CTL_OFFS + channel * LVDS_MUX_CTL_WIDTH;
		reg |= ROUTE_IPU_DI(ipu, di) << shift;
		dev_dbg(&ldb->pdev->dev,
			"Single mode channel %d route to IPU%d-DI%d\n",
				channel, ipu, di);
	} else {
		static bool first = true;

		if (first) {
			if (mode == LDB_SEP0) {
				reg &= ~LVDS0_MUX_CTL_MASK;
				channel = 0;
			} else {
				reg &= ~LVDS1_MUX_CTL_MASK;
				channel = 1;
			}
			first = false;
		} else {
			if (mode == LDB_SEP0) {
				reg &= ~LVDS1_MUX_CTL_MASK;
				channel = 1;
			} else {
				reg &= ~LVDS0_MUX_CTL_MASK;
				channel = 0;
			}
		}

		shift = LVDS0_MUX_CTL_OFFS + channel * LVDS_MUX_CTL_WIDTH;
		reg |= ROUTE_IPU_DI(ipu, di) << shift;

		dev_dbg(&ldb->pdev->dev,
			"Separate mode channel %d route to IPU%d-DI%d\n",
			channel, ipu, di);
	}
	writel(reg, ldb->gpr3_reg);

	return 0;
}

static int ldb_disp_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret = 0, i;
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	struct fsl_mxc_ldb_platform_data *plat_data = ldb->pdev->dev.platform_data;
	struct i2c_client *i2c_dev;
	struct resource *res;
	uint32_t base_addr;
	uint32_t reg, setting_idx;
	uint32_t ch_mask = 0, ch_val = 0;
	int lvds_channel = 0;
	uint32_t ipu_id, disp_id;

	/* if input format not valid, make RGB666 as default*/
	if (!valid_mode(setting->if_fmt)) {
		dev_warn(&ldb->pdev->dev, "Input pixel format not valid"
					" use default RGB666\n");
		setting->if_fmt = IPU_PIX_FMT_RGB666;
	}

	if (!ldb->inited) {
		char di_clk[] = "ipu1_di0_clk";
		char ldb_clk[] = "ldb_di0_clk";

		setting_idx = 0;
		res = platform_get_resource(ldb->pdev, IORESOURCE_MEM, 0);
		if (IS_ERR(res))
			return -ENOMEM;

		base_addr = res->start;
		ldb->reg = ioremap(base_addr, res->end - res->start + 1);
		ldb->control_reg = ldb->reg + 2;
		ldb->gpr3_reg = ldb->reg + 3;

		ldb->lvds_bg_reg = regulator_get(&ldb->pdev->dev, plat_data->lvds_bg_reg);
		if (!IS_ERR(ldb->lvds_bg_reg)) {
			regulator_set_voltage(ldb->lvds_bg_reg, 2500000, 2500000);
			regulator_enable(ldb->lvds_bg_reg);
		}

		/* ipu selected by platform data setting */
		setting->dev_id = plat_data->ipu_id;

		reg = readl(ldb->control_reg);

		/* refrence resistor select */
		reg &= ~LDB_BGREF_RMODE_MASK;
		if (plat_data->ext_ref)
			reg |= LDB_BGREF_RMODE_EXT;
		else
			reg |= LDB_BGREF_RMODE_INT;

		/* TODO: now only use SPWG data mapping for both channel */
		reg &= ~(LDB_BIT_MAP_CH0_MASK | LDB_BIT_MAP_CH1_MASK);
		reg |= LDB_BIT_MAP_CH0_SPWG | LDB_BIT_MAP_CH1_SPWG;

		/* channel mode setting */
		reg &= ~(LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
		reg &= ~(LDB_DATA_WIDTH_CH0_MASK | LDB_DATA_WIDTH_CH1_MASK);

		if (bits_per_pixel(setting->if_fmt) == 24)
			reg |= LDB_DATA_WIDTH_CH0_24 | LDB_DATA_WIDTH_CH1_24;
		else
			reg |= LDB_DATA_WIDTH_CH0_18 | LDB_DATA_WIDTH_CH1_18;

		if (g_ldb_mode)
			ldb->mode = g_ldb_mode;
		else
			ldb->mode = plat_data->mode;

		if ((ldb->mode == LDB_SIN0) || (ldb->mode == LDB_SIN1)) {
			ret = ldb->mode - LDB_SIN0;
			if (plat_data->disp_id != ret) {
				dev_warn(&ldb->pdev->dev,
					"change IPU DI%d to IPU DI%d for LDB "
					"channel%d.\n",
					plat_data->disp_id, ret, ret);
				plat_data->disp_id = ret;
			}
		} else if (((ldb->mode == LDB_SEP0) || (ldb->mode == LDB_SEP1))
				&& (cpu_is_mx6q() || cpu_is_mx6dl())) {
			if (plat_data->disp_id == plat_data->sec_disp_id) {
				dev_err(&ldb->pdev->dev,
					"For LVDS separate mode,"
					"two DIs should be different!\n");
				return -EINVAL;
			}

			if (((!plat_data->disp_id) && (ldb->mode == LDB_SEP1))
				|| ((plat_data->disp_id) &&
					(ldb->mode == LDB_SEP0))) {
				dev_dbg(&ldb->pdev->dev,
					"LVDS separate mode:"
					"swap DI configuration!\n");
				ipu_id = plat_data->ipu_id;
				disp_id = plat_data->disp_id;
				plat_data->ipu_id = plat_data->sec_ipu_id;
				plat_data->disp_id = plat_data->sec_disp_id;
				plat_data->sec_ipu_id = ipu_id;
				plat_data->sec_disp_id = disp_id;
			}
		}

		if (ldb->mode == LDB_SPL_DI0) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI0
				| LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_SPL_DI1) {
			reg |= LDB_SPLIT_MODE_EN | LDB_CH0_MODE_EN_TO_DI1
				| LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_DUL_DI0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI0 | LDB_CH1_MODE_EN_TO_DI0;
			setting->disp_id = 0;
		} else if (ldb->mode == LDB_DUL_DI1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			reg |= LDB_CH0_MODE_EN_TO_DI1 | LDB_CH1_MODE_EN_TO_DI1;
			setting->disp_id = 1;
		} else if (ldb->mode == LDB_SIN0) {
			reg &= ~LDB_SPLIT_MODE_EN;
			setting->disp_id = plat_data->disp_id;
			if (setting->disp_id == 0)
				reg |= LDB_CH0_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH0_MODE_EN_TO_DI1;
			ch_mask = LDB_CH0_MODE_MASK;
			ch_val = reg & LDB_CH0_MODE_MASK;
		} else if (ldb->mode == LDB_SIN1) {
			reg &= ~LDB_SPLIT_MODE_EN;
			setting->disp_id = plat_data->disp_id;
			if (setting->disp_id == 0)
				reg |= LDB_CH1_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH1_MODE_EN_TO_DI1;
			ch_mask = LDB_CH1_MODE_MASK;
			ch_val = reg & LDB_CH1_MODE_MASK;
		} else { /* separate mode*/
			setting->disp_id = plat_data->disp_id;

			/* first output is LVDS0 or LVDS1 */
			if (ldb->mode == LDB_SEP0)
				lvds_channel = 0;
			else
				lvds_channel = 1;

			reg &= ~LDB_SPLIT_MODE_EN;

			if ((lvds_channel == 0) && (setting->disp_id == 0))
				reg |= LDB_CH0_MODE_EN_TO_DI0;
			else if ((lvds_channel == 0) && (setting->disp_id == 1))
				reg |= LDB_CH0_MODE_EN_TO_DI1;
			else if ((lvds_channel == 1) && (setting->disp_id == 0))
				reg |= LDB_CH1_MODE_EN_TO_DI0;
			else
				reg |= LDB_CH1_MODE_EN_TO_DI1;
			ch_mask = lvds_channel ? LDB_CH1_MODE_MASK :
					LDB_CH0_MODE_MASK;
			ch_val = reg & ch_mask;

			if (bits_per_pixel(setting->if_fmt) == 24) {
				if (lvds_channel == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_24;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_24;
			} else {
				if (lvds_channel == 0)
					reg &= ~LDB_DATA_WIDTH_CH1_18;
				else
					reg &= ~LDB_DATA_WIDTH_CH0_18;
			}
		}

		writel(reg, ldb->control_reg);
		if (ldb->mode <  LDB_SIN0) {
			ch_mask = LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK;
			ch_val = reg & (LDB_CH0_MODE_MASK | LDB_CH1_MODE_MASK);
		}

		/* clock setting */
		if ((cpu_is_mx6q() || cpu_is_mx6dl()) &&
			((ldb->mode == LDB_SEP0) || (ldb->mode == LDB_SEP1)))
			ldb_clk[6] += lvds_channel;
		else
			ldb_clk[6] += setting->disp_id;
		ldb->setting[setting_idx].ldb_di_clk = clk_get(&ldb->pdev->dev,
								ldb_clk);
		if (IS_ERR(ldb->setting[setting_idx].ldb_di_clk)) {
			dev_err(&ldb->pdev->dev, "get ldb clk0 failed\n");
			iounmap(ldb->reg);
			return PTR_ERR(ldb->setting[setting_idx].ldb_di_clk);
		}
		di_clk[3] += setting->dev_id;
		di_clk[7] += setting->disp_id;
		ldb->setting[setting_idx].di_clk = clk_get(&ldb->pdev->dev,
								di_clk);
		if (IS_ERR(ldb->setting[setting_idx].di_clk)) {
			dev_err(&ldb->pdev->dev, "get di clk0 failed\n");
			iounmap(ldb->reg);
			return PTR_ERR(ldb->setting[setting_idx].di_clk);
		}

		dev_dbg(&ldb->pdev->dev, "ldb_clk to di clk: %s -> %s\n", ldb_clk, di_clk);

		ldb->inited = true;

		i2c_dev = ldb_i2c_client[lvds_channel];

	} else { /* second time for separate mode */
		char di_clk[] = "ipu1_di0_clk";
		char ldb_clk[] = "ldb_di0_clk";

		if ((ldb->mode == LDB_SPL_DI0) ||
			(ldb->mode == LDB_SPL_DI1) ||
			(ldb->mode == LDB_DUL_DI0) ||
			(ldb->mode == LDB_DUL_DI1) ||
			(ldb->mode == LDB_SIN0) ||
			(ldb->mode == LDB_SIN1)) {
			dev_err(&ldb->pdev->dev, "for second ldb disp"
					"ldb mode should in separate mode\n");
			return -EINVAL;
		}

		setting_idx = 1;
		if (cpu_is_mx6q() || cpu_is_mx6dl()) {
			setting->dev_id = plat_data->sec_ipu_id;
			setting->disp_id = plat_data->sec_disp_id;
		} else {
			setting->dev_id = plat_data->ipu_id;
			setting->disp_id = !plat_data->disp_id;
		}
		if (setting->disp_id == ldb->setting[0].di) {
			dev_err(&ldb->pdev->dev, "Err: for second ldb disp in"
				"separate mode, DI should be different!\n");
			return -EINVAL;
		}

		/* second output is LVDS0 or LVDS1 */
		if (ldb->mode == LDB_SEP0)
			lvds_channel = 1;
		else
			lvds_channel = 0;

		reg = readl(ldb->control_reg);
		if ((lvds_channel == 0) && (setting->disp_id == 0))
			reg |= LDB_CH0_MODE_EN_TO_DI0;
		else if ((lvds_channel == 0) && (setting->disp_id == 1))
			reg |= LDB_CH0_MODE_EN_TO_DI1;
		else if ((lvds_channel == 1) && (setting->disp_id == 0))
			reg |= LDB_CH1_MODE_EN_TO_DI0;
		else
			reg |= LDB_CH1_MODE_EN_TO_DI1;
		ch_mask = lvds_channel ?  LDB_CH1_MODE_MASK :
				LDB_CH0_MODE_MASK;
		ch_val = reg & ch_mask;

		if (bits_per_pixel(setting->if_fmt) == 24) {
			if (lvds_channel == 0)
				reg |= LDB_DATA_WIDTH_CH0_24;
			else
				reg |= LDB_DATA_WIDTH_CH1_24;
		} else {
			if (lvds_channel == 0)
				reg |= LDB_DATA_WIDTH_CH0_18;
			else
				reg |= LDB_DATA_WIDTH_CH1_18;
		}
		writel(reg, ldb->control_reg);

		/* clock setting */
		if (cpu_is_mx6q() || cpu_is_mx6dl())
			ldb_clk[6] += lvds_channel;
		else
			ldb_clk[6] += setting->disp_id;
		ldb->setting[setting_idx].ldb_di_clk = clk_get(&ldb->pdev->dev,
								ldb_clk);
		if (IS_ERR(ldb->setting[setting_idx].ldb_di_clk)) {
			dev_err(&ldb->pdev->dev, "get ldb clk1 failed\n");
			return PTR_ERR(ldb->setting[setting_idx].ldb_di_clk);
		}
		di_clk[3] += setting->dev_id;
		di_clk[7] += setting->disp_id;
		ldb->setting[setting_idx].di_clk = clk_get(&ldb->pdev->dev,
								di_clk);
		if (IS_ERR(ldb->setting[setting_idx].di_clk)) {
			dev_err(&ldb->pdev->dev, "get di clk1 failed\n");
			return PTR_ERR(ldb->setting[setting_idx].di_clk);
		}

		i2c_dev = ldb_i2c_client[lvds_channel];

		dev_dbg(&ldb->pdev->dev, "ldb_clk to di clk: %s -> %s\n", ldb_clk, di_clk);
	}

	if (i2c_dev)
		mxc_dispdrv_setdev(ldb->disp_ldb, &i2c_dev->dev);
	else
		mxc_dispdrv_setdev(ldb->disp_ldb, NULL);

	ldb->setting[setting_idx].ch_mask = ch_mask;
	ldb->setting[setting_idx].ch_val = ch_val;

	if (cpu_is_mx6q() || cpu_is_mx6dl())
		ldb_ipu_ldb_route(setting->dev_id, setting->disp_id, ldb);

	/*
	 * ldb_di0_clk -> ipux_di0_clk
	 * ldb_di1_clk -> ipux_di1_clk
	 */
	clk_set_parent(ldb->setting[setting_idx].di_clk,
			ldb->setting[setting_idx].ldb_di_clk);

	/* must use spec video mode defined by driver */
	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				ldb_modedb, ldb_modedb_sz, NULL, setting->default_bpp);
	if (ret != 1)
		fb_videomode_to_var(&setting->fbi->var, &ldb_modedb[0]);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < ldb_modedb_sz; i++) {
		struct fb_videomode m;
		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &ldb_modedb[i])) {
			fb_add_videomode(&ldb_modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	/* get screen size in edid */
	if (i2c_dev) {
		ret = mxc_ldb_edidread(i2c_dev);
		if (ret > 0) {
			fb_edid_to_monspecs(&g_edid[lvds_channel][0],
						&setting->fbi->monspecs);
			/* centimeter to millimeter */
			setting->fbi->var.width =
					setting->fbi->monspecs.max_x * 10;
			setting->fbi->var.height =
					setting->fbi->monspecs.max_y * 10;
		} else {
			/* ignore i2c access failure */
			ret = 0;
		}
	}

	/* save current ldb setting for fb notifier */
	ldb->setting[setting_idx].active = true;
	ldb->setting[setting_idx].ipu = setting->dev_id;
	ldb->setting[setting_idx].di = setting->disp_id;

	return ret;
}

static void ldb_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct ldb_data *ldb = mxc_dispdrv_getdata(disp);
	int i;

	writel(0, ldb->control_reg);

	for (i = 0; i < 2; i++) {
		clk_disable(ldb->setting[i].ldb_di_clk);
		clk_put(ldb->setting[i].ldb_di_clk);
	}

	iounmap(ldb->reg);
}

static struct mxc_dispdrv_driver ldb_drv = {
	.name 	= DISPDRV_LDB,
	.init 	= ldb_disp_init,
	.deinit	= ldb_disp_deinit,
	.setup = ldb_disp_setup,
	.enable = ldb_disp_enable,
	.disable = ldb_disp_disable,
};

static int mxc_ldb_edidread(struct i2c_client *client)
{
	int ret = 0;
	unsigned char regaddr = 0;
	struct i2c_adapter *adp = client->adapter;
	int ldb_id = (int)client->dev.platform_data;
	struct i2c_msg msg[2] = {
		{
		.addr	= 0x50,
		.flags	= 0,
		.len	= 1,
		.buf	= &regaddr,
		}, {
		.addr	= 0x50,
		.flags	= I2C_M_RD,
		.len	= 512,
		.buf	= &g_edid[ldb_id][0],
		},
	};

	ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		return -EIO;
	}

	return ret;
}

static ssize_t mxc_ldb_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = mxc_ldb_edidread(client);
	if (ret < 0)
		strcpy(buf, "plugout\n");
	else
		strcpy(buf, "plugin\n");

	return strlen(buf);
}

static DEVICE_ATTR(cable_state, S_IRUGO, mxc_ldb_show_state, NULL);

static int __devinit mxc_ldb_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	int ldb_id = (int)client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	ldb_i2c_client[ldb_id] = client;

	ret = device_create_file(&client->dev, &dev_attr_cable_state);
	if (ret < 0)
		dev_warn(&client->dev,
			"cound not create sys node for cable state\n");


	return 0;
}

static int __devexit mxc_ldb_i2c_remove(struct i2c_client *client)
{
	int ldb_id = (int)client->dev.platform_data;
	ldb_i2c_client[ldb_id] = NULL;
	return 0;
}

static const struct i2c_device_id mxc_ldb_i2c_id[] = {
	{ "mxc_ldb_i2c", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mxc_ldb_i2c_id);

static struct i2c_driver mxc_ldb_i2c_driver = {
	.driver = {
		   .name = "mxc_ldb_i2c",
		   },
	.probe = mxc_ldb_i2c_probe,
	.remove = mxc_ldb_i2c_remove,
	.id_table = mxc_ldb_i2c_id,
};

static int __init mxc_ldb_i2c_init(void)
{
	return i2c_add_driver(&mxc_ldb_i2c_driver);
}

static void __exit mxc_ldb_i2c_exit(void)
{
	i2c_del_driver(&mxc_ldb_i2c_driver);
}

module_init(mxc_ldb_i2c_init);
module_exit(mxc_ldb_i2c_exit);

/*!
 * This function is called by the driver framework to initialize the LDB
 * device.
 *
 * @param	dev	The device structure for the LDB passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int ldb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct ldb_data *ldb;

	ldb = kzalloc(sizeof(struct ldb_data), GFP_KERNEL);
	if (!ldb) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	ldb->pdev = pdev;
	ldb->disp_ldb = mxc_dispdrv_register(&ldb_drv);
	mxc_dispdrv_setdata(ldb->disp_ldb, ldb);

	dev_set_drvdata(&pdev->dev, ldb);

alloc_failed:
	return ret;
}

static int ldb_remove(struct platform_device *pdev)
{
	struct ldb_data *ldb = dev_get_drvdata(&pdev->dev);

	if (!ldb->inited)
		return 0;
	mxc_dispdrv_puthandle(ldb->disp_ldb);
	mxc_dispdrv_unregister(ldb->disp_ldb);
	kfree(ldb);
	return 0;
}

static struct platform_driver mxcldb_driver = {
	.driver = {
		   .name = "mxc_ldb",
		   },
	.probe = ldb_probe,
	.remove = ldb_remove,
};

static int __init ldb_init(void)
{
	return platform_driver_register(&mxcldb_driver);
}

static void __exit ldb_uninit(void)
{
	platform_driver_unregister(&mxcldb_driver);
}

module_init(ldb_init);
module_exit(ldb_uninit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC LDB driver");
MODULE_LICENSE("GPL");
