/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxc_edid.c
 *
 * @brief MXC EDID driver
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/i2c.h>
#include <linux/fb.h>
#include <mach/mxc_edid.h>
#include "../edid.h"

#undef DEBUG  /* define this for verbose EDID parsing output */
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(fmt, ## args)
#else
#define DPRINTK(fmt, args...)
#endif

const struct fb_videomode mxc_cea_mode[64] = {
	/* #1: 640x480p@59.94/60Hz 4:3 */
	[1] = {
		NULL, 60, 640, 480, 39722, 48, 16, 33, 10, 96, 2, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_4_3, 0,
	},
	/* #2: 720x480p@59.94/60Hz 4:3 */
	[2] = {
		NULL, 60, 720, 480, 37037, 60, 16, 30, 9, 62, 6, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_4_3, 0,
	},
	/* #3: 720x480p@59.94/60Hz 16:9 */
	[3] = {
		NULL, 60, 720, 480, 37037, 60, 16, 30, 9, 62, 6, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #4: 1280x720p@59.94/60Hz 16:9 */
	[4] = {
		NULL, 60, 1280, 720, 13468, 220, 110, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0
	},
	/* #5: 1920x1080i@59.94/60Hz 16:9 */
	[5] = {
		NULL, 60, 1920, 1080, 13763, 148, 88, 15, 2, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #6: 720(1440)x480iH@59.94/60Hz 4:3 */
	[6] = {
		NULL, 60, 1440, 480, 18554/*37108*/, 114, 38, 15, 4, 124, 3, 0,
		FB_VMODE_INTERLACED | FB_VMODE_ASPECT_4_3, 0,
	},
	/* #7: 720(1440)x480iH@59.94/60Hz 16:9 */
	[7] = {
		NULL, 60, 1440, 480, 18554/*37108*/, 114, 38, 15, 4, 124, 3, 0,
		FB_VMODE_INTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #8: 720(1440)x240pH@59.94/60Hz 4:3 */
	[8] = {
		NULL, 60, 1440, 240, 18554, 114, 38, 16, 4, 124, 3, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #9: 720(1440)x240pH@59.94/60Hz 16:9 */
	[9] = {
		NULL, 60, 1440, 240, 18554, 114, 38, 16, 4, 124, 3, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #16: 1920x1080p@60Hz 16:9 */
	[16] = {
		NULL, 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #17: 720x576pH@50Hz 4:3 */
	[17] = {
		NULL, 50, 720, 576, 37037, 68, 12, 39, 5, 64, 5, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_4_3, 0,
	},
	/* #18: 720x576pH@50Hz 16:9 */
	[18] = {
		NULL, 50, 720, 576, 37037, 68, 12, 39, 5, 64, 5, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #19: 1280x720p@50Hz */
	[19] = {
		NULL, 50, 1280, 720, 13468, 220, 440, 20, 5, 40, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #20: 1920x1080i@50Hz */
	[20] = {
		NULL, 50, 1920, 1080, 13480, 148, 528, 15, 5, 528, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #31: 1920x1080p@50Hz */
	[31] = {
		NULL, 50, 1920, 1080, 6734, 148, 528, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #32: 1920x1080p@23.98/24Hz */
	[32] = {
		NULL, 24, 1920, 1080, 13468, 148, 638, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
	/* #35: (2880)x480p4x@59.94/60Hz */
	[35] = {
		NULL, 60, 2880, 480, 9250, 240, 64, 30, 9, 248, 6, 0,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_4_3, 0,
	},
};

/*
 * We have a special version of fb_mode_is_equal that ignores
 * pixclock, since for many CEA modes, 2 frequencies are supported
 * e.g. 640x480 @ 60Hz or 59.94Hz
 */
int mxc_edid_fb_mode_is_equal(bool use_aspect,
			const struct fb_videomode *mode1,
			const struct fb_videomode *mode2)
{
	u32 mask;

	if (use_aspect)
		mask = ~0;
	else
		mask = ~FB_VMODE_ASPECT_MASK;

	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->hsync_len    == mode2->hsync_len &&
		mode1->vsync_len    == mode2->vsync_len &&
		mode1->left_margin  == mode2->left_margin &&
		mode1->right_margin == mode2->right_margin &&
		mode1->upper_margin == mode2->upper_margin &&
		mode1->lower_margin == mode2->lower_margin &&
		mode1->sync         == mode2->sync &&
		(mode1->vmode & mask) == (mode2->vmode & mask));
}

static void get_detailed_timing(unsigned char *block,
				struct fb_videomode *mode)
{
	mode->xres = H_ACTIVE;
	mode->yres = V_ACTIVE;
	mode->pixclock = PIXEL_CLOCK;
	mode->pixclock /= 1000;
	mode->pixclock = KHZ2PICOS(mode->pixclock);
	mode->right_margin = H_SYNC_OFFSET;
	mode->left_margin = (H_ACTIVE + H_BLANKING) -
		(H_ACTIVE + H_SYNC_OFFSET + H_SYNC_WIDTH);
	mode->upper_margin = V_BLANKING - V_SYNC_OFFSET -
		V_SYNC_WIDTH;
	mode->lower_margin = V_SYNC_OFFSET;
	mode->hsync_len = H_SYNC_WIDTH;
	mode->vsync_len = V_SYNC_WIDTH;
	if (HSYNC_POSITIVE)
		mode->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (VSYNC_POSITIVE)
		mode->sync |= FB_SYNC_VERT_HIGH_ACT;
	mode->refresh = PIXEL_CLOCK/((H_ACTIVE + H_BLANKING) *
				     (V_ACTIVE + V_BLANKING));
	if (INTERLACED) {
		mode->yres *= 2;
		mode->upper_margin *= 2;
		mode->lower_margin *= 2;
		mode->vsync_len *= 2;
		mode->vmode |= FB_VMODE_INTERLACED;
	}
	mode->flag = FB_MODE_IS_DETAILED;

	if ((H_SIZE / 16) == (V_SIZE / 9))
		mode->vmode |= FB_VMODE_ASPECT_16_9;
	else if ((H_SIZE / 4) == (V_SIZE / 3))
		mode->vmode |= FB_VMODE_ASPECT_4_3;
	else if ((mode->xres / 16) == (mode->yres / 9))
		mode->vmode |= FB_VMODE_ASPECT_16_9;
	else if ((mode->xres / 4) == (mode->yres / 3))
		mode->vmode |= FB_VMODE_ASPECT_4_3;

	if (mode->vmode & FB_VMODE_ASPECT_16_9)
		DPRINTK("Aspect ratio: 16:9\n");
	if (mode->vmode & FB_VMODE_ASPECT_4_3)
		DPRINTK("Aspect ratio: 4:3\n");
	DPRINTK("      %d MHz ",  PIXEL_CLOCK/1000000);
	DPRINTK("%d %d %d %d ", H_ACTIVE, H_ACTIVE + H_SYNC_OFFSET,
	       H_ACTIVE + H_SYNC_OFFSET + H_SYNC_WIDTH, H_ACTIVE + H_BLANKING);
	DPRINTK("%d %d %d %d ", V_ACTIVE, V_ACTIVE + V_SYNC_OFFSET,
	       V_ACTIVE + V_SYNC_OFFSET + V_SYNC_WIDTH, V_ACTIVE + V_BLANKING);
	DPRINTK("%sHSync %sVSync\n\n", (HSYNC_POSITIVE) ? "+" : "-",
	       (VSYNC_POSITIVE) ? "+" : "-");
}

int mxc_edid_parse_ext_blk(unsigned char *edid,
		struct mxc_edid_cfg *cfg,
		struct fb_monspecs *specs)
{
	char detail_timing_desc_offset;
	struct fb_videomode *mode, *m;
	unsigned char index = 0x0;
	unsigned char *block;
	int i, num = 0, revision;

	if (edid[index++] != 0x2) /* only support cea ext block now */
		return -1;
	revision = edid[index++];
	DPRINTK("cea extent revision %d\n", revision);
	mode = kzalloc(50 * sizeof(struct fb_videomode), GFP_KERNEL);
	if (mode == NULL)
		return -1;

	detail_timing_desc_offset = edid[index++];

	if (revision >= 2) {
		cfg->cea_underscan = (edid[index] >> 7) & 0x1;
		cfg->cea_basicaudio = (edid[index] >> 6) & 0x1;
		cfg->cea_ycbcr444 = (edid[index] >> 5) & 0x1;
		cfg->cea_ycbcr422 = (edid[index] >> 4) & 0x1;

		DPRINTK("CEA underscan %d\n", cfg->cea_underscan);
		DPRINTK("CEA basicaudio %d\n", cfg->cea_basicaudio);
		DPRINTK("CEA ycbcr444 %d\n", cfg->cea_ycbcr444);
		DPRINTK("CEA ycbcr422 %d\n", cfg->cea_ycbcr422);
	}

	if (revision >= 3) {
		/* short desc */
		DPRINTK("CEA Short desc timmings\n");
		index++;
		while (index < detail_timing_desc_offset) {
			unsigned char tagcode, blklen;

			tagcode = (edid[index] >> 5) & 0x7;
			blklen = (edid[index]) & 0x1f;

			DPRINTK("Tagcode %x Len %d\n", tagcode, blklen);

			switch (tagcode) {
			case 0x2: /*Video data block*/
				{
					int cea_idx;
					i = 0;
					while (i < blklen) {
						index++;
						cea_idx = edid[index] & 0x7f;
						if (cea_idx < ARRAY_SIZE(mxc_cea_mode) &&
								(mxc_cea_mode[cea_idx].xres)) {
							DPRINTK("Support CEA Format #%d\n", cea_idx);
							mode[num] = mxc_cea_mode[cea_idx];
							mode[num].flag |= FB_MODE_IS_STANDARD;
							num++;
						}
						i++;
					}
					break;
				}
			case 0x3: /*Vendor specific data*/
				{
					unsigned char IEEE_reg_iden[3];
					unsigned char deep_color;
					IEEE_reg_iden[0] = edid[index+1];
					IEEE_reg_iden[1] = edid[index+2];
					IEEE_reg_iden[2] = edid[index+3];
					deep_color = edid[index+6];

					if ((IEEE_reg_iden[0] == 0x03) &&
							(IEEE_reg_iden[1] == 0x0c) &&
							(IEEE_reg_iden[2] == 0x00))
						cfg->hdmi_cap = 1;

					if (deep_color & 0x40)
						cfg->vsd_dc_48bit = true;
					if (deep_color & 0x20)
						cfg->vsd_dc_36bit = true;
					if (deep_color & 0x10)
						cfg->vsd_dc_30bit = true;
					if (deep_color & 0x08)
						cfg->vsd_dc_y444 = true;
					if (deep_color & 0x01)
						cfg->vsd_dvi_dual = true;

					DPRINTK("VSD hdmi capability %d\n", cfg->hdmi_cap);
					DPRINTK("VSD support deep color 48bit %d\n", cfg->vsd_dc_48bit);
					DPRINTK("VSD support deep color 36bit %d\n", cfg->vsd_dc_36bit);
					DPRINTK("VSD support deep color 30bit %d\n", cfg->vsd_dc_30bit);
					DPRINTK("VSD support deep color y444 %d\n", cfg->vsd_dc_y444);
					DPRINTK("VSD support dvi dual %d\n", cfg->vsd_dvi_dual);

					index += blklen;
					break;
				}
			case 0x1: /*Audio data block*/
			case 0x4: /*Speaker allocation block*/
			case 0x7: /*User extended block*/
			default:
				/* skip */
				index += blklen;
				break;
			}

			index++;
		}
	}

	/* long desc */
	DPRINTK("CEA long desc timmings\n");
	index = detail_timing_desc_offset;
	block = edid + index;
	while (index < (EDID_LENGTH - DETAILED_TIMING_DESCRIPTION_SIZE)) {
		if (!(block[0] == 0x00 && block[1] == 0x00)) {
			get_detailed_timing(block, &mode[num]);
			num++;
		}
		block += DETAILED_TIMING_DESCRIPTION_SIZE;
		index += DETAILED_TIMING_DESCRIPTION_SIZE;
	}

	if (!num) {
		kfree(mode);
		return 0;
	}

	m = kmalloc((num + specs->modedb_len) *
			sizeof(struct fb_videomode), GFP_KERNEL);
	if (!m)
		return 0;

	if (specs->modedb_len) {
		memmove(m, specs->modedb,
			specs->modedb_len * sizeof(struct fb_videomode));
		kfree(specs->modedb);
	}
	memmove(m+specs->modedb_len, mode,
		num * sizeof(struct fb_videomode));
	kfree(mode);

	specs->modedb_len += num;
	specs->modedb = m;

	return 0;
}

static int mxc_edid_readblk(struct i2c_adapter *adp,
		unsigned short addr, unsigned char *edid)
{
	int ret = 0, extblknum = 0;
	unsigned char regaddr = 0x0;
	struct i2c_msg msg[2] = {
		{
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &regaddr,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		DPRINTK("unable to read EDID block\n");
		return -EIO;
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	extblknum = edid[0x7E];

	if (extblknum) {
		regaddr = 128;
		msg[1].buf = edid + EDID_LENGTH;

		ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
		if (ret != ARRAY_SIZE(msg)) {
			DPRINTK("unable to read EDID ext block\n");
			return -EIO;
		}
	}

	return extblknum;
}

static int mxc_edid_readsegblk(struct i2c_adapter *adp, unsigned short addr,
			unsigned char *edid, int seg_num)
{
	int ret = 0;
	unsigned char segment = 0x1, regaddr = 0;
	struct i2c_msg msg[3] = {
		{
		.addr	= 0x30,
		.flags	= 0,
		.len	= 1,
		.buf	= &segment,
		}, {
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &regaddr,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		DPRINTK("unable to read EDID block\n");
		return -EIO;
	}

	if (seg_num == 2) {
		regaddr = 128;
		msg[2].buf = edid + EDID_LENGTH;

		ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
		if (ret != ARRAY_SIZE(msg)) {
			DPRINTK("unable to read EDID block\n");
			return -EIO;
		}
	}

	return ret;
}

int mxc_edid_var_to_vic(struct fb_var_screeninfo *var)
{
	int i;
	struct fb_videomode m;

	for (i = 0; i < ARRAY_SIZE(mxc_cea_mode); i++) {
		fb_var_to_videomode(&m, var);
		if (mxc_edid_fb_mode_is_equal(false, &m, &mxc_cea_mode[i]))
			break;
	}

	if (i == ARRAY_SIZE(mxc_cea_mode))
		return 0;

	return i;
}

EXPORT_SYMBOL(mxc_edid_var_to_vic);

int mxc_edid_mode_to_vic(const struct fb_videomode *mode)
{
	int i;
	bool use_aspect = (mode->vmode & FB_VMODE_ASPECT_MASK);

	for (i = 0; i < ARRAY_SIZE(mxc_cea_mode); i++) {
		if (mxc_edid_fb_mode_is_equal(use_aspect, mode, &mxc_cea_mode[i]))
			break;
	}

	if (i == ARRAY_SIZE(mxc_cea_mode))
		return 0;

	return i;
}
EXPORT_SYMBOL(mxc_edid_mode_to_vic);

/* make sure edid has 512 bytes*/
int mxc_edid_read(struct i2c_adapter *adp, unsigned short addr,
	unsigned char *edid, struct mxc_edid_cfg *cfg, struct fb_info *fbi)
{
	int ret = 0, extblknum;
	if (!adp || !edid || !cfg || !fbi)
		return -EINVAL;

	memset(edid, 0, EDID_LENGTH*4);
	memset(cfg, 0, sizeof(struct mxc_edid_cfg));

	extblknum = mxc_edid_readblk(adp, addr, edid);
	if (extblknum < 0)
		return extblknum;

	/* edid first block parsing */
	memset(&fbi->monspecs, 0, sizeof(fbi->monspecs));
	fb_edid_to_monspecs(edid, &fbi->monspecs);

	if (extblknum) {
		int i;

		/* need read segment block? */
		if (extblknum > 1) {
			ret = mxc_edid_readsegblk(adp, addr,
				edid + EDID_LENGTH*2, extblknum - 1);
			if (ret < 0)
				return ret;
		}

		for (i = 1; i <= extblknum; i++)
			/* edid ext block parsing */
			mxc_edid_parse_ext_blk(edid + i*EDID_LENGTH,
					cfg, &fbi->monspecs);
	}

	return 0;
}
EXPORT_SYMBOL(mxc_edid_read);

