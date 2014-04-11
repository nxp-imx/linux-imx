/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_vadc.h"
#include "mxc_v4l2_capture.h"

/* Resource names for the VADC driver. */
#define VAFE_REGS_ADDR_RES_NAME "vadc-vafe"
#define VDEC_REGS_ADDR_RES_NAME "vadc-vdec"

#define reg32_write(addr, val) __raw_writel(val, addr)
#define reg32_read(addr)       __raw_readl(addr)
#define reg32setbit(addr, bitpos) \
	reg32_write((addr), (reg32_read((addr)) | (1<<(bitpos))))

#define reg32clrbit(addr, bitpos) \
	reg32_write((addr), (reg32_read((addr)) & (0xFFFFFFFF ^ (1<<(bitpos)))))

#define GPC_CNTR		0x00
#define IMX6SX_GPC_CNTR_VADC_ANALOG_OFF_MASK	BIT(17)
#define IMX6SX_GPC_CNTR_VADC_POWER_DOWN_MASK	BIT(18)

void __iomem *vafe_regbase;
void __iomem *vdec_regbase;

static void __iomem *gpc_regbase;

/*
 * Maintains the information on the current state of the sensor.
 */
struct vadc_data {
	struct sensor_data sen;
	struct clk *vadc_clk;
	struct regmap *gpr;
	u32 vadc_in;
	v4l2_std_id std_id;
};

/* List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	VADC_NTSC = 0,	/* Locked on (M) NTSC video signal. */
	VADC_PAL,		/* (B, G, H, I, N)PAL video signal. */
} video_fmt_idx;

/* Number of video standards supported (including 'not locked' signal). */
#define VADC_STD_MAX		(VADC_PAL + 1)

/* Video format structure. */
typedef struct {
	int v4l2_id;		/* Video for linux ID. */
	char name[16];		/* Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;		/* Raw width. */
	u16 raw_height;		/* Raw height. */
	u16 active_width;	/* Active width. */
	u16 active_height;	/* Active height. */
} video_fmt_t;

/* Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	/* NTSC */
	{
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,
	 .raw_height = 525,
	 .active_width = 720,
	 .active_height = 480,
	 },
	/* (B, G, H, I, N) PAL */
	{
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 },
};

/* Standard index of vadc. */
static video_fmt_idx video_idx = VADC_NTSC;
static void vadc_get_std(struct vadc_data *vadc, v4l2_std_id *std);

static void afe_voltage_clampingmode(void)
{
	reg32_write(AFE_CLAMP, 0x07);
	reg32_write(AFE_CLMPAMP, 0x60);
	reg32_write(AFE_CLMPDAT, 0xF0);
}

static void afe_alwayson_clampingmode(void)
{
	reg32_write(AFE_CLAMP, 0x15);
	reg32_write(AFE_CLMPDAT, 0x08);
	reg32_write(AFE_CLMPAMP, 0x00);
}

static void afe_init(void)
{
	pr_debug("%s\n", __func__);

	reg32_write(AFE_PDBUF, 0x1f);
	reg32_write(AFE_PDADC, 0x0f);
	reg32_write(AFE_PDSARH, 0x01);
	reg32_write(AFE_PDSARL, 0xff);
	reg32_write(AFE_PDADCRFH, 0x01);
	reg32_write(AFE_PDADCRFL, 0xff);
	reg32_write(AFE_ICTRL, 0x3a);
	reg32_write(AFE_ICTLSTG, 0x1e);

	reg32_write(AFE_RCTRLSTG, 0x1e);
	reg32_write(AFE_INPBUF, 0x035);
	reg32_write(AFE_INPFLT, 0x02);
	reg32_write(AFE_ADCDGN, 0x40);
	reg32_write(AFE_TSTSEL, 0x10);

	reg32_write(AFE_ACCTST, 0x07);

	reg32_write(AFE_BGREG, 0x08);

	reg32_write(AFE_ADCGN, 0x09);

	/* set current controlled clamping
	* always on, low current */
	reg32_write(AFE_CLAMP, 0x11);
	reg32_write(AFE_CLMPAMP, 0x08);
}

static void vdec_mode_timing_init(v4l2_std_id std)
{
	if (std == V4L2_STD_NTSC) {
		/* NTSC 720x480 */
		reg32_write(VDEC_HACTS, 0x66);
		reg32_write(VDEC_HACTE, 0x24);

		reg32_write(VDEC_VACTS, 0x29);
		reg32_write(VDEC_VACTE, 0x04);

		/* set V Position */
		reg32_write(VDEC_VRTPOS, 0x2);
	} else if (std == V4L2_STD_PAL) {
		/* PAL 720x576 */
		reg32_write(VDEC_HACTS, 0x66);
		reg32_write(VDEC_HACTE, 0x24);

		reg32_write(VDEC_VACTS, 0x29);
		reg32_write(VDEC_VACTE, 0x04);

		/* set V Position */
		reg32_write(VDEC_VRTPOS, 0x6);
	} else
		pr_debug("Error not support video mode\n");

	/* set H Position */
	reg32_write(VDEC_HZPOS, 0x60);

	/* set H ignore start */
	reg32_write(VDEC_HSIGS, 0xf8);

	/* set H ignore end */
	reg32_write(VDEC_HSIGE, 0x18);
}

/*
* vdec_init()
* Initialises the VDEC registers
* Returns: nothing
*/
static void vdec_init(struct vadc_data *vadc)
{
	v4l2_std_id std;

	pr_debug("%s\n", __func__);

	/* Get work mode PAL or NTSC */
	vadc_get_std(vadc, &std);

	vdec_mode_timing_init(std);

	/* vcr detect threshold high, automatic detections */
	reg32_write(VDEC_VSCON2, 0);

	reg32_write(VDEC_BASE + 0x110, 0x01);

	/* set the noramp mode on the Hloop PLL. */
	reg32_write(VDEC_BASE+(0x14*4), 0x10);

	/* set the YC relative delay.*/
	reg32_write(VDEC_YCDEL, 0x90);

	/* setup the Hpll */
	reg32_write(VDEC_BASE+(0x13*4), 0x13);

	/* setup the 2d comb */
	/* set the gain of the Hdetail output to 3
	 * set the notch alpha gain to 1 */
	reg32_write(VDEC_CFC2, 0x34);

	/* setup various 2d comb bits.*/
	reg32_write(VDEC_BASE+(0x02*4), 0x01);
	reg32_write(VDEC_BASE+(0x03*4), 0x18);
	reg32_write(VDEC_BASE+(0x04*4), 0x34);

	/* set the start of the burst gate */
	reg32_write(VDEC_BRSTGT, 0x30);

	/* set 1f motion gain */
	reg32_write(VDEC_BASE+(0x0f*4), 0x20);

	/* set the 1F chroma motion detector thresh for colour reverse detection */
	reg32_write(VDEC_THSH1, 0x02);
	reg32_write(VDEC_BASE+(0x4a*4), 0x20);
	reg32_write(VDEC_BASE+(0x4b*4), 0x08);

	reg32_write(VDEC_BASE+(0x4c*4), 0x08);

	/* set the threshold for the narrow/wide adaptive chroma BW */
	reg32_write(VDEC_BASE+(0x20*4), 0x20);

	/* turn up the colour with the new colour gain reg */
	/* hue: */
	reg32_write(VDEC_HUE, 0x00);

	/* cbgain: 22 B4 */
	reg32_write(VDEC_CBGN, 0xb4);
	/* cr gain 80 */
	reg32_write(VDEC_CRGN, 0x80);
	/* luma gain (contrast) */
	reg32_write(VDEC_CNTR, 0x80);

	/* setup the signed black level register, brightness */
	reg32_write(VDEC_BRT, 0x00);

	/* filter the standard detection
	 * enable the comb for the ntsc443 */
	reg32_write(VDEC_STDDBG, 0x23);

	/* setup chroma kill thresh for no chroma */
	reg32_write(VDEC_CHBTH, 0x0);

	/* set chroma loop to wider BW
	 * no set it to normal BW. i fixed the bw problem.*/
	reg32_write(VDEC_YCDEL, 0x00);

	/* set the compensation in the chroma loop for the Hloop
	 * set the ratio for the nonarithmetic 3d comb modes.*/
	reg32_write(VDEC_BASE + (0x1d*4), 0x90);

	/* set the threshold for the nonarithmetic mode for the 2d comb
	 * the higher the value the more Fc Fh offset we will tolerate before turning off the comb. */
	reg32_write(VDEC_BASE + (0x33*4), 0xa0);

	/* setup the bluescreen output colour */
	reg32_write(VDEC_BASE + (0x3d*4), 35);
	reg32_write(VDEC_BLSCRCR, 114);
	reg32_write(VDEC_BLSCRCB, 212);

	/* disable the active blanking */
	reg32_write(VDEC_BASE + (0x15*4), 0x02);

	/* setup the luma agc for automatic gain. */
	reg32_write(VDEC_LMAGC2, 0x5e);
	reg32_write(VDEC_BASE + (0x40*4), 0x81);

	/* setup chroma agc */
	reg32_write(VDEC_CHAGC2, 0x09);
	reg32_write(VDEC_BASE + (0x43*4), 0xa0);

	/* setup the MV thresh lower nibble
	 * setup the sync top cap, upper nibble */
	reg32_write(VDEC_BASE + (0x3a*4), 0x80);
	reg32_write(VDEC_SHPIMP, 0x00);

	/* enable div by 4 on out for 10 bit output
	 * enable vga progressive output. */
	reg32_write(VDEC_BASE + (0xf3*4), 0x0c);

	/* set for 11 bit intput
	 * also enable dc offset integrator to go on every clock. */
	reg32_write(VDEC_BASE + (0xf4*4), 0x10);

	/* setup the vsync block */
	reg32_write(VDEC_VSCON1, 0x87);

	/* set the nosignal threshold
	 * set the vsync threshold */
	reg32_write(VDEC_VSSGTH, 0x35);

	/* set length for min hphase filter (or saturate limit if saturate is chosen) */
	reg32_write(VDEC_BASE + (0x45*4), 0x40);

	/* choose the internal 66Mhz clock */
	reg32_write(VDEC_BASE + (0xf8*4), 0x01);

	/* enable the internal resampler,
	 * select min filter not saturate for hphase noise filter for vcr detect.
	 * enable vcr pause mode different field lengths */
	reg32_write(VDEC_BASE + (0x46*4), 0x90);

	/* disable VCR detection, lock to the Hsync rather than the Vsync */
	reg32_write(VDEC_VSCON2, 0x04);

	/* set tiplevel goal for dc clamp. */
	reg32_write(VDEC_BASE + (0x3c*4), 0xB0);

	/* override SECAM detection and force SECAM off */
	reg32_write(VDEC_BASE + (0x2f*4), 0x20);

	/* Set r3d_hardblend in 3D control2 reg */
	reg32_write(VDEC_BASE + (0x0c*4), 0x04);
}

/* set Input selector & input pull-downs */
static void vadc_select_input(int vadc_in)
{
	switch (vadc_in) {
	case 0:
		reg32_write(AFE_INPFLT, 0x02);
		reg32_write(AFE_OFFDRV, 0x00);
		reg32_write(AFE_INPCONFIG, 0x1e);
		break;
	case 1:
		reg32_write(AFE_INPFLT, 0x02);
		reg32_write(AFE_OFFDRV, 0x00);
		reg32_write(AFE_INPCONFIG, 0x2d);
		break;
	case 2:
		reg32_write(AFE_INPFLT, 0x02);
		reg32_write(AFE_OFFDRV, 0x00);
		reg32_write(AFE_INPCONFIG, 0x4b);
		break;
	case 3:
		reg32_write(AFE_INPFLT, 0x02);
		reg32_write(AFE_OFFDRV, 0x00);
		reg32_write(AFE_INPCONFIG, 0x87);
		break;
	default:
		pr_debug("error video input %d\n", vadc_in);
	}
}


static void vadc_power_up(struct vadc_data *vadc)
{
	/* Power on vadc analog */
	reg32clrbit(gpc_regbase + GPC_CNTR, 17);

	/* Power down vadc ext power */
	reg32clrbit(gpc_regbase + GPC_CNTR, 18);

	/* software reset afe  */
	regmap_update_bits(vadc->gpr, IOMUXC_GPR1,
			IMX6SX_GPR1_VADC_SW_RST_MASK,
			IMX6SX_GPR1_VADC_SW_RST_RESET);

	msleep(10);

	/* clock config for vadc */
	reg32_write(VDEC_BASE + 0x320, 0xe3);
	reg32_write(VDEC_BASE + 0x324, 0x38);
	reg32_write(VDEC_BASE + 0x328, 0x8e);
	reg32_write(VDEC_BASE + 0x32c, 0x23);

	/* Release reset bit  */
	regmap_update_bits(vadc->gpr, IOMUXC_GPR1,
			IMX6SX_GPR1_VADC_SW_RST_MASK,
			IMX6SX_GPR1_VADC_SW_RST_RELEASE);

	/* Power on vadc ext power */
	reg32setbit(gpc_regbase + GPC_CNTR, 18);
}

static void vadc_init(struct vadc_data *vadc)
{
	pr_debug("%s\n", __func__);

	vadc_power_up(vadc);

	afe_init();

	/* select Video Input 0-3 */
	vadc_select_input(vadc->vadc_in);

	afe_voltage_clampingmode();

	vdec_init(vadc);

	/*
	* current control loop will move sinewave input off below
	* the bottom of the signal range visible when the testbus is viewed as magnitude,
	* so have to break before this point while capturing ENOB data:
	*/
	afe_alwayson_clampingmode();
}

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void vadc_get_std(struct vadc_data *vadc, v4l2_std_id *std)
{
	int tmp;
	int idx;

	pr_debug("In vadc_get_std\n");

	/* Read PAL mode detected result */
	tmp = reg32_read(VDEC_VIDMOD);
	tmp &= (VDEC_VIDMOD_PAL_MASK | VDEC_VIDMOD_M625_MASK);

	if (tmp) {
		*std = V4L2_STD_PAL;
		idx = VADC_PAL;
	} else {
		*std = V4L2_STD_NTSC;
		idx = VADC_NTSC;
	}

	/* This assumes autodetect which this device uses. */
	if (*std != vadc->std_id) {
		video_idx = idx;
		vadc->std_id = *std;
		vadc->sen.pix.width = video_fmts[video_idx].raw_width;
		vadc->sen.pix.height = video_fmts[video_idx].raw_height;
	}
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	pr_debug("ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -EINVAL;
	}

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	pr_debug("In vadc:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct vadc_data *vadc = s->priv;
	v4l2_std_id std;

	pr_debug("vadc:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 vadc->sen.pix.width, vadc->sen.pix.height);
		f->fmt.pix = vadc->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE:
		vadc_get_std(vadc, &std);
		f->fmt.pix.pixelformat = (u32)std;
		break;
	default:
		f->fmt.pix = vadc->sen.pix;
		break;
	}
	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct vadc_data *vadc = s->priv;
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = vadc->sen.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = vadc->sen.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = vadc->sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = vadc->sen.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = vadc->sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = vadc->sen.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = vadc->sen.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In vadc:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "vadc_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	struct vadc_data *vadc = s->priv;

	/* support only one format  */
	if (fmt->index > 0)
		return -EINVAL;

	fmt->pixelformat = vadc->sen.pix.pixelformat;
	return 0;
}


/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct vadc_data *vadc = s->priv;

	vadc_init(vadc);

	return 0;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc vadc_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num,
	  ioctl_dev_exit},
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_g_chip_ident_num,
	  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static struct v4l2_int_slave vadc_slave = {
	.ioctls = vadc_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(vadc_ioctl_desc),
};

static const struct of_device_id fsl_vadc_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-vadc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_vadc_dt_ids);

static struct v4l2_int_device vadc_int_device = {
	.module = THIS_MODULE,
	.name = "vadc",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &vadc_slave,
	},
};

static int vadc_probe(struct platform_device *pdev)
{
	struct vadc_data *vadc;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *gpc_np;
	struct resource *res;
	u32 csi_id;
	int ret = -1;

	vadc = devm_kzalloc(&pdev->dev,	sizeof(struct vadc_data), GFP_KERNEL);
	if (!vadc) {
		dev_err(&pdev->dev, "Cannot allocate device data\n");
		return -ENOMEM;
	}

	/* Set initial values for the sensor struct. */
	vadc->sen.streamcap.timeperframe.denominator = 30;
	vadc->sen.streamcap.timeperframe.numerator = 1;
	vadc->std_id = V4L2_STD_ALL;
	video_idx = VADC_NTSC;
	vadc->sen.pix.width = video_fmts[video_idx].raw_width;
	vadc->sen.pix.height = video_fmts[video_idx].raw_height;
	vadc->sen.pix.pixelformat = V4L2_PIX_FMT_YUV444;  /* YUV444 */
	vadc->sen.pix.priv = 1;  /* 1 is used to indicate TV in */
	vadc->sen.on = true;

	/* map vafe address  */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, VAFE_REGS_ADDR_RES_NAME);
	if (!res) {
		dev_err(&pdev->dev, "No vafe base address found.\n");
		return -ENOMEM;
	}
	vafe_regbase = devm_ioremap_resource(&pdev->dev, res);
	if (!vafe_regbase) {
		dev_err(&pdev->dev, "ioremap failed with vafe base\n");
		return -ENOMEM;
	}

	/* map vdec address  */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, VDEC_REGS_ADDR_RES_NAME);
	if (!res) {
		dev_err(&pdev->dev, "No vdec base address found.\n");
		return -ENODEV;
	}
	vdec_regbase = devm_ioremap_resource(&pdev->dev, res);
	if (!vdec_regbase) {
		dev_err(&pdev->dev, "ioremap failed with vdec base\n");
		return -ENOMEM;
	}

	/* Get clock */
	vadc->vadc_clk = devm_clk_get(&pdev->dev, "vadc");
	if (IS_ERR(vadc->vadc_clk)) {
		ret = PTR_ERR(vadc->vadc_clk);
		return ret;
	}

	vadc->sen.sensor_clk = devm_clk_get(&pdev->dev, "csi");
	if (IS_ERR(vadc->sen.sensor_clk)) {
		ret = PTR_ERR(vadc->sen.sensor_clk);
		return ret;
	}

	clk_prepare_enable(vadc->sen.sensor_clk);
	clk_prepare_enable(vadc->vadc_clk);

	/* Get csi_id to setting vadc to csi mux in gpr */
	ret = of_property_read_u32(np, "csi_id", &csi_id);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property csi_id\n");
		return ret;
	}

	/* remap GPR register */
	vadc->gpr = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
			"gpr");
	if (IS_ERR(vadc->gpr)) {
		dev_dbg(&pdev->dev, "can not get gpr\n");
		return -ENOMEM;
	}

	/* Configuration vadc-to-csi 0 or 1 */
	if (csi_id) {
		regmap_update_bits(vadc->gpr, IOMUXC_GPR5,
				IMX6SX_GPR5_CSI2_MUX_CTRL_MASK,
				IMX6SX_GPR5_CSI2_MUX_CTRL_CVD);
	} else {
		regmap_update_bits(vadc->gpr, IOMUXC_GPR5,
				IMX6SX_GPR5_CSI1_MUX_CTRL_MASK,
				IMX6SX_GPR5_CSI1_MUX_CTRL_CVD);
	}

	/* Get default vadc_in number  */
	ret = of_property_read_u32(np, "vadc_in", &vadc->vadc_in);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property vadc_in\n");
		return ret;
	}

	/* map GPC register  */
	gpc_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-gpc");
	gpc_regbase = of_iomap(gpc_np, 0);
	if (!gpc_regbase) {
		dev_err(&pdev->dev, "ioremap failed with gpc base\n");
		goto error;
	}

	platform_set_drvdata(pdev, vadc);

	/* Register vadc as v4l2 slave device */
	vadc_int_device.priv = vadc;
	ret = v4l2_int_device_register(&vadc_int_device);

	pr_info("vadc driver loaded\n");

	return ret;

error:
	iounmap(gpc_regbase);
	return ret;
}

static int vadc_remove(struct platform_device *pdev)
{
	struct vadc_data *vadc = platform_get_drvdata(pdev);

	v4l2_int_device_unregister(&vadc_int_device);

	clk_disable_unprepare(vadc->sen.sensor_clk);
	clk_disable_unprepare(vadc->vadc_clk);

	return true;
}

static struct platform_driver vadc_driver = {
	.driver = {
		   .name = "fsl_vadc",
		   .of_match_table = of_match_ptr(fsl_vadc_dt_ids),
		   },
	.probe = vadc_probe,
	.remove = vadc_remove,
};

module_platform_driver(vadc_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("fsl VADC/VDEC driver");
MODULE_LICENSE("GPL");
