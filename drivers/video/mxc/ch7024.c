/*
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ch7024.c
 * @brief Driver for CH7024 TV encoder
 *
 * @ingroup Framebuffer
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <mach/gpio.h>
#include <mach/hw_events.h>

/*!
 * CH7024 registers
 */
#define CH7024_DEVID		0x00
#define CH7024_REVID		0x01
#define CH7024_PG		0x02

#define CH7024_RESET		0x03
#define CH7024_POWER		0x04
#define CH7024_TVHUE		0x05
#define CH7024_TVSAT		0x06
#define CH7024_TVCTA		0x07
#define CH7024_TVBRI		0x08
#define CH7024_TVSHARP		0x09
#define CH7024_OUT_FMT		0x0A
#define CH7024_XTAL		0x0B
#define CH7024_IDF1		0x0C
#define CH7024_IDF2		0x0D
#define CH7024_SYNC		0x0E
#define CH7024_TVFILTER1	0x0F
#define CH7024_TVFILTER2	0x10
#define CH7024_IN_TIMING1	0x11
#define CH7024_IN_TIMING2	0x12
#define CH7024_IN_TIMING3	0x13
#define CH7024_IN_TIMING4	0x14
#define CH7024_IN_TIMING5	0x15
#define CH7024_IN_TIMING6	0x16
#define CH7024_IN_TIMING7	0x17
#define CH7024_IN_TIMING8	0x18
#define CH7024_IN_TIMING9	0x19
#define CH7024_IN_TIMING10	0x1A
#define CH7024_IN_TIMING11	0x1B
#define CH7024_ACIV		0x1C
#define CH7024_CLK_TREE		0x1D
#define CH7024_OUT_TIMING1	0x1E
#define CH7024_OUT_TIMING2	0x1F
#define CH7024_V_POS1		0x20
#define CH7024_V_POS2		0x21
#define CH7024_H_POS1		0x22
#define CH7024_H_POS2		0x23
#define CH7024_PCLK_A1		0x24
#define CH7024_PCLK_A2		0x25
#define CH7024_PCLK_A3		0x26
#define CH7024_PCLK_A4		0x27
#define CH7024_CLK_P1		0x28
#define CH7024_CLK_P2		0x29
#define CH7024_CLK_P3		0x2A
#define CH7024_CLK_N1		0x2B
#define CH7024_CLK_N2		0x2C
#define CH7024_CLK_N3		0x2D
#define CH7024_CLK_T		0x2E
#define CH7024_PLL1		0x2F
#define CH7024_PLL2		0x30
#define CH7024_PLL3		0x31
#define CH7024_SC_FREQ1		0x34
#define CH7024_SC_FREQ2		0x35
#define CH7024_SC_FREQ3		0x36
#define CH7024_SC_FREQ4		0x37
#define CH7024_DAC_TRIM		0x62
#define CH7024_DATA_IO		0x63
#define CH7024_ATT_DISP		0x7E

/*!
 * CH7024 register values
 */
/* video output formats */
#define CH7024_VOS_NTSC_M	0x0
#define CH7024_VOS_NTSC_J	0x1
#define CH7024_VOS_NTSC_443	0x2
#define CH7024_VOS_PAL_BDGHKI	0x3
#define CH7024_VOS_PAL_M	0x4
#define CH7024_VOS_PAL_N	0x5
#define CH7024_VOS_PAL_NC	0x6
#define CH7024_VOS_PAL_60	0x7
/* crystal predefined */
#define CH7024_XTAL_13MHZ	0x4
#define CH7024_XTAL_26MHZ	0xB

/* chip ID */
#define CH7024_DEVICE_ID	0x45

/* clock source define */
#define CLK_HIGH	0
#define CLK_LOW		1

/* CH7024 presets structs */
struct ch7024_clock {
	u32 A;
	u32 P;
	u32 N;
	u32 T;
	u8 PLLN1;
	u8 PLLN2;
	u8 PLLN3;
};

struct ch7024_input_timing {
	u32 HTI;
	u32 VTI;
	u32 HAI;
	u32 VAI;
	u32 HW;
	u32 HO;
	u32 VW;
	u32 VO;
	u32 VOS;
};

#define TVOUT_FMT_OFF	0
#define TVOUT_FMT_NTSC	1
#define TVOUT_FMT_PAL	2

static int enabled;		/* enable power on or not */
static int pm_status;		/* status before suspend */

static struct i2c_client *ch7024_client;
static struct fb_info *ch7024_fbi;
static int ch7024_cur_mode;
static u32 detect_gpio;
static struct regulator *io_reg;
static struct regulator *core_reg;
static struct regulator *analog_reg;

static void hp_detect_wq_handler(struct work_struct *);
DECLARE_DELAYED_WORK(ch7024_wq, hp_detect_wq_handler);

static inline int ch7024_read_reg(u8 reg)
{
	return i2c_smbus_read_byte_data(ch7024_client, reg);
}

static inline int ch7024_write_reg(u8 reg, u8 word)
{
	return i2c_smbus_write_byte_data(ch7024_client, reg, word);
}

/**
 * PAL B/D/G/H/K/I clock and timting structures
 */
static struct ch7024_clock ch7024_clk_pal = {
	.A = 0x0,
	.P = 0x36b00,
	.N = 0x41eb00,
	.T = 0x3f,
	.PLLN1 = 0x0,
	.PLLN2 = 0x1b,
	.PLLN3 = 0x12,
};

static struct ch7024_input_timing ch7024_timing_pal = {
	.HTI = 950,
	.VTI = 560,
	.HAI = 640,
	.VAI = 480,
	.HW = 60,
	.HO = 250,
	.VW = 40,
	.VO = 40,
	.VOS = CH7024_VOS_PAL_BDGHKI,
};

/**
 * NTSC_M clock and timting structures
 * TODO: change values to work well.
 */
static struct ch7024_clock ch7024_clk_ntsc = {
	.A = 0x0,
	.P = 0x2ac90,
	.N = 0x36fc90,
	.T = 0x3f,
	.PLLN1 = 0x0,
	.PLLN2 = 0x1b,
	.PLLN3 = 0x12,
};

static struct ch7024_input_timing ch7024_timing_ntsc = {
	.HTI = 801,
	.VTI = 554,
	.HAI = 640,
	.VAI = 480,
	.HW = 60,
	.HO = 101,
	.VW = 20,
	.VO = 54,
	.VOS = CH7024_VOS_NTSC_M,
};

static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 640, 480, 37594,
	 0, 101,
	 0, 54,
	 60, 20,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 640, 480, 37594,
	 0, 250,
	 0, 40,
	 60, 40,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

/**
 * ch7024_setup
 * initial the CH7024 chipset by setting register
 * @param:
 * 	vos: output video format
 * @return:
 * 	0 successful
 * 	otherwise failed
 */
static int ch7024_setup(int vos)
{
	struct ch7024_input_timing *ch_timing;
	struct ch7024_clock *ch_clk;
#ifdef DEBUG_CH7024
	int i, val;
#endif

	/* select output video format */
	if (vos == TVOUT_FMT_PAL) {
		ch_timing = &ch7024_timing_pal;
		ch_clk = &ch7024_clk_pal;
		pr_debug("CH7024: change to PAL video\n");
	} else if (vos == TVOUT_FMT_NTSC) {
		ch_timing = &ch7024_timing_ntsc;
		ch_clk = &ch7024_clk_ntsc;
		pr_debug("CH7024: change to NTSC video\n");
	} else {

		pr_debug("CH7024: no such video format.\n");
		return -EINVAL;
	}
	ch7024_write_reg(CH7024_RESET, 0x0);
	ch7024_write_reg(CH7024_RESET, 0x3);

	ch7024_write_reg(CH7024_POWER, 0x0C);	/* power on, disable DAC */
	ch7024_write_reg(CH7024_XTAL, CH7024_XTAL_26MHZ);
	ch7024_write_reg(CH7024_SYNC, 0x0D);	/* SLAVE mode, and TTL */
	ch7024_write_reg(CH7024_IDF1, 0x00);
	ch7024_write_reg(CH7024_TVFILTER1, 0x00);	/* set XCH=0 */
	ch7024_write_reg(CH7024_CLK_TREE, 0x9E);	/* Invert input clk */

	/* set input clock and divider */
	/* set PLL */
	ch7024_write_reg(CH7024_PLL1, ch_clk->PLLN1);
	ch7024_write_reg(CH7024_PLL2, ch_clk->PLLN2);
	ch7024_write_reg(CH7024_PLL3, ch_clk->PLLN3);
	/* set A register */
	ch7024_write_reg(CH7024_PCLK_A1, (ch_clk->A >> 24) & 0xFF);
	ch7024_write_reg(CH7024_PCLK_A2, (ch_clk->A >> 16) & 0xFF);
	ch7024_write_reg(CH7024_PCLK_A3, (ch_clk->A >> 8) & 0xFF);
	ch7024_write_reg(CH7024_PCLK_A4, ch_clk->A & 0xFF);
	/* set P register */
	ch7024_write_reg(CH7024_CLK_P1, (ch_clk->P >> 16) & 0xFF);
	ch7024_write_reg(CH7024_CLK_P2, (ch_clk->P >> 8) & 0xFF);
	ch7024_write_reg(CH7024_CLK_P3, ch_clk->P & 0xFF);
	/* set N register */
	ch7024_write_reg(CH7024_CLK_N1, (ch_clk->N >> 16) & 0xFF);
	ch7024_write_reg(CH7024_CLK_N2, (ch_clk->N >> 8) & 0xFF);
	ch7024_write_reg(CH7024_CLK_N3, ch_clk->N & 0xFF);
	/* set T register */
	ch7024_write_reg(CH7024_CLK_T, ch_clk->T & 0xFF);

	/* set sub-carrier frequency generation method */
	ch7024_write_reg(CH7024_ACIV, 0x00);	/* ACIV = 0, automatical SCF */
	/* TV out pattern and DAC switch */
	ch7024_write_reg(CH7024_OUT_FMT, (0x10 | ch_timing->VOS) & 0xFF);

	/* input settings */
	/* input format, RGB666 */
	ch7024_write_reg(CH7024_IDF2, 0x02);
	/* HAI/HTI VAI */
	ch7024_write_reg(CH7024_IN_TIMING1, ((ch_timing->HTI >> 5) & 0x38) |
			 ((ch_timing->HAI >> 8) & 0x07));
	ch7024_write_reg(CH7024_IN_TIMING2, ch_timing->HAI & 0xFF);
	ch7024_write_reg(CH7024_IN_TIMING8, ch_timing->VAI & 0xFF);
	/* HTI VTI */
	ch7024_write_reg(CH7024_IN_TIMING3, ch_timing->HTI & 0xFF);
	ch7024_write_reg(CH7024_IN_TIMING9, ch_timing->VTI & 0xFF);
	/* HW/HO(h) VW */
	ch7024_write_reg(CH7024_IN_TIMING4, ((ch_timing->HW >> 5) & 0x18) |
			 ((ch_timing->HO >> 8) & 0x7));
	ch7024_write_reg(CH7024_IN_TIMING6, ch_timing->HW & 0xFF);
	ch7024_write_reg(CH7024_IN_TIMING11, ch_timing->VW & 0x3F);
	/* HO(l) VO/VAI/VTI */
	ch7024_write_reg(CH7024_IN_TIMING5, ch_timing->HO & 0xFF);
	ch7024_write_reg(CH7024_IN_TIMING7, ((ch_timing->VO >> 4) & 0x30) |
			 ((ch_timing->VTI >> 6) & 0x0C) |
			 ((ch_timing->VAI >> 8) & 0x03));
	ch7024_write_reg(CH7024_IN_TIMING10, ch_timing->VO & 0xFF);

	/* adjust the brightness */
	ch7024_write_reg(CH7024_TVBRI, 0x90);

	ch7024_write_reg(CH7024_OUT_TIMING1, 0x4);
	ch7024_write_reg(CH7024_OUT_TIMING2, 0xe0);

	if (vos == TVOUT_FMT_PAL) {
		ch7024_write_reg(CH7024_V_POS1, 0x03);
		ch7024_write_reg(CH7024_V_POS2, 0x7d);
	} else {
		ch7024_write_reg(CH7024_V_POS1, 0x02);
		ch7024_write_reg(CH7024_V_POS2, 0x7b);
	}

	ch7024_write_reg(CH7024_POWER, 0x00);

#ifdef DEBUG_CH7024
	for (i = 0; i < CH7024_SC_FREQ4; i++) {

		val = ch7024_read_reg(i);
		pr_debug("CH7024, reg[0x%x] = %x\n", i, val);
	}
#endif
	return 0;
}

/**
 * ch7024_enable
 * Enable the ch7024 Power to begin TV encoder
 */
static int ch7024_enable(void)
{
	int en = enabled;

	if (!enabled) {
		regulator_enable(core_reg);
		regulator_enable(io_reg);
		regulator_enable(analog_reg);
		msleep(200);
		enabled = 1;
		ch7024_write_reg(CH7024_POWER, 0x00);
		pr_debug("CH7024 power on.\n");
	}
	return en;
}

/**
 * ch7024_disable
 * Disable the ch7024 Power to stop TV encoder
 */
static void ch7024_disable(void)
{
	if (enabled) {
		enabled = 0;
		ch7024_write_reg(CH7024_POWER, 0x0D);
		regulator_disable(analog_reg);
		regulator_disable(io_reg);
		regulator_disable(core_reg);
		pr_debug("CH7024 power off.\n");
	}
}

static int ch7024_detect(void)
{
	int en;
	int detect = 0;

	if (gpio_get_value(detect_gpio) == 1) {
		set_irq_type(ch7024_client->irq, IRQF_TRIGGER_FALLING);

		en = ch7024_enable();

		ch7024_write_reg(CH7024_DAC_TRIM, 0xB4);
		msleep(50);
		detect = ch7024_read_reg(CH7024_ATT_DISP) & 0x3;
		ch7024_write_reg(CH7024_DAC_TRIM, 0x34);

		if (!en)
			ch7024_disable();
	} else {
		set_irq_type(ch7024_client->irq, IRQF_TRIGGER_RISING);
	}
	dev_dbg(&ch7024_client->dev, "detect = %d\n", detect);
	return detect;
}

static irqreturn_t hp_detect_handler(int irq, void *data)
{
	disable_irq(irq);
	schedule_delayed_work(&ch7024_wq, 50);

	return IRQ_HANDLED;
}

static void hp_detect_wq_handler(struct work_struct *work)
{
	int detect;
	struct mxc_hw_event event = { HWE_PHONEJACK_PLUG, 0 };

	detect = ch7024_detect();

	enable_irq(ch7024_client->irq);

	sysfs_notify(&ch7024_client->dev.kobj, NULL, "headphone");

	/* send hw event by netlink */
	event.args = detect;
	hw_event_send(1, &event);
}

int ch7024_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		if ((ch7024_fbi != NULL) || strcmp(fbi->fix.id, "DISP3 BG"))
			break;

		ch7024_fbi = fbi;
		fb_add_videomode(&video_modes[0], &ch7024_fbi->modelist);
		fb_add_videomode(&video_modes[1], &ch7024_fbi->modelist);
		break;
	case FB_EVENT_MODE_CHANGE:
		if (ch7024_fbi != fbi)
			break;

		if (!fbi->mode) {
			ch7024_disable();
			ch7024_cur_mode = TVOUT_FMT_OFF;
			return 0;
		}

		if (fb_mode_is_equal(fbi->mode, &video_modes[0])) {
			ch7024_cur_mode = TVOUT_FMT_NTSC;
			ch7024_enable();
			ch7024_setup(TVOUT_FMT_NTSC);
		} else if (fb_mode_is_equal(fbi->mode, &video_modes[1])) {
			ch7024_cur_mode = TVOUT_FMT_PAL;
			ch7024_enable();
			ch7024_setup(TVOUT_FMT_PAL);
		} else {
			ch7024_disable();
			ch7024_cur_mode = TVOUT_FMT_OFF;
			return 0;
		}
		break;
	case FB_EVENT_BLANK:
		if ((ch7024_fbi != fbi) || (ch7024_cur_mode == TVOUT_FMT_OFF))
			return 0;

		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			ch7024_enable();
			ch7024_setup(ch7024_cur_mode);
		} else {
			ch7024_disable();
		}
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = ch7024_fb_event,
};

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	int detect;

	detect = ch7024_detect();

	if (detect == 0) {
		strcpy(buf, "none\n");
	} else if (detect == 1) {
		strcpy(buf, "cvbs\n");
	} else {
		strcpy(buf, "headset\n");
	}

	return strlen(buf);
}

DRIVER_ATTR(headphone, 0644, show_headphone, NULL);

static ssize_t show_brightness(struct device_driver *dev, char *buf)
{
	u32 reg;
	reg = ch7024_read_reg(CH7024_TVBRI);
	return snprintf(buf, PAGE_SIZE, "%u", reg);
}

static ssize_t store_brightness(struct device_driver *dev, const char *buf,
				size_t count)
{
	char *endp;
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (brightness > 255)
		brightness = 255;

	ch7024_write_reg(CH7024_TVBRI, brightness);

	return count;
}

DRIVER_ATTR(brightness, 0644, show_brightness, store_brightness);

static ssize_t show_contrast(struct device_driver *dev, char *buf)
{
	u32 reg;
	reg = ch7024_read_reg(CH7024_TVCTA);

	reg *= 2;		/* Scale to 0 - 255 */

	return snprintf(buf, PAGE_SIZE, "%u", reg);
}

static ssize_t store_contrast(struct device_driver *dev, const char *buf,
			      size_t count)
{
	char *endp;
	int contrast = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	contrast /= 2;
	if (contrast > 127)
		contrast = 127;

	ch7024_write_reg(CH7024_TVCTA, contrast);

	return count;
}

DRIVER_ATTR(contrast, 0644, show_contrast, store_contrast);

static ssize_t show_hue(struct device_driver *dev, char *buf)
{
	u32 reg;
	reg = ch7024_read_reg(CH7024_TVHUE);

	reg *= 2;		/* Scale to 0 - 255 */

	return snprintf(buf, PAGE_SIZE, "%u", reg);
}

static ssize_t store_hue(struct device_driver *dev, const char *buf,
			 size_t count)
{
	char *endp;
	int hue = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	hue /= 2;
	if (hue > 127)
		hue = 127;

	ch7024_write_reg(CH7024_TVHUE, hue);

	return count;
}

DRIVER_ATTR(hue, 0644, show_hue, store_hue);

static ssize_t show_saturation(struct device_driver *dev, char *buf)
{
	u32 reg;
	reg = ch7024_read_reg(CH7024_TVSAT);

	reg *= 2;		/* Scale to 0 - 255 */

	return snprintf(buf, PAGE_SIZE, "%u", reg);
}

static ssize_t store_saturation(struct device_driver *dev, const char *buf,
				size_t count)
{
	char *endp;
	int saturation = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	saturation /= 2;
	if (saturation > 127)
		saturation = 127;

	ch7024_write_reg(CH7024_TVSAT, saturation);

	return count;
}

DRIVER_ATTR(saturation, 0644, show_saturation, store_saturation);

static ssize_t show_sharpness(struct device_driver *dev, char *buf)
{
	u32 reg;
	reg = ch7024_read_reg(CH7024_TVSHARP);

	reg *= 32;		/* Scale to 0 - 255 */

	return snprintf(buf, PAGE_SIZE, "%u", reg);
}

static ssize_t store_sharpness(struct device_driver *dev, const char *buf,
			       size_t count)
{
	char *endp;
	int sharpness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	sharpness /= 32;	/* Scale to 0 - 7 */
	if (sharpness > 7)
		sharpness = 7;

	ch7024_write_reg(CH7024_TVSHARP, sharpness);

	return count;
}

DRIVER_ATTR(sharpness, 0644, show_sharpness, store_sharpness);

static int ch7024_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	int ret, i;
	u32 id;
	u32 irqtype;
	struct mxc_tvout_platform_data *plat_data = client->dev.platform_data;

	ch7024_client = client;

	io_reg = regulator_get(&client->dev, plat_data->io_reg);
	core_reg = regulator_get(&client->dev, plat_data->core_reg);
	analog_reg = regulator_get(&client->dev, plat_data->analog_reg);

	regulator_enable(io_reg);
	regulator_enable(core_reg);
	regulator_enable(analog_reg);
	msleep(200);

	id = ch7024_read_reg(CH7024_DEVID);

	regulator_disable(core_reg);
	regulator_disable(io_reg);
	regulator_disable(analog_reg);

	if (id < 0 || id != CH7024_DEVICE_ID) {
		printk(KERN_ERR
		       "ch7024: TV encoder not present: id = %x\n", id);
		return -ENODEV;
	}
	printk(KERN_ERR "ch7024: TV encoder present: id = %x\n", id);

	detect_gpio = plat_data->detect_line;

	if (client->irq > 0) {
		if (ch7024_detect() == 0)
			irqtype = IRQF_TRIGGER_RISING;
		else
			irqtype = IRQF_TRIGGER_FALLING;

		ret = request_irq(client->irq, hp_detect_handler, irqtype,
				  client->name, client);
		if (ret < 0)
			goto err0;

		ret = driver_create_file(&client->driver->driver,
					 &driver_attr_headphone);
		if (ret < 0)
			goto err1;
	}

	ret = driver_create_file(&client->driver->driver,
				 &driver_attr_brightness);
	if (ret)
		goto err2;

	ret = driver_create_file(&client->driver->driver,
				 &driver_attr_contrast);
	if (ret)
		goto err3;
	ret = driver_create_file(&client->driver->driver, &driver_attr_hue);
	if (ret)
		goto err4;
	ret = driver_create_file(&client->driver->driver,
				 &driver_attr_saturation);
	if (ret)
		goto err5;
	ret = driver_create_file(&client->driver->driver,
				 &driver_attr_sharpness);
	if (ret)
		goto err6;

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0) {
			ch7024_fbi = registered_fb[i];
			break;
		}
	}
	if (ch7024_fbi != NULL) {
		fb_add_videomode(&video_modes[0], &ch7024_fbi->modelist);
		fb_add_videomode(&video_modes[1], &ch7024_fbi->modelist);
	}
	fb_register_client(&nb);

	return 0;
      err6:
	driver_remove_file(&client->driver->driver, &driver_attr_saturation);
      err5:
	driver_remove_file(&client->driver->driver, &driver_attr_hue);
      err4:
	driver_remove_file(&client->driver->driver, &driver_attr_contrast);
      err3:
	driver_remove_file(&client->driver->driver, &driver_attr_brightness);
      err2:
	driver_remove_file(&client->driver->driver, &driver_attr_headphone);
      err1:
	free_irq(client->irq, client);
      err0:
	return ret;
}

static int ch7024_remove(struct i2c_client *client)
{
	free_irq(client->irq, client);

	regulator_put(io_reg);
	regulator_put(core_reg);
	regulator_put(analog_reg);

	driver_remove_file(&client->driver->driver, &driver_attr_headphone);
	driver_remove_file(&client->driver->driver, &driver_attr_brightness);
	driver_remove_file(&client->driver->driver, &driver_attr_contrast);
	driver_remove_file(&client->driver->driver, &driver_attr_hue);
	driver_remove_file(&client->driver->driver, &driver_attr_saturation);
	driver_remove_file(&client->driver->driver, &driver_attr_sharpness);

	fb_unregister_client(&nb);

	ch7024_client = 0;

	return 0;
}

#ifdef CONFIG_PM
/*!
 * PM suspend/resume routing
 */
static int ch7024_suspend(struct i2c_client *client, pm_message_t state)
{
	pr_debug("Ch7024 suspend routing..\n");
	if (enabled) {
		ch7024_disable();
		pm_status = 1;
	} else {
		pm_status = 0;
	}
	return 0;
}

static int ch7024_resume(struct i2c_client *client)
{
	pr_debug("Ch7024 resume routing..\n");
	if (pm_status) {
		ch7024_enable();
		ch7024_setup(ch7024_cur_mode);
	}
	return 0;
}
#else
#define ch7024_suspend	NULL
#define ch7024_resume	NULL
#endif

static const struct i2c_device_id ch7024_id[] = {
	{ "ch7024", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ch7024_id);

static struct i2c_driver ch7024_driver = {
	.driver = {
		   .name = "ch7024",
		   },
	.probe = ch7024_probe,
	.remove = ch7024_remove,
	.suspend = ch7024_suspend,
	.resume = ch7024_resume,
	.id_table = ch7024_id,
};

static int __init ch7024_init(void)
{
	return i2c_add_driver(&ch7024_driver);
}

static void __exit ch7024_exit(void)
{
	i2c_del_driver(&ch7024_driver);
}

module_init(ch7024_init);
module_exit(ch7024_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CH7024 TV encoder driver");
MODULE_LICENSE("GPL");
