/*
 * linux/drivers/char/mxc_si4702.c
 *
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mxc_si4702.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>

#define SI4702_DEV_NAME		"si4702"
#define DEV_MAJOR		0	/* this could be module param */
#define DEV_MINOR		0
#define DEV_BASE_MINOR		0
#define DEV_MINOR_COUNT		256
#define SI4702_I2C_ADDR	0x10	/* 7bits I2C address */
#define DELAY_WAIT	0xffff	/* loop_counter max value */
/* register define */
#define SI4702_DEVICEID		0x00
#define SI4702_CHIPID			0x01
#define SI4702_POWERCFG		0x02
#define SI4702_CHANNEL		0x03
#define SI4702_SYSCONFIG1		0x04
#define SI4702_SYSCONFIG2		0x05
#define SI4702_SYSCONFIG3		0x06
#define SI4702_TEST1			0x07
#define SI4702_TEST2			0x08
#define SI4702_B00TCONFIG		0x09
#define SI4702_STATUSRSSI		0x0A
#define SI4702_READCHAN		0x0B
#define SI4702_REG_NUM		0x10
#define SI4702_REG_BYTE		(SI4702_REG_NUM * 2)
#define SI4702_DEVICE_ID		0x1242
#define SI4702_RW_REG_NUM	(SI4702_STATUSRSSI - SI4702_POWERCFG)
#define SI4702_RW_OFFSET	\
	(SI4702_REG_NUM - SI4702_STATUSRSSI + SI4702_POWERCFG)

#define SI4702_SPACE_MASK	0x0030
#define SI4702_SPACE_200K	0x0
#define SI4702_SPACE_100K	0x10
#define SI4702_SPACE_50K	0x20

#define SI4702_BAND_MASK	0x00c0
#define SI4702_BAND_LSB		6

#define SI4702_SEEKTH_MASK	0xff00
#define SI4702_SEEKTH_LSB	8

#define SI4702_SNR_MASK		0x00f0
#define SI4702_SNR_LSB		4

#define SI4702_CNT_MASK		0x000f
#define SI4702_CNT_LSB		0

#define SI4702_VOL_MASK		0x000f
#define SI4702_VOL_LSB		0

#define SI4702_CHAN_MASK	0x03ff
#define SI4702_TUNE_BIT		0x8000
#define SI4702_STC_BIT		0x4000
#define SI4702_DMUTE_BIT	0x4000
#define SI4702_SEEKUP_BIT	0x0200
#define SI4702_SEEK_BIT		0x0100
#define SI4702_SF_BIT		0x2000
#define SI4702_ENABLE_BIT	0x0001
#define SI4702_DISABLE_BIT	0x0040

enum {
	BAND_USA = 0,
	BAND_JAP_W,
	BAND_JAP
};

struct si4702_info {
	int min_band;
	int max_band;
	int space;
	int volume;
	int channel;
	int mute;
};

struct si4702_drvdata {
	struct regulator *vio;
	struct regulator *vdd;
	struct class *radio_class;
	struct si4702_info info;
	/*by default, dev major is zero, and it's alloc dynamicaly. */
	int major;
	int minor;
	struct cdev *cdev;
	int count;		/* open count */
	struct i2c_client *client;
	unsigned char reg_rw_buf[SI4702_REG_BYTE];
	struct mxc_fm_platform_data *plat_data;
};

static struct si4702_drvdata *si4702_drvdata;

DEFINE_SPINLOCK(count_lock);

#ifdef DEBUG
static void si4702_dump_reg(void)
{
	int i, j;
	unsigned char *reg_rw_buf;

	if (NULL == si4702_drvdata)
		return;

	reg_rw_buf = si4702_drvdata->reg_rw_buf;

	for (i = 0; i < 10; i++) {
		j = i * 2 + 12;
		pr_debug("reg[%02d] = %04x\n", i,
			 ((reg_rw_buf[j] << 8) & 0xFF00) +
			 (reg_rw_buf[j + 1] & 0x00FF));
	}
	for (; i < 16; i++) {
		j = (i - 10) * 2;
		pr_debug("reg[%02d] = %04x\n", i,
			 ((reg_rw_buf[j] << 8) & 0xFF00) +
			 (reg_rw_buf[j + 1] & 0x00FF));
	}
}
#else
static void si4702_dump_reg(void)
{
}
#endif /* DEBUG */

/*
 *check the si4702 spec for the read/write concequence.
 *
 *0 2       A    F0         A    F
 *-------------------------------
 *      buf:0 2      A     F
 */
#define REG_to_BUF(reg) (((reg >= 0) && (reg < SI4702_STATUSRSSI)) ? \
		(reg - SI4702_STATUSRSSI + SI4702_REG_NUM) : \
		((reg >= SI4702_STATUSRSSI) && (reg < SI4702_REG_NUM)) ? \
		(reg - SI4702_STATUSRSSI) : -1)

static int si4702_read_reg(const int reg, u16 *value)
{
	int ret, index;
	unsigned char *reg_rw_buf;

	if (NULL == si4702_drvdata)
		return -1;

	reg_rw_buf = si4702_drvdata->reg_rw_buf;

	index = REG_to_BUF(reg);

	if (-1 == index)
		return -1;

	ret =
	    i2c_master_recv(si4702_drvdata->client, reg_rw_buf,
			    SI4702_REG_BYTE);

	*value = (reg_rw_buf[index * 2] << 8) & 0xFF00;
	*value |= reg_rw_buf[index * 2 + 1] & 0x00FF;

	return ret < 0 ? ret : 0;
}

static int si4702_write_reg(const int reg, const u16 value)
{
	int index, ret;
	unsigned char *reg_rw_buf;

	if (NULL == si4702_drvdata)
		return -1;

	reg_rw_buf = si4702_drvdata->reg_rw_buf;

	index = REG_to_BUF(reg);

	if (-1 == index)
		return -1;

	reg_rw_buf[index * 2] = (value & 0xFF00) >> 8;
	reg_rw_buf[index * 2 + 1] = value & 0x00FF;

	ret = i2c_master_send(si4702_drvdata->client,
			      &reg_rw_buf[SI4702_RW_OFFSET * 2],
			      (SI4702_STATUSRSSI - SI4702_POWERCFG) * 2);
	return ret < 0 ? ret : 0;
}

static void si4702_gpio_get(void)
{
	if (NULL == si4702_drvdata)
		return;

	si4702_drvdata->plat_data->gpio_get();
}

static void si4702_gpio_put(void)
{
	if (NULL == si4702_drvdata)
		return;

	si4702_drvdata->plat_data->gpio_put();
}

static void si4702_reset(void)
{
	if (NULL == si4702_drvdata)
		return;

	si4702_drvdata->plat_data->reset();
}

static void si4702_clock_en(int flag)
{
	if (NULL == si4702_drvdata)
		return;

	si4702_drvdata->plat_data->clock_ctl(flag);
}

static int si4702_id_detect(struct i2c_client *client)
{
	int ret, index;
	unsigned int ID = 0;
	unsigned char reg_rw_buf[SI4702_REG_BYTE];

	si4702_gpio_get();
	si4702_reset();
	si4702_clock_en(1);

	ret = i2c_master_recv(client, (char *)reg_rw_buf, SI4702_REG_BYTE);

	si4702_gpio_put();

	if (ret < 0)
		return ret;

	index = REG_to_BUF(SI4702_DEVICEID);
	if (index < 0)
		return index;

	ID = (reg_rw_buf[index * 2] << 8) & 0xFF00;
	ID |= reg_rw_buf[index * 2 + 1] & 0x00FF;

	return ID;
}

/* valid args 50/100/200 */
static int si4702_set_space(int space)
{
	u16 reg;
	int ret;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	ret = si4702_read_reg(SI4702_SYSCONFIG2, &reg);
	if (ret == -1)
		return ret;

	reg &= ~SI4702_SPACE_MASK;
	switch (space) {
	case 50:
		reg |= SI4702_SPACE_50K;
		break;
	case 100:
		reg |= SI4702_SPACE_100K;
		break;
	case 200:
		ret |= SI4702_SPACE_200K;
		break;
	default:
		return -1;
	}

	ret = si4702_write_reg(SI4702_SYSCONFIG2, reg);
	if (ret == -1)
		return ret;

	info = &si4702_drvdata->info;
	info->space = space;
	return 0;
}

static int si4702_set_band_range(int band)
{
	u16 reg;
	int ret, band_min, band_max;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	switch (band) {
	case BAND_USA:
		band_min = 87500;
		band_max = 108000;
		break;
	case BAND_JAP_W:
		band_min = 76000;
		band_max = 108000;
		break;
	case BAND_JAP:
		band_min = 76000;
		band_max = 90000;
		break;
	default:
		return -1;
	}

	ret = si4702_read_reg(SI4702_SYSCONFIG2, &reg);
	if (ret == -1)
		return ret;

	reg = (reg & ~SI4702_BAND_MASK)
	    | ((band << SI4702_BAND_LSB) & SI4702_BAND_MASK);
	ret = si4702_write_reg(SI4702_SYSCONFIG2, reg);
	if (ret == -1)
		return ret;

	info = &si4702_drvdata->info;
	info->min_band = band_min;
	info->max_band = band_max;
	return 0;
}

static int si4702_set_seekth(u8 seekth)
{
	u16 reg;
	int ret;

	if (NULL == si4702_drvdata)
		return -1;

	ret = si4702_read_reg(SI4702_SYSCONFIG2, &reg);
	if (ret == -1)
		return ret;

	reg =
	    (reg & ~SI4702_SEEKTH_MASK) | ((seekth << SI4702_SEEKTH_LSB) &
					   SI4702_SEEKTH_MASK);
	ret = si4702_write_reg(SI4702_SYSCONFIG2, reg);
	if (ret == -1)
		return ret;

	return 0;
}

static int si4702_set_sksnr(u8 sksnr)
{
	u16 reg;
	int ret;

	if (NULL == si4702_drvdata)
		return -1;

	ret = si4702_read_reg(SI4702_SYSCONFIG3, &reg);
	if (ret == -1)
		return ret;

	reg =
	    (reg & ~SI4702_SNR_MASK) | ((sksnr << SI4702_SNR_LSB) &
					SI4702_SNR_MASK);
	ret = si4702_write_reg(SI4702_SYSCONFIG3, reg);
	if (ret == -1)
		return ret;

	return 0;
}

static int si4702_set_skcnt(u8 skcnt)
{
	u16 reg;
	int ret;

	if (NULL == si4702_drvdata)
		return -1;

	ret = si4702_read_reg(SI4702_SYSCONFIG3, &reg);
	if (ret == -1)
		return ret;

	reg = (reg & ~SI4702_CNT_MASK) | (skcnt & SI4702_CNT_MASK);
	ret = si4702_write_reg(SI4702_SYSCONFIG3, reg);
	if (ret == -1)
		return ret;

	return 0;
}

static int si4702_set_vol(int vol)
{
	u16 reg;
	int ret;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	ret = si4702_read_reg(SI4702_SYSCONFIG2, &reg);
	if (ret == -1)
		return ret;

	reg = (reg & ~SI4702_VOL_MASK) | (vol & SI4702_VOL_MASK);
	ret = si4702_write_reg(SI4702_SYSCONFIG2, reg);
	if (ret == -1)
		return ret;

	info = &si4702_drvdata->info;
	info->volume = vol;

	return 0;
}

static u8 si4702_channel_select(u32 freq)
{
	u16 loop_counter = 0;
	s16 channel;
	u16 si4702_reg_data;
	u8 error_ind = 0;
	struct i2c_client *client;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	info = &si4702_drvdata->info;
	client = si4702_drvdata->client;

	dev_info(&client->dev, "Input frequnce is %d\n", freq);
	if (freq < 76000 || freq > 108000) {
		dev_err(&client->dev, "Input frequnce is invalid\n");
		return -1;
	}
	/* convert freq to channel */
	channel = (freq - info->min_band) / info->space;

	si4702_reg_data = SI4702_TUNE_BIT | (channel & SI4702_CHAN_MASK);
	/* set channel */
	error_ind = si4702_write_reg(SI4702_CHANNEL, si4702_reg_data);
	if (error_ind) {
		dev_err(&client->dev, "Failed to set channel\n");
		return -1;
	}
	dev_info(&client->dev, "Set channel to %d\n", channel);

	/* wait for STC == 1 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);

		if (error_ind) {
			dev_err(&client->dev, "Failed to read setted STC\n");
			return -1;
		}
		if ((si4702_reg_data & SI4702_STC_BIT) != 0)
			break;
	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&client->dev, "Can't wait for STC bit set");
		return -1;
	}
	dev_info(&client->dev, "loop counter %d\n", loop_counter);

	loop_counter = 0;
	/* clear tune bit */
	error_ind = si4702_write_reg(SI4702_CHANNEL, 0);

	if (error_ind) {
		dev_err(&client->dev, "Failed to set stop tune\n");
		return -1;
	}

	/* wait for STC == 0 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);

		if (error_ind) {
			dev_err(&client->dev, "Failed to set read STC\n");
			return -1;
		}
		if ((si4702_reg_data & SI4702_STC_BIT) == 0)
			break;
	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&client->dev, "Can't wait for STC bit clean");
		return -1;
	}
	dev_info(&client->dev, "loop counter %d\n", loop_counter);

	/* read RSSI */
	error_ind = si4702_read_reg(SI4702_READCHAN, &si4702_reg_data);

	if (error_ind) {
		dev_err(&client->dev, "Failed to read RSSI\n");
		return -1;
	}

	channel = si4702_reg_data & SI4702_CHAN_MASK;
	dev_info(&client->dev, "seek finish: channel(%d)\n", channel);

	return 0;
}

static s32 si4702_channel_seek(s16 dir)
{
	u16 loop_counter = 0;
	u16 si4702_reg_data, reg_power_cfg;
	u8 error_ind = 0;
	u32 channel, freq;
	struct i2c_client *client;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	info = &si4702_drvdata->info;
	client = si4702_drvdata->client;

	error_ind = si4702_read_reg(SI4702_POWERCFG, &reg_power_cfg);

	if (info->mute) {
		/* check disable mute */
		reg_power_cfg &= ~SI4702_DMUTE_BIT;
	} else {
		reg_power_cfg |= SI4702_DMUTE_BIT;
	}

	if (dir) {
		reg_power_cfg |= SI4702_SEEKUP_BIT;
	} else {
		reg_power_cfg &= ~SI4702_SEEKUP_BIT;
	}
	/* start seek */
	reg_power_cfg |= SI4702_SEEK_BIT;
	error_ind = si4702_write_reg(SI4702_POWERCFG, reg_power_cfg);

	if (error_ind) {
		dev_err(&client->dev, "Failed to set seek start bit\n");
		return -1;
	}

	/* wait STC == 1 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);
		if (error_ind) {
			dev_err(&client->dev, "Failed to read STC bit\n");
			return -1;
		}

		if ((si4702_reg_data & SI4702_STC_BIT) != 0)
			break;
	} while (++loop_counter < DELAY_WAIT);

	/* clear seek bit */
	reg_power_cfg &= ~SI4702_SEEK_BIT;
	error_ind = si4702_write_reg(SI4702_POWERCFG, reg_power_cfg);
	if (error_ind) {
		dev_err(&client->dev, "Failed to stop seek\n");
		return -1;
	}

	if (loop_counter >= DELAY_WAIT) {
		dev_err(&client->dev, "Can't wait for STC bit set\n");
		return -1;
	}

	/* check whether SF==1 (seek failed bit) */
	if ((si4702_reg_data & SI4702_SF_BIT) != 0) {
		dev_err(&client->dev, "Failed to seek any channel\n");
		return -1;
	}

	loop_counter = 0;
	/* wait STC == 0 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);

		if (error_ind) {
			dev_err(&client->dev,
				"Failed to wait STC bit to clear\n");
			return -1;
		}
		if ((si4702_reg_data & SI4702_STC_BIT) == 0)
			break;
	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&client->dev, "Can't wait for STC bit clean");
		return -1;
	}

	error_ind = si4702_read_reg(SI4702_READCHAN, &si4702_reg_data);

	if (error_ind) {
		dev_err(&client->dev, "I2C simulate failed\n");
		return -1;
	}

	channel = si4702_reg_data & SI4702_CHAN_MASK;
	freq = channel * info->space + info->min_band;
	dev_err(&client->dev,
		"seek finish: channel(%d), freq(%dKHz)\n", channel, freq);

	return 0;
}

static int si4702_startup(void)
{
	u16 magic = 0, id;
	struct i2c_client *client;
	struct mxc_fm_platform_data *data;

	if (NULL == si4702_drvdata)
		return -1;

	if (si4702_drvdata->vio)
		regulator_enable(si4702_drvdata->vio);
	if (si4702_drvdata->vdd)
		regulator_enable(si4702_drvdata->vdd);
	data = si4702_drvdata->plat_data;
	client = si4702_drvdata->client;

	/* read prior to write, otherwise write op will fail */
	si4702_read_reg(SI4702_DEVICEID, &id);
	dev_err(&client->dev, "si4702: DEVICEID: 0x%x\n", id);

	si4702_clock_en(1);
	msleep(100);

	/* disable mute, stereo, seek down, powerup */
	si4702_write_reg(SI4702_POWERCFG, SI4702_DMUTE_BIT | SI4702_ENABLE_BIT);
	msleep(500);
	si4702_read_reg(SI4702_TEST1, &magic);
	if (magic != 0x3C04)
		dev_err(&client->dev, "magic number 0x%x.\n", magic);
	/* close tune, set channel to 0 */
	si4702_write_reg(SI4702_CHANNEL, 0);
	/* disable interrupt, disable GPIO */
	si4702_write_reg(SI4702_SYSCONFIG1, 0);
	/* set volume to middle level */
	si4702_set_vol(0xf);

	si4702_set_space(data->space);
	si4702_set_band_range(data->band);
	si4702_set_seekth(data->seekth);
	si4702_set_skcnt(data->skcnt);
	si4702_set_sksnr(data->sksnr);

	return 0;
}

static void si4702_shutdown(void)
{
	if (NULL == si4702_drvdata)
		return;

	si4702_write_reg(SI4702_POWERCFG, SI4702_DMUTE_BIT |
			 SI4702_ENABLE_BIT | SI4702_DISABLE_BIT);
	msleep(100);
	si4702_clock_en(0);

	if (si4702_drvdata->vdd)
		regulator_disable(si4702_drvdata->vdd);
	if (si4702_drvdata->vio)
		regulator_disable(si4702_drvdata->vio);
}

enum {
	FM_STARTUP = 0,
	FM_SHUTDOWN,
	FM_RESET,
	FM_VOLUP,
	FM_VOLDOWN,
	FM_SEEK_UP,
	FM_SEEK_DOWN,
	FM_MUTEON,
	FM_MUTEDIS,
	FM_SEL,
	FM_SEEKTH,
	FM_DL,
	FM_CTL_MAX
};

static const char *const fm_control[FM_CTL_MAX] = {
	[FM_STARTUP] = "start",
	[FM_SHUTDOWN] = "halt",
	[FM_RESET] = "reset",
	[FM_VOLUP] = "volup",
	[FM_VOLDOWN] = "voldown",
	[FM_SEEK_UP] = "seeku",
	[FM_SEEK_DOWN] = "seekd",
	[FM_MUTEON] = "mute",
	[FM_MUTEDIS] = "muted",
	[FM_SEL] = "select",
	[FM_SEEKTH] = "seekth",
	[FM_DL] = "delay"
};

static int cmd(unsigned int index, int arg)
{
	struct i2c_client *client;
	struct mxc_fm_platform_data *plat_data;

	if (NULL == si4702_drvdata)
		return -1;

	client = si4702_drvdata->client;
	plat_data = si4702_drvdata->plat_data;

	switch (index) {
	case FM_SHUTDOWN:
		dev_err(&client->dev, "FM_SHUTDOWN\n");
		si4702_shutdown();
		break;
	case FM_STARTUP:
		dev_err(&client->dev, "FM_STARTUP\n");
		si4702_reset();
		si4702_startup();
		break;
	case FM_RESET:
		dev_err(&client->dev, "FM_RESET\n");
		si4702_reset();
		break;
	case FM_SEEK_DOWN:
		dev_err(&client->dev, "SEEK DOWN\n");
		si4702_channel_seek(0);
		break;
	case FM_SEEK_UP:
		dev_err(&client->dev, "SEEK UP\n");
		si4702_channel_seek(1);
		break;
	case FM_SEL:
		dev_err(&client->dev, "select %d\n", arg * 100);
		si4702_channel_select(arg * 100);
		break;
	case FM_SEEKTH:
		dev_err(&client->dev, "seekth = %d\n", arg);
		si4702_set_seekth(arg);
		break;
	case FM_DL:
		dev_err(&client->dev, "delay = %d\n", arg);
		break;
	default:
		dev_err(&client->dev, "error command\n");
		break;
	}
	return 0;
}

static ssize_t si4702_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct si4702_drvdata *drv_data = dev_get_drvdata(dev);
	u16 device_id;

	dev_err(&(drv_data->client->dev), "si4702 show\n");
	si4702_read_reg(SI4702_DEVICEID, &device_id);
	pr_info("device id %x\n", device_id);
	si4702_dump_reg();
	return 0;
}

static ssize_t si4702_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	int state = 0;
	const char *const *s;
	char *p = NULL;
	int error;
	int len, arg = 0;
	struct si4702_drvdata *drv_data = dev_get_drvdata(dev);
	struct i2c_client *client = drv_data->client;

	dev_err(&client->dev, "si4702 store %d\n", count);

	p = memchr(buf, ' ', count);
	if (p) {
		len = p - buf;
		*p = '\0';
	} else
		len = count;

	len -= 1;
	dev_err(&client->dev, "cmd %s\n", buf);

	for (s = &fm_control[state]; state < FM_CTL_MAX; s++, state++) {
		if (*s && !strncmp(buf, *s, len)) {
			break;
		}
	}
	if (state < FM_CTL_MAX && *s) {
		if (p)
			arg = simple_strtoul(p + 1, NULL, 0);
		dev_err(&client->dev, "arg = %d\n", arg);
		error = cmd(state, arg);
	} else {
		dev_err(&client->dev, "error cmd\n");
		error = -EINVAL;
	}

	return error ? error : count;
}

static int ioctl_si4702(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int mute = 0;
	u16 data;
	int error;
	u8 volume;
	unsigned int freq;
	int dir;
	struct i2c_client *client;
	struct si4702_info *info;

	if (NULL == si4702_drvdata)
		return -1;

	info = &si4702_drvdata->info;
	client = si4702_drvdata->client;

	dev_err(&client->dev, "ioctl, cmd: 0x%x, arg: 0x%lx\n", cmd, arg);

	switch (cmd) {
	case SI4702_SETVOLUME:
		/* get volume from user */
		if (copy_from_user(&volume, argp, sizeof(u8))) {

			dev_err(&client->dev,
				"ioctl, copy volume value from user failed\n");
			return -EFAULT;
		}
		dev_err(&client->dev, "volume %d\n", volume);
		/* refill the register value */
		volume &= 0x0f;
		if (info->mute)
			error = si4702_write_reg(SI4702_POWERCFG, 0x0001);
		else
			error = si4702_write_reg(SI4702_POWERCFG, 0x4001);

		error = si4702_write_reg(SI4702_CHANNEL, 0);
		error = si4702_write_reg(SI4702_SYSCONFIG1, 0);
		error = si4702_write_reg(SI4702_SYSCONFIG2, 0x0f10 | volume);
		if (error) {
			dev_err(&client->dev, "ioctl, set volume failed\n");
			return -EFAULT;
		}
		/* renew the device info */
		info->volume = volume;

		break;
	case SI4702_GETVOLUME:
		/* just copy volume value to user */
		if (copy_to_user(argp, &(info->volume), sizeof(unsigned int))) {
			dev_err(&client->dev, "ioctl, copy to user failed\n");
			return -EFAULT;
		}
		break;
	case SI4702_MUTEON:
		mute = 1;
	case SI4702_MUTEOFF:
		if (mute) {
			/* enable mute */
			si4702_read_reg(SI4702_POWERCFG, &data);
			data &= 0x00FF;
			error = si4702_write_reg(SI4702_POWERCFG, data);
		} else {
			si4702_read_reg(SI4702_POWERCFG, &data);
			data &= 0x00FF;
			data |= 0x4000;
			error = si4702_write_reg(SI4702_POWERCFG, data);
		}
		if (error) {
			dev_err(&client->dev, "ioctl, set mute failed\n");
			return -EFAULT;
		}
		break;
	case SI4702_SELECT:
		if (copy_from_user(&freq, argp, sizeof(unsigned int))) {

			dev_err(&client->dev,
				"ioctl, copy frequence from user failed\n");
			return -EFAULT;
		}
		/* check frequence */
		if (freq > info->max_band || freq < info->min_band) {
			dev_err(&client->dev,
				"the frequence select is out of band\n");
			return -EINVAL;
		}
		if (si4702_channel_select(freq)) {
			dev_err(&client->dev,
				"ioctl, failed to select channel\n");
			return -EFAULT;
		}
		break;
	case SI4702_SEEK:
		if (copy_from_user(&dir, argp, sizeof(int))) {

			dev_err(&client->dev, "ioctl, copy from user failed\n");
			return -EFAULT;
		}
		/* get seeked channel */
		dir = si4702_channel_seek(dir);
		if (dir == -1) {
			return -EAGAIN;
		} else if (dir == -2) {
			return -EFAULT;
		}
		if (copy_to_user(argp, &dir, sizeof(int))) {

			dev_err(&client->dev,
				"ioctl, copy seek frequnce to user failed\n");
			return -EFAULT;
		}
		break;
	default:
		dev_err(&client->dev, "SI4702: Invalid ioctl command\n");
		return -EINVAL;

	}
	return 0;
}

static int open_si4702(struct inode *inode, struct file *file)
{
	struct i2c_client *client;

	if (NULL == si4702_drvdata)
		return -1;

	client = si4702_drvdata->client;

	spin_lock(&count_lock);
	if (si4702_drvdata->count != 0) {
		dev_err(&client->dev, "device has been open already\n");
		spin_unlock(&count_lock);
		return -EBUSY;
	}
	si4702_drvdata->count++;
	spin_unlock(&count_lock);

	/* request and active GPIO */
	si4702_gpio_get();
	/* reset the si4702 from it's reset pin */
	si4702_reset();

	/* startup si4702 */
	if (si4702_startup()) {
		spin_lock(&count_lock);
		si4702_drvdata->count--;
		spin_unlock(&count_lock);
		return -ENODEV;
	}

	return 0;
}

static int release_si4702(struct inode *inode, struct file *file)
{
	struct i2c_client *client;

	if (NULL == si4702_drvdata)
		return -1;

	client = si4702_drvdata->client;

	dev_err(&client->dev, "release\n");
	/* software shutdown */
	si4702_shutdown();
	/* inactive, free GPIO, cut power */
	si4702_gpio_put();

	spin_lock(&count_lock);
	si4702_drvdata->count--;
	spin_unlock(&count_lock);

	return 0;
}

static int si4702_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int si4702_resume(struct i2c_client *client)
{
	return 0;
}

static struct device_attribute si4702_dev_attr = {
	.attr = {
		 .name = "si4702_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = si4702_show,
	.store = si4702_store,
};

static struct file_operations si4702_fops = {
	.owner = THIS_MODULE,
	.open = open_si4702,
	.release = release_si4702,
	.ioctl = ioctl_si4702,
};

static int __devinit si4702_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int ret = 0;
	struct mxc_fm_platform_data *plat_data;
	struct si4702_drvdata *drv_data;
	struct device *dev;

	dev_info(&client->dev, "si4702 device probe process start.\n");

	plat_data = (struct mxc_fm_platform_data *)client->dev.platform_data;
	if (plat_data == NULL) {
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	}

	drv_data = kmalloc(sizeof(struct si4702_drvdata), GFP_KERNEL);
	if (drv_data == NULL) {
		dev_err(&client->dev, "lack of kernel memory!\n");
		return -ENOMEM;
	}
	memset(drv_data, 0, sizeof(struct si4702_drvdata));
	drv_data->plat_data = plat_data;
	drv_data->major = DEV_MAJOR;
	drv_data->minor = DEV_MINOR;
	drv_data->count = 0;

	/*enable power supply */
	if (plat_data->reg_vio != NULL) {
		drv_data->vio = regulator_get(&client->dev, plat_data->reg_vio);
		if (drv_data->vio == ERR_PTR(-ENOENT))
			goto free_drv_data;
		regulator_enable(drv_data->vio);
	}

	/* here, we assume that vio and vdd are not the same */
	if (plat_data->reg_vdd != NULL) {
		drv_data->vdd = regulator_get(&client->dev, plat_data->reg_vdd);
		if (drv_data->vdd == ERR_PTR(-ENOENT))
			goto disable_vio;
		regulator_enable(drv_data->vdd);
	}

	/*attach client and check device id */
	if (SI4702_DEVICE_ID != si4702_id_detect(client)) {
		dev_err(&client->dev, "id wrong.\n");
		goto disable_vdd;
	}
	dev_info(&client->dev, "chip id %x detect.\n", SI4702_DEVICE_ID);
	drv_data->client = client;

	/*user interface begain */
	/*create device file in sysfs as a user interface,
	 * also for debug support */
	ret = device_create_file(&client->dev, &si4702_dev_attr);
	if (ret) {
		dev_err(&client->dev, "create device file failed!\n");
		goto gpio_put;	/* shall i use some meanful error code? */
	}

	/*create a char dev for application code access */
	if (drv_data->major) {
		ret = register_chrdev(drv_data->major, "si4702", &si4702_fops);;
	} else {
		ret = register_chrdev(0, "si4702", &si4702_fops);
	}

	if (drv_data->major == 0)
		drv_data->major = ret;

	/* create class and device for udev information */
	drv_data->radio_class = class_create(THIS_MODULE, "radio");
	if (IS_ERR(drv_data->radio_class)) {
		dev_err(&client->dev, "SI4702: failed to create radio class\n");
		goto char_dev_remove;
	}

	dev = device_create(drv_data->radio_class, NULL,
			    MKDEV(drv_data->major, drv_data->minor), NULL,
			    "si4702");
	if (IS_ERR(dev)) {
		dev_err(&client->dev,
			"SI4702: failed to create radio class device\n");
		goto class_remove;
	}
	/*User interface end */
	dev_set_drvdata(&client->dev, drv_data);
	si4702_drvdata = drv_data;

	si4702_gpio_get();
	dev_info(&client->dev, "si4702 device probe successfully.\n");
	si4702_shutdown();

	return 0;

class_remove:
	class_destroy(drv_data->radio_class);
char_dev_remove:
	unregister_chrdev(drv_data->major, "si4702");
	device_remove_file(&client->dev, &si4702_dev_attr);
gpio_put:
	si4702_gpio_put();
disable_vdd:
	if (plat_data->reg_vdd) {
		regulator_disable(drv_data->vdd);
		regulator_put(drv_data->vdd);
	}
disable_vio:
	if (plat_data->reg_vio) {
		regulator_disable(drv_data->vio);
		regulator_put(drv_data->vio);
	}

free_drv_data:
	kfree(drv_data);

	return -ENODEV;
}

static int __devexit si4702_remove(struct i2c_client *client)
{
	struct mxc_fm_platform_data *plat_data;
	struct si4702_drvdata *drv_data = dev_get_drvdata(&client->dev);

	plat_data = (struct mxc_fm_platform_data *)client->dev.platform_data;

	device_destroy(drv_data->radio_class,
		       MKDEV(drv_data->major, drv_data->minor));
	class_destroy(drv_data->radio_class);

	unregister_chrdev(drv_data->major, "si4702");
	device_remove_file(&client->dev, &si4702_dev_attr);
	si4702_gpio_put();

	if (plat_data->reg_vdd)
		regulator_put(drv_data->vdd);

	if (plat_data->reg_vio)
		regulator_put(drv_data->vio);

	kfree(si4702_drvdata);
	si4702_drvdata = NULL;

	return 0;
}

static const struct i2c_device_id si4702_id[] = {
	{"si4702", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, si4702_id);

static struct i2c_driver i2c_si4702_driver = {
	.driver = {
		   .name = "si4702",
		   },
	.probe = si4702_probe,
	.remove = si4702_remove,
	.suspend = si4702_suspend,
	.resume = si4702_resume,
	.id_table = si4702_id,
};

static int __init init_si4702(void)
{
	/*add to i2c driver */
	pr_info("add si4702 i2c driver\n");
	return i2c_add_driver(&i2c_si4702_driver);
}

static void __exit exit_si4702(void)
{
	i2c_del_driver(&i2c_si4702_driver);
}

module_init(init_si4702);
module_exit(exit_si4702);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SI4702 FM driver");
MODULE_LICENSE("GPL");
