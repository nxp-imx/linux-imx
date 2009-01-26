/*
 * linux/drivers/char/mxc_si4702.c
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/regulator/regulator.h>
#include <asm/uaccess.h>
#include <linux/err.h>
#include <linux/mxc_si4702.h>

#define SI4702_DEV_NAME		"si4702"
#define DEV_MAJOR		0
#define DEV_MINOR		0
#define DEV_BASE_MINOR		0
#define DEV_MINOR_COUNT		256
#define SI4702_I2C_ADDR	0x10	/* 7bits I2C address */
#define BAND		87500	/* 87.5 MHz */
#define MAX_BAND	108000
#define SPACING		100	/* 100 KHz */
#define DELAY_WAIT	100	/* loop_counter max value */
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
#define BYTE_TO_WORD(hi, lo)	(((hi) << 8) & 0xFF00) | ((lo) & 0x00FF)

/*module_param();*/

struct si4702_info {
	int volume;
	int channel;
	int mute:1;
};

static struct regulator *reg_vio;
static struct regulator *reg_vdd;
static struct class *radio_class;
static struct device *class_dev;
static struct si4702_info dev_info;
/*by default, dev major is zero, and it's alloc dynamicaly. */
static int dev_major = DEV_MAJOR;
static int dev_minor = DEV_MINOR;
static struct cdev si4702_dev;
static int count;		/* open count */
DEFINE_SPINLOCK(count_lock);
static struct i2c_client *si4702_client;
static unsigned char reg_rw_buf[SI4702_REG_BYTE];
static int si4702_id_detect(struct i2c_client *client);
static int __devinit si4702_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit si4702_remove(struct i2c_client *client);
static int si4702_suspend(struct i2c_client *client, pm_message_t state);
static int si4702_resume(struct i2c_client *client);
static ssize_t si4702_show(struct device *dev,
			   struct device_attribute *attr, char *buf);
static ssize_t si4702_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count);
static int ioctl_si4702(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);
static int release_si4702(struct inode *inode, struct file *file);
static int open_si4702(struct inode *inode, struct file *file);

#undef DEBUG
#ifdef DEBUG
static void si4702_dump_reg(void);
#endif

static struct mxc_fm_platform_data *plat_data;

static struct device_attribute si4702_dev_attr = {
	.attr = {
		 .name = "si4702_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = si4702_show,
	.store = si4702_store,
};

static const struct i2c_device_id si4702_id[] = {
	{ "si4702", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, si4702_id);

static struct i2c_driver i2c_si4702_driver = {
	.driver = {
		   .name = "si4702",
		   },
	.probe = si4702_probe,
	.remove = si4702_remove,
	.id_table = si4702_id,
	.suspend = si4702_suspend,
	.resume = si4702_resume,
};
static struct file_operations si4702_fops = {
	.owner = THIS_MODULE,
	.open = open_si4702,
	.release = release_si4702,
	.ioctl = ioctl_si4702,
};

static int __devinit si4702_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	dev_t dev;
	struct mxc_fm_platform_data *data;

	dev_info(&client->dev, "si4702 device probe process start.\n");

	data = (struct mxc_fm_platform_data *)client->dev.platform_data;
	if (data == NULL) {
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	} else {
		plat_data = data;
	}

	/*enable power supply */
	reg_vio = regulator_get(&client->dev, data->reg_vio);
	if (reg_vio == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_enable(reg_vio);	/*shall i check the return value */
	regulator_put(reg_vio, &client->dev);

	reg_vdd = regulator_get(&client->dev, data->reg_vdd);
	if (reg_vdd == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_enable(reg_vdd);
	regulator_put(reg_vdd, &client->dev);
	/*attach client and check device id */
	if (SI4702_DEVICE_ID != si4702_id_detect(client)) {
		dev_info(&client->dev, "id wrong.\n");
		goto disable_power;
	}
	dev_info(&client->dev, "chip id %x detect.\n", SI4702_DEVICE_ID);
	si4702_client = client;

	plat_data->gpio_get();

	/*user interface begain */
	/*create device file in sysfs as a user interface,
	 * also for debug support */
	ret = device_create_file(&client->dev, &si4702_dev_attr);
	if (ret) {
		dev_err(&client->dev, "create device file failed!\n");
		goto gpio_put;	/* shall i use some meanful error code? */
	}

	/*create a char dev for application code access */
	if (dev_major) {
		ret = 0;
	} else {
		/*does this name field have any usefull meaning */
		ret = alloc_chrdev_region(&dev, DEV_BASE_MINOR,
					  DEV_MINOR_COUNT, "si4702");
		dev_major = MAJOR(dev);
		dev_minor = MINOR(dev);
	}

	if (ret < 0)
		goto device_file_remove;

	cdev_init(&si4702_dev, &si4702_fops);
	si4702_dev.owner = THIS_MODULE;

	if (cdev_add(&si4702_dev, dev, DEV_MINOR_COUNT))
		goto device_file_remove;

	/* create class and device for udev information */
	radio_class = class_create(THIS_MODULE, "radio");
	if (IS_ERR(radio_class)) {
		dev_err(&si4702_client->dev,
			"SI4702: failed to create radio class\n");
		goto char_dev_remove;
	}

	class_dev =
	    device_create(radio_class, NULL, MKDEV(dev_major, dev_minor),
			  NULL, SI4702_DEV_NAME);
	if (IS_ERR(class_dev)) {
		dev_err(&si4702_client->dev,
			"SI4702: failed to create radio class device\n");
		goto class_remove;
	}
	/*User interface end */

	return 0;
      class_remove:
	class_destroy(radio_class);
      char_dev_remove:
	cdev_del(&si4702_dev);
      device_file_remove:
	device_remove_file(&client->dev, &si4702_dev_attr);
      gpio_put:
	plat_data->gpio_put();
      disable_power:
	reg_vio = regulator_get(&client->dev, data->reg_vio);
	if (reg_vio == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_disable(reg_vio);	/*shall i check the return value */
	regulator_put(reg_vio, &client->dev);

	reg_vdd = regulator_get(&client->dev, data->reg_vdd);
	if (reg_vdd == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_disable(reg_vdd);
	regulator_put(reg_vdd, &client->dev);

	return 0;
}

static int __devexit si4702_remove(struct i2c_client *client)
{
	struct mxc_fm_platform_data *data;

	data = (struct mxc_fm_platform_data *)client->dev.platform_data;

	device_destroy(radio_class, MKDEV(dev_major, dev_minor));
	class_destroy(radio_class);
	cdev_del(&si4702_dev);
	device_remove_file(&client->dev, &si4702_dev_attr);
	plat_data->gpio_put();

	reg_vio = regulator_get(&client->dev, data->reg_vio);
	if (reg_vio == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_disable(reg_vio);	/*shall i check the return value */
	regulator_put(reg_vio, &client->dev);

	reg_vdd = regulator_get(&client->dev, data->reg_vdd);
	if (reg_vdd == ERR_PTR(-ENOENT))
		return -ENOENT;
	regulator_disable(reg_vdd);
	regulator_put(reg_vdd, &client->dev);

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

/*
 *check the si4702 spec for the read/write concequence.
 *
 *0 2       A    F0         A    F
 *-------------------------------
 *      buf:0 2      A     F
 */

#define REG_to_BUF(reg) (((reg >= 0) && (reg < SI4702_STATUSRSSI))?\
		(reg - SI4702_STATUSRSSI + SI4702_REG_NUM):\
		((reg >= SI4702_STATUSRSSI) && (reg < SI4702_REG_NUM))?\
		(reg - SI4702_STATUSRSSI) : -1)

static int si4702_read_reg(const int reg, u16 *value)
{
	int ret, index;

	if (NULL == si4702_client)
		return -1;

	index = REG_to_BUF(reg);

	if (-1 == index)
		return -1;

	ret = i2c_master_recv(si4702_client, reg_rw_buf, SI4702_REG_BYTE);

	*value = (reg_rw_buf[index * 2] << 8) & 0xFF00;
	*value |= reg_rw_buf[index * 2 + 1] & 0x00FF;

	return ret;
}

static int si4702_write_reg(const int reg, const u16 value)
{
	int index;

	if (NULL == si4702_client)
		return -1;

	index = REG_to_BUF(reg);

	if (-1 == index)
		return -1;

	reg_rw_buf[index * 2] = (value & 0xFF00) >> 8;
	reg_rw_buf[index * 2 + 1] = value & 0x00FF;

	return i2c_master_send(si4702_client,
			       &reg_rw_buf[SI4702_RW_OFFSET * 2],
			       (SI4702_STATUSRSSI - SI4702_POWERCFG) * 2);
}

static int si4702_id_detect(struct i2c_client *client)
{
	int ret, index;
	unsigned int ID = 0;

	plat_data->gpio_get();
	plat_data->reset();
	plat_data->clock_ctl(1);

	ret = i2c_master_recv(client, (char *)reg_rw_buf, SI4702_REG_BYTE);

	plat_data->gpio_put();

	if (ret < 0)
		return ret;

	index = REG_to_BUF(SI4702_DEVICEID);
	if (index < 0)
		return index;

	ID = (reg_rw_buf[index * 2] << 8) & 0xFF00;
	ID |= reg_rw_buf[index * 2 + 1] & 0x00FF;

	return ID;
}

static u8 si4702_channel_select(u32 freq)
{
	u16 loop_counter = 0;
	s16 channel;
	u16 si4702_read_data, si4702_write_data;
	u8 error_ind = 0;
	u8 si4702_channel_start_tune[] = { 0x40, 0x01, 0x80, 0x00 };
	u8 si4702_channel_stop_tune[] = { 0x40, 0x01, 0x00 };

	if (dev_info.mute) {
		/* enable mute */
		si4702_channel_start_tune[0] = 0;
		si4702_channel_stop_tune[0] = 0;
	}
	dev_info(&si4702_client->dev, "Input frequnce is %d\n", freq);
	/* convert freq to channel */
	channel = (freq - BAND) / SPACING;
	if (channel < 0 || channel > 1023) {
		dev_err(&si4702_client->dev, "Input frequnce is invalid\n");
		return -1;
	}
	/* fill channel bits */
	si4702_channel_start_tune[2] |= channel >> 8;
	si4702_channel_start_tune[3] |= channel & 0xFF;

	/* set tune bit */
	si4702_write_data =
	    BYTE_TO_WORD(si4702_channel_start_tune[0],
			 si4702_channel_start_tune[1]);
	error_ind = si4702_write_reg(SI4702_POWERCFG, si4702_write_data);

	si4702_write_data =
	    BYTE_TO_WORD(si4702_channel_start_tune[2],
			 si4702_channel_start_tune[3]);
	error_ind = si4702_write_reg(SI4702_CHANNEL, si4702_write_data);

	if (error_ind) {
		dev_err(&si4702_client->dev, "Failed to set start tune\n");
		return -1;
	}

	/* wait for STC == 1 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_read_data);

		if (error_ind) {
			dev_err(&si4702_client->dev,
				"Failed to read setted STC\n");
			return -1;
		}
		if ((si4702_read_data & 0x4000) != 0)
			break;
		/* sleep to wait */
		msleep(200);

	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&si4702_client->dev, "Can't wait for STC bit set");
		return -1;
	}
	loop_counter = 0;

	/* clear tune bit */
	error_ind = si4702_write_reg(SI4702_CHANNEL, 0);

	if (error_ind) {
		dev_err(&si4702_client->dev, "Failed to set stop tune\n");
		return -1;
	}

	/* wait for STC == 0 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_read_data);

		if (error_ind) {
			dev_err(&si4702_client->dev,
				"Failed to set read STC\n");
			return -1;
		}
		if ((si4702_read_data & 0x4000) == 0)
			break;
		/* sleep to wait */
		msleep(200);

	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&si4702_client->dev, "Can't wait for STC bit set");
		return -1;
	}
#if 1
	/* read RSSI */
	error_ind = si4702_read_reg(SI4702_READCHAN, &si4702_read_data);

	if (error_ind) {
		dev_err(&si4702_client->dev, "Failed to read RSSI\n");
		return -1;
	}

	channel = si4702_read_data & 0x03ff;
	dev_err(&si4702_client->dev, "seek finish: channel(%d)\n", channel);
#endif
	return 0;
}

static s32 si4702_channel_seek(s16 dir)
{
	u16 loop_counter = 0;
	u16 si4702_reg_data;
	u8 error_ind = 0;
	s32 seek_error = 0;
	u32 channel;

	error_ind = si4702_read_reg(SI4702_POWERCFG, &si4702_reg_data);

	if (dev_info.mute) {
		/* check disable mute */
		si4702_reg_data &= 0xBFFF;
	} else {
		si4702_reg_data |= 0x4000;
	}

	if (dir) {
		si4702_reg_data |= 0x0200;
	} else {
		si4702_reg_data &= 0xFDFF;
	}
	/* start seek */
	si4702_reg_data |= 0x4100;
	error_ind = si4702_write_reg(SI4702_POWERCFG, si4702_reg_data);

	if (error_ind) {
		dev_err(&si4702_client->dev, "Failed to set seek start bit\n");
		return -1;
	}

	/* wait STC == 1 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);
		if (error_ind) {
			dev_err(&si4702_client->dev,
				"Failed to read STC bit\n");
			return -1;
		}

		if ((si4702_reg_data & 0x4000) != 0)
			break;
		/* sleep to wait */
		msleep(50);

	} while (++loop_counter < DELAY_WAIT);

	if (loop_counter >= DELAY_WAIT) {
		dev_err(&si4702_client->dev, "Can't wait for STC bit set\n");
		return -1;
	}
	loop_counter = 0;

	/* check whether SF==1 (seek failed bit) */
	if ((si4702_reg_data & 0x2000) != 0) {
		dev_err(&si4702_client->dev, "Failed to seek any channel\n");
		seek_error = -2;
	}

	/* clear seek bit */
	error_ind = si4702_read_reg(SI4702_POWERCFG, &si4702_reg_data);
	si4702_reg_data &= 0xFEFF;
	error_ind = si4702_write_reg(SI4702_POWERCFG, si4702_reg_data);

	if (error_ind) {
		dev_err(&si4702_client->dev, "Failed to stop seek\n");
		return -1;
	}
	/* wait STC == 0 */
	do {
		error_ind =
		    si4702_read_reg(SI4702_STATUSRSSI, &si4702_reg_data);

		if (error_ind) {
			dev_err(&si4702_client->dev,
				"Failed to wait STC bit to clear\n");
			return -1;
		}
		if ((si4702_reg_data & 0x4000) == 0)
			break;
		/* sleep to wait */
		msleep(50);
	} while (++loop_counter < DELAY_WAIT);

	/* check loop_counter */
	if (loop_counter >= DELAY_WAIT) {
		dev_err(&si4702_client->dev, "Can't wait for STC bit set");
		return -1;
	}

	error_ind = si4702_read_reg(SI4702_READCHAN, &si4702_reg_data);

	if (error_ind) {
		dev_err(&si4702_client->dev, "I2C simulate failed\n");
		return -1;
	}

	if (seek_error == 0) {
		channel = si4702_reg_data & 0x03ff;
		seek_error = channel * SPACING + BAND;
		dev_err(&si4702_client->dev,
			"seek finish: channel(%d), freq(%dKHz)\n", channel,
			seek_error);
	}

	return seek_error;
}

#define MORE_TIME
static int si4702_startup(void)
{
	u16 magic = 0, id;

#ifdef MORE_TIME
	si4702_read_reg(SI4702_DEVICEID, &id);
	dev_err(&si4702_client->dev, "si4702: DEVICEID: 0x%x\n", id);
#endif

	plat_data->clock_ctl(1);
	mdelay(100);

	/* disable mute, stereo, seek down, powerup */
	si4702_write_reg(SI4702_POWERCFG, 0x4001);
	mdelay(500);
	si4702_read_reg(SI4702_TEST1, &magic);
	if (magic != 0x3C04)
		dev_err(&si4702_client->dev, "magic number 0x%x.\n", magic);
	/* close tune, set channel to 0 */
	si4702_write_reg(SI4702_CHANNEL, 0);
	/* disable interrupt, disable GPIO */
	si4702_write_reg(SI4702_SYSCONFIG1, 0);
	/* seek threshold, band, space select to Europe, volume to max */
	si4702_write_reg(SI4702_SYSCONFIG2, 0x0f13);
	si4702_write_reg(SI4702_SYSCONFIG3, 0x48);

	return 0;
}

static void si4702_shutdown(void)
{
	si4702_write_reg(SI4702_POWERCFG, 0x4041);
	plat_data->clock_ctl(0);
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
	[FM_MUTEDIS] = "muted"
};

static int cmd(unsigned int index)
{
	switch (index) {
	case FM_SHUTDOWN:
		dev_err(&si4702_client->dev, "FM_SHUTDOWN\n");
		si4702_shutdown();
		break;
	case FM_STARTUP:
		dev_err(&si4702_client->dev, "FM_STARTUP\n");
		plat_data->reset();
		si4702_startup();
		break;
	case FM_RESET:
		dev_err(&si4702_client->dev, "FM_RESET\n");
		plat_data->reset();
		break;
	case FM_SEEK_DOWN:
		dev_err(&si4702_client->dev, "SEEK DOWN\n");
		si4702_channel_seek(0);
		break;
	case FM_SEEK_UP:
		dev_err(&si4702_client->dev, "SEEK UP\n");
		si4702_channel_seek(1);
		break;
	default:
		dev_err(&si4702_client->dev, "error command\n");
		break;
	}
	return 0;
}

static ssize_t si4702_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	dev_err(&si4702_client->dev, "si4702 show\n");
	return 0;
}

static ssize_t si4702_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t count)
{
	int state = 0;
	const char *const *s;
	char *p;
	int error;
	int len;

	dev_err(&si4702_client->dev, "si4702 store %d\n", count);

	p = memchr(buf, '\n', count);
	len = p ? p - buf : count;

	dev_err(&si4702_client->dev, "cmd %s\n", buf);

	for (s = &fm_control[state]; state < FM_CTL_MAX; s++, state++) {
		if (*s && !strncmp(buf, *s, len)) {
			dev_err(&si4702_client->dev, "state %d\n", state);
			break;
		}
	}
	if (state < FM_CTL_MAX && *s)
		error = cmd(state);
	else
		error = -EINVAL;
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

	dev_err(&si4702_client->dev,
		"ioctl, cmd: 0x%x, arg: 0x%lx\n", cmd, arg);

	switch (cmd) {
	case SI4702_SETVOLUME:
		/* get volume from user */
		if (copy_from_user(&volume, argp, sizeof(u8))) {

			dev_err(&si4702_client->dev,
				"ioctl, copy volume value from user failed\n");
			return -EFAULT;
		}
		dev_err(&si4702_client->dev, "volume %d\n", volume);
		/* refill the register value */
		volume &= 0x0f;
		if (dev_info.mute)
			error = si4702_write_reg(SI4702_POWERCFG, 0x0001);
		else
			error = si4702_write_reg(SI4702_POWERCFG, 0x4001);

		error = si4702_write_reg(SI4702_CHANNEL, 0);
		error = si4702_write_reg(SI4702_SYSCONFIG1, 0);
		error = si4702_write_reg(SI4702_SYSCONFIG2, 0x0f10 | volume);
		if (error) {
			dev_err(&si4702_client->dev,
				"ioctl, set volume failed\n");
			return -EFAULT;
		}
		/* renew the device info */
		dev_info.volume = volume;

		break;
	case SI4702_GETVOLUME:
		/* just copy volume value to user */
		if (copy_to_user(argp, &dev_info.volume, sizeof(unsigned int))) {
			dev_err(&si4702_client->dev,
				"ioctl, copy to user failed\n");
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
			dev_err(&si4702_client->dev,
				"ioctl, set mute failed\n");
			return -EFAULT;
		}
		break;
	case SI4702_SELECT:
		if (copy_from_user(&freq, argp, sizeof(unsigned int))) {

			dev_err(&si4702_client->dev,
				"ioctl, copy frequence from user failed\n");
			return -EFAULT;
		}
		/* check frequence */
		if (freq > MAX_BAND || freq < BAND) {
			dev_err(&si4702_client->dev,
				"the frequence select is out of band\n");
			return -EINVAL;
		}
		if (si4702_channel_select(freq)) {
			dev_err(&si4702_client->dev,
				"ioctl, failed to select channel\n");
			return -EFAULT;
		}
		break;
	case SI4702_SEEK:
		if (copy_from_user(&dir, argp, sizeof(int))) {

			dev_err(&si4702_client->dev,
				"ioctl, copy from user failed\n");
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

			dev_err(&si4702_client->dev,
				"ioctl, copy seek frequnce to user failed\n");
			return -EFAULT;
		}
		break;
	default:
		dev_err(&si4702_client->dev, "SI4702: Invalid ioctl command\n");
		return -EINVAL;

	}
	return 0;
}

static int open_si4702(struct inode *inode, struct file *file)
{
	spin_lock(&count_lock);
	if (count != 0) {
		dev_err(&si4702_client->dev, "device has been open already\n");
		spin_unlock(&count_lock);
		return -EBUSY;
	}
	count++;
	spin_unlock(&count_lock);

	/* detect headphone as RF */
	if (!1) {
		dev_err(&si4702_client->dev,
			"Headphone has not been inserted\n");
		spin_lock(&count_lock);
		count--;
		spin_unlock(&count_lock);
		return -ENODEV;
	}

	/* request and active GPIO */
	plat_data->gpio_get();
	/* reset the si4702 from it's reset pin */
	plat_data->reset();

	/* startup si4702 */
	if (si4702_startup()) {
		spin_lock(&count_lock);
		count--;
		spin_unlock(&count_lock);
		return -ENODEV;
	}

	return 0;
}

static int release_si4702(struct inode *inode, struct file *file)
{
	dev_err(&si4702_client->dev, "release\n");
	/* software shutdown */
	si4702_shutdown();
	/* inactive, free GPIO, cut power */
	plat_data->gpio_put();

	spin_lock(&count_lock);
	count--;
	spin_unlock(&count_lock);

	return 0;
}

#ifdef DEBUG
static void si4702_dump_reg()
{
	int i;

	for (i = 0; i < SI4702_REG_BYTE; i += 2)
		printk(KERN_INFO "reg[%02d] = %04x\n", i / 2,
		       ((reg_rw_buf[i] << 8) & 0xFF00) +
		       (reg_rw_buf[i + 1] & 0x00FF));
}

static void test(void)
{
	u16 device_id, power_conf;

	plat_data->gpio_get();
	plat_data->reset();

	si4702_read_reg(SI4702_DEVICEID, &device_id);
	printk(KERN_INFO "device id %x\n", device_id);
	si4702_read_reg(SI4702_POWERCFG, &power_conf);
	printk(KERN_INFO "power config %x\n", power_conf);
	si4702_dump_reg();
	si4702_write_reg(SI4702_POWERCFG, 0x01);
	si4702_dump_reg();
	si4702_read_reg(SI4702_POWERCFG, &power_conf);
	printk(KERN_INFO "power config after %x\n", power_conf);

	plat_data->gpio_put();

	si4702_dump_reg();
}
#endif				/* DEBUG */

static int __init init_si4702(void)
{
	/*add to i2c driver */
	printk(KERN_INFO "add si4702 i2c driver\n");
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
