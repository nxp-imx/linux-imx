/*
 * linux/drivers/hwmon/mma7450.c
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

/*include file*/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/fsl_devices.h>

/*macro define*/
#define MMA7450_I2C_ADDR	0x1D
#define DEVICE_NAME		"mma7450"
#define POLL_INTERVAL		100
#define DEBUG

#define INPUT_FUZZ	4
#define INPUT_FLAT	4

enum {
	REG_XOUTL = 0x00,
	REG_XOUTH,
	REG_YOUTL,
	REG_YOUTH,
	REG_ZOUTL,
	REG_ZOUTH,
	REG_XOUT8,
	REG_YOUT8,
	REG_ZOUT8,
	REG_STATUS,
	REG_DETSRC,
	REG_TOUT,
	REG_RESERVED_0,
	REG_I2CAD,
	REG_USRINF,
	REG_WHOAMI,
	REG_XOFFL,
	REG_XOFFH,
	REG_YOFFL,
	REG_YOFFH,
	REG_ZOFFL,
	REG_ZOFFH,
	REG_MCTL,
	REG_INTRST,
	REG_CTL1,
	REG_CTL2,
	REG_LDTH,
	REG_PDTH,
	REG_PD,
	REG_LT,
	REG_TW,
	REG_REVERVED_1,
};

enum {
	MOD_STANDBY = 0,
	MOD_MEASURE,
	MOD_LEVEL_D,
	MOD_PULSE_D,
};

enum {
	INT_1L_2P = 0,
	INT_1P_2L,
	INT_1SP_2P,
};

struct mma7450_status {
	u8 mod;
	u8 ctl1;
	u8 ctl2;
};

/*forward declear*/
static ssize_t mma7450_show(struct device *dev,
			    struct device_attribute *attr, char *buf);
static ssize_t mma7450_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count);
static int mma7450_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int mma7450_remove(struct i2c_client *client);
static int mma7450_suspend(struct i2c_client *client, pm_message_t state);
static int mma7450_resume(struct i2c_client *client);
static void mma_bh_handler(struct work_struct *work);

/*definition*/
static struct regulator *reg_dvdd_io;
static struct regulator *reg_avdd;
static struct i2c_client *mma7450_client;
static struct device *hwmon_dev;
static struct input_polled_dev *mma7450_idev;
static struct mxc_mma7450_platform_data *plat_data;
static u8 mma7450_mode;
static struct device_attribute mma7450_dev_attr = {
	.attr = {
		 .name = "mma7450_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = mma7450_show,
	.store = mma7450_store,
};

static const struct i2c_device_id mma7450_id[] = {
	{"mma7450", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mma7450_id);

static struct i2c_driver i2c_mma7450_driver = {
	.driver = {
		   .name = "mma7450",
		   },
	.probe = mma7450_probe,
	.remove = mma7450_remove,
	.suspend = mma7450_suspend,
	.resume = mma7450_resume,
	.id_table = mma7450_id,
};

static struct mma7450_status mma_status = {
	.mod = 0,
	.ctl1 = 0,
	.ctl2 = 0,
};

DECLARE_WORK(mma_work, mma_bh_handler);

#ifdef DEBUG
enum {
	MMA_REG_R = 0,
	MMA_REG_W,
	MMA_SET_MOD,
	MMA_SET_L_THR,
	MMA_SET_P_THR,
	MMA_SET_INTP,
	MMA_SET_INTB,
	MMA_SET_G,
	MMA_I2C_EABLE,
	MMA_OFF_X,
	MMA_OFF_Y,
	MMA_OFF_Z,
	MMA_SELF_TEST,
	MMA_SET_LDPL,
	MMA_SET_PDPL,
	MMA_SET_PDV,
	MMA_SET_LTV,
	MMA_SET_TW,
	MMA_CMD_MAX
};

static char *command[MMA_CMD_MAX] = {
	[MMA_REG_R] = "readreg",
	[MMA_REG_W] = "writereg",
	[MMA_SET_MOD] = "setmod",
	[MMA_SET_L_THR] = "setlt",
	[MMA_SET_P_THR] = "setpt",
	[MMA_SET_INTP] = "setintp",
	[MMA_SET_INTB] = "setintb",
	[MMA_SET_G] = "setg",
	[MMA_I2C_EABLE] = "setie",
	[MMA_OFF_X] = "setxo",
	[MMA_OFF_Y] = "setyo",
	[MMA_OFF_Z] = "setzo",
	[MMA_SELF_TEST] = "selft",
	[MMA_SET_LDPL] = "setldp",
	[MMA_SET_PDPL] = "setpdp",
	[MMA_SET_PDV] = "setpdv",
	[MMA_SET_LTV] = "setltv",
	[MMA_SET_TW] = "settw",
};

static void set_mod(u8 mode)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_MCTL);
	/* shall I test the ret value? */
	ret = (ret & ~0x3) | (mode & 0x3);
	mma_status.mod = ret;
	i2c_smbus_write_byte_data(mma7450_client, REG_MCTL, ret);
}

static void set_level_thr(u8 lth)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_LDTH, lth);
}

static void set_pulse_thr(u8 pth)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_PDTH, pth);
}

static void set_int_pin(u8 pin)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_CTL1);
	ret = (ret & ~0x1) | (pin & 0x1);
	mma_status.ctl1 = ret;
	i2c_smbus_write_byte_data(mma7450_client, REG_CTL1, ret);
}

static void set_int_bit(u8 bit)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_CTL1);
	ret = (ret & ~0x6) | ((bit << 1) & 0x6);
	mma_status.ctl1 = ret;
	i2c_smbus_write_byte_data(mma7450_client, REG_CTL1, ret);
}

static void set_g_level(u8 gl)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_MCTL);
	ret = (ret & ~0xC) | ((gl << 2) & 0xC);
	i2c_smbus_write_byte_data(mma7450_client, REG_MCTL, ret);
}

static void set_i2c_enable(u8 i2c_e)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_I2CAD);
	ret = (ret & ~0x80) | ((i2c_e << 7) & 0x80);
	i2c_smbus_write_byte_data(mma7450_client, REG_I2CAD, ret);
}

static void set_x_offset(u16 xo)
{
	u8 data;

	data = (xo & 0xFF);
	i2c_smbus_write_byte_data(mma7450_client, REG_XOFFL, data);
	data = (xo & 0xFF00) >> 8;
	i2c_smbus_write_byte_data(mma7450_client, REG_XOFFH, data);
}

static void set_y_offset(u16 yo)
{
	u8 data;

	data = (yo & 0xFF);
	i2c_smbus_write_byte_data(mma7450_client, REG_YOFFL, data);
	data = (yo & 0xFF00) >> 8;
	i2c_smbus_write_byte_data(mma7450_client, REG_YOFFH, data);
}

static void set_z_offset(u16 zo)
{
	u8 data;

	data = (zo & 0xFF);
	i2c_smbus_write_byte_data(mma7450_client, REG_ZOFFL, data);
	data = (zo & 0xFF00) >> 8;
	i2c_smbus_write_byte_data(mma7450_client, REG_ZOFFH, data);
}

static void selftest(u8 st)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_MCTL);
	ret = (ret & ~0x10) | ((st << 4) & 0x10);
	i2c_smbus_write_byte_data(mma7450_client, REG_MCTL, ret);
}

static void set_level_det_p(u8 ldp)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_CTL2);
	ret = (ret & ~0x1) | ((ldp << 0) & 0x1);
	mma_status.ctl2 = ret;
	i2c_smbus_write_byte_data(mma7450_client, REG_CTL2, ret);
}

static void set_pulse_det_p(u8 pdp)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mma7450_client, REG_CTL2);
	ret = (ret & ~0x2) | ((pdp << 1) & 0x2);
	mma_status.ctl2 = ret;
	i2c_smbus_write_byte_data(mma7450_client, REG_CTL2, ret);
}

static void set_pulse_duration(u8 pd)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_PD, pd);
}

static void set_latency_time(u8 lt)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_LT, lt);
}

static void set_time_window(u8 tw)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_TW, tw);
}

static void parse_arg(const char *arg, int *reg, int *value)
{
	const char *p;

	for (p = arg;; p++) {
		if (*p == ' ' || *p == '\0')
			break;
	}

	p++;

	*reg = simple_strtoul(arg, NULL, 16);
	*value = simple_strtoul(p, NULL, 16);
}

static void cmd_read_reg(const char *arg)
{
	int reg, value, ret;

	parse_arg(arg, &reg, &value);
	ret = i2c_smbus_read_byte_data(mma7450_client, reg);
	dev_info(&mma7450_client->dev, "read reg0x%x = %x\n", reg, ret);
}

static void cmd_write_reg(const char *arg)
{
	int reg, value, ret;

	parse_arg(arg, &reg, &value);
	ret = i2c_smbus_write_byte_data(mma7450_client, reg, value);
	dev_info(&mma7450_client->dev, "write reg result %s\n",
		 ret ? "failed" : "success");
}

static int exec_command(const char *buf, size_t count)
{
	const char *p, *s;
	const char *arg;
	int i, value = 0;

	for (p = buf;; p++) {
		if (*p == ' ' || *p == '\0' || p - buf >= count)
			break;
	}
	arg = p + 1;

	for (i = MMA_REG_R; i < MMA_CMD_MAX; i++) {
		s = command[i];
		if (s && !strncmp(buf, s, p - buf)) {
			dev_info(&mma7450_client->dev, "command %s\n", s);
			goto mma_exec_command;
		}
	}

	dev_err(&mma7450_client->dev, "command is not found\n");
	return -1;

      mma_exec_command:
	if (i != MMA_REG_R && i != MMA_REG_W)
		value = simple_strtoul(arg, NULL, 16);

	switch (i) {
	case MMA_REG_R:
		cmd_read_reg(arg);
		break;
	case MMA_REG_W:
		cmd_write_reg(arg);
		break;
	case MMA_SET_MOD:
		set_mod(value);
		break;
	case MMA_SET_L_THR:
		set_level_thr(value);
		break;
	case MMA_SET_P_THR:
		set_pulse_thr(value);
		break;
	case MMA_SET_INTP:
		set_int_pin(value);
		break;
	case MMA_SET_INTB:
		set_int_bit(value);
		break;
	case MMA_SET_G:
		set_g_level(value);
		break;
	case MMA_I2C_EABLE:
		set_i2c_enable(value);
		break;
	case MMA_OFF_X:
		set_x_offset(value);
		break;
	case MMA_OFF_Y:
		set_y_offset(value);
		break;
	case MMA_OFF_Z:
		set_z_offset(value);
		break;
	case MMA_SELF_TEST:
		selftest(value);
		break;
	case MMA_SET_LDPL:
		set_level_det_p(value);
		break;
	case MMA_SET_PDPL:
		set_pulse_det_p(value);
		break;
	case MMA_SET_PDV:
		set_pulse_duration(value);
		break;
	case MMA_SET_LTV:
		set_latency_time(value);
		break;
	case MMA_SET_TW:
		set_time_window(value);
		break;
	default:
		dev_err(&mma7450_client->dev, "command is not found\n");
		break;
	}

	return 0;
}

static ssize_t mma7450_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	int ret, reg;

	for (reg = REG_XOUTL; reg < REG_REVERVED_1; reg++) {
		ret = i2c_smbus_read_byte_data(mma7450_client, reg);
		dev_info(&mma7450_client->dev, "reg0x%02x:\t%03d\t0x%02x\n",
			 reg, (s8) ret, ret);
	}

	return 0;
}

static ssize_t mma7450_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	exec_command(buf, count);

	return count;
}

#else

static ssize_t mma7450_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t mma7450_store(struct device *dev,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{
	return count;
}

#endif

static void report_abs(void)
{
	u8 status, mod = mma_status.mod;
	s16 x, y, z;

	status = i2c_smbus_read_byte_data(mma7450_client, REG_STATUS);
	if (!(status & 0x01)) {	/* data ready in measurement mode? */
		return;
	}
	if ((mod & 0x0c) == 0) {	/* 8g range */
		x = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_XOUTL);
		x |= 0xFF00 &
		    (i2c_smbus_read_byte_data(mma7450_client, REG_XOUTH) << 8);
		y = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_YOUTL);
		y |= 0xFF00 &
		    (i2c_smbus_read_byte_data(mma7450_client, REG_YOUTH) << 8);
		z = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_ZOUTL);
		z |= 0xFF00 &
		    (i2c_smbus_read_byte_data(mma7450_client, REG_ZOUTH) << 8);
	} else {		/* 2g/4g range */
		x = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_XOUT8);
		y = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_YOUT8);
		z = 0xFF & i2c_smbus_read_byte_data(mma7450_client, REG_ZOUT8);
	}

	status = i2c_smbus_read_byte_data(mma7450_client, REG_STATUS);
	if (status & 0x02) {	/* data is overwrite */
		return;
	}

	/* convert signed 10bits to signed 16bits */
	x = (short)(x << 6) >> 6;
	y = (short)(y << 6) >> 6;
	z = (short)(z << 6) >> 6;

	input_report_abs(mma7450_idev->input, ABS_X, x);
	input_report_abs(mma7450_idev->input, ABS_Y, y);
	input_report_abs(mma7450_idev->input, ABS_Z, z);
	input_sync(mma7450_idev->input);
}

static void mma_bh_handler(struct work_struct *work)
{
}

static void mma7450_dev_poll(struct input_polled_dev *dev)
{
	report_abs();
}

static irqreturn_t mma7450_interrupt(int irq, void *dev_id)
{
	struct input_dev *input_dev = dev_id;
	u8 int_bit, int_pin;

	int_bit = mma_status.ctl1 & 0x6;
	int_pin = mma_status.ctl1 & 0x1;

	switch (mma_status.mod & 0x03) {
	case 1:
		/*only int1 report data ready int */
		if (plat_data->int1 != irq)
			goto error_bad_int;
		schedule_work(&mma_work);
		break;
	case 2:
		/* for level and pulse detection mode,
		 * choice tasklet to handle interrupt quickly.
		 * Currently, leave it doing nothing*/
		if (plat_data->int1 == irq) {
			if ((int_bit == 0) && (int_pin != 0))
				goto error_bad_int;
			if ((int_bit == 0x2) && (int_pin != 0x1))
				goto error_bad_int;
			if (int_bit == 0x4)
				goto error_bad_int;
		}
		if (plat_data->int2 == irq) {
			if ((int_bit == 0) && (int_pin != 0x1))
				goto error_bad_int;
			if ((int_bit == 0x2) && (int_pin != 0))
				goto error_bad_int;
			if (int_bit == 0x4)
				goto error_bad_int;
		}

		dev_info(&input_dev->dev, "motion detected in level mod\n");

		break;
	case 3:
		if (plat_data->int1 == irq) {
			if ((int_bit == 0) && (int_pin != 0x1))
				goto error_bad_int;
			if ((int_bit == 0x2) && (int_pin != 0))
				goto error_bad_int;
			if ((int_bit == 0x4) && (int_pin != 0x1))
				goto error_bad_int;
		}
		if (plat_data->int2 == irq) {
			if ((int_bit == 0) && (int_pin != 0))
				goto error_bad_int;
			if ((int_bit == 0x2) && (int_pin != 0x1))
				goto error_bad_int;
			if ((int_bit == 0x4) && (int_pin != 0))
				goto error_bad_int;
		}

		if (mma_status.ctl2 & 0x02)
			dev_info(&input_dev->dev,
				 "freefall detected in pulse mod\n");
		else
			dev_info(&input_dev->dev,
				 "motion detected in pulse mod\n");

		break;
	case 0:
	default:
		break;
	}
      error_bad_int:
	return IRQ_RETVAL(1);
}

static int mma7450_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct input_dev *idev;

	plat_data =
	    (struct mxc_mma7450_platform_data *)client->dev.platform_data;
	if (plat_data == NULL) {
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	}

	/*enable power supply */
	/*when to power on/off the power is to be considered later */
	/*shall I check the return value */
	reg_dvdd_io = regulator_get(&client->dev, plat_data->reg_dvdd_io);
	if (reg_dvdd_io != ERR_PTR(-ENOENT))
		regulator_enable(reg_dvdd_io);
	else
		return -EINVAL;

	reg_avdd = regulator_get(&client->dev, plat_data->reg_avdd);
	if (reg_avdd != ERR_PTR(-ENOENT))
		regulator_enable(reg_avdd);
	else {
		regulator_put(reg_dvdd_io);
		return -EINVAL;
	}

	/*bind the right device to the driver */
	ret = i2c_smbus_read_byte_data(client, REG_I2CAD);
	if (MMA7450_I2C_ADDR != (0x7F & ret)) {	/*compare the address value */
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x!\n", ret,
			MMA7450_I2C_ADDR);
		goto error_disable_power;
	}
	mma7450_client = client;

	/*interrupt register */
	/*when to register interrupt is to be considered later */

	/*create device file in sysfs as user interface */
	ret = device_create_file(&client->dev, &mma7450_dev_attr);
	if (ret) {
		dev_err(&client->dev, "create device file failed!\n");
		goto error_disable_power;
	}

	/*register to hwmon device */
	hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(hwmon_dev)) {
		dev_err(&client->dev, "hwmon register failed!\n");
		ret = PTR_ERR(hwmon_dev);
		goto error_rm_dev_file;
	}

	/*input poll device register */
	mma7450_idev = input_allocate_polled_device();
	if (!mma7450_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		ret = -ENOMEM;
		goto error_rm_hwmon_dev;
	}
	mma7450_idev->poll = mma7450_dev_poll;
	mma7450_idev->poll_interval = POLL_INTERVAL;
	idev = mma7450_idev->input;
	idev->name = DEVICE_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -512, 512, INPUT_FUZZ, INPUT_FLAT);
	ret = input_register_polled_device(mma7450_idev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_free_poll_dev;
	}

	/* configure gpio as input for interrupt monitor */
	plat_data->gpio_pin_get();

	set_irq_type(plat_data->int1, IRQF_TRIGGER_RISING);
	/* register interrupt handle */
	ret = request_irq(plat_data->int1, mma7450_interrupt,
			  IRQF_TRIGGER_RISING, DEVICE_NAME, idev);

	if (ret) {
		dev_err(&client->dev, "request_irq(%d) returned error %d\n",
			plat_data->int1, ret);
		goto error_rm_poll_dev;
	}

	set_irq_type(plat_data->int2, IRQF_TRIGGER_RISING);
	ret = request_irq(plat_data->int2, mma7450_interrupt,
			  IRQF_TRIGGER_RISING, DEVICE_NAME, idev);
	if (ret) {
		dev_err(&client->dev, "request_irq(%d) returned error %d\n",
			plat_data->int2, ret);
		goto error_free_irq1;
	}

	dev_info(&client->dev, "mma7450 device is probed successfully.\n");

	set_mod(1);
	return 0;		/*what value shall be return */

	/*error handle */
      error_free_irq1:
	free_irq(plat_data->int1, 0);
      error_rm_poll_dev:
	input_unregister_polled_device(mma7450_idev);
      error_free_poll_dev:
	input_free_polled_device(mma7450_idev);
      error_rm_hwmon_dev:
	hwmon_device_unregister(hwmon_dev);
      error_rm_dev_file:
	device_remove_file(&client->dev, &mma7450_dev_attr);
      error_disable_power:
	regulator_disable(reg_dvdd_io);	/*shall I check the return value */
	regulator_disable(reg_avdd);
	regulator_put(reg_dvdd_io);
	regulator_put(reg_avdd);

	return ret;
}

static int mma7450_remove(struct i2c_client *client)
{
	free_irq(plat_data->int2, mma7450_idev->input);
	free_irq(plat_data->int1, mma7450_idev->input);
	plat_data->gpio_pin_put();
	input_unregister_polled_device(mma7450_idev);
	input_free_polled_device(mma7450_idev);
	hwmon_device_unregister(hwmon_dev);
	device_remove_file(&client->dev, &mma7450_dev_attr);
	regulator_disable(reg_dvdd_io);	/*shall I check the return value */
	regulator_disable(reg_avdd);
	regulator_put(reg_dvdd_io);
	regulator_put(reg_avdd);
	return 0;
}

static int mma7450_suspend(struct i2c_client *client, pm_message_t state)
{
	mma7450_mode = i2c_smbus_read_byte_data(mma7450_client, REG_MCTL);
	i2c_smbus_write_byte_data(mma7450_client, REG_MCTL,
				  mma7450_mode & ~0x3);
	return 0;
}

static int mma7450_resume(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(mma7450_client, REG_MCTL, mma7450_mode);
	return 0;
}

static int __init init_mma7450(void)
{
	/*register driver */
	printk(KERN_INFO "add mma i2c driver\n");
	return i2c_add_driver(&i2c_mma7450_driver);
}

static void __exit exit_mma7450(void)
{
	printk(KERN_INFO "del mma i2c driver.\n");
	return i2c_del_driver(&i2c_mma7450_driver);
}

module_init(init_mma7450);
module_exit(exit_mma7450);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA7450 sensor driver");
MODULE_LICENSE("GPL");
