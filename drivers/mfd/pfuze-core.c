/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/slab.h>
#include <linux/module.h>
/*#define VERBOSE_DEBUG*/
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pfuze.h>
#include <linux/delay.h>

#define PFUZE_REG_DEVICE_ID	0
#define PFUZE_REG_REVISION_ID	3
#define PFUZE_REG_FAB_ID	4
#define PFUZE_REG_INT_STATUS0	5
#define PFUZE_REG_INT_MASK0	6
#define PFUZE_REG_INT_SENSE0	7

struct mc_pfuze {
	struct i2c_client *i2c_client;
	struct mutex lock;
	int irq;		/*irq number */
	irq_handler_t irqhandler[PFUZE_NUM_IRQ];
	void *irqdata[PFUZE_NUM_IRQ];
};

static const struct i2c_device_id pfuze_device_id[] = {
	{"pfuze100", 0},
	{},
};

enum pfuze_id {
	PFUZE_ID_PFUZE100,
	PFUZE_ID_INVALID,
};
static const char *pfuze_chipname[] = {
	[PFUZE_ID_PFUZE100] = "pfuze100",
};

void pfuze_lock(struct mc_pfuze *mc_pfuze)
{
	if (!mutex_trylock(&mc_pfuze->lock)) {
		dev_dbg(&mc_pfuze->i2c_client->dev, "wait for %s from %pf\n",
			__func__, __builtin_return_address(0));
		mutex_lock(&mc_pfuze->lock);
	}
	dev_dbg(&mc_pfuze->i2c_client->dev, "%s from %pf\n",
		__func__, __builtin_return_address(0));
}

EXPORT_SYMBOL(pfuze_lock);

void pfuze_unlock(struct mc_pfuze *mc_pfuze)
{
	dev_dbg(&mc_pfuze->i2c_client->dev, "%s from %pf\n", __func__,
		__builtin_return_address(0));
	mutex_unlock(&mc_pfuze->lock);
}

EXPORT_SYMBOL(pfuze_unlock);

int pfuze_reg_read(struct mc_pfuze *mc_pfuze, unsigned int offset,
		   unsigned char *val)
{
	unsigned char data;
	int ret, i;
	BUG_ON(!mutex_is_locked(&mc_pfuze->lock));
	if (offset > PFUZE_NUMREGS)
		return -EINVAL;
	data = (unsigned char)offset;
	for (i = 0; i < PFUZE_I2C_RETRY_TIMES; i++) {
		ret =
		    i2c_smbus_read_i2c_block_data(mc_pfuze->i2c_client, data, 1,
						  val);
		if (ret == 1)
			break;
		msleep(1);
	}
	if (ret != 1) {
		dev_err(&mc_pfuze->i2c_client->dev, "recv failed!:%i,%x\n", ret,
			*val);
		return -1;
	}

	return 0;
}

EXPORT_SYMBOL(pfuze_reg_read);

int pfuze_reg_write(struct mc_pfuze *mc_pfuze, unsigned int offset,
		    unsigned char val)
{
	unsigned char buf[2];
	int ret, i;

	buf[0] = (unsigned char)offset;
	memcpy(&buf[1], &val, 1);
	for (i = 0; i < PFUZE_I2C_RETRY_TIMES; i++) {
		ret = i2c_master_send(mc_pfuze->i2c_client, buf, 2);
		if (ret == 2)
			break;
		msleep(1);
	}
	if (ret != 2) {
		dev_err(&mc_pfuze->i2c_client->dev, "write failed!:%i\n", ret);
		return ret;
	}
	return 0;
}

EXPORT_SYMBOL(pfuze_reg_write);

int pfuze_reg_rmw(struct mc_pfuze *mc_pfuze, unsigned int offset,
		  unsigned char mask, unsigned char val)
{
	int ret;
	unsigned char valread;
	BUG_ON(val & ~mask);
	ret = pfuze_reg_read(mc_pfuze, offset, &valread);
	if (ret)
		return ret;
	valread = (valread & ~mask) | val;
	return pfuze_reg_write(mc_pfuze, offset, valread);
}

EXPORT_SYMBOL(pfuze_reg_rmw);

int pfuze_irq_mask(struct mc_pfuze *mc_pfuze, int irq)
{
	int ret;
	unsigned int offmask = PFUZE_REG_INT_MASK0 + 3 * (irq / 8);
	unsigned char irqbit = 1 << (irq % 8);
	unsigned char mask;

	if (irq < 0 || irq >= PFUZE_NUM_IRQ)
		return -EINVAL;

	ret = pfuze_reg_read(mc_pfuze, offmask, &mask);
	if (ret)
		return ret;
	if (mask & irqbit)
		/* already masked */
		return 0;
	return pfuze_reg_write(mc_pfuze, offmask, mask | irqbit);
}

EXPORT_SYMBOL(pfuze_irq_mask);

int pfuze_irq_unmask(struct mc_pfuze *mc_pfuze, int irq)
{
	int ret;
	unsigned int offmask = PFUZE_REG_INT_MASK0 + 3 * (irq / 8);
	unsigned char irqbit = 1 << (irq % 8);
	unsigned char mask;

	if (irq < 0 || irq >= PFUZE_NUM_IRQ)
		return -EINVAL;

	ret = pfuze_reg_read(mc_pfuze, offmask, &mask);
	if (ret)
		return ret;
	if (!(mask & irqbit))
		/* already masked */
		return 0;
	return pfuze_reg_write(mc_pfuze, offmask, mask & ~irqbit);
}

EXPORT_SYMBOL(pfuze_irq_unmask);

int pfuze_irq_status(struct mc_pfuze *mc_pfuze, int irq, int *enabled,
		     int *pending)
{
	int ret;
	unsigned int offmask = PFUZE_REG_INT_MASK0 + 3 * (irq / 8);
	unsigned int offstat = PFUZE_REG_INT_STATUS0 + 3 * (irq / 8);
	unsigned char irqbit = 1 << (irq % 8);

	if (irq < 0 || irq >= PFUZE_NUM_IRQ)
		return -EINVAL;
	if (enabled) {
		unsigned char mask;
		ret = pfuze_reg_read(mc_pfuze, offmask, &mask);
		if (ret)
			return ret;
		*enabled = mask & irqbit;
	}
	if (pending) {
		unsigned char stat;
		ret = pfuze_reg_read(mc_pfuze, offstat, &stat);
		if (ret)
			return ret;
		*pending = stat & irqbit;
	}
	return 0;
}

EXPORT_SYMBOL(pfuze_irq_status);

int pfuze_irq_ack(struct mc_pfuze *mc_pfuze, int irq)
{
	unsigned int offstat = PFUZE_REG_INT_STATUS0 + 3 * (irq / 8);
	unsigned char val = 1 << (irq % 8);
	BUG_ON(irq < 0 || irq >= PFUZE_NUM_IRQ);
	return pfuze_reg_write(mc_pfuze, offstat, val);
}

EXPORT_SYMBOL(pfuze_irq_ack);

int pfuze_irq_request_nounmask(struct mc_pfuze *mc_pfuze, int irq,
			       irq_handler_t handler, const char *name,
			       void *dev)
{
	BUG_ON(!mutex_is_locked(&mc_pfuze->lock));
	BUG_ON(!handler);

	if (irq < 0 || irq >= PFUZE_NUM_IRQ)
		return -EINVAL;
	if (mc_pfuze->irqhandler[irq])
		return -EBUSY;
	mc_pfuze->irqhandler[irq] = handler;
	mc_pfuze->irqdata[irq] = dev;

	return 0;
}

EXPORT_SYMBOL(pfuze_irq_request_nounmask);

int pfuze_irq_request(struct mc_pfuze *mc_pfuze, int irq,
		      irq_handler_t handler, const char *name, void *dev)
{
	int ret;

	ret = pfuze_irq_request_nounmask(mc_pfuze, irq, handler, name, dev);
	if (ret)
		return ret;
	ret = pfuze_irq_unmask(mc_pfuze, irq);
	if (ret) {
		mc_pfuze->irqhandler[irq] = NULL;
		mc_pfuze->irqdata[irq] = NULL;
		return ret;
	}
	return 0;
}

EXPORT_SYMBOL(pfuze_irq_request);

int pfuze_irq_free(struct mc_pfuze *mc_pfuze, int irq, void *dev)
{
	int ret;
	BUG_ON(!mutex_is_locked(&mc_pfuze->lock));

	if (irq < 0 || irq >= PFUZE_NUM_IRQ || !mc_pfuze->irqhandler[irq] ||
	    mc_pfuze->irqdata[irq] != dev)
		return -EINVAL;

	ret = pfuze_irq_mask(mc_pfuze, irq);
	if (ret)
		return ret;
	mc_pfuze->irqhandler[irq] = NULL;
	mc_pfuze->irqdata[irq] = NULL;
	return 0;
}

EXPORT_SYMBOL(pfuze_irq_free);

static inline irqreturn_t pfuze_irqhandler(struct mc_pfuze *mc_pfuze, int irq)
{
	return mc_pfuze->irqhandler[irq] (irq, mc_pfuze->irqdata[irq]);
}

static int pfuze_irq_handle(struct mc_pfuze *mc_pfuze, unsigned int offstat,
			    unsigned int offmask, int baseirq)
{
	unsigned char stat, mask;
	int ret = pfuze_reg_read(mc_pfuze, offstat, &stat);
	int num_handled = 0;

	if (ret)
		return ret;
	ret = pfuze_reg_read(mc_pfuze, offmask, &mask);
	if (ret)
		return ret;
	while (stat & ~mask) {
		unsigned char irq = __ffs(stat & ~mask);
		stat &= ~(1 << irq);
		if (likely(mc_pfuze->irqhandler[baseirq + irq])) {
			irqreturn_t handled;
			handled = pfuze_irqhandler(mc_pfuze, baseirq + irq);
			if (handled == IRQ_HANDLED)
				num_handled++;
		} else {
			dev_err(&mc_pfuze->i2c_client->dev,
				"BUG: irq %u but no handler\n", baseirq + irq);
			mask |= 1 << irq;
			ret = pfuze_reg_write(mc_pfuze, offmask, mask);
		}
	}
	return num_handled;
}

static irqreturn_t pfuze_irq_thread(int irq, void *data)
{
	struct mc_pfuze *mc_pfuze = data;
	irqreturn_t ret;
	int i, handled = 0;

	pfuze_lock(mc_pfuze);
	for (i = 0; i < 7; i++) {
		ret = pfuze_irq_handle(mc_pfuze, PFUZE_REG_INT_STATUS0 + 3 * i,
				       PFUZE_REG_INT_MASK0 + 3 * i, 8 * i);
		if (ret > 0)
			handled = 1;
	}
	pfuze_unlock(mc_pfuze);
	return IRQ_RETVAL(handled);
}

static int pfuze_identify(struct mc_pfuze *mc_pfuze, enum pfuze_id *id)
{
	unsigned char value;
	int ret;
	ret = pfuze_reg_read(mc_pfuze, PFUZE_REG_DEVICE_ID, &value);
	if (ret)
		return ret;
	switch (value & 0x0f) {
	case 0x0:
		*id = PFUZE_ID_PFUZE100;
		break;
	default:
		*id = PFUZE_ID_INVALID;
		dev_warn(&mc_pfuze->i2c_client->dev, "Illegal ID: %x\n", value);
		break;
	}
	ret = pfuze_reg_read(mc_pfuze, PFUZE_REG_REVISION_ID, &value);
	if (ret)
		return ret;
	dev_info(&mc_pfuze->i2c_client->dev,
		 "ID: %d,Full lay: %x ,Metal lay: %x\n",
		 *id, (value & 0xf0) >> 4, value & 0x0f);
	ret = pfuze_reg_read(mc_pfuze, PFUZE_REG_FAB_ID, &value);
	if (ret)
		return ret;
	dev_info(&mc_pfuze->i2c_client->dev, "FAB: %x ,FIN: %x\n",
		 (value & 0xc) >> 2, value & 0x3);
	return 0;
}

static const struct i2c_device_id *i2c_match_id(const struct i2c_device_id *id,
						const struct i2c_client *client)
{
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0)
			return id;
		id++;
	}
	return NULL;

}

static const char *pfuze_get_chipname(struct mc_pfuze *mc_pfuze)
{
	const struct i2c_driver *idrv =
	    to_i2c_driver(mc_pfuze->i2c_client->dev.driver);
	const struct i2c_device_id *devid =
	    i2c_match_id(idrv->id_table, mc_pfuze->i2c_client);
	if (!devid)
		return NULL;
	return pfuze_chipname[devid->driver_data];
}

unsigned int pfuze_get_flags(struct mc_pfuze *mc_pfuze)
{
	struct pfuze_platform_data *pdata =
	    dev_get_platdata(&mc_pfuze->i2c_client->dev);
	return pdata->flags;
}

EXPORT_SYMBOL(pfuze_get_flags);

static int pfuze_add_subdevice_pdata(struct mc_pfuze *mc_pfuze,
				     const char *format, void *pdata,
				     size_t pdata_size)
{
	char buf[30];
	const char *name = pfuze_get_chipname(mc_pfuze);
	struct mfd_cell cell = {
		.platform_data = pdata,
		.pdata_size = pdata_size,
	};
	if (snprintf(buf, sizeof(buf), format, name) > sizeof(buf))
		return -E2BIG;
	cell.name = kmemdup(buf, strlen(buf) + 1, GFP_KERNEL);
	if (!cell.name)
		return -ENOMEM;
	return mfd_add_devices(&mc_pfuze->i2c_client->dev, -1, &cell, 1, NULL,
			       0);
}

static ssize_t pfuze_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	unsigned char i, value;
	int offset = PFUZE_NUMREGS;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc_pfuze *mc_pfuze = i2c_get_clientdata(client);

	pfuze_lock(mc_pfuze);
	for (i = 0; i < offset; i++) {
		pfuze_reg_read(mc_pfuze, i, &value);
		pr_info("reg%03d: %02x\t\t", i, value);
	}
	pfuze_unlock(mc_pfuze);
	pr_info("\n");

	return 0;
}

static ssize_t pfuze_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t count)
{
	unsigned char reg, value;
	int ret;
	char *p;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc_pfuze *mc_pfuze = i2c_get_clientdata(client);

	reg = simple_strtoul(buf, NULL, 10);

	p = NULL;
	p = memchr(buf, ' ', count);

	if (p == NULL) {
		pfuze_lock(mc_pfuze);
		pfuze_reg_read(mc_pfuze, reg, &value);
		pfuze_unlock(mc_pfuze);
		pr_debug("reg%03d: %02x\n", reg, value);
		return count;
	}

	p += 1;

	value = simple_strtoul(p, NULL, 16);
	pfuze_lock(mc_pfuze);
	ret = pfuze_reg_write(mc_pfuze, reg, value);
	if (ret == 0)
		pr_debug("write reg%03d: %02x\n", reg, value);
	else
		pr_debug("register update failed\n");

	pfuze_unlock(mc_pfuze);
	return count;
}

static struct device_attribute pfuze_dev_attr = {
	.attr = {
		 .name = "pfuze_ctl",
		 .mode = S_IRUSR | S_IWUSR,
		 },
	.show = pfuze_show,
	.store = pfuze_store,
};

static int pfuze_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct mc_pfuze *mc_pfuze;
	struct pfuze_platform_data *pdata = client->dev.platform_data;
	int ret, i;
	enum pfuze_id pfuze_id;
	mc_pfuze = kzalloc(sizeof(*mc_pfuze), GFP_KERNEL);
	if (!mc_pfuze)
		return -ENOMEM;
	i2c_set_clientdata(client, mc_pfuze);
	mc_pfuze->i2c_client = client;

	mutex_init(&mc_pfuze->lock);
	pfuze_lock(mc_pfuze);
	ret = pfuze_identify(mc_pfuze, &pfuze_id);
	if (ret || pfuze_id == PFUZE_ID_INVALID)
		goto err_revision;
	/* mask all of irqs */
	for (i = 0; i < 7; i++) {
		ret =
		    pfuze_reg_write(mc_pfuze, PFUZE_REG_INT_MASK0 + 3 * i,
				    0xff);
		if (ret)
			goto err_mask;
	}
	if (client->irq)
		ret = request_threaded_irq(client->irq, NULL, pfuze_irq_thread,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW, "pfuze",
					mc_pfuze);
	if (ret) {
	      err_mask:
	      err_revision:
		pfuze_unlock(mc_pfuze);
		dev_set_drvdata(&client->dev, NULL);
		kfree(mc_pfuze);
		return ret;
	}
	pfuze_unlock(mc_pfuze);
	if (pdata->flags & PFUZE_USE_REGULATOR) {
		struct pfuze_regulator_platform_data regulator_pdata = {
			.num_regulators = pdata->num_regulators,
			.regulators = pdata->regulators,
			.pfuze_init = pdata->pfuze_init,
		};
		pfuze_add_subdevice_pdata(mc_pfuze, "%s-regulator",
					  &regulator_pdata,
					  sizeof(regulator_pdata));
	}
	ret = device_create_file(&client->dev, &pfuze_dev_attr);
	if (ret)
		dev_err(&client->dev, "create device file failed!\n");
	return 0;
}

static int __devexit pfuze_remove(struct i2c_client *client)
{
	struct mc_pfuze *mc_pfuze = dev_get_drvdata(&client->dev);
	free_irq(mc_pfuze->i2c_client->irq, mc_pfuze);
	mfd_remove_devices(&client->dev);
	kfree(mc_pfuze);
	return 0;
}

static struct i2c_driver pfuze_driver = {
	.id_table = pfuze_device_id,
	.driver = {
		   .name = "mc_pfuze",
		   .owner = THIS_MODULE,
		   },
	.probe = pfuze_probe,
	.remove = __devexit_p(pfuze_remove),
};

static int __init pfuze_init(void)
{
	return i2c_add_driver(&pfuze_driver);
}

subsys_initcall(pfuze_init);

static void __exit pfuze_exit(void)
{
	i2c_del_driver(&pfuze_driver);
}

module_exit(pfuze_exit);

MODULE_DESCRIPTION("Core driver for Freescale PFUZE PMIC");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_LICENSE("GPL v2");
