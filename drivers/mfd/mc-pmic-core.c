/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/mfd/mc-pmic.h>
#include <linux/delay.h>

struct mc_pmic {
	struct i2c_client *i2c_client;
	struct mutex lock;
	int irq;

	irq_handler_t irqhandler[MC_PMIC_NUM_IRQ];
	void *irqdata[MC_PMIC_NUM_IRQ];
};

/*!
 * This is the enumeration of versions of PMIC
 */
enum mc_pmic_id {
	MC_PMIC_ID_MC34708,
	MC_PMIC_ID_INVALID,
};

struct mc_version_t {
	/*!
	 * PMIC version identifier.
	 */
	enum mc_pmic_id id;
	/*!
	 * Revision of the PMIC.
	 */
	int revision;
};

#define PMIC_I2C_RETRY_TIMES		10

static const struct i2c_device_id mc_pmic_device_id[] = {
	{"mc34708", 0},
	{},
};

static const char *mc_pmic_chipname[] = {
	[MC_PMIC_ID_MC34708] = "mc34708",
};

void mc_pmic_lock(struct mc_pmic *mc_pmic)
{
	if (!mutex_trylock(&mc_pmic->lock)) {
		dev_dbg(&mc_pmic->i2c_client->dev, "wait for %s from %pf\n",
			__func__, __builtin_return_address(0));

		mutex_lock(&mc_pmic->lock);
	}
	dev_dbg(&mc_pmic->i2c_client->dev, "%s from %pf\n",
		__func__, __builtin_return_address(0));
}

EXPORT_SYMBOL(mc_pmic_lock);

void mc_pmic_unlock(struct mc_pmic *mc_pmic)
{
	dev_dbg(&mc_pmic->i2c_client->dev, "%s from %pf\n",
		__func__, __builtin_return_address(0));
	mutex_unlock(&mc_pmic->lock);
}

EXPORT_SYMBOL(mc_pmic_unlock);

int
mc_pmic_i2c_24bit_read(struct i2c_client *client, unsigned int offset,
		       unsigned int *value)
{
	unsigned char buf[3];
	int ret;
	int i;

	memset(buf, 0, 3);
	for (i = 0; i < PMIC_I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_read_i2c_block_data(client, offset, 3, buf);
		if (ret == 3)
			break;
		msleep(1);
	}

	if (ret == 3) {
		*value = buf[0] << 16 | buf[1] << 8 | buf[2];
		return ret;
	} else {
		dev_err(&client->dev, "24bit read error, ret = %d\n", ret);
		return -1;	/* return -1 on failure */
	}
}

int mc_pmic_reg_read(struct mc_pmic *mc_pmic, unsigned int offset, u32 * val)
{
	BUG_ON(!mutex_is_locked(&mc_pmic->lock));

	if (offset > MC_PMIC_NUMREGS)
		return -EINVAL;

	if (mc_pmic_i2c_24bit_read(mc_pmic->i2c_client, offset, val) == -1)
		return -1;
	*val &= 0xffffff;

	dev_vdbg(&mc_pmic->i2c_client->dev, "mc_pmic read [%02d] -> 0x%06x\n",
		 offset, *val);

	return 0;
}

EXPORT_SYMBOL(mc_pmic_reg_read);
int
mc_pmic_i2c_24bit_write(struct i2c_client *client,
			unsigned int offset, unsigned int reg_val)
{
	char buf[3];
	int ret;
	int i;

	buf[0] = (reg_val >> 16) & 0xff;
	buf[1] = (reg_val >> 8) & 0xff;
	buf[2] = (reg_val) & 0xff;

	for (i = 0; i < PMIC_I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_write_i2c_block_data(client, offset, 3, buf);
		if (ret == 0)
			break;
		msleep(1);
	}
	if (i == PMIC_I2C_RETRY_TIMES)
		dev_err(&client->dev, "24bit write error, ret = %d\n", ret);

	return ret;
}

int mc_pmic_reg_write(struct mc_pmic *mc_pmic, unsigned int offset, u32 val)
{
	BUG_ON(!mutex_is_locked(&mc_pmic->lock));

	if (offset > MC_PMIC_NUMREGS)
		return -EINVAL;

	if (mc_pmic_i2c_24bit_write(mc_pmic->i2c_client, offset, val))
		return -1;
	val &= 0xffffff;

	dev_vdbg(&mc_pmic->i2c_client->dev, "mc_pmic write[%02d] -> 0x%06x\n",
		 offset, val);

	return 0;
}

EXPORT_SYMBOL(mc_pmic_reg_write);

int
mc_pmic_reg_rmw(struct mc_pmic *mc_pmic, unsigned int offset, u32 mask, u32 val)
{
	int ret;
	u32 valread;

	BUG_ON(val & ~mask);

	ret = mc_pmic_reg_read(mc_pmic, offset, &valread);
	if (ret)
		return ret;

	valread = (valread & ~mask) | val;

	return mc_pmic_reg_write(mc_pmic, offset, valread);
}

EXPORT_SYMBOL(mc_pmic_reg_rmw);

int mc_pmic_irq_mask(struct mc_pmic *mc_pmic, int irq)
{
	int ret;
	unsigned int offmask =
	    irq < 24 ? MC_PMIC_REG_INT_MASK0 : MC_PMIC_REG_INT_MASK1;
	u32 irqbit = 1 << (irq < 24 ? irq : irq - 24);
	u32 mask;

	if (irq < 0 || irq >= MC_PMIC_NUM_IRQ)
		return -EINVAL;

	ret = mc_pmic_reg_read(mc_pmic, offmask, &mask);
	if (ret)
		return ret;

	if (mask & irqbit)
		/* already masked */
		return 0;

	return mc_pmic_reg_write(mc_pmic, offmask, mask | irqbit);
}

EXPORT_SYMBOL(mc_pmic_irq_mask);

int mc_pmic_irq_unmask(struct mc_pmic *mc_pmic, int irq)
{
	int ret;
	unsigned int offmask =
	    irq < 24 ? MC_PMIC_REG_INT_MASK0 : MC_PMIC_REG_INT_MASK1;
	u32 irqbit = 1 << (irq < 24 ? irq : irq - 24);
	u32 mask;

	if (irq < 0 || irq >= MC_PMIC_NUM_IRQ)
		return -EINVAL;

	ret = mc_pmic_reg_read(mc_pmic, offmask, &mask);
	if (ret)
		return ret;

	if (!(mask & irqbit))
		/* already unmasked */
		return 0;

	return mc_pmic_reg_write(mc_pmic, offmask, mask & ~irqbit);
}

EXPORT_SYMBOL(mc_pmic_irq_unmask);

int
mc_pmic_irq_status(struct mc_pmic *mc_pmic, int irq, int *enabled, int *pending)
{
	int ret;
	unsigned int offmask =
	    irq < 24 ? MC_PMIC_REG_INT_MASK0 : MC_PMIC_REG_INT_MASK1;
	unsigned int offstat =
	    irq < 24 ? MC_PMIC_REG_INT_STATUS0 : MC_PMIC_REG_INT_STATUS1;
	u32 irqbit = 1 << (irq < 24 ? irq : irq - 24);

	if (irq < 0 || irq >= MC_PMIC_NUM_IRQ)
		return -EINVAL;

	if (enabled) {
		u32 mask;

		ret = mc_pmic_reg_read(mc_pmic, offmask, &mask);
		if (ret)
			return ret;

		*enabled = mask & irqbit;
	}

	if (pending) {
		u32 stat;

		ret = mc_pmic_reg_read(mc_pmic, offstat, &stat);
		if (ret)
			return ret;

		*pending = stat & irqbit;
	}

	return 0;
}

EXPORT_SYMBOL(mc_pmic_irq_status);

int mc_pmic_irq_ack(struct mc_pmic *mc_pmic, int irq)
{
	unsigned int offstat =
	    irq < 24 ? MC_PMIC_REG_INT_STATUS0 : MC_PMIC_REG_INT_STATUS1;
	unsigned int val = 1 << (irq < 24 ? irq : irq - 24);

	BUG_ON(irq < 0 || irq >= MC_PMIC_NUM_IRQ);

	return mc_pmic_reg_write(mc_pmic, offstat, val);
}

EXPORT_SYMBOL(mc_pmic_irq_ack);

int
mc_pmic_irq_request_nounmask(struct mc_pmic *mc_pmic, int irq,
			     irq_handler_t handler, const char *name, void *dev)
{
	BUG_ON(!mutex_is_locked(&mc_pmic->lock));
	BUG_ON(!handler);

	if (irq < 0 || irq >= MC_PMIC_NUM_IRQ)
		return -EINVAL;

	if (mc_pmic->irqhandler[irq])
		return -EBUSY;

	mc_pmic->irqhandler[irq] = handler;
	mc_pmic->irqdata[irq] = dev;

	return 0;
}

EXPORT_SYMBOL(mc_pmic_irq_request_nounmask);

int
mc_pmic_irq_request(struct mc_pmic *mc_pmic, int irq,
		    irq_handler_t handler, const char *name, void *dev)
{
	int ret;

	ret = mc_pmic_irq_request_nounmask(mc_pmic, irq, handler, name, dev);
	if (ret)
		return ret;

	ret = mc_pmic_irq_unmask(mc_pmic, irq);
	if (ret) {
		mc_pmic->irqhandler[irq] = NULL;
		mc_pmic->irqdata[irq] = NULL;
		return ret;
	}

	return 0;
}

EXPORT_SYMBOL(mc_pmic_irq_request);

int mc_pmic_irq_free(struct mc_pmic *mc_pmic, int irq, void *dev)
{
	int ret;
	BUG_ON(!mutex_is_locked(&mc_pmic->lock));

	if (irq < 0 || irq >= MC_PMIC_NUM_IRQ || !mc_pmic->irqhandler[irq] ||
	    mc_pmic->irqdata[irq] != dev)
		return -EINVAL;

	ret = mc_pmic_irq_mask(mc_pmic, irq);
	if (ret)
		return ret;

	mc_pmic->irqhandler[irq] = NULL;
	mc_pmic->irqdata[irq] = NULL;

	return 0;
}

EXPORT_SYMBOL(mc_pmic_irq_free);

static inline irqreturn_t mc_pmic_irqhandler(struct mc_pmic *mc_pmic, int irq)
{
	return mc_pmic->irqhandler[irq] (irq, mc_pmic->irqdata[irq]);
}

/*
 * returns: number of handled irqs or negative error
 * locking: holds mc_pmic->lock
 */
static int
mc_pmic_irq_handle(struct mc_pmic *mc_pmic,
		   unsigned int offstat, unsigned int offmask, int baseirq)
{
	u32 stat, mask;
	int ret = mc_pmic_reg_read(mc_pmic, offstat, &stat);
	int num_handled = 0;

	if (ret)
		return ret;

	ret = mc_pmic_reg_read(mc_pmic, offmask, &mask);
	if (ret)
		return ret;

	while (stat & ~mask) {
		int irq = __ffs(stat & ~mask);

		stat &= ~(1 << irq);

		if (likely(mc_pmic->irqhandler[baseirq + irq])) {
			irqreturn_t handled;

			handled = mc_pmic_irqhandler(mc_pmic, baseirq + irq);
			if (handled == IRQ_HANDLED)
				num_handled++;
		} else {
			dev_err(&mc_pmic->i2c_client->dev,
				"BUG: irq %u but no handler\n", baseirq + irq);

			mask |= 1 << irq;

			ret = mc_pmic_reg_write(mc_pmic, offmask, mask);
		}
	}

	return num_handled;
}

static irqreturn_t mc_pmic_irq_thread(int irq, void *data)
{
	struct mc_pmic *mc_pmic = data;
	irqreturn_t ret;
	int handled = 0;

	mc_pmic_lock(mc_pmic);

	ret = mc_pmic_irq_handle(mc_pmic, MC_PMIC_REG_INT_STATUS0,
				 MC_PMIC_REG_INT_MASK0, 0);
	if (ret > 0)
		handled = 1;

	ret = mc_pmic_irq_handle(mc_pmic, MC_PMIC_REG_INT_STATUS1,
				 MC_PMIC_REG_INT_MASK1, 24);
	if (ret > 0)
		handled = 1;

	mc_pmic_unlock(mc_pmic);

	return IRQ_RETVAL(handled);
}

#define maskval(reg, mask)	(((reg) & (mask)) >> __ffs(mask))
static int mc_pmic_identify(struct mc_pmic *mc_pmic, struct mc_version_t * ver)
{

	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;
	int ret;
	ret = mc_pmic_reg_read(mc_pmic, MC_PMIC_REG_IDENTIFICATION, &rev_id);
	if (ret) {
		dev_dbg(&mc_pmic->i2c_client->dev, "read ID error!%x\n", ret);
		return ret;
	}
	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;
	ver->id = MC_PMIC_ID_MC34708;

	ver->revision = ((rev1 * 10) + rev2);
	dev_dbg(&mc_pmic->i2c_client->dev,
		"mc_pmic Rev %d.%d FinVer %x detected\n", rev1, rev2, finid);
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

static const struct i2c_device_id *i2c_get_device_id(const struct i2c_client
						     *idev)
{
	const struct i2c_driver *idrv = to_i2c_driver(idev->dev.driver);

	return i2c_match_id(idrv->id_table, idev);
}

static const char *mc_pmic_get_chipname(struct mc_pmic *mc_pmic)
{
	const struct i2c_device_id *devid =
	    i2c_get_device_id(mc_pmic->i2c_client);

	if (!devid)
		return NULL;

	return mc_pmic_chipname[devid->driver_data];
}

int mc_pmic_get_flags(struct mc_pmic *mc_pmic)
{
	struct mc_pmic_platform_data *pdata =
	    dev_get_platdata(&mc_pmic->i2c_client->dev);

	return pdata->flags;
}

EXPORT_SYMBOL(mc_pmic_get_flags);

static int
mc_pmic_add_subdevice_pdata(struct mc_pmic *mc_pmic,
			    const char *format, void *pdata, size_t pdata_size)
{
	char buf[30];
	const char *name = mc_pmic_get_chipname(mc_pmic);

	struct mfd_cell cell = {
		.platform_data = pdata,
	};

	/* there is no asnprintf in the kernel :-( */
	if (snprintf(buf, sizeof(buf), format, name) > sizeof(buf))
		return -E2BIG;

	cell.name = kmemdup(buf, strlen(buf) + 1, GFP_KERNEL);
	if (!cell.name)
		return -ENOMEM;

	return mfd_add_devices(&mc_pmic->i2c_client->dev, -1, &cell, 1, NULL,
			       0);
}

static int mc_pmic_add_subdevice(struct mc_pmic *mc_pmic, const char *format)
{
	return mc_pmic_add_subdevice_pdata(mc_pmic, format, NULL, 0);
}

static int
mc_pmic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mc_pmic *mc_pmic;
	struct mc_version_t version;
	struct mc_pmic_platform_data *pdata = client->dev.platform_data;
	int ret;
	mc_pmic = kzalloc(sizeof(*mc_pmic), GFP_KERNEL);
	if (!mc_pmic)
		return -ENOMEM;
	i2c_set_clientdata(client, mc_pmic);
	mc_pmic->i2c_client = client;

	mutex_init(&mc_pmic->lock);
	mc_pmic_lock(mc_pmic);
	mc_pmic_identify(mc_pmic, &version);
	/* mask all irqs */
	ret = mc_pmic_reg_write(mc_pmic, MC_PMIC_REG_INT_MASK0, 0x00ffffff);
	if (ret)
		goto err_mask;

	ret = mc_pmic_reg_write(mc_pmic, MC_PMIC_REG_INT_MASK1, 0x00ffffff);
	if (ret)
		goto err_mask;

	ret = request_threaded_irq(client->irq, NULL, mc_pmic_irq_thread,
				   IRQF_ONESHOT | IRQF_TRIGGER_HIGH, "mc_pmic",
				   mc_pmic);

	if (ret) {
 err_mask:
		mc_pmic_unlock(mc_pmic);
		dev_set_drvdata(&client->dev, NULL);
		kfree(mc_pmic);
		return ret;
	}

	mc_pmic_unlock(mc_pmic);

	if (pdata->flags & MC_PMIC_USE_ADC)
		mc_pmic_add_subdevice(mc_pmic, "%s-adc");


	if (pdata->flags & MC_PMIC_USE_REGULATOR) {
		struct mc_pmic_regulator_platform_data regulator_pdata = {
			.num_regulators = pdata->num_regulators,
			.regulators = pdata->regulators,
		};

		mc_pmic_add_subdevice_pdata(mc_pmic, "%s-regulator",
					    &regulator_pdata,
					    sizeof(regulator_pdata));
	}

	if (pdata->flags & MC_PMIC_USE_RTC)
		mc_pmic_add_subdevice(mc_pmic, "%s-rtc");

	if (pdata->flags & MC_PMIC_USE_TOUCHSCREEN)
		mc_pmic_add_subdevice(mc_pmic, "%s-ts");

	return 0;
}

static int __devexit mc_pmic_remove(struct i2c_client *client)
{
	struct mc_pmic *mc_pmic = dev_get_drvdata(&client->dev);

	free_irq(mc_pmic->i2c_client->irq, mc_pmic);

	mfd_remove_devices(&client->dev);

	kfree(mc_pmic);

	return 0;
}

static struct i2c_driver mc_pmic_driver = {
	.id_table = mc_pmic_device_id,
	.driver = {
		   .name = "mc_pmic",
		   .owner = THIS_MODULE,
		   },
	.probe = mc_pmic_probe,
	.remove = __devexit_p(mc_pmic_remove),
};

static int __init mc_pmic_init(void)
{
	return i2c_add_driver(&mc_pmic_driver);
}

subsys_initcall(mc_pmic_init);

static void __exit mc_pmic_exit(void)
{
	i2c_del_driver(&mc_pmic_driver);
}

module_exit(mc_pmic_exit);

MODULE_DESCRIPTION("Core driver for Freescale MC34708 PMIC");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_LICENSE("GPL v2");
