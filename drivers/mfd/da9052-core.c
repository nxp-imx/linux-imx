/*
 * da9052-core.c  --  Device access for Dialog DA9052
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd <dchen@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/mfd/core.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/adc.h>

#define SUCCESS		0
#define FAILURE		1

struct da9052_eh_nb eve_nb_array[EVE_CNT];
static struct da9052_ssc_ops ssc_ops;
struct mutex manconv_lock;
static struct semaphore eve_nb_array_lock;

void da9052_lock(struct da9052 *da9052)
{
	mutex_lock(&da9052->ssc_lock);
}
EXPORT_SYMBOL(da9052_lock);

void da9052_unlock(struct da9052 *da9052)
{
	mutex_unlock(&da9052->ssc_lock);
}
EXPORT_SYMBOL(da9052_unlock);

int da9052_ssc_write(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg)
{
	int ret = 0;

	/* Reg address should be a valid address on PAGE0 or PAGE1 */
	if ((sscmsg->addr < DA9052_PAGE0_REG_START) ||
	(sscmsg->addr > DA9052_PAGE1_REG_END) ||
	((sscmsg->addr > DA9052_PAGE0_REG_END) &&
	(sscmsg->addr < DA9052_PAGE1_REG_START)))
		return INVALID_REGISTER;

	ret = ssc_ops.write(da9052, sscmsg);

	/* Update local cache if required */
	if (!ret) {
		/* Check if this register is Non-volatile*/
		if (da9052->ssc_cache[sscmsg->addr].type != VOLATILE) {
			/* Update value */
			da9052->ssc_cache[sscmsg->addr].val = sscmsg->data;
			/* Make this cache entry valid */
			da9052->ssc_cache[sscmsg->addr].status = VALID;
		}
	}

	return ret;
}

int da9052_ssc_read(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg)
{
	int ret = 0;

	/* Reg addr should be a valid address on PAGE0 or PAGE1 */
	if ((sscmsg->addr < DA9052_PAGE0_REG_START) ||
	(sscmsg->addr > DA9052_PAGE1_REG_END) ||
	((sscmsg->addr > DA9052_PAGE0_REG_END) &&
	(sscmsg->addr < DA9052_PAGE1_REG_START)))
		return INVALID_REGISTER;

	/*
	 * Check if this is a Non-volatile register, if yes then return value -
	 * from cache instead of actual reading from hardware. Before reading -
	 * cache entry, make sure that the entry is valid
	 */
	/* The read request is for Non-volatile register */
	/* Check if we have valid cached value for this */
	if (da9052->ssc_cache[sscmsg->addr].status == VALID) {
		/* We have valid cached value, copy this value */
		sscmsg->data = da9052->ssc_cache[sscmsg->addr].val;

		return 0;
	}

	ret = ssc_ops.read(da9052, sscmsg);

	/* Update local cache if required */
	if (!ret) {
		/* Check if this register is Non-volatile*/
		if (da9052->ssc_cache[sscmsg->addr].type != VOLATILE) {
			/* Update value */
			da9052->ssc_cache[sscmsg->addr].val = sscmsg->data;
			/* Make this cache entry valid */
			da9052->ssc_cache[sscmsg->addr].status = VALID;
		}
	}

	return ret;
}

int da9052_ssc_write_many(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg,
				int msg_no)
{
	int ret = 0;
	int cnt = 0;

	/* Check request size */
	if (msg_no > MAX_READ_WRITE_CNT)
		return -EIO;

	ret = ssc_ops.write_many(da9052, sscmsg, msg_no);
	/* Update local cache, if required */
	for (cnt = 0; cnt < msg_no; cnt++) {
		/* Check if this register is Non-volatile*/
		if (da9052->ssc_cache[sscmsg[cnt].addr].type != VOLATILE) {
			/* Update value */
			da9052->ssc_cache[sscmsg[cnt].addr].val =
			sscmsg[cnt].data;
			/* Make this cache entry valid */
			da9052->ssc_cache[sscmsg[cnt].addr].status = VALID;
		}
	}
	return ret;
}

int da9052_ssc_read_many(struct da9052 *da9052, struct da9052_ssc_msg *sscmsg,
			int msg_no)
{
	int ret = 0;
	int cnt = 0;

	/* Check request size */
	if (msg_no > MAX_READ_WRITE_CNT)
		return -EIO;

	ret = ssc_ops.read_many(da9052, sscmsg, msg_no);
	/* Update local cache, if required */
	for (cnt = 0; cnt < msg_no; cnt++) {
		/* Check if this register is Non-volatile*/
		if (da9052->ssc_cache[sscmsg[cnt].addr].type
			!= VOLATILE) {
			/* Update value */
			da9052->ssc_cache[sscmsg[cnt].addr].val =
			sscmsg[cnt].data;
			/* Make this cache entry valid */
			da9052->ssc_cache[sscmsg[cnt].addr].status = VALID;
		}
	}
	return ret;
}

static irqreturn_t da9052_eh_isr(int irq, void *dev_id)
{
	struct da9052 *da9052 = dev_id;
	/* Schedule work to be done */
	schedule_work(&da9052->eh_isr_work);
	/* Disable IRQ */
	disable_irq_nosync(DA9052_IRQ);
	return IRQ_HANDLED;
}

int eh_register_nb(struct da9052 *da9052, struct da9052_eh_nb *nb)
{

	if (nb == NULL) {
		printk(KERN_INFO "EH REGISTER FUNCTION FAILED\n");
		return -EINVAL;
	}

	if (nb->eve_type >= EVE_CNT) {
		printk(KERN_INFO "Invalid DA9052 Event Type\n");
		return -EINVAL;
	}

	/* Initialize list head inside notifier block */
	INIT_LIST_HEAD(&nb->nb_list);

	/* Acquire NB array lock */
	if (down_interruptible(&eve_nb_array_lock))
		return -EAGAIN;

	/* Add passed NB to corresponding EVENT list */
	list_add_tail(&nb->nb_list, &(eve_nb_array[nb->eve_type].nb_list));

	/* Release NB array lock */
	up(&eve_nb_array_lock);

	return 0;
}

int eh_unregister_nb(struct da9052 *da9052, struct da9052_eh_nb *nb)
{

	if (nb == NULL)
		return -EINVAL;

	/* Acquire nb array lock */
	if (down_interruptible(&eve_nb_array_lock))
		return -EAGAIN;

	/* Remove passed NB from list */
	list_del_init(&(nb->nb_list));

	/* Release NB array lock */
	up(&eve_nb_array_lock);

	return 0;
}

static int process_events(struct da9052 *da9052, int events_sts)
{

	int cnt = 0;
	int tmp_events_sts = 0;
	unsigned char event = 0;

	struct list_head *ptr;
	struct da9052_eh_nb *nb_ptr;

	/* Now we have retrieved all events, process them one by one */
	for (cnt = 0; cnt < EVE_CNT; cnt++) {
		/*
		 * Starting with highest priority event,
		 * traverse through all event
		 */
		tmp_events_sts = events_sts;

		/* Find the event associated with higher priority */
		event = cnt;

		/* Check if interrupt is received for this event */
		if (!((tmp_events_sts >> cnt) & 0x1))
			/* Event bit is not set for this event */
			/* Move to next event */
			continue;

		if (event == PEN_DOWN_EVE) {
			if (list_empty(&(eve_nb_array[event].nb_list)))
				continue;
		}

		/* Event bit is set, execute all registered call backs */
		if (down_interruptible(&eve_nb_array_lock)){
			printk(KERN_CRIT "Can't acquire eve_nb_array_lock \n");
			return -EIO;
		}

		list_for_each(ptr, &(eve_nb_array[event].nb_list)) {
			/*
			 * nb_ptr will point to the structure in which
			 * nb_list is embedded
			 */
			nb_ptr = list_entry(ptr, struct da9052_eh_nb, nb_list);
			nb_ptr->call_back(nb_ptr, events_sts);
		}
		up(&eve_nb_array_lock);
	}
	return 0;
}

void eh_workqueue_isr(struct work_struct *work)
{
	struct da9052 *da9052 =
		container_of(work, struct da9052, eh_isr_work);

	struct da9052_ssc_msg eve_data[4];
	struct da9052_ssc_msg eve_mask_data[4];
	int events_sts, ret;
	u32 mask;
	unsigned char cnt = 0;

	/* nIRQ is asserted, read event registeres to know what happened */
	events_sts = 0;
	mask = 0;

	/* Prepare ssc message to read all four event registers */
	for (cnt = 0; cnt < DA9052_EVE_REGISTERS; cnt++) {
		eve_data[cnt].addr = (DA9052_EVENTA_REG + cnt);
		eve_data[cnt].data = 0;
	}

	/* Prepare ssc message to read all four event registers */
	for (cnt = 0; cnt < DA9052_EVE_REGISTERS; cnt++) {
		eve_mask_data[cnt].addr = (DA9052_IRQMASKA_REG + cnt);
		eve_mask_data[cnt].data = 0;
	}

	/* Now read all event and mask registers */
	da9052_lock(da9052);

	ret = da9052_ssc_read_many(da9052,eve_data, DA9052_EVE_REGISTERS);
	if (ret) {
		enable_irq(DA9052_IRQ);
		da9052_unlock(da9052);
		return;
	}

	ret = da9052_ssc_read_many(da9052,eve_mask_data, DA9052_EVE_REGISTERS);
	if (ret) {
		enable_irq(DA9052_IRQ);
		da9052_unlock(da9052);
		return;
	}
	/* Collect all events */
	for (cnt = 0; cnt < DA9052_EVE_REGISTERS; cnt++)
		events_sts |= (eve_data[cnt].data << (DA9052_EVE_REGISTER_SIZE
							* cnt));
	/* Collect all mask */
	for (cnt = 0; cnt < DA9052_EVE_REGISTERS; cnt++)
		mask |= (eve_mask_data[cnt].data << (DA9052_EVE_REGISTER_SIZE
						* cnt));
	events_sts &= ~mask;
	da9052_unlock(da9052);

	/* Check if we really got any event */
	if (events_sts == 0) {
		enable_irq(DA9052_IRQ);
		da9052_unlock(da9052);
		return;
	}

	/* Process all events occurred */
	process_events(da9052, events_sts);

	da9052_lock(da9052);
	/* Now clear EVENT registers */
	for (cnt = 0; cnt < 4; cnt++) {
		if (eve_data[cnt].data) {
			ret = da9052_ssc_write(da9052, &eve_data[cnt]);
			if (ret) {
				enable_irq(DA9052_IRQ);
				da9052_unlock(da9052);
				return;
			}
		}
	}
	da9052_unlock(da9052);

	/*
	* This delay is necessary to avoid hardware fake interrupts
	* from DA9052.
	*/
#if defined  CONFIG_PMIC_DA9052 || defined  CONFIG_PMIC_DA9053AA
	udelay(50);
#endif
	/* Enable HOST interrupt */
	enable_irq(DA9052_IRQ);
}

static void da9052_eh_restore_irq(void)
{
	/* Put your platform and board specific code here */
	free_irq(DA9052_IRQ, NULL);
}

static int da9052_add_subdevice_pdata(struct da9052 *da9052,
		const char *name, void *pdata, size_t pdata_size)
{
	struct mfd_cell cell = {
		.name = name,
		.platform_data = pdata,
		.data_size = pdata_size,
	};
	return mfd_add_devices(da9052->dev, -1, &cell, 1, NULL, 0);
}

static int da9052_add_subdevice(struct da9052 *da9052, const char *name)
{
	return da9052_add_subdevice_pdata(da9052, name, NULL, 0);
}

static int add_da9052_devices(struct da9052 *da9052)
{
	s32 ret = 0;
	struct da9052_platform_data *pdata = da9052->dev->platform_data;
	struct da9052_leds_platform_data leds_data = {
			.num_leds = pdata->led_data->num_leds,
			.led = pdata->led_data->led,
	};
	struct da9052_regulator_platform_data regulator_pdata = {
		.regulators = pdata->regulators,
	};

	struct da9052_tsi_platform_data tsi_data = *(pdata->tsi_data);

	if (pdata && pdata->init) {
		ret = pdata->init(da9052);
		if (ret != 0)
			return ret;
	} else
		printk(KERN_CRIT "No platform initialisation supplied \n");

	ret = da9052_add_subdevice(da9052, "da9052-rtc");
	if (ret)
		return ret;
	ret = da9052_add_subdevice(da9052, "da9052-onkey");
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "WLED-1");
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "WLED-2");
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "WLED-3");
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "da9052-adc");
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "da9052-wdt");
	if (ret)
		return ret;

	ret = da9052_add_subdevice_pdata(da9052, "da9052-gpio",
				pdata, sizeof(*pdata));
	if (ret)
		return ret;

	ret = da9052_add_subdevice_pdata(da9052, "da9052-leds",
				&leds_data, sizeof(leds_data));
	if (ret)
		return ret;

	ret = da9052_add_subdevice_pdata(da9052, "da9052-regulator",
		&regulator_pdata, sizeof(regulator_pdata));
	if (ret)
		return ret;

	ret = da9052_add_subdevice_pdata(da9052, "da9052-tsi",
				&tsi_data, sizeof(tsi_data));
	if (ret)
		return ret;

	ret = da9052_add_subdevice(da9052, "da9052-bat");
	if (ret)
		return ret;

	return ret;
}

int da9052_ssc_init(struct da9052 *da9052)
{
	int cnt;
	struct da9052_platform_data *pdata;

	/* Initialize eve_nb_array */
	for (cnt = 0; cnt < EVE_CNT; cnt++)
		INIT_LIST_HEAD(&(eve_nb_array[cnt].nb_list));

	/* Initialize mutex required for ADC Manual read */
	mutex_init(&manconv_lock);

	/* Initialize NB array lock */
	init_MUTEX(&eve_nb_array_lock);

	/* Assign the read-write function pointers */
	da9052->read = da9052_ssc_read;
	da9052->write = da9052_ssc_write;
	da9052->read_many = da9052_ssc_read_many;
	da9052->write_many = da9052_ssc_write_many;

	if (SPI  == da9052->connecting_device && ssc_ops.write == NULL) {
		/* Assign the read/write pointers to SPI/read/write */
		ssc_ops.write = da9052_spi_write;
		ssc_ops.read = da9052_spi_read;
		ssc_ops.write_many = da9052_spi_write_many;
		ssc_ops.read_many = da9052_spi_read_many;
	}
	else if (I2C  == da9052->connecting_device && ssc_ops.write == NULL) {
		/* Assign the read/write pointers to SPI/read/write */
		ssc_ops.write = da9052_i2c_write;
		ssc_ops.read = da9052_i2c_read;
		ssc_ops.write_many = da9052_i2c_write_many;
		ssc_ops.read_many = da9052_i2c_read_many;
	} else
		return -1;
	/* Assign the EH notifier block register/de-register functions */
	da9052->register_event_notifier = eh_register_nb;
	da9052->unregister_event_notifier = eh_unregister_nb;

	/* Initialize ssc lock */
	mutex_init(&da9052->ssc_lock);

	pdata = da9052->dev->platform_data;
	add_da9052_devices(da9052);

	INIT_WORK(&da9052->eh_isr_work, eh_workqueue_isr);

	if (request_irq(DA9052_IRQ, da9052_eh_isr, IRQ_TYPE_LEVEL_LOW,
		DA9052_EH_DEVICE_NAME, da9052))
		return -EIO;

	return 0;
}

void da9052_ssc_exit(struct da9052 *da9052)
{
	printk(KERN_INFO "DA9052: Unregistering SSC device.\n");
	mutex_destroy(&manconv_lock);
	/* Restore IRQ line */
	da9052_eh_restore_irq();
	free_irq(DA9052_IRQ, NULL);
	mutex_destroy(&da9052->ssc_lock);
	mutex_destroy(&da9052->eve_nb_lock);
	return;
}

MODULE_AUTHOR("Dialog Semiconductor Ltd <dchen@diasemi.com>");
MODULE_DESCRIPTION("DA9052 MFD Core");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9052_SSC_DEVICE_NAME);
