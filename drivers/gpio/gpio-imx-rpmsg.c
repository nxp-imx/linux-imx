// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/imx_rpmsg.h>
#include <linux/init.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>
#include <linux/workqueue.h>

#define IMX_RPMSG_GPIO_PER_PORT	32
#define RPMSG_TIMEOUT	1000
#define IMX_RPMSG_GPIO_PORT_PER_SOC_MAX	10

enum gpio_input_trigger_type {
	GPIO_RPMSG_TRI_IGNORE,
	GPIO_RPMSG_TRI_RISING,
	GPIO_RPMSG_TRI_FALLING,
	GPIO_RPMSG_TRI_BOTH_EDGE,
	GPIO_RPMSG_TRI_LOW_LEVEL,
	GPIO_RPMSG_TRI_HIGH_LEVEL,
};

enum gpio_rpmsg_header_type {
	GPIO_RPMSG_SETUP,
	GPIO_RPMSG_REPLY,
	GPIO_RPMSG_NOTIFY,
};

enum gpio_rpmsg_header_cmd {
	GPIO_RPMSG_INPUT_INIT,
	GPIO_RPMSG_OUTPUT_INIT,
	GPIO_RPMSG_INPUT_GET,
};

struct gpio_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 pin_idx;
	u8 port_idx;
	union {
		u8 event;
		u8 retcode;
		u8 value;
	} out;
	union {
		u8 wakeup;
		u8 value;
	} in;
} __packed __aligned(8);

struct imx_rpmsg_gpio_pin {
	u8 irq_shutdown;
	u8 irq_unmask;
	u8 irq_mask;
	u32 irq_wake_enable;
	u32 irq_type;
	struct gpio_rpmsg_data msg;
};

struct imx_rpmsg_gpio_port {
	struct gpio_chip gc;
	struct irq_chip chip;
	struct imx_rpmsg_gpio_pin gpio_pins[IMX_RPMSG_GPIO_PER_PORT];
	int idx;
};

struct imx_gpio_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct gpio_rpmsg_data *notify_msg;
	struct gpio_rpmsg_data *reply_msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct imx_rpmsg_gpio_port *port_store[IMX_RPMSG_GPIO_PORT_PER_SOC_MAX];
	struct mutex lock;
};

static struct imx_gpio_rpmsg_info gpio_rpmsg;

static int gpio_send_message(struct imx_rpmsg_gpio_port *port,
			     struct gpio_rpmsg_data *msg,
			     struct imx_gpio_rpmsg_info *info)
{
	unsigned long ret;
	int err;

	if (!info->rpdev) {
		pr_debug("rpmsg io channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	cpu_latency_qos_add_request(&info->pm_qos_req,
			0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct gpio_rpmsg_data));

	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	ret = wait_for_completion_timeout(&info->cmd_complete,
				msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!ret) {
		dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
		err = -ETIMEDOUT;
		goto err_out;
	}

	if (info->reply_msg->out.retcode != 0) {
		dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
			info->reply_msg->out.retcode);
		err = -EINVAL;
		goto err_out;
	}

	/* copy the reply message */
	memcpy(&port->gpio_pins[info->reply_msg->pin_idx].msg,
	       info->reply_msg, sizeof(*info->reply_msg));

	err = 0;

err_out:
	cpu_latency_qos_remove_request(&info->pm_qos_req);

	return err;
}

static int gpio_rpmsg_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct gpio_rpmsg_data *msg = (struct gpio_rpmsg_data *)data;

	if (msg->header.type == GPIO_RPMSG_REPLY) {
		/* TBD: Add irq request_id check for A core msg */
		*gpio_rpmsg.reply_msg = *msg;
		complete(&gpio_rpmsg.cmd_complete);
	} else if (msg->header.type == GPIO_RPMSG_NOTIFY) {
		*gpio_rpmsg.notify_msg = *msg;
		generic_handle_domain_irq_safe(gpio_rpmsg.port_store[msg->port_idx]->gc.irq.domain,
					       msg->pin_idx);
	} else
		dev_err(&gpio_rpmsg.rpdev->dev, "wrong command type!\n");

	return 0;
}

static struct gpio_rpmsg_data *gpio_get_pin_msg(struct imx_rpmsg_gpio_port *port, unsigned int offset)
{
	struct gpio_rpmsg_data *msg = &port->gpio_pins[offset].msg;

	memset(msg, 0, sizeof(struct gpio_rpmsg_data));

	return msg;
};

static int imx_rpmsg_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data *msg = NULL;
	int ret;

	mutex_lock(&gpio_rpmsg.lock);

	msg = gpio_get_pin_msg(port, gpio);
	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->header.cmd = GPIO_RPMSG_INPUT_GET;
	msg->pin_idx = gpio;
	msg->port_idx = port->idx;

	ret = gpio_send_message(port, msg, &gpio_rpmsg);
	if (!ret)
		ret = !!port->gpio_pins[gpio].msg.in.value;

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_gpio_direction_input(struct gpio_chip *gc,
					  unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data *msg = NULL;
	int ret;

	mutex_lock(&gpio_rpmsg.lock);

	msg = gpio_get_pin_msg(port, gpio);
	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->header.cmd = GPIO_RPMSG_INPUT_INIT;
	msg->pin_idx = gpio;
	msg->port_idx = port->idx;

	msg->out.event = GPIO_RPMSG_TRI_IGNORE;
	msg->in.wakeup = 0;

	ret = gpio_send_message(port, msg, &gpio_rpmsg);

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static inline void imx_rpmsg_gpio_direction_output_init(struct gpio_chip *gc,
		unsigned int gpio, int val, struct gpio_rpmsg_data *msg)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);

	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->header.cmd = GPIO_RPMSG_OUTPUT_INIT;
	msg->pin_idx = gpio;
	msg->port_idx = port->idx;
	msg->out.value = val;
}

static int imx_rpmsg_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data *msg = NULL;
	int ret;

	mutex_lock(&gpio_rpmsg.lock);

	msg = gpio_get_pin_msg(port, gpio);
	imx_rpmsg_gpio_direction_output_init(gc, gpio, val, msg);
	ret = gpio_send_message(port, msg, &gpio_rpmsg);

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_gpio_direction_output(struct gpio_chip *gc,
					unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data *msg = NULL;
	int ret;

	mutex_lock(&gpio_rpmsg.lock);

	msg = gpio_get_pin_msg(port, gpio);
	imx_rpmsg_gpio_direction_output_init(gc, gpio, val, msg);
	ret = gpio_send_message(port, msg, &gpio_rpmsg);

	mutex_unlock(&gpio_rpmsg.lock);

	return ret;
}

static int imx_rpmsg_irq_set_type(struct irq_data *d, u32 type)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;
	int edge = 0;
	int ret = 0;

	/*
	* The gpio rpmsg should use handle_simple_irq() to handle edge irq.
	* Because this controller don't need to clear the irq status in the
	* irq_ack.
	*/
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_RPMSG_TRI_RISING;
		irq_set_handler_locked(d, handle_simple_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_RPMSG_TRI_FALLING;
		irq_set_handler_locked(d, handle_simple_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		edge = GPIO_RPMSG_TRI_BOTH_EDGE;
		irq_set_handler_locked(d, handle_simple_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_RPMSG_TRI_LOW_LEVEL;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_RPMSG_TRI_HIGH_LEVEL;
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		ret = -EINVAL;
		irq_set_handler_locked(d, handle_bad_irq);
		break;
	}

	port->gpio_pins[gpio_idx].irq_type = edge;
	return ret;
}

static int imx_rpmsg_irq_set_wake(struct irq_data *d, u32 enable)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	port->gpio_pins[gpio_idx].irq_wake_enable = enable;

	return 0;
}

/*
 * This function will be called at:
 *  - one interrupt setup.
 *  - the end of one interrupt happened
 * The gpio over rpmsg driver will not write the real register, so save
 * all infos before this function and then send all infos to M core in this
 * step.
 */
static void imx_rpmsg_unmask_irq(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	port->gpio_pins[gpio_idx].irq_unmask = 1;
}

static void imx_rpmsg_mask_irq(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;
	/*
	 * No need to implement the callback at A core side.
	 * M core will mask interrupt after a interrupt occurred, and then
	 * sends a notify to A core.
	 * After A core dealt with the notify, A core will send a rpmsg to
	 * M core to unmask this interrupt again.
	 */
	port->gpio_pins[gpio_idx].irq_mask = 1;
}

static void imx_rpmsg_irq_shutdown(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	u32 gpio_idx = d->hwirq;

	port->gpio_pins[gpio_idx].irq_shutdown = 1;
}

static void imx_rpmsg_irq_bus_lock(struct irq_data *d)
{
	mutex_lock(&gpio_rpmsg.lock);
}

static void imx_rpmsg_irq_bus_sync_unlock(struct irq_data *d)
{
	struct imx_rpmsg_gpio_port *port = irq_data_get_irq_chip_data(d);
	struct gpio_rpmsg_data *msg = NULL;
	u32 gpio_idx = d->hwirq;

	if (port == NULL) {
		mutex_unlock(&gpio_rpmsg.lock);
		return;
	}

	/*
	 * For mask irq, do nothing here.
	 * M core will mask interrupt after a interrupt occurred, and then
	 * sends a notify to A core.
	 * After A core dealt with the notify, A core will send a rpmsg to
	 * M core to unmask this interrupt again.
	 */

	if (port->gpio_pins[gpio_idx].irq_mask && !port->gpio_pins[gpio_idx].irq_unmask) {
		port->gpio_pins[gpio_idx].irq_mask = 0;
		mutex_unlock(&gpio_rpmsg.lock);
		return;
	}

	msg = gpio_get_pin_msg(port, gpio_idx);
	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->header.cmd = GPIO_RPMSG_INPUT_INIT;
	msg->pin_idx = gpio_idx;
	msg->port_idx = port->idx;

	if (port->gpio_pins[gpio_idx].irq_shutdown) {
		msg->out.event = GPIO_RPMSG_TRI_IGNORE;
		msg->in.wakeup = 0;
		port->gpio_pins[gpio_idx].irq_shutdown = 0;
	} else {
		msg->out.event = port->gpio_pins[gpio_idx].irq_type;
		if (!msg->out.event)
			msg->out.event = GPIO_RPMSG_TRI_IGNORE;
		if (port->gpio_pins[gpio_idx].irq_unmask) {
			msg->in.wakeup = 0;
			port->gpio_pins[gpio_idx].irq_unmask = 0;
		} else /* irq set wake */
			msg->in.wakeup = port->gpio_pins[gpio_idx].irq_wake_enable;
	}

	gpio_send_message(port, msg, &gpio_rpmsg);
	mutex_unlock(&gpio_rpmsg.lock);
}

static struct irq_chip imx_rpmsg_irq_chip = {
	.irq_mask = imx_rpmsg_mask_irq,
	.irq_unmask = imx_rpmsg_unmask_irq,
	.irq_set_wake = imx_rpmsg_irq_set_wake,
	.irq_set_type = imx_rpmsg_irq_set_type,
	.irq_shutdown = imx_rpmsg_irq_shutdown,
	.irq_bus_lock = imx_rpmsg_irq_bus_lock,
	.irq_bus_sync_unlock = imx_rpmsg_irq_bus_sync_unlock,
	.flags = IRQCHIP_IMMUTABLE,
};

static int imx_rpmsg_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_gpio_port *port;
	struct gpio_irq_chip *girq;
	struct gpio_chip *gc;
	int ret;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = of_property_read_u32(np, "port_idx", &port->idx);
	if (ret)
		return ret;

	gpio_rpmsg.port_store[port->idx] = port;

	gc = &port->gc;
	gc->owner = THIS_MODULE;
	gc->parent = dev;
	gc->fwnode = of_fwnode_handle(np);
	gc->label = kasprintf(GFP_KERNEL, "imx-rpmsg-gpio-%d", port->idx);
	gc->ngpio = IMX_RPMSG_GPIO_PER_PORT;
	gc->base = -1;

	gc->direction_input = imx_rpmsg_gpio_direction_input;
	gc->direction_output = imx_rpmsg_gpio_direction_output;
	gc->get = imx_rpmsg_gpio_get;
	gc->set = imx_rpmsg_gpio_set;

	platform_set_drvdata(pdev, port);

	girq = &gc->irq;
	gpio_irq_chip_set_chip(girq, &imx_rpmsg_irq_chip);
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->chip->name = kasprintf(GFP_KERNEL, "rpmsg-irq-port-%d", port->idx);

	return devm_gpiochip_add_data(dev, gc, port);
}

static const struct of_device_id imx_rpmsg_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-gpio" },
	{ /* sentinel */ }
};

static struct platform_driver imx_rpmsg_gpio_driver = {
	.driver	= {
		.name = "gpio-imx-rpmsg",
		.of_match_table = imx_rpmsg_gpio_dt_ids,
	},
	.probe = imx_rpmsg_gpio_probe,
};

static int gpio_rpmsg_probe(struct rpmsg_device *rpdev)
{
	gpio_rpmsg.rpdev = rpdev;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	gpio_rpmsg.reply_msg = devm_kzalloc(&rpdev->dev,
					    sizeof(struct gpio_rpmsg_data),
					    GFP_KERNEL);
	if (!gpio_rpmsg.reply_msg)
		return -ENOMEM;

	gpio_rpmsg.notify_msg = devm_kzalloc(&rpdev->dev,
					    sizeof(struct gpio_rpmsg_data),
					    GFP_KERNEL);
	if (!gpio_rpmsg.notify_msg)
		return -ENOMEM;

	init_completion(&gpio_rpmsg.cmd_complete);
	mutex_init(&gpio_rpmsg.lock);

	return platform_driver_register(&imx_rpmsg_gpio_driver);
}

static void gpio_rpmsg_remove(struct rpmsg_device *rpdev)
{
	platform_driver_unregister(&imx_rpmsg_gpio_driver);
}

static struct rpmsg_device_id gpio_rpmsg_id_table[] = {
	{ .name = "rpmsg-io-channel" },
	{},
};

static struct rpmsg_driver gpio_rpmsg_driver = {
	.drv.name	= "gpio_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= gpio_rpmsg_id_table,
	.probe		= gpio_rpmsg_probe,
	.remove		= gpio_rpmsg_remove,
	.callback	= gpio_rpmsg_cb,
};


static int __init gpio_imx_rpmsg_init(void)
{
	return register_rpmsg_driver(&gpio_rpmsg_driver);
}
device_initcall(gpio_imx_rpmsg_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX7ULP rpmsg gpio driver");
MODULE_LICENSE("GPL");
