// SPDX-License-Identifier: GPL-2.0+
//
// MXC GPIO support. (c) 2008 Daniel Mack <daniel@caiaq.de>
// Copyright 2008 Juergen Beisert, kernel@pengutronix.de
//
// Based on code from Freescale Semiconductor,
// Authors: Daniel Mack, Juergen Beisert.
// Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/bug.h>
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
#include <linux/firmware/imx/sci.h>

#define IMX_SC_PAD_FUNC_GET_WAKEUP	9
#define IMX_SC_PAD_FUNC_SET_WAKEUP	4
#define IMX_SC_PAD_WAKEUP_OFF		0
#endif

enum mxc_gpio_hwtype {
	IMX1_GPIO,	/* runs on i.mx1 */
	IMX21_GPIO,	/* runs on i.mx21 and i.mx27 */
	IMX31_GPIO,	/* runs on i.mx31 */
	IMX35_GPIO,	/* runs on all other i.mx */
};

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
struct mxc_gpio_pad_wakeup {
	u32 pin_id;
	u32 type;
	u32 line;
};

struct imx_sc_msg_gpio_get_pad_wakeup {
	struct imx_sc_rpc_msg hdr;
	union {
		struct req_pad {
			u16 pad;
		} __packed req;
		struct resp_wakeup {
			u8 wakeup;
		} resp;
	} data;
} __packed;

struct imx_sc_msg_gpio_set_pad_wakeup {
	struct imx_sc_rpc_msg hdr;
	u16 pad;
	u8 wakeup;
} __packed;

#endif

/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};

struct mxc_gpio_reg_saved {
	u32 icr1;
	u32 icr2;
	u32 imr;
	u32 gdir;
	u32 edge_sel;
	u32 dr;
};

struct mxc_gpio_port {
	struct list_head node;
	void __iomem *base;
	struct clk *clk;
	int irq;
	int irq_high;
	struct irq_domain *domain;
	struct gpio_chip gc;
	struct device *dev;
	u32 both_edges;
	struct mxc_gpio_reg_saved gpio_saved_reg;
	bool power_off;
	bool gpio_ranges;
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	u32 pad_wakeup_num;
	struct mxc_gpio_pad_wakeup pad_wakeup[32];
#endif
};

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
static struct imx_sc_ipc *gpio_ipc_handle;
#endif

static struct mxc_gpio_hwdata imx1_imx21_gpio_hwdata = {
	.dr_reg		= 0x1c,
	.gdir_reg	= 0x00,
	.psr_reg	= 0x24,
	.icr1_reg	= 0x28,
	.icr2_reg	= 0x2c,
	.imr_reg	= 0x30,
	.isr_reg	= 0x34,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x03,
	.high_level	= 0x02,
	.rise_edge	= 0x00,
	.fall_edge	= 0x01,
};

static struct mxc_gpio_hwdata imx31_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= -EINVAL,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static struct mxc_gpio_hwdata imx35_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= 0x1c,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static enum mxc_gpio_hwtype mxc_gpio_hwtype;
static struct mxc_gpio_hwdata *mxc_gpio_hwdata;

#define GPIO_DR			(mxc_gpio_hwdata->dr_reg)
#define GPIO_GDIR		(mxc_gpio_hwdata->gdir_reg)
#define GPIO_PSR		(mxc_gpio_hwdata->psr_reg)
#define GPIO_ICR1		(mxc_gpio_hwdata->icr1_reg)
#define GPIO_ICR2		(mxc_gpio_hwdata->icr2_reg)
#define GPIO_IMR		(mxc_gpio_hwdata->imr_reg)
#define GPIO_ISR		(mxc_gpio_hwdata->isr_reg)
#define GPIO_EDGE_SEL		(mxc_gpio_hwdata->edge_sel_reg)

#define GPIO_INT_LOW_LEV	(mxc_gpio_hwdata->low_level)
#define GPIO_INT_HIGH_LEV	(mxc_gpio_hwdata->high_level)
#define GPIO_INT_RISE_EDGE	(mxc_gpio_hwdata->rise_edge)
#define GPIO_INT_FALL_EDGE	(mxc_gpio_hwdata->fall_edge)
#define GPIO_INT_BOTH_EDGES	0x4

static const struct platform_device_id mxc_gpio_devtype[] = {
	{
		.name = "imx1-gpio",
		.driver_data = IMX1_GPIO,
	}, {
		.name = "imx21-gpio",
		.driver_data = IMX21_GPIO,
	}, {
		.name = "imx31-gpio",
		.driver_data = IMX31_GPIO,
	}, {
		.name = "imx35-gpio",
		.driver_data = IMX35_GPIO,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id mxc_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx1-gpio", .data = &mxc_gpio_devtype[IMX1_GPIO], },
	{ .compatible = "fsl,imx21-gpio", .data = &mxc_gpio_devtype[IMX21_GPIO], },
	{ .compatible = "fsl,imx31-gpio", .data = &mxc_gpio_devtype[IMX31_GPIO], },
	{ .compatible = "fsl,imx35-gpio", .data = &mxc_gpio_devtype[IMX35_GPIO], },
	{ .compatible = "fsl,imx7d-gpio", .data = &mxc_gpio_devtype[IMX35_GPIO], },
	{ /* sentinel */ }
};

/*
 * MX2 has one interrupt *for all* gpio ports. The list is used
 * to save the references to all ports, so that mx2_gpio_irq_handler
 * can walk through all interrupt status registers.
 */
static LIST_HEAD(mxc_gpio_ports);

/* Note: This driver assumes 32 GPIOs are handled in one register */

static int gpio_set_irq_type(struct irq_data *d, u32 type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 bit, val;
	u32 gpio_idx = d->hwirq;
	int edge;
	void __iomem *reg = port->base;

	port->both_edges &= ~(1 << gpio_idx);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_INT_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_INT_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		if (GPIO_EDGE_SEL >= 0) {
			edge = GPIO_INT_BOTH_EDGES;
		} else {
			val = port->gc.get(&port->gc, gpio_idx);
			if (val) {
				edge = GPIO_INT_LOW_LEV;
				pr_debug("mxc: set GPIO %d to low trigger\n", gpio_idx);
			} else {
				edge = GPIO_INT_HIGH_LEV;
				pr_debug("mxc: set GPIO %d to high trigger\n", gpio_idx);
			}
			port->both_edges |= 1 << gpio_idx;
		}
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_INT_LOW_LEV;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_INT_HIGH_LEV;
		break;
	default:
		return -EINVAL;
	}

	if (GPIO_EDGE_SEL >= 0) {
		val = readl(port->base + GPIO_EDGE_SEL);
		if (edge == GPIO_INT_BOTH_EDGES)
			writel(val | (1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
		else
			writel(val & ~(1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
	}

	if (edge != GPIO_INT_BOTH_EDGES) {
		reg += GPIO_ICR1 + ((gpio_idx & 0x10) >> 2); /* lower or upper register */
		bit = gpio_idx & 0xf;
		val = readl(reg) & ~(0x3 << (bit << 1));
		writel(val | (edge << (bit << 1)), reg);
	}

	writel(1 << gpio_idx, port->base + GPIO_ISR);

	return 0;
}

static void mxc_flip_edge(struct mxc_gpio_port *port, u32 gpio)
{
	void __iomem *reg = port->base;
	u32 bit, val;
	int edge;

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = readl(reg);
	edge = (val >> (bit << 1)) & 3;
	val &= ~(0x3 << (bit << 1));
	if (edge == GPIO_INT_HIGH_LEV) {
		edge = GPIO_INT_LOW_LEV;
		pr_debug("mxc: switch GPIO %d to low trigger\n", gpio);
	} else if (edge == GPIO_INT_LOW_LEV) {
		edge = GPIO_INT_HIGH_LEV;
		pr_debug("mxc: switch GPIO %d to high trigger\n", gpio);
	} else {
		pr_err("mxc: invalid configuration for GPIO %d: %x\n",
		       gpio, edge);
		return;
	}
	writel(val | (edge << (bit << 1)), reg);
}

/* handle 32 interrupts in one status register */
static void mxc_gpio_irq_handler(struct mxc_gpio_port *port, u32 irq_stat)
{
	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;

		if (port->both_edges & (1 << irqoffset))
			mxc_flip_edge(port, irqoffset);

		generic_handle_irq(irq_find_mapping(port->domain, irqoffset));

		irq_stat &= ~(1 << irqoffset);
	}
}

/* MX1 and MX3 has one interrupt *per* gpio port */
static void mx3_gpio_irq_handler(struct irq_desc *desc)
{
	u32 irq_stat;
	struct mxc_gpio_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	irq_stat = readl(port->base + GPIO_ISR) & readl(port->base + GPIO_IMR);

	mxc_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);
}

/* MX2 has one interrupt *for all* gpio ports */
static void mx2_gpio_irq_handler(struct irq_desc *desc)
{
	u32 irq_msk, irq_stat;
	struct mxc_gpio_port *port;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	/* walk through all interrupt status registers */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		irq_msk = readl(port->base + GPIO_IMR);
		if (!irq_msk)
			continue;

		irq_stat = readl(port->base + GPIO_ISR) & irq_msk;
		if (irq_stat)
			mxc_gpio_irq_handler(port, irq_stat);
	}
	chained_irq_exit(chip, desc);
}

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
static int mxc_gpio_get_pad_wakeup(struct mxc_gpio_port *port)
{
	struct imx_sc_msg_gpio_get_pad_wakeup msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	u8 wakeup_type;
	int ret;
	int i;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_PAD;
	hdr->func = IMX_SC_PAD_FUNC_GET_WAKEUP;
	hdr->size = 2;

	for (i = 0; i < port->pad_wakeup_num; i++) {
		/* get original pad type */
		wakeup_type = port->pad_wakeup[i].type;
		msg.data.req.pad = port->pad_wakeup[i].pin_id;
		ret = imx_scu_call_rpc(gpio_ipc_handle, &msg, true);
		if (ret) {
			dev_err(port->gc.parent, "get pad wakeup failed, ret %d\n", ret);
			return ret;
		}
		wakeup_type = msg.data.resp.wakeup;
		/* return wakeup gpio pin's line */
		if (wakeup_type != port->pad_wakeup[i].type)
			return port->pad_wakeup[i].line;
	}

	return -EINVAL;
}

static void mxc_gpio_set_pad_wakeup(struct mxc_gpio_port *port, bool enable)
{
	struct imx_sc_msg_gpio_set_pad_wakeup msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;
	int i;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_PAD;
	hdr->func = IMX_SC_PAD_FUNC_SET_WAKEUP;
	hdr->size = 2;

	for (i = 0; i < port->pad_wakeup_num; i++) {
		msg.pad = port->pad_wakeup[i].pin_id;
		msg.wakeup = enable ? port->pad_wakeup[i].type : IMX_SC_PAD_WAKEUP_OFF;
		ret = imx_scu_call_rpc(gpio_ipc_handle, &msg, true);
		if (ret) {
			dev_err(port->gc.parent, "set pad wakeup failed, ret %d\n", ret);
			return;
		}
	}
}

static void mxc_gpio_handle_pad_wakeup(struct mxc_gpio_port *port, int line)
{
	struct irq_desc *desc = irq_to_desc(port->irq);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 irq_stat;

	/* skip invalid line */
	if (line > 31) {
		dev_err(port->gc.parent, "invalid wakeup line %d\n", line);
		return;
	}

	dev_info(port->gc.parent, "wakeup by pad, line %d\n", line);

	chained_irq_enter(chip, desc);

	irq_stat = (1 << line);

	mxc_gpio_irq_handler(port, irq_stat);

	chained_irq_exit(chip, desc);
}
#endif

/*
 * Set interrupt number "irq" in the GPIO as a wake-up source.
 * While system is running, all registered GPIO interrupts need to have
 * wake-up enabled. When system is suspended, only selected GPIO interrupts
 * need to have wake-up enabled.
 * @param  irq          interrupt source number
 * @param  enable       enable as wake-up if equal to non-zero
 * @return       This function returns 0 on success.
 */
static int gpio_set_wake_irq(struct irq_data *d, u32 enable)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;
	u32 gpio_idx = d->hwirq;
	int ret;

	if (enable) {
		if (port->irq_high && (gpio_idx >= 16))
			ret = enable_irq_wake(port->irq_high);
		else
			ret = enable_irq_wake(port->irq);
	} else {
		if (port->irq_high && (gpio_idx >= 16))
			ret = disable_irq_wake(port->irq_high);
		else
			ret = disable_irq_wake(port->irq);
	}

	return ret;
}

static int mxc_gpio_irq_reqres(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;

	if (gpiochip_lock_as_irq(&port->gc, d->hwirq)) {
		dev_err(port->gc.parent,
			"unable to lock HW IRQ %lu for IRQ\n",
			d->hwirq);
		return -EINVAL;
	}

	return irq_chip_pm_get(d);
}

static void mxc_gpio_irq_relres(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct mxc_gpio_port *port = gc->private;

	gpiochip_unlock_as_irq(&port->gc, d->hwirq);
	irq_chip_pm_put(d);
}

static int mxc_gpio_init_gc(struct mxc_gpio_port *port, int irq_base,
			    struct device *dev)
{
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int rv;

	gc = devm_irq_alloc_generic_chip(port->dev, "gpio-mxc", 1, irq_base,
					 port->base, handle_level_irq);
	if (!gc)
		return -ENOMEM;
	gc->private = port;

	ct = gc->chip_types;
	ct->chip.parent_device = dev;
	ct->chip.irq_ack = irq_gc_ack_set_bit;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->chip.irq_set_type = gpio_set_irq_type;
	ct->chip.irq_set_wake = gpio_set_wake_irq;
	ct->chip.irq_request_resources = mxc_gpio_irq_reqres;
	ct->chip.irq_release_resources = mxc_gpio_irq_relres,
	ct->chip.flags = IRQCHIP_MASK_ON_SUSPEND;
	ct->regs.ack = GPIO_ISR;
	ct->regs.mask = GPIO_IMR;

	rv = devm_irq_setup_generic_chip(port->dev, gc, IRQ_MSK(32),
					 IRQ_GC_INIT_NESTED_LOCK,
					 IRQ_NOREQUEST, 0);

	return rv;
}

static void mxc_gpio_get_hw(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxc_gpio_dt_ids, &pdev->dev);
	enum mxc_gpio_hwtype hwtype;

	if (of_id)
		pdev->id_entry = of_id->data;
	hwtype = pdev->id_entry->driver_data;

	if (mxc_gpio_hwtype) {
		/*
		 * The driver works with a reasonable presupposition,
		 * that is all gpio ports must be the same type when
		 * running on one soc.
		 */
		BUG_ON(mxc_gpio_hwtype != hwtype);
		return;
	}

	if (hwtype == IMX35_GPIO)
		mxc_gpio_hwdata = &imx35_gpio_hwdata;
	else if (hwtype == IMX31_GPIO)
		mxc_gpio_hwdata = &imx31_gpio_hwdata;
	else
		mxc_gpio_hwdata = &imx1_imx21_gpio_hwdata;

	mxc_gpio_hwtype = hwtype;
}

static int mxc_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(gc);

	return irq_find_mapping(port->domain, offset);
}

static int mxc_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(chip);
	int ret;

	if (port->gpio_ranges) {
		ret = gpiochip_generic_request(chip, offset);
		if (ret)
			return ret;
	}

	ret = pm_runtime_get_sync(chip->parent);
	return ret < 0 ? ret : 0;
}

static void mxc_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct mxc_gpio_port *port = gpiochip_get_data(chip);

	if (port->gpio_ranges)
		gpiochip_generic_free(chip, offset);
	pm_runtime_put(chip->parent);
}

static int mxc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mxc_gpio_port *port;
	int irq_count;
	int irq_base = 0;
	int err;
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	int i;
#endif

	mxc_gpio_get_hw(pdev);

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->dev = &pdev->dev;

	port->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	irq_count = platform_irq_count(pdev);
	if (irq_count < 0)
		return irq_count;

	if (irq_count > 1) {
		port->irq_high = platform_get_irq(pdev, 1);
		if (port->irq_high < 0)
			port->irq_high = 0;
	}

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return port->irq;

	/* the controller clock is optional */
	port->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(port->clk))
		return PTR_ERR(port->clk);

	err = clk_prepare_enable(port->clk);
	if (err) {
		dev_err(&pdev->dev, "Unable to enable clock.\n");
		return err;
	}

#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	/*
	 * parse pad wakeup info from dtb, each pad has to provide
	 * <pin_id, type, line>, these info should be put in each
	 * gpio node and with a "pad-wakeup-num" to indicate the
	 * total lines are with pad wakeup enabled.
	 */
	if (!of_property_read_u32(np, "pad-wakeup-num", &port->pad_wakeup_num)) {
		if (port->pad_wakeup_num != 0) {
			if (!gpio_ipc_handle) {
				err = imx_scu_get_handle(&gpio_ipc_handle);
				if (err)
					return err;
			}
			for (i = 0; i < port->pad_wakeup_num; i++) {
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 0, &port->pad_wakeup[i].pin_id);
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 1, &port->pad_wakeup[i].type);
				of_property_read_u32_index(np, "pad-wakeup",
					i * 3 + 2, &port->pad_wakeup[i].line);
			}
			err = imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_WAKE, IMX_SC_IRQ_PAD, true);
			if (err)
				dev_warn(&pdev->dev, "Enable irq failed, GPIO pad wakeup NOT supported\n");
		}
	}
#endif

	if (of_device_is_compatible(np, "fsl,imx7d-gpio"))
		port->power_off = true;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	err = pm_runtime_get_sync(&pdev->dev);
	if (err < 0)
		goto out_pm_dis;

	/* disable the interrupt and clear the status */
	writel(0, port->base + GPIO_IMR);
	writel(~0, port->base + GPIO_ISR);

	if (mxc_gpio_hwtype == IMX21_GPIO) {
		/*
		 * Setup one handler for all GPIO interrupts. Actually setting
		 * the handler is needed only once, but doing it for every port
		 * is more robust and easier.
		 */
		irq_set_chained_handler(port->irq, mx2_gpio_irq_handler);
	} else {
		/* setup one handler for each entry */
		irq_set_chained_handler_and_data(port->irq,
						 mx3_gpio_irq_handler, port);
		if (port->irq_high > 0)
			/* setup handler for GPIO 16 to 31 */
			irq_set_chained_handler_and_data(port->irq_high,
							 mx3_gpio_irq_handler,
							 port);
	}

	err = bgpio_init(&port->gc, &pdev->dev, 4,
			 port->base + GPIO_PSR,
			 port->base + GPIO_DR, NULL,
			 port->base + GPIO_GDIR, NULL,
			 BGPIOF_READ_OUTPUT_REG_SET);
	if (err)
		goto out_bgio;

	if (of_property_read_bool(np, "gpio-ranges")) {
		port->gc.request = gpiochip_generic_request;
		port->gc.free = gpiochip_generic_free;
	}

	if (of_property_read_bool(np, "gpio_ranges"))
		port->gpio_ranges = true;
	else
		port->gpio_ranges = false;

	port->gc.request = mxc_gpio_request;
	port->gc.free = mxc_gpio_free;
	port->gc.parent = &pdev->dev;
	port->gc.to_irq = mxc_gpio_to_irq;
	port->gc.base = (pdev->id < 0) ? of_alias_get_id(np, "gpio") * 32 :
					     pdev->id * 32;

	err = devm_gpiochip_add_data(&pdev->dev, &port->gc, port);
	if (err)
		goto out_bgio;

	irq_base = devm_irq_alloc_descs(&pdev->dev, -1, 0, 32, numa_node_id());
	if (irq_base < 0) {
		err = irq_base;
		goto out_bgio;
	}

	port->domain = irq_domain_add_legacy(np, 32, irq_base, 0,
					     &irq_domain_simple_ops, NULL);
	if (!port->domain) {
		err = -ENODEV;
		goto out_bgio;
	}

	/* gpio-mxc can be a generic irq chip */
	err = mxc_gpio_init_gc(port, irq_base, &pdev->dev);
	if (err < 0)
		goto out_irqdomain_remove;

	list_add_tail(&port->node, &mxc_gpio_ports);

	platform_set_drvdata(pdev, port);
	pm_runtime_put(&pdev->dev);

	return 0;

out_pm_dis:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(port->clk);
out_irqdomain_remove:
	irq_domain_remove(port->domain);
out_bgio:
	clk_disable_unprepare(port->clk);
	dev_info(&pdev->dev, "%s failed with errno %d\n", __func__, err);
	return err;
}

static void mxc_gpio_save_regs(struct mxc_gpio_port *port)
{
	if (!port->power_off)
		return;

	port->gpio_saved_reg.icr1 = readl(port->base + GPIO_ICR1);
	port->gpio_saved_reg.icr2 = readl(port->base + GPIO_ICR2);
	port->gpio_saved_reg.imr = readl(port->base + GPIO_IMR);
	port->gpio_saved_reg.gdir = readl(port->base + GPIO_GDIR);
	port->gpio_saved_reg.edge_sel = readl(port->base + GPIO_EDGE_SEL);
	port->gpio_saved_reg.dr = readl(port->base + GPIO_DR);
}

static void mxc_gpio_restore_regs(struct mxc_gpio_port *port)
{
	if (!port->power_off)
		return;

	writel(port->gpio_saved_reg.icr1, port->base + GPIO_ICR1);
	writel(port->gpio_saved_reg.icr2, port->base + GPIO_ICR2);
	writel(port->gpio_saved_reg.imr, port->base + GPIO_IMR);
	writel(port->gpio_saved_reg.gdir, port->base + GPIO_GDIR);
	writel(port->gpio_saved_reg.edge_sel, port->base + GPIO_EDGE_SEL);
	writel(port->gpio_saved_reg.dr, port->base + GPIO_DR);
}

static int __maybe_unused mxc_gpio_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);

	mxc_gpio_save_regs(port);
	clk_disable_unprepare(port->clk);

	return 0;
}

static int __maybe_unused mxc_gpio_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);
	int ret;

	ret = clk_prepare_enable(port->clk);
	if (ret)
		return ret;

	mxc_gpio_restore_regs(port);

	return 0;
}

static int __maybe_unused mxc_gpio_noirq_suspend(struct device *dev)
{
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);

	mxc_gpio_set_pad_wakeup(port, true);
#endif
	return 0;
}

static int __maybe_unused mxc_gpio_noirq_resume(struct device *dev)
{
#ifdef CONFIG_GPIO_MXC_PAD_WAKEUP
	struct platform_device *pdev = to_platform_device(dev);
	struct mxc_gpio_port *port = platform_get_drvdata(pdev);
	int wakeup_line = mxc_gpio_get_pad_wakeup(port);

	mxc_gpio_set_pad_wakeup(port, false);

	if (wakeup_line >= 0)
		mxc_gpio_handle_pad_wakeup(port, wakeup_line);
#endif
	return 0;
}

static const struct dev_pm_ops mxc_gpio_dev_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(mxc_gpio_noirq_suspend, mxc_gpio_noirq_resume)
	SET_RUNTIME_PM_OPS(mxc_gpio_runtime_suspend, mxc_gpio_runtime_resume, NULL)
};

static int mxc_gpio_syscore_suspend(void)
{
	struct mxc_gpio_port *port;
	int ret;

	/* walk through all ports */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		ret = clk_prepare_enable(port->clk);
		if (ret)
			return ret;
		mxc_gpio_save_regs(port);
		clk_disable_unprepare(port->clk);
	}

	return 0;
}

static void mxc_gpio_syscore_resume(void)
{
	struct mxc_gpio_port *port;
	int ret;

	/* walk through all ports */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		ret = clk_prepare_enable(port->clk);
		if (ret) {
			pr_err("mxc: failed to enable gpio clock %d\n", ret);
			return;
		}
		mxc_gpio_restore_regs(port);
		clk_disable_unprepare(port->clk);
	}
}

static struct syscore_ops mxc_gpio_syscore_ops = {
	.suspend = mxc_gpio_syscore_suspend,
	.resume = mxc_gpio_syscore_resume,
};

static struct platform_driver mxc_gpio_driver = {
	.driver		= {
		.name	= "gpio-mxc",
		.of_match_table = mxc_gpio_dt_ids,
		.suppress_bind_attrs = true,
		.pm = &mxc_gpio_dev_pm_ops,
	},
	.probe		= mxc_gpio_probe,
	.id_table	= mxc_gpio_devtype,
};

static int __init gpio_mxc_init(void)
{
	register_syscore_ops(&mxc_gpio_syscore_ops);

	return platform_driver_register(&mxc_gpio_driver);
}
subsys_initcall(gpio_mxc_init);
MODULE_LICENSE("GPL v2");
