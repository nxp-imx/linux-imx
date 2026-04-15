// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Pericom PI4IOE5V6408 GPIO Expander.
 *
 * Copyright (C) 2018 Google, LLC.
 * Copyright (C) 2023 SECO Northern Europe GmbH
 */

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define PI4IO_CHIP_ID 0x1
#define PI4IO_IO_DIRECTION 0x3
#define PI4IO_OUTPUT 0x5
#define PI4IO_OUTPUT_HI_IMPEDANCE 0x7
#define PI4IO_INPUT_DEFAULT_STATE 0x9
#define PI4IO_INPUT_STATUS 0xF
#define PI4IO_INTERRUPT_MASK 0x11
#define PI4IO_INTERRUPT_STATUS 0x13

#define PI4IO_CHIP_ID_VAL 0xA0
#define PI4IO_CHIP_ID_MASK 0xFC
#define GPIOF_DIR_OUT (0<<0)
#define GPIOF_DIR_IN (1<<0)
#define PI4IO_DIRECTION_TO_GPIOD(x) ((x) ? GPIOF_DIR_OUT : GPIOF_DIR_IN)
#define GPIOD_DIRECTION_TO_PI4IO(x) ((x) == GPIOF_DIR_OUT ? 1 : 0)

static bool pi4io_readable_reg(struct device *dev, unsigned int reg)
{
	/* All readable registers are odd-numbered. */
	return (reg % 2) == 1;
}

static bool pi4io_writeable_reg(struct device *dev, unsigned int reg)
{
	/* All odd-numbered registers are writable except for 0xF. */
	if ((reg % 2) == 1)
		if (reg != PI4IO_INPUT_STATUS)
			return true;

	return false;
}

static bool pi4io_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == PI4IO_INPUT_STATUS || reg == PI4IO_INTERRUPT_STATUS)
		return true;

	return false;
}

static const struct regmap_config pi4io_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x13,
	.writeable_reg = pi4io_writeable_reg,
	.readable_reg = pi4io_readable_reg,
	.volatile_reg = pi4io_volatile_reg,
};

struct pi4io_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct gpio_chip chip;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	struct irq_chip irqchip;
	unsigned int irq_type[8];
	unsigned int irq_enabled;
	unsigned int irq_status;
};

static int pi4io_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	int ret, io_dir;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_read(pi4io->regmap, PI4IO_IO_DIRECTION, &io_dir);
	if (ret) {
		dev_err(dev, "Failed to read I/O direction: %d", ret);
		return ret;
	}

	return PI4IO_DIRECTION_TO_GPIOD((io_dir >> offset) & 1);
}

static int pi4io_gpio_set_direction(
	struct gpio_chip *chip, unsigned int offset, int direction)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_update_bits(pi4io->regmap, PI4IO_IO_DIRECTION, 1 << offset,
		GPIOD_DIRECTION_TO_PI4IO(direction) << offset);
	if (ret) {
		dev_err(dev, "Failed to set direction: %d", ret);
		return ret;
	}

	/* We desire the hi-impedance state to track the output state. */
	ret = regmap_update_bits(pi4io->regmap, PI4IO_OUTPUT_HI_IMPEDANCE,
		1 << offset, direction << offset);

	return ret;
}

static int pi4io_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret, out;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = pi4io_gpio_get_direction(chip, offset);

	if (ret == GPIOF_DIR_IN)
		ret = regmap_read(pi4io->regmap, PI4IO_INPUT_STATUS, &out);
	else if (ret == GPIOF_DIR_OUT)
		ret = regmap_read(pi4io->regmap, PI4IO_OUTPUT, &out);
	else
		return ret;

	if (ret) {
		dev_err(dev, "Failed to read output: %d", ret);
		return ret;
	}

	if (out & (1 << offset))
		return 1;

	return 0;
}

static int pi4io_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_update_bits(
		pi4io->regmap, PI4IO_OUTPUT, 1 << offset, value << offset);
	if (ret)
		dev_err(dev, "Failed to write output: %d", ret);

	return ret;
}

static int pi4io_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_IN);
}

static int pi4io_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_OUT);
	if (ret) {
		dev_err(dev, "Failed to set direction: %d", ret);
		return ret;
	}

	pi4io_gpio_set(chip, offset, value);

	return 0;
}

static int pi4io_gpio_setup(struct pi4io_priv *pi4io)
{
	int ret;
	struct device *dev = &pi4io->i2c->dev;
	struct gpio_chip *gc = &pi4io->chip;

	gc->ngpio = 8;
	gc->label = pi4io->i2c->name;
	gc->parent = &pi4io->i2c->dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = true;

	gc->get_direction = pi4io_gpio_get_direction;
	gc->direction_input = pi4io_gpio_direction_input;
	gc->direction_output = pi4io_gpio_direction_output;
	gc->get = pi4io_gpio_get;
	gc->set = pi4io_gpio_set;
	ret = devm_gpiochip_add_data(dev, gc, pi4io);
	if (ret) {
		dev_err(dev, "devm_gpiochip_add_data failed: %d", ret);
		return ret;
	}

	return 0;
}

static int pi4io_reset(struct pi4io_priv *pi4io)
{
	struct device *dev = &pi4io->i2c->dev;

	if (pi4io->reset_gpio) {
		dev_dbg(dev, "Performing reset");

		/*
		 * According to the datasheet, the reset time should
		 * be 150 ns. However, when using 150 ns, the device
		 * doesn't come up correctly.
		 */
		gpiod_set_value_cansleep(pi4io->reset_gpio, 1);
		udelay(150);
		gpiod_set_value_cansleep(pi4io->reset_gpio, 0);
		udelay(150);
	}

	return 0;
};

static irqreturn_t pi4io_irq(int irq, void *data)
{
	struct pi4io_priv *pi4io = data;
	struct device *dev = &pi4io->i2c->dev;
	unsigned long change, offset;
	int ret, status;

	ret = regmap_read(pi4io->regmap, PI4IO_INTERRUPT_STATUS, &status);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupts: %d", ret);
		return IRQ_NONE;
	}

	/*
	 * Only if the interrupt for a specific port is enabled,
	 * the irq_mapping further down is working. To prevent
	 * errors from popping up, we only process interrupts
	 * for ports that are enabled.
	 */
	mutex_lock(&pi4io->lock);
	change = status & pi4io->irq_enabled;
	mutex_unlock(&pi4io->lock);

	dev_dbg(dev, "irq_enabled: %d status: %d change: %lu",
			pi4io->irq_enabled, status, change);

	for_each_set_bit(offset, &change, pi4io->chip.ngpio) {
		ret = irq_find_mapping(pi4io->chip.irq.domain, offset);

		if (ret != 0) {
			handle_nested_irq(ret);
			dev_dbg(dev, "IRQ handled");
		} else {
			dev_err(dev, "Invalid IRQ mapping");
			return IRQ_NONE;
		}
	}

	return IRQ_HANDLED;
}

static void pi4io_irq_ack(struct irq_data *data)
{
	/* NOOP */
}

static void pi4io_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);

	dev_dbg(&pi4io->i2c->dev, "Masking hwirq %lu", hwirq);
	gpiochip_disable_irq(&pi4io->chip, hwirq);
}

static void pi4io_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);

	dev_dbg(&pi4io->i2c->dev, "Unmasking hwirq %lu", hwirq);
	gpiochip_enable_irq(&pi4io->chip, hwirq);
}

static int pi4io_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);

	dev_dbg(&pi4io->i2c->dev, "Set wake");
	return irq_set_irq_wake(pi4io->i2c->irq, on);
}

static void pi4io_irq_enable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);

	pi4io->irq_enabled |= (1 << data->hwirq);
	dev_dbg(&pi4io->i2c->dev, "Enabled IRQs: %u",
			pi4io->irq_enabled);
}

static void pi4io_irq_disable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);

	pi4io->irq_enabled &= ~(1 << data->hwirq);
	dev_dbg(&pi4io->i2c->dev, "Enabled IRQs: %u",
			pi4io->irq_enabled);
}

static void pi4io_irq_bus_lock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);

	dev_dbg(&pi4io->i2c->dev, "Lock bus");
	mutex_lock(&pi4io->lock);
}

static void pi4io_irq_bus_sync_unlock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);
	int ret;

	/*
	 * The regmap update should be done in the irq_enable function.
	 * However, accessing the regmap in the enable/disable function
	 * leads to a segmentation fault.
	 */

	ret = regmap_update_bits(pi4io->regmap, PI4IO_INPUT_DEFAULT_STATE, 1 << data->hwirq,
			pi4io->irq_type[data->hwirq] << data->hwirq);
	if (ret)
		dev_err(&pi4io->i2c->dev, "Unable to set interrupt for %lu",
				data->hwirq);

	ret = regmap_update_bits(pi4io->regmap, PI4IO_INTERRUPT_MASK, 1 << data->hwirq,
			~pi4io->irq_enabled & (1 << data->hwirq));
	if (ret)
		dev_err(&pi4io->i2c->dev, "Masking of %lu failed",
				data->hwirq);

	mutex_unlock(&pi4io->lock);
	dev_dbg(&pi4io->i2c->dev, "Unlock bus");
}

static int pi4io_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);
	unsigned int irq_type;

	/*
	 * The pi4io only supports high/low level interrupts. The right
	 * way would be to only support those two modes. However, gpiomon
	 * (and probably other tools too) do crash, if we do not support
	 * all five modes.
	 *
	 * Note: The edge both mode doesn't work and will do the same as
	 * the rising edge interrupt.
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_LEVEL_HIGH:
		dev_dbg(&pi4io->i2c->dev, "Set rising edge IRQ for %lu",
				data->hwirq);
		irq_type = 0;
		break;

	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_LOW:
		dev_dbg(&pi4io->i2c->dev, "Set falling edge IRQ for %lu",
			data->hwirq);
		irq_type = 1;
		break;

	default:
		dev_err(&pi4io->i2c->dev, "Invalid IRQ type %u for %lu",
			type, data->hwirq);
		return -EINVAL;
	}

	pi4io->irq_type[data->hwirq] = irq_type;

	return 0;
}

static int pi4io_irq_init_hw(struct gpio_chip *gc)
{
	struct pi4io_priv *pi4io = gpiochip_get_data(gc);
	int ret, status;

	/*
	 * Disable (mask) all interrupts and read the status
	 * register. Reading the status register resets all
	 * active interrupts.
	 */

	pi4io->irq_enabled = 0;

	ret = regmap_update_bits(pi4io->regmap, PI4IO_INTERRUPT_MASK, 0xFF, 0xFF);
	if (ret)
		dev_err(&pi4io->i2c->dev, "Masking of all interrupts failed");

	ret = regmap_read(pi4io->regmap, PI4IO_INTERRUPT_STATUS, &status);
	if (ret < 0)
		dev_err(&pi4io->i2c->dev, "Failed to read interrupts: %d", ret);

	dev_dbg(&pi4io->i2c->dev, "Current interrupt status: %d", status);

	return 0;
}

/*
 * @TODO Rework the return/error handling
 */
static int pi4io_probe(
	struct i2c_client *client)
{
	int ret, chip_id;
	struct device *dev = &client->dev;
	struct pi4io_priv *pi4io;
	struct gpio_irq_chip *gpio_irqchip;

	pi4io = devm_kzalloc(dev, sizeof(struct pi4io_priv), GFP_KERNEL);
	if (!pi4io)
		return -ENOMEM;

	i2c_set_clientdata(client, pi4io);
	pi4io->i2c = client;

	pi4io->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (!pi4io->reset_gpio)
		dev_warn(dev, "Could not get reset-gpios");

	pi4io_reset(pi4io);
	pi4io->regmap = devm_regmap_init_i2c(pi4io->i2c, &pi4io_regmap);
	ret = regmap_read(pi4io->regmap, PI4IO_CHIP_ID, &chip_id);
	if (ret < 0) {
		dev_err(dev, "Failed to read Chip ID: %d", ret);
		return ret;
	}
	if ((chip_id & PI4IO_CHIP_ID_MASK) != PI4IO_CHIP_ID_VAL) {
		dev_err(dev, "Invalid Chip ID");
		return -EINVAL;
	}

	if (pi4io->i2c->irq) {
		dev_dbg(dev, "Setup IRQ");

		pi4io->irqchip.name = "pi4io";
		pi4io->irqchip.irq_enable = pi4io_irq_enable;
		pi4io->irqchip.irq_disable = pi4io_irq_disable;
		pi4io->irqchip.irq_ack = pi4io_irq_ack;
		pi4io->irqchip.irq_mask = pi4io_irq_mask;
		pi4io->irqchip.irq_unmask = pi4io_irq_unmask;
		pi4io->irqchip.irq_set_wake = pi4io_irq_set_wake;
		pi4io->irqchip.irq_bus_lock = pi4io_irq_bus_lock;
		pi4io->irqchip.irq_bus_sync_unlock = pi4io_irq_bus_sync_unlock;
		pi4io->irqchip.irq_set_type = pi4io_irq_set_type;

		ret = devm_request_threaded_irq(dev, pi4io->i2c->irq,
					NULL, pi4io_irq, IRQF_ONESHOT |
					IRQF_TRIGGER_FALLING | IRQF_SHARED,
					dev_name(dev), pi4io);

		if (ret)
			goto fail;

		gpio_irqchip = &pi4io->chip.irq;
		gpio_irqchip->chip = &pi4io->irqchip;
		/* This will let us handle the parent IRQ in the driver */
		gpio_irqchip->parent_handler = NULL;
		gpio_irqchip->num_parents = 0;
		gpio_irqchip->parents = NULL;
		gpio_irqchip->default_type = IRQ_TYPE_NONE;
		gpio_irqchip->handler = handle_level_irq;
		gpio_irqchip->init_hw = pi4io_irq_init_hw;
		gpio_irqchip->threaded = true;
	}

	ret = pi4io_gpio_setup(pi4io);
	if (ret < 0) {
		dev_err(dev, "Failed to setup GPIOs: %d", ret);
		return ret;
	}

	dev_info(dev, "PI4IO probe finished");
	return 0;

fail:
	dev_err(dev, "PI4IO probe error %d for %s", ret,
		pi4io->i2c->name);

	return ret;
}

static const struct i2c_device_id pi4io_id_table[] = {
	{ "pi4io", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pi4io_id_table);

static const struct of_device_id pi4io_of_table[] = {
	{ .compatible = "diodes,pi4io" },
	{ },
};
MODULE_DEVICE_TABLE(of, pi4io_of_table);

static struct i2c_driver pi4io_driver = {
	.driver = {
		.name = "pi4io-gpio",
	},
	.probe = pi4io_probe,
	.id_table = pi4io_id_table,
};
module_i2c_driver(pi4io_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("PI4IOE5V6408");
MODULE_LICENSE("GPL");
