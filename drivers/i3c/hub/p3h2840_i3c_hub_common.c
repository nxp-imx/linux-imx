// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * This P3H2X4X driver file implements functions for Hub probe and DT parsing.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/mfd/p3h2840.h>
#include <linux/util_macros.h>

#include "p3h2840_i3c_hub.h"

/* LDO voltage DT settings */
#define P3H2X4X_DT_LDO_VOLT_1_0V		1000000
#define P3H2X4X_DT_LDO_VOLT_1_1V		1100000
#define P3H2X4X_DT_LDO_VOLT_1_2V		1200000
#define P3H2X4X_DT_LDO_VOLT_1_8V		1800000

static const int p3h2x4x_pullup_tbl[] = {
	250, 500, 1000, 2000
};

static const int p3h2x4x_io_strength_tbl[] = {
	20, 30, 40, 50
};

static u8 p3h2x4x_pullup_dt_to_reg(int dt_value)
{
	return find_closest(dt_value, p3h2x4x_pullup_tbl,
			  ARRAY_SIZE(p3h2x4x_pullup_tbl));
}

static u8 p3h2x4x_io_strength_dt_to_reg(int dt_value)
{
	return find_closest(dt_value, p3h2x4x_io_strength_tbl,
			  ARRAY_SIZE(p3h2x4x_io_strength_tbl));
}

static int p3h2x4x_configure_pullup(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 pullup;

	pullup = P3H2X4X_TP0145_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp0145_pullup));

	pullup |= P3H2X4X_TP2367_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp2367_pullup));

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_LDO_AND_PULLUP_CONF,
							  P3H2X4X_PULLUP_CONF_MASK, pullup);
}

static int p3h2x4x_configure_io_strength(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 io_strength;

	io_strength = P3H2X4X_CP0_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.cp0_io_strength));

	io_strength |= P3H2X4X_CP1_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.cp1_io_strength));

	io_strength |= P3H2X4X_TP0145_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp0145_io_strength));

	io_strength |= P3H2X4X_TP2367_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp2367_io_strength));

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_IO_STRENGTH,
							  P3H2X4X_IO_STRENGTH_MASK, io_strength);
}

static int p3h2x4x_configure_ldo(struct device *dev)
{
	static const char * const supplies[] = {
		"vcc1",
		"vcc2",
		"vcc3",
		"vcc4"
	};
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(supplies); i++) {
		ret = devm_regulator_get_enable_optional(dev->parent, supplies[i]);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		if (ret && ret != -ENODEV)
			dev_warn(dev, "Failed to enable %s (%d)\n",
				 supplies[i], ret);
	}

	/* This delay is required for the regulator to stabilize its output voltage */
	mdelay(5);

	return 0;
}

static int p3h2x4x_configure_tp(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *hub = dev_get_drvdata(dev);
	u8 mode = 0, smbus = 0, pullup = 0, target_port = 0;
	int tp, ret;

	for (tp = 0; tp < P3H2X4X_TP_MAX_COUNT; tp++) {
		pullup |= hub->hub_config.tp_config[tp].pullup_en ? P3H2X4X_SET_BIT(tp) : 0;
		mode |= (hub->hub_config.tp_config[tp].mode != P3H2X4X_TP_MODE_I3C) ?
			P3H2X4X_SET_BIT(tp) : 0;
		smbus |= (hub->hub_config.tp_config[tp].mode == P3H2X4X_TP_MODE_SMBUS) ?
			 P3H2X4X_SET_BIT(tp) : 0;
		target_port |= (hub->tp_bus[tp].tp_mask == P3H2X4X_SET_BIT(tp)) ?
			       hub->tp_bus[tp].tp_mask : 0;
	}

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_PULLUP_EN, pullup, pullup);
	if (ret)
		return ret;

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_IO_MODE_CONF, mode, mode);
	if (ret)
		return ret;

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_SMBUS_AGNT_EN, smbus, smbus);
	if (ret)
		return ret;

	if (target_port & ~smbus) {
		ret = regmap_write(hub->regmap, P3H2X4X_CP_MUX_SET,
				   P3H2X4X_CONTROLLER_PORT_MUX_REQ);
		if (ret)
			return ret;
	}

	return regmap_update_bits(hub->regmap, P3H2X4X_TP_ENABLE, target_port, target_port);
}

static int p3h2x4x_configure_hw(struct device *dev)
{
	int ret;

	ret = p3h2x4x_configure_ldo(dev);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_pullup(dev);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_io_strength(dev);
	if (ret)
		return ret;

	return p3h2x4x_configure_tp(dev);
}

static void p3h2x4x_get_target_port_dt_conf(struct device *dev,
					    const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u64 tp_port;

	for_each_available_child_of_node_scoped(node, dev_node) {
		if (of_property_read_reg(dev_node, 0, &tp_port, NULL))
			continue;

		if (tp_port < P3H2X4X_TP_MAX_COUNT) {
			p3h2x4x_i3c_hub->tp_bus[tp_port].of_node = dev_node;
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_mask = P3H2X4X_SET_BIT(tp_port);
			p3h2x4x_i3c_hub->tp_bus[tp_port].p3h2x4x_i3c_hub = p3h2x4x_i3c_hub;
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_port = tp_port;
		}
	}
}

static void p3h2x4x_parse_tp_dt_settings(struct device *dev,
					 const struct device_node *node,
					 struct tp_configuration tp_config[])
{
	u64 id;

	for_each_available_child_of_node_scoped(node, tp_node) {
		if (of_property_read_reg(tp_node, 0, &id, NULL))
			continue;

		if (id >= P3H2X4X_TP_MAX_COUNT) {
			dev_warn(dev, "Invalid target port index found in DT: %lli\n", id);
			continue;
		}

		if (strcmp(tp_node->name, "i3c") == 0)
			tp_config[id].mode = P3H2X4X_TP_MODE_I3C;

		if (strcmp(tp_node->name, "i2c") == 0)
			tp_config[id].mode = P3H2X4X_TP_MODE_I2C;

		if (strcmp(tp_node->name, "smbus") == 0)
			tp_config[id].mode = P3H2X4X_TP_MODE_SMBUS;

		tp_config[id].pullup_en =
			of_property_read_bool(tp_node, "nxp,pullup-enable");
	}
}

static void p3h2x4x_get_hub_dt_conf(struct device *dev,
				    const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);

	of_property_read_u32(node, "nxp,tp0145-pullup-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp0145_pullup);
	of_property_read_u32(node, "nxp,tp2367-pullup-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp2367_pullup);
	of_property_read_u32(node, "nxp,cp0-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.cp0_io_strength);
	of_property_read_u32(node, "nxp,cp1-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.cp1_io_strength);
	of_property_read_u32(node, "nxp,tp0145-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp0145_io_strength);
	of_property_read_u32(node, "nxp,tp2367-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp2367_io_strength);

	p3h2x4x_parse_tp_dt_settings(dev, node, p3h2x4x_i3c_hub->hub_config.tp_config);
}

static void p3h2x4x_default_configuration(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	int tp_count;

	p3h2x4x_i3c_hub->hub_config.tp0145_pullup = P3H2X4X_TP_PULLUP_500R;
	p3h2x4x_i3c_hub->hub_config.tp2367_pullup = P3H2X4X_TP_PULLUP_500R;
	p3h2x4x_i3c_hub->hub_config.cp0_io_strength = P3H2X4X_IO_STRENGTH_20_OHM;
	p3h2x4x_i3c_hub->hub_config.cp1_io_strength = P3H2X4X_IO_STRENGTH_20_OHM;
	p3h2x4x_i3c_hub->hub_config.tp0145_io_strength = P3H2X4X_IO_STRENGTH_20_OHM;
	p3h2x4x_i3c_hub->hub_config.tp2367_io_strength = P3H2X4X_IO_STRENGTH_20_OHM;

	for (tp_count = 0; tp_count < P3H2X4X_TP_MAX_COUNT; ++tp_count)
		p3h2x4x_i3c_hub->hub_config.tp_config[tp_count].mode =  P3H2X4X_TP_MODE_I3C;
}

static int p3h2x4x_i3c_hub_access_reg_get(void *dvrdata, u64 *val)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dvrdata;
	u32 reg_val;
	int ret;

	ret = regmap_read(p3h2x4x_i3c_hub->regmap, p3h2x4x_i3c_hub->reg_addr, &reg_val);
	if (ret)
		return ret;

	*val = reg_val & 0xFF;
	return 0;
}

static int p3h2x4x_i3c_hub_access_reg_set(void *dvrdata, u64 val)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dvrdata;

	return regmap_write(p3h2x4x_i3c_hub->regmap, p3h2x4x_i3c_hub->reg_addr, val & 0xFF);
}

DEFINE_DEBUGFS_ATTRIBUTE(p3h2x4x_i3c_hub_access_reg, p3h2x4x_i3c_hub_access_reg_get,
			 p3h2x4x_i3c_hub_access_reg_set, "%llx\n");

static int p3h2x4x_i3c_hub_debugfs_init(struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub,
					const char *hub_id)
{
	struct dentry *p3h2x4x_conf_dir;
	struct dentry *sysdir;
	struct dentry *tp_dir;
	struct dentry *cp_dir;
	struct dentry *reg_dir;

	char file_name[32];
	int i;

	sysdir = debugfs_create_dir(hub_id, NULL);
	if (IS_ERR(sysdir))
		return PTR_ERR(sysdir);

	p3h2x4x_i3c_hub->sysfs_debug_dir = sysdir;

	sysdir = debugfs_create_dir("dt-p3h2x4x-conf", p3h2x4x_i3c_hub->sysfs_debug_dir);
	if (IS_ERR(sysdir))
		goto err_remove;

	p3h2x4x_conf_dir = sysdir;

	cp_dir = debugfs_create_dir("p3h2x4x-control-ports", p3h2x4x_conf_dir);
	if (IS_ERR(cp_dir))
		goto err_remove;

	/* for control ports 0 */
	debugfs_create_u32("cp0-io-strength-ohms", 0400, cp_dir,
			   &p3h2x4x_i3c_hub->hub_config.cp0_io_strength);

	/* for control ports 1 */
	debugfs_create_u32("cp1-io-strength-ohms", 0400, cp_dir,
			   &p3h2x4x_i3c_hub->hub_config.cp1_io_strength);

	for (i = 0; i < P3H2X4X_TP_MAX_COUNT; ++i) {
		sprintf(file_name, "target-port-%d", i);

		tp_dir = debugfs_create_dir(file_name, p3h2x4x_conf_dir);
		if (IS_ERR(tp_dir))
			goto err_remove;

		debugfs_create_u32("mode", 0400, tp_dir,
				   (u32 *)&p3h2x4x_i3c_hub->hub_config.tp_config[i].mode);

		debugfs_create_bool("pullup-enable", 0400, tp_dir,
				    &p3h2x4x_i3c_hub->hub_config.tp_config[i].pullup_en);

		debugfs_create_bool("IBI-enable", 0400, tp_dir,
				    &p3h2x4x_i3c_hub->hub_config.tp_config[i].ibi_en);

		debugfs_create_bool("always_enable", 0400, tp_dir,
				    &p3h2x4x_i3c_hub->hub_config.tp_config[i].always_enable);

		if ((i / 2) % 2 == 0) {  /* for target groups 0145*/
			debugfs_create_u32("io-strength-ohms", 0400, tp_dir,
					   &p3h2x4x_i3c_hub->hub_config.tp0145_io_strength);

			debugfs_create_u32("internal-pullups-ohms", 0400, tp_dir,
					   &p3h2x4x_i3c_hub->hub_config.tp0145_pullup);
		} else {
			debugfs_create_u32("io-strength-ohms", 0400, tp_dir,
					   &p3h2x4x_i3c_hub->hub_config.tp2367_io_strength);

			debugfs_create_u32("internal-pullups-ohms", 0400, tp_dir,
					   &p3h2x4x_i3c_hub->hub_config.tp2367_pullup);
		}
	}

	sysdir = debugfs_create_dir("reg", p3h2x4x_i3c_hub->sysfs_debug_dir);
	if (IS_ERR(sysdir))
		goto err_remove;

	reg_dir = sysdir;

	sysdir = debugfs_create_file_unsafe("access", 0600, reg_dir, p3h2x4x_i3c_hub,
					    &p3h2x4x_i3c_hub_access_reg);
	if (IS_ERR(sysdir))
		goto err_remove;

	debugfs_create_u8("offset", 0600, reg_dir, &p3h2x4x_i3c_hub->reg_addr);

	return 0;

err_remove:
	debugfs_remove_recursive(p3h2x4x_i3c_hub->sysfs_debug_dir);
	return PTR_ERR(sysdir);
}

static int p3h2x4x_i3c_hub_probe(struct platform_device *pdev)
{
	struct p3h2x4x_dev *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
	struct device *dev = &pdev->dev;
	struct device_node *node;
	char hub_id[64];
	int ret, i;

	p3h2x4x_i3c_hub = devm_kzalloc(dev, sizeof(*p3h2x4x_i3c_hub), GFP_KERNEL);
	if (!p3h2x4x_i3c_hub)
		return -ENOMEM;

	p3h2x4x_i3c_hub->regmap = p3h2x4x->regmap;
	p3h2x4x_i3c_hub->dev = dev;

	platform_set_drvdata(pdev, p3h2x4x_i3c_hub);

	p3h2x4x_default_configuration(dev);

	ret = devm_mutex_init(dev, &p3h2x4x_i3c_hub->etx_mutex);
	if (ret)
		return ret;

	for (i = 0; i < P3H2X4X_TP_MAX_COUNT; i++) {
		ret = devm_mutex_init(dev, &p3h2x4x_i3c_hub->tp_bus[i].port_mutex);
		if (ret)
			return ret;
	}

	/* get hub node from DT */
	node =  dev->parent->of_node;
	if (!node)
		return dev_err_probe(dev, -ENODEV, "No Device Tree entry found\n");

	p3h2x4x_get_hub_dt_conf(dev, node);
	p3h2x4x_get_target_port_dt_conf(dev, node);

	/* Unlock access to protected registers */
	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
			   P3H2X4X_REGISTERS_UNLOCK_CODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to unlock HUB's protected registers\n");

	ret = p3h2x4x_configure_hw(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to configure the HUB\n");

	/* Register logic for native vertual I3C ports */
	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		p3h2x4x_i3c_hub->i3cdev = p3h2x4x->i3cdev;
		i3cdev_set_drvdata(p3h2x4x->i3cdev, p3h2x4x_i3c_hub);
		sprintf(hub_id, "i3c-hub-device-%d-%llx",
			i3c_dev_get_master(p3h2x4x_i3c_hub->i3cdev->desc)->bus.id,
			p3h2x4x_i3c_hub->i3cdev->desc->info.pid);
		ret = p3h2x4x_tp_i3c_algo(p3h2x4x_i3c_hub);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to register i3c bus\n");
	} else {
		struct i2c_client *client = to_i2c_client(pdev->dev.parent);

		snprintf(hub_id, sizeof(hub_id), "i3c-hub-device-%d",
			 client->adapter->nr);
	}

	/* Register logic for native SMBus ports */
	ret = p3h2x4x_tp_smbus_algo(p3h2x4x_i3c_hub);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add i2c adapter\n");

	/* Lock access to protected registers */
	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
			   P3H2X4X_REGISTERS_LOCK_CODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to lock HUB's protected registers\n");

	ret = p3h2x4x_i3c_hub_debugfs_init(p3h2x4x_i3c_hub, hub_id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to create I3C HUB debugfs\n");

	return 0;
}

static void p3h2x4x_i3c_hub_remove(struct platform_device *pdev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = platform_get_drvdata(pdev);
	struct p3h2x4x_dev *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	u8 i;

	debugfs_remove_recursive(p3h2x4x_i3c_hub->sysfs_debug_dir);

	for (i = 0; i < P3H2X4X_TP_MAX_COUNT; i++) {
		if (!p3h2x4x_i3c_hub->tp_bus[i].is_registered)
			continue;

		if (p3h2x4x_i3c_hub->hub_config.tp_config[i].mode == P3H2X4X_TP_MODE_SMBUS)
			i2c_del_adapter(p3h2x4x_i3c_hub->tp_bus[i].tp_smbus_adapter);
		else if (p3h2x4x_i3c_hub->hub_config.tp_config[i].mode == P3H2X4X_TP_MODE_I3C)
			i3c_master_unregister(&p3h2x4x_i3c_hub->tp_bus[i]
					      .hub_controller.controller);
	}

	if (p3h2x4x->is_p3h2x4x_in_i3c) {
		i3c_device_disable_ibi(p3h2x4x->i3cdev);
		i3c_device_free_ibi(p3h2x4x->i3cdev);
	}
}

static struct platform_driver p3h2x4x_i3c_hub_driver = {
	.driver = {
		.name = "p3h2x4x-i3c-hub",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = p3h2x4x_i3c_hub_probe,
	.remove = p3h2x4x_i3c_hub_remove,
};
module_platform_driver(p3h2x4x_i3c_hub_driver);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("P3H2X4X I3C HUB driver");
MODULE_LICENSE("GPL");
