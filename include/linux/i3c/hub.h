/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2026 NXP
 * Generic hub definitions and helper interfaces.
 */
#ifndef _LINUX_I3C_HUB_H
#define _LINUX_I3C_HUB_H

#include <linux/i3c/master.h>

static inline struct i3c_master_controller *
i3c_hub_update_desc_parent(struct i3c_i2c_dev_desc *desc,
			   struct i3c_master_controller *parent)
{
	struct i3c_master_controller *orig_parent = desc->master;

	desc->master = parent;
	return orig_parent;
}

static inline void
i3c_hub_restore_desc_parent(struct i3c_i2c_dev_desc *desc,
			    struct i3c_master_controller *parent)
{
	desc->master = parent;
}

/**
 * struct i3c_hub - Generic I3C hub context
 * @parent: Parent I3C master controller
 * @ops: Vendor callbacks for port connection control
 * @hub_dev: I3C device representing the hub on the parent bus
 */
struct i3c_hub {
	struct i3c_master_controller *parent;
	const struct i3c_hub_ops *ops;
	struct i3c_device *hub_dev;
};

struct i3c_hub_controller {
	struct i3c_master_controller *parent;
	struct i3c_master_controller controller;
	struct i3c_hub *hub;
};

struct i3c_hub_ops {
	void (*enable_port)(struct i3c_master_controller *controller);
	void (*disable_port)(struct i3c_master_controller *controller);
};

/**
 * i3c_hub_enable_port() - Enable hub connection for a controller
 * @controller: Virtual controller representing a hub port
 *
 * Retrieves hub context from controller drvdata and invokes the vendor
 * callback to enable the associated port connection.
 */
static inline void i3c_hub_enable_port(struct i3c_master_controller *controller)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return;

	hub = hub_controller->hub;

	if (hub && hub->ops && hub->ops->enable_port)
		hub->ops->enable_port(controller);
}

/**
 * i3c_hub_disable_port() - Disable hub connection for a controller
 * @controller: Virtual controller representing a hub port
 *
 * Retrieves hub context from controller drvdata and invokes the vendor
 * callback to disable the associated port connection.
 */
static inline void i3c_hub_disable_port(struct i3c_master_controller *controller)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return;

	hub = hub_controller->hub;

	if (hub && hub->ops && hub->ops->disable_port)
		hub->ops->disable_port(controller);
}

/**
 * i3c_hub_master_ops() - Return virtual controller ops for hub ports
 *
 * Provides i3c_master_controller_ops used by controllers created for hub
 * ports.
 */
const struct i3c_master_controller_ops *i3c_hub_master_ops(void);

struct i3c_hub *i3c_hub_init(struct i3c_master_controller *parent,
			     const struct i3c_hub_ops *ops,
			     struct i3c_device *hub_dev);

#endif
