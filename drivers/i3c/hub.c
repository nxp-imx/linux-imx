// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 * Generic I3C Hub core implementing virtual controller operations.
 */
#include <linux/i3c/device.h>
#include <linux/i3c/hub.h>

#include "internals.h"

/**
 * i3c_hub_master_bus_init() - Bind controller to hub device
 * @controller: Virtual controller for a hub port
 *
 * Associates the virtual controller with the hub device descriptor so that
 * transfers are executed through the hub on the parent bus.
 */
static int i3c_hub_master_bus_init(struct i3c_master_controller *controller)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->hub_dev)
		return -ENODEV;

	controller->this = hub->hub_dev->desc;
	return 0;
}

static void i3c_hub_master_bus_cleanup(struct i3c_master_controller *controller)
{
	controller->this = NULL;
}

static int i3c_hub_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	return 0;
}

static int i3c_hub_reattach_i3c_dev(struct i3c_dev_desc *dev, u8 old_dyn_addr)
{
	return 0;
}

static void i3c_hub_detach_i3c_dev(struct i3c_dev_desc *dev)
{
}

/**
 * i3c_hub_do_daa() - Perform DAA via hub port
 * @hub: Hub instance
 * @controller: Virtual controller for a hub port
 *
 * Enables the port connection, performs DAA on the parent controller,
 * then disables the connection.
 */
static int i3c_hub_do_daa(struct i3c_hub *hub,
			  struct i3c_master_controller *controller)
{
	int ret;

	if (!hub || !hub->parent)
		return -ENODEV;

	i3c_hub_enable_port(controller);
	ret = i3c_master_do_daa(hub->parent);
	i3c_hub_disable_port(controller);

	return ret;
}

static bool i3c_hub_supports_ccc_cmd(struct i3c_hub *hub,
				     const struct i3c_ccc_cmd *cmd)
{
	return i3c_master_supports_ccc_cmd(hub->parent, cmd);
}

/**
 * i3c_hub_send_ccc_cmd() - Send CCC through hub port
 * @hub: Hub instance
 * @controller: Virtual controller
 * @cmd: CCC command
 *
 * Enables the port connection while issuing CCC on the parent controller.
 */
static int i3c_hub_send_ccc_cmd(struct i3c_hub *hub,
				struct i3c_master_controller *controller,
				struct i3c_ccc_cmd *cmd)
{
	int ret;

	if (!hub || !hub->parent)
		return -ENODEV;

	i3c_hub_enable_port(controller);
	ret = i3c_master_send_ccc_cmd(hub->parent, cmd);
	i3c_hub_disable_port(controller);

	return ret;
}

/**
 * i3c_hub_master_priv_xfers() - Execute private transfers via hub
 * @dev: Target device descriptor
 * @xfers: Transfer array
 * @nxfers: Number of transfers
 * @mode: transfer mode (SDR, HDR, etc.)
 *
 * Handles address adjustment and forwards private transfers through the hub
 * device.
 */
static int i3c_hub_master_priv_xfers(struct i3c_dev_desc *dev,
				     struct i3c_priv_xfer *xfers,
				     int nxfers)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(dev);
	struct i3c_hub_controller *hub_controller;
	struct i3c_dev_desc *hub_dev;
	u8 hub_addr, target_addr;
	struct i3c_hub *hub;
	int ret;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->hub_dev)
		return -ENODEV;

	hub_dev = hub->hub_dev->desc;

	i3c_hub_enable_port(controller);

	hub_addr = hub_dev->info.dyn_addr ?
		   hub_dev->info.dyn_addr : hub_dev->info.static_addr;

	target_addr = dev->info.dyn_addr ?
		      dev->info.dyn_addr : dev->info.static_addr;

	if (hub_addr != target_addr) {
		hub_dev->info.dyn_addr = target_addr;
		ret = i3c_master_reattach_i3c_dev(hub_dev, target_addr);
		if (ret)
			goto disable;
	}

	ret = i3c_device_do_priv_xfers(hub->hub_dev, xfers, nxfers);

	if (hub_addr != target_addr) {
		hub_dev->info.dyn_addr = hub_addr;
		ret |= i3c_master_reattach_i3c_dev(hub_dev, hub_addr);
	}

disable:
	i3c_hub_disable_port(controller);
	return ret;
}

static int i3c_hub_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	return 0;
}

static void i3c_hub_detach_i2c_dev(struct i2c_dev_desc *dev)
{
}

static int i3c_hub_i2c_xfers(struct i2c_dev_desc *dev,
			     struct i2c_msg *xfers, int nxfers)
{
	return 0;
}

static int i3c_hub_master_do_daa(struct i3c_master_controller *controller)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	return i3c_hub_do_daa(hub, controller);
}

static int i3c_hub_master_send_ccc_cmd(struct i3c_master_controller *controller,
				       struct i3c_ccc_cmd *cmd)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->parent)
		return -ENODEV;

	if (cmd->id == I3C_CCC_RSTDAA(true))
		return 0;

	return i3c_hub_send_ccc_cmd(hub, controller, cmd);
}

static bool i3c_hub_master_supports_ccc_cmd(struct i3c_master_controller *controller,
					    const struct i3c_ccc_cmd *cmd)
{
	struct i3c_hub_controller *hub_controller;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return false;

	hub = hub_controller->hub;

	return i3c_hub_supports_ccc_cmd(hub, cmd);
}

/**
 * i3c_hub_request_ibi() - Request IBI through parent controller
 * @desc: Target device descriptor
 * @req: IBI setup
 *
 * Temporarily updates parent controller context to request IBI for a device
 * connected through the hub.
 */
static int i3c_hub_request_ibi(struct i3c_dev_desc *desc,
			       const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);
	struct i3c_hub_controller *hub_controller;
	struct i3c_master_controller *orig_parent;
	struct i3c_master_controller *parent;
	struct i3c_hub *hub;
	int ret;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->parent)
		return -ENODEV;

	parent = hub->parent;

	orig_parent = i3c_hub_update_desc_parent(&desc->common, parent);

	ret = i3c_master_direct_attach_i3c_dev(parent, desc);
	if (ret) {
		i3c_hub_restore_desc_parent(&desc->common, orig_parent);
		return ret;
	}

	mutex_unlock(&desc->ibi_lock);
	kfree(desc->ibi);
	desc->ibi = NULL;
	ret = i3c_dev_request_ibi_locked(desc, req);
	mutex_lock(&desc->ibi_lock);

	i3c_hub_restore_desc_parent(&desc->common, orig_parent);

	return ret;
}

static void i3c_hub_free_ibi(struct i3c_dev_desc *desc)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);
	struct i3c_hub_controller *hub_controller;
	struct i3c_master_controller *orig_parent;
	struct i3c_master_controller *parent;
	struct i3c_hub *hub;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return;

	hub = hub_controller->hub;

	parent = hub->parent;

	i3c_hub_enable_port(controller);

	orig_parent = i3c_hub_update_desc_parent(&desc->common, parent);
	i3c_master_direct_detach_i3c_dev(desc);
	mutex_unlock(&desc->ibi_lock);
	i3c_dev_free_ibi_locked(desc);
	mutex_lock(&desc->ibi_lock);
	i3c_hub_restore_desc_parent(&desc->common, orig_parent);

	i3c_hub_disable_port(controller);
}

/**
 * i3c_hub_enable_ibi() - Enable IBI via hub port
 * @desc: Target device descriptor
 *
 * Enables port connection and forwards the IBI enable request to the parent
 * controller.
 */
static int i3c_hub_enable_ibi(struct i3c_dev_desc *desc)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);
	struct i3c_hub_controller *hub_controller;
	struct i3c_master_controller *orig_parent;
	struct i3c_master_controller *parent;
	struct i3c_hub *hub;
	int ret;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->parent)
		return -ENODEV;

	parent = hub->parent;

	i3c_hub_enable_port(controller);

	orig_parent = i3c_hub_update_desc_parent(&desc->common, parent);

	down_write(&parent->bus.lock);
	mutex_unlock(&desc->ibi_lock);
	ret = i3c_dev_enable_ibi_locked(desc);
	mutex_lock(&desc->ibi_lock);
	up_write(&parent->bus.lock);

	i3c_hub_restore_desc_parent(&desc->common, orig_parent);

	i3c_hub_disable_port(controller);

	return ret;
}

/**
 * i3c_hub_disable_ibi() - Disable IBI via hub port
 * @desc: Target device descriptor
 *
 * Enables port connection and forwards the IBI disable request to the parent
 * controller.
 */
static int i3c_hub_disable_ibi(struct i3c_dev_desc *desc)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);
	struct i3c_hub_controller *hub_controller;
	struct i3c_master_controller *orig_parent;
	struct i3c_master_controller *parent;
	struct i3c_hub *hub;
	int ret;

	hub_controller = dev_get_drvdata(&controller->dev);
	if (!hub_controller || !hub_controller->hub)
		return -ENODEV;

	hub = hub_controller->hub;

	if (!hub->parent)
		return -ENODEV;

	parent = hub->parent;

	i3c_hub_enable_port(controller);

	orig_parent = i3c_hub_update_desc_parent(&desc->common, parent);

	down_write(&parent->bus.lock);
	mutex_unlock(&desc->ibi_lock);
	ret = i3c_dev_disable_ibi_locked(desc);
	mutex_lock(&desc->ibi_lock);
	up_write(&parent->bus.lock);

	i3c_hub_restore_desc_parent(&desc->common, orig_parent);

	i3c_hub_disable_port(controller);

	return ret;
}

static void i3c_hub_recycle_ibi_slot(struct i3c_dev_desc *desc,
				     struct i3c_ibi_slot *slot)
{
}

static const struct i3c_master_controller_ops i3c_hub_master_ops_data = {
	.bus_init = i3c_hub_master_bus_init,
	.bus_cleanup = i3c_hub_master_bus_cleanup,
	.attach_i3c_dev = i3c_hub_attach_i3c_dev,
	.reattach_i3c_dev = i3c_hub_reattach_i3c_dev,
	.detach_i3c_dev = i3c_hub_detach_i3c_dev,
	.do_daa = i3c_hub_master_do_daa,
	.supports_ccc_cmd = i3c_hub_master_supports_ccc_cmd,
	.send_ccc_cmd = i3c_hub_master_send_ccc_cmd,
	.priv_xfers = i3c_hub_master_priv_xfers,
	.attach_i2c_dev = i3c_hub_attach_i2c_dev,
	.detach_i2c_dev = i3c_hub_detach_i2c_dev,
	.i2c_xfers = i3c_hub_i2c_xfers,
	.request_ibi = i3c_hub_request_ibi,
	.free_ibi = i3c_hub_free_ibi,
	.enable_ibi = i3c_hub_enable_ibi,
	.disable_ibi = i3c_hub_disable_ibi,
	.recycle_ibi_slot = i3c_hub_recycle_ibi_slot,
};

/**
 * i3c_hub_init() - Initialize hub context
 * @hub: Hub instance
 * @parent: Parent I3C master controller
 * @ops: Vendor callbacks
 * @hub_dev: I3C hub device
 */
struct i3c_hub *i3c_hub_init(struct i3c_master_controller *parent,
			     const struct i3c_hub_ops *ops,
			     struct i3c_device *hub_dev)
{
	struct i3c_hub *hub;

	hub = devm_kzalloc(&hub_dev->dev,
			   sizeof(*hub),
			   GFP_KERNEL);

	if (!hub)
		return ERR_PTR(-ENOMEM);

	hub->parent = parent;
	hub->ops = ops;
	hub->hub_dev = hub_dev;

	return hub;
}
EXPORT_SYMBOL_GPL(i3c_hub_init);

const struct i3c_master_controller_ops *i3c_hub_master_ops(void)
{
	return &i3c_hub_master_ops_data;
}
EXPORT_SYMBOL_GPL(i3c_hub_master_ops);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("Generic I3C hub support");
MODULE_LICENSE("GPL");
