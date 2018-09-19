/*
 * Copyright 2018 NXP
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
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/imx_rpmsg.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_domain.h>
#include <linux/rpmsg.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <soc/imx8/sc/sci.h>

#define RPMSG_TIMEOUT 1000
#define REGISTER_PERIOD 1000

enum can_cmd {
	CAN_RPMSG_REGISTER = 0,
	CAN_RPMSG_UNREGISTER,
	CAN_RPMSG_EVENT,
};

enum can_cmd_type {
	CAN_RPMSG_REQUEST = 0,
	CAN_RPMSG_RESPONSE,
	CAN_RPMSG_NOTIFICATION,
};

enum can_event_type {
	CAN_REVERSE = 0,
	CAN_FORWORD,
};

struct can_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 index;
	union {
		u8 reserved1;
		u8 retcode;
	};
	union {
		u32 reserved2;
		u32 event;
	};
	u32 user;
} __attribute__((packed));


struct rpmsg_can_drvdata {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct can_rpmsg_data *msg;
	struct pm_qos_request pm_qos_req;
	struct delayed_work can_register_work;
	struct completion cmd_complete;
};

#ifdef CONFIG_EXTCON
static const unsigned int imx_can_rpmsg_extcon_register_cables[] = {
	EXTCON_CAN_RPMSG_REGISTER,
	EXTCON_NONE,
};
static const unsigned int imx_can_rpmsg_extcon_event_cables[] = {
	EXTCON_CAN_RPMSG_EVENT,
	EXTCON_NONE,
};
struct extcon_dev *rg_edev;
struct extcon_dev *ev_edev;
#endif

static struct rpmsg_can_drvdata *can_rpmsg;
static struct class* can_rpmsg_class;
int state = 0;

/*send can message to M4 core through rpmsg*/
static int can_send_message(struct can_rpmsg_data *msg,
			struct rpmsg_can_drvdata *info, bool ack)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	pm_qos_add_request(&info->pm_qos_req,
			PM_QOS_CPU_DMA_LATENCY, 0);

	if (ack) {
		reinit_completion(&info->cmd_complete);
	}

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct can_rpmsg_data));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	if (ack) {
		err = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(RPMSG_TIMEOUT));
		if (!err) {
			dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
			err = -ETIMEDOUT;
			goto err_out;
		}

		if (info->msg->retcode != 0) {
			dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
				info->msg->retcode);
			err = -EINVAL;
			goto err_out;
		}

		err = 0;
	}

err_out:
	pm_qos_remove_request(&info->pm_qos_req);

	return err;
}

void open_power_domain(void)
{
	sc_ipc_t ipcHndl;
	sc_err_t sciErr;
	uint32_t mu_id;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("Cannot obtain MU ID\n");
		return;
	}

	sciErr = sc_ipc_open(&ipcHndl, mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("sc_ipc_open failed! (sciError = %d)\n", sciErr);
		return;
	}

	/*power on display/camera related power domain when A core take over control*/
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_LVDS_1, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_LVDS_1_I2C_0, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_DC_1, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_DC_1_PLL_1, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_CSI_0_I2C_0, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_ISI_CH0, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_ISI_CH1, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_ISI_CH2, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_ISI_CH3, SC_PM_PW_MODE_ON);
	sciErr = sc_pm_set_resource_power_mode(ipcHndl, SC_R_CSI_0, SC_PM_PW_MODE_ON);
}

/*can_rpmsg_cb is called once get rpmsg from M4*/
static int can_rpmsg_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct can_rpmsg_data *msg = (struct can_rpmsg_data *)data;

	can_rpmsg->msg = msg;
	if (msg->header.cmd == CAN_RPMSG_REGISTER) {
		complete(&can_rpmsg->cmd_complete);
		if (msg->retcode == 0)
			open_power_domain();
#ifdef CONFIG_EXTCON
		if (msg->retcode == 0)
			extcon_set_state_sync(rg_edev, EXTCON_CAN_RPMSG_REGISTER, 1);
#endif
	} else if (msg->header.cmd == CAN_RPMSG_UNREGISTER) {
		complete(&can_rpmsg->cmd_complete);
#ifdef CONFIG_EXTCON
		if (msg->retcode == 0)
			extcon_set_state_sync(rg_edev, EXTCON_CAN_RPMSG_REGISTER, 0);
#endif
	} else if (msg->header.cmd == CAN_RPMSG_EVENT) {
		msg->header.type = CAN_RPMSG_RESPONSE;
		msg->retcode = 0;
#ifdef CONFIG_EXTCON
		if(msg->event == CAN_FORWORD)
			 extcon_set_state_sync(ev_edev, EXTCON_CAN_RPMSG_EVENT, 0);
		else
			 extcon_set_state_sync(ev_edev, EXTCON_CAN_RPMSG_EVENT, 1);
#endif
		if (can_send_message(msg, can_rpmsg, false))
			dev_warn(&rpdev->dev, "can_rpmsg_cb send message error \n");
	}

	return 0;
}

/* register can event to M4 core
 * M4 will release camera/display once register successfully.
 */
static void can_init_handler(struct work_struct *work)
{
	struct can_rpmsg_data msg;

	msg.header.cate = IMX_RPMSG_CAN;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = 0;
	msg.header.cmd = CAN_RPMSG_REGISTER;
	msg.index = 0;
	msg.reserved1 = 0;
	msg.reserved2 = 0;
	msg.user = 0;
	while (can_send_message(&msg, can_rpmsg, true)) {
		msleep(REGISTER_PERIOD);
	}
}

static int can_rpmsg_probe(struct rpmsg_device *rpdev)
{
#ifdef CONFIG_EXTCON
	int ret = 0;
	rg_edev = devm_extcon_dev_allocate(&rpdev->dev, imx_can_rpmsg_extcon_register_cables);
	if (IS_ERR(rg_edev)) {
		dev_err(&rpdev->dev, "failed to allocate extcon device\n");
	}
	ret = devm_extcon_dev_register(&rpdev->dev,rg_edev);
	if (ret < 0) {
		dev_err(&rpdev->dev, "failed to register extcon device\n");
	}
	ev_edev = devm_extcon_dev_allocate(&rpdev->dev, imx_can_rpmsg_extcon_event_cables);
	if (IS_ERR(ev_edev)) {
		dev_err(&rpdev->dev, "failed to allocate extcon device\n");
	}
	ret = devm_extcon_dev_register(&rpdev->dev,ev_edev);
	if (ret < 0) {
		dev_err(&rpdev->dev, "failed to register extcon device\n");
	}

#endif
	can_rpmsg->rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&can_rpmsg->cmd_complete);

	INIT_DELAYED_WORK(&can_rpmsg->can_register_work,
				can_init_handler);
	schedule_delayed_work(&can_rpmsg->can_register_work, 0);

	return 0;
}

static struct rpmsg_device_id can_rpmsg_id_table[] = {
	{ .name	= "rpmsg-can-channel" },
	{ },
};

static struct rpmsg_driver can_rpmsg_driver = {
	.drv.name       = "can_rpmsg",
	.drv.owner      = THIS_MODULE,
	.id_table       = can_rpmsg_id_table,
	.probe          = can_rpmsg_probe,
	.callback       = can_rpmsg_cb,
};

static ssize_t register_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%u\n", state);
}

/*register can to M4 once echo 1 > /sys/devices/platform/can_rpmsg/register*/
static ssize_t register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t size)
{
	size_t err;

	if (!size)
		return -EINVAL;

	unsigned long state_set = simple_strtoul(buf, NULL, 10);
	if (state_set == 1 && state_set != state) {
		err = register_rpmsg_driver(&can_rpmsg_driver);
		if (err < 0) {
			pr_err("register rpmsg driver failed\n");
			return -EINVAL;
		}
		/*detach the power domain pd_dc0*/
		dev_pm_domain_detach(dev, false);
		state = state_set;
	}
	return size;
}

static DEVICE_ATTR(register, 0664, register_show, register_store);

static struct rpmsg_can_drvdata *
rpmsg_can_get_devtree_pdata(struct device *dev)
{
	struct rpmsg_can_drvdata *ddata;

	ddata = devm_kzalloc(dev,
			     sizeof(*ddata),
			     GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	return ddata;
}
static int virtual_can_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmsg_can_drvdata *ddata;
	int err;

	ddata = rpmsg_can_get_devtree_pdata(dev);
	if (IS_ERR(ddata))
		return PTR_ERR(ddata);

	can_rpmsg = ddata;
	platform_set_drvdata(pdev, ddata);

	can_rpmsg_class = class_create(THIS_MODULE, "can_rpmsg");
	if (IS_ERR(can_rpmsg_class)) {
		dev_err(dev, "failed to create class.\n");
		return PTR_ERR(can_rpmsg_class);
	}
	err = device_create_file(dev, &dev_attr_register);
	if (err)
		return err;
	return 0;
}

static void virtual_can_remove(struct platform_device *pdev)
{
	class_destroy(can_rpmsg_class);
	unregister_rpmsg_driver(&can_rpmsg_driver);
}

static const struct of_device_id imx_can_rpmsg_id[] = {
	{ .compatible = "nxp,imx-can-rpmsg", },
	{},
};

static struct platform_driver virtual_can_device_driver = {
	.probe          = virtual_can_probe,
	.remove         = virtual_can_remove,
	.driver         =  {
		.name   = "rpmsg-can",
		.of_match_table = imx_can_rpmsg_id,
	}
};

static int rpmsg_can_init(void)
{
	int err;
	err = platform_driver_register(&virtual_can_device_driver);
	if (err) {
		pr_err("Failed to register rpmsg can driver\n");
		return err;
	}
	return 0;
}

static void __exit rpmsg_can_exit(void)
{
	platform_driver_unregister(&virtual_can_device_driver);
}

core_initcall(rpmsg_can_init);
module_exit(rpmsg_can_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("CAN driver based on rpmsg");
