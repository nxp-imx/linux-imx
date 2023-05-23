/*
 * Copyright 2023 NXP
 *
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/display-rpmsg.h>

#include "watch_dial.c"

#define IMX_RPMSG_DISPLAY            (0x0BU)
#define RPMSG_LOW_POWER_DISPLAY_ID   (0x43211234)

#define RPMSG_USER_DATA_LENGTH       11   // 16-4-1

#define DISP_RPMSG_REQUEST          0x0
#define DISP_RPMSG_RESPONSE         0x1
#define DISP_RPMSG_NOTIFICATION     0x2

#define DISP_RETURN_CODE_SUCEESS     (0x0U)
#define DISP_RETURN_CODE_FAIL        (0x1U)
#define DISP_RETURN_CODE_UNSUPPORTED (0x2U)

#define RPMSG_TIMEOUT 1000

enum lpd_cmd {
	DISP_CMD_REGISTER = 0U,
	DISP_CMD_UNREGISTER,
	DISP_CMD_APD_POWER_STATE,
	DISP_CMD_DISPLAY_MODE,
	DISP_CMD_DISP_POWER_CTRL,
	DISP_CMD_BACKLIGHT_CTRL,
	DISP_CMD_APPLICATION_CMD,
};

enum lpd_disp_type {
	DISP_TYPE_APD = 0U,
	DISP_TYPE_RTD,
	DISP_TYPE_MIX,
};

enum lpd_backlight_type {
	DISP_BL_NONE = 0U,
	DISP_BL_APD,
	DISP_BL_RTD,
};

struct lpd_rpmsg_data {
	struct imx_rpmsg_head header;
	u32 displayId;
	union {
		u8 reserved1;
		u8 retCode;
	};
	union {
		u8 powerState;
		struct displayMode {
			u16 width;
			u16 height;
			u16 stride;
			u16 format;
			u16 modifier;
		} mode;
		struct _register {
			u8 type;
			u8 bl_ctrl;
			u32 shared_buf;
		} reg;
		struct _app_cmd {
			u8 cmd;
			u8 param;
		} app;
		u8 power_on;
		u8 unreg_reason;
		u8 buf[RPMSG_USER_DATA_LENGTH];
	};
} __attribute__ ((__packed__));

struct lpd_shared_mem {
	void __iomem *vaddr;
	phys_addr_t paddr;
	size_t size;
};

struct lpd_drm_callbacks {
	lpd_drm_power_t pm;
	lpd_drm_application_t app;
};

struct lpd_rpmsg_drvdata {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct device *drm_dev;
	struct lpd_shared_mem mem;
	struct lpd_drm_callbacks cbs;
	struct lpd_rpmsg_data *msg;
	int bl_ctrl;
	struct completion cmd_complete;
	struct mutex lock;
};

static struct lpd_rpmsg_drvdata *lpd_rpmsg;

static int rpmsg_display_notifier(struct notifier_block *nb, unsigned long action, void *unused)
{
	int ret;
	u8 state;
	struct lpd_rpmsg_data msg = {};

	if (action == SYS_RESTART)
		state = DISP_APD_REBOOT;
	else
		state = DISP_APD_SHUTDOWN;

	msg.header.cate = IMX_RPMSG_DISPLAY;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = DISP_RPMSG_REQUEST;
	msg.header.cmd = DISP_CMD_APD_POWER_STATE;

	msg.displayId = RPMSG_LOW_POWER_DISPLAY_ID;
	msg.powerState = state;

	/* No ACK from M core */
	ret = rpmsg_send(lpd_rpmsg->rpdev->ept, &msg, sizeof(struct lpd_rpmsg_data));

	if (ret) {
		pr_info("rpmsg send failed:%d\n", ret);
		return NOTIFY_BAD;
	}

	return NOTIFY_DONE;
};

static struct notifier_block rpmsg_display_nb = {
	.notifier_call = rpmsg_display_notifier,
};

static int lpd_rpmsg_send_request(u8 cmd, struct lpd_rpmsg_data *msg)
{
        int ret;

        msg->header.cate = IMX_RPMSG_DISPLAY;
        msg->header.major = IMX_RMPSG_MAJOR;
        msg->header.minor = IMX_RMPSG_MINOR;
        msg->header.type = DISP_RPMSG_REQUEST;
        msg->header.cmd = cmd;
        msg->displayId = RPMSG_LOW_POWER_DISPLAY_ID;

        reinit_completion(&lpd_rpmsg->cmd_complete);

        ret = rpmsg_send(lpd_rpmsg->rpdev->ept, msg, sizeof(struct lpd_rpmsg_data));
        if (ret) {
                dev_err(&lpd_rpmsg->rpdev->dev, "rpmsg send failed:%d\n", ret);
                return ret;
        }

        ret = wait_for_completion_timeout(&lpd_rpmsg->cmd_complete,
                                        msecs_to_jiffies(RPMSG_TIMEOUT));
        if (!ret) {
                dev_err(&lpd_rpmsg->rpdev->dev, "rpmsg_send timeout!\n");
                return -ETIMEDOUT;
        }

        return 0;
}

int lpd_display_register(struct device *dev, bool enable)
{
        struct lpd_rpmsg_data msg = {};

	if (lpd_rpmsg == NULL) {
		pr_err("%s: lpd_rpmsg is not initialized!\n", __func__);
		return -1;
	}
	lpd_rpmsg->drm_dev = dev;

	if (enable) {
	        msg.reg.type = DISP_TYPE_APD;
	        msg.reg.bl_ctrl = lpd_rpmsg->bl_ctrl;
		msg.reg.shared_buf = (uint32_t)(lpd_rpmsg->mem.paddr);
	        lpd_rpmsg_send_request(DISP_CMD_REGISTER, &msg);
	} else {
                msg.unreg_reason = 0; //TBD
                lpd_rpmsg_send_request(DISP_CMD_UNREGISTER, &msg);
	}

        return 0;
}
EXPORT_SYMBOL_GPL(lpd_display_register);

int lpd_api_register(int func_id, void *cb)
{
	switch (func_id) {
	case LPD_DRM_CALLBACK_PM:
		lpd_rpmsg->cbs.pm = (lpd_drm_power_t)cb;
		break;
	case LPD_DRM_CALLBACK_APP:
		lpd_rpmsg->cbs.app = (lpd_drm_application_t)cb;
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(lpd_api_register);

static int lpd_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct lpd_rpmsg_data *msg = (struct lpd_rpmsg_data *)data;

	if (msg->header.type == DISP_RPMSG_RESPONSE) { // response from MCU
		lpd_rpmsg->msg = msg;
		complete(&lpd_rpmsg->cmd_complete);
		switch (msg->header.cmd) {
		case DISP_CMD_REGISTER:
			if (msg->retCode != DISP_RETURN_CODE_SUCEESS)
				dev_err(&lpd_rpmsg->rpdev->dev, "low power display register failed\n");
			else
				dev_info(&lpd_rpmsg->rpdev->dev, "low power display register successfully\n");
			break;
                case DISP_CMD_UNREGISTER:
                        break;
                case DISP_CMD_APD_POWER_STATE:
                        break;
		default:
			break;
		}
	} else if (msg->header.type == DISP_RPMSG_NOTIFICATION) { // notification from MCU
		lpd_rpmsg->msg = msg;
		dev_info(&lpd_rpmsg->rpdev->dev, "Notification from MCU\n");
	} else if (msg->header.type == DISP_RPMSG_REQUEST) { // request from MCU
		dev_info(&lpd_rpmsg->rpdev->dev, "Request from MCU: %d\n", msg->header.cmd);
		switch (msg->header.cmd) {
		case DISP_CMD_DISP_POWER_CTRL:
			if (lpd_rpmsg->cbs.pm != NULL) {
				if (msg->power_on == LPD_DEVICE_POWER_ON || msg->power_on == LPD_DEVICE_POWER_OFF) {
					lpd_rpmsg->cbs.pm(lpd_rpmsg->drm_dev, msg->power_on);
				} else {
					dev_info(&lpd_rpmsg->rpdev->dev, "power control command is invalid\n");
				}
			}
			break;
		case DISP_CMD_BACKLIGHT_CTRL:
			break;
		case DISP_CMD_APPLICATION_CMD:
			if (lpd_rpmsg->cbs.app != NULL) {
				int value = msg->app.param;
				lpd_rpmsg->cbs.app(lpd_rpmsg->drm_dev, msg->app.cmd, (void*)&value);
				dev_info(&lpd_rpmsg->rpdev->dev, "application command=%d, %d\n", msg->app.cmd, msg->app.param);
			}
			break;
		default:
			break;
		}
	}

	return 0;
}

static void lpd_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "low power display rpmsg driver is removed\n");
}

static int lpd_rpmsg_probe(struct rpmsg_device *rpdev)
{
	uint32_t *buf, *data;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n", rpdev->src, rpdev->dst);

	lpd_rpmsg->rpdev = rpdev;
	mutex_init(&lpd_rpmsg->lock);
	init_completion(&lpd_rpmsg->cmd_complete);

	buf = (uint32_t *)lpd_rpmsg->mem.vaddr;
	data = (uint32_t *)gimp_image_dial.pixel_data;
	memcpy(buf, data, gimp_image_dial.height * gimp_image_dial.width * gimp_image_dial.bytes_per_pixel);

	return register_reboot_notifier(&rpmsg_display_nb);;
}

static struct rpmsg_device_id lpd_rpmsg_id_table[] = {
	{ .name	= "rpmsg-display-channel" },
	{ },
};

static struct rpmsg_driver rpmsg_lpd_driver = {
	.drv.name	= "display-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= lpd_rpmsg_id_table,
	.probe		= lpd_rpmsg_probe,
	.callback	= lpd_rpmsg_cb,
	.remove		= lpd_rpmsg_remove,
};

int lpd_update_display_mode(u16 width, u16 height, u16 format, u16 modifier)
{
	struct lpd_rpmsg_data msg = {};

	msg.mode.width = width;
	msg.mode.height = height;
	msg.mode.stride = 0;
	msg.mode.format = format;
	msg.mode.modifier = modifier;
	lpd_rpmsg_send_request(DISP_CMD_DISPLAY_MODE, &msg);

	return 0;
}
EXPORT_SYMBOL_GPL(lpd_update_display_mode);

int lpd_send_power_state(unsigned char state)
{
	struct lpd_rpmsg_data msg = {};

	if ((state != DISP_APD_SUSPEND) && (state != DISP_APD_RESUME)
		&& (state != DISP_APD_BLANK) && (state != DISP_APD_UNBLANK))
		return -EBADRQC;

	msg.powerState = state;
	lpd_rpmsg_send_request(DISP_CMD_APD_POWER_STATE, &msg);

	return 0;
}
EXPORT_SYMBOL_GPL(lpd_send_power_state);

static int __maybe_unused low_power_display_suspend_noirq(struct device *dev)
{
	return lpd_send_power_state(DISP_APD_SUSPEND);
}

static int __maybe_unused low_power_display_resume_noirq(struct device *dev)
{
	return lpd_send_power_state(DISP_APD_RESUME);;
}

static const struct dev_pm_ops low_power_display_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(low_power_display_suspend_noirq,
			low_power_display_resume_noirq)
};

static int low_power_display_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lpd_rpmsg_drvdata *drvdata;
	struct device_node *node;
	struct resource res;
	int ret;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	node = of_parse_phandle(np, "memory-region", 0);
	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		dev_err(dev, "unable to resolve memory region\n");
		return ret;
	}
	drvdata->mem.vaddr = devm_ioremap_wc(&pdev->dev, res.start, resource_size(&res));
	drvdata->mem.paddr = res.start;
	drvdata->mem.size = resource_size(&res);

	lpd_rpmsg = drvdata;
	lpd_rpmsg->dev = dev;
	platform_set_drvdata(pdev, drvdata);
	ret = of_property_read_u32(pdev->dev.of_node, "backlight-ctrl",	&lpd_rpmsg->bl_ctrl);
	if (ret)
		lpd_rpmsg->bl_ctrl = 0;

	return register_rpmsg_driver(&rpmsg_lpd_driver);
}

static const struct of_device_id low_power_display_id[] = {
	{ "nxp,low-power-display", },
	{},
};
MODULE_DEVICE_TABLE(of, low_power_display_id);

static struct platform_driver low_power_display_driver = {
	.driver = {
		.name = "low-power-display",
		.owner = THIS_MODULE,
		.of_match_table = low_power_display_id,
		.pm = &low_power_display_pm_ops,
	},
	.probe = low_power_display_probe,
};
module_platform_driver(low_power_display_driver);

MODULE_AUTHOR("Zhang Bo <bo.zhang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX RPMSG Low Power Display Driver");
MODULE_LICENSE("GPL");
