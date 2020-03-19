// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP.
 */

#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/firmware/imx/sci.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define IMX_SC_MISC_FUNC_GET_TEMP	13
#define IMX_SC_C_TEMP			0
#define IMX_SC_TEMP_PASSIVE_COOL_DELTA	10000

static struct imx_sc_ipc *thermal_ipc_handle;

struct imx_sc_sensor {
	struct thermal_zone_device *tzd;
	u32 resource_id;
	struct thermal_cooling_device *cdev;
	int temp_passive;
	int temp_critical;
};

struct imx_sc_thermal_data {
	struct imx_sc_sensor *sensor;
};

/* The driver support 1 passive trip point and 1 critical trip point */
enum imx_thermal_trip {
	IMX_TRIP_PASSIVE,
	IMX_TRIP_CRITICAL,
	IMX_TRIP_NUM,
};

struct req_get_temp {
	u16 resource_id;
	u8 type;
} __packed __aligned(4);

struct resp_get_temp {
	s16 celsius;
	s8 tenths;
} __packed __aligned(4);

struct imx_sc_msg_misc_get_temp {
	struct imx_sc_rpc_msg hdr;
	union {
		struct req_get_temp req;
		struct resp_get_temp resp;
	} data;
} __packed __aligned(4);

static int imx_sc_thermal_get_temp(void *data, int *temp)
{
	struct imx_sc_msg_misc_get_temp msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	struct imx_sc_sensor *sensor = data;
	int ret;

	msg.data.req.resource_id = sensor->resource_id;
	msg.data.req.type = IMX_SC_C_TEMP;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_MISC;
	hdr->func = IMX_SC_MISC_FUNC_GET_TEMP;
	hdr->size = 2;

	ret = imx_scu_call_rpc(thermal_ipc_handle, &msg, true);
	if (ret) {
		/*
		 * if the SS power domain is down, read temp will fail, so
		 * we can print error once and return 0 directly.
		 */
		pr_err_once("read temp sensor %d failed, could be SS NOT powered up,\
			     return 0 for this thermal zone, ret %d\n",
			     sensor->resource_id, ret);
		*temp = 0;
		return 0;
	}

	*temp = msg.data.resp.celsius * 1000 + msg.data.resp.tenths * 100;

	return 0;
}

static int imx_sc_thermal_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	int trip_temp;
	struct imx_sc_sensor *sensor = p;

	if (!sensor->tzd)
		return 0;

	trip_temp = (trip == IMX_TRIP_PASSIVE) ? sensor->temp_passive :
					     sensor->temp_critical;

	if (sensor->tzd->temperature >=
		(trip_temp - IMX_SC_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int imx_sc_thermal_set_trip_temp(void *p, int trip, int temp)
{
	struct imx_sc_sensor *sensor = p;

	if (trip == IMX_TRIP_CRITICAL)
		sensor->temp_critical = temp;

	if (trip == IMX_TRIP_PASSIVE)
		sensor->temp_passive = temp;

	return 0;
}

static const struct thermal_zone_of_device_ops imx_sc_thermal_ops = {
	.get_temp = imx_sc_thermal_get_temp,
	.get_trend = imx_sc_thermal_get_trend,
	.set_trip_temp = imx_sc_thermal_set_trip_temp,
};

static int imx_sc_thermal_register_sensor(struct platform_device *pdev,
					  struct imx_sc_sensor *sensor)
{
	struct thermal_zone_device *tzd;

	tzd = devm_thermal_zone_of_sensor_register(&pdev->dev,
						   sensor->resource_id,
						   sensor,
						   &imx_sc_thermal_ops);
	if (IS_ERR(tzd)) {
		dev_err(&pdev->dev, "failed to register sensor: %d\n",
			sensor->resource_id);
		return PTR_ERR(tzd);
	}

	sensor->tzd = tzd;

	return 0;
}

static int imx_sc_thermal_get_sensor_id(struct device_node *sensor_np, u32 *id)
{
	struct of_phandle_args sensor_specs;
	int ret;

	ret = of_parse_phandle_with_args(sensor_np, "thermal-sensors",
			"#thermal-sensor-cells",
			0, &sensor_specs);
	if (ret)
		return ret;

	if (sensor_specs.args_count >= 1) {
		*id = sensor_specs.args[0];
		WARN(sensor_specs.args_count > 1,
				"%pOFn: too many cells in sensor specifier %d\n",
				sensor_specs.np, sensor_specs.args_count);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int imx_sc_thermal_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *sensor_np = NULL;
	struct imx_sc_thermal_data *data;
	struct imx_sc_sensor *sensors;
	const struct thermal_trip *trip;
	u32 sensor_num;
	int ret, i;

	ret = imx_scu_get_handle(&thermal_ipc_handle);
	if (ret)
		return ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = of_property_read_u32(np, "tsens-num", &sensor_num);
	if (ret || !sensor_num) {
		dev_err(&pdev->dev, "failed to get valid temp sensor number!\n");
		return -EINVAL;
	}

	sensors = devm_kzalloc(&pdev->dev, sizeof(*data->sensor) * sensor_num,
			       GFP_KERNEL);
	if (!sensors)
		return -ENOMEM;

	data->sensor = sensors;

	np = of_find_node_by_name(NULL, "thermal-zones");
	if (!np)
		return -ENODEV;

	for (i = 0; i < sensor_num; i++) {
		struct imx_sc_sensor *sensor = &data->sensor[i];

		sensor_np = of_get_next_child(np, sensor_np);
		ret = imx_sc_thermal_get_sensor_id(sensor_np, &sensor->resource_id);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to get valid sensor resource id: %d\n",
				ret);
			break;
		}

		ret = imx_sc_thermal_register_sensor(pdev, sensor);
		if (ret) {
			dev_err(&pdev->dev, "failed to register thermal sensor: %d\n",
				ret);
			break;
		}
		trip = of_thermal_get_trip_points(sensor->tzd);
		sensor->temp_passive = trip[0].temperature;
		sensor->temp_critical = trip[1].temperature;

		sensor->cdev = devfreq_cooling_register();
		if (IS_ERR(sensor->cdev)) {
			dev_err(&pdev->dev,
				"failed to register devfreq cooling device: %d\n",
				ret);
			return ret;
		}

		ret = thermal_zone_bind_cooling_device(sensor->tzd,
			IMX_TRIP_PASSIVE,
			sensor->cdev,
			THERMAL_NO_LIMIT,
			THERMAL_NO_LIMIT,
			THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&sensor->tzd->device,
				"binding zone %s with cdev %s failed:%d\n",
				sensor->tzd->type, sensor->cdev->type, ret);
			devfreq_cooling_unregister(sensor->cdev);
			return ret;
		}
	}

	of_node_put(np);
	of_node_put(sensor_np);

	return ret;
}

static const struct of_device_id imx_sc_thermal_table[] = {
	{ .compatible = "fsl,imx8qxp-sc-thermal", },
	{ .compatible = "fsl,imx8qm-sc-thermal", },
	{}
};
MODULE_DEVICE_TABLE(of, imx_sc_thermal_table);

static struct platform_driver imx_sc_thermal_driver = {
		.probe = imx_sc_thermal_probe,
		.driver = {
			.name = "imx-sc-thermal",
			.of_match_table = imx_sc_thermal_table,
		},
};
module_platform_driver(imx_sc_thermal_driver);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("Thermal driver for NXP i.MX SoCs with system controller");
MODULE_LICENSE("GPL v2");
