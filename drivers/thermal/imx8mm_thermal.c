// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#include "thermal_core.h"

#define TER			0x0	/* TMU enable */
#define	TPS			0x4
#define TRITSR			0x20	/* TMU immediate temp */

#define TER_EN			BIT(31)
#define TRITSR_VAL_MASK		0xff

#define PROBE_SEL_ALL		GENMASK(31, 30)

#define PROBE0_STATUS_OFFSET	30
#define PROBE0_VAL_OFFSET	16
#define SIGN_BIT		BIT(7)
#define TEMP_VAL_MASK		0x7f

#define TEMP_LOW_LIMIT		10
#define IMX_TEMP_PASSIVE_COOL_DELTA 10000

#define FLAGS_TMU_VER1		0x1
#define FLAGS_TMU_VER2		0x2

struct imx8mm_tmu;

struct thermal_soc_data {
	u32 num_sensors;
	u32 flags;
};

struct tmu_sensor {
	struct imx8mm_tmu *priv;
	u32 hw_id;
	int temp_passive;
	int temp_critical;
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *cdev;
};

struct imx8mm_tmu {
	void __iomem *base;
	struct clk *clk;
	const struct thermal_soc_data *socdata;
	struct tmu_sensor sensors[0];
};

/* The driver support 1 passive trip point and 1 critical trip point */
enum imx_thermal_trip {
	IMX_TRIP_PASSIVE,
	IMX_TRIP_CRITICAL,
	IMX_TRIP_NUM,
};


static int tmu_get_temp(void *data, int *temp)
{
	struct tmu_sensor *sensor = data;
	struct imx8mm_tmu *tmu = sensor->priv;
	bool ready;
	u32 val;

	/* the temp sensor need about 1ms to finish the measurement */
	usleep_range(1000, 2000);

	if (tmu->socdata->flags == FLAGS_TMU_VER1) {
		val = readl_relaxed(tmu->base + TRITSR) & TRITSR_VAL_MASK;
		if (val < TEMP_LOW_LIMIT)
			return -EAGAIN;
	} else {
		val = readl_relaxed(tmu->base + TRITSR);
		ready = val & (1 << (sensor->hw_id + PROBE0_STATUS_OFFSET));
		val = (val >> sensor->hw_id * PROBE0_VAL_OFFSET) & TRITSR_VAL_MASK;
		if (val & SIGN_BIT) /* negative */
			val = (~(val & TEMP_VAL_MASK) + 1);

		*temp = val;
		if (!ready || *temp < -40 || *temp > 125)
			return -EAGAIN;
	}

	*temp = val * 1000;

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	int trip_temp;
	struct tmu_sensor *sensor = p;

	if (!sensor->tzd)
		return 0;

	trip_temp = (trip == IMX_TRIP_PASSIVE) ? sensor->temp_passive : sensor->temp_critical;

	if (sensor->tzd->temperature >= (trip_temp - IMX_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int tmu_set_trip_temp(void *p, int trip, int temp)
{
	struct tmu_sensor *sensor = p;

	if (trip == IMX_TRIP_CRITICAL)
		sensor->temp_critical = temp;

	if (trip == IMX_TRIP_PASSIVE)
		sensor->temp_passive = temp;

	return 0;
}

static struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.get_trend = tmu_get_trend,
	.set_trip_temp = tmu_set_trip_temp,
};

static int imx8mm_tmu_probe(struct platform_device *pdev)
{
	const struct thermal_trip *trips;
	struct imx8mm_tmu *tmu;
	const struct thermal_soc_data *data;
	u32 val, num_sensors;
	int ret, i;

	data = of_device_get_match_data(&pdev->dev);
	num_sensors = data->num_sensors;

	tmu = devm_kzalloc(&pdev->dev, struct_size(tmu, sensors, num_sensors), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	platform_set_drvdata(pdev, tmu);
	tmu->socdata = data;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);

	tmu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tmu->clk)) {
		ret = PTR_ERR(tmu->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to get tmu clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(tmu->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tmu clock: %d\n", ret);
		return ret;
	}

	for (i = 0; i < num_sensors; i++) {
		tmu->sensors[i].priv = tmu;
		tmu->sensors[i].tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, i,
							&tmu->sensors[i], &tmu_tz_ops);
		if (IS_ERR(tmu->sensors[i].tzd)) {
			dev_err(&pdev->dev,
				"failed to register thermal zone sensor[%d]: %d\n", i, ret);
			return PTR_ERR(tmu->sensors[i].tzd);
		}

		tmu->sensors[i].hw_id = i;

		trips = of_thermal_get_trip_points(tmu->sensors[i].tzd);

		/* get the thermal trip temp */
		tmu->sensors[i].temp_passive = trips[0].temperature;
		tmu->sensors[i].temp_critical = trips[1].temperature;

		tmu->sensors[i].cdev = device_cooling_register(NULL, 1);
		if (IS_ERR(tmu->sensors[i].cdev)) {
			ret = PTR_ERR(tmu->sensors[i].cdev);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "failed to register devfreq cooling device %d\n", ret);
			return ret;
		}

		ret = thermal_zone_bind_cooling_device(tmu->sensors[i].tzd,
			IMX_TRIP_PASSIVE,
			tmu->sensors[i].cdev,
			THERMAL_NO_LIMIT,
			THERMAL_NO_LIMIT,
			THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&pdev->dev,
				"binding zone %s with cdev %s failed:%d\n",
				tmu->sensors[i].tzd->type, tmu->sensors[i].cdev->type, ret);
			device_cooling_unregister(tmu->sensors[i].cdev);
			return ret;
		}
	}

	/* disable the monitor for config */
	val = readl_relaxed(tmu->base + TER);
	val &= ~TER_EN;
	writel_relaxed(val, tmu->base + TER);

	/* enable all the probes for V2 TMU */
	if (tmu->socdata->flags == FLAGS_TMU_VER2) {
		val = readl_relaxed(tmu->base + TPS);
		val |= PROBE_SEL_ALL;
		writel_relaxed(val, tmu->base + TPS);
	}

	/* enable the monitor */
	val = readl_relaxed(tmu->base + TER);
	val |= TER_EN;
	writel_relaxed(val, tmu->base + TER);

	return 0;
}

static int imx8mm_tmu_remove(struct platform_device *pdev)
{
	struct imx8mm_tmu *tmu = platform_get_drvdata(pdev);
	u32 val;

	/* disable TMU */
	val = readl_relaxed(tmu->base + TER);
	val &= ~TER_EN;
	writel_relaxed(val, tmu->base + TER);

	clk_disable_unprepare(tmu->clk);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

struct thermal_soc_data imx8mm_tmu_data = {
	.num_sensors = 1,
	.flags = FLAGS_TMU_VER1,
};

struct thermal_soc_data imx8mp_tmu_data = {
	.num_sensors = 2,
	.flags = FLAGS_TMU_VER2,
};

static const struct of_device_id imx8mm_tmu_table[] = {
	{ .compatible = "fsl,imx8mm-tmu", .data = &imx8mm_tmu_data, },
	{ .compatible = "fsl,imx8mp-tmu", .data = &imx8mp_tmu_data, },
	{ },
};

static struct platform_driver imx8mm_tmu = {
	.driver = {
		.name	= "i.mx8mm_thermal",
		.of_match_table = imx8mm_tmu_table,
	},
	.probe = imx8mm_tmu_probe,
	.remove = imx8mm_tmu_remove,
};
module_platform_driver(imx8mm_tmu);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("i.MX8MM Thermal Monitor Unit driver");
MODULE_LICENSE("GPL v2");
