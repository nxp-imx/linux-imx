// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.

#include <linux/clk.h>
#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/thermal.h>
#include <linux/units.h>
#include <linux/slab.h>

#include "thermal_core.h"
#include "thermal_hwmon.h"

#define SITES_MAX		16
#define TMR_DISABLE		0x0
#define TMR_ME			0x80000000
#define TMR_ALPF		0x0c000000
#define TMR_ALPF_V2		0x03000000
#define TMTMIR_DEFAULT	0x0000000f
#define TIER_DISABLE	0x0
#define TEUMR0_V2		0x51009c00
#define TMSARA_V2		0xe
#define TMU_VER1		0x1
#define TMU_VER2		0x2
#define TMU_TEMP_PASSIVE_COOL_DELTA	10000

#define REGS_TMR	0x000	/* Mode Register */
#define TMR_DISABLE	0x0
#define TMR_ME		0x80000000
#define TMR_ALPF	0x0c000000
#define TMR_MSITE_ALL	GENMASK(15, 0)

#define REGS_TMTMIR	0x008	/* Temperature measurement interval Register */
#define TMTMIR_DEFAULT	0x0000000f

#define REGS_V2_TMSR	0x008	/* monitor site register */

#define REGS_V2_TMTMIR	0x00c	/* Temperature measurement interval Register */

#define REGS_TIER	0x020	/* Interrupt Enable Register */
#define TIER_DISABLE	0x0


#define REGS_TTCFGR	0x080	/* Temperature Configuration Register */
#define REGS_TSCFGR	0x084	/* Sensor Configuration Register */

#define REGS_TRITSR(n)	(0x100 + 16 * (n)) /* Immediate Temperature
					    * Site Register
					    */
#define TRITSR_V	BIT(31)
#define REGS_V2_TMSAR(n)	(0x304 + 16 * (n))	/* TMU monitoring
						* site adjustment register
						*/
#define REGS_TTRnCR(n)	(0xf10 + 4 * (n)) /* Temperature Range n
					   * Control Register
					   */
#define REGS_IPBRR(n)		(0xbf8 + 4 * (n)) /* IP Block Revision
						   * Register n
						   */
#define REGS_V2_TEUMR(n)	(0xf00 + 4 * (n))

enum tmu_throttle_id {
	THROTTLE_DEVFREQ = 0,
	THROTTLE_NUM,
};
static const char *const throt_names[] = {
	[THROTTLE_DEVFREQ] = "devfreq",
};

struct tmu_throttle_params {
	const char *name;
	struct thermal_cooling_device *cdev;
	int max_state;
	bool inited;
};

/*
 * Thermal zone data
 */
struct qoriq_sensor {
	int				id;
	struct thermal_zone_device	*tzd;
	int                             *trip_temp;
	int                             ntrip_temp;
	int                             temp_delta;
	struct tmu_throttle_params      throt_cfgs[THROTTLE_NUM];
};

struct qoriq_tmu_data {
	int ver;
	struct regmap *regmap;
	struct clk *clk;
	struct qoriq_sensor	sensor[SITES_MAX];
};

static struct qoriq_tmu_data *qoriq_sensor_to_data(struct qoriq_sensor *s)
{
	return container_of(s, struct qoriq_tmu_data, sensor[s->id]);
}

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(qsensor);
	u32 val;
	/*
	 * REGS_TRITSR(id) has the following layout:
	 *
	 * For TMU Rev1:
	 * 31  ... 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 8 bit value representing
	 * temperature in Celsius.

	 * For TMU Rev2:
	 * 31  ... 8 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 9 bit value representing
	 * temperature in KelVin.
	 */
	if (regmap_read_poll_timeout(qdata->regmap,
				     REGS_TRITSR(qsensor->id),
				     val,
				     val & TRITSR_V,
				     USEC_PER_MSEC,
				     10 * USEC_PER_MSEC))
		return -ENODATA;

	if (qdata->ver == TMU_VER1)
		*temp = (val & GENMASK(7, 0)) * MILLIDEGREE_PER_DEGREE;
	else
		*temp = kelvin_to_millicelsius(val & GENMASK(8, 0));

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	struct qoriq_sensor *qsensor = p;
	int trip_temp;

	if (!qsensor->tzd)
		return 0;

	trip_temp = trip < qsensor->ntrip_temp ? qsensor->trip_temp[trip] :
					     qsensor->trip_temp[0];

	if (qsensor->tzd->temperature >= (trip_temp - qsensor->temp_delta))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int tmu_set_trip_temp(void *p, int trip,
			     int temp)
{
	struct qoriq_sensor *qsensor = p;

	if (trip < qsensor->ntrip_temp)
		qsensor->trip_temp[trip] = temp;

	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.get_trend = tmu_get_trend,
	.set_trip_temp = tmu_set_trip_temp,
};

static struct tmu_throttle_params *
find_throttle_cfg_by_name(struct qoriq_sensor *sens, const char *name)
{
	unsigned int i;

	for (i = 0; sens->throt_cfgs[i].name && i<THROTTLE_NUM; i++)
		if (!strcmp(sens->throt_cfgs[i].name, name))
			return &sens->throt_cfgs[i];

	return NULL;
}

/**
 * tmu_init_throttle_cdev() - Parse the throttle configurations
 * and register them as cooling devices.
 */
static int tmu_init_throttle_cdev(struct device *dev,
				  struct qoriq_tmu_data *qdata, int index)
{
	struct qoriq_tmu_data *qt = qdata;
	struct qoriq_sensor *sensor = &qt->sensor[index];
	struct device_node *np_tc, *np_tcc;
	struct thermal_cooling_device *tcd;
	const char *name;
	u32 val;
	int i, ret = 0;

	for (i = 0; i < THROTTLE_NUM; i++) {
		sensor->throt_cfgs[i].name = throt_names[i];
		sensor->throt_cfgs[i].inited = false;
	}

	np_tc = of_get_child_by_name(dev->of_node, "throttle-cfgs");
	if (!np_tc) {
		dev_info(dev,
			 "throttle-cfg: no throttle-cfgs"
			 " - use default devfreq cooling device\n");
		tcd = device_cooling_register(NULL, 1);
		if (IS_ERR(tcd)) {
			ret = PTR_ERR(tcd);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to register"
					"devfreq cooling device: %d\n",
					ret);
			return ret;
		}
		return 0;
	}

	ret = of_property_read_u32(np_tc, "throttle,temp_delta", &val);
	if (ret) {
		dev_info(dev,
			 "throttle-cfg: missing temp_delta parameter,"
			 "use default 3000 (3C)\n");
		sensor->temp_delta = 3000;
	} else {
		sensor->temp_delta = val;
	}

	for_each_child_of_node(np_tc, np_tcc) {
		struct tmu_throttle_params *ttp;
		name = np_tcc->name;
		ttp = find_throttle_cfg_by_name(sensor, name);
		if (!ttp) {
			dev_err(dev,
				"throttle-cfg: could not find %s\n", name);
			continue;
		}

		ret = of_property_read_u32(np_tcc, "throttle,max_state", &val);
		if (ret) {
			dev_info(dev,
				 "throttle-cfg: %s: missing throttle max state\n", name);
			continue;
		}
		ttp->max_state = val;

		tcd = device_gpu_cooling_register(np_tcc, ttp->max_state);
		of_node_put(np_tcc);
		if (IS_ERR(tcd)) {
			ret = PTR_ERR(tcd);
			dev_err(dev,
				"throttle-cfg: %s: failed to register cooling device\n",
				name);
			continue;
		}
		ttp->cdev = tcd;
		ttp->inited = true;
	}

	of_node_put(np_tc);
	return ret;
}

static int qoriq_tmu_register_tmu_zone(struct device *dev,
				       struct qoriq_tmu_data *qdata)
{
	int id;
	const struct thermal_trip *trip;
	int i, ntrips;

	if (qdata->ver == TMU_VER1) {
		regmap_write(qdata->regmap, REGS_TMR,
			     TMR_MSITE_ALL | TMR_ME | TMR_ALPF);
	} else {
		regmap_write(qdata->regmap, REGS_V2_TMSR, TMR_MSITE_ALL);
		regmap_write(qdata->regmap, REGS_TMR, TMR_ME | TMR_ALPF_V2);
	}

	for (id = 0; id < SITES_MAX; id++) {
		struct thermal_zone_device *tzd;
		struct qoriq_sensor *sensor = &qdata->sensor[id];
		int ret;

		sensor->id = id;

		tzd = devm_thermal_zone_of_sensor_register(dev, id,
							   sensor,
							   &tmu_tz_ops);
		ret = PTR_ERR_OR_ZERO(tzd);
		if (ret) {
			if (ret == -ENODEV)
				continue;

			regmap_write(qdata->regmap, REGS_TMR, TMR_DISABLE);
			return ret;
		}

		sensor->tzd = tzd;

		if (devm_thermal_add_hwmon_sysfs(tzd))
			dev_warn(dev,
				 "Failed to add hwmon sysfs attributes\n");
		/* first thermal zone takes care of system-wide device cooling */
		if (id == 0) {
			trip = of_thermal_get_trip_points(tzd);
			ntrips = of_thermal_get_ntrips(tzd);
			sensor->trip_temp = kzalloc(ntrips
					* sizeof(*sensor->trip_temp), GFP_KERNEL);
			if (!sensor->trip_temp) {
				ret = -ENOMEM;
				return ret;
			}
			for (i=0; i<ntrips; i++) {
				sensor->trip_temp[i] = trip[i].temperature;
			}
			sensor->ntrip_temp = ntrips;

			ret = tmu_init_throttle_cdev(dev, qdata, id);
			if (ret) {
				kfree(sensor->trip_temp);
				return ret;
			}
		}
	}

	return 0;
}

static int qoriq_tmu_calibration(struct device *dev,
				 struct qoriq_tmu_data *data)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = dev->of_node;

	len = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (len < 0 || len > 4) {
		dev_err(dev, "invalid range data.\n");
		return len;
	}

	val = of_property_read_u32_array(np, "fsl,tmu-range", range, len);
	if (val != 0) {
		dev_err(dev, "failed to read range data.\n");
		return val;
	}

	/* Init temperature range registers */
	for (i = 0; i < len; i++)
		regmap_write(data->regmap, REGS_TTRnCR(i), range[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		regmap_write(data->regmap, REGS_TTCFGR, val);
		val = of_read_number(calibration + 1, 1);
		regmap_write(data->regmap, REGS_TSCFGR, val);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	int i;

	/* Disable interrupt, using polling instead */
	regmap_write(data->regmap, REGS_TIER, TIER_DISABLE);

	/* Set update_interval */

	if (data->ver == TMU_VER1) {
		regmap_write(data->regmap, REGS_TMTMIR, TMTMIR_DEFAULT);
	} else {
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
		regmap_write(data->regmap, REGS_V2_TEUMR(0), TEUMR0_V2);
		for (i = 0; i < SITES_MAX; i++)
			regmap_write(data->regmap, REGS_V2_TMSAR(i), TMSARA_V2);
	}

	/* Disable monitoring */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
}

static const struct regmap_range qoriq_yes_ranges[] = {
	regmap_reg_range(REGS_TMR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(3)),
	regmap_reg_range(REGS_V2_TEUMR(0), REGS_V2_TEUMR(2)),
	regmap_reg_range(REGS_V2_TMSAR(0), REGS_V2_TMSAR(15)),
	regmap_reg_range(REGS_IPBRR(0), REGS_IPBRR(1)),
	/* Read only registers below */
	regmap_reg_range(REGS_TRITSR(0), REGS_TRITSR(15)),
};

static const struct regmap_access_table qoriq_wr_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges) - 1,
};

static const struct regmap_access_table qoriq_rd_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges),
};

static void qoriq_tmu_action(void *p)
{
	struct qoriq_tmu_data *data = p;

	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
	clk_disable_unprepare(data->clk);
}

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 ver;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const bool little_endian = of_property_read_bool(np, "little-endian");
	const enum regmap_endian format_endian =
		little_endian ? REGMAP_ENDIAN_LITTLE : REGMAP_ENDIAN_BIG;
	const struct regmap_config regmap_config = {
		.reg_bits		= 32,
		.val_bits		= 32,
		.reg_stride		= 4,
		.rd_table		= &qoriq_rd_table,
		.wr_table		= &qoriq_wr_table,
		.val_format_endian	= format_endian,
		.max_register		= SZ_4K,
	};
	void __iomem *base;

	data = devm_kzalloc(dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	ret = PTR_ERR_OR_ZERO(base);
	if (ret) {
		dev_err(dev, "Failed to get memory region\n");
		return ret;
	}

	data->regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	ret = PTR_ERR_OR_ZERO(data->regmap);
	if (ret) {
		dev_err(dev, "Failed to init regmap (%d)\n", ret);
		return ret;
	}

	data->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, qoriq_tmu_action, data);
	if (ret)
		return ret;

	/* version register offset at: 0xbf8 on both v1 and v2 */
	ret = regmap_read(data->regmap, REGS_IPBRR(0), &ver);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read IP block version\n");
		return ret;
	}
	data->ver = (ver >> 8) & 0xff;

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = qoriq_tmu_calibration(dev, data);	/* TMU calibration */
	if (ret < 0)
		return ret;

	ret = qoriq_tmu_register_tmu_zone(dev, data);
	if (ret < 0) {
		dev_err(dev, "Failed to register sensors\n");
		return ret;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int qoriq_tmu_remove(struct platform_device *pdev)
{
	int i, j;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	for (i=0; i<SITES_MAX; i++) {
		struct qoriq_sensor *sens = &data->sensor[i];

		if (i == 0) {
			for (j=0; j<THROTTLE_NUM; j++)
				if (sens->throt_cfgs[j].inited)
					device_gpu_cooling_unregister(sens->throt_cfgs[j].cdev);
			if (sens->trip_temp)
				kfree(sens->trip_temp);
		}
		thermal_zone_of_sensor_unregister(&pdev->dev, sens->tzd);
	}

	/* Disable monitoring */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);

	clk_disable_unprepare(data->clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int __maybe_unused qoriq_tmu_suspend(struct device *dev)
{
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, 0);
	if (ret)
		return ret;

	clk_disable_unprepare(data->clk);

	return 0;
}

static int __maybe_unused qoriq_tmu_resume(struct device *dev)
{
	int ret;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	/* Enable monitoring */
	return regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, TMR_ME);
}

static SIMPLE_DEV_PM_OPS(qoriq_tmu_pm_ops,
			 qoriq_tmu_suspend, qoriq_tmu_resume);

static const struct of_device_id qoriq_tmu_match[] = {
	{ .compatible = "fsl,qoriq-tmu", },
	{ .compatible = "fsl,imx8mq-tmu", },
	{},
};
MODULE_DEVICE_TABLE(of, qoriq_tmu_match);

static struct platform_driver qoriq_tmu = {
	.driver	= {
		.name		= "qoriq_thermal",
		.pm		= &qoriq_tmu_pm_ops,
		.of_match_table	= qoriq_tmu_match,
	},
	.probe	= qoriq_tmu_probe,
	.remove	= qoriq_tmu_remove,
};
module_platform_driver(qoriq_tmu);

MODULE_AUTHOR("Jia Hongtao <hongtao.jia@nxp.com>");
MODULE_DESCRIPTION("QorIQ Thermal Monitoring Unit driver");
MODULE_LICENSE("GPL v2");
