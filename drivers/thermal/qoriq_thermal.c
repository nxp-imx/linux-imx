// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.

#include <linux/clk.h>
#include <linux/device_cooling.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/thermal.h>
#include <linux/slab.h>

#include "thermal_core.h"

#define SITES_MAX	16
#define TMU_TEMP_PASSIVE_COOL_DELTA	10000
#define TMR_DISABLE		0x0
#define TMR_ME			0x80000000
#define TMR_ALPF		0x0c000000
#define TMR_ALPF_V2		0x03000000
#define TMTMIR_DEFAULT	0x0000000f
#define TIER_DISABLE	0x0
#define TEUMR0_V2		0x51009c00
#define TMU_VER1		0x1
#define TMU_VER2		0x2

enum tmu_throttle_id {
	THROTTLE_DEVFREQ = 0,
	THROTTLE_NUM,
};
static const char *const throt_names[] = {
	[THROTTLE_DEVFREQ] = "devfreq",
};

/*
 * QorIQ TMU Registers
 */
struct qoriq_tmu_site_regs {
	u32 tritsr;		/* Immediate Temperature Site Register */
	u32 tratsr;		/* Average Temperature Site Register */
	u8 res0[0x8];
};

struct qoriq_tmu_regs_v1 {
	u32 tmr;		/* Mode Register */
	u32 tsr;		/* Status Register */
	u32 tmtmir;		/* Temperature measurement interval Register */
	u8 res0[0x14];
	u32 tier;		/* Interrupt Enable Register */
	u32 tidr;		/* Interrupt Detect Register */
	u32 tiscr;		/* Interrupt Site Capture Register */
	u32 ticscr;		/* Interrupt Critical Site Capture Register */
	u8 res1[0x10];
	u32 tmhtcrh;		/* High Temperature Capture Register */
	u32 tmhtcrl;		/* Low Temperature Capture Register */
	u8 res2[0x8];
	u32 tmhtitr;		/* High Temperature Immediate Threshold */
	u32 tmhtatr;		/* High Temperature Average Threshold */
	u32 tmhtactr;	/* High Temperature Average Crit Threshold */
	u8 res3[0x24];
	u32 ttcfgr;		/* Temperature Configuration Register */
	u32 tscfgr;		/* Sensor Configuration Register */
	u8 res4[0x78];
	struct qoriq_tmu_site_regs site[SITES_MAX];
	u8 res5[0x9f8];
	u32 ipbrr0;		/* IP Block Revision Register 0 */
	u32 ipbrr1;		/* IP Block Revision Register 1 */
	u8 res6[0x310];
	u32 ttrcr[4];		/* Temperature Range Control Register */
};

struct qoriq_tmu_regs_v2 {
	u32 tmr;		/* Mode Register */
	u32 tsr;		/* Status Register */
	u32 tmsr;		/* monitor site register */
	u32 tmtmir;		/* Temperature measurement interval Register */
	u8 res0[0x10];
	u32 tier;		/* Interrupt Enable Register */
	u32 tidr;		/* Interrupt Detect Register */
	u8 res1[0x8];
	u32 tiiscr;		/* interrupt immediate site capture register */
	u32 tiascr;		/* interrupt average site capture register */
	u32 ticscr;		/* Interrupt Critical Site Capture Register */
	u32 res2;
	u32 tmhtcr;		/* monitor high temperature capture register */
	u32 tmltcr;		/* monitor low temperature capture register */
	u32 tmrtrcr;	/* monitor rising temperature rate capture register */
	u32 tmftrcr;	/* monitor falling temperature rate capture register */
	u32 tmhtitr;	/* High Temperature Immediate Threshold */
	u32 tmhtatr;	/* High Temperature Average Threshold */
	u32 tmhtactr;	/* High Temperature Average Crit Threshold */
	u32 res3;
	u32 tmltitr;	/* monitor low temperature immediate threshold */
	u32 tmltatr;	/* monitor low temperature average threshold register */
	u32 tmltactr;	/* monitor low temperature average critical threshold */
	u32 res4;
	u32 tmrtrctr;	/* monitor rising temperature rate critical threshold */
	u32 tmftrctr;	/* monitor falling temperature rate critical threshold*/
	u8 res5[0x8];
	u32 ttcfgr;	/* Temperature Configuration Register */
	u32 tscfgr;	/* Sensor Configuration Register */
	u8 res6[0x78];
	struct qoriq_tmu_site_regs site[SITES_MAX];
	u8 res7[0x9f8];
	u32 ipbrr0;		/* IP Block Revision Register 0 */
	u32 ipbrr1;		/* IP Block Revision Register 1 */
	u8 res8[0x300];
	u32 teumr0;
	u32 teumr1;
	u32 teumr2;
	u32 res9;
	u32 ttrcr[4];	/* Temperature Range Control Register */
 };
 
struct qoriq_tmu_data;

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
	struct thermal_zone_device	*tzd;
	struct qoriq_tmu_data		*qdata;
	int				id;
	int				*trip_temp;
	int				ntrip_temp;
	int				temp_delta;
	struct tmu_throttle_params	throt_cfgs[THROTTLE_NUM];
};

struct qoriq_tmu_data {
	int ver;
	struct qoriq_tmu_regs_v1 __iomem *regs;
	struct qoriq_tmu_regs_v2 __iomem *regs_v2;
	struct clk *clk;
	bool little_endian;
	struct qoriq_sensor	*sensor[SITES_MAX];
};

static void tmu_write(struct qoriq_tmu_data *p, u32 val, void __iomem *addr)
{
	if (p->little_endian)
		iowrite32(val, addr);
	else
		iowrite32be(val, addr);
}

static u32 tmu_read(struct qoriq_tmu_data *p, void __iomem *addr)
{
	if (p->little_endian)
		return ioread32(addr);
	else
		return ioread32be(addr);
}

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qsensor->qdata;
	u32 val;

	val = tmu_read(qdata, &qdata->regs->site[qsensor->id].tritsr);
	*temp = (val & 0xff) * 1000;

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	struct qoriq_sensor *qsensor = p;
	int trip_temp;

	if (!qsensor->tzd)
		return 0;

	trip_temp = trip < qsensor->ntrip_temp ? qsensor->trip_temp[trip] : qsensor->trip_temp[0];

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
static int tmu_init_throttle_cdev(struct platform_device *pdev, int index)
{
	struct device *dev = &pdev->dev;
	struct qoriq_tmu_data *qt = dev_get_drvdata(dev);
	struct qoriq_sensor *sensor = qt->sensor[index];
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
				dev_err(&pdev->dev,
					"failed to register"
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

		tcd = device_cooling_register(np_tcc, ttp->max_state);
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

static int qoriq_tmu_register_tmu_zone(struct platform_device *pdev)
{
	struct qoriq_tmu_data *qdata = platform_get_drvdata(pdev);
	const struct thermal_trip *trip;
	int id, sites = 0, ret;
	int i, ntrips;

	for (id = 0; id < SITES_MAX; id++) {
		qdata->sensor[id] = devm_kzalloc(&pdev->dev,
				sizeof(struct qoriq_sensor), GFP_KERNEL);
		if (!qdata->sensor[id])
			return -ENOMEM;

		qdata->sensor[id]->id = id;
		qdata->sensor[id]->qdata = qdata;
		qdata->sensor[id]->tzd = devm_thermal_zone_of_sensor_register(
				&pdev->dev, id, qdata->sensor[id], &tmu_tz_ops);
		if (IS_ERR(qdata->sensor[id]->tzd)) {
			if (PTR_ERR(qdata->sensor[id]->tzd) == -ENODEV)
				continue;
			else
				return PTR_ERR(qdata->sensor[id]->tzd);
		}

		/* first thermal zone takes care of system-wide device cooling */
		if (id == 0) {
			trip = of_thermal_get_trip_points(qdata->sensor[id]->tzd);
			ntrips = of_thermal_get_ntrips(qdata->sensor[id]->tzd);
			qdata->sensor[id]->trip_temp = kzalloc(ntrips
				* sizeof(*qdata->sensor[id]->trip_temp), GFP_KERNEL);
			if (!qdata->sensor[id]->trip_temp) {
				ret = -ENOMEM;
				return ret;
			}
			for (i=0; i<ntrips; i++) {
				qdata->sensor[id]->trip_temp[i] = trip[i].temperature;
			}
			qdata->sensor[id]->ntrip_temp = ntrips;

			ret = tmu_init_throttle_cdev(pdev, id);
			if (ret) {
				kfree(qdata->sensor[id]->trip_temp);
				return ret;
			}
		}

		if (qdata->ver == TMU_VER1)
			sites |= 0x1 << (15 - id);
		else
			sites |= 0x1 << id;
	}

	/* Enable monitoring */
	if (sites != 0) {
		if (qdata->ver == TMU_VER1) {
			tmu_write(qdata, sites | TMR_ME | TMR_ALPF,
					&qdata->regs->tmr);
		} else {
			tmu_write(qdata, sites, &qdata->regs_v2->tmsr);
			tmu_write(qdata, TMR_ME | TMR_ALPF_V2,
					&qdata->regs_v2->tmr);
		}
	}

	return 0;
}

static int qoriq_tmu_calibration(struct platform_device *pdev)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = pdev->dev.of_node;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	len = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (len < 0 || len > 4) {
		dev_err(&pdev->dev, "invalid range data.\n");
		return len;
	}

	val = of_property_read_u32_array(np, "fsl,tmu-range", range, len);
	if (val != 0) {
		dev_err(&pdev->dev, "failed to read range data.\n");
		return val;
	}

	/* Init temperature range registers */
	for (i = 0; i < len; i++)
		tmu_write(data, range[i], &data->regs->ttrcr[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(&pdev->dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		tmu_write(data, val, &data->regs->ttcfgr);
		val = of_read_number(calibration + 1, 1);
		tmu_write(data, val, &data->regs->tscfgr);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	/* Disable interrupt, using polling instead */
	tmu_write(data, TIER_DISABLE, &data->regs->tier);

	/* Set update_interval */
	if (data->ver == TMU_VER1) {
		tmu_write(data, TMTMIR_DEFAULT, &data->regs->tmtmir);
	} else {
		tmu_write(data, TMTMIR_DEFAULT, &data->regs_v2->tmtmir);
		tmu_write(data, TEUMR0_V2, &data->regs_v2->teumr0);
	}

	/* Disable monitoring */
	tmu_write(data, TMR_DISABLE, &data->regs->tmr);
}

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 ver;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;

	data = devm_kzalloc(&pdev->dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->little_endian = of_property_read_bool(np, "little-endian");

	data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->regs)) {
		dev_err(&pdev->dev, "Failed to get memory region\n");
		return PTR_ERR(data->regs);
	}

	data->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	/* version register offset at: 0xbf8 on both v1 and v2 */
	ver = tmu_read(data, &data->regs->ipbrr0);
	data->ver = (ver >> 8) & 0xff;
	if (data->ver == TMU_VER2)
		data->regs_v2 = (void __iomem *)data->regs;

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = qoriq_tmu_calibration(pdev);	/* TMU calibration */
	if (ret < 0)
		goto err;

	ret = qoriq_tmu_register_tmu_zone(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register sensors\n");
		ret = -ENODEV;
		goto err;
	}

	return 0;

err:
	clk_disable_unprepare(data->clk);
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int qoriq_tmu_remove(struct platform_device *pdev)
{
	int i, j;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	for (i=0; i<SITES_MAX; i++) {
		struct qoriq_sensor *sens = data->sensor[i];

		if (i == 0) {
			for (j=0; j<THROTTLE_NUM; j++)
				if (sens->throt_cfgs[j].inited)
					device_cooling_unregister(sens->throt_cfgs[j].cdev);
			if (sens->trip_temp)
				kfree(sens->trip_temp);
		}
		thermal_zone_of_sensor_unregister(&pdev->dev, sens->tzd);
	}

	/* Disable monitoring */
	tmu_write(data, TMR_DISABLE, &data->regs->tmr);

	clk_disable_unprepare(data->clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int __maybe_unused qoriq_tmu_suspend(struct device *dev)
{
	u32 tmr;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	/* Disable monitoring */
	tmr = tmu_read(data, &data->regs->tmr);
	tmr &= ~TMR_ME;
	tmu_write(data, tmr, &data->regs->tmr);

	clk_disable_unprepare(data->clk);

	return 0;
}

static int __maybe_unused qoriq_tmu_resume(struct device *dev)
{
	u32 tmr;
	int ret;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	/* Enable monitoring */
	tmr = tmu_read(data, &data->regs->tmr);
	tmr |= TMR_ME;
	tmu_write(data, tmr, &data->regs->tmr);

	return 0;
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
