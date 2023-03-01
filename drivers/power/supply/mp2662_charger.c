// SPDX-License-Identifier: GPL-2.0
/*
 * mp2662 battery charger driver
 *
 * Copyright 2023 NXP
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>

#include <linux/of.h>

#define MP2662_MANUFACTURER		"Monolithic Power Systems"
#define MP2662_IRQ_PIN			"mp2662_irq"

static const char *const mp2662_chip_name = "mp2662";

enum mp2662_fields {
	F_IILIM, F_SYSVMIN,						/* REG00 */
	F_CEB, F_EN_HIZ,						/* REG01 */
	F_ICHG, F_REG_RST,						/* REG02 */
	F_IPRECHG, F_ITERM,						/* REG03 */
	F_VREG,								/* REG04 */
	F_WD,								/* REG05 */
	F_TREG,								/* REG07 */
	F_THERM_STAT, F_PG_STAT, F_CHG_STAT,				/* REG08 */
	F_NTC_FAULT, F_BAT_FAULT, F_CHG_FAULT,				/* REG09 */
	F_MAX_FIELDS
};

/* initial field values, converted to register values */
struct mp2662_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 treg;	/* thermal regulation threshold */
};

struct mp2662_state {
	u8 online;
	u8 chrg_status;
	u8 chrg_fault;
	u8 bat_fault;
};

struct mp2662_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;

	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	struct mp2662_init_data init_data;
	struct mp2662_state state;

	struct mutex lock; /* protect state data */
};

static const struct regmap_range mp2662_readonly_reg_ranges[] = {
	regmap_reg_range(0x08, 0x09),
};

static const struct regmap_access_table mp2662_writeable_regs = {
	.no_ranges = mp2662_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(mp2662_readonly_reg_ranges),
};

static const struct regmap_range mp2662_volatile_reg_ranges[] = {
	regmap_reg_range(0x01, 0x02),
	regmap_reg_range(0x05, 0x05),
	regmap_reg_range(0x08, 0x09),
};

static const struct regmap_access_table mp2662_volatile_regs = {
	.yes_ranges = mp2662_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(mp2662_volatile_reg_ranges),
};

static const struct regmap_config mp2662_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x09,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &mp2662_writeable_regs,
	.volatile_table = &mp2662_volatile_regs,
};

static const struct reg_field mp2662_reg_fields[] = {
	/* REG00 */
	[F_IILIM]		= REG_FIELD(0x00, 0, 3),
	[F_SYSVMIN]		= REG_FIELD(0x00, 4, 7),
	/* REG01 */
	[F_CEB]			= REG_FIELD(0x01, 3, 3),
	[F_EN_HIZ]		= REG_FIELD(0x01, 4, 4),
	/* REG02 */
	[F_ICHG]		= REG_FIELD(0x02, 0, 5),
	[F_REG_RST]		= REG_FIELD(0x02, 7, 7),
	/* REG03 */
	[F_IPRECHG]		= REG_FIELD(0x03, 0, 3),
	[F_ITERM]		= REG_FIELD(0x03, 0, 3), // mp2662 charger SPEC define IPRECHG = ITERM
	/* REG04 */
	[F_VREG]		= REG_FIELD(0x04, 2, 7),
	/* REG05 */
	[F_WD]			= REG_FIELD(0x05, 0, 7),
	/* REG07 */
	[F_TREG]		= REG_FIELD(0x07, 4, 5),
	/* REG08 */
	[F_THERM_STAT]		= REG_FIELD(0x08, 0, 0),
	[F_PG_STAT]		= REG_FIELD(0x08, 1, 1),
	[F_CHG_STAT]		= REG_FIELD(0x08, 3, 4),
	/* REG09 */
	[F_NTC_FAULT]		= REG_FIELD(0x09, 0, 1),
	[F_BAT_FAULT]		= REG_FIELD(0x09, 3, 3),
	[F_CHG_FAULT]		= REG_FIELD(0x09, 5, 5),
};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum mp2662_table_ids {
	/* range tables */
	TBL_ICHG,
	TBL_ITERM,
	TBL_IILIM,
	TBL_VREG,
	TBL_SYSVMIN,

	/* lookup tables */
	TBL_TREG,
};

/* Thermal Regulation Threshold lookup table, in degrees Celsius */
static const u32 mp2662_treg_tbl[] = { 60, 80, 100, 120 };

#define mp2662_TREG_TBL_SIZE		ARRAY_SIZE(mp2662_treg_tbl)

struct mp2662_range {
	u32 min;
	u32 max;
	u32 step;
};

struct mp2662_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct mp2662_range  rt;
	struct mp2662_lookup lt;
} mp2662_tables[] = {
	/* range tables */
	[TBL_ICHG] =	{ .rt = {8000,	  456000, 8000} },	 /* uA */
	[TBL_ITERM] =	{ .rt = {1000,   31000, 2000} },	 /* uA */
	[TBL_IILIM] =   { .rt = {50000,  500000, 30000} },	 /* uA */
	[TBL_VREG] =	{ .rt = {3600000, 4545000, 15000} },	 /* uV */
	[TBL_SYSVMIN] = { .rt = {3800000, 5080000, 80000} },	 /* uV */

	/* lookup tables */
	[TBL_TREG] =	{ .lt = {mp2662_treg_tbl, mp2662_TREG_TBL_SIZE} },
};

static int mp2662_field_read(struct mp2662_device *mp,
					enum mp2662_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(mp->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}

static int mp2662_field_write(struct mp2662_device *mp,
					enum mp2662_fields field_id, u8 val)
{
	return regmap_field_write(mp->rmap_fields[field_id], val);
}

static u8 mp2662_find_idx(u32 value, enum mp2662_table_ids id)
{
	u8 idx;

	if (id >= TBL_TREG) {
		const u32 *tbl = mp2662_tables[id].lt.tbl;
		u32 tbl_size = mp2662_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct mp2662_range *rtbl = &mp2662_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for (idx = 1;
		     idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value);
		     idx++)
			;
	}

	return idx - 1;
}

static u32 mp2662_find_val(u8 idx, enum mp2662_table_ids id)
{
	const struct mp2662_range *rtbl;

	/* lookup table */
	if (id >= TBL_TREG)
		return mp2662_tables[id].lt.tbl[idx];

	/* range table */
	rtbl = &mp2662_tables[id].rt;

	return (rtbl->min + idx * rtbl->step);
}

enum mp2662_status {
	STATUS_NOT_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING,
	STATUS_TERMINATION_DONE,
};

enum mp2662_chrg_fault {
	CHRG_FAULT_NORMAL,
	CHRG_FAULT_INPUT,
	CHRG_FAULT_THERMAL_SHUTDOWN,
	CHRG_FAULT_TIMER_EXPIRED,
};

static irqreturn_t __mp2662_handle_irq(struct mp2662_device *mp);

static int mp2662_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct mp2662_device *mp = power_supply_get_drvdata(psy);
	struct mp2662_state state;
	int ret;

	mutex_lock(&mp->lock);
	/* update state in case we lost an interrupt */
	__mp2662_handle_irq(mp);
	state = mp->state;
	mutex_unlock(&mp->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING ||
			 state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!state.online || state.chrg_status == STATUS_NOT_CHARGING ||
		    state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else if (state.chrg_status == STATUS_PRE_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		else if (state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else /* unreachable */
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = MP2662_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = mp2662_chip_name;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!state.chrg_fault && !state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.chrg_fault == CHRG_FAULT_TIMER_EXPIRED)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else if (state.chrg_fault == CHRG_FAULT_THERMAL_SHUTDOWN)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = mp2662_find_val(mp->init_data.ichg, TBL_ICHG);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = mp2662_find_val(mp->init_data.vreg, TBL_VREG);
		break;

	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		val->intval = mp2662_find_val(mp->init_data.iprechg, TBL_ITERM);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = mp2662_find_val(mp->init_data.iterm, TBL_ITERM);
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mp2662_field_read(mp, F_IILIM);
		if (ret < 0)
			return ret;

		val->intval = mp2662_find_val(ret, TBL_IILIM);
		break;

	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = mp2662_field_read(mp, F_SYSVMIN);
		if (ret < 0)
			return ret;

		val->intval = mp2662_find_val(ret, TBL_SYSVMIN);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mp2662_get_chip_state(struct mp2662_device *mp,
				  struct mp2662_state *state)
{
	int i, ret;

	struct {
		enum mp2662_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_PG_STAT,	    &state->online},
		{F_BAT_FAULT,	&state->bat_fault},
		{F_CHG_FAULT,	&state->chrg_fault}
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = mp2662_field_read(mp, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	dev_dbg(mp->dev, "S:CHG/PG=%d/%d, F:CHG/BAT=%d/%d\n",
		state->chrg_status, state->online,
		state->chrg_fault, state->bat_fault);

	return 0;
}

static irqreturn_t __mp2662_handle_irq(struct mp2662_device *mp)
{
	struct mp2662_state new_state;
	int ret;

	ret = mp2662_get_chip_state(mp, &new_state);
	if (ret < 0)
		return IRQ_NONE;

	if (!memcmp(&mp->state, &new_state, sizeof(new_state)))
		return IRQ_NONE;

	if (!new_state.online && mp->state.online) {	    /* power removed */
		/* disable charger */
		ret = mp2662_field_write(mp, F_CEB, 1);
		if (ret < 0)
			goto error;
	} else if (new_state.online && !mp->state.online) { /* power inserted */
		/* enable charger */
		ret = mp2662_field_write(mp, F_CEB, 0);
		if (ret < 0)
			goto error;
	}

	mp->state = new_state;
	power_supply_changed(mp->charger);

	return IRQ_HANDLED;
error:
	dev_err(mp->dev, "Error communicating with the chip: %pe\n",
		ERR_PTR(ret));
	return IRQ_HANDLED;
}

static irqreturn_t mp2662_irq_handler_thread(int irq, void *private)
{
	struct mp2662_device *mp = private;
	irqreturn_t ret;

	mutex_lock(&mp->lock);
	ret = __mp2662_handle_irq(mp);
	mutex_unlock(&mp->lock);

	return ret;
}

static int mp2662_chip_reset(struct mp2662_device *mp)
{
	int ret;
	int rst_check_counter = 10;

	ret = mp2662_field_write(mp, F_REG_RST, 1);
	if (ret < 0)
		return ret;

	do {
		ret = mp2662_field_read(mp, F_REG_RST);
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}

static int mp2662_hw_init(struct mp2662_device *mp)
{
	int ret;
	int i;

	const struct {
		enum mp2662_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG,	 mp->init_data.ichg},
		{F_VREG,	 mp->init_data.vreg},
		{F_ITERM,	 mp->init_data.iterm},
		{F_IPRECHG,	 mp->init_data.iprechg},
		{F_SYSVMIN,	 mp->init_data.sysvmin},
		{F_TREG,	 mp->init_data.treg},
	};

	ret = mp2662_chip_reset(mp);
	if (ret < 0) {
		dev_err(mp->dev, "Reset failed %d\n", ret);
		return ret;
	}

	/* disable watchdog */
	ret = mp2662_field_write(mp, F_WD, 0);
	if (ret < 0) {
		dev_err(mp->dev, "Disabling watchdog failed %d\n", ret);
		return ret;
	}

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = mp2662_field_write(mp, init_data[i].id,
					  init_data[i].value);
		if (ret < 0) {
			dev_err(mp->dev, "Writing init data failed %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static const enum power_supply_property mp2662_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
};

static char *mp2662_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc mp2662_power_supply_desc = {
	.name = "mp2662-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mp2662_power_supply_props,
	.num_properties = ARRAY_SIZE(mp2662_power_supply_props),
	.get_property = mp2662_power_supply_get_property,
};

static int mp2662_power_supply_init(struct mp2662_device *mp)
{
	struct power_supply_config psy_cfg = { .drv_data = mp, };

	psy_cfg.supplied_to = mp2662_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(mp2662_charger_supplied_to);

	mp->charger = power_supply_register(mp->dev, &mp2662_power_supply_desc,
					    &psy_cfg);

	return PTR_ERR_OR_ZERO(mp->charger);
}

static int mp2662_irq_probe(struct mp2662_device *mp)
{
	struct gpio_desc *irq;

	irq = devm_gpiod_get(mp->dev, MP2662_IRQ_PIN, GPIOD_IN);
	if (IS_ERR(irq)) {
		dev_err(mp->dev, "Could not probe irq pin.\n");
		return PTR_ERR(irq);
	}

	return gpiod_to_irq(irq);
}

static int mp2662_fw_read_u32_props(struct mp2662_device *mp)
{
	int ret;
	u32 property;
	int i;
	struct mp2662_init_data *init = &mp->init_data;
	struct {
		char *name;
		bool optional;
		enum mp2662_table_ids tbl_id;
		u8 *conv_data; /* holds converted value from given property */
	} props[] = {
		/* required properties */
		{"mp,charge-current", false, TBL_ICHG, &init->ichg},
		{"mp,battery-regulation-voltage", false, TBL_VREG, &init->vreg},
		{"mp,termination-current", false, TBL_ITERM, &init->iterm},
		{"mp,precharge-current", false, TBL_ITERM, &init->iprechg},
		{"mp,minimum-sys-voltage", false, TBL_SYSVMIN, &init->sysvmin},

		/* optional properties */
		{"mp,thermal-regulation-threshold", true, TBL_TREG, &init->treg},
	};

	/* initialize data for optional properties */
	init->treg = 3; /* 120 degrees Celsius */

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = device_property_read_u32(mp->dev, props[i].name,
					       &property);
		if (ret < 0) {
			if (props[i].optional)
				continue;

			dev_err(mp->dev, "Unable to read property %d %s\n", ret,
				props[i].name);

			return ret;
		}

		*props[i].conv_data = mp2662_find_idx(property,
						       props[i].tbl_id);
	}

	return 0;
}

static int mp2662_fw_probe(struct mp2662_device *mp)
{
	int ret;

	ret = mp2662_fw_read_u32_props(mp);
	if (ret < 0)
		return ret;

	return 0;
}

static int mp2662_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct mp2662_device *mp;
	int ret;
	int i;

	mp = devm_kzalloc(dev, sizeof(*mp), GFP_KERNEL);
	if (!mp)
		return -ENOMEM;

	mp->client = client;
	mp->dev = dev;

	mutex_init(&mp->lock);

	mp->rmap = devm_regmap_init_i2c(client, &mp2662_regmap_config);
	if (IS_ERR(mp->rmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(mp->rmap);
	}

	for (i = 0; i < ARRAY_SIZE(mp2662_reg_fields); i++) {
		const struct reg_field *reg_fields = mp2662_reg_fields;

		mp->rmap_fields[i] = devm_regmap_field_alloc(dev, mp->rmap,
							     reg_fields[i]);
		if (IS_ERR(mp->rmap_fields[i])) {
			dev_err(dev, "Cannot allocate regmap field\n");
			return PTR_ERR(mp->rmap_fields[i]);
		}
	}
	i2c_set_clientdata(client, mp);

	if (!dev->platform_data) {
		ret = mp2662_fw_probe(mp);
		if (ret < 0) {
			dev_err(dev, "Cannot read device properties.\n");
			return ret;
		}
	} else {
		return -ENODEV;
	}

	ret = mp2662_hw_init(mp);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}

	if (client->irq <= 0)
		client->irq = mp2662_irq_probe(mp);

	if (client->irq < 0) {
		dev_err(dev, "No irq resource found.\n");
		return client->irq;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					mp2662_irq_handler_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					MP2662_IRQ_PIN, mp);
	if (ret) {
		dev_err(dev, "failed to request gpio IRQ\n");
		return ret;
	}

	ret = mp2662_power_supply_init(mp);
	if (ret < 0) {
		dev_err(dev, "Failed to register power supply\n");
		return ret;
	}
	return 0;
}

static void mp2662_remove(struct i2c_client *client)
{
	struct mp2662_device *mp = i2c_get_clientdata(client);

	power_supply_unregister(mp->charger);

	/* reset all registers to default values */
	mp2662_chip_reset(mp);
}

static const struct i2c_device_id mp2662_i2c_ids[] = {
	{ "mp2662", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mp2662_i2c_ids);

static const struct of_device_id mp2662_of_match[] = {
	{ .compatible = "mp,mp2662", },
	{},
};
MODULE_DEVICE_TABLE(of, mp2662_of_match);

static struct i2c_driver mp2662_driver = {
	.driver = {
		.name = "mp2662-charger",
		.of_match_table = of_match_ptr(mp2662_of_match),
	},
	.probe = mp2662_probe,
	.remove = mp2662_remove,
	.id_table = mp2662_i2c_ids,
};
module_i2c_driver(mp2662_driver);

MODULE_AUTHOR("Chloe Sun <dandan.sun@nxp.com>");
MODULE_DESCRIPTION("mp2662 charger driver");
MODULE_LICENSE("GPL");
