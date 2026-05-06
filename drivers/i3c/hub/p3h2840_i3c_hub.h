/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025-2026 NXP
 * This header file contain private device structure definition.
 */

#ifndef P3H2840_I3C_HUB_H
#define P3H2840_I3C_HUB_H

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/i3c/device.h>
#include <linux/i3c/hub.h>
#include <linux/i3c/master.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>

/* I3C HUB REGISTERS */

/* Device Information Registers */
#define P3H2X4X_DEV_INFO_0					0x00
#define P3H2X4X_DEV_INFO_1					0x01
#define P3H2X4X_PID_5						0x02
#define P3H2X4X_PID_4						0x03
#define P3H2X4X_PID_3						0x04
#define P3H2X4X_PID_2						0x05
#define P3H2X4X_PID_1						0x06
#define P3H2X4X_PID_0						0x07
#define P3H2X4X_BCR						0x08
#define P3H2X4X_DCR						0x09
#define P3H2X4X_DEV_CAPAB					0x0a
#define P3H2X4X_DEV_REV						0x0b

/* Device Configuration Registers */
#define P3H2X4X_CP_CONF						0x11
#define P3H2X4X_TP_ENABLE					0x12

#define P3H2X4X_DEV_CONF					0x13
#define P3H2X4X_IO_STRENGTH					0x14
#define P3H2X4X_TP0145_IO_STRENGTH_MASK				GENMASK(1, 0)
#define P3H2X4X_TP0145_IO_STRENGTH(x)	\
		FIELD_PREP(P3H2X4X_TP0145_IO_STRENGTH_MASK, x)
#define P3H2X4X_TP2367_IO_STRENGTH_MASK				GENMASK(3, 2)
#define P3H2X4X_TP2367_IO_STRENGTH(x)	\
		FIELD_PREP(P3H2X4X_TP2367_IO_STRENGTH_MASK, x)
#define P3H2X4X_CP0_IO_STRENGTH_MASK				GENMASK(5, 4)
#define P3H2X4X_CP0_IO_STRENGTH(x)	\
		FIELD_PREP(P3H2X4X_CP0_IO_STRENGTH_MASK, x)
#define P3H2X4X_CP1_IO_STRENGTH_MASK				GENMASK(7, 6)
#define P3H2X4X_CP1_IO_STRENGTH(x)	\
		FIELD_PREP(P3H2X4X_CP1_IO_STRENGTH_MASK, x)
#define P3H2X4X_IO_STRENGTH_MASK					GENMASK(7, 0)

#define P3H2X4X_TP_IO_MODE_CONF					0x17
#define P3H2X4X_TP_SMBUS_AGNT_EN				0x18

#define P3H2X4X_LDO_AND_PULLUP_CONF				0x19

#define P3H2X4X_TP0145_PULLUP_CONF_MASK				GENMASK(7, 6)
#define P3H2X4X_TP0145_PULLUP_CONF(x)	\
		FIELD_PREP(P3H2X4X_TP0145_PULLUP_CONF_MASK, x)
#define P3H2X4X_TP2367_PULLUP_CONF_MASK				GENMASK(5, 4)
#define P3H2X4X_TP2367_PULLUP_CONF(x)	\
		FIELD_PREP(P3H2X4X_TP2367_PULLUP_CONF_MASK, x)
#define P3H2X4X_PULLUP_CONF_MASK					GENMASK(7, 4)

#define P3H2X4X_CP_IBI_CONF					0x1a

#define P3H2X4X_TP_SMBUS_AGNT_IBI_CONFIG			0x1b

#define P3H2X4X_IBI_MDB_CUSTOM					0x1c
#define P3H2X4X_JEDEC_CONTEXT_ID				0x1d
#define P3H2X4X_TP_GPIO_MODE_EN					0x1e

/* Device Status and IBI Registers */
#define P3H2X4X_DEV_AND_IBI_STS					0x20
#define P3H2X4X_TP_SMBUS_AGNT_IBI_STS				0x21
#define P3H2X4X_SMBUS_AGENT_EVENT_FLAG_STATUS			BIT(4)

/* Controller Port Control/Status Registers */
#define P3H2X4X_CP_MUX_SET					0x38
#define P3H2X4X_CONTROLLER_PORT_MUX_REQ				BIT(0)
#define P3H2X4X_CP_MUX_STS					0x39
#define P3H2X4X_CONTROLLER_PORT_MUX_CONNECTION_STATUS		BIT(0)

/* Target Ports Control Registers */
#define P3H2X4X_TP_SMBUS_AGNT_TRANS_START			0x50
#define P3H2X4X_TP_NET_CON_CONF					0x51

#define P3H2X4X_TP_PULLUP_EN					0x53

#define P3H2X4X_TP_SCL_OUT_EN					0x54
#define P3H2X4X_TP_SDA_OUT_EN					0x55
#define P3H2X4X_TP_SCL_OUT_LEVEL				0x56
#define P3H2X4X_TP_SDA_OUT_LEVEL				0x57
#define P3H2X4X_TP_IN_DETECT_MODE_CONF				0x58
#define P3H2X4X_TP_SCL_IN_DETECT_IBI_EN				0x59
#define P3H2X4X_TP_SDA_IN_DETECT_IBI_EN				0x5a

/* Target Ports Status Registers */
#define P3H2X4X_TP_SCL_IN_LEVEL_STS				0x60
#define P3H2X4X_TP_SDA_IN_LEVEL_STS				0x61
#define P3H2X4X_TP_SCL_IN_DETECT_FLG				0x62
#define P3H2X4X_TP_SDA_IN_DETECT_FLG				0x63

/* SMBus Agent Configuration and Status Registers */
#define P3H2X4X_TP0_SMBUS_AGNT_STS				0x64
#define P3H2X4X_TP1_SMBUS_AGNT_STS				0x65
#define P3H2X4X_TP2_SMBUS_AGNT_STS				0x66
#define P3H2X4X_TP3_SMBUS_AGNT_STS				0x67
#define P3H2X4X_TP4_SMBUS_AGNT_STS				0x68
#define P3H2X4X_TP5_SMBUS_AGNT_STS				0x69
#define P3H2X4X_TP6_SMBUS_AGNT_STS				0x6a
#define P3H2X4X_TP7_SMBUS_AGNT_STS				0x6b
#define P3H2X4X_ONCHIP_TD_AND_SMBUS_AGNT_CONF			0x6c

/* buf receive flag set */
#define P3H2X4X_TARGET_BUF_CA_TF				BIT(0)
#define P3H2X4X_TARGET_BUF_0_RECEIVE				BIT(1)
#define P3H2X4X_TARGET_BUF_1_RECEIVE				BIT(2)
#define P3H2X4X_TARGET_BUF_0_1_RECEIVE				GENMASK(2, 1)
#define P3H2X4X_TARGET_BUF_OVRFL				GENMASK(3, 1)
#define BUF_RECEIVED_FLAG_MASK					GENMASK(3, 1)
#define BUF_RECEIVED_FLAG_TF_MASK				GENMASK(3, 0)

#define P3H2X4X_TARGET_AGENT_LOCAL_DEV				0x11
#define P3H2X4X_TARGET_BUFF_0_PAGE				0x12
#define P3H2X4X_TARGET_BUFF_1_PAGE				0x13

/* Special Function Registers */
#define P3H2X4X_LDO_AND_CPSEL_STS				0x79
#define P3H2X4X_CP_SDA1_LEVEL					BIT(7)
#define P3H2X4X_CP_SCL1_LEVEL					BIT(6)

#define P3H2X4X_CP_SEL_PIN_INPUT_CODE_MASK			GENMASK(5, 4)
#define P3H2X4X_CP_SEL_PIN_INPUT_CODE_GET(x)	\
		(((x) & P3H2X4X_CP_SEL_PIN_INPUT_CODE_MASK) >> 4)
#define P3H2X4X_CP_SDA1_SCL1_PINS_CODE_MASK			GENMASK(7, 6)
#define P3H2X4X_CP_SDA1_SCL1_PINS_CODE_GET(x)	\
		(((x) & P3H2X4X_CP_SDA1_SCL1_PINS_CODE_MASK) >> 6)
#define P3H2X4X_VCCIO1_PWR_GOOD					BIT(3)
#define P3H2X4X_VCCIO0_PWR_GOOD					BIT(2)
#define P3H2X4X_CP1_VCCIO_PWR_GOOD				BIT(1)
#define P3H2X4X_CP0_VCCIO_PWR_GOOD				BIT(0)

#define P3H2X4X_BUS_RESET_SCL_TIMEOUT				0x7a
#define P3H2X4X_ONCHIP_TD_PROTO_ERR_FLG				0x7b
#define P3H2X4X_DEV_CMD						0x7c
#define P3H2X4X_ONCHIP_TD_STS					0x7d
#define P3H2X4X_ONCHIP_TD_ADDR_CONF				0x7e
#define P3H2X4X_PAGE_PTR					0x7f

/* Paged Transaction Registers */
#define P3H2X4X_CONTROLLER_BUFFER_PAGE				0x10
#define P3H2X4X_CONTROLLER_AGENT_BUFF				0x80
#define P3H2X4X_CONTROLLER_AGENT_BUFF_DATA			0x84

#define P3H2X4X_TARGET_BUFF_LENGTH				0x80
#define P3H2X4X_TARGET_BUFF_ADDRESS				0x81
#define P3H2X4X_TARGET_BUFF_DATA				0x82

#define P3H2X4X_TP_MAX_COUNT					0x08
#define P3H2X4X_CP_MAX_COUNT					0x02
#define P3H2X4X_TP_LOCAL_DEV					0x08

/* LDO Disable/Enable DT settings */
#define P3H2X4X_LDO_VOLT_1_0V					0x00
#define P3H2X4X_LDO_VOLT_1_1V					0x01
#define P3H2X4X_LDO_VOLT_1_2V					0x02
#define P3H2X4X_LDO_VOLT_1_8V					0x03

#define P3H2X4X_LDO_DISABLED					0x00
#define P3H2X4X_LDO_ENABLED					0x01

#define P3H2X4X_IBI_DISABLED					0x00
#define P3H2X4X_IBI_ENABLED					0x01

/* Pullup selection DT settings */
#define P3H2X4X_TP_PULLUP_250R					0x00
#define P3H2X4X_TP_PULLUP_500R					0x01
#define P3H2X4X_TP_PULLUP_1000R					0x02
#define P3H2X4X_TP_PULLUP_2000R					0x03

#define P3H2X4X_TP_PULLUP_DISABLED				0x00
#define P3H2X4X_TP_PULLUP_ENABLED				0x01

#define P3H2X4X_IO_STRENGTH_20_OHM				0x00
#define P3H2X4X_IO_STRENGTH_30_OHM				0x01
#define P3H2X4X_IO_STRENGTH_40_OHM				0x02
#define P3H2X4X_IO_STRENGTH_50_OHM				0x03

#define P3H2X4X_TP_MODE_I3C					0x00
#define P3H2X4X_TP_MODE_SMBUS					0x01
#define P3H2X4X_TP_MODE_GPIO					0x02
#define P3H2X4X_TP_MODE_I2C					0x03

#define ONE_BYTE_SIZE						0x01

/* holding SDA low when both SMBus Target Agent received data buffers are full.
 * This feature can be used as a flow-control mechanism for MCTP applications to
 * avoid MCTP transmitters on Target Ports time out when the SMBus agent buffers
 * are not serviced in time by upstream controller and only receives write message
 * from its downstream ports.
 * SMBUS_AGENT_TX_RX_LOOPBACK_EN/TARGET_AGENT_BUF_FULL_SDA_LOW_EN
 */

#define P3H2X4X_TARGET_AGENT_DFT_IBI_CONF			0x20
#define P3H2X4X_TARGET_AGENT_DFT_IBI_CONF_MASK			0x21

/* Transaction status checking mask */

#define P3H2X4X_SMBUS_TRANSACTION_FINISH_FLAG		1
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SHIFT		4

#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_OK		0
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_ADDR_NAK		1
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_DATA_NAK		2
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_WTR_NAK		3
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SYNC_RCV		4
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SYNC_RCVCLR	5
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_FAULT		6
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_ARB_LOSS		7
#define P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SCL_TO		8

#define P3H2X4X_TP_BUFFER_STATUS_MASK				0x0f
#define P3H2X4X_TP_TRANSACTION_CODE_MASK			0xf0

/* SMBus transaction types fields */
#define P3H2X4X_SMBUS_400KHZ					BIT(2)

/* SMBus polling */
#define P3H2X4X_POLLING_ROLL_PERIOD_MS				10

/* Hub buffer size */
#define P3H2X4X_CONTROLLER_BUFFER_SIZE				88
#define P3H2X4X_TARGET_BUFFER_SIZE				80
#define P3H2X4X_SMBUS_DESCRIPTOR_SIZE				4
#define P3H2X4X_SMBUS_PAYLOAD_SIZE	\
		(P3H2X4X_CONTROLLER_BUFFER_SIZE - P3H2X4X_SMBUS_DESCRIPTOR_SIZE)
#define P3H2X4X_SMBUS_TARGET_PAYLOAD_SIZE	(P3H2X4X_TARGET_BUFFER_SIZE - 2)

/* Hub SMBus transaction time */
#define P3H2X4X_SMBUS_400KHZ_TRANSFER_TIMEOUT(x)		((20 * (x)) + 80)

#define P3H2X4X_NO_PAGE_PER_TP					4

#define P3H2X4X_MAX_PAYLOAD_LEN					2
#define P3H2X4X_NUM_SLOTS					6

#define P3H2X4X_HUB_ID						0

#define P3H2X4X_SET_BIT(n)				BIT(n)

enum p3h2x4x_tp {
	TP_0,
	TP_1,
	TP_2,
	TP_3,
	TP_4,
	TP_5,
	TP_6,
	TP_7,
};

enum p3h2x4x_rcv_buf {
	RCV_BUF_0,
	RCV_BUF_1,
	RCV_BUF_OF,
};

struct tp_configuration {
	bool pullup_en;
	bool ibi_en;
	bool always_enable;
	int mode;
};

struct hub_configuration {
	int tp0145_pullup;
	int tp2367_pullup;
	int cp0_io_strength;
	int cp1_io_strength;
	int tp0145_io_strength;
	int tp2367_io_strength;
	struct tp_configuration tp_config[P3H2X4X_TP_MAX_COUNT];
};

struct tp_bus {
	bool is_registered;	    /* bus was registered in the framework. */
	u8 tp_mask;
	u8 tp_port;
	struct mutex port_mutex;      /* per port mutex */
	struct device_node *of_node;
	struct i2c_client *tp_smbus_client;
	struct i2c_adapter *tp_smbus_adapter;
	struct i3c_hub_controller hub_controller;
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
};

struct p3h2x4x_i3c_hub_dev {
	u8 reg_addr;				/* Offset for reading HUB's register. */
	struct dentry *sysfs_debug_dir;
	struct device *dev;
	struct regmap *regmap;
	struct mutex etx_mutex;      /* all port mutex */
	struct i3c_device *i3cdev;
	struct i2c_client *i2c_client;
	struct hub_configuration hub_config;
	struct tp_bus tp_bus[P3H2X4X_TP_MAX_COUNT];
	struct i3c_hub *hub;
};

/**
 * p3h2x4x_tp_smbus_algo - add i2c adapter for target port configured as SMBus.
 * @priv: p3h2x4x device structure.
 * @tp: target port.
 * Return: 0 in case of success, a negative EINVAL code if the error.
 */
int p3h2x4x_tp_smbus_algo(struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub);

/**
 * p3h2x4x_tp_i3c_algo - register i3c controller for target port configured as I3C.
 * @priv: p3h2x4x device structure.
 * @tp: target port.
 * Return: 0 in case of success, a negative EINVAL code if the error.
 */
int p3h2x4x_tp_i3c_algo(struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub);

/**
 * p3h2x4x_ibi_handler - IBI handler.
 * @i3cdev: i3c device.
 * @payload: two byte IBI payload data.
 */
void p3h2x4x_ibi_handler(struct i3c_device *i3cdev,
			 const struct i3c_ibi_payload *payload);
#endif /* P3H2840_I3C_HUB_H */
