/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the PM service. These
 * function are ported to clients that communicate to the SC.
 *
 * @addtogroup PM_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/pm/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_pm_set_sys_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
				  sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_SET_SYS_POWER_MODE;
	RPC_D8(&msg, 0) = pt;
	RPC_D8(&msg, 1) = mode;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_sys_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
				  sc_pm_power_mode_t *mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_GET_SYS_POWER_MODE;
	RPC_D8(&msg, 0) = pt;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (mode != NULL)
		*mode = RPC_D8(&msg, 0);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_SET_RESOURCE_POWER_MODE;
	RPC_D16(&msg, 0) = resource;
	RPC_D8(&msg, 2) = mode;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t *mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_GET_RESOURCE_POWER_MODE;
	RPC_D16(&msg, 0) = resource;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (mode != NULL)
		*mode = RPC_D8(&msg, 0);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_SET_CLOCK_RATE;
	RPC_D32(&msg, 0) = *rate;
	RPC_D16(&msg, 4) = resource;
	RPC_D8(&msg, 6) = clk;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	*rate = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_GET_CLOCK_RATE;
	RPC_D16(&msg, 0) = resource;
	RPC_D8(&msg, 2) = clk;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (rate != NULL)
		*rate = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_clock_enable(sc_ipc_t ipc, sc_rsrc_t resource,
			    sc_pm_clk_t clk, bool enable, bool autog)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_CLOCK_ENABLE;
	RPC_D16(&msg, 0) = resource;
	RPC_D8(&msg, 2) = clk;
	RPC_D8(&msg, 3) = enable;
	RPC_D8(&msg, 4) = autog;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_reset(sc_ipc_t ipc, sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_RESET;
	RPC_D8(&msg, 0) = type;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_reset_reason(sc_ipc_t ipc, sc_pm_reset_reason_t *reason)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_RESET_REASON;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (reason != NULL)
		*reason = RPC_D8(&msg, 0);
	return (sc_err_t)result;
}

sc_err_t sc_pm_boot(sc_ipc_t ipc, sc_rm_pt_t pt,
		    sc_rsrc_t resource_cpu, sc_faddr_t boot_addr,
		    sc_rsrc_t resource_mu, sc_rsrc_t resource_dev)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_BOOT;
	RPC_D32(&msg, 0) = boot_addr >> 32;
	RPC_D32(&msg, 4) = boot_addr;
	RPC_D16(&msg, 8) = resource_cpu;
	RPC_D16(&msg, 10) = resource_mu;
	RPC_D16(&msg, 12) = resource_dev;
	RPC_D8(&msg, 14) = pt;
	RPC_SIZE(&msg) = 5;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_pm_reboot(sc_ipc_t ipc, sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_REBOOT;
	RPC_D8(&msg, 0) = type;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, true);

	return;
}

sc_err_t sc_pm_reboot_partition(sc_ipc_t ipc, sc_rm_pt_t pt,
				sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_REBOOT_PARTITION;
	RPC_D8(&msg, 0) = pt;
	RPC_D8(&msg, 1) = type;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_cpu_start(sc_ipc_t ipc, sc_rsrc_t resource, bool enable,
			 sc_faddr_t address)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PM;
	RPC_FUNC(&msg) = PM_FUNC_CPU_START;
	RPC_D32(&msg, 0) = address >> 32;
	RPC_D32(&msg, 4) = address;
	RPC_D16(&msg, 8) = resource;
	RPC_D8(&msg, 10) = enable;
	RPC_SIZE(&msg) = 4;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

/**@}*/
