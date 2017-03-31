/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the MISC service. These
 * function are ported to clients that communicate to the SC.
 *
 * @addtogroup MISC_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/misc/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_misc_set_control(sc_ipc_t ipc, sc_rsrc_t resource,
			     sc_ctrl_t ctrl, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SET_CONTROL;
	RPC_D32(&msg, 0) = ctrl;
	RPC_D32(&msg, 4) = val;
	RPC_D16(&msg, 8) = resource;
	RPC_SIZE(&msg) = 4;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_get_control(sc_ipc_t ipc, sc_rsrc_t resource,
			     sc_ctrl_t ctrl, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_GET_CONTROL;
	RPC_D32(&msg, 0) = ctrl;
	RPC_D16(&msg, 4) = resource;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	if (val != NULL)
		*val = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_max_dma_group(sc_ipc_t ipc, sc_rm_pt_t pt,
				   sc_misc_dma_group_t max)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SET_MAX_DMA_GROUP;
	RPC_D8(&msg, 0) = pt;
	RPC_D8(&msg, 1) = max;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_dma_group(sc_ipc_t ipc, sc_rsrc_t resource,
			       sc_misc_dma_group_t group)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SET_DMA_GROUP;
	RPC_D16(&msg, 0) = resource;
	RPC_D8(&msg, 2) = group;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_image_load(sc_ipc_t ipc, uint32_t addr_src,
				 uint32_t addr_dst, uint32_t len, bool fw)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SECO_IMAGE_LOAD;
	RPC_D32(&msg, 0) = addr_src;
	RPC_D32(&msg, 4) = addr_dst;
	RPC_D32(&msg, 8) = len;
	RPC_D8(&msg, 12) = fw;
	RPC_SIZE(&msg) = 5;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_authenticate(sc_ipc_t ipc,
				   sc_misc_seco_auth_cmd_t cmd,
				   uint32_t addr_meta)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SECO_AUTHENTICATE;
	RPC_D32(&msg, 0) = addr_meta;
	RPC_D8(&msg, 4) = cmd;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_debug_out(sc_ipc_t ipc, uint8_t ch)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_DEBUG_OUT;
	RPC_D8(&msg, 0) = ch;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	return;
}

sc_err_t sc_misc_waveform_capture(sc_ipc_t ipc, bool enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_WAVEFORM_CAPTURE;
	RPC_D8(&msg, 0) = enable;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_ari(sc_ipc_t ipc, sc_rsrc_t resource,
			 sc_rsrc_t resource_mst, uint16_t ari, bool enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_SET_ARI;
	RPC_D16(&msg, 0) = resource;
	RPC_D16(&msg, 2) = resource_mst;
	RPC_D16(&msg, 4) = ari;
	RPC_D8(&msg, 6) = enable;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_boot_status(sc_ipc_t ipc, sc_misc_boot_status_t status)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = MISC_FUNC_BOOT_STATUS;
	RPC_D8(&msg, 0) = status;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, true);

	return;
}

/**@}*/
