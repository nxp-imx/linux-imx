/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the PAD service. These
 * function are ported to clients that communicate to the SC.
 *
 * @addtogroup PAD_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/pad/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_pad_set_mux(sc_ipc_t ipc, sc_pin_t pin,
			uint8_t mux, sc_pad_config_t config, sc_pad_iso_t iso)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_MUX;
	RPC_D16(&msg, 0) = pin;
	RPC_D8(&msg, 2) = mux;
	RPC_D8(&msg, 3) = config;
	RPC_D8(&msg, 4) = iso;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_mux(sc_ipc_t ipc, sc_pin_t pin,
			uint8_t *mux, sc_pad_config_t *config,
			sc_pad_iso_t *iso)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_MUX;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (mux != NULL)
		*mux = RPC_D8(&msg, 0);
	if (config != NULL)
		*config = RPC_D8(&msg, 1);
	if (iso != NULL)
		*iso = RPC_D8(&msg, 2);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp(sc_ipc_t ipc, sc_pin_t pin, uint32_t ctrl)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_GP;
	RPC_D32(&msg, 0) = ctrl;
	RPC_D16(&msg, 4) = pin;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp(sc_ipc_t ipc, sc_pin_t pin, uint32_t *ctrl)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_GP;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (ctrl != NULL)
		*ctrl = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_wakeup(sc_ipc_t ipc, sc_pin_t pin, sc_pad_wakeup_t wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_WAKEUP;
	RPC_D16(&msg, 0) = pin;
	RPC_D8(&msg, 2) = wakeup;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_wakeup(sc_ipc_t ipc, sc_pin_t pin, sc_pad_wakeup_t *wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_WAKEUP;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (wakeup != NULL)
		*wakeup = RPC_D8(&msg, 0);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_all(sc_ipc_t ipc, sc_pin_t pin, uint8_t mux,
			sc_pad_config_t config, sc_pad_iso_t iso, uint32_t ctrl,
			sc_pad_wakeup_t wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_ALL;
	RPC_D32(&msg, 0) = ctrl;
	RPC_D16(&msg, 4) = pin;
	RPC_D8(&msg, 6) = mux;
	RPC_D8(&msg, 7) = config;
	RPC_D8(&msg, 8) = iso;
	RPC_D8(&msg, 9) = wakeup;
	RPC_SIZE(&msg) = 4;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_all(sc_ipc_t ipc, sc_pin_t pin, uint8_t *mux,
			sc_pad_config_t *config, sc_pad_iso_t *iso,
			uint32_t *ctrl, sc_pad_wakeup_t *wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_ALL;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (ctrl != NULL)
		*ctrl = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	if (mux != NULL)
		*mux = RPC_D8(&msg, 4);
	if (config != NULL)
		*config = RPC_D8(&msg, 5);
	if (iso != NULL)
		*iso = RPC_D8(&msg, 6);
	if (wakeup != NULL)
		*wakeup = RPC_D8(&msg, 7);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set(sc_ipc_t ipc, sc_pin_t pin, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET;
	RPC_D32(&msg, 0) = val;
	RPC_D16(&msg, 4) = pin;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get(sc_ipc_t ipc, sc_pin_t pin, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (val != NULL)
		*val = RPC_D32(&msg, 0);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28lpp(sc_ipc_t ipc, sc_pin_t pin,
			     sc_pad_28lpp_dse_t dse, bool sre, bool hys,
			     bool pe, sc_pad_28lpp_ps_t ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_GP_28LPP;
	RPC_D16(&msg, 0) = pin;
	RPC_D8(&msg, 2) = dse;
	RPC_D8(&msg, 3) = ps;
	RPC_D8(&msg, 4) = sre;
	RPC_D8(&msg, 5) = hys;
	RPC_D8(&msg, 6) = pe;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28lpp(sc_ipc_t ipc, sc_pin_t pin,
			     sc_pad_28lpp_dse_t *dse, bool *sre, bool *hys,
			     bool *pe, sc_pad_28lpp_ps_t *ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_GP_28LPP;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (dse != NULL)
		*dse = RPC_D8(&msg, 0);
	if (ps != NULL)
		*ps = RPC_D8(&msg, 1);
	if (sre != NULL)
		*sre = RPC_D8(&msg, 2);
	if (hys != NULL)
		*hys = RPC_D8(&msg, 3);
	if (pe != NULL)
		*pe = RPC_D8(&msg, 4);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28fdsoi(sc_ipc_t ipc, sc_pin_t pin,
			       sc_pad_28fdsoi_dse_t dse, sc_pad_28fdsoi_ps_t ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_GP_28FDSOI;
	RPC_D16(&msg, 0) = pin;
	RPC_D8(&msg, 2) = dse;
	RPC_D8(&msg, 3) = ps;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28fdsoi(sc_ipc_t ipc, sc_pin_t pin,
			       sc_pad_28fdsoi_dse_t *dse,
			       sc_pad_28fdsoi_ps_t *ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_GP_28FDSOI;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (dse != NULL)
		*dse = RPC_D8(&msg, 0);
	if (ps != NULL)
		*ps = RPC_D8(&msg, 1);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pin_t pin,
				    uint8_t compen, bool fastfrz,
				    uint8_t rasrcp, uint8_t rasrcn,
				    bool nasrc_sel)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_SET_GP_28FDSOI_COMP;
	RPC_D16(&msg, 0) = pin;
	RPC_D8(&msg, 2) = compen;
	RPC_D8(&msg, 3) = rasrcp;
	RPC_D8(&msg, 4) = rasrcn;
	RPC_D8(&msg, 5) = fastfrz;
	RPC_D8(&msg, 6) = nasrc_sel;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pin_t pin,
				    uint8_t *compen, bool *fastfrz,
				    uint8_t *rasrcp, uint8_t *rasrcn,
				    bool *nasrc_sel, bool *compok,
				    uint8_t *nasrc)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = SC_RPC_SVC_PAD;
	RPC_FUNC(&msg) = PAD_FUNC_GET_GP_28FDSOI_COMP;
	RPC_D16(&msg, 0) = pin;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	if (compen != NULL)
		*compen = RPC_D8(&msg, 0);
	if (rasrcp != NULL)
		*rasrcp = RPC_D8(&msg, 1);
	if (rasrcn != NULL)
		*rasrcn = RPC_D8(&msg, 2);
	if (nasrc != NULL)
		*nasrc = RPC_D8(&msg, 3);
	if (fastfrz != NULL)
		*fastfrz = RPC_D8(&msg, 4);
	if (nasrc_sel != NULL)
		*nasrc_sel = RPC_D8(&msg, 5);
	if (compok != NULL)
		*compok = RPC_D8(&msg, 6);
	return (sc_err_t)result;
}

/**@}*/
