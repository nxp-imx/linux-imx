/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the PAD RPC implementation.
 *
 * @addtogroup PAD_SVC
 * @{
 */

#ifndef _SC_PAD_RPC_H
#define _SC_PAD_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC PAD function calls.
 */
typedef enum pad_func_e {
	PAD_FUNC_UNKNOWN,	/* Unknown function */
	PAD_FUNC_SET_MUX,	/* Index for pad_set_mux() RPC call */
	PAD_FUNC_SET_GP,	/* Index for pad_set_gp() RPC call */
	PAD_FUNC_SET_GP_28LPP,	/* Index for pad_set_gp_28lpp() RPC call */
	PAD_FUNC_SET_WAKEUP,	/* Index for pad_set_wakeup() RPC call */
	PAD_FUNC_SET_ALL,	/* Index for pad_set_all() RPC call */
	PAD_FUNC_GET_MUX,	/* Index for pad_get_mux() RPC call */
	PAD_FUNC_GET_GP,	/* Index for pad_get_gp() RPC call */
	PAD_FUNC_GET_GP_28LPP,	/* Index for pad_get_gp_28lpp() RPC call */
	PAD_FUNC_GET_WAKEUP,	/* Index for pad_get_wakeup() RPC call */
	PAD_FUNC_GET_ALL,	/* Index for pad_get_all() RPC call */
	PAD_FUNC_SET_GP_28FDSOI,	/* Index for pad_set_gp_28fdsoi() RPC call */
	PAD_FUNC_GET_GP_28FDSOI,	/* Index for pad_get_gp_28fdsoi() RPC call */
	PAD_FUNC_SET_GP_28FDSOI_COMP,	/* Index for pad_set_gp_28fdsoi_comp() RPC call */
	PAD_FUNC_GET_GP_28FDSOI_COMP,	/* Index for pad_get_gp_28fdsoi_comp() RPC call */
	PAD_FUNC_SET,		/* Index for pad_set() RPC call */
	PAD_FUNC_GET,		/* Index for pad_get() RPC call */
} pad_func_t;

/* Functions */

/*!
 * This function dispatches an incoming PAD RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void pad_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an PAD RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void pad_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_PAD_RPC_H */

/**@}*/
