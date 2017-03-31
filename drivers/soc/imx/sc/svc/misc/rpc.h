/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the MISC RPC implementation.
 *
 * @addtogroup MISC_SVC
 * @{
 */

#ifndef _SC_MISC_RPC_H
#define _SC_MISC_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC MISC function calls.
 */
typedef enum misc_func_e {
	MISC_FUNC_UNKNOWN = 0,	/* Unknown function */
	MISC_FUNC_SET_CONTROL = 1,	/* Index for misc_set_control() RPC call */
	MISC_FUNC_GET_CONTROL = 2,	/* Index for misc_get_control() RPC call */
	MISC_FUNC_SET_MAX_DMA_GROUP = 4,	/* Index for misc_set_max_dma_group() RPC call */
	MISC_FUNC_SET_DMA_GROUP = 5,	/* Index for misc_set_dma_group() RPC call */
	MISC_FUNC_SECO_IMAGE_LOAD = 8,	/* Index for misc_seco_image_load() RPC call */
	MISC_FUNC_SECO_AUTHENTICATE = 9,	/* Index for misc_seco_authenticate() RPC call */
	MISC_FUNC_DEBUG_OUT = 10,	/* Index for misc_debug_out() RPC call */
	MISC_FUNC_WAVEFORM_CAPTURE = 6,	/* Index for misc_waveform_capture() RPC call */
	MISC_FUNC_SET_ARI = 3,	/* Index for misc_set_ari() RPC call */
	MISC_FUNC_BOOT_STATUS = 7,	/* Index for misc_boot_status() RPC call */
} misc_func_t;

/* Functions */

/*!
 * This function dispatches an incoming MISC RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void misc_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an MISC RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void misc_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_MISC_RPC_H */

/**@}*/
