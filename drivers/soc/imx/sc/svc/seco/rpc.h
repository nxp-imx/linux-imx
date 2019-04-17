/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the SECO RPC implementation.
 *
 * @addtogroup SECO_SVC
 * @{
 */

#ifndef SC_SECO_RPC_H
#define SC_SECO_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC SECO function calls
 */
/*@{*/
#define SECO_FUNC_UNKNOWN 0 /* Unknown function */
#define SECO_FUNC_IMAGE_LOAD 1U /* Index for seco_image_load() RPC call */
#define SECO_FUNC_AUTHENTICATE 2U /* Index for seco_authenticate() RPC call */
#define SECO_FUNC_FORWARD_LIFECYCLE 3U /* Index for seco_forward_lifecycle() RPC call */
#define SECO_FUNC_RETURN_LIFECYCLE 4U /* Index for seco_return_lifecycle() RPC call */
#define SECO_FUNC_COMMIT 5U /* Index for seco_commit() RPC call */
#define SECO_FUNC_ATTEST_MODE 6U /* Index for seco_attest_mode() RPC call */
#define SECO_FUNC_ATTEST 7U /* Index for seco_attest() RPC call */
#define SECO_FUNC_GET_ATTEST_PKEY 8U /* Index for seco_get_attest_pkey() RPC call */
#define SECO_FUNC_GET_ATTEST_SIGN 9U /* Index for seco_get_attest_sign() RPC call */
#define SECO_FUNC_ATTEST_VERIFY 10U /* Index for seco_attest_verify() RPC call */
#define SECO_FUNC_GEN_KEY_BLOB 11U /* Index for seco_gen_key_blob() RPC call */
#define SECO_FUNC_LOAD_KEY 12U /* Index for seco_load_key() RPC call */
#define SECO_FUNC_GET_MP_KEY 13U /* Index for seco_get_mp_key() RPC call */
#define SECO_FUNC_UPDATE_MPMR 14U /* Index for seco_update_mpmr() RPC call */
#define SECO_FUNC_GET_MP_SIGN 15U /* Index for seco_get_mp_sign() RPC call */
#define SECO_FUNC_BUILD_INFO 16U /* Index for seco_build_info() RPC call */
#define SECO_FUNC_CHIP_INFO 17U /* Index for seco_chip_info() RPC call */
#define SECO_FUNC_ENABLE_DEBUG 18U /* Index for seco_enable_debug() RPC call */
#define SECO_FUNC_GET_EVENT 19U /* Index for seco_get_event() RPC call */
#define SECO_FUNC_FUSE_WRITE 20U /* Index for seco_fuse_write() RPC call */
#define SECO_FUNC_PATCH 21U /* Index for seco_patch() RPC call */
#define SECO_FUNC_START_RNG 22U /* Index for seco_start_rng() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming SECO RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void seco_dispatch(sc_rm_pt_t caller_pt, sc_rsrc_t mu, sc_rpc_msg_t *msg);

#endif /* SC_SECO_RPC_H */

/**@}*/

