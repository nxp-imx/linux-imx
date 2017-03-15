/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the RM RPC implementation.
 *
 * @addtogroup RM_SVC
 * @{
 */

#ifndef _SC_RM_RPC_H
#define _SC_RM_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC RM function calls.
 */
typedef enum rm_func_e {
	RM_FUNC_UNKNOWN,	/* Unknown function */
	RM_FUNC_PARTITION_ALLOC,	/* Index for rm_partition_alloc() RPC call */
	RM_FUNC_PARTITION_FREE,	/* Index for rm_partition_free() RPC call */
	RM_FUNC_PARTITION_STATIC,	/* Index for rm_partition_static() RPC call */
	RM_FUNC_PARTITION_LOCK,	/* Index for rm_partition_lock() RPC call */
	RM_FUNC_GET_PARTITION,	/* Index for rm_get_partition() RPC call */
	RM_FUNC_SET_PARENT,	/* Index for rm_set_parent() RPC call */
	RM_FUNC_MOVE_ALL,	/* Index for rm_move_all() RPC call */
	RM_FUNC_ASSIGN_RESOURCE,	/* Index for rm_assign_resource() RPC call */
	RM_FUNC_SET_RESOURCE_MOVABLE,	/* Index for rm_set_resource_movable() RPC call */
	RM_FUNC_SET_MASTER_ATTRIBUTES,	/* Index for rm_set_master_attributes() RPC call */
	RM_FUNC_SET_MASTER_SID,	/* Index for rm_set_master_sid() RPC call */
	RM_FUNC_SET_PERIPHERAL_PERMISSIONS,	/* Index for rm_set_peripheral_permissions() RPC call */
	RM_FUNC_IS_RESOURCE_OWNED,	/* Index for rm_is_resource_owned() RPC call */
	RM_FUNC_IS_RESOURCE_MASTER,	/* Index for rm_is_resource_master() RPC call */
	RM_FUNC_IS_RESOURCE_PERIPHERAL,	/* Index for rm_is_resource_peripheral() RPC call */
	RM_FUNC_GET_RESOURCE_INFO,	/* Index for rm_get_resource_info() RPC call */
	RM_FUNC_MEMREG_ALLOC,	/* Index for rm_memreg_alloc() RPC call */
	RM_FUNC_MEMREG_FREE,	/* Index for rm_memreg_free() RPC call */
	RM_FUNC_ASSIGN_MEMREG,	/* Index for rm_assign_memreg() RPC call */
	RM_FUNC_SET_MEMREG_PERMISSIONS,	/* Index for rm_set_memreg_permissions() RPC call */
	RM_FUNC_IS_MEMREG_OWNED,	/* Index for rm_is_memreg_owned() RPC call */
	RM_FUNC_GET_MEMREG_INFO,	/* Index for rm_get_memreg_info() RPC call */
	RM_FUNC_ASSIGN_PIN,	/* Index for rm_assign_pin() RPC call */
	RM_FUNC_SET_PIN_MOVABLE,	/* Index for rm_set_pin_movable() RPC call */
	RM_FUNC_IS_PIN_OWNED,	/* Index for rm_is_pin_owned() RPC call */
	RM_FUNC_GET_DID,	/* Index for rm_get_did() RPC call */
} rm_func_t;

/* Functions */

/*!
 * This function dispatches an incoming RM RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void rm_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an RM RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void rm_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_RM_RPC_H */

/**@}*/
