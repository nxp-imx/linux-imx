/*
 * This is the MCC library header file
 *
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * SPDX-License-Identifier: GPL-2.0+ and/or BSD-3-Clause
 * The GPL-2.0+ license for this file can be found in the COPYING.GPL file
 * included with this distribution or at
 * http://www.gnu.org/licenses/gpl-2.0.html
 * The BSD-3-Clause License for this file can be found in the COPYING.BSD file
 * included with this distribution or at
 * http://opensource.org/licenses/BSD-3-Clause
 */

#ifndef __MCC_COMMON__
#define __MCC_COMMON__

#define MCC_INIT_STRING    "mccisrd"
#define MCC_VERSION_STRING "002.000"
#define null ((void*)0)

/*!
 * \brief MCC_BOOLEAN type.
 *
 * Boolean type definiton for the MCC library.
 */
typedef unsigned int MCC_BOOLEAN;

/*!
 * \brief MCC_MEM_SIZE type.
 *
 * Mem size type definiton for the MCC library.
 */
typedef unsigned int MCC_MEM_SIZE;

/*!
 * \brief MCC_CORE type.
 *
 * This unsigned integer value specifies the core number for the endpoint definition.
 *
 * \see MCC_NODE
 * \see MCC_PORT
 * \see MCC_ENDPOINT
 */
typedef unsigned int MCC_CORE;

/*!
 * \brief MCC_NODE type.
 *
 * This unsigned integer value specifies the node number for the endpoint definition.
 *
 * \see MCC_CORE
 * \see MCC_PORT
 * \see MCC_ENDPOINT
 */
typedef unsigned int MCC_NODE;

/*!
 * \brief MCC_PORT type.
 *
 * This unsigned integer value specifies the port number for the endpoint definition.
 *
 * \see MCC_CORE
 * \see MCC_NODE
 * \see MCC_ENDPOINT
 */
typedef unsigned int MCC_PORT;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief Endpoint structure.
 *
 * Endpoints are receive buffer queues, implemented in shared RAM, 
 * and are addressed by a triplet containing core, node, and port.
 *
 * \see MCC_BOOKEEPING_STRUCT
 * \see MCC_CORE
 * \see MCC_NODE
 * \see MCC_PORT
 */
struct mcc_endpoint {
    /*! \brief Core number - identifies the core within the processor */
    MCC_CORE core;

    /*! \brief Node number - in Linux any user process participating in MCC is a unique node;
    MQX has only one node */
    MCC_NODE node;

    /*! \brief Port number - both Linux and MQX can have an arbitrary number of ports per node */
    MCC_PORT port;
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_endpoint MCC_ENDPOINT;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief Receive buffer structure.
 *
 * This is the receive buffer structure used for exchanging data.
 *
 * \see MCC_BOOKEEPING_STRUCT
 */
struct mcc_receive_buffer {
    /*! \brief Pointer to the next receive buffer */
    struct mcc_receive_buffer *next;

    /*! \brief Source endpoint */
    MCC_ENDPOINT source;

    /*! \brief Length of data stored in this buffer */
    MCC_MEM_SIZE data_len;

    /*! \brief Space for data storage */
    char data [MCC_ATTR_BUFFER_SIZE_IN_BYTES];
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_receive_buffer MCC_RECEIVE_BUFFER;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief List of buffers.
 *
 * Each endpoint keeps the list of received buffers.
 * The list of free buffers is kept in bookkeeping data structure. 
 *
 * \see MCC_RECEIVE_BUFFER
 * \see MCC_BOOKEEPING_STRUCT
 */
struct mcc_receive_list {
    /*! \brief Head of a buffers list */
    MCC_RECEIVE_BUFFER * head;

    /*! \brief Tail of a buffers list */
    MCC_RECEIVE_BUFFER * tail;
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_receive_list MCC_RECEIVE_LIST;

#define BUFFER_QUEUED (0)
#define BUFFER_FREED  (1)
typedef unsigned int MCC_SIGNAL_TYPE;
#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief Signals and signal queues.
 *
 * This is one item of a signal queue.
 *
 * \see MCC_SIGNAL_TYPE
 * \see MCC_ENDPOINT
 * \see MCC_BOOKEEPING_STRUCT
 */
struct mcc_signal {
    /*! \brief Signal type - BUFFER_QUEUED or BUFFER_FREED */
    MCC_SIGNAL_TYPE type;

    /*! \brief Destination endpoint */
    MCC_ENDPOINT    destination;
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_signal MCC_SIGNAL;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief Endpoint registration table.
 *
 * This is used for matching each endpoint structure with it's list of received buffers.
 *
 * \see MCC_ENDPOINT
 * \see MCC_RECEIVE_LIST
 * \see MCC_BOOKEEPING_STRUCT
 */
struct mcc_endpoint_map_item {
    /*! \brief Endpoint tripplet */
    MCC_ENDPOINT      endpoint;

    /*! \brief List of received buffers */
    MCC_RECEIVE_LIST  list;
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_endpoint_map_item MCC_ENDPOINT_MAP_ITEM;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief MCC info structure.
 *
 * This is used for additional information about the MCC implementation.
 *
 * \see MCC_BOOKEEPING_STRUCT
 */
struct mcc_info_struct {
    /*! \brief <major>.<minor> - minor is changed whenever patched, major indicates compatibility */
    char version_string[sizeof(MCC_VERSION_STRING)];
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_info_struct MCC_INFO_STRUCT;

#if defined(__IAR_SYSTEMS_ICC__)
__packed
#endif
/*!
 * \brief Share Memory data - Bookkeeping data and buffers.
 *
 * This is used for "bookkeeping data" such as endpoint and signal queue head 
 * and tail pointers and fixed size data buffers. The whole mcc_bookeeping_struct
 * as well as each individual structure members has to be defined and stored in the
 * memory as packed structure. This way, the same structure member offsets will be ensured
 * on all cores/OSes/compilers. Compiler-specific pragmas for data packing have to be applied.
 *
 * \see MCC_RECEIVE_LIST
 * \see MCC_SIGNAL
 * \see MCC_ENDPOINT_MAP_ITEM
 * \see MCC_RECEIVE_BUFFER
 */
struct mcc_bookeeping_struct {
    /*! \brief String that indicates if this structure has been already initialized */
    char init_string[sizeof(MCC_INIT_STRING)];

    /*! \brief String that indicates the MCC library version */
    char version_string[sizeof(MCC_VERSION_STRING)];

    /*! \brief List of free buffers */
    MCC_RECEIVE_LIST free_list;

    /*! \brief Each core has it's own queue of received signals */
    MCC_SIGNAL signals_received[MCC_NUM_CORES][MCC_MAX_OUTSTANDING_SIGNALS];

    /*! \brief Signal queue head for each core */
    unsigned int signal_queue_head[MCC_NUM_CORES];

    /*! \brief Signal queue tail for each core */
    unsigned int signal_queue_tail[MCC_NUM_CORES];

    /*! \brief Endpoint map */
    MCC_ENDPOINT_MAP_ITEM endpoint_table[MCC_ATTR_MAX_RECEIVE_ENDPOINTS];

    /*! \brief Receive buffers, the number is defined in mcc_config.h (MCC_ATTR_NUM_RECEIVE_BUFFERS) */
    MCC_RECEIVE_BUFFER r_buffers[MCC_ATTR_NUM_RECEIVE_BUFFERS];
#if defined(__IAR_SYSTEMS_ICC__)
};
#else
}__attribute__((packed));
#endif
typedef struct mcc_bookeeping_struct MCC_BOOKEEPING_STRUCT;

extern MCC_BOOKEEPING_STRUCT * bookeeping_data;

/*
 * Common Macros
 */
#define MCC_RESERVED_PORT_NUMBER        (0)
#define MCC_MAX_RECEIVE_ENDPOINTS_COUNT (255)

/*
 * Errors
 */
#define MCC_SUCCESS         (0) /* function returned successfully */
#define MCC_ERR_TIMEOUT     (1) /* blocking function timed out before completing */
#define MCC_ERR_INVAL       (2) /* invalid input parameter */
#define MCC_ERR_NOMEM       (3) /* out of shared memory for message transmission */
#define MCC_ERR_ENDPOINT    (4) /* invalid endpoint / endpoint doesn't exist */
#define MCC_ERR_SEMAPHORE   (5) /* semaphore handling error */
#define MCC_ERR_DEV         (6) /* Device Open Error */
#define MCC_ERR_INT         (7) /* Interrupt Error */
#define MCC_ERR_SQ_FULL     (8) /* Signal queue is full */
#define MCC_ERR_SQ_EMPTY    (9) /* Signal queue is empty */
#define MCC_ERR_VERSION     (10) /* Incorrect MCC version used - compatibility issue */
#define MCC_ERR_OSSYNC      (11) /* OS-dependent synchronization module issue */

/*
 * OS Selection
 */
#define MCC_LINUX         (1) /* Linux OS used */
#define MCC_MQX           (2) /* MQX RTOS used */

MCC_RECEIVE_LIST * mcc_get_endpoint_list(MCC_ENDPOINT endpoint);
MCC_RECEIVE_BUFFER * mcc_dequeue_buffer(MCC_RECEIVE_LIST *list);
void mcc_queue_buffer(MCC_RECEIVE_LIST *list, MCC_RECEIVE_BUFFER * r_buffer);
int mcc_remove_endpoint(MCC_ENDPOINT endpoint);
int mcc_register_endpoint(MCC_ENDPOINT endpoint);
int mcc_queue_signal(MCC_CORE core, MCC_SIGNAL signal);
int mcc_dequeue_signal(MCC_CORE core, MCC_SIGNAL *signal);

#define MCC_SIGNAL_QUEUE_FULL(core)  (((bookeeping_data->signal_queue_tail[core] + 1) % MCC_MAX_OUTSTANDING_SIGNALS) == bookeeping_data->signal_queue_head[core])
#define MCC_SIGNAL_QUEUE_EMPTY(core) (bookeeping_data->signal_queue_head[core] == bookeeping_data->signal_queue_tail[core])
#define MCC_ENDPOINTS_EQUAL(e1, e2)  ((e1.core == e2.core) && (e1.node == e2.node) && (e1.port == e2.port))

#if (MCC_ATTR_MAX_RECEIVE_ENDPOINTS > MCC_MAX_RECEIVE_ENDPOINTS_COUNT)
#error User-defined maximum number of endpoints can not exceed the value of MCC_MAX_RECEIVE_ENDPOINTS_COUNT
#endif

#endif /* __MCC_COMMON__ */

