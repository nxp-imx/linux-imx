/*
 * This file contains MCC library API functions implementation
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

#include "mcc_config.h"
#if (MCC_OS_USED == MCC_MQX)
#include <string.h>
#include "mcc_config.h"
#include "mcc_common.h"
#include "mcc_api.h"
#include "mcc_mqx.h"
#elif (MCC_OS_USED == MCC_LINUX)
#include <linux/mcc_api.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>
#include <linux/imx_sema4.h>
#endif

const char * const init_string    = MCC_INIT_STRING;
const char * const version_string = MCC_VERSION_STRING;

static int mcc_recv_common_part(MCC_ENDPOINT *endpoint, unsigned int timeout_ms, MCC_RECEIVE_LIST **list);
static int mcc_get_buffer_internal(void **buffer, MCC_MEM_SIZE *buf_size, unsigned int timeout_ms);
static int mcc_free_buffer_internal(void *buffer);
/*!
 * \brief This function initializes the Multi Core Communication subsystem for a given node.
 *
 * This function should only be called once per node (once in MQX, once per a process in Linux).
 * It tries to initialize the bookkeeping structure when the init_string member of this structure
 * is not equal to MCC_INIT_STRING, i.e. when no other core had performed the initialization yet.
 * Note, that this way of bookkeeping data re-initialization protection is not powerful enough and
 * the user application should not rely on this method. Instead, the application should be designed 
 * to unambiguously assign the core that will perform the MCC initialization. 
 * Clear the shared memory before the first core is attempting to initialize the MCC 
 * (in some cases MCC_INIT_STRING remains in the shared memory after the application reset and could
 * cause that the bookkeeping data structure is not initialized correctly). 
 *
 * \param[in] node Node number that will be used in endpoints created by this process.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_INT (interrupt registration error)
 * \return MCC_ERR_VERSION (incorrect MCC version used - compatibility issue)
 * \return MCC_ERR_OSSYNC (OS synchronization module(s) initialization failed)
 *
 * \see mcc_destroy
 * \see MCC_BOOKEEPING_STRUCT
 */
int mcc_initialize(MCC_NODE node)
{
    int i,j = 0;
    int return_value = MCC_SUCCESS;
    MCC_SIGNAL tmp_signals_received = {(MCC_SIGNAL_TYPE)0, {(MCC_CORE)0, (MCC_NODE)0, (MCC_PORT)0}};

    /* Initialize synchronization module for shared data protection */
    return_value = mcc_init_semaphore(MCC_SHMEM_SEMAPHORE_NUMBER);
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Initialize all necessary OS synchronization module(s) 
       (for unblocking tasks waiting for new received data and for unblocking tasks waiting for a free buffer) */
    return_value = mcc_init_os_sync();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Register CPU-to-CPU interrupt for inter-core signaling */
    //mcc_register_cpu_to_cpu_isr(MCC_CORE0_CPU_TO_CPU_VECTOR);
    return_value = mcc_register_cpu_to_cpu_isr();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Initialize the bookeeping structure */
    bookeeping_data = (MCC_BOOKEEPING_STRUCT *)mcc_get_bookeeping_data();
    MCC_DCACHE_INVALIDATE_MLINES(bookeeping_data, sizeof(MCC_BOOKEEPING_STRUCT));
    if(strcmp(bookeeping_data->init_string, init_string) != 0) {
        /* MCC not initialized yet, do it now */
        /* Zero it all - no guarantee Linux or uboot didnt touch it before it was reserved */
        memset((void*) bookeeping_data, 0, sizeof(struct mcc_bookeeping_struct));

        /* Set init_string in case it has not been set yet by another core */
        mcc_memcpy((void*)init_string, bookeeping_data->init_string, (unsigned int)sizeof(bookeeping_data->init_string));

        /* Set version_string */
        mcc_memcpy((void*)version_string, bookeeping_data->version_string, (unsigned int)sizeof(bookeeping_data->version_string));

        /* Initialize the free list */
        bookeeping_data->free_list.head = (MCC_RECEIVE_BUFFER*)MCC_MEM_VIRT_TO_PHYS(&bookeeping_data->r_buffers[0]);
        bookeeping_data->free_list.tail = (MCC_RECEIVE_BUFFER*)MCC_MEM_VIRT_TO_PHYS(&bookeeping_data->r_buffers[MCC_ATTR_NUM_RECEIVE_BUFFERS-1]);

        /* Initialize receive buffers */
        for(i=0; i<MCC_ATTR_NUM_RECEIVE_BUFFERS-1; i++) {
            bookeeping_data->r_buffers[i].next = (MCC_RECEIVE_BUFFER*)MCC_MEM_VIRT_TO_PHYS(&bookeeping_data->r_buffers[i+1]);
        }
        bookeeping_data->r_buffers[MCC_ATTR_NUM_RECEIVE_BUFFERS-1].next = null;

        /* Initialize signal queues */
        for(i=0; i<MCC_NUM_CORES; i++) {
            for(j=0; j<MCC_MAX_OUTSTANDING_SIGNALS; j++) {
                bookeeping_data->signals_received[i][j] = tmp_signals_received;
            }
            bookeeping_data->signal_queue_head[i] = 0;
            bookeeping_data->signal_queue_tail[i] = 0;
        }

        /* Mark all endpoint ports as free */
        for(i=0; i<MCC_ATTR_MAX_RECEIVE_ENDPOINTS; i++) {
            bookeeping_data->endpoint_table[i].endpoint.port = MCC_RESERVED_PORT_NUMBER;
        }
    }
    else {
        /* MCC already initialized - check the major number of the version string to ensure compatibility */
        if(strncmp(bookeeping_data->version_string, version_string, 4) != 0) {
            return_value = MCC_ERR_VERSION;
        }
    }
    
    MCC_DCACHE_FLUSH_MLINES(bookeeping_data, sizeof(MCC_BOOKEEPING_STRUCT));
    return return_value;
}

/*!
 * \brief This function de-initializes the Multi Core Communication subsystem for a given node.
 *
 * The function frees all resources of the node. Deletes all endpoints and frees any buffers that may have been queued there.
 *
 * \param[in] node Node number to be deinitialized.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_OSSYNC (OS synchronization module(s) deinitialization failed)
 *
 * \see mcc_initialize
 */
int mcc_destroy(MCC_NODE node)
{
    int i = 0, return_value;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* All endpoints of the particular node have to be removed from the endpoint table */
    MCC_DCACHE_INVALIDATE_MLINES(&bookeeping_data->endpoint_table[0], MCC_ATTR_MAX_RECEIVE_ENDPOINTS * sizeof(MCC_ENDPOINT_MAP_ITEM));
    for(i = 0; i < MCC_ATTR_MAX_RECEIVE_ENDPOINTS; i++) {
        if (bookeeping_data->endpoint_table[i].endpoint.node == node) {
            /* Remove the endpoint from the table */
            mcc_remove_endpoint(bookeeping_data->endpoint_table[i].endpoint);
        }
    }

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Deinitialize synchronization module */
    mcc_deinit_semaphore(MCC_SHMEM_SEMAPHORE_NUMBER);

    /* De-initialize OS synchronization module(s) 
       (for unblocking tasks waiting for new data and for unblocking tasks waiting for a free buffer */
    return_value = mcc_deinit_os_sync();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}

/*!
 * \brief This function creates an endpoint.
 *
 * The function creates an endpoint on the local node with the specified port number.
 * The core and node provided in the endpoint must match the caller's core and
 * node, and the port argument must match the endpoint port.
 *
 * \param[out] endpoint Pointer to the endpoint triplet to be created.
 * \param[in] port Port number.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_NOMEM (maximum number of endpoints exceeded)
 * \return MCC_ERR_ENDPOINT (invalid value for port or endpoint already registered)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_destroy_endpoint
 * \see MCC_ENDPOINT
 */
int mcc_create_endpoint(MCC_ENDPOINT *endpoint, MCC_PORT port)
{
    int return_value = MCC_SUCCESS;

    /* Fill the endpoint structure */
    endpoint->core = (MCC_CORE)MCC_CORE_NUMBER;
    endpoint->node = (MCC_NODE)MCC_NODE_NUMBER;
    endpoint->port = (MCC_PORT)port;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Add new endpoint data into the book-keeping structure */
    return_value = mcc_register_endpoint(*endpoint);
    if(return_value != MCC_SUCCESS) {
        mcc_release_semaphore();
        return return_value;
    }

    /* Semaphore-protected section end */
    return_value =  mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}

/*!
 * \brief This function destroys an endpoint.
 *
 * The function destroys an endpoint on the local node and frees any buffers that may be queued.
 *
 * \param[in] endpoint Pointer to the endpoint triplet to be deleted.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint doesn't exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_create_endpoint
 * \see MCC_ENDPOINT
 */
int mcc_destroy_endpoint(MCC_ENDPOINT *endpoint)
{
    int return_value = MCC_SUCCESS;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Remove the endpoint data from the book-keeping structure */
    return_value = mcc_remove_endpoint(*endpoint);
    if(return_value != MCC_SUCCESS) {
        mcc_release_semaphore();
        return return_value;
    }

    /* Clear OS synchronization module parts associated with the endpoint to be destroyed */
    mcc_clear_os_sync_for_ep(endpoint);
    
    /* Semaphore-protected section end */
    return_value =  mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}

/*!
 * \brief This function sends a message to an endpoint.
 *
 * The message is copied into the MCC buffer and the destination core is signaled.
 *
 * \param[in] src_endpoint Pointer to the local endpoint identifying the source endpoint.
 * \param[in] dest_endpoint Pointer to the destination endpoint to send the message to.
 * \param[in] msg Pointer to the message to be sent.
 * \param[in] msg_size Size of the message to be sent in bytes.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_INVAL (the msg_size exceeds the size of a data buffer)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a buffer became available)
 * \return MCC_ERR_NOMEM (no free buffer available and timeout_ms set to 0)
 * \return MCC_ERR_SQ_FULL (signal queue is full)
 *
 * \see mcc_recv
 * \see mcc_recv_nocopy
 * \see MCC_ENDPOINT
 */
int mcc_send(MCC_ENDPOINT *src_endpoint, MCC_ENDPOINT *dest_endpoint, void *msg, MCC_MEM_SIZE msg_size, unsigned int timeout_ms)
{
    int return_value;
    MCC_RECEIVE_LIST *list;
    MCC_RECEIVE_BUFFER * buf;
    MCC_SIGNAL affiliated_signal;
    MCC_MEM_SIZE buffer_size;

    /* Reuse the mcc_get_buffer_internal() function to get the MCC buffer pointer. */
    return_value = mcc_get_buffer_internal((void**)&buf, &buffer_size, timeout_ms);
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Check if the size of the message to be sent does not exceed the size of the mcc buffer */
    if(msg_size > buffer_size) {
        while(MCC_SUCCESS != mcc_free_buffer_internal(buf)) {};
        return MCC_ERR_INVAL;
    }

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* As the mcc_get_buffer_internal() returns the pointer to the data field, it
       is necessary to adjust the pointer to point at the MCC buffer structure beginning. */
    buf = (MCC_RECEIVE_BUFFER *)((unsigned int)buf - (unsigned int)(&(((MCC_RECEIVE_BUFFER*)0)->data)));

    /* Copy the message into the MCC receive buffer */
    MCC_DCACHE_INVALIDATE_MLINES((void*)buf, sizeof(MCC_RECEIVE_BUFFER));
    mcc_memcpy(msg, (void*)buf->data, (unsigned int)msg_size);
    mcc_memcpy((void*)src_endpoint, (void*)&buf->source, sizeof(MCC_ENDPOINT));
    buf->data_len = msg_size;
    MCC_DCACHE_FLUSH_MLINES((void*)buf, sizeof(MCC_RECEIVE_BUFFER));

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*dest_endpoint);

    if(list == null) {
        /* The endpoint does not exists (has not been registered so far),
         free the buffer and return immediately - error */
        /* Enqueue the buffer back into the free list */
        MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
        mcc_queue_buffer(&bookeeping_data->free_list, buf);
      
        mcc_release_semaphore();
        return MCC_ERR_ENDPOINT;
    }

    /* Write the signal type into the signal queue of the particular core */
    affiliated_signal.type = BUFFER_QUEUED;
    affiliated_signal.destination = *dest_endpoint;
    return_value = mcc_queue_signal(dest_endpoint->core, affiliated_signal);
    if(return_value != MCC_SUCCESS) {
        /* Signal queue is full, free the buffer and return immediately - error */
        MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
        mcc_queue_buffer(&bookeeping_data->free_list, buf);

        mcc_release_semaphore();
        return return_value;
    }

    /* Enqueue the buffer into the endpoint buffer list */
    mcc_queue_buffer(list, buf);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();

    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Signal the other core by generating the CPU-to-CPU interrupt */
    return_value = mcc_generate_cpu_to_cpu_interrupt();

    return return_value;
}

/*!
 * \private
 *  
 * \brief This function dequeues a buffer from the free list.
 *
 * This is an internal implementation of the mcc_get_buffer() function. It is called
 * either from the mcc_send() or from mcc_get_buffer() when the non-copy-send
 * mechanism is enabled by the MCC_SEND_RECV_NOCOPY_API_ENABLED macro in mcc_config.h. 
 *
 * \param[out] buffer Pointer to the MCC buffer dequeued from the free list.
 * \param[out] buf_size Pointer to an MCC_MEM_SIZE that is used for passing the size of the dequeued MCC buffer to the application.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a buffer became available)
 * \return MCC_ERR_NOMEM (no free buffer available and timeout_ms set to 0)
 *
 * \see mcc_send_nocopy
 * \see mcc_send
 * \see mcc_get_buffer 
 */
static int mcc_get_buffer_internal(void **buffer, MCC_MEM_SIZE *buf_size, unsigned int timeout_ms)
{
    int return_value;
    MCC_RECEIVE_BUFFER * buf = null;

    *buffer = null;
    
    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Dequeue the buffer from the free list */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
    buf = mcc_dequeue_buffer(&bookeeping_data->free_list);

    /* Semaphore-protected section end */
    mcc_release_semaphore();

    if(buf == null) {
        /* Non-blocking call */
        if(timeout_ms == 0) {
            return MCC_ERR_NOMEM;
        } else {
            /* Wait for the buffer freed event */
            return_value = mcc_wait_for_buffer_freed((MCC_RECEIVE_BUFFER **)&buf, timeout_ms);
            if(MCC_SUCCESS != return_value) {
                return return_value;
            }
        }
    }

    /* Return the MCC buffer size and the pointer to the dequeued MCC buffer */
    *buf_size = (MCC_MEM_SIZE)sizeof(bookeeping_data->r_buffers[0].data);
    *buffer = (void*)buf->data;
    return MCC_SUCCESS;
}

#if MCC_SEND_RECV_NOCOPY_API_ENABLED
/*!
 * \brief This function dequeues a buffer from the free list.
 *
 * The application has take the responsibility for MCC buffer de-allocation and 
 * filling the data to be sent into the pre-allocated MCC buffer.
 *
 * \param[out] buffer Pointer to the MCC buffer dequeued from the free list.
 * \param[out] buf_size Pointer to an MCC_MEM_SIZE that is used for passing the size of the dequeued MCC buffer to the application.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a buffer became available)
 * \return MCC_ERR_NOMEM (no free buffer available and timeout_ms set to 0)
 *
 * \see mcc_send_nocopy
 */
int mcc_get_buffer(void **buffer, MCC_MEM_SIZE *buf_size, unsigned int timeout_ms)
{
    return mcc_get_buffer_internal(buffer, buf_size, timeout_ms);
}

/*!
 * \brief This function sends a message to an endpoint. The data is NOT copied 
 * from the user-app. buffer but the pointer to already filled message buffer is
 * provided.  
 *
 * The application has to take the responsibility for:
 *  1. MCC buffer de-allocation
 *  2. filling the data to be sent into the pre-allocated MCC buffer
 *  3. not exceeding the buffer size when filling the data (MCC_ATTR_BUFFER_SIZE_IN_BYTES) 
 * 
 * Once the data cache is used on the target platform it is good to have MCC buffers 
 * in shared RAM aligned to the cache line size in order not to corrupt entities placed 
 * just before and just after the MCC buffer when flushing the MCC buffer content into
 * the shared RAM. It is also the application responsibility to flush the data in
 * that case. If the alignment condition is not fulfilled the application
 * has to take care about the data cache coherency. 
 * The following scenarios can happen:
 * A. Data cache is OFF:
 *    - No cache operation needs to be done, the application just 
 *     1. calls the mcc_get_buffer() function, 
 *     2. fills data into the provided MCC buffer,
 *     3. and finally issues the mcc_send_nocopy() function.
 * B. Data cache is ON, shared RAM MCC buffers ALIGNED to the cache line size:
 *    - The application has to perform following steps: 
 *     1. call the mcc_get_buffer() to get the pointer to a free message buffer 
 *     2. copy data to be sent into the message buffer
 *     3. flush all cache lines occupied by the message buffer new data
 *       (maximum of MCC_ATTR_BUFFER_SIZE_IN_BYTES bytes).
 *     4. call the mcc_send_nocopy() with the correct buffer pointer and the message size passed 
 * C. Data cache is ON, shared RAM MCC buffers NOT ALIGNED:
 *    - The application has to perform following steps: 
 *     1. call the mcc_get_buffer() to get the pointer to a free message buffer 
 *     2. grab the hw semaphore by calling the mcc_get_semaphore() low level MCC function.
 *     3. invalidate all cache lines occupied by data to be filled into the free message buffer.
 *       (maximum of MCC_ATTR_BUFFER_SIZE_IN_BYTES bytes).
 *     4. copy data to be sent into the message buffer.
 *     5. flush all cache lines occupied by the message buffer new data 
 *       (maximum of MCC_ATTR_BUFFER_SIZE_IN_BYTES bytes).
 *     6. release the hw semaphore by calling the mcc_release_semaphore() low level MCC function.
 *     7. call the mcc_send_nocopy() with the correct buffer pointer and the message size passed.
 * 
 * After the mcc_send_nocopy() function is issued the message buffer is no more owned 
 * by the sending task and must not be touched anymore unless the mcc_send_nocopy() 
 * function fails and returns an error. In that case the application should try 
 * to re-issue the mcc_send_nocopy() again and if it is still not possible to send 
 * the message and the application wants to give it up from whatever reasons 
 * (for instance the MCC_ERR_ENDPOINT error is returned meaning the endpoint 
 * has not been created yet) the mcc_free_buffer() function could be called, 
 * passing the pointer to the buffer to be freed as a parameter.     
 *
 * \param[in] src_endpoint Pointer to the local endpoint identifying the source endpoint.
 * \param[in] dest_endpoint Pointer to the destination endpoint to send the message to.
 * \param[in] buffer_p Pointer to the MCC buffer of the shared memory where the 
 *            data to be sent is stored.
 * \param[in] msg_size Size of the message to be sent in bytes.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_INVAL (the msg_size exceeds the size of a data buffer)
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_SQ_FULL (signal queue is full)
 *
 * \see mcc_send
 * \see mcc_get_buffer
 * \see MCC_ENDPOINT
 */
int mcc_send_nocopy(MCC_ENDPOINT *src_endpoint, MCC_ENDPOINT *dest_endpoint, void *buffer_p, MCC_MEM_SIZE msg_size)
{
    int return_value;
    MCC_RECEIVE_BUFFER * buf;
    MCC_RECEIVE_LIST *list;
    MCC_SIGNAL affiliated_signal;

    /* Check if the size of the message to be sent does not exceed the size of the mcc buffer */
    if(msg_size > sizeof(bookeeping_data->r_buffers[0].data)) {
        return MCC_ERR_INVAL;
    }

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*dest_endpoint);

    if(list == null) {
        /* The endpoint does not exists (has not been registered so far) */
        mcc_release_semaphore();
        return MCC_ERR_ENDPOINT;
    }

    /* Store the message size and the source endpoint in the MCC_RECEIVE_BUFFER structure */
    buf = (MCC_RECEIVE_BUFFER *)((unsigned int)buffer_p - (unsigned int)(&(((MCC_RECEIVE_BUFFER*)0)->data)));
    MCC_DCACHE_INVALIDATE_MLINES((void*)&buf->source, sizeof(MCC_ENDPOINT) + sizeof(MCC_MEM_SIZE));
    ((MCC_RECEIVE_BUFFER*)buf)->data_len = msg_size;
    mcc_memcpy((void*)src_endpoint, (void*)&buf->source, sizeof(MCC_ENDPOINT));
    MCC_DCACHE_FLUSH_MLINES((void*)(void*)&buf->source, sizeof(MCC_ENDPOINT) + sizeof(MCC_MEM_SIZE));


    /* Write the signal type into the signal queue of the particular core */
    affiliated_signal.type = BUFFER_QUEUED;
    affiliated_signal.destination = *dest_endpoint;
    return_value = mcc_queue_signal(dest_endpoint->core, affiliated_signal);
    if(return_value != MCC_SUCCESS) {
        /* Signal queue is full - error */
        mcc_release_semaphore();
        return return_value;
    }

    /* Enqueue the buffer into the endpoint buffer list */
    mcc_queue_buffer(list, (MCC_RECEIVE_BUFFER*)buf);

    /* Semaphore-protected section end */
    mcc_release_semaphore();

    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Signal the other core by generating the CPU-to-CPU interrupt */
    return_value = mcc_generate_cpu_to_cpu_interrupt();

    return return_value;
}
#endif /* MCC_SEND_RECV_NOCOPY_API_ENABLED */

/*!
 * \brief This function receives a message from the specified endpoint if one is available.
 *        The data is copied from the receive buffer into the user supplied buffer.
 *
 * This is the "receive with copy" version of the MCC receive function. This version is simple
 * to use but it requires copying data from shared memory into the user space buffer.
 * The user has no obligation or burden to manage the shared memory buffers.
 *
 * \param[out] src_endpoint Pointer to the MCC_ENDPOINT structure to be filled by the endpoint identifying the message sender.
 * \param[in] dest_endpoint Pointer to the local receiving endpoint to receive from.
 * \param[in] buffer Pointer to the user-app. buffer where data will be copied into.
 * \param[in] buffer_size The maximum number of bytes to copy.
 * \param[out] recv_size Pointer to an MCC_MEM_SIZE that will contain the number of bytes actually copied into the buffer.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a new message came)
 *
 * \see mcc_send
 * \see mcc_recv_nocopy
 * \see MCC_ENDPOINT
 */
int mcc_recv(MCC_ENDPOINT *src_endpoint, MCC_ENDPOINT *dest_endpoint, void *buffer, MCC_MEM_SIZE buffer_size, MCC_MEM_SIZE *recv_size, unsigned int timeout_ms)
{
    MCC_RECEIVE_LIST *list = null;
    MCC_RECEIVE_BUFFER * buf;
    MCC_SIGNAL affiliated_signal;
    MCC_ENDPOINT tmp_destination = {(MCC_CORE)0, (MCC_NODE)0, (MCC_PORT)0};
    int return_value, i = 0;

    return_value = mcc_recv_common_part(dest_endpoint, timeout_ms, (MCC_RECEIVE_LIST**)&list);
    if(return_value != MCC_SUCCESS)
        return return_value;
    
    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    MCC_DCACHE_INVALIDATE_MLINES((void*)list, sizeof(MCC_RECEIVE_LIST*));

    if(list->head == (MCC_RECEIVE_BUFFER*)0) {
        /* Buffer not dequeued before the timeout */
        mcc_release_semaphore();
        return MCC_ERR_TIMEOUT;
    }

    /* Copy the message from the MCC receive buffer into the user-app. buffer */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->source, sizeof(MCC_ENDPOINT) + sizeof(MCC_MEM_SIZE));
    mcc_memcpy((void*)&list->head->source, (void*)src_endpoint, sizeof(MCC_ENDPOINT));
    if (list->head->data_len > buffer_size) {
        list->head->data_len = buffer_size;
    }
    *recv_size = (MCC_MEM_SIZE)(list->head->data_len);
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->data, list->head->data_len);
    mcc_memcpy((void*)list->head->data, buffer, list->head->data_len);

    /* Dequeue the buffer from the endpoint list */
    list->head = (MCC_RECEIVE_BUFFER*)MCC_MEM_VIRT_TO_PHYS(list->head);
    buf = mcc_dequeue_buffer(list);

    /* Enqueue the buffer into the free list */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
    mcc_queue_buffer(&bookeeping_data->free_list, buf);

    /* Notify all cores (except of itself) via CPU-to-CPU interrupt that a buffer has been freed */
    affiliated_signal.type = BUFFER_FREED;
    affiliated_signal.destination = tmp_destination;
    for (i=0; i<MCC_NUM_CORES; i++) {
        if(i != MCC_CORE_NUMBER) {
            mcc_queue_signal(i, affiliated_signal);
        }
    }

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();

    mcc_generate_cpu_to_cpu_interrupt();

    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}

#if MCC_SEND_RECV_NOCOPY_API_ENABLED
/*!
 * \brief This function receives a message from the specified endpoint if one is available. The data is NOT copied into the user-app. buffer.
 *
 * This is the "zero-copy receive" version of the MCC receive function. No data is copied. 
 * Only the pointer to the data is returned. This version is fast, but it requires the user to manage
 * buffer allocation. Specifically, the user must decide when a buffer is no longer in use and
 * make the appropriate API call to free it, see mcc_free_buffer.
 *
 * \param[out] src_endpoint Pointer to the MCC_ENDPOINT structure to be filled by the endpoint identifying the message sender.
 * \param[in] dest_endpoint Pointer to the local receiving endpoint to receive from.
 * \param[out] buffer_p Pointer to the MCC buffer of the shared memory where the received data is stored.
 * \param[out] recv_size Pointer to an MCC_MEM_SIZE that will contain the number of valid bytes in the buffer.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a new message came)
 *
 * \see mcc_send
 * \see mcc_recv
 * \see MCC_ENDPOINT
 */
int mcc_recv_nocopy(MCC_ENDPOINT *src_endpoint, MCC_ENDPOINT *dest_endpoint, void **buffer_p, MCC_MEM_SIZE *recv_size, unsigned int timeout_ms)
{
    MCC_RECEIVE_LIST *list = null;
    int return_value;

    return_value = mcc_recv_common_part(dest_endpoint, timeout_ms, (MCC_RECEIVE_LIST**)&list);
    if(return_value != MCC_SUCCESS)
        return return_value;
    
    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    MCC_DCACHE_INVALIDATE_MLINES((void*)list, sizeof(MCC_RECEIVE_LIST*));

    if(list->head == (MCC_RECEIVE_BUFFER*)0) {
        /* Buffer not dequeued before the timeout */
        mcc_release_semaphore();
        return MCC_ERR_TIMEOUT;
    }

    /* Get the message pointer from the head of the receive buffer list */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->data, list->head->data_len);
    *buffer_p = (void*)&list->head->data;
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->source, sizeof(MCC_ENDPOINT) + sizeof(MCC_MEM_SIZE));
    mcc_memcpy((void*)&list->head->source, (void*)src_endpoint, sizeof(MCC_ENDPOINT));
    *recv_size = (MCC_MEM_SIZE)(list->head->data_len);

    /* Dequeue the buffer from the endpoint list */
    mcc_dequeue_buffer(list);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}
#endif /* MCC_SEND_RECV_NOCOPY_API_ENABLED */

/*!
 * \private
 *
 * \brief This function is common part for mcc_recv() and mcc_recv_nocopy() function.
 *
 * It tries to get the list of buffers kept by the particular endpoint. If the list is empty
 * it waits for a new message until the timeout expires. 
 *
 * \param[in] endpoint Pointer to the receiving endpoint to receive from.
 * \param[in] timeout_ms Timeout, in milliseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 * \param[out] list Pointer to the list of buffers kept by the particular endpoint.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a new message came)
 *
 * \see mcc_recv
 * \see mcc_recv_nocopy
 * \see MCC_ENDPOINT
 */
static int mcc_recv_common_part(MCC_ENDPOINT *endpoint, unsigned int timeout_ms, MCC_RECEIVE_LIST **list)
{
    MCC_RECEIVE_LIST *tmp_list;
    int return_value;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Get list of buffers kept by the particular endpoint */
    tmp_list = mcc_get_endpoint_list(*endpoint);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* The endpoint is not valid */
    if(tmp_list == null) {
        return MCC_ERR_ENDPOINT;
    }

    if(tmp_list->head == (MCC_RECEIVE_BUFFER*)0) {
        /* Non-blocking call */
        if(timeout_ms == 0) {
            return MCC_ERR_TIMEOUT;
        }
        /* Blocking call */
        else {
            /* Wait for the buffer queued event */
            return_value = mcc_wait_for_buffer_queued(endpoint, timeout_ms);
            if(MCC_SUCCESS != return_value) {
                return return_value;
            }
        }
    }
    else {
        tmp_list->head = (MCC_RECEIVE_BUFFER*)MCC_MEM_PHYS_TO_VIRT(tmp_list->head);
    }
    /* Clear event bit specified for the particular endpoint */
    mcc_clear_os_sync_for_ep(endpoint);

    *list = (MCC_RECEIVE_LIST*)tmp_list;
    return MCC_SUCCESS;
}

/*!
 * \brief This function returns the number of buffers currently queued at the endpoint.
 *
 * The function checks if messages are available on a receive endpoint. While the call only checks the
 * availability of messages, it does not dequeue them.
 *
 * \param[in] endpoint Pointer to the endpoint structure.
 * \param[out] num_msgs Pointer to an unsigned int that will contain the number of buffers queued.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_recv
 * \see mcc_recv_nocopy
 * \see MCC_ENDPOINT
 */
int mcc_msgs_available(MCC_ENDPOINT *endpoint, unsigned int *num_msgs)
{
    unsigned int count = 0;
    MCC_RECEIVE_LIST *list;
    MCC_RECEIVE_BUFFER * buf;
    int return_value;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*endpoint);
    if(list == null) {
        /* The endpoint does not exists (has not been registered so far), return immediately - error */
        mcc_release_semaphore();
        return MCC_ERR_ENDPOINT;
    }

    buf = list->head;
    while(buf != (MCC_RECEIVE_BUFFER*)0) {
        count++;
        MCC_DCACHE_INVALIDATE_MLINES((void*)&buf->next, sizeof(MCC_RECEIVE_BUFFER*));
        buf = (MCC_RECEIVE_BUFFER*)buf->next;
    }
    *num_msgs = count;

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}

/*!
 * \private
 *  
 * \brief This function frees a buffer previously returned by mcc_recv_nocopy().
 *
 * Once the zero-copy mechanism of receiving data is used, this function
 * has to be called to free a buffer and to make it available for the next data
 * transfer.
 *
 * \param[in] buffer Pointer to the buffer to be freed.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_recv_nocopy
 */
static int mcc_free_buffer_internal(void *buffer)
{
    MCC_SIGNAL affiliated_signal;
    MCC_ENDPOINT tmp_destination = {(MCC_CORE)0, (MCC_NODE)0, (MCC_PORT)0};
    int return_value, i = 0;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Enqueue the buffer into the free list */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
    mcc_queue_buffer(&bookeeping_data->free_list, (MCC_RECEIVE_BUFFER *)((unsigned int)buffer - (unsigned int)(&(((MCC_RECEIVE_BUFFER*)0)->data))));

    /* Notify all cores (except of itself) via CPU-to-CPU interrupt that a buffer has been freed */
    affiliated_signal.type = BUFFER_FREED;
    affiliated_signal.destination = tmp_destination;
    for (i=0; i<MCC_NUM_CORES; i++) {
        if(i != MCC_CORE_NUMBER) {
            mcc_queue_signal(i, affiliated_signal);
        }
    }

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();

    mcc_generate_cpu_to_cpu_interrupt();

    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}
#if MCC_SEND_RECV_NOCOPY_API_ENABLED
/*!
 * \brief This function frees a buffer previously returned by mcc_recv_nocopy().
 *
 * Once the zero-copy mechanism of receiving data is used, this function
 * has to be called to free a buffer and to make it available for the next data
 * transfer.
 *
 * \param[in] buffer Pointer to the buffer to be freed.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_recv_nocopy
 */
int mcc_free_buffer(void *buffer)
{
    return mcc_free_buffer_internal(buffer);
}
#endif /* MCC_SEND_RECV_NOCOPY_API_ENABLED */

/*!
 * \brief This function returns information about the MCC sub system.
 *
 * The function returns implementation-specific information.
 *
 * \param[in] node Node number.
 * \param[out] info_data Pointer to the MCC_INFO_STRUCT structure to hold returned data.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see MCC_INFO_STRUCT
 */
int mcc_get_info(MCC_NODE node, MCC_INFO_STRUCT* info_data)
{
    int return_value;

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    mcc_memcpy(bookeeping_data->version_string, (void*)info_data->version_string, (unsigned int)sizeof(bookeeping_data->version_string));

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
}
