/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Freescale IMX Linux-specific MCC implementation.
 * MCC library API functions implementation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/string.h>

#include "mcc_config.h"
#if(MCC_OS_USED == MCC_LINUX)
#include <linux/slab.h>
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>
#elif (MCC_OS_USED == MCC_MQX)
#include "mcc_api.h"
#include "mcc_common.h"
#endif

#if (MCC_OS_USED == MCC_MQX)
#include "mcc_mqx.h"
extern LWEVENT_STRUCT lwevent_buffer_queued[MCC_MQX_LWEVENT_COMPONENTS_COUNT];
extern LWEVENT_STRUCT lwevent_buffer_freed;
#elif (MCC_OS_USED == MCC_LINUX)
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/imx_sema4.h>

extern wait_queue_head_t buffer_freed_wait_queue, buffer_queued_wait_queue;
extern unsigned int imx_mcc_buffer_queued, imx_mcc_buffer_freed;
extern unsigned long mcc_shm_offset;
#endif

const char * const init_string    = MCC_INIT_STRING;
const char * const version_string = MCC_VERSION_STRING;

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
 *
 * \see mcc_destroy
 * \see MCC_BOOKEEPING_STRUCT
 */
int mcc_initialize(MCC_NODE node)
{
    int i,j = 0;
    int return_value = MCC_SUCCESS;
    MCC_SIGNAL tmp_signals_received = {(MCC_SIGNAL_TYPE)0, {(MCC_CORE)0, (MCC_NODE)0, (MCC_PORT)0}};

#if (MCC_OS_USED == MCC_MQX)
    for(i=0; i<MCC_MQX_LWEVENT_COMPONENTS_COUNT; i++) {
        _lwevent_create(&lwevent_buffer_queued[i],0);
    }
    _lwevent_create(&lwevent_buffer_freed,0);
#endif

    /* Initialize synchronization module */
    return_value = mcc_init_semaphore(MCC_SHMEM_SEMAPHORE_NUMBER);
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Register CPU-to-CPU interrupt for inter-core signaling */
    return_value = mcc_register_cpu_to_cpu_isr();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Initialize the bookeeping structure */
#if (MCC_OS_USED == MCC_LINUX)
    bookeeping_data = (MCC_BOOKEEPING_STRUCT *)ioremap_nocache(MCC_BASE_ADDRESS, sizeof(struct mcc_bookeeping_struct));
    mcc_shm_offset = (unsigned long)bookeeping_data - (unsigned long)MCC_BASE_ADDRESS; 
#elif (MCC_OS_USED == MCC_MQX)
    bookeeping_data = (MCC_BOOKEEPING_STRUCT *)MCC_BASE_ADDRESS;
#endif
    MCC_DCACHE_INVALIDATE_MLINES(bookeeping_data, sizeof(MCC_BOOKEEPING_STRUCT));
    if(strcmp(bookeeping_data->init_string, init_string) != 0) {

        /* Zero it all - no guarantee Linux or uboot didnt touch it before it was reserved */
        _mem_zero((void*) bookeeping_data, (_mem_size) sizeof(struct mcc_bookeeping_struct));

        /* Set init_string in case it has not been set yet by another core */
        mcc_memcpy((void*)init_string, bookeeping_data->init_string, (unsigned int)sizeof(bookeeping_data->init_string));

        /* Set version_string */
        mcc_memcpy((void*)version_string, bookeeping_data->version_string, (unsigned int)sizeof(bookeeping_data->version_string));

        /* Initialize the free list */
#if (MCC_OS_USED == MCC_LINUX)
        bookeeping_data->free_list.head = (MCC_RECEIVE_BUFFER*)VIRT_TO_MQX(&bookeeping_data->r_buffers[0]);
        bookeeping_data->free_list.tail = (MCC_RECEIVE_BUFFER*)VIRT_TO_MQX(&bookeeping_data->r_buffers[MCC_ATTR_NUM_RECEIVE_BUFFERS-1]);
#elif (MCC_OS_USED == MCC_MQX)
        bookeeping_data->free_list.head = &bookeeping_data->r_buffers[0];
        bookeeping_data->free_list.tail = &bookeeping_data->r_buffers[MCC_ATTR_NUM_RECEIVE_BUFFERS-1];
#endif

        /* Initialize receive buffers */
        for(i=0; i<MCC_ATTR_NUM_RECEIVE_BUFFERS-1; i++) {
#if (MCC_OS_USED == MCC_LINUX)
            bookeeping_data->r_buffers[i].next = (MCC_RECEIVE_BUFFER*)VIRT_TO_MQX(&bookeeping_data->r_buffers[i+1]);
#elif (MCC_OS_USED == MCC_MQX)
            bookeeping_data->r_buffers[i].next = &bookeeping_data->r_buffers[i+1];
#endif
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
    MCC_DCACHE_FLUSH_MLINES(bookeeping_data, sizeof(MCC_BOOKEEPING_STRUCT));
    return return_value;
}
EXPORT_SYMBOL(mcc_initialize);

/*!
 * \brief This function de-initializes the Multi Core Communication subsystem for a given node.
 *
 * The function frees all resources of the node. Deletes all endpoints and frees any buffers that may have been queued there.
 *
 * \param[in] node Node number to be deinitialized.
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 *
 * \see mcc_initialize
 */
int mcc_destroy(MCC_NODE node)
{
    int i = 0, return_value;

#if (MCC_OS_USED == MCC_MQX)
    for(i=0; i<MCC_MQX_LWEVENT_COMPONENTS_COUNT; i++) {
        _lwevent_destroy(&lwevent_buffer_queued[i]);
    }
    _lwevent_destroy(&lwevent_buffer_freed);
#endif

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

    return return_value;
}
EXPORT_SYMBOL(mcc_destroy);

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
EXPORT_SYMBOL(mcc_create_endpoint);

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

    /* Add new endpoint data into the book-keeping structure */
    return_value = mcc_remove_endpoint(*endpoint);
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
 * \brief This function sends a message to an endpoint.
 *
 * The message is copied into the MCC buffer and the destination core is signaled.
 *
 * \param[in] endpoint Pointer to the receiving endpoint to send to.
 * \param[in] msg Pointer to the message to be sent.
 * \param[in] msg_size Size of the message to be sent in bytes.
 * \param[in] timeout_us Timeout, in microseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_INVAL (the msg_size exceeds the size of a data buffer)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a buffer became available)
 * \return MCC_ERR_NOMEM (no free buffer available and timeout_us set to 0)
 * \return MCC_ERR_SQ_FULL (signal queue is full)
 *
 * \see mcc_recv_copy
 * \see mcc_recv_nocopy
 * \see MCC_ENDPOINT
 */
int mcc_send(MCC_ENDPOINT *endpoint, void *msg, MCC_MEM_SIZE msg_size, unsigned int timeout_us)
{
    int return_value;
    MCC_RECEIVE_LIST *list;
    MCC_RECEIVE_BUFFER * buf;
    MCC_SIGNAL affiliated_signal;
#if (MCC_OS_USED == MCC_MQX)
    int end_time_set_flag = 0;
    unsigned int time_us_tmp;
    MQX_TICK_STRUCT tick_time;
#elif (MCC_OS_USED == MCC_LINUX)
    unsigned long timeout_j; /* jiffies */
#endif

    /* Check if the size of the message to be sent does not exceed the size of the mcc buffer */
    if(msg_size > sizeof(bookeeping_data->r_buffers[0].data)) {
        return MCC_ERR_INVAL;
    }

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Dequeue the buffer from the free list */
    MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
    buf = mcc_dequeue_buffer(&bookeeping_data->free_list);

    while(buf == null) {
        mcc_release_semaphore();

        /* Non-blocking call */
        if(timeout_us == 0) {
            return MCC_ERR_NOMEM;
        }
        /* Blocking calls: CPU-to-CPU ISR sets the event and thus resumes tasks waiting for a free MCC buffer.
         * As the interrupt request is send to all cores when a buffer is freed it could happen that several
         * tasks from different cores/nodes are waiting for a free buffer and all of them are notified that the buffer
         * has been freed. This function has to check (after the wake up) that a buffer is really available and has not been already
         * grabbed by another "competitor task" that has been faster. If so, it has to wait again for the next notification. */
        /* wait forever */
        else if(timeout_us == 0xFFFFFFFF) {
#if (MCC_OS_USED == MCC_MQX)
            _lwevent_wait_ticks(&lwevent_buffer_freed, 1, TRUE, 0);
            _lwevent_clear(&lwevent_buffer_freed, 1);
#elif (MCC_OS_USED == MCC_LINUX)
            timeout_j = usecs_to_jiffies(1000);
            wait_event_timeout(buffer_freed_wait_queue, imx_mcc_buffer_freed == 1, timeout_j);
#endif
        }
        /* timeout_us > 0 */
        else {
#if (MCC_OS_USED == MCC_MQX)
            if(!end_time_set_flag) {
                _time_get_ticks(&tick_time);
                _time_add_usec_to_ticks(&tick_time, timeout_us);
                end_time_set_flag = 1;
            }
            return_value = _lwevent_wait_until(&lwevent_buffer_freed, 1, TRUE, &tick_time);
            if(return_value == LWEVENT_WAIT_TIMEOUT) {
                /* Buffer not dequeued before the timeout */
                return MCC_ERR_TIMEOUT;
            }
            _lwevent_clear(&lwevent_buffer_freed, 1);
#elif (MCC_OS_USED == MCC_LINUX)
            timeout_j = usecs_to_jiffies(timeout_us);
            wait_event_timeout(buffer_freed_wait_queue, imx_mcc_buffer_freed == 1, timeout_j);
#endif
        }
        return_value = mcc_get_semaphore();
        MCC_DCACHE_INVALIDATE_MLINES((void*)&bookeeping_data->free_list, sizeof(MCC_RECEIVE_LIST*));
        buf = mcc_dequeue_buffer(&bookeeping_data->free_list);
    }

#if (MCC_OS_USED == MCC_LINUX)
    if (imx_mcc_buffer_freed)
        imx_mcc_buffer_freed = 0;
#endif

    /* Copy the message into the MCC receive buffer */
    MCC_DCACHE_INVALIDATE_MLINES((void*)buf, sizeof(MCC_RECEIVE_BUFFER));
    mcc_memcpy(msg, (void*)buf->data, (unsigned int)msg_size);
    buf->data_len = msg_size;
    MCC_DCACHE_FLUSH_MLINES((void*)buf, sizeof(MCC_RECEIVE_BUFFER));

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*endpoint);

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
    affiliated_signal.destination = *endpoint;
    return_value = mcc_queue_signal(endpoint->core, affiliated_signal);
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
EXPORT_SYMBOL(mcc_send);

/*!
 * \brief This function receives a message from the specified endpoint if one is available.
 *        The data is copied from the receive buffer into the user supplied buffer.
 *
 * This is the "receive with copy" version of the MCC receive function. This version is simple
 * to use but it requires copying data from shared memory into the user space buffer.
 * The user has no obligation or burden to manage the shared memory buffers.
 *
 * \param[in] endpoint Pointer to the receiving endpoint to receive from.
 * \param[in] buffer Pointer to the user-app. buffer where data will be copied into.
 * \param[in] buffer_size The maximum number of bytes to copy.
 * \param[out] recv_size Pointer to an MCC_MEM_SIZE that will contain the number of bytes actually copied into the buffer.
 * \param[in] timeout_us Timeout, in microseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
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
int mcc_recv_copy(MCC_ENDPOINT *endpoint, void *buffer, MCC_MEM_SIZE buffer_size, MCC_MEM_SIZE *recv_size, unsigned int timeout_us)
{
    MCC_RECEIVE_LIST *list;
    MCC_RECEIVE_BUFFER * buf;
    MCC_SIGNAL affiliated_signal;
    MCC_ENDPOINT tmp_destination = {(MCC_CORE)0, (MCC_NODE)0, (MCC_PORT)0};
    int return_value, i = 0;
#if (MCC_OS_USED == MCC_MQX)
    unsigned int time_us_tmp;
    unsigned int lwevent_index = endpoint->port / MCC_MQX_LWEVENT_GROUP_SIZE;
    unsigned int lwevent_group_index = endpoint->port % MCC_MQX_LWEVENT_GROUP_SIZE;
    MQX_TICK_STRUCT tick_time;
#elif (MCC_OS_USED == MCC_LINUX)
    unsigned long timeout_j; /* jiffies */
#endif

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*endpoint);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* The endpoint is not valid */
    if(list == null)
        return MCC_ERR_ENDPOINT;

    if(list->head == (MCC_RECEIVE_BUFFER*)0) {
        /* Non-blocking call */
        if(timeout_us == 0) {
            return MCC_ERR_TIMEOUT;
        }
        /* Blocking call */
        else {
#if (MCC_OS_USED == MCC_MQX)
            if(timeout_us == 0xFFFFFFFF) {
                _lwevent_wait_ticks(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index, TRUE, 0);
            }
            /* timeout_us > 0 */
            else {
                _time_get_ticks(&tick_time);
                _time_add_usec_to_ticks(&tick_time, timeout_us);
                _lwevent_wait_until(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index, TRUE, &tick_time);
            }
#elif (MCC_OS_USED == MCC_LINUX)
            if(timeout_us == 0xFFFFFFFF) {
                wait_event(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1);
                mcc_get_semaphore();
		/*
		 * double check if the list head is null or not,
		 * if yes, wait again.
		 */
                while (list->head == null) {
                    imx_mcc_buffer_queued = 0;
                    mcc_release_semaphore();
                    wait_event(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1);
                    mcc_get_semaphore();
                }
            }
	    else {
                timeout_j = usecs_to_jiffies(timeout_us);
                wait_event_timeout(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1, timeout_j);
                mcc_get_semaphore();
            }

            if (list->head == null) {
                pr_err("%s can't get queued buffer.\n", __func__);
	    }

            list->head = (MCC_RECEIVE_BUFFER*)MQX_TO_VIRT(list->head);
            mcc_release_semaphore();
#endif
        }
    }
#if (MCC_OS_USED == MCC_LINUX)
    else {
        list->head = (MCC_RECEIVE_BUFFER*)MQX_TO_VIRT(list->head);
    }
#endif

#if (MCC_OS_USED == MCC_MQX)
    /* Clear event bit specified for the particular endpoint in the lwevent_buffer_queued lwevent group */
    _lwevent_clear(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index);
#endif

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
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->data_len, sizeof(MCC_MEM_SIZE));
    if (list->head->data_len > buffer_size) {
        list->head->data_len = buffer_size;
    }
    *recv_size = (MCC_MEM_SIZE)(list->head->data_len);
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->data, list->head->data_len);
    mcc_memcpy((void*)list->head->data, buffer, list->head->data_len);
#if (MCC_OS_USED == MCC_LINUX)
    if (imx_mcc_buffer_queued)
        imx_mcc_buffer_queued = 0;
    list->head = (MCC_RECEIVE_BUFFER*)VIRT_TO_MQX(list->head);
#endif

    /* Dequeue the buffer from the endpoint list */
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
    if(return_value != MCC_SUCCESS)
        return return_value;
    mcc_generate_cpu_to_cpu_interrupt();

    return return_value;
}
EXPORT_SYMBOL(mcc_recv_copy);

/*!
 * \brief This function receives a message from the specified endpoint if one is available. The data is NOT copied into the user-app. buffer.
 *
 * This is the "zero-copy receive" version of the MCC receive function. No data is copied. 
 * Only the pointer to the data is returned. This version is fast, but it requires the user to manage
 * buffer allocation. Specifically, the user must decide when a buffer is no longer in use and
 * make the appropriate API call to free it, see mcc_free_buffer.
 *
 * \param[in] endpoint Pointer to the receiving endpoint to receive from.
 * \param[out] buffer_p Pointer to the MCC buffer of the shared memory where the received data is stored.
 * \param[out] recv_size Pointer to an MCC_MEM_SIZE that will contain the number of valid bytes in the buffer.
 * \param[in] timeout_us Timeout, in microseconds, to wait for a free buffer. A value of 0 means don't wait (non-blocking call). A value of 0xffffffff means wait forever (blocking call).
 *
 * \return MCC_SUCCESS
 * \return MCC_ERR_ENDPOINT (the endpoint does not exist)
 * \return MCC_ERR_SEMAPHORE (semaphore handling error)
 * \return MCC_ERR_TIMEOUT (timeout exceeded before a new message came)
 *
 * \see mcc_send
 * \see mcc_recv_copy
 * \see MCC_ENDPOINT
 */
int mcc_recv_nocopy(MCC_ENDPOINT *endpoint, void **buffer_p, MCC_MEM_SIZE *recv_size, unsigned int timeout_us)
{
    MCC_RECEIVE_LIST *list;
    int return_value;
#if (MCC_OS_USED == MCC_MQX)
    unsigned int time_us_tmp;
    unsigned int lwevent_index = endpoint->port / MCC_MQX_LWEVENT_GROUP_SIZE;
    unsigned int lwevent_group_index = endpoint->port % MCC_MQX_LWEVENT_GROUP_SIZE;
    MQX_TICK_STRUCT tick_time;
#elif (MCC_OS_USED == MCC_LINUX)
    unsigned long timeout_j; /* jiffies */
#endif

    /* Semaphore-protected section start */
    return_value = mcc_get_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* Get list of buffers kept by the particular endpoint */
    list = mcc_get_endpoint_list(*endpoint);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    /* The endpoint is not valid */
    if(list == null) {
        return MCC_ERR_ENDPOINT;
    }

    if(list->head == (MCC_RECEIVE_BUFFER*)0) {
        /* Non-blocking call */
        if(timeout_us == 0) {
            return MCC_ERR_TIMEOUT;
        }
        /* Blocking call */
        else {
#if (MCC_OS_USED == MCC_MQX)
            if(timeout_us == 0xFFFFFFFF) {
                _lwevent_wait_ticks(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index, TRUE, 0);
            }
            /* timeout_us > 0 */
            else {
                _time_get_ticks(&tick_time);
                _time_add_usec_to_ticks(&tick_time, timeout_us);
                _lwevent_wait_until(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index, TRUE, &tick_time);
            }
#elif ((MCC_OS_USED == MCC_LINUX))
            if(timeout_us == 0xFFFFFFFF) {
                wait_event(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1);
                mcc_get_semaphore();
		/*
		 * double check if the list head is null or not,
		 * if yes, wait again.
		 */
                while (list->head == null) {
                    imx_mcc_buffer_queued = 0;
                    mcc_release_semaphore();
                    wait_event(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1);
                    mcc_get_semaphore();
                }
            }
	    else {
                timeout_j = usecs_to_jiffies(timeout_us);
                wait_event_timeout(buffer_queued_wait_queue, imx_mcc_buffer_queued == 1, timeout_j);
                mcc_get_semaphore();

                if (list->head == null)
                    pr_err("%s can't get queued buffer.\n", __func__);
            }

            list->head = (MCC_RECEIVE_BUFFER*)MQX_TO_VIRT(list->head);
            mcc_release_semaphore();
#endif
        }
    }
#if (MCC_OS_USED == MCC_LINUX)
    else {
    list->head = (MCC_RECEIVE_BUFFER*)MQX_TO_VIRT(list->head);
    }
#endif

#if (MCC_OS_USED == MCC_MQX)
    /* Clear event bit specified for the particular endpoint in the lwevent_buffer_queued lwevent group */
    _lwevent_clear(&lwevent_buffer_queued[lwevent_index], 1<<lwevent_group_index);
#endif

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
    MCC_DCACHE_INVALIDATE_MLINES((void*)&list->head->data_len, sizeof(MCC_MEM_SIZE));
    *recv_size = (MCC_MEM_SIZE)(list->head->data_len);
#if (MCC_OS_USED == MCC_LINUX)
    list->head = (MCC_RECEIVE_BUFFER*)VIRT_TO_MQX(list->head);
#endif

    /* Dequeue the buffer from the endpoint list */
    mcc_dequeue_buffer(list);

    /* Semaphore-protected section end */
    return_value = mcc_release_semaphore();
    if(return_value != MCC_SUCCESS)
        return return_value;

    return return_value;
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
 * \see mcc_recv_copy
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
    if(return_value != MCC_SUCCESS)
        return return_value;
    mcc_generate_cpu_to_cpu_interrupt();

    return return_value;
}

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
EXPORT_SYMBOL(mcc_get_info);
