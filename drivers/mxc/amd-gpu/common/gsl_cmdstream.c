/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "gsl.h"
#include "gsl_hal.h"
#include "gsl_cmdstream.h"

#ifdef GSL_LOCKING_FINEGRAIN
#define GSL_CMDSTREAM_MUTEX_CREATE()        device->cmdstream_mutex = kos_mutex_create("gsl_cmdstream"); \
                                            if (!device->cmdstream_mutex) return (GSL_FAILURE);
#define GSL_CMDSTREAM_MUTEX_LOCK()          kos_mutex_lock(device->cmdstream_mutex)
#define GSL_CMDSTREAM_MUTEX_UNLOCK()        kos_mutex_unlock(device->cmdstream_mutex)
#define GSL_CMDSTREAM_MUTEX_FREE()          kos_mutex_free(device->cmdstream_mutex); device->cmdstream_mutex = 0;
#else
#define GSL_CMDSTREAM_MUTEX_CREATE()
#define GSL_CMDSTREAM_MUTEX_LOCK()
#define GSL_CMDSTREAM_MUTEX_UNLOCK()
#define GSL_CMDSTREAM_MUTEX_FREE()
#endif


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

int
kgsl_cmdstream_init(gsl_device_t *device)
{
    GSL_CMDSTREAM_MUTEX_CREATE();

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

int
kgsl_cmdstream_close(gsl_device_t *device)
{
    GSL_CMDSTREAM_MUTEX_FREE();

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

gsl_timestamp_t
kgsl_cmdstream_readtimestamp0(gsl_deviceid_t device_id, gsl_timestamp_type_t type)
{
    gsl_timestamp_t   timestamp = -1;
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> gsl_timestamp_t kgsl_cmdstream_readtimestamp(gsl_deviceid_t device_id=%d gsl_timestamp_type_t type=%d)\n", device_id, type );
#if (defined(GSL_BLD_G12) && defined(IRQTHREAD_POLL))
    kos_event_signal(device->irqthread_event);
#endif
    if (type == GSL_TIMESTAMP_CONSUMED)
    {
        // start-of-pipeline timestamp
        GSL_CMDSTREAM_GET_SOP_TIMESTAMP(device, (unsigned int*)&timestamp);
    }
    else if (type == GSL_TIMESTAMP_RETIRED)
    {
		// end-of-pipeline timestamp
		GSL_CMDSTREAM_GET_EOP_TIMESTAMP(device, (unsigned int*)&timestamp);
    }
    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_readtimestamp. Return value %d\n", timestamp );
    return (timestamp);
}

//----------------------------------------------------------------------------

KGSL_API gsl_timestamp_t
kgsl_cmdstream_readtimestamp(gsl_deviceid_t device_id, gsl_timestamp_type_t type)
{
	gsl_timestamp_t timestamp = -1;
	GSL_API_MUTEX_LOCK();
	timestamp = kgsl_cmdstream_readtimestamp0(device_id, type);
	GSL_API_MUTEX_UNLOCK();
	return timestamp;
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_cmdstream_issueibcmds(gsl_deviceid_t device_id, int drawctxt_index, gpuaddr_t ibaddr, int sizedwords, gsl_timestamp_t *timestamp, unsigned int flags)
{
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
    int status = GSL_FAILURE;
    GSL_API_MUTEX_LOCK();
    
    kgsl_device_active(device);
     
    if (device->ftbl.cmdstream_issueibcmds)
    {
        status = device->ftbl.cmdstream_issueibcmds(device, drawctxt_index, ibaddr, sizedwords, timestamp, flags);
    }
    GSL_API_MUTEX_UNLOCK();
    return status;
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_add_timestamp(gsl_deviceid_t device_id, gsl_timestamp_t *timestamp)
{
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
    int status = GSL_FAILURE;
    GSL_API_MUTEX_LOCK();
    if (device->ftbl.device_addtimestamp)
    {
        status = device->ftbl.device_addtimestamp(device, timestamp);
    }
    GSL_API_MUTEX_UNLOCK();
    return status;
}

//----------------------------------------------------------------------------

KGSL_API
int kgsl_cmdstream_waittimestamp(gsl_deviceid_t device_id, gsl_timestamp_t timestamp, unsigned int timeout)
{
    gsl_device_t* device  = &gsl_driver.device[device_id-1];
	int status = GSL_FAILURE;
	if (device->ftbl.device_waittimestamp)
    {
        status = device->ftbl.device_waittimestamp(device, timestamp, timeout);
    }
    return status;
}

//----------------------------------------------------------------------------

void
kgsl_cmdstream_memqueue_drain(gsl_device_t *device)
{
    gsl_memnode_t     *memnode, *nextnode, *freehead;
    gsl_timestamp_t   timestamp, ts_processed;
    gsl_memqueue_t    *memqueue = &device->memqueue;

    GSL_CMDSTREAM_MUTEX_LOCK();

    // check head
    if (memqueue->head == NULL)
    {
        GSL_CMDSTREAM_MUTEX_UNLOCK();
        return;
    }
    // get current EOP timestamp
    ts_processed = kgsl_cmdstream_readtimestamp0(device->id, GSL_TIMESTAMP_RETIRED);
    timestamp = memqueue->head->timestamp;
    // check head timestamp
    if (!(((ts_processed - timestamp) >= 0) || ((ts_processed - timestamp) < -GSL_TIMESTAMP_EPSILON)))
    {
        GSL_CMDSTREAM_MUTEX_UNLOCK();
        return;
    }
    memnode  = memqueue->head;
    freehead = memqueue->head;
    // get node list to free
    for(;;)
    {
        nextnode  = memnode->next;
        if (nextnode == NULL)
        {
            // entire queue drained
            memqueue->head = NULL;
            memqueue->tail = NULL;
            break;
        }
        timestamp = nextnode->timestamp;
        if (!(((ts_processed - timestamp) >= 0) || ((ts_processed - timestamp) < -GSL_TIMESTAMP_EPSILON)))
        {
            // drained up to a point
            memqueue->head = nextnode;
            memnode->next  = NULL;
            break;
        }
        memnode = nextnode;
    }
    // free nodes
    while (freehead)
    {
        memnode  = freehead;
        freehead = memnode->next;
        kgsl_sharedmem_free0(&memnode->memdesc, memnode->pid);
        kos_free(memnode);
    }

    GSL_CMDSTREAM_MUTEX_UNLOCK();
}

//----------------------------------------------------------------------------

int
kgsl_cmdstream_freememontimestamp(gsl_deviceid_t device_id, gsl_memdesc_t *memdesc, gsl_timestamp_t timestamp, gsl_timestamp_type_t type)
{
    gsl_memnode_t  *memnode;
    gsl_device_t   *device = &gsl_driver.device[device_id-1];
    gsl_memqueue_t *memqueue;
    (void)type; // unref. For now just use EOP timestamp

	GSL_API_MUTEX_LOCK();
	GSL_CMDSTREAM_MUTEX_LOCK();

	memqueue = &device->memqueue;

    memnode  = kos_malloc(sizeof(gsl_memnode_t));

    if (!memnode)
    {
        // other solution is to idle and free which given that the upper level driver probably wont check, probably a better idea
		GSL_CMDSTREAM_MUTEX_UNLOCK();
		GSL_API_MUTEX_UNLOCK();
        return (GSL_FAILURE);
    }

    memnode->timestamp = timestamp;
    memnode->pid       = GSL_CALLER_PROCESSID_GET();
    memnode->next      = NULL;
    kos_memcpy(&memnode->memdesc, memdesc, sizeof(gsl_memdesc_t));

    // add to end of queue
    if (memqueue->tail != NULL)
    {
        memqueue->tail->next = memnode;
        memqueue->tail       = memnode;
    }
    else
    {
        KOS_ASSERT(memqueue->head == NULL);
        memqueue->head = memnode;
        memqueue->tail = memnode;
    }

    GSL_CMDSTREAM_MUTEX_UNLOCK();
	GSL_API_MUTEX_UNLOCK();

    return (GSL_SUCCESS);
}

static int kgsl_cmdstream_timestamp_cmp(gsl_timestamp_t ts_new, gsl_timestamp_t ts_old)
{
	gsl_timestamp_t ts_diff = ts_new - ts_old;
	return (ts_diff >= 0) || (ts_diff < -GSL_TIMESTAMP_EPSILON);
}

int kgsl_cmdstream_check_timestamp(gsl_deviceid_t device_id, gsl_timestamp_t timestamp)
{
	gsl_timestamp_t ts_processed;
	ts_processed = kgsl_cmdstream_readtimestamp0(device_id, GSL_TIMESTAMP_RETIRED);
	return kgsl_cmdstream_timestamp_cmp(ts_processed, timestamp);
}
