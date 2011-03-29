/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __GSL_DEVICE_H
#define __GSL_DEVICE_H

#ifdef _LINUX
#include <linux/wait.h>
#include <linux/workqueue.h>
#endif

//////////////////////////////////////////////////////////////////////////////
//  types
//////////////////////////////////////////////////////////////////////////////

// --------------
// function table
// --------------
typedef struct _gsl_functable_t {
    int (*device_init)            (gsl_device_t *device);
    int (*device_close)           (gsl_device_t *device);
    int (*device_destroy)         (gsl_device_t *device);
    int (*device_start)           (gsl_device_t *device, gsl_flags_t flags);
    int (*device_stop)            (gsl_device_t *device);
    int (*device_getproperty)     (gsl_device_t *device, gsl_property_type_t type, void *value, unsigned int sizebytes);
    int (*device_setproperty)     (gsl_device_t *device, gsl_property_type_t type, void *value, unsigned int sizebytes);
    int (*device_idle)            (gsl_device_t *device, unsigned int timeout);
    int (*device_regread)         (gsl_device_t *device, unsigned int offsetwords, unsigned int *value);
    int (*device_regwrite)        (gsl_device_t *device, unsigned int offsetwords, unsigned int value);
    int (*device_waitirq)         (gsl_device_t *device, gsl_intrid_t intr_id, unsigned int *count, unsigned int timeout);
	int (*device_waittimestamp)   (gsl_device_t *device, gsl_timestamp_t timestamp, unsigned int timeout);
    int (*device_runpending)      (gsl_device_t *device);
    int (*device_addtimestamp)    (gsl_device_t *device_id, gsl_timestamp_t *timestamp);
    int (*intr_isr)               (gsl_device_t *device);
	int (*mmu_tlbinvalidate)      (gsl_device_t *device, unsigned int reg_invalidate, unsigned int pid);
	int (*mmu_setpagetable)       (gsl_device_t *device, unsigned int reg_ptbase, gpuaddr_t ptbase, unsigned int pid);
    int (*cmdstream_issueibcmds)  (gsl_device_t *device, int drawctxt_index, gpuaddr_t ibaddr, int sizedwords, gsl_timestamp_t *timestamp, gsl_flags_t flags);
    int (*context_create)         (gsl_device_t *device, gsl_context_type_t type, unsigned int *drawctxt_id, gsl_flags_t flags);
    int (*context_destroy)        (gsl_device_t *device_id, unsigned int drawctxt_id);
} gsl_functable_t;

// -------------
// device object 
// -------------
struct _gsl_device_t {

    unsigned int      refcnt;
    unsigned int      callerprocess[GSL_CALLER_PROCESS_MAX];    // caller process table
    gsl_functable_t   ftbl;
    gsl_flags_t       flags;
    gsl_deviceid_t    id;
    unsigned int      chip_id;
    gsl_memregion_t   regspace;
    gsl_intr_t        intr;
    gsl_memdesc_t     memstore;
    gsl_memqueue_t    memqueue; // queue of memfrees pending timestamp elapse 

#ifdef  GSL_DEVICE_SHADOW_MEMSTORE_TO_USER
    unsigned int      memstoreshadow[GSL_CALLER_PROCESS_MAX];
#endif // GSL_DEVICE_SHADOW_MEMSTORE_TO_USER

#ifndef GSL_NO_MMU
    gsl_mmu_t         mmu;
#endif // GSL_NO_MMU

#ifdef GSL_BLD_YAMATO
    gsl_memregion_t   gmemspace;
    gsl_ringbuffer_t  ringbuffer;
#ifdef GSL_LOCKING_FINEGRAIN
    oshandle_t        drawctxt_mutex;
#endif
    unsigned int      drawctxt_count;
    gsl_drawctxt_t    *drawctxt_active;
    gsl_drawctxt_t    drawctxt[GSL_CONTEXT_MAX];
#endif // GSL_BLD_YAMATO

#ifdef GSL_BLD_G12
#ifdef GSL_LOCKING_FINEGRAIN
    oshandle_t        cmdwindow_mutex;
#endif
    unsigned int      intrcnt[GSL_G12_INTR_COUNT];
    gsl_timestamp_t   current_timestamp;
    gsl_timestamp_t   timestamp;
#ifndef _LINUX	
    unsigned int      irq_thread;
    oshandle_t        irq_thread_handle;
#endif
#ifdef IRQTHREAD_POLL
    oshandle_t        irqthread_event;
#endif
#endif // GSL_BLD_G12
#ifdef GSL_LOCKING_FINEGRAIN
    oshandle_t        cmdstream_mutex;
#endif
#ifndef _LINUX	
    oshandle_t        timestamp_event;
#else
	wait_queue_head_t timestamp_waitq;
	struct workqueue_struct	*irq_workq;
	struct work_struct irq_work;	
	struct work_struct irq_err_work;
#endif
    void              *autogate;
};


//////////////////////////////////////////////////////////////////////////////
//  prototypes
//////////////////////////////////////////////////////////////////////////////
int     kgsl_device_init(gsl_device_t *device, gsl_deviceid_t device_id);
int     kgsl_device_close(gsl_device_t *device);
int     kgsl_device_destroy(gsl_device_t *device);
int     kgsl_device_attachcallback(gsl_device_t *device, unsigned int pid);
int     kgsl_device_detachcallback(gsl_device_t *device, unsigned int pid);
int     kgsl_device_runpending(gsl_device_t *device);

int     kgsl_yamato_getfunctable(gsl_functable_t *ftbl);
int     kgsl_g12_getfunctable(gsl_functable_t *ftbl);

int kgsl_clock(gsl_deviceid_t dev, int enable);
int kgsl_device_active(gsl_device_t *dev);
int kgsl_device_clock(gsl_deviceid_t id, int enable);
int kgsl_device_autogate_init(gsl_device_t *dev);
void kgsl_device_autogate_exit(gsl_device_t *dev);


#endif  // __GSL_DEVICE_H
