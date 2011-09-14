//------------------------------------------------------------------------------
// <copyright file="hif.c" company="Atheros">
//    Copyright (c) 2004-2008 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// HIF layer reference implementation for Linux Native MMC stack
//
// Author(s): ="Atheros"
//==============================================================================
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/kthread.h>

/* by default setup a bounce buffer for the data packets, if the underlying host controller driver
   does not use DMA you may be able to skip this step and save the memory allocation and transfer time */
#define HIF_USE_DMA_BOUNCE_BUFFER 1
#include "hif_internal.h"
#define ATH_MODULE_NAME hif
#include "a_debug.h"


#if HIF_USE_DMA_BOUNCE_BUFFER
/* macro to check if DMA buffer is WORD-aligned and DMA-able.  Most host controllers assume the
 * buffer is DMA'able and will bug-check otherwise (i.e. buffers on the stack).  
 * virt_addr_valid check fails on stack memory.  
 */
#define BUFFER_NEEDS_BOUNCE(buffer)  (((A_UINT32)(buffer) & 0x3) || !virt_addr_valid((buffer)))
#else
#define BUFFER_NEEDS_BOUNCE(buffer)   (FALSE)
#endif

/* ATHENV */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && defined(CONFIG_PM)
#define dev_to_sdio_func(d)	container_of(d, struct sdio_func, dev)
#define to_sdio_driver(d)      container_of(d, struct sdio_driver, drv)
static int hifDeviceSuspend(struct device *dev);
static int hifDeviceResume(struct device *dev);
#endif /* CONFIG_PM */
static int hifDeviceInserted(struct sdio_func *func, const struct sdio_device_id *id);
static void hifDeviceRemoved(struct sdio_func *func);
static HIF_DEVICE *addHifDevice(struct sdio_func *func);
static HIF_DEVICE *getHifDevice(struct sdio_func *func);
static void delHifDevice(HIF_DEVICE * device);
static int Func0_CMD52WriteByte(struct mmc_card *card, unsigned int address, unsigned char byte);

int reset_sdio_on_unload = 0;
module_param(reset_sdio_on_unload, int, 0644);

extern A_UINT32 nohifscattersupport;


/* ------ Static Variables ------ */
static const struct sdio_device_id ar6k_id_table[] = {
    {  SDIO_DEVICE(MANUFACTURER_CODE, (MANUFACTURER_ID_AR6002_BASE | 0x0))  },
    {  SDIO_DEVICE(MANUFACTURER_CODE, (MANUFACTURER_ID_AR6002_BASE | 0x1))  },
    {  SDIO_DEVICE(MANUFACTURER_CODE, (MANUFACTURER_ID_AR6003_BASE | 0x0))  },
    {  SDIO_DEVICE(MANUFACTURER_CODE, (MANUFACTURER_ID_AR6003_BASE | 0x1))  },
    { /* null */                                         },
};
MODULE_DEVICE_TABLE(sdio, ar6k_id_table);

static struct sdio_driver ar6k_driver = {
	.name = "ar6k_wlan",
	.id_table = ar6k_id_table,
	.probe = hifDeviceInserted,
	.remove = hifDeviceRemoved,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && defined(CONFIG_PM)
/* New suspend/resume based on linux-2.6.32
 * Need to patch linux-2.6.32 with mmc2.6.32_suspend.patch
 * Need to patch with msmsdcc2.6.29_suspend.patch for msm_sdcc host
     */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static struct dev_pm_ops ar6k_device_pm_ops = {
#else
static struct pm_ops ar6k_device_pm_ops = {
#endif 
	.suspend = hifDeviceSuspend,
	.resume = hifDeviceResume,
};
#endif /* CONFIG_PM */

/* make sure we only unregister when registered. */
static int registered = 0;

OSDRV_CALLBACKS osdrvCallbacks;
extern A_UINT32 onebitmode;
extern A_UINT32 busspeedlow;
extern A_UINT32 debughif;

static void ResetAllCards(void);

#ifdef DEBUG

ATH_DEBUG_INSTANTIATE_MODULE_VAR(hif,
                                 "hif",
                                 "(Linux MMC) Host Interconnect Framework",
                                 ATH_DEBUG_MASK_DEFAULTS,
                                 0,
                                 NULL);
                                 
#endif


/* ------ Functions ------ */
A_STATUS HIFInit(OSDRV_CALLBACKS *callbacks)
{
    int status;
    AR_DEBUG_ASSERT(callbacks != NULL);

    A_REGISTER_MODULE_DEBUG_INFO(hif);

    /* store the callback handlers */
    osdrvCallbacks = *callbacks;

    /* Register with bus driver core */
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: HIFInit registering\n"));
    registered = 1;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && defined(CONFIG_PM)
    if (callbacks->deviceSuspendHandler && callbacks->deviceResumeHandler) {
        ar6k_driver.drv.pm = &ar6k_device_pm_ops;
    }
#endif /* CONFIG_PM */
    status = sdio_register_driver(&ar6k_driver);
    AR_DEBUG_ASSERT(status==0);

    if (status != 0) {
        return A_ERROR;
    }

    return A_OK;

}

static A_STATUS
__HIFReadWrite(HIF_DEVICE *device,
             A_UINT32 address,
             A_UCHAR *buffer,
             A_UINT32 length,
             A_UINT32 request,
             void *context)
{
    A_UINT8 opcode;
    A_STATUS    status = A_OK;
    int     ret;
    A_UINT8 *tbuffer;
    A_BOOL   bounced = FALSE;

    AR_DEBUG_ASSERT(device != NULL);
    AR_DEBUG_ASSERT(device->func != NULL);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Device: 0x%p, buffer:0x%p (addr:0x%X)\n", 
                    device, buffer, address));

    do {
        if (request & HIF_EXTENDED_IO) {
            //AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Command type: CMD53\n"));
        } else {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: Invalid command type: 0x%08x\n", request));
            status = A_EINVAL;
            break;
        }

        if (request & HIF_BLOCK_BASIS) {
            /* round to whole block length size */
            length = (length / HIF_MBOX_BLOCK_SIZE) * HIF_MBOX_BLOCK_SIZE;
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE,
                            ("AR6000: Block mode (BlockLen: %d)\n",
                            length));
        } else if (request & HIF_BYTE_BASIS) {
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE,
                            ("AR6000: Byte mode (BlockLen: %d)\n",
                            length));
        } else {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: Invalid data mode: 0x%08x\n", request));
            status = A_EINVAL;
            break;
        }

#if 0
        /* useful for checking register accesses */
        if (length & 0x3) {
            A_PRINTF(KERN_ALERT"AR6000: HIF (%s) is not a multiple of 4 bytes, addr:0x%X, len:%d\n",
                                request & HIF_WRITE ? "write":"read", address, length);
        }
#endif

        if (request & HIF_WRITE) {
            if ((address >= HIF_MBOX_START_ADDR(0)) &&
                (address <= HIF_MBOX_END_ADDR(3)))
            {
    
                AR_DEBUG_ASSERT(length <= HIF_MBOX_WIDTH);
    
                /*
                 * Mailbox write. Adjust the address so that the last byte
                 * falls on the EOM address.
                 */
                address += (HIF_MBOX_WIDTH - length);
            }
        }

        if (request & HIF_FIXED_ADDRESS) {
            opcode = CMD53_FIXED_ADDRESS;
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Address mode: Fixed 0x%X\n", address));
        } else if (request & HIF_INCREMENTAL_ADDRESS) {
            opcode = CMD53_INCR_ADDRESS;
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Address mode: Incremental 0x%X\n", address));
        } else {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: Invalid address mode: 0x%08x\n", request));
            status = A_EINVAL;
            break;
        }

        if (request & HIF_WRITE) {
#if HIF_USE_DMA_BOUNCE_BUFFER
            if (BUFFER_NEEDS_BOUNCE(buffer)) {
                AR_DEBUG_ASSERT(device->dma_buffer != NULL);
                tbuffer = device->dma_buffer;
                    /* copy the write data to the dma buffer */
                AR_DEBUG_ASSERT(length <= HIF_DMA_BUFFER_SIZE);
                memcpy(tbuffer, buffer, length);
                bounced = TRUE;
            } else {
                tbuffer = buffer;    
            }
#else
	        tbuffer = buffer;
#endif
            if (opcode == CMD53_FIXED_ADDRESS) {
                ret = sdio_writesb(device->func, address, tbuffer, length);
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: writesb ret=%d address: 0x%X, len: %d, 0x%X\n",
						  ret, address, length, *(int *)tbuffer));
            } else {
                ret = sdio_memcpy_toio(device->func, address, tbuffer, length);
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: writeio ret=%d address: 0x%X, len: %d, 0x%X\n",
						  ret, address, length, *(int *)tbuffer));
            }
        } else if (request & HIF_READ) {
#if HIF_USE_DMA_BOUNCE_BUFFER
            if (BUFFER_NEEDS_BOUNCE(buffer)) {
                AR_DEBUG_ASSERT(device->dma_buffer != NULL);
                AR_DEBUG_ASSERT(length <= HIF_DMA_BUFFER_SIZE);
                tbuffer = device->dma_buffer;
                bounced = TRUE;
            } else {
                tbuffer = buffer;    
            }
#else
            tbuffer = buffer;
#endif
            if (opcode == CMD53_FIXED_ADDRESS) {
                ret = sdio_readsb(device->func, tbuffer, address, length);
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: readsb ret=%d address: 0x%X, len: %d, 0x%X\n",
						  ret, address, length, *(int *)tbuffer));
            } else {
                ret = sdio_memcpy_fromio(device->func, tbuffer, address, length);
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: readio ret=%d address: 0x%X, len: %d, 0x%X\n",
						  ret, address, length, *(int *)tbuffer));
            }
#if HIF_USE_DMA_BOUNCE_BUFFER
            if (bounced) {
    	           /* copy the read data from the dma buffer */
                memcpy(buffer, tbuffer, length);
            }
#endif
        } else {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: Invalid direction: 0x%08x\n", request));
            status = A_EINVAL;
            break;
        }

        if (ret) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: SDIO bus operation failed! MMC stack returned : %d \n", ret));
            status = A_ERROR;
        }
    } while (FALSE);

    return status;
}

void AddToAsyncList(HIF_DEVICE *device, BUS_REQUEST *busrequest)
{
    unsigned long flags;
    BUS_REQUEST *async;
    BUS_REQUEST *active;
    
    spin_lock_irqsave(&device->asynclock, flags);
    active = device->asyncreq;
    if (active == NULL) {
        device->asyncreq = busrequest;
        device->asyncreq->inusenext = NULL;
    } else {
        for (async = device->asyncreq;
             async != NULL;
             async = async->inusenext) {
             active =  async;
        }
        active->inusenext = busrequest;
        busrequest->inusenext = NULL;
    }
    spin_unlock_irqrestore(&device->asynclock, flags);
}


/* queue a read/write request */
A_STATUS
HIFReadWrite(HIF_DEVICE *device,
             A_UINT32 address,
             A_UCHAR *buffer,
             A_UINT32 length,
             A_UINT32 request,
             void *context)
{
    A_STATUS    status = A_OK;
    BUS_REQUEST *busrequest;


    AR_DEBUG_ASSERT(device != NULL);
    AR_DEBUG_ASSERT(device->func != NULL);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Device: %p addr:0x%X\n", device,address));

    do {            
        if ((request & HIF_ASYNCHRONOUS) || (request & HIF_SYNCHRONOUS)){
            /* serialize all requests through the async thread */
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Execution mode: %s\n", 
                        (request & HIF_ASYNCHRONOUS)?"Async":"Synch"));
            busrequest = hifAllocateBusRequest(device);
            if (busrequest == NULL) {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, 
                    ("AR6000: no async bus requests available (%s, addr:0x%X, len:%d) \n", 
                        request & HIF_READ ? "READ":"WRITE", address, length));
                return A_ERROR;
            }
            busrequest->address = address;
            busrequest->buffer = buffer;
            busrequest->length = length;
            busrequest->request = request;
            busrequest->context = context;
            
            AddToAsyncList(device, busrequest);
            
            if (request & HIF_SYNCHRONOUS) {
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: queued sync req: 0x%X\n", (unsigned int)busrequest));

                /* wait for completion */
                up(&device->sem_async);
                if (down_interruptible(&busrequest->sem_req) != 0) {
                    /* interrupted, exit */
                    return A_ERROR;
                } else {
                    A_STATUS status = busrequest->status;
                    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: sync return freeing 0x%X: 0x%X\n", 
						      (unsigned int)busrequest, busrequest->status));
                    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: freeing req: 0x%X\n", (unsigned int)request));
                    hifFreeBusRequest(device, busrequest);
                    return status;
                }
            } else {
                AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: queued async req: 0x%X\n", (unsigned int)busrequest));
                up(&device->sem_async);
                return A_PENDING;
            }
        } else {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR,
                            ("AR6000: Invalid execution mode: 0x%08x\n", (unsigned int)request));
            status = A_EINVAL;
            break;
        }
    } while(0);

    return status;
}
/* thread to serialize all requests, both sync and async */
static int async_task(void *param)
 {
    HIF_DEVICE *device;
    BUS_REQUEST *request;
    A_STATUS status;
    unsigned long flags;

    device = (HIF_DEVICE *)param;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async task\n"));
    set_current_state(TASK_INTERRUPTIBLE);
    while(!device->async_shutdown) {
        /* wait for work */
        if (down_interruptible(&device->sem_async) != 0) {
            /* interrupted, exit */
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async task interrupted\n"));
            break;
        }
        if (device->async_shutdown) {
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async task stopping\n"));
            break;
        }
        /* we want to hold the host over multiple cmds if possible, but holding the host blocks card interrupts */
        sdio_claim_host(device->func);
        spin_lock_irqsave(&device->asynclock, flags);
        /* pull the request to work on */
        while (device->asyncreq != NULL) {
            request = device->asyncreq;
            if (request->inusenext != NULL) {
                device->asyncreq = request->inusenext;
            } else {
                device->asyncreq = NULL;
            }
            spin_unlock_irqrestore(&device->asynclock, flags);
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async_task processing req: 0x%X\n", (unsigned int)request));
            
            if (request->pScatterReq != NULL) {
                A_ASSERT(device->scatter_enabled);
                    /* this is a queued scatter request, pass the request to scatter routine which
                     * executes it synchronously, note, no need to free the request since scatter requests
                     * are maintained on a separate list */
                status = DoHifReadWriteScatter(device,request);
            } else {                
                    /* call HIFReadWrite in sync mode to do the work */
                status = __HIFReadWrite(device, request->address, request->buffer,
                                      request->length, request->request & ~HIF_SYNCHRONOUS, NULL);
                if (request->request & HIF_ASYNCHRONOUS) {
                    void *context = request->context;
                    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async_task freeing req: 0x%X\n", (unsigned int)request));
                    hifFreeBusRequest(device, request);
                    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async_task completion routine req: 0x%X\n", (unsigned int)request));
                    device->htcCallbacks.rwCompletionHandler(context, status);
                } else {
                    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: async_task upping req: 0x%X\n", (unsigned int)request));
                    request->status = status;
                    up(&request->sem_req);
                }
            }
            spin_lock_irqsave(&device->asynclock, flags);
        }
        spin_unlock_irqrestore(&device->asynclock, flags);
        sdio_release_host(device->func);
    }

    complete_and_exit(&device->async_completion, 0);
    return 0;
 }

A_STATUS
HIFConfigureDevice(HIF_DEVICE *device, HIF_DEVICE_CONFIG_OPCODE opcode,
                   void *config, A_UINT32 configLen)
{
    A_UINT32 count;
    A_STATUS status;
    
    switch(opcode) {
        case HIF_DEVICE_GET_MBOX_BLOCK_SIZE:
            ((A_UINT32 *)config)[0] = HIF_MBOX0_BLOCK_SIZE;
            ((A_UINT32 *)config)[1] = HIF_MBOX1_BLOCK_SIZE;
            ((A_UINT32 *)config)[2] = HIF_MBOX2_BLOCK_SIZE;
            ((A_UINT32 *)config)[3] = HIF_MBOX3_BLOCK_SIZE;
            break;

        case HIF_DEVICE_GET_MBOX_ADDR:
            for (count = 0; count < 4; count ++) {
                ((A_UINT32 *)config)[count] = HIF_MBOX_START_ADDR(count);
            }
            
            if (configLen >= sizeof(HIF_DEVICE_MBOX_INFO)) {    
                SetExtendedMboxWindowInfo((A_UINT16)device->func->device,
                                          (HIF_DEVICE_MBOX_INFO *)config);
            }
                        
            break;
        case HIF_DEVICE_GET_IRQ_PROC_MODE:
            *((HIF_DEVICE_IRQ_PROCESSING_MODE *)config) = HIF_DEVICE_IRQ_SYNC_ONLY;
            break;
       case HIF_CONFIGURE_QUERY_SCATTER_REQUEST_SUPPORT:
            if (!device->scatter_enabled) {
                return A_ENOTSUP;    
            }
            status = SetupHIFScatterSupport(device, (HIF_DEVICE_SCATTER_SUPPORT_INFO *)config);
            if (A_FAILED(status)) {
                device->scatter_enabled = FALSE;           
            }
            return status;
        case HIF_DEVICE_GET_OS_DEVICE:
                /* pass back a pointer to the SDIO function's "dev" struct */
            ((HIF_DEVICE_OS_DEVICE_INFO *)config)->pOSDevice = &device->func->dev;
            break; 
        default:
            AR_DEBUG_PRINTF(ATH_DEBUG_WARN,
                            ("AR6000: Unsupported configuration opcode: %d\n", opcode));
            return A_ERROR;
    }

    return A_OK;
}

void
HIFShutDownDevice(HIF_DEVICE *device)
{
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: +HIFShutDownDevice\n"));
    if (device != NULL) {
        AR_DEBUG_ASSERT(device->func != NULL);
    } else {
            /* since we are unloading the driver anyways, reset all cards in case the SDIO card
             * is externally powered and we are unloading the SDIO stack.  This avoids the problem when
             * the SDIO stack is reloaded and attempts are made to re-enumerate a card that is already
             * enumerated */
        AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: HIFShutDownDevice, resetting\n"));
        ResetAllCards();

        /* Unregister with bus driver core */
        if (registered) {
            registered = 0;
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE,
                            ("AR6000: Unregistering with the bus driver\n"));
            sdio_unregister_driver(&ar6k_driver);
            AR_DEBUG_PRINTF(ATH_DEBUG_TRACE,
                            ("AR6000: Unregistered\n"));
        }
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: -HIFShutDownDevice\n"));
}

static void
hifIRQHandler(struct sdio_func *func)
{
    A_STATUS status;
    HIF_DEVICE *device;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: +hifIRQHandler\n"));

    device = getHifDevice(func);
    /* release the host during ints so we can pick it back up when we process cmds */
    sdio_release_host(device->func);
    status = device->htcCallbacks.dsrHandler(device->htcCallbacks.context);
    sdio_claim_host(device->func);
    AR_DEBUG_ASSERT(status == A_OK || status == A_ECANCELED);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: -hifIRQHandler\n"));
}

/* handle HTC startup via thread*/
static int startup_task(void *param)
{
    HIF_DEVICE *device;

    device = (HIF_DEVICE *)param;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: call HTC from startup_task\n"));
        /* start  up inform DRV layer */
    if ((osdrvCallbacks.deviceInsertedHandler(osdrvCallbacks.context,device)) != A_OK) {
        AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Device rejected\n"));
    }
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && defined(CONFIG_PM)
/* handle HTC startup via thread*/
static int resume_task(void *param)
{
    HIF_DEVICE *device;
    device = (HIF_DEVICE *)param;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: call HTC from resume_task\n"));
        /* start  up inform DRV layer */
    if (device && device->claimedContext && osdrvCallbacks.deviceResumeHandler &&
        osdrvCallbacks.deviceResumeHandler(device->claimedContext) != A_OK) {
        AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: Device rejected\n"));
    }
    return 0;
}
#endif /* CONFIG_PM */

static int hifDeviceInserted(struct sdio_func *func, const struct sdio_device_id *id)
{
    int ret;
    HIF_DEVICE * device;
    int count;
    struct task_struct* startup_task_struct;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE,
		    ("AR6000: hifDeviceInserted, Function: 0x%X, Vendor ID: 0x%X, Device ID: 0x%X, block size: 0x%X/0x%X\n",
		     func->num, func->vendor, func->device, func->max_blksize, func->cur_blksize));

    addHifDevice(func);
    device = getHifDevice(func);

    spin_lock_init(&device->lock);

    spin_lock_init(&device->asynclock);
    
    DL_LIST_INIT(&device->ScatterReqHead);
    
    if (!nohifscattersupport) {
            /* try to allow scatter operation on all instances,
             * unless globally overridden */
        device->scatter_enabled = TRUE;
    }
    
        /* enable the SDIO function */
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: claim\n"));
    sdio_claim_host(func);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: enable\n"));

    if ((id->device & MANUFACTURER_ID_AR6K_BASE_MASK) >= MANUFACTURER_ID_AR6003_BASE) {
        /* enable 4-bit ASYNC interrupt on AR6003 or later devices */    
        ret = Func0_CMD52WriteByte(func->card, CCCR_SDIO_IRQ_MODE_REG, SDIO_IRQ_MODE_ASYNC_4BIT_IRQ);
        if (ret) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("AR6000: failed to enable 4-bit ASYNC IRQ mode %d \n",ret));    
            sdio_release_host(func);
            return ret;
        }
        AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: 4-bit ASYNC IRQ mode enabled\n"));   
    }
    
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
    /* give us some time to enable, in ms */
    func->enable_timeout = 100;
#endif
    ret = sdio_enable_func(func);
    if (ret) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), Unable to enable AR6K: 0x%X\n",
					  __FUNCTION__, ret)); 
        sdio_release_host(func);
        return ret;
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: set block size 0x%X\n", HIF_MBOX_BLOCK_SIZE));
    ret = sdio_set_block_size(func, HIF_MBOX_BLOCK_SIZE);
    sdio_release_host(func);
    if (ret) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), Unable to set block size 0x%x  AR6K: 0x%X\n",
					  __FUNCTION__, HIF_MBOX_BLOCK_SIZE, ret));
        return ret;
    }
    /* Initialize the bus requests to be used later */
    A_MEMZERO(device->busRequest, sizeof(device->busRequest));
    for (count = 0; count < BUS_REQUEST_MAX_NUM; count ++) {
        sema_init(&device->busRequest[count].sem_req, 0);
        hifFreeBusRequest(device, &device->busRequest[count]);
    }

    /* create async I/O thread */
    device->async_shutdown = 0;
    device->async_task = kthread_create(async_task,
                                       (void *)device,
                                       "AR6K Async");
    if (IS_ERR(device->async_task)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), to create async task\n", __FUNCTION__));
        return A_ERROR;
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: start async task\n"));
    sema_init(&device->sem_async, 0);
    wake_up_process(device->async_task );

    /* create startup thread */
    startup_task_struct = kthread_create(startup_task,
                                  (void *)device,
                                  "AR6K startup");
    if (IS_ERR(startup_task_struct)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), to create startup task\n", __FUNCTION__));
        return A_ERROR;
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: start startup task\n"));
    wake_up_process(startup_task_struct);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: return %d\n", ret));
    return ret;
}


void
HIFAckInterrupt(HIF_DEVICE *device)
{
    AR_DEBUG_ASSERT(device != NULL);

    /* Acknowledge our function IRQ */
}

void
HIFUnMaskInterrupt(HIF_DEVICE *device)
{
    int ret;;

    AR_DEBUG_ASSERT(device != NULL);
    AR_DEBUG_ASSERT(device->func != NULL);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: HIFUnMaskInterrupt\n"));

    /* Register the IRQ Handler */
    sdio_claim_host(device->func);
    ret = sdio_claim_irq(device->func, hifIRQHandler);
    sdio_release_host(device->func);
    AR_DEBUG_ASSERT(ret == 0);
}

void HIFMaskInterrupt(HIF_DEVICE *device)
{
    int ret;;

    AR_DEBUG_ASSERT(device != NULL);
    AR_DEBUG_ASSERT(device->func != NULL);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: HIFMaskInterrupt\n"));

    /* Mask our function IRQ */
    sdio_claim_host(device->func);
    ret = sdio_release_irq(device->func);
    sdio_release_host(device->func);
    AR_DEBUG_ASSERT(ret == 0);
}

BUS_REQUEST *hifAllocateBusRequest(HIF_DEVICE *device)
{
    BUS_REQUEST *busrequest;
    unsigned long flag;

    /* Acquire lock */
    spin_lock_irqsave(&device->lock, flag);

    /* Remove first in list */
    if((busrequest = device->s_busRequestFreeQueue) != NULL)
    {
        device->s_busRequestFreeQueue = busrequest->next;
    }

    /* Release lock */
    spin_unlock_irqrestore(&device->lock, flag);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: hifAllocateBusRequest: 0x%p\n", busrequest));
    return busrequest;
}

void
hifFreeBusRequest(HIF_DEVICE *device, BUS_REQUEST *busrequest)
{
    unsigned long flag;

    AR_DEBUG_ASSERT(busrequest != NULL);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: hifFreeBusRequest: 0x%p\n", busrequest));
    /* Acquire lock */
    spin_lock_irqsave(&device->lock, flag);


    /* Insert first in list */
    busrequest->next = device->s_busRequestFreeQueue;
    busrequest->inusenext = NULL;
    device->s_busRequestFreeQueue = busrequest;

    /* Release lock */
    spin_unlock_irqrestore(&device->lock, flag);
}

static int hifDisableFunc(HIF_DEVICE *device, struct sdio_func *func)
{
    int ret = 0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: +hifDeviceRemoved\n"));
    device = getHifDevice(func);
    if (!IS_ERR(device->async_task)) {
        init_completion(&device->async_completion);
        device->async_shutdown = 1;
        up(&device->sem_async);
        wait_for_completion(&device->async_completion);
        device->async_task = NULL;
    }
    /* Disable the card */
    sdio_claim_host(device->func);
    ret = sdio_disable_func(device->func);

    if (reset_sdio_on_unload) {
        /* reset the SDIO interface.  This is useful in automated testing where the card
         * does not need to be removed at the end of the test.  It is expected that the user will 
         * also unload/reload the host controller driver to force the bus driver to re-enumerate the slot */
        AR_DEBUG_PRINTF(ATH_DEBUG_WARN, ("AR6000: reseting SDIO card back to uninitialized state \n"));
        
        /* NOTE : sdio_f0_writeb() cannot be used here, that API only allows access
         *        to undefined registers in the range of: 0xF0-0xFF */
         
        ret = Func0_CMD52WriteByte(device->func->card, SDIO_CCCR_ABORT, (1 << 3)); 
        if (ret) {
             AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("AR6000: reset failed : %d \n",ret));    
        }
    }

    sdio_release_host(device->func);
    return ret;
}

static void hifDeviceRemoved(struct sdio_func *func)
{
    A_STATUS status = A_OK;
    HIF_DEVICE *device;
    AR_DEBUG_ASSERT(func != NULL);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: +hifDeviceRemoved\n"));
    device = getHifDevice(func);
    if (device->claimedContext != NULL) {
        status = osdrvCallbacks.deviceRemovedHandler(device->claimedContext, device);
    }

    if (device->is_suspend) {
        device->is_suspend = FALSE;
    } else {
        if (hifDisableFunc(device, func)!=0) {
            status = A_ERROR;
        }
    }
    CleanupHIFScatterResources(device);
     
    delHifDevice(device);
    AR_DEBUG_ASSERT(status == A_OK);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: -hifDeviceRemoved\n"));
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27) && defined(CONFIG_PM)
static int hifDeviceSuspend(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
    A_STATUS status = A_OK;
    HIF_DEVICE *device;   
    device = getHifDevice(func);
    if (device && device->claimedContext && osdrvCallbacks.deviceSuspendHandler) {
        status = osdrvCallbacks.deviceSuspendHandler(device->claimedContext);
    }
    if (status == A_OK) {
        hifDisableFunc(device, func);
        device->is_suspend = TRUE;
    } else if (status == A_EBUSY) {
        status = A_OK; /* assume that sdio host controller will take care the power of wifi chip */
    }
    return A_SUCCESS(status) ? 0 : status;
}

static int hifDeviceResume(struct device *dev)
{
    struct task_struct* pTask;
    const char *taskName;
    int (*taskFunc)(void *);
	struct sdio_func *func = dev_to_sdio_func(dev);
    A_STATUS ret = A_OK;
    HIF_DEVICE *device;   
    device = getHifDevice(func);

    if (device->is_suspend) {
       /* enable the SDIO function */
        sdio_claim_host(func);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
        /* give us some time to enable, in ms */
        func->enable_timeout = 100;
#endif
        ret = sdio_enable_func(func);
        if (ret) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), Unable to enable AR6K: 0x%X\n",
					  __FUNCTION__, ret));
            sdio_release_host(func);
            return ret;
    }
        ret = sdio_set_block_size(func, HIF_MBOX_BLOCK_SIZE);
        sdio_release_host(func);
        if (ret) {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), Unable to set block size 0x%x  AR6K: 0x%X\n",
					  __FUNCTION__, HIF_MBOX_BLOCK_SIZE, ret));
            return ret;
        }
        device->is_suspend = FALSE;
        /* create async I/O thread */
        if (!device->async_task) {
            device->async_shutdown = 0;
            device->async_task = kthread_create(async_task,
                                           (void *)device,
                                           "AR6K Async");
           if (IS_ERR(device->async_task)) {
               AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), to create async task\n", __FUNCTION__));
                return A_ERROR;
           }
           AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: start async task\n"));
           wake_up_process(device->async_task );    
        }
    }

    if (!device->claimedContext) {
        printk("WARNING!!! No claimedContext during resume wlan\n"); 
        taskFunc = startup_task;
        taskName = "AR6K startup";
    } else {
        taskFunc = resume_task;
        taskName = "AR6K resume";
    }
    /* create resume thread */
    pTask = kthread_create(taskFunc, (void *)device, taskName);
    if (IS_ERR(pTask)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERROR, ("AR6000: %s(), to create resume task\n", __FUNCTION__));
        return A_ERROR;
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: start resume task\n"));
    wake_up_process(pTask);
    return A_SUCCESS(ret) ? 0 : ret;
}
#endif /* CONFIG_PM */

static HIF_DEVICE *
addHifDevice(struct sdio_func *func)
{
    HIF_DEVICE *hifdevice;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: addHifDevice\n"));
    AR_DEBUG_ASSERT(func != NULL);
    hifdevice = (HIF_DEVICE *)kzalloc(sizeof(HIF_DEVICE), GFP_KERNEL);
    AR_DEBUG_ASSERT(hifdevice != NULL);
#if HIF_USE_DMA_BOUNCE_BUFFER
    hifdevice->dma_buffer = kmalloc(HIF_DMA_BUFFER_SIZE, GFP_KERNEL);
    AR_DEBUG_ASSERT(hifdevice->dma_buffer != NULL);
#endif
    hifdevice->func = func;
    sdio_set_drvdata(func, hifdevice);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: addHifDevice; 0x%p\n", hifdevice));
    return hifdevice;
}

static HIF_DEVICE *
getHifDevice(struct sdio_func *func)
{
    AR_DEBUG_ASSERT(func != NULL);
    return (HIF_DEVICE *)sdio_get_drvdata(func);
}

static void
delHifDevice(HIF_DEVICE * device)
{
    AR_DEBUG_ASSERT(device!= NULL);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRACE, ("AR6000: delHifDevice; 0x%p\n", device));
    if (device->dma_buffer != NULL) {
        kfree(device->dma_buffer);
    }
    kfree(device);
}

static void ResetAllCards(void)
{
}

void HIFClaimDevice(HIF_DEVICE  *device, void *context)
{
    device->claimedContext = context;
}

void HIFReleaseDevice(HIF_DEVICE  *device)
{
    device->claimedContext = NULL;
}

A_STATUS HIFAttachHTC(HIF_DEVICE *device, HTC_CALLBACKS *callbacks)
{
    if (device->htcCallbacks.context != NULL) {
            /* already in use! */
        return A_ERROR;
    }
    device->htcCallbacks = *callbacks;
    return A_OK;
}

void HIFDetachHTC(HIF_DEVICE *device)
{
    A_MEMZERO(&device->htcCallbacks,sizeof(device->htcCallbacks));
}

#define SDIO_SET_CMD52_ARG(arg,rw,func,raw,address,writedata) \
    (arg) = (((rw) & 1) << 31)           | \
            (((func) & 0x7) << 28)       | \
            (((raw) & 1) << 27)          | \
            (1 << 26)                    | \
            (((address) & 0x1FFFF) << 9) | \
            (1 << 8)                     | \
            ((writedata) & 0xFF)
            
#define SDIO_SET_CMD52_READ_ARG(arg,func,address) \
    SDIO_SET_CMD52_ARG(arg,0,(func),0,address,0x00)
#define SDIO_SET_CMD52_WRITE_ARG(arg,func,address,value) \
    SDIO_SET_CMD52_ARG(arg,1,(func),0,address,value)
    
static int Func0_CMD52WriteByte(struct mmc_card *card, unsigned int address, unsigned char byte)
{
    struct mmc_command ioCmd;
    unsigned long      arg;
    
    memset(&ioCmd,0,sizeof(ioCmd));
    SDIO_SET_CMD52_WRITE_ARG(arg,0,address,byte);
    ioCmd.opcode = SD_IO_RW_DIRECT;
    ioCmd.arg = arg;
    ioCmd.flags = MMC_RSP_R5 | MMC_CMD_AC;
    
    return mmc_wait_for_cmd(card->host, &ioCmd, 0);
}




