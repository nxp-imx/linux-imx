/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#ifdef _LINUX
#include <linux/delay.h>
#include <linux/sched.h>
#endif

#ifdef GSL_BLD_YAMATO

#include "gsl_ringbuffer.h"
#include "gsl_drawctxt.h"

//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

static int
kgsl_yamato_gmeminit(gsl_device_t *device)
{
    rb_edram_info_u rb_edram_info = {0};
    unsigned int    gmem_size;
    unsigned int    edram_value = 0;

    // make sure edram range is aligned to size
    KOS_ASSERT((device->gmemspace.gpu_base  & (device->gmemspace.sizebytes - 1)) == 0);

    // get edram_size value equivalent
    gmem_size = (device->gmemspace.sizebytes >> 14);
    while (gmem_size >>= 1)
    {
       edram_value++;
    }

    rb_edram_info.f.edram_size         = edram_value;
    rb_edram_info.f.edram_mapping_mode = 0;                                     // EDRAM_MAP_UPPER
    rb_edram_info.f.edram_range        = (device->gmemspace.gpu_base >> 14);    // must be aligned to size

    device->ftbl.device_regwrite(device, mmRB_EDRAM_INFO, (unsigned int)rb_edram_info.val);

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

static int
kgsl_yamato_gmemclose(gsl_device_t *device)
{
    device->ftbl.device_regwrite(device, mmRB_EDRAM_INFO, 0x00000000);

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

void
kgsl_yamato_rbbmintrcallback(gsl_intrid_t id, void *cookie)
{
    gsl_device_t  *device = (gsl_device_t *) cookie;

    switch(id)
    {
        // error condition interrupt
        case GSL_INTR_YDX_RBBM_READ_ERROR:
	    printk(KERN_ERR "GPU: Z430 RBBM Read Error\n");
	    schedule_work(&device->irq_err_work);
            break;

        // non-error condition interrupt
        case GSL_INTR_YDX_RBBM_DISPLAY_UPDATE:
        case GSL_INTR_YDX_RBBM_GUI_IDLE:

            kos_event_signal(device->intr.evnt[id]);
            break;

        default:

            break;
    }
}

//----------------------------------------------------------------------------

void
kgsl_yamato_cpintrcallback(gsl_intrid_t id, void *cookie)
{
    gsl_device_t  *device = (gsl_device_t *) cookie;

    switch(id)
    {
        case GSL_INTR_YDX_CP_RING_BUFFER:
#ifndef _LINUX		
              kos_event_signal(device->timestamp_event);
#else
			  wake_up_interruptible_all(&(device->timestamp_waitq));
#endif
            break;
        default:
            break;
    }
}
//----------------------------------------------------------------------------

void
kgsl_yamato_sqintrcallback(gsl_intrid_t id, void *cookie)
{
    (void) cookie;      // unreferenced formal parameter
	/*gsl_device_t  *device = (gsl_device_t *) cookie;*/

    switch(id)
    {
        // error condition interrupt
        case GSL_INTR_YDX_SQ_PS_WATCHDOG:
        case GSL_INTR_YDX_SQ_VS_WATCHDOG:

			// todo: take appropriate action

			break;

		default:

			break;
    }
}

//----------------------------------------------------------------------------

#ifdef _DEBUG

static int
kgsl_yamato_bist(gsl_device_t *device)
{
    int           status = GSL_FAILURE;
    unsigned int  link[2];

    if (!(device->flags & GSL_FLAGS_STARTED))
    {
        return (GSL_FAILURE);
    }

    status = kgsl_ringbuffer_bist(&device->ringbuffer);
    if (status != GSL_SUCCESS)
    {
        return (status);
    }

    // interrupt bist
    link[0] = pm4_type3_packet(PM4_INTERRUPT, 1);
    link[1] = CP_INT_CNTL__RB_INT_MASK;
	kgsl_ringbuffer_issuecmds(device, 1, &link[0], 2, GSL_CALLER_PROCESSID_GET());

    status = kgsl_mmu_bist(&device->mmu);
    if (status != GSL_SUCCESS)
    {
        return (status);
    }

    return (status);
}
#endif

//----------------------------------------------------------------------------

int
kgsl_yamato_isr(gsl_device_t *device)
{
    unsigned int            status;
#ifdef _DEBUG
    mh_mmu_page_fault_u     page_fault        = {0};
    mh_axi_error_u          axi_error         = {0};
    mh_clnt_axi_id_reuse_u  clnt_axi_id_reuse = {0};
    rbbm_read_error_u       read_error        = {0};
#endif // DEBUG

    // determine if yamato is interrupting, and if so, which block
    device->ftbl.device_regread(device, mmMASTER_INT_SIGNAL, &status);

    if (status & MASTER_INT_SIGNAL__MH_INT_STAT)
    {
#ifdef _DEBUG
        // obtain mh error information
        device->ftbl.device_regread(device, mmMH_MMU_PAGE_FAULT, (unsigned int *)&page_fault);
        device->ftbl.device_regread(device, mmMH_AXI_ERROR, (unsigned int *)&axi_error);
        device->ftbl.device_regread(device, mmMH_CLNT_AXI_ID_REUSE, (unsigned int *)&clnt_axi_id_reuse);
#endif // DEBUG

        kgsl_intr_decode(device, GSL_INTR_BLOCK_YDX_MH);
    }

    if (status & MASTER_INT_SIGNAL__CP_INT_STAT)
    {
        kgsl_intr_decode(device, GSL_INTR_BLOCK_YDX_CP);
    }

    if (status & MASTER_INT_SIGNAL__RBBM_INT_STAT)
    {
#ifdef _DEBUG
        // obtain rbbm error information
        device->ftbl.device_regread(device, mmRBBM_READ_ERROR, (unsigned int *)&read_error);
#endif // DEBUG

        kgsl_intr_decode(device, GSL_INTR_BLOCK_YDX_RBBM);
    }

    if (status & MASTER_INT_SIGNAL__SQ_INT_STAT)
    {
        kgsl_intr_decode(device, GSL_INTR_BLOCK_YDX_SQ);
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_tlbinvalidate(gsl_device_t *device, unsigned int reg_invalidate, unsigned int pid)
{
	unsigned int		 link[2];
    mh_mmu_invalidate_u  mh_mmu_invalidate = {0};

    mh_mmu_invalidate.f.invalidate_all = 1;
    mh_mmu_invalidate.f.invalidate_tc  = 1;

	// if possible, invalidate via command stream, otherwise via direct register writes
	if (device->flags & GSL_FLAGS_STARTED)
	{
		link[0] = pm4_type0_packet(reg_invalidate, 1);
		link[1] = mh_mmu_invalidate.val;

		kgsl_ringbuffer_issuecmds(device, 1, &link[0], 2, pid);
	}
	else
	{

        device->ftbl.device_regwrite(device, reg_invalidate, mh_mmu_invalidate.val);
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_setpagetable(gsl_device_t *device, unsigned int reg_ptbase, gpuaddr_t ptbase, unsigned int pid)
{
	unsigned int  link[25];

    // if there is an active draw context, set via command stream,
	if (device->flags & GSL_FLAGS_STARTED)
    {
        // wait for graphics pipe to be idle
        link[0] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
        link[1] = 0x00000000;

        // set page table base
        link[2] = pm4_type0_packet(reg_ptbase, 1);
		link[3] = ptbase;

        // HW workaround: to resolve MMU page fault interrupts caused by the VGT. It prevents 
		// the CP PFP from filling the VGT DMA request fifo too early, thereby ensuring that 
		// the VGT will not fetch vertex/bin data until after the page table base register 
		// has been updated.
		//
		// Two null DRAW_INDX_BIN packets are inserted right after the page table base update, 
		// followed by a wait for idle. The null packets will fill up the VGT DMA request 
		// fifo and prevent any further vertex/bin updates from occurring until the wait 
		// has finished.
		link[4]  = pm4_type3_packet(PM4_SET_CONSTANT, 2);
		link[5]  = (0x4 << 16) | (mmPA_SU_SC_MODE_CNTL - 0x2000);
		link[6]  = 0;          // disable faceness generation
		link[7]  = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
		link[8]  = device->mmu.dummyspace.gpuaddr;
		link[9]  = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
		link[10] = 0;          // viz query info
		link[11] = 0x0003C004; // draw indicator
		link[12] = 0;          // bin base
		link[13] = 3;          // bin size
		link[14] = device->mmu.dummyspace.gpuaddr; // dma base
		link[15] = 6;          // dma size
		link[16] = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
		link[17] = 0;          // viz query info
		link[18] = 0x0003C004; // draw indicator
		link[19] = 0;          // bin base
		link[20] = 3;          // bin size
		link[21] = device->mmu.dummyspace.gpuaddr; // dma base
		link[22] = 6;          // dma size
		link[23] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
		link[24] = 0x00000000;

		kgsl_ringbuffer_issuecmds(device, 1, &link[0], 25, pid);
	}
	else
	{
		device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);
		device->ftbl.device_regwrite(device, reg_ptbase, ptbase);
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

static void kgsl_yamato_irqerr(struct work_struct *work)
{
	gsl_device_t *device = &gsl_driver.device[GSL_DEVICE_YAMATO-1];
	device->ftbl.device_destroy(device);
}

int
kgsl_yamato_init(gsl_device_t *device)
{
    int  status = GSL_FAILURE;

    device->flags |= GSL_FLAGS_INITIALIZED;

    kgsl_hal_setpowerstate(device->id, GSL_PWRFLAGS_POWER_ON, 100);

    //We need to make sure all blocks are powered up and clocked before
    //issuing a soft reset.  The overrides will be turned off (set to 0)
    //later in kgsl_yamato_start.
    device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE1, 0xfffffffe);
    device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE2, 0xffffffff);

    // soft reset
    device->ftbl.device_regwrite(device, mmRBBM_SOFT_RESET, 0xFFFFFFFF);
    kos_sleep(50);
    device->ftbl.device_regwrite(device, mmRBBM_SOFT_RESET, 0x00000000);

    // RBBM control
    device->ftbl.device_regwrite(device, mmRBBM_CNTL, 0x00004442);

    // setup MH arbiter
    device->ftbl.device_regwrite(device, mmMH_ARBITER_CONFIG, *(unsigned int *) &gsl_cfg_yamato_mharb);

    // SQ_*_PROGRAM
    device->ftbl.device_regwrite(device, mmSQ_VS_PROGRAM, 0x00000000);
    device->ftbl.device_regwrite(device, mmSQ_PS_PROGRAM, 0x00000000);

    // init interrupt
    status = kgsl_intr_init(device);
    if (status != GSL_SUCCESS)
    {
        device->ftbl.device_stop(device);
        return (status);
    }

    // init mmu
    status = kgsl_mmu_init(device);
    if (status != GSL_SUCCESS)
    {
        device->ftbl.device_stop(device);
        return (status);
    }

    /* handle error condition */
    INIT_WORK(&device->irq_err_work, kgsl_yamato_irqerr);

    return(status);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_close(gsl_device_t *device)
{
    if (device->refcnt == 0)
    {
        // shutdown mmu
        kgsl_mmu_close(device);

        // shutdown interrupt
        kgsl_intr_close(device);

        kgsl_hal_setpowerstate(device->id, GSL_PWRFLAGS_POWER_OFF, 0);

        device->flags &= ~GSL_FLAGS_INITIALIZED;
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_destroy(gsl_device_t *device)
{
    int           i;
    unsigned int  pid;

#ifdef _DEBUG
    // for now, signal catastrophic failure in a brute force way
    KOS_ASSERT(0);
#endif // _DEBUG

    // todo: - hard reset core?

    kgsl_drawctxt_destroyall(device);

    for (i = 0; i < GSL_CALLER_PROCESS_MAX; i++)
    {
        pid = device->callerprocess[i];
        if (pid)
        {
            device->ftbl.device_stop(device);
            kgsl_driver_destroy(pid);

            // todo: terminate client process?
        }
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_start(gsl_device_t *device, gsl_flags_t flags)
{
    int           status = GSL_FAILURE;
    unsigned int  pm1, pm2;

    KGSL_DEBUG(GSL_DBGFLAGS_PM4, KGSL_DEBUG_DUMPFBSTART(device));

    (void) flags;      // unreferenced formal parameter

    kgsl_hal_setpowerstate(device->id, GSL_PWRFLAGS_CLK_ON, 100);

    // default power management override when running in safe mode
    pm1 = (device->flags & GSL_FLAGS_SAFEMODE) ? 0xFFFFFFFE : 0x00000000;
    pm2 = (device->flags & GSL_FLAGS_SAFEMODE) ? 0x000000FF : 0x00000000;
    device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE1, pm1);
    device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE2, pm2);

    // enable rbbm interrupts
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_RBBM_READ_ERROR, kgsl_yamato_rbbmintrcallback, (void *) device);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_RBBM_DISPLAY_UPDATE, kgsl_yamato_rbbmintrcallback, (void *) device);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_RBBM_GUI_IDLE, kgsl_yamato_rbbmintrcallback, (void *) device);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_RBBM_READ_ERROR);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_RBBM_DISPLAY_UPDATE);
#if defined GSL_RB_TIMESTAMP_INTERUPT	
	kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_RING_BUFFER, kgsl_yamato_cpintrcallback, (void *) device);    
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_RING_BUFFER);
#endif

  //kgsl_intr_enable(&device->intr, GSL_INTR_YDX_RBBM_GUI_IDLE);

    // enable sq interrupts
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_SQ_PS_WATCHDOG, kgsl_yamato_sqintrcallback, (void *) device);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_SQ_VS_WATCHDOG, kgsl_yamato_sqintrcallback, (void *) device);
  //kgsl_intr_enable(&device->intr, GSL_INTR_YDX_SQ_PS_WATCHDOG);
  //kgsl_intr_enable(&device->intr, GSL_INTR_YDX_SQ_VS_WATCHDOG);

    // init gmem
    kgsl_yamato_gmeminit(device);

    // init ring buffer
    status = kgsl_ringbuffer_init(device);
    if (status != GSL_SUCCESS)
    {
        device->ftbl.device_stop(device);
        return (status);
    }

    // init draw context
    status = kgsl_drawctxt_init(device);
    if (status != GSL_SUCCESS)
    {
        device->ftbl.device_stop(device);
        return (status);
    }

    device->flags |= GSL_FLAGS_STARTED;

    KGSL_DEBUG(GSL_DBGFLAGS_BIST, kgsl_yamato_bist(device));

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_stop(gsl_device_t *device)
{
	// HW WORKAROUND: Ringbuffer hangs during next start if it is stopped without any
	// commands ever being submitted. To avoid this, submit a dummy wait packet.
	unsigned int cmds[2];
    cmds[0] = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
    cmds[0] = 0;
	kgsl_ringbuffer_issuecmds(device, 0, cmds, 2, GSL_CALLER_PROCESSID_GET());

    // disable rbbm interrupts
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_RBBM_READ_ERROR);
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_RBBM_DISPLAY_UPDATE);
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_RBBM_GUI_IDLE);
#if defined GSL_RB_TIMESTAMP_INTERUPT	
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_CP_RING_BUFFER);
#endif

    // disable sq interrupts
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_SQ_PS_WATCHDOG);
    kgsl_intr_detach(&device->intr, GSL_INTR_YDX_SQ_VS_WATCHDOG);

    kgsl_drawctxt_close(device);

    // shutdown ringbuffer
    kgsl_ringbuffer_close(&device->ringbuffer);

    // shutdown gmem
    kgsl_yamato_gmemclose(device);

    if(device->refcnt == 0)
    {
        kgsl_hal_setpowerstate(device->id, GSL_PWRFLAGS_CLK_OFF, 0);
    }

    device->flags &= ~GSL_FLAGS_STARTED;

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_getproperty(gsl_device_t *device, gsl_property_type_t type, void *value, unsigned int sizebytes)
{
    int  status = GSL_FAILURE;

#ifndef _DEBUG
    (void) sizebytes;       // unreferenced formal parameter
#endif

    if (type == GSL_PROP_DEVICE_INFO)
    {
        gsl_devinfo_t  *devinfo = (gsl_devinfo_t *) value;

        KOS_ASSERT(sizebytes == sizeof(gsl_devinfo_t));

        devinfo->device_id         = device->id;
        devinfo->chip_id           = (gsl_chipid_t)device->chip_id;
        devinfo->mmu_enabled       = kgsl_mmu_isenabled(&device->mmu);
        devinfo->gmem_hostbaseaddr = device->gmemspace.mmio_virt_base;
        devinfo->gmem_gpubaseaddr  = device->gmemspace.gpu_base;
        devinfo->gmem_sizebytes    = device->gmemspace.sizebytes;
	devinfo->high_precision    = 0;

        status = GSL_SUCCESS;
    }

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_setproperty(gsl_device_t *device, gsl_property_type_t type, void *value, unsigned int sizebytes)
{
    int  status = GSL_FAILURE;

#ifndef _DEBUG
    (void) sizebytes;           // unreferenced formal parameter
#endif

    if (type == GSL_PROP_DEVICE_POWER)
    {
        gsl_powerprop_t  *power = (gsl_powerprop_t *) value;

        KOS_ASSERT(sizebytes == sizeof(gsl_powerprop_t));

        if (!(device->flags & GSL_FLAGS_SAFEMODE))
        {
            if (power->flags & GSL_PWRFLAGS_OVERRIDE_ON)
            {
                device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE1, 0xfffffffe);
                device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE2, 0xffffffff);
            }
            else if (power->flags & GSL_PWRFLAGS_OVERRIDE_OFF)
            {
                device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE1, 0x00000000);
                device->ftbl.device_regwrite(device, mmRBBM_PM_OVERRIDE2, 0x00000000);
            }
            else
            {
                kgsl_hal_setpowerstate(device->id, power->flags, power->value);
            }
        }

        status = GSL_SUCCESS;
    }
    else if (type == GSL_PROP_DEVICE_DMI)
    {
        gsl_dmiprop_t  *dmi = (gsl_dmiprop_t *) value;

        KOS_ASSERT(sizebytes == sizeof(gsl_dmiprop_t));

        //
        //  In order to enable DMI, it must not already be enabled.
        //
        switch (dmi->flags)
        {
            case GSL_DMIFLAGS_ENABLE_SINGLE:
            case GSL_DMIFLAGS_ENABLE_DOUBLE:
                if (!gsl_driver.dmi_state)
                {
                    gsl_driver.dmi_state = OS_TRUE;
                    gsl_driver.dmi_mode  = dmi->flags;
                    gsl_driver.dmi_frame = -1;
                    status = GSL_SUCCESS;
                }
                break;
            case GSL_DMIFLAGS_DISABLE:
                //
                //  To disable, we must be enabled.
                //
                if (gsl_driver.dmi_state)
                {
                    gsl_driver.dmi_state = OS_FALSE;
                    gsl_driver.dmi_mode  = -1;
                    gsl_driver.dmi_frame = -2;
                    status = GSL_SUCCESS;
                }
                break;
            case GSL_DMIFLAGS_NEXT_BUFFER:
                //
                //  Going to the next buffer is dependent upon what mod we are in with respect to single, double, or triple buffering.
                //  DMI must also be enabled.
                //
                if (gsl_driver.dmi_state)
                {
                    unsigned int    cmdbuf[10];
                    unsigned int    *cmds = &cmdbuf[0];
                    int             size;

                    if (gsl_driver.dmi_frame == -1)
                    {
                        size = 8;

                        *cmds++ =  pm4_type0_packet(mmRBBM_DSPLY, 1);
                        switch (gsl_driver.dmi_mode)
                        {
                            case GSL_DMIFLAGS_ENABLE_SINGLE:
                                gsl_driver.dmi_max_frame = 1;
                                *cmds++ = 0x041000410;
                                break;
                            case GSL_DMIFLAGS_ENABLE_DOUBLE:
                                gsl_driver.dmi_max_frame = 2;
                                *cmds++ = 0x041000510;
                                break;
                            case GSL_DMIFLAGS_ENABLE_TRIPLE:
                                gsl_driver.dmi_max_frame = 3;
                                *cmds++ = 0x041000610;
                                break;
                        }
                    }
                    else
                    {
                        size = 6;
                    }


                    //
                    //  Wait for 3D core to be idle and wait for vsync
                    //
                    *cmds++ =  pm4_type0_packet(mmWAIT_UNTIL, 1);
                    *cmds++ = 0x00008000;           //  3d idle
                    // *cmds++ = 0x00008008;         //  3d idle & vsync

                    //
                    //  Update the render latest register.
                    //
                    *cmds++ =  pm4_type0_packet(mmRBBM_RENDER_LATEST, 1);
                    switch (gsl_driver.dmi_frame)
                    {
                        case 0:
                            //
                            //  Render frame 0
                            //
                            *cmds++ = 0;
                            //
                            //  Wait for our max frame # indicator to be de-asserted
                            //
                            *cmds++ =  pm4_type0_packet(mmWAIT_UNTIL, 1);
                            *cmds++ = 0x00000008 << gsl_driver.dmi_max_frame;
                            gsl_driver.dmi_frame = 1;
                            break;
                        case -1:
                        case 1:
                            //
                            //  Render frame 1
                            //
                            *cmds++ = 1;
                            *cmds++ =  pm4_type0_packet(mmWAIT_UNTIL, 1);
                            *cmds++ = 0x00000010;   //  Wait for frame 0 to be deasserted
                            gsl_driver.dmi_frame = 2;
                            break;
                        case 2:
                            //
                            //  Render frame 2
                            //
                            *cmds++ = 2;
                            *cmds++ =  pm4_type0_packet(mmWAIT_UNTIL, 1);
                            *cmds++ = 0x00000020;   //  Wait for frame 1 to be deasserted
                            gsl_driver.dmi_frame = 0;
                            break;
                    }
                    
                    // issue the commands
                    kgsl_ringbuffer_issuecmds(device, 1, &cmdbuf[0], size, GSL_CALLER_PROCESSID_GET());

                    gsl_driver.dmi_frame %= gsl_driver.dmi_max_frame;
                    status = GSL_SUCCESS;
                }
                break;
            default:
                status = GSL_FAILURE;
                break;
        }
    }

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_idle(gsl_device_t *device, unsigned int timeout)
{
    int               status  = GSL_FAILURE;
    gsl_ringbuffer_t  *rb     = &device->ringbuffer;
    rbbm_status_u     rbbm_status;

    (void) timeout;      // unreferenced formal parameter

    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_REGPOLL, device->id, mmRBBM_STATUS, 0x80000000, "kgsl_yamato_idle"));

    GSL_RB_MUTEX_LOCK();

    // first, wait until the CP has consumed all the commands in the ring buffer
    if (rb->flags & GSL_FLAGS_STARTED)
    {
        do
        {
            GSL_RB_GET_READPTR(rb, &rb->rptr);

        } while (rb->rptr != rb->wptr);
    }

    // now, wait for the GPU to finish its operations
    for ( ; ; )
    {
        device->ftbl.device_regread(device, mmRBBM_STATUS, (unsigned int *)&rbbm_status);

        if (!(rbbm_status.val & 0x80000000))
        {
            status = GSL_SUCCESS;
            break;
        }

    }
	
    GSL_RB_MUTEX_UNLOCK();

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_regread(gsl_device_t *device, unsigned int offsetwords, unsigned int *value)
{
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX,
            {
                if (!(gsl_driver.flags_debug & GSL_DBGFLAGS_DUMPX_WITHOUT_IFH))
                {
                    if(offsetwords == mmCP_RB_RPTR || offsetwords == mmCP_RB_WPTR)
                    {
                        *value = device->ringbuffer.wptr;
                        return (GSL_SUCCESS);
                    }
                }
            });

    GSL_HAL_REG_READ(device->id, (unsigned int) device->regspace.mmio_virt_base, offsetwords, value);

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_regwrite(gsl_device_t *device, unsigned int offsetwords, unsigned int value)
{
    KGSL_DEBUG(GSL_DBGFLAGS_PM4, KGSL_DEBUG_DUMPREGWRITE(offsetwords, value));

    GSL_HAL_REG_WRITE(device->id, (unsigned int) device->regspace.mmio_virt_base, offsetwords, value);

    // idle device when running in safe mode
    if (device->flags & GSL_FLAGS_SAFEMODE)
    {
        device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_waitirq(gsl_device_t *device, gsl_intrid_t intr_id, unsigned int *count, unsigned int timeout)
{
    int  status = GSL_FAILURE_NOTSUPPORTED;

    if (intr_id == GSL_INTR_YDX_CP_IB1_INT || intr_id == GSL_INTR_YDX_CP_IB2_INT ||
        intr_id == GSL_INTR_YDX_CP_SW_INT  || intr_id == GSL_INTR_YDX_RBBM_DISPLAY_UPDATE)
    {
        if (kgsl_intr_isenabled(&device->intr, intr_id) == GSL_SUCCESS)
        {
            // wait until intr completion event is received
            if (kos_event_wait(device->intr.evnt[intr_id], timeout) == OS_SUCCESS)
            {
                *count = 1;
                status = GSL_SUCCESS;
            }
            else
            {
                status = GSL_FAILURE_TIMEOUT;
            }
        }
    }

    return (status);
}

int kgsl_yamato_check_timestamp(gsl_deviceid_t device_id, gsl_timestamp_t timestamp)
{
	int i;
	/* Reason to use a wait loop:
	 * When bus is busy, for example vpu is working too, the timestamp is
	 * possiblly not yet refreshed to memory by yamato. For most cases, it
	 * will hit on first loop cycle. So it don't effect performance.
	 */
	for (i = 0; i < 10; i++) {
		if (kgsl_cmdstream_check_timestamp(device_id, timestamp))
			return 1;
		udelay(10);
	}
	return 0;
}

int
kgsl_yamato_waittimestamp(gsl_device_t *device, gsl_timestamp_t timestamp, unsigned int timeout)
{
#if defined GSL_RB_TIMESTAMP_INTERUPT
#ifndef _LINUX
	return kos_event_wait( device->timestamp_event, timeout );
#else
	int status = wait_event_interruptible_timeout(device->timestamp_waitq,
							kgsl_yamato_check_timestamp(device->id, timestamp),
							msecs_to_jiffies(timeout));
	if (status > 0)
		return GSL_SUCCESS;
	else
		return GSL_FAILURE;
#endif
#else
	return (GSL_SUCCESS);
#endif
}
//----------------------------------------------------------------------------

int
kgsl_yamato_runpending(gsl_device_t *device)
{
    (void) device;

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_yamato_getfunctable(gsl_functable_t *ftbl)
{
    ftbl->device_init           = kgsl_yamato_init;
    ftbl->device_close          = kgsl_yamato_close;
    ftbl->device_destroy        = kgsl_yamato_destroy;
    ftbl->device_start          = kgsl_yamato_start;
    ftbl->device_stop           = kgsl_yamato_stop;
    ftbl->device_getproperty    = kgsl_yamato_getproperty;
    ftbl->device_setproperty    = kgsl_yamato_setproperty;
    ftbl->device_idle           = kgsl_yamato_idle;
	ftbl->device_waittimestamp  = kgsl_yamato_waittimestamp;
    ftbl->device_regread        = kgsl_yamato_regread;
    ftbl->device_regwrite       = kgsl_yamato_regwrite;
    ftbl->device_waitirq        = kgsl_yamato_waitirq;
    ftbl->device_runpending     = kgsl_yamato_runpending;
    ftbl->intr_isr              = kgsl_yamato_isr;
    ftbl->mmu_tlbinvalidate     = kgsl_yamato_tlbinvalidate;
    ftbl->mmu_setpagetable      = kgsl_yamato_setpagetable;
    ftbl->cmdstream_issueibcmds = kgsl_ringbuffer_issueibcmds;
    ftbl->context_create        = kgsl_drawctxt_create;
    ftbl->context_destroy       = kgsl_drawctxt_destroy;

    return (GSL_SUCCESS);
}

#endif

