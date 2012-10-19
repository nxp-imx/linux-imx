/* Copyright (c) 2002,2007-2009, Code Aurora Forum. All rights reserved.
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

#ifdef GSL_BLD_YAMATO

//////////////////////////////////////////////////////////////////////////////
// ucode
//////////////////////////////////////////////////////////////////////////////
#define uint32 unsigned int

#include "pm4_microcode.inl"
#include "pfp_microcode_nrt.inl"

#undef  uint32


//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////
#define GSL_RB_NOP_SIZEDWORDS               2               // default is 2
#define GSL_RB_PROTECTED_MODE_CONTROL       0x00000000      // protected mode error checking below register address 0x800
                                                            // note: if CP_INTERRUPT packet is used then checking needs
                                                            // to change to below register address 0x7C8


//////////////////////////////////////////////////////////////////////////////
//  ringbuffer size log2 quadwords equivalent
//////////////////////////////////////////////////////////////////////////////
OSINLINE unsigned int
gsl_ringbuffer_sizelog2quadwords(unsigned int sizedwords)
{
    unsigned int sizelog2quadwords = 0;
    int i = sizedwords >> 1;
    while (i >>= 1)
    {
       sizelog2quadwords++;
    }
    return (sizelog2quadwords);
}


//////////////////////////////////////////////////////////////////////////////
// private prototypes
//////////////////////////////////////////////////////////////////////////////
#ifdef _DEBUG
static void     kgsl_ringbuffer_debug(gsl_ringbuffer_t *rb, gsl_rb_debug_t *rb_debug);
#endif


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

void
kgsl_cp_intrcallback(gsl_intrid_t id, void *cookie)
{
    gsl_ringbuffer_t  *rb = (gsl_ringbuffer_t *) cookie;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> void kgsl_cp_intrcallback(gsl_intrid_t id=%I, void *cookie=0x%08x)\n", id, cookie );

    switch(id)
    {
        // error condition interrupt
        case GSL_INTR_YDX_CP_T0_PACKET_IN_IB:
        case GSL_INTR_YDX_CP_OPCODE_ERROR:
        case GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR:
        case GSL_INTR_YDX_CP_RESERVED_BIT_ERROR:
        case GSL_INTR_YDX_CP_IB_ERROR:
	    printk(KERN_ERR "GPU: CP Error\n");
	    schedule_work(&rb->device->irq_err_work);
            break;

        // non-error condition interrupt
        case GSL_INTR_YDX_CP_SW_INT:
        case GSL_INTR_YDX_CP_IB2_INT:
        case GSL_INTR_YDX_CP_IB1_INT:
        case GSL_INTR_YDX_CP_RING_BUFFER:

            // signal intr completion event
            kos_event_signal(rb->device->intr.evnt[id]);
            break;

        default:

            break;
    }

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_cp_intrcallback.\n" );
}

//----------------------------------------------------------------------------

void
kgsl_ringbuffer_watchdog()
{
    gsl_ringbuffer_t  *rb = &(gsl_driver.device[GSL_DEVICE_YAMATO-1]).ringbuffer;       // device_id is 1 based

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> void kgsl_ringbuffer_watchdog()\n" );

    if (rb->flags & GSL_FLAGS_STARTED)
    {
        GSL_RB_MUTEX_LOCK();

        GSL_RB_GET_READPTR(rb, &rb->rptr);

        // ringbuffer is currently not empty
        if (rb->rptr != rb->wptr)
        {
            // and a rptr sample was taken during interval n-1
            if (rb->watchdog.flags & GSL_FLAGS_ACTIVE)
            {
                // and the rptr did not advance between interval n-1 and n
                if (rb->rptr == rb->watchdog.rptr_sample)
                {
                    // then the core has hung
                    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_FATAL,
                                     "ERROR: Watchdog detected core hung.\n" );

                    rb->device->ftbl.device_destroy(rb->device);
                    return;
                }
            }

            // save rptr sample for interval n
            rb->watchdog.flags       |= GSL_FLAGS_ACTIVE;
            rb->watchdog.rptr_sample  = rb->rptr;
        }
        else
        {
            // clear rptr sample for interval n
            rb->watchdog.flags &= ~GSL_FLAGS_ACTIVE;
        }

        GSL_RB_MUTEX_UNLOCK();
    }

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_watchdog.\n" );
}

//----------------------------------------------------------------------------

#ifdef _DEBUG

OSINLINE void
kgsl_ringbuffer_checkregister(unsigned int reg, int pmodecheck)
{
	if (pmodecheck)
	{
		// check for register protection mode violation
		if (reg <= (GSL_RB_PROTECTED_MODE_CONTROL & 0x3FFF))
		{
			kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Register protection mode violation.\n" );
			KOS_ASSERT(0);
		}
	}

	// range check register offset 
	if (reg > (gsl_driver.device[GSL_DEVICE_YAMATO-1].regspace.sizebytes >> 2))
	{
		kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Register out of range.\n" );
		KOS_ASSERT(0);
	}
}

//----------------------------------------------------------------------------

void
kgsl_ringbuffer_checkpm4type0(unsigned int header, unsigned int** cmds, int pmodeoff)
{
    pm4_type0     pm4header = *((pm4_type0*) &header);
    unsigned int  reg;

    if (pm4header.one_reg_wr)
    {
        reg = pm4header.base_index;
    }
    else
    {
        reg = pm4header.base_index + pm4header.count;
    }

	kgsl_ringbuffer_checkregister(reg, !pmodeoff);

    *cmds += pm4header.count + 1;
}

//----------------------------------------------------------------------------

void
kgsl_ringbuffer_checkpm4type3(unsigned int header, unsigned int** cmds, int indirection, int pmodeoff)
{
    pm4_type3     pm4header = *((pm4_type3*) &header);
    unsigned int  *ordinal2 = *cmds;
    unsigned int  *ibcmds, *end;
    unsigned int  reg, length;

	// check indirect buffer level
	if (indirection > 2)
	{
		kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Only two levels of indirection supported.\n" );
		KOS_ASSERT(0);
	}

    switch(pm4header.it_opcode)
    {
    case PM4_INDIRECT_BUFFER:
    case PM4_INDIRECT_BUFFER_PFD:

        // determine ib host base and end address
        ibcmds = (unsigned int*) kgsl_sharedmem_convertaddr(*ordinal2, 0);
        end    = ibcmds + *(ordinal2 + 1);

        // walk through the ib
        while(ibcmds < end)
        {
            unsigned int tmpheader = *(ibcmds++);

            switch(tmpheader & PM4_PKT_MASK)
            {
            case PM4_TYPE0_PKT:
                kgsl_ringbuffer_checkpm4type0(tmpheader, &ibcmds, pmodeoff);
                break;

            case PM4_TYPE1_PKT:
            case PM4_TYPE2_PKT:
                break;

            case PM4_TYPE3_PKT:
                kgsl_ringbuffer_checkpm4type3(tmpheader, &ibcmds, (indirection + 1), pmodeoff);
                break;
            }
        }
        break;

    case PM4_ME_INIT:

        if(indirection != 0)
        {
			kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: ME INIT packet cannot reside in an ib.\n" );
            KOS_ASSERT(0);
        }
        break;

    case PM4_REG_RMW:

        reg = (*ordinal2) & 0x1FFF;

		kgsl_ringbuffer_checkregister(reg, !pmodeoff);

        break;

    case PM4_SET_CONSTANT:

        if((((*ordinal2) >> 16) & 0xFF) == 0x4)         // incremental register update
        {
            reg    = 0x2000 + ((*ordinal2) & 0x3FF);    // gfx decode space address starts at 0x2000
            length = pm4header.count - 1;

			kgsl_ringbuffer_checkregister(reg + length, 0);
        }
        break;

    case PM4_LOAD_CONSTANT_CONTEXT:

        if(((*(ordinal2 + 1) >> 16) & 0xFF) == 0x4)      // incremental register update
        {
            reg    = 0x2000 + (*(ordinal2 + 1) & 0x3FF); // gfx decode space address starts at 0x2000
            length = *(ordinal2 + 2);

			kgsl_ringbuffer_checkregister(reg + length, 0);
        }
        break;

    case PM4_COND_WRITE:

        if(((*ordinal2) & 0x00000100) == 0x0)           // write to register
        {
            reg = *(ordinal2 + 4) & 0x3FFF;

			kgsl_ringbuffer_checkregister(reg, !pmodeoff);
        }
        break;
    }

    *cmds += pm4header.count + 1;
}

//----------------------------------------------------------------------------

void
kgsl_ringbuffer_checkpm4(unsigned int* cmds, unsigned int sizedwords, int pmodeoff)
{
    unsigned int *ringcmds = cmds;
    unsigned int *end      = cmds + sizedwords;

    while(ringcmds < end)
    {
        unsigned int header = *(ringcmds++);

        switch(header & PM4_PKT_MASK)
        {
            case PM4_TYPE0_PKT:
                kgsl_ringbuffer_checkpm4type0(header, &ringcmds, pmodeoff);
                break;

            case PM4_TYPE1_PKT:
            case PM4_TYPE2_PKT:
                break;

            case PM4_TYPE3_PKT:
                kgsl_ringbuffer_checkpm4type3(header, &ringcmds, 0, pmodeoff);
                break;
        }
    }
}

#endif // _DEBUG

//----------------------------------------------------------------------------

static void
kgsl_ringbuffer_submit(gsl_ringbuffer_t *rb)
{
    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> static void kgsl_ringbuffer_submit(gsl_ringbuffer_t *rb=0x%08x)\n", rb );

    KOS_ASSERT(rb->wptr != 0);

    kgsl_device_active(rb->device);
    
    GSL_RB_UPDATE_WPTR_POLLING(rb);

    // send the wptr to the hw
    rb->device->ftbl.device_regwrite(rb->device, mmCP_RB_WPTR, rb->wptr);

    rb->flags |= GSL_FLAGS_ACTIVE;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_submit.\n" );
}

//----------------------------------------------------------------------------

static int
kgsl_ringbuffer_waitspace(gsl_ringbuffer_t *rb, unsigned int numcmds, int wptr_ahead)
{
    int           nopcount;
    unsigned int  freecmds;
    unsigned int  *cmds;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> static int kgsl_ringbuffer_waitspace(gsl_ringbuffer_t *rb=0x%08x, unsigned int numcmds=%d, int wptr_ahead=%d)\n",
                    rb, numcmds, wptr_ahead );


    // if wptr ahead, fill the remaining with NOPs
    if (wptr_ahead)
    {
        nopcount = rb->sizedwords - rb->wptr - 1;   // -1 for header

        cmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
        GSL_RB_WRITE(cmds, pm4_nop_packet(nopcount));
        rb->wptr++;

        kgsl_ringbuffer_submit(rb);

        rb->wptr = 0;

        GSL_RB_STATS(rb->stats.wraps++);
    }

    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_RBWAIT, GSL_DEVICE_YAMATO, rb->wptr, numcmds, "kgsl_ringbuffer_waitspace"));

    // wait for space in ringbuffer
    for( ; ; )
    {
        GSL_RB_GET_READPTR(rb, &rb->rptr);

        freecmds = rb->rptr - rb->wptr;

        if ((freecmds == 0) || (freecmds > numcmds))
        {
            break;
        }

    }

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_waitspace. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

static unsigned int *
kgsl_ringbuffer_addcmds(gsl_ringbuffer_t *rb, unsigned int numcmds)
{
    unsigned int  *ptr;
    int           status = GSL_SUCCESS;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> static unsigned int* kgsl_ringbuffer_addcmds(gsl_ringbuffer_t *rb=0x%08x, unsigned int numcmds=%d)\n",
                    rb, numcmds );

    KOS_ASSERT(numcmds < rb->sizedwords);

    // update host copy of read pointer when running in safe mode
    if (rb->device->flags & GSL_FLAGS_SAFEMODE)
    {
        GSL_RB_GET_READPTR(rb, &rb->rptr);
    }

    // check for available space
    if (rb->wptr >= rb->rptr)
    {
        // wptr ahead or equal to rptr
        if ((rb->wptr + numcmds) > (rb->sizedwords - GSL_RB_NOP_SIZEDWORDS))   // reserve dwords for nop packet
        {
            status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
        }
    }
    else
    {
        // wptr behind rptr
        if ((rb->wptr + numcmds) >= rb->rptr)
        {
            status  = kgsl_ringbuffer_waitspace(rb, numcmds, 0);
        }

        // check for remaining space
        if ((rb->wptr + numcmds) > (rb->sizedwords - GSL_RB_NOP_SIZEDWORDS))   // reserve dwords for nop packet
        {
            status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
        }
    }

    ptr       = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
    rb->wptr += numcmds;

    if (status == GSL_SUCCESS)
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_waitspace. Return value 0x%08x\n", ptr );
        return (ptr);
    }
    else
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_waitspace. Return value 0x%08x\n", NULL );
        return (NULL);
    }
}

//----------------------------------------------------------------------------
int
kgsl_ringbuffer_start(gsl_ringbuffer_t *rb)
{
    int           status;
    cp_rb_cntl_u  cp_rb_cntl;
    int           i;
    unsigned int  *cmds;
    gsl_device_t  *device = rb->device;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> static int kgsl_ringbuffer_start(gsl_ringbuffer_t *rb=0x%08x)\n", rb );

    if (rb->flags & GSL_FLAGS_STARTED)
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_start. Return value %B\n", GSL_SUCCESS );
        return (GSL_SUCCESS);
    }

    // clear memptrs values
    kgsl_sharedmem_set0(&rb->memptrs_desc, 0, 0, sizeof(gsl_rbmemptrs_t));

    // clear ringbuffer
    kgsl_sharedmem_set0(&rb->buffer_desc, 0, 0x12341234, (rb->sizedwords << 2));

    // setup WPTR polling address
    device->ftbl.device_regwrite(device, mmCP_RB_WPTR_BASE, (rb->memptrs_desc.gpuaddr + GSL_RB_MEMPTRS_WPTRPOLL_OFFSET));

    // setup WPTR delay
    device->ftbl.device_regwrite(device, mmCP_RB_WPTR_DELAY, 0/*0x70000010*/);

    // setup RB_CNTL
    device->ftbl.device_regread(device, mmCP_RB_CNTL, (unsigned int *)&cp_rb_cntl);

    cp_rb_cntl.f.rb_bufsz       = gsl_ringbuffer_sizelog2quadwords(rb->sizedwords); // size of ringbuffer
    cp_rb_cntl.f.rb_blksz       = rb->blksizequadwords;                             // quadwords to read before updating mem RPTR
    cp_rb_cntl.f.rb_poll_en     = GSL_RB_CNTL_POLL_EN;                              // WPTR polling
    cp_rb_cntl.f.rb_no_update   = GSL_RB_CNTL_NO_UPDATE;                            // mem RPTR writebacks

    device->ftbl.device_regwrite(device, mmCP_RB_CNTL, cp_rb_cntl.val);

    // setup RB_BASE
    device->ftbl.device_regwrite(device, mmCP_RB_BASE, rb->buffer_desc.gpuaddr);

    // setup RPTR_ADDR
    device->ftbl.device_regwrite(device, mmCP_RB_RPTR_ADDR, rb->memptrs_desc.gpuaddr + GSL_RB_MEMPTRS_RPTR_OFFSET);

    // explicitly clear all cp interrupts when running in safe mode
    if (rb->device->flags & GSL_FLAGS_SAFEMODE)
    {
        device->ftbl.device_regwrite(device, mmCP_INT_ACK, 0xFFFFFFFF);
    }

    // setup scratch/timestamp addr
    device->ftbl.device_regwrite(device, mmSCRATCH_ADDR, device->memstore.gpuaddr + GSL_DEVICE_MEMSTORE_OFFSET(soptimestamp));

    // setup scratch/timestamp mask
    device->ftbl.device_regwrite(device, mmSCRATCH_UMSK, GSL_RB_MEMPTRS_SCRATCH_MASK);

    // load the CP ucode
    device->ftbl.device_regwrite(device, mmCP_DEBUG, 0x02000000);
    device->ftbl.device_regwrite(device, mmCP_ME_RAM_WADDR, 0);

    for (i = 0; i < PM4_MICROCODE_SIZE; i++ )
    {
        device->ftbl.device_regwrite(device, mmCP_ME_RAM_DATA, aPM4_Microcode[i][0]);
        device->ftbl.device_regwrite(device, mmCP_ME_RAM_DATA, aPM4_Microcode[i][1]);
        device->ftbl.device_regwrite(device, mmCP_ME_RAM_DATA, aPM4_Microcode[i][2]);
    }

    // load the prefetch parser ucode
    device->ftbl.device_regwrite(device, mmCP_PFP_UCODE_ADDR, 0);

    for ( i = 0; i < PFP_MICROCODE_SIZE_NRT; i++ )
    {
        device->ftbl.device_regwrite(device, mmCP_PFP_UCODE_DATA, aPFP_Microcode_nrt[i]);
    }

    // queue thresholds ???
    device->ftbl.device_regwrite(device, mmCP_QUEUE_THRESHOLDS, 0x000C0804);

    // reset pointers
    rb->rptr = 0;
    rb->wptr = 0;

    // init timestamp
    rb->timestamp    = 0;
    GSL_RB_INIT_TIMESTAMP(rb);

    // clear ME_HALT to start micro engine
    device->ftbl.device_regwrite(device, mmCP_ME_CNTL, 0);

    // ME_INIT
    cmds  = kgsl_ringbuffer_addcmds(rb, 19);

    GSL_RB_WRITE(cmds, PM4_HDR_ME_INIT);
    GSL_RB_WRITE(cmds, 0x000003ff);                         // All fields present (bits 9:0)
    GSL_RB_WRITE(cmds, 0x00000000);                         // Disable/Enable Real-Time Stream processing (present but ignored)
    GSL_RB_WRITE(cmds, 0x00000000);                         // Enable (2D to 3D) and (3D to 2D) implicit synchronization (present but ignored)
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmRB_SURFACE_INFO));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmPA_SC_WINDOW_OFFSET));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmVGT_MAX_VTX_INDX));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmSQ_PROGRAM_CNTL));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmRB_DEPTHCONTROL));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmPA_SU_POINT_SIZE));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmPA_SC_LINE_CNTL));
    GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(mmPA_SU_POLY_OFFSET_FRONT_SCALE));
    GSL_RB_WRITE(cmds, 0x80000180);                         // Vertex and Pixel Shader Start Addresses in instructions (3 DWORDS per instruction)
    GSL_RB_WRITE(cmds, 0x00000001);                         // Maximum Contexts
    GSL_RB_WRITE(cmds, 0x00000000);                         // Write Confirm Interval and The CP will wait the wait_interval * 16 clocks between polling
    GSL_RB_WRITE(cmds, 0x00000000);                         // NQ and External Memory Swap
    GSL_RB_WRITE(cmds, GSL_RB_PROTECTED_MODE_CONTROL);      // Protected mode error checking
    GSL_RB_WRITE(cmds, 0x00000000);                         // Disable header dumping and Header dump address
    GSL_RB_WRITE(cmds, 0x00000000);                         // Header dump size

    KGSL_DEBUG(GSL_DBGFLAGS_PM4CHECK, kgsl_ringbuffer_checkpm4((unsigned int *)rb->buffer_desc.hostptr, 19, 1));
    KGSL_DEBUG(GSL_DBGFLAGS_PM4, KGSL_DEBUG_DUMPPM4((unsigned int *)rb->buffer_desc.hostptr, 19));

    kgsl_ringbuffer_submit(rb);

    // idle device to validate ME INIT
    status = device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);

    if (status == GSL_SUCCESS)
    {
        rb->flags |= GSL_FLAGS_STARTED;
    }

    // enable cp interrupts
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_SW_INT, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_T0_PACKET_IN_IB, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_OPCODE_ERROR, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_RESERVED_BIT_ERROR, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_IB_ERROR, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_IB2_INT, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_IB1_INT, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_attach(&device->intr, GSL_INTR_YDX_CP_RING_BUFFER, kgsl_cp_intrcallback, (void *) rb);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_SW_INT);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_T0_PACKET_IN_IB);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_OPCODE_ERROR);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_RESERVED_BIT_ERROR);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_IB_ERROR);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_IB2_INT);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_IB1_INT);
    kgsl_intr_enable(&device->intr, GSL_INTR_YDX_CP_RING_BUFFER);

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_start. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_ringbuffer_stop(gsl_ringbuffer_t *rb)
{
    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> static int kgsl_ringbuffer_stop(gsl_ringbuffer_t *rb=0x%08x)\n", rb );

    if (rb->flags & GSL_FLAGS_STARTED)
    {
        // disable cp interrupts
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_SW_INT);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_T0_PACKET_IN_IB);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_OPCODE_ERROR);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_PROTECTED_MODE_ERROR);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_RESERVED_BIT_ERROR);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_IB_ERROR);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_IB2_INT);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_IB1_INT);
        kgsl_intr_detach(&rb->device->intr, GSL_INTR_YDX_CP_RING_BUFFER);

        // ME_HALT
        rb->device->ftbl.device_regwrite(rb->device, mmCP_ME_CNTL, 0x10000000);

        rb->flags &= ~GSL_FLAGS_STARTED;
    }

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_stop. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_ringbuffer_init(gsl_device_t *device)
{
    int               status;
    gsl_flags_t       flags;
    gsl_ringbuffer_t  *rb = &device->ringbuffer;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_ringbuffer_init(gsl_device_t *device=0x%08x)\n", device );

    rb->device           = device;
    rb->sizedwords       = (2 << gsl_cfg_rb_sizelog2quadwords);
    rb->blksizequadwords = gsl_cfg_rb_blksizequadwords;

    GSL_RB_MUTEX_CREATE();

    // allocate memory for ringbuffer, needs to be double octword aligned
    // align on page from contiguous physical memory
    flags = (GSL_MEMFLAGS_ALIGNPAGE | GSL_MEMFLAGS_CONPHYS | GSL_MEMFLAGS_STRICTREQUEST);
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, flags = (GSL_MEMFLAGS_ALIGNPAGE | GSL_MEMFLAGS_STRICTREQUEST)); /* set MMU table for ringbuffer */

    status = kgsl_sharedmem_alloc0(device->id, flags, (rb->sizedwords << 2), &rb->buffer_desc);

    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_RINGBUF_SET, (unsigned int)rb->buffer_desc.gpuaddr, (unsigned int)rb->buffer_desc.hostptr, 0, "kgsl_ringbuffer_init"));

    if (status != GSL_SUCCESS)
    {
        kgsl_ringbuffer_close(rb);
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_init. Return value %B\n", status );
        return (status);
    }

    // allocate memory for polling and timestamps
    flags = (GSL_MEMFLAGS_ALIGN32 | GSL_MEMFLAGS_CONPHYS);
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, flags = GSL_MEMFLAGS_ALIGN32);

    status = kgsl_sharedmem_alloc0(device->id, flags, sizeof(gsl_rbmemptrs_t), &rb->memptrs_desc);

    if (status != GSL_SUCCESS)
    {
        kgsl_ringbuffer_close(rb);
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_init. Return value %B\n", status );
        return (status);
    }

    // overlay structure on memptrs memory
    rb->memptrs = (gsl_rbmemptrs_t *)rb->memptrs_desc.hostptr;

    rb->flags |= GSL_FLAGS_INITIALIZED;

	// validate command stream data when running in safe mode
	if (device->flags & GSL_FLAGS_SAFEMODE)
	{
		gsl_driver.flags_debug |= GSL_DBGFLAGS_PM4CHECK;
	}

    // start ringbuffer
    status = kgsl_ringbuffer_start(rb);

    if (status != GSL_SUCCESS)
    {
        kgsl_ringbuffer_close(rb);
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_init. Return value %B\n", status );
        return (status);
    }

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_init. Return value %B\n", GSL_SUCCESS );
    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_ringbuffer_close(gsl_ringbuffer_t *rb)
{
    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_ringbuffer_close(gsl_ringbuffer_t *rb=0x%08x)\n", rb );

    GSL_RB_MUTEX_LOCK();

    // stop ringbuffer
    kgsl_ringbuffer_stop(rb);

    // free buffer
    if (rb->buffer_desc.hostptr)
    {
        kgsl_sharedmem_free0(&rb->buffer_desc, GSL_CALLER_PROCESSID_GET());
    }

    // free memory pointers
    if (rb->memptrs_desc.hostptr)
    {
        kgsl_sharedmem_free0(&rb->memptrs_desc, GSL_CALLER_PROCESSID_GET());
    }

    rb->flags &= ~GSL_FLAGS_INITIALIZED;

    GSL_RB_MUTEX_UNLOCK();

    GSL_RB_MUTEX_FREE();

    kos_memset(rb, 0, sizeof(gsl_ringbuffer_t));

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_close. Return value %B\n", GSL_SUCCESS );
    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

gsl_timestamp_t
kgsl_ringbuffer_issuecmds(gsl_device_t *device, int pmodeoff, unsigned int *cmds, int sizedwords, unsigned int pid)
{
    gsl_ringbuffer_t  *rb = &device->ringbuffer;
    unsigned int      pmodesizedwords;
    unsigned int      *ringcmds;
    unsigned int      timestamp;

    pmodeoff = 0;

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> gsl_timestamp_t kgsl_ringbuffer_issuecmds(gsl_device_t *device=0x%08x, int pmodeoff=%d, unsigned int *cmds=0x%08x, int sizedwords=%d, unsigned int pid=0x%08x)\n",
                     device, pmodeoff, cmds, sizedwords, pid );

	if (!(device->ringbuffer.flags & GSL_FLAGS_STARTED))
	{
		return (0);
	}

    // set mmu pagetable
	kgsl_mmu_setpagetable(device, pid);

    KGSL_DEBUG(GSL_DBGFLAGS_PM4CHECK, kgsl_ringbuffer_checkpm4(cmds, sizedwords, pmodeoff));
    KGSL_DEBUG(GSL_DBGFLAGS_PM4, KGSL_DEBUG_DUMPPM4(cmds, sizedwords));

    // reserve space to temporarily turn off protected mode error checking if needed
    pmodesizedwords = pmodeoff ? 8 : 0;

#if defined GSL_RB_TIMESTAMP_INTERUPT
    pmodesizedwords += 2;
#endif        
    // allocate space in ringbuffer
    ringcmds = kgsl_ringbuffer_addcmds(rb, pmodesizedwords + sizedwords + 6);

    if (pmodeoff)
    {
        // disable protected mode error checking
        *ringcmds++ = pm4_type3_packet(PM4_ME_INIT, 2);
        *ringcmds++ = 0x00000080;
        *ringcmds++ = 0x00000000;
    }

    // copy the cmds to the ringbuffer
    kos_memcpy(ringcmds, cmds, (sizedwords << 2));

    ringcmds += sizedwords;

    if (pmodeoff)
    {
        *ringcmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
        *ringcmds++ = 0;

        // re-enable protected mode error checking
        *ringcmds++ = pm4_type3_packet(PM4_ME_INIT, 2);
        *ringcmds++ = 0x00000080;
        *ringcmds++ = GSL_RB_PROTECTED_MODE_CONTROL;
    }

    // increment timestamp
    rb->timestamp++;
    timestamp = rb->timestamp;

    // start-of-pipeline and end-of-pipeline timestamps
    *ringcmds++ = pm4_type0_packet(mmCP_TIMESTAMP, 1);
    *ringcmds++ = rb->timestamp;
    *ringcmds++ = pm4_type3_packet(PM4_EVENT_WRITE, 3);
    *ringcmds++ = CACHE_FLUSH_TS;
    *ringcmds++ = device->memstore.gpuaddr + GSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp);
    *ringcmds++ = rb->timestamp;

#if defined GSL_RB_TIMESTAMP_INTERUPT    
    *ringcmds++ = pm4_type3_packet(PM4_INTERRUPT, 1);
    *ringcmds++ = 0x80000000;
#endif
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, KGSL_DEBUG_DUMPX(BB_DUMP_MEMWRITE, (unsigned int)((char*)ringcmds - ((pmodesizedwords + sizedwords + 6) << 2)), (unsigned int)((char*)ringcmds - ((pmodesizedwords + sizedwords + 6) << 2)), (pmodesizedwords + sizedwords + 6) << 2, "kgsl_ringbuffer_issuecmds"));

    // issue the commands
    kgsl_ringbuffer_submit(rb);

    // stats
    GSL_RB_STATS(rb->stats.wordstotal += sizedwords);
    GSL_RB_STATS(rb->stats.issues++);

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_issuecmds. Return value %d\n", timestamp );

    // return timestamp of issued commands
    return (timestamp);
}

//----------------------------------------------------------------------------
int
kgsl_ringbuffer_issueibcmds(gsl_device_t *device, int drawctxt_index, gpuaddr_t ibaddr, int sizedwords, gsl_timestamp_t *timestamp, gsl_flags_t flags)
{
    unsigned int  link[3];
    int dumpx_swap;
    (void)dumpx_swap; // used only when BB_DUMPX is defined

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> gsl_timestamp_t kgsl_ringbuffer_issueibcmds(gsl_device_t device=%0x%08x, int drawctxt_index=%d, gpuaddr_t ibaddr=0x%08x, int sizedwords=%d, gsl_timestamp_t *timestamp=0x%08x)\n",
                     device, drawctxt_index, ibaddr, sizedwords, timestamp );

    if (!(device->ringbuffer.flags & GSL_FLAGS_STARTED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_issueibcmds. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    KOS_ASSERT(ibaddr);
    KOS_ASSERT(sizedwords);

    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX, dumpx_swap = kgsl_dumpx_parse_ibs(ibaddr, sizedwords));

	GSL_RB_MUTEX_LOCK();

	// context switch if needed
	kgsl_drawctxt_switch(device, &device->drawctxt[drawctxt_index], flags);

    link[0] = PM4_HDR_INDIRECT_BUFFER_PFD;
    link[1] = ibaddr;
    link[2] = sizedwords;

	*timestamp = kgsl_ringbuffer_issuecmds(device, 0, &link[0], 3, GSL_CALLER_PROCESSID_GET());

	GSL_RB_MUTEX_UNLOCK();

    // idle device when running in safe mode
    if (device->flags & GSL_FLAGS_SAFEMODE)
    {
        device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);
    }
    else
    {
        KGSL_DEBUG(GSL_DBGFLAGS_DUMPX,
        {
            // insert wait for idle after every IB1
            // this is conservative but works reliably and is ok even for performance simulations
            device->ftbl.device_idle(device, GSL_TIMEOUT_DEFAULT);
        });
    }
    KGSL_DEBUG(GSL_DBGFLAGS_DUMPX,
    {
        if(dumpx_swap)
        {
            KGSL_DEBUG_DUMPX( BB_DUMP_EXPORT_CBUF, 0, 0, 0, "resolve");
            KGSL_DEBUG_DUMPX( BB_DUMP_FLUSH,0,0,0," ");
        }
    });

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_issueibcmds. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

#ifdef _DEBUG
static void
kgsl_ringbuffer_debug(gsl_ringbuffer_t *rb, gsl_rb_debug_t *rb_debug)
{
    kos_memset(rb_debug, 0, sizeof(gsl_rb_debug_t));

    rb_debug->pm4_ucode_rel = PM4_MICROCODE_VERSION;
    rb_debug->pfp_ucode_rel = PFP_MICROCODE_VERSION;

    rb->device->ftbl.device_regread(rb->device, mmCP_RB_BASE,      (unsigned int *)&rb_debug->cp_rb_base);
    rb->device->ftbl.device_regread(rb->device, mmCP_RB_CNTL,      (unsigned int *)&rb_debug->cp_rb_cntl);
    rb->device->ftbl.device_regread(rb->device, mmCP_RB_RPTR_ADDR, (unsigned int *)&rb_debug->cp_rb_rptr_addr);
    rb->device->ftbl.device_regread(rb->device, mmCP_RB_RPTR,      (unsigned int *)&rb_debug->cp_rb_rptr);
    rb->device->ftbl.device_regread(rb->device, mmCP_RB_WPTR,      (unsigned int *)&rb_debug->cp_rb_wptr);
    rb->device->ftbl.device_regread(rb->device, mmCP_RB_WPTR_BASE, (unsigned int *)&rb_debug->cp_rb_wptr_base);
    rb->device->ftbl.device_regread(rb->device, mmSCRATCH_UMSK,    (unsigned int *)&rb_debug->scratch_umsk);
    rb->device->ftbl.device_regread(rb->device, mmSCRATCH_ADDR,    (unsigned int *)&rb_debug->scratch_addr);
    rb->device->ftbl.device_regread(rb->device, mmCP_ME_CNTL,      (unsigned int *)&rb_debug->cp_me_cntl);
    rb->device->ftbl.device_regread(rb->device, mmCP_ME_STATUS,    (unsigned int *)&rb_debug->cp_me_status);
    rb->device->ftbl.device_regread(rb->device, mmCP_DEBUG,        (unsigned int *)&rb_debug->cp_debug);
    rb->device->ftbl.device_regread(rb->device, mmCP_STAT,         (unsigned int *)&rb_debug->cp_stat);
    rb->device->ftbl.device_regread(rb->device, mmRBBM_STATUS,     (unsigned int *)&rb_debug->rbbm_status);
    rb_debug->sop_timestamp = kgsl_cmdstream_readtimestamp(rb->device->id, GSL_TIMESTAMP_CONSUMED);
    rb_debug->eop_timestamp = kgsl_cmdstream_readtimestamp(rb->device->id, GSL_TIMESTAMP_RETIRED);
}
#endif


//----------------------------------------------------------------------------

int
kgsl_ringbuffer_querystats(gsl_ringbuffer_t *rb, gsl_rbstats_t *stats)
{
#ifdef GSL_STATS_RINGBUFFER
    KOS_ASSERT(stats);

    if (!(rb->flags & GSL_FLAGS_STARTED))
    {
        return (GSL_FAILURE);
    }

    kos_memcpy(stats, &rb->stats, sizeof(gsl_rbstats_t));

    return (GSL_SUCCESS);
#else
    // unreferenced formal parameters
    (void) rb;
    (void) stats;

    return (GSL_FAILURE_NOTSUPPORTED);
#endif // GSL_STATS_RINGBUFFER
}

//----------------------------------------------------------------------------

int
kgsl_ringbuffer_bist(gsl_ringbuffer_t *rb)
{
    unsigned int    *cmds;
    unsigned int    temp, k, j;
    int             status;
    int             i;
#ifdef _DEBUG
    gsl_rb_debug_t  rb_debug;
#endif

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_ringbuffer_bist(gsl_ringbuffer_t *rb=0x%08x)\n", rb );

    if (!(rb->flags & GSL_FLAGS_STARTED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // simple nop submit
    cmds = kgsl_ringbuffer_addcmds(rb, 2);
    if (!cmds)
    {
#ifdef _DEBUG
        kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    GSL_RB_WRITE(cmds, pm4_nop_packet(1));
    GSL_RB_WRITE(cmds, 0xDEADBEEF);

    kgsl_ringbuffer_submit(rb);

    status = rb->device->ftbl.device_idle(rb->device, GSL_TIMEOUT_DEFAULT);

    if (status != GSL_SUCCESS)
    {
#ifdef _DEBUG
        kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", status );
        return (status);
    }

    // simple scratch submit
    cmds = kgsl_ringbuffer_addcmds(rb, 2);
    if (!cmds)
    {
#ifdef _DEBUG
        kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    GSL_RB_WRITE(cmds, pm4_type0_packet(mmSCRATCH_REG7, 1));
    GSL_RB_WRITE(cmds, 0xFEEDF00D);

    kgsl_ringbuffer_submit(rb);

    status = rb->device->ftbl.device_idle(rb->device, GSL_TIMEOUT_DEFAULT);

    if (status != GSL_SUCCESS)
    {
#ifdef _DEBUG
        kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", status );
        return (status);
    }

    rb->device->ftbl.device_regread(rb->device, mmSCRATCH_REG7, &temp);

    if (temp != 0xFEEDF00D)
    {
#ifdef _DEBUG
        kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
        kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // simple wraps
    for (i = 0; i < 256; i+=2)
    {
        j    = ((rb->sizedwords >> 2) - 256) + i;

        cmds = kgsl_ringbuffer_addcmds(rb, j);
        if (!cmds)
        {
#ifdef _DEBUG
            kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
            kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
            return (GSL_FAILURE);
        }

        k = 0;

        while (k < j)
        {
            k+=2;
            GSL_RB_WRITE(cmds, pm4_type0_packet(mmSCRATCH_REG7, 1));
            GSL_RB_WRITE(cmds, k);
        }

        kgsl_ringbuffer_submit(rb);

        status = rb->device->ftbl.device_idle(rb->device, GSL_TIMEOUT_DEFAULT);

        if (status != GSL_SUCCESS)
        {
#ifdef _DEBUG
            kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
            kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", status );
            return (status);
        }

        rb->device->ftbl.device_regread(rb->device, mmSCRATCH_REG7, &temp);

        if (temp != k)
        {
#ifdef _DEBUG
            kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
            kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
            return (GSL_FAILURE);
        }
    }

    // max size submits, TODO do this at least with regreads
    for (i = 0; i < 256; i++)
    {
        cmds = kgsl_ringbuffer_addcmds(rb, (rb->sizedwords >> 2));
        if (!cmds)
        {
#ifdef _DEBUG
            kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
            kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_FAILURE );
            return (GSL_FAILURE);
        }

        GSL_RB_WRITE(cmds, pm4_nop_packet((rb->sizedwords >> 2) - 1));

        kgsl_ringbuffer_submit(rb);

        status = rb->device->ftbl.device_idle(rb->device, GSL_TIMEOUT_DEFAULT);

        if (status != GSL_SUCCESS)
        {
#ifdef _DEBUG
            kgsl_ringbuffer_debug(rb, &rb_debug);
#endif
            kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", status );
            return (status);
        }
    }

    // submit load with randomness

#ifdef GSL_RB_USE_MEM_TIMESTAMP
    // scratch memptr validate
#endif // GSL_RB_USE_MEM_TIMESTAMP

#ifdef GSL_RB_USE_MEM_RPTR
    // rptr memptr validate
#endif // GSL_RB_USE_MEM_RPTR

#ifdef  GSL_RB_USE_WPTR_POLLING
    // wptr memptr validate
#endif // GSL_RB_USE_WPTR_POLLING

    kgsl_log_write( KGSL_LOG_GROUP_COMMAND | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_ringbuffer_bist. Return value %d\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

#endif

