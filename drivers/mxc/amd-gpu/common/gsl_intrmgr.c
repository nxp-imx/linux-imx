/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
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

//////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#define GSL_INTRID_VALIDATE(id) (((id) < 0) || ((id) >= GSL_INTR_COUNT))


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

static const gsl_intrblock_reg_t * 
kgsl_intr_id2block(gsl_intrid_t id)
{
    const gsl_intrblock_reg_t  *block;
    int                        i;

    // interrupt id to hw block
    for (i = 0; i < GSL_INTR_BLOCK_COUNT; i++) 
    {
        block = &gsl_cfg_intrblock_reg[i];

        if (block->first_id <= id && id <= block->last_id) 
        {
            return (block);
        }
    }

    return (NULL);
}

//----------------------------------------------------------------------------

void
kgsl_intr_decode(gsl_device_t *device, gsl_intrblock_t block_id)
{
    const gsl_intrblock_reg_t  *block = &gsl_cfg_intrblock_reg[block_id];
    gsl_intrid_t               id;
    unsigned int               status;

    // read the block's interrupt status bits
    /* exclude CP block here to avoid hang in heavy loading with VPU+GPU */
    if ((block_id == GSL_INTR_BLOCK_YDX_CP) && (device->flags & GSL_FLAGS_STARTED)) {
	status = 0x80000000;
    } else {
	device->ftbl.device_regread(device, block->status_reg, &status);
    }

    // mask off any interrupts which are disabled
    status &= device->intr.enabled[block->id];

    // acknowledge the block's interrupts
    device->ftbl.device_regwrite(device, block->clear_reg, status);

    // loop through the block's masks, determine which interrupt bits are active, and call callback (or TODO queue DPC)
    for (id = block->first_id; id <= block->last_id; id++) 
    {
        if (status & gsl_cfg_intr_mask[id])
        {
            device->intr.handler[id].callback(id, device->intr.handler[id].cookie);
        }
    }
}

//----------------------------------------------------------------------------

KGSL_API void
kgsl_intr_isr(gsl_device_t *device)
{
    if (device->intr.flags & GSL_FLAGS_INITIALIZED) {
	kgsl_device_active(device);
	device->ftbl.intr_isr(device);
    }
}

//----------------------------------------------------------------------------

int kgsl_intr_init(gsl_device_t *device)
{
    if (device->ftbl.intr_isr == NULL)
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (device->intr.flags & GSL_FLAGS_INITIALIZED)
    {
        return (GSL_SUCCESS);
    }

    device->intr.device  = device;
    device->intr.flags  |= GSL_FLAGS_INITIALIZED;

    // os_interrupt_setcallback(YAMATO_INTR, kgsl_intr_isr);
    // os_interrupt_enable(YAMATO_INTR);

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int kgsl_intr_close(gsl_device_t *device)
{
    const gsl_intrblock_reg_t  *block;
    int                        i, id;

    if (device->intr.flags & GSL_FLAGS_INITIALIZED)
    {
        // check if there are any enabled interrupts lingering around
        for (i = 0; i < GSL_INTR_BLOCK_COUNT; i++) 
        {
            if (device->intr.enabled[i]) 
            {
                block = &gsl_cfg_intrblock_reg[i];

                // loop through the block's masks, disable interrupts which active
                for (id = block->first_id; id <= block->last_id; id++) 
                {
                    if (device->intr.enabled[i] & gsl_cfg_intr_mask[id])
                    {
                        kgsl_intr_disable(&device->intr, (gsl_intrid_t)id);
                    }
                }
            }
        }

        kos_memset(&device->intr, 0, sizeof(gsl_intr_t));
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int kgsl_intr_enable(gsl_intr_t *intr, gsl_intrid_t id)
{
    const gsl_intrblock_reg_t  *block;
    unsigned int               mask;
    unsigned int               enabled;

    if (GSL_INTRID_VALIDATE(id)) 
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (intr->handler[id].callback == NULL) 
    {
        return (GSL_FAILURE_NOTINITIALIZED);
    }

    block = kgsl_intr_id2block(id);
    if (block == NULL) 
    {
        return (GSL_FAILURE_SYSTEMERROR);
    }

    mask    = gsl_cfg_intr_mask[id];
    enabled = intr->enabled[block->id];

    if (mask && !(enabled & mask))
    {
        intr->evnt[id] = kos_event_create(0);

        enabled                 |= mask;
        intr->enabled[block->id] = enabled;
        intr->device->ftbl.device_regwrite(intr->device, block->mask_reg, enabled);
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int kgsl_intr_disable(gsl_intr_t *intr, gsl_intrid_t id)
{
    const gsl_intrblock_reg_t  *block;
    unsigned int               mask;
    unsigned int               enabled;

    if (GSL_INTRID_VALIDATE(id))
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (intr->handler[id].callback == NULL) 
    {
        return (GSL_FAILURE_NOTINITIALIZED);
    }

    block = kgsl_intr_id2block(id);
    if (block == NULL) 
    {
        return (GSL_FAILURE_SYSTEMERROR);
    }

    mask    = gsl_cfg_intr_mask[id];
    enabled = intr->enabled[block->id];

    if (enabled & mask) 
    {
        enabled                 &= ~mask;
        intr->enabled[block->id] = enabled;
        intr->device->ftbl.device_regwrite(intr->device, block->mask_reg, enabled);

        kos_event_signal(intr->evnt[id]); // wake up waiting threads before destroying the event
        kos_event_destroy(intr->evnt[id]);
        intr->evnt[id] = 0;
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_intr_attach(gsl_intr_t *intr, gsl_intrid_t id, gsl_intr_callback_t callback, void *cookie)
{
    if (GSL_INTRID_VALIDATE(id) || callback == NULL) 
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (intr->handler[id].callback != NULL) 
    {
        if (intr->handler[id].callback == callback && intr->handler[id].cookie == cookie) 
        {
            return (GSL_FAILURE_ALREADYINITIALIZED);
        } 
        else 
        {
            return (GSL_FAILURE_NOMOREAVAILABLE);
        }
    }

    intr->handler[id].callback = callback;
    intr->handler[id].cookie   = cookie;

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_intr_detach(gsl_intr_t *intr, gsl_intrid_t id)
{
    if (GSL_INTRID_VALIDATE(id)) 
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (intr->handler[id].callback == NULL) 
    {
        return (GSL_FAILURE_NOTINITIALIZED);
    }

    kgsl_intr_disable(intr, id);

    intr->handler[id].callback = NULL;
    intr->handler[id].cookie   = NULL;

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_intr_isenabled(gsl_intr_t *intr, gsl_intrid_t id)
{
    int                        status = GSL_FAILURE;
    const gsl_intrblock_reg_t  *block = kgsl_intr_id2block(id);

    if (block != NULL)
    {
        // check if interrupt is enabled
        if (intr->enabled[block->id] & gsl_cfg_intr_mask[id])
        {
            status = GSL_SUCCESS;
        }
    }

    return (status);
}
