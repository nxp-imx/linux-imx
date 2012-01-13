/****************************************************************************
*
*    Copyright (C) 2005 - 2011 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/




#include "gc_hal_kernel_linux.h"
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>

#define _GC_OBJ_ZONE    gcvZONE_DEVICE

#ifdef FLAREON
    static struct dove_gpio_irq_handler gc500_handle;
#endif

#define gcmIS_CORE_PRESENT(Device, Core) (Device->irqLines[Core] > 0)

/******************************************************************************\
*************************** Memory Allocation Wrappers *************************
\******************************************************************************/

static gceSTATUS
_AllocateMemory(
    IN gckGALDEVICE Device,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical,
    OUT gctPHYS_ADDR *Physical,
    OUT gctUINT32 *PhysAddr
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=0x%x Bytes=%lu", Device, Bytes);

    gcmkVERIFY_ARGUMENT(Device != NULL);
    gcmkVERIFY_ARGUMENT(Logical != NULL);
    gcmkVERIFY_ARGUMENT(Physical != NULL);
    gcmkVERIFY_ARGUMENT(PhysAddr != NULL);

    gcmkONERROR(gckOS_AllocateContiguous(
        Device->os, gcvFALSE, &Bytes, Physical, Logical
        ));

    *PhysAddr = ((PLINUX_MDL)*Physical)->dmaHandle - Device->baseAddress;

    /* Success. */
    gcmkFOOTER_ARG(
        "*Logical=0x%x *Physical=0x%x *PhysAddr=0x%08x",
        *Logical, *Physical, *PhysAddr
        );

    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_FreeMemory(
    IN gckGALDEVICE Device,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical)
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=0x%x Logical=0x%x Physical=0x%x",
                   Device, Logical, Physical);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    status = gckOS_FreeContiguous(
        Device->os, Physical, Logical,
        ((PLINUX_MDL) Physical)->numPages * PAGE_SIZE
        );

    gcmkFOOTER();
    return status;
}



/******************************************************************************\
******************************* Interrupt Handler ******************************
\******************************************************************************/
static irqreturn_t isrRoutine(int irq, void *ctxt)
{
    gceSTATUS status;
    gckGALDEVICE device;

    device = (gckGALDEVICE) ctxt;

    /* Call kernel interrupt notification. */
    status = gckKERNEL_Notify(device->kernels[gcvCORE_MAJOR], gcvNOTIFY_INTERRUPT, gcvTRUE);

    if (gcmIS_SUCCESS(status))
    {
        device->dataReadys[gcvCORE_MAJOR] = gcvTRUE;

        up(&device->semas[gcvCORE_MAJOR]);

        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static int threadRoutine(void *ctxt)
{
    gckGALDEVICE device = (gckGALDEVICE) ctxt;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                   "Starting isr Thread with extension=%p",
                   device);

    for (;;)
    {
        static int down;

        down = down_interruptible(&device->semas[gcvCORE_MAJOR]);
        device->dataReadys[gcvCORE_MAJOR] = gcvFALSE;

        if (device->killThread == gcvTRUE)
        {
            /* The daemon exits. */
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        gckKERNEL_Notify(device->kernels[gcvCORE_MAJOR], gcvNOTIFY_INTERRUPT, gcvFALSE);
    }
}

static irqreturn_t isrRoutine2D(int irq, void *ctxt)
{
    gceSTATUS status;
    gckGALDEVICE device;

    device = (gckGALDEVICE) ctxt;

    /* Call kernel interrupt notification. */
    status = gckKERNEL_Notify(device->kernels[gcvCORE_2D], gcvNOTIFY_INTERRUPT, gcvTRUE);

    if (gcmIS_SUCCESS(status))
    {
        device->dataReadys[gcvCORE_2D] = gcvTRUE;

        up(&device->semas[gcvCORE_2D]);

        return IRQ_HANDLED;
    }

    return IRQ_NONE;
}

static int threadRoutine2D(void *ctxt)
{
    gckGALDEVICE device = (gckGALDEVICE) ctxt;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                   "Starting isr Thread with extension=%p",
                   device);

    for (;;)
    {
        static int down;

        down = down_interruptible(&device->semas[gcvCORE_2D]);
        device->dataReadys[gcvCORE_2D] = gcvFALSE;

        if (device->killThread == gcvTRUE)
        {
            /* The daemon exits. */
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        gckKERNEL_Notify(device->kernels[gcvCORE_2D], gcvNOTIFY_INTERRUPT, gcvFALSE);
    }
}

static irqreturn_t isrRoutineVG(int irq, void *ctxt)
{
#if gcdENABLE_VG
    gceSTATUS status;
    gckGALDEVICE device;

    device = (gckGALDEVICE) ctxt;

	/* Serve the interrupt. */
	status = gckVGINTERRUPT_Enque(device->kernels[gcvCORE_VG]->vg->interrupt);

	/* Determine the return value. */
	return (status == gcvSTATUS_NOT_OUR_INTERRUPT)
		? IRQ_RETVAL(0)
		: IRQ_RETVAL(1);
#else
    return IRQ_NONE;
#endif
}

static int threadRoutineVG(void *ctxt)
{
    gckGALDEVICE device = (gckGALDEVICE) ctxt;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                   "Starting isr Thread with extension=%p",
                   device);

    for (;;)
    {
        static int down;

        down = down_interruptible(&device->semas[gcvCORE_VG]);
        device->dataReadys[gcvCORE_VG] = gcvFALSE;

        if (device->killThread == gcvTRUE)
        {
            /* The daemon exits. */
            while (!kthread_should_stop())
            {
                gckOS_Delay(device->os, 1);
            }

            return 0;
        }

        gckKERNEL_Notify(device->kernels[gcvCORE_VG], gcvNOTIFY_INTERRUPT, gcvFALSE);
    }
}

#if gcdPOWEROFF_TIMEOUT
/*
** PM Thread Routine
**/
static int threadRoutinePM(void *ctxt)
{
    gckGALDEVICE device = (gckGALDEVICE) ctxt;
    gckHARDWARE hardware = device->kernels[gcvCORE_MAJOR]->hardware;
    gceCHIPPOWERSTATE state;

    for(;;)
    {
        /* wait for idle */
        gcmkVERIFY_OK(
            gckOS_WaitSignal(device->os, hardware->powerOffSignal, gcvINFINITE));

        /* We try to power off every 200 ms, until GPU is not idle */
        do
        {
            if (device->killThread == gcvTRUE)
            {
                /* The daemon exits. */
                while (!kthread_should_stop())
                {
                    gckOS_Delay(device->os, 1);
                }
                return 0;
            }

            gcmkVERIFY_OK(
                gckHARDWARE_SetPowerManagementState(
                    hardware,
                    gcvPOWER_OFF_TIMEOUT));

            /* relax cpu 200 ms before retry */
            gckOS_Delay(device->os, 200);

            gcmkVERIFY_OK(
                gckHARDWARE_QueryPowerManagementState(hardware, &state));
        }
        while (state == gcvPOWER_IDLE);
    }
}
#endif

/******************************************************************************\
******************************* gckGALDEVICE Code ******************************
\******************************************************************************/

/*******************************************************************************
**
**  gckGALDEVICE_Construct
**
**  Constructor.
**
**  INPUT:
**
**  OUTPUT:
**
**      gckGALDEVICE * Device
**          Pointer to a variable receiving the gckGALDEVICE object pointer on
**          success.
*/
gceSTATUS
gckGALDEVICE_Construct(
    IN gctINT IrqLine,
    IN gctUINT32 RegisterMemBase,
    IN gctSIZE_T RegisterMemSize,
    IN gctINT IrqLine2D,
    IN gctUINT32 RegisterMemBase2D,
    IN gctSIZE_T RegisterMemSize2D,
    IN gctINT IrqLineVG,
    IN gctUINT32 RegisterMemBaseVG,
    IN gctSIZE_T RegisterMemSizeVG,
    IN gctUINT32 ContiguousBase,
    IN gctSIZE_T ContiguousSize,
    IN gctSIZE_T BankSize,
    IN gctINT FastClear,
    IN gctINT Compression,
    IN gctUINT32 PhysBaseAddr,
    IN gctUINT32 PhysSize,
    IN gctINT Signal,
    OUT gckGALDEVICE *Device
    )
{
    gctUINT32 internalBaseAddress = 0, internalAlignment = 0;
    gctUINT32 externalBaseAddress = 0, externalAlignment = 0;
    gctUINT32 horizontalTileSize, verticalTileSize;
    struct resource* mem_region;
    gctUINT32 physAddr;
    gctUINT32 physical;
    gckGALDEVICE device;
    gceSTATUS status;
    gctINT32 i;
    gceHARDWARE_TYPE type;
    gckDB sharedDB = gcvNULL;

    gcmkHEADER_ARG("IrqLine=%d RegisterMemBase=0x%08x RegisterMemSize=%u "
                   "IrqLine2D=%d RegisterMemBase2D=0x%08x RegisterMemSize2D=%u "
                   "IrqLineVG=%d RegisterMemBaseVG=0x%08x RegisterMemSizeVG=%u "
                   "ContiguousBase=0x%08x ContiguousSize=%lu BankSize=%lu "
                   "FastClear=%d Compression=%d PhysBaseAddr=0x%x PhysSize=%d Signal=%d",
                   IrqLine, RegisterMemBase, RegisterMemSize,
                   IrqLine2D, RegisterMemBase2D, RegisterMemSize2D,
                   IrqLineVG, RegisterMemBaseVG, RegisterMemSizeVG,
                   ContiguousBase, ContiguousSize, BankSize, FastClear, Compression,
                   PhysBaseAddr, PhysSize, Signal);

    /* Allocate device structure. */
    device = kmalloc(sizeof(struct _gckGALDEVICE), GFP_KERNEL | __GFP_NOWARN);

    if (!device)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    memset(device, 0, sizeof(struct _gckGALDEVICE));

    /*Initialize the clock structure*/
    if (IrqLine != -1) {
        device->clk_3d_core = clk_get(NULL, "gpu3d_clk");
        if (!IS_ERR(device->clk_3d_core)) {
            device->clk_3d_shader = clk_get(NULL, "gpu3d_shader_clk");
            if (IS_ERR(device->clk_3d_shader)) {
                IrqLine = -1;
                clk_put(device->clk_3d_core);
                device->clk_3d_core = NULL;
                device->clk_3d_shader = NULL;
                gckOS_Print("galcore: clk_get gpu3d_shader_clk failed, disable 3d!\n");
            }
        } else {
            IrqLine = -1;
            device->clk_3d_core = NULL;
            gckOS_Print("galcore: clk_get gpu3d_clk failed, disable 3d!\n");
        }
    }
    if ((IrqLine2D != -1) || (IrqLineVG != -1)) {
        device->clk_2d_core = clk_get(NULL, "gpu2d_clk");
        if (IS_ERR(device->clk_2d_core)) {
            IrqLine2D = -1;
            IrqLineVG = -1;
            device->clk_2d_core = NULL;
            gckOS_Print("galcore: clk_get 2d clock failed, disable 2d/vg!\n");
        }
    }

    if (IrqLine != -1)
    {
        device->requestedRegisterMemBases[gcvCORE_MAJOR]    = RegisterMemBase;
        device->requestedRegisterMemSizes[gcvCORE_MAJOR]    = RegisterMemSize;
    }

    if (IrqLine2D != -1)
    {
        device->requestedRegisterMemBases[gcvCORE_2D]       = RegisterMemBase2D;
        device->requestedRegisterMemSizes[gcvCORE_2D]       = RegisterMemSize2D;
    }

    if (IrqLineVG != -1)
    {
        device->requestedRegisterMemBases[gcvCORE_VG]       = RegisterMemBaseVG;
        device->requestedRegisterMemSizes[gcvCORE_VG]       = RegisterMemSizeVG;
    }

    device->requestedContiguousBase  = 0;
    device->requestedContiguousSize  = 0;


    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        physical = device->requestedRegisterMemBases[i];

        /* Set up register memory region. */
        if (physical != 0)
        {
            mem_region = request_mem_region(
                physical, device->requestedRegisterMemSizes[i], "galcore register region"
                );

            if (mem_region == gcvNULL)
            {
                gcmkTRACE_ZONE(
                    gcvLEVEL_ERROR, gcvZONE_DRIVER,
                    "%s(%d): Failed to claim %lu bytes @ 0x%08X\n",
                    __FUNCTION__, __LINE__,
                    physical, device->requestedRegisterMemSizes[i]
                    );

                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }

            device->registerBases[i] = (gctPOINTER) ioremap_nocache(
                physical, device->requestedRegisterMemSizes[i]);

            if (device->registerBases[i] == gcvNULL)
            {
                gcmkTRACE_ZONE(
                    gcvLEVEL_ERROR, gcvZONE_DRIVER,
                    "%s(%d): Unable to map %ld bytes @ 0x%08X\n",
                    __FUNCTION__, __LINE__,
                    physical, device->requestedRegisterMemSizes[i]
                    );

                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }

            physical += device->requestedRegisterMemSizes[i];
        }
        else
        {
            device->registerBases[i] = gcvNULL;
        }
    }

    /* Set the base address */
    device->baseAddress = PhysBaseAddr;

    /* Construct the gckOS object. */
    gcmkONERROR(gckOS_Construct(device, &device->os));

    if (IrqLine != -1)
    {
        /* Construct the gckKERNEL object. */
        gcmkONERROR(gckKERNEL_Construct(
            device->os, gcvCORE_MAJOR, device,
            gcvNULL, &device->kernels[gcvCORE_MAJOR]));

        sharedDB = device->kernels[gcvCORE_MAJOR]->db;

        /* Initialize core mapping */
        for (i = 0; i < 8; i++)
        {
            device->coreMapping[i] = gcvCORE_MAJOR;
        }

        /* Setup the ISR manager. */
        gcmkONERROR(gckHARDWARE_SetIsrManager(
            device->kernels[gcvCORE_MAJOR]->hardware,
            (gctISRMANAGERFUNC) gckGALDEVICE_Setup_ISR,
            (gctISRMANAGERFUNC) gckGALDEVICE_Release_ISR,
            device
            ));

        gcmkONERROR(gckHARDWARE_SetFastClear(
            device->kernels[gcvCORE_MAJOR]->hardware, FastClear, Compression
            ));


#if COMMAND_PROCESSOR_VERSION == 1
        /* Start the command queue. */
        gcmkONERROR(gckCOMMAND_Start(device->kernels[gcvCORE_MAJOR]->command));
#endif
    }
    else
    {
        device->kernels[gcvCORE_MAJOR] = gcvNULL;
    }

    if (IrqLine2D != -1)
    {
        gcmkONERROR(gckKERNEL_Construct(
            device->os, gcvCORE_2D, device,
            sharedDB, &device->kernels[gcvCORE_2D]));

        if (sharedDB == gcvNULL) sharedDB = device->kernels[gcvCORE_2D]->db;

        /* Verify the hardware type */
        gcmkONERROR(gckHARDWARE_GetType(device->kernels[gcvCORE_2D]->hardware, &type));

        if (type != gcvHARDWARE_2D)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Unexpected hardware type: %d\n",
                __FUNCTION__, __LINE__,
                type
                );

            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        /* Initialize core mapping */
        if (device->kernels[gcvCORE_MAJOR] == gcvNULL)
        {
            for (i = 0; i < 8; i++)
            {
                device->coreMapping[i] = gcvCORE_2D;
            }
        }
        else
        {
            device->coreMapping[gcvHARDWARE_2D] = gcvCORE_2D;
        }

        /* Setup the ISR manager. */
        gcmkONERROR(gckHARDWARE_SetIsrManager(
            device->kernels[gcvCORE_2D]->hardware,
            (gctISRMANAGERFUNC) gckGALDEVICE_Setup_ISR_2D,
            (gctISRMANAGERFUNC) gckGALDEVICE_Release_ISR_2D,
            device
            ));

#if COMMAND_PROCESSOR_VERSION == 1
        /* Start the command queue. */
        gcmkONERROR(gckCOMMAND_Start(device->kernels[gcvCORE_2D]->command));
#endif

#if gcdMULTICORE_MAPPING
        device->kernels[gcvCORE_2D]->anotherKernel = device->kernels[gcvCORE_MAJOR];
#endif
    }
    else
    {
        device->kernels[gcvCORE_2D] = gcvNULL;
    }

#if gcdMULTICORE_MAPPING
    device->kernels[gcvCORE_MAJOR]->anotherKernel = device->kernels[gcvCORE_2D];
#endif

    if (IrqLineVG != -1)
    {
#if gcdENABLE_VG
        gcmkONERROR(gckKERNEL_Construct(
            device->os, gcvCORE_VG, device,
            sharedDB, &device->kernels[gcvCORE_VG]));
        /* Initialize core mapping */
        if (device->kernels[gcvCORE_MAJOR] == gcvNULL
            && device->kernels[gcvCORE_2D] == gcvNULL
            )
        {
            for (i = 0; i < 8; i++)
            {
                device->coreMapping[i] = gcvCORE_VG;
            }
        }
        else
        {
            device->coreMapping[gcvHARDWARE_VG] = gcvCORE_VG;
        }

#endif
    }
    else
    {
        device->kernels[gcvCORE_VG] = gcvNULL;
    }

    /* Initialize the ISR. */
    device->irqLines[gcvCORE_MAJOR] = IrqLine;
    device->irqLines[gcvCORE_2D]    = IrqLine2D;
    device->irqLines[gcvCORE_VG]    = IrqLineVG;

    /* Initialize the kernel thread semaphores. */
    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (device->irqLines[i] != -1) sema_init(&device->semas[i], 0);
    }

    device->signal = Signal;

    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        if (device->kernels[i] != gcvNULL) break;
    }

    if (i == gcdCORE_COUNT) gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

#if gcdENABLE_VG
    if (i == gcvCORE_VG)
    {
        /* Query the ceiling of the system memory. */
        gcmkONERROR(gckVGHARDWARE_QuerySystemMemory(
                device->kernels[i]->vg->hardware,
                &device->systemMemorySize,
                &device->systemMemoryBaseAddress
                ));
            /* query the amount of video memory */
        gcmkONERROR(gckVGHARDWARE_QueryMemory(
            device->kernels[i]->vg->hardware,
            &device->internalSize, &internalBaseAddress, &internalAlignment,
            &device->externalSize, &externalBaseAddress, &externalAlignment,
            &horizontalTileSize, &verticalTileSize
            ));
    }
    else
#endif
    {
        /* Query the ceiling of the system memory. */
        gcmkONERROR(gckHARDWARE_QuerySystemMemory(
                device->kernels[i]->hardware,
                &device->systemMemorySize,
                &device->systemMemoryBaseAddress
                ));

            /* query the amount of video memory */
        gcmkONERROR(gckHARDWARE_QueryMemory(
            device->kernels[i]->hardware,
            &device->internalSize, &internalBaseAddress, &internalAlignment,
            &device->externalSize, &externalBaseAddress, &externalAlignment,
            &horizontalTileSize, &verticalTileSize
            ));
    }


    /* Set up the internal memory region. */
    if (device->internalSize > 0)
    {
        status = gckVIDMEM_Construct(
            device->os,
            internalBaseAddress, device->internalSize, internalAlignment,
            0, &device->internalVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable internal heap. */
            device->internalSize = 0;
        }
        else
        {
            /* Map internal memory. */
            device->internalLogical
                = (gctPOINTER) ioremap_nocache(physical, device->internalSize);

            if (device->internalLogical == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }

            device->internalPhysical = (gctPHYS_ADDR) physical;
            physical += device->internalSize;
        }
    }

    if (device->externalSize > 0)
    {
        /* create the external memory heap */
        status = gckVIDMEM_Construct(
            device->os,
            externalBaseAddress, device->externalSize, externalAlignment,
            0, &device->externalVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable internal heap. */
            device->externalSize = 0;
        }
        else
        {
            /* Map external memory. */
            device->externalLogical
                = (gctPOINTER) ioremap_nocache(physical, device->externalSize);

            if (device->externalLogical == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }

            device->externalPhysical = (gctPHYS_ADDR) physical;
            physical += device->externalSize;
        }
    }

    /* set up the contiguous memory */
    device->contiguousSize = ContiguousSize;

    if (ContiguousSize > 0)
    {
        if (ContiguousBase == 0)
        {
            while (device->contiguousSize > 0)
            {
                /* Allocate contiguous memory. */
                status = _AllocateMemory(
                    device,
                    device->contiguousSize,
                    &device->contiguousBase,
                    &device->contiguousPhysical,
                    &physAddr
                    );

                if (gcmIS_SUCCESS(status))
                {
                    status = gckVIDMEM_Construct(
                        device->os,
                        physAddr | device->systemMemoryBaseAddress,
                        device->contiguousSize,
                        64,
                        BankSize,
                        &device->contiguousVidMem
                        );

                    if (gcmIS_SUCCESS(status))
                    {
                        break;
                    }

                    gcmkONERROR(_FreeMemory(
                        device,
                        device->contiguousBase,
                        device->contiguousPhysical
                        ));

                    device->contiguousBase     = gcvNULL;
                    device->contiguousPhysical = gcvNULL;
                }

                if (device->contiguousSize <= (4 << 20))
                {
                    device->contiguousSize = 0;
                }
                else
                {
                    device->contiguousSize -= (4 << 20);
                }
            }
        }
        else
        {
            /* Create the contiguous memory heap. */
            status = gckVIDMEM_Construct(
                device->os,
                (ContiguousBase - device->baseAddress) | device->systemMemoryBaseAddress,
                 ContiguousSize,
                64, BankSize,
                &device->contiguousVidMem
                );

            if (gcmIS_ERROR(status))
            {
                /* Error, disable contiguous memory pool. */
                device->contiguousVidMem = gcvNULL;
                device->contiguousSize   = 0;
            }
            else
            {
                mem_region = request_mem_region(
                    ContiguousBase, ContiguousSize, "galcore managed memory"
                    );

                if (mem_region == gcvNULL)
                {
                    gcmkTRACE_ZONE(
                        gcvLEVEL_ERROR, gcvZONE_DRIVER,
                        "%s(%d): Failed to claim %ld bytes @ 0x%08X\n",
                        __FUNCTION__, __LINE__,
                        ContiguousSize, ContiguousBase
                        );

                    gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                }

                device->requestedContiguousBase  = ContiguousBase;
                device->requestedContiguousSize  = ContiguousSize;

#if !gcdDYNAMIC_MAP_RESERVED_MEMORY && gcdENABLE_VG
                if (gcmIS_CORE_PRESENT(device, gcvCORE_VG))
                {
                    device->contiguousBase
#if gcdPAGED_MEMORY_CACHEABLE
                        = (gctPOINTER) ioremap_cached(ContiguousBase, ContiguousSize);
#else
                        = (gctPOINTER) ioremap_nocache(ContiguousBase, ContiguousSize);
#endif
                    if (device->contiguousBase == gcvNULL)
                    {
                        device->contiguousVidMem = gcvNULL;
                        device->contiguousSize = 0;

                        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                    }
                }
#endif

                device->contiguousPhysical = (gctPHYS_ADDR) ContiguousBase;
                device->contiguousSize     = ContiguousSize;
                device->contiguousMapped   = gcvTRUE;
            }
        }
    }

    /* Return pointer to the device. */
    * Device = device;

    gcmkFOOTER_ARG("*Device=0x%x", * Device);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    gcmkVERIFY_OK(gckGALDEVICE_Destroy(device));

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Destroy
**
**  Class destructor.
**
**  INPUT:
**
**      Nothing.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Destroy(
    gckGALDEVICE Device)
{
    gctINT i;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Device=0x%x", Device);

    if (Device != gcvNULL)
    {
        for (i = 0; i < gcdCORE_COUNT; i++)
        {
            if (Device->kernels[i] != gcvNULL)
            {
                /* Destroy the gckKERNEL object. */
                gcmkVERIFY_OK(gckKERNEL_Destroy(Device->kernels[i]));
                Device->kernels[i] = gcvNULL;
            }
        }

        {
            if (Device->internalLogical != gcvNULL)
            {
                /* Unmap the internal memory. */
                iounmap(Device->internalLogical);
                Device->internalLogical = gcvNULL;
            }

            if (Device->internalVidMem != gcvNULL)
            {
                /* Destroy the internal heap. */
                gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->internalVidMem));
                Device->internalVidMem = gcvNULL;
            }
        }

        {
            if (Device->externalLogical != gcvNULL)
            {
                /* Unmap the external memory. */
                iounmap(Device->externalLogical);
                Device->externalLogical = gcvNULL;
            }

            if (Device->externalVidMem != gcvNULL)
            {
                /* destroy the external heap */
                gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->externalVidMem));
                Device->externalVidMem = gcvNULL;
            }
        }

        {
            if (Device->contiguousBase != gcvNULL)
            {
                if (Device->contiguousMapped)
                {
#if !gcdDYNAMIC_MAP_RESERVED_MEMORY && gcdENABLE_VG
                    if (Device->contiguousBase)
                    {
                        /* Unmap the contiguous memory. */
                        iounmap(Device->contiguousBase);
                    }
#endif
                }
                else
                {
                    gcmkONERROR(_FreeMemory(
                        Device,
                        Device->contiguousBase,
                        Device->contiguousPhysical
                        ));
                }

                Device->contiguousBase     = gcvNULL;
                Device->contiguousPhysical = gcvNULL;
            }

            if (Device->requestedContiguousBase != 0)
            {
                release_mem_region(Device->requestedContiguousBase, Device->requestedContiguousSize);
                Device->requestedContiguousBase = 0;
                Device->requestedContiguousSize = 0;
            }

            if (Device->contiguousVidMem != gcvNULL)
            {
                /* Destroy the contiguous heap. */
                gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->contiguousVidMem));
                Device->contiguousVidMem = gcvNULL;
            }
        }

        for (i = 0; i < gcdCORE_COUNT; i++)
        {
            if (Device->registerBases[i] != gcvNULL)
            {
                /* Unmap register memory. */
                iounmap(Device->registerBases[i]);
			    if (Device->requestedRegisterMemBases[i] != 0)
			    {
				    release_mem_region(Device->requestedRegisterMemBases[i], Device->requestedRegisterMemSizes[i]);
			    }

                Device->registerBases[i] = gcvNULL;
                Device->requestedRegisterMemBases[i] = 0;
                Device->requestedRegisterMemSizes[i] = 0;
            }
        }

        /*Disable clock*/
        if (Device->clk_3d_core) {
           clk_put(Device->clk_3d_core);
           Device->clk_3d_core = NULL;
        }
        if (Device->clk_3d_shader) {
           clk_put(Device->clk_3d_shader);
           Device->clk_3d_shader = NULL;
        }
        if (Device->clk_2d_core) {
           clk_put(Device->clk_2d_core);
           Device->clk_2d_core = NULL;
        }

        /* Destroy the gckOS object. */
        if (Device->os != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_Destroy(Device->os));
            Device->os = gcvNULL;
        }

        /* Free the device. */
        kfree(Device);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Setup_ISR
**
**  Start the ISR routine.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Setup successfully.
**      gcvSTATUS_GENERIC_IO
**          Setup failed.
*/
gceSTATUS
gckGALDEVICE_Setup_ISR(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status;
    gctINT ret;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->irqLines[gcvCORE_MAJOR] < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Hook up the isr based on the irq line. */
#ifdef FLAREON
    gc500_handle.dev_name  = "galcore interrupt service";
    gc500_handle.dev_id    = Device;
    gc500_handle.handler   = isrRoutine;
    gc500_handle.intr_gen  = GPIO_INTR_LEVEL_TRIGGER;
    gc500_handle.intr_trig = GPIO_TRIG_HIGH_LEVEL;

    ret = dove_gpio_request(
        DOVE_GPIO0_7, &gc500_handle
        );
#else
    ret = request_irq(
        Device->irqLines[gcvCORE_MAJOR], isrRoutine, IRQF_DISABLED,
        "galcore interrupt service", Device
        );
#endif

    if (ret != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not register irq line %d (error=%d)\n",
            __FUNCTION__, __LINE__,
            Device->irqLines[gcvCORE_MAJOR], ret
            );

        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Mark ISR as initialized. */
    Device->isrInitializeds[gcvCORE_MAJOR] = gcvTRUE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckGALDEVICE_Setup_ISR_2D(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status;
    gctINT ret;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->irqLines[gcvCORE_2D] < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Hook up the isr based on the irq line. */
#ifdef FLAREON
    gc500_handle.dev_name  = "galcore interrupt service";
    gc500_handle.dev_id    = Device;
    gc500_handle.handler   = isrRoutine2D;
    gc500_handle.intr_gen  = GPIO_INTR_LEVEL_TRIGGER;
    gc500_handle.intr_trig = GPIO_TRIG_HIGH_LEVEL;

    ret = dove_gpio_request(
        DOVE_GPIO0_7, &gc500_handle
        );
#else
    ret = request_irq(
        Device->irqLines[gcvCORE_2D], isrRoutine2D, IRQF_DISABLED,
        "galcore interrupt service for 2D", Device
        );
#endif

    if (ret != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not register irq line %d (error=%d)\n",
            __FUNCTION__, __LINE__,
            Device->irqLines[gcvCORE_2D], ret
            );

        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Mark ISR as initialized. */
    Device->isrInitializeds[gcvCORE_2D] = gcvTRUE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckGALDEVICE_Setup_ISR_VG(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status;
    gctINT ret;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->irqLines[gcvCORE_VG] < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Hook up the isr based on the irq line. */
#ifdef FLAREON
    gc500_handle.dev_name  = "galcore interrupt service";
    gc500_handle.dev_id    = Device;
    gc500_handle.handler   = isrRoutineVG;
    gc500_handle.intr_gen  = GPIO_INTR_LEVEL_TRIGGER;
    gc500_handle.intr_trig = GPIO_TRIG_HIGH_LEVEL;

    ret = dove_gpio_request(
        DOVE_GPIO0_7, &gc500_handle
        );
#else
    ret = request_irq(
        Device->irqLines[gcvCORE_VG], isrRoutineVG, IRQF_DISABLED,
        "galcore interrupt service for 2D", Device
        );
#endif

    if (ret != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not register irq line %d (error=%d)\n",
            __FUNCTION__, __LINE__,
            Device->irqLines[gcvCORE_VG], ret
            );

        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Mark ISR as initialized. */
    Device->isrInitializeds[gcvCORE_VG] = gcvTRUE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Release_ISR
**
**  Release the irq line.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Release_ISR(
    IN gckGALDEVICE Device
    )
{
    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    /* release the irq */
    if (Device->isrInitializeds[gcvCORE_MAJOR])
    {
#ifdef FLAREON
        dove_gpio_free(DOVE_GPIO0_7, "galcore interrupt service");
#else
        free_irq(Device->irqLines[gcvCORE_MAJOR], Device);
#endif

	    Device->isrInitializeds[gcvCORE_MAJOR] = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckGALDEVICE_Release_ISR_2D(
    IN gckGALDEVICE Device
    )
{
    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    /* release the irq */
    if (Device->isrInitializeds[gcvCORE_2D])
    {
#ifdef FLAREON
        dove_gpio_free(DOVE_GPIO0_7, "galcore interrupt service");
#else
        free_irq(Device->irqLines[gcvCORE_2D], Device);
#endif

	    Device->isrInitializeds[gcvCORE_2D] = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckGALDEVICE_Release_ISR_VG(
    IN gckGALDEVICE Device
    )
{
    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    /* release the irq */
    if (Device->isrInitializeds[gcvCORE_VG])
    {
#ifdef FLAREON
        dove_gpio_free(DOVE_GPIO0_7, "galcore interrupt service");
#else
        free_irq(Device->irqLines[gcvCORE_VG], Device);
#endif

	    Device->isrInitializeds[gcvCORE_VG] = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckGALDEVICE_Start_Threads
**
**  Start the daemon threads.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Start successfully.
**      gcvSTATUS_GENERIC_IO
**          Start failed.
*/
gceSTATUS
gckGALDEVICE_Start_Threads(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status;
    struct task_struct * task;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->kernels[gcvCORE_MAJOR] != gcvNULL)
    {
        /* Start the kernel thread. */
        task = kthread_run(threadRoutine, Device, "galcore daemon thread");

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        Device->threadCtxts[gcvCORE_MAJOR]          = task;
        Device->threadInitializeds[gcvCORE_MAJOR]   = gcvTRUE;

#if gcdPOWEROFF_TIMEOUT
        /* Start the kernel thread. */
        task = kthread_run(threadRoutinePM, Device, "galcore pm thread");

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        Device->pmThreadCtxts          = task;
        Device->pmThreadInitializeds   = gcvTRUE;
#endif
    }

    if (Device->kernels[gcvCORE_2D] != gcvNULL)
    {
        /* Start the kernel thread. */
        task = kthread_run(threadRoutine2D, Device, "galcore daemon thread for 2D");

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        Device->threadCtxts[gcvCORE_2D]         = task;
        Device->threadInitializeds[gcvCORE_2D]  = gcvTRUE;
    }
    else
    {
        Device->threadInitializeds[gcvCORE_2D]  = gcvFALSE;
    }

    if (Device->kernels[gcvCORE_VG] != gcvNULL)
    {
        /* Start the kernel thread. */
        task = kthread_run(threadRoutineVG, Device, "galcore daemon thread for VG");

        if (IS_ERR(task))
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Could not start the kernel thread.\n",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_GENERIC_IO);
        }

        Device->threadCtxts[gcvCORE_VG]         = task;
        Device->threadInitializeds[gcvCORE_VG]  = gcvTRUE;
    }
    else
    {
        Device->threadInitializeds[gcvCORE_VG]  = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Stop_Threads
**
**  Stop the gal device, including the following actions: stop the daemon
**  thread, release the irq.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Stop_Threads(
    gckGALDEVICE Device
    )
{
    gctINT i;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    for (i = 0; i < gcdCORE_COUNT; i++)
    {
        /* Stop the kernel threads. */
        if (Device->threadInitializeds[i])
        {
            Device->killThread = gcvTRUE;
            up(&Device->semas[i]);

            kthread_stop(Device->threadCtxts[i]);
            Device->threadCtxts[i]        = gcvNULL;
            Device->threadInitializeds[i] = gcvFALSE;
        }
    }

#if gcdPOWEROFF_TIMEOUT
    /* Stop the kernel threads. */
    if (Device->pmThreadInitializeds)
    {
        gckHARDWARE hardware = Device->kernels[gcvCORE_MAJOR]->hardware;
        Device->killThread = gcvTRUE;
        gckOS_Signal(Device->os, hardware->powerOffSignal, gcvTRUE);

        kthread_stop(Device->pmThreadCtxts);
        Device->pmThreadCtxts        = gcvNULL;
        Device->pmThreadInitializeds = gcvFALSE;
    }
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckGALDEVICE_Start
**
**  Start the gal device, including the following actions: setup the isr routine
**  and start the daemoni thread.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Start successfully.
*/
gceSTATUS
gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=0x%x", Device);

    /* Start the kernel thread. */
    gcmkONERROR(gckGALDEVICE_Start_Threads(Device));

    if (Device->kernels[gcvCORE_MAJOR] != gcvNULL)
    {
        /* Setup the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Setup_ISR(Device));

        /* Switch to SUSPEND power state. */
        gcmkONERROR(gckHARDWARE_SetPowerManagementState(
            Device->kernels[gcvCORE_MAJOR]->hardware, gcvPOWER_OFF_BROADCAST
            ));
    }

    if (Device->kernels[gcvCORE_2D] != gcvNULL)
    {
        /* Setup the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Setup_ISR_2D(Device));

        /* Switch to SUSPEND power state. */
        gcmkONERROR(gckHARDWARE_SetPowerManagementState(
            Device->kernels[gcvCORE_2D]->hardware, gcvPOWER_OFF_BROADCAST
            ));
    }

    if (Device->kernels[gcvCORE_VG] != gcvNULL)
    {
        /* Setup the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Setup_ISR_VG(Device));
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Stop
**
**  Stop the gal device, including the following actions: stop the daemon
**  thread, release the irq.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Stop(
    gckGALDEVICE Device
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=0x%x", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->kernels[gcvCORE_MAJOR] != gcvNULL)
    {
        /* Switch to OFF power state. */
        gcmkONERROR(gckHARDWARE_SetPowerManagementState(
            Device->kernels[gcvCORE_MAJOR]->hardware, gcvPOWER_OFF
            ));

        /* Remove the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Release_ISR(Device));
    }

    if (Device->kernels[gcvCORE_2D] != gcvNULL)
    {
        /* Setup the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Release_ISR_2D(Device));

        /* Switch to OFF power state. */
        gcmkONERROR(gckHARDWARE_SetPowerManagementState(
            Device->kernels[gcvCORE_2D]->hardware, gcvPOWER_OFF
            ));
    }

    if (Device->kernels[gcvCORE_VG] != gcvNULL)
    {
        /* Setup the ISR routine. */
        gcmkONERROR(gckGALDEVICE_Release_ISR_VG(Device));

#if gcdENABLE_VG
        /* Switch to OFF power state. */
        gcmkONERROR(gckVGHARDWARE_SetPowerManagementState(
            Device->kernels[gcvCORE_VG]->vg->hardware, gcvPOWER_OFF
            ));
#endif
    }

    /* Stop the kernel thread. */
    gcmkONERROR(gckGALDEVICE_Stop_Threads(Device));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}
