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

/////////////////////////////////////////////////////////////////////////////
// macros
//////////////////////////////////////////////////////////////////////////////
#define GSL_SHMEM_APERTURE_MARK(aperture_id)    \
    (shmem->priv |= (((aperture_id + 1) << GSL_APERTURE_SHIFT) & GSL_APERTURE_MASK))

#define GSL_SHMEM_APERTURE_ISMARKED(aperture_id)    \
    (((shmem->priv & GSL_APERTURE_MASK) >> GSL_APERTURE_SHIFT) & (aperture_id + 1))

#define GSL_MEMFLAGS_APERTURE_GET(flags, aperture_id)                                                       \
    aperture_id = (gsl_apertureid_t)((flags & GSL_MEMFLAGS_APERTURE_MASK) >> GSL_MEMFLAGS_APERTURE_SHIFT);  \
    KOS_ASSERT(aperture_id < GSL_APERTURE_MAX);

#define GSL_MEMFLAGS_CHANNEL_GET(flags, channel_id)                                                     \
    channel_id = (gsl_channelid_t)((flags & GSL_MEMFLAGS_CHANNEL_MASK) >> GSL_MEMFLAGS_CHANNEL_SHIFT);  \
    KOS_ASSERT(channel_id < GSL_CHANNEL_MAX);

#define GSL_MEMDESC_APERTURE_SET(memdesc, aperture_index)   \
    memdesc->priv = (memdesc->priv & ~GSL_APERTURE_MASK) | ((aperture_index << GSL_APERTURE_SHIFT) & GSL_APERTURE_MASK);

#define GSL_MEMDESC_DEVICE_SET(memdesc, device_id)  \
    memdesc->priv = (memdesc->priv & ~GSL_DEVICEID_MASK) | ((device_id << GSL_DEVICEID_SHIFT) & GSL_DEVICEID_MASK);

#define GSL_MEMDESC_EXTALLOC_SET(memdesc, flag) \
    memdesc->priv = (memdesc->priv & ~GSL_EXTALLOC_MASK) | ((flag << GSL_EXTALLOC_SHIFT) & GSL_EXTALLOC_MASK);

#define GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index)                           \
    KOS_ASSERT(memdesc);                                                            \
    aperture_index = ((memdesc->priv & GSL_APERTURE_MASK) >> GSL_APERTURE_SHIFT);   \
    KOS_ASSERT(aperture_index < GSL_SHMEM_MAX_APERTURES);

#define GSL_MEMDESC_DEVICE_GET(memdesc, device_id)                                              \
    KOS_ASSERT(memdesc);                                                                        \
    device_id = (gsl_deviceid_t)((memdesc->priv & GSL_DEVICEID_MASK) >> GSL_DEVICEID_SHIFT);    \
    KOS_ASSERT(device_id <= GSL_DEVICE_MAX);

#define GSL_MEMDESC_EXTALLOC_ISMARKED(memdesc)  \
    ((memdesc->priv & GSL_EXTALLOC_MASK) >> GSL_EXTALLOC_SHIFT)


//////////////////////////////////////////////////////////////////////////////
// aperture index in shared memory object
//////////////////////////////////////////////////////////////////////////////
OSINLINE int
kgsl_sharedmem_getapertureindex(gsl_sharedmem_t *shmem, gsl_apertureid_t aperture_id, gsl_channelid_t channel_id)
{
    KOS_ASSERT(shmem->aperturelookup[aperture_id][channel_id] < shmem->numapertures);

    return (shmem->aperturelookup[aperture_id][channel_id]);
}


//////////////////////////////////////////////////////////////////////////////
// functions
//////////////////////////////////////////////////////////////////////////////

int
kgsl_sharedmem_init(gsl_sharedmem_t *shmem)
{
    int                i;
    int                status;
    gsl_shmemconfig_t  config;
    int                mmu_virtualized;
    gsl_apertureid_t   aperture_id;
    gsl_channelid_t    channel_id;
    unsigned int       hostbaseaddr;
    gpuaddr_t          gpubaseaddr;
    int                sizebytes;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_sharedmem_init(gsl_sharedmem_t *shmem=0x%08x)\n", shmem );

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_init. Return value %B\n", GSL_SUCCESS );
        return (GSL_SUCCESS);
    }

    status = kgsl_hal_getshmemconfig(&config);
    if (status != GSL_SUCCESS)
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Unable to get sharedmem config.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_init. Return value %B\n", status );
        return (status);
    }

    shmem->numapertures = config.numapertures;

    for (i = 0; i < shmem->numapertures; i++)
    {
        aperture_id     = config.apertures[i].id;
        channel_id      = config.apertures[i].channel;
        hostbaseaddr    = config.apertures[i].hostbase;
        gpubaseaddr     = config.apertures[i].gpubase;
        sizebytes       = config.apertures[i].sizebytes;
        mmu_virtualized = 0;

        // handle mmu virtualized aperture
        if (aperture_id == GSL_APERTURE_MMU)
        {
            mmu_virtualized = 1;
            aperture_id     = GSL_APERTURE_EMEM;
        }

        // make sure aligned to page size
        KOS_ASSERT((gpubaseaddr & ((1 << GSL_PAGESIZE_SHIFT) - 1)) == 0);

        // make a multiple of page size
        sizebytes = (sizebytes & ~((1 << GSL_PAGESIZE_SHIFT) - 1));

        if (sizebytes > 0)
        {
            shmem->apertures[i].memarena = kgsl_memarena_create(aperture_id, mmu_virtualized, hostbaseaddr, gpubaseaddr, sizebytes);

            if (!shmem->apertures[i].memarena)
            {
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Unable to allocate memarena.\n" );
                kgsl_sharedmem_close(shmem);
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_init. Return value %B\n", GSL_FAILURE );
                return (GSL_FAILURE);
            }

            shmem->apertures[i].id       = aperture_id;
            shmem->apertures[i].channel  = channel_id;
            shmem->apertures[i].numbanks = 1;

            // create aperture lookup table
            if (GSL_SHMEM_APERTURE_ISMARKED(aperture_id))
            {
                // update "current aperture_id"/"current channel_id" index
                shmem->aperturelookup[aperture_id][channel_id] = i;
            }
            else
            {
                // initialize "current aperture_id"/"channel_id" indexes
                for (channel_id = GSL_CHANNEL_1; channel_id < GSL_CHANNEL_MAX; channel_id++)
                {
                    shmem->aperturelookup[aperture_id][channel_id] = i;
                }

                GSL_SHMEM_APERTURE_MARK(aperture_id);
            }
        }
    }

    shmem->flags |= GSL_FLAGS_INITIALIZED;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_init. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_close(gsl_sharedmem_t *shmem)
{
    int  i;
    int  result = GSL_SUCCESS;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_sharedmem_close(gsl_sharedmem_t *shmem=0x%08x)\n", shmem );

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        for (i = 0; i < shmem->numapertures; i++)
        {
            if (shmem->apertures[i].memarena)
            {
                result = kgsl_memarena_destroy(shmem->apertures[i].memarena);
            }
        }

        kos_memset(shmem, 0, sizeof(gsl_sharedmem_t));
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_close. Return value %B\n", result );

    return (result);
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_alloc0(gsl_deviceid_t device_id, gsl_flags_t flags, int sizebytes, gsl_memdesc_t *memdesc)
{
    gsl_apertureid_t  aperture_id;
    gsl_channelid_t   channel_id;
    gsl_deviceid_t    tmp_id;
    int               aperture_index, org_index;
    int               result  = GSL_FAILURE;
    gsl_mmu_t         *mmu    = NULL;
    gsl_sharedmem_t   *shmem  = &gsl_driver.shmem;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_alloc(gsl_deviceid_t device_id=%D, gsl_flags_t flags=0x%08x, int sizebytes=%d, gsl_memdesc_t *memdesc=%M)\n",
                    device_id, flags, sizebytes, memdesc );

    KOS_ASSERT(sizebytes);
    KOS_ASSERT(memdesc);

    GSL_MEMFLAGS_APERTURE_GET(flags, aperture_id);
    GSL_MEMFLAGS_CHANNEL_GET(flags, channel_id);

    kos_memset(memdesc, 0, sizeof(gsl_memdesc_t));

    if (!(shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Shared memory not initialized.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_alloc. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    // execute pending device action
    tmp_id = (device_id != GSL_DEVICE_ANY) ? device_id : device_id+1;
    for ( ; tmp_id <= GSL_DEVICE_MAX; tmp_id++)
    {
        if (gsl_driver.device[tmp_id-1].flags & GSL_FLAGS_INITIALIZED)
        {
            kgsl_device_runpending(&gsl_driver.device[tmp_id-1]);

            if (tmp_id == device_id)
            {
                break;
            }
        }
    }

    // convert any device to an actual existing device
    if (device_id == GSL_DEVICE_ANY)
    {
        for ( ; ; )
        {
            device_id++;

            if (device_id <= GSL_DEVICE_MAX)
            {
                if (gsl_driver.device[device_id-1].flags & GSL_FLAGS_INITIALIZED)
                {
                    break;
                }
            }
            else
            {
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Invalid device.\n" );
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_alloc. Return value %B\n", GSL_FAILURE );
                return (GSL_FAILURE);
            }
        }
    }

    KOS_ASSERT(device_id > GSL_DEVICE_ANY && device_id <= GSL_DEVICE_MAX);

    // get mmu reference
    mmu = &gsl_driver.device[device_id-1].mmu;

    aperture_index = kgsl_sharedmem_getapertureindex(shmem, aperture_id, channel_id);

    //do not proceed if it is a strict request, the aperture requested is not present, and the MMU is enabled
    if (!((flags & GSL_MEMFLAGS_STRICTREQUEST) && aperture_id != shmem->apertures[aperture_index].id && kgsl_mmu_isenabled(mmu)))
    {
        // do allocation
        result = kgsl_memarena_alloc(shmem->apertures[aperture_index].memarena, flags, sizebytes, memdesc);

        // if allocation failed
        if (result != GSL_SUCCESS)
        {
            org_index = aperture_index;

            // then failover to other channels within the current aperture
            for (channel_id = GSL_CHANNEL_1; channel_id < GSL_CHANNEL_MAX; channel_id++)
            {
                aperture_index = kgsl_sharedmem_getapertureindex(shmem, aperture_id, channel_id);

                if (aperture_index != org_index)
                {
                    // do allocation
                    result = kgsl_memarena_alloc(shmem->apertures[aperture_index].memarena, flags, sizebytes, memdesc);

                    if (result == GSL_SUCCESS)
                    {
                        break;
                    }
                }
            }

            // if allocation still has not succeeded, then failover to EMEM/MMU aperture, but
            // not if it's a strict request and the MMU is enabled
            if (result != GSL_SUCCESS && aperture_id != GSL_APERTURE_EMEM
                && !((flags & GSL_MEMFLAGS_STRICTREQUEST) && kgsl_mmu_isenabled(mmu)))
            {
                aperture_id = GSL_APERTURE_EMEM;

                // try every channel
                for (channel_id = GSL_CHANNEL_1; channel_id < GSL_CHANNEL_MAX; channel_id++)
                {
                    aperture_index = kgsl_sharedmem_getapertureindex(shmem, aperture_id, channel_id);

                    if (aperture_index != org_index)
                    {
                        // do allocation
                        result = kgsl_memarena_alloc(shmem->apertures[aperture_index].memarena, flags, sizebytes, memdesc);

                        if (result == GSL_SUCCESS)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    if (result == GSL_SUCCESS)
    {
        GSL_MEMDESC_APERTURE_SET(memdesc, aperture_index);
        GSL_MEMDESC_DEVICE_SET(memdesc, device_id);

        if (kgsl_memarena_isvirtualized(shmem->apertures[aperture_index].memarena))
        {
            gsl_scatterlist_t scatterlist;

            scatterlist.contiguous = 0;
            scatterlist.num        = memdesc->size / GSL_PAGESIZE;

            if (memdesc->size & (GSL_PAGESIZE-1))
            {
                scatterlist.num++;
            }

            scatterlist.pages = kos_malloc(sizeof(unsigned int) * scatterlist.num);
            if (scatterlist.pages)
            {
                // allocate physical pages
                result = kgsl_hal_allocphysical(memdesc->gpuaddr, scatterlist.num, scatterlist.pages);
                if (result == GSL_SUCCESS)
                {
                    result = kgsl_mmu_map(mmu, memdesc->gpuaddr, &scatterlist, flags, GSL_CALLER_PROCESSID_GET());
                    if (result != GSL_SUCCESS)
                    {
                        kgsl_hal_freephysical(memdesc->gpuaddr, scatterlist.num, scatterlist.pages);
                    }
                }

                kos_free(scatterlist.pages);
            }
            else
            {
                result = GSL_FAILURE;
            }

            if (result != GSL_SUCCESS)
            {
                kgsl_memarena_free(shmem->apertures[aperture_index].memarena, memdesc);
            }
        }
    }

    KGSL_DEBUG_TBDUMP_SETMEM( memdesc->gpuaddr, 0, memdesc->size );

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_alloc. Return value %B\n", result );

    return (result);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_alloc(gsl_deviceid_t device_id, gsl_flags_t flags, int sizebytes, gsl_memdesc_t *memdesc)
{
	int status = GSL_SUCCESS;
	GSL_API_MUTEX_LOCK();
	status = kgsl_sharedmem_alloc0(device_id, flags, sizebytes, memdesc);
	GSL_API_MUTEX_UNLOCK();
	return status;
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_free0(gsl_memdesc_t *memdesc, unsigned int pid)
{
    int              status = GSL_SUCCESS;
    int              aperture_index;
    gsl_deviceid_t   device_id;
    gsl_sharedmem_t  *shmem;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "--> int kgsl_sharedmem_free(gsl_memdesc_t *memdesc=%M)\n", memdesc );

    GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index);
    GSL_MEMDESC_DEVICE_GET(memdesc, device_id);

    shmem = &gsl_driver.shmem;

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        if (kgsl_memarena_isvirtualized(shmem->apertures[aperture_index].memarena))
        {
            status |= kgsl_mmu_unmap(&gsl_driver.device[device_id-1].mmu, memdesc->gpuaddr, memdesc->size, pid);

            if (!GSL_MEMDESC_EXTALLOC_ISMARKED(memdesc))
            {
                status |= kgsl_hal_freephysical(memdesc->gpuaddr, memdesc->size / GSL_PAGESIZE, NULL);
            }
        }

        kgsl_memarena_free(shmem->apertures[aperture_index].memarena, memdesc);

        // clear descriptor
        kos_memset(memdesc, 0, sizeof(gsl_memdesc_t));
    }
    else
    {
        status = GSL_FAILURE;
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_free. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_free(gsl_memdesc_t *memdesc)
{
	int status = GSL_SUCCESS;
    GSL_API_MUTEX_LOCK();
    status = kgsl_sharedmem_free0(memdesc, GSL_CALLER_PROCESSID_GET());
    GSL_API_MUTEX_UNLOCK();
    return status;
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_read0(const gsl_memdesc_t *memdesc, void *dst, unsigned int offsetbytes, unsigned int sizebytes, unsigned int touserspace)
{
    int              aperture_index;
    gsl_sharedmem_t  *shmem;
    unsigned int     gpuoffsetbytes;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_read(gsl_memdesc_t *memdesc=%M, void *dst=0x%08x, unsigned int offsetbytes=%d, unsigned int sizebytes=%d)\n",
                    memdesc, dst, offsetbytes, sizebytes );

    GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index);

    if (GSL_MEMDESC_EXTALLOC_ISMARKED(memdesc))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_read. Return value %B\n", GSL_FAILURE_BADPARAM );
        return (GSL_FAILURE_BADPARAM);
    }

    shmem = &gsl_driver.shmem;

    if (!(shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Shared memory not initialized.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_read. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    KOS_ASSERT(dst);
    KOS_ASSERT(sizebytes);

    if (memdesc->gpuaddr < shmem->apertures[aperture_index].memarena->gpubaseaddr)
    {
        return (GSL_FAILURE_BADPARAM);
    }

    if (memdesc->gpuaddr + sizebytes > shmem->apertures[aperture_index].memarena->gpubaseaddr + shmem->apertures[aperture_index].memarena->sizebytes)
    {
        return (GSL_FAILURE_BADPARAM);
    }

    gpuoffsetbytes = (memdesc->gpuaddr - shmem->apertures[aperture_index].memarena->gpubaseaddr) + offsetbytes;

    GSL_HAL_MEM_READ(dst, shmem->apertures[aperture_index].memarena->hostbaseaddr, gpuoffsetbytes, sizebytes, touserspace);

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_read. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_read(const gsl_memdesc_t *memdesc, void *dst, unsigned int offsetbytes, unsigned int sizebytes, unsigned int touserspace)
{
	int status = GSL_SUCCESS;
	GSL_API_MUTEX_LOCK();
	status = kgsl_sharedmem_read0(memdesc, dst, offsetbytes, sizebytes, touserspace);
	GSL_API_MUTEX_UNLOCK();
	return status;
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_write0(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, void *src, unsigned int sizebytes, unsigned int fromuserspace)
{
    int              aperture_index;
    gsl_sharedmem_t  *shmem;
    unsigned int     gpuoffsetbytes;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_write(gsl_memdesc_t *memdesc=%M, unsigned int offsetbytes=%d, void *src=0x%08x, unsigned int sizebytes=%d)\n",
                    memdesc, offsetbytes, src, sizebytes );

    GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index);

    if (GSL_MEMDESC_EXTALLOC_ISMARKED(memdesc))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_write. Return value %B\n", GSL_FAILURE_BADPARAM );
        return (GSL_FAILURE_BADPARAM);
    }

    shmem = &gsl_driver.shmem;

    if (!(shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Shared memory not initialized.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_write. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    KOS_ASSERT(src);
    KOS_ASSERT(sizebytes);
    KOS_ASSERT(memdesc->gpuaddr >= shmem->apertures[aperture_index].memarena->gpubaseaddr);
    KOS_ASSERT((memdesc->gpuaddr + sizebytes) <= (shmem->apertures[aperture_index].memarena->gpubaseaddr + shmem->apertures[aperture_index].memarena->sizebytes));

    gpuoffsetbytes = (memdesc->gpuaddr - shmem->apertures[aperture_index].memarena->gpubaseaddr) + offsetbytes;

    GSL_HAL_MEM_WRITE(shmem->apertures[aperture_index].memarena->hostbaseaddr, gpuoffsetbytes, src, sizebytes, fromuserspace);

    KGSL_DEBUG(GSL_DBGFLAGS_PM4MEM, KGSL_DEBUG_DUMPMEMWRITE((memdesc->gpuaddr + offsetbytes), sizebytes, src));

    KGSL_DEBUG_TBDUMP_SYNCMEM( (memdesc->gpuaddr + offsetbytes), src, sizebytes );

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_write. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_write(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, void *src, unsigned int sizebytes, unsigned int fromuserspace)
{
	int status = GSL_SUCCESS;
	GSL_API_MUTEX_LOCK();
	status = kgsl_sharedmem_write0(memdesc, offsetbytes, src, sizebytes, fromuserspace);
	GSL_API_MUTEX_UNLOCK();
	return status;
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_set0(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, unsigned int value, unsigned int sizebytes)
{
    int              aperture_index;
    gsl_sharedmem_t  *shmem;
    unsigned int     gpuoffsetbytes;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_set(gsl_memdesc_t *memdesc=%M, unsigned int offsetbytes=%d, unsigned int value=0x%08x, unsigned int sizebytes=%d)\n",
                    memdesc, offsetbytes, value, sizebytes );

    GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index);

    if (GSL_MEMDESC_EXTALLOC_ISMARKED(memdesc))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_set. Return value %B\n", GSL_FAILURE_BADPARAM );
        return (GSL_FAILURE_BADPARAM);
    }

    shmem = &gsl_driver.shmem;

    if (!(shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Shared memory not initialized.\n" );
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_set. Return value %B\n", GSL_FAILURE );
        return (GSL_FAILURE);
    }

    KOS_ASSERT(sizebytes);
    KOS_ASSERT(memdesc->gpuaddr >= shmem->apertures[aperture_index].memarena->gpubaseaddr);
    KOS_ASSERT((memdesc->gpuaddr + sizebytes) <= (shmem->apertures[aperture_index].memarena->gpubaseaddr + shmem->apertures[aperture_index].memarena->sizebytes));

    gpuoffsetbytes = (memdesc->gpuaddr - shmem->apertures[aperture_index].memarena->gpubaseaddr) + offsetbytes;

    GSL_HAL_MEM_SET(shmem->apertures[aperture_index].memarena->hostbaseaddr, gpuoffsetbytes, value, sizebytes);

    KGSL_DEBUG(GSL_DBGFLAGS_PM4MEM, KGSL_DEBUG_DUMPMEMSET((memdesc->gpuaddr + offsetbytes), sizebytes, value));

    KGSL_DEBUG_TBDUMP_SETMEM( (memdesc->gpuaddr + offsetbytes), value, sizebytes );

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_set. Return value %B\n", GSL_SUCCESS );

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_set(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, unsigned int value, unsigned int sizebytes)
{
	int status = GSL_SUCCESS;
	GSL_API_MUTEX_LOCK();
	status = kgsl_sharedmem_set0(memdesc, offsetbytes, value, sizebytes);
	GSL_API_MUTEX_UNLOCK();
	return status;
}

//----------------------------------------------------------------------------

KGSL_API unsigned int
kgsl_sharedmem_largestfreeblock(gsl_deviceid_t device_id, gsl_flags_t flags)
{
    gsl_apertureid_t  aperture_id;
    gsl_channelid_t   channel_id;
    int               aperture_index;
    unsigned int      result = 0;
    gsl_sharedmem_t   *shmem;

    // device_id is ignored at this level, it would be used with per-device memarena's

    // unreferenced formal parameter
    (void) device_id;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_largestfreeblock(gsl_deviceid_t device_id=%D, gsl_flags_t flags=0x%08x)\n",
                    device_id, flags );

    GSL_MEMFLAGS_APERTURE_GET(flags, aperture_id);
    GSL_MEMFLAGS_CHANNEL_GET(flags, channel_id);

    GSL_API_MUTEX_LOCK();

    shmem = &gsl_driver.shmem;

    if (!(shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Shared memory not initialized.\n" );
        GSL_API_MUTEX_UNLOCK();
        kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_largestfreeblock. Return value %d\n", 0 );
        return (0);
    }

    aperture_index = kgsl_sharedmem_getapertureindex(shmem, aperture_id, channel_id);

    if (aperture_id == shmem->apertures[aperture_index].id)
    {
        result = kgsl_memarena_getlargestfreeblock(shmem->apertures[aperture_index].memarena, flags);
    }

    GSL_API_MUTEX_UNLOCK();

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_largestfreeblock. Return value %d\n", result );

    return (result);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_map(gsl_deviceid_t device_id, gsl_flags_t flags, const gsl_scatterlist_t *scatterlist, gsl_memdesc_t *memdesc)
{
    int              status = GSL_FAILURE;
    gsl_sharedmem_t  *shmem = &gsl_driver.shmem;
    int              aperture_index;
    gsl_deviceid_t   tmp_id;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_map(gsl_deviceid_t device_id=%D, gsl_flags_t flags=0x%08x, gsl_scatterlist_t scatterlist=%M, gsl_memdesc_t *memdesc=%M)\n",
                    device_id, flags, memdesc, scatterlist );

    // execute pending device action
    tmp_id = (device_id != GSL_DEVICE_ANY) ? device_id : device_id+1;
    for ( ; tmp_id <= GSL_DEVICE_MAX; tmp_id++)
    {
        if (gsl_driver.device[tmp_id-1].flags & GSL_FLAGS_INITIALIZED)
        {
            kgsl_device_runpending(&gsl_driver.device[tmp_id-1]);

            if (tmp_id == device_id)
            {
                break;
            }
        }
    }

    // convert any device to an actual existing device
    if (device_id == GSL_DEVICE_ANY)
    {
        for ( ; ; )
        {
            device_id++;

            if (device_id <= GSL_DEVICE_MAX)
            {
                if (gsl_driver.device[device_id-1].flags & GSL_FLAGS_INITIALIZED)
                {
                    break;
                }
            }
            else
            {
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_ERROR, "ERROR: Invalid device.\n" );
                kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_map. Return value %B\n", GSL_FAILURE );
                return (GSL_FAILURE);
            }
        }
    }

    KOS_ASSERT(device_id > GSL_DEVICE_ANY && device_id <= GSL_DEVICE_MAX);

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        aperture_index = kgsl_sharedmem_getapertureindex(shmem, GSL_APERTURE_EMEM, GSL_CHANNEL_1);

        if (kgsl_memarena_isvirtualized(shmem->apertures[aperture_index].memarena))
        {
            KOS_ASSERT(scatterlist->num);
            KOS_ASSERT(scatterlist->pages);

            status = kgsl_memarena_alloc(shmem->apertures[aperture_index].memarena, flags, scatterlist->num *GSL_PAGESIZE, memdesc);
            if (status == GSL_SUCCESS)
            {
                GSL_MEMDESC_APERTURE_SET(memdesc, aperture_index);
                GSL_MEMDESC_DEVICE_SET(memdesc, device_id);

                // mark descriptor's memory as externally allocated -- i.e. outside GSL
                GSL_MEMDESC_EXTALLOC_SET(memdesc, 1);

                status = kgsl_mmu_map(&gsl_driver.device[device_id-1].mmu, memdesc->gpuaddr, scatterlist, flags, GSL_CALLER_PROCESSID_GET());
                if (status != GSL_SUCCESS)
                {
                    kgsl_memarena_free(shmem->apertures[aperture_index].memarena, memdesc);
                }
            }
        }
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_map. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_unmap(gsl_memdesc_t *memdesc)
{
    return (kgsl_sharedmem_free0(memdesc, GSL_CALLER_PROCESSID_GET()));
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_getmap(const gsl_memdesc_t *memdesc, gsl_scatterlist_t *scatterlist)
{
    int              status = GSL_SUCCESS;
    int              aperture_index;
    gsl_deviceid_t   device_id;
    gsl_sharedmem_t  *shmem;

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE,
                    "--> int kgsl_sharedmem_getmap(gsl_memdesc_t *memdesc=%M, gsl_scatterlist_t scatterlist=%M)\n",
                    memdesc, scatterlist );

    GSL_MEMDESC_APERTURE_GET(memdesc, aperture_index);
    GSL_MEMDESC_DEVICE_GET(memdesc, device_id);

    shmem = &gsl_driver.shmem;

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        KOS_ASSERT(scatterlist->num);
        KOS_ASSERT(scatterlist->pages);
        KOS_ASSERT(memdesc->gpuaddr >= shmem->apertures[aperture_index].memarena->gpubaseaddr);
        KOS_ASSERT((memdesc->gpuaddr + memdesc->size) <= (shmem->apertures[aperture_index].memarena->gpubaseaddr + shmem->apertures[aperture_index].memarena->sizebytes));

        kos_memset(scatterlist->pages, 0, sizeof(unsigned int) * scatterlist->num);

        if (kgsl_memarena_isvirtualized(shmem->apertures[aperture_index].memarena))
        {
            status = kgsl_mmu_getmap(&gsl_driver.device[device_id-1].mmu, memdesc->gpuaddr, memdesc->size, scatterlist, GSL_CALLER_PROCESSID_GET());
        }
        else
        {
            // coalesce physically contiguous pages into a single scatter list entry
            scatterlist->pages[0]   = memdesc->gpuaddr;
            scatterlist->contiguous = 1;
        }
    }

    kgsl_log_write( KGSL_LOG_GROUP_MEMORY | KGSL_LOG_LEVEL_TRACE, "<-- kgsl_sharedmem_getmap. Return value %B\n", status );

    return (status);
}

//----------------------------------------------------------------------------

int
kgsl_sharedmem_querystats(gsl_sharedmem_t *shmem, gsl_sharedmem_stats_t *stats)
{
#ifdef GSL_STATS_MEM
    int  status = GSL_SUCCESS;
    int  i;

    KOS_ASSERT(stats);

    if (shmem->flags & GSL_FLAGS_INITIALIZED)
    {
        for (i = 0; i < shmem->numapertures; i++)
        {
            if (shmem->apertures[i].memarena)
            {
                stats->apertures[i].id      = shmem->apertures[i].id;
                stats->apertures[i].channel = shmem->apertures[i].channel;

                status |= kgsl_memarena_querystats(shmem->apertures[i].memarena, &stats->apertures[i].memarena);
            }
        }
    }
    else
    {
        kos_memset(stats, 0, sizeof(gsl_sharedmem_stats_t));
    }

    return (status);
#else
    // unreferenced formal parameters
    (void) shmem;
    (void) stats;

    return (GSL_FAILURE_NOTSUPPORTED);
#endif // GSL_STATS_MEM
}

//----------------------------------------------------------------------------

unsigned int
kgsl_sharedmem_convertaddr(unsigned int addr, int type)
{
    gsl_sharedmem_t  *shmem  = &gsl_driver.shmem;
    unsigned int     cvtaddr = 0;
    unsigned int     gpubaseaddr, hostbaseaddr, sizebytes;
    int              i;

    if ((shmem->flags & GSL_FLAGS_INITIALIZED))
    {
        for (i = 0; i < shmem->numapertures; i++)
        {
            hostbaseaddr = shmem->apertures[i].memarena->hostbaseaddr;
            gpubaseaddr  = shmem->apertures[i].memarena->gpubaseaddr;
            sizebytes    = shmem->apertures[i].memarena->sizebytes;

            // convert from gpu to host
            if (type == 0)
            {
                if (addr >= gpubaseaddr && addr < (gpubaseaddr + sizebytes))
                {
                    cvtaddr = hostbaseaddr + (addr - gpubaseaddr);
                    break;
                }
            }
            // convert from host to gpu
            else if (type == 1)
            {
                if (addr >= hostbaseaddr && addr < (hostbaseaddr + sizebytes))
                {
                    cvtaddr = gpubaseaddr + (addr - hostbaseaddr);
                    break;
                }
            }
        }
    }

    return (cvtaddr);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_cacheoperation(const gsl_memdesc_t *memdesc, unsigned int offsetbytes, unsigned int sizebytes, unsigned int operation)
{
    int status  = GSL_FAILURE;

    /* unreferenced formal parameter */
    (void)memdesc;
    (void)offsetbytes;
    (void)sizebytes;
    (void)operation;

    /* do cache operation */

    return (status);
}

//----------------------------------------------------------------------------

KGSL_API int
kgsl_sharedmem_fromhostpointer(gsl_deviceid_t device_id, gsl_memdesc_t *memdesc, void* hostptr)
{
    int status  = GSL_FAILURE;

    memdesc->gpuaddr = (gpuaddr_t)hostptr;  /* map physical address with hostptr    */
    memdesc->hostptr = hostptr;             /* set virtual address also in memdesc  */

    /* unreferenced formal parameter */
    (void)device_id;

    return (status);
}
