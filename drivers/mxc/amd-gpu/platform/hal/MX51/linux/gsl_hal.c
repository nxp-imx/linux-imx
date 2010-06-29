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

#include "gsl_hal.h"
#include "gsl_halconfig.h"
#include "gsl_memcfg.h"
#include "gsl_linux_map.h"

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

//////////////////////////////////////////////////////////////////////////////
// constants
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// defines
//////////////////////////////////////////////////////////////////////////////

#define GSL_HAL_MEM1                        0
#define GSL_HAL_MEM2                        1   
#define GSL_HAL_MEM3                        2

//#define GSL_HAL_DEBUG

//////////////////////////////////////////////////////////////////////////////
// types
//////////////////////////////////////////////////////////////////////////////

typedef struct _gsl_hal_t {
    gsl_memregion_t z160_regspace;
    gsl_memregion_t z430_regspace;
    gsl_memregion_t memchunk;
    gsl_memregion_t memspace[GSL_SHMEM_MAX_APERTURES];
} gsl_hal_t;

extern phys_addr_t gpu_2d_regbase;
extern int gpu_2d_regsize;
extern phys_addr_t gpu_3d_regbase;
extern int gpu_3d_regsize;
extern int gmem_size;
extern phys_addr_t gpu_reserved_mem;
extern int gpu_reserved_mem_size;

//////////////////////////////////////////////////////////////////////////////
//  functions
//////////////////////////////////////////////////////////////////////////////

KGSLHAL_API int
kgsl_hal_allocphysical(unsigned int virtaddr, unsigned int numpages, unsigned int scattergatterlist[])
{
    //
    // allocate physically contiguous memory
    //
    
	int i;
	void *va;

	va = gsl_linux_map_alloc(virtaddr, numpages*PAGE_SIZE);

	if (!va)
		return (GSL_FAILURE_OUTOFMEM);

	for(i = 0; i < numpages; i++)
	{
		scattergatterlist[i] = page_to_phys(vmalloc_to_page(va));
		va += PAGE_SIZE;
	}

	return (GSL_SUCCESS);
}

// ---------------------------------------------------------------------------

KGSLHAL_API int
kgsl_hal_freephysical(unsigned int virtaddr, unsigned int numpages, unsigned int scattergatterlist[])
{
    //
    // free physical memory
    //

	gsl_linux_map_free(virtaddr);

    return(GSL_SUCCESS);
}

//----------------------------------------------------------------------------

KGSLHAL_API int 
kgsl_hal_init(void)
{
    gsl_hal_t *hal;
    unsigned long totalsize, mem1size;
    unsigned int va, pa;

    if (gsl_driver.hal)
    {
        return (GSL_FAILURE_ALREADYINITIALIZED);
    }

    gsl_driver.hal = (void *)kos_malloc(sizeof(gsl_hal_t));

    if (!gsl_driver.hal)
    {
        return (GSL_FAILURE_OUTOFMEM);
    }

    kos_memset(gsl_driver.hal, 0, sizeof(gsl_hal_t));


    // overlay structure on hal memory
    hal = (gsl_hal_t *) gsl_driver.hal;

    // setup register space
    if(gpu_3d_regbase && gpu_3d_regsize){
        hal->z430_regspace.mmio_phys_base = gpu_3d_regbase;
        hal->z430_regspace.sizebytes = gpu_3d_regsize;
    }else{
        hal->z430_regspace.mmio_phys_base   = GSL_HAL_GPUBASE_REG_YDX;
        hal->z430_regspace.sizebytes        = GSL_HAL_SIZE_REG_YDX;
    }
    hal->z430_regspace.mmio_virt_base   = (unsigned char*)ioremap(hal->z430_regspace.mmio_phys_base, hal->z430_regspace.sizebytes);

    if (hal->z430_regspace.mmio_virt_base == NULL) 
    {
        return (GSL_FAILURE_SYSTEMERROR);
    }

#ifdef GSL_HAL_DEBUG
    printk(KERN_INFO "%s: hal->z430_regspace.mmio_phys_base = 0x%p\n", __func__, (void*)hal->z430_regspace.mmio_phys_base);
    printk(KERN_INFO "%s: hal->z430_regspace.mmio_virt_base = 0x%p\n", __func__, (void*)hal->z430_regspace.mmio_virt_base);
    printk(KERN_INFO "%s: hal->z430_regspace.sizebytes      = 0x%08x\n", __func__, hal->z430_regspace.sizebytes);
#endif

    if(gpu_2d_regbase && gpu_2d_regsize){
        hal->z160_regspace.mmio_phys_base = gpu_2d_regbase;
        hal->z160_regspace.sizebytes = gpu_2d_regsize;
    }else{
        hal->z160_regspace.mmio_phys_base   = GSL_HAL_GPUBASE_REG_G12;
        hal->z160_regspace.sizebytes        = GSL_HAL_SIZE_REG_G12;
    }
    hal->z160_regspace.mmio_virt_base   = (unsigned char*)ioremap(hal->z160_regspace.mmio_phys_base, hal->z160_regspace.sizebytes);

    if (hal->z160_regspace.mmio_virt_base == NULL) 
    {
        return (GSL_FAILURE_SYSTEMERROR);
    }

#ifdef GSL_HAL_DEBUG
    printk(KERN_INFO "%s: hal->z160_regspace.mmio_phys_base = 0x%p\n", __func__, (void*)hal->z160_regspace.mmio_phys_base);
    printk(KERN_INFO "%s: hal->z160_regspace.mmio_virt_base = 0x%p\n", __func__, (void*)hal->z160_regspace.mmio_virt_base);
    printk(KERN_INFO "%s: hal->z160_regspace.sizebytes      = 0x%08x\n", __func__, hal->z160_regspace.sizebytes);
#endif
    
#ifdef GSL_MMU_TRANSLATION_ENABLED
    totalsize = GSL_HAL_SHMEM_SIZE_EMEM2 + GSL_HAL_SHMEM_SIZE_PHYS;
    mem1size = GSL_HAL_SHMEM_SIZE_EMEM1;
    if (gpu_reserved_mem && gpu_reserved_mem_size >= totalsize)
    {
        pa = gpu_reserved_mem;
        va = (unsigned int)ioremap(gpu_reserved_mem, totalsize);
    }
    else
    {
        va = (unsigned int)dma_alloc_coherent(0, totalsize, (dma_addr_t *)&pa, GFP_DMA | GFP_KERNEL);
    }
#else
    if(gpu_reserved_mem && gpu_reserved_mem_size >= SZ_8M){
        totalsize = gpu_reserved_mem_size;
        pa = gpu_reserved_mem;
        va = (unsigned int)ioremap(gpu_reserved_mem, gpu_reserved_mem_size);
    }else{
        gpu_reserved_mem = 0;
        totalsize = GSL_HAL_SHMEM_SIZE_EMEM1 + GSL_HAL_SHMEM_SIZE_EMEM2 + GSL_HAL_SHMEM_SIZE_PHYS;
        va = (unsigned int)dma_alloc_coherent(0, totalsize, (dma_addr_t *)&pa, GFP_DMA | GFP_KERNEL);
    }
    mem1size = totalsize - (GSL_HAL_SHMEM_SIZE_EMEM2 + GSL_HAL_SHMEM_SIZE_PHYS);
#endif
    
    if (va)
    {
        kos_memset((void *)va, 0, totalsize);
    
        hal->memchunk.mmio_virt_base = (void *)va;
        hal->memchunk.mmio_phys_base = pa;
        hal->memchunk.sizebytes      = totalsize;

#ifdef GSL_HAL_DEBUG
        printk(KERN_INFO "%s: hal->memchunk.mmio_phys_base = 0x%p\n", __func__, (void*)hal->memchunk.mmio_phys_base);
        printk(KERN_INFO "%s: hal->memchunk.mmio_virt_base = 0x%p\n", __func__, (void*)hal->memchunk.mmio_virt_base);
        printk(KERN_INFO "%s: hal->memchunk.sizebytes      = 0x%08x\n", __func__, hal->memchunk.sizebytes);
#endif
        
        hal->memspace[GSL_HAL_MEM2].mmio_virt_base = (void *) va;
        hal->memspace[GSL_HAL_MEM2].gpu_base       = pa;
        hal->memspace[GSL_HAL_MEM2].sizebytes      = GSL_HAL_SHMEM_SIZE_EMEM2;
        va += GSL_HAL_SHMEM_SIZE_EMEM2;
        pa += GSL_HAL_SHMEM_SIZE_EMEM2;

#ifdef GSL_HAL_DEBUG
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM2].gpu_base       = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM2].gpu_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM2].mmio_virt_base = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM2].mmio_virt_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM2].sizebytes      = 0x%08x\n", __func__, hal->memspace[GSL_HAL_MEM2].sizebytes);
#endif
        
        hal->memspace[GSL_HAL_MEM3].mmio_virt_base  = (void *) va;
        hal->memspace[GSL_HAL_MEM3].gpu_base        = pa;
        hal->memspace[GSL_HAL_MEM3].sizebytes       = GSL_HAL_SHMEM_SIZE_PHYS;
        va += GSL_HAL_SHMEM_SIZE_PHYS;
        pa += GSL_HAL_SHMEM_SIZE_PHYS;

#ifdef GSL_HAL_DEBUG
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM3].gpu_base       = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM3].gpu_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM3].mmio_virt_base = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM3].mmio_virt_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM3].sizebytes      = 0x%08x\n", __func__, hal->memspace[GSL_HAL_MEM3].sizebytes);
#endif
        
#ifdef GSL_MMU_TRANSLATION_ENABLED
        gsl_linux_map_init();
        hal->memspace[GSL_HAL_MEM1].mmio_virt_base = (void *)GSL_LINUX_MAP_RANGE_START;
        hal->memspace[GSL_HAL_MEM1].gpu_base       = GSL_LINUX_MAP_RANGE_START;
        hal->memspace[GSL_HAL_MEM1].sizebytes      = mem1size;
#else
        hal->memspace[GSL_HAL_MEM1].mmio_virt_base = (void *) va;
        hal->memspace[GSL_HAL_MEM1].gpu_base       = pa;
        hal->memspace[GSL_HAL_MEM1].sizebytes      = mem1size;
#endif

#ifdef GSL_HAL_DEBUG
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM1].gpu_base       = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM1].gpu_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM1].mmio_virt_base = 0x%p\n", __func__, (void*)hal->memspace[GSL_HAL_MEM1].mmio_virt_base);
        printk(KERN_INFO "%s: hal->memspace[GSL_HAL_MEM1].sizebytes      = 0x%08x\n", __func__, hal->memspace[GSL_HAL_MEM1].sizebytes);
#endif
    }
    else
    {
        kgsl_hal_close();
        return (GSL_FAILURE_SYSTEMERROR);
    }

    return GSL_SUCCESS;
}

//----------------------------------------------------------------------------

KGSLHAL_API int 
kgsl_hal_close(void)
{
    gsl_hal_t  *hal;

    if (gsl_driver.hal)
    {
        // overlay structure on hal memory
        hal = (gsl_hal_t *) gsl_driver.hal;

        // unmap registers
        if (hal->z430_regspace.mmio_virt_base)
        {
            iounmap(hal->z430_regspace.mmio_virt_base);
        }
        if (hal->z160_regspace.mmio_virt_base)
        {
            iounmap(hal->z160_regspace.mmio_virt_base);
        }
        
        // free physical block
        if (hal->memchunk.mmio_virt_base && gpu_reserved_mem)
        {
            iounmap(hal->memchunk.mmio_virt_base);
        }
        else
        {
            dma_free_coherent(0, hal->memchunk.sizebytes, hal->memchunk.mmio_virt_base, hal->memchunk.mmio_phys_base);
        }

#ifdef GSL_MMU_TRANSLATION_ENABLED
		gsl_linux_map_destroy();
#endif

        // release hal struct
        kos_memset(hal, 0, sizeof(gsl_hal_t));
        kos_free(gsl_driver.hal);
        gsl_driver.hal = NULL;
    }

    return (GSL_SUCCESS);
}

//----------------------------------------------------------------------------

KGSLHAL_API int
kgsl_hal_getshmemconfig(gsl_shmemconfig_t *config)
{ 
    int        status = GSL_FAILURE_DEVICEERROR;
    gsl_hal_t  *hal   = (gsl_hal_t *) gsl_driver.hal;

    kos_memset(config, 0, sizeof(gsl_shmemconfig_t));

    if (hal)
    {
        config->numapertures = GSL_SHMEM_MAX_APERTURES;

#ifdef GSL_MMU_TRANSLATION_ENABLED
        config->apertures[0].id        = GSL_APERTURE_MMU;
#else
        config->apertures[0].id        = GSL_APERTURE_EMEM;
#endif
        config->apertures[0].channel   = GSL_CHANNEL_1;
        config->apertures[0].hostbase  = (unsigned int)hal->memspace[GSL_HAL_MEM1].mmio_virt_base;
        config->apertures[0].gpubase   = hal->memspace[GSL_HAL_MEM1].gpu_base;
        config->apertures[0].sizebytes = hal->memspace[GSL_HAL_MEM1].sizebytes;

        config->apertures[1].id        = GSL_APERTURE_EMEM;
        config->apertures[1].channel   = GSL_CHANNEL_2;
        config->apertures[1].hostbase  = (unsigned int)hal->memspace[GSL_HAL_MEM2].mmio_virt_base;
        config->apertures[1].gpubase   = hal->memspace[GSL_HAL_MEM2].gpu_base;
        config->apertures[1].sizebytes = hal->memspace[GSL_HAL_MEM2].sizebytes;

        config->apertures[2].id        = GSL_APERTURE_PHYS;
        config->apertures[2].channel   = GSL_CHANNEL_1;
        config->apertures[2].hostbase  = (unsigned int)hal->memspace[GSL_HAL_MEM3].mmio_virt_base;
        config->apertures[2].gpubase   = hal->memspace[GSL_HAL_MEM3].gpu_base;
        config->apertures[2].sizebytes = hal->memspace[GSL_HAL_MEM3].sizebytes;

        status = GSL_SUCCESS;
    }

    return (status);
}

//----------------------------------------------------------------------------

KGSLHAL_API int
kgsl_hal_getdevconfig(gsl_deviceid_t device_id, gsl_devconfig_t *config)
{ 
    int        status = GSL_FAILURE_DEVICEERROR;
    gsl_hal_t  *hal   = (gsl_hal_t *) gsl_driver.hal;

    kos_memset(config, 0, sizeof(gsl_devconfig_t));

    if (hal)
    {
        switch (device_id)
        {
            case GSL_DEVICE_YAMATO:
            {
                mh_mmu_config_u      mmu_config   = {0};

                config->gmemspace.gpu_base        = 0;
                config->gmemspace.mmio_virt_base  = 0;
                config->gmemspace.mmio_phys_base  = 0;
                if(gmem_size){
                    config->gmemspace.sizebytes = gmem_size;
                }else{
                    config->gmemspace.sizebytes = GSL_HAL_SIZE_GMEM;
                }

                config->regspace.gpu_base         = 0;
                config->regspace.mmio_virt_base   = (unsigned char *)hal->z430_regspace.mmio_virt_base;
                config->regspace.mmio_phys_base   = (unsigned int) hal->z430_regspace.mmio_phys_base;
                config->regspace.sizebytes        = GSL_HAL_SIZE_REG_YDX;

                mmu_config.f.mmu_enable           = 1;
#ifdef GSL_MMU_TRANSLATION_ENABLED
                mmu_config.f.split_mode_enable    = 0;
                mmu_config.f.rb_w_clnt_behavior   = 1;
                mmu_config.f.cp_w_clnt_behavior   = 1;
                mmu_config.f.cp_r0_clnt_behavior  = 1;
                mmu_config.f.cp_r1_clnt_behavior  = 1;
                mmu_config.f.cp_r2_clnt_behavior  = 1;
                mmu_config.f.cp_r3_clnt_behavior  = 1;
                mmu_config.f.cp_r4_clnt_behavior  = 1;
                mmu_config.f.vgt_r0_clnt_behavior = 1;
                mmu_config.f.vgt_r1_clnt_behavior = 1;
                mmu_config.f.tc_r_clnt_behavior   = 1;
                mmu_config.f.pa_w_clnt_behavior   = 1;
#endif // GSL_MMU_TRANSLATION_ENABLED

                config->mmu_config                = mmu_config.val;
#ifdef GSL_MMU_TRANSLATION_ENABLED
                config->va_base                   = hal->memspace[GSL_HAL_MEM1].gpu_base;
                config->va_range                  = hal->memspace[GSL_HAL_MEM1].sizebytes;
#else
                config->va_base                   = 0x00000000;
                config->va_range                  = 0x00000000;
#endif // GSL_MMU_TRANSLATION_ENABLED
        
                // turn off memory protection unit by setting acceptable physical address range to include all pages
                config->mpu_base                  = 0x00000000; // hal->memchunk.mmio_virt_base;
                config->mpu_range                 = 0xFFFFF000; // hal->memchunk.sizebytes;

                status = GSL_SUCCESS;
                break;
            }

            case GSL_DEVICE_G12:
            {
                mh_mmu_config_u      mmu_config   = {0};

                config->regspace.gpu_base       = 0;
                config->regspace.mmio_virt_base = (unsigned char *)hal->z160_regspace.mmio_virt_base;
                config->regspace.mmio_phys_base = (unsigned int) hal->z160_regspace.mmio_phys_base;
                config->regspace.sizebytes      = GSL_HAL_SIZE_REG_G12;

                mmu_config.f.mmu_enable           = 1;

#ifdef GSL_MMU_TRANSLATION_ENABLED
                config->mmu_config              = 0x00555551;
                config->va_base                 = hal->memspace[GSL_HAL_MEM1].gpu_base;
                config->va_range                = hal->memspace[GSL_HAL_MEM1].sizebytes;
#else
                config->mmu_config              = mmu_config.val;
                config->va_base                 = 0x00000000;
                config->va_range                = 0x00000000;
#endif // GSL_MMU_TRANSLATION_ENABLED

                config->mpu_base                = 0x00000000; //(unsigned int) hal->memchunk.mmio_virt_base;
                config->mpu_range               = 0xFFFFF000; //hal->memchunk.sizebytes;

                status = GSL_SUCCESS;
                break;
            }

            default:

                break;
        }
    }

    return (status);
}

//----------------------------------------------------------------------------
//
// kgsl_hal_getchipid
//
// The proper platform method, build from RBBM_PERIPHIDx and RBBM_PATCH_RELEASE
//
KGSLHAL_API gsl_chipid_t
kgsl_hal_getchipid(gsl_deviceid_t device_id)
{
    gsl_chipid_t chipid;
    unsigned int coreid, majorid, minorid, patchid, revid;

    // YDX
    kgsl_device_regread(device_id, mmRBBM_PERIPHID1, &coreid);
    coreid &= 0xF;

    // 2.
    kgsl_device_regread(device_id, mmRBBM_PERIPHID2, &majorid);
    majorid = (majorid >> 4) & 0xF;

    kgsl_device_regread(device_id, mmRBBM_PATCH_RELEASE, &revid);
    
    //   2.
    minorid = ((revid >> 0)  & 0xFF); // this is a 16bit field, but extremely unlikely it would ever get this high
    
    //     1
    patchid = ((revid >> 16) & 0xFF);

    chipid = ((coreid << 24) | (majorid << 16) | (minorid << 8) | (patchid << 0)); 

    return (chipid);
}

//----------------------------------------------------------------------------

KGSLHAL_API int 
kgsl_hal_getplatformtype(char *platform)
{
    if (gsl_driver.hal)
    {
        kos_strcpy(platform, GSL_HAL_PLATFORM);
        return (GSL_SUCCESS);
    }
    else
    {
        return (GSL_FAILURE_NOTINITIALIZED);
    }
}

//---------------------------------------------------------------------------

KGSLHAL_API int
kgsl_hal_setpowerstate(gsl_deviceid_t device_id, int state, unsigned int value)
{
	struct clk *gpu_clk = 0;
	struct clk *garb_clk = clk_get(0, "garb_clk");
	struct clk *emi_garb_clk = clk_get(0, "emi_garb_clk");

    // unreferenced formal parameters
    (void) value;
	
	switch (device_id)
	{
	case GSL_DEVICE_G12:
		gpu_clk = clk_get(0, "gpu2d_clk");
		break;
	case GSL_DEVICE_YAMATO:
		gpu_clk = clk_get(0, "gpu3d_clk");
		break;
	default:
		return (GSL_FAILURE_DEVICEERROR);
	}
	
	if (!gpu_clk)
		return (GSL_FAILURE_DEVICEERROR);

    switch (state)
    {
	case GSL_PWRFLAGS_CLK_ON:
		break;
	case GSL_PWRFLAGS_POWER_ON:
		clk_enable(gpu_clk);
		clk_enable(garb_clk);
		clk_enable(emi_garb_clk);
		kgsl_device_autogate_init(&gsl_driver.device[device_id-1]);
		break;
	case GSL_PWRFLAGS_CLK_OFF:
		break;
	case GSL_PWRFLAGS_POWER_OFF:
		if (kgsl_device_idle(device_id, GSL_TIMEOUT_DEFAULT) != GSL_SUCCESS)
		{
			return (GSL_FAILURE_DEVICEERROR);
		}
		kgsl_device_autogate_exit(&gsl_driver.device[device_id-1]);
		clk_disable(gpu_clk);
		clk_disable(garb_clk);
		clk_disable(emi_garb_clk);
		break;
	default:
		break;
    }
	
    return (GSL_SUCCESS);
}

KGSLHAL_API int kgsl_clock(gsl_deviceid_t dev, int enable)
{
	struct clk *gpu_clk;
	struct clk *garb_clk = clk_get(0, "garb_clk");
	struct clk *emi_garb_clk = clk_get(0, "emi_garb_clk");

	switch (dev)
	{
	case GSL_DEVICE_G12:
		gpu_clk = clk_get(0, "gpu2d_clk");
		break;
	case GSL_DEVICE_YAMATO:
		gpu_clk = clk_get(0, "gpu3d_clk");
		break;
	default:
		printk(KERN_ERR "GPU device %d is invalid!\n", dev);
		return (GSL_FAILURE_DEVICEERROR);
	}

	if (IS_ERR(gpu_clk)) {
		printk(KERN_ERR "%s: GPU clock get failed!\n", __func__);
		return (GSL_FAILURE_DEVICEERROR);
	}

	if (enable) {
		clk_enable(gpu_clk);
		clk_enable(garb_clk);
		clk_enable(emi_garb_clk);
	} else {
		clk_disable(gpu_clk);
		clk_disable(garb_clk);
		clk_disable(emi_garb_clk);
	}

	return (GSL_SUCCESS);
}
