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

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "gsl_linux_map.h"

struct gsl_linux_map
{
	struct list_head list;
	unsigned int gpu_addr;
	void *kernel_virtual_addr;
	unsigned int size;
};

static LIST_HEAD(gsl_linux_map_list);
static DEFINE_MUTEX(gsl_linux_map_mutex);

int gsl_linux_map_init()
{
	mutex_lock(&gsl_linux_map_mutex);
	INIT_LIST_HEAD(&gsl_linux_map_list);
	mutex_unlock(&gsl_linux_map_mutex);

	return 0;
}

void *gsl_linux_map_alloc(unsigned int gpu_addr, unsigned int size)
{
	struct gsl_linux_map * map;
	struct list_head *p;
	void *va;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			mutex_unlock(&gsl_linux_map_mutex);
			return map->kernel_virtual_addr;
		}
	}

	va = __vmalloc(size, GFP_KERNEL, pgprot_writecombine(pgprot_kernel));
	if(va == NULL){
		mutex_unlock(&gsl_linux_map_mutex);
		return NULL;
	}

	map = (struct gsl_linux_map *)kmalloc(sizeof(*map), GFP_KERNEL);
	map->gpu_addr = gpu_addr;
	map->kernel_virtual_addr = va;
	map->size = size;

	INIT_LIST_HEAD(&map->list);
	list_add_tail(&map->list, &gsl_linux_map_list);

	mutex_unlock(&gsl_linux_map_mutex);
	return va;
}

void gsl_linux_map_free(unsigned int gpu_addr)
{
	int found = 0;
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			found = 1;
			break;
		}
	}

	if(found){
		vfree(map->kernel_virtual_addr);
		list_del(&map->list);
		kfree(map);
	}

	mutex_unlock(&gsl_linux_map_mutex);
}

void *gsl_linux_map_find(unsigned int gpu_addr)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr == gpu_addr){
			mutex_unlock(&gsl_linux_map_mutex);
			return map->kernel_virtual_addr;
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_read(void *dst, unsigned int gpuoffset, unsigned int sizebytes, unsigned int touserspace)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset && 
			(map->gpu_addr +  map->size) > gpuoffset){
			void *src = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
                        if (touserspace)
                        {
                            return (void *)copy_to_user(dst, map->kernel_virtual_addr + gpuoffset - map->gpu_addr, sizebytes);
                        }
                        else
                        {
	                    return memcpy(dst, src, sizebytes);
                        }
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_write(void *src, unsigned int gpuoffset, unsigned int sizebytes, unsigned int fromuserspace)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset && 
			(map->gpu_addr +  map->size) > gpuoffset){
			void *dst = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
                        if (fromuserspace)
                        {
                            return (void *)copy_from_user(map->kernel_virtual_addr + gpuoffset - map->gpu_addr, src, sizebytes);
                        }
                        else
                        {
                            return memcpy(dst, src, sizebytes);
                        }
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

void *gsl_linux_map_set(unsigned int gpuoffset, unsigned int value, unsigned int sizebytes)
{
	struct gsl_linux_map * map;
	struct list_head *p;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each(p, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		if(map->gpu_addr <= gpuoffset && 
			(map->gpu_addr +  map->size) > gpuoffset){
			void *ptr = map->kernel_virtual_addr + (gpuoffset - map->gpu_addr);
			mutex_unlock(&gsl_linux_map_mutex);
			return memset(ptr, value, sizebytes);
		}
	}

	mutex_unlock(&gsl_linux_map_mutex);
	return NULL;
}

int gsl_linux_map_destroy()
{
	struct gsl_linux_map * map;
	struct list_head *p, *tmp;

	mutex_lock(&gsl_linux_map_mutex);
	
	list_for_each_safe(p, tmp, &gsl_linux_map_list){
		map = list_entry(p, struct gsl_linux_map, list);
		vfree(map->kernel_virtual_addr);
		list_del(&map->list);
		kfree(map);
	}

	INIT_LIST_HEAD(&gsl_linux_map_list);

	mutex_unlock(&gsl_linux_map_mutex);
	return 0;
}
