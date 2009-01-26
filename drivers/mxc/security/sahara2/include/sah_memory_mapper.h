/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/**
* @file sah_memory_mapper.h
*
* @brief Re-creates SAHARA data structures in Kernel memory such that they are
*       suitable for DMA.
*
*/

#ifndef SAH_MEMORY_MAPPER_H
#define SAH_MEMORY_MAPPER_H

#include <sah_driver_common.h>
#include <sah_queue_manager.h>


/******************************************************************************
* External function declarations
******************************************************************************/
sah_Head_Desc *sah_Copy_Descriptors(fsl_shw_uco_t * user_ctx,
					sah_Head_Desc * desc);

sah_Link *sah_Copy_Links(fsl_shw_uco_t * user_ctx, sah_Link * ptr);

sah_Head_Desc *sah_Physicalise_Descriptors(sah_Head_Desc * desc);

sah_Link *sah_Physicalise_Links (sah_Link *ptr);

sah_Head_Desc *sah_DePhysicalise_Descriptors (sah_Head_Desc *desc);

sah_Link *sah_DePhysicalise_Links (sah_Link *ptr);

sah_Link *sah_Make_Links(fsl_shw_uco_t * user_ctx,
			 sah_Link * ptr, sah_Link ** tail);


void sah_Destroy_Descriptors (sah_Head_Desc *desc);

void sah_Destroy_Links (sah_Link *link);

void sah_Free_Chained_Descriptors (sah_Head_Desc *desc);

void sah_Free_Chained_Links (sah_Link *link);

int sah_Init_Mem_Map (void);

void sah_Stop_Mem_Map (void);

int sah_Block_Add_Page (int big);

sah_Desc *sah_Alloc_Descriptor (void);
sah_Head_Desc *sah_Alloc_Head_Descriptor (void);
void sah_Free_Descriptor (sah_Desc *desc);
void sah_Free_Head_Descriptor (sah_Head_Desc *desc);
sah_Link *sah_Alloc_Link (void);
void sah_Free_Link (sah_Link *link);

void *wire_user_memory(void *address, uint32_t length, void **page_ctx);
void unwire_user_memory(void **page_ctx);

os_error_code map_user_memory(struct vm_area_struct *vma,
			      uint32_t physical_addr, uint32_t size);
os_error_code unmap_user_memory(uint32_t user_addr, uint32_t size);

#endif  /* SAH_MEMORY_MAPPER_H */

/* End of sah_memory_mapper.h */
