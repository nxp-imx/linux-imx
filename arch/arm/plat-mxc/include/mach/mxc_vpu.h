/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @defgroup VPU Video Processor Unit Driver
 */

/*!
 * @file arch-mxc/mxc_vpu.h
 *
 * @brief VPU system initialization and file operation definition
 *
 * @ingroup VPU
 */

#ifndef __ASM_ARCH_MXC_VPU_H__
#define __ASM_ARCH_MXC_VPU_H__

#include <linux/fs.h>

struct vpu_mem_desc {
	u32 size;
	dma_addr_t phy_addr;
	u32 cpu_addr;		/* cpu address to free the dma mem */
	u32 virt_uaddr;		/* virtual user space address */
};

#define VPU_IOC_MAGIC  'V'

#define VPU_IOC_PHYMEM_ALLOC	_IO(VPU_IOC_MAGIC, 0)
#define VPU_IOC_PHYMEM_FREE	_IO(VPU_IOC_MAGIC, 1)
#define VPU_IOC_WAIT4INT	_IO(VPU_IOC_MAGIC, 2)
#define VPU_IOC_PHYMEM_DUMP	_IO(VPU_IOC_MAGIC, 3)
#define VPU_IOC_REG_DUMP	_IO(VPU_IOC_MAGIC, 4)
#define VPU_IOC_VL2CC_FLUSH	_IO(VPU_IOC_MAGIC, 5)
#define VPU_IOC_IRAM_SETTING	_IO(VPU_IOC_MAGIC, 6)
#define VPU_IOC_CLKGATE_SETTING	_IO(VPU_IOC_MAGIC, 7)
#define VPU_IOC_GET_WORK_ADDR   _IO(VPU_IOC_MAGIC, 8)
#define VPU_IOC_GET_PIC_PARA_ADDR   _IO(VPU_IOC_MAGIC, 9)
#define VPU_IOC_GET_USER_DATA_ADDR   _IO(VPU_IOC_MAGIC, 10)

#define BIT_CODE_RUN			0x000
#define BIT_CODE_DOWN			0x004
#define	BIT_INT_CLEAR			0x00C
#define	BIT_INT_STATUS			0x010

#define BIT_WORK_CTRL_BUF_BASE		0x100
#define BIT_WORK_CTRL_BUF_REG(i)	(BIT_WORK_CTRL_BUF_BASE + i * 4)
#define BIT_CODE_BUF_ADDR		BIT_WORK_CTRL_BUF_REG(0)
#define BIT_WORK_BUF_ADDR		BIT_WORK_CTRL_BUF_REG(1)
#define BIT_PARA_BUF_ADDR		BIT_WORK_CTRL_BUF_REG(2)
#define BIT_BIT_STREAM_CTRL		BIT_WORK_CTRL_BUF_REG(3)
#define BIT_FRAME_MEM_CTRL		BIT_WORK_CTRL_BUF_REG(4)
#define BIT_BIT_STREAM_PARAM		BIT_WORK_CTRL_BUF_REG(5)

#define BIT_RESET_CTRL			0x11C

/* i could be 0, 1, 2, 3 */
#define	BIT_RD_PTR_BASE			0x120
#define BIT_RD_PTR_REG(i)		(BIT_RD_PTR_BASE + i * 8)
#define BIT_WR_PTR_REG(i)		(BIT_RD_PTR_BASE + i * 8 + 4)

/* i could be 0, 1, 2, 3 */
#define BIT_FRM_DIS_FLG_BASE		(cpu_is_mx51() ? 0x150 : 0x140)
#define	BIT_FRM_DIS_FLG_REG(i)		(BIT_FRM_DIS_FLG_BASE + i * 4)

#define BIT_BUSY_FLAG			0x160
#define BIT_RUN_COMMAND			0x164
#define BIT_INT_ENABLE			0x170

#define	BITVAL_PIC_RUN			8

#define	VPU_SLEEP_REG_VALUE		10
#define	VPU_WAKE_REG_VALUE		11

int vl2cc_init(u32 vl2cc_hw_base);
void vl2cc_enable(void);
void vl2cc_flush(void);
void vl2cc_disable(void);
void vl2cc_cleanup(void);

#endif
