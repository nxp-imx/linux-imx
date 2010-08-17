/*
 *  Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_IO_H__
#define __ASM_ARCH_MXC_IO_H__

/* Allow IO space to be anywhere in the memory */
#define IO_SPACE_LIMIT 0xffffffff

extern void __iomem *__mxc_ioremap(unsigned long cookie, size_t size,
				unsigned int mtype);

#define __arch_ioremap(a, s, f) __mxc_ioremap(a, s, f)
#define __arch_iounmap __iounmap

/* io address mapping macro */
#define __io(a)		__typesafe_io(a)

#define __mem_pci(a)	(a)

/*!
 * This function is called to read a CPLD register over CSPI.
 *
 * @param        offset    number of the cpld register to be read
 *
 * @return       Returns 0 on success -1 on failure.
 */
unsigned int __weak spi_cpld_read(unsigned int offset);

/*!
 * This function is called to write to a CPLD register over CSPI.
 *
 * @param        offset    number of the cpld register to be written
 * @param        reg_val   value to be written
 *
 * @return       Returns 0 on success -1 on failure.
 */
unsigned int __weak spi_cpld_write(unsigned int offset, unsigned int reg_val);
#endif
