/*
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_MXC_PCMCIA_H__
#define __ASM_ARCH_MXC_PCMCIA_H__

#include <mach/hardware.h>

#define	WINDOW_SIZE	0x1000000	/* The size of a window: 16M    */
#define PCMCIA_WINDOWS  5	/* How many windows / socket    */
#define SOCKET_NO	1	/* How many sockets             */

#define ATTRIBUTE_MEMORY_WINDOW	0
#define IO_WINDOW		1
#define COMMON_MEMORY_WINDOW	2

/*
 * PCMCIA socket
 */
#define	PCMCIAPrtSp	WINDOW_SIZE	/* PCMCIA window size              */
#define PCMCIASp	(4*PCMCIAPrtSp)	/* PCMCIA Space [byte]             */
#define PCMCIAIOSp	PCMCIAPrtSp	/* PCMCIA I/O Space [byte]         */
#define PCMCIAAttrSp	PCMCIAPrtSp	/* PCMCIA Attribute Space [byte]   */
#define PCMCIAMemSp	PCMCIAPrtSp	/* PCMCIA Memory Space [byte]      */

#define PCMCIA0Sp	PCMCIASp	/* PCMCIA 0 Space [byte]           */
#define PCMCIA0IOSp	PCMCIAIOSp	/* PCMCIA 0 I/O Space [byte]       */
#define PCMCIA0AttrSp	PCMCIAAttrSp	/* PCMCIA 0 Attribute Space [byte] */
#define PCMCIA0MemSp	PCMCIAMemSp	/* PCMCIA 0 Memory Space [byte]    */

#define _PCMCIA(Nb)			/* PCMCIA [0..1]		   */ \
			(PCMCIA_MEM_BASE_ADDR + (Nb) * PCMCIASp)

#define _PCMCIAAttr(Nb)	_PCMCIA (Nb)	/* PCMCIA I/O [0..1]               */

#define _PCMCIAIO(Nb)			/* PCMCIA Attribute [0..1]	   */ \
			(_PCMCIA (Nb) + (IO_WINDOW) * PCMCIAPrtSp)
#define _PCMCIAMem(Nb)			/* PCMCIA Memory [0..1]		   */ \
			(_PCMCIA (Nb) + (COMMON_MEMORY_WINDOW) * PCMCIAPrtSp)

#define _PCMCIA0	_PCMCIA (0)	/* PCMCIA 0                        */
#define _PCMCIA0IO	_PCMCIAIO (0)	/* PCMCIA 0 I/O                    */
#define _PCMCIA0Attr	_PCMCIAAttr (0)	/* PCMCIA 0 Attribute              */
#define _PCMCIA0Mem	_PCMCIAMem (0)	/* PCMCIA 0 Memory                 */

/*
 * Module: PCMCIA, Addr Range: 0xB8004000 - 0xB8004FFF, Size: 4 Kbyte
 */
#define PCMCIA_BASE_ADDR	(PCMCIA_CTL_BASE_ADDR)	/* PCMCIA Base Address */
#define PCMCIA_IO_ADDR(x)	(*(volatile u32 *)PCMCIA_IO_ADDRESS(x))

#define _reg_PCMCIA_PIPR	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x00)	/* PCMCIA input pins register */
#define _reg_PCMCIA_PSCR	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x04)	/* PCMCIA Status Changed Register */
#define _reg_PCMCIA_PER		PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x08)	/* PCMCIA Enable Register */

/* win: 0-4 */
#define _reg_PCMCIA_PBR(win)	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x0C + 4 * (win))	/* PCMCIA Base Register x */
#define _reg_PCMCIA_POR(win)	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x28 + 4 * (win))	/* PCMCIA Option Register x */
#define _reg_PCMCIA_POFR(win)	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x44 + 4 * (win))	/* PCMCIA Offset Register x */

#define _reg_PCMCIA_PGCR	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x60)	/* PCMCIA General Control Register */
#define _reg_PCMCIA_PGSR	PCMCIA_IO_ADDR(PCMCIA_BASE_ADDR + 0x64)	/* PCMCIA General Status Register */

/* PCMCIA_PIPR - PCMCIA Input Pins Register - fields */
#define PCMCIA_PIPR_POWERON            (1 <<  8)	/* card indicates "power on" */
#define PCMCIA_PIPR_RDY                (1 <<  7)	/* card is ready */
#define PCMCIA_PIPR_BVD2               (1 <<  6)	/* battery voltage 2/SPKR in */
#define PCMCIA_PIPR_BVD1               (1 <<  5)	/* battery voltage 1/STSCHG */
#define PCMCIA_PIPR_CD                 (3 <<  3)	/* card detect 1 and 2 */
#define PCMCIA_PIPR_WP                 (1 <<  2)	/* write protect switch enabled */
#define PCMCIA_PIPR_VS                 (3 <<  0)	/* voltage sense bits */
#define PCMCIA_PIPR_VS_5V              (1 <<  0)	/* 5v */

/* PCMCIA_PSCR - PCMCIA Status Change Register - fields */
#define PCMCIA_PSCR_POWC               (1 << 11)	/*  */
#define PCMCIA_PSCR_RDYR               (1 << 10)	/*  */
#define PCMCIA_PSCR_RDYF               (1 <<  9)	/*  */
#define PCMCIA_PSCR_RDYH               (1 <<  8)	/*  */
#define PCMCIA_PSCR_RDYL               (1 <<  7)	/*  */
#define PCMCIA_PSCR_BVDC2              (1 <<  6)	/*  */
#define PCMCIA_PSCR_BVDC1              (1 <<  5)	/*  */
#define PCMCIA_PSCR_CDC2               (1 <<  4)	/*  */
#define PCMCIA_PSCR_CDC1               (1 <<  3)	/*  */
#define PCMCIA_PSCR_WPC                (1 <<  2)	/*  */
#define PCMCIA_PSCR_VSC2               (1 <<  1)	/*  */
#define PCMCIA_PSCR_VSC1               (1 <<  0)	/*  */

/* PCMCIA_PER - PCMCIA Enable Register - fields */
#define PCMCIA_PER_ERRINTEN            (1 << 12)	/* error interrupt enable */
#define PCMCIA_PER_POWERONEN           (1 << 11)	/* power on interrupt enable */
#define PCMCIA_PER_RDYRE               (1 << 10)	/* RDY/nIREQ pin rising edge */
#define PCMCIA_PER_RDYFE               (1 <<  9)	/* RDY/nIREQ pin falling edge */
#define PCMCIA_PER_RDYHE               (1 <<  8)	/* RDY/nIREQ pin high */
#define PCMCIA_PER_RDYLE               (1 <<  7)	/* RDY/nIREQ pin low */
#define PCMCIA_PER_BVDE2               (1 <<  6)	/* battery voltage 2/SPKR in */
#define PCMCIA_PER_BVDE1               (1 <<  5)	/* battery voltage 1/STSCHG */
#define PCMCIA_PER_CDE2                (1 <<  4)	/* card detect 2  */
#define PCMCIA_PER_CDE1                (1 <<  3)	/* card detect 1 */
#define PCMCIA_PER_WPE                 (1 <<  2)	/* write protect */
#define PCMCIA_PER_VSE2                (1 <<  1)	/* voltage sense 2 */
#define PCMCIA_PER_VSE1                (1 <<  0)	/* voltage sense 1 */

/* PCMCIA_POR[0-4] - PCMCIA Option Registers 0-4 - fields */
#define PCMCIA_POR_PV                  (1 << 29)	/* set iff bank is valid */
#define PCMCIA_POR_WPEN                (1 << 28)	/* write protect (WP) input signal is enabled */
#define PCMCIA_POR_WP                  (1 << 27)	/* write protected */

#define PCMCIA_POR_PRS_SHIFT           (25)
#define PCMCIA_POR_PRS(x)              (((x) & 0x3) << PCMCIA_POR_PRS_SHIFT)
#define PCMCIA_POR_PRS_MASK            PCMCIA_POR_PRS(3)	/* PCMCIA region select */
#define PCMCIA_POR_PRS_COMMON          (0)	/* values of POR_PRS field */
#define PCMCIA_POR_PRS_TRUE_IDE        (1)
#define PCMCIA_POR_PRS_ATTRIBUTE       (2)
#define PCMCIA_POR_PRS_IO              (3)

#define PCMCIA_POR_PPS_8               (1 << 24)	/* PCMCIA Port size =  8bits */
#define PCMCIA_POR_PPS_16              (0 << 24)	/* PCMCIA Port size = 16bits */

#define PCMCIA_POR_PSL_SHIFT           (17)	/* strobe length */
#define PCMCIA_POR_PSL(x)              (((x) & 0x7F) << PCMCIA_POR_PSL_SHIFT)
#define PCMCIA_POR_PSL_MASK            PCMCIA_POR_PSL(0x7f)

#define PCMCIA_POR_PSST_SHIFT          (11)	/* strobe setup time */
#define PCMCIA_POR_PSST(x)             (((x) & 0x3F) << PCMCIA_POR_PSST_SHIFT)
#define PCMCIA_POR_PSST_MASK           PCMCIA_POR_PSST(0x3f)

#define PCMCIA_POR_PSHT_SHIFT          (5)	/* strobe hold time  */
#define PCMCIA_POR_PSHT(x)             (((x) & 0x3F) << PCMCIA_POR_PSHT_SHIFT)
#define PCMCIA_POR_PSHT_MASK           PCMCIA_POR_PSHT(0x3f)

#define PCMCIA_POR_BSIZE_SHIFT         (0)	/* bank size */
#define PCMCIA_POR_BSIZE(x)            (((x) & 0x1F) << PCMCIA_POR_BSIZE_SHIFT)
#define PCMCIA_POR_BSIZE_MASK          PCMCIA_POR_BSIZE(0x1F)

/* some handy BSIZE values */
#define POR_BSIZE_1                    PCMCIA_POR_BSIZE(0x00)
#define POR_BSIZE_2                    PCMCIA_POR_BSIZE(0x01)
#define POR_BSIZE_4                    PCMCIA_POR_BSIZE(0x03)
#define POR_BSIZE_8                    PCMCIA_POR_BSIZE(0x02)
#define POR_BSIZE_16                   PCMCIA_POR_BSIZE(0x06)
#define POR_BSIZE_32                   PCMCIA_POR_BSIZE(0x07)
#define POR_BSIZE_64                   PCMCIA_POR_BSIZE(0x05)
#define POR_BSIZE_128                  PCMCIA_POR_BSIZE(0x04)
#define POR_BSIZE_256                  PCMCIA_POR_BSIZE(0x0C)
#define POR_BSIZE_512                  PCMCIA_POR_BSIZE(0x0D)
#define POR_BSIZE_1K                   PCMCIA_POR_BSIZE(0x0F)
#define POR_BSIZE_2K                   PCMCIA_POR_BSIZE(0x0E)

#define POR_BSIZE_4K                   PCMCIA_POR_BSIZE(0x0A)
#define POR_BSIZE_8K                   PCMCIA_POR_BSIZE(0x0B)
#define POR_BSIZE_16K                  PCMCIA_POR_BSIZE(0x09)
#define POR_BSIZE_32K                  PCMCIA_POR_BSIZE(0x08)
#define POR_BSIZE_64K                  PCMCIA_POR_BSIZE(0x18)
#define POR_BSIZE_128K                 PCMCIA_POR_BSIZE(0x19)
#define POR_BSIZE_256K                 PCMCIA_POR_BSIZE(0x1B)
#define POR_BSIZE_512K                 PCMCIA_POR_BSIZE(0x1A)
#define POR_BSIZE_1M                   PCMCIA_POR_BSIZE(0x1E)
#define POR_BSIZE_2M                   PCMCIA_POR_BSIZE(0x1F)
#define POR_BSIZE_4M                   PCMCIA_POR_BSIZE(0x1D)
#define POR_BSIZE_8M                   PCMCIA_POR_BSIZE(0x1C)
#define POR_BSIZE_16M                  PCMCIA_POR_BSIZE(0x14)
#define POR_BSIZE_32M                  PCMCIA_POR_BSIZE(0x15)
#define POR_BSIZE_64M                  PCMCIA_POR_BSIZE(0x17)

/* Window size */
#define POR_1		0x1
#define POR_2		0x2
#define POR_4		0x4
#define POR_8		0x8
#define POR_16		0x10
#define POR_32		0x20
#define POR_64		0x40
#define POR_128		0x80
#define POR_256		0x100
#define POR_512		0x200

#define POR_1K		0x400
#define POR_2K		0x800
#define POR_4K		0x1000
#define POR_8K		0x2000
#define POR_16K		0x4000
#define POR_32K		0x8000
#define POR_64K		0x10000
#define POR_128K	0x20000
#define POR_256K	0x40000
#define POR_512K	0x80000

#define POR_1M		0x100000
#define POR_2M		0x200000
#define POR_4M		0x400000
#define POR_8M		0x800000
#define POR_16M		0x1000000
#define POR_32M		0x2000000
#define POR_64M		0x4000000

/* PCMCIA_PGCR - PCMCIA General Control Register - fields */
#define PCMCIA_PGCR_LPMEN              (1 <<  3)	/* Low power Mode Enable */
#define PCMCIA_PGCR_SPKREN             (1 <<  2)	/* SPKROUT routing enable */
#define PCMCIA_PGCR_POE                (1 <<  1)	/* Controller out enable */
#define PCMCIA_PGCR_RESET              (1 <<  0)	/* Card reset */

/* PCMCIA_PGSR - PCMCIA General Status Register - fields */
#define PCMCIA_PGSR_NWINE              (1 <<  4)	/* No Window error */
#define PCMCIA_PGSR_LPE                (1 <<  3)	/* Low Power error */
#define PCMCIA_PGSR_SE                 (1 <<  2)	/* Size error */
#define PCMCIA_PGSR_CDE                (1 <<  1)	/* Card Detect error */
#define PCMCIA_PGSR_WPE                (1 <<  0)	/* Write Protect error */

#endif				/* __ASM_ARCH_MXC_PCMCIA_H__ */
