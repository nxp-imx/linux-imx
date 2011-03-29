/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file sah_hardware_interface.h
*
* @brief Provides an interface to the SAHARA hardware registers.
*
*/

#ifndef SAH_HARDWARE_INTERFACE_H
#define SAH_HARDWARE_INTERFACE_H

#include <sah_driver_common.h>
#include <sah_status_manager.h>

/* These values can be used with sah_HW_Write_Control(). */
#ifdef SAHARA1
/** Define platform as Little-Endian */
#define CTRL_LITTLE_END     0x00000002
/** Bit to cause endian change in transfers */
#define CTRL_INT_EN         0x00000004
/** Set High Assurance mode */
#define CTRL_HA             0x00000008
#else
/** Bit to cause byte swapping in (data?) transfers */
#define CTRL_BYTE_SWAP      0x00000001
/** Bit to cause halfword swapping in (data?) transfers */
#define CTRL_HALFWORD_SWAP  0x00000002
/** Bit to cause endian change in (data?) transfers */
#define CTRL_INT_EN         0x00000010
/** Set High Assurance mode */
#define CTRL_HA             0x00000020
/** Disable High Assurance */
#define CTRL_HA_DISABLE     0x00000040
/** Reseed the RNG CHA */
#define CTRL_RNG_RESEED     0x00000080
#endif


/* These values can be used with sah_HW_Write_Command(). */
/** Reset the Sahara */
#define CMD_RESET           0x00000001
/** Set Sahara into Batch mode. */
#define CMD_BATCH           0x00010000
/** Clear the Sahara interrupt */
#define CMD_CLR_INT_BIT     0x00000100
/** Clear the Sahara error */
#define CMD_CLR_ERROR_BIT   0x00000200


/** error status register contains error */
#define STATUS_ERROR 0x00000010

/** Op status register contains op status */
#define OP_STATUS 0x00000020


/* High Level functions */
int                sah_HW_Reset(void);
fsl_shw_return_t   sah_HW_Set_HA(void);
sah_Execute_Status sah_Wait_On_Sahara(void);

/* Low Level functions */
uint32_t    sah_HW_Read_Version(void);
uint32_t    sah_HW_Read_Control(void);
uint32_t    sah_HW_Read_Status(void);
uint32_t    sah_HW_Read_Error_Status(void);
uint32_t    sah_HW_Read_Op_Status(void);
uint32_t    sah_HW_Read_DAR(void);
uint32_t    sah_HW_Read_CDAR(void);
uint32_t    sah_HW_Read_IDAR(void);
uint32_t    sah_HW_Read_Fault_Address(void);
uint32_t    sah_HW_Read_MM_Status(void);
uint32_t    sah_HW_Read_Config(void);
void        sah_HW_Write_Command(uint32_t command);
void        sah_HW_Write_Control(uint32_t control);
void        sah_HW_Write_DAR(uint32_t pointer);
void        sah_HW_Write_Config(uint32_t configuration);

#if defined DIAG_DRV_IF || defined(DO_DBG)

void sah_Dump_Words(const char *prefix, const unsigned *data, dma_addr_t addr,
		    unsigned length);
#endif

#endif  /* SAH_HARDWARE_INTERFACE_H */

/* End of sah_hardware_interface.c */
