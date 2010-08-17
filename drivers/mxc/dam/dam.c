/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file dam.c
 * @brief This is the brief documentation for this dam.c file.
 *
 * This file contains the implementation of the DAM driver main services
 *
 * @ingroup DAM
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "dam.h"

/*!
 * This include to define bool type, false and true definitions.
 */
#include <mach/hardware.h>

#define ModifyRegister32(a, b, c)	(c = (((c)&(~(a))) | (b)))

#define DAM_VIRT_BASE_ADDR	IO_ADDRESS(AUDMUX_BASE_ADDR)

#ifndef _reg_DAM_PTCR1
#define    _reg_DAM_PTCR1   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x00)))
#endif

#ifndef _reg_DAM_PDCR1
#define    _reg_DAM_PDCR1  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x04)))
#endif

#ifndef _reg_DAM_PTCR2
#define    _reg_DAM_PTCR2   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x08)))
#endif

#ifndef _reg_DAM_PDCR2
#define    _reg_DAM_PDCR2  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x0C)))
#endif

#ifndef _reg_DAM_PTCR3
#define    _reg_DAM_PTCR3   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x10)))
#endif

#ifndef _reg_DAM_PDCR3
#define    _reg_DAM_PDCR3  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x14)))
#endif

#ifndef _reg_DAM_PTCR4
#define    _reg_DAM_PTCR4   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x18)))
#endif

#ifndef _reg_DAM_PDCR4
#define    _reg_DAM_PDCR4  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x1C)))
#endif

#ifndef _reg_DAM_PTCR5
#define    _reg_DAM_PTCR5   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x20)))
#endif

#ifndef _reg_DAM_PDCR5
#define    _reg_DAM_PDCR5  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x24)))
#endif

#ifndef _reg_DAM_PTCR6
#define    _reg_DAM_PTCR6   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x28)))
#endif

#ifndef _reg_DAM_PDCR6
#define    _reg_DAM_PDCR6  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x2C)))
#endif

#ifndef _reg_DAM_PTCR7
#define    _reg_DAM_PTCR7   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x30)))
#endif

#ifndef _reg_DAM_PDCR7
#define    _reg_DAM_PDCR7  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x34)))
#endif

#ifndef _reg_DAM_CNMCR
#define    _reg_DAM_CNMCR   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x38)))
#endif

#ifndef _reg_DAM_PTCR
#define    _reg_DAM_PTCR(a)   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + a*8)))
#endif

#ifndef _reg_DAM_PDCR
#define    _reg_DAM_PDCR(a)   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 4 + a*8)))
#endif

/*!
 * PTCR Registers bit shift definitions
 */
#define dam_synchronous_mode_shift               11
#define dam_receive_clock_select_shift           12
#define dam_receive_clock_direction_shift        16
#define dam_receive_frame_sync_select_shift      17
#define dam_receive_frame_sync_direction_shift   21
#define dam_transmit_clock_select_shift          22
#define dam_transmit_clock_direction_shift       26
#define dam_transmit_frame_sync_select_shift     27
#define dam_transmit_frame_sync_direction_shift  31
#define dam_selection_mask                      0xF

/*!
 * HPDCR Register bit shift definitions
 */
#define dam_internal_network_mode_shift           0
#define dam_mode_shift                            8
#define dam_transmit_receive_switch_shift        12
#define dam_receive_data_select_shift            13

/*!
 * HPDCR Register bit masq definitions
 */
#define dam_mode_masq                          0x03
#define dam_internal_network_mode_mask         0xFF

/*!
 * CNMCR Register bit shift definitions
 */
#define dam_ce_bus_port_cntlow_shift              0
#define dam_ce_bus_port_cnthigh_shift             8
#define dam_ce_bus_port_clkpol_shift             16
#define dam_ce_bus_port_fspol_shift              17
#define dam_ce_bus_port_enable_shift             18

#define DAM_NAME   "dam"

EXPORT_SYMBOL(dam_select_mode);
EXPORT_SYMBOL(dam_select_RxClk_direction);
EXPORT_SYMBOL(dam_select_RxClk_source);
EXPORT_SYMBOL(dam_select_RxD_source);
EXPORT_SYMBOL(dam_select_RxFS_direction);
EXPORT_SYMBOL(dam_select_RxFS_source);
EXPORT_SYMBOL(dam_select_TxClk_direction);
EXPORT_SYMBOL(dam_select_TxClk_source);
EXPORT_SYMBOL(dam_select_TxFS_direction);
EXPORT_SYMBOL(dam_select_TxFS_source);
EXPORT_SYMBOL(dam_set_internal_network_mode_mask);
EXPORT_SYMBOL(dam_set_synchronous);
EXPORT_SYMBOL(dam_switch_Tx_Rx);
EXPORT_SYMBOL(dam_reset_register);

/*!
 * This function selects the operation mode of the port.
 *
 * @param        port              the DAM port to configure
 * @param        the_mode          the operation mode of the port
 *
 * @return       This function returns the result of the operation
 *               (0 if successful, -1 otherwise).
 */
int dam_select_mode(dam_port port, dam_mode the_mode)
{
	int result;
	result = 0;

	ModifyRegister32(dam_mode_masq << dam_mode_shift,
			 the_mode << dam_mode_shift, _reg_DAM_PDCR(port));

	return result;
}

/*!
 * This function controls Receive clock signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Rx clock signal direction
 */
void dam_select_RxClk_direction(dam_port port, signal_direction direction)
{
	ModifyRegister32(1 << dam_receive_clock_direction_shift,
			 direction << dam_receive_clock_direction_shift,
			 _reg_DAM_PTCR(port));
}

/*!
 * This function controls Receive clock signal source for the port.
 *
 * @param        p_config          the DAM port to configure
 * @param        from_RxClk        the signal comes from RxClk or TxClk of
 *                                 the source port
 * @param        p_source          the source port
 */
void dam_select_RxClk_source(dam_port p_config,
			     bool from_RxClk, dam_port p_source)
{
	ModifyRegister32(dam_selection_mask << dam_receive_clock_select_shift,
			 ((from_RxClk << 3) | p_source) <<
			 dam_receive_clock_select_shift,
			 _reg_DAM_PTCR(p_config));
}

/*!
 * This function selects the source port for the RxD data.
 *
 * @param        p_config          the DAM port to configure
 * @param        p_source          the source port
 */
void dam_select_RxD_source(dam_port p_config, dam_port p_source)
{
	ModifyRegister32(dam_selection_mask << dam_receive_data_select_shift,
			 p_source << dam_receive_data_select_shift,
			 _reg_DAM_PDCR(p_config));
}

/*!
 * This function controls Receive Frame Sync signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Rx Frame Sync signal direction
 */
void dam_select_RxFS_direction(dam_port port, signal_direction direction)
{
	ModifyRegister32(1 << dam_receive_frame_sync_direction_shift,
			 direction << dam_receive_frame_sync_direction_shift,
			 _reg_DAM_PTCR(port));
}

/*!
 * This function controls Receive Frame Sync signal source for the port.
 *
 * @param        p_config          the DAM port to configure
 * @param        from_RxFS         the signal comes from RxFS or TxFS of
 *                                 the source port
 * @param        p_source          the source port
 */
void dam_select_RxFS_source(dam_port p_config,
			    bool from_RxFS, dam_port p_source)
{
	ModifyRegister32(dam_selection_mask <<
			 dam_receive_frame_sync_select_shift,
			 ((from_RxFS << 3) | p_source) <<
			 dam_receive_frame_sync_select_shift,
			 _reg_DAM_PTCR(p_config));
}

/*!
 * This function controls Transmit clock signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Tx clock signal direction
 */
void dam_select_TxClk_direction(dam_port port, signal_direction direction)
{
	ModifyRegister32(1 << dam_transmit_clock_direction_shift,
			 direction << dam_transmit_clock_direction_shift,
			 _reg_DAM_PTCR(port));
}

/*!
 * This function controls Transmit clock signal source for the port.
 *
 * @param        p_config          the DAM port to configure
 * @param        from_RxClk        the signal comes from RxClk or TxClk of
 *                                 the source port
 * @param        p_source          the source port
 */
void dam_select_TxClk_source(dam_port p_config,
			     bool from_RxClk, dam_port p_source)
{
	ModifyRegister32(dam_selection_mask << dam_transmit_clock_select_shift,
			 ((from_RxClk << 3) | p_source) <<
			 dam_transmit_clock_select_shift,
			 _reg_DAM_PTCR(p_config));
}

/*!
 * This function controls Transmit Frame Sync signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Tx Frame Sync signal direction
 */
void dam_select_TxFS_direction(dam_port port, signal_direction direction)
{
	ModifyRegister32(1 << dam_transmit_frame_sync_direction_shift,
			 direction << dam_transmit_frame_sync_direction_shift,
			 _reg_DAM_PTCR(port));
}

/*!
 * This function controls Transmit Frame Sync signal source for the port.
 *
 * @param        p_config          the DAM port to configure
 * @param        from_RxFS         the signal comes from RxFS or TxFS of
 *                                 the source port
 * @param        p_source          the source port
 */
void dam_select_TxFS_source(dam_port p_config,
			    bool from_RxFS, dam_port p_source)
{
	ModifyRegister32(dam_selection_mask <<
			 dam_transmit_frame_sync_select_shift,
			 ((from_RxFS << 3) | p_source) <<
			 dam_transmit_frame_sync_select_shift,
			 _reg_DAM_PTCR(p_config));
}

/*!
 * This function sets a bit mask that selects the port from which of the RxD
 * signals are to be ANDed together for internal network mode.
 * Bit 6 represents RxD from Port7 and bit0 represents RxD from Port1.
 * 1 excludes RxDn from ANDing. 0 includes RxDn for ANDing.
 *
 * @param        port              the DAM port to configure
 * @param        bit_mask          the bit mask
 *
 * @return       This function returns the result of the operation
 *               (0 if successful, -1 otherwise).
 */
int dam_set_internal_network_mode_mask(dam_port port, unsigned char bit_mask)
{
	int result;
	result = 0;

	ModifyRegister32(dam_internal_network_mode_mask <<
			 dam_internal_network_mode_shift,
			 bit_mask << dam_internal_network_mode_shift,
			 _reg_DAM_PDCR(port));

	return result;
}

/*!
 * This function controls whether or not the port is in synchronous mode.
 * When the synchronous mode is selected, the receive and the transmit sections
 * use common clock and frame sync signals.
 * When the synchronous mode is not selected, separate clock and frame sync
 * signals are used for the transmit and the receive sections.
 * The defaut value is the synchronous mode selected.
 *
 * @param        port              the DAM port to configure
 * @param        synchronous       the state to assign
 */
void dam_set_synchronous(dam_port port, bool synchronous)
{
	ModifyRegister32(1 << dam_synchronous_mode_shift,
			 synchronous << dam_synchronous_mode_shift,
			 _reg_DAM_PTCR(port));
}

/*!
 * This function swaps the transmit and receive signals from (Da-TxD, Db-RxD)
 * to (Da-RxD, Db-TxD).
 * This default signal configuration is Da-TxD, Db-RxD.
 *
 * @param        port              the DAM port to configure
 * @param        value             the switch state
 */
void dam_switch_Tx_Rx(dam_port port, bool value)
{
	ModifyRegister32(1 << dam_transmit_receive_switch_shift,
			 value << dam_transmit_receive_switch_shift,
			 _reg_DAM_PDCR(port));
}

/*!
 * This function resets the two registers of the selected port.
 *
 * @param        port              the DAM port to reset
 */
void dam_reset_register(dam_port port)
{
	ModifyRegister32(0xFFFFFFFF, 0x00000000, _reg_DAM_PTCR(port));
	ModifyRegister32(0xFFFFFFFF, 0x00000000, _reg_DAM_PDCR(port));
}

/*!
 * This function implements the init function of the DAM device.
 * This function is called when the module is loaded.
 *
 * @return       This function returns 0.
 */
static int __init dam_init(void)
{
	return 0;
}

/*!
 * This function implements the exit function of the SPI device.
 * This function is called when the module is unloaded.
 *
 */
static void __exit dam_exit(void)
{
}

module_init(dam_init);
module_exit(dam_exit);

MODULE_DESCRIPTION("DAM char device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
