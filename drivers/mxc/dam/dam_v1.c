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
 * @file dam_v1.c
 * @brief This is the brief documentation for this dam_v1.c file.
 *
 * This file contains the implementation of the DAM driver main services
 *
 * @ingroup DAM
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include "dam.h"

/*!
 * This include to define bool type, false and true definitions.
 */
#include <mach/hardware.h>

#define DAM_VIRT_BASE_ADDR	IO_ADDRESS(AUDMUX_BASE_ADDR)

#define ModifyRegister32(a, b, c)	do {\
	__raw_writel(((__raw_readl(c)) & (~(a))) | (b), (c));\
} while (0)

#ifndef _reg_DAM_HPCR1
#define    _reg_DAM_HPCR1   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x00)))
#endif

#ifndef _reg_DAM_HPCR2
#define    _reg_DAM_HPCR2   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x04)))
#endif

#ifndef _reg_DAM_HPCR3
#define    _reg_DAM_HPCR3   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x08)))
#endif

#ifndef _reg_DAM_PPCR1
#define    _reg_DAM_PPCR1   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x10)))
#endif

#ifndef _reg_DAM_PPCR2
#define    _reg_DAM_PPCR2  (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x14)))
#endif

#ifndef _reg_DAM_PPCR3
#define    _reg_DAM_PPCR3   (*((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x1c)))
#endif

#ifndef _reg_DAM_HPCR
#define    _reg_DAM_HPCR(a)   ((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + (a)*4))
#endif

#ifndef _reg_DAM_PPCR
#define    _reg_DAM_PPCR(a)   ((volatile unsigned long *) \
		(DAM_VIRT_BASE_ADDR + 0x0c + (0x04 << (a-3))))
#endif

/*!
 * HPCR/PPCR Registers bit shift definitions
 */
#define dam_transmit_frame_sync_direction_shift  31
#define dam_transmit_clock_direction_shift       30
#define dam_transmit_frame_sync_select_shift     26
#define dam_transmit_clock_select_shift          26
#define dam_receive_frame_sync_direction_shift   25
#define dam_receive_clock_direction_shift        24
#define dam_receive_clock_select_shift           20
#define dam_receive_frame_sync_select_shift      20

#define dam_receive_data_select_shift            13
#define dam_synchronous_mode_shift               12

#define dam_transmit_receive_switch_shift        10

#define dam_mode_shift                            8
#define dam_internal_network_mode_shift           0

/*!
 * HPCR/PPCR Register bit masq definitions
 */
/*#define dam_selection_mask              0xF*/
#define dam_fs_selection_mask             0xF
#define dam_clk_selection_mask            0xF
#define dam_dat_selection_mask		        0x7
/*#define dam_mode_masq                   0x03*/
#define dam_internal_network_mode_mask    0xFF

/*!
 * HPCR/PPCR Register reset value definitions
 */
#define dam_hpcr_default_value 0x00001000
#define dam_ppcr_default_value 0x00001000

#define DAM_NAME   "dam"
static struct class *mxc_dam_class;

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
 * DAM major
 */
#ifdef TEST_DAM
static int major_dam;

typedef struct _mxc_cfg {
	int reg;
	int val;
} mxc_cfg;

#endif

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

	if (port >= 3)
		the_mode = normal_mode;
	ModifyRegister32(1 << dam_mode_shift,
			 the_mode << dam_mode_shift, _reg_DAM_HPCR(port));

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
	if (port < 3) {
		ModifyRegister32(1 << dam_receive_clock_direction_shift,
				 direction << dam_receive_clock_direction_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_receive_clock_direction_shift,
				 direction << dam_receive_clock_direction_shift,
				 _reg_DAM_PPCR(port));
	}
	return;
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
	if (p_config < 3) {
		ModifyRegister32(dam_clk_selection_mask <<
				 dam_receive_clock_select_shift,
				 ((from_RxClk << 3) | p_source) <<
				 dam_receive_clock_select_shift,
				 _reg_DAM_HPCR(p_config));
	} else {
		ModifyRegister32(dam_clk_selection_mask <<
				 dam_receive_clock_select_shift,
				 ((from_RxClk << 3) | p_source) <<
				 dam_receive_clock_select_shift,
				 _reg_DAM_PPCR(p_config));
	}
	return;
}

/*!
 * This function selects the source port for the RxD data.
 *
 * @param        p_config          the DAM port to configure
 * @param        p_source          the source port
 */
void dam_select_RxD_source(dam_port p_config, dam_port p_source)
{
	if (p_config < 3) {
		ModifyRegister32(dam_dat_selection_mask <<
				 dam_receive_data_select_shift,
				 p_source << dam_receive_data_select_shift,
				 _reg_DAM_HPCR(p_config));
	} else {
		ModifyRegister32(dam_dat_selection_mask <<
				 dam_receive_data_select_shift,
				 p_source << dam_receive_data_select_shift,
				 _reg_DAM_PPCR(p_config));
	}
	return;
}

/*!
 * This function controls Receive Frame Sync signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Rx Frame Sync signal direction
 */
void dam_select_RxFS_direction(dam_port port, signal_direction direction)
{
	if (port < 3) {
		ModifyRegister32(1 << dam_receive_frame_sync_direction_shift,
				 direction <<
				 dam_receive_frame_sync_direction_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_receive_frame_sync_direction_shift,
				 direction <<
				 dam_receive_frame_sync_direction_shift,
				 _reg_DAM_PPCR(port));
	}
	return;
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
	if (p_config < 3) {
		ModifyRegister32(dam_fs_selection_mask <<
				 dam_receive_frame_sync_select_shift,
				 ((from_RxFS << 3) | p_source) <<
				 dam_receive_frame_sync_select_shift,
				 _reg_DAM_HPCR(p_config));
	} else {
		ModifyRegister32(dam_fs_selection_mask <<
				 dam_receive_frame_sync_select_shift,
				 ((from_RxFS << 3) | p_source) <<
				 dam_receive_frame_sync_select_shift,
				 _reg_DAM_PPCR(p_config));
	}
	return;
}

/*!
 * This function controls Transmit clock signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Tx clock signal direction
 */
void dam_select_TxClk_direction(dam_port port, signal_direction direction)
{
	if (port < 3) {
		ModifyRegister32(1 << dam_transmit_clock_direction_shift,
				 direction <<
				 dam_transmit_clock_direction_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_transmit_clock_direction_shift,
				 direction <<
				 dam_transmit_clock_direction_shift,
				 _reg_DAM_PPCR(port));
	}
	return;
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
	if (p_config < 3) {
		ModifyRegister32(dam_clk_selection_mask <<
				 dam_transmit_clock_select_shift,
				 ((from_RxClk << 3) | p_source) <<
				 dam_transmit_clock_select_shift,
				 _reg_DAM_HPCR(p_config));
	} else {
		ModifyRegister32(dam_clk_selection_mask <<
				 dam_transmit_clock_select_shift,
				 ((from_RxClk << 3) | p_source) <<
				 dam_transmit_clock_select_shift,
				 _reg_DAM_PPCR(p_config));
	}
	return;
}

/*!
 * This function controls Transmit Frame Sync signal direction for the port.
 *
 * @param        port              the DAM port to configure
 * @param        direction         the Tx Frame Sync signal direction
 */
void dam_select_TxFS_direction(dam_port port, signal_direction direction)
{
	if (port < 3) {
		ModifyRegister32(1 << dam_transmit_frame_sync_direction_shift,
				 direction <<
				 dam_transmit_frame_sync_direction_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_transmit_frame_sync_direction_shift,
				 direction <<
				 dam_transmit_frame_sync_direction_shift,
				 _reg_DAM_HPCR(port));
	}
	return;
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
	if (p_config < 3) {
		ModifyRegister32(dam_fs_selection_mask <<
				 dam_transmit_frame_sync_select_shift,
				 ((from_RxFS << 3) | p_source) <<
				 dam_transmit_frame_sync_select_shift,
				 _reg_DAM_HPCR(p_config));
	} else {
		ModifyRegister32(dam_fs_selection_mask <<
				 dam_transmit_frame_sync_select_shift,
				 ((from_RxFS << 3) | p_source) <<
				 dam_transmit_frame_sync_select_shift,
				 _reg_DAM_PPCR(p_config));
	}
	return;
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
			 _reg_DAM_HPCR(port));
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
	if (port < 3) {
		ModifyRegister32(1 << dam_synchronous_mode_shift,
				 synchronous << dam_synchronous_mode_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_synchronous_mode_shift,
				 synchronous << dam_synchronous_mode_shift,
				 _reg_DAM_PPCR(port));
	}
	return;
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
	if (port < 3) {
		ModifyRegister32(1 << dam_transmit_receive_switch_shift,
				 value << dam_transmit_receive_switch_shift,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(1 << dam_transmit_receive_switch_shift,
				 value << dam_transmit_receive_switch_shift,
				 _reg_DAM_PPCR(port));
	}
	return;
}

/*!
 * This function resets the two registers of the selected port.
 *
 * @param        port              the DAM port to reset
 */
void dam_reset_register(dam_port port)
{
	if (port < 3) {
		ModifyRegister32(0xFFFFFFFF, dam_hpcr_default_value,
				 _reg_DAM_HPCR(port));
	} else {
		ModifyRegister32(0xFFFFFFFF, dam_ppcr_default_value,
				 _reg_DAM_PPCR(port));
	}
	return;
}

#ifdef TEST_DAM

/*!
 * This function implements IOCTL controls on a DAM device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter :\n
 * DAM_CONFIG_SSI1:\n
 * data from port 1 to port 4, clock and FS from port 1 (SSI1)\n
 * DAM_CONFIG_SSI2:\n
 * data from port 2 to port 5, clock and FS from port 2 (SSI2)\n
 * DAM_CONFIG_SSI_NETWORK_MODE:\n
 * network mode for mix digital with data from port 1 to port4,\n
 * data from port 2 to port 4, clock and FS from port 1 (SSI1)
 *
 * @return       This function returns 0 if successful.
 */
static int dam_ioctl(struct inode *inode,
		     struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

/*!
 * This function implements the open method on a DAM device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 *
 * @return       This function returns 0.
 */
static int dam_open(struct inode *inode, struct file *file)
{
	/* DBG_PRINTK("ssi : dam_open()\n"); */
	return 0;
}

/*!
 * This function implements the release method on a DAM device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 *
 * @return       This function returns 0.
 */
static int dam_free(struct inode *inode, struct file *file)
{
	/* DBG_PRINTK("ssi : dam_free()\n"); */
	return 0;
}

/*!
 * This structure defines file operations for a DAM device.
 */
static struct file_operations dam_fops = {

	/*!
	 * the owner
	 */
	.owner = THIS_MODULE,

	/*!
	 * the ioctl operation
	 */
	.ioctl = dam_ioctl,

	/*!
	 * the open operation
	 */
	.open = dam_open,

	/*!
	 * the release operation
	 */
	.release = dam_free,
};

#endif

/*!
 * This function implements the init function of the DAM device.
 * This function is called when the module is loaded.
 *
 * @return       This function returns 0.
 */
static int __init dam_init(void)
{
#ifdef TEST_DAM
	struct device *temp_class;
	printk(KERN_DEBUG "dam : dam_init(void) \n");

	major_dam = register_chrdev(0, DAM_NAME, &dam_fops);
	if (major_dam < 0) {
		printk(KERN_WARNING "Unable to get a major for dam");
		return major_dam;
	}

	mxc_dam_class = class_create(THIS_MODULE, DAM_NAME);
	if (IS_ERR(mxc_dam_class)) {
		goto err_out;
	}

	temp_class = device_create(mxc_dam_class, NULL,
				   MKDEV(major_dam, 0), NULL, DAM_NAME);
	if (IS_ERR(temp_class)) {
		goto err_out;
	}
#endif
	return 0;

      err_out:
	printk(KERN_ERR "Error creating dam class device.\n");
	device_destroy(mxc_dam_class, MKDEV(major_dam, 0));
	class_destroy(mxc_dam_class);
	unregister_chrdev(major_dam, DAM_NAME);
	return -1;
}

/*!
 * This function implements the exit function of the SPI device.
 * This function is called when the module is unloaded.
 *
 */
static void __exit dam_exit(void)
{
#ifdef TEST_DAM
	device_destroy(mxc_dam_class, MKDEV(major_dam, 0));
	class_destroy(mxc_dam_class);
	unregister_chrdev(major_dam, DAM_NAME);
	printk(KERN_DEBUG "dam : successfully unloaded\n");
#endif
}

module_init(dam_init);
module_exit(dam_exit);

MODULE_DESCRIPTION("DAM char device driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
