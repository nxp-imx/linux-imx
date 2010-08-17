/*
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
  * @file include/asm-arm/arch-mxc/audio_controls.h
  * @brief this file implements the mxc sound driver interface to OSS framework
  * @ingroup SOUND_DRV
  */
#ifndef __ASM_ARCH_MXC_AUDIO_CONTROLS_H__
#define __ASM_ARCH_MXC_AUDIO_CONTROLS_H__

/*!
 * This ioctl can be used to get the adder configuration, use the audio control
 * SNDCTL_MC13783_READ_OUT_MIXER.\n
 * Possible returned values are :
 * @see MC13783_AUDIO_ADDER_STEREO
 * @see MC13783_AUDIO_ADDER_STEREO_OPPOSITE
 * @see MC13783_AUDIO_ADDER_MONO
 * @see MC13783_AUDIO_ADDER_MONO_OPPOSITE
 *
 */
#define SNDCTL_MC13783_READ_OUT_ADDER        		_SIOR('Z', 6, int)

/*!
 * To set the adder configuration, use the audio control
 * SNDCTL_MC13783_WRITE_OUT_MIXER. Possible arguments are : \n
 * @see MC13783_AUDIO_ADDER_STEREO
 * @see MC13783_AUDIO_ADDER_STEREO_OPPOSITE
 * @see MC13783_AUDIO_ADDER_MONO
 * @see MC13783_AUDIO_ADDER_MONO_OPPOSITE
 *
 */
#define SNDCTL_MC13783_WRITE_OUT_ADDER     		_SIOWR('Z', 7, int)

/*!
 * To get the codec balance configuration, use the audio control
 * SNDCTL_MC13783_READ_OUT_BALANCE.\n
 * Range is 0 (-21 dB left) to 100 (-21 dB right), linear, 3dB step ;
 * 50 is no balance.
 * \n    Examples:
 * \n      0 : -21dB left   50 : balance deactivated   100 : -21 dB right
 *
 */
#define SNDCTL_MC13783_READ_OUT_BALANCE     		 _SIOR('Z', 8, int)

/*!
 * To set the codec balance configuration, use the audio control
 * SNDCTL_MC13783_WRITE_OUT_BALANCE.\n
 * Range is 0 (-21 dB left) to 100 (-21 dB right), linear, 3dB step ;
 * 50 is no balance.
 * \n    Examples:
 * \n      0 : -21dB left   50 : balance deactivated   100 : -21 dB right
 *
 */
#define SNDCTL_MC13783_WRITE_OUT_BALANCE      		_SIOWR('Z', 9, int)

/*!
 * To set the codec filter configuration, use the audio control
 * SNDCTL_MC13783_WRITE_CODEC_FILTER.
 * The new configuration replaces the old one.\n
 * Possible arguments are :
 * @see MC13783_CODEC_FILTER_DISABLE
 * @see MC13783_CODEC_FILTER_HIGH_PASS_IN
 * @see MC13783_CODEC_FILTER_HIGH_PASS_OUT
 * @see MC13783_CODEC_FILTER_DITHERING \n
 *
 */
#define SNDCTL_MC13783_WRITE_CODEC_FILTER      		_SIOWR('Z', 20, int)

/*!
 * To get the codec filter configuration, use the audio control :
 * SNDCTL_MC13783_READ_CODEC_FILTER.
 * The new configuration replaces the old one.\n
 * Possible returned values are :
 * @see MC13783_CODEC_FILTER_DISABLE
 * @see MC13783_CODEC_FILTER_HIGH_PASS_IN
 * @see MC13783_CODEC_FILTER_HIGH_PASS_OUT
 * @see MC13783_CODEC_FILTER_DITHERING \n
 *
 */
#define SNDCTL_MC13783_READ_CODEC_FILTER       		_SIOR('Z', 21, int)

/*
 * To set the clock configuration, use the audio control
 * SNDCTL_MC13783_WRITE_MASTER_CLOCK.		\n
 * Possible arguments are : 			\n
 *          1 : to MCU master			\n
 *          2 : to MC13783 master
 */
#define SNDCTL_MC13783_WRITE_MASTER_CLOCK                   _SIOR('Z', 30, int)

/*!
 * To set the output port, use the audio control
 * SNDCTL_MC13783_WRITE_PORT.\n
 * Possible returned values are :
 * \n         1 : to port 4
 * \n         2 : to port 5
 * Possible returned values are :
 * \n         1 : port 4
 * \n         2 : port 5
 */
#define SNDCTL_MC13783_WRITE_PORT                 	_SIOR('Z', 31, int)

/*!
 * Argument for the mc13783 adder configuration
 * @see SNDCTL_MC13783_WRITE_OUT_ADDER
 * @see SNDCTL_MC13783_READ_OUT_ADDER
 */
#define MC13783_AUDIO_ADDER_STEREO                	0x1
/*!
 * Argument for the mc13783 adder configuration
 * @see SNDCTL_MC13783_WRITE_OUT_ADDER
 * @see SNDCTL_MC13783_READ_OUT_ADDER
 */
#define MC13783_AUDIO_ADDER_STEREO_OPPOSITE       	0x2
/*!
 * Argument for the mc13783 adder configuration
 * @see SNDCTL_MC13783_WRITE_OUT_ADDER
 * @see SNDCTL_MC13783_READ_OUT_ADDER
 */
#define MC13783_AUDIO_ADDER_MONO                  	0x4
/*!
 * Argument for the mc13783 adder configuration
 * @see SNDCTL_MC13783_WRITE_OUT_ADDER
 * @see SNDCTL_MC13783_READ_OUT_ADDER
 */
#define MC13783_AUDIO_ADDER_MONO_OPPOSITE         	0x8

/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_DISABLE              	0x0
/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_HIGH_PASS_IN         	0x1
/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_HIGH_PASS_OUT        	0x2
/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_DITHERING            	0x4

/*!
 * Argument for the system audio clocking selection
 * @see MXC_AUDIO_CLOCKING_MCU_MASTER
 * @see SNDCTL_CLK_SET_MASTER
 */
#define MXC_AUDIO_CLOCKING_MC13783_MASTER         	0x0

/*!
 * Argument for the system audio clocking selection
 * @see MXC_AUDIO_CLOCKING_MC13783_MASTER
 * @see SNDCTL_CLK_SET_MASTER
 */
#define MXC_AUDIO_CLOCKING_MCU_MASTER           	0x1

/*!
 * Argument for the DAM output port selection
 * @see SNDCTL_DAM_SET_OUT_PORT
 * @see MXC_DAM_OUT_PORT_AD2
 */
#define MXC_DAM_OUT_PORT_AD1                    	0x0

/*!
 * Argument for the DAM output port selection
 * @see SNDCTL_DAM_SET_OUT_PORT
 * @see MXC_DAM_OUT_PORT_AD1
 */
#define MXC_DAM_OUT_PORT_AD2                    	0x1

/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_DISABLE              	0x0

/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_HIGH_PASS_IN         	0x1

/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_HIGH_PASS_OUT        	0x2

/*!
 * Argument for the mc13783 codec filter configuration
 * @see SNDCTL_MC13783_WRITE_CODEC_FILTER
 * @see SNDCTL_MC13783_READ_CODEC_FILTER
 */
#define MC13783_CODEC_FILTER_DITHERING            	0x4

#endif				/* __ASM_ARCH_MXC_AUDIO_CONTROLS_H__ */
