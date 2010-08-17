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
 * @file mc13783/pmic_audio.c
 * @brief Implementation of the PMIC(mc13783) Audio driver APIs.
 *
 * The PMIC Audio driver and this API were developed to support the
 * audio playback, recording, and mixing capabilities of the power
 * management ICs that are available from Freescale Semiconductor, Inc.
 *
 * The following operating modes are supported:
 *
 * @verbatim
       Operating Mode               mc13783
       ---------------------------- -------
       Stereo DAC Playback           Yes
       Stereo DAC Input Mixing       Yes
       Voice CODEC Playback          Yes
       Voice CODEC Input Mixing      Yes
       Voice CODEC Mono Recording    Yes
       Voice CODEC Stereo Recording  Yes
       Microphone Bias Control       Yes
       Output Amplifier Control      Yes
       Output Mixing Control         Yes
       Input Amplifier Control       Yes
       Master/Slave Mode Select      Yes
       Anti Pop Bias Circuit Control Yes
   @endverbatim
 *
 * Note that the Voice CODEC may also be referred to as the Telephone
 * CODEC in the PMIC DTS documentation. Also note that, while the power
 * management ICs do provide similar audio capabilities, each PMIC may
 * support additional configuration settings and features. Therefore, it
 * is highly recommended that the appropriate power management IC DTS
 * documents be used in conjunction with this API interface.
 *
 * @ingroup PMIC_AUDIO
 */

#include <linux/module.h>
#include <linux/interrupt.h>	/* For tasklet interface.              */
#include <linux/platform_device.h>	/* For kernel module interface.        */
#include <linux/init.h>
#include <linux/spinlock.h>	/* For spinlock interface.             */
#include <linux/pmic_adc.h>	/* For PMIC ADC driver interface.      */
#include <linux/pmic_status.h>
#include <mach/pmic_audio.h>	/* For PMIC Audio driver interface.    */

/*
 * mc13783 PMIC Audio API
 */

/* EXPORTED FUNCTIONS */
EXPORT_SYMBOL(MIN_STDAC_SAMPLING_RATE_HZ);
EXPORT_SYMBOL(MAX_STDAC_SAMPLING_RATE_HZ);
EXPORT_SYMBOL(pmic_audio_open);
EXPORT_SYMBOL(pmic_audio_close);
EXPORT_SYMBOL(pmic_audio_set_protocol);
EXPORT_SYMBOL(pmic_audio_get_protocol);
EXPORT_SYMBOL(pmic_audio_enable);
EXPORT_SYMBOL(pmic_audio_disable);
EXPORT_SYMBOL(pmic_audio_reset);
EXPORT_SYMBOL(pmic_audio_reset_all);
EXPORT_SYMBOL(pmic_audio_set_callback);
EXPORT_SYMBOL(pmic_audio_clear_callback);
EXPORT_SYMBOL(pmic_audio_get_callback);
EXPORT_SYMBOL(pmic_audio_antipop_enable);
EXPORT_SYMBOL(pmic_audio_antipop_disable);
EXPORT_SYMBOL(pmic_audio_digital_filter_reset);
EXPORT_SYMBOL(pmic_audio_vcodec_set_clock);
EXPORT_SYMBOL(pmic_audio_vcodec_get_clock);
EXPORT_SYMBOL(pmic_audio_vcodec_set_rxtx_timeslot);
EXPORT_SYMBOL(pmic_audio_vcodec_get_rxtx_timeslot);
EXPORT_SYMBOL(pmic_audio_vcodec_set_secondary_txslot);
EXPORT_SYMBOL(pmic_audio_vcodec_get_secondary_txslot);
EXPORT_SYMBOL(pmic_audio_vcodec_set_config);
EXPORT_SYMBOL(pmic_audio_vcodec_clear_config);
EXPORT_SYMBOL(pmic_audio_vcodec_get_config);
EXPORT_SYMBOL(pmic_audio_vcodec_enable_bypass);
EXPORT_SYMBOL(pmic_audio_vcodec_disable_bypass);
EXPORT_SYMBOL(pmic_audio_stdac_set_clock);
EXPORT_SYMBOL(pmic_audio_stdac_get_clock);
EXPORT_SYMBOL(pmic_audio_stdac_set_rxtx_timeslot);
EXPORT_SYMBOL(pmic_audio_stdac_get_rxtx_timeslot);
EXPORT_SYMBOL(pmic_audio_stdac_set_config);
EXPORT_SYMBOL(pmic_audio_stdac_clear_config);
EXPORT_SYMBOL(pmic_audio_stdac_get_config);
EXPORT_SYMBOL(pmic_audio_input_set_config);
EXPORT_SYMBOL(pmic_audio_input_clear_config);
EXPORT_SYMBOL(pmic_audio_input_get_config);
EXPORT_SYMBOL(pmic_audio_vcodec_set_mic);
EXPORT_SYMBOL(pmic_audio_vcodec_get_mic);
EXPORT_SYMBOL(pmic_audio_vcodec_set_mic_on_off);
EXPORT_SYMBOL(pmic_audio_vcodec_get_mic_on_off);
EXPORT_SYMBOL(pmic_audio_vcodec_set_record_gain);
EXPORT_SYMBOL(pmic_audio_vcodec_get_record_gain);
EXPORT_SYMBOL(pmic_audio_vcodec_enable_micbias);
EXPORT_SYMBOL(pmic_audio_vcodec_disable_micbias);
EXPORT_SYMBOL(pmic_audio_vcodec_enable_mixer);
EXPORT_SYMBOL(pmic_audio_vcodec_disable_mixer);
EXPORT_SYMBOL(pmic_audio_stdac_enable_mixer);
EXPORT_SYMBOL(pmic_audio_stdac_disable_mixer);
EXPORT_SYMBOL(pmic_audio_output_set_port);
EXPORT_SYMBOL(pmic_audio_output_get_port);
EXPORT_SYMBOL(pmic_audio_output_clear_port);
EXPORT_SYMBOL(pmic_audio_output_set_stereo_in_gain);
EXPORT_SYMBOL(pmic_audio_output_get_stereo_in_gain);
EXPORT_SYMBOL(pmic_audio_output_set_pgaGain);
EXPORT_SYMBOL(pmic_audio_output_get_pgaGain);
EXPORT_SYMBOL(pmic_audio_output_enable_mixer);
EXPORT_SYMBOL(pmic_audio_output_disable_mixer);
EXPORT_SYMBOL(pmic_audio_output_set_balance);
EXPORT_SYMBOL(pmic_audio_output_get_balance);
EXPORT_SYMBOL(pmic_audio_output_enable_mono_adder);
EXPORT_SYMBOL(pmic_audio_output_disable_mono_adder);
EXPORT_SYMBOL(pmic_audio_output_set_mono_adder_gain);
EXPORT_SYMBOL(pmic_audio_output_get_mono_adder_gain);
EXPORT_SYMBOL(pmic_audio_output_set_config);
EXPORT_SYMBOL(pmic_audio_output_clear_config);
EXPORT_SYMBOL(pmic_audio_output_get_config);
EXPORT_SYMBOL(pmic_audio_output_enable_phantom_ground);
EXPORT_SYMBOL(pmic_audio_output_disable_phantom_ground);
EXPORT_SYMBOL(pmic_audio_set_autodetect);
#ifdef DEBUG_AUDIO
EXPORT_SYMBOL(pmic_audio_dump_registers);
#endif				/* DEBUG_AUDIO */
/*!
 * Define the minimum sampling rate (in Hz) that is supported by the
 * Stereo DAC.
 */
const unsigned MIN_STDAC_SAMPLING_RATE_HZ = 8000;

/*!
 * Define the maximum sampling rate (in Hz) that is supported by the
 * Stereo DAC.
 */
const unsigned MAX_STDAC_SAMPLING_RATE_HZ = 96000;

/*! @def SET_BITS
 * Set a register field to a given value.
 */
#define SET_BITS(reg, field, value)    (((value) << reg.field.offset) & \
		reg.field.mask)
/*! @def GET_BITS
 * Get the current value of a given register field.
 */
#define GET_BITS(reg, field, value)    (((value) & reg.field.mask) >> \
		reg.field.offset)

/*!
 * @brief Define the possible states for a device handle.
 *
 * This enumeration is used to track the current state of each device handle.
 */
typedef enum {
	HANDLE_FREE,		/*!< Handle is available for use. */
	HANDLE_IN_USE		/*!< Handle is currently in use.  */
} HANDLE_STATE;

/*!
 * @brief Identifies the hardware interrupt source.
 *
 * This enumeration identifies which of the possible hardware interrupt
 * sources actually caused the current interrupt handler to be called.
 */
typedef enum {
	CORE_EVENT_MC2BI,	/*!< Microphone Bias 2 detect. */
	CORE_EVENT_HSDETI,	/*!< Detect Headset attach  */
	CORE_EVENT_HSLI,	/*!< Detect Stereo Headset  */
	CORE_EVENT_ALSPTHI,	/*!< Detect Thermal shutdown of ALSP  */
	CORE_EVENT_AHSSHORTI	/*!< Detect Short circuit on AHS outputs  */
} PMIC_CORE_EVENT;

/*!
 * @brief This structure is used to track the state of a microphone input.
 */
typedef struct {
	PMIC_AUDIO_INPUT_PORT mic;	/*!< Microphone input port.      */
	PMIC_AUDIO_INPUT_MIC_STATE micOnOff;	/*!< Microphone On/Off state.    */
	PMIC_AUDIO_MIC_AMP_MODE ampMode;	/*!< Input amplifier mode.       */
	PMIC_AUDIO_MIC_GAIN gain;	/*!< Input amplifier gain level. */
} PMIC_MICROPHONE_STATE;

/*!
 * @brief Tracks whether a headset is currently attached or not.
 */
typedef enum {
	NO_HEADSET,		/*!< No headset currently attached. */
	HEADSET_ON		/*!< Headset has been attached.     */
} HEADSET_STATUS;

/*!
 * @brief mc13783 only enum that indicates the path to output taken
 * by the voice codec output
 */
typedef enum {
	VCODEC_DIRECT_OUT,	/*!< Vcodec signal out direct */
	VCODEC_MIXER_OUT	/*!< Output via the mixer     */
} PMIC_AUDIO_VCODEC_OUTPUT_PATH;

/*!
 * @brief This structure is used to define a specific hardware register field.
 *
 * All hardware register fields are defined using an offset to the LSB
 * and a mask. The offset is used to right shift a register value before
 * applying the mask to actually obtain the value of the field.
 */
typedef struct {
	const unsigned char offset;	/*!< Offset of LSB of register field.          */
	const unsigned int mask;	/*!< Mask value used to isolate register field. */
} REGFIELD;

/*!
 * @brief This structure lists all fields of the AUD_CODEC hardware register.
 */
typedef struct {
	REGFIELD CDCSSISEL;	/*!< codec SSI bus select              */
	REGFIELD CDCCLKSEL;	/*!< Codec clock input select          */
	REGFIELD CDCSM;		/*!< Codec slave / master select       */
	REGFIELD CDCBCLINV;	/*!< Codec bitclock inversion          */
	REGFIELD CDCFSINV;	/*!< Codec framesync inversion         */
	REGFIELD CDCFS;		/*!< Bus protocol selection - 2 bits   */
	REGFIELD CDCCLK;	/*!< Codec clock setting - 3 bits      */
	REGFIELD CDCFS8K16K;	/*!< Codec framesync select            */
	REGFIELD CDCEN;		/*!< Codec enable                      */
	REGFIELD CDCCLKEN;	/*!< Codec clocking enable             */
	REGFIELD CDCTS;		/*!< Codec SSI tristate                */
	REGFIELD CDCDITH;	/*!< Codec dithering                   */
	REGFIELD CDCRESET;	/*!< Codec filter reset                */
	REGFIELD CDCBYP;	/*!< Codec bypass                      */
	REGFIELD CDCALM;	/*!< Codec analog loopback             */
	REGFIELD CDCDLM;	/*!< Codec digital loopback            */
	REGFIELD AUDIHPF;	/*!< Transmit high pass filter enable  */
	REGFIELD AUDOHPF;	/*!< Receive high pass filter enable   */
} REGISTER_AUD_CODEC;

/*!
 * @brief This variable is used to access the AUD_CODEC hardware register.
 *
 * This variable defines how to access all of the fields within the
 * AUD_CODEC hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_AUD_CODEC regAUD_CODEC = {
	{0, 0x000001},		/* CDCSSISEL    */
	{1, 0x000002},		/* CDCCLKSEL    */
	{2, 0x000004},		/* CDCSM        */
	{3, 0x000008},		/* CDCBCLINV    */
	{4, 0x000010},		/* CDCFSINV     */
	{5, 0x000060},		/* CDCFS        */
	{7, 0x000380},		/* CDCCLK       */
	{10, 0x000400},		/* CDCFS8K16K   */
	{11, 0x000800},		/* CDCEN        */
	{12, 0x001000},		/* CDCCLKEN     */
	{13, 0x002000},		/* CDCTS        */
	{14, 0x004000},		/* CDCDITH      */
	{15, 0x008000},		/* CDCRESET     */
	{16, 0x010000},		/* CDCBYP       */
	{17, 0x020000},		/* CDCALM       */
	{18, 0x040000},		/* CDCDLM       */
	{19, 0x080000},		/* AUDIHPF      */
	{20, 0x100000}		/* AUDOHPF      */
	/* Unused       */
	/* Unused       */
	/* Unused       */

};

/*!
 * @brief This structure lists all fields of the ST_DAC hardware register.
 */
 /* VVV */
typedef struct {
	REGFIELD STDCSSISEL;	/*!< Stereo DAC SSI bus select                            */
	REGFIELD STDCCLKSEL;	/*!< Stereo DAC clock input select                        */
	REGFIELD STDCSM;	/*!< Stereo DAC slave / master select                     */
	REGFIELD STDCBCLINV;	/*!< Stereo DAC bitclock inversion                        */
	REGFIELD STDCFSINV;	/*!< Stereo DAC framesync inversion                       */
	REGFIELD STDCFS;	/*!< Bus protocol selection - 2 bits                      */
	REGFIELD STDCCLK;	/*!< Stereo DAC clock setting - 3 bits                    */
	REGFIELD STDCFSDLYB;	/*!< Stereo DAC framesync delay bar                       */
	REGFIELD STDCEN;	/*!< Stereo DAC enable                                    */
	REGFIELD STDCCLKEN;	/*!< Stereo DAC clocking enable                           */
	REGFIELD STDCRESET;	/*!< Stereo DAC filter reset                              */
	REGFIELD SPDIF;		/*!< Stereo DAC SSI SPDIF mode. Mode no longer available. */
	REGFIELD SR;		/*!< Stereo DAC sample rate - 4 bits                      */
} REGISTER_ST_DAC;

/*!
 * @brief This variable is used to access the ST_DAC hardware register.
 *
 * This variable defines how to access all of the fields within the
 * ST_DAC hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_ST_DAC regST_DAC = {
	{0, 0x000001},		/* STDCSSISEL        */
	{1, 0x000002},		/* STDCCLKSEL        */
	{2, 0x000004},		/* STDCSM            */
	{3, 0x000008},		/* STDCBCLINV        */
	{4, 0x000010},		/* STDCFSINV         */
	{5, 0x000060},		/* STDCFS            */
	{7, 0x000380},		/* STDCCLK           */
	{10, 0x000400},		/* STDCFSDLYB        */
	{11, 0x000800},		/* STDCEN            */
	{12, 0x001000},		/* STDCCLKEN         */
	{15, 0x008000},		/* STDCRESET         */
	{16, 0x010000},		/* SPDIF             */
	{17, 0x1E0000}		/* SR                */
};

/*!
 * @brief This structure lists all of the fields in the SSI_NETWORK hardware register.
 */
typedef struct {
	REGFIELD CDCTXRXSLOT;	/*!< Codec timeslot assignment  - 2 bits                                  */
	REGFIELD CDCTXSECSLOT;	/*!< Codec secondary transmit timeslot - 2 bits                           */
	REGFIELD CDCRXSECSLOT;	/*!< Codec secondary receive timeslot - 2 bits                            */
	REGFIELD CDCRXSECGAIN;	/*!< Codec secondary receive channel gain setting - 2 bits                */
	REGFIELD CDCSUMGAIN;	/*!< Codec summed receive signal gain setting                             */
	REGFIELD CDCFSDLY;	/*!< Codec framesync delay                                                */
	REGFIELD STDCSLOTS;	/*!< Stereo DAC number of timeslots select  - 2 bits                      */
	REGFIELD STDCRXSLOT;	/*!< Stereo DAC timeslot assignment - 2 bits                              */
	REGFIELD STDCRXSECSLOT;	/*!< Stereo DAC secondary receive timeslot  - 2 bits                      */
	REGFIELD STDCRXSECGAIN;	/*!< Stereo DAC secondary receive channel gain setting   - 2 bits         */
	REGFIELD STDCSUMGAIN;	/*!< Stereo DAC summed receive signal gain setting                        */
} REGISTER_SSI_NETWORK;

/*!
 * @brief This variable is used to access the SSI_NETWORK hardware register.
 *
 * This variable defines how to access all of the fields within the
 * SSI_NETWORK hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_SSI_NETWORK regSSI_NETWORK = {
	{2, 0x00000c},		/* CDCTXRXSLOT          */
	{4, 0x000030},		/* CDCTXSECSLOT         */
	{6, 0x0000c0},		/* CDCRXSECSLOT         */
	{8, 0x000300},		/* CDCRXSECGAIN         */
	{10, 0x000400},		/* CDCSUMGAIN           */
	{11, 0x000800},		/* CDCFSDLY             */
	{12, 0x003000},		/* STDCSLOTS            */
	{14, 0x00c000},		/* STDCRXSLOT           */
	{16, 0x030000},		/* STDCRXSECSLOT        */
	{18, 0x0c0000},		/* STDCRXSECGAIN        */
	{20, 0x100000}		/* STDCSUMGAIN          */
};

/*!
 * @brief This structure lists all fields of the AUDIO_TX hardware register.
 *
 *
 */
typedef struct {
	REGFIELD MC1BEN;	/*!< Microphone bias 1 enable                                */
	REGFIELD MC2BEN;	/*!< Microphone bias 2 enable                                */
	REGFIELD MC2BDETDBNC;	/*!< Microphone bias detect debounce setting                 */
	REGFIELD MC2BDETEN;	/*!< Microphone bias 2 detect enable                         */
	REGFIELD AMC1REN;	/*!< Amplifier Amc1R enable                                  */
	REGFIELD AMC1RITOV;	/*!< Amplifier Amc1R current to voltage mode enable          */
	REGFIELD AMC1LEN;	/*!< Amplifier Amc1L enable                                  */
	REGFIELD AMC1LITOV;	/*!< Amplifier Amc1L current to voltage mode enable          */
	REGFIELD AMC2EN;	/*!< Amplifier Amc2 enable                                   */
	REGFIELD AMC2ITOV;	/*!< Amplifier Amc2 current to voltage mode enable           */
	REGFIELD ATXINEN;	/*!< Amplifier Atxin enable                                  */
	REGFIELD ATXOUTEN;	/*!< Reserved for output TXOUT enable, currently not used    */
	REGFIELD RXINREC;	/*!< RXINR/RXINL to voice CODEC ADC routing enable           */
	REGFIELD PGATXR;	/*!< Transmit gain setting right - 5 bits                    */
	REGFIELD PGATXL;	/*!< Transmit gain setting left  - 5 bits                    */
} REGISTER_AUDIO_TX;

/*!
 * @brief This variable is used to access the AUDIO_TX hardware register.
 *
 * This variable defines how to access all of the fields within the
 * AUDIO_TX hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_AUDIO_TX regAUDIO_TX = {
	{0, 0x000001},		/* MC1BEN              */
	{1, 0x000002},		/* MC2BEN              */
	{2, 0x000004},		/* MC2BDETDBNC         */
	{3, 0x000008},		/* MC2BDETEN           */
	{5, 0x000020},		/* AMC1REN             */
	{6, 0x000040},		/* AMC1RITOV           */
	{7, 0x000080},		/* AMC1LEN             */
	{8, 0x000100},		/* AMC1LITOV           */
	{9, 0x000200},		/* AMC2EN              */
	{10, 0x000400},		/* AMC2ITOV            */
	{11, 0x000800},		/* ATXINEN             */
	{12, 0x001000},		/* ATXOUTEN            */
	{13, 0x002000},		/* RXINREC             */
	{14, 0x07c000},		/* PGATXR              */
	{19, 0xf80000}		/* PGATXL              */
};

/*!
 * @brief This structure lists all fields of the AUDIO_RX_0 hardware register.
 */
typedef struct {
	REGFIELD VAUDIOON;	/*!<    Forces VAUDIO in active on mode               */
	REGFIELD BIASEN;	/*!<    Audio bias enable                             */
	REGFIELD BIASSPEED;	/*!<    Turn on ramp speed of the audio bias          */
	REGFIELD ASPEN;		/*!<    Amplifier Asp enable                          */
	REGFIELD ASPSEL;	/*!<    Asp input selector                            */
	REGFIELD ALSPEN;	/*!<    Amplifier Alsp enable                         */
	REGFIELD ALSPREF;	/*!<    Bias Alsp at common audio reference           */
	REGFIELD ALSPSEL;	/*!<    Alsp input selector                           */
	REGFIELD LSPLEN;	/*!<    Output LSPL enable                            */
	REGFIELD AHSREN;	/*!<    Amplifier AhsR enable                         */
	REGFIELD AHSLEN;	/*!<    Amplifier AhsL enable                         */
	REGFIELD AHSSEL;	/*!<    Ahsr and Ahsl input selector                  */
	REGFIELD HSPGDIS;	/*!<    Phantom ground disable                        */
	REGFIELD HSDETEN;	/*!<    Headset detect enable                         */
	REGFIELD HSDETAUTOB;	/*!<    Amplifier state determined by headset detect  */
	REGFIELD ARXOUTREN;	/*!<    Output RXOUTR enable                          */
	REGFIELD ARXOUTLEN;	/*!<    Output RXOUTL enable                          */
	REGFIELD ARXOUTSEL;	/*!<    Arxout input selector                         */
	REGFIELD CDCOUTEN;	/*!<    Output CDCOUT enable                          */
	REGFIELD HSLDETEN;	/*!<    Headset left channel detect enable            */
	REGFIELD ADDCDC;	/*!<    Adder channel codec selection                 */
	REGFIELD ADDSTDC;	/*!<    Adder channel stereo DAC selection            */
	REGFIELD ADDRXIN;	/*!<    Adder channel line in selection               */
} REGISTER_AUDIO_RX_0;

/*!
 * @brief This variable is used to access the AUDIO_RX_0 hardware register.
 *
 * This variable defines how to access all of the fields within the
 * AUDIO_RX_0 hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_AUDIO_RX_0 regAUDIO_RX_0 = {
	{0, 0x000001},		/* VAUDIOON    */
	{1, 0x000002},		/* BIASEN      */
	{2, 0x000004},		/* BIASSPEED   */
	{3, 0x000008},		/* ASPEN       */
	{4, 0x000010},		/* ASPSEL      */
	{5, 0x000020},		/* ALSPEN      */
	{6, 0x000040},		/* ALSPREF     */
	{7, 0x000080},		/* ALSPSEL     */
	{8, 0x000100},		/* LSPLEN      */
	{9, 0x000200},		/* AHSREN      */
	{10, 0x000400},		/* AHSLEN      */
	{11, 0x000800},		/* AHSSEL      */
	{12, 0x001000},		/* HSPGDIS     */
	{13, 0x002000},		/* HSDETEN     */
	{14, 0x004000},		/* HSDETAUTOB  */
	{15, 0x008000},		/* ARXOUTREN   */
	{16, 0x010000},		/* ARXOUTLEN   */
	{17, 0x020000},		/* ARXOUTSEL   */
	{18, 0x040000},		/* CDCOUTEN    */
	{19, 0x080000},		/* HSLDETEN    */
	{21, 0x200000},		/* ADDCDC      */
	{22, 0x400000},		/* ADDSTDC     */
	{23, 0x800000}		/* ADDRXIN     */
};

/*!
 * @brief This structure lists all fields of the AUDIO_RX_1 hardware register.
 */
typedef struct {
	REGFIELD PGARXEN;	/*!<    Codec receive PGA enable                 */
	REGFIELD PGARX;		/*!<    Codec receive gain setting - 4 bits      */
	REGFIELD PGASTEN;	/*!<    Stereo DAC PGA enable                    */
	REGFIELD PGAST;		/*!<    Stereo DAC gain setting  - 4 bits        */
	REGFIELD ARXINEN;	/*!<    Amplifier Arx enable                     */
	REGFIELD ARXIN;		/*!<    Amplifier Arx additional gain setting    */
	REGFIELD PGARXIN;	/*!<    PGArxin gain setting  - 4 bits           */
	REGFIELD MONO;		/*!<    Mono adder setting   - 2 bits            */
	REGFIELD BAL;		/*!<    Balance control      - 3 bits            */
	REGFIELD BALLR;		/*!<    Left / right balance                     */
} REGISTER_AUDIO_RX_1;

/*!
 * @brief This variable is used to access the AUDIO_RX_1 hardware register.
 *
 * This variable defines how to access all of the fields within the
 * AUDIO_RX_1 hardware register. The initial values consist of the offset
 * and mask values needed to access each of the register fields.
 */
static const REGISTER_AUDIO_RX_1 regAUDIO_RX_1 = {
	{0, 0x000001},		/* PGARXEN    */
	{1, 0x00001e},		/* PGARX      */
	{5, 0x000020},		/* PGASTEN   */
	{6, 0x0003c0},		/* PGAST       */
	{10, 0x000400},		/* ARXINEN      */
	{11, 0x000800},		/* ARXIN      */
	{12, 0x00f000},		/* PGARXIN     */
	{16, 0x030000},		/* MONO     */
	{18, 0x1c0000},		/* BAL      */
	{21, 0x200000}		/* BALLR      */
};

/*! Define a mask to access the entire hardware register. */
static const unsigned int REG_FULLMASK = 0xffffff;

/*! Reset value for the AUD_CODEC register. */
static const unsigned int RESET_AUD_CODEC = 0x180027;

/*! Reset value for the ST_DAC register.
 *
 *  Note that we avoid resetting any of the arbitration bits.
 */
static const unsigned int RESET_ST_DAC = 0x0E0004;

/*! Reset value for the SSI_NETWORK register. */
static const unsigned int RESET_SSI_NETWORK = 0x013060;

/*! Reset value for the AUDIO_TX register.
 *
 *  Note that we avoid resetting any of the arbitration bits.
 */
static const unsigned int RESET_AUDIO_TX = 0x420000;

/*! Reset value for the AUDIO_RX_0 register. */
static const unsigned int RESET_AUDIO_RX_0 = 0x001000;

/*! Reset value for the AUDIO_RX_1 register. */
static const unsigned int RESET_AUDIO_RX_1 = 0x00D35A;

/*! Reset mask for the SSI network Vcodec part. first 12 bits
 * 0 - 11 */
static const unsigned int REG_SSI_VCODEC_MASK = 0x000fff;

/*! Reset mask for the SSI network STDAC part. last 12 bits
 * 12 - 24 */
static const unsigned int REG_SSI_STDAC_MASK = 0xfff000;

/*! Constant NULL value for initializing/reseting the audio handles. */
static const PMIC_AUDIO_HANDLE AUDIO_HANDLE_NULL = (PMIC_AUDIO_HANDLE) NULL;

/*!
 * @brief This structure maintains the current state of the Stereo DAC.
 */
typedef struct {
	PMIC_AUDIO_HANDLE handle;	/*!< Handle used to access
					   the Stereo DAC.         */
	HANDLE_STATE handleState;	/*!< Current handle state.   */
	PMIC_AUDIO_DATA_BUS busID;	/*!< Data bus used to access
					   the Stereo DAC.         */
	bool protocol_set;
	PMIC_AUDIO_BUS_PROTOCOL protocol;	/*!< Data bus protocol.      */
	PMIC_AUDIO_BUS_MODE masterSlave;	/*!< Master/Slave mode
						   select.                 */
	PMIC_AUDIO_NUMSLOTS numSlots;	/*!< Number of timeslots
					   used.                   */
	PMIC_AUDIO_CALLBACK callback;	/*!< Event notification
					   callback function
					   pointer.                */
	PMIC_AUDIO_EVENTS eventMask;	/*!< Event notification mask. */
	PMIC_AUDIO_CLOCK_IN_SOURCE clockIn;	/*!< Stereo DAC clock input
						   source select.          */
	PMIC_AUDIO_STDAC_SAMPLING_RATE samplingRate;	/*!< Stereo DAC sampling rate
							   select.                 */
	PMIC_AUDIO_STDAC_CLOCK_IN_FREQ clockFreq;	/*!< Stereo DAC clock input
							   frequency.              */
	PMIC_AUDIO_CLOCK_INVERT invert;	/*!< Stereo DAC clock signal
					   invert select.          */
	PMIC_AUDIO_STDAC_TIMESLOTS timeslot;	/*!< Stereo DAC data
						   timeslots select.       */
	PMIC_AUDIO_STDAC_CONFIG config;	/*!< Stereo DAC configuration
					   options.                */
} PMIC_AUDIO_STDAC_STATE;

/*!
 * @brief This variable maintains the current state of the Stereo DAC.
 *
 * This variable tracks the current state of the Stereo DAC audio hardware
 * along with any information that is required by the device driver to
 * manage the hardware (e.g., callback functions and event notification
 * masks).
 *
 * The initial values represent the reset/power on state of the Stereo DAC.
 */
static PMIC_AUDIO_STDAC_STATE stDAC = {
	(PMIC_AUDIO_HANDLE) NULL,	/* handle       */
	HANDLE_FREE,		/* handleState  */
	AUDIO_DATA_BUS_1,	/* busID        */
	false,
	NORMAL_MSB_JUSTIFIED_MODE,	/* protocol     */
	BUS_MASTER_MODE,	/* masterSlave  */
	USE_2_TIMESLOTS,	/* numSlots     */
	(PMIC_AUDIO_CALLBACK) NULL,	/* callback     */
	(PMIC_AUDIO_EVENTS) NULL,	/* eventMask    */
	CLOCK_IN_CLIA,		/* clockIn      */
	STDAC_RATE_44_1_KHZ,	/* samplingRate */
	STDAC_CLI_13MHZ,	/* clockFreq    */
	NO_INVERT,		/* invert       */
	USE_TS0_TS1,		/* timeslot     */
	(PMIC_AUDIO_STDAC_CONFIG) 0	/* config       */
};

/*!
 * @brief This structure maintains the current state of the Voice CODEC.
 */
typedef struct {
	PMIC_AUDIO_HANDLE handle;	/*!< Handle used to access
					   the Voice CODEC.       */
	HANDLE_STATE handleState;	/*!< Current handle state.  */
	PMIC_AUDIO_DATA_BUS busID;	/*!< Data bus used to access
					   the Voice CODEC.       */
	bool protocol_set;
	PMIC_AUDIO_BUS_PROTOCOL protocol;	/*!< Data bus protocol.     */
	PMIC_AUDIO_BUS_MODE masterSlave;	/*!< Master/Slave mode
						   select.                */
	PMIC_AUDIO_NUMSLOTS numSlots;	/*!< Number of timeslots
					   used.                  */
	PMIC_AUDIO_CALLBACK callback;	/*!< Event notification
					   callback function
					   pointer.               */
	PMIC_AUDIO_EVENTS eventMask;	/*!< Event notification
					   mask.                  */
	PMIC_AUDIO_CLOCK_IN_SOURCE clockIn;	/*!< Voice CODEC clock input
						   source select.         */
	PMIC_AUDIO_VCODEC_SAMPLING_RATE samplingRate;	/*!< Voice CODEC sampling
							   rate select.           */
	PMIC_AUDIO_VCODEC_CLOCK_IN_FREQ clockFreq;	/*!< Voice CODEC clock input
							   frequency.             */
	PMIC_AUDIO_CLOCK_INVERT invert;	/*!< Voice CODEC clock
					   signal invert select.  */
	PMIC_AUDIO_VCODEC_TIMESLOT timeslot;	/*!< Voice CODEC data
						   timeslot select.       */
	PMIC_AUDIO_VCODEC_TIMESLOT secondaryTXtimeslot;

	PMIC_AUDIO_VCODEC_CONFIG config;	/*!< Voice CODEC
						   configuration
						   options.            */
	PMIC_MICROPHONE_STATE leftChannelMic;	/*!< Left channel
						   microphone
						   configuration.      */
	PMIC_MICROPHONE_STATE rightChannelMic;	/*!< Right channel
						   microphone
						   configuration.      */
} PMIC_AUDIO_VCODEC_STATE;

/*!
 * @brief This variable maintains the current state of the Voice CODEC.
 *
 * This variable tracks the current state of the Voice CODEC audio hardware
 * along with any information that is required by the device driver to
 * manage the hardware (e.g., callback functions and event notification
 * masks).
 *
 * The initial values represent the reset/power on state of the Voice CODEC.
 */
static PMIC_AUDIO_VCODEC_STATE vCodec = {
	(PMIC_AUDIO_HANDLE) NULL,	/* handle          */
	HANDLE_FREE,		/* handleState     */
	AUDIO_DATA_BUS_2,	/* busID           */
	false,
	NETWORK_MODE,		/* protocol        */
	BUS_SLAVE_MODE,		/* masterSlave     */
	USE_4_TIMESLOTS,	/* numSlots        */
	(PMIC_AUDIO_CALLBACK) NULL,	/* callback        */
	(PMIC_AUDIO_EVENTS) NULL,	/* eventMask       */
	CLOCK_IN_CLIB,		/* clockIn         */
	VCODEC_RATE_8_KHZ,	/* samplingRate    */
	VCODEC_CLI_13MHZ,	/* clockFreq       */
	NO_INVERT,		/* invert          */
	USE_TS0,		/* timeslot pri    */
	USE_TS2,		/* timeslot sec TX */
	INPUT_HIGHPASS_FILTER | OUTPUT_HIGHPASS_FILTER,	/* config          */
	/* leftChannelMic  */
	{NO_MIC,		/*    mic          */
	 MICROPHONE_OFF,	/*    micOnOff     */
	 AMP_OFF,		/*    ampMode      */
	 MIC_GAIN_0DB		/*    gain         */
	 },
	/* rightChannelMic */
	{NO_MIC,		/*    mic          */
	 MICROPHONE_OFF,	/*    micOnOff     */
	 AMP_OFF,		/*    ampMode      */
	 MIC_GAIN_0DB		/*    gain         */
	 }
};

/*!
 * @brief This maintains the current state of the External Stereo Input.
 */
typedef struct {
	PMIC_AUDIO_HANDLE handle;	/*!< Handle used to access the
					   External Stereo Inputs.     */
	HANDLE_STATE handleState;	/*!< Current handle state.       */
	PMIC_AUDIO_CALLBACK callback;	/*!< Event notification callback
					   function pointer.           */
	PMIC_AUDIO_EVENTS eventMask;	/*!< Event notification mask.    */
	PMIC_AUDIO_STEREO_IN_GAIN inputGain;	/*!< External Stereo Input
						   amplifier gain level.       */
} PMIC_AUDIO_EXT_STEREO_IN_STATE;

/*!
 * @brief This maintains the current state of the External Stereo Input.
 *
 * This variable tracks the current state of the External Stereo Input audio
 * hardware along with any information that is required by the device driver
 * to manage the hardware (e.g., callback functions and event notification
 * masks).
 *
 * The initial values represent the reset/power on state of the External
 * Stereo Input.
 */
static PMIC_AUDIO_EXT_STEREO_IN_STATE extStereoIn = {
	(PMIC_AUDIO_HANDLE) NULL,	/* handle      */
	HANDLE_FREE,		/* handleState */
	(PMIC_AUDIO_CALLBACK) NULL,	/* callback    */
	(PMIC_AUDIO_EVENTS) NULL,	/* eventMask   */
	STEREO_IN_GAIN_0DB	/* inputGain   */
};

/*!
 * @brief This maintains the current state of the callback & Eventmask.
 */
typedef struct {
	PMIC_AUDIO_CALLBACK callback;	/*!< Event notification callback
					   function pointer.           */
	PMIC_AUDIO_EVENTS eventMask;	/*!< Event notification mask.    */
} PMIC_AUDIO_EVENT_STATE;

static PMIC_AUDIO_EVENT_STATE event_state = {
	(PMIC_AUDIO_CALLBACK) NULL,	/*Callback */
	(PMIC_AUDIO_EVENTS) NULL,	/* EventMask */

};

/*!
 * @brief This maintains the current state of the Audio Output Section.
 */
typedef struct {
	PMIC_AUDIO_OUTPUT_PORT outputPort;	/*!< Current audio
						   output port.     */
	PMIC_AUDIO_OUTPUT_PGA_GAIN vCodecoutputPGAGain;	/*!< Output PGA gain
							   level codec      */
	PMIC_AUDIO_OUTPUT_PGA_GAIN stDacoutputPGAGain;	/*!< Output PGA gain
							   level stDAC      */
	PMIC_AUDIO_OUTPUT_PGA_GAIN extStereooutputPGAGain;	/*!< Output PGA gain
								   level stereo ext */
	PMIC_AUDIO_OUTPUT_BALANCE_GAIN balanceLeftGain;	/*!< Left channel
							   balance gain
							   level.           */
	PMIC_AUDIO_OUTPUT_BALANCE_GAIN balanceRightGain;	/*!< Right channel
								   balance gain
								   level.           */
	PMIC_AUDIO_MONO_ADDER_OUTPUT_GAIN monoAdderGain;	/*!< Mono adder gain
								   level.           */
	PMIC_AUDIO_OUTPUT_CONFIG config;	/*!< Audio output
						   section config
						   options.         */
	PMIC_AUDIO_VCODEC_OUTPUT_PATH vCodecOut;

} PMIC_AUDIO_AUDIO_OUTPUT_STATE;

/*!
 * @brief This variable maintains the current state of the Audio Output Section.
 *
 * This variable tracks the current state of the Audio Output Section.
 *
 * The initial values represent the reset/power on state of the Audio
 * Output Section.
 */
static PMIC_AUDIO_AUDIO_OUTPUT_STATE audioOutput = {
	(PMIC_AUDIO_OUTPUT_PORT) NULL,	/* outputPort       */
	OUTPGA_GAIN_0DB,	/* outputPGAGain    */
	OUTPGA_GAIN_0DB,	/* outputPGAGain    */
	OUTPGA_GAIN_0DB,	/* outputPGAGain    */
	BAL_GAIN_0DB,		/* balanceLeftGain  */
	BAL_GAIN_0DB,		/* balanceRightGain */
	MONOADD_GAIN_0DB,	/* monoAdderGain    */
	(PMIC_AUDIO_OUTPUT_CONFIG) 0,	/* config           */
	VCODEC_DIRECT_OUT
};

/*! The current headset status. */
static HEADSET_STATUS headsetState = NO_HEADSET;

/* Removed PTT variable */
/*! Define a 1 ms wait interval that is needed to ensure that certain
 *  hardware operations are successfully completed.
 */
static const unsigned long delay_1ms = (HZ / 1000);

/*!
 * @brief This spinlock is used to provide mutual exclusion.
 *
 * Create a spinlock that can be used to provide mutually exclusive
 * read/write access to the globally accessible data structures
 * that were defined above. Mutually exclusive access is required to
 * ensure that the audio data structures are consistent at all times
 * when possibly accessed by multiple threads of execution (for example,
 * while simultaneously handling a user request and an interrupt event).
 *
 * We need to use a spinlock whenever we do need to provide mutual
 * exclusion while possibly executing in a hardware interrupt context.
 * Spinlocks should be held for the minimum time that is necessary
 * because hardware interrupts are disabled while a spinlock is held.
 *
 */
static DEFINE_SPINLOCK(lock);

/*!
 * @brief This mutex is used to provide mutual exclusion.
 *
 * Create a mutex that can be used to provide mutually exclusive
 * read/write access to the globally accessible data structures
 * that were defined above. Mutually exclusive access is required to
 * ensure that the audio data structures are consistent at all times
 * when possibly accessed by multiple threads of execution.
 *
 * Note that we use a mutex instead of the spinlock whenever disabling
 * interrupts while in the critical section is not required. This helps
 * to minimize kernel interrupt handling latency.
 */
static DECLARE_MUTEX(mutex);

/*!
 * @brief Global variable to track currently active interrupt events.
 *
 * This global variable is used to keep track of all of the currently
 * active interrupt events for the audio driver. Note that access to this
 * variable may occur while within an interrupt context and, therefore,
 * must be guarded by using a spinlock.
 */
/* static PMIC_CORE_EVENT eventID = 0; */

/* Prototypes for all static audio driver functions. */
/*
static PMIC_STATUS pmic_audio_mic_boost_enable(void);
static PMIC_STATUS pmic_audio_mic_boost_disable(void);*/
static PMIC_STATUS pmic_audio_close_handle(const PMIC_AUDIO_HANDLE handle);
static PMIC_STATUS pmic_audio_reset_device(const PMIC_AUDIO_HANDLE handle);

static PMIC_STATUS pmic_audio_deregister(void *callback,
					 PMIC_AUDIO_EVENTS * const eventMask);

/*************************************************************************
 * Audio device access APIs.
 *************************************************************************
 */

/*!
 * @name General Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Audio
 * hardware.
 */
/*@{*/

PMIC_STATUS pmic_audio_set_autodetect(int val)
{
	PMIC_STATUS status;
	unsigned int reg_mask = 0, reg_write = 0;
	reg_mask = SET_BITS(regAUDIO_RX_0, VAUDIOON, 1);
	status = pmic_write_reg(REG_AUDIO_RX_0, reg_mask, reg_mask);
	if (status != PMIC_SUCCESS)
		return status;
	reg_mask = 0;
	if (val == 1) {
		reg_write = SET_BITS(regAUDIO_RX_0, HSDETEN, 1) |
		    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
	} else {
		reg_write = 0;
	}
	reg_mask =
	    SET_BITS(regAUDIO_RX_0, HSDETEN, 1) | SET_BITS(regAUDIO_RX_0,
							   HSDETAUTOB, 1);
	status = pmic_write_reg(REG_AUDIO_RX_0, reg_write, reg_mask);

	return status;
}

/*!
 * @brief Request exclusive access to the PMIC Audio hardware.
 *
 * Attempt to open and gain exclusive access to a key PMIC audio hardware
 * component (e.g., the Stereo DAC or the Voice CODEC). Depending upon the
 * type of audio operation that is desired and the nature of the audio data
 * stream, the Stereo DAC and/or the Voice CODEC will be a required hardware
 * component and needs to be acquired by calling this function.
 *
 * If the open request is successful, then a numeric handle is returned
 * and this handle must be used in all subsequent function calls to complete
 * the configuration of either the Stereo DAC or the Voice CODEC and along
 * with any other associated audio hardware components that will be needed.
 *
 * The same handle must also be used in the close call when use of the PMIC
 * audio hardware is no longer required.
 *
 * The open request will fail if the requested audio hardware component has
 * already been acquired by a previous open call but not yet closed.
 *
 * @param  handle          Device handle to be used for subsequent PMIC
 *                              audio API calls.
 * @param  device          The required PMIC audio hardware component.
 *
 * @retval      PMIC_SUCCESS         If the open request was successful
 * @retval      PMIC_PARAMETER_ERROR If the handle argument is NULL.
 * @retval      PMIC_ERROR           If the audio hardware component is
 *                                   unavailable.
 */
PMIC_STATUS pmic_audio_open(PMIC_AUDIO_HANDLE * const handle,
			    const PMIC_AUDIO_SOURCE device)
{
	PMIC_STATUS rc = PMIC_ERROR;

	if (handle == (PMIC_AUDIO_HANDLE *) NULL) {
		/* Do not dereference a NULL pointer. */
		return PMIC_PARAMETER_ERROR;
	}

	/* We only need to acquire a mutex here because the interrupt handler
	 * never modifies the device handle or device handle state. Therefore,
	 * we don't need to worry about conflicts with the interrupt handler
	 * or the need to execute in an interrupt context.
	 *
	 * But we do need a critical section here to avoid problems in case
	 * multiple calls to pmic_audio_open() are made since we can only allow
	 * one of them to succeed.
	 */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	/* Check the current device handle state and acquire the handle if
	 * it is available.
	 */

	if ((device == STEREO_DAC) && (stDAC.handleState == HANDLE_FREE)) {
		stDAC.handle = (PMIC_AUDIO_HANDLE) (&stDAC);
		stDAC.handleState = HANDLE_IN_USE;
		*handle = stDAC.handle;
		rc = PMIC_SUCCESS;
	} else if ((device == VOICE_CODEC)
		   && (vCodec.handleState == HANDLE_FREE)) {
		vCodec.handle = (PMIC_AUDIO_HANDLE) (&vCodec);
		vCodec.handleState = HANDLE_IN_USE;
		*handle = vCodec.handle;
		rc = PMIC_SUCCESS;
	} else if ((device == EXTERNAL_STEREO_IN) &&
		   (extStereoIn.handleState == HANDLE_FREE)) {
		extStereoIn.handle = (PMIC_AUDIO_HANDLE) (&extStereoIn);
		extStereoIn.handleState = HANDLE_IN_USE;
		*handle = extStereoIn.handle;
		rc = PMIC_SUCCESS;
	} else {
		*handle = AUDIO_HANDLE_NULL;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Terminate further access to the PMIC audio hardware.
 *
 * Terminate further access to the PMIC audio hardware that was previously
 * acquired by calling pmic_audio_open(). This now allows another thread to
 * successfully call pmic_audio_open() to gain access.
 *
 * Note that we will shutdown/reset the Voice CODEC or Stereo DAC as well as
 * any associated audio input/output components that are no longer required.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the close request was successful.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 */
PMIC_STATUS pmic_audio_close(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* We need a critical section here to maintain a consistent state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	/* We can now call pmic_audio_close_handle() to actually do the work. */
	rc = pmic_audio_close_handle(handle);

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Configure the data bus protocol to be used.
 *
 * Provide the parameters needed to properly configure the audio data bus
 * protocol so that data can be read/written to either the Stereo DAC or
 * the Voice CODEC.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   busID           Select data bus to be used.
 * @param   protocol        Select the data bus protocol.
 * @param   masterSlave     Select the data bus timing mode.
 * @param   numSlots        Define the number of timeslots (only if in
 *                              master mode).
 *
 * @retval      PMIC_SUCCESS         If the protocol was successful configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or the protocol parameters
 *                                   are invalid.
 */
PMIC_STATUS pmic_audio_set_protocol(const PMIC_AUDIO_HANDLE handle,
				    const PMIC_AUDIO_DATA_BUS busID,
				    const PMIC_AUDIO_BUS_PROTOCOL protocol,
				    const PMIC_AUDIO_BUS_MODE masterSlave,
				    const PMIC_AUDIO_NUMSLOTS numSlots)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int ST_DAC_MASK = SET_BITS(regST_DAC, STDCSSISEL, 1) |
	    SET_BITS(regST_DAC, STDCFS, 3) | SET_BITS(regST_DAC, STDCSM, 1);

	unsigned int reg_mask;
	/*unsigned int VCODEC_MASK = SET_BITS(regAUD_CODEC, CDCSSISEL, 1) |
	   SET_BITS(regAUD_CODEC, CDCFS, 3) | SET_BITS(regAUD_CODEC, CDCSM, 1); */

	unsigned int SSI_NW_MASK = SET_BITS(regSSI_NETWORK, STDCSLOTS, 1);
	unsigned int reg_value = 0;
	unsigned int ssi_nw_value = 0;

	/* Enter a critical section so that we can ensure only one
	 * state change request is completed at a time.
	 */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (handle == (PMIC_AUDIO_HANDLE) NULL) {
		rc = PMIC_PARAMETER_ERROR;
	} else {
		if ((handle == vCodec.handle) &&
		    (vCodec.handleState == HANDLE_IN_USE)) {
			if ((stDAC.handleState == HANDLE_IN_USE) &&
			    (stDAC.busID == busID) && (stDAC.protocol_set)) {
				pr_debug("The requested bus already in USE\n");
				rc = PMIC_PARAMETER_ERROR;
			} else if ((masterSlave == BUS_MASTER_MODE)
				   && (numSlots != USE_4_TIMESLOTS)) {
				pr_debug
				    ("mc13783 supports only 4 slots in Master mode\n");
				rc = PMIC_NOT_SUPPORTED;
			} else if ((masterSlave == BUS_SLAVE_MODE)
				   && (numSlots != USE_4_TIMESLOTS)) {
				pr_debug
				    ("Driver currently supports only 4 slots in Slave mode\n");
				rc = PMIC_NOT_SUPPORTED;
			} else if (!((protocol == NETWORK_MODE) ||
				     (protocol == I2S_MODE))) {
				pr_debug
				    ("mc13783 Voice codec works only in Network and I2S modes\n");
				rc = PMIC_NOT_SUPPORTED;
			} else {
				pr_debug
				    ("Proceeding to configure Voice Codec\n");
				if (busID == AUDIO_DATA_BUS_1) {
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCSSISEL,
						     0);
				} else {
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCSSISEL,
						     1);
				}
				reg_mask = SET_BITS(regAUD_CODEC, CDCSSISEL, 1);
				if (PMIC_SUCCESS !=
				    pmic_write_reg(REG_AUDIO_CODEC,
						   reg_value, reg_mask))
					return PMIC_ERROR;

				if (masterSlave == BUS_MASTER_MODE) {
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCSM, 0);
				} else {
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCSM, 1);
				}
				reg_mask = SET_BITS(regAUD_CODEC, CDCSM, 1);
				if (PMIC_SUCCESS !=
				    pmic_write_reg(REG_AUDIO_CODEC,
						   reg_value, reg_mask))
					return PMIC_ERROR;

				if (protocol == NETWORK_MODE) {
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCFS, 1);
				} else {	/* protocol == I2S, other options have been already eliminated */
					reg_value =
					    SET_BITS(regAUD_CODEC, CDCFS, 2);
				}
				reg_mask = SET_BITS(regAUD_CODEC, CDCFS, 3);
				if (PMIC_SUCCESS !=
				    pmic_write_reg(REG_AUDIO_CODEC,
						   reg_value, reg_mask))
					return PMIC_ERROR;

				ssi_nw_value =
				    SET_BITS(regSSI_NETWORK, CDCFSDLY, 1);
				/*if (pmic_write_reg
				   (REG_AUDIO_CODEC, reg_value,
				   VCODEC_MASK) != PMIC_SUCCESS) {
				   rc = PMIC_ERROR;
				   } else { */
				vCodec.busID = busID;
				vCodec.protocol = protocol;
				vCodec.masterSlave = masterSlave;
				vCodec.numSlots = numSlots;
				vCodec.protocol_set = true;

				pr_debug
				    ("mc13783 Voice codec successfully configured\n");
				rc = PMIC_SUCCESS;
			}

		} else if ((handle == stDAC.handle) &&
			   (stDAC.handleState == HANDLE_IN_USE)) {
			if ((vCodec.handleState == HANDLE_IN_USE) &&
			    (vCodec.busID == busID) && (vCodec.protocol_set)) {
				pr_debug("The requested bus already in USE\n");
				rc = PMIC_PARAMETER_ERROR;
			} else if (((protocol == NORMAL_MSB_JUSTIFIED_MODE) ||
				    (protocol == I2S_MODE))
				   && (numSlots != USE_2_TIMESLOTS)) {
				pr_debug
				    ("STDAC uses only 2 slots in Normal and I2S modes\n");
				rc = PMIC_PARAMETER_ERROR;
			} else if ((protocol == NETWORK_MODE) &&
				   !((numSlots == USE_2_TIMESLOTS) ||
				     (numSlots == USE_4_TIMESLOTS) ||
				     (numSlots == USE_8_TIMESLOTS))) {
				pr_debug
				    ("STDAC uses only 2,4 or 8 slots in Network mode\n");
				rc = PMIC_PARAMETER_ERROR;
			} else if (protocol == SPD_IF_MODE) {
				pr_debug
				    ("STDAC driver currently does not support SPD IF mode\n");
				rc = PMIC_NOT_SUPPORTED;
			} else {
				pr_debug
				    ("Proceeding to configure Stereo DAC\n");
				if (busID == AUDIO_DATA_BUS_1) {
					reg_value =
					    SET_BITS(regST_DAC, STDCSSISEL, 0);
				} else {
					reg_value =
					    SET_BITS(regST_DAC, STDCSSISEL, 1);
				}
				if (masterSlave == BUS_MASTER_MODE) {
					reg_value |=
					    SET_BITS(regST_DAC, STDCSM, 0);
				} else {
					reg_value |=
					    SET_BITS(regST_DAC, STDCSM, 1);
				}
				if (protocol == NETWORK_MODE) {
					reg_value |=
					    SET_BITS(regST_DAC, STDCFS, 1);
				} else if (protocol ==
					   NORMAL_MSB_JUSTIFIED_MODE) {
					reg_value |=
					    SET_BITS(regST_DAC, STDCFS, 0);
				} else {	/* I2S mode as the other option has already been eliminated */
					reg_value |=
					    SET_BITS(regST_DAC, STDCFS, 2);
				}

				if (pmic_write_reg
				    (REG_AUDIO_STEREO_DAC,
				     reg_value, ST_DAC_MASK) != PMIC_SUCCESS) {
					rc = PMIC_ERROR;
				} else {
					if (numSlots == USE_2_TIMESLOTS) {
						reg_value =
						    SET_BITS(regSSI_NETWORK,
							     STDCSLOTS, 3);
					} else if (numSlots == USE_4_TIMESLOTS) {
						reg_value =
						    SET_BITS(regSSI_NETWORK,
							     STDCSLOTS, 2);
					} else {	/* Use 8 timeslots - L , R and 6 other */
						reg_value =
						    SET_BITS(regSSI_NETWORK,
							     STDCSLOTS, 1);
					}
					if (pmic_write_reg
					    (REG_AUDIO_SSI_NETWORK,
					     reg_value,
					     SSI_NW_MASK) != PMIC_SUCCESS) {
						rc = PMIC_ERROR;
					} else {
						stDAC.busID = busID;
						stDAC.protocol = protocol;
						stDAC.protocol_set = true;
						stDAC.masterSlave = masterSlave;
						stDAC.numSlots = numSlots;
						pr_debug
						    ("mc13783 Stereo DAC successfully configured\n");
						rc = PMIC_SUCCESS;
					}
				}

			}
		} else {
			rc = PMIC_PARAMETER_ERROR;
			/* Handle  can only be Voice Codec or Stereo DAC */
			pr_debug("Handles only STDAC and VCODEC\n");
		}

	}
	/* Exit critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Retrieve the current data bus protocol configuration.
 *
 * Retrieve the parameters that define the current audio data bus protocol.
 *
 * @param  handle          Device handle from pmic_audio_open() call.
 * @param  busID           The data bus being used.
 * @param  protocol        The data bus protocol being used.
 * @param  masterSlave     The data bus timing mode being used.
 * @param  numSlots        The number of timeslots being used (if in
 *                              master mode).
 *
 * @retval      PMIC_SUCCESS         If the protocol was successful retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 */
PMIC_STATUS pmic_audio_get_protocol(const PMIC_AUDIO_HANDLE handle,
				    PMIC_AUDIO_DATA_BUS * const busID,
				    PMIC_AUDIO_BUS_PROTOCOL * const protocol,
				    PMIC_AUDIO_BUS_MODE * const masterSlave,
				    PMIC_AUDIO_NUMSLOTS * const numSlots)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	if ((busID != (PMIC_AUDIO_DATA_BUS *) NULL) &&
	    (protocol != (PMIC_AUDIO_BUS_PROTOCOL *) NULL) &&
	    (masterSlave != (PMIC_AUDIO_BUS_MODE *) NULL) &&
	    (numSlots != (PMIC_AUDIO_NUMSLOTS *) NULL)) {
		/* Enter a critical section so that we return a consistent state. */
		if (down_interruptible(&mutex))
			return PMIC_SYSTEM_ERROR_EINTR;

		if ((handle == stDAC.handle) &&
		    (stDAC.handleState == HANDLE_IN_USE)) {
			*busID = stDAC.busID;
			*protocol = stDAC.protocol;
			*masterSlave = stDAC.masterSlave;
			*numSlots = stDAC.numSlots;
			rc = PMIC_SUCCESS;
		} else if ((handle == vCodec.handle) &&
			   (vCodec.handleState == HANDLE_IN_USE)) {
			*busID = vCodec.busID;
			*protocol = vCodec.protocol;
			*masterSlave = vCodec.masterSlave;
			*numSlots = vCodec.numSlots;
			rc = PMIC_SUCCESS;
		}

		/* Exit critical section. */
		up(&mutex);
	}

	return rc;
}

/*!
 * @brief Enable the Stereo DAC or the Voice CODEC.
 *
 * Explicitly enable the Stereo DAC or the Voice CODEC to begin audio
 * playback or recording as required. This should only be done after
 * successfully configuring all of the associated audio components (e.g.,
 * microphones, amplifiers, etc.).
 *
 * Note that the timed delays used in this function are necessary to
 * ensure reliable operation of the Voice CODEC and Stereo DAC. The
 * Stereo DAC seems to be particularly sensitive and it has been observed
 * to fail to generate the required master mode clock signals if it is
 * not allowed enough time to initialize properly.
 *
 * @param      handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the device was successful enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 * @retval      PMIC_ERROR           If the device could not be enabled.
 */
PMIC_STATUS pmic_audio_enable(const PMIC_AUDIO_HANDLE handle)
{
	const unsigned int AUDIO_BIAS_ENABLE = SET_BITS(regAUDIO_RX_0,
							VAUDIOON, 1);
	const unsigned int STDAC_ENABLE = SET_BITS(regST_DAC, STDCEN, 1);
	const unsigned int VCODEC_ENABLE = SET_BITS(regAUD_CODEC, CDCEN, 1);
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		pmic_write_reg(REG_AUDIO_RX_0, AUDIO_BIAS_ENABLE,
			       AUDIO_BIAS_ENABLE);
		reg_mask =
		    SET_BITS(regAUDIO_RX_0, HSDETEN,
			     1) | SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
		reg_write =
		    SET_BITS(regAUDIO_RX_0, HSDETEN,
			     1) | SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
		rc = pmic_write_reg(REG_AUDIO_RX_0, reg_write, reg_mask);
		if (rc == PMIC_SUCCESS)
			pr_debug("pmic_audio_enable\n");
		/* We can enable the Stereo DAC. */
		rc = pmic_write_reg(REG_AUDIO_STEREO_DAC,
				    STDAC_ENABLE, STDAC_ENABLE);
		/*pmic_read_reg(REG_AUDIO_STEREO_DAC, &reg_value); */
		if (rc != PMIC_SUCCESS) {
			pr_debug("Failed to enable the Stereo DAC\n");
			rc = PMIC_ERROR;
		}
	} else if ((handle == vCodec.handle)
		   && (vCodec.handleState == HANDLE_IN_USE)) {
		/* Must first set the audio bias bit to power up the audio circuits. */
		pmic_write_reg(REG_AUDIO_RX_0, AUDIO_BIAS_ENABLE,
			       AUDIO_BIAS_ENABLE);
		reg_mask = SET_BITS(regAUDIO_RX_0, HSDETEN, 1) |
		    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
		reg_write = SET_BITS(regAUDIO_RX_0, HSDETEN, 1) |
		    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
		rc = pmic_write_reg(REG_AUDIO_RX_0, reg_write, reg_mask);

		/* Then we can enable the Voice CODEC. */
		rc = pmic_write_reg(REG_AUDIO_CODEC, VCODEC_ENABLE,
				    VCODEC_ENABLE);

		/* pmic_read_reg(REG_AUDIO_CODEC, &reg_value); */
		if (rc != PMIC_SUCCESS) {
			pr_debug("Failed to enable the Voice codec\n");
			rc = PMIC_ERROR;
		}
	}
	/* Exit critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Disable the Stereo DAC or the Voice CODEC.
 *
 * Explicitly disable the Stereo DAC or the Voice CODEC to end audio
 * playback or recording as required.
 *
 * @param   	handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the device was successful disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 * @retval      PMIC_ERROR           If the device could not be disabled.
 */
PMIC_STATUS pmic_audio_disable(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int STDAC_DISABLE = SET_BITS(regST_DAC, STDCEN, 1);
	const unsigned int VCODEC_DISABLE = SET_BITS(regAUD_CODEC, CDCEN, 1);

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;
	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		rc = pmic_write_reg(REG_AUDIO_STEREO_DAC, 0, STDAC_DISABLE);
	} else if ((handle == vCodec.handle)
		   && (vCodec.handleState == HANDLE_IN_USE)) {
		rc = pmic_write_reg(REG_AUDIO_CODEC, 0, VCODEC_DISABLE);
	}
	if (rc == PMIC_SUCCESS) {
		pr_debug("Disabled successfully\n");
	}
	/* Exit critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Reset the selected audio hardware control registers to their
 *        power on state.
 *
 * This resets all of the audio hardware control registers currently
 * associated with the device handle back to their power on states. For
 * example, if the handle is associated with the Stereo DAC and a
 * specific output port and output amplifiers, then this function will
 * reset all of those components to their initial power on state.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the reset operation was successful.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 * @retval      PMIC_ERROR           If the reset was unsuccessful.
 */
PMIC_STATUS pmic_audio_reset(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	rc = pmic_audio_reset_device(handle);

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Reset all audio hardware control registers to their power on state.
 *
 * This resets all of the audio hardware control registers back to their
 * power on states. Use this function with care since it also invalidates
 * (i.e., automatically closes) all currently opened device handles.
 *
 * @retval      PMIC_SUCCESS         If the reset operation was successful.
 * @retval      PMIC_ERROR           If the reset was unsuccessful.
 */
PMIC_STATUS pmic_audio_reset_all(void)
{
	PMIC_STATUS rc = PMIC_SUCCESS;
	unsigned int audio_ssi_reset = 0;
	unsigned int audio_rx1_reset = 0;
	/* We need a critical section here to maintain a consistent state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	/* First close all opened device handles, also deregisters callbacks. */
	pmic_audio_close_handle(stDAC.handle);
	pmic_audio_close_handle(vCodec.handle);
	pmic_audio_close_handle(extStereoIn.handle);

	if (pmic_write_reg(REG_AUDIO_RX_1, RESET_AUDIO_RX_1,
			   PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		audio_rx1_reset = 1;
	}
	if (pmic_write_reg(REG_AUDIO_SSI_NETWORK, RESET_SSI_NETWORK,
			   PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		audio_ssi_reset = 1;
	}
	if (pmic_write_reg
	    (REG_AUDIO_STEREO_DAC, RESET_ST_DAC,
	     PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		/* Also reset the driver state information to match. Note that we
		 * keep the device handle and event callback settings unchanged
		 * since these don't affect the actual hardware and we rely on
		 * the user to explicitly close the handle or deregister callbacks
		 */
		if (audio_ssi_reset) {
			/* better to check if SSI is also reset as some fields are represennted in SSI reg */
			stDAC.busID = AUDIO_DATA_BUS_1;
			stDAC.protocol = NORMAL_MSB_JUSTIFIED_MODE;
			stDAC.masterSlave = BUS_MASTER_MODE;
			stDAC.protocol_set = false;
			stDAC.numSlots = USE_2_TIMESLOTS;
			stDAC.clockIn = CLOCK_IN_CLIA;
			stDAC.samplingRate = STDAC_RATE_44_1_KHZ;
			stDAC.clockFreq = STDAC_CLI_13MHZ;
			stDAC.invert = NO_INVERT;
			stDAC.timeslot = USE_TS0_TS1;
			stDAC.config = (PMIC_AUDIO_STDAC_CONFIG) 0;
		}
	}

	if (pmic_write_reg(REG_AUDIO_CODEC, RESET_AUD_CODEC,
			   PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		/* Also reset the driver state information to match. Note that we
		 * keep the device handle and event callback settings unchanged
		 * since these don't affect the actual hardware and we rely on
		 * the user to explicitly close the handle or deregister callbacks
		 */
		if (audio_ssi_reset) {
			vCodec.busID = AUDIO_DATA_BUS_2;
			vCodec.protocol = NETWORK_MODE;
			vCodec.masterSlave = BUS_SLAVE_MODE;
			vCodec.protocol_set = false;
			vCodec.numSlots = USE_4_TIMESLOTS;
			vCodec.clockIn = CLOCK_IN_CLIB;
			vCodec.samplingRate = VCODEC_RATE_8_KHZ;
			vCodec.clockFreq = VCODEC_CLI_13MHZ;
			vCodec.invert = NO_INVERT;
			vCodec.timeslot = USE_TS0;
			vCodec.config =
			    INPUT_HIGHPASS_FILTER | OUTPUT_HIGHPASS_FILTER;
		}
	}

	if (pmic_write_reg(REG_AUDIO_RX_0, RESET_AUDIO_RX_0,
			   PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		/* Also reset the driver state information to match. */
		audioOutput.outputPort = (PMIC_AUDIO_OUTPUT_PORT) NULL;
		audioOutput.vCodecoutputPGAGain = OUTPGA_GAIN_0DB;
		audioOutput.stDacoutputPGAGain = OUTPGA_GAIN_0DB;
		audioOutput.extStereooutputPGAGain = OUTPGA_GAIN_0DB;
		audioOutput.balanceLeftGain = BAL_GAIN_0DB;
		audioOutput.balanceRightGain = BAL_GAIN_0DB;
		audioOutput.monoAdderGain = MONOADD_GAIN_0DB;
		audioOutput.config = (PMIC_AUDIO_OUTPUT_CONFIG) 0;
		audioOutput.vCodecOut = VCODEC_DIRECT_OUT;
	}

	if (pmic_write_reg(REG_AUDIO_TX, RESET_AUDIO_TX,
			   PMIC_ALL_BITS) != PMIC_SUCCESS) {
		rc = PMIC_ERROR;
	} else {
		/* Also reset the driver state information to match. Note that we
		 * reset the vCodec fields since all of the input/recording
		 * devices are only connected to the Voice CODEC and are managed
		 * as part of the Voice CODEC state.
		 */
		if (audio_rx1_reset) {
			vCodec.leftChannelMic.mic = NO_MIC;
			vCodec.leftChannelMic.micOnOff = MICROPHONE_OFF;
			vCodec.leftChannelMic.ampMode = CURRENT_TO_VOLTAGE;
			vCodec.leftChannelMic.gain = MIC_GAIN_0DB;
			vCodec.rightChannelMic.mic = NO_MIC;
			vCodec.rightChannelMic.micOnOff = MICROPHONE_OFF;
			vCodec.rightChannelMic.ampMode = AMP_OFF;
			vCodec.rightChannelMic.gain = MIC_GAIN_0DB;
		}
	}
	/* Finally, also reset any global state variables. */
	headsetState = NO_HEADSET;
	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Set the Audio callback function.
 *
 * Register a callback function that will be used to signal PMIC audio
 * events. For example, the OSS audio driver should register a callback
 * function in order to be notified of headset connect/disconnect events.
 *
 * @param   func            A pointer to the callback function.
 * @param   eventMask       A mask selecting events to be notified.
 * @param   hs_state        To know the headset state.
 *
 *
 *
 * @retval      PMIC_SUCCESS         If the callback was successfully
 *                                   registered.
 * @retval      PMIC_PARAMETER_ERROR If the handle or the eventMask is invalid.
 */
PMIC_STATUS pmic_audio_set_callback(void *func,
				    const PMIC_AUDIO_EVENTS eventMask,
				    PMIC_HS_STATE *hs_state)
{
	unsigned long flags;
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	pmic_event_callback_t eventNotify;

	/* We need to start a critical section here to ensure a consistent state
	 * in case simultaneous calls to pmic_audio_set_callback() are made. In
	 * that case, we must serialize the calls to ensure that the "callback"
	 * and "eventMask" state variables are always consistent.
	 *
	 * Note that we don't actually need to acquire the spinlock until later
	 * when we are finally ready to update the "callback" and "eventMask"
	 * state variables which are shared with the interrupt handler.
	 */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	rc = PMIC_ERROR;
	/* Register for PMIC events from the core protocol driver. */
	if (eventMask & MICROPHONE_DETECTED) {
		/* We need to register for the A1 amplifier interrupt. */
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_MC2BI);
		rc = pmic_event_subscribe(EVENT_MC2BI, eventNotify);

		if (rc != PMIC_SUCCESS) {
			pr_debug
			    ("%s: pmic_event_subscribe() for EVENT_HSDETI "
			     "failed\n", __FILE__);
			goto End;
		}
	}

	if (eventMask & HEADSET_DETECTED) {
		/* We need to register for the A1 amplifier interrupt. */
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_HSDETI);
		rc = pmic_event_subscribe(EVENT_HSDETI, eventNotify);

		if (rc != PMIC_SUCCESS) {
			pr_debug
			    ("%s: pmic_event_subscribe() for EVENT_HSDETI "
			     "failed\n", __FILE__);
			goto Cleanup_HDT;
		}

	}
	if (eventMask & HEADSET_STEREO) {
		/* We need to register for the A1 amplifier interrupt. */
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_HSLI);
		rc = pmic_event_subscribe(EVENT_HSLI, eventNotify);

		if (rc != PMIC_SUCCESS) {
			pr_debug
			    ("%s: pmic_event_subscribe() for EVENT_HSLI "
			     "failed\n", __FILE__);
			goto Cleanup_HST;
		}
	}
	if (eventMask & HEADSET_THERMAL_SHUTDOWN) {
		/* We need to register for the A1 amplifier interrupt. */
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_ALSPTHI);
		rc = pmic_event_subscribe(EVENT_ALSPTHI, eventNotify);

		if (rc != PMIC_SUCCESS) {
			pr_debug
			    ("%s: pmic_event_subscribe() for EVENT_ALSPTHI "
			     "failed\n", __FILE__);
			goto Cleanup_TSD;
		}
		pr_debug("Registered for EVENT_ALSPTHI\n");
	}
	if (eventMask & HEADSET_SHORT_CIRCUIT) {
		/* We need to register for the A1 amplifier interrupt. */
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_AHSSHORTI);
		rc = pmic_event_subscribe(EVENT_AHSSHORTI, eventNotify);

		if (rc != PMIC_SUCCESS) {
			pr_debug
			    ("%s: pmic_event_subscribe() for EVENT_AHSSHORTI "
			     "failed\n", __FILE__);
			goto Cleanup_HShort;
		}
		pr_debug("Registered for EVENT_AHSSHORTI\n");
	}

	/* We also need the spinlock here to avoid possible problems
	 * with the interrupt handler  when we update the
	 * "callback" and "eventMask" state variables.
	 */
	spin_lock_irqsave(&lock, flags);

	/* Successfully registered for all events. */
	event_state.callback = func;
	event_state.eventMask = eventMask;

	/* The spinlock is no longer needed now that we've finished
	 * updating the "callback" and "eventMask" state variables.
	 */
	spin_unlock_irqrestore(&lock, flags);

	goto End;

	/* This section unregisters any already registered events if we should
	 * encounter an error partway through the registration process. Note
	 * that we don't check the return status here since it is already set
	 * to PMIC_ERROR before we get here.
	 */
      Cleanup_HShort:

	if (eventMask & HEADSET_SHORT_CIRCUIT) {
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_AHSSHORTI);
		pmic_event_unsubscribe(EVENT_AHSSHORTI, eventNotify);
	}

      Cleanup_TSD:

	if (eventMask & HEADSET_THERMAL_SHUTDOWN) {
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_ALSPTHI);
		pmic_event_unsubscribe(EVENT_ALSPTHI, eventNotify);
	}

      Cleanup_HST:

	if (eventMask & HEADSET_STEREO) {
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_HSLI);
		pmic_event_unsubscribe(EVENT_HSLI, eventNotify);
	}

      Cleanup_HDT:

	if (eventMask & HEADSET_DETECTED) {
		eventNotify.func = func;
		eventNotify.param = (void *)(CORE_EVENT_HSDETI);
		pmic_event_unsubscribe(EVENT_HSDETI, eventNotify);
	}

      End:
	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Deregisters the existing audio callback function.
 *
 * Deregister the callback function that was previously registered by calling
 * pmic_audio_set_callback().
 *
 *
 * @retval      PMIC_SUCCESS         If the callback was successfully
 *                                   deregistered.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 */
PMIC_STATUS pmic_audio_clear_callback(void)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* We need a critical section to maintain a consistent state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (event_state.callback != (PMIC_AUDIO_CALLBACK) NULL) {
		rc = pmic_audio_deregister(&(event_state.callback),
					   &(event_state.eventMask));
	}

	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Get the current audio callback function settings.
 *
 * Get the current callback function and event mask.
 *
 * @param   func            The current callback function.
 * @param   eventMask       The current event selection mask.
 *
 * @retval      PMIC_SUCCESS         If the callback information was
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 */
PMIC_STATUS pmic_audio_get_callback(PMIC_AUDIO_CALLBACK * const func,
				    PMIC_AUDIO_EVENTS * const eventMask)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* We only need to acquire the mutex here because we will not be updating
	 * anything that may affect the interrupt handler. We just need to ensure
	 * that the callback fields are not changed while we are in the critical
	 * section by calling either pmic_audio_set_callback() or
	 * pmic_audio_clear_callback().
	 */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((func != (PMIC_AUDIO_CALLBACK *) NULL) &&
	    (eventMask != (PMIC_AUDIO_EVENTS *) NULL)) {

		*func = event_state.callback;
		*eventMask = event_state.eventMask;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Enable the anti-pop circuitry to avoid extra noise when inserting
 *        or removing a external device (e.g., a headset).
 *
 * Enable the use of the built-in anti-pop circuitry to prevent noise from
 * being generated when an external audio device is inserted or removed
 * from an audio plug. A slow ramp speed may be needed to avoid extra noise.
 *
 * @param       rampSpeed       The desired anti-pop circuitry ramp speed.
 *
 * @retval      PMIC_SUCCESS         If the anti-pop circuitry was successfully
 *                                   enabled.
 * @retval      PMIC_ERROR           If the anti-pop circuitry could not be
 *                                   enabled.
 */
PMIC_STATUS pmic_audio_antipop_enable(const PMIC_AUDIO_ANTI_POP_RAMP_SPEED
				      rampSpeed)
{
	PMIC_STATUS rc = PMIC_ERROR;
	unsigned int reg_value = 0;
	const unsigned int reg_mask = SET_BITS(regAUDIO_RX_0, BIASEN, 1) |
	    SET_BITS(regAUDIO_RX_0, BIASSPEED, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	/*
	 * Antipop is enabled by enabling the BIAS (BIASEN) and setting the
	 * BIASSPEED .
	 * BIASEN is just to make sure that BIAS is enabled
	 */
	reg_value = SET_BITS(regAUDIO_RX_0, BIASEN, 1)
	    | SET_BITS(regAUDIO_RX_0, BIASSPEED, 0) | SET_BITS(regAUDIO_RX_0,
							       HSLDETEN, 1);
	rc = pmic_write_reg(REG_AUDIO_RX_0, reg_value, reg_mask);
	return rc;
}

/*!
 * @brief Disable the anti-pop circuitry.
 *
 * Disable the use of the built-in anti-pop circuitry to prevent noise from
 * being generated when an external audio device is inserted or removed
 * from an audio plug.
 *
 * @retval      PMIC_SUCCESS         If the anti-pop circuitry was successfully
 *                                   disabled.
 * @retval      PMIC_ERROR           If the anti-pop circuitry could not be
 *                                   disabled.
 */
PMIC_STATUS pmic_audio_antipop_disable(void)
{
	PMIC_STATUS rc = PMIC_ERROR;
	const unsigned int reg_mask = SET_BITS(regAUDIO_RX_0, BIASSPEED, 1) |
	    SET_BITS(regAUDIO_RX_0, BIASEN, 1);
	const unsigned int reg_write = SET_BITS(regAUDIO_RX_0, BIASSPEED, 1) |
	    SET_BITS(regAUDIO_RX_0, BIASEN, 0);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	/*
	 * Antipop is disabled by setting BIASSPEED  = 0. BIASEN bit remains set
	 * as only antipop needs to be disabled
	 */
	rc = pmic_write_reg(REG_AUDIO_RX_0, reg_write, reg_mask);

	return rc;
}

/*!
 * @brief Performs a reset of the Voice CODEC/Stereo DAC digital filter.
 *
 * The digital filter should be reset whenever the clock or sampling rate
 * configuration has been changed.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the digital filter was successfully
 *                                   reset.
 * @retval      PMIC_ERROR           If the digital filter could not be reset.
 */
PMIC_STATUS pmic_audio_digital_filter_reset(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regST_DAC, STDCRESET, 1);
		if (pmic_write_reg(REG_AUDIO_STEREO_DAC, reg_mask,
				   reg_mask) != PMIC_SUCCESS) {
			rc = PMIC_ERROR;
		} else {
			pr_debug("STDAC filter reset\n");
		}

	} else if ((handle == vCodec.handle) &&
		   (vCodec.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regAUD_CODEC, CDCRESET, 1);
		if (pmic_write_reg(REG_AUDIO_CODEC, reg_mask,
				   reg_mask) != PMIC_SUCCESS) {
			rc = PMIC_ERROR;
		} else {
			pr_debug("CODEC filter reset\n");
		}
	}
	return rc;
}

/*!
 * @brief Get the most recent PTT button voltage reading.
 *
 * This feature is not supported by mc13783
 * @param       level                PTT button level.
 *
 * @retval      PMIC_SUCCESS         If the most recent PTT button voltage was
 *                                   returned.
 * @retval      PMIC_PARAMETER_ERROR If a NULL pointer argument was given.
 */
PMIC_STATUS pmic_audio_get_ptt_button_level(unsigned int *const level)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;
}

#ifdef DEBUG_AUDIO

/*!
 * @brief Provide a hexadecimal dump of all PMIC audio registers (DEBUG only)
 *
 * This function is intended strictly for debugging purposes only and will
 * print the current values of the following PMIC registers:
 *
 * - AUD_CODEC
 * - ST_DAC
 * - AUDIO_RX_0
 * - AUDIO_RX_1
 * - AUDIO_TX
 * - AUDIO_SSI_NW
 *
 * The register fields will not be decoded.
 *
 * Note that we don't dump any of the arbitration bits because we cannot
 * access the true arbitration bit settings when reading the registers
 * from the secondary SPI bus.
 *
 * Also note that we must not call this function with interrupts disabled,
 * for example, while holding a spinlock, because calls to pmic_read_reg()
 * eventually end up in the SPI driver which will want to perform a
 * schedule() operation. If schedule() is called with interrupts disabled,
 * then you will see messages like the following:
 *
 * BUG: scheduling while atomic: ...
 *
 */
void pmic_audio_dump_registers(void)
{
	unsigned int reg_value = 0;

	/* Dump the AUD_CODEC (Voice CODEC) register. */
	if (pmic_read_reg(REG_AUDIO_CODEC, &reg_value, REG_FULLMASK)
	    == PMIC_SUCCESS) {
		pr_debug("Audio Codec = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read audio codec\n");
	}

	/* Dump the ST DAC (Stereo DAC) register. */
	if (pmic_read_reg
	    (REG_AUDIO_STEREO_DAC, &reg_value, REG_FULLMASK) == PMIC_SUCCESS) {
		pr_debug("Stereo DAC = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read Stereo DAC\n");
	}

	/* Dump the SSI NW register. */
	if (pmic_read_reg
	    (REG_AUDIO_SSI_NETWORK, &reg_value, REG_FULLMASK) == PMIC_SUCCESS) {
		pr_debug("SSI Network = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read SSI network\n");
	}

	/* Dump the Audio RX 0 register. */
	if (pmic_read_reg(REG_AUDIO_RX_0, &reg_value, REG_FULLMASK)
	    == PMIC_SUCCESS) {
		pr_debug("Audio RX 0 = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read audio RX 0\n");
	}

	/* Dump the Audio RX 1 register. */
	if (pmic_read_reg(REG_AUDIO_RX_1, &reg_value, REG_FULLMASK)
	    == PMIC_SUCCESS) {
		pr_debug("Audio RX 1 = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read audio RX 1\n");
	}
	/* Dump the Audio TX register. */
	if (pmic_read_reg(REG_AUDIO_TX, &reg_value, REG_FULLMASK) ==
	    PMIC_SUCCESS) {
		pr_debug("Audio Tx = 0x%x\n", reg_value);
	} else {
		pr_debug("Failed to read audio TX\n");
	}

}

#endif				/* DEBUG_AUDIO */

/*@}*/

/*************************************************************************
 * General Voice CODEC configuration.
 *************************************************************************
 */

/*!
 * @name General Voice CODEC Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Voice
 * CODEC hardware.
 */
/*@{*/

/*!
 * @brief Set the Voice CODEC clock source and operating characteristics.
 *
 * Define the Voice CODEC clock source and operating characteristics. This
 * must be done before the Voice CODEC is enabled.
 *
 *
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       clockIn         Select the clock signal source.
 * @param       clockFreq       Select the clock signal frequency.
 * @param       samplingRate    Select the audio data sampling rate.
 * @param       invert          Enable inversion of the frame sync and/or
 *                              bit clock inputs.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC clock settings were
 *                                   successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or clock configuration was
 *                                   invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC clock configuration
 *                                   could not be set.
 */
PMIC_STATUS pmic_audio_vcodec_set_clock(const PMIC_AUDIO_HANDLE handle,
					const PMIC_AUDIO_CLOCK_IN_SOURCE
					clockIn,
					const PMIC_AUDIO_VCODEC_CLOCK_IN_FREQ
					clockFreq,
					const PMIC_AUDIO_VCODEC_SAMPLING_RATE
					samplingRate,
					const PMIC_AUDIO_CLOCK_INVERT invert)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_value = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	/* Validate all of the calling parameters. */
	if (handle == (PMIC_AUDIO_HANDLE) NULL) {
		rc = PMIC_PARAMETER_ERROR;
	} else if ((handle == vCodec.handle) &&
		   (vCodec.handleState == HANDLE_IN_USE)) {
		if ((clockIn != CLOCK_IN_CLIA) && (clockIn != CLOCK_IN_CLIB)) {
			rc = PMIC_PARAMETER_ERROR;
		} else if (!((clockFreq >= VCODEC_CLI_13MHZ)
			     && (clockFreq <= VCODEC_CLI_33_6MHZ))) {
			rc = PMIC_PARAMETER_ERROR;
		} else if ((samplingRate != VCODEC_RATE_8_KHZ)
			   && (samplingRate != VCODEC_RATE_16_KHZ)) {
			rc = PMIC_PARAMETER_ERROR;
		} else if (!((invert >= NO_INVERT)
			     && (invert <= INVERT_FRAMESYNC))) {
			rc = PMIC_PARAMETER_ERROR;
		} else {
			/*reg_mask = SET_BITS(regAUD_CODEC, CDCCLK, 7) |
			   SET_BITS(regAUD_CODEC, CDCCLKSEL, 1) |
			   SET_BITS(regAUD_CODEC, CDCFS8K16K, 1) |
			   SET_BITS(regAUD_CODEC, CDCBCLINV, 1) |
			   SET_BITS(regAUD_CODEC, CDCFSINV, 1); */
			if (clockIn == CLOCK_IN_CLIA) {
				reg_value =
				    SET_BITS(regAUD_CODEC, CDCCLKSEL, 0);
			} else {
				reg_value =
				    SET_BITS(regAUD_CODEC, CDCCLKSEL, 1);
			}
			reg_mask = SET_BITS(regAUD_CODEC, CDCCLKSEL, 1);
			if (PMIC_SUCCESS !=
			    pmic_write_reg(REG_AUDIO_CODEC,
					   reg_value, reg_mask))
				return PMIC_ERROR;

			reg_value = 0;
			if (clockFreq == VCODEC_CLI_13MHZ) {
				reg_value |= SET_BITS(regAUD_CODEC, CDCCLK, 0);
			} else if (clockFreq == VCODEC_CLI_15_36MHZ) {
				reg_value |= SET_BITS(regAUD_CODEC, CDCCLK, 1);
			} else if (clockFreq == VCODEC_CLI_16_8MHZ) {
				reg_value |= SET_BITS(regAUD_CODEC, CDCCLK, 2);
			} else if (clockFreq == VCODEC_CLI_26MHZ) {
				reg_value |= SET_BITS(regAUD_CODEC, CDCCLK, 4);
			} else {
				reg_value |= SET_BITS(regAUD_CODEC, CDCCLK, 7);
			}
			reg_mask = SET_BITS(regAUD_CODEC, CDCCLK, 7);
			if (PMIC_SUCCESS !=
			    pmic_write_reg(REG_AUDIO_CODEC,
					   reg_value, reg_mask))
				return PMIC_ERROR;

			reg_value = 0;
			reg_mask = 0;

			if (samplingRate == VCODEC_RATE_8_KHZ) {
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCFS8K16K, 0);
			} else {
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCFS8K16K, 1);
			}
			reg_mask = SET_BITS(regAUD_CODEC, CDCFS8K16K, 1);
			if (PMIC_SUCCESS !=
			    pmic_write_reg(REG_AUDIO_CODEC,
					   reg_value, reg_mask))
				return PMIC_ERROR;
			reg_value = 0;
			reg_mask =
			    SET_BITS(regAUD_CODEC, CDCBCLINV,
				     1) | SET_BITS(regAUD_CODEC, CDCFSINV, 1);

			if (invert & INVERT_BITCLOCK) {
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCBCLINV, 1);
			}
			if (invert & INVERT_FRAMESYNC) {
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCFSINV, 1);
			}
			if (invert & NO_INVERT) {
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCBCLINV, 0);
				reg_value |=
				    SET_BITS(regAUD_CODEC, CDCFSINV, 0);
			}
			if (pmic_write_reg
			    (REG_AUDIO_CODEC, reg_value,
			     reg_mask) != PMIC_SUCCESS) {
				rc = PMIC_ERROR;
			} else {
				pr_debug("CODEC clock set\n");
				vCodec.clockIn = clockIn;
				vCodec.clockFreq = clockFreq;
				vCodec.samplingRate = samplingRate;
				vCodec.invert = invert;
			}

		}

	} else {
		rc = PMIC_PARAMETER_ERROR;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the Voice CODEC clock source and operating characteristics.
 *
 * Get the current Voice CODEC clock source and operating characteristics.
 *
 * @param  	handle          Device handle from pmic_audio_open() call.
 * @param  	clockIn         The clock signal source.
 * @param  	clockFreq       The clock signal frequency.
 * @param  	samplingRate    The audio data sampling rate.
 * @param       invert          Inversion of the frame sync and/or
 *                              bit clock inputs is enabled/disabled.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC clock settings were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC clock configuration
 *                                   could not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_clock(const PMIC_AUDIO_HANDLE handle,
					PMIC_AUDIO_CLOCK_IN_SOURCE *
					const clockIn,
					PMIC_AUDIO_VCODEC_CLOCK_IN_FREQ *
					const clockFreq,
					PMIC_AUDIO_VCODEC_SAMPLING_RATE *
					const samplingRate,
					PMIC_AUDIO_CLOCK_INVERT * const invert)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure that we return a consistent state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (clockIn != (PMIC_AUDIO_CLOCK_IN_SOURCE *) NULL) &&
	    (clockFreq != (PMIC_AUDIO_VCODEC_CLOCK_IN_FREQ *) NULL) &&
	    (samplingRate != (PMIC_AUDIO_VCODEC_SAMPLING_RATE *) NULL) &&
	    (invert != (PMIC_AUDIO_CLOCK_INVERT *) NULL)) {
		*clockIn = vCodec.clockIn;
		*clockFreq = vCodec.clockFreq;
		*samplingRate = vCodec.samplingRate;
		*invert = vCodec.invert;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the Voice CODEC primary audio channel timeslot.
 *
 * Set the Voice CODEC primary audio channel timeslot. This function must be
 * used if the default timeslot for the primary audio channel is to be changed.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       timeslot        Select the primary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC primary audio channel
 *                                   timeslot was successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or audio channel timeslot
 *                                   was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC primary audio channel
 *                                   timeslot could not be set.
 */
PMIC_STATUS pmic_audio_vcodec_set_rxtx_timeslot(const PMIC_AUDIO_HANDLE handle,
						const PMIC_AUDIO_VCODEC_TIMESLOT
						timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	const unsigned int reg_mask = SET_BITS(regSSI_NETWORK, CDCTXRXSLOT, 3);

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    ((timeslot == USE_TS0) || (timeslot == USE_TS1) ||
	     (timeslot == USE_TS2) || (timeslot == USE_TS3))) {
		reg_write = SET_BITS(regSSI_NETWORK, CDCTXRXSLOT, timeslot);

		rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK, reg_write, reg_mask);

		if (rc == PMIC_SUCCESS) {
			vCodec.timeslot = timeslot;
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current Voice CODEC primary audio channel timeslot.
 *
 * Get the current Voice CODEC primary audio channel timeslot.
 *
 * @param	handle          Device handle from pmic_audio_open() call.
 * @param       timeslot        The primary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC primary audio channel
 *                                   timeslot was successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC primary audio channel
 *                                   timeslot could not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_rxtx_timeslot(const PMIC_AUDIO_HANDLE handle,
						PMIC_AUDIO_VCODEC_TIMESLOT *
						const timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (timeslot != (PMIC_AUDIO_VCODEC_TIMESLOT *) NULL)) {
		*timeslot = vCodec.timeslot;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the Voice CODEC secondary recording audio channel timeslot.
 *
 * Set the Voice CODEC secondary audio channel timeslot. This function must be
 * used if the default timeslot for the secondary audio channel is to be
 * changed. The secondary audio channel timeslot is used to transmit the audio
 * data that was recorded by the Voice CODEC from the secondary audio input
 * channel.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   timeslot        Select the secondary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC secondary audio channel
 *                                   timeslot was successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or audio channel timeslot
 *                                   was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC secondary audio channel
 *                                   timeslot could not be set.
 */
PMIC_STATUS pmic_audio_vcodec_set_secondary_txslot(const PMIC_AUDIO_HANDLE
						   handle,
						   const
						   PMIC_AUDIO_VCODEC_TIMESLOT
						   timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = SET_BITS(regSSI_NETWORK, CDCTXSECSLOT, 3);
	unsigned int reg_write = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		/* How to handle primary slot and secondary slot being the same */
		if ((timeslot >= USE_TS0) && (timeslot <= USE_TS3)
		    && (timeslot != vCodec.timeslot)) {
			reg_write =
			    SET_BITS(regSSI_NETWORK, CDCTXSECSLOT, timeslot);

			rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				vCodec.secondaryTXtimeslot = timeslot;
			}
		}

	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the Voice CODEC secondary recording audio channel timeslot.
 *
 * Get the Voice CODEC secondary audio channel timeslot.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       timeslot        The secondary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC secondary audio channel
 *                                   timeslot was successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC secondary audio channel
 *                                   timeslot could not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_secondary_txslot(const PMIC_AUDIO_HANDLE
						   handle,
						   PMIC_AUDIO_VCODEC_TIMESLOT *
						   const timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (timeslot != (PMIC_AUDIO_VCODEC_TIMESLOT *) NULL)) {
		rc = PMIC_SUCCESS;
		*timeslot = vCodec.secondaryTXtimeslot;
	}

	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Set/Enable the Voice CODEC options.
 *
 * Set or enable various Voice CODEC options. The available options include
 * the use of dithering, highpass digital filters, and loopback modes.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       config          The Voice CODEC options to enable.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC options were
 *                                   successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or Voice CODEC options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC options could not be
 *                                   successfully set/enabled.
 */
PMIC_STATUS pmic_audio_vcodec_set_config(const PMIC_AUDIO_HANDLE handle,
					 const PMIC_AUDIO_VCODEC_CONFIG config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (config & DITHERING) {
			reg_write = SET_BITS(regAUD_CODEC, CDCDITH, 0);
			reg_mask = SET_BITS(regAUD_CODEC, CDCDITH, 1);
		}

		if (config & INPUT_HIGHPASS_FILTER) {
			reg_write |= SET_BITS(regAUD_CODEC, AUDIHPF, 1);
			reg_mask |= SET_BITS(regAUD_CODEC, AUDIHPF, 1);
		}

		if (config & OUTPUT_HIGHPASS_FILTER) {
			reg_write |= SET_BITS(regAUD_CODEC, AUDOHPF, 1);
			reg_mask |= SET_BITS(regAUD_CODEC, AUDOHPF, 1);
		}

		if (config & DIGITAL_LOOPBACK) {
			reg_write |= SET_BITS(regAUD_CODEC, CDCDLM, 1);
			reg_mask |= SET_BITS(regAUD_CODEC, CDCDLM, 1);
		}

		if (config & ANALOG_LOOPBACK) {
			reg_write |= SET_BITS(regAUD_CODEC, CDCALM, 1);
			reg_mask |= SET_BITS(regAUD_CODEC, CDCALM, 1);
		}

		if (config & VCODEC_MASTER_CLOCK_OUTPUTS) {
			reg_write |= SET_BITS(regAUD_CODEC, CDCCLKEN, 1) |
			    SET_BITS(regAUD_CODEC, CDCTS, 0);
			reg_mask |= SET_BITS(regAUD_CODEC, CDCCLKEN, 1) |
			    SET_BITS(regAUD_CODEC, CDCTS, 1);

		}

		if (config & TRISTATE_TS) {
			reg_write |= SET_BITS(regAUD_CODEC, CDCTS, 1);
			reg_mask |= SET_BITS(regAUD_CODEC, CDCTS, 1);
		}

		if (reg_mask == 0) {
			/* We should not reach this point without having to configure
			 * anything so we flag it as an error.
			 */
			rc = PMIC_ERROR;
		} else {
			rc = pmic_write_reg(REG_AUDIO_CODEC,
					    reg_write, reg_mask);
		}

		if (rc == PMIC_SUCCESS) {
			vCodec.config |= config;
		}
	}
	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Clear/Disable the Voice CODEC options.
 *
 * Clear or disable various Voice CODEC options.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   config          The Voice CODEC options to be cleared/disabled.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC options were
 *                                   successfully cleared/disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or the Voice CODEC options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC options could not be
 *                                   cleared/disabled.
 */
PMIC_STATUS pmic_audio_vcodec_clear_config(const PMIC_AUDIO_HANDLE handle,
					   const PMIC_AUDIO_VCODEC_CONFIG
					   config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (config & DITHERING) {
			reg_mask = SET_BITS(regAUD_CODEC, CDCDITH, 1);
			reg_write = SET_BITS(regAUD_CODEC, CDCDITH, 1);
		}

		if (config & INPUT_HIGHPASS_FILTER) {
			reg_mask |= SET_BITS(regAUD_CODEC, AUDIHPF, 1);
		}

		if (config & OUTPUT_HIGHPASS_FILTER) {
			reg_mask |= SET_BITS(regAUD_CODEC, AUDOHPF, 1);
		}

		if (config & DIGITAL_LOOPBACK) {
			reg_mask |= SET_BITS(regAUD_CODEC, CDCDLM, 1);
		}

		if (config & ANALOG_LOOPBACK) {
			reg_mask |= SET_BITS(regAUD_CODEC, CDCALM, 1);
		}

		if (config & VCODEC_MASTER_CLOCK_OUTPUTS) {
			reg_mask |= SET_BITS(regAUD_CODEC, CDCCLKEN, 1);
		}

		if (config & TRISTATE_TS) {
			reg_mask |= SET_BITS(regAUD_CODEC, CDCTS, 1);
		}

		if (reg_mask == 0) {
			/* We should not reach this point without having to configure
			 * anything so we flag it as an error.
			 */
			rc = PMIC_ERROR;
		} else {
			rc = pmic_write_reg(REG_AUDIO_CODEC,
					    reg_write, reg_mask);
		}

		if (rc == PMIC_SUCCESS) {
			vCodec.config |= config;
		}

	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current Voice CODEC options.
 *
 * Get the current Voice CODEC options.
 *
 * @param   	handle          Device handle from pmic_audio_open() call.
 * @param	config          The current set of Voice CODEC options.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC options were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC options could not be
 *                                   retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_config(const PMIC_AUDIO_HANDLE handle,
					 PMIC_AUDIO_VCODEC_CONFIG *
					 const config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (config != (PMIC_AUDIO_VCODEC_CONFIG *) NULL)) {
		*config = vCodec.config;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Enable the Voice CODEC bypass audio pathway.
 *
 * Enables the Voice CODEC bypass pathway for audio data. This allows direct
 * output of the voltages on the TX data bus line to the output amplifiers
 * (bypassing the digital-to-analog converters within the Voice CODEC).
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC bypass was successfully
 *                                   enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC bypass could not be
 *                                   enabled.
 */
PMIC_STATUS pmic_audio_vcodec_enable_bypass(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_write = SET_BITS(regAUD_CODEC, CDCBYP, 1);
	const unsigned int reg_mask = SET_BITS(regAUD_CODEC, CDCBYP, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		rc = pmic_write_reg(REG_AUDIO_CODEC, reg_write, reg_mask);
	}

	return rc;
}

/*!
 * @brief Disable the Voice CODEC bypass audio pathway.
 *
 * Disables the Voice CODEC bypass pathway for audio data. This means that
 * the TX data bus line will deliver digital data to the digital-to-analog
 * converters within the Voice CODEC.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC bypass was successfully
 *                                   disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC bypass could not be
 *                                   disabled.
 */
PMIC_STATUS pmic_audio_vcodec_disable_bypass(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_write = 0;
	const unsigned int reg_mask = SET_BITS(regAUD_CODEC, CDCBYP, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		rc = pmic_write_reg(REG_AUDIO_CODEC, reg_write, reg_mask);
	}

	return rc;
}

/*@}*/

/*************************************************************************
 * General Stereo DAC configuration.
 *************************************************************************
 */

/*!
 * @name General Stereo DAC Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Stereo
 * DAC hardware.
 */
/*@{*/

/*!
 * @brief Set the Stereo DAC clock source and operating characteristics.
 *
 * Define the Stereo DAC clock source and operating characteristics. This
 * must be done before the Stereo DAC is enabled.
 *
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   clockIn         Select the clock signal source.
 * @param   clockFreq       Select the clock signal frequency.
 * @param   samplingRate    Select the audio data sampling rate.
 * @param   invert          Enable inversion of the frame sync and/or
 *                              bit clock inputs.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC clock settings were
 *                                   successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or clock configuration was
 *                                   invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC clock configuration
 *                                   could not be set.
 */
PMIC_STATUS pmic_audio_stdac_set_clock(const PMIC_AUDIO_HANDLE handle,
				       const PMIC_AUDIO_CLOCK_IN_SOURCE clockIn,
				       const PMIC_AUDIO_STDAC_CLOCK_IN_FREQ
				       clockFreq,
				       const PMIC_AUDIO_STDAC_SAMPLING_RATE
				       samplingRate,
				       const PMIC_AUDIO_CLOCK_INVERT invert)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_value = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;
	/* Validate all of the calling parameters. */
	if (handle == (PMIC_AUDIO_HANDLE) NULL) {
		rc = PMIC_PARAMETER_ERROR;
	} else if ((handle == stDAC.handle) &&
		   (stDAC.handleState == HANDLE_IN_USE)) {
		if ((clockIn != CLOCK_IN_CLIA) && (clockIn != CLOCK_IN_CLIB)) {
			rc = PMIC_PARAMETER_ERROR;
		} else if ((stDAC.masterSlave == BUS_MASTER_MODE)
			   && !((clockFreq >= STDAC_CLI_3_36864MHZ)
				&& (clockFreq <= STDAC_CLI_33_6MHZ))) {
			rc = PMIC_PARAMETER_ERROR;
		} else if ((stDAC.masterSlave == BUS_SLAVE_MODE)
			   && !((clockFreq >= STDAC_MCLK_PLL_DISABLED)
				&& (clockFreq <= STDAC_BCLK_IN_PLL))) {
			rc = PMIC_PARAMETER_ERROR;
		} else if (!((samplingRate >= STDAC_RATE_8_KHZ)
			     && (samplingRate <= STDAC_RATE_96_KHZ))) {
			rc = PMIC_PARAMETER_ERROR;
		}
		/*
		   else if(!((invert >= NO_INVERT) && (invert <= INVERT_FRAMESYNC)))
		   {
		   rc = PMIC_PARAMETER_ERROR;
		   } */
		else {
			reg_mask = SET_BITS(regST_DAC, STDCCLK, 7) |
			    SET_BITS(regST_DAC, STDCCLKSEL, 1) |
			    SET_BITS(regST_DAC, SR, 15) |
			    SET_BITS(regST_DAC, STDCBCLINV, 1) |
			    SET_BITS(regST_DAC, STDCFSINV, 1);
			if (clockIn == CLOCK_IN_CLIA) {
				reg_value = SET_BITS(regST_DAC, STDCCLKSEL, 0);
			} else {
				reg_value = SET_BITS(regST_DAC, STDCCLKSEL, 1);
			}
			/* How to take care of sample rates in SLAVE mode */
			if ((clockFreq == STDAC_CLI_3_36864MHZ)
			    || ((clockFreq == STDAC_FSYNC_IN_PLL))) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 6);
			} else if ((clockFreq == STDAC_CLI_12MHZ)
				   || (clockFreq == STDAC_MCLK_PLL_DISABLED)) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 5);
			} else if (clockFreq == STDAC_CLI_13MHZ) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 0);
			} else if (clockFreq == STDAC_CLI_15_36MHZ) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 1);
			} else if (clockFreq == STDAC_CLI_16_8MHZ) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 2);
			} else if (clockFreq == STDAC_CLI_26MHZ) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 4);
			} else if ((clockFreq == STDAC_CLI_33_6MHZ)
				   || (clockFreq == STDAC_BCLK_IN_PLL)) {
				reg_value |= SET_BITS(regST_DAC, STDCCLK, 7);
			}

			reg_value |= SET_BITS(regST_DAC, SR, samplingRate);

			if (invert & INVERT_BITCLOCK) {
				reg_value |= SET_BITS(regST_DAC, STDCBCLINV, 1);
			}
			if (invert & INVERT_FRAMESYNC) {
				reg_value |= SET_BITS(regST_DAC, STDCFSINV, 1);
			}
			if (invert & NO_INVERT) {
				reg_value |= SET_BITS(regST_DAC, STDCBCLINV, 0);
				reg_value |= SET_BITS(regST_DAC, STDCFSINV, 0);
			}
			if (pmic_write_reg
			    (REG_AUDIO_STEREO_DAC, reg_value,
			     reg_mask) != PMIC_SUCCESS) {
				rc = PMIC_ERROR;
			} else {
				pr_debug("STDAC clock set\n");
				rc = PMIC_SUCCESS;
				stDAC.clockIn = clockIn;
				stDAC.clockFreq = clockFreq;
				stDAC.samplingRate = samplingRate;
				stDAC.invert = invert;
			}

		}

	} else {
		rc = PMIC_PARAMETER_ERROR;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the Stereo DAC clock source and operating characteristics.
 *
 * Get the current Stereo DAC clock source and operating characteristics.
 *
 * @param  handle          Device handle from pmic_audio_open() call.
 * @param  clockIn         The clock signal source.
 * @param  clockFreq       The clock signal frequency.
 * @param  samplingRate    The audio data sampling rate.
 * @param  invert          Inversion of the frame sync and/or
 *                         bit clock inputs is enabled/disabled.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC clock settings were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC clock configuration
 *                                   could not be retrieved.
 */
PMIC_STATUS pmic_audio_stdac_get_clock(const PMIC_AUDIO_HANDLE handle,
				       PMIC_AUDIO_CLOCK_IN_SOURCE *
				       const clockIn,
				       PMIC_AUDIO_STDAC_SAMPLING_RATE *
				       const samplingRate,
				       PMIC_AUDIO_STDAC_CLOCK_IN_FREQ *
				       const clockFreq,
				       PMIC_AUDIO_CLOCK_INVERT * const invert)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) &&
	    (stDAC.handleState == HANDLE_IN_USE) &&
	    (clockIn != (PMIC_AUDIO_CLOCK_IN_SOURCE *) NULL) &&
	    (samplingRate != (PMIC_AUDIO_STDAC_SAMPLING_RATE *) NULL) &&
	    (clockFreq != (PMIC_AUDIO_STDAC_CLOCK_IN_FREQ *) NULL) &&
	    (invert != (PMIC_AUDIO_CLOCK_INVERT *) NULL)) {
		*clockIn = stDAC.clockIn;
		*samplingRate = stDAC.samplingRate;
		*clockFreq = stDAC.clockFreq;
		*invert = stDAC.invert;
		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the Stereo DAC primary audio channel timeslot.
 *
 * Set the Stereo DAC primary audio channel timeslot. This function must be
 * used if the default timeslot for the primary audio channel is to be changed.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param	timeslot        Select the primary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC primary audio channel
 *                                   timeslot was successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or audio channel timeslot
 *                                   was invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC primary audio channel
 *                                   timeslot could not be set.
 */
PMIC_STATUS pmic_audio_stdac_set_rxtx_timeslot(const PMIC_AUDIO_HANDLE handle,
					       const PMIC_AUDIO_STDAC_TIMESLOTS
					       timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = SET_BITS(regSSI_NETWORK, STDCRXSLOT, 3);

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		if ((timeslot == USE_TS0_TS1) || (timeslot == USE_TS2_TS3)
		    || (timeslot == USE_TS4_TS5) || (timeslot == USE_TS6_TS7)) {
			if (pmic_write_reg
			    (REG_AUDIO_SSI_NETWORK, timeslot,
			     reg_mask) != PMIC_SUCCESS) {
				rc = PMIC_ERROR;
			} else {
				pr_debug("STDAC primary timeslot set\n");
				stDAC.timeslot = timeslot;
				rc = PMIC_SUCCESS;
			}

		} else {
			rc = PMIC_PARAMETER_ERROR;
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current Stereo DAC primary audio channel timeslot.
 *
 * Get the current Stereo DAC primary audio channel timeslot.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   timeslot        The primary audio channel timeslot.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC primary audio channel
 *                                   timeslot was successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC primary audio channel
 *                                   timeslot could not be retrieved.
 */
PMIC_STATUS pmic_audio_stdac_get_rxtx_timeslot(const PMIC_AUDIO_HANDLE handle,
					       PMIC_AUDIO_STDAC_TIMESLOTS *
					       const timeslot)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) &&
	    (stDAC.handleState == HANDLE_IN_USE) &&
	    (timeslot != (PMIC_AUDIO_STDAC_TIMESLOTS *) NULL)) {
		*timeslot = stDAC.timeslot;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set/Enable the Stereo DAC options.
 *
 * Set or enable various Stereo DAC options. The available options include
 * resetting the digital filter and enabling the bus master clock outputs.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 * @param   config          The Stereo DAC options to enable.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC options were
 *                                   successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or Stereo DAC options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC options could not be
 *                                   successfully set/enabled.
 */
PMIC_STATUS pmic_audio_stdac_set_config(const PMIC_AUDIO_HANDLE handle,
					const PMIC_AUDIO_STDAC_CONFIG config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		if (config & STDAC_MASTER_CLOCK_OUTPUTS) {
			reg_write |= SET_BITS(regST_DAC, STDCCLKEN, 1);
			reg_mask |= SET_BITS(regST_DAC, STDCCLKEN, 1);
		}

		rc = pmic_write_reg(REG_AUDIO_STEREO_DAC, reg_write, reg_mask);

		if (rc == PMIC_SUCCESS) {
			stDAC.config |= config;
			pr_debug("STDAC config set\n");

		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Clear/Disable the Stereo DAC options.
 *
 * Clear or disable various Stereo DAC options.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       config          The Stereo DAC options to be cleared/disabled.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC options were
 *                                   successfully cleared/disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or the Stereo DAC options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC options could not be
 *                                   cleared/disabled.
 */
PMIC_STATUS pmic_audio_stdac_clear_config(const PMIC_AUDIO_HANDLE handle,
					  const PMIC_AUDIO_STDAC_CONFIG config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {

		if (config & STDAC_MASTER_CLOCK_OUTPUTS) {
			reg_mask |= SET_BITS(regST_DAC, STDCCLKEN, 1);
		}

		if (reg_mask != 0) {
			rc = pmic_write_reg(REG_AUDIO_STEREO_DAC,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				stDAC.config &= ~config;
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current Stereo DAC options.
 *
 * Get the current Stereo DAC options.
 *
 * @param   	handle          Device handle from pmic_audio_open() call.
 * @param   	config          The current set of Stereo DAC options.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC options were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC options could not be
 *                                   retrieved.
 */
PMIC_STATUS pmic_audio_stdac_get_config(const PMIC_AUDIO_HANDLE handle,
					PMIC_AUDIO_STDAC_CONFIG * const config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == stDAC.handle) &&
	    (stDAC.handleState == HANDLE_IN_USE) &&
	    (config != (PMIC_AUDIO_STDAC_CONFIG *) NULL)) {
		*config = stDAC.config;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*@}*/

/*************************************************************************
 * Audio input section configuration.
 *************************************************************************
 */

/*!
 * @name Audio Input Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC audio
 * input hardware.
 */
/*@{*/

/*!
 * @brief Set/Enable the audio input section options.
 *
 * Set or enable various audio input section options. The only available
 * option right now is to enable the automatic disabling of the microphone
 * input amplifiers when a microphone/headset is inserted or removed.
 * NOT SUPPORTED BY MC13783
 *
 * @param  	handle          Device handle from pmic_audio_open() call.
 * @param   	config          The audio input section options to enable.
 *
 * @retval      PMIC_SUCCESS         If the audio input section options were
 *                                   successfully configured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or audio input section
 *                                   options were invalid.
 * @retval      PMIC_ERROR           If the audio input section options could
 *                                   not be successfully set/enabled.
 */
PMIC_STATUS pmic_audio_input_set_config(const PMIC_AUDIO_HANDLE handle,
					const PMIC_AUDIO_INPUT_CONFIG config)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;
}

/*!
 * @brief Clear/Disable the audio input section options.
 *
 * Clear or disable various audio input section options.
 *
 * @param   	handle          Device handle from pmic_audio_open() call.
 * @param   	config          The audio input section options to be
 *                              cleared/disabled.
 * NOT SUPPORTED BY MC13783
 *
 * @retval      PMIC_SUCCESS         If the audio input section options were
 *                                   successfully cleared/disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or the audio input section
 *                                   options were invalid.
 * @retval      PMIC_ERROR           If the audio input section options could
 *                                   not be cleared/disabled.
 */
PMIC_STATUS pmic_audio_input_clear_config(const PMIC_AUDIO_HANDLE handle,
					  const PMIC_AUDIO_INPUT_CONFIG config)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;

}

/*!
 * @brief Get the current audio input section options.
 *
 * Get the current audio input section options.
 *
 * @param[in]   handle          Device handle from pmic_audio_open() call.
 * @param[out]  config          The current set of audio input section options.
 * NOT SUPPORTED BY MC13783
 *
 * @retval      PMIC_SUCCESS         If the audio input section options were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the audio input section options could
 *                                   not be retrieved.
 */
PMIC_STATUS pmic_audio_input_get_config(const PMIC_AUDIO_HANDLE handle,
					PMIC_AUDIO_INPUT_CONFIG * const config)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;
}

/*@}*/

/*************************************************************************
 * Audio recording using the Voice CODEC.
 *************************************************************************
 */

/*!
 * @name Audio Recording Using the Voice CODEC Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Voice CODEC
 * to perform audio recording.
 */
/*@{*/

/*!
 * @brief Select the microphone inputs to be used for Voice CODEC recording.
 *
 * Select left (mc13783-only) and right microphone inputs for Voice CODEC
 * recording. It is possible to disable or not use a particular microphone
 * input channel by specifying NO_MIC as a parameter.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       leftChannel     Select the left microphone input channel.
 * @param       rightChannel    Select the right microphone input channel.
 *
 * @retval      PMIC_SUCCESS         If the microphone input channels were
 *                                   successfully enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or microphone input ports
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the microphone input channels could
 *                                   not be successfully enabled.
 */
PMIC_STATUS pmic_audio_vcodec_set_mic(const PMIC_AUDIO_HANDLE handle,
				      const PMIC_AUDIO_INPUT_PORT leftChannel,
				      const PMIC_AUDIO_INPUT_PORT rightChannel)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (!((leftChannel == NO_MIC) || (leftChannel == MIC1_LEFT))) {
			rc = PMIC_PARAMETER_ERROR;
		} else if (!((rightChannel == NO_MIC)
			     || (rightChannel == MIC1_RIGHT_MIC_MONO)
			     || (rightChannel == TXIN_EXT)
			     || (rightChannel == MIC2_AUX))) {
			rc = PMIC_PARAMETER_ERROR;
		} else {
			if (leftChannel == NO_MIC) {
			} else {	/* Left channel MIC enable */
				reg_mask = SET_BITS(regAUDIO_TX, AMC1LEN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 1);
				reg_write = SET_BITS(regAUDIO_TX, AMC1LEN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 0);
			}
			/*For right channel enable one and clear the other two as well as RXINREC */
			if (rightChannel == NO_MIC) {
			} else if (rightChannel == MIC1_RIGHT_MIC_MONO) {
				reg_mask |= SET_BITS(regAUDIO_TX, AMC1REN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 1) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 1) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 1);
				reg_write |= SET_BITS(regAUDIO_TX, AMC1REN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 0) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 0) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 0);
			} else if (rightChannel == MIC2_AUX) {
				reg_mask |= SET_BITS(regAUDIO_TX, AMC1REN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 1) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 1) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 1);
				reg_write |= SET_BITS(regAUDIO_TX, AMC1REN, 0) |
				    SET_BITS(regAUDIO_TX, RXINREC, 0) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 1) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 0);
			} else {	/* TX line in */
				reg_mask |= SET_BITS(regAUDIO_TX, AMC1REN, 1) |
				    SET_BITS(regAUDIO_TX, RXINREC, 1) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 1) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 1);
				reg_write |= SET_BITS(regAUDIO_TX, AMC1REN, 0) |
				    SET_BITS(regAUDIO_TX, RXINREC, 0) |
				    SET_BITS(regAUDIO_TX, AMC2EN, 0) |
				    SET_BITS(regAUDIO_TX, ATXINEN, 1);
			}

			if (reg_mask == 0) {
				rc = PMIC_PARAMETER_ERROR;
			} else {
				rc = pmic_write_reg(REG_AUDIO_TX,
						    reg_write, reg_mask);

				if (rc == PMIC_SUCCESS) {
					pr_debug
					    ("MIC inputs configured successfully\n");
					vCodec.leftChannelMic.mic = leftChannel;
					vCodec.rightChannelMic.mic =
					    rightChannel;

				}
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current microphone inputs being used for Voice CODEC
 *        recording.
 *
 * Get the left (mc13783-only) and right microphone inputs currently being
 * used for Voice CODEC recording.
 *
 * @param   	handle          Device handle from pmic_audio_open() call.
 * @param 	leftChannel     The left microphone input channel.
 * @param	rightChannel    The right microphone input channel.
 *
 * @retval      PMIC_SUCCESS         If the microphone input channels were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the microphone input channels could
 *                                   not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_mic(const PMIC_AUDIO_HANDLE handle,
				      PMIC_AUDIO_INPUT_PORT * const leftChannel,
				      PMIC_AUDIO_INPUT_PORT *
				      const rightChannel)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (leftChannel != (PMIC_AUDIO_INPUT_PORT *) NULL) &&
	    (rightChannel != (PMIC_AUDIO_INPUT_PORT *) NULL)) {
		*leftChannel = vCodec.leftChannelMic.mic;
		*rightChannel = vCodec.rightChannelMic.mic;
		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);
	return rc;
}

/*!
 * @brief Enable/disable the microphone input.
 *
 * This function enables/disables the current microphone input channel. The
 * input amplifier is automatically turned off when the microphone input is
 * disabled.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param       leftChannel     The left microphone input channel state.
 * @param       rightChannel    the right microphone input channel state.
 *
 * @retval      PMIC_SUCCESS         If the microphone input channels were
 *                                   successfully reconfigured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or microphone input states
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the microphone input channels could
 *                                   not be reconfigured.
 */
PMIC_STATUS pmic_audio_vcodec_set_mic_on_off(const PMIC_AUDIO_HANDLE handle,
					     const PMIC_AUDIO_INPUT_MIC_STATE
					     leftChannel,
					     const PMIC_AUDIO_INPUT_MIC_STATE
					     rightChannel)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;
	unsigned int curr_left = 0;
	unsigned int curr_right = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		curr_left = vCodec.leftChannelMic.mic;
		curr_right = vCodec.rightChannelMic.mic;
		if ((curr_left == NO_MIC) && (curr_right == NO_MIC)) {
			rc = PMIC_PARAMETER_ERROR;
		} else {
			if (curr_left == MIC1_LEFT) {
				if ((leftChannel == MICROPHONE_ON) &&
				    (vCodec.leftChannelMic.micOnOff ==
				     MICROPHONE_OFF)) {
					/* Enable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1LEN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1LEN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC, 0);

				} else if ((leftChannel == MICROPHONE_OFF) &&
					   (vCodec.leftChannelMic.micOnOff ==
					    MICROPHONE_ON)) {
					/* Disable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1LEN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1LEN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC, 0);

				} else {
					/* Both are in same state . Nothing to be done */
				}

			}
			if (curr_right == MIC1_RIGHT_MIC_MONO) {
				if ((rightChannel == MICROPHONE_ON) &&
				    (vCodec.leftChannelMic.micOnOff ==
				     MICROPHONE_OFF)) {
					/* Enable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     0) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 0);
				} else if ((rightChannel == MICROPHONE_OFF)
					   && (vCodec.leftChannelMic.micOnOff ==
					       MICROPHONE_ON)) {
					/* Disable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     0) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 0);
				} else {
					/* Both are in same state . Nothing to be done */
				}
			} else if (curr_right == MIC2_AUX) {
				if ((rightChannel == MICROPHONE_ON)
				    && (vCodec.leftChannelMic.micOnOff ==
					MICROPHONE_OFF)) {
					/* Enable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 0);
				} else if ((rightChannel == MICROPHONE_OFF)
					   && (vCodec.leftChannelMic.micOnOff ==
					       MICROPHONE_ON)) {
					/* Disable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     0) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 0);
				} else {
					/* Both are in same state . Nothing to be done */
				}
			} else if (curr_right == TXIN_EXT) {
				if ((rightChannel == MICROPHONE_ON)
				    && (vCodec.leftChannelMic.micOnOff ==
					MICROPHONE_OFF)) {
					/* Enable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     0) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
				} else if ((rightChannel == MICROPHONE_OFF)
					   && (vCodec.leftChannelMic.micOnOff ==
					       MICROPHONE_ON)) {
					/* Disable the microphone */
					reg_mask |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     1) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   1) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     1) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 1);
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1REN,
						     0) | SET_BITS(regAUDIO_TX,
								   RXINREC,
								   0) |
					    SET_BITS(regAUDIO_TX, AMC2EN,
						     0) | SET_BITS(regAUDIO_TX,
								   ATXINEN, 0);
				} else {
					/* Both are in same state . Nothing to be done */
				}
			}
			if (reg_mask == 0) {
			} else {
				rc = pmic_write_reg(REG_AUDIO_TX,
						    reg_write, reg_mask);

				if (rc == PMIC_SUCCESS) {
					pr_debug
					    ("MIC states configured successfully\n");
					vCodec.leftChannelMic.micOnOff =
					    leftChannel;
					vCodec.rightChannelMic.micOnOff =
					    rightChannel;
				}
			}
		}

	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Return the current state of the microphone inputs.
 *
 * This function returns the current state (on/off) of the microphone
 * input channels.
 *
 * @param       handle          Device handle from pmic_audio_open() call.
 * @param	leftChannel     The current left microphone input channel
 *                              state.
 * @param	rightChannel    the current right microphone input channel
 *                              state.
 *
 * @retval      PMIC_SUCCESS         If the microphone input channel states
 *                                   were successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the microphone input channel states
 *                                   could not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_mic_on_off(const PMIC_AUDIO_HANDLE handle,
					     PMIC_AUDIO_INPUT_MIC_STATE *
					     const leftChannel,
					     PMIC_AUDIO_INPUT_MIC_STATE *
					     const rightChannel)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (leftChannel != (PMIC_AUDIO_INPUT_MIC_STATE *) NULL) &&
	    (rightChannel != (PMIC_AUDIO_INPUT_MIC_STATE *) NULL)) {
		*leftChannel = vCodec.leftChannelMic.micOnOff;
		*rightChannel = vCodec.rightChannelMic.micOnOff;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the microphone input amplifier mode and gain level.
 *
 * This function sets the current microphone input amplifier operating mode
 * and gain level.
 *
 * @param       handle           Device handle from pmic_audio_open() call.
 * @param       leftChannelMode  The left microphone input amplifier mode.
 * @param       leftChannelGain  The left microphone input amplifier gain level.
 * @param       rightChannelMode The right microphone input amplifier mode.
 * @param       rightChannelGain The right microphone input amplifier gain
 *                               level.
 *
 * @retval      PMIC_SUCCESS         If the microphone input amplifiers were
 *                                   successfully reconfigured.
 * @retval      PMIC_PARAMETER_ERROR If the handle or microphone input amplifier
 *                                   modes or gain levels were invalid.
 * @retval      PMIC_ERROR           If the microphone input amplifiers could
 *                                   not be reconfigured.
 */
PMIC_STATUS pmic_audio_vcodec_set_record_gain(const PMIC_AUDIO_HANDLE handle,
					      const PMIC_AUDIO_MIC_AMP_MODE
					      leftChannelMode,
					      const PMIC_AUDIO_MIC_GAIN
					      leftChannelGain,
					      const PMIC_AUDIO_MIC_AMP_MODE
					      rightChannelMode,
					      const PMIC_AUDIO_MIC_GAIN
					      rightChannelGain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (!(((leftChannelGain >= MIC_GAIN_MINUS_8DB)
		       && (leftChannelGain <= MIC_GAIN_PLUS_23DB))
		      && ((rightChannelGain >= MIC_GAIN_MINUS_8DB)
			  && (rightChannelGain <= MIC_GAIN_PLUS_23DB)))) {
			rc = PMIC_PARAMETER_ERROR;
			pr_debug("VCODEC set record gain - wrong gain value\n");
		} else if (((leftChannelMode != AMP_OFF)
			    && (leftChannelMode != VOLTAGE_TO_VOLTAGE)
			    && (leftChannelMode != CURRENT_TO_VOLTAGE))
			   || ((rightChannelMode != VOLTAGE_TO_VOLTAGE)
			       && (rightChannelMode != CURRENT_TO_VOLTAGE)
			       && (rightChannelMode != AMP_OFF))) {
			rc = PMIC_PARAMETER_ERROR;
			pr_debug("VCODEC set record gain - wrong amp mode\n");
		} else {
			if (vCodec.leftChannelMic.mic == MIC1_LEFT) {
				reg_mask = SET_BITS(regAUDIO_TX, AMC1LITOV, 1) |
				    SET_BITS(regAUDIO_TX, PGATXL, 31);
				if (leftChannelMode == VOLTAGE_TO_VOLTAGE) {
					reg_write =
					    SET_BITS(regAUDIO_TX, AMC1LITOV, 0);
				} else {
					reg_write =
					    SET_BITS(regAUDIO_TX, AMC1LITOV, 1);
				}
				reg_write |=
				    SET_BITS(regAUDIO_TX, PGATXL,
					     leftChannelGain);
			}
			if (vCodec.rightChannelMic.mic == MIC1_RIGHT_MIC_MONO) {
				reg_mask |=
				    SET_BITS(regAUDIO_TX, AMC1RITOV,
					     1) | SET_BITS(regAUDIO_TX, PGATXR,
							   31);
				if (rightChannelMode == VOLTAGE_TO_VOLTAGE) {
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1RITOV, 0);
				} else {
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC1RITOV, 1);
				}
				reg_write |=
				    SET_BITS(regAUDIO_TX, PGATXR,
					     rightChannelGain);
			} else if (vCodec.rightChannelMic.mic == MIC2_AUX) {
				reg_mask |= SET_BITS(regAUDIO_TX, AMC2ITOV, 1);
				reg_mask |= SET_BITS(regAUDIO_TX, PGATXR, 31);
				if (rightChannelMode == VOLTAGE_TO_VOLTAGE) {
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC2ITOV, 0);
				} else {
					reg_write |=
					    SET_BITS(regAUDIO_TX, AMC2ITOV, 1);
				}
				reg_write |=
				    SET_BITS(regAUDIO_TX, PGATXR,
					     rightChannelGain);
			} else if (vCodec.rightChannelMic.mic == TXIN_EXT) {
				reg_mask |= SET_BITS(regAUDIO_TX, PGATXR, 31);
				/* No current to voltage option for TX IN amplifier */
				reg_write |=
				    SET_BITS(regAUDIO_TX, PGATXR,
					     rightChannelGain);
			}

			if (reg_mask == 0) {
			} else {
				rc = pmic_write_reg(REG_AUDIO_TX,
						    reg_write, reg_mask);
				reg_write =
				    SET_BITS(regAUDIO_TX, PGATXL,
					     leftChannelGain);
				reg_mask = SET_BITS(regAUDIO_TX, PGATXL, 31);
				rc = pmic_write_reg(REG_AUDIO_TX,
						    reg_write, reg_mask);

				if (rc == PMIC_SUCCESS) {
					pr_debug("MIC amp mode and gain set\n");
					vCodec.leftChannelMic.ampMode =
					    leftChannelMode;
					vCodec.leftChannelMic.gain =
					    leftChannelGain;
					vCodec.rightChannelMic.ampMode =
					    rightChannelMode;
					vCodec.rightChannelMic.gain =
					    rightChannelGain;

				}
			}
		}
	}

	/* Exit critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current microphone input amplifier mode and gain level.
 *
 * This function gets the current microphone input amplifier operating mode
 * and gain level.
 *
 * @param	handle           Device handle from pmic_audio_open() call.
 * @param	leftChannelMode  The left microphone input amplifier mode.
 * @param	leftChannelGain  The left microphone input amplifier gain level.
 * @param 	rightChannelMode The right microphone input amplifier mode.
 * @param       rightChannelGain The right microphone input amplifier gain
 *                               level.
 *
 * @retval      PMIC_SUCCESS         If the microphone input amplifier modes
 *                                   and gain levels were successfully
 *                                   retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the microphone input amplifier modes
 *                                   and gain levels could not be retrieved.
 */
PMIC_STATUS pmic_audio_vcodec_get_record_gain(const PMIC_AUDIO_HANDLE handle,
					      PMIC_AUDIO_MIC_AMP_MODE *
					      const leftChannelMode,
					      PMIC_AUDIO_MIC_GAIN *
					      const leftChannelGain,
					      PMIC_AUDIO_MIC_AMP_MODE *
					      const rightChannelMode,
					      PMIC_AUDIO_MIC_GAIN *
					      const rightChannelGain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == vCodec.handle) &&
	    (vCodec.handleState == HANDLE_IN_USE) &&
	    (leftChannelMode != (PMIC_AUDIO_MIC_AMP_MODE *) NULL) &&
	    (leftChannelGain != (PMIC_AUDIO_MIC_GAIN *) NULL) &&
	    (rightChannelMode != (PMIC_AUDIO_MIC_AMP_MODE *) NULL) &&
	    (rightChannelGain != (PMIC_AUDIO_MIC_GAIN *) NULL)) {
		*leftChannelMode = vCodec.leftChannelMic.ampMode;
		*leftChannelGain = vCodec.leftChannelMic.gain;
		*rightChannelMode = vCodec.rightChannelMic.ampMode;
		*rightChannelGain = vCodec.rightChannelMic.gain;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Enable a microphone bias circuit.
 *
 * This function enables one of the available microphone bias circuits.
 *
 * @param       handle           Device handle from pmic_audio_open() call.
 * @param       biasCircuit      The microphone bias circuit to be enabled.
 *
 * @retval      PMIC_SUCCESS         If the microphone bias circuit was
 *                                   successfully enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or selected microphone bias
 *                                   circuit was invalid.
 * @retval      PMIC_ERROR           If the microphone bias circuit could not
 *                                   be enabled.
 */
PMIC_STATUS pmic_audio_vcodec_enable_micbias(const PMIC_AUDIO_HANDLE handle,
					     const PMIC_AUDIO_MIC_BIAS
					     biasCircuit)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (biasCircuit & MIC_BIAS1) {
			reg_write = SET_BITS(regAUDIO_TX, MC1BEN, 1);
			reg_mask = SET_BITS(regAUDIO_TX, MC1BEN, 1);
		}
		if (biasCircuit & MIC_BIAS2) {
			reg_write |= SET_BITS(regAUDIO_TX, MC2BEN, 1);
			reg_mask |= SET_BITS(regAUDIO_TX, MC2BEN, 1);
		}
		if (reg_mask != 0) {
			rc = pmic_write_reg(REG_AUDIO_TX, reg_write, reg_mask);
		}
	}

	return rc;
}

/*!
 * @brief Disable a microphone bias circuit.
 *
 * This function disables one of the available microphone bias circuits.
 *
 * @param      handle           Device handle from pmic_audio_open() call.
 * @param      biasCircuit      The microphone bias circuit to be disabled.
 *
 * @retval      PMIC_SUCCESS         If the microphone bias circuit was
 *                                   successfully disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or selected microphone bias
 *                                   circuit was invalid.
 * @retval      PMIC_ERROR           If the microphone bias circuit could not
 *                                   be disabled.
 */
PMIC_STATUS pmic_audio_vcodec_disable_micbias(const PMIC_AUDIO_HANDLE handle,
					      const PMIC_AUDIO_MIC_BIAS
					      biasCircuit)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (biasCircuit & MIC_BIAS1) {
			reg_mask = SET_BITS(regAUDIO_TX, MC1BEN, 1);
		}

		if (biasCircuit & MIC_BIAS2) {
			reg_mask |= SET_BITS(regAUDIO_TX, MC2BEN, 1);
		}

		if (reg_mask != 0) {
			rc = pmic_write_reg(REG_AUDIO_TX, reg_write, reg_mask);
		}
	}

	return rc;
}

/*@}*/

/*************************************************************************
 * Audio Playback Using the Voice CODEC.
 *************************************************************************
 */

/*!
 * @name Audio Playback Using the Voice CODEC Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Voice CODEC
 * to perform audio playback.
 */
/*@{*/

/*!
 * @brief Configure and enable the Voice CODEC mixer.
 *
 * This function configures and enables the Voice CODEC mixer.
 *
 * @param       handle              Device handle from pmic_audio_open() call.
 * @param       rxSecondaryTimeslot The timeslot used for the secondary audio
 *                                  channel.
 * @param       gainIn              The secondary audio channel gain level.
 * @param       gainOut             The mixer output gain level.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC mixer was successfully
 *                                   configured and enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or mixer configuration
 *                                   was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC mixer could not be
 *                                   reconfigured or enabled.
 */
PMIC_STATUS pmic_audio_vcodec_enable_mixer(const PMIC_AUDIO_HANDLE handle,
					   const PMIC_AUDIO_VCODEC_TIMESLOT
					   rxSecondaryTimeslot,
					   const PMIC_AUDIO_VCODEC_MIX_IN_GAIN
					   gainIn,
					   const PMIC_AUDIO_VCODEC_MIX_OUT_GAIN
					   gainOut)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		if (!((rxSecondaryTimeslot >= USE_TS0)
		      && (rxSecondaryTimeslot <= USE_TS3))) {
			pr_debug
			    ("VCODEC enable mixer - wrong sec rx timeslot\n");
		} else if (!((gainIn >= VCODEC_NO_MIX)
			     && (gainIn <= VCODEC_MIX_IN_MINUS_12DB))) {
			pr_debug("VCODEC enable mixer - wrong mix in gain\n");

		} else if (!((gainOut >= VCODEC_MIX_OUT_0DB)
			     && (gainOut <= VCODEC_MIX_OUT_MINUS_6DB))) {
			pr_debug("VCODEC enable mixer - wrong mix out gain\n");
		} else {

			reg_mask = SET_BITS(regSSI_NETWORK, CDCRXSECSLOT, 3) |
			    SET_BITS(regSSI_NETWORK, CDCRXSECGAIN, 3) |
			    SET_BITS(regSSI_NETWORK, CDCSUMGAIN, 1);
			reg_write =
			    SET_BITS(regSSI_NETWORK, CDCRXSECSLOT,
				     rxSecondaryTimeslot) |
			    SET_BITS(regSSI_NETWORK, CDCRXSECGAIN,
				     gainIn) | SET_BITS(regSSI_NETWORK,
							CDCSUMGAIN, gainOut);
			rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
					    reg_write, reg_mask);
			if (rc == PMIC_SUCCESS) {
				pr_debug("Vcodec mixer enabled\n");
			}
		}
	}

	return rc;
}

/*!
 * @brief Disable the Voice CODEC mixer.
 *
 * This function disables the Voice CODEC mixer.
 *
 * @param       handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the Voice CODEC mixer was successfully
 *                                   disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Voice CODEC mixer could not be
 *                                   disabled.
 */
PMIC_STATUS pmic_audio_vcodec_disable_mixer(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask;

	if ((handle == vCodec.handle) && (vCodec.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regSSI_NETWORK, CDCRXSECGAIN, 1);
		rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
				    VCODEC_NO_MIX, reg_mask);

		if (rc == PMIC_SUCCESS) {
			pr_debug("Vcodec mixer disabled\n");
		}

	}

	return rc;
}

/*@}*/

/*************************************************************************
 * Audio Playback Using the Stereo DAC.
 *************************************************************************
 */

/*!
 * @name Audio Playback Using the Stereo DAC Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC Stereo DAC
 * to perform audio playback.
 */
/*@{*/

/*!
 * @brief Configure and enable the Stereo DAC mixer.
 *
 * This function configures and enables the Stereo DAC mixer.
 *
 * @param      handle              Device handle from pmic_audio_open() call.
 * @param      rxSecondaryTimeslot The timeslot used for the secondary audio
 *                                  channel.
 * @param      gainIn              The secondary audio channel gain level.
 * @param      gainOut             The mixer output gain level.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC mixer was successfully
 *                                   configured and enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or mixer configuration
 *                                   was invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC mixer could not be
 *                                   reconfigured or enabled.
 */
PMIC_STATUS pmic_audio_stdac_enable_mixer(const PMIC_AUDIO_HANDLE handle,
					  const PMIC_AUDIO_STDAC_TIMESLOTS
					  rxSecondaryTimeslot,
					  const PMIC_AUDIO_STDAC_MIX_IN_GAIN
					  gainIn,
					  const PMIC_AUDIO_STDAC_MIX_OUT_GAIN
					  gainOut)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		if (!((rxSecondaryTimeslot >= USE_TS0_TS1)
		      && (rxSecondaryTimeslot <= USE_TS6_TS7))) {
			rc = PMIC_PARAMETER_ERROR;
			pr_debug("STDAC enable mixer - wrong sec timeslot\n");
		} else if (!((gainIn >= STDAC_NO_MIX)
			     && (gainIn <= STDAC_MIX_IN_MINUS_12DB))) {
			rc = PMIC_PARAMETER_ERROR;
			pr_debug("STDAC enable mixer - wrong mix in gain\n");
		} else if (!((gainOut >= STDAC_MIX_OUT_0DB)
			     && (gainOut <= STDAC_MIX_OUT_MINUS_6DB))) {
			rc = PMIC_PARAMETER_ERROR;
			pr_debug("STDAC enable mixer - wrong mix out gain\n");
		} else {

			reg_mask = SET_BITS(regSSI_NETWORK, STDCRXSECSLOT, 3) |
			    SET_BITS(regSSI_NETWORK, STDCRXSECGAIN, 3) |
			    SET_BITS(regSSI_NETWORK, STDCSUMGAIN, 1);
			reg_write =
			    SET_BITS(regSSI_NETWORK, STDCRXSECSLOT,
				     rxSecondaryTimeslot) |
			    SET_BITS(regSSI_NETWORK, STDCRXSECGAIN,
				     gainIn) | SET_BITS(regSSI_NETWORK,
							STDCSUMGAIN, gainOut);
			rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
					    reg_write, reg_mask);
			if (rc == PMIC_SUCCESS) {
				pr_debug("STDAC mixer enabled\n");
			}
		}

	}

	return rc;
}

/*!
 * @brief Disable the Stereo DAC mixer.
 *
 * This function disables the Stereo DAC mixer.
 *
 * @param       handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the Stereo DAC mixer was successfully
 *                                   disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the Stereo DAC mixer could not be
 *                                   disabled.
 */
PMIC_STATUS pmic_audio_stdac_disable_mixer(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_write = 0;
	const unsigned int reg_mask =
	    SET_BITS(regSSI_NETWORK, STDCRXSECGAIN, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK, reg_write, reg_mask);
	}

	return rc;
}

/*@}*/

/*************************************************************************
 * Audio Output Control
 *************************************************************************
 */

/*!
 * @name Audio Output Section Setup and Configuration APIs
 * Functions for general setup and configuration of the PMIC audio output
 * section to support playback.
 */
/*@{*/

/*!
 * @brief Select the audio output ports.
 *
 * This function selects the audio output ports to be used. This also enables
 * the appropriate output amplifiers.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   port                The audio output ports to be used.
 *
 * @retval      PMIC_SUCCESS         If the audio output ports were successfully
 *                                   acquired.
 * @retval      PMIC_PARAMETER_ERROR If the handle or output ports were
 *                                   invalid.
 * @retval      PMIC_ERROR           If the audio output ports could not be
 *                                   acquired.
 */
PMIC_STATUS pmic_audio_output_set_port(const PMIC_AUDIO_HANDLE handle,
				       const PMIC_AUDIO_OUTPUT_PORT port)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((port == MONO_ALERT) || (port == MONO_EXTOUT)) {
		rc = PMIC_NOT_SUPPORTED;
	} else {
		if (((handle == stDAC.handle)
		     && (stDAC.handleState == HANDLE_IN_USE))
		    || ((handle == extStereoIn.handle)
			&& (extStereoIn.handleState == HANDLE_IN_USE))
		    || ((handle == vCodec.handle)
			&& (vCodec.handleState == HANDLE_IN_USE)
			&& (audioOutput.vCodecOut == VCODEC_MIXER_OUT))) {
			/* Stereo signal and MIXER source needs to be routed to the port
			   / Avoid Codec direct out */

			if (port & MONO_SPEAKER) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ASPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ASPSEL, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ASPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ASPSEL, 1);
			}
			if (port & MONO_LOUDSPEAKER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, ALSPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPREF, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ALSPEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ALSPREF,
							   1) |
				    SET_BITS(regAUDIO_RX_0, ALSPSEL, 1);
			}
			if (port & STEREO_HEADSET_LEFT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1) |
				    SET_BITS(regAUDIO_RX_0, AHSSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, AHSLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   AHSSEL, 1);
			}
			if (port & STEREO_HEADSET_RIGHT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1) |
				    SET_BITS(regAUDIO_RX_0, AHSSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, AHSREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   AHSSEL, 1);
			}
			if (port & STEREO_OUT_LEFT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
			}
			if (port & STEREO_OUT_RIGHT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
			}
			if (port & STEREO_LEFT_LOW_POWER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, LSPLEN, 1);

				reg_write |= SET_BITS(regAUDIO_RX_0, LSPLEN, 1);
			}
		} else if ((handle == vCodec.handle)
			   && (vCodec.handleState == HANDLE_IN_USE)
			   && (audioOutput.vCodecOut == VCODEC_DIRECT_OUT)) {
			if (port & MONO_SPEAKER) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ASPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ASPSEL, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ASPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ASPSEL, 0);
			}
			if (port & MONO_LOUDSPEAKER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, ALSPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPREF, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ALSPEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ALSPREF,
							   1) |
				    SET_BITS(regAUDIO_RX_0, ALSPSEL, 0);
			}

			if (port & STEREO_HEADSET_LEFT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1) |
				    SET_BITS(regAUDIO_RX_0, AHSSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, AHSLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   AHSSEL, 0);
			}
			if (port & STEREO_HEADSET_RIGHT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1) |
				    SET_BITS(regAUDIO_RX_0, AHSSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, AHSREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   AHSSEL, 0);
			}
			if (port & STEREO_OUT_LEFT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 0);
			}
			if (port & STEREO_OUT_RIGHT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN,
					     1) | SET_BITS(regAUDIO_RX_0,
							   ARXOUTSEL, 0);
			}
			if (port & MONO_CDCOUT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, CDCOUTEN, 1);

				reg_write |=
				    SET_BITS(regAUDIO_RX_0, CDCOUTEN, 1);
			}
		}

		if (reg_mask == 0) {

		} else {
			rc = pmic_write_reg(REG_AUDIO_RX_0,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				pr_debug("output ports  enabled\n");
				audioOutput.outputPort = port;

			}
		}
	}
	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Deselect/disable the audio output ports.
 *
 * This function disables the audio output ports that were previously enabled
 * by calling pmic_audio_output_set_port().
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   port                The audio output ports to be disabled.
 *
 * @retval      PMIC_SUCCESS         If the audio output ports were successfully
 *                                   disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or output ports were
 *                                   invalid.
 * @retval      PMIC_ERROR           If the audio output ports could not be
 *                                   disabled.
 */
PMIC_STATUS pmic_audio_output_clear_port(const PMIC_AUDIO_HANDLE handle,
					 const PMIC_AUDIO_OUTPUT_PORT port)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((port == MONO_ALERT) || (port == MONO_EXTOUT)) {
		rc = PMIC_NOT_SUPPORTED;
	} else {
		if (((handle == stDAC.handle)
		     && (stDAC.handleState == HANDLE_IN_USE))
		    || ((handle == extStereoIn.handle)
			&& (extStereoIn.handleState == HANDLE_IN_USE))
		    || ((handle == vCodec.handle)
			&& (vCodec.handleState == HANDLE_IN_USE)
			&& (audioOutput.vCodecOut == VCODEC_MIXER_OUT))) {
			/* Stereo signal and MIXER source needs to be routed to the port /
			   Avoid Codec direct out */
			if (port & MONO_SPEAKER) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ASPEN, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ASPEN, 0);
			}
			if (port & MONO_LOUDSPEAKER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, ALSPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPREF, 1);

				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ALSPEN,
					     0) | SET_BITS(regAUDIO_RX_0,
							   ALSPREF, 0);

			}
			if (port & STEREO_HEADSET_LEFT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1);
				reg_write |= SET_BITS(regAUDIO_RX_0, AHSLEN, 0);
			}
			if (port & STEREO_HEADSET_RIGHT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1);
				reg_write |= SET_BITS(regAUDIO_RX_0, AHSREN, 0);
			}
			if (port & STEREO_OUT_LEFT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN, 0);
			}
			if (port & STEREO_OUT_RIGHT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN, 0);
			}
			if (port & STEREO_LEFT_LOW_POWER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, LSPLEN, 1);
				reg_write |= SET_BITS(regAUDIO_RX_0, LSPLEN, 0);
			}
		} else if ((handle == vCodec.handle)
			   && (vCodec.handleState == HANDLE_IN_USE)
			   && (audioOutput.vCodecOut == VCODEC_DIRECT_OUT)) {
			if (port & MONO_SPEAKER) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ASPEN, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ASPEN, 0);
			}
			if (port & MONO_LOUDSPEAKER) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, ALSPEN, 1) |
				    SET_BITS(regAUDIO_RX_0, ALSPREF, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ALSPEN,
					     0) | SET_BITS(regAUDIO_RX_0,
							   ALSPREF, 0);
			}
			if (port & STEREO_HEADSET_LEFT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1);
				reg_write |= SET_BITS(regAUDIO_RX_0, AHSLEN, 0);
			}
			if (port & STEREO_HEADSET_RIGHT) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1);
				reg_write |= SET_BITS(regAUDIO_RX_0, AHSREN, 0);
			}
			if (port & STEREO_OUT_LEFT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTLEN, 0);
			}
			if (port & STEREO_OUT_RIGHT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, ARXOUTREN, 0);
			}
			if (port & MONO_CDCOUT) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, CDCOUTEN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, CDCOUTEN, 0);
			}
		}
#ifdef CONFIG_HEADSET_DETECT_ENABLE

		if (port & STEREO_HEADSET_LEFT) {
			reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1);
			reg_write |= SET_BITS(regAUDIO_RX_0, AHSLEN, 0);
		}
		if (port & STEREO_HEADSET_RIGHT) {
			reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1);
			reg_write |= SET_BITS(regAUDIO_RX_0, AHSREN, 0);
		}
#endif

		if (reg_mask == 0) {

		} else {
			rc = pmic_write_reg(REG_AUDIO_RX_0,
					    reg_write, reg_mask);
			if (rc == PMIC_SUCCESS) {
				pr_debug("output ports disabled\n");
				audioOutput.outputPort &= ~port;
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current audio output ports.
 *
 * This function retrieves the audio output ports that are currently being
 * used.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   port                The audio output ports currently being used.
 *
 * @retval      PMIC_SUCCESS         If the audio output ports were successfully
 *                                   retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the audio output ports could not be
 *                                   retrieved.
 */
PMIC_STATUS pmic_audio_output_get_port(const PMIC_AUDIO_HANDLE handle,
				       PMIC_AUDIO_OUTPUT_PORT * const port)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((((handle == stDAC.handle) &&
	      (stDAC.handleState == HANDLE_IN_USE)) ||
	     ((handle == vCodec.handle) &&
	      (vCodec.handleState == HANDLE_IN_USE)) ||
	     ((handle == extStereoIn.handle) &&
	      (extStereoIn.handleState == HANDLE_IN_USE))) &&
	    (port != (PMIC_AUDIO_OUTPUT_PORT *) NULL)) {
		*port = audioOutput.outputPort;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the gain level for the external stereo inputs.
 *
 * This function sets the gain levels for the external stereo inputs.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The external stereo input gain level.
 *
 * @retval      PMIC_SUCCESS         If the gain level was successfully set.
 * @retval      PMIC_PARAMETER_ERROR If the handle or gain level was invalid.
 * @retval      PMIC_ERROR           If the gain level could not be set.
 */
PMIC_STATUS pmic_audio_output_set_stereo_in_gain(const PMIC_AUDIO_HANDLE handle,
						 const PMIC_AUDIO_STEREO_IN_GAIN
						 gain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1) |
	    SET_BITS(regAUDIO_RX_1, ARXIN, 1);
	unsigned int reg_write = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	/* The ARX amplifier for stereo is also enabled over here */

	if ((gain == STEREO_IN_GAIN_0DB) || (gain == STEREO_IN_GAIN_PLUS_18DB)) {
		if ((handle == extStereoIn.handle) &&
		    (extStereoIn.handleState == HANDLE_IN_USE)) {

			if (gain == STEREO_IN_GAIN_0DB) {
				reg_write |= SET_BITS(regAUDIO_RX_1, ARXIN, 1);
			} else {
				reg_write |= SET_BITS(regAUDIO_RX_1, ARXIN, 0);
			}

			rc = pmic_write_reg(REG_AUDIO_RX_1,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				pr_debug("Ext stereo gain set\n");
				extStereoIn.inputGain = gain;

			}

		} else {
			rc = PMIC_PARAMETER_ERROR;
		}
	}

	return rc;
}

/*!
 * @brief Get the current gain level for the external stereo inputs.
 *
 * This function retrieves the current gain levels for the external stereo
 * inputs.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The current external stereo input gain
 *                                  level.
 *
 * @retval      PMIC_SUCCESS         If the gain level was successfully
 *                                   retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the gain level could not be retrieved.
 */
PMIC_STATUS pmic_audio_output_get_stereo_in_gain(const PMIC_AUDIO_HANDLE handle,
						 PMIC_AUDIO_STEREO_IN_GAIN *
						 const gain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((handle == extStereoIn.handle) &&
	    (extStereoIn.handleState == HANDLE_IN_USE) &&
	    (gain != (PMIC_AUDIO_STEREO_IN_GAIN *) NULL)) {
		*gain = extStereoIn.inputGain;
		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Set the output PGA gain level.
 *
 * This function sets the audio output PGA gain level.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The output PGA gain level.
 *
 * @retval      PMIC_SUCCESS         If the gain level was successfully set.
 * @retval      PMIC_PARAMETER_ERROR If the handle or gain level was invalid.
 * @retval      PMIC_ERROR           If the gain level could not be set.
 */
PMIC_STATUS pmic_audio_output_set_pgaGain(const PMIC_AUDIO_HANDLE handle,
					  const PMIC_AUDIO_OUTPUT_PGA_GAIN gain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = 0;
	unsigned int reg_gain;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (!((gain >= OUTPGA_GAIN_MINUS_33DB)
	      && (gain <= OUTPGA_GAIN_PLUS_6DB))) {
		rc = PMIC_NOT_SUPPORTED;
		pr_debug("output set PGA gain - wrong gain value\n");
	} else {
		reg_gain = gain + 2;
		if ((handle == extStereoIn.handle) &&
		    (extStereoIn.handleState == HANDLE_IN_USE)) {
			reg_mask = SET_BITS(regAUDIO_RX_1, ARXIN, 15) |
			    SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
			reg_write = SET_BITS(regAUDIO_RX_1, ARXIN, reg_gain) |
			    SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		} else if ((handle == vCodec.handle) &&
			   (vCodec.handleState == HANDLE_IN_USE)) {
			reg_mask = SET_BITS(regAUDIO_RX_1, PGARX, 15);
			reg_write = SET_BITS(regAUDIO_RX_1, PGARX, reg_gain);
		} else if ((handle == stDAC.handle) &&
			   (stDAC.handleState == HANDLE_IN_USE)) {
			reg_mask = SET_BITS(regAUDIO_RX_1, PGAST, 15);
			reg_write = SET_BITS(regAUDIO_RX_1, PGAST, reg_gain);
		}

		if (reg_mask == 0) {

		} else {
			rc = pmic_write_reg(REG_AUDIO_RX_1,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				pr_debug("Output PGA gains set\n");

				if (handle == stDAC.handle) {
					audioOutput.stDacoutputPGAGain = gain;
				} else if (handle == vCodec.handle) {
					audioOutput.vCodecoutputPGAGain = gain;
				} else {
					audioOutput.extStereooutputPGAGain =
					    gain;
				}
			} else {
				pr_debug
				    ("Error writing PGA gains to register\n");
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the output PGA gain level.
 *
 * This function retrieves the current audio output PGA gain level.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The current output PGA gain level.
 *
 * @retval      PMIC_SUCCESS         If the gain level was successfully
 *                                   retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the gain level could not be retrieved.
 */
PMIC_STATUS pmic_audio_output_get_pgaGain(const PMIC_AUDIO_HANDLE handle,
					  PMIC_AUDIO_OUTPUT_PGA_GAIN *
					  const gain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (gain != (PMIC_AUDIO_OUTPUT_PGA_GAIN *) NULL) {
		if ((handle == extStereoIn.handle) &&
		    (extStereoIn.handleState == HANDLE_IN_USE)) {
			*gain = audioOutput.extStereooutputPGAGain;
			rc = PMIC_SUCCESS;
		} else if ((handle == vCodec.handle) &&
			   (vCodec.handleState == HANDLE_IN_USE)) {
			*gain = audioOutput.vCodecoutputPGAGain;
			rc = PMIC_SUCCESS;
		} else if ((handle == stDAC.handle) &&
			   (stDAC.handleState == HANDLE_IN_USE)) {
			*gain = audioOutput.stDacoutputPGAGain;
			rc = PMIC_SUCCESS;
		} else {
			rc = PMIC_PARAMETER_ERROR;
		}
	} else {
		rc = PMIC_PARAMETER_ERROR;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Enable the output mixer.
 *
 * This function enables the output mixer for the audio stream that
 * corresponds to the current handle (i.e., the Voice CODEC, Stereo DAC, or
 * the external stereo inputs).
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the mixer was successfully enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the mixer could not be enabled.
 */
PMIC_STATUS pmic_audio_output_enable_mixer(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;
	unsigned int reg_mask_mix = 0;
	unsigned int reg_write_mix = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if (((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE))) {
		reg_mask = SET_BITS(regAUDIO_RX_1, PGASTEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, PGASTEN, 1);
		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDSTDC, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDSTDC, 1);
	} else if ((handle == vCodec.handle) &&
		   (vCodec.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regAUDIO_RX_1, PGARXEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, PGARXEN, 1);
		audioOutput.vCodecOut = VCODEC_MIXER_OUT;

		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDCDC, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDCDC, 1);
	} else if ((handle == extStereoIn.handle) &&
		   (extStereoIn.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
	}

	if (reg_mask == 0) {

	} else {
		rc = pmic_write_reg(REG_AUDIO_RX_1, reg_write, reg_mask);

		if (rc == PMIC_SUCCESS) {

			rc = pmic_write_reg(REG_AUDIO_RX_0,
					    reg_write_mix, reg_mask_mix);
			if (rc == PMIC_SUCCESS) {
				pr_debug("Output PGA mixers enabled\n");
				rc = PMIC_SUCCESS;
			}

		} else {
			pr_debug("Error writing mixer enable to register\n");
		}

	}

	return rc;
}

/*!
 * @brief Disable the output mixer.
 *
 * This function disables the output mixer for the audio stream that
 * corresponds to the current handle (i.e., the Voice CODEC, Stereo DAC, or
 * the external stereo inputs).
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the mixer was successfully disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the mixer could not be disabled.
 */
PMIC_STATUS pmic_audio_output_disable_mixer(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;

	unsigned int reg_mask_mix = 0;
	unsigned int reg_write_mix = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */
	if (((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE))) {
		/*reg_mask = SET_BITS(regAUDIO_RX_1, PGASTEN, 1);
		   reg_write = SET_BITS(regAUDIO_RX_1, PGASTEN, 0); */

		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDSTDC, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDSTDC, 0);
	} else if ((handle == vCodec.handle) &&
		   (vCodec.handleState == HANDLE_IN_USE)) {
		reg_mask = SET_BITS(regAUDIO_RX_1, PGARXEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, PGARXEN, 0);
		audioOutput.vCodecOut = VCODEC_DIRECT_OUT;

		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDCDC, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDCDC, 0);
	} else if ((handle == extStereoIn.handle) &&
		   (extStereoIn.handleState == HANDLE_IN_USE)) {
		/*reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		   reg_write = SET_BITS(regAUDIO_RX_1, ARXINEN, 0); */

		reg_mask_mix = SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
		reg_write_mix = SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
	}

	if (reg_mask == 0) {

	} else {
		rc = pmic_write_reg(REG_AUDIO_RX_1, reg_write, reg_mask);

		if (rc == PMIC_SUCCESS) {

			rc = pmic_write_reg(REG_AUDIO_RX_0,
					    reg_write_mix, reg_mask_mix);
			if (rc == PMIC_SUCCESS) {
				pr_debug("Output PGA mixers disabled\n");
			}
		}
	}
	return rc;
}

/*!
 * @brief Configure and enable the output balance amplifiers.
 *
 * This function configures and enables the output balance amplifiers.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   leftGain            The desired left channel gain level.
 * @param   rightGain           The desired right channel gain level.
 *
 * @retval      PMIC_SUCCESS         If the output balance amplifiers were
 *                                   successfully configured and enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or gain levels were invalid.
 * @retval      PMIC_ERROR           If the output balance amplifiers could not
 *                                   be reconfigured or enabled.
 */
PMIC_STATUS pmic_audio_output_set_balance(const PMIC_AUDIO_HANDLE handle,
					  const PMIC_AUDIO_OUTPUT_BALANCE_GAIN
					  leftGain,
					  const PMIC_AUDIO_OUTPUT_BALANCE_GAIN
					  rightGain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;
	unsigned int reg_mask_ch = 0;
	unsigned int reg_write_ch = 0;

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if (!((leftGain >= BAL_GAIN_MINUS_21DB) && (leftGain <= BAL_GAIN_0DB))) {
		rc = PMIC_PARAMETER_ERROR;
	} else if (!((rightGain >= BAL_GAIN_MINUS_21DB)
		     && (rightGain <= BAL_GAIN_0DB))) {
		rc = PMIC_PARAMETER_ERROR;
	} else {
		if (((handle == stDAC.handle) &&
		     (stDAC.handleState == HANDLE_IN_USE)) ||
		    ((handle == vCodec.handle) &&
		     (vCodec.handleState == HANDLE_IN_USE)) ||
		    ((handle == extStereoIn.handle) &&
		     (extStereoIn.handleState == HANDLE_IN_USE))) {
			/* In mc13783 only one channel can be attenuated wrt the other.
			 * It is not possible to specify attenuation for both
			 * This function will return an error if both channels
			 * are required to be attenuated
			 * The BALLR bit is set/reset depending on whether leftGain
			 * or rightGain is specified*/
			if ((rightGain == BAL_GAIN_0DB)
			    && (leftGain == BAL_GAIN_0DB)) {
				/* Nothing to be done */
			} else if ((rightGain != BAL_GAIN_0DB)
				   && (leftGain == BAL_GAIN_0DB)) {
				/* Attenuate right channel */
				reg_mask = SET_BITS(regAUDIO_RX_1, BAL, 7);
				reg_mask_ch = SET_BITS(regAUDIO_RX_1, BALLR, 1);
				reg_write =
				    SET_BITS(regAUDIO_RX_1, BAL,
					     (BAL_GAIN_0DB - rightGain));
				/* The enum and the register values are reversed in order .. */
				reg_write_ch =
				    SET_BITS(regAUDIO_RX_1, BALLR, 0);
				/* BALLR = 0 selects right channel for atten */
			} else if ((rightGain == BAL_GAIN_0DB)
				   && (leftGain != BAL_GAIN_0DB)) {
				/* Attenuate left channel */

				reg_mask = SET_BITS(regAUDIO_RX_1, BAL, 7);
				reg_mask_ch = SET_BITS(regAUDIO_RX_1, BALLR, 1);
				reg_write =
				    SET_BITS(regAUDIO_RX_1, BAL,
					     (BAL_GAIN_0DB - leftGain));
				reg_write_ch =
				    SET_BITS(regAUDIO_RX_1, BALLR, 1);
				/* BALLR = 1 selects left channel for atten */
			} else {
				rc = PMIC_PARAMETER_ERROR;
			}

			if ((reg_mask == 0) || (reg_mask_ch == 0)) {

			} else {
				rc = pmic_write_reg(REG_AUDIO_RX_1,
						    reg_write_ch, reg_mask_ch);

				if (rc == PMIC_SUCCESS) {
					rc = pmic_write_reg(REG_AUDIO_RX_1,
							    reg_write,
							    reg_mask);

					if (rc == PMIC_SUCCESS) {
						pr_debug
						    ("Output balance attenuation set\n");
						audioOutput.balanceLeftGain =
						    leftGain;
						audioOutput.balanceRightGain =
						    rightGain;
					}
				}
			}
		}
	}
	return rc;
}

/*!
 * @brief Get the current output balance amplifier gain levels.
 *
 * This function retrieves the current output balance amplifier gain levels.
 *
 * @param   	handle              Device handle from pmic_audio_open() call.
 * @param  	leftGain            The current left channel gain level.
 * @param 	rightGain           The current right channel gain level.
 *
 * @retval      PMIC_SUCCESS         If the output balance amplifier gain levels
 *                                   were successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the output balance amplifier gain levels
 *                                   could be retrieved.
 */
PMIC_STATUS pmic_audio_output_get_balance(const PMIC_AUDIO_HANDLE handle,
					  PMIC_AUDIO_OUTPUT_BALANCE_GAIN *
					  const leftGain,
					  PMIC_AUDIO_OUTPUT_BALANCE_GAIN *
					  const rightGain)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((((handle == stDAC.handle) &&
	      (stDAC.handleState == HANDLE_IN_USE)) ||
	     ((handle == vCodec.handle) &&
	      (vCodec.handleState == HANDLE_IN_USE)) ||
	     ((handle == extStereoIn.handle) &&
	      (extStereoIn.handleState == HANDLE_IN_USE))) &&
	    ((leftGain != (PMIC_AUDIO_OUTPUT_BALANCE_GAIN *) NULL) &&
	     (rightGain != (PMIC_AUDIO_OUTPUT_BALANCE_GAIN *) NULL))) {
		*leftGain = audioOutput.balanceLeftGain;
		*rightGain = audioOutput.balanceRightGain;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Configure and enable the output mono adder.
 *
 * This function configures and enables the output mono adder.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   mode                The desired mono adder operating mode.
 *
 * @retval      PMIC_SUCCESS         If the mono adder was successfully
 *                                   configured and enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle or mono adder mode was
 *                                   invalid.
 * @retval      PMIC_ERROR           If the mono adder could not be reconfigured
 *                                   or enabled.
 */
PMIC_STATUS pmic_audio_output_enable_mono_adder(const PMIC_AUDIO_HANDLE handle,
						const PMIC_AUDIO_MONO_ADDER_MODE
						mode)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_write = 0;
	unsigned int reg_mask = SET_BITS(regAUDIO_RX_1, MONO, 3);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if ((mode >= MONO_ADDER_OFF) && (mode <= STEREO_OPPOSITE_PHASE)) {
		if (((handle == stDAC.handle) &&
		     (stDAC.handleState == HANDLE_IN_USE)) ||
		    ((handle == vCodec.handle) &&
		     (vCodec.handleState == HANDLE_IN_USE)) ||
		    ((handle == extStereoIn.handle) &&
		     (extStereoIn.handleState == HANDLE_IN_USE))) {
			if (mode == MONO_ADDER_OFF) {
				reg_write = SET_BITS(regAUDIO_RX_1, MONO, 0);
			} else if (mode == MONO_ADD_LEFT_RIGHT) {
				reg_write = SET_BITS(regAUDIO_RX_1, MONO, 2);
			} else if (mode == MONO_ADD_OPPOSITE_PHASE) {
				reg_write = SET_BITS(regAUDIO_RX_1, MONO, 3);
			} else {	/* stereo opposite */

				reg_write = SET_BITS(regAUDIO_RX_1, MONO, 1);
			}

			rc = pmic_write_reg(REG_AUDIO_RX_1,
					    reg_write, reg_mask);

			if (rc == PMIC_SUCCESS) {
				pr_debug("Output mono adder mode set\n");

			}

		} else {
			rc = PMIC_PARAMETER_ERROR;
		}
	} else {
		rc = PMIC_PARAMETER_ERROR;
	}
	return rc;
}

/*!
 * @brief Disable the output mono adder.
 *
 * This function disables the output mono adder.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the mono adder was successfully
 *                                   disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the mono adder could not be disabled.
 */
PMIC_STATUS pmic_audio_output_disable_mono_adder(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_write = 0;
	const unsigned int reg_mask = SET_BITS(regAUDIO_RX_1, MONO, 3);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	if (((handle == stDAC.handle) &&
	     (stDAC.handleState == HANDLE_IN_USE)) ||
	    ((handle == vCodec.handle) &&
	     (vCodec.handleState == HANDLE_IN_USE)) ||
	    ((handle == extStereoIn.handle) &&
	     (extStereoIn.handleState == HANDLE_IN_USE))) {
		rc = pmic_write_reg(REG_AUDIO_RX_1, reg_write, reg_mask);
	}

	return rc;
}

/*!
 * @brief Configure the mono adder output gain level.
 *
 * This function configures the mono adder output amplifier gain level.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The desired output gain level.
 *
 * @retval      PMIC_SUCCESS         If the mono adder output amplifier gain
 *                                   level was successfully set.
 * @retval      PMIC_PARAMETER_ERROR If the handle or gain level was invalid.
 * @retval      PMIC_ERROR           If the mono adder output amplifier gain
 *                                   level could not be reconfigured.
 */
PMIC_STATUS pmic_audio_output_set_mono_adder_gain(const PMIC_AUDIO_HANDLE
						  handle,
						  const
						  PMIC_AUDIO_MONO_ADDER_OUTPUT_GAIN
						  gain)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;
}

/*!
 * @brief Get the current mono adder output gain level.
 *
 * This function retrieves the current mono adder output amplifier gain level.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   gain                The current output gain level.
 *
 * @retval      PMIC_SUCCESS         If the mono adder output amplifier gain
 *                                   level was successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the mono adder output amplifier gain
 *                                   level could not be retrieved.
 */
PMIC_STATUS pmic_audio_output_get_mono_adder_gain(const PMIC_AUDIO_HANDLE
						  handle,
						  PMIC_AUDIO_MONO_ADDER_OUTPUT_GAIN
						  * const gain)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;
	return rc;
}

/*!
 * @brief Set various audio output section options.
 *
 * This function sets one or more audio output section configuration
 * options. The currently supported options include whether to disable
 * the non-inverting mono speaker output, enabling the loudspeaker common
 * bias circuit, enabling detection of headset insertion/removal, and
 * whether to automatically disable the headset amplifiers when a headset
 * insertion/removal has been detected.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   config              The desired audio output section
 *                                  configuration options to be set.
 *
 * @retval      PMIC_SUCCESS         If the desired configuration options were
 *                                   all successfully set.
 * @retval      PMIC_PARAMETER_ERROR If the handle or configuration options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the desired configuration options
 *                                   could not be set.
 */
PMIC_STATUS pmic_audio_output_set_config(const PMIC_AUDIO_HANDLE handle,
					 const PMIC_AUDIO_OUTPUT_CONFIG config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (((handle == stDAC.handle) &&
	     (stDAC.handleState == HANDLE_IN_USE)) ||
	    ((handle == vCodec.handle) &&
	     (vCodec.handleState == HANDLE_IN_USE)) ||
	    ((handle == extStereoIn.handle) &&
	     (extStereoIn.handleState == HANDLE_IN_USE))) {
		if (config & MONO_SPEAKER_INVERT_OUT_ONLY) {
			/* If this is one of the parameters */
			rc = PMIC_NOT_SUPPORTED;
		} else {
			if (config & MONO_LOUDSPEAKER_COMMON_BIAS) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ALSPREF, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ALSPREF, 1);
			}
			if (config & HEADSET_DETECT_ENABLE) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, HSDETEN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, HSDETEN, 1);
			}
			if (config & STEREO_HEADSET_AMP_AUTO_DISABLE) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
			}

			if (reg_mask == 0) {
				rc = PMIC_PARAMETER_ERROR;
			} else {
				rc = pmic_write_reg(REG_AUDIO_RX_0,
						    reg_write, reg_mask);

				if (rc == PMIC_SUCCESS) {
					pr_debug("Output config set\n");
					audioOutput.config |= config;

				}
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Clear various audio output section options.
 *
 * This function clears one or more audio output section configuration
 * options.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param   config              The desired audio output section
 *                                  configuration options to be cleared.
 *
 * @retval      PMIC_SUCCESS         If the desired configuration options were
 *                                   all successfully cleared.
 * @retval      PMIC_PARAMETER_ERROR If the handle or configuration options
 *                                   were invalid.
 * @retval      PMIC_ERROR           If the desired configuration options
 *                                   could not be cleared.
 */
PMIC_STATUS pmic_audio_output_clear_config(const PMIC_AUDIO_HANDLE handle,
					   const PMIC_AUDIO_OUTPUT_CONFIG
					   config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	/*unsigned int reg_write_RX = 0;
	   unsigned int reg_mask_RX  = 0;
	   unsigned int reg_write_TX = 0;
	   unsigned int reg_mask_TX  = 0; */
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if (((handle == stDAC.handle) &&
	     (stDAC.handleState == HANDLE_IN_USE)) ||
	    ((handle == vCodec.handle) &&
	     (vCodec.handleState == HANDLE_IN_USE)) ||
	    ((handle == extStereoIn.handle) &&
	     (extStereoIn.handleState == HANDLE_IN_USE))) {
		if (config & MONO_SPEAKER_INVERT_OUT_ONLY) {
			/* If this is one of the parameters */
			rc = PMIC_NOT_SUPPORTED;
		} else {
			if (config & MONO_LOUDSPEAKER_COMMON_BIAS) {
				reg_mask = SET_BITS(regAUDIO_RX_0, ALSPREF, 1);
				reg_write = SET_BITS(regAUDIO_RX_0, ALSPREF, 0);
			}

			if (config & HEADSET_DETECT_ENABLE) {
				reg_mask |= SET_BITS(regAUDIO_RX_0, HSDETEN, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, HSDETEN, 0);
			}

			if (config & STEREO_HEADSET_AMP_AUTO_DISABLE) {
				reg_mask |=
				    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 1);
				reg_write |=
				    SET_BITS(regAUDIO_RX_0, HSDETAUTOB, 0);
			}

			if (reg_mask == 0) {
				rc = PMIC_PARAMETER_ERROR;
			} else {
				rc = pmic_write_reg(REG_AUDIO_RX_0,
						    reg_write, reg_mask);

				if (rc == PMIC_SUCCESS) {
					pr_debug("Output config cleared\n");
					audioOutput.config &= ~config;

				}
			}
		}
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Get the current audio output section options.
 *
 * This function retrieves the current audio output section configuration
 * option settings.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 * @param  config              The current audio output section
 *                                  configuration option settings.
 *
 * @retval      PMIC_SUCCESS         If the current configuration options were
 *                                   successfully retrieved.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the current configuration options
 *                                   could not be retrieved.
 */
PMIC_STATUS pmic_audio_output_get_config(const PMIC_AUDIO_HANDLE handle,
					 PMIC_AUDIO_OUTPUT_CONFIG *
					 const config)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Use a critical section to ensure a consistent hardware state. */
	if (down_interruptible(&mutex))
		return PMIC_SYSTEM_ERROR_EINTR;

	if ((((handle == stDAC.handle) &&
	      (stDAC.handleState == HANDLE_IN_USE)) ||
	     ((handle == vCodec.handle) &&
	      (vCodec.handleState == HANDLE_IN_USE)) ||
	     ((handle == extStereoIn.handle) &&
	      (extStereoIn.handleState == HANDLE_IN_USE))) &&
	    (config != (PMIC_AUDIO_OUTPUT_CONFIG *) NULL)) {
		*config = audioOutput.config;

		rc = PMIC_SUCCESS;
	}

	/* Exit the critical section. */
	up(&mutex);

	return rc;
}

/*!
 * @brief Enable the phantom ground circuit that is used to help identify
 *        the type of headset that has been inserted.
 *
 * This function enables the phantom ground circuit that is used to help
 * identify the type of headset (e.g., stereo or mono) that has been inserted.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the phantom ground circuit was
 *                                   successfully enabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the phantom ground circuit could not
 *                                   be enabled.
 */
PMIC_STATUS pmic_audio_output_enable_phantom_ground()
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_mask = SET_BITS(regAUDIO_RX_0, HSPGDIS, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	rc = pmic_write_reg(REG_AUDIO_RX_0, 0, reg_mask);
	if (rc == PMIC_SUCCESS) {
		pr_debug("Phantom ground enabled\n");

	}
	return rc;
}

/*!
 * @brief Disable the phantom ground circuit that is used to help identify
 *        the type of headset that has been inserted.
 *
 * This function disables the phantom ground circuit that is used to help
 * identify the type of headset (e.g., stereo or mono) that has been inserted.
 *
 * @param   handle              Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the phantom ground circuit was
 *                                   successfully disabled.
 * @retval      PMIC_PARAMETER_ERROR If the handle was invalid.
 * @retval      PMIC_ERROR           If the phantom ground circuit could not
 *                                   be disabled.
 */
PMIC_STATUS pmic_audio_output_disable_phantom_ground()
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	const unsigned int reg_mask = SET_BITS(regAUDIO_RX_0, HSPGDIS, 1);

	/* No critical section required here since we are not updating any
	 * global data.
	 */

	rc = pmic_write_reg(REG_AUDIO_RX_0, 1, reg_mask);
	if (rc == PMIC_SUCCESS) {
		pr_debug("Phantom ground disabled\n");

	}
	return rc;
}

/*@}*/

/**************************************************************************
 * Static functions.
 **************************************************************************
 */

/*!
 * @name Audio Driver Internal Support Functions
 * These non-exported internal functions are used to support the functionality
 * of the exported audio APIs.
 */
/*@{*/

/*!
 * @brief Enables the 5.6V boost for the microphone bias 2 circuit.
 *
 * This function enables the switching regulator SW3 and configures it to
 * provide the 5.6V boost that is required for driving the microphone bias 2
 * circuit when using a 5-pole jack configuration (which is the case for the
 * Sphinx board).
 *
 * @retval      PMIC_SUCCESS         The 5.6V boost was successfully enabled.
 * @retval      PMIC_ERROR           Failed to enable the 5.6V boost.
 */
/*
static PMIC_STATUS pmic_audio_mic_boost_enable(void)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;

	return rc;
}
*/
/*!
 * @brief Disables the 5.6V boost for the microphone bias 2 circuit.
 *
 * This function disables the switching regulator SW3 to turn off the 5.6V
 * boost for the microphone bias 2 circuit.
 *
 * @retval      PMIC_SUCCESS         The 5.6V boost was successfully disabled.
 * @retval      PMIC_ERROR           Failed to disable the 5.6V boost.
 */
/*
static PMIC_STATUS pmic_audio_mic_boost_disable(void)
{
	PMIC_STATUS rc = PMIC_NOT_SUPPORTED;

	return rc;
}
*/

/*!
 * @brief Free a device handle previously acquired by calling pmic_audio_open().
 *
 * Terminate further access to the PMIC audio hardware that was previously
 * acquired by calling pmic_audio_open(). This now allows another thread to
 * successfully call pmic_audio_open() to gain access.
 *
 * Note that we will shutdown/reset the Voice CODEC or Stereo DAC as well as
 * any associated audio input/output components that are no longer required.
 *
 * Also note that this function should only be called with the mutex already
 * acquired.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the close request was successful.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 */
static PMIC_STATUS pmic_audio_close_handle(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;

	/* Match up the handle to the audio device and then close it. */
	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		/* Also shutdown the Stereo DAC hardware. The simplest way to
		 * do this is to simply call pmic_audio_reset_device() which will
		 * restore the ST_DAC register to it's initial power-on state.
		 *
		 * This will also shutdown the audio output section if no one
		 * else is still using it.
		 */
		rc = pmic_audio_reset_device(stDAC.handle);

		if (rc == PMIC_SUCCESS) {
			stDAC.handle = AUDIO_HANDLE_NULL;
			stDAC.handleState = HANDLE_FREE;
		}
	} else if ((handle == vCodec.handle) &&
		   (vCodec.handleState == HANDLE_IN_USE)) {
		/* Also shutdown the Voice CODEC and audio input hardware. The
		 * simplest way to do this is to simply call pmic_audio_reset_device()
		 * which will restore the AUD_CODEC register to it's initial
		 * power-on state.
		 *
		 * This will also shutdown the audio output section if no one
		 * else is still using it.
		 */
		rc = pmic_audio_reset_device(vCodec.handle);
		if (rc == PMIC_SUCCESS) {
			vCodec.handle = AUDIO_HANDLE_NULL;
			vCodec.handleState = HANDLE_FREE;
		}
	} else if ((handle == extStereoIn.handle) &&
		   (extStereoIn.handleState == HANDLE_IN_USE)) {

		/* Call pmic_audio_reset_device() here to shutdown the audio output
		 * section if no one else is still using it.
		 */
		rc = pmic_audio_reset_device(extStereoIn.handle);

		if (rc == PMIC_SUCCESS) {
			extStereoIn.handle = AUDIO_HANDLE_NULL;
			extStereoIn.handleState = HANDLE_FREE;
		}
	}

	return rc;
}

/*!
 * @brief Reset the selected audio hardware control registers to their
 *        power on state.
 *
 * This resets all of the audio hardware control registers currently
 * associated with the device handle back to their power on states. For
 * example, if the handle is associated with the Stereo DAC and a
 * specific output port and output amplifiers, then this function will
 * reset all of those components to their initial power on state.
 *
 * This function can only be called if the mutex has already been acquired.
 *
 * @param   handle          Device handle from pmic_audio_open() call.
 *
 * @retval      PMIC_SUCCESS         If the reset operation was successful.
 * @retval      PMIC_PARAMETER_ERROR If the handle is invalid.
 * @retval      PMIC_ERROR           If the reset was unsuccessful.
 */
static PMIC_STATUS pmic_audio_reset_device(const PMIC_AUDIO_HANDLE handle)
{
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	unsigned int reg_mask = 0;

	if ((handle == stDAC.handle) && (stDAC.handleState == HANDLE_IN_USE)) {
		/* Also shutdown the audio output section if nobody else is using it.
		   if ((vCodec.handleState == HANDLE_FREE) &&
		   (extStereoIn.handleState == HANDLE_FREE))
		   {
		   pmic_write_reg(REG_RX_AUD_AMPS, RESET_RX_AUD_AMPS,
		   REG_FULLMASK);
		   } */

		rc = pmic_write_reg(REG_AUDIO_STEREO_DAC,
				    RESET_ST_DAC, REG_FULLMASK);

		if (rc == PMIC_SUCCESS) {
			rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
					    RESET_SSI_NETWORK,
					    REG_SSI_STDAC_MASK);
			if (rc == PMIC_SUCCESS) {
				/* Also reset the driver state information to match. Note that we
				 * keep the device handle and event callback settings unchanged
				 * since these don't affect the actual hardware and we rely on
				 * the user to explicitly close the handle or deregister callbacks
				 */
				stDAC.busID = AUDIO_DATA_BUS_1;
				stDAC.protocol = NORMAL_MSB_JUSTIFIED_MODE;
				stDAC.protocol_set = false;
				stDAC.masterSlave = BUS_MASTER_MODE;
				stDAC.numSlots = USE_2_TIMESLOTS;
				stDAC.clockIn = CLOCK_IN_CLIA;
				stDAC.samplingRate = STDAC_RATE_44_1_KHZ;
				stDAC.clockFreq = STDAC_CLI_13MHZ;
				stDAC.invert = NO_INVERT;
				stDAC.timeslot = USE_TS0_TS1;
				stDAC.config = (PMIC_AUDIO_STDAC_CONFIG) 0;

			}
		}
	} else if ((handle == vCodec.handle)
		   && (vCodec.handleState == HANDLE_IN_USE)) {
		/* Disable the audio input section when disabling the Voice CODEC. */
		pmic_write_reg(REG_AUDIO_TX, RESET_AUDIO_TX, REG_FULLMASK);

		rc = pmic_write_reg(REG_AUDIO_CODEC,
				    RESET_AUD_CODEC, REG_FULLMASK);

		if (rc == PMIC_SUCCESS) {
			rc = pmic_write_reg(REG_AUDIO_SSI_NETWORK,
					    RESET_SSI_NETWORK,
					    REG_SSI_VCODEC_MASK);
			if (rc == PMIC_SUCCESS) {

				/* Also reset the driver state information to match. Note that we
				 * keep the device handle and event callback settings unchanged
				 * since these don't affect the actual hardware and we rely on
				 * the user to explicitly close the handle or deregister callbacks
				 */
				vCodec.busID = AUDIO_DATA_BUS_2;
				vCodec.protocol = NETWORK_MODE;
				vCodec.protocol_set = false;
				vCodec.masterSlave = BUS_SLAVE_MODE;
				vCodec.numSlots = USE_4_TIMESLOTS;
				vCodec.clockIn = CLOCK_IN_CLIB;
				vCodec.samplingRate = VCODEC_RATE_8_KHZ;
				vCodec.clockFreq = VCODEC_CLI_13MHZ;
				vCodec.invert = NO_INVERT;
				vCodec.timeslot = USE_TS0;
				vCodec.config =
				    INPUT_HIGHPASS_FILTER |
				    OUTPUT_HIGHPASS_FILTER;

			}
		}

	} else if ((handle == extStereoIn.handle) &&
		   (extStereoIn.handleState == HANDLE_IN_USE)) {
		/* Disable the Ext stereo Amplifier and disable it as analog mixer input */
		reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		pmic_write_reg(REG_AUDIO_RX_1, 0, reg_mask);

		reg_mask = SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
		pmic_write_reg(REG_AUDIO_RX_0, 0, reg_mask);

		/* We don't need to reset any other registers for this case. */
		rc = PMIC_SUCCESS;
	}

	return rc;
}

/*!
 * @brief Deregister the callback function and event mask currently associated
 *        with an audio device handle.
 *
 * This function deregisters any existing callback function and event mask for
 * the given audio device handle. This is done by either calling the
 * pmic_audio_clear_callback() API or by closing the device handle.
 *
 * Note that this function should only be called with the mutex already
 * acquired. We will also acquire the spinlock here to prevent possible
 * race conditions with the interrupt handler.
 *
 * @param[in]   callback            The current event callback function pointer.
 * @param[in]   eventMask           The current audio event mask.
 *
 * @retval      PMIC_SUCCESS         If the callback function and event mask
 *                                   were both successfully deregistered.
 * @retval      PMIC_ERROR           If either the callback function or the
 *                                   event mask was not successfully
 *                                   deregistered.
 */

static PMIC_STATUS pmic_audio_deregister(void *callback,
					 PMIC_AUDIO_EVENTS * const eventMask)
{
	unsigned long flags;
	pmic_event_callback_t eventNotify;
	PMIC_STATUS rc = PMIC_SUCCESS;

	/* Deregister each of the PMIC events that we had previously
	 * registered for by calling pmic_event_subscribe().
	 */
	if (*eventMask & (HEADSET_DETECTED)) {
		/* We need to deregister for the A1 amplifier interrupt. */
		eventNotify.func = callback;
		eventNotify.param = (void *)(CORE_EVENT_HSDETI);
		if (pmic_event_unsubscribe(EVENT_HSDETI, eventNotify) ==
		    PMIC_SUCCESS) {
			*eventMask &= ~(HEADSET_DETECTED);
			pr_debug("Deregistered for EVENT_HSDETI\n");
		} else {
			rc = PMIC_ERROR;
		}
	}

	if (*eventMask & (HEADSET_STEREO)) {
		/* We need to deregister for the A1 amplifier interrupt. */
		eventNotify.func = callback;
		eventNotify.param = (void *)(CORE_EVENT_HSLI);
		if (pmic_event_unsubscribe(EVENT_HSLI, eventNotify) ==
		    PMIC_SUCCESS) {
			*eventMask &= ~(HEADSET_STEREO);
			pr_debug("Deregistered for EVENT_HSLI\n");
		} else {
			rc = PMIC_ERROR;
		}
	}
	if (*eventMask & (HEADSET_THERMAL_SHUTDOWN)) {
		/* We need to deregister for the A1 amplifier interrupt. */
		eventNotify.func = callback;
		eventNotify.param = (void *)(CORE_EVENT_ALSPTHI);
		if (pmic_event_unsubscribe(EVENT_ALSPTHI, eventNotify) ==
		    PMIC_SUCCESS) {
			*eventMask &= ~(HEADSET_THERMAL_SHUTDOWN);
			pr_debug("Deregistered for EVENT_ALSPTHI\n");
		} else {
			rc = PMIC_ERROR;
		}
	}
	if (*eventMask & (HEADSET_SHORT_CIRCUIT)) {
		/* We need to deregister for the A1 amplifier interrupt. */
		eventNotify.func = callback;
		eventNotify.param = (void *)(CORE_EVENT_AHSSHORTI);
		if (pmic_event_unsubscribe(EVENT_AHSSHORTI, eventNotify) ==
		    PMIC_SUCCESS) {
			*eventMask &= ~(HEADSET_SHORT_CIRCUIT);
			pr_debug("Deregistered for EVENT_AHSSHORTI\n");
		} else {
			rc = PMIC_ERROR;
		}
	}

	if (rc == PMIC_SUCCESS) {
		/* We need to grab the spinlock here to create a critical section to
		 * avoid any possible race conditions with the interrupt handler
		 */
		spin_lock_irqsave(&lock, flags);

		/* Restore the initial reset values for the callback function
		 * and event mask parameters. This should be NULL and zero,
		 * respectively.
		 */
		callback = NULL;
		*eventMask = 0;

		/* Exit the critical section. */
		spin_unlock_irqrestore(&lock, flags);
	}

	return rc;
}

/*!
 * @brief enable/disable fm output.
 *
 * @param[in]   enable            true to enable false to disable
 */
PMIC_STATUS pmic_audio_fm_output_enable(bool enable)
{
	unsigned int reg_mask = 0;
	unsigned int reg_write = 0;
	PMIC_STATUS rc = PMIC_PARAMETER_ERROR;
	if (enable) {
		pmic_audio_antipop_enable(ANTI_POP_RAMP_FAST);
		reg_mask |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, AHSLEN, 1);
		reg_mask |= SET_BITS(regAUDIO_RX_0, AHSREN, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, AHSREN, 1);

		reg_mask |= SET_BITS(regAUDIO_RX_0, AHSSEL, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, AHSSEL, 1);

		reg_mask |= SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);

		reg_mask |= SET_BITS(regAUDIO_RX_0, HSPGDIS, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, HSPGDIS, 0);
	} else {
		reg_mask |= SET_BITS(regAUDIO_RX_0, ADDRXIN, 1);
		reg_write |= SET_BITS(regAUDIO_RX_0, ADDRXIN, 0);
	}
	rc = pmic_write_reg(REG_AUDIO_RX_0, reg_write, reg_mask);
	if (rc != PMIC_SUCCESS)
		return rc;
	if (enable) {
		reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
	} else {
		reg_mask = SET_BITS(regAUDIO_RX_1, ARXINEN, 1);
		reg_write = SET_BITS(regAUDIO_RX_1, ARXINEN, 0);
	}
	rc = pmic_write_reg(REG_AUDIO_RX_1, reg_write, reg_mask);
	return rc;
}

/*@}*/

/**************************************************************************
 * Module initialization and termination functions.
 *
 * Note that if this code is compiled into the kernel, then the
 * module_init() function will be called within the device_initcall()
 * group.
 **************************************************************************
 */

/*!
 * @name Audio Driver Loading/Unloading Functions
 * These non-exported internal functions are used to support the audio
 * device driver initialization and de-initialization operations.
 */
/*@{*/

/*!
 * @brief This is the audio device driver initialization function.
 *
 * This function is called by the kernel when this device driver is first
 * loaded.
 */
static int __init mc13783_pmic_audio_init(void)
{
	printk(KERN_INFO "PMIC Audio driver loading...\n");

	return 0;
}

/*!
 * @brief This is the audio device driver de-initialization function.
 *
 * This function is called by the kernel when this device driver is about
 * to be unloaded.
 */
static void __exit mc13783_pmic_audio_exit(void)
{
	printk(KERN_INFO "PMIC Audio driver unloading...\n");

	/* Close all device handles that are still open. This will also
	 * deregister any callbacks that may still be active.
	 */
	if (stDAC.handleState == HANDLE_IN_USE) {
		pmic_audio_close(stDAC.handle);
	}
	if (vCodec.handleState == HANDLE_IN_USE) {
		pmic_audio_close(vCodec.handle);
	}
	if (extStereoIn.handleState == HANDLE_IN_USE) {
		pmic_audio_close(extStereoIn.handle);
	}

	/* Explicitly reset all of the audio registers so that there is no
	 * possibility of leaving the  audio hardware in a state
	 * where it can cause problems if there is no device driver loaded.
	 */
	pmic_write_reg(REG_AUDIO_STEREO_DAC, RESET_ST_DAC, REG_FULLMASK);
	pmic_write_reg(REG_AUDIO_CODEC, RESET_AUD_CODEC, REG_FULLMASK);
	pmic_write_reg(REG_AUDIO_TX, RESET_AUDIO_TX, REG_FULLMASK);
	pmic_write_reg(REG_AUDIO_SSI_NETWORK, RESET_SSI_NETWORK, REG_FULLMASK);
	pmic_write_reg(REG_AUDIO_RX_0, RESET_AUDIO_RX_0, REG_FULLMASK);
	pmic_write_reg(REG_AUDIO_RX_1, RESET_AUDIO_RX_1, REG_FULLMASK);
}

/*@}*/

/*
 * Module entry points and description information.
 */

module_init(mc13783_pmic_audio_init);
module_exit(mc13783_pmic_audio_exit);

MODULE_DESCRIPTION("PMIC - mc13783 ADC driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
