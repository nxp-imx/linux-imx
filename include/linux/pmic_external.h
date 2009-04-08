/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_MXC_PMIC_EXTERNAL_H__
#define __ASM_ARCH_MXC_PMIC_EXTERNAL_H__

#ifdef __KERNEL__
#include <linux/list.h>
#endif

/*!
 * @defgroup PMIC_DRVRS PMIC Drivers
 */

/*!
 * @defgroup PMIC_CORE PMIC Protocol Drivers
 * @ingroup PMIC_DRVRS
 */

/*!
 * @file arch-mxc/pmic_external.h
 * @brief This file contains interface of PMIC protocol driver.
 *
 * @ingroup PMIC_CORE
 */

#include <linux/ioctl.h>
#include <linux/pmic_status.h>

/*!
 * This is the enumeration of versions of PMIC
 */
typedef enum {
	PMIC_MC13783 = 1,	/*!< MC13783 */
	PMIC_SC55112 = 2,	/*!< SC55112 */
	PMIC_MC13892 = 3,
	PMIC_MC34704 = 4
} pmic_id_t;

/*!
 * @struct pmic_version_t
 * @brief PMIC version and revision
 */
typedef struct {
	/*!
	 * PMIC version identifier.
	 */
	pmic_id_t id;
	/*!
	 * Revision of the PMIC.
	 */
	int revision;
} pmic_version_t;

/*!
 * struct pmic_event_callback_t
 * @brief This structure contains callback function pointer and its
 * parameter to be used when un/registering and launching a callback
 * for an event.
 */
typedef struct {
	/*!
	 * call back function
	 */
	void (*func) (void *);

	/*!
	 * call back function parameter
	 */
	void *param;
} pmic_event_callback_t;

/*!
 * This structure is used with IOCTL.
 * It defines register, register value, register mask and event number
 */
typedef struct {
	/*!
	 * register number
	 */
	int reg;
	/*!
	 * value of register
	 */
	unsigned int reg_value;
	/*!
	 * mask of bits, only used with PMIC_WRITE_REG
	 */
	unsigned int reg_mask;
} register_info;

/*!
 * @name IOCTL definitions for sc55112 core driver
 */
/*! @{ */
/*! Read a PMIC register */
#define PMIC_READ_REG          _IOWR('P', 0xa0, register_info*)
/*! Write a PMIC register */
#define PMIC_WRITE_REG         _IOWR('P', 0xa1, register_info*)
/*! Subscribe a PMIC interrupt event */
#define PMIC_SUBSCRIBE         _IOR('P', 0xa2, int)
/*! Unsubscribe a PMIC interrupt event */
#define PMIC_UNSUBSCRIBE       _IOR('P', 0xa3, int)
/*! Subscribe a PMIC event for user notification*/
#define PMIC_NOTIFY_USER       _IOR('P', 0xa4, int)
/*! Get the PMIC event occured for which user recieved notification */
#define PMIC_GET_NOTIFY	       _IOW('P', 0xa5, int)
/*! @} */

/*!
 * This is PMIC registers valid bits
 */
#define PMIC_ALL_BITS           0xFFFFFF
#define PMIC_MAX_EVENTS		48

#define PMIC_ARBITRATION	"NULL"

#ifdef CONFIG_MXC_PMIC_MC13783
/*!
 * This is the enumeration of register names of MC13783
 */
typedef enum {
	/*!
	 * REG_INTERRUPT_STATUS_0
	 */
	REG_INTERRUPT_STATUS_0 = 0,
	/*!
	 * REG_INTERRUPT_MASK_0
	 */
	REG_INTERRUPT_MASK_0,
	/*!
	 * REG_INTERRUPT_SENSE_0
	 */
	REG_INTERRUPT_SENSE_0,
	/*!
	 * REG_INTERRUPT_STATUS_1
	 */
	REG_INTERRUPT_STATUS_1,
	/*!
	 * REG_INTERRUPT_MASK_1
	 */
	REG_INTERRUPT_MASK_1,
	/*!
	 * REG_INTERRUPT_SENSE_1
	 */
	REG_INTERRUPT_SENSE_1,
	/*!
	 * REG_POWER_UP_MODE_SENSE
	 */
	REG_POWER_UP_MODE_SENSE,
	/*!
	 * REG_REVISION
	 */
	REG_REVISION,
	/*!
	 * REG_SEMAPHORE
	 */
	REG_SEMAPHORE,
	/*!
	 * REG_ARBITRATION_PERIPHERAL_AUDIO
	 */
	REG_ARBITRATION_PERIPHERAL_AUDIO,
	/*!
	 * REG_ARBITRATION_SWITCHERS
	 */
	REG_ARBITRATION_SWITCHERS,
	/*!
	 * REG_ARBITRATION_REGULATORS_0
	 */
	REG_ARBITRATION_REGULATORS_0,
	/*!
	 * REG_ARBITRATION_REGULATORS_1
	 */
	REG_ARBITRATION_REGULATORS_1,
	/*!
	 * REG_POWER_CONTROL_0
	 */
	REG_POWER_CONTROL_0,
	/*!
	 * REG_POWER_CONTROL_1
	 */
	REG_POWER_CONTROL_1,
	/*!
	 * REG_POWER_CONTROL_2
	 */
	REG_POWER_CONTROL_2,
	/*!
	 * REG_REGEN_ASSIGNMENT
	 */
	REG_REGEN_ASSIGNMENT,
	/*!
	 * REG_CONTROL_SPARE
	 */
	REG_CONTROL_SPARE,
	/*!
	 * REG_MEMORY_A
	 */
	REG_MEMORY_A,
	/*!
	 * REG_MEMORY_B
	 */
	REG_MEMORY_B,
	/*!
	 * REG_RTC_TIME
	 */
	REG_RTC_TIME,
	/*!
	 * REG_RTC_ALARM
	 */
	REG_RTC_ALARM,
	/*!
	 * REG_RTC_DAY
	 */
	REG_RTC_DAY,
	/*!
	 * REG_RTC_DAY_ALARM
	 */
	REG_RTC_DAY_ALARM,
	/*!
	 * REG_SWITCHERS_0
	 */
	REG_SWITCHERS_0,
	/*!
	 * REG_SWITCHERS_1
	 */
	REG_SWITCHERS_1,
	/*!
	 * REG_SWITCHERS_2
	 */
	REG_SWITCHERS_2,
	/*!
	 * REG_SWITCHERS_3
	 */
	REG_SWITCHERS_3,
	/*!
	 * REG_SWITCHERS_4
	 */
	REG_SWITCHERS_4,
	/*!
	 * REG_SWITCHERS_5
	 */
	REG_SWITCHERS_5,
	/*!
	 * REG_REGULATOR_SETTING_0
	 */
	REG_REGULATOR_SETTING_0,
	/*!
	 * REG_REGULATOR_SETTING_1
	 */
	REG_REGULATOR_SETTING_1,
	/*!
	 * REG_REGULATOR_MODE_0
	 */
	REG_REGULATOR_MODE_0,
	/*!
	 * REG_REGULATOR_MODE_1
	 */
	REG_REGULATOR_MODE_1,
	/*!
	 * REG_POWER_MISCELLANEOUS
	 */
	REG_POWER_MISCELLANEOUS,
	/*!
	 * REG_POWER_SPARE
	 */
	REG_POWER_SPARE,
	/*!
	 * REG_AUDIO_RX_0
	 */
	REG_AUDIO_RX_0,
	/*!
	 * REG_AUDIO_RX_1
	 */
	REG_AUDIO_RX_1,
	/*!
	 * REG_AUDIO_TX
	 */
	REG_AUDIO_TX,
	/*!
	 * REG_AUDIO_SSI_NETWORK
	 */
	REG_AUDIO_SSI_NETWORK,
	/*!
	 * REG_AUDIO_CODEC
	 */
	REG_AUDIO_CODEC,
	/*!
	 * REG_AUDIO_STEREO_DAC
	 */
	REG_AUDIO_STEREO_DAC,
	/*!
	 * REG_AUDIO_SPARE
	 */
	REG_AUDIO_SPARE,
	/*!
	 * REG_ADC_0
	 */
	REG_ADC_0,
	/*!
	 * REG_ADC_1
	 */
	REG_ADC_1,
	/*!
	 * REG_ADC_2
	 */
	REG_ADC_2,
	/*!
	 * REG_ADC_3
	 */
	REG_ADC_3,
	/*!
	 * REG_ADC_4
	 */
	REG_ADC_4,
	/*!
	 * REG_CHARGER
	 */
	REG_CHARGER,
	/*!
	 * REG_USB
	 */
	REG_USB,
	/*!
	 * REG_CHARGE_USB_SPARE
	 */
	REG_CHARGE_USB_SPARE,
	/*!
	 * REG_LED_CONTROL_0
	 */
	REG_LED_CONTROL_0,
	/*!
	 * REG_LED_CONTROL_1
	 */
	REG_LED_CONTROL_1,
	/*!
	 * REG_LED_CONTROL_2
	 */
	REG_LED_CONTROL_2,
	/*!
	 * REG_LED_CONTROL_3
	 */
	REG_LED_CONTROL_3,
	/*!
	 * REG_LED_CONTROL_4
	 */
	REG_LED_CONTROL_4,
	/*!
	 * REG_LED_CONTROL_5
	 */
	REG_LED_CONTROL_5,
	/*!
	 * REG_SPARE
	 */
	REG_SPARE,
	/*!
	 * REG_TRIM_0
	 */
	REG_TRIM_0,
	/*!
	 * REG_TRIM_1
	 */
	REG_TRIM_1,
	/*!
	 * REG_TEST_0
	 */
	REG_TEST_0,
	/*!
	 * REG_TEST_1
	 */
	REG_TEST_1,
	/*!
	 * REG_TEST_2
	 */
	REG_TEST_2,
	/*!
	 * REG_TEST_3
	 */
	REG_TEST_3,
	/*!
	 * REG_NB
	 */
	REG_NB,
} pmic_reg;

/*!
 * This is event list of mc13783 interrupt
 */

typedef enum {
	/*!
	 * ADC has finished requested conversions
	 */
	EVENT_ADCDONEI = 0,
	/*!
	 * ADCBIS has finished requested conversions
	 */
	EVENT_ADCBISDONEI = 1,
	/*!
	 * Touchscreen wakeup
	 */
	EVENT_TSI = 2,
	/*!
	 * ADC reading above high limit
	 */
	EVENT_WHIGHI = 3,
	/*!
	 * ADC reading below low limit
	 */
	EVENT_WLOWI = 4,
	/*!
	 * Charger attach and removal
	 */
	EVENT_CHGDETI = 6,
	/*!
	 * Charger over-voltage detection
	 */
	EVENT_CHGOVI = 7,
	/*!
	 * Charger path reverse current
	 */
	EVENT_CHGREVI = 8,
	/*!
	 * Charger path short circuit
	 */
	EVENT_CHGSHORTI = 9,
	/*!
	 * BP regulator current or voltage regulation
	 */
	EVENT_CCCVI = 10,
	/*!
	 * Charge current below threshold
	 */
	EVENT_CHRGCURRI = 11,
	/*!
	 * BP turn on threshold detection
	 */
	EVENT_BPONI = 12,
	/*!
	 * End of life / low battery detect
	 */
	EVENT_LOBATLI = 13,
	/*!
	 * Low battery warning
	 */
	EVENT_LOBATHI = 14,
	/*!
	 * USB detect
	 */
	EVENT_USBI = 16,
	/*!
	 * USB ID Line detect
	 */
	EVENT_IDI = 19,
	/*!
	 * Single ended 1 detect
	 */
	EVENT_SE1I = 21,
	/*!
	 * Car-kit detect
	 */
	EVENT_CKDETI = 22,
	/*!
	 * 1 Hz time-tick
	 */
	EVENT_E1HZI = 24,
	/*!
	 * Time of day alarm
	 */
	EVENT_TODAI = 25,
	/*!
	 * ON1B event
	 */
	EVENT_ONOFD1I = 27,
	/*!
	 * ON2B event
	 */
	EVENT_ONOFD2I = 28,
	/*!
	 * ON3B event
	 */
	EVENT_ONOFD3I = 29,
	/*!
	 * System reset
	 */
	EVENT_SYSRSTI = 30,
	/*!
	 * RTC reset occurred
	 */
	EVENT_RTCRSTI = 31,
	/*!
	 * Power cut event
	 */
	EVENT_PCI = 32,
	/*!
	 * Warm start event
	 */
	EVENT_WARMI = 33,
	/*!
	 * Memory hold event
	 */
	EVENT_MEMHLDI = 34,
	/*!
	 * Power ready
	 */
	EVENT_PWRRDYI = 35,
	/*!
	 * Thermal warning lower threshold
	 */
	EVENT_THWARNLI = 36,
	/*!
	 * Thermal warning higher threshold
	 */
	EVENT_THWARNHI = 37,
	/*!
	 * Clock source change
	 */
	EVENT_CLKI = 38,
	/*!
	 * Semaphore
	 */
	EVENT_SEMAFI = 39,
	/*!
	 * Microphone bias 2 detect
	 */
	EVENT_MC2BI = 41,
	/*!
	 * Headset attach
	 */
	EVENT_HSDETI = 42,
	/*!
	 * Stereo headset detect
	 */
	EVENT_HSLI = 43,
	/*!
	 * Thermal shutdown ALSP
	 */
	EVENT_ALSPTHI = 44,
	/*!
	 * Short circuit on AHS outputs
	 */
	EVENT_AHSSHORTI = 45,
	/*!
	 * number of event
	 */
	EVENT_NB,
} type_event;

/*!
 * This enumeration all senses of MC13783.
 */
typedef enum {
	/*!
	 * Charger attach sense
	 */
	SENSE_CHGDETS = 6,
	/*!
	 * Charger over-voltage sense
	 */
	SENSE_CHGOVS,
	/*!
	 * Charger reverse current
	 * If 1 current flows into phone
	 */
	SENSE_CHGREVS,
	/*!
	 * Charger short circuit
	 */
	SENSE_CHGSHORTS,
	/*!
	 * Charger regulator operating mode
	 */
	SENSE_CCCVS,
	/*!
	 * Charger current below threshold
	 */
	SENSE_CHGCURRS,
	/*!
	 * BP turn on
	 */
	SENSE_BPONS,
	/*!
	 * Low bat detect
	 */
	SENSE_LOBATLS,
	/*!
	 * Low bat warning
	 */
	SENSE_LOBATHS,
	/*!
	 * UDPS
	 */
	SENSE_UDPS,
	/*!
	 * USB 4V4
	 */
	SENSE_USB4V4S,
	/*!
	 * USB 2V0
	 */
	SENSE_USB2V0S,
	/*!
	 * USB 0V8
	 */
	SENSE_USB0V8S,
	/*!
	 * ID Floats
	 */
	SENSE_ID_FLOATS,
	/*!
	 * ID Gnds
	 */
	SENSE_ID_GNDS,
	/*!
	 * Single ended
	 */
	SENSE_SE1S,
	/*!
	 * Car-kit detect
	 */
	SENSE_CKDETS,
	/*!
	 * UDMS
	 */
	SENSE_UDMS,
	/*!
	 * mic bias detect
	 */
	SENSE_MC2BS,
	/*!
	 * headset attached
	 */
	SENSE_HSDETS,
	/*!
	 * ST headset attached
	 */
	SENSE_HSLS,
	/*!
	 * Thermal shutdown ALSP
	 */
	SENSE_ALSPTHS,
	/*!
	 * short circuit on AHS
	 */
	SENSE_AHSSHORTS,
	/*!
	 * ON1B pin is hight
	 */
	SENSE_ONOFD1S,
	/*!
	 * ON2B pin is hight
	 */
	SENSE_ONOFD2S,
	/*!
	 * ON3B pin is hight
	 */
	SENSE_ONOFD3S,
	/*!
	 * System reset power ready
	 */
	SENSE_PWRRDYS,
	/*!
	 * Thermal warning higher threshold
	 */
	SENSE_THWARNHS,
	/*!
	 * Thermal warning lower threshold
	 */
	SENSE_THWARNLS,
	/*!
	 * Clock source is XTAL
	 */
	SENSE_CLKS,
} t_sensor;

/*!
 * This structure is used to read all sense bits of MC13783.
 */
typedef struct {
	/*!
	 * Charger attach sense
	 */
	bool sense_chgdets;
	/*!
	 * Charger over-voltage sense
	 */
	bool sense_chgovs;
	/*!
	 * Charger reverse current
	 * If 1 current flows into phone
	 */
	bool sense_chgrevs;
	/*!
	 * Charger short circuit
	 */
	bool sense_chgshorts;
	/*!
	 * Charger regulator operating mode
	 */
	bool sense_cccvs;
	/*!
	 * Charger current below threshold
	 */
	bool sense_chgcurrs;
	/*!
	 * BP turn on
	 */
	bool sense_bpons;
	/*!
	 * Low bat detect
	 */
	bool sense_lobatls;
	/*!
	 * Low bat warning
	 */
	bool sense_lobaths;
	/*!
	 * USB 4V4
	 */
	bool sense_usb4v4s;
	/*!
	 * USB 2V0
	 */
	bool sense_usb2v0s;
	/*!
	 * USB 0V8
	 */
	bool sense_usb0v8s;
	/*!
	 * ID Floats
	 */
	bool sense_id_floats;
	/*!
	 * ID Gnds
	 */
	bool sense_id_gnds;
	/*!
	 * Single ended
	 */
	bool sense_se1s;
	/*!
	 * Car-kit detect
	 */
	bool sense_ckdets;
	/*!
	 * mic bias detect
	 */
	bool sense_mc2bs;
	/*!
	 * headset attached
	 */
	bool sense_hsdets;
	/*!
	 * ST headset attached
	 */
	bool sense_hsls;
	/*!
	 * Thermal shutdown ALSP
	 */
	bool sense_alspths;
	/*!
	 * short circuit on AHS
	 */
	bool sense_ahsshorts;
	/*!
	 * ON1B pin is hight
	 */
	bool sense_onofd1s;
	/*!
	 * ON2B pin is hight
	 */
	bool sense_onofd2s;
	/*!
	 * ON3B pin is hight
	 */
	bool sense_onofd3s;
	/*!
	 * System reset power ready
	 */
	bool sense_pwrrdys;
	/*!
	 * Thermal warning higher threshold
	 */
	bool sense_thwarnhs;
	/*!
	 * Thermal warning lower threshold
	 */
	bool sense_thwarnls;
	/*!
	 * Clock source is XTAL
	 */
	bool sense_clks;
} t_sensor_bits;

#endif				/*CONFIG_MXC_PMIC_MC13783 */

#if defined(CONFIG_MXC_PMIC_MC13892_MODULE) || defined(CONFIG_MXC_PMIC_MC13892)
enum {
	REG_INT_STATUS0 = 0,
	REG_INT_MASK0,
	REG_INT_SENSE0,
	REG_INT_STATUS1,
	REG_INT_MASK1,
	REG_INT_SENSE1,
	REG_PU_MODE_S,
	REG_IDENTIFICATION,
	REG_UNUSED0,
	REG_ACC0,
	REG_ACC1,		/*10 */
	REG_UNUSED1,
	REG_UNUSED2,
	REG_POWER_CTL0,
	REG_POWER_CTL1,
	REG_POWER_CTL2,
	REG_REGEN_ASSIGN,
	REG_UNUSED3,
	REG_MEM_A,
	REG_MEM_B,
	REG_RTC_TIME,		/*20 */
	REG_RTC_ALARM,
	REG_RTC_DAY,
	REG_RTC_DAY_ALARM,
	REG_SW_0,
	REG_SW_1,
	REG_SW_2,
	REG_SW_3,
	REG_SW_4,
	REG_SW_5,
	REG_SETTING_0,		/*30 */
	REG_SETTING_1,
	REG_MODE_0,
	REG_MODE_1,
	REG_POWER_MISC,
	REG_UNUSED4,
	REG_UNUSED5,
	REG_UNUSED6,
	REG_UNUSED7,
	REG_UNUSED8,
	REG_UNUSED9,		/*40 */
	REG_UNUSED10,
	REG_UNUSED11,
	REG_ADC0,
	REG_ADC1,
	REG_ADC2,
	REG_ADC3,
	REG_ADC4,
	REG_CHARGE,
	REG_USB0,
	REG_USB1,		/*50 */
	REG_LED_CTL0,
	REG_LED_CTL1,
	REG_LED_CTL2,
	REG_LED_CTL3,
	REG_UNUSED12,
	REG_UNUSED13,
	REG_TRIM0,
	REG_TRIM1,
	REG_TEST0,
	REG_TEST1,		/*60 */
	REG_TEST2,
	REG_TEST3,
	REG_TEST4,
};

typedef enum {
	EVENT_ADCDONEI = 0,
	EVENT_ADCBISDONEI = 1,
	EVENT_TSI = 2,
	EVENT_VBUSVI = 3,
	EVENT_IDFACI = 4,
	EVENT_USBOVI = 5,
	EVENT_CHGDETI = 6,
	EVENT_CHGFAULTI = 7,
	EVENT_CHGREVI = 8,
	EVENT_CHGRSHORTI = 9,
	EVENT_CCCVI = 10,
	EVENT_CHGCURRI = 11,
	EVENT_BPONI = 12,
	EVENT_LOBATLI = 13,
	EVENT_LOBATHI = 14,
	EVENT_IDFLOATI = 19,
	EVENT_IDGNDI = 20,
	EVENT_SE1I = 21,
	EVENT_CKDETI = 22,
	EVENT_1HZI = 24,
	EVENT_TODAI = 25,
	EVENT_PWRONI = 27,
	EVENT_WDIRESETI = 29,
	EVENT_SYSRSTI = 30,
	EVENT_RTCRSTI = 31,
	EVENT_PCI = 32,
	EVENT_WARMI = 33,
	EVENT_MEMHLDI = 34,
	EVENT_THWARNLI = 36,
	EVENT_THWARNHI = 37,
	EVENT_CLKI = 38,
	EVENT_SCPI = 40,
	EVENT_LBPI = 44,
	EVENT_NB,
} type_event;

typedef enum {
	SENSE_VBUSVS = 3,
	SENSE_IDFACS = 4,
	SENSE_USBOVS = 5,
	SENSE_CHGDETS = 6,
	SENSE_CHGREVS = 8,
	SENSE_CHGRSHORTS = 9,
	SENSE_CCCVS = 10,
	SENSE_CHGCURRS = 11,
	SENSE_BPONS = 12,
	SENSE_LOBATLS = 13,
	SENSE_LOBATHS = 14,
	SENSE_IDFLOATS = 19,
	SENSE_IDGNDS = 20,
	SENSE_SE1S = 21,
	SENSE_PWRONS = 27,
	SENSE_THWARNLS = 36,
	SENSE_THWARNHS = 37,
	SENSE_CLKS = 38,
	SENSE_LBPS = 44,
	SENSE_NB,
} t_sensor;

typedef struct {
	bool sense_vbusvs;
	bool sense_idfacs;
	bool sense_usbovs;
	bool sense_chgdets;
	bool sense_chgrevs;
	bool sense_chgrshorts;
	bool sense_cccvs;
	bool sense_chgcurrs;
	bool sense_bpons;
	bool sense_lobatls;
	bool sense_lobaths;
	bool sense_idfloats;
	bool sense_idgnds;
	bool sense_se1s;
	bool sense_pwrons;
	bool sense_thwarnls;
	bool sense_thwarnhs;
	bool sense_clks;
	bool sense_lbps;
} t_sensor_bits;

extern struct i2c_client *mc13892_client;
int pmic_i2c_24bit_read(struct i2c_client *client, unsigned int reg_num,
			unsigned int *value);
int pmic_read(int reg_num, unsigned int *reg_val);
int pmic_write(int reg_num, const unsigned int reg_val);
void gpio_pmic_active(void);
void pmic_event_list_init(void);
void mc13892_power_off(void);

#endif

#if defined(CONFIG_MXC_PMIC_MC34704_MODULE) || defined(CONFIG_MXC_PMIC_MC34704)

typedef enum {
	/* register names for mc34704 */
	REG_MC34704_GENERAL1 = 0x01,
	REG_MC34704_GENERAL2 = 0x02,
	REG_MC34704_GENERAL3 = 0x03,
	REG_MC34704_VGSET1 = 0x04,
	REG_MC34704_VGSET2 = 0x05,
	REG_MC34704_REG2SET1 = 0x06,
	REG_MC34704_REG2SET2 = 0x07,
	REG_MC34704_REG3SET1 = 0x08,
	REG_MC34704_REG3SET2 = 0x09,
	REG_MC34704_REG4SET1 = 0x0A,
	REG_MC34704_REG4SET2 = 0x0B,
	REG_MC34704_REG5SET1 = 0x0C,
	REG_MC34704_REG5SET2 = 0x0D,
	REG_MC34704_REG5SET3 = 0x0E,
	REG_MC34704_REG6SET1 = 0x0F,
	REG_MC34704_REG6SET2 = 0x10,
	REG_MC34704_REG6SET3 = 0x11,
	REG_MC34704_REG7SET1 = 0x12,
	REG_MC34704_REG7SET2 = 0x13,
	REG_MC34704_REG7SET3 = 0x14,
	REG_MC34704_REG8SET1 = 0x15,
	REG_MC34704_REG8SET2 = 0x16,
	REG_MC34704_REG8SET3 = 0x17,
	REG_MC34704_FAULTS = 0x18,
	REG_MC34704_I2CSET1 = 0x19,
	REG_MC34704_REG3DAC = 0x49,
	REG_MC34704_REG7CR0 = 0x58,
	REG_MC34704_REG7DAC = 0x59,
	REG_NB = 0x60,
} pmic_reg;

typedef enum {
	/* events for mc34704 */
	EVENT_FLT1 = 0,
	EVENT_FLT2,
	EVENT_FLT3,
	EVENT_FLT4,
	EVENT_FLT5,
	EVENT_FLT6,
	EVENT_FLT7,
	EVENT_FLT8,
	EVENT_NB,
} type_event;

typedef enum {
	MCU_SENSOR_NOT_SUPPORT
} t_sensor;

typedef enum {
	MCU_SENSOR_BIT_NOT_SUPPORT
} t_sensor_bits;

#endif				/* MXC_PMIC_MC34704 */

/* EXPORTED FUNCTIONS */
#ifdef __KERNEL__

#if defined(CONFIG_MXC_PMIC)
/*!
 * This function is used to determine the PMIC type and its revision.
 *
 * @return      Returns the PMIC type and its revision.
 */
pmic_version_t pmic_get_version(void);

/*!
 * This function is called by PMIC clients to read a register on PMIC.
 *
 * @param        priority   priority of access
 * @param        reg        number of register
 * @param        reg_value   return value of register
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_read_reg(int reg, unsigned int *reg_value,
			  unsigned int reg_mask);
/*!
 * This function is called by PMIC clients to write a register on MC13783.
 *
 * @param        priority   priority of access
 * @param        reg        number of register
 * @param        reg_value  New value of register
 * @param        reg_mask   Bitmap mask indicating which bits to modify
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_write_reg(int reg, unsigned int reg_value,
			   unsigned int reg_mask);

/*!
 * This function is called by PMIC clients to subscribe on an event.
 *
 * @param        event_sub   structure of event, it contains type of event and callback
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_event_subscribe(type_event event,
				 pmic_event_callback_t callback);
/*!
* This function is called by PMIC clients to un-subscribe on an event.
*
* @param        event_unsub   structure of event, it contains type of event and callback
*
* @return       This function returns PMIC_SUCCESS if successful.
*/
PMIC_STATUS pmic_event_unsubscribe(type_event event,
				   pmic_event_callback_t callback);
/*!
* This function is called to read all sensor bits of PMIC.
*
* @param        sensor    Sensor to be checked.
*
* @return       This function returns true if the sensor bit is high;
*               or returns false if the sensor bit is low.
*/
bool pmic_check_sensor(t_sensor sensor);

/*!
* This function checks one sensor of PMIC.
*
* @param        sensor_bits  structure of all sensor bits.
*
* @return       This function returns PMIC_SUCCESS if successful.
*/
PMIC_STATUS pmic_get_sensors(t_sensor_bits * sensor_bits);

void pmic_event_callback(type_event event);
void pmic_event_list_init(void);

#endif				/*CONFIG_MXC_PMIC*/
#endif				/* __KERNEL__ */
/* CONFIG_MXC_PMIC_MC13783 || CONFIG_MXC_PMIC_MC9SDZ60 */

struct pmic_platform_data {
	int (*init)(void *);
	int power_key_irq;
};

#endif				/* __ASM_ARCH_MXC_PMIC_EXTERNAL_H__ */
