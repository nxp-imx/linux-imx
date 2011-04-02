/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
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
#include <linux/spi/spi.h>

/*!
 * This is the enumeration of versions of PMIC
 */
typedef enum {
	PMIC_MC13783 = 1,	/*!< MC13783 */
	PMIC_SC55112 = 2,	/*!< SC55112 */
	PMIC_MC13892 = 3,
	PMIC_MC34704 = 4,
	PMIC_MC34708 = 5
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


#if defined(CONFIG_MXC_PMIC_MC13892_MODULE) ||	\
	defined(CONFIG_MXC_PMIC_MC13892) ||	\
	defined(CONFIG_MXC_PMIC_MC34708_MODULE) || \
	defined(CONFIG_MXC_PMIC_MC34708)
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
	EVENT_PWRON3I = 26,
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

enum {
	REG_MC34708_INT_STATUS0 = 0,
	REG_MC34708_INT_MASK0,
	REG_MC34708_INT_SENSE0,
	REG_MC34708_INT_STATUS1,
	REG_MC34708_INT_MASK1,
	REG_MC34708_INT_SENSE1,
	REG_MC34708_PU_MODE_S,
	REG_MC34708_IDENTIFICATION,
	REG_MC34708_REGU_FAULT_S,
	REG_MC34708_ACC0,
	REG_MC34708_ACC1,		/*10 */
	REG_MC34708_UNUSED1,
	REG_MC34708_UNUSED2,
	REG_MC34708_POWER_CTL0,
	REG_MC34708_POWER_CTL1,
	REG_MC34708_POWER_CTL2,
	REG_MC34708_MEM_A,
	REG_MC34708_MEM_B,
	REG_MC34708_MEM_C,
	REG_MC34708_MEM_D,
	REG_MC34708_RTC_TIME,		/*20 */
	REG_MC34708_RTC_ALARM,
	REG_MC34708_RTC_DAY,
	REG_MC34708_RTC_DAY_ALARM,
	REG_MC34708_SW_1_A_B,
	REG_MC34708_SW_2_3,
	REG_MC34708_SW_4_A_B,
	REG_MC34708_SW_5,
	REG_MC34708_SW_1_2_OP,
	REG_MC34708_SW_3_4_5_OP,
	REG_MC34708_SETTING_0,		/*30 */
	REG_MC34708_SWBST,
	REG_MC34708_MODE_0,
	REG_MC34708_GPIOLV0,
	REG_MC34708_GPIOLV1,
	REG_MC34708_GPIOLV2,
	REG_MC34708_GPIOLV3,
	REG_MC34708_USB_TIMING,
	REG_MC34708_USB_BUTTON,
	REG_MC34708_USB_CONTROL,
	REG_MC34708_USB_DEVICE_TYPE, /*40 */
	REG_MC34708_UNUSED3,
	REG_MC34708_UNUSED4,
	REG_MC34708_ADC0,
	REG_MC34708_ADC1,
	REG_MC34708_ADC2,
	REG_MC34708_ADC3,
	REG_MC34708_ADC4,
	REG_MC34708_ADC5,
	REG_MC34708_ADC6,
	REG_MC34708_ADC7,		/*50 */
	REG_MC34708_BATTERY_PRO,
	REG_MC34708_CHARGER_DEBOUNCE,
	REG_MC34708_CHARGER_SOURCE,
	REG_MC34708_CHARGER_LED_CON,
	REG_MC34708_PWM_CON,
	REG_MC34708_UNUSED5,
	REG_MC34708_UNUSED6,
	REG_MC34708_UNUSED7,
	REG_MC34708_UNUSED8,
	REG_MC34708_UNUSED9,		/*60 */
	REG_MC34708_UNUSED10,
	REG_MC34708_UNUSED11,
	REG_MC34708_UNUSED12,
};

extern struct i2c_client *mc34708_client;
void mc34708_power_off(void);

#elif defined(CONFIG_MXC_PMIC_MC34704_MODULE) || defined(CONFIG_MXC_PMIC_MC34704)

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

#else
typedef int type_event;
typedef int t_sensor;
typedef int t_sensor_bits;

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
PMIC_STATUS pmic_get_sensors(t_sensor_bits *sensor_bits);

void pmic_event_callback(type_event event);
void pmic_event_list_init(void);

unsigned int pmic_get_active_events(unsigned int *active_events);
int pmic_event_mask(type_event event);
int pmic_event_unmask(type_event event);
int pmic_spi_setup(struct spi_device *spi);

#endif				/*CONFIG_MXC_PMIC*/
#endif				/* __KERNEL__ */
/* CONFIG_MXC_PMIC_MC13783 || CONFIG_MXC_PMIC_MC9SDZ60 */

struct pmic_platform_data {
	int (*init)(void *);
	int power_key_irq;
};

#endif				/* __ASM_ARCH_MXC_PMIC_EXTERNAL_H__ */
