/*
 *  Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "board-mx27ads.h"
#include "gpio_mux.h"
#include "crm_regs.h"

static int g_uart_activated[MXC_UART_NR] = { 0, 0, 0, 0, 0, 0 };

/*!
 * @file mach-mx27/mx27ads_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX27
 */

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
	if (port < 0 || port >= MXC_UART_NR) {
		pr_info("Wrong port number: %d\n", port);
		BUG();
	}

	if (g_uart_activated[port]) {
		pr_info("UART %d has been activated multi-times\n", port + 1);
		return;
	}
	g_uart_activated[port] = 1;

	switch (port) {
	case 0:
		gpio_request_mux(MX27_PIN_UART1_TXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART1_RXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART1_CTS, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART1_RTS, GPIO_MUX_PRIMARY);
		break;
	case 1:
		gpio_request_mux(MX27_PIN_UART2_TXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART2_RXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART2_CTS, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART2_RTS, GPIO_MUX_PRIMARY);
		break;
	case 2:
		gpio_request_mux(MX27_PIN_UART3_TXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART3_RXD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART3_CTS, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_UART3_RTS, GPIO_MUX_PRIMARY);

		/* Enable IRDA in CPLD */
		__raw_writew(PBC_BCTRL2_IRDA_EN, PBC_BCTRL2_CLEAR_REG);
		break;
	case 3:
		gpio_request_mux(MX27_PIN_USBH1_TXDM, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_USBH1_RXDP, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_USBH1_TXDP, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_USBH1_FS, GPIO_MUX_ALT);
		break;
	case 4:
		gpio_request_mux(MX27_PIN_CSI_D6, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_D7, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_VSYNC, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_HSYNC, GPIO_MUX_ALT);
		break;
	case 5:
		gpio_request_mux(MX27_PIN_CSI_D0, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_D1, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_D2, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_CSI_D3, GPIO_MUX_ALT);
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	if (port < 0 || port >= MXC_UART_NR) {
		pr_info("Wrong port number: %d\n", port);
		BUG();
	}

	if (g_uart_activated[port] == 0) {
		pr_info("UART %d has not been activated \n", port + 1);
		return;
	}
	g_uart_activated[port] = 0;

	switch (port) {
	case 0:
		gpio_free_mux(MX27_PIN_UART1_TXD);
		gpio_free_mux(MX27_PIN_UART1_RXD);
		gpio_free_mux(MX27_PIN_UART1_CTS);
		gpio_free_mux(MX27_PIN_UART1_RTS);
		break;
	case 1:
		gpio_free_mux(MX27_PIN_UART2_TXD);
		gpio_free_mux(MX27_PIN_UART2_RXD);
		gpio_free_mux(MX27_PIN_UART2_CTS);
		gpio_free_mux(MX27_PIN_UART2_RTS);
		break;
	case 2:
		gpio_free_mux(MX27_PIN_UART3_TXD);
		gpio_free_mux(MX27_PIN_UART3_RXD);
		gpio_free_mux(MX27_PIN_UART3_CTS);
		gpio_free_mux(MX27_PIN_UART3_RTS);

		/* Disable IRDA in CPLD */
		__raw_writew(PBC_BCTRL2_IRDA_EN, PBC_BCTRL2_SET_REG);
		break;
	case 3:
		gpio_free_mux(MX27_PIN_USBH1_TXDM);
		gpio_free_mux(MX27_PIN_USBH1_RXDP);
		gpio_free_mux(MX27_PIN_USBH1_TXDP);
		gpio_free_mux(MX27_PIN_USBH1_FS);
		break;
	case 4:
		gpio_free_mux(MX27_PIN_CSI_D6);
		gpio_free_mux(MX27_PIN_CSI_D7);
		gpio_free_mux(MX27_PIN_CSI_VSYNC);
		gpio_free_mux(MX27_PIN_CSI_HSYNC);
		break;
	case 5:
		gpio_free_mux(MX27_PIN_CSI_D0);
		gpio_free_mux(MX27_PIN_CSI_D1);
		gpio_free_mux(MX27_PIN_CSI_D2);
		gpio_free_mux(MX27_PIN_CSI_D3);
		break;
	default:
		break;
	}
}

void gpio_power_key_active(void)
{
}
EXPORT_SYMBOL(gpio_power_key_active);

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{
	return;
}

static int usbh1_hs_active;
/*!
 * Setup GPIO for USB, Total 34 signals
 * PIN Configuration for USBOTG:   High/Full speed OTG
 *	PE2,PE1,PE0,PE24,PE25 -- PRIMARY
	PC7 - PC13  -- PRIMARY
	PB23,PB24 -- PRIMARY

  * PIN Configuration for USBH2:    : High/Full/Low speed host
  *	PA0 - PA4 -- PRIMARY
       PD19, PD20,PD21,PD22,PD23,PD24,PD26 --Alternate (SECONDARY)

  * PIN Configuration for USBH1:  Full/low speed host
  *  PB25 - PB31  -- PRIMARY
      PB22  -- PRIMARY
 */
int gpio_usbh1_active(void)
{
	if (usbh1_hs_active)
		return 0;

	if (gpio_request_mux(MX27_PIN_USBH1_SUSP, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_RCV, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_FS, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_OE_B, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_TXDM, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_TXDP, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_RXDM, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH1_RXDP, GPIO_MUX_PRIMARY))
		return -EINVAL;

	__raw_writew(PBC_BCTRL3_FSH_MOD, PBC_BCTRL3_CLEAR_REG);
	__raw_writew(PBC_BCTRL3_FSH_VBUS_EN, PBC_BCTRL3_CLEAR_REG);
	usbh1_hs_active = 1;
	return 0;
}
void gpio_usbh1_inactive(void)
{
	if (usbh1_hs_active == 0)
		return;

	gpio_free_mux(MX27_PIN_USBH1_SUSP);
	gpio_free_mux(MX27_PIN_USBH1_RCV);
	gpio_free_mux(MX27_PIN_USBH1_FS);
	gpio_free_mux(MX27_PIN_USBH1_OE_B);
	gpio_free_mux(MX27_PIN_USBH1_TXDM);
	gpio_free_mux(MX27_PIN_USBH1_TXDP);
	gpio_free_mux(MX27_PIN_USBH1_RXDM);
	gpio_free_mux(MX27_PIN_USBH1_RXDP);
	__raw_writew(PBC_BCTRL3_FSH_VBUS_EN, PBC_BCTRL3_SET_REG);

	usbh1_hs_active = 0;
}

static int usbh2_hs_active;
/*
 * conflicts with CSPI1 (MC13783) and CSPI2 (Connector)
 */
int gpio_usbh2_active(void)
{
	if (usbh2_hs_active)
		return 0;

	if (gpio_set_puen(MX27_PIN_USBH2_CLK, 0) ||
	    gpio_set_puen(MX27_PIN_USBH2_DIR, 0) ||
	    gpio_set_puen(MX27_PIN_USBH2_DATA7, 0) ||
	    gpio_set_puen(MX27_PIN_USBH2_NXT, 0) ||
	    gpio_set_puen(MX27_PIN_USBH2_STP, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_SS2, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_SS1, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_SS0, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_SCLK, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_MISO, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI2_MOSI, 0) ||
	    gpio_set_puen(MX27_PIN_CSPI1_SS2, 0))
		return -EINVAL;

	if (gpio_request_mux(MX27_PIN_USBH2_CLK, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH2_DIR, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH2_DATA7, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH2_NXT, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBH2_STP, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_CSPI2_SS2, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI2_SS1, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI2_SS0, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI2_SCLK, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI2_MISO, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI2_MOSI, GPIO_MUX_ALT) ||
	    gpio_request_mux(MX27_PIN_CSPI1_SS2, GPIO_MUX_ALT))
		return -EINVAL;

	__raw_writew(PBC_BCTRL3_HSH_EN, PBC_BCTRL3_CLEAR_REG);
	usbh2_hs_active = 1;
	return 0;
}
void gpio_usbh2_inactive(void)
{
	if (usbh2_hs_active == 0)
		return;

	gpio_free_mux(MX27_PIN_USBH2_CLK);
	gpio_free_mux(MX27_PIN_USBH2_DIR);
	gpio_free_mux(MX27_PIN_USBH2_DATA7);
	gpio_free_mux(MX27_PIN_USBH2_NXT);
	gpio_free_mux(MX27_PIN_USBH2_STP);

	gpio_free_mux(MX27_PIN_CSPI2_SS2);
	gpio_free_mux(MX27_PIN_CSPI2_SS1);
	gpio_free_mux(MX27_PIN_CSPI2_SS0);
	gpio_free_mux(MX27_PIN_CSPI2_SCLK);
	gpio_free_mux(MX27_PIN_CSPI2_MISO);
	gpio_free_mux(MX27_PIN_CSPI2_MOSI);
	gpio_free_mux(MX27_PIN_CSPI1_SS2);

	gpio_set_puen(MX27_PIN_USBH2_CLK, 1);
	gpio_set_puen(MX27_PIN_USBH2_DIR, 1);
	gpio_set_puen(MX27_PIN_USBH2_DATA7, 1);
	gpio_set_puen(MX27_PIN_USBH2_NXT, 1);
	gpio_set_puen(MX27_PIN_USBH2_STP, 1);
	gpio_set_puen(MX27_PIN_CSPI2_SS2, 1);
	gpio_set_puen(MX27_PIN_CSPI2_SS1, 1);
	gpio_set_puen(MX27_PIN_CSPI2_SS0, 1);
	gpio_set_puen(MX27_PIN_CSPI2_SCLK, 1);
	gpio_set_puen(MX27_PIN_CSPI2_MISO, 1);
	gpio_set_puen(MX27_PIN_CSPI2_MOSI, 1);
	gpio_set_puen(MX27_PIN_CSPI1_SS2, 1);
	__raw_writew(PBC_BCTRL3_HSH_EN, PBC_BCTRL3_SET_REG);

	usbh2_hs_active = 0;
}

static int usbotg_hs_active;
int gpio_usbotg_hs_active(void)
{
	if (usbotg_hs_active)
		return 0;

	if (gpio_request_mux(MX27_PIN_USBOTG_DATA5, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA6, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA0, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA2, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA1, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA3, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA4, GPIO_MUX_PRIMARY) ||

	    gpio_request_mux(MX27_PIN_USBOTG_DIR, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_STP, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_NXT, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_CLK, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USBOTG_DATA7, GPIO_MUX_PRIMARY) ||

	    gpio_request_mux(MX27_PIN_USB_OC_B, GPIO_MUX_PRIMARY) ||
	    gpio_request_mux(MX27_PIN_USB_PWR, GPIO_MUX_PRIMARY))
		return -EINVAL;

	__raw_writew(PBC_BCTRL3_OTG_HS_EN, PBC_BCTRL3_CLEAR_REG);
	__raw_writew(PBC_BCTRL3_OTG_VBUS_EN, PBC_BCTRL3_CLEAR_REG);

	usbotg_hs_active = 1;
	return 0;
}

void gpio_usbotg_hs_inactive(void)
{
	if (usbotg_hs_active == 0)
		return;

	gpio_free_mux(MX27_PIN_USBOTG_DATA5);
	gpio_free_mux(MX27_PIN_USBOTG_DATA6);
	gpio_free_mux(MX27_PIN_USBOTG_DATA0);
	gpio_free_mux(MX27_PIN_USBOTG_DATA2);
	gpio_free_mux(MX27_PIN_USBOTG_DATA1);
	gpio_free_mux(MX27_PIN_USBOTG_DATA3);
	gpio_free_mux(MX27_PIN_USBOTG_DATA4);

	gpio_free_mux(MX27_PIN_USBOTG_DIR);
	gpio_free_mux(MX27_PIN_USBOTG_STP);
	gpio_free_mux(MX27_PIN_USBOTG_NXT);
	gpio_free_mux(MX27_PIN_USBOTG_CLK);
	gpio_free_mux(MX27_PIN_USBOTG_DATA7);

	gpio_free_mux(MX27_PIN_USB_OC_B);
	gpio_free_mux(MX27_PIN_USB_PWR);
	__raw_writew(PBC_BCTRL3_OTG_HS_EN, PBC_BCTRL3_SET_REG);

	usbotg_hs_active = 0;
}

int gpio_usbotg_fs_active(void)
{
	return gpio_usbotg_hs_active();
}

void gpio_usbotg_fs_inactive(void)
{
	gpio_usbotg_hs_inactive();
}

/*!
 * end Setup GPIO for USB
 *
 */

/************************************************************************/
/* for i2c gpio                                                         */
/* I2C1:  PD17,PD18 -- Primary 					*/
/* I2C2:  PC5,PC6    -- Primary					*/
/************************************************************************/
/*!
* Setup GPIO for an I2C device to be active
*
* @param  i2c_num         an I2C device
*/
void gpio_i2c_active(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		gpio_request_mux(MX27_PIN_I2C_CLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_I2C_DATA, GPIO_MUX_PRIMARY);
		break;
	case 1:
		gpio_request_mux(MX27_PIN_I2C2_SCL, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_I2C2_SDA, GPIO_MUX_PRIMARY);
		break;
	default:
		printk(KERN_ERR "gpio_i2c_active no compatible I2C adapter\n");
		break;
	}
}

/*!
 *  * Setup GPIO for an I2C device to be inactive
 *   *
 *    * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		gpio_free_mux(MX27_PIN_I2C_CLK);
		gpio_free_mux(MX27_PIN_I2C_DATA);
		break;
	case 1:
		gpio_free_mux(MX27_PIN_I2C2_SCL);
		gpio_free_mux(MX27_PIN_I2C2_SDA);
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		gpio_request_mux(MX27_PIN_CSPI1_MOSI, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_MISO, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_SCLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_RDY, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_SS0, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_SS1, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI1_SS2, GPIO_MUX_PRIMARY);
		break;
	case 1:
		/*SPI2  */
		gpio_request_mux(MX27_PIN_CSPI2_MOSI, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI2_MISO, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI2_SCLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI2_SS0, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI2_SS1, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_CSPI2_SS2, GPIO_MUX_PRIMARY);
		break;
	case 2:
		/*SPI3  */
		gpio_request_mux(MX27_PIN_SD1_D0, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_SD1_CMD, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_SD1_CLK, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_SD1_D3, GPIO_MUX_ALT);
		break;

	default:
		break;
	}
}

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		gpio_free_mux(MX27_PIN_CSPI1_MOSI);
		gpio_free_mux(MX27_PIN_CSPI1_MISO);
		gpio_free_mux(MX27_PIN_CSPI1_SCLK);
		gpio_free_mux(MX27_PIN_CSPI1_RDY);
		gpio_free_mux(MX27_PIN_CSPI1_SS0);
		gpio_free_mux(MX27_PIN_CSPI1_SS1);
		gpio_free_mux(MX27_PIN_CSPI1_SS2);
		break;
	case 1:
		/*SPI2  */
		gpio_free_mux(MX27_PIN_CSPI2_MOSI);
		gpio_free_mux(MX27_PIN_CSPI2_MISO);
		gpio_free_mux(MX27_PIN_CSPI2_SCLK);
		gpio_free_mux(MX27_PIN_CSPI2_SS0);
		gpio_free_mux(MX27_PIN_CSPI2_SS1);
		gpio_free_mux(MX27_PIN_CSPI2_SS2);
		break;
	case 2:
		/*SPI3  */
		gpio_free_mux(MX27_PIN_SD1_D0);
		gpio_free_mux(MX27_PIN_SD1_CMD);
		gpio_free_mux(MX27_PIN_SD1_CLK);
		gpio_free_mux(MX27_PIN_SD1_D3);
		break;

	default:
		break;
	}
}

/*!
 * Setup GPIO for a nand flash device to be active
 *
 */
void gpio_nand_active(void)
{
	unsigned long reg;
	reg = __raw_readl(IO_ADDRESS(SYSCTRL_BASE_ADDR) + SYS_FMCR);
	reg &= ~(1 << 4);
	__raw_writel(reg, IO_ADDRESS(SYSCTRL_BASE_ADDR) + SYS_FMCR);

	gpio_request_mux(MX27_PIN_NFRB, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFCE_B, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFWP_B, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFCLE, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFALE, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFRE_B, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_NFWE_B, GPIO_MUX_PRIMARY);
}

/*!
 * Setup GPIO for a nand flash device to be inactive
 *
 */
void gpio_nand_inactive(void)
{
	gpio_free_mux(MX27_PIN_NFRB);
	gpio_free_mux(MX27_PIN_NFCE_B);
	gpio_free_mux(MX27_PIN_NFWP_B);
	gpio_free_mux(MX27_PIN_NFCLE);
	gpio_free_mux(MX27_PIN_NFALE);
	gpio_free_mux(MX27_PIN_NFRE_B);
	gpio_free_mux(MX27_PIN_NFWE_B);
}

/*!
 * Setup GPIO for CSI device to be active
 *
 */
void gpio_sensor_active(void)
{
	gpio_request_mux(MX27_PIN_CSI_D0, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D1, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D2, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D3, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D4, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_MCLK, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_PIXCLK, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D5, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D6, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_D7, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_VSYNC, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CSI_HSYNC, GPIO_MUX_PRIMARY);

#ifdef CONFIG_MXC_CAMERA_MC521DA
	__raw_writew(0x100, PBC_BCTRL2_SET_REG);
#else
	__raw_writew(0x400, PBC_BCTRL2_SET_REG);
#endif
}

void gpio_sensor_inactive(void)
{
	gpio_free_mux(MX27_PIN_CSI_D0);
	gpio_free_mux(MX27_PIN_CSI_D1);
	gpio_free_mux(MX27_PIN_CSI_D2);
	gpio_free_mux(MX27_PIN_CSI_D3);
	gpio_free_mux(MX27_PIN_CSI_D4);
	gpio_free_mux(MX27_PIN_CSI_MCLK);
	gpio_free_mux(MX27_PIN_CSI_PIXCLK);
	gpio_free_mux(MX27_PIN_CSI_D5);
	gpio_free_mux(MX27_PIN_CSI_D6);
	gpio_free_mux(MX27_PIN_CSI_D7);
	gpio_free_mux(MX27_PIN_CSI_VSYNC);
	gpio_free_mux(MX27_PIN_CSI_HSYNC);

#ifdef CONFIG_MXC_CAMERA_MC521DA
	__raw_writew(0x100, PBC_BCTRL2_CLEAR_REG);
#else
	__raw_writew(0x400, PBC_BCTRL2_CLEAR_REG);
#endif
}

void gpio_sensor_reset(bool flag)
{
	u16 temp;

	if (flag) {
		temp = 0x200;
		__raw_writew(temp, PBC_BCTRL2_CLEAR_REG);
	} else {
		temp = 0x200;
		__raw_writew(temp, PBC_BCTRL2_SET_REG);
	}
}

/*!
 * Setup GPIO for LCDC device to be active
 *
 */
void gpio_lcdc_active(void)
{
	gpio_request_mux(MX27_PIN_LSCLK, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD0, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD1, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD2, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD3, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD4, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD5, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD6, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD7, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD8, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD9, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD10, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD11, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD12, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD13, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD14, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD15, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD16, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_LD17, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_REV, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CLS, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_PS, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_SPL_SPR, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_HSYNC, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_VSYNC, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_CONTRAST, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_OE_ACD, GPIO_MUX_PRIMARY);
}

/*!
 * Setup GPIO for LCDC device to be inactive
 *
 */
void gpio_lcdc_inactive(void)
{
	gpio_free_mux(MX27_PIN_LSCLK);
	gpio_free_mux(MX27_PIN_LD0);
	gpio_free_mux(MX27_PIN_LD1);
	gpio_free_mux(MX27_PIN_LD2);
	gpio_free_mux(MX27_PIN_LD3);
	gpio_free_mux(MX27_PIN_LD4);
	gpio_free_mux(MX27_PIN_LD5);
	gpio_free_mux(MX27_PIN_LD6);
	gpio_free_mux(MX27_PIN_LD7);
	gpio_free_mux(MX27_PIN_LD8);
	gpio_free_mux(MX27_PIN_LD9);
	gpio_free_mux(MX27_PIN_LD10);
	gpio_free_mux(MX27_PIN_LD11);
	gpio_free_mux(MX27_PIN_LD12);
	gpio_free_mux(MX27_PIN_LD13);
	gpio_free_mux(MX27_PIN_LD14);
	gpio_free_mux(MX27_PIN_LD15);
	gpio_free_mux(MX27_PIN_LD16);
	gpio_free_mux(MX27_PIN_LD17);
	gpio_free_mux(MX27_PIN_REV);
	gpio_free_mux(MX27_PIN_CLS);
	gpio_free_mux(MX27_PIN_PS);
	gpio_free_mux(MX27_PIN_SPL_SPR);
	gpio_free_mux(MX27_PIN_HSYNC);
	gpio_free_mux(MX27_PIN_VSYNC);
	gpio_free_mux(MX27_PIN_CONTRAST);
	gpio_free_mux(MX27_PIN_OE_ACD);
}

/*!
 * Setup GPIO PA25 low to start hard reset FS453 TV encoder
 *
 */
void gpio_fs453_reset_low(void)
{
	gpio_free_mux(MX27_PIN_CLS);
	if (gpio_request_mux(MX27_PIN_CLS, GPIO_MUX_GPIO)) {
		printk(KERN_ERR "bug: request GPIO PA25 failed.\n");
		return;
	}

	/* PA25 (CLS) as output */
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_CLS), "cls");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_CLS), 0);
	gpio_config_mux(MX27_PIN_CLS, GPIO_MUX_GPIO);
	gpio_set_value(IOMUX_TO_GPIO(MX27_PIN_CLS), 0);
}

/*!
 * Setup GPIO PA25 high to end hard reset FS453 TV encoder
 *
 */
void gpio_fs453_reset_high(void)
{
	gpio_free_mux(MX27_PIN_CLS);
	if (gpio_request_mux(MX27_PIN_CLS, GPIO_MUX_GPIO)) {
		printk(KERN_ERR "bug: request GPIO PA25 failed.\n");
		return;
	}

	/* PA25 (CLS) as output */
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_CLS), "cls");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_CLS), 0);
	gpio_config_mux(MX27_PIN_CLS, GPIO_MUX_GPIO);
	gpio_set_value(IOMUX_TO_GPIO(MX27_PIN_CLS), 1);
}

/*!
 * This function configures the IOMux block for PMIC standard operations.
 *
 */
void gpio_pmic_active(void)
{
	gpio_config_mux(MX27_PIN_TOUT, GPIO_MUX_GPIO);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_TOUT), "tout");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_TOUT));
}

/*!
 * GPIO settings not required for keypad
 *
 */
void gpio_keypad_active(void)
{
}

/*!
 * GPIO settings not required for keypad
 *
 */
void gpio_keypad_inactive(void)
{
}

/*!
 * Setup GPIO for ATA device to be active
 *
 */
void gpio_ata_active(void)
{
	gpio_request_mux(MX27_PIN_ATA_DATA0, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA1, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA2, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA3, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA4, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA5, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA6, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA7, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA8, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA9, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA10, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA11, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA12, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA13, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA14, GPIO_MUX_PRIMARY);
	gpio_request_mux(MX27_PIN_ATA_DATA15, GPIO_MUX_PRIMARY);

	gpio_request_mux(MX27_PIN_PC_CD1_B, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_CD2_B, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_WAIT_B, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_READY, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_PWRON, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_VS1, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_VS2, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_BVD1, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_BVD2, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_RST, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_IOIS16, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_RW_B, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_PC_POE, GPIO_MUX_ALT);

	__raw_writew(PBC_BCTRL2_ATAFEC_EN | PBC_BCTRL2_ATAFEC_SEL |
		     PBC_BCTRL2_ATA_EN, PBC_BCTRL2_CLEAR_REG);
}

/*!
 * Setup GPIO for ATA device to be inactive
 *
 */
void gpio_ata_inactive(void)
{
	__raw_writew(PBC_BCTRL2_ATAFEC_EN | PBC_BCTRL2_ATAFEC_SEL |
		     PBC_BCTRL2_ATA_EN, PBC_BCTRL2_SET_REG);

	gpio_free_mux(MX27_PIN_ATA_DATA0);
	gpio_free_mux(MX27_PIN_ATA_DATA1);
	gpio_free_mux(MX27_PIN_ATA_DATA2);
	gpio_free_mux(MX27_PIN_ATA_DATA3);
	gpio_free_mux(MX27_PIN_ATA_DATA4);
	gpio_free_mux(MX27_PIN_ATA_DATA5);
	gpio_free_mux(MX27_PIN_ATA_DATA6);
	gpio_free_mux(MX27_PIN_ATA_DATA7);
	gpio_free_mux(MX27_PIN_ATA_DATA8);
	gpio_free_mux(MX27_PIN_ATA_DATA9);
	gpio_free_mux(MX27_PIN_ATA_DATA10);
	gpio_free_mux(MX27_PIN_ATA_DATA11);
	gpio_free_mux(MX27_PIN_ATA_DATA12);
	gpio_free_mux(MX27_PIN_ATA_DATA13);
	gpio_free_mux(MX27_PIN_ATA_DATA14);
	gpio_free_mux(MX27_PIN_ATA_DATA15);

	gpio_free_mux(MX27_PIN_PC_CD1_B);
	gpio_free_mux(MX27_PIN_PC_CD2_B);
	gpio_free_mux(MX27_PIN_PC_WAIT_B);
	gpio_free_mux(MX27_PIN_PC_READY);
	gpio_free_mux(MX27_PIN_PC_PWRON);
	gpio_free_mux(MX27_PIN_PC_VS1);
	gpio_free_mux(MX27_PIN_PC_VS2);
	gpio_free_mux(MX27_PIN_PC_BVD1);
	gpio_free_mux(MX27_PIN_PC_BVD2);
	gpio_free_mux(MX27_PIN_PC_RST);
	gpio_free_mux(MX27_PIN_IOIS16);
	gpio_free_mux(MX27_PIN_PC_RW_B);
	gpio_free_mux(MX27_PIN_PC_POE);
}

/*!
 * Setup GPIO for FEC device to be active
 *
 */
void gpio_fec_active(void)
{
	gpio_request_mux(MX27_PIN_ATA_DATA15, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA15), "ata_data15");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA15), 0);
	gpio_request_mux(MX27_PIN_ATA_DATA14, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA14), "ata_data14");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA14), 0);
	gpio_request_mux(MX27_PIN_ATA_DATA13, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA13), "ata_data13");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA13));
	gpio_request_mux(MX27_PIN_ATA_DATA12, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA12), "ata_data12");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA12));
	gpio_request_mux(MX27_PIN_ATA_DATA11, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA11), "ata_data11");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA11));
	gpio_request_mux(MX27_PIN_ATA_DATA10, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA10), "ata_data10");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA10));
	gpio_request_mux(MX27_PIN_ATA_DATA9, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA9), "ata_data9");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA9));
	gpio_request_mux(MX27_PIN_ATA_DATA8, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA8), "ata_data8");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA8));
	gpio_request_mux(MX27_PIN_ATA_DATA7, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA7), "ata_data7");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA7), 0);

	gpio_request_mux(MX27_PIN_ATA_DATA6, GPIO_MUX_ALT);
	gpio_request_mux(MX27_PIN_ATA_DATA5, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA5), "ata_data5");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA5));
	gpio_request_mux(MX27_PIN_ATA_DATA4, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA4), "ata_data4");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA4));
	gpio_request_mux(MX27_PIN_ATA_DATA3, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA3), "ata_data3");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA3));
	gpio_request_mux(MX27_PIN_ATA_DATA2, GPIO_MUX_INPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA2), "ata_data2");
	gpio_direction_input(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA2));
	gpio_request_mux(MX27_PIN_ATA_DATA1, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA1), "ata_data1");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA1), 0);
	gpio_request_mux(MX27_PIN_ATA_DATA0, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA0), "ata_data0");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_ATA_DATA0), 0);
	gpio_request_mux(MX27_PIN_SD3_CLK, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_SD3_CLK), "sd3_clk");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_SD3_CLK), 0);
	gpio_request_mux(MX27_PIN_SD3_CMD, GPIO_MUX_OUTPUT1);
	gpio_request(IOMUX_TO_GPIO(MX27_PIN_SD3_CMD), "sd3_cmd");
	gpio_direction_output(IOMUX_TO_GPIO(MX27_PIN_SD3_CMD), 0);

	__raw_writew(PBC_BCTRL2_ATAFEC_EN, PBC_BCTRL2_CLEAR_REG);
	__raw_writew(PBC_BCTRL2_ATAFEC_SEL, PBC_BCTRL2_SET_REG);
}

/*!
 * Setup GPIO for FEC device to be inactive
 *
 */
void gpio_fec_inactive(void)
{
	gpio_free(MX27_PIN_ATA_DATA0);
	gpio_free(MX27_PIN_ATA_DATA1);
	gpio_free(MX27_PIN_ATA_DATA2);
	gpio_free(MX27_PIN_ATA_DATA3);
	gpio_free(MX27_PIN_ATA_DATA4);
	gpio_free(MX27_PIN_ATA_DATA5);
	gpio_free(MX27_PIN_ATA_DATA6);
	gpio_free(MX27_PIN_ATA_DATA7);
	gpio_free(MX27_PIN_ATA_DATA8);
	gpio_free(MX27_PIN_ATA_DATA9);
	gpio_free(MX27_PIN_ATA_DATA10);
	gpio_free(MX27_PIN_ATA_DATA11);
	gpio_free(MX27_PIN_ATA_DATA12);
	gpio_free(MX27_PIN_ATA_DATA13);
	gpio_free(MX27_PIN_ATA_DATA14);
	gpio_free(MX27_PIN_ATA_DATA15);

	gpio_free(MX27_PIN_SD3_CMD);
	gpio_free(MX27_PIN_SD3_CLK);

	gpio_free_mux(MX27_PIN_ATA_DATA0);
	gpio_free_mux(MX27_PIN_ATA_DATA1);
	gpio_free_mux(MX27_PIN_ATA_DATA2);
	gpio_free_mux(MX27_PIN_ATA_DATA3);
	gpio_free_mux(MX27_PIN_ATA_DATA4);
	gpio_free_mux(MX27_PIN_ATA_DATA5);
	gpio_free_mux(MX27_PIN_ATA_DATA6);
	gpio_free_mux(MX27_PIN_ATA_DATA7);
	gpio_free_mux(MX27_PIN_ATA_DATA8);
	gpio_free_mux(MX27_PIN_ATA_DATA9);
	gpio_free_mux(MX27_PIN_ATA_DATA10);
	gpio_free_mux(MX27_PIN_ATA_DATA11);
	gpio_free_mux(MX27_PIN_ATA_DATA12);
	gpio_free_mux(MX27_PIN_ATA_DATA13);
	gpio_free_mux(MX27_PIN_ATA_DATA14);
	gpio_free_mux(MX27_PIN_ATA_DATA15);

	gpio_free_mux(MX27_PIN_SD3_CMD);
	gpio_free_mux(MX27_PIN_SD3_CLK);
}

/*!
 * Setup GPIO for SLCDC device to be active
 *
 */
void gpio_slcdc_active(int type)
{
	switch (type) {
	case 0:
		gpio_request_mux(MX27_PIN_SSI3_CLK, GPIO_MUX_ALT);	/* CLK */
		gpio_request_mux(MX27_PIN_SSI3_TXDAT, GPIO_MUX_ALT);	/* CS  */
		gpio_request_mux(MX27_PIN_SSI3_RXDAT, GPIO_MUX_ALT);	/* RS  */
		gpio_request_mux(MX27_PIN_SSI3_FS, GPIO_MUX_ALT);	/* D0  */
		break;

	case 1:
		gpio_request_mux(MX27_PIN_SD2_D1, GPIO_MUX_GPIO);	/* CLK */
		gpio_request_mux(MX27_PIN_SD2_D2, GPIO_MUX_GPIO);	/* D0  */
		gpio_request_mux(MX27_PIN_SD2_D3, GPIO_MUX_GPIO);	/* RS  */
		gpio_request_mux(MX27_PIN_SD2_CMD, GPIO_MUX_GPIO);	/* CS  */
		break;

	case 2:
		gpio_request_mux(MX27_PIN_LD0, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD1, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD2, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD3, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD4, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD5, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD6, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD7, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD8, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD9, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD10, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD11, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD12, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD13, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD14, GPIO_MUX_GPIO);
		gpio_request_mux(MX27_PIN_LD15, GPIO_MUX_GPIO);
		break;

	default:
		break;
	}

	return;
}

/*!
 * Setup GPIO for SLCDC device to be inactive
 *
 */
void gpio_slcdc_inactive(int type)
{
	switch (type) {
	case 0:
		gpio_free_mux(MX27_PIN_SSI3_CLK);	/* CLK */
		gpio_free_mux(MX27_PIN_SSI3_TXDAT);	/* CS  */
		gpio_free_mux(MX27_PIN_SSI3_RXDAT);	/* RS  */
		gpio_free_mux(MX27_PIN_SSI3_FS);	/* D0  */
		break;

	case 1:
		gpio_free_mux(MX27_PIN_SD2_D1);	/* CLK */
		gpio_free_mux(MX27_PIN_SD2_D2);	/* D0  */
		gpio_free_mux(MX27_PIN_SD2_D3);	/* RS  */
		gpio_free_mux(MX27_PIN_SD2_CMD);	/* CS  */
		break;

	case 2:
		gpio_free_mux(MX27_PIN_LD0);
		gpio_free_mux(MX27_PIN_LD1);
		gpio_free_mux(MX27_PIN_LD2);
		gpio_free_mux(MX27_PIN_LD3);
		gpio_free_mux(MX27_PIN_LD4);
		gpio_free_mux(MX27_PIN_LD5);
		gpio_free_mux(MX27_PIN_LD6);
		gpio_free_mux(MX27_PIN_LD7);
		gpio_free_mux(MX27_PIN_LD8);
		gpio_free_mux(MX27_PIN_LD9);
		gpio_free_mux(MX27_PIN_LD10);
		gpio_free_mux(MX27_PIN_LD11);
		gpio_free_mux(MX27_PIN_LD12);
		gpio_free_mux(MX27_PIN_LD13);
		gpio_free_mux(MX27_PIN_LD14);
		gpio_free_mux(MX27_PIN_LD15);
		break;

	default:
		break;
	}

	return;
}

void gpio_ssi_active(int ssi_num)
{
	switch (ssi_num) {
	case 0:
		gpio_request_mux(MX27_PIN_SSI1_FS, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI1_RXDAT, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI1_TXDAT, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI1_CLK, GPIO_MUX_PRIMARY);
		gpio_set_puen(MX27_PIN_SSI1_FS, 0);
		gpio_set_puen(MX27_PIN_SSI1_RXDAT, 0);
		gpio_set_puen(MX27_PIN_SSI1_TXDAT, 0);
		gpio_set_puen(MX27_PIN_SSI1_CLK, 0);
		break;
	case 1:
		gpio_request_mux(MX27_PIN_SSI2_FS, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI2_RXDAT, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI2_TXDAT, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SSI2_CLK, GPIO_MUX_PRIMARY);
		gpio_set_puen(MX27_PIN_SSI2_FS, 0);
		gpio_set_puen(MX27_PIN_SSI2_RXDAT, 0);
		gpio_set_puen(MX27_PIN_SSI2_TXDAT, 0);
		gpio_set_puen(MX27_PIN_SSI2_CLK, 0);
		break;
	default:
		break;
	}
	return;
}

/*!
 *  * Setup GPIO for a SSI port to be inactive
 *   *
 *    * @param  ssi_num         an SSI port num
 */

void gpio_ssi_inactive(int ssi_num)
{
	switch (ssi_num) {
	case 0:
		gpio_free_mux(MX27_PIN_SSI1_FS);
		gpio_free_mux(MX27_PIN_SSI1_RXDAT);
		gpio_free_mux(MX27_PIN_SSI1_TXDAT);
		gpio_free_mux(MX27_PIN_SSI1_CLK);
		break;
	case 1:
		gpio_free_mux(MX27_PIN_SSI2_FS);
		gpio_free_mux(MX27_PIN_SSI2_RXDAT);
		gpio_free_mux(MX27_PIN_SSI2_TXDAT);
		gpio_free_mux(MX27_PIN_SSI2_CLK);
		break;
	default:
		break;
	}
	return;
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	u16 data;
	switch (module) {
	case 0:
		gpio_request_mux(MX27_PIN_SD1_CLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD1_CMD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD1_D0, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD1_D1, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD1_D2, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD1_D3, GPIO_MUX_PRIMARY);
		/* 22k pull up for sd1 dat3 pins */
		data = __raw_readw(IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		data |= 0x0c;
		__raw_writew(data, IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		/*mxc_clks_enable(SDHC1_CLK);
		   mxc_clks_enable(PERCLK2); */
		break;
	case 1:
		gpio_request_mux(MX27_PIN_SD2_CLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD2_CMD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD2_D0, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD2_D1, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD2_D2, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD2_D3, GPIO_MUX_PRIMARY);
		/* 22k pull up for sd2 pins */
		data = __raw_readw(IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		data &= ~0xfff0;
		data |= 0xfff0;
		__raw_writew(data, IO_ADDRESS(SYSCTRL_BASE_ADDR + 0x54));
		/*mxc_clks_enable(SDHC2_CLK);
		   mxc_clks_enable(PERCLK2); */
		break;
	case 2:
		gpio_request_mux(MX27_PIN_SD3_CLK, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_SD3_CMD, GPIO_MUX_PRIMARY);
		gpio_request_mux(MX27_PIN_ATA_DATA0, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_ATA_DATA1, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_ATA_DATA2, GPIO_MUX_ALT);
		gpio_request_mux(MX27_PIN_ATA_DATA3, GPIO_MUX_ALT);
		/*mxc_clks_enable(SDHC3_CLK);
		   mxc_clks_enable(PERCLK2); */
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
		gpio_free_mux(MX27_PIN_SD1_CLK);
		gpio_free_mux(MX27_PIN_SD1_CMD);
		gpio_free_mux(MX27_PIN_SD1_D0);
		gpio_free_mux(MX27_PIN_SD1_D1);
		gpio_free_mux(MX27_PIN_SD1_D2);
		gpio_free_mux(MX27_PIN_SD1_D3);
		/*mxc_clks_disable(SDHC1_CLK); */
		break;
	case 1:
		gpio_free_mux(MX27_PIN_SD2_CLK);
		gpio_free_mux(MX27_PIN_SD2_CMD);
		gpio_free_mux(MX27_PIN_SD2_D0);
		gpio_free_mux(MX27_PIN_SD2_D1);
		gpio_free_mux(MX27_PIN_SD2_D2);
		gpio_free_mux(MX27_PIN_SD2_D3);
		/*mxc_clks_disable(SDHC2_CLK); */
		break;
	case 2:
		gpio_free_mux(MX27_PIN_SD3_CLK);
		gpio_free_mux(MX27_PIN_SD3_CMD);
		gpio_free_mux(MX27_PIN_ATA_DATA0);
		gpio_free_mux(MX27_PIN_ATA_DATA1);
		gpio_free_mux(MX27_PIN_ATA_DATA2);
		gpio_free_mux(MX27_PIN_ATA_DATA3);
		/*mxc_clks_disable(SDHC3_CLK); */
		break;
	default:
		break;
	}
}

/*
 * Probe for the card. If present the GPIO data would be set.
 */
int sdhc_get_card_det_status(struct device *dev)
{
	return 0;
}

/*
 * Return the card detect pin.
 */
int sdhc_init_card_det(int id)
{
	int ret = 0;
	switch (id) {
	case 0:
		ret = EXPIO_INT_SD1_EN;
		break;
	case 1:
		ret = EXPIO_INT_SD2_EN;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

/*
 * Power on/off Sharp QVGA panel.
 */
void board_power_lcd(int on)
{
	if (on)
		__raw_writew(PBC_BCTRL1_LCDON, PBC_BCTRL1_SET_REG);
	else
		__raw_writew(PBC_BCTRL1_LCDON, PBC_BCTRL1_CLEAR_REG);
}

void gpio_owire_active(void)
{
	gpio_request_mux(MX27_PIN_RTCK, GPIO_MUX_ALT);
}

void gpio_owire_inactive(void)
{
	gpio_request_mux(MX27_PIN_RTCK, GPIO_MUX_PRIMARY);
}

EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);
EXPORT_SYMBOL(config_uartdma_event);
EXPORT_SYMBOL(gpio_usbh1_active);
EXPORT_SYMBOL(gpio_usbh1_inactive);
EXPORT_SYMBOL(gpio_usbh2_active);
EXPORT_SYMBOL(gpio_usbh2_inactive);
EXPORT_SYMBOL(gpio_usbotg_hs_active);
EXPORT_SYMBOL(gpio_usbotg_hs_inactive);
EXPORT_SYMBOL(gpio_usbotg_fs_active);
EXPORT_SYMBOL(gpio_usbotg_fs_inactive);
EXPORT_SYMBOL(gpio_i2c_active);
EXPORT_SYMBOL(gpio_i2c_inactive);
EXPORT_SYMBOL(gpio_spi_active);
EXPORT_SYMBOL(gpio_spi_inactive);
EXPORT_SYMBOL(gpio_nand_active);
EXPORT_SYMBOL(gpio_nand_inactive);
EXPORT_SYMBOL(gpio_sensor_active);
EXPORT_SYMBOL(gpio_sensor_inactive);
EXPORT_SYMBOL(gpio_sensor_reset);
EXPORT_SYMBOL(gpio_lcdc_active);
EXPORT_SYMBOL(gpio_lcdc_inactive);
EXPORT_SYMBOL(gpio_fs453_reset_low);
EXPORT_SYMBOL(gpio_fs453_reset_high);
EXPORT_SYMBOL(gpio_pmic_active);
EXPORT_SYMBOL(gpio_keypad_active);
EXPORT_SYMBOL(gpio_keypad_inactive);
EXPORT_SYMBOL(gpio_ata_active);
EXPORT_SYMBOL(gpio_ata_inactive);
EXPORT_SYMBOL(gpio_fec_active);
EXPORT_SYMBOL(gpio_fec_inactive);
EXPORT_SYMBOL(gpio_slcdc_active);
EXPORT_SYMBOL(gpio_slcdc_inactive);
EXPORT_SYMBOL(gpio_ssi_active);
EXPORT_SYMBOL(gpio_ssi_inactive);
EXPORT_SYMBOL(gpio_sdhc_active);
EXPORT_SYMBOL(gpio_sdhc_inactive);
EXPORT_SYMBOL(sdhc_get_card_det_status);
EXPORT_SYMBOL(sdhc_init_card_det);
EXPORT_SYMBOL(board_power_lcd);
EXPORT_SYMBOL(gpio_owire_active);
EXPORT_SYMBOL(gpio_owire_inactive);
