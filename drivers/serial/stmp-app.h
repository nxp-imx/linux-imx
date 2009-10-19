/*
 * Freescale STMP37XX/STMP378X Application UART driver
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __STMP_APPUART_H
#define __STMP_APPUART_H

#define RX_BUFFER_SIZE	4
#define TX_BUFFER_SIZE	0xFFF0

#include <mach/dma.h>

/* #define RX_CHAIN 2 */

struct stmp_appuart_port {
	int	keep_irq;
	int	irq[3];
	void __iomem *mem;
	u32	memsize;
	int	dma_rx, dma_tx;
	struct clk *clk;
	struct device *dev;
	struct uart_port port;
	unsigned tx_buffer_index;
	struct stmp3xxx_dma_descriptor tx_desc;
#ifndef RX_CHAIN
	struct stmp3xxx_dma_descriptor rx_desc;
#else
	struct stmp3xxx_dma_descriptor rxd[RX_CHAIN];
	struct stmp37xx_circ_dma_chain rx_chain;
#endif

	u32 ctrl;
	u8 running;
	spinlock_t lock; /* protects irq handler */
};

#ifdef CONFIG_CPU_FREQ
static int stmp_appuart_updateclk(struct device *dev, void *clkdata);
static int stmp_appuart_notifier(struct notifier_block *self,
		unsigned long phase, void *p);
#endif /* CONFIG_CPU_FREQ */
static int __init stmp_appuart_probe(struct platform_device *device);
static int stmp_appuart_remove(struct platform_device *device);
static int stmp_appuart_suspend(struct platform_device *device,
		pm_message_t state);
static int stmp_appuart_resume(struct platform_device *device);
static int __init stmp_appuart_init(void);
static void __exit stmp_appuart_exit(void);
static int stmp_appuart_request_port(struct uart_port *u);
static void stmp_appuart_release_port(struct uart_port *u);
static int stmp_appuart_verify_port(struct uart_port *u,
		struct serial_struct *);
static void stmp_appuart_config_port(struct uart_port *u, int flags);
static const char *stmp_appuart_type(struct uart_port *u);
static void stmp_appuart_settermios(struct uart_port *u,
		struct ktermios *nw, struct ktermios *old);
static void stmp_appuart_shutdown(struct uart_port *u);
static int stmp_appuart_startup(struct uart_port *u);
static u32 stmp_appuart_get_mctrl(struct uart_port *u);
static void stmp_appuart_set_mctrl(struct uart_port *u, unsigned mctrl);
static void stmp_appuart_enable_ms(struct uart_port *port);
static void stmp_appuart_break_ctl(struct uart_port *port, int ctl);
static unsigned int stmp_appuart_tx_empty(struct uart_port *u);
static void stmp_appuart_stop_rx(struct uart_port *u);
static void stmp_appuart_start_tx(struct uart_port *u);
static void stmp_appuart_stop_tx(struct uart_port *u);
static int stmp_appuart_copy_tx(struct uart_port *u, u8 *target, int size);
#endif
