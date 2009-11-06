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
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/regs-apbx.h>
#include <mach/regs-uartapp.h>
#include <mach/regs-pinctrl.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>

#include <asm/mach-types.h>

#include "stmp-app.h"

static int pio_mode /* = 0 */; 	/* PIO mode = 1, DMA mode = 0	*/

static struct platform_driver stmp_appuart_driver = {
	.probe = stmp_appuart_probe,
	.remove = __devexit_p(stmp_appuart_remove),
	.suspend = stmp_appuart_suspend,
	.resume = stmp_appuart_resume,
	.driver = {
		.name = "stmp3xxx-appuart",
		.owner = THIS_MODULE,
	},
};

static struct uart_driver stmp_appuart_uart = {
	.owner = THIS_MODULE,
	.driver_name = "appuart",
	.dev_name = "ttySP",
	.major = 242,
	.minor = 0,
	.nr = 1,
};

static inline struct stmp_appuart_port *to_appuart(struct uart_port *u)
{
	return container_of(u, struct stmp_appuart_port, port);
}

static struct uart_ops stmp_appuart_ops = {
	.tx_empty       = stmp_appuart_tx_empty,
	.start_tx       = stmp_appuart_start_tx,
	.stop_tx	= stmp_appuart_stop_tx,
	.stop_rx	= stmp_appuart_stop_rx,
	.enable_ms      = stmp_appuart_enable_ms,
	.break_ctl      = stmp_appuart_break_ctl,
	.set_mctrl	= stmp_appuart_set_mctrl,
	.get_mctrl      = stmp_appuart_get_mctrl,
	.startup	= stmp_appuart_startup,
	.shutdown       = stmp_appuart_shutdown,
	.set_termios    = stmp_appuart_settermios,
	.type	   	= stmp_appuart_type,
	.release_port   = stmp_appuart_release_port,
	.request_port   = stmp_appuart_request_port,
	.config_port    = stmp_appuart_config_port,
	.verify_port    = stmp_appuart_verify_port,
};

static inline int chr(int c)
{
	if (c < 0x20 || c > 0x7F)
		return '#';
	return c;
}

/* Allocate and initialize rx and tx DMA chains */
static inline int stmp_appuart_dma_init(struct stmp_appuart_port *s)
{
	int err = 0;
	struct stmp3xxx_dma_descriptor *t = &s->tx_desc;
#ifndef RX_CHAIN
	struct stmp3xxx_dma_descriptor *r = &s->rx_desc;
#else
	int i;
#endif

	err = stmp3xxx_dma_request(s->dma_rx, s->dev, dev_name(s->dev));
	if (err)
		goto out;
	err = stmp3xxx_dma_request(s->dma_tx, s->dev, dev_name(s->dev));
	if (err)
		goto out1;

#ifndef RX_CHAIN
	err = stmp3xxx_dma_allocate_command(s->dma_rx, r);
	if (err)
		goto out2;
#endif
	err = stmp3xxx_dma_allocate_command(s->dma_tx, t);
	if (err)
		goto out3;
	t->virtual_buf_ptr = dma_alloc_coherent(s->dev,
						TX_BUFFER_SIZE,
						&t->command->buf_ptr, GFP_DMA);
	if (!t->virtual_buf_ptr)
		goto out4;
#ifdef DEBUG
	memset(t->virtual_buf_ptr, 0x4B, TX_BUFFER_SIZE);
#endif

#ifndef RX_CHAIN
	r->virtual_buf_ptr = dma_alloc_coherent(s->dev,
						RX_BUFFER_SIZE,
						&r->command->buf_ptr, GFP_DMA);
	if (!r->virtual_buf_ptr)
		goto out5;
#ifdef DEBUG
	memset(r->virtual_buf_ptr, 0x4C, RX_BUFFER_SIZE);
#endif
#else
	stmp3xxx_dma_make_chain(s->dma_rx, &s->rx_chain, s->rxd, RX_CHAIN);
	for (i = 0; i < RX_CHAIN; i++) {
		struct stmp3xxx_dma_descriptor *r = s->rxd + i;

		r->command->cmd =
		    BF(RX_BUFFER_SIZE, APBX_CHn_CMD_XFER_COUNT) |
		    BF(1, APBX_CHn_CMD_CMDWORDS) |
		    BM_APBX_CHn_CMD_WAIT4ENDCMD |
		    BM_APBX_CHn_CMD_SEMAPHORE |
		    BM_APBX_CHn_CMD_IRQONCMPLT |
		    BM_APBX_CHn_CMD_CHAIN |
		    BF_APBX_CHn_CMD_COMMAND(BV_APBX_CHn_CMD_COMMAND__DMA_WRITE);
		r->virtual_buf_ptr = dma_alloc_coherent(s->dev,
							RX_BUFFER_SIZE,
							&r->command->buf_ptr,
							GFP_DMA);
		r->command->pio_words[0] =	/* BM_UARTAPP_CTRL0_RUN | */
		    BF(RX_BUFFER_SIZE, UARTAPP_CTRL0_XFER_COUNT) |
		    BM_UARTAPP_CTRL0_RXTO_ENABLE |
		    BF(3, UARTAPP_CTRL0_RXTIMEOUT);
	}
#endif
	return 0;

	/*
	 * would be necessary on other error paths

	dma_free_coherent( s->dev, RX_BUFFER_SIZE, r->virtual_buf_ptr,
			   r->command->buf_ptr);
	*/
out5:
	dma_free_coherent(s->dev, TX_BUFFER_SIZE, t->virtual_buf_ptr,
			   t->command->buf_ptr);
out4:
	stmp3xxx_dma_free_command(s->dma_tx, t);
out3:
#ifndef RX_CHAIN
	stmp3xxx_dma_free_command(s->dma_rx, r);
#endif
out2:
	stmp3xxx_dma_release(s->dma_tx);
out1:
	stmp3xxx_dma_release(s->dma_rx);
out:
	WARN_ON(err);
	return err;
}


static void stmp_appuart_on(struct platform_device *dev)
{
	struct stmp_appuart_port *s = platform_get_drvdata(dev);

	if (!pio_mode) {
		/*
		   Tell DMA to select UART.
		   Both DMA channels are shared between app UART and IrDA.
		   Target id of 0 means UART, 1 means IrDA
		 */
		stmp3xxx_dma_set_alt_target(s->dma_rx, 0);
		stmp3xxx_dma_set_alt_target(s->dma_tx, 0);
		/*
		  Reset DMA channels
		 */
		stmp3xxx_dma_reset_channel(s->dma_rx);
		stmp3xxx_dma_reset_channel(s->dma_tx);
		stmp3xxx_dma_enable_interrupt(s->dma_rx);
		stmp3xxx_dma_enable_interrupt(s->dma_tx);
	}
}

#ifdef CONFIG_CPU_FREQ
static int stmp_appuart_updateclk(struct device *dev, void *clkdata)
{
	struct stmp_appuart_port *s = dev_get_drvdata(dev);

	if (s) {
		s->port.uartclk = clk_get_rate(s->clk) * 1000;
		/* FIXME: perform actual update */
	}
	return 0;
}

static int stmp_appuart_notifier(struct notifier_block *self,
				 unsigned long phase, void *p)
{
	int r = 0;

	if ((phase == CPUFREQ_POSTCHANGE) || (phase == CPUFREQ_RESUMECHANGE)) {
		/* get new uartclock and setspeed */
		r = driver_for_each_device(&stmp_appuart_driver.driver,
					   NULL, p, stmp_appuart_updateclk);
	}
	return (r == 0) ? NOTIFY_OK : NOTIFY_DONE;
}

static struct notifier_block stmp_appuart_nb = {
	.notifier_call = &stmp_appuart_notifier,
};
#endif /* CONFIG_CPU_FREQ */

static int __devinit stmp_appuart_probe(struct platform_device *device)
{
	struct stmp_appuart_port *s;
	int err = 0;
	struct resource *r;
	int i;
	u32 version;
	int (*pinctl)(int req, int id);

	s = kzalloc(sizeof(struct stmp_appuart_port), GFP_KERNEL);
	if (!s) {
		err = -ENOMEM;
		goto out;
	}

	spin_lock_init(&s->lock);

	s->clk = clk_get(NULL, "uart");
	if (IS_ERR(s->clk)) {
		err = PTR_ERR(s->clk);
		goto out_free;
	}
	clk_enable(s->clk);
	r = platform_get_resource(device, IORESOURCE_MEM, 0);
	if (!r) {
		err = -ENXIO;
		goto out_free_clk;
	}
	s->port.mapbase = r->start;
	s->port.irq = platform_get_irq(device, 0);
	s->port.ops = &stmp_appuart_ops;
	s->port.iotype = UPIO_MEM;
	s->port.line = device->id < 0 ? 0 : device->id;
	s->port.fifosize = 16;
	s->port.timeout = HZ/10;
	s->port.uartclk = clk_get_rate(s->clk) * 1000;
	s->port.type = PORT_IMX;
	s->port.dev = s->dev = get_device(&device->dev);
	s->ctrl = 0;
	s->keep_irq = 0;

	r = platform_get_resource(device, IORESOURCE_MEM, 0);
	if (!r) {
		err = -ENXIO;
		goto out_free_clk;
	}

	dev_dbg(s->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(s->irq); i++) {
		s->irq[i] = platform_get_irq(device, i);
		dev_dbg(s->dev, "Resources: irq[%d] = %d\n", i, s->irq[i]);
		if (s->irq[i] < 0) {
			err = s->irq[i];
			goto out_free_clk;
		}
	}

	r = platform_get_resource(device, IORESOURCE_DMA, 0);
	if (!r) {
		err = -ENXIO;
		goto out_free;
	}
	s->dma_rx = r->start;

	r = platform_get_resource(device, IORESOURCE_DMA, 1);
	if (!r) {
		err = -ENXIO;
		goto out_free;
	}
	s->dma_tx = r->start;

	r = platform_get_resource(device, IORESOURCE_MEM, 0);
	if (!r) {
		err = -ENXIO;
		goto out_free;
	}
	s->mem = (void __iomem *)(r->start - STMP3XXX_REGS_PHBASE
			+ (u32)STMP3XXX_REGS_BASE);
	s->memsize = r->end - r->start;

#ifdef CONFIG_CPU_FREQ
	cpufreq_register_notifier(&stmp_appuart_nb,
				  CPUFREQ_TRANSITION_NOTIFIER);
#endif
	platform_set_drvdata(device, s);

	device_init_wakeup(&device->dev, 1);

	stmp_appuart_dma_init(s);
	stmp_appuart_on(device);

	pinctl = device->dev.platform_data;
	if (pinctl) {
		err = pinctl(1, device->id);
		if (err)
			goto out_free_clk;
	}

	err = uart_add_one_port(&stmp_appuart_uart, &s->port);
	if (err)
		goto out_free_pins;

	version = __raw_readl(REGS_UARTAPP1_BASE + HW_UARTAPP_VERSION);
	printk(KERN_INFO "Found APPUART %d.%d.%d\n",
	       (version >> 24) & 0xFF,
	       (version >> 16) & 0xFF, version & 0xFFFF);
	return 0;

out_free_pins:
	if (pinctl)
		pinctl(0, device->id);
out_free_clk:
	clk_put(s->clk);
out_free:
	platform_set_drvdata(device, NULL);
	kfree(s);
out:
	return err;
}

static int __devexit stmp_appuart_remove(struct platform_device *device)
{
	struct stmp_appuart_port *s;
	void (*pinctl)(int req, int id);

	s = platform_get_drvdata(device);
	if (s) {
		pinctl = device->dev.platform_data;
		put_device(s->dev);
		clk_disable(s->clk);
		clk_put(s->clk);
		uart_remove_one_port(&stmp_appuart_uart, &s->port);
		if (pinctl)
			pinctl(0, device->id);
		kfree(s);
		platform_set_drvdata(device, NULL);
	}

	return 0;
}

static int stmp_appuart_suspend(struct platform_device *device,
				pm_message_t state)
{
#ifdef CONFIG_PM
	struct stmp_appuart_port *s = platform_get_drvdata(device);

	if (!s)
		return 0;
	s->keep_irq = device_may_wakeup(&device->dev);
	uart_suspend_port(&stmp_appuart_uart, &s->port);
	if (!s->keep_irq)
		clk_disable(s->clk);
#endif
	return 0;
}

static int stmp_appuart_resume(struct platform_device *device)
{
#ifdef CONFIG_PM
	struct stmp_appuart_port *s = platform_get_drvdata(device);

	if (!s)
		return 0;

	if (!s->keep_irq)
		clk_enable(s->clk);
	stmp_appuart_on(device);
	uart_resume_port(&stmp_appuart_uart, &s->port);
	s->keep_irq = 0;
#endif
	return 0;
}

static int __init stmp_appuart_init()
{
	int r;

	r = uart_register_driver(&stmp_appuart_uart);
	if (r)
		goto out;
	r = platform_driver_register(&stmp_appuart_driver);
	if (r)
		goto out_err;
	return 0;
out_err:
	uart_unregister_driver(&stmp_appuart_uart);
out:
	return r;
}

static void __exit stmp_appuart_exit()
{
	platform_driver_unregister(&stmp_appuart_driver);
	uart_unregister_driver(&stmp_appuart_uart);
}

module_init(stmp_appuart_init)
module_exit(stmp_appuart_exit)

static void stmp_appuart_stop_rx(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);
	stmp3xxx_clearl(BM_UARTAPP_CTRL2_RXE, s->mem);
}

static void stmp_appuart_break_ctl(struct uart_port *u, int ctl)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s: break = %s\n", __func__, ctl ? "on" : "off");
	if (ctl)
		stmp3xxx_setl(BM_UARTAPP_LINECTRL_BRK,
			      s->mem + HW_UARTAPP_LINECTRL);
	else
		stmp3xxx_clearl(BM_UARTAPP_LINECTRL_BRK,
				s->mem + HW_UARTAPP_LINECTRL);
}

static void stmp_appuart_enable_ms(struct uart_port *port)
{
	/* just empty */
}

static void stmp_appuart_set_mctrl(struct uart_port *u, unsigned mctrl)
{
	struct stmp_appuart_port *s = to_appuart(u);

	u32 ctrl = __raw_readl(s->mem + HW_UARTAPP_CTRL2);

	dev_dbg(s->dev, "%s (%x)\n", __func__, mctrl);
	ctrl &= ~BM_UARTAPP_CTRL2_RTS;
	if (mctrl & TIOCM_RTS) {
		dev_dbg(s->dev, "...RTS\n");
		ctrl |= BM_UARTAPP_CTRL2_RTS;
	}
	s->ctrl = mctrl;
	dev_dbg(s->dev, "...%x; ctrl = %x\n", s->ctrl, ctrl);
	__raw_writel(ctrl, s->mem + HW_UARTAPP_CTRL2);
}

static u32 stmp_appuart_get_mctrl(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);
	u32 stat = __raw_readl(s->mem + HW_UARTAPP_STAT);
	int ctrl2 = __raw_readl(s->mem + HW_UARTAPP_CTRL2);
	u32 mctrl = s->ctrl;

	dev_dbg(s->dev, "%s:\n", __func__);
	mctrl &= ~TIOCM_CTS;
	if (stat & BM_UARTAPP_STAT_CTS) {
		dev_dbg(s->dev, "CTS");
		mctrl |= TIOCM_CTS;
	}
	if (ctrl2 & BM_UARTAPP_CTRL2_RTS) {
		dev_dbg(s->dev, "RTS");
		mctrl |= TIOCM_RTS;
	}
	dev_dbg(s->dev, "...%x\n", mctrl);
	return mctrl;
}

static int stmp_appuart_request_port(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);
	int err = 0;

	if (!request_mem_region((u32)s->mem, s->memsize, dev_name(s->dev)))
		err = -ENXIO;
	return err;

}

static void stmp_appuart_release_port(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	release_mem_region((u32)s->mem, s->memsize);
}

static int stmp_appuart_verify_port(struct uart_port *u,
				    struct serial_struct *ser)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);
	return 0;
}

static void stmp_appuart_config_port(struct uart_port *u, int flags)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);
}

static const char *stmp_appuart_type(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);
	return dev_name(s->dev);
}

static void stmp_appuart_settermios(struct uart_port *u,
				    struct ktermios *nw, struct ktermios *old)
{
	static struct ktermios saved;
	struct stmp_appuart_port *s = to_appuart(u);
	unsigned int cflag;
	u32 bm, ctrl, ctrl2, div;
	int err = 0;
	unsigned baud;

	dev_dbg(s->dev, "%s\n", __func__);

	if (nw)
		memcpy(&saved, nw, sizeof *nw);
	else
		nw = old = &saved;

	cflag = nw->c_cflag;

	ctrl = BM_UARTAPP_LINECTRL_FEN;
	ctrl2 = __raw_readl(s->mem + HW_UARTAPP_CTRL2);

	/* byte size */
	switch (cflag & CSIZE) {
	case CS5:
		bm = 0;
		break;
	case CS6:
		bm = 1;
		break;
	case CS7:
		bm = 2;
		break;
	case CS8:
		bm = 3;
		break;
	default:
		err = -EINVAL;
		break;
	}
	if (err)
		goto out;

	dev_dbg(s->dev, "Byte size %d bytes, mask %x\n",
		bm + 5, BF(bm, UARTAPP_LINECTRL_WLEN));
	ctrl |= BF(bm, UARTAPP_LINECTRL_WLEN);

	/* parity */
	if (cflag & PARENB) {
		dev_dbg(s->dev, "Parity check enabled\n");
		ctrl |= BM_UARTAPP_LINECTRL_PEN | BM_UARTAPP_LINECTRL_SPS;
		if ((cflag & PARODD) == 0) {
			dev_dbg(s->dev, "(Even) mask = %x\n",
				BM_UARTAPP_LINECTRL_PEN |
				BM_UARTAPP_LINECTRL_SPS |
				BM_UARTAPP_LINECTRL_EPS);
			ctrl |= BM_UARTAPP_LINECTRL_EPS;
		} else
			dev_dbg(s->dev, "(Odd) mask = %x\n",
				BM_UARTAPP_LINECTRL_PEN |
				BM_UARTAPP_LINECTRL_SPS);
	} else
		dev_dbg(s->dev, "Parity check disabled.\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB) {
		dev_dbg(s->dev, "Stop bits, mask = %x\n",
			BM_UARTAPP_LINECTRL_STP2);
		ctrl |= BM_UARTAPP_LINECTRL_STP2;
	} else
		dev_dbg(s->dev, "No stop bits\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS) {
		dev_dbg(s->dev, "RTS/CTS flow control\n");
		ctrl2 |= BM_UARTAPP_CTRL2_CTSEN /* | BM_UARTAPP_CTRL2_RTSEN */ ;
	} else {
		dev_dbg(s->dev, "RTS/CTS disabled\n");
		ctrl2 &= ~BM_UARTAPP_CTRL2_CTSEN;
	}

	/* set baud rate */
	baud = uart_get_baud_rate(u, nw, old, 0, u->uartclk);
	dev_dbg(s->dev, "Baud rate requested: %d (clk = %d)\n",
		baud, u->uartclk);
	div = u->uartclk * 32 / baud;
	ctrl |= BF(div & 0x3F, UARTAPP_LINECTRL_BAUD_DIVFRAC);
	ctrl |= BF(div >> 6, UARTAPP_LINECTRL_BAUD_DIVINT);

	if ((cflag & CREAD) != 0) {
		dev_dbg(s->dev, "RX started\n");
		ctrl2 |= BM_UARTAPP_CTRL2_RXE | BM_UARTAPP_CTRL2_RXDMAE;
	}

	if (!err) {
		dev_dbg(s->dev, "CTRLS = %x + %x\n", ctrl, ctrl2);
		__raw_writel(ctrl,
			     s->mem + HW_UARTAPP_LINECTRL);
		__raw_writel(ctrl2,
			     s->mem + HW_UARTAPP_CTRL2);
	}
out:
	return /* err */ ;
}

static int stmp_appuart_free_irqs(struct stmp_appuart_port *s)
{
	int irqn = 0;

	if (s->keep_irq) {
		dev_dbg(s->dev, "keep_irq != 0, ignoring\n");
		return 0;
	}
	for (irqn = 0; irqn < ARRAY_SIZE(s->irq); irqn++)
		free_irq(s->irq[irqn], s);
	return 0;
}

void stmp_appuart_rx(struct stmp_appuart_port *s, u8 * rx_buffer, int count)
{
	u8 c;
	int flag;
	struct tty_struct *tty = s->port.info->port.tty;
	u32 stat;

	spin_lock(&s->lock);
	stat = __raw_readl(s->mem + HW_UARTAPP_STAT);

	if (count < 0) {
		count =
		    __raw_readl(s->mem +
				HW_UARTAPP_STAT) & BM_UARTAPP_STAT_RXCOUNT;
		dev_dbg(s->dev, "count = %d\n", count);
	}

	for (;;) {
		if (!rx_buffer) {
			if (stat & BM_UARTAPP_STAT_RXFE)
				break;
			c = __raw_readl(s->mem + HW_UARTAPP_DATA) & 0xFF;
		} else {
			if (count-- <= 0)
				break;
			c = *rx_buffer++;
			dev_dbg(s->dev, "Received: %x(%c)\n", c, chr(c));
		}

		flag = TTY_NORMAL;
		if (stat & BM_UARTAPP_STAT_BERR) {
			stat &= ~BM_UARTAPP_STAT_BERR;
			s->port.icount.brk++;
			if (uart_handle_break(&s->port))
				goto ignore;
			flag = TTY_BREAK;
		} else if (stat & BM_UARTAPP_STAT_PERR) {
			stat &= ~BM_UARTAPP_STAT_PERR;
			s->port.icount.parity++;
			flag = TTY_PARITY;
		} else if (stat & BM_UARTAPP_STAT_FERR) {
			stat &= ~BM_UARTAPP_STAT_FERR;
			s->port.icount.frame++;
			flag = TTY_FRAME;
		}

		if (stat & BM_UARTAPP_STAT_OERR)
			s->port.icount.overrun++;

		if (uart_handle_sysrq_char(&s->port, c))
			goto ignore;

		uart_insert_char(&s->port, stat, BM_UARTAPP_STAT_OERR, c, flag);
ignore:
		if (pio_mode) {
			__raw_writel(stat, s->mem + HW_UARTAPP_STAT);
			stat =
			    __raw_readl(s->mem + HW_UARTAPP_STAT);
		}
	}

	__raw_writel(stat, s->mem + HW_UARTAPP_STAT);
	tty_flip_buffer_push(tty);
	spin_unlock(&s->lock);
}

static inline void stmp_appuart_submit_rx(struct stmp_appuart_port *s)
{
#ifndef RX_CHAIN
	struct stmp3xxx_dma_descriptor *r = &s->rx_desc;

	dev_dbg(s->dev, "Submitting RX DMA request\n");
	r->command->cmd =
	    BM_APBX_CHn_CMD_HALTONTERMINATE |
	    BF(RX_BUFFER_SIZE, APBX_CHn_CMD_XFER_COUNT) |
	    BF(1, APBX_CHn_CMD_CMDWORDS) |
	    BM_APBX_CHn_CMD_WAIT4ENDCMD |
	    BM_APBX_CHn_CMD_SEMAPHORE |
	    BM_APBX_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBX_CHn_CMD_COMMAND__DMA_WRITE, APBX_CHn_CMD_COMMAND);
	r->command->pio_words[0] =
	    __raw_readl(REGS_UARTAPP1_BASE +
			HW_UARTAPP_CTRL0) | BF(RX_BUFFER_SIZE,
					       UARTAPP_CTRL0_XFER_COUNT) |
	    BM_UARTAPP_CTRL0_RXTO_ENABLE | BF(3, UARTAPP_CTRL0_RXTIMEOUT);
	r->command->pio_words[0] &= ~BM_UARTAPP_CTRL0_RUN;

	stmp3xxx_dma_reset_channel(s->dma_rx);
	stmp3xxx_dma_go(s->dma_rx, r, 1);
#endif
}

static irqreturn_t stmp_appuart_irq_int(int irq, void *context)
{
	u32 istatus;
	struct stmp_appuart_port *s = context;
	u32 stat = __raw_readl(s->mem + HW_UARTAPP_STAT);

	istatus = __raw_readl(s->mem + HW_UARTAPP_INTR);
	dev_dbg(s->dev, "IRQ: int(%d), status = %08X\n", irq, istatus);

	if (istatus & BM_UARTAPP_INTR_CTSMIS) {
		uart_handle_cts_change(&s->port, stat & BM_UARTAPP_STAT_CTS);
		dev_dbg(s->dev, "CTS change: %x\n", stat & BM_UARTAPP_STAT_CTS);
		stmp3xxx_clearl(BM_UARTAPP_INTR_CTSMIS,
				s->mem + HW_UARTAPP_INTR);
	}

	else if (istatus & BM_UARTAPP_INTR_RTIS) {
		dev_dbg(s->dev, "RX timeout, draining out\n");
		stmp_appuart_submit_rx(s);
	}

	else
		dev_info(s->dev, "Unhandled status %x\n", istatus);

	stmp3xxx_clearl(istatus & 0xFFFF,
			s->mem + HW_UARTAPP_INTR);

	return IRQ_HANDLED;
}

static irqreturn_t stmp_appuart_irq_rx(int irq, void *context)
{
	struct stmp_appuart_port *s = context;
	int count = -1;

	stmp3xxx_dma_clear_interrupt(s->dma_rx);
	dev_dbg(s->dev, "%s(%d), count = %d\n", __func__, irq, count);

#ifndef RX_CHAIN
	stmp_appuart_rx(s, s->rx_desc.virtual_buf_ptr, count);
	stmp_appuart_submit_rx(s);
#else
	if (circ_advance_cooked(&s->rx_chain) == 0) {
		BUG();
		return IRQ_HANDLED;
	}

	circ_advance_active(&s->rx_chain, 1);
	while (s->rx_chain.cooked_count) {
		stmp_appuart_rx(s,
				stmp3xxx_dma_circ_get_cooked_head(&s->
								  rx_chain)->virtual_buf_ptr,
				-1);
		circ_advance_free(&s->rx_chain, 1);
	}
#endif
	return IRQ_HANDLED;
}

static void stmp_appuart_submit_tx(struct stmp_appuart_port *s, int size)
{
	struct stmp3xxx_dma_descriptor *d = &s->tx_desc;

	dev_dbg(s->dev, "Submitting TX DMA request, %d bytes\n", size);
	d->command->pio_words[0] =
	    /* BM_UARTAPP_CTRL1_RUN | */ BF(size, UARTAPP_CTRL1_XFER_COUNT);
	d->command->cmd = BF(size, APBX_CHn_CMD_XFER_COUNT) |
	    BF(1, APBX_CHn_CMD_CMDWORDS) |
	    BM_APBX_CHn_CMD_WAIT4ENDCMD |
	    BM_APBX_CHn_CMD_SEMAPHORE |
	    BM_APBX_CHn_CMD_IRQONCMPLT |
	    BF(BV_APBX_CHn_CMD_COMMAND__DMA_READ, APBX_CHn_CMD_COMMAND);
	stmp3xxx_dma_go(s->dma_tx, d, 1);
}

static irqreturn_t stmp_appuart_irq_tx(int irq, void *context)
{
	struct stmp_appuart_port *s = context;
	struct uart_port *u = &s->port;
	int bytes;

	stmp3xxx_dma_clear_interrupt(s->dma_tx);
	dev_dbg(s->dev, "%s(%d)\n", __func__, irq);

	bytes = stmp_appuart_copy_tx(u, s->tx_desc.virtual_buf_ptr,
				     TX_BUFFER_SIZE);
	if (bytes > 0) {
		dev_dbg(s->dev, "Sending %d bytes\n", bytes);
		stmp_appuart_submit_tx(s, bytes);
	}
	return IRQ_HANDLED;
}

static int stmp_appuart_request_irqs(struct stmp_appuart_port *s)
{
	int err = 0;

	/*
	 * order counts. resources should be listed in the same order
	 */
	irq_handler_t handlers[] = {
		stmp_appuart_irq_int,
		stmp_appuart_irq_rx,
		stmp_appuart_irq_tx,
	};
	char *handlers_names[] = {
		"appuart internal",
		"appuart rx",
		"appuart tx",
	};
	int irqn;

	if (s->keep_irq) {
		dev_dbg(s->dev, "keep_irq is set, skipping request_irq");
		return 0;
	}
	for (irqn = 0; irqn < ARRAY_SIZE(handlers); irqn++) {
		err = request_irq(s->irq[irqn], handlers[irqn],
				  0, handlers_names[irqn], s);
		dev_dbg(s->dev, "Requested IRQ %d with status %d\n",
			s->irq[irqn], err);
		if (err)
			goto out;
	}
	return 0;
out:
	stmp_appuart_free_irqs(s);
	return err;
}

static struct timer_list timer_task;

static void stmp_appuart_check_rx(unsigned long data)
{
	stmp_appuart_rx((struct stmp_appuart_port *)data, NULL, -1);
	mod_timer(&timer_task, jiffies + 2 * HZ);
}

static int stmp_appuart_startup(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);
	int err;

	dev_dbg(s->dev, "%s\n", __func__);

	s->tx_buffer_index = 0;

	err = stmp_appuart_request_irqs(s);
	if (err)
		goto out;

	if (!s->keep_irq)
		/* Release the block from reset and start the clocks. */
		stmp3xxx_reset_block(s->mem, 0);

	stmp3xxx_setl(BM_UARTAPP_CTRL2_UARTEN,
			s->mem + HW_UARTAPP_CTRL2);
	/* Enable the Application UART DMA bits. */
	if (!pio_mode) {
		stmp3xxx_setl(BM_UARTAPP_CTRL2_TXDMAE | BM_UARTAPP_CTRL2_RXDMAE
			      | BM_UARTAPP_CTRL2_DMAONERR,
			      s->mem + HW_UARTAPP_CTRL2);
		/* clear any pending interrupts */
		__raw_writel(0, s->mem + HW_UARTAPP_INTR);

		/* reset all dma channels */
		stmp3xxx_dma_reset_channel(s->dma_tx);
		stmp3xxx_dma_reset_channel(s->dma_rx);
	} else {
		__raw_writel(BM_UARTAPP_INTR_RXIEN |
			     BM_UARTAPP_INTR_RTIEN,
			     s->mem + HW_UARTAPP_INTR);
	}
	stmp3xxx_setl(BM_UARTAPP_INTR_CTSMIEN,
			s->mem + HW_UARTAPP_INTR);

	/*
	 * Enable fifo so all four bytes of a DMA word are written to
	 * output (otherwise, only the LSB is written, ie. 1 in 4 bytes)
	 */
	stmp3xxx_setl(BM_UARTAPP_LINECTRL_FEN, s->mem + HW_UARTAPP_LINECTRL);

	if (!pio_mode) {
#ifndef RX_CHAIN
		stmp_appuart_submit_rx(s);
#else
		circ_clear_chain(&s->rx_chain);
		stmp3xxx_dma_go(s->dma_rx, &s->rxd[0], 0);
		circ_advance_active(&s->rx_chain, 1);
#endif
	} else {
		init_timer(&timer_task);
		timer_task.function = stmp_appuart_check_rx;
		timer_task.expires = jiffies + HZ;
		timer_task.data = (unsigned long)s;
		add_timer(&timer_task);
	}

out:
	return err;
}

static void stmp_appuart_shutdown(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);

	if (!s->keep_irq)
		/* set the IP block to RESET; this should disable clock too. */
		stmp3xxx_setl(
			BM_UARTAPP_CTRL0_SFTRST, s->mem + HW_UARTAPP_CTRL0);

	if (!pio_mode) {
		/* reset all dma channels */
		stmp3xxx_dma_reset_channel(s->dma_tx);
		stmp3xxx_dma_reset_channel(s->dma_rx);
	} else {
		del_timer(&timer_task);
	}
	stmp_appuart_free_irqs(s);
}

static unsigned int stmp_appuart_tx_empty(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	if (pio_mode)
		if (__raw_readl(s->mem + HW_UARTAPP_STAT) &
		    BM_UARTAPP_STAT_TXFE)
			return TIOCSER_TEMT;
		else
			return 0;
	else
		return stmp3xxx_dma_running(s->dma_tx) ? 0 : TIOCSER_TEMT;
}

static void stmp_appuart_start_tx(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);
	int bytes;

	dev_dbg(s->dev, "%s\n", __func__);

	/* enable transmitter */
	stmp3xxx_setl(BM_UARTAPP_CTRL2_TXE, s->mem + HW_UARTAPP_CTRL2);

	if (!pio_mode) {
		if (stmp3xxx_dma_running(s->dma_tx))
			return;
		bytes = stmp_appuart_copy_tx(u, s->tx_desc.virtual_buf_ptr,
					     TX_BUFFER_SIZE);
		if (bytes <= 0)
			return;

		dev_dbg(s->dev, "Started DMA transfer with descriptor %p, "
			"command %p, %d bytes long\n",
			&s->tx_desc, s->tx_desc.command, bytes);
		stmp_appuart_submit_tx(s, bytes);
	} else {
		int count = 0;
		u8 c;

		while (!
		       (__raw_readl
			(s->mem + HW_UARTAPP_STAT) & BM_UARTAPP_STAT_TXFF)) {
			if (stmp_appuart_copy_tx(u, &c, 1) <= 0)
				break;
			dev_dbg(s->dev, "%d: '%c'/%x\n", ++count, chr(c), c);
			__raw_writel(c, s->mem + HW_UARTAPP_DATA);
		}
	}
}

static void stmp_appuart_stop_tx(struct uart_port *u)
{
	struct stmp_appuart_port *s = to_appuart(u);

	dev_dbg(s->dev, "%s\n", __func__);
	stmp3xxx_clearl(BM_UARTAPP_CTRL2_TXE, s->mem + HW_UARTAPP_CTRL2);
}

static int stmp_appuart_copy_tx(struct uart_port *u, u8 * target,
				int tx_buffer_size)
{
	int last = 0, portion;
	struct circ_buf *xmit = &u->info->xmit;

	while (last < tx_buffer_size) {	/* let's fill the only descriptor */
		if (u->x_char) {
			target[last++] = u->x_char;
			u->x_char = 0;
		} else if (!uart_circ_empty(xmit) && !uart_tx_stopped(u)) {
			portion = min((u32) tx_buffer_size,
				      (u32) uart_circ_chars_pending(xmit));
			portion = min((u32) portion,
				      (u32) CIRC_CNT_TO_END(xmit->head,
							    xmit->tail,
							    UART_XMIT_SIZE));
			memcpy(target + last, &xmit->buf[xmit->tail], portion);
			xmit->tail = (xmit->tail + portion) &
			    (UART_XMIT_SIZE - 1);
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
				uart_write_wakeup(u);
			last += portion;
		} else {	/* All tx data copied into buffer */
			return last;
		}
	}
	return last;
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("stmp3xxx app uart driver");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
module_param(pio_mode, int, 0);
