/*
 * Freescale STMP378X LCDIF low-level routines
 *
 * Author: Vitaly Wool <vital@embeddedalley.com>
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
/* #define DEBUG */

#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/dma.h>
#include <mach/regs-lcdif.h>
#include <mach/regs-pinctrl.h>
#include <mach/lcdif.h>

#define MAX_CHAIN_LEN		10

static struct stmp3xxx_dma_descriptor video_dma_descriptor[MAX_CHAIN_LEN];
static struct stmp3xxx_lcd_dma_chain_info dma_chain_info[MAX_CHAIN_LEN];
static unsigned dma_chain_info_pos;

void stmp3xxx_init_lcdif(void)
{
	stmp3xxx_clearl(BM_LCDIF_CTRL_CLKGATE, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	/* Reset controller */
	stmp3xxx_setl(BM_LCDIF_CTRL_SFTRST, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	udelay(10);

	/* Take controller out of reset */
	stmp3xxx_clearl(BM_LCDIF_CTRL_SFTRST | BM_LCDIF_CTRL_CLKGATE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL);

	/* Setup the bus protocol */
	stmp3xxx_clearl(BM_LCDIF_CTRL1_MODE86,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1);
	stmp3xxx_clearl(BM_LCDIF_CTRL1_BUSY_ENABLE,
			REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

	/* Take display out of reset */
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);

	/* VSYNC is an input by default */
	stmp3xxx_setl(BM_LCDIF_VDCTRL0_VSYNC_OEB,
		      REGS_LCDIF_BASE + HW_LCDIF_VDCTRL0);

	/* Reset display */
	stmp3xxx_clearl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);
	udelay(10);
	stmp3xxx_setl(BM_LCDIF_CTRL1_RESET, REGS_LCDIF_BASE + HW_LCDIF_CTRL1);
	udelay(10);
}

EXPORT_SYMBOL(stmp3xxx_init_lcdif);

static int stmp378x_lcd_master = 1;
int stmp3xxx_lcdif_dma_init(struct device *dev, dma_addr_t phys, int memsize,
			    int lcd_master)
{
	int ret = 0;

	stmp378x_lcd_master = lcd_master;
	if (lcd_master) {
		stmp3xxx_setl(BM_LCDIF_CTRL_LCDIF_MASTER,
			      REGS_LCDIF_BASE + HW_LCDIF_CTRL);

		__raw_writel(phys, REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF);
		__raw_writel(phys, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);
	} else {
		ret =
		    stmp3xxx_dma_request(STMP3XXX_DMA
					 (LCD_DMA_CHANNEL, STMP3XXX_BUS_APBH),
					 dev, "lcdif");
		if (ret) {
			dev_err(dev,
				"stmp3xxx_dma_request failed: error %d\n", ret);
			goto out;
		}

		stmp3xxx_dma_reset_channel(STMP3XXX_DMA
					   (LCD_DMA_CHANNEL,
					    STMP3XXX_BUS_APBH));

		stmp3xxx_dma_clear_interrupt(STMP3XXX_DMA
					     (LCD_DMA_CHANNEL,
					      STMP3XXX_BUS_APBH));
		stmp3xxx_dma_enable_interrupt(STMP3XXX_DMA
					      (LCD_DMA_CHANNEL,
					       STMP3XXX_BUS_APBH));

		dotclk_dma_chain_init(memsize, phys, video_dma_descriptor,
				      dma_chain_info, &dma_chain_info_pos);
	}
out:
	return ret;
}

EXPORT_SYMBOL(stmp3xxx_lcdif_dma_init);

void stmp3xxx_lcdif_dma_release(void)
{
	int i;

	if (stmp378x_lcd_master) {
		stmp3xxx_clearl(BM_LCDIF_CTRL_LCDIF_MASTER,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		return;
	}

	for (i = 0; i < dma_chain_info_pos; i++)
		stmp3xxx_dma_free_command(STMP3XXX_DMA
					  (LCD_DMA_CHANNEL, STMP3XXX_BUS_APBH),
					  &video_dma_descriptor[i]);
	stmp3xxx_dma_release(STMP3XXX_DMA(LCD_DMA_CHANNEL, STMP3XXX_BUS_APBH));

	dma_chain_info_pos = 0;
}

EXPORT_SYMBOL(stmp3xxx_lcdif_dma_release);

void stmp3xxx_lcdif_run(void)
{
	if (stmp378x_lcd_master) {
		stmp3xxx_setl(BM_LCDIF_CTRL_LCDIF_MASTER,
			      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		stmp3xxx_setl(BM_LCDIF_CTRL_RUN,
			      REGS_LCDIF_BASE + HW_LCDIF_CTRL);
	} else {
		video_dma_descriptor[dma_chain_info_pos - 1].command->cmd &=
		    ~BM_APBH_CHn_CMD_SEMAPHORE;
		stmp3xxx_dma_go(STMP3XXX_DMA
				(LCD_DMA_CHANNEL, STMP3XXX_BUS_APBH),
				video_dma_descriptor, 1);
	}
}

EXPORT_SYMBOL(stmp3xxx_lcdif_run);

void stmp3xxx_lcdif_stop(void)
{
	if (stmp378x_lcd_master) {
		stmp3xxx_clearl(BM_LCDIF_CTRL_RUN,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		stmp3xxx_clearl(BM_LCDIF_CTRL_LCDIF_MASTER,
				REGS_LCDIF_BASE + HW_LCDIF_CTRL);
		udelay(100);
	} else {
		video_dma_descriptor[dma_chain_info_pos - 1].command->cmd |=
		    BM_APBH_CHn_CMD_SEMAPHORE;
		udelay(100);
	}
	stmp3xxx_setl(BM_LCDIF_CTRL_CLKGATE, REGS_LCDIF_BASE + HW_LCDIF_CTRL);
}

EXPORT_SYMBOL(stmp3xxx_lcdif_stop);

int stmp3xxx_lcdif_pan_display(dma_addr_t addr)
{
	if (stmp378x_lcd_master)
		__raw_writel(addr, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);
	else {
		int i;
		/* Modify the chain addresses */
		for (i = 0; i < dma_chain_info_pos; ++i) {
			*dma_chain_info[i].dma_addr_p = addr +
			    dma_chain_info[i].offset;
			barrier();
		}
	}
	return 0;
}

EXPORT_SYMBOL(stmp3xxx_lcdif_pan_display);

static BLOCKING_NOTIFIER_HEAD(lcdif_client_list);

int stmp3xxx_lcdif_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&lcdif_client_list, nb);
}

EXPORT_SYMBOL(stmp3xxx_lcdif_register_client);

void stmp3xxx_lcdif_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&lcdif_client_list, nb);
}

EXPORT_SYMBOL(stmp3xxx_lcdif_unregister_client);

void stmp3xxx_lcdif_notify_clients(unsigned long event,
				   struct stmp3xxx_platform_fb_entry *pentry)
{
	blocking_notifier_call_chain(&lcdif_client_list, event, pentry);
}

EXPORT_SYMBOL(stmp3xxx_lcdif_notify_clients);
