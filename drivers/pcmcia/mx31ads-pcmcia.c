/*======================================================================
    drivers/pcmcia/mx31ads-pcmica.c

    Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.

    Device driver for the PCMCIA control functionality of i.Mx31
    microprocessors.

    The contents of this file are subject to the Mozilla Public
    License Version 1.1 (the "License"); you may not use this file
    except in compliance with the License. You may obtain a copy of
    the License at http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS
    IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
    implied. See the License for the specific language governing
    rights and limitations under the License.

    The initial developer of the original code is John G. Dorsey
    <john+@cs.cmu.edu>.  Portions created by John G. Dorsey are
    Copyright (C) 1999 John G. Dorsey.  All Rights Reserved.

    Alternatively, the contents of this file may be used under the
    terms of the GNU Public License version 2 (the "GPL"), in which
    case the provisions of the GPL are applicable instead of the
    above.  If you wish to allow the use of your version of this file
    only under the terms of the GPL and not to allow others to use
    your version of this file under the MPL, indicate your decision
    by deleting the provisions above and replace them with the notice
    and other provisions required by the GPL.  If you do not delete
    the provisions above, a recipient may use your version of this
    file under either the MPL or the GPL.

======================================================================*/

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <asm/mach-types.h>
#include <mach/pcmcia.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>

#include "mx31ads-pcmcia.h"
#include <linux/irq.h>

#define	MX31ADS_PCMCIA_IRQ	MXC_INT_PCMCIA

/*
 * The mapping of window size to bank size value
 */
static bsize_map_t bsize_map[] = {
	/* Window size  Bank size       */
	{POR_1, POR_BSIZE_1},
	{POR_2, POR_BSIZE_2},
	{POR_4, POR_BSIZE_4},
	{POR_8, POR_BSIZE_8},
	{POR_16, POR_BSIZE_16},
	{POR_32, POR_BSIZE_32},
	{POR_64, POR_BSIZE_64},
	{POR_128, POR_BSIZE_128},
	{POR_256, POR_BSIZE_256},
	{POR_512, POR_BSIZE_512},

	{POR_1K, POR_BSIZE_1K},
	{POR_2K, POR_BSIZE_2K},
	{POR_4K, POR_BSIZE_4K},
	{POR_8K, POR_BSIZE_8K},
	{POR_16K, POR_BSIZE_16K},
	{POR_32K, POR_BSIZE_32K},
	{POR_64K, POR_BSIZE_64K},
	{POR_128K, POR_BSIZE_128K},
	{POR_256K, POR_BSIZE_256K},
	{POR_512K, POR_BSIZE_512K},

	{POR_1M, POR_BSIZE_1M},
	{POR_2M, POR_BSIZE_2M},
	{POR_4M, POR_BSIZE_4M},
	{POR_8M, POR_BSIZE_8M},
	{POR_16M, POR_BSIZE_16M},
	{POR_32M, POR_BSIZE_32M},
	{POR_64M, POR_BSIZE_64M}
};

#define to_mx31ads_pcmcia_socket(x)	container_of(x, struct mx31ads_pcmcia_socket, socket)

/* mx31ads_pcmcia_find_bsize()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Find the bsize according to the window size passed in
 *
 * Return:
 */
static int mx31ads_pcmcia_find_bsize(unsigned long win_size)
{
	int i, nr = sizeof(bsize_map) / sizeof(bsize_map_t);
	int bsize = -1;

	for (i = 0; i < nr; i++) {
		if (bsize_map[i].win_size == win_size) {
			bsize = bsize_map[i].bsize;
			break;
		}
	}

	pr_debug(KERN_INFO "nr = %d bsize = 0x%0x\n", nr, bsize);
	if (bsize < 0 || i > nr) {
		pr_debug(KERN_INFO "No such bsize\n");
		return -ENODEV;
	}

	return bsize;
}

/* mx31ads_common_pcmcia_sock_init()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * (Re-)Initialise the socket, turning on status interrupts
 * and PCMCIA bus.  This must wait for power to stabilise
 * so that the card status signals report correctly.
 *
 * Returns: 0
 */
static int mx31ads_common_pcmcia_sock_init(struct pcmcia_socket *sock)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);

	pr_debug(KERN_INFO "initializing socket\n");

	skt->ops->socket_init(skt);
	return 0;
}

/*
 * mx31ads_common_pcmcia_config_skt
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Convert PCMCIA socket state to our socket configure structure.
 */
static int
mx31ads_common_pcmcia_config_skt(struct mx31ads_pcmcia_socket *skt,
				 socket_state_t *state)
{
	int ret;

	ret = skt->ops->configure_socket(skt, state);
	if (ret == 0) {
		/*
		 * This really needs a better solution.  The IRQ
		 * may or may not be claimed by the driver.
		 */
		if (skt->irq_state != 1 && state->io_irq) {
			skt->irq_state = 1;
			set_irq_type(skt->irq, IRQF_TRIGGER_FALLING);
		} else if (skt->irq_state == 1 && state->io_irq == 0) {
			skt->irq_state = 0;
			set_irq_type(skt->irq, IRQF_TRIGGER_RISING);
		}

		skt->cs_state = *state;
	}

	if (ret < 0)
		pr_debug(KERN_ERR "mx31ads_common_pcmcia: unable to configure"
			 " socket\n");

	return ret;
}

/*
 * mx31ads_common_pcmcia_suspend()
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Remove power on the socket, disable IRQs from the card.
 * Turn off status interrupts, and disable the PCMCIA bus.
 *
 * Returns: 0
 */
static int mx31ads_common_pcmcia_suspend(struct pcmcia_socket *sock)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);
	int ret;

	pr_debug(KERN_INFO "suspending socket\n");

	ret = mx31ads_common_pcmcia_config_skt(skt, &dead_socket);
	if (ret == 0)
		skt->ops->socket_suspend(skt);

	return ret;
}

static unsigned int mx31ads_common_pcmcia_skt_state(struct mx31ads_pcmcia_socket
						    *skt)
{
	struct pcmcia_state state;
	unsigned int stat;

	memset(&state, 0, sizeof(struct pcmcia_state));

	skt->ops->socket_state(skt, &state);

	stat = state.detect ? SS_DETECT : 0;
	stat |= state.ready ? SS_READY : 0;
	stat |= state.wrprot ? SS_WRPROT : 0;
	stat |= state.vs_3v ? SS_3VCARD : 0;
	stat |= state.vs_Xv ? SS_XVCARD : 0;

	/* The power status of individual sockets is not available
	 * explicitly from the hardware, so we just remember the state
	 * and regurgitate it upon request:
	 */
	stat |= skt->cs_state.Vcc ? SS_POWERON : 0;

	if (skt->cs_state.flags & SS_IOCARD)
		stat |= state.bvd1 ? SS_STSCHG : 0;
	else {
		if (state.bvd1 == 0)
			stat |= SS_BATDEAD;
		else if (state.bvd2 == 0)
			stat |= SS_BATWARN;
	}

	pr_debug(KERN_INFO "stat = 0x%08x\n", stat);

	return stat;
}

/*
 *  Implements the get_status() operation for the in-kernel PCMCIA
 * service (formerly SS_GetStatus in Card Services). Essentially just
 * fills in bits in `status' according to internal driver state or
 * the value of the voltage detect chipselect register.
 *
 * As a debugging note, during card startup, the PCMCIA core issues
 * three set_socket() commands in a row the first with RESET deasserted,
 * the second with RESET asserted, and the last with RESET deasserted
 * again. Following the third set_socket(), a get_status() command will
 * be issued. The kernel is looking for the SS_READY flag (see
 * setup_socket(), reset_socket(), and unreset_socket() in cs.c).
 *
 * Returns: 0
 */
static int mx31ads_common_pcmcia_get_status(struct pcmcia_socket *sock,
					    unsigned int *status)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);

	skt->status = mx31ads_common_pcmcia_skt_state(skt);
	*status = skt->status;

	return 0;
}

/*
 * Implements the set_socket() operation for the in-kernel PCMCIA
 * service (formerly SS_SetSocket in Card Services). We more or
 * less punt all of this work and let the kernel handle the details
 * of power configuration, reset, &c. We also record the value of
 * `state' in order to regurgitate it to the PCMCIA core later.
 *
 * Returns: 0
 */
static int mx31ads_common_pcmcia_set_socket(struct pcmcia_socket *sock,
					    socket_state_t *state)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);

	pr_debug(KERN_INFO
		 "mask: %s%s%s%s%s%sflags: %s%s%s%s%s%sVcc %d Vpp %d irq %d\n",
		 (state->csc_mask == 0) ? "<NONE> " : "",
		 (state->csc_mask & SS_DETECT) ? "DETECT " : "",
		 (state->csc_mask & SS_READY) ? "READY " : "",
		 (state->csc_mask & SS_BATDEAD) ? "BATDEAD " : "",
		 (state->csc_mask & SS_BATWARN) ? "BATWARN " : "",
		 (state->csc_mask & SS_STSCHG) ? "STSCHG " : "",
		 (state->flags == 0) ? "<NONE> " : "",
		 (state->flags & SS_PWR_AUTO) ? "PWR_AUTO " : "",
		 (state->flags & SS_IOCARD) ? "IOCARD " : "",
		 (state->flags & SS_RESET) ? "RESET " : "",
		 (state->flags & SS_SPKR_ENA) ? "SPKR_ENA " : "",
		 (state->flags & SS_OUTPUT_ENA) ? "OUTPUT_ENA " : "",
		 state->Vcc, state->Vpp, state->io_irq);

	pr_debug(KERN_INFO
		 "csc_mask: %08x flags: %08x Vcc: %d Vpp: %d io_irq: %d\n",
		 state->csc_mask, state->flags, state->Vcc, state->Vpp,
		 state->io_irq);

	return mx31ads_common_pcmcia_config_skt(skt, state);
}

/*
 * Set address and profile to window registers PBR, POR, POFR
 */
static int mx31ads_pcmcia_set_window_reg(ulong start, ulong end, u_int window)
{
	int bsize;
	ulong size = end - start + 1;

	bsize = mx31ads_pcmcia_find_bsize(size);
	if (bsize < 0) {
		pr_debug("Cannot set the window register\n");
		return -1;
	}
	/* Disable the window */
	_reg_PCMCIA_POR(window) &= ~PCMCIA_POR_PV;

	/* Set PBR, POR, POFR */
	_reg_PCMCIA_PBR(window) = start;
	_reg_PCMCIA_POR(window) &= ~(PCMCIA_POR_PRS_MASK
				     | PCMCIA_POR_WPEN
				     | PCMCIA_POR_WP
				     | PCMCIA_POR_BSIZE_MASK
				     | PCMCIA_POR_PPS_8);
	_reg_PCMCIA_POR(window) |= bsize | PCMCIA_POR_PPS_16;

	switch (window) {
	case IO_WINDOW:
		_reg_PCMCIA_POR(window) |= PCMCIA_POR_PRS(PCMCIA_POR_PRS_IO);
		break;

	case ATTRIBUTE_MEMORY_WINDOW:
		_reg_PCMCIA_POR(window) |=
		    PCMCIA_POR_PRS(PCMCIA_POR_PRS_ATTRIBUTE);
		break;

	case COMMON_MEMORY_WINDOW:
		_reg_PCMCIA_POR(window) |=
		    PCMCIA_POR_PRS(PCMCIA_POR_PRS_COMMON);
		break;

	default:
		pr_debug("Window %d is not support\n", window);
		return -1;
	}
	_reg_PCMCIA_POFR(window) = 0;

	/* Enable the window */
	_reg_PCMCIA_POR(window) |= PCMCIA_POR_PV;

	return 0;
}

/*
 * Implements the set_io_map() operation for the in-kernel PCMCIA
 * service (formerly SS_SetIOMap in Card Services). We configure
 * the map speed as requested, but override the address ranges
 * supplied by Card Services.
 *
 * Returns: 0 on success, -1 on error
 */
static int
mx31ads_common_pcmcia_set_io_map(struct pcmcia_socket *sock,
				 struct pccard_io_map *map)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);
	unsigned short speed = map->speed;

	pr_debug("map %u  speed %u start 0x%08x stop 0x%08x\n",
		 map->map, map->speed, map->start, map->stop);
	pr_debug("flags: %s%s%s%s%s%s%s%s\n",
		 (map->flags == 0) ? "<NONE>" : "",
		 (map->flags & MAP_ACTIVE) ? "ACTIVE " : "",
		 (map->flags & MAP_16BIT) ? "16BIT " : "",
		 (map->flags & MAP_AUTOSZ) ? "AUTOSZ " : "",
		 (map->flags & MAP_0WS) ? "0WS " : "",
		 (map->flags & MAP_WRPROT) ? "WRPROT " : "",
		 (map->flags & MAP_USE_WAIT) ? "USE_WAIT " : "",
		 (map->flags & MAP_PREFETCH) ? "PREFETCH " : "");

	if (map->map >= MAX_IO_WIN) {
		pr_debug(KERN_ERR "%s(): map (%d) out of range\n", __FUNCTION__,
			 map->map);
		return -1;
	}

	if (map->flags & MAP_ACTIVE) {
		if (speed == 0)
			speed = PCMCIA_IO_ACCESS;
	} else {
		speed = 0;
	}

	skt->spd_io[map->map] = speed;
	skt->ops->set_timing(skt);

	if (map->stop == 1)
		map->stop = PAGE_SIZE - 1;

	skt->socket.io_offset = (unsigned long)skt->virt_io;
	map->stop -= map->start;
	map->stop += (unsigned long)skt->virt_io;
	map->start = (unsigned long)skt->virt_io;

	mx31ads_pcmcia_set_window_reg(skt->res_io.start, skt->res_io.end,
				      IO_WINDOW);

	pr_debug(KERN_ERR "IO window: _reg_PCMCIA_PBR(%d) = %08x\n",
		 IO_WINDOW, _reg_PCMCIA_PBR(IO_WINDOW));
	pr_debug(KERN_ERR "IO window: _reg_PCMCIA_POR(%d) = %08x\n",
		 IO_WINDOW, _reg_PCMCIA_POR(IO_WINDOW));

	return 0;
}

/*
 * Implements the set_mem_map() operation for the in-kernel PCMCIA
 * service (formerly SS_SetMemMap in Card Services). We configure
 * the map speed as requested, but override the address ranges
 * supplied by Card Services.
 *
 * Returns: 0 on success, -1 on error
 */
static int
mx31ads_common_pcmcia_set_mem_map(struct pcmcia_socket *sock,
				  struct pccard_mem_map *map)
{
	struct mx31ads_pcmcia_socket *skt = to_mx31ads_pcmcia_socket(sock);
	struct resource *res;
	unsigned short speed = map->speed;

	pr_debug
	    (KERN_INFO
	     "map %u speed %u card_start %08x flags%08x static_start %08lx\n",
	     map->map, map->speed, map->card_start, map->flags,
	     map->static_start);
	pr_debug(KERN_INFO "flags: %s%s%s%s%s%s%s%s\n",
		 (map->flags == 0) ? "<NONE>" : "",
		 (map->flags & MAP_ACTIVE) ? "ACTIVE " : "",
		 (map->flags & MAP_16BIT) ? "16BIT " : "",
		 (map->flags & MAP_AUTOSZ) ? "AUTOSZ " : "",
		 (map->flags & MAP_0WS) ? "0WS " : "",
		 (map->flags & MAP_WRPROT) ? "WRPROT " : "",
		 (map->flags & MAP_ATTRIB) ? "ATTRIB " : "",
		 (map->flags & MAP_USE_WAIT) ? "USE_WAIT " : "");

	if (map->map >= MAX_WIN)
		return -EINVAL;

	if (map->flags & MAP_ACTIVE) {
		if (speed == 0)
			speed = 300;
	} else {
		speed = 0;
	}

	if (map->flags & MAP_ATTRIB) {
		res = &skt->res_attr;
		skt->spd_attr[map->map] = speed;
		skt->spd_mem[map->map] = 0;
		mx31ads_pcmcia_set_window_reg(res->start, res->end,
					      ATTRIBUTE_MEMORY_WINDOW);

		pr_debug(KERN_INFO "Attr window: _reg_PCMCIA_PBR(%d) = %08x\n",
			 ATTRIBUTE_MEMORY_WINDOW,
			 _reg_PCMCIA_PBR(ATTRIBUTE_MEMORY_WINDOW));
		pr_debug(KERN_INFO "_reg_PCMCIA_POR(%d) = %08x\n",
			 ATTRIBUTE_MEMORY_WINDOW,
			 _reg_PCMCIA_POR(ATTRIBUTE_MEMORY_WINDOW));

	} else {
		res = &skt->res_mem;
		skt->spd_attr[map->map] = 0;
		skt->spd_mem[map->map] = speed;
		mx31ads_pcmcia_set_window_reg(res->start, res->end,
					      COMMON_MEMORY_WINDOW);

		pr_debug(KERN_INFO "Com window: _reg_PCMCIA_PBR(%d) = %08x\n",
			 COMMON_MEMORY_WINDOW,
			 _reg_PCMCIA_PBR(COMMON_MEMORY_WINDOW));
		pr_debug(KERN_INFO "Com window: _reg_PCMCIA_POR(%d) = %08x\n",
			 COMMON_MEMORY_WINDOW,
			 _reg_PCMCIA_POR(COMMON_MEMORY_WINDOW));
	}

	skt->ops->set_timing(skt);

	map->static_start = res->start + map->card_start;

	return 0;
}

static struct pccard_operations mx31ads_common_pcmcia_operations = {
	.init = mx31ads_common_pcmcia_sock_init,
	.suspend = mx31ads_common_pcmcia_suspend,
	.get_status = mx31ads_common_pcmcia_get_status,
	.set_socket = mx31ads_common_pcmcia_set_socket,
	.set_io_map = mx31ads_common_pcmcia_set_io_map,
	.set_mem_map = mx31ads_common_pcmcia_set_mem_map,
};

/* ============================================================================== */

static inline void mx31ads_pcmcia_irq_config(void)
{
	/* Setup irq */
	_reg_PCMCIA_PER =
	    (PCMCIA_PER_RDYLE | PCMCIA_PER_CDE1 | PCMCIA_PER_CDE2);
}

static inline void mx31ads_pcmcia_invalidate_windows(void)
{
	int i;

	for (i = 0; i < PCMCIA_WINDOWS; i++) {
		_reg_PCMCIA_PBR(i) = 0;
		_reg_PCMCIA_POR(i) = 0;
		_reg_PCMCIA_POFR(i) = 0;
	}
}

extern void gpio_pcmcia_active(void);
extern void gpio_pcmcia_inactive(void);

static int mx31ads_pcmcia_hw_init(struct mx31ads_pcmcia_socket *skt)
{
	/* Configure the pins for PCMCIA */
	gpio_pcmcia_active();

	/*
	 * enabling interrupts at this time causes a flood of interrupts
	 * if a card is present, so wait for configure_socket
	 * to enable them when requested.
	 *
	 * mx31ads_pcmcia_irq_config();
	 */
	mx31ads_pcmcia_invalidate_windows();

	/* Register interrupt. */
	skt->irq = MX31ADS_PCMCIA_IRQ;

	return 0;
}

static void mx31ads_pcmcia_free_irq(struct mx31ads_pcmcia_socket *skt,
				    unsigned int irq)
{
	free_irq(irq, skt);
}

static void mx31ads_pcmcia_hw_shutdown(struct mx31ads_pcmcia_socket *skt)
{
	mx31ads_pcmcia_invalidate_windows();
	mx31ads_pcmcia_free_irq(skt, MX31ADS_PCMCIA_IRQ);

	/* Disable the pins */
	gpio_pcmcia_inactive();
}

/*
 * Get the socket state
 */
static void
mx31ads_pcmcia_socket_state(struct mx31ads_pcmcia_socket *skt,
			    struct pcmcia_state *state)
{
	unsigned long pins;

	pins = _reg_PCMCIA_PIPR;
	pr_debug(KERN_INFO "_reg_PCMCIA_PIPR = 0x%08lx\n", pins);

	state->ready = (pins & PCMCIA_PIPR_RDY) ? 1 : 0;
	state->bvd2 = (pins & PCMCIA_PIPR_BVD2) ? 1 : 0;
	state->bvd1 = (pins & PCMCIA_PIPR_BVD1) ? 1 : 0;

	if ((pins & PCMCIA_PIPR_CD) == PCMCIA_PIPR_CD) {
		state->detect = 0;
		skt->cs_state.csc_mask |= SS_INSERTION;
	} else {
		state->detect = 1;
	}
	state->detect = (pins & PCMCIA_PIPR_CD) ? 0 : 1;
	state->wrprot = (pins & PCMCIA_PIPR_WP) ? 1 : 0;
	state->poweron = (pins & PCMCIA_PIPR_POWERON) ? 1 : 0;
#if 0
	if ((pins & PCMCIA_PIPR_CD) == PCMCIA_PIPR_CD) {
		state->detect = 0;
		skt->cs_state.csc_mask |= SS_INSERTION;
	} else {
		state->detect = 1;
	}
	if (pins & PCMCIA_PIPR_VS_5V) {
		state->vs_3v = 0;
		skt->cs_state.Vcc = 33;
	} else {
		state->vs_3v = 1;
		skt->cs_state.Vcc = 50;
	}
#endif
	state->vs_3v = (pins & PCMCIA_PIPR_VS_5V) ? 0 : 1;
	state->vs_Xv = 0;
}

static __inline__ void mx31ads_pcmcia_low_power(bool enable)
{
	if (enable)
		_reg_PCMCIA_PGCR |= PCMCIA_PGCR_LPMEN;
	else
		_reg_PCMCIA_PGCR &= ~PCMCIA_PGCR_LPMEN;
}

static __inline__ void mx31ads_pcmcia_soft_reset(void)
{
	_reg_PCMCIA_PGCR |= PCMCIA_PGCR_RESET;
	msleep(2);

	_reg_PCMCIA_PGCR &= ~(PCMCIA_PGCR_RESET | PCMCIA_PGCR_LPMEN);
	_reg_PCMCIA_PGCR |= PCMCIA_PGCR_POE;
	msleep(2);
	pr_debug(KERN_INFO "_reg_PCMCIA_PGCR = %08x\n", _reg_PCMCIA_PGCR);
}

static int
mx31ads_pcmcia_configure_socket(struct mx31ads_pcmcia_socket *skt,
				const socket_state_t *state)
{
	int ret = 0;

	if (state->Vcc != 0 && state->Vcc != 33 && state->Vcc != 50) {
		pr_debug(KERN_ERR "mx31ads-pcmcia: unrecognized Vcc %d\n",
			 state->Vcc);
		return -1;
	}

	pr_debug(KERN_INFO "PIPR = %x, desired Vcc = %d.%dV\n",
		 _reg_PCMCIA_PIPR, state->Vcc / 10, state->Vcc % 10);

	if (!(skt->socket.state & SOCKET_PRESENT) && (skt->pre_stat == 1)) {
		pr_debug(KERN_INFO "Socket enter low power mode\n");
		skt->pre_stat = 0;
		mx31ads_pcmcia_low_power(1);
	}

	if (state->flags & SS_RESET) {
		mx31ads_pcmcia_soft_reset();

		/* clean out previous tenant's trash */
		_reg_PCMCIA_PGSR = (PCMCIA_PGSR_NWINE
				    | PCMCIA_PGSR_LPE
				    | PCMCIA_PGSR_SE
				    | PCMCIA_PGSR_CDE | PCMCIA_PGSR_WPE);
	}
	/* enable interrupts if requested, else turn 'em off */
	if (skt->irq)
		mx31ads_pcmcia_irq_config();
	else
		_reg_PCMCIA_PER = 0;

	if (skt->socket.state & SOCKET_PRESENT) {
		skt->pre_stat = 1;
	}
	return ret;
}

static void mx31ads_pcmcia_enable_irq(struct mx31ads_pcmcia_socket *skt,
				      unsigned int irq)
{
	set_irq_type(irq, IRQF_TRIGGER_RISING);
	set_irq_type(irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
}

static void mx31ads_pcmcia_disable_irq(struct mx31ads_pcmcia_socket *skt,
				       unsigned int irq)
{
	set_irq_type(irq, IRQF_TRIGGER_NONE);
}

/*
 * Enable card status IRQs on (re-)initialisation.  This can
 * be called at initialisation, power management event, or
 * pcmcia event.
 */
static void mx31ads_pcmcia_socket_init(struct mx31ads_pcmcia_socket *skt)
{
	mx31ads_pcmcia_soft_reset();

	mx31ads_pcmcia_enable_irq(skt, MX31ADS_PCMCIA_IRQ);
}

/*
 * Disable card status IRQ on suspend.
 */
static void mx31ads_pcmcia_socket_suspend(struct mx31ads_pcmcia_socket *skt)
{
	mx31ads_pcmcia_disable_irq(skt, MX31ADS_PCMCIA_IRQ);
	mx31ads_pcmcia_low_power(1);
}

/* ==================================================================================== */

/*
 * PCMCIA strobe hold time
 */
static inline u_int mx31ads_pcmcia_por_psht(u_int pcmcia_cycle_ns,
					    u_int hclk_cycle_ns)
{
	u_int psht;

	return psht = pcmcia_cycle_ns / hclk_cycle_ns;
}

/*
 * PCMCIA strobe set up time
 */
static inline u_int mx31ads_pcmcia_por_psst(u_int pcmcia_cycle_ns,
					    u_int hclk_cycle_ns)
{
	u_int psst;

	return psst = pcmcia_cycle_ns / hclk_cycle_ns;
}

/*
 * PCMCIA strobe length time
 */
static inline u_int mx31ads_pcmcia_por_pslt(u_int pcmcia_cycle_ns,
					    u_int hclk_cycle_ns)
{
	u_int pslt;

	return pslt = pcmcia_cycle_ns / hclk_cycle_ns + 2;
}

/*
 * mx31ads_pcmcia_default_mecr_timing
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * Calculate MECR clock wait states for given CPU clock
 * speed and command wait state. This function can be over-
 * written by a board specific version.
 *
 * The default is to simply calculate the BS values as specified in
 * the INTEL SA1100 development manual
 * "Expansion Memory (PCMCIA) Configuration Register (MECR)"
 * that's section 10.2.5 in _my_ version of the manual ;)
 */
static unsigned int mx31ads_pcmcia_default_mecr_timing(struct
						       mx31ads_pcmcia_socket
						       *skt,
						       unsigned int cpu_speed,
						       unsigned int cmd_time)
{
	return 0;
}

/*
 * Calculate the timing code
 */
static u_int mx31ads_pcmcia_cal_code(u_int speed_ns, u_int clk_ns)
{
	u_int code;

	code = PCMCIA_POR_PSHT(mx31ads_pcmcia_por_psht(speed_ns, clk_ns))
	    | PCMCIA_POR_PSST(mx31ads_pcmcia_por_psst(speed_ns, clk_ns))
	    | PCMCIA_POR_PSL(mx31ads_pcmcia_por_pslt(speed_ns, clk_ns));

	return code;
}

/*
 * set MECR value for socket <sock> based on this sockets
 * io, mem and attribute space access speed.
 * Call board specific BS value calculation to allow boards
 * to tweak the BS values.
 */
static int mx31ads_pcmcia_set_window_timing(u_int speed_ns, u_int window,
					    u_int clk_ns)
{
	u_int code = 0;

	switch (window) {
	case IO_WINDOW:
		code = mx31ads_pcmcia_cal_code(speed_ns, clk_ns);
		break;
	case COMMON_MEMORY_WINDOW:
		code = mx31ads_pcmcia_cal_code(speed_ns, clk_ns);
		break;
	case ATTRIBUTE_MEMORY_WINDOW:
		code = mx31ads_pcmcia_cal_code(speed_ns, clk_ns);
		break;
	default:
		break;
	}

	/* Disable the window */
	_reg_PCMCIA_POR(window) &= ~PCMCIA_POR_PV;

	/* Clear the register fisrt */
	_reg_PCMCIA_POR(window) &= ~(PCMCIA_POR_PSST_MASK
				     | PCMCIA_POR_PSL_MASK
				     | PCMCIA_POR_PSHT_MASK);
	/* And then set the register */
	_reg_PCMCIA_POR(window) |= code;

	/* Enable the window */
	_reg_PCMCIA_POR(window) |= PCMCIA_POR_PV;

	return 0;
}

static unsigned short calc_speed(unsigned short *spds, int num,
				 unsigned short dflt)
{
	unsigned short speed = 0;
	int i;

	for (i = 0; i < num; i++)
		if (speed < spds[i])
			speed = spds[i];
	if (speed == 0)
		speed = dflt;

	return speed;
}

static void
mx31ads_common_pcmcia_get_timing(struct mx31ads_pcmcia_socket *skt,
				 struct mx31ads_pcmcia_timing *timing)
{
	timing->io = calc_speed(skt->spd_io, MAX_IO_WIN, PCMCIA_IO_ACCESS);
	timing->mem = calc_speed(skt->spd_mem, MAX_WIN, PCMCIA_3V_MEM_ACCESS);
	timing->attr =
	    calc_speed(skt->spd_attr, MAX_WIN, PCMCIA_ATTR_MEM_ACCESS);
}

static int mx31ads_pcmcia_set_timing(struct mx31ads_pcmcia_socket *skt)
{
	u_int clk_ns;
	struct mx31ads_pcmcia_timing timing;

	/* How many nanoseconds */
	clk_ns = (1000 * 1000 * 1000) / clk_get_rate(skt->clk);
	pr_debug(KERN_INFO "clk_ns = %d\n", clk_ns);

	mx31ads_common_pcmcia_get_timing(skt, &timing);
	pr_debug(KERN_INFO "timing: io %d, mem %d, attr %d\n", timing.io,
		 timing.mem, timing.attr);

	mx31ads_pcmcia_set_window_timing(timing.io, IO_WINDOW, clk_ns);
	mx31ads_pcmcia_set_window_timing(timing.mem, COMMON_MEMORY_WINDOW,
					 clk_ns);
	mx31ads_pcmcia_set_window_timing(timing.attr, ATTRIBUTE_MEMORY_WINDOW,
					 clk_ns);

	return 0;
}

static int mx31ads_pcmcia_show_timing(struct mx31ads_pcmcia_socket *skt,
				      char *buf)
{
	return 0;
}

static struct pcmcia_low_level mx31ads_pcmcia_ops = {
	.owner = THIS_MODULE,
	.hw_init = mx31ads_pcmcia_hw_init,
	.hw_shutdown = mx31ads_pcmcia_hw_shutdown,
	.socket_state = mx31ads_pcmcia_socket_state,
	.configure_socket = mx31ads_pcmcia_configure_socket,

	.socket_init = mx31ads_pcmcia_socket_init,
	.socket_suspend = mx31ads_pcmcia_socket_suspend,

	.get_timing = mx31ads_pcmcia_default_mecr_timing,
	.set_timing = mx31ads_pcmcia_set_timing,
	.show_timing = mx31ads_pcmcia_show_timing,
};

/* =================================================================================== */

LIST_HEAD(mx31ads_pcmcia_sockets);
DECLARE_MUTEX(mx31ads_pcmcia_sockets_lock);

static DEFINE_SPINLOCK(status_lock);

struct bittbl {
	unsigned int mask;
	const char *name;
};

static struct bittbl status_bits[] = {
	{SS_WRPROT, "SS_WRPROT"},
	{SS_BATDEAD, "SS_BATDEAD"},
	{SS_BATWARN, "SS_BATWARN"},
	{SS_READY, "SS_READY"},
	{SS_DETECT, "SS_DETECT"},
	{SS_POWERON, "SS_POWERON"},
	{SS_STSCHG, "SS_STSCHG"},
	{SS_3VCARD, "SS_3VCARD"},
	{SS_XVCARD, "SS_XVCARD"},
};

static struct bittbl conf_bits[] = {
	{SS_PWR_AUTO, "SS_PWR_AUTO"},
	{SS_IOCARD, "SS_IOCARD"},
	{SS_RESET, "SS_RESET"},
	{SS_DMA_MODE, "SS_DMA_MODE"},
	{SS_SPKR_ENA, "SS_SPKR_ENA"},
	{SS_OUTPUT_ENA, "SS_OUTPUT_ENA"},
};

static void
dump_bits(char **p, const char *prefix, unsigned int val, struct bittbl *bits,
	  int sz)
{
	char *b = *p;
	int i;

	b += sprintf(b, "%-9s:", prefix);
	for (i = 0; i < sz; i++)
		if (val & bits[i].mask)
			b += sprintf(b, " %s", bits[i].name);
	*b++ = '\n';
	*p = b;
}

/*
 * Implements the /sys/class/pcmcia_socket/??/status file.
 *
 * Returns: the number of characters added to the buffer
 */
static ssize_t show_status(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mx31ads_pcmcia_socket *skt =
	    container_of(dev, struct mx31ads_pcmcia_socket, socket.dev);
	char *p = buf;

	p += sprintf(p, "slot     : %d\n", skt->nr);

	dump_bits(&p, "status", skt->status,
		  status_bits, ARRAY_SIZE(status_bits));
	dump_bits(&p, "csc_mask", skt->cs_state.csc_mask,
		  status_bits, ARRAY_SIZE(status_bits));
	dump_bits(&p, "cs_flags", skt->cs_state.flags,
		  conf_bits, ARRAY_SIZE(conf_bits));

	p += sprintf(p, "Vcc      : %d\n", skt->cs_state.Vcc);
	p += sprintf(p, "Vpp      : %d\n", skt->cs_state.Vpp);
	p += sprintf(p, "IRQ      : %d (%d)\n", skt->cs_state.io_irq, skt->irq);
	if (skt->ops->show_timing)
		p += skt->ops->show_timing(skt, p);

	return p - buf;
}

static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);

static void mx31ads_common_check_status(struct mx31ads_pcmcia_socket *skt)
{
	unsigned int events;

	pr_debug(KERN_INFO "entering PCMCIA monitoring thread\n");

	do {
		unsigned int status;
		unsigned long flags;

		status = mx31ads_common_pcmcia_skt_state(skt);

		spin_lock_irqsave(&status_lock, flags);
		events = (status ^ skt->status) & skt->cs_state.csc_mask;
		skt->status = status;
		spin_unlock_irqrestore(&status_lock, flags);

		pr_debug(KERN_INFO "events: %s%s%s%s%s%s\n",
			 events == 0 ? "<NONE>" : "",
			 events & SS_DETECT ? "DETECT " : "",
			 events & SS_READY ? "READY " : "",
			 events & SS_BATDEAD ? "BATDEAD " : "",
			 events & SS_BATWARN ? "BATWARN " : "",
			 events & SS_STSCHG ? "STSCHG " : "");

		if (events)
			pcmcia_parse_events(&skt->socket, events);
	} while (events);
}

/*
 * Service routine for socket driver interrupts (requested by the
 * low-level PCMCIA init() operation via mx31ads_common_pcmcia_thread()).
 * The actual interrupt-servicing work is performed by
 * mx31ads_common_pcmcia_thread(), largely because the Card Services event-
 * handling code performs scheduling operations which cannot be
 * executed from within an interrupt context.
 */
static irqreturn_t mx31ads_common_pcmcia_interrupt(int irq, void *dev)
{
	struct mx31ads_pcmcia_socket *skt = dev;
	volatile u32 pscr, pgsr;

	dev_dbg(dev, "servicing IRQ %d\n", irq);

	/* clear interrupt states */
	pscr = _reg_PCMCIA_PSCR;
	_reg_PCMCIA_PSCR = pscr;

	pgsr = _reg_PCMCIA_PGSR;
	_reg_PCMCIA_PGSR = pgsr;

	mx31ads_common_check_status(skt);

	return IRQ_HANDLED;
}

/* Let's poll for events in addition to IRQs since IRQ only is unreliable... */
static void mx31ads_common_pcmcia_poll_event(unsigned long dummy)
{
	struct mx31ads_pcmcia_socket *skt =
	    (struct mx31ads_pcmcia_socket *)dummy;
	pr_debug(KERN_INFO "polling for events\n");

	mod_timer(&skt->poll_timer, jiffies + PCMCIA_POLL_PERIOD);

	mx31ads_common_check_status(skt);
}

#define mx31ads_pcmcia_cpufreq_register()
#define mx31ads_pcmcia_cpufreq_unregister()

static int mx31ads_common_drv_pcmcia_probe(struct platform_device *pdev,
					   struct pcmcia_low_level *ops)
{
	struct mx31ads_pcmcia_socket *skt;
	int vs, value, ret;
	struct pccard_io_map map;

	down(&mx31ads_pcmcia_sockets_lock);

	skt = kzalloc(sizeof(struct mx31ads_pcmcia_socket), GFP_KERNEL);
	if (!skt) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * Initialise the socket structure.
	 */
	skt->socket.ops = &mx31ads_common_pcmcia_operations;
	skt->socket.owner = ops->owner;
	skt->socket.driver_data = skt;

	init_timer(&skt->poll_timer);
	skt->poll_timer.function = mx31ads_common_pcmcia_poll_event;
	skt->poll_timer.data = (unsigned long)skt;
	skt->poll_timer.expires = jiffies + PCMCIA_POLL_PERIOD;

	skt->irq = MX31ADS_PCMCIA_IRQ;
	skt->socket.dev.parent = &pdev->dev;
	skt->ops = ops;

	skt->clk = clk_get(NULL, "ahb_clk");

	skt->res_skt.start = _PCMCIA(0);
	skt->res_skt.end = _PCMCIA(0) + PCMCIASp - 1;
	skt->res_skt.name = MX31ADS_PCMCIA;
	skt->res_skt.flags = IORESOURCE_MEM;

	ret = request_resource(&iomem_resource, &skt->res_skt);
	if (ret)
		goto out_err_1;

	skt->res_io.start = _PCMCIAIO(0);
	skt->res_io.end = _PCMCIAIO(0) + PCMCIAIOSp - 1;
	skt->res_io.name = "io";
	skt->res_io.flags = IORESOURCE_MEM | IORESOURCE_BUSY;

	ret = request_resource(&skt->res_skt, &skt->res_io);
	if (ret)
		goto out_err_2;

	skt->res_mem.start = _PCMCIAMem(0);
	skt->res_mem.end = _PCMCIAMem(0) + PCMCIAMemSp - 1;
	skt->res_mem.name = "memory";
	skt->res_mem.flags = IORESOURCE_MEM;

	ret = request_resource(&skt->res_skt, &skt->res_mem);
	if (ret)
		goto out_err_3;

	skt->res_attr.start = _PCMCIAAttr(0);
	skt->res_attr.end = _PCMCIAAttr(0) + PCMCIAAttrSp - 1;
	skt->res_attr.name = "attribute";
	skt->res_attr.flags = IORESOURCE_MEM;

	ret = request_resource(&skt->res_skt, &skt->res_attr);
	if (ret)
		goto out_err_4;

	skt->virt_io = ioremap(skt->res_io.start, 0x10000);
	if (skt->virt_io == NULL) {
		ret = -ENOMEM;
		goto out_err_5;
	}

	if (list_empty(&mx31ads_pcmcia_sockets))
		mx31ads_pcmcia_cpufreq_register();

	list_add(&skt->node, &mx31ads_pcmcia_sockets);

	/*
	 * We initialize default socket timing here, because
	 * we are not guaranteed to see a SetIOMap operation at
	 * runtime.
	 */
	ops->set_timing(skt);

	ret = ops->hw_init(skt);
	if (ret)
		goto out_err_6;

	ret = request_irq(skt->irq, mx31ads_common_pcmcia_interrupt,
			  IRQF_SHARED | IRQF_DISABLED, "PCMCIA IRQ", skt);
	if (ret)
		goto out_err_6;

	skt->socket.features = SS_CAP_STATIC_MAP | SS_CAP_PCCARD;
	skt->socket.resource_ops = &pccard_static_ops;
	skt->socket.irq_mask = 0;
	skt->socket.map_size = PCMCIAPrtSp;
	skt->socket.pci_irq = skt->irq;
	skt->socket.io_offset = (unsigned long)skt->virt_io;

	skt->status = mx31ads_common_pcmcia_skt_state(skt);
	skt->pre_stat = 0;
	ret = pcmcia_register_socket(&skt->socket);
	if (ret)
		goto out_err_7;
	/* FIXED ME  workaround for binding with ide-cs. ide usage io port 0x100~0x107 and 0x10e */
	map.map = 0;
	map.flags = MAP_ACTIVE | MAP_16BIT;
	map.start = 0;
	map.stop = PCMCIAIOSp - 1;
	map.speed = 0;
	mx31ads_common_pcmcia_set_io_map(&skt->socket, &map);

	vs = _reg_PCMCIA_PIPR & PCMCIA_PIPR_VS;
	value = vs & PCMCIA_PIPR_VS_5V ? 50 : 33;
	dev_dbg(&pdev->dev, "PCMCIA: Voltage the card supports: %d.%dV\n",
		value / 10, value % 10);

	add_timer(&skt->poll_timer);

	ret = device_create_file(&skt->socket.dev, &dev_attr_status);
	if (ret < 0)
		goto out_err_8;

	platform_set_drvdata(pdev, skt);
	ret = 0;
	goto out;

      out_err_8:
	del_timer_sync(&skt->poll_timer);
	pcmcia_unregister_socket(&skt->socket);

      out_err_7:
	flush_scheduled_work();
	free_irq(skt->irq, skt);
	ops->hw_shutdown(skt);
      out_err_6:
	list_del(&skt->node);
	iounmap(skt->virt_io);
      out_err_5:
	release_resource(&skt->res_attr);
      out_err_4:
	release_resource(&skt->res_mem);
      out_err_3:
	release_resource(&skt->res_io);
      out_err_2:
	release_resource(&skt->res_skt);
      out_err_1:

	kfree(skt);
      out:
	up(&mx31ads_pcmcia_sockets_lock);
	return ret;
}

static int mx31ads_drv_pcmcia_remove(struct platform_device *pdev)
{
	struct mx31ads_pcmcia_socket *skt = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	down(&mx31ads_pcmcia_sockets_lock);

	del_timer_sync(&skt->poll_timer);

	pcmcia_unregister_socket(&skt->socket);

	flush_scheduled_work();

	skt->ops->hw_shutdown(skt);

	mx31ads_common_pcmcia_config_skt(skt, &dead_socket);

	list_del(&skt->node);
	iounmap(skt->virt_io);
	skt->virt_io = NULL;
	release_resource(&skt->res_attr);
	release_resource(&skt->res_mem);
	release_resource(&skt->res_io);
	release_resource(&skt->res_skt);

	if (list_empty(&mx31ads_pcmcia_sockets))
		mx31ads_pcmcia_cpufreq_unregister();

	up(&mx31ads_pcmcia_sockets_lock);

	kfree(skt);

	return 0;
}

static int mx31ads_drv_pcmcia_probe(struct platform_device *pdev)
{
	if (!machine_is_mx31ads())
		return -ENODEV;

	return mx31ads_common_drv_pcmcia_probe(pdev, &mx31ads_pcmcia_ops);
}

static int mx31ads_drv_pcmcia_suspend(struct platform_device *pdev,
				      pm_message_t state)
{
	return pcmcia_socket_dev_suspend(&pdev->dev, state);
}

static int mx31ads_drv_pcmcia_resume(struct platform_device *pdev)
{
	return pcmcia_socket_dev_resume(&pdev->dev);
}

/*
 * Low level functions
 */
static struct platform_driver mx31ads_pcmcia_driver = {
	.driver = {
		   .name = MX31ADS_PCMCIA,
		   },
	.probe = mx31ads_drv_pcmcia_probe,
	.remove = mx31ads_drv_pcmcia_remove,
	.suspend = mx31ads_drv_pcmcia_suspend,
	.resume = mx31ads_drv_pcmcia_resume,
};

/* mx31ads_pcmcia_init()
 *
 */
static int __init mx31ads_pcmcia_init(void)
{
	int ret;

	ret = platform_driver_register(&mx31ads_pcmcia_driver);
	if (ret)
		return ret;
	pr_debug(KERN_INFO "PCMCIA: Initialize i.Mx31 pcmcia socket\n");

	return ret;
}

/* mx31ads_pcmcia_exit()
 *
 */
static void __exit mx31ads_pcmcia_exit(void)
{
	platform_driver_unregister(&mx31ads_pcmcia_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX31 PCMCIA Socket Controller");
MODULE_LICENSE("GPL");

module_init(mx31ads_pcmcia_init);
module_exit(mx31ads_pcmcia_exit);
