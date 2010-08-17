/*
 * linux/drivers/pcmcia/mx31ads-pcmcia.h
 *
 * Copyright 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This file contains definitions for the PCMCIA support code common to
 * integrated SOCs like the i.Mx31 microprocessors.
 */
#ifndef _ASM_ARCH_PCMCIA
#define _ASM_ARCH_PCMCIA

/* include the world */
#include <linux/cpufreq.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/cistpl.h>
#include "cs_internal.h"

#define	MX31ADS_PCMCIA	"Mx31ads_pcmcia_socket"

struct device;
struct pcmcia_low_level;

/*
 * This structure encapsulates per-socket state which we might need to
 * use when responding to a Card Services query of some kind.
 */
struct mx31ads_pcmcia_socket {
	struct pcmcia_socket socket;

	/*
	 * Info from low level handler
	 */
	struct device *dev;
	unsigned int nr;
	unsigned int irq;

	struct clk *clk;

	/*
	 * Core PCMCIA state
	 */
	struct pcmcia_low_level *ops;

	unsigned int status;
	unsigned int pre_stat;
	socket_state_t cs_state;

	unsigned short spd_io[MAX_IO_WIN];
	unsigned short spd_mem[MAX_WIN];
	unsigned short spd_attr[MAX_WIN];

	struct resource res_skt;
	struct resource res_io;
	struct resource res_mem;
	struct resource res_attr;
	void *virt_io;

	unsigned int irq_state;

	struct timer_list poll_timer;
	struct list_head node;
};

struct pcmcia_state {
	unsigned detect:1,
	    ready:1, bvd1:1, bvd2:1, wrprot:1, vs_3v:1, vs_Xv:1, poweron:1;
};

struct pcmcia_low_level {
	struct module *owner;

	/* first socket in system */
	int first;
	/* nr of sockets */
	int nr;

	int (*hw_init) (struct mx31ads_pcmcia_socket *);
	void (*hw_shutdown) (struct mx31ads_pcmcia_socket *);

	void (*socket_state) (struct mx31ads_pcmcia_socket *,
			      struct pcmcia_state *);
	int (*configure_socket) (struct mx31ads_pcmcia_socket *,
				 const socket_state_t *);

	/*
	 * Enable card status IRQs on (re-)initialisation.  This can
	 * be called at initialisation, power management event, or
	 * pcmcia event.
	 */
	void (*socket_init) (struct mx31ads_pcmcia_socket *);

	/*
	 * Disable card status IRQs and PCMCIA bus on suspend.
	 */
	void (*socket_suspend) (struct mx31ads_pcmcia_socket *);

	/*
	 * Hardware specific timing routines.
	 * If provided, the get_timing routine overrides the SOC default.
	 */
	unsigned int (*get_timing) (struct mx31ads_pcmcia_socket *,
				    unsigned int, unsigned int);
	int (*set_timing) (struct mx31ads_pcmcia_socket *);
	int (*show_timing) (struct mx31ads_pcmcia_socket *, char *);

#ifdef CONFIG_CPU_FREQ
	/*
	 * CPUFREQ support.
	 */
	int (*frequency_change) (struct mx31ads_pcmcia_socket *, unsigned long,
				 struct cpufreq_freqs *);
#endif
};

struct mx31ads_pcmcia_timing {
	unsigned short io;
	unsigned short mem;
	unsigned short attr;
};

typedef struct {
	ulong win_size;
	int bsize;
} bsize_map_t;

/*
 * The PC Card Standard, Release 7, section 4.13.4, says that twIORD
 * has a minimum value of 165ns. Section 4.13.5 says that twIOWR has
 * a minimum value of 165ns, as well. Section 4.7.2 (describing
 * common and attribute memory write timing) says that twWE has a
 * minimum value of 150ns for a 250ns cycle time (for 5V operation;
 * see section 4.7.4), or 300ns for a 600ns cycle time (for 3.3V
 * operation, also section 4.7.4). Section 4.7.3 says that taOE
 * has a maximum value of 150ns for a 300ns cycle time (for 5V
 * operation), or 300ns for a 600ns cycle time (for 3.3V operation).
 *
 * When configuring memory maps, Card Services appears to adopt the policy
 * that a memory access time of "0" means "use the default." The default
 * PCMCIA I/O command width time is 165ns. The default PCMCIA 5V attribute
 * and memory command width time is 150ns; the PCMCIA 3.3V attribute and
 * memory command width time is 300ns.
 */
#define PCMCIA_IO_ACCESS		(165)
#define PCMCIA_5V_MEM_ACCESS	(150)
#define PCMCIA_3V_MEM_ACCESS	(300)
#define PCMCIA_ATTR_MEM_ACCESS	(300)

/*
 * The socket driver actually works nicely in interrupt-driven form,
 * so the (relatively infrequent) polling is "just to be sure."
 */
#define PCMCIA_POLL_PERIOD    (2*HZ)
#endif
