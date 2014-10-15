/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx6_mmdc.c
 *
 * @brief MX6 MMDC specific file.
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <asm/fncpy.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/iram_alloc.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/hardware/gic.h>
#include "crm_regs.h"


/* DDR settings */
unsigned long (*iram_ddr_settings)[2];
unsigned long (*normal_mmdc_settings)[2];
unsigned long (*iram_iomux_settings)[2];
void __iomem *mmdc_base;
void __iomem *iomux_base;
void __iomem *gic_dist_base;
void __iomem *gic_cpu_base;
void __iomem *imx_scu_base;

void (*mx6_change_ddr_freq)(u32 freq, void *ddr_settings, bool dll_mode, void* iomux_offsets) = NULL;

void (*mx6sl_lpddr2_change_freq)(u32 freq, int low_bus_freq_mode,
								void *ddr_settings) = NULL;

void (*mx6sl_ddr3_change_freq)(u32 freq, void *ddr_settings, bool dll_mode, void* iomux_offsets, int low_bus_freq_mode) = NULL;

extern unsigned int ddr_low_rate;
extern unsigned int ddr_med_rate;
extern unsigned int ddr_normal_rate;
extern int low_bus_freq_mode;
extern int audio_bus_freq_mode;
extern int mmdc_med_rate;
extern unsigned long iram_tlb_phys_addr;
extern void __iomem *ccm_base;
extern void mx6_ddr_freq_change(u32 freq, void *ddr_settings, bool dll_mode, void *iomux_offsets);
extern void mx6sl_ddr_iram(int ddr_freq, int low_bus_freq_mode, void *ddr_settings);
extern void mx6sl_ddr3_freq_change(u32 freq, void *ddr_settings, bool dll_mode, void *iomux_offsets, int low_bus_freq_mode);
extern unsigned long save_ttbr1(void);
extern void restore_ttbr1(u32 ttbr1);

extern unsigned long mx6_ddr3_iram_end asm("mx6_ddr3_iram_end");
extern unsigned long mx6_ddr3_iram_start asm("mx6_ddr3_iram_start");
extern unsigned long mx6sl_lpddr2_iram_end asm("mx6sl_lpddr2_iram_end");
extern unsigned long mx6sl_lpddr2_iram_start asm("mx6sl_lpddr2_iram_start");
extern unsigned long mx6sl_ddr3_iram_end asm("mx6sl_ddr3_iram_end");
extern unsigned long mx6sl_ddr3_iram_start asm("mx6sl_ddr3_iram_start");
#ifdef CONFIG_SMP
extern unsigned long wfe_ddr3_freq_change_start asm("wfe_ddr3_freq_change_start");
extern unsigned long wfe_ddr3_freq_change_end asm("wfe_ddr3_freq_change_end");
void (*wfe_change_ddr_freq)(u32 cpuid, u32 *ddr_freq_change_done);
extern void wfe_ddr3_freq_change(u32 cpuid, u32 *ddr_freq_change_done);
u32 *wait_for_ddr_freq_update;
static unsigned long wfe_freq_change_iram_base;
static volatile unsigned int cpus_in_wfe;
#endif

unsigned long ddr_freq_change_iram_phys_addr;

static void *ddr_freq_change_iram_base;
static int ddr_settings_size;
static int iomux_settings_size;
static int curr_ddr_rate;
static unsigned int ddr_type;

#define MIN_DLL_ON_FREQ		333000000
#define MAX_DLL_OFF_FREQ		125000000

unsigned long ddr3_dll_mx6q[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x1C, 0x04088032},
	{0x1C, 0x0408803a},
	{0x1C, 0x08408030},
	{0x1C, 0x08408038},
	{0x818, 0x0},
};

unsigned long ddr3_calibration[][2] = {
	{0x83c, 0x0},
	{0x840, 0x0},
	{0x483c, 0x0},
	{0x4840, 0x0},
	{0x848, 0x0},
	{0x4848, 0x0},
	{0x850, 0x0},
	{0x4850, 0x0},
};

unsigned long ddr3_dll_mx6dl[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x1C, 0x04008032},
	{0x1C, 0x0400803a},
	{0x1C, 0x07208030},
	{0x1C, 0x07208038},
	{0x818, 0x0},
};

unsigned long ddr3_dll_mx6sl[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x30, 0x0},
	{0x1C, 0x04008032},
	{0x1C, 0x0400803a},
	{0x1C, 0x00048031},
	{0x1C, 0x00048039},
	{0x1C, 0x05208030},
	{0x1C, 0x05208038},
	{0x1C, 0x04008040},
	{0x1C, 0x04008048},
	{0x818, 0x0},
};

unsigned long ddr3_calibration_mx6sl[][2] = {
	{0x83c, 0x0},
	{0x840, 0x0},
	{0x848, 0x0},
	{0x850, 0x0},
};

unsigned long iomux_offsets_mx6q[][2] = {
	{0x5A8, 0x0},
	{0x5B0, 0x0},
	{0x524, 0x0},
	{0x51C, 0x0},
	{0x518, 0x0},
	{0x50C, 0x0},
	{0x5B8, 0x0},
	{0x5C0, 0x0},
};

unsigned long iomux_offsets_mx6dl[][2] = {
	{0x4BC, 0x0},
	{0x4C0, 0x0},
	{0x4C4, 0x0},
	{0x4C8, 0x0},
	{0x4CC, 0x0},
	{0x4D0, 0x0},
	{0x4D4, 0x0},
	{0x4D8, 0x0},
};

unsigned long iomux_offsets_mx6sl[][2] = {
	{0x344, 0x0},
	{0x348, 0x0},
	{0x34c, 0x0},
	{0x350, 0x0},
};

unsigned long ddr3_400[][2] = {
	{0x83c, 0x42490249},
	{0x840, 0x02470247},
	{0x483c, 0x42570257},
	{0x4840, 0x02400240},
	{0x848, 0x4039363C},
	{0x4848, 0x3A39333F},
	{0x850, 0x38414441},
	{0x4850, 0x472D4833}
};


unsigned long lpddr2_400M_6sl[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x14, 0x0},
	{0x38, 0x0},
};


unsigned long lpddr2_100M_6sl[][2] = {
	{0x0c, 0x0C0D2073},
	{0x10, 0x00040482},
	{0x14, 0x00000049},
	{0x38, 0x00060222},
};


unsigned long lpddr2_24M_6sl[][2] = {
	{0x0c, 0x03032073},
	{0x10, 0x00200482},
	{0x14, 0x00000049},
	{0x38, 0x00040222},
};

unsigned long *irq_used;

unsigned long irqs_used_mx6q[] = {
	MXC_INT_INTERRUPT_139_NUM,
	MX6Q_INT_PERFMON1,
	MX6Q_INT_PERFMON2,
	MX6Q_INT_PERFMON3,
};

unsigned long irqs_used_mx6dl[] = {
	MXC_INT_INTERRUPT_139_NUM,
	MX6Q_INT_PERFMON1,
};

int can_change_ddr_freq(void)
{
	return 1;
}


#ifdef CONFIG_SMP
/* Each active core apart from the one changing the DDR frequency will execute
  * this function. The rest of the cores have to remain in WFE state until the frequency
  * is changed.
  */
irqreturn_t wait_in_wfe_irq(int irq, void *dev_id)
{
	u32 me;

	me = smp_processor_id();
#ifdef CONFIG_LOCAL_TIMERS
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER,
				   &me);
#endif
	wfe_change_ddr_freq(0xff << (me * 8), (u32 *)&iram_iomux_settings[0][1]);

#ifdef CONFIG_LOCAL_TIMERS
		clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT,
				   &me);
#endif

	return IRQ_HANDLED;
}
#endif

/* Change the DDR frequency. */
int update_ddr_freq(int ddr_rate)
{
	int i, j;
	bool dll_off = false;
	int me = 0;
	u32 ttbr1;
#ifdef CONFIG_SMP
	unsigned int online_cpus = 0;
	unsigned int reg;
	int cpu = 0;
#endif
	if (!can_change_ddr_freq())
		return -1;

	if (ddr_rate == curr_ddr_rate)
		return 0;

	if (low_bus_freq_mode || audio_bus_freq_mode)
		dll_off = true;

	if (cpu_is_mx6sl() && (ddr_type == MX6_DDR3)) {
		iram_ddr_settings[0][0] = ARRAY_SIZE(ddr3_dll_mx6sl);
		iram_ddr_settings[0][1] = ARRAY_SIZE(ddr3_calibration_mx6sl);
	} else {
		iram_ddr_settings[0][0] = ddr_settings_size;
	}

	if (ddr_type == MX6_DDR3)
		iram_iomux_settings[0][0] = iomux_settings_size;

	if (cpu_is_mx6sl()) {
		if (ddr_type == MX6_LPDDR2) {
			if (ddr_rate == ddr_normal_rate) {
				for (i = 0; i < iram_ddr_settings[0][0]; i++) {
					iram_ddr_settings[i + 1][0] =
							normal_mmdc_settings[i][0];
					iram_ddr_settings[i + 1][1] =
							normal_mmdc_settings[i][1];
				}
			} else if (ddr_rate == ddr_med_rate) {
				for (i = 0; i < iram_ddr_settings[0][0]; i++) {
					iram_ddr_settings[i + 1][0] =
							lpddr2_100M_6sl[i][0];
					iram_ddr_settings[i + 1][1] =
							lpddr2_100M_6sl[i][1];
				}
			} else {
				for (i = 0; i < iram_ddr_settings[0][0]; i++) {
					iram_ddr_settings[i + 1][0] =
							lpddr2_24M_6sl[i][0];
					iram_ddr_settings[i + 1][1] =
							lpddr2_24M_6sl[i][1];
				}
			}
		} else {
			for (i = 0; i < ddr_settings_size; i++) {
					iram_ddr_settings[i + 1][0] =
							normal_mmdc_settings[i][0];
					iram_ddr_settings[i + 1][1] =
							normal_mmdc_settings[i][1];
				}
		}
	} else {
		if (ddr_rate == ddr_med_rate && cpu_is_mx6q()) {
			for (i = 0; i < ARRAY_SIZE(ddr3_dll_mx6q); i++) {
				iram_ddr_settings[i + 1][0] =
						normal_mmdc_settings[i][0];
				iram_ddr_settings[i + 1][1] =
						normal_mmdc_settings[i][1];
			}
			for (j = 0, i = ARRAY_SIZE(ddr3_dll_mx6q); i < iram_ddr_settings[0][0]; j++, i++) {
					iram_ddr_settings[i + 1][0] =
							ddr3_400[j][0];
					iram_ddr_settings[i + 1][1] =
							ddr3_400[j][1];
			}
		} else if (ddr_rate == ddr_normal_rate) {
			for (i = 0; i < iram_ddr_settings[0][0]; i++) {
					iram_ddr_settings[i + 1][0] =
							normal_mmdc_settings[i][0];
					iram_ddr_settings[i + 1][1] =
							normal_mmdc_settings[i][1];
			}
		}
	}
	/* Ensure that all Cores are in WFE. */
	local_irq_disable();

#ifdef CONFIG_SMP
	me = smp_processor_id();

	/* Make sure all the online cores are active */
	while (1) {
		bool not_exited_busfreq = false;
		for_each_online_cpu(cpu) {
			u32 reg = __raw_readl(imx_scu_base + 0x08);
			if (reg & (0x02 << (cpu * 8)))
				not_exited_busfreq = true;
		}
		if (!not_exited_busfreq)
			break;
	}

	wmb();
	*wait_for_ddr_freq_update = 1;
	dsb();

	online_cpus = readl_relaxed(imx_scu_base + 0x08);
	for_each_online_cpu(cpu) {
		*((char *)(&online_cpus) + (u8)cpu) = 0x02;
		if (cpu != me) {
			/* set the interrupt to be pending in the GIC. */
			reg = 1 << (irq_used[cpu] % 32);
			writel_relaxed(reg, gic_dist_base + GIC_DIST_PENDING_SET
				+ (irq_used[cpu] / 32) * 4);
		}
	}
	/* Wait for the other active CPUs to idle */
	while (1) {
		u32 reg = readl_relaxed(imx_scu_base + 0x08);
		reg |= (0x02 << (me * 8));
		if (reg == online_cpus)
			break;
	}
#endif

	/* Ensure iram_tlb_phys_addr is flushed to DDR. */
	__cpuc_flush_dcache_area(&iram_tlb_phys_addr, sizeof(iram_tlb_phys_addr));
	outer_clean_range(virt_to_phys(&iram_tlb_phys_addr), virt_to_phys(&iram_tlb_phys_addr + 1));

	/* Save TTBR1 */
	ttbr1 = save_ttbr1();
	/* Now we can change the DDR frequency. */
	if (cpu_is_mx6sl())
		if (ddr_type == MX6_DDR3)
			mx6sl_ddr3_change_freq(ddr_rate, iram_ddr_settings, dll_off, iram_iomux_settings, low_bus_freq_mode);
		else
			mx6sl_lpddr2_change_freq(ddr_rate, low_bus_freq_mode, iram_ddr_settings);
	else
		mx6_change_ddr_freq(ddr_rate, iram_ddr_settings, dll_off, iram_iomux_settings);

	restore_ttbr1(ttbr1);

	curr_ddr_rate = ddr_rate;

#ifdef CONFIG_SMP
	/* DDR frequency change is done . */
	*wait_for_ddr_freq_update = false;

	/* Wake up all the cores. */
	sev();
#endif
	local_irq_enable();

	printk(KERN_DEBUG "Bus freq set to %d done! cpu=%d\n", ddr_rate, me);

	return 0;
}

int init_mmdc_settings(void)
{
	int i;
	unsigned long ddr_code_size = 0;
	unsigned long wfe_code_size = 0;
#ifdef CONFIG_SMP
	int err, cpu;
#endif

	imx_scu_base = IO_ADDRESS(SCU_BASE_ADDR);

	mmdc_base = ioremap(MMDC_P0_BASE_ADDR, SZ_32K);
	iomux_base = ioremap(MX6Q_IOMUXC_BASE_ADDR, SZ_16K);
	gic_dist_base = ioremap(IC_DISTRIBUTOR_BASE_ADDR, SZ_16K);
	gic_cpu_base = ioremap(IC_INTERFACES_BASE_ADDR, SZ_16K);

	ddr_type = (__raw_readl(MMDC_MDMISC_OFFSET) & MMDC_MDMISC_DDR_TYPE_MASK) >> MMDC_MDMISC_DDR_TYPE_OFFSET;
	printk(KERN_NOTICE "DDR type is %s\n", ddr_type == 0 ? "DDR3" : "LPDDR2");

	if (ddr_type == MX6_DDR3) {
		if (cpu_is_mx6sl()) {
			ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6sl) + ARRAY_SIZE(ddr3_calibration_mx6sl);
			iomux_settings_size = ARRAY_SIZE(iomux_offsets_mx6sl);
			ddr_code_size = (&mx6sl_ddr3_iram_end -&mx6sl_ddr3_iram_start) *4;
		} else {
			ddr_code_size = (&mx6_ddr3_iram_end -&mx6_ddr3_iram_start) *4;
#ifdef CONFIG_SMP
			wfe_code_size = (&wfe_ddr3_freq_change_end -&wfe_ddr3_freq_change_start) *4;
#endif
			if (cpu_is_mx6q()) {
				ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6q) + ARRAY_SIZE(ddr3_calibration);
				iomux_settings_size = ARRAY_SIZE(iomux_offsets_mx6q);
			}
			if (cpu_is_mx6dl()) {
				ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6dl) + ARRAY_SIZE(ddr3_calibration);
				iomux_settings_size = ARRAY_SIZE(iomux_offsets_mx6dl);
			}
		}
	} else {
		if (cpu_is_mx6sl()) {
			ddr_settings_size = ARRAY_SIZE(lpddr2_400M_6sl);
			ddr_code_size = (&mx6sl_lpddr2_iram_end -&mx6sl_lpddr2_iram_start) *4;
		}
	}

	/* Use preallocated IRAM memory */
	ddr_freq_change_iram_phys_addr = MX6_IRAM_DDR_FREQ_ADDR;
	/*
	 * Don't ioremap the address, we have fixed the IRAM address
	 * at IRAM_BASE_ADDR_VIRT
	 */
	ddr_freq_change_iram_base = (void *)IRAM_BASE_ADDR_VIRT +
		(ddr_freq_change_iram_phys_addr - IRAM_BASE_ADDR);

	normal_mmdc_settings = kmalloc((ddr_settings_size * 8), GFP_KERNEL);
	if (cpu_is_mx6sl()) {
		if (ddr_type == MX6_DDR3) {
			memcpy(normal_mmdc_settings, ddr3_dll_mx6sl, sizeof(ddr3_dll_mx6sl));
			memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6sl)),
				ddr3_calibration_mx6sl, sizeof(ddr3_calibration_mx6sl));
			memcpy(ddr_freq_change_iram_base, mx6sl_ddr3_freq_change, ddr_code_size);
			mx6sl_ddr3_change_freq = (void *)ddr_freq_change_iram_base;
		} else {
			memcpy(normal_mmdc_settings, lpddr2_400M_6sl, sizeof(lpddr2_400M_6sl));
			memcpy(ddr_freq_change_iram_base, mx6sl_ddr_iram, ddr_code_size);
			mx6sl_lpddr2_change_freq = (void *)ddr_freq_change_iram_base;
		}
	} else {
		memcpy(ddr_freq_change_iram_base, mx6_ddr_freq_change, ddr_code_size);
		mx6_change_ddr_freq = (void *)ddr_freq_change_iram_base;
		if (cpu_is_mx6q()) {
			memcpy(normal_mmdc_settings, ddr3_dll_mx6q, sizeof(ddr3_dll_mx6q));
			memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6q)), ddr3_calibration, sizeof(ddr3_calibration));
		}
		if (cpu_is_mx6dl()) {
			memcpy(normal_mmdc_settings, ddr3_dll_mx6dl, sizeof(ddr3_dll_mx6dl));
			memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6dl)), ddr3_calibration, sizeof(ddr3_calibration));
		}
	}

	/* Store the original DDR settings at boot. */
	for (i = 0; i < ddr_settings_size; i++) {
		/*Writes via command mode register cannot be read back.
		 * Hence hardcode them in the initial static array.
		 * This may require modification on a per customer basis.
		 */
		if (normal_mmdc_settings[i][0] != 0x1C)
			normal_mmdc_settings[i][1] =
				__raw_readl(mmdc_base
				+ normal_mmdc_settings[i][0]);
	}

	/* Store the size of the array in iRAM also,
	 * increase the size by 8 bytes.
	 */
	iram_iomux_settings = ddr_freq_change_iram_base + ddr_code_size;
	iram_ddr_settings = iram_iomux_settings + (iomux_settings_size * 8) + 8;

	if ((ddr_code_size + wfe_code_size + (iomux_settings_size + ddr_settings_size) * 8 + 16)
		> MX6_IRAM_DDR_FREQ_CODE_SIZE) {
		printk(KERN_ERR "Not enough memory allocated for DDR Frequency change code.\n");
		return -EINVAL;
	}

	if (ddr_type == MX6_DDR3) {
		/* Store the IOMUX settings at boot. */
		if (cpu_is_mx6q()) {
			for (i = 0; i < iomux_settings_size; i++) {
				iomux_offsets_mx6q[i][1] =
					__raw_readl(iomux_base
					+ iomux_offsets_mx6q[i][0]);
				iram_iomux_settings[i+1][0] = iomux_offsets_mx6q[i][0];
				iram_iomux_settings[i+1][1] = iomux_offsets_mx6q[i][1];
			}
#ifdef CONFIG_SMP
			irq_used = irqs_used_mx6q;
#endif
		}

		if (cpu_is_mx6dl()) {
			for (i = 0; i < iomux_settings_size; i++) {
				iomux_offsets_mx6dl[i][1] =
					__raw_readl(iomux_base
					+ iomux_offsets_mx6dl[i][0]);
				iram_iomux_settings[i+1][0] = iomux_offsets_mx6dl[i][0];
				iram_iomux_settings[i+1][1] = iomux_offsets_mx6dl[i][1];
			}
#ifdef CONFIG_SMP
			irq_used = irqs_used_mx6dl;
#endif
		}
		if (cpu_is_mx6sl()) {
			for (i = 0; i < iomux_settings_size; i++) {
				iomux_offsets_mx6sl[i][1] =	__raw_readl(iomux_base
					+ iomux_offsets_mx6sl[i][0]);
				iram_iomux_settings[i+1][0] = iomux_offsets_mx6sl[i][0];
				iram_iomux_settings[i+1][1] = iomux_offsets_mx6sl[i][1];
			}
		}
	}

#ifdef CONFIG_SMP
	wfe_freq_change_iram_base = (unsigned long)((u32 *)iram_ddr_settings + (ddr_settings_size * 8) + 8);

	if (wfe_freq_change_iram_base & (FNCPY_ALIGN - 1))
			wfe_freq_change_iram_base += FNCPY_ALIGN - ((uintptr_t)wfe_freq_change_iram_base % (FNCPY_ALIGN));

	wfe_change_ddr_freq = (void *)fncpy((void *)wfe_freq_change_iram_base,
		&wfe_ddr3_freq_change, wfe_code_size);

	/* Store the variable used to communicate between cores in a non-cacheable IRAM area */
	wait_for_ddr_freq_update = (u32 *)&iram_iomux_settings[0][1];

	if (!cpu_is_mx6sl()) {
		for_each_online_cpu(cpu) {
			/* Set up a reserved interrupt to get all the active cores into a WFE state
			  * before changing the DDR frequency.
			  */
			err = request_irq(irq_used[cpu], wait_in_wfe_irq, IRQF_PERCPU, "mmdc_1",
					  NULL);
			if (err) {
				printk(KERN_ERR "MMDC: Unable to attach to %ld,err = %d\n", irq_used[cpu], err);
				return err;
			}
			err = irq_set_affinity(irq_used[cpu], cpumask_of(cpu));
			if (err) {
				printk(KERN_ERR "MMDC: unable to set irq affinity irq=%ld,\n", irq_used[cpu]);
				return err;
			}
		}
	}
#endif
	curr_ddr_rate = ddr_normal_rate;

	return 0;
}
