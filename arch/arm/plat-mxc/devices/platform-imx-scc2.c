/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/mxc_scc2_driver.h>
#include <linux/delay.h>

#define imx_scc2_data_entry_single(soc)	\
{						\
	.iobase = soc ## _SCC_BASE_ADDR,	\
	.ram_start = soc ## _SCC_RAM_BASE_ADDR,	\
	.irq_smn = soc ## _INT_SCC_SMN,		\
	.irq_scm = soc ## _INT_SCC_SCM,		\
}

#ifdef CONFIG_SOC_IMX51
const struct imx_mxc_scc2_data imx51_mxc_scc2_data __initconst =
	imx_scc2_data_entry_single(MX51);
#endif /* ifdef CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
const struct imx_mxc_scc2_data imx53_mxc_scc2_data __initconst =
	imx_scc2_data_entry_single(MX53);
#endif /* ifdef CONFIG_SOC_IMX53 */

#define SCM_RD_DELAY	1000000 /* in nanoseconds */
#define SEC_TO_NANOSEC  1000000000 /*Second to nanoseconds */
static __init void mxc_init_scc_iram(struct resource res[])
{
	uint32_t reg_value;
	uint32_t reg_mask = 0;
	uint8_t *UMID_base;
	uint32_t *MAP_base;
	uint8_t i;
	uint32_t partition_no;
	uint32_t scc_partno;
	void *scm_ram_base;
	void *scc_base;
	uint32_t ram_partitions, ram_partition_size, ram_size;
	uint32_t scm_version_register;
	struct timespec stime;
	struct timespec curtime;
	long scm_rd_timeout = 0;
	long cur_ns = 0;
	long start_ns = 0;

	scc_base = ioremap((uint32_t) res[0].start, 0x140);
	if (scc_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP SCC REGS\n");
		return;
	}

	scm_version_register = __raw_readl(scc_base + SCM_VERSION_REG);
	ram_partitions = 1 + ((scm_version_register & SCM_VER_NP_MASK)
		>> SCM_VER_NP_SHIFT);
	ram_partition_size = (uint32_t) (1 <<
		((scm_version_register & SCM_VER_BPP_MASK)
		>> SCM_VER_BPP_SHIFT));

	ram_size = (uint32_t)(ram_partitions * ram_partition_size);

	scm_ram_base = ioremap((uint32_t) res[1].start, ram_size);

	if (scm_ram_base == NULL) {
		printk(KERN_ERR "FAILED TO MAP SCC RAM\n");
		return;
	}

	/* Wait for any running SCC operations to finish or fail */
	getnstimeofday(&stime);
	do {
		reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
		getnstimeofday(&curtime);
		if (curtime.tv_nsec > stime.tv_nsec)
			scm_rd_timeout = curtime.tv_nsec - stime.tv_nsec;
		else{
			/*Converted second to nanosecond and add to
			nsec when current nanosec is less than
			start time nanosec.*/
			cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
			curtime.tv_nsec;
			start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
				stime.tv_nsec;
			scm_rd_timeout = cur_ns - start_ns;
		}
	} while (((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
	&& ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_FAIL));

	/* Check for failures */
	if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY) {
		/* Special message for bad secret key fuses */
		if (reg_value & SCM_STATUS_KST_BAD_KEY)
			printk(KERN_ERR "INVALID SCC KEY FUSE PATTERN\n");
		else
		    printk(KERN_ERR "SECURE RAM FAILURE\n");

		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

	scm_rd_timeout = 0;

#ifdef CONFIG_ARCH_MX5
	/* Release all partitions for SCC2 driver on MX53*/
	if (cpu_is_mx53())
		scc_partno = 0;
	/* Release final two partitions for SCC2 driver on MX51 */
	else
		scc_partno = ram_partitions -
			(MX51_SCC_RAM_SIZE / ram_partition_size);
#else
	scc_partno = 0;
#endif


	for (partition_no = scc_partno; partition_no < ram_partitions;
	     partition_no++) {
		reg_value = (((partition_no << SCM_ZCMD_PART_SHIFT) &
			SCM_ZCMD_PART_MASK) | ((0x03 << SCM_ZCMD_CCMD_SHIFT) &
			SCM_ZCMD_CCMD_MASK));
		__raw_writel(reg_value, scc_base + SCM_ZCMD_REG);
		udelay(1);
		/* Wait for zeroization to complete */
		getnstimeofday(&stime);
		do {
			reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
			getnstimeofday(&curtime);
			if (curtime.tv_nsec > stime.tv_nsec)
				scm_rd_timeout = curtime.tv_nsec -
				stime.tv_nsec;
			else {
				/*Converted second to nanosecond and add to
				nsec when current nanosec is less than
				start time nanosec.*/
				cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
				curtime.tv_nsec;
				start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
					stime.tv_nsec;
				scm_rd_timeout = cur_ns - start_ns;
			}
		} while (((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_READY) && ((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_FAIL) && (scm_rd_timeout <= SCM_RD_DELAY));

		if (scm_rd_timeout > SCM_RD_DELAY)
			printk(KERN_ERR "SCM Status Register Read timeout"
			"for Partition No:%d", partition_no);

		if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
			break;
	}

	/* 4 partitions on MX53 */
	if (cpu_is_mx53())
		reg_mask = 0xFF;

	/*Check all expected partitions released */
	reg_value = __raw_readl(scc_base + SCM_PART_OWNERS_REG);
	if ((reg_value & reg_mask) != 0) {
		printk(KERN_ERR "FAILED TO RELEASE IRAM PARTITION\n");
		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

	/* we are done if this is MX53, since no sharing of IRAM and SCC_RAM */
	if (cpu_is_mx53())
		goto exit;

	reg_mask = 0;
	scm_rd_timeout = 0;
	/* Allocate remaining partitions for general use */
	for (partition_no = 0; partition_no < scc_partno; partition_no++) {
		/* Supervisor mode claims a partition for it's own use
		by writing zero to SMID register.*/
		__raw_writel(0, scc_base + (SCM_SMID0_REG + 8 * partition_no));

		/* Wait for any zeroization to complete */
		getnstimeofday(&stime);
		do {
			reg_value = __raw_readl(scc_base + SCM_STATUS_REG);
			getnstimeofday(&curtime);
			if (curtime.tv_nsec > stime.tv_nsec)
				scm_rd_timeout = curtime.tv_nsec -
				stime.tv_nsec;
			else{
				/*Converted second to nanosecond and add to
				nsec when current nanosec is less than
				start time nanosec.*/
				cur_ns = (curtime.tv_sec * SEC_TO_NANOSEC) +
				curtime.tv_nsec;
				start_ns = (stime.tv_sec * SEC_TO_NANOSEC) +
					stime.tv_nsec;
				scm_rd_timeout = cur_ns - start_ns;
			}
		} while (((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_READY) && ((reg_value & SCM_STATUS_SRS_MASK) !=
		SCM_STATUS_SRS_FAIL) && (scm_rd_timeout <= SCM_RD_DELAY));

		if (scm_rd_timeout > SCM_RD_DELAY)
			printk(KERN_ERR "SCM Status Register Read timeout"
			"for Partition No:%d", partition_no);

		if ((reg_value & SCM_STATUS_SRS_MASK) != SCM_STATUS_SRS_READY)
			break;
		/* Set UMID=0 and permissions for universal data
		read/write access */
		MAP_base = scm_ram_base +
			(uint32_t) (partition_no * ram_partition_size);
		UMID_base = (uint8_t *) MAP_base + 0x10;
		for (i = 0; i < 16; i++)
			UMID_base[i] = 0;

		MAP_base[0] = (SCM_PERM_NO_ZEROIZE | SCM_PERM_HD_SUP_DISABLE |
			SCM_PERM_HD_READ | SCM_PERM_HD_WRITE |
			SCM_PERM_HD_EXECUTE | SCM_PERM_TH_READ |
			SCM_PERM_TH_WRITE);
		reg_mask |= (3 << (2 * (partition_no)));
	}

	/* Check all expected partitions allocated */
	reg_value = __raw_readl(scc_base + SCM_PART_OWNERS_REG);
	if ((reg_value & reg_mask) != reg_mask) {
		printk(KERN_ERR "FAILED TO ACQUIRE IRAM PARTITION\n");
		iounmap(scm_ram_base);
		iounmap(scc_base);
		return;
	}

exit:
	iounmap(scm_ram_base);
	iounmap(scc_base);
	printk(KERN_INFO "IRAM READY\n");
}


struct platform_device *__init imx_add_mxc_scc2(
		const struct imx_mxc_scc2_data *data)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->ram_start,
			.end = data->ram_start + SZ_128K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq_smn,
			.end = data->irq_smn,
			.flags = IORESOURCE_IRQ,
		}, {
			.start = data->irq_scm,
			.end = data->irq_scm,
			.flags = IORESOURCE_IRQ,
		},
	};

	mxc_init_scc_iram(res);

	return imx_add_platform_device("mxc_scc", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
