/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ahci_platform.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/ahci_sata.h>

/* In order to decrease the pwr consumption, release the CLK resources when
 * there is no SATA device adaptored into the AHCI SATA port.
 * The HOTPLUG feature can't be enabled in this situation.
 * Please change this macro to '0' if the HOTPLUG is mandatory required.
 */
#ifdef CONFIG_SATA_AHCI_FSL_NO_HOTPLUG_MODE
#define AHCI_SAVE_PWR_WITHOUT_HOTPLUG 1
#else
#define AHCI_SAVE_PWR_WITHOUT_HOTPLUG 0
#endif

static struct clk *sata_clk, *sata_ref_clk;

int write_phy_ctl_ack_polling(u32 data, void __iomem *mmio,
		int max_iterations, u32 exp_val)
{
	u32 i, val;

	writel(data, mmio + PORT_PHY_CTL);

	for (i = 0; i < max_iterations + 1; i++) {
		val = readl(mmio + PORT_PHY_SR);
		val =  (val >> PORT_PHY_STAT_ACK_LOC) & 0x1;
		if (val == exp_val)
			return 0;
		if (i == max_iterations) {
			printk(KERN_ERR "Wait for CR ACK error!\n");
			return 1;
		}
		udelay(200);
	}
	return 0;
}

int sata_phy_cr_addr(u32 addr, void __iomem *mmio)
{
	u32 temp_wr_data;

	/* write addr */
	temp_wr_data = addr;
	writel(temp_wr_data, mmio + PORT_PHY_CTL);

	/* capture addr */
	temp_wr_data |= PORT_PHY_CTL_CAP_ADR_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert cap addr */
	temp_wr_data &= 0xffff;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

int sata_phy_cr_write(u32 data, void __iomem *mmio)
{
	u32 temp_wr_data;

	/* write data */
	temp_wr_data = data;

	/* capture data */
	temp_wr_data |= PORT_PHY_CTL_CAP_DAT_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert cap data */
	temp_wr_data &= 0xffff;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	/* assert wr signal */
	temp_wr_data |= PORT_PHY_CTL_WRITE_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* deassert wr _signal */
	temp_wr_data = 0x0;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

int sata_phy_cr_read(u32 *data, void __iomem *mmio)
{
	u32 temp_rd_data, temp_wr_data;

	/* assert rd signal */
	temp_wr_data = PORT_PHY_CTL_READ_LOC;

	/* wait for ack */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 1))
		return 1;

	/* after got ack return data */
	temp_rd_data = readl(mmio + PORT_PHY_SR);
	*data = (temp_rd_data & 0xffff);

	/* deassert rd _signal */
	temp_wr_data = 0x0 ;

	/* wait for ack de-assetion */
	if (write_phy_ctl_ack_polling(temp_wr_data, mmio, 100, 0))
		return 1;

	return 0;
}

/* HW Initialization, if return 1, initialization is failed. */
static int sata_init(struct device *dev)
{
	void __iomem *mmio;
	u32 tmpdata;
	int ret = 0, iterations = 20;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	mmio = ioremap(MX53_SATA_BASE_ADDR, SZ_2K);
	if (mmio == NULL) {
		dev_err(dev, "Failed to map SATA REGS\n");
		goto release_sata_clk;
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_mem;
	}

	tmpdata = readl(mmio + HOST_CAP);
	if (!(tmpdata & HOST_CAP_SSS)) {
		tmpdata |= HOST_CAP_SSS;
		writel(tmpdata, mmio + HOST_CAP);
	}

	if (!(readl(mmio + HOST_PORTS_IMPL) & 0x1))
		writel((readl(mmio + HOST_PORTS_IMPL) | 0x1),
			mmio + HOST_PORTS_IMPL);

	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	writel(tmpdata, mmio + HOST_TIMER1MS);

	if (machine_is_mx53_smd() || machine_is_mx53_loco()
			|| board_is_mx53_ard_b()) {
		/* FSL IMX AHCI SATA uses the internal usb phy1 clk */
		sata_ref_clk = clk_get(NULL, "usb_phy1_clk");
		if (IS_ERR(sata_ref_clk)) {
			dev_err(dev, "no sata ref clock.\n");
			ret = PTR_ERR(sata_ref_clk);
			goto release_mem;
		}
		ret = clk_enable(sata_ref_clk);
		if (ret) {
			dev_err(dev, "can't enable sata ref clock.\n");
			goto put_sata_ref_clk;
		}
	} else {
		/* External CLK input is used for AHCI */
		/* write addr */
		tmpdata = PHY_CR_CLOCK_FREQ_OVRD;
		writel(tmpdata, mmio + PORT_PHY_CTL);
		/* capture addr */
		tmpdata |= PORT_PHY_CTL_CAP_ADR_LOC;
		/* Wait for ack */
		if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 1)) {
			ret = -EIO;
			goto release_sata_clk;
		}

		/* deassert cap data */
		tmpdata &= 0xFFFF;
		/* wait for ack de-assertion */
		if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 0)) {
			ret = -EIO;
			goto release_sata_clk;
		}

		/* write data */
		/* Configure the PHY CLK input refer to different OSC
		 * For 25MHz, pre[13,14]:01, ncy[12,8]:06,
		 * ncy5[7,6]:02, int_ctl[5,3]:0, prop_ctl[2,0]:7.
		 * For 50MHz, pre:00, ncy:06, ncy5:02, int_ctl:0, prop_ctl:7.
		 */
		/* EVK revA */
		if (board_is_mx53_evk_a())
			tmpdata = (0x1 << 15) | (0x1 << 13) | (0x6 << 8)
				| (0x2 << 6) | 0x7;
		/* Others are 50MHz */
		else
			tmpdata = (0x1 << 15) | (0x0 << 13) | (0x6 << 8)
				| (0x2 << 6) | 0x7;

		writel(tmpdata, mmio + PORT_PHY_CTL);
		/* capture data */
		tmpdata |= PORT_PHY_CTL_CAP_DAT_LOC;
		/* wait for ack */
		if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 1)) {
			ret = -EIO;
			goto release_sata_clk;
		}

		/* deassert cap data */
		tmpdata &= 0xFFFF;
		/* wait for ack de-assertion */
		if (write_phy_ctl_ack_polling(tmpdata, mmio, 100, 0)) {
			ret = -EIO;
			goto release_sata_clk;
		}

		/* assert wr signal and wait for ack */
		if (write_phy_ctl_ack_polling(PORT_PHY_CTL_WRITE_LOC, mmio,
					100, 1)) {
			ret = -EIO;
			goto release_sata_clk;
		}
		/* deassert rd _signal and wait for ack de-assertion */
		if (write_phy_ctl_ack_polling(0, mmio, 100, 0)) {
			ret = -EIO;
			goto release_sata_clk;
		}
	}

	if (AHCI_SAVE_PWR_WITHOUT_HOTPLUG) {
		/* Release resources when there is no device on the port */
		do {
			if ((readl(mmio + PORT_SATA_SR) & 0xF) == 0)
				msleep(25);
			else
				break;

			if (iterations == 0) {
				pr_info("No sata disk.\n");
				ret = -ENODEV;
				/* Enter into PDDQ mode, save power */
				tmpdata = readl(mmio + PORT_PHY_CTL);
				writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC,
							mmio + PORT_PHY_CTL);

				if (machine_is_mx53_smd()
					|| machine_is_mx53_loco()
					|| board_is_mx53_ard_b())
					goto no_device;
				else
					goto release_mem;
			}
		} while (iterations-- > 0);
	}

	iounmap(mmio);
	return ret;

no_device:
	clk_disable(sata_ref_clk);
put_sata_ref_clk:
	clk_put(sata_ref_clk);
release_mem:
	iounmap(mmio);
release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void sata_exit(struct device *dev)
{
	if (machine_is_mx53_smd() || machine_is_mx53_loco()
			|| board_is_mx53_ard_b()) {
		/* FSL IMX AHCI SATA uses the internal usb phy1 clk */
		clk_disable(sata_ref_clk);
		clk_put(sata_ref_clk);
	}

	clk_disable(sata_clk);
	clk_put(sata_clk);
}

struct ahci_platform_data sata_data = {
	.init = sata_init,
	.exit = sata_exit,
};

