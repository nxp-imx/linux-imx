/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_net.h>
#include <linux/slab.h>

#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init imx6_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np, *from = NULL;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low;
	u32 macaddr_high = 0;
	u32 macaddr1_high = 0;
	u8 *macaddr;
	int i;

	for (i = 0; i < 2; i++) {
		enet_np = of_find_compatible_node(from, NULL, compatible);
		if (!enet_np)
			return;

		from = enet_np;

		if (of_get_mac_address(enet_np))
			goto put_enet_node;

		ocotp_np = of_find_compatible_node(NULL,
			NULL, "fsl,imx6q-ocotp");
		if (!ocotp_np) {
			pr_warn("failed to find ocotp node\n");
			goto put_enet_node;
		}

		base = of_iomap(ocotp_np, 0);
		if (!base) {
			pr_warn("failed to map ocotp\n");
			goto put_ocotp_node;
		}

		macaddr_low = readl_relaxed(base + OCOTP_MACn(1));
		if (i)
			macaddr1_high = readl_relaxed(base + OCOTP_MACn(2));
		else
			macaddr_high = readl_relaxed(base + OCOTP_MACn(0));

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (!newmac)
			goto put_ocotp_node;

		newmac->value = newmac + 1;
		newmac->length = 6;
		newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newmac->name) {
			kfree(newmac);
			goto put_ocotp_node;
		}

		macaddr = newmac->value;
		if (i) {
			macaddr[5] = (macaddr_low >> 16) & 0xff;
			macaddr[4] = (macaddr_low >> 24) & 0xff;
			macaddr[3] = macaddr1_high & 0xff;
			macaddr[2] = (macaddr1_high >> 8) & 0xff;
			macaddr[1] = (macaddr1_high >> 16) & 0xff;
			macaddr[0] = (macaddr1_high >> 24) & 0xff;
		} else {
			macaddr[5] = macaddr_high & 0xff;
			macaddr[4] = (macaddr_high >> 8) & 0xff;
			macaddr[3] = (macaddr_high >> 16) & 0xff;
			macaddr[2] = (macaddr_high >> 24) & 0xff;
			macaddr[1] = macaddr_low & 0xff;
			macaddr[0] = (macaddr_low >> 8) & 0xff;
		}

		of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
	}
}

#if !defined(CONFIG_SOC_IMX6SL)
u32 imx6_lpddr2_freq_change_start, imx6_lpddr2_freq_change_end;
void mx6_lpddr2_freq_change(u32 freq, int bus_freq_mode) {}
#endif

#if !defined(CONFIG_SOC_IMX6SX)
struct imx6_busfreq_info {
} __aligned(8);
u32 imx6sx_ddr3_freq_change_start, imx6sx_ddr3_freq_change_end;
void imx6sx_ddr3_freq_change(struct imx6_busfreq_info *busfreq_info) {}
void imx6sx_lpddr2_freq_change(u32 freq, int bus_freq_mode) {}
#endif

#if !defined(CONFIG_SOC_IMX6Q)
u32 mx6_ddr3_freq_change_start, mx6_ddr3_freq_change_end;
u32 wfe_ddr3_freq_change_start, wfe_ddr3_freq_change_end;
void mx6_ddr3_freq_change(u32 freq, void *ddr_settings,
	bool dll_mode, void *iomux_offsets) {}
void wfe_ddr3_freq_change(u32 cpuid, u32 *ddr_freq_change_done) {}
#endif
