// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2025-2026 Emcraft Systems
// Author: Vladimir Skvortsov <vskvortsov@emcraft.com>
//
// HyperBus Memory Controller driver for NXP i.MX RT SoCs
// using FlexSPI peripheral in HyperBus mode.
//
// Performs full FlexSPI initialization for HyperBus mode from scratch,
// independent of ROM bootloader or U-Boot configuration. Flash size,
// clock rate, and differential clock are configured from DTS properties.
// Provides read16/write16 via raw AHB memory-mapped I/O; HyperBus
// big-endian byte order is handled at the CFI layer via map->swap.

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/cfi_endian.h>
#include <linux/mtd/hyperbus.h>
#include <linux/mtd/mtd.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>

/* FlexSPI registers */
#define FSPI_MCR0			0x00
#define FSPI_MCR0_AHB_TIMEOUT(x)	((x) << 24)
#define FSPI_MCR0_IP_TIMEOUT(x)		((x) << 16)
#define FSPI_MCR0_LEARN_EN		BIT(15)
#define FSPI_MCR0_OCTCOMB_EN		BIT(13)
#define FSPI_MCR0_RXCLKSRC(x)		((x) << 4)
#define FSPI_MCR0_MDIS			BIT(1)
#define FSPI_MCR0_SWRST			BIT(0)

#define FSPI_MCR2			0x08
#define FSPI_MCR2_IDLE_WAIT(x)		((x) << 24)
#define FSPI_MCR2_SCKBDIFFOPT		BIT(19)	/* Differential clock for HyperBus */
#define FSPI_MCR2_ABRDATSZ		BIT(8)
#define FSPI_MCR2_ABRLEARN		BIT(7)
#define FSPI_MCR2_ABR_READ		BIT(6)
#define FSPI_MCR2_ABRWRITE		BIT(5)
#define FSPI_MCR2_ABRDUMMY		BIT(4)
#define FSPI_MCR2_ABRCADDR		BIT(2)
#define FSPI_MCR2_ABRRADDR		BIT(1)
#define FSPI_MCR2_ABR_CMD		BIT(0)

#define FSPI_AHBCR			0x0c
#define FSPI_AHBCR_RDADDROPT		BIT(6)
#define FSPI_AHBCR_PREF_EN		BIT(5)
#define FSPI_AHBCR_BUFF_EN		BIT(4)
#define FSPI_AHBCR_CACH_EN		BIT(3)

#define FSPI_LUTKEY			0x18
#define FSPI_LUTKEY_VALUE		0x5AF05AF0
#define FSPI_LCKCR			0x1C
#define FSPI_LCKER_UNLOCK		0x2
#define FSPI_LCKER_LOCK			0x1

#define FSPI_FLSHA1CR0			0x60
#define FSPI_FLSHA2CR0			0x64
#define FSPI_FLSHB1CR0			0x68
#define FSPI_FLSHB2CR0			0x6C

#define FSPI_FLSHA1CR1			0x70
#define FSPI_FLSHXCR1_CAS(x)		((x) << 11)	/* Column Address Space */
#define FSPI_FLSHXCR1_WA		BIT(10)		/* Word-addressable */
#define FSPI_FLSHXCR1_TCSH(x)		((x) << 5)	/* CS hold time (cycles) */
#define FSPI_FLSHXCR1_TCSS(x)		(x)		/* CS setup time (cycles) */

#define FSPI_FLSHA1CR2			0x80
#define FSPI_FLSHXCR2_AWRSEQI(x)	((x) << 8)	/* AHB write LUT seq index */
#define FSPI_FLSHXCR2_ARDSEQI(x)	(x)		/* AHB read LUT seq index */

#define FSPI_DLLACR			0xC0
#define FSPI_DLLACR_REFPHASEGAP(x)	((x) << 15) /* Ref clock phase adjust gap */
#define FSPI_DLLACR_OVRDEN		BIT(8)
#define FSPI_DLLACR_SLVDLY(x)		((x) << 3)
#define FSPI_DLLACR_DLLRESET		BIT(1)
#define FSPI_DLLACR_DLLEN		BIT(0)

#define FSPI_DLLBCR			0xC4
#define FSPI_DLLBCR_OVRDEN		BIT(8)
#define FSPI_DLLBCR_SLVDLY(x)		((x) << 3)
#define FSPI_DLLBCR_DLLRESET		BIT(1)
#define FSPI_DLLBCR_DLLEN		BIT(0)

#define FSPI_STS0			0xE0
#define FSPI_STS0_ARB_IDLE		BIT(1)
#define FSPI_STS0_SEQ_IDLE		BIT(0)

#define FSPI_STS2			0xE8
#define FSPI_STS2_BREFLOCK		BIT(3)
#define FSPI_STS2_BSLVLOCK		BIT(2)
#define FSPI_STS2_AREFLOCK		BIT(1)
#define FSPI_STS2_ASLVLOCK		BIT(0)

/* DLL calibration threshold: same as spi-nxp-fspi.c */
#define FSPI_DLL_CALIB_HZ		100000000

#define FSPI_LUT_BASE			0x200
#define FSPI_LUT_NUM			16	/* imxrt1170 has 16 LUT entries */

/* LUT instructions */
#define LUT_STOP			0x00
#define LUT_CMD_DDR			0x21
#define LUT_ADDR_DDR			0x22
#define LUT_CADDR_DDR			0x23
#define LUT_WRITE_DDR			0x28
#define LUT_READ_DDR			0x29
#define LUT_DUMMY_DDR			0x2C

/* LUT encoding */
#define PAD_SHIFT			8
#define INSTR_SHIFT			10
#define OPRND_SHIFT			16

#define LUT_PAD(x)			(fls(x) - 1)

#define LUT_DEF(idx, ins, pad, opr)				\
	((((ins) << INSTR_SHIFT) | ((pad) << PAD_SHIFT) |	\
	(opr)) << (((idx) % 2) * OPRND_SHIFT))

/* LUT sequence indices (each sequence = 4 LUT registers = 16 bytes) */
#define SEQ_IDX_READ			0
#define SEQ_IDX_WRITE			1

/* HyperFlash defaults */
#define HYPERFLASH_CAS			3	/* Column Address Space width */

/*
 * HyperFlash initial access time (tACC) from the S26KS512 datasheet.
 * Used to compute the number of dummy clock cycles at runtime based on
 * the actual bus frequency: dummy_clocks = ceil(tACC_ns * freq_MHz / 1000).
 * Result is clamped to [5, 16] per datasheet Table 4 (valid latency codes).
 *
 * With RXCLKSRC=3 (flash-provided DQS/RWDS), the controller uses RWDS
 * transitions to know when data is valid.  During the dummy phase RWDS
 * is ignored, so dummy_clocks must be <= the flash's configured latency
 * (NVCR).  If dummy_clocks > flash latency, the controller misses the
 * initial RWDS toggles and loses data.
 */
#define HYPERFLASH_TACC_NS		96
#define HYPERFLASH_DUMMY_MIN		5
#define HYPERFLASH_DUMMY_MAX		16

/* Timeout for controller operations */
#define FSPI_TIMEOUT_US			1000000

struct imxrt_hbmc_devtype {
	bool has_refphasegap;	/* DLLACR REFPHASEGAP field (RT1170+) */
};

struct imxrt_hbmc_priv {
	struct hyperbus_ctlr ctlr;
	struct hyperbus_device hbdev;
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 flash_size;
	bool sck_diff;
	struct clk *clk;
	struct clk *clk_en;
	const struct imxrt_hbmc_devtype *devtype;
};

static inline void fspi_writel(struct imxrt_hbmc_priv *priv, u32 val, u32 off)
{
	writel(val, priv->iobase + off);
}

static inline u32 fspi_readl(struct imxrt_hbmc_priv *priv, u32 off)
{
	return readl(priv->iobase + off);
}

static void fspi_wait_idle(struct imxrt_hbmc_priv *priv, struct device *dev)
{
	u32 val;

	if (readl_poll_timeout(priv->iobase + FSPI_STS0, val,
			       (val & (FSPI_STS0_ARB_IDLE | FSPI_STS0_SEQ_IDLE)) ==
			       (FSPI_STS0_ARB_IDLE | FSPI_STS0_SEQ_IDLE),
			       10, FSPI_TIMEOUT_US))
		dev_warn(dev, "FlexSPI idle timeout\n");
}

/*
 * Perform a software reset of FlexSPI state machines.
 *
 * Note: MCR0 |= SWRST preserves register configuration and only resets the
 * internal state machines (AHB + serial flash domains). The SWRST bit
 * auto-clears when reset is complete.
 */
static void fspi_swreset(struct imxrt_hbmc_priv *priv)
{
	u32 val;

	val = fspi_readl(priv, FSPI_MCR0);
	val |= FSPI_MCR0_SWRST;
	fspi_writel(priv, val, FSPI_MCR0);

	readl_poll_timeout(priv->iobase + FSPI_MCR0, val,
			   !(val & FSPI_MCR0_SWRST), 10, FSPI_TIMEOUT_US);
}

/*
 * DLL calibration mode for clock rates > 100MHz.
 *
 * The DLL auto-calibrates the slave delay line to achieve a target delay
 * of ((SLVDLYTARGET+1) * 1/32) of the reference clock cycle.
 * SLVDLYTARGET=0xF means half the clock cycle, recommended for >100MHz.
 *
 * On i.MX RT1170, REFPHASEGAP = 2 is recommended by NXP SDK
 * (fsl_flexspi.c) when DLL is enabled.  This field does not exist
 * on earlier SoCs (RT1050/RT1060).
 *
 * Must wait for REF and SLV lock bits in STS2 before accessing flash.
 * ERR050272 workaround: add 4us delay after lock (lock status may be
 * inaccurate).
 *
 * In HyperBus differential clock mode, port B carries the controller-
 * generated CK# signal and has no external reference, so DLL B cannot
 * lock and is kept in override mode.
 */
static int fspi_dll_calibration(struct imxrt_hbmc_priv *priv,
				struct device *dev)
{
	u32 dllacr, val;

	/* Reset DLL A */
	fspi_writel(priv, FSPI_DLLACR_DLLRESET, FSPI_DLLACR);
	fspi_writel(priv, 0, FSPI_DLLACR);

	/*
	 * DLL B: keep in override mode for differential clock (port B
	 * carries CK#, no reference for calibration).
	 */
	fspi_writel(priv, FSPI_DLLBCR_OVRDEN, FSPI_DLLBCR);

	/* Enable DLL A with SLVDLYTARGET = 0xF (half clock cycle) */
	dllacr = FSPI_DLLACR_DLLEN | FSPI_DLLACR_SLVDLY(0xF);
	if (priv->devtype->has_refphasegap)
		dllacr |= FSPI_DLLACR_REFPHASEGAP(2);
	fspi_writel(priv, dllacr, FSPI_DLLACR);

	/* Wait for DLL A REF and SLV lock */
	if (readl_poll_timeout(priv->iobase + FSPI_STS2, val,
			       (val & (FSPI_STS2_AREFLOCK | FSPI_STS2_ASLVLOCK)) ==
			       (FSPI_STS2_AREFLOCK | FSPI_STS2_ASLVLOCK),
			       10, FSPI_TIMEOUT_US)) {
		dev_err(dev, "DLL-A lock failed (STS2=0x%08x)\n", val);
		return -ETIMEDOUT;
	}

	/* ERR050272 workaround */
	udelay(4);

	return 0;
}

/*
 * DLL override mode for clock rates <= 100MHz.
 * Uses a fixed delay with no calibration.
 */
static void fspi_dll_override(struct imxrt_hbmc_priv *priv)
{
	fspi_writel(priv, FSPI_DLLACR_OVRDEN, FSPI_DLLACR);
	fspi_writel(priv, FSPI_DLLBCR_OVRDEN, FSPI_DLLBCR);
}

/*
 * Full FlexSPI initialization for HyperBus mode.
 *
 * Programs all FlexSPI registers from scratch, making the driver
 * self-contained and independent of prior boot stage configuration.
 * Flash size and differential clock come from DTS; clock rate must
 * be set before calling this function.
 *
 * Init sequence follows spi-nxp-fspi.c:
 *   MDIS → swreset → configure registers → clear MDIS → LUT → DLL.
 *
 * DLL is configured based on actual clock rate: calibration mode for
 * >100MHz, override mode otherwise.
 */
static int imxrt_hbmc_init_hw(struct imxrt_hbmc_priv *priv,
			      struct device *dev)
{
	void __iomem *lut_base = priv->iobase + FSPI_LUT_BASE;
	unsigned long clk_rate, bus_freq_mhz;
	unsigned int dummy_clocks;
	int i;

	clk_rate = clk_get_rate(priv->clk);
	bus_freq_mhz = clk_rate / 2000000;

	/* Compute dummy clocks from bus frequency and flash tACC */
	dummy_clocks = DIV_ROUND_UP(HYPERFLASH_TACC_NS * bus_freq_mhz, 1000);
	dummy_clocks = clamp_val(dummy_clocks,
				 HYPERFLASH_DUMMY_MIN, HYPERFLASH_DUMMY_MAX);
	dev_info(dev, "FlexSPI clock: %lu Hz, bus freq %lu MHz, "
		 "read latency %u clocks\n",
		 clk_rate, bus_freq_mhz, dummy_clocks);

	/* Wait for controller idle before reconfiguration */
	fspi_wait_idle(priv, dev);

	/* Disable module for safe register programming */
	fspi_writel(priv, fspi_readl(priv, FSPI_MCR0) | FSPI_MCR0_MDIS,
		    FSPI_MCR0);

	/* Software reset — resets state machines, preserves register config */
	fspi_swreset(priv);

	/* MCR0: set all fields with MDIS still set */
	fspi_writel(priv, FSPI_MCR0_MDIS |
			  FSPI_MCR0_AHB_TIMEOUT(0xFF) |
			  FSPI_MCR0_IP_TIMEOUT(0xFF) |
			  FSPI_MCR0_LEARN_EN |
			  FSPI_MCR0_OCTCOMB_EN |
			  FSPI_MCR0_RXCLKSRC(3),
		    FSPI_MCR0);

	/*
	 * MCR2: IDLE_WAIT=0x20 idle cycles before suspend, auto-resume
	 * bits for all command phases, and optionally differential clock.
	 */
	fspi_writel(priv, FSPI_MCR2_IDLE_WAIT(0x20) |
			  (priv->sck_diff ? FSPI_MCR2_SCKBDIFFOPT : 0) |
			  FSPI_MCR2_ABRDATSZ |
			  FSPI_MCR2_ABRLEARN |
			  FSPI_MCR2_ABR_READ |
			  FSPI_MCR2_ABRWRITE |
			  FSPI_MCR2_ABRDUMMY |
			  FSPI_MCR2_ABRCADDR |
			  FSPI_MCR2_ABRRADDR |
			  FSPI_MCR2_ABR_CMD,
		    FSPI_MCR2);

	/*
	 * Flash size in KB — clear all port sizes first (ROM bootloader may
	 * have set non-zero values), then set only port A1.
	 */
	fspi_writel(priv, 0, FSPI_FLSHA2CR0);
	fspi_writel(priv, 0, FSPI_FLSHB1CR0);
	fspi_writel(priv, 0, FSPI_FLSHB2CR0);
	fspi_writel(priv, priv->flash_size / SZ_1K, FSPI_FLSHA1CR0);

	/*
	 * FLSHA1CR1: CAS=3 (column address width for HyperBus),
	 * WA (word-addressable), TCSH/TCSS=3 (CS hold/setup in cycles).
	 */
	fspi_writel(priv, FSPI_FLSHXCR1_CAS(HYPERFLASH_CAS) |
			  FSPI_FLSHXCR1_WA |
			  FSPI_FLSHXCR1_TCSH(3) |
			  FSPI_FLSHXCR1_TCSS(3),
		    FSPI_FLSHA1CR1);

	/* AHB read uses LUT sequence 0, AHB write uses LUT sequence 1 */
	fspi_writel(priv, FSPI_FLSHXCR2_AWRSEQI(SEQ_IDX_WRITE) |
			  FSPI_FLSHXCR2_ARDSEQI(SEQ_IDX_READ),
		    FSPI_FLSHA1CR2);

	/* AHBCR: read address optimization, AHB cache */
	fspi_writel(priv, FSPI_AHBCR_RDADDROPT | FSPI_AHBCR_CACH_EN,
		    FSPI_AHBCR);

	/* Clear MDIS — enable the module before LUT programming and DLL */
	fspi_writel(priv, FSPI_MCR0_AHB_TIMEOUT(0xFF) |
			  FSPI_MCR0_IP_TIMEOUT(0xFF) |
			  FSPI_MCR0_LEARN_EN |
			  FSPI_MCR0_OCTCOMB_EN |
			  FSPI_MCR0_RXCLKSRC(3),
		    FSPI_MCR0);

	/*
	 * Program LUT sequences (module must be enabled — LUT writes
	 * silently fail when MDIS=1).
	 *
	 * HyperBus 48-bit Command-Address phase:
	 *   CMD_DDR  (8pad, cmd)   - CA[47:40]: R/W#, memory/register space
	 *   ADDR_DDR (8pad, 0x18)  - CA[39:16]: 24-bit row address
	 *   CADDR_DDR(8pad, 0x10)  - CA[15:0]:  16-bit column address
	 */
	fspi_writel(priv, FSPI_LUTKEY_VALUE, FSPI_LUTKEY);
	fspi_writel(priv, FSPI_LCKER_UNLOCK, FSPI_LCKCR);

	/* Clear all LUT entries */
	for (i = 0; i < FSPI_LUT_NUM * 4; i++)
		writel(0, lut_base + i * 4);

	/*
	 * Sequence 0: Read (CMD_DDR 0xA0 + ADDR + CADDR + DUMMY + READ)
	 * 0xA0 = HyperBus read, memory space
	 */
	writel(LUT_DEF(0, LUT_CMD_DDR, LUT_PAD(8), 0xA0) |
	       LUT_DEF(1, LUT_ADDR_DDR, LUT_PAD(8), 0x18),
	       lut_base + SEQ_IDX_READ * 16);
	writel(LUT_DEF(0, LUT_CADDR_DDR, LUT_PAD(8), 0x10) |
	       LUT_DEF(1, LUT_DUMMY_DDR, LUT_PAD(8), dummy_clocks),
	       lut_base + SEQ_IDX_READ * 16 + 4);
	writel(LUT_DEF(0, LUT_READ_DDR, LUT_PAD(8), 0x04) |
	       LUT_DEF(1, LUT_STOP, 0, 0),
	       lut_base + SEQ_IDX_READ * 16 + 8);

	/*
	 * Sequence 1: Write (CMD_DDR 0x20 + ADDR + CADDR + WRITE)
	 * 0x20 = HyperBus write, memory space
	 */
	writel(LUT_DEF(0, LUT_CMD_DDR, LUT_PAD(8), 0x20) |
	       LUT_DEF(1, LUT_ADDR_DDR, LUT_PAD(8), 0x18),
	       lut_base + SEQ_IDX_WRITE * 16);
	writel(LUT_DEF(0, LUT_CADDR_DDR, LUT_PAD(8), 0x10) |
	       LUT_DEF(1, LUT_WRITE_DDR, LUT_PAD(8), 0x02),
	       lut_base + SEQ_IDX_WRITE * 16 + 4);

	fspi_writel(priv, FSPI_LUTKEY_VALUE, FSPI_LUTKEY);
	fspi_writel(priv, FSPI_LCKER_LOCK, FSPI_LCKCR);

	/* DLL configuration based on clock rate */
	if (clk_rate > FSPI_DLL_CALIB_HZ)
		return fspi_dll_calibration(priv, dev);

	fspi_dll_override(priv);

	return 0;
}

/*
 * read16/write16: Raw 16-bit AHB access to HyperFlash.
 *
 * The HyperBus big-endian byte order is handled by setting map->swap =
 * CFI_BIG_ENDIAN, which makes the CFI layer byte-swap command/status
 * words in cfi_build_cmd()/cfi_merge_status(). This keeps the map I/O
 * functions raw, so data written to flash has the same byte order as
 * the source buffer.
 */
static u16 imxrt_hbmc_read16(struct hyperbus_device *hbdev, unsigned long addr)
{
	return __raw_readw(hbdev->map.virt + addr);
}

static void imxrt_hbmc_write16(struct hyperbus_device *hbdev,
			       unsigned long addr, u16 val)
{
	__raw_writew(val, hbdev->map.virt + addr);
}

/* Bulk data transfer — raw AHB access, no byte-swapping. */
static void imxrt_hbmc_copy_from(struct hyperbus_device *hbdev, void *to,
				 unsigned long from, ssize_t len)
{
	memcpy_fromio(to, hbdev->map.virt + from, len);
}

static void imxrt_hbmc_copy_to(struct hyperbus_device *hbdev, unsigned long to,
			       const void *from, ssize_t len)
{
	memcpy_toio(hbdev->map.virt + to, from, len);
}

/*
 * Derive flash size from partition layout: find the highest
 * (offset + size) across all partition subnodes.
 */
static u32 imxrt_hbmc_get_flash_size(struct device_node *flash_np)
{
	struct device_node *part;
	u32 flash_size = 0;

	for_each_child_of_node(flash_np, part) {
		u32 reg[2];

		if (!of_property_read_u32_array(part, "reg", reg, 2)) {
			u32 end = reg[0] + reg[1];

			if (end > flash_size)
				flash_size = end;
		}
	}

	return flash_size;
}

static const struct hyperbus_ops imxrt_hbmc_ops = {
	.read16 = imxrt_hbmc_read16,
	.write16 = imxrt_hbmc_write16,
	.copy_from = imxrt_hbmc_copy_from,
	.copy_to = imxrt_hbmc_copy_to,
};

static int imxrt_hbmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imxrt_hbmc_priv *priv;
	struct resource *res;
	u32 max_freq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->devtype = of_device_get_match_data(dev);

	/* Map FlexSPI registers */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fspi_base");
	priv->iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->iobase))
		return PTR_ERR(priv->iobase);

	/* Map AHB memory-mapped flash region */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fspi_mmap");
	priv->ahb_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->ahb_addr))
		return PTR_ERR(priv->ahb_addr);

	/* Get child node (flash device) */
	priv->hbdev.np = of_get_next_child(np, NULL);
	if (!priv->hbdev.np) {
		dev_err(dev, "no flash child node found\n");
		return -ENODEV;
	}

	/* Flash size from partition layout — determines FLSHA1CR0 and map boundary */
	priv->flash_size = imxrt_hbmc_get_flash_size(priv->hbdev.np);
	if (!priv->flash_size) {
		dev_err(dev, "no partitions found, cannot determine flash size\n");
		ret = -EINVAL;
		goto put_node;
	}
	dev_info(dev, "flash size %u MB (from partition layout)\n",
		 priv->flash_size / SZ_1M);

	/* Use differential clock if specified in controller node */
	priv->sck_diff = of_property_read_bool(np, "nxp-flexspi,differential-clock");

	/* Enable clocks */
	priv->clk = devm_clk_get(dev, "fspi");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_err(dev, "failed to get fspi clock: %d\n", ret);
		goto put_node;
	}

	priv->clk_en = devm_clk_get(dev, "fspi_en");
	if (IS_ERR(priv->clk_en)) {
		ret = PTR_ERR(priv->clk_en);
		dev_err(dev, "failed to get fspi_en clock: %d\n", ret);
		goto put_node;
	}

	/*
	 * If spi-max-frequency is specified, set the root clock rate
	 * before enabling. For DDR HyperBus, root clock = 2 x bus freq.
	 */
	if (!of_property_read_u32(priv->hbdev.np, "spi-max-frequency",
				  &max_freq)) {
		ret = clk_set_rate(priv->clk,
				   (unsigned long)max_freq * 2);
		if (ret) {
			dev_err(dev, "failed to set clock rate: %d\n", ret);
			goto put_node;
		}
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto put_node;

	ret = clk_prepare_enable(priv->clk_en);
	if (ret)
		goto disable_clk;

	/* Full FlexSPI init for HyperBus mode */
	ret = imxrt_hbmc_init_hw(priv, dev);
	if (ret)
		goto disable_clk_en;

	/*
	 * Map size must match FLSHA1CR0 — accesses beyond cause bus faults.
	 */
	priv->hbdev.map.size = priv->flash_size;
	priv->hbdev.map.virt = priv->ahb_addr;

	/*
	 * HyperBus is big-endian on the bus. Tell the CFI layer to
	 * byte-swap command/status words (in cfi_build_cmd/cfi_merge_status)
	 * while keeping raw map I/O.
	 */
	priv->hbdev.map.swap = CFI_BIG_ENDIAN;

	priv->ctlr.dev = dev;
	priv->ctlr.ops = &imxrt_hbmc_ops;
	priv->hbdev.ctlr = &priv->ctlr;

	ret = hyperbus_register_device(&priv->hbdev);
	if (ret) {
		dev_err(dev, "failed to register HyperBus device: %d\n", ret);
		goto disable_clk_en;
	}

	dev_info(dev, "i.MX RT HyperBus controller initialized\n");

	return 0;

disable_clk_en:
	clk_disable_unprepare(priv->clk_en);
disable_clk:
	clk_disable_unprepare(priv->clk);
put_node:
	of_node_put(priv->hbdev.np);
	return ret;
}

static void imxrt_hbmc_remove(struct platform_device *pdev)
{
	struct imxrt_hbmc_priv *priv = platform_get_drvdata(pdev);

	hyperbus_unregister_device(&priv->hbdev);
	clk_disable_unprepare(priv->clk_en);
	clk_disable_unprepare(priv->clk);
	of_node_put(priv->hbdev.np);
}

static const struct imxrt_hbmc_devtype imxrt1170_hbmc_data = {
	.has_refphasegap = true,
};

static const struct of_device_id imxrt_hbmc_dt_ids[] = {
	{ .compatible = "nxp,imxrt1170-fspi-hyperbus", .data = &imxrt1170_hbmc_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imxrt_hbmc_dt_ids);

static struct platform_driver imxrt_hbmc_platform_driver = {
	.probe = imxrt_hbmc_probe,
	.remove_new = imxrt_hbmc_remove,
	.driver = {
		.name = "hbmc-imxrt",
		.of_match_table = imxrt_hbmc_dt_ids,
	},
};

module_platform_driver(imxrt_hbmc_platform_driver);

MODULE_DESCRIPTION("HyperBus controller driver for i.MX RT SoCs");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hbmc-imxrt");
MODULE_AUTHOR("Vladimir Skvortsov <vskvortsov@emcraft.com>");
