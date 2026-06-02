// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/time64.h>

/* DSI CSR registers */
#define DSI_CLOCK_CNTL			0x0
#define  DPHY_CLKIN			BIT(2)
#define  DPHY_CLKOUT			BIT(3)

#define DSI_CLK_SETTING			0x8
#define  DPHY_REF_CLK_DIV(x)		FIELD_PREP(GENMASK(3, 0), (x))
#define  DPHY_REF_CLK_SRC		BIT(7)
#define  SRC_24MHZ_OR_25MHZ		0
#define  SRC_PIXEL_LINK_CLOCK		BIT(7)
#define  DPHY_PLL_DIV(x)		FIELD_PREP(GENMASK(11, 8), (x))

#define DSI_PHY_MODE_CONTROL		0x18
#define  RST_N				BIT(12)
#define  TEST_STOP_CLK_EN		BIT(11)
#define  SHUTDOWN_N			BIT(10)
#define  PHY_MODE			BIT(9)
#define  CPHY_MODE			BIT(9)
#define  DPHY_MODE			0
#define  PLL_GP_CLK_EN			BIT(3)
#define  PLL_CLKSEL_MASK		GENMASK(2, 1)
#define  PLL_CLKSEL_STOP		FIELD_PREP(PLL_CLKSEL_MASK, 0)
#define  PLL_CLKSEL_GEN			FIELD_PREP(PLL_CLKSEL_MASK, 1)
#define  PLL_CLKSEL_EXT			FIELD_PREP(PLL_CLKSEL_MASK, 2)
#define  PLL_ATB_SENSE_SEL		BIT(0)

#define DSI_PHY_FREQ_CONTROL		0x1c
#define  PHY_HSFREQRANGE(x)		FIELD_PREP(GENMASK(22, 16), (x))
#define  PHY_CFGCLKFREQRANGE(x)		FIELD_PREP(GENMASK(7, 0), (x))

#define DSI_PLL_CTRL0			0x2c
#define  M_MASK				GENMASK(11, 0)
#define  M(x)				FIELD_PREP(M_MASK, ((x) * 4))
#define  VCO_CTRL_MASK			GENMASK(19, 14)
#define  VCO_CTRL(x)			FIELD_PREP(VCO_CTRL_MASK, (x))
#define  PROP_CTRL_MASK			GENMASK(25, 20)
#define  PROP_CTRL(x)			FIELD_PREP(PROP_CTRL_MASK, (x))
#define  INT_CTRL_MASK			GENMASK(31, 26)
#define  INT_CTRL(x)			FIELD_PREP(INT_CTRL_MASK, (x))

#define DSI_PLL_CTRL1			0x30
#define  CPBIAS_CNTRL_MASK		GENMASK(8, 2)
#define  CPBIAS_CNTRL(x)		FIELD_PREP(CPBIAS_CNTRL_MASK, (x))
#define  GMP_CNTRL_MASK			GENMASK(1, 0)
#define  GMP_CNTRL(x)			FIELD_PREP(GMP_CNTRL_MASK, (x))

#define DSI_PLL_CTRL2			0x34
#define  N_MASK				GENMASK(25, 22)
#define  N(x)				FIELD_PREP(N_MASK, ((x) - 1))
#define  PLL_PRG_MASK			GENMASK(21, 5)
#define  PLL_PRG(x)			FIELD_PREP(PLL_PRG_MASK, (x))
#define  PLL_OPMODE_MASK		GENMASK(4, 0)
#define  PLL_OPMODE(x)			FIELD_PREP(PLL_OPMODE_MASK, (x))

#define DSI_PLL_CTRL3			0x38
#define  PLL_TH3_MASK			GENMASK(25, 18)
#define  PLL_TH3(x)			FIELD_PREP(PLL_TH3_MASK, (x))
#define  PLL_TH2_MASK			GENMASK(17, 10)
#define  PLL_TH2(x)			FIELD_PREP(PLL_TH2_MASK, (x))
#define  PLL_TH1_MASK			GENMASK(9, 0)
#define  PLL_TH1(x)			FIELD_PREP(PLL_TH1_MASK, (x))

#define DSI_PLL_STATUS1			0x50
#define  LOCK_PLL			BIT(10)

#define DSI_EXT_STATUS0			0x58
#define  DSI_PHY_READY			BIT(13)

#define DSI_PPI_EXT_CTRL0		0x60

#define DSI_PHY_PROP_CTRL		0x5
#define DSI_PHY_CPBIAS_CNTRL50		0x10
#define DSI_PHY_GMP_CNTRL10		0x01

#define MHZ(x)				((x) * 1000000UL)

#define FOUT_MAX			MHZ(1250)
#define FOUT_MIN			MHZ(40)

#define FVCO_MAX			MHZ(2250)
#define FVCO_MIN			MHZ(320)

#define MBPS(x)				((x) * 1000000UL)

#define DATA_RATE_MAX_SPEED		MBPS(2500)
#define DATA_RATE_MIN_SPEED		MBPS(80)

#define M_MAX				625UL
#define M_MIN				64UL

#define N_MAX				16U
#define N_MIN				1U

/* MIPI DPHY registers */
#define PPI_STARTUP_RW_COMMON_DPHY_2	0x3008
#define  RCAL_ADDR_MASK			GENMASK(7, 0)
#define  RCAL_ADDR(x)			FIELD_PREP(RCAL_ADDR_MASK, (x))

#define PPI_STARTUP_RW_COMMON_DPHY_3	0x300c
#define  PLL_START_ADDR_MASK		GENMASK(7, 0)
#define  PLL_START_ADDR(x)		FIELD_PREP(PLL_START_ADDR_MASK, (x))

#define PPI_STARTUP_RW_COMMON_DPHY_6	0x3018
#define  LP_DCO_CAL_ADDR_MASK		GENMASK(7, 0)
#define  LP_DCO_CAL_ADDR(x)		FIELD_PREP(LP_DCO_CAL_ADDR_MASK, (x))

#define PPI_STARTUP_RW_COMMON_DPHY_A	0x3028
#define  HIBERNATE_ADDR_MASK		GENMASK(7, 0)
#define  HIBERNATE_ADDR(x)		FIELD_PREP(HIBERNATE_ADDR_MASK, (x))

#define PPI_STARTUP_RW_COMMON_STARTUP_1	0x3044
#define  PHY_READY_DLY_MASK		GENMASK(11, 0)
#define  PHY_READY_DLY(x)		FIELD_PREP(PHY_READY_DLY_MASK, (x))

#define PPI_CALIBCTRL_RW_COMMON_BG_0	0x3098
#define  BG_MAX_COUNTER_MASK		GENMASK(8, 0)
#define  BG_MAX_COUNTER(x)		FIELD_PREP(BG_MAX_COUNTER_MASK, (x))

#define PPI_RW_LPDCOCAL_TIMEBASE	0x3804
#define  LPCDCOCAL_TIMEBASE_MASK	GENMASK(9, 0)
#define  LPCDCOCAL_TIMEBASE(x)		FIELD_PREP(LPCDCOCAL_TIMEBASE_MASK, (x))

#define PPI_RW_LPDCOCAL_NREF		0x3808
#define  LPCDCOCAL_NREF_MASK		GENMASK(10, 0)
#define  LPCDCOCAL_NREF(x)		FIELD_PREP(LPCDCOCAL_NREF_MASK, (x))

#define PPI_RW_LPDCOCAL_NREF_RANGE	0x380c
#define  LPCDCOCAL_NREF_RANGE_MASK	GENMASK(4, 0)
#define  LPCDCOCAL_NREF_RANGE(x)	FIELD_PREP(LPCDCOCAL_NREF_RANGE_MASK, (x))

#define PPI_RW_LPDCOCAL_TWAIT_CONFIG	0x3814
#define  LPCDCOCAL_TWAIT_COARSE_MASK	GENMASK(8, 0)
#define  LPCDCOCAL_TWAIT_COARSE(x)	FIELD_PREP(LPCDCOCAL_TWAIT_COARSE_MASK, (x))
#define  LPCDCOCAL_TWAIT_PON_MASK	GENMASK(15, 9)
#define  LPCDCOCAL_TWAIT_PON(x)		FIELD_PREP(LPCDCOCAL_TWAIT_PON_MASK, (x))

#define PPI_RW_LPDCOCAL_VT_CONFIG	0x3818
#define  LPCDCOCAL_TWAIT_FINE_MASK	GENMASK(15, 7)
#define  LPCDCOCAL_TWAIT_FINE(x)	FIELD_PREP(LPCDCOCAL_TWAIT_FINE_MASK, (x))
#define  LPCDCOCAL_VT_NREF_RANGE_MASK	GENMASK(6, 2)
#define  LPCDCOCAL_VT_NREF_RANGE(x)	FIELD_PREP(LPCDCOCAL_VT_NREF_RANGE_MASK, (x))

#define PPI_RW_LPDCOCAL_COARSE_CFG	0x3820
#define  SCALE_REF_MASK			GENMASK(8, 4)
#define  SCALE_REF(x)			FIELD_PREP(SCALE_REF_MASK, (x))
#define  NCOARSE_DIAG_MASK		GENMASK(3, 2)
#define  NCOARSE_DIAG(x)		FIELD_PREP(NCOARSE_DIAG_MASK, (x))
#define  NCOARSE_START_MASK		GENMASK(1, 0)
#define  NCOARSE_START(x)		FIELD_PREP(NCOARSE_START_MASK, (x))

#define PPI_RW_DTB_SELECTOR		0x38cc
#define  DTB_SOURCE_SELECT		BIT(8)
#define  DTB_SELECT_ADDR_MASK		GENMASK(7, 0)
#define  DTB_SELECT_ADDR(x)		FIELD_PREP(DTB_SELECT_ADDR_MASK, (x))

#define PPI_RW_COMMON_CFG		0x38d8
#define  CFG_CLK_DIV_FACTOR_MASK	GENMASK(1, 0)
#define  CFG_CLK_DIV_FACTOR(x)		FIELD_PREP(CFG_CLK_DIV_FACTOR_MASK, (x))

#define PPI_RW_TERMCAL_CFG_0		0x3900
#define  TERMCAL_TIMER_MASK		GENMASK(6, 0)
#define  TERMCAL_TIMER(x)		FIELD_PREP(TERMCAL_TIMER_MASK, (x))

#define PPI_RW_PLL_STARTUP_CFG_0	0x3980
#define  PLL_RST_TIME_MASK		GENMASK(9, 0)
#define  PLL_RST_TIME(x)		FIELD_PREP(PLL_RST_TIME_MASK, (x))

#define PPI_RW_PLL_STARTUP_CFG_1	0x3984
#define  PLL_GEAR_SHIFT_TIME_MASK	GENMASK(9, 0)
#define  PLL_GEAR_SHIFT_TIME(x)		FIELD_PREP(PLL_GEAR_SHIFT_TIME_MASK, (x))

#define PPI_RW_PLL_STARTUP_CFG_2	0x3988
#define  PLL_LOCK_DET_TIME_MASK		GENMASK(9, 0)
#define  PLL_LOCK_DET_TIME(x)		FIELD_PREP(PLL_LOCK_DET_TIME_MASK, (x))

#define DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2	0x4088
#define  OA_LANE0_SEL_LANE_CFG		BIT(0)

#define DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_3	0x408c
#define  OA_LANE0_HSTX_SEL_CLKLB	BIT(8)
#define  OA_LANE0_HSTX_SEL_PHASE0	BIT(4)

#define DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2	0x4888
#define  OA_LANE1_SEL_LANE_CFG		BIT(0)

#define DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_3	0x488c
#define  OA_LANE1_HSTX_SEL_CLKLB	BIT(8)
#define  OA_LANE1_HSTX_SEL_PHASE0	BIT(4)

#define DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2	0x5088
#define  OA_LANE2_SEL_LANE_CFG		BIT(0)

#define DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_3	0x508c
#define  OA_LANE2_HSTX_SEL_CLKLB	BIT(8)
#define  OA_LANE2_HSTX_SEL_PHASE0	BIT(4)

#define DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2	0x5888
#define  OA_LANE3_SEL_LANE_CFG		BIT(0)

#define DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_3	0x588c
#define  OA_LANE3_HSTX_SEL_CLKLB	BIT(8)
#define  OA_LANE3_HSTX_SEL_PHASE0	BIT(4)

#define DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2	0x6088
#define  OA_LANE4_SEL_LANE_CFG		BIT(0)

#define DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_3	0x608c
#define  OA_LANE4_HSTX_SEL_CLKLB	BIT(8)
#define  OA_LANE4_HSTX_SEL_PHASE0	BIT(4)

#define DIG_IOCTRL_RW_AFE_CB_CTRL_2_2	0x7088
#define  OA_CB_PLL_BUSTIEZ		BIT(15)

#define DIG_IOCTRL_RW_AFE_CB_CTRL_2_4	0x7090
#define  CB_SEL_VCOMMON_PROG_MASK	GENMASK(13, 11)
#define  CB_SEL_VCOMMON_PROG(x)		FIELD_PREP(CB_SEL_VCOMMON_PROG_MASK, (x))
#define  CB_SEL_HSRX_CM_DET_VREF_MASK	GENMASK(10, 9)
#define  CB_SEL_HSRX_CM_DET_VREF(x)	FIELD_PREP(CB_SEL_HSRX_CM_DET_VREF_MASK, (x))
#define  CB_SEL_TRIO2_ALP_VREF_MASK	GENMASK(8, 6)
#define  CB_SEL_TRIO2_ALP_VREF(x)	FIELD_PREP(CB_SEL_TRIO2_ALP_VREF_MASK, (x))
#define  CB_SEL_TRIO1_ALP_VREF_MASK	GENMASK(5, 3)
#define  CB_SEL_TRIO1_ALP_VREF(x)	FIELD_PREP(CB_SEL_TRIO1_ALP_VREF_MASK, (x))
#define  CB_SEL_TRIO0_ALP_VREF_MASK	GENMASK(2, 0)
#define  CB_SEL_TRIO0_ALP_VREF(x)	FIELD_PREP(CB_SEL_TRIO0_ALP_VREF_MASK, (x))

#define DIG_IOCTRL_RW_AFE_CB_CTRL_2_6	0x7098
#define  OA_CB_HSTXLB_DCO_PON_OVR_EN	BIT(12)

#define CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0	0x73c0
#define  CB_LP_DCO_EN_DLY_MASK		GENMASK(7, 2)
#define  CB_LP_DCO_EN_DLY(x)		FIELD_PREP(CB_LP_DCO_EN_DLY_MASK, (x))

#define CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2	0x73c8
#define  GLOBAL_ULPS_OVR_EN		BIT(12)

#define CORE_DIG_DLANE_RW_LP(l, x)	(0xc100 + 0x800 * (l) + 0x4 * (x))
#define  LP_0_ITMINRX_REG_MASK		GENMASK(15, 12)
#define  LP_0_ITMINRX_REG(x)		FIELD_PREP(LP_0_ITMINRX_REG_MASK, (x))
#define  LP_0_TTAGO_REG_MASK		GENMASK(11, 8)
#define  LP_0_TTAGO_REG(x)		FIELD_PREP(LP_0_TTAGO_REG_MASK, (x))
#define  LP_0_TTASURE_REG_MASK		GENMASK(7, 4)
#define  LP_0_TTASURE_REG(x)		FIELD_PREP(LP_0_TTASURE_REG_MASK, (x))
#define  LP_0_TTAGET_REG_MASK		GENMASK(3, 0)
#define  LP_0_TTAGET_REG(x)		FIELD_PREP(LP_0_TTAGET_REG_MASK, (x))

#define CORE_DIG_DLANE_RW_HS_TX(l, x)	(0xc400 + 0x800 * (l) + 0x4 * (x))
#define  HS_TX_0_THSTRAIL_REG_MASK	GENMASK(15, 0)
#define  HS_TX_0_THSTRAIL_REG(x)	FIELD_PREP(HS_TX_0_THSTRAIL_REG_MASK, (x))
#define  HS_TX_1_THSZERO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_1_THSZERO_REG(x)		FIELD_PREP(HS_TX_1_THSZERO_REG_MASK, (x))
#define  HS_TX_3_TLPTXOVERLAP_REG_MASK	GENMASK(7, 0)
#define  HS_TX_3_TLPTXOVERLAP_REG(x)	FIELD_PREP(HS_TX_3_TLPTXOVERLAP_REG_MASK, (x))
#define  HS_TX_4_TLPX_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_4_TLPX_DCO_REG(x)	FIELD_PREP(HS_TX_4_TLPX_DCO_REG_MASK, (x))
#define  HS_TX_5_THSTRAIL_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_5_THSTRAIL_DCO_REG(x)	FIELD_PREP(HS_TX_5_THSTRAIL_DCO_REG_MASK, (x))
#define  HS_TX_6_TLP11END_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_6_TLP11END_DCO_REG(x)	FIELD_PREP(HS_TX_6_TLP11END_DCO_REG_MASK, (x))
#define  HS_TX_9_THSPRPR_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_9_THSPRPR_DCO_REG(x)	FIELD_PREP(HS_TX_9_THSPRPR_DCO_REG_MASK, (x))
#define  HS_TX_10_TLP11INIT_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_10_TLP11INIT_DCO_REG(x)		FIELD_PREP(HS_TX_10_TLP11INIT_DCO_REG_MASK, (x))
#define  HS_TX_12_THSEXIT_DCO_REG_MASK	GENMASK(15, 0)
#define  HS_TX_12_THSEXIT_DCO_REG(x)	FIELD_PREP(HS_TX_12_THSEXIT_DCO_REG_MASK, (x))

#define CORE_DIG_DLANE_CLK_RW_LP(x)	(0xe100 + 0x4 * (x))

#define CORE_DIG_DLANE_CLK_RW_HS_TX(x)	(0xe400 + 0x4 * (x))
#define  HS_TX_2_TCLKPRE_REG_MASK	GENMASK(15, 0)
#define  HS_TX_2_TCLKPRE_REG(x)		FIELD_PREP(HS_TX_2_TCLKPRE_REG_MASK, (x))
#define  HS_TX_8_TCLKPOST_REG_MASK	GENMASK(15, 0)
#define  HS_TX_8_TCLKPOST_REG(x)	FIELD_PREP(HS_TX_8_TCLKPOST_REG_MASK, (x))

struct phy_pll_config {
	unsigned long fout;
	u32 m;
	u32 n;
	u32 p;
};

struct imx952_mipi_dphy_priv {
	struct device *dev;
	void __iomem *regs;
	struct regmap *dsi_csr;
	struct clk *clk_apb;
	struct clk *clk_cfg;
	struct clk *clk_ref;
	unsigned long clk_ref_rate;
	unsigned long clk_cfg_rate;

	/* clk provider */
	struct clk_hw hw;
	struct phy_pll_config cur_pll_cfg;
	int submode;
};

struct phy_pll_vco30 {
	unsigned long max_fout;
	u8 vco_cntl30;
};

struct phy_pll_hsfreqrange {
	unsigned long max_mbps;
	u8 hsfreqrange;
};

/* CDPHY Databook Table 3-7 PLL Configuration for a PHY Operating Range (N=1) */
static const struct phy_pll_vco30 vco30_map[] = {
	{   49, 0x2, },
	{   65, 0x1, },
	{   75, 0x3, },
	{   98, 0x2, },
	{  131, 0x1, },
	{  151, 0x3, },
	{  197, 0x2, },
	{  262, 0x1, },
	{  302, 0x3, },
	{  393, 0x2, },
	{  525, 0x1, },
	{  603, 0x3, },
	{  787, 0x2, },
	{ 1050, 0x1, },
	{ 1207, 0x3, },
	{ 1250, 0x2, },
};

/* i.MX952 TRM "Table: Bit rates and Phy_hsfreqrange" */
static const struct phy_pll_hsfreqrange hsfreqrange_map[] = {
	{   79, 0x00 },
	{   89, 0x10 },
	{   99, 0x20 },
	{  109, 0x30 },
	{  119, 0x01 },
	{  129, 0x11 },
	{  139, 0x21 },
	{  149, 0x31 },
	{  159, 0x02 },
	{  169, 0x12 },
	{  179, 0x22 },
	{  189, 0x32 },
	{  204, 0x03 },
	{  219, 0x13 },
	{  234, 0x23 },
	{  249, 0x33 },
	{  274, 0x04 },
	{  299, 0x14 },
	{  324, 0x25 },
	{  349, 0x35 },
	{  399, 0x05 },
	{  449, 0x16 },
	{  499, 0x26 },
	{  549, 0x37 },
	{  599, 0x07 },
	{  649, 0x18 },
	{  699, 0x28 },
	{  749, 0x39 },
	{  799, 0x09 },
	{  849, 0x19 },
	{  899, 0x29 },
	{  949, 0x3a },
	{  999, 0x0a },
	{ 1049, 0x1a },
	{ 1099, 0x2a },
	{ 1149, 0x3b },
	{ 1199, 0x0b },
	{ 1249, 0x1b },
	{ 1299, 0x2b },
	{ 1349, 0x3c },
	{ 1399, 0x0c },
	{ 1449, 0x1c },
	{ 1499, 0x2c },
	{ 1549, 0x3d },
	{ 1599, 0x0d },
	{ 1649, 0x1d },
	{ 1699, 0x2e },
	{ 1749, 0x3e },
	{ 1779, 0x0e },
	{ 1849, 0x1e },
	{ 1899, 0x2f },
	{ 1949, 0x3f },
	{ 1999, 0x0f },
	{ 2049, 0x40 },
	{ 2099, 0x41 },
	{ 2149, 0x42 },
	{ 2199, 0x43 },
	{ 2249, 0x44 },
	{ 2299, 0x45 },
	{ 2349, 0x46 },
	{ 2399, 0x47 },
	{ 2449, 0x48 },
	{ 2500, 0x49 },
};

static inline struct imx952_mipi_dphy_priv *
to_imx952_mipi_dphy_priv(struct clk_hw *hw)
{
	return container_of(hw, struct imx952_mipi_dphy_priv, hw);
}

static inline unsigned long data_rate_to_fout(unsigned long data_rate)
{
	/* Fout is half of data rate */
	return data_rate / 2;
}

static inline unsigned long fout_to_data_rate(unsigned long fout)
{
	/* data rate is two times Fout */
	return fout * 2;
}

static unsigned long
imx952_mipi_dphy_pll_find_mnp(struct imx952_mipi_dphy_priv *priv,
			      unsigned long fout, u32 *m, u32 *n, u32 *p)
{
	unsigned int min_n, max_n, _n, best_n, _p, best_p;
	unsigned long fin = priv->clk_ref_rate;
	unsigned long min_delta = ULONG_MAX;
	struct device *dev = priv->dev;
	unsigned long best_fout = 0;
	unsigned long _m, best_m;
	unsigned long delta;
	u64 tmp;

	/*
	 * CDPHY Databook 3.3.7 PLL Programmability:
	 * Fvco = (Fclkin * M) / N
	 * Fout = Fvco / P
	 * M = phy_m[11:0] / 4
	 * N = phy_n[3:0] + 1
	 * Fout = (Fclkin * (phy_m[11:0] / 4)) / ((phy_n[3:0] + 1) * P)
	 *
	 * P is controlled by phy_cpbias_cntrl[6] and phy_vco_cntrl[5:4].
	 * P could be 1/2/4/8/16/32.
	 */
	min_n = DIV_ROUND_UP_ULL((u64)fin * 10, MHZ(192));
	max_n = DIV_ROUND_DOWN_ULL((u64)fin * 10, MHZ(40));

	/* clamp possible N(s) */
	min_n = clamp(min_n, N_MIN, N_MAX);
	max_n = clamp(max_n, N_MIN, N_MAX);

	dev_dbg(dev, "Fout = %lu, n_range = [%u, %u]\n", fout, min_n, max_n);

	for (_p = 1; _p <= 32; _p *= 2) {
		/* check Fvco range */
		if (fout * _p > FVCO_MAX || fout * _p < FVCO_MIN)
			continue;

		for (_n = min_n; _n <= max_n; _n++) {
			/* M = (Fout * N * P) / Fin */
			_m = DIV_ROUND_CLOSEST(fout * _n * _p, fin);

			/* check M range */
			if (_m < M_MIN || _m > M_MAX)
				continue;

			/* calculate temporary Fout */
			tmp = _m * fin;
			do_div(tmp, _n * _p);
			if (tmp < FOUT_MIN || tmp > FOUT_MAX)
				continue;

			delta = abs(fout - tmp);
			if (delta < min_delta) {
				best_n = _n;
				best_m = _m;
				best_p = _p;
				min_delta = delta;
				best_fout = tmp;
			}
		}
	}

	if (best_fout) {
		*m = best_m;
		*n = best_n;
		*p = best_p;
		dev_dbg(dev, "best Fout = %lu, m = %u, n = %u, p = %u\n",
			best_fout, *m, *n, *p);
	} else {
		dev_dbg(dev, "failed to find best Fout\n");
		return 0;
	}

	return best_fout;
}

static u8
imx952_mipi_dphy_pll_get_hsfreqrange(const struct phy_pll_config *pll_cfg)
{
	unsigned long mbps = fout_to_data_rate(pll_cfg->fout) / MHZ(1);
	int i;

	for (i = 0; i < ARRAY_SIZE(hsfreqrange_map); i++)
		if (mbps <= hsfreqrange_map[i].max_mbps)
			return hsfreqrange_map[i].hsfreqrange;

	return 0;
}

static unsigned long
imx952_mipi_dphy_pll_get_cfgclkrange(struct imx952_mipi_dphy_priv *priv)
{
	/*
	 * i.MX952 TRM DSI_PHY_FREQ_CONTROL register description mentions an
	 * equation for cfgclkfreqrange.
	 */
	return (priv->clk_cfg_rate / MHZ(1) - 17) * 4;
}

/* CDPHY databook Table 3-5 VCO Ranges and Division Factors */
static u8
imx952_mipi_dphy_pll_get_vco_ctrl54(const struct phy_pll_config *pll_cfg)
{
	switch (pll_cfg->p) {
	case 1:
		return 0x0 << 4;
	case 2:
		return 0x3 << 4;
	case 4:
		return 0x1 << 4;
	case 8:
		return 0x2 << 4;
	case 16:
		return 0x3 << 4;
	case 32:
		return 0x0 << 4;
	}

	return 0;
}

static u8
imx952_mipi_dphy_pll_get_vco_ctrl30(const struct phy_pll_config *pll_cfg)
{
	unsigned long fout_mhz = pll_cfg->fout / MHZ(1);
	int i;

	for (i = 0; i < ARRAY_SIZE(vco30_map); i++)
		if (fout_mhz <= vco30_map[i].max_fout)
			return vco30_map[i].vco_cntl30;

	return 0;
}

/* CDPHY Databook Table 3-7 PLL Configuration for a PHY Operating Range (N=1) */
static u8
imx952_mipi_dphy_pll_get_int_ctrl(const struct phy_pll_config *pll_cfg)
{
	unsigned long fout_mhz = pll_cfg->fout / MHZ(1);

	return fout_mhz > 393 ? 0x0 : 0x1;
}

/* CDPHY databook Table 3-5 VCO Ranges and Division Factors */
static u8
imx952_mipi_dphy_pll_get_cpbias_cntrl6(const struct phy_pll_config *pll_cfg)
{
	switch (pll_cfg->p) {
	case 1:
	case 4:
	case 8:
	case 16:
		return 0x0 << 6;
	case 2:
	case 32:
		return 0x1 << 6;
	}

	return 0;
}

static void imx952_mipi_dphy_pll_configure(struct imx952_mipi_dphy_priv *priv)
{
	const struct phy_pll_config *cfg = &priv->cur_pll_cfg;
	u32 val;

	/* CDPHY databook Figure 7-6 Start-up Sequence Overview */
	/* T0: reset + shutdown */
	regmap_clear_bits(priv->dsi_csr, DSI_PHY_MODE_CONTROL,
			  RST_N | SHUTDOWN_N);

	/* T1 */
	/* 24MHz DPHY PLL clock source */
	regmap_write(priv->dsi_csr, DSI_CLK_SETTING, 0);

	/* CDPHY databook Appendix A, "Static Input Configuration" */
	regmap_update_bits(priv->dsi_csr, DSI_PHY_MODE_CONTROL,
			   PLL_CLKSEL_MASK | PHY_MODE,
			   PLL_CLKSEL_GEN | DPHY_MODE);

	/* T2 */
	val = PHY_CFGCLKFREQRANGE(imx952_mipi_dphy_pll_get_cfgclkrange(priv)) |
	      PHY_HSFREQRANGE(imx952_mipi_dphy_pll_get_hsfreqrange(cfg));
	regmap_write(priv->dsi_csr, DSI_PHY_FREQ_CONTROL, val);

	/* M and P */
	val = M(cfg->m) |
	      INT_CTRL(imx952_mipi_dphy_pll_get_int_ctrl(cfg)) |
	      VCO_CTRL(imx952_mipi_dphy_pll_get_vco_ctrl54(cfg) |
		       imx952_mipi_dphy_pll_get_vco_ctrl30(cfg)) |
	      PROP_CTRL(DSI_PHY_PROP_CTRL);
	regmap_write(priv->dsi_csr, DSI_PLL_CTRL0, val);

	/* P */
	val = CPBIAS_CNTRL(imx952_mipi_dphy_pll_get_cpbias_cntrl6(cfg) |
			   DSI_PHY_CPBIAS_CNTRL50) |
			   GMP_CNTRL(DSI_PHY_GMP_CNTRL10);
	regmap_update_bits(priv->dsi_csr, DSI_PLL_CTRL1,
			   CPBIAS_CNTRL_MASK | GMP_CNTRL_MASK, val);

	/* N */
	regmap_update_bits(priv->dsi_csr, DSI_PLL_CTRL2, N_MASK, N(cfg->n));

	/* PRG */
	regmap_update_bits(priv->dsi_csr, DSI_PLL_CTRL2,
			   PLL_PRG_MASK, PLL_PRG(0x3));

	/* OPMODE */
	regmap_update_bits(priv->dsi_csr, DSI_PLL_CTRL2,
			   PLL_OPMODE_MASK, PLL_OPMODE(0x10));

	/* PLL lock detect threshold control */
	regmap_update_bits(priv->dsi_csr, DSI_PLL_CTRL3,
			   PLL_TH1_MASK | PLL_TH2_MASK | PLL_TH3_MASK,
			   PLL_TH1(0x1) | PLL_TH2(0xff) | PLL_TH3(0x3));
}

static int imx952_mipi_dphy_pll_prepare(struct clk_hw *hw)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);
	int ret;

	ret = clk_prepare_enable(priv->clk_ref);
	if (ret < 0) {
		dev_err(priv->dev, "failed to enable ref clk: %d\n", ret);
		return ret;
	}

	return 0;
}

static void imx952_mipi_dphy_pll_unprepare(struct clk_hw *hw)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);

	clk_disable_unprepare(priv->clk_ref);
}

static int imx952_mipi_dphy_pll_enable(struct clk_hw *hw)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);

	/* gate clocks */
	regmap_write(priv->dsi_csr, DSI_CLOCK_CNTL, DPHY_CLKOUT | DPHY_CLKIN);

	imx952_mipi_dphy_pll_configure(priv);

	/* Ungate clocks */
	regmap_write(priv->dsi_csr, DSI_CLOCK_CNTL, 0);

	return 0;
}

static void imx952_mipi_dphy_pll_disable(struct clk_hw *hw)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);

	/* set 1 to disable */
	regmap_write(priv->dsi_csr, DSI_CLOCK_CNTL, DPHY_CLKOUT | DPHY_CLKIN);
}

static unsigned long
imx952_mipi_dphy_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);

	return priv->cur_pll_cfg.fout;
}

static int imx952_mipi_dphy_pll_find_settings(struct imx952_mipi_dphy_priv *priv,
					      struct phy_pll_config *pll_cfg,
					      unsigned long rate)
{
	if (rate > data_rate_to_fout(DATA_RATE_MAX_SPEED) ||
	    rate < data_rate_to_fout(DATA_RATE_MIN_SPEED))
		return -EINVAL;

	pll_cfg->fout = imx952_mipi_dphy_pll_find_mnp(priv, rate, &pll_cfg->m,
						      &pll_cfg->n, &pll_cfg->p);
	if (pll_cfg->fout == 0)
		return -EINVAL;

	return 0;
}

static long
imx952_mipi_dphy_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);
	struct phy_pll_config pll_cfg;
	int ret;

	ret = imx952_mipi_dphy_pll_find_settings(priv, &pll_cfg, rate);
	if (ret)
		return 0;

	return pll_cfg.fout;
}

static int
imx952_mipi_dphy_pll_set_rate(struct clk_hw *hw,
			      unsigned long rate, unsigned long parent_rate)
{
	struct imx952_mipi_dphy_priv *priv = to_imx952_mipi_dphy_priv(hw);
	struct phy_pll_config pll_cfg;
	int ret;

	ret = imx952_mipi_dphy_pll_find_settings(priv, &pll_cfg, rate);
	if (ret)
		return ret;

	memcpy(&priv->cur_pll_cfg, &pll_cfg, sizeof(pll_cfg));

	return 0;
}

static const struct clk_ops dphy_pll_ops = {
	.prepare = imx952_mipi_dphy_pll_prepare,
	.unprepare = imx952_mipi_dphy_pll_unprepare,
	.enable = imx952_mipi_dphy_pll_enable,
	.disable = imx952_mipi_dphy_pll_disable,
	.recalc_rate = imx952_mipi_dphy_pll_recalc_rate,
	.round_rate = imx952_mipi_dphy_pll_round_rate,
	.set_rate = imx952_mipi_dphy_pll_set_rate,
};

static int imx952_mipi_dphy_pll_register(struct imx952_mipi_dphy_priv *priv)
{
	struct device *dev = priv->dev;
	struct clk_init_data init;
	const char *parent_name;
	struct clk *phyclk;
	int ret;

	parent_name = __clk_get_name(priv->clk_ref);

	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = 0;
	init.name = "dphy_pll";
	init.ops = &dphy_pll_ops;

	priv->hw.init = &init;

	phyclk = devm_clk_register(dev, &priv->hw);
	if (IS_ERR(phyclk))
		return dev_err_probe(dev, PTR_ERR(phyclk),
				     "failed to register clock\n");

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, phyclk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register clock provider\n");

	return 0;
}

static int imx952_mipi_dphy_init(struct phy *phy)
{
	struct imx952_mipi_dphy_priv *priv = phy_get_drvdata(phy);
	u16 val;
	int ret;

	ret = pm_runtime_get_sync(&phy->dev);
	if (ret < 0) {
		dev_err(&phy->dev, "failed to get PM runtime: %d\n", ret);
		return ret;
	}

	val = DTB_SOURCE_SELECT | DTB_SELECT_ADDR(0xe0);
	writew(val, priv->regs + PPI_RW_DTB_SELECTOR);

	return 0;
}

static int imx952_mipi_dphy_exit(struct phy *phy)
{
	pm_runtime_put(&phy->dev);

	return 0;
}

static int imx952_mipi_dphy_power_off(struct phy *phy)
{
	struct imx952_mipi_dphy_priv *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->hw.clk);

	return 0;
}

static void
imx952_mipi_dphy_static_configure(struct imx952_mipi_dphy_priv *priv)
{
	u16 val;

	val = readw(priv->regs + CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0);
	val &= ~CB_LP_DCO_EN_DLY_MASK;
	val |= CB_LP_DCO_EN_DLY(63);
	writew(val, priv->regs + CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_0);

	val = PHY_READY_DLY(563);
	writew(val, priv->regs + PPI_STARTUP_RW_COMMON_STARTUP_1);

	val = RCAL_ADDR(0x3);
	writew(val, priv->regs + PPI_STARTUP_RW_COMMON_DPHY_2);

	val = PLL_START_ADDR(0x26);
	writew(val, priv->regs + PPI_STARTUP_RW_COMMON_DPHY_3);

	val = LP_DCO_CAL_ADDR(0x10);
	writew(val, priv->regs + PPI_STARTUP_RW_COMMON_DPHY_6);

	val = HIBERNATE_ADDR(0x21);
	writew(val, priv->regs + PPI_STARTUP_RW_COMMON_DPHY_A);

	val = readw(priv->regs + CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2);
	val |= GLOBAL_ULPS_OVR_EN;
	writew(val, priv->regs + CORE_DIG_ANACTRL_RW_COMMON_ANACTRL_2);

	val = BG_MAX_COUNTER(500);
	writew(val, priv->regs + PPI_CALIBCTRL_RW_COMMON_BG_0);

	val = TERMCAL_TIMER(23);
	writew(val, priv->regs + PPI_RW_TERMCAL_CFG_0);

	val = LPCDCOCAL_TIMEBASE(95);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_TIMEBASE);

	val = LPCDCOCAL_NREF(800);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_NREF);

	val = LPCDCOCAL_NREF_RANGE(15);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_NREF_RANGE);

	val = LPCDCOCAL_TWAIT_COARSE(29) | LPCDCOCAL_TWAIT_PON(127);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_TWAIT_CONFIG);

	val = LPCDCOCAL_TWAIT_FINE(29) | LPCDCOCAL_VT_NREF_RANGE(15);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_VT_CONFIG);

	val = SCALE_REF(0x10) | NCOARSE_DIAG(0x1) | NCOARSE_START(0x1);
	writew(val, priv->regs + PPI_RW_LPDCOCAL_COARSE_CFG);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_CB_CTRL_2_2);
	val |= OA_CB_PLL_BUSTIEZ;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_CB_CTRL_2_2);

	val = PLL_RST_TIME(239);
	writew(val, priv->regs + PPI_RW_PLL_STARTUP_CFG_0);

	val = PLL_GEAR_SHIFT_TIME(119);
	writew(val, priv->regs + PPI_RW_PLL_STARTUP_CFG_1);

	val = PLL_LOCK_DET_TIME(0);
	writew(val, priv->regs + PPI_RW_PLL_STARTUP_CFG_2);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_3);
	val &= ~OA_LANE0_HSTX_SEL_CLKLB;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_3);
	val &= ~OA_LANE1_HSTX_SEL_CLKLB;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_3);
	val &= ~OA_LANE2_HSTX_SEL_CLKLB;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_3);
	val &= ~OA_LANE3_HSTX_SEL_CLKLB;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_3);
	val &= ~OA_LANE4_HSTX_SEL_CLKLB;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_CB_CTRL_2_6);
	val |= OA_CB_HSTXLB_DCO_PON_OVR_EN;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_CB_CTRL_2_6);

	val = CFG_CLK_DIV_FACTOR(0x3);
	writew(val, priv->regs + PPI_RW_COMMON_CFG);
}

#define LPTX_EN_DLY			5
#define D2A_HSTX_DLY			3

/* T_DCO_MAX = 4.77 */
#define T_DCO_MAX			477
#define T_DCO_MAX_NS_DIV_ROUND_UP(x)	DIV_ROUND_UP((x) * 100, T_DCO_MAX)
#define T_DCO_MAX_PS_DIV_ROUND_UP(x)	DIV_ROUND_UP((x) * 100, T_DCO_MAX * 1000)
#define T_DCO_MAX_PS_DIV_ROUND_DOWN(x)	DIV_ROUND_DOWN_ULL((x) * 100, T_DCO_MAX * 1000)
#define T_DCO_MAX_PS_MULTI(x)		((T_DCO_MAX * 1000 * (x)) / 100)

/* T_DCO_CUSTOM = 5.02 */
#define T_DCO_CUSTOM			502
#define T_DCO_CUSTOM_PS_DIV_ROUND_UP(x)	DIV_ROUND_UP((x) * 100, T_DCO_CUSTOM * 1000)

#define T_CLK_PREPARE_NS_MIN		38
#define T_CLK_PREPARE_NS_MAX		95
#define T_CLK_PREPARE_PS_MIN		(T_CLK_PREPARE_NS_MIN * 1000)
#define T_CLK_PREPARE_PS_MAX		(T_CLK_PREPARE_NS_MAX * 1000)

#define T_CLK_PREPARE_CLK_ZERO_NS_MIN	300
#define T_CLK_PREPARE_CLK_ZERO_PS_MIN	(T_CLK_PREPARE_CLK_ZERO_NS_MIN * 1000)

#define T_LPX_NS_MIN			50
#define T_LPX_PS_MIN			(50 * 1000)
#define T_HS_EXIT_NS_MIN		100

#define LPTX_IO_SR0_FALL_DLY_PS		25000
#define LPTX_IO_SR0_FALL_DLY_DIV2_PS	(LPTX_IO_SR0_FALL_DLY_PS / 2)

static inline unsigned long
multi(struct imx952_mipi_dphy_priv *priv, unsigned long x)
{
	return x * priv->submode / 10;
}

static void
imx952_mipi_dphy_dynamic_configure(struct imx952_mipi_dphy_priv *priv,
				   struct phy_configure_opts_mipi_dphy *cfg)
{
	unsigned long t_hs_trail_ps, eot_ps, hs_trail_reg, hs_trail_dco_reg;
	unsigned long t_clk_trail_ps, clk_trail_reg, clk_trail_dco_reg;
	unsigned long lptx_clk_rate_hz = PSEC_PER_SEC / cfg->lpx;
	unsigned long clk_prepare_dco_ps, clk_prepare_dco_reg;
	unsigned long hs_prepare_dco_ps, hs_prepare_dco_reg;
	unsigned long fout = priv->cur_pll_cfg.fout;
	unsigned long clk_post_ps, clk_post_reg;
	unsigned long clk_zero_ps, clk_zero_reg;
	unsigned long hs_zero_ps, hs_zero_reg;
	unsigned long tlp11init_dco_reg;
	unsigned long wordclk_period_ps;
	unsigned long tlptxoverlap_reg;
	unsigned long hs_exit_dco_reg;
	unsigned long tlpx_dco_reg;
	unsigned long hs_clk_rate;
	unsigned long ui;
	u16 val;

	hs_clk_rate = fout_to_data_rate(fout);

	ui = ALIGN(PSEC_PER_SEC, hs_clk_rate);
	do_div(ui, hs_clk_rate);

	wordclk_period_ps = ui * 8;

	val = LP_0_ITMINRX_REG(0x4) | LP_0_TTAGO_REG(0x6) |
	      LP_0_TTASURE_REG(0x3) | LP_0_TTAGET_REG(0xc);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_LP(0, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_LP(1, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_LP(2, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_LP(3, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_LP(0));

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2);
	val &= ~OA_LANE0_SEL_LANE_CFG;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_2);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2);
	val &= ~OA_LANE1_SEL_LANE_CFG;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_2);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2);
	val |= OA_LANE2_SEL_LANE_CFG;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_2);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2);
	val &= ~OA_LANE3_SEL_LANE_CFG;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_2);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2);
	val &= ~OA_LANE4_SEL_LANE_CFG;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_2);

	val = CB_SEL_VCOMMON_PROG(0x4) | CB_SEL_HSRX_CM_DET_VREF(0x1) |
	      CB_SEL_TRIO2_ALP_VREF(0x2) | CB_SEL_TRIO1_ALP_VREF(0x2) |
	      CB_SEL_TRIO0_ALP_VREF(0x2);
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_CB_CTRL_2_4);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_3);
	val |= OA_LANE0_HSTX_SEL_PHASE0;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE0_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_3);
	val |= OA_LANE1_HSTX_SEL_PHASE0;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE1_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_3);
	val &= ~OA_LANE2_HSTX_SEL_PHASE0;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE2_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_3);
	val |= OA_LANE3_HSTX_SEL_PHASE0;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE3_CTRL_2_3);

	val = readw(priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_3);
	val |= OA_LANE4_HSTX_SEL_PHASE0;
	writew(val, priv->regs + DIG_IOCTRL_RW_AFE_LANE4_CTRL_2_3);

	/* tlptxoverlap_reg */
	tlptxoverlap_reg = T_DCO_MAX_NS_DIV_ROUND_UP(LPTX_EN_DLY);
	val = HS_TX_3_TLPTXOVERLAP_REG(tlptxoverlap_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 3));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 3));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 3));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 3));

	/* tlp11init_dco_reg */
	tlp11init_dco_reg = T_DCO_MAX_NS_DIV_ROUND_UP(5 * NSEC_PER_SEC / lptx_clk_rate_hz) - 1;
	val = HS_TX_10_TLP11INIT_DCO_REG(tlp11init_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 10));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 10));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 10));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 10));

	/* tlpx_dco_reg */
	tlpx_dco_reg = T_DCO_MAX_NS_DIV_ROUND_UP(multi(priv, T_LPX_NS_MIN)) - 1;
	val = HS_TX_4_TLPX_DCO_REG(tlpx_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 4));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 4));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 4));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 4));

	/* hs_prepare_dco_reg */
	hs_prepare_dco_ps = 40000 + 4 * ui +
		DIV_ROUND_DOWN_ULL((85000 + 6 * ui) - (40000 + 4 * ui), 2);
	hs_prepare_dco_reg =
		T_DCO_CUSTOM_PS_DIV_ROUND_UP(hs_prepare_dco_ps + LPTX_IO_SR0_FALL_DLY_DIV2_PS) - 1;
	val = HS_TX_9_THSPRPR_DCO_REG(hs_prepare_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 9));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 9));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 9));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 9));

	/* hs_zero_reg */
	hs_zero_ps = 145000 + 10 * ui - hs_prepare_dco_ps;
	hs_zero_reg = DIV_ROUND_UP_ULL((multi(priv, T_LPX_PS_MIN) + hs_prepare_dco_ps + hs_zero_ps +
					T_DCO_MAX_PS_MULTI(5) - 3 * wordclk_period_ps),
				       wordclk_period_ps) - 1;
	val = HS_TX_1_THSZERO_REG(hs_zero_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 1));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 1));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 1));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 1));

	/* hs_trail_reg */
	t_hs_trail_ps = max(8 * ui, 60000 + 4 * ui);	/* n = 1, see cfg->hs_trail */
	eot_ps = 105000 + 12 * ui;
	t_hs_trail_ps = t_hs_trail_ps + (eot_ps - t_hs_trail_ps) / 2;
	hs_trail_reg = DIV_ROUND_UP(t_hs_trail_ps, wordclk_period_ps) - 1 + D2A_HSTX_DLY;
	val = HS_TX_0_THSTRAIL_REG(hs_trail_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 0));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 0));

	/* hs_trail_dco_reg */
	hs_trail_dco_reg = T_DCO_MAX_PS_DIV_ROUND_DOWN(hs_trail_reg * wordclk_period_ps -
						       T_DCO_MAX_PS_MULTI(4)) - 1;
	val = HS_TX_5_THSTRAIL_DCO_REG(hs_trail_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 5));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 5));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 5));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 5));

	/* tlp11end_dco_reg */
	val = HS_TX_6_TLP11END_DCO_REG(tlp11init_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 6));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 6));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 6));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 6));

	/* hs_exit_dco_reg */
	hs_exit_dco_reg = T_DCO_MAX_NS_DIV_ROUND_UP(multi(priv, T_HS_EXIT_NS_MIN)) - 1;
	val = HS_TX_12_THSEXIT_DCO_REG(hs_exit_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(0, 12));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(1, 12));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(2, 12));
	writew(val, priv->regs + CORE_DIG_DLANE_RW_HS_TX(3, 12));

	/* clock lane */
	val = HS_TX_3_TLPTXOVERLAP_REG(tlptxoverlap_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(3));
	val = HS_TX_10_TLP11INIT_DCO_REG(tlp11init_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(10));
	val = HS_TX_4_TLPX_DCO_REG(tlpx_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(4));

	/* clk_prepare_dco_reg */
	clk_prepare_dco_ps = T_CLK_PREPARE_PS_MIN +
		DIV_ROUND_DOWN_ULL(T_CLK_PREPARE_PS_MAX - T_CLK_PREPARE_PS_MIN, 2);
	clk_prepare_dco_reg =
		T_DCO_CUSTOM_PS_DIV_ROUND_UP(clk_prepare_dco_ps + LPTX_IO_SR0_FALL_DLY_DIV2_PS) - 1;
	val = HS_TX_9_THSPRPR_DCO_REG(clk_prepare_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(9));

	/* clk_zero_reg */
	clk_zero_ps = multi(priv, T_CLK_PREPARE_CLK_ZERO_PS_MIN - clk_prepare_dco_ps);
	clk_zero_reg = DIV_ROUND_UP_ULL((multi(priv, T_LPX_PS_MIN) + clk_prepare_dco_ps +
		clk_zero_ps + T_DCO_MAX_PS_MULTI(5) - 3 * wordclk_period_ps), wordclk_period_ps)
		- 1;
	val = HS_TX_1_THSZERO_REG(clk_zero_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(1));

	val = HS_TX_2_TCLKPRE_REG(D2A_HSTX_DLY);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(2));

	t_clk_trail_ps = 60000 + (eot_ps - 60000) / 2;
	clk_trail_reg = DIV_ROUND_UP(t_clk_trail_ps, wordclk_period_ps) - 1 + D2A_HSTX_DLY;
	val = HS_TX_0_THSTRAIL_REG(clk_trail_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(0));

	clk_trail_dco_reg = T_DCO_MAX_PS_DIV_ROUND_DOWN(clk_trail_reg * wordclk_period_ps -
							T_DCO_MAX_PS_MULTI(4)) - 1;
	val = HS_TX_5_THSTRAIL_DCO_REG(clk_trail_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(5));

	clk_post_ps = multi(priv, 60000 + 52 * ui);
	clk_post_reg = DIV_ROUND_UP_ULL(clk_post_ps, wordclk_period_ps) - 3;
	val = HS_TX_8_TCLKPOST_REG(clk_post_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(8));

	val = HS_TX_6_TLP11END_DCO_REG(tlp11init_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(6));
	val = HS_TX_12_THSEXIT_DCO_REG(hs_exit_dco_reg);
	writew(val, priv->regs + CORE_DIG_DLANE_CLK_RW_HS_TX(12));

	dev_dbg(priv->dev, "fout: %luHz, ui: %lups, tlptxoverlap_reg: 0x%lx, "
		"tlp11init_dco_reg: 0x%lx, tlpx_dco_reg: 0x%lx, "
		"hs_prepare_dco_reg: 0x%lx, hs_trail_dco_reg: 0x%lx, "
		"hs_zero_reg: 0x%lx, hs_trail_reg: 0x%lx, "
		"hs_exit_dco_reg: 0x%lx, clk_zero_reg: 0x%lx, "
		"clk_prepare_dco_reg: 0x%lx, clk_trail_reg: 0x%lx, "
		"clk_trail_dco_reg: 0x%lx, clk_post_reg: 0x%lx\n",
		fout, ui, tlptxoverlap_reg, tlp11init_dco_reg, tlpx_dco_reg,
		hs_prepare_dco_reg, hs_trail_dco_reg, hs_zero_reg, hs_trail_reg,
		hs_exit_dco_reg, clk_zero_reg, clk_prepare_dco_reg,
		clk_trail_reg, clk_trail_dco_reg, clk_post_reg);
}

static int
imx952_mipi_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct phy_configure_opts_mipi_dphy *dphy_opts = &opts->mipi_dphy;
	unsigned long fout = data_rate_to_fout(dphy_opts->hs_clk_rate);
	struct imx952_mipi_dphy_priv *priv = phy_get_drvdata(phy);
	u32 val;
	int ret;

	ret = clk_set_rate(priv->hw.clk, fout);
	if (ret) {
		dev_err(priv->dev, "failed to set PHY PLL fout %lu: %d\n",
			fout, ret);
		return ret;
	}

	/* TO ~ T2 */
	ret = clk_prepare_enable(priv->hw.clk);
	if (ret) {
		dev_err(priv->dev, "failed to enable PHY PLL: %d\n", ret);
		return ret;
	}

	/* T3 */
	imx952_mipi_dphy_static_configure(priv);
	imx952_mipi_dphy_dynamic_configure(priv, dphy_opts);

	/* forcetxstopmode */
	regmap_write(priv->dsi_csr, DSI_PPI_EXT_CTRL0,
		     BIT(6) | BIT(5) |  BIT(4) | BIT(3) | BIT(1));

	/* T4 */
	regmap_update_bits(priv->dsi_csr, DSI_PHY_MODE_CONTROL,
			   RST_N | SHUTDOWN_N, RST_N | SHUTDOWN_N);

	/* T5 */
	/* wait for PHY PLL lock */
	ret = regmap_read_poll_timeout(priv->dsi_csr, DSI_PLL_STATUS1,
				       val, val & LOCK_PLL, 5, 10000);
	if (ret) {
		dev_err(priv->dev, "lock PLL timeout\n");
		return ret;
	}

	/* wait for PHY ready and stopstate L0 */
	ret = regmap_read_poll_timeout(priv->dsi_csr, DSI_EXT_STATUS0, val,
				       val & DSI_PHY_READY, 5, 10000);
	if (ret) {
		dev_err(priv->dev, "failed to wait for PHY ready and stopstate L0\n");
		return ret;
	}

	/* T6: No dynamic register to be programmed. */

	/* T7 */
	/* clear forcetxstopmode */
	regmap_write(priv->dsi_csr, DSI_PPI_EXT_CTRL0, 0);

	return 0;
}

static int
imx952_mipi_dphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct imx952_mipi_dphy_priv *priv = phy_get_drvdata(phy);

	priv->submode = submode;
	dev_dbg(priv->dev, "set submode %d\n", submode);

	return 0;
}

static int
imx952_mipi_dphy_validate(struct phy *phy, enum phy_mode mode, int submode,
			  union phy_configure_opts *opts)
{
	struct phy_configure_opts_mipi_dphy *dphy_opts = &opts->mipi_dphy;
	struct imx952_mipi_dphy_priv *priv = phy_get_drvdata(phy);
	unsigned long fout, rounded_fout;

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	fout = data_rate_to_fout(dphy_opts->hs_clk_rate);

	rounded_fout = clk_round_rate(priv->hw.clk, fout);
	if (rounded_fout == 0) {
		dev_dbg(priv->dev, "failed to round PHY PLL fout %lu\n", fout);
		return -EINVAL;
	}

	return 0;
}

static const struct phy_ops imx952_mipi_dphy_phy_ops = {
	.init = imx952_mipi_dphy_init,
	.exit = imx952_mipi_dphy_exit,
	.power_off = imx952_mipi_dphy_power_off,
	.set_mode = imx952_mipi_dphy_set_mode,
	.configure = imx952_mipi_dphy_configure,
	.validate = imx952_mipi_dphy_validate,
	.owner = THIS_MODULE,
};

static int imx952_mipi_dphy_probe(struct platform_device *pdev)
{
	struct imx952_mipi_dphy_priv *priv;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct phy *phy;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = dev;

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->dsi_csr = syscon_regmap_lookup_by_phandle(dev->of_node,
							"nxp,blk-ctrl");
	if (IS_ERR(priv->dsi_csr))
		return dev_err_probe(dev, PTR_ERR(priv->dsi_csr),
				     "failed to get display dsi csr\n");

	priv->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(priv->clk_apb))
		return dev_err_probe(dev, PTR_ERR(priv->clk_apb),
				     "failed to get apb clk\n");

	priv->clk_cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(priv->clk_cfg))
		return dev_err_probe(dev, PTR_ERR(priv->clk_cfg),
				     "failed to get cfg clk\n");

	priv->clk_ref = devm_clk_get(dev, "ref");
	if (IS_ERR(priv->clk_ref))
		return dev_err_probe(dev, PTR_ERR(priv->clk_ref),
				     "failed to get ref clk\n");

	priv->clk_ref_rate = clk_get_rate(priv->clk_ref);
	dev_dbg(dev, "phy ref clk rate: %lu\n", priv->clk_ref_rate);

	priv->clk_cfg_rate = clk_get_rate(priv->clk_cfg);
	dev_dbg(dev, "phy cfg clk rate: %lu\n", priv->clk_cfg_rate);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = imx952_mipi_dphy_pll_register(priv);
	if (ret)
		return dev_err_probe(dev, ret, "register clk failed\n");

	phy = devm_phy_create(dev, dev->of_node, &imx952_mipi_dphy_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create PHY\n");

	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return dev_err_probe(dev, PTR_ERR(phy_provider),
				     "failed to register PHY provider\n");

	return 0;
}

static int imx952_mipi_dphy_suspend(struct device *dev)
{
	struct imx952_mipi_dphy_priv *priv = dev_get_drvdata(dev);

	clk_disable_unprepare(priv->clk_cfg);
	clk_disable_unprepare(priv->clk_apb);

	return 0;
}

static int imx952_mipi_dphy_resume(struct device *dev)
{
	struct imx952_mipi_dphy_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(priv->clk_apb);
	if (ret) {
		dev_err(priv->dev, "failed to enable apb clk\n");
		return ret;
	}

	ret = clk_prepare_enable(priv->clk_cfg);
	if (ret) {
		dev_err(priv->dev, "failed to enable cfg clk: %d\n", ret);
		clk_disable_unprepare(priv->clk_apb);
		return ret;
	}

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx952_mipi_dphy_pm_ops,
				 imx952_mipi_dphy_suspend,
				 imx952_mipi_dphy_resume, NULL);

static const struct of_device_id imx952_mipi_dphy_of_match[] = {
	{ .compatible = "nxp,imx952-mipi-dphy", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx952_mipi_dphy_of_match);

static struct platform_driver imx952_mipi_dphy_driver = {
	.probe = imx952_mipi_dphy_probe,
	.driver = {
		.name = "imx952-mipi-dphy",
		.of_match_table = imx952_mipi_dphy_of_match,
		.pm = pm_ptr(&imx952_mipi_dphy_pm_ops),
	},
};
module_platform_driver(imx952_mipi_dphy_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Freescale i.MX952 Synopsys DesignWare MIPI DPHY driver");
MODULE_LICENSE("GPL");
