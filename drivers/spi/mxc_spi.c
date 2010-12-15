/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-licensisr_locke.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup SPI Configurable Serial Peripheral Interface (CSPI) Driver
 */

/*!
 * @file mxc_spi.c
 * @brief This file contains the implementation of the SPI master controller services
 *
 *
 * @ingroup SPI
 */

#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/fsl_devices.h>

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPICONFIG		0x08
#define MXC_CSPIINT			0x0C

#define MXC_CSPICTRL_DISABLE	0x0
#define MXC_CSPICTRL_SLAVE	0x0
#define MXC_CSPICTRL_CSMASK	0x3
#define MXC_CSPICTRL_SMC	(1 << 3)

#define MXC_CSPIINT_TEEN_SHIFT		0
#define MXC_CSPIINT_THEN_SHIFT	1
#define MXC_CSPIINT_TFEN_SHIFT		2
#define MXC_CSPIINT_RREN_SHIFT		3
#define MXC_CSPIINT_RHEN_SHIFT       4
#define MXC_CSPIINT_RFEN_SHIFT        5
#define MXC_CSPIINT_ROEN_SHIFT        6

#define MXC_HIGHPOL	0x0
#define MXC_NOPHA	0x0
#define MXC_LOWSSPOL		0x0

#define MXC_CSPISTAT_TE		0
#define MXC_CSPISTAT_TH		1
#define MXC_CSPISTAT_TF		2
#define MXC_CSPISTAT_RR		3
#define MXC_CSPISTAT_RH		4
#define MXC_CSPISTAT_RF		5
#define MXC_CSPISTAT_RO		6

#define MXC_CSPIPERIOD_32KHZ	(1 << 15)

/*!
 * @struct mxc_spi_unique_def
 * @brief This structure contains information that differs with
 * SPI master controller hardware version
 */
struct mxc_spi_unique_def {
	/* Width of valid bits in MXC_CSPIINT */
	unsigned int intr_bit_shift;
	/* Chip Select shift */
	unsigned int cs_shift;
	/* Bit count shift */
	unsigned int bc_shift;
	/* Bit count mask */
	unsigned int bc_mask;
	/* Data Control shift */
	unsigned int drctrl_shift;
	/* Transfer Complete shift */
	unsigned int xfer_complete;
	/* Bit counnter overflow shift */
	unsigned int bc_overflow;
	/* FIFO Size */
	unsigned int fifo_size;
	/* Control reg address */
	unsigned int ctrl_reg_addr;
	/* Status reg address */
	unsigned int stat_reg_addr;
	/* Period reg address */
	unsigned int period_reg_addr;
	/* Test reg address */
	unsigned int test_reg_addr;
	/* Reset reg address */
	unsigned int reset_reg_addr;
	/* SPI mode mask */
	unsigned int mode_mask;
	/* SPI enable */
	unsigned int spi_enable;
	/* XCH bit */
	unsigned int xch;
	/* Spi mode shift */
	unsigned int mode_shift;
	/* Spi master mode enable */
	unsigned int master_enable;
	/* TX interrupt enable diff */
	unsigned int tx_inten_dif;
	/* RX interrupt enable bit diff */
	unsigned int rx_inten_dif;
	/* Interrupt status diff */
	unsigned int int_status_dif;
	/* Low pol shift */
	unsigned int low_pol_shift;
	/* Phase shift */
	unsigned int pha_shift;
	/* SS control shift */
	unsigned int ss_ctrl_shift;
	/* SS pol shift */
	unsigned int ss_pol_shift;
	/* Maximum data rate */
	unsigned int max_data_rate;
	/* Data mask */
	unsigned int data_mask;
	/* Data shift */
	unsigned int data_shift;
	/* Loopback control */
	unsigned int lbc;
	/* RX count off */
	unsigned int rx_cnt_off;
	/* RX count mask */
	unsigned int rx_cnt_mask;
	/* Reset start */
	unsigned int reset_start;
	/* SCLK control inactive state shift */
	unsigned int sclk_ctl_shift;
};

struct mxc_spi;

/*!
 * Structure to group together all the data buffers and functions
 * used in data transfers.
 */
struct mxc_spi_xfer {
	/* Transmit buffer */
	const void *tx_buf;
	/* Receive buffer */
	void *rx_buf;
	/* Data transfered count */
	unsigned int count;
	/* Data received count, descending sequence, zero means no more data to
	   be received */
	unsigned int rx_count;
	/* Function to read the FIFO data to rx_buf */
	void (*rx_get) (struct mxc_spi *, u32 val);
	/* Function to get the data to be written to FIFO */
	 u32(*tx_get) (struct mxc_spi *);
};

/*!
 * This structure is a way for the low level driver to define their own
 * \b spi_master structure. This structure includes the core \b spi_master
 * structure that is provided by Linux SPI Framework/driver as an
 * element and has other elements that are specifically required by this
 * low-level driver.
 */
struct mxc_spi {
	/* SPI Master and a simple I/O queue runner */
	struct spi_bitbang mxc_bitbang;
	/* Completion flags used in data transfers */
	struct completion xfer_done;
	/* Data transfer structure */
	struct mxc_spi_xfer transfer;
	/* Resource structure, which will maintain base addresses and IRQs */
	struct resource *res;
	/* Base address of CSPI, used in readl and writel */
	void *base;
	/* CSPI IRQ number */
	int irq;
	/* CSPI Clock id */
	struct clk *clk;
	/* CSPI input clock SCLK */
	unsigned long spi_ipg_clk;
	/* CSPI registers' bit pattern */
	struct mxc_spi_unique_def *spi_ver_def;
	/* Control reg address */
	void *ctrl_addr;
	/* Status reg address */
	void *stat_addr;
	/* Period reg address */
	void *period_addr;
	/* Test reg address */
	void *test_addr;
	/* Reset reg address */
	void *reset_addr;
	/* Chipselect active function */
	void (*chipselect_active) (int cspi_mode, int status, int chipselect);
	/* Chipselect inactive function */
	void (*chipselect_inactive) (int cspi_mode, int status, int chipselect);
};

#ifdef CONFIG_SPI_MXC_TEST_LOOPBACK
struct spi_chip_info {
	int lb_enable;
};

static struct spi_chip_info lb_chip_info = {
	.lb_enable = 1,
};

static struct spi_board_info loopback_info[] = {
#ifdef CONFIG_SPI_MXC_SELECT1
	{
	 .modalias = "spidev",
	 .controller_data = &lb_chip_info,
	 .irq = 0,
	 .max_speed_hz = 4000000,
	 .bus_num = 1,
	 .chip_select = 4,
	 },
#endif
#ifdef CONFIG_SPI_MXC_SELECT2
	{
	 .modalias = "spidev",
	 .controller_data = &lb_chip_info,
	 .irq = 0,
	 .max_speed_hz = 4000000,
	 .bus_num = 2,
	 .chip_select = 4,
	 },
#endif
#ifdef CONFIG_SPI_MXC_SELECT3
	{
	 .modalias = "spidev",
	 .controller_data = &lb_chip_info,
	 .irq = 0,
	 .max_speed_hz = 4000000,
	 .bus_num = 3,
	 .chip_select = 4,
	 },
#endif
};
#endif

static struct mxc_spi_unique_def spi_ver_2_3 = {
	.intr_bit_shift = 8,
	.cs_shift = 18,
	.bc_shift = 20,
	.bc_mask = 0xFFF,
	.drctrl_shift = 16,
	.xfer_complete = (1 << 7),
	.bc_overflow = 0,
	.fifo_size = 64,
	.ctrl_reg_addr = 4,
	.stat_reg_addr = 0x18,
	.period_reg_addr = 0x1C,
	.test_reg_addr = 0x20,
	.reset_reg_addr = 0x8,
	.mode_mask = 0xF,
	.spi_enable = 0x1,
	.xch = (1 << 2),
	.mode_shift = 4,
	.master_enable = 0,
	.tx_inten_dif = 0,
	.rx_inten_dif = 0,
	.int_status_dif = 0,
	.low_pol_shift = 4,
	.pha_shift = 0,
	.ss_ctrl_shift = 8,
	.ss_pol_shift = 12,
	.max_data_rate = 0xF,
	.data_mask = 0xFF,
	.data_shift = 8,
	.lbc = (1 << 31),
	.rx_cnt_off = 8,
	.rx_cnt_mask = (0x7F << 8),
	.reset_start = 0,
	.sclk_ctl_shift = 20,
};

static struct mxc_spi_unique_def spi_ver_0_7 = {
	.intr_bit_shift = 8,
	.cs_shift = 12,
	.bc_shift = 20,
	.bc_mask = 0xFFF,
	.drctrl_shift = 8,
	.xfer_complete = (1 << 7),
	.bc_overflow = 0,
	.fifo_size = 8,
	.ctrl_reg_addr = 0,
	.stat_reg_addr = 0x14,
	.period_reg_addr = 0x18,
	.test_reg_addr = 0x1C,
	.reset_reg_addr = 0x0,
	.mode_mask = 0x1,
	.spi_enable = 0x1,
	.xch = (1 << 2),
	.mode_shift = 1,
	.master_enable = 1 << 1,
	.tx_inten_dif = 0,
	.rx_inten_dif = 0,
	.int_status_dif = 0,
	.low_pol_shift = 4,
	.pha_shift = 5,
	.ss_ctrl_shift = 6,
	.ss_pol_shift = 7,
	.max_data_rate = 0x7,
	.data_mask = 0x7,
	.data_shift = 16,
	.lbc = (1 << 14),
	.rx_cnt_off = 4,
	.rx_cnt_mask = (0xF << 4),
	.reset_start = 1,
};

static struct mxc_spi_unique_def spi_ver_0_5 = {
	.intr_bit_shift = 9,
	.cs_shift = 12,
	.bc_shift = 20,
	.bc_mask = 0xFFF,
	.drctrl_shift = 8,
	.xfer_complete = (1 << 8),
	.bc_overflow = (1 << 7),
	.fifo_size = 8,
	.ctrl_reg_addr = 0,
	.stat_reg_addr = 0x14,
	.period_reg_addr = 0x18,
	.test_reg_addr = 0x1C,
	.reset_reg_addr = 0x0,
	.mode_mask = 0x1,
	.spi_enable = 0x1,
	.xch = (1 << 2),
	.mode_shift = 1,
	.master_enable = 1 << 1,
	.tx_inten_dif = 0,
	.rx_inten_dif = 0,
	.int_status_dif = 0,
	.low_pol_shift = 4,
	.pha_shift = 5,
	.ss_ctrl_shift = 6,
	.ss_pol_shift = 7,
	.max_data_rate = 0x7,
	.data_mask = 0x7,
	.data_shift = 16,
	.lbc = (1 << 14),
	.rx_cnt_off = 4,
	.rx_cnt_mask = (0xF << 4),
	.reset_start = 1,
};

static struct mxc_spi_unique_def spi_ver_0_4 = {
	.intr_bit_shift = 9,
	.cs_shift = 24,
	.bc_shift = 8,
	.bc_mask = 0x1F,
	.drctrl_shift = 20,
	.xfer_complete = (1 << 8),
	.bc_overflow = (1 << 7),
	.fifo_size = 8,
	.ctrl_reg_addr = 0,
	.stat_reg_addr = 0x14,
	.period_reg_addr = 0x18,
	.test_reg_addr = 0x1C,
	.reset_reg_addr = 0x0,
	.mode_mask = 0x1,
	.spi_enable = 0x1,
	.xch = (1 << 2),
	.mode_shift = 1,
	.master_enable = 1 << 1,
	.tx_inten_dif = 0,
	.rx_inten_dif = 0,
	.int_status_dif = 0,
	.low_pol_shift = 4,
	.pha_shift = 5,
	.ss_ctrl_shift = 6,
	.ss_pol_shift = 7,
	.max_data_rate = 0x7,
	.data_mask = 0x7,
	.data_shift = 16,
	.lbc = (1 << 14),
	.rx_cnt_off = 4,
	.rx_cnt_mask = (0xF << 4),
	.reset_start = 1,
};

static struct mxc_spi_unique_def spi_ver_0_0 = {
	.intr_bit_shift = 18,
	.cs_shift = 19,
	.bc_shift = 0,
	.bc_mask = 0x1F,
	.drctrl_shift = 12,
	.xfer_complete = (1 << 3),
	.bc_overflow = (1 << 8),
	.fifo_size = 8,
	.ctrl_reg_addr = 0,
	.stat_reg_addr = 0x0C,
	.period_reg_addr = 0x14,
	.test_reg_addr = 0x10,
	.reset_reg_addr = 0x1C,
	.mode_mask = 0x1,
	.spi_enable = (1 << 10),
	.xch = (1 << 9),
	.mode_shift = 11,
	.master_enable = 1 << 11,
	.tx_inten_dif = 9,
	.rx_inten_dif = 10,
	.int_status_dif = 1,
	.low_pol_shift = 5,
	.pha_shift = 6,
	.ss_ctrl_shift = 7,
	.ss_pol_shift = 8,
	.max_data_rate = 0x10,
	.data_mask = 0x1F,
	.data_shift = 14,
	.lbc = (1 << 14),
	.rx_cnt_off = 4,
	.rx_cnt_mask = (0xF << 4),
	.reset_start = 1,
};

extern void gpio_spi_active(int cspi_mod);
extern void gpio_spi_inactive(int cspi_mod);

#define MXC_SPI_BUF_RX(type)	\
void mxc_spi_buf_rx_##type(struct mxc_spi *master_drv_data, u32 val)\
{\
	type *rx = master_drv_data->transfer.rx_buf;\
	*rx++ = (type)val;\
	master_drv_data->transfer.rx_buf = rx;\
}

#define MXC_SPI_BUF_TX(type)    \
u32 mxc_spi_buf_tx_##type(struct mxc_spi *master_drv_data)\
{\
	u32 val;\
	const type *tx = master_drv_data->transfer.tx_buf;\
	val = *tx++;\
	master_drv_data->transfer.tx_buf = tx;\
	return val;\
}

MXC_SPI_BUF_RX(u8)
    MXC_SPI_BUF_TX(u8)
    MXC_SPI_BUF_RX(u16)
    MXC_SPI_BUF_TX(u16)
    MXC_SPI_BUF_RX(u32)
    MXC_SPI_BUF_TX(u32)

/*!
 * This function enables CSPI interrupt(s)
 *
 * @param        master_data the pointer to mxc_spi structure
 * @param        irqs        the irq(s) to set (can be a combination)
 *
 * @return       This function returns 0 if successful, -1 otherwise.
 */
static int spi_enable_interrupt(struct mxc_spi *master_data, unsigned int irqs)
{
	if (irqs & ~((1 << master_data->spi_ver_def->intr_bit_shift) - 1)) {
		return -1;
	}

	__raw_writel((irqs | __raw_readl(MXC_CSPIINT + master_data->ctrl_addr)),
		     MXC_CSPIINT + master_data->ctrl_addr);
	return 0;
}

/*!
 * This function disables CSPI interrupt(s)
 *
 * @param        master_data the pointer to mxc_spi structure
 * @param        irqs        the irq(s) to reset (can be a combination)
 *
 * @return       This function returns 0 if successful, -1 otherwise.
 */
static int spi_disable_interrupt(struct mxc_spi *master_data, unsigned int irqs)
{
	if (irqs & ~((1 << master_data->spi_ver_def->intr_bit_shift) - 1)) {
		return -1;
	}

	__raw_writel((~irqs &
		      __raw_readl(MXC_CSPIINT + master_data->ctrl_addr)),
		     MXC_CSPIINT + master_data->ctrl_addr);
	return 0;
}

/*!
 * This function sets the baud rate for the SPI module.
 *
 * @param        master_data the pointer to mxc_spi structure
 * @param        baud        the baud rate
 *
 * @return       This function returns the baud rate divisor.
 */
static unsigned int spi_find_baudrate(struct mxc_spi *master_data,
				      unsigned int baud)
{
	unsigned int divisor;
	unsigned int shift = 0;

	/* Calculate required divisor (rounded) */
	divisor = (master_data->spi_ipg_clk + baud / 2) / baud;
	while (divisor >>= 1)
		shift++;

	if (master_data->spi_ver_def == &spi_ver_0_0) {
		shift = (shift - 1) * 2;
	} else if (master_data->spi_ver_def == &spi_ver_2_3) {
		shift = shift;
	} else {
		shift -= 2;
	}

	if (shift > master_data->spi_ver_def->max_data_rate)
		shift = master_data->spi_ver_def->max_data_rate;

	return shift << master_data->spi_ver_def->data_shift;
}

/*!
 * This function loads the transmit fifo.
 *
 * @param  base             the CSPI base address
 * @param  count            number of words to put in the TxFIFO
 * @param  master_drv_data  spi master structure
 */
static void spi_put_tx_data(void *base, unsigned int count,
			    struct mxc_spi *master_drv_data)
{
	unsigned int ctrl_reg;
	unsigned int data;
	int i = 0;

	/* Perform Tx transaction */
	for (i = 0; i < count; i++) {
		data = master_drv_data->transfer.tx_get(master_drv_data);
		__raw_writel(data, base + MXC_CSPITXDATA);
	}

	ctrl_reg = __raw_readl(base + MXC_CSPICTRL);

	ctrl_reg |= master_drv_data->spi_ver_def->xch;

	__raw_writel(ctrl_reg, base + MXC_CSPICTRL);

	return;
}

/*!
 * This function configures the hardware CSPI for the current SPI device.
 * It sets the word size, transfer mode, data rate for this device.
 *
 * @param       spi     	the current SPI device
 * @param	is_active 	indicates whether to active/deactivate the current device
 */
void mxc_spi_chipselect(struct spi_device *spi, int is_active)
{
	struct mxc_spi *master_drv_data;
	struct mxc_spi_xfer *ptransfer;
	struct mxc_spi_unique_def *spi_ver_def;
	unsigned int ctrl_reg = 0;
	unsigned int config_reg = 0;
	unsigned int xfer_len;
	unsigned int cs_value;

	if (is_active == BITBANG_CS_INACTIVE) {
		/*Need to deselect the slave */
		return;
	}

	/* Get the master controller driver data from spi device's master */

	master_drv_data = spi_master_get_devdata(spi->master);
	clk_enable(master_drv_data->clk);
	spi_ver_def = master_drv_data->spi_ver_def;

	xfer_len = spi->bits_per_word;

	if (spi_ver_def == &spi_ver_2_3) {
		/* Control Register Settings for transfer to this slave */
		ctrl_reg = master_drv_data->spi_ver_def->spi_enable;
		ctrl_reg |=
		    ((spi->chip_select & MXC_CSPICTRL_CSMASK) << spi_ver_def->
		     cs_shift);
		ctrl_reg |=
		    (((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
		      spi_ver_def->mode_mask) << spi_ver_def->mode_shift);
		ctrl_reg |=
		    spi_find_baudrate(master_drv_data, spi->max_speed_hz);
		ctrl_reg |=
		    (((xfer_len -
		       1) & spi_ver_def->bc_mask) << spi_ver_def->bc_shift);

		if (spi->mode & SPI_CPHA)
			config_reg |=
			    (((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
			      spi_ver_def->mode_mask) <<
			     spi_ver_def->pha_shift);

		if ((spi->mode & SPI_CPOL)) {
			config_reg |=
			    (((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
			      spi_ver_def->mode_mask) <<
			     spi_ver_def->low_pol_shift);
			config_reg |=
			    (((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
			      spi_ver_def->mode_mask) <<
			     spi_ver_def->sclk_ctl_shift);
		}
		cs_value = (__raw_readl(MXC_CSPICONFIG +
					master_drv_data->ctrl_addr) >>
			    spi_ver_def->ss_pol_shift) & spi_ver_def->mode_mask;
		if (spi->mode & SPI_CS_HIGH) {
			config_reg |=
			    ((((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
			       spi_ver_def->mode_mask) | cs_value) <<
			     spi_ver_def->ss_pol_shift);
		} else
			config_reg |=
			    ((~((1 << (spi->chip_select &
				       MXC_CSPICTRL_CSMASK)) &
				spi_ver_def->mode_mask) & cs_value) <<
			     spi_ver_def->ss_pol_shift);
		config_reg |=
		    (((1 << (spi->chip_select & MXC_CSPICTRL_CSMASK)) &
		      spi_ver_def->mode_mask) << spi_ver_def->ss_ctrl_shift);
		__raw_writel(0, master_drv_data->base + MXC_CSPICTRL);
		__raw_writel(ctrl_reg, master_drv_data->base + MXC_CSPICTRL);
		__raw_writel(config_reg,
			     MXC_CSPICONFIG + master_drv_data->ctrl_addr);
	} else {
		/* Control Register Settings for transfer to this slave */
		ctrl_reg = master_drv_data->spi_ver_def->spi_enable;
		ctrl_reg |=
		    (((spi->chip_select & MXC_CSPICTRL_CSMASK) << spi_ver_def->
		      cs_shift) | spi_ver_def->mode_mask <<
		     spi_ver_def->mode_shift);
		ctrl_reg |=
		    spi_find_baudrate(master_drv_data, spi->max_speed_hz);
		ctrl_reg |=
		    (((xfer_len -
		       1) & spi_ver_def->bc_mask) << spi_ver_def->bc_shift);
		if (spi->mode & SPI_CPHA)
			ctrl_reg |=
			    spi_ver_def->mode_mask << spi_ver_def->pha_shift;
		if (spi->mode & SPI_CPOL)
			ctrl_reg |=
			    spi_ver_def->mode_mask << spi_ver_def->
			    low_pol_shift;
		if (spi->mode & SPI_CS_HIGH)
			ctrl_reg |=
			    spi_ver_def->mode_mask << spi_ver_def->ss_pol_shift;
		if (spi_ver_def == &spi_ver_0_7)
			ctrl_reg |=
			    spi_ver_def->mode_mask << spi_ver_def->
			    ss_ctrl_shift;

		__raw_writel(ctrl_reg, master_drv_data->base + MXC_CSPICTRL);
	}

	/* Initialize the functions for transfer */
	ptransfer = &master_drv_data->transfer;
	if (xfer_len <= 8) {
		ptransfer->rx_get = mxc_spi_buf_rx_u8;
		ptransfer->tx_get = mxc_spi_buf_tx_u8;
	} else if (xfer_len <= 16) {
		ptransfer->rx_get = mxc_spi_buf_rx_u16;
		ptransfer->tx_get = mxc_spi_buf_tx_u16;
	} else {
		ptransfer->rx_get = mxc_spi_buf_rx_u32;
		ptransfer->tx_get = mxc_spi_buf_tx_u32;
	}
#ifdef CONFIG_SPI_MXC_TEST_LOOPBACK
	{
		struct spi_chip_info *lb_chip =
		    (struct spi_chip_info *)spi->controller_data;
		if (!lb_chip)
			__raw_writel(0, master_drv_data->test_addr);
		else if (lb_chip->lb_enable)
			__raw_writel(spi_ver_def->lbc,
				     master_drv_data->test_addr);
	}
#endif
	clk_disable(master_drv_data->clk);
	return;
}

/*!
 * This function is called when an interrupt occurs on the SPI modules.
 * It is the interrupt handler for the SPI modules.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 *
 * @return       The function returns IRQ_HANDLED when handled.
 */
static irqreturn_t mxc_spi_isr(int irq, void *dev_id)
{
	struct mxc_spi *master_drv_data = dev_id;
	irqreturn_t ret = IRQ_NONE;
	unsigned int status;
	int fifo_size;
	unsigned int pass_counter;

	fifo_size = master_drv_data->spi_ver_def->fifo_size;
	pass_counter = fifo_size;

	/* Read the interrupt status register to determine the source */
	status = __raw_readl(master_drv_data->stat_addr);
	do {
		u32 rx_tmp =
		    __raw_readl(master_drv_data->base + MXC_CSPIRXDATA);

		if (master_drv_data->transfer.rx_buf)
			master_drv_data->transfer.rx_get(master_drv_data,
							 rx_tmp);
		(master_drv_data->transfer.count)--;
		(master_drv_data->transfer.rx_count)--;
		ret = IRQ_HANDLED;
		if (pass_counter-- == 0) {
			break;
		}
		status = __raw_readl(master_drv_data->stat_addr);
	} while (status &
		 (1 <<
		  (MXC_CSPISTAT_RR +
		   master_drv_data->spi_ver_def->int_status_dif)));

	if (master_drv_data->transfer.rx_count)
		return ret;

	if (master_drv_data->transfer.count) {
		if (master_drv_data->transfer.tx_buf) {
			u32 count = (master_drv_data->transfer.count >
				     fifo_size) ? fifo_size :
			    master_drv_data->transfer.count;
			master_drv_data->transfer.rx_count = count;
			spi_put_tx_data(master_drv_data->base, count,
					master_drv_data);
		}
	} else {
		complete(&master_drv_data->xfer_done);
	}

	return ret;
}

/*!
 * This function initialize the current SPI device.
 *
 * @param        spi     the current SPI device.
 *
 */
int mxc_spi_setup(struct spi_device *spi)
{
	if (spi->max_speed_hz < 0) {
		return -EINVAL;
	}

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	pr_debug("%s: mode %d, %u bpw, %d hz\n", __FUNCTION__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	return 0;
}

static int mxc_spi_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	return 0;
}

/*!
 * This function is called when the data has to transfer from/to the
 * current SPI device in poll mode
 *
 * @param        spi        the current spi device
 * @param        t          the transfer request - read/write buffer pairs
 *
 * @return       Returns 0 on success.
 */
int mxc_spi_poll_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct mxc_spi *master_drv_data = NULL;
	int count, i;
	volatile unsigned int status;
	u32 rx_tmp;
	u32 fifo_size;
	int chipselect_status;

	mxc_spi_chipselect(spi, BITBANG_CS_ACTIVE);

	/* Get the master controller driver data from spi device's master */
	master_drv_data = spi_master_get_devdata(spi->master);

	chipselect_status = __raw_readl(MXC_CSPICONFIG +
					master_drv_data->ctrl_addr);
	chipselect_status >>= master_drv_data->spi_ver_def->ss_pol_shift &
	    master_drv_data->spi_ver_def->mode_mask;
	if (master_drv_data->chipselect_active)
		master_drv_data->chipselect_active(spi->master->bus_num,
						   chipselect_status,
						   (spi->chip_select &
						    MXC_CSPICTRL_CSMASK) + 1);

	clk_enable(master_drv_data->clk);

	/* Modify the Tx, Rx, Count */
	master_drv_data->transfer.tx_buf = t->tx_buf;
	master_drv_data->transfer.rx_buf = t->rx_buf;
	master_drv_data->transfer.count = t->len;
	fifo_size = master_drv_data->spi_ver_def->fifo_size;

	count = (t->len > fifo_size) ? fifo_size : t->len;
	spi_put_tx_data(master_drv_data->base, count, master_drv_data);

	while ((((status = __raw_readl(master_drv_data->test_addr)) &
		 master_drv_data->spi_ver_def->rx_cnt_mask) >> master_drv_data->
		spi_ver_def->rx_cnt_off) != count)
		;

	for (i = 0; i < count; i++) {
		rx_tmp = __raw_readl(master_drv_data->base + MXC_CSPIRXDATA);
		master_drv_data->transfer.rx_get(master_drv_data, rx_tmp);
	}

	clk_disable(master_drv_data->clk);
	if (master_drv_data->chipselect_inactive)
		master_drv_data->chipselect_inactive(spi->master->bus_num,
						     chipselect_status,
						     (spi->chip_select &
						      MXC_CSPICTRL_CSMASK) + 1);
	return 0;
}

/*!
 * This function is called when the data has to transfer from/to the
 * current SPI device. It enables the Rx interrupt, initiates the transfer.
 * When Rx interrupt occurs, the completion flag is set. It then disables
 * the Rx interrupt.
 *
 * @param        spi        the current spi device
 * @param        t          the transfer request - read/write buffer pairs
 *
 * @return       Returns 0 on success -1 on failure.
 */
int mxc_spi_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct mxc_spi *master_drv_data = NULL;
	int count;
	int chipselect_status;
	u32 fifo_size;

	/* Get the master controller driver data from spi device's master */

	master_drv_data = spi_master_get_devdata(spi->master);

	chipselect_status = __raw_readl(MXC_CSPICONFIG +
					master_drv_data->ctrl_addr);
	chipselect_status >>= master_drv_data->spi_ver_def->ss_pol_shift &
	    master_drv_data->spi_ver_def->mode_mask;
	if (master_drv_data->chipselect_active)
		master_drv_data->chipselect_active(spi->master->bus_num,
						   chipselect_status,
						   (spi->chip_select &
						    MXC_CSPICTRL_CSMASK) + 1);

	clk_enable(master_drv_data->clk);
	/* Modify the Tx, Rx, Count */
	master_drv_data->transfer.tx_buf = t->tx_buf;
	master_drv_data->transfer.rx_buf = t->rx_buf;
	master_drv_data->transfer.count = t->len;
	fifo_size = master_drv_data->spi_ver_def->fifo_size;
	INIT_COMPLETION(master_drv_data->xfer_done);

	/* Enable the Rx Interrupts */

	spi_enable_interrupt(master_drv_data,
			     1 << (MXC_CSPIINT_RREN_SHIFT +
				   master_drv_data->spi_ver_def->rx_inten_dif));
	count = (t->len > fifo_size) ? fifo_size : t->len;

	/* Perform Tx transaction */
	master_drv_data->transfer.rx_count = count;
	spi_put_tx_data(master_drv_data->base, count, master_drv_data);

	/* Wait for transfer completion */
	wait_for_completion(&master_drv_data->xfer_done);

	/* Disable the Rx Interrupts */

	spi_disable_interrupt(master_drv_data,
			      1 << (MXC_CSPIINT_RREN_SHIFT +
				    master_drv_data->spi_ver_def->
				    rx_inten_dif));

	clk_disable(master_drv_data->clk);
	if (master_drv_data->chipselect_inactive)
		master_drv_data->chipselect_inactive(spi->master->bus_num,
						     chipselect_status,
						     (spi->chip_select &
						      MXC_CSPICTRL_CSMASK) + 1);
	return t->len - master_drv_data->transfer.count;
}

/*!
 * This function releases the current SPI device's resources.
 *
 * @param        spi     the current SPI device.
 *
 */
void mxc_spi_cleanup(struct spi_device *spi)
{
}

/*!
 * This function is called during the driver binding process. Based on the CSPI
 * hardware module that is being probed this function adds the appropriate SPI module
 * structure in the SPI core driver.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions.
 *
 * @return  The function returns 0 on successful registration and initialization
 *          of CSPI module. Otherwise returns specific error code.
 */
static int mxc_spi_probe(struct platform_device *pdev)
{
	struct mxc_spi_master *mxc_platform_info;
	struct spi_master *master;
	struct mxc_spi *master_drv_data = NULL;
	unsigned int spi_ver;
	int ret = -ENODEV;

	/* Get the platform specific data for this master device */

	mxc_platform_info = (struct mxc_spi_master *)pdev->dev.platform_data;
	if (!mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data for CSPI\n");
		return -EINVAL;
	}

	/* Allocate SPI master controller */

	master = spi_alloc_master(&pdev->dev, sizeof(struct mxc_spi));
	if (!master) {
		dev_err(&pdev->dev, "can't alloc for spi_master\n");
		return -ENOMEM;
	}

	/* Set this device's driver data to master */

	platform_set_drvdata(pdev, master);

	/* Set this master's data from platform_info */

	master->bus_num = pdev->id + 1;
	master->num_chipselect = mxc_platform_info->maxchipselect;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
#ifdef CONFIG_SPI_MXC_TEST_LOOPBACK
	master->num_chipselect += 1;
#endif
	/* Set the master controller driver data for this master */

	master_drv_data = spi_master_get_devdata(master);
	master_drv_data->mxc_bitbang.master = spi_master_get(master);
	if (mxc_platform_info->chipselect_active)
		master_drv_data->chipselect_active =
		    mxc_platform_info->chipselect_active;
	if (mxc_platform_info->chipselect_inactive)
		master_drv_data->chipselect_inactive =
		    mxc_platform_info->chipselect_inactive;

	/* Identify SPI version */

	spi_ver = mxc_platform_info->spi_version;
	if (spi_ver == 7) {
		master_drv_data->spi_ver_def = &spi_ver_0_7;
	} else if (spi_ver == 5) {
		master_drv_data->spi_ver_def = &spi_ver_0_5;
	} else if (spi_ver == 4) {
		master_drv_data->spi_ver_def = &spi_ver_0_4;
	} else if (spi_ver == 0) {
		master_drv_data->spi_ver_def = &spi_ver_0_0;
	} else if (spi_ver == 23) {
		master_drv_data->spi_ver_def = &spi_ver_2_3;
	}

	dev_dbg(&pdev->dev, "SPI_REV 0.%d\n", spi_ver);

	/* Set the master bitbang data */

	master_drv_data->mxc_bitbang.chipselect = mxc_spi_chipselect;
	master_drv_data->mxc_bitbang.txrx_bufs = mxc_spi_transfer;
	master_drv_data->mxc_bitbang.master->setup = mxc_spi_setup;
	master_drv_data->mxc_bitbang.master->cleanup = mxc_spi_cleanup;
	master_drv_data->mxc_bitbang.setup_transfer = mxc_spi_setup_transfer;

	/* Initialize the completion object */

	init_completion(&master_drv_data->xfer_done);

	/* Set the master controller register addresses and irqs */

	master_drv_data->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!master_drv_data->res) {
		dev_err(&pdev->dev, "can't get platform resource for CSPI%d\n",
			master->bus_num);
		ret = -ENOMEM;
		goto err;
	}

	if (!request_mem_region(master_drv_data->res->start,
				master_drv_data->res->end -
				master_drv_data->res->start + 1, pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed for CSPI%d\n",
			master->bus_num);
		ret = -ENOMEM;
		goto err;
	}

	master_drv_data->base = ioremap(master_drv_data->res->start,
		master_drv_data->res->end - master_drv_data->res->start + 1);
	if (!master_drv_data->base) {
		dev_err(&pdev->dev, "invalid base address for CSPI%d\n",
			master->bus_num);
		ret = -EINVAL;
		goto err1;
	}

	master_drv_data->irq = platform_get_irq(pdev, 0);
	if (master_drv_data->irq < 0) {
		dev_err(&pdev->dev, "can't get IRQ for CSPI%d\n",
			master->bus_num);
		ret = -EINVAL;
		goto err1;
	}

	/* Register for SPI Interrupt */

	ret = request_irq(master_drv_data->irq, mxc_spi_isr,
			  0, "CSPI_IRQ", master_drv_data);
	if (ret != 0) {
		dev_err(&pdev->dev, "request_irq failed for CSPI%d\n",
			master->bus_num);
		goto err1;
	}

	/* Setup any GPIO active */

	gpio_spi_active(master->bus_num - 1);

	/* Enable the CSPI Clock, CSPI Module, set as a master */

	master_drv_data->ctrl_addr =
	    master_drv_data->base + master_drv_data->spi_ver_def->ctrl_reg_addr;
	master_drv_data->stat_addr =
	    master_drv_data->base + master_drv_data->spi_ver_def->stat_reg_addr;
	master_drv_data->period_addr =
	    master_drv_data->base +
	    master_drv_data->spi_ver_def->period_reg_addr;
	master_drv_data->test_addr =
	    master_drv_data->base + master_drv_data->spi_ver_def->test_reg_addr;
	master_drv_data->reset_addr =
	    master_drv_data->base +
	    master_drv_data->spi_ver_def->reset_reg_addr;

	master_drv_data->clk = clk_get(&pdev->dev, "cspi_clk");
	clk_enable(master_drv_data->clk);
	master_drv_data->spi_ipg_clk = clk_get_rate(master_drv_data->clk);

	__raw_writel(master_drv_data->spi_ver_def->reset_start,
		     master_drv_data->reset_addr);
	udelay(1);
	__raw_writel((master_drv_data->spi_ver_def->spi_enable +
		      master_drv_data->spi_ver_def->master_enable),
		     master_drv_data->base + MXC_CSPICTRL);
	__raw_writel(MXC_CSPIPERIOD_32KHZ, master_drv_data->period_addr);
	__raw_writel(0, MXC_CSPIINT + master_drv_data->ctrl_addr);

	/* Start the SPI Master Controller driver */

	ret = spi_bitbang_start(&master_drv_data->mxc_bitbang);

	if (ret != 0)
		goto err2;

	printk(KERN_INFO "CSPI: %s-%d probed\n", pdev->name, pdev->id);

#ifdef CONFIG_SPI_MXC_TEST_LOOPBACK
	{
		int i;
		struct spi_board_info *bi = &loopback_info[0];
		for (i = 0; i < ARRAY_SIZE(loopback_info); i++, bi++) {
			if (bi->bus_num != master->bus_num)
				continue;

			dev_info(&pdev->dev,
				 "registering loopback device '%s'\n",
				 bi->modalias);

			spi_new_device(master, bi);
		}
	}
#endif
	clk_disable(master_drv_data->clk);
	return ret;

      err2:
	gpio_spi_inactive(master->bus_num - 1);
	clk_disable(master_drv_data->clk);
	clk_put(master_drv_data->clk);
	free_irq(master_drv_data->irq, master_drv_data);
      err1:
	iounmap(master_drv_data->base);
	release_mem_region(pdev->resource[0].start,
			   pdev->resource[0].end - pdev->resource[0].start + 1);
      err:
	spi_master_put(master);
	kfree(master);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

/*!
 * Dissociates the driver from the SPI master controller. Disables the CSPI module.
 * It handles the release of SPI resources like IRQ, memory,..etc.
 *
 * @param   pdev  the device structure used to give information on which SPI
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxc_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	if (master) {
		struct mxc_spi *master_drv_data =
		    spi_master_get_devdata(master);

		gpio_spi_inactive(master->bus_num - 1);

		/* Disable the CSPI module */
		clk_enable(master_drv_data->clk);
		__raw_writel(MXC_CSPICTRL_DISABLE,
			     master_drv_data->base + MXC_CSPICTRL);
		clk_disable(master_drv_data->clk);
		/* Unregister for SPI Interrupt */

		free_irq(master_drv_data->irq, master_drv_data);

		iounmap(master_drv_data->base);
		release_mem_region(master_drv_data->res->start,
				   master_drv_data->res->end -
				   master_drv_data->res->start + 1);

		/* Stop the SPI Master Controller driver */

		spi_bitbang_stop(&master_drv_data->mxc_bitbang);

		spi_master_put(master);
	}

	printk(KERN_INFO "CSPI: %s-%d removed\n", pdev->name, pdev->id);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int spi_bitbang_suspend(struct spi_bitbang *bitbang)
{
	unsigned long flags;
	unsigned limit = 500;

	spin_lock_irqsave(&bitbang->lock, flags);
	while (!list_empty(&bitbang->queue) && limit--) {
		spin_unlock_irqrestore(&bitbang->lock, flags);

		dev_dbg(&bitbang->master->dev, "wait for queue\n");
		msleep(10);

		spin_lock_irqsave(&bitbang->lock, flags);
	}
	if (!list_empty(&bitbang->queue)) {
		dev_err(&bitbang->master->dev, "queue didn't empty\n");
		spin_unlock_irqrestore(&bitbang->lock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&bitbang->lock, flags);

	return 0;
}

static void spi_bitbang_resume(struct spi_bitbang *bitbang)
{
	spin_lock_init(&bitbang->lock);
	INIT_LIST_HEAD(&bitbang->queue);

	bitbang->busy = 0;
}

/*!
 * This function puts the SPI master controller in low-power mode/state.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct mxc_spi *master_drv_data = spi_master_get_devdata(master);
	int ret = 0;

	spi_bitbang_suspend(&master_drv_data->mxc_bitbang);
	clk_enable(master_drv_data->clk);
	__raw_writel(MXC_CSPICTRL_DISABLE,
		     master_drv_data->base + MXC_CSPICTRL);
	clk_disable(master_drv_data->clk);
	gpio_spi_inactive(master->bus_num - 1);

	return ret;
}

/*!
 * This function brings the SPI master controller back from low-power state.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_spi_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct mxc_spi *master_drv_data = spi_master_get_devdata(master);

	gpio_spi_active(master->bus_num - 1);

	spi_bitbang_resume(&master_drv_data->mxc_bitbang);
	clk_enable(master_drv_data->clk);
	__raw_writel((master_drv_data->spi_ver_def->spi_enable +
				master_drv_data->spi_ver_def->master_enable),
				master_drv_data->base + MXC_CSPICTRL);
	clk_disable(master_drv_data->clk);

	return 0;
}
#else
#define mxc_spi_suspend  NULL
#define mxc_spi_resume   NULL
#endif				/* CONFIG_PM */

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_spi_driver = {
	.driver = {
		   .name = "mxc_spi",
		   .owner = THIS_MODULE,
		   },
	.probe = mxc_spi_probe,
	.remove = mxc_spi_remove,
	.suspend = mxc_spi_suspend,
	.resume = mxc_spi_resume,
};

/*!
 * This function implements the init function of the SPI device.
 * It is called when the module is loaded. It enables the required
 * clocks to CSPI module(if any) and activates necessary GPIO pins.
 *
 * @return       This function returns 0.
 */
static int __init mxc_spi_init(void)
{
	pr_debug("Registering the SPI Controller Driver\n");
	return platform_driver_register(&mxc_spi_driver);
}

/*!
 * This function implements the exit function of the SPI device.
 * It is called when the module is unloaded. It deactivates the
 * the GPIO pin associated with CSPI hardware modules.
 *
 */
static void __exit mxc_spi_exit(void)
{
	pr_debug("Unregistering the SPI Controller Driver\n");
	platform_driver_unregister(&mxc_spi_driver);
}

subsys_initcall(mxc_spi_init);
module_exit(mxc_spi_exit);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
