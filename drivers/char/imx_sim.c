/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_sim.c
 *
 * @brief Driver for Freescale IMX SIM interface
 *
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/sched.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mxc_sim_interface.h>
#include <linux/fsl_devices.h>

#include <asm/io.h>

#define SIM_INTERNAL_CLK  0
#define SIM_RFU          -1

/* Default communication parameters: FI=372, DI=1, PI1=5V, II=50mA, WWT=10 */
#define SIM_PARAM_DEFAULT { 0, 1, 1, 5, 1, 0, 0, 0, 10 }

/* Transmit and receive buffer sizes */
#define SIM_XMT_BUFFER_SIZE 256
#define SIM_RCV_BUFFER_SIZE 256

/* Interface character references */
#define SIM_IFC_TXI(letter, number) (letter + number * 4)
#define SIM_IFC_TA1   SIM_IFC_TXI(0, 0)
#define SIM_IFC_TB1   SIM_IFC_TXI(0, 1)
#define SIM_IFC_TC1   SIM_IFC_TXI(0, 2)
#define SIM_IFC_TD1   SIM_IFC_TXI(0, 3)
#define SIM_IFC_TA2   SIM_IFC_TXI(1, 0)
#define SIM_IFC_TB2   SIM_IFC_TXI(1, 1)
#define SIM_IFC_TC2   SIM_IFC_TXI(1, 2)
#define SIM_IFC_TD2   SIM_IFC_TXI(1, 3)
#define SIM_IFC_TA3   SIM_IFC_TXI(2, 0)
#define SIM_IFC_TB3   SIM_IFC_TXI(2, 1)
#define SIM_IFC_TC3   SIM_IFC_TXI(2, 2)
#define SIM_IFC_TD3   SIM_IFC_TXI(2, 3)
#define SIM_IFC_TA4   SIM_IFC_TXI(3, 0)
#define SIM_IFC_TB4   SIM_IFC_TXI(3, 1)
#define SIM_IFC_TC4   SIM_IFC_TXI(3, 2)
#define SIM_IFC_TD4   SIM_IFC_TXI(3, 3)

/* ATR and OPS states */
#define SIM_STATE_REMOVED              0
#define SIM_STATE_OPERATIONAL_IDLE     1
#define SIM_STATE_OPERATIONAL_COMMAND  2
#define SIM_STATE_OPERATIONAL_RESPONSE 3
#define SIM_STATE_OPERATIONAL_STATUS1  4
#define SIM_STATE_OPERATIONAL_STATUS2  5
#define SIM_STATE_OPERATIONAL_PTS      6
#define SIM_STATE_DETECTED_ATR_T0       7
#define SIM_STATE_DETECTED_ATR_TS       8
#define SIM_STATE_DETECTED_ATR_TXI      9
#define SIM_STATE_DETECTED_ATR_THB      10
#define SIM_STATE_DETECTED_ATR_TCK      11

/* Definitions of the offset of the SIM hardware registers */
#define PORT1_CNTL 	0x00	/* 00 */
#define SETUP 		0x04	/* 04 */
#define PORT1_DETECT 	0x08	/* 08 */
#define PORT1_XMT_BUF 	0x0C	/* 0c */
#define PORT1_RCV_BUF 	0x10	/* 10 */
#define PORT0_CNTL 	0x14	/* 14 */
#define CNTL 		0x18	/* 18 */
#define CLK_PRESCALER 	0x1C	/* 1c */
#define RCV_THRESHOLD 	0x20	/* 20 */
#define ENABLE 		0x24	/* 24 */
#define XMT_STATUS 	0x28	/* 28 */
#define RCV_STATUS 	0x2C	/* 2c */
#define INT_MASK 	0x30	/* 30 */
#define PORTO_XMT_BUF 	0x34	/* 34 */
#define PORT0_RCV_BUF 	0x38	/* 38 */
#define PORT0_DETECT 	0x3C	/* 3c */
#define DATA_FORMAT 	0x40	/* 40 */
#define XMT_THRESHOLD 	0x44	/* 44 */
#define GUARD_CNTL 	0x48	/* 48 */
#define OD_CONFIG 	0x4C	/* 4c */
#define RESET_CNTL 	0x50	/* 50 */
#define CHAR_WAIT 	0x54	/* 54 */
#define GPCNT 		0x58	/* 58 */
#define DIVISOR 	0x5C	/* 5c */
#define BWT 		0x60	/* 60 */
#define BGT 		0x64	/* 64 */
#define BWT_H 		0x68	/* 68 */
#define XMT_FIFO_STAT 	0x6C	/* 6c */
#define RCV_FIFO_CNT 	0x70	/* 70 */
#define RCV_FIFO_WPTR 	0x74	/* 74 */
#define RCV_FIFO_RPTR 	0x78	/* 78 */

/* SIM port[0|1]_cntl register bits */
#define SIM_PORT_CNTL_SFPD   (1<<7)
#define SIM_PORT_CNTL_3VOLT  (1<<6)
#define SIM_PORT_CNTL_SCSP   (1<<5)
#define SIM_PORT_CNTL_SCEN   (1<<4)
#define SIM_PORT_CNTL_SRST   (1<<3)
#define SIM_PORT_CNTL_STEN   (1<<2)
#define SIM_PORT_CNTL_SVEN   (1<<1)
#define SIM_PORT_CNTL_SAPD   (1<<0)

/* SIM od_config register bits */
#define SIM_OD_CONFIG_OD_P1  (1<<1)
#define SIM_OD_CONFIG_OD_P0  (1<<0)

/* SIM enable register bits */
#define SIM_ENABLE_XMTEN     (1<<1)
#define SIM_ENABLE_RCVEN     (1<<0)

/* SIM int_mask register bits */
#define SIM_INT_MASK_RFEM    (1<<13)
#define SIM_INT_MASK_BGTM    (1<<12)
#define SIM_INT_MASK_BWTM    (1<<11)
#define SIM_INT_MASK_RTM     (1<<10)
#define SIM_INT_MASK_CWTM    (1<<9)
#define SIM_INT_MASK_GPCM    (1<<8)
#define SIM_INT_MASK_TDTFM   (1<<7)
#define SIM_INT_MASK_TFOM    (1<<6)
#define SIM_INT_MASK_XTM     (1<<5)
#define SIM_INT_MASK_TFEIM   (1<<4)
#define SIM_INT_MASK_ETCIM   (1<<3)
#define SIM_INT_MASK_OIM     (1<<2)
#define SIM_INT_MASK_TCIM    (1<<1)
#define SIM_INT_MASK_RIM     (1<<0)

/* SIM xmt_status register bits */
#define SIM_XMT_STATUS_GPCNT (1<<8)
#define SIM_XMT_STATUS_TDTF  (1<<7)
#define SIM_XMT_STATUS_TFO   (1<<6)
#define SIM_XMT_STATUS_TC    (1<<5)
#define SIM_XMT_STATUS_ETC   (1<<4)
#define SIM_XMT_STATUS_TFE   (1<<3)
#define SIM_XMT_STATUS_XTE   (1<<0)

/* SIM rcv_status register bits */
#define SIM_RCV_STATUS_BGT   (1<<11)
#define SIM_RCV_STATUS_BWT   (1<<10)
#define SIM_RCV_STATUS_RTE   (1<<9)
#define SIM_RCV_STATUS_CWT   (1<<8)
#define SIM_RCV_STATUS_CRCOK (1<<7)
#define SIM_RCV_STATUS_LRCOK (1<<6)
#define SIM_RCV_STATUS_RDRF  (1<<5)
#define SIM_RCV_STATUS_RFD   (1<<4)
#define SIM_RCV_STATUS_RFE   (1<<1)
#define SIM_RCV_STATUS_OEF   (1<<0)

/* SIM cntl register bits */
#define SIM_CNTL_BWTEN       (1<<15)
#define SIM_CNTL_XMT_CRC_LRC (1<<14)
#define SIM_CNTL_CRCEN       (1<<13)
#define SIM_CNTL_LRCEN       (1<<12)
#define SIM_CNTL_CWTEN       (1<<11)
#define SIM_CNTL_SAMPLE12    (1<<4)
#define SIM_CNTL_ONACK       (1<<3)
#define SIM_CNTL_ANACK       (1<<2)
#define SIM_CNTL_ICM         (1<<1)
#define SIM_CNTL_GPCNT_CLK_SEL(x)   ((x&0x03)<<9)
#define SIM_CNTL_GPCNT_CLK_SEL_MASK     (0x03<<9)
#define SIM_CNTL_BAUD_SEL(x)        ((x&0x07)<<6)
#define SIM_CNTL_BAUD_SEL_MASK          (0x07<<6)

/* SIM rcv_threshold register bits */
#define SIM_RCV_THRESHOLD_RTH(x)    ((x&0x0f)<<9)
#define SIM_RCV_THRESHOLD_RTH_MASK      (0x0f<<9)
#define SIM_RCV_THRESHOLD_RDT(x)   ((x&0x1ff)<<0)
#define SIM_RCV_THRESHOLD_RDT_MASK     (0x1ff<<0)

/* SIM xmt_threshold register bits */
#define SIM_XMT_THRESHOLD_XTH(x)    ((x&0x0f)<<4)
#define SIM_XMT_THRESHOLD_XTH_MASK      (0x0f<<4)
#define SIM_XMT_THRESHOLD_TDT(x)    ((x&0x0f)<<0)
#define SIM_XMT_THRESHOLD_TDT_MASK      (0x0f<<0)

/* SIM guard_cntl register bits */
#define SIM_GUARD_CNTL_RCVR11              (1<<8)
#define SIM_GIARD_CNTL_GETU(x)           (x&0xff)
#define SIM_GIARD_CNTL_GETU_MASK           (0xff)

/* SIM port[0|]_detect register bits */
#define SIM_PORT_DETECT_SPDS  (1<<3)
#define SIM_PORT_DETECT_SPDP  (1<<2)
#define SIM_PORT_DETECT_SDI   (1<<1)
#define SIM_PORT_DETECT_SDIM  (1<<0)

/* END of REGS definitions */

/* ATR parser data (the parser state is stored in the main device structure) */
typedef struct {
	uint8_t T0;		/* ATR T0 */
	uint8_t TS;		/* ATR TS */
	/* ATR TA1, TB1, TC1, TD1, TB1, ... , TD4 */
	uint8_t TXI[16];
	uint8_t THB[15];	/* ATR historical bytes */
	uint8_t TCK;		/* ATR checksum */
	uint16_t ifc_valid;	/* valid interface characters */
	uint8_t ifc_current_valid;	/* calid ifcs in the current batch */
	uint8_t cnt;		/* number of current batch */
	uint8_t num_hb;		/* number of historical bytes */
} sim_atrparser_t;

/* Main SIM driver structure */
typedef struct {
	/* card inserted = 1, ATR received = 2, card removed = 0 */
	int present;
	/* current ATR or OPS state */
	int state;
	/* current power state */
	int power;
	/* error code occured during transfer */
	int errval;
	struct clk *clk;	/* Clock id */
	uint8_t clk_flag;
	struct resource *res;	/* IO map memory */
	void __iomem *ioaddr;	/* Mapped address */
	int ipb_irq;		/* sim ipb IRQ num */
	int dat_irq;		/* sim dat IRQ num */
	/* parser for incoming ATR stream */
	sim_atrparser_t atrparser;
	/* raw ATR stream received */
	sim_atr_t atr;
	/* communication parameters according to ATR */
	sim_param_t param_atr;
	/* current communication parameters */
	sim_param_t param;
	/* current TPDU or PTS transfer */
	sim_xfer_t xfer;
	/* transfer is on the way = 1, idle = 2 */
	int xfer_ongoing;
	/* remaining bytes to transmit for the current transfer */
	int xmt_remaining;
	/* transmit position */
	int xmt_pos;
	/* receive position / number of bytes received */
	int rcv_count;
	uint8_t rcv_buffer[SIM_RCV_BUFFER_SIZE];
	uint8_t xmt_buffer[SIM_XMT_BUFFER_SIZE];
	/* transfer completion notifier */
	struct completion xfer_done;
	/* async notifier for card and ATR detection */
	struct fasync_struct *fasync;
	/* Platform specific data */
	struct mxc_sim_platform_data *plat_data;
} sim_t;

static int sim_param_F[] = {
	SIM_INTERNAL_CLK, 372, 558, 744, 1116, 1488, 1860, SIM_RFU,
	SIM_RFU, 512, 768, 1024, 1536, 2048, SIM_RFU, SIM_RFU
};

static int sim_param_D[] = {
	SIM_RFU, 64 * 1, 64 * 2, 64 * 4, 64 * 8, 64 * 16, SIM_RFU, SIM_RFU,
	SIM_RFU, SIM_RFU, 64 * 1 / 2, 64 * 1 / 4, 64 * 1 / 8, 64 * 1 / 16,
	64 * 1 / 32, 64 * 1 / 64
};

static struct miscdevice sim_dev;

/* Function: sim_calc_param
 *
 * Description: determine register values depending on communication parameters
 *
 * Parameters:
 * uint32_t fi             ATR frequency multiplier index
 * uint32_t di             ATR frequency divider index
 * uint32_t* ptr_divisor   location to store divisor result
 * uint32_t* ptr_sample12  location to store sample12 result
 *
 * Return Values:
 *  SIM_OK                          calculation finished without errors
 * -SIM_E_PARAM_DIVISOR_RANGE       calculated divisor > 255
 * -SIM_E_PARAM_FBYD_NOTDIVBY8OR12  F/D not divisable by 12 (as required)
 * -SIM_E_PARAM_FBYD_WITHFRACTION   F/D has a remainder
 * -SIM_E_PARAM_DI_INVALID          frequency multiplyer index not supported
 * -SIM_E_PARAM_FI_INVALID          frequency divider index not supported
 */

static int sim_calc_param(uint32_t fi, uint32_t di, uint32_t *ptr_divisor,
			  uint32_t *ptr_sample12)
{
	int32_t errval = SIM_OK;
	int32_t f = sim_param_F[fi];
	int32_t d = sim_param_D[di];
	int32_t stage2_fra = (64 * f) % d;
	int32_t stage2_div = (64 * f) / d;
	uint32_t sample12 = 1;
	uint32_t divisor = 31;

	pr_debug("%s entering.\n", __func__);
	if ((f > 0) || (d > 0)) {
		if (stage2_fra == 0) {
			if ((stage2_div % 12) == 0) {
				sample12 = 1;
				divisor = stage2_div / 12;
			} else if ((stage2_div % 8) == 0) {
				sample12 = 0;
				divisor = stage2_div / 8;
			} else
				sample12 = -1;
			if (sample12 >= 0) {
				if (divisor < 256) {
					pr_debug("fi=%i", fi);
					pr_debug("di=%i", di);
					pr_debug("f=%i", f);
					pr_debug("d=%i/64", d);
					pr_debug("div=%i", stage2_div);
					pr_debug("divisor=%i", divisor);
					pr_debug("sample12=%i\n", sample12);

					*ptr_divisor = divisor;
					*ptr_sample12 = sample12;
					errval = SIM_OK;
				} else
					errval = -SIM_E_PARAM_DIVISOR_RANGE;
			} else
				errval = -SIM_E_PARAM_FBYD_NOTDIVBY8OR12;
		} else
			errval = -SIM_E_PARAM_FBYD_WITHFRACTION;
	} else
		errval = -SIM_E_PARAM_FI_INVALID;

	return errval;
};

/* Function: sim_set_param
 *
 * Description: apply communication parameters (setup devisor and sample12)
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 * sim_param_t* param      pointer to communication parameters
 *
 * Return Values:
 * see function sim_calc_param
 */

static int sim_set_param(sim_t *sim, sim_param_t *param)
{
	uint32_t divisor, sample12, reg_data;
	int errval;

	pr_debug("%s entering.\n", __func__);
	errval = sim_calc_param(param->FI, param->DI, &divisor, &sample12);
	if (errval == SIM_OK) {
		__raw_writel(divisor, sim->ioaddr + DIVISOR);
		if (sample12) {
			reg_data = __raw_readl(sim->ioaddr + CNTL);
			reg_data |= SIM_CNTL_SAMPLE12;
			__raw_writel(reg_data, sim->ioaddr + CNTL);
		} else {
			reg_data = __raw_readl(sim->ioaddr + CNTL);
			reg_data &= ~SIM_CNTL_SAMPLE12;
			__raw_writel(reg_data, sim->ioaddr + CNTL);
		}
	}

	return errval;
};

/* Function: sim_atr_received
 *
 * Description: this function is called whenever a valid ATR has been received.
 * It determines the communication parameters from the ATR received and notifies
 * the user space application with SIGIO.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_atr_received(sim_t *sim)
{
	sim_param_t param_default = SIM_PARAM_DEFAULT;
	sim->param_atr = param_default;

	pr_debug("%s entering.\n", __func__);
	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TA1))) {
		sim->param_atr.FI = sim->atrparser.TXI[SIM_IFC_TA1] >> 4;
		sim->param_atr.DI = sim->atrparser.TXI[SIM_IFC_TA1] & 0x0f;
	}
	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TB1))) {
		sim->param_atr.PI1 = (sim->atrparser.TXI[SIM_IFC_TB1] >> 4)
		    & 0x07;
		sim->param_atr.II = sim->atrparser.TXI[SIM_IFC_TB1] & 0x07f;
	}
	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TC1)))
		sim->param_atr.N = sim->atrparser.TXI[SIM_IFC_TC1];

	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TD1)))
		sim->param_atr.T = sim->atrparser.TXI[SIM_IFC_TD1] & 0x0f;

	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TB2)))
		sim->param_atr.PI2 = sim->atrparser.TXI[SIM_IFC_TB2];

	if (sim->atrparser.ifc_valid & (1 << (SIM_IFC_TC2)))
		sim->param_atr.WWT = sim->atrparser.TXI[SIM_IFC_TC2];

	if (sim->fasync)
		kill_fasync(&sim->fasync, SIGIO, POLL_IN);

};

/* Function: sim_xmt_fill
 *
 * Description: fill the transmit FIFO until the FIFO is full or
 * the end of the transmission has been reached.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_xmt_fill(sim_t *sim)
{
	uint32_t reg_data;
	int bytesleft;

	reg_data = __raw_readl(sim->ioaddr + XMT_FIFO_STAT);
	bytesleft = 16 - ((reg_data >> 8) & 0x0f);

	pr_debug("txfill: remaining=%i bytesleft=%i\n",
		 sim->xmt_remaining, bytesleft);
	if (bytesleft > sim->xmt_remaining)
		bytesleft = sim->xmt_remaining;

	sim->xmt_remaining -= bytesleft;
	for (; bytesleft > 0; bytesleft--) {
		__raw_writel(sim->xmt_buffer[sim->xmt_pos],
			     sim->ioaddr + PORT1_XMT_BUF);
		sim->xmt_pos++;
	};
/* FIXME: optimization - keep filling until fifo full */
};

/* Function: sim_xmt_start
 *
 * Description: initiate a transfer
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 * int pos                 position in the xfer transmit buffer
 * int count               number of bytes to be transmitted
 */

static void sim_xmt_start(sim_t *sim, int pos, int count)
{
	uint32_t reg_data;

	pr_debug("tx\n");
	sim->xmt_remaining = count;
	sim->xmt_pos = pos;
	sim_xmt_fill(sim);

	if (sim->xmt_remaining) {
		reg_data = __raw_readl(sim->ioaddr + INT_MASK);
		reg_data &= ~SIM_INT_MASK_TDTFM;
		__raw_writel(reg_data, sim->ioaddr + INT_MASK);
	} else {
		reg_data = __raw_readl(sim->ioaddr + INT_MASK);
		reg_data &= ~SIM_INT_MASK_TCIM;
		__raw_writel(reg_data, sim->ioaddr + INT_MASK);
		__raw_writel(SIM_XMT_STATUS_TC | SIM_XMT_STATUS_TDTF,
			     sim->ioaddr + XMT_STATUS);
		reg_data = __raw_readl(sim->ioaddr + ENABLE);
		reg_data |= SIM_ENABLE_XMTEN;
		__raw_writel(reg_data, sim->ioaddr + ENABLE);
	}
};

/* Function: sim_atr_add
 *
 * Description: add a byte to the raw ATR string
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 * uint8_t data            byte to be added
 */

static void sim_atr_add(sim_t *sim, uint8_t data)
{
	pr_debug("%s entering.\n", __func__);
	if (sim->atr.size < SIM_ATR_LENGTH_MAX)
		sim->atr.t[sim->atr.size++] = data;
	else
		printk(KERN_ERR "sim.c: ATR received is too big!\n");
};

/* Function: sim_fsm
 *
 * Description: main finite state machine running in ISR context.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 * uint8_t data            byte received
 */

static void sim_fsm(sim_t *sim, uint16_t data)
{
	uint32_t temp, i = 0;
	switch (sim->state) {

		pr_debug("%s stat is %d \n", __func__, sim->state);
		/* OPS FSM */

	case SIM_STATE_OPERATIONAL_IDLE:
		printk(KERN_INFO "data received unexpectidly (%04x)\n", data);
		break;

	case SIM_STATE_OPERATIONAL_COMMAND:
		if (data == sim->xmt_buffer[1]) {
			if (sim->xfer.rcv_length) {
				sim->state = SIM_STATE_OPERATIONAL_RESPONSE;
			} else {
				sim->state = SIM_STATE_OPERATIONAL_STATUS1;
				if (sim->xfer.xmt_length > 5)
					sim_xmt_start(sim, 5,
						      sim->xfer.xmt_length - 5);
			};
		} else if (((data & 0xf0) == 0x60) | ((data & 0xf0) == 0x90)) {
			sim->xfer.sw1 = data;
			sim->state = SIM_STATE_OPERATIONAL_STATUS2;
		} else {
			sim->errval = -SIM_E_NACK;
			complete(&sim->xfer_done);
		};
		break;

	case SIM_STATE_OPERATIONAL_RESPONSE:
		sim->rcv_buffer[sim->rcv_count] = data;
		sim->rcv_count++;
		if (sim->rcv_count == sim->xfer.rcv_length)
			sim->state = SIM_STATE_OPERATIONAL_STATUS1;
		break;

	case SIM_STATE_OPERATIONAL_STATUS1:
		sim->xfer.sw1 = data;
		sim->state = SIM_STATE_OPERATIONAL_STATUS2;
		break;

	case SIM_STATE_OPERATIONAL_STATUS2:
		sim->xfer.sw2 = data;
		sim->state = SIM_STATE_OPERATIONAL_IDLE;
		complete(&sim->xfer_done);
		break;

	case SIM_STATE_OPERATIONAL_PTS:
		sim->rcv_buffer[sim->rcv_count] = data;
		sim->rcv_count++;
		if (sim->rcv_count == sim->xfer.rcv_length)
			sim->state = SIM_STATE_OPERATIONAL_IDLE;
		break;

		/* ATR FSM */

	case SIM_STATE_DETECTED_ATR_T0:
		sim_atr_add(sim, data);
		pr_debug("T0 %02x\n", data);
		sim->atrparser.T0 = data;
		sim->state = SIM_STATE_DETECTED_ATR_TS;
		break;

	case SIM_STATE_DETECTED_ATR_TS:
		sim_atr_add(sim, data);
		pr_debug("TS %02x\n", data);
		sim->atrparser.TS = data;
		if (data & 0xf0) {
			sim->atrparser.ifc_current_valid = (data >> 4) & 0x0f;
			sim->atrparser.num_hb = data & 0x0f;
			sim->atrparser.ifc_valid = 0;
			sim->state = SIM_STATE_DETECTED_ATR_TXI;
			sim->atrparser.cnt = 0;
		} else {
			goto sim_fsm_atr_thb;
		};
		break;

	case SIM_STATE_DETECTED_ATR_TXI:
		sim_atr_add(sim, data);
		i = ffs(sim->atrparser.ifc_current_valid) - 1;
		pr_debug("T%c%i %02x\n", 'A' + i, sim->atrparser.cnt + 1, data);
		sim->atrparser.TXI[SIM_IFC_TXI(i, sim->atrparser.cnt)] = data;
		sim->atrparser.ifc_valid |= 1 << SIM_IFC_TXI(i,
							     sim->atrparser.
							     cnt);
		sim->atrparser.ifc_current_valid &= ~(1 << i);

		if (sim->atrparser.ifc_current_valid == 0) {
			if (i == 3) {
				sim->atrparser.ifc_current_valid = (data >> 4)
				    & 0x0f;
				sim->atrparser.cnt++;

				if (sim->atrparser.cnt >= 4) {
					/* error */
					printk(KERN_ERR "ERROR !\n");
					break;
				};

				if (sim->atrparser.ifc_current_valid == 0)
					goto sim_fsm_atr_thb;
			} else {
sim_fsm_atr_thb:
				if (sim->atrparser.num_hb) {
					sim->state = SIM_STATE_DETECTED_ATR_THB;
					sim->atrparser.cnt = 0;
				} else {
					goto sim_fsm_atr_tck;
				};
			};
		};
		break;

	case SIM_STATE_DETECTED_ATR_THB:
		sim_atr_add(sim, data);
		pr_debug("THB%i %02x\n", i, data);
		sim->atrparser.THB[sim->atrparser.cnt] = data;
		sim->atrparser.cnt++;

		if (sim->atrparser.cnt == sim->atrparser.num_hb) {
sim_fsm_atr_tck:
			i = sim->atrparser.ifc_valid & (1 << (SIM_IFC_TD1));
			temp = sim->atrparser.TXI[SIM_IFC_TD1] & 0x0f;
			if ((i && temp) == SIM_PROTOCOL_T1)
				sim->state = SIM_STATE_DETECTED_ATR_TCK;
			else
				goto sim_fsm_atr_received;
		};
		break;

	case SIM_STATE_DETECTED_ATR_TCK:
		sim_atr_add(sim, data);
		/* checksum not required for T=0 */
		sim->atrparser.TCK = data;
sim_fsm_atr_received:
		sim->state = SIM_STATE_OPERATIONAL_IDLE;
		sim->present = SIM_PRESENT_OPERATIONAL;
		sim_atr_received(sim);
		break;
	};
};

/* Function: sim_irq_handler
 *
 * Description: interrupt service routine.
 *
 * Parameters:
 * int irq                 interrupt number
 * void *dev_id            pointer to SIM device handler
 *
 * Return values:
 * IRQ_HANDLED             OS specific
 */

static irqreturn_t sim_irq_handler(int irq, void *dev_id)
{
	uint32_t reg_data, reg_data0, reg_data1;

	sim_t *sim = (sim_t *) dev_id;

	pr_debug("%s entering\n", __func__);

	reg_data0 = __raw_readl(sim->ioaddr + XMT_STATUS);
	reg_data1 = __raw_readl(sim->ioaddr + INT_MASK);
	if ((reg_data0 & SIM_XMT_STATUS_TC)
	    && (!(reg_data1 & SIM_INT_MASK_TCIM))) {
		pr_debug("TC_IRQ\n");
		__raw_writel(SIM_XMT_STATUS_TC, sim->ioaddr + XMT_STATUS);
		reg_data = __raw_readl(sim->ioaddr + INT_MASK);
		reg_data |= SIM_INT_MASK_TCIM;
		__raw_writel(reg_data, sim->ioaddr + INT_MASK);
		reg_data = __raw_readl(sim->ioaddr + ENABLE);
		reg_data &= ~SIM_ENABLE_XMTEN;
		__raw_writel(reg_data, sim->ioaddr + ENABLE);
	};

	reg_data0 = __raw_readl(sim->ioaddr + XMT_STATUS);
	reg_data1 = __raw_readl(sim->ioaddr + INT_MASK);
	if ((reg_data0 & SIM_XMT_STATUS_TDTF)
	    && (!(reg_data1 & SIM_INT_MASK_TDTFM))) {
		pr_debug("TDTF_IRQ\n");
		__raw_writel(SIM_XMT_STATUS_TDTF, sim->ioaddr + XMT_STATUS);
		sim_xmt_fill(sim);

		if (sim->xmt_remaining == 0) {
			__raw_writel(SIM_XMT_STATUS_TC,
				     sim->ioaddr + XMT_STATUS);
			reg_data = __raw_readl(sim->ioaddr + INT_MASK);
			reg_data &= ~SIM_INT_MASK_TCIM;
			reg_data |= SIM_INT_MASK_TDTFM;
			__raw_writel(reg_data, sim->ioaddr + INT_MASK);
		};
	};

	reg_data0 = __raw_readl(sim->ioaddr + RCV_STATUS);
	reg_data1 = __raw_readl(sim->ioaddr + INT_MASK);
	if ((reg_data0 & SIM_RCV_STATUS_RDRF)
	    && (!(reg_data1 & SIM_INT_MASK_RIM))) {
		pr_debug("%s RDRF_IRQ\n", __func__);
		__raw_writel(SIM_RCV_STATUS_RDRF, sim->ioaddr + RCV_STATUS);

		while (__raw_readl(sim->ioaddr + RCV_FIFO_CNT)) {
			uint32_t data;
			data = __raw_readl(sim->ioaddr + PORT1_RCV_BUF);
			pr_debug("RX = %02x state = %i\n", data, sim->state);
			if (data & 0x700) {
				if (sim->xfer_ongoing) {
					/* error */
					printk(KERN_ERR "ERROR !\n");
					return IRQ_HANDLED;
				};
			} else
				sim_fsm(sim, data);
		};
	};

	reg_data0 = __raw_readl(sim->ioaddr + PORT0_DETECT);
	if (reg_data0 & SIM_PORT_DETECT_SDI) {
		pr_debug("%s PD_IRQ\n", __func__);
		reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
		reg_data |= SIM_PORT_DETECT_SDI;
		__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);

		reg_data0 = __raw_readl(sim->ioaddr + PORT0_DETECT);
		if (reg_data0 & SIM_PORT_DETECT_SPDP) {
			reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
			reg_data &= ~SIM_PORT_DETECT_SPDS;
			__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);

			if (sim->present != SIM_PRESENT_REMOVED) {
				pr_debug("Removed sim card\n");
				sim->present = SIM_PRESENT_REMOVED;
				sim->state = SIM_STATE_REMOVED;

				if (sim->fasync)
					kill_fasync(&sim->fasync,
						    SIGIO, POLL_IN);
			};
		} else {
			reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
			reg_data |= SIM_PORT_DETECT_SPDS;
			__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);

			if (sim->present == SIM_PRESENT_REMOVED) {
				pr_debug("Inserted sim card\n");
				sim->state = SIM_STATE_DETECTED_ATR_T0;
				sim->present = SIM_PRESENT_DETECTED;

				if (sim->fasync)
					kill_fasync(&sim->fasync,
						    SIGIO, POLL_IN);
			};
		};
	};

	return IRQ_HANDLED;
};

/* Function: sim_power_on
 *
 * Description: run the power on sequence
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_power_on(sim_t *sim)
{
	uint32_t reg_data;

	/* power on sequence */
	pr_debug("%s Powering on the sim port.\n", __func__);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data |= SIM_PORT_CNTL_SVEN;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	msleep(10);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data |= SIM_PORT_CNTL_SCEN;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	msleep(10);
	reg_data = SIM_RCV_THRESHOLD_RTH(0) | SIM_RCV_THRESHOLD_RDT(1);
	__raw_writel(reg_data, sim->ioaddr + RCV_THRESHOLD);
	__raw_writel(SIM_RCV_STATUS_RDRF, sim->ioaddr + RCV_STATUS);
	reg_data = __raw_readl(sim->ioaddr + INT_MASK);
	reg_data &= ~SIM_INT_MASK_RIM;
	__raw_writel(reg_data, sim->ioaddr + INT_MASK);
	__raw_writel(31, sim->ioaddr + DIVISOR);
	reg_data = __raw_readl(sim->ioaddr + CNTL);
	reg_data |= SIM_CNTL_SAMPLE12;
	__raw_writel(reg_data, sim->ioaddr + CNTL);
	reg_data = __raw_readl(sim->ioaddr + ENABLE);
	reg_data |= SIM_ENABLE_RCVEN;
	__raw_writel(reg_data, sim->ioaddr + ENABLE);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data |= SIM_PORT_CNTL_SRST;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	pr_debug("%s port0_ctl is 0x%x.\n", __func__,
		 __raw_readl(sim->ioaddr + PORT0_CNTL));
	sim->power = SIM_POWER_ON;
};

/* Function: sim_power_off
 *
 * Description: run the power off sequence
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_power_off(sim_t *sim)
{
	uint32_t reg_data;

	pr_debug("%s entering.\n", __func__);
	/* sim_power_off sequence */
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data &= ~SIM_PORT_CNTL_SCEN;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	reg_data = __raw_readl(sim->ioaddr + ENABLE);
	reg_data &= ~SIM_ENABLE_RCVEN;
	__raw_writel(reg_data, sim->ioaddr + ENABLE);
	reg_data = __raw_readl(sim->ioaddr + INT_MASK);
	reg_data |= SIM_INT_MASK_RIM;
	__raw_writel(reg_data, sim->ioaddr + INT_MASK);
	__raw_writel(0, sim->ioaddr + RCV_THRESHOLD);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data &= ~SIM_PORT_CNTL_SRST;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data &= ~SIM_PORT_CNTL_SVEN;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	sim->power = SIM_POWER_OFF;
};

/* Function: sim_start
 *
 * Description: ramp up the SIM interface
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_start(sim_t *sim)
{
	uint32_t reg_data, clk_rate, clk_div = 0;

	pr_debug("%s entering.\n", __func__);
	/* Configuring SIM for Operation */
	reg_data = SIM_XMT_THRESHOLD_XTH(0) | SIM_XMT_THRESHOLD_TDT(4);
	__raw_writel(reg_data, sim->ioaddr + XMT_THRESHOLD);
	__raw_writel(0, sim->ioaddr + SETUP);
	/* ~ 4 MHz */
	clk_rate = clk_get_rate(sim->clk);
	clk_div = clk_rate / sim->plat_data->clk_rate;
	if (clk_rate % sim->plat_data->clk_rate)
		clk_div++;
	pr_debug("%s prescaler is 0x%x.\n", __func__, clk_div);
	__raw_writel(clk_div, sim->ioaddr + CLK_PRESCALER);

	reg_data = SIM_CNTL_GPCNT_CLK_SEL(0) | SIM_CNTL_BAUD_SEL(7)
	    | SIM_CNTL_SAMPLE12 | SIM_CNTL_ANACK | SIM_CNTL_ICM;
	__raw_writel(reg_data, sim->ioaddr + CNTL);
	__raw_writel(31, sim->ioaddr + DIVISOR);
	reg_data = __raw_readl(sim->ioaddr + OD_CONFIG);
	reg_data |= SIM_OD_CONFIG_OD_P0;
	__raw_writel(reg_data, sim->ioaddr + OD_CONFIG);
	reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
	reg_data |= SIM_PORT_CNTL_3VOLT | SIM_PORT_CNTL_STEN;
	__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);

	/* presense detect */
	pr_debug("%s p0_det is 0x%x \n", __func__,
		 __raw_readl(sim->ioaddr + PORT0_DETECT));
	if (__raw_readl(sim->ioaddr + PORT0_DETECT) & SIM_PORT_DETECT_SPDP) {
		pr_debug("%s card removed \n", __func__);
		reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
		reg_data &= ~SIM_PORT_DETECT_SPDS;
		__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);
		sim->present = SIM_PRESENT_REMOVED;
		sim->state = SIM_STATE_REMOVED;
	} else {
		pr_debug("%s card inserted \n", __func__);
		reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
		reg_data |= SIM_PORT_DETECT_SPDS;
		__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);
		sim->present = SIM_PRESENT_DETECTED;
		sim->state = SIM_STATE_DETECTED_ATR_T0;
	};
	reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
	reg_data |= SIM_PORT_DETECT_SDI;
	reg_data &= ~SIM_PORT_DETECT_SDIM;
	__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);

	/*
	 * Since there is no PD0 layout on MX51, assume
	 * that there is a SIM card in slot defaulty.
	 * */
	if (0 == (sim->plat_data->detect)) {
		reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
		reg_data |= SIM_PORT_DETECT_SPDS;
		__raw_writel(reg_data, sim->ioaddr + PORT0_DETECT);
		sim->present = SIM_PRESENT_DETECTED;
		sim->state = SIM_STATE_DETECTED_ATR_T0;
	}

	if (sim->present == SIM_PRESENT_DETECTED)
		sim_power_on(sim);

};

/* Function: sim_stop
 *
 * Description: shut down the SIM interface
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_stop(sim_t *sim)
{
	pr_debug("%s entering.\n", __func__);
	__raw_writel(0, sim->ioaddr + SETUP);
	__raw_writel(0, sim->ioaddr + ENABLE);
	__raw_writel(0, sim->ioaddr + PORT0_CNTL);
	__raw_writel(0x06, sim->ioaddr + CNTL);
	__raw_writel(0, sim->ioaddr + CLK_PRESCALER);
	__raw_writel(0, sim->ioaddr + SETUP);
	__raw_writel(0, sim->ioaddr + OD_CONFIG);
	__raw_writel(0, sim->ioaddr + XMT_THRESHOLD);
	__raw_writel(0xb8, sim->ioaddr + XMT_STATUS);
	__raw_writel(4, sim->ioaddr + RESET_CNTL);
	mdelay(1);
};

/* Function: sim_data_reset
 *
 * Description: reset a SIM structure to default values
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_data_reset(sim_t *sim)
{
	sim_param_t param_default = SIM_PARAM_DEFAULT;
	sim->present = SIM_PRESENT_REMOVED;
	sim->state = SIM_STATE_REMOVED;
	sim->power = SIM_POWER_OFF;
	sim->errval = SIM_OK;
	memset(&sim->atrparser, 0, sizeof(sim->atrparser));
	memset(&sim->atr, 0, sizeof(sim->atr));
	sim->param_atr = param_default;
	memset(&sim->param, 0, sizeof(sim->param));
	memset(&sim->xfer, 0, sizeof(sim->xfer));
	sim->xfer_ongoing = 0;
	sim->xmt_remaining = 0;
	sim->xmt_pos = 0;
	sim->rcv_count = 0;
	memset(sim->rcv_buffer, 0, SIM_RCV_BUFFER_SIZE);
	memset(sim->xmt_buffer, 0, SIM_XMT_BUFFER_SIZE);
};

/* Function: sim_cold_reset
 *
 * Description: cold reset the SIM interface, including card
 * power down and interface hardware reset.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_cold_reset(sim_t *sim)
{
	pr_debug("%s entering.\n", __func__);
	if (sim->present != SIM_PRESENT_REMOVED) {
		sim_power_off(sim);
		sim_stop(sim);
		sim_data_reset(sim);
		sim->state = SIM_STATE_DETECTED_ATR_T0;
		sim->present = SIM_PRESENT_DETECTED;
		msleep(50);
		sim_start(sim);
		sim_power_on(sim);
	};
};

/* Function: sim_warm_reset
 *
 * Description: warm reset the SIM interface: just invoke the
 * reset signal and reset the SIM structure for the interface.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static void sim_warm_reset(sim_t *sim)
{
	uint32_t reg_data;

	pr_debug("%s entering.\n", __func__);
	if (sim->present != SIM_PRESENT_REMOVED) {
		reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
		reg_data |= SIM_PORT_CNTL_SRST;
		__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
		sim_data_reset(sim);
		msleep(50);
		reg_data = __raw_readl(sim->ioaddr + PORT0_CNTL);
		reg_data &= ~SIM_PORT_CNTL_SRST;
		__raw_writel(reg_data, sim->ioaddr + PORT0_CNTL);
	};
};

/* Function: sim_card_lock
 *
 * Description: physically lock the SIM card.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static int sim_card_lock(sim_t *sim)
{
	int errval;

	pr_debug("%s entering.\n", __func__);
	/* place holder for true physcial locking */
	if (sim->present != SIM_PRESENT_REMOVED)
		errval = SIM_OK;
	else
		errval = -SIM_E_NOCARD;
	return errval;
};

/* Function: sim_card_eject
 *
 * Description: physically unlock and eject the SIM card.
 *
 * Parameters:
 * sim_t* sim              pointer to SIM device handler
 */

static int sim_card_eject(sim_t *sim)
{
	int errval;

	pr_debug("%s entering.\n", __func__);
	/* place holder for true physcial locking */
	if (sim->present != SIM_PRESENT_REMOVED)
		errval = SIM_OK;
	else
		errval = -SIM_E_NOCARD;
	return errval;
};

/* Function: sim_ioctl
 *
 * Description: handle ioctl calls
 *
 * Parameters: OS specific
 */

static int sim_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int ret, errval = SIM_OK;
	unsigned long timeout;

	sim_t *sim = (sim_t *) file->private_data;

	pr_debug("%s entering.\n", __func__);
	switch (cmd) {
		pr_debug("ioctl cmd %d is issued...\n", cmd);

	case SIM_IOCTL_GET_ATR:
		if (sim->present != SIM_PRESENT_OPERATIONAL) {
			errval = -SIM_E_NOCARD;
			break;
		};
		ret = copy_to_user((sim_atr_t *) arg, &sim->atr,
				   sizeof(sim_atr_t));
		if (ret)
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_GET_PARAM_ATR:
		if (sim->present != SIM_PRESENT_OPERATIONAL) {
			errval = -SIM_E_NOCARD;
			break;
		};
		ret = copy_to_user((sim_param_t *) arg, &sim->param_atr,
				   sizeof(sim_param_t));
		if (ret)
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_GET_PARAM:
		ret = copy_to_user((sim_param_t *) arg, &sim->param,
				   sizeof(sim_param_t));
		if (ret)
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_SET_PARAM:
		ret = copy_from_user(&sim->param, (sim_param_t *) arg,
				     sizeof(sim_param_t));
		if (ret)
			errval = -SIM_E_ACCESS;
		else
			errval = sim_set_param(sim, &sim->param);
		break;

	case SIM_IOCTL_POWER_ON:
		if (sim->power == SIM_POWER_ON) {
			errval = -SIM_E_POWERED_ON;
			break;
		};
		sim_power_on(sim);
		break;

	case SIM_IOCTL_POWER_OFF:
		if (sim->power == SIM_POWER_OFF) {
			errval = -SIM_E_POWERED_OFF;
			break;
		};
		sim_power_off(sim);
		break;

	case SIM_IOCTL_COLD_RESET:
		if (sim->power == SIM_POWER_OFF) {
			errval = -SIM_E_POWERED_OFF;
			break;
		};
		sim_cold_reset(sim);
		break;

	case SIM_IOCTL_WARM_RESET:
		sim_warm_reset(sim);
		if (sim->power == SIM_POWER_OFF) {
			errval = -SIM_E_POWERED_OFF;
			break;
		};
		break;

	case SIM_IOCTL_XFER:
		if (sim->present != SIM_PRESENT_OPERATIONAL) {
			errval = -SIM_E_NOCARD;
			break;
		};

		ret = copy_from_user(&sim->xfer, (sim_xfer_t *) arg,
				     sizeof(sim_xfer_t));
		if (ret) {
			errval = -SIM_E_ACCESS;
			break;
		};

		ret = copy_from_user(sim->xmt_buffer, sim->xfer.xmt_buffer,
				     sim->xfer.xmt_length);
		if (ret) {
			errval = -SIM_E_ACCESS;
			break;
		};

		sim->rcv_count = 0;
		sim->xfer.sw1 = 0;
		sim->xfer.sw2 = 0;

		if (sim->xfer.type == SIM_XFER_TYPE_TPDU) {
			if (sim->xfer.xmt_length < 5) {
				errval = -SIM_E_TPDUSHORT;
				break;
			}
			sim->state = SIM_STATE_OPERATIONAL_COMMAND;
		} else if (sim->xfer.type == SIM_XFER_TYPE_PTS) {
			if (sim->xfer.xmt_length == 0) {
				errval = -SIM_E_PTSEMPTY;
				break;
			}
			sim->state = SIM_STATE_OPERATIONAL_PTS;
		} else {
			errval = -SIM_E_INVALIDXFERTYPE;
			break;
		};

		if (sim->xfer.xmt_length > SIM_XMT_BUFFER_SIZE) {
			errval = -SIM_E_INVALIDXMTLENGTH;
			break;
		};

		if (sim->xfer.rcv_length > SIM_XMT_BUFFER_SIZE) {
			errval = -SIM_E_INVALIDRCVLENGTH;
			break;
		};

		sim->errval = 0;
		sim->xfer_ongoing = 1;
		init_completion(&sim->xfer_done);
		sim_xmt_start(sim, 0, 5);
		timeout =
		    wait_for_completion_interruptible_timeout(&sim->xfer_done,
							      sim->xfer.
							      timeout);
		sim->xfer_ongoing = 0;

		if (sim->errval) {
			errval = sim->errval;
			break;
		};

		if (timeout == 0) {
			errval = -SIM_E_TIMEOUT;
			break;
		}

		ret = copy_to_user(sim->xfer.rcv_buffer, sim->rcv_buffer,
				   sim->xfer.rcv_length);
		if (ret) {
			errval = -SIM_E_ACCESS;
			break;
		};

		ret = copy_to_user((sim_xfer_t *) arg, &sim->xfer,
				   sizeof(sim_xfer_t));
		if (ret)
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_GET_PRESENSE:
		if (put_user(sim->present, (int *)arg))
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_CARD_LOCK:
		errval = sim_card_lock(sim);
		break;

	case SIM_IOCTL_CARD_EJECT:
		errval = sim_card_eject(sim);
		break;

	};

	return errval;
};

/* Function: sim_fasync
 *
 * Description: async handler
 *
 * Parameters: OS specific
 */

static int sim_fasync(int fd, struct file *file, int mode)
{
	sim_t *sim = (sim_t *) file->private_data;
	pr_debug("%s entering.\n", __func__);
	return fasync_helper(fd, file, mode, &sim->fasync);
}

/* Function: sim_open
 *
 * Description: ramp up interface when being opened
 *
 * Parameters: OS specific
 */

static int sim_open(struct inode *inode, struct file *file)
{
	int errval = SIM_OK;

	sim_t *sim = dev_get_drvdata(sim_dev.parent);
	file->private_data = sim;

	pr_debug("%s entering.\n", __func__);
	if (!sim->ioaddr) {
		errval = -ENOMEM;
		return errval;
	}

	if (!(sim->clk_flag)) {
		pr_debug("\n%s enable the clock\n", __func__);
		clk_enable(sim->clk);
		sim->clk_flag = 1;
	}

	sim_start(sim);

	return errval;
};

/* Function: sim_release
 *
 * Description: shut down interface when being closed
 *
 * Parameters: OS specific
 */

static int sim_release(struct inode *inode, struct file *file)
{
	uint32_t reg_data;

	sim_t *sim = (sim_t *) file->private_data;

	pr_debug("%s entering.\n", __func__);
	if (sim->clk_flag) {
		pr_debug("\n%s disable the clock\n", __func__);
		clk_disable(sim->clk);
		sim->clk_flag = 0;
	}

	/* disable presense detection */
	reg_data = __raw_readl(sim->ioaddr + PORT0_DETECT);
	__raw_writel(reg_data | SIM_PORT_DETECT_SDIM,
		     sim->ioaddr + PORT0_DETECT);

	if (sim->present != SIM_PRESENT_REMOVED) {
		sim_power_off(sim);
		if (sim->fasync)
			kill_fasync(&sim->fasync, SIGIO, POLL_IN);
	};

	sim_stop(sim);

	sim_fasync(-1, file, 0);

	pr_debug("exit\n");
	return 0;
};

static const struct file_operations sim_fops = {
	.open = sim_open,
	.ioctl = sim_ioctl,
	.fasync = sim_fasync,
	.release = sim_release
};

static struct miscdevice sim_dev = {
	MISC_DYNAMIC_MINOR,
	"mxc_sim",
	&sim_fops
};

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int sim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mxc_sim_platform_data *sim_plat = pdev->dev.platform_data;

	sim_t *sim = kzalloc(sizeof(sim_t), GFP_KERNEL);

	if (sim == 0) {
		ret = -ENOMEM;
		printk(KERN_ERR "Can't get the MEMORY\n");
		return ret;
	};

	BUG_ON(pdev == NULL);

	sim->plat_data = sim_plat;
	sim->clk_flag = 0;

	sim->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!sim->res) {
		ret = -ENOMEM;
		printk(KERN_ERR "Can't get the MEMORY\n");
		goto out;
	}

	/* request the sim clk and sim_serial_clk */
	sim->clk = clk_get(NULL, sim->plat_data->clock_sim);
	if (IS_ERR(sim->clk)) {
		ret = PTR_ERR(sim->clk);
		printk(KERN_ERR "Get CLK ERROR !\n");
		goto out;
	}
	pr_debug("sim clock:%lu\n", clk_get_rate(sim->clk));

	sim->ipb_irq = platform_get_irq(pdev, 0);
	sim->dat_irq = platform_get_irq(pdev, 1);
	if (!(sim->ipb_irq | sim->dat_irq)) {
		ret = -ENOMEM;
		goto out1;
	}

	if (!request_mem_region(sim->res->start,
				sim->res->end -
				sim->res->start + 1, pdev->name)) {
		printk(KERN_ERR "request_mem_region failed\n");
		ret = -ENOMEM;
		goto out1;
	}

	sim->ioaddr = (void *)ioremap(sim->res->start, sim->res->end -
				      sim->res->start + 1);
	if (sim->ipb_irq)
		ret = request_irq(sim->ipb_irq, sim_irq_handler,
				  0, "mxc_sim_ipb", sim);
	if (sim->dat_irq)
		ret |= request_irq(sim->dat_irq, sim_irq_handler,
				   0, "mxc_sim_dat", sim);

	if (ret) {
		printk(KERN_ERR "Can't get the irq\n");
		goto out2;
	};

	platform_set_drvdata(pdev, sim);
	sim_dev.parent = &(pdev->dev);

	misc_register(&sim_dev);

	return ret;
out2:
	if (sim->ipb_irq)
		free_irq(sim->ipb_irq, sim);
	if (sim->dat_irq)
		free_irq(sim->dat_irq, sim);
	release_mem_region(sim->res->start,
			   sim->res->end - sim->res->start + 1);
out1:
	clk_put(sim->clk);
out:
	kfree(sim);
	return ret;
}

static int sim_remove(struct platform_device *pdev)
{
	sim_t *sim = platform_get_drvdata(pdev);

	clk_put(sim->clk);

	if (sim->ipb_irq)
		free_irq(sim->ipb_irq, sim);
	if (sim->dat_irq)
		free_irq(sim->dat_irq, sim);

	iounmap(sim->ioaddr);

	kfree(sim);
	release_mem_region(sim->res->start,
			   sim->res->end - sim->res->start + 1);

	misc_deregister(&sim_dev);
	return 0;
}

static struct platform_driver sim_driver = {
	.driver = {
		   .name = "mxc_sim",
		   },
	.probe = sim_probe,
	.remove = sim_remove,
	.suspend = NULL,
	.resume = NULL,
};

static int __init sim_drv_init(void)
{
	return platform_driver_register(&sim_driver);
}

static void __exit sim_drv_exit(void)
{
	platform_driver_unregister(&sim_driver);
}

module_init(sim_drv_init);
module_exit(sim_drv_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC SIM Driver");
MODULE_LICENSE("GPL");
