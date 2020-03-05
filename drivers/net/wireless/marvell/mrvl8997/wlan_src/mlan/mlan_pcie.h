/** @file mlan_pcie.h
 *
 *  @brief This file contains definitions for PCIE interface.
 *  driver.
 *
 *
 *  Copyright 2014-2020 NXP
 *
 *  This software file (the File) is distributed by NXP
 *  under the terms of the GNU General Public License Version 2, June 1991
 *  (the License).  You may use, redistribute and/or modify the File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

/********************************************************
Change log:
    02/01/2012: initial version
********************************************************/

#ifndef _MLAN_PCIE_H_
#define _MLAN_PCIE_H_

/* PCIE INTERNAL REGISTERS */
/** PCIE data exchange register 0 */
#define PCIE_SCRATCH_0_REG              0x0C10
/** PCIE data exchange register 1 */
#define PCIE_SCRATCH_1_REG              0x0C14
/** PCIE CPU interrupt events */
#define PCIE_CPU_INT_EVENT              0x0C18
/** PCIE CPU interrupt status */
#define PCIE_CPU_INT_STATUS             0x0C1C

/** PCIe CPU Interrupt Status Mask */
#define PCIE_CPU_INT2ARM_ISM     	0x0C28
/** PCIE host interrupt status */
#define PCIE_HOST_INT_STATUS            0x0C30
/** PCIE host interrupt mask */
#define PCIE_HOST_INT_MASK              0x0C34
/** PCIE host interrupt status mask */
#define PCIE_HOST_INT_STATUS_MASK       0x0C3C
/** PCIE data exchange register 2 */
#define PCIE_SCRATCH_2_REG              0x0C40
/** PCIE data exchange register 3 */
#define PCIE_SCRATCH_3_REG              0x0C44

#define PCIE_IP_REV_REG                 0x0C48

/** PCIE data exchange register 4 */
#define PCIE_SCRATCH_4_REG              0x0CD0
/** PCIE data exchange register 5 */
#define PCIE_SCRATCH_5_REG              0x0CD4
/** PCIE data exchange register 6 */
#define PCIE_SCRATCH_6_REG              0x0CD8
/** PCIE data exchange register 7 */
#define PCIE_SCRATCH_7_REG              0x0CDC
/** PCIE data exchange register 8 */
#define PCIE_SCRATCH_8_REG              0x0CE0
/** PCIE data exchange register 9 */
#define PCIE_SCRATCH_9_REG              0x0CE4
/** PCIE data exchange register 10 */
#define PCIE_SCRATCH_10_REG             0x0CE8
/** PCIE data exchange register 11 */
#define PCIE_SCRATCH_11_REG             0x0CEC
/** PCIE data exchange register 12 */
#define PCIE_SCRATCH_12_REG             0x0CF0

/* PCIE read data pointer for queue 0 and 1 */
#define PCIE_RD_DATA_PTR_Q0_Q1          0xC1A4	/* 0x8000C1A4 */
/* PCIE read data pointer for queue 2 and 3 */
#define PCIE_RD_DATA_PTR_Q2_Q3          0xC1A8	/* 0x8000C1A8 */
/* PCIE write data pointer for queue 0 and 1 */
#define PCIE_WR_DATA_PTR_Q0_Q1          0xC174	/* 0x8000C174 */
/* PCIE write data pointer for queue 2 and 3 */
#define PCIE_WR_DATA_PTR_Q2_Q3          0xC178	/* 0x8000C178 */

/** Download ready interrupt for CPU */
#define CPU_INTR_DNLD_RDY               MBIT(0)
/** Command ready interrupt for CPU */
#define CPU_INTR_DOOR_BELL              MBIT(1)
/** Confirmation that sleep confirm message has been processed.
 Device will enter sleep after receiving this interrupt */
#define CPU_INTR_SLEEP_CFM_DONE         MBIT(2)
/** Reset interrupt for CPU */
#define CPU_INTR_RESET                  MBIT(3)
/** Set Event Done interupt to the FW*/
#define CPU_INTR_EVENT_DONE             MBIT(5)

/** Data sent interrupt for host */
#define HOST_INTR_DNLD_DONE             MBIT(0)
/** Data receive interrupt for host */
#define HOST_INTR_UPLD_RDY              MBIT(1)
/** Command sent interrupt for host */
#define HOST_INTR_CMD_DONE              MBIT(2)
/** Event ready interrupt for host */
#define HOST_INTR_EVENT_RDY             MBIT(3)
/** Interrupt mask for host */
#define HOST_INTR_MASK		       (HOST_INTR_DNLD_DONE | \
					HOST_INTR_UPLD_RDY  | \
					HOST_INTR_CMD_DONE  | \
					HOST_INTR_EVENT_RDY)

/** Lower 32bits command address holding register */
#define REG_CMD_ADDR_LO                 PCIE_SCRATCH_0_REG
/** Upper 32bits command address holding register */
#define REG_CMD_ADDR_HI                 PCIE_SCRATCH_1_REG
/** Command length holding register */
#define REG_CMD_SIZE                    PCIE_SCRATCH_2_REG

/** Lower 32bits command response address holding register */
#define REG_CMDRSP_ADDR_LO              PCIE_SCRATCH_4_REG
/** Upper 32bits command response address holding register */
#define REG_CMDRSP_ADDR_HI              PCIE_SCRATCH_5_REG

/* TX buffer description read pointer */
#define REG_TXBD_RDPTR                  PCIE_RD_DATA_PTR_Q0_Q1
/* TX buffer description write pointer */
#define REG_TXBD_WRPTR                  PCIE_WR_DATA_PTR_Q0_Q1
/* TX buffer 2 description read pointer */
#define REG_TXBD2_RDPTR                 PCIE_RD_DATA_PTR_Q2_Q3
/* TX buffer 2 description write pointer */
#define REG_TXBD2_WRPTR                 PCIE_WR_DATA_PTR_Q2_Q3
/* RX buffer description write pointer */
#define REG_RXBD_WRPTR                  PCIE_WR_DATA_PTR_Q0_Q1
/* RX buffer description read pointer */
#define REG_RXBD_RDPTR                  PCIE_RD_DATA_PTR_Q0_Q1
/** TxBD's Read/Write pointer start from bit 16 */
#define TXBD_RW_PTR_START               16
/** RxBD's Read/Write pointer start from bit 0 */
#define RXBD_RW_PTR_STRAT               0
/** Tx/Rx Read/Write pointer's mask */
#define TXRX_RW_PTR_MASK                     0x00000FFF
/** Tx/Rx Read/Write pointer's wrap mask */
#define TXRX_RW_PTR_WRAP_MASK                0x00001FFF
/** Tx/Rx Read/Write pointer's rollover indicate bit */
#define TXRX_RW_PTR_ROLLOVER_IND             MBIT(12)

#define MLAN_BD_FLAG_SOP                MBIT(0)
#define MLAN_BD_FLAG_EOP                MBIT(1)
#define MLAN_BD_FLAG_XS_SOP             MBIT(2)
#define MLAN_BD_FLAG_XS_EOP             MBIT(3)

#define REG_EVTBD_WRPTR                 PCIE_SCRATCH_10_REG
/* Event buffer description read pointer */
#define REG_EVTBD_RDPTR                 PCIE_SCRATCH_11_REG
/* Driver ready signature write pointer */
#define REG_DRV_READY                   PCIE_SCRATCH_12_REG

/** Event Read/Write pointer mask */
#define EVT_RW_PTR_MASK                 0x0f
/** Event Read/Write pointer rollover bit */
#define EVT_RW_PTR_ROLLOVER_IND         MBIT(7)

/* Define PCIE block size for firmware download */
#define MLAN_PCIE_BLOCK_SIZE_FW_DNLD    256

/** Extra added macros **/
#define MLAN_EVENT_HEADER_LEN           8

/** Max interrupt status register read limit */
#define MAX_READ_REG_RETRY              10000

/** Set PCIE host buffer configurations */
mlan_status wlan_set_pcie_buf_config(mlan_private *pmpriv);
/** Prepare command PCIE host buffer config */
mlan_status wlan_cmd_pcie_host_buf_cfg(pmlan_private pmpriv,
				       IN HostCmd_DS_COMMAND *cmd,
				       IN t_u16 cmd_action,
				       IN t_void *pdata_buf);

/** Wakeup PCIE card */
mlan_status wlan_pcie_wakeup(mlan_adapter *pmadapter);
/** Enable host interrupt */
mlan_status wlan_enable_host_int(pmlan_adapter pmadapter);

/** PCIE command response completion function */
mlan_status wlan_pcie_cmdrsp_complete(mlan_adapter *pmadapter,
				      mlan_buffer *pmbuf);
/** PCIE event completion function */
mlan_status wlan_pcie_event_complete(mlan_adapter *pmadapter,
				     mlan_buffer *pmbuf);

/** Set DRV_READY register */
mlan_status wlan_set_drv_ready_reg(mlan_adapter *pmadapter, t_u32 val);
/** multi interface download check */
mlan_status wlan_check_winner_status(mlan_adapter *pmadapter, t_u32 *val);
/** Firmware status check */
mlan_status wlan_check_fw_status(mlan_adapter *pmadapter, t_u32 pollnum);
/** PCIE init */
mlan_status wlan_pcie_init(mlan_adapter *pmadapter);

/** Init Firmware */
mlan_status wlan_pcie_init_fw(pmlan_adapter pmadapter);

/** Read interrupt status */
mlan_status wlan_interrupt(t_u16 msg_id, pmlan_adapter pmadapter);
mlan_status wlan_process_msix_int(mlan_adapter *pmadapter);
/** Process Interrupt Status */
mlan_status wlan_process_int_status(mlan_adapter *pmadapter);
/** Transfer data to card */
mlan_status wlan_pcie_host_to_card(mlan_adapter *pmadapter, t_u8 type,
				   mlan_buffer *mbuf, mlan_tx_param *tx_param);
/** Ring buffer allocation function */
mlan_status wlan_alloc_pcie_ring_buf(pmlan_adapter pmadapter);
/** Ring buffer deallocation function */
mlan_status wlan_free_pcie_ring_buf(pmlan_adapter pmadapter);
/** Ring buffer cleanup function, e.g. on deauth */
mlan_status wlan_clean_pcie_ring_buf(pmlan_adapter pmadapter);
#endif /* _MLAN_PCIE_H_ */
