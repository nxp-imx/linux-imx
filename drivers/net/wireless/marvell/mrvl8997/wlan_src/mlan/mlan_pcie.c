/** @file mlan_pcie.c
 *
 *  @brief This file contains PCI-E specific code
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

#include "mlan.h"
#ifdef STA_SUPPORT
#include "mlan_join.h"
#endif
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_init.h"
#include "mlan_wmm.h"
#include "mlan_11n.h"
#include "mlan_pcie.h"

/********************************************************
			Local Variables
********************************************************/

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

static mlan_status wlan_pcie_delete_evtbd_ring(mlan_adapter *pmadapter);
static mlan_status wlan_pcie_delete_rxbd_ring(mlan_adapter *pmadapter);

/* Disable the host interrupts */
mlan_status wlan_disable_host_int(mlan_adapter *pmadapter);

/**
 *  @brief This function disables the host interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_disable_host_int_mask(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE_HOST_INT_MASK,
				0x00000000)) {
		PRINTM(MWARN, "Disable host interrupt failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function enables the host interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_enable_host_int_mask(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	/* Simply write the mask to the register */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE_HOST_INT_MASK,
				HOST_INTR_MASK)) {
		PRINTM(MWARN, "Enable host interrupt failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function enables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_enable_host_int_status_mask(mlan_adapter *pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	/* Enable host int status mask */
	if (pcb->
	    moal_write_reg(pmadapter->pmoal_handle, PCIE_HOST_INT_STATUS_MASK,
			   HOST_INTR_MASK)) {
		PRINTM(MWARN, "Enable host interrupt status mask failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function creates buffer descriptor ring for TX
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_create_txbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 i;

	ENTER();
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer.
	 */
	pmadapter->txbd_wrptr = 0;
	pmadapter->txbd_pending = 0;
	pmadapter->txbd_rdptr = 0;

	/* allocate shared memory for the BD ring and divide the same in to
	   several descriptors */
	pmadapter->txbd_ring_size =
		sizeof(mlan_pcie_data_buf) * MLAN_MAX_TXRX_BD;
	PRINTM(MINFO, "TX ring: allocating %d bytes\n",
	       pmadapter->txbd_ring_size);

	ret = pcb->moal_malloc_consistent(pmadapter->pmoal_handle,
					  pmadapter->txbd_ring_size,
					  &pmadapter->txbd_ring_vbase,
					  &pmadapter->txbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MINFO, "TX ring: - base: %p, pbase: %#x:%x,"
	       "len: %x\n", pmadapter->txbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->txbd_ring_pbase >> 32),
	       (t_u32)pmadapter->txbd_ring_pbase, pmadapter->txbd_ring_size);

	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		pmadapter->txbd_ring[i] = (mlan_pcie_data_buf *)
			(pmadapter->txbd_ring_vbase +
			 (sizeof(mlan_pcie_data_buf) * i));
		pmadapter->tx_buf_list[i] = MNULL;
		pmadapter->txbd_ring[i]->paddr = 0;
		pmadapter->txbd_ring[i]->len = 0;
		pmadapter->txbd_ring[i]->flags = 0;
		pmadapter->txbd_ring[i]->frag_len = 0;
		pmadapter->txbd_ring[i]->offset = 0;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function frees TX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_delete_txbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;

	ENTER();

	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		if (pmadapter->tx_buf_list[i]) {
			pmbuf = pmadapter->tx_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa,
					       MLAN_RX_DATA_BUF_SIZE,
					       PCI_DMA_TODEVICE);
			wlan_write_data_complete(pmadapter, pmbuf,
						 MLAN_STATUS_FAILURE);
		}
		pmadapter->tx_buf_list[i] = MNULL;

		if (pmadapter->txbd_ring[i]) {
			pmadapter->txbd_ring[i]->paddr = 0;
			pmadapter->txbd_ring[i]->len = 0;
			pmadapter->txbd_ring[i]->flags = 0;
			pmadapter->txbd_ring[i]->frag_len = 0;
			pmadapter->txbd_ring[i]->offset = 0;
		}
		pmadapter->txbd_ring[i] = MNULL;
	}

	if (pmadapter->txbd_ring_vbase) {
		pmadapter->callbacks.moal_mfree_consistent(pmadapter->
							   pmoal_handle,
							   pmadapter->
							   txbd_ring_size,
							   pmadapter->
							   txbd_ring_vbase,
							   pmadapter->
							   txbd_ring_pbase);
	}
	pmadapter->txbd_pending = 0;
	pmadapter->txbd_ring_size = 0;
	pmadapter->txbd_wrptr = 0;
	pmadapter->txbd_rdptr = 0;
	pmadapter->txbd_ring_vbase = MNULL;
	pmadapter->txbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function creates buffer descriptor ring for RX
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_create_rxbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
	t_u32 i;

	ENTER();
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer. The read pointer starts at 0 (zero) while the write pointer
	 * starts at zero with rollover bit set
	 */
	pmadapter->rxbd_rdptr = 0;
	pmadapter->rxbd_wrptr = TXRX_RW_PTR_ROLLOVER_IND;
	/* allocate shared memory for the BD ring and divide the same in to
	   several descriptors */
	pmadapter->rxbd_ring_size =
		sizeof(mlan_pcie_data_buf) * MLAN_MAX_TXRX_BD;
	PRINTM(MINFO, "RX ring: allocating %d bytes\n",
	       pmadapter->rxbd_ring_size);

	ret = pcb->moal_malloc_consistent(pmadapter->pmoal_handle,
					  pmadapter->rxbd_ring_size,
					  &pmadapter->rxbd_ring_vbase,
					  &pmadapter->rxbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MINFO, "RX ring: - base: %p, pbase: %#x:%x,"
	       "len: %#x\n", pmadapter->rxbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->rxbd_ring_pbase >> 32),
	       (t_u32)pmadapter->rxbd_ring_pbase, pmadapter->rxbd_ring_size);

	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		pmadapter->rxbd_ring[i] = (mlan_pcie_data_buf *)
			(pmadapter->rxbd_ring_vbase +
			 (sizeof(mlan_pcie_data_buf) * i));

		/* Allocate buffer here so that firmware can DMA data on it */
		pmbuf = wlan_alloc_mlan_buffer(pmadapter, MLAN_RX_DATA_BUF_SIZE,
					       MLAN_RX_HEADER_LEN,
					       MOAL_ALLOC_MLAN_BUFFER);
		if (!pmbuf) {
			PRINTM(MERROR,
			       "RX ring create : Unable to allocate mlan_buffer\n");
			wlan_pcie_delete_rxbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->rx_buf_list[i] = pmbuf;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MLAN_RX_DATA_BUF_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "Rx ring create : moal_map_memory failed\n");
			wlan_pcie_delete_rxbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		PRINTM(MINFO, "RX ring: add new mlan_buffer base: %p, "
		       "buf_base: %p, buf_pbase: %#x:%x, "
		       "buf_len: %#x\n", pmbuf, pmbuf->pbuf,
		       (t_u32)((t_u64)pmbuf->buf_pa >> 32),
		       (t_u32)pmbuf->buf_pa, pmbuf->data_len);

		pmadapter->rxbd_ring[i]->paddr = pmbuf->buf_pa;
		pmadapter->rxbd_ring[i]->len = (t_u16)pmbuf->data_len;
		pmadapter->rxbd_ring[i]->flags =
			MLAN_BD_FLAG_SOP | MLAN_BD_FLAG_EOP;
		pmadapter->rxbd_ring[i]->offset = 0;
		pmadapter->rxbd_ring[i]->frag_len = (t_u16)pmbuf->data_len;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees RX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_delete_rxbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;

	ENTER();
	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		if (pmadapter->rx_buf_list[i]) {
			pmbuf = pmadapter->rx_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa,
					       MLAN_RX_DATA_BUF_SIZE,
					       PCI_DMA_FROMDEVICE);
			wlan_free_mlan_buffer(pmadapter,
					      pmadapter->rx_buf_list[i]);
		}
		pmadapter->rx_buf_list[i] = MNULL;

		if (pmadapter->rxbd_ring[i]) {
			pmadapter->rxbd_ring[i]->paddr = 0;
			pmadapter->rxbd_ring[i]->offset = 0;
			pmadapter->rxbd_ring[i]->frag_len = 0;
		}
		pmadapter->rxbd_ring[i] = MNULL;
	}

	if (pmadapter->rxbd_ring_vbase)
		pmadapter->callbacks.moal_mfree_consistent(pmadapter->
							   pmoal_handle,
							   pmadapter->
							   rxbd_ring_size,
							   pmadapter->
							   rxbd_ring_vbase,
							   pmadapter->
							   rxbd_ring_pbase);
	pmadapter->rxbd_ring_size = 0;
	pmadapter->rxbd_rdptr = 0;
	pmadapter->rxbd_wrptr = 0;
	pmadapter->rxbd_ring_vbase = MNULL;
	pmadapter->rxbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function creates buffer descriptor ring for Events
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_create_evtbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;
	t_u32 i;

	ENTER();
	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer. The read pointer starts at 0 (zero) while the write pointer
	 * starts at zero with rollover bit set
	 */
	pmadapter->evtbd_rdptr = 0;
	pmadapter->evtbd_wrptr = EVT_RW_PTR_ROLLOVER_IND;

	pmadapter->evtbd_ring_size =
		sizeof(mlan_pcie_evt_buf) * MLAN_MAX_EVT_BD;
	PRINTM(MINFO, "Evt ring: allocating %d bytes\n",
	       pmadapter->evtbd_ring_size);

	ret = pcb->moal_malloc_consistent(pmadapter->pmoal_handle,
					  pmadapter->evtbd_ring_size,
					  &pmadapter->evtbd_ring_vbase,
					  &pmadapter->evtbd_ring_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	PRINTM(MINFO, "Evt ring: - base: %p, pbase: %#x:%x,"
	       "len: %#x\n", pmadapter->evtbd_ring_vbase,
	       (t_u32)((t_u64)pmadapter->evtbd_ring_pbase >> 32),
	       (t_u32)pmadapter->evtbd_ring_pbase, pmadapter->evtbd_ring_size);

	for (i = 0; i < MLAN_MAX_EVT_BD; i++) {
		pmadapter->evtbd_ring[i] = (mlan_pcie_evt_buf *)
			(pmadapter->evtbd_ring_vbase +
			 (sizeof(mlan_pcie_evt_buf) * i));

		/* Allocate buffer here so that firmware can DMA data on it */
		pmbuf = wlan_alloc_mlan_buffer(pmadapter, MAX_EVENT_SIZE,
					       MLAN_RX_HEADER_LEN,
					       MOAL_ALLOC_MLAN_BUFFER);
		if (!pmbuf) {
			PRINTM(MERROR,
			       "Event ring create : Unable to allocate mlan_buffer\n");
			wlan_pcie_delete_evtbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->evt_buf_list[i] = pmbuf;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MAX_EVENT_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "Event ring create : moal_map_memory failed\n");
			wlan_pcie_delete_evtbd_ring(pmadapter);
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->evtbd_ring[i]->paddr = pmbuf->buf_pa;
		pmadapter->evtbd_ring[i]->len = (t_u16)pmbuf->data_len;
		pmadapter->evtbd_ring[i]->flags = 0;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees event buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_delete_evtbd_ring(mlan_adapter *pmadapter)
{
	t_u32 i;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf = MNULL;

	ENTER();
	for (i = 0; i < MLAN_MAX_EVT_BD; i++) {
		if (pmadapter->evt_buf_list[i]) {
			pmbuf = pmadapter->evt_buf_list[i];
			pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					       pmbuf->pbuf + pmbuf->data_offset,
					       pmbuf->buf_pa,
					       MAX_EVENT_SIZE,
					       PCI_DMA_FROMDEVICE);
			wlan_free_mlan_buffer(pmadapter, pmbuf);
		}

		pmadapter->evt_buf_list[i] = MNULL;

		if (pmadapter->evtbd_ring[i]) {
			pmadapter->evtbd_ring[i]->paddr = 0;
			pmadapter->evtbd_ring[i]->len = 0;
			pmadapter->evtbd_ring[i]->flags = 0;
		}
		pmadapter->evtbd_ring[i] = MNULL;
	}

	if (pmadapter->evtbd_ring_vbase)
		pmadapter->callbacks.moal_mfree_consistent(pmadapter->
							   pmoal_handle,
							   pmadapter->
							   evtbd_ring_size,
							   pmadapter->
							   evtbd_ring_vbase,
							   pmadapter->
							   evtbd_ring_pbase);

	pmadapter->evtbd_rdptr = 0;
	pmadapter->evtbd_wrptr = 0;
	pmadapter->evtbd_ring_size = 0;
	pmadapter->evtbd_ring_vbase = MNULL;
	pmadapter->evtbd_ring_pbase = 0;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function allocates buffer for CMD and CMDRSP
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_alloc_cmdrsp_buf(mlan_adapter *pmadapter)
{
	mlan_buffer *pmbuf = MNULL;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	 /** Virtual base address of command response */
	t_u8 *cmdrsp_vbase;
    /** Physical base address of command response */
	t_u64 cmdrsp_pbase = 0;

	ENTER();

	/* Allocate memory for receiving command response data */
	pmbuf = wlan_alloc_mlan_buffer(pmadapter, 0, 0, MOAL_MALLOC_BUFFER);
	if (!pmbuf) {
		PRINTM(MERROR,
		       "Command resp buffer create : Unable to allocate mlan_buffer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	ret = pcb->moal_malloc_consistent(pmadapter->pmoal_handle,
					  MRVDRV_SIZE_OF_CMD_BUFFER,
					  &cmdrsp_vbase, &cmdrsp_pbase);

	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "%s: No free moal_malloc_consistent\n",
		       __FUNCTION__);
		/* free pmbuf */
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	pmbuf->buf_pa = cmdrsp_pbase;
	pmbuf->pbuf = cmdrsp_vbase;
	pmbuf->data_offset = 0;
	pmbuf->data_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	pmbuf->total_pcie_buf_len = MRVDRV_SIZE_OF_CMD_BUFFER;
	pmadapter->cmdrsp_buf = pmbuf;
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function frees CMD and CMDRSP buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_delete_cmdrsp_buf(mlan_adapter *pmadapter)
{
	mlan_buffer *pmbuf = MNULL;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u8 *cmdrsp_vbase;
	t_u64 cmdrsp_pbase;
	ENTER();

	if (!pmadapter) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (pmadapter->cmdrsp_buf) {
		pmbuf = pmadapter->cmdrsp_buf;
		cmdrsp_vbase = pmbuf->pbuf;
		cmdrsp_pbase = pmbuf->buf_pa;
		if (cmdrsp_vbase)
			pmadapter->callbacks.moal_mfree_consistent(pmadapter->
								   pmoal_handle,
								   pmbuf->
								   total_pcie_buf_len,
								   cmdrsp_vbase,
								   cmdrsp_pbase);
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		pmadapter->cmdrsp_buf = MNULL;
	}

	if (pmadapter->cmd_buf) {
		pmbuf = pmadapter->cmd_buf;
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pmbuf->pbuf + pmbuf->data_offset,
				       pmbuf->buf_pa,
				       MRVDRV_SIZE_OF_CMD_BUFFER,
				       PCI_DMA_TODEVICE);
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

#define PCIE_TXBD_EMPTY(wrptr, rdptr)                       \
	(((wrptr & TXRX_RW_PTR_MASK) == (rdptr & TXRX_RW_PTR_MASK)) \
	 && ((wrptr & TXRX_RW_PTR_ROLLOVER_IND) ==          \
	     (rdptr & TXRX_RW_PTR_ROLLOVER_IND)))

/**
 *  @brief This function flushes the TX buffer descriptor ring
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_flush_txbd_ring(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (!PCIE_TXBD_EMPTY(pmadapter->txbd_wrptr, pmadapter->txbd_rdptr)) {
		pmadapter->txbd_flush = MTRUE;
		/* write pointer already set at last send */
		/* send dnld-rdy intr again, wait for completion */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, PCIE_CPU_INT_EVENT,
				   CPU_INTR_DNLD_RDY)) {
			PRINTM(MERROR,
			       "SEND DATA (FLUSH): failed to assert dnld-rdy interrupt.\n");
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function unmaps and frees downloaded data buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_send_data_complete(mlan_adapter *pmadapter)
{
	const t_u32 num_tx_buffs = MLAN_MAX_TXRX_BD;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	mlan_buffer *pmbuf;
	t_u32 wrdoneidx;
	t_u32 rdptr;
	t_u32 unmap_count = 0;
	t_u32 rdptr_start = TXBD_RW_PTR_START;

	ENTER();

	/* Read the TX ring read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle, REG_TXBD_RDPTR, &rdptr)) {
		PRINTM(MERROR,
		       "SEND DATA COMP: failed to read REG_TXBD_RDPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	PRINTM(MINFO, "SEND DATA COMP:  rdptr_prev=0x%x, rdptr=0x%x\n",
	       pmadapter->txbd_rdptr, rdptr);

	rdptr = rdptr >> rdptr_start;

	/* free from previous txbd_rdptr to current txbd_rdptr */
	while (((pmadapter->txbd_rdptr & TXRX_RW_PTR_MASK)
		!= (rdptr & TXRX_RW_PTR_MASK))
	       || ((pmadapter->txbd_rdptr & TXRX_RW_PTR_ROLLOVER_IND)
		   != (rdptr & TXRX_RW_PTR_ROLLOVER_IND))) {
		wrdoneidx = pmadapter->txbd_rdptr & (num_tx_buffs - 1);
		pmbuf = pmadapter->tx_buf_list[wrdoneidx];
		if (pmbuf) {
			PRINTM(MDAT_D,
			       "SEND DATA COMP: Detach pmbuf %p at tx_ring[%d], pmadapter->txbd_rdptr=0x%x\n",
			       pmbuf, wrdoneidx, pmadapter->txbd_rdptr);
			ret = pcb->moal_unmap_memory(pmadapter->pmoal_handle,
						     pmbuf->pbuf +
						     pmbuf->data_offset,
						     pmbuf->buf_pa,
						     pmbuf->data_len,
						     PCI_DMA_TODEVICE);
			if (ret == MLAN_STATUS_FAILURE) {
				PRINTM(MERROR, "%s: moal_unmap_memory failed\n",
				       __FUNCTION__);
				break;
			}
			unmap_count++;
			pmadapter->txbd_pending--;
			if (pmadapter->txbd_flush)
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_FAILURE);
			else
				wlan_write_data_complete(pmadapter, pmbuf,
							 MLAN_STATUS_SUCCESS);
		}

		pmadapter->tx_buf_list[wrdoneidx] = MNULL;
		pmadapter->txbd_ring[wrdoneidx]->paddr = 0;
		pmadapter->txbd_ring[wrdoneidx]->len = 0;
		pmadapter->txbd_ring[wrdoneidx]->flags = 0;
		pmadapter->txbd_ring[wrdoneidx]->frag_len = 0;
		pmadapter->txbd_ring[wrdoneidx]->offset = 0;
		pmadapter->txbd_rdptr++;
		if ((pmadapter->txbd_rdptr & TXRX_RW_PTR_MASK) == num_tx_buffs)
			pmadapter->txbd_rdptr = ((pmadapter->txbd_rdptr &
						  TXRX_RW_PTR_ROLLOVER_IND) ^
						 TXRX_RW_PTR_ROLLOVER_IND);
	}

	if (unmap_count)
		pmadapter->data_sent = MFALSE;
	if (pmadapter->txbd_flush) {
		if (PCIE_TXBD_EMPTY
		    (pmadapter->txbd_wrptr, pmadapter->txbd_rdptr))
			pmadapter->txbd_flush = MFALSE;
		else
			wlan_pcie_flush_txbd_ring(pmadapter);
	}
done:
	LEAVE();
	return ret;
}

#define PCIE_TXBD_NOT_FULL(wrptr, rdptr)                    \
	(((wrptr & TXRX_RW_PTR_MASK) != (rdptr & TXRX_RW_PTR_MASK)) \
	 || ((wrptr & TXRX_RW_PTR_ROLLOVER_IND) ==          \
	     (rdptr & TXRX_RW_PTR_ROLLOVER_IND)))

/**
 *  @brief This function downloads data to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param type      packet type
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include PCIE header)
 *  @param tx_param  A pointer to mlan_tx_param
 *
 *  @return          MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_send_data(mlan_adapter *pmadapter, t_u8 type,
		    mlan_buffer *pmbuf, mlan_tx_param *tx_param)
{
	const t_u32 num_tx_buffs = MLAN_MAX_TXRX_BD;
	mlan_status ret = MLAN_STATUS_PENDING;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 rxbd_val = 0;
	t_u32 wrindx;
	t_u16 *tmp;
	t_u8 *payload;
	t_u32 wr_ptr_start = TXBD_RW_PTR_START;

	ENTER();

	if (!(pmadapter && pmbuf)) {
		PRINTM(MERROR, "%s() has no buffer", __FUNCTION__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!(pmbuf->pbuf && pmbuf->data_len)) {
		PRINTM(MERROR, "Invalid parameter <%p, %#x>\n",
		       pmbuf->pbuf, pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	PRINTM(MINFO, "SEND DATA: <Rd: %#x, Wr: %#x>\n",
	       pmadapter->txbd_rdptr, pmadapter->txbd_wrptr);
	if (PCIE_TXBD_NOT_FULL(pmadapter->txbd_wrptr, pmadapter->txbd_rdptr)) {
		pmadapter->data_sent = MTRUE;

		payload = pmbuf->pbuf + pmbuf->data_offset;
		tmp = (t_u16 *)&payload[0];
		*tmp = wlan_cpu_to_le16((t_u16)pmbuf->data_len);
		tmp = (t_u16 *)&payload[2];
		*tmp = wlan_cpu_to_le16(type);

		/* Map pmbuf, and attach to tx ring */
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, pmbuf->data_len,
					 PCI_DMA_TODEVICE)) {
			PRINTM(MERROR,
			       "SEND DATA: failed to moal_map_memory\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		wrindx = pmadapter->txbd_wrptr & (num_tx_buffs - 1);
		PRINTM(MDAT_D,
		       "SEND DATA: Attach pmbuf %p at tx_ring[%d], txbd_wrptr=0x%x\n",
		       pmbuf, wrindx, pmadapter->txbd_wrptr);
		pmadapter->tx_buf_list[wrindx] = pmbuf;
		pmadapter->txbd_ring[wrindx]->paddr = pmbuf->buf_pa;
		pmadapter->txbd_ring[wrindx]->len = (t_u16)pmbuf->data_len;
		pmadapter->txbd_ring[wrindx]->flags = MLAN_BD_FLAG_SOP |
			MLAN_BD_FLAG_EOP;
		pmadapter->txbd_ring[wrindx]->frag_len = (t_u16)pmbuf->data_len;
		pmadapter->txbd_ring[wrindx]->offset = 0;

		pmadapter->txbd_pending++;
		pmadapter->txbd_wrptr++;
		if ((pmadapter->txbd_wrptr & TXRX_RW_PTR_MASK) == num_tx_buffs)
			pmadapter->txbd_wrptr = ((pmadapter->txbd_wrptr &
						  TXRX_RW_PTR_ROLLOVER_IND) ^
						 TXRX_RW_PTR_ROLLOVER_IND);
		rxbd_val = pmadapter->rxbd_wrptr & TXRX_RW_PTR_WRAP_MASK;
		PRINTM(MINFO, "REG_TXBD_WRPT(0x%x) = 0x%x\n",
		       REG_TXBD_WRPTR,
		       ((pmadapter->txbd_wrptr << wr_ptr_start) | rxbd_val));
		/* Write the TX ring write pointer in to REG_TXBD_WRPTR */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_TXBD_WRPTR,
					(pmadapter->
					 txbd_wrptr << wr_ptr_start) |
					rxbd_val)) {
			PRINTM(MERROR,
			       "SEND DATA: failed to write REG_TXBD_WRPTR\n");
			ret = MLAN_STATUS_FAILURE;
			goto done_unmap;
		}
		PRINTM(MINFO, "SEND DATA: Updated <Rd: %#x, Wr: %#x>\n",
		       pmadapter->txbd_rdptr, pmadapter->txbd_wrptr);

		if (PCIE_TXBD_NOT_FULL
		    (pmadapter->txbd_wrptr, pmadapter->txbd_rdptr))
			pmadapter->data_sent = MFALSE;

		PRINTM(MINFO, "Sent packet to firmware successfully\n");
	} else {
		PRINTM(MERROR,
		       "TX Ring full, can't send anymore packets to firmware\n");
		PRINTM(MINFO, "SEND DATA (FULL!): <Rd: %#x, Wr: %#x>\n",
		       pmadapter->txbd_rdptr, pmadapter->txbd_wrptr);
		pmadapter->data_sent = MTRUE;
		/* Send the TX ready interrupt */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, PCIE_CPU_INT_EVENT,
				   CPU_INTR_DNLD_RDY))
			PRINTM(MERROR,
			       "SEND DATA (FULL): failed to assert dnld-rdy interrupt\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	LEAVE();
	return ret;
done_unmap:
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				   pmbuf->pbuf + pmbuf->data_offset,
				   pmbuf->buf_pa, pmbuf->data_len,
				   PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "SEND DATA: failed to moal_unmap_memory\n");
		ret = MLAN_STATUS_FAILURE;
	}
	pmadapter->txbd_pending--;
	pmadapter->tx_buf_list[wrindx] = MNULL;
	pmadapter->txbd_ring[wrindx]->paddr = 0;
	pmadapter->txbd_ring[wrindx]->len = 0;
	pmadapter->txbd_ring[wrindx]->flags = 0;
	pmadapter->txbd_ring[wrindx]->frag_len = 0;
	pmadapter->txbd_ring[wrindx]->offset = 0;
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function handles received buffer ring and
 *  dispatches packets to upper
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_process_recv_data(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 rdptr, rd_index;
	mlan_buffer *pmbuf = MNULL;
	t_u32 txbd_val = 0;
	t_u16 rx_len, rx_type;
	const t_u32 num_rx_buffs = MLAN_MAX_TXRX_BD;

	ENTER();

	/* Read the RX ring Read pointer set by firmware */
	if (pcb->moal_read_reg(pmadapter->pmoal_handle, REG_RXBD_RDPTR, &rdptr)) {
		PRINTM(MERROR, "RECV DATA: failed to read REG_RXBD_RDPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	while (((rdptr & TXRX_RW_PTR_MASK) !=
		(pmadapter->rxbd_rdptr & TXRX_RW_PTR_MASK))
	       || ((rdptr & TXRX_RW_PTR_ROLLOVER_IND) !=
		   (pmadapter->rxbd_rdptr & TXRX_RW_PTR_ROLLOVER_IND))) {
		/* detach pmbuf (with data) from Rx Ring */
		rd_index = pmadapter->rxbd_rdptr & (num_rx_buffs - 1);
		if (rd_index > MLAN_MAX_TXRX_BD - 1) {
			PRINTM(MERROR, "RECV DATA: Invalid Rx buffer index.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		pmbuf = pmadapter->rx_buf_list[rd_index];

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					   pmbuf->pbuf + pmbuf->data_offset,
					   pmbuf->buf_pa, MLAN_RX_DATA_BUF_SIZE,
					   PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR,
			       "RECV DATA: moal_unmap_memory failed.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		pmadapter->rx_buf_list[rd_index] = MNULL;
		PRINTM(MDAT_D,
		       "RECV DATA: Detach pmbuf %p at rx_ring[%d], pmadapter->rxbd_rdptr=0x%x\n",
		       pmbuf, rd_index, pmadapter->rxbd_rdptr);

		/* Get data length from interface header -
		   first 2 bytes are len, second 2 bytes are type */
		rx_len = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset));
		rx_len = wlan_le16_to_cpu(rx_len);
		rx_type = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset + 2));
		rx_type = wlan_le16_to_cpu(rx_type);

		PRINTM(MINFO,
		       "RECV DATA: <Wr: %#x, Rd: %#x>, Len=%d rx_type=%d\n",
		       pmadapter->rxbd_wrptr, rdptr, rx_len, rx_type);

		if (rx_len <= MLAN_RX_DATA_BUF_SIZE) {
			/* send buffer to host (which will free it) */
			pmbuf->data_len = rx_len - INTF_HEADER_LEN;
			pmbuf->data_offset += INTF_HEADER_LEN;
			PRINTM(MINFO,
			       "RECV DATA: Received packet from FW successfully\n");
			pmadapter->callbacks.moal_spin_lock(pmadapter->
							    pmoal_handle,
							    pmadapter->
							    rx_data_queue.
							    plock);
			util_enqueue_list_tail(pmadapter->pmoal_handle,
					       &pmadapter->rx_data_queue,
					       (pmlan_linked_list)pmbuf, MNULL,
					       MNULL);
			pmadapter->rx_pkts_queued++;
			pmadapter->callbacks.moal_spin_unlock(pmadapter->
							      pmoal_handle,
							      pmadapter->
							      rx_data_queue.
							      plock);

			pmadapter->data_received = MTRUE;
			/* Create new buffer and attach it to Rx Ring */
			pmbuf = wlan_alloc_mlan_buffer(pmadapter,
						       MLAN_RX_DATA_BUF_SIZE,
						       MLAN_RX_HEADER_LEN,
						       MOAL_ALLOC_MLAN_BUFFER);
			if (!pmbuf) {
				PRINTM(MERROR,
				       "RECV DATA: Unable to allocate mlan_buffer\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
		} else {
			/* Queue the mlan_buffer again */
			PRINTM(MERROR, "PCIE: Drop invalid packet, length=%d",
			       rx_len);
		}

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MLAN_RX_DATA_BUF_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR, "RECV DATA: moal_map_memory failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		PRINTM(MDAT_D,
		       "RECV DATA: Attach new pmbuf %p at rx_ring[%d]\n", pmbuf,
		       rd_index);
		pmadapter->rx_buf_list[rd_index] = pmbuf;
		pmadapter->rxbd_ring[rd_index]->paddr = pmbuf->buf_pa;
		pmadapter->rxbd_ring[rd_index]->len = (t_u16)pmbuf->data_len;
		pmadapter->rxbd_ring[rd_index]->flags =
			MLAN_BD_FLAG_SOP | MLAN_BD_FLAG_EOP;
		pmadapter->rxbd_ring[rd_index]->offset = 0;
		pmadapter->rxbd_ring[rd_index]->frag_len =
			(t_u16)pmbuf->data_len;

		pmadapter->rxbd_rdptr++;
		pmadapter->rxbd_wrptr++;
		/* update rxbd rdptrs */
		if ((pmadapter->rxbd_rdptr & TXRX_RW_PTR_MASK) ==
		    MLAN_MAX_TXRX_BD) {
			pmadapter->rxbd_rdptr =
				((pmadapter->
				  rxbd_rdptr & TXRX_RW_PTR_ROLLOVER_IND) ^
				 TXRX_RW_PTR_ROLLOVER_IND);
		}
		/* update rxbd's wrptrs */
		if ((pmadapter->rxbd_wrptr & TXRX_RW_PTR_MASK) ==
		    MLAN_MAX_TXRX_BD) {
			pmadapter->rxbd_wrptr =
				((pmadapter->
				  rxbd_wrptr & TXRX_RW_PTR_ROLLOVER_IND) ^
				 TXRX_RW_PTR_ROLLOVER_IND);
		}
		PRINTM(MINFO, "RECV DATA: Updated <Wr: %#x, Rd: %#x, %#x>\n",
		       pmadapter->rxbd_wrptr, pmadapter->rxbd_rdptr, rdptr);
		txbd_val = pmadapter->txbd_wrptr & TXRX_RW_PTR_WRAP_MASK;
		txbd_val = txbd_val << TXBD_RW_PTR_START;
		/* Write the RX ring write pointer in to REG_RXBD_WRPTR */
		if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_RXBD_WRPTR,
					pmadapter->rxbd_wrptr | txbd_val)) {
			PRINTM(MERROR,
			       "RECV DATA: failed to write REG_RXBD_WRPTR\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		/* Read the RX ring read pointer set by firmware */
		if (pcb->
		    moal_read_reg(pmadapter->pmoal_handle, REG_RXBD_RDPTR,
				  &rdptr)) {
			PRINTM(MERROR,
			       "RECV DATA: failed to read REG_RXBD_RDPTR\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function downloads command to the card.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include PCIE header)
 *
 *  @return 	     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_send_cmd(mlan_adapter *pmadapter, mlan_buffer *pmbuf)
{
	mlan_status ret = MLAN_STATUS_PENDING;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u8 *payload = MNULL;

	ENTER();
	if (!(pmadapter && pmbuf)) {
		PRINTM(MERROR, "%s() has no buffer", __FUNCTION__);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!(pmbuf->pbuf && pmbuf->data_len)) {
		PRINTM(MERROR, "Invalid parameter <%p, %#x>\n",
		       pmbuf->pbuf, pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Make sure a command response buffer is available */
	if (!pmadapter->cmdrsp_buf) {
		PRINTM(MERROR,
		       "No response buffer available, send command failed\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	pmadapter->cmd_sent = MTRUE;
	payload = pmbuf->pbuf + pmbuf->data_offset;
	*(t_u16 *)&payload[0] = wlan_cpu_to_le16((t_u16)pmbuf->data_len);
	*(t_u16 *)&payload[2] = wlan_cpu_to_le16(MLAN_TYPE_CMD);

	if (MLAN_STATUS_FAILURE == pcb->moal_map_memory(pmadapter->pmoal_handle,
							pmbuf->pbuf +
							pmbuf->data_offset,
							&pmbuf->buf_pa,
							MLAN_RX_CMD_BUF_SIZE,
							PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "Command buffer : moal_map_memory failed\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pmadapter->cmd_buf = pmbuf;

	/* To send a command, the driver will:
	   1. Write the 64bit physical address of the data buffer to
	   SCRATCH1 + SCRATCH0
	   2. Ring the door bell (i.e. set the door bell interrupt)

	   In response to door bell interrupt, the firmware will perform
	   the DMA of the command packet (first header to obtain the total
	   length and then rest of the command).
	 */
	if (pmadapter->cmdrsp_buf) {
		/* Write the lower 32bits of the cmdrsp buffer physical
		   address */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, REG_CMDRSP_ADDR_LO,
				   (t_u32)pmadapter->cmdrsp_buf->buf_pa)) {
			PRINTM(MERROR,
			       "Failed to write download command to boot code.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		/* Write the upper 32bits of the cmdrsp buffer physical
		   address */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, REG_CMDRSP_ADDR_HI,
				   (t_u32)((t_u64)pmadapter->cmdrsp_buf->
					   buf_pa >> 32))) {
			PRINTM(MERROR,
			       "Failed to write download command to boot code.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

	/* Write the lower 32bits of the physical address to REG_CMD_ADDR_LO */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_CMD_ADDR_LO,
				(t_u32)pmadapter->cmd_buf->buf_pa)) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Write the upper 32bits of the physical address to REG_CMD_ADDR_HI */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_CMD_ADDR_HI,
				(t_u32)((t_u64)pmadapter->cmd_buf->
					buf_pa >> 32))) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the command length to REG_CMD_SIZE */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_CMD_SIZE,
				pmadapter->cmd_buf->data_len)) {
		PRINTM(MERROR,
		       "Failed to write command length to REG_CMD_SIZE\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Ring the door bell */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE_CPU_INT_EVENT,
				CPU_INTR_DOOR_BELL)) {
		PRINTM(MERROR, "Failed to assert door-bell interrupt.\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
done:
	if ((ret == MLAN_STATUS_FAILURE) && pmadapter)
		pmadapter->cmd_sent = MFALSE;

	LEAVE();
	return ret;
}

#define MLAN_SLEEP_COOKIE_DEF 0xBEEFBEEF
#define MAX_DELAY_LOOP_COUNT 100

/**
 *  @brief This function maintains delay for sleep cookie
 *
 *  @param pmadapter                A pointer to mlan_adapter structure
 *  @param max_delay_loop_count     Maximum number of delay loop count
 *
 *  @return          		    N/A
 */
static void
mlan_delay_for_sleep_cookie(mlan_adapter *pmadapter, t_u32 max_delay_loop_cnt)
{
	t_u8 *buffer;
	t_u32 sleep_cookie = 0;
	t_u32 count = 0;
	pmlan_buffer pmbuf = pmadapter->cmdrsp_buf;

	for (count = 0; count < max_delay_loop_cnt; count++) {

		buffer = pmbuf->pbuf;
		sleep_cookie = *(t_u32 *)buffer;

		if (sleep_cookie == MLAN_SLEEP_COOKIE_DEF) {
			PRINTM(MINFO, "sleep cookie FOUND at count = %d!!\n",
			       count);
			break;
		}
		wlan_udelay(pmadapter, 20);
	}

	if (count >= max_delay_loop_cnt)
		PRINTM(MERROR, "sleep cookie not found!!\n");
}

/**
 *  @brief This function handles command complete interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_process_cmd_resp(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	pmlan_buffer pmbuf = pmadapter->cmdrsp_buf;
	t_u16 resp_len = 0;

	ENTER();

	PRINTM(MINFO, "Rx CMD Response\n");

	if (pmbuf == MNULL) {
		PRINTM(MMSG, "Rx CMD response pmbuf is null\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Get data length from interface header -
	   first 2 bytes are len, second 2 bytes are type */
	resp_len = *((t_u16 *)(pmbuf->pbuf + pmbuf->data_offset));

	pmadapter->upld_len = wlan_le16_to_cpu(resp_len);
	pmadapter->upld_len -= INTF_HEADER_LEN;

	if (!pmadapter->curr_cmd) {
		if (pmadapter->ps_state == PS_STATE_SLEEP_CFM) {
			wlan_process_sleep_confirm_resp(pmadapter, pmbuf->pbuf +
							pmbuf->data_offset +
							INTF_HEADER_LEN,
							pmadapter->upld_len);
			/* We are sending sleep confirm done interrupt in the middle of
			 * sleep handshake. There is a corner case when Tx done interrupt
			 * is received from firmware during sleep handshake due to which
			 * host and firmware power states go out of sync causing Tx data
			 * timeout problem.
			 * Hence sleep confirm done interrupt is sent at the end of sleep
			 * handshake to fix the problem
			 *
			 * Host could be reading the interrupt during polling (while loop) or
			 * to address a FW interrupt. In either case, after clearing the interrupt
			 * driver needs to send a sleep confirm event at the end of processing
			 * command response right here. This marks the end of the sleep handshake
			 * with firmware.
			 */
			wlan_pcie_enable_host_int_mask(pmadapter);
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						PCIE_CPU_INT_EVENT,
						CPU_INTR_SLEEP_CFM_DONE)) {
				PRINTM(MERROR, "Write register failed\n");
				LEAVE();
				return MLAN_STATUS_FAILURE;
			}
			mlan_delay_for_sleep_cookie(pmadapter,
						    MAX_DELAY_LOOP_COUNT);

		}
		memcpy(pmadapter, pmadapter->upld_buf, pmbuf->pbuf +
		       pmbuf->data_offset + INTF_HEADER_LEN,
		       MIN(MRVDRV_SIZE_OF_CMD_BUFFER, pmadapter->upld_len));

	} else {
		pmadapter->cmd_resp_received = MTRUE;
		pmbuf->data_len = pmadapter->upld_len;
		pmbuf->data_offset += INTF_HEADER_LEN;
		pmadapter->curr_cmd->respbuf = pmbuf;

		/* Take the pointer and set it to CMD node and will
		   return in the response complete callback */
		pmadapter->cmdrsp_buf = MNULL;
		/* Clear the cmd-rsp buffer address in scratch registers. This
		   will prevent firmware from writing to the same response
		   buffer again. */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, REG_CMDRSP_ADDR_LO,
				   0)) {
			PRINTM(MERROR,
			       "Rx CMD: failed to clear cmd_rsp address.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		/* Write the upper 32bits of the cmdrsp buffer physical
		   address */
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, REG_CMDRSP_ADDR_HI,
				   0)) {
			PRINTM(MERROR,
			       "Rx CMD: failed to clear cmd_rsp address.\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
	}

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function handles command response completion
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_cmdrsp_complete(mlan_adapter *pmadapter, mlan_buffer *pmbuf)
{
	mlan_buffer *pcmdmbuf;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	/*return the cmd response pmbuf */
	if (pmbuf) {
		pmbuf->data_len = MRVDRV_SIZE_OF_CMD_BUFFER;
		pmbuf->data_offset -= INTF_HEADER_LEN;
		pmadapter->cmdrsp_buf = pmbuf;
	}

	/*unmap the cmd pmbuf, so the cpu can access the memory in the command node */
	pcmdmbuf = pmadapter->cmd_buf;

	if (pcmdmbuf) {
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pcmdmbuf->pbuf + pcmdmbuf->data_offset,
				       pcmdmbuf->buf_pa,
				       WLAN_UPLD_SIZE, PCI_DMA_TODEVICE);
		pmadapter->cmd_buf = MNULL;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles FW event ready interrupt
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_process_event_ready(mlan_adapter *pmadapter)
{
	t_u32 rd_index = pmadapter->evtbd_rdptr & (MLAN_MAX_EVT_BD - 1);
	t_u32 rdptr, event;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (pmadapter->event_received) {
		PRINTM(MINFO, "Event being processed, do not "
		       "process this interrupt just yet\n");
		LEAVE();
		return MLAN_STATUS_SUCCESS;
	}

	if (rd_index >= MLAN_MAX_EVT_BD) {
		PRINTM(MINFO, "Invalid rd_index...\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	/* Read the event ring read pointer set by firmware */
	if (pcb->
	    moal_read_reg(pmadapter->pmoal_handle, REG_EVTBD_RDPTR, &rdptr)) {
		PRINTM(MERROR, "EvtRdy: failed to read REG_EVTBD_RDPTR\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	PRINTM(MINFO, "EvtRdy: Initial <Wr: 0x%x, Rd: 0x%x>\n",
	       pmadapter->evtbd_wrptr, rdptr);
	if (((rdptr & EVT_RW_PTR_MASK) !=
	     (pmadapter->evtbd_rdptr & EVT_RW_PTR_MASK)) ||
	    ((rdptr & EVT_RW_PTR_ROLLOVER_IND) !=
	     (pmadapter->evtbd_rdptr & EVT_RW_PTR_ROLLOVER_IND))) {
		mlan_buffer *pmbuf_evt;
		t_u16 evt_len;

		PRINTM(MINFO, "EvtRdy: Read Index: %d\n", rd_index);
		pmbuf_evt = pmadapter->evt_buf_list[rd_index];

		/*unmap the pmbuf for CPU Access */
		pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				       pmbuf_evt->pbuf + pmbuf_evt->data_offset,
				       pmbuf_evt->buf_pa,
				       MAX_EVENT_SIZE, PCI_DMA_FROMDEVICE);

		/* Take the pointer and set it to event pointer in adapter
		   and will return back after event handling callback */
		pmadapter->evt_buf_list[rd_index] = MNULL;
		pmadapter->evtbd_ring[rd_index]->paddr = 0;
		pmadapter->evtbd_ring[rd_index]->len = 0;
		pmadapter->evtbd_ring[rd_index]->flags = 0;

		event = *((t_u32 *)&pmbuf_evt->
			  pbuf[pmbuf_evt->data_offset + INTF_HEADER_LEN]);
		pmadapter->event_cause = wlan_le32_to_cpu(event);
		/* The first 4bytes will be the event transfer header
		   len is 2 bytes followed by type which is 2 bytes */
		evt_len = *((t_u16 *)&pmbuf_evt->pbuf[pmbuf_evt->data_offset]);
		evt_len = wlan_le16_to_cpu(evt_len);

		if ((evt_len > 0) && (evt_len > MLAN_EVENT_HEADER_LEN) &&
		    (evt_len - MLAN_EVENT_HEADER_LEN < MAX_EVENT_SIZE))
			memcpy(pmadapter, pmadapter->event_body,
			       pmbuf_evt->pbuf + pmbuf_evt->data_offset +
			       MLAN_EVENT_HEADER_LEN,
			       evt_len - MLAN_EVENT_HEADER_LEN);

		pmbuf_evt->data_offset += INTF_HEADER_LEN;
		pmbuf_evt->data_len = evt_len - INTF_HEADER_LEN;
		PRINTM(MINFO, "Event length: %d\n", pmbuf_evt->data_len);

		pmadapter->event_received = MTRUE;
		pmadapter->pmlan_buffer_event = pmbuf_evt;

		pmadapter->evtbd_rdptr++;
		if ((pmadapter->evtbd_rdptr & EVT_RW_PTR_MASK) ==
		    MLAN_MAX_EVT_BD) {
			pmadapter->evtbd_rdptr =
				((pmadapter->
				  evtbd_rdptr & EVT_RW_PTR_ROLLOVER_IND) ^
				 EVT_RW_PTR_ROLLOVER_IND);
		}
		/* Do not update the event write pointer here, wait till the
		   buffer is released. This is just to make things simpler,
		   we need to find a better method of managing these buffers.
		 */
	} else {
		PRINTM(MINTR, "------>EVENT DONE\n");
		if (pcb->
		    moal_write_reg(pmadapter->pmoal_handle, PCIE_CPU_INT_EVENT,
				   CPU_INTR_EVENT_DONE)) {
			PRINTM(MERROR,
			       "Failed to asset event done interrupt\n");
			return MLAN_STATUS_FAILURE;
		}
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function handles event completion
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_event_complete(mlan_adapter *pmadapter, mlan_buffer *pmbuf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 wrptr = pmadapter->evtbd_wrptr & EVT_RW_PTR_MASK;
	t_u32 rdptr;

	ENTER();
	if (!pmbuf) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (wrptr >= MLAN_MAX_EVT_BD) {
		PRINTM(MERROR, "EvtCom: Invalid wrptr 0x%x\n", wrptr);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Read the event ring read pointer set by firmware */
	if (pcb->
	    moal_read_reg(pmadapter->pmoal_handle, REG_EVTBD_RDPTR, &rdptr)) {
		PRINTM(MERROR, "EvtCom: failed to read REG_EVTBD_RDPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	if (!pmadapter->evt_buf_list[wrptr]) {
		pmbuf->data_len = MAX_EVENT_SIZE;
		pmbuf->data_offset -= INTF_HEADER_LEN;

		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_map_memory(pmadapter->pmoal_handle,
					 pmbuf->pbuf + pmbuf->data_offset,
					 &pmbuf->buf_pa, MAX_EVENT_SIZE,
					 PCI_DMA_FROMDEVICE)) {
			PRINTM(MERROR, "EvtCom: failed to moal_map_memory\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		pmadapter->evt_buf_list[wrptr] = pmbuf;
		pmadapter->evtbd_ring[wrptr]->paddr = pmbuf->buf_pa;
		pmadapter->evtbd_ring[wrptr]->len = (t_u16)pmbuf->data_len;
		pmadapter->evtbd_ring[wrptr]->flags = 0;
		pmbuf = MNULL;
	} else {
		PRINTM(MINFO, "EvtCom: ERROR: Buffer is still valid at "
		       "index %d, <%p, %p>\n", wrptr,
		       pmadapter->evt_buf_list[wrptr], pmbuf);
	}

	pmadapter->evtbd_wrptr++;
	if ((pmadapter->evtbd_wrptr & EVT_RW_PTR_MASK) == MLAN_MAX_EVT_BD) {
		pmadapter->evtbd_wrptr = ((pmadapter->evtbd_wrptr &
					   EVT_RW_PTR_ROLLOVER_IND) ^
					  EVT_RW_PTR_ROLLOVER_IND);
	}
	PRINTM(MINFO, "EvtCom: Updated <Wr: 0x%x, Rd: 0x%x>\n",
	       pmadapter->evtbd_wrptr, rdptr);

	/* Write the event ring write pointer in to REG_EVTBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_EVTBD_WRPTR,
				pmadapter->evtbd_wrptr)) {
		PRINTM(MERROR, "EvtCom: failed to write REG_EVTBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

done:
	/* Free the buffer for failure case */
	if (ret && pmbuf)
		wlan_free_mlan_buffer(pmadapter, pmbuf);

	PRINTM(MINFO, "EvtCom: Check Events Again\n");
	ret = wlan_pcie_process_event_ready(pmadapter);

	LEAVE();
	return ret;
}

/**
 *  @brief This function downloads boot command to the card.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pmbuf        A pointer to mlan_buffer
 *
 *  @return 	        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_pcie_send_boot_cmd(mlan_adapter *pmadapter, mlan_buffer *pmbuf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (!pmadapter || !pmbuf) {
		PRINTM(MERROR, "NULL Pointer\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (MLAN_STATUS_FAILURE == pcb->moal_map_memory(pmadapter->pmoal_handle,
							pmbuf->pbuf +
							pmbuf->data_offset,
							&pmbuf->buf_pa,
							WLAN_UPLD_SIZE,
							PCI_DMA_TODEVICE)) {
		PRINTM(MERROR, "BootCmd: failed to moal_map_memory\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	if (!(pmbuf->pbuf && pmbuf->data_len && pmbuf->buf_pa)) {
		PRINTM(MERROR, "%s: Invalid buffer <%p, %#x:%x, len=%d>\n",
		       __func__, pmbuf->pbuf,
		       (t_u32)((t_u64)pmbuf->buf_pa >> 32),
		       (t_u32)pmbuf->buf_pa, pmbuf->data_len);
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the lower 32bits of the physical address to scratch
	 * register 0 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				PCIE_SCRATCH_0_REG, (t_u32)pmbuf->buf_pa)) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the upper 32bits of the physical address to scratch
	 * register 1 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE_SCRATCH_1_REG,
				(t_u32)((t_u64)pmbuf->buf_pa >> 32))) {
		PRINTM(MERROR,
		       "Failed to write download command to boot code\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Write the command length to scratch register 2 */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle,
				PCIE_SCRATCH_2_REG, pmbuf->data_len)) {
		PRINTM(MERROR,
		       "Failed to write command length to scratch register 2\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	/* Ring the door bell */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, PCIE_CPU_INT_EVENT,
				CPU_INTR_DOOR_BELL)) {
		PRINTM(MERROR, "Failed to assert door-bell interrupt\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}

	LEAVE();
	return ret;

done:
	if (MLAN_STATUS_FAILURE ==
	    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
				   pmbuf->pbuf + pmbuf->data_offset,
				   pmbuf->buf_pa, WLAN_UPLD_SIZE,
				   PCI_DMA_TODEVICE))
		PRINTM(MERROR, "BootCmd: failed to moal_unmap_memory\n");
	LEAVE();
	return ret;
}

/**
 *  @brief  This function init rx port in firmware
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_init_fw(IN pmlan_adapter pmadapter)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 txbd_val = 0;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	txbd_val = pmadapter->txbd_wrptr & TXRX_RW_PTR_WRAP_MASK;
	txbd_val = txbd_val << TXBD_RW_PTR_START;
	/* Write the RX ring write pointer in to REG_RXBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_RXBD_WRPTR,
				pmadapter->rxbd_wrptr | txbd_val)) {
		PRINTM(MERROR, "Init FW: failed to write REG_RXBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
		goto done;
	}
	/* Write the event ring write pointer in to REG_EVTBD_WRPTR */
	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_EVTBD_WRPTR,
				pmadapter->evtbd_wrptr)) {
		PRINTM(MERROR, "Init FW: failed to write REG_EVTBD_WRPTR\n");
		ret = MLAN_STATUS_FAILURE;
	}
done:
	return ret;
}

/**
 *  @brief  This function downloads FW blocks to device
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param fw			A pointer to firmware image
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_prog_fw_w_helper(IN mlan_adapter *pmadapter, IN mlan_fw_image *fw)
{
	mlan_status ret = MLAN_STATUS_FAILURE;
	t_u8 *firmware = fw->pfw_buf;
	t_u32 firmware_len = fw->fw_len;
	t_u32 offset = 0;
	mlan_buffer *pmbuf = MNULL;
	t_u32 txlen, tx_blocks = 0, tries, len;
	t_u32 block_retry_cnt = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();
	if (!pmadapter) {
		PRINTM(MERROR, "adapter structure is not valid\n");
		goto done;
	}

	if (!firmware || !firmware_len) {
		PRINTM(MERROR,
		       "No firmware image found! Terminating download\n");
		goto done;
	}

	PRINTM(MINFO, "Downloading FW image (%d bytes)\n", firmware_len);

	if (wlan_disable_host_int(pmadapter)) {
		PRINTM(MERROR, "prog_fw: Disabling interrupts failed\n");
		goto done;
	}

	pmbuf = wlan_alloc_mlan_buffer(pmadapter, WLAN_UPLD_SIZE, 0,
				       MOAL_ALLOC_MLAN_BUFFER);
	if (!pmbuf) {
		PRINTM(MERROR, "prog_fw: Unable to allocate mlan_buffer\n");
		goto done;
	}

	/* Perform firmware data transfer */
	do {
		t_u32 ireg_intr = 0;
		t_u32 read_retry_cnt = 0;

		/* More data? */
		if (offset >= firmware_len)
			break;

		for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
			ret = pcb->moal_read_reg(pmadapter->pmoal_handle,
						 PCIE_SCRATCH_2_REG, &len);
			if (ret) {
				PRINTM(MWARN,
				       "Failed reading length from boot code\n");
				goto done;
			}
			if (len)
				break;
			wlan_udelay(pmadapter, 10);
		}

		if (!len) {
			break;
		} else if (len > WLAN_UPLD_SIZE) {
			PRINTM(MERROR,
			       "FW download failure @ %d, invalid length %d\n",
			       offset, len);
			goto done;
		}

		txlen = len;

		if (len & MBIT(0)) {
			block_retry_cnt++;
			if (block_retry_cnt > MAX_WRITE_IOMEM_RETRY) {
				PRINTM(MERROR,
				       "FW download failure @ %d, over max "
				       "retry count\n", offset);
				goto done;
			}
			PRINTM(MERROR, "FW CRC error indicated by the "
			       "helper: len = 0x%04X, txlen = %d\n", len,
			       txlen);
			len &= ~MBIT(0);
			/* Setting this to 0 to resend from same offset */
			txlen = 0;
		} else {
			block_retry_cnt = 0;
			/* Set blocksize to transfer - checking for last block */
			if (firmware_len - offset < txlen)
				txlen = firmware_len - offset;

			PRINTM(MINFO, ".");

			tx_blocks = (txlen + MLAN_PCIE_BLOCK_SIZE_FW_DNLD - 1) /
				MLAN_PCIE_BLOCK_SIZE_FW_DNLD;

			/* Copy payload to buffer */
			memmove(pmadapter, pmbuf->pbuf + pmbuf->data_offset,
				&firmware[offset], txlen);
			pmbuf->data_len = txlen;
		}

		/* Send the boot command to device */
		if (wlan_pcie_send_boot_cmd(pmadapter, pmbuf)) {
			PRINTM(MERROR,
			       "Failed to send firmware download command\n");
			goto done;
		}
		/* Wait for the command done interrupt */
		do {
			if (read_retry_cnt > MAX_READ_REG_RETRY) {
				PRINTM(MERROR,
				       "prog_fw: Failed to get command done interrupt "
				       "retry count = %d\n", read_retry_cnt);
				goto done;
			}
			if (pcb->
			    moal_read_reg(pmadapter->pmoal_handle,
					  PCIE_CPU_INT_STATUS, &ireg_intr)) {
				PRINTM(MERROR,
				       "prog_fw: Failed to read "
				       "interrupt status during fw dnld\n");
				/* buffer was mapped in send_boot_cmd, unmap first */
				pcb->moal_unmap_memory(pmadapter->pmoal_handle,
						       pmbuf->pbuf +
						       pmbuf->data_offset,
						       pmbuf->buf_pa,
						       WLAN_UPLD_SIZE,
						       PCI_DMA_TODEVICE);
				goto done;
			}
			read_retry_cnt++;
		} while ((ireg_intr & CPU_INTR_DOOR_BELL) ==
			 CPU_INTR_DOOR_BELL);
		/* got interrupt - can unmap buffer now */
		if (MLAN_STATUS_FAILURE ==
		    pcb->moal_unmap_memory(pmadapter->pmoal_handle,
					   pmbuf->pbuf + pmbuf->data_offset,
					   pmbuf->buf_pa, WLAN_UPLD_SIZE,
					   PCI_DMA_TODEVICE)) {
			PRINTM(MERROR,
			       "prog_fw: failed to moal_unmap_memory\n");
			goto done;
		}
		offset += txlen;
	} while (MTRUE);

	PRINTM(MINFO, "FW download over, size %d bytes\n", offset);
	ret = MLAN_STATUS_SUCCESS;

done:
	if (pmbuf)
		wlan_free_mlan_buffer(pmadapter, pmbuf);

	LEAVE();
	return ret;
}

/********************************************************
			Global Functions
********************************************************/
/**
 *  @brief PCIE wakeup handler
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return 	      MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_wakeup(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 data = 0;

	ENTER();

	/* Enable interrupts or any chip access will wakeup device */
	ret = pmadapter->callbacks.moal_read_reg(pmadapter->pmoal_handle,
						 PCIE_IP_REV_REG, &data);

	if (ret == MLAN_STATUS_SUCCESS) {
		PRINTM(MINFO,
		       "PCIE wakeup: Read PCI register to wakeup device ...\n");
	} else {
		PRINTM(MINFO, "PCIE wakeup: Failed to wakeup device ...\n");
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function gets interrupt status.
 *
 */
/**
 *  @param msg_id  A message id
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @return         MLAN_STATUS_FAILURE -- if the intererupt is not for us.
 */
mlan_status
wlan_interrupt(t_u16 msg_id, pmlan_adapter pmadapter)
{
	t_u32 pcie_ireg;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_void *pmoal_handle = pmadapter->pmoal_handle;
	t_void *pint_lock = pmadapter->pint_lock;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (pmadapter->pcie_int_mode == PCIE_INT_MODE_MSI) {
		pcb->moal_spin_lock(pmoal_handle, pint_lock);
		pmadapter->pcie_ireg = 1;
		pcb->moal_spin_unlock(pmoal_handle, pint_lock);
	} else if (pmadapter->pcie_int_mode == PCIE_INT_MODE_MSIX) {
		pcie_ireg = (1 << msg_id) & HOST_INTR_MASK;
		if (pcie_ireg) {
			if (!pmadapter->pps_uapsd_mode &&
			    (pmadapter->ps_state == PS_STATE_SLEEP)
				) {
				pmadapter->pm_wakeup_fw_try = MFALSE;
				pmadapter->ps_state = PS_STATE_AWAKE;
				pmadapter->pm_wakeup_card_req = MFALSE;
			}
		}
		pcb->moal_spin_lock(pmoal_handle, pint_lock);
		pmadapter->pcie_ireg |= pcie_ireg;
		pcb->moal_spin_unlock(pmoal_handle, pint_lock);

		PRINTM(MINTR, "ireg: 0x%08x\n", pcie_ireg);
	} else {
		wlan_disable_host_int(pmadapter);
		if (pcb->
		    moal_read_reg(pmoal_handle, PCIE_HOST_INT_STATUS,
				  &pcie_ireg)) {
			PRINTM(MWARN, "Read register failed\n");
			LEAVE();
			return MLAN_STATUS_FAILURE;
		}

		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "pcie_ireg=0x%x\n", pcie_ireg);
			if (!pmadapter->pps_uapsd_mode &&
			    (pmadapter->ps_state == PS_STATE_SLEEP)
				) {
				/* Potentially for PCIe we could get other
				 * interrupts like shared. */
				pmadapter->pm_wakeup_fw_try = MFALSE;
				pmadapter->ps_state = PS_STATE_AWAKE;
				pmadapter->pm_wakeup_card_req = MFALSE;

			}

			pcb->moal_spin_lock(pmoal_handle, pint_lock);
			pmadapter->pcie_ireg |= pcie_ireg;
			pcb->moal_spin_unlock(pmoal_handle, pint_lock);

			/* Clear the pending interrupts */
			if (pcb->
			    moal_write_reg(pmoal_handle, PCIE_HOST_INT_STATUS,
					   ~pcie_ireg)) {
				PRINTM(MWARN, "Write register failed\n");
				LEAVE();
				return ret;
			}
		} else {
			wlan_pcie_enable_host_int_mask(pmadapter);
			PRINTM(MINFO, "This is not our interrupt\n");
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function checks the msix interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_process_msix_int(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 pcie_ireg = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pint_lock);
	pcie_ireg = pmadapter->pcie_ireg & HOST_INTR_MASK;
	pmadapter->pcie_ireg = 0;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle, pmadapter->pint_lock);

	if (pcie_ireg & HOST_INTR_DNLD_DONE) {
		PRINTM(MINFO, "<--- DATA sent Interrupt --->\n");
		ret = wlan_pcie_send_data_complete(pmadapter);
		if (ret)
			goto done;
	}
	if (pcie_ireg & HOST_INTR_UPLD_RDY) {
		PRINTM(MINFO, "Rx DATA\n");
		ret = wlan_pcie_process_recv_data(pmadapter);
		if (ret)
			goto done;
	}
	if (pcie_ireg & HOST_INTR_EVENT_RDY) {
		PRINTM(MINFO, "Rx EVENT\n");
		ret = wlan_pcie_process_event_ready(pmadapter);
		if (ret)
			goto done;
	}
	if (pcie_ireg & HOST_INTR_CMD_DONE) {
		if (pmadapter->cmd_sent) {
			PRINTM(MINFO, "<--- CMD sent Interrupt --->\n");
			pmadapter->cmd_sent = MFALSE;
		}
		ret = wlan_pcie_process_cmd_resp(pmadapter);
		if (ret)
			goto done;
	}

	PRINTM(MINFO, "cmd_sent=%d data_sent=%d\n", pmadapter->cmd_sent,
	       pmadapter->data_sent);

done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function checks the interrupt status and
 *  handle it accordingly.
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_process_int_status(mlan_adapter *pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 pcie_ireg = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pmadapter->pint_lock);
	if (pmadapter->pcie_int_mode != PCIE_INT_MODE_MSI)
		pcie_ireg = pmadapter->pcie_ireg;
	pmadapter->pcie_ireg = 0;
	pcb->moal_spin_unlock(pmadapter->pmoal_handle, pmadapter->pint_lock);
	if (pmadapter->pcie_int_mode == PCIE_INT_MODE_MSI) {
		if (pcb->
		    moal_read_reg(pmadapter->pmoal_handle, PCIE_HOST_INT_STATUS,
				  &pcie_ireg)) {
			PRINTM(MWARN, "Read register failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}
		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "pcie_ireg=0x%x\n", pcie_ireg);
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						PCIE_HOST_INT_STATUS,
						~pcie_ireg)) {
				PRINTM(MWARN, "Write register failed\n");
				ret = MLAN_STATUS_FAILURE;
				goto done;
			}
			if (!pmadapter->pps_uapsd_mode &&
			    (pmadapter->ps_state == PS_STATE_SLEEP)
				) {
				/* Potentially for PCIe we could get other
				 * interrupts like shared. */
				pmadapter->pm_wakeup_fw_try = MFALSE;
				pmadapter->ps_state = PS_STATE_AWAKE;
				pmadapter->pm_wakeup_card_req = MFALSE;
				mlan_collect_power_statistic((t_void *)
							     pmadapter);
			}
		}
	}
	while (pcie_ireg & HOST_INTR_MASK) {
		if (pcie_ireg & HOST_INTR_DNLD_DONE) {
			pcie_ireg &= ~HOST_INTR_DNLD_DONE;
			PRINTM(MINFO, "<--- DATA sent Interrupt --->\n");
			ret = wlan_pcie_send_data_complete(pmadapter);
			if (ret)
				goto done;
		}
		if (pcie_ireg & HOST_INTR_UPLD_RDY) {
			pcie_ireg &= ~HOST_INTR_UPLD_RDY;
			PRINTM(MINFO, "Rx DATA\n");
			ret = wlan_pcie_process_recv_data(pmadapter);
			if (ret)
				goto done;
		}
		if (pcie_ireg & HOST_INTR_EVENT_RDY) {
			pcie_ireg &= ~HOST_INTR_EVENT_RDY;
			PRINTM(MINFO, "Rx EVENT\n");
			ret = wlan_pcie_process_event_ready(pmadapter);
			if (ret)
				goto done;
		}
		if (pcie_ireg & HOST_INTR_CMD_DONE) {
			pcie_ireg &= ~HOST_INTR_CMD_DONE;
			if (pmadapter->cmd_sent) {
				PRINTM(MINFO, "<--- CMD sent Interrupt --->\n");
				pmadapter->cmd_sent = MFALSE;
			}
			ret = wlan_pcie_process_cmd_resp(pmadapter);
			if (ret)
				goto done;
		}
		if (pmadapter->pcie_int_mode == PCIE_INT_MODE_MSI) {
			pcb->moal_spin_lock(pmadapter->pmoal_handle,
					    pmadapter->pint_lock);
			pmadapter->pcie_ireg = 0;
			pcb->moal_spin_unlock(pmadapter->pmoal_handle,
					      pmadapter->pint_lock);
		}
		if (pcb->
		    moal_read_reg(pmadapter->pmoal_handle, PCIE_HOST_INT_STATUS,
				  &pcie_ireg)) {
			PRINTM(MWARN, "Read register failed\n");
			ret = MLAN_STATUS_FAILURE;
			goto done;
		}

		if ((pcie_ireg != 0xFFFFFFFF) && (pcie_ireg)) {
			PRINTM(MINTR, "Poll: pcie_ireg=0x%x\n", pcie_ireg);
			if (pcb->moal_write_reg(pmadapter->pmoal_handle,
						PCIE_HOST_INT_STATUS,
						~pcie_ireg)) {
				PRINTM(MWARN, "Write register failed\n");
				return MLAN_STATUS_FAILURE;
			}
			if (pmadapter->pcie_int_mode != PCIE_INT_MODE_MSI) {
				pcb->moal_spin_lock(pmadapter->pmoal_handle,
						    pmadapter->pint_lock);
				pcie_ireg |= pmadapter->pcie_ireg;
				pmadapter->pcie_ireg = 0;
				pcb->moal_spin_unlock(pmadapter->pmoal_handle,
						      pmadapter->pint_lock);
			}
			/* Don't update the pmadapter->pcie_ireg,
			 * serving the status right now */
		}
	}
	PRINTM(MINFO, "cmd_sent=%d data_sent=%d\n", pmadapter->cmd_sent,
	       pmadapter->data_sent);
	if (pmadapter->pcie_int_mode != PCIE_INT_MODE_MSI) {
		if (pmadapter->ps_state != PS_STATE_SLEEP)
			wlan_pcie_enable_host_int_mask(pmadapter);

	}
done:
	LEAVE();
	return ret;
}

/**
 *  @brief This function disables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_disable_host_int(mlan_adapter *pmadapter)
{
	mlan_status ret;

	ENTER();
	ret = wlan_pcie_disable_host_int_mask(pmadapter);
	LEAVE();
	return ret;
}

/**
 *  @brief This function enables the host interrupts.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_enable_host_int(mlan_adapter *pmadapter)
{
	mlan_status ret;

	ENTER();
	ret = wlan_pcie_enable_host_int_status_mask(pmadapter);
	if (ret) {
		LEAVE();
		return ret;
	}
	ret = wlan_pcie_enable_host_int_mask(pmadapter);

	LEAVE();
	return ret;
}

/**
 *  @brief This function sets DRV_READY register
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param val        Value
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 */
mlan_status
wlan_set_drv_ready_reg(mlan_adapter *pmadapter, t_u32 val)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (pcb->moal_write_reg(pmadapter->pmoal_handle, REG_DRV_READY, val)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function checks if the interface is ready to download
 *  or not while other download interface is present
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param val        Winner status (0: winner)
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 *
 */
mlan_status
wlan_check_winner_status(mlan_adapter *pmadapter, t_u32 *val)
{
	t_u32 winner = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();

	if (MLAN_STATUS_SUCCESS !=
	    pcb->moal_read_reg(pmadapter->pmoal_handle, PCIE_SCRATCH_3_REG,
			       &winner)) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	*val = winner;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function checks if the firmware is ready to accept
 *  command or not.
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *  @param pollnum    Maximum polling number
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_check_fw_status(mlan_adapter *pmadapter, t_u32 pollnum)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u32 firmware_stat;
	t_u32 tries;

	ENTER();

	/* Wait for firmware initialization event */
	for (tries = 0; tries < pollnum; tries++) {
		if (pcb->
		    moal_read_reg(pmadapter->pmoal_handle, PCIE_SCRATCH_3_REG,
				  &firmware_stat))
			ret = MLAN_STATUS_FAILURE;
		else
			ret = MLAN_STATUS_SUCCESS;
		if (ret)
			continue;
		if (firmware_stat == FIRMWARE_READY) {
			ret = MLAN_STATUS_SUCCESS;
			break;
		} else {
			wlan_mdelay(pmadapter, 100);
			ret = MLAN_STATUS_FAILURE;
		}
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function init the pcie interface
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_init(mlan_adapter *pmadapter)
{

	ENTER();

	PRINTM(MINFO, "Setting driver ready signature\n");
	if (wlan_set_drv_ready_reg(pmadapter, FIRMWARE_READY)) {
		PRINTM(MERROR, "Failed to write driver ready signature\n");
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief  This function downloads firmware to card
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmfw			A pointer to firmware image
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_dnld_fw(IN pmlan_adapter pmadapter, IN pmlan_fw_image pmfw)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	/* Download the firmware image via helper */
	ret = wlan_prog_fw_w_helper(pmadapter, pmfw);
	if (ret != MLAN_STATUS_SUCCESS) {
		PRINTM(MERROR, "prog_fw failed ret=%#x\n", ret);
		LEAVE();
		return ret;
	}
	LEAVE();
	return ret;
}

/**
 *  @brief This function downloads data from driver to card.
 *
 *  Both commands and data packets are transferred to the card
 *  by this function. This function adds the PCIE specific header
 *  to the front of the buffer before transferring. The header
 *  contains the length of the packet and the type. The firmware
 *  handles the packets based upon this set type.
 *
 *  @param pmadapter A pointer to mlan_adapter structure
 *  @param type	     data or command
 *  @param pmbuf     A pointer to mlan_buffer (pmbuf->data_len should include PCIE header)
 *  @param tx_param  A pointer to mlan_tx_param (can be MNULL if type is command)
 *
 *  @return 	     MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_pcie_host_to_card(mlan_adapter *pmadapter, t_u8 type,
		       mlan_buffer *pmbuf, mlan_tx_param *tx_param)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	if (!pmbuf) {
		PRINTM(MERROR, "Passed NULL pmbuf to %s\n", __FUNCTION__);
		return MLAN_STATUS_FAILURE;
	}

	if (type == MLAN_TYPE_DATA)
		ret = wlan_pcie_send_data(pmadapter, type, pmbuf, tx_param);
	else if (type == MLAN_TYPE_CMD)
		ret = wlan_pcie_send_cmd(pmadapter, pmbuf);

	LEAVE();
	return ret;
}

/**
 *  @brief This function allocates the PCIE ring buffers
 *
 *  The following initializations steps are followed -
 *      - Allocate TXBD ring buffers
 *      - Allocate RXBD ring buffers
 *      - Allocate event BD ring buffers
 *      - Allocate command and command response buffer
 *      - Allocate sleep cookie buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_alloc_pcie_ring_buf(pmlan_adapter pmadapter)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	pmadapter->cmdrsp_buf = MNULL;
	ret = wlan_pcie_create_txbd_ring(pmadapter);
	if (ret)
		goto err_cre_txbd;
	ret = wlan_pcie_create_rxbd_ring(pmadapter);
	if (ret)
		goto err_cre_rxbd;
	ret = wlan_pcie_create_evtbd_ring(pmadapter);
	if (ret)
		goto err_cre_evtbd;
	ret = wlan_pcie_alloc_cmdrsp_buf(pmadapter);
	if (ret)
		goto err_alloc_cmdbuf;

	return ret;

err_alloc_cmdbuf:
	wlan_pcie_delete_evtbd_ring(pmadapter);
err_cre_evtbd:
	wlan_pcie_delete_rxbd_ring(pmadapter);
err_cre_rxbd:
	wlan_pcie_delete_txbd_ring(pmadapter);
err_cre_txbd:

	LEAVE();
	return ret;
}

/**
 *  @brief This function frees the allocated ring buffers.
 *
 *  The following are freed by this function -
 *      - TXBD ring buffers
 *      - RXBD ring buffers
 *      - Event BD ring buffers
 *      - Command and command response buffer
 *      - Sleep cookie buffer
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS
 */
mlan_status
wlan_free_pcie_ring_buf(pmlan_adapter pmadapter)
{
	ENTER();

	wlan_pcie_delete_cmdrsp_buf(pmadapter);
	wlan_pcie_delete_evtbd_ring(pmadapter);
	wlan_pcie_delete_rxbd_ring(pmadapter);
	wlan_pcie_delete_txbd_ring(pmadapter);
	pmadapter->cmdrsp_buf = MNULL;

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function cleans up packets in the ring buffers.
 *
 *  The following are cleaned by this function -
 *      - TXBD ring buffers
 *
 *  @param pmadapter  A pointer to mlan_adapter structure
 *
 *  @return           MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_clean_pcie_ring_buf(pmlan_adapter pmadapter)
{
	ENTER();
	wlan_pcie_flush_txbd_ring(pmadapter);
	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief This function prepares command to set PCI-Express
 *  host buffer configuration
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_set_pcie_buf_config(mlan_private *pmpriv)
{
	pmlan_adapter pmadapter = MNULL;
	HostCmd_DS_PCIE_HOST_BUF_DETAILS host_spec;
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();
	if (!pmpriv) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}
	pmadapter = pmpriv->adapter;
	memset(pmadapter, &host_spec, 0,
	       sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS));

	/* Send the ring base addresses and count to firmware */
	host_spec.txbd_addr_lo = (t_u32)(pmadapter->txbd_ring_pbase);
	host_spec.txbd_addr_hi =
		(t_u32)(((t_u64)pmadapter->txbd_ring_pbase) >> 32);
	host_spec.txbd_count = MLAN_MAX_TXRX_BD;
	host_spec.rxbd_addr_lo = (t_u32)(pmadapter->rxbd_ring_pbase);
	host_spec.rxbd_addr_hi =
		(t_u32)(((t_u64)pmadapter->rxbd_ring_pbase) >> 32);
	host_spec.rxbd_count = MLAN_MAX_TXRX_BD;
	host_spec.evtbd_addr_lo = (t_u32)(pmadapter->evtbd_ring_pbase);
	host_spec.evtbd_addr_hi =
		(t_u32)(((t_u64)pmadapter->evtbd_ring_pbase) >> 32);
	host_spec.evtbd_count = MLAN_MAX_EVT_BD;

	ret = wlan_prepare_cmd(pmpriv, HostCmd_CMD_PCIE_HOST_BUF_DETAILS,
			       HostCmd_ACT_GEN_SET, 0, MNULL, &host_spec);
	if (ret) {
		PRINTM(MERROR, "PCIE_HOST_BUF_CFG: send command failed\n");
		ret = MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief This function prepares command PCIE host buffer config.
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param cmd          A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action   The action: GET or SET
 *  @param pdata_buf    A pointer to data buffer
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status
wlan_cmd_pcie_host_buf_cfg(pmlan_private pmpriv,
			   IN HostCmd_DS_COMMAND *cmd,
			   IN t_u16 cmd_action, IN t_void *pdata_buf)
{
	HostCmd_DS_PCIE_HOST_BUF_DETAILS *ppcie_hoost_spec =
		&cmd->params.pcie_host_spec;

	ENTER();

	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_PCIE_HOST_BUF_DETAILS);
	cmd->size =
		wlan_cpu_to_le16((sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS)) +
				 S_DS_GEN);

	if (cmd_action == HostCmd_ACT_GEN_SET) {
		memcpy(pmpriv->adapter, ppcie_hoost_spec, pdata_buf,
		       sizeof(HostCmd_DS_PCIE_HOST_BUF_DETAILS));
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}
