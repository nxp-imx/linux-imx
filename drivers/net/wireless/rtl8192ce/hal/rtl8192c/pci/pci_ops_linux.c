/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *   
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _HCI_OPS_OS_C_

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <osdep_intf.h>
#include <pci_ops.h>
#include <circ_buf.h>
#include <recv_osdep.h>
#include <rtl8192c_hal.h>

#if defined (PLATFORM_LINUX) && defined (PLATFORM_WINDOWS)

#error "Shall be Linux or Windows, but not both!\n"

#endif


static int rtl8192ce_init_rx_ring(_adapter * padapter)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	struct pci_dev	*pdev = pdvobjpriv->ppcidev;
	struct net_device	*dev = padapter->pnetdev;
    	struct recv_stat	*entry = NULL;
	dma_addr_t *mapping = NULL;
	struct sk_buff *skb = NULL;
    	int i, rx_queue_idx;

_func_enter_;

	//rx_queue_idx 0:RX_MPDU_QUEUE
	//rx_queue_idx 1:RX_CMD_QUEUE
	for(rx_queue_idx = 0; rx_queue_idx < 1/*RX_MAX_QUEUE*/; rx_queue_idx ++){
    		precvpriv->rx_ring[rx_queue_idx].desc = 
			pci_alloc_consistent(pdev,
							sizeof(*precvpriv->rx_ring[rx_queue_idx].desc) * precvpriv->rxringcount,
							&precvpriv->rx_ring[rx_queue_idx].dma);

    		if (!precvpriv->rx_ring[rx_queue_idx].desc 
			|| (unsigned long)precvpriv->rx_ring[rx_queue_idx].desc & 0xFF) {
        		DBG_8192C("Cannot allocate RX ring\n");
        		return _FAIL;
    		}

    		_rtw_memset(precvpriv->rx_ring[rx_queue_idx].desc, 0, sizeof(*precvpriv->rx_ring[rx_queue_idx].desc) * precvpriv->rxringcount);
    		precvpriv->rx_ring[rx_queue_idx].idx = 0;

    		for (i = 0; i < precvpriv->rxringcount; i++) {
			skb = dev_alloc_skb(precvpriv->rxbuffersize);
        		if (!skb){
				DBG_8192C("Cannot allocate skb for RX ring\n");
            			return _FAIL;
        		}

        		entry = &precvpriv->rx_ring[rx_queue_idx].desc[i];

			precvpriv->rx_ring[rx_queue_idx].rx_buf[i] = skb;

			mapping = (dma_addr_t *)skb->cb;

			//just set skb->cb to mapping addr for pci_unmap_single use
			*mapping = pci_map_single(pdev, skb_tail_pointer(skb),
						precvpriv->rxbuffersize,
						PCI_DMA_FROMDEVICE);

			//entry->BufferAddress = cpu_to_le32(*mapping);
			entry->rxdw6 = cpu_to_le32(*mapping);

			//entry->Length = precvpriv->rxbuffersize;
			//entry->OWN = 1;
			entry->rxdw0 |= cpu_to_le32(precvpriv->rxbuffersize & 0x00003fff);
			entry->rxdw0 |= cpu_to_le32(OWN);
    		}

		//entry->EOR = 1;
		entry->rxdw0 |= cpu_to_le32(EOR);
	}

_func_exit_;

    	return _SUCCESS;
}

static void rtl8192ce_free_rx_ring(_adapter * padapter)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	struct pci_dev	*pdev = pdvobjpriv->ppcidev;
	int i, rx_queue_idx;

_func_enter_;

	//rx_queue_idx 0:RX_MPDU_QUEUE
	//rx_queue_idx 1:RX_CMD_QUEUE
	for (rx_queue_idx = 0; rx_queue_idx < 1/*RX_MAX_QUEUE*/; rx_queue_idx++) {
		for (i = 0; i < precvpriv->rxringcount; i++) {
			struct sk_buff *skb = precvpriv->rx_ring[rx_queue_idx].rx_buf[i];
			if (!skb)
				continue;

			pci_unmap_single(pdev,
					 *((dma_addr_t *) skb->cb),
					 precvpriv->rxbuffersize,
					 PCI_DMA_FROMDEVICE);
			kfree_skb(skb);
		}

		pci_free_consistent(pdev,
				    sizeof(*precvpriv->rx_ring[rx_queue_idx].desc) *
				    precvpriv->rxringcount,
				    precvpriv->rx_ring[rx_queue_idx].desc,
				    precvpriv->rx_ring[rx_queue_idx].dma);
		precvpriv->rx_ring[rx_queue_idx].desc = NULL;
	}

_func_exit_;
}


static int rtl8192ce_init_tx_ring(_adapter * padapter, unsigned int prio, unsigned int entries)
{
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	struct pci_dev	*pdev = pdvobjpriv->ppcidev;
	struct tx_desc	*ring;
	dma_addr_t		dma;
	int	i;

_func_enter_;

	//DBG_8192C("%s entries num:%d\n", __func__, entries);
	ring = pci_alloc_consistent(pdev, sizeof(*ring) * entries, &dma);
	if (!ring || (unsigned long)ring & 0xFF) {
		DBG_8192C("Cannot allocate TX ring (prio = %d)\n", prio);
		return _FAIL;
	}

	_rtw_memset(ring, 0, sizeof(*ring) * entries);
	pxmitpriv->tx_ring[prio].desc = ring;
	pxmitpriv->tx_ring[prio].dma = dma;
	pxmitpriv->tx_ring[prio].idx = 0;
	pxmitpriv->tx_ring[prio].entries = entries;
	_rtw_init_queue(&pxmitpriv->tx_ring[prio].queue);
	pxmitpriv->tx_ring[prio].qlen = 0;

	//DBG_8192C("%s queue:%d, ring_addr:%p\n", __func__, prio, ring);

	for (i = 0; i < entries; i++) {
		ring[i].txdw10 = cpu_to_le32((u32) dma + ((i + 1) % entries) * sizeof(*ring));
	}

_func_exit_;

	return _SUCCESS;
}

static void rtl8192ce_free_tx_ring(_adapter * padapter, unsigned int prio)
{
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	struct pci_dev	*pdev = pdvobjpriv->ppcidev;
	struct rtw_tx_ring *ring = &pxmitpriv->tx_ring[prio];

_func_enter_;

	while (ring->qlen) {
		struct tx_desc	*entry = &ring->desc[ring->idx];
		struct xmit_buf	*pxmitbuf = rtl8192ce_dequeue_xmitbuf(ring);

		pci_unmap_single(pdev, le32_to_cpu(entry->txdw8), pxmitbuf->len, PCI_DMA_TODEVICE);

		rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

		ring->idx = (ring->idx + 1) % ring->entries;
	}

	pci_free_consistent(pdev, sizeof(*ring->desc) * ring->entries, ring->desc, ring->dma);
	ring->desc = NULL;

_func_exit_;
}

static void init_desc_ring_var(_adapter * padapter)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	u8 i = 0;

	for (i = 0; i < HW_QUEUE_ENTRY; i++) {
		pxmitpriv->txringcount[i] = TXDESC_NUM;
	}

	//we just alloc 2 desc for beacon queue,
	//because we just need first desc in hw beacon.
	pxmitpriv->txringcount[BCN_QUEUE_INX] = 2;

	// BE queue need more descriptor for performance consideration
	// or, No more tx desc will happen, and may cause mac80211 mem leakage.
	//if(!padapter->registrypriv.wifi_spec)
	//	pxmitpriv->txringcount[BE_QUEUE_INX] = TXDESC_NUM_BE_QUEUE;

	pxmitpriv->txringcount[TXCMD_QUEUE_INX] = 2;

	precvpriv->rxbuffersize = MAX_RECVBUF_SZ;	//2048;//1024;
	precvpriv->rxringcount = PCI_MAX_RX_COUNT;	//64;
}


u32 rtl8192ce_init_desc_ring(_adapter * padapter)
{
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	int	i, ret = _SUCCESS;

_func_enter_;

	init_desc_ring_var(padapter);

	ret = rtl8192ce_init_rx_ring(padapter);
	if (ret == _FAIL) {
		return ret;
	}

	// general process for other queue */
	for (i = 0; i < PCI_MAX_TX_QUEUE_COUNT; i++) {
		ret = rtl8192ce_init_tx_ring(padapter, i, pxmitpriv->txringcount[i]);
		if (ret == _FAIL)
			goto err_free_rings;
	}

	return ret;

err_free_rings:

	rtl8192ce_free_rx_ring(padapter);

	for (i = 0; i <PCI_MAX_TX_QUEUE_COUNT; i++)
		if (pxmitpriv->tx_ring[i].desc)
			rtl8192ce_free_tx_ring(padapter, i);

_func_exit_;

	return ret;
}

u32 rtl8192ce_free_desc_ring(_adapter * padapter)
{
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	u32 i;

_func_enter_;

	// free rx rings 
	rtl8192ce_free_rx_ring(padapter);

	// free tx rings
	for (i = 0; i < HW_QUEUE_ENTRY; i++) {
		rtl8192ce_free_tx_ring(padapter, i);
	}

_func_exit_;

	return _SUCCESS;
}

void rtl8192ce_reset_desc_ring(_adapter * padapter)
{
	_irqL	irqL;
	struct xmit_priv	*pxmitpriv = &padapter->xmitpriv;
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	int i,rx_queue_idx;

	for(rx_queue_idx = 0; rx_queue_idx < RX_MAX_QUEUE; rx_queue_idx ++){	
    		if(precvpriv->rx_ring[rx_queue_idx].desc) {
        		struct recv_stat *entry = NULL;
        		for (i = 0; i < precvpriv->rxringcount; i++) {
            			entry = &precvpriv->rx_ring[rx_queue_idx].desc[i];
				entry->rxdw0 |= cpu_to_le32(OWN);
        		}
        		precvpriv->rx_ring[rx_queue_idx].idx = 0;
    		}
	}

	_enter_critical(&pdvobjpriv->irq_th_lock, &irqL);
	for (i = 0; i < PCI_MAX_TX_QUEUE_COUNT; i++) {
		if (pxmitpriv->tx_ring[i].desc) {
			struct rtw_tx_ring *ring = &pxmitpriv->tx_ring[i];

			while (ring->qlen) {
				struct tx_desc	*entry = &ring->desc[ring->idx];
				struct xmit_buf	*pxmitbuf = rtl8192ce_dequeue_xmitbuf(ring);

				pci_unmap_single(pdvobjpriv->ppcidev, le32_to_cpu(entry->txdw8), pxmitbuf->len, PCI_DMA_TODEVICE);

				rtw_free_xmitbuf(pxmitpriv, pxmitbuf);

				ring->idx = (ring->idx + 1) % ring->entries;
			}
			ring->idx = 0;
		}
	}
	_exit_critical(&pdvobjpriv->irq_th_lock, &irqL);
}

//
// 2009/10/28 MH Enable rtl8192ce DMA64 function. We need to enable 0x719 BIT5
//
#ifdef CONFIG_64BIT_DMA
u8 PlatformEnable92CEDMA64(PADAPTER Adapter)
{
	struct dvobj_priv	*pdvobjpriv = &Adapter->dvobjpriv;
	struct pci_dev	*pdev = pdvobjpriv->ppcidev;
	u8	bResult = _TRUE;
	u8	value;

	pci_read_config_byte(pdev,0x719, &value);

	// 0x719 Bit5 is DMA64 bit fetch. 
	value |= (BIT5);

	pci_write_config_byte(pdev,0x719, value);
	
	return bResult;
}
#endif

static void rtl8192ce_xmit_beacon(PADAPTER Adapter)
{
#if defined (CONFIG_AP_MODE) && defined (CONFIG_NATIVEAP_MLME)
	struct mlme_priv *pmlmepriv = &Adapter->mlmepriv;

	if(check_fwstate(pmlmepriv, WIFI_AP_STATE))
	{
		//send_beacon(Adapter);
		if(pmlmepriv->update_bcn == _TRUE)
		{
			tx_beacon_hdl(Adapter, NULL);
		}
	}
#endif
}

void rtl8192ce_prepare_bcn_tasklet(void *priv)
{
	_adapter	*padapter = (_adapter*)priv;

	rtl8192ce_xmit_beacon(padapter);
}


static void rtl8192ce_tx_isr(PADAPTER Adapter, int prio)
{
	struct xmit_priv	*pxmitpriv = &Adapter->xmitpriv;
	struct dvobj_priv	*pdvobjpriv = &Adapter->dvobjpriv;
	struct rtw_tx_ring	*ring = &pxmitpriv->tx_ring[prio];
#ifdef CONFIG_CONCURRENT_MODE
	PADAPTER pbuddy_adapter = Adapter->pbuddy_adapter;
#endif	

	while(ring->qlen)
	{
		struct tx_desc	*entry = &ring->desc[ring->idx];
		struct xmit_buf	*pxmitbuf;

	        //  beacon packet will only use the first descriptor defautly,
	        //  and the OWN may not be cleared by the hardware 
	        // 
	        if(prio != BCN_QUEUE_INX) {
			if(le32_to_cpu(entry->txdw0) & OWN)
				return;
			ring->idx = (ring->idx + 1) % ring->entries;
		}

		pxmitbuf = rtl8192ce_dequeue_xmitbuf(ring);

		pci_unmap_single(pdvobjpriv->ppcidev, le32_to_cpu(entry->txdw8), pxmitbuf->len, PCI_DMA_TODEVICE);

		if ( prio == BCN_QUEUE_INX){
			entry->txdw0 &= ~OWN;
		}

		rtw_free_xmitbuf(&(pxmitbuf->padapter->xmitpriv), pxmitbuf);
	}

	if(prio != BCN_QUEUE_INX) {
		if ((check_fwstate(&Adapter->mlmepriv, _FW_UNDER_SURVEY) != _TRUE)
			&& rtw_txframes_pending(Adapter))
		{
			// try to deal with the pending packets
			tasklet_schedule(&(Adapter->xmitpriv.xmit_tasklet));
		}
#ifdef CONFIG_CONCURRENT_MODE
		if ((check_fwstate(&pbuddy_adapter->mlmepriv, _FW_UNDER_SURVEY) != _TRUE)
			&& rtw_txframes_pending(pbuddy_adapter))
		{
			// try to deal with the pending packets
			tasklet_schedule(&(pbuddy_adapter->xmitpriv.xmit_tasklet));
		}
#endif
	}
}


s32	rtl8192ce_interrupt(PADAPTER Adapter)
{
	_irqL	irqL;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct dvobj_priv	*pdvobjpriv = &Adapter->dvobjpriv;
	struct xmit_priv	*pxmitpriv = &Adapter->xmitpriv;
	RT_ISR_CONTENT	isr_content;
	//PRT_ISR_CONTENT	pisr_content = &pdvobjpriv->isr_content;
	int	ret = _SUCCESS;


	_enter_critical(&pdvobjpriv->irq_th_lock, &irqL);

	//read ISR: 4/8bytes
	InterruptRecognized8192CE(Adapter, &isr_content);

	// Shared IRQ or HW disappared 
	if (!isr_content.IntArray[0] || isr_content.IntArray[0] == 0xffff) {
		ret = _FAIL;
		goto done;
	}

	//<1> beacon related
	if (isr_content.IntArray[0] & IMR_TBDOK) {
		//DBG_8192C("beacon ok interrupt!\n");
	}

	if (isr_content.IntArray[0] & IMR_TBDER) {
		//DBG_8192C("beacon err interrupt!\n");
	}

	if (isr_content.IntArray[0] & IMR_BDOK) {
		//DBG_8192C("Beacon Queue DMA OK interrupt!\n");
		rtl8192ce_tx_isr(Adapter, BCN_QUEUE_INX);		
	}
	
	if (isr_content.IntArray[0] & IMR_BcnInt) {
		struct tasklet_struct  *bcn_tasklet = &Adapter->recvpriv.irq_prepare_beacon_tasklet;
#ifdef CONFIG_CONCURRENT_MODE
		if (Adapter->iface_type == IFACE_PORT1)
			bcn_tasklet = &Adapter->pbuddy_adapter->recvpriv.irq_prepare_beacon_tasklet;
#endif
		tasklet_hi_schedule(bcn_tasklet);
	}
#ifdef CONFIG_CONCURRENT_MODE	
	if (isr_content.IntArray[1] & IMR_BcnInt_E) {
		struct tasklet_struct  *bcn_tasklet = &Adapter->recvpriv.irq_prepare_beacon_tasklet;
		if (Adapter->iface_type == IFACE_PORT0)
			bcn_tasklet = &Adapter->pbuddy_adapter->recvpriv.irq_prepare_beacon_tasklet;
		tasklet_hi_schedule(bcn_tasklet);
	}
#endif
	
	//<2> Rx related
	if (isr_content.IntArray[0] & IMR_RX_MASK) {
		pHalData->IntrMaskToSet[0] &= (~IMR_RX_MASK);
		rtw_write32(Adapter, REG_HIMR, pHalData->IntrMaskToSet[0]);
		tasklet_hi_schedule(&Adapter->recvpriv.recv_tasklet);
	}

	/*if (isr_content.IntArray[0] & IMR_ROK) {
		//DBG_8192C("Rx ok interrupt!\n");
		tasklet_hi_schedule(&Adapter->recvpriv.recv_tasklet);
	}

	if (isr_content.IntArray[0] & IMR_RDU) {
		//DBG_8192C("rx descriptor unavailable!\n");
		// reset int situation 
		tasklet_hi_schedule(&Adapter->recvpriv.recv_tasklet);
	}

	if (isr_content.IntArray[0] & IMR_RXFOVW) {
		//DBG_8192C("rx overflow !\n");
		tasklet_hi_schedule(&Adapter->recvpriv.recv_tasklet);
	}*/


	//<3> Tx related
	if (isr_content.IntArray[0] & IMR_TXFOVW) {
		DBG_8192C("IMR_TXFOVW!\n");
	}

	//if (pisr_content->IntArray[0] & IMR_TX_MASK) {
	//	tasklet_hi_schedule(&pxmitpriv->xmit_tasklet);
	//}

	if (isr_content.IntArray[0] & IMR_MGNTDOK) {
		//DBG_8192C("Manage ok interrupt!\n");		
		rtl8192ce_tx_isr(Adapter, MGT_QUEUE_INX);
	}

	if (isr_content.IntArray[0] & IMR_HIGHDOK) {
		//DBG_8192C("HIGH_QUEUE ok interrupt!\n");
		rtl8192ce_tx_isr(Adapter, HIGH_QUEUE_INX);
	}

	if (isr_content.IntArray[0] & IMR_BKDOK) {
		//DBG_8192C("BK Tx OK interrupt!\n");
		rtl8192ce_tx_isr(Adapter, BK_QUEUE_INX);
	}

	if (isr_content.IntArray[0] & IMR_BEDOK) {
		//DBG_8192C("BE TX OK interrupt!\n");
		rtl8192ce_tx_isr(Adapter, BE_QUEUE_INX);
	}

	if (isr_content.IntArray[0] & IMR_VIDOK) {
		//DBG_8192C("VI TX OK interrupt!\n");
		rtl8192ce_tx_isr(Adapter, VI_QUEUE_INX);
	}

	if (isr_content.IntArray[0] & IMR_VODOK) {
		//DBG_8192C("Vo TX OK interrupt!\n");
		rtl8192ce_tx_isr(Adapter, VO_QUEUE_INX);
	}

done:

	_exit_critical(&pdvobjpriv->irq_th_lock, &irqL);

	return ret;
}

/*	Aries add, 20120305
	clone another recvframe and associate with secondary_adapter.
*/
#ifdef CONFIG_CONCURRENT_MODE
static int rtl8192ce_if2_clone_recvframe(_adapter *sec_padapter, union recv_frame *precvframe, union recv_frame *precvframe_if2, struct  sk_buff *clone_pkt, struct phy_stat *pphy_info)
{
	struct rx_pkt_attrib	*pattrib = NULL;

	
	precvframe_if2->u.hdr.adapter = sec_padapter;
	_rtw_init_listhead(&precvframe_if2->u.hdr.list);	
	precvframe_if2->u.hdr.precvbuf = NULL;	//can't access the precvbuf for new arch.
	precvframe_if2->u.hdr.len=0;

	_rtw_memcpy(&precvframe_if2->u.hdr.attrib, &precvframe->u.hdr.attrib, sizeof(struct rx_pkt_attrib));

	
	if(clone_pkt)
	{
		clone_pkt->dev = sec_padapter->pnetdev;
		precvframe_if2->u.hdr.pkt = clone_pkt;
		precvframe_if2->u.hdr.rx_head = precvframe_if2->u.hdr.rx_data = precvframe_if2->u.hdr.rx_tail = clone_pkt->data;
		precvframe_if2->u.hdr.rx_end = clone_pkt->data + clone_pkt->len;
	}
	else
	{
		DBG_8192C("rtl8192ce_rx_mpdu:can not allocate memory for skb_copy\n");
		return _FAIL;
	}
	recvframe_put(precvframe_if2, clone_pkt->len);
	rtl8192c_translate_rx_signal_stuff(precvframe_if2, pphy_info);

	return _SUCCESS;
}
#endif

static void rtl8192ce_rx_mpdu(_adapter *padapter)
{
	struct recv_priv	*precvpriv = &padapter->recvpriv;
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	_queue			*pfree_recv_queue = &precvpriv->free_recv_queue;
	union recv_frame	*precvframe = NULL;
	struct phy_stat	*pphy_info = NULL;
	struct rx_pkt_attrib	*pattrib = NULL;
	u8	qos_shift_sz = 0;
	u32	skb_len, alloc_sz;
	int	rx_queue_idx = RX_MPDU_QUEUE;
	u32	count = precvpriv->rxringcount;

#ifdef CONFIG_CONCURRENT_MODE
	_adapter *sec_padapter = padapter->pbuddy_adapter;
	union recv_frame	*precvframe_if2 = NULL;
	u8 *paddr1 = NULL ;
	u8 *secondary_myid = NULL ;

	struct sk_buff  *clone_pkt = NULL;
#endif	
	//RX NORMAL PKT
	while (count--)
	{
		struct recv_stat *pdesc = &precvpriv->rx_ring[rx_queue_idx].desc[precvpriv->rx_ring[rx_queue_idx].idx];//rx descriptor
		struct sk_buff *skb = precvpriv->rx_ring[rx_queue_idx].rx_buf[precvpriv->rx_ring[rx_queue_idx].idx];//rx pkt

		if (le32_to_cpu(pdesc->rxdw0) & OWN){//OWN bit
			// wait data to be filled by hardware */
			return;
		}
		else
		{
			struct sk_buff  *pkt_copy = NULL;

			precvframe = rtw_alloc_recvframe(pfree_recv_queue);
			if(precvframe==NULL)
			{
				RT_TRACE(_module_rtl871x_recv_c_,_drv_err_,("recvbuf2recvframe: precvframe==NULL\n"));
				DBG_8192C("recvbuf2recvframe: precvframe==NULL\n");
				goto done;
			}

			_rtw_init_listhead(&precvframe->u.hdr.list);
			precvframe->u.hdr.len=0;

			pci_unmap_single(pdvobjpriv->ppcidev,
					*((dma_addr_t *)skb->cb), 
					precvpriv->rxbuffersize, 
					PCI_DMA_FROMDEVICE);

			rtl8192c_query_rx_desc_status(precvframe, pdesc);

			pattrib = &precvframe->u.hdr.attrib;
			if(pattrib->physt)
			{
				pphy_info = (struct phy_stat *)(skb->data + pattrib->shift_sz);
			}

			//	Modified by Albert 20101213
			//	For 8 bytes IP header alignment.
			if (pattrib->qos)	//	Qos data, wireless lan header length is 26
			{
				qos_shift_sz = 6;
			}
			else
			{
				qos_shift_sz = 0;
			}

			skb_len = pattrib->pkt_len;

			// for first fragment packet, driver need allocate 1536+drvinfo_sz+RXDESC_SIZE to defrag packet.
			// modify alloc_sz for recvive crc error packet by thomas 2011-06-02
			if((pattrib->mfrag == 1)&&(pattrib->frag_num == 0)){
				//alloc_sz = 1664;	//1664 is 128 alignment.
				if(skb_len <= 1650)
					alloc_sz = 1664;
				else
					alloc_sz = skb_len + 14;
			}
			else {
				alloc_sz = skb_len;
				//	6 is for IP header 8 bytes alignment in QoS packet case.
				//	8 is for skb->data 4 bytes alignment.
				alloc_sz += 14;
			}

			pkt_copy = dev_alloc_skb(alloc_sz);
			pkt_copy->len = skb_len;
			if(pkt_copy)
			{
				pkt_copy->dev = padapter->pnetdev;
				precvframe->u.hdr.pkt = pkt_copy;
				skb_reserve( pkt_copy, 8 - ((SIZE_PTR)( pkt_copy->data ) & 7 ));//force pkt_copy->data at 8-byte alignment address
				skb_reserve( pkt_copy, qos_shift_sz );//force ip_hdr at 8-byte alignment address according to shift_sz.
				_rtw_memcpy(pkt_copy->data, (skb->data + pattrib->drvinfo_sz + pattrib->shift_sz), skb_len);
				precvframe->u.hdr.rx_head = precvframe->u.hdr.rx_data = precvframe->u.hdr.rx_tail = pkt_copy->data;
				precvframe->u.hdr.rx_end = pkt_copy->data + alloc_sz;
			}
			else
			{	
				DBG_8192C("rtl8192ce_rx_mpdu:can not allocate memory for skb copy\n");
				*((dma_addr_t *) skb->cb) = pci_map_single(pdvobjpriv->ppcidev, skb_tail_pointer(skb), precvpriv->rxbuffersize, PCI_DMA_FROMDEVICE);
				goto done;
			}

			recvframe_put(precvframe, skb_len);

//			rtl8192c_translate_rx_signal_stuff(precvframe, pphy_info);

#ifdef CONFIG_CONCURRENT_MODE
			if(!rtw_buddy_adapter_up(padapter))
				goto skip_if2_recv;
			
			paddr1 = GetAddr1Ptr(precvframe->u.hdr.rx_data);
			if(IS_MCAST(paddr1) == _FALSE) {	//unicast packets
				secondary_myid = myid(&sec_padapter->eeprompriv);

				if(_rtw_memcmp(paddr1, secondary_myid, ETH_ALEN)) {
					pkt_copy->dev = sec_padapter->pnetdev;
					precvframe->u.hdr.adapter = sec_padapter;
				}
			} else {
				precvframe_if2 = rtw_alloc_recvframe(pfree_recv_queue);
				if(precvframe_if2==NULL) {
					DBG_8192C("%s(): precvframe_if2==NULL\n", __func__);
					goto done;
				}
				clone_pkt = skb_copy(pkt_copy, GFP_ATOMIC);
				if (!clone_pkt){
					DBG_8192C("%s(): skb_copy==NULL\n", __func__);
					rtw_free_recvframe(precvframe_if2, pfree_recv_queue);
					goto done;
				}
				if (rtl8192ce_if2_clone_recvframe(sec_padapter, precvframe, precvframe_if2, clone_pkt, pphy_info) != _SUCCESS) 
					rtw_free_recvframe(precvframe_if2, pfree_recv_queue);
		
				if(rtw_recv_entry(precvframe_if2) != _SUCCESS)
					RT_TRACE(_module_hci_ops_os_c_,_drv_err_,("recvbuf2recvframe: rtw_recv_entry(precvframe) != _SUCCESS\n"));
			}
skip_if2_recv:
#endif
			rtl8192c_translate_rx_signal_stuff(precvframe, pphy_info);

			if(rtw_recv_entry(precvframe) != _SUCCESS)
			{
				RT_TRACE(_module_hci_ops_os_c_,_drv_err_,("recvbuf2recvframe: rtw_recv_entry(precvframe) != _SUCCESS\n"));
			}

			//precvpriv->rx_ring[rx_queue_idx].rx_buf[precvpriv->rx_ring[rx_queue_idx].idx] = skb;
			*((dma_addr_t *) skb->cb) = pci_map_single(pdvobjpriv->ppcidev, skb_tail_pointer(skb), precvpriv->rxbuffersize, PCI_DMA_FROMDEVICE);
		}
done:

		pdesc->rxdw6 = cpu_to_le32(*((dma_addr_t *)skb->cb));

		pdesc->rxdw0 |= cpu_to_le32(precvpriv->rxbuffersize & 0x00003fff);
		pdesc->rxdw0 |= cpu_to_le32(OWN);

		if (precvpriv->rx_ring[rx_queue_idx].idx == precvpriv->rxringcount-1)
			pdesc->rxdw0 |= cpu_to_le32(EOR);
		precvpriv->rx_ring[rx_queue_idx].idx = (precvpriv->rx_ring[rx_queue_idx].idx + 1) % precvpriv->rxringcount;

	}

}

void rtl8192ce_recv_tasklet(void *priv)
{
	_irqL	irqL;
	_adapter	*padapter = (_adapter*)priv;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;

	rtl8192ce_rx_mpdu(padapter);
	_enter_critical(&pdvobjpriv->irq_th_lock, &irqL);
	pHalData->IntrMaskToSet[0] |= IMR_RX_MASK;
	rtw_write32(padapter, REG_HIMR, pHalData->IntrMaskToSet[0]);
	_exit_critical(&pdvobjpriv->irq_th_lock, &irqL);
}

static u8 pci_read8(struct intf_hdl *pintfhdl, u32 addr)
{
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;

	return 0xff & readb((u8 *)pdvobjpriv->pci_mem_start + addr);
}

static u16 pci_read16(struct intf_hdl *pintfhdl, u32 addr)
{
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;

	return readw((u8 *)pdvobjpriv->pci_mem_start + addr);
}

static u32 pci_read32(struct intf_hdl *pintfhdl, u32 addr)
{
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;

	return readl((u8 *)pdvobjpriv->pci_mem_start + addr);
}

//2009.12.23. by tynli. Suggested by SD1 victorh. For ASPM hang on AMD and Nvidia.
// 20100212 Tynli: Do read IO operation after write for all PCI bridge suggested by SD1.
// Origianally this is only for INTEL.
static int pci_write8(struct intf_hdl *pintfhdl, u32 addr, u8 val)
{
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;

	writeb(val, (u8 *)pdvobjpriv->pci_mem_start + addr);
	//readb((u8 *)pdvobjpriv->pci_mem_start + addr);
	return 1;
}

static int pci_write16(struct intf_hdl *pintfhdl, u32 addr, u16 val)
{	
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;
	writew(val, (u8 *)pdvobjpriv->pci_mem_start + addr);
	//readw((u8 *)pdvobjpriv->pci_mem_start + addr);
	return 2;
}

static int pci_write32(struct intf_hdl *pintfhdl, u32 addr, u32 val)
{
	struct dvobj_priv  *pdvobjpriv = (struct dvobj_priv  *)pintfhdl->pintf_dev;
	writel(val, (u8 *)pdvobjpriv->pci_mem_start + addr);
	//readl((u8 *)pdvobjpriv->pci_mem_start + addr);
	return 4;
}


static void pci_read_mem(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *rmem)
{
	
}

static void pci_write_mem(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *wmem)
{
	
}

static u32 pci_read_port(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *rmem)
{	
	return 0;
}

void rtl8192ce_xmit_tasklet(void *priv)
{
	//_irqL irqL;
	_adapter			*padapter = (_adapter*)priv;
	//struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	//PRT_ISR_CONTENT	pisr_content = &pdvobjpriv->isr_content;

	/*_enter_critical(&pdvobjpriv->irq_th_lock, &irqL);

	if (pisr_content->IntArray[0] & IMR_BDOK) {
		//DBG_8192C("beacon interrupt!\n");
		rtl8192ce_tx_isr(padapter, BCN_QUEUE_INX);
	}
	
	if (pisr_content->IntArray[0] & IMR_MGNTDOK) {
		//DBG_8192C("Manage ok interrupt!\n");		
		rtl8192ce_tx_isr(padapter, MGT_QUEUE_INX);
	}

	if (pisr_content->IntArray[0] & IMR_HIGHDOK) {
		//DBG_8192C("HIGH_QUEUE ok interrupt!\n");
		rtl8192ce_tx_isr(padapter, HIGH_QUEUE_INX);
	}

	if (pisr_content->IntArray[0] & IMR_BKDOK) {
		//DBG_8192C("BK Tx OK interrupt!\n");
		rtl8192ce_tx_isr(padapter, BK_QUEUE_INX);
	}

	if (pisr_content->IntArray[0] & IMR_BEDOK) {
		//DBG_8192C("BE TX OK interrupt!\n");
		rtl8192ce_tx_isr(padapter, BE_QUEUE_INX);
	}

	if (pisr_content->IntArray[0] & IMR_VIDOK) {
		//DBG_8192C("VI TX OK interrupt!\n");
		rtl8192ce_tx_isr(padapter, VI_QUEUE_INX);
	}

	if (pisr_content->IntArray[0] & IMR_VODOK) {
		//DBG_8192C("Vo TX OK interrupt!\n");
		rtl8192ce_tx_isr(padapter, VO_QUEUE_INX);
	}

	_exit_critical(&pdvobjpriv->irq_th_lock, &irqL);

	if (check_fwstate(&padapter->mlmepriv, _FW_UNDER_SURVEY) != _TRUE)
	{*/
		// try to deal with the pending packets
		rtl8192ce_xmitframe_resume(padapter);
	//}

}

static u32 pci_write_port(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *wmem)
{
	_adapter			*padapter = (_adapter *)pintfhdl->padapter;

	padapter->pnetdev->trans_start = jiffies;

	return 0;
}

void rtl8192ce_set_intf_ops(struct _io_ops	*pops)
{
	_func_enter_;
	
	_rtw_memset((u8 *)pops, 0, sizeof(struct _io_ops));	

	pops->_read8 = &pci_read8;
	pops->_read16 = &pci_read16;
	pops->_read32 = &pci_read32;

	pops->_read_mem = &pci_read_mem;
	pops->_read_port = &pci_read_port;	

	pops->_write8 = &pci_write8;
	pops->_write16 = &pci_write16;
	pops->_write32 = &pci_write32;

	pops->_write_mem = &pci_write_mem;
	pops->_write_port = &pci_write_port;
		
	_func_exit_;

}

