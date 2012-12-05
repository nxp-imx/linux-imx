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
#ifndef __RTL8723A_XMIT_H__
#define __RTL8723A_XMIT_H__

#include <rtl8192c_xmit.h>


void rtl8723a_update_txdesc(struct xmit_frame *pxmitframe, u8 *pmem);
void rtl8723a_fill_fake_txdesc(PADAPTER padapter, u8 *pDesc, u32 BufferLen, u8 IsPsPoll, u8 IsBTQosNull);

#ifdef CONFIG_SDIO_HCI
s32 rtl8723as_init_xmit_priv(PADAPTER padapter);
void rtl8723as_free_xmit_priv(PADAPTER padapter);
s32 rtl8723as_hal_xmit(PADAPTER padapter, struct xmit_frame *pxmitframe);
void rtl8723as_mgnt_xmit(PADAPTER padapter, struct xmit_frame *pmgntframe);
s32 rtl8723as_xmit_buf_handler(PADAPTER padapter);
#define hal_xmit_handler rtl8723as_xmit_buf_handler
#endif

#endif
