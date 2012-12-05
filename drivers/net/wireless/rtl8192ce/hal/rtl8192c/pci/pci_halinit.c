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
#define _HCI_HAL_INIT_C_

#include <drv_conf.h>
#include <osdep_service.h>
#include <drv_types.h>
#include <rtw_efuse.h>

#include <rtl8192c_hal.h>
#include <rtl8192c_led.h>

#if defined (PLATFORM_LINUX) && defined (PLATFORM_WINDOWS)

#error "Shall be Linux or Windows, but not both!\n"

#endif

#ifndef CONFIG_PCI_HCI

#error "CONFIG_PCI_HCI shall be on!\n"

#endif

#include <pci_ops.h>
#include <pci_hal.h>
#include <pci_osintf.h>


// For Two MAC FPGA verify we must disable all MAC/BB/RF setting
#define FPGA_UNKNOWN		0
#define FPGA_2MAC			1
#define FPGA_PHY			2
#define ASIC					3
#define BOARD_TYPE			ASIC

#if BOARD_TYPE == FPGA_2MAC
#define HAL_FW_ENABLE		0
#define HAL_MAC_ENABLE		0
#define HAL_BB_ENABLE		0
#define HAL_RF_ENABLE		0
#else // FPGA_PHY and ASIC
#define HAL_FW_ENABLE		1
#define HAL_MAC_ENABLE		1
#define HAL_BB_ENABLE		1
#define HAL_RF_ENABLE		1

#define FPGA_RF_UNKOWN	0
#define FPGA_RF_8225		1
#define FPGA_RF_0222D		2
#define FPGA_RF				FPGA_RF_0222D
#endif

static u8 getChnlGroup(u8 chnl)
{
	u8	group=0;

	if (chnl < 3)			// Cjanel 1-3
		group = 0;
	else if (chnl < 9)		// Channel 4-9
		group = 1;
	else					// Channel 10-14
		group = 2;
	
	return group;
}

static void readTxPowerInfo_8192C(
	IN PADAPTER		Adapter,
	BOOLEAN			AutoLoadFail,
	IN u8			*hwinfo
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;
	u8	rf_path, index, tempval;
	u16	i;

	// EEPROM CCK Tx Power
	// EEPROM HT40 1S Tx Power
	for(rf_path=0; rf_path<2; rf_path++)
	{
		for(i=0; i<3; i++)
		{
			if(!AutoLoadFail)
			{
				pHalData->EEPROMChnlAreaTxPwrCCK[rf_path][i] = hwinfo[EEPROM_CCK_TX_PWR_INX+rf_path*3+i];	
				pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][i] = 	hwinfo[EEPROM_HT40_1S_TX_PWR_INX+rf_path*3+i];
			}
			else
			{
				pHalData->EEPROMChnlAreaTxPwrCCK[rf_path][i] = EEPROM_Default_TxPowerLevel;
				pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][i] = EEPROM_Default_TxPowerLevel;
			}
		}
	}
	// EEPROM HT40 2S Tx Power Diff
	for(i=0; i<3; i++)
	{
		if(!AutoLoadFail)
			tempval = hwinfo[EEPROM_HT40_2S_TX_PWR_INX_DIFF+i];
		else
			tempval = EEPROM_Default_HT40_2SDiff;
		pHalData->EEPROMChnlAreaTxPwrHT40_2SDiff[RF_PATH_A][i] = (tempval&0xf);
		pHalData->EEPROMChnlAreaTxPwrHT40_2SDiff[RF_PATH_B][i] = ((tempval&0xf0)>>4);
	}
#if 0
	// Display if needed
	for(rf_path=0; rf_path<2; rf_path++)
		for (i=0; i<3; i++)
			DBG_8192C("RF-%c EEPROM CCK Area(%d) = 0x%x\n", 
				(rf_path==0)? 'A':'B', i, pHalData->EEPROMChnlAreaTxPwrCCK[rf_path][i]);
	for(rf_path=0; rf_path<2; rf_path++)
		for (i=0; i<3; i++)
			DBG_8192C("RF-%c EEPROM HT40 1S Area(%d) = 0x%x\n", 
				(rf_path==0)? 'A':'B', i, pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][i]));
	for(rf_path=0; rf_path<2; rf_path++)
		for (i=0; i<3; i++)
			DBG_8192C("RF-%c EEPROM HT40 2S Diff Area(%d) = 0x%x\n", 
				(rf_path==0)? 'A':'B', i, pHalData->EEPROMChnlAreaTxPwrHT40_2SDiff[rf_path][i]));
#endif
	// Assign eeprom val to hal vars for later use
	for(rf_path=0; rf_path<2; rf_path++)
	{
		// Assign dedicated channel tx power
		for(i=0; i<14; i++)	// channel 1~3 use the same Tx Power Level.
		{
			index = getChnlGroup((u8)i);
			// Record A & B CCK /OFDM - 1T/2T Channel area tx power
			pHalData->TxPwrLevelCck[rf_path][i]  = 
			pHalData->EEPROMChnlAreaTxPwrCCK[rf_path][index];
			pHalData->TxPwrLevelHT40_1S[rf_path][i]  = 
			pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][index];

			if((pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][index] - 
				pHalData->EEPROMChnlAreaTxPwrHT40_2SDiff[rf_path][index]) > 0)
			{
				pHalData->TxPwrLevelHT40_2S[rf_path][i]  = 
					pHalData->EEPROMChnlAreaTxPwrHT40_1S[rf_path][index] - 
					pHalData->EEPROMChnlAreaTxPwrHT40_2SDiff[rf_path][index];
			}
			else
			{
				pHalData->TxPwrLevelHT40_2S[rf_path][i]  = 0;
			}
		}

		/*for(i=0; i<14; i++)
		{
			DBG_8192C("RF-%c, Ch(%d) [CCK = 0x%x / HT40_1S = 0x%x / HT40_2S = 0x%x]\n", 
				(rf_path==0)? 'A':'B', i+1, pHalData->TxPwrLevelCck[rf_path][i], 
				pHalData->TxPwrLevelHT40_1S[rf_path][i], 
				pHalData->TxPwrLevelHT40_2S[rf_path][i]));
		}*/
	}
	
	for(i=0; i<3; i++)
	{
		// Read Power diff limit.
		if(!AutoLoadFail)
		{
			pHalData->EEPROMPwrLimitHT40[i] = hwinfo[EEPROM_HT40_MAX_PWR_OFFSET+i];
			pHalData->EEPROMPwrLimitHT20[i] = hwinfo[EEPROM_HT20_MAX_PWR_OFFSET+3+i];
		}
		else
		{
			pHalData->EEPROMPwrLimitHT40[i] = 0;
			pHalData->EEPROMPwrLimitHT20[i] = 0;
		}
	}

	for(rf_path=0; rf_path<2; rf_path++)
	{
		// Fill Pwr group
		for(i=0; i<14; i++)
		{
			index = getChnlGroup((u8)i);
			if(rf_path == RF_PATH_A)
			{
				pHalData->PwrGroupHT20[rf_path][i] = (pHalData->EEPROMPwrLimitHT20[index]&0xf);
				pHalData->PwrGroupHT40[rf_path][i] = (pHalData->EEPROMPwrLimitHT40[index]&0xf);
			}
			else if(rf_path == RF_PATH_B)
			{
				pHalData->PwrGroupHT20[rf_path][i] = ((pHalData->EEPROMPwrLimitHT20[index]&0xf0)>>4);
				pHalData->PwrGroupHT40[rf_path][i] = ((pHalData->EEPROMPwrLimitHT40[index]&0xf0)>>4);
			}
			//DBG_8192C("RF-%c PwrGroupHT20[%d] = 0x%x\n", (rf_path==0)? 'A':'B', i, pHalData->PwrGroupHT20[rf_path][i]);
			//DBG_8192C("RF-%c PwrGroupHT40[%d] = 0x%x\n", (rf_path==0)? 'A':'B', i, pHalData->PwrGroupHT40[rf_path][i]);
		}
	}


	for(i=0; i<14; i++)	// channel 1~3 use the same Tx Power Level.
	{
		// Read tx power difference between HT OFDM 20/40 MHZ
		index = getChnlGroup((u8)i);
		if(!AutoLoadFail)
			tempval = hwinfo[EEPROM_HT20_TX_PWR_INX_DIFF+index];
		else
			tempval = EEPROM_Default_HT20_Diff;		
		pHalData->TxPwrHt20Diff[RF_PATH_A][i] = (tempval&0xF);
		pHalData->TxPwrHt20Diff[RF_PATH_B][i] = ((tempval>>4)&0xF);

		if(pHalData->TxPwrHt20Diff[RF_PATH_A][i] & BIT3)	//4bit sign number to 8 bit sign number
			pHalData->TxPwrHt20Diff[RF_PATH_A][i] |= 0xF0;		

		if(pHalData->TxPwrHt20Diff[RF_PATH_B][i] & BIT3)	//4bit sign number to 8 bit sign number
			pHalData->TxPwrHt20Diff[RF_PATH_B][i] |= 0xF0;		

		// Read OFDM<->HT tx power diff
		index = getChnlGroup((u8)i);
		if(!AutoLoadFail)
			tempval = hwinfo[EEPROM_OFDM_TX_PWR_INX_DIFF+index];
		else
			tempval = EEPROM_Default_LegacyHTTxPowerDiff;
		pHalData->TxPwrLegacyHtDiff[RF_PATH_A][i] = (tempval&0xF);
		pHalData->TxPwrLegacyHtDiff[RF_PATH_B][i] = ((tempval>>4)&0xF);
	}
	//for ccx use
	pHalData->LegacyHTTxPowerDiff = pHalData->TxPwrLegacyHtDiff[RF_PATH_A][7];
	
	//for(i=0; i<14; i++)
	//	DBG_8192C("RF-A Ht20 to HT40 Diff[%d] = 0x%x\n", i, pHalData->TxPwrHt20Diff[RF_PATH_A][i]);
	//for(i=0; i<14; i++)
	//	DBG_8192C("RF-B Ht20 to HT40 Diff[%d] = 0x%x\n", i, pHalData->TxPwrHt20Diff[RF_PATH_B][i]);
	//for(i=0; i<14; i++)
	//	DBG_8192C("RF-A Legacy to Ht40 Diff[%d] = 0x%x\n", i, pHalData->TxPwrLegacyHtDiff[RF_PATH_A][i]);
	//for(i=0; i<14; i++)
	//	DBG_8192C("RF-B Legacy to HT40 Diff[%d] = 0x%x\n", i, pHalData->TxPwrLegacyHtDiff[RF_PATH_B][i]);

	if(!AutoLoadFail)
	{
		pHalData->EEPROMRegulatory = (hwinfo[EEPROM_RF_OPT1]&0x7);	//bit0~2
	}
	else
	{
		pHalData->EEPROMRegulatory = 0;
	}
	DBG_8192C("EEPROMRegulatory = 0x%x\n", pHalData->EEPROMRegulatory);
	//
	// Get TSSI value for each path.
	//
	if(!AutoLoadFail)
	{
		pHalData->EEPROMTSSI[RF_PATH_A] = hwinfo[EEPROM_TSSI_A];
		pHalData->EEPROMTSSI[RF_PATH_B] = hwinfo[EEPROM_TSSI_B];
	}
	else
	{
		pHalData->EEPROMTSSI[RF_PATH_A] = EEPROM_Default_TSSI;
		pHalData->EEPROMTSSI[RF_PATH_B] = EEPROM_Default_TSSI;
	}
	DBG_8192C("TSSI_A = 0x%x, TSSI_B = 0x%x\n", pHalData->EEPROMTSSI[RF_PATH_A], pHalData->EEPROMTSSI[RF_PATH_B]);
		
	//
	// ThermalMeter from EEPROM
	//
	if(!AutoLoadFail)
		tempval = hwinfo[EEPROM_THERMAL_METER];
	else
		tempval = EEPROM_Default_ThermalMeter;
	pHalData->EEPROMThermalMeter = (tempval&0x1f);	//[4:0]

	if(pHalData->EEPROMThermalMeter == 0x1f || AutoLoadFail)
		pdmpriv->bAPKThermalMeterIgnore = _TRUE;

#if 0
	if(pHalData->EEPROMThermalMeter < 0x06 || pHalData->EEPROMThermalMeter > 0x1c)
		pHalData->EEPROMThermalMeter = 0x12;
#endif	
	
	pdmpriv->ThermalMeter[0] = pHalData->EEPROMThermalMeter;	
	DBG_8192C("ThermalMeter = 0x%x\n", pHalData->EEPROMThermalMeter);
}


static BOOLEAN
Check11nProductID(
	IN	PADAPTER	Adapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	if(((pHalData->EEPROMDID==0x8191)) ||
		(pHalData->EEPROMDID==0x8193)||
		((pHalData->EEPROMDID==0x8176) && (
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8176) || // <= Start of 88CE Solo
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8175) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8181) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8182) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8184) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8185) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8219) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8207) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8208) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8209) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8210) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8220) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8211) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8212) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8213) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8214) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8221) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8215) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8216) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8217) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8218) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8222) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x8186) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x8187) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8191) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8192) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8193) ||
			(pHalData->EEPROMSVID == 0x1A3B && pHalData->EEPROMSMID == 0x1139) ||
			(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x8194) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8197) ||
			(pHalData->EEPROMSVID == 0x1462 && pHalData->EEPROMSMID == 0x3824) ||
			(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x8198) ||
			(pHalData->EEPROMSVID == 0x185F && pHalData->EEPROMSMID == 0x8176) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8206) ||
			(pHalData->EEPROMSVID == 0x1A32 && pHalData->EEPROMSMID == 0x0315) ||
			(pHalData->EEPROMSVID == 0x144F && pHalData->EEPROMSMID == 0x7185) ||
			(pHalData->EEPROMSVID == 0x103C && pHalData->EEPROMSMID == 0x1629) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8199) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8203) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8204) ||
			(pHalData->EEPROMSVID == 0x1043 && pHalData->EEPROMSMID == 0x84B5) ||
			(pHalData->EEPROMSVID == 0x1A32 && pHalData->EEPROMSMID == 0x2315) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7611) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8200) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8201) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8202) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8205) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8195) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8150) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9176) || // <= Start of 88CE Combo
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9181) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9182) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9184) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9185) ||
			(pHalData->EEPROMSVID == 0x103C && pHalData->EEPROMSMID == 0x169F) ||
			(pHalData->EEPROMSVID == 0x103C && pHalData->EEPROMSMID == 0x1786) ||
			(pHalData->EEPROMSVID == 0x10CF && pHalData->EEPROMSMID == 0x16B3) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x9186) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x9187) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9191) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9192) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9193) ||
			(pHalData->EEPROMSVID == 0x1A3B && pHalData->EEPROMSMID == 0x2057) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9194) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9196) ||
			(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x9197) ||
			(pHalData->EEPROMSVID == 0x1462 && pHalData->EEPROMSMID == 0x3874) ||
			(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x9198) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9201) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9202) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9203) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9204) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9195) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9199) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9200) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9150)))	||
		((pHalData->EEPROMDID==0x8177)) ||
		((pHalData->EEPROMDID==0x8178) && (
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8178) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x8179) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x8180) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8186) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8191) ||
			(pHalData->EEPROMSVID == 0x1A3B && pHalData->EEPROMSMID == 0x1178) ||
			(pHalData->EEPROMSVID == 0x1043 && pHalData->EEPROMSMID == 0x84B6) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8189) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7622) ||
			(pHalData->EEPROMSVID == 0x1B9A && pHalData->EEPROMSMID == 0x1400) ||
			(pHalData->EEPROMSVID == 0x1B9A && pHalData->EEPROMSMID == 0x1401) ||
			(pHalData->EEPROMSVID == 0x1B9A && pHalData->EEPROMSMID == 0x1402) ||
			(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9178) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x9179) ||
			(pHalData->EEPROMSVID == 0x1025 && pHalData->EEPROMSMID == 0x9180)))	||
		((pHalData->EEPROMDID == 0x8723)))
	{
		return _TRUE;
	}
	else
	{
		return _FALSE;
	}

}

static VOID
hal_CustomizedBehavior_8192CE(
	PADAPTER			Adapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);	
	EEPROM_EFUSE_PRIV	*pEEPROM = GET_EEPROM_EFUSE_PRIV(Adapter);
	struct led_priv	*pledpriv = &(Adapter->ledpriv);

	pledpriv->LedStrategy = SW_LED_MODE7; //Default LED strategy.
	pHalData->bLedOpenDrain = _TRUE;// Support Open-drain arrangement for controlling the LED. Added by Roger, 2009.10.16.
	
	switch(pHalData->CustomerID)
	{

		case RT_CID_DEFAULT:
			break;

		case RT_CID_819x_SAMSUNG:
			//pMgntInfo->bAutoConnectEnable = FALSE;
			//pMgntInfo->bForcedShowRateStill = TRUE;
			break;

		case RT_CID_TOSHIBA:
			pHalData->CurrentChannel = 10;
			//pHalData->EEPROMRegulatory = 1;
			break;

		case RT_CID_CCX:
			//pMgntInfo->IndicateByDeauth = _TRUE;
			break;

		case RT_CID_819x_Lenovo:
			// Customize Led mode	
			pledpriv->LedStrategy = SW_LED_MODE7;
			// Customize  Link any for auto connect
			// This Value should be set after InitializeMgntVariables
			//pMgntInfo->bAutoConnectEnable = FALSE;
			//pMgntInfo->pHTInfo->RxReorderPendingTime = 50;
			DBG_8192C("RT_CID_819x_Lenovo \n");

			if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8200) ||
				(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9199) )
			{
				pledpriv->LedStrategy = SW_LED_MODE9;
			}
			else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8201) ||
					(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8202) ||
					(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9200))
			{
				pledpriv->LedStrategy = SW_LED_MODE7;
			}
			else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8205)||
					(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9204))
			{
				pledpriv->LedStrategy = SW_LED_MODE7;
				pHalData->bLedOpenDrain = _FALSE;
			}
			else
			{
				pledpriv->LedStrategy = SW_LED_MODE7;
			}
			break;

		case RT_CID_819x_QMI:
			pledpriv->LedStrategy = SW_LED_MODE8; // Customize Led mode	
			break;			

		case RT_CID_819x_HP:
			pledpriv->LedStrategy = SW_LED_MODE7; // Customize Led mode	
			break;

		case RT_CID_819x_Acer:
			break;			

		case RT_CID_819x_Edimax_ASUS:
			pledpriv->LedStrategy = SW_LED_MODE10; // Customize Led mode	
			break;				

		case RT_CID_WHQL:
			//Adapter->bInHctTest = TRUE;
			break;

		case RT_CID_819x_PRONETS:
			pledpriv->LedStrategy = SW_LED_MODE9; // Customize Led mode	
			break;			
	
		default:
			MSG_8192C("Unkown hardware Type \n");
			break;
	}
	MSG_8192C("hal_CustomizedBehavior_8192CE(): RT Customized ID: 0x%02X\n", pHalData->CustomerID);

	if((pEEPROM->bautoload_fail_flag) || (!Check11nProductID(Adapter)))
	{
		if(pHalData->CurrentWirelessMode != WIRELESS_MODE_B)
			pHalData->CurrentWirelessMode = WIRELESS_MODE_G;
	}
}

static VOID
hal_CustomizeByCustomerID_92CE(
	IN	PADAPTER		pAdapter
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);

	// If the customer ID had been changed by registry, do not cover up by EEPROM.
	if(pHalData->CustomerID == RT_CID_DEFAULT)
	{
		switch(pHalData->EEPROMCustomerID)
		{	
			case EEPROM_CID_DEFAULT:
				if(pHalData->EEPROMDID==0x8176)
				{
					if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6151) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6152) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6154) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6155) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6177) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6178) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6179) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6180) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7151) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7152) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7154) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7155) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7177) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7178) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7179) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7180) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8151) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8152) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8154) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8155) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8181) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8185) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8219) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8207) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8208) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8209) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8210) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8220) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8211) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8212) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8213) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8214) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8221) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8215) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8216) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8217) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8218) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8222) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9151) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9152) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9154) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9155) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9181) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9185) )
							pHalData->CustomerID = RT_CID_TOSHIBA;
					else if(pHalData->EEPROMSVID == 0x1025)
							pHalData->CustomerID = RT_CID_819x_Acer;	
					else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6191) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6192) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6193) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7191) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7192) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7193) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8191) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8192) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8193) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9191) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9192) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9193) )
							pHalData->CustomerID = RT_CID_819x_SAMSUNG;
					else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8195) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9195) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7194) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8200) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8201) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8202) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8205) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9199) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9200) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9204))
							pHalData->CustomerID = RT_CID_819x_Lenovo;
					else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8197) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9196) )
							pHalData->CustomerID = RT_CID_819x_CLEVO;
					else if((pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x8194) ||
							(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x8198) ||
							(pHalData->EEPROMSVID == 0x185F && pHalData->EEPROMSMID == 0x8176) ||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8206) ||
							(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x9197) ||
							(pHalData->EEPROMSVID == 0x1028 && pHalData->EEPROMSMID == 0x9198))
							pHalData->CustomerID = RT_CID_819x_DELL;
					else if((pHalData->EEPROMSVID == 0x103C && pHalData->EEPROMSMID == 0x1629) ||// HP LiteOn
							(pHalData->EEPROMSVID == 0x103C && pHalData->EEPROMSMID == 0x1786) )
			 				pHalData->CustomerID = RT_CID_819x_HP;
					else if((pHalData->EEPROMSVID == 0x1A32 && pHalData->EEPROMSMID == 0x2315))// QMI
							pHalData->CustomerID = RT_CID_819x_QMI;	
					else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8203))// QMI				
							pHalData->CustomerID = RT_CID_819x_PRONETS;
					else if((pHalData->EEPROMSVID == 0x1043 && pHalData->EEPROMSMID == 0x84B5)||
							(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7611))// ASUS
						pHalData->CustomerID = RT_CID_819x_Edimax_ASUS;					
					else
						pHalData->CustomerID = RT_CID_DEFAULT;
				}
				else if(pHalData->EEPROMDID==0x8178)
				{
					if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6181) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x6185) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7181) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7185) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8181) ||						
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8185) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9181) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9182) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9184) ||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x9185) )
							pHalData->CustomerID = RT_CID_TOSHIBA;
					else if(pHalData->EEPROMSVID == 0x1025)
						pHalData->CustomerID = RT_CID_819x_Acer;
					else if((pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x8186))// PRONETS					
						pHalData->CustomerID = RT_CID_819x_PRONETS;
					else if((pHalData->EEPROMSVID == 0x1043 && pHalData->EEPROMSMID == 0x84B6)||
						(pHalData->EEPROMSVID == 0x10EC && pHalData->EEPROMSMID == 0x7622))// ASUS				
						pHalData->CustomerID = RT_CID_819x_Edimax_ASUS;						
					else
						pHalData->CustomerID = RT_CID_DEFAULT;
				}
				else
				{
					pHalData->CustomerID = RT_CID_DEFAULT;
				}
				break;
				
			case EEPROM_CID_TOSHIBA:       
				pHalData->CustomerID = RT_CID_TOSHIBA;
				break;

			case EEPROM_CID_CCX:
				pHalData->CustomerID = RT_CID_CCX;
				break;

			case EEPROM_CID_QMI:
				pHalData->CustomerID = RT_CID_819x_QMI;
				break;
				
			case EEPROM_CID_WHQL:
				/*pAdapter->bInHctTest = TRUE;
				
				pMgntInfo->bSupportTurboMode = FALSE;
				pMgntInfo->bAutoTurboBy8186 = FALSE;
				pMgntInfo->PowerSaveControl.bInactivePs = FALSE;
				pMgntInfo->PowerSaveControl.bIPSModeBackup = FALSE;
				pMgntInfo->PowerSaveControl.bLeisurePs = FALSE;
				pMgntInfo->PowerSaveControl.bLeisurePsModeBackup = FALSE;
				pMgntInfo->keepAliveLevel = 0;	
				pAdapter->bUnloadDriverwhenS3S4 = FALSE;*/
				break;
					
			default:
				pHalData->CustomerID = RT_CID_DEFAULT;
				break;
					
		}
	}
	//MSG_8192C("MGNT Customer ID: 0x%2x\n", pHalData->CustomerID);
	
	hal_CustomizedBehavior_8192CE(pAdapter);
}


//
//	Description:
//		Config HW adapter information into initial value.
//
//	Assumption:
//		1. After Auto load fail(i.e, check CR9346 fail)
//
//	Created by Roger, 2008.10.21.
//
static	VOID
ConfigAdapterInfo8192CForAutoLoadFail(
	IN PADAPTER			Adapter
)
{
	EEPROM_EFUSE_PRIV	*pEEPROM = GET_EEPROM_EFUSE_PRIV(Adapter);
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	u16	i;
	u8	sMacAddr[6] = {0x00, 0xE0, 0x4C, 0x81, 0x92, 0x00};
	u8	*hwinfo = 0;

	DBG_8192C("====> ConfigAdapterInfo8192CForAutoLoadFail\n");

	// Initialize IC Version && Channel Plan
	pHalData->EEPROMVID = 0;
	pHalData->EEPROMDID = 0;
	DBG_8192C("EEPROM VID = 0x%4x\n", pHalData->EEPROMVID);
	DBG_8192C("EEPROM DID = 0x%4x\n", pHalData->EEPROMDID);	
	
	pHalData->EEPROMChannelPlan = 0;
	DBG_8192C("EEPROM ChannelPlan = 0x%4x\n", pHalData->EEPROMChannelPlan);
	rtl8192c_EfuseParseChnlPlan(Adapter, hwinfo, _TRUE);

	pHalData->EEPROMCustomerID = 0;
	DBG_8192C("EEPROM Customer ID: 0x%2x\n", pHalData->EEPROMCustomerID);
	hal_CustomizeByCustomerID_92CE(Adapter);

       //
	//<Roger_Notes> In this case, we random assigh MAC address here. 2008.10.15.
	//
	//Initialize Permanent MAC address
	//if(!Adapter->bInHctTest)
       // sMacAddr[5] = (u1Byte)GetRandomNumber(1, 254);	
	for(i = 0; i < 6; i++)
		pEEPROM->mac_addr[i] = sMacAddr[i];
	
	//NicIFSetMacAddress(Adapter, Adapter->PermanentAddress);

	DBG_8192C("Permanent Address = %02x-%02x-%02x-%02x-%02x-%02x\n", 
	pEEPROM->mac_addr[0], pEEPROM->mac_addr[1], 
	pEEPROM->mac_addr[2], pEEPROM->mac_addr[3], 
	pEEPROM->mac_addr[4], pEEPROM->mac_addr[5]);
	
	//
	// Read tx power index from efuse or eeprom
	//
	readTxPowerInfo_8192C(Adapter, pEEPROM->bautoload_fail_flag, hwinfo);
	//
	// Read Bluetooth co-exist and initialize
	//
#ifdef CONFIG_BT_COEXIST
	//ReadBluetoothCoexistInfoFromHWPG(Adapter, pEEPROM->bautoload_fail_flag, hwinfo);
	rtl8192c_ReadBluetoothCoexistInfo(Adapter, hwinfo, pEEPROM->bautoload_fail_flag);
#endif

	pHalData->EEPROMVersion = 1;		// Default version is 1
	pHalData->bTXPowerDataReadFromEEPORM = _FALSE;
			
	pHalData->EEPROMBoardType = EEPROM_Default_BoardType;	
	DBG_8192C("BoardType = %#x\n", pHalData->EEPROMBoardType);

	DBG_8192C("<==== ConfigAdapterInfo8192CForAutoLoadFail\n");
}


static VOID
readAdapterInfo_8723E(
	IN PADAPTER			Adapter,
	IN	BOOLEAN			bPseudoTest
	)
{
/*	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8				hwinfo[HWSET_MAX_SIZE];

	RT_TRACE(COMP_INIT, DBG_LOUD, ("====> readAdapterInfo_8723E()\n"));

	//
	// This part read and parse the eeprom/efuse content
	//
	hal_InitPGData(Adapter, hwinfo, bPseudoTest);
	Hal_EfuseParseIDCode(Adapter, hwinfo);
	hal_EfuseParsePIDVID_8723E(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	hal_EfuseParseMACAddr_8723E(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseTxPowerInfo_8723(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseBTCoexistInfo8723(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseEEPROMVer(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseChnlPlan(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseCustomerID(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseAntennaDiversity(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	Hal_EfuseParseRateIndicationOption(Adapter, hwinfo, pHalData->AutoloadFailFlag);
	hal_EfuseParseBoardType_8723E(Adapter, hwinfo, pHalData->AutoloadFailFlag);

	//
	// The following part initialize some vars by PG info.
	//
	Hal_InitChannelPlan(Adapter);
	hal_CustomizeByCustomerID_8723E(Adapter);

	RT_TRACE(COMP_INIT, DBG_LOUD, ("<==== readAdapterInfo_8723E()\n"));*/
}

static	VOID
_ReadAdapterInfo8192CE(
	IN PADAPTER			Adapter
	)
{
	EEPROM_EFUSE_PRIV	*pEEPROM = GET_EEPROM_EFUSE_PRIV(Adapter);
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	u16			i,usValue;
	u16			EEPROMId;
	u8			tempval;
	u8			hwinfo[HWSET_MAX_SIZE];

	//DBG_8192C("====> ReadAdapterInfo8192CE\n");

	if (pEEPROM->EepromOrEfuse == EEPROM_93C46)
	{	// Read frin EEPROM
		//DBG_8192C("EEPROM\n");
	}
	else if (pEEPROM->EepromOrEfuse == EEPROM_BOOT_EFUSE)
	{	// Read from EFUSE	
		//DBG_8192C("EFUSE\n");

		// Read EFUSE real map to shadow!!
		EFUSE_ShadowMapUpdate(Adapter, EFUSE_WIFI, _FALSE);

		_rtw_memcpy((void*)hwinfo, (void*)pEEPROM->efuse_eeprom_data, HWSET_MAX_SIZE);
	}

	// Print current HW setting map!!
	//RT_PRINT_DATA(COMP_INIT, DBG_LOUD, ("MAP \n"), hwinfo, HWSET_MAX_SIZE);			

	// Checl 0x8129 again for making sure autoload status!!
	EEPROMId = le16_to_cpu(*((u16 *)&hwinfo[0]));
	if( EEPROMId != RTL8190_EEPROM_ID )
	{
		DBG_8192C("EEPROM ID(%#x) is invalid!!\n", EEPROMId);
		pEEPROM->bautoload_fail_flag = _TRUE;
	}
	else
	{
		//DBG_8192C("Autoload OK\n");
		pEEPROM->bautoload_fail_flag = _FALSE;
	}	

	//
	if (pEEPROM->bautoload_fail_flag == _TRUE)
	{
		ConfigAdapterInfo8192CForAutoLoadFail(Adapter);
		return;
	}

	// VID, DID	 SE	0xA-D
	pHalData->EEPROMVID		=le16_to_cpu( *(u16 *)&hwinfo[EEPROM_VID]);
	pHalData->EEPROMDID		=le16_to_cpu( *(u16 *)&hwinfo[EEPROM_DID]);
	pHalData->EEPROMSVID	=le16_to_cpu(*(u16 *)&hwinfo[EEPROM_SVID]);
	pHalData->EEPROMSMID	=le16_to_cpu(*(u16 *)&hwinfo[EEPROM_SMID]);

	DBG_8192C("EEPROMId = 0x%4x\n", EEPROMId);
	DBG_8192C("EEPROM VID = 0x%4x\n", pHalData->EEPROMVID);
	DBG_8192C("EEPROM DID = 0x%4x\n", pHalData->EEPROMDID);
	DBG_8192C("EEPROM SVID = 0x%4x\n", pHalData->EEPROMSVID);
	DBG_8192C("EEPROM SMID = 0x%4x\n", pHalData->EEPROMSMID);


	//Read Permanent MAC address
/*
	for(i = 0; i < 6; i += 2)
	{
		usValue = *(u16 *)&hwinfo[EEPROM_MAC_ADDR+i];
		*((u16 *)(&pEEPROM->mac_addr[i])) = usValue;
	}
*/
	_rtw_memcpy(pEEPROM->mac_addr,(u8 *)&hwinfo[EEPROM_MAC_ADDR],MAC_ADDR_LEN	 );
	//NicIFSetMacAddress(Adapter, Adapter->PermanentAddress);

	DBG_8192C("ReadAdapterInfo8192CE(), Permanent Address = %02x-%02x-%02x-%02x-%02x-%02x\n", 
	pEEPROM->mac_addr[0], pEEPROM->mac_addr[1], 
	pEEPROM->mac_addr[2], pEEPROM->mac_addr[3], 
	pEEPROM->mac_addr[4], pEEPROM->mac_addr[5]); 

	//
	// Read tx power index from efuse or eeprom
	//
	readTxPowerInfo_8192C(Adapter, pEEPROM->bautoload_fail_flag, hwinfo);

	//
	// Read Bluetooth co-exist and initialize
	//
#ifdef CONFIG_BT_COEXIST
	//ReadBluetoothCoexistInfoFromHWPG(Adapter, pEEPROM->bautoload_fail_flag, hwinfo);
	rtl8192c_ReadBluetoothCoexistInfo(Adapter, hwinfo, pEEPROM->bautoload_fail_flag);
#endif
	
	//
	// Read IC Version && Channel Plan
	//
	// Version ID, Channel plan
	rtl8192c_EfuseParseChnlPlan(Adapter, hwinfo, pEEPROM->bautoload_fail_flag);

	pHalData->EEPROMVersion  = le16_to_cpu(*(u16 *)&hwinfo[EEPROM_VERSION]);
	pHalData->bTXPowerDataReadFromEEPORM = _TRUE;

	pHalData->EEPROMCustomerID = *(u8 *)&hwinfo[EEPROM_CUSTOMER_ID];
	DBG_8192C("EEPROM Customer ID: 0x%2x\n", pHalData->EEPROMCustomerID);

	hal_CustomizeByCustomerID_92CE(Adapter);

	// Antenna Diversity setting. 
	if(Adapter->registrypriv.antdiv_cfg == 2) // 2: From Efuse
		pHalData->AntDivCfg = (hwinfo[EEPROM_RF_OPT1]&0x18)>>3;
	else
		pHalData->AntDivCfg = Adapter->registrypriv.antdiv_cfg;  // 0:OFF , 1:ON,

	DBG_8192C("SWAS: bHwAntDiv = %x\n", pHalData->AntDivCfg);

	pHalData->InterfaceSel = (INTERFACE_SELECT_8192CPCIe)(hwinfo[EEPROM_RF_OPT1]&0xE0)>>5;
	DBG_8192C("Board Type: 0x%x\n", pHalData->InterfaceSel);

	// 2008/01/16 MH We can only know RF type in the function. So we have to init
	// DIG RATR table again.
	//InitRateAdaptive(Adapter);
	
	//DBG_8192C("<==== ReadAdapterInfo8192CE\n");
}

static int _ReadAdapterInfo8192C(PADAPTER Adapter)
{
	EEPROM_EFUSE_PRIV	*pEEPROM = GET_EEPROM_EFUSE_PRIV(Adapter);
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	struct mlme_priv		*pmlmepriv = &(Adapter->mlmepriv);
	struct registry_priv	*pregistrypriv = &Adapter->registrypriv;
	u8	tmpU1b;
	u32	value32;

	//DBG_8192C("====> ReadAdapterInfo8192C\n");

	if(IS_HARDWARE_TYPE_8723A(Adapter)/* && 
		INCLUDE_MULTI_FUNC_BT(Adapter)*/)
	{		
		value32 = rtw_read32(Adapter, EFUSE_TEST);
		value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
		rtw_write32(Adapter, EFUSE_TEST, value32);		
	}

	// For debug test now!!!!!
	//PHY_RFShadowRefresh(Adapter);

	// Read IC Version && Channel Plan	
	//pHalData->VersionID = rtl8192c_ReadChipVersion(Adapter);
	pHalData->rf_chip = RF_6052;
#if 0 //move this to initializeadapter
	if (pHalData->rf_type == RF_1T1R)
		pHalData->bRFPathRxEnable[0] = _TRUE;
	else
		pHalData->bRFPathRxEnable[0] = pHalData->bRFPathRxEnable[1] = _TRUE;
#endif
	DBG_8192C("VersionID = 0x%4x\n", pHalData->VersionID);

	tmpU1b = rtw_read8(Adapter, REG_9346CR);

	if (tmpU1b & BIT4)
	{
		DBG_8192C("Boot from EEPROM\n");
		pEEPROM->EepromOrEfuse = EEPROM_93C46;
	}
	else 
	{
		DBG_8192C("Boot from EFUSE\n");
		pEEPROM->EepromOrEfuse = EEPROM_BOOT_EFUSE;
	}

	// Autoload OK
	if (tmpU1b & BIT5)
	{
		DBG_8192C("Autoload OK\n");
		pEEPROM->bautoload_fail_flag = _FALSE;
		if(IS_HARDWARE_TYPE_8723A(Adapter))
			readAdapterInfo_8723E(Adapter, _FALSE);
		else
			_ReadAdapterInfo8192CE(Adapter);
	}	
	else
	{ 	// Auto load fail.		
		DBG_8192C("AutoLoad Fail reported from CR9346!!\n");
		pEEPROM->bautoload_fail_flag = _TRUE;
		if(IS_HARDWARE_TYPE_8723A(Adapter))
			readAdapterInfo_8723E(Adapter, _FALSE);
		else
		{
			ConfigAdapterInfo8192CForAutoLoadFail(Adapter);
			if (pEEPROM->EepromOrEfuse == EEPROM_BOOT_EFUSE)
				EFUSE_ShadowMapUpdate(Adapter, EFUSE_WIFI, _FALSE);
		}
	}

	//DBG_8192C("<==== ReadAdapterInfo8192C\n");

	return _SUCCESS;
}

static void ReadAdapterInfo8192CE(PADAPTER Adapter)
{
	// Read EEPROM size before call any EEPROM function
	//Adapter->EepromAddressSize=Adapter->HalFunc.GetEEPROMSizeHandler(Adapter);
	Adapter->EepromAddressSize = GetEEPROMSize8192C(Adapter);
	
	_ReadAdapterInfo8192C(Adapter);
}


void rtl8192ce_interface_configure(_adapter *padapter)
{
	struct dvobj_priv	*pdvobjpriv = &padapter->dvobjpriv;
	struct pwrctrl_priv	*pwrpriv = &padapter->pwrctrlpriv;

_func_enter_;

	////close ASPM for AMD defaultly
	pdvobjpriv->const_amdpci_aspm = 0;

	//// ASPM PS mode.
	//// 0 - Disable ASPM, 1 - Enable ASPM without Clock Req, 
	//// 2 - Enable ASPM with Clock Req, 3- Alwyas Enable ASPM with Clock Req,
	//// 4-  Always Enable ASPM without Clock Req.
	//// set defult to RTL8192CE:3 RTL8192E:2
	pdvobjpriv->const_pci_aspm = 0;

	//// Setting for PCI-E device */
	pdvobjpriv->const_devicepci_aspm_setting = 0x03;

	//// Setting for PCI-E bridge */
	pdvobjpriv->const_hostpci_aspm_setting = 0x02;

	//// In Hw/Sw Radio Off situation.
	//// 0 - Default, 1 - From ASPM setting without low Mac Pwr, 
	//// 2 - From ASPM setting with low Mac Pwr, 3 - Bus D3
	//// set default to RTL8192CE:0 RTL8192SE:2
	pdvobjpriv->const_hwsw_rfoff_d3 = 0;

	//// This setting works for those device with backdoor ASPM setting such as EPHY setting.
	//// 0: Not support ASPM, 1: Support ASPM, 2: According to chipset.
	pdvobjpriv->const_support_pciaspm = 1;

	pwrpriv->reg_rfoff = 0;
	pwrpriv->rfoff_reason = 0;

_func_exit_;
}

VOID
DisableInterrupt8192CE (
	IN PADAPTER			Adapter
	)
{
	struct dvobj_priv	*pdvobjpriv=&Adapter->dvobjpriv;
#ifdef CONFIG_CONCURRENT_MODE
	// Aries add, 20120314
        if ((Adapter->isprimary == _FALSE) && Adapter->pbuddy_adapter){
		Adapter = Adapter->pbuddy_adapter;
		pdvobjpriv=&Adapter->dvobjpriv;
	}
#endif
	// Because 92SE now contain two DW IMR register range.
	rtw_write32(Adapter, REG_HIMR, IMR8190_DISABLED);	
	//RT_TRACE(COMP_INIT,DBG_LOUD,("***DisableInterrupt8192CE.\n"));
	// From WMAC code
	//PlatformEFIOWrite4Byte(Adapter, REG_HIMR+4,IMR, IMR8190_DISABLED);
	//RTPRINT(FISR, ISR_CHK, ("Disable IMR=%x"));

	rtw_write32(Adapter, REG_HIMRE, IMR8190_DISABLED);	// by tynli

	pdvobjpriv->irq_enabled = 0;

}

VOID
ClearInterrupt8192CE(
	IN PADAPTER			Adapter
	)
{
	u32	tmp = 0;
	
#ifdef CONFIG_CONCURRENT_MODE
	// Aries add, 20120314
        if ((Adapter->isprimary == _FALSE) && Adapter->pbuddy_adapter){
		Adapter = Adapter->pbuddy_adapter;
	}
#endif	
	tmp = rtw_read32(Adapter, REG_HISR);	
	rtw_write32(Adapter, REG_HISR, tmp);	

	tmp = 0;
	tmp = rtw_read32(Adapter, REG_HISRE);	
	rtw_write32(Adapter, REG_HISRE, tmp);	
}


VOID
EnableInterrupt8192CE(
	IN PADAPTER			Adapter
	)
{
	HAL_DATA_TYPE	*pHalData=GET_HAL_DATA(Adapter);
	struct dvobj_priv	*pdvobjpriv=&Adapter->dvobjpriv;
#ifdef CONFIG_CONCURRENT_MODE
	// Aries add, 20120314
        if ((Adapter->isprimary == _FALSE) && Adapter->pbuddy_adapter){
		Adapter = Adapter->pbuddy_adapter;
		pHalData=GET_HAL_DATA(Adapter);
		pdvobjpriv=&Adapter->dvobjpriv;
	}
#endif
	pdvobjpriv->irq_enabled = 1;

	pHalData->IntrMask[0] = pHalData->IntrMaskToSet[0];
	pHalData->IntrMask[1] = pHalData->IntrMaskToSet[1];

	rtw_write32(Adapter, REG_HIMR, pHalData->IntrMask[0]&0xFFFFFFFF);

	rtw_write32(Adapter, REG_HIMRE, pHalData->IntrMask[1]&0xFFFFFFFF);

}

void
InterruptRecognized8192CE(
	IN	PADAPTER			Adapter,
	OUT	PRT_ISR_CONTENT	pIsrContent
	)
{
	HAL_DATA_TYPE	*pHalData=GET_HAL_DATA(Adapter);
#ifdef CONFIG_CONCURRENT_MODE
	// Aries add, 20120314
        if ((Adapter->isprimary == _FALSE) && Adapter->pbuddy_adapter){
		Adapter = Adapter->pbuddy_adapter;
		pHalData=GET_HAL_DATA(Adapter);
	}
#endif
	pIsrContent->IntArray[0] = rtw_read32(Adapter, REG_HISR);	
	pIsrContent->IntArray[0] &= pHalData->IntrMask[0];
	rtw_write32(Adapter, REG_HISR, pIsrContent->IntArray[0]);

	//For HISR extension. Added by tynli. 2009.10.07.
	pIsrContent->IntArray[1] = rtw_read32(Adapter, REG_HISRE);	
	pIsrContent->IntArray[1] &= pHalData->IntrMask[1];
	rtw_write32(Adapter, REG_HISRE, pIsrContent->IntArray[1]);

}

VOID
UpdateInterruptMask8192CE(
	IN	PADAPTER		Adapter,
	IN	u32		AddMSR, 	u32		AddMSR1,
	IN	u32		RemoveMSR, u32		RemoveMSR1
	)
{
	HAL_DATA_TYPE	*pHalData=GET_HAL_DATA(Adapter);
#ifdef CONFIG_CONCURRENT_MODE
	// Aries add, 20120314
        if ((Adapter->isprimary == _FALSE) && Adapter->pbuddy_adapter){
		Adapter = Adapter->pbuddy_adapter;
		pHalData=GET_HAL_DATA(Adapter);
	}
#endif
	if( AddMSR )
	{
		pHalData->IntrMaskToSet[0] |= AddMSR;
	}
	if( AddMSR1 )
	{
		pHalData->IntrMaskToSet[1] |= AddMSR1;
	}
	
	if( RemoveMSR )
	{
		pHalData->IntrMaskToSet[0] &= (~RemoveMSR);
	}

	if( RemoveMSR1 )
	{
		pHalData->IntrMaskToSet[1] &= (~RemoveMSR1);
	}
	
	DisableInterrupt8192CE( Adapter );
	EnableInterrupt8192CE( Adapter );
}

 static VOID
_InitBeaconParameters(
	IN  PADAPTER Adapter
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	rtw_write16(Adapter, REG_BCN_CTRL, 0x1010);

	// TODO: Remove these magic number
	rtw_write16(Adapter, REG_TBTT_PROHIBIT,0x6404);// ms

	rtw_write8(Adapter, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME);// 5ms
	rtw_write8(Adapter, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME); // 2ms

	// Suggested by designer timchen. Change beacon AIFS to the largest number
	// beacause test chip does not contension before sending beacon. by tynli. 2009.11.03
	if(IS_NORMAL_CHIP( pHalData->VersionID)){
		rtw_write16(Adapter, REG_BCNTCFG, 0x660F);
	}
	else{		
		rtw_write16(Adapter, REG_BCNTCFG, 0x66FF);
	}

}

static VOID
HwConfigureRTL8192CE(
		IN	PADAPTER			Adapter
		)
{

	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8	regBwOpMode = 0;
	u32	regRATR = 0, regRRSR = 0;


	//1 This part need to modified according to the rate set we filtered!!
	//
	// Set RRSR, RATR, and BW_OPMODE registers
	//
	switch(pHalData->CurrentWirelessMode)
	{
	case WIRELESS_MODE_B:
		regBwOpMode = BW_OPMODE_20MHZ;
		regRATR = RATE_ALL_CCK;
		regRRSR = RATE_ALL_CCK;
		break;
	case WIRELESS_MODE_A:
		regBwOpMode = BW_OPMODE_5G |BW_OPMODE_20MHZ;
		regRATR = RATE_ALL_OFDM_AG;
		regRRSR = RATE_ALL_OFDM_AG;
		break;
	case WIRELESS_MODE_G:
		regBwOpMode = BW_OPMODE_20MHZ;
		regRATR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
		regRRSR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
		break;
	case WIRELESS_MODE_UNKNOWN:
	case WIRELESS_MODE_AUTO:
	case WIRELESS_MODE_N_24G:
		// It support CCK rate by default.
		// CCK rate will be filtered out only when associated AP does not support it.
		regBwOpMode = BW_OPMODE_20MHZ;
		regRATR = RATE_ALL_CCK | RATE_ALL_OFDM_AG | RATE_ALL_OFDM_1SS | RATE_ALL_OFDM_2SS;
		regRRSR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
		break;
	case WIRELESS_MODE_N_5G:
		regBwOpMode = BW_OPMODE_5G;
		regRATR = RATE_ALL_OFDM_AG | RATE_ALL_OFDM_1SS | RATE_ALL_OFDM_2SS;
		regRRSR = RATE_ALL_OFDM_AG;
		break;
	default:
		break;
	}

	rtw_write8(Adapter, REG_INIRTS_RATE_SEL, 0x8);

	// 2007/02/07 Mark by Emily becasue we have not verify whether this register works	
	//For 92C,which reg?
	rtw_write8(Adapter, REG_BWOPMODE, regBwOpMode);


	// Init value for RRSR.
	rtw_write32(Adapter, REG_RRSR, regRRSR);

	// Set SLOT time
	rtw_write8(Adapter,REG_SLOT, 0x09);

	// Set AMPDU min space
	rtw_write8(Adapter,REG_AMPDU_MIN_SPACE, 0x0);

	// CF-End setting.
	rtw_write16(Adapter,REG_FWHW_TXQ_CTRL, 0x1F80);

	// Set retry limit
	rtw_write16(Adapter,REG_RL, 0x0707);

	// BAR settings
	rtw_write32(Adapter, REG_BAR_MODE_CTRL, 0x0201ffff);

	// HW SEQ CTRL
	rtw_write8(Adapter,REG_HWSEQ_CTRL, 0xFF); //set 0x0 to 0xFF by tynli. Default enable HW SEQ NUM.

	// Set Data / Response auto rate fallack retry count
	rtw_write32(Adapter, REG_DARFRC, 0x01000000);
	rtw_write32(Adapter, REG_DARFRC+4, 0x07060504);
	rtw_write32(Adapter, REG_RARFRC, 0x01000000);
	rtw_write32(Adapter, REG_RARFRC+4, 0x07060504);

#if 0//cosa, for 92s
	if(	(pHalData->bt_coexist.BluetoothCoexist) &&
		(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4) )
	{
		PlatformEFIOWrite4Byte(Adapter, REG_AGGLEN_LMT, 0x97427431);
		RTPRINT(FBT, BT_TRACE, ("BT write 0x%x = 0x97427431\n", REG_AGGLEN_LMT));
	}
	else
#endif
	{
		// Aggregation threshold
		rtw_write32(Adapter, REG_AGGLEN_LMT, 0xb972a841);
	}
	
	// Beacon related, for rate adaptive
	rtw_write8(Adapter, REG_ATIMWND, 0x2);
#if 0 // Just set 0x55D to default 0xff. Suggested by TimChen. Marked by tynli.
	if(IS_NORMAL_CHIP(pHalData->VersionID) )
	{
		// Change  0xff to 0x0A. Advised by TimChen. 2009.01.25. by tynli.
		PlatformEFIOWrite1Byte(Adapter, REG_BCN_MAX_ERR, 0x0a);
	}
	else
#endif
	{
		// For client mode and ad hoc mode TSF setting
		rtw_write8(Adapter, REG_BCN_MAX_ERR, 0xff);
	}

	// 20100211 Joseph: Change original setting of BCN_CTRL(0x550) from 
	// 0x1e(0x2c for test chip) ro 0x1f(0x2d for test chip). Set BIT0 of this register disable ATIM
	// function. Since we do not use HIGH_QUEUE anymore, ATIM function is no longer used.
	// Also, enable ATIM function may invoke HW Tx stop operation. This may cause ping failed
	// sometimes in long run test. So just disable it now.
	if(IS_81XXC_TEST_CHIP(pHalData->VersionID))
		//PlatformAtomicExchange((pu4Byte)(&pHalData->RegBcnCtrlVal), 0x2d);
		pHalData->RegBcnCtrlVal = 0x2d;
	else
	{
		//PlatformAtomicExchange((pu4Byte)(&pHalData->RegBcnCtrlVal), 0x1d);
		pHalData->RegBcnCtrlVal = 0x1d;
	}
#ifdef CONFIG_CONCURRENT_MODE
	rtw_write16(Adapter, REG_BCN_CTRL, 0x1010);	// For 2 PORT TSF SYNC
#else
	rtw_write8(Adapter, REG_BCN_CTRL, (u8)(pHalData->RegBcnCtrlVal));
#endif
	// Marked out by Bruce, 2010-09-09.
	// This register is configured for the 2nd Beacon (multiple BSSID). 
	// We shall disable this register if we only support 1 BSSID.
	rtw_write8(Adapter, REG_BCN_CTRL_1, 0);

	// TBTT prohibit hold time. Suggested by designer TimChen.
	rtw_write8(Adapter, REG_TBTT_PROHIBIT+1,0xff); // 8 ms

	// 20091211 Joseph: Do not set 0x551[1] suggested by Scott.
	// 
	// Disable BCNQ SUB1 0x551[1]. Suggested by TimChen. 2009.12.04. by tynli.
	// For protecting HW to decrease the TSF value when temporarily the real TSF value 
	// is smaller than the TSF counter.
	//regTmp = PlatformEFIORead1Byte(Adapter, REG_USTIME_TSF);
	//PlatformEFIOWrite1Byte(Adapter, REG_USTIME_TSF, (regTmp|BIT1)); // 8 ms

	rtw_write8(Adapter, REG_PIFS, 0x1C);
	rtw_write8(Adapter, REG_AGGR_BREAK_TIME, 0x16);

#if 0//cosa, for 92s
	if(	(pHalData->bt_coexist.BluetoothCoexist) &&
		(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4) )
	{
		PlatformEFIOWrite2Byte(Adapter, REG_NAV_PROT_LEN, 0x0020);
		RTPRINT(FBT, BT_TRACE, ("BT write 0x%x = 0x0020\n", REG_NAV_PROT_LEN));
		PlatformEFIOWrite2Byte(Adapter, REG_PROT_MODE_CTRL, 0x0402);
		RTPRINT(FBT, BT_TRACE, ("BT write 0x%x = 0x0402\n", REG_PROT_MODE_CTRL));
	}
	else
#endif
	{
		rtw_write16(Adapter, REG_NAV_PROT_LEN, 0x0040);
		rtw_write16(Adapter, REG_PROT_MODE_CTRL, 0x08ff);
	}

	if(!Adapter->registrypriv.wifi_spec)
	{
		//For Rx TP. Suggested by SD1 Richard. Added by tynli. 2010.04.12.
		rtw_write32(Adapter, REG_FAST_EDCA_CTRL, 0x03086666);
	}
	else
	{
		//For WiFi WMM. suggested by timchen. Added by tynli.	
		rtw_write16(Adapter, REG_FAST_EDCA_CTRL, 0x0);
	}

#if (BOARD_TYPE == FPGA_2MAC)
	// ACKTO for IOT issue.
	rtw_write8(Adapter, REG_ACKTO, 0x40);

	// Set Spec SIFS (used in NAV)
	rtw_write16(Adapter,REG_SPEC_SIFS, 0x1010);
	rtw_write16(Adapter,REG_MAC_SPEC_SIFS, 0x1010);

	// Set SIFS for CCK
	rtw_write16(Adapter,REG_SIFS_CTX, 0x1010);	

	// Set SIFS for OFDM
	rtw_write16(Adapter,REG_SIFS_TRX, 0x1010);

#else
	// ACKTO for IOT issue.
	rtw_write8(Adapter, REG_ACKTO, 0x40);

	// Set Spec SIFS (used in NAV)
	rtw_write16(Adapter,REG_SPEC_SIFS, 0x100a);
	rtw_write16(Adapter,REG_MAC_SPEC_SIFS, 0x100a);

	// Set SIFS for CCK
	rtw_write16(Adapter,REG_SIFS_CTX, 0x100a);	

	// Set SIFS for OFDM
	rtw_write16(Adapter,REG_SIFS_TRX, 0x100a);
#endif

	// Set Multicast Address. 2009.01.07. by tynli.
	rtw_write32(Adapter, REG_MAR, 0xffffffff);
	rtw_write32(Adapter, REG_MAR+4, 0xffffffff);

	//Reject all control frame - default value is 0
	rtw_write16(Adapter,REG_RXFLTMAP1,0x0);

	//Nav limit , suggest by scott
	rtw_write8(Adapter, 0x652, 0x0);

	//For 92C,how to??
	//PlatformEFIOWrite1Byte(Adapter, MLT, 0x8f);

	// Set Contention Window here		

	// Set Tx AGC

	// Set Tx Antenna including Feedback control
		
	// Set Auto Rate fallback control
				
	//
	// For Min Spacing configuration.
	//
//1 JOSEPH_REVISE
#if 0
	Adapter->MgntInfo.MinSpaceCfg = 0x90;	//cosa, asked by scott, for MCS15 short GI, padding patch, 0x237[7:3] = 0x12.
	Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_AMPDU_MIN_SPACE, (pu1Byte)(&Adapter->MgntInfo.MinSpaceCfg));	
        switch(pHalData->RF_Type)
	{
		case RF_1T2R:
		case RF_1T1R:
			RT_TRACE(COMP_INIT, DBG_LOUD, ("Initializeadapter: RF_Type%s\n", (pHalData->RF_Type==RF_1T1R? "(1T1R)":"(1T2R)")));
			Adapter->MgntInfo.MinSpaceCfg = (MAX_MSS_DENSITY_1T<<3);						
			break;
		case RF_2T2R:
		case RF_2T2R_GREEN:
			RT_TRACE(COMP_INIT, DBG_LOUD, ("Initializeadapter:RF_Type(2T2R)\n"));
			Adapter->MgntInfo.MinSpaceCfg = (MAX_MSS_DENSITY_2T<<3);			
			break;
	}
	PlatformEFIOWrite1Byte(Adapter, AMPDU_MIN_SPACE, Adapter->MgntInfo.MinSpaceCfg);
#endif 
}

static u32
_LLTWrite(
	IN  PADAPTER	Adapter,
	IN	u32		address,
	IN	u32		data
	)
{
	u32	status = _SUCCESS;
	s32	count = 0;
	u32	value = _LLT_INIT_ADDR(address) | _LLT_INIT_DATA(data) | _LLT_OP(_LLT_WRITE_ACCESS);

	rtw_write32(Adapter, REG_LLT_INIT, value);

	//polling
	do{

		value = rtw_read32(Adapter, REG_LLT_INIT	);
		if(_LLT_NO_ACTIVE == _LLT_OP_VALUE(value)){
			break;
		}

		if(count > POLLING_LLT_THRESHOLD){
			DBG_8192C("Failed to polling write LLT done at address %x!\n", address);
			status = _FAIL;
			break;
		}
	}while(++count);

	return status;

}

#define LLT_CONFIG	5
static u32
LLT_table_init(
	IN PADAPTER	Adapter
	)
{
	u16	i;
	HAL_DATA_TYPE* pHalData = GET_HAL_DATA(Adapter);
	u8	txpktbuf_bndy;
	u8	maxPage;
	u32	status;

	//UCHAR	txpktbufSz = 252; //174(0xAE) 120(0x78) 252(0xFC)
#if LLT_CONFIG == 1	// Normal
	maxPage = 255;
	txpktbuf_bndy = 252;
#elif LLT_CONFIG == 2 // 2MAC FPGA
	maxPage = 127;
	txpktbuf_bndy = 124;
#elif LLT_CONFIG == 3 // WMAC
	maxPage = 255;
	txpktbuf_bndy = 174;
#elif LLT_CONFIG == 4 // WMAC
	maxPage = 255;
	txpktbuf_bndy = 246;
#elif LLT_CONFIG == 5 // tynli_test for WiFi.
	maxPage = 255;
	txpktbuf_bndy = 246;
#endif

	// Set reserved page for each queue
	// 11.	RQPN 0x200[31:0]	= 0x80BD1C1C				// load RQPN
#if LLT_CONFIG == 1
	rtw_write8(Adapter,REG_RQPN_NPQ, 0x1c);
	rtw_write32(Adapter,REG_RQPN, 0x80a71c1c);
#elif LLT_CONFIG == 2
	rtw_write32(Adapter,REG_RQPN, 0x845B1010);
#elif LLT_CONFIG == 3
	rtw_write32(Adapter,REG_RQPN, 0x84838484);
#elif LLT_CONFIG == 4
	rtw_write32(Adapter,REG_RQPN, 0x80bd1c1c);
	//rtw_write8(Adapter,REG_RQPN_NPQ, 0x1c);
#elif LLT_CONFIG == 5 // tynli_test for WiFi WMM. Suggested by TimChen
	if(IS_NORMAL_CHIP(pHalData->VersionID))
		rtw_write16(Adapter, REG_RQPN_NPQ, 0x0000);

	if(Adapter->registrypriv.wifi_spec)
		rtw_write32(Adapter,REG_RQPN, 0x80b01c29);
	else
		rtw_write32(Adapter,REG_RQPN, 0x80bf0d29);
#endif

	// 12.	TXRKTBUG_PG_BNDY 0x114[31:0] = 0x27FF00F6	//TXRKTBUG_PG_BNDY	
	rtw_write32(Adapter,REG_TRXFF_BNDY, (0x27FF0000 |txpktbuf_bndy));

	// 13.	TDECTRL[15:8] 0x209[7:0] = 0xF6				// Beacon Head for TXDMA
	rtw_write8(Adapter,REG_TDECTRL+1, txpktbuf_bndy);

	// 14.	BCNQ_PGBNDY 0x424[7:0] =  0xF6				//BCNQ_PGBNDY
	// 2009/12/03 Why do we set so large boundary. confilct with document V11.
	rtw_write8(Adapter,REG_TXPKTBUF_BCNQ_BDNY, txpktbuf_bndy);
	rtw_write8(Adapter,REG_TXPKTBUF_MGQ_BDNY, txpktbuf_bndy);

	// 15.	WMAC_LBK_BF_HD 0x45D[7:0] =  0xF6			//WMAC_LBK_BF_HD
	rtw_write8(Adapter,0x45D, txpktbuf_bndy);
	
	// Set Tx/Rx page size (Tx must be 128 Bytes, Rx can be 64,128,256,512,1024 bytes)
	// 16.	PBP [7:0] = 0x11								// TRX page size
	rtw_write8(Adapter,REG_PBP, 0x11);		

	// 17.	DRV_INFO_SZ
	rtw_write8(Adapter,REG_RX_DRVINFO_SZ, DRVINFO_SZ);

	// 18.	LLT_table_init(Adapter); 
	for(i = 0 ; i < (txpktbuf_bndy - 1) ; i++){
		status = _LLTWrite(Adapter, i , i + 1);
		if(_SUCCESS != status){
			return status;
		}
	}

	// end of list
	status = _LLTWrite(Adapter, (txpktbuf_bndy - 1), 0xFF); 
	if(_SUCCESS != status){
		return status;
	}

	// Make the other pages as ring buffer
	// This ring buffer is used as beacon buffer if we config this MAC as two MAC transfer.
	// Otherwise used as local loopback buffer. 
	for(i = txpktbuf_bndy ; i < maxPage ; i++){
		status = _LLTWrite(Adapter, i, (i + 1)); 
		if(_SUCCESS != status){
			return status;
		}
	}

	// Let last entry point to the start entry of ring buffer
	status = _LLTWrite(Adapter, maxPage, txpktbuf_bndy);
	if(_SUCCESS != status)
	{
		return status;
	}	
	
	return _SUCCESS;
	
}

//I don't kown why udelay is not enough for REG_APSD_CTRL+1
//so I add more 200 us for every udelay.
#define MORE_DELAY_VS_WIN

static u32 InitMAC(IN	PADAPTER Adapter)
{
	u8	bytetmp;
	u16	wordtmp;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct recv_priv	*precvpriv = &Adapter->recvpriv;
	struct xmit_priv	*pxmitpriv = &Adapter->xmitpriv;
	u16	retry = 0;
	u16	tmpU2b = 0;

	DBG_8192C("=======>InitMAC()\n");

	// Independent some steps for RTL8192CE power on sequence.
	// Create RTL8723e power on sequence v01 which suggested by Scott.
	// First modified by tynli. 2011.01.05.
	// Revise for power sequence v07. 2011.03.21. by tynli.
	if(IS_HARDWARE_TYPE_8723AE(Adapter))
	{
		//For RTL8723e. by tynli. 2010.12.29.
		// Modify for document v003 released by Scott. 2011.01.28.
		
		// 0.
		rtw_write8(Adapter, REG_RSV_CTRL, 0x0);

		//Move step 5 to the first step. Suggested by SD1 Jackie. 2011.03.07. by tynli.
		// 1. 0x04[10] = 0			// disable ROF mode
		bytetmp = rtw_read8(Adapter, REG_APS_FSMCO+1);
			rtw_write8(Adapter, REG_APS_FSMCO+1, bytetmp&(~BIT2));

		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			// 2. wait till 0x04[17] = 1  //power ready (RTL8723 MP chip only, test chip bypass)
			bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+2);
			retry = 0;
			while(!(bytetmp & BIT1) && retry < 100){
				retry++;
				rtw_udelay_os(10);
				bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+2);
				rtw_udelay_os(10);
			}

			if(retry >= 100)
			{
				DBG_8192C("InitMAC(): Poll 0x4[17] ==1 fail!!!\n");
				return _FAIL;
			}
		}

#ifdef RTL8723_FPGA_VERIFICATION
		//
		// For new FPGA bit file. 2011.01.28.
		//
		// 2. 0x04[16] = 1,			// release WLON reset
		bytetmp = rtw_read8(Adapter, REG_APS_FSMCO+2);
		rtw_write8(Adapter, REG_APS_FSMCO+2, (bytetmp|BIT0));
		rtw_udelay_os(10);
#endif

		// 3.	0x04[15] = 0			// disable HWPDN (control by DRV)
		// 4.	0x04[12:11] = 2b'00 // disable WL suspend
		bytetmp = rtw_read8(Adapter, REG_APS_FSMCO+1);
		rtw_write8(Adapter, REG_APS_FSMCO+1, bytetmp&(~(BIT3|BIT4|BIT7)));

		// 7. 0x04[8] = 1 		// polling until return 0
		bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1);
		rtw_write8(Adapter,REG_APS_FSMCO+1, (bytetmp|BIT0));
		rtw_udelay_os(2);

		// 8. wait till 0x04[8] = 0
		bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1);
		retry = 0;
		while(bytetmp & BIT0 && retry < 100){
			retry++;
			rtw_udelay_os(50);
			bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1);
		}
		if(retry >= 100)
		{
			DBG_8192C("Init MAC New flow failed!!!\n");
			return _FAIL;
		}

		// 9. 0x68[3] = 1		// EFUSE autoloaded, check if driver writes again
		// 10. 0x68[0] =1		// EFUSE autoloaded, check if driver writes again
		//bytetmp = PlatformEFIORead1Byte(Adapter, REG_MULTI_FUNC_CTRL);
		//PlatformEFIOWrite1Byte(Adapter, REG_MULTI_FUNC_CTRL, (bytetmp|WL_HWROF_EN|WL_HWPDN_EN));

		//----------------------------------------------------
		// The following steps will be done after InitMAC().
		// 12. Init MAC register
		// 13. 0x58[7] = 1 		// Enable PDNINT to indicate ROF or PDN
		// 14. after card initialization, Read 0x60[1] = 1, goto step 15; 0x42[1] = 0, goto 16
		// 15. goto Radio off or power down flow
		// 16. Init BB and RF
		//----------------------------------------------------
	}
	else
	{
		//2009.12.23. Added by tynli. We should disable PCI host L0s. Suggested by SD1 victorh.
		// 20100422 Joseph: Marked out. This is suggested by SD1 Glayrainx.
		// Lenovo does not like driver to control ASPM setting. All ASPM setting depends on ROOT.
		//RT_DISABLE_HOST_L0S(Adapter);
		
		// 2009/10/13 MH Enable backdoor.
		//PlatformEnable92CEBackDoor(Adapter);

		//
		// 2009/10/13 MH Add for resume sequence of power domain from document of Afred...
		// 2009/12/03 MH Modify according to power document V11. Chapter V.11.
		//	
		// 0.	RSV_CTRL 0x1C[7:0] = 0x00			// unlock ISO/CLK/Power control register
		rtw_write8(Adapter,REG_RSV_CTRL, 0x00);

#ifdef CONFIG_BT_COEXIST
		// For Bluetooth Power save
		// 209/12/03 MH The setting is not written power document now. ?????
		if(	(pHalData->bt_coexist.BT_Coexist) /*&&
			(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4)*/	)
		{
			u32	value32;
			value32 = rtw_read32(Adapter, REG_APS_FSMCO);
			value32 |= (SOP_ABG|SOP_AMB|XOP_BTCK);
			rtw_write32(Adapter, REG_APS_FSMCO, value32);
		}
#endif
		// 1.	AFE_XTAL_CTRL [7:0] = 0x0F		//enable XTAL
		// 2.	SPS0_CTRL 0x11[7:0] = 0x2b		//enable SPS into PWM mode
		// 3.	delay (1ms) 			//this is not necessary when initially power on
		
		// C.	Resume Sequence
		// a.	SPS0_CTRL 0x11[7:0] = 0x2b
		rtw_write8(Adapter,REG_SPS0_CTRL, 0x2b);
		
		// b.	AFE_XTAL_CTRL [7:0] = 0x0F
		rtw_write8(Adapter, REG_AFE_XTAL_CTRL, 0x0F);
		
		// c.	DRV runs power on init flow

#ifdef CONFIG_BT_COEXIST
		// Temporarily fix system hang problem.
		if(	(pHalData->bt_coexist.BT_Coexist) /*&&
			(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4)*/	)
		{
			u32 u4bTmp = rtw_read32(Adapter, REG_AFE_XTAL_CTRL);
		
			//AFE_XTAL_CTRL+2 0x26[1] = 1
			u4bTmp &= (~0x00024800);
			rtw_write32(Adapter, REG_AFE_XTAL_CTRL, u4bTmp);
		}
#endif

		// auto enable WLAN
		// 4.	APS_FSMCO 0x04[8] = 1; //wait till 0x04[8] = 0	
		//Power On Reset for MAC Block
		bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1) | BIT0;
		rtw_udelay_os(2);

	//#ifdef MORE_DELAY_VS_WIN
	//	rtw_udelay_os(200);
	//#endif

		rtw_write8(Adapter,REG_APS_FSMCO+1, bytetmp);
		//DbgPrint("Reg0xEC = %x\n", rtw_read32(Adapter, 0xEC));
		rtw_udelay_os(2);

	//#ifdef MORE_DELAY_VS_WIN
	//	rtw_udelay_os(200);
	//#endif

		// 5.	Wait while 0x04[8] == 0 goto 2, otherwise goto 1
		// 2009/12/03 MH The document V11 loop is not the same current code.
		bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1);
		//DBG_8192C("Reg0xEC = %x\n", rtw_read32(Adapter, 0xEC));
		rtw_udelay_os(2);

	//#ifdef MORE_DELAY_VS_WIN
	//	rtw_mdelay_os(1);
	//#endif

		retry = 0;
		//DBG_8192C("################>%s()Reg0xEC:%x:%x\n", __FUNCTION__,rtw_read32(Adapter, 0xEC),bytetmp);
		while((bytetmp & BIT0) && retry < 1000){
			retry++;
			rtw_udelay_os(50);
			bytetmp = rtw_read8(Adapter,REG_APS_FSMCO+1);
			//DBG_8192C("################>%s()Reg0xEC:%x:%x\n", __FUNCTION__,rtw_read32(Adapter, 0xEC),bytetmp);
			rtw_udelay_os(50);
		}

		// Enable Radio off, GPIO, and LED function
		// 6.	APS_FSMCO 0x04[15:0] = 0x0012		//when enable HWPDN
		rtw_write16(Adapter, REG_APS_FSMCO, 0x1012); //tynli_test to 0x1012. SD1. 2009.12.08.

		 // release RF digital isolation
		 // 7.	SYS_ISO_CTRL 0x01[1]	= 0x0;
		//bytetmp = PlatformEFIORead1Byte(Adapter, REG_SYS_ISO_CTRL+1) & ~BIT1; //marked by tynli.
		//PlatformSleepUs(2);
		//Set REG_SYS_ISO_CTRL 0x1=0x82 to prevent wake# problem. Suggested by Alfred.
		// 2009/12/03 MH We need to check this modification

		tmpU2b = rtw_read16(Adapter,REG_SYS_ISO_CTRL);

		//if(IS_HARDWARE_TYPE_8723AE(Adapter))
		//	rtw_write8(Adapter, REG_SYS_ISO_CTRL, (tmpU2b|ISO_DIOR|PWC_EV12V));
		//else
			rtw_write8(Adapter, REG_SYS_ISO_CTRL+1, 0x82);

		rtw_udelay_os(2);		// By experience!!??

#ifdef CONFIG_BT_COEXIST
		// Enable MAC DMA/WMAC/SCHEDULE block
		// 8.	AFE_XTAL_CTRL [17] = 0;			//with BT, driver will set in card disable
		if(	(pHalData->bt_coexist.BT_Coexist) /*&&
			(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4)	*/) //tynli_test 2009.12.16.
		{
			bytetmp = rtw_read8(Adapter, REG_AFE_XTAL_CTRL+2) & 0xfd;
			rtw_write8(Adapter, REG_AFE_XTAL_CTRL+2, bytetmp);
		}
#endif
	}

	// Release MAC IO register reset
	// 9.	CR 0x100[7:0]	= 0xFF;
	// 10.	CR 0x101[1]	= 0x01; // Enable SEC block
	rtw_write16(Adapter,REG_CR, 0x3ff);

	//WritePortUchar(0x553, 0xff);  //reset ??? richard 0507
	// What is the meaning???
	//PlatformEFIOWrite1Byte(Adapter,0x553, 0xFF);	
	
	//PlatformSleepUs(1500);
	//bytetmp = PlatformEFIORead1Byte(Adapter,REG_CR);
	//bytetmp |= (BIT6 | BIT7);//(CmdTxEnb|CmdRxEnb);
	//PlatformEFIOWrite1Byte(Adapter,REG_CR, bytetmp);

	// 2009/12/03 MH The section of initialize code does not exist in the function~~~!
	// These par is inserted into function LLT_table_init
	// 11.	RQPN 0x200[31:0]	= 0x80BD1C1C				// load RQPN
	// 12.	TXRKTBUG_PG_BNDY 0x114[31:0] = 0x27FF00F6	//TXRKTBUG_PG_BNDY
	// 13.	TDECTRL[15:8] 0x209[7:0] = 0xF6				// Beacon Head for TXDMA
	// 14.	BCNQ_PGBNDY 0x424[7:0] =  0xF6				//BCNQ_PGBNDY
	// 15.	WMAC_LBK_BF_HD 0x45D[7:0] =  0xF6			//WMAC_LBK_BF_HD
	// 16.	PBP [7:0] = 0x11								// TRX page size
	// 17.	DRV_INFO_SZ = 0x04
	
	//System init
	// 18.	LLT_table_init(Adapter); 
	if(LLT_table_init(Adapter) == _FAIL)
	{
		return _FAIL;
	}

	// Clear interrupt and enable interrupt
	// 19.	HISR 0x124[31:0] = 0xffffffff; 
	//	   	HISRE 0x12C[7:0] = 0xFF
	// NO 0x12c now!!!!!
	rtw_write32(Adapter,REG_HISR, 0xffffffff);
	rtw_write8(Adapter,REG_HISRE, 0xff);
	
	// 20.	HIMR 0x120[31:0] |= [enable INT mask bit map]; 
	// 21.	HIMRE 0x128[7:0] = [enable INT mask bit map]
	// The IMR should be enabled later after all init sequence is finished.

	// ========= PCIE related register setting =======
	// 22.	PCIE configuration space configuration
	// 23.	Ensure PCIe Device 0x80[15:0] = 0x0143 (ASPM+CLKREQ), 
	//          and PCIe gated clock function is enabled.	
	// PCIE configuration space will be written after all init sequence.(Or by BIOS)


	// Set Rx FF0 boundary : 9K/10K
	// 2009/12/03 MH This should be relative to HW setting. But he power document does
	// not contain the description.
	rtw_write16(Adapter,REG_TRXFF_BNDY+2, 0x27ff);

	//
	// 2009/12/03 MH THe below section is not related to power document Vxx .
	// This is only useful for driver and OS setting.
	//
	// -------------------Software Relative Setting----------------------
	//
	if(IS_81XXC_TEST_CHIP(pHalData->VersionID))
	{
		// Set High priority queue select : HPQ:BC/H/VO/VI/MG, LPQ:BE/BK
		// [5]:H, [4]:MG, [3]:BK, [2]:BE, [1]:VI, [0]:VO
		rtw_write16(Adapter,REG_TRXDMA_CTRL, 0x3501);
	}
	else
	{
		wordtmp = rtw_read16(Adapter,REG_TRXDMA_CTRL);
		wordtmp &= 0xf;
		wordtmp |= 0xF771;
		rtw_write16(Adapter,REG_TRXDMA_CTRL, wordtmp);
	}
	
	// Reported Tx status from HW for rate adaptive.
	// 2009/12/03 MH This should be realtive to power on step 14. But in document V11  
	// still not contain the description.!!!
	rtw_write8(Adapter,REG_FWHW_TXQ_CTRL+1, 0x1F);

	// Set RCR register
	rtw_write32(Adapter,REG_RCR, pHalData->ReceiveConfig);

	// Set TCR register
	rtw_write32(Adapter,REG_TCR, pHalData->TransmitConfig);

	// disable earlymode
	rtw_write8(Adapter,0x4d0, 0x0);
		
	//
	// Set TX/RX descriptor physical address(from OS API).
	//
	rtw_write32(Adapter, REG_BCNQ_DESA, (u64)pxmitpriv->tx_ring[BCN_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_MGQ_DESA, (u64)pxmitpriv->tx_ring[MGT_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_VOQ_DESA, (u64)pxmitpriv->tx_ring[VO_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_VIQ_DESA, (u64)pxmitpriv->tx_ring[VI_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_BEQ_DESA, (u64)pxmitpriv->tx_ring[BE_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_BKQ_DESA, (u64)pxmitpriv->tx_ring[BK_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_HQ_DESA, (u64)pxmitpriv->tx_ring[HIGH_QUEUE_INX].dma & DMA_BIT_MASK(32));
	rtw_write32(Adapter, REG_RX_DESA, (u64)precvpriv->rx_ring[RX_MPDU_QUEUE].dma & DMA_BIT_MASK(32));

#ifdef CONFIG_64BIT_DMA
	// 2009/10/28 MH For DMA 64 bits. We need to assign the high 32 bit address
	// for NIC HW to transmit data to correct path.
	rtw_write32(Adapter, REG_BCNQ_DESA+4, 
		((u64)pxmitpriv->tx_ring[BCN_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter, REG_MGQ_DESA+4, 
		((u64)pxmitpriv->tx_ring[MGT_QUEUE_INX].dma)>>32);  
	rtw_write32(Adapter, REG_VOQ_DESA+4, 
		((u64)pxmitpriv->tx_ring[VO_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter, REG_VIQ_DESA+4, 
		((u64)pxmitpriv->tx_ring[VI_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter, REG_BEQ_DESA+4, 
		((u64)pxmitpriv->tx_ring[BE_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter, REG_BKQ_DESA+4,
		((u64)pxmitpriv->tx_ring[BK_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter,REG_HQ_DESA+4,
		((u64)pxmitpriv->tx_ring[HIGH_QUEUE_INX].dma)>>32);
	rtw_write32(Adapter, REG_RX_DESA+4, 
		((u64)precvpriv->rx_ring[RX_MPDU_QUEUE].dma)>>32);


	// 2009/10/28 MH If RX descriptor address is not equal to zero. We will enable
	// DMA 64 bit functuion.
	// Note: We never saw thd consition which the descripto address are divided into
	// 4G down and 4G upper seperate area.
	if (((u64)precvpriv->rx_ring[RX_MPDU_QUEUE].dma)>>32 != 0)
	{
		//DBG_8192C("RX_DESC_HA=%08lx\n", ((u64)priv->rx_ring_dma[RX_MPDU_QUEUE])>>32);
		DBG_8192C("Enable DMA64 bit\n");

		// Check if other descriptor address is zero and abnormally be in 4G lower area.
		if (((u64)pxmitpriv->tx_ring[MGT_QUEUE_INX].dma)>>32)
		{
			DBG_8192C("MGNT_QUEUE HA=0\n");
		}
		
		PlatformEnable92CEDMA64(Adapter);
	}
	else
	{
		DBG_8192C("Enable DMA32 bit\n");
	}
#endif

	// 2009/12/03 MH This should be included in power
	// Set to 0x22. Suggested to SD1 Alan. by tynli. 2009.12.10.
	// 20100317 Joseph: Resume the setting of Tx/Rx DMA burst size(Reg0x303) to 0x77 suggested by SD1.
	// 20100324 Joseph: Set different value for Tx/Rx DMA burst size(Reg0x303) suggested by SD1.
	// 92CE set to 0x77 and 88CE set to 0x22.
	if(IS_HARDWARE_TYPE_8723AE(Adapter))
	{
		// Set to 0x74 (Tx DMA[6:4]=128 bytes, Rx DMA[2:0]=256 bytes) for 8723e.
		// Suggested by SD1 Alan. Added by tynli. 2011.02.15. 
		rtw_write8(Adapter,REG_PCIE_CTRL_REG+3, 0x74);
	}
	else
	{
		if(IS_92C_SERIAL(pHalData->VersionID))
			rtw_write8(Adapter,REG_PCIE_CTRL_REG+3, 0x77);
		else
			rtw_write8(Adapter,REG_PCIE_CTRL_REG+3, 0x22);
	}

	// 20100318 Joseph: Reset interrupt migration setting when initialization. Suggested by SD1.
	rtw_write32(Adapter, REG_INT_MIG, 0);	
	pHalData->bInterruptMigration = _FALSE;

	// 20090928 Joseph: Add temporarily.
	// Reconsider when to do this operation after asking HWSD.
	bytetmp = rtw_read8(Adapter, REG_APSD_CTRL);
	rtw_write8(Adapter, REG_APSD_CTRL, bytetmp & ~BIT6);
	do{
		retry++;
		bytetmp = rtw_read8(Adapter, REG_APSD_CTRL);
	}while((retry<200) && (bytetmp&BIT7)); //polling until BIT7 is 0. by tynli

	// 2009/10/26 MH For led test.
	// After MACIO reset,we must refresh LED state.
	rtl8192ce_gen_RefreshLedState(Adapter);

	//2009.10.19. Reset H2C protection register. by tynli.
	rtw_write32(Adapter, REG_MCUTST_1, 0x0);

#if MP_DRIVER == 1
	rtw_write32(Adapter, REG_MACID, 0x87654321);
	rtw_write32(Adapter, 0x0700, 0x87654321);
#endif

	//
	// -------------------Software Relative Setting----------------------
	//

	DBG_8192C("<=======InitMAC()\n");

	return _SUCCESS;
	
}

static VOID
EnableAspmBackDoor92CE(IN	PADAPTER Adapter)
{
	struct pwrctrl_priv		*pwrpriv = &Adapter->pwrctrlpriv;

	// 0x70f BIT7 is used to control L0S
	// 20100212 Tynli: Set register offset 0x70f in PCI configuration space to the value 0x23 
	// for all bridge suggested by SD1. Origianally this is only for INTEL.
	// 20100422 Joseph: Set PCI configuration space offset 0x70F to 0x93 to Enable L0s for all platform.
	// This is suggested by SD1 Glayrainx and for Lenovo's request.
	//if(GetPciBridgeVendor(Adapter) == PCI_BRIDGE_VENDOR_INTEL)
		rtw_write8(Adapter, 0x34b, 0x93);
	//else
	//	PlatformEFIOWrite1Byte(Adapter, 0x34b, 0x23);
	rtw_write16(Adapter, 0x350, 0x870c);
	rtw_write8(Adapter, 0x352, 0x1);

	// 0x719 Bit3 is for L1 BIT4 is for clock request 
	// 20100427 Joseph: Disable L1 for Toshiba AMD platform. If AMD platform do not contain
	// L1 patch, driver shall disable L1 backdoor.
	if(pwrpriv->b_support_backdoor)
		rtw_write8(Adapter, 0x349, 0x1b);
	else
		rtw_write8(Adapter, 0x349, 0x03);
	rtw_write16(Adapter, 0x350, 0x2718);
	rtw_write8(Adapter, 0x352, 0x1);
}

static u32 rtl8192ce_hal_init(PADAPTER Adapter)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	EEPROM_EFUSE_PRIV	*pEEPROM = GET_EEPROM_EFUSE_PRIV(Adapter);
	struct pwrctrl_priv		*pwrpriv = &Adapter->pwrctrlpriv;
	u32	rtStatus = _SUCCESS;
	u8	tmpU1b;
	u8	eRFPath;
	u32	i;
	BOOLEAN bSupportRemoteWakeUp, isCTest, is2T2R;

_func_enter_;

	//
	// No I/O if device has been surprise removed
	//
	if (Adapter->bSurpriseRemoved)
	{
		DBG_8192C("rtl8192ce_hal_init(): bSurpriseRemoved!\n");
		return _SUCCESS;
	}

	DBG_8192C("=======>rtl8192ce_hal_init()\n");

	//rtl8192ce_reset_desc_ring(Adapter);

	//
	// 1. MAC Initialize
	//
	rtStatus = InitMAC(Adapter);
	if(rtStatus != _SUCCESS)
	{
		DBG_8192C("Init MAC failed\n");
		return rtStatus;
	}

#if (MP_DRIVER != 1)
#if HAL_FW_ENABLE
	rtStatus = FirmwareDownload92C(Adapter, _FALSE);
	if(rtStatus != _SUCCESS)
	{
		DBG_8192C("FwLoad failed\n");
		rtStatus = _SUCCESS;
		Adapter->bFWReady = _FALSE;
		pHalData->fw_ractrl = _FALSE;
	}
	else
	{
		DBG_8192C("FwLoad SUCCESSFULLY!!!\n");
		Adapter->bFWReady = _TRUE;
		pHalData->fw_ractrl = _TRUE;
	}

	InitializeFirmwareVars92C(Adapter);
#endif
#endif

// 20100318 Joseph: These setting only for FPGA.
// Add new type "ASIC" and set RFChipID and RF_Type in ReadAdapter function.
#if BOARD_TYPE==FPGA_2MAC
	pHalData->rf_chip = RF_PSEUDO_11N;
	pHalData->rf_type = RF_2T2R;
#elif BOARD_TYPE==FPGA_PHY
	#if FPGA_RF==FPGA_RF_8225
		pHalData->rf_chip = RF_8225;
		pHalData->rf_type = RF_2T2R;
	#elif FPGA_RF==FPGA_RF_0222D
		pHalData->rf_chip = RF_6052;
		pHalData->rf_type = RF_2T2R;
	#endif
#endif

	//
	// 2. Initialize MAC/PHY Config by MACPHY_reg.txt
	//
#if (HAL_MAC_ENABLE == 1)
	DBG_8192C("MAC Config Start!\n");
	rtStatus = PHY_MACConfig8192C(Adapter);
	if (rtStatus != _SUCCESS)
	{
		DBG_8192C("MAC Config failed\n");
		return rtStatus;
	}
	DBG_8192C("MAC Config Finished!\n");

	rtw_write32(Adapter,REG_RCR, rtw_read32(Adapter, REG_RCR)&~(RCR_ADF) );
#endif	// #if (HAL_MAC_ENABLE == 1)

	//
	// 3. Initialize BB After MAC Config PHY_reg.txt, AGC_Tab.txt
	//
#if (HAL_BB_ENABLE == 1)
	DBG_8192C("BB Config Start!\n");
	rtStatus = PHY_BBConfig8192C(Adapter);
	if (rtStatus!= _SUCCESS)
	{
		DBG_8192C("BB Config failed\n");
		return rtStatus;
	}
	DBG_8192C("BB Config Finished!\n");
#endif	// #if (HAL_BB_ENABLE == 1)


	_InitBeaconParameters(Adapter);

	
	//
	// 4. Initiailze RF RAIO_A.txt RF RAIO_B.txt
	//
	// 2007/11/02 MH Before initalizing RF. We can not use FW to do RF-R/W.
	//pHalData->Rf_Mode = RF_OP_By_SW_3wire;
#if (HAL_RF_ENABLE == 1)		
	DBG_8192C("RF Config started!\n");
	rtStatus = PHY_RFConfig8192C(Adapter);
	if(rtStatus != _SUCCESS)
	{
		DBG_8192C("RF Config failed\n");
		return rtStatus;
	}
	DBG_8192C("RF Config Finished!\n");

	if(IS_VENDOR_UMC_A_CUT(pHalData->VersionID) && !IS_92C_SERIAL(pHalData->VersionID))
	{
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_RX_G1, bMaskDWord, 0x30255);
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_RX_G2, bMaskDWord, 0x50a00);		
	}
	else if(IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
	{
		DBG_8192C("RF Config Finished! IS_VENDOR_UMC_B_CUT\n");
	
		PHY_SetRFReg(Adapter, RF_PATH_A, 0x0C, bMaskDWord, 0x894AE);
		PHY_SetRFReg(Adapter, RF_PATH_A, 0x0A, bMaskDWord, 0x1AF31);
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_IPA, bMaskDWord, 0x8F425);
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_SYN_G2, bMaskDWord, 0x4F200);
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_RCK1, bMaskDWord, 0x44053);
		PHY_SetRFReg(Adapter, RF_PATH_A, RF_RCK2, bMaskDWord, 0x80201);
		//PHY_SetRFReg(Adapter, RF_PATH_A, RF_RX_G1, bMaskDWord, 0x00255);
	}

	// 20100329 Joseph: Restore RF register value for later use in channel switching.
	pHalData->RfRegChnlVal[0] = PHY_QueryRFReg(Adapter, (RF_RADIO_PATH_E)0, RF_CHNLBW, bRFRegOffsetMask);
	pHalData->RfRegChnlVal[1] = PHY_QueryRFReg(Adapter, (RF_RADIO_PATH_E)1, RF_CHNLBW, bRFRegOffsetMask);
#endif	// #if (HAL_RF_ENABLE == 1)	

	// After read predefined TXT, we must set BB/MAC/RF register as our requirement
	/*---- Set CCK and OFDM Block "ON"----*/
	PHY_SetBBReg(Adapter, rFPGA0_RFMOD, bCCKEn, 0x1);
	PHY_SetBBReg(Adapter, rFPGA0_RFMOD, bOFDMEn, 0x1);
#if (MP_DRIVER == 0)
	// Set to 20MHz by default
	PHY_SetBBReg(Adapter, rFPGA0_AnalogParameter2, BIT10, 1);
#endif

	pHalData->CurrentWirelessMode = WIRELESS_MODE_AUTO;

	//3 Set Hardware(MAC default setting.)
	HwConfigureRTL8192CE(Adapter);

	//3 Set Wireless Mode
	// TODO: Emily 2006.07.13. Wireless mode should be set according to registry setting and RF type
	//Default wireless mode is set to "WIRELESS_MODE_N_24G|WIRELESS_MODE_G", 
	//and the RRSR is set to Legacy OFDM rate sets. We do not include the bit mask 
	//of WIRELESS_MODE_B currently. Emily, 2006.11.13
	//For wireless mode setting from mass. 
	//if(Adapter->ResetProgress == RESET_TYPE_NORESET)
	//	Adapter->HalFunc.SetWirelessModeHandler(Adapter, Adapter->RegWirelessMode);

	//3Security related
	//-----------------------------------------------------------------------------
	// Set up security related. 070106, by rcnjko:
	// 1. Clear all H/W keys.
	// 2. Enable H/W encryption/decryption.
	//-----------------------------------------------------------------------------
	// 92SE not enable security now
	{
		u8 SECR_value = 0x0;

		invalidate_cam_all(Adapter);

		if(IS_NORMAL_CHIP(pHalData->VersionID))
			SECR_value |= (BIT6|BIT7);

		// Joseph debug: MAC_SEC_EN need to be set
		rtw_write8(Adapter, REG_CR+1, (rtw_read8(Adapter, REG_CR+1)|BIT1));

		rtw_write8(Adapter, REG_SECCFG, SECR_value);
	}

	pHalData->CurrentChannel = 6;//default set to 6

	/* Write correct tx power index */
	PHY_SetTxPowerLevel8192C(Adapter, pHalData->CurrentChannel);

	//2=======================================================
	// RF Power Save
	//2=======================================================
#if 1
	// Fix the bug that Hw/Sw radio off before S3/S4, the RF off action will not be executed 
	// in MgntActSet_RF_State() after wake up, because the value of pHalData->eRFPowerState 
	// is the same as eRfOff, we should change it to eRfOn after we config RF parameters.
	// Added by tynli. 2010.03.30.
	pwrpriv->rf_pwrstate = rf_on;

	// 20100326 Joseph: Copy from GPIOChangeRFWorkItemCallBack() function to check HW radio on/off.
	// 20100329 Joseph: Revise and integrate the HW/SW radio off code in initialization.
	tmpU1b = rtw_read8(Adapter, REG_MAC_PINMUX_CFG)&(~BIT3);
	rtw_write8(Adapter, REG_MAC_PINMUX_CFG, tmpU1b);
	tmpU1b = rtw_read8(Adapter, REG_GPIO_IO_SEL);
	DBG_8192C("GPIO_IN=%02x\n", tmpU1b);
	pwrpriv->rfoff_reason |= (tmpU1b & BIT3) ? 0 : RF_CHANGE_BY_HW;
	pwrpriv->rfoff_reason |= (pwrpriv->reg_rfoff) ? RF_CHANGE_BY_SW : 0;

	if(pwrpriv->rfoff_reason & RF_CHANGE_BY_HW)
		pwrpriv->b_hw_radio_off = _TRUE;

	if(pwrpriv->rfoff_reason > RF_CHANGE_BY_PS)
	{ // H/W or S/W RF OFF before sleep.
		DBG_8192C("InitializeAdapter8192CE(): Turn off RF for RfOffReason(%d) ----------\n", pwrpriv->rfoff_reason);
		//MgntActSet_RF_State(Adapter, rf_off, pwrpriv->rfoff_reason, _TRUE);
	}
	else
	{
		pwrpriv->rf_pwrstate = rf_on;
		pwrpriv->rfoff_reason = 0;

		DBG_8192C("InitializeAdapter8192CE(): Turn on  ----------\n");

		// LED control
		rtw_led_control(Adapter, LED_CTL_POWER_ON);

		//
		// If inactive power mode is enabled, disable rf while in disconnected state.
		// But we should still tell upper layer we are in rf on state.
		// 2007.07.16, by shien chang.
		//
		//if(!Adapter->bInHctTest)
			//IPSEnter(Adapter);
	}
#endif

	// Fix the bug that when the system enters S3/S4 then tirgger HW radio off, after system
	// wakes up, the scan OID will be set from upper layer, but we still in RF OFF state and scan
	// list is empty, such that the system might consider the NIC is in RF off state and will wait 
	// for several seconds (during this time the scan OID will not be set from upper layer anymore)
	// even though we have already HW RF ON, so we tell the upper layer our RF state here.
	// Added by tynli. 2010.04.01.
	//DrvIFIndicateCurrentPhyStatus(Adapter);

	if(Adapter->registrypriv.hw_wps_pbc)
	{
		tmpU1b = rtw_read8(Adapter, GPIO_IO_SEL);
		tmpU1b &= ~(HAL_8192C_HW_GPIO_WPS_BIT);
		rtw_write8(Adapter, GPIO_IO_SEL, tmpU1b);	//enable GPIO[2] as input mode
	}

	//
	// Execute TX power tracking later
	//

	// We must set MAC address after firmware download. HW do not support MAC addr
	// autoload now.
	for(i=0; i<6; i++)
	{
#ifdef  CONFIG_CONCURRENT_MODE			//Add by Aries, 20120229
		if(Adapter->iface_type == IFACE_PORT1)			
			rtw_write8(Adapter, (REG_MACID1+i), pEEPROM->mac_addr[i]);			
		else
#endif
		rtw_write8(Adapter, (REG_MACID+i), pEEPROM->mac_addr[i]);		 	
	}

	// Joseph. Turn on the secret lock of ASPM.
	EnableAspmBackDoor92CE(Adapter);

#ifdef CONFIG_BT_COEXIST
	_InitBTCoexist(Adapter);
#endif

	rtl8192c_InitHalDm(Adapter);
//	Add by Aries , 20120229
#if defined(CONFIG_CONCURRENT_MODE) || defined(CONFIG_TX_MCAST2UNI)

#ifdef CONFIG_CHECK_AC_LIFETIME
	// Enable lifetime check for the four ACs
	rtw_write8(Adapter, REG_LIFETIME_EN, 0x0F);
#endif	// CONFIG_CHECK_AC_LIFETIME

#ifdef CONFIG_TX_MCAST2UNI
	rtw_write16(Adapter, REG_PKT_VO_VI_LIFE_TIME, 0x0400);	// unit: 256us. 256ms
	rtw_write16(Adapter, REG_PKT_BE_BK_LIFE_TIME, 0x0400);	// unit: 256us. 256ms
#else	// CONFIG_TX_MCAST2UNI
	rtw_write16(Adapter, REG_PKT_VO_VI_LIFE_TIME, 0x3000);	// unit: 256us. 3s
	rtw_write16(Adapter, REG_PKT_BE_BK_LIFE_TIME, 0x3000);	// unit: 256us. 3s
#endif	// CONFIG_TX_MCAST2UNI
#endif	// CONFIG_CONCURRENT_MODE || CONFIG_TX_MCAST2UNI


	pHalData->RegBcnCtrlVal = rtw_read8(Adapter, REG_BCN_CTRL);
	pHalData->RegTxPause = rtw_read8(Adapter, REG_TXPAUSE); 
	pHalData->RegFwHwTxQCtrl = rtw_read8(Adapter, REG_FWHW_TXQ_CTRL+2);
	pHalData->RegReg542 = rtw_read8(Adapter, REG_TBTT_PROHIBIT+2);

	//EnableInterrupt8192CE(Adapter);

#if (MP_DRIVER == 1)
	Adapter->mppriv.channel = pHalData->CurrentChannel;
	MPT_InitializeAdapter(Adapter, Adapter->mppriv.channel);
#else
	// 20100329 Joseph: Disable te Caliberation operation when Radio off.
	// This prevent from outputing signal when initialization in Radio-off state.
	if(pwrpriv->rf_pwrstate == rf_on)
	{
		if(Adapter->ledpriv.LedStrategy != SW_LED_MODE10)
			rtl8192c_PHY_SetRFPathSwitch(Adapter, pHalData->bDefaultAntenna);	//Wifi default use Main

		if(pHalData->bIQKInitialized )
			rtl8192c_PHY_IQCalibrate(Adapter, _TRUE);
		else
		{
			rtl8192c_PHY_IQCalibrate(Adapter, _FALSE);
			pHalData->bIQKInitialized = _TRUE;
		}		
		
		rtl8192c_dm_CheckTXPowerTracking(Adapter);
		rtl8192c_PHY_LCCalibrate(Adapter);
	}
#endif

#if 0
	//WoWLAN setting. by tynli.
	Adapter->HalFunc.GetHalDefVarHandler(Adapter, HAL_DEF_WOWLAN , &bSupportRemoteWakeUp);
	if(bSupportRemoteWakeUp) // WoWLAN setting. by tynli.
	{
		u8	u1bTmp;
		u8	i;
#if 0
		u4Byte	u4bTmp;

		//Disable L2 support
		u4bTmp = PlatformEFIORead4Byte(Adapter, REG_PCIE_CTRL_REG);
		u4bTmp &= ~(BIT17);
		 PlatformEFIOWrite4Byte(Adapter, REG_PCIE_CTRL_REG, u4bTmp);
#endif

		// enable Rx DMA. by tynli.
		u1bTmp = rtw_read8(Adapter, REG_RXPKT_NUM+2);
		u1bTmp &= ~(BIT2);
		rtw_write8(Adapter, REG_RXPKT_NUM+2, u1bTmp);

		if(pPSC->WoWLANMode == eWakeOnMagicPacketOnly)
		{
			//Enable magic packet and WoWLAN function in HW.
			rtw_write8(Adapter, REG_WOW_CTRL, WOW_MAGIC);
		}
		else if (pPSC->WoWLANMode == eWakeOnPatternMatchOnly)
		{
			//Enable pattern match and WoWLAN function in HW.
			rtw_write8(Adapter, REG_WOW_CTRL, WOW_WOMEN);
		}
		else if (pPSC->WoWLANMode == eWakeOnBothTypePacket)
		{
			//Enable magic packet, pattern match, and WoWLAN function in HW.
			rtw_write8(Adapter, REG_WOW_CTRL, WOW_MAGIC|WOW_WOMEN); 
		}
		
		PlatformClearPciPMEStatus(Adapter);

		if(ADAPTER_TEST_STATUS_FLAG(Adapter, ADAPTER_STATUS_FIRST_INIT))
		{
			//Reset WoWLAN register and related data structure at the first init. 2009.06.18. by tynli.
			ResetWoLPara(Adapter);
		}
		else
		{
			if(pPSC->WoWLANMode > eWakeOnMagicPacketOnly)
			{
				//Rewrite WOL pattern and mask to HW.
				for(i=0; i<(MAX_SUPPORT_WOL_PATTERN_NUM-2); i++)
				{
					Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_WF_MASK, (pu1Byte)(&i)); 
					Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_WF_CRC, (pu1Byte)(&i)); 
				}
			}
		}
	}
#endif

	if (pHalData->rf_type == RF_1T1R){
		pHalData->bRFPathRxEnable[0] = _TRUE;
	}
	else{
		pHalData->bRFPathRxEnable[0] = pHalData->bRFPathRxEnable[1] = _TRUE;
	}

	pHalData->RegFwHwTxQCtrl = rtw_read8(Adapter, REG_FWHW_TXQ_CTRL+2);

#if 0
	isNormal = IS_NORMAL_CHIP(pHalData->VersionID);
	is92C = IS_92C_SERIAL(pHalData->VersionID);
#else
	isCTest = IS_81XXC_TEST_CHIP(pHalData->VersionID);
	is2T2R = IS_2T2R(pHalData->VersionID);
#endif

	tmpU1b = EFUSE_Read1Byte(Adapter, 0x1FA);

	if(!(tmpU1b & BIT0))
	{
		PHY_SetRFReg(Adapter, RF_PATH_A, 0x15, 0x0F, 0x05);
		DBG_8192C("PA BIAS path A\n");
	}	

#if 0
	if(!(tmpU1b & BIT1) && isNormal && is92C)
#else
	if(!(tmpU1b & BIT1) && (!isCTest) && is2T2R)
#endif
	{
		PHY_SetRFReg(Adapter, RF_PATH_B, 0x15, 0x0F, 0x05);
		DBG_8192C("PA BIAS path B\n");
	}

	if(!(tmpU1b & BIT4))
	{
		tmpU1b = rtw_read8(Adapter, 0x16);
		tmpU1b &= 0x0F; 
		rtw_write8(Adapter, 0x16, tmpU1b | 0x80);
		rtw_udelay_os(10);
		rtw_write8(Adapter, 0x16, tmpU1b | 0x90);
		DBG_8192C("under 1.5V\n");
	}

/*{
	DBG_8192C("===== Start Dump Reg =====");
	for(i = 0 ; i <= 0xeff ; i+=4)
	{
		if(i%16==0)
			DBG_8192C("\n%04x: ",i);
		DBG_8192C("0x%08x ",rtw_read32(Adapter, i));
	}
	DBG_8192C("\n ===== End Dump Reg =====\n");
}*/

_func_exit_;

	return rtStatus;
}

//
// 2009/10/13 MH Acoording to documetn form Scott/Alfred....
// This is based on version 8.1.
//
static VOID
PowerOffAdapter8192CE(
	IN	PADAPTER			Adapter	
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8	u1bTmp;
	u32	u4bTmp, u4bTmpGPIO;
	
	DBG_8192C("=======>PowerOffAdapter8192CE()\n");
	// XVI.1.1 PCIe Card Disable

	if(IS_HARDWARE_TYPE_8723AE(Adapter))
	{
		//CardDisableRTL8723E(Adapter);
		return;
	}

	// A.	Target 

	// a.	WLAN all disable (RF off, A15 off, DCORE reset)
	// b.	MCU reset
	// c.	A33 disable (AFE bandgap and m-bias on, others disable)
	// d.	XTAL off
	// e.	PON on
	// f.		HCI D3 mode (the same as S3 state)
	// g.	REG can be accessed by host. Resume by register control.


	// B.	Off Sequence 

	//
	// 2009/10/13 MH Refer to document RTL8191C power sec V8.1 sequence.
	// Chapter 6.1 for power card disable.
	//
	// A. Ensure PCIe Device 0x80[15:0] = 0x0143 (ASPM+CLKREQ), and PCIe gated 
	// clock function is enabled.
	rtw_pci_enable_aspm(Adapter);

	//==== RF Off Sequence ====
	// a.	TXPAUSE 0x522[7:0] = 0xFF			//Pause MAC TX queue
	rtw_write8(Adapter, REG_TXPAUSE, 0xFF);
	
	// b.	RF path 0 offset 0x00 = 0x00		// disable RF
	//PlatformEFIOWrite1Byte(Adapter, 0x00, 0x00);
	PHY_SetRFReg(Adapter, RF_PATH_A, 0x00, bRFRegOffsetMask, 0x00);
	//tynli_test. by sd1 Jonbon. Turn off then resume, system will restart.
	rtw_write8(Adapter, REG_RF_CTRL, 0x00);

	// c.	APSD_CTRL 0x600[7:0] = 0x40
	rtw_write8(Adapter, REG_APSD_CTRL, 0x40);

	// d.	SYS_FUNC_EN 0x02[7:0] = 0xE2		//reset BB state machine
	rtw_write8(Adapter, REG_SYS_FUNC_EN, 0xE2);

	// e.		SYS_FUNC_EN 0x02[7:0] = 0xE0		//reset BB state machine
	rtw_write8(Adapter, REG_SYS_FUNC_EN, 0xE0);

	//  ==== Reset digital sequence   ======
	
	if((rtw_read8(Adapter, REG_MCUFWDL)&BIT7) && 
		Adapter->bFWReady) //8051 RAM code
	{	
		rtl8192c_FirmwareSelfReset(Adapter);
	}

	// Clear rpwm value for initial toggle bit trigger.
	rtw_write8(Adapter, REG_PCIE_HRPWM, 0x00);

	// f.	SYS_FUNC_EN 0x03[7:0]=0x51		// reset MCU, MAC register, DCORE
	rtw_write8(Adapter, REG_SYS_FUNC_EN+1, 0x51);

	// g.	MCUFWDL 0x80[1:0]=0				// reset MCU ready status
	rtw_write8(Adapter, REG_MCUFWDL, 0x00);
	
	//  ==== Pull GPIO PIN to balance level and LED control ======

	// h.		GPIO_PIN_CTRL 0x44[31:0]=0x000		// 
	rtw_write32(Adapter, REG_GPIO_PIN_CTRL, 0x00000000);

	// i.		Value = GPIO_PIN_CTRL[7:0]
	//u1bTmp = rtw_read8(Adapter, REG_GPIO_PIN_CTRL);
	u4bTmp = rtw_read32(Adapter, REG_GPIO_PIN_CTRL) & 0xFFFF00FF;	
	u4bTmpGPIO = u4bTmp&0x000000FF;

#ifdef CONFIG_BT_COEXIST
	if((pHalData->bt_coexist.BT_Coexist) &&
		((pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4)||
		(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC8)))
	{ // when BT COEX exist
		//j. GPIO_PIN_CTRL 0x44[31:0] = 0x00F30000 | (value <<8); //write external PIN level 
		rtw_write32(Adapter, REG_GPIO_PIN_CTRL, 0x00F30000| (u1bTmp <<8));
	}
	else
#endif
	{ //Without BT COEX
		//j.	GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value <<8); //write external PIN level 
		u4bTmp |= ((u4bTmpGPIO<<8) | 0x00FF0000);
		rtw_write32(Adapter, REG_GPIO_PIN_CTRL, u4bTmp);
	}

	if( IS_HARDWARE_TYPE_8723AE(Adapter) )	  
	{	
		//
		// <Roger_Notes> For RTL8723u multi-function configuration which was autoload from Efuse offset 0x0a and 0x0b,
		// WLAN HW GPIO[9], GPS HW GPIO[10] and BT HW GPIO[11]. 
		// Added by Roger, 2010.10.07.
		//		
		//2. Disable GPIO[8] and GPIO[12]
		rtw_write16(Adapter, REG_GPIO_IO_SEL_2, 0x0000); // Configure all pins as input mode.
	    	u4bTmp = rtw_read32(Adapter, REG_GPIO_PIN_CTRL_2) & 0xFFFF001F;  
		u4bTmpGPIO = (u4bTmp&0x0000001F);
		if( IS_MULTI_FUNC_CHIP(Adapter) )
			u4bTmp |= ((u4bTmpGPIO<<8) | 0x00100000); // Set pin 8 and pin 12 to output mode. pin8 need to be input mode. 20110111 tynli
		else
			u4bTmp |= ((u4bTmpGPIO<<8) | 0x001C0000); // Set pin 8, 10, 11 and pin 12 to output mode. pin8 need to be input mode. 20110111 tynli
		rtw_write32(Adapter, REG_GPIO_PIN_CTRL_2, u4bTmp);		
	}
	else
	{
		// k.	GPIO_MUXCFG 0x42 [15:0] = 0x0780
		rtw_write16(Adapter, REG_GPIO_IO_SEL, 0x0790);
	}

	// l.	LEDCFG 0x4C[15:0] = 0x8080
#if 0
	if(pHalData->InterfaceSel == INTF_SEL1_BT_COMBO_MINICARD)
	{
		rtw_write32(Adapter, REG_LEDCFG0, 0x00028080);	
		DBG_8192C("poweroffadapter 0x4c 0x%x\n", rtw_read32(Adapter, REG_LEDCFG0));
	}
	else
#endif
		rtw_write16(Adapter, REG_LEDCFG0, 0x8080);

	//  ==== Disable analog sequence ===

	// m.	AFE_PLL_CTRL[7:0] = 0x80			//disable PLL
	rtw_write8(Adapter, REG_AFE_PLL_CTRL, 0x80);

	// n.	SPS0_CTRL 0x11[7:0] = 0x22			//enter PFM mode
	if(!IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
		rtw_write8(Adapter, REG_SPS0_CTRL, 0x23);

#ifdef CONFIG_BT_COEXIST
	if(	(pHalData->bt_coexist.BT_Coexist) /*&&
		(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4)	*/)
	{ // when BT COEX exist
		u4bTmp = rtw_read32(Adapter, REG_AFE_XTAL_CTRL);

		//AFE_XTAL_CTRL+2 0x26[9:7] = 3b'111
		//AFE_XTAL_CTRL+2 0x26[1] = 1
		//AFE_XTAL_CTRL 0x24[14] = 1
		//AFE_XTAL_CTRL 0x24[11] = 1
		u4bTmp |= 0x03824800;
		rtw_write32(Adapter, REG_AFE_XTAL_CTRL, u4bTmp);
	}
	else
#endif
	{
		// o.	AFE_XTAL_CTRL 0x24[7:0] = 0x0E		// disable XTAL, if No BT COEX
		rtw_write8(Adapter, REG_AFE_XTAL_CTRL, 0x0e);
	}
	
	// p.	RSV_CTRL 0x1C[7:0] = 0x0E			// lock ISO/CLK/Power control register
	//rtw_write8(Adapter, REG_RSV_CTRL, 0x0e);
	
	//  ==== interface into suspend ===

	// q.	APS_FSMCO[15:8] = 0x58				// PCIe suspend mode
	//PlatformEFIOWrite1Byte(Adapter, REG_APS_FSMCO+1, 0x58);
	// by tynli. Suggested by SD1.
	// According to power document V11, we need to set this value as 0x18. Otherwise, we
	// may not L0s sometimes. This indluences power consumption. Bases on SD1's test,
	// set as 0x00 do not affect power current. And if it is set as 0x18, they had ever
	// met auto load fail problem. 2009/12/03 MH/Tylin/Alan add the description.
	rtw_write8(Adapter, REG_APS_FSMCO+1, 0x10); //tynli_test. SD1 2009.12.08.

	// p.	RSV_CTRL 0x1C[7:0] = 0x0E			// lock ISO/CLK/Power control register
	rtw_write8(Adapter, REG_RSV_CTRL, 0x0e);

	// r.	Note: for PCIe interface, PON will not turn off m-bias and BandGap 
	// in PCIe suspend mode. 


	// 2009/10/16 MH SD1 Victor need the test for isolation.
	// tynli_test set BIT0 to 1. moved to shutdown
	//rtw_write8(Adapter, 0x0, PlatformEFIORead1Byte(Adapter, 0x0)|BIT0);
	

	// 2009/10/13 MH Disable 92SE card disable sequence.
	DBG_8192C("<=======PowerOffAdapter8192CE()\n");
	
}	// PowerOffAdapter8192CE

//
// Description: For WoWLAN, when D0 support PME, we should clear PME status from 0x81
//			to 0x01 to prevent S3/S4 hang. Suggested by SD1 Jonbon/Isaac.
//
//	2009.04. by tynli.
static VOID
PlatformClearPciPMEStatus(
	IN		PADAPTER		Adapter	
)
{
	struct dvobj_priv	*pdvobjpriv = &Adapter->dvobjpriv;
	struct pci_dev 	*pdev = pdvobjpriv->ppcidev;
	u8	value_offset, value, tmp;
	int	result;

	result = pci_read_config_byte(pdev, 0x34, &value_offset);

	DBG_8192C("PlatformClearPciPMEStatus(): PCI configration 0x34 = 0x%2x\n", value_offset);

	if(result != 0) //returns how many bytes of caller-supplied data it wrote
	{
		DBG_8192C("PlatformClearPciPMEStatus() Failed!(%d)\n",result);
	}
	else
	{
		do
		{
			if(value_offset == 0x00) //end of pci capability
			{
				value = 0xff;
				break;
			}

			result = pci_read_config_byte(pdev, value_offset, &value);

			DBG_8192C("PlatformClearPciPMEStatus(): in pci configration1, value_offset%x = %x\n", value_offset, value);

			if(result != 1) //returns how many bytes of caller-supplied data it wrote
			{
				DBG_8192C("PlatformClearPciPMEStatus() Failed!(%d)\n",result);
			}
			else
			{
				if(value == 0x01)
					break;
				else
				{
					value_offset = value_offset + 0x1;
					result = pci_read_config_byte(pdev, value_offset, &value);
					//DBG_8192C("PlatformClearPciPMEStatus(): in pci configration2, value_offset%x = %x\n", value_offset, value);
					value_offset = value;
				}
			}
		}while(_TRUE);
		
		if(value == 0x01)
		{
			value_offset = value_offset + 0x05;
			result = pci_read_config_byte(pdev, value_offset, &value);
			//DBG_8192C("*** 1 PME value = %x \n", value);
			if(value & BIT7)
			{
				tmp = value | BIT7;
				pci_write_config_byte(pdev, value_offset, tmp);

				pci_read_config_byte(pdev, value_offset, &tmp);
				//DBG_8192C("*** 2 PME value = %x \n", tmp);
				DBG_8192C("PlatformClearPciPMEStatus(): Clear PME status 0x%2x to 0x%2x\n", value_offset, tmp);
			}
			else
				DBG_8192C("PlatformClearPciPMEStatus(): PME status(0x%2x) = 0x%2x\n", value_offset, value);
		}
	}
	//DBG_8192C("PlatformClearPciPMEStatus(): PME_status offset = %x, EN = %x\n", value_offset, PCIClkReq);

}

static u32 rtl8192ce_hal_deinit(PADAPTER Adapter)
{
	u8	u1bTmp = 0;
	u8	bSupportRemoteWakeUp = _FALSE;
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	struct pwrctrl_priv		*pwrpriv = &Adapter->pwrctrlpriv;
	
_func_enter_;

	if (Adapter->bHaltInProgress == _TRUE)
	{
		DBG_8192C("====> Abort rtl8192ce_hal_deinit()\n");
		return _FAIL;
	}

	Adapter->bHaltInProgress = _TRUE;

	//
	// No I/O if device has been surprise removed
	//
	if (Adapter->bSurpriseRemoved)
	{
		Adapter->bHaltInProgress = _FALSE;
		return _SUCCESS;
	}

	Adapter->bDriverIsGoingToUnload = _TRUE;

	RT_SET_PS_LEVEL(pwrpriv, RT_RF_OFF_LEVL_HALT_NIC);

	// Without supporting WoWLAN or the driver is in awake (D0) state, we should 
	// call PowerOffAdapter8192CE() to run the power sequence. 2009.04.23. by tynli.
	if(!bSupportRemoteWakeUp )//||!pMgntInfo->bPwrSaveState) 
	{
		// 2009/10/13 MH For power off test.
		PowerOffAdapter8192CE(Adapter);	
	}
	else
	{
		u8	bSleep = _TRUE;

		//RxDMA
		//tynli_test 2009.12.16.
		u1bTmp = rtw_read8(Adapter, REG_RXPKT_NUM+2);
		rtw_write8(Adapter, REG_RXPKT_NUM+2, u1bTmp|BIT2);	

		//PlatformDisableASPM(Adapter);
		Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_SWITCH_EPHY_WoWLAN, (u8 *)&bSleep);

		//tynli_test. 2009.12.17.
		u1bTmp = rtw_read8(Adapter, REG_SPS0_CTRL);
		rtw_write8(Adapter, REG_SPS0_CTRL, (u1bTmp|BIT1));

		//
		rtw_write8(Adapter, REG_APS_FSMCO+1, 0x0);

		PlatformClearPciPMEStatus(Adapter);

		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			// tynli_test for normal chip wowlan. 2010.01.26. Suggested by Sd1 Isaac and designer Alfred.
			rtw_write8(Adapter, REG_SYS_CLKR, (rtw_read8(Adapter, REG_SYS_CLKR)|BIT3));

			//prevent 8051 to be reset by PERST#
			rtw_write8(Adapter, REG_RSV_CTRL, 0x20);
			rtw_write8(Adapter, REG_RSV_CTRL, 0x60);
		}	
	}

	rtl8192ce_gen_RefreshLedState(Adapter);

	Adapter->bHaltInProgress = _FALSE;

_func_exit_;

	return _SUCCESS;
}

static void StopTxBeacon(_adapter *padapter)
{
	HAL_DATA_TYPE*	pHalData = GET_HAL_DATA(padapter);
	u8 			tmp1Byte = 0;

	if(IS_81XXC_TEST_CHIP(pHalData->VersionID))
	{
		tmp1Byte = rtw_read8(padapter, REG_TXPAUSE);
		rtw_write8(padapter, REG_TXPAUSE, tmp1Byte | BIT6);
	 }
	else
	 {	/*	Aries , reference usbpart, 20120321	*/
	 	rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) & (~BIT6));
		pHalData->RegFwHwTxQCtrl &= (~BIT6);
		
//		tmp1Byte = rtw_read8(padapter, REG_FWHW_TXQ_CTRL+2);
//		rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, tmp1Byte & (~BIT6));
//		rtw_write8(padapter, REG_TBTT_PROHIBIT+1, 0x64);
//		tmp1Byte = rtw_read8(padapter, REG_TBTT_PROHIBIT+2);
//		tmp1Byte &= ~(BIT0);
//		rtw_write8(padapter, REG_TBTT_PROHIBIT+2, tmp1Byte);

	 }
	
	 //CheckFwRsvdPageContent(padapter);  // 2010.06.23. Added by tynli.
}

static void ResumeTxBeacon(_adapter *padapter)
{
	HAL_DATA_TYPE*	pHalData = GET_HAL_DATA(padapter);	
	u8 			tmp1Byte = 0;

	if(IS_81XXC_TEST_CHIP(pHalData->VersionID))
	 {
		tmp1Byte = rtw_read8(padapter, REG_TXPAUSE);
		rtw_write8(padapter, REG_TXPAUSE, tmp1Byte & (~BIT6));
	 }
	else
	 {	/*	Aries , reference usb part, 20120312	*/
	 	rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) | BIT6);
		pHalData->RegFwHwTxQCtrl |= BIT6;
		
//		tmp1Byte = rtw_read8(padapter, REG_FWHW_TXQ_CTRL+2);
//		rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, tmp1Byte | BIT6);
//		rtw_write8(padapter, REG_TBTT_PROHIBIT+1, 0xff);
//		tmp1Byte = rtw_read8(padapter, REG_TBTT_PROHIBIT+2);
//		tmp1Byte |= BIT0;
//		rtw_write8(padapter, REG_TBTT_PROHIBIT+2, tmp1Byte);

	}
}

static void hw_var_set_opmode(PADAPTER Adapter, u8 variable, u8* val)
{
	u8	val8;
	u8	mode = *((u8 *)val);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);


#ifdef CONFIG_CONCURRENT_MODE
	if(Adapter->iface_type == IFACE_PORT1)
	{
		// disable Port1 TSF update
		rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(4));
		
		// set net_type
		val8 = rtw_read8(Adapter, MSR)&0x03;
		val8 |= (mode<<2);
		rtw_write8(Adapter, MSR, val8);
		
		//reset TSF1	
		rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(1));

		DBG_871X("%s()-%d mode = %d\n", __FUNCTION__, __LINE__, mode);
	
		if((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_))
		{
			if(!check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE))			
			{
				StopTxBeacon(Adapter);
#ifdef CONFIG_CONCURRENT_MODE
				UpdateInterruptMask8192CE( Adapter, 0, 0, RT_IBSS_INT_MASKS, IMR_BcnInt_E);
#else
				UpdateInterruptMask8192CE( Adapter, 0, 0, RT_IBSS_INT_MASKS, 0);
#endif
			}
			rtw_write8(Adapter,REG_BCN_CTRL_1, 0x19);//disable atim wnd
			//rtw_write8(Adapter,REG_BCN_CTRL_1, 0x18);

		}
		else if((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_)*/)
		{
                        rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
			ResumeTxBeacon(Adapter);
			rtw_write8(Adapter,REG_BCN_CTRL_1, 0x1a);
		}
		else if(mode == _HW_STATE_AP_)
		{
			rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
			ResumeTxBeacon(Adapter);
			
			rtw_write8(Adapter, REG_BCN_CTRL_1, 0x12);

			//Set RCR
			//rtw_write32(padapter, REG_RCR, 0x70002a8e);//CBSSID_DATA must set to 0
			rtw_write32(Adapter, REG_RCR, 0x7000228e);//CBSSID_DATA must set to 0
			//enable to rx data frame				
			rtw_write16(Adapter, REG_RXFLTMAP2, 0xFFFF);
			//enable to rx ps-poll
			rtw_write16(Adapter, REG_RXFLTMAP1, 0x0400);

			//Beacon Control related register for first time 
			rtw_write8(Adapter, REG_BCNDMATIM, 0x02); // 2ms		
			rtw_write8(Adapter, REG_DRVERLYINT, 0x05);// 5ms
			//rtw_write8(Adapter, REG_BCN_MAX_ERR, 0xFF);
			rtw_write8(Adapter, REG_ATIMWND_1, 0x0a); // 10ms for port1
			rtw_write16(Adapter, REG_BCNTCFG, 0x00);
			rtw_write16(Adapter, REG_TBTT_PROHIBIT, 0xff04);
			rtw_write16(Adapter, REG_TSFTR_SYN_OFFSET, 0x7fff);// +32767 (~32ms)
	

		       	//enable BCN1 Function for if2
			//don't enable update TSF1 for if2 (due to TSF update when beacon/probe rsp are received)
			rtw_write8(Adapter, REG_BCN_CTRL_1, (DIS_TSF_UDT0_NORMAL_CHIP|EN_BCN_FUNCTION | EN_TXBCN_RPT|BIT(1)));

#ifdef CONFIG_CONCURRENT_MODE
			if(check_buddy_fwstate(Adapter, WIFI_FW_NULL_STATE))
				rtw_write8(Adapter, REG_BCN_CTRL, 
					rtw_read8(Adapter, REG_BCN_CTRL) & ~EN_BCN_FUNCTION);
#endif

                        //BCN1 TSF will sync to BCN0 TSF with offset(0x518) if if1_sta linked
			//rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(5));
			//rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(3));
					
			//dis BCN0 ATIM  WND if if1 is station
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(0));

#ifdef CONFIG_TSF_RESET_OFFLOAD
			// Reset TSF for STA+AP concurrent mode
			if ( check_buddy_fwstate(Adapter, (WIFI_STATION_STATE|WIFI_ASOC_STATE)) ) {
				if (reset_tsf(Adapter, IFACE_PORT1) == _FALSE)
					DBG_871X("ERROR! %s()-%d: Reset port1 TSF fail\n",
						__FUNCTION__, __LINE__);
			}
#endif	// CONFIG_TSF_RESET_OFFLOAD
#ifdef CONFIG_CONCURRENT_MODE
			UpdateInterruptMask8192CE( Adapter, RT_IBSS_INT_MASKS, IMR_BcnInt_E, 0, 0);
#else
			UpdateInterruptMask8192CE( Adapter, RT_IBSS_INT_MASKS, 0, 0, 0);
#endif
		}

	}
	else	// (Adapter->iface_type == IFACE_PORT1)
#endif //CONFIG_CONCURRENT_MODE
	{
		// disable Port0 TSF update
		rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));
		
		// set net_type
		val8 = rtw_read8(Adapter, MSR)&0x0c;
		val8 |= mode;
		rtw_write8(Adapter, MSR, val8);
		
		//reset TSF0	
		rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(0));

		DBG_871X("%s()-%d mode = %d\n", __FUNCTION__, __LINE__, mode);
		
		if((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_))
		{
#ifdef CONFIG_CONCURRENT_MODE
			if(!check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE))		
#endif //CONFIG_CONCURRENT_MODE
			{
				StopTxBeacon(Adapter);
#ifdef CONFIG_CONCURRENT_MODE
				UpdateInterruptMask8192CE(Adapter,  0, 0, RT_IBSS_INT_MASKS, IMR_BcnInt_E);
#else
				UpdateInterruptMask8192CE(Adapter, 0, 0, RT_IBSS_INT_MASKS, 0);
#endif
			}
			
			rtw_write8(Adapter,REG_BCN_CTRL, 0x19);//disable atim wnd
			//rtw_write8(Adapter,REG_BCN_CTRL, 0x18);
		}
		else if((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_)*/)
		{
			rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
			ResumeTxBeacon(Adapter);
			rtw_write8(Adapter,REG_BCN_CTRL, 0x1a);
		}
		else if(mode == _HW_STATE_AP_)
		{
			rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
			ResumeTxBeacon(Adapter);

			rtw_write8(Adapter, REG_BCN_CTRL, 0x12);
					
			//Set RCR
			//write32(padapter, REG_RCR, 0x70002a8e);//CBSSID_DATA must set to 0
			rtw_write32(Adapter, REG_RCR, 0x7000228e);//CBSSID_DATA must set to 0
			//enable to rx data frame				
			rtw_write16(Adapter, REG_RXFLTMAP2, 0xFFFF);
			//enable to rx ps-poll
			rtw_write16(Adapter, REG_RXFLTMAP1, 0x0400);

			//Beacon Control related register for first time 
			rtw_write8(Adapter, REG_BCNDMATIM, 0x02); // 2ms		
			rtw_write8(Adapter, REG_DRVERLYINT, 0x05);// 5ms
			//write8(Adapter, REG_BCN_MAX_ERR, 0xFF);
			rtw_write8(Adapter, REG_ATIMWND, 0x0a); // 10ms for port0
			rtw_write16(Adapter, REG_BCNTCFG, 0x00);
			rtw_write16(Adapter, REG_TBTT_PROHIBIT, 0xff04);
			rtw_write16(Adapter, REG_TSFTR_SYN_OFFSET, 0x7fff);// +32767 (~32ms)
	
		        //enable BCN0 Function for if1
			//don't enable update TSF0 for if1 (due to TSF update when beacon/probe rsp are received)
			rtw_write8(Adapter, REG_BCN_CTRL, (DIS_TSF_UDT0_NORMAL_CHIP|EN_BCN_FUNCTION | EN_TXBCN_RPT|BIT(1)));
					
#ifdef CONFIG_CONCURRENT_MODE
			if(check_buddy_fwstate(Adapter, WIFI_FW_NULL_STATE))
				rtw_write8(Adapter, REG_BCN_CTRL_1, 
					rtw_read8(Adapter, REG_BCN_CTRL_1) & ~EN_BCN_FUNCTION);
#endif
			//BCN1 TSF will sync to BCN0 TSF with offset(0x518) if if1_sta linked
			//only interface 2 as AP MODE need to sync
			//rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(5));
			
							
			//dis BCN1 ATIM  WND if if2 is station
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(0));
#ifdef CONFIG_CONCURRENT_MODE
			UpdateInterruptMask8192CE( Adapter, RT_IBSS_INT_MASKS, IMR_BcnInt_E, 0, 0);
#else
			UpdateInterruptMask8192CE( Adapter, RT_IBSS_INT_MASKS, 0, 0, 0);
#endif


#ifdef CONFIG_TSF_RESET_OFFLOAD
			// Reset TSF for STA+AP concurrent mode
			if ( check_buddy_fwstate(Adapter, (WIFI_STATION_STATE|WIFI_ASOC_STATE)) ) {
				if (reset_tsf(Adapter, IFACE_PORT0) == _FALSE)
					DBG_871X("ERROR! %s()-%d: Reset port0 TSF fail\n",
						__FUNCTION__, __LINE__);
			}
#endif	// CONFIG_TSF_RESET_OFFLOAD			
					
		}


	}

}

static void hw_var_set_macaddr(PADAPTER Adapter, u8 variable, u8* val)
{
	u8 idx = 0;
	u32 reg_macid;

#ifdef CONFIG_CONCURRENT_MODE
	if(Adapter->iface_type == IFACE_PORT1)
	{
		reg_macid = REG_MACID1;
	}
	else
#endif
	{
		reg_macid = REG_MACID;
	}

	for(idx = 0 ; idx < 6; idx++)
	{
		rtw_write8(Adapter, (reg_macid+idx), val[idx]);
	}
	
}

static void hw_var_set_bssid(PADAPTER Adapter, u8 variable, u8* val)
{
	u8	idx = 0;
	u32 reg_bssid;

#ifdef CONFIG_CONCURRENT_MODE
	if(Adapter->iface_type == IFACE_PORT1)
	{
		reg_bssid = REG_BSSID1;
	}
	else
#endif
	{
		reg_bssid = REG_BSSID;
	}

	for(idx = 0 ; idx < 6; idx++)
	{
		rtw_write8(Adapter, (reg_bssid+idx), val[idx]);
	}

}

static void hw_var_set_bcn_func(PADAPTER Adapter, u8 variable, u8* val)
{
	u32 bcn_ctrl_reg;

#ifdef CONFIG_CONCURRENT_MODE
	if(Adapter->iface_type == IFACE_PORT1)
	{
		bcn_ctrl_reg = REG_BCN_CTRL_1;

		if(*((u8 *)val))
		{
			rtw_write8(Adapter, bcn_ctrl_reg, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
		}
		else
		{
			rtw_write8(Adapter, bcn_ctrl_reg, rtw_read8(Adapter, bcn_ctrl_reg)&(~(EN_BCN_FUNCTION | EN_TXBCN_RPT)));
		}
	}	
	else
#endif		
	{		
		bcn_ctrl_reg = REG_BCN_CTRL;
		if(*((u8 *)val))
		{
			rtw_write8(Adapter, bcn_ctrl_reg, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
		}
		else
		{
			//rtw_write8(Adapter, bcn_ctrl_reg, rtw_read8(Adapter, bcn_ctrl_reg)&(~(EN_BCN_FUNCTION | EN_TXBCN_RPT)));
			rtw_write8(Adapter, bcn_ctrl_reg, (rtw_read8(Adapter, bcn_ctrl_reg)&(~(EN_TXBCN_RPT))) | DIS_TSF_UDT0_NORMAL_CHIP);
		}
	}
}

static void hw_var_set_correct_tsf(PADAPTER Adapter, u8 variable, u8* val)
{
#ifdef CONFIG_CONCURRENT_MODE
	u64	tsf;
	struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

	//tsf = pmlmeext->TSFValue - ((u32)pmlmeext->TSFValue % (pmlmeinfo->bcn_interval*1024)) -1024; //us
	tsf = pmlmeext->TSFValue - rtw_modular64(pmlmeext->TSFValue, (pmlmeinfo->bcn_interval*1024)) -1024; //us

	if(((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//pHalData->RegTxPause |= STOP_BCNQ;BIT(6)
		//rtw_write8(Adapter, REG_TXPAUSE, (rtw_read8(Adapter, REG_TXPAUSE)|BIT(6)));
		StopTxBeacon(Adapter);
	}

	if(Adapter->iface_type == IFACE_PORT1)
	{
		//disable related TSF function
		rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(3)));
							
		rtw_write32(Adapter, REG_TSFTR1, tsf);
		rtw_write32(Adapter, REG_TSFTR1+4, tsf>>32);

		//enable related TSF function
		rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(3));


#ifdef CONFIG_TSF_RESET_OFFLOAD
		// Update buddy port's TSF(TBTT) if it is SoftAP for beacon TX issue!
		if ( (pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE
			&& check_buddy_fwstate(Adapter, WIFI_AP_STATE) ) { 
			if (reset_tsf(Adapter, IFACE_PORT0) == _FALSE)
				DBG_871X("ERROR! %s()-%d: Reset port0 TSF fail\n",
					__FUNCTION__, __LINE__);
		}
#endif	// CONFIG_TSF_RESET_OFFLOAD

	}
	else	// Adapter->iface_type == IFACE_PORT1
	{
		//disable related TSF function
		rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(3)));
		// disable TSF update instead! May induce burst beacon TX
		//rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));
							
		rtw_write32(Adapter, REG_TSFTR, tsf);
		rtw_write32(Adapter, REG_TSFTR+4, tsf>>32);

		//enable related TSF function
		rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(3));
		//rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));		
		
		// Update buddy port's TSF if it is SoftAP for beacon TX issue!
		if ( (pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE
			&& check_buddy_fwstate(Adapter, WIFI_AP_STATE)
		) { 
			//disable related TSF function
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(3)));
			// disable TSF update instead!
			//rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(4));

			rtw_write32(Adapter, REG_TSFTR1, tsf);
			rtw_write32(Adapter, REG_TSFTR1+4, tsf>>32);

			//enable related TSF function
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(3));
			//rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(4)));
		}		
#ifdef CONFIG_TSF_RESET_OFFLOAD
		// Update buddy port's TSF if it is SoftAP for beacon TX issue!
		if ( (pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE
			&& check_buddy_fwstate(Adapter, WIFI_AP_STATE) ) {
			if (reset_tsf(Adapter, IFACE_PORT1) == _FALSE)
				DBG_871X("ERROR! %s()-%d: Reset port1 TSF fail\n",
					__FUNCTION__, __LINE__);
		}
#endif	// CONFIG_TSF_RESET_OFFLOAD
	}
				
							
	if(((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//pHalData->RegTxPause  &= (~STOP_BCNQ);
		//rtw_write8(Adapter, REG_TXPAUSE, (rtw_read8(Adapter, REG_TXPAUSE)&(~BIT(6))));
		ResumeTxBeacon(Adapter);
	}
#endif	// CONFIG_CONCURRENT_MODE
}

static void hw_var_set_mlme_disconnect(PADAPTER Adapter, u8 variable, u8* val)
{
#ifdef CONFIG_CONCURRENT_MODE
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	PADAPTER pbuddy_adapter = Adapter->pbuddy_adapter;

				
	if(check_buddy_mlmeinfo_state(Adapter, _HW_STATE_NOLINK_))	
		rtw_write16(Adapter, REG_RXFLTMAP2, 0x00);
	

	if(Adapter->iface_type == IFACE_PORT1)
	{
		int i;
		u8 reg_bcn_ctrl_1;

	   	// a.Driver set 0x422 bit 6 =0
	   	pHalData->RegFwHwTxQCtrl = rtw_read8(Adapter, REG_FWHW_TXQ_CTRL+2);
		rtw_write8(Adapter, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) & (~BIT6));
		pHalData->RegFwHwTxQCtrl &= (~BIT6);


#ifdef CONFIG_BEACON_DISABLE_OFFLOAD
		u8 reg_bcn_disable_cnt = rtw_read8(Adapter, REG_FW_BCN_DIS_CNT);
		DBG_871X("%s()-%d: reg_bcn_disable_cnt=%02x\n", __FUNCTION__, __LINE__, reg_bcn_disable_cnt);
	   	
		reg_bcn_ctrl_1 = rtw_read8(Adapter, REG_BCN_CTRL_1); 
		DBG_871X("%s()-%d: reg_bcn_ctrl_1=%02x\n", __FUNCTION__, __LINE__, reg_bcn_ctrl_1);

	   	// b. driver set h2c cmd
	   	rtl8192c_dis_beacon_fun_cmd(Adapter);	   
	 
		/*
			// FW Job for port 0
			
		   c. 8051 set nettype to ap
		   d. 8051 check dma_int
		   e. 8051 set nettype to no_link
		   f.8051 dis_tsf_update   0x550 bit 4
		   g.8051 reset  beacon function test count   0x553 bit0.
		   h.8051 disable beacon function   0x550  bit3
		   i. 8051 sent ready to driver

	 	*/
	 	
	 	// The worst case is 100 + 15 ms
		rtw_msleep_os(120);
		
		for (i=0; i< 10; i++) {
			reg_bcn_ctrl_1 = rtw_read8(Adapter, REG_BCN_CTRL_1); 
			if ( (reg_bcn_ctrl_1 & BIT(3)) == 0 ) {
				//DBG_871X("%s()-%d: BEACON_DISABLE_OFFLOAD finished! reg=%02x\n", __FUNCTION__, __LINE__, reg);
				break;
			}
			DBG_871X("%s()-%d: BEACON_DISABLE_OFFLOAD not finished! REG_BCN_CTRL_1=%02x\n", __FUNCTION__, __LINE__, reg_bcn_ctrl_1);
			DBG_871X("%s()-%d: reg_bcn_disable_cnt=%02x\n", __FUNCTION__, __LINE__, rtw_read8(Adapter, REG_FW_BCN_DIS_CNT));
			DBG_871X("%s()-%d: REG_BCN_CTRL=%02x\n", __FUNCTION__, __LINE__, rtw_read8(Adapter, REG_BCN_CTRL));
			DBG_871X("%s()-%d: FWISR=%08x\n", __FUNCTION__, __LINE__, rtw_read32(Adapter, REG_FWISR));
			rtw_msleep_os(100);
		}
		DBG_871X("%s()-%d: reg_bcn_disable_cnt=%02x\n", __FUNCTION__, __LINE__, rtw_read8(Adapter, REG_FW_BCN_DIS_CNT));
		DBG_871X("%s()-%d: reg_bcn_ctrl_1=%02x\n", __FUNCTION__, __LINE__, reg_bcn_ctrl_1);

#else   // CONFIG_BEACON_DISABLE_OFFLOAD

		//disable update TSF1
		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(4));	
		}
		else
		{
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(4)|BIT(5));
		}

		//reset TSF1
		rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(1));

		// disable Port1's beacon function
		rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(3)));

#endif  // CONFIG_BEACON_DISABLE_OFFLOAD
	 	
		// j, Driver set 0x422 bit 6 =1
		pHalData->RegFwHwTxQCtrl = rtw_read8(Adapter, REG_FWHW_TXQ_CTRL+2);
		rtw_write8(Adapter, REG_FWHW_TXQ_CTRL+2, (pHalData->RegFwHwTxQCtrl) | BIT6);
		pHalData->RegFwHwTxQCtrl |= BIT6;

		// k. re_download beacon pkt 
		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE))	
			set_tx_beacon_cmd(pbuddy_adapter);
		   

	}
	else	// (Adapter->iface_type == IFACE_PORT1)
	{
		//disable update TSF
		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));	
		}
		else
		{
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4)|BIT(5));
		}

		//reset TSF
		rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(0));

		// Can't disable Port0's beacon function due to it is used by RA
	}
#endif
}

static void hw_var_set_mlme_sitesurvey(PADAPTER Adapter, u8 variable, u8* val)
{	
#ifdef CONFIG_CONCURRENT_MODE	
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);

	struct mlme_ext_priv *pmlmeext = &Adapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);
#ifdef CONFIG_TDLS
	struct tdls_info *ptdlsinfo = &Adapter->pbuddy_adapter->tdlsinfo;
#endif //CONFIG_TDLS

	if(*((u8 *)val))//under sitesurvey
	{
		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			//config RCR to receive different BSSID & not to receive data frame
			//pHalData->ReceiveConfig &= (~(RCR_CBSSID_DATA | RCR_CBSSID_BCN));			
			u32 v = rtw_read32(Adapter, REG_RCR);
			v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN );//| RCR_ADF
			rtw_write32(Adapter, REG_RCR, v);

			//disable update TSF
			if((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
			{
				if(Adapter->iface_type == IFACE_PORT1)
				{
					rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)|BIT(4));
				}
				else
				{
					rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));
				}				
			}
	
		}	
		else
		{
			//config RCR to receive different BSSID & not to receive data frame			
			rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR) & 0xfffff7bf);
			//disable update TSF
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4)|BIT(5));
		}
				
		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE) &&
			check_buddy_fwstate(Adapter, _FW_LINKED))		
		{
			StopTxBeacon(Adapter);
		}		
				
	}
	else//sitesurvey done
	{
		//enable to rx data frame
		//write32(Adapter, REG_RCR, read32(padapter, REG_RCR)|RCR_ADF);
		rtw_write16(Adapter, REG_RXFLTMAP2,0xFFFF);

		//enable update TSF
		if(IS_NORMAL_CHIP(pHalData->VersionID))
		{
			if(Adapter->iface_type == IFACE_PORT1)
				rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(4)));
			else
				rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));

			if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE ||
				check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE))
			{
				rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_BCN);
			}
#ifdef CONFIG_TDLS		
			else	 if ( ptdlsinfo->setup_state == TDLS_LINKED_STATE )
			{
				rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_BCN);
			}
#endif // CONFIG_TDLS				
			else	
			{
				rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
			}
		}
		else
		{
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(BIT(4)|BIT(5))));
			rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA);
		}
					
		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE) &&
			check_buddy_fwstate(Adapter, _FW_LINKED))
		{
			ResumeTxBeacon(Adapter);			
		}
					
	}
#endif			
}

static void hw_var_set_mlme_join(PADAPTER Adapter, u8 variable, u8* val)
{
#ifdef CONFIG_CONCURRENT_MODE
	u8	RetryLimit = 0x30;
	u8	type = *((u8 *)val);
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	struct mlme_priv	*pmlmepriv = &Adapter->mlmepriv;

	if(type == 0) // prepare to join
	{		
		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE) &&
			check_buddy_fwstate(Adapter, _FW_LINKED))		
		{
			StopTxBeacon(Adapter);
		}
	
		//enable to rx data frame.Accept all data frame
		//rtw_write32(padapter, REG_RCR, rtw_read32(padapter, REG_RCR)|RCR_ADF);
		rtw_write16(Adapter, REG_RXFLTMAP2,0xFFFF);

		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE))
			rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_BCN);
		else
			rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);

		if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE)
		{
			RetryLimit = (pHalData->CustomerID == RT_CID_CCX) ? 7 : 48;
		}
		else // Ad-hoc Mode
		{
			RetryLimit = 0x7;
		}
	}
	else if(type == 1) //joinbss_event call back when join res < 0
	{		
		if(check_buddy_mlmeinfo_state(Adapter, _HW_STATE_NOLINK_))		
			rtw_write16(Adapter, REG_RXFLTMAP2,0x00);

		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE) &&
			check_buddy_fwstate(Adapter, _FW_LINKED))
		{
			ResumeTxBeacon(Adapter);			
			
			//reset TSF 1/2 after ResumeTxBeacon
			//rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(1)|BIT(0));	
			
		}
	}
	else if(type == 2) //sta add event call back
	{
	 
		//enable update TSF
		if(Adapter->iface_type == IFACE_PORT1)
			rtw_write8(Adapter, REG_BCN_CTRL_1, rtw_read8(Adapter, REG_BCN_CTRL_1)&(~BIT(4)));
		else
			rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));
		 
	
		if(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE))
		{
			//fixed beacon issue for 8191su...........
			rtw_write8(Adapter,0x542 ,0x02);
			RetryLimit = 0x7;
		}


		if(check_buddy_mlmeinfo_state(Adapter, WIFI_FW_AP_STATE) &&
			check_buddy_fwstate(Adapter, _FW_LINKED))
		{
			ResumeTxBeacon(Adapter);
			
			//reset TSF 1/2 after ResumeTxBeacon
			//rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(1)|BIT(0));
		}
		
	}

	rtw_write16(Adapter, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
	
#endif
}
void SetHwReg8192CE(PADAPTER Adapter, u8 variable, u8* val)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;

_func_enter_;
	
	switch(variable)
	{
		case HW_VAR_MEDIA_STATUS:
			{
				u8 val8;

				val8 = rtw_read8(Adapter, MSR)&0x0c;
				val8 |= *((u8 *)val);
				rtw_write8(Adapter, MSR, val8);
			}
			break;
		case HW_VAR_MEDIA_STATUS1:
			{
				u8 val8;
				
				val8 = rtw_read8(Adapter, MSR)&0x03;
				val8 |= *((u8 *)val) <<2;
				rtw_write8(Adapter, MSR, val8);
			}
			break;
		case HW_VAR_SET_OPMODE:
#if 1
			hw_var_set_opmode(Adapter, variable, val);
#else			
			{
				u8	val8;
				u8	mode = *((u8 *)val);

				if((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_))
				{
					StopTxBeacon(Adapter);
					rtw_write8(Adapter,REG_BCN_CTRL, 0x18);
					UpdateInterruptMask8192CE(Adapter, 0, RT_BSS_INT_MASKS);
				}
				else if((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_)*/)
				{
					rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
					ResumeTxBeacon(Adapter);
					rtw_write8(Adapter,REG_BCN_CTRL, 0x1a);
				}
				else if(mode == _HW_STATE_AP_)
				{

					rtw_write16(Adapter, REG_CR, rtw_read16(Adapter, REG_CR)|BIT(8));
				
					ResumeTxBeacon(Adapter);
					
					rtw_write8(Adapter, REG_BCN_CTRL, 0x12);

					
					//Set RCR
					//rtw_write32(padapter, REG_RCR, 0x70002a8e);//CBSSID_DATA must set to 0
					rtw_write32(Adapter, REG_RCR, 0x7000228e);//CBSSID_DATA must set to 0
					//enable to rx data frame				
					rtw_write16(Adapter, REG_RXFLTMAP2, 0xFFFF);
					//enable to rx ps-poll
					rtw_write16(Adapter, REG_RXFLTMAP1, 0x0400);

					//Beacon Control related register for first time 
					rtw_write8(Adapter, REG_BCNDMATIM, 0x02); // 2ms		
					rtw_write8(Adapter, REG_DRVERLYINT, 0x05);// 5ms
					//rtw_write8(Adapter, REG_BCN_MAX_ERR, 0xFF);
					rtw_write8(Adapter, REG_ATIMWND, 0x0a); // 10ms
					rtw_write16(Adapter, REG_BCNTCFG, 0x00);
					rtw_write16(Adapter, REG_TBTT_PROHIBIT, 0x6404);		
	
					//reset TSF		
					rtw_write8(Adapter, REG_DUAL_TSF_RST, BIT(0));

					//enable TSF Function for if1
					rtw_write8(Adapter, REG_BCN_CTRL, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
			
					//enable update TSF for if1
					if(IS_NORMAL_CHIP(pHalData->VersionID))
					{			
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));			
					}
					else
					{
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(BIT(4)|BIT(5))));
					}
					
					UpdateInterruptMask8192CE(Adapter, RT_BSS_INT_MASKS, 0);
					
				}

				val8 = rtw_read8(Adapter, MSR)&0x0c;
				val8 |= mode;
				rtw_write8(Adapter, MSR, val8);
			}
#endif
			break;
                case HW_VAR_MAC_ADDR:
			hw_var_set_macaddr(Adapter, variable, val);			
			break;
		case HW_VAR_BSSID:
#if 1
			hw_var_set_bssid(Adapter, variable, val);
#else			
			{
				u8	idx = 0;
				for(idx = 0 ; idx < 6; idx++)
				{
					rtw_write8(Adapter, (REG_BSSID+idx), val[idx]);
				}
			}
#endif
			break;
		case HW_VAR_BASIC_RATE:
			{
				u16			BrateCfg = 0;
				u8			RateIndex = 0;
				
				// 2007.01.16, by Emily
				// Select RRSR (in Legacy-OFDM and CCK)
				// For 8190, we select only 24M, 12M, 6M, 11M, 5.5M, 2M, and 1M from the Basic rate.
				// We do not use other rates.
				rtl8192c_HalSetBrateCfg( Adapter, val, &BrateCfg );
			
				//CCK 2M ACK should be disabled for some BCM and Atheros AP IOT
				//because CCK 2M has poor TXEVM
				//CCK 5.5M & 11M ACK should be enabled for better performance
				pHalData->BasicRateSet = BrateCfg = (BrateCfg|0xd) & 0x15d;

				if(Adapter->mlmeextpriv.mlmext_info.assoc_AP_vendor == ciscoAP && ((BrateCfg &0x150)==0))
				{
					// if peer is cisco and didn't use ofdm rate,
					// we use 6M for ack.
					BrateCfg |=0x010;
				}

				BrateCfg |= 0x01; // default enable 1M ACK rate

				DBG_8192C("HW_VAR_BASIC_RATE: BrateCfg(%#x)\n", BrateCfg);

				// Set RRSR rate table.
				rtw_write8(Adapter, REG_RRSR, BrateCfg&0xff);
				rtw_write8(Adapter, REG_RRSR+1, (BrateCfg>>8)&0xff);

				// Set RTS initial rate
				while(BrateCfg > 0x1)
				{
					BrateCfg = (BrateCfg>> 1);
					RateIndex++;
				}
				rtw_write8(Adapter, REG_INIRTS_RATE_SEL, RateIndex);
			}
			break;
		case HW_VAR_TXPAUSE:
			rtw_write8(Adapter, REG_TXPAUSE, *((u8 *)val));	
			break;
		case HW_VAR_BCN_FUNC:
#if 1
			hw_var_set_bcn_func(Adapter, variable, val);
#else
			if(*((u8 *)val))
			{
				rtw_write8(Adapter, REG_BCN_CTRL, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
			}
			else
			{
				rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(EN_BCN_FUNCTION | EN_TXBCN_RPT)));
			}
#endif
			break;
		case HW_VAR_CORRECT_TSF:
#ifdef CONFIG_CONCURRENT_MODE
			hw_var_set_correct_tsf(Adapter, variable, val);
#else
			{
				u64	tsf;
				struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
				struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

				//tsf = pmlmeext->TSFValue - ((u32)pmlmeext->TSFValue % (pmlmeinfo->bcn_interval*1024)) -1024; //us
				tsf = pmlmeext->TSFValue - rtw_modular64(pmlmeext->TSFValue, (pmlmeinfo->bcn_interval*1024)) -1024; //us

				if(((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
				{				
					//pHalData->RegTxPause |= STOP_BCNQ;BIT(6)
					//rtw_write8(Adapter, REG_TXPAUSE, (rtw_read8(Adapter, REG_TXPAUSE)|BIT(6)));
					StopTxBeacon(Adapter);
				}

				//disable related TSF function
				rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(3)));
							
				rtw_write32(Adapter, REG_TSFTR, tsf);
				rtw_write32(Adapter, REG_TSFTR+4, tsf>>32);

				//enable related TSF function
				rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(3));
				
							
				if(((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) || ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
				{
					//pHalData->RegTxPause  &= (~STOP_BCNQ);
					//rtw_write8(Adapter, REG_TXPAUSE, (rtw_read8(Adapter, REG_TXPAUSE)&(~BIT(6))));
					ResumeTxBeacon(Adapter);
				}
			}
#endif
			break;
		case HW_VAR_CHECK_BSSID:
			if(*((u8 *)val))
			{
				if(IS_NORMAL_CHIP(pHalData->VersionID))
				{
					rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
				}
				else
				{
					rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA);
				}
			}
			else
			{
				u32	val32;

				val32 = rtw_read32(Adapter, REG_RCR);

				if(IS_NORMAL_CHIP(pHalData->VersionID))
				{
					val32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);
				}
				else
				{
					val32 &= 0xfffff7bf;
				}

				rtw_write32(Adapter, REG_RCR, val32);
			}
			break;
		case HW_VAR_MLME_DISCONNECT:
#ifdef CONFIG_CONCURRENT_MODE
			hw_var_set_mlme_disconnect(Adapter, variable, val);
#else
			{
				//Set RCR to not to receive data frame when NO LINK state
				//rtw_write32(Adapter, REG_RCR, rtw_read32(padapter, REG_RCR) & ~RCR_ADF);
				rtw_write16(Adapter, REG_RXFLTMAP2,0x00);

				//reset TSF
				rtw_write8(Adapter, REG_DUAL_TSF_RST, (BIT(0)|BIT(1)));

				//disable update TSF
				if(IS_NORMAL_CHIP(pHalData->VersionID))
				{
					rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));	
				}
				else
				{
					rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4)|BIT(5));
				}
			}
#endif
			break;
		case HW_VAR_MLME_SITESURVEY:
/*Aries add,  20120312*/
#ifdef CONFIG_CONCURRENT_MODE
			hw_var_set_mlme_sitesurvey(Adapter, variable,  val);
#else
			if(*((u8 *)val))//under sitesurvey
			{
				if(IS_NORMAL_CHIP(pHalData->VersionID))
				{
					//config RCR to receive different BSSID & not to receive data frame
					//pHalData->ReceiveConfig &= (~(RCR_CBSSID_DATA | RCR_CBSSID_BCN));			
					u32 v = rtw_read32(Adapter, REG_RCR);
					v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN );//| RCR_ADF
					rtw_write32(Adapter, REG_RCR, v);
					rtw_write16(Adapter, REG_RXFLTMAP2,0x00);

					//disable update TSF
					rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4));
				}	
				else
				{
					//config RCR to receive different BSSID & not to receive data frame			
					rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR) & 0xfffff7bf);


					//disable update TSF
					rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)|BIT(4)|BIT(5));
				}
			}
			else//sitesurvey done
			{
				struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
				struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
#ifdef CONFIG_TDLS
				struct tdls_info *ptdlsinfo = &Adapter->tdlsinfo;
#endif //CONFIG_TDLS

				if ((is_client_associated_to_ap(Adapter) == _TRUE) ||
					((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) )
				{
					//enable to rx data frame
					//rtw_write32(Adapter, REG_RCR, rtw_read32(padapter, REG_RCR)|RCR_ADF);
					rtw_write16(Adapter, REG_RXFLTMAP2,0xFFFF);

					//enable update TSF
					if(IS_NORMAL_CHIP(pHalData->VersionID))
					{
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));
					}
					else
					{
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(BIT(4)|BIT(5))));
					}
				}
				else if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
				{
					rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_ADF);
					
					//enable update TSF
					if(IS_NORMAL_CHIP(pHalData->VersionID))			
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));			
					else			
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(BIT(4)|BIT(5))));	
				}


				if(IS_NORMAL_CHIP(pHalData->VersionID))
				{
					if((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_BCN);
					}
#ifdef CONFIG_TDLS
					else	 if ( ptdlsinfo->setup_state == TDLS_LINKED_STATE )
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_BCN);
					}
#endif // CONFIG_TDLS
					else	
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
					}
				}
				else
				{
#ifdef CONFIG_TDLS
					if ( ptdlsinfo->setup_state == TDLS_LINKED_STATE )
					{
					
					}
					else
#endif // CONFIG_TDLS
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA);
					}
				}
			}
#endif
			break;
		case HW_VAR_MLME_JOIN:
/*Aries add,  20120312*/
#ifdef CONFIG_CONCURRENT_MODE
			hw_var_set_mlme_join(Adapter, variable,  val);
#else
			{
				u8	RetryLimit = 0x30;
				u8	type = *((u8 *)val);
				struct mlme_priv	*pmlmepriv = &Adapter->mlmepriv;

				if(type == 0) // prepare to join
				{
					//enable to rx data frame.Accept all data frame
					//rtw_write32(padapter, REG_RCR, rtw_read32(padapter, REG_RCR)|RCR_ADF);
					rtw_write16(Adapter, REG_RXFLTMAP2,0xFFFF);

					if(IS_NORMAL_CHIP(pHalData->VersionID))
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA|RCR_CBSSID_BCN);
					}
					else
					{
						rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR)|RCR_CBSSID_DATA);
					}

					if(check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE)
					{
						RetryLimit = (pHalData->CustomerID == RT_CID_CCX) ? 7 : 48;
					}
					else // Ad-hoc Mode
					{
						RetryLimit = 0x7;
					}
				}
				else if(type == 1) //joinbss_event call back when join res < 0
				{
					//if(IS_NORMAL_CHIP(pHalData->VersionID))
					//{
						//config RCR to receive different BSSID & not to receive data frame during linking				
					//	u32 v = rtw_read32(Adapter, REG_RCR);
					//	v &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN );//| RCR_ADF
					//	rtw_write32(Adapter, REG_RCR, v);
					//}	
					//else
					//{
						//config RCR to receive different BSSID & not to receive data frame during linking	
					//	rtw_write32(Adapter, REG_RCR, rtw_read32(Adapter, REG_RCR) & 0xfffff7bf);
					//}

					rtw_write16(Adapter, REG_RXFLTMAP2,0x00);
				}
				else if(type == 2) //sta add event call back
				{
					if(IS_NORMAL_CHIP(pHalData->VersionID))
					{
						//enable update TSF
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~BIT(4)));
					}
					else
					{
						//enable update TSF
						rtw_write8(Adapter, REG_BCN_CTRL, rtw_read8(Adapter, REG_BCN_CTRL)&(~(BIT(4)|BIT(5))));
					}

					if(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE))
					{
						//fixed beacon issue for 8191su...........
						//rtw_write8(Adapter,0x542 ,0x02);
						RetryLimit = 0x7;
					}
				}

				rtw_write16(Adapter, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
			}
#endif
			break;
		case HW_VAR_BEACON_INTERVAL:
			rtw_write16(Adapter, REG_BCN_INTERVAL, *((u16 *)val));
			break;
		case HW_VAR_SLOT_TIME:
			{
				u8	u1bAIFS, aSifsTime;
				struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;
				struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

				DBG_8192C("Set HW_VAR_SLOT_TIME: SlotTime(%#x)\n", val[0]);
				rtw_write8(Adapter, REG_SLOT, val[0]);

				if(pmlmeinfo->WMM_enable == 0)
				{
					if(pmlmeext->cur_wireless_mode == WIRELESS_11B)
						aSifsTime = 10;
					else
						aSifsTime = 16;
					
					u1bAIFS = aSifsTime + (2 * pmlmeinfo->slotTime);
					
					// <Roger_EXP> Temporary removed, 2008.06.20.
					rtw_write8(Adapter, REG_EDCA_VO_PARAM, u1bAIFS);
					rtw_write8(Adapter, REG_EDCA_VI_PARAM, u1bAIFS);
					rtw_write8(Adapter, REG_EDCA_BE_PARAM, u1bAIFS);
					rtw_write8(Adapter, REG_EDCA_BK_PARAM, u1bAIFS);
				}
			}
			break;
		case HW_VAR_ACK_PREAMBLE:
			{
				u8	regTmp;
				u8	bShortPreamble = *( (PBOOLEAN)val );
				// Joseph marked out for Netgear 3500 TKIP channel 7 issue.(Temporarily)
				//regTmp = (pHalData->nCur40MhzPrimeSC)<<5;
				regTmp = 0;
				if(bShortPreamble)
					regTmp |= 0x80;

				rtw_write8(Adapter, REG_RRSR+2, regTmp);
			}
			break;
		case HW_VAR_SEC_CFG:
#ifdef CONFIG_CONCURRENT_MODE
			rtw_write8(Adapter, REG_SECCFG, 0x0c|BIT(5));// enable tx enc and rx dec engine, and no key search for MC/BC				
#else
			rtw_write8(Adapter, REG_SECCFG, *((u8 *)val));
#endif
			break;
		case HW_VAR_DM_FLAG:
			pdmpriv->DMFlag = *((u8 *)val);
			break;
		case HW_VAR_DM_FUNC_OP:
			if(val[0])
			{// save dm flag
				pdmpriv->DMFlag_tmp = pdmpriv->DMFlag;
			}
			else
			{// restore dm flag
				pdmpriv->DMFlag = pdmpriv->DMFlag_tmp;
			}
			break;
		case HW_VAR_DM_FUNC_SET:
			pdmpriv->DMFlag |= *((u8 *)val);
			break;
		case HW_VAR_DM_FUNC_CLR:
			pdmpriv->DMFlag &= *((u8 *)val);
			break;
		case HW_VAR_CAM_EMPTY_ENTRY:
			{
				u8	ucIndex = *((u8 *)val);
				u8	i;
				u32	ulCommand=0;
				u32	ulContent=0;
				u32	ulEncAlgo=CAM_AES;

				for(i=0;i<CAM_CONTENT_COUNT;i++)
				{
					// filled id in CAM config 2 byte
					if( i == 0)
					{
						ulContent |=(ucIndex & 0x03) | ((u16)(ulEncAlgo)<<2);
						//ulContent |= CAM_VALID;
					}
					else
					{
						ulContent = 0;
					}
					// polling bit, and No Write enable, and address
					ulCommand= CAM_CONTENT_COUNT*ucIndex+i;
					ulCommand= ulCommand | CAM_POLLINIG|CAM_WRITE;
					// write content 0 is equall to mark invalid
					rtw_write32(Adapter, WCAMI, ulContent);  //delay_ms(40);
					//RT_TRACE(COMP_SEC, DBG_LOUD, ("CAM_empty_entry(): WRITE A4: %lx \n",ulContent));
					rtw_write32(Adapter, RWCAM, ulCommand);  //delay_ms(40);
					//RT_TRACE(COMP_SEC, DBG_LOUD, ("CAM_empty_entry(): WRITE A0: %lx \n",ulCommand));
				}
			}
			break;
		case HW_VAR_CAM_INVALID_ALL:
			rtw_write32(Adapter, RWCAM, BIT(31)|BIT(30));
			break;
		case HW_VAR_CAM_WRITE:
			{
				u32	cmd;
				u32	*cam_val = (u32 *)val;
				rtw_write32(Adapter, WCAMI, cam_val[0]);
				
				cmd = CAM_POLLINIG | CAM_WRITE | cam_val[1];
				rtw_write32(Adapter, RWCAM, cmd);
			}
			break;
		case HW_VAR_AC_PARAM_VO:
			rtw_write32(Adapter, REG_EDCA_VO_PARAM, ((u32 *)(val))[0]);
			break;
		case HW_VAR_AC_PARAM_VI:
			rtw_write32(Adapter, REG_EDCA_VI_PARAM, ((u32 *)(val))[0]);
			break;
		case HW_VAR_AC_PARAM_BE:
			pHalData->AcParam_BE = ((u32 *)(val))[0];
			rtw_write32(Adapter, REG_EDCA_BE_PARAM, ((u32 *)(val))[0]);
			break;
		case HW_VAR_AC_PARAM_BK:
			rtw_write32(Adapter, REG_EDCA_BK_PARAM, ((u32 *)(val))[0]);
			break;
		case HW_VAR_ACM_CTRL:
			{
				u8	acm_ctrl = *((u8 *)val);
				u8	AcmCtrl = rtw_read8( Adapter, REG_ACMHWCTRL);

				if(acm_ctrl > 1)
					AcmCtrl = AcmCtrl | 0x1;

				if(acm_ctrl & BIT(3))
					AcmCtrl |= AcmHw_VoqEn;
				else
					AcmCtrl &= (~AcmHw_VoqEn);

				if(acm_ctrl & BIT(2))
					AcmCtrl |= AcmHw_ViqEn;
				else
					AcmCtrl &= (~AcmHw_ViqEn);

				if(acm_ctrl & BIT(1))
					AcmCtrl |= AcmHw_BeqEn;
				else
					AcmCtrl &= (~AcmHw_BeqEn);

				DBG_871X("[HW_VAR_ACM_CTRL] Write 0x%X\n", AcmCtrl );
				rtw_write8(Adapter, REG_ACMHWCTRL, AcmCtrl );
			}
			break;
		case HW_VAR_AMPDU_MIN_SPACE:
			{
				u8	MinSpacingToSet;
				u8	SecMinSpace;

				MinSpacingToSet = *((u8 *)val);
				if(MinSpacingToSet <= 7)
				{
					switch(Adapter->securitypriv.dot11PrivacyAlgrthm)
					{
						case _NO_PRIVACY_:
						case _AES_:
							SecMinSpace = 0;
							break;

						case _WEP40_:
						case _WEP104_:
						case _TKIP_:
						case _TKIP_WTMIC_:
						default:
							SecMinSpace = 7;
							break;
					}

					if(MinSpacingToSet < SecMinSpace){
						MinSpacingToSet = SecMinSpace;
					}

					//RT_TRACE(COMP_MLME, DBG_LOUD, ("Set HW_VAR_AMPDU_MIN_SPACE: %#x\n", Adapter->MgntInfo.MinSpaceCfg));
					rtw_write8(Adapter, REG_AMPDU_MIN_SPACE, (rtw_read8(Adapter, REG_AMPDU_MIN_SPACE) & 0xf8) | MinSpacingToSet);
				}
			}
			break;
		case HW_VAR_AMPDU_FACTOR:
			{
				u8	RegToSet_Normal[4]={0x41,0xa8,0x72, 0xb9};
				u8	RegToSet_BT[4]={0x31,0x74,0x42, 0x97};
				u8	FactorToSet;
				u8	*pRegToSet;
				u8	index = 0;

#if 0//cosa, for 92s
				if(	(pHalData->bt_coexist.BT_Coexist) &&
					(pHalData->bt_coexist.BT_CoexistType == BT_CSR_BC4) )
					pRegToSet = RegToSet_BT; // 0x97427431;
				else
#endif
					pRegToSet = RegToSet_Normal; // 0xb972a841;

				FactorToSet = *((u8 *)val);
				if(FactorToSet <= 3)
				{
					FactorToSet = (1<<(FactorToSet + 2));
					if(FactorToSet>0xf)
						FactorToSet = 0xf;

					for(index=0; index<4; index++)
					{
						if((pRegToSet[index] & 0xf0) > (FactorToSet<<4))
							pRegToSet[index] = (pRegToSet[index] & 0x0f) | (FactorToSet<<4);
					
						if((pRegToSet[index] & 0x0f) > FactorToSet)
							pRegToSet[index] = (pRegToSet[index] & 0xf0) | (FactorToSet);
						
						rtw_write8(Adapter, (REG_AGGLEN_LMT+index), pRegToSet[index]);
					}

					//RT_TRACE(COMP_MLME, DBG_LOUD, ("Set HW_VAR_AMPDU_FACTOR: %#x\n", FactorToSet));
				}
			}
			break;
		case HW_VAR_SET_RPWM:
			rtw_write8(Adapter, REG_PCIE_HRPWM, *((u8 *)val));
			break;
		case HW_VAR_H2C_FW_PWRMODE:
			{
				u8	psmode = (*(u8 *)val);
			
				// Forece leave RF low power mode for 1T1R to prevent conficting setting in Fw power
				// saving sequence. 2010.06.07. Added by tynli. Suggested by SD3 yschang.
				if( (psmode != PS_MODE_ACTIVE) && (!IS_92C_SERIAL(pHalData->VersionID)))
				{
					rtl8192c_dm_RF_Saving(Adapter, _TRUE);
				}
				rtl8192c_set_FwPwrMode_cmd(Adapter, psmode);
			}
			break;
		case HW_VAR_H2C_FW_JOINBSSRPT:
			{
				u8	mstatus = (*(u8 *)val);
				rtl8192c_set_FwJoinBssReport_cmd(Adapter, mstatus);
			}
			break;
#ifdef CONFIG_P2P_PS
		case HW_VAR_H2C_FW_P2P_PS_OFFLOAD:
			{
				u8	p2p_ps_state = (*(u8 *)val);
				rtl8192c_set_p2p_ps_offload_cmd(Adapter, p2p_ps_state);
			}
			break;
#endif // CONFIG_P2P_PS
		case HW_VAR_INITIAL_GAIN:
			{				
				DIG_T	*pDigTable = &pdmpriv->DM_DigTable;					
				u32 		rx_gain = ((u32 *)(val))[0];
				
				if(rx_gain == 0xff){//restore rx gain
					pDigTable->CurIGValue = pDigTable->BackupIGValue;
					rtw_write8(Adapter,rOFDM0_XAAGCCore1, pDigTable->CurIGValue);
					rtw_write8(Adapter,rOFDM0_XBAGCCore1, pDigTable->CurIGValue);
				}
				else{
					pDigTable->BackupIGValue = pDigTable->CurIGValue;					
					PHY_SetBBReg(Adapter, rOFDM0_XAAGCCore1, 0x7f,rx_gain );
					PHY_SetBBReg(Adapter, rOFDM0_XBAGCCore1, 0x7f,rx_gain);
					pDigTable->CurIGValue = rx_gain;
				}
			}
			break;
#ifdef CONFIG_SW_ANTENNA_DIVERSITY
		case HW_VAR_ANTENNA_DIVERSITY_JOIN:
			{
				u8	Optimum_antenna = (*(u8 *)val);
				//switch antenna to Optimum_antenna
				DBG_8192C("HW_VAR_ANTENNA_DIVERSITY_JOIN cur_ant(%d),opt_ant(%d)\n", pHalData->CurAntenna, Optimum_antenna);
				if(pHalData->CurAntenna !=  Optimum_antenna)		
				{
					PHY_SetBBReg(Adapter, rFPGA0_XA_RFInterfaceOE, 0x300, Optimum_antenna);
					DBG_8192C("#### Change to Optimum_antenna(%s)\n",(2==Optimum_antenna)?"A":"B");
				}
			}
			break;
		case HW_VAR_ANTENNA_DIVERSITY_LINK:
			SwAntDivRestAfterLink8192C(Adapter);
			break;
		case HW_VAR_ANTENNA_DIVERSITY_SELECT:
			{
				u8	Optimum_antenna = (*(u8 *)val);

				//DBG_8192C("==> HW_VAR_ANTENNA_DIVERSITY_SELECT , Ant_(%s)\n",(Optimum_antenna==2)?"A":"B");

				PHY_SetBBReg(Adapter, rFPGA0_XA_RFInterfaceOE, 0x300, Optimum_antenna);
			}
			break;
#endif
		case HW_VAR_EFUSE_BYTES: // To set EFUE total used bytes, added by Roger, 2008.12.22.
			pHalData->EfuseUsedBytes = *((u16 *)val);			
			break;
		case HW_VAR_FIFO_CLEARN_UP:
			{
				#define RW_RELEASE_EN		BIT18
				#define RXDMA_IDLE			BIT17
				
				struct pwrctrl_priv *pwrpriv = &Adapter->pwrctrlpriv;

				u8 trycnt = 100;	
				
				//pause tx
				rtw_write8(Adapter,REG_TXPAUSE,0xff);
			
				//keep sn
				Adapter->xmitpriv.nqos_ssn = rtw_read16(Adapter,REG_NQOS_SEQ);

				if(pwrpriv->bkeepfwalive != _TRUE)
				{
					//RX DMA stop
					rtw_write32(Adapter,REG_RXPKT_NUM,(rtw_read32(Adapter,REG_RXPKT_NUM)|RW_RELEASE_EN));
					do{
						if(!(rtw_read32(Adapter,REG_RXPKT_NUM)&RXDMA_IDLE))
							break;
					}while(trycnt--);
					if(trycnt ==0)				
						DBG_8192C("Stop RX DMA failed...... \n");
						
					//RQPN Load 0
					rtw_write16(Adapter,REG_RQPN_NPQ,0x0);
					rtw_write32(Adapter,REG_RQPN,0x80000000);
					rtw_mdelay_os(10);
				}			
	
			}
			break;

		case HW_VAR_CHECK_TXBUF:
#ifdef CONFIG_CONCURRENT_MODE				
			{
				int i;
				u8	RetryLimit = 0x01;
				
				rtw_write16(Adapter, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
		
				for(i=0;i<1000;i++)
				{
					if(rtw_read32(Adapter, 0x200) != rtw_read32(Adapter, 0x204))
					{
						//DBG_871X("packet in tx packet buffer - 0x204=%x, 0x200=%x (%d)\n", rtw_read32(Adapter, 0x204), rtw_read32(Adapter, 0x200), i);
						rtw_msleep_os(10);
					}
					else
					{
						DBG_871X("no packet in tx packet buffer (%d)\n", i);
						break;
					}
				}

				RetryLimit = 0x30;	
				rtw_write16(Adapter, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
		
			}
#endif
			break;
		default:
			break;
	}

_func_exit_;
}

void GetHwReg8192CE(PADAPTER Adapter, u8 variable, u8* val)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

_func_enter_;

	switch(variable)
	{
		case HW_VAR_BASIC_RATE:
			*((u16 *)(val)) = pHalData->BasicRateSet;
		case HW_VAR_TXPAUSE:
			val[0] = rtw_read8(Adapter, REG_TXPAUSE);
			break;
		case HW_VAR_TX_BCN_DONE:
			{
				u32 xmitbcnDown;
				xmitbcnDown= rtw_read32(Adapter, REG_TDECTRL);
				if(xmitbcnDown & BCN_VALID  ){
					rtw_write32(Adapter,REG_TDECTRL, xmitbcnDown | BCN_VALID  ); // write 1 to clear, Clear by sw
					val[0] = _TRUE;
				}
			}
			break;
		case HW_VAR_DM_FLAG:
			val[0] = pHalData->dmpriv.DMFlag;
			break;
		case HW_VAR_RF_TYPE:
			val[0] = pHalData->rf_type;
			break;
		case HW_VAR_FWLPS_RF_ON:
			{
				//When we halt NIC, we should check if FW LPS is leave.
				u32	valRCR;
				
				if(Adapter->pwrctrlpriv.rf_pwrstate == rf_off)
				{
					// If it is in HW/SW Radio OFF or IPS state, we do not check Fw LPS Leave,
					// because Fw is unload.
					val[0] = _TRUE;
				}
				else
				{
					valRCR = rtw_read32(Adapter, REG_RCR);
					valRCR &= 0x00070000;
					if(valRCR)
						val[0] = _FALSE;
					else
						val[0] = _TRUE;
				}
			}
			break;
#ifdef CONFIG_ANTENNA_DIVERSITY
		case HW_VAR_CURRENT_ANTENNA:
			val[0] = pHalData->CurAntenna;
			break;
#endif
		case HW_VAR_EFUSE_BYTES: // To get EFUE total used bytes, added by Roger, 2008.12.22.
			*((u16 *)(val)) = pHalData->EfuseUsedBytes;	
			break;
		default:
			break;
	}

_func_exit_;
}

//
//	Description: 
//		Query setting of specified variable.
//
u8
GetHalDefVar8192CE(
	IN	PADAPTER				Adapter,
	IN	HAL_DEF_VARIABLE		eVariable,
	IN	PVOID					pValue
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8			bResult = _TRUE;

	switch(eVariable)
	{
		case HAL_DEF_UNDERCORATEDSMOOTHEDPWDB:
			*((int *)pValue) = pHalData->dmpriv.UndecoratedSmoothedPWDB;
			break;
		case HAL_DEF_RX_PACKET_OFFSET:
		case HAL_DEF_DRVINFO_SZ:
			*(( u32*)pValue) = DRVINFO_SZ;
			break;
		case HAL_DEF_MAX_RECVBUF_SZ:
			*(( u32*)pValue) = MAX_RECVBUF_SZ;
			break;
		default:
			//RT_TRACE(COMP_INIT, DBG_WARNING, ("GetHalDefVar8192CUsb(): Unkown variable: %d!\n", eVariable));
			bResult = _FALSE;
			break;
	}

	return bResult;
}


//
//	Description:
//		Change default setting of specified variable.
//
u8
SetHalDefVar8192CE(
	IN	PADAPTER				Adapter,
	IN	HAL_DEF_VARIABLE		eVariable,
	IN	PVOID					pValue
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u8			bResult = _TRUE;

	switch(eVariable)
	{
		default:
			//RT_TRACE(COMP_INIT, DBG_TRACE, ("SetHalDefVar819xUsb(): Unkown variable: %d!\n", eVariable));
			bResult = _FALSE;
			break;
	}

	return bResult;
}

void UpdateHalRAMask8192CE(PADAPTER padapter, u32 mac_id)
{
	//volatile unsigned int result;
	u8	init_rate=0;
	u8	networkType, raid;	
	u32	mask;
	u8	shortGIrate = _FALSE;
	int	supportRateNum = 0;
	struct sta_info	*psta;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;
	struct mlme_ext_priv	*pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
	WLAN_BSSID_EX 		*cur_network = &(pmlmeinfo->network);
#ifdef CONFIG_BT_COEXIST
	struct btcoexist_priv	*pbtpriv = &(pHalData->bt_coexist);
#endif

	if (mac_id >= NUM_STA) //CAM_SIZE
	{
		return;
	}

	psta = pmlmeinfo->FW_sta_info[mac_id].psta;
	if(psta == NULL)
	{
		return;
	}

	switch (mac_id)
	{
		case 0: // for infra mode
#ifdef CONFIG_CONCURRENT_MODE
		case 2:// first station uses macid=0, second station uses macid=2
#endif
			supportRateNum = rtw_get_rateset_len(cur_network->SupportedRates);
			networkType = judge_network_type(padapter, cur_network->SupportedRates, supportRateNum) & 0xf;
			//pmlmeext->cur_wireless_mode = networkType;
			raid = networktype_to_raid(networkType);

			mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);
			mask |= (pmlmeinfo->HT_enable)? update_MSC_rate(&(pmlmeinfo->HT_caps)): 0;
			mask |= ((raid<<28)&0xf0000000);

			if (support_short_GI(padapter, &(pmlmeinfo->HT_caps)))
			{
				shortGIrate = _TRUE;
			}

			break;

		case 1://for broadcast/multicast
			supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
			if(pmlmeext->cur_wireless_mode & WIRELESS_11B)
				networkType = WIRELESS_11B;
			else
				networkType = WIRELESS_11G;
			raid = networktype_to_raid(networkType);

			mask = update_basic_rate(cur_network->SupportedRates, supportRateNum);
			mask |= ((raid<<28)&0xf0000000);

			break;

		default: //for each sta in IBSS
#ifdef CONFIG_TDLS
			if(psta->tdls_sta_state & TDLS_LINKED_STATE)
			{
				shortGIrate = update_sgi_tdls(padapter, psta);
				mask = update_mask_tdls(padapter, psta);
				raid = mask>>28;
				break;
			}
			else
#endif //CONFIG_TDLS
			{
				supportRateNum = rtw_get_rateset_len(pmlmeinfo->FW_sta_info[mac_id].SupportedRates);
				networkType = judge_network_type(padapter, pmlmeinfo->FW_sta_info[mac_id].SupportedRates, supportRateNum) & 0xf;
				//pmlmeext->cur_wireless_mode = networkType;
				raid = networktype_to_raid(networkType);

				mask = update_supported_rate(cur_network->SupportedRates, supportRateNum);
				mask |= ((raid<<28)&0xf0000000);

				//todo: support HT in IBSS

				break;
			}
	}

#if 0
	//
	// Modify rate adaptive bitmap for BT coexist.
	//
	if( (pHalData->bt_coexist.BluetoothCoexist) &&
		(pHalData->bt_coexist.BT_CoexistType == BT_CSR) &&
		(pHalData->bt_coexist.BT_CUR_State) &&
		(pHalData->bt_coexist.BT_Ant_isolation) &&
		((pHalData->bt_coexist.BT_Service==BT_SCO)||
		(pHalData->bt_coexist.BT_Service==BT_Busy)) )
		mask &= 0x0fffcfc0;
	else
		mask &= 0x0FFFFFFF;
#endif

	init_rate = get_highest_rate_idx(mask)&0x3f;

	if(pHalData->fw_ractrl == _TRUE)
	{
		u8 arg = 0;

		arg = mac_id&0x1f;//MACID
		
		arg |= BIT(7);
		
		if (shortGIrate==_TRUE)
			arg |= BIT(5);

		DBG_871X("update raid entry, mask=0x%x, arg=0x%x\n", mask, arg);

		rtl8192c_set_raid_cmd(padapter, mask, arg);	
		
	}
	else
	{
		if (shortGIrate==_TRUE)
			init_rate |= BIT(6);

		rtw_write8(padapter, (REG_INIDATA_RATE_SEL+mac_id), init_rate);
	}


	//set ra_id
	psta->raid = raid;
	psta->init_rate = init_rate;

	//set correct initial date rate for each mac_id
	pdmpriv->INIDATA_RATE[mac_id] = init_rate;
}

void SetBeaconRelatedRegisters8192CE(PADAPTER padapter)
{
	u32	value32;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mlme_ext_priv	*pmlmeext = &(padapter->mlmeextpriv);
	struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);
		// Aries add, 20120314
	u32 bcn_ctrl_reg = REG_BCN_CTRL;
	
#ifdef CONFIG_CONCURRENT_MODE
        if (padapter->iface_type == IFACE_PORT1){
		bcn_ctrl_reg = REG_BCN_CTRL_1;
        }
#endif
	//
	// Beacon interval (in unit of TU).
	//
	rtw_write16(padapter, REG_BCN_INTERVAL, pmlmeinfo->bcn_interval);
	//
	// ATIM window
	//
	rtw_write16(padapter, REG_ATIMWND, 0x2);
	_InitBeaconParameters(padapter);	

	
	//2008.10.24 added by tynli for beacon changed.
	//PHY_SetBeaconHwReg( Adapter, BcnInterval );

	//
	// DrvErlyInt (in unit of TU). (Time to send interrupt to notify driver to change beacon content)
	//
	//PlatformEFIOWrite1Byte(Adapter, BCN_DMA_INT_92C+1, 0xC);

	//
	// BcnDMATIM(in unit of us). Indicates the time before TBTT to perform beacon queue DMA 
	//
	//PlatformEFIOWrite2Byte(Adapter, BCN_DMATIM_92C, 256); // HWSD suggest this value 2006.11.14

	//
	// Force beacon frame transmission even after receiving beacon frame from other ad hoc STA
	//
	//PlatformEFIOWrite2Byte(Adapter, BCN_ERRTH_92C, 100); // Reference from WMAC code 2006.11.14
	//suggest by wl, 20090902
	if(IS_81XXC_TEST_CHIP(GET_HAL_DATA(padapter)->VersionID))
	{
		// Suggested by designer timchen. Change beacon AIFS to the largest number
		// beacause test chip does not contension before sending beacon. by tynli. 2009.11.03
		rtw_write16(padapter, REG_BCNTCFG, 0x66ff);
	}
	else
	{
		rtw_write16(padapter, REG_BCNTCFG, 0x660f);
	}

	//For throughput
	//PlatformEFIOWrite2Byte(Adapter,TBTT_PROHIBIT_92C,0x0202);
	//suggest by wl, 20090902
	//PlatformEFIOWrite1Byte(Adapter,REG_RXTSF_OFFSET_CCK, 0x30);	
	//PlatformEFIOWrite1Byte(Adapter,REG_RXTSF_OFFSET_OFDM, 0x30);
	
	// Suggested by TimChen. 2009.01.25.
	// Rx RF to MAC data path time.
	rtw_write8(padapter,REG_RXTSF_OFFSET_CCK, 0x18);	
	rtw_write8(padapter,REG_RXTSF_OFFSET_OFDM, 0x18);

	rtw_write8(padapter,0x606, 0x30);

	//
	// Update interrupt mask for IBSS.
	//
#ifdef CONFIG_CONCURRENT_MODE
	UpdateInterruptMask8192CE( padapter, RT_IBSS_INT_MASKS, IMR_BcnInt_E, 0, 0);
#else
	UpdateInterruptMask8192CE( padapter, RT_IBSS_INT_MASKS, 0, 0, 0);
#endif

	ResumeTxBeacon(padapter);

	//rtw_write8(padapter, 0x422, rtw_read8(padapter, 0x422)|BIT(6));
	
	//rtw_write8(padapter, 0x541, 0xff);

	//rtw_write8(padapter, 0x542, rtw_read8(padapter, 0x541)|BIT(0));

	rtw_write8(padapter, bcn_ctrl_reg, rtw_read8(padapter, bcn_ctrl_reg)|BIT(1));

}

static void rtl8192ce_init_default_value(_adapter * padapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct dm_priv	*pdmpriv = &pHalData->dmpriv;

	//init default value
	pHalData->fw_ractrl = _FALSE;
	pHalData->LastHMEBoxNum = 0;
	pHalData->bIQKInitialized = _FALSE;
	pHalData->bDefaultAntenna = 1;

	//
	// Set TCR-Transmit Control Register. The value is set in InitializeAdapter8190Pci()
	//
	pHalData->TransmitConfig = CFENDFORM | BIT12 | BIT13;

	//
	// Set RCR-Receive Control Register . The value is set in InitializeAdapter8190Pci().
	//
	pHalData->ReceiveConfig = (\
		//RCR_APPFCS	
		// | RCR_APWRMGT
		// |RCR_ADD3
		// | RCR_ADF |
		RCR_AMF | RCR_APP_MIC| RCR_APP_ICV
		| RCR_AICV | RCR_ACRC32	// Accept ICV error, CRC32 Error
		| RCR_AB | RCR_AM			// Accept Broadcast, Multicast	 
     		| RCR_APM 					// Accept Physical match
     		//| RCR_AAP					// Accept Destination Address packets
     		| RCR_APP_PHYST_RXFF		// Accept PHY status
     		| RCR_HTC_LOC_CTRL
		//(pHalData->EarlyRxThreshold<<RCR_FIFO_OFFSET)	);
		);

	//
	// Set Interrupt Mask Register
	//
	// Make reference from WMAC code 2006.10.02, maybe we should disable some of the interrupt. by Emily
	pHalData->IntrMask[0]	= (u32)(			\
								IMR_ROK			|
								IMR_VODOK		|
								IMR_VIDOK 		|
								IMR_BEDOK 		|
								IMR_BKDOK		|
//								IMR_TBDER		|
								IMR_MGNTDOK 	|
//								IMR_TBDOK		|
								IMR_HIGHDOK 	|
								IMR_BDOK		|
//								IMR_ATIMEND		|
								IMR_RDU			|
								IMR_RXFOVW		|
//								IMR_BcnInt		|
								IMR_PSTIMEOUT	| // P2P PS Timeout
//								IMR_TXFOVW		|
//								IMR_TIMEOUT1		|
//								IMR_TIMEOUT2		|
//								IMR_BCNDOK1		|
//								IMR_BCNDOK2		|
//								IMR_BCNDOK3		|
//								IMR_BCNDOK4		|
//								IMR_BCNDOK5		|
//								IMR_BCNDOK6		|
//								IMR_BCNDOK7		|
//								IMR_BCNDOK8		|
//								IMR_BCNDMAINT1	|
//								IMR_BCNDMAINT2	|
//								IMR_BCNDMAINT3	|
//								IMR_BCNDMAINT4	|
//								IMR_BCNDMAINT5	|
//								IMR_BCNDMAINT6	|
								0);
	pHalData->IntrMask[1] 	= (u32)(\
//								IMR_WLANOFF		|
//								IMR_OCPINT		|
//								IMR_CPWM		|
								IMR_C2HCMD		|
//								IMR_RXERR		|
//								IMR_TXERR		|
								0);

	pHalData->IntrMaskToSet[0] = pHalData->IntrMask[0];
	pHalData->IntrMaskToSet[1] = pHalData->IntrMask[1];

	//init dm default value
	pdmpriv->TM_Trigger = 0;
	pdmpriv->binitialized = _FALSE;
	pdmpriv->prv_traffic_idx = 3;
	pdmpriv->initialize = 0;
}

void rtl8192ce_set_hal_ops(_adapter * padapter)
{
	struct hal_ops	*pHalFunc = &padapter->HalFunc;

_func_enter_;

	padapter->HalData = rtw_malloc(sizeof(HAL_DATA_TYPE));
	if(padapter->HalData == NULL){
		DBG_8192C("cant not alloc memory for HAL DATA \n");
	}
	_rtw_memset(padapter->HalData, 0, sizeof(HAL_DATA_TYPE));

	padapter->hal_data_sz = sizeof(HAL_DATA_TYPE);
	pHalFunc->hal_init = &rtl8192ce_hal_init;
	pHalFunc->hal_deinit = &rtl8192ce_hal_deinit;

	//pHalFunc->free_hal_data = &rtl8192c_free_hal_data;

	pHalFunc->inirp_init = &rtl8192ce_init_desc_ring;
	pHalFunc->inirp_deinit = &rtl8192ce_free_desc_ring;

	pHalFunc->init_xmit_priv = &rtl8192ce_init_xmit_priv;
	pHalFunc->free_xmit_priv = &rtl8192ce_free_xmit_priv;

	pHalFunc->init_recv_priv = &rtl8192ce_init_recv_priv;
	pHalFunc->free_recv_priv = &rtl8192ce_free_recv_priv;

#ifdef CONFIG_SW_LED
	pHalFunc->InitSwLeds = &rtl8192ce_InitSwLeds;
	pHalFunc->DeInitSwLeds = &rtl8192ce_DeInitSwLeds;
#else //case of hw led or no led
	pHalFunc->InitSwLeds = NULL;
	pHalFunc->DeInitSwLeds = NULL;	
#endif //CONFIG_SW_LED

	//pHalFunc->dm_init = &rtl8192c_init_dm_priv;
	//pHalFunc->dm_deinit = &rtl8192c_deinit_dm_priv;

	pHalFunc->init_default_value = &rtl8192ce_init_default_value;
	pHalFunc->intf_chip_configure = &rtl8192ce_interface_configure;
	pHalFunc->read_adapter_info = &ReadAdapterInfo8192CE;

	pHalFunc->enable_interrupt = &EnableInterrupt8192CE;
	pHalFunc->disable_interrupt = &DisableInterrupt8192CE;
	pHalFunc->interrupt_handler = &rtl8192ce_interrupt;

	//pHalFunc->set_bwmode_handler = &PHY_SetBWMode8192C;
	//pHalFunc->set_channel_handler = &PHY_SwChnl8192C;

	//pHalFunc->hal_dm_watchdog = &rtl8192c_HalDmWatchDog;

	pHalFunc->SetHwRegHandler = &SetHwReg8192CE;
	pHalFunc->GetHwRegHandler = &GetHwReg8192CE;
  	pHalFunc->GetHalDefVarHandler = &GetHalDefVar8192CE;
 	pHalFunc->SetHalDefVarHandler = &SetHalDefVar8192CE;

	pHalFunc->UpdateRAMaskHandler = &UpdateHalRAMask8192CE;
	pHalFunc->SetBeaconRelatedRegistersHandler = &SetBeaconRelatedRegisters8192CE;

	//pHalFunc->Add_RateATid = &rtl8192c_Add_RateATid;

//#ifdef CONFIG_SW_ANTENNA_DIVERSITY
	//pHalFunc->SwAntDivBeforeLinkHandler = &SwAntDivBeforeLink8192C;
	//pHalFunc->SwAntDivCompareHandler = &SwAntDivCompare8192C;
//#endif

	pHalFunc->hal_xmit = &rtl8192ce_hal_xmit;
	pHalFunc->mgnt_xmit = &rtl8192ce_mgnt_xmit;

	//pHalFunc->read_bbreg = &rtl8192c_PHY_QueryBBReg;
	//pHalFunc->write_bbreg = &rtl8192c_PHY_SetBBReg;
	//pHalFunc->read_rfreg = &rtl8192c_PHY_QueryRFReg;
	//pHalFunc->write_rfreg = &rtl8192c_PHY_SetRFReg;

#ifdef CONFIG_HOSTAPD_MLME
	pHalFunc->hostap_mgnt_xmit_entry = &rtl8192ce_hostap_mgnt_xmit_entry;
#endif

	rtl8192c_set_hal_ops(pHalFunc);
	
_func_exit_;

}

