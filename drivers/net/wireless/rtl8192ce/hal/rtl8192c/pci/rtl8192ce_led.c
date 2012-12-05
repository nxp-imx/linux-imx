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

#include "drv_types.h"
#include "rtl8192c_hal.h"

//================================================================================
//	Constant.
//================================================================================

//
// Default LED behavior.
//
#define LED_BLINK_NORMAL_INTERVAL	100
#define LED_BLINK_SLOWLY_INTERVAL	200

#define LED_BLINK_NORMAL_INTERVAL_NETTRONIX  100
#define LED_BLINK_SLOWLY_INTERVAL_NETTRONIX  2000

#define LED_BLINK_SLOWLY_INTERVAL_PORNET 1000
#define LED_BLINK_NORMAL_INTERVAL_PORNET 100
#define LED_BLINK_FAST_INTERVAL_BITLAND 30

//
// 060403, rcnjko: Customized for AzWave.
//
#define LED_CM2_BLINK_ON_INTERVAL			250
#define LED_CM2_BLINK_OFF_INTERVAL		4750

#define LED_CM8_BLINK_OFF_INTERVAL		3750	//for QMI

// 080124, lanhsin: Customized for RunTop
#define LED_RunTop_BLINK_INTERVAL			300

//
// 060421, rcnjko: Customized for Sercomm Printer Server case.
//
#define LED_CM3_BLINK_INTERVAL				1500

//================================================================================
//	Prototype of protected function.
//================================================================================


static void
BlinkTimerCallback(
	unsigned long data
	);

//================================================================================
// LED_819xUsb routines. 
//================================================================================

//
//	Description:
//		Initialize an LED_871x object.
//
static void
InitLed871x(
	_adapter			*padapter,
	PLED_871x		pLed,
	LED_PIN_871x	LedPin
	)
{
	pLed->padapter = padapter;

	pLed->LedPin = LedPin;

	pLed->CurrLedState = RTW_LED_OFF;
	pLed->bLedOn = _FALSE;

	pLed->bLedBlinkInProgress = _FALSE;
	pLed->BlinkTimes = 0;
	pLed->BlinkingLedState = LED_UNKNOWN;

	_init_timer(&(pLed->BlinkTimer), padapter->pnetdev, BlinkTimerCallback, pLed);

}


//
//	Description:
//		DeInitialize an LED_871x object.
//
static void
DeInitLed871x(
	PLED_871x			pLed
	)
{
	_cancel_timer_ex(&(pLed->BlinkTimer));

	// We should reset bLedBlinkInProgress if we cancel the LedControlTimer, 2005.03.10, by rcnjko.
	pLed->bLedBlinkInProgress = _FALSE;
}


//
//	Description:
//		Turn on LED according to LedPin specified.
//
static void
SwLedOn(
	_adapter			*padapter, 
	PLED_871x		pLed
)
{
	u8	LedCfg;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct led_priv	*ledpriv = &(padapter->ledpriv);

	if( (padapter->bSurpriseRemoved == _TRUE) || ( padapter->bDriverStopped == _TRUE))
	{
		return;
	}
	
	// 2009/10/26 MH Issau if tyhe device is 8c DID is 0x8176, we need to enable bit6 to
	// enable GPIO8 for controlling LED.	

	switch(pLed->LedPin)
	{	
		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
			if(ledpriv->LedStrategy == SW_LED_MODE10)
			{
				//DBG_8192C("In SwLedOn SW_LED_MODE10, LedAddr:%X LEDPIN=%d \n",REG_LEDCFG0, pLed->LedPin);

				LedCfg = rtw_read8(padapter, REG_LEDCFG0);			
				rtw_write8(padapter, REG_LEDCFG0, LedCfg&0x10); // SW control led0 on.			
			}
			else
			{
				//DBG_8192C("In SwLedOn,LedAddr:%X LEDPIN=%d\n",REG_LEDCFG2, pLed->LedPin);

				LedCfg = rtw_read8(padapter, REG_LEDCFG2);			
				rtw_write8(padapter, REG_LEDCFG2, (LedCfg&0xf0)|BIT5|BIT6); // SW control led0 on.
			}
			break;

		case LED_PIN_LED1:
			//DBG_8192C("In SwLedOn,LedAddr:%X LEDPIN=%d\n",REG_LEDCFG1, pLed->LedPin);

			LedCfg = rtw_read8(padapter, REG_LEDCFG1);			
			rtw_write8(padapter, REG_LEDCFG1, LedCfg&0x10); // SW control led0 on.
			break;

		default:
			break;
	}

	pLed->bLedOn = _TRUE;
}


//
//	Description:
//		Turn off LED according to LedPin specified.
//
static void
SwLedOff(
	_adapter			*padapter, 
	PLED_871x		pLed
)
{
	u8	LedCfg;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct led_priv	*ledpriv = &(padapter->ledpriv);

	if((padapter->bSurpriseRemoved == _TRUE) || ( padapter->bDriverStopped == _TRUE))	
	{
             return;
	}

	//
	// 2009/10/23 MH Issau eed to move the LED GPIO from bit  0 to bit3.
	// 2009/10/26 MH Issau if tyhe device is 8c DID is 0x8176, we need to enable bit6 to
	// enable GPIO8 for controlling LED.	
	// 2010/06/16 Supprt Open-drain arrangement for controlling the LED. Added by Roger.
	//
	switch(pLed->LedPin)
	{

		case LED_PIN_GPIO0:
			break;

		case LED_PIN_LED0:
			if(ledpriv->LedStrategy == SW_LED_MODE10)
			{
				//DBG_8192C("In SwLedOff,LedAddr:%X LEDPIN=%d\n",REG_LEDCFG0, pLed->LedPin);
				LedCfg = rtw_read8(padapter, REG_LEDCFG0);
				
				LedCfg &= 0x10; // Set to software control.							
				rtw_write8(padapter, REG_LEDCFG0, LedCfg|BIT3);				
			}
			else
			{
				//DBG_8192C("In SwLedOff,LedAddr:%X LEDPIN=%d\n",REG_LEDCFG2, pLed->LedPin);
				LedCfg = rtw_read8(padapter, REG_LEDCFG2);
				
				LedCfg &= 0xf0; // Set to software control.				
				if(pHalData->bLedOpenDrain == _TRUE) // Open-drain arrangement for controlling the LED
				{
					LedCfg &= 0x90; // Set to software control.				
					rtw_write8(padapter, REG_LEDCFG2, (LedCfg|BIT3));				
					LedCfg = rtw_read8(padapter, REG_MAC_PINMUX_CFG);
					LedCfg &= 0xFE;
					rtw_write8(padapter, REG_MAC_PINMUX_CFG, LedCfg);	

				}
				else
				{
					rtw_write8(padapter, REG_LEDCFG2, (LedCfg|BIT3|BIT5|BIT6));
				}
			}
			break;

		case LED_PIN_LED1:
			//DBG_8192C("In SwLedOff,LedAddr:%X LEDPIN=%d\n",REG_LEDCFG1, pLed->LedPin);
			LedCfg = rtw_read8(padapter, REG_LEDCFG1);
			
			LedCfg &= 0x10; // Set to software control. 							
			rtw_write8(padapter, REG_LEDCFG1, LedCfg|BIT3);
			break;

		default:
			break;
	}

	pLed->bLedOn = _FALSE;
	
}

//
//	Description:
//		Turn on LED according to LedPin specified.
//
static VOID
HwLedBlink(
	IN	PADAPTER			Adapter, 
	IN	PLED_871x			pLed
)
{

	
	switch(pLed->LedPin)
	{
	case LED_PIN_GPIO0:
		break;

	case LED_PIN_LED0:
		//PlatformEFIOWrite1Byte(Adapter, LED0Cfg, 0x2);
		break;

	case LED_PIN_LED1:
		//PlatformEFIOWrite1Byte(Adapter, LED1Cfg, 0x2);
		break;

	default:
		break;
	}

	pLed->bLedOn = _TRUE;
}


//================================================================================
// Interface to manipulate LED objects.
//================================================================================


//
//	Description:
//		Implementation of LED blinking behavior.
//		It toggle off LED and schedule corresponding timer if necessary.
//
static void
SwLedBlink(
	PLED_871x			pLed
	)
{
	_adapter			*padapter = pLed->padapter;
	struct mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	u8				bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.
	if( pLed->BlinkingLedState == RTW_LED_ON ) 
	{
		SwLedOn(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn on\n", pLed->BlinkTimes));
	}
	else 
	{
		SwLedOff(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,( "Blinktimes (%d): turn off\n", pLed->BlinkTimes));
	}

	// Determine if we shall change LED state again.
	pLed->BlinkTimes--;
	switch(pLed->CurrLedState)
	{
		case LED_BLINK_NORMAL: 
		case LED_BLINK_TXRX:
		case LED_BLINK_RUNTOP:
			if(pLed->BlinkTimes == 0)
			{
				bStopBlinking = _TRUE;
			}
			break;
			
		case LED_SCAN_BLINK:
			if( ( ( check_fwstate(pmlmepriv, WIFI_STATION_STATE) && check_fwstate(pmlmepriv, _FW_LINKED) ) ||
				( check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE) ) ) && // Linked.
				!check_fwstate(pmlmepriv, _FW_UNDER_SURVEY) && // Not in scan stage.
				(pLed->BlinkTimes % 2 == 0)  // Even
			) 
			{
				bStopBlinking = _TRUE;
			}
			break;

		case LED_NO_LINK_BLINK:
		case LED_BLINK_StartToBlink:
			if( check_fwstate(pmlmepriv, _FW_LINKED) && check_fwstate(pmlmepriv, WIFI_STATION_STATE) )// Linked.
			{
				bStopBlinking = _TRUE;
			}
			if( (check_fwstate(pmlmepriv, _FW_LINKED)== _TRUE)&&
				(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) )
			{
				bStopBlinking = _TRUE;
			}
			else if(pLed->BlinkTimes == 0)
			{
				bStopBlinking = _TRUE;
			}
			break;

		case LED_BLINK_CAMEO:
			if( check_fwstate(pmlmepriv, _FW_LINKED) && check_fwstate(pmlmepriv, WIFI_STATION_STATE) )// Linked.
			{
				bStopBlinking = _TRUE;
			}
			if( check_fwstate(pmlmepriv, _FW_LINKED) &&
				(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) )
			{
				bStopBlinking = _TRUE;
			}
			break;

		default:
			bStopBlinking = _TRUE;
			break;
				
	}
		
	if(bStopBlinking)
	{
		if( padapter->pwrctrlpriv.rf_pwrstate != rf_on)
		{
			SwLedOff(padapter, pLed);
		}
		else if(pLed->CurrLedState == LED_BLINK_TXRX)
		{
			SwLedOff(padapter, pLed);
		}
		else if(pLed->CurrLedState == LED_BLINK_RUNTOP)
		{
			SwLedOff(padapter, pLed);
		}
		else if( (check_fwstate(pmlmepriv, _FW_LINKED)== _TRUE) && (pLed->bLedOn == _FALSE))
		{
			SwLedOn(padapter, pLed);
		}
		else if( (check_fwstate(pmlmepriv, _FW_LINKED)== _FALSE) &&  pLed->bLedOn == _TRUE)
		{
			SwLedOff(padapter, pLed);
		}

		pLed->BlinkTimes = 0;
		pLed->bLedBlinkInProgress = _FALSE;
	}
	else
	{
		// Assign LED state to toggle.
		if( pLed->BlinkingLedState == RTW_LED_ON ) 
			pLed->BlinkingLedState = RTW_LED_OFF;
		else 
			pLed->BlinkingLedState = RTW_LED_ON;

		// Schedule a timer to toggle LED state. 
		switch( pLed->CurrLedState )
		{
			case LED_BLINK_NORMAL:
			case LED_BLINK_TXRX:
			case LED_BLINK_StartToBlink:
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				break;
			
			case LED_BLINK_SLOWLY:
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL);
				break;

			case LED_SCAN_BLINK:
			case LED_NO_LINK_BLINK:
				if( pLed->bLedOn )
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				else
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_OFF_INTERVAL);
				break;

			case LED_BLINK_RUNTOP:
				_set_timer(&(pLed->BlinkTimer), LED_RunTop_BLINK_INTERVAL);
				break;

			case LED_BLINK_CAMEO:
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
				break;

			default:
				DBG_8192C("SwLedCm2Blink(): unexpected state!\n");
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL);
				break;
		}
	}
}


static void
SwLedBlink5(
	PLED_871x			pLed
	)
{
	_adapter				*padapter = pLed->padapter;
	struct mlme_priv		*pmlmepriv = &(padapter->mlmepriv);
	u8					bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.
	if( pLed->BlinkingLedState == RTW_LED_ON ) 
	{
		SwLedOn(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,( "Blinktimes (%d): turn on\n", pLed->BlinkTimes));
	}
	else 
	{
		SwLedOff(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn off\n", pLed->BlinkTimes));
	}

	switch(pLed->CurrLedState)
	{
		case RTW_LED_OFF:
			SwLedOff(padapter, pLed);
			break;
	
		case LED_BLINK_SLOWLY:			
			if( pLed->bLedOn )
				pLed->BlinkingLedState = RTW_LED_OFF; 
			else
				pLed->BlinkingLedState = RTW_LED_ON;
			_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_NETTRONIX);
			break;

		case LED_BLINK_NORMAL:
			pLed->BlinkTimes--;
			if( pLed->BlinkTimes == 0 )
			{
				bStopBlinking = _TRUE;
			}
			if(bStopBlinking)
			{
				if( padapter->pwrctrlpriv.rf_pwrstate != rf_on )
				{
					SwLedOff(padapter, pLed);
				}
				else 
				{
					pLed->bLedSlowBlinkInProgress = _TRUE;
					pLed->CurrLedState = LED_BLINK_SLOWLY;
					if( pLed->bLedOn )
						pLed->BlinkingLedState = RTW_LED_OFF; 
					else
						pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_NETTRONIX);
				}
				pLed->BlinkTimes = 0;
				pLed->bLedBlinkInProgress = _FALSE;	
			}
			else
			{
				if( padapter->pwrctrlpriv.rf_pwrstate != rf_on )
				{
					SwLedOff(padapter, pLed);
				}
				else
				{
					if( pLed->bLedOn )
						pLed->BlinkingLedState = RTW_LED_OFF; 
					else
						pLed->BlinkingLedState = RTW_LED_ON; 

					_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL_NETTRONIX);
				}
			}
			break;

		default:
			break;
	}

}

static void
SwLedBlink6(
	PLED_871x			pLed
	)
{
	_adapter				*padapter = pLed->padapter;
	struct mlme_priv		*pmlmepriv = &(padapter->mlmepriv);
	u8					bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.
	if( pLed->BlinkingLedState == RTW_LED_ON) 
	{
		SwLedOn(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn on\n", pLed->BlinkTimes));
	}
	else 
	{
		SwLedOff(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn off\n", pLed->BlinkTimes));
	}

	switch(pLed->CurrLedState)
	{
		case RTW_LED_OFF:
			SwLedOff(padapter, pLed);
			break;

		case LED_BLINK_SLOWLY:
			if( pLed->bLedOn )
				pLed->BlinkingLedState = RTW_LED_OFF; 
			else
				pLed->BlinkingLedState = RTW_LED_ON; 
			_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
			break;

		case LED_BLINK_NORMAL:
			pLed->BlinkTimes--;
			if( pLed->BlinkTimes == 0 )
			{
				bStopBlinking = _TRUE;
			}
			if(bStopBlinking)
			{
				if( padapter->pwrctrlpriv.rf_pwrstate != rf_on )
				{
					SwLedOff(padapter, pLed);
				}
				else 
				{
					pLed->bLedSlowBlinkInProgress = _TRUE;
					pLed->CurrLedState = LED_BLINK_SLOWLY;
					if( pLed->bLedOn )
						pLed->BlinkingLedState = RTW_LED_OFF; 
					else
						pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
				}
				pLed->BlinkTimes = 0;
				pLed->bLedBlinkInProgress = _FALSE;	
			}
			else
			{
				if( padapter->pwrctrlpriv.rf_pwrstate != rf_on )
				{
					SwLedOff(padapter, pLed);
				}
				else
				{
					if( pLed->bLedOn )
						pLed->BlinkingLedState = RTW_LED_OFF; 
					else
						pLed->BlinkingLedState = RTW_LED_ON; 

					_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL_PORNET);
				}
			}
			break;

		default:
			break;
	}

}

static void
SwLedBlink7(
	PLED_871x			pLed
	)
{
	_adapter	*padapter = pLed->padapter;

	SwLedOn(padapter, pLed);
	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn on\n", pLed->BlinkTimes));
}


static void
SwLedBlink8(
	PLED_871x			pLed
	)
{
	_adapter			*padapter = pLed->padapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	u8				bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.

	if(RT_IS_FUNC_DISABLED(pHalData, DF_IO_BIT))
	{
		_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
		return;
	}
	
	if( pLed->BlinkingLedState == RTW_LED_ON ) 
	{
		SwLedOn(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn on\n", pLed->BlinkTimes));
	}
	else 
	{
		SwLedOff(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn off\n", pLed->BlinkTimes));
	}	

	// Determine if we shall change LED state again.
	if(pLed->CurrLedState != LED_NO_LINK_BLINK)
		pLed->BlinkTimes--;

	switch(pLed->CurrLedState)
	{
		case LED_BLINK_NORMAL: 
		case LED_SCAN_BLINK:			
			if(pLed->BlinkTimes == 0)
			{
				bStopBlinking = _TRUE;
			}
			break;

		default:
			break;
	}

	if(bStopBlinking)
	{
		if( padapter->pwrctrlpriv.rf_pwrstate != rf_on && padapter->pwrctrlpriv.rfoff_reason > RF_CHANGE_BY_PS)
		{
			pLed->CurrLedState = RTW_LED_OFF;				
			SwLedOff(padapter, pLed);
		}
		else 
		{
			pLed->CurrLedState = RTW_LED_ON;					
			SwLedOn(padapter, pLed);
		}
		
		pLed->BlinkTimes = 0;
		pLed->bLedBlinkInProgress = _FALSE;	
	}
	else
	{
		// Assign LED state to toggle.
		if( pLed->BlinkingLedState == RTW_LED_ON ) 
			pLed->BlinkingLedState = RTW_LED_OFF;
		else 
			pLed->BlinkingLedState = RTW_LED_ON;

		// Schedule a timer to toggle LED state. 
		switch( pLed->CurrLedState )
		{
			case LED_BLINK_NORMAL:
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				break;

			default:
				DBG_8192C("SwLedCm8Blink(): unexpected state!\n");
				break;
		}		
	}
}

static void
SwLedBlink9(
	PLED_871x			pLed
	)
{
	_adapter			*padapter = pLed->padapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	struct mlme_priv	*pmlmepriv = &(padapter->mlmepriv);
	u8				bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.

	if(RT_IS_FUNC_DISABLED(pHalData, DF_IO_BIT))
	{
		_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
		return;
	}

	if( pLed->BlinkingLedState == RTW_LED_ON ) 
	{
		SwLedOn(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn on\n", pLed->BlinkTimes));
	}
	else 
	{
		SwLedOff(padapter, pLed);
		RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Blinktimes (%d): turn off\n", pLed->BlinkTimes));
	}

	switch(pLed->CurrLedState)
	{
		case LED_BLINK_NORMAL: 
		case LED_SCAN_BLINK:			
			if(pLed->BlinkTimes == 0)
			{
				bStopBlinking = _TRUE;
			}
			break;

		case LED_NO_LINK_BLINK:
			if( check_fwstate(pmlmepriv, _FW_LINKED) && check_fwstate(pmlmepriv, WIFI_STATION_STATE) )// Linked.
			{
				bStopBlinking = _TRUE;
			}
			if( check_fwstate(pmlmepriv, _FW_LINKED) &&
				(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) )
			{
				bStopBlinking = _TRUE;
			}
			break;

		default:
			break;
	}

	if(bStopBlinking)
	{
		if( padapter->pwrctrlpriv.rf_pwrstate != rf_on && padapter->pwrctrlpriv.rfoff_reason > RF_CHANGE_BY_PS)
		{
			pLed->CurrLedState = RTW_LED_OFF;				
			SwLedOff(padapter, pLed);
		}
		else if( check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
		{
			pLed->CurrLedState = RTW_LED_ON;					
			SwLedOn(padapter, pLed);
		}
		else if( check_fwstate(pmlmepriv, _FW_LINKED) == _FALSE)
		{
			pLed->CurrLedState = LED_NO_LINK_BLINK; 	
			if( pLed->bLedOn )
				_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
			else
				_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
		}

		pLed->BlinkTimes = 0;
		if(pLed->CurrLedState != LED_NO_LINK_BLINK)
			pLed->bLedBlinkInProgress = _FALSE;	
	}
	else
	{
		// Assign LED state to toggle.
		if( pLed->BlinkingLedState == RTW_LED_ON ) 
			pLed->BlinkingLedState = RTW_LED_OFF;
		else 
			pLed->BlinkingLedState = RTW_LED_ON;

		// Schedule a timer to toggle LED state. 
		switch( pLed->CurrLedState )
		{
			case LED_BLINK_NORMAL:
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_FAST_INTERVAL_BITLAND);
				break;

			case LED_SCAN_BLINK:
			case LED_NO_LINK_BLINK:
				if( pLed->bLedOn )
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				else
					_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
				break;

			default:
				DBG_8192C("SwLedCm2Blink(): unexpected state!\n");
				break;
		}		
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("SwLedBlink9 CurrLedAction %d, \n", pLed->CurrLedState));
}

VOID
SwLedBlink10(
	PLED_871x			pLed
	)
{
	_adapter			*Adapter = pLed->padapter;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct mlme_priv	*pmlmepriv = &(Adapter->mlmepriv);
	u8				bStopBlinking = _FALSE;

	// Change LED according to BlinkingLedState specified.

	if(RT_IS_FUNC_DISABLED(pHalData, DF_IO_BIT))
	{
		_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
		return;
	}
	
	if( pLed->BlinkingLedState == RTW_LED_ON ) 
	{
		SwLedOn(Adapter, pLed);
		//DBG_8192C("Blinktimes (%ld): turn on\n", pLed->BlinkTimes);
	}	
	else 
	{
		SwLedOff(Adapter, pLed);
		//DBG_8192C("Blinktimes (%ld): turn off\n", pLed->BlinkTimes);
	}

	// Determine if we shall change LED state again.
	if(pLed->CurrLedState != LED_NO_LINK_BLINK)
		pLed->BlinkTimes--;
	
	switch(pLed->CurrLedState)
	{
		case LED_BLINK_NORMAL: 	
		case LED_SCAN_BLINK:			
			if(pLed->BlinkTimes == 0)
			{
				bStopBlinking = _TRUE;
			}
			break;
		default:
			break;
	}

	if(bStopBlinking)
	{
		if( (Adapter->pwrctrlpriv.rf_pwrstate != rf_on && Adapter->pwrctrlpriv.rfoff_reason > RF_CHANGE_BY_PS) ||
			( check_fwstate(pmlmepriv, _FW_LINKED) == _FALSE))
		{
			pLed->CurrLedState = RTW_LED_OFF;				
			SwLedOff(Adapter, pLed);
		}
		else if( check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
		{
			pLed->CurrLedState = RTW_LED_ON;					
			SwLedOn(Adapter, pLed);
		}

		pLed->BlinkTimes = 0;
		pLed->bLedBlinkInProgress = _FALSE;	
	}
	else
	{
		// Assign LED state to toggle.
		if( pLed->BlinkingLedState == RTW_LED_ON ) 
			pLed->BlinkingLedState = RTW_LED_OFF;
		else 
			pLed->BlinkingLedState = RTW_LED_ON;

		// Schedule a timer to toggle LED state. 
		switch( pLed->CurrLedState )
		{		
			case LED_BLINK_NORMAL:
			case LED_SCAN_BLINK:				
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_FAST_INTERVAL_BITLAND);
				break;

			default:
				//DBG_8192C("SwLedCm2Blink(): unexpected state!\n");
				break;
		}		
	}

	//DBG_8192C("SwLedBlink10 CurrLedAction %d, \n", pLed->CurrLedState);

}


//
//	Description:
//		Callback function of LED BlinkTimer, 
//		it just schedules to corresponding BlinkWorkItem.
//
static void
BlinkTimerCallback(
	unsigned long data
	)
{
	PLED_871x	 	pLed = (PLED_871x)data;
	_adapter			*padapter = pLed->padapter;
	struct led_priv	*ledpriv = &(padapter->ledpriv);

	if( (padapter->bSurpriseRemoved == _TRUE) || ( padapter->bDriverStopped == _TRUE))
       {
             return;
       }

	switch(ledpriv->LedStrategy)
	{
		case SW_LED_MODE1:
		//		SwLedBlink(pLed);
			break;
		case SW_LED_MODE2:
		//		SwLedBlink(pLed);
			break;
		case SW_LED_MODE3:
		//		SwLedBlink(pLed);
			break;
		case SW_LED_MODE5:
		//		SwLedBlink5(pLed);
			break;
		case SW_LED_MODE6:
		//		SwLedBlink6(pLed);
				break;
		case SW_LED_MODE7:
			SwLedBlink7(pLed);
			break;
		case SW_LED_MODE8:
			SwLedBlink8(pLed);
			break;

		case SW_LED_MODE9:
			SwLedBlink9(pLed);
			break;			

		case SW_LED_MODE10:
			SwLedBlink10(pLed);
			break;

		default:
	//			SwLedBlink(pLed);
			break;
	}
}



//================================================================================
// Default LED behavior.
//================================================================================

//
//	Description:	
//		Implement each led action for SW_LED_MODE0.
//		This is default strategy.
//
static void
SwLedControlMode0(
	_adapter		*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);
	PLED_871x	pLed0 = &(ledpriv->SwLed0);
	PLED_871x	pLed1 = &(ledpriv->SwLed1);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_TX:
		case LED_CTL_RX:	
			break;

		case LED_CTL_LINK:
			pLed0->CurrLedState = RTW_LED_ON;
			SwLedOn(padapter, pLed0);

			pLed1->CurrLedState = LED_BLINK_NORMAL;
			HwLedBlink(padapter, pLed1);
			break;

		case LED_CTL_POWER_ON:
			pLed0->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed0);

			pLed1->CurrLedState = LED_BLINK_NORMAL;
			HwLedBlink(padapter, pLed1);

			break;

		case LED_CTL_POWER_OFF:
			pLed0->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed0);

			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);
			break;

		case LED_CTL_SITE_SURVEY:
			break;

		case LED_CTL_NO_LINK:
			pLed0->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed0);

			pLed1->CurrLedState = LED_BLINK_NORMAL;
			HwLedBlink(padapter, pLed1);
			break;	

		default:
			break;
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Led0 %d Led1 %d\n", pLed0->CurrLedState, pLed1->CurrLedState));
	
}


static void
SwLedControlMode1(
	_adapter		*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv		*ledpriv = &(padapter->ledpriv);
	PLED_871x			pLed = &(ledpriv->SwLed0);
	struct mlme_priv		*pmlmepriv = &(padapter->mlmepriv);

	switch(LedAction)
	{
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				pLed->bLedBlinkInProgress = _TRUE;

				pLed->CurrLedState = LED_BLINK_NORMAL;
				pLed->BlinkTimes = 2;

				if( pLed->bLedOn )
					pLed->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
			}
			break;

		case LED_CTL_SITE_SURVEY:
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				pLed->bLedBlinkInProgress = _TRUE;

				if( (check_fwstate(pmlmepriv, _FW_LINKED) && check_fwstate(pmlmepriv,WIFI_STATION_STATE)) ||
					(check_fwstate(pmlmepriv,WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) )	
				{
					pLed->CurrLedState = LED_SCAN_BLINK;
					pLed->BlinkTimes = 4;
				}
				else
				{
					pLed->CurrLedState = LED_NO_LINK_BLINK;
					pLed->BlinkTimes = 24;
				}

				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_OFF_INTERVAL);
				}
			}
			else
			{
				if(pLed->CurrLedState != LED_NO_LINK_BLINK)
				{
					if( (check_fwstate(pmlmepriv, _FW_LINKED) && check_fwstate(pmlmepriv, WIFI_STATION_STATE)) ||
						(check_fwstate(pmlmepriv, WIFI_ADHOC_STATE) || check_fwstate(pmlmepriv, WIFI_ADHOC_MASTER_STATE)) )	
					{
						pLed->CurrLedState = LED_SCAN_BLINK;
					}
					else
					{
						pLed->CurrLedState = LED_NO_LINK_BLINK;
					}
				}
			}
			break;

		case LED_CTL_NO_LINK:
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				pLed->bLedBlinkInProgress = _TRUE;

				pLed->CurrLedState = LED_NO_LINK_BLINK;
				pLed->BlinkTimes = 24;

				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_OFF_INTERVAL);
				}
			}
			else
			{
				pLed->CurrLedState = LED_NO_LINK_BLINK;
			}
			break;

		case LED_CTL_LINK:
			pLed->CurrLedState = RTW_LED_ON;
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				SwLedOn(padapter, pLed);
			}
			break;

		case LED_CTL_POWER_OFF:
			pLed->CurrLedState = RTW_LED_OFF;
			if(pLed->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed);
			break;
				
		default:
			break;

	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Led %d \n", pLed->CurrLedState));
}


static void
SwLedControlMode2(
	_adapter				*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	 *ledpriv = &(padapter->ledpriv);
	PLED_871x 		pLed0 = &(ledpriv->SwLed0);
	PLED_871x 		pLed1 = &(ledpriv->SwLed1);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
			pLed0->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed0);
			
			pLed1->CurrLedState = LED_BLINK_CAMEO;
			if( pLed1->bLedBlinkInProgress == _FALSE )
			{
				pLed1->bLedBlinkInProgress = _TRUE;

				pLed1->BlinkTimes = 6;
			
				if( pLed1->bLedOn )
					pLed1->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed1->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed1->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
			}
			break;

		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed0->bLedBlinkInProgress == _FALSE )
			{
				pLed0->bLedBlinkInProgress = _TRUE;

				pLed0->CurrLedState = LED_BLINK_TXRX;
				pLed0->BlinkTimes = 2;

				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
				
				_set_timer(&(pLed0->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
			}
			break;

		case LED_CTL_NO_LINK:
			pLed1->CurrLedState = LED_BLINK_CAMEO;
			if( pLed1->bLedBlinkInProgress == _FALSE )
			{
				pLed1->bLedBlinkInProgress = _TRUE;
				
				pLed1->BlinkTimes = 6;

				if( pLed1->bLedOn )
					pLed1->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed1->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed1->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
			}
			break;

		case LED_CTL_LINK:
			pLed1->CurrLedState = RTW_LED_ON;
			if( pLed1->bLedBlinkInProgress == _FALSE )
			{
				SwLedOn(padapter, pLed1);
			}
			break;

		case LED_CTL_POWER_OFF:
			pLed0->CurrLedState = RTW_LED_OFF;
			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed0);
			SwLedOff(padapter, pLed1);
			break;

		default:
			break;

	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Led0 %d, Led1 %d \n", pLed0->CurrLedState, pLed1->CurrLedState));
}


 static void
 SwLedControlMode3(
	_adapter				*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);
	PLED_871x		pLed0 = &(ledpriv->SwLed0);
	PLED_871x		pLed1 = &(ledpriv->SwLed1);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
			pLed0->CurrLedState = RTW_LED_ON;
			SwLedOn(padapter, pLed0);
			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);
			break;
			
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed1->bLedBlinkInProgress == _FALSE )
			{
				pLed1->bLedBlinkInProgress = _TRUE;

				pLed1->CurrLedState = LED_BLINK_RUNTOP;
				pLed1->BlinkTimes = 2;
			
				if( pLed1->bLedOn )
					pLed1->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed1->BlinkingLedState = RTW_LED_ON; 
				
				_set_timer(&(pLed1->BlinkTimer), LED_RunTop_BLINK_INTERVAL);
			}
			break;

		case LED_CTL_POWER_OFF:
			pLed0->CurrLedState = RTW_LED_OFF;
			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed0);
			SwLedOff(padapter, pLed1);
			break;

		default:
			break;
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Led0 %d, Led1 %d \n", pLed0->CurrLedState, pLed1->CurrLedState));
}


static void
SwLedControlMode4(
	_adapter				*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);
	PLED_871x		pLed0 = &(ledpriv->SwLed0);
	PLED_871x		pLed1 = &(ledpriv->SwLed1);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
			pLed1->CurrLedState = RTW_LED_ON;
			SwLedOn(padapter, pLed1);
			pLed0->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed0);
			break;
			
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed0->bLedBlinkInProgress == _FALSE )
			{
				pLed0->bLedBlinkInProgress = _TRUE;

				pLed0->CurrLedState = LED_BLINK_RUNTOP;
				pLed0->BlinkTimes = 2;
			
				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
				
				_set_timer(&(pLed0->BlinkTimer), LED_RunTop_BLINK_INTERVAL);
			}
			break;

		case LED_CTL_POWER_OFF:
			pLed0->CurrLedState = RTW_LED_OFF;
			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed0);
			SwLedOff(padapter, pLed1);
			break;

		default:
			break;
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Led %d\n", pLed0->CurrLedState));
}



//added by vivi, for led new mode
static void
SwLedControlMode5(
	_adapter				*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);
	PLED_871x		pLed0 = &(ledpriv->SwLed0);
	PLED_871x		pLed1 = &(ledpriv->SwLed1);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
		case LED_CTL_START_TO_LINK:
		case LED_CTL_NO_LINK:
			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);


			if( pLed0->bLedSlowBlinkInProgress == _FALSE )
			{
				pLed0->bLedSlowBlinkInProgress = _TRUE;
				pLed0->CurrLedState = LED_BLINK_SLOWLY;
				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed0->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_NETTRONIX);
			}
			
			break;
		
		case LED_CTL_TX:
		case LED_CTL_RX:	
			pLed1->CurrLedState = RTW_LED_ON;
			SwLedOn(padapter, pLed1);

			if( pLed0->bLedBlinkInProgress == _FALSE )
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedSlowBlinkInProgress = _FALSE;
				pLed0->bLedBlinkInProgress = _TRUE;
				pLed0->CurrLedState = LED_BLINK_NORMAL;
				pLed0->BlinkTimes = 2;

				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed0->BlinkTimer), LED_BLINK_NORMAL_INTERVAL_NETTRONIX);
			}		
			break;

		case LED_CTL_LINK:
			pLed1->CurrLedState = RTW_LED_ON;
			SwLedOn(padapter, pLed1);

			if( pLed0->bLedSlowBlinkInProgress == _FALSE )
			{
				pLed0->bLedSlowBlinkInProgress = _TRUE;
				pLed0->CurrLedState = LED_BLINK_SLOWLY;
				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed0->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_NETTRONIX);
			}
			break;


		case LED_CTL_POWER_OFF:
			pLed0->CurrLedState = RTW_LED_OFF;
			pLed1->CurrLedState = RTW_LED_OFF;
			if( pLed0->bLedSlowBlinkInProgress == _TRUE )
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedSlowBlinkInProgress = _FALSE;
			}
			if(pLed0->bLedBlinkInProgress == _TRUE)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed0);
			SwLedOff(padapter, pLed1);
			break;

		default:
			break;
	}
}

//added by vivi, for led new mode
static void
SwLedControlMode6(
	_adapter				*padapter,
	LED_CTL_MODE		LedAction
)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);
	PLED_871x pLed0 = &(ledpriv->SwLed0);
	PLED_871x pLed1 = &(ledpriv->SwLed1);
	
	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
		case LED_CTL_START_TO_LINK:
		case LED_CTL_NO_LINK:
		case LED_CTL_LINK:
		case LED_CTL_SITE_SURVEY:
			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);
			
			if( pLed0->bLedSlowBlinkInProgress == _FALSE )
			{
				pLed0->bLedSlowBlinkInProgress = _TRUE;
				pLed0->CurrLedState = LED_BLINK_SLOWLY;
				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed0->BlinkTimer), LED_BLINK_SLOWLY_INTERVAL_PORNET);
			}
			break;

		case LED_CTL_TX:
		case LED_CTL_RX:
			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);
			if( pLed0->bLedBlinkInProgress == _FALSE )
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedSlowBlinkInProgress = _FALSE;
				pLed0->bLedBlinkInProgress = _TRUE;
				pLed0->CurrLedState = LED_BLINK_NORMAL;
				pLed0->BlinkTimes = 2;
				if( pLed0->bLedOn )
					pLed0->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed0->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed0->BlinkTimer), LED_BLINK_NORMAL_INTERVAL_PORNET);
			}		
			break;

		case LED_CTL_POWER_OFF:
			pLed1->CurrLedState = RTW_LED_OFF;
			SwLedOff(padapter, pLed1);
			
			pLed0->CurrLedState = RTW_LED_OFF;
			if( pLed0->bLedSlowBlinkInProgress == _TRUE )
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedSlowBlinkInProgress = _FALSE;
			}
			if(pLed0->bLedBlinkInProgress == _TRUE)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(padapter, pLed0);
			break;

		default:
			break;
	}
}


//added by chiyokolin, for Lenovo
VOID
SwLedControlMode7(
	IN	PADAPTER			Adapter,
	IN	LED_CTL_MODE		LedAction
	)
{
	struct led_priv	*ledpriv = &(Adapter->ledpriv);
	PLED_871x		pLed0 = &(ledpriv->SwLed0);	

	switch(LedAction)
	{
		case LED_CTL_POWER_ON:
		case LED_CTL_LINK:
		case LED_CTL_NO_LINK:			
			SwLedOn(Adapter, pLed0);
			break;

		case LED_CTL_POWER_OFF:
			SwLedOff(Adapter, pLed0);
			break;

		default:
			break;
	}
}

//added by chiyokolin, for QMI
VOID
SwLedControlMode8(
	IN	PADAPTER			Adapter,
	IN	LED_CTL_MODE		LedAction
	)
{
	struct led_priv	*ledpriv = &(Adapter->ledpriv);
	PLED_871x		pLed = &(ledpriv->SwLed0);
	struct mlme_priv	*pmlmepriv = &(Adapter->mlmepriv);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed->bLedBlinkInProgress == _FALSE && (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE))
			{
				pLed->bLedBlinkInProgress = _TRUE;

				pLed->CurrLedState = LED_BLINK_NORMAL;
				pLed->BlinkTimes = 2;

				if( pLed->bLedOn )
					pLed->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
			}
			break;

		case LED_CTL_SITE_SURVEY:
		case LED_CTL_POWER_ON:
		case LED_CTL_NO_LINK:
		case LED_CTL_LINK:
			pLed->CurrLedState = RTW_LED_ON;
			if(pLed->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->bLedBlinkInProgress = _FALSE;
			}
			SwLedOn(Adapter, pLed);
			break;

		case LED_CTL_POWER_OFF:
			pLed->CurrLedState = RTW_LED_OFF;
			if(pLed->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed);
			break;

		default:
			break;
		}
}


//added by chiyokolin, for MSI
VOID
SwLedControlMode9(
	IN	PADAPTER			Adapter,
	IN	LED_CTL_MODE		LedAction
	)
{
	struct led_priv	*ledpriv = &(Adapter->ledpriv);
	PLED_871x		pLed = &(ledpriv->SwLed0);	
	struct mlme_priv	*pmlmepriv = &(Adapter->mlmepriv);

	// Decide led state
	switch(LedAction)
	{
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed->bLedBlinkInProgress == _FALSE && (check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE))
			{
				pLed->bLedBlinkInProgress = _TRUE;

				pLed->CurrLedState = LED_BLINK_NORMAL;
				pLed->BlinkTimes = 2;

				if( pLed->bLedOn )
					pLed->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed->BlinkTimer), LED_BLINK_FAST_INTERVAL_BITLAND);
			}
			break;

		case LED_CTL_SITE_SURVEY:
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				pLed->bLedBlinkInProgress = _TRUE;
				pLed->CurrLedState = LED_SCAN_BLINK;
				pLed->BlinkTimes = 2;
			
				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
				}				
			}
			else if(pLed->CurrLedState != LED_SCAN_BLINK)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->CurrLedState = LED_SCAN_BLINK;
				pLed->BlinkTimes = 2;

				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
				}				
			}			
			break;

		case LED_CTL_POWER_ON:
		case LED_CTL_NO_LINK:
			if( pLed->bLedBlinkInProgress == _FALSE )
			{
				pLed->bLedBlinkInProgress = _TRUE;

				pLed->CurrLedState = LED_NO_LINK_BLINK;
				pLed->BlinkTimes = 24;

				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
				}
			}
			else if(pLed->CurrLedState != LED_SCAN_BLINK && pLed->CurrLedState != LED_NO_LINK_BLINK)
			{
				pLed->CurrLedState = LED_NO_LINK_BLINK;
				pLed->BlinkTimes = 24;

				if( pLed->bLedOn )
				{
					pLed->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed->BlinkTimer), LED_CM2_BLINK_ON_INTERVAL);
				}
				else
				{
					pLed->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed->BlinkTimer), LED_CM8_BLINK_OFF_INTERVAL);
				}
			}
			break;

		case LED_CTL_LINK:
			pLed->CurrLedState = RTW_LED_ON;
			if(pLed->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->bLedBlinkInProgress = _FALSE;
			}
			SwLedOn(Adapter, pLed);
			break;

		case LED_CTL_POWER_OFF:
			pLed->CurrLedState = RTW_LED_OFF;
			if(pLed->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed->BlinkTimer));
				pLed->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed);
			break;

		default:
			break;
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("Ledcontrol 9 current led state %d, \n", pLed->CurrLedState));
	
}

//added by chiyokolin, for Edimax-ASUS
VOID
SwLedControlMode10(
	IN	PADAPTER			Adapter,
	IN	LED_CTL_MODE		LedAction
	)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	struct led_priv	*ledpriv = &(Adapter->ledpriv);
	PLED_871x pLed0 = &(ledpriv->SwLed0);
	PLED_871x pLed1 = &(ledpriv->SwLed1);
	struct mlme_priv	*pmlmepriv = &(Adapter->mlmepriv);

	if(IS_92C_SERIAL(pHalData->VersionID))
	{
		pLed0 = &(ledpriv->SwLed1);	
		pLed1 = &(ledpriv->SwLed0);			
	}

	// Decide led state
	switch(LedAction)
	{	
#if 1	
		case LED_CTL_TX:
		case LED_CTL_RX:
			if( pLed1->bLedBlinkInProgress == _FALSE && pLed1->bLedWPSBlinkInProgress == _FALSE && 
				(check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE))
			{
				pLed1->bLedBlinkInProgress = _TRUE;

				pLed1->CurrLedState = LED_BLINK_NORMAL;
				pLed1->BlinkTimes = 2;

				if( pLed1->bLedOn )
					pLed1->BlinkingLedState = RTW_LED_OFF; 
				else
					pLed1->BlinkingLedState = RTW_LED_ON; 
				_set_timer(&(pLed1->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
			}			
			break;

		case LED_CTL_SITE_SURVEY:	
			if( pLed1->bLedBlinkInProgress == _FALSE && pLed1->bLedWPSBlinkInProgress == _FALSE)
			{
				pLed1->bLedBlinkInProgress = _TRUE;
				pLed1->CurrLedState = LED_SCAN_BLINK;
				pLed1->BlinkTimes = 12;
			
				if( pLed1->bLedOn )
				{
					pLed1->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed1->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				}
				else
				{
					pLed1->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed1->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				}				
			}
			else if(pLed1->CurrLedState != LED_SCAN_BLINK && pLed1->bLedWPSBlinkInProgress == _FALSE)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->CurrLedState = LED_SCAN_BLINK;
				pLed1->BlinkTimes = 24;

				if( pLed1->bLedOn )
				{
					pLed1->BlinkingLedState = RTW_LED_OFF; 
					_set_timer(&(pLed1->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				}
				else
				{
					pLed1->BlinkingLedState = RTW_LED_ON; 
					_set_timer(&(pLed1->BlinkTimer), LED_BLINK_NORMAL_INTERVAL);
				}				
			}						
			break;
			
		case LED_CTL_START_WPS:	
		case LED_CTL_START_WPS_BOTTON: 
			pLed1->CurrLedState = RTW_LED_ON;
			if(pLed1->bLedBlinkInProgress == _TRUE)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;					
			}

			if(pLed1->bLedWPSBlinkInProgress == _FALSE)
			{
				pLed1->bLedWPSBlinkInProgress = _TRUE;
				SwLedOn(Adapter, pLed1);					
			}
			break;
			
		case 	LED_CTL_STOP_WPS:
		case 	LED_CTL_STOP_WPS_FAIL:
		case 	LED_CTL_STOP_WPS_FAIL_OVERLAP:
			if(check_fwstate(pmlmepriv, _FW_LINKED) == _TRUE)
			{
				pLed0->CurrLedState = RTW_LED_ON;
				if(pLed0->bLedBlinkInProgress)
				{
					_cancel_timer_ex(&(pLed0->BlinkTimer));
					pLed0->bLedBlinkInProgress = _FALSE;
				}
				SwLedOn(Adapter, pLed0);
			}
			else
			{
				pLed0->CurrLedState = RTW_LED_OFF;
				if(pLed0->bLedBlinkInProgress)
				{
					_cancel_timer_ex(&(pLed0->BlinkTimer));
					pLed0->bLedBlinkInProgress = _FALSE;
				}
				SwLedOff(Adapter, pLed0);
			}

			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed1);			
			
			pLed1->bLedWPSBlinkInProgress = _FALSE;
			
			break;

		case LED_CTL_LINK:
			pLed0->CurrLedState = RTW_LED_ON;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			SwLedOn(Adapter, pLed0);
			break;
			
		case LED_CTL_NO_LINK:
			if(pLed1->bLedWPSBlinkInProgress == _TRUE)
			{			
				SwLedOn(Adapter, pLed1);					
				break;
			}

			if(pLed1->CurrLedState == LED_SCAN_BLINK)
				break;;		
			
			pLed0->CurrLedState = RTW_LED_OFF;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed0);

			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed1);			

			break;


		case LED_CTL_POWER_ON:
		case LED_CTL_POWER_OFF:
			if(pLed1->bLedWPSBlinkInProgress == _TRUE)
			{			
				SwLedOn(Adapter, pLed1);					
				break;
			}
			pLed0->CurrLedState = RTW_LED_OFF;
			if(pLed0->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed0->BlinkTimer));
				pLed0->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed0);

			pLed1->CurrLedState = RTW_LED_OFF;
			if(pLed1->bLedBlinkInProgress)
			{
				_cancel_timer_ex(&(pLed1->BlinkTimer));
				pLed1->bLedBlinkInProgress = _FALSE;
			}
			SwLedOff(Adapter, pLed1);			

			break;

		default:
			break;
			
#else

	case LED_CTL_POWER_OFF:
		SwLedOff(Adapter, pLed0);
		SwLedOff(Adapter, pLed1);			
		break;

	default:	
		SwLedOn(Adapter, pLed0);
		SwLedOn(Adapter, pLed1);			
		break;
		
#endif
	}

	//DBG_8192C("Ledcontrol 10 current led0 state %d led1 state %d, \n", pLed0->CurrLedState, pLed1->CurrLedState);
	
}


/*-----------------------------------------------------------------------------
 * Function:	gen_RefreshLedState()
 *
 * Overview:	When we call the function, media status is no link. It must be in SW/HW
 *			radio off. Or IPS state. If IPS no link we will turn on LED, otherwise, we must turn off.
 *			After MAC IO reset, we must write LED control 0x2f2 again.
 *
 * Input:		IN	PADAPTER			Adapter)
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	03/27/2009	MHC		Create for LED judge only~!!   
 *
 *---------------------------------------------------------------------------*/
VOID
rtl8192ce_gen_RefreshLedState(
	IN	PADAPTER			Adapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct pwrctrl_priv	*pwrctrlpriv = &Adapter->pwrctrlpriv;
	struct led_priv	*pledpriv = &(Adapter->ledpriv);
	PLED_871x		pLed0 = &(Adapter->ledpriv.SwLed0);

	DBG_8192C("gen_RefreshLedState:() pwrctrlpriv->rfoff_reason=%x\n", pwrctrlpriv->rfoff_reason);

	if(Adapter->bDriverIsGoingToUnload)
	{
		switch(pledpriv->LedStrategy)
		{
			case SW_LED_MODE9:
			case SW_LED_MODE10:
				rtw_led_control(Adapter, LED_CTL_POWER_OFF);
				break;				

			default:
				// Turn off LED if RF is not ON.
				SwLedOff(Adapter, pLed0);		
				break;				
		}	
	}
	else if(pwrctrlpriv->rfoff_reason == RF_CHANGE_BY_IPS )
	{
		switch(pledpriv->LedStrategy)
		{
			case SW_LED_MODE7:
				SwLedOn(Adapter, pLed0);
				break;

			case SW_LED_MODE8:
			case SW_LED_MODE9:
				rtw_led_control(Adapter, LED_CTL_NO_LINK);
				break;					

			default:	
				SwLedOn(Adapter, pLed0);
				break;					
		}
	}
	else if(pwrctrlpriv->rfoff_reason == RF_CHANGE_BY_INIT)
	{
		switch(pledpriv->LedStrategy)
		{
			case SW_LED_MODE7:
				SwLedOn(Adapter, pLed0);
				break;

			case SW_LED_MODE9:
				rtw_led_control(Adapter, LED_CTL_NO_LINK);
				break;				

			default:
				SwLedOn(Adapter, pLed0);
				break;
				
		}		
	}
	else		// SW/HW radio off
	{

		switch(pledpriv->LedStrategy)
		{
			case SW_LED_MODE9:
				rtw_led_control(Adapter, LED_CTL_POWER_OFF);
				break;				

			default:
				// Turn off LED if RF is not ON.
				SwLedOff(Adapter, pLed0);		
				break;				
		}	
	}

}	// gen_RefreshLedState

//
//	Description:	
//		Dispatch LED action according to pHalData->LedStrategy. 
//
void
LedControl8192CE(
	IN	PADAPTER			Adapter,
	IN	LED_CTL_MODE		LedAction
	)
{
	struct led_priv	*ledpriv = &(Adapter->ledpriv);

#if(MP_DRIVER == 1)
	return;
#endif

       if( (Adapter->bSurpriseRemoved == _TRUE) || ( Adapter->bDriverStopped == _TRUE))	
       {
             return;
       }

	//if(priv->bInHctTest)
	//	return;
	
	if(	//pHalData->eRFPowerState != eRfOn && //marked by tynli.
		(Adapter->pwrctrlpriv.rfoff_reason > RF_CHANGE_BY_PS) &&
		(LedAction == LED_CTL_TX || 
		 LedAction == LED_CTL_RX || 
		 LedAction == LED_CTL_SITE_SURVEY || 
		 LedAction == LED_CTL_LINK || 
		 LedAction == LED_CTL_NO_LINK||
		 LedAction == LED_CTL_START_TO_LINK ||
		 LedAction == LED_CTL_POWER_ON) )
	{
		return;
	}

	RT_TRACE(_module_rtl8712_led_c_,_drv_info_,("LedAction %d, \n", LedAction));

	switch(ledpriv->LedStrategy)
	{
		case SW_LED_MODE0:
			//SwLedControlMode0(Adapter, LedAction);
			break;
		case SW_LED_MODE1:
			//SwLedControlMode1(Adapter, LedAction);
			break;
		case SW_LED_MODE2:
			//SwLedControlMode2(Adapter, LedAction);
			break;
		case SW_LED_MODE3:
			//SwLedControlMode3(Adapter, LedAction);
			break;
		case SW_LED_MODE4:
			//SwLedControlMode4(Adapter, LedAction);
			break;
		//added by vivi, for led new mode, DLINK
		case SW_LED_MODE5:
			//SwLedControlMode5(Adapter, LedAction);
			break;

		//added by vivi, for led new mode, PRONET
		case SW_LED_MODE6:
			//SwLedControlMode6(Adapter, LedAction);
			break;

		case SW_LED_MODE7:
			SwLedControlMode7(Adapter, LedAction);
			break;

		case SW_LED_MODE8:
			SwLedControlMode8(Adapter, LedAction);
			break;

		case SW_LED_MODE9:
			SwLedControlMode9(Adapter, LedAction);
			break;

		case SW_LED_MODE10:
			SwLedControlMode10(Adapter, LedAction);
			break;

		default:
			break;
	}
}

//
//	Description:
//		Initialize all LED_871x objects.
//
void
rtl8192ce_InitSwLeds(
	_adapter	*padapter
	)
{
	struct led_priv *pledpriv = &(padapter->ledpriv);

	pledpriv->LedControlHandler = LedControl8192CE;

	InitLed871x(padapter, &(pledpriv->SwLed0), LED_PIN_LED0);

	InitLed871x(padapter,&(pledpriv->SwLed1), LED_PIN_LED1);
}


//
//	Description:
//		DeInitialize all LED_819xUsb objects.
//
void
rtl8192ce_DeInitSwLeds(
	_adapter	*padapter
	)
{
	struct led_priv	*ledpriv = &(padapter->ledpriv);

	DeInitLed871x( &(ledpriv->SwLed0) );
	DeInitLed871x( &(ledpriv->SwLed1) );
}

