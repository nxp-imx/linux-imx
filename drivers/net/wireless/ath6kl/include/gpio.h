//------------------------------------------------------------------------------
// <copyright file="gpio.h" company="Atheros">
//    Copyright (c) 2005 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

#define AR6001_GPIO_PIN_COUNT 18
#define AR6002_GPIO_PIN_COUNT 18
#define AR6003_GPIO_PIN_COUNT 28

/*
 * Possible values for WMIX_GPIO_SET_REGISTER_CMDID.
 * NB: These match hardware order, so that addresses can
 * easily be computed.
 */
#define GPIO_ID_OUT             0x00000000
#define GPIO_ID_OUT_W1TS        0x00000001
#define GPIO_ID_OUT_W1TC        0x00000002
#define GPIO_ID_ENABLE          0x00000003
#define GPIO_ID_ENABLE_W1TS     0x00000004
#define GPIO_ID_ENABLE_W1TC     0x00000005
#define GPIO_ID_IN              0x00000006
#define GPIO_ID_STATUS          0x00000007
#define GPIO_ID_STATUS_W1TS     0x00000008
#define GPIO_ID_STATUS_W1TC     0x00000009
#define GPIO_ID_PIN0            0x0000000a
#define GPIO_ID_PIN(n)          (GPIO_ID_PIN0+(n))

#define GPIO_LAST_REGISTER_ID   GPIO_ID_PIN(17)
#define GPIO_ID_NONE            0xffffffff
