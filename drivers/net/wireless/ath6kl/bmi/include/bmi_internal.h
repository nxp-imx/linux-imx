//------------------------------------------------------------------------------
// <copyright file="bmi_internal.h" company="Atheros">
//    Copyright (c) 2004-2008 Atheros Corporation.  All rights reserved.
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
//
// Author(s): ="Atheros"
//==============================================================================
#ifndef BMI_INTERNAL_H
#define BMI_INTERNAL_H

#include "a_config.h"
#include "athdefs.h"
#include "a_types.h"
#include "a_osapi.h"
#define ATH_MODULE_NAME bmi
#include "a_debug.h"
#include "AR6002/hw2.0/hw/mbox_host_reg.h"
#include "bmi_msg.h"

#define ATH_DEBUG_BMI  ATH_DEBUG_MAKE_MODULE_MASK(0)


#define BMI_COMMUNICATION_TIMEOUT       100000

/* ------ Global Variable Declarations ------- */
A_BOOL bmiDone;

A_STATUS
bmiBufferSend(HIF_DEVICE *device,
              A_UCHAR *buffer,
              A_UINT32 length);

A_STATUS
bmiBufferReceive(HIF_DEVICE *device,
                 A_UCHAR *buffer,
                 A_UINT32 length,
                 A_BOOL want_timeout);

#endif
