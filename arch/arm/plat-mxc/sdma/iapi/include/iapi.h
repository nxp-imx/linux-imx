/******************************************************************************
 *
 * Copyright 2007-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 ******************************************************************************
 *
 * File: iapi.h
 *
 * $Id iapi.h $
 *
 * Description:
 *  Unique include for the whole IAPI library.
 *
 *
 *  http//compass.mot.com/go/115342679
 *
 * $Log iapi.h $
 *
 * ***************************************************************************/

#ifndef _iapi_h
#define _iapi_h

/* ****************************************************************************
 * Include File Section
 * ***************************************************************************/

#include "sdmaStruct.h"
#include "iapiDefaults.h"
#include "iapiLow.h"
#include "iapiMiddle.h"
#include "iapiHigh.h"

#ifdef MCU
#include "iapiLowMcu.h"
#include "iapiMiddleMcu.h"
#endif				/* MCU */

#endif				/* _iapi_h */
