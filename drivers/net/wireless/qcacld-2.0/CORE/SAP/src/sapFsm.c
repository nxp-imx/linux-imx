/*
 * Copyright (c) 2012-2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/*===========================================================================

                      s a p F s m . C

  OVERVIEW:

  This software unit holds the implementation of the WLAN SAP Finite
  State Machine modules

  DEPENDENCIES:

  Are listed for each API below.
===========================================================================*/

/*===========================================================================

                      EDIT HISTORY FOR FILE


  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.



  when        who     what, where, why
----------    ---    --------------------------------------------------------
2010-03-15         Created module

===========================================================================*/


/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "sapInternal.h"
// Pick up the SME API definitions
#include "sme_Api.h"
#include "smeInside.h"
// Pick up the PMC API definitions
#include "pmcApi.h"
#include "wlan_nv.h"
#include "vos_utils.h"
#include "_ieee80211_common.h"

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Global Data Definitions
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *  External declarations for global context
 * -------------------------------------------------------------------------*/
#ifdef FEATURE_WLAN_CH_AVOID
extern sapSafeChannelType safeChannels[];
#endif /* FEATURE_WLAN_CH_AVOID */

/*----------------------------------------------------------------------------
 * Static Variable Definitions
 * -------------------------------------------------------------------------*/

#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
/*
 * TODO: At present SAP Channel leakage matrix for ch 144
 * is not available from system's team. So to play it safe
 * and avoid crash if channel 144 is request, in following
 * matix channel 144 is added such that it will cause code
 * to avoid selecting channel 144.
 *
 * THESE ENTRIES SHOULD BE REPLACED WITH CORRECT VALUES AS
 * PROVIDED BY SYSTEM'S TEAM.
 */
/* channel tx leakage table - ht80 */
tSapChanMatrixInfo ht80_chan[] =
{
 {52,
   {{36, 148},            {40, 199},
    {44, 193},            {48, 197},
    {52, SAP_TX_LEAKAGE_MIN}, {56, 153},
    {60, 137},            {64, 134},
    {100, 358},           {104, 350},
    {108, 404},           {112, 344},
    {116, 424},           {120, 429},
    {124, 437},           {128, 435},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},


 {56,
   {{36, 171},            {40, 178},
    {44, 171},            {48, 178},
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_MIN}, {64, 280},
    {100, 351},           {104, 376},
    {108, 362},           {112, 362},
    {116, 403},           {120, 397},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {60,
   {{36, 156},            {40, 146},
    {44, SAP_TX_LEAKAGE_MIN}, {48, SAP_TX_LEAKAGE_MIN},
    {52, 180},            {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 376},           {104, 360},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, 395},           {120, 399},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {64,
   {{36, 217},            {40, 221},
    {44, SAP_TX_LEAKAGE_MIN}, {48, SAP_TX_LEAKAGE_MIN},
    {52, 176},            {56, 176},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 384},           {104, 390},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, 375},           {120, 374},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {100,
   {{36, 357},            {40, 326},
    {44, 321},            {48, 326},
    {52, 378},            {56, 396},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, SAP_TX_LEAKAGE_MIN},{104, SAP_TX_LEAKAGE_MIN},
    {108, 196},           {112, 116},
    {116, 166},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {104,
   {{36, 325},            {40, 325},
    {44, 305},            {48, 352},
    {52, 411},            {56, 411},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, SAP_TX_LEAKAGE_MIN},{104, SAP_TX_LEAKAGE_MIN},
    {108, SAP_TX_LEAKAGE_MIN},{112, 460},
    {116, 198},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {108,
   {{36, 304},            {40, 332},
    {44, 310},            {48, 335},
    {52, 431},            {56, 391},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 280},           {104, SAP_TX_LEAKAGE_MIN},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, 185},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {112,
   {{36, 327},            {40, 335},
    {44, 331},            {48, 345},
    {52, 367},            {56, 401},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 131},           {104, 132},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, 189},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {116,
   {{36, 384},            {40, 372},
    {44, 389},            {48, 396},
    {52, 348},            {56, 336},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 172},           {104, 169},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {120,
   {{36, 395},            {40, 419},
    {44, 439},            {48, 407},
    {52, 321},            {56, 334},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 134},           {104, 186},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, 159},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {124,
   {{36, 469},            {40, 433},
    {44, 434},            {48, 435},
    {52, 332},            {56, 345},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 146},           {104, 177},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, 350},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, 138},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {128,
   {{36, 408},            {40, 434},
    {44, 449},            {48, 444},
    {52, 341},            {56, 374},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 205},           {104, 208},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, 142},           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {132,
   {{36, SAP_TX_LEAKAGE_MAX },            {40, SAP_TX_LEAKAGE_MAX },
    {44, SAP_TX_LEAKAGE_MAX },            {48, SAP_TX_LEAKAGE_MAX },
    {52, SAP_TX_LEAKAGE_MAX }, {56, SAP_TX_LEAKAGE_MAX },
    {60, SAP_TX_LEAKAGE_MIN }, {64, SAP_TX_LEAKAGE_MIN },
    {100, SAP_TX_LEAKAGE_MIN },{104, SAP_TX_LEAKAGE_MIN },
    {108, SAP_TX_LEAKAGE_MIN },           {112, SAP_TX_LEAKAGE_MIN },
    {116, SAP_TX_LEAKAGE_MIN },{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN }
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {136,
   {{36, SAP_TX_LEAKAGE_MAX },            {40, SAP_TX_LEAKAGE_MAX },
    {44, SAP_TX_LEAKAGE_MAX },            {48, SAP_TX_LEAKAGE_MAX },
    {52, SAP_TX_LEAKAGE_MAX }, {56, SAP_TX_LEAKAGE_MAX },
    {60, SAP_TX_LEAKAGE_MIN }, {64, SAP_TX_LEAKAGE_MIN },
    {100, SAP_TX_LEAKAGE_MIN },{104, SAP_TX_LEAKAGE_MIN },
    {108, SAP_TX_LEAKAGE_MIN },{112, SAP_TX_LEAKAGE_MIN },
    {116, SAP_TX_LEAKAGE_MIN },           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN }
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {140,
   {{36, SAP_TX_LEAKAGE_MAX },            {40, SAP_TX_LEAKAGE_MAX },
    {44, SAP_TX_LEAKAGE_MAX },            {48, SAP_TX_LEAKAGE_MAX },
    {52, SAP_TX_LEAKAGE_MAX },            {56, SAP_TX_LEAKAGE_MAX },
    {60, SAP_TX_LEAKAGE_MIN },            {64, SAP_TX_LEAKAGE_MIN },
    {100, SAP_TX_LEAKAGE_MIN },{104, SAP_TX_LEAKAGE_MIN },
    {108, SAP_TX_LEAKAGE_MIN },{112, SAP_TX_LEAKAGE_MIN },
    {116, SAP_TX_LEAKAGE_MIN },           {120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

#ifdef FEATURE_WLAN_CH144
 {144,
   {{36, 695},            {40, 695 },
    {44, 684},            {48, 684 },
    {52, 664},            {56, 664 },
    {60, 658},            {64, 658 },
    {100, 601},{104, 601 },
    {108, 545},{112, 545 },
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN}, {128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, 262},{136, 262},
    {140, SAP_TX_LEAKAGE_MIN},
    {144, SAP_TX_LEAKAGE_MIN}
}},
#endif
};

/* channel tx leakage table - ht40 */
tSapChanMatrixInfo ht40_chan[] =
{
 {52,
   {{36, SAP_TX_LEAKAGE_AUTO_MIN}, {40, SAP_TX_LEAKAGE_AUTO_MIN},
    {44, 230},            {48, 230 },
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_AUTO_MIN}, {64, SAP_TX_LEAKAGE_AUTO_MIN},
    {100, 625},           {104, 323 },
    {108, 646},{112, 646 },
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {56,
   {{36, SAP_TX_LEAKAGE_AUTO_MIN}, {40, SAP_TX_LEAKAGE_AUTO_MIN},
    {44, SAP_TX_LEAKAGE_AUTO_MIN}, {48, SAP_TX_LEAKAGE_AUTO_MIN},
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 611},{104, 611 },
    {108, 617},{112, 617 },
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {60,
   {{36, SAP_TX_LEAKAGE_AUTO_MIN}, {40,SAP_TX_LEAKAGE_AUTO_MIN},
    {44, SAP_TX_LEAKAGE_AUTO_MIN}, {48, SAP_TX_LEAKAGE_AUTO_MIN},
    {52, 190}, {56, 190},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 608},{104, 608 },
    {108, 623},{112, 623 },
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {64,
   {{36, SAP_TX_LEAKAGE_AUTO_MIN}, {40, SAP_TX_LEAKAGE_AUTO_MIN},
    {44, SAP_TX_LEAKAGE_AUTO_MIN}, {48, SAP_TX_LEAKAGE_AUTO_MIN},
    {52, 295},            {56, 295 },
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 594},           {104, 594 },
    {108, 625},{112, 625 },
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {100,
   {{36, 618},            {40, 618 },
    {44, 604},            {48, 604 },
    {52, 596},            {56, 596 },
    {60, 584},            {64, 584 },
    {100, SAP_TX_LEAKAGE_MIN},{104, SAP_TX_LEAKAGE_MIN },
    {108, 299},           {112, 299 },
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN}, {128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, 538},           {136, 538 },
    {140, 598}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {104,
   {{36, 636},            {40, 636 },
    {44, 601},            {48, 601 },
    {52, 616}, {56, 616 },
    {60, 584}, {64, 584 },
    {100, SAP_TX_LEAKAGE_MIN},{104, SAP_TX_LEAKAGE_MIN},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_AUTO_MIN},{120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN},{128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, 553},           {136, 553 },
    {140, 568}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {108,
   {{36, 600},            {40, 600 },
    {44, 627},            {48, 627 },
    {52, 611}, {56, 611 },
    {60, 611}, {64, 611 },
    {100, 214},{104, 214},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_AUTO_MIN},{120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN},{128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, SAP_TX_LEAKAGE_AUTO_MIN},{136, SAP_TX_LEAKAGE_AUTO_MIN},
    {140, 534}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {112,
   {{36, 645},            {40, 645 },
    {44, 641},            {48, 641 },
    {52, 618}, {56, 618 },
    {60, 612}, {64, 612 },
    {100, 293},{104, 293},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN},{128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, SAP_TX_LEAKAGE_AUTO_MIN},{136, SAP_TX_LEAKAGE_AUTO_MIN},
    {140, 521}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {116,
   {{36, 661},            {40, 661 },
    {44, 624},            {48, 624 },
    {52, 634}, {56, 634 },
    {60, 611},            {64, 611 },
    {100, SAP_TX_LEAKAGE_AUTO_MIN},{104, SAP_TX_LEAKAGE_AUTO_MIN},
    {108, 217},{112, 217 },
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN},{128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, SAP_TX_LEAKAGE_AUTO_MIN},{136, SAP_TX_LEAKAGE_AUTO_MIN},
    {140, SAP_TX_LEAKAGE_AUTO_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {120,
   {{36, 667},            {40, 667 },
    {44, 645},            {48, 645 },
    {52, 633}, {56, 633 },
    {60, 619}, {64, 619 },
    {100, SAP_TX_LEAKAGE_AUTO_MIN}, {104, SAP_TX_LEAKAGE_AUTO_MIN},
    {108, 291},{112, 291},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_AUTO_MIN},{136, SAP_TX_LEAKAGE_AUTO_MIN},
    {140, SAP_TX_LEAKAGE_AUTO_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {124,
   {{36, 676},            {40, 676 },
    {44, 668},            {48, 668 },
    {52, 595}, {56, 595 },
    {60, 622}, {64, 622 },
    {100, SAP_TX_LEAKAGE_AUTO_MIN}, {104, SAP_TX_LEAKAGE_AUTO_MIN},
    {108, SAP_TX_LEAKAGE_AUTO_MIN}, {112, SAP_TX_LEAKAGE_AUTO_MIN},
    {116, 225},{120, 225},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_AUTO_MIN},{136, SAP_TX_LEAKAGE_AUTO_MIN},
    {140, SAP_TX_LEAKAGE_AUTO_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {128,
   {{36, 678},            {40, 678 },
    {44, 664},            {48, 664 },
    {52, 651}, {56, 651 },
    {60, 643}, {64, 643 },
    {100, SAP_TX_LEAKAGE_AUTO_MIN}, {104, SAP_TX_LEAKAGE_AUTO_MIN},
    {108, SAP_TX_LEAKAGE_AUTO_MIN}, {112, SAP_TX_LEAKAGE_AUTO_MIN},
    {116, 293},{120, 293},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_AUTO_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {132,
   {{36, 689},            {40, 689 },
    {44, 669},            {48, 669 },
    {52, 662}, {56, 662 },
    {60, 609}, {64, 609 },
    {100, 538},{104, 538 },
    {108, SAP_TX_LEAKAGE_AUTO_MIN}, {112, SAP_TX_LEAKAGE_AUTO_MIN},
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, 247},{128, 247},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN }
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {136,
   {{36, 703},            {40, 703 },
    {44, 688},            {48, SAP_TX_LEAKAGE_MIN },
    {52, 671}, {56, 671 },
    {60, 658}, {64, 658 },
    {100, 504},{104, 504 },
    {108, SAP_TX_LEAKAGE_AUTO_MIN},{112, SAP_TX_LEAKAGE_AUTO_MIN},
    {116, SAP_TX_LEAKAGE_AUTO_MIN},{120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, 289},{128, 289},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN }
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {140,
   {{36, 695},            {40, 695 },
    {44, 684},            {48, 684 },
    {52, 664},            {56, 664 },
    {60, 658},            {64, 658 },
    {100, 601},{104, 601 },
    {108, 545},{112, 545 },
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN}, {128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, 262},{136, 262},
    {140, SAP_TX_LEAKAGE_MIN }
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

#ifdef FEATURE_WLAN_CH144
 {144,
   {{36, 695},            {40, 695 },
    {44, 684},            {48, 684 },
    {52, 664},            {56, 664 },
    {60, 658},            {64, 658 },
    {100, 601},{104, 601 },
    {108, 545},{112, 545 },
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, SAP_TX_LEAKAGE_AUTO_MIN},
    {124, SAP_TX_LEAKAGE_AUTO_MIN}, {128, SAP_TX_LEAKAGE_AUTO_MIN},
    {132, 262},{136, 262},
    {140, SAP_TX_LEAKAGE_MIN},
    {144, SAP_TX_LEAKAGE_MIN}
}},
#endif
};

/* channel tx leakage table - ht20 */
tSapChanMatrixInfo ht20_chan[] =
{
 {52,
   {{36, SAP_TX_LEAKAGE_AUTO_MIN}, {40, 286},
    {44, 225},            {48, 121},
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, 300},            {64, SAP_TX_LEAKAGE_AUTO_MIN},
    {100, 637},           {104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {56,
   {{36, 468},            {40, SAP_TX_LEAKAGE_AUTO_MIN},
    {44, SAP_TX_LEAKAGE_AUTO_MIN}, {48, 206},
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, SAP_TX_LEAKAGE_MAX},{104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {60,
   {{36, 507},            {40, 440},
    {44, SAP_TX_LEAKAGE_AUTO_MIN}, {48, 313},
    {52, SAP_TX_LEAKAGE_MIN}, {56, SAP_TX_LEAKAGE_MIN},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, SAP_TX_LEAKAGE_MAX},{104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {64,
   {{36, 516},            {40, 520},
    {44, 506},            {48, SAP_TX_LEAKAGE_AUTO_MIN},
    {52, 301},            {56, 258},
    {60, SAP_TX_LEAKAGE_MIN}, {64, SAP_TX_LEAKAGE_MIN},
    {100, 620},           {104, 617},
    {108, SAP_TX_LEAKAGE_MAX},{112, SAP_TX_LEAKAGE_MAX},
    {116, SAP_TX_LEAKAGE_MAX},{120, SAP_TX_LEAKAGE_MAX},
    {124, SAP_TX_LEAKAGE_MAX},{128, SAP_TX_LEAKAGE_MAX},
    {132, SAP_TX_LEAKAGE_MAX},{136, SAP_TX_LEAKAGE_MAX},
    {140, SAP_TX_LEAKAGE_MAX}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {100,
   {{36, 616},            {40, 601},
    {44, 604},            {48, 589},
    {52, 612},            {56, 592},
    {60, 590},            {64, 582},
    {100, SAP_TX_LEAKAGE_MIN},{104, 131},
    {108, SAP_TX_LEAKAGE_AUTO_MIN}, {112, SAP_TX_LEAKAGE_AUTO_MIN},
    {116, SAP_TX_LEAKAGE_AUTO_MIN}, {120, 522},
    {124, 571},           {128, 589},
    {132, 593},           {136, 598},
    {140, 594}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {104,
   {{36, 622},            {40, 624},
    {44, 618},            {48, 610},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, SAP_TX_LEAKAGE_MIN},{104, SAP_TX_LEAKAGE_MIN},
    {108, SAP_TX_LEAKAGE_MIN},{112, 463},
    {116, 483},{120, 503},
    {124, 523},           {128, 565},
    {132, 570},           {136, 588},
    {140, 585}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {108,
   {{36, 620},            {40, 638},
    {44, 611},            {48, 614},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 477},{104, SAP_TX_LEAKAGE_MIN},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, 477},{120, 497},
    {124, 517},{128, 537},
    {132, 557},{136, 577},
    {140, 603}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {112,
   {{36, 636},            {40, 623},
    {44, 638},            {48, 628},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, 606},
    {100, 501},{104, 481},
    {108, SAP_TX_LEAKAGE_MIN},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_MIN},{120, 481},
    {124, 501},{128, 421},
    {132, 541},{136, 561},
    {140, 583}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {116,
   {{36, 646},            {40, 648},
    {44, 633},            {48, 634},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, 615},            {64, 594},
    {100, 575},{104, 554},
    {108, 534},{112, SAP_TX_LEAKAGE_MIN},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, 534},{136, 554},
    {140, 574}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {120,
   {{36, 643},            {40, 649},
    {44, 654},            {48, 629},
    {52, SAP_TX_LEAKAGE_MAX}, {56, 621},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 565},           {104, 545},
    {108, 525},{112, 505},
    {116, SAP_TX_LEAKAGE_MIN},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, 505},
    {132, 525},{136, 545},
    {140, 565}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {124,
   {{36, 638},            {40, 657},
    {44, 663},            {48, 649},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 581},           {104, 561},
    {108, 541},{112, 521},
    {116, 499},{120, SAP_TX_LEAKAGE_MIN},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, 499},{136, 519},
    {140, 539}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {128,
   {{36, 651},            {40, 651},
    {44, 674},            {48, 640},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, 603},           {104, 560},
    {108, 540},{112, 520},
    {116, 499},{120, 479},
    {124, SAP_TX_LEAKAGE_MIN},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, 479},
    {140, 499}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {132,
   {{36, 643},            {40, 668},
    {44, 651},            {48, 657},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, SAP_TX_LEAKAGE_MAX},{104, 602},
    {108, 578},           {112, 570},
    {116, 550},{120, 530},
    {124, 510},{128, SAP_TX_LEAKAGE_MIN},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, 490}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {136,
   {{36, 654},            {40, 667},
    {44, 666},            {48, 642},
    {52, SAP_TX_LEAKAGE_MAX}, {56, SAP_TX_LEAKAGE_MAX},
    {60, SAP_TX_LEAKAGE_MAX}, {64, SAP_TX_LEAKAGE_MAX},
    {100, SAP_TX_LEAKAGE_MAX},{104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, 596},
    {116, 555},           {120, 535},
    {124, 515},{128, 495},
    {132, SAP_TX_LEAKAGE_MIN},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

 {140,
   {{36, 679},            {40, 673},
    {44, 667},            {48, 656},
    {52, 634},            {56, 663},
    {60, 662},            {64, 660},
    {100, SAP_TX_LEAKAGE_MAX},{104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, 590},
    {116, 573},           {120, 553},
    {124, 533},{128, 513},
    {132, 490},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN}
#ifdef FEATURE_WLAN_CH144
    ,{144, SAP_TX_LEAKAGE_MIN}
#endif
}},

#ifdef FEATURE_WLAN_CH144
 {144,
   {{36, 679},            {40, 673},
    {44, 667},            {48, 656},
    {52, 634},            {56, 663},
    {60, 662},            {64, 660},
    {100, SAP_TX_LEAKAGE_MAX},{104, SAP_TX_LEAKAGE_MAX},
    {108, SAP_TX_LEAKAGE_MAX},{112, 590},
    {116, 573},           {120, 553},
    {124, 533},{128, 513},
    {132, 490},{136, SAP_TX_LEAKAGE_MIN},
    {140, SAP_TX_LEAKAGE_MIN},
    {144, SAP_TX_LEAKAGE_MIN}
}},
#endif
};
#endif //end of WLAN_ENABLE_CHNL_MATRIX_RESTRICTION


/*----------------------------------------------------------------------------
 * Static Function Declarations and Definitions
 * -------------------------------------------------------------------------*/
#ifdef SOFTAP_CHANNEL_RANGE
static VOS_STATUS sapGetChannelList(ptSapContext sapContext, v_U8_t **channelList,
                                 v_U8_t  *numberOfChannels);
#endif

/*==========================================================================
  FUNCTION    sapGet5GHzChannelList

  DESCRIPTION
    Function for initializing list of  2.4/5 Ghz [NON-DFS/DFS] available
    channels in the current regulatory domain.

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext: SAP Context

  RETURN VALUE
    NA

  SIDE EFFECTS
============================================================================*/
static VOS_STATUS sapGet5GHzChannelList(ptSapContext sapContext);

/*==========================================================================
  FUNCTION    sapStopDfsCacTimer

  DESCRIPTION
    Function to sttop the DFS CAC timer when SAP is stopped
  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext: SAP Context
  RETURN VALUE
    DFS Timer start status
  SIDE EFFECTS
============================================================================*/

static int sapStopDfsCacTimer(ptSapContext sapContext);

/*==========================================================================
  FUNCTION    sapStartDfsCacTimer

  DESCRIPTION
    Function to start the DFS CAC timer when SAP is started on DFS Channel
  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext: SAP Context
  RETURN VALUE
    DFS Timer start status
  SIDE EFFECTS
============================================================================*/

int sapStartDfsCacTimer(ptSapContext sapContext);

/*----------------------------------------------------------------------------
 * Externalized Function Definitions
* -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Function Declarations and Documentation
 * -------------------------------------------------------------------------*/

/*==========================================================================
  FUNCTION    sapEventInit

  DESCRIPTION
    Function for initializing sWLAN_SAPEvent structure

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapEvent    : State machine event

  RETURN VALUE

    None

  SIDE EFFECTS
============================================================================*/
static inline void sapEventInit(ptWLAN_SAPEvent sapEvent)
{
   sapEvent->event = eSAP_MAC_SCAN_COMPLETE;
   sapEvent->params = 0;
   sapEvent->u1 = 0;
   sapEvent->u2 = 0;
}

#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
/*
 * This function gives the leakage matrix for given NOL channel and cbMode
 *
 * PARAMETERS
 * IN
 * sapContext        : Pointer to vos global context structure
 * cbMode            : target channel bonding mode
 * NOL_channel       : the NOL channel whose leakage matrix is required
 * pTarget_chnl_mtrx : pointer to target channel matrix returned.
 *
 * RETURN VALUE
 * BOOLEAN
 * TRUE:  leakage matrix was found
 * FALSE: leakage matrix was not found
 */
v_BOOL_t
sapFindTargetChannelInChannelMatrix(ptSapContext sapContext,
                                    ePhyChanBondState cbMode,
                                    v_U8_t NOL_channel,
                                    tSapTxLeakInfo **pTarget_chnl_mtrx)
{
    tSapTxLeakInfo *target_chan_matrix = NULL;
    tSapChanMatrixInfo *pchan_matrix = NULL;
    v_U32_t     nchan_matrix;
    int i = 0;

    switch (cbMode) {
        case PHY_SINGLE_CHANNEL_CENTERED:
            /* HT20 */
            pchan_matrix = ht20_chan;
            nchan_matrix = sizeof(ht20_chan)/sizeof(tSapChanMatrixInfo);
        break;
        case PHY_DOUBLE_CHANNEL_HIGH_PRIMARY:
        case PHY_DOUBLE_CHANNEL_LOW_PRIMARY:
            /* HT40 */
            pchan_matrix = ht40_chan;
            nchan_matrix = sizeof(ht40_chan)/sizeof(tSapChanMatrixInfo);
        break;
#ifdef WLAN_FEATURE_11AC
        case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_LOW:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_LOW:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_HIGH:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_HIGH:
            /* HT80 */
            pchan_matrix = ht80_chan;
            nchan_matrix = sizeof(ht80_chan)/sizeof(tSapChanMatrixInfo);
        break;
#endif
        default:
            /* handle exception and fall back to HT20 table */
            pchan_matrix = ht20_chan;
            nchan_matrix = sizeof(ht20_chan)/sizeof(tSapChanMatrixInfo);
        break;
    }

    for (i = 0; i < nchan_matrix; i++)
    {
        /* find the SAP channel to map the leakage matrix */
        if (NOL_channel == pchan_matrix[i].channel)
        {
            target_chan_matrix = pchan_matrix[i].chan_matrix;
            break;
        }
    }

    if (NULL == target_chan_matrix)
    {
        return VOS_FALSE;
    } else {
        *pTarget_chnl_mtrx = target_chan_matrix;
        return VOS_TRUE;
    }
}

/*
 * This function removes the channels from temp channel list that
 * (if selected as target channel) will cause leakage in one of
 * the NOL channels
 *
 * PARAMETERS
 * IN
 * sapContext        : Pointer to vos global context structure
 * cbMode            : target channel bonding mode
 * pNol              : DFS NOL
 * pTempChannelList  : the target channel list
 *
 * RETURN VALUE
 * VOS_STATUS code associated with performing the operation
 */

VOS_STATUS
sapMarkChannelsLeakingIntoNOL(ptSapContext sapContext,
                              ePhyChanBondState cbMode,
                              tSapDfsNolInfo *pNol,
                              v_U8_t tempChannelListSize,
                              v_U8_t *pTempChannelList)
{
    tSapTxLeakInfo *target_chan_matrix = NULL;
#ifdef FEATURE_WLAN_CH144
    v_U32_t         num_channel = (RF_CHAN_144 - RF_CHAN_36) + 1;
#else
    v_U32_t         num_channel = (RF_CHAN_140 - RF_CHAN_36) + 1;
#endif
    v_U32_t         i = 0;
    v_U32_t         j = 0;
    v_U32_t         k = 0;
    v_U8_t          dfs_nol_channel;
    tHalHandle      hal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    tpAniSirGlobal  mac;

    if (NULL == hal) {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
            "%s: Invalid hal pointer", __func__);
        return VOS_STATUS_E_FAULT;
    }

    mac = PMAC_STRUCT(hal);

    /* traverse target_chan_matrix and */
    for (i = 0; i < NUM_5GHZ_CHANNELS ; i++) {
        dfs_nol_channel = pNol[i].dfs_channel_number;
        if ( pNol[i].radar_status_flag == eSAP_DFS_CHANNEL_USABLE ||
             pNol[i].radar_status_flag == eSAP_DFS_CHANNEL_AVAILABLE ) {
            /* not present in NOL */
            continue;
        }
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_DEBUG,
                  FL("sapdfs: processing NOL channel: %d"),
                  dfs_nol_channel );
        if (VOS_FALSE == sapFindTargetChannelInChannelMatrix(sapContext,
                                                         cbMode,
                                                         dfs_nol_channel,
                                                         &target_chan_matrix))
        {
        /*
         * should never happen, we should always find a table here,
         * if we don't, need a fix here!
         */
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                      FL("Couldn't find target channel matrix!"));
            VOS_ASSERT(0);
            return VOS_STATUS_E_FAILURE;
        }

        /*
         * following is based on assumption that both pTempChannelList
         * and target channel matrix are in increasing order of channelID
         */
        for (j = 0, k = 0; j < tempChannelListSize &&
                           k < num_channel; ) {
            if (pTempChannelList[j] == 0) {
                j++;
            } else {
                if (target_chan_matrix[k].leak_chan != pTempChannelList[j]) {
                    k++;
                } else {
                    /* check leakage from candidate channel to NOL channel */
                    if (target_chan_matrix[k].leak_lvl <=
                         mac->sap.SapDfsInfo.tx_leakage_threshold)
                    {
                        /*
                         * this means that candidate channel will have bad
                         * leakage in NOL channel, remove the candidate channel
                         * from temp list
                         */
                        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                                  FL("sapdfs: channel: %d will have bad leakage"
                                     " due to channel: %d\n"),
                                  dfs_nol_channel,
                                  pTempChannelList[j]);
                        pTempChannelList[j] = 0;
                    }
                    j++;
                    k++;
                }
            }
        } /* end of for loop checking temp channel list leakage into NOL */
    } /* end of loop that selects each NOL */
    return VOS_STATUS_SUCCESS;
}

#endif // end of WLAN_ENABLE_CHNL_MATRIX_RESTRICTION

/*
 * This function adds availabe channel to bitmap
 *
 * PARAMETERS
 * IN
 * pBitmap: bitmap to populate
 * channel: channel to set in bitmap
 */
static void sapSetBitmap(chan_bonding_bitmap *pBitmap, v_U8_t channel)
{
    int i = 0;
    int start_channel = 0;
    for ( i = 0; i < MAX_80MHZ_BANDS; i++ ) {
        start_channel = pBitmap->chanBondingSet[i].startChannel;
        if (channel >= start_channel && channel <= start_channel + 12) {
            pBitmap->chanBondingSet[i].channelMap |=
                1 << ((channel - start_channel)/4);
            return;
        }
    }
    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
              FL("Channel=%d is not in the bitmap"), channel);
}

/*
 * This function reads the bitmap and populates available channel
 * list according to channel bonding mode. This will be called for
 * 80 MHz and 40 Mhz only. For 20 MHz no need for bitmap hence list
 * is directly created while parsing the main list
 *
 * PARAMETERS
 * IN
 * pBitmap: bitmap to populate
 * cbModeCurrent: cb mode to check for channel availability
 * availableChannels: available channel list to populate
 *
 * RETURN VALUE
 * number of channels found
 */
static v_U8_t sapPopulateAvailableChannels(chan_bonding_bitmap *pBitmap,
                                           ePhyChanBondState cbModeCurrent,
                                           v_U8_t *availableChannels)
{
    v_U8_t   i = 0;
    v_U8_t   channelCount = 0;
    v_U8_t   start_channel = 0;

    switch (cbModeCurrent) {
#ifdef WLAN_FEATURE_11AC
        /* HT80 */
        case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_LOW:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_LOW:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_HIGH:
        case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_HIGH:
            for ( i = 0; i < MAX_80MHZ_BANDS; i++ ) {
                start_channel = pBitmap->chanBondingSet[i].startChannel;
                if (pBitmap->chanBondingSet[i].channelMap == SAP_80MHZ_MASK) {
                    availableChannels[channelCount++] = start_channel;
                    availableChannels[channelCount++] = start_channel + 4;
                    availableChannels[channelCount++] = start_channel + 8;
                    availableChannels[channelCount++] = start_channel + 12;
                }
            }
        break;
#endif
        /* HT40 */
        case PHY_DOUBLE_CHANNEL_HIGH_PRIMARY:
        case PHY_DOUBLE_CHANNEL_LOW_PRIMARY:
            for ( i = 0; i < MAX_80MHZ_BANDS; i++ ) {
                start_channel = pBitmap->chanBondingSet[i].startChannel;
                if ((pBitmap->chanBondingSet[i].channelMap & SAP_40MHZ_MASK_L)
                        == SAP_40MHZ_MASK_L) {
                    availableChannels[channelCount++] = start_channel;
                    availableChannels[channelCount++] = start_channel + 4;
                } else {
                    if ((pBitmap->chanBondingSet[i].channelMap &
                         SAP_40MHZ_MASK_H) == SAP_40MHZ_MASK_H) {
                        availableChannels[channelCount++] = start_channel + 8;
                        availableChannels[channelCount++] = start_channel + 12;
                    }
               }
            }
        break;
        default:
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                      FL("Invalid case."));
        break;
    }

    return channelCount;
}

/*
 * FUNCTION  sapDfsIsW53Invalid
 *
 * DESCRIPTION Checks if the passed channel is W53 and returns if
 *             SAP W53 opearation is allowed.
 *
 * DEPENDENCIES PARAMETERS
 * IN hHAL : HAL pointer
 * channelID: Channel Number to be verified
 *
 * RETURN VALUE  : v_BOOL_t
 *                 VOS_TRUE: If W53 operation is disabled
 *                 VOS_FALSE: If W53 operation is enabled
 *
 * SIDE EFFECTS
 */
v_BOOL_t sapDfsIsW53Invalid(tHalHandle hHal, v_U8_t channelID)
{
    tpAniSirGlobal pMac;

    pMac = PMAC_STRUCT(hHal);
    if (NULL == pMac)
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pMac"));
        return VOS_FALSE;
    }

    /*
     * Check for JAPAN W53 Channel operation capability
     */
    if (VOS_TRUE ==  pMac->sap.SapDfsInfo.is_dfs_w53_disabled &&
        VOS_TRUE == IS_CHAN_JAPAN_W53(channelID))
    {
        return VOS_TRUE;
    }

    return VOS_FALSE;
}

/*
 * FUNCTION  sapDfsIsChannelInPreferredLocation
 *
 * DESCRIPTION Checks if the passed channel is in accordance with preferred
 *          Channel location settings.
 *
 * DEPENDENCIES PARAMETERS
 * IN hHAL : HAL pointer
 * channelID: Channel Number to be verified
 *
 * RETURN VALUE  :v_BOOL_t
 *        VOS_TRUE:If Channel location is same as the preferred location
 *        VOS_FALSE:If Channel location is not same as the preferred location
 *
 * SIDE EFFECTS
 */
v_BOOL_t sapDfsIsChannelInPreferredLocation(tHalHandle hHal, v_U8_t channelID)
{
    tpAniSirGlobal pMac;

    pMac = PMAC_STRUCT(hHal);
    if (NULL == pMac) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pMac"));
        return VOS_TRUE;
    }
    if ( (SAP_CHAN_PREFERRED_INDOOR ==
            pMac->sap.SapDfsInfo.sap_operating_chan_preferred_location) &&
         (VOS_TRUE == IS_CHAN_JAPAN_OUTDOOR(channelID)) )
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                   FL("CHAN=%d is Outdoor so invalid,preferred Indoor only"),
                   channelID);
        return VOS_FALSE;
    }
    else if ( (SAP_CHAN_PREFERRED_OUTDOOR ==
                 pMac->sap.SapDfsInfo.sap_operating_chan_preferred_location) &&
              (VOS_TRUE == IS_CHAN_JAPAN_INDOOR(channelID)) )
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                   FL("CHAN=%d is Indoor so invalid,preferred Outdoor only"),
                   channelID);
        return VOS_FALSE;
    }

    return VOS_TRUE;
}

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
/**
 * sap_check_in_avoid_ch_list() - checks if given channel present is channel
 * avoidance list
 * avoid_channels_info struct
 * @sap_ctx:        sap context.
 * @channel:        channel to be checked in sap_ctx's avoid ch list
 *
 * sap_ctx contains sap_avoid_ch_info strcut containing the list of channels on
 * which MDM device's AP with MCC was detected. This function checks if given
 * channel is present in that list.
 *
 * Return: true, if channel was present, false othersie.
 */
bool
sap_check_in_avoid_ch_list(ptSapContext sap_ctx, uint8_t channel)
{
	uint8_t i = 0;
	struct sap_avoid_channels_info *ie_info =
		&sap_ctx->sap_detected_avoid_ch_ie;

	for (i = 0; i < sizeof(ie_info->channels); i++) {
		if (ie_info->channels[i] == channel)
			return true;
	}
	return false;
}
#endif /* FEATURE_AP_MCC_CH_AVOIDANCE */

/**
 * sap_is_valid_acs_channel() - checks if given channel is in acs channel range
 * @sap_ctx: sap context.
 * @channel: channel to be checked in acs range
 *
 * Return: true, if channel is valid, false otherwise.
 */
static bool sap_is_valid_acs_channel(ptSapContext sap_ctx, uint8_t channel)
{
	int i = 0;

	/* Check whether acs is enabled */
	if (!sap_ctx->acs_cfg->acs_mode)
		return true;

	if ((channel < sap_ctx->acs_cfg->start_ch) ||
			(channel > sap_ctx->acs_cfg->end_ch)) {
		return false;
	}
	if (!sap_ctx->acs_cfg->ch_list) {
		/* List not present, return */
		return true;
	} else {
		for (i = 0; i < sap_ctx->acs_cfg->ch_list_count; i++)
			if (channel == sap_ctx->acs_cfg->ch_list[i])
				return true;
	}

	return false;
}

/*
 * This function randomly pick up an AVAILABLE channel
 */
static v_U8_t sapRandomChannelSel(ptSapContext sapContext)
{
    v_U32_t  random_byte = 0;
    v_U8_t   available_chnl_count = 0;
    uint8_t  avail_dfs_chan_count = 0;
    uint8_t  avail_non_dfs_chan_count = 0;
    v_U8_t   valid_chnl_count = 0;
    v_U8_t   availableChannels[WNI_CFG_VALID_CHANNEL_LIST_LEN] = {0,};
    uint8_t  avail_dfs_chan_list[WNI_CFG_VALID_CHANNEL_LIST_LEN] = {0,};
    uint8_t  avail_non_dfs_chan_list[WNI_CFG_VALID_CHANNEL_LIST_LEN] = {0,};
    v_U8_t   target_channel = 0;
    v_BOOL_t isChannelNol = VOS_FALSE;
    v_BOOL_t is_valid_acs_chan = VOS_FALSE;
    chan_bonding_bitmap channelBitmap;
    v_U8_t   i = 0;
    v_U8_t   channelID;
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    tpAniSirGlobal pMac;
    tANI_U32 chanWidth;
    ePhyChanBondState cbModeCurrent;
    v_U8_t   *tempChannels = NULL;
    uint8_t dfs_region;

    if (NULL == hHal) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid hHal"));
        return target_channel;
    }

    pMac = PMAC_STRUCT(hHal);
    if (NULL == pMac) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pMac"));
        return target_channel;
    }

    /*
     * Retrieve the original one and store it.
     * use the stored original value when you call this function next time
     * so fall back mechanism always starts with original ini value.
     */

    if (pMac->sap.SapDfsInfo.orig_cbMode == 0)
    {
        pMac->sap.SapDfsInfo.orig_cbMode = sme_SelectCBMode(hHal,
                                         sapContext->csrRoamProfile.phyMode,
                                         sapContext->channel,
                                         sapContext->secondary_ch,
                                         &sapContext->vht_channel_width,
                                         sapContext->ch_width_orig);
        pMac->sap.SapDfsInfo.orig_cbMode = csrConvertCBIniValueToPhyCBState(
                                              pMac->sap.SapDfsInfo.orig_cbMode);
        cbModeCurrent = pMac->sap.SapDfsInfo.orig_cbMode;
    }
    else
    {
        cbModeCurrent = pMac->sap.SapDfsInfo.orig_cbMode;
    }

    /*
     * Retrieve the original one and store it.
     * use the stored original value when you call this function next time
     * so fall back mechanism always starts with original ini value.
     */
    if (pMac->sap.SapDfsInfo.orig_chanWidth == 0)
    {
        pMac->sap.SapDfsInfo.orig_chanWidth =
                  sapContext->ch_width_orig;
        chanWidth = pMac->sap.SapDfsInfo.orig_chanWidth;
    }
    else
    {
        chanWidth = pMac->sap.SapDfsInfo.orig_chanWidth;
    }

    if (sapGet5GHzChannelList(sapContext))
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                  FL("Getting 5Ghz channel list failed"));
        return target_channel;
    }

    vos_nv_get_dfs_region(&dfs_region);

    /*
     * valid_chnl_count will be used to find number of valid channels
     * after following for loop ends
     */
    valid_chnl_count = sapContext->SapAllChnlList.numChannel;
    /* loop to check ACS range or NOL channels */
    for (i = 0; i < sapContext->SapAllChnlList.numChannel; i++)
    {
        channelID = sapContext->SapAllChnlList.channelList[i].channel;

        /*
         * IN JAPAN REGULATORY DOMAIN CHECK IF THE FOLLOWING TWO
         * TWO RULES APPLY AND FILTER THE AVAILABLE CHANNELS
         * ACCORDINGLY.
         *
         * 1. If we are operating in Japan regulatory domain
         * Check if Japan W53 Channel operation is NOT
         * allowed and if its not allowed then mark all the
         * W53 channels as Invalid.
         *
         * 2. If we are operating in Japan regulatory domain
         * Check if channel switch between Indoor/Outdoor
         * is allowed. If it is not allowed then limit
         * the avaiable channels to Indoor or Outdoor
         * channels only based up on the SAP Channel location
         * indicated by "sap_operating_channel_location" param.
         */
        if (DFS_MKK4_DOMAIN == dfs_region)
        {
            /*
             * Check for JAPAN W53 Channel operation capability
             */
            if (VOS_TRUE ==  sapDfsIsW53Invalid(hHal, channelID))
            {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                      FL("index:%d, Channel=%d Invalid,Japan W53 Disabled"),
                      i, channelID);
                sapContext->SapAllChnlList.channelList[i].valid = VOS_FALSE;
                valid_chnl_count--;
                continue;
            }

            /*
             * If SAP's preferred channel location is Indoor
             * then set all the outdoor channels in the domain
             * to invalid.If the preferred channel location is
             * outdoor then set all the Indoor channels in the
             * domain to Invalid.
             */
            if (VOS_FALSE ==
                        sapDfsIsChannelInPreferredLocation(hHal, channelID))
            {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                   FL("CHAN=%d is invalid,preferred Channel Location %d Only"),
                   channelID,
                   pMac->sap.SapDfsInfo.sap_operating_chan_preferred_location);
                sapContext->SapAllChnlList.channelList[i].valid = VOS_FALSE;
                valid_chnl_count--;
                continue;
            }
        }

        if (vos_nv_getChannelEnabledState(channelID) == NV_CHANNEL_DFS)
        {
            isChannelNol = sapDfsIsChannelInNolList(sapContext,
                                                   channelID,
                                                   PHY_SINGLE_CHANNEL_CENTERED);
            if (VOS_TRUE == isChannelNol)
            {
                /*
                 * Mark this channel invalid since it is still in
                 * DFS Non-Occupancy-Period which is 30 mins.
                 */
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                          FL("index: %d, Channel = %d Present in NOL"),
                          i, channelID);
                sapContext->SapAllChnlList.channelList[i].valid = VOS_FALSE;
                valid_chnl_count--;
                continue;
            }
        }

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
        /* avoid channels on which another MDM AP in MCC mode is detected. */
        if (pMac->sap.sap_channel_avoidance
                && sapContext->sap_detected_avoid_ch_ie.present) {
            if (sap_check_in_avoid_ch_list(sapContext, channelID)) {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                          FL("index: %d, Channel = %d, avoided due to "
                          "presence of another AP+AP MCC device in same "
                          "channel."),
                          i, channelID);
                sapContext->SapAllChnlList.channelList[i].valid = VOS_FALSE;
            }
        }
#endif /* FEATURE_AP_MCC_CH_AVOIDANCE */

        /* check if the channel is within ACS channel range */
        is_valid_acs_chan = sap_is_valid_acs_channel(sapContext,
                                          channelID);
        if (is_valid_acs_chan == false)
        {
            /*
             * mark this channel invalid since it is out of ACS channel range
             */
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                     FL("index: %d, Channel=%d out of ACS channel range %d-%d"),
                     i, channelID, sapContext->acs_cfg->start_ch,
                     sapContext->acs_cfg->end_ch);
            sapContext->SapAllChnlList.channelList[i].valid = VOS_FALSE;
            valid_chnl_count--;
            continue;
        }
    } /* end of check for NOL or ACS channels */

    /* valid_chnl_count now have number of valid channels */
    tempChannels = vos_mem_malloc(valid_chnl_count);
    if (tempChannels == NULL) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("sapdfs: memory alloc failed"));
        return target_channel;
    }

    do
    {
        v_U8_t   j = 0;
#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
        tSapDfsNolInfo *pNol = pMac->sap.SapDfsInfo.sapDfsChannelNolList;
#endif

        /* prepare temp list of just the valid channels */
        for (i = 0; i < sapContext->SapAllChnlList.numChannel; i++) {
            if (sapContext->SapAllChnlList.channelList[i].valid) {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                          FL("sapdfs: Adding Channel = %d to temp List"),
                          sapContext->SapAllChnlList.channelList[i].channel);
                tempChannels[j++] =
                    sapContext->SapAllChnlList.channelList[i].channel;
            }
        }

#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("sapdfs: Processing temp channel list against NOL."));
        if (VOS_STATUS_SUCCESS != sapMarkChannelsLeakingIntoNOL(sapContext,
                                                               cbModeCurrent,
                                                               pNol,
                                                               valid_chnl_count,
                                                               tempChannels)) {
            vos_mem_free(tempChannels);
            return target_channel;
        }
#endif
        vos_mem_zero(availableChannels, sizeof(availableChannels));
        vos_mem_zero(&channelBitmap, sizeof(channelBitmap));
        channelBitmap.chanBondingSet[0].startChannel = 36;
        channelBitmap.chanBondingSet[1].startChannel = 52;
        channelBitmap.chanBondingSet[2].startChannel = 100;
        channelBitmap.chanBondingSet[3].startChannel = 116;
        channelBitmap.chanBondingSet[4].startChannel = 132;
        channelBitmap.chanBondingSet[5].startChannel = 149;
        /* now loop through whatever is left of channel list */
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  FL("sapdfs: Moving temp channel list to final."));
        for (i = 0; i < valid_chnl_count; i++ ){
            /*
             * add channel from temp channel list to bitmap or fianl
             * channel list (in case of 20MHz width)
             */
            if (tempChannels[i] != 0) {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_DEBUG,
                          FL("sapdfs: processing channel: %d "),
                          tempChannels[i]);
                /* for 20MHz, directly create available channel list */
                if (cbModeCurrent == PHY_SINGLE_CHANNEL_CENTERED) {
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_DEBUG,
                              FL("sapdfs: Channel=%d added to available list"),
                              tempChannels[i]);
                    availableChannels[available_chnl_count++] =
                        tempChannels[i];
                } else {
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_DEBUG,
                              FL("sapdfs: Channel=%d added to bitmap"),
                              tempChannels[i]);
                    sapSetBitmap(&channelBitmap, tempChannels[i]);
                }
            }
        }

        /* if 40 MHz or 80 MHz, populate available channel list from bitmap */
        if (cbModeCurrent != PHY_SINGLE_CHANNEL_CENTERED) {
            available_chnl_count = sapPopulateAvailableChannels(&channelBitmap,
                                              cbModeCurrent,
                                              availableChannels);
            /* if no valid channel bonding found, fallback to lower bandwidth */
            if (available_chnl_count == 0) {
                if (cbModeCurrent >=
                     PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_CENTERED) {
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:No 80MHz cb found, falling to 40MHz"));
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:Changing chanWidth from [%d] to [%d]"),
                                 chanWidth, eHT_CHANNEL_WIDTH_40MHZ);
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:Changing CB mode from [%d] to [%d]"),
                                 cbModeCurrent, PHY_DOUBLE_CHANNEL_LOW_PRIMARY);
                    cbModeCurrent = PHY_DOUBLE_CHANNEL_LOW_PRIMARY;
                    chanWidth = eHT_CHANNEL_WIDTH_40MHZ;
                    /* continue to start of do loop */
                    continue;
                } else if (cbModeCurrent >=
                             PHY_DOUBLE_CHANNEL_LOW_PRIMARY ) {
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:No 40MHz cb found, falling to 20MHz"));
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:Changing chanWidth from [%d] to [%d]"),
                                 chanWidth, eHT_CHANNEL_WIDTH_20MHZ);
                    VOS_TRACE(VOS_MODULE_ID_SAP,
                              VOS_TRACE_LEVEL_WARN,
                              FL("sapdfs:Changing CB mode from [%d] to [%d]"),
                                 cbModeCurrent, PHY_SINGLE_CHANNEL_CENTERED);
                    cbModeCurrent = PHY_SINGLE_CHANNEL_CENTERED;
                    chanWidth = eHT_CHANNEL_WIDTH_20MHZ;
                    /* continue to start of do loop */
                    continue;
                }
            }
        }

        /*
         * by now, available channels list will be populated or
         * no channels are avaialbe
         */
        if (available_chnl_count) {
            for (i=0;i<available_chnl_count;i++) {
                if (VOS_IS_DFS_CH(availableChannels[i])) {
                    avail_dfs_chan_list[avail_dfs_chan_count++] =
                                                        availableChannels[i];
                } else {
                    avail_non_dfs_chan_list[avail_non_dfs_chan_count++] =
                                                        availableChannels[i];
                }
            }
        } else {
            VOS_TRACE(VOS_MODULE_ID_SAP,
                      VOS_TRACE_LEVEL_INFO_LOW,
                      FL("No target channel found"));
            break;
        }

        vos_rand_get_bytes(0, (v_U8_t*)&random_byte, 1);
        /* Give preference to non-DFS channel */
        if (!pMac->f_prefer_non_dfs_on_radar) {
             i = (random_byte + vos_timer_get_system_ticks()) %
                 available_chnl_count;
             target_channel = availableChannels[i];
        } else if (avail_non_dfs_chan_count) {
            i = (random_byte + vos_timer_get_system_ticks()) %
                 avail_non_dfs_chan_count;
            target_channel = avail_non_dfs_chan_list[i];
        } else {
            i = (random_byte + vos_timer_get_system_ticks()) %
                 avail_dfs_chan_count;
            target_channel = avail_dfs_chan_list[i];
        }

        pMac->sap.SapDfsInfo.new_chanWidth = chanWidth;
        pMac->sap.SapDfsInfo.new_cbMode = cbModeCurrent;
        VOS_TRACE(VOS_MODULE_ID_SAP,
                  VOS_TRACE_LEVEL_INFO_LOW,
                  FL("sapdfs: New CB mode = %d"),
                  pMac->sap.SapDfsInfo.new_cbMode);
        VOS_TRACE(VOS_MODULE_ID_SAP,
                  VOS_TRACE_LEVEL_INFO_LOW,
                  FL("sapdfs: New Channel width = %d"),
                  pMac->sap.SapDfsInfo.new_chanWidth);
        VOS_TRACE(VOS_MODULE_ID_SAP,
                  VOS_TRACE_LEVEL_INFO_LOW,
                  FL("sapdfs: target_channel = %d"), target_channel);
        break;
    } while(1); /* this loop will iterate at max 3 times */

    vos_mem_free(tempChannels);
    return target_channel;
}

/*
 * Mark the channels in NOL with time and eSAP_DFS_CHANNEL_UNAVAILABLE
 */
void sapMarkDfsChannels(ptSapContext sapContext, v_U8_t* channels,
        v_U8_t numChannels, v_U64_t time)
{
    int i, j;
    tSapDfsNolInfo *psapDfsChannelNolList = NULL;
    v_U8_t nRegDomainDfsChannels;
    tHalHandle hHal;
    tpAniSirGlobal pMac;
    v_U64_t time_elapsed_since_last_radar;
    v_U64_t time_when_radar_found;

    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);

    if (NULL == channels)
       return;

    if (NULL == hHal) {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   FL("invalid hHal"));
        return;
    }

    pMac = PMAC_STRUCT(hHal);
    if (NULL == pMac) {
         VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   FL("invalid pMac"));
        return;
    }

    /*
     * Mark the current channel on which Radar is found
     * in the NOL list as eSAP_DFS_CHANNEL_UNAVAILABLE.
     */

    psapDfsChannelNolList = pMac->sap.SapDfsInfo.sapDfsChannelNolList;
    nRegDomainDfsChannels = pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels;

    for (i = 0; i < numChannels; i++) {
        for (j = 0; j <= nRegDomainDfsChannels; j++)
        {
            if (psapDfsChannelNolList[j].dfs_channel_number ==
                    channels[i])
            {
                time_when_radar_found =
                        psapDfsChannelNolList[j].radar_found_timestamp;
                time_elapsed_since_last_radar = time -
                                         time_when_radar_found;
                /* If channel is already in NOL, don't update it again.
                 * This is useful when marking bonding channels which are
                 * already unavailable.
                 */
                if ((psapDfsChannelNolList[j].radar_status_flag ==
                        eSAP_DFS_CHANNEL_UNAVAILABLE)&&
                    (time_elapsed_since_last_radar <
                                             SAP_DFS_NON_OCCUPANCY_PERIOD))
                {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                            FL("Channel=%d already in NOL"),
                            channels[i]);
                }
                else
                {
                    /*
                     * Capture the Radar Found timestamp on the Current
                     * Channel in ms.
                     */
                    psapDfsChannelNolList[j].radar_found_timestamp = time;
                    /* Mark the Channel to be UNAVAILABLE for next 30 mins */
                    psapDfsChannelNolList[j].radar_status_flag =
                        eSAP_DFS_CHANNEL_UNAVAILABLE;

                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                            FL("Channel=%d Added to NOL LIST"),
                            channels[i]);
                }
            }
        }
    }
}


/*
 * This Function is to get bonding channels from primary channel.
 *
 */
v_U8_t sapGetBondingChannels(ptSapContext sapContext, v_U8_t channel,
        v_U8_t* channels, v_U8_t size, ePhyChanBondState chanBondState)
{
   tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
   tpAniSirGlobal pMac;
   v_U8_t numChannel;

   if(channels == NULL)
      return 0;

   if(size < MAX_BONDED_CHANNELS) return 0;

   if (NULL != hHal)
   {
      pMac = PMAC_STRUCT( hHal );
   }
   else
      return 0;

   VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
           FL("cbmode: %d, channel: %d"),
           chanBondState, channel);

   switch (chanBondState) {
      case PHY_SINGLE_CHANNEL_CENTERED:
         numChannel = 1;
         channels[0] = channel;
         break;
      case PHY_DOUBLE_CHANNEL_HIGH_PRIMARY:
         numChannel = 2;
         channels[0] = channel - 4;
         channels[1] = channel;
         break;
      case PHY_DOUBLE_CHANNEL_LOW_PRIMARY:
         numChannel = 2;
         channels[0] = channel;
         channels[1] = channel + 4;
         break;
#ifdef WLAN_FEATURE_11AC
      case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_LOW:
         numChannel = 4;
         channels[0] = channel;
         channels[1] = channel + 4;
         channels[2] = channel + 8;
         channels[3] = channel + 12;
         break;
      case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_LOW:
         numChannel = 4;
         channels[0] = channel - 4;
         channels[1] = channel;
         channels[2] = channel + 4;
         channels[3] = channel + 8;
         break;
      case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_HIGH:
         numChannel = 4;
         channels[0] = channel - 8;
         channels[1] = channel - 4;
         channels[2] = channel;
         channels[3] = channel + 4;
         break;
      case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_HIGH:
         numChannel = 4;
         channels[0] = channel - 12;
         channels[1] = channel - 8;
         channels[2] = channel - 4;
         channels[3] = channel;
         break;
#endif
      default:
         numChannel = 1;
         channels[0] = channel;
         break;
   }

   return numChannel;
}

/*
 * This Function Checks if a given bonded channel is AVAILABLE or USABLE
 * for DFS operation.
 */
v_BOOL_t
sapDfsIsChannelInNolList(ptSapContext sapContext, v_U8_t channelNumber,
        ePhyChanBondState chanBondState)
{
    int i = 0, j;
    v_U64_t timeElapsedSinceLastRadar,timeWhenRadarFound,currentTime = 0;
    v_U64_t max_jiffies;
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    tpAniSirGlobal pMac;
    v_U8_t channels[MAX_BONDED_CHANNELS];
    v_U8_t numChannels;

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return VOS_FALSE;
    }
    else
    {
        pMac = PMAC_STRUCT( hHal );
    }

    if ((pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels == 0) ||
            (pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels >
            NUM_5GHZ_CHANNELS))
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                "%s: invalid dfs channel count %d",
                __func__,
                pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels);
        return VOS_FALSE;
    }

    /* get the bonded channels */
    numChannels = sapGetBondingChannels(sapContext, channelNumber, channels,
          MAX_BONDED_CHANNELS, chanBondState );

    /* check for NOL, first on will break the loop */
    for (j=0; j < numChannels; j++)
    {
       for (i =0 ; i< pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels; i++)
       {
          if(pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                .dfs_channel_number == channels[j])
          {
             if ( (pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                      .radar_status_flag == eSAP_DFS_CHANNEL_USABLE)
                   ||
                   (pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                    .radar_status_flag == eSAP_DFS_CHANNEL_AVAILABLE) )
             {
                /*
                 * Allow SAP operation on this channel
                 * either the DFS channel has not been used
                 * for SAP operation or it is available for
                 * SAP operation since it is past Non-Occupancy-Period
                 * so, return FALSE.
                 */
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                      FL("Channel = %d not in NOL, CHANNEL AVAILABLE"),
                      pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                      .dfs_channel_number);
             }
             else if (pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                   .radar_status_flag == eSAP_DFS_CHANNEL_UNAVAILABLE)
             {
                /*
                 * If a DFS Channel is UNAVAILABLE then
                 * check to see if it is past Non-occupancy-period
                 * of 30 minutes. If it is past 30 mins then
                 * mark the channel as AVAILABLE and return FALSE
                 * as the channel is not anymore in NON-Occupancy-Period.
                 */
                timeWhenRadarFound = pMac->sap.SapDfsInfo
                   .sapDfsChannelNolList[i]
                   .radar_found_timestamp;
                currentTime = vos_get_monotonic_boottime();

                if (currentTime < timeWhenRadarFound) {
                    /* vos_get_monotonic_boottime() can overflow. Jiffies is
                     * initialized such that 32 bit jiffies value wrap 5 minutes
                     * after boot so jiffies wrap bugs show up earlier.
                     */
                     max_jiffies = (v_U64_t)UINT_MAX * 1000;
                     timeElapsedSinceLastRadar = (max_jiffies -
                                  timeWhenRadarFound) + (currentTime);
                } else {
                     timeElapsedSinceLastRadar = currentTime -
                                                       timeWhenRadarFound;
                }
                if (timeElapsedSinceLastRadar >= SAP_DFS_NON_OCCUPANCY_PERIOD)
                {
                   pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                      .radar_status_flag = eSAP_DFS_CHANNEL_AVAILABLE;
                   pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                      .radar_found_timestamp = 0;

                   VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                         FL("Channel=%d not in NOL, CHANNEL AVAILABLE"),
                         pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                         .dfs_channel_number);
                }
                else
                {
                   /*
                    * Channel is not still available for SAP operation
                    * so return TRUE; As the Channel is still
                    * in Non-occupancy-Period.
                    */
                   VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                         FL("Channel=%d still in NOL, CHANNEL UNAVAILABLE"),
                         pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                         .dfs_channel_number);
                   break;
                }
             }
          } /* if */
       } /* loop for dfs channels */

       if (i < pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels)
          break;

    } /* loop for bonded channels */

    /* if any of the channel is not available, mark all available channels as
     * unavailable with same time stamp.
     */
    if (j < numChannels &&
          i < pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels)
    {
       if (numChannels > MAX_BONDED_CHANNELS) {
           VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
                 FL( "numChannels > MAX_BONDED_CHANNELS so resetting"));
           numChannels = MAX_BONDED_CHANNELS;
       }
       sapMarkDfsChannels(sapContext,
             channels,
             numChannels,
             pMac->sap.SapDfsInfo.sapDfsChannelNolList[i]
                .radar_found_timestamp);

       /* set DFS-NOL back to keep it update-to-date in CNSS */
       sapSignalHDDevent(sapContext, NULL, eSAP_DFS_NOL_SET,
             (v_PVOID_t) eSAP_STATUS_SUCCESS);

      return VOS_TRUE;
    }

    return VOS_FALSE;
}

/**
 * sap_select_default_oper_chan_ini() - Selects operating channel based on ini
 * @hal: pointer to HAL
 * @acs_11a: 11a acs cfg
 *
 * Return: selected operating channel
 */
uint8_t sap_select_default_oper_chan_ini(tHalHandle hal, uint32_t acs_11a)
{
	uint32_t operating_band = 0;
	uint8_t channel;
	ccmCfgGetInt(hal, WNI_CFG_SAP_CHANNEL_SELECT_OPERATING_BAND,
			&operating_band);
	if (acs_11a || operating_band == RF_SUBBAND_5_LOW_GHZ ||
		operating_band == RF_SUBBAND_5_MID_GHZ ||
		operating_band == RF_SUBBAND_5_HIGH_GHZ) {
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
				FL("Default channel selection from band %d"),
				operating_band);
		if (operating_band)
			(operating_band == RF_SUBBAND_5_LOW_GHZ) ?
				(channel = SAP_DEFAULT_LOW_5GHZ_CHANNEL) :
				(operating_band == RF_SUBBAND_5_MID_GHZ) ?
				(channel = SAP_DEFAULT_MID_5GHZ_CHANNEL) :
				(operating_band == RF_SUBBAND_5_HIGH_GHZ) ?
				(channel = SAP_DEFAULT_HIGH_5GHZ_CHANNEL) : 0;
		else
			channel = SAP_DEFAULT_LOW_5GHZ_CHANNEL;

	} else {
		channel = SAP_DEFAULT_24GHZ_CHANNEL;
	}
	VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			FL("channel selected to start bss %d"), channel);
	return channel;
}

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
/**
 * sap_create_session_info() - create session info based on
 *              the input chan and phymode
 * @sap_context: ptSapContext ptr
 * @session_info: information returned.
 * @sap_ch: requesting channel number
 *
 * Return: TRUE if session info returned
 */
static v_BOOL_t sap_create_session_info(
	ptSapContext sap_context,
	session_info_t *session_info,
	v_U16_t sap_ch)
{
	tHalHandle hHal;
	eCsrPhyMode sap_phymode;

	hHal = (tHalHandle)vos_get_context( VOS_MODULE_ID_SME,
			sap_context->pvosGCtx);
	if (NULL == hHal) {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
		    "In %s, invalid hHal", __func__);
		return FALSE;
	}

	sap_phymode = sap_context->csrRoamProfile.phyMode;
	return sme_create_sap_session_info(
		hHal, sap_phymode, sap_ch, session_info);
}
/**
 * sap_find_station_session_info() - get active station session info
 * @sap_context: ptSapContext ptr
 * @session_info: information returned.
 *
 * Return: TRUE if session info returned
 */
static v_BOOL_t sap_find_station_session_info(
	ptSapContext sap_context,
	session_info_t * session_info)
{
	tHalHandle hHal;

	hHal = (tHalHandle)vos_get_context( VOS_MODULE_ID_SME,
			sap_context->pvosGCtx);
	if (NULL == hHal) {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
		    "In %s, invalid hHal", __func__);
		return FALSE;
	}
	return sme_find_sta_session_info(hHal, session_info);
}
/**
 * sap_find_all_session_info() - get all active session info
 * @sap_context: ptSapContext ptr
 * @session_info: information returned.
 * @count: number of session returned.
 *
 * Return: TRUE if any session info returned
 */
static v_BOOL_t sap_find_all_session_info(
	ptSapContext sap_context,
	session_info_t *session_info,
	v_U8_t *count)
{
	tHalHandle hHal;

	hHal = (tHalHandle)vos_get_context( VOS_MODULE_ID_SME,
			sap_context->pvosGCtx);
	if (NULL == hHal) {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
		    "In %s, invalid hHal", __func__);
		return FALSE;
	}
	return sme_find_all_session_info(hHal, session_info, count);
}
/**
 * sap_overlap_check() - check channel overlap or not
 * @sap_context: ptSapContext ptr
 * @info_1: session info 1.
 * @info_2: session info 2.
 *
 * Return: TRUE if two session channels are overlap
 */
static v_BOOL_t sap_overlap_check(
	session_info_t *info_1,
	session_info_t *info_2)
{
	v_BOOL_t intf = TRUE;
	if (!(((info_1->lfreq >= info_2->lfreq
		&& info_1->lfreq < info_2->hfreq) ||
		(info_1->hfreq > info_2->lfreq
		&& info_1->hfreq <= info_2->hfreq))
		|| ((info_2->lfreq >= info_1->lfreq
		&& info_2->lfreq < info_1->hfreq) ||
		(info_2->hfreq > info_1->lfreq
		&& info_2->hfreq <= info_1->hfreq))
		)) {
		intf = FALSE;
	}

	VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
	    "%s:info_1 mode %d band %d och %d lf %d"
	    "hf %d cf %d hbw %d",
	    __func__, info_1->con_mode, info_1->band,
	    info_1->och, info_1->lfreq, info_1->hfreq,
	    info_1->cfreq, info_1->hbw);
	VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
	    "%s:info_2 mode %d band %d och %d lf %d"
	    "hf %d cf %d hbw %d",
	    __func__, info_2->con_mode, info_2->band,
	    info_2->och, info_2->lfreq, info_2->hfreq,
	    info_2->cfreq, info_2->hbw);
	VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
	    "%s:info_1 %s with info_2",
	    __func__, intf ? "overlap" : "not overlap");

	return intf;
}
/**
 * sap_check_mcc_valid() - check mcc violation or not
 * @sap_context: ptSapContext ptr
 * @chan: chan to check
 * @band: band of the channel
 * @session_info: session info array of all active sessions
 * @session_count: session counts
 *
 * Return: VOS_STATUS_SUCCESS if the new sap chan is valid.
 */
static VOS_STATUS sap_check_mcc_valid(
	ptSapContext sap_context,
	v_SINT_t chan,
	eCsrBand band,
	session_info_t *session_info,
	v_U8_t session_count)
{
	session_info_t *info;
	session_info_t sessions[VOS_MAX_CONCURRENCY_PERSONA + 1];
	v_U8_t	i, j;
	v_U8_t session_cnt[VOS_MAX_CONCURRENCY_PERSONA + 1];
	v_U32_t channels[VOS_MAX_CONCURRENCY_PERSONA + 1];
	v_U8_t chan_cnt = 0;

	if (session_count <= 0)
		return VOS_STATUS_SUCCESS;
	else if (session_count > VOS_MAX_CONCURRENCY_PERSONA) {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
		    "%s: invalid session count %d", __func__,
		    session_count);
		return VOS_STATUS_E_INVAL;
	}
	/*
	 * create channel & session matrix
	 */
	vos_mem_copy(sessions, session_info,
		sizeof(session_info_t) * session_count);
	info = &sessions[session_count];
	info->con_mode = VOS_STA_SAP_MODE;
	info->och = chan;
	session_count++;
	for (i = 0; i < session_count &&
		chan_cnt < VOS_MAX_CONCURRENCY_PERSONA; i++) {
		info = &sessions[i];
		for (j = 0; j < chan_cnt; j++) {
			if (info->och == channels[j]) {
				session_cnt[j]++;
				break;
			}
		}
		if ((j >= chan_cnt) &&
		    (chan_cnt < (VOS_MAX_CONCURRENCY_PERSONA + 1))) {
			channels[chan_cnt] = info->och;
			session_cnt[chan_cnt] = 1;
			chan_cnt++;
		}
	}
	/*
	 * 1 .FW doesn't support > 2 home channel MCC.
	 */
	if (chan_cnt > MAX_CONCURRENCY_CHAN_COUNT) {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
		    "%s: exceed 2 home chan in MCC (chan %d band %d)",
		    __func__, chan, band);
		return VOS_STATUS_E_FAILURE;
	}
	/*
	 * 2. FW doesn't support the MCC case in which >= 3 SAP sessions
	 *  on one channel, e.g. AP1 AP2 AP3 on channel A and AP4 on
	 * channel B is not supported.
	 */
	if (chan_cnt > 1) {
		for (j = 0; j < chan_cnt; j++) {
			if (session_cnt[j] >
				    MAX_SESSSION_PER_CHAN_MCC) {
				VOS_TRACE( VOS_MODULE_ID_SAP,
					VOS_TRACE_LEVEL_ERROR,
				    "%s: exceed 2 session in MCC "
				    "(chan %d band %d)",
				    __func__, chan, band);
				return VOS_STATUS_E_FAILURE;
			}
		}
	}
	/*
	 * 3. Don't support MCC on DFS channel.
	 */
	if (chan_cnt > 0) {
		for (j = 0; j < chan_cnt; j++) {
			if (channels[j] != 0
			    && vos_nv_getChannelEnabledState(channels[j])
			    == NV_CHANNEL_DFS) {
				VOS_TRACE( VOS_MODULE_ID_SAP,
				    VOS_TRACE_LEVEL_ERROR,
				    "%s: dfs not support in MCC dfs chan %d"
				    "(chan %d band %d)",
				    __func__, channels[j], chan, band);
				return VOS_STATUS_E_FAILURE;
			}
		}
	}
	return VOS_STATUS_SUCCESS;
}

/**
* sap_concurrence_chan_override() - override SAP channel if necessary
* @sap_context: ptSapContext ptr
* @cc_switch_mode: override policy
* @con_ch: the override result channel
*
* This function will check and override sap channel based on configurated
* MCC to SCC policy :
* gWlanMccToSccSwitchMode = 0: disabled.
* gWlanMccToSccSwitchMode = 1: override to SCC if channel overlap in
*    same band.
* gWlanMccToSccSwitchMode = 2: force to SCC in same band.
*
* gWlanBandSwitchEnable = false: disabled.
* gWlanBandSwitchEnable = true:  enable band switch for MCC to SCC
*
* Return: VOS_STATUS_SUCCESS: Success
*             other value will fail the sap start request
*/
static VOS_STATUS
sap_concurrency_chan_override(
	ptSapContext sap_context,
	v_U8_t cc_switch_mode,
	tANI_U8 *con_ch)
{
	v_U8_t	i;
	v_SINT_t target_chan;
	eCsrBand target_band;
	session_info_t target_info;
	v_SINT_t candidate_chan = 0;
	v_SINT_t candidate[2 * VOS_MAX_CONCURRENCY_PERSONA + 1];
	v_U8_t candidate_count = 0;
	session_info_t session_info[VOS_MAX_CONCURRENCY_PERSONA];
	v_U8_t session_count = 0;
	VOS_STATUS status = VOS_STATUS_SUCCESS;

	if (sap_context->channel == AUTO_CHANNEL_SELECT) {
		target_band = sap_context->target_band;
		target_chan = 0;
		candidate_chan = 0;
	} else {
		if (sap_context->channel > MAX_2_4GHZ_CHANNEL) {
			target_band = eCSR_BAND_5G;
			sap_context->target_band = eCSR_BAND_5G;
		} else {
			target_band = eCSR_BAND_24;
			sap_context->target_band = eCSR_BAND_24;
		}
		target_chan = sap_context->channel;
		candidate_chan = sap_context->channel;
		if (!sap_create_session_info(sap_context, &target_info,
			target_chan)) {
			VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
			    "Failed to create channel(%d) info", target_chan);
			return VOS_STATUS_E_FAILURE;
		}
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			"%s:curr sap mode %d band %d och %d lf %d hf"
			"%d cf %d hbw %d",
			__func__, target_info.con_mode, target_info.band,
			target_info.och, target_info.lfreq, target_info.hfreq,
			target_info.cfreq, target_info.hbw);
	}

	/*
	 * 1. find all active session info
	 */
	sap_find_all_session_info(sap_context, session_info, &session_count);
	/*
	 * 2. get candidate chan list from more preference to less preference
	 */
	for (i = 0; i < session_count; i++) {
		session_info_t *info = &session_info[i];
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
		    "%s: mode %d band %d och %d lf %d hf %d cf %d hbw %d",
		    __func__, info->con_mode, info->band, info->och,
		    info->lfreq, info->hfreq, info->cfreq, info->hbw);
		if (info->band != target_band) {
			if (sap_context->band_switch_enable) {
				if (info->band == eCSR_BAND_5G) {
					sap_context->ch_width_orig =
						sap_context->ch_width_5g_orig;
				} else {
					sap_context->ch_width_orig =
						sap_context->ch_width_24g_orig;
				}
			} else {
				continue;
			}
		}
		if (cc_switch_mode == VOS_MCC_TO_SCC_SWITCH_ENABLE
			&& target_chan != 0
			&& sap_overlap_check(&target_info, info))
			candidate[candidate_count++] = info->och;
		else if (cc_switch_mode == VOS_MCC_TO_SCC_SWITCH_FORCE)
			candidate[candidate_count++] = info->och;
	}
	candidate[candidate_count++] = target_chan;
	if (cc_switch_mode == VOS_MCC_TO_SCC_SWITCH_ENABLE
		&& target_chan == 0) {
		for (i = 0; i < session_count; i++) {
			session_info_t *info = &session_info[i];
			VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
			    "%s: mode %d band %d och %d lf %d"
			    "hf %d cf %d hbw %d",
			    __func__, info->con_mode, info->band,
			    info->och, info->lfreq, info->hfreq,
			    info->cfreq, info->hbw);
			if (info->band != target_band) {
				if (sap_context->band_switch_enable) {
					if (info->band == eCSR_BAND_5G) {
						sap_context->ch_width_orig =
							sap_context->ch_width_5g_orig;
					} else {
						sap_context->ch_width_orig =
							sap_context->ch_width_24g_orig;
					}
				} else {
					continue;
				}
			}
			candidate[candidate_count++] = info->och;
		}
	}
	/*
	 * 3. check MCC violation and find the first good channel.
	 */
	for (i = 0; i < candidate_count; i++) {
		status = sap_check_mcc_valid(sap_context, candidate[i],
				target_band,
				session_info,
				session_count);
		if (status == VOS_STATUS_SUCCESS) {
			candidate_chan = candidate[i];
			VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
				   "%s: found chan %d band %d",
				   __func__, candidate_chan, target_band);
			break;
		}
	}
	if (status == VOS_STATUS_SUCCESS) {
		sap_context->channel = candidate_chan;
		*con_ch = sap_context->channel;
		if (target_chan != candidate_chan)
			VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
				"In %s, override to chan %d band %d from %d",
				__func__, candidate_chan, target_band,
				target_chan);
		else
			VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
				"In %s, no override chan %d band %d",
				__func__, target_chan, target_band);
	} else {
		VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
			"In %s, mcc violation chan %d band %d",
			__func__, target_chan, target_band);
	}
	return status;
}

/**
* sap_same_band_channel_switch_validate() -
*    check target chan valid or not during chan switch
* @sap_context: ptSapContext ptr
* @target_chan: target chan to switch
*
* This function checks whether target chan violates the same band
* SCC policy.
*
* Return: TRUE: target chan doesn't violate the policy
*/
static bool
sap_same_band_channel_switch_validate(
	ptSapContext sap_context,
	v_U16_t target_chan)
{
	session_info_t station_info;
	v_SINT_t candidate_chan = 0;
	eCsrBand target_band;

	if (target_chan == 0)
		return false;

	if (target_chan > MAX_2_4GHZ_CHANNEL)
		target_band = eCSR_BAND_5G;
	else
		target_band = eCSR_BAND_24;

	/*
	 * find and check active station session on same band
	 */
	if (sap_find_station_session_info(sap_context, &station_info)) {
		if (station_info.band == target_band)
			candidate_chan = station_info.och;
	}
	return candidate_chan == 0 || candidate_chan == target_chan;
}
/**
* sap_overlap_channel_switch_validate() -
*    check target chan valid or not during chan switch
* @sap_context: ptSapContext ptr
* @target_chan: target chan to switch
*
* This function checks whether target chan violates the same band
* overlap policy.
*
* Return: TRUE: target chan doesn't violate the policy
*/
static bool
sap_overlap_channel_switch_validate(
	ptSapContext sap_context,
	v_U16_t target_chan)
{
	session_info_t station_info, target_info;
	v_SINT_t candidate_chan = 0;

	if (target_chan == 0)
		return false;

	if (!sap_create_session_info(sap_context, &target_info,
		target_chan))
		return false;
	/*
	 * find and check active station session on same band
	 */
	if (sap_find_station_session_info(sap_context, &station_info)) {
		if (station_info.band == target_info.band)
			if (sap_overlap_check(&target_info, &station_info))
				candidate_chan = station_info.och;
	}
	return candidate_chan == 0 || candidate_chan == target_chan;
}
/**
* sap_channel_switch_validate() -
*    check target chan valid or not during chan switch
* @sap_context: ptSapContext ptr
* @target_chan: target chan to switch
*
* This function checks whether target chan violates the configurated MCC to
* SCC policy.
*
* Return: TRUE: target chan doesn't violate the policy
*/
bool
sap_channel_switch_validate(
	ptSapContext sap_context,
	tHalHandle hal,
	uint16_t target_channel,
	eCsrPhyMode sap_phy_mode,
	uint8_t cc_switch_mode,
	uint32_t session_id)
{
	if (sap_context->cc_switch_mode == VOS_MCC_TO_SCC_SWITCH_FORCE)
		return sap_same_band_channel_switch_validate(sap_context,
				target_channel);
	else if (sap_context->cc_switch_mode
			== VOS_MCC_TO_SCC_SWITCH_ENABLE)
		return sap_overlap_channel_switch_validate(sap_context,
				target_channel);
	else
		return true;
}
#endif

/*==========================================================================
  FUNCTION    sapGotoChannelSel

  DESCRIPTION
    Function for initiating scan request for SME

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    sapEvent    : State machine event
    sapDoAcsPreStartBss: VOS_TRUE, if ACS scan is issued pre start BSS.
                         VOS_FALSE, if ACS scan is issued post start BSS.

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapGotoChannelSel
(
    ptSapContext sapContext,
    ptWLAN_SAPEvent sapEvent,
    v_BOOL_t sapDoAcsPreStartBss
)
{
    /* Initiate a SCAN request */
    eHalStatus halStatus;
    tCsrScanRequest scanRequest;/* To be initialised if scan is required */
    v_U32_t    scanRequestID = 0;
    VOS_STATUS vosStatus = VOS_STATUS_SUCCESS;

#ifdef SOFTAP_CHANNEL_RANGE
    v_U8_t     *channelList = NULL;
    v_U8_t     numOfChannels = 0 ;
#endif
    tHalHandle hHal;
    tANI_U8   con_ch;

    hHal = (tHalHandle)vos_get_context( VOS_MODULE_ID_SME, sapContext->pvosGCtx);
    if (NULL == hHal)
    {
        /* we have a serious problem */
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
                   "In %s, invalid hHal", __func__);
        return VOS_STATUS_E_FAULT;
    }

    if (vos_concurrent_beaconing_sessions_running()) {
        con_ch = sme_GetConcurrentOperationChannel(hHal);
#ifdef FEATURE_WLAN_STA_AP_MODE_DFS_DISABLE
        if (con_ch && sapContext->channel == AUTO_CHANNEL_SELECT) {
            sapContext->dfs_ch_disable = VOS_TRUE;
        } else if (con_ch && sapContext->channel != con_ch &&
                 VOS_IS_DFS_CH(sapContext->channel)) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
                       "In %s, MCC DFS not supported in AP_AP Mode", __func__);
            return VOS_STATUS_E_ABORTED;
        }
#endif
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
        vosStatus = sap_concurrency_chan_override(
                      sapContext,
                      sapContext->cc_switch_mode,
                      &con_ch);
        if (vosStatus != VOS_STATUS_SUCCESS) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                "%s: invalid SAP channel(%d) configuration",
                __func__,sapContext->channel);
            return VOS_STATUS_E_ABORTED;
        }
#endif
    }

    if (vos_get_concurrency_mode() == VOS_STA_SAP)
    {
#ifdef FEATURE_WLAN_STA_AP_MODE_DFS_DISABLE
        if (sapContext->channel == AUTO_CHANNEL_SELECT)
            sapContext->dfs_ch_disable = VOS_TRUE;
        else if (VOS_IS_DFS_CH(sapContext->channel)) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
                       "In %s, DFS not supported in STA_AP Mode, chan=%d",
                       __func__, sapContext->channel);
            return VOS_STATUS_E_ABORTED;
        }
#endif
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
        vosStatus = sap_concurrency_chan_override(
                sapContext,
                sapContext->cc_switch_mode,
                &con_ch);
        if (vosStatus != VOS_STATUS_SUCCESS) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                "%s: invalid SAP channel(%d) configuration",
                __func__,sapContext->channel);
            return VOS_STATUS_E_ABORTED;
        }
#else
        /* If STA-AP concurrency is enabled take the concurrent connected
         * channel first. In other cases wpa_supplicant should take care */
        con_ch = sme_GetConcurrentOperationChannel(hHal);
        if (con_ch)
        { /*if a valid channel is returned then use concurrent channel.
                  Else take whatever comes from configuartion*/
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                FL("Channel overridden due to MCC->SCC switch: %d -> %d"),
                sapContext->channel, con_ch);
            sapContext->channel = con_ch;
            sme_SelectCBMode(hHal, sapContext->csrRoamProfile.phyMode,
                                con_ch, 0, &sapContext->vht_channel_width,
                                sapContext->ch_width_orig);
        }
#endif
    }
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    else if (sapContext->ap_p2pclient_concur_enable &&
             vos_get_concurrency_mode() == (VOS_SAP|VOS_P2P_CLIENT)) {
#ifdef FEATURE_WLAN_STA_AP_MODE_DFS_DISABLE
        if (sapContext->channel == AUTO_CHANNEL_SELECT)
            sapContext->dfs_ch_disable = VOS_TRUE;
        else if (VOS_IS_DFS_CH(sapContext->channel)) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
                       "In %s, DFS not supported in STA_AP Mode, chan=%d",
                       __func__, sapContext->channel);
            return VOS_STATUS_E_ABORTED;
        }
#endif
        vosStatus = sap_concurrency_chan_override(
                sapContext,
                sapContext->cc_switch_mode,
                &con_ch);
        if (vosStatus != VOS_STATUS_SUCCESS) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                "%s: invalid SAP channel(%d) configuration",
                __func__,sapContext->channel);
            return VOS_STATUS_E_ABORTED;
        }
    }
#endif

    if (sapContext->channel == AUTO_CHANNEL_SELECT)
    {
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                   "%s acs_cfg->skip_status = %d ", __func__,
                    sapContext->acs_cfg->skip_scan_status);
        if (sapContext->acs_cfg->skip_scan_status != eSAP_SKIP_ACS_SCAN) {
#endif
            vos_mem_zero(&scanRequest, sizeof(scanRequest));

            /* Set scanType to Active scan. FW takes care of using passive
             * scan for DFS and active for non DFS channels.
             */
            scanRequest.scanType = eSIR_ACTIVE_SCAN;

            /* Set min and max channel time to zero */
            scanRequest.minChnTime = 0;
            scanRequest.maxChnTime = 0;

            /* Set BSSType to default type */
            scanRequest.BSSType = eCSR_BSS_TYPE_ANY;

            if (ACS_FW_REPORT_PARAM_CONFIGURED)
                scanRequest.BSSType = eCSR_BSS_TYPE_INFRA_AP;

#ifndef SOFTAP_CHANNEL_RANGE
            /*Scan all the channels */
            scanRequest.ChannelInfo.numOfChannels = 0;

            scanRequest.ChannelInfo.ChannelList = NULL;

            scanRequest.requestType = eCSR_SCAN_REQUEST_FULL_SCAN;
            //eCSR_SCAN_REQUEST_11D_SCAN;

#else

            sapGetChannelList(sapContext, &channelList, &numOfChannels);
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
            if (numOfChannels != 0) {
#endif
                /*Scan the channels in the list*/
                scanRequest.ChannelInfo.numOfChannels = numOfChannels;

                scanRequest.ChannelInfo.ChannelList = channelList;

                scanRequest.requestType = eCSR_SCAN_SOFTAP_CHANNEL_RANGE;

                sapContext->channelList = channelList;
                sapContext->num_of_channel = numOfChannels;
#endif
                /* Set requestType to Full scan */

                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, calling sme_ScanRequest", __func__);
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
                if (sapContext->acs_cfg->skip_scan_status == eSAP_DO_NEW_ACS_SCAN) {
#endif
                    sme_ScanFlushResult(hHal, sapContext->sessionId);
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
                }
#endif

                if (VOS_TRUE == sapDoAcsPreStartBss)
                {
                    halStatus = sme_ScanRequest(hHal,
                                    sapContext->sessionId,
                                    &scanRequest,
                                    /* when ID == 0 11D scan/active
                                     * scan with callback,
                                     * min-maxChntime set in csrScanRequest()?
                                     */
                                    &scanRequestID,
                                    /*csrScanCompleteCallback callback,*/
                                    &WLANSAP_PreStartBssAcsScanCallback,
                                    /* pContext scanRequestID filled up*/
                                    sapContext);

                }
                else
                {
                    halStatus = sme_ScanRequest(hHal,
                                    sapContext->sessionId,
                                    &scanRequest,
                                    /* when ID == 0 11D scan/active
                                     * scan with callback,
                                     * min-maxChntime set in csrScanRequest()?
                                     */
                                    &scanRequestID,
                                    /*csrScanCompleteCallback callback,*/
                                    &WLANSAP_ScanCallback,
                                    /* pContext scanRequestID filled up*/
                                    sapContext);
                }
                if (eHAL_STATUS_SUCCESS != halStatus)
                {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                        "%s:sme_ScanRequest  fail %d!!!", __func__, halStatus);
                    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                        "SoftAP Configuring for default channel, Ch= %d",
                        sapContext->channel);

                        sapContext->channel =
                                     sap_select_default_oper_chan_ini(hHal, 0);
#ifdef SOFTAP_CHANNEL_RANGE
                    if(sapContext->channelList != NULL)
                    {
                        sapContext->channel = sapContext->channelList[0];
                        vos_mem_free(sapContext->channelList);
                        sapContext->channelList = NULL;
                        sapContext->num_of_channel = 0;
                    }
#endif
                    if (VOS_TRUE == sapDoAcsPreStartBss)
                    {
                        /*
                         * In case of ACS req before start Bss,
                         * return failure so that the calling
                         * fucntion can use the default channel.
                         */
                        return VOS_STATUS_E_FAILURE;
                    }
                    else
                    {
                        /* Fill in the event structure */
                        sapEventInit(sapEvent);
                        /* Handle event */
                        vosStatus = sapFsm(sapContext, sapEvent);
                    }
                }
                else
                {
                    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                         "In %s, return from sme_ScanReq, scanID=%d, Ch= %d",
                        __func__, scanRequestID, sapContext->channel);
                }
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
            } else
               sapContext->acs_cfg->skip_scan_status = eSAP_SKIP_ACS_SCAN;
        }

        if (sapContext->acs_cfg->skip_scan_status == eSAP_SKIP_ACS_SCAN) {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "## %s SKIPPED ACS SCAN", __func__);
            if (VOS_TRUE == sapDoAcsPreStartBss)
            {
                WLANSAP_PreStartBssAcsScanCallback(hHal, sapContext,
                                                   sapContext->sessionId,
                                                   0,
                                                   eCSR_SCAN_SUCCESS);
            }
            else
            {
                WLANSAP_ScanCallback(hHal, sapContext,
                                     sapContext->sessionId,
                                     0,
                                     eCSR_SCAN_SUCCESS);
            }
         }
#endif
    }
    else
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  "In %s, for configured channel, Ch= %d",
                  __func__, sapContext->channel);
        if (VOS_TRUE == sapDoAcsPreStartBss)
        {
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                      "In %s, ACS end due to channel override, Selected Ch= %d",
                       __func__, sapContext->channel);

            sapContext->acs_cfg->pri_ch = sapContext->channel;
            sap_config_acs_result(hHal, sapContext, 0);
            return VOS_STATUS_E_CANCELED;
        }
        else
        {
            /* Fill in the event structure */
            // Eventhough scan was not done, means a user set channel was chosen
            sapEventInit(sapEvent);
            /* Handle event */
            vosStatus = sapFsm(sapContext, sapEvent);
        }
    }

    /* If scan failed, get default channel and advance state machine as success with default channel */
    /* Have to wait for the call back to be called to get the channel cannot advance state machine here as said above */
    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, before exiting sapGotoChannelSel channel=%d", __func__, sapContext->channel);

    return VOS_STATUS_SUCCESS;
}// sapGotoChannelSel
#define SAP_OPEN_SESSION_TIMEOUT 500

/**
 * sap_OpenSession() - Opens a SAP session
 * @hHal: Hal handle
 * @sapContext:  Sap Context value
 * @session_id: Pointer to the session id
 *
 * Function for opening SME and SAP sessions when system is in SoftAP role
 *
 * Return: eHalStatus
 */

eHalStatus
sap_OpenSession (tHalHandle hHal, ptSapContext sapContext,
                 uint32_t *session_id)
{
    tANI_U32 type, subType;
    eHalStatus halStatus;
    VOS_STATUS status = VOS_STATUS_E_FAILURE;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    if (sapContext->csrRoamProfile.csrPersona == VOS_P2P_GO_MODE)
        status = vos_get_vdev_types(VOS_P2P_GO_MODE, &type, &subType);
    else
        status = vos_get_vdev_types(VOS_STA_SAP_MODE, &type, &subType);

    if (VOS_STATUS_SUCCESS != status)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL, "failed to get vdev type");
        return VOS_STATUS_E_FAILURE;
    }
    vos_event_reset(&sapContext->sap_session_opened_evt);
    /* Open SME Session for Softap */
    halStatus = sme_OpenSession(hHal,
                                &WLANSAP_RoamCallback,
                                sapContext,
                                sapContext->self_mac_addr,
                                &sapContext->sessionId,
                                type, subType);

    if(eHAL_STATUS_SUCCESS != halStatus )
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  "Error: In %s calling sme_RoamConnect status = %d",
                  __func__, halStatus);

        return VOS_STATUS_E_FAILURE;
    }

    sme_set_allowed_action_frames(hHal,
		ALLOWED_ACTION_FRAMES_BITMAP0_SAP, false);
    status = vos_wait_single_event(
            &sapContext->sap_session_opened_evt,
            SAP_OPEN_SESSION_TIMEOUT);
    if (!VOS_IS_STATUS_SUCCESS(status)) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  "wait for sap open session event timed out");
        sme_CloseSession(hHal, sapContext->sessionId, NULL, FALSE);
        return VOS_STATUS_E_FAILURE;
    }

    pMac->sap.sapCtxList [ sapContext->sessionId ].sessionID =
                               sapContext->sessionId;
    pMac->sap.sapCtxList [ sapContext->sessionId ].pSapContext = sapContext;
    pMac->sap.sapCtxList [ sapContext->sessionId ].sapPersona=
                               sapContext->csrRoamProfile.csrPersona;
    *session_id = sapContext->sessionId;
    sapContext->isSapSessionOpen = eSAP_TRUE;
    return VOS_STATUS_SUCCESS;
}


/*==========================================================================
  FUNCTION    sapGotoStarting

  DESCRIPTION
    Function for initiating start bss request for SME

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    sapEvent    : State machine event
    bssType     : Type of bss to start, INRA AP
    status      : Return the SAP status here

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapGotoStarting
(
    ptSapContext sapContext,
    ptWLAN_SAPEvent sapEvent,
    eCsrRoamBssType bssType
)
{
    /* tHalHandle */
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    eHalStatus halStatus;

    /*- - - - - - - - TODO:once configs from hdd available - - - - - - - - -*/
    char key_material[32]={ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1,};
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
    sapContext->key_type = 0x05;
    sapContext->key_length = 32;
    vos_mem_copy(sapContext->key_material, key_material, sizeof(key_material));  /* Need a key size define */

    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s", __func__);

    if (NULL == hHal)
    {
        /* we have a serious problem */
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
                   "In %s, invalid hHal", __func__);
        return VOS_STATUS_E_FAULT;
    }

    /* No Need to Req for Power with power offload enabled */
    if(!pMac->psOffloadEnabled)
    {
       //TODO: What shall we do if failure????
       halStatus = pmcRequestFullPower( hHal,
                            WLANSAP_pmcFullPwrReqCB,
                            sapContext,
                            eSME_REASON_OTHER);
    }

    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
            "%s: session: %d", __func__, sapContext->sessionId);

    halStatus = sme_RoamConnect(hHal, sapContext->sessionId,
            &sapContext->csrRoamProfile,
            &sapContext->csrRoamId);
    if (eHAL_STATUS_SUCCESS != halStatus)
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
        "%s: Failed to issue sme_RoamConnect", __func__);
        return halStatus;

}// sapGotoStarting

/*==========================================================================
  FUNCTION    sapGotoDisconnecting

  DESCRIPTION
    Processing of SAP FSM Disconnecting state

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    status      : Return the SAP status here

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapGotoDisconnecting
(
    ptSapContext sapContext
)
{
    eHalStatus halStatus;
    tHalHandle hHal;

    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    if (NULL == hHal)
    {
        /* we have a serious problem */
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s, invalid hHal", __func__);
        return VOS_STATUS_E_FAULT;
    }

    sapFreeRoamProfile(&sapContext->csrRoamProfile);
    halStatus = sme_RoamStopBss(hHal, sapContext->sessionId);
    if(eHAL_STATUS_SUCCESS != halStatus )
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "Error: In %s calling sme_RoamStopBss status = %d", __func__, halStatus);
        return VOS_STATUS_E_FAILURE;
    }

    return VOS_STATUS_SUCCESS;
}

eHalStatus sapRoamSessionCloseCallback(void *pContext)
{
    ptSapContext sapContext = (ptSapContext)pContext;
    return sapSignalHDDevent(sapContext, NULL,
                    eSAP_STOP_BSS_EVENT, (v_PVOID_t) eSAP_STATUS_SUCCESS);
}

/*==========================================================================
  FUNCTION    sapGotoDisconnected

  DESCRIPTION
    Function for setting the SAP FSM to Disconnection state

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    sapEvent    : State machine event
    status      : Return the SAP status here

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapGotoDisconnected
(
    ptSapContext sapContext
)
{
    VOS_STATUS vosStatus = VOS_STATUS_E_FAILURE;
    tWLAN_SAPEvent sapEvent;
    // Processing has to be coded
    // Clean up stations from TL etc as AP BSS is shut down then set event
    sapEvent.event = eSAP_MAC_READY_FOR_CONNECTIONS;// hardcoded
    sapEvent.params = 0;
    sapEvent.u1 = 0;
    sapEvent.u2 = 0;
    /* Handle event */
    vosStatus = sapFsm(sapContext, &sapEvent);

    return vosStatus;
}

#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
/**
 * sap_handle_acs_scan_event() - handle acs scan event for SAP
 * @sap_context: ptSapContext
 * @sap_event: tSap_Event
 * @status: status of acs scan
 *
 * The function is to handle the eSAP_ACS_SCAN_SUCCESS_EVENT event.
 *
 * Return: void
 */
static void sap_handle_acs_scan_event(ptSapContext sap_context,
		tSap_Event *sap_event, eSapStatus status)
{
	sap_event->sapHddEventCode = eSAP_ACS_SCAN_SUCCESS_EVENT;
	sap_event->sapevt.sap_acs_scan_comp.status = status;
	sap_event->sapevt.sap_acs_scan_comp.num_of_channels =
			sap_context->num_of_channel;
	sap_event->sapevt.sap_acs_scan_comp.channellist =
			sap_context->channelList;
}
#else
static void sap_handle_acs_scan_event(ptSapContext sap_context,
		tSap_Event *sap_event, eSapStatus status)
{
}
#endif
/*==========================================================================
  FUNCTION    sapSignalHDDevent

  DESCRIPTION
    Function for HDD to send the event notification using callback

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    pCsrRoamInfo : Pointer to CSR roam information
    sapHddevent      : SAP HDD event
    context          : to pass the element for future support

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapSignalHDDevent
(
    ptSapContext sapContext, /* sapContext value */
    tCsrRoamInfo *pCsrRoamInfo,
    eSapHddEvent sapHddevent,
    void         *context
)
{
    VOS_STATUS  vosStatus = VOS_STATUS_SUCCESS;
    tSap_Event sapApAppEvent; /* This now encodes ALL event types */
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    tpAniSirGlobal pMac;
    /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

    /* Format the Start BSS Complete event to return... */
    if (NULL == sapContext->pfnSapEventCallback)
    {
        return VOS_STATUS_E_FAILURE;
    }
    if (NULL == hHal)
    {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                  "In %s invalid hHal", __func__);
        return VOS_STATUS_E_FAILURE;
    }
    pMac = PMAC_STRUCT( hHal );

    switch (sapHddevent)
    {
        case eSAP_STA_ASSOC_IND:
            //  TODO - Indicate the assoc request indication to OS
            sapApAppEvent.sapHddEventCode = eSAP_STA_ASSOC_IND;

            vos_mem_copy( &sapApAppEvent.sapevt.sapAssocIndication.staMac, pCsrRoamInfo->peerMac,sizeof(tSirMacAddr));
            sapApAppEvent.sapevt.sapAssocIndication.staId = pCsrRoamInfo->staId;
            sapApAppEvent.sapevt.sapAssocIndication.status = 0;
            // Required for indicating the frames to upper layer
            sapApAppEvent.sapevt.sapAssocIndication.beaconLength = pCsrRoamInfo->beaconLength;
            sapApAppEvent.sapevt.sapAssocIndication.beaconPtr = pCsrRoamInfo->beaconPtr;
            sapApAppEvent.sapevt.sapAssocIndication.assocReqLength = pCsrRoamInfo->assocReqLength;
            sapApAppEvent.sapevt.sapAssocIndication.assocReqPtr = pCsrRoamInfo->assocReqPtr;
            sapApAppEvent.sapevt.sapAssocIndication.fWmmEnabled = pCsrRoamInfo->wmmEnabledSta;
            sapApAppEvent.sapevt.sapAssocIndication.ecsa_capable =
                pCsrRoamInfo->ecsa_capable;

            if ( pCsrRoamInfo->u.pConnectedProfile != NULL )
            {
               sapApAppEvent.sapevt.sapAssocIndication.negotiatedAuthType = pCsrRoamInfo->u.pConnectedProfile->AuthType;
               sapApAppEvent.sapevt.sapAssocIndication.negotiatedUCEncryptionType = pCsrRoamInfo->u.pConnectedProfile->EncryptionType;
               sapApAppEvent.sapevt.sapAssocIndication.negotiatedMCEncryptionType = pCsrRoamInfo->u.pConnectedProfile->mcEncryptionType;
               sapApAppEvent.sapevt.sapAssocIndication.fAuthRequired = pCsrRoamInfo->fAuthRequired;
            }
            break;
       case eSAP_START_BSS_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                       "eSAP_START_BSS_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_START_BSS_EVENT;
            sapApAppEvent.sapevt.sapStartBssCompleteEvent.status = (eSapStatus )context;
            if(pCsrRoamInfo != NULL ){
                sapApAppEvent.sapevt.sapStartBssCompleteEvent.staId = pCsrRoamInfo->staId;
            }
            else
            {
                sapApAppEvent.sapevt.sapStartBssCompleteEvent.staId =
                    tl_shim_get_sta_id_by_addr(sapContext->pvosGCtx,
                        sapContext->self_mac_addr);
            }

            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "%s(eSAP_START_BSS_EVENT): staId = %d",
                __func__, sapApAppEvent.sapevt.sapStartBssCompleteEvent.staId);

            sapApAppEvent.sapevt.sapStartBssCompleteEvent.operatingChannel = (v_U8_t)sapContext->channel;
            sapApAppEvent.sapevt.sapStartBssCompleteEvent.sessionId =
                    sapContext->sessionId;
            break;

        case eSAP_DFS_CAC_START:
        case eSAP_DFS_CAC_INTERRUPTED:
        case eSAP_DFS_CAC_END:
        case eSAP_DFS_RADAR_DETECT:
        case eSAP_DFS_NO_AVAILABLE_CHANNEL:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                "In %s, SAP event callback event = %s : %d", __func__,
                "eSAP_DFS event", sapHddevent);
            sapApAppEvent.sapHddEventCode = sapHddevent;
            sapApAppEvent.sapevt.sapStopBssCompleteEvent.status =
                                           (eSapStatus )context;
            break;
        case eSAP_ACS_SCAN_SUCCESS_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                "In %s, SAP event callback event = %s : %d", __func__,
                "ACS Scan event", sapHddevent);
            sap_handle_acs_scan_event(sapContext, &sapApAppEvent,
                        (eSapStatus)context);
            break;
        case eSAP_ACS_CHANNEL_SELECTED:
            sapApAppEvent.sapHddEventCode = sapHddevent;
            if ( eSAP_STATUS_SUCCESS == (eSapStatus )context)
            {
                sapApAppEvent.sapevt.sapChSelected.pri_ch =
                                                      sapContext->acs_cfg->pri_ch;
                sapApAppEvent.sapevt.sapChSelected.ht_sec_ch =
                                                      sapContext->acs_cfg->ht_sec_ch;
                sapApAppEvent.sapevt.sapChSelected.ch_width =
                                                      sapContext->acs_cfg->ch_width;
                sapApAppEvent.sapevt.sapChSelected.vht_seg0_center_ch =
                                             sapContext->acs_cfg->vht_seg0_center_ch;
                sapApAppEvent.sapevt.sapChSelected.vht_seg1_center_ch =
                                             sapContext->acs_cfg->vht_seg1_center_ch;
            }
            else if (eSAP_STATUS_FAILURE == (eSapStatus )context)
            {
                sapApAppEvent.sapevt.sapChSelected.pri_ch = 0;
            }
            break;

        case eSAP_STOP_BSS_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STOP_BSS_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_STOP_BSS_EVENT;
            sapApAppEvent.sapevt.sapStopBssCompleteEvent.status = (eSapStatus )context;
            break;

        case eSAP_STA_ASSOC_EVENT:
        case eSAP_STA_REASSOC_EVENT:
        {
            tSirSmeChanInfo *pChanInfo;
            tSap_StationAssocReassocCompleteEvent *sta_event_ptr;

            sta_event_ptr =
                &sapApAppEvent.sapevt.sapStationAssocReassocCompleteEvent;
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STA_ASSOC_EVENT");
            if (pCsrRoamInfo->fReassocReq)
                sapApAppEvent.sapHddEventCode = eSAP_STA_REASSOC_EVENT;
            else
                sapApAppEvent.sapHddEventCode = eSAP_STA_ASSOC_EVENT;

            //TODO: Need to fill the SET KEY information and pass to HDD
            vos_mem_copy(&sta_event_ptr->staMac,
                         pCsrRoamInfo->peerMac,sizeof(tSirMacAddr));
            sta_event_ptr->staId = pCsrRoamInfo->staId;
            sta_event_ptr->statusCode = pCsrRoamInfo->statusCode;
            sta_event_ptr->iesLen = pCsrRoamInfo->rsnIELen;
            vos_mem_copy(sta_event_ptr->ies, pCsrRoamInfo->prsnIE,
                         pCsrRoamInfo->rsnIELen);
            sta_event_ptr->ampdu = pCsrRoamInfo->ampdu;
            sta_event_ptr->sgi_enable = pCsrRoamInfo->sgi_enable;
            sta_event_ptr->tx_stbc = pCsrRoamInfo->tx_stbc;
            sta_event_ptr->rx_stbc = pCsrRoamInfo->rx_stbc;
            sta_event_ptr->ch_width = pCsrRoamInfo->ch_width;
            sta_event_ptr->mode = pCsrRoamInfo->mode;
            sta_event_ptr->max_supp_idx = pCsrRoamInfo->max_supp_idx;
            sta_event_ptr->max_ext_idx = pCsrRoamInfo->max_ext_idx;
            sta_event_ptr->max_mcs_idx = pCsrRoamInfo->max_mcs_idx;
            sta_event_ptr->rx_mcs_map = pCsrRoamInfo->rx_mcs_map;
            sta_event_ptr->tx_mcs_map = pCsrRoamInfo->tx_mcs_map;

#ifdef FEATURE_WLAN_WAPI
            if(pCsrRoamInfo->wapiIELen)
            {
                v_U8_t  len = sta_event_ptr->iesLen;
                sta_event_ptr->iesLen
                                                        += pCsrRoamInfo->wapiIELen;
                vos_mem_copy(&sta_event_ptr->ies[len],
                        pCsrRoamInfo->pwapiIE,
                            pCsrRoamInfo->wapiIELen);
            }
#endif

            if(pCsrRoamInfo->addIELen)
            {
                v_U8_t  len = sta_event_ptr->iesLen;
                sta_event_ptr->iesLen
                                                        += pCsrRoamInfo->addIELen;
                vos_mem_copy(&sta_event_ptr->ies[len], pCsrRoamInfo->paddIE,
                            pCsrRoamInfo->addIELen);
            }

            /* also fill up the channel info from the csrRoamInfo */
            pChanInfo =
            &sta_event_ptr->chan_info;

            pChanInfo->chan_id = pCsrRoamInfo->chan_info.chan_id;
            pChanInfo->mhz = pCsrRoamInfo->chan_info.mhz;
            pChanInfo->info = pCsrRoamInfo->chan_info.info;
            pChanInfo->band_center_freq1 = pCsrRoamInfo->chan_info.band_center_freq1;
            pChanInfo->band_center_freq2 = pCsrRoamInfo->chan_info.band_center_freq2;
            pChanInfo->reg_info_1 = pCsrRoamInfo->chan_info.reg_info_1;
            pChanInfo->reg_info_2 = pCsrRoamInfo->chan_info.reg_info_2;
            pChanInfo->nss = pCsrRoamInfo->chan_info.nss;
            pChanInfo->rate_flags = pCsrRoamInfo->chan_info.rate_flags;
            pChanInfo->sub20_channelwidth =
                                     pCsrRoamInfo->chan_info.sub20_channelwidth;
            sta_event_ptr->wmmEnabled = pCsrRoamInfo->wmmEnabledSta;
            sta_event_ptr->status = (eSapStatus )context;
            sta_event_ptr->timingMeasCap = pCsrRoamInfo->timingMeasCap;
            sta_event_ptr->ecsa_capable = pCsrRoamInfo->ecsa_capable;
            //TODO: Need to fill sapAuthType
            //sapApAppEvent.sapevt.sapStationAssocReassocCompleteEvent.SapAuthType = pCsrRoamInfo->pProfile->negotiatedAuthType;
            break;
        }

        case eSAP_STA_DISASSOC_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STA_DISASSOC_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_STA_DISASSOC_EVENT;

            vos_mem_copy( &sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.staMac,
                          pCsrRoamInfo->peerMac, sizeof(tSirMacAddr));
            sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.staId = pCsrRoamInfo->staId;
            if (pCsrRoamInfo->reasonCode == eCSR_ROAM_RESULT_FORCED)
                sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.reason = eSAP_USR_INITATED_DISASSOC;
            else
                sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.reason = eSAP_MAC_INITATED_DISASSOC;

            sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.statusCode = pCsrRoamInfo->statusCode;
            sapApAppEvent.sapevt.sapStationDisassocCompleteEvent.status = (eSapStatus )context;
            break;

        case eSAP_STA_SET_KEY_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STA_SET_KEY_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_STA_SET_KEY_EVENT;
            sapApAppEvent.sapevt.sapStationSetKeyCompleteEvent.status = (eSapStatus )context;
            vos_mem_copy(&sapApAppEvent.sapevt.sapStationSetKeyCompleteEvent.peerMacAddr,
                         pCsrRoamInfo->peerMac,sizeof(tSirMacAddr));
            break;

        case eSAP_STA_DEL_KEY_EVENT :
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STA_DEL_KEY_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_STA_DEL_KEY_EVENT;
            sapApAppEvent.sapevt.sapStationDeleteKeyCompleteEvent.status = (eSapStatus )context;
            //TODO: Should we need to send the key information
            //sapApAppEvent.sapevt.sapStationDeleteKeyCompleteEvent.keyId = ;
            break;

        case eSAP_STA_MIC_FAILURE_EVENT :
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_STA_MIC_FAILURE_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_STA_MIC_FAILURE_EVENT;
            vos_mem_copy( &sapApAppEvent.sapevt.sapStationMICFailureEvent.srcMacAddr,
                          pCsrRoamInfo->u.pMICFailureInfo->srcMacAddr,
                          sizeof(tSirMacAddr));
            vos_mem_copy( &sapApAppEvent.sapevt.sapStationMICFailureEvent.staMac,
                          pCsrRoamInfo->u.pMICFailureInfo->taMacAddr,
                          sizeof(tSirMacAddr));
            vos_mem_copy( &sapApAppEvent.sapevt.sapStationMICFailureEvent.dstMacAddr,
                          pCsrRoamInfo->u.pMICFailureInfo->dstMacAddr,
                          sizeof(tSirMacAddr));
            sapApAppEvent.sapevt.sapStationMICFailureEvent.multicast = pCsrRoamInfo->u.pMICFailureInfo->multicast;
            sapApAppEvent.sapevt.sapStationMICFailureEvent.IV1 = pCsrRoamInfo->u.pMICFailureInfo->IV1;
            sapApAppEvent.sapevt.sapStationMICFailureEvent.keyId = pCsrRoamInfo->u.pMICFailureInfo->keyId;
            vos_mem_copy( sapApAppEvent.sapevt.sapStationMICFailureEvent.TSC,
                          pCsrRoamInfo->u.pMICFailureInfo->TSC,
                          SIR_CIPHER_SEQ_CTR_SIZE);
            break;

        case eSAP_ASSOC_STA_CALLBACK_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_ASSOC_STA_CALLBACK_EVENT");
            break;

        case eSAP_WPS_PBC_PROBE_REQ_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_WPS_PBC_PROBE_REQ_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_WPS_PBC_PROBE_REQ_EVENT;

            vos_mem_copy( &sapApAppEvent.sapevt.sapPBCProbeReqEvent.WPSPBCProbeReq,
                          pCsrRoamInfo->u.pWPSPBCProbeReq,
                          sizeof(tSirWPSPBCProbeReq));
            break;
       case eSAP_REMAIN_CHAN_READY:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_REMAIN_CHAN_READY");
           sapApAppEvent.sapHddEventCode = eSAP_REMAIN_CHAN_READY;
            break;

       case eSAP_DISCONNECT_ALL_P2P_CLIENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_DISCONNECT_ALL_P2P_CLIENT");
            sapApAppEvent.sapHddEventCode = eSAP_DISCONNECT_ALL_P2P_CLIENT;
            sapApAppEvent.sapevt.sapActionCnf.actionSendSuccess = (eSapStatus)context;
            break;

       case eSAP_MAC_TRIG_STOP_BSS_EVENT :
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_MAC_TRIG_STOP_BSS_EVENT");
            sapApAppEvent.sapHddEventCode = eSAP_MAC_TRIG_STOP_BSS_EVENT;
            sapApAppEvent.sapevt.sapActionCnf.actionSendSuccess = (eSapStatus)context;
            break;


        case eSAP_UNKNOWN_STA_JOIN:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_UNKNOWN_STA_JOIN");
            sapApAppEvent.sapHddEventCode = eSAP_UNKNOWN_STA_JOIN;
            vos_mem_copy((v_PVOID_t)sapApAppEvent.sapevt.sapUnknownSTAJoin.macaddr.bytes,
                         (v_PVOID_t)context, sizeof(v_MACADDR_t));
            break;

        case eSAP_MAX_ASSOC_EXCEEDED:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                       FL("SAP event callback event = %s"),
                          "eSAP_MAX_ASSOC_EXCEEDED");
            sapApAppEvent.sapHddEventCode = eSAP_MAX_ASSOC_EXCEEDED;
            vos_mem_copy((v_PVOID_t)sapApAppEvent.sapevt.sapMaxAssocExceeded.macaddr.bytes,
                    (v_PVOID_t)pCsrRoamInfo->peerMac, sizeof(v_MACADDR_t));
            break;

        case eSAP_CHANNEL_CHANGE_EVENT:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, SAP event callback event = %s",
                    __func__, "eSAP_CHANNEL_CHANGE_EVENT");
            /* Reconfig ACS result info. For DFS AP-AP Mode Sec AP ACS
             * follows pri AP
             */
            sapContext->acs_cfg->pri_ch = sapContext->channel;
            sapContext->acs_cfg->ch_width = sapContext->vht_channel_width;
            sap_config_acs_result(hHal, sapContext, sapContext->secondary_ch);

            sapApAppEvent.sapHddEventCode = eSAP_CHANNEL_CHANGE_EVENT;
            sapApAppEvent.sapevt.sapChSelected.pri_ch =
                                                sapContext->acs_cfg->pri_ch;
            sapApAppEvent.sapevt.sapChSelected.ht_sec_ch =
                                            sapContext->acs_cfg->ht_sec_ch;
            sapApAppEvent.sapevt.sapChSelected.ch_width =
                                            sapContext->acs_cfg->ch_width;
            sapApAppEvent.sapevt.sapChSelected.vht_seg0_center_ch =
                                        sapContext->acs_cfg->vht_seg0_center_ch;
            sapApAppEvent.sapevt.sapChSelected.vht_seg1_center_ch =
                                        sapContext->acs_cfg->vht_seg1_center_ch;
            break;

        case eSAP_DFS_NOL_GET:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, SAP event callback event = %s",
                    __func__, "eSAP_DFS_NOL_GET");
            sapApAppEvent.sapHddEventCode = eSAP_DFS_NOL_GET;
            sapApAppEvent.sapevt.sapDfsNolInfo.sDfsList =
                NUM_5GHZ_CHANNELS * sizeof(tSapDfsNolInfo);
            sapApAppEvent.sapevt.sapDfsNolInfo.pDfsList =
                (v_PVOID_t)(&pMac->sap.SapDfsInfo.sapDfsChannelNolList[0]);
            break;

        case eSAP_DFS_NOL_SET:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, SAP event callback event = %s",
                    __func__, "eSAP_DFS_NOL_SET");
            sapApAppEvent.sapHddEventCode = eSAP_DFS_NOL_SET;
            sapApAppEvent.sapevt.sapDfsNolInfo.sDfsList =
                pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels *
                    sizeof(tSapDfsNolInfo);
            sapApAppEvent.sapevt.sapDfsNolInfo.pDfsList =
                (v_PVOID_t)(&pMac->sap.SapDfsInfo.sapDfsChannelNolList[0]);
            break;
        case eSAP_ECSA_CHANGE_CHAN_IND:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, SAP event callback event = %s",
                    __func__, "eSAP_ECSA_CHANGE_CHAN_IND");
            sapApAppEvent.sapHddEventCode = eSAP_ECSA_CHANGE_CHAN_IND;
            sapApAppEvent.sapevt.sap_chan_cng_ind.new_chan =
                                           pCsrRoamInfo->target_channel;
            break;
        default:
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                       FL("SAP Unknown callback event = %d"),
                     sapHddevent);
            break;
    }
    vosStatus = (*sapContext->pfnSapEventCallback)
                (
                 &sapApAppEvent,
                 sapContext->pUsrContext//userdataforcallback - hdd opaque handle
                 );

    return vosStatus;

} /* sapSignalApAppStartBssEvent */

/*==========================================================================
  FUNCTION  sap_find_valid_concurrent_session

  DESCRIPTION
    This function will return sapcontext of any valid sap session.

  PARAMETERS

    IN
    hHal        : HAL pointer

  RETURN VALUE
    ptSapContext : valid sap context

  SIDE EFFECTS
    NA
============================================================================*/
ptSapContext sap_find_valid_concurrent_session (tHalHandle hHal)
{
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
    v_U8_t intf = 0;

    for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
    {
         if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
             (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
             pMac->sap.sapCtxList[intf].pSapContext != NULL)
         {
             return pMac->sap.sapCtxList[intf].pSapContext;
         }
    }

    return NULL;
}

/*==========================================================================
  FUNCTION   sap_CloseSession

  DESCRIPTION
    This function will close all the sme sessions as well as zero-out the
    sap global structure

  PARAMETERS

    IN
    hHal        : HAL pointer
    sapContext  : Sap Context value
    callback    : Roam Session close callback
    valid       : Sap context is valid or no

  RETURN VALUE
    The eHalStatus code associated with performing the operation
    eHAL_STATUS_SUCCESS: Success

  SIDE EFFECTS
    NA
============================================================================*/
eHalStatus sap_CloseSession(tHalHandle hHal,
                            ptSapContext sapContext,
                            csrRoamSessionCloseCallback callback,
                            v_BOOL_t valid)
{
    eHalStatus halstatus;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    if (FALSE == valid)
    {
        halstatus = sme_CloseSession(hHal,
                                     sapContext->sessionId,
                                     callback, NULL);
    }
    else
    {
        halstatus = sme_CloseSession(hHal,
                                     sapContext->sessionId,
                                     callback, sapContext);
    }

    sapContext->isCacStartNotified = VOS_FALSE;
    sapContext->isCacEndNotified = VOS_FALSE;
    pMac->sap.sapCtxList[sapContext->sessionId].pSapContext = NULL;
    sapContext->isSapSessionOpen = false;

    if (NULL == sap_find_valid_concurrent_session(hHal))
    {
        /* If timer is running then stop the timer and destory
         * it
         */
         VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
         "sapdfs: no session are valid, so clearing dfs global structure");
        /* CAC timer will be initiated and started only when SAP starts on
        * DFS channel and it will be stopped and destroyed immediately once the
        * radar detected or timedout. So as per design CAC timer should be
         * destroyed after stop..*/
        if (pMac->sap.SapDfsInfo.is_dfs_cac_timer_running)
        {
            vos_timer_stop(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);
            pMac->sap.SapDfsInfo.is_dfs_cac_timer_running = 0;
            vos_timer_destroy(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);
        }
        pMac->sap.SapDfsInfo.cac_state = eSAP_DFS_DO_NOT_SKIP_CAC;
        sap_CacResetNotify(hHal);
        vos_mem_zero(&pMac->sap, sizeof(pMac->sap));

        /*
         * No valid concurrent AP sessions(SAP/P2PGO), switch back the
         * allowed action frames bitmask to STA mode and set the same
         * to FW
         */
        sme_set_allowed_action_frames(hHal,
			ALLOWED_ACTION_FRAMES_BITMAP0_STA, true);
    }

    return halstatus;
}

/*==========================================================================
  FUNCTION  sap_CacResetNotify

  DESCRIPTION Function will be called up on stop bss indication to clean up
              DFS global structure.

  DEPENDENCIES PARAMETERS
     IN hHAL : HAL pointer

  RETURN VALUE  : void.

  SIDE EFFECTS
============================================================================*/
void sap_CacResetNotify(tHalHandle hHal)
{
    v_U8_t intf = 0;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
    {
         ptSapContext pSapContext =
                    (ptSapContext)pMac->sap.sapCtxList [intf].pSapContext;
         if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
             (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
             pMac->sap.sapCtxList [intf].pSapContext != NULL)
         {
              pSapContext->isCacStartNotified = VOS_FALSE;
              pSapContext->isCacEndNotified = VOS_FALSE;
         }
    }
}

/*==========================================================================
  FUNCTION  sap_CacStartNotify

  DESCRIPTION Function will be called to Notify eSAP_DFS_CAC_START event
              to HDD

  DEPENDENCIES PARAMETERS
     IN hHAL : HAL pointer

  RETURN VALUE  : VOS_STATUS.

  SIDE EFFECTS
============================================================================*/
VOS_STATUS sap_CacStartNotify(tHalHandle hHal)
{
    v_U8_t intf = 0;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
    VOS_STATUS vosStatus = VOS_STATUS_E_FAILURE;

    for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
    {
         ptSapContext pSapContext =
                    (ptSapContext)pMac->sap.sapCtxList [intf].pSapContext;
         if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
             (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
             pMac->sap.sapCtxList [intf].pSapContext != NULL &&
             (VOS_FALSE == pSapContext->isCacStartNotified))
         {
              VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
              "sapdfs: Signaling eSAP_DFS_CAC_START to HDD for sapctx[%pK]",
              pSapContext);

              vosStatus = sapSignalHDDevent(pSapContext, NULL,
                                            eSAP_DFS_CAC_START,
                                            (v_PVOID_t) eSAP_STATUS_SUCCESS);
              if (VOS_STATUS_SUCCESS != vosStatus)
              {
                  VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                            "In %s, failed setting isCacStartNotified on interface[%d]",
                            __func__, intf);
                  return vosStatus;
              }
              pSapContext->isCacStartNotified = VOS_TRUE;
         }
    }
    return vosStatus;
}

/*==========================================================================
  FUNCTION  sap_CacEndNotify

  DESCRIPTION Function will be called to Notify eSAP_DFS_CAC_END event
              to HDD

  DEPENDENCIES PARAMETERS
     IN hHAL : HAL pointer

  RETURN VALUE  : VOS_STATUS.

  SIDE EFFECTS
============================================================================*/
VOS_STATUS sap_CacEndNotify(tHalHandle hHal, tCsrRoamInfo *roamInfo)
{
     v_U8_t intf;
     tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
     VOS_STATUS vosStatus = VOS_STATUS_E_FAILURE;

     /*
      * eSAP_DFS_CHANNEL_CAC_END:
      * CAC Period elapsed and there was no radar
      * found so, SAP can continue beaconing.
      * sap_radar_found_status is set to 0
      */
     for ( intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
     {
           ptSapContext pSapContext =
               (ptSapContext)pMac->sap.sapCtxList [intf].pSapContext;
           if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
               (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
               pMac->sap.sapCtxList [intf].pSapContext != NULL &&
               (VOS_FALSE == pSapContext->isCacEndNotified))
           {
                pSapContext = pMac->sap.sapCtxList [intf].pSapContext;
                vosStatus = sapSignalHDDevent(pSapContext, NULL,
                                              eSAP_DFS_CAC_END,
                                              (v_PVOID_t) eSAP_STATUS_SUCCESS);
                if (VOS_STATUS_SUCCESS != vosStatus)
                {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                              "In %s, failed setting isCacEndNotified on interface[%d]",
                              __func__, intf);
                    return vosStatus;
                }
                pSapContext->isCacEndNotified = VOS_TRUE;
                pMac->sap.SapDfsInfo.sap_radar_found_status = VOS_FALSE;
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                          "sapdfs: Start beacon request on sapctx[%pK]",
                          pSapContext);

                /* Start beaconing on the new channel */
                WLANSAP_StartBeaconReq((v_PVOID_t)pSapContext);

                /* Transition from eSAP_STARTING to eSAP_STARTED
                 * (both without substates)
                 */
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                          "sapdfs: channel[%d] from state %s => %s",
                           pSapContext->channel, "eSAP_STARTING",
                          "eSAP_STARTED");

                pSapContext->sapsMachine = eSAP_STARTED;

                /*Action code for transition */
                vosStatus = sapSignalHDDevent(pSapContext, roamInfo,
                                              eSAP_START_BSS_EVENT,
                                              (v_PVOID_t)eSAP_STATUS_SUCCESS);
                if (VOS_STATUS_SUCCESS != vosStatus)
                {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                              "In %s, failed setting isCacEndNotified on interface[%d]",
                              __func__, intf);
                    return vosStatus;
                }

                /* Transition from eSAP_STARTING to eSAP_STARTED
                 * (both without substates)
                 */
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                          "In %s, from state %s => %s",
                          __func__, "eSAP_DFS_CAC_WAIT", "eSAP_STARTED");
            }
      }
      /*
       * All APs are done with CAC timer, all APs should start beaconing.
       * Lets assume AP1 and AP2 started beaconing on DFS channel, Now lets
       * say AP1 goes down and comes back on same DFS channel. In this case
       * AP1 shouldn't start CAC timer and start beacon immediately beacause
       * AP2 is already beaconing on this channel. This case will be handled
       * by checking against eSAP_DFS_SKIP_CAC while starting the timer.
       */
      pMac->sap.SapDfsInfo.cac_state = eSAP_DFS_SKIP_CAC;
      return vosStatus;
}

/**
 * sap_relaunch_acs_result_handler() - handle relaunched ACS result
 * @sap_context: pointer of ptSapContext
 * @roam_info:   pointer to tCsrRoamInfo which contains result channel
 *
 * This function handle the relaunched ACS result and check if need to
 * trigger CSA.
 *
 * Return: VOS_STATUS
 */
VOS_STATUS
sap_relaunch_acs_result_handler(ptSapContext sap_context,
				tCsrRoamInfo *roam_info)
{
	VOS_STATUS vos_status;

	vos_mem_zero(sap_context->acs_cfg,
		     sizeof(struct sap_acs_cfg));

	vos_status = sapSignalHDDevent(sap_context, NULL,
				       eSAP_ACS_CHANNEL_SELECTED,
				       (v_PVOID_t)eSAP_STATUS_SUCCESS);

	if (!VOS_IS_STATUS_SUCCESS(vos_status))
		VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
			  FL("Failed to indicate ACS complete."));

	VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
		  "In %s: acs_target_channel=%d cur_channel=%d",
		  __func__,
		  roam_info->target_channel,
		  sap_context->channel);

	if (!((roam_info->target_channel == sap_context->channel) ||
	      (roam_info->target_channel == SAP_CHANNEL_NOT_SELECTED))) {
		vos_status = sapSignalHDDevent(sap_context, roam_info,
					       eSAP_ECSA_CHANGE_CHAN_IND,
					       (v_PVOID_t)NULL);
		if (!VOS_IS_STATUS_SUCCESS(vos_status))
			VOS_TRACE(VOS_MODULE_ID_SAP,
				  VOS_TRACE_LEVEL_ERROR,
				  FL("trigger channel switch failed"));
	}

	return vos_status;
}

/*==========================================================================
  FUNCTION    sapFsm

  DESCRIPTION
    SAP State machine entry function

  DEPENDENCIES
    NA.

  PARAMETERS

    IN
    sapContext  : Sap Context value
    sapEvent    : State machine event
    status      : Return the SAP status here

  RETURN VALUE
    The VOS_STATUS code associated with performing the operation

    VOS_STATUS_SUCCESS: Success

  SIDE EFFECTS
============================================================================*/
VOS_STATUS
sapFsm
(
    ptSapContext sapContext,    /* sapContext value */
    ptWLAN_SAPEvent sapEvent   /* State machine event */
)
{
   /* Retrieve the phy link state machine structure
     * from the sapContext value
     */
    eSapFsmStates_t stateVar = sapContext->sapsMachine; /*state var that keeps track of state machine*/
    tCsrRoamInfo    *roamInfo = (tCsrRoamInfo *)(sapEvent->params);
    v_U32_t msg = sapEvent->event;  /* State machine input event message */
    VOS_STATUS vosStatus = VOS_STATUS_E_FAILURE;
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    tpAniSirGlobal pMac;
    v_U32_t cbMode;
    v_BOOL_t b_leak_chan = FALSE;
#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
    v_U8_t temp_chan;
    tSapDfsNolInfo *pNol;
#endif

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return VOS_STATUS_E_FAILURE;
    }
    pMac = PMAC_STRUCT( hHal );

    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_DEBUG, "%s: sapContext=%pK, stateVar=%d, msg=0x%x", __func__, sapContext, stateVar, msg);

    switch (stateVar)
    {
        case eSAP_DISCONNECTED:
            if ((msg == eSAP_HDD_START_INFRA_BSS))
            {
                /* Transition from eSAP_DISCONNECTED to eSAP_CH_SELECT (both without substates) */
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, new from state %s => %s, session id %d",
                            __func__, "eSAP_DISCONNECTED", "eSAP_CH_SELECT", sapContext->sessionId);

                if (sapContext->isSapSessionOpen == eSAP_FALSE) {
                    uint32_t type, subtype;
                    if (sapContext->csrRoamProfile.csrPersona ==
                            VOS_P2P_GO_MODE)
                        vosStatus = vos_get_vdev_types(VOS_P2P_GO_MODE,
                                &type, &subtype);
                    else
                        vosStatus = vos_get_vdev_types(VOS_STA_SAP_MODE,
                                &type, &subtype);
                    if (VOS_STATUS_SUCCESS != vosStatus) {
                        VOS_TRACE(VOS_MODULE_ID_SAP,
                                VOS_TRACE_LEVEL_FATAL,
                                "failed to get vdev type");
                        return VOS_STATUS_E_FAILURE;
                    }
                    vosStatus = sme_OpenSession(hHal, &WLANSAP_RoamCallback,
                            sapContext, sapContext->self_mac_addr,
                            &sapContext->sessionId, type, subtype);
                    if (VOS_STATUS_SUCCESS != vosStatus) {
                        VOS_TRACE(VOS_MODULE_ID_SAP,
                                VOS_TRACE_LEVEL_ERROR,
                                FL("Error: calling sme_OpenSession"));
                        return VOS_STATUS_E_FAILURE;
                    }
                    sapContext->isSapSessionOpen = eSAP_TRUE;
                }

                /* init dfs channel nol */
                sapInitDfsChannelNolList(sapContext);

                /* Set SAP device role */
                sapContext->sapsMachine = eSAP_CH_SELECT;

                /*
                 * Perform sme_ScanRequest
                 * This scan request is post start bss
                 * request so, set the third to false.
                 */
                vosStatus = sapGotoChannelSel(sapContext, sapEvent, VOS_FALSE);

            }
            else if (msg == eSAP_DFS_CHANNEL_CAC_START)
            {
               /* No need of state check here, caller is expected to perform
                * the checks before sending the event
                */
               sapContext->sapsMachine = eSAP_DFS_CAC_WAIT;

               VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
               "sapdfs: from state eSAP_DISCONNECTED => SAP_DFS_CAC_WAIT");
               if ( pMac->sap.SapDfsInfo.is_dfs_cac_timer_running != VOS_TRUE)
               {
                   VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                             "sapdfs: starting dfs cac timer on sapctx[%pK]",
                             sapContext);
                   sapStartDfsCacTimer(sapContext);
               }

               vosStatus = sap_CacStartNotify(hHal);
            }
            else if (msg == eSAP_CHANNEL_SELECTION_RETRY)
            {
                 /* Set SAP device role */
                sapContext->sapsMachine = eSAP_CH_SELECT;

                /*
                 * Perform sme_ScanRequest
                 * This scan request is post start bss
                 * request so, set the third to false.
                 */
                vosStatus = sapGotoChannelSel(sapContext, sapEvent, VOS_FALSE);
            }
            else
            {
                 VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "In %s, in state %s, event msg %d",
                             __func__, "eSAP_DISCONNECTED", msg);
            }

            break;

        case eSAP_CH_SELECT:

            if (msg == eSAP_MAC_SCAN_COMPLETE)
            {
                 tHalHandle  hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
                 if (NULL == hHal)
                 {
                     VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                         "In %s, NULL hHal in state %s, msg %d ", __func__,
                         "eSAP_CH_SELECT", msg);
                     return VOS_STATUS_E_FAULT;
                 }
                 cbMode = sme_SelectCBMode(hHal,
                                        sapContext->csrRoamProfile.phyMode,
                                        sapContext->channel,
                                        sapContext->secondary_ch,
                                        &sapContext->vht_channel_width,
                                        sapContext->ch_width_orig);

                 cbMode = sme_GetCBPhyStateFromCBIniValue(cbMode);
#ifdef WLAN_ENABLE_CHNL_MATRIX_RESTRICTION
                 temp_chan = sapContext->channel;
                 pNol = pMac->sap.SapDfsInfo.sapDfsChannelNolList;

                 sapMarkChannelsLeakingIntoNOL(sapContext,
                         cbMode, pNol, 1, &temp_chan);

                 /* if selelcted channel has leakage to channels
                    in NOL, the temp_chan will be reset */
                 b_leak_chan = (temp_chan != sapContext->channel);
#endif
                 /* check if channel is in DFS_NOL or
                    if the channel has leakage to the channels in NOL */
                 if (sapDfsIsChannelInNolList(sapContext, sapContext->channel,
                             cbMode) || b_leak_chan)
                 {
                     v_U8_t ch;

                     /* find a new available channel */
                     ch = sapRandomChannelSel(sapContext);
                     if (ch == 0) {
                         /* No available channel found */
                         VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                             FL("No available channel found!!!"));
                         sapSignalHDDevent(sapContext, NULL,
                                           eSAP_DFS_NO_AVAILABLE_CHANNEL,
                                           (v_PVOID_t) eSAP_STATUS_SUCCESS);
                         return VOS_STATUS_E_FAULT;
                     }

                     VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                         FL("channel %d is in NOL, StartBss on new channel %d"),
                         sapContext->channel, ch);

                     sapContext->channel = ch;
                     sme_SelectCBMode(hHal, sapContext->csrRoamProfile.phyMode,
                                       sapContext->channel,
                                       sapContext->secondary_ch,
                                       &sapContext->vht_channel_width,
                                       sapContext->ch_width_orig);
                 }
                 if (sapContext->channel > 14 &&
                         (sapContext->csrRoamProfile.phyMode ==
                         eCSR_DOT11_MODE_11g ||
                         sapContext->csrRoamProfile.phyMode ==
                         eCSR_DOT11_MODE_11g_ONLY))
                     sapContext->csrRoamProfile.phyMode = eCSR_DOT11_MODE_11a;

                 /* when AP2 is started while AP1 is performing ACS, we may not
                  * have the AP1 channel yet.So here after the completion of AP2
                  * ACS check if AP1 ACS resulting channel is DFS and if yes
                  * override AP2 ACS scan result with AP1 DFS channel
                  */
                 if (vos_concurrent_beaconing_sessions_running()) {
                     v_U16_t con_ch;

                     con_ch = sme_GetConcurrentOperationChannel(hHal);
                     if (con_ch && VOS_IS_DFS_CH(con_ch))
                         sapContext->channel = con_ch;
                 }
                 /* Transition from eSAP_CH_SELECT to eSAP_STARTING (both without substates) */
                 VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state %s => %s",
                            __func__, "eSAP_CH_SELECT", "eSAP_STARTING");
                 // Channel selected. Now can sapGotoStarting
                 sapContext->sapsMachine = eSAP_STARTING;
                 // Specify the channel
                 sapContext->csrRoamProfile.ChannelInfo.numOfChannels = 1;
                 sapContext->csrRoamProfile.ChannelInfo.ChannelList = &sapContext->csrRoamProfile.operationChannel;
                 sapContext->csrRoamProfile.operationChannel = (tANI_U8)sapContext->channel;
                 sapContext->csrRoamProfile.vht_channel_width =
                                               sapContext->vht_channel_width;

                 sapContext->csrRoamProfile.beacon_tx_rate =
                                                    sapContext->beacon_tx_rate;

                 VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                         "%s: notify hostapd about channel selection: %d",
                         __func__, sapContext->channel);
                 sapSignalHDDevent(sapContext, NULL, eSAP_CHANNEL_CHANGE_EVENT,
                                        (v_PVOID_t) eSAP_STATUS_SUCCESS);
                 vosStatus = sapGotoStarting( sapContext, sapEvent, eCSR_BSS_TYPE_INFRA_AP);
            }
            else if (msg == eSAP_CHANNEL_SELECTION_FAILED) {
                 sapContext->sapsMachine = eSAP_DISCONNECTED;
                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                           "\n\n***In %s, Cannot start BSS, ACS Fail***\n\n",
                             __func__);
                sapSignalHDDevent(sapContext, NULL, eSAP_START_BSS_EVENT,
                               (v_PVOID_t) eSAP_STATUS_FAILURE);
            } else if (msg == eSAP_HDD_STOP_INFRA_BSS) {
                 sapContext->sapsMachine = eSAP_DISCONNECTED;
                 sapSignalHDDevent(sapContext, NULL, eSAP_START_BSS_EVENT,
                                                (v_PVOID_t)eSAP_STATUS_FAILURE);
                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                     "%s: BSS stopped during Ch select in Progress", __func__);
            } else {
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "In %s, in state %s, invalid event msg %d",
                            __func__, "eSAP_CH_SELECT", msg);
            }
            break;

        case eSAP_DFS_CAC_WAIT:
            if (msg == eSAP_DFS_CHANNEL_CAC_START)
            {
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state %s => %s",
                            __func__, "eSAP_CH_SELECT", "eSAP_DFS_CAC_WAIT");
                if ( pMac->sap.SapDfsInfo.is_dfs_cac_timer_running != VOS_TRUE)
                    sapStartDfsCacTimer(sapContext);

                vosStatus = sap_CacStartNotify(hHal);

            }
            else if (msg == eSAP_DFS_CHANNEL_CAC_RADAR_FOUND)
            {
               v_U8_t  intf;
               /* Radar found while performing channel availability
                * check, need to switch the channel again
                */
               eCsrPhyMode phyMode = sapContext->csrRoamProfile.phyMode;
               tHalHandle hHal =
                  (tHalHandle)vos_get_context(VOS_MODULE_ID_SME, sapContext->pvosGCtx);


               VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                  "ENTERTRED CAC WAIT STATE-->eSAP_DISCONNECTING\n");
                if (NULL == hHal)
                {
                   VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                               "In %s, NULL hHal in state %s, msg %d",
                               __func__, "eSAP_DFS_CAC_WAIT", msg);
                }
                else if (pMac->sap.SapDfsInfo.target_channel)
                {
                   sme_SelectCBMode(hHal, phyMode,
                                    pMac->sap.SapDfsInfo.target_channel,
                                    0, &sapContext->vht_channel_width,
                                    sapContext->ch_width_orig);
                }

                for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
                {
                     ptSapContext sapContext;
                     sapContext = pMac->sap.sapCtxList [intf].pSapContext;
                     if (((VOS_STA_SAP_MODE ==
                           pMac->sap.sapCtxList[intf].sapPersona) ||
                          (VOS_P2P_GO_MODE ==
                           pMac->sap.sapCtxList[intf].sapPersona)) &&
                           sapContext != NULL &&
                           sapContext->sapsMachine != eSAP_DISCONNECTED)
                     {
                         /* SAP to be moved to DISCONNECTING state */
                         sapContext->sapsMachine = eSAP_DISCONNECTING;
                         /*
                          * eSAP_DFS_CHANNEL_CAC_RADAR_FOUND:
                          * A Radar is found on current DFS Channel
                          * while in CAC WAIT period So, do a channel switch
                          * to randomly selected  target channel.
                          * Send the Channel change message to SME/PE.
                          * sap_radar_found_status is set to 1
                          */
                         sapSignalHDDevent(sapContext, NULL,
                                           eSAP_DFS_RADAR_DETECT,
                                           (v_PVOID_t) eSAP_STATUS_SUCCESS);

                         WLANSAP_ChannelChangeRequest((v_PVOID_t)sapContext,
                              pMac->sap.SapDfsInfo.target_channel);
                     }
                }
            }
            else if (msg == eSAP_DFS_CHANNEL_CAC_END)
            {
                vosStatus = sap_CacEndNotify(hHal, roamInfo);
            }
            else if (msg == eSAP_HDD_STOP_INFRA_BSS)
            {
                /* Transition from eSAP_DFS_CAC_WAIT to eSAP_DISCONNECTING */
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                        "In %s, from state %s => %s",
                        __func__,
                        "eSAP_DFS_CAC_WAIT",
                        "eSAP_DISCONNECTING");

                /*
                 * Stop the CAC timer only in following conditions
                 * single AP: if there is a single AP then stop the timer
                 * mulitple APs: incase of multiple APs, make sure that
                 *               all APs are down.
                 */
                if (NULL == sap_find_valid_concurrent_session(hHal))
                {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                              "sapdfs: no sessions are valid, stopping timer");

                    sapStopDfsCacTimer(sapContext);
                }

                sapContext->sapsMachine = eSAP_DISCONNECTING;
                vosStatus = sapGotoDisconnecting(sapContext);
            }
            else
            {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                          "In %s, in state %s, invalid event msg %d",
                          __func__, "eSAP_DFS_CAC_WAIT", msg);
            }
            break;

        case eSAP_STARTING:
            if (msg == eSAP_MAC_START_BSS_SUCCESS )
            {
                /* Transition from eSAP_STARTING to eSAP_STARTED (both without substates) */
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state channel = %d %s => %s",
                            __func__,sapContext->channel, "eSAP_STARTING", "eSAP_STARTED");

                 sapContext->sapsMachine = eSAP_STARTED;

                 /*Action code for transition */
                 vosStatus = sapSignalHDDevent( sapContext, roamInfo, eSAP_START_BSS_EVENT, (v_PVOID_t)eSAP_STATUS_SUCCESS);

                  /* The upper layers have been informed that AP is up and
                  * running, however, the AP is still not beaconing, until
                  * CAC is done if the operating channel is DFS
                  */
                 if (NV_CHANNEL_DFS ==
                     vos_nv_getChannelEnabledState(sapContext->channel)
                    )
                 {
                    if ((VOS_FALSE == pMac->sap.SapDfsInfo.ignore_cac) &&
                        (eSAP_DFS_DO_NOT_SKIP_CAC ==
                         pMac->sap.SapDfsInfo.cac_state))
                    {
                        /* Move the device in CAC_WAIT_STATE */
                        sapContext->sapsMachine = eSAP_DFS_CAC_WAIT;

                        /* TODO: Need to stop the OS transmit queues,
                         * so that no traffic can flow down the stack
                         */

                        /* Start CAC wait timer */
                        if (pMac->sap.SapDfsInfo.is_dfs_cac_timer_running !=
                            TRUE)
                            sapStartDfsCacTimer(sapContext);

                        vosStatus = sap_CacStartNotify(hHal);

                    }
                    else
                    {
                        WLANSAP_StartBeaconReq((v_PVOID_t)sapContext);
                    }
                 }
             }
             else if (msg == eSAP_HDD_STOP_INFRA_BSS)
             {
                 /* Transition from eSAP_STARTING to eSAP_DISCONNECTPENDING */
                 VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state %s => %s",
                             __func__, "eSAP_STARTING", "eSAP_DISCONNECTPENDING");
                 sapContext->sapsMachine = eSAP_DISCONNECTPENDING;
             }
             else if (msg == eSAP_MAC_START_FAILS)
             {
                 /*Transition from eSAP_STARTING to eSAP_DISCONNECTED (both without substates)*/
                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                            "In %s, from state %s => %s", __func__,
                            "eSAP_STARTING", "eSAP_DISCONNECTED");

                 /*Advance outer statevar */
                 sapContext->sapsMachine = eSAP_DISCONNECTED;
                 vosStatus = sapSignalHDDevent(sapContext, NULL,
                                               eSAP_START_BSS_EVENT,
                                               (v_PVOID_t)eSAP_STATUS_FAILURE);
                 vosStatus = sapGotoDisconnected(sapContext);
                 /* Close the SME session*/

                 if (eSAP_TRUE == sapContext->isSapSessionOpen)
                 {
                    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
                    if (NULL == hHal)
                    {
                       VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                                  "In %s, NULL hHal in state %s, msg %d",
                                  __func__, "eSAP_STARTING", msg);
                    }
                    else if (eHAL_STATUS_SUCCESS ==
                         sap_CloseSession(hHal,
                                          sapContext, NULL, FALSE))
                     {
                         sapContext->isSapSessionOpen = eSAP_FALSE;
                     }
                 }
             }
             else if (msg == eSAP_OPERATING_CHANNEL_CHANGED)
             {
                 /* The operating channel has changed, update hostapd */
                 sapContext->channel =
                     (tANI_U8)pMac->sap.SapDfsInfo.target_channel;
                sapContext->sapsMachine = eSAP_STARTED;

                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                           "In %s, from state %s => %s",
                           __func__, "eSAP_STARTING", "eSAP_STARTED");

                 /* Indicate change in the state to upper layers */
                 vosStatus = sapSignalHDDevent(sapContext, roamInfo,
                                               eSAP_START_BSS_EVENT,
                                               (v_PVOID_t)eSAP_STATUS_SUCCESS);
             }
             else
             {
                 VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                             "In %s, in state %s, invalid event msg %d",
                             __func__, "eSAP_STARTING", msg);
                 /* Intentionally left blank */
             }
             break;
        case eSAP_DISCONNECTPENDING:
            /* eSAP_DISCONNECTPENDING is state for handling SAP Disconnection
             * when DUT is in eSAP_STARTING state. Should not used for OTHER
             * purpose.
             */
            if (msg == eSAP_MAC_START_BSS_SUCCESS) {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                           "In %s, from state %s => %s", __func__,
                           "eSAP_DISCONNECTPENDING", "eSAP_DISCONNECTING");
                sapContext->sapsMachine = eSAP_DISCONNECTING;
                vosStatus = sapSignalHDDevent(sapContext, NULL,
                                              eSAP_START_BSS_EVENT,
                                              (v_PVOID_t)eSAP_STATUS_FAILURE);
                vosStatus = sapGotoDisconnecting(sapContext);
            } else if (msg == eSAP_MAC_START_FAILS) {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                           "In %s, from state %s => %s", __func__,
                           "eSAP_DISCONNECTPENDING", "eSAP_DISCONNECTED");

                sapContext->sapsMachine = eSAP_DISCONNECTED;
                vosStatus = sapSignalHDDevent(sapContext, NULL,
                                              eSAP_START_BSS_EVENT,
                                              (v_PVOID_t)eSAP_STATUS_FAILURE);
                vosStatus = sapGotoDisconnected(sapContext);
                if (eSAP_TRUE == sapContext->isSapSessionOpen)
                {
                    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
                    if (NULL == hHal)
                    {
                       VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                                  "In %s, NULL hHal in state %s, msg %d",
                                  __func__, "eSAP_DISCONNECTPENDING", msg);
                    }
                    else if (eHAL_STATUS_SUCCESS ==
                         sap_CloseSession(hHal, sapContext, NULL, FALSE))
                    {
                         sapContext->isSapSessionOpen = eSAP_FALSE;
                    }
                }
            } else {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                           "In %s, in state %s, invalid event msg %d",
                            __func__, "eSAP_DISCONNECTPENDING", msg);
            }
            break;
        case eSAP_STARTED:
            if (msg == eSAP_HDD_STOP_INFRA_BSS)
            {
                /* Transition from eSAP_STARTED to eSAP_DISCONNECTING (both without substates) */
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state %s => %s",
                           __func__, "eSAP_STARTED", "eSAP_DISCONNECTING");
                sapContext->sapsMachine = eSAP_DISCONNECTING;
                vosStatus = sapGotoDisconnecting(sapContext);
            }
            else if (eSAP_DFS_CHNL_SWITCH_ANNOUNCEMENT_START == msg)
            {
                v_U8_t  intf;
                /* Radar is seen on the current operating channel
                 * send CSA IE for all associated stations
                 */
                if (pMac != NULL)
                {
                    /* Request for CSA IE transmission */
                    for ( intf = 0; intf < SAP_MAX_NUM_SESSION; intf++)
                    {
                         ptSapContext pSapContext;

                        if (((VOS_STA_SAP_MODE ==
                                pMac->sap.sapCtxList [intf].sapPersona) ||
                             (VOS_P2P_GO_MODE ==
                                pMac->sap.sapCtxList [intf].sapPersona)) &&
                          pMac->sap.sapCtxList [intf].pSapContext != NULL )
                        {
                            pSapContext = pMac->sap.sapCtxList [intf].pSapContext;
                            VOS_TRACE(VOS_MODULE_ID_SAP,
                                      VOS_TRACE_LEVEL_INFO_MED,
                                      "sapdfs: Sending CSAIE for sapctx[%pK]",
                                      pSapContext);

                            vosStatus =
                            WLANSAP_DfsSendCSAIeRequest((v_PVOID_t)pSapContext);
                        }
                    }
                }
            } else if (eSAP_MAC_SCAN_COMPLETE == msg) {
                if (!sapContext->acs_cfg->acs_mode) {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                              "In %s, not a in ACS mode", __func__);
                    return VOS_STATUS_E_INVAL;
                }

                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                          "In %s, ACS scan complete in state %s",
                          __func__,
                          "eSAP_STARTED");

                if (!roamInfo) {
                    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                              "In %s, roam info can't be NULL",
                              __func__);
                    return VOS_STATUS_E_INVAL;
                }

                vosStatus = sap_relaunch_acs_result_handler(sapContext,
                                                            roamInfo);
            }
            else
            {
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, "In %s, in state %s, invalid event msg %d",
                           __func__, "eSAP_STARTED", msg);
            }
            break;

        case eSAP_DISCONNECTING:
            if (msg == eSAP_MAC_READY_FOR_CONNECTIONS)
            {
                /* Transition from eSAP_DISCONNECTING to eSAP_DISCONNECTED (both without substates) */
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH, "In %s, from state %s => %s",
                          __func__, "eSAP_DISCONNECTING", "eSAP_DISCONNECTED");

                sapContext->sapsMachine = eSAP_DISCONNECTED;

                /* Close the SME session*/
                if (eSAP_TRUE == sapContext->isSapSessionOpen)
                {
                    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);

                    if (NULL == hHal)
                    {
                        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                                   "In %s, NULL hHal in state %s, msg %d",
                                   __func__, "eSAP_DISCONNECTING", msg);
                    }
                    else
                    {
                        sapContext->isSapSessionOpen = eSAP_FALSE;
                        if (!(eHAL_STATUS_SUCCESS ==
                            sap_CloseSession(hHal,
                                     sapContext,
                                     sapRoamSessionCloseCallback, TRUE)))
                        {
                            vosStatus = sapSignalHDDevent(sapContext, NULL,
                                              eSAP_STOP_BSS_EVENT,
                                              (v_PVOID_t) eSAP_STATUS_SUCCESS);
                        }
                    }
                }
            }
            else if (msg == eWNI_SME_CHANNEL_CHANGE_REQ)
            {
               VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                         "sapdfs: Send channel change request on sapctx[%pK]",
                         sapContext);
               /* Most likely, radar has been detected and SAP wants to
                * change the channel
                */
               vosStatus = WLANSAP_ChannelChangeRequest((v_PVOID_t)sapContext,
                                          pMac->sap.SapDfsInfo.target_channel);

               VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
                         "In %s, Sending DFS eWNI_SME_CHANNEL_CHANGE_REQ",
                         __func__);
            }
            else if (msg == eWNI_SME_CHANNEL_CHANGE_RSP)
            {
                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
                          "In %s, in state %s, event msg %d result %d",
                          __func__, "eSAP_DISCONNECTING ", msg, sapEvent->u2);

                if (sapEvent->u2 == eCSR_ROAM_RESULT_CHANNEL_CHANGE_FAILURE)
                {
                    vosStatus = sapGotoDisconnecting(sapContext);
                }
            }
            else
            {
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                           "In %s, in state %s, invalid event msg %d",
                          __func__, "eSAP_DISCONNECTING", msg);
            }
            break;
      }
      return vosStatus;
}// sapFsm


eSapStatus
sapconvertToCsrProfile(tsap_Config_t *pconfig_params, eCsrRoamBssType bssType, tCsrRoamProfile *profile)
{
    //Create Roam profile for SoftAP to connect
    profile->BSSType = eCSR_BSS_TYPE_INFRA_AP;
    profile->SSIDs.numOfSSIDs = 1;
    profile->csrPersona = pconfig_params->persona;
    profile->disableDFSChSwitch = pconfig_params->disableDFSChSwitch;

    vos_mem_zero(profile->SSIDs.SSIDList[0].SSID.ssId,
                 sizeof(profile->SSIDs.SSIDList[0].SSID.ssId));

    //Flag to not broadcast the SSID information
    profile->SSIDs.SSIDList[0].ssidHidden =  pconfig_params->SSIDinfo.ssidHidden;

    profile->SSIDs.SSIDList[0].SSID.length = pconfig_params->SSIDinfo.ssid.length;
    vos_mem_copy(&profile->SSIDs.SSIDList[0].SSID.ssId, pconfig_params->SSIDinfo.ssid.ssId,
                  sizeof(pconfig_params->SSIDinfo.ssid.ssId));

    profile->negotiatedAuthType = eCSR_AUTH_TYPE_OPEN_SYSTEM;

    if (pconfig_params->authType == eSAP_OPEN_SYSTEM)
    {
        profile->negotiatedAuthType = eCSR_AUTH_TYPE_OPEN_SYSTEM;
    }
    else if (pconfig_params->authType == eSAP_SHARED_KEY)
    {
        profile->negotiatedAuthType = eCSR_AUTH_TYPE_SHARED_KEY;
    }
    else
    {
        profile->negotiatedAuthType = eCSR_AUTH_TYPE_AUTOSWITCH;
    }

    profile->AuthType.numEntries = 1;
    profile->AuthType.authType[0] = eCSR_AUTH_TYPE_OPEN_SYSTEM;

    //Always set the Encryption Type
    profile->EncryptionType.numEntries = 1;
    profile->EncryptionType.encryptionType[0] = pconfig_params->RSNEncryptType;

    profile->mcEncryptionType.numEntries = 1;
    profile->mcEncryptionType.encryptionType[0] = pconfig_params->mcRSNEncryptType;

    if (pconfig_params->privacy & eSAP_SHARED_KEY)
    {
        profile->AuthType.authType[0] = eCSR_AUTH_TYPE_SHARED_KEY;
    }

    profile->privacy = pconfig_params->privacy;
    profile->fwdWPSPBCProbeReq = pconfig_params->fwdWPSPBCProbeReq;

    if (pconfig_params->authType == eSAP_SHARED_KEY)
    {
        profile->csr80211AuthType = eSIR_SHARED_KEY;
    }
    else if (pconfig_params->authType == eSAP_OPEN_SYSTEM)
    {
        profile->csr80211AuthType = eSIR_OPEN_SYSTEM;
    }
    else
    {
        profile->csr80211AuthType = eSIR_AUTO_SWITCH;
    }

    //Initialize we are not going to use it
    profile->pWPAReqIE = NULL;
    profile->nWPAReqIELength = 0;

    //set the RSN/WPA IE
    profile->pRSNReqIE = NULL;
    profile->nRSNReqIELength = pconfig_params->RSNWPAReqIELength;
    if (pconfig_params->RSNWPAReqIELength)
    {
        profile->pRSNReqIE = vos_mem_malloc(pconfig_params->RSNWPAReqIELength);
        if( NULL == profile->pRSNReqIE )
        {
           VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR, " %s Fail to alloc memory", __func__);
           return eSAP_STATUS_FAILURE;
        }
        vos_mem_copy(profile->pRSNReqIE, pconfig_params->RSNWPAReqIE,
                                  pconfig_params->RSNWPAReqIELength);
        profile->nRSNReqIELength = pconfig_params->RSNWPAReqIELength;
    }

    // Turn off CB mode
    profile->CBMode = eCSR_CB_OFF;

    //set the phyMode to accept anything
    //Best means everything because it covers all the things we support
    /*eCSR_DOT11_MODE_BEST*/
    profile->phyMode = pconfig_params->SapHw_mode;

    //Configure beaconInterval
    profile->beaconInterval = (tANI_U16)pconfig_params->beacon_int;

    // set DTIM period
    profile->dtimPeriod = pconfig_params->dtim_period;

    //set Uapsd enable bit
    profile->ApUapsdEnable = pconfig_params->UapsdEnable;

    //Enable protection parameters
    profile->protEnabled       = pconfig_params->protEnabled;
    profile->obssProtEnabled   = pconfig_params->obssProtEnabled;
    profile->cfg_protection    = pconfig_params->ht_capab;

    //country code
    if (pconfig_params->countryCode[0])
        vos_mem_copy(profile->countryCode, pconfig_params->countryCode, WNI_CFG_COUNTRY_CODE_LEN);
    profile->ieee80211d = pconfig_params->ieee80211d;
    //wps config info
    profile->wps_state = pconfig_params->wps_state;

#ifdef WLAN_FEATURE_11W
    // MFP capable/required
    profile->MFPCapable = pconfig_params->mfpCapable ? 1 : 0;
    profile->MFPRequired = pconfig_params->mfpRequired ? 1 : 0;
#endif

    if (pconfig_params->probeRespIEsBufferLen > 0 &&
        pconfig_params->pProbeRespIEsBuffer != NULL)
    {
        profile->addIeParams.probeRespDataLen =
            pconfig_params->probeRespIEsBufferLen;
        profile->addIeParams.probeRespData_buff =
            pconfig_params->pProbeRespIEsBuffer;
    }
    else
    {
        profile->addIeParams.probeRespDataLen = 0;
        profile->addIeParams.probeRespData_buff = NULL;
    }
    /*assoc resp IE */
    if (pconfig_params->assocRespIEsLen > 0 &&
            pconfig_params->pAssocRespIEsBuffer != NULL)
    {
        profile->addIeParams.assocRespDataLen =
            pconfig_params->assocRespIEsLen;
        profile->addIeParams.assocRespData_buff =
            pconfig_params->pAssocRespIEsBuffer;
    }
    else
    {
        profile->addIeParams.assocRespDataLen = 0;
        profile->addIeParams.assocRespData_buff = NULL;
    }

    if (pconfig_params->probeRespBcnIEsLen > 0 &&
            pconfig_params->pProbeRespBcnIEsBuffer!= NULL)
    {
        profile->addIeParams.probeRespBCNDataLen =
            pconfig_params->probeRespBcnIEsLen;
        profile->addIeParams.probeRespBCNData_buff =
            pconfig_params->pProbeRespBcnIEsBuffer;
    }
    else
    {
        profile->addIeParams.probeRespBCNDataLen = 0;
        profile->addIeParams.probeRespBCNData_buff = NULL;
    }
    profile->sap_dot11mc = pconfig_params->sap_dot11mc;

    if (pconfig_params->supported_rates.numRates) {
        vos_mem_copy(profile->supported_rates.rate,
                pconfig_params->supported_rates.rate,
                pconfig_params->supported_rates.numRates);
        profile->supported_rates.numRates =
            pconfig_params->supported_rates.numRates;
    }

    if (pconfig_params->extended_rates.numRates) {
        vos_mem_copy(profile->extended_rates.rate,
                pconfig_params->extended_rates.rate,
                pconfig_params->extended_rates.numRates);
        profile->extended_rates.numRates =
            pconfig_params->extended_rates.numRates;
    }

    return eSAP_STATUS_SUCCESS; /* Success. */
}

void sapFreeRoamProfile(tCsrRoamProfile *profile)
{
   if(profile->pRSNReqIE)
   {
      vos_mem_free(profile->pRSNReqIE);
      profile->pRSNReqIE = NULL;
   }
}


void
sapSortMacList(v_MACADDR_t *macList, v_U8_t size)
{
    v_U8_t outer, inner;
    v_MACADDR_t temp;
    v_SINT_t nRes = -1;

    if ((NULL == macList) || (size > MAX_ACL_MAC_ADDRESS)) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  FL("either buffer is NULL or size = %d is more."), size);
        return;
    }

    for(outer = 0; outer < size; outer++)
    {
        for(inner = 0; inner < size - 1; inner++)
        {
            nRes = vos_mem_compare2((macList + inner)->bytes, (macList + inner + 1)->bytes, sizeof(v_MACADDR_t));
            if (nRes > 0)
            {
                vos_mem_copy(temp.bytes, (macList + inner + 1)->bytes, sizeof(v_MACADDR_t));
                vos_mem_copy((macList + inner + 1)->bytes, (macList + inner)->bytes, sizeof(v_MACADDR_t));
                vos_mem_copy((macList + inner)->bytes, temp.bytes, sizeof(v_MACADDR_t));
             }
        }
    }
}

eSapBool
sapSearchMacList(v_MACADDR_t *macList, v_U8_t num_mac, v_U8_t *peerMac, v_U8_t *index)
{
    v_SINT_t nRes = -1;
    v_S7_t nStart = 0, nEnd, nMiddle;
    nEnd = num_mac - 1;

    if ((NULL == macList) || (num_mac > MAX_ACL_MAC_ADDRESS)) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  FL("either buffer is NULL or size = %d is more."),
                  num_mac);
        return eSAP_FALSE;
    }

    while (nStart <= nEnd)
    {
        nMiddle = (nStart + nEnd) / 2;
        nRes = vos_mem_compare2(&macList[nMiddle], peerMac, sizeof(v_MACADDR_t));

        if (0 == nRes)
        {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "search SUCC");
            // "index equals NULL" means the caller does not need the
            // index value of the peerMac being searched
            if (index != NULL)
            {
                *index = (v_U8_t) nMiddle;
                VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                        "index %d", *index);
            }
            return eSAP_TRUE;
        }
        if (nRes < 0)
            nStart = nMiddle + 1;
        else
            nEnd = nMiddle - 1;
    }

    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
            "search not succ");
    return eSAP_FALSE;
}

void
sapAddMacToACL(v_MACADDR_t *macList, v_U8_t *size, v_U8_t *peerMac)
{
    v_SINT_t nRes = -1;
    int i;

    if ((NULL == macList) || (*size > MAX_ACL_MAC_ADDRESS)) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  FL("either buffer is NULL or size = %d is incorrect."),
                  *size);
        return;
    }

    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,"add acl entered");
    for (i=((*size)-1); i>=0; i--)
    {
        nRes = vos_mem_compare2(&macList[i], peerMac, sizeof(v_MACADDR_t));
        if (nRes > 0)
        {
            /* Move alphabetically greater mac addresses one index down to allow for insertion
               of new mac in sorted order */
            vos_mem_copy((macList+i+1)->bytes,(macList+i)->bytes, sizeof(v_MACADDR_t));
        }
        else
        {
            break;
        }
    }
    //This should also take care of if the element is the first to be added in the list
    vos_mem_copy((macList+i+1)->bytes, peerMac, sizeof(v_MACADDR_t));
    // increment the list size
    (*size)++;
}

void
sapRemoveMacFromACL(v_MACADDR_t *macList, v_U8_t *size, v_U8_t index)
{
    int i;
    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,"remove acl entered");
    /* return if the list passed is empty. Ideally this should never happen since this funcn is always
       called after sapSearchMacList to get the index of the mac addr to be removed and this will
       only get called if the search is successful. Still no harm in having the check */
    if ((NULL == macList) || (*size == 0) || (*size > MAX_ACL_MAC_ADDRESS)) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  FL("either buffer is NULL or size = %d is incorrect"),
                  *size);
        return;
    }

    for (i=index; i<((*size)-1); i++)
    {
        /* Move mac addresses starting from "index" passed one index up to delete the void
           created by deletion of a mac address in ACL */
        vos_mem_copy((macList+i)->bytes,(macList+i+1)->bytes, sizeof(v_MACADDR_t));
    }
    // The last space should be made empty since all mac addesses moved one step up
    vos_mem_zero((macList+(*size)-1)->bytes, sizeof(v_MACADDR_t));
    //reduce the list size by 1
    (*size)--;
}

void sapPrintACL(v_MACADDR_t *macList, v_U8_t size)
{
    int i;
    v_BYTE_t *macArray;
    VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,"print acl entered");

    if ((NULL == macList) || (size == 0) || (size >= MAX_ACL_MAC_ADDRESS))
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                    "In %s, either buffer is NULL or size %d is incorrect."
                    , __func__, size);
        return;
    }

    for (i=0; i<size; i++)
    {
        macArray = (macList+i)->bytes;
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                "** ACL entry %i - "MAC_ADDRESS_STR, i,
                MAC_ADDR_ARRAY(macArray));
    }
    return;
}

VOS_STATUS
sapIsPeerMacAllowed(ptSapContext sapContext, v_U8_t *peerMac)
{
    if (eSAP_ALLOW_ALL == sapContext->eSapMacAddrAclMode)
              return VOS_STATUS_SUCCESS;

    if (sapSearchMacList(sapContext->acceptMacList, sapContext->nAcceptMac, peerMac, NULL))
        return VOS_STATUS_SUCCESS;

    if (sapSearchMacList(sapContext->denyMacList, sapContext->nDenyMac, peerMac, NULL))
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  "In %s, Peer "MAC_ADDRESS_STR" in deny list",
                  __func__, MAC_ADDR_ARRAY(peerMac));
        return VOS_STATUS_E_FAILURE;
    }

    // A new station CAN associate, unless in deny list. Less stringent mode
    if (eSAP_ACCEPT_UNLESS_DENIED == sapContext->eSapMacAddrAclMode)
        return VOS_STATUS_SUCCESS;

    // A new station CANNOT associate, unless in accept list. More stringent mode
    if (eSAP_DENY_UNLESS_ACCEPTED == sapContext->eSapMacAddrAclMode)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  "In %s, Peer "MAC_ADDRESS_STR" denied, Mac filter mode is eSAP_DENY_UNLESS_ACCEPTED",
                  __func__,  MAC_ADDR_ARRAY(peerMac));
        return VOS_STATUS_E_FAILURE;
    }

    /* The new STA is neither in accept list nor in deny list. In this case, deny the association
     * but send a wifi event notification indicating the mac address being denied
     */
    if (eSAP_SUPPORT_ACCEPT_AND_DENY == sapContext->eSapMacAddrAclMode)
    {
        sapSignalHDDevent(sapContext, NULL, eSAP_UNKNOWN_STA_JOIN, (v_PVOID_t)peerMac);
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_HIGH,
                  "In %s, Peer "MAC_ADDRESS_STR" denied, Mac filter mode is eSAP_SUPPORT_ACCEPT_AND_DENY",
                  __func__, MAC_ADDR_ARRAY(peerMac));
        return VOS_STATUS_E_FAILURE;
    }
    return VOS_STATUS_SUCCESS;
}

#ifdef SOFTAP_CHANNEL_RANGE
static VOS_STATUS sapGetChannelList(ptSapContext sapContext,
                                 v_U8_t **channelList, v_U8_t *numberOfChannels)
{
    v_U8_t  loopCount;
    v_U8_t *list;
    v_U8_t channelCount;
    v_U8_t startChannelNum, bandStartChannel;
    v_U8_t endChannelNum, bandEndChannel ;
    v_U32_t enableLTECoex;
    tHalHandle hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
#ifdef FEATURE_WLAN_CH_AVOID
    v_U8_t i;
#endif
    tpAniSirGlobal pmac = PMAC_STRUCT(hHal);

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
             "Invalid HAL pointer from pvosGCtx on sapGetChannelList");
        *numberOfChannels = 0;
        *channelList = NULL;
        return VOS_STATUS_E_FAULT;
    }

    if ( eCSR_BAND_ALL == sapContext->scanBandPreference)
    {
        startChannelNum = sapContext->acs_cfg->start_ch;
        endChannelNum = sapContext->acs_cfg->end_ch;
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
             "%s: startChannel %d, EndChannel %d, HW:%d",
             __func__, startChannelNum, endChannelNum,
             sapContext->acs_cfg->hw_mode);

        WLANSAP_extend_to_acs_range(&startChannelNum, &endChannelNum,
                            &bandStartChannel, &bandEndChannel);

        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO,
             "%s: expanded startChannel %d,EndChannel %d",
             __func__,startChannelNum,endChannelNum);
    }
    else
    {
        if ( sapContext->allBandScanned == eSAP_FALSE )
        {
            //first band scan
            sapContext->currentPreferredBand = sapContext->scanBandPreference;
        }
        else
        {
            //scan next band
            if ( eCSR_BAND_24 == sapContext->scanBandPreference )
                sapContext->currentPreferredBand = eCSR_BAND_5G;
            else
                sapContext->currentPreferredBand = eCSR_BAND_24;
        }
        switch(sapContext->currentPreferredBand)
        {
            case eCSR_BAND_24:
               bandStartChannel = RF_CHAN_1;
               bandEndChannel = RF_CHAN_14;
               startChannelNum = 1;
               endChannelNum = 14;
               break;

            case eCSR_BAND_5G:
               bandStartChannel = RF_CHAN_36;
               bandEndChannel = RF_CHAN_165;
               startChannelNum = 36;
               endChannelNum = 165;
               break;

            default:
               VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                 "sapGetChannelList:bandPreference not valid ");
               /* assume 2.4 GHz */
               bandStartChannel = RF_CHAN_1;
               bandEndChannel = RF_CHAN_14;
               startChannelNum = 1;
               endChannelNum = 14;
               break;
        }
    }

    ccmCfgGetInt(hHal, WNI_CFG_ENABLE_LTE_COEX, &enableLTECoex);

    /*Check if LTE coex is enabled and 2.4GHz is selected*/
    if (enableLTECoex && (bandStartChannel == RF_CHAN_1)
       && (bandEndChannel == RF_CHAN_14))
    {
        /*Set 2.4GHz upper limit to channel 9 for LTE COEX*/
        bandEndChannel = RF_CHAN_9;
    }
    /* Allocate the max number of channel supported */
    list = (v_U8_t *)vos_mem_malloc(NUM_5GHZ_CHANNELS);
    if (NULL == list)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "%s: Unable to allocate channel list", __func__);
        *numberOfChannels = 0;
        *channelList = NULL;
        return VOS_STATUS_E_RESOURCES;
    }

    /*Search for the Active channels in the given range */
    channelCount = 0;
    for( loopCount = bandStartChannel; loopCount <= bandEndChannel; loopCount++ )
    {
        if((startChannelNum <= rfChannels[loopCount].channelNum)&&
            (endChannelNum >= rfChannels[loopCount].channelNum ))
        {
            if (((TRUE == pmac->scan.fEnableDFSChnlScan) &&
                 (regChannels[loopCount].enabled)) ||
                ((FALSE == pmac->scan.fEnableDFSChnlScan) &&
                 (NV_CHANNEL_ENABLE == regChannels[loopCount].enabled)))
            {
#ifdef FEATURE_WLAN_CH_AVOID
                for( i = 0; i < NUM_20MHZ_RF_CHANNELS; i++ )
                {
                    if( (safeChannels[i].channelNumber ==
                                rfChannels[loopCount].channelNum) )
                    {
                        /* Check if channel is safe */
                        if(VOS_TRUE == safeChannels[i].isSafe)
                        {
#endif
#ifdef FEATURE_WLAN_AP_AP_ACS_OPTIMIZE
                            v_U8_t ch;
                            ch = rfChannels[loopCount].channelNum;
                            if ((sapContext->acs_cfg->skip_scan_status
                                == eSAP_DO_PAR_ACS_SCAN)) {
                                if ((ch >= sapContext->acs_cfg->skip_scan_range1_stch &&
                                ch <= sapContext->acs_cfg->skip_scan_range1_endch) ||
                                (ch >= sapContext->acs_cfg->skip_scan_range2_stch &&
                                ch <= sapContext->acs_cfg->skip_scan_range2_endch)) {

                                    list[channelCount] =
                                        rfChannels[loopCount].channelNum;
                                    channelCount++;
                                    VOS_TRACE( VOS_MODULE_ID_SAP,
                                        VOS_TRACE_LEVEL_INFO,
                                        "%s:%d %d added to ACS ch range",
                                        __func__, channelCount, ch);
                                } else
                                    VOS_TRACE( VOS_MODULE_ID_SAP,
                                        VOS_TRACE_LEVEL_INFO_HIGH,
                                        "%s:%d %d skipped from ACS ch range",
                                        __func__, channelCount, ch);

                            } else {
                                    list[channelCount] =
                                        rfChannels[loopCount].channelNum;
                                    channelCount++;
                                    VOS_TRACE( VOS_MODULE_ID_SAP,
                                        VOS_TRACE_LEVEL_INFO,
                                        "%s:%d %d added to ACS ch range",
                                        __func__, channelCount, ch);
                            }
#else
                            list[channelCount] =
                                    rfChannels[loopCount].channelNum;
                            channelCount++;
#endif
#ifdef FEATURE_WLAN_CH_AVOID
                        }
                        break;
                    }
                }
#endif
            }
        }
    }
    if (0 == channelCount)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
        "sapGetChannelList:No active channels present in the given range for the current region");
        /*LTE COEX: channel range outside the restricted 2.4GHz band limits*/
        if (enableLTECoex && (startChannelNum > bandEndChannel))
        {
            VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_FATAL,
            "sapGetChannelList:SAP cannot be started as LTE COEX restricted 2.4GHz limits");
        }
    }

    /* return the channel list and number of channels to scan*/
    *numberOfChannels = channelCount;
    if(channelCount != 0)
    {
       *channelList = list;
    }
    else
    {
       *channelList = NULL;
        vos_mem_free(list);
    }

    for (loopCount = 0; loopCount <channelCount; loopCount ++ )
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_DEBUG,
             "%s: channel number: %d",
             __func__,list[loopCount]);
    }
    return VOS_STATUS_SUCCESS;
}
#endif

/*
 * Function for initializing list of  2.4/5 Ghz [NON-DFS/DFS]
 * available channels in the current regulatory domain.
 */
static VOS_STATUS sapGet5GHzChannelList(ptSapContext sapContext)
{
    v_U8_t count = 0;
    int i;
    if (NULL == sapContext)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
              "Invalid sapContext pointer on sapGetChannelList");
        return VOS_STATUS_E_FAULT;
    }

    if (sapContext->SapAllChnlList.channelList) {
        vos_mem_free(sapContext->SapAllChnlList.channelList);
        sapContext->SapAllChnlList.channelList = NULL;
    }

    sapContext->SapAllChnlList.channelList =
                (tChannelInfo *)vos_mem_malloc(WNI_CFG_VALID_CHANNEL_LIST_LEN *
                                               sizeof(tChannelInfo));
    if (NULL == sapContext->SapAllChnlList.channelList)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   " Memory Allocation failed sapGetChannelList");
                 return VOS_STATUS_E_FAULT;
    }

    for( i = RF_CHAN_36; i <= RF_CHAN_165; i++ )
    {
        if( regChannels[i].enabled == NV_CHANNEL_ENABLE ||
            regChannels[i].enabled == NV_CHANNEL_DFS )
        {
            sapContext->SapAllChnlList.channelList[count].channel =
                                          rfChannels[i].channelNum;
            VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                      "%s[%d] CHANNEL = %d",__func__, __LINE__,
                      sapContext->SapAllChnlList.channelList[count].channel);
            sapContext->SapAllChnlList.channelList[count].valid = VOS_TRUE;
            count++;
        }
    }

    sapContext->SapAllChnlList.numChannel = count;
    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
              "%s[%d] NUMBER OF CHANNELS count = %d"
              "sapContext->SapAllChnlList.numChannel = %d",
              __func__,__LINE__,count,sapContext->SapAllChnlList.numChannel);
    return VOS_STATUS_SUCCESS;
}

/*
 * This function randomly selects the channel to switch after the detection
 * of radar
 * param sapContext - sap context
 * dfs_event - Dfs information from DFS
 * return - channel to which AP wishes to switch
 */
v_U8_t sapIndicateRadar(ptSapContext sapContext, tSirSmeDfsEventInd *dfs_event)
{
    v_U8_t target_channel = 0;
    tHalHandle hHal;
    tpAniSirGlobal pMac;

    if (NULL == sapContext || NULL == dfs_event)
    {
        /* Invalid sap context of dfs event passed */
        return 0;
    }
    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return 0;
    }
    pMac = PMAC_STRUCT( hHal );

    if (!dfs_event->dfs_radar_status)
    {
        /*dfs status does not indicate a radar on the channel-- False Alarm*/
        return 0;
    }

    /*
     * SAP needs to generate Channel Switch IE
     * if the radar is found in the STARTED state
     */
    if (eSAP_STARTED == sapContext->sapsMachine)
        pMac->sap.SapDfsInfo.csaIERequired = VOS_TRUE;

    if (sapContext->csrRoamProfile.disableDFSChSwitch)
    {
       return sapContext->channel;
    }

    /* set the Radar Found flag in SapDfsInfo */
    pMac->sap.SapDfsInfo.sap_radar_found_status = VOS_TRUE;

    sapGet5GHzChannelList(sapContext);

    if (dfs_event->chan_list.nchannels > SIR_DFS_MAX_20M_SUB_CH) {
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
              FL("nchannels >SIR_DFS_MAX_20M_SUB_CH so resetting"));
        dfs_event->chan_list.nchannels = SIR_DFS_MAX_20M_SUB_CH;
    }

    sapMarkDfsChannels(sapContext, dfs_event->chan_list.channels,
          dfs_event->chan_list.nchannels, vos_get_monotonic_boottime());

    /*
     * (1) skip static turbo channel as it will require STA to be in
     * static turbo to work.
     * (2) skip channel which's marked with radar detction
     * (3) WAR: we allow user to config not to use any DFS channel
     * (4) When we pick a channel, skip excluded 11D channels
     * (5) Create the available channel list with the above rules
     */

    target_channel = sapRandomChannelSel(sapContext);
    if (0 == target_channel)
    {
        sapSignalHDDevent(sapContext, NULL, eSAP_DFS_NO_AVAILABLE_CHANNEL,
                          (v_PVOID_t) eSAP_STATUS_SUCCESS);
    }
    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_WARN,
              FL("sapdfs: New selected target channel is [%d]"),
              target_channel);
    return target_channel;
}

/*
 * CAC timer callback function.
 * Post eSAP_DFS_CHANNEL_CAC_END event to sapFsm().
 */
void sapDfsCacTimerCallback(void *data)
{
    ptSapContext sapContext;
    tWLAN_SAPEvent sapEvent;
    tHalHandle hHal = (tHalHandle)data;
    tpAniSirGlobal pMac;

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return;
    }
    pMac = PMAC_STRUCT( hHal );
    sapContext = sap_find_valid_concurrent_session(hHal);

    if (NULL == sapContext)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s no SAP contexts are found", __func__);
        return;
    }

    /*
     * SAP may not be in CAC wait state, when the timer runs out.
     * if following flag is set, then timer is in initialized state,
     * destroy timer here.
     */
    if (pMac->sap.SapDfsInfo.is_dfs_cac_timer_running == true) {
        vos_timer_destroy(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);
        pMac->sap.SapDfsInfo.is_dfs_cac_timer_running = false;
    }
    /* Check to ensure that SAP is in DFS WAIT state*/
    if (sapContext->sapsMachine == eSAP_DFS_CAC_WAIT)
    {
        /*
         * CAC Complete, post eSAP_DFS_CHANNEL_CAC_END to sapFsm
         */
        VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
        "sapdfs: Sending eSAP_DFS_CHANNEL_CAC_END for target_channel = %d on sapctx[%pK]",
        sapContext->channel, sapContext);

        sapEvent.event = eSAP_DFS_CHANNEL_CAC_END;
        sapEvent.params = 0;
        sapEvent.u1 = 0;
        sapEvent.u2 = 0;

        sapFsm(sapContext, &sapEvent);
    }

}

/*
 * Function to stop the DFS CAC Timer
 */
static int sapStopDfsCacTimer(ptSapContext sapContext)
{
    tHalHandle hHal;
    tpAniSirGlobal pMac;
    if (sapContext == NULL)
        return 0;

    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);
    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return 0;
    }
    pMac = PMAC_STRUCT(hHal);

    if (VOS_TIMER_STATE_RUNNING !=
            vos_timer_getCurrentState(
                &pMac->sap.SapDfsInfo.sap_dfs_cac_timer)) {
        return 0;
    }

    vos_timer_stop(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);
    pMac->sap.SapDfsInfo.is_dfs_cac_timer_running = 0;
    vos_timer_destroy(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);

    return 0;
}

/**
 * sap_is_channel_bonding_etsi_weather_channel() - Routine to check if
 *                                                 sap channel is bonded to
 *                                                 weather radar channel.
 * @sap_context:                                   SAP context
 *
 * Check if the current SAP operating channel is bonded to weather radar
 * channel in ETSI domain.
 *
 * Return: True if bonded to weather channel in ETSI
 */
static bool
sap_is_channel_bonding_etsi_weather_channel(ptSapContext sap_context)
{
    if(IS_CH_BONDING_WITH_WEATHER_CH(sap_context->channel) &&
      (sap_context->vht_channel_width != eHT_CHANNEL_WIDTH_20MHZ)) {
        return true;
    }
    return false;
}

/*
 * Function to start the DFS CAC Timer
 * when SAP is started on a DFS channel
 */
int sapStartDfsCacTimer(ptSapContext sapContext)
{
    VOS_STATUS status;
    v_U32_t cacTimeOut;
    tHalHandle hHal = NULL;
    tpAniSirGlobal pMac = NULL;
    uint8_t dfs_region;

    if (sapContext == NULL)
    {
        return 0;
    }
    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return 0;
    }
    pMac = PMAC_STRUCT( hHal );

    if (pMac->sap.SapDfsInfo.ignore_cac)
    {
        /*
         * If User has set to ignore the CAC
         * so, continue without CAC Timer.
         */
        return 2;
    }
    cacTimeOut = DEFAULT_CAC_TIMEOUT;

    vos_nv_get_dfs_region(&dfs_region);

    if ((dfs_region == DFS_ETSI_DOMAIN) &&
       ((IS_ETSI_WEATHER_CH(sapContext->channel)) ||
       (sap_is_channel_bonding_etsi_weather_channel(sapContext))))
    {
        cacTimeOut = ETSI_WEATHER_CH_CAC_TIMEOUT;
    }
    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_MED,
              "sapdfs: SAP_DFS_CHANNEL_CAC_START on CH - %d, CAC TIMEOUT - %d sec",
              sapContext->channel, cacTimeOut/1000);

    vos_timer_init(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer,
                   VOS_TIMER_TYPE_SW,
                   sapDfsCacTimerCallback, (v_PVOID_t)hHal);

    /*Start the CAC timer*/
    status = vos_timer_start(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer, cacTimeOut);
    if (status == VOS_STATUS_SUCCESS)
    {
        pMac->sap.SapDfsInfo.is_dfs_cac_timer_running = true;
        return 1;
    }
    else
    {
        pMac->sap.SapDfsInfo.is_dfs_cac_timer_running = false;
        vos_timer_destroy(&pMac->sap.SapDfsInfo.sap_dfs_cac_timer);
        return 0;
    }
}

/*
 * This function initializes the NOL list
 * parameters required to track the radar
 * found DFS channels in the current Reg. Domain .
 */
VOS_STATUS sapInitDfsChannelNolList(ptSapContext sapContext)
{
    v_U8_t count = 0;
    int i;
    v_BOOL_t bFound = FALSE;
    tHalHandle hHal;
    tpAniSirGlobal pMac;

    if (NULL == sapContext)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
              "Invalid sapContext pointer on sapInitDfsChannelNolList");
        return VOS_STATUS_E_FAULT;
    }
    hHal = VOS_GET_HAL_CB(sapContext->pvosGCtx);

    if (NULL == hHal)
    {
        VOS_TRACE( VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                   "In %s invalid hHal", __func__);
        return VOS_STATUS_E_FAULT;
    }
    pMac = PMAC_STRUCT( hHal );

    /* to indicate hdd to get cnss dfs nol */
    if (VOS_STATUS_SUCCESS == sapSignalHDDevent(sapContext, NULL,
                                  eSAP_DFS_NOL_GET,
                                  (v_PVOID_t) eSAP_STATUS_SUCCESS))
    {
        bFound = TRUE;
    }

    for ( i = RF_CHAN_36; i <= RF_CHAN_165; i++ )
    {
        if ( regChannels[i].enabled == NV_CHANNEL_DFS )
        {
            /* if dfs nol is not found, initialize it */
            if (!bFound)
            {
                pMac->sap.SapDfsInfo.sapDfsChannelNolList[count]
                   .dfs_channel_number = rfChannels[i].channelNum;

                VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
                       "%s: CHANNEL = %d", __func__,
                       pMac->sap.SapDfsInfo
                       .sapDfsChannelNolList[count].dfs_channel_number);

                pMac->sap.SapDfsInfo.sapDfsChannelNolList[count]
                   .radar_status_flag = eSAP_DFS_CHANNEL_USABLE;
                pMac->sap.SapDfsInfo.sapDfsChannelNolList[count]
                   .radar_found_timestamp = 0;
            }
            count++;
        }
    }

    pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels = count;

    VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_INFO_LOW,
              "%s[%d] NUMBER OF DFS CHANNELS = %d",
              __func__, __LINE__,
              pMac->sap.SapDfsInfo.numCurrentRegDomainDfsChannels);

    return VOS_STATUS_SUCCESS;
}

/*
 * This function will calculate how many interfaces
 * have sap persona and returns total number of sap persona.
 */
v_U8_t sap_get_total_number_sap_intf(tHalHandle hHal)
{
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
    v_U8_t intf = 0;
    v_U8_t intf_count = 0;

    for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++) {
         if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
             (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
             pMac->sap.sapCtxList[intf].pSapContext != NULL) {
             intf_count++;
         }
    }
    return intf_count;
}

/*
 * This function will find the concurrent sap context apart from
 * passed sap context and return its channel change ready status
 */
tANI_BOOLEAN is_concurrent_sap_ready_for_channel_change(tHalHandle hHal,
                                       ptSapContext sapContext)
{
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);
    ptSapContext pSapContext;
    v_U8_t intf = 0;
    tANI_BOOLEAN is_ready_for_chng = VOS_TRUE;

    for (intf = 0; intf < SAP_MAX_NUM_SESSION; intf++) {
         if (((VOS_STA_SAP_MODE == pMac->sap.sapCtxList [intf].sapPersona) ||
             (VOS_P2P_GO_MODE == pMac->sap.sapCtxList [intf].sapPersona)) &&
             pMac->sap.sapCtxList[intf].pSapContext != NULL) {
             pSapContext =
                    (ptSapContext)pMac->sap.sapCtxList [intf].pSapContext;
             if (pSapContext == sapContext) {
                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                           FL("sapCtx matched [%pK]"), sapContext);
                 continue;
             } else {
                 VOS_TRACE(VOS_MODULE_ID_SAP, VOS_TRACE_LEVEL_ERROR,
                           FL("concurrent sapCtx[%pK] didn't matche with [%pK]"),
                           pSapContext, sapContext);
                 is_ready_for_chng = is_ready_for_chng &&
                     pSapContext->is_sap_ready_for_chnl_chng;
             }
         }
    }
    return is_ready_for_chng;
}
