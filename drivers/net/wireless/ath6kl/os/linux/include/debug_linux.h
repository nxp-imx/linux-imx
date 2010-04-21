/*
 * Copyright (c) 2004-2006 Atheros Communications Inc.
 * All rights reserved.
 *
 * 
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
 *
 */

#ifndef _DEBUG_LINUX_H_
#define _DEBUG_LINUX_H_

    /* macro to remove parens */
#define ATH_PRINTX_ARG(arg...) arg

#ifdef DEBUG
    /* NOTE: the AR_DEBUG_PRINTF macro is defined here to handle special handling of variable arg macros
     * which may be compiler dependent. */
#define AR_DEBUG_PRINTF(mask, args) do {        \
    if (GET_ATH_MODULE_DEBUG_VAR_MASK(ATH_MODULE_NAME) & (mask)) {                    \
        A_PRINTF(ATH_PRINTX_ARG args);    \
    }                                            \
} while (0)
#else
    /* on non-debug builds, keep in error and warning messages in the driver, all other
     * message tracing will get compiled out */
#define AR_DEBUG_PRINTF(mask, args) \
    if ((mask) & (ATH_DEBUG_ERR | ATH_DEBUG_WARN)) { A_PRINTF(ATH_PRINTX_ARG args); }

#endif

    /* compile specific macro to get the function name string */
#define _A_FUNCNAME_  __func__


#endif /* _DEBUG_LINUX_H_ */
