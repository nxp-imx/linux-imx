/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
* @file    diagnostic.h
*
* @brief   Macros for outputting kernel and user space diagnostics.
*/

#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#ifndef __KERNEL__		/* linux flag */
#include <stdio.h>
#endif
#include "fsl_platform.h"

#if defined(FSL_HAVE_SAHARA2) || defined(FSL_HAVE_SAHARA4)
#define DEV_NAME "sahara"
#elif defined(FSL_HAVE_RNGA) || defined(FSL_HAVE_RNGB) ||                     \
      defined(FSL_HAVE_RNGC)
#define DEV_NAME "shw"
#endif

/*!
********************************************************************
* @brief   This macro logs diagnostic messages to stderr.
*
* @param   diag  String that must be logged, char *.
*
* @return   void
*
*/
//#if defined DIAG_SECURITY_FUNC || defined DIAG_ADAPTOR
#define LOG_DIAG(diag)                                              \
({                                                                  \
    const char* fname = strrchr(__FILE__, '/');                           \
                                                                    \
     sah_Log_Diag(fname ? fname+1 : __FILE__, __LINE__, diag);     \
})

#ifdef __KERNEL__

#define LOG_DIAG_ARGS(fmt, ...)                                               \
({                                                                            \
    const char* fname = strrchr(__FILE__, '/');                               \
    os_printk(KERN_ALERT "%s:%i: " fmt "\n",                                  \
              fname ? fname+1 : __FILE__,                                     \
              __LINE__,                                                       \
              __VA_ARGS__);                                                   \
})

#else

#define LOG_DIAG_ARGS(fmt, ...)                                               \
({                                                                            \
    const char* fname = strrchr(__FILE__, '/');                               \
    printf("%s:%i: " fmt "\n",                                                \
           fname ? fname+1 : __FILE__,                                        \
           __LINE__,                                                          \
           __VA_ARGS__);                                                      \
})

#ifndef __KERNEL__
void sah_Log_Diag(char *source_name, int source_line, char *diag);
#endif
#endif /* if define DIAG_SECURITY_FUNC ... */

#ifdef __KERNEL__
/*!
********************************************************************
* @brief   This macro logs kernel diagnostic  messages to the kernel
*          log.
*
* @param   diag  String that must be logged, char *.
*
* @return  As for printf()
*/
#if 0
#if defined(DIAG_DRV_IF) || defined(DIAG_DRV_QUEUE) ||                        \
  defined(DIAG_DRV_STATUS) || defined(DIAG_DRV_INTERRUPT) ||                  \
   defined(DIAG_MEM) || defined(DIAG_SECURITY_FUNC) || defined(DIAG_ADAPTOR)
#endif
#endif

#define LOG_KDIAG_ARGS(fmt, ...)                                              \
({                                                                            \
    os_printk (KERN_ALERT "%s (%s:%i): " fmt "\n",                            \
               DEV_NAME, strrchr(__FILE__, '/')+1, __LINE__, __VA_ARGS__);              \
})

#define LOG_KDIAG(diag)                                                       \
    os_printk (KERN_ALERT "%s (%s:%i): %s\n",                             \
               DEV_NAME, strrchr(__FILE__, '/')+1, __LINE__, diag);

#define sah_Log_Diag(n, l, d)                                                 \
    os_printk(KERN_ALERT "%s:%i: %s\n", n, l, d)

#else				/* not KERNEL */

#define sah_Log_Diag(n, l, d)                                                 \
    printf("%s:%i: %s\n", n, l, d)

#endif				/* __KERNEL__ */

#endif				/* DIAGNOSTIC_H */
