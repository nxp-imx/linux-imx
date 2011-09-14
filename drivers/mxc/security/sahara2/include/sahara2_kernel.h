/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#define DRIVER_NAME sahara2

#define SAHARA_MAJOR_NODE 78

#include "portable_os.h"

#include "platform_abstractions.h"

/* Forward-declare prototypes using signature macros */

OS_DEV_ISR_DCL(sahara2_isr);

OS_DEV_INIT_DCL(sahara2_init);

OS_DEV_SHUTDOWN_DCL(sahara2_shutdown);

OS_DEV_OPEN_DCL(sahara2_open);

OS_DEV_CLOSE_DCL(sahara2_release);

OS_DEV_IOCTL_DCL(sahara2_ioctl);

struct sahara2_kernel_user {
	void *command_ring[32];
};

struct sahara2_sym_arg {
	char *key;
	unsigned key_len;
};

/*! These need to be added to Linux / OS abstractions */
/*
module_init(OS_DEV_INIT_REF(sahara2_init));
module_cleanup(OS_DEV_SHUTDOWN_REF(sahara2_shutdown));
*/
