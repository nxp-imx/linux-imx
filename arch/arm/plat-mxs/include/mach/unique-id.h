/*
 * Unique ID interface for ID storage providers
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __UNIQUE_ID_H
#define __UNIQUE_ID_H

struct uid_ops {
	ssize_t (*id_show)(void *context, char *page, int ascii);
	ssize_t (*id_store)(void *context, const char *page,
			size_t count, int ascii);
};

struct kobject *uid_provider_init(const char *name,
		struct uid_ops *ops, void *context);
void uid_provider_remove(const char *name);
#endif
