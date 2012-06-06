/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef RESMEM_ACCOUNT_H
#define RESMEM_ACCOUNT_H

struct reserved_memory_account {
	struct list_head link;
	const char *name;
	void *data;
	size_t (*get_total_pages) (struct reserved_memory_account *m);
	size_t (*get_page_used_by_process) (struct task_struct *p, struct reserved_memory_account *m);
};

void register_reserved_memory_account(struct reserved_memory_account *handler);
void unregister_reserved_memory_account(struct reserved_memory_account *handler);

#endif
