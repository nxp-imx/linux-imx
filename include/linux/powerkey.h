/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef POWER_KEY_H
#define POWER_KEY_H

typedef void (*key_press_call_back)(void *);
typedef void (*register_key_press_call_back)(key_press_call_back, void *);

struct power_key_platform_data {
	register_key_press_call_back register_key_press_handler;
	void *param; /* param of the handler */
};

#endif /* POWER_KEY_H */
