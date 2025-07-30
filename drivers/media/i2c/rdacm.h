// SPDX-License-Identifier: GPL-2.0+
/*
 * IMI RDACM GMSL Camera Driver
 *
 * Copyright 2025 NXP
 */
#ifndef RDACM_H
#define RDACM_H


/* The RDACM state transitions
 *
 *	1. Linux boot (two possible HW states):
 *		  - low ampl (HW reset)
 *		  - high ampl (only OS reset)
 *
 *	2. RDACM_CORE_CMD_INIT_HIGH_AMPL:
 *		  - MAX9271 low ampl -> high ampl communication
 *
 *	3. RDACM_CORE_CMD_INIT_SERIALIZER:
 *		  - MAX9271 GSML link and address translation
 *
 *	4. RDACM_CORE_CMD_INIT_CAMERA:
 *		  -	OV10635 configured
 * 
 *	5. RDACM_CORE_CMD_S_STREAM:
 *		  - OV10635 streaming
 */

/* These ioctls are internal to the kernel */
#define RDACM_CORE_CMD_INIT_HIGH_AMPL		_IO('R', 1)
#define RDACM_CORE_CMD_INIT_SERIALIZER		_IO('R', 2)
#define RDACM_CORE_CMD_INIT_CAMERA	        _IO('R', 3)
#define RDACM_CORE_CMD_S_STREAM		        _IO('R', 4)

#endif /* RDACM_H */
