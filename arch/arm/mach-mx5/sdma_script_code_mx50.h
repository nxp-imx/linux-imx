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

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file sdma_script_code.h
 * @brief This file contains functions of SDMA scripts code initialization
 *
 * The file was generated automatically. Based on sdma scripts library.
 *
 * @ingroup SDMA
 */
/*******************************************************************************

		SDMA RELEASE LABEL:     "SDMA_CODEX.01.00.00"

*******************************************************************************/

#ifndef SDMA_SCRIPT_CODE_MX50_H
#define SDMA_SCRIPT_CODE_MX50_H


/*!
* SDMA ROM scripts start addresses and sizes
*/

#define start_ADDR_MX50   0
#define start_SIZE_MX50   18

#define core_ADDR_MX50   80
#define core_SIZE_MX50   232

#define common_ADDR_MX50   312
#define common_SIZE_MX50   330

#define ap_2_ap_ADDR_MX50   642
#define ap_2_ap_SIZE_MX50   41

#define app_2_mcu_ADDR_MX50   683
#define app_2_mcu_SIZE_MX50   64

#define mcu_2_app_ADDR_MX50   747
#define mcu_2_app_SIZE_MX50   70

#define uart_2_mcu_ADDR_MX50   817
#define uart_2_mcu_SIZE_MX50   74

#define shp_2_mcu_ADDR_MX50   891
#define shp_2_mcu_SIZE_MX50   69

#define mcu_2_shp_ADDR_MX50   960
#define mcu_2_shp_SIZE_MX50   72

#define uartsh_2_mcu_ADDR_MX50   1032
#define uartsh_2_mcu_SIZE_MX50   68

#define loop_DMAs_routines_ADDR_MX50   1100
#define loop_DMAs_routines_SIZE_MX50   227

#define test_ADDR_MX50   1327
#define test_SIZE_MX50   63

#define signature_ADDR_MX50   1023
#define signature_SIZE_MX50   1

/*!
* SDMA RAM scripts start addresses and sizes
*/

#define mcu_2_ssiapp_ADDR_MX50   6144
#define mcu_2_ssiapp_SIZE_MX50   96

#define mcu_2_ssish_ADDR_MX50   6240
#define mcu_2_ssish_SIZE_MX50   95

/*!
* SDMA RAM image start address and size
*/

#define RAM_CODE_START_ADDR_MX50     6144
#define RAM_CODE_SIZE_MX50           191

/*!
* Buffer that holds the SDMA RAM image
*/
__attribute__ ((__aligned__(4)))
static const short sdma_code_mx50[] = {
0xc1e3, 0x57db, 0x52f3, 0x6a01, 0x008f, 0x00d5, 0x7d01, 0x008d,
0x05a0, 0x5deb, 0x0478, 0x7d03, 0x0479, 0x7d2c, 0x7c36, 0x0479,
0x7c1f, 0x56ee, 0x0f00, 0x0660, 0x7d05, 0x6509, 0x7e43, 0x620a,
0x7e41, 0x981e, 0x620a, 0x7e3e, 0x6509, 0x7e3c, 0x0512, 0x0512,
0x02ad, 0x0760, 0x7d03, 0x55fb, 0x6dd3, 0x9829, 0x55fb, 0x1d04,
0x6dd3, 0x6ac8, 0x7f2f, 0x1f01, 0x2003, 0x4800, 0x7ce4, 0x9851,
0x55fb, 0x6dd7, 0x0015, 0x7805, 0x6209, 0x6ac8, 0x6209, 0x6ac8,
0x6dd7, 0x9850, 0x55fb, 0x6dd7, 0x0015, 0x0015, 0x7805, 0x620a,
0x6ac8, 0x620a, 0x6ac8, 0x6dd7, 0x9850, 0x55fb, 0x6dd7, 0x0015,
0x0015, 0x0015, 0x7805, 0x620b, 0x6ac8, 0x620b, 0x6ac8, 0x6dd7,
0x7c09, 0x6ddf, 0x7f07, 0x0000, 0x55eb, 0x4d00, 0x7d07, 0xc1fa,
0x57db, 0x9804, 0x0007, 0x68cc, 0x680c, 0xc213, 0xc20a, 0x9801,
0xc1d9, 0xc1e3, 0x57db, 0x52f3, 0x6a21, 0x008f, 0x00d5, 0x7d01,
0x008d, 0x05a0, 0x5deb, 0x56fb, 0x0478, 0x7d03, 0x0479, 0x7d32,
0x7c39, 0x0479, 0x7c28, 0x0b70, 0x0311, 0x53eb, 0x0f00, 0x0360,
0x7d05, 0x6509, 0x7e3f, 0x620a, 0x7e3d, 0x9882, 0x620a, 0x7e3a,
0x6509, 0x7e38, 0x0512, 0x0512, 0x02ad, 0x0760, 0x7d0a, 0x5a06,
0x7f31, 0x1f01, 0x2003, 0x4800, 0x7cea, 0x0b70, 0x0311, 0x5313,
0x98b3, 0x5a26, 0x7f27, 0x1f01, 0x2003, 0x4800, 0x7ce0, 0x0b70,
0x0311, 0x5313, 0x98b3, 0x0015, 0x7804, 0x6209, 0x5a06, 0x6209,
0x5a26, 0x98b2, 0x0015, 0x0015, 0x7804, 0x620a, 0x5a06, 0x620a,
0x5a26, 0x98b2, 0x0015, 0x0015, 0x0015, 0x7804, 0x620b, 0x5a06,
0x620b, 0x5a26, 0x7c07, 0x0000, 0x55eb, 0x4d00, 0x7d06, 0xc1fa,
0x57db, 0x9865, 0x0007, 0x680c, 0xc213, 0xc20a, 0x9862
};
#endif
