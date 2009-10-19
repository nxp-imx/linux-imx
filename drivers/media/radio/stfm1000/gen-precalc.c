/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/* generate precalculated tables */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "stfm1000-regs.h"

static void generate_tune1(void)
{
	int start, end;
	int ndiv;           // N Divider in PLL
	int incr;           // Increment in PLL
	int cicosr;         // CIC oversampling ratio
	int sdnominal;      // value to serve pilot/interpolator loop in SD
	int i, temp;        // used in tuning table construction

	start = STFM1000_FREQUENCY_100KHZ_MIN;
	end = start + STFM1000_FREQUENCY_100KHZ_RANGE;

	printf("const struct stfm1000_tune1\n"
		"stfm1000_tune1_table[STFM1000_FREQUENCY_100KHZ_RANGE] = {\n");

	for (i = start; i < end; i++) {

		ndiv = (int)((i+14)/15) - 48;
		incr = i - (int)(i/15)*15;
		cicosr = (int)(i*2/3.0/16.0 + 0.5);
		sdnominal = (int)(i*100.0e3/1.5/(double)cicosr/2.0/2.0*2.0*8.0*256.0/228.0e3*65536);

		temp = 0x00000000;                                    // clear
		temp = temp | ((cicosr<<9) & STFM1000_TUNE1_CICOSR);  // bits[14:9] 0x00007E00
		temp = temp | ((ndiv<<4)   & STFM1000_TUNE1_PLL_DIV); // bits[8:4]  0x000001F0
		temp = temp | ((incr)      & STFM1000_TUNE1_PLL_DIV); // bits[3:0]  0x0000000F

		printf("\t[%d - STFM1000_FREQUENCY_100KHZ_MIN] = "
			"{ .tune1 = 0x%08x, .sdnom = 0x%08x },\n",
			i, temp, sdnominal);
	}
	printf("};\n");

}

int main(int argc, char *argv[])
{
	printf("#include \"stfm1000-regs.h\"\n\n");

	generate_tune1();

	return 0;
}
