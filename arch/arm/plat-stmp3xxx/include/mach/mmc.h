#ifndef _MACH_MMC_H
#define _MACH_MMC_H

#include <mach/regs-ssp.h>

struct stmp3xxxmmc_platform_data {
	int (*hw_init)(void);
	void (*hw_release)(void);
	void (*cmd_pullup)(int enable);
	int (*get_wp)(void);
	unsigned long (*setclock)(unsigned long hz);
	int read_uA;
	int write_uA;
};


extern unsigned long stmp3xxxmmc_setclock_ssp1(unsigned long hz);
extern void stmp3xxxmmc_cmd_pullup_ssp1(int enable);
extern void stmp3xxxmmc_hw_release_ssp1(void);
extern int stmp3xxxmmc_hw_init_ssp1(void);
extern int stmp3xxxmmc_get_wp(void);

#endif
