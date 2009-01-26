/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


#ifndef __DRYICE_REGS_H__
#define __DRYICE_REGS_H__

/***********************************************************************
 * DryIce Register Definitions
 ***********************************************************************/

/* DryIce Time Counter MSB Reg */
#define DTCMR   0x00

/* DryIce Time Counter LSB Reg */
#define DTCLR   0x04

/* DryIce Clock Alarm MSB Reg */
#define DCAMR   0x08

/* DryIce Clock Alarm LSB Reg */
#define DCALR   0x0c

/* DryIce Control Reg */
#define DCR     0x10
#define DCR_TDCHL   (1 << 30)  /* Tamper Detect Config Hard Lock */
#define DCR_TDCSL   (1 << 29)  /* Tamper Detect COnfig Soft Lock */
#define DCR_KSHL    (1 << 28)  /* Key Select Hard Lock */
#define DCR_KSSL    (1 << 27)  /* Key Select Soft Lock */
#define DCR_RKHL    (1 << 26)  /* Random Key Hard Lock */
#define DCR_RKSL    (1 << 25)  /* Random Key Soft Lock */
#define DCR_PKRHL   (1 << 24)  /* Programmed Key Read Hard Lock */
#define DCR_PKRSL   (1 << 23)  /* Programmed Key Read Soft Lock */
#define DCR_PKWHL   (1 << 22)  /* Programmed Key Write Hard Lock */
#define DCR_PKWSL   (1 << 21)  /* Programmed Key Write Soft Lock */
#define DCR_MCHL    (1 << 20)  /* Monotonic Counter Hard Lock */
#define DCR_MCSL    (1 << 19)  /* Monotonic Counter Soft Lock */
#define DCR_TCHL    (1 << 18)  /* Time Counter Hard Lock */
#define DCR_TCSL    (1 << 17)  /* Time Counter Soft Lock */
#define DCR_FSHL    (1 << 16)  /* Failure State Hard Lock */
#define DCR_NSA     (1 << 15)  /* Non-Secure Access */
#define DCR_OSCB    (1 << 14)  /* Oscillator Bypass */
#define DCR_APE     (1 << 4)   /* Alarm Pin Enable */
#define DCR_TCE     (1 << 3)   /* Time Counter Enable */
#define DCR_MCE     (1 << 2)   /* Monotonic Counter Enable */
#define DCR_SWR     (1 << 0)   /* Software Reset (w/o) */

/* DryIce Status Reg */
#define DSR     0x14
#define DSR_WTD     (1 << 23)  /* Wire-mesh Tampering Detected */
#define DSR_ETBD    (1 << 22)  /* External Tampering B Detected */
#define DSR_ETAD    (1 << 21)  /* External Tampering A Detected */
#define DSR_EBD     (1 << 20)  /* External Boot Detected */
#define DSR_SAD     (1 << 19)  /* Security Alarm Detected */
#define DSR_TTD     (1 << 18)  /* Temperature Tampering Detected */
#define DSR_CTD     (1 << 17)  /* Clock Tampering Detected */
#define DSR_VTD     (1 << 16)  /* Voltage Tampering Detected */
#define DSR_KBF     (1 << 11)  /* Key Busy Flag */
#define DSR_WBF     (1 << 10)  /* Write Busy Flag */
#define DSR_WNF     (1 << 9)   /* Write Next Flag */
#define DSR_WCF     (1 << 8)   /* Write Complete Flag */
#define DSR_WEF     (1 << 7)   /* Write Error Flag */
#define DSR_RKE     (1 << 6)   /* Random Key Error */
#define DSR_RKV     (1 << 5)   /* Random Key Valid */
#define DSR_CAF     (1 << 4)   /* Clock Alarm Flag */
#define DSR_MCO     (1 << 3)   /* Monotonic Counter Overflow */
#define DSR_TCO     (1 << 2)   /* Time Counter Overflow */
#define DSR_NVF     (1 << 1)   /* Non-Valid Flag */
#define DSR_SVF     (1 << 0)   /* Security Violation Flag */

#define DSR_TAMPER_BITS (DSR_WTD | DSR_ETBD | DSR_ETAD | DSR_EBD | DSR_SAD | \
			 DSR_TTD | DSR_CTD | DSR_VTD | DSR_MCO | DSR_TCO)

/* ensure that external tamper defs match register bits */
#if DSR_WTD != DI_TAMPER_EVENT_WTD
#error "Mismatch between DSR_WTD and DI_TAMPER_EVENT_WTD"
#endif
#if DSR_ETBD != DI_TAMPER_EVENT_ETBD
#error "Mismatch between DSR_ETBD and DI_TAMPER_EVENT_ETBD"
#endif
#if DSR_ETAD != DI_TAMPER_EVENT_ETAD
#error "Mismatch between DSR_ETAD and DI_TAMPER_EVENT_ETAD"
#endif
#if DSR_EBD != DI_TAMPER_EVENT_EBD
#error "Mismatch between DSR_EBD and DI_TAMPER_EVENT_EBD"
#endif
#if DSR_SAD != DI_TAMPER_EVENT_SAD
#error "Mismatch between DSR_SAD and DI_TAMPER_EVENT_SAD"
#endif
#if DSR_TTD != DI_TAMPER_EVENT_TTD
#error "Mismatch between DSR_TTD and DI_TAMPER_EVENT_TTD"
#endif
#if DSR_CTD != DI_TAMPER_EVENT_CTD
#error "Mismatch between DSR_CTD and DI_TAMPER_EVENT_CTD"
#endif
#if DSR_VTD != DI_TAMPER_EVENT_VTD
#error "Mismatch between DSR_VTD and DI_TAMPER_EVENT_VTD"
#endif
#if DSR_MCO != DI_TAMPER_EVENT_MCO
#error "Mismatch between DSR_MCO and DI_TAMPER_EVENT_MCO"
#endif
#if DSR_TCO != DI_TAMPER_EVENT_TCO
#error "Mismatch between DSR_TCO and DI_TAMPER_EVENT_TCO"
#endif

/* DryIce Interrupt Enable Reg */
#define DIER    0x18
#define DIER_WNIE   (1 << 9)   /* Write Next Interrupt Enable */
#define DIER_WCIE   (1 << 8)   /* Write Complete Interrupt Enable */
#define DIER_WEIE   (1 << 7)   /* Write Error Interrupt Enable */
#define DIER_RKIE   (1 << 5)   /* Random Key Interrupt Enable */
#define DIER_CAIE   (1 << 4)   /* Clock Alarm Interrupt Enable */
#define DIER_MOIE   (1 << 3)   /* Monotonic Overflow Interrupt En */
#define DIER_TOIE   (1 << 2)   /* Time Overflow Interrupt Enable */
#define DIER_SVIE   (1 << 0)   /* Security Violation Interrupt En */

/* DryIce Monotonic Counter Reg */
#define DMCR    0x1c

/* DryIce Key Select Reg */
#define DKSR    0x20
#define DKSR_IIM_KEY           0x0
#define DKSR_PROG_KEY          0x4
#define DKSR_RAND_KEY          0x5
#define DKSR_PROG_XOR_IIM_KEY  0x6
#define DKSR_RAND_XOR_IIM_KEY  0x7

/* DryIce Key Control Reg */
#define DKCR    0x24
#define DKCR_LRK    (1 << 0)   /* Load Random Key */

/* DryIce Tamper Configuration Reg */
#define DTCR    0x28
#define DTCR_ETGFB_SHIFT  27   /* Ext Tamper Glitch Filter B */
#define DTCR_ETGFB_MASK   0xf8000000
#define DTCR_ETGFA_SHIFT  22   /* Ext Tamper Glitch Filter A */
#define DTCR_ETGFA_MASK   0x07c00000
#define DTCR_WTGF_SHIFT   17   /* Wire-mesh Tamper Glitch Filter */
#define DTCR_WTGF_MASK    0x003e0000
#define DTCR_WGFE   (1 << 16)  /* Wire-mesh Glitch Filter Enable */
#define DTCR_SAOE   (1 << 15)  /* Security Alarm Output Enable */
#define DTCR_MOE    (1 << 9)   /* Monotonic Overflow Enable */
#define DTCR_TOE    (1 << 8)   /* Time Overflow Enable */
#define DTCR_WTE    (1 << 7)   /* Wire-mesh Tampering Enable */
#define DTCR_ETBE   (1 << 6)   /* External Tampering B Enable */
#define DTCR_ETAE   (1 << 5)   /* External Tampering A Enable */
#define DTCR_EBE    (1 << 4)   /* External Boot Enable */
#define DTCR_SAIE   (1 << 3)   /* Security Alarm Input Enable */
#define DTCR_TTE    (1 << 2)   /* Temperature Tamper Enable */
#define DTCR_CTE    (1 << 1)   /* Clock Tamper Enable */
#define DTCR_VTE    (1 << 0)   /* Voltage Tamper Enable */

/* DryIce Analog Configuration Reg */
#define DACR    0x2c
#define DACR_VRC_SHIFT    6    /* Voltage Reference Configuration */
#define DACR_VRC_MASK     0x000001c0
#define DACR_HTDC_SHIFT   3    /* High Temperature Detect Configuration */
#define DACR_HTDC_MASK    0x00000038
#define DACR_LTDC_SHIFT   0    /* Low Temperature Detect Configuration */
#define DACR_LTDC_MASK    0x00000007

/* DryIce General Purpose Reg */
#define DGPR    0x3c

/* DryIce Programmed Key0-7 Regs */
#define DPKR0   0x40
#define DPKR1   0x44
#define DPKR2   0x48
#define DPKR3   0x4c
#define DPKR4   0x50
#define DPKR5   0x54
#define DPKR6   0x58
#define DPKR7   0x5c

/* DryIce Random Key0-7 Regs */
#define DRKR0   0x60
#define DRKR1   0x64
#define DRKR2   0x68
#define DRKR3   0x6c
#define DRKR4   0x70
#define DRKR5   0x74
#define DRKR6   0x78
#define DRKR7   0x7c

#define DI_ADDRESS_RANGE  (DRKR7 + 4)

/*
 * this doesn't really belong here but the
 * portability layer doesn't include it
 */
#ifdef LINUX_KERNEL
#define EXTERN_SYMBOL(symbol)  EXPORT_SYMBOL(symbol)
#else
#define EXTERN_SYMBOL(symbol)  do {} while (0)
#endif

#endif /* __DRYICE_REGS_H__ */
