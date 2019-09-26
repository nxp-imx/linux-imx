/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright 2019 NXP */

#ifndef _MISC_IMX_SECVIO_SC_H_
#define _MISC_IMX_SECVIO_SC_H_

#define HPSVS__LP_SEC_VIO__MASK BIT(31)
#define HPSVS__SW_LPSV__MASK    BIT(15)
#define HPSVS__SW_FSV__MASK     BIT(14)
#define HPSVS__SW_SV__MASK      BIT(13)
#define HPSVS__SV5__MASK        BIT(5)
#define HPSVS__SV4__MASK        BIT(4)
#define HPSVS__SV3__MASK        BIT(3)
#define HPSVS__SV2__MASK        BIT(2)
#define HPSVS__SV1__MASK        BIT(1)
#define HPSVS__SV0__MASK        BIT(0)

#define HPSVS__ALL_SV__MASK (HPSVS__LP_SEC_VIO__MASK | \
			     HPSVS__SW_LPSV__MASK | \
			     HPSVS__SW_FSV__MASK | \
			     HPSVS__SW_SV__MASK | \
			     HPSVS__SV5__MASK | \
			     HPSVS__SV4__MASK | \
			     HPSVS__SV3__MASK | \
			     HPSVS__SV2__MASK | \
			     HPSVS__SV1__MASK | \
			     HPSVS__SV0__MASK)

#define LPS__ESVD__MASK  BIT(16)
#define LPS__ET2D__MASK  BIT(10)
#define LPS__ET1D__MASK  BIT(9)
#define LPS__WMT2D__MASK BIT(8)
#define LPS__WMT1D__MASK BIT(7)
#define LPS__VTD__MASK   BIT(6)
#define LPS__TTD__MASK   BIT(5)
#define LPS__CTD__MASK   BIT(4)
#define LPS__PGD__MASK   BIT(3)
#define LPS__MCR__MASK   BIT(2)
#define LPS__SRTCR__MASK BIT(1)
#define LPS__LPTA__MASK  BIT(0)

#define LPS__ALL_TP__MASK (LPS__ESVD__MASK | \
			   LPS__ET2D__MASK | \
			   LPS__ET1D__MASK | \
			   LPS__WMT2D__MASK | \
			   LPS__WMT1D__MASK | \
			   LPS__VTD__MASK | \
			   LPS__TTD__MASK | \
			   LPS__CTD__MASK | \
			   LPS__PGD__MASK | \
			   LPS__MCR__MASK | \
			   LPS__SRTCR__MASK | \
			   LPS__LPTA__MASK)

#define LPTDS__ET10D__MASK  BIT(7)
#define LPTDS__ET9D__MASK   BIT(6)
#define LPTDS__ET8D__MASK   BIT(5)
#define LPTDS__ET7D__MASK   BIT(4)
#define LPTDS__ET6D__MASK   BIT(3)
#define LPTDS__ET5D__MASK   BIT(2)
#define LPTDS__ET4D__MASK   BIT(1)
#define LPTDS__ET3D__MASK   BIT(0)

#define LPTDS__ALL_TP__MASK (LPTDS__ET10D__MASK | \
			     LPTDS__ET9D__MASK | \
			     LPTDS__ET8D__MASK | \
			     LPTDS__ET7D__MASK | \
			     LPTDS__ET6D__MASK | \
			     LPTDS__ET5D__MASK | \
			     LPTDS__ET4D__MASK | \
			     LPTDS__ET3D__MASK)

/* Struct for notification */
struct notifier_info {
	u32 sv_status;   /* From HPSVS */
	u32 tp_status_1; /* From LPS */
	u32 tp_status_2; /* From LPTDS */
};

int register_imx_secvio_sc_notifier(struct notifier_block *nb);
int unregister_imx_secvio_sc_notifier(struct notifier_block *nb);

#endif /* _MISC_IMX_SECVIO_SC_H_ */
