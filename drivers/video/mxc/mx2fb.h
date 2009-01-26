/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file    mx2fb.h
 *
 * @brief Header file for the MX27 Frame buffer
 *
 * @ingroup Framebuffer
 */
#ifndef __MX2FB_H__
#define __MX2FB_H__

/*! @brief MX27 LCDC graphic window information */
struct fb_gwinfo {
	/*! Non-zero if graphic window is enabled */
	__u32 enabled;

	/* The fields below are valid only when graphic window is enabled */

	/*! Graphic window alpha value from 0 to 255 */
	__u32 alpha_value;

	/*! Non-zero if graphic window color keying is enabled. */
	__u32 ck_enabled;

	/*
	 * The fields ck_red, ck_green and ck_blue are valid only when
	 * graphic window and the color keying are enabled. They are the
	 * color component of graphic window color keying.
	 */

	/*! Color keying red component */
	__u32 ck_red;

	/*! Color keying green component */
	__u32 ck_green;

	/*! Color keying blue component */
	__u32 ck_blue;

	/*! Graphic window x position */
	__u32 xpos;

	/*! Graphic window y position */
	__u32 ypos;

	/*! Non-zero if graphic window vertical scan in reverse direction. */
	__u32 vs_reversed;

	/*
	 * The following fields are valid for FBIOGET_GWINFO and
	 * mx2_gw_set(). FBIOPUT_GWINFO ignores these fields.
	 */
	__u32 base;		/* Graphic window start address */
	__u32 xres;		/* Visible x resolution */
	__u32 yres;		/* Visible y resolution */
	__u32 xres_virtual;	/* Virtual x resolution */
};

/* 0x46E0-0x46FF are reserved for MX27 */
#define FBIOGET_GWINFO		0x46E0	/*!< Get graphic window information */
#define FBIOPUT_GWINFO		0x46E1	/*!< Set graphic window information */

struct mx2fb_gbl_alpha {
	int enable;
	int alpha;
};

struct mx2fb_color_key {
	int enable;
	__u32 color_key;
};

#define MX2FB_SET_GBL_ALPHA	_IOW('M', 0, struct mx2fb_gbl_alpha)
#define MX2FB_SET_CLR_KEY	_IOW('M', 1, struct mx2fb_color_key)
#define MX2FB_WAIT_FOR_VSYNC	_IOW('F', 0x20, u_int32_t)

#ifdef __KERNEL__

/*
 * LCDC register definitions
 */
#define LCDC_LSSAR		0x00
#define LCDC_LSR		0x04
#define LCDC_LVPWR		0x08
#define LCDC_LCPR		0x0C
#define LCDC_LCWHBR		0x10
#define LCDC_LCCMR		0x14
#define LCDC_LPCR		0x18
#define LCDC_LHCR		0x1C
#define LCDC_LVCR		0x20
#define LCDC_LPOR		0x24
#define LCDC_LSCR		0x28
#define LCDC_LPCCR		0x2C
#define LCDC_LDCR		0x30
#define LCDC_LRMCR		0x34
#define LCDC_LICR		0x38
#define LCDC_LIER		0x3C
#define LCDC_LISR		0x40
#define LCDC_LGWSAR		0x50
#define LCDC_LGWSR		0x54
#define LCDC_LGWVPWR		0x58
#define LCDC_LGWPOR		0x5C
#define LCDC_LGWPR		0x60
#define LCDC_LGWCR		0x64
#define LCDC_LGWDCR		0x68
#define LCDC_LAUSCR		0x80
#define LCDC_LAUSCCR		0x84

#define LCDC_REG(reg)		(IO_ADDRESS(LCDC_BASE_ADDR) + reg)

#define MX2FB_INT_BOF		0x0001	/* Beginning of Frame */
#define MX2FB_INT_EOF		0x0002	/* End of Frame */
#define MX2FB_INT_ERR_RES	0x0004	/* Error Response */
#define MX2FB_INT_UDR_ERR	0x0008	/* Under Run Error */
#define MX2FB_INT_GW_BOF	0x0010	/* Graphic Window BOF */
#define MX2FB_INT_GW_EOF	0x0020	/* Graphic Window EOF */
#define MX2FB_INT_GW_ERR_RES	0x0040	/* Graphic Window ERR_RES */
#define MX2FB_INT_GW_UDR_ERR	0x0080	/* Graphic Window UDR_ERR */

#define FB_EVENT_MXC_EOF	0x8001	/* End of Frame event */

int mx2fb_register_client(struct notifier_block *nb);
int mx2fb_unregister_client(struct notifier_block *nb);

void mx2_gw_set(struct fb_gwinfo *gwinfo);

#endif				/* __KERNEL__ */

#endif				/* __MX2FB_H__ */
