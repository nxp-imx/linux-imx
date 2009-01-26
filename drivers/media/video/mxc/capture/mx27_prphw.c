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
 * @file mx27_prphw.c
 *
 * @brief MX27 Video For Linux 2 capture driver
 *
 * @ingroup MXC_V4L2_CAPTURE
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/delay.h>

#include "mx27_prp.h"

#define PRP_MIN_IN_WIDTH	32
#define PRP_MAX_IN_WIDTH	2044
#define PRP_MIN_IN_HEIGHT	32
#define PRP_MAX_IN_HEIGHT	2044

typedef struct _coeff_t {
	unsigned long coeff[2];
	unsigned long cntl;
} coeff_t[2][2];

static coeff_t *PRP_RSZ_COEFF = (coeff_t *) PRP_CH1_RZ_HORI_COEF1;

static unsigned char scale_get(scale_t * t,
			       unsigned char *i, unsigned char *out);
static int gcd(int x, int y);
static int ratio(int x, int y, int *den);
static int prp_scale_bilinear(scale_t * t, int coeff, int base, int nxt);
static int prp_scale_ave(scale_t * t, unsigned char base);
static int ave_scale(scale_t * t, int inv, int outv);
static int scale(scale_t * t, int inv, int outv);

/*!
 * @param t	table
 * @param i	table index
 * @param out	bilinear	# input pixels to advance
 *		average		whether result is ready for output
 * @return	coefficient
*/
static unsigned char scale_get(scale_t * t, unsigned char *i,
			       unsigned char *out)
{
	unsigned char c;

	c = t->tbl[*i];
	(*i)++;
	*i %= t->len;

	if (out) {
		if (t->algo == ALGO_BIL) {
			for ((*out) = 1;
			     (*i) && ((*i) < t->len) && !t->tbl[(*i)]; (*i)++) {
				(*out)++;
			}
			if ((*i) == t->len)
				(*i) = 0;
		} else
			*out = c >> BC_COEF;
	}

	c &= SZ_COEF - 1;

	if (c == SZ_COEF - 1)
		c = SZ_COEF;

	return c;
}

/*!
 * @brief Get maximum common divisor.
 * @param x	First input value
 * @param y	Second input value
 * @return	Maximum common divisor of x and y
 */
static int gcd(int x, int y)
{
	int k;

	if (x < y) {
		k = x;
		x = y;
		y = k;
	}

	while ((k = x % y)) {
		x = y;
		y = k;
	}

	return y;
}

/*!
 * @brief Get ratio.
 * @param x	First input value
 * @param y	Second input value
 * @param den	Denominator of the ratio (corresponding to y)
 * @return	Numerator of the ratio (corresponding to x)
 */
static int ratio(int x, int y, int *den)
{
	int g;

	if (!x || !y)
		return 0;

	g = gcd(x, y);
	*den = y / g;

	return x / g;
}

/*!
 * @brief Build PrP coefficient entry based on bilinear algorithm
 *
 * @param t	The pointer to scale_t structure
 * @param coeff	The weighting coefficient
 * @param base	The base of the coefficient
 * @param nxt	Number of pixels to be read
 *
 * @return	The length of current coefficient table on success
 *		-1 on failure
 */
static int prp_scale_bilinear(scale_t * t, int coeff, int base, int nxt)
{
	int i;

	if (t->len >= sizeof(t->tbl))
		return -1;

	coeff = ((coeff << BC_COEF) + (base >> 1)) / base;
	if (coeff >= SZ_COEF - 1)
		coeff--;

	coeff |= SZ_COEF;
	t->tbl[(int)t->len++] = (unsigned char)coeff;

	for (i = 1; i < nxt; i++) {
		if (t->len >= MAX_TBL)
			return -1;

		t->tbl[(int)t->len++] = 0;
	}

	return t->len;
}

#define _bary(name)	static const unsigned char name[]

_bary(c1) = {
7};

_bary(c2) = {
4, 4};

_bary(c3) = {
2, 4, 2};

_bary(c4) = {
2, 2, 2, 2};

_bary(c5) = {
1, 2, 2, 2, 1};

_bary(c6) = {
1, 1, 2, 2, 1, 1};

_bary(c7) = {
1, 1, 1, 2, 1, 1, 1};

_bary(c8) = {
1, 1, 1, 1, 1, 1, 1, 1};

_bary(c9) = {
1, 1, 1, 1, 1, 1, 1, 1, 0};

_bary(c10) = {
0, 1, 1, 1, 1, 1, 1, 1, 1, 0};

_bary(c11) = {
0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0};

_bary(c12) = {
0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0};

_bary(c13) = {
0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0};

_bary(c14) = {
0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0};

_bary(c15) = {
0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0};

_bary(c16) = {
1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

_bary(c17) = {
0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

_bary(c18) = {
0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0};

_bary(c19) = {
0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0};

_bary(c20) = {
0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0};

static const unsigned char *ave_coeff[] = {
	c1, c2, c3, c4, c5, c6, c7, c8, c9, c10,
	c11, c12, c13, c14, c15, c16, c17, c18, c19, c20
};

/*!
 * @brief Build PrP coefficient table based on average algorithm
 *
 * @param t	The pointer to scale_t structure
 * @param base	The base of the coefficient
 *
 * @return	The length of current coefficient table on success
 *		-1 on failure
 */
static int prp_scale_ave(scale_t * t, unsigned char base)
{
	if (t->len + base > sizeof(t->tbl))
		return -1;

	memcpy(&t->tbl[(int)t->len], ave_coeff[(int)base - 1], base);
	t->len = (unsigned char)(t->len + base);
	t->tbl[t->len - 1] |= SZ_COEF;

	return t->len;
}

/*!
 * @brief Build PrP coefficient table based on average algorithm
 *
 * @param t	The pointer to scale_t structure
 * @param inv	Input resolution
 * @param outv	Output resolution
 *
 * @return	The length of current coefficient table on success
 *		-1 on failure
 */
static int ave_scale(scale_t * t, int inv, int outv)
{
	int ratio_count;

	ratio_count = 0;
	if (outv != 1) {
		unsigned char a[20];
		int v;

		/* split n:m into multiple n[i]:1 */
		for (v = 0; v < outv; v++)
			a[v] = (unsigned char)(inv / outv);

		inv %= outv;
		if (inv) {
			/* find start of next layer */
			v = (outv - inv) >> 1;
			inv += v;
			for (; v < inv; v++)
				a[v]++;
		}

		for (v = 0; v < outv; v++) {
			if (prp_scale_ave(t, a[v]) < 0)
				return -1;

			t->ratio[ratio_count] = a[v];
			ratio_count++;
		}
	} else if (prp_scale_ave(t, inv) < 0) {
		return -1;
	} else {
		t->ratio[ratio_count++] = (char)inv;
		ratio_count++;
	}

	return t->len;
}

/*!
 * @brief Build PrP coefficient table
 *
 * @param t	The pointer to scale_t structure
 * @param inv	input resolution reduced ratio
 * @param outv	output resolution reduced ratio
 *
 * @return	The length of current coefficient table on success
 *		-1 on failure
 */
static int scale(scale_t * t, int inv, int outv)
{
	int v;			/* overflow counter */
	int coeff, nxt;		/* table output */

	t->len = 0;
	if (t->algo == ALGO_AUTO) {
		/* automatic choice - bilinear for shrinking less than 2:1 */
		t->algo = ((outv != inv) && ((2 * outv) > inv)) ?
		    ALGO_BIL : ALGO_AVG;
	}

	/* 1:1 resize must use averaging, bilinear will hang */
	if ((inv == outv) && (t->algo == ALGO_BIL)) {
		pr_debug("Warning: 1:1 resize must use averaging algo\n");
		t->algo = ALGO_AVG;
	}

	memset(t->tbl, 0, sizeof(t->tbl));
	if (t->algo == ALGO_BIL) {
		t->ratio[0] = (char)inv;
		t->ratio[1] = (char)outv;
	} else
		memset(t->ratio, 0, sizeof(t->ratio));

	if (inv == outv) {
		/* force scaling */
		t->ratio[0] = 1;
		if (t->algo == ALGO_BIL)
			t->ratio[1] = 1;

		return prp_scale_ave(t, 1);
	}

	if (inv < outv) {
		pr_debug("Upscaling not supported %d:%d\n", inv, outv);
		return -1;
	}

	if (t->algo != ALGO_BIL)
		return ave_scale(t, inv, outv);

	v = 0;
	if (inv >= 2 * outv) {
		/* downscale: >=2:1 bilinear approximation */
		coeff = inv - 2 * outv;
		v = 0;
		nxt = 0;
		do {
			v += coeff;
			nxt = 2;
			while (v >= outv) {
				v -= outv;
				nxt++;
			}

			if (prp_scale_bilinear(t, 1, 2, nxt) < 0)
				return -1;
		} while (v);
	} else {
		/* downscale: bilinear */
		int in_pos_inc = 2 * outv;
		int out_pos = inv;
		int out_pos_inc = 2 * inv;
		int init_carry = inv - outv;
		int carry = init_carry;

		v = outv + in_pos_inc;
		do {
			coeff = v - out_pos;
			out_pos += out_pos_inc;
			carry += out_pos_inc;
			for (nxt = 0; v < out_pos; nxt++) {
				v += in_pos_inc;
				carry -= in_pos_inc;
			}
			if (prp_scale_bilinear(t, coeff, in_pos_inc, nxt) < 0)
				return -1;
		} while (carry != init_carry);
	}
	return t->len;
}

/*!
 * @brief Build PrP coefficient table
 *
 * @param pscale	The pointer to scale_t structure which holdes
 * 			coefficient tables
 * @param din		Scale ratio numerator
 * @param dout		Scale ratio denominator
 * @param inv		Input resolution
 * @param vout		Output resolution
 * @param pout		Internal output resolution
 * @param retry		Retry times (round the output length) when need
 *
 * @return		Zero on success, others on failure
 */
int prp_scale(scale_t * pscale, int din, int dout, int inv,
	      unsigned short *vout, unsigned short *pout, int retry)
{
	int num;
	int den;
	unsigned short outv;

	/* auto-generation of values */
	if (!(dout && din)) {
		if (!*vout)
			dout = din = 1;
		else {
			din = inv;
			dout = *vout;
		}
	}

	if (din < dout) {
		pr_debug("Scale err, unsupported ratio %d : %d\n", din, dout);
		return -1;
	}

      lp_retry:
	num = ratio(din, dout, &den);
	if (!num) {
		pr_debug("Scale err, unsupported ratio %d : %d\n", din, dout);
		return -1;
	}

	if (num > MAX_TBL || scale(pscale, num, den) < 0) {
		dout++;
		if (retry--)
			goto lp_retry;

		pr_debug("Scale err, unsupported ratio %d : %d\n", num, den);
		return -1;
	}

	if (pscale->algo == ALGO_BIL) {
		unsigned char i, j, k;

		outv =
		    (unsigned short)(inv / pscale->ratio[0] * pscale->ratio[1]);
		inv %= pscale->ratio[0];
		for (i = j = 0; inv > 0; j++) {
			unsigned char nxt;

			k = scale_get(pscale, &i, &nxt);
			if (inv == 1 && k < SZ_COEF) {
				/* needs 2 pixels for this output */
				break;
			}
			inv -= nxt;
		}
		outv = outv + j;
	} else {
		unsigned char i, tot;

		for (tot = i = 0; pscale->ratio[i]; i++)
			tot = tot + pscale->ratio[i];

		outv = (unsigned short)(inv / tot) * i;
		inv %= tot;
		for (i = 0; inv > 0; i++, outv++)
			inv -= pscale->ratio[i];
	}

	if (!(*vout) || ((*vout) > outv))
		*vout = outv;

	if (pout)
		*pout = outv;

	return 0;
}

/*!
 * @brief Reset PrP block
 */
int prphw_reset(void)
{
	unsigned long val;
	unsigned long flag;
	int i;

	flag = PRP_CNTL_RST;
	val = PRP_CNTL_RSTVAL;

	__raw_writel(flag, PRP_CNTL);

	/* timeout */
	for (i = 0; i < 1000; i++) {
		if (!(__raw_readl(PRP_CNTL) & flag)) {
			pr_debug("PrP reset over\n");
			break;
		}
		msleep(1);
	}

	/* verify reset value */
	if (__raw_readl(PRP_CNTL) != val) {
		pr_info("PrP reset err, val = 0x%08X\n", __raw_readl(PRP_CNTL));
		return -1;
	}

	return 0;
}

/*!
 * @brief Enable PrP channel.
 * @param channel	Channel number to be enabled
 * @return		Zero on success, others on failure
 */
int prphw_enable(int channel)
{
	unsigned long val;

	val = __raw_readl(PRP_CNTL);
	if (channel & PRP_CHANNEL_1)
		val |= PRP_CNTL_CH1EN;
	if (channel & PRP_CHANNEL_2)
		val |= (PRP_CNTL_CH2EN | PRP_CNTL_CH2_FLOWEN);

	__raw_writel(val, PRP_CNTL);

	return 0;
}

/*!
 * @brief Disable PrP channel.
 * @param channel	Channel number to be disable
 * @return		Zero on success, others on failure
 */
int prphw_disable(int channel)
{
	unsigned long val;

	val = __raw_readl(PRP_CNTL);
	if (channel & PRP_CHANNEL_1)
		val &= ~PRP_CNTL_CH1EN;
	if (channel & PRP_CHANNEL_2)
		val &= ~(PRP_CNTL_CH2EN | PRP_CNTL_CH2_FLOWEN);

	__raw_writel(val, PRP_CNTL);

	return 0;
}

/*!
 * @brief Set PrP input buffer address.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_inptr(emma_prp_cfg * cfg)
{
	if (cfg->in_csi & PRP_CSI_EN)
		return -1;

	__raw_writel(cfg->in_ptr, PRP_SOURCE_Y_PTR);
	if (cfg->in_pix == PRP_PIXIN_YUV420) {
		u32 size;

		size = cfg->in_line_stride * cfg->in_height;
		__raw_writel(cfg->in_ptr + size, PRP_SOURCE_CB_PTR);
		__raw_writel(cfg->in_ptr + size + (size >> 2),
			     PRP_SOURCE_CR_PTR);
	}
	return 0;
}

/*!
 * @brief Set PrP channel 1 output buffer 1 address.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_ch1ptr(emma_prp_cfg * cfg)
{
	if (cfg->ch1_pix == PRP_PIX1_UNUSED)
		return -1;

	__raw_writel(cfg->ch1_ptr, PRP_DEST_RGB1_PTR);

	/* support double buffer in loop mode only */
	if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP) {
		if (cfg->ch1_ptr2)
			__raw_writel(cfg->ch1_ptr2, PRP_DEST_RGB2_PTR);
		else
			__raw_writel(cfg->ch1_ptr, PRP_DEST_RGB2_PTR);
	}

	return 0;
}

/*!
 * @brief Set PrP channel 1 output buffer 2 address.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_ch1ptr2(emma_prp_cfg * cfg)
{
	if (cfg->ch1_pix == PRP_PIX1_UNUSED ||
	    (cfg->in_csi & PRP_CSI_LOOP) != PRP_CSI_LOOP)
		return -1;

	if (cfg->ch1_ptr2)
		__raw_writel(cfg->ch1_ptr2, PRP_DEST_RGB2_PTR);
	else
		return -1;

	return 0;
}

/*!
 * @brief Set PrP channel 2 output buffer 1 address.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_ch2ptr(emma_prp_cfg * cfg)
{
	u32 size;

	if (cfg->ch2_pix == PRP_PIX2_UNUSED)
		return -1;

	__raw_writel(cfg->ch2_ptr, PRP_DEST_Y_PTR);

	if (cfg->ch2_pix == PRP_PIX2_YUV420) {
		size = cfg->ch2_width * cfg->ch2_height;
		__raw_writel(cfg->ch2_ptr + size, PRP_DEST_CB_PTR);
		__raw_writel(cfg->ch2_ptr + size + (size >> 2),
			     PRP_DEST_CR_PTR);
	}

	__raw_writel(__raw_readl(PRP_CNTL) | PRP_CNTL_CH2B1, PRP_CNTL);
	return 0;
}

/*!
 * @brief Set PrP channel 2 output buffer 2 address.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_ch2ptr2(emma_prp_cfg * cfg)
{
	u32 size;

	if (cfg->ch2_pix == PRP_PIX2_UNUSED ||
	    (cfg->in_csi & PRP_CSI_LOOP) != PRP_CSI_LOOP)
		return -1;

	__raw_writel(cfg->ch2_ptr2, PRP_SOURCE_Y_PTR);
	if (cfg->ch2_pix == PRP_PIX2_YUV420) {
		size = cfg->ch2_width * cfg->ch2_height;
		__raw_writel(cfg->ch2_ptr2 + size, PRP_SOURCE_CB_PTR);
		__raw_writel(cfg->ch2_ptr2 + size + (size >> 2),
			     PRP_SOURCE_CR_PTR);
	}

	__raw_writel(__raw_readl(PRP_CNTL) | PRP_CNTL_CH2B2, PRP_CNTL);
	return 0;
}

/*!
 * @brief Build CSC table
 * @param csc	CSC table
 *		in	csc[0]=index		0..3 : A.1 A.0 B.1 B.0
 *			csc[1]=direction	0 : YUV2RGB  1 : RGB2YUV
 *		out	csc[0..4] are coefficients c[9] is offset
 *			csc[0..8] are coefficients c[9] is offset
 */
void csc_tbl(short csc[10])
{
	static const unsigned short _r2y[][9] = {
		{0x4D, 0x4B, 0x3A, 0x57, 0x55, 0x40, 0x40, 0x6B, 0x29},
		{0x42, 0x41, 0x32, 0x4C, 0x4A, 0x38, 0x38, 0x5E, 0x24},
		{0x36, 0x5C, 0x25, 0x3B, 0x63, 0x40, 0x40, 0x74, 0x18},
		{0x2F, 0x4F, 0x20, 0x34, 0x57, 0x38, 0x38, 0x66, 0x15},
	};
	static const unsigned short _y2r[][5] = {
		{0x80, 0xb4, 0x2c, 0x5b, 0x0e4},
		{0x95, 0xcc, 0x32, 0x68, 0x104},
		{0x80, 0xca, 0x18, 0x3c, 0x0ec},
		{0x95, 0xe5, 0x1b, 0x44, 0x1e0},
	};
	unsigned short *_csc;
	int _csclen;

	csc[9] = csc[0] & 1;
	_csclen = csc[0] & 3;

	if (csc[1]) {
		_csc = (unsigned short *)_r2y[_csclen];
		_csclen = sizeof(_r2y[0]);
	} else {
		_csc = (unsigned short *)_y2r[_csclen];
		_csclen = sizeof(_y2r[0]);
		memset(csc + 5, 0, sizeof(short) * 4);
	}
	memcpy(csc, _csc, _csclen);
}

/*!
 * @brief Setup PrP resize coefficient registers
 *
 * @param ch	PrP channel number
 * @param dir	Direction, 0 - horizontal, 1 - vertical
 * @param scale	The pointer to scale_t structure
 */
static void prp_set_scaler(int ch, int dir, scale_t * scale)
{
	int i;
	unsigned int coeff[2];
	unsigned int valid;

	for (coeff[0] = coeff[1] = valid = 0, i = 19; i >= 0; i--) {
		int j;

		j = i > 9 ? 1 : 0;
		coeff[j] = (coeff[j] << BC_COEF) |
		    (scale->tbl[i] & (SZ_COEF - 1));

		if (i == 5 || i == 15)
			coeff[j] <<= 1;

		valid = (valid << 1) | (scale->tbl[i] >> BC_COEF);
	}

	valid |= (scale->len << 24) | ((2 - scale->algo) << 31);

	for (i = 0; i < 2; i++)
		(*PRP_RSZ_COEFF)[1 - ch][dir].coeff[i] = coeff[i];

	(*PRP_RSZ_COEFF)[1 - ch][dir].cntl = valid;
}

/*!
 * @brief Setup PrP registers relevant to input.
 * @param cfg		Pointer to PrP configuration parameter
 * @param prp_cntl	Holds the value for PrP control register
 * @return		Zero on success, others on failure
 */
static int prphw_input_cfg(emma_prp_cfg * cfg, unsigned long *prp_cntl)
{
	unsigned long mask;

	switch (cfg->in_pix) {
	case PRP_PIXIN_YUV420:
		*prp_cntl |= PRP_CNTL_IN_YUV420;
		mask = 0x7;
		break;
	case PRP_PIXIN_YUYV:
	case PRP_PIXIN_YVYU:
	case PRP_PIXIN_UYVY:
	case PRP_PIXIN_VYUY:
		*prp_cntl |= PRP_CNTL_IN_YUV422;
		mask = 0x1;
		break;
	case PRP_PIXIN_RGB565:
		*prp_cntl |= PRP_CNTL_IN_RGB16;
		mask = 0x1;
		break;
	case PRP_PIXIN_RGB888:
		*prp_cntl |= PRP_CNTL_IN_RGB32;
		mask = 0;
		break;
	default:
		pr_debug("Unsupported input pix format 0x%08X\n", cfg->in_pix);
		return -1;
	}

	/* align the input image width */
	if (cfg->in_width & mask) {
		pr_debug("in_width misaligned. in_width=%d\n", cfg->in_width);
		return -1;
	}

	if ((cfg->in_width < PRP_MIN_IN_WIDTH)
	    || (cfg->in_width > PRP_MAX_IN_WIDTH)) {
		pr_debug("Unsupported input width %d\n", cfg->in_width);
		return -1;
	}

	cfg->in_height &= ~1;	/* truncate to make even */

	if ((cfg->in_height < PRP_MIN_IN_HEIGHT)
	    || (cfg->in_height > PRP_MAX_IN_HEIGHT)) {
		pr_debug("Unsupported input height %d\n", cfg->in_height);
		return -1;
	}

	if (!(cfg->in_csi & PRP_CSI_EN))
		if (!cfg->in_line_stride)
			cfg->in_line_stride = cfg->in_width;

	__raw_writel(cfg->in_pix, PRP_SRC_PIXEL_FORMAT_CNTL);
	__raw_writel((cfg->in_width << 16) | cfg->in_height,
		     PRP_SOURCE_FRAME_SIZE);
	__raw_writel((cfg->in_line_skip << 16) | cfg->in_line_stride,
		     PRP_SOURCE_LINE_STRIDE);

	if (!(cfg->in_csi & PRP_CSI_EN)) {
		__raw_writel(cfg->in_ptr, PRP_SOURCE_Y_PTR);
		if (cfg->in_pix == PRP_PIXIN_YUV420) {
			unsigned int size;

			size = cfg->in_line_stride * cfg->in_height;
			__raw_writel(cfg->in_ptr + size, PRP_SOURCE_CB_PTR);
			__raw_writel(cfg->in_ptr + size + (size >> 2),
				     PRP_SOURCE_CR_PTR);
		}
	}

	/* always cropping */
	*prp_cntl |= PRP_CNTL_WINEN;

	/* color space conversion */
	if (!cfg->in_csc[1]) {
		if (cfg->in_csc[0] > 3) {
			pr_debug("in_csc invalid 0x%X\n", cfg->in_csc[0]);
			return -1;
		}
		if ((cfg->in_pix == PRP_PIXIN_RGB565)
		    || (cfg->in_pix == PRP_PIXIN_RGB888))
			cfg->in_csc[1] = 1;
		else
			cfg->in_csc[0] = 0;
		csc_tbl(cfg->in_csc);
	}

	__raw_writel((cfg->in_csc[0] << 21) | (cfg->in_csc[1] << 11)
		     | cfg->in_csc[2], PRP_CSC_COEF_012);
	__raw_writel((cfg->in_csc[3] << 21) | (cfg->in_csc[4] << 11)
		     | cfg->in_csc[5], PRP_CSC_COEF_345);
	__raw_writel((cfg->in_csc[6] << 21) | (cfg->in_csc[7] << 11)
		     | cfg->in_csc[8] | (cfg->in_csc[9] << 31),
		     PRP_CSC_COEF_678);

	if (cfg->in_csi & PRP_CSI_EN) {
		*prp_cntl |= PRP_CNTL_CSI;

		/* loop mode enable, ch1 ch2 together */
		if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP)
			*prp_cntl |= (PRP_CNTL_CH1_LOOP | PRP_CNTL_CH2_LOOP);
	}

	return 0;
}

/*!
 * @brief Setup PrP registers relevant to channel 2.
 * @param cfg		Pointer to PrP configuration parameter
 * @param prp_cntl	Holds the value for PrP control register
 * @return		Zero on success, others on failure
 */
static int prphw_ch2_cfg(emma_prp_cfg * cfg, unsigned long *prp_cntl)
{
	switch (cfg->ch2_pix) {
	case PRP_PIX2_YUV420:
		*prp_cntl |= PRP_CNTL_CH2_YUV420;
		break;
	case PRP_PIX2_YUV422:
		*prp_cntl |= PRP_CNTL_CH2_YUV422;
		break;
	case PRP_PIX2_YUV444:
		*prp_cntl |= PRP_CNTL_CH2_YUV444;
		break;
	case PRP_PIX2_UNUSED:
		return 0;
	default:
		pr_debug("Unsupported channel 2 pix format 0x%08X\n",
			 cfg->ch2_pix);
		return -1;
	}

	if (cfg->ch2_pix == PRP_PIX2_YUV420) {
		cfg->ch2_height &= ~1;	/* ensure U/V presence */
		cfg->ch2_width &= ~7;	/* ensure U/V word aligned */
	} else if (cfg->ch2_pix == PRP_PIX2_YUV422) {
		cfg->ch2_width &= ~1;	/* word aligned */
	}

	__raw_writel((cfg->ch2_width << 16) | cfg->ch2_height,
		     PRP_CH2_OUT_IMAGE_SIZE);

	if (cfg->ch2_pix == PRP_PIX2_YUV420) {
		u32 size;

		/* Luminanance band start address */
		__raw_writel(cfg->ch2_ptr, PRP_DEST_Y_PTR);

		if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP) {
			if (!cfg->ch2_ptr2)
				__raw_writel(cfg->ch2_ptr, PRP_SOURCE_Y_PTR);
			else
				__raw_writel(cfg->ch2_ptr2, PRP_SOURCE_Y_PTR);
		}

		/* Cb and Cr band start address */
		size = cfg->ch2_width * cfg->ch2_height;
		__raw_writel(cfg->ch2_ptr + size, PRP_DEST_CB_PTR);
		__raw_writel(cfg->ch2_ptr + size + (size >> 2),
			     PRP_DEST_CR_PTR);

		if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP) {
			if (!cfg->ch2_ptr2) {
				__raw_writel(cfg->ch2_ptr + size,
					     PRP_SOURCE_CB_PTR);
				__raw_writel(cfg->ch2_ptr + size + (size >> 2),
					     PRP_SOURCE_CR_PTR);
			} else {
				__raw_writel(cfg->ch2_ptr2 + size,
					     PRP_SOURCE_CB_PTR);
				__raw_writel(cfg->ch2_ptr2 + size + (size >> 2),
					     PRP_SOURCE_CR_PTR);
			}
		}
	} else {		/* Pixel interleaved YUV422 or YUV444 */
		__raw_writel(cfg->ch2_ptr, PRP_DEST_Y_PTR);

		if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP) {
			if (!cfg->ch2_ptr2)
				__raw_writel(cfg->ch2_ptr, PRP_SOURCE_Y_PTR);
			else
				__raw_writel(cfg->ch2_ptr2, PRP_SOURCE_Y_PTR);
		}
	}
	*prp_cntl |= PRP_CNTL_CH2B1 | PRP_CNTL_CH2B2;

	return 0;
}

/*!
 * @brief Setup PrP registers relevant to channel 1.
 * @param cfg		Pointer to PrP configuration parameter
 * @param prp_cntl	Holds the value for PrP control register
 * @return		Zero on success, others on failure
 */
static int prphw_ch1_cfg(emma_prp_cfg * cfg, unsigned long *prp_cntl)
{
	int ch1_bpp = 0;

	switch (cfg->ch1_pix) {
	case PRP_PIX1_RGB332:
		*prp_cntl |= PRP_CNTL_CH1_RGB8;
		ch1_bpp = 1;
		break;
	case PRP_PIX1_RGB565:
		*prp_cntl |= PRP_CNTL_CH1_RGB16;
		ch1_bpp = 2;
		break;
	case PRP_PIX1_RGB888:
		*prp_cntl |= PRP_CNTL_CH1_RGB32;
		ch1_bpp = 4;
		break;
	case PRP_PIX1_YUYV:
	case PRP_PIX1_YVYU:
	case PRP_PIX1_UYVY:
	case PRP_PIX1_VYUY:
		*prp_cntl |= PRP_CNTL_CH1_YUV422;
		ch1_bpp = 2;
		break;
	case PRP_PIX1_UNUSED:
		return 0;
	default:
		pr_debug("Unsupported channel 1 pix format 0x%08X\n",
			 cfg->ch1_pix);
		return -1;
	}

	/* parallel or cascade resize */
	if (cfg->ch1_scale.algo & PRP_ALGO_BYPASS)
		*prp_cntl |= PRP_CNTL_UNCHAIN;

	/* word align */
	if (ch1_bpp == 2)
		cfg->ch1_width &= ~1;
	else if (ch1_bpp == 1)
		cfg->ch1_width &= ~3;

	if (!cfg->ch1_stride)
		cfg->ch1_stride = cfg->ch1_width;

	__raw_writel(cfg->ch1_pix, PRP_CH1_PIXEL_FORMAT_CNTL);
	__raw_writel((cfg->ch1_width << 16) | cfg->ch1_height,
		     PRP_CH1_OUT_IMAGE_SIZE);
	__raw_writel(cfg->ch1_stride * ch1_bpp, PRP_CH1_LINE_STRIDE);
	__raw_writel(cfg->ch1_ptr, PRP_DEST_RGB1_PTR);

	/* double buffer for loop mode */
	if ((cfg->in_csi & PRP_CSI_LOOP) == PRP_CSI_LOOP) {
		if (cfg->ch1_ptr2)
			__raw_writel(cfg->ch1_ptr2, PRP_DEST_RGB2_PTR);
		else
			__raw_writel(cfg->ch1_ptr, PRP_DEST_RGB2_PTR);
	}

	return 0;
}

/*!
 * @brief Setup PrP registers.
 * @param cfg	Pointer to PrP configuration parameter
 * @return	Zero on success, others on failure
 */
int prphw_cfg(emma_prp_cfg * cfg)
{
	unsigned long prp_cntl = 0;
	unsigned long val;

	/* input pixel format checking */
	if (prphw_input_cfg(cfg, &prp_cntl))
		return -1;

	if (prphw_ch2_cfg(cfg, &prp_cntl))
		return -1;

	if (prphw_ch1_cfg(cfg, &prp_cntl))
		return -1;

	/* register setting */
	__raw_writel(prp_cntl, PRP_CNTL);

	/* interrupt configuration */
	val = PRP_INTRCNTL_RDERR | PRP_INTRCNTL_LBOVF;
	if (cfg->ch1_pix != PRP_PIX1_UNUSED)
		val |= PRP_INTRCNTL_CH1FC | PRP_INTRCNTL_CH1WERR;
	if (cfg->ch2_pix != PRP_PIX2_UNUSED)
		val |=
		    PRP_INTRCNTL_CH2FC | PRP_INTRCNTL_CH2WERR |
		    PRP_INTRCNTL_CH2OVF;
	__raw_writel(val, PRP_INTRCNTL);

	prp_set_scaler(1, 0, &cfg->scale[0]);	/* Channel 1 width */
	prp_set_scaler(1, 1, &cfg->scale[1]);	/* Channel 1 height */
	prp_set_scaler(0, 0, &cfg->scale[2]);	/* Channel 2 width */
	prp_set_scaler(0, 1, &cfg->scale[3]);	/* Channel 2 height */

	return 0;
}

/*!
 * @brief Check PrP interrupt status.
 * @return	PrP interrupt status
 */
int prphw_isr(void)
{
	int status;

	status = __raw_readl(PRP_INTRSTATUS) & 0x1FF;

	if (status & (PRP_INTRSTAT_RDERR | PRP_INTRSTAT_CH1WERR |
		      PRP_INTRSTAT_CH2WERR))
		pr_debug("isr bus error. status= 0x%08X\n", status);
	else if (status & PRP_INTRSTAT_CH2OVF)
		pr_debug("isr ch 2 buffer overflow. status= 0x%08X\n", status);
	else if (status & PRP_INTRSTAT_LBOVF)
		pr_debug("isr line buffer overflow. status= 0x%08X\n", status);

	/* silicon bug?? enable bit does not self clear? */
	if (!(__raw_readl(PRP_CNTL) & PRP_CNTL_CH1_LOOP))
		__raw_writel(__raw_readl(PRP_CNTL) & (~PRP_CNTL_CH1EN),
			     PRP_CNTL);
	if (!(__raw_readl(PRP_CNTL) & PRP_CNTL_CH2_LOOP))
		__raw_writel(__raw_readl(PRP_CNTL) & (~PRP_CNTL_CH2EN),
			     PRP_CNTL);

	__raw_writel(status, PRP_INTRSTATUS);	/* clr irq */

	return status;
}

static struct clk *emma_clk;

/*!
 * @brief  PrP module clock enable
 */
void prphw_init(void)
{
	emma_clk = clk_get(NULL, "emma_clk");
	clk_enable(emma_clk);
}

/*!
 * @brief PrP module clock disable
 */
void prphw_exit(void)
{
	clk_disable(emma_clk);
	clk_put(emma_clk);
}
