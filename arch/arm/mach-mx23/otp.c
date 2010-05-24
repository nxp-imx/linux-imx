/*
 * Unique ID manipulation: Freescale STMP378X OTP bits read/write procedures
 *
 * Author: dmitry pervushin <dimka@embeddedalley.com>
 *
 * Copyright 2008-2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/unique-id.h>
#include <mach/regs-ocotp.h>
#include <mach/regs-power.h>
#include <mach/mx23.h>

static DEFINE_MUTEX(otp_mutex);
static unsigned otp_mode;
static unsigned long otp_hclk_saved;
static u32 otp_voltage_saved;

static int otp_full; /* = 0. By default, show/set only customer bits */
#define OTP_USER_OFFSET 0
#define OTP_USER_SIZE	4

#define REGS_OCOTP_BASE (IO_ADDRESS(OCOTP_PHYS_ADDR))
#define BF(value, field) (((value) << BP_##field) & BM_##field)
/**
 * otp_wait_busy - wait for completion of operation
 *
 * @flags: flags that should be clear in addition to _BUSY and _ERROR
 *
 * Returns 0 on success or -ETIMEDOUT on error
 **/
static int otp_wait_busy(u32 flags)
{
	int count;
	u32 c;

	for (count = 10000; count >= 0; count--) {
		c = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CTRL);
		if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
			break;
		cpu_relax();
	}
	if (count < 0)
		return -ETIMEDOUT;
	return 0;
}

/**
 * otp_open - open OTP bits for read or write access
 *
 * @mode: either O_RDONLY or O_WRONLY
 *
 * Returns 0 on success, error code otherwise
 **/
static int otp_open(int mode)
{
	int r;
	struct clk *hclk;
	int err;

	if (!mutex_trylock(&otp_mutex)) {
		printk(KERN_ERR"%s: already opened\n", __func__);
		return -EAGAIN;
	}

	if (mode == O_RDONLY) {
		pr_debug("%s: read-only mode\n", __func__);

		r = otp_wait_busy(0);
		if (r) {
			err = -ETIMEDOUT;
			goto out;
		}

		/* 2. Set RD_BANK_OPEN */
		__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN, REGS_OCOTP_BASE + HW_OCOTP_CTRL_SET);
		udelay(10);

		otp_wait_busy(0);
	}

	else if (mode == O_WRONLY) {
		pr_debug("%s: write-only mode\n", __func__);
		hclk = clk_get(NULL, "hclk");
		if (IS_ERR(hclk)) {
			err = PTR_ERR(hclk);
			goto out;
		}

		/*
		   WARNING  ACHTUNG  UWAGA

		   the code below changes HCLK clock rate to 24M. This is
		   required to write OTP bits (7.2.2 in STMP378x Target
		   Specification), and might affect LCD operation, for example.
		   Moreover, this hacky code changes VDDIO to 2.8V; and resto-
		   res it only on otp_close(). This may affect... anything.

		   You are warned now.
		 */
		otp_hclk_saved = clk_get_rate(hclk);
		clk_set_rate(hclk, 24000);
		/* Set the voltage to 2.8V */
		otp_voltage_saved = __raw_readl(REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
		__raw_writel(
			(otp_voltage_saved & ~BM_POWER_VDDIOCTRL_TRG) | 0x00, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);

		r = otp_wait_busy(BM_OCOTP_CTRL_RD_BANK_OPEN);
		if (r < 0) {
			__raw_writel(otp_voltage_saved, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
			clk_set_rate(hclk, otp_hclk_saved);
			clk_put(hclk);
			err = -ETIMEDOUT;
			goto out;
		}

		clk_put(hclk);
	}

	else {
		pr_debug("%s: unknown mode '%d'\n", __func__, mode);
		err = -EINVAL;
		goto out;
	}

	otp_mode = mode;
	return 0;
out:
	mutex_unlock(&otp_mutex);
	pr_debug("%s: status %d\n", __func__, err);
	return err;
}

/**
 * otp_close - close the OTP bits after opening by otp_open
 **/
static void otp_close(void)
{
	struct clk *hclk;

	if (!mutex_is_locked(&otp_mutex)) {
		printk(KERN_ERR"%s: wasn't opened\n", __func__);
		return;
	}

	if (otp_mode == O_RDONLY) {
		/* 5. clear RD_BANK_OPEN */
		__raw_writel(BM_OCOTP_CTRL_RD_BANK_OPEN, REGS_OCOTP_BASE + HW_OCOTP_CTRL_CLR);
	}

	else if (otp_mode == O_WRONLY) {
		hclk = clk_get(NULL, "hclk");
		clk_set_rate(hclk, otp_hclk_saved);
		__raw_writel(otp_voltage_saved, REGS_POWER_BASE + HW_POWER_VDDIOCTRL);
		otp_wait_busy(0);
		__raw_writel(BM_OCOTP_CTRL_RELOAD_SHADOWS, REGS_OCOTP_BASE + HW_OCOTP_CTRL_SET);
		otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);
	}

	else {
		return; /* -EINVAL. Who does really check close? */
	}

	otp_mode = 0;
	mutex_unlock(&otp_mutex);
}

/**
 * otp_read_bits - read the content of OTP
 *
 * @start: offset from 0, in u32's
 * @len: number of OTP u32's to read
 * @bits: caller-allocated buffer to save bits
 * @size: size of @bits
 *
 * Returns number of u32's saved to buffer
 **/
static size_t otp_read_bits(int start, int len, u32 *bits, size_t size)
{
	int ofs;

	BUG_ON(!mutex_is_locked(&otp_mutex));

	/* read all stuff that caller needs */
	if (start + len > 4 * 8)  		/* 4 banks, 8 registers each */
		len = 4 * 8 - start;

	for (ofs = start; ofs < len; ofs++) {
		if (size/sizeof(*bits) <= 0)	/* we drained out the buffer */
			break;
		*bits = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CUSTn(ofs));
		bits++;
		size -= sizeof(*bits);
	}

	return ofs - start;	/* number of u32's that we saved to buffer */
}

/**
 * otp_write_bits - store OTP bits
 *
 * @offset: offset from 0, in u32's
 * @data: the u32 to write
 * @magic: the magic value to be stored in UNLOCK field
 *
 **/
static int otp_write_bits(int offset, u32 data, u32 magic)
{
	u32 c;
	int r;

	BUG_ON(!mutex_is_locked(&otp_mutex));

	if (offset < 0 || offset > 0x1F)
		return -EINVAL;

	c = __raw_readl(REGS_OCOTP_BASE + HW_OCOTP_CTRL);
	c &= ~BM_OCOTP_CTRL_ADDR;
	c |= BF(offset, OCOTP_CTRL_ADDR);
	c |= BF(magic, OCOTP_CTRL_WR_UNLOCK);
	__raw_writel(c, REGS_OCOTP_BASE + HW_OCOTP_CTRL);

	__raw_writel(data, REGS_OCOTP_BASE + HW_OCOTP_DATA);

	r = otp_wait_busy(0);
	if (r < 0)
		return r;

	udelay(2);
	return 0;
}

static ssize_t otp_id_show(void *context, char *page, int ascii)
{
	char s[60];
	int ret;
	int n, i, j, r;
	u32 otp_bits[4 * 8];

	r = otp_open(O_RDONLY);
	if (r < 0)
		return 0;
	n = otp_read_bits(0, 4 * 8, otp_bits, sizeof(otp_bits));
	otp_close();

	ret = 0;


	if (ascii) {

		strcpy(page, "");
		ret = 0;

		if (otp_full) {
			for (i = 0; i < 4; i++) {

				ret += sprintf(s, "Bank %d: ", i);
				strcat(page, s);

				for (j = 0; j < 8; j++) {

					if (i * 4 + j > n)
						break;
					ret += sprintf(s, "%08X ",
						otp_bits[i * 4 + j]);
					strcat(page, s);
				}

				strcat(page, "\n");
				ret++;
			}
		} else {
			for (i = 0; i < OTP_USER_SIZE; i++) {
				ret += sprintf(s, "%08X ",
					otp_bits[i + OTP_USER_OFFSET]);
				strcat(page, s);
			}
			strcat(page, "\n");
			ret++;
		}
	} else {

		if (otp_full) {
			memcpy(page, otp_bits, sizeof(otp_bits));
			ret = sizeof(otp_bits);
		} else {
			memcpy(page, otp_bits + OTP_USER_OFFSET,
					OTP_USER_SIZE * sizeof(u32));
			ret = OTP_USER_SIZE * sizeof(u32);
		}
	}

	return ret;
}

static int otp_check_dry_run(const char *page, size_t count)
{
	if (count >= 3 && memcmp(page, "+++", 3) == 0)
		return 3;
	return 0;
}

static ssize_t otp_id_store(void *context, const char *page,
				size_t count, int ascii)
{
	int r = 0;
	const char *p, *cp, *d;
	unsigned long index, value;
	char tmps[20];	/* subject of strtoul */
	int dry_run;

	r = otp_open(O_WRONLY);
	if (r < 0) {
		printk(KERN_ERR"Cannot open OTP in WRITE mode\n");
		return r;
	}

	if (ascii) {

		dry_run = otp_check_dry_run(page, count);
		if (dry_run > 0)
			page += dry_run;

		index = 0;
		cp = page;

		memset(tmps, 0, sizeof(tmps));

		for (index = 0, cp = page; cp != NULL; index++) {
			p = strchr(cp, ',');

			d = strchr(cp, ':');
			if (d && (!p || d < p)) {
				strncpy(tmps, cp,
					min_t(int, d - cp, sizeof(tmps) - 1));
				r = strict_strtoul(tmps, 0, &index);
				if (r < 0) {
					pr_debug("Cannot parse '%s'\n", tmps);
					break;
				}
				cp = d + 1;
			}

			memset(tmps, 0, sizeof(tmps));

			if (!p)
				strncpy(tmps, cp, sizeof(tmps));
			else
				strncpy(tmps, cp,
					min_t(int, p - cp, sizeof(tmps) - 1));
			r = strict_strtoul(tmps, 0, &value);
			if (r < 0) {
				pr_debug("Cannot parse '%s'\n", tmps);
				break;
			}

			memset(tmps, 0, sizeof(tmps));

			cp = p ? ++p : NULL;

			if (!otp_full) {
				index += OTP_USER_OFFSET;
				if (index > OTP_USER_SIZE) {
					printk(KERN_ERR"Cannot write at "
							"offset %ld\n", index);
					continue;
				}
			}

			r = 0;
			if (!dry_run) {
				pr_debug("Index %ld, value 0x%08lx\n",
						index, value);
				r = otp_write_bits(index, value, 0x3e77);
			} else
				printk(KERN_NOTICE
					"Dry-run: writing 0x%08lX => [%ld]\n",
						value, index);
			if (r < 0)
				break;
		}
	} else {
		printk(KERN_ERR"Binary write is not supported\n");
		r = -ENOSYS;
	}
	otp_close();
	return (r >= 0) ? count : r;
}

static struct uid_ops otp_ops = {
	.id_show = otp_id_show,
	.id_store = otp_id_store,
};

static int __init_or_module otp_init(void)
{
	void *p;

	mutex_init(&otp_mutex);
	p = uid_provider_init("otp", &otp_ops, NULL);
	if (IS_ERR(p))
		return PTR_ERR(p);
	return 0;
}

static void __exit otp_remove(void)
{
	uid_provider_remove("otp");
}

module_param(otp_full, int, 0600);
module_init(otp_init);
module_exit(otp_remove);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("dmitry pervushin <dimka@embeddedalley.com>");
MODULE_DESCRIPTION("Unique ID: OTP");
