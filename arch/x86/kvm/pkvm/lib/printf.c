// SPDX-License-Identifier: GPL-2.0
/* -*- linux-c -*- ------------------------------------------------------- *
 *
 *   Copyright (C) 1991, 1992 Linus Torvalds
 *   Copyright 2007 rPath, Inc. - All Rights Reserved
 *
 * ----------------------------------------------------------------------- */
#include <linux/stdarg.h>
#include <linux/kernel.h>
#include <linux/sprintf.h>
#include <asm/string.h>

static char *number(char *str, char *end, unsigned long long num, int base)
{
	static const char digits[16] = "0123456789abcdef";
	char tmp[20];
	int i = 0;

	if (num == 0) {
		tmp[i++] = '0';
	} else {
		while (num != 0) {
			tmp[i++] = digits[num % base];
			num /= base;
		}
	}

	while (i-- > 0 && str < end)
		*str++ = tmp[i];

	return str;
}

int vscnprintf(char *buf, size_t size, const char *fmt, va_list args)
{
	char *str = buf;
	char *end = buf + size;

	if (size == 0)
		return 0;

	/* Reserve space for null terminator */
	end--;

	for (; *fmt && str < end; ++fmt) {
		if (*fmt != '%') {
			*str++ = *fmt;
			continue;
		}

		fmt++;
		switch (*fmt) {
		case 's': {
			const char *s = va_arg(args, char *);

			if (!s)
				s = "(null)";
			while (*s && str < end)
				*str++ = *s++;
			break;
		}
		case 'd': {
			/* Use long long to safely handle negation of INT_MIN */
			long long num = va_arg(args, int);

			if (num < 0) {
				if (str < end)
					*str++ = '-';
				num = -num;
			}
			str = number(str, end, (unsigned long long)num, 10);
			break;
		}
		case 'u': {
			str = number(str, end, va_arg(args, unsigned int), 10);
			break;
		}
		case 'x': {
			str = number(str, end, va_arg(args, unsigned int), 16);
			break;
		}
		case 'l': {
			fmt++;
			if (*fmt == 'x')
				str = number(str, end, va_arg(args, unsigned long), 16);
			else if (*fmt == 'u')
				str = number(str, end, va_arg(args, unsigned long), 10);
			else if (*fmt == 'd') {
				/* Use long long to safely handle negation of INT_MIN */
				long long num = va_arg(args, long);

				if (num < 0) {
					if (str < end)
						*str++ = '-';
					num = -num;
				}
				str = number(str, end, (unsigned long long)num, 10);
			}
			break;
		}
		case 'p': {
			fmt++;
			/* Support %px, %pS, %p etc. as raw hex for now */
			if (str < end - 1) {
				*str++ = '0';
				*str++ = 'x';
			}
			str = number(str, end, (unsigned long)va_arg(args, void *), 16);
			break;
		}
		case '%': {
			*str++ = '%';
			break;
		}
		default:
			if (str < end)
				*str++ = '%';
			if (str < end)
				*str++ = *fmt;
			break;
		}
	}

	*str = '\0';
	return str - buf;
}

int scnprintf(char *buf, size_t size, const char *fmt, ...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = vscnprintf(buf, size, fmt, args);
	va_end(args);

	return i;
}
