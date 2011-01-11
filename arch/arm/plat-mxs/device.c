/*
 * Copyright (C) 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/gpmi-nfc.h>

#include <mach/device.h>

static int mxs_device_num;
static int mxs_device_done;
static DEFINE_MUTEX(device_mutex);
static struct list_head mxs_device_level[] = {
	LIST_HEAD_INIT(mxs_device_level[0]),
	LIST_HEAD_INIT(mxs_device_level[1]),
	LIST_HEAD_INIT(mxs_device_level[2]),
	LIST_HEAD_INIT(mxs_device_level[3]),
};

static u64 common_dmamask = DMA_BIT_MASK(32);

void mxs_nop_release(struct device *dev)
{
	/* Nothing */
}

int mxs_add_devices(struct platform_device *pdev, int num, int level)
{
	int i, ret = -ENOMEM;
	if (pdev == NULL || IS_ERR(pdev) || num <= 0)
		return -EINVAL;

	if (level < 0)
		level = 0;
	else if (level >= ARRAY_SIZE(mxs_device_level))
		level = ARRAY_SIZE(mxs_device_level) - 1;

	mutex_lock(&device_mutex);
	if (mxs_device_done) {
		ret = 0;
		for (i = 0; i < num; i++)
			ret |= platform_device_register(pdev + i);
		goto out;
	}

	if ((mxs_device_num + num) > MXS_MAX_DEVICES)
		goto out;
	mxs_device_num += num;
	for (i = 0; i < num; i++)
		list_add_tail(&pdev[i].dev.devres_head,
			      &mxs_device_level[level]);
	ret = 0;
out:
	mutex_unlock(&device_mutex);
	return ret;
}

int mxs_add_device(struct platform_device *pdev, int level)
{
	return mxs_add_devices(pdev, 1, level);
}

#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
static struct platform_device mxs_duart = {
	.name = "mxs-duart",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_MXS_DMA_ENGINE)
static struct platform_device mxs_dma[] = {
	{
	 .name = "mxs-dma-apbh",
	 .id = 0,
	 .dev = {
		 .release = mxs_nop_release,
		 },
	 },
	{
	 .name = "mxs-dma-apbx",
	 .id = 0,
	 .dev = {
		 .release = mxs_nop_release,
		 },
	 },
};
#endif

#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
static struct platform_device mxs_i2c[] = {
#if defined(CONFIG_I2C_MXS_SELECT0)
	{
	 .name	= "mxs-i2c",
	 .id	= 0,
	 .dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
#endif
#if defined(CONFIG_I2C_MXS_SELECT1)
	{
	 .name	= "mxs-i2c",
	 .id	= 1,
	 .dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
#endif
};
#endif

#if defined(CONFIG_MTD_NAND_GPMI_NFC) || \
	defined(CONFIG_MTD_NAND_GPMI_NFC_MODULE)
static struct platform_device gpmi_nfc = {
	.name = GPMI_NFC_DRIVER_NAME,
	.id = 0,
	.dev = {
		.dma_mask          = &common_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.release           = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_MMC_MXS) || \
	defined(CONFIG_MMC_MXS_MODULE)
static struct platform_device mxs_mmc[] = {
	{
	 .name	= "mxs-mmc",
	 .id	= 0,
	 .dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
	{
	 .name	= "mxs-mmc",
	 .id	= 1,
	 .dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
};
#endif

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
static struct platform_device mxs_spi[] = {
	{
	 .name	= "mxs-spi",
	 .id	= 0,
	 .dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
	 },
};
#endif

#if defined(CONFIG_MXS_WATCHDOG) || defined(CONFIG_MXS_WATCHDOG_MODULE)
static struct platform_device mxs_wdt = {
	.name = "mxs-wdt",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_FEC) || \
	defined(CONFIG_FEC_MODULE)
static struct platform_device mxs_fec[] = {
	{
	.name = "fec",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
	},
	{
	.name = "fec",
	.id = 1,
	.dev = {
		.release = mxs_nop_release,
		},
	},
};
#endif

#if defined(CONFIG_FEC_L2SWITCH)
static struct platform_device mxs_l2switch[] = {
	{
	.name = "mxs-l2switch",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
	},
};
#endif

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
static struct platform_device mxs_fb = {
	.name	= "mxs-fb",
	.id	= 0,
	.dev = {
		.dma_mask	       = &common_dmamask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_BACKLIGHT_MXS) || \
	defined(CONFIG_BACKLIGHT_MXS_MODULE)
struct platform_device mxs_bl = {
	.name	= "mxs-bl",
	.id	= 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_VIDEO_MXS_PXP) || \
	defined(CONFIG_VIDEO_MXS_PXP_MODULE)
static struct platform_device mxs_pxp = {
	.name		= "mxs-pxp",
	.id		= 0,
	.dev		= {
		.release = mxs_nop_release,
		.dma_mask		= &common_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_RTC_DRV_MXS) || defined(CONFIG_RTC_DRV_MXS_MODULE)
static struct platform_device mxs_rtc = {
	.name = "mxs-rtc",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#ifdef CONFIG_MXS_LRADC
static struct platform_device mxs_lradc = {
	.name = "mxs-lradc",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_KEYBOARD_MXS) || defined(CONFIG_KEYBOARD_MXS_MODULE)
static struct platform_device mxs_kbd = {
	.name = "mxs-kbd",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)
static struct platform_device mxs_ts = {
	.name = "mxs-ts",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_SERIAL_MXS_AUART) || defined(CONFIG_SERIAL_MXS_AUART_MODULE)
static struct platform_device mxs_auart[] = {
#ifdef CONFIG_MXS_AUART0_DEVICE_ENABLE
	{
	 .name = "mxs-auart",
	 .id = 0,
	 .dev = {
		 .release = mxs_nop_release,
		 .dma_mask = &common_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
		 },
	 },
#endif
#ifdef CONFIG_MXS_AUART1_DEVICE_ENABLE
	{
	 .name = "mxs-auart",
	 .id = 1,
	 .dev = {
		 .release = mxs_nop_release,
		 .dma_mask = &common_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
		 },
	 },
#endif
#ifdef CONFIG_MXS_AUART2_DEVICE_ENABLE
	{
	 .name = "mxs-auart",
	 .id = 2,
	 .dev = {
		 .release = mxs_nop_release,
		 .dma_mask = &common_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
		 },
	 },
#endif
#ifdef CONFIG_MXS_AUART3_DEVICE_ENABLE
	{
	 .name = "mxs-auart",
	 .id = 3,
	 .dev = {
		 .release = mxs_nop_release,
		 .dma_mask = &common_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
		 },
	 },
#endif
#ifdef CONFIG_MXS_AUART4_DEVICE_ENABLE
	{
	 .name = "mxs-auart",
	 .id = 4,
	 .dev = {
		 .release = mxs_nop_release,
		 .dma_mask = &common_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
		 },
	},
#endif
};
#endif

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct platform_device mxs_led = {
	.name = "mxs-leds",
	.id = 0,
	.dev = {
		 .release = mxs_nop_release,
		 },
};
#endif

#if defined(CONFIG_CAN_FLEXCAN) || \
	defined(CONFIG_CAN_FLEXCAN_MODULE)
static struct platform_device mxs_flexcan[] = {
	{
	.name = "FlexCAN",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
	},
	{
	.name = "FlexCAN",
	.id = 1,
	.dev = {
		.release = mxs_nop_release,
		},
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_DCP)
static struct platform_device mxs_dcp = {
	.name = "dcp",
	.id = 0,
	.dev	= {
		.release = mxs_nop_release,
		.dma_mask	= &common_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};
#endif

#if defined(CONFIG_BATTERY_MXS)
static struct platform_device mxs_battery = {
	.name   = "mxs-battery",
	.id = 0,
	.dev	= {
		.release = mxs_nop_release,
		} ,
};
#endif

#if defined(CONFIG_SND_SOC_SGTL5000) || \
	defined(CONFIG_SND_SOC_SGTL5000_MODULE)
static struct platform_device mxs_sgtl5000[] = {
	{
	.name = "mxs-sgtl5000",
	.id = 0,
	.dev =	{
		.release = mxs_nop_release,
		},
	},
};
#endif

#if defined(CONFIG_MXS_VIIM) || defined(CONFIG_MXS_VIIM_MODULE)
struct platform_device mxs_viim = {
	.name   = "mxs_viim",
	.id     = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
	defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
static struct platform_device mxs_spdif[] = {
	{
	.name = "mxs-spdif",
	.id = 0,
	.dev =	{
		.release = mxs_nop_release,
		},
	},
};
#endif

#if defined(CONFIG_SND_MXS_SOC_ADC) || \
	defined(CONFIG_SND_MXS_SOC_ADC_MODULE)
static struct platform_device mxs_adc = {
	.name = "mxs-adc-audio",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

static struct platform_device busfreq_device = {
	.name = "busfreq",
	.id = 0,
	.dev = {
		.release = mxs_nop_release,
		},
};

#ifdef CONFIG_MXS_PERSISTENT
static struct platform_device mxs_persistent = {
	.name			= "mxs-persistent",
	.id			= 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#ifdef CONFIG_MXS_PERFMON
static struct platform_device mxs_perfmon = {
	.name			= "mxs-perfmon",
	.id			= 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

#ifdef CONFIG_FSL_OTP
static struct platform_device otp_device = {
	.name			= "ocotp",
	.id			= 0,
	.dev = {
		.release = mxs_nop_release,
		},
};
#endif

static inline void mxs_init_busfreq(void)
{
	(void)platform_device_register(&busfreq_device);
}

static struct mxs_dev_lookup dev_lookup[] = {
#if defined(CONFIG_SERIAL_MXS_DUART) || \
	defined(CONFIG_SERIAL_MXS_DUART_MODULE)
	{
	 .name = "mxs-duart",
	 .size = 1,
	 .pdev = &mxs_duart,
	 },
#endif
#if defined(CONFIG_MXS_DMA_ENGINE)
	{
	 .name = "mxs-dma",
	 .size = ARRAY_SIZE(mxs_dma),
	 .pdev = mxs_dma,
	 },
#endif

#if defined(CONFIG_I2C_MXS) || \
	defined(CONFIG_I2C_MXS_MODULE)
	{
	 .name	= "mxs-i2c",
	 .size	= ARRAY_SIZE(mxs_i2c),
	 .pdev	= mxs_i2c,
	 },
#endif

#if defined(CONFIG_MTD_NAND_GPMI_NFC) || \
	defined(CONFIG_MTD_NAND_GPMI_NFC_MODULE)
	{
	.name = GPMI_NFC_DRIVER_NAME,
	.size = 1,
	.pdev = &gpmi_nfc,
	},
#endif

#if defined(CONFIG_MMC_MXS) || \
	defined(CONFIG_MMC_MXS_MODULE)
	{
	.name = "mxs-mmc",
	.size = ARRAY_SIZE(mxs_mmc),
	.pdev = mxs_mmc,
	},
#endif

#if defined(CONFIG_SPI_MXS) || defined(CONFIG_SPI_MXS_MODULE)
	{
	.name = "mxs-spi",
	.size = ARRAY_SIZE(mxs_spi),
	.pdev = mxs_spi,
	},
#endif

#if defined(CONFIG_MXS_WATCHDOG) || defined(CONFIG_MXS_WATCHDOG_MODULE)
	{
	 .name = "mxs-wdt",
	 .size = 1,
	 .pdev = &mxs_wdt,
	 },
#endif

#if defined(CONFIG_RTC_DRV_MXS) || defined(CONFIG_RTC_DRV_MXS_MODULE)
	{
	 .name = "mxs-rtc",
	 .size = 1,
	 .pdev = &mxs_rtc,
	 },
#endif

#if defined(CONFIG_MXS_PERSISTENT)
	{
	.name = "mxs-persistent",
	.size = 1,
	.pdev = &mxs_persistent,
	},
#endif

#if defined(CONFIG_MXS_PERFMON)
	{
	.name = "mxs-perfmon",
	.size = 1,
	.pdev = &mxs_perfmon,
	},
#endif

#if defined(CONFIG_FSL_OTP)
	{
	.name = "ocotp",
	.size = 1,
	.pdev = &otp_device,
	},
#endif

#if defined(CONFIG_FB_MXS) || defined(CONFIG_FB_MXS_MODULE)
	{
	 .name	= "mxs-fb",
	 .size	= 1,
	 .pdev	= &mxs_fb,
	 },
#endif
#if defined(CONFIG_BACKLIGHT_MXS) || \
	defined(CONFIG_BACKLIGHT_MXS_MODULE)
	{
	 .name	= "mxs-bl",
	 .size	= 1,
	 .pdev	= &mxs_bl,
	 },
#endif

#if defined(CONFIG_VIDEO_MXS_PXP) || \
	defined(CONFIG_VIDEO_MXS_PXP_MODULE)
	{
	 .name	= "mxs-pxp",
	 .size	= 1,
	 .pdev	= &mxs_pxp,
	 },
#endif

#if defined(CONFIG_MXS_VIIM) || defined(CONFIG_MXS_VIIM_MODULE)
	{
	 .name	= "mxs_viim",
	 .size	= 1,
	 .pdev	= &mxs_viim,
	 },
#endif

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	{
	.name = "mxs-fec",
	.size = ARRAY_SIZE(mxs_fec),
	.pdev = mxs_fec,
	},
#endif

#if defined(CONFIG_FEC_L2SWITCH)
	{
	.name = "mxs-l2switch",
	.size = ARRAY_SIZE(mxs_l2switch),
	.pdev = mxs_l2switch,
	},
#endif

#ifdef CONFIG_MXS_LRADC
	{
	 .name = "mxs-lradc",
	 .size = 1,
	 .pdev = &mxs_lradc,
	 },
#endif

#if defined(CONFIG_KEYBOARD_MXS) || defined(CONFIG_KEYBOARD_MXS_MODULE)
	{
	 .name = "mxs-kbd",
	 .size = 1,
	 .pdev = &mxs_kbd,
	 },
#endif

#if defined(CONFIG_TOUCHSCREEN_MXS) || defined(CONFIG_TOUCHSCREEN_MXS_MODULE)
	{
	 .name = "mxs-ts",
	 .size = 1,
	 .pdev = &mxs_ts,
	},
#endif

#if defined(CONFIG_SERIAL_MXS_AUART) || defined(CONFIG_SERIAL_MXS_AUART_MODULE)
	{
	 .name = "mxs-auart",
	 .size = ARRAY_SIZE(mxs_auart),
	 .pdev = mxs_auart,
	 },
#endif

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
	{
	 .name = "mxs-leds",
	 .size = 1,
	 .pdev = &mxs_led,
	 },
#endif

#if defined(CONFIG_CAN_FLEXCAN) || \
	defined(CONFIG_CAN_FLEXCAN_MODULE)
	{
	.name = "FlexCAN",
	.size = ARRAY_SIZE(mxs_flexcan),
	.pdev = mxs_flexcan,
	},
#endif

#if defined(CONFIG_CRYPTO_DEV_DCP)
	{
	.name = "dcp",
	.size = 1,
	.pdev = &mxs_dcp,
	},
#endif

#if defined(CONFIG_BATTERY_MXS)
	{
	 .name = "mxs-battery",
	 .size = 1,
	 .pdev = &mxs_battery,
	},
#endif

#if defined(CONFIG_SND_SOC_SGTL5000) || \
	defined(CONFIG_SND_SOC_SGTL5000_MODULE)
	{
	.name = "mxs-sgtl5000",
	.size = ARRAY_SIZE(mxs_sgtl5000),
	.pdev = mxs_sgtl5000,
	},
#endif

#if defined(CONFIG_SND_SOC_MXS_SPDIF) || \
	defined(CONFIG_SND_SOC_MXS_SPDIF_MODULE)
	{
	.name = "mxs-spdif",
	.size = ARRAY_SIZE(mxs_spdif),
	.pdev = mxs_spdif,
	},
#endif

#if defined(CONFIG_SND_MXS_SOC_ADC) || \
	defined(CONFIG_SND_MXS_SOC_ADC_MODULE)
	{
	.name = "mxs-adc",
	.size = 1,
	.pdev = &mxs_adc,
	},
#endif

};

struct platform_device *mxs_get_device(char *name, int id)
{
	int i, j;
	struct mxs_dev_lookup *lookup;
	struct platform_device *pdev = (struct platform_device *)-ENODEV;
	if (name == NULL || id < 0 || IS_ERR(name))
		return (struct platform_device *)-EINVAL;

	mutex_lock(&device_mutex);
	for (i = 0; i < ARRAY_SIZE(dev_lookup); i++) {
		lookup = &dev_lookup[i];
		if (!strcmp(name, lookup->name)) {
			if (test_bit(0, &lookup->lock)) {
				pdev = (struct platform_device *)-EBUSY;
				break;
			}

			if (id >= lookup->size)
				break;
			for (j = 0; j < lookup->size; j++) {
				if (id == (lookup->pdev[j]).id) {
					pdev = &lookup->pdev[j];
					break;
				}
			}
			break;
		}

	}
	mutex_unlock(&device_mutex);
	return pdev;
}

struct mxs_dev_lookup *mxs_get_devices(char *name)
{
	int i;
	struct mxs_dev_lookup *lookup;
	if (name == NULL || IS_ERR(name))
		return (struct mxs_dev_lookup *)-EINVAL;

	mutex_lock(&device_mutex);
	for (i = 0; i < ARRAY_SIZE(dev_lookup); i++) {
		lookup = &dev_lookup[i];
		if (!strcmp(name, lookup->name)) {
			if (test_and_set_bit(0, &lookup->lock))
				lookup = (struct mxs_dev_lookup *)-EBUSY;
			mutex_unlock(&device_mutex);
			return lookup;
		}

	}
	mutex_unlock(&device_mutex);
	return (struct mxs_dev_lookup *)-ENODEV;
}

int mxs_device_init(void)
{
	int i, ret = 0;
	struct list_head *p, *n;
	struct device *dev;
	struct platform_device *pdev;
	mutex_lock(&device_mutex);
	mxs_device_done = 1;
	mutex_unlock(&device_mutex);

	for (i = 0; i < ARRAY_SIZE(mxs_device_level); i++) {
		list_for_each_safe(p, n, mxs_device_level + i) {
			dev = list_entry(p, struct device, devres_head);
			list_del(p);
			pdev = container_of(dev, struct platform_device, dev);
			ret |= platform_device_register(pdev);
		}
	}

#if defined(CONFIG_BACKLIGHT_MXS) || \
	defined(CONFIG_BACKLIGHT_MXS_MODULE)
	platform_device_register(&mxs_bl);
#endif

	mxs_init_busfreq();
	return ret;
}

device_initcall(mxs_device_init);
