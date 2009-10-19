/*
 * Freescale STMP378X DCP driver for bootstream update. Only handles the OTP KEY
 * case and can only encrypt/decrypt.
 *
 * Author: Pantelis Antoniou <pantelis@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/stmp3xxx.h>
#include <mach/platform.h>
#include <mach/regs-dcp.h>
#include <mach/dcp_bootstream_ioctl.h>

/* use this channel (same as the ROM) */
#define ROM_DCP_CHAN 3

/* Defines the channel mask for the rom dcp channel */
#define ROM_DCP_CHAN_MASK (1 << ROM_DCP_CHAN)

/* Defines the initialization value for the dcp control register */
#define DCP_CTRL_INIT \
   (BM_DCP_CTRL_GATHER_RESIDUAL_WRITES | \
    BM_DCP_CTRL_ENABLE_CONTEXT_CACHING)

/* Defines the initialization value for the dcp channel control register */
#define DCP_CHANNELCTRL_INIT \
    BF(ROM_DCP_CHAN_MASK, DCP_CHANNELCTRL_ENABLE_CHANNEL)

/* DCP work packet 1 value used to calculate CBC-MAC over the image header */
#define DCP_PKT1_ENCRYPT \
   (BM_DCP_PACKET1_DECR_SEMAPHORE | \
    BM_DCP_PACKET1_ENABLE_CIPHER | \
    BM_DCP_PACKET1_CIPHER_ENCRYPT | \
    BM_DCP_PACKET1_CIPHER_INIT | \
    BM_DCP_PACKET1_OTP_KEY)

/* DCP work packet 1 value used to decrypt DEK in key dictionary */
#define DCP_PKT1_DECRYPT \
   (BM_DCP_PACKET1_DECR_SEMAPHORE | \
    BM_DCP_PACKET1_ENABLE_CIPHER | \
    BM_DCP_PACKET1_CIPHER_INIT | \
    BM_DCP_PACKET1_OTP_KEY)

/* DCP (decryption) work packet definition */
struct hw_dcp_packet {
	uint32_t pNext;     /* next dcp work packet address */
	uint32_t pkt1;      /* dcp work packet 1 (control 0) */
	uint32_t pkt2;      /* dcp work packet 2 (control 1) */
	uint32_t pSrc;      /* source buffer address */
	uint32_t pDst;      /* destination buffer address */
	uint32_t size;      /* buffer size in bytes */
	uint32_t pPayload;  /* payload buffer address */
	uint32_t stat;      /* dcp status (written by dcp) */
};

struct dma_area {
	struct hw_dcp_packet hw_packet;
	uint16_t block[16];
};

struct stmp3xxx_dcp_bootstream_data {
	struct device *dev;
	struct dma_area *dma_area;
	dma_addr_t dma_area_phys;
};

/* Only one instance allowed, so this is OK */
static struct stmp3xxx_dcp_bootstream_data *global_dbd;

static int stmp3xxx_dcp_bootstream_ioctl(struct inode *inode, struct file *file,
					 unsigned int cmd, unsigned long arg)
{
	struct stmp3xxx_dcp_bootstream_data *dbd = global_dbd;
	struct dma_area *da = dbd->dma_area;
	void __user *argp = (void __user *)arg;
	unsigned long timeout;

	/* be paranoid */
	if (dbd == NULL)
		return -EBADF;

	if (cmd != DBS_ENC && cmd != DBS_DEC)
		return -EINVAL;

	/* copy to (aligned) block */
	if (copy_from_user(da->block, argp, 16))
		return -EFAULT;

	/* Soft reset and remove the clock gate */
	stmp3xxx_setl(BM_DCP_CTRL_SFTRST, REGS_DCP_BASE + HW_DCP_CTRL);

	/* At 24Mhz, it takes no more than 4 clocks (160 ns) Maximum for
	 * the part to reset, reading the register twice should
	 * be sufficient to get 4 clks delay.
	 */
	__raw_readl(REGS_DCP_BASE + HW_DCP_CTRL);
	__raw_readl(REGS_DCP_BASE + HW_DCP_CTRL);

	stmp3xxx_clearl(BM_DCP_CTRL_SFTRST | BM_DCP_CTRL_CLKGATE,
			REGS_DCP_BASE + HW_DCP_CTRL);

	/* Initialize control registers */
	__raw_writel(DCP_CTRL_INIT, REGS_DCP_BASE + HW_DCP_CTRL);
	__raw_writel(DCP_CHANNELCTRL_INIT, REGS_DCP_BASE + HW_DCP_CHANNELCTRL);

	/* The loader does not enable context switching. Give the context
	 * buffer pointer an illegal address so if context switching is
	 * inadvertantly enabled, the dcp will return an error instead of
	 * trashing good memory. The dcp dma cannot access rom, so any rom
	 * address will do.
	 */
	__raw_writel(0xFFFF0000, REGS_DCP_BASE + HW_DCP_CONTEXT);

	stmp3xxx_clearl(-1, REGS_DCP_BASE + HW_DCP_CHnSTAT(ROM_DCP_CHAN));
	stmp3xxx_clearl(-1, REGS_DCP_BASE + HW_DCP_STAT);

	da->hw_packet.pNext = 0;
	da->hw_packet.pkt1 = BM_DCP_PACKET1_DECR_SEMAPHORE |
	    BM_DCP_PACKET1_ENABLE_CIPHER | BM_DCP_PACKET1_OTP_KEY |
	    BM_DCP_PACKET1_INTERRUPT |
	    (cmd == DBS_ENC ? BM_DCP_PACKET1_CIPHER_ENCRYPT : 0);
	da->hw_packet.pkt2 = BF(0, DCP_PACKET2_CIPHER_CFG) |
	    BF(0, DCP_PACKET2_KEY_SELECT) |
	    BF(BV_DCP_PACKET2_CIPHER_MODE__ECB, DCP_PACKET2_CIPHER_MODE) |
	    BF(BV_DCP_PACKET2_CIPHER_SELECT__AES128, DCP_PACKET2_CIPHER_SELECT);
	da->hw_packet.pSrc = dbd->dma_area_phys +
	    offsetof(struct dma_area, block);
	da->hw_packet.pDst = da->hw_packet.pSrc;	/* in-place */
	da->hw_packet.size = 16;
	da->hw_packet.pPayload = 0;
	da->hw_packet.stat = 0;

	/* Load the work packet pointer and bump the channel semaphore */
	__raw_writel(dbd->dma_area_phys +
		     offsetof(struct dma_area, hw_packet),
		     REGS_DCP_BASE + HW_DCP_CHnCMDPTR(ROM_DCP_CHAN));
	__raw_writel(BF(1, DCP_CHnSEMA_INCREMENT),
		     REGS_DCP_BASE + HW_DCP_CHnSEMA(ROM_DCP_CHAN));

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout) &&
	       (__raw_readl(REGS_DCP_BASE + HW_DCP_STAT) &
		BF(ROM_DCP_CHAN_MASK, DCP_STAT_IRQ)) == 0)
		cpu_relax();

	if (!time_before(jiffies, timeout)) {
		dev_err(dbd->dev, "Timeout while waiting STAT\n");
		return -ETIMEDOUT;
	}

	if ((__raw_readl(HW_DCP_CHnSTAT(ROM_DCP_CHAN)) & 0xff) != 0) {
		dev_err(dbd->dev, "Channel stat error 0x%02x\n",
			__raw_readl(REGS_DCP_BASE +
				    HW_DCP_CHnSTAT(ROM_DCP_CHAN)) & 0xff);
		return -EFAULT;
	}

	if (copy_to_user(argp, da->block, 16))
		return -EFAULT;

	return 0;
}

static struct file_operations stmp3xxx_dcp_bootstream_fops = {
	.owner =	THIS_MODULE,
	.ioctl =	stmp3xxx_dcp_bootstream_ioctl,
};

static struct miscdevice stmp3xxx_dcp_bootstream_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "dcpboot",
	.fops = &stmp3xxx_dcp_bootstream_fops,
};

static int __devinit stmp3xxx_dcp_bootstream_probe(struct platform_device *pdev)
{
	struct stmp3xxx_dcp_bootstream_data *dbd;
	int err;

	/* we only allow a single device */
	if (global_dbd != NULL)
		return -ENODEV;

	dbd = kzalloc(sizeof(*dbd), GFP_KERNEL);
	if (dbd == NULL)
		return -ENOMEM;
	memset(dbd, 0, sizeof(*dbd));

	dbd->dev = &pdev->dev;
	platform_set_drvdata(pdev, dbd);

	err = misc_register(&stmp3xxx_dcp_bootstream_misc);
	if (err != 0) {
		dev_err(&pdev->dev, "Unable to register misc device\n");
		goto err_done;
	}

	dbd->dma_area = dma_alloc_coherent(&pdev->dev, sizeof(*dbd->dma_area),
					   &dbd->dma_area_phys, GFP_KERNEL);
	if (dbd->dma_area == NULL) {
		dev_err(&pdev->dev, "Unable to allocate DMAable memory\n");
		goto err_dereg;
	}

	global_dbd = dbd;

	return 0;

err_dereg:
	misc_deregister(&stmp3xxx_dcp_bootstream_misc);
err_done:
	kfree(dbd);
	return err;
}

static int stmp3xxx_dcp_bootstream_remove(struct platform_device *pdev)
{
	struct stmp3xxx_dcp_bootstream_data *dbd;

	dbd = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	dma_free_coherent(&pdev->dev, sizeof(*dbd->dma_area),
			  dbd->dma_area, dbd->dma_area_phys);
	misc_deregister(&stmp3xxx_dcp_bootstream_misc);

	kfree(dbd);

	global_dbd = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int stmp3xxx_dcp_bootstream_suspend(struct platform_device *pdev,
					   pm_message_t state)
{
	return 0;
}

static int stmp3xxx_dcp_bootstream_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define stmp3xxx_dcp_bootstream_suspend	NULL
#define	stmp3xxx_dcp_bootstream_resume	NULL
#endif

static struct platform_driver stmp3xxx_dcp_bootstream_driver = {
	.probe		= stmp3xxx_dcp_bootstream_probe,
	.remove		= stmp3xxx_dcp_bootstream_remove,
	.suspend	= stmp3xxx_dcp_bootstream_suspend,
	.resume		= stmp3xxx_dcp_bootstream_resume,
	.driver		= {
		.name   = "stmp3xxx-dcpboot",
		.owner	= THIS_MODULE,
	},
};

static int __init stmp3xxx_dcp_bootstream_init(void)
{
	return platform_driver_register(&stmp3xxx_dcp_bootstream_driver);
}

static void __exit stmp3xxx_dcp_bootstream_exit(void)
{
	platform_driver_unregister(&stmp3xxx_dcp_bootstream_driver);
}

MODULE_AUTHOR("Pantelis Antoniou <pantelis@embeddedalley.com>");
MODULE_DESCRIPTION("DCP bootstream driver");
MODULE_LICENSE("GPL");

module_init(stmp3xxx_dcp_bootstream_init);
module_exit(stmp3xxx_dcp_bootstream_exit);
