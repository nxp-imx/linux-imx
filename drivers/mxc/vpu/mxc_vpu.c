/*
 * Copyright 2006-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_vpu.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/iram_alloc.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <asm/sizes.h>
#include <mach/clock.h>
#include <mach/hardware.h>

#include <mach/mxc_vpu.h>

struct vpu_priv {
	struct fasync_struct *async_queue;
	struct work_struct work;
	struct workqueue_struct *workqueue;
};

/* To track the allocated memory buffer */
typedef struct memalloc_record {
	struct list_head list;
	struct vpu_mem_desc mem;
} memalloc_record;

struct iram_setting {
	u32 start;
	u32 end;
};

static DEFINE_SPINLOCK(vpu_lock);
static LIST_HEAD(head);

static int vpu_major;
static int vpu_clk_usercount;
static struct class *vpu_class;
static struct vpu_priv vpu_data;
static u8 open_count;
static struct clk *vpu_clk;
static struct vpu_mem_desc bitwork_mem = { 0 };
static struct vpu_mem_desc pic_para_mem = { 0 };
static struct vpu_mem_desc user_data_mem = { 0 };
static struct vpu_mem_desc share_mem = { 0 };

static void __iomem *vpu_base;
static int vpu_irq;
static u32 phy_vpu_base_addr;
static struct mxc_vpu_platform_data *vpu_plat;

/* IRAM setting */
static struct iram_setting iram;

/* implement the blocking ioctl */
static int codec_done;
static wait_queue_head_t vpu_queue;

static u32 workctrl_regsave[6];
static u32 rd_ptr_regsave[4];
static u32 wr_ptr_regsave[4];
static u32 dis_flag_regsave[4];

#define	READ_REG(x)		__raw_readl(vpu_base + x)
#define	WRITE_REG(val, x)	__raw_writel(val, vpu_base + x)
#define	SAVE_WORK_REGS	do {					\
	int i;							\
	for (i = 0; i < ARRAY_SIZE(workctrl_regsave)/2; i++)	\
		workctrl_regsave[i] = READ_REG(BIT_WORK_CTRL_BUF_REG(i));\
} while (0)
#define	RESTORE_WORK_REGS	do {				\
	int i;							\
	for (i = 0; i < ARRAY_SIZE(workctrl_regsave)/2; i++)	\
		WRITE_REG(workctrl_regsave[i], BIT_WORK_CTRL_BUF_REG(i));\
} while (0)
#define	SAVE_CTRL_REGS	do {					\
	int i;							\
	for (i = ARRAY_SIZE(workctrl_regsave)/2;		\
			i < ARRAY_SIZE(workctrl_regsave); i++)		\
		workctrl_regsave[i] = READ_REG(BIT_WORK_CTRL_BUF_REG(i));\
} while (0)
#define	RESTORE_CTRL_REGS	do {				\
	int i;							\
	for (i = ARRAY_SIZE(workctrl_regsave)/2;		\
			i < ARRAY_SIZE(workctrl_regsave); i++)		\
		WRITE_REG(workctrl_regsave[i], BIT_WORK_CTRL_BUF_REG(i));\
} while (0)
#define	SAVE_RDWR_PTR_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(rd_ptr_regsave); i++)		\
		rd_ptr_regsave[i] = READ_REG(BIT_RD_PTR_REG(i));	\
	for (i = 0; i < ARRAY_SIZE(wr_ptr_regsave); i++)		\
		wr_ptr_regsave[i] = READ_REG(BIT_WR_PTR_REG(i));	\
} while (0)
#define	RESTORE_RDWR_PTR_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(rd_ptr_regsave); i++)		\
		WRITE_REG(rd_ptr_regsave[i], BIT_RD_PTR_REG(i));	\
	for (i = 0; i < ARRAY_SIZE(wr_ptr_regsave); i++)		\
		WRITE_REG(wr_ptr_regsave[i], BIT_WR_PTR_REG(i));	\
} while (0)
#define	SAVE_DIS_FLAG_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(dis_flag_regsave); i++)		\
		dis_flag_regsave[i] = READ_REG(BIT_FRM_DIS_FLG_REG(i));	\
} while (0)
#define	RESTORE_DIS_FLAG_REGS	do {					\
	int i;								\
	for (i = 0; i < ARRAY_SIZE(dis_flag_regsave); i++)		\
		WRITE_REG(dis_flag_regsave[i], BIT_FRM_DIS_FLG_REG(i));	\
} while (0)

/*!
 * Private function to alloc dma buffer
 * @return status  0 success.
 */
static int vpu_alloc_dma_buffer(struct vpu_mem_desc *mem)
{
	mem->cpu_addr = (unsigned long)
	    dma_alloc_coherent(NULL, PAGE_ALIGN(mem->size),
			       (dma_addr_t *) (&mem->phy_addr),
			       GFP_DMA | GFP_KERNEL);
	pr_debug("[ALLOC] mem alloc cpu_addr = 0x%x\n", mem->cpu_addr);
	if ((void *)(mem->cpu_addr) == NULL) {
		printk(KERN_ERR "Physical memory allocation error!\n");
		return -1;
	}
	return 0;
}

/*!
 * Private function to free dma buffer
 */
static void vpu_free_dma_buffer(struct vpu_mem_desc *mem)
{
	if (mem->cpu_addr != 0) {
		dma_free_coherent(0, PAGE_ALIGN(mem->size),
				  (void *)mem->cpu_addr, mem->phy_addr);
	}
}

/*!
 * Private function to free buffers
 * @return status  0 success.
 */
static int vpu_free_buffers(void)
{
	struct memalloc_record *rec, *n;
	struct vpu_mem_desc mem;

	list_for_each_entry_safe(rec, n, &head, list) {
		mem = rec->mem;
		if (mem.cpu_addr != 0) {
			vpu_free_dma_buffer(&mem);
			pr_debug("[FREE] freed paddr=0x%08X\n", mem.phy_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

static inline void vpu_worker_callback(struct work_struct *w)
{
	struct vpu_priv *dev = container_of(w, struct vpu_priv,
				work);

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	codec_done = 1;
	wake_up_interruptible(&vpu_queue);

	/*
	 * Clock is gated on when dec/enc started, gate it off when
	 * interrupt is received.
	 */
	clk_disable(vpu_clk);
}

/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;

	READ_REG(BIT_INT_STATUS);
	WRITE_REG(0x1, BIT_INT_CLEAR);

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
	spin_lock(&vpu_lock);
	if ((open_count++ == 0) && cpu_is_mx32())
		vl2cc_enable();
	filp->private_data = (void *)(&vpu_data);
	spin_unlock(&vpu_lock);
	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static int vpu_ioctl(struct inode *inode, struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0;

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
		{
			struct memalloc_record *rec;

			rec = kzalloc(sizeof(*rec), GFP_KERNEL);
			if (!rec)
				return -ENOMEM;

			ret = copy_from_user(&(rec->mem),
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				return -EFAULT;
			}

			pr_debug("[ALLOC] mem alloc size = 0x%x\n",
				 rec->mem.size);

			ret = vpu_alloc_dma_buffer(&(rec->mem));
			if (ret == -1) {
				kfree(rec);
				printk(KERN_ERR
				       "Physical memory allocation error!\n");
				break;
			}
			ret = copy_to_user((void __user *)arg, &(rec->mem),
					   sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				ret = -EFAULT;
				break;
			}

			spin_lock(&vpu_lock);
			list_add(&rec->list, &head);
			spin_unlock(&vpu_lock);

			break;
		}
	case VPU_IOC_PHYMEM_FREE:
		{
			struct memalloc_record *rec, *n;
			struct vpu_mem_desc vpu_mem;

			ret = copy_from_user(&vpu_mem,
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret)
				return -EACCES;

			pr_debug("[FREE] mem freed cpu_addr = 0x%x\n",
				 vpu_mem.cpu_addr);
			if ((void *)vpu_mem.cpu_addr != NULL) {
				vpu_free_dma_buffer(&vpu_mem);
			}

			spin_lock(&vpu_lock);
			list_for_each_entry_safe(rec, n, &head, list) {
				if (rec->mem.cpu_addr == vpu_mem.cpu_addr) {
					/* delete from list */
					list_del(&rec->list);
					kfree(rec);
					break;
				}
			}
			spin_unlock(&vpu_lock);

			break;
		}
	case VPU_IOC_WAIT4INT:
		{
			u_long timeout = (u_long) arg;
			if (!wait_event_interruptible_timeout
			    (vpu_queue, codec_done != 0,
			     msecs_to_jiffies(timeout))) {
				printk(KERN_WARNING "VPU blocking: timeout.\n");
				ret = -ETIME;
			} else if (signal_pending(current)) {
				printk(KERN_WARNING
				       "VPU interrupt received.\n");
				ret = -ERESTARTSYS;
			} else
				codec_done = 0;
			break;
		}
	case VPU_IOC_VL2CC_FLUSH:
		if (cpu_is_mx32()) {
			vl2cc_flush();
		}
		break;
	case VPU_IOC_IRAM_SETTING:
		{
			ret = copy_to_user((void __user *)arg, &iram,
					   sizeof(struct iram_setting));
			if (ret)
				ret = -EFAULT;

			break;
		}
	case VPU_IOC_CLKGATE_SETTING:
		{
			u32 clkgate_en;

			if (get_user(clkgate_en, (u32 __user *) arg))
				return -EFAULT;

			if (clkgate_en) {
				clk_enable(vpu_clk);
			} else {
				clk_disable(vpu_clk);
			}

			break;
		}
	case VPU_IOC_GET_SHARE_MEM:
		{
			spin_lock(&vpu_lock);
			if (share_mem.cpu_addr != 0) {
				ret = copy_to_user((void __user *)arg,
						   &share_mem,
						   sizeof(struct vpu_mem_desc));
				spin_unlock(&vpu_lock);
				break;
			} else {
				if (copy_from_user(&share_mem,
						   (struct vpu_mem_desc *)arg,
						 sizeof(struct vpu_mem_desc))) {
					spin_unlock(&vpu_lock);
					return -EFAULT;
				}
				if (vpu_alloc_dma_buffer(&share_mem) == -1)
					ret = -EFAULT;
				else {
					if (copy_to_user((void __user *)arg,
							 &share_mem,
							 sizeof(struct
								vpu_mem_desc)))
						ret = -EFAULT;
				}
			}
			spin_unlock(&vpu_lock);
			break;
		}
	case VPU_IOC_GET_WORK_ADDR:
		{
			if (bitwork_mem.cpu_addr != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &bitwork_mem,
						 sizeof(struct vpu_mem_desc));
				break;
			} else {
				if (copy_from_user(&bitwork_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct vpu_mem_desc)))
					return -EFAULT;

				if (vpu_alloc_dma_buffer(&bitwork_mem) == -1)
					ret = -EFAULT;
				else if (copy_to_user((void __user *)arg,
						      &bitwork_mem,
						      sizeof(struct
							     vpu_mem_desc)))
					ret = -EFAULT;
			}
			break;
		}
	case VPU_IOC_GET_PIC_PARA_ADDR:
		{
			if (pic_para_mem.cpu_addr != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &pic_para_mem,
						 sizeof(struct vpu_mem_desc));
				break;
			} else {
				if (copy_from_user(&pic_para_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct vpu_mem_desc)))
					return -EFAULT;

				if (vpu_alloc_dma_buffer(&pic_para_mem) == -1)
					ret = -EFAULT;
				else if (copy_to_user((void __user *)arg,
						      &pic_para_mem,
						      sizeof(struct
							     vpu_mem_desc)))
					ret = -EFAULT;
			}
			break;
		}
	case VPU_IOC_GET_USER_DATA_ADDR:
		{
			if (user_data_mem.cpu_addr != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &user_data_mem,
						 sizeof(struct vpu_mem_desc));
				break;
			} else {
				if (copy_from_user(&user_data_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct vpu_mem_desc)))
					return -EFAULT;

				if (vpu_alloc_dma_buffer(&user_data_mem) == -1)
					ret = -EFAULT;
				else if (copy_to_user((void __user *)arg,
						      &user_data_mem,
						      sizeof(struct
							     vpu_mem_desc)))
					ret = -EFAULT;
			}
			break;
		}
	case VPU_IOC_SYS_SW_RESET:
		{
			if (vpu_plat->reset)
				vpu_plat->reset();

			break;
		}
	case VPU_IOC_REG_DUMP:
		break;
	case VPU_IOC_PHYMEM_DUMP:
		break;
	default:
		{
			printk(KERN_ERR "No such IOCTL, cmd is %d\n", cmd);
			break;
		}
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	spin_lock(&vpu_lock);
	if (open_count > 0 && !(--open_count)) {
		vpu_free_buffers();

		if (cpu_is_mx32())
			vl2cc_disable();

		/* Free shared memory when vpu device is idle */
		vpu_free_dma_buffer(&share_mem);
		share_mem.cpu_addr = 0;
	}
	spin_unlock(&vpu_lock);

	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_priv *dev = (struct vpu_priv *)filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_noncached(vm->vm_page_prot);
	pfn = phy_vpu_base_addr >> PAGE_SHIFT;
	pr_debug("size=0x%x,  page no.=0x%x\n",
		 (int)(vm->vm_end - vm->vm_start), (int)pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_mem(struct file *fp, struct vm_area_struct *vm)
{
	int request_size;
	request_size = vm->vm_end - vm->vm_start;

	pr_debug(" start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vm->vm_start), (unsigned int)(vm->vm_pgoff),
		 request_size);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;

}

/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	if (vm->vm_pgoff)
		return vpu_map_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int vpu_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;
	unsigned long addr = 0;

	vpu_plat = pdev->dev.platform_data;

	if (VPU_IRAM_SIZE)
		iram_alloc(VPU_IRAM_SIZE, &addr);
	if (addr == 0)
		iram.start = iram.end = 0;
	else {
		iram.start = addr;
		iram.end = addr + VPU_IRAM_SIZE - 1;
	}

	if (cpu_is_mx32()) {
		err = vl2cc_init(iram.start);
		if (err != 0)
			return err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ERR "vpu: unable to get vpu base addr\n");
		return -ENODEV;
	}
	phy_vpu_base_addr = res->start;
	vpu_base = ioremap(res->start, res->end - res->start);

	vpu_major = register_chrdev(vpu_major, "mxc_vpu", &vpu_fops);
	if (vpu_major < 0) {
		printk(KERN_ERR "vpu: unable to get a major for VPU\n");
		err = -EBUSY;
		goto error;
	}

	vpu_class = class_create(THIS_MODULE, "mxc_vpu");
	if (IS_ERR(vpu_class)) {
		err = PTR_ERR(vpu_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(vpu_class, NULL, MKDEV(vpu_major, 0),
				   NULL, "mxc_vpu");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	vpu_clk = clk_get(&pdev->dev, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		err = -ENOENT;
		goto err_out_class;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		printk(KERN_ERR "vpu: unable to get vpu interrupt\n");
		err = -ENXIO;
		goto err_out_class;
	}
	vpu_irq = res->start;

	err = request_irq(vpu_irq, vpu_irq_handler, 0, "VPU_CODEC_IRQ",
			  (void *)(&vpu_data));
	if (err)
		goto err_out_class;

	vpu_data.workqueue = create_workqueue("vpu_wq");
	INIT_WORK(&vpu_data.work, vpu_worker_callback);
	printk(KERN_INFO "VPU initialized\n");
	goto out;

      err_out_class:
	device_destroy(vpu_class, MKDEV(vpu_major, 0));
	class_destroy(vpu_class);
      err_out_chrdev:
	unregister_chrdev(vpu_major, "mxc_vpu");
      error:
	iounmap(vpu_base);
	if (cpu_is_mx32()) {
		vl2cc_cleanup();
	}
      out:
	return err;
}

static int vpu_dev_remove(struct platform_device *pdev)
{
	free_irq(vpu_irq, &vpu_data);
	cancel_work_sync(&vpu_data.work);
	flush_workqueue(vpu_data.workqueue);
	destroy_workqueue(vpu_data.workqueue);

	iounmap(vpu_base);
	if (VPU_IRAM_SIZE)
		iram_free(iram.start, VPU_IRAM_SIZE);

	return 0;
}

#ifdef CONFIG_PM
static int vpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	unsigned long timeout;

	/* Wait for vpu go to idle state, suspect vpu cannot be changed
	   to idle state after about 1 sec */
	if (open_count > 0) {
		timeout = jiffies + HZ;
		clk_enable(vpu_clk);
		while (READ_REG(BIT_BUSY_FLAG)) {
			msleep(1);
			if (time_after(jiffies, timeout))
				goto out;
		}
		clk_disable(vpu_clk);
	}

	/* Make sure clock is disabled before suspend */
	vpu_clk_usercount = clk_get_usecount(vpu_clk);
	for (i = 0; i < vpu_clk_usercount; i++)
		clk_disable(vpu_clk);

	if (!cpu_is_mx53()) {
		clk_enable(vpu_clk);
		if (bitwork_mem.cpu_addr != 0) {
			SAVE_WORK_REGS;
			SAVE_CTRL_REGS;
			SAVE_RDWR_PTR_REGS;
			SAVE_DIS_FLAG_REGS;

			WRITE_REG(0x1, BIT_BUSY_FLAG);
			WRITE_REG(VPU_SLEEP_REG_VALUE, BIT_RUN_COMMAND);
			while (READ_REG(BIT_BUSY_FLAG))
				;
		}
		clk_disable(vpu_clk);
	}

	if (cpu_is_mx37() || cpu_is_mx51())
		mxc_pg_enable(pdev);

	return 0;

out:
	clk_disable(vpu_clk);
	return -EAGAIN;

}

static int vpu_resume(struct platform_device *pdev)
{
	int i;

	if (cpu_is_mx37() || cpu_is_mx51())
		mxc_pg_disable(pdev);

	if (cpu_is_mx53())
		goto recover_clk;

	clk_enable(vpu_clk);
	if (bitwork_mem.cpu_addr != 0) {
		u32 *p = (u32 *) bitwork_mem.cpu_addr;
		u32 data;
		u16 data_hi;
		u16 data_lo;

		RESTORE_WORK_REGS;

		WRITE_REG(0x0, BIT_RESET_CTRL);
		WRITE_REG(0x0, BIT_CODE_RUN);

		/*
		 * Re-load boot code, from the codebuffer in external RAM.
		 * Thankfully, we only need 4096 bytes, same for all platforms.
		 */
		if (cpu_is_mx51()) {
			for (i = 0; i < 2048; i += 4) {
				data = p[(i / 2) + 1];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
					  BIT_CODE_DOWN);

				data = p[i / 2];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG(((i + 2) << 16) | data_hi,
					  BIT_CODE_DOWN);
				WRITE_REG(((i + 3) << 16) | data_lo,
					  BIT_CODE_DOWN);
			}
		} else {
			for (i = 0; i < 2048; i += 2) {
				if (cpu_is_mx37())
					data = swab32(p[i / 2]);
				else
					data = p[i / 2];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;

				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
					  BIT_CODE_DOWN);
			}
		}

		RESTORE_CTRL_REGS;

		WRITE_REG(BITVAL_PIC_RUN, BIT_INT_ENABLE);

		WRITE_REG(0x1, BIT_BUSY_FLAG);
		WRITE_REG(0x1, BIT_CODE_RUN);
		while (READ_REG(BIT_BUSY_FLAG))
			;

		RESTORE_RDWR_PTR_REGS;
		RESTORE_DIS_FLAG_REGS;

		WRITE_REG(0x1, BIT_BUSY_FLAG);
		WRITE_REG(VPU_WAKE_REG_VALUE, BIT_RUN_COMMAND);
		while (READ_REG(BIT_BUSY_FLAG))
			;
	}
	clk_disable(vpu_clk);

recover_clk:
	/* Recover vpu clock */
	for (i = 0; i < vpu_clk_usercount; i++)
		clk_enable(vpu_clk);

	return 0;
}
#else
#define	vpu_suspend	NULL
#define	vpu_resume	NULL
#endif				/* !CONFIG_PM */

/*! Driver definition
 *
 */
static struct platform_driver mxcvpu_driver = {
	.driver = {
		   .name = "mxc_vpu",
		   },
	.probe = vpu_dev_probe,
	.remove = vpu_dev_remove,
	.suspend = vpu_suspend,
	.resume = vpu_resume,
};

static int __init vpu_init(void)
{
	int ret = platform_driver_register(&mxcvpu_driver);

	init_waitqueue_head(&vpu_queue);

	return ret;
}

static void __exit vpu_exit(void)
{
	if (vpu_major > 0) {
		device_destroy(vpu_class, MKDEV(vpu_major, 0));
		class_destroy(vpu_class);
		unregister_chrdev(vpu_major, "mxc_vpu");
		vpu_major = 0;
	}

	if (cpu_is_mx32()) {
		vl2cc_cleanup();
	}

	vpu_free_dma_buffer(&bitwork_mem);
	vpu_free_dma_buffer(&pic_para_mem);
	vpu_free_dma_buffer(&user_data_mem);

	clk_put(vpu_clk);

	platform_driver_unregister(&mxcvpu_driver);
	return;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_init(vpu_init);
module_exit(vpu_exit);
