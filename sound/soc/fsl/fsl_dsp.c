/*
 * Freescale DSP driver
 *
 * Copyright (c) 2012-2013 by Tensilica Inc. ALL RIGHTS RESERVED.
 * Copyright 2018-2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Copyright (c) 2001 William L. Pitts
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are freely
 * permitted provided that the above copyright notice and this
 * paragraph and the following disclaimer are duplicated in all
 * such forms.
 *
 * This software is provided "AS IS" and without any express or
 * implied warranties, including, without limitation, the implied
 * warranties of merchantability and fitness for a particular
 * purpose.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/file.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/platform_data/dma-imx.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/mx8_mu.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <uapi/linux/mxc_dsp.h>
#include <linux/firmware/imx/svc/misc.h>
#include <dt-bindings/firmware/imx/rsrc.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#include "fsl_dsp.h"
#include "fsl_dsp_pool.h"
#include "fsl_dsp_xaf_api.h"

/* ...allocate new client */
struct xf_client *xf_client_alloc(struct fsl_dsp *dsp_priv)
{
	struct xf_client *client;
	u32             id;

	id = dsp_priv->xf_client_map[0].next;

	/* ...try to allocate a client handle */
	if (id != 0) {
		/* ...allocate client memory */
		client = kmalloc(sizeof(*client), GFP_KERNEL);
		if (!client)
			return ERR_PTR(-ENOMEM);

		/* ...advance the head of free clients */
		dsp_priv->xf_client_map[0].next =
				dsp_priv->xf_client_map[id].next;

		/* ...put associate client id with given object */
		dsp_priv->xf_client_map[id].client = client;

		/* ...mark client is not yet bound to proxy */
		client->proxy = NULL;

		/* ...save global proxy client identifier */
		client->id = id;

		return client;
	}

	/* ...number of clients exceeded */
	return ERR_PTR(-EBUSY);
}

/* ...recycle client object */
static inline void xf_client_free(struct xf_client *client)
{
	int     id = client->id;
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)client->global;

	/* ...put proxy client id into free clients list */
	dsp_priv->xf_client_map[id].next = dsp_priv->xf_client_map[0].next;
	dsp_priv->xf_client_map[0].next = id;

	/* ...destroy client data */
	kfree(client);
}

/* ...lookup client basing on id */
struct xf_client *xf_client_lookup(struct fsl_dsp *dsp_priv, u32 id)
{
	if ((id >= XF_CFG_MAX_IPC_CLIENTS) ||
		(dsp_priv->xf_client_map[id].next < XF_CFG_MAX_IPC_CLIENTS)
	   )
		return NULL;
	else
		return dsp_priv->xf_client_map[id].client;
}

/* ...helper function for retrieving the client handle */
static inline struct xf_client *xf_get_client(struct file *file)
{
	struct xf_client *client;
	u32             id;

	client = (struct xf_client *)file->private_data;
	if (!client)
		return ERR_PTR(-EINVAL);

	id = client->id;
	if (id >= XF_CFG_MAX_IPC_CLIENTS)
		return ERR_PTR(-EINVAL);

	return client;
}

static int fsl_dsp_client_register(struct xf_client *client)
{
	struct fsl_dsp *dsp_priv;
	struct device *dev;

	dsp_priv = (struct fsl_dsp *)client->global;
	dev = dsp_priv->dev;

	/* ...make sure client is not registered yet */
	if (client->proxy != NULL) {
		pr_err("client-%x already registered", client->id);
		return -EBUSY;
	}

	/* ...complete association (no communication with remote proxy here) */
	client->proxy = &dsp_priv->proxy;

	pr_debug("client-%x registered within proxy", client->id);

	return 0;
}

/* ...unregister client from shared memory interface */
static int fsl_dsp_client_unregister(struct xf_client *client)
{
	struct xf_proxy *proxy = client->proxy;

	/* ...make sure client is registered */
	if (proxy == NULL) {
		pr_err("client-%x is not registered", client->id);
		return -EBUSY;
	}

	/* ...just clean proxy reference */
	client->proxy = NULL;

	pr_debug("client-%x registered within proxy", client->id);

	return 0;
}

static int fsl_dsp_ipc_msg_to_dsp(struct xf_client *client,
							void __user *user)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)client->global;
	struct device *dev = dsp_priv->dev;
	struct xf_proxy_message msg;
	void *buffer;
	unsigned long ret = 0;

	ret = copy_from_user(&msg, user, sizeof(struct xf_proxy_message));
	if (ret) {
		dev_err(dev, "failed to get message from user space\n");
		return -EFAULT;
	}

	/* ...make sure message pointer is sane */
	buffer = xf_proxy_a2b(&dsp_priv->proxy, msg.address);
	if (buffer == (void *)-1)
		return -EFAULT;

	/* ...put current proxy client into message session id */
	msg.session_id = XF_MSG_AP_FROM_USER(msg.session_id, client->id);

	xf_cmd_send(&dsp_priv->proxy,
				msg.session_id,
				msg.opcode,
				buffer,
				msg.length);

	return 0;
}

static int fsl_dsp_ipc_msg_from_dsp(struct xf_client *client,
							void __user *user)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)client->global;
	struct device *dev = dsp_priv->dev;
	struct xf_message *m;
	struct xf_proxy_message msg;
	unsigned long ret = 0;

	m = xf_cmd_recv(&dsp_priv->proxy, &client->wait, &client->queue, 0);
	if (IS_ERR(m)) {
		xf_unlock(&dsp_priv->proxy.lock);
		dev_err(dev, "receiving failed: %d", (int)PTR_ERR(m));
		return PTR_ERR(m);
	}

	/* ...check if there is a response available */
	if (m == NULL)
		return -EAGAIN;

	/* ...prepare message parameters (lock is taken) */
	msg.session_id = XF_MSG_AP_TO_USER(m->id);
	msg.opcode = m->opcode;
	msg.length = m->length;
	msg.address = xf_proxy_b2a(&dsp_priv->proxy, m->buffer);
	msg.ret = m->ret;

	/* ...return the message back to a pool and release lock */
	xf_msg_free(&dsp_priv->proxy, m);
	xf_unlock(&dsp_priv->proxy.lock);

	ret = copy_to_user(user, &msg, sizeof(struct xf_proxy_message));
	if (ret) {
		dev_err(dev, "failed to response message to user space\n");
		return -EFAULT;
	}

	return 0;
}

static int fsl_dsp_get_shmem_info(struct xf_client *client,
							void __user *user)
{
	struct fsl_dsp *dsp_priv = (struct fsl_dsp *)client->global;
	struct device *dev = dsp_priv->dev;
	struct shmem_info mem_info;
	unsigned long ret = 0;

	mem_info.phys_addr = dsp_priv->scratch_buf_phys;
	mem_info.size = dsp_priv->scratch_buf_size;

	ret = copy_to_user(user, &mem_info, sizeof(struct shmem_info));
	if (ret) {
		dev_err(dev, "failed to response message to user space\n");
		return -EFAULT;
	}

	return ret;
}

static struct miscdevice dsp_miscdev = {
	.name	= "mxc_hifi4",
	.minor	= MISC_DYNAMIC_MINOR,
};

static long fsl_dsp_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct xf_client *client;
	struct fsl_dsp *dsp_priv;
	struct xf_proxy  *proxy;
	struct device *dev;
	void __user *user;
	long ret = 0;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	dsp_priv = (struct fsl_dsp *)client->global;
	proxy = &dsp_priv->proxy;
	dev = dsp_priv->dev;
	user = (void __user *)arg;

	mutex_lock(&dsp_priv->dsp_mutex);

	if (!proxy->is_ready) {
		mutex_unlock(&dsp_priv->dsp_mutex);
		dev_err(dev, "dsp firmware is not ready\n");
		return -EFAULT;
	}

	switch (cmd) {
	case DSP_CLIENT_REGISTER:
		ret = fsl_dsp_client_register(client);
		break;
	case DSP_CLIENT_UNREGISTER:
		ret = fsl_dsp_client_unregister(client);
		break;
	case DSP_IPC_MSG_SEND:
		ret = fsl_dsp_ipc_msg_to_dsp(client, user);
		break;
	case DSP_IPC_MSG_RECV:
		ret = fsl_dsp_ipc_msg_from_dsp(client, user);
		break;
	case DSP_GET_SHMEM_INFO:
		ret = fsl_dsp_get_shmem_info(client, user);
		break;
	default:
		break;
	}

	mutex_unlock(&dsp_priv->dsp_mutex);

	return ret;
}

void resource_release(struct fsl_dsp *dsp_priv)
{
	int i;

	/* ...initialize client association map */
	for (i = 0; i < XF_CFG_MAX_IPC_CLIENTS - 1; i++)
		dsp_priv->xf_client_map[i].next = i + 1;
	/* ...set list terminator */
	dsp_priv->xf_client_map[i].next = 0;

	/* ...set pointer to shared memory */
	xf_proxy_init(&dsp_priv->proxy);
}

int fsl_dsp_open_func(struct fsl_dsp *dsp_priv, struct xf_client *client)
{
	struct device *dev = dsp_priv->dev;
	int ret = 0;

	/* ...initialize waiting queue */
	init_waitqueue_head(&client->wait);

	/* ...initialize client pending message queue */
	xf_msg_queue_init(&client->queue);

	/* ...mark user data is not mapped */
	client->vm_start = 0;

	/* ...reset mappings counter */
	atomic_set(&client->vm_use, 0);

	client->global = (void *)dsp_priv;
	dsp_priv->proxy.is_active = 1;

	pm_runtime_get_sync(dev);

	mutex_lock(&dsp_priv->dsp_mutex);
	/* increase reference counter when opening device */
	atomic_long_inc(&dsp_priv->refcnt);
	mutex_unlock(&dsp_priv->dsp_mutex);

	return ret;
}

static int fsl_dsp_open(struct inode *inode, struct file *file)
{
	struct fsl_dsp *dsp_priv = dev_get_drvdata(dsp_miscdev.parent);
	struct xf_client *client;
	int ret = 0;

	/* ...basic sanity checks */
	if (!inode || !file)
		return -EINVAL;

	/* ...allocate new proxy client object */
	client = xf_client_alloc(dsp_priv);
	if (IS_ERR(client))
		return PTR_ERR(client);

	fsl_dsp_open_func(dsp_priv, client);

	file->private_data = (void *)client;

	return ret;
}

static int fsl_dsp_wait_idle(struct fsl_dsp *dsp_priv)
{
	int timeout = 200;

	if (dsp_priv->dsp_is_lpa) {
		/* FW code is on OCRAM_A, Need wait DSP idle before gate */
		/* OCRAM_A clock. Or DSP will hang */
		while (!imx_audiomix_dsp_pwaitmode(dsp_priv->audiomix)) {
			if (!timeout--) {
				dev_err(dsp_priv->dev, "DSP failed to idle\n");
				return -ETIME;
			}
			udelay(5);
		}
	}

	return 0;
}

int fsl_dsp_close_func(struct xf_client *client)
{
	struct fsl_dsp *dsp_priv;
	struct device *dev;
	struct xf_proxy *proxy;

	/* ...basic sanity checks */
	proxy = client->proxy;

	/* release all pending messages */
	if (proxy)
		xf_msg_free_all(proxy, &client->queue);

	dsp_priv = (struct fsl_dsp *)client->global;

	/* wait until DSP idle */
	fsl_dsp_wait_idle(dsp_priv);

	dev = dsp_priv->dev;
	pm_runtime_put_sync(dev);

	/* ...recycle client id and release memory */
	xf_client_free(client);

	mutex_lock(&dsp_priv->dsp_mutex);
	/* decrease reference counter when closing device */
	atomic_long_dec(&dsp_priv->refcnt);
	/* If device is free, reinitialize the resource of
	 * dsp driver and framework
	 */
	if (atomic_long_read(&dsp_priv->refcnt) <= 0) {
		/* we are closing up, wait for proxy processing
		 * function to finish */
		cancel_work_sync(&dsp_priv->proxy.work);
		resource_release(dsp_priv);
	}

	mutex_unlock(&dsp_priv->dsp_mutex);

	return 0;
}

static int fsl_dsp_close(struct inode *inode, struct file *file)
{
	struct xf_client *client;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	fsl_dsp_close_func(client);

	return 0;
}

/* ...wait until data is available in the response queue */
static unsigned int fsl_dsp_poll(struct file *file, poll_table *wait)
{
	struct xf_proxy *proxy;
	struct xf_client *client;
	int mask;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* ...get proxy interface */
	proxy = client->proxy;
	if (!proxy)
		return -EPERM;

	/* ...register client waiting queue */
	poll_wait(file, &client->wait, wait);

	/* ...return current queue state */
	mask = (xf_msg_queue_head(&client->queue) ? POLLIN | POLLRDNORM : 0);

	return mask;
}

/*******************************************************************************
 * Low-level mmap interface
 ******************************************************************************/

/* ...add reference to shared buffer */
static void dsp_mmap_open(struct vm_area_struct *vma)
{
	struct xf_client *client = vma->vm_private_data;

	/* ...probably just increase counter of open references? - tbd */
	atomic_inc(&client->vm_use);

	pr_debug("xf_mmap_open: vma = %p, client = %p", vma, client);
}

/* ...close reference to shared buffer */
static void dsp_mmap_close(struct vm_area_struct *vma)
{
	struct xf_client *client = vma->vm_private_data;

	pr_debug("xf_mmap_close: vma = %p, b = %p", vma, client);

	/* ...decrement number of mapping */
	atomic_dec(&client->vm_use);
}

/* ...memory map operations */
static const struct vm_operations_struct dsp_mmap_ops = {
	.open   = dsp_mmap_open,
	.close  = dsp_mmap_close,
};

/* ...shared memory mapping */
static int fsl_dsp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct xf_proxy *proxy;
	struct xf_client *client;
	unsigned long   size;
	unsigned long   pfn;
	int             r;
	struct fsl_dsp *dsp_priv;

	/* ...basic sanity checks */
	client = xf_get_client(file);
	if (IS_ERR(client))
		return PTR_ERR(client);

	/* ...get proxy interface */
	proxy = client->proxy;
	if (!proxy)
		return -EPERM;

	/* ...check it was not mapped already */
	if (client->vm_start != 0)
		return -EBUSY;

	/* ...check mapping flags (tbd) */
	if ((vma->vm_flags & (VM_READ | VM_WRITE | VM_SHARED))
				!= (VM_READ | VM_WRITE | VM_SHARED))
		return -EPERM;

	/* ...set memory map operations */
	vma->vm_ops = &dsp_mmap_ops;

	/* ...assign private data */
	client->vm_start = vma->vm_start;

	/* ...set private memory data */
	vma->vm_private_data = client;

	/* ...set page number of shared memory */
	dsp_priv = (struct fsl_dsp *)client->global;
	pfn = dsp_priv->scratch_buf_phys >> PAGE_SHIFT;
	size = dsp_priv->scratch_buf_size;

	/* ...remap shared memory to user-space */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	r = remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot);
	if (r != 0) {
		pr_err("mapping failed: %d", r);
		return r;
	}

	/* ...system-specific hook for registering shared memory mapping */
	return 0;
}

void *memset_dsp(void *dest, int c, size_t count)
{
	uint *dl = (uint *)dest;
	void *dl_1, *dl_2;
	size_t align = 4;
	size_t n, n1, n2;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest) & (sizeof(*dl) - 1)) != 0) {
		dl = (unsigned int *)(((ulong)dest + align - 1) &
								(~(align - 1)));
		dl_1 = dest;
		dl_2 = (void *)(((ulong)dest + count) & (~(align - 1)));
		n1 = (ulong)dl - (ulong)dl_1;
		n2 = (ulong)dest + count - (ulong)dl_2;
		n = (count - n1 - n2) / align;

		while (n--) {
			writel_relaxed(0,  dl);
			dl++;
		}
		while (n1--) {
			writeb_relaxed(0, dl_1);
			dl_1++;
		}
		while (n2--) {
			writeb_relaxed(0, dl_2);
			dl_2++;
		}
	} else {
		n = count / align;
		n1 = count - n * align;
		dl_1 = dest + n * align;
		while (n--) {
			writel_relaxed(0,  dl);
			dl++;
		}
		while (n1--) {
			writeb_relaxed(0, dl_1);
			dl_1++;
		}
	}

	return dest;
}

void *memcpy_dsp(void *dest, const void *src, size_t count)
{
	unsigned int *dl = (unsigned int *)dest, *sl = (unsigned int *)src;
	size_t n = round_up(count, 4) / 4;

	if (src == dest)
		return dest;

	/* while all data is aligned (common case), copy a word at a time */
	if ((((ulong)dest | (ulong)src) & (sizeof(*dl) - 1)) != 0)
		pr_info("dest %p src %p not 4 bytes aligned\n", dest, src);

	while (n--) {
		writel_relaxed(*sl,  dl);
		dl++;
		sl++;
	}

	return dest;
}

static void fsl_dsp_start(struct fsl_dsp *dsp_priv)
{
	switch (dsp_priv->dsp_board_type){
	case DSP_IMX8QM_TYPE:
	case DSP_IMX8QXP_TYPE:
#if defined(CONFIG_IMX_SCU)
		imx_sc_pm_cpu_start(dsp_priv->dsp_ipcHandle,
				    IMX_SC_R_DSP, true, dsp_priv->iram);
#endif
		break;
	case DSP_IMX8MP_TYPE:
		imx_audiomix_dsp_start(dsp_priv->audiomix);
		break;
	default:
		break;
	}
}

static bool fsl_dsp_is_reset(struct fsl_dsp *dsp_priv)
{
	switch (dsp_priv->dsp_board_type) {
	case DSP_IMX8QM_TYPE:
	case DSP_IMX8QXP_TYPE:
		return true;
	case DSP_IMX8MP_TYPE:
		return imx_audiomix_dsp_reset(dsp_priv->audiomix);
	default:
		return true;
	}
}

static void dsp_load_firmware(const struct firmware *fw, void *context)
{
	struct fsl_dsp *dsp_priv = context;
	struct device *dev = dsp_priv->dev;
	Elf32_Ehdr *ehdr; /* Elf header structure pointer */
	Elf32_Shdr *shdr; /* Section header structure pointer */
	Elf32_Addr  sh_addr;
	unsigned char *strtab = 0; /* String table pointer */
	unsigned char *image; /* Binary image pointer */
	int i; /* Loop counter */
	unsigned long addr;

	if (!fw) {
		dev_info(dev, "external firmware not found\n");
		return;
	}

	addr = (unsigned long)fw->data;
	ehdr = (Elf32_Ehdr *)addr;

	/* Find the section header string table for output info */
	shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
			(ehdr->e_shstrndx * sizeof(Elf32_Shdr)));

	strtab = (unsigned char *)(addr + shdr->sh_offset);

	/* Load each appropriate section */
	for (i = 0; i < ehdr->e_shnum; ++i) {
		shdr = (Elf32_Shdr *)(addr + ehdr->e_shoff +
				(i * sizeof(Elf32_Shdr)));

		if (!(shdr->sh_flags & SHF_ALLOC) ||
			shdr->sh_addr == 0 || shdr->sh_size == 0)
			continue;

		dev_dbg(dev, "%sing %s @ 0x%08lx (%ld bytes)\n",
			(shdr->sh_type == SHT_NOBITS) ? "Clear" : "Load",
			&strtab[shdr->sh_name], (unsigned long)shdr->sh_addr,
			(long)shdr->sh_size);

		sh_addr = shdr->sh_addr;

		if (shdr->sh_type == SHT_NOBITS) {
			memset_dsp((void *)(dsp_priv->sdram_vir_addr +
				(sh_addr - dsp_priv->sdram_phys_addr)),
				0,
				shdr->sh_size);
		} else {
			image = (unsigned char *)addr + shdr->sh_offset;
			if ((!strcmp(&strtab[shdr->sh_name], ".rodata")) ||
				(!strcmp(&strtab[shdr->sh_name], ".text"))   ||
				(!strcmp(&strtab[shdr->sh_name], ".data"))   ||
				(!strcmp(&strtab[shdr->sh_name], ".bss"))
			) {
				memcpy_dsp((void *)(dsp_priv->sdram_vir_addr
				  + (sh_addr - dsp_priv->sdram_phys_addr)),
				  (const void *)image,
				  shdr->sh_size);
			} else {
				/* sh_addr is from DSP view, we need to
				 * fixup addr because we load the firmware from
				 * the ARM core side
				 */
				sh_addr -= dsp_priv->fixup_offset;

				memcpy_dsp((void *)(dsp_priv->regs +
						(sh_addr - dsp_priv->paddr)),
						(const void *)image,
						shdr->sh_size);
			}
		}
	}

	/* start the core */
	fsl_dsp_start(dsp_priv);
}

/* Initialization of the MU code. */
int dsp_mu_init(struct fsl_dsp *dsp_priv)
{
	struct device *dev = dsp_priv->dev;
	struct device_node *np;
	unsigned int	dsp_mu_id;
	u32 irq;
	int ret = 0;

	/*
	 * Get the address of MU to be used for communication with the dsp
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu-dsp");
	if (!np) {
		dev_err(dev, "Cannot find MU entry in device tree\n");
		return -EINVAL;
	}
	dsp_priv->mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!dsp_priv->mu_base_virtaddr);

	ret = of_property_read_u32_index(np,
				"fsl,dsp_ap_mu_id", 0, &dsp_mu_id);
	if (ret) {
		dev_err(dev, "Cannot get mu_id %d\n", ret);
		return -EINVAL;
	}

	dsp_priv->dsp_mu_id = dsp_mu_id;

	irq = of_irq_get(np, 0);

	ret = devm_request_irq(dsp_priv->dev, irq, fsl_dsp_mu_isr,
			IRQF_EARLY_RESUME, "dsp_mu_isr", &dsp_priv->proxy);
	if (ret) {
		dev_err(dev, "request_irq failed %d, err = %d\n", irq, ret);
		return -EINVAL;
	}

	if (dsp_priv->dsp_is_lpa) {
		ret = irq_set_irq_wake(irq, 1);
		if (ret) {
			dev_err(dev, "Failed to set IRQ %d as wake source: %d\n",
					irq, ret);
			return ret;
		}
	}

	return ret;
}

static const struct file_operations dsp_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= fsl_dsp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = fsl_dsp_ioctl,
#endif
	.open		= fsl_dsp_open,
	.poll		= fsl_dsp_poll,
	.mmap		= fsl_dsp_mmap,
	.release	= fsl_dsp_close,
};

extern struct snd_compr_ops dsp_platform_compr_lpa_ops;

static const struct snd_soc_component_driver dsp_soc_platform_lpa_drv  = {
	.name		= FSL_DSP_COMP_NAME,
	.compr_ops      = &dsp_platform_compr_lpa_ops,
};

extern struct snd_compr_ops dsp_platform_compr_ops;

static const struct snd_soc_component_driver dsp_soc_platform_drv  = {
	.name		= FSL_DSP_COMP_NAME,
	.compr_ops      = &dsp_platform_compr_ops,
};


int fsl_dsp_configure_audmix(struct fsl_dsp *dsp_priv) {
	struct device_node *np;
	struct platform_device *pdev;

	np = of_find_node_by_name(NULL, "audiomix_dsp");
	if (!np)
		return -EPROBE_DEFER;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return -EPROBE_DEFER;

	dsp_priv->audiomix = dev_get_drvdata(&pdev->dev);
	if (!dsp_priv->audiomix)
		return -EPROBE_DEFER;

	return 0;
}

int fsl_dsp_configure_scu(struct fsl_dsp *dsp_priv)
{
	int ret;

	/* there is no SCU on i.MX8MP */
	if (dsp_priv->dsp_board_type == DSP_IMX8MP_TYPE)
		return 0;

	ret = imx_scu_get_handle(&dsp_priv->dsp_ipcHandle);
	if (ret) {
		dev_err(dsp_priv->dev, "Cannot get scu handle %d\n", ret);
		return ret;
	}

	if (dsp_priv->dsp_board_type == DSP_IMX8QXP_TYPE) {
		ret = imx_sc_misc_set_control(dsp_priv->dsp_ipcHandle, IMX_SC_R_DSP,
					IMX_SC_C_OFS_SEL, 1);
		if (ret) {
			dev_err(dsp_priv->dev, "Error system address offset source select\n");
			return -EIO;
		}

		ret = imx_sc_misc_set_control(dsp_priv->dsp_ipcHandle, IMX_SC_R_DSP,
					IMX_SC_C_OFS_PERIPH, 0x5A);
		if (ret) {
			dev_err(dsp_priv->dev, "Error system address offset of PERIPH %d\n",
				ret);
			return -EIO;
		}

		ret = imx_sc_misc_set_control(dsp_priv->dsp_ipcHandle, IMX_SC_R_DSP,
					IMX_SC_C_OFS_IRQ, 0x51);
		if (ret) {
			dev_err(dsp_priv->dev, "Error system address offset of IRQ\n");
			return -EIO;
		}

		ret = imx_sc_misc_set_control(dsp_priv->dsp_ipcHandle, IMX_SC_R_DSP,
					IMX_SC_C_OFS_AUDIO, 0x80);
		if (ret) {
			dev_err(dsp_priv->dev, "Error system address offset of AUDIO\n");
			return -EIO;
		}
	} else {
		ret = imx_sc_misc_set_control(dsp_priv->dsp_ipcHandle, IMX_SC_R_DSP,
					IMX_SC_C_OFS_SEL, 0);
		if (ret) {
			dev_err(dsp_priv->dev, "Error system address offset source select\n");
			return -EIO;
		}
	}

	return 0;
}

int fsl_dsp_configure(struct fsl_dsp *dsp_priv)
{
	switch (dsp_priv->dsp_board_type) {
	case DSP_IMX8QM_TYPE:
	case DSP_IMX8QXP_TYPE:
		return fsl_dsp_configure_scu(dsp_priv);
	case DSP_IMX8MP_TYPE:
		return fsl_dsp_configure_audmix(dsp_priv);
	default:
		return -ENODEV;
	}
}

/**
 * fsl_dsp_attach_pm_domains
 */
static int fsl_dsp_attach_pm_domains(struct device *dev,
				     struct fsl_dsp *dsp)
{
	int ret;
	int i;

	if (dsp->num_domains <= 1)
		return 0;

	dsp->pd_dev = devm_kmalloc_array(dev, dsp->num_domains,
					 sizeof(*dsp->pd_dev),
					 GFP_KERNEL);
	if (!dsp->pd_dev)
		return -ENOMEM;

	dsp->pd_dev_link = devm_kmalloc_array(dev,
					      dsp->num_domains,
					      sizeof(*dsp->pd_dev_link),
					      GFP_KERNEL);
	if (!dsp->pd_dev_link)
		return -ENOMEM;

	for (i = 0; i < dsp->num_domains; i++) {
		dsp->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(dsp->pd_dev[i]))
			return PTR_ERR(dsp->pd_dev[i]);

		dsp->pd_dev_link[i] = device_link_add(dev,
						      dsp->pd_dev[i],
						      DL_FLAG_STATELESS |
						      DL_FLAG_PM_RUNTIME |
						      DL_FLAG_RPM_ACTIVE);
		if (IS_ERR(dsp->pd_dev_link[i])) {
			dev_pm_domain_detach(dsp->pd_dev[i], false);
			ret = PTR_ERR(dsp->pd_dev_link[i]);
			goto detach_pm;
		}
	}
	return 0;

detach_pm:
	while (--i >= 0) {
		device_link_del(dsp->pd_dev_link[i]);
		dev_pm_domain_detach(dsp->pd_dev[i], false);
	}
	return ret;
}

/**
 * fsl_dsp_detach_pm_domains
 */
static int fsl_dsp_detach_pm_domains(struct device *dev,
				     struct fsl_dsp *dsp)
{
	int i;

	if (dsp->num_domains <= 1)
		return 0;

	for (i = 0; i < dsp->num_domains; i++) {
		device_link_del(dsp->pd_dev_link[i]);
		dev_pm_domain_detach(dsp->pd_dev[i], false);
	}

	return 0;
}

static int fsl_dsp_mem_setup_lpa(struct fsl_dsp *dsp_priv)
{
	struct device_node *np = dsp_priv->dev->of_node;
	struct device_node *reserved_node;
	struct resource reserved_res;
	int offset;

	reserved_node = of_parse_phandle(np, "ocram", 0);
	if (!reserved_node) {
		dev_err(dsp_priv->dev, "failed to get reserved region node\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		dev_err(dsp_priv->dev, "failed to get reserved region address\n");
		return -EINVAL;
	}

	dsp_priv->ocram_phys_addr = reserved_res.start;
	dsp_priv->ocram_reserved_size = (reserved_res.end - reserved_res.start)
		+ 1;
	if (dsp_priv->ocram_reserved_size <= 0) {
		dev_err(dsp_priv->dev, "invalid value of reserved region size\n");
		return -EINVAL;
	}

	dsp_priv->ocram_vir_addr = ioremap_wc(dsp_priv->ocram_phys_addr,
			dsp_priv->ocram_reserved_size);
	if (!dsp_priv->ocram_vir_addr) {
		dev_err(dsp_priv->dev, "failed to remap ocram space for dsp firmware\n");
		return -ENXIO;
	}
	memset_io(dsp_priv->ocram_vir_addr, 0, dsp_priv->ocram_reserved_size);

	reserved_node = of_parse_phandle(np, "ocram-e", 0);
	if (!reserved_node) {
		dev_err(dsp_priv->dev, "failed to get reserved region node\n");
		return -ENODEV;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		dev_err(dsp_priv->dev, "failed to get reserved region address\n");
		return -EINVAL;
	}

	dsp_priv->ocram_e_phys_addr = reserved_res.start;
	dsp_priv->ocram_e_reserved_size = (reserved_res.end - reserved_res.start)
		+ 1;
	if (dsp_priv->ocram_e_reserved_size <= 0) {
		dev_err(dsp_priv->dev, "invalid value of reserved region size\n");
		return -EINVAL;
	}

	dsp_priv->ocram_e_vir_addr = ioremap_wc(dsp_priv->ocram_e_phys_addr,
			dsp_priv->ocram_e_reserved_size);
	if (!dsp_priv->ocram_e_vir_addr) {
		dev_err(dsp_priv->dev, "failed to remap ocram_e space for dsp firmware\n");
		return -ENXIO;
	}
	memset_io(dsp_priv->ocram_e_vir_addr, 0, dsp_priv->ocram_e_reserved_size);

	/* msg ring buffer memory */
	dsp_priv->msg_buf_virt = dsp_priv->ocram_e_vir_addr;
	dsp_priv->msg_buf_phys = dsp_priv->ocram_e_phys_addr;
	dsp_priv->msg_buf_size = MSG_BUF_SIZE;
	offset = MSG_BUF_SIZE;

	/* keep dsp framework's global data when suspend/resume */
	dsp_priv->dsp_config_virt = dsp_priv->ocram_e_vir_addr + offset;
	dsp_priv->dsp_config_phys = dsp_priv->ocram_e_phys_addr + offset;
	dsp_priv->dsp_config_size = DSP_CONFIG_SIZE;

	dsp_priv->scratch_buf_virt = dsp_priv->ocram_vir_addr;
	dsp_priv->scratch_buf_phys = dsp_priv->ocram_phys_addr;
	dsp_priv->scratch_buf_size = dsp_priv->ocram_reserved_size;
	dsp_priv->dram_reserved_vir_addr = dsp_priv->sdram_vir_addr;
	dsp_priv->dram_reserved_phys_addr = dsp_priv->sdram_phys_addr;
	dsp_priv->dram_reserved_size = dsp_priv->sdram_reserved_size;
	dsp_priv->sdram_vir_addr = dsp_priv->regs + SYSRAM_OFFSET;
	dsp_priv->sdram_phys_addr = dsp_priv->paddr + SYSRAM_OFFSET;
	dsp_priv->sdram_reserved_size = SYSRAM_SIZE;

	return 0;
}

static int fsl_dsp_mem_setup(struct fsl_dsp *dsp_priv)
{
	void *buf_virt;
	dma_addr_t buf_phys;
	int size, offset;

	size = MSG_BUF_SIZE + DSP_CONFIG_SIZE;
	buf_virt = dma_alloc_coherent(dsp_priv->dev, size, &buf_phys, GFP_KERNEL);
	if (!buf_virt) {
		dev_err(dsp_priv->dev, "failed alloc memory.\n");
		return -ENOMEM;
	}

	/* msg ring buffer memory */
	dsp_priv->msg_buf_virt = buf_virt;
	dsp_priv->msg_buf_phys = buf_phys;
	dsp_priv->msg_buf_size = MSG_BUF_SIZE;
	offset = MSG_BUF_SIZE;

	/* keep dsp framework's global data when suspend/resume */
	dsp_priv->dsp_config_virt = buf_virt + offset;
	dsp_priv->dsp_config_phys = buf_phys + offset;
	dsp_priv->dsp_config_size = DSP_CONFIG_SIZE;

	/* scratch memory for dsp framework. The sdram reserved memory
	 * is split into two equal parts currently. The front part is
	 * used to keep the dsp firmware, the other part is considered
	 * as scratch memory for dsp framework.
	 */
	dsp_priv->scratch_buf_virt = dsp_priv->sdram_vir_addr +
		dsp_priv->sdram_reserved_size / 2;
	dsp_priv->scratch_buf_phys = dsp_priv->sdram_phys_addr +
		dsp_priv->sdram_reserved_size / 2;
	dsp_priv->scratch_buf_size = dsp_priv->sdram_reserved_size / 2;

	return 0;
}

static int fsl_dsp_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *reserved_node;
	struct resource reserved_res;
	struct fsl_dsp *dsp_priv;
	const char *fw_name;
	const char *audio_iface;
	struct resource *res;
	void __iomem *regs;
	int size, i;
	int ret;
	char tmp[16];

	dsp_priv = devm_kzalloc(&pdev->dev, sizeof(*dsp_priv), GFP_KERNEL);
	if (!dsp_priv)
		return -ENOMEM;

	if (of_device_is_compatible(np, "fsl,imx8qxp-dsp-v1"))
		dsp_priv->dsp_board_type = DSP_IMX8QXP_TYPE;
	else if (of_device_is_compatible(np, "fsl,imx8qm-dsp-v1"))
		dsp_priv->dsp_board_type = DSP_IMX8QM_TYPE;
	else
		dsp_priv->dsp_board_type = DSP_IMX8MP_TYPE;

	if (of_device_is_compatible(np, "fsl,imx8mp-dsp-lpa")) {
		dsp_priv->dsp_board_type = DSP_IMX8MP_TYPE;
		dsp_priv->dsp_is_lpa = 1;
	}

	dsp_priv->dev = &pdev->dev;

	/* Get the addresses and IRQ */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	dsp_priv->paddr = res->start;
	dsp_priv->regs  = regs;

	dsp_priv->dram0 = dsp_priv->paddr + DRAM0_OFFSET;
	dsp_priv->dram1 = dsp_priv->paddr + DRAM1_OFFSET;
	dsp_priv->iram  = dsp_priv->paddr + IRAM_OFFSET;
	dsp_priv->sram  = dsp_priv->paddr + SYSRAM_OFFSET;

	dsp_priv->num_domains = of_count_phandle_with_args(np, "power-domains",
							   "#power-domain-cells");
	ret = fsl_dsp_attach_pm_domains(&pdev->dev, dsp_priv);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsp_priv);
	pm_runtime_enable(&pdev->dev);
	dsp_priv->dsp_mu_init = 1;
	dsp_priv->proxy.is_ready = 1;
	pm_runtime_get_sync(&pdev->dev);

	ret = fsl_dsp_configure(dsp_priv);
	if (ret < 0) {
		pm_runtime_put_sync(&pdev->dev);
		goto configure_fail;
	}

	ret = dsp_mu_init(dsp_priv);
	if (ret) {
		pm_runtime_put_sync(&pdev->dev);
		goto mu_init_fail;
	}

	pm_runtime_put_sync(&pdev->dev);
	dsp_priv->dsp_mu_init = 0;
	dsp_priv->proxy.is_ready = 0;

	ret = of_property_read_string(np, "fsl,dsp-firmware", &fw_name);
	dsp_priv->fw_name = fw_name;

	ret = of_property_read_string(np, "audio-interface", &audio_iface);
	dsp_priv->audio_iface = audio_iface;

	ret = of_property_read_u32(np, "fixup-offset", &dsp_priv->fixup_offset);

	if (!dsp_priv->dsp_is_lpa) {
		dsp_miscdev.fops = &dsp_fops,
			dsp_miscdev.parent = &pdev->dev,
			ret = misc_register(&dsp_miscdev);
		if (ret) {
			dev_err(&pdev->dev, "failed to register misc device %d\n", ret);
			goto misc_register_fail;
		}
	}

	reserved_node = of_parse_phandle(np, "memory-region", 0);
	if (!reserved_node) {
		dev_err(&pdev->dev, "failed to get reserved region node\n");
		ret = -ENODEV;
		goto reserved_node_fail;
	}

	if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
		dev_err(&pdev->dev, "failed to get reserved region address\n");
		ret = -EINVAL;
		goto reserved_node_fail;
	}

	dsp_priv->sdram_phys_addr = reserved_res.start;
	dsp_priv->sdram_reserved_size = (reserved_res.end - reserved_res.start)
									+ 1;
	if (dsp_priv->sdram_reserved_size <= 0) {
		dev_err(&pdev->dev, "invalid value of reserved region size\n");
		ret = -EINVAL;
		goto reserved_node_fail;
	}

	dsp_priv->sdram_vir_addr = ioremap_wc(dsp_priv->sdram_phys_addr,
						dsp_priv->sdram_reserved_size);
	if (!dsp_priv->sdram_vir_addr) {
		dev_err(&pdev->dev, "failed to remap sdram space for dsp firmware\n");
		ret = -ENXIO;
		goto reserved_node_fail;
	}
	memset_io(dsp_priv->sdram_vir_addr, 0, dsp_priv->sdram_reserved_size);

	size = MSG_BUF_SIZE + DSP_CONFIG_SIZE;

	if (dsp_priv->dsp_is_lpa) {
		ret = fsl_dsp_mem_setup_lpa(dsp_priv);
		if (ret) {
			dev_err(&pdev->dev, "lpa mem setup fail.\n");
			goto reserved_node_fail;
		}
	} else {
		if (fsl_dsp_mem_setup(dsp_priv)) {
			dev_err(&pdev->dev, "failed alloc memory.\n");
			ret = -ENOMEM;
			goto alloc_coherent_fail;
		}
	}

	/* initialize the reference counter for dsp_priv
	 * structure
	 */
	atomic_long_set(&dsp_priv->refcnt, 0);

	/* ...initialize client association map */
	for (i = 0; i < XF_CFG_MAX_IPC_CLIENTS - 1; i++)
		dsp_priv->xf_client_map[i].next = i + 1;
	/* ...set list terminator */
	dsp_priv->xf_client_map[i].next = 0;

	/* ...set pointer to shared memory */
	xf_proxy_init(&dsp_priv->proxy);

	/* ...initialize mutex */
	mutex_init(&dsp_priv->dsp_mutex);

	if (dsp_priv->dsp_is_lpa) {
		ret = devm_snd_soc_register_component(&pdev->dev, &dsp_soc_platform_lpa_drv, NULL, 0);
		if (ret) {
			dev_err(&pdev->dev, "registering soc platform failed\n");
			goto register_component_fail;
		}
	} else {
		ret = devm_snd_soc_register_component(&pdev->dev, &dsp_soc_platform_drv, NULL, 0);
		if (ret) {
			dev_err(&pdev->dev, "registering soc platform failed\n");
			goto register_component_fail;
		}
	}

	dsp_priv->esai_ipg_clk = devm_clk_get(&pdev->dev, "esai_ipg");
	if (IS_ERR(dsp_priv->esai_ipg_clk))
		dsp_priv->esai_ipg_clk = NULL;

	dsp_priv->esai_mclk = devm_clk_get(&pdev->dev, "esai_mclk");
	if (IS_ERR(dsp_priv->esai_mclk))
		dsp_priv->esai_mclk = NULL;

	dsp_priv->asrc_mem_clk = devm_clk_get(&pdev->dev, "asrc_mem");
	if (IS_ERR(dsp_priv->asrc_mem_clk))
		dsp_priv->asrc_mem_clk = NULL;

	dsp_priv->asrc_ipg_clk = devm_clk_get(&pdev->dev, "asrc_ipg");
	if (IS_ERR(dsp_priv->asrc_ipg_clk))
		dsp_priv->asrc_ipg_clk = NULL;

	for (i = 0; i < 4; i++) {
		sprintf(tmp, "asrck_%x", i);
		dsp_priv->asrck_clk[i] = devm_clk_get(&pdev->dev, tmp);
		if (IS_ERR(dsp_priv->asrck_clk[i]))
			dsp_priv->asrck_clk[i] = NULL;
	}

	dsp_priv->dsp_ocrama_clk = devm_clk_get(&pdev->dev, "ocram");
	if (IS_ERR(dsp_priv->dsp_ocrama_clk))
		dsp_priv->dsp_ocrama_clk = NULL;

	dsp_priv->audio_root_clk = devm_clk_get(&pdev->dev, "audio_root");
	if (IS_ERR(dsp_priv->audio_root_clk))
		dsp_priv->audio_root_clk = NULL;

	dsp_priv->audio_axi_clk = devm_clk_get(&pdev->dev, "audio_axi");
	if (IS_ERR(dsp_priv->audio_axi_clk))
		dsp_priv->audio_axi_clk = NULL;

	dsp_priv->dsp_root_clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(dsp_priv->dsp_root_clk))
		dsp_priv->dsp_root_clk = NULL;

	dsp_priv->debug_clk = devm_clk_get(&pdev->dev, "debug");
	if (IS_ERR(dsp_priv->debug_clk))
		dsp_priv->debug_clk = NULL;

	dsp_priv->mu2_clk = devm_clk_get(&pdev->dev, "mu2");
	if (IS_ERR(dsp_priv->mu2_clk))
		dsp_priv->mu2_clk = NULL;

	dsp_priv->sdma_root_clk = devm_clk_get(&pdev->dev, "sdma_root");
	if (IS_ERR(dsp_priv->sdma_root_clk))
		dsp_priv->sdma_root_clk = NULL;
	dsp_priv->sai_ipg_clk = devm_clk_get(&pdev->dev, "sai_ipg");
	if (IS_ERR(dsp_priv->sai_ipg_clk))
		dsp_priv->sai_ipg_clk = NULL;
	dsp_priv->sai_mclk = devm_clk_get(&pdev->dev, "sai_mclk");
	if (IS_ERR(dsp_priv->sai_mclk))
		dsp_priv->sai_mclk = NULL;
	dsp_priv->pll8k_clk = devm_clk_get(&pdev->dev, "pll8k");
	if (IS_ERR(dsp_priv->pll8k_clk))
		dsp_priv->pll8k_clk = NULL;
	dsp_priv->pll11k_clk = devm_clk_get(&pdev->dev, "pll11k");
	if (IS_ERR(dsp_priv->pll11k_clk))
		dsp_priv->pll11k_clk = NULL;
	dsp_priv->uart_ipg_clk = devm_clk_get(&pdev->dev, "uart_ipg");
	if (IS_ERR(dsp_priv->uart_ipg_clk))
		dsp_priv->uart_ipg_clk = NULL;
	dsp_priv->uart_per_clk = devm_clk_get(&pdev->dev, "uart_per");
	if (IS_ERR(dsp_priv->uart_per_clk))
		dsp_priv->uart_per_clk = NULL;

	return 0;

register_component_fail:
	dma_free_coherent(&pdev->dev, size, dsp_priv->msg_buf_virt,
				dsp_priv->msg_buf_phys);
alloc_coherent_fail:
	if (dsp_priv->sdram_vir_addr)
		iounmap(dsp_priv->sdram_vir_addr);
	if (dsp_priv->ocram_vir_addr)
		iounmap(dsp_priv->ocram_vir_addr);
	if (dsp_priv->ocram_e_vir_addr)
		iounmap(dsp_priv->ocram_e_vir_addr);

reserved_node_fail:
	if (!dsp_priv->dsp_is_lpa)
		misc_deregister(&dsp_miscdev);
misc_register_fail:
mu_init_fail:
configure_fail:
	pm_runtime_disable(&pdev->dev);
	fsl_dsp_detach_pm_domains(&pdev->dev, dsp_priv);
	return ret;
}

static int fsl_dsp_remove(struct platform_device *pdev)
{
	struct fsl_dsp *dsp_priv = platform_get_drvdata(pdev);
	int size;

	if (!dsp_priv->dsp_is_lpa)
		misc_deregister(&dsp_miscdev);

	size = MSG_BUF_SIZE + DSP_CONFIG_SIZE;
	dma_free_coherent(&pdev->dev, size, dsp_priv->msg_buf_virt,
				dsp_priv->msg_buf_phys);
	if (dsp_priv->sdram_vir_addr)
		iounmap(dsp_priv->sdram_vir_addr);
	if (dsp_priv->ocram_vir_addr)
		iounmap(dsp_priv->ocram_vir_addr);
	if (dsp_priv->ocram_e_vir_addr)
		iounmap(dsp_priv->ocram_e_vir_addr);

	pm_runtime_disable(&pdev->dev);
	fsl_dsp_detach_pm_domains(&pdev->dev, dsp_priv);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_dsp_runtime_resume(struct device *dev)
{
	struct fsl_dsp *dsp_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &dsp_priv->proxy;
	int ret;
	int i;

	ret = clk_prepare_enable(dsp_priv->esai_ipg_clk);
	if (ret) {
		dev_err(dev, "failed to enable esai ipg clock: %d\n", ret);
		goto esai_ipg_clk;
	}

	ret = clk_prepare_enable(dsp_priv->esai_mclk);
	if (ret) {
		dev_err(dev, "failed to enable esai mclk: %d\n", ret);
		goto esai_mclk;
	}

	ret = clk_prepare_enable(dsp_priv->asrc_mem_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable asrc_mem_clk ret = %d\n", ret);
		goto asrc_mem_clk;
	}

	ret = clk_prepare_enable(dsp_priv->asrc_ipg_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable asrc_ipg_clk ret = %d\n", ret);
		goto asrc_ipg_clk;
	}

	for (i = 0; i < 4; i++) {
		ret = clk_prepare_enable(dsp_priv->asrck_clk[i]);
		if (ret < 0) {
			dev_err(dev, "failed to prepare arc clk %d\n", i);
			goto asrck_clk;
		}
	}

	ret = clk_prepare_enable(dsp_priv->dsp_ocrama_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable dsp_ocrama_clk ret = %d\n", ret);
		goto ocrama_clk;
	}

	ret = clk_prepare_enable(dsp_priv->dsp_root_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable dsp_root_clk ret = %d\n", ret);
		goto dsp_root_clk;
	}

	ret = clk_prepare_enable(dsp_priv->audio_root_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable audio_root_clk ret = %d\n", ret);
		goto audio_root_clk;
	}

	ret = clk_prepare_enable(dsp_priv->audio_axi_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable audio_axi_clk ret = %d\n", ret);
		goto audio_axi_clk;
	}

	ret = clk_prepare_enable(dsp_priv->debug_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable debug_clk ret = %d\n", ret);
		goto debug_clk;
	}

	ret = clk_prepare_enable(dsp_priv->mu2_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable mu2_clk ret = %d\n", ret);
		goto mu2_clk;
	}

	ret = clk_prepare_enable(dsp_priv->sdma_root_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable sdma_root _clk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->sai_ipg_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable sai_ipg_clk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->sai_mclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable sai_mclk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->pll8k_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable pll8k_clk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->pll11k_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable pll11k_clk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->uart_ipg_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable uart_ipg_clk ret = %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(dsp_priv->uart_per_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable uart_per_clk ret = %d\n", ret);
		return ret;
	}

	if (!dsp_priv->dsp_mu_init && !proxy->is_ready && !fsl_dsp_is_reset(dsp_priv)) {
		dsp_priv->dsp_mu_init = 1;
		proxy->is_ready = 1;
	}

	/*
	 * Use PID for checking the audiomix is reset or not.
	 * After resetting, the PID should be 0, then we set the PID=1 in resume.
	 */
	if (!dsp_priv->dsp_mu_init && !proxy->is_ready && dsp_priv->dsp_board_type == DSP_IMX8MP_TYPE)
		imx_audiomix_dsp_pid_set(dsp_priv->audiomix, 0x1);

	if (!dsp_priv->dsp_mu_init) {
		MU_Init(dsp_priv->mu_base_virtaddr);
		MU_EnableRxFullInt(dsp_priv->mu_base_virtaddr, 0);
		dsp_priv->dsp_mu_init = 1;
	}

	if (!proxy->is_ready) {
		init_completion(&proxy->cmd_complete);

		ret = request_firmware_nowait(THIS_MODULE,
				FW_ACTION_HOTPLUG, dsp_priv->fw_name,
				dev,
				GFP_KERNEL, dsp_priv, dsp_load_firmware);

		if (ret) {
			dev_err(dev, "failed to load firmware\n");
			return ret;
		}

		ret = icm_ack_wait(proxy, 0);
		if (ret)
			return ret;

		dev_info(dev, "dsp driver registered\n");
	}

	return 0;

mu2_clk:
	clk_disable_unprepare(dsp_priv->debug_clk);
debug_clk:
	clk_disable_unprepare(dsp_priv->audio_axi_clk);
audio_axi_clk:
	clk_disable_unprepare(dsp_priv->audio_root_clk);
audio_root_clk:
	clk_disable_unprepare(dsp_priv->dsp_root_clk);
dsp_root_clk:
	clk_disable_unprepare(dsp_priv->dsp_ocrama_clk);
ocrama_clk:
	for (i = 0; i < 4; i++)
		clk_disable_unprepare(dsp_priv->asrck_clk[i]);
asrck_clk:
	clk_disable_unprepare(dsp_priv->asrc_ipg_clk);
asrc_ipg_clk:
	clk_disable_unprepare(dsp_priv->asrc_mem_clk);
asrc_mem_clk:
	clk_disable_unprepare(dsp_priv->esai_mclk);
esai_mclk:
	clk_disable_unprepare(dsp_priv->esai_ipg_clk);
esai_ipg_clk:
	return ret;
}

static int fsl_dsp_runtime_suspend(struct device *dev)
{
	struct fsl_dsp *dsp_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &dsp_priv->proxy;
	int i;

	dsp_priv->dsp_mu_init = 0;
	proxy->is_ready = 0;

	for (i = 0; i < 4; i++)
		clk_disable_unprepare(dsp_priv->asrck_clk[i]);

	clk_disable_unprepare(dsp_priv->asrc_ipg_clk);
	clk_disable_unprepare(dsp_priv->asrc_mem_clk);

	clk_disable_unprepare(dsp_priv->esai_mclk);
	clk_disable_unprepare(dsp_priv->esai_ipg_clk);

	clk_disable_unprepare(dsp_priv->dsp_ocrama_clk);
	clk_disable_unprepare(dsp_priv->dsp_root_clk);
	clk_disable_unprepare(dsp_priv->audio_root_clk);
	clk_disable_unprepare(dsp_priv->audio_axi_clk);
	clk_disable_unprepare(dsp_priv->debug_clk);
	clk_disable_unprepare(dsp_priv->mu2_clk);
	clk_disable_unprepare(dsp_priv->sdma_root_clk);
	clk_disable_unprepare(dsp_priv->sai_ipg_clk);
	clk_disable_unprepare(dsp_priv->sai_mclk);
	clk_disable_unprepare(dsp_priv->pll8k_clk);
	clk_disable_unprepare(dsp_priv->pll11k_clk);
	clk_disable_unprepare(dsp_priv->uart_ipg_clk);
	clk_disable_unprepare(dsp_priv->uart_per_clk);

	return 0;
}
#endif /* CONFIG_PM */


#ifdef CONFIG_PM_SLEEP
static int fsl_dsp_suspend(struct device *dev)
{
	struct fsl_dsp *dsp_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &dsp_priv->proxy;
	int ret = 0;

	if (dsp_priv->dsp_is_lpa)
		return ret;

	if (proxy->is_ready & pm_runtime_active(dev)) {
		ret = xf_cmd_send_suspend(proxy);
		if (ret) {
			dev_err(dev, "dsp suspend fail\n");
			return ret;
		}
	}

	ret = pm_runtime_force_suspend(dev);

	return ret;
}

static int fsl_dsp_resume(struct device *dev)
{
	struct fsl_dsp *dsp_priv = dev_get_drvdata(dev);
	struct xf_proxy *proxy = &dsp_priv->proxy;
	int ret = 0;

	if (dsp_priv->dsp_is_lpa)
		return ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	if (proxy->is_ready) {
		ret = xf_cmd_send_resume(proxy);
		if (ret) {
			dev_err(dev, "dsp resume fail\n");
			return ret;
		}
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_dsp_pm = {
	SET_RUNTIME_PM_OPS(fsl_dsp_runtime_suspend,
					fsl_dsp_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_dsp_suspend, fsl_dsp_resume)
};

static const struct of_device_id fsl_dsp_ids[] = {
	{ .compatible = "fsl,imx8qxp-dsp-v1", },
	{ .compatible = "fsl,imx8qm-dsp-v1", },
	{ .compatible = "fsl,imx8mp-dsp-v1", },
	{ .compatible = "fsl,imx8mp-dsp-lpa", },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_dsp_ids);

static struct platform_driver fsl_dsp_driver = {
	.probe = fsl_dsp_probe,
	.remove = fsl_dsp_remove,
	.driver = {
		.name = "fsl-dsp",
		.of_match_table = fsl_dsp_ids,
		.pm = &fsl_dsp_pm,
	},
};
module_platform_driver(fsl_dsp_driver);

MODULE_DESCRIPTION("Freescale DSP driver");
MODULE_ALIAS("platform:fsl-dsp");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
