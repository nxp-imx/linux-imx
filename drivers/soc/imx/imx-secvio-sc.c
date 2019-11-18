// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 */

/*
 * This module interact with the SCU which relay request to/from the SNVS block
 * to detect if security violation occurred.
 *
 * The module exports an API to add processing when a SV is detected:
 *  - register_imx_secvio_sc_notifier
 *  - unregister_imx_secvio_sc_notifier
 *
 * The module exposes 2 files in debugfs:
 *  - secvio/info:
 *      * Read: It returns the value of the fuses and SNVS registers which are
 *              readable and related to secvio and tampers
 *      * Write: A write of the format "<hex id> [<hex value 0> <hex value 1>
 *               <hex value 2> <hex value 3> <hex value 4>](<nb values>)"
 *               will write the SNVS register having the provided id with the
 *               values provided (cf SECO ducumentation)
 *  - secvio/enable: State of the IRQ
 */

/* Includes */
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/audit.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/svc/seco/api.h>

#include <soc/imx8/imx-secvio-sc.h>

/* Definitions */

/* Access for sc_seco_secvio_config API */
#define SECVIO_CONFIG_READ  0
#define SECVIO_CONFIG_WRITE 1

/* Register IDs for sc_seco_secvio_config API */
#define HPSVS_ID 0x18
#define LPS_ID 0x4c
#define LPTDS_ID 0xa4
#define HPVIDR_ID 0xf8

/* Forward declarations */
static int imx_secvio_sc_process_state(struct device *dev);
static int enable_irq(struct device *dev);
static int disable_irq(struct device *dev);
static void imx_secvio_sc_debugfs_remove(struct device *dev);
static int imx_secvio_sc_teardown(struct device *dev);
static int call_secvio_config(struct device *dev, u8 id, u8 access, u32 *data0,
			      u32 *data1, u32 *data2, u32 *data3, u32 *data4,
			      u8 size);

/* Notifier list for new CB */
static BLOCKING_NOTIFIER_HEAD(imx_secvio_sc_notifier_chain);

/* Internal Structure */
struct imx_secvio_sc_data {
	struct device *dev;
	sc_ipc_t ipcHandle;
	struct notifier_block irq_nb;
	struct notifier_block report_nb;
	struct notifier_block audit_nb;

	u8 enabled;

#ifdef CONFIG_DEBUG_FS
	u8 *io_buffer;
	size_t io_buffer_size;

	struct dentry *dfs;
	struct semaphore sem;
#endif /* CONFIG_DEBUG_FS */

	u32 version;
};

/* Mapping of error code returned by SCU */
struct mapping_sci_err {
	const char *txt;
	u8 sciErr;
	u32 osErr;
};

const struct mapping_sci_err sci_err_list[] = {
	{"None", SC_ERR_NONE, 0},
	{"Wrong version", SC_ERR_VERSION, -EDOM},
	{"Wrong config", SC_ERR_CONFIG, -ERANGE},
	{"Bad parameter", SC_ERR_PARM, -EINVAL},
	{"No access", SC_ERR_NOACCESS, -EACCES},
	{"Locked", SC_ERR_LOCKED, -EROFS},
	{"Unavailable", SC_ERR_UNAVAILABLE, -ENAVAIL},
	{"Not found", SC_ERR_NOTFOUND, -ENOENT},
	{"No power", SC_ERR_NOPOWER, -ECOMM},
	{"Issue IPC", SC_ERR_IPC, -EPROTO},
	{"Busy", SC_ERR_BUSY, -EBUSY},
	{"Fail", SC_ERR_FAIL, -EFAULT},
};

/* Code */
static int sci_err_to_errno(sc_err_t sciErr, const char **txt)
{
	int ret = -ENOMSG;
	int idx;
	const struct mapping_sci_err *mapping;
	const char *ret_txt = "UNKNOWN";

	for (idx = 0; idx < ARRAY_SIZE(sci_err_list); idx++) {
		mapping = &sci_err_list[idx];

		if (sciErr == mapping->sciErr) {
			ret = mapping->osErr;
			ret_txt = mapping->txt;
			break;
		}
	}

	if (txt)
		*txt = ret_txt;

	return ret;
}

int register_imx_secvio_sc_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&imx_secvio_sc_notifier_chain,
						nb);
}
EXPORT_SYMBOL(register_imx_secvio_sc_notifier);

int unregister_imx_secvio_sc_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&imx_secvio_sc_notifier_chain,
						  nb);
}
EXPORT_SYMBOL(unregister_imx_secvio_sc_notifier);

static int imx_secvio_sc_notifier_call_chain(struct notifier_info *info)
{
	return blocking_notifier_call_chain(&imx_secvio_sc_notifier_chain,
					    0, (void *)info);
}

static int imx_secvio_sc_notify(struct notifier_block *nb,
				unsigned long event, void *group)
{
	int ret = 0;
	struct imx_secvio_sc_data *data =
				container_of(nb, struct imx_secvio_sc_data,
					     irq_nb);
	struct device *dev = data->dev;

	/* Filter event for us */
	if (!((event & SC_IRQ_SECVIO) &&
	      (*(sc_irq_group_t *)group == SC_IRQ_GROUP_WAKE)))
		goto exit;

	dev_warn(dev, "secvio security violation detected\n");

	ret = imx_secvio_sc_process_state(dev);

exit:
	return ret;
}

static int imx_secvio_sc_process_state(struct device *dev)
{
	int ret = 0;
	struct notifier_info info = {0};

	/* Read secvio status */
	ret = call_secvio_config(dev, HPSVS_ID, SECVIO_CONFIG_READ,
				 &info.sv_status, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot read secvio status: %d\n", ret);
	info.sv_status &= HPSVS__ALL_SV__MASK;

	/* Read tampers status */
	ret = call_secvio_config(dev, LPS_ID, SECVIO_CONFIG_READ,
				 &info.tp_status_1, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot read tamper 1 status: %d\n", ret);
	info.tp_status_1 &= LPS__ALL_TP__MASK;

	ret = call_secvio_config(dev, LPTDS_ID, SECVIO_CONFIG_READ,
				 &info.tp_status_2, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot read  tamper 2 status: %d\n", ret);
	info.tp_status_2 &= LPTDS__ALL_TP__MASK;

	dev_dbg(dev, "Status: %.8x, %.8x, %.8x\n", info.sv_status,
		info.tp_status_1, info.tp_status_2);

	/* Clear status */
	ret = call_secvio_config(dev, HPSVS_ID, SECVIO_CONFIG_WRITE,
				 &info.sv_status, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot clear secvio status: %d\n", ret);

	ret = call_secvio_config(dev, LPS_ID, SECVIO_CONFIG_WRITE,
				 &info.tp_status_1, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot clear temper 1 status: %d\n", ret);

	ret = call_secvio_config(dev, LPTDS_ID, SECVIO_CONFIG_WRITE,
				 &info.tp_status_2, NULL, NULL, NULL, NULL, 1);
	if (ret)
		dev_err(dev, "Cannot clear tamper 2 status: %d\n", ret);

	/* Re-enable interrupt */
	ret = enable_irq(dev);
	if (ret)
		dev_err(dev, "Failed to enable IRQ\n");

	/* Call chain of CB registered to this module */
	if (imx_secvio_sc_notifier_call_chain(&info))
		dev_warn(dev, "Issues when calling the notifier chain\n");

	return ret;
}

static int report_to_user_notify(struct notifier_block *nb,
				 unsigned long status, void *notif_info)
{
	struct notifier_info *info = notif_info;
	struct imx_secvio_sc_data *data =
				container_of(nb, struct imx_secvio_sc_data,
					     report_nb);
	struct device *dev = data->dev;

	/* Inform what is the cause of the interrupt */
	if (info->sv_status & HPSVS__LP_SEC_VIO__MASK) {
		dev_info(dev, "SNVS secvio: LPSV\n");
	} else if (info->sv_status & HPSVS__SW_LPSV__MASK) {
		dev_info(dev, "SNVS secvio: SW LPSV\n");
	} else if (info->sv_status & HPSVS__SW_FSV__MASK) {
		dev_info(dev, "SNVS secvio: SW FSV\n");
	} else if (info->sv_status & HPSVS__SW_SV__MASK) {
		dev_info(dev, "SNVS secvio: SW SV\n");
	} else if (info->sv_status & HPSVS__SV5__MASK) {
		dev_info(dev, "SNVS secvio: SV 5\n");
	} else if (info->sv_status & HPSVS__SV4__MASK) {
		dev_info(dev, "SNVS secvio: SV 4\n");
	} else if (info->sv_status & HPSVS__SV3__MASK) {
		dev_info(dev, "SNVS secvio: SV 3\n");
	} else if (info->sv_status & HPSVS__SV2__MASK) {
		dev_info(dev, "SNVS secvio: SV 2\n");
	} else if (info->sv_status & HPSVS__SV1__MASK) {
		dev_info(dev, "SNVS secvio: SV 1\n");
	} else if (info->sv_status & HPSVS__SV0__MASK) {
		dev_info(dev, "SNVS secvio: SV 0\n");
	}

	/* Process in case of tamper */
	if (info->tp_status_1 & LPS__ESVD__MASK)
		dev_info(dev, "SNVS tamper: External SV\n");
	else if (info->tp_status_1 & LPS__ET2D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 2\n");
	else if (info->tp_status_1 & LPS__ET1D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 1\n");
	else if (info->tp_status_1 & LPS__WMT2D__MASK)
		dev_info(dev, "SNVS tamper: Wire Mesh 2\n");
	else if (info->tp_status_1 & LPS__WMT1D__MASK)
		dev_info(dev, "SNVS tamper: Wire Mesh 1\n");
	else if (info->tp_status_1 & LPS__VTD__MASK)
		dev_info(dev, "SNVS tamper: Voltage\n");
	else if (info->tp_status_1 & LPS__TTD__MASK)
		dev_info(dev, "SNVS tamper: Temperature\n");
	else if (info->tp_status_1 & LPS__CTD__MASK)
		dev_info(dev, "SNVS tamper: Clock\n");
	else if (info->tp_status_1 & LPS__PGD__MASK)
		dev_info(dev, "SNVS tamper: Power Glitch\n");
	else if (info->tp_status_1 & LPS__MCR__MASK)
		dev_info(dev, "SNVS tamper: Monotonic Counter rollover\n");
	else if (info->tp_status_1 & LPS__SRTCR__MASK)
		dev_info(dev, "SNVS tamper: Secure RTC rollover\n");
	else if (info->tp_status_1 & LPS__LPTA__MASK)
		dev_info(dev, "SNVS tamper: Time alarm\n");
	else if (info->tp_status_2 & LPTDS__ET10D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 10\n");
	else if (info->tp_status_2 & LPTDS__ET9D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 9\n");
	else if (info->tp_status_2 & LPTDS__ET8D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 8\n");
	else if (info->tp_status_2 & LPTDS__ET7D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 7\n");
	else if (info->tp_status_2 & LPTDS__ET6D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 6\n");
	else if (info->tp_status_2 & LPTDS__ET5D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 5\n");
	else if (info->tp_status_2 & LPTDS__ET4D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 4\n");
	else if (info->tp_status_2 & LPTDS__ET3D__MASK)
		dev_info(dev, "SNVS tamper: Tamper 3\n");

	return 0;
}

static int report_to_audit_notify(struct notifier_block *nb,
				  unsigned long status, void *notif_info)
{
	int ret = 0;
	struct audit_buffer *ab;
	struct notifier_info *info = notif_info;

	ab = audit_log_start(NULL, GFP_KERNEL, AUDIT_INTEGRITY_RULE);
	if (!ab) {
		ret = -ENOMEM;
		goto exit;
	}

	audit_log_format(ab, " svs=0x%.08x lpsr=0x%.08x lptds=0x%.08x",
			 info->sv_status, info->tp_status_1, info->tp_status_2);
	audit_log_task_info(ab, current);
	audit_log_end(ab);

exit:
	return ret;
}

static int call_secvio_config(struct device *dev, u8 id, u8 access, u32 *data0,
			      u32 *data1, u32 *data2, u32 *data3, u32 *data4,
			      u8 size)
{
	int ret = 0;
	sc_err_t sciErr;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	u32 regs[5] = {0xDEADBEEF};
	const char *details;

	if (size > 5)
		ret = -EINVAL;

	switch (size) {
	case 5:
		if (!data4) {
			ret = -EINVAL;
			goto exit;
		}
		/* fallthrough */
	case 4:
		if (!data3) {
			ret = -EINVAL;
			goto exit;
		}
		/* fallthrough */
	case 3:
		if (!data2) {
			ret = -EINVAL;
			goto exit;
		}
		/* fallthrough */
	case 2:
		if (!data1) {
			ret = -EINVAL;
			goto exit;
		}
		/* fallthrough */
	case 1:
		if (!data0) {
			ret = -EINVAL;
			goto exit;
		}
	}

	/* Pass parameters */
	switch (size) {
	case 5:
		regs[4] = *data4;
		/* fallthrough */
	case 4:
		regs[3] = *data3;
		/* fallthrough */
	case 3:
		regs[2] = *data2;
		/* fallthrough */
	case 2:
		regs[1] = *data1;
		/* fallthrough */
	case 1:
		regs[0] = *data0;
	}

	dev_dbg(dev, "Send %s [%.8x %.8x %.8x %.8x %.8x] -> %.4x\n",
		((access) ? "W" : "R"), regs[0], regs[1], regs[2], regs[3],
		regs[4], id);
	sciErr = sc_seco_secvio_config(data->ipcHandle, id, access, &regs[0],
				       &regs[1], &regs[2], &regs[3],
				       &regs[4], size);
	dev_dbg(dev, "Received [%.8x %.8x %.8x %.8x %.8x]\n",
		regs[0], regs[1], regs[2], regs[3], regs[4]);

	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &details);
		dev_err(dev, "Fail %s secvio config %s(%d)"
			"[id: %.2x data:{%.8x %.8x %.8x %.8x %.8x}(%d)]",
			((access) ? "write" : "read"), details, sciErr,
			id, regs[0], regs[1], regs[2], regs[3], regs[4], size);
	}

	if (!access) {
		/* Read parameters */
		switch (size) {
		case 5:
			*data4 = regs[4];
			/* fallthrough */
		case 4:
			*data3 = regs[3];
			/* fallthrough */
		case 3:
			*data2 = regs[2];
			/* fallthrough */
		case 2:
			*data1 = regs[1];
			/* fallthrough */
		case 1:
			*data0 = regs[0];
			/* fallthrough */
		}
	}

exit:
	return ret;
}

#ifdef CONFIG_DEBUG_FS

/* Buffer for copy to user */
#define IMX_SECVIO_BUFFER_SIZE 4000

/* Structure of values to read */
struct snvs_sc_regs {
	const char *name;
	u32 id;
	int size;
};

static struct snvs_sc_regs s_snvs_sc_reglist[] = {
	/* Locks */
	{"HPLR", 0x0, 1},
	{"LPLR", 0x34, 1},
	/* Security violation */
	{"HPSICR", 0xc, 1},
	{"HPSVCR", 0x10, 1},
	{"HPSVS", 0x18, 1},
	{"LPSVC", 0x40, 1},
	/* Temper detectors */
	{"LPTDC", 0x48, 2},
	{"LPSR", 0x4c, 1},
	{"LPTDS", 0xa4, 1},
	/* */
	{"LPTGFC", 0x44, 3},
	{"LPATCTL", 0xe0, 1},
	{"LPATCLK", 0xe4, 1},
	{"LPATRC", 0xe8, 2},
	/* Misc */
	{"LPMKC", 0x3c, 1},
	{"LPSMC", 0x5c, 2},
	{"LPPGD", 0x64, 1},
	{"HPVID", 0xf8, 2},
};

struct snvs_sc_dgo_regs {
	const char *name;
	u32 offset;
};

static struct snvs_sc_dgo_regs s_snvs_sc_dgo_reglist[] = {
	{"Offset ctrl", 0x0},
	{"Tamper PU/PD ctrl", 0x10},
	{"Analog test ctrl", 0x20},
	{"Temper sensor trim ctrl", 0x30},
	{"Misc ctrl", 0x40},
	{"Core voltage monitor ctrl", 0x50},
};

struct ocotp_fuse {
	const char *name;
	int word;
};

static struct ocotp_fuse ocotp_fuse_list[] = {
	{"30", 30},
	{"31", 31},
	{"260", 260},
	{"261", 261},
	{"262", 262},
	{"263", 263},
	{"768", 768},
};

static int imx_secvio_sc_file_open(struct inode *node, struct file *filp)
{
	/* debugfs_create_file set inode->i_private to our private data */
	filp->private_data = node->i_private;

	return 0;
}

static int buffer_write(struct device *dev, void *buffer, size_t total_size,
			size_t *offset, const char *fmt, ...)
{
	u8 *p = buffer;
	va_list args;
	int i, ret = 0;
	size_t space_left = total_size - *offset;

	va_start(args, fmt);
	i = vsnprintf(p + *offset, space_left, fmt, args);
	va_end(args);

	if (i < 0) {
		dev_err(dev, "Failed to write to buffer\n");
		ret = i;
	} else if (i >= space_left) {
		dev_err(dev, "No space, need %d, left: %lu\n", i, space_left);
		ret = -ENOSPC;
	}

	*offset += i;

	return ret;
}

static ssize_t imx_secvio_sc_info_read(struct file *filp,
				       char __user *user_buffer,
				       size_t size, loff_t *offset)
{
	int idx;
	int ret = 0;
	struct device *dev = filp->private_data;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	size_t buffer_pos = 0;

	if (!access_ok(VERIFY_WRITE, user_buffer, size)) {
		dev_err(dev, "Access check failed for %p(%ld)\n", user_buffer,
			size);
		return -EFAULT;
	}

	if (down_interruptible(&data->sem))
		return -ERESTARTSYS;

	/* Stop reading if an offset is passed*/
	if (*offset) {
		ret = 0;
		goto exit;
	}

	ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
			   &buffer_pos, "Fuses:\n");
	if (ret) {
		dev_err(dev, "Failed to report fuse header\n");
		goto exit;
	}

	/* Read fuses */
	for (idx = 0; idx < ARRAY_SIZE(ocotp_fuse_list); idx++) {
		u32 value = 0xDEADBEEF;
		sc_err_t sciErr;
		struct ocotp_fuse *fuse = &ocotp_fuse_list[idx];

		sciErr = sc_misc_otp_fuse_read(data->ipcHandle, fuse->word,
					       &value);
		if (sciErr != SC_ERR_NONE) {
			ret = sci_err_to_errno(sciErr, NULL);
			dev_err(dev, "Failed to access fuse %s: %d\n",
				fuse->name, sciErr);
			goto exit;
		}

		/* Write name of fuse */
		ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
				   &buffer_pos, "(%.4x)%-12s: 0x%.8x\n",
				   fuse->word, fuse->name, value);
		if (ret) {
			dev_err(dev, "Failed to report fuse %s\n", fuse->name);
			goto exit;
		}
	}

	ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
			   &buffer_pos, "SNVS:\n");
	if (ret) {
		dev_err(dev, "Failed to report reg header\n");
		goto exit;
	}

	/* Loop over all the registers to read */
	for (idx = 0; idx < ARRAY_SIZE(s_snvs_sc_reglist); idx++) {
		int idx_value;
		u32 regs[5] = {0xDEADBEEF};
		struct snvs_sc_regs *reg = &s_snvs_sc_reglist[idx];

		ret = call_secvio_config(dev, reg->id, SECVIO_CONFIG_READ,
					 &regs[0], &regs[1], &regs[2], &regs[3],
					 &regs[4], reg->size);
		if (ret) {
			dev_err(dev, "Failed to read reg %s\n", reg->name);
			goto exit;
		}

		/* Write name of register */

		ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
				   &buffer_pos, "(0x%.2x) %-12s:", reg->id,
				   reg->name);
		if (ret) {
			dev_err(dev, "Failed to report name for reg %s\n",
				reg->name);
			goto exit;
		}

		/* loop over number of values to write */
		for (idx_value = 0; idx_value < reg->size; idx_value++) {
			ret = buffer_write(dev, data->io_buffer,
					   data->io_buffer_size, &buffer_pos,
					   " 0x%.8x", regs[idx_value]);
			if (ret) {
				dev_err(dev,
					"Failed to report idx %d for reg %s\n",
					idx_value, reg->name);
				goto exit;
			}
		}

		/* Add return line */
		ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
				   &buffer_pos, "\n");
		if (ret) {
			dev_err(dev, "Failed to report newline for reg %s\n",
				reg->name);
			goto exit;
		}
	}

	ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
			   &buffer_pos, "SNVS DGO:\n");
	if (ret) {
		dev_err(dev, "Failed to report digital reg header\n");
		goto exit;
	}

	/* Loop over all the registers to read */
	for (idx = 0; idx < ARRAY_SIZE(s_snvs_sc_dgo_reglist); idx++) {
		u32 value;
		struct snvs_sc_dgo_regs *reg = &s_snvs_sc_dgo_reglist[idx];

		ret = sc_seco_secvio_dgo_config(data->ipcHandle, reg->offset,
						SECVIO_CONFIG_READ, &value);
		if (ret) {
			dev_err(dev, "Failed to access reg %s: %d\n", reg->name,
				ret);
			goto exit;
		}

		/* Write name of register and value */
		ret = buffer_write(dev, data->io_buffer, data->io_buffer_size,
				   &buffer_pos, "(0x%.2x) %-12s: 0x%.8x\n",
				   reg->offset, reg->name, value);
		if (ret) {
			dev_err(dev, "Failed to report reg %s\n", reg->name);
			goto exit;
		}
	}

	if (size < buffer_pos)
		dev_info(dev, "Data is %ld bytes but buffer is only %ld\n",
			 buffer_pos, size);
	else
		size = buffer_pos;

	ret = simple_read_from_buffer(user_buffer, size, offset,
				      data->io_buffer, buffer_pos);

exit:
	up(&data->sem);
	return ret;
}

static ssize_t imx_secvio_sc_info_write(struct file *filp,
					const char __user *user_buffer,
					size_t size, loff_t *offset)
{
	int idx;
	int ret = 0;
	struct device *dev = filp->private_data;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);

	u32 id;
	int nb_vals;
	u32 vals[5];

	if (!access_ok(VERIFY_READ, user_buffer, size)) {
		dev_err(dev, "Access check failed for %p(%ld)\n", user_buffer,
			size);
		return -EFAULT;
	}

	if (down_interruptible(&data->sem))
		return -ERESTARTSYS;

	/* Stop reading if an offset is passed*/
	if (*offset) {
		ret = size;
		goto exit;
	}

	/* Reset buffer */
	data->io_buffer[0] = '\0';

	ret = simple_write_to_buffer(data->io_buffer, data->io_buffer_size,
				     offset, user_buffer, size);
	if (ret < 0) {
		dev_err(dev, "Failed to copy user buffer\n");
		ret = -EIO;
		goto exit;
	}

	ret = sscanf(data->io_buffer, "%x [%x %x %x %x %x](%d)", &id, &vals[0],
		     &vals[1], &vals[2], &vals[3], &vals[4], &nb_vals);
	if (ret < 0) {
		dev_err(dev, "Failed to process input\n");
		ret = -EIO;
		goto exit;
	} else if (ret != 7) {
		dev_err(dev, "Failed to process all parameters\n");
		ret = -EIO;
		goto exit;
	}

	dev_dbg(dev, "writing id %.2x [0x%.8x 0x%.8x 0x%.8x 0x%.8x 0x%.8x](%d)",
		id, vals[0], vals[1], vals[2], vals[3], vals[4], nb_vals);

	if (nb_vals > 5) {
		dev_warn(dev, "Input nb values %d too big, reduced\n", nb_vals);
		nb_vals = 5;
	}

	/* Check valid id */
	ret = -EINVAL;
	for (idx = 0; idx < ARRAY_SIZE(s_snvs_sc_reglist); idx++) {
		struct snvs_sc_regs *reg = &s_snvs_sc_reglist[idx];

		if (id == reg->id) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		dev_err(dev, "Input id 0x%x is not valid\n", id);
		goto exit;
	}

	ret = call_secvio_config(dev, id, SECVIO_CONFIG_WRITE, &vals[0],
				 &vals[1], &vals[2], &vals[3], &vals[4],
				 nb_vals);
	if (ret)
		dev_err(dev, "Failed to write reg %x\n", id);

	ret = size;

exit:
	up(&data->sem);
	return ret;
}

static const struct file_operations imx_secvio_sc_info_ops = {
	.open		= imx_secvio_sc_file_open,
	.read		= imx_secvio_sc_info_read,
	.write		= imx_secvio_sc_info_write,
};

static ssize_t imx_secvio_sc_enable_read(struct file *filp,
					 char __user *user_buffer,
					 size_t size, loff_t *offset)
{
	int ret = 0;
	struct device *dev = filp->private_data;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	size_t buffer_pos = 0;

	if (!access_ok(VERIFY_WRITE, user_buffer, size)) {
		dev_err(dev, "Access check failed for %p(%ld)\n", user_buffer,
			size);
		return -EFAULT;
	}

	if (down_interruptible(&data->sem))
		return -ERESTARTSYS;

	buffer_write(dev, data->io_buffer, data->io_buffer_size, &buffer_pos,
		     "%d\n", data->enabled);
	if (ret) {
		dev_err(dev, "Failed to write enabled state\n");
		goto exit;
	}

	ret = simple_read_from_buffer(user_buffer, size, offset,
				      data->io_buffer, buffer_pos);

exit:
	up(&data->sem);
	return ret;
}

static ssize_t imx_secvio_sc_enable_write(struct file *filp,
					  const char __user *user_buffer,
					  size_t size, loff_t *offset)
{
	int ret = 0;
	struct device *dev = filp->private_data;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	int state;

	if (!access_ok(VERIFY_READ, user_buffer, size)) {
		dev_err(dev, "Access check failed for %p(%ld)\n", user_buffer,
			size);
		return -EFAULT;
	}

	if (down_interruptible(&data->sem))
		return -ERESTARTSYS;

	/* Reset buffer */
	data->io_buffer[0] = '\0';

	ret = simple_write_to_buffer(data->io_buffer, data->io_buffer_size,
				     offset, user_buffer, size);
	if (ret < 0) {
		dev_err(dev, "Failed to copy user buffer\n");
		ret = -EIO;
		goto exit;
	}

	ret = sscanf(data->io_buffer, "%d", &state);
	if (ret < 0) {
		dev_err(dev, "Failed to process input\n");
		ret = -EIO;
		goto exit;
	} else if (ret != 1) {
		dev_err(dev, "Failed to process all parameters\n");
		ret = -EIO;
		goto exit;
	}

	if (state) {
		ret = enable_irq(dev);
		if (ret) {
			dev_err(dev, "Fail to enable IRQ\n");
			goto exit;
		}
		data->enabled = 1;
	} else {
		ret = disable_irq(dev);
		if (ret) {
			dev_err(dev, "Fail to disable IRQ\n");
			goto exit;
		}
		data->enabled = 0;
	}

	ret = size;

exit:
	up(&data->sem);
	return ret;
}

static const struct file_operations imx_secvio_sc_enable_ops = {
	.open		= imx_secvio_sc_file_open,
	.read		= imx_secvio_sc_enable_read,
	.write		= imx_secvio_sc_enable_write,
};

#endif /* CONFIG_DEBUG_FS */

static int imx_secvio_sc_debugfs(struct device *dev)
{
	int ret = 0;
#ifdef CONFIG_DEBUG_FS
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);

	data->io_buffer_size = IMX_SECVIO_BUFFER_SIZE;
	data->io_buffer = devm_kzalloc(dev, data->io_buffer_size, GFP_KERNEL);
	if (!data->io_buffer) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate mem for IO buffer\n");
		goto error;
	}

	/* Create a folder */
	data->dfs = debugfs_create_dir(dev_name(dev), NULL);
	if (!data->dfs) {
		dev_err(dev, "Failed to create dfs dir\n");
		ret = -EIO;
		goto error;
	}

	/* Create the semaphore for file operations */
	sema_init(&data->sem, 1);

	/* Create the file to read info and write to reg */
	debugfs_create_file("info", 0x666,
			    data->dfs, dev,
			    &imx_secvio_sc_info_ops);

	/* Create the file to enable/disable IRQs */
	debugfs_create_file("enable", 0x666,
			    data->dfs, dev,
			    &imx_secvio_sc_enable_ops);

	goto exit;

error:
	imx_secvio_sc_debugfs_remove(dev);

exit:
#endif /* CONFIG_DEBUG_FS */
	return ret;
}

static int enable_irq(struct device *dev)
{
	int ret = 0, ret2;
	sc_err_t sciErr;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	const char *err_expl = NULL;
	u32 irq_status;

	/* Enable the IRQ */
	sciErr = sc_irq_enable(data->ipcHandle, SC_R_MU_1A,
			       SC_IRQ_GROUP_WAKE,
			       SC_IRQ_SECVIO, true);
	if (sciErr) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "Cannot enable SCU IRQ: %s(%d)\n", err_expl,
			sciErr);
		goto exit;
	}

	/* Enable interrupt */
	sciErr = sc_seco_secvio_enable(data->ipcHandle);
	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "Cannot enable SNVS irq: %s(%d)\n", err_expl,
			sciErr);
		goto exit;
	};

	/* Unmask interrupt */
	sciErr = sc_irq_status(data->ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_WAKE,
			       &irq_status);
	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "Cannot unmask irq: %s(%d)\n", err_expl,
			sciErr);
		goto exit;
	};

exit:
	if (ret) {
		ret2 = disable_irq(dev);
		if (ret2)
			dev_warn(dev, "Failed to disable the IRQ\n");
	}

	return ret;
}

static int disable_irq(struct device *dev)
{
	int ret = 0;
	sc_err_t sciErr;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);
	const char *err_expl = NULL;

	/* Disable the IRQ */
	sciErr = sc_irq_enable(data->ipcHandle, SC_R_MU_1A,
			       SC_IRQ_GROUP_WAKE,
			       SC_IRQ_SECVIO, false);
	if (sciErr) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "Cannot enable SCU IRQ: %s(%d)\n", err_expl,
			sciErr);
		goto exit;
	}

exit:
	return ret;
}

static void imx_secvio_sc_debugfs_remove(struct device *dev)
{
#ifdef CONFIG_DEBUG_FS
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);

	debugfs_remove(data->dfs);

#endif /* CONFIG_DEBUG_FS */
}

static int imx_secvio_sc_setup(struct device *dev)
{
	int ret = 0, ret2;
	u32 mu_id;
	sc_err_t sciErr;
	struct imx_secvio_sc_data *data;
	u32 irq_status;
	bool own_secvio;
	const char *err_expl = NULL;

	/* Allocate private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate mem for data\n");
		goto error;
	}

	data->dev = dev;

	dev_set_drvdata(dev, data);

	/* Get Messaging Unit */
	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "can not obtain mu id: %s(%d)\n", err_expl,
			sciErr);
		goto error;
	}

	/* Get a handle */
	sciErr = sc_ipc_open(&data->ipcHandle, mu_id);
	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "can not open mu channel to scu: %s(%d)\n",
			err_expl,
			sciErr);
		goto error;
	};

	/* Check the API is accessible getting version */
	ret = call_secvio_config(dev, HPVIDR_ID, SECVIO_CONFIG_READ,
				 &data->version, NULL, NULL, NULL, NULL, 1);
	if (ret) {
		dev_err(dev, "Cannot read snvs version: %d\n", ret);
		goto error;
	}

	/* Init debug FS */
	ret = imx_secvio_sc_debugfs(dev);
	if (ret) {
		dev_err(dev, "Failed to set debugfs\n");
		goto error;
	}

	/* Check we own the SECVIO */
	own_secvio = sc_rm_is_resource_owned(data->ipcHandle, SC_R_SECVIO);
	if (!own_secvio) {
		dev_err(dev, "Secvio resource is not owned\n");
		ret = -EPERM;
		goto error;
	}

	/* Check IRQ exists */
	sciErr = sc_irq_status(data->ipcHandle, SC_R_MU_1A,
			       SC_IRQ_GROUP_WAKE,
			       &irq_status);
	if (sciErr != SC_ERR_NONE) {
		ret = sci_err_to_errno(sciErr, &err_expl);
		dev_err(dev, "Cannot get IRQ state: %s(%d)\n", err_expl,
			sciErr);
		goto error;
	}

	ret = enable_irq(dev);
	if (ret) {
		dev_err(dev, "Failed to enable IRQ\n");
		goto error;
	}

	/* Set the callback for notification */
	data->irq_nb.notifier_call = imx_secvio_sc_notify;

	/* Register the handle of IRQ */
	ret = register_scu_notifier(&data->irq_nb);
	if (ret) {
		dev_err(dev, "Failed to register IRQ notification handler\n");
		goto error;
	}

	/* Register the handle for reporting to user */
	data->report_nb.notifier_call = report_to_user_notify;
	ret = register_imx_secvio_sc_notifier(&data->report_nb);
	if (ret) {
		dev_err(dev, "Failed to register report notif handler\n");
		goto error;
	}

	/* Register the handle to report to audit FW */
	data->audit_nb.notifier_call = report_to_audit_notify;
	ret = register_imx_secvio_sc_notifier(&data->audit_nb);
	if (ret) {
		dev_err(dev, "Failed to register report audit handler\n");
		goto error;
	}

	/* Process current state of the secvio and tampers */
	imx_secvio_sc_process_state(dev);

	data->enabled = 1;

	goto exit;

error:
	ret2 = imx_secvio_sc_teardown(dev);
	if (ret2)
		dev_warn(dev, "Failed to teardown\n");

exit:
	return ret;
}

static int imx_secvio_sc_teardown(struct device *dev)
{
	int ret = 0;
	struct imx_secvio_sc_data *data = dev_get_drvdata(dev);

	if (!data)
		goto exit;

	if (data->dfs)
		imx_secvio_sc_debugfs_remove(dev);

	if (data->ipcHandle) {
		ret = disable_irq(dev);
		if (ret) {
			dev_err(dev, "failed to disable IRQ\n");
			goto exit;
		}

		sc_ipc_close(data->ipcHandle);
	}

	if (data->report_nb.notifier_call) {
		ret = unregister_imx_secvio_sc_notifier(&data->report_nb);
		if (ret) {
			dev_err(dev, "Failed to unregister report to user\n");
			goto exit;
		}
	}

	if (data->audit_nb.notifier_call) {
		ret = unregister_imx_secvio_sc_notifier(&data->audit_nb);
		if (ret) {
			dev_err(dev, "Failed to unregister report to audit\n");
			goto exit;
		}
	}

	if (data->irq_nb.notifier_call) {
		ret = unregister_scu_notifier(&data->irq_nb);
		if (ret) {
			dev_err(dev, "Failed to unregister SCU IRQ\n");
			goto exit;
		}
	}

exit:
	return ret;
}

static int imx_secvio_sc_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	ret = imx_secvio_sc_setup(dev);
	if (ret)
		dev_err(dev, "Failed to setup\n");

	return ret;
}

static int imx_secvio_sc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

	ret = imx_secvio_sc_teardown(dev);
	if (ret)
		dev_err(dev, "Failed to tear down\n");

	return ret;
}

static const struct of_device_id imx_secvio_sc_dt_ids[] = {
	{ .compatible = "fsl,imx-sc-secvio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sc_dt_ids);

static struct platform_driver imx_secvio_sc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "imx_secvio_sc",
		.of_match_table = imx_secvio_sc_dt_ids,
	},
	.probe		= imx_secvio_sc_probe,
	.remove         = imx_secvio_sc_remove,
};
module_platform_driver(imx_secvio_sc_driver);

MODULE_AUTHOR("Franck LENORMAND <franck.lenormand@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX driver to handle SNVS secvio irq sent by SCFW");
MODULE_LICENSE("GPL");
