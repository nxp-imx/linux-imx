/*
 * imx_mcc_tty.c - pty demo driver used to test imx mcc
 * posix pty interface.
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>

/**
 * struct mcctty_port - Wrapper struct for imx mcc pty port.
 * @port:		TTY port data
 * @rx_lock:		Lock for rx_buf.
 * @rx_buf:		Read buffer
 */
struct mcctty_port {
	struct delayed_work	read;
	struct tty_port		port;
	spinlock_t		rx_lock;
	char			*rx_buf;
};

static struct mcctty_port mcc_tty_port;

enum {
	MCC_NODE_A9 = 0,
	MCC_NODE_M4 = 0,

	MCC_A9_PORT = 1,
	MCC_M4_PORT = 2,
};

/* mcc tty/pingpong demo */
static MCC_ENDPOINT mcc_endpoint_a9_pingpong = {0, MCC_NODE_A9, MCC_A9_PORT};
static MCC_ENDPOINT mcc_endpoint_m4_pingpong = {1, MCC_NODE_M4, MCC_M4_PORT};
struct mcc_tty_msg {
	char data[MCC_ATTR_BUFFER_SIZE_IN_BYTES - 24];
};

static struct tty_port_operations  mcctty_port_ops = { };

static int mcctty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	return tty_port_install(&mcc_tty_port.port, driver, tty);
}

static int mcctty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void mcctty_close(struct tty_struct *tty, struct file *filp)
{
	return tty_port_close(tty->port, tty, filp);
}

static int mcctty_write(struct tty_struct *tty, const unsigned char *buf,
			 int total)
{
	int i, count, ret = 0, space;
	unsigned char *cbuf, *tmp;
	MCC_MEM_SIZE num_of_received_bytes;
	struct mcc_tty_msg tty_msg;
	struct mcctty_port *cport = &mcc_tty_port;

	if (NULL == buf) {
		pr_err("buf shouldn't be null.\n");
		return -ENOMEM;
	}

	count = total;
	tmp = (unsigned char *)buf;
	for (i = 0; i <= count / 999; i++) {
		strlcpy(tty_msg.data, tmp, count >= 1000 ? 1000 : count + 1);
		if (count >= 1000)
			count -= 999;

		/*
		 * wait until the remote endpoint is created by
		 * the other core
		 */
		ret = mcc_send(&mcc_endpoint_a9_pingpong,
				&mcc_endpoint_m4_pingpong, &tty_msg,
				sizeof(struct mcc_tty_msg),
				0xffffffff);

		while (MCC_ERR_ENDPOINT == ret) {
			pr_err("\n send err ret %d, re-send\n", ret);
			ret = mcc_send(&mcc_endpoint_a9_pingpong,
					&mcc_endpoint_m4_pingpong, &tty_msg,
					sizeof(struct mcc_tty_msg),
					0xffffffff);
			msleep(5000);
		}

		ret = mcc_recv(&mcc_endpoint_m4_pingpong,
				&mcc_endpoint_a9_pingpong, &tty_msg,
				sizeof(struct mcc_tty_msg),
				&num_of_received_bytes, 0xffffffff);

		if (MCC_SUCCESS != ret) {
			pr_err("A9 Main task receive error: %d\n", ret);
		} else {
			/* flush the recv-ed data to tty node */
			spin_lock_bh(&cport->rx_lock);
			space = tty_prepare_flip_string(&cport->port, &cbuf,
							strlen(tty_msg.data));
			if ((space <= 0) || (cport->rx_buf == NULL))
				goto tty_unlock;

			memcpy(cport->rx_buf, &tty_msg.data,
					strlen(tty_msg.data));
			memcpy(cbuf, cport->rx_buf, space);
			tty_flip_buffer_push(&cport->port);
tty_unlock:
			spin_unlock_bh(&cport->rx_lock);
		}
	}
	return total;
}

static int mcctty_write_room(struct tty_struct *tty)
{
	int room;

	/* report the space in the mcc buffer */
	room = MCC_ATTR_BUFFER_SIZE_IN_BYTES;

	return room;
}

static const struct tty_operations imxmcctty_ops = {
	.install		= mcctty_install,
	.open			= mcctty_open,
	.close			= mcctty_close,
	.write			= mcctty_write,
	.write_room		= mcctty_write_room,
};

static struct tty_driver *mcctty_driver;

static int imx_mcc_tty_probe(struct platform_device *pdev)
{
	int ret;
	struct mcctty_port *cport = &mcc_tty_port;
	MCC_INFO_STRUCT mcc_info;

	mcctty_driver = tty_alloc_driver(1,
			TTY_DRIVER_RESET_TERMIOS |
			TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(mcctty_driver))
		return PTR_ERR(mcctty_driver);

	mcctty_driver->driver_name = "mcc_tty";
	mcctty_driver->name = "ttyMCC";
	mcctty_driver->major = TTYAUX_MAJOR;
	mcctty_driver->minor_start = 3;
	mcctty_driver->type = TTY_DRIVER_TYPE_PTY;
	mcctty_driver->init_termios = tty_std_termios;
	mcctty_driver->init_termios.c_cflag |= CLOCAL;

	tty_set_operations(mcctty_driver, &imxmcctty_ops);

	tty_port_init(&cport->port);
	cport->port.ops = &mcctty_port_ops;
	spin_lock_init(&cport->rx_lock);

	ret = tty_register_driver(mcctty_driver);
	if (ret < 0) {
		pr_err("Couldn't install mcc tty driver: err %d\n", ret);
		goto error;
	} else
		pr_info("Install mcc tty driver!\n");

	/* Allocate the buffer we use for reading data */
	cport->rx_buf = kzalloc(MCC_ATTR_BUFFER_SIZE_IN_BYTES,
			GFP_KERNEL);
	if (!cport->rx_buf) {
		ret = -ENOMEM;
		goto error;
	}

	ret = mcc_initialize(MCC_NODE_A9);
	if (ret) {
		pr_err("failed to initialize mcc.\n");
		ret = -ENODEV;
		goto error;
	}

	ret = mcc_get_info(MCC_NODE_A9, &mcc_info);
	if (ret) {
		pr_err("failed to get mcc info.\n");
		ret = -ENODEV;
		goto error;
	} else {
		pr_info("\nA9 mcc prepares run, MCC version is %s\n",
				mcc_info.version_string);
		pr_info("imx mcc tty/pingpong test begin.\n");
	}

	ret = mcc_create_endpoint(&mcc_endpoint_a9_pingpong,
			MCC_A9_PORT);
	if (ret) {
		pr_err("failed to create a9 mcc ep.\n");
		ret = -ENODEV;
		goto error;
	}

	return 0;

error:
	tty_unregister_driver(mcctty_driver);
	put_tty_driver(mcctty_driver);
	tty_port_destroy(&cport->port);
	mcctty_driver = NULL;

	return ret;
}

static int imx_mcc_tty_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct mcctty_port *cport = &mcc_tty_port;

	/* stop reading, null the read buffer. */
	kfree(cport->rx_buf);
	cport->rx_buf = NULL;

	/* destory the mcc tty endpoint here */
	ret = mcc_destroy_endpoint(&mcc_endpoint_a9_pingpong);
	if (ret)
		pr_err("failed to destory a9 mcc ep.\n");
	else
		pr_info("destory a9 mcc ep.\n");

	tty_unregister_driver(mcctty_driver);
	tty_port_destroy(&cport->port);
	put_tty_driver(mcctty_driver);

	return ret;
}

static const struct of_device_id imx6sx_mcc_tty_ids[] = {
	{ .compatible = "fsl,imx6sx-mcc-tty", },
	{ /* sentinel */ }
};

static struct platform_driver imxmcctty_driver = {
	.driver = {
		.name = "imx6sx-mcc-tty",
		.owner  = THIS_MODULE,
		.of_match_table = imx6sx_mcc_tty_ids,
		},
	.probe = imx_mcc_tty_probe,
	.remove = imx_mcc_tty_remove,
};

/*!
 * Initialise the imxmcctty_driver.
 *
 * @return  The function always returns 0.
 */

static int __init imxmcctty_init(void)
{
	if (platform_driver_register(&imxmcctty_driver) != 0)
		return -ENODEV;

	printk(KERN_INFO "IMX MCC TTY driver module loaded\n");
	return 0;
}

static void __exit imxmcctty_exit(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&imxmcctty_driver);
}

module_init(imxmcctty_init);
module_exit(imxmcctty_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MCC TTY driver");
MODULE_LICENSE("GPL");
