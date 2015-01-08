/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>

#include <linux/imx_sema4.h>
#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>
#include <linux/mcc_linux.h>
#include <linux/mcc_imx6sx.h>

enum {
	MCC_NODE_A9 = 0,
	MCC_NODE_M4 = 0,

	MCC_A9_PORT = 1,
	MCC_M4_PORT = 2,
};

/* mcc pingpong test */
MCC_ENDPOINT mcc_endpoint_a9_pingpong = {0, MCC_NODE_A9, MCC_A9_PORT};
MCC_ENDPOINT mcc_endpoint_m4_pingpong = {1, MCC_NODE_M4, MCC_M4_PORT};
/* mcc can test */
MCC_ENDPOINT mcc_endpoint_a9_can = {0, MCC_NODE_A9, MCC_A9_PORT};
MCC_ENDPOINT mcc_endpoint_m4_can = {1, MCC_NODE_A9, MCC_A9_PORT};

struct mcc_pp_msg {
	unsigned int data;
};

/* Set the max len of the can msg to be 1000 bytes */
struct mcc_can_msg {
	char data[MCC_ATTR_BUFFER_SIZE_IN_BYTES - 24];
};

static ssize_t imx_mcc_can_test_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 can_test_en;
	int i = 0, ret = 0;
	struct mcc_can_msg msg;
	MCC_MEM_SIZE num_of_received_bytes;
	MCC_INFO_STRUCT mcc_info;

	sscanf(buf, "%d\n", &can_test_en);

	if (can_test_en) {
		pr_info("imx mcc can communication test begin.\n");
		ret = mcc_get_info(MCC_NODE_A9, &mcc_info);
		if (ret) {
			pr_err("failed to get mcc info.\n");
			return -EINVAL;
		}

		ret = mcc_create_endpoint(&mcc_endpoint_a9_can, MCC_A9_PORT);
		if (ret) {
			pr_err("failed to create a9 mcc ep.\n");
			return -EINVAL;
		}

		pr_info("\nA9 mcc prepares run, MCC version is %s\n",
				mcc_info.version_string);

		while (i < 0x10000) {
			i++;
			/*
			 * wait for the "sleep" msg from the remote ep.
			 */
			ret = mcc_recv(&mcc_endpoint_m4_can,
					&mcc_endpoint_a9_can, &msg,
					sizeof(struct mcc_can_msg),
					&num_of_received_bytes, 0xffffffff);
			if (ret < 0) {
				pr_err("A9 Main task recv error: %d\n", ret);
				break;
			}
			pr_info("%s", msg.data);
		}

		ret = mcc_destroy_endpoint(&mcc_endpoint_a9_can);
		if (ret) {
			pr_err("failed to destory a9 mcc ep.\n");
			return -EINVAL;
		} else {
			pr_info("destory a9 mcc ep.\n");
		}
	}

	pr_info("imx mcc test end after %08d times recv tests.\n", i);
	return count;
}

static ssize_t imx_mcc_pingpong_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 pingpong_en;
	int i = 0, ret = 0;
	struct timeval tv1, tv2, tv3;
	u32 tv_count1 = 0, tv_count2 = 0;
	struct mcc_pp_msg msg;
	MCC_MEM_SIZE num_of_received_bytes;
	MCC_INFO_STRUCT mcc_info;

	sscanf(buf, "%d\n", &pingpong_en);
	if (pingpong_en) {
		pr_info("imx mcc pingpong test begin.\n");
		ret = mcc_get_info(MCC_NODE_A9, &mcc_info);
		if (ret) {
			pr_err("failed to get mcc info.\n");
			return -EINVAL;
		}

		ret = mcc_create_endpoint(&mcc_endpoint_a9_pingpong,
				MCC_A9_PORT);
		if (ret) {
			pr_err("failed to create a9 mcc ep.\n");
			return -EINVAL;
		}

		pr_info("\nA9 mcc prepares run, MCC version is %s\n",
				mcc_info.version_string);
		msg.data = 1;
		while (i < 0x50000) {
			i++;
			i++;
			/*
			 * wait until the remote endpoint is created by
			 * the other core
			 */
			if (pingpong_en > 1)
				do_gettimeofday(&tv1);

			ret = mcc_send(&mcc_endpoint_a9_pingpong,
					&mcc_endpoint_m4_pingpong, &msg,
					sizeof(struct mcc_pp_msg),
					0xffffffff);
			if (ret < 0) {
				pr_err("A9 Main task send error: %d\n", ret);
				break;
			}

			if (pingpong_en > 1) {
				do_gettimeofday(&tv2);
				tv_count1 = (tv2.tv_sec - tv1.tv_sec)
					* USEC_PER_SEC
					+ tv2.tv_usec - tv1.tv_usec;
			}
			while (MCC_ERR_ENDPOINT == ret) {
				pr_err("\n send err ret %d, re-send\n", ret);
				ret = mcc_send(&mcc_endpoint_a9_pingpong,
						&mcc_endpoint_m4_pingpong, &msg,
						sizeof(struct mcc_pp_msg),
						0xffffffff);
				msleep(5000);
			}

			if (pingpong_en > 1)
				do_gettimeofday(&tv2);

			ret = mcc_recv(&mcc_endpoint_m4_pingpong,
					&mcc_endpoint_a9_pingpong, &msg,
					sizeof(struct mcc_pp_msg),
					&num_of_received_bytes, 0xffffffff);

			if (pingpong_en > 1) {
				do_gettimeofday(&tv3);
				tv_count2 = (tv3.tv_sec - tv2.tv_sec)
					* USEC_PER_SEC
					+ tv3.tv_usec - tv2.tv_usec;
				pr_info("imx mcc: Data transfer speed tests"
						"in pingpong. a9 -> m4:%08dus."
						"a9 <- m4:%08dus.\n",
						tv_count1, tv_count2);
			}

			if (MCC_SUCCESS != ret) {
				pr_err("A9 Main task receive error: %d\n", ret);
				break;
			} else {
				pr_info("%08x Main task received a msg"
					" from [%d, %d, %d] endpoint\n", i,
					mcc_endpoint_m4_pingpong.core,
					mcc_endpoint_m4_pingpong.node,
					mcc_endpoint_m4_pingpong.port);
				pr_info("Message: Size=0x%08x, data = 0x%08x\n",
					num_of_received_bytes, msg.data);
				msg.data++;
			}
		}
		ret = mcc_destroy_endpoint(&mcc_endpoint_a9_pingpong);
		if (ret) {
			pr_err("failed to destory a9 mcc ep.\n");
			return ret;
		} else {
			pr_info("destory a9 mcc ep.\n");
		}
		pr_info("imx mcc test end after %08d times tests.\n", i/2);
	}

	if (ret)
		return ret;
	else
		return count;
}

static DEVICE_ATTR(pingpong_en, S_IWUGO, NULL, imx_mcc_pingpong_en);
static DEVICE_ATTR(can_test_en, S_IWUGO, NULL, imx_mcc_can_test_en);

static struct attribute *imx_mcc_attrs[] = {
	&dev_attr_pingpong_en.attr,
	&dev_attr_can_test_en.attr,
	NULL
};

static struct attribute_group imx_mcc_attrgroup = {
	.attrs	= imx_mcc_attrs,
};

static int imx_mcc_test_probe(struct platform_device *pdev)
{
	int ret = 0;
	MCC_INFO_STRUCT mcc_info;

	ret = mcc_initialize(MCC_NODE_A9);
	if (ret) {
		pr_err("failed to initialize mcc.\n");
		ret = -EINVAL;
		return ret;
	}

	ret = mcc_get_info(MCC_NODE_A9, &mcc_info);
	if (ret) {
		pr_err("failed to get mcc info.\n");
		ret = -EINVAL;
		goto out_node;
	}
	pr_info("\nA9 mcc prepares run, MCC version is %s\n",
			mcc_info.version_string);

	if (strcmp(mcc_info.version_string, MCC_VERSION_STRING) != 0) {
		pr_err("\nError, different versions of the MCC library");
		pr_err("is used on each core!\n");
		pr_err("\nApplication is stopped now.\n");
		ret = -EINVAL;
		goto out_node;
	}

	/* add attributes for device. */
	ret = sysfs_create_group(&pdev->dev.kobj, &imx_mcc_attrgroup);
	if (ret)
		goto out_node;

	return ret;

out_node:
	mcc_destroy(MCC_NODE_A9);

	return ret;
}

static int imx_mcc_test_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id imx_mcc_test_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-mcc-test", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_mcc_test_dt_ids);

static struct platform_driver imx_mcc_test_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "imx6sx-mcc-test",
		   .of_match_table = imx_mcc_test_dt_ids,
		   },
	.probe = imx_mcc_test_probe,
	.remove = imx_mcc_test_remove,
};

static int __init imx_mcc_test_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_mcc_test_driver);
	if (ret)
		pr_err("failed to register imx mcc test driver.\n");
	else
		pr_info("imx mcc test is registered.\n");
	return ret;
}
late_initcall(imx_mcc_test_init);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX MCC test driver");
MODULE_LICENSE("GPL");
