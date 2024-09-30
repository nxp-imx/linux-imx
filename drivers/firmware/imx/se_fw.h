/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2024 NXP
 */

#ifndef SE_MU_H
#define SE_MU_H

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/mailbox_client.h>

#define MAX_MESSAGE_SIZE		31
#define MAX_RECV_SIZE			MAX_MESSAGE_SIZE
#define MAX_RECV_SIZE_BYTES		(MAX_RECV_SIZE << 2)
#define MAX_MESSAGE_SIZE_BYTES		(MAX_MESSAGE_SIZE << 2)

#define ELE_MSG_DATA_NUM		10

#define MSG_TAG(x)			(((x) & 0xff000000) >> 24)
#define MSG_COMMAND(x)			(((x) & 0x00ff0000) >> 16)
#define MSG_SIZE(x)			(((x) & 0x0000ff00) >> 8)
#define MSG_VER(x)			((x) & 0x000000ff)
#define RES_STATUS(x)			((x) & 0x000000ff)
#define RES_IND(x)			(((x) & 0x0000ff00) >> 8)
#define MAX_DATA_SIZE_PER_USER		(65 * 1024)
#define S4_DEFAULT_MUAP_INDEX		(2)
#define S4_MUAP_DEFAULT_MAX_USERS	(4)
#define MESSAGING_VERSION_6		0x6
#define MESSAGING_VERSION_7		0x7

#define DEFAULT_MESSAGING_TAG_COMMAND           (0x17u)
#define DEFAULT_MESSAGING_TAG_RESPONSE          (0xe1u)

#define ELE_MU_IO_FLAGS_USE_SEC_MEM	(0x02u)
#define ELE_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)

#define SOC_ID_OF_IMX95			0x9500

struct ele_imem_buf {
	u8 *buf;
	phys_addr_t phyaddr;
	u32 size;
	u32 state;
};

struct ele_buf_desc {
	u8 *shared_buf_ptr;
	u8 *usr_buf_ptr;
	u32 size;
	struct list_head link;
};

/* Status of a char device */
enum mu_device_status_t {
	MU_FREE,
	MU_OPENED
};

struct ele_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

/* Private struct for each char device instance. */
struct ele_mu_device_ctx {
	struct device *dev;
	struct ele_mu_priv *priv;
	struct miscdevice miscdev;

	enum mu_device_status_t status;
	wait_queue_head_t wq;
	struct semaphore fops_lock;

	bool signal_recvd;
	u32 pending_hdr;
	struct list_head pending_in;
	struct list_head pending_out;

	struct ele_shared_mem secure_mem;
	struct ele_shared_mem non_secure_mem;
	u32 mu_buff_offset;

	u32 temp_cmd[MAX_MESSAGE_SIZE];
	u32 temp_resp[MAX_RECV_SIZE];
	u32 temp_resp_size;
	struct notifier_block ele_notify;
};

/* Header of the messages exchange with the EdgeLock Enclave */
struct mu_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

#define ELE_MU_HDR_SZ	4
#define TAG_OFFSET	(ELE_MU_HDR_SZ - 1)
#define CMD_OFFSET	(ELE_MU_HDR_SZ - 2)
#define SZ_OFFSET	(ELE_MU_HDR_SZ - 3)
#define VER_OFFSET	(ELE_MU_HDR_SZ - 4)

struct ele_api_msg {
	u32 header; /* u8 Tag; u8 Command; u8 Size; u8 Ver; */
	u32 data[ELE_MSG_DATA_NUM];
};

struct perf_time_frame {
	struct timespec64 t_start;
	struct timespec64 t_end;
};

struct ele_mu_priv {
	struct list_head priv_data;
	struct ele_mu_device_ctx *cmd_receiver_dev;
	struct ele_mu_device_ctx *waiting_rsp_dev;
	/*
	 * prevent parallel access to the MU registers
	 * e.g. a user trying to send a command while the other one is
	 * sending a response.
	 */
	struct mutex mu_lock;
	/*
	 * prevent a command to be sent on the MU while another one is still
	 * processing. (response to a command is allowed)
	 */
	struct mutex mu_cmd_lock;
	struct device *dev;
	u8 ele_mu_did;
	u8 pdev_name[20];
	u32 ele_mu_id;
	u8 cmd_tag;
	u8 rsp_tag;
	u8 success_tag;
	u8 base_api_ver;
	u8 fw_api_ver;
	u32 fw_fail;
	const void *info;

	struct mbox_client ele_mb_cl;
	struct mbox_chan *tx_chan, *rx_chan;
	struct ele_api_msg *tx_msg, *rx_msg;
	struct completion done;
	spinlock_t lock;
	/*
	 * Flag to retain the state of initialization done at
	 * the time of ele-mu probe.
	 */
	uint32_t flags;
	u8 max_dev_ctx;
	struct ele_mu_device_ctx **ctxs;
	struct ele_imem_buf imem;
	struct perf_time_frame time_frame;
	struct imx_sc_ipc *ipc_scu;
	u8 part_owner;
	u8 *se_img_file_to_load;
};

uint32_t get_se_soc_id(struct device *dev);
phys_addr_t get_phy_buf_mem_pool(struct device *dev,
				 char *mem_pool_name,
				 u32 **buf,
				 uint32_t size);
void free_phybuf_mem_pool(struct device *dev,
			  char *mem_pool_name,
			  u32 *buf,
			  uint32_t size);
#endif
