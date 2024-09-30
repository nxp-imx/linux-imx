/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause*/
/*
 * Copyright 2019-2024 NXP
 */

#ifndef ELE_MU_IOCTL_H
#define ELE_MU_IOCTL_H

/* IOCTL definitions. */

struct ele_mu_ioctl_setup_iobuf {
	u8 *user_buf;
	u32 length;
	u32 flags;
	u64 ele_addr;
};

struct ele_mu_ioctl_shared_mem_cfg {
	u32 base_offset;
	u32 size;
};

struct ele_mu_ioctl_get_mu_info {
	u8 ele_mu_id;
	u8 interrupt_idx;
	u8 tz;
	u8 did;
	u8 cmd_tag;
	u8 rsp_tag;
	u8 success_tag;
	u8 base_api_ver;
	u8 fw_api_ver;
};

struct ele_mu_ioctl_signed_message {
	u8 *message;
	u32 msg_size;
	u32 error_code;
};

struct ele_mu_ioctl_get_soc_info {
	u16 soc_id;
	u16 soc_rev;
	u8  board_type;
};

struct ele_time_frame {
	struct timespec64 t_start;
	struct timespec64 t_end;
};

/* IO Buffer Flags */
#define ELE_MU_IO_FLAGS_IS_OUTPUT	(0x00u)
#define ELE_MU_IO_FLAGS_IS_INPUT	(0x01u)
#define ELE_MU_IO_FLAGS_USE_SEC_MEM	(0x02u)
#define ELE_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)
#define ELE_MU_IO_DATA_BUF_SHE_V2X  	(0x08u)
#define ELE_MU_IO_FLAGS_IS_IN_OUT	(0x10u)

/* IOCTLS */
#define ELE_MU_IOCTL			0x0A /* like MISC_MAJOR. */

/*
 * ioctl to designated the current fd as logical-reciever.
 * This is ioctl is send when the nvm-daemon, a slave to the
 * firmware is started by the user.
 */
#define ELE_MU_IOCTL_ENABLE_CMD_RCV	_IO(ELE_MU_IOCTL, 0x01)

/*
 * ioctl to get configure the SCU shared buffer.
 */
#define ELE_MU_IOCTL_SHARED_BUF_CFG	_IOW(ELE_MU_IOCTL, 0x02, \
					     struct ele_mu_ioctl_shared_mem_cfg)
/*
 * ioctl to get the buffer allocated from the memory, which is shared
 * between kernel and FW.
 * Post allocation, the kernel tagged the allocated memory with:
 *  Output
 *  Input
 *  Input-Output
 *  Short address
 *  Secure-memory
 */
#define ELE_MU_IOCTL_SETUP_IOBUF	_IOWR(ELE_MU_IOCTL, 0x03, \
					struct ele_mu_ioctl_setup_iobuf)

/*
 * ioctl to get the mu information, that is used to exchange message
 * with FW, from user-spaced.
 */
#define ELE_MU_IOCTL_GET_MU_INFO	_IOR(ELE_MU_IOCTL, 0x04, \
					struct ele_mu_ioctl_get_mu_info)

/*
 * ioctl to send signed message to SE.
 */
#define ELE_MU_IOCTL_SIGNED_MESSAGE     _IOR(ELE_MU_IOCTL, 0x05, \
					struct ele_mu_ioctl_signed_message)

/*
 * ioctl to get SoC Info from user-space.
 */
#define ELE_MU_IOCTL_GET_SOC_INFO      _IOR(ELE_MU_IOCTL, 0x06, \
					struct ele_mu_ioctl_get_soc_info)

/*
 * ioctl to capture the timestamp at the request to FW and response from FW
 * for a crypto operation
 */
#define ELE_MU_IOCTL_GET_TIMER	_IOR(ELE_MU_IOCTL, 0x08, struct ele_time_frame)
#endif
