/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause*/
/*
 * Copyright 2019-2021 NXP
 */

#ifndef SENTNL_MU_IOCTL_H
#define SENTNL_MU_IOCTL_H

/* IOCTL definitions. */

struct sentnl_mu_ioctl_setup_iobuf {
	u8 *user_buf;
	u32 length;
	u32 flags;
	u64 sentnl_addr;
};

struct sentnl_mu_ioctl_shared_mem_cfg {
	u32 base_offset;
	u32 size;
};

struct sentnl_mu_ioctl_get_mu_info {
	u8 sentnl_mu_id;
	u8 interrupt_idx;
	u8 tz;
	u8 did;
};

struct sentnl_mu_ioctl_signed_message {
	u8 *message;
	u32 msg_size;
	u32 error_code;
};

#define SENTNL_MU_IO_FLAGS_IS_INTPUT		(0x01u)
#define SENTNL_MU_IO_FLAGS_USE_SEC_MEM		(0x02u)
#define SENTNL_MU_IO_FLAGS_USE_SHORT_ADDR	(0x04u)

#define SENTNL_MU_IOCTL			0x0A /* like MISC_MAJOR. */
#define SENTNL_MU_IOCTL_ENABLE_CMD_RCV	_IO(SENTNL_MU_IOCTL, 0x01)
#define SENTNL_MU_IOCTL_SHARED_BUF_CFG	_IOW(SENTNL_MU_IOCTL, 0x02, \
					struct sentnl_mu_ioctl_shared_mem_cfg)
#define SENTNL_MU_IOCTL_SETUP_IOBUF	_IOWR(SENTNL_MU_IOCTL, 0x03, \
					struct sentnl_mu_ioctl_setup_iobuf)
#define SENTNL_MU_IOCTL_GET_MU_INFO	_IOR(SENTNL_MU_IOCTL, 0x04, \
					struct sentnl_mu_ioctl_get_mu_info)
#define SENTNL_MU_IOCTL_SIGNED_MESSAGE	_IOWR(SENTNL_MU_IOCTL, 0x05, \
					struct sentnl_mu_ioctl_signed_message)

#endif
