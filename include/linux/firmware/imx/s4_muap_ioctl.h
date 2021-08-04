/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause*/
/*
 * Copyright 2019-2020 NXP
 */

#ifndef S4_MUAP_IOCTL_H
#define S4_MUAP_IOCTL_H

/* IOCTL definitions. */

#define MAX_IMG_COUNT	5
#define HASH_MAX_LEN	64
#define IV_MAX_LEN	32

struct s4_muap_ioctl_signed_message {
	u8 *message;
	u32 msg_size;
	u32 error_code;
};

struct container_hdr {
	u8 version;
	u8 length_lsb;
	u8 length_msb;
	u8 tag;
	u32 flags;
	u16 sw_version;
	u8 fuse_version;
	u8 num_images;
	u16 sig_blk_offset;
	u16 reserved;
} __packed;

struct image_info {
	u32 offset;
	u32 size;
	u64 dst;
	u64 entry;
	u32 hab_flags;
	u32 meta;
	u8 hash[HASH_MAX_LEN];
	u8 iv[IV_MAX_LEN];
} __packed;

struct s4_muap_auth_image {
	struct container_hdr chdr;
	struct image_info img_info[MAX_IMG_COUNT];
	u32 resp;
};

struct s4_read_info {
	u32 cmd_id;
	u32 size;
	u32 resp[8];
};

#define S4_MUAP_IOCTL			0x0A /* like MISC_MAJOR. */

#define S4_MUAP_IOCTL_GET_INFO_CMD	_IOWR(S4_MUAP_IOCTL, 0x01,\
					struct s4_read_info)

#define S4_MUAP_IOCTL_IMG_AUTH_CMD	_IOWR(S4_MUAP_IOCTL, 0x02,\
					struct s4_muap_auth_image)

#define S4_MUAP_IOCTL_SIGNED_MSG_CMD	_IOWR(S4_MUAP_IOCTL, 0x03,\
					struct s4_muap_ioctl_signed_message)
#endif
