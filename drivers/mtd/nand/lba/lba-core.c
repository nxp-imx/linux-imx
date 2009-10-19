/*
 * Freescale STMP37XX/STMP378X LBA/core driver
 *
 * Author: Dmitrij Frasenyak <sed@embeddedalley.com>
 *
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/ctype.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/stmp3xxx.h>
#include <mach/dma.h>
#include "gpmi.h"
#include "lba.h"

#define LBA_SELFPM_TIMEOUT 2000 /* msecs */
dma_addr_t g_cmd_handle;
dma_addr_t g_data_handle;
uint8_t    *g_data_buffer;
uint8_t    *g_cmd_buffer;

uint8_t lba_get_status1(void *priv)
{
	uint8_t cmd_buf[] = { 0x70 } ;
	struct lba_cmd lba_flags[] = {
		{1 , F_CMD |  FE_W4R},
		{0,  F_DATA_READ | FE_END},
	};
	*g_data_buffer = 0;
	queue_cmd(priv, cmd_buf, 0, 1, g_data_handle, 1, lba_flags);
	queue_run(priv);
	return *g_data_buffer;
}


int lba_wait_for_ready(void *priv)
{
	int stat;
	unsigned long j_start = jiffies;

	stat = lba_get_status1(priv);
	if ((stat & 0x60) != 0x60) {
		while (((stat & 0x60) != 0x60) &&
		       (jiffies - j_start < msecs_to_jiffies(2000))) {
			schedule();
			stat = lba_get_status1(priv);
		}
	}
	if (stat != 0x60)
		return stat;

	return 0;
}

int lba_write_sectors(void *priv, unsigned int sector,	unsigned int count,
		      void *buffer, dma_addr_t handle)
{
	uint8_t cmd_buf[] = {
		0x80,
		count & 0xff, (count >> 8) & 0xff, /* Count */
		(sector & 0xff), (sector >> 8) & 0xff, /* Address */
		(sector >> 16) & 0xff, (sector >> 24) & 0xff, /* Addres */
		/* Data goes here */
		0x10
	};

	struct lba_cmd flags_t1[] = { /* Transmition mode 1/A */
		{7 , F_CMD | FE_CMD_INC | FE_W4R},
		{0,  F_DATA_WRITE},
		{1 , F_CMD | FE_END}
	};

	if (count > 8)
		return -EINVAL;

	if (lba_wait_for_ready(priv))
		return -EIO;

	while (count) {
		int cnt = (count < 8) ? count : 8;
		int data_len = cnt * 512;

		queue_cmd(priv, cmd_buf, 0, 8,
			  handle, data_len, flags_t1);

		handle += data_len;
		count -= cnt;

	}

	queue_run(priv);

	return count;

}

int lba_read_sectors(void *priv, unsigned int sector,	unsigned int count,
		      void *buffer, dma_addr_t handle)
{

	int data_len;
	int cnt;
	uint8_t cmd_buf[] = {
		0x00,
		count & 0xff, (count >> 8) & 0xff, /* Count */
		(sector & 0xff), (sector >> 8) & 0xff, /* Addr */
		(sector >> 16) & 0xff, (sector >> 24) & 0xff, /* Addr */
		0x30
		/* Data goes here <data> */

	};

	struct lba_cmd flags_r3[] = { /* Read mode 3/A */
		{7 , F_CMD | FE_CMD_INC | FE_W4R},
		{1 , F_CMD },
		{0 , F_DATA_READ | FE_W4R | FE_END  },
	};
	struct lba_cmd flags_r3c[] = { /* Read mode 3/A */
		{0 , F_DATA_READ | FE_W4R | FE_END },
	};
	struct lba_cmd *flags = flags_r3;
	int flags_len = 8;

	if (count > 8)
		return -EINVAL;

	if (lba_wait_for_ready(priv))
		return -EIO;

	while (count) {
		cnt = (count < 8) ? count : 8;
		data_len = cnt * 512;
		queue_cmd(priv, cmd_buf, 0, flags_len, handle, data_len, flags);
		handle += data_len;
		count -= cnt;
		flags = flags_r3c;
		flags_len = 0;
	}

	queue_run(priv);

	return count;

}


uint8_t lba_get_id1(void *priv, uint8_t *ret_buffer)
{
	uint8_t cmd_buf[] = { 0x90 , 0x00, /* Data read 5bytes*/ };
	struct lba_cmd lba_flags[] = {
		{2 , F_CMD | FE_CMD_INC | FE_W4R},
		{0,  F_DATA_READ | FE_END},
	};

	queue_cmd(priv, cmd_buf, 0, 2, g_data_handle, 5, lba_flags);
	queue_run(priv);
	memcpy(ret_buffer, g_data_buffer, 5);

	return 0;
}

uint8_t lba_get_id2(void *priv, uint8_t *ret_buffer)
{
	uint8_t cmd_buf[] = { 0x92 , 0x00, /* Data read 5bytes*/ };
	struct lba_cmd lba_flags[] = {
		{2 , F_CMD | FE_CMD_INC | FE_W4R},
		{0,  F_DATA_READ | FE_END},
	};

	queue_cmd(priv, cmd_buf, 0, 2, g_data_handle, 5, lba_flags);
	queue_run(priv);
	memcpy(ret_buffer, g_data_buffer, 5);
	return 0;
}

uint8_t lba_get_status2(void *priv)
{
	uint8_t cmd_buf[] = { 0x71 };
	struct lba_cmd lba_flags[] = {
		{1 , F_CMD |  FE_CMD_INC | FE_W4R},
		{0 , F_DATA_READ | FE_END},
	};
	*g_data_buffer = 0;
	queue_cmd(priv, cmd_buf, 0, 1, g_data_handle, 1, lba_flags);
	queue_run(priv);
	return *g_data_buffer;
}

static uint8_t lba_parse_status2(void *priv)
{
	uint8_t stat;

	stat = lba_get_status2(priv);
	printk(KERN_INFO "Status2:|");
	if (stat & 0x40)
		printk(" C.PAR.ERR |"); /* no KERN_ here */
	if (stat & 0x20)
		printk(" NO spare |");
	if (stat & 0x10)
		printk(" ADDR OoRange |");
	if (stat & 0x8)
		printk(" high speed |");
	if ((stat & 0x6) == 6)
		printk(" MDP |");
	if ((stat & 0x6) == 4)
		printk(" VFP |");
	if ((stat & 0x6) == 2)
		printk(" PNP |");
	if (stat & 1)
		printk(" PSW |");

	printk("\n");
	return 0;
}


int lba_2mdp(void *priv)
{
	uint8_t cmd_buf[] = { 0xFC  };
	struct lba_cmd lba_flags[] = {
		{1 , F_CMD | FE_W4R | FE_END}
	};

	queue_cmd(priv, cmd_buf, 0, 1, 0, 0, lba_flags);
	queue_run(priv);
	return 0;
}

void _lba_misc_cmd_set(void *priv, uint8_t *cmd_buf)
{
	struct lba_cmd lba_flags[] = {
		{6 , F_CMD | FE_CMD_INC | FE_W4R },
		{1 , F_CMD | FE_END },
	};

	queue_cmd(priv, cmd_buf, 0, 7, 0, 0, lba_flags);
	queue_run(priv);
}

uint8_t _lba_misc_cmd_get(void *priv, uint8_t *cmd_buf)
{
	struct lba_cmd lba_flags[] = {
		{6 , F_CMD | FE_CMD_INC | FE_W4R },
		{1 , F_CMD  },
		{0 , F_DATA_READ | FE_W4R | FE_END}
	};

	queue_cmd(priv, cmd_buf, 0, 7, g_data_handle, 1, lba_flags);
	queue_run(priv);
	return *g_data_buffer;
}
void lba_mdp2vfp(void *priv, uint8_t pass[2])
{
	uint8_t cmd_buf[] = { 0x0, 0xbe, pass[0], pass[1], 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

void lba_bcm2vfp(void *priv, uint8_t pass[2])
{
	lba_mdp2vfp(priv, pass);
}

void lba_powersave_enable(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xba, 0, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}
void lba_powersave_disable(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xbb, 0, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

void lba_highspeed_enable(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xbc, 0, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

void lba_highspeed_disable(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xbd, 0, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

void lba_prot1_set(void *priv, uint8_t mode)
{
	uint8_t cmd_buf[] = { 0x0, 0xa2, mode, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

void lba_prot2_set(void *priv, uint8_t mode)
{
	uint8_t cmd_buf[] = { 0x0, 0xa3, mode, 0, 0, 0, 0x57 };
	_lba_misc_cmd_set(priv, cmd_buf);
}

uint8_t lba_prot1_get(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xb2, 0, 0, 0, 0, 0x57 };
	return _lba_misc_cmd_get(priv, cmd_buf);
}

uint8_t lba_prot2_get(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xb3, 0, 0, 0, 0, 0x57 };
	return _lba_misc_cmd_get(priv, cmd_buf);
}

uint64_t lba_mdp_size_get(void *priv)
{
	uint8_t cmd_buf[] = { 0x0, 0xb0, 0, 0, 0, 0, 0x57 };
	struct lba_cmd lba_flags[] = {
		{6 , F_CMD | FE_CMD_INC | FE_W4R },
		{1 , F_CMD  },
		{0 , F_DATA_READ | FE_W4R | FE_END}
	};

	memset((void *)g_data_buffer, 0, 8);
	queue_cmd(priv, cmd_buf, 0, 7, g_data_handle, 5, lba_flags);
	queue_run(priv);
	return le64_to_cpu(*(long long *)g_data_buffer);
}

void lba_cache_flush(void *priv)
{
	uint8_t cmd_buf[] = { 0xF9 };
	struct lba_cmd lba_flags[] = {
		{1 , F_CMD | FE_W4R },
		{0 , FE_W4R | FE_END}
	};

	queue_cmd(priv, cmd_buf, 0, 7, g_data_handle, 5, lba_flags);
	queue_run(priv);
}

void lba_reboot(void *priv)
{
	uint8_t cmd_buf[] = { 0xFD };
	struct lba_cmd lba_flags[] = {
		{1 , F_CMD | FE_W4R },
		{0 , FE_W4R | FE_END}
	};

	queue_cmd(priv, cmd_buf, 0, 7, g_data_handle, 5, lba_flags);
	queue_run(priv);
}

void lba_def_state(void *priv)
{
	lba_wait_for_ready(priv);
	lba_reboot(priv);

	lba_wait_for_ready(priv);
	lba_parse_status2(priv);

	lba_wait_for_ready(priv);
	lba_2mdp(priv);

	lba_wait_for_ready(priv);
	lba_prot1_set(priv, LBA_T_SIZE8); /* 512 * 8 */

	lba_wait_for_ready(priv);
/* Type C read; Type A write; */
	lba_prot2_set(priv, LBA_P_WRITE_A | LBA_P_READ_C);
}

/*
 * Should be called with mode locked
 */
void lba_core_setvfp_passwd(struct lba_data *data, uint8_t pass[2])
{
	memcpy(data->pass, pass, 2);
}

int lba_core_lock_mode(struct lba_data *data, int mode)
{
	void *priv = &data->nand;

	if (down_interruptible(&data->mode_lock))
		return -EAGAIN;
	/*
	 * MDP and VFP are the only supported
	 * modes for now.
	 */
	if ((mode != LBA_MODE_MDP) &&
	    (mode != LBA_MODE_VFP)) {
		up(&data->mode_lock);
		return -EINVAL;
	}

	while ((data->mode & LBA_MODE_MASK) == LBA_MODE_SUSP) {
		up(&data->mode_lock);

		if (wait_event_interruptible(
			    data->suspend_q,
			    (data->mode & LBA_MODE_MASK) != LBA_MODE_SUSP))
			return -EAGAIN;

		if (down_interruptible(&data->mode_lock))
			return -EAGAIN;

		data->last_access = jiffies;
	}

	if (data->mode & LBA_MODE_SELFPM) {
		queue_plug(data);
		data->mode &= ~LBA_MODE_SELFPM;
	}

	if (mode == data->mode)
		return 0;

	/*
	 * mode = VFP || MDP only
	 * Revisit when more modes are added
	 */
	switch (data->mode) {
	case LBA_MODE_RST:
	case LBA_MODE_PNR:
	case LBA_MODE_BCM:
		lba_def_state(priv);
		if (mode == LBA_MODE_MDP) {
			data->mode = LBA_MODE_MDP;
			break;
		}
		/*no break -> fall down to set VFP mode*/
	case LBA_MODE_MDP:
		lba_wait_for_ready(priv);
		lba_mdp2vfp(priv, data->pass);
		data->mode = LBA_MODE_VFP;
		break;
	case LBA_MODE_VFP:
		lba_wait_for_ready(priv);
		lba_2mdp(priv);
		data->mode = LBA_MODE_MDP;
		break;
	default:
		up(&data->mode_lock);
		return -EINVAL;
	}

	return 0;
}

int lba_core_unlock_mode(struct lba_data *data)
{
	data->last_access = jiffies;
	up(&data->mode_lock);
	wake_up(&data->selfpm_q);
	return 0;
}

static int selfpm_timeout_expired(struct lba_data *data)
{
	return  jiffies_to_msecs(jiffies - data->last_access) > 2000;
}

static int lba_selfpm_thread(void *d)
{
	struct lba_data *data = d;

	set_user_nice(current, -5);

	while (!kthread_should_stop()) {

		if (wait_event_interruptible(data->selfpm_q,
			   kthread_should_stop() ||
			    !(data->mode & LBA_MODE_SELFPM)))
			continue;

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(LBA_SELFPM_TIMEOUT));

		if (down_trylock(&data->mode_lock))
			continue;

		if (!selfpm_timeout_expired(data)) {
			up(&data->mode_lock);
			continue;
		}
		data->mode |= LBA_MODE_SELFPM;
		lba_wait_for_ready((void *)data->nand);
		lba_cache_flush((void *)data->nand);
		queue_release(data);
		up(&data->mode_lock);

	}

	return 0;
}

int lba_core_init(struct lba_data *data)
{
	uint8_t id_buf[5];
	uint8_t capacity;
	uint8_t id1_template[5] = {0x98, 0xDC, 0x00, 0x15, 0x00};
	uint8_t id2_template[5] = {0x98, 0x21, 0x00, 0x55, 0xAA};
	void *priv = (void *)data->nand;


	g_data = data;
	g_cmd_handle = queue_get_cmd_handle(priv);
	g_data_handle = queue_get_data_handle(priv);
	g_data_buffer = queue_get_data_ptr(priv);
	g_cmd_buffer = queue_get_cmd_ptr(priv);

	spin_lock_init(&data->lock);
	sema_init(&data->mode_lock, 1);
	init_waitqueue_head(&data->suspend_q);
	init_waitqueue_head(&data->selfpm_q);


	lba_get_id1(data->nand, id_buf);
	if (!memcmp(id_buf, id1_template, 5))
		printk(KERN_INFO
		       "LBA: Found LBA/SLC NAND emulated ID\n");
	else
		return -ENODEV;

	lba_get_id2(data->nand, id_buf);
	capacity = id_buf[2];
	id_buf[2] = 0;

	if (memcmp(id_buf, id2_template, 5)) {
		printk(KERN_INFO
		       "LBA: Uknown LBA device\n");
		return -ENODEV;
	}
	printk(KERN_INFO
	       "LBA: Found %dGbytes LBA NAND device\n",
	       1 << capacity);

	lba_wait_for_ready(priv);
	lba_parse_status2(priv);

	lba_def_state(priv);
	data->mode = LBA_MODE_MDP;

	g_data->pnp_size = 0xff;
	g_data->vfp_size = 16384;

	lba_wait_for_ready(priv);
	g_data->mdp_size = lba_mdp_size_get(priv);

	lba_wait_for_ready(priv);
	/*lba_powersave_enable(priv);*/
	/*lba_highspeed_enable(priv);*/

	lba_wait_for_ready(priv);
	lba_parse_status2(priv);

	data->thread = kthread_create(lba_selfpm_thread,
				      data, "lba-selfpm-%d", 1);
	if (IS_ERR(data->thread))
		return  PTR_ERR(data->thread);

	lba_blk_init(g_data);

	wake_up_process(data->thread);
	return 0;

};

int lba_core_remove(struct lba_data *data)
{
	kthread_stop(data->thread);
	lba_blk_remove(data);
	lba_wait_for_ready((void *)data->nand);
	lba_cache_flush((void *)data->nand);
	return 0;
}

int lba_core_suspend(struct platform_device *pdev, struct lba_data *data)
{
	BUG_ON((data->mode & 0xffff) == LBA_MODE_SUSP);
	if (down_interruptible(&data->mode_lock))
	    return -EAGAIN;
	if (data->mode & LBA_MODE_SELFPM)
		queue_plug(data);

	data->mode = LBA_MODE_SUSP | LBA_MODE_SELFPM;
	up(&data->mode_lock);
	lba_wait_for_ready((void *)data->nand);
	lba_cache_flush((void *)data->nand);
	return 0;
}

int lba_core_resume(struct platform_device *pdev, struct lba_data *data)
{
	BUG_ON((data->mode & 0xffff) != LBA_MODE_SUSP);
	lba_def_state((void *)data->nand);
	data->last_access = jiffies;
	data->mode = LBA_MODE_MDP;
	wake_up(&data->suspend_q);
	return 0;
}
