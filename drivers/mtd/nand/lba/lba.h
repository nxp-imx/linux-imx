/*
 * Freescale STMP37XX/STMP378X LBA interface
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

#ifndef __INCLUDE_LBA_H__
#define __INCLUDE_LBA_H__


#include <linux/spinlock.h>
#include <linux/kthread.h>
#include "gpmi.h"

struct lba_cmd {
	uint8_t len;
#define F_MASK       0x0f
#define F_ALE        0x01
#define F_CMD        0x02
#define F_DATA_READ  0x04
#define F_DATA_WRITE 0x08

#define FE_W4R        0x10
#define FE_CMD_INC    0x20
#define FE_END        0x40

	uint8_t flag;
};

#define LBA_P_READ_A 0
#define LBA_P_READ_B 2
#define LBA_P_READ_C 3
#define LBA_P_WRITE_A 0
#define LBA_P_WRITE_B 4
#define LBA_T_SIZE1 1
#define LBA_T_SIZE4 2
#define LBA_T_SIZE8 4
#define LBA_T_CRC   (1 << 6)
#define LBA_T_ECC_CHECK (2 << 6)
#define LBA_T_ECC_CORRECT (3 << 6)

struct lba_data {
	void __iomem *io_base;
	struct clk *clk;
	int irq;

	spinlock_t lock;
	int use_count;
	int mode;
	struct semaphore mode_lock;
#define LBA_MODE_MASK       0x0000ffff
#define LBA_FLAG_MASK       0xffff0000
#define LBA_MODE_RST        0
#define LBA_MODE_PNR        1
#define LBA_MODE_BCM        2
#define LBA_MODE_MDP        3
#define LBA_MODE_VFP        4
#define LBA_MODE_SUSP       5
#define LBA_MODE_SELFPM     0x80000000
	wait_queue_head_t	suspend_q;
	wait_queue_head_t	selfpm_q;
	struct task_struct	*thread;
	long long last_access;
	/* PNR specific */
	/* BCM specific */
	/* VFP specific */
	uint8_t pass[2];

	/* Size of the partiotions: pages for PNP; sectors for others */
	unsigned int   pnp_size;
	unsigned int   vfp_size;
	long long      mdp_size;
	void *priv;
	/*should be last*/
	struct gpmi_perchip_data nand[0];

};

extern struct lba_data *g_data;

void stmp37cc_dma_print_chain(struct stmp37xx_circ_dma_chain *chain);

int lba_blk_init(struct lba_data *data);
int lba_blk_remove(struct lba_data *data);
int lba_blk_suspend(struct platform_device *pdev, struct lba_data *data);
int lba_blk_resume(struct platform_device *pdev, struct lba_data *data);


int lba_core_init(struct lba_data *data);
int lba_core_remove(struct lba_data *data);
int lba_core_suspend(struct platform_device *pdev, struct lba_data *data);
int lba_core_resume(struct platform_device *pdev, struct lba_data *data);
int lba_core_lock_mode(struct lba_data *data, int mode);
int lba_core_unlock_mode(struct lba_data *data);

int lba_write_sectors(void *priv, unsigned int sector,	unsigned int count,
		      void *buffer, dma_addr_t handle);
int lba_read_sectors(void *priv, unsigned int sector,	unsigned int count,
		      void *buffer, dma_addr_t handle);
void lba_protocol1_set(void *priv, uint8_t param);
uint8_t lba_protocol1_get(void *priv);
uint8_t lba_get_status1(void *priv);
uint8_t lba_get_status2(void *priv);

uint8_t lba_get_id1(void *priv, uint8_t *ret_buffer);
uint8_t lba_get_id2(void *priv, uint8_t *);


int queue_cmd(void *priv,
	      uint8_t *cmd_buf, dma_addr_t cmd_handle, int cmd_len,
	      dma_addr_t data, int data_len,
	      struct lba_cmd *cmd_flags);

int queue_run(void *priv);

dma_addr_t queue_get_cmd_handle(void *priv);

uint8_t *queue_get_cmd_ptr(void *priv);

dma_addr_t queue_get_data_handle(void *priv);

uint8_t *queue_get_data_ptr(void *priv);

void queue_plug(struct lba_data *data);
void queue_release(struct lba_data *data);


#endif
