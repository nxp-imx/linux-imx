/*
 * Copyright (C) 2014-2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/busfreq-imx6.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include "common.h"
#include "hardware.h"
#include "mcc_config.h"
#include <linux/imx_sema4.h>
#include <linux/mcc_imx6sx.h>
#include <linux/mcc_linux.h>

#define MU_ATR0_OFFSET	0x0
#define MU_ARR0_OFFSET	0x10
#define MU_ASR		0x20
#define MU_ACR		0x24

#define MU_LPM_HANDSHAKE_INDEX		0
#define MU_LPM_BUS_HIGH_READY_FOR_M4	0xFFFF6666
#define MU_LPM_M4_FREQ_CHANGE_READY	0xFFFF7777
#define MU_LPM_M4_REQUEST_HIGH_BUS	0x2222CCCC
#define MU_LPM_M4_RELEASE_HIGH_BUS	0x2222BBBB
#define MU_LPM_M4_WAKEUP_SRC_VAL	0x55555000
#define MU_LPM_M4_WAKEUP_SRC_MASK	0xFFFFF000
#define MU_LPM_M4_WAKEUP_IRQ_MASK	0xFF0
#define MU_LPM_M4_WAKEUP_IRQ_SHIFT	0x4
#define MU_LPM_M4_WAKEUP_ENABLE_MASK	0xF
#define MU_LPM_M4_WAKEUP_ENABLE_SHIFT	0x0

static void __iomem *mu_base;
static u32 m4_message;
static struct delayed_work mu_work;
static u32 m4_wake_irqs[4];
static bool m4_freq_low;

struct imx_sema4_mutex *mcc_shm_ptr;
unsigned int imx_mcc_buffer_freed = 0, imx_mcc_buffer_queued = 0;
/* Used for blocking send */
static DECLARE_WAIT_QUEUE_HEAD(buffer_freed_wait_queue);
/* Used for blocking recv */
static DECLARE_WAIT_QUEUE_HEAD(buffer_queued_wait_queue);

bool imx_mu_is_m4_in_low_freq(void)
{
	return m4_freq_low;
}

void imx_mu_enable_m4_irqs_in_gic(bool enable)
{
	int i, j;

	for (i = 0; i < 4; i++) {
		if (m4_wake_irqs[i] == 0)
			continue;
		for (j = 0; j < 32; j++) {
			if (m4_wake_irqs[i] & (1 << j)) {
				if (enable)
					enable_irq((i + 1) * 32 + j);
				else
					disable_irq((i + 1) * 32 + j);
			}
		}
	}
}

static irqreturn_t mcc_m4_dummy_isr(int irq, void *param)
{
	return IRQ_HANDLED;
}

static void imx_mu_send_message(unsigned int index, unsigned int data)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	/* wait for transfer buffer empty */
	do {
		val = readl_relaxed(mu_base + MU_ASR);
		if (time_after(jiffies, timeout)) {
			pr_err("Waiting MU transmit buffer empty timeout!\n");
			break;
		}
	} while ((val & (1 << (20 + index))) == 0);

	writel_relaxed(data, mu_base + index * 0x4 + MU_ATR0_OFFSET);
}

static void mu_work_handler(struct work_struct *work)
{
	int ret;
	u32 irq, enable, idx, mask;

	pr_debug("receive M4 message 0x%x\n", m4_message);

	switch (m4_message) {
	case MU_LPM_M4_REQUEST_HIGH_BUS:
		request_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(true);
		imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_BUS_HIGH_READY_FOR_M4);
		m4_freq_low = false;
		break;
	case MU_LPM_M4_RELEASE_HIGH_BUS:
		release_bus_freq(BUS_FREQ_HIGH);
		imx6sx_set_m4_highfreq(false);
		imx_mu_send_message(MU_LPM_HANDSHAKE_INDEX,
			MU_LPM_M4_FREQ_CHANGE_READY);
		m4_freq_low = true;
		break;
	default:
		if ((m4_message & MU_LPM_M4_WAKEUP_SRC_MASK) ==
			MU_LPM_M4_WAKEUP_SRC_VAL) {
			irq = (m4_message & MU_LPM_M4_WAKEUP_IRQ_MASK) >>
				MU_LPM_M4_WAKEUP_IRQ_SHIFT;

			enable = (m4_message & MU_LPM_M4_WAKEUP_ENABLE_MASK) >>
				MU_LPM_M4_WAKEUP_ENABLE_SHIFT;

			idx = irq / 32 - 1;
			mask = 1 << irq % 32;

			if (enable && can_request_irq(irq, 0)) {
				ret = request_irq(irq, mcc_m4_dummy_isr,
					IRQF_NO_SUSPEND, "imx-m4-dummy", NULL);
				if (ret) {
					pr_err("%s: register interrupt %d failed, rc %d\n",
						__func__, irq, ret);
					break;
				}
				disable_irq(irq);
				m4_wake_irqs[idx] = m4_wake_irqs[idx] | mask;
			}
			imx_gpc_add_m4_wake_up_irq(irq, enable);
		}
		break;
	}
	m4_message = 0;
	/* enable RIE3 interrupt */
	writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(27),
		mu_base + MU_ACR);
}

/*!
 * \brief This function clears the CPU-to-CPU int flag for the particular core.
 *
 * Implementation is platform-specific.
 */
void mcc_clear_cpu_to_cpu_interrupt(void)
{
	u32 val;

	val = readl_relaxed(mu_base + MU_ASR);
	/* write 1 to BIT31 to clear the bit31(GIP3) of MU_ASR */
	val = val | (1 << 31);
	writel_relaxed(val, mu_base + MU_ASR);
}

/*!
 * \brief This function triggers the CPU-to-CPU interrupt.
 *
 * Platform-specific software triggering the inter-CPU interrupts.
 */
int mcc_triger_cpu_to_cpu_interrupt(void)
{
	int i = 0;
	u32 val;

	val = readl_relaxed(mu_base + MU_ACR);

	if ((val & BIT(19)) != 0) {
		do {
			val = readl_relaxed(mu_base + MU_ACR);
			msleep(1);
		} while (((val & BIT(19)) > 0) && (i++ < 100));
	}

	if ((val & BIT(19)) == 0) {
		/* Enable the bit19(GIR3) of MU_ACR */
		val = readl_relaxed(mu_base + MU_ACR);
		val |= BIT(19);
		writel_relaxed(val, mu_base + MU_ACR);
		return 0;
	} else {
		pr_info("mcc int still be triggered after %d ms polling!\n", i);
		return -EIO;
	}
}

/*!
 * \brief This function disable the CPU-to-CPU interrupt.
 *
 * Platform-specific software disable the inter-CPU interrupts.
 */
int imx_mcc_bsp_int_disable(void)
{
	u32 val;

	/* Disablethe bit31(GIE3) and bit19(GIR3) of MU_ACR */
	val = readl_relaxed(mu_base + MU_ACR);
	val &= ~(BIT(31) | BIT(27));
	writel_relaxed(val, mu_base + MU_ACR);

	/* flush */
	val = readl_relaxed(mu_base + MU_ACR);
	return 0;
}

/*!
 * \brief This function enable the CPU-to-CPU interrupt.
 *
 * Platform-specific software enable the inter-CPU interrupts.
 */
int imx_mcc_bsp_int_enable(void)
{
	u32 val;

	/* Enable bit31(GIE3) and bit19(GIR3) of MU_ACR */
	val = readl_relaxed(mu_base + MU_ACR);
	val |= (BIT(31) | BIT(27));
	writel_relaxed(val, mu_base + MU_ACR);

	/* flush */
	val = readl_relaxed(mu_base + MU_ACR);
	return 0;
}

int mcc_wait_for_buffer_freed(MCC_RECEIVE_BUFFER **buffer, unsigned int timeout)
{
    int return_value;
    unsigned long timeout_j; /* jiffies */
    MCC_RECEIVE_BUFFER *buf = null;

	/*
	 * Blocking calls: CPU-to-CPU ISR sets the event and thus
	 * resumes tasks waiting for a free MCC buffer.
	 * As the interrupt request is send to all cores when a buffer
	 * is freed it could happen that several tasks from different
	 * cores/nodes are waiting for a free buffer and all of them
	 * are notified that the buffer has been freed. This function
	 * has to check (after the wake up) that a buffer is really
	 * available and has not been already grabbed by another
	 * "competitor task" that has been faster. If so, it has to
	 * wait again for the next notification.
	 */
	while (buf == null) {
		if (timeout == 0xffffffff) {
			/*
			 * In order to level up the robust, do not always
			 * wait event here. Wake up itself after every 1~s.
			 */
			timeout_j = usecs_to_jiffies(1000);
			wait_event_timeout(buffer_freed_wait_queue,
					imx_mcc_buffer_freed == 1, timeout_j);
		} else {
			timeout_j = msecs_to_jiffies(timeout);
			wait_event_timeout(buffer_freed_wait_queue,
					imx_mcc_buffer_freed == 1, timeout_j);
		}

		return_value = mcc_get_semaphore();
		if (return_value != MCC_SUCCESS)
			return return_value;

		MCC_DCACHE_INVALIDATE_MLINES((void *)
				&bookeeping_data->free_list,
				sizeof(MCC_RECEIVE_LIST *));

		buf = mcc_dequeue_buffer(&bookeeping_data->free_list);
		mcc_release_semaphore();
		if (imx_mcc_buffer_freed)
			imx_mcc_buffer_freed = 0;
	}

	*buffer = buf;
	return MCC_SUCCESS;
}

int mcc_wait_for_buffer_queued(MCC_ENDPOINT *endpoint, unsigned int timeout)
{
	unsigned long timeout_j; /* jiffies */
	MCC_RECEIVE_LIST *tmp_list;

	/* Get list of buffers kept by the particular endpoint */
	tmp_list = mcc_get_endpoint_list(*endpoint);

	if (timeout == 0xffffffff) {
		wait_event(buffer_queued_wait_queue,
				imx_mcc_buffer_queued == 1);
		mcc_get_semaphore();
		/*
		* double check if the tmp_list head is still null
		* or not, if yes, wait again.
		*/
		while (tmp_list->head == null) {
			imx_mcc_buffer_queued = 0;
			mcc_release_semaphore();
			wait_event(buffer_queued_wait_queue,
					imx_mcc_buffer_queued == 1);
			mcc_get_semaphore();
		}
	} else {
		timeout_j = msecs_to_jiffies(timeout);
		wait_event_timeout(buffer_queued_wait_queue,
				imx_mcc_buffer_queued == 1, timeout_j);
		mcc_get_semaphore();
	}

	if (imx_mcc_buffer_queued)
		imx_mcc_buffer_queued = 0;

	if (tmp_list->head == null) {
		pr_err("%s can't get queued buffer.\n", __func__);
		mcc_release_semaphore();
		return MCC_ERR_TIMEOUT;
	}

	tmp_list->head = (MCC_RECEIVE_BUFFER *)
		MCC_MEM_PHYS_TO_VIRT(tmp_list->head);
	mcc_release_semaphore();

	return MCC_SUCCESS;
}

static irqreturn_t imx_mu_isr(int irq, void *param)
{
	u32 irqs;

	irqs = readl_relaxed(mu_base + MU_ASR);

	if (irqs & (1 << 27)) {
		/* get message from receive buffer */
		m4_message = readl_relaxed(mu_base + MU_ARR0_OFFSET);
		/* disable RIE3 interrupt */
		writel_relaxed(readl_relaxed(mu_base + MU_ACR) & (~BIT(27)),
			mu_base + MU_ACR);
		schedule_delayed_work(&mu_work, 0);
	}

	/*
	 * MCC CPU-to-CPU interrupt.
	 * Each core can interrupt the other. There are two logical signals:
	 * - Receive data available for (Node,Port)
	 * - signaled when a buffer is queued to a Receive Data Queue.
	 * - Buffer available
	 * - signaled when a buffer is queued to the Free Buffer Queue.
	 * It is possible that several signals can occur while one interrupt
	 * is being processed.
	 * Therefore, a Receive Signal Queue of received signals is also
	 * required
	 * - one for each core.
	 * The interrupting core queues to the tail and the interrupted core
	 * pulls from the head.
	 * For a circular file, no semaphore is required since only the sender
	 * modifies the tail and only the receiver modifies the head.
	 */
	if (irqs & (1 << 31)) {
		/*
		 * Try to lock the core mutex. If successfully locked, perform
		 * mcc_dequeue_signal(), release the gate and finally clear the
		 * interrupt flag. If trylock fails (HW semaphore already locked
		 * by another core), do not clear the interrupt flag  this
		 * way the CPU-to-CPU isr is re-issued again until the HW
		 * semaphore is locked. Higher priority ISRs will be serviced
		 * while issued at the time we are waiting for the unlocked
		 * gate. To prevent trylog failure due to core mutex currently
		 * locked by our own core(a task), the cpu-to-cpu isr is
		 * temporarily disabled when mcc_get_semaphore() is called and
		 * re-enabled again when mcc_release_semaphore() is issued.
		 */
		MCC_SIGNAL serviced_signal;
		if (SEMA4_A9_LOCK == imx_sema4_mutex_trylock(mcc_shm_ptr)) {
			while (MCC_SUCCESS == mcc_dequeue_signal(
				MCC_CORE_NUMBER, &serviced_signal)) {
				if ((serviced_signal.type == BUFFER_QUEUED) &&
					(serviced_signal.destination.core ==
					MCC_CORE_NUMBER)) {
					/*
					 * Unblock receiver, in case of
					 * asynchronous communication
					 */
					imx_mcc_buffer_queued = 1;
					wake_up(&buffer_queued_wait_queue);
				} else if (serviced_signal.type ==
					BUFFER_FREED) {
					/*
					 * Unblock sender, in case of
					 * asynchronous communication
					 */
					imx_mcc_buffer_freed = 1;
					wake_up(&buffer_freed_wait_queue);
				}
			}

			/* Clear the interrupt flag */
			mcc_clear_cpu_to_cpu_interrupt();

			/* Unlocks the core mutex */
			imx_sema4_mutex_unlock(mcc_shm_ptr);
		}
	}

	return IRQ_HANDLED;
}

static int imx_mu_probe(struct platform_device *pdev)
{
	int ret;
	u32 irq;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6sx-mu");
	mu_base = of_iomap(np, 0);
	WARN_ON(!mu_base);

	irq = platform_get_irq(pdev, 0);

	ret = request_irq(irq, imx_mu_isr,
		IRQF_EARLY_RESUME, "imx-mu", NULL);
	if (ret) {
		pr_err("%s: register interrupt %d failed, rc %d\n",
			__func__, irq, ret);
		return ret;
	}
	INIT_DELAYED_WORK(&mu_work, mu_work_handler);

	/* enable the bit27(RIE3) of MU_ACR */
	writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(27),
		mu_base + MU_ACR);
	/* enable the bit31(GIE3) of MU_ACR, used for MCC */
	writel_relaxed(readl_relaxed(mu_base + MU_ACR) | BIT(31),
		mu_base + MU_ACR);

	/* MU always as a wakeup source for low power mode */
	imx_gpc_add_m4_wake_up_irq(irq, true);

	pr_info("MU is ready for cross core communication!\n");

	return 0;
}

static const struct of_device_id imx_mu_ids[] = {
	{ .compatible = "fsl,imx6sx-mu" },
	{ }
};

static struct platform_driver imx_mu_driver = {
	.driver = {
		.name   = "imx-mu",
		.owner  = THIS_MODULE,
		.of_match_table = imx_mu_ids,
	},
	.probe = imx_mu_probe,
};

static int __init imx6_mu_init(void)
{
	return platform_driver_register(&imx_mu_driver);
}
subsys_initcall(imx6_mu_init);
