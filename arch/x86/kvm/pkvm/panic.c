// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <asm/io.h>
#include <asm/apic.h>
#include <asm/kvm_pkvm.h>
#include <asm/msr.h>
#include <asm-generic/bug.h>
#include "debug.h"
#include "pkvm.h"
#include "memory.h"

/*
 * NOTE: Both PERSISTENT_RAM_SIG and persistent_ram_buffer definition copied
 * from fs/pstore/ram_core.c
 */
#define PERSISTENT_RAM_SIG (0x43474244) /* DBGC */

struct persistent_ram_buffer {
	uint32_t    sig;
	atomic_t    start;
	atomic_t    size;
	uint8_t     data[];
};

static void pkvm_write_ramoops_console(const char *msg)
{
	struct persistent_ram_buffer *buffer;
	size_t msg_len = strlen(msg);
	size_t capacity;
	u32 start, size;

	/* Size check is sufficient; address validity is verified during init */
	if (!pkvm_ramoops_console_size)
		return;

	buffer = __pkvm_va(pkvm_ramoops_console_pa);

	/*
	 * Sanity check if writing to correct area by making sure it is valid
	 * persistent_ram_buffer structure which starts with PERSISTENT_RAM_SIG
	 */
	if (READ_ONCE(buffer->sig) != PERSISTENT_RAM_SIG)
		return;

	/*
	 * The ramoops console uses the 'persistent_ram_buffer' ABI defined in
	 * fs/pstore/ram_core.c. The buffer starts with a header:
	 * [u32 sig] [u32 start] [u32 size] [raw data...]
	 */
	capacity = pkvm_ramoops_console_size - sizeof(struct persistent_ram_buffer);

	/*
	 * 'start' indicates where the host stopped logging. In the
	 * persistent_ram_buffer ABI, this field is the "write pointer".
	 *
	 * By reading this value, the hypervisor knows where the host VM (which
	 * is now frozen) left off, allowing the panic message to be appended
	 * to the console log.
	 */
	start = atomic_read(&buffer->start);
	size = atomic_read(&buffer->size);

	if (start >= capacity)
		start = 0;

	/* Circular append logic */
	if (start + msg_len <= capacity) {
		memcpy(&buffer->data[start], msg, msg_len);
		start += msg_len;
	} else {
		size_t first_part = capacity - start;
		size_t second_part = msg_len - first_part;

		memcpy(&buffer->data[start], msg, first_part);
		if (second_part > capacity)
			second_part = capacity;
		memcpy(&buffer->data[0], msg + first_part, second_part);
		start = second_part;
	}

	if (start >= capacity)
		start = 0;

	/* Update total valid data size, capping at the buffer capacity */
	if (size < capacity) {
		size += msg_len;
		if (size > capacity)
			size = capacity;
	}

	/* Sync metadata so host pstore can find the new data after reboot */
	atomic_set(&buffer->start, start);
	atomic_set(&buffer->size, size);

	/* Ensure data is visible in physical RAM before the hardware reset */
	clflush_cache_range(buffer, pkvm_ramoops_console_size);
}

/*
 * To not include ACPI based reboot and its complexity, try to reset the system
 * via PCI warm reset (0xCF9), the keyboard controller (PS/2), PCI cold reset
 * register (CF9), or triple fault.
 *
 * All based on arch/x86/kernel/reboot.c native_machine_emergency_restart().
 */
static void __noreturn pkvm_emergency_reset(void)
{
	struct desc_ptr idt;

	/*
	 * PCI warm reset (0xCF9), which is usually effectively the ACPI reset,
	 * i.e. the modern recommended default.
	 */
	outb(0x06, 0xcf9);
	pkvm_udelay(50000);

	/* Keyboard controller (PS/2) reset */
	outb(0xfe, 0x64);
	pkvm_udelay(50000);

	/* PCI cold reset */
	outb(0x0e, 0xcf9);
	pkvm_udelay(50000);

	/* Triple fault */
	idt.size = 0;
	idt.address = 0;
	asm volatile("lidt %0" : : "m"(idt));
	asm volatile("int3");

	while (1)
		asm volatile("cli; hlt");
}

atomic_t pkvm_panic_in_progress = ATOMIC_INIT(0);

void __noreturn pkvm_panic(const char *fmt, ...)
{
	static char panic_msg[1024];
	va_list args;

	/*
	 * Ensure only one CPU handles the panic and writes to ramoops.
	 * This also signals the VM exit handler to catch and hold all other
	 * CPUs.
	 */
	if (atomic_cmpxchg(&pkvm_panic_in_progress, 0, 1) != 0) {
		while (1)
			asm volatile("cli; hlt");
	}

	/*
	 * Broadcast INIT to all other CPUs (excluding self) via x2APIC ICR
	 * to pull them out of the guest mode. This ensures that the host will not
	 * interfere with panic handling and e.g. will not interfere with
	 * ramoops update.
	 */
	native_x2apic_icr_write(APIC_DEST_ALLBUT | APIC_INT_ASSERT | APIC_DM_INIT, 0);

	/*
	 * Wait for all IPIs to deliver and targets to enter Wait-for-SIPI.
	 * Note: using pkvm_wait_vcpu_kicked_out() is tempting but for reliability,
	 * since we are in the middle of fatal error handling, it is better to
	 * use simple hw delay. This avoids relying on various sw states (like
	 * vcpu->mode) that could be corrupted by the fatal error itself.
	 */
	pkvm_udelay(10000);

	va_start(args, fmt);
	vscnprintf(panic_msg, sizeof(panic_msg), fmt, args);
	va_end(args);

	pkvm_err("%s", panic_msg);

	pkvm_write_ramoops_console(panic_msg);

	pkvm_emergency_reset();
}
