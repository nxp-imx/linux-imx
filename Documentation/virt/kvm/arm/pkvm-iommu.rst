.. SPDX-License-Identifier: GPL-2.0

==================
pKVM DMA Isolation
==================

:Author: Android KVM <android-kvm@google.com>

Introduction
============

pKVM (see pkvm.rst) supports running virtual machines that are not accessible by
the host kernel. This is enforced by the hypervisor running in EL2, which
configures the CPU stage-2 MMU to unmap these VMs.

However, DMA-capable devices access memory without going through the CPU
stage-2 MMU. This allows them to access hypervisor or VM memory, defeating
hypervisor isolation.

An IOMMU (Input-Output Memory Management Unit) can be used to restrict a
device's view of memory, thereby protecting the hypervisor and VMs.

In this document, "IOMMU" refers to the IP block managing device DMA, which
can be translating or non-translating. We use the term MPU (Memory Protection
Unit) to refer specifically to a non-translating IOMMU.

This document covers only the host kernel IOMMU management; guest VM
management is out of scope.

HW Requirements
===============

To support pKVM, IOMMUs MUST fulfill the following HW requirements:

1. An IOMMU MUST be present for each DMA-capable device. The IOMMU must enforce
   memory access permissions and can be non-translating.

2. IOMMUs SHOULD operate at a granularity equal to the CPU page size.

3. The IOMMU MUST cover the entire input address range, and the output range
   must cover all physical memory.

4. If the entire memory range is not covered, all accesses to uncovered ranges
   MUST be denied.

5. IOMMU MMIO regions MUST be protected against DMA access.

6. When invalidating hardware structures, the IOMMU MUST provide software with
   a mechanism to wait for the completion of all DMA transactions processed
   using the old contents of those structures.

FW Requirements
===============

The IOMMUs MUST reset to a state where DMA is blocked. This can be achieved by
saving/restoring specific MMIO registers (e.g., GBPA on SMMUv3). See the
"SMMUv3" section for more details.

Hypervisor IOMMU Drivers
========================

To manage IOMMUs from the hypervisor, a driver must run in EL2.

Driver Init
-----------

A kernel driver must register the hypervisor driver with KVM at boot before
the kernel deprivileges. For built-in drivers, this can be done via
``core_initcall()``. For modules, this must be done from ``module_init()``,
and the module must be loaded via the kernel command line:
``kvm-arm.protected_modules=module_name``.

To register a driver with KVM, the kernel driver calls:

  ``kvm_iommu_register_driver(struct kvm_iommu_driver *kernel_ops, size_t nr_pages)``

* ``kernel_ops``: A pointer to ``kvm_iommu_driver``, which includes various
    operations needed by KVM to interact with the kernel (EL1) part of the
    driver.
* ``nr_pages``: Explained in the "Memory Allocation" section below.

Later, when the hypervisor is ready, the ``init_driver`` callback is invoked.
This callback performs any probe/init tasks required while the kernel is still
trusted (as global data is shared with the hypervisor).

At the end of this callback, the driver must call:

  ``kvm_iommu_init_hyp(struct kvm_iommu_ops* hyp_ops, pkvm_handle_t *drv_id)``

* ``hyp_ops``: A pointer to the structure in the hypervisor driver containing
    the main callbacks required for the runtime to interact with the driver.
* ``drv_id``: The returned driver ID. It is possible to register multiple
    drivers in the hypervisor. This ID is used in subsequent hypercalls
    (e.g., ``alloc_domain``, ``set_identity``).

Memory Allocation
-----------------

The hypervisor manages two pools of memory that drivers can use:

1. **Atomic Pool:**
   Allocated early at boot from the pKVM carveout. Memory is allocated and
   freed using ``kvm_iommu_donate_pages_atomic()`` and
   ``kvm_iommu_reclaim_pages_atomic()`` from the hypervisor driver.

   The size of this carveout is determined by:
   * ``CONFIG_IOMMU_POOL_PAGES``: The default value.
   * ``kvm-arm.hyp_iommu_pages``: Kernel command line override.

   At driver init, ``nr_pages`` is passed from the driver after probing the HW.
   If this request is larger than the pre-allocated hypervisor pool, initialization
   will fail.

   This pool is typically used in contexts that cannot fail memory allocations
   (ATOMIC).

2. **On-demand Pool:**
   Allocated on demand. The hypervisor driver can use ``kvm_iommu_donate_pages()``
   and ``kvm_iommu_reclaim_page()`` to allocate from this pool. These MUST be
   called only from the context of IOMMU HVCs.

   When ``kvm_iommu_donate_pages()`` fails to allocate, it automatically encodes
   a request returned via the HVC (see ``hyp_reqs_smccc_encode``). The caller
   can check the return value, top up the allocator using
   ``__pkvm_topup_hyp_alloc_mgt_gfp()``, and repeat the HVC.

   All HVCs have wrapper functions in ``arch/arm64/kvm/iommu.c`` which handle
   return values and top up the allocator if needed. This allows transparent
   usage when using KVM IOMMU provided abstractions.

   This pool is typically used for HW-related allocations (e.g., page table pages).

3. **Heap:**
   For small allocations (a few bytes), the hypervisor supports a heap accessed
   via ``hyp_alloc()`` and ``hyp_free()``. This pool is also on-demand; it is
   topped up similarly to case #2 and automatically checked via HVC wrappers.

   This is typically used for data structure allocations.

Runtime
-------

There are two main design patterns supported. pKVM provides example drivers
for both implementations (see the "SMMUv3" section).

**1) Separate Page Tables**

If the HW supports two stages of translation (e.g., SMMUv3), stage-2 can be
managed by the hypervisor (similar to the CPU). A hypervisor IOMMU driver
creates a shadow page table of the CPU stage-2, configured for all devices.

This is achieved as follows:

* During driver init in the hypervisor, ``kvm_iommu_snapshot_host_stage2()``
    is called to shadow the CPU stage-2 page table in the IOMMU.
* At runtime, ``host_stage2_idmap`` is called from ``hyp_ops`` each time
    the page state changes. The driver is expected to update the page table to
    match the CPU.

For IPs like SMMUv3, managing stage-2 typically involves using trap-and-emulate
to configure the stage-2 page table.

To implement trap-and-emulate for MMIO, the region can be unmapped from the
host kernel using ``___pkvm_host_donate_hyp()``. The driver then handles page
faults using the ``dabt_handler`` hyp op.

It is possible to use IPs other than SMMUv3. Simpler MPUs might have a
programming interface that can be managed entirely by the hypervisor without
requiring trap-and-emulate.

Power Management
----------------

Due to the lack of deployed standards for IOMMU power management, the
hypervisor cannot enforce a standard method.

However, the hypervisor provides the concept of ``power_domains``, with one
implementation based on HVCs.

An IOMMU can register using:
``pkvm_init_power_domain(power_domain, ops)``

* ``power_domain``: Contains the power domain configuration.
* ``ops``: The on/off callbacks.

For the HVC power domain, only the device ID is required.

From the kernel driver—ideally within runtime PM callbacks—calls are made to
``pkvm_iommu_suspend(device_id)`` and ``pkvm_iommu_resume(device_id)``,
which ultimately invoke the ops implemented by the hypervisor driver.

``CONFIG_ARM_SMMU_V3_PKVM_PV`` implements this interface and can be used as
an example.

**IMPORTANT:** Delegating power management to the untrusted kernel may put
the system at risk. To safely use HVC power management, devices **MUST**
reset to a blocking state as mentioned in "FW Requirements".

**2) Para-virtual IOMMU**

If the IOMMU does not support dual-stage translation or lacks a separate MPU
for the hypervisor, the hypervisor must manage the single stage of the IOMMU
and provide a para-virtual interface to the kernel.

This is also useful for DMA performance, as a single stage is used instead of
two. However, it adds overhead to IOMMU map/unmap calls.

To use the para-virtual interface, the driver must implement the following
``hyp_ops``:

* ``alloc_domain`` / ``free_domain``: Similar to the kernel; allocates a free translation regime.
* ``attach_dev`` / ``detach_dev``: Adds/Removes a device to/from a domain.
* ``map_pages`` / ``unmap_pages``: Maps/Unmaps pages in a domain.

The kernel driver mostly forwards these ops to the hypervisor; wrappers exist
in ``kvm/iommu.c``.

**VERY IMPORTANT**
When drivers use a para-virtual interface to map addresses (memory or MMIO)
in the IOMMU, that memory becomes accessible by DMA. This must be tracked by
the hypervisor to prevent donating the memory to VMs or itself.

* **Before** the driver maps memory in the IOMMU, it **MUST** call
    ``__pkvm_host_use_dma()`` to track this memory in the hypervisor.
* **After** the driver unmaps an address from the IOMMU **and** invalidates
    the TLB entries, it **MUST** call ``__pkvm_host_unuse_dma()``.

Refer to ``CONFIG_ARM_SMMU_V3_PKVM_PV`` for an example.

SMMUv3
======

We provide implementations for both designs mentioned above.

Please note that **neither** provided driver performs save/restore of the
SMMU state across power on/off events. It is assumed that the HW/FW handles
this. Otherwise, the SMMU must at least reset to a blocking state, and the
vendor must implement SW state restoration.

The drivers also assume that caches are clean upon power on.

Trap & Emulate Dual Translation
-------------------------------

Enabled using ``CONFIG_ARM_SMMU_V3_PKVM``.
This config also requires the main kernel driver ``CONFIG_ARM_SMMU_V3`` to
be enabled.

Para-virtual Single-Stage Translation
-------------------------------------

Enabled using ``CONFIG_ARM_SMMU_V3_PKVM_PV``.

**Note:** Only **one** of these drivers must be enabled at a time.

To use the dual-translation driver or the identity domain in the PV driver,
the kernel command line ``kvm-arm.hyp_iommu_pages`` must be set to allow
the driver to populate the identity stage-2. The value is typically close
to the return value of ``host_s2_pgtable_pages()`` on your system. Some
extra pages might be required if your SMMU supports other dynamically
allocated HW structures (e.g., L2 STE).

Other IOMMUs
============

A template empty driver exists that you can use to start a new driver,
``CONFIG_PKVM_IOMMU_TEMPLATE```

Based on the IOMMU design, you can check one of the SMMUv3 provided driver
implementation for any unclear points.

FAQ
===

Common problems:

1. **Hypervisor driver causes system crash (e.g., SERROR) while accessing MMIO.**

   This usually means the hypervisor driver believes the device is on while
   it is not. Please check the "Power Management" section to ensure proper
   integration between the kernel and the hypervisor.

2. **DMA fails because the IOMMU is disabled.**

   This may be a power management issue where the hypervisor believes the
   kernel has powered off the device while it is still on. Please check the
   "Power Management" section to ensure proper integration between the
   kernel and the hypervisor.

3. **SMMUv3 driver is missing some features.**

   We attempt to maintain feature parity with the upstream driver, excluding
   features not commonly used in Android (e.g., ACPI, IOPF).

   Vendors typically fork the driver to add extra logic. Please feel free to
   reach out with any specific requests.