#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y   := cpu.o mm.o system.o devices.o dummy_gpio.o irq.o bus_freq.o usb_dr.o usb_h1.o usb_h2.o usb_h3.o pm.o cpu_op-mx6.o mx6_wfi.o mx6_anatop_regulator.o

obj-$(CONFIG_ARCH_MX6) += clock.o mx6q_suspend.o
obj-$(CONFIG_MACH_MX6Q_ARM2) += board-mx6q_arm2.o
obj-$(CONFIG_MACH_MX6Q_SABRELITE) += board-mx6q_sabrelite.o
obj-$(CONFIG_SMP) += plat_hotplug.o platsmp.o headsmp.o
obj-$(CONFIG_LOCAL_TIMERS) += localtimer.o
