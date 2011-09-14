#
# Makefile for the linux kernel.
#

# Object file lists.
obj-y   := cpu.o mm.o system.o devices.o dummy_gpio.o irq.o bus_freq.o usb_dr.o usb_h1.o pm.o cpu_op-mx6.o

obj-$(CONFIG_ARCH_MX6) += clock.o mx6q_suspend.o
obj-$(CONFIG_MACH_MX6Q_SABREAUTO) += board-mx6q_sabreauto.o
obj-$(CONFIG_SMP) += plat_hotplug.o platsmp.o headsmp.o localtimer.o
