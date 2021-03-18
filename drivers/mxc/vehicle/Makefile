#
# Makefile for evs driver
#

ccflags-y := -std=gnu99 -Wno-declaration-after-statement -DPB_SYSTEM_HEADER=\"pb_system.h\" -I$(srctree)/drivers/mxc/vehicle/nanopb-c/

obj-$(CONFIG_VEHICLE_CORE) += \
	nanopb-c/ \
	vehicle_core.o \
	vehicle_protocol_callback.o \
	vehiclehalproto.pb.o

obj-$(CONFIG_VEHICLE_RPMSG_M4) += \
	vehicle_rpmsg_m4.o

obj-$(CONFIG_VEHICLE_DUMMY) += \
	vehicle_dummy_hw.o
