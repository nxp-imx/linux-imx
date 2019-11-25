##
## Makefile for the VPU and M0 driver
##

EXTRA_CFLAGS += $(DEFINES)

obj-$(CONFIG_MXC_VPU_WINDSOR) = vpu-windsor.o
vpu-windsor-objs = vpu_encoder_b0.o \
	vpu_encoder_ctrl.o \
	vpu_event_msg.o \
	vpu_encoder_mem.o \
	vpu_encoder_rpc.o \
	vpu_encoder_mu.o \
	vpu_encoder_pm.o

clean:
	rm -rf $(vpu-windsor-objs)
