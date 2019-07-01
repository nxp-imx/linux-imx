# define_trace.h needs to know how to find our header
CFLAGS_trace.o			:= -I$(src)

obj-$(CONFIG_USB_CDNS3)		+= cdns3.o

cdns3-y				:= core.o
cdns3-$(CONFIG_USB_CDNS3_GADGET)	+= gadget.o ep0.o trace.o
cdns3-$(CONFIG_USB_CDNS3_HOST)	+= host.o
