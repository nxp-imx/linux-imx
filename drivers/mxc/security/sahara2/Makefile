# Makefile for the Linux Sahara2 driver
#
# This makefile works within a kernel driver tree

# Need to augment this to support optionally building user-mode support
API_SOURCES = fsl_shw_sym.c  fsl_shw_user.c  fsl_shw_hash.c  fsl_shw_auth.c \
              fsl_shw_hmac.c  fsl_shw_rand.c  sf_util.c  km_adaptor.c  fsl_shw_keystore.c  \
              fsl_shw_wrap.c \


SOURCES = sah_driver_interface.c  sah_hardware_interface.c            \
          sah_interrupt_handler.c  sah_queue.c  sah_queue_manager.c   \
          sah_status_manager.c  sah_memory_mapper.c


# Turn on for mostly full debugging
# DIAGS = -DDIAG_DRV_STATUS -DDIAG_DRV_QUEUE -DDIAG_DRV_INTERRUPT -DDIAG_DRV_IF
# DIAGS += -DDIAG_DURING_INTERRUPT

# Turn on for lint-type checking
#EXTRA_CFLAGS = -Wall -W -Wstrict-prototypes -Wmissing-prototypes
EXTRA_CFLAGS += -DLINUX_KERNEL $(DIAGS)


ifeq ($(CONFIG_MXC_SAHARA_POLL_MODE),y)
EXTRA_CFLAGS += -DSAHARA_POLL_MODE
EXTRA_CFLAGS += -DSAHARA_POLL_MODE_TIMEOUT=$(CONFIG_SAHARA_POLL_MODE_TIMEOUT)
endif

ifeq ($(CONFIG_MXC_SAHARA_USER_MODE),y)
EXTRA_CFLAGS += -DSAHARA_USER_MODE
SOURCES +=
endif

ifeq ($(CONFIG_PM),y)
EXTRA_CFLAGS += -DSAHARA_POWER_MANAGMENT
endif

EXTRA_CFLAGS += -Idrivers/mxc/security/sahara2/include

# handle buggy BSP -- uncomment if these are undefined during build
#EXTRA_CFLAGS += -DSAHARA_BASE_ADDR=HAC_BASE_ADDR -DINT_SAHARA=INT_HAC_RTIC


obj-$(CONFIG_MXC_SAHARA) += sahara.o

sahara-objs := $(SOURCES:.c=.o) $(API_SOURCES:.c=.o)
