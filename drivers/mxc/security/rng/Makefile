# Makefile for the Linux RNG Driver
#
# This makefile works within a kernel driver tree

 # Makefile for rng_driver


# Possible configurable paramters
CFG_RNG += -DRNGA_MAX_REQUEST_SIZE=32

#DBG_RNGA = -DRNGA_DEBUG
#DBG_RNGA += -DRNGA_REGISTER_DEBUG
#DBG_RNGA += -DRNGA_ENTROPY_DEBUG

EXTRA_CFLAGS = -DLINUX_KERNEL $(CFG_RNG) $(DBG_RNG)


ifeq ($(CONFIG_MXC_RNG_TEST_DRIVER),y)
EXTRA_CFLAGS += -DRNG_REGISTER_PEEK_POKE
endif
ifeq ($(CONFIG_RNG_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif


EXTRA_CFLAGS += -Idrivers/mxc/security/rng/include -Idrivers/mxc/security/sahara2/include

obj-$(CONFIG_MXC_SECURITY_RNG)	+= shw.o
#shw-objs := shw_driver.o shw_memory_mapper.o ../sahara2/fsl_shw_keystore.o
shw-objs := shw_driver.o shw_memory_mapper.o ../sahara2/fsl_shw_keystore.o  \
            fsl_shw_sym.o  fsl_shw_wrap.o shw_dryice.o  des_key.o           \
            shw_hash.o  shw_hmac.o
            
obj-$(CONFIG_MXC_SECURITY_RNG)	+= rng.o
rng-objs :=	rng_driver.o
