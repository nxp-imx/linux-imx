#
# Makefile for I.API sources.
#
ifneq ($(KBUILD_SRC),)
ccflags-y += -I$(KBUILD_SRC)/arch/arm/plat-mxc/sdma/iapi/include \
                -I$(KBUILD_SRC)/include/linux \
                -DMCU -DOS=LINUX \
                -DL_I_T_T_L_E_ENDIAN=0 -DB_I_G_ENDIAN=1 \
                -DENDIANNESS=L_I_T_T_L_E_ENDIAN
else
ccflags-y += -Iarch/arm/plat-mxc/sdma/iapi/include \
                -Iinclude/linux \
                -DMCU -DOS=LINUX \
                -DL_I_T_T_L_E_ENDIAN=0 -DB_I_G_ENDIAN=1 \
                -DENDIANNESS=L_I_T_T_L_E_ENDIAN
endif

obj-y	+= iapiLow.o iapiLowMcu.o iapiMiddle.o iapiMiddleMcu.o iapiHigh.o iapiDefaults.o iapiOS.o
