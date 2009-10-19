stfm1000-objs	:= stfm1000-core.o stfm1000-i2c.o stfm1000-precalc.o stfm1000-filter.o stfm1000-rds.o

clean-files += stfm1000-precalc.o

obj-$(CONFIG_RADIO_STFM1000) += stfm1000.o
obj-$(CONFIG_RADIO_STFM1000_ALSA) += stfm1000-alsa.o

stfm1000-core.o: $(obj)/stfm1000-precalc.h

hostprogs-$(CONFIG_RADIO_STFM1000) := gen-precalc
$(obj)/stfm1000-precalc.c: $(obj)/gen-precalc $(src)/stfm1000-regs.h
	$(obj)/gen-precalc >$@

EXTRA_CFLAGS += -Idrivers/media/radio
