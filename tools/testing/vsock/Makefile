# SPDX-License-Identifier: GPL-2.0-only
all: test vsock_perf
test: vsock_test vsock_diag_test
vsock_test: vsock_test.o timeout.o control.o util.o
vsock_diag_test: vsock_diag_test.o timeout.o control.o util.o
vsock_perf: vsock_perf.o

CFLAGS += -g -O2 -Werror -Wall -I. -I../../include -I../../../usr/include -Wno-pointer-sign -fno-strict-overflow -fno-strict-aliasing -fno-common -MMD -U_FORTIFY_SOURCE -D_GNU_SOURCE
.PHONY: all test clean
clean:
	${RM} *.o *.d vsock_test vsock_diag_test vsock_perf
-include *.d

VSOCK_INSTALL_PATH ?=

install: all
ifdef VSOCK_INSTALL_PATH
	mkdir -p $(VSOCK_INSTALL_PATH)
	install -m 744 vsock_test $(VSOCK_INSTALL_PATH)
	install -m 744 vsock_perf $(VSOCK_INSTALL_PATH)
	install -m 744 vsock_diag_test $(VSOCK_INSTALL_PATH)
else
	$(error Error: set VSOCK_INSTALL_PATH to use install)
endif
