# SPDX-License-Identifier: GPL-2.0-or-later

CFLAGS += -Wall -O2 -g $(KHDR_INCLUDES) -D_FILE_OFFSET_BITS=64 -I../.. -I../../../../.. -I../../../../include

TEST_GEN_PROGS := fusectl_test fuse_test fuse_daemon
TEST_GEN_FILES := \
	test_bpf.bpf \
	fd_bpf.bpf \
	fd.sh \
	fuse_mnt

LDLIBS := -lpthread -lelf

EXTRA_CLEAN := *.bpf
BPF_FLAGS = -Wall -Werror -O2 -g -emit-llvm \
	    -I ../../../../../include \
	    -idirafter /usr/lib/gcc/x86_64-linux-gnu/10/include \
	    -idirafter /usr/local/include \
	    -idirafter /usr/include/x86_64-linux-gnu \
	    -idirafter /usr/include \

include ../../lib.mk

VAR_CFLAGS := $(shell pkg-config fuse --cflags 2>/dev/null)
ifeq ($(VAR_CFLAGS),)
VAR_CFLAGS := -D_FILE_OFFSET_BITS=64 -I/usr/include/fuse
endif

VAR_LDLIBS := $(shell pkg-config fuse --libs 2>/dev/null)
ifeq ($(VAR_LDLIBS),)
VAR_LDLIBS := -lfuse -pthread
endif

$(OUTPUT)/fuse_mnt: CFLAGS += $(VAR_CFLAGS)
$(OUTPUT)/fuse_mnt: LDLIBS += $(VAR_LDLIBS)

EXTRA_SOURCES := bpf_loader.c
$(TEST_GEN_PROGS) : $(EXTRA_SOURCES)

$(OUTPUT)/%.ir: %.c
	clang $(BPF_FLAGS) -c $< -o $@

$(OUTPUT)/%.bpf: $(OUTPUT)/%.ir
	llc -march=bpf -filetype=obj -o $@ $<

$(OUTPUT)/fd.sh: fd.txt
	cp $< $@
	chmod 755 $@
