# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(src)	# needed for trace events
ccflags-y += -Wmissing-prototypes -Wmissing-declarations

obj-$(CONFIG_ANDROID_BINDER_IPC_RUST) += rust_binder.o
rust_binder-y := \
	rust_binder_main.o	\
	rust_binderfs.o		\
	rust_binder_events.o	\
	page_range_helper.o

# Hooks needs to be built-in always so that DDK modules with vendor hooks do
# not form a hard dependency on rust_binder.ko
ifdef CONFIG_ANDROID_BINDER_IPC_RUST
obj-y += rust_binder_hooks.o
endif
