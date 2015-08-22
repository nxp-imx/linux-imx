#
# arch/arm/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995-2002 Russell King
#
OBJCOPYFLAGS	:=-O binary -R .comment -S

#   ZRELADDR == virt_to_phys(PAGE_OFFSET + TEXT_OFFSET)
ifdef CONFIG_PHYS_OFFSET
add_hex = $(shell printf 0x%x $$(( $(1) + $(2) )) )
ZRELADDR    := $(call add_hex, $(CONFIG_PHYS_OFFSET), $(TEXT_OFFSET))
endif

PHYS_OFFSET := $(CONFIG_PHYS_OFFSET)
export ZRELADDR PARAMS_PHYS PHYS_OFFSET

targets := Image zImage xipImage bootpImage uImage

ifeq ($(CONFIG_XIP_KERNEL),y)

cmd_deflate_xip_data = $(CONFIG_SHELL) -c '$(src)/deflate_xip_data.sh $< $@'

ifeq ($(CONFIG_XIP_DEFLATED_DATA),y)
quiet_cmd_mkxip = XIPZ    $@
cmd_mkxip = $(cmd_objcopy) && $(cmd_deflate_xip_data)
else
quiet_cmd_mkxip = $(quiet_cmd_objcopy)
cmd_mkxip = $(cmd_objcopy)
endif

$(obj)/xipImage: vmlinux FORCE
	$(call if_changed,mkxip)
	@$(kecho) '  Physical Address of xipImage: $(CONFIG_XIP_PHYS_ADDR)'

$(obj)/Image $(obj)/zImage: FORCE
	@echo 'Kernel configured for XIP (CONFIG_XIP_KERNEL=y)'
	@echo 'Only the xipImage target is available in this case'
	@false

else

$(obj)/xipImage: FORCE
	@echo 'Kernel not configured for XIP (CONFIG_XIP_KERNEL!=y)'
	@false

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/compressed/vmlinux: $(obj)/Image FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

ifeq ($(CONFIG_KERNEL_COMPRESS_NONE),y)
$(obj)/zImage: FORCE
	@$(kecho) 'Kernel configured for no compression (CONFIG_COMPRESS_NONE=y)'
	@false
else
$(obj)/zImage:	$(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
endif

endif

ifneq ($(LOADADDR),)
  UIMAGE_LOADADDR=$(LOADADDR)
else
  ifeq ($(CONFIG_ZBOOT_ROM),y)
    UIMAGE_LOADADDR=$(CONFIG_ZBOOT_ROM_TEXT)
  else
    UIMAGE_LOADADDR=$(ZRELADDR)
  endif
endif

ifeq ($(UIMAGE_LOADADDR),)
  UIMAGE_LOADADDR=$(shell /bin/bash -c 'printf "0x%08x" \
					$$[$(CONFIG_DRAM_BASE) + 0x008000]')
endif

ifeq ($(UIMAGE_ENTRYADDR),)
  ifeq ($(CONFIG_THUMB2_KERNEL),y)
    # Set bit 0 to 1 so that "mov pc, rx" switches to Thumb-2 mode
    UIMAGE_ENTRYADDR?=$(shell echo $(UIMAGE_LOADADDR) | sed -e "s/.$$/1/")
  else
    UIMAGE_ENTRYADDR?=$(UIMAGE_LOADADDR)
  endif
endif

check_for_multiple_loadaddr = \
if [ $(words $(UIMAGE_LOADADDR)) -ne 1 ]; then \
	echo 'multiple (or no) load addresses: $(UIMAGE_LOADADDR)'; \
	echo 'This is incompatible with uImages'; \
	echo 'Specify LOADADDR on the commandline to build an uImage'; \
	false; \
fi

ifeq ($(CONFIG_KERNEL_COMPRESS_NONE),y)
$(obj)/uImage:	$(obj)/Image FORCE
	$(call if_changed,uimage)
else
$(obj)/uImage:	$(obj)/zImage FORCE
	@$(check_for_multiple_loadaddr)
	$(call if_changed,uimage)
endif

$(obj)/bootp/bootp: $(obj)/zImage FORCE
	$(Q)$(MAKE) $(build)=$(obj)/bootp $@

$(obj)/bootpImage: $(obj)/bootp/bootp FORCE
	$(call if_changed,objcopy)

subdir-	    := bootp compressed dts
