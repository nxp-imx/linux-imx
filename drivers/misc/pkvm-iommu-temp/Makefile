ifneq ($(KERNELRELEASE),)
clean-files := pkvm/hyp.lds pkvm/hyp-reloc.S

obj-$(CONFIG_PKVM_IOMMU_TEMPLATE) := pkvm_iommu_temp.o
ccflags-$(CONFIG_PKVM_IOMMU_TEMPLATE) += -Iarch/arm64/kvm/

pkvm_iommu_temp-y := pkvm-iommu-temp.o pkvm/kvm_nvhe.o

$(obj)/pkvm/kvm_nvhe.o: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/pkvm $(obj)/pkvm/kvm_nvhe.o
else
all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
endif

