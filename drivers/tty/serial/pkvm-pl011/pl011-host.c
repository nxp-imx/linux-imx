#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/kvm_pkvm_module.h>

int __kvm_nvhe_pl011_hyp_init(const struct pkvm_module_ops *ops);

static int __init pl011_nvhe_init(void)
{
	return pkvm_load_el2_module(__kvm_nvhe_pl011_hyp_init);
}
module_init(pl011_nvhe_init);

MODULE_LICENSE("GPL");
