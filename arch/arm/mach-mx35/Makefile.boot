ifeq ($(CONFIG_MACH_MX35EVB), y)
   zreladdr-y	:= 0x90008000
params_phys-y	:= 0x90000100
initrd_phys-y	:= 0x90800000
else
   zreladdr-y	:= 0x80008000
params_phys-y	:= 0x80000100
initrd_phys-y	:= 0x80800000
endif
