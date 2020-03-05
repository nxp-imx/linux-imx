ANDROID_CROSS_COMPILE:=$(abspath .)/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-
MRVL_ANDROID_SRC_WLAN:= $(LOCAL_PATH)/wlan_src
default:
	$(MAKE) -j -C $(KDIR) M=$(MRVL_ANDROID_SRC_WLAN) ARCH=arm
	CROSS_COMPILE=$(ANDROID_CROSS_COMPILE) SOURCE_DIR=$
	(MRVL_ANDROID_SRC_WLAN) modules
clean:
	$(MAKE) -C $(MRVL_ANDROID_SRC_WLAN) clean
	$(MAKE) -C $(MRVL_ANDROID_SRC_MUART) clean
