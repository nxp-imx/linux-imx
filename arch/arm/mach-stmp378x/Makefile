#
# Makefile for the linux kernel.
#
obj-y += power.o persistent.o

# Power Management
obj-$(CONFIG_PM) += pm.o sleep.o

# Chip family select
obj-$(CONFIG_ARCH_STMP37XX) += stmp37xx.o stmp37xx_lcdif.o
obj-$(CONFIG_ARCH_STMP378X) += stmp378x.o stmp378x_lcdif.o

obj-$(CONFIG_MACH_STMP37XX) += stmp37xx_devb.o
obj-$(CONFIG_MACH_STMP378X) += stmp378x_devb.o stmp378x_i2c.o stmp378x_pwm_led.o

# LCD panels support
obj-$(CONFIG_FB_STMP37XX_HX8238A)	+= lcd_hx8238a.o
obj-$(CONFIG_FB_STMP37XX_LMS350)	+= lcd_lms350.o
obj-$(CONFIG_FB_STMP37XX_LMS430)	+= lcd_lms430.o

# TVOUT support
obj-$(CONFIG_FB_STMP378X_TVENC)	+= tvenc.o

obj-$(CONFIG_STMP3XXX_UNIQUE_ID_OTP)	+= otp.o
obj-$(CONFIG_STMP3XXX_UNIQUE_ID_FILE)	+= unique-id-file.o

obj-$(CONFIG_INPUT_STMP3XXX_ROTDEC)	+= stmp378x_devb_rotdec.o
obj-$(CONFIG_STMP378X_RAM_FREQ_SCALING)	+= emi.o

