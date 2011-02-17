obj-$(CONFIG_FB_MXC_TVOUT_TVE)             	+= tve.o
obj-$(CONFIG_FB_MXC_SII902X)                    += mxcfb_sii902x.o
obj-$(CONFIG_FB_MXC_LDB)			+= ldb.o
obj-$(CONFIG_FB_MODE_HELPERS)			+= mxc_edid.o
ifeq ($(CONFIG_ARCH_MX21)$(CONFIG_ARCH_MX27)$(CONFIG_ARCH_MX25),y)
	obj-$(CONFIG_FB_MXC_TVOUT)              += fs453.o
	obj-$(CONFIG_FB_MXC_SYNC_PANEL)         += mx2fb.o mxcfb_modedb.o
	obj-$(CONFIG_FB_MXC_EPSON_PANEL)        += mx2fb_epson.o
else
ifeq ($(CONFIG_MXC_IPU_V1),y)
	obj-$(CONFIG_FB_MXC_SYNC_PANEL)         += mxcfb.o mxcfb_modedb.o
else
	obj-$(CONFIG_FB_MXC_SYNC_PANEL)         += mxc_ipuv3_fb.o
endif
	obj-$(CONFIG_FB_MXC_EPSON_PANEL)        += mxcfb_epson.o
	obj-$(CONFIG_FB_MXC_EPSON_QVGA_PANEL)   += mxcfb_epson_qvga.o
	obj-$(CONFIG_FB_MXC_TOSHIBA_QVGA_PANEL) += mxcfb_toshiba_qvga.o
	obj-$(CONFIG_FB_MXC_SHARP_128_PANEL)    += mxcfb_sharp_128x128.o
endif
obj-$(CONFIG_FB_MXC_EPSON_VGA_SYNC_PANEL)   += mxcfb_epson_vga.o
obj-$(CONFIG_FB_MXC_CLAA_WVGA_SYNC_PANEL)   += mxcfb_claa_wvga.o
obj-$(CONFIG_FB_MXC_SEIKO_WVGA_SYNC_PANEL)   += mxcfb_seiko_wvga.o
obj-$(CONFIG_FB_MXC_TVOUT_CH7024)           += ch7024.o
obj-$(CONFIG_FB_MXC_CH7026)		    		+= mxcfb_ch7026.o
obj-$(CONFIG_FB_MXC_EINK_PANEL)             += mxc_epdc_fb.o
obj-$(CONFIG_FB_MXC_ELCDIF_FB)		    += mxc_elcdif_fb.o
