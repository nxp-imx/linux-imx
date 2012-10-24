# i.MX Platform Support
snd-soc-imx-objs := imx-ssi.o imx-esai.o imx-hdmi-dai.o hdmi_pcm.o
snd-soc-imx-fiq-objs := imx-pcm-fiq.o
snd-soc-imx-mx2-objs := imx-pcm-dma-mx2.o imx-hdmi-dma.o
snd-soc-imx-spdif-dai-objs := imx-spdif-dai.o

obj-$(CONFIG_SND_IMX_SOC) += snd-soc-imx.o
obj-$(CONFIG_SND_MXC_SOC_FIQ) += snd-soc-imx-fiq.o
obj-$(CONFIG_SND_MXC_SOC_MX2) += snd-soc-imx-mx2.o
obj-$(CONFIG_SND_MXC_SOC_SPDIF_DAI) += snd-soc-imx-spdif-dai.o

# i.MX Machine Support
snd-soc-eukrea-tlv320-objs := eukrea-tlv320.o
snd-soc-phycore-ac97-objs := phycore-ac97.o
snd-soc-mx27vis-aic32x4-objs := mx27vis-aic32x4.o
snd-soc-wm1133-ev1-objs := wm1133-ev1.o
snd-soc-imx-wm8958-objs := imx-wm8958.o
snd-soc-imx-wm8962-objs := imx-wm8962.o
snd-soc-imx-sgtl5000-objs := imx-sgtl5000.o
snd-soc-imx-cs42888-objs := imx-cs42888.o
snd-soc-imx-spdif-objs := imx-spdif.o
snd-soc-imx-si4763-objs := imx-si4763.o
snd-soc-imx-hdmi-objs := imx-hdmi.o

obj-$(CONFIG_SND_SOC_EUKREA_TLV320) += snd-soc-eukrea-tlv320.o
obj-$(CONFIG_SND_SOC_PHYCORE_AC97) += snd-soc-phycore-ac97.o
obj-$(CONFIG_SND_SOC_MX27VIS_AIC32X4) += snd-soc-mx27vis-aic32x4.o
obj-$(CONFIG_SND_MXC_SOC_WM1133_EV1) += snd-soc-wm1133-ev1.o
obj-$(CONFIG_SND_SOC_IMX_WM8958) += snd-soc-imx-wm8958.o
obj-$(CONFIG_SND_SOC_IMX_WM8962) += snd-soc-imx-wm8962.o
obj-$(CONFIG_SND_SOC_IMX_SGTL5000) += snd-soc-imx-sgtl5000.o
obj-$(CONFIG_SND_SOC_IMX_CS42888) += snd-soc-imx-cs42888.o
obj-$(CONFIG_SND_SOC_IMX_SPDIF) += snd-soc-imx-spdif.o
obj-$(CONFIG_SND_SOC_IMX_HDMI) += snd-soc-imx-hdmi.o
obj-$(CONFIG_SND_SOC_IMX_SI4763) += snd-soc-imx-si4763.o

AFLAGS_hdmi_pcm.o := -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=softfp
