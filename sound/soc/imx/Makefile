# i.MX Platform Support
snd-soc-imx-objs := imx-pcm.o
snd-soc-imx-ssi-objs := imx-ssi.o
snd-soc-imx-esai-objs := imx-esai.o

ifdef CONFIG_MACH_MX27
snd-soc-imx-objs += imx-pcm-dma-mx2.o
endif

# i.MX Machine Support
snd-soc-phycore-ac97-objs := phycore-ac97.o
snd-soc-wm1133-ev1-objs := wm1133-ev1.o

obj-$(CONFIG_SND_SOC_PHYCORE_AC97) += snd-soc-phycore-ac97.o
obj-$(CONFIG_SND_MXC_SOC_WM1133_EV1) += snd-soc-wm1133-ev1.o

obj-$(CONFIG_SND_MXC_SOC) += snd-soc-imx.o
obj-$(CONFIG_SND_MXC_SOC_SSI) += snd-soc-imx-ssi.o
obj-$(CONFIG_SND_MXC_SOC_ESAI) += snd-soc-imx-esai.o
obj-$(CONFIG_SND_MXC_SOC_AC97) += snd-soc-imx-ac97.o

# i.MX Machine Support
snd-soc-imx-3stack-wm8350-objs := imx-3stack-wm8350.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_WM8350) += snd-soc-imx-3stack-wm8350.o
snd-soc-imx-3stack-sgtl5000-objs := imx-3stack-sgtl5000.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_SGTL5000) += snd-soc-imx-3stack-sgtl5000.o
snd-soc-imx-3stack-ak4647-objs := imx-3stack-ak4647.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_AK4647) += snd-soc-imx-3stack-ak4647.o
snd-soc-imx-3stack-wm8580-objs := imx-3stack-wm8580.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_WM8580) += snd-soc-imx-3stack-wm8580.o
snd-soc-imx-3stack-ak5702-objs := imx-3stack-ak5702.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_AK5702) += snd-soc-imx-3stack-ak5702.o
snd-soc-imx-3stack-bt-objs := imx-3stack-bt.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_BLUETOOTH) += snd-soc-imx-3stack-bt.o
snd-soc-imx-3stack-cs42888-objs := imx-3stack-cs42888.o
obj-$(CONFIG_SND_SOC_IMX_3STACK_CS42888) += snd-soc-imx-3stack-cs42888.o

