/*
 * include/linux/fsl_devices.h
 *
 * Definitions for any platform device related flags or structures for
 * Freescale processor devices
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * Copyright 2004-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _FSL_DEVICE_H_
#define _FSL_DEVICE_H_

#include <linux/types.h>
#include <linux/cdev.h>

/*
 * Some conventions on how we handle peripherals on Freescale chips
 *
 * unique device: a platform_device entry in fsl_plat_devs[] plus
 * associated device information in its platform_data structure.
 *
 * A chip is described by a set of unique devices.
 *
 * Each sub-arch has its own master list of unique devices and
 * enumerates them by enum fsl_devices in a sub-arch specific header
 *
 * The platform data structure is broken into two parts.  The
 * first is device specific information that help identify any
 * unique features of a peripheral.  The second is any
 * information that may be defined by the board or how the device
 * is connected externally of the chip.
 *
 * naming conventions:
 * - platform data structures: <driver>_platform_data
 * - platform data device flags: FSL_<driver>_DEV_<FLAG>
 * - platform data board flags: FSL_<driver>_BRD_<FLAG>
 *
 */

enum fsl_usb2_operating_modes {
	FSL_USB2_MPH_HOST,
	FSL_USB2_DR_HOST,
	FSL_USB2_DR_DEVICE,
	FSL_USB2_DR_OTG,
};

/* this used for usb port type */
enum fsl_usb2_modes {
	FSL_USB_DR_HOST,
	FSL_USB_DR_DEVICE,
	FSL_USB_MPH_HOST1,
	FSL_USB_MPH_HOST2,
	FSL_USB_UNKNOWN, /* unkonwn status */
};

enum fsl_usb2_phy_modes {
	FSL_USB2_PHY_NONE,
	FSL_USB2_PHY_ULPI,
	FSL_USB2_PHY_UTMI,
	FSL_USB2_PHY_UTMI_WIDE,
	FSL_USB2_PHY_SERIAL,
	FSL_USB2_PHY_HSIC,
};

enum usb_wakeup_event {
	WAKEUP_EVENT_INVALID,
	WAKEUP_EVENT_VBUS,
	WAKEUP_EVENT_ID,
	WAKEUP_EVENT_DPDM, /* for remote wakeup */
};

struct clk;
struct platform_device;
struct fsl_usb2_wakeup_platform_data;

struct fsl_usb2_platform_data {
	/* board specific information */
	enum fsl_usb2_operating_modes	operating_mode;
	enum fsl_usb2_phy_modes		phy_mode;
	unsigned int			port_enables;
	unsigned int			workaround;

	int		(*init)(struct platform_device *);
	void		(*exit)(struct platform_device *);
	void __iomem	*regs;		/* ioremap'd register base */
	struct clk	*clk;
	unsigned	power_budget;	/* hcd->power_budget */
	unsigned	big_endian_mmio:1;
	unsigned	big_endian_desc:1;
	unsigned	es:1;		/* need USBMODE:ES */
	unsigned	le_setup_buf:1;
	unsigned	have_sysif_regs:1;
	unsigned	invert_drvvbus:1;
	unsigned	invert_pwr_fault:1;

	unsigned	suspended:1;
	unsigned	already_suspended:1;

	/* Freescale private */
	char		*name;
	u32		phy_regs;	/* usb phy register base */
	u32 		xcvr_type;	/* PORTSC_PTS_* */
	char 		*transceiver;	/* transceiver name */
	u32		id_gpio;

	struct fsl_xcvr_ops *xcvr_ops;
	struct fsl_xcvr_power *xcvr_pwr;
	int (*gpio_usb_active) (void);
	void (*gpio_usb_inactive) (void);
	void (*usb_clock_for_pm) (bool);
	void (*platform_suspend)(struct fsl_usb2_platform_data *);
	void (*platform_resume)(struct fsl_usb2_platform_data *);
	void (*wake_up_enable)(struct fsl_usb2_platform_data *, bool);
	void (*phy_lowpower_suspend)(struct fsl_usb2_platform_data *, bool);
	void (*platform_driver_vbus)(bool on); /* for vbus shutdown/open */
	enum usb_wakeup_event (*is_wakeup_event)(struct fsl_usb2_platform_data *);
	void (*wakeup_handler)(struct fsl_usb2_platform_data *);
	void (*hsic_post_ops)(void);
	void (*hsic_device_connected)(void);
	/*
	 * Some platforms, like i.mx6x needs to discharge dp/dm at device mode
	 * or there is wakeup interrupt caused by dp/dm change when the cable
	 * is disconnected with Host.
	 */
	void (*dr_discharge_line) (bool);
	/* only set it when vbus lower very slow during OTG switch */
	bool need_discharge_vbus;
	void (*platform_rh_suspend)(struct fsl_usb2_platform_data *);
	void (*platform_rh_resume)(struct fsl_usb2_platform_data *);
	void (*platform_set_disconnect_det)(struct fsl_usb2_platform_data *, bool);
	void (*platform_phy_power_on)(void);

	struct fsl_usb2_wakeup_platform_data *wakeup_pdata;
	struct platform_device *pdev;
	unsigned	change_ahb_burst:1;
	unsigned	ahb_burst_mode:3;
	unsigned	lowpower:1;
	unsigned	irq_delay:1;
	enum usb_wakeup_event	wakeup_event;
	u32		pmflags;	/* PM from otg or system */
	spinlock_t lock;

	void __iomem *charger_base_addr; /* used for i.mx6 usb charger detect */

	/* register save area for suspend/resume */
	u32		pm_command;
	u32		pm_status;
	u32		pm_intr_enable;
	u32		pm_frame_index;
	u32		pm_segment;
	u32		pm_frame_list;
	u32		pm_async_next;
	u32		pm_configured_flag;
	u32		pm_portsc;
	u32		pm_usbgenctrl;
};

struct fsl_usb2_wakeup_platform_data {
	char *name;
	void (*usb_clock_for_pm) (bool);
	void (*usb_wakeup_exhandle) (void);
	struct fsl_usb2_platform_data *usb_pdata[3];
	/* This waitqueue is used to wait "usb_wakeup thread" to finish
	 * during system resume routine. "usb_wakeup theard" should be finished
	 * prior to usb resume routine.
	 */
	wait_queue_head_t wq;
	/* This flag is used to indicate the "usb_wakeup thread" is finished during
	 * usb wakeup routine.
	 */
	bool usb_wakeup_is_pending;
};

/* Flags in fsl_usb2_mph_platform_data */
#define FSL_USB2_PORT0_ENABLED	0x00000001
#define FSL_USB2_PORT1_ENABLED	0x00000002

#define FLS_USB2_WORKAROUND_ENGCM09152	(1 << 0)

struct mxc_mlb_platform_data {
	struct device *dev;
	u32 buf_addr;
	u32 phy_addr;
	char *reg_nvcc;
	char *mlb_clk;
	char *mlb_pll_clk;
	void (*fps_sel)(int mlbfs);
	struct cdev cdev;
};

struct spi_device;

struct fsl_spi_platform_data {
	u32 	initial_spmode;	/* initial SPMODE value */
	s16	bus_num;
	unsigned int flags;
#define SPI_QE_CPU_MODE		(1 << 0) /* QE CPU ("PIO") mode */
#define SPI_CPM_MODE		(1 << 1) /* CPM/QE ("DMA") mode */
#define SPI_CPM1		(1 << 2) /* SPI unit is in CPM1 block */
#define SPI_CPM2		(1 << 3) /* SPI unit is in CPM2 block */
#define SPI_QE			(1 << 4) /* SPI unit is in QE block */
	/* board specific information */
	u16	max_chipselect;
	void	(*cs_control)(struct spi_device *spi, bool on);
	u32	sysclk;
};

struct fsl_mxc_tve_platform_data {
	char *dac_reg;
	char *dig_reg;
};

struct fsl_mxc_lightsensor_platform_data {
	char *vdd_reg;
	int rext;
};

struct fsl_mxc_ldb_platform_data {
	char *lvds_bg_reg;
	u32 ext_ref;
#define LDB_SPL_DI0	1
#define LDB_SPL_DI1	2
#define LDB_DUL_DI0	3
#define LDB_DUL_DI1	4
#define LDB_SIN0	5
#define LDB_SIN1	6
#define LDB_SEP0	7
#define LDB_SEP1	8
	int mode;
	int ipu_id;
	int disp_id;

	/*only work for separate mode*/
	int sec_ipu_id;
	int sec_disp_id;
};

struct mxc_fb_platform_data {
	struct fb_videomode *mode;
	int num_modes;
	char *mode_str;
	u32 interface_pix_fmt;
};

struct fsl_mxc_lcd_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	void (*reset) (void);
	int (*get_pins) (void);
	void (*put_pins) (void);
	void (*enable_pins) (void);
	void (*disable_pins) (void);
	int default_ifmt;
	int ipu_id;
	int disp_id;
};

struct fsl_mxc_tvout_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	u32 detect_line;
};

struct fsl_mxc_dvi_platform_data {
	void (*init) (void);
	int (*update) (void);
	char *analog_regulator;
	int ipu_id;
	int disp_id;
};

struct fsl_mxc_hdmi_platform_data {
	void (*init) (int, int);
	int (*get_pins) (void);
	void (*put_pins) (void);
	void (*enable_pins) (void);
	void (*disable_pins) (void);
	/* HDMI PHY register config for pass HCT */
	u16 phy_reg_vlev;
	u16 phy_reg_cksymtx;
};

struct fsl_mxc_hdmi_core_platform_data {
	int ipu_id;
	int disp_id;
};

struct fsl_mxc_capture_platform_data {
	int csi;
	int ipu;
	u8 mclk_source;
	u8 is_mipi;
};


struct fsl_mxc_camera_platform_data {
	char *core_regulator;
	char *io_regulator;
	char *analog_regulator;
	char *gpo_regulator;
	u32 mclk;
	u8 mclk_source;
	u32 csi;
	void (*pwdn)(int pwdn);
	void (*io_init)(void);
};

struct fsl_mxc_tvin_platform_data {
	char *dvddio_reg;
	char *dvdd_reg;
	char *avdd_reg;
	char *pvdd_reg;
	void (*pwdn)(int pwdn);
	void (*reset)(void);
	void (*io_init)(void);
	bool cvbs;
	/* adv7280 mipi-csi i2c slave addr */
	u8 csi_tx_addr;
};

struct mpc8xx_pcmcia_ops {
	void(*hw_ctrl)(int slot, int enable);
	int(*voltage_set)(int slot, int vcc, int vpp);
};

struct mxc_pm_platform_data {
	void (*suspend_enter) (void);
	void (*suspend_exit) (void);
};

struct mxc_iim_platform_data {
	const s8        *name;
	u32     virt_base;
	u32     reg_base;
	u32     reg_end;
	u32     reg_size;
	u32     bank_start;
	u32     bank_end;
	u32     irq;
	u32     action;
	struct mutex mutex;
	struct completion completion;
	spinlock_t lock;
	struct clk *clk;
	struct device *dev;
	void   (*enable_fuse)(void);
	void   (*disable_fuse)(void);
};

struct mxc_otp_platform_data {
	char    **fuse_name;
	char    *regulator_name;
	char    *clock_name;
	unsigned int min_volt;
	unsigned int max_volt;
	unsigned int    fuse_num;
};

/*
 * This struct is to define the number of SSIs on a platform,
 * DAM source port config, DAM external port config,
 * regulator names, and other stuff audio needs.
 */
struct mxc_audio_platform_data {
	int ssi_num;
	int src_port;
	int ext_port;

	int intr_id_hp;
	int ext_ram;
	struct clk *ssi_clk[2];

	int hp_gpio;
	int hp_active_low;	/* headphone irq is active low */

	int mic_gpio;
	int mic_active_low;	/* micphone irq is active low */

	int sysclk;
	const char *codec_name;

	int (*init) (void);	/* board specific init */
	int (*amp_enable) (int enable);
	int (*clock_enable) (int enable);
	int (*finit) (void);	/* board specific finit */
	void *priv;		/* used by board specific functions */
};

/* Generic parameters for audio codecs
 */
struct mxc_audio_codec_platform_data {
	int rates; /* codec platform data */
};

struct mxc_pwm_platform_data {
	int pwmo_invert;
	void (*enable_pwm_pad) (void);
	void (*disable_pwm_pad) (void);
};

struct mxc_epit_platform_data {
};
struct mxc_spdif_platform_data {
	int spdif_tx;		/* S/PDIF tx enabled for this board */
	int spdif_rx;		/* S/PDIF rx enabled for this board */
	int spdif_clk_44100;	/* tx clk mux in SPDIF_REG_STC; -1 for none */
	int spdif_clk_48000;	/* tx clk mux in SPDIF_REG_STC; -1 for none */
	int spdif_div_44100;	/* tx clk div in SPDIF_REG_STC */
	int spdif_div_48000;	/* tx clk div in SPDIF_REG_STC */
	int spdif_div_32000;	/* tx clk div in SPDIF_REG_STC */
	int spdif_rx_clk;	/* rx clk mux select in SPDIF_REG_SRPC */
	int (*spdif_clk_set_rate) (struct clk *clk, unsigned long rate);
	struct clk *spdif_clk;
	struct clk *spdif_core_clk;
	struct clk *spdif_audio_clk;
};

struct p1003_ts_platform_data {
	int (*hw_status) (void);
};

/* Returns non-zero if the current suspend operation would
 * lead to a deep sleep (i.e. power removed from the core,
 * instead of just the clock).
 */
#if defined(CONFIG_PPC_83xx) && defined(CONFIG_SUSPEND)
int fsl_deep_sleep(void);
#else
static inline int fsl_deep_sleep(void) { return 0; }
#endif

struct mxs_perfmon_bit_config {
	int reg;
	int field;
	const char *name;
};

struct mxs_platform_perfmon_data {
	struct mxs_perfmon_bit_config *bit_config_tab;
	int bit_config_cnt;
	void (*plt_init) (void);
	void (*plt_exit) (void);
};

#endif /* _FSL_DEVICE_H_ */
