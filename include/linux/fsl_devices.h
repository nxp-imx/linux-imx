/*
 * include/linux/fsl_devices.h
 *
 * Definitions for any platform device related flags or structures for
 * Freescale processor devices
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _FSL_DEVICE_H_
#define _FSL_DEVICE_H_

#include <linux/types.h>

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
};

struct fsl_usb2_wakeup_platform_data;
struct platform_device;
struct fsl_usb2_platform_data {
	/* board specific information */
	enum fsl_usb2_operating_modes	operating_mode;
	enum fsl_usb2_phy_modes		phy_mode;
	unsigned int			port_enables;
	char *name;		/* pretty print */
	int (*platform_init) (struct platform_device *);
	void (*platform_uninit) (struct fsl_usb2_platform_data *);
	void __iomem *regs;	/* ioremap'd register base */
	u32 phy_regs;	/* usb phy register base */
	u32 xcvr_type;		/* PORTSC_PTS_* */
	char *transceiver;	/* transceiver name */
	unsigned power_budget;	/* for hcd->power_budget */
	struct platform_device *pdev;
	struct fsl_xcvr_ops *xcvr_ops;
	struct fsl_xcvr_power *xcvr_pwr;
	int (*gpio_usb_active) (void);
	void (*gpio_usb_inactive) (void);
	void (*usb_clock_for_pm) (bool);
	void (*platform_suspend)(struct fsl_usb2_platform_data *);
	void (*platform_resume)(struct fsl_usb2_platform_data *);
	void (*wake_up_enable)(struct fsl_usb2_platform_data *pdata, bool on);
	void (*phy_lowpower_suspend)(struct fsl_usb2_platform_data *pdata, bool on);
	void (*platform_driver_vbus)(bool on); /* platform special function for vbus shutdown/open */
	bool (*is_wakeup_event)(struct fsl_usb2_platform_data *pdata);
	unsigned			big_endian_mmio:1;
	unsigned			big_endian_desc:1;
	unsigned			es:1;	/* need USBMODE:ES */
	unsigned			have_sysif_regs:1;
	unsigned			le_setup_buf:1;
	unsigned change_ahb_burst:1;
	unsigned ahb_burst_mode:3;
	unsigned			suspended:1;
	unsigned			already_suspended:1;
	unsigned            lowpower:1;
	unsigned            irq_delay:1;
	unsigned            wakeup_event:1;
	struct fsl_usb2_wakeup_platform_data *wakeup_pdata;

	u32				id_gpio;
	/* register save area for suspend/resume */
	u32				pm_command;
	u32				pm_status;
	u32				pm_intr_enable;
	u32				pm_frame_index;
	u32				pm_segment;
	u32				pm_frame_list;
	u32				pm_async_next;
	u32				pm_configured_flag;
	u32				pm_portsc;
};

/* Flags in fsl_usb2_mph_platform_data */
#define FSL_USB2_PORT0_ENABLED	0x00000001
#define FSL_USB2_PORT1_ENABLED	0x00000002

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

struct fsl_ata_platform_data {
       int     adma_flag;      /* AMDA mode is used or not, 1:used.*/
       int     udma_mask;      /* UDMA modes h/w can handle */
       int     mwdma_mask;      /* MDMA modes h/w can handle */
       int     pio_mask;      /* PIO modes h/w can handle */
       int     fifo_alarm;     /* value for fifo_alarm reg */
       int     max_sg;         /* longest sglist h/w can handle */
       int     (*init)(struct platform_device *pdev);
       void    (*exit)(void);
	   char    *io_reg;
	   char    *core_reg;
};

/*!
 * This structure is used to define the One wire platform data.
 * It includes search rom accelerator.
 */
struct mxc_w1_config {
	int search_rom_accelerator;
};
/*!
 * This structure is used to define the SPI master controller's platform
 * data. It includes the SPI  bus number and the maximum number of
 * slaves/chips it supports.
 */
struct mxc_spi_master {
	/*!
	 * SPI Master's bus number.
	 */
	unsigned int bus_num;
	/*!
	 * SPI Master's maximum number of chip selects.
	 */
	unsigned int maxchipselect;
	/*!
	 * CSPI Hardware Version.
	 */
	unsigned int spi_version;
	/*!
	 * CSPI chipselect pin table.
	 * Workaround for ecspi chipselect pin may not keep correct level when
	 * idle.
	 */
	void (*chipselect_active) (int cspi_mode, int status, int chipselect);
	void (*chipselect_inactive) (int cspi_mode, int status, int chipselect);
};

struct mxc_ipu_config {
	int rev;
	void (*reset) (void);
	struct clk *di_clk[2];
	struct clk *csi_clk[2];
};

struct mxc_ir_platform_data {
	int uart_ir_mux;
	int ir_rx_invert;
	int ir_tx_invert;
	struct clk *uart_clk;
};

struct mxc_i2c_platform_data {
	u32 i2c_clk;
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

	int hp_irq;
	int (*hp_status) (void);

	int sysclk;

	int (*init) (void);	/* board specific init */
	int (*amp_enable) (int enable);
	int (*clock_enable) (int enable);
	int (*finit) (void);	/* board specific finit */
	void *priv;		/* used by board specific functions */
};

struct mxc_spdif_platform_data {
	int spdif_tx;
	int spdif_rx;
	int spdif_clk_44100;
	int spdif_clk_48000;
	int spdif_clkid;
	struct clk *spdif_clk;
	struct clk *spdif_core_clk;
	struct clk *spdif_audio_clk;
};

struct mxc_asrc_platform_data {
	struct clk *asrc_core_clk;
	struct clk *asrc_audio_clk;
	unsigned int channel_bits;
	int clk_map_ver;
};

struct mxc_bt_platform_data {
	char *bt_vdd;
	char *bt_vdd_parent;
	char *bt_vusb;
	char *bt_vusb_parent;
	void (*bt_reset) (void);
};

struct mxc_audio_codec_platform_data {
	char *core_regulator;
	char *io_regulator;
	char *analog_regulator;
	void (*pwdn)(int pwdn);
};

struct mxc_lightsensor_platform_data {
	char *vdd_reg;
	int rext;
};

struct mxc_fb_platform_data {
	struct fb_videomode *mode;
	int num_modes;
	char *mode_str;
	u32 interface_pix_fmt;
};

struct mxc_lcd_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	void (*reset) (void);
};


struct mxc_tsc_platform_data {
	char *vdd_reg;
	int penup_threshold;
	void (*active) (void);
	void (*inactive) (void);
};

struct mxc_tvout_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	u32 detect_line;
};

struct mxc_tvin_platform_data {
	char *dvddio_reg;
	char *dvdd_reg;
	char *avdd_reg;
	char *pvdd_reg;
	void (*pwdn) (int pwdn);
	void (*reset) (void);
	bool cvbs;
};

struct mxc_epdc_fb_mode {
	struct fb_videomode *vmode;
	int vscan_holdoff;
	int sdoed_width;
	int sdoed_delay;
	int sdoez_width;
	int sdoez_delay;
	int gdclk_hp_offs;
	int gdsp_offs;
	int gdoe_offs;
	int gdclk_offs;
	int num_ce;
};

struct mxc_epdc_fb_platform_data {
	struct mxc_epdc_fb_mode *epdc_mode;
	int num_modes;
	void (*get_pins) (void);
	void (*put_pins) (void);
	void (*enable_pins) (void);
	void (*disable_pins) (void);
};

struct mxc_pm_platform_data {
	void (*suspend_enter) (void);
	void (*suspend_exit) (void);
};

/*! Platform data for the IDE drive structure. */
struct mxc_ide_platform_data {
	char *power_drive;	/*!< The power pointer */
	char *power_io;		/*!< The power pointer */
};

struct mxc_camera_platform_data {
	char *core_regulator;
	char *io_regulator;
	char *analog_regulator;
	char *gpo_regulator;
	u32 mclk;
	u32 csi;
	void (*pwdn)(int pwdn);
};

/*gpo1-3 is in fixed state by hardware design,
 * only deal with reset pin and clock_enable pin
 * only poll mode can be used to control the chip,
 * interrupt mode is not supported by 3ds*/
struct mxc_fm_platform_data {
	char *reg_vio;
	char *reg_vdd;
	void (*gpio_get) (void);
	void (*gpio_put) (void);
	void (*reset) (void);
	void (*clock_ctl) (int flag);
	u8	sksnr; /*0,disable;1,most stop;0xf,fewest stop*/
	u8	skcnt; /*0,disable;1,most stop;0xf,fewest stop*/
	/*
	00 = 87.5-108 MHz (USA,Europe) (Default).
	01 = 76-108 MHz (Japan wide band).
	10 = 76-90 MHz (Japan).
	11 = Reserved.
	*/
	u8	band;
	/*
	00 = 200 kHz (USA, Australia) (default).
	01 = 100 kHz (Europe, Japan).
	10 = 50 kHz.
	*/
	u8	space;
	u8	seekth;
};

struct mxc_mma7450_platform_data {
	char *reg_dvdd_io;
	char *reg_avdd;
	void (*gpio_pin_get) (void);
	void (*gpio_pin_put) (void);
	int int1;
	int int2;
};

struct mxc_keyp_platform_data {
	u16 *matrix;
	void (*active) (void);
	void (*inactive) (void);
	char *vdd_reg;
};

struct mxc_unifi_platform_data {
	void (*hardreset) (int pin_level);
	void (*enable) (int en);
	/* power parameters */
	char *reg_gpo1;
	char *reg_gpo2;
	char *reg_1v5_ana_bb;
	char *reg_vdd_vpa;
	char *reg_1v5_dd;

	int host_id;

	void *priv;
};

struct mxc_gps_platform_data {
	char *core_reg;
	char *analog_reg;
	struct regulator *gps_regu_core;
	struct regulator *gps_regu_analog;
};

struct mxc_mlb_platform_data {
	u32 buf_address;
	u32 phy_address;
	char *reg_nvcc;
	char *mlb_clk;
};

struct flexcan_platform_data {
	char *core_reg;
	char *io_reg;
	char *root_clk_id;
	void (*xcvr_enable) (int id, int en);
	void (*active) (int id);
	void (*inactive) (int id);
	/* word 1 */
	unsigned int br_presdiv:8;
	unsigned int br_rjw:2;
	unsigned int br_propseg:3;
	unsigned int br_pseg1:3;
	unsigned int br_pseg2:3;
	unsigned int maxmb:6;
	unsigned int xmit_maxmb:6;
	unsigned int wd1_resv:1;

	/* word 2 */
	unsigned int fifo:1;
	unsigned int wakeup:1;
	unsigned int srx_dis:1;
	unsigned int wak_src:1;
	unsigned int bcc:1;
	unsigned int lprio:1;
	unsigned int abort:1;
	unsigned int br_clksrc:1;
	unsigned int loopback:1;
	unsigned int smp:1;
	unsigned int boff_rec:1;
	unsigned int tsyn:1;
	unsigned int listen:1;
	unsigned int ext_msg:1;
	unsigned int std_msg:1;
};

struct tve_platform_data {
	char *dac_reg;
	char *dig_reg;
};

struct ldb_platform_data {
	char *lvds_bg_reg;
	u32 ext_ref;
};

 struct mxc_vpu_platform_data {
	void (*reset) (void);
};

struct mxc_esai_platform_data {
	void (*activate_esai_ports) (void);
	void (*deactivate_esai_ports) (void);
};

struct mxc_pwm_platform_data {
	int pwmo_invert;
	void (*enable_pwm_pad) (void);
	void (*disable_pwm_pad) (void);
};

/* The name that links the i.MX NAND Flash Controller driver to its devices. */

#define IMX_NFC_DRIVER_NAME  ("imx_nfc")

/* Resource names for the i.MX NAND Flash Controller driver. */

#define IMX_NFC_BUFFERS_ADDR_RES_NAME         \
			("i.MX NAND Flash Controller Buffer")
#define IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME    \
			("i.MX NAND Flash Controller Primary Registers")
#define IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME  \
			("i.MX NAND Flash Controller Secondary Registers")
#define IMX_NFC_INTERRUPT_RES_NAME            \
			("i.MX NAND Flash Controller Interrupt")

/**
 * struct imx_nfc_platform_data - i.MX NFC driver platform data.
 *
 * This structure communicates information to the i.MX NFC driver that can't be
 * expressed as resources.
 *
 * @nfc_major_version:  The "major version" of the NFC hardware.
 * @nfc_minor_version:  The "minor version" of the NFC hardware.
 * @force_ce:           If true, this flag causes the driver to assert the
 *                      hardware chip enable signal for the currently selected
 *                      chip as long as the MTD NAND Flash HAL has the chip
 *                      selected (not just when an I/O transaction is in
 *                      progress).
 * @target_cycle_in_ns: The target read and write cycle period, in nanoseconds.
 *                      NAND Flash part data sheets give minimum times for read
 *                      and write cycles in nanoseconds (usually tRC and tWC,
 *                      respectively). Set this value to the maximum of these
 *                      two parameters. The driver will set the NFC clock as
 *                      close as possible without violating this value.
 * @clock_name:         The name of the clock used by the NAND Flash controller.
 * @init:               A pointer to a function the driver must call so the
 *                      platform can prepare for this device to operate. This
 *                      pointer may be NULL.
 * @exit:               A pointer to a function the driver must call so the
 *                      platform clean up after this device stops operating.
 *                      This pointer may be NULL.
 * @set_page_size:      A pointer to a function the driver can call to set the
 *                      page size. This pointer may be NULL.
 *
 *                      For some i.MX SoC's, the NFC gets information about the
 *                      page size from signals driven by a system register
 *                      outside the NFC. The address and format of this external
 *                      register varies across SoC's. In other SoC's, the NFC
 *                      still receives this signal, but it is overridden by a
 *                      page size register in the NFC itself.
 *
 *                      For SoC's where the page size *must* be set in an
 *                      external register, the driver must rely on a platform-
 *                      specific function, and this member must point to it.
 *
 *                      For SoC's where the NFC has its own page size register,
 *                      the driver will set that register itself and ignore the
 *                      external signals. In this case, there's no need for the
 *                      platform-specific function and this member must be NULL.
 *
 *                      This function accepts the page size in bytes (MTD calls
 *                      this the "writesize") discovered by the NAND Flash MTD
 *                      base driver (e.g., 512, 2048, 4096). This size refers
 *                      specifically to the the data bytes in the page, *not*
 *                      including out-of-band bytes. The return value is zero if
 *                      the operation succeeded. The driver does *not* view a
 *                      non-zero value as an error code - only an indication of
 *                      failure. The driver will decide for itself what error
 *                      code to return to its caller.
 * @interleave:         Indicates that the driver should "interleave" the NAND
 *                      Flash chips it finds. If true, the driver will aggregate
 *                      the chips "horizontally" such that MTD will see a single
 *                      chip with a potentially very large page size. This can
 *                      improve write performance for some applications.
 * @partitions:         An optional pointer to an array of partitions. If this
 *                      is NULL, the driver will create a single MTD that
 *                      represents the entire medium.
 * @partition_count:    The number of elements in the partition array.
 */

struct imx_nfc_platform_data {
	unsigned int          nfc_major_version;
	unsigned int          nfc_minor_version;
	int                   force_ce;
	unsigned int          target_cycle_in_ns;
	char                  *clock_name;
	int                   (*init)(void);
	void                  (*exit)(void);
	int                   (*set_page_size)(unsigned int data_size_in_bytes);
	int                   interleave;
	struct mtd_partition  *partitions;
	unsigned int	      partition_count;
};

struct mxc_sim_platform_data {
	unsigned int clk_rate;
	char *clock_sim;
	char *power_sim;
	int (*init)(struct platform_device *pdev);
	void (*exit)(void);
	unsigned int detect; /* 1 have detect pin, 0 not */
};

struct mpc8xx_pcmcia_ops {
	void(*hw_ctrl)(int slot, int enable);
	int(*voltage_set)(int slot, int vcc, int vpp);
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

#endif /* _FSL_DEVICE_H_ */
