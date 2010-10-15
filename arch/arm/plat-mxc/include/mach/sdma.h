
/*
 * Copyright 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_SDMA_H__
#define __ASM_ARCH_MXC_SDMA_H__

/*!
 * @defgroup SDMA Smart Direct Memory Access (SDMA) Driver
 */

/*!
 * @file arch-mxc/sdma.h
 *
 * @brief This file contains the SDMA API declarations.
 *
 * SDMA is responsible on moving data between peripherals and memories (MCU, EMI and DSP).
 *
 * @ingroup SDMA
 */

#include <linux/interrupt.h>
#include <asm/dma.h>
#include <stdarg.h>

#include <mach/hardware.h>
#include <mach/dma.h>

/*!
 * This defines maximum DMA address
 */
#define MAX_DMA_ADDRESS 0xffffffff

/*!
 * This defines maximum number of DMA channels
 */
#ifdef CONFIG_MXC_SDMA_API
#define MAX_DMA_CHANNELS 32
#define MAX_BD_NUMBER    16
#define MXC_SDMA_DEFAULT_PRIORITY 1
#define MXC_SDMA_MIN_PRIORITY 1
#define MXC_SDMA_MAX_PRIORITY 7
#else
#define MAX_DMA_CHANNELS 0
#endif

#define MXC_FIFO_MEM_DEST_FIXED   0x1
#define MXC_FIFO_MEM_SRC_FIXED    0x2

#define SDMA_ASRC_INFO_WML_OFF	0
#define SDMA_ASRC_INFO_WML_MASK ((1 << 10) - 1)
#define SDMA_ASRC_INFO_PS	(1 << 10)
#define SDMA_ASRC_INFO_PA	(1 << 11)
#define SDMA_ASRC_INFO_TXFR_DIR	(1 << 14)
#define SDMA_ASRC_INFO_N_OFF	(24)
#define SDMA_ASRC_INFO_N_MASK	((1 << 4) - 1)

#define SDMA_ASRC_P2P_INFO_LWML_OFF 0
#define SDMA_ASRC_P2P_INFO_LWML_MASK ((1 << 8) - 1)
#define SDMA_ASRC_P2P_INFO_PS	(1 << 8)
#define SDMA_ASRC_P2P_INFO_PA	(1 << 9)
#define SDMA_ASRC_P2P_INFO_SPDIF (1 << 10)
#define SDMA_ASRC_P2P_INFO_SP (1 << 11)
#define SDMA_ASRC_P2P_INFO_DP (1 << 12)
#define SDMA_ASRC_P2P_INFO_HWML_OFF 14
#define SDMA_ASRC_P2P_INFO_HWML_MASK ((1 << 10) - 1)
#define SDMA_ASRC_P2P_INFO_LWE (1 << 28)
#define SDMA_ASRC_P2P_INFO_HWE (1 << 29)
#define SDMA_ASRC_P2P_INFO_CONT (1 << 31)

/*!
 * This enumerates  transfer types
 */
typedef enum {
	emi_2_per = 0,		/*!< EMI memory to peripheral */
	emi_2_int,		/*!< EMI memory to internal RAM */
	emi_2_emi,		/*!< EMI memory to EMI memory */
	emi_2_dsp,		/*!< EMI memory to DSP memory */
	per_2_int,		/*!< Peripheral to internal RAM */
	per_2_emi,		/*!< Peripheral to internal EMI memory */
	per_2_dsp,		/*!< Peripheral to DSP memory */
	per_2_per,		/*!< Peripheral to Peripheral */
	int_2_per,		/*!< Internal RAM to peripheral */
	int_2_int,		/*!< Internal RAM to Internal RAM */
	int_2_emi,		/*!< Internal RAM to EMI memory */
	int_2_dsp,		/*!< Internal RAM to DSP memory */
	dsp_2_per,		/*!< DSP memory to peripheral */
	dsp_2_int,		/*!< DSP memory to internal RAM */
	dsp_2_emi,		/*!< DSP memory to EMI memory */
	dsp_2_dsp,		/*!< DSP memory to DSP memory */
	emi_2_dsp_loop,		/*!< EMI memory to DSP memory loopback */
	dsp_2_emi_loop,		/*!< DSP memory to EMI memory loopback */
	dvfs_pll,		/*!< DVFS script with PLL change       */
	dvfs_pdr		/*!< DVFS script without PLL change    */
} sdma_transferT;

/*!
 * This enumerates peripheral types
 */
typedef enum {
	SSI,			/*!< MCU domain SSI */
	SSI_SP,			/*!< Shared SSI */
	MMC,			/*!< MMC */
	SDHC,			/*!< SDHC */
	UART,			/*!< MCU domain UART */
	UART_SP,		/*!< Shared UART */
	FIRI,			/*!< FIRI */
	CSPI,			/*!< MCU domain CSPI */
	CSPI_SP,		/*!< Shared CSPI */
	SIM,			/*!< SIM */
	ATA,			/*!< ATA */
	CCM,			/*!< CCM */
	EXT,			/*!< External peripheral */
	MSHC,			/*!< Memory Stick Host Controller */
	MSHC_SP,		/*!< Shared Memory Stick Host Controller */
	DSP,			/*!< DSP */
	MEMORY,			/*!< Memory */
	FIFO_MEMORY,		/*!< FIFO type Memory */
	SPDIF,			/*!< SPDIF */
	IPU_MEMORY,		/*!< IPU Memory */
	ASRC,			/*!< ASRC */
	ESAI,			/*!< ESAI */
} sdma_periphT;

#ifndef TRANSFER_32BIT
/*!
 * This defines SDMA access data size
 */
#define TRANSFER_32BIT      0x00
#define TRANSFER_8BIT       0x01
#define TRANSFER_16BIT      0x02
#define TRANSFER_24BIT      0x03

#endif

/*!
 * This defines maximum device name length passed during mxc_request_dma().
 */
#define MAX_DEVNAME_LENGTH 32

/*!
 * This defines SDMA interrupt callback function prototype.
 */
typedef void (*dma_callback_t) (void *arg);

/*!
 * Structure containing sdma channel parameters.
 */
typedef struct {
	__u32 watermark_level;	/*!< Lower/upper threshold that
				 *   triggers SDMA event
				 *   for p2p, this is event1 watermark level
				 */
	__u32 per_address;	/*!< Peripheral source/destination
				 *   physical address
				 *   for p2p, this is destination address
				 */
	sdma_periphT peripheral_type;	/*!< Peripheral type */
	sdma_transferT transfer_type;	/*!< Transfer type   */
	int event_id;		/*!< Event number,
				 *   needed by all channels
				 *   that started by peripherals dma
				 *   request (per_2_*,*_2_per)
				 *   Not used for memory and DSP
				 *   transfers.
				 */
	int event_id2;		/*!< Second event number,
				 *   used in ATA scripts only.
				 */
	int bd_number;		/*!< Buffer descriptors number.
				 *   If not set, single buffer
				 *   descriptor will be used.
				 */
	dma_callback_t callback;	/*!   callback function            */
	void *arg;		/*!   callback argument            */
	unsigned long word_size:8;	/*!< SDMA data access word size    */
	unsigned long ext:1;	/*!< 1: extend parameter structure */
} dma_channel_params;

typedef struct {
	dma_channel_params common;
	unsigned long p2p_dir:1;	/*!< 0: per2 to per.
					 * the device of peripheral_type is per.
					 * 1: per to per2
					 * the device of peripheral_type is per2
					 */
	unsigned long info_bits;	/*!< info field in context */
	unsigned long info_mask;	/*!< info field mask in context */
	__u32 watermark_level2;	/*!< event2 threshold that
				 *   triggers SDMA event
				 *   just valid for p2p.
				 */
	__u32 per_address2;	/*!< Peripheral source
				 *   physical address.
				 *   just valid for p2p.
				 */
	struct dma_channel_info info;	/*!< the channel special parameter */
} dma_channel_ext_params;

/*!
 * Structure containing sdma request  parameters.
 */
typedef struct {
	/*!   physical source memory address        */
	__u8 *sourceAddr;
	/*!   physical destination memory address   */
	__u8 *destAddr;
	/*!   amount of data to transfer,
	 * updated during mxc_dma_get_config
	 */
	__u16 count;
	/*!< DONE bit of the buffer descriptor,
	 * updated during mxc_dma_get_config
	 * 0 - means the BD is done and closed by SDMA
	 * 1 - means the BD is still being processed by SDMA
	 */
	int bd_done;
	/*!< CONT bit of the buffer descriptor,
	 * set it if full multi-buffer descriptor mechanism
	 * required.
	 */
	int bd_cont;
	/*!< ERROR bit of the buffer descriptor,
	 * updated during mxc_dma_get_config.
	 * If it is set - there was an error during BD processing.
	 */
	int bd_error;
} dma_request_t;

/*!
 * Structure containing sdma request  parameters.
 */
typedef struct {
	/*! address of ap_2_ap script */
	int mxc_sdma_ap_2_ap_addr;
	/*! address of ap_2_bp script */
	int mxc_sdma_ap_2_bp_addr;
	/*! address of ap_2_ap_fixed script */
	int mxc_sdma_ap_2_ap_fixed_addr;
	/*! address of bp_2_ap script */
	int mxc_sdma_bp_2_ap_addr;
	/*! address of loopback_on_dsp_side script */
	int mxc_sdma_loopback_on_dsp_side_addr;
	/*! address of mcu_interrupt_only script */
	int mxc_sdma_mcu_interrupt_only_addr;

	/*! address of firi_2_per script */
	int mxc_sdma_firi_2_per_addr;
	/*! address of firi_2_mcu script */
	int mxc_sdma_firi_2_mcu_addr;
	/*! address of per_2_firi script */
	int mxc_sdma_per_2_firi_addr;
	/*! address of mcu_2_firi script */
	int mxc_sdma_mcu_2_firi_addr;

	/*! address of uart_2_per script */
	int mxc_sdma_uart_2_per_addr;
	/*! address of uart_2_mcu script */
	int mxc_sdma_uart_2_mcu_addr;
	/*! address of per_2_app script */
	int mxc_sdma_per_2_app_addr;
	/*! address of mcu_2_app script */
	int mxc_sdma_mcu_2_app_addr;
	/*! address of per_2_per script */
	int mxc_sdma_per_2_per_addr;

	/*! address of uartsh_2_per script */
	int mxc_sdma_uartsh_2_per_addr;
	/*! address of uartsh_2_mcu script */
	int mxc_sdma_uartsh_2_mcu_addr;
	/*! address of per_2_shp script */
	int mxc_sdma_per_2_shp_addr;
	/*! address of mcu_2_shp script */
	int mxc_sdma_mcu_2_shp_addr;

	/*! address of ata_2_mcu script */
	int mxc_sdma_ata_2_mcu_addr;
	/*! address of mcu_2_ata script */
	int mxc_sdma_mcu_2_ata_addr;

	/*! address of app_2_per script */
	int mxc_sdma_app_2_per_addr;
	/*! address of app_2_mcu script */
	int mxc_sdma_app_2_mcu_addr;
	/*! address of shp_2_per script */
	int mxc_sdma_shp_2_per_addr;
	/*! address of shp_2_mcu script */
	int mxc_sdma_shp_2_mcu_addr;

	/*! address of mshc_2_mcu script */
	int mxc_sdma_mshc_2_mcu_addr;
	/*! address of mcu_2_mshc script */
	int mxc_sdma_mcu_2_mshc_addr;

	/*! address of spdif_2_mcu script */
	int mxc_sdma_spdif_2_mcu_addr;
	/*! address of mcu_2_spdif script */
	int mxc_sdma_mcu_2_spdif_addr;

	/*! address of asrc_2_mcu script */
	int mxc_sdma_asrc_2_mcu_addr;

	/*! address of ext_mem_2_ipu script */
	int mxc_sdma_ext_mem_2_ipu_addr;

	/*! address of descrambler script */
	int mxc_sdma_descrambler_addr;

	/*! address of dptc_dvfs script */
	int mxc_sdma_dptc_dvfs_addr;

	int mxc_sdma_utra_addr;

	/*! address of peripheral ssi to mcu script */
	int mxc_sdma_ssiapp_2_mcu_addr;
	/*! address of mcu to peripheral ssi script */
	int mxc_sdma_mcu_2_ssiapp_addr;

	/*! address of shared peripheral ssi to mcu script */
	int mxc_sdma_ssish_2_mcu_addr;
	/*! address of mcu to shared peripheral ssi script */
	int mxc_sdma_mcu_2_ssish_addr;

	/*! address where ram code starts */
	int mxc_sdma_ram_code_start_addr;
	/*! size of the ram code */
	int mxc_sdma_ram_code_size;

	/*! RAM image address */
	unsigned short *mxc_sdma_start_addr;
} sdma_script_start_addrs;

/*! Structure to store the initialized dma_channel parameters */
typedef struct mxc_sdma_channel_params {
	/*! Channel type (static channel number or dynamic channel) */
	unsigned int channel_num;
	/*! Channel priority [0x1(lowest) - 0x7(highest)] */
	unsigned int chnl_priority;
	/*! Channel params */
	dma_channel_params chnl_params;
} mxc_sdma_channel_params_t;

/*! Structure to store the initialized dma_channel extend parameters */
typedef struct mxc_sdma_channel_ext_params {
	/*! Channel type (static channel number or dynamic channel) */
	unsigned int channel_num;
	/*! Channel priority [0x1(lowest) - 0x7(highest)] */
	unsigned int chnl_priority;
	/*! Channel extend params */
	dma_channel_ext_params chnl_ext_params;
} mxc_sdma_channel_ext_params_t;

/*! Private SDMA data structure */
typedef struct mxc_dma_channel_private {
	/*! ID of the buffer that was processed */
	unsigned int buf_tail;
	/*! Tasklet for the channel */
	struct tasklet_struct chnl_tasklet;
	/*! Flag indicates if interrupt is required after every BD transfer */
	int intr_after_every_bd;
} mxc_dma_channel_private_t;

/*!
 * Setup channel according to parameters.
 * Must be called once after mxc_request_dma()
 *
 * @param   channel           channel number
 * @param   p                 channel parameters pointer
 * @return  0 on success, error code on fail
 */
int mxc_dma_setup_channel(int channel, dma_channel_params *p);

/*!
 * Setup the channel priority. This can be used to change the default priority
 * for the channel.
 *
 * @param   channel           channel number
 * @param   priority          priority to be set for the channel
 *
 * @return  0 on success, error code on failure
 */
int mxc_dma_set_channel_priority(unsigned int channel, unsigned int priority);

/*!
 * Allocates dma channel.
 * If channel's value is 0, then the function allocates a free channel
 * dynamically and sets its value to channel.
 * Else allocates requested channel if it is free.
 * If the channel is busy or no free channels (in dynamic allocation) -EBUSY returned.
 *
 * @param   channel           pointer to channel number
 * @param   devicename        device name
 * @return  0 on success, error code on fail
 */
int mxc_request_dma(int *channel, const char *devicename);

/*!
 * Configures request parameters. Can be called multiple times after
 * mxc_request_dma() and mxc_dma_setup_channel().
 *
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to set
 * @return  0 on success, error code on fail
 */
/* int mxc_dma_set_config(int channel, dma_request_t *p, int bd_index); */
int mxc_dma_set_config(int channel, dma_request_t *p, int bd_index);

/*!
 * Returns request parameters.
 *
 * @param   channel           channel number
 * @param   p                 request parameters pointer
 * @param   bd_index          index of buffer descriptor to get
 * @return  0 on success, error code on fail
 */
/* int mxc_dma_get_config(int channel, dma_request_t *p, int bd_index); */
int mxc_dma_get_config(int channel, dma_request_t *p, int bd_index);

/*!
 * This function is used by MXC IPC's write_ex2. It passes the a pointer to the
 * data control structure to iapi_write_ipcv2()
 *
 * @param channel  SDMA channel number
 * @param ctrl_ptr Data Control structure pointer
 */
int mxc_sdma_write_ipcv2(int channel, void *ctrl_ptr);

/*!
 * This function is used by MXC IPC's read_ex2. It passes the a pointer to the
 * data control structure to iapi_read_ipcv2()
 *
 * @param channel   SDMA channel number
 * @param ctrl_ptr  Data Control structure pointer
 */
int mxc_sdma_read_ipcv2(int channel, void *ctrl_ptr);

/*!
 * Starts dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_start(int channel);

/*!
 * Stops dma channel.
 *
 * @param   channel           channel number
 */
int mxc_dma_stop(int channel);

/*!
 * Frees dma channel.
 *
 * @param   channel           channel number
 */
void mxc_free_dma(int channel);

/*!
 * Sets callback function. Used with standard dma api
 *  for supporting interrupts
 *
 * @param   channel           channel number
 * @param   callback          callback function pointer
 * @param   arg               argument for callback function
 */
void mxc_dma_set_callback(int channel, dma_callback_t callback, void *arg);

/*!
 * Allocates uncachable buffer. Uses hash table.
 *
 * @param   size    size of allocated buffer
 * @return  pointer to buffer
 */
void *sdma_malloc(size_t size);

#ifdef CONFIG_SDMA_IRAM
/*!
 * Allocates uncachable buffer from IRAM..
 *
 * @param   size    size of allocated buffer
 * @return  pointer to buffer
 */
void *sdma_iram_malloc(size_t size);
#endif				/*CONFIG_SDMA_IRAM */

/*!
 * Frees uncachable buffer. Uses hash table.
 */
void sdma_free(void *buf);

/*!
 * Converts virtual to physical address. Uses hash table.
 *
 * @param   buf  virtual address pointer
 * @return  physical address value
 */
unsigned long sdma_virt_to_phys(void *buf);

/*!
 * Converts physical to virtual address. Uses hash table.
 *
 * @param   buf  physical address value
 * @return  virtual address pointer
 */
void *sdma_phys_to_virt(unsigned long buf);

/*!
 * Configures the BD_INTR bit on a buffer descriptor parameters.
 *
 *
 * @param   channel           channel number
 * @param   bd_index          index of buffer descriptor to set
 * @param   bd_intr           flag to set or clear the BD_INTR bit
 */
void mxc_dma_set_bd_intr(int channel, int bd_index, int bd_intr);

/*!
 * Gets the BD_INTR bit on a buffer descriptor.
 *
 *
 * @param   channel           channel number
 * @param   bd_index          index of buffer descriptor to set
 *
 * @return returns the BD_INTR bit status
 */
int mxc_dma_get_bd_intr(int channel, int bd_index);

/*!
 * Stop the current transfer
 *
 * @param   channel           channel number
 * @param   buffer_number     number of buffers (beginning with 0),
 *                            whose done bits should be reset to 0
 */
int mxc_dma_reset(int channel, int buffer_number);

/*!
 * This functions Returns the SDMA paramaters associated for a module
 *
 * @param channel_id the ID of the module requesting DMA
 * @return returns the sdma parameters structure for the device
 */
mxc_sdma_channel_params_t *mxc_sdma_get_channel_params(mxc_dma_device_t
						       channel_id);

/*!
 * This functions marks the SDMA channels that are statically allocated
 *
 * @param chnl the channel array used to store channel information
 */
void mxc_get_static_channels(mxc_dma_channel_t *chnl);

/*!
 * Initializes SDMA driver
 */
int __init sdma_init(void);

#define DEFAULT_ERR     1

#endif
