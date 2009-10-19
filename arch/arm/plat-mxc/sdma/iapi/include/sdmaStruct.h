/******************************************************************************
 *
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 ******************************************************************************
 *
 * File: sdmaStruct.h
 *
 * $Id sdmaStruct.h $
 *
 * Description: provides necessary definitions and inclusion for ipcmStruct.c
 *
 * $Log $
 *
 *****************************************************************************/
#ifndef _sdmaStruct_h
#define _sdmaStruct_h

/* ****************************************************************************
 * Include File Section
 ******************************************************************************/

/* ****************************************************************************
 * Macro-command Section
 ******************************************************************************/

/**
 * Identifier NULL
 */
#ifndef NULL
#define NULL 0
#endif

/**
 * Boolean identifiers
 */
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

/**
 * Number of channels
 */
#define CH_NUM  32
/**
 * Number of events
 */
#ifdef MXC_SDMA_V2
#define EVENTS_NUM   48
#else
#define EVENTS_NUM   32
#endif
/**
 * Channel configuration
 */
#define DONT_OWN_CHANNEL   0
#define OWN_CHANNEL        1

/**
 * Ownership (value defined to computed decimal value)
 */
#define CH_OWNSHP_OFFSET_EVT 0
#define CH_OWNSHP_OFFSET_MCU 1
#define CH_OWNSHP_OFFSET_DSP 2
/**
 * Indexof the greg which holds address to start a script from when channel
 * becomes current.
 */
#define SDMA_NUMBER_GREGS 8

/**
 * Channel contexts management
 */

#define CHANNEL_CONTEXT_BASE_ADDRESS		0x800
/**
 * Buffer descriptor status values.
 */
#define BD_DONE  0x01
#define BD_WRAP  0x02
#define BD_CONT  0x04
#define BD_INTR  0x08
#define BD_RROR  0x10
#define BD_LAST  0x20
#define BD_EXTD  0x80


/**
 * Data Node descriptor status values.
 */
#define DND_END_OF_FRAME  0x80
#define DND_END_OF_XFER   0x40
#define DND_DONE          0x20
#define DND_UNUSED        0x01

/**
 * IPCV2 descriptor status values.
 */
#define BD_IPCV2_END_OF_FRAME  0x40


#define IPCV2_MAX_NODES        50
/**
 * Error bit set in the CCB status field by the SDMA,
 * in setbd routine, in case of a transfer error
 */
#define DATA_ERROR  0x10000000

/**
 * Buffer descriptor commands.
 */
#define C0_ADDR             0x01
#define C0_LOAD             0x02
#define C0_DUMP             0x03
#define C0_SETCTX           0x07
#define C0_GETCTX           0x03
#define C0_SETDM            0x01
#define C0_SETPM            0x04
#define C0_GETDM            0x02
#define C0_GETPM            0x08
/**
 * Transfer types, encoded in the BD command field
 */
#define TRANSFER_32BIT      0x00
#define TRANSFER_8BIT       0x01
#define TRANSFER_16BIT      0x02
#define TRANSFER_24BIT      0x03
/**
 * Change endianness indicator in the BD command field
 */
#define CHANGE_ENDIANNESS   0x80
/**
 * Size in bytes
 */
#define SDMA_BD_SIZE  8
#define SDMA_EXTENDED_BD_SIZE  12
#define BD_NUMBER     4
/**
 * Channel interrupt policy
 */
#define DEFAULT_POLL 0
#define CALLBACK_ISR 1
/**
 * Channel status
 */
#define UNINITIALIZED 0
#define   INITIALIZED 1

/**
 * IoCtl particular values
 */
#define SET_BIT_ALL     0xFFFFFFFF
#define BD_NUM_OFFSET   16
#define BD_NUM_MASK     0xFFFF0000

/**
 * Maximum values for IoCtl calls, used in high or middle level calls
 */
#define MAX_BD_NUM         256
#define MAX_BD_SIZE        65536
#define MAX_BLOCKING       2
#define MAX_SYNCH          2
#define MAX_OWNERSHIP      8
#define MAX_CH_PRIORITY    8
#define MAX_TRUST          2
#define MAX_WML            256


/**
 * Access to channelDescriptor fields
 */
enum {
  IAPI_CHANNELNUMBER,
  IAPI_BUFFERDESCNUMBER,
  IAPI_BUFFERSIZE,
  IAPI_BLOCKING,
  IAPI_CALLBACKSYNCH,
  IAPI_OWNERSHIP,
  IAPI_PRIORITY,
  IAPI_TRUST,
  IAPI_UNUSED,
  IAPI_CALLBACKISR_PTR,
  IAPI_CCB_PTR,
  IAPI_BDWRAP,
  IAPI_WML
};

/**
 * Default values for channel descriptor - nobody ownes the channel
 */
#define CD_DEFAULT_OWNERSHIP  7


/**
 * User Type Section
 */

/**
 * Command/Mode/Count of buffer descriptors
 */

#if (ENDIANNESS==B_I_G_ENDIAN)
typedef struct iapi_modeCount_ipcv2 {
  unsigned long  status          :  8; /**< L, E , D bits stored here */
  unsigned long  reserved  :  8;
  unsigned long  count         : 16; /**< <size of the buffer pointed by this BD  */
} modeCount_ipcv2;
#else
typedef struct iapi_modeCount_ipcv2 {
  unsigned long  count         : 16; /**<size of the buffer pointed by this BD */
  unsigned long  reserved      :  8; /**Reserved*/
  unsigned long  status        :  8; /**< L, E , D bits stored here */
} modeCount_ipcv2;
#endif
/**
 * Data Node descriptor - IPCv2
 * (differentiated between evolutions of SDMA)
 */
typedef struct iapi_dataNodeDescriptor {
  modeCount_ipcv2  mode;          /**<command, status and count */
  void *           bufferAddr;    /**<address of the buffer described */
} dataNodeDescriptor;

#if (ENDIANNESS==B_I_G_ENDIAN)
typedef struct iapi_modeCount_ipcv1_v2 {
  unsigned long   endianness: 1;
  unsigned long   reserved: 7;
  unsigned long   status  :  8; /**< E,R,I,C,W,D status bits stored here */
  unsigned long   count   : 16; /**< size of the buffer pointed by this BD */
} modeCount_ipcv1_v2;
#else
typedef struct iapi_modeCount_ipcv1_v2 {
  unsigned long  count   : 16; /**<size of the buffer pointed by this BD */
  unsigned long  status  :  8; /**< E,R,I,C,W,D status bits stored here */
  unsigned long   reserved: 7;
  unsigned long   endianness: 1;
} modeCount_ipcv1_v2;
#endif
/**
 * Buffer descriptor
 * (differentiated between evolutions of SDMA)
 */
typedef struct iapi_bufferDescriptor_ipcv1_v2 {
  modeCount_ipcv1_v2  mode; /**<command, status and count */
  void *     bufferAddr;    /**<address of the buffer described */
  void *     extBufferAddr; /**<extended buffer address */
} bufferDescriptor_ipcv1_v2;


/**
 * Mode/Count of data node descriptors - IPCv2
 */

#if (ENDIANNESS==B_I_G_ENDIAN)
typedef struct iapi_modeCount {
  unsigned long  command :  8; /**< command mostlky used for channel 0 */
  unsigned long  status  :  8; /**< E,R,I,C,W,D status bits stored here */
  unsigned long  count   : 16; /**< size of the buffer pointed by this BD */
} modeCount;
#else
typedef struct iapi_modeCount {
  unsigned long  count   : 16; /**<size of the buffer pointed by this BD */
  unsigned long  status  :  8; /**< E,R,I,C,W,D status bits stored here */
  unsigned long  command :  8; /**< command mostlky used for channel 0 */
} modeCount;
#endif


/**
 * Buffer descriptor
 * (differentiated between evolutions of SDMA)
 */
typedef struct iapi_bufferDescriptor {
  modeCount  mode;          /**<command, status and count */
  void *     bufferAddr;    /**<address of the buffer described */
  void *     extBufferAddr; /**<extended buffer address */
} bufferDescriptor;



struct iapi_channelControlBlock;
struct iapi_channelDescriptor;
/**
 * Channel Descriptor
 */
typedef struct iapi_channelDescriptor {
  unsigned char   channelNumber   ;/**<stores the channel number */
  unsigned char   bufferDescNumber;/**<number of BD's automatically allocated for this channel */
  unsigned short  bufferSize      ;/**<size (in bytes) of buffer descriptors */
  unsigned long   blocking      :3;/**<blocking / non blocking feature selection */
  unsigned long   callbackSynch :1;/**<polling/ callback method selection */
  unsigned long   ownership     :3;/**<ownership of the channel (host+dedicated+event)*/
  unsigned long   priority      :3;/**<reflects the SDMA channel priority register */
  unsigned long   trust         :1;/**<trusted buffers or kernel allocated */
  unsigned long   useDataSize   :1;/**<indicates if the dataSize field is meaningfull */
  unsigned long   dataSize      :2;/**<data transfer size - 8,16,24 or 32 bits*/
  unsigned long   forceClose    :1;/**<if TRUE, close channel even with BD owned by SDMA*/
  unsigned long   scriptId      :7;/**<number of the script */
  unsigned long   watermarkLevel:10;/**<Watermark level for the peripheral access*/
  unsigned long   eventMask1;      /**<First Event mask */
  unsigned long   eventMask2;      /**<Second  Event mask */
  unsigned long   peripheralAddr;  /**<Address of the peripheral or its fifo when needed */
  void (* callbackISR_ptr)(struct iapi_channelDescriptor*, void*);   /**<pointer to the callback function (or NULL) */
  struct iapi_channelControlBlock * ccb_ptr; /**<pointer to the channel control block associated to this channel */
} channelDescriptor;

/**
 * Channel Status
 */
typedef struct iapi_channelStatus {
  unsigned long  unused        :29;/**<*/
  unsigned long  openedInit    : 1;/**<channel is initialized or not */
  unsigned long  stateDirection: 1;/**<sdma is reading/writing (as seen from channel owner core) */
  unsigned long  execute       : 1;/**<channel is being processed (started) or not */
} channelStatus;

/**
 * Channel control Block
 */
typedef struct iapi_channelControlBlock {
  bufferDescriptor *  currentBDptr;      /**<current buffer descriptor processed */
  bufferDescriptor *  baseBDptr;         /**<first element of buffer descriptor array */
  channelDescriptor * channelDescriptor; /**<pointer to the channel descriptor */
  channelStatus		  status;            /**<open/close ; started/stopped ; read/write */
} channelControlBlock;

/**
 * Context structure.
 */
#if (ENDIANNESS==B_I_G_ENDIAN)
typedef struct iapi_stateRegisters {
  unsigned long sf     : 1;/**<source falut while loading data */
  unsigned long unused0: 1;/**<*/
  unsigned long rpc    :14;/**<return program counter */
  unsigned long t      : 1;/**<test bit:status of arithmetic & test instruction*/
  unsigned long unused1: 1;/**<*/
  unsigned long pc     :14;/**<program counter */
  unsigned long lm     : 2;/**<loop mode */
  unsigned long epc    :14;/**<loop end program counter */
  unsigned long df     : 1;/**<destiantion falut while storing data */
  unsigned long unused2: 1;/**<*/
  unsigned long spc    :14;/**<loop start program counter */
} stateRegiters;
#else
typedef struct iapi_stateRegisters {
  unsigned long pc     :14;/**<program counter */
  unsigned long unused1: 1;/**<*/
  unsigned long t      : 1;/**<test bit: status of arithmetic & test instruction*/
  unsigned long rpc    :14;/**<return program counter */
  unsigned long unused0: 1;/**<*/
  unsigned long sf     : 1;/**<source falut while loading data */
  unsigned long spc    :14;/**<loop start program counter */
  unsigned long unused2: 1;/**<*/
  unsigned long df     : 1;/**<destiantion falut while storing data */
  unsigned long epc    :14;/**<loop end program counter */
  unsigned long lm     : 2;/**<loop mode */
} stateRegiters;
#endif

/**
 * This is SDMA version of SDMA
 */
typedef struct iapi_contextData {
  stateRegiters  channelState;              /**<channel state bits */
  unsigned long  gReg[ SDMA_NUMBER_GREGS ]; /**<general registers */
  unsigned long  mda; /**<burst dma destination address register */
  unsigned long  msa; /**<burst dma source address register */
  unsigned long  ms;  /**<burst dma status  register */
  unsigned long  md;  /**<burst dma data    register */
  unsigned long  pda; /**<peripheral dma destination address register */
  unsigned long  psa; /**<peripheral dma source address register */
  unsigned long  ps;  /**<peripheral dma  status  register */
  unsigned long  pd;  /**<peripheral dma  data    register */
  unsigned long  ca;  /**<CRC polynomial  register */
  unsigned long  cs;  /**<CRC accumulator register */
  unsigned long  dda; /**<dedicated core destination address register */
  unsigned long  dsa; /**<dedicated core source address register */
  unsigned long  ds;  /**<dedicated core status  register */
  unsigned long  dd;  /**<dedicated core data    register */
  unsigned long  scratch0;  /**<scratch */
  unsigned long  scratch1;  /**<scratch */
  unsigned long  scratch2;  /**<scratch */
  unsigned long  scratch3;  /**<scratch */
  unsigned long  scratch4;  /**<scratch */
  unsigned long  scratch5;  /**<scratch */
  unsigned long  scratch6;  /**<scratch */
  unsigned long  scratch7;  /**<scratch */

} contextData;

/**
 *This structure holds the necessary data for the assignment in the
 * dynamic channel-script association
 */
typedef struct iapi_script_data {
  unsigned short load_address;/**<start address of the script*/
  unsigned long  wml;		   /**<parameters for the channel descriptor*/
  unsigned long  shp_addr;    /**<shared peripheral base address*/
  unsigned long  per_addr;    /**<peripheral base address for p2p source*/
  unsigned long  event_mask1; /**<First Event mask */
  unsigned long  event_mask2; /**<Second Event mask */
} script_data;

/**
 *This structure holds the the useful bits of the CONFIG register
 */
typedef struct iapi_configs_data {
  unsigned long   dspdma   :1; /*indicates if the DSPDMA is used */
  unsigned long   rtdobs   :1; /*indicates if Real-Time Debug pins are enabled */
  unsigned long   acr   :1; /**indicates if AHB freq /core freq = 2 or 1 */
  unsigned long   csm   :2; /**indicates which context switch mode is selected*/
} configs_data;

#endif /* _sdmaStruct_h */
