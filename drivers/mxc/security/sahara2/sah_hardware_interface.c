/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file sah_hardware_interface.c
 *
 * @brief Provides an interface to the SAHARA hardware registers.
 *
 */

/* SAHARA Includes */
#include <sah_driver_common.h>
#include <sah_hardware_interface.h>
#include <sah_memory_mapper.h>
#include <sah_kernel.h>

#if defined DIAG_DRV_IF || defined(DO_DBG)
#include <diagnostic.h>
#ifndef LOG_KDIAG
#define LOG_KDIAG(x) os_printk("%s\n", x)
#endif

static void sah_Dump_Link(const char *prefix, const sah_Link * link,
			  dma_addr_t addr);

/* This is for sprintf() to use when constructing output. */
#define DIAG_MSG_SIZE   1024
/* was 200 */
#define MAX_DUMP        200
static char Diag_msg[DIAG_MSG_SIZE];

#endif				/* DIAG_DRV_IF */

/*!
 * Number of descriptors sent to Sahara.  This value should only be updated
 * with the main queue lock held.
 */
uint32_t dar_count;

/*! The "link-list optimize" bit in the Header of a Descriptor */
#define SAH_HDR_LLO 0x01000000

/* IO_ADDRESS() is Linux macro -- need portable equivalent */
#define SAHARA_BASE_ADDRESS             IO_ADDRESS(SAHA_BASE_ADDR)
#define SAHARA_VERSION_REGISTER_OFFSET  0x000
#define SAHARA_DAR_REGISTER_OFFSET      0x004
#define SAHARA_CONTROL_REGISTER_OFFSET  0x008
#define SAHARA_COMMAND_REGISTER_OFFSET  0x00C
#define SAHARA_STATUS_REGISTER_OFFSET   0x010
#define SAHARA_ESTATUS_REGISTER_OFFSET  0x014
#define SAHARA_FLT_ADD_REGISTER_OFFSET  0x018
#define SAHARA_CDAR_REGISTER_OFFSET     0x01C
#define SAHARA_IDAR_REGISTER_OFFSET     0x020
#define SAHARA_OSTATUS_REGISTER_OFFSET  0x028
#define SAHARA_CONFIG_REGISTER_OFFSET   0x02C
#define SAHARA_MM_STAT_REGISTER_OFFSET  0x030

/*! Register within Sahara which contains hardware version. (1 or 2).  */
#define SAHARA_VERSION_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_VERSION_REGISTER_OFFSET)

/*! Register within Sahara which is used to provide new work to the block. */
#define SAHARA_DAR_REGISTER     (SAHARA_BASE_ADDRESS + \
                                 SAHARA_DAR_REGISTER_OFFSET)

/*! Register with Sahara which is used for configuration. */
#define SAHARA_CONTROL_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_CONTROL_REGISTER_OFFSET)

/*! Register with Sahara which is used for changing status. */
#define SAHARA_COMMAND_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_COMMAND_REGISTER_OFFSET)

/*! Register with Sahara which is contains status and state. */
#define SAHARA_STATUS_REGISTER  (SAHARA_BASE_ADDRESS + \
                                 SAHARA_STATUS_REGISTER_OFFSET)

/*! Register with Sahara which is contains error status information. */
#define SAHARA_ESTATUS_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_ESTATUS_REGISTER_OFFSET)

/*! Register with Sahara which is contains faulting address information. */
#define SAHARA_FLT_ADD_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_FLT_ADD_REGISTER_OFFSET)

/*! Register with Sahara which is contains current descriptor address. */
#define SAHARA_CDAR_REGISTER    (SAHARA_BASE_ADDRESS + \
                                 SAHARA_CDAR_REGISTER_OFFSET)

/*! Register with Sahara which is contains initial descriptor address (of a
    chain). */
#define SAHARA_IDAR_REGISTER    (SAHARA_BASE_ADDRESS + \
                                 SAHARA_IDAR_REGISTER_OFFSET)

/*! Register with Sahara which is contains op status information. */
#define SAHARA_OSTATUS_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_OSTATUS_REGISTER_OFFSET)

/*! Register with Sahara which is contains configuration information. */
#define SAHARA_CONFIG_REGISTER (SAHARA_BASE_ADDRESS + \
                                SAHARA_CONFIG_REGISTER_OFFSET)

/*! Register with Sahara which is contains configuration information. */
#define SAHARA_MM_STAT_REGISTER (SAHARA_BASE_ADDRESS + \
                                 SAHARA_MM_STAT_REGISTER_OFFSET)

/* Local Functions */
#if defined DIAG_DRV_IF || defined DO_DBG
void sah_Dump_Region(const char *prefix, const unsigned char *data,
		     dma_addr_t addr, unsigned length);

#endif				/* DIAG_DRV_IF */

/* time out value when polling SAHARA status register for completion */
static uint32_t sah_poll_timeout = 0xFFFFFFFF;

/*!
 * Polls Sahara to determine when its current operation is complete
 *
 * @return   last value found in Sahara's status register
 */
sah_Execute_Status sah_Wait_On_Sahara()
{
	uint32_t count = 0;	/* ensure we don't get stuck in the loop forever */
	sah_Execute_Status status;	/* Sahara's status register */
	uint32_t stat_reg;

	pr_debug("Entered sah_Wait_On_Sahara\n");

	do {
		/* get current status register from Sahara */
		stat_reg = sah_HW_Read_Status();
		status = stat_reg & SAH_EXEC_STATE_MASK;

		/* timeout if SAHARA takes too long to complete */
		if (++count == sah_poll_timeout) {
			status = SAH_EXEC_FAULT;
			printk("sah_Wait_On_Sahara timed out\n");
		}

		/* stay in loop as long as Sahara is still busy */
	} while ((status == SAH_EXEC_BUSY) || (status == SAH_EXEC_DONE1_BUSY2));

	if (status == SAH_EXEC_ERROR1) {
		if (stat_reg & OP_STATUS) {
			status = SAH_EXEC_OPSTAT1;
		}
	}

	return status;
}				/* sah_Wait_on_Sahara() */

/*!
 * This function resets the SAHARA hardware. The following operations are
 * performed:
 *   1. Resets SAHARA.
 *   2. Requests BATCH mode.
 *   3. Enables interrupts.
 *   4. Requests Little Endian mode.
 *
 * @brief     SAHARA hardware reset function.
 *
 * @return   void
 */
int sah_HW_Reset(void)
{
	sah_Execute_Status sah_state;
	int status;		/* this is the value to return to the calling routine */
	uint32_t saha_control = 0;

#ifndef USE_3WORD_BURST
#ifdef FSL_HAVE_SAHARA2
	saha_control |= (8 << 16);	/* Allow 8-word burst */
#endif
#else
/***************** HARDWARE BUG WORK AROUND ******************/
/* A burst size of > 4 can cause Sahara DMA to issue invalid AHB transactions
 * when crossing 1KB boundaries.  By limiting the 'burst size' to 3, these
 * invalid transactions will not be generated, but Sahara will still transfer
 * data more efficiently than if the burst size were set to 1.
 */
	saha_control |= (3 << 16);	/* Limit DMA burst size. For versions 2/3 */
#endif				/* USE_3WORD_BURST */

#ifdef DIAG_DRV_IF
	snprintf(Diag_msg, DIAG_MSG_SIZE,
		 "Address of SAHARA_BASE_ADDRESS = 0x%08x\n",
		 SAHARA_BASE_ADDRESS);
	LOG_KDIAG(Diag_msg);
	snprintf(Diag_msg, DIAG_MSG_SIZE,
		 "Sahara Status register before reset: %08x",
		 sah_HW_Read_Status());
	LOG_KDIAG(Diag_msg);
#endif

	/* Write the Reset & BATCH mode command to the SAHARA Command register. */
	sah_HW_Write_Command(CMD_BATCH | CMD_RESET);
#ifdef SAHARA4_NO_USE_SQUIB
	{
		uint32_t cfg = sah_HW_Read_Config();
		cfg &= ~0x10000;
		sah_HW_Write_Config(cfg);
	}
#endif

	sah_poll_timeout = 0x0FFFFFFF;
	sah_state = sah_Wait_On_Sahara();
#ifdef DIAG_DRV_IF
	snprintf(Diag_msg, DIAG_MSG_SIZE,
		 "Sahara Status register after reset: %08x",
		 sah_HW_Read_Status());
	LOG_KDIAG(Diag_msg);
#endif
	/* on reset completion, check that Sahara is in the idle state */
	status = (sah_state == SAH_EXEC_IDLE) ? 0 : OS_ERROR_FAIL_S;

	/* Set initial value out of reset */
	sah_HW_Write_Control(saha_control);

#ifndef NO_RESEED_WORKAROUND
/***************** HARDWARE BUG WORK AROUND ******************/
/* In order to set the 'auto reseed' bit, must first acquire a random value. */
	/*
	 * to solve a hardware bug, a random number must be generated before
	 * the 'RNG Auto Reseed' bit can be set. So this generates a random
	 * number that is thrown away.
	 *
	 * Note that the interrupt bit has not been set at this point so
	 * the result can be polled.
	 */
#ifdef DIAG_DRV_IF
	LOG_KDIAG("Create and submit Random Number Descriptor");
#endif

	if (status == OS_ERROR_OK_S) {
		/* place to put random number */
		volatile uint32_t *random_data_ptr;
		sah_Head_Desc *random_desc;
		dma_addr_t desc_dma;
		dma_addr_t rand_dma;
		const int rnd_cnt = 3;	/* how many random 32-bit values to get */

		/* Get space for data -- assume at least 32-bit aligned! */
		random_data_ptr = os_alloc_memory(rnd_cnt * sizeof(uint32_t),
						  GFP_ATOMIC);

		random_desc = sah_Alloc_Head_Descriptor();

		if ((random_data_ptr == NULL) || (random_desc == NULL)) {
			status = OS_ERROR_FAIL_S;
		} else {
			int i;

			/* Clear out values */
			for (i = 0; i < rnd_cnt; i++) {
				random_data_ptr[i] = 0;
			}

			rand_dma = os_pa(random_data_ptr);

			random_desc->desc.header = 0xB18C0000;	/* LLO get random number */
			random_desc->desc.len1 =
			    rnd_cnt * sizeof(*random_data_ptr);
			random_desc->desc.ptr1 = (void *)rand_dma;
			random_desc->desc.original_ptr1 =
			    (void *)random_data_ptr;

			random_desc->desc.len2 = 0;	/* not used */
			random_desc->desc.ptr2 = 0;	/* not used */

			random_desc->desc.next = 0;	/* chain terminates here */
			random_desc->desc.original_next = 0;	/* chain terminates here */

			desc_dma = random_desc->desc.dma_addr;

			/* Force in-cache data out to RAM */
			os_cache_clean_range(random_data_ptr,
					     rnd_cnt *
					     sizeof(*random_data_ptr));

			/* pass descriptor to Sahara */
			sah_HW_Write_DAR(desc_dma);

			/*
			 * Wait for RNG to complete (interrupts are disabled at this point
			 * due to sahara being reset previously) then check for error
			 */
			sah_state = sah_Wait_On_Sahara();
			/* Force CPU to ignore in-cache and reload from RAM */
			os_cache_inv_range(random_data_ptr,
					   rnd_cnt * sizeof(*random_data_ptr));

			/* if it didn't move to done state, an error occured */
			if (
#ifndef SUBMIT_MULTIPLE_DARS
				   (sah_state != SAH_EXEC_IDLE) &&
#endif
				   (sah_state != SAH_EXEC_DONE1)
			    ) {
				status = OS_ERROR_FAIL_S;
				os_printk
				    ("(sahara) Failure: state is %08x; random_data is"
				     " %08x\n", sah_state, *random_data_ptr);
				os_printk
				    ("(sahara) CDAR: %08x, IDAR: %08x, FADR: %08x,"
				     " ESTAT: %08x\n", sah_HW_Read_CDAR(),
				     sah_HW_Read_IDAR(),
				     sah_HW_Read_Fault_Address(),
				     sah_HW_Read_Error_Status());
			} else {
				int i;
				int seen_rand = 0;

				for (i = 0; i < rnd_cnt; i++) {
					if (*random_data_ptr != 0) {
						seen_rand = 1;
						break;
					}
				}
				if (!seen_rand) {
					status = OS_ERROR_FAIL_S;
					os_printk
					    ("(sahara) Error:  Random number is zero!\n");
				}
			}
		}

		if (random_data_ptr) {
			os_free_memory((void *)random_data_ptr);
		}
		if (random_desc) {
			sah_Free_Head_Descriptor(random_desc);
		}
	}
/***************** END HARDWARE BUG WORK AROUND ******************/
#endif

	if (status == 0) {
#ifdef FSL_HAVE_SAHARA2
		saha_control |= CTRL_RNG_RESEED;
#endif

#ifndef SAHARA_POLL_MODE
		saha_control |= CTRL_INT_EN;	/* enable interrupts */
#else
		sah_poll_timeout = SAHARA_POLL_MODE_TIMEOUT;
#endif

#ifdef DIAG_DRV_IF
		snprintf(Diag_msg, DIAG_MSG_SIZE,
			 "Setting up Sahara's Control Register: %08x\n",
			 saha_control);
		LOG_KDIAG(Diag_msg);
#endif

		/* Rewrite the setup to the SAHARA Control register */
		sah_HW_Write_Control(saha_control);
#ifdef DIAG_DRV_IF
		snprintf(Diag_msg, DIAG_MSG_SIZE,
			 "Sahara Status register after control write: %08x",
			 sah_HW_Read_Status());
		LOG_KDIAG(Diag_msg);
#endif

#ifdef FSL_HAVE_SAHARA4
		{
			uint32_t cfg = sah_HW_Read_Config();
			sah_HW_Write_Config(cfg | 0x100);	/* Add RNG auto-reseed */
		}
#endif
	} else {
#ifdef DIAG_DRV_IF
		LOG_KDIAG("Reset failed\n");
#endif
	}

	return status;
}				/* sah_HW_Reset() */

/*!
 * This function enables High Assurance mode.
 *
 * @brief     SAHARA hardware enable High Assurance mode.
 *
 * @return   FSL_RETURN_OK_S             - if HA was set successfully
 * @return   FSL_RETURN_INTERNAL_ERROR_S - if HA was not set due to SAHARA
 *                                         being busy.
 */
fsl_shw_return_t sah_HW_Set_HA(void)
{
	/* This is the value to write to the register */
	uint32_t value;

	/* Read from the control register. */
	value = sah_HW_Read_Control();

	/* Set the HA bit */
	value |= CTRL_HA;

	/* Write to the control register. */
	sah_HW_Write_Control(value);

	/* Read from the control register. */
	value = sah_HW_Read_Control();

	return (value & CTRL_HA) ? FSL_RETURN_OK_S :
	    FSL_RETURN_INTERNAL_ERROR_S;
}

/*!
 * This function reads the SAHARA hardware Version Register.
 *
 * @brief     Read SAHARA hardware Version Register.
 *
 * @return   uint32_t Register value.
 */
uint32_t sah_HW_Read_Version(void)
{
	return os_read32(SAHARA_VERSION_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Control Register.
 *
 * @brief     Read SAHARA hardware Control Register.
 *
 * @return   uint32_t Register value.
 */
uint32_t sah_HW_Read_Control(void)
{
	return os_read32(SAHARA_CONTROL_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Status Register.
 *
 * @brief     Read SAHARA hardware Status Register.
 *
 * @return   uint32_t Register value.
 */
uint32_t sah_HW_Read_Status(void)
{
	return os_read32(SAHARA_STATUS_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Error Status Register.
 *
 * @brief     Read SAHARA hardware Error Status Register.
 *
 * @return   uint32_t Error Status value.
 */
uint32_t sah_HW_Read_Error_Status(void)
{
	return os_read32(SAHARA_ESTATUS_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Op Status Register.
 *
 * @brief     Read SAHARA hardware Op Status Register.
 *
 * @return   uint32_t Op Status value.
 */
uint32_t sah_HW_Read_Op_Status(void)
{
	return os_read32(SAHARA_OSTATUS_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Descriptor Address Register.
 *
 * @brief     Read SAHARA hardware DAR Register.
 *
 * @return   uint32_t DAR value.
 */
uint32_t sah_HW_Read_DAR(void)
{
	return os_read32(SAHARA_DAR_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Current Descriptor Address Register.
 *
 * @brief     Read SAHARA hardware CDAR Register.
 *
 * @return   uint32_t CDAR value.
 */
uint32_t sah_HW_Read_CDAR(void)
{
	return os_read32(SAHARA_CDAR_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Initial Descriptor Address Register.
 *
 * @brief     Read SAHARA hardware IDAR Register.
 *
 * @return   uint32_t IDAR value.
 */
uint32_t sah_HW_Read_IDAR(void)
{
	return os_read32(SAHARA_IDAR_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Fault Address Register.
 *
 * @brief     Read SAHARA Fault Address Register.
 *
 * @return   uint32_t Fault Address value.
 */
uint32_t sah_HW_Read_Fault_Address(void)
{
	return os_read32(SAHARA_FLT_ADD_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Multiple Master Status Register.
 *
 * @brief     Read SAHARA hardware MM Stat Register.
 *
 * @return   uint32_t MM Stat value.
 */
uint32_t sah_HW_Read_MM_Status(void)
{
	return os_read32(SAHARA_MM_STAT_REGISTER);
}

/*!
 * This function reads the SAHARA hardware Configuration Register.
 *
 * @brief     Read SAHARA Configuration Register.
 *
 * @return   uint32_t Configuration value.
 */
uint32_t sah_HW_Read_Config(void)
{
	return os_read32(SAHARA_CONFIG_REGISTER);
}

/*!
 * This function writes a command to the SAHARA hardware Command Register.
 *
 * @brief     Write to SAHARA hardware Command Register.
 *
 * @param    command     An unsigned 32bit command value.
 *
 * @return   void
 */
void sah_HW_Write_Command(uint32_t command)
{
	os_write32(SAHARA_COMMAND_REGISTER, command);
}

/*!
 * This function writes a control value to the SAHARA hardware Control
 * Register.
 *
 * @brief     Write to SAHARA hardware Control Register.
 *
 * @param    control     An unsigned 32bit control value.
 *
 * @return   void
 */
void sah_HW_Write_Control(uint32_t control)
{
	os_write32(SAHARA_CONTROL_REGISTER, control);
}

/*!
 * This function writes a configuration value to the SAHARA hardware Configuration
 * Register.
 *
 * @brief     Write to SAHARA hardware Configuration Register.
 *
 * @param    configuration     An unsigned 32bit configuration value.
 *
 * @return   void
 */
void sah_HW_Write_Config(uint32_t configuration)
{
	os_write32(SAHARA_CONFIG_REGISTER, configuration);
}

/*!
 * This function writes a descriptor address to the SAHARA Descriptor Address
 * Register.
 *
 * @brief     Write to SAHARA Descriptor Address Register.
 *
 * @param    pointer     An unsigned 32bit descriptor address value.
 *
 * @return   void
 */
void sah_HW_Write_DAR(uint32_t pointer)
{
	os_write32(SAHARA_DAR_REGISTER, pointer);
	dar_count++;
}

#if defined DIAG_DRV_IF || defined DO_DBG

static char *interpret_header(uint32_t header)
{
	unsigned desc_type = ((header >> 24) & 0x70) | ((header >> 16) & 0xF);

	switch (desc_type) {
	case 0x12:
		return "5/SKHA_ST_CTX";
	case 0x13:
		return "35/SKHA_LD_MODE_KEY";
	case 0x14:
		return "38/SKHA_LD_MODE_IN_CPHR_ST_CTX";
	case 0x15:
		return "4/SKHA_IN_CPHR_OUT";
	case 0x16:
		return "34/SKHA_ST_SBOX";
	case 0x18:
		return "1/SKHA_LD_MODE_IV_KEY";
	case 0x19:
		return "33/SKHA_ST_SBOX";
	case 0x1D:
		return "2/SKHA_LD_MODE_IN_CPHR_OUT";
	case 0x22:
		return "11/MDHA_ST_MD";
	case 0x25:
		return "10/MDHA_HASH_ST_MD";
	case 0x28:
		return "6/MDHA_LD_MODE_MD_KEY";
	case 0x2A:
		return "39/MDHA_ICV";
	case 0x2D:
		return "8/MDHA_LD_MODE_HASH_ST_MD";
	case 0x3C:
		return "18/RNG_GEN";
	case 0x40:
		return "19/PKHA_LD_N_E";
	case 0x41:
		return "36/PKHA_LD_A3_B0";
	case 0x42:
		return "27/PKHA_ST_A_B";
	case 0x43:
		return "22/PKHA_LD_A_B";
	case 0x44:
		return "23/PKHA_LD_A0_A1";
	case 0x45:
		return "24/PKHA_LD_A2_A3";
	case 0x46:
		return "25/PKHA_LD_B0_B1";
	case 0x47:
		return "26/PKHA_LD_B2_B3";
	case 0x48:
		return "28/PKHA_ST_A0_A1";
	case 0x49:
		return "29/PKHA_ST_A2_A3";
	case 0x4A:
		return "30/PKHA_ST_B0_B1";
	case 0x4B:
		return "31/PKHA_ST_B2_B3";
	case 0x4C:
		return "32/PKHA_EX_ST_B1";
	case 0x4D:
		return "20/PKHA_LD_A_EX_ST_B";
	case 0x4E:
		return "21/PKHA_LD_N_EX_ST_B";
	case 0x4F:
		return "37/PKHA_ST_B1_B2";
	default:
		return "??/UNKNOWN";
	}
}				/* cvt_desc_name() */

/*!
 * Dump chain of descriptors to the log.
 *
 * @brief Dump descriptor chain
 *
 * @param    chain     Kernel virtual address of start of chain of descriptors
 *
 * @return   void
 */
void sah_Dump_Chain(const sah_Desc * chain, dma_addr_t addr)
{
	int desc_no = 1;

	pr_debug("Chain for Sahara\n");

	while (chain != NULL) {
		char desc_name[50];

		sprintf(desc_name, "Desc %02d (%s)\n" KERN_DEBUG "Desc  ",
			desc_no++, interpret_header(chain->header));

		sah_Dump_Words(desc_name, (unsigned *)chain, addr,
			       6 /* #words in h/w link */ );
		if (chain->original_ptr1) {
			if (chain->header & SAH_HDR_LLO) {
				sah_Dump_Region(" Data1",
						(unsigned char *)chain->
						original_ptr1,
						(dma_addr_t) chain->ptr1,
						chain->len1);
			} else {
				sah_Dump_Link(" Link1", chain->original_ptr1,
					      (dma_addr_t) chain->ptr1);
			}
		}
		if (chain->ptr2) {
			if (chain->header & SAH_HDR_LLO) {
				sah_Dump_Region(" Data2",
						(unsigned char *)chain->
						original_ptr2,
						(dma_addr_t) chain->ptr2,
						chain->len2);
			} else {
				sah_Dump_Link(" Link2", chain->original_ptr2,
					      (dma_addr_t) chain->ptr2);
			}
		}

		addr = (dma_addr_t) chain->next;
		chain = (chain->next) ? (chain->original_next) : NULL;
	}
}

/*!
 * Dump chain of links to the log.
 *
 * @brief Dump chain of links
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    link      Kernel virtual address of start of chain of links
 *
 * @return   void
 */
static void sah_Dump_Link(const char *prefix, const sah_Link * link,
			  dma_addr_t addr)
{
#ifdef DUMP_SCC_DATA
	extern uint8_t *sahara_partition_base;
	extern dma_addr_t sahara_partition_phys;
#endif

	while (link != NULL) {
		sah_Dump_Words(prefix, (unsigned *)link, addr,
			       3 /* # words in h/w link */ );
		if (link->flags & SAH_STORED_KEY_INFO) {
#ifdef SAH_DUMP_DATA
#ifdef DUMP_SCC_DATA
			sah_Dump_Region("  Data",
					(uint8_t *) link->data -
					(uint8_t *) sahara_partition_phys +
					sahara_partition_base,
					(dma_addr_t) link->data, link->len);
#else
			pr_debug("  Key Slot %d\n", link->slot);
#endif
#endif
		} else {
#ifdef SAH_DUMP_DATA
			sah_Dump_Region("  Data", link->original_data,
					(dma_addr_t) link->data, link->len);
#endif
		}
		addr = (dma_addr_t) link->next;
		link = link->original_next;
	}
}

/*!
 * Dump given region of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    data      Kernel virtual address of start of region to dump
 * @param    length    Amount of data to dump
 *
 * @return   void
 */
void sah_Dump_Region(const char *prefix, const unsigned char *data,
		     dma_addr_t addr, unsigned length)
{
	unsigned count;
	char *output;
	unsigned data_len;

	sprintf(Diag_msg, "%s (%08X,%u):", prefix, addr, length);

	/* Restrict amount of data to dump */
	if (length > MAX_DUMP) {
		data_len = MAX_DUMP;
	} else {
		data_len = length;
	}

	/* We've already printed some text in output buffer, skip over it */
	output = Diag_msg + strlen(Diag_msg);

	for (count = 0; count < data_len; count++) {
		if ((count % 4) == 0) {
			*output++ = ' ';
		}
		sprintf(output, "%02X", *data++);
		output += 2;
	}

	pr_debug("%s\n", Diag_msg);
}

/*!
 * Dump given word of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix       Text to put in front of dumped data
 * @param    data         Kernel virtual address of start of region to dump
 * @param    word_count   Amount of data to dump
 *
 * @return   void
 */
void sah_Dump_Words(const char *prefix, const unsigned *data, dma_addr_t addr,
		    unsigned word_count)
{
	char *output;

	sprintf(Diag_msg, "%s (%08X,%uw): ", prefix, addr, word_count);

	/* We've already printed some text in output buffer, skip over it */
	output = Diag_msg + strlen(Diag_msg);

	while (word_count--) {
		sprintf(output, "%08X ", *data++);
		output += 9;
	}

	pr_debug("%s\n", Diag_msg);
	
}

#endif				/* DIAG_DRV_IF */

/* End of sah_hardware_interface.c */
