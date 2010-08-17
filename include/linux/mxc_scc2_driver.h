
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

#ifndef SCC_DRIVER_H
#define SCC_DRIVER_H

/*
 * NAMING CONVENTIONS
 * ==================
 * (A note to maintainers and other interested parties)
 *
 * Use scc_ or SCC_ prefix for 'high-level' interface routines and the types
 * passed to those routines.  Try to avoid #defines in these interfaces.
 *
 * Use SMN_ or SCM_ prefix for the #defines used with scc_read_register() and
 * scc_write_register, or values passed/retrieved from those routines.
 */

/*! @file mxc_scc2_driver.h
 *
 * @brief (Header file to use the SCC2 driver.)
 *
 * The SCC2 driver is available to other kernel modules directly.  Secure
 * Partition functionality is extended to users through the SHW API.  Other
 * functionality of the SCC2 is limited to kernel-space users.
 *
 * With the exception of #scc_monitor_security_failure(), all routines are
 * 'synchronous', i.e. they will not return to their caller until the requested
 * action is complete, or fails to complete.  Some of these functions could
 * take quite a while to perform, depending upon the request.
 *
 * Routines are provided to:
 * @li trigger a security-violation alarm - #scc_set_sw_alarm()
 * @li get configuration and version information - #scc_get_configuration()
 * @li zeroize memory - #scc_zeroize_memories()
 * @li Work with secure partitions: #scc_allocate_partition()
 *     #scc_engage_partition() #scc_diminish_permissions()
 *     #scc_release_partition()
 * @li Encrypt or decrypt regions of data: #scc_encrypt_region()
 *     #scc_decrypt_region()
 * @li monitor the Security Failure alarm - #scc_monitor_security_failure()
 * @li stop monitoring Security Failure alarm -
 *     #scc_stop_monitoring_security_failure()
 * @li write registers of the SCC - #scc_write_register()
 * @li read registers of the SCC - #scc_read_register()
 *
 * The SCC2 encrypts and decrypts using Triple DES with an internally stored
 * key.  When the SCC2 is in Secure mode, it uses its secret, unique-per-chip
 * key.  When it is in Non-Secure mode, it uses a default key.  This ensures
 * that secrets stay secret if the SCC2 is not in Secure mode.
 *
 * Not all functions that could be provided in a 'high level' manner have been
 * implemented.  Among the missing are interfaces to the ASC/AIC components and
 * the timer functions.  These and other features must be accessed through
 * #scc_read_register() and #scc_write_register(), using the @c \#define values
 * provided.
 *
 * Here is a glossary of acronyms used in the SCC2 driver documentation:
 * - CBC - Cipher Block Chaining.  A method of performing a block cipher.
 *    Each block is encrypted using some part of the result of the previous
 *    block's encryption.  It needs an 'initialization vector' to seed the
 *    operation.
 * - ECB - Electronic Code Book.  A method of performing a block cipher.
 *    With a given key, a given block will always encrypt to the same value.
 * - DES - Data Encryption Standard.  (8-byte) Block cipher algorithm which
 *    uses 56-bit keys.  In SCC2, this key is constant and unique to the device.
 *    SCC uses the "triple DES" form of this algorithm.
 * - AIC - Algorithm Integrity Checker.
 * - ASC - Algorithm Sequence Checker.
 * - SMN - Security Monitor.  The part of the SCC2 responsible for monitoring
 *    for security problems and notifying the CPU and other PISA components.
 * - SCM - Secure Memory.  The part of the SCC2 which handles the cryptography.
 * - SCC - Security Controller.  Central security mechanism for PISA.
 * - PISA - Platform-Independent Security Architecture.
 */

/* Temporarily define compile-time flags to make Doxygen happy. */
#ifdef DOXYGEN_HACK
/** @defgroup scccompileflags SCC Driver compile-time flags
 *
 * These preprocessor flags should be set, if desired, in a makefile so
 * that they show up on the compiler command line.
 */
/** @addtogroup scccompileflags */

/** @{ */
/**
 * Compile-time flag to change @ref smnregs and @ref scmregs
 * offset values for the SCC's implementation on the MX.21 board.
 *
 * This must also be set properly for any code which calls the
 * scc_read_register() or scc_write_register() functions or references the
 * register offsets.
 */
#define TAHITI
/** @} */
#undef TAHITI

#endif				/* DOXYGEN_HACK */

/*! Major Version of the driver.  Used for
    scc_configuration->driver_major_version */
#define SCC_DRIVER_MAJOR_VERSION    2
/*! Old Minor Version of the driver. */
#define SCC_DRIVER_MINOR_VERSION_0    0
/*! Minor Version of the driver.  Used for
    scc_configuration->driver_minor_version */
#define SCC_DRIVER_MINOR_VERSION_2    2


/*!
 *  Interrupt line number of SCM interrupt.
 */
#define INT_SCC_SCM         MXC_INT_SCC_SCM

/*!
 *  Interrupt line number of the SMN interrupt.
 */
#define INT_SCC_SMN         MXC_INT_SCC_SMN

/**
 * @typedef scc_return_t
 */
/** Common status return values from SCC driver functions. */
	typedef enum scc_return_t {
		SCC_RET_OK = 0,	 /**< Function succeeded  */
		SCC_RET_FAIL,	 /**< Non-specific failure */
		SCC_RET_VERIFICATION_FAILED,
				 /**< Decrypt validation failed */
		SCC_RET_TOO_MANY_FUNCTIONS,
				 /**< At maximum registered functions */
		SCC_RET_BUSY,	 /**< SCC is busy and cannot handle request */
		/**< Encryption or decryption failed because@c count_out_bytes
			says that @c data_out is too small to hold the value. */
		SCC_RET_INSUFFICIENT_SPACE,
	} scc_return_t;

/**
 * @typedef scc_partition_status_t
 */
/** Partition status information. */
	typedef enum scc_partition_status_t {
		SCC_PART_S_UNUSABLE,
				  /**< Partition not implemented */
		SCC_PART_S_UNAVAILABLE,
				  /**< Partition owned by other host */
		SCC_PART_S_AVAILABLE,
				  /**< Partition available */
		SCC_PART_S_ALLOCATED,
				  /**< Partition owned by host but not engaged*/
		SCC_PART_S_ENGAGED,
				  /**< Partition owned by host and engaged */
	} scc_partition_status_t;

/**
 * Configuration information about SCC and the driver.
 *
 * This struct/typedef contains information from the SCC and the driver to
 * allow the user of the driver to determine the size of the SCC's memories and
 * the version of the SCC and the driver.
 */
	typedef struct scc_config_t {
		int driver_major_version;
				/**< Major version of the SCC driver code  */
		int driver_minor_version;
				/**< Minor version of the SCC driver code  */
		int scm_version; /**< Version from SCM Configuration register */
		int smn_version; /**< Version from SMN Status register */
		/**< Number of bytes per block of RAM; also
			block size of the crypto algorithm. */
		int block_size_bytes;
		int partition_size_bytes;
				/**< Number of bytes in each partition */
		int partition_count;
				/**< Number of partitions on this platform */
	} scc_config_t;

/**
 * @typedef scc_enc_dec_t
 */
/**
 * Determine whether SCC will run its cryptographic
 * function as an encryption or decryption.
 */
	typedef enum scc_enc_dec_t {
		SCC_ENCRYPT,	/**< Encrypt (from Red to Black) */
		SCC_DECRYPT	/**< Decrypt (from Black to Red) */
	} scc_enc_dec_t;

/**
 * @typedef scc_verify_t
 */
/**
 * Tell the driver whether it is responsible for verifying the integrity of a
 * secret.  During an encryption, using other than #SCC_VERIFY_MODE_NONE will
 * cause a check value to be generated and appended to the plaintext before
 * encryption.  During decryption, the check value will be verified after
 * decryption, and then stripped from the message.
 */
	typedef enum scc_verify_t {
    /** No verification value added or checked.  Input plaintext data must be
     *  be a multiple of the blocksize (#scc_get_configuration()).  */
		SCC_VERIFY_MODE_NONE,
    /** Driver will generate/validate a 2-byte CCITT CRC.  Input plaintext
		will be padded to a multiple of the blocksize, adding 3-10 bytes
		to the resulting output ciphertext.  Upon decryption, this padding
		will be stripped, and the CRC will be verified. */
		SCC_VERIFY_MODE_CCITT_CRC
	} scc_verify_t;

/**
 * @typedef scc_cypher_mode_t
 */
/**
 * Select the cypher mode to use for partition cover/uncover operations.
 */

	typedef enum scc_cypher_mode_t {
		SCC_CYPHER_MODE_ECB = 1,
				   /**< ECB mode */
		SCC_CYPHER_MODE_CBC = 2,
				   /**< CBC mode */
	} scc_cypher_mode_t;

/**
 * Allocate a partition of secure memory
 *
 * @param       smid_value  Value to use for the SMID register.  Must be 0 for
 *                          kernel mode ownership.
 * @param[out]  part_no     (If successful) Assigned partition number.
 * @param[out]  part_base   Kernel virtual address of the partition.
 * @param[out]  part_phys   Physical address of the partition.
 *
 * @return      SCC_RET_OK if successful.
 */
	extern scc_return_t
	    scc_allocate_partition(uint32_t smid_value,
				   int *part_no,
				   void **part_base, uint32_t *part_phys);

/* Note: This function has to be run in the same context (userspace or kernel
 * mode) as the process that will be using the partition.  Because the SCC2 API
 * is not accessible in user mode, this function is also provided as a macro in
 * in fsl_shw.h.  Kernel-mode users that include this file are able to use this
 * version of the function without having to include the whole SHW API.  If the
 * macro definition was defined before we got here, un-define it so this
 * version will be used instead.
 */

#ifdef scc_engage_partition
#undef scc_engage_partition
#endif

/**
 * Engage partition of secure memory
 *
 * @param part_base (kernel) Virtual
 * @param UMID NULL, or 16-byte UMID for partition security
 * @param permissions ORed values of the type SCM_PERM_* which will be used as
 *                    initial partition permissions.  SHW API users should use
 *                    the FSL_PERM_* definitions instead.
 *
 * @return SCC_RET_OK if successful.
 */
	extern scc_return_t
	    scc_engage_partition(void *part_base,
				 const uint8_t *UMID, uint32_t permissions);

/**
 * Release a partition of secure memory
 *
 * @param   part_base   Kernel virtual address of the partition to be released.
 *
 * @return  SCC_RET_OK if successful.
 */
	extern scc_return_t scc_release_partition(void *part_base);

/**
 * Diminish the permissions on a partition of secure memory
 *
 * @param part_base   Kernel virtual address of the partition.
 *
 * @param permissions ORed values of the type SCM_PERM_* which will be used as
 *                    initial partition permissions.  SHW API users should use
 *                    the FSL_PERM_* definitions instead.
 *
 * @return  SCC_RET_OK if successful.
 */
	extern scc_return_t
	    scc_diminish_permissions(void *part_base, uint32_t permissions);

/**
 * Query the status of a partition of secure memory
 *
 * @param part_base   Kernel virtual address of the partition.
 *
 * @return  SCC_RET_OK if successful.
 */
	extern scc_partition_status_t scc_partition_status(void *part_base);

/**
 * Calculate the physical address from the kernel virtual address.
 */
	extern uint32_t scc_virt_to_phys(void *address);
/*scc_return_t
scc_verify_slot_access(uint64_t owner_id, uint32_t slot, uint32_t access_len);*/


/**
 * Encrypt a region of secure memory.
 *
 * @param   part_base    Kernel virtual address of the partition.
 * @param   offset_bytes Offset from the start of the partition to the plaintext
 *                       data.
 * @param   byte_count   Length of the region (octets).
 * @param   black_data   Physical location to store the encrypted data.
 * @param   IV           Value to use for the Initialization Vector.
 * @param   cypher_mode  Cyphering mode to use, specified by type
 *                       #scc_cypher_mode_t
 *
 * @return  SCC_RET_OK if successful.
 */
	extern scc_return_t
	    scc_encrypt_region(uint32_t part_base, uint32_t offset_bytes,
			       uint32_t byte_count, uint8_t *black_data,
			       uint32_t *IV, scc_cypher_mode_t cypher_mode);

/**
 * Decrypt a region into secure memory
 *
 * @param   part_base    Kernel virtual address of the partition.
 * @param   offset_bytes Offset from the start of the partition to store the
 *                       plaintext data.
 * @param   byte_count   Length of the region (octets).
 * @param   black_data   Physical location of the encrypted data.
 * @param   IV           Value to use for the Initialization Vector.
 * @param   cypher_mode  Cyphering mode to use, specified by type
 *                       #scc_cypher_mode_t
 *
 * @return  SCC_RET_OK if successful.
 */
	extern scc_return_t
	    scc_decrypt_region(uint32_t part_base, uint32_t offset_bytes,
			       uint32_t byte_count, uint8_t *black_data,
			       uint32_t *IV, scc_cypher_mode_t cypher_mode);

/**
 * Retrieve configuration information from the SCC.
 *
 * This function always succeeds.
 *
 * @return   A pointer to the configuration information.  This is a pointer to
 *           static memory and must not be freed.  The values never change, and
 *           the return value will never be null.
 */
	extern scc_config_t *scc_get_configuration(void);

/**
 * Zeroize Red and Black memories of the SCC.  This will start the Zeroizing
 * process.  The routine will return when the memories have zeroized or failed
 * to do so.  The driver will poll waiting for this to occur, so this
 * routine must not be called from interrupt level.  Some future version of
 * driver may elect instead to sleep.
 *
 * @return 0 or error if initialization fails.
 */
	extern scc_return_t scc_zeroize_memories(void);

/**
 * Signal a software alarm to the SCC.  This will take the SCC and other PISA
 * parts out of Secure mode and into Security Failure mode.  The SCC will stay
 * in failed mode until a reboot.
 *
 * @internal
 * If the SCC is not already in fail state, simply write the
 * #SMN_COMMAND_SET_SOFTWARE_ALARM bit in #SMN_COMMAND_REG.  Since there is no
 * reason to wait for the interrupt to bounce back, simply act as though
 * one did.
 */
	extern void scc_set_sw_alarm(void);

/**
 * This routine will register a function to be called should a Security Failure
 * be signalled by the SCC (Security Monitor).
 *
 * The callback function may be called from interrupt level, it may be called
 * from some process' task.  It should therefore not take a long time to
 * perform its operation, and it may not sleep.
 *
 * @param  callback_func  Function pointer to routine which will receive
 *                        notification of the security failure.
 * @return         0 if function was successfully registered, non-zero on
 *                 failure.  See #scc_return_t.
 *
 * @internal
 *  There is a fixed global static array which keeps track of the requests to
 *  monitor the failure.
 *
 *  Add @c callback_func to the first empty slot in #scc_callbacks[].  If there
 *  is no room, return #SCC_RET_TOO_MANY_FUNCTIONS.
 */
	extern scc_return_t scc_monitor_security_failure(void
							 callback_func(void));

/**
 * This routine will deregister a function previously registered with
 * #scc_monitor_security_failure().
 *
 * @param callback_func Function pointer to routine previously registered with
 *                      #scc_stop_monitoring_security_failure().
 */
	extern void scc_stop_monitoring_security_failure(void
							 callback_func(void));

/**
 * Read value from an SCC register.
 * The offset will be checked for validity (range) as well as whether it is
 * accessible (e.g. not busy, not in failed state) at the time of the call.
 *
 * @param[in]   register_offset  The (byte) offset within the SCC block
 *                               of the register to be queried.  See
 *                              @ref scmregs and @ref smnregs.
 * @param[out]  value            Pointer to where value from the register
 *                               should be placed.
 * @return      0 if OK, non-zero on error.  See #scc_return_t.
 *
 * @internal
 *  Verify that the register_offset is a) valid, b) refers to a readable
 *  register, and c) the SCC is in a state which would allow a read of this
 *  register.
 */
	extern scc_return_t scc_read_register(int register_offset,
					      uint32_t *value);

/**
 * Write a new value into an SCC register.
 * The offset will be checked for validity (range) as well as whether it is
 * accessible (e.g. not busy, not in failed state) at the time of the call.
 *
 * @param[in]  register_offset  The (byte) offset within the SCC block
 *                              of the register to be modified.  See
 *                              @ref scmregs and @ref smnregs
 * @param[in]  value            The value to store into the register.
 * @return     0 if OK, non-zero on error.  See #scc_return_t.
 *
 * @internal
 *  Verify that the register_offset is a) valid, b) refers to a writeable
 *  register, and c) the SCC is in a state which would allow a write to this
 *  register.
 */
	extern scc_return_t scc_write_register(int register_offset,
					       uint32_t value);

/**
 * @defgroup scmregs SCM Registers
 *
 * These values are offsets into the SCC for the Secure Memory
 * (SCM) registers.  They are used in the @c register_offset parameter of
 * #scc_read_register() and #scc_write_register().
 */
/** @addtogroup scmregs */
/** @{ */
/** Offset of SCM Version ID Register */
#define SCM_VERSION_REG		0x000
/** Offset of SCM Interrupt Control Register */
#define SCM_INT_CTL_REG		0x008
/** Offset of SCM Status Register */
#define SCM_STATUS_REG		0x00c
/** Offset of SCM Error Status Register */
#define SCM_ERR_STATUS_REG	0x010
/** Offset of SCM Fault Address Register */
#define SCM_FAULT_ADR_REG	0x014
/** Offset of SCM Partition Owners Register */
#define SCM_PART_OWNERS_REG	0x018
/** Offset of SCM Partitions Engaged Register */
#define SCM_PART_ENGAGED_REG	0x01c
/** Offset of SCM Unique Number 0 Register */
#define SCM_UNIQUE_ID0_REG	0x020
/** Offset of SCM Unique Number 1 Register */
#define SCM_UNIQUE_ID1_REG	0x024
/** Offset of SCM Unique Number 2 Register */
#define SCM_UNIQUE_ID2_REG	0x028
/** Offset of SCM Unique Number 3 Register */
#define SCM_UNIQUE_ID3_REG	0x02c
/** Offset of SCM Zeroize Command Register */
#define SCM_ZCMD_REG		0x050
/** Offset of SCM Cipher Command Register */
#define SCM_CCMD_REG		0x054
/** Offset of SCM Cipher Black RAM Start Address Register */
#define SCM_C_BLACK_ST_REG	0x058
/** Offset of SCM Internal Debug Register */
#define SCM_DBG_STATUS_REG	0x05c
/** Offset of SCM Cipher IV 0 Register */
#define SCM_AES_CBC_IV0_REG	0x060
/** Offset of SCM Cipher IV 1 Register */
#define SCM_AES_CBC_IV1_REG	0x064
/** Offset of SCM Cipher IV 2 Register */
#define SCM_AES_CBC_IV2_REG	0x068
/** Offset of SCM Cipher IV 3 Register */
#define SCM_AES_CBC_IV3_REG	0x06c
/** Offset of SCM SMID Partition 0 Register */
#define SCM_SMID0_REG		0x080
/** Offset of SCM Partition 0 Access Permissions Register */
#define SCM_ACC0_REG		0x084
/** Offset of SCM SMID Partition 1 Register */
#define SCM_SMID1_REG		0x088
/** Offset of SCM Partition 1 Access Permissions Register */
#define SCM_ACC1_REG		0x08c
/** Offset of SCM SMID Partition 2 Register */
#define SCM_SMID2_REG		0x090
/** Offset of SCM Partition 2 Access Permissions Register */
#define SCM_ACC2_REG		0x094
/** Offset of SCM SMID Partition 3 Register */
#define SCM_SMID3_REG		0x098
/** Offset of SCM Partition 3 Access Permissions Register */
#define SCM_ACC3_REG		0x09c
/** Offset of SCM SMID Partition 4 Register */
#define SCM_SMID4_REG		0x0a0
/** Offset of SCM Partition 4 Access Permissions Register */
#define SCM_ACC4_REG		0x0a4
/** Offset of SCM SMID Partition 5 Register */
#define SCM_SMID5_REG		0x0a8
/** Offset of SCM Partition 5 Access Permissions Register */
#define SCM_ACC5_REG		0x0ac
/** Offset of SCM SMID Partition 6 Register */
#define SCM_SMID6_REG		0x0b0
/** Offset of SCM Partition 6 Access Permissions Register */
#define SCM_ACC6_REG		0x0b4
/** Offset of SCM SMID Partition 7 Register */
#define SCM_SMID7_REG		0x0b8
/** Offset of SCM Partition 7 Access Permissions Register */
#define SCM_ACC7_REG		0x0bc
/** Offset of SCM SMID Partition 8 Register */
#define SCM_SMID8_REG		0x0c0
/** Offset of SCM Partition 8 Access Permissions Register */
#define SCM_ACC8_REG		0x0c4
/** Offset of SCM SMID Partition 9 Register */
#define SCM_SMID9_REG		0x0c8
/** Offset of SCM Partition 9 Access Permissions Register */
#define SCM_ACC9_REG		0x0cc
/** Offset of SCM SMID Partition 10 Register */
#define SCM_SMID10_REG		0x0d0
/** Offset of SCM Partition 10 Access Permissions Register */
#define SCM_ACC10_REG		0x0d4
/** Offset of SCM SMID Partition 11 Register */
#define SCM_SMID11_REG		0x0d8
/** Offset of SCM Partition 11 Access Permissions Register */
#define SCM_ACC11_REG		0x0dc
/** Offset of SCM SMID Partition 12 Register */
#define SCM_SMID12_REG		0x0e0
/** Offset of SCM Partition 12 Access Permissions Register */
#define SCM_ACC12_REG		0x0e4
/** Offset of SCM SMID Partition 13 Register */
#define SCM_SMID13_REG		0x0e8
/** Offset of SCM Partition 13 Access Permissions Register */
#define SCM_ACC13_REG		0x0ec
/** Offset of SCM SMID Partition 14 Register */
#define SCM_SMID14_REG		0x0f0
/** Offset of SCM Partition 14 Access Permissions Register */
#define SCM_ACC14_REG		0x0f4
/** Offset of SCM SMID Partition 15 Register */
#define SCM_SMID15_REG		0x0f8
/** Offset of SCM Partition 15 Access Permissions Register */
#define SCM_ACC15_REG		0x0fc
/** @} */

/** Number of bytes of register space for the SCM. */
#define SCM_REG_BANK_SIZE	0x100

/** Number of bytes of register space for the SCM. */
#define SCM_REG_BANK_SIZE	0x100

/** Offset of the SMN registers */
#define SMN_ADDR_OFFSET		0x100

/**
 * @defgroup smnregs SMN Registers
 *
 * These values are offsets into the SCC for the Security Monitor
 * (SMN) registers.  They are used in the @c register_offset parameter of the
 * #scc_read_register() and #scc_write_register().
 */
/** @addtogroup smnregs */
/** @{ */
/** Offset of SMN Status Register */
#define SMN_STATUS_REG		(SMN_ADDR_OFFSET+0x00000000)
/** Offset of SMH Command Register */
#define SMN_COMMAND_REG		(SMN_ADDR_OFFSET+0x00000004)
/** Offset of SMH Sequence Start Register */
#define SMN_SEQ_START_REG	(SMN_ADDR_OFFSET+0x00000008)
/** Offset of SMH Sequence End Register */
#define SMN_SEQ_END_REG		(SMN_ADDR_OFFSET+0x0000000c)
/** Offset of SMH Sequence Check Register */
#define SMN_SEQ_CHECK_REG	(SMN_ADDR_OFFSET+0x00000010)
/** Offset of SMH BitBank Count Register */
#define SMN_BB_CNT_REG		(SMN_ADDR_OFFSET+0x00000014)
/** Offset of SMH BitBank Increment Register */
#define SMN_BB_INC_REG		(SMN_ADDR_OFFSET+0x00000018)
/** Offset of SMH BitBank Decrement Register */
#define SMN_BB_DEC_REG		(SMN_ADDR_OFFSET+0x0000001c)
/** Offset of SMH Compare Register */
#define SMN_COMPARE_REG		(SMN_ADDR_OFFSET+0x00000020)
/** Offset of SMH Plaintext Check Register */
#define SMN_PT_CHK_REG		(SMN_ADDR_OFFSET+0x00000024)
/** Offset of SMH Ciphertext Check Register */
#define SMN_CT_CHK_REG		(SMN_ADDR_OFFSET+0x00000028)
/** Offset of SMH Timer Initial Value Register */
#define SMN_TIMER_IV_REG	(SMN_ADDR_OFFSET+0x0000002c)
/** Offset of SMH Timer Control Register */
#define SMN_TIMER_CTL_REG	(SMN_ADDR_OFFSET+0x00000030)
/** Offset of SMH Security Violation Register */
#define SMN_SEC_VIO_REG		(SMN_ADDR_OFFSET+0x00000034)
/** Offset of SMH Timer Register */
#define SMN_TIMER_REG		(SMN_ADDR_OFFSET+0x00000038)
/** Offset of SMH High-Assurance Control Register */
#define SMN_HAC_REG		(SMN_ADDR_OFFSET+0x0000003c)
/** Number of bytes allocated to the SMN registers */
#define SMN_REG_BANK_SIZE	0x40
/** @} */

/** Number of bytes of total register space for the SCC. */
#define SCC_ADDRESS_RANGE	(SMN_ADDR_OFFSET + SMN_REG_BANK_SIZE)

/**
 * @defgroup smnstatusregdefs SMN Status Register definitions (SMN_STATUS)
 */
/** @addtogroup smnstatusregdefs */
/** @{ */
/** SMN version id. */
#define SMN_STATUS_VERSION_ID_MASK        0xfc000000
/**  number of bits to shift #SMN_STATUS_VERSION_ID_MASK to get it to LSB */
#define SMN_STATUS_VERSION_ID_SHIFT       28
/** Illegal bus master access attempted. */
#define SMN_STATUS_ILLEGAL_MASTER         0x01000000
/** Scan mode entered/exited since last reset. */
#define SMN_STATUS_SCAN_EXIT              0x00800000
/** Some security peripheral is initializing */
#define SMN_STATUS_PERIP_INIT             0x00010000
/** Internal error detected in SMN. */
#define SMN_STATUS_SMN_ERROR              0x00004000
/** SMN has an outstanding interrupt. */
#define SMN_STATUS_SMN_STATUS_IRQ         0x00004000
/** Software Alarm was triggered. */
#define SMN_STATUS_SOFTWARE_ALARM         0x00002000
/** Timer has expired. */
#define SMN_STATUS_TIMER_ERROR            0x00001000
/** Plaintext/Ciphertext compare failed. */
#define SMN_STATUS_PC_ERROR               0x00000800
/** Bit Bank detected overflow or underflow */
#define SMN_STATUS_BITBANK_ERROR          0x00000400
/** Algorithm Sequence Check failed. */
#define SMN_STATUS_ASC_ERROR              0x00000200
/** Security Policy Block detected error. */
#define SMN_STATUS_SECURITY_POLICY_ERROR  0x00000100
/** Security Violation Active error. */
#define SMN_STATUS_SEC_VIO_ACTIVE_ERROR   0x00000080
/** Processor booted from internal ROM. */
#define SMN_STATUS_INTERNAL_BOOT          0x00000020
/** SMN's internal state. */
#define SMN_STATUS_STATE_MASK             0x0000001F
/** Number of bits to shift #SMN_STATUS_STATE_MASK to get it to LSB. */
#define SMN_STATUS_STATE_SHIFT            0
/** @} */

/**
 * @defgroup sccscmstates SMN Model Secure State Controller States (SMN_STATE_MASK)
 */
/** @addtogroup sccscmstates */
/** @{ */
/** This is the first state of the SMN after power-on reset  */
#define SMN_STATE_START         0x0
/** The SMN is zeroizing its RAM during reset */
#define SMN_STATE_ZEROIZE_RAM   0x5
/** SMN has passed internal checks, and is waiting for Software check-in */
#define SMN_STATE_HEALTH_CHECK  0x6
/** Fatal Security Violation.  SMN is locked, SCM is inoperative. */
#define SMN_STATE_FAIL          0x9
/** SCC is in secure state.  SCM is using secret key. */
#define SMN_STATE_SECURE        0xA
/** Due to non-fatal error, device is not secure.  SCM is using default key. */
#define SMN_STATE_NON_SECURE    0xC
/** @} */

/** @{ */
/** SCM Status bit: Key Status is Default Key in Use */
#define SCM_STATUS_KST_DEFAULT_KEY	0x80000000
/** SCM Status bit: Key Status is (reserved) */
#define SCM_STATUS_KST_RESERVED1	0x40000000
/** SCM Status bit: Key status is Wrong Key */
#define SCM_STATUS_KST_WRONG_KEY	0x20000000
/** SCM Status bit: Bad Key detected */
#define SCM_STATUS_KST_BAD_KEY	0x10000000
/** SCM Status bit: Error has occurred */
#define SCM_STATUS_ERR		0x00008000
/** SCM Status bit: Monitor State is Failed */
#define SCM_STATUS_MSS_FAIL	0x00004000
/** SCM Status bit: Monitor State is Secure */
#define SCM_STATUS_MSS_SEC	0x00002000
/** SCM Status bit: Secure Storage is Failed */
#define SCM_STATUS_RSS_FAIL	0x00000400
/** SCM Status bit: Secure Storage is Secure */
#define SCM_STATUS_RSS_SEC	0x00000200
/** SCM Status bit: Secure Storage is Initializing */
#define SCM_STATUS_RSS_INIT	0x00000100
/** SCM Status bit: Unique Number Valid */
#define SCM_STATUS_UNV		0x00000080
/** SCM Status bit: Big Endian mode */
#define SCM_STATUS_BIG		0x00000040
/** SCM Status bit: Using Secret Key */
#define SCM_STATUS_USK		0x00000020
/** SCM Status bit: Ram is being blocked */
#define SCM_STATUS_BAR		0x00000010
/** Bit mask of SRS */
#define SCM_STATUS_SRS_MASK	0x0000000F
/** Number of bits to shift SRS to/from MSb */
#define SCM_STATUS_SRS_SHIFT	0
/** @} */

#define SCM_STATUS_SRS_RESET	0x0	/**< Reset, Zeroise All */
#define SCM_STATUS_SRS_READY	0x1	/**< All Ready */
#define SCM_STATUS_SRS_ZBUSY	0x2	/**< Zeroize Busy (Partition Only) */
#define SCM_STATUS_SRS_CBUSY	0x3	/**< Cipher Busy */
#define SCM_STATUS_SRS_ABUSY	0x4	/**< All Busy */
#define SCM_STATUS_SRS_ZDONE	0x5	/**< Zeroize Done, Cipher Ready */
#define SCM_STATUS_SRS_CDONE	0x6	/**< Cipher Done, Zeroize Ready */
#define SCM_STATUS_SRS_ZDONE2	0x7	/**< Zeroize Done, Cipher Busy */
#define SCM_STATUS_SRS_CDONE2	0x8	/**< Cipher Done, Zeroize Busy */
#define SCM_STATUS_SRS_ADONE	0xD	/**< All Done */
#define SCM_STATUS_SRS_FAIL	    0xF	/**< Fail State */


/* Format of the SCM VERSION ID REGISTER */
#define SCM_VER_BPP_MASK    0xFF000000	/**< Bytes Per Partition Mask */
#define SCM_VER_BPP_SHIFT   24		/**< Bytes Per Partition Shift */
#define SCM_VER_BPCB_MASK   0x001F0000	/**< Bytes Per Cipher Block Mask */
#define SCM_VER_BPCB_SHIFT  16		/**< Bytes Per Cipher Block Shift */
#define SCM_VER_NP_MASK     0x0000F000	/**< Number of Partitions Mask */
#define SCM_VER_NP_SHIFT    12		/**< Number of Partitions Shift */
#define SCM_VER_MAJ_MASK    0x00000F00	/**< Major Version Mask */
#define SCM_VER_MAJ_SHIFT   8		/**< Major Version Shift */
#define SCM_VER_MIN_MASK    0x000000FF	/**< Minor Version Mask */
#define SCM_VER_MIN_SHIFT   0		/**< Minor Version Shift */

/**< SCC Hardware version supported by this driver */
#define SCM_MAJOR_VERSION_2 2

/* Format of the SCM ERROR STATUS REGISTER */
#define SCM_ERRSTAT_MID_MASK    0x00F00000  /**< Master ID Mask */
#define SCM_ERRSTAT_MID_SHIFT   20	    /**< Master ID Shift */
#define SCM_ERRSTAT_ILM         0x00080000  /**< Illegal Master */
#define SCM_ERRSTAT_SUP         0x00008000  /**< Supervisor Access */
#define SCM_ERRSTAT_ERC_MASK    0x00000F00  /**< Error Code Mask */
#define SCM_ERRSTAT_ERC_SHIFT   8	    /**< Error Code Shift */
#define SCM_ERRSTAT_SMS_MASK    0x000000F0  /**< Secure Monitor State Mask */
#define SCM_ERRSTAT_SMS_SHIFT   4	    /**< Secure Monitor State Shift */
#define SCM_ERRSTAT_SRS_MASK    0x0000000F  /**< Secure Ram State Mask */
#define SCM_ERRSTAT_SRS_SHIFT   0	    /**< Secure Ram State Shift */

/* SCM ERROR STATUS REGISTER ERROR CODES */
#define SCM_ERCD_UNK_ADDR       0x1 /**< Unknown Address */
#define SCM_ERCD_UNK_CMD        0x2 /**< Unknown Command */
#define SCM_ERCD_READ_PERM      0x3 /**< Read Permission Error */
#define SCM_ERCD_WRITE_PERM     0x4 /**< Write Permission Error */
#define SCM_ERCD_DMA_ERROR      0x5 /**< DMA Error */
#define SCM_ERCD_BLK_OVFL       0x6 /**< Encryption Block Length Overflow */
#define SCM_ERCD_NO_KEY         0x7 /**< Key Not Engaged */
#define SCM_ERCD_ZRZ_OVFL       0x8 /**< Zeroize Command Queue Overflow */
#define SCM_ERCD_CPHR_OVFL      0x9 /**< Cipher Command Queue Overflow */
#define SCM_ERCD_PROC_INTR      0xA /**< Process Interrupted */
#define SCM_ERCD_WRNG_KEY       0xB /**< Wrong Key */
#define SCM_ERCD_DEVICE_BUSY    0xC /**< Device Busy */
#define SCM_ERCD_UNALGN_ADDR    0xD /**< DMA Unaligned Address */

/* Format of the CIPHER COMMAND REGISTER */
#define SCM_CCMD_LENGTH_MASK	0xFFF00000 /**< Cipher Length Mask */
#define SCM_CCMD_LENGTH_SHIFT	20	   /**< Cipher Length Shift */
#define SCM_CCMD_OFFSET_MASK	0x000FFF00 /**< Block Offset Mask */
#define SCM_CCMD_OFFSET_SHIFT	8	   /**< Block Offset Shift */
#define SCM_CCMD_PART_MASK	0x000000F0     /**< Partition Number Mask */
#define SCM_CCMD_PART_SHIFT	4	       /**< Partition Number Shift */
#define SCM_CCMD_CCMD_MASK	0x0000000F     /**< Cipher Command Mask */
#define SCM_CCMD_CCMD_SHIFT	0	       /**< Cipher Command Shift */

/* Values for SCM_CCMD_CCMD field */
#define SCM_CCMD_AES_DEC_ECB 1 /**< Decrypt without Chaining (ECB) */
#define SCM_CCMD_AES_ENC_ECB 3 /**< Encrypt without Chaining (ECB) */
#define SCM_CCMD_AES_DEC_CBC 5 /**< Decrypt with Chaining (CBC) */
#define SCM_CCMD_AES_ENC_CBC 7 /**< Encrypt with Chaining (CBC) */

#define SCM_CCMD_AES     1     /**< Use AES Mode */
#define SCM_CCMD_DEC     0     /**< Decrypt */
#define SCM_CCMD_ENC     2     /**< Encrypt */
#define SCM_CCMD_ECB     0     /**< Perform operation without chaining (ECB) */
#define SCM_CCMD_CBC     4     /**< Perform operation with chaining (CBC) */

/* Format of the ZEROIZE COMMAND REGISTER */
#define SCM_ZCMD_PART_MASK	0x000000F0  /**< Target Partition Mask */
#define SCM_ZCMD_PART_SHIFT	4	    /**< Target Partition Shift */
#define SCM_ZCMD_CCMD_MASK	0x0000000F  /**< Zeroize Command Mask */
#define SCM_ZCMD_CCMD_SHIFT	0	    /**< Zeroize Command Shift */

/* MASTER ACCESS PERMISSIONS REGISTER */
/* Note that API users should use the FSL_PERM_ defines instead of these */
/** SCM Access Permission: Do not zeroize/deallocate partition
	on SMN Fail state */
#define SCM_PERM_NO_ZEROIZE	0x10000000
/** SCM Access Permission: Ignore Supervisor/User mode
	in permission determination */
#define SCM_PERM_HD_SUP_DISABLE	0x00000800
/** SCM Access Permission: Allow Read Access to  Host Domain */
#define SCM_PERM_HD_READ	0x00000400
/** SCM Access Permission: Allow Write Access to  Host Domain */
#define SCM_PERM_HD_WRITE	0x00000200
/** SCM Access Permission: Allow Execute Access to  Host Domain */
#define SCM_PERM_HD_EXECUTE	0x00000100
/** SCM Access Permission: Allow Read Access to Trusted Host Domain */
#define SCM_PERM_TH_READ	0x00000040
/** SCM Access Permission: Allow Write Access to Trusted Host Domain */
#define SCM_PERM_TH_WRITE	0x00000020
/** SCM Access Permission: Allow Read Access to Other/World Domain */
#define SCM_PERM_OT_READ	0x00000004
/** SCM Access Permission: Allow Write Access to Other/World Domain */
#define SCM_PERM_OT_WRITE	0x00000002
/** SCM Access Permission: Allow Execute Access to Other/World Domain */
#define SCM_PERM_OT_EXECUTE	0x00000001
/**< Valid bits that can be set in the Permissions register */
#define SCM_PERM_MASK 0xC0000F67

/* Zeroize Command register definitions */
#define ZCMD_DEALLOC_PART 3	 /**< Deallocate Partition */
#define Z_INT_EN	0x00000002   /**< Zero Interrupt Enable */

/**
 * @defgroup scmpartitionownersregdefs SCM Partition Owners Register
 */
/** @addtogroup scmpartitionownersregdefs */
/** @{ */
/** Number of bits to shift partition number to get to its field. */
#define SCM_POWN_SHIFT   2
/** Mask for a field once the register has been shifted. */
#define SCM_POWN_MASK    3
/** Partition is free */
#define SCM_POWN_PART_FREE       0
/** Partition is unable to be allocated */
#define SCM_POWN_PART_UNUSABLE   1
/** Partition is owned by another master */
#define SCM_POWN_PART_OTHER      2
/** Partition is owned by this master */
#define SCM_POWN_PART_OWNED      3
/** @} */

/**
 * @defgroup smnpartitionsengagedregdefs SCM Partitions Engaged Register
 */
/** @addtogroup smnpartitionsengagedregdefs */
/** @{ */
/** Number of bits to shift partition number to get to its field. */
#define SCM_PENG_SHIFT   1
/** Engaged value for a field once the register has been shifted. */
#define SCM_PENG_ENGAGED    1
/** @} */

/** Number of bytes between each subsequent SMID register */
#define SCM_SMID_WIDTH      8

/**
 * @defgroup smncommandregdefs SMN Command Register Definitions (SMN_COMMAND_REG)
 */
/** @addtogroup smncommandregdefs */
/** @{ */

/** These bits are unimplemented or reserved */
#define SMN_COMMAND_ZEROS_MASK   0xfffffff0
#define SMN_COMMAND_CLEAR_INTERRUPT     0x8 /**< Clear SMN Interrupt */
#define SMN_COMMAND_CLEAR_BIT_BANK      0x4 /**< Clear SMN Bit Bank */
#define SMN_COMMAND_ENABLE_INTERRUPT    0x2 /**< Enable SMN Interrupts */
#define SMN_COMMAND_SET_SOFTWARE_ALARM  0x1 /**< Set Software Alarm */
/** @} */

/**
 * @defgroup smntimercontroldefs SMN Timer Control Register definitions (SMN_TIMER_CONTROL)
 */
/** @addtogroup smntimercontroldefs */
/** @{ */
/** These bits are reserved or zero */
#define SMN_TIMER_CTRL_ZEROS_MASK 0xfffffffc
/** Load the timer from #SMN_TIMER_IV_REG */
#define SMN_TIMER_LOAD_TIMER             0x2
/** Setting to zero stops the Timer */
#define SMN_TIMER_STOP_MASK              0x1
/** Setting this value starts the timer */
#define SMN_TIMER_START_TIMER            0x1
/** @} */

/**
 * @defgroup scmchainmodedefs SCM_CHAINING_MODE_MASK - Bit definitions
 */
/** @addtogroup scmchainmodedefs */
/** @{ */
#define SCM_CBC_MODE            0x2 /**< Cipher block chaining */
#define SCM_ECB_MODE            0x0 /**< Electronic codebook. */
/** @} */

/* Bit definitions in the SCM_CIPHER_MODE_MASK */
/**
 * @defgroup scmciphermodedefs SCM_CIPHER_MODE_MASK - Bit definitions
 */
/** @addtogroup scmciphermodedefs */
/** @{ */
#define SCM_DECRYPT_MODE        0x1 /**< decrypt from black to red memory */
#define SCM_ENCRYPT_MODE        0x0 /**< encrypt from red to black memory */
/** @} */

/**
 * @defgroup smndbgdetdefs SMN Debug Detector Status Register (SCM_DEBUG_DETECT_STAT)
 */
/** @addtogroup smndbgdetdefs */
/** @{ */
#define SMN_DBG_ZEROS_MASK  0xfffff000 /**< These bits are zero or reserved */
#define SMN_DBG_D12             0x0800 /**< Error detected on Debug Port D12 */
#define SMN_DBG_D11             0x0400 /**< Error detected on Debug Port D11 */
#define SMN_DBG_D10             0x0200 /**< Error detected on Debug Port D10 */
#define SMN_DBG_D9              0x0100 /**< Error detected on Debug Port D9 */
#define SMN_DBG_D8              0x0080 /**< Error detected on Debug Port D8 */
#define SMN_DBG_D7              0x0040 /**< Error detected on Debug Port D7 */
#define SMN_DBG_D6              0x0020 /**< Error detected on Debug Port D6 */
#define SMN_DBG_D5              0x0010 /**< Error detected on Debug Port D5 */
#define SMN_DBG_D4              0x0008 /**< Error detected on Debug Port D4 */
#define SMN_DBG_D3              0x0004 /**< Error detected on Debug Port D3 */
#define SMN_DBG_D2              0x0002 /**< Error detected on Debug Port D2 */
#define SMN_DBG_D1              0x0001 /**< Error detected on Debug Port D1 */
/** @} */

/** Mask for the usable bits of the Sequence Start Register
    (#SMN_SEQ_START_REG) */
#define SMN_SEQUENCE_START_MASK    0x0000ffff

/** Mask for the usable bits of the Sequence End Register
    (#SMN_SEQ_END_REG) */
#define SMN_SEQUENCE_END_MASK      0x0000ffff

/** Mask for the usable bits of the Sequence Check Register
    (#SMN_SEQ_CHECK_REG) */
#define SMN_SEQUENCE_CHECK_MASK    0x0000ffff

/** Mask for the usable bits of the Bit Counter Register
    (#SMN_BB_CNT_REG) */
#define SMN_BIT_COUNT_MASK         0x000007ff

/** Mask for the usable bits of the Bit Bank Increment Size Register
    (#SMN_BB_INC_REG) */
#define SMN_BITBANK_INC_SIZE_MASK  0x000007ff

/** Mask for the usable bits of the Bit Bank Decrement Register
    (#SMN_BB_DEC_REG) */
#define SMN_BITBANK_DECREMENT_MASK 0x000007ff

/** Mask for the usable bits of the Compare Size Register
    (#SMN_COMPARE_REG) */
#define SMN_COMPARE_SIZE_MASK      0x0000003f

/*! @} */

#endif				/* SCC_DRIVER_H */
