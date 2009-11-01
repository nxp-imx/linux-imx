
/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_SCC_DRIVER_H__
#define __ASM_ARCH_MXC_SCC_DRIVER_H__

/* Start marker for C++ compilers */
#ifdef __cplusplus
extern "C" {
#endif

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

/*!
 * @defgroup MXCSCC SCC Driver
 *
 * @ingroup MXCSECDRVRS
 */

/*!
 * @file arch-mxc/mxc_scc_driver.h
 *
 * @brief (Header file to use the SCC driver.)
 *
 * The SCC driver will only be available to other kernel modules.  That is,
 * there will be no node file in /dev, no way for a user-mode program to access
 * the driver, no way for a user program to access the device directly.
 *
 * With the exception of #scc_monitor_security_failure(), all routines are
 * 'synchronous', i.e. they will not return to their caller until the requested
 * action is complete, or fails to complete.  Some of these functions could
 * take quite a while to perform, depending upon the request.
 *
 * Routines are provided to:
 * @li encrypt or decrypt secrets - #scc_crypt()
 * @li trigger a security-violation alarm - #scc_set_sw_alarm()
 * @li get configuration and version information - #scc_get_configuration()
 * @li zeroize memory - #scc_zeroize_memories()
 * @li Work on wrapped and stored secret values: #scc_alloc_slot(),
 *     #scc_dealloc_slot(), scc_load_slot(), #scc_decrypt_slot(),
 *     #scc_encrypt_slot(), #scc_get_slot_info()

 * @li monitor the Security Failure alarm - #scc_monitor_security_failure()
 * @li stop monitoring Security Failure alarm -
 *     #scc_stop_monitoring_security_failure()
 * @li write registers of the SCC - #scc_write_register()
 * @li read registers of the SCC - #scc_read_register()
 *
 * The driver does not allow "storage" of data in either the Red or Black
 * memories.  Any decrypted secret is returned to the user, and if the user
 * wants to use it at a later point, the encrypted form must again be passed
 * to the driver, and it must be decrypted again.
 *
 * The SCC encrypts and decrypts using Triple DES with an internally stored
 * key.  When the SCC is in Secure mode, it uses its secret, unique-per-chip
 * key.  When it is in Non-Secure mode, it uses a default key.  This ensures
 * that secrets stay secret if the SCC is not in Secure mode.
 *
 * Not all functions that could be provided in a 'high level' manner have been
 * implemented.  Among the missing are interfaces to the ASC/AIC components and
 * the timer functions.  These and other features must be accessed through
 * #scc_read_register() and #scc_write_register(), using the @c \#define values
 * provided.
 *
 * Here is a glossary of acronyms used in the SCC driver documentation:
 * - CBC - Cipher Block Chaining.  A method of performing a block cipher.
 *    Each block is encrypted using some part of the result of the previous
 *    block's encryption.  It needs an 'initialization vector' to seed the
 *    operation.
 * - ECB - Electronic Code Book.  A method of performing a block cipher.
 *    With a given key, a given block will always encrypt to the same value.
 * - DES - Data Encryption Standard.  (8-byte) Block cipher algorithm which
 *    uses 56-bit keys.  In SCC, this key is constant and unique to the device.
 *    SCC uses the "triple DES" form of this algorithm.
 * - AIC - Algorithm Integrity Checker.
 * - ASC - Algorithm Sequence Checker.
 * - SMN - Security Monitor.  The part of the SCC responsible for monitoring
 *    for security problems and notifying the CPU and other PISA components.
 * - SCM - Secure Memory.  The part of the SCC which handles the cryptography.
 * - SCC - Security Controller.  Central security mechanism for PISA.
 * - PISA - Platform-Independent Security Architecture.
 */

/* Temporarily define compile-time flags to make Doxygen happy. */
#ifdef DOXYGEN_HACK
/*! @defgroup scccompileflags SCC Driver compile-time flags
 *
 * These preprocessor flags should be set, if desired, in a makefile so
 * that they show up on the compiler command line.
 */
/*! @addtogroup scccompileflags */

/*! @{ */
/*!
 * Compile-time flag to change @ref smnregs and @ref scmregs
 * offset values for the SCC's implementation on the MX.21 board.
 *
 * This must also be set properly for any code which calls the
 * scc_read_register() or scc_write_register() functions or references the
 * register offsets.
 */
#define TAHITI
/*! @} */
#undef TAHITI

#endif				/* DOXYGEN_HACK */

/*! Major Version of the driver.  Used for
    scc_configuration->driver_major_version */
#define SCC_DRIVER_MAJOR_VERSION_1 1
/*! Old Minor Version of the driver. */
#define SCC_DRIVER_MINOR_VERSION_0 0
/*! Old Minor Version of the driver. */
#define SCC_DRIVER_MINOR_VERSION_4 4
/*! Old Minor Version of the driver. */
#define SCC_DRIVER_MINOR_VERSION_5 5
/*! Old Minor Version of the driver. */
#define SCC_DRIVER_MINOR_VERSION_6 6
/*! Minor Version of the driver.  Used for
    scc_configuration->driver_minor_version */
#define SCC_DRIVER_MINOR_VERSION_8 8


/*!
 * @typedef scc_return_t
 */
/*! Common status return values from SCC driver functions. */
	typedef enum scc_return_t {
		SCC_RET_OK = 0,	/*!< Function succeeded  */
		SCC_RET_FAIL,	/*!< Non-specific failure */
		SCC_RET_VERIFICATION_FAILED,	/*!< Decrypt validation failed */
		SCC_RET_TOO_MANY_FUNCTIONS,	/*!< At maximum registered functions */
		SCC_RET_BUSY,	/*!< SCC is busy and cannot handle request */
		SCC_RET_INSUFFICIENT_SPACE,	/*!< Encryption or decryption failed because
							   @c count_out_bytes says that @c data_out is
							   too small to hold the value. */
	} scc_return_t;

/*!
 * Configuration information about SCC and the driver.
 *
 * This struct/typedef contains information from the SCC and the driver to
 * allow the user of the driver to determine the size of the SCC's memories and
 * the version of the SCC and the driver.
 */
	typedef struct scc_config_t {
		int driver_major_version;	/*!< Major version of the SCC driver code  */
		int driver_minor_version;	/*!< Minor version of the SCC driver code  */
		int scm_version;	/*!< Version from SCM Configuration register */
		int smn_version;	/*!< Version from SMN Status register */
		int block_size_bytes;	/*!< Number of bytes per block of RAM; also
					   block size of the crypto algorithm. */
		int black_ram_size_blocks;	/*!< Number of blocks of Black RAM */
		int red_ram_size_blocks;	/*!< Number of blocks of Red RAM */
	} scc_config_t;

/*!
 * @typedef scc_enc_dec_t
 */
/*!
 * Determine whether SCC will run its cryptographic
 * function as an encryption or decryption.  Used as an argument to
 * #scc_crypt().
 */
	typedef enum scc_enc_dec_t {
		SCC_ENCRYPT,	/*!< Encrypt (from Red to Black) */
		SCC_DECRYPT	/*!< Decrypt (from Black to Red) */
	} scc_enc_dec_t;

/*
 * @typedef scc_crypto_mode_t
 */
/*!
 * Determine whether SCC will run its cryptographic function in ECB (electronic
 * codebook) or CBC (cipher-block chaining) mode.  Used as an argument to
 * #scc_crypt().
 */
	typedef enum scc_crypto_mode_t {
		SCC_ECB_MODE,	/*!< Electronic Codebook Mode */
		SCC_CBC_MODE	/*!< Cipher Block Chaining Mode  */
	} scc_crypto_mode_t;

/*!
 * @typedef scc_verify_t
 */
/*!
 * Tell the driver whether it is responsible for verifying the integrity of a
 * secret.  During an encryption, using other than #SCC_VERIFY_MODE_NONE will
 * cause a check value to be generated and appended to the plaintext before
 * encryption.  During decryption, the check value will be verified after
 * decryption, and then stripped from the message.
 */
	typedef enum scc_verify_t {
		/*! No verification value added or checked.  Input plaintext data must be
		 *  be a multiple of the blocksize (#scc_get_configuration()).  */
		SCC_VERIFY_MODE_NONE,
		/*! Driver will generate/validate a 2-byte CCITT CRC.  Input plaintext will
		   be padded to a multiple of the blocksize, adding 3-10 bytes to the
		   resulting output ciphertext.  Upon decryption, this padding will be
		   stripped, and the CRC will be verified. */
		SCC_VERIFY_MODE_CCITT_CRC
	} scc_verify_t;

/*!
 * Determine if the given credentials match that of the key slot.
 *
 * @param[in]  owner_id     A value which will control access to the slot.
 * @param[in]  slot         Key Slot to query
 * @param[in]  access_len   Length of the key
 *
 * @return     0 on success, non-zero on failure.  See #scc_return_t.
 */
	 scc_return_t
	    scc_verify_slot_access(uint64_t owner_id, uint32_t slot,
				   uint32_t access_len);

/*!
 * Retrieve configuration information from the SCC.
 *
 * This function always succeeds.
 *
 * @return   A pointer to the configuration information.  This is a pointer to
 *           static memory and must not be freed.  The values never change, and
 *           the return value will never be null.
 */
	extern scc_config_t *scc_get_configuration(void);

/*!
 * Zeroize Red and Black memories of the SCC.  This will start the Zeroizing
 * process.  The routine will return when the memories have zeroized or failed
 * to do so.  The driver will poll waiting for this to occur, so this
 * routine must not be called from interrupt level.  Some future version of
 * driver may elect instead to sleep.
 *
 * @return 0 or error if initialization fails.
 */
	extern scc_return_t scc_zeroize_memories(void);

/*!
 * Perform a Triple DES encryption or decryption operation.
 *
 * This routine will cause the SCM to perform an encryption or decryption with
 * its internal key.  If the SCC's #SMN_STATUS register shows that the SCC is
 * in #SMN_STATE_SECURE, then the Secret Key will be used.  If it is
 * #SMN_STATE_NON_SECURE (or health check), then the Default Key will be used.
 *
 * This function will perform in a variety of ways, depending upon the values
 * of @c direction, @c crypto_mode, and @c check_mode.  If
 * #SCC_VERIFY_MODE_CCITT_CRC mode is requested, upon successful completion,
 * the @c count_in_bytes will be different from the returned value of @c
 * count_out_bytes.  This is because the two-byte CRC and some amount of
 * padding (at least one byte) will either be added or stripped.
 *
 * This function will not return until the SCC has performed the operation (or
 * reported failure to do so).  It must therefore not be called from interrupt
 * level.  In the current version, it will poll the SCC for completion.  In
 * future versions, it may sleep.
 *
 * @param[in]    count_in_bytes The number of bytes to move through the crypto
 *                            function.  Must be greater than zero.
 *
 * @param[in]    data_in      Pointer to the array of bytes to be used as input
 *                            to the crypto function.
 *
 * @param[in]    init_vector  Pointer to the block-sized (8 byte) array of
 *                            bytes which form the initialization vector for
 *                            this operation.  A non-null value is required
 *                            when @c crypto_mode has the value #SCC_CBC_MODE;
 *                            the value is ignored in #SCC_ECB_MODE.
 *
 * @param[in]    direction    Direct the driver to perform encryption or
 *                            decryption.
 *
 * @param[in]    crypto_mode  Run the crypto function in ECB or CBC mode.
 *
 * @param[in]    check_mode   During encryption, generate and append a check
 *                            value to the plaintext and pad the resulting
 *                            data.  During decryption, validate the plaintext
 *                            with that check value and remove the padding.
 *
 * @param[in,out] count_out_bytes On input, the number of bytes available for
 *                            copying to @c data_out.  On return, the number of
 *                            bytes copied to @c data_out.
 *
 * @param[out] data_out       Pointer to the array of bytes that are where the
 *                            output of the crypto function are to be placed.
 *                            For encryption, this must be able to hold a
 *                            longer ciphertext than the plaintext message at
 *                            @c data_in.  The driver will append a 'pad' of
 *                            1-8 bytes to the message, and if @c check_mode is
 *                            used, additional bytes may be added, the number
 *                            depending upon the type of check being requested.
 *
 * @return     0 on success, non-zero on failure.  See #scc_return_t.
 *
 * @internal
 * This function will verify SCC state and the functions parameters.  It will
 * acquire the crypto lock, and set up some SCC registers and variables common
 * to encryption and decryption.  A rough check will be made to verify that
 * enough space is available in @c count_out_bytes.  Upon success, either the
 * #scc_encrypt or #scc_decrypt routine will be called to do the actual work.
 * The crypto lock will then be released.
 */
extern scc_return_t scc_crypt(unsigned long count_in_bytes,
				      const uint8_t *data_in,
				      const uint8_t *init_vector,
				      scc_enc_dec_t direction,
				      scc_crypto_mode_t crypto_mode,
				      scc_verify_t check_mode,
				      uint8_t *data_out,
				      unsigned long *count_out_bytes);


/*!
 * Allocate a key slot for a stored key (or other stored value).
 *
 * This feature is to allow decrypted secret values to be kept in RED RAM.
 * This can all visibility of the data only by Sahara.
 *
 * @param   value_size_bytes  Size, in bytes, of RED key/value.  Currently only
 *                            a size up to 32 bytes is supported.
 *
 * @param      owner_id       A value which will control access to the slot.
 *                            It must be passed into to any subsequent calls to
 *                            use the assigned slot.
 *
 * @param[out] slot           The slot number for the key.
 *
 * @return     0 on success, non-zero on failure.  See #scc_return_t.
 */
	extern scc_return_t scc_alloc_slot(uint32_t value_size_bytes,
					   uint64_t owner_id, uint32_t *slot);

/*!
 * Deallocate the key slot of a stored key or secret value.
 *
 * @param      owner_id       The id which owns the @c slot.
 *
 * @param      slot           The slot number for the key.

 * @return     0 on success, non-zero on failure.  See #scc_return_t.
 */
	extern scc_return_t scc_dealloc_slot(uint64_t owner_id, uint32_t slot);

/*!
 * Load a value into a slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param key_data      Data to load into the slot
 * @param key_length    Length, in bytes, of @c key_data to copy to SCC.
 *
 * @return SCC_RET_OK on success.  SCC_RET_FAIL will be returned if slot
 * specified cannot be accessed for any reason, or SCC_RET_INSUFFICIENT_SPACE
 * if @c key_length exceeds the size of the slot.
 */
	extern scc_return_t scc_load_slot(uint64_t owner_id, uint32_t slot,
					  const uint8_t *key_data,
					  uint32_t key_length);
/*!
 * Read a value from a slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param key_length    Length, in bytes, of @c key_data to copy from SCC.
 * @param key_data      Location to write the key
 *
 * @return SCC_RET_OK on success.  SCC_RET_FAIL will be returned if slot
 * specified cannot be accessed for any reason, or SCC_RET_INSUFFICIENT_SPACE
 * if @c key_length exceeds the size of the slot.
 */
	extern scc_return_t scc_read_slot(uint64_t owner_id, uint32_t slot,
					  uint32_t key_length,
					  uint8_t *key_data);

/*!
 * Allocate a key slot to fit the requested size.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param length        Length, in bytes, of @c black_data
 * @param black_data    Location to store result of encrypting RED data in slot
 *
 * @return SCC_RET_OK on success, SCC_RET_FAIL if slot specified cannot be
 *         accessed for any reason.
 */
	extern scc_return_t scc_encrypt_slot(uint64_t owner_id, uint32_t slot,
					     uint32_t length,
					     uint8_t *black_data);

/*!
 * Decrypt some black data and leave result in the slot.
 *
 * @param owner_id      Value of owner of slot
 * @param slot          Handle of slot
 * @param length        Length, in bytes, of @c black_data
 * @param black_data    Location of data to dencrypt and store in slot
 *
 * @return SCC_RET_OK on success, SCC_RET_FAIL if slot specified cannot be
 *         accessed for any reason.
 */
	extern scc_return_t scc_decrypt_slot(uint64_t owner_id, uint32_t slot,
					     uint32_t length,
					     const uint8_t *black_data);

/*!
 * Get attributes of data in RED slot.
 *
 * @param      owner_id         The id which owns the @c slot.
 *
 * @param      slot             The slot number for the key.
 *
 * @param[out] address          Physical address of RED value.
 *
 * @param[out] value_size_bytes Length, in bytes, of RED value,
 *                              or NULL if unneeded..
 *
 * @param[out] slot_size_bytes  Length, in bytes, of slot size,
 *                              or NULL if unneeded..
 *
 * @return     0 on success, non-zero on failure.  See #scc_return_t.
 */
	extern scc_return_t scc_get_slot_info(uint64_t owner_id, uint32_t slot,
					      uint32_t *address,
					      uint32_t *value_size_bytes,
					      uint32_t *slot_size_bytes);

/*!
 * Signal a software alarm to the SCC.  This will take the SCC and other PISA
 * parts out of Secure mode and into Security Failure mode.  The SCC will stay
 * in failed mode until a reboot.
 *
 * @internal
 * If the SCC is not already in fail state, simply write the
 * #SMN_COMMAND_SET_SOFTWARE_ALARM bit in #SMN_COMMAND.  Since there is no
 * reason to wait for the interrupt to bounce back, simply act as though
 * one did.
 */
	extern void scc_set_sw_alarm(void);

/*!
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

/*!
 * This routine will deregister a function previously registered with
 * #scc_monitor_security_failure().
 *
 * @param callback_func Function pointer to routine previously registered with
 *                      #scc_stop_monitoring_security_failure().
 */
	extern void scc_stop_monitoring_security_failure(void
							 callback_func(void));

/*!
 * Read value from an SCC register.
 * The offset will be checked for validity (range) as well as whether it is
 * accessible (e.g. not busy, not in failed state) at the time of the call.
 *
 * @param[in]   register_offset  The (byte) offset within the SCC block
 *                               of the register to be queried. See
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

/*!
 * Write a new value into an SCC register.
 * The offset will be checked for validity (range) as well as whether it is
 * accessible (e.g. not busy, not in failed state) at the time of the call.
 *
 * @param[in]  register_offset  The (byte) offset within the SCC block
 *                              of the register to be modified. See
 *                              @ref scmregs and @ref smnregs.
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

/*
 * NOTE TO MAINTAINERS
 *
 * All of the doxygen comments for the register offset values are in this the
 * following comment section.  Any changes to register names or definitions
 * must be reflected in this section and in both the TAHITI and non-TAHITI
 *version of the memory map.
 */

/*!
 * @defgroup scmregs SCM Registers
 *
 * These values are offsets into the SCC for the Secure Memory
 * (SCM) registers.  They are used in the @c register_offset parameter of
 * #scc_read_register() and #scc_write_register().
 */
/*! @addtogroup scmregs */
/*! @{ */
/*! @def SCM_RED_START
 * Starting block offset in red memory for cipher function. */

/*! @def SCM_BLACK_START
 * Starting block offset in black memory for cipher function. */

/*! @def SCM_LENGTH
 * Number of blocks to process during cipher function */

/*! @def SCM_CONTROL
 * SCM Control register.
 * See @ref scmcontrolregdefs "SCM Control Register definitions" for details.
 */

/*! @def SCM_STATUS
 * SCM Status register.
 * See @ref scmstatusregdefs "SCM Status Register Definitions" for details.
 */

/*! @def SCM_ERROR_STATUS
 * SCM Error Status Register.
 * See @ref scmerrstatdefs "SCM Error Status Register definitions" for
 * details. */

/*! @def SCM_INTERRUPT_CTRL
 * SCM Interrupt Control Register.
 * See @ref scminterruptcontroldefs "SCM Interrupt Control Register definitions"
 * for details.
 */

/*! @def SCM_CONFIGURATION
 * SCM Configuration Register.
 * See @ref scmconfigdefs "SCM Configuration Register Definitions" for
 * details.
 */

/*! @def SCM_INIT_VECTOR_0
 * Upper Half of the Initialization Vector */

/*! @def SCM_INIT_VECTOR_1
 * Lower Half of the Initialization Vector */

/*! @def SCM_RED_MEMORY
 * Starting location of first block of Red memory */

/*! @def SCM_BLACK_MEMORY
 * Starting location of first block of Black memory */

	/*! @} *//* end of SCM group */

/*!
 * @defgroup smnregs SMN Registers
 *
 * These values are offsets into the SCC for the Security Monitor
 * (SMN) registers.  They are used in the @c register_offset parameter of the
 * #scc_read_register() and #scc_write_register().
 */
/*! @addtogroup smnregs */
/*! @{ */
/*! @def SMN_STATUS
 * Status register for SMN.
 * See @ref smnstatusregdefs "SMN Status Register definitions" for further
 * information.
 */

/*! @def SMN_COMMAND
 * Command register for SMN. See
 * @ref smncommandregdefs "Command Register Definitions" for further
 * information.
 */

/*! @def SMN_SEQUENCE_START
 * Sequence Start register for ASC. See #SMN_SEQUENCE_START_MASK
 */

/*! @def SMN_SEQUENCE_END
 * Sequence End register for ASC. See #SMN_SEQUENCE_CHECK_MASK
 */

/*! @def SMN_SEQUENCE_CHECK
 * Sequence Check register for ASC. See #SMN_SEQUENCE_END_MASK
 */

/*! @def SMN_BIT_COUNT
 * Bit Bank Repository for AIC. See #SMN_BIT_COUNT_MASK
 */

/*! @def SMN_BITBANK_INC_SIZE
 * Bit Bank Increment Size for AIC. See #SMN_BITBANK_INC_SIZE_MASK
 */

/*! @def SMN_BITBANK_DECREMENT
 * Bit Bank Decrement for AIC. See #SMN_BITBANK_DECREMENT_MASK
 */

/*! @def SMN_COMPARE_SIZE
 * Compare Size register for Plaintext/Ciphertext checker.  See
 * #SMN_COMPARE_SIZE_MASK */

/*! @def SMN_PLAINTEXT_CHECK
 * Plaintext Check register for Plaintext/Ciphertext checker.
 */

/*! @def SMN_CIPHERTEXT_CHECK
 * Ciphertext Check register for Plaintext/Ciphertext checker.
 */

/*! @def SMN_TIMER_IV
 * Timer Initial Value register
 */

/*! @def SMN_TIMER_CONTROL
 * Timer Control register.
 * See @ref smntimercontroldefs "SMN Timer Control Register definitions".
 */

/*! @def SMN_DEBUG_DETECT_STAT
 * Debug Detector Status Register
 * See @ref smndbgdetdefs "SMN Debug Detector Status Register"for definitions.
 */

/*! @def SMN_TIMER
 * Current value of the Timer Register
 */

	/*! @} *//* end of SMN group */

/*
 * SCC MEMORY MAP
 *
 */

/* SCM registers */
#define SCM_RED_START           0x00000000	/*          read/write       */
#define SCM_BLACK_START         0x00000004	/*          read/write       */
#define SCM_LENGTH              0x00000008	/*          read/write       */
#define SCM_CONTROL             0x0000000C	/*          read/write       */
#define SCM_STATUS              0x00000010	/*          read only        */
#define SCM_ERROR_STATUS        0x00000014	/*          read/write       */
#define SCM_INTERRUPT_CTRL      0x00000018	/*          read/write       */
#define SCM_CONFIGURATION       0x0000001C	/*          read only        */
#define SCM_INIT_VECTOR_0       0x00000020	/*          read/write       */
#define SCM_INIT_VECTOR_1       0x00000024	/*          read/write       */
#define SCM_RED_MEMORY          0x00000400	/*          read/write       */
#define SCM_BLACK_MEMORY        0x00000800	/*          read/write       */

/* SMN Registers */
#define SMN_STATUS              0x00001000	/*          read/write       */
#define SMN_COMMAND             0x00001004	/*          read/write       */
#define SMN_SEQUENCE_START      0x00001008	/*          read/write       */
#define SMN_SEQUENCE_END        0x0000100C	/*          read/write       */
#define SMN_SEQUENCE_CHECK      0x00001010	/*          read/write       */
#define SMN_BIT_COUNT           0x00001014	/*          read only        */
#define SMN_BITBANK_INC_SIZE    0x00001018	/*          read/write       */
#define SMN_BITBANK_DECREMENT   0x0000101C	/*          write only       */
#define SMN_COMPARE_SIZE        0x00001020	/*          read/write       */
#define SMN_PLAINTEXT_CHECK     0x00001024	/*          read/write       */
#define SMN_CIPHERTEXT_CHECK    0x00001028	/*          read/write       */
#define SMN_TIMER_IV            0x0000102C	/*          read/write       */
#define SMN_TIMER_CONTROL       0x00001030	/*          read/write       */
#define SMN_DEBUG_DETECT_STAT   0x00001034	/*          read/write       */
#define SMN_TIMER               0x00001038	/*          read only        */

/*! Total address space of the SCC, in bytes */
#define SCC_ADDRESS_RANGE    0x103c

/*!
 * @defgroup smnstatusregdefs SMN Status Register definitions (SMN_STATUS)
 */
/*! @addtogroup smnstatusregdefs */
/*! @{ */
/*! SMN version id. */
#define SMN_STATUS_VERSION_ID_MASK        0xfc000000
/*!  number of bits to shift #SMN_STATUS_VERSION_ID_MASK to get it to LSB */
#define SMN_STATUS_VERSION_ID_SHIFT       26
/*! Cacheable access to SMN attempted.  */
#define SMN_STATUS_CACHEABLE_ACCESS       0x02000000
/*! Illegal bus master access attempted. */
#define SMN_STATUS_ILLEGAL_MASTER         0x01000000
/*! Scan mode entered/exited since last reset. */
#define SMN_STATUS_SCAN_EXIT              0x00800000
/*! Unaligned access attempted. */
#define SMN_STATUS_UNALIGNED_ACCESS       0x00400000
/*! Bad byte offset access attempted. */
#define SMN_STATUS_BYTE_ACCESS            0x00200000
/*! Illegal address access attempted. */
#define SMN_STATUS_ILLEGAL_ADDRESS        0x00100000
/*! User access attempted. */
#define SMN_STATUS_USER_ACCESS            0x00080000
/*! SCM is using DEFAULT key.  */
#define SMN_STATUS_DEFAULT_KEY            0x00040000
/*! SCM detects weak or bad key.  */
#define SMN_STATUS_BAD_KEY                0x00020000
/*! Illegal access to SCM detected. */
#define SMN_STATUS_ILLEGAL_ACCESS         0x00010000
/*! Internal error detected in SCM. */
#define SMN_STATUS_SCM_ERROR              0x00008000
/*! SMN has an outstanding interrupt. */
#define SMN_STATUS_SMN_STATUS_IRQ         0x00004000
/*! Software Alarm was triggered. */
#define SMN_STATUS_SOFTWARE_ALARM         0x00002000
/*! Timer has expired. */
#define SMN_STATUS_TIMER_ERROR            0x00001000
/*! Plaintext/Ciphertext compare failed. */
#define SMN_STATUS_PC_ERROR               0x00000800
/*! Bit Bank detected overflow or underflow */
#define SMN_STATUS_BITBANK_ERROR          0x00000400
/*! Algorithm Sequence Check failed. */
#define SMN_STATUS_ASC_ERROR              0x00000200
/*! Security Policy Block detected error. */
#define SMN_STATUS_SECURITY_POLICY_ERROR  0x00000100
/*! At least one Debug signal is active. */
#define SMN_STATUS_DEBUG_ACTIVE           0x00000080
/*! SCM failed to zeroize its memory. */
#define SMN_STATUS_ZEROIZE_FAIL           0x00000040
/*! Processor booted from internal ROM. */
#define SMN_STATUS_INTERNAL_BOOT          0x00000020
/*! SMN's internal state. */
#define SMN_STATUS_STATE_MASK             0x0000001F
/*! Number of bits to shift #SMN_STATUS_STATE_MASK to get it to LSB. */
#define SMN_STATUS_STATE_SHIFT            0
/*! @} */

/*!
 * @defgroup sccscmstates SMN Model Secure State Controller States (SMN_STATE_MASK)
 */
/*! @addtogroup sccscmstates */
/*! @{ */
/*! This is the first state of the SMN after power-on reset  */
#define SMN_STATE_START         0x0
/*! The SMN is zeroizing its RAM during reset */
#define SMN_STATE_ZEROIZE_RAM   0x5
/*! SMN has passed internal checks, and is waiting for Software check-in */
#define SMN_STATE_HEALTH_CHECK  0x6
/*! Fatal Security Violation.  SMN is locked, SCM is inoperative. */
#define SMN_STATE_FAIL          0x9
/*! SCC is in secure state.  SCM is using secret key. */
#define SMN_STATE_SECURE        0xA
/*! Due to non-fatal error, device is not secure.  SCM is using default key. */
#define SMN_STATE_NON_SECURE    0xC
/*! @} */

/*!
 * @defgroup scmconfigdefs SCM Configuration Register definitions (SCM_CONFIGURATION)
 **/
/*! @addtogroup scmconfigdefs */
/*! @{ */
/*! Version number of the Secure Memory. */
#define SCM_CFG_VERSION_ID_MASK         0xf8000000
/*! Number of bits to shift #SCM_CFG_VERSION_ID_MASK to get it to LSB. */
#define SCM_CFG_VERSION_ID_SHIFT        27
/*! Version one value for SCC configuration */
#define SCM_VERSION_1    1
/*! Size, in blocks, of Red memory. */
#define SCM_CFG_BLACK_SIZE_MASK         0x07fe0000
/*! Number of bits to shift #SCM_CFG_BLACK_SIZE_MASK to get it to LSB. */
#define SCM_CFG_BLACK_SIZE_SHIFT        17
/*! Size, in blocks, of Black memory. */
#define SCM_CFG_RED_SIZE_MASK           0x0001ff80
/*! Number of bits to shift #SCM_CFG_RED_SIZE_MASK to get it to LSB. */
#define SCM_CFG_RED_SIZE_SHIFT          7
/*! Number of bytes per block. */
#define SCM_CFG_BLOCK_SIZE_MASK         0x0000007f
/*! Number of bits to shift #SCM_CFG_BLOCK_SIZE_MASK to get it to LSB. */
#define SCM_CFG_BLOCK_SIZE_SHIFT        0
/*! @} */

/*!
 * @defgroup smncommandregdefs SMN Command Register Definitions (SMN_COMMAND)
 */
/*! @addtogroup smncommandregdefs */
/*! @{ */
#define SMN_COMMAND_ZEROS_MASK   0xffffff70	/*!< These bits are unimplemented
						   or reserved */
#define SMN_COMMAND_TAMPER_LOCK         0x10 /*!< Lock Tamper Detect Bit */
#define SMN_COMMAND_CLEAR_INTERRUPT     0x8	/*!< Clear SMN Interrupt */
#define SMN_COMMAND_CLEAR_BIT_BANK      0x4	/*!< Clear SMN Bit Bank */
#define SMN_COMMAND_ENABLE_INTERRUPT    0x2	/*!< Enable SMN Interrupts */
#define SMN_COMMAND_SET_SOFTWARE_ALARM  0x1	/*!< Set Software Alarm */
/*! @} */

/*!
 * @defgroup smntimercontroldefs SMN Timer Control Register definitions (SMN_TIMER_CONTROL)
 */
/*! @addtogroup smntimercontroldefs */
/*! @{ */
/*! These bits are reserved or zero */
#define SMN_TIMER_CTRL_ZEROS_MASK 0xfffffffc
/*! Load the timer from #SMN_TIMER_IV */
#define SMN_TIMER_LOAD_TIMER             0x2
/*! Setting to zero stops the Timer */
#define SMN_TIMER_STOP_MASK              0x1
/*! Setting this value starts the timer */
#define SMN_TIMER_START_TIMER            0x1
/*! @} */

/*!
 * @defgroup scminterruptcontroldefs SCM Interrupt Control Register definitions (SCM_INTERRUPT_CTRL)
 *
 * These are the bit definitions for the #SCM_INTERRUPT_CTRL register.
 */
/*! @addtogroup scminterruptcontroldefs */
/*! @{ */
/*! Clear SCM memory */
#define SCM_INTERRUPT_CTRL_ZEROIZE_MEMORY      0x4
/*! Clear outstanding SCM interrupt */
#define SCM_INTERRUPT_CTRL_CLEAR_INTERRUPT     0x2
/*! Inhibit SCM interrupts */
#define SCM_INTERRUPT_CTRL_MASK_INTERRUPTS     0x1
/*! @} */

/*!
 * @defgroup scmcontrolregdefs SCM Control Register definitions (SCM_CONTROL).
 * These values are used with the #SCM_CONTROL register.
 */
/*! @addtogroup scmcontrolregdefs */
/*! @{ */
/*! These bits are zero or reserved */
#define SCM_CONTROL_ZEROS_MASK    0xfffffff8
/*! Setting this will start encrypt/decrypt */
#define SCM_CONTROL_START_CIPHER        0x04
/*! CBC/ECB flag.
 * See @ref scmchainmodedefs "Chaining Mode bit definitions."
 */
#define SCM_CONTROL_CHAINING_MODE_MASK  0x02
/*! Encrypt/decrypt choice.
 * See @ref scmciphermodedefs "Cipher Mode bit definitions." */
#define SCM_CONTROL_CIPHER_MODE_MASK    0x01
/*! @} */

/*!
 * @defgroup scmchainmodedefs  SCM_CHAINING_MODE_MASK - Bit definitions
 */
/*! @addtogroup scmchainmodedefs */
/*! @{ */
#define SCM_CBC_MODE            0x2	/*!< Cipher block chaining */
#define SCM_ECB_MODE            0x0	/*!< Electronic codebook. */
/*! @} */

/* Bit definitions in the SCM_CIPHER_MODE_MASK */
/*!
 * @defgroup scmciphermodedefs SCM_CIPHER_MODE_MASK - Bit definitions
 */
/*! @{ */
#define SCM_DECRYPT_MODE        0x1	/*!< decrypt from black to red memory */
#define SCM_ENCRYPT_MODE        0x0	/*!< encrypt from red to black memory */
/*! @} */

/*!
 * @defgroup scmstatusregdefs  SCM Status Register (SCM_STATUS).
 * Bit and field definitions of the SCM_STATUS register.
 */
/*! @addtogroup scmstatusregdefs */
/*! @{ */
/*! These bits are zero or reserved */
#define SCM_STATUS_ZEROS_MASK        0xffffe000
/*! Ciphering failed due to length error. */
#define SCM_STATUS_LENGTH_ERROR          0x1000
/*! SMN has stopped blocking access to the SCM */
#define SCM_STATUS_BLOCK_ACCESS_REMOVED  0x0800
/*! Ciphering done. */
#define SCM_STATUS_CIPHERING_DONE        0x0400
/*! Zeroizing done. */
#define SCM_STATUS_ZEROIZING_DONE        0x0200
/*! SCM wants attention. Interrupt status is available. */
#define SCM_STATUS_INTERRUPT_STATUS      0x0100
/*! Secret Key is in use. */
#define SCM_STATUS_SECRET_KEY            0x0080
/*! Secret Key is in use.  Deprecated.  Use #SCM_STATUS_SECRET_KEY. */
#define SCM_STATUS_DEFAULT_KEY           0x0080
/*! Internal error to SCM. */
#define SCM_STATUS_INTERNAL_ERROR        0x0040
/*! Secret key is not valid. */
#define SCM_STATUS_BAD_SECRET_KEY        0x0020
/*! Failed to zeroize memory. */
#define SCM_STATUS_ZEROIZE_FAILED        0x0010
/*! SMN is blocking access to Secure Memory. */
#define SCM_STATUS_SMN_BLOCKING_ACCESS   0x0008
/*! SCM is current encrypting or decrypting data. */
#define SCM_STATUS_CIPHERING             0x0004
/*! SCM is currently zeroizing data. */
#define SCM_STATUS_ZEROIZING             0x0002
/*! SCM is busy and access to memory is blocked. */
#define SCM_STATUS_BUSY                  0x0001
/*! @} */

/*!
 * @defgroup scmerrstatdefs SCM Error Status Register (SCM_ERROR_STATUS)
 *
 * These definitions are associated with the SCM Error Status Register
 * (SCM_ERROR_STATUS).
 */
/*! @addtogroup scmerrstatdefs */
/*! @{ */
/*! These bits are zero or reserved */
#define SCM_ERR_ZEROS_MASK      0xffffc000
/*! Cacheable access to SCM was attempted */
#define SCM_ERR_CACHEABLE_ACCESS    0x2000
/*! Access attempted by illegal bus master */
#define SCM_ERR_ILLEGAL_MASTER      0x1000
/*! Unaligned access attempted */
#define SCM_ERR_UNALIGNED_ACCESS    0x0800
/*! Byte or half-word access attempted */
#define SCM_ERR_BYTE_ACCESS         0x0400
/*! Illegal address attempted */
#define SCM_ERR_ILLEGAL_ADDRESS     0x0200
/*! User access attempted */
#define SCM_ERR_USER_ACCESS         0x0100
/*! Access attempted while SCM was using default key */
#define SCM_ERR_SECRET_KEY_IN_USE   0x0080
/*! Access attempted while SCM had internal error */
#define SCM_ERR_INTERNAL_ERROR      0x0040
/*! Access attempted while SCM was detecting Bad Key */
#define SCM_ERR_BAD_SECRET_KEY      0x0020
/*! The SCM failed to Zeroize memory */
#define SCM_ERR_ZEROIZE_FAILED      0x0010
/*! Access attempted while SMN was Blocking Access */
#define SCM_ERR_SMN_BLOCKING_ACCESS 0x0008
/*! Access attempted while SCM was CIPHERING */
#define SCM_ERR_CIPHERING           0x0004
/*! Access attempted while SCM was ZEROIZING */
#define SCM_ERR_ZEROIZING           0x0002
/*! Access attempted while SCM was BUSY */
#define SCM_ERR_BUSY                0x0001
/*! @} */

/*!
 * @defgroup smndbgdetdefs SMN Debug Detector Status Register
 * (SCM_DEBUG_DETECT_STAT)
 */
/*! @addtogroup smndbgdetdefs */
/*! @{ */
#define SMN_DBG_ZEROS_MASK  0xfffff000	/*!< These bits are zero or reserved */
#define SMN_DBG_D12             0x0800	/*!< Error detected on Debug Port D12 */
#define SMN_DBG_D11             0x0400	/*!< Error detected on Debug Port D11 */
#define SMN_DBG_D10             0x0200	/*!< Error detected on Debug Port D10 */
#define SMN_DBG_D9              0x0100	/*!< Error detected on Debug Port D9 */
#define SMN_DBG_D8              0x0080	/*!< Error detected on Debug Port D8 */
#define SMN_DBG_D7              0x0040	/*!< Error detected on Debug Port D7 */
#define SMN_DBG_D6              0x0020	/*!< Error detected on Debug Port D6 */
#define SMN_DBG_D5              0x0010	/*!< Error detected on Debug Port D5 */
#define SMN_DBG_D4              0x0008	/*!< Error detected on Debug Port D4 */
#define SMN_DBG_D3              0x0004	/*!< Error detected on Debug Port D3 */
#define SMN_DBG_D2              0x0002	/*!< Error detected on Debug Port D2 */
#define SMN_DBG_D1              0x0001	/*!< Error detected on Debug Port D1 */
/*! @} */

/*! Mask for the usable bits of the Sequence Start Register
    (#SMN_SEQUENCE_START) */
#define SMN_SEQUENCE_START_MASK    0x0000ffff

/*! Mask for the usable bits of the Sequence End Register
    (#SMN_SEQUENCE_END) */
#define SMN_SEQUENCE_END_MASK      0x0000ffff

/*! Mask for the usable bits of the Sequence Check Register
    (#SMN_SEQUENCE_CHECK) */
#define SMN_SEQUENCE_CHECK_MASK    0x0000ffff

/*! Mask for the usable bits of the Bit Counter Register
    (#SMN_BIT_COUNT) */
#define SMN_BIT_COUNT_MASK         0x000007ff

/*! Mask for the usable bits of the Bit Bank Increment Size Register
    (#SMN_BITBANK_INC_SIZE) */
#define SMN_BITBANK_INC_SIZE_MASK  0x000007ff

/*! Mask for the usable bits of the Bit Bank Decrement Register
    (#SMN_BITBANK_DECREMENT) */
#define SMN_BITBANK_DECREMENT_MASK 0x000007ff

/*! Mask for the usable bits of the Compare Size Register
    (#SMN_COMPARE_SIZE) */
#define SMN_COMPARE_SIZE_MASK      0x0000003f

/* Close out marker for C++ compilers */
#ifdef __cplusplus
}
#endif
#endif				/* __ASM_ARCH_MXC_SCC_DRIVER_H__ */
