/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


#ifndef __DRYICE_H__
#define __DRYICE_H__


/*!
 * @file dryice.h
 * @brief Definition of DryIce API.
 */

/*! @page dryice_api DryIce API
 *
 * Definition of the DryIce API.
 *
 * The DryIce API implements a software interface to the DryIce hardware
 * block. Methods are provided to store, retrieve, generate, and  manage
 * cryptographic keys and to monitor security tamper events.
 *
 * See @ref dryice_api for the DryIce API.
 */

/*!
 * This defines the SCC key length (in bits)
 */
#define SCC_KEY_LEN     168

/*!
 * This defines the maximum key length (in bits)
 */
#define MAX_KEY_LEN     256
#define MAX_KEY_BYTES	((MAX_KEY_LEN) / 8)
#define MAX_KEY_WORDS	((MAX_KEY_LEN) / 32)

/*!
 * @name DryIce Function Flags
 */
/*@{*/
#define DI_FUNC_FLAG_ASYNC       0x01  /*!< do not block */
#define DI_FUNC_FLAG_READ_LOCK   0x02  /*!< set read lock for this resource */
#define DI_FUNC_FLAG_WRITE_LOCK  0x04  /*!< set write lock for resource */
#define DI_FUNC_FLAG_HARD_LOCK   0x08  /*!< locks will be hard (default soft) */
#define DI_FUNC_FLAG_WORD_KEY    0x10  /*!< key provided as 32-bit words */
/*@}*/

/*!
 * @name DryIce Tamper Events
 */
/*@{*/
#define DI_TAMPER_EVENT_WTD   (1 << 23)  /*!< wire-mesh tampering det */
#define DI_TAMPER_EVENT_ETBD  (1 << 22)  /*!< ext tampering det: input B */
#define DI_TAMPER_EVENT_ETAD  (1 << 21)  /*!< ext tampering det: input A */
#define DI_TAMPER_EVENT_EBD   (1 << 20)  /*!< external boot detected */
#define DI_TAMPER_EVENT_SAD   (1 << 19)  /*!< security alarm detected */
#define DI_TAMPER_EVENT_TTD   (1 << 18)  /*!< temperature tampering det */
#define DI_TAMPER_EVENT_CTD   (1 << 17)  /*!< clock tampering det */
#define DI_TAMPER_EVENT_VTD   (1 << 16)  /*!< voltage tampering det */
#define DI_TAMPER_EVENT_MCO   (1 <<  3)  /*!< monotonic counter overflow */
#define DI_TAMPER_EVENT_TCO   (1 <<  2)  /*!< time counter overflow */
/*@}*/

/*!
 * DryIce Key Sources
 */
typedef enum di_key {
	DI_KEY_FK,   /*!< the fused (IIM) key */
	DI_KEY_PK,   /*!< the programmed key */
	DI_KEY_RK,   /*!< the random key */
	DI_KEY_FPK,  /*!< the programmed key XORed with the fused key */
	DI_KEY_FRK,  /*!< the random key XORed with the fused key */
} di_key_t;

/*!
 * DryIce Error Codes
 */
typedef enum dryice_return {
	DI_SUCCESS = 0,  /*!< operation was successful */
	DI_ERR_BUSY,     /*!< device or resource busy */
	DI_ERR_STATE,    /*!< dryice is in incompatible state */
	DI_ERR_INUSE,    /*!< resource is already in use */
	DI_ERR_UNSET,    /*!< resource has not been initialized */
	DI_ERR_WRITE,    /*!< error occurred during register write */
	DI_ERR_INVAL,    /*!< invalid argument */
	DI_ERR_FAIL,     /*!< operation failed */
	DI_ERR_HLOCK,    /*!< resource is hard locked */
	DI_ERR_SLOCK,    /*!< resource is soft locked */
	DI_ERR_NOMEM,    /*!< out of memory */
} di_return_t;

/*!
 * These functions define the DryIce API.
 */

/*!
 * Write a given key to the Programmed Key registers in DryIce, and
 * optionally lock the Programmed Key against either reading or further
 * writing. The value is held until a call to the release_programmed_key
 * interface is made, or until the appropriate HW reset if the write-lock
 * flags are used.  Unused key bits will be zeroed.
 *
 * @param[in]  key_data   A pointer to the key data to be programmed, with
 *                        the most significant byte or word first.  This
 *                        will be interpreted as a byte pointer unless the
 *                        WORD_KEY flag is set, in which case it will be
 *                        treated as a word pointer and the key data will be
 *                        read a word at a time, starting with the MSW.
 *                        When called asynchronously, the data pointed to by
 *                        key_data must persist until the operation completes.
 *
 * @param[in]  key_bits   The number of bits in the key to be stored.
 *                        This must be a multiple of 8 and within the
 *                        range of 0 and MAX_KEY_LEN.
 *
 * @param[in]  flags      This is a bit-wise OR of the flags to be passed
 *                        to the function.  Flags can include:
 *                        ASYNC, READ_LOCK, WRITE_LOCK, HARD_LOCK, and
 *                        WORD_KEY.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, INVAL
 *                        on invalid arguments, INUSE if key has already been
 *                        programmed, STATE if DryIce is in the wrong state,
 *                        HLOCK or SLOCK if the key registers are locked for
 *                        writing, and WRITE if a write error occurs
 *                        (See #di_return_t).
 */
extern di_return_t dryice_set_programmed_key(const void *key_data, int key_bits,
					     int flags);

/*!
 * Read the Programmed Key registers and write the contents into a buffer.
 *
 * @param[out] key_data   A byte pointer to where the key data will be written,
 *                        with the most significant byte being written first.
 *
 * @param[in]  key_bits   The number of bits of the key to be retrieved.
 *                        This must be a multiple of 8 and within the
 *                        range of 0 and MAX_KEY_LEN.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, INVAL
 *                        on invalid arguments, UNSET if key has not been
 *                        programmed, STATE if DryIce is in the wrong state,
 *                        and HLOCK or SLOCK if the key registers are locked for
 *                        reading (See #di_return_t).
 */
extern di_return_t dryice_get_programmed_key(uint8_t *key_data, int key_bits);

/*!
 * Allow the set_programmed_key interface to be used to write a new
 * Programmed Key to DryIce. Note that this interface does not overwrite
 * the value in the Programmed Key registers.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy,
 *                        UNSET if the key has not been previously set, and
 *                        HLOCK or SLOCK if the key registers are locked for
 *                        writing (See #di_return_t).
 */
extern di_return_t dryice_release_programmed_key(void);

/*!
 * Generate and load a new Random Key in DryIce, and optionally lock the
 * Random Key against further change.
 *
 * @param[in]  flags      This is a bit-wise OR of the flags to be passed
 *                        to the function.  Flags can include:
 *                        ASYNC, READ_LOCK, WRITE_LOCK, and HARD_LOCK.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, STATE
 *                        if DryIce is in the wrong state, FAIL if the key gen
 *                        failed, HLOCK or SLOCK if the key registers are
 *                        locked, and WRITE if a write error occurs
 *                        (See #di_return_t).
 */
extern di_return_t dryice_set_random_key(int flags);

/*!
 * Set the key selection in DryIce to determine the key used by an
 * encryption module such as SCC. The selection is held until a call to the
 * Release Selected Key interface is made, or until the appropriate HW
 * reset if the LOCK flags are used.
 *
 * @param[in]   key       The source of the key to be used by the SCC
 *                        (See #di_key_t).
 *
 * @param[in]  flags      This is a bit-wise OR of the flags to be passed
 *                        to the function.  Flags can include:
 *                        ASYNC, WRITE_LOCK, and HARD_LOCK.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, INVAL
 *                        on invalid arguments, INUSE if a selection has already
 *                        been made, STATE if DryIce is in the wrong state,
 *                        HLOCK or SLOCK if the selection register is locked,
 *                        and WRITE if a write error occurs
 */
extern di_return_t dryice_select_key(di_key_t key, int flags);

/*!
 * Check which key will be used in the SCC. This is needed because in some
 * DryIce states, the Key Select Register is overridden by a default value
 * (the Fused/IIM key).
 *
 * @param[out] key        The source of the key that is currently selected for
 *                        use by the SCC.  This may be different from the key
 *                        specified by the dryice_select_key function
 *                        (See #di_key_t).  This value is set even if an error
 *                        code (except for BUSY) is returned.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, STATE if
 *                        DryIce is in the wrong state, INVAL on invalid
 *                        arguments, or UNSET if no key has been selected
 *                        (See #di_return_t).
 */
extern di_return_t dryice_check_key(di_key_t *key);

/*!
 * Allow the dryice_select_key interface to be used to set a new key selection
 * in DryIce. Note that this interface does not overwrite the value in DryIce.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, UNSET
 *                        if the no selection has been made previously, and
 *                        HLOCK or SLOCK if the selection register is locked
 *                        (See #di_return_t).
 */
extern di_return_t dryice_release_key_selection(void);

/*!
 * Returns tamper-detection status bits. Also an optional timestamp when
 * DryIce is in the Non-valid state. If DryIce is not in Failure or Non-valid
 * state, this interface returns a failure code.
 *
 * @param[out] events     This is a bit-wise OR of the following events:
 *                        WTD (Wire Mesh), ETBD (External Tamper B),
 *                        ETAD (External Tamper A), EBD (External Boot),
 *                        SAD (Security Alarm), TTD (Temperature Tamper),
 *                        CTD (Clock Tamper), VTD (Voltage Tamper),
 *                        MCO (Monolithic Counter Overflow), and
 *                        TCO (Time Counter Overflow).
 *
 * @param[out] timestamp  This is the value of the time counter in seconds
 *                        when the tamper occurred.  A timestamp will not be
 *                        returned if a NULL pointer is specified.  If DryIce
 *                        is not in the Non-valid state the time cannot be
 *                        read, so a timestamp of 0 will be returned.
 *
 * @param[in]  flags      This is a bit-wise OR of the flags to be passed
 *                        to the function.  Flags is ignored currently by
 *                        this function.
 *
 * @return                Returns SUCCESS (0), BUSY if DryIce is busy, and
 *                        INVAL on invalid arguments (See #di_return_t).
 */
extern di_return_t
dryice_get_tamper_event(uint32_t *events, uint32_t *timestamp, int flags);

/*!
 * Provide a callback function to be called upon the completion of DryIce calls
 * that are executed asynchronously.
 *
 * @param[in]  func       This is a pointer to a function of type:
 *                        void callback(di_return_t rc, unsigned long cookie)
 *                        The return code of the async function is passed
 *                        back in "rc" along with the cookie provided when
 *                        registering the callback.
 *
 * @param[in]  cookie     This is an "opaque" cookie of type unsigned long that
 *                        is returned on subsequent callbacks.  It may be of any
 *                        value.
 *
 * @return                Returns SUCCESS (0), or BUSY if DryIce is busy
 *                        (See #di_return_t).
 */
extern di_return_t dryice_register_callback(void (*func)(di_return_t rc,
							 unsigned long cookie),
					    unsigned long cookie);

#endif /* __DRYICE_H__ */
