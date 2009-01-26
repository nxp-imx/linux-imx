/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef RNG_RNGA_H
#define RNG_RNGA_H

/*! @defgroup rngaregs RNGA Registers
 *  @ingroup RNG
 * These are the definitions for the RNG registers and their offsets
 * within the RNG.  They are used in the @c register_offset parameter of
 * #rng_read_register() and #rng_write_register().
 */
/*! @addtogroup rngaregs */
/*! @{ */

/*! Control Register.  See @ref rngacontrolreg.  */
#define RNGA_CONTROL                    0x00
/*! Status Register. See @ref rngastatusreg. */
#define RNGA_STATUS                     0x04
/*! Register for adding to the Entropy of the RNG */
#define RNGA_ENTROPY                    0x08
/*! Register containing latest 32 bits of random value */
#define RNGA_OUTPUT_FIFO                0x0c
/*! Mode Register.  Non-secure mode access only.  See @ref rngmodereg. */
#define RNGA_MODE                       0x10
/*! Verification Control Register.  Non-secure mode access only.  See
 *  @ref rngvfctlreg. */
#define RNGA_VERIFICATION_CONTROL       0x14
/*! Oscillator Control Counter Register.  Non-secure mode access only.
 *  See @ref rngosccntctlreg. */
#define RNGA_OSCILLATOR_CONTROL_COUNTER 0x18
/*! Oscillator 1 Counter Register.  Non-secure mode access only.  See
 *  @ref rngosccntreg. */
#define RNGA_OSCILLATOR1_COUNTER        0x1c
/*! Oscillator 2 Counter Register.  Non-secure mode access only.  See
 *  @ref rngosccntreg. */
#define RNGA_OSCILLATOR2_COUNTER        0x20
/*! Oscillator Counter Status Register.  Non-secure mode access only.  See
 *  @ref rngosccntstatreg. */
#define RNGA_OSCILLATOR_COUNTER_STATUS  0x24
/*! @} */

/*! Total address space of the RNGA, in bytes */
#define RNG_ADDRESS_RANGE 0x28

/*! @defgroup rngacontrolreg RNGA Control Register Definitions
 *  @ingroup RNG
 */
/*! @addtogroup rngacontrolreg */
/*! @{ */
/*! These bits are unimplemented or reserved */
#define RNGA_CONTROL_ZEROS_MASK      0x0fffffe0
/*! 'RNG type' - should be 0 for RNGA */
#define RNGA_CONTROL_RNG_TYPE_MASK   0xf0000000
/*! Number of bits to shift the type to get it to LSB */
#define RNGA_CONTROL_RNG_TYPE_SHIFT  28
/*! Put RNG to sleep */
#define RNGA_CONTROL_SLEEP           0x00000010
/*! Clear interrupt & status */
#define RNGA_CONTROL_CLEAR_INTERRUPT 0x00000008
/*! Mask interrupt generation */
#define RNGA_CONTROL_MASK_INTERRUPTS 0x00000004
/*! Enter into Secure Mode.  Notify SCC of security violation should FIFO
 *  underflow occur. */
#define RNGA_CONTROL_HIGH_ASSURANCE  0x00000002
/*! Load data into FIFO */
#define RNGA_CONTROL_GO              0x00000001
/*! @} */

/*! @defgroup rngastatusreg RNGA Status Register Definitions
 *  @ingroup RNG
 */
/*! @addtogroup rngastatusreg */
/*! @{ */
/*! RNG Oscillator not working */
#define RNGA_STATUS_OSCILLATOR_DEAD         0x80000000
/*! These bits are undefined or reserved */
#define RNGA_STATUS_ZEROS1_MASK             0x7f000000
/*! How big FIFO is, in bytes */
#define RNGA_STATUS_OUTPUT_FIFO_SIZE_MASK   0x00ff0000
/*! How many bits right to shift fifo size to make it LSB */
#define RNGA_STATUS_OUTPUT_FIFO_SIZE_SHIFT  16
/*! How many bytes are currently in the FIFO */
#define RNGA_STATUS_OUTPUT_FIFO_LEVEL_MASK  0x0000ff00
/*! How many bits right to shift fifo level to make it LSB */
#define RNGA_STATUS_OUTPUT_FIFO_LEVEL_SHIFT 8
/*! These bits are undefined or reserved. */
#define RNGA_STATUS_ZEROS2_MASK             0x000000e0
/*! RNG is sleeping. */
#define RNGA_STATUS_SLEEP                   0x00000010
/*! Error detected. */
#define RNGA_STATUS_ERROR_INTERRUPT         0x00000008
/*! FIFO was empty on some read since last status read. */
#define RNGA_STATUS_FIFO_UNDERFLOW          0x00000004
/*! FIFO was empty on most recent read. */
#define RNGA_STATUS_LAST_READ_STATUS        0x00000002
/*! Security violation occurred.  Will only happen in High Assurance mode.  */
#define RNGA_STATUS_SECURITY_VIOLATION      0x00000001
/*! @} */

/*! @defgroup rngmodereg RNG Mode Register Definitions
 *  @ingroup RNG
 */
/*! @addtogroup rngmodereg */
/*! @{ */
/*! These bits are undefined or reserved */
#define RNGA_MODE_ZEROS_MASK                0xfffffffc
/*! RNG is in / put RNG in Oscillator Frequency Test Mode. */
#define RNGA_MODE_OSCILLATOR_FREQ_TEST      0x00000002
/*! Put RNG in verification mode / RNG is in verification mode. */
#define RNGA_MODE_VERIFICATION              0x00000001
/*! @} */

/*! @defgroup rngvfctlreg RNG Verification Control Register Definitions
 *  @ingroup RNG
 */
/*! @addtogroup rngvfctlreg */
/*! @{ */
/*! These bits are undefined or reserved. */
#define RNGA_VFCTL_ZEROS_MASK               0xfffffff8
/*! Reset the shift registers. */
#define RNGA_VFCTL_RESET_SHIFT_REGISTERS    0x00000004
/*! Drive shift registers from system clock. */
#define RNGA_VFCTL_FORCE_SYSTEM_CLOCK       0x00000002
/*! Turn off shift register clocks. */
#define RNGA_VFCTL_SHIFT_CLOCK_OFF          0x00000001
/*! @} */

/*!
 * @defgroup rngosccntctlreg RNG Oscillator Counter Control Register Definitions
 * @ingroup RNG
 */
/*! @addtogroup rngosccntctlreg */
/*! @{ */
/*! These bits are undefined or reserved. */
#define RNGA_OSCCR_ZEROS_MASK               0xfffc0000
/*! Bits containing clock cycle counter */
#define RNGA_OSCCR_CLOCK_CYCLES_MASK        0x0003ffff
/*! Bits to shift right RNG_OSCCR_CLOCK_CYCLES_MASK */
#define RNGA_OSCCR_CLOCK_CYCLES_SHIFT       0
/*! @} */

/*!
 * @defgroup rngosccntreg RNG Oscillator (1 and 2) Counter Register Definitions
 * @ingroup RNG
 */
/*! @addtogroup rngosccntreg */
/*! @{ */
/*! These bits are undefined or reserved. */
#define RNGA_COUNTER_ZEROS_MASK             0xfff00000
/*! Bits containing number of clock pulses received from the oscillator. */
#define RNGA_COUNTER_PULSES_MASK            0x000fffff
/*! Bits right to shift RNG_COUNTER_PULSES_MASK to make it LSB. */
#define RNGA_COUNTER_PULSES_SHIFT           0
/*! @} */

/*!
 * @defgroup rngosccntstatreg RNG Oscillator Counter Status Register Definitions
 * @ingroup RNG
 */
/*! @addtogroup rngosccntstatreg */
/*! @{ */
/*! These bits are undefined or reserved. */
#define RNGA_COUNTER_STATUS_ZEROS_MASK      0xfffffffc
/*! Oscillator 2 has toggled 0x400 times */
#define RNGA_COUNTER_STATUS_OSCILLATOR2     0x00000002
/*! Oscillator 1 has toggled 0x400 times */
#define RNGA_COUNTER_STATUS_OSCILLATOR1     0x00000001
/*! @} */

#endif				/* RNG_RNGA_H */
