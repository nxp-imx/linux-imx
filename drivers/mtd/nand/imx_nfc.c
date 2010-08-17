/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#define DRIVER_VERSION "1.0"

/* Define this macro to enable event reporting. */

#define EVENT_REPORTING

/*
 * For detailed information that will be helpful in understanding this driver,
 * see:
 *
 *     Documentation/imx_nfc.txt
 */

/*
 * Macros that describe NFC hardware have names of the form:
 *
 *     NFC_*
 *
 * Macros that apply only to specific versions of the NFC have names of the
 * following form:
 *
 *     NFC_<M>_<N>_*
 *
 * where:
 *
 *     <M> is the major version of the NFC hardware.
 *     <N> is the minor version of the NFC hardware.
 *
 * The minor version can be 'X', which means that the macro applies to *all*
 * NFCs of the same major version.
 *
 * For NFC versions with only one set of registers, macros that give offsets
 * against the base address have names of the form:
 *
 *     *<RegisterName>_REG_OFF
 *
 * Macros that give the position of a field's LSB within a given register have
 * names of the form:
 *
 *     *<RegisterName>_<FieldName>_POS
 *
 * Macros that mask a field within a given register have names of the form:
 *
 *     *<RegisterName>_<FieldName>_MSK
 */

/*
 * Macro definitions for ALL NFC versions.
 */

#define NFC_MAIN_BUF_SIZE  (512)

/*
 * Macro definitions for version 1.0 NFCs.
 */

#define NFC_1_0_BUF_SIZE_REG_OFF             (0x00)
#define NFC_1_0_BUF_ADDR_REG_OFF             (0x04)
#define NFC_1_0_FLASH_ADDR_REG_OFF           (0x06)
#define NFC_1_0_FLASH_CMD_REG_OFF            (0x08)
#define NFC_1_0_CONFIG_REG_OFF               (0x0A)
#define NFC_1_0_ECC_STATUS_RESULT_REG_OFF    (0x0C)
#define NFC_1_0_RSLTMAIN_AREA_REG_OFF        (0x0E)
#define NFC_1_0_RSLTSPARE_AREA_REG_OFF       (0x10)
#define NFC_1_0_WRPROT_REG_OFF               (0x12)
#define NFC_1_0_UNLOCKSTART_BLKADDR_REG_OFF  (0x14)
#define NFC_1_0_UNLOCKEND_BLKADDR_REG_OFF    (0x16)
#define NFC_1_0_NF_WRPRST_REG_OFF            (0x18)

#define NFC_1_0_CONFIG1_REG_OFF              (0x1A)
#define NFC_1_0_CONFIG1_NF_CE_POS            (7)
#define NFC_1_0_CONFIG1_NF_CE_MSK            (0x1 << 7)

#define NFC_1_0_CONFIG2_REG_OFF              (0x1C)

/*
* Macro definitions for version 2.X NFCs.
*/

#define NFC_2_X_FLASH_ADDR_REG_OFF   (0x06)
#define NFC_2_X_FLASH_CMD_REG_OFF    (0x08)
#define NFC_2_X_CONFIG_REG_OFF       (0x0A)

#define NFC_2_X_WR_PROT_REG_OFF      (0x12)

#define NFC_2_X_NF_WR_PR_ST_REG_OFF  (0x18)

#define NFC_2_X_CONFIG2_REG_OFF      (0x1C)
#define NFC_2_X_CONFIG2_FCMD_POS     (0)
#define NFC_2_X_CONFIG2_FCMD_MSK     (0x1 << 0)
#define NFC_2_X_CONFIG2_FADD_POS     (1)
#define NFC_2_X_CONFIG2_FADD_MSK     (0x1 << 1)
#define NFC_2_X_CONFIG2_FDI_POS      (2)
#define NFC_2_X_CONFIG2_FDI_MSK      (0x1 << 2)
#define NFC_2_X_CONFIG2_FDO_POS      (3)
#define NFC_2_X_CONFIG2_FDO_MSK      (0x7 << 3)
#define NFC_2_X_CONFIG2_INT_POS      (15)
#define NFC_2_X_CONFIG2_INT_MSK      (0x1 << 15)

/*
* Macro definitions for version 2.0 NFCs.
*/

#define NFC_2_0_BUF_ADDR_REG_OFF       (0x04)
#define NFC_2_0_BUF_ADDR_RBA_POS       (0)
#define NFC_2_0_BUF_ADDR_RBA_MSK       (0xf << 0)

#define NFC_2_0_ECC_STATUS_REG_OFF     (0x0C)
#define NFC_2_0_ECC_STATUS_NOSER1_POS  (0)
#define NFC_2_0_ECC_STATUS_NOSER1_MSK  (0xF << 0)
#define NFC_2_0_ECC_STATUS_NOSER2_POS  (4)
#define NFC_2_0_ECC_STATUS_NOSER2_MSK  (0xF << 4)
#define NFC_2_0_ECC_STATUS_NOSER3_POS  (8)
#define NFC_2_0_ECC_STATUS_NOSER3_MSK  (0xF << 8)
#define NFC_2_0_ECC_STATUS_NOSER4_POS  (12)
#define NFC_2_0_ECC_STATUS_NOSER4_MSK  (0xF << 12)

#define NFC_2_0_CONFIG1_REG_OFF        (0x1A)
#define NFC_2_0_CONFIG1_SP_EN_POS      (2)
#define NFC_2_0_CONFIG1_SP_EN_MSK      (0x1 << 2)
#define NFC_2_0_CONFIG1_ECC_EN_POS     (3)
#define NFC_2_0_CONFIG1_ECC_EN_MSK     (0x1 << 3)
#define NFC_2_0_CONFIG1_INT_MSK_POS    (4)
#define NFC_2_0_CONFIG1_INT_MSK_MSK    (0x1 << 4)
#define NFC_2_0_CONFIG1_NF_BIG_POS     (5)
#define NFC_2_0_CONFIG1_NF_BIG_MSK     (0x1 << 5)
#define NFC_2_0_CONFIG1_NFC_RST_POS    (6)
#define NFC_2_0_CONFIG1_NFC_RST_MSK    (0x1 << 6)
#define NFC_2_0_CONFIG1_NF_CE_POS      (7)
#define NFC_2_0_CONFIG1_NF_CE_MSK      (0x1 << 7)
#define NFC_2_0_CONFIG1_ONE_CYLE_POS   (8)
#define NFC_2_0_CONFIG1_ONE_CYLE_MSK   (0x1 << 8)
#define NFC_2_0_CONFIG1_MLC_POS        (9)
#define NFC_2_0_CONFIG1_MLC_MSK        (0x1 << 9)

#define NFC_2_0_UNLOCK_START_REG_OFF   (0x14)
#define NFC_2_0_UNLOCK_END_REG_OFF     (0x16)

/*
* Macro definitions for version 2.1 NFCs.
*/

#define NFC_2_1_BUF_ADDR_REG_OFF        (0x04)
#define NFC_2_1_BUF_ADDR_RBA_POS        (0)
#define NFC_2_1_BUF_ADDR_RBA_MSK        (0x7 << 0)
#define NFC_2_1_BUF_ADDR_CS_POS         (4)
#define NFC_2_1_BUF_ADDR_CS_MSK         (0x3 << 4)

#define NFC_2_1_ECC_STATUS_REG_OFF      (0x0C)
#define NFC_2_1_ECC_STATUS_NOSER1_POS   (0)
#define NFC_2_1_ECC_STATUS_NOSER1_MSK   (0xF << 0)
#define NFC_2_1_ECC_STATUS_NOSER2_POS   (4)
#define NFC_2_1_ECC_STATUS_NOSER2_MSK   (0xF << 4)
#define NFC_2_1_ECC_STATUS_NOSER3_POS   (8)
#define NFC_2_1_ECC_STATUS_NOSER3_MSK   (0xF << 8)
#define NFC_2_1_ECC_STATUS_NOSER4_POS   (12)
#define NFC_2_1_ECC_STATUS_NOSER4_MSK   (0xF << 12)

#define NFC_2_1_CONFIG1_REG_OFF         (0x1A)
#define NFC_2_1_CONFIG1_ECC_MODE_POS    (0)
#define NFC_2_1_CONFIG1_ECC_MODE_MSK    (0x1 << 0)
#define NFC_2_1_CONFIG1_DMA_MODE_POS    (1)
#define NFC_2_1_CONFIG1_DMA_MODE_MSK    (0x1 << 1)
#define NFC_2_1_CONFIG1_SP_EN_POS       (2)
#define NFC_2_1_CONFIG1_SP_EN_MSK       (0x1 << 2)
#define NFC_2_1_CONFIG1_ECC_EN_POS      (3)
#define NFC_2_1_CONFIG1_ECC_EN_MSK      (0x1 << 3)
#define NFC_2_1_CONFIG1_INT_MSK_POS     (4)
#define NFC_2_1_CONFIG1_INT_MSK_MSK     (0x1 << 4)
#define NFC_2_1_CONFIG1_NF_BIG_POS      (5)
#define NFC_2_1_CONFIG1_NF_BIG_MSK      (0x1 << 5)
#define NFC_2_1_CONFIG1_NFC_RST_POS     (6)
#define NFC_2_1_CONFIG1_NFC_RST_MSK     (0x1 << 6)
#define NFC_2_1_CONFIG1_NF_CE_POS       (7)
#define NFC_2_1_CONFIG1_NF_CE_MSK       (0x1 << 7)
#define NFC_2_1_CONFIG1_SYM_POS         (8)
#define NFC_2_1_CONFIG1_SYM_MSK         (0x1 << 8)
#define NFC_2_1_CONFIG1_PPB_POS         (9)
#define NFC_2_1_CONFIG1_PPB_MSK         (0x3 << 9)
#define NFC_2_1_CONFIG1_FP_INT_POS      (11)
#define NFC_2_1_CONFIG1_FP_INT_MSK      (0x1 << 11)

#define NFC_2_1_UNLOCK_START_0_REG_OFF  (0x20)
#define NFC_2_1_UNLOCK_END_0_REG_OFF    (0x22)
#define NFC_2_1_UNLOCK_START_1_REG_OFF  (0x24)
#define NFC_2_1_UNLOCK_END_1_REG_OFF    (0x26)
#define NFC_2_1_UNLOCK_START_2_REG_OFF  (0x28)
#define NFC_2_1_UNLOCK_END_2_REG_OFF    (0x2A)
#define NFC_2_1_UNLOCK_START_3_REG_OFF  (0x2C)
#define NFC_2_1_UNLOCK_END_3_REG_OFF    (0x2E)

/*
* Macro definitions for version 3.X NFCs.
*/

/*
* Macro definitions for version 3.1 NFCs.
*/

#define NFC_3_1_FLASH_ADDR_CMD_REG_OFF          (0x00)
#define NFC_3_1_CONFIG1_REG_OFF                 (0x04)
#define NFC_3_1_ECC_STATUS_RESULT_REG_OFF       (0x08)
#define NFC_3_1_LAUNCH_NFC_REG_OFF              (0x0C)

#define NFC_3_1_WRPROT_REG_OFF                  (0x00)
#define NFC_3_1_WRPROT_UNLOCK_BLK_ADD0_REG_OFF  (0x04)
#define NFC_3_1_CONFIG2_REG_OFF                 (0x14)
#define NFC_3_1_IPC_REG_OFF                     (0x18)

/*
* Macro definitions for version 3.2 NFCs.
*/

#define NFC_3_2_CMD_REG_OFF                (0x00)

#define NFC_3_2_ADD0_REG_OFF               (0x04)
#define NFC_3_2_ADD1_REG_OFF               (0x08)
#define NFC_3_2_ADD2_REG_OFF               (0x0C)
#define NFC_3_2_ADD3_REG_OFF               (0x10)
#define NFC_3_2_ADD4_REG_OFF               (0x14)
#define NFC_3_2_ADD5_REG_OFF               (0x18)
#define NFC_3_2_ADD6_REG_OFF               (0x1C)
#define NFC_3_2_ADD7_REG_OFF               (0x20)
#define NFC_3_2_ADD8_REG_OFF               (0x24)
#define NFC_3_2_ADD9_REG_OFF               (0x28)
#define NFC_3_2_ADD10_REG_OFF              (0x2C)
#define NFC_3_2_ADD11_REG_OFF              (0x30)

#define NFC_3_2_CONFIG1_REG_OFF            (0x34)
#define NFC_3_2_CONFIG1_SP_EN_POS          (0)
#define NFC_3_2_CONFIG1_SP_EN_MSK          (0x1 << 0)
#define NFC_3_2_CONFIG1_NF_CE_POS          (1)
#define NFC_3_2_CONFIG1_NF_CE_MSK          (0x1 << 1)
#define NFC_3_2_CONFIG1_RST_POS            (2)
#define NFC_3_2_CONFIG1_RST_MSK            (0x1 << 2)
#define NFC_3_2_CONFIG1_RBA_POS            (4)
#define NFC_3_2_CONFIG1_RBA_MSK            (0x7 << 4)
#define NFC_3_2_CONFIG1_ITER_POS           (8)
#define NFC_3_2_CONFIG1_ITER_MSK           (0xf << 8)
#define NFC_3_2_CONFIG1_CS_POS             (12)
#define NFC_3_2_CONFIG1_CS_MSK             (0x7 << 12)
#define NFC_3_2_CONFIG1_STATUS_POS         (16)
#define NFC_3_2_CONFIG1_STATUS_MSK         (0xFFFF<<16)

#define NFC_3_2_ECC_STATUS_REG_OFF         (0x38)
#define NFC_3_2_ECC_STATUS_NOBER1_POS      (0)
#define NFC_3_2_ECC_STATUS_NOBER1_MSK      (0xF << 0)
#define NFC_3_2_ECC_STATUS_NOBER2_POS      (4)
#define NFC_3_2_ECC_STATUS_NOBER2_MSK      (0xF << 4)
#define NFC_3_2_ECC_STATUS_NOBER3_POS      (8)
#define NFC_3_2_ECC_STATUS_NOBER3_MSK      (0xF << 8)
#define NFC_3_2_ECC_STATUS_NOBER4_POS      (12)
#define NFC_3_2_ECC_STATUS_NOBER4_MSK      (0xF << 12)
#define NFC_3_2_ECC_STATUS_NOBER5_POS      (16)
#define NFC_3_2_ECC_STATUS_NOBER5_MSK      (0xF << 16)
#define NFC_3_2_ECC_STATUS_NOBER6_POS      (20)
#define NFC_3_2_ECC_STATUS_NOBER6_MSK      (0xF << 20)
#define NFC_3_2_ECC_STATUS_NOBER7_POS      (24)
#define NFC_3_2_ECC_STATUS_NOBER7_MSK      (0xF << 24)
#define NFC_3_2_ECC_STATUS_NOBER8_POS      (28)
#define NFC_3_2_ECC_STATUS_NOBER8_MSK      (0xF << 28)


#define NFC_3_2_STATUS_SUM_REG_OFF         (0x3C)
#define NFC_3_2_STATUS_SUM_NAND_SUM_POS    (0x0)
#define NFC_3_2_STATUS_SUM_NAND_SUM_MSK    (0xFF << 0)
#define NFC_3_2_STATUS_SUM_ECC_SUM_POS     (8)
#define NFC_3_2_STATUS_SUM_ECC_SUM_MSK     (0xFF << 8)

#define NFC_3_2_LAUNCH_REG_OFF             (0x40)
#define NFC_3_2_LAUNCH_FCMD_POS            (0)
#define NFC_3_2_LAUNCH_FCMD_MSK            (0x1 << 0)
#define NFC_3_2_LAUNCH_FADD_POS            (1)
#define NFC_3_2_LAUNCH_FADD_MSK            (0x1 << 1)
#define NFC_3_2_LAUNCH_FDI_POS             (2)
#define NFC_3_2_LAUNCH_FDI_MSK             (0x1 << 2)
#define NFC_3_2_LAUNCH_FDO_POS             (3)
#define NFC_3_2_LAUNCH_FDO_MSK             (0x7 << 3)
#define NFC_3_2_LAUNCH_AUTO_PROG_POS       (6)
#define NFC_3_2_LAUNCH_AUTO_PROG_MSK       (0x1 << 6)
#define NFC_3_2_LAUNCH_AUTO_READ_POS       (7)
#define NFC_3_2_LAUNCH_AUTO_READ_MSK       (0x1 << 7)
#define NFC_3_2_LAUNCH_AUTO_ERASE_POS      (9)
#define NFC_3_2_LAUNCH_AUTO_ERASE_MSK      (0x1 << 9)
#define NFC_3_2_LAUNCH_COPY_BACK0_POS      (10)
#define NFC_3_2_LAUNCH_COPY_BACK0_MSK      (0x1 << 10)
#define NFC_3_2_LAUNCH_COPY_BACK1_POS      (11)
#define NFC_3_2_LAUNCH_COPY_BACK1_MSK      (0x1 << 11)
#define NFC_3_2_LAUNCH_AUTO_STATUS_POS     (12)
#define NFC_3_2_LAUNCH_AUTO_STATUS_MSK     (0x1 << 12)

#define NFC_3_2_WRPROT_REG_OFF             (0x00)
#define NFC_3_2_WRPROT_WPC_POS             (0)
#define NFC_3_2_WRPROT_WPC_MSK             (0x7 << 0)
#define NFC_3_2_WRPROT_CS2L_POS            (3)
#define NFC_3_2_WRPROT_CS2L_MSK            (0x7 << 3)
#define NFC_3_2_WRPROT_BLS_POS             (6)
#define NFC_3_2_WRPROT_BLS_MSK             (0x3 << 6)
#define NFC_3_2_WRPROT_LTS0_POS            (8)
#define NFC_3_2_WRPROT_LTS0_MSK            (0x1 << 8)
#define NFC_3_2_WRPROT_LS0_POS             (9)
#define NFC_3_2_WRPROT_LS0_MSK             (0x1 << 9)
#define NFC_3_2_WRPROT_US0_POS             (10)
#define NFC_3_2_WRPROT_US0_MSK             (0x1 << 10)
#define NFC_3_2_WRPROT_LTS1_POS            (11)
#define NFC_3_2_WRPROT_LTS1_MSK            (0x1 << 11)
#define NFC_3_2_WRPROT_LS1_POS             (12)
#define NFC_3_2_WRPROT_LS1_MSK             (0x1 << 12)
#define NFC_3_2_WRPROT_US1_POS             (13)
#define NFC_3_2_WRPROT_US1_MSK             (0x1 << 13)
#define NFC_3_2_WRPROT_LTS2_POS            (14)
#define NFC_3_2_WRPROT_LTS2_MSK            (0x1 << 14)
#define NFC_3_2_WRPROT_LS2_POS             (15)
#define NFC_3_2_WRPROT_LS2_MSK             (0x1 << 15)
#define NFC_3_2_WRPROT_US2_POS             (16)
#define NFC_3_2_WRPROT_US2_MSK             (0x1 << 16)
#define NFC_3_2_WRPROT_LTS3_POS            (17)
#define NFC_3_2_WRPROT_LTS3_MSK            (0x1 << 17)
#define NFC_3_2_WRPROT_LS3_POS             (18)
#define NFC_3_2_WRPROT_LS3_MSK             (0x1 << 18)
#define NFC_3_2_WRPROT_US3_POS             (19)
#define NFC_3_2_WRPROT_US3_MSK             (0x1 << 19)
#define NFC_3_2_WRPROT_LTS4_POS            (20)
#define NFC_3_2_WRPROT_LTS4_MSK            (0x1 << 20)
#define NFC_3_2_WRPROT_LS4_POS             (21)
#define NFC_3_2_WRPROT_LS4_MSK             (0x1 << 21)
#define NFC_3_2_WRPROT_US4_POS             (22)
#define NFC_3_2_WRPROT_US4_MSK             (0x1 << 22)
#define NFC_3_2_WRPROT_LTS5_POS            (23)
#define NFC_3_2_WRPROT_LTS5_MSK            (0x1 << 23)
#define NFC_3_2_WRPROT_LS5_POS             (24)
#define NFC_3_2_WRPROT_LS5_MSK             (0x1 << 24)
#define NFC_3_2_WRPROT_US5_POS             (25)
#define NFC_3_2_WRPROT_US5_MSK             (0x1 << 25)
#define NFC_3_2_WRPROT_LTS6_POS            (26)
#define NFC_3_2_WRPROT_LTS6_MSK            (0x1 << 26)
#define NFC_3_2_WRPROT_LS6_POS             (27)
#define NFC_3_2_WRPROT_LS6_MSK             (0x1 << 27)
#define NFC_3_2_WRPROT_US6_POS             (28)
#define NFC_3_2_WRPROT_US6_MSK             (0x1 << 28)
#define NFC_3_2_WRPROT_LTS7_POS            (29)
#define NFC_3_2_WRPROT_LTS7_MSK            (0x1 << 29)
#define NFC_3_2_WRPROT_LS7_POS             (30)
#define NFC_3_2_WRPROT_LS7_MSK             (0x1 << 30)
#define NFC_3_2_WRPROT_US7_POS             (31)
#define NFC_3_2_WRPROT_US7_MSK             (0x1 << 31)

#define NFC_3_2_UNLOCK_BLK_ADD0_REG_OFF    (0x04)
#define NFC_3_2_UNLOCK_BLK_ADD0_USBA0_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD0_USBA0_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD0_UEBA0_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD0_UEBA0_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD1_REG_OFF    (0x08)
#define NFC_3_2_UNLOCK_BLK_ADD1_USBA1_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD1_USBA1_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD1_UEBA1_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD1_UEBA1_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD2_REG_OFF    (0x0C)
#define NFC_3_2_UNLOCK_BLK_ADD2_USBA2_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD2_USBA2_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD2_UEBA2_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD2_UEBA2_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD3_REG_OFF    (0x10)
#define NFC_3_2_UNLOCK_BLK_ADD3_USBA3_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD3_USBA3_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD3_UEBA3_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD3_UEBA3_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD4_REG_OFF    (0x14)
#define NFC_3_2_UNLOCK_BLK_ADD4_USBA4_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD4_USBA4_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD4_UEBA4_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD4_UEBA4_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD5_REG_OFF    (0x18)
#define NFC_3_2_UNLOCK_BLK_ADD5_USBA5_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD5_USBA5_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD5_UEBA5_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD5_UEBA5_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD6_REG_OFF    (0x1C)
#define NFC_3_2_UNLOCK_BLK_ADD6_USBA6_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD6_USBA6_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD6_UEBA6_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD6_UEBA6_MSK  (0xFFFF<<16)

#define NFC_3_2_UNLOCK_BLK_ADD7_REG_OFF    (0x20)
#define NFC_3_2_UNLOCK_BLK_ADD7_USBA7_POS  (0)
#define NFC_3_2_UNLOCK_BLK_ADD7_USBA7_MSK  (0xFFFF << 0)
#define NFC_3_2_UNLOCK_BLK_ADD7_UEBA7_POS  (16)
#define NFC_3_2_UNLOCK_BLK_ADD7_UEBA7_MSK  (0xFFFF<<16)

#define NFC_3_2_CONFIG2_REG_OFF            (0x24)
#define NFC_3_2_CONFIG2_PS_POS             (0)
#define NFC_3_2_CONFIG2_PS_MSK             (0x3 << 0)
#define NFC_3_2_CONFIG2_SYM_POS            (2)
#define NFC_3_2_CONFIG2_SYM_MSK            (0x1 << 2)
#define NFC_3_2_CONFIG2_ECC_EN_POS         (3)
#define NFC_3_2_CONFIG2_ECC_EN_MSK         (0x1 << 3)
#define NFC_3_2_CONFIG2_CMD_PHASES_POS     (4)
#define NFC_3_2_CONFIG2_CMD_PHASES_MSK     (0x1 << 4)
#define NFC_3_2_CONFIG2_ADDR_PHASES0_POS   (5)
#define NFC_3_2_CONFIG2_ADDR_PHASES0_MSK   (0x1 << 5)
#define NFC_3_2_CONFIG2_ECC_MODE_POS       (6)
#define NFC_3_2_CONFIG2_ECC_MODE_MSK       (0x1 << 6)
#define NFC_3_2_CONFIG2_PPB_POS            (7)
#define NFC_3_2_CONFIG2_PPB_MSK            (0x3 << 7)
#define NFC_3_2_CONFIG2_EDC_POS            (9)
#define NFC_3_2_CONFIG2_EDC_MSK            (0x7 << 9)
#define NFC_3_2_CONFIG2_ADDR_PHASES1_POS   (12)
#define NFC_3_2_CONFIG2_ADDR_PHASES1_MSK   (0x3 << 12)
#define NFC_3_2_CONFIG2_AUTO_DONE_MSK_POS  (14)
#define NFC_3_2_CONFIG2_AUTO_DONE_MSK_MSK  (0x1 << 14)
#define NFC_3_2_CONFIG2_INT_MSK_POS        (15)
#define NFC_3_2_CONFIG2_INT_MSK_MSK        (0x1 << 15)
#define NFC_3_2_CONFIG2_SPAS_POS           (16)
#define NFC_3_2_CONFIG2_SPAS_MSK           (0xFF << 16)
#define NFC_3_2_CONFIG2_ST_CMD_POS         (24)
#define NFC_3_2_CONFIG2_ST_CMD_MSK         (0xFF << 24)

#define NFC_3_2_CONFIG3_REG_OFF            (0x28)
#define NFC_3_2_CONFIG3_ADD_OP_POS         (0)
#define NFC_3_2_CONFIG3_ADD_OP_MSK         (0x3 << 0)
#define NFC_3_2_CONFIG3_TOO_POS            (2)
#define NFC_3_2_CONFIG3_TOO_MSK            (0x1 << 2)
#define NFC_3_2_CONFIG3_FW_POS             (3)
#define NFC_3_2_CONFIG3_FW_MSK             (0x1 << 3)
#define NFC_3_2_CONFIG3_SB2R_POS           (4)
#define NFC_3_2_CONFIG3_SB2R_MSK           (0x7 << 4)
#define NFC_3_2_CONFIG3_NF_BIG_POS         (7)
#define NFC_3_2_CONFIG3_NF_BIG_MSK         (0x1 << 7)
#define NFC_3_2_CONFIG3_SBB_POS            (8)
#define NFC_3_2_CONFIG3_SBB_MSK            (0x7 << 8)
#define NFC_3_2_CONFIG3_DMA_MODE_POS       (11)
#define NFC_3_2_CONFIG3_DMA_MODE_MSK       (0x1 << 11)
#define NFC_3_2_CONFIG3_NUM_OF_DEVICES_POS (12)
#define NFC_3_2_CONFIG3_NUM_OF_DEVICES_MSK (0x7 << 12)
#define NFC_3_2_CONFIG3_RBB_MODE_POS       (15)
#define NFC_3_2_CONFIG3_RBB_MODE_MSK       (0x1 << 15)
#define NFC_3_2_CONFIG3_FMP_POS            (16)
#define NFC_3_2_CONFIG3_FMP_MSK            (0xF << 16)
#define NFC_3_2_CONFIG3_NO_SDMA_POS        (20)
#define NFC_3_2_CONFIG3_NO_SDMA_MSK        (0x1 << 20)

#define NFC_3_2_IPC_REG_OFF                (0x2C)
#define NFC_3_2_IPC_CREQ_POS               (0)
#define NFC_3_2_IPC_CREQ_MSK               (0x1 << 0)
#define NFC_3_2_IPC_CACK_POS               (1)
#define NFC_3_2_IPC_CACK_MSK               (0x1 << 1)
#define NFC_3_2_IPC_DMA_STATUS_POS         (26)
#define NFC_3_2_IPC_DMA_STATUS_MSK         (0x3 << 26)
#define NFC_3_2_IPC_RB_B_POS               (28)
#define NFC_3_2_IPC_RB_B_MSK               (0x1 << 28)
#define NFC_3_2_IPC_LPS_POS                (29)
#define NFC_3_2_IPC_LPS_MSK                (0x1 << 29)
#define NFC_3_2_IPC_AUTO_PROG_DONE_POS     (30)
#define NFC_3_2_IPC_AUTO_PROG_DONE_MSK     (0x1 << 30)
#define NFC_3_2_IPC_INT_POS                (31)
#define NFC_3_2_IPC_INT_MSK                (0x1 << 31)

#define NFC_3_2_AXI_ERR_ADD_REG_OFF        (0x30)

/**
 * enum override - Choices for overrides.
 *
 * Some functions of this driver can be overriden at run time. This is a
 * convenient enumerated type for all such options.
 */

enum override {
	NEVER         = -1,
	DRIVER_CHOICE =  0,
	ALWAYS        =  1,
};

/**
 * struct physical_geometry - Physical geometry description.
 *
 * This structure describes the physical geometry of the medium.
 *
 * @chip_count:      The number of chips in the medium.
 * @chip_size:       The size, in bytes, of a single chip (excluding the
 *                   out-of-band bytes).
 * @block_size:      The size, in bytes, of a single block (excluding the
 *                   out-of-band bytes).
 * @page_data_size:  The size, in bytes, of the data area in a page (excluding
 *                   the out-of-band bytes).
 * @page_oob_size:   The size, in bytes, of the out-of-band area in a page.
 */

struct physical_geometry {
	unsigned int  chip_count;
	uint64_t      chip_size;
	unsigned int  block_size;
	unsigned int  page_data_size;
	unsigned int  page_oob_size;
};

/**
 * struct nfc_geometry - NFC geometry description.
 *
 * This structure describes the NFC's view of the medium geometry, which may be
 * different from the physical geometry (for example, we treat pages that are
 * physically 2K+112 as if they are 2K+64).
 *
 * @page_data_size:  The size of the data area in a page (excluding the
 *                   out-of-band bytes). This is almost certain to be the same
 *                   as the physical data size.
 * @page_oob_size:   The size of the out-of-band area in a page. This may be
 *                   different from the physical OOB size.
 * @ecc_algorithm:   The name of the ECC algorithm (e.g., "Reed-Solomon" or
 *                   "BCH").
 * @ecc_strength:    A number that describes the strength of the ECC algorithm.
 *                   For example, various i.MX SoC's support ECC-1, ECC-4 or
 *                   ECC-8 of the Reed-Solomon ECC algorithm.
 * @buffer_count:    The number of main/spare buffers used with this geometry.
 * @spare_buf_size:  The number of bytes held in each spare buffer.
 * @spare_buf_spill: The number of extra bytes held in the last spare buffer.
 * @mtd_layout:      The MTD layout that best matches the geometry described by
 *                   the rest of this structure. The logical layer might not use
 *                   this structure, especially when interleaving.
 */

struct nfc_geometry {
	const unsigned int     page_data_size;
	const unsigned int     page_oob_size;
	const char             *ecc_algorithm;
	const int              ecc_strength;
	const unsigned int     buffer_count;
	const unsigned int     spare_buf_size;
	const unsigned int     spare_buf_spill;
	struct nand_ecclayout  mtd_layout;
};

/**
 * struct logical_geometry - Logical geometry description.
 *
 * This structure describes the logical geometry we expose to MTD. This geometry
 * may be different from the physical or NFC geometries, especially when
 * interleaving.
 *
 * @chip_count:      The number of chips in the medium.
 * @chip_size:       The size, in bytes, of a single chip (excluding the
 *                   out-of-band bytes).
 * @usable_size:     The size, in bytes, of the medium that MTD can actually
 *                   use. This may be less than the chip size multiplied by the
 *                   number of chips.
 * @block_size:      The size, in bytes, of a single block (excluding the
 *                   out-of-band bytes).
 * @page_data_size:  The size of the data area in a page (excluding the
 *                   out-of-band bytes).
 * @page_oob_size:   The size of the out-of-band area in a page.
 */

struct logical_geometry {
	unsigned int           chip_count;
	uint32_t               chip_size;
	uint32_t               usable_size;
	unsigned int           block_size;
	unsigned int           page_data_size;
	unsigned int           page_oob_size;
	struct nand_ecclayout  *mtd_layout;
};

/**
 * struct imx_nfc_data - i.MX NFC per-device data.
 *
 * Note that the "device" managed by this driver represents the NAND Flash
 * controller *and* the NAND Flash medium behind it. Thus, the per-device data
 * structure has information about the controller, the chips to which it is
 * connected, and properties of the medium as a whole.
 *
 * @dev:                 A pointer to the owning struct device.
 * @pdev:                A pointer to the owning struct platform_device.
 * @pdata:               A pointer to the device's platform data.
 * @buffers:             A pointer to the NFC buffers.
 * @primary_regs:        A pointer to the NFC primary registers.
 * @secondary_regs:      A pointer to the NFC secondary registers.
 * @clock:               A pointer to the NFC struct clk.
 * @interrupt:           The NFC interrupt number.
 * @physical_geometry:   A description of the medium's physical geometry.
 * @nfc:                 A pointer to the NFC HAL.
 * @nfc_geometry:        A description of the medium geometry as viewed by the
 *                       NFC.
 * @done:                The struct completion we use to handle interrupts.
 * @logical_geometry:    A description of the logical geometry exposed to MTD.
 * @interrupt_override:  Can override how the driver uses interrupts when
 *                       waiting for the NFC.
 * @auto_op_override:    Can override whether the driver uses automatic NFC
 *                       operations.
 * @inject_ecc_error:    Indicates that the driver should inject an ECC error in
 *                       the next read operation that uses ECC. User space
 *                       programs can set this value through the sysfs node of
 *                       the same name. If this value is less than zero, the
 *                       driver will inject an uncorrectable ECC error. If this
 *                       value is greater than zero, the driver will inject that
 *                       number of correctable errors, capped at whatever
 *                       possible maximum currently applies.
 * @current_chip:        The chip currently selected by the NAND Fash MTD HIL.
 *                       A negative value indicates that no chip is selected.
 *                       We use this field to detect when the HIL begins and
 *                       ends essential transactions. This helps us to know when
 *                       we should turn the NFC clock on or off.
 * @command:             The last command the HIL tried to send by calling
 *                       cmdfunc(). Later functions use this information to
 *                       adjust their behavior. The sentinel value ~0 indicates
 *                       no command.
 * @command_is_new:      Indicates the command has just come down to cmdfunc()
 *                       from the HIL and hasn't yet been handled. Other
 *                       functions use this to adjust their behavior.
 * @page_address:        The current page address of interest. For reads, this
 *                       information arrives in calls to cmdfunc(), but we don't
 *                       actually use it until later.
 * @nand:                The data structure that represents this NAND Flash
 *                       medium to the MTD NAND Flash system.
 * @mtd:                 The data structure that represents this NAND Flash
 *                       medium to MTD.
 * @partitions:          A pointer to a set of partitions collected from one of
 *                       several possible sources (e.g., the boot loader, the
 *                       kernel command line, etc.). See the global variable
 *                       partition_source_types for the list of partition
 *                       sources we examine. If this member is NULL, then no
 *                       partitions were discovered.
 * @partition_count:     The number of discovered partitions.
 */

struct imx_nfc_data {

	/* System Interface */
	struct device                 *dev;
	struct platform_device        *pdev;

	/* Platform Configuration */
	struct imx_nfc_platform_data  *pdata;
	void                          *buffers;
	void                          *primary_regs;
	void                          *secondary_regs;
	struct clk                    *clock;
	unsigned int                  interrupt;

	/* Flash Hardware */
	struct physical_geometry      physical_geometry;

	/* NFC HAL and NFC Utilities */
	struct nfc_hal                *nfc;
	struct nfc_geometry           *nfc_geometry;
	struct completion             done;

	/* Medium Abstraction Layer */
	struct logical_geometry       logical_geometry;
	enum override                 interrupt_override;
	enum override                 auto_op_override;
	int                           inject_ecc_error;

	/* MTD Interface Layer */
	int                           current_chip;
	unsigned int                  command;
	int                           command_is_new;
	int                           page_address;

	/* NAND Flash MTD */
	struct nand_chip              nand;

	/* MTD */
	struct mtd_info               mtd;
	struct mtd_partition          *partitions;
	unsigned int                  partition_count;

};

/**
 * struct nfc_hal - i.MX NFC HAL
 *
 * This structure embodies an abstract interface to the underlying NFC hardware.
 *
 * @major_version:       The major version number of the NFC to which this
 *                       structure applies.
 * @minor_version:       The minor version number of the NFC to which this
 *                       structure applies.
 * @max_chip_count:      The maximum number of chips the NFC can possibly
 *                       support. This may *not* be the actual number of chips
 *                       currently connected. This value is constant for NFC's
 *                       of a given version.
 * @max_buffer_count:    The number of main/spare buffers available in the NFC's
 *                       memory. This value is constant for NFC's of a given
 *                       version.
 * @spare_buf_stride:    The stride, in bytes, from the beginning of one spare
 *                       buffer to the beginning of the next one. This value is
 *                       constant for NFC's of a given version.
 * @has_secondary_regs:  Indicates if the NFC has a secondary register set that
 *                       must be mapped in.
 * @can_be_symmetric:    Indicates if the NFC supports a "symmetric" clock. When
 *                       the clock is "symmetric," the hardware waits one NFC
 *                       clock for every read/write cycle. When the clock is
 *                       "asymmetric," the hardware waits two NFC clocks for
 *                       every read/write cycle.
 * @init:                Initializes the NFC and any version-specific data
 *                       structures. This function will be called after
 *                       everything has been set up for communication with the
 *                       NFC itself, but before the platform has set up off-chip
 *                       communication. Thus, this function must not attempt to
 *                       communicate with the NAND Flash hardware. A non-zero
 *                       return value indicates failure.
 * @set_geometry:        Based on the physical geometry, this function selects
 *                       an NFC geometry structure and configures the NFC
 *                       hardware to match. A  non-zero return value indicates
 *                       failure.
 * @exit:                Shuts down the NFC and cleans up version-specific data
 *                       structures. This function will be called after the
 *                       platform has shut down off-chip communication but while
 *                       communication with the NFC still works.
 * @set_closest_cycle:   Configures the hardware to make the NAND Flash bus
 *                       cycle period as close as possible to the given cycle
 *                       period. This function is called during boot up and may
 *                       assume that, at the time it's called, the parent clock
 *                       is running at the highest rate it will ever run. Thus,
 *                       this function need never worry that the NAND Flash bus
 *                       will run faster and potentially make it impossible to
 *                       communicate with the NAND Flash device -- it will only
 *                       run slower.
 * @mask_interrupt:      Masks the NFC's interrupt.
 * @unmask_interrupt:    Unmasks the NFC's interrupt.
 * @clear_interrupt:     Clears the NFC's interrupt.
 * @is_interrupting:     Returns true if the NFC is interrupting.
 * @is_ready:            Returns true if all the chips in the medium are ready.
 *                       This member may be set to NULL, which indicates that
 *                       the underlying NFC hardware doesn't expose  ready/busy
 *                       signals.
 * @set_force_ce:        If passed true, forces the hardware chip enable signal
 *                       for the current chip to be asserted always. If passed
 *                       false, causes the chip enable signal to be asserted
 *                       only during I/O.
 * @set_ecc:             Sets ECC on or off.
 * @get_ecc_status:      Examines the hardware ECC status and returns:
 *                           == 0  =>  No errors.
 *                           >  0  =>  The number of corrected errors.
 *                           <  0  =>  There were uncorrectable errors.
 * @get_symmetric:       Gets the current symmetric clock setting. For versions
 *                       that don't support symmetric clocks, this function
 *                       always returns false.
 * @set_symmetric:       For versions that support symmetric clocks, sets
 *                       whether or not the clock is symmetric.
 * @select_chip:         Selects the current chip.
 * @command_cycle:       Sends a command code and then returns immediately
 *                       *without* waiting for the NFC to finish.
 * @write_cycle:         Applies a single write cycle to the current chip,
 * 			 sending the given byte, and waiting for the NFC to
 *                       finish.
 * @read_cycle:          Applies a single read cycle to the current chip and
 *                       returns the result, necessarily waiting for the NFC to
 *                       finish. The width of the result is the same as the
 *                       width of the Flash bus.
 * @read_page:           Applies read cycles to the current chip to read an
 *                       entire page into the NFC. Note that ECC is enabled or
 *                       disabled with the set_ecc function pointer (see above).
 *                       This function waits for the NFC to finish before
 *                       returning.
 * @send_page:           Applies write cycles to send an entire page from the
 *                       NFC to the current chip. Note that ECC is enabled or
 *                       disabled with the set_ecc function pointer (see above).
 *                       This function waits for the NFC to finish before
 *                       returning.
 * @start_auto_read:     Starts an automatic read operation. A NULL pointer
 *                       indicates automatic read operations aren't available
 *                       with this NFC version.
 * @wait_for_auto_read:  Blocks until an automatic read operation is ready for
 *                       the CPU to copy a page out of the NFC.
 * @resume_auto_read:    Resumes an automatic read operation after the CPU has
 *                       copied a page out.
 * @start_auto_write:    Starts an automatic write operation. A NULL pointer
 *                       indicates automatic write operations aren't available
 *                       with this NFC version.
 * @wait_for_auto_write: Blocks until an automatic write operation is ready for
 *                       the CPU to copy a page into the NFC.
 * @start_auto_erase:    Starts an automatic erase operation. A NULL pointer
 *                       indicates automatic erase operations aren't available
 *                       with this NFC version.
 */

struct nfc_hal {
	const unsigned int  major_version;
	const unsigned int  minor_version;
	const unsigned int  max_chip_count;
	const unsigned int  max_buffer_count;
	const unsigned int  spare_buf_stride;
	const int           has_secondary_regs;
	const int           can_be_symmetric;
	int       (*init)               (struct imx_nfc_data *);
	int       (*set_geometry)       (struct imx_nfc_data *);
	void      (*exit)               (struct imx_nfc_data *);
	int       (*set_closest_cycle)  (struct imx_nfc_data *, unsigned ns);
	void      (*mask_interrupt)     (struct imx_nfc_data *);
	void      (*unmask_interrupt)   (struct imx_nfc_data *);
	void      (*clear_interrupt)    (struct imx_nfc_data *);
	int       (*is_interrupting)    (struct imx_nfc_data *);
	int       (*is_ready)           (struct imx_nfc_data *);
	void      (*set_force_ce)       (struct imx_nfc_data *, int on);
	void      (*set_ecc)            (struct imx_nfc_data *, int on);
	int       (*get_ecc_status)     (struct imx_nfc_data *);
	int       (*get_symmetric)      (struct imx_nfc_data *);
	void      (*set_symmetric)      (struct imx_nfc_data *, int on);
	void      (*select_chip)        (struct imx_nfc_data *, int chip);
	void      (*command_cycle)      (struct imx_nfc_data *, unsigned cmd);
	void      (*write_cycle)        (struct imx_nfc_data *, unsigned byte);
	unsigned  (*read_cycle)         (struct imx_nfc_data *);
	void      (*read_page)          (struct imx_nfc_data *);
	void      (*send_page)          (struct imx_nfc_data *);
	int       (*start_auto_read)    (struct imx_nfc_data *,
		unsigned start, unsigned count, unsigned column, unsigned page);
	int       (*wait_for_auto_read) (struct imx_nfc_data *);
	int       (*resume_auto_read)   (struct imx_nfc_data *);
	int       (*start_auto_write)   (struct imx_nfc_data *,
		unsigned start, unsigned count, unsigned column, unsigned page);
	int       (*wait_for_auto_write)(struct imx_nfc_data *);
	int       (*start_auto_erase)   (struct imx_nfc_data *,
		unsigned start, unsigned count, unsigned page);
};

/*
 * This variable controls whether or not probing is enabled. If false, then
 * the driver will refuse to probe. The "enable" module parameter controls the
 * value of this variable.
 */

static int imx_nfc_module_enable = true;

#ifdef EVENT_REPORTING

/*
 * This variable controls whether the driver reports event information by
 * printing to the console. The "report_events" module parameter controls the
 * value of this variable.
 */

static int imx_nfc_module_report_events; /* implicitly initialized false*/

#endif

/*
 * This variable potentially overrides the driver's choice to interleave. The
 * "interleave_override" module parameter controls the value of this variable.
 */

static enum override imx_nfc_module_interleave_override = DRIVER_CHOICE;

/*
 * When set, this variable forces the driver to use the bytewise copy functions
 * to get data in and out of the NFC. This is intended for testing, not typical
 * use.
 */

static int imx_nfc_module_force_bytewise_copy; /* implicitly initialized false*/

/*
 * The list of algorithms we use to get partition information from the
 * environment.
 */

#ifdef CONFIG_MTD_PARTITIONS
static const char *partition_source_types[] = { "cmdlinepart", NULL };
#endif

/*
 * The following structures describe the NFC geometries we support.
 *
 * Notice that pieces of some structure definitions are commented out and edited
 * because various parts of the MTD system can't handle the way our hardware
 * formats the out-of-band area.
 *
 * Here are the problems:
 *
 * - struct nand_ecclayout expects no more than 64 ECC bytes.
 *
 *     The eccpos member of struct nand_ecclayout can't hold more than 64 ECC
 *     byte positions. Some of our formats have more so, unedited, they won't
 *     even compile. We comment out all ECC byte positions after the 64th one.
 *
 * - struct nand_ecclayout expects no more than 8 free spans.
 *
 *     The oobfree member of struct nand_ecclayout can't hold more than 8 free
 *     spans. Some of our formats have more so, unedited, they won't even
 *     compile. We comment out all free spans after the eighth one.
 *
 * - The MEMGETOOBSEL command in the mtdchar driver.
 *
 *     The mtdchar ioctl command MEMGETOOBSEL checks the number of ECC bytes
 *     against the length of the eccpos array in struct nand_oobinfo
 *     (see include/mtd/mtd-abi.h). This array can handle up to 32 ECC bytes,
 *     but some of our formats have more.
 *
 *     To make this work, we cap the value assigned to eccbytes at 32.
 *
 *     Notice that struct nand_oobinfo, used by mtdchar, is *different* from the
 *     struct nand_ecclayout that MTD uses internally. The latter structure
 *     can accomodate up to 64 ECC byte positions. Thus, we declare up to 64
 *     ECC byte positions here, even if the advertised number of ECC bytes is
 *     less.
 *
 *     This command is obsolete and, if no one used it, we wouldn't care about
 *     this problem. Unfortunately The nandwrite program uses it, and everyone
 *     expects nandwrite to work (it's how everyone usually lays down their
 *     JFFS2 file systems).
 */

static struct nfc_geometry  nfc_geometry_512_16_RS_ECC1 = {
	.page_data_size  = 512,
	.page_oob_size   = 16,
	.ecc_algorithm   = "Reed-Solomon",
	.ecc_strength    = 1,
	.buffer_count    = 1,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 5,
		.eccpos   = {6, 7, 8, 9, 10},
		.oobavail = 11,
		.oobfree  = { {0, 6}, {11, 5} },
	}
};

static struct nfc_geometry  nfc_geometry_512_16_RS_ECC4 = {
	.page_data_size  = 512,
	.page_oob_size   = 16,
	.ecc_algorithm   = "Reed-Solomon",
	.ecc_strength    = 4,
	.buffer_count    = 1,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 9,
		.eccpos   = {7, 8, 9, 10, 11, 12, 13, 14, 15},
		.oobavail = 7,
		.oobfree  = { {0, 7} },
	}
};

static struct nfc_geometry  nfc_geometry_512_16_BCH_ECC4 = {
	.page_data_size  = 512,
	.page_oob_size   = 16,
	.ecc_algorithm   = "BCH",
	.ecc_strength    = 4,
	.buffer_count    = 1,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 8,
		.eccpos   = {8, 9, 10, 11, 12, 13, 14, 15},
		.oobavail = 8,
		.oobfree  = { {0, 8} },
	}
};

static struct nfc_geometry  nfc_geometry_2K_64_RS_ECC4 = {
	.page_data_size  = 2048,
	.page_oob_size   = 64,
	.ecc_algorithm   = "Reed-Solomon",
	.ecc_strength    = 4,
	.buffer_count    = 4,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 32 /*9 * 4*/, /* See notes above. */
		.eccpos   = {

			(0*16)+7 , (0*16)+8 , (0*16)+9 , (0*16)+10, (0*16)+11,
			(0*16)+12, (0*16)+13, (0*16)+14, (0*16)+15,

			(1*16)+7 , (1*16)+8 , (1*16)+9 , (1*16)+10, (1*16)+11,
			(1*16)+12, (1*16)+13, (1*16)+14, (1*16)+15,

			(2*16)+7 , (2*16)+8 , (2*16)+9 , (2*16)+10, (2*16)+11,
			(2*16)+12, (2*16)+13, (2*16)+14, (2*16)+15,

			(3*16)+7 , (3*16)+8 , (3*16)+9 , (3*16)+10, (3*16)+11,
			(3*16)+12, (3*16)+13, (3*16)+14, (3*16)+15,

		},
		.oobavail = 7 * 4,
		.oobfree  = {
			{(0*16)+0, 7},
			{(1*16)+0, 7},
			{(2*16)+0, 7},
			{(3*16)+0, 7},
		},
	}
};

static struct nfc_geometry  nfc_geometry_2K_64_BCH_ECC4 = {
	.page_data_size  = 2048,
	.page_oob_size   = 64,
	.ecc_algorithm   = "BCH",
	.ecc_strength    = 4,
	.buffer_count    = 4,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 8 * 4,
		.eccpos   = {

			(0*16)+8 , (0*16)+9 , (0*16)+10, (0*16)+11,
			(0*16)+12, (0*16)+13, (0*16)+14, (0*16)+15,

			(1*16)+8 , (1*16)+9 , (1*16)+10, (1*16)+11,
			(1*16)+12, (1*16)+13, (1*16)+14, (1*16)+15,

			(2*16)+8 , (2*16)+9 , (2*16)+10, (2*16)+11,
			(2*16)+12, (2*16)+13, (2*16)+14, (2*16)+15,

			(3*16)+8 , (3*16)+9 , (3*16)+10, (3*16)+11,
			(3*16)+12, (3*16)+13, (3*16)+14, (3*16)+15,

		},
		.oobavail = 8 * 4,
		.oobfree  = {
			{(0*16)+0, 8},
			{(1*16)+0, 8},
			{(2*16)+0, 8},
			{(3*16)+0, 8},
		},
	}
};

static struct nfc_geometry  nfc_geometry_4K_128_BCH_ECC4 = {
	.page_data_size  = 4096,
	.page_oob_size   = 128,
	.ecc_algorithm   = "BCH",
	.ecc_strength    = 4,
	.buffer_count    = 8,
	.spare_buf_size  = 16,
	.spare_buf_spill = 0,
	.mtd_layout = {
		.eccbytes = 8 * 8,
		.eccpos   = {

			(0*16)+8 , (0*16)+9 , (0*16)+10, (0*16)+11,
			(0*16)+12, (0*16)+13, (0*16)+14, (0*16)+15,

			(1*16)+8 , (1*16)+9 , (1*16)+10, (1*16)+11,
			(1*16)+12, (1*16)+13, (1*16)+14, (1*16)+15,

			(2*16)+8 , (2*16)+9 , (2*16)+10, (2*16)+11,
			(2*16)+12, (2*16)+13, (2*16)+14, (2*16)+15,

			(3*16)+8 , (3*16)+9 , (3*16)+10, (3*16)+11,
			(3*16)+12, (3*16)+13, (3*16)+14, (3*16)+15,

			(4*16)+8 , (4*16)+9 , (4*16)+10, (4*16)+11,
			(4*16)+12, (4*16)+13, (4*16)+14, (4*16)+15,

			(5*16)+8 , (5*16)+9 , (5*16)+10, (5*16)+11,
			(5*16)+12, (5*16)+13, (5*16)+14, (5*16)+15,

			(6*16)+8 , (6*16)+9 , (6*16)+10, (6*16)+11,
			(6*16)+12, (6*16)+13, (6*16)+14, (6*16)+15,

			(7*16)+8 , (7*16)+9 , (7*16)+10, (7*16)+11,
			(7*16)+12, (7*16)+13, (7*16)+14, (7*16)+15,

		},
		.oobavail = 8 * 8,
		.oobfree  = {
			{(0*16)+0, 8},
			{(1*16)+0, 8},
			{(2*16)+0, 8},
			{(3*16)+0, 8},
			{(4*16)+0, 8},
			{(5*16)+0, 8},
			{(6*16)+0, 8},
			{(7*16)+0, 8},
		},
	}
};

static struct nfc_geometry  nfc_geometry_4K_218_BCH_ECC8 = {
	.page_data_size  = 4096,
	.page_oob_size   = 218,
	.ecc_algorithm   = "BCH",
	.ecc_strength    = 8,
	.buffer_count    = 8,
	.spare_buf_size  = 26,
	.spare_buf_spill = 10,
	.mtd_layout = {
		.eccbytes = 32 /*10 * 8*/, /* See notes above. */
		.eccpos   = {

			(0*26)+12, (0*26)+13, (0*26)+14, (0*26)+15, (0*26)+16,
			(0*26)+17, (0*26)+18, (0*26)+19, (0*26)+20, (0*26)+21,

			(1*26)+12, (1*26)+13, (1*26)+14, (1*26)+15, (1*26)+16,
			(1*26)+17, (1*26)+18, (1*26)+19, (1*26)+20, (1*26)+21,

			(2*26)+12, (2*26)+13, (2*26)+14, (2*26)+15, (2*26)+16,
			(2*26)+17, (2*26)+18, (2*26)+19, (2*26)+20, (2*26)+21,

			(3*26)+12, (3*26)+13, (3*26)+14, (3*26)+15, (3*26)+16,
			(3*26)+17, (3*26)+18, (3*26)+19, (3*26)+20, (3*26)+21,

			(4*26)+12, (4*26)+13, (4*26)+14, (4*26)+15, (4*26)+16,
			(4*26)+17, (4*26)+18, (4*26)+19, (4*26)+20, (4*26)+21,

			(5*26)+12, (5*26)+13, (5*26)+14, (5*26)+15, (5*26)+16,
			(5*26)+17, (5*26)+18, (5*26)+19, (5*26)+20, (5*26)+21,

			(6*26)+12, (6*26)+13, (6*26)+14, (6*26)+15,
			/*  See notes above.
			(6*26)+16,
			(6*26)+17, (6*26)+18, (6*26)+19, (6*26)+20, (6*26)+21,

			(7*26)+12, (7*26)+13, (7*26)+14, (7*26)+15, (7*26)+16,
			(7*26)+17, (7*26)+18, (7*26)+19, (7*26)+20, (7*26)+21,
			*/

		},
		.oobavail = 96 /*(16 * 8) + 10*/, /* See notes above. */
		.oobfree  = {
			{(0*26)+0, 12}, {(0*26)+22, 4},
			{(1*26)+0, 12}, {(1*26)+22, 4},
			{(2*26)+0, 12}, {(2*26)+22, 4},
			{(3*26)+0, 48}, /* See notes above. */
			/* See notes above.
			{(3*26)+0, 12}, {(3*26)+22, 4},
			{(4*26)+0, 12}, {(4*26)+22, 4},
			{(5*26)+0, 12}, {(5*26)+22, 4},
			{(6*26)+0, 12}, {(6*26)+22, 4},
			{(7*26)+0, 12}, {(7*26)+22, 4 + 10},
			*/
		},
	}
};

/*
 * When the logical geometry differs from the NFC geometry (e.g.,
 * interleaving), we synthesize a layout rather than use the one that comes with
 * the NFC geometry. See mal_set_logical_geometry().
 */

static struct nand_ecclayout  synthetic_layout;

/*
 * These structures describe how the BBT code will find block marks in the OOB
 * area of a page. Don't be confused by the fact that this is the same type used
 * to describe bad block tables. Some of the same information is needed, so the
 * designers use the same structure for two conceptually distinct functions.
 */

static uint8_t block_mark_pattern[] = { 0xff, 0xff };

static struct nand_bbt_descr small_page_block_mark_descriptor = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs    = 5,
	.len     = 1,
	.pattern = block_mark_pattern,
};

static struct nand_bbt_descr large_page_block_mark_descriptor = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs    = 0,
	.len     = 2,
	.pattern = block_mark_pattern,
};

/*
 * Bad block table descriptors for the main and mirror tables.
 */

static uint8_t bbt_main_pattern[]   = { 'B', 'b', 't', '0' };
static uint8_t bbt_mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descriptor = {
	.options =
		NAND_BBT_LASTBLOCK |
		NAND_BBT_CREATE    |
		NAND_BBT_WRITE     |
		NAND_BBT_2BIT      |
		NAND_BBT_VERSION   |
		NAND_BBT_PERCHIP,
	.offs      = 0,
	.len       = 4,
	.veroffs   = 4,
	.maxblocks = 4,
	.pattern   = bbt_main_pattern,
};

static struct nand_bbt_descr bbt_mirror_descriptor = {
	.options =
		NAND_BBT_LASTBLOCK |
		NAND_BBT_CREATE    |
		NAND_BBT_WRITE     |
		NAND_BBT_2BIT      |
		NAND_BBT_VERSION   |
		NAND_BBT_PERCHIP,
	.offs      = 0,
	.len       = 4,
	.veroffs   = 4,
	.maxblocks = 4,
	.pattern   = bbt_mirror_pattern,
};

#ifdef EVENT_REPORTING

/**
 * struct event - A single record in the event trace.
 *
 * @time:         The time at which the event occurred.
 * @nesting:      Indicates function call nesting.
 * @description:  A description of the event.
 */

struct event {
	ktime_t       time;
	unsigned int  nesting;
	char          *description;
};

/**
 * The event trace.
 *
 * @overhead:  The delay to take a time stamp and nothing else.
 * @nesting:   The current nesting level.
 * @overflow:  Indicates the trace overflowed.
 * @next:      Index of the next event to write.
 * @events:    The array of events.
 */

#define MAX_EVENT_COUNT  (200)

static struct {
	ktime_t       overhead;
	int           nesting;
	int           overflow;
	unsigned int  next;
	struct event  events[MAX_EVENT_COUNT];
} event_trace;

/**
 * reset_event_trace() - Resets the event trace.
 */
static void reset_event_trace(void)
{
	event_trace.nesting  = 0;
	event_trace.overflow = false;
	event_trace.next     = 0;
}

/**
 * add_event() - Adds an event to the event trace.
 *
 * @description:  A description of the event.
 * @delta:        A delta to the nesting level for this event [-1, 0, 1].
 */
static inline void add_event(char *description, int delta)
{
	struct event  *event;

	if (!imx_nfc_module_report_events)
		return;

	if (event_trace.overflow)
		return;

	if (event_trace.next >= MAX_EVENT_COUNT) {
		event_trace.overflow = true;
		return;
	}

	event = event_trace.events + event_trace.next;

	event->time = ktime_get();

	event->description = description;

	if (!delta)
		event->nesting = event_trace.nesting;
	else if (delta < 0) {
		event->nesting = event_trace.nesting - 1;
		event_trace.nesting -= 2;
	} else {
		event->nesting = event_trace.nesting + 1;
		event_trace.nesting += 2;
	}

	if (event_trace.nesting < 0)
		event_trace.nesting = 0;

	event_trace.next++;

}

/**
 * add_state_event_l() - Adds an event to display some state.
 *
 * @address:   The address to examine.
 * @mask:      A mask to apply to the contents of the given address.
 * @clear:     An event message to add if the result is zero.
 * @not_zero:  An event message to add if the result is not zero.
 */
static void add_state_event_l(void *address, uint32_t mask,
						char *zero, char *not_zero)
{
	int  state;
	state = !!(__raw_readl(address) & mask);
	if (state)
		add_event(not_zero, 0);
	else
		add_event(zero, 0);
}

/**
 * start_event_trace() - Starts an event trace and adds the first event.
 *
 * @description:  A description of the first event.
 */
static void start_event_trace(char *description)
{

	ktime_t  t0;
	ktime_t  t1;

	if (!imx_nfc_module_report_events)
		return;

	reset_event_trace();

	t0 = ktime_get();
	t1 = ktime_get();

	event_trace.overhead = ktime_sub(t1, t0);

	add_event(description, 1);

}

/**
 * dump_event_trace() - Dumps the event trace.
 */
static void dump_event_trace(void)
{
	unsigned int  i;
	time_t        seconds;
	long          nanoseconds;
	char          line[100];
	int           o;
	struct event  *first_event;
	struct event  *last_event;
	struct event  *matching_event;
	struct event  *event;
	ktime_t       delta;

	/* Check if event reporting is turned off. */

	if (!imx_nfc_module_report_events)
		return;

	/* Print important facts about this event trace. */

	printk(KERN_DEBUG "\n+--------------\n");

	printk(KERN_DEBUG "|  Overhead    : [%d:%d]\n",
				event_trace.overhead.tv.sec,
				event_trace.overhead.tv.nsec);

	if (!event_trace.next) {
		printk(KERN_DEBUG "|  No Events\n");
		return;
	}

	first_event = event_trace.events;
	last_event  = event_trace.events + (event_trace.next - 1);

	delta = ktime_sub(last_event->time, first_event->time);
	printk(KERN_DEBUG "|  Elapsed Time: [%d:%d]\n",
						delta.tv.sec, delta.tv.nsec);

	if (event_trace.overflow)
		printk(KERN_DEBUG "|  Overflow!\n");

	/* Print the events in this history. */

	for (i = 0, event = event_trace.events;
					i < event_trace.next; i++, event++) {

		/* Get the delta between this event and the previous event. */

		if (!i) {
			seconds     = 0;
			nanoseconds = 0;
		} else {
			delta = ktime_sub(event[0].time, event[-1].time);
			seconds     = delta.tv.sec;
			nanoseconds = delta.tv.nsec;
		}

		/* Print the current event. */

		o = 0;

		o = snprintf(line, sizeof(line) - o, "|  [%ld:% 10ld]%*s %s",
							seconds, nanoseconds,
							event->nesting, "",
							event->description);
		/* Check if this is the last event in a nested series. */

		if (i && (event[0].nesting < event[-1].nesting)) {

			for (matching_event = event - 1;; matching_event--) {

				if (matching_event < event_trace.events) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(event->time,
							matching_event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" <%d:%d]", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		/* Check if this is the first event in a nested series. */

		if ((i < event_trace.next - 1) &&
				(event[0].nesting < event[1].nesting)) {

			for (matching_event = event + 1;; matching_event++) {

				if (matching_event >=
					(event_trace.events+event_trace.next)) {
					matching_event = 0;
					break;
				}

				if (matching_event->nesting == event->nesting)
					break;

			}

			if (matching_event) {
				delta = ktime_sub(matching_event->time,
								event->time);
				o += snprintf(line + o, sizeof(line) - o,
						" [%d:%d>", delta.tv.sec,
								delta.tv.nsec);
			}

		}

		printk(KERN_DEBUG "%s\n", line);

	}

	printk(KERN_DEBUG "+--------------\n");

}

/**
 * stop_event_trace() - Stops an event trace.
 *
 * @description:  A description of the last event.
 */
static void stop_event_trace(char *description)
{
	struct event  *event;

	if (!imx_nfc_module_report_events)
		return;

	/*
	 * We want the end of the trace, no matter what happens. If the trace
	 * has already overflowed, or is about to, just jam this event into the
	 * last spot. Otherwise, add this event like any other.
	 */

	if (event_trace.overflow || (event_trace.next >= MAX_EVENT_COUNT)) {
		event = event_trace.events + (MAX_EVENT_COUNT - 1);
		event->time = ktime_get();
		event->description = description;
		event->nesting     = 0;
	} else {
		add_event(description, -1);
	}

	dump_event_trace();
	reset_event_trace();

}

#else /* EVENT_REPORTING */

#define start_event_trace(description)                   do {} while (0)
#define add_event(description, delta)                    do {} while (0)
#define add_state_event_l(address, mask, zero, not_zero) do {} while (0)
#define stop_event_trace(description)                    do {} while (0)
#define dump_event_trace()                               do {} while (0)

#endif /* EVENT_REPORTING */

/**
 * unimplemented() - Announces intentionally unimplemented features.
 *
 * @this:  Per-device data.
 * @msg:   A message about the unimplemented feature.
 */
static inline void unimplemented(struct imx_nfc_data *this, const char * msg)
{
	dev_err(this->dev, "Intentionally unimplemented: %s", msg);
}

/**
 * raw_read_mask_w() - Reads masked bits in a 16-bit hardware register.
 */
static inline uint16_t raw_read_mask_w(uint16_t mask, void *address)
{
	return __raw_readw(address) & mask;
}

/**
 * raw_set_mask_w() - Sets bits in a 16-bit hardware register.
 */
static inline void raw_set_mask_w(uint16_t mask, void *address)
{
	__raw_writew(__raw_readw(address) | mask, address);
}

/**
 * raw_clr_mask_w() - Clears bits in a 16-bit hardware register.
 */
static inline void raw_clr_mask_w(uint16_t mask, void *address)
{
	__raw_writew(__raw_readw(address) & (~mask), address);
}

/**
 * raw_read_mask_l() - Reads masked bits in a 32-bit hardware register.
 */
static inline uint32_t raw_read_mask_l(uint32_t mask, void *address)
{
	return __raw_readl(address) & mask;
}

/**
 * raw_set_mask_l() - Sets bits in a 32-bit hardware register.
 */
static inline void raw_set_mask_l(uint32_t mask, void *address)
{
	__raw_writel(__raw_readl(address) | mask, address);
}

/**
 * raw_clr_mask_l() - Clears bits in a 32-bit hardware register.
 */
static inline void raw_clr_mask_l(uint32_t mask, void *address)
{
	__raw_writel(__raw_readl(address) & (~mask), address);
}

/**
 * is_large_page_chip() - Returns true for large page media.
 *
 * @this:  Per-device data.
 */
static inline int is_large_page_chip(struct imx_nfc_data *this)
{
	return (this->physical_geometry.page_data_size > 512);
}

/**
 * is_small_page_chip() - Returns true for small page media.
 *
 * @this:  Per-device data.
 */
static inline int is_small_page_chip(struct imx_nfc_data *this)
{
	return !is_large_page_chip(this);
}

/**
 * get_cycle_in_ns() - Returns the given device's cycle period, in ns.
 *
 * @this:  Per-device data.
 */
static inline unsigned get_cycle_in_ns(struct imx_nfc_data *this)
{
	unsigned long  cycle_in_ns;

	cycle_in_ns = 1000000000 / clk_get_rate(this->clock);

	if (!this->nfc->get_symmetric(this))
		cycle_in_ns *= 2;

	return cycle_in_ns;

}

/**
 * nfc_util_set_best_cycle() - Sets the closest possible NAND Flash bus cycle.
 *
 * This function computes the clock setup that will best approximate the given
 * target Flash bus cycle period.
 *
 * For some NFC versions, we can make the clock "symmetric." When the clock
 * is "symmetric," the hardware waits one NFC clock for every read/write cycle.
 * When the clock is "asymmetric," the hardware waits two NFC clocks for every
 * read/write cycle. Thus, making the clock asymmetric essentially divides the
 * NFC clock by two.
 *
 * We compute the target frequency that matches the given target period. We then
 * discover the closest available match with that frequency and the closest
 * available match with double that frequency (for use with an asymmetric
 * clock). We implement the best choice of original clock and symmetric or
 * asymmetric setting, preferring symmetric clocks.
 *
 * @this:      Per-device data.
 * @ns:        The target cycle period, in nanoseconds.
 * @no_asym:   Disallow making the clock asymmetric.
 * @no_sym:    Disallow making the clock  symmetric.
 */
static int nfc_util_set_best_cycle(struct imx_nfc_data *this,
				unsigned int ns, int no_asym, int no_sym)
{
	unsigned long  target_hz;
	long           symmetric_hz;
	long           symmetric_delta_hz;
	long           asymmetric_hz;
	long           asymmetric_delta_hz;
	unsigned long  best_hz;
	int            best_symmetry_setting;
	struct device  *dev = this->dev;

	/* The target cycle period must be greater than zero. */

	if (!ns)
		return -EINVAL;

	/* Compute the target frequency. */

	target_hz = 1000000000 / ns;

	/* Find out how close we can get with a symmetric clock. */

	if (!no_sym && this->nfc->can_be_symmetric)
		symmetric_hz = clk_round_rate(this->clock, target_hz);
	else
		symmetric_hz = -EINVAL;

	/* Find out how close we can get with an asymmetric clock. */

	if (!no_asym)
		asymmetric_hz = clk_round_rate(this->clock, target_hz * 2);
	else
		asymmetric_hz = -EINVAL;

	/* Does anything work at all? */

	if ((symmetric_hz == -EINVAL) && (asymmetric_hz == -EINVAL)) {
		dev_err(dev, "Can't support Flash bus cycle of %uns\n", ns);
		return -EINVAL;
	}

	/* Discover the best match. */

	if ((symmetric_hz != -EINVAL) && (asymmetric_hz != -EINVAL)) {

		symmetric_delta_hz  = target_hz - symmetric_hz;
		asymmetric_delta_hz = target_hz - (asymmetric_hz / 2);

		if (symmetric_delta_hz <= asymmetric_delta_hz)
			best_symmetry_setting = true;
		else
			best_symmetry_setting = false;

	} else if (symmetric_hz != -EINVAL) {
		best_symmetry_setting = true;
	} else {
		best_symmetry_setting = false;
	}

	best_hz = best_symmetry_setting ? symmetric_hz : asymmetric_hz;

	/* Implement the best match. */

	this->nfc->set_symmetric(this, best_symmetry_setting);

	return clk_set_rate(this->clock, best_hz);

}

/**
 * nfc_util_wait_for_the_nfc() - Waits for the NFC to finish an operation.
 *
 * @this:     Per-device data.
 * @use_irq:  Indicates that we should wait for an interrupt rather than polling
 *            and delaying.
 */
static void nfc_util_wait_for_the_nfc(struct imx_nfc_data *this, int use_irq)
{
	unsigned       spin_count;
	struct device  *dev = this->dev;

	/* Apply the override, if any. */

	switch (this->interrupt_override) {

	case NEVER:
		use_irq = false;
		break;

	case DRIVER_CHOICE:
		break;

	case ALWAYS:
		use_irq = true;
		break;

	}

	/* Check if we're using interrupts. */

	if (use_irq) {

		/*
		 * If control arrives here, the caller wants to use interrupts.
		 * Presumably, this operation is known to take a very long time.
		 */

		if (this->nfc->is_interrupting(this)) {
			add_event("Waiting for the NFC (early interrupt)", 1);
			this->nfc->clear_interrupt(this);
		} else {
			add_event("Waiting for the NFC (interrupt)", 1);
			this->nfc->unmask_interrupt(this);
			wait_for_completion(&this->done);
		}

		add_event("NFC done", -1);

	} else {

		/*
		 * If control arrives here, the caller doesn't want to use
		 * interrupts. Presumably, this operation is too quick to
		 * justify the overhead. Leave the interrupt masked, and loop
		 * until the interrupt bit lights up, or we time out.
		 *
		 * We spin for a maximum of about 2ms before declaring a time
		 * out. No operation we could possibly spin on should take that
		 * long.
		 */

		spin_count = 2000;

		add_event("Waiting for the NFC (polling)", 1);

		for (; spin_count > 0; spin_count--) {

			if (this->nfc->is_interrupting(this)) {
				this->nfc->clear_interrupt(this);
				add_event("NFC done", -1);
				return;
			}

			udelay(1);

		}

		/* Timed out. */

		add_event("Timed out", -1);

		dev_err(dev, "[wait_for_the_nfc] ===== Time Out =====\n");
		dump_event_trace();

	}

}

/**
 *  nfc_util_bytewise_copy_from_nfc_mem() - Copies bytes from the NFC memory.
 *
 * @from:  A pointer to the source memory.
 * @to:    A pointer to the destination memory.
 * @size:  The number of bytes to copy.
 */
static void nfc_util_bytewise_copy_from_nfc_mem(const void *from,
							void *to, size_t n)
{
	unsigned int   i;
	const uint8_t  *f = from;
	uint8_t        *t = to;
	uint16_t       *p;
	uint16_t       x;

	for (i = 0; i < n; i++, f++, t++) {

		p = (uint16_t *) (((unsigned long) f) & ~((unsigned long) 1));

		x = __raw_readw(p);

		if (((unsigned long) f) & 0x1)
			*t = (x >> 8) & 0xff;
		else
			*t = (x >> 0) & 0xff;

	}

}

/**
 * nfc_util_bytewise_copy_to_nfc_mem() - Copies bytes to the NFC memory.
 *
 * @from:  A pointer to the source memory.
 * @to:    A pointer to the destination memory.
 * @size:  The number of bytes to copy.
 */
static void nfc_util_bytewise_copy_to_nfc_mem(const void *from,
							void *to, size_t n)
{
	unsigned int   i;
	const uint8_t  *f = from;
	uint8_t        *t = to;
	uint16_t       *p;
	uint16_t       x;

	for (i = 0; i < n; i++, f++, t++) {

		p = (uint16_t *) (((unsigned long) t) & ~((unsigned long) 1));

		x = __raw_readw(p);

		if (((unsigned long) t) & 0x1)
			((uint8_t *)(&x))[1] = *f;
		else
			((uint8_t *)(&x))[0] = *f;

		__raw_writew(x, p);

	}

}

/**
 * nfc_util_copy_from_nfc_mem() - Copies from the NFC memory to main memory.
 *
 * @from:  A pointer to the source memory.
 * @to:    A pointer to the destination memory.
 * @size:  The number of bytes to copy.
 */
static void nfc_util_copy_from_nfc_mem(const void *from, void *to, size_t n)
{
	unsigned int  chunk_count;

	/*
	 * Check if we're testing bytewise copies.
	 */

	if (imx_nfc_module_force_bytewise_copy)
		goto force_bytewise_copy;

	/*
	 * We'd like to use memcpy to get data out of the NFC but, because that
	 * memory responds only to 16- and 32-byte reads, we can only do so
	 * safely if both the start and end of both the source and destination
	 * are perfectly aligned on 4-byte boundaries.
	 */

	if (!(((unsigned long) from) & 0x3) && !(((unsigned long) to) & 0x3)) {

		/*
		 * If control arrives here, both the source and destination are
		 * aligned on 4-byte boundaries. Compute the number of whole,
		 * 4-byte chunks we can move.
		 */

		chunk_count = n / 4;

		/*
		 * Move all the chunks we can, and then update the pointers and
		 * byte count to show what's left.
		 */

		if (chunk_count) {
			memcpy(to, from, chunk_count * 4);
			from += chunk_count * 4;
			to   += chunk_count * 4;
			n    -= chunk_count * 4;
		}

	}

	/*
	 * Move what's left.
	 */

force_bytewise_copy:

	nfc_util_bytewise_copy_from_nfc_mem(from, to, n);

}

/**
 * nfc_util_copy_to_nfc_mem() - Copies from main memory to the NFC memory.
 *
 * @from:  A pointer to the source memory.
 * @to:    A pointer to the destination memory.
 * @size:  The number of bytes to copy.
 */
static void nfc_util_copy_to_nfc_mem(const void *from, void *to, size_t n)
{
	unsigned int  chunk_count;

	/*
	 * Check if we're testing bytewise copies.
	 */

	if (imx_nfc_module_force_bytewise_copy)
		goto force_bytewise_copy;

	/*
	 * We'd like to use memcpy to get data into the NFC but, because that
	 * memory responds only to 16- and 32-byte writes, we can only do so
	 * safely if both the start and end of both the source and destination
	 * are perfectly aligned on 4-byte boundaries.
	 */

	if (!(((unsigned long) from) & 0x3) && !(((unsigned long) to) & 0x3)) {

		/*
		 * If control arrives here, both the source and destination are
		 * aligned on 4-byte boundaries. Compute the number of whole,
		 * 4-byte chunks we can move.
		 */

		chunk_count = n / 4;

		/*
		 * Move all the chunks we can, and then update the pointers and
		 * byte count to show what's left.
		 */

		if (chunk_count) {
			memcpy(to, from, chunk_count * 4);
			from += chunk_count * 4;
			to   += chunk_count * 4;
			n    -= chunk_count * 4;
		}

	}

	/*
	 * Move what's left.
	 */

force_bytewise_copy:

	nfc_util_bytewise_copy_to_nfc_mem(from, to, n);

}

/**
 * nfc_util_copy_from_the_nfc() - Copies bytes out of the NFC.
 *
 * This function makes the data in the NFC look like a contiguous, model page.
 *
 * @this:   Per-device data.
 * @start:  The index of the starting byte in the NFC.
 * @buf:    A pointer to the destination buffer.
 * @len:    The number of bytes to copy out.
 */
static void nfc_util_copy_from_the_nfc(struct imx_nfc_data *this,
			unsigned int start, uint8_t *buf, unsigned int len)
{
	unsigned int         i;
	unsigned int         count;
	unsigned int         offset;
	unsigned int         data_size;
	unsigned int         oob_size;
	unsigned int         total_size;
	void                 *spare_base;
	unsigned int         first_spare;
	void                 *from;
	struct nfc_geometry  *geometry = this->nfc_geometry;

	/*
	 * During initialization, the HIL will attempt to read ID bytes. For
	 * some NFC hardware versions, the ID bytes are deposited in the NFC
	 * memory, so this function will be called to deliver them. At that
	 * point, we won't know the NFC geometry. That's OK because we're only
	 * going to be reading a byte at a time.
	 *
	 * If we don't yet know the NFC geometry, just plug in some values that
	 * make things work for now.
	 */

	if (unlikely(!geometry)) {
		data_size = NFC_MAIN_BUF_SIZE;
		oob_size  = 0;
	} else {
		data_size = geometry->page_data_size;
		oob_size  = geometry->page_oob_size;
	}

	total_size = data_size + oob_size;

	/* Validate. */

	if ((start >= total_size) || ((start + len) > total_size)) {
		dev_err(this->dev, "Bad copy from NFC memory: [%u, %u]\n",
								start, len);
		return;
	}

	/* Check if we're copying anything at all. */

	if (!len)
		return;

	/* Check if anything comes from the main area. */

	if (start < data_size) {

		/* Compute the bytes to copy from the main area. */

		count = min(len, data_size - start);

		/* Copy. */

		nfc_util_copy_from_nfc_mem(this->buffers + start, buf, count);

		buf   += count;
		start += count;
		len   -= count;

	}

	/* Check if we're done. */

	if (!len)
		return;

	/* Compute the base address of the spare buffers. */

	spare_base = this->buffers +
			(this->nfc->max_buffer_count * NFC_MAIN_BUF_SIZE);

	/* Discover in which spare buffer the copying begins. */

	first_spare = (start - data_size) / geometry->spare_buf_size;

	/* Check if anything comes from the regular spare buffer area. */

	if (first_spare < geometry->buffer_count) {

		/* Start copying from spare buffers. */

		for (i = first_spare; i < geometry->buffer_count; i++) {

			/* Compute the offset into this spare area. */

			offset = start -
				(data_size + (geometry->spare_buf_size * i));

			/* Compute the address of that offset. */

			from = spare_base + offset +
					(this->nfc->spare_buf_stride * i);

			/* Compute the bytes to copy from this spare area. */

			count = min(len, geometry->spare_buf_size - offset);

			/* Copy. */

			nfc_util_copy_from_nfc_mem(from, buf, count);

			buf   += count;
			start += count;
			len   -= count;

		}

	}

	/* Check if we're done. */

	if (!len)
		return;

	/* Compute the offset into the extra spare area. */

	offset = start -
		(data_size + (geometry->spare_buf_size*geometry->buffer_count));

	/* Compute the address of that offset. */

	from = spare_base + offset +
			(this->nfc->spare_buf_stride * geometry->buffer_count);

	/* Compute the bytes to copy from the extra spare area. */

	count = min(len, geometry->spare_buf_spill - offset);

	/* Copy. */

	nfc_util_copy_from_nfc_mem(from, buf, count);

}

/**
 * nfc_util_copy_to_the_nfc() - Copies bytes into the NFC memory.
 *
 * This function makes the data in the NFC look like a contiguous, model page.
 *
 * @this:   Per-device data.
 * @buf:   A pointer to the source buffer.
 * @start: The index of the starting byte in the NFC memory.
 * @len:   The number of bytes to copy in.
 */
static void nfc_util_copy_to_the_nfc(struct imx_nfc_data *this,
		const uint8_t *buf, unsigned int start, unsigned int len)
{
	unsigned int         i;
	unsigned int         count;
	unsigned int         offset;
	unsigned int         data_size;
	unsigned int         oob_size;
	unsigned int         total_size;
	void                 *spare_base;
	unsigned int         first_spare;
	void                 *to;
	struct nfc_geometry  *geometry = this->nfc_geometry;

	/* Establish some important facts. */

	data_size  = geometry->page_data_size;
	oob_size   = geometry->page_oob_size;
	total_size = data_size + oob_size;

	/* Validate. */

	if ((start >= total_size) || ((start + len) > total_size)) {
		dev_err(this->dev, "Bad copy to NFC memory: [%u, %u]\n",
								start, len);
		return;
	}

	/* Check if we're copying anything at all. */

	if (!len)
		return;

	/* Check if anything goes to the main area. */

	if (start < data_size) {

		/* Compute the bytes to copy to the main area. */

		count = min(len, data_size - start);

		/* Copy. */

		nfc_util_copy_to_nfc_mem(buf, this->buffers + start, count);

		buf   += count;
		start += count;
		len   -= count;

	}

	/* Check if we're done. */

	if (!len)
		return;

	/* Compute the base address of the spare buffers. */

	spare_base = this->buffers +
			(this->nfc->max_buffer_count * NFC_MAIN_BUF_SIZE);

	/* Discover in which spare buffer the copying begins. */

	first_spare = (start - data_size) / geometry->spare_buf_size;

	/* Check if anything goes to the regular spare buffer area. */

	if (first_spare < geometry->buffer_count) {

		/* Start copying to spare buffers. */

		for (i = first_spare; i < geometry->buffer_count; i++) {

			/* Compute the offset into this spare area. */

			offset = start -
				(data_size + (geometry->spare_buf_size * i));

			/* Compute the address of that offset. */

			to = spare_base + offset +
					(this->nfc->spare_buf_stride * i);

			/* Compute the bytes to copy to this spare area. */

			count = min(len, geometry->spare_buf_size - offset);

			/* Copy. */

			nfc_util_copy_to_nfc_mem(buf, to, count);

			buf   += count;
			start += count;
			len   -= count;

		}

	}

	/* Check if we're done. */

	if (!len)
		return;

	/* Compute the offset into the extra spare area. */

	offset = start -
		(data_size + (geometry->spare_buf_size*geometry->buffer_count));

	/* Compute the address of that offset. */

	to = spare_base + offset +
			(this->nfc->spare_buf_stride * geometry->buffer_count);

	/* Compute the bytes to copy to the extra spare area. */

	count = min(len, geometry->spare_buf_spill - offset);

	/* Copy. */

	nfc_util_copy_to_nfc_mem(buf, to, count);

}

/**
 * nfc_util_isr() - i.MX NFC ISR.
 *
 * @irq:      The arriving interrupt number.
 * @context:  A cookie for this ISR.
 */
static irqreturn_t nfc_util_isr(int irq, void *cookie)
{
	struct imx_nfc_data  *this = cookie;
	this->nfc->mask_interrupt(this);
	this->nfc->clear_interrupt(this);
	complete(&this->done);
	return IRQ_HANDLED;
}

/**
 * nfc_util_send_cmd() - Sends a command to the current chip, without waiting.
 *
 * @this:     Per-device data.
 * @command:  The command code.
 */

static void nfc_util_send_cmd(struct imx_nfc_data *this, unsigned int command)
{

	add_event("Entering nfc_util_send_cmd", 1);

	this->nfc->command_cycle(this, command);

	add_event("Exiting nfc_util_send_cmd", -1);

}

/**
 * nfc_util_send_cmd_and_addrs() - Sends a cmd and addrs to the current chip.
 *
 * This function conveniently combines sending a command, and then sending
 * optional addresses, waiting for the NFC to finish will all steps.
 *
 * @this:     Per-device data.
 * @command:  The command code.
 * @column:   The column address to send, or -1 if no column address applies.
 * @page:     The page address to send, or -1 if no page address applies.
 */

static void nfc_util_send_cmd_and_addrs(struct imx_nfc_data *this,
					unsigned command, int column, int page)
{
	uint32_t  page_mask;

	add_event("Entering nfc_util_send_cmd_and_addrs", 1);

	/* Send the command.*/

	add_event("Sending the command...", 0);

	nfc_util_send_cmd(this, command);

	nfc_util_wait_for_the_nfc(this, false);

	/* Send the addresses. */

	add_event("Sending the addresses...", 0);

	if (column != -1) {

		this->nfc->write_cycle(this, (column >> 0) & 0xff);

		if (is_large_page_chip(this))
			this->nfc->write_cycle(this, (column >> 8) & 0xff);

	}

	if (page != -1) {

		page_mask = this->nand.pagemask;

		do {
			this->nfc->write_cycle(this, page & 0xff);
			page_mask >>= 8;
			page      >>= 8;
		} while (page_mask != 0);

	}

	add_event("Exiting nfc_util_send_cmd_and_addrs", -1);

}

/**
 * nfc_2_x_exit() - Version-specific shut down.
 *
 * @this:  Per-device data.
 */
static void nfc_2_x_exit(struct imx_nfc_data *this)
{
}

/**
 * nfc_2_x_clear_interrupt() - Clears an interrupt.
 *
 * @this:  Per-device data.
 */
static void nfc_2_x_clear_interrupt(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;
	raw_clr_mask_w(NFC_2_X_CONFIG2_INT_MSK, base + NFC_2_X_CONFIG2_REG_OFF);
}

/**
 * nfc_2_x_is_interrupting() - Returns the interrupt bit status.
 *
 * @this:  Per-device data.
 */
static int nfc_2_x_is_interrupting(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;
	return raw_read_mask_w(NFC_2_X_CONFIG2_INT_MSK,
						base + NFC_2_X_CONFIG2_REG_OFF);
}

/**
 * nfc_2_x_command_cycle() - Sends a command.
 *
 * @this:     Per-device data.
 * @command:  The command code.
 */
static void nfc_2_x_command_cycle(struct imx_nfc_data *this, unsigned command)
{
	void  *base = this->primary_regs;

	/* Write the command we want to send. */

	__raw_writew(command, base + NFC_2_X_FLASH_CMD_REG_OFF);

	/* Launch a command cycle. */

	__raw_writew(NFC_2_X_CONFIG2_FCMD_MSK, base + NFC_2_X_CONFIG2_REG_OFF);

}

/**
 * nfc_2_x_write_cycle() - Writes a single byte.
 *
 * @this:  Per-device data.
 * @byte:  The byte.
 */
static void nfc_2_x_write_cycle(struct imx_nfc_data *this, unsigned int byte)
{
	void  *base = this->primary_regs;

	/* Give the NFC the byte we want to write. */

	__raw_writew(byte, base + NFC_2_X_FLASH_ADDR_REG_OFF);

	/* Launch an address cycle.
	 *
	 * This is *sort* of a hack, but not really. The intent of the NFC
	 * design is for this operation to send an address byte. In fact, the
	 * NFC neither knows nor cares what we're sending. It justs runs a write
	 * cycle.
	 */

	__raw_writew(NFC_2_X_CONFIG2_FADD_MSK, base + NFC_2_X_CONFIG2_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, false);

}

/**
 * nfc_2_0_init() - Version-specific hardware initialization.
 *
 * @this:  Per-device data.
 */
static int nfc_2_0_init(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;

	/* Initialize the interrupt machinery. */

	this->nfc->mask_interrupt(this);
	this->nfc->clear_interrupt(this);

	/* Unlock the NFC memory. */

	__raw_writew(0x2, base + NFC_2_X_CONFIG_REG_OFF);

	/* Set the unlocked block range to cover the entire medium. */

	__raw_writew(0 , base + NFC_2_0_UNLOCK_START_REG_OFF);
	__raw_writew(~0, base + NFC_2_0_UNLOCK_END_REG_OFF);

	/* Unlock all blocks. */

	__raw_writew(0x4, base + NFC_2_X_WR_PROT_REG_OFF);

	/* Return success. */

	return 0;

}

/**
 * nfc_2_0_mask_interrupt() - Masks interrupts.
 *
 * @this:  Per-device data.
 */
static void nfc_2_0_mask_interrupt(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;
	raw_set_mask_w(NFC_2_0_CONFIG1_INT_MSK_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);
}

/**
 * nfc_2_0_unmask_interrupt() - Unmasks interrupts.
 *
 * @this:  Per-device data.
 */
static void nfc_2_0_unmask_interrupt(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;
	raw_clr_mask_w(NFC_2_0_CONFIG1_INT_MSK_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);
}

/**
 * nfc_2_0_set_ecc() - Turns ECC on or off.
 *
 * @this:  Per-device data.
 * @on:    Indicates if ECC should be on or off.
 */
static void nfc_2_0_set_ecc(struct imx_nfc_data *this, int on)
{
	void  *base = this->primary_regs;

	if (on)
		raw_set_mask_w(NFC_2_0_CONFIG1_ECC_EN_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);
	else
		raw_clr_mask_w(NFC_2_0_CONFIG1_ECC_EN_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);

}

/**
 * nfc_2_0_get_ecc_status() - Reports ECC errors.
 *
 * @this:  Per-device data.
 */
static int nfc_2_0_get_ecc_status(struct imx_nfc_data *this)
{
	unsigned int  i;
	void          *base = this->primary_regs;
	uint16_t      status_reg;
	unsigned int  buffer_status[4];
	int           status;

	/* Get the entire status register. */

	status_reg = __raw_readw(base + NFC_2_0_ECC_STATUS_REG_OFF);

	/* Pick out the status for each buffer. */

	buffer_status[0] = (status_reg & NFC_2_0_ECC_STATUS_NOSER1_MSK)
					>> NFC_2_0_ECC_STATUS_NOSER1_POS;

	buffer_status[1] = (status_reg & NFC_2_0_ECC_STATUS_NOSER2_MSK)
					>> NFC_2_0_ECC_STATUS_NOSER2_POS;

	buffer_status[2] = (status_reg & NFC_2_0_ECC_STATUS_NOSER3_MSK)
					>> NFC_2_0_ECC_STATUS_NOSER3_POS;

	buffer_status[3] = (status_reg & NFC_2_0_ECC_STATUS_NOSER4_MSK)
					>> NFC_2_0_ECC_STATUS_NOSER4_POS;

	/* Loop through the buffers we're actually using. */

	status = 0;

	for (i = 0; i < this->nfc_geometry->buffer_count; i++) {

		if (buffer_status[i] > this->nfc_geometry->ecc_strength) {
			status = -1;
			break;
		}

		status += buffer_status[i];

	}

	/* Return the final result. */

	return status;

}

/**
 * nfc_2_0_get_symmetric() - Indicates if the clock is symmetric.
 *
 * @this:  Per-device data.
 */
static int nfc_2_0_get_symmetric(struct imx_nfc_data *this)
{
	void  *base = this->primary_regs;

	return !!raw_read_mask_w(NFC_2_0_CONFIG1_ONE_CYLE_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);

}

/**
 * nfc_2_0_set_symmetric() - Turns symmetric clock mode on or off.
 *
 * @this:  Per-device data.
 */
static void nfc_2_0_set_symmetric(struct imx_nfc_data *this, int on)
{
	void  *base = this->primary_regs;

	if (on)
		raw_set_mask_w(NFC_2_0_CONFIG1_ONE_CYLE_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);
	else
		raw_clr_mask_w(NFC_2_0_CONFIG1_ONE_CYLE_MSK,
						base + NFC_2_0_CONFIG1_REG_OFF);

}

/**
 * nfc_2_0_set_geometry() - Configures for the medium geometry.
 *
 * @this:  Per-device data.
 */
static int nfc_2_0_set_geometry(struct imx_nfc_data *this)
{
	struct physical_geometry  *physical = &this->physical_geometry;

	/* Select an NFC geometry. */

	switch (physical->page_data_size) {

	case 512:
		this->nfc_geometry = &nfc_geometry_512_16_RS_ECC4;
		break;

	case 2048:
		this->nfc_geometry = &nfc_geometry_2K_64_RS_ECC4;
		break;

	default:
		dev_err(this->dev, "NFC can't handle page size: %u",
						physical->page_data_size);
		return !0;
		break;

	}

	/*
	 * This NFC version receives page size information from a register
	 * that's external to the NFC. We must rely on platform-specific code
	 * to set this register for us.
	 */

	return this->pdata->set_page_size(physical->page_data_size);

}

/**
 * nfc_2_0_select_chip() - Selects the current chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip number to select, or -1 to select no chip.
 */
static void nfc_2_0_select_chip(struct imx_nfc_data *this, int chip)
{
}

/**
 * nfc_2_0_read_cycle() - Applies a single read cycle to the current chip.
 *
 * @this:  Per-device data.
 */
static unsigned int nfc_2_0_read_cycle(struct imx_nfc_data *this)
{
	uint8_t       byte;
	unsigned int  result;
	void          *base = this->primary_regs;

	/* Read into main buffer 0. */

	__raw_writew(0x0, base + NFC_2_0_BUF_ADDR_REG_OFF);

	/* Launch a "Data Out" operation. */

	__raw_writew(0x4 << NFC_2_X_CONFIG2_FDO_POS,
						base + NFC_2_X_CONFIG2_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, false);

	/* Get the result from the NFC memory. */

	nfc_util_copy_from_the_nfc(this, 0, &byte, 1);
	result = byte;

	/* Return the results. */

	return result;

}

/**
 * nfc_2_0_read_page() - Reads a page from the current chip into the NFC.
 *
 * @this:  Per-device data.
 */
static void nfc_2_0_read_page(struct imx_nfc_data *this)
{
	unsigned int  i;
	void          *base = this->primary_regs;

	/* Loop over the number of buffers in use. */

	for (i = 0; i < this->nfc_geometry->buffer_count; i++) {

		/* Make the NFC read into the current buffer. */

		__raw_writew(i << NFC_2_0_BUF_ADDR_RBA_POS,
					base + NFC_2_0_BUF_ADDR_REG_OFF);

		/* Launch a page data out operation. */

		__raw_writew(0x1 << NFC_2_X_CONFIG2_FDO_POS,
					base + NFC_2_X_CONFIG2_REG_OFF);

		/* Wait for the NFC to finish. */

		nfc_util_wait_for_the_nfc(this, true);

	}

}

/**
 * nfc_2_0_send_page() - Sends a page from the NFC to the current chip.
 *
 * @this:  Per-device data.
 */
static void nfc_2_0_send_page(struct imx_nfc_data *this)
{
	unsigned int  i;
	void          *base = this->primary_regs;

	/* Loop over the number of buffers in use. */

	for (i = 0; i < this->nfc_geometry->buffer_count; i++) {

		/* Make the NFC send from the current buffer. */

		__raw_writew(i << NFC_2_0_BUF_ADDR_RBA_POS,
					base + NFC_2_0_BUF_ADDR_REG_OFF);

		/* Launch a page data in operation. */

		__raw_writew(0x1 << NFC_2_X_CONFIG2_FDI_POS,
					base + NFC_2_X_CONFIG2_REG_OFF);

		/* Wait for the NFC to finish. */

		nfc_util_wait_for_the_nfc(this, true);

	}

}

/**
 * nfc_3_2_init() - Hardware initialization.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_init(struct imx_nfc_data *this)
{
	int           error;
	unsigned int  no_sdma;
	unsigned int  fmp;
	unsigned int  rbb_mode;
	unsigned int  num_of_devices;
	unsigned int  dma_mode;
	unsigned int  sbb;
	unsigned int  nf_big;
	unsigned int  sb2r;
	unsigned int  fw;
	unsigned int  too;
	unsigned int  add_op;
	uint32_t      config3;
	void          *primary_base   = this->primary_regs;
	void          *secondary_base = this->secondary_regs;

	/* Initialize the interrupt machinery. */

	this->nfc->mask_interrupt(this);
	this->nfc->clear_interrupt(this);

	/* Set up the clock. */

	error = this->nfc->set_closest_cycle(this,
					this->pdata->target_cycle_in_ns);

	if (error)
		return !0;

	/* We never read the spare area alone. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_SP_EN_MSK,
				primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Tell the NFC the "Read Status" command code. */

	raw_clr_mask_l(NFC_3_2_CONFIG2_ST_CMD_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

	raw_set_mask_l(NAND_CMD_STATUS << NFC_3_2_CONFIG2_ST_CMD_POS,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

	/*
	 * According to erratum ENGcm09051, the CONFIG3 register doesn't reset
	 * correctly, so we need to re-build the entire register just in case.
	 */

	/*
	 * Set the NO_SDMA bit to tell the NFC that we are NOT using SDMA. If
	 * you clear this bit (to indicates you *are* using SDMA), but you
	 * don't actually set up SDMA, the NFC has been observed to crash the
	 * hardware when it asserts its DMA request signals. In the future, we
	 * *may* use SDMA, but it's not worth the effort at this writing.
	 */

	no_sdma = 0x1;

	/*
	 * Set the default FIFO Mode Protection (128 bytes). FMP doesn't work if
	 * the NO_SDMA bit is set.
	 */

	fmp = 0x2;

	/*
	 * The rbb_mode bit determines how the NFC figures out whether chips are
	 * ready during automatic operations only (this has no effect on atomic
	 * operations). The two choices are either to monitor the ready/busy
	 * signals, or to read the status register. We monitor the ready/busy
	 * signals.
	 */

	rbb_mode = 0x1;

	/*
	 * We don't yet know how many devices are connected. We'll find out in
	 * out in nfc_3_2_set_geometry().
	 */

	num_of_devices = 0;

	/* Set the default DMA mode. */

	dma_mode = 0x0;

	/* Set the default status busy bit. */

	sbb = 0x6;

	/* Little-endian (the default). */

	nf_big = 0x0;

	/* Set the default (standard) status bit to record. */

	sb2r = 0x0;

	/* We support only 8-bit Flash bus width. */

	fw = 0x1;

	/* We don't support "two-on-one." */

	too = 0x0;

	/* Set the addressing option. */

	add_op = 0x3;

	/* Set the CONFIG3 register. */

	config3 = 0;

	config3 |= no_sdma        << NFC_3_2_CONFIG3_NO_SDMA_POS;
	config3 |= fmp            << NFC_3_2_CONFIG3_FMP_POS;
	config3 |= rbb_mode       << NFC_3_2_CONFIG3_RBB_MODE_POS;
	config3 |= num_of_devices << NFC_3_2_CONFIG3_NUM_OF_DEVICES_POS;
	config3 |= dma_mode       << NFC_3_2_CONFIG3_DMA_MODE_POS;
	config3 |= sbb            << NFC_3_2_CONFIG3_SBB_POS;
	config3 |= nf_big         << NFC_3_2_CONFIG3_NF_BIG_POS;
	config3 |= sb2r           << NFC_3_2_CONFIG3_SB2R_POS;
	config3 |= fw             << NFC_3_2_CONFIG3_FW_POS;
	config3 |= too            << NFC_3_2_CONFIG3_TOO_POS;
	config3 |= add_op         << NFC_3_2_CONFIG3_ADD_OP_POS;

	__raw_writel(config3, secondary_base + NFC_3_2_CONFIG3_REG_OFF);

	/* Return success. */

	return 0;

}

/**
 * nfc_3_2_set_geometry() - Configures for the medium geometry.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_set_geometry(struct imx_nfc_data *this)
{
	unsigned int              ps;
	unsigned int              cmd_phases;
	unsigned int              pages_per_chip;
	unsigned int              addr_phases0;
	unsigned int              addr_phases1;
	unsigned int              pages_per_block;
	unsigned int              ecc_mode;
	unsigned int              ppb;
	unsigned int              spas;
	unsigned int              mask;
	uint32_t                  config2;
	unsigned int              num_of_devices;
	uint32_t                  config3;
	unsigned int              x;
	unsigned int              chip;
	struct physical_geometry  *physical = &this->physical_geometry;
	void                      *secondary_base = this->secondary_regs;

	/*
	 * Select an NFC geometry based on the physical geometry and the
	 * capabilities of this NFC.
	 */

	switch (physical->page_data_size) {

	case 512:
		this->nfc_geometry = &nfc_geometry_512_16_BCH_ECC4;
		ps = 0;
		break;

	case 2048:
		this->nfc_geometry = &nfc_geometry_2K_64_BCH_ECC4;
		ps = 1;
		break;

	case 4096:

		switch (this->physical_geometry.page_oob_size) {

		case 128:
			this->nfc_geometry = &nfc_geometry_4K_128_BCH_ECC4;
			break;

		case 218:
			this->nfc_geometry = &nfc_geometry_4K_218_BCH_ECC8;
			break;

		default:
			dev_err(this->dev,
				"NFC can't handle page geometry: %u+%u",
				physical->page_data_size,
				physical->page_oob_size);
			return !0;
			break;

		}

		ps = 2;

		break;

	default:
		dev_err(this->dev, "NFC can't handle page size: %u",
						physical->page_data_size);
		return !0;
		break;

	}

	/* Compute the ECC mode. */

	switch (this->nfc_geometry->ecc_strength) {

	case 4:
		ecc_mode = 0;
		break;

	case 8:
		ecc_mode = 1;
		break;

	default:
		dev_err(this->dev, "NFC can't handle ECC strength: %u",
					this->nfc_geometry->ecc_strength);
		return !0;
		break;

	}

	/* Compute the pages per block. */

	pages_per_block = physical->block_size / physical->page_data_size;

	switch (pages_per_block) {
	case 32:
		ppb = 0;
		break;
	case 64:
		ppb = 1;
		break;
	case 128:
		ppb = 2;
		break;
	case 256:
		ppb = 3;
		break;
	default:
		dev_err(this->dev, "NFC can't handle pages per block: %d",
							pages_per_block);
		return !0;
		break;
	}

	/*
	 * The hardware needs to know the physical size of the spare area, in
	 * units of half-words (16 bits). This may be different from the amount
	 * of the spare area we actually expose to MTD. For example, for for
	 * 2K+112, we only expose 64 spare bytes, but the hardware needs to know
	 * the real facts.
	 */

	spas = this->physical_geometry.page_oob_size >> 1;

	/*
	 * The number of command phases needed to read a page is directly
	 * dependent on whether this is a small page or large page device. Large
	 * page devices need more address phases, terminated by a second command
	 * phase.
	 */

	cmd_phases = is_large_page_chip(this) ? 1 : 0;

	/*
	 * The num_adr_phases1 field contains the number of phases needed to
	 * transmit addresses for read and program operations. This is the sum
	 * of the number of phases for a page address and the number of phases
	 * for a column address.
	 *
	 * The number of phases for a page address is the number of bytes needed
	 * to contain a page address.
	 *
	 * The number of phases for a column address is the number of bytes
	 * needed to contain a column address.
	 *
	 * After computing the sum, we subtract three because a value of zero in
	 * this field indicates three address phases, and this is the minimum
	 * number of phases the hardware can comprehend.
	 *
	 * We compute the number of phases based on the *physical* geometry, not
	 * the NFC geometry. For example, even if we are treating a very large
	 * device as if it contains fewer pages than it actually does, the
	 * hardware still needs the additional address phases.
	 */

	pages_per_chip =
		physical->chip_size >> (fls(physical->page_data_size) - 1);

	addr_phases1 = (fls(pages_per_chip) >> 3) + 1;

	addr_phases1 += (fls(physical->page_data_size) >> 3) + 1;

	addr_phases1 -= 3;

	/*
	 * The num_adr_phases0 field contains the number of phases needed to
	 * transmit a page address for an erase operation. That is, this is
	 * the value of addr_phases1, less the number of phases for the column
	 * address.
	 *
	 * The hardware expresses this phase count as one or two cycles less
	 * than the count indicated by add_phases1 (see the reference manual).
	 */

	addr_phases0 = is_large_page_chip(this) ? 1 : 0;

	/* Set the CONFIG2 register. */

	mask =
		NFC_3_2_CONFIG2_PS_MSK           |
		NFC_3_2_CONFIG2_CMD_PHASES_MSK   |
		NFC_3_2_CONFIG2_ADDR_PHASES0_MSK |
		NFC_3_2_CONFIG2_ECC_MODE_MSK     |
		NFC_3_2_CONFIG2_PPB_MSK          |
		NFC_3_2_CONFIG2_ADDR_PHASES1_MSK |
		NFC_3_2_CONFIG2_SPAS_MSK         ;

	config2 = __raw_readl(secondary_base + NFC_3_2_CONFIG2_REG_OFF);

	config2 &= ~mask;

	config2 |= ps           << NFC_3_2_CONFIG2_PS_POS;
	config2 |= cmd_phases   << NFC_3_2_CONFIG2_CMD_PHASES_POS;
	config2 |= addr_phases0 << NFC_3_2_CONFIG2_ADDR_PHASES0_POS;
	config2 |= ecc_mode     << NFC_3_2_CONFIG2_ECC_MODE_POS;
	config2 |= ppb          << NFC_3_2_CONFIG2_PPB_POS;
	config2 |= addr_phases1 << NFC_3_2_CONFIG2_ADDR_PHASES1_POS;
	config2 |= spas         << NFC_3_2_CONFIG2_SPAS_POS;

	config2 = __raw_writel(config2,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

	/*
	 * Compute the num_of_devices field.
	 *
	 * It's very important to set this field correctly. This controls the
	 * set of ready/busy lines to which the NFC listens with automatic
	 * transactions. If this number is too large, the NFC will listen to
	 * ready/busy signals that are electrically floating, or it will try to
	 * read the status registers of chips that don't exist. Conversely, if
	 * the number is too small, the NFC could believe an operation is
	 * finished when some chips are still busy.
	 */

	num_of_devices = physical->chip_count - 1;

	/* Set the CONFIG3 register. */

	mask = NFC_3_2_CONFIG3_NUM_OF_DEVICES_MSK;

	config3 = __raw_readl(secondary_base + NFC_3_2_CONFIG3_REG_OFF);

	config3 &= ~mask;

	config3 |= num_of_devices << NFC_3_2_CONFIG3_NUM_OF_DEVICES_POS;

	__raw_writel(config3, secondary_base + NFC_3_2_CONFIG3_REG_OFF);

	/*
	 * Check if the physical chip count is a power of 2. If not, then
	 * automatic operations aren't available. This is because we use an
	 * addressing option (see the ADD_OP field of CONFIG3) that requires
	 * a number of chips that is a power of 2.
	 */

	if (ffs(physical->chip_count) != fls(physical->chip_count)) {
		this->nfc->start_auto_read  = 0;
		this->nfc->start_auto_write = 0;
		this->nfc->start_auto_erase = 0;
	}

	/* Unlock the NFC RAM. */

	x = __raw_readl(secondary_base + NFC_3_2_WRPROT_REG_OFF);
	x &= ~NFC_3_2_WRPROT_BLS_MSK;
	x |= 0x2  << NFC_3_2_WRPROT_BLS_POS;
	__raw_writel(x, secondary_base + NFC_3_2_WRPROT_REG_OFF);

	/* Loop over chip selects, setting the unlocked ranges. */

	for (chip = 0; chip < this->nfc->max_chip_count; chip++) {

		/* Set the unlocked range to cover the entire chip.*/

		__raw_writel(0xffff0000, secondary_base +
				NFC_3_2_UNLOCK_BLK_ADD0_REG_OFF + (chip * 4));

		/* Unlock. */

		x = __raw_readl(secondary_base + NFC_3_2_WRPROT_REG_OFF);
		x &= ~(NFC_3_2_WRPROT_CS2L_MSK | NFC_3_2_WRPROT_WPC_MSK);
		x |= chip << NFC_3_2_WRPROT_CS2L_POS;
		x |= 0x4  << NFC_3_2_WRPROT_WPC_POS ;
		__raw_writel(x, secondary_base + NFC_3_2_WRPROT_REG_OFF);

	}

	/* Return success. */

	return 0;

}

/**
 * nfc_3_2_exit() - Hardware cleanup.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_exit(struct imx_nfc_data *this)
{
}

/**
 * nfc_3_2_set_closest_cycle() - Version-specific hardware cleanup.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_set_closest_cycle(struct imx_nfc_data *this, unsigned ns)
{
	struct clk     *parent_clock;
	unsigned long  parent_clock_rate_in_hz;
	unsigned long  sym_low_clock_rate_in_hz;
	unsigned long  asym_low_clock_rate_in_hz;
	unsigned int   sym_high_cycle_in_ns;
	unsigned int   asym_high_cycle_in_ns;

	/*
	 * According to ENGcm09121:
	 *
	 *  - If the NFC is set to SYMMETRIC mode, the NFC clock divider must
	 *    divide the EMI Slow Clock by NO MORE THAN 4.
	 *
	 *  - If the NFC is set for ASYMMETRIC mode, the NFC clock divider must
	 *    divide the EMI Slow Clock by NO MORE THAN 3.
	 *
	 * We need to compute the corresponding cycle time constraints. Start
	 * by getting information about the parent clock.
	 */

	parent_clock            = clk_get_parent(this->clock);
	parent_clock_rate_in_hz = clk_get_rate(parent_clock);

	/* Compute the limit frequencies. */

	sym_low_clock_rate_in_hz  = parent_clock_rate_in_hz / 4;
	asym_low_clock_rate_in_hz = parent_clock_rate_in_hz / 3;

	/* Compute the corresponding limit cycle periods. */

	sym_high_cycle_in_ns  = 1000000000 / sym_low_clock_rate_in_hz;
	asym_high_cycle_in_ns = (1000000000 / asym_low_clock_rate_in_hz) * 2;

	/* Attempt to implement the given cycle. */

	return nfc_util_set_best_cycle(this, ns,
			ns > asym_high_cycle_in_ns, ns > sym_high_cycle_in_ns);

}

/**
 * nfc_3_2_mask_interrupt() - Masks interrupts.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_mask_interrupt(struct imx_nfc_data *this)
{
	void  *secondary_base = this->secondary_regs;
	raw_set_mask_l(NFC_3_2_CONFIG2_INT_MSK_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);
}

/**
 * nfc_3_2_unmask_interrupt() - Unmasks interrupts.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_unmask_interrupt(struct imx_nfc_data *this)
{
	void  *secondary_base = this->secondary_regs;
	raw_clr_mask_l(NFC_3_2_CONFIG2_INT_MSK_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);
}

/**
 * nfc_3_2_clear_interrupt() - Clears an interrupt.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_clear_interrupt(struct imx_nfc_data *this)
{
	int   done;
	void  *secondary_base = this->secondary_regs;

	/* Request IP bus interface access. */

	raw_set_mask_l(NFC_3_2_IPC_CREQ_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

	/* Wait for access. */

	do
		done = !!raw_read_mask_l(NFC_3_2_IPC_CACK_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);
	while (!done);

	/* Clear the interrupt. */

	raw_clr_mask_l(NFC_3_2_IPC_INT_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

	/* Release the IP bus interface. */

	raw_clr_mask_l(NFC_3_2_IPC_CREQ_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

}

/**
 * nfc_3_2_is_interrupting() - Returns the interrupt bit status.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_is_interrupting(struct imx_nfc_data *this)
{
	void  *secondary_base = this->secondary_regs;
	return !!raw_read_mask_l(NFC_3_2_IPC_INT_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);
}

/**
 * nfc_3_2_is_ready() - Returns the ready/busy status.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_is_ready(struct imx_nfc_data *this)
{
	void  *secondary_base = this->secondary_regs;
	return !!raw_read_mask_l(NFC_3_2_IPC_RB_B_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);
}

/**
 * nfc_3_2_set_force_ce() - Can force CE to be asserted always.
 *
 * @this:  Per-device data.
 * @on:    Indicates if the hardware CE signal should be asserted always.
 */
static void nfc_3_2_set_force_ce(struct imx_nfc_data *this, int on)
{
	void  *primary_base = this->primary_regs;

	if (on)
		raw_set_mask_l(NFC_3_2_CONFIG1_NF_CE_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);
	else
		raw_clr_mask_l(NFC_3_2_CONFIG1_NF_CE_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

}

/**
 * nfc_3_2_set_ecc() - Turns ECC on or off.
 *
 * @this:  Per-device data.
 * @on:    Indicates if ECC should be on or off.
 */
static void nfc_3_2_set_ecc(struct imx_nfc_data *this, int on)
{
	void  *secondary_base = this->secondary_regs;

	if (on)
		raw_set_mask_l(NFC_3_2_CONFIG2_ECC_EN_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);
	else
		raw_clr_mask_l(NFC_3_2_CONFIG2_ECC_EN_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

}

/**
 * nfc_3_2_get_ecc_status() - Reports ECC errors.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_get_ecc_status(struct imx_nfc_data *this)
{
	unsigned int  i;
	void          *base = this->primary_regs;
	uint16_t      status_reg;
	unsigned int  buffer_status[8];
	int           status;

	/* Get the entire status register. */

	status_reg = __raw_readw(base + NFC_3_2_ECC_STATUS_REG_OFF);

	/* Pick out the status for each buffer. */

	buffer_status[0] = (status_reg & NFC_3_2_ECC_STATUS_NOBER1_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER1_POS;

	buffer_status[1] = (status_reg & NFC_3_2_ECC_STATUS_NOBER2_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER2_POS;

	buffer_status[2] = (status_reg & NFC_3_2_ECC_STATUS_NOBER3_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER3_POS;

	buffer_status[3] = (status_reg & NFC_3_2_ECC_STATUS_NOBER4_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER4_POS;

	buffer_status[4] = (status_reg & NFC_3_2_ECC_STATUS_NOBER5_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER5_POS;

	buffer_status[5] = (status_reg & NFC_3_2_ECC_STATUS_NOBER6_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER6_POS;

	buffer_status[6] = (status_reg & NFC_3_2_ECC_STATUS_NOBER7_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER7_POS;

	buffer_status[7] = (status_reg & NFC_3_2_ECC_STATUS_NOBER8_MSK)
					>> NFC_3_2_ECC_STATUS_NOBER8_POS;

	/* Loop through the buffers we're actually using. */

	status = 0;

	for (i = 0; i < this->nfc_geometry->buffer_count; i++) {

		if (buffer_status[i] > this->nfc_geometry->ecc_strength) {
			status = -1;
			break;
		}

		status += buffer_status[i];

	}

	/* Return the final result. */

	return status;

}

/**
 * nfc_3_2_get_symmetric() - Indicates if the clock is symmetric.
 *
 * @this:  Per-device data.
 */
static int nfc_3_2_get_symmetric(struct imx_nfc_data *this)
{
	void  *secondary_base = this->secondary_regs;

	return !!raw_read_mask_w(NFC_3_2_CONFIG2_SYM_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

}

/**
 * nfc_3_2_set_symmetric() - Turns symmetric clock mode on or off.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_set_symmetric(struct imx_nfc_data *this, int on)
{
	void  *secondary_base = this->secondary_regs;

	if (on)
		raw_set_mask_l(NFC_3_2_CONFIG2_SYM_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);
	else
		raw_clr_mask_l(NFC_3_2_CONFIG2_SYM_MSK,
				secondary_base + NFC_3_2_CONFIG2_REG_OFF);

}

/**
 * nfc_3_2_select_chip() - Selects the current chip.
 *
 * @this:  Per-device data.
 * @chip:  The chip number to select, or -1 to select no chip.
 */
static void nfc_3_2_select_chip(struct imx_nfc_data *this, int chip)
{
	unsigned long  x;
	void           *primary_base = this->primary_regs;

	if (chip < 0)
		return;

	x = __raw_readl(primary_base + NFC_3_2_CONFIG1_REG_OFF);

	x &= ~NFC_3_2_CONFIG1_CS_MSK;

	x |= (chip << NFC_3_2_CONFIG1_CS_POS) & NFC_3_2_CONFIG1_CS_MSK;

	__raw_writel(x, primary_base + NFC_3_2_CONFIG1_REG_OFF);

}

/**
 * nfc_3_2_command_cycle() - Sends a command.
 *
 * @this:     Per-device data.
 * @command:  The command code.
 */
static void nfc_3_2_command_cycle(struct imx_nfc_data *this, unsigned command)
{
	void  *primary_base = this->primary_regs;

	/* Write the command we want to send. */

	__raw_writel(command, primary_base + NFC_3_2_CMD_REG_OFF);

	/* Launch a command cycle. */

	__raw_writel(NFC_3_2_LAUNCH_FCMD_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

}

/**
 * nfc_3_2_write_cycle() - writes a single byte.
 *
 * @this:  Per-device data.
 * @byte:  The byte.
 */
static void nfc_3_2_write_cycle(struct imx_nfc_data *this, unsigned int byte)
{
	void  *primary_base = this->primary_regs;

	/* Give the NFC the byte we want to write. */

	__raw_writel(byte, primary_base + NFC_3_2_ADD0_REG_OFF);

	/* Launch an address cycle.
	 *
	 * This is *sort* of a hack, but not really. The intent of the NFC
	 * design is for this operation to send an address byte. In fact, the
	 * NFC neither knows nor cares what we're sending. It justs runs a write
	 * cycle.
	 */

	__raw_writel(NFC_3_2_LAUNCH_FADD_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, false);

}

/**
 * nfc_3_2_read_cycle() - Applies a single read cycle to the current chip.
 *
 * @this:  Per-device data.
 */
static unsigned int nfc_3_2_read_cycle(struct imx_nfc_data *this)
{
	unsigned int  result;
	void          *primary_base = this->primary_regs;

	/* Launch a "Data Out" operation. */

	__raw_writel(0x4 << NFC_3_2_LAUNCH_FDO_POS,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, false);

	/* Get the result. */

	result = __raw_readl(primary_base + NFC_3_2_CONFIG1_REG_OFF)
						>> NFC_3_2_CONFIG1_STATUS_POS;
	result &= 0xff;

	/* Return the result. */

	return result;

}

/**
 * nfc_3_2_read_page() - Reads a page into the NFC memory.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_read_page(struct imx_nfc_data *this)
{
	void  *primary_base = this->primary_regs;

	/* Start reading into buffer 0. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_RBA_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Launch a page data out operation. */

	__raw_writel(0x1 << NFC_3_2_LAUNCH_FDO_POS,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, true);

}

/**
 * nfc_3_2_send_page() - Sends a page from the NFC to the current chip.
 *
 * @this:  Per-device data.
 */
static void nfc_3_2_send_page(struct imx_nfc_data *this)
{
	void  *primary_base = this->primary_regs;

	/* Start sending from buffer 0. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_RBA_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Launch a page data in operation. */

	__raw_writel(NFC_3_2_LAUNCH_FDI_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, true);

}

/**
 * nfc_3_2_add_state_events() - Adds events to display important state.
 *
 * @this:   Per-device data.
 */
static void nfc_3_2_add_state_events(struct imx_nfc_data *this)
{
#ifdef EVENT_REPORTING
	void  *secondary_base = this->secondary_regs;

	add_state_event_l
		(
		secondary_base + NFC_3_2_IPC_REG_OFF,
		NFC_3_2_IPC_INT_MSK,
		"  Interrupt     : 0",
		"  Interrupt     : X"
		);

	add_state_event_l
		(
		secondary_base + NFC_3_2_IPC_REG_OFF,
		NFC_3_2_IPC_AUTO_PROG_DONE_MSK,
		"  auto_prog_done: 0",
		"  auto_prog_done: X"
		);

	add_state_event_l
		(
		secondary_base + NFC_3_2_IPC_REG_OFF,
		NFC_3_2_IPC_RB_B_MSK,
		"  Medium        : Busy",
		"  Medium        : Ready"
		);
#endif
}

/**
 * nfc_3_2_get_auto_loop_params() - Gets automatic operation loop parameters.
 *
 * This function and the corresponding "setter" enable the automatic operations
 * to keep some state as they iterate over chips.
 *
 * The most "obvious" way to save state would be to allocate a private data
 * structure and hang it off the owning struct nfc_hal. On the other hand,
 * writing the code to allocate the memory and then release it when the NFC
 * shuts down is annoying - and we have some perfectly good memory in the NFC
 * hardware that we can use. Since we only use two commands at a time, we can
 * stash our loop limits and loop index in the top 16 bits of the NAND_CMD
 * register. To paraphrase the reference manual:
 *
 *
 * NAND_CMD
 *
 * |<--  4 bits  -->|<--  4 bits -->|<--          8 bits          -->|
 * +----------------+---------------+--------------------------------+
 * |     First      |     Last      |           Loop Index           |
 * +----------------+---------------+--------------------------------+
 * |         NAND COMMAND1          |         NAND COMMAND0          |
 * +--------------------------------+--------------------------------+
 * |<--         16 bits          -->|<--         16 bits          -->|
 *
 *
 * @this:   Per-device data.
 * @first:  A pointer to a variable that will receive the first chip number.
 * @last:   A pointer to a variable that will receive the last chip number.
 * @index:  A pointer to a variable that will receive the current chip number.
 */
static void nfc_3_2_get_auto_loop_params(struct imx_nfc_data *this,
			unsigned *first, unsigned *last, unsigned *index)
{
	uint32_t  x;
	void      *primary_base = this->primary_regs;

	x = __raw_readl(primary_base + NFC_3_2_CMD_REG_OFF);

	*first = (x >> 28) & 0x0f;
	*last  = (x >> 24) & 0x0f;
	*index = (x >> 16) & 0xff;

}

/**
 * nfc_3_2_set_auto_loop_params() - Sets automatic operation loop parameters.
 *
 * See nfc_3_2_get_auto_loop_params() for detailed information about these
 * functions.
 *
 * @this:   Per-device data.
 * @first:  The first chip number.
 * @last:   The last chip number.
 * @index:  The current chip number.
 */
static void nfc_3_2_set_auto_loop_params(struct imx_nfc_data *this,
				unsigned first, unsigned last, unsigned index)
{
	uint32_t  x;
	void      *primary_base = this->primary_regs;

	x = __raw_readl(primary_base + NFC_3_2_CMD_REG_OFF);

	x &= 0x0000ffff;
	x |= (first & 0x0f) << 28;
	x |= (last  & 0x0f) << 24;
	x |= (index & 0xff) << 16;

	__raw_writel(x, primary_base + NFC_3_2_CMD_REG_OFF);

}

/**
 * nfc_3_2_get_auto_addresses() - Gets automatic operation addresses.
 *
 * @this:    Per-device data.
 * @group:   The address group number.
 * @chip:    A pointer to a variable that will receive the chip number.
 * @column:  A pointer to a variable that will receive the column address.
 *           A NULL pointer indicates there is no column address.
 * @page:    A pointer to a variable that will receive the page address.
 */
static void nfc_3_2_get_auto_addresses(struct imx_nfc_data *this,
	unsigned group, unsigned *chip, unsigned *column, unsigned *page)
{
	uint32_t                  x;
	unsigned int              chip_count;
	unsigned int              cs_width;
	unsigned int              cs_mask;
	unsigned int              page_lsbs;
	unsigned int              page_msbs;
	uint32_t                  *low;
	uint16_t                  *high;
	void                      *primary_base   = this->primary_regs;
	void                      *secondary_base = this->secondary_regs;

	/*
	 * The width of the chip select field depends on the number of connected
	 * chips.
	 *
	 * Notice that these computations work only if the number of chips is a
	 * power of 2. In fact, that is a fundamental limitation for using
	 * automatic operations.
	 */

	x = __raw_readl(secondary_base + NFC_3_2_CONFIG3_REG_OFF);

	chip_count =
		(x & NFC_3_2_CONFIG3_NUM_OF_DEVICES_MSK) >>
					NFC_3_2_CONFIG3_NUM_OF_DEVICES_POS;
	chip_count++;

	cs_width = ffs(chip_count) - 1;
	cs_mask  = chip_count - 1;

	/* Construct pointers to the pieces of the given address group. */

	low = primary_base + NFC_3_2_ADD0_REG_OFF;
	low += group;

	high = primary_base + NFC_3_2_ADD8_REG_OFF;
	high += group;

	/* Check if there's a column address. */

	if (column) {

		/*
		 * The low 32 bits of the address group look like this:
		 *
		 *      16 - n     n
		 * | <-  bits  ->|<->|<-  16 bits   ->|
		 * +-------------+---+----------------+
		 * |  Page LSBs  |CS |     Column     |
		 * +-------------+---+----------------+
		 */

		x = __raw_readl(low);

		*column   = x & 0xffff;
		*chip     = (x >> 16) & cs_mask;
		page_lsbs = x >> (16 + cs_width);

		/* The high 16 bits contain the MSB's of the page address. */

		page_msbs = __raw_readw(high);

		*page = (page_msbs << (16 - cs_width)) | page_lsbs;

	} else {

		/*
		 * The low 32 bits of the address group look like this:
		 *
		 *                                 n
		 * | <-     (32 - n) bits      ->|<->|
		 * +-----------------------------+---+
		 * |          Page LSBs          |CS |
		 * +-----------------------------+---+
		 */

		x = __raw_readl(low);

		*chip     = x &  cs_mask;
		page_lsbs = x >> cs_width;

		/* The high 16 bits contain the MSB's of the page address. */

		page_msbs = __raw_readw(high);

		*page = (page_msbs << (32 - cs_width)) | page_lsbs;

	}

}

/**
 * nfc_3_2_set_auto_addresses() - Sets automatic operation addresses.
 *
 * @this:    Per-device data.
 * @group:   The address group number.
 * @chip:    The chip number.
 * @column:  The column address. The sentinel value ~0 indicates that there is
 *           no column address.
 * @page:    The page address.
 */
static void nfc_3_2_set_auto_addresses(struct imx_nfc_data *this,
		unsigned group, unsigned chip, unsigned column, unsigned page)
{
	uint32_t                  x;
	unsigned                  chip_count;
	unsigned int              cs_width;
	unsigned int              cs_mask;
	uint32_t                  *low;
	uint16_t                  *high;
	void                      *primary_base = this->primary_regs;
	void                      *secondary_base = this->secondary_regs;

	/*
	 * The width of the chip select field depends on the number of connected
	 * chips.
	 *
	 * Notice that these computations work only if the number of chips is a
	 * power of 2. In fact, that is a fundamental limitation for using
	 * automatic operations.
	 */

	x = __raw_readl(secondary_base + NFC_3_2_CONFIG3_REG_OFF);

	chip_count =
		(x & NFC_3_2_CONFIG3_NUM_OF_DEVICES_MSK) >>
					NFC_3_2_CONFIG3_NUM_OF_DEVICES_POS;
	chip_count++;

	cs_width = ffs(chip_count) - 1;
	cs_mask  = chip_count - 1;

	/* Construct pointers to the pieces of the given address group. */

	low = primary_base + NFC_3_2_ADD0_REG_OFF;
	low += group;

	high = primary_base + NFC_3_2_ADD8_REG_OFF;
	high += group;

	/* Check if we have a column address. */

	if (column != ~0) {

		/*
		 * The low 32 bits of the address group look like this:
		 *
		 *      16 - n     n
		 * | <-  bits  ->|<->|<-  16 bits   ->|
		 * +-------------+---+----------------+
		 * |  Page LSBs  |CS |     Column     |
		 * +-------------+---+----------------+
		 */

		x  = 0;
		x |= column & 0xffff;
		x |= (chip & cs_mask) << 16;
		x |= page << (16 + cs_width);

		__raw_writel(x, low);

		/* The high 16 bits contain the MSB's of the page address. */

		x = (page >> (16 - cs_width)) & 0xffff;

		__raw_writew(x, high);

	} else {

		/*
		 * The low 32 bits of the address group look like this:
		 *
		 *                                 n
		 * | <-     (32 - n) bits      ->|<->|
		 * +-----------------------------+---+
		 * |          Page LSBs          |CS |
		 * +-----------------------------+---+
		 */

		x  = 0;
		x |= chip & cs_mask;
		x |= page << cs_width;

		__raw_writel(x, low);

		/* The high 16 bits contain the MSB's of the page address. */

		x = (page >> (32 - cs_width)) & 0xffff;

		__raw_writew(x, high);

	}

}

/**
 * nfc_3_2_start_auto_read() - Starts an automatic read.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 * @start:  The first physical chip number on which to operate.
 * @count:  The number of physical chips on which to operate.
 * @column: The column address.
 * @page:   The page address.
 */
static int nfc_3_2_start_auto_read(struct imx_nfc_data *this,
		unsigned start, unsigned count, unsigned column, unsigned page)
{
	uint32_t  x;
	int       return_value = 0;
	void      *primary_base = this->primary_regs;

	add_event("Entering nfc_3_2_start_auto_read", 1);

	/* Check for nonsense. */

	if ((start > 7) || (!count) || (count > 8)) {
		return_value = !0;
		goto exit;
	}

	/* Set state. */

	nfc_3_2_set_auto_loop_params(this, start, start + count - 1, start);
	nfc_3_2_set_auto_addresses(this, 0, start, column, page);

	/* Set up for ONE iteration at a time. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_ITER_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Reset to buffer 0. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_RBA_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/*
	 * Set up the commands. Note that the number of command phases was
	 * configured in the set_geometry() function so, even though we're
	 * giving both commands here, they won't necessarily both be used.
	 */

	x = __raw_readl(primary_base + NFC_3_2_CMD_REG_OFF);

	x &= 0xffff0000;
	x |= NAND_CMD_READ0     << 0;
	x |= NAND_CMD_READSTART << 8;

	__raw_writel(x, primary_base + NFC_3_2_CMD_REG_OFF);

	/* Launch the operation. */

	add_event("Launching", 0);

	__raw_writel(NFC_3_2_LAUNCH_AUTO_READ_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

exit:	/* Return. */

	add_event("Exiting nfc_3_2_start_auto_read", -1);

	return return_value;

}

/**
 * nfc_3_2_wait_for_auto_read() - Waits until auto read is ready for the CPU.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 */
static int nfc_3_2_wait_for_auto_read(struct imx_nfc_data *this)
{
	unsigned int  first;
	unsigned int  last;
	unsigned int  index;
	int           return_value = 0;

	add_event("Entering nfc_3_2_wait_for_auto_read", 1);

	/* Get state. */

	nfc_3_2_get_auto_loop_params(this, &first, &last, &index);

	/* This function should be called for every chip. */

	if ((index < first) || (index > last)) {
		return_value = !0;
		goto exit;
	}

	/* Wait for the NFC to completely finish and interrupt. */

	nfc_util_wait_for_the_nfc(this, true);

exit:	/* Return. */

	add_event("Exiting nfc_3_2_wait_for_auto_read", -1);

	return return_value;

}

/**
 * nfc_3_2_resume_auto_read() - Resumes auto read after CPU intervention.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 */
static int nfc_3_2_resume_auto_read(struct imx_nfc_data *this)
{
	unsigned int  first;
	unsigned int  last;
	unsigned int  index;
	unsigned int  chip;
	unsigned int  column;
	unsigned int  page;
	int           return_value = 0;
	void          *primary_base = this->primary_regs;

	add_event("Entering nfc_3_2_resume_auto_read", 1);

	/* Get state. */

	nfc_3_2_get_auto_loop_params(this, &first, &last, &index);
	nfc_3_2_get_auto_addresses(this, 0, &chip, &column, &page);

	/* This function should be called for every chip, except the last. */

	if ((index < first) || (index >= last)) {
		return_value = !0;
		goto exit;
	}

	/* Move to the next chip. */

	index++;

	/* Update state. */

	nfc_3_2_set_auto_loop_params(this, first, last, index);
	nfc_3_2_set_auto_addresses(this, 0, index, column, page);

	/* Reset to buffer 0. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_RBA_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Launch the operation. */

	add_event("Launching", 0);

	__raw_writel(NFC_3_2_LAUNCH_AUTO_READ_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

exit:	/* Return. */

	add_event("Exiting nfc_3_2_resume_auto_read", -1);

	return return_value;

}

/**
 * nfc_3_2_start_auto_write() - Starts an automatic write.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 * @start:  The first physical chip number on which to operate.
 * @count:  The number of physical chips on which to operate.
 * @column: The column address.
 * @page:   The page address.
 */
static int nfc_3_2_start_auto_write(struct imx_nfc_data *this,
		unsigned start, unsigned count, unsigned column, unsigned page)
{
	uint32_t  x;
	int       return_value = 0;
	void      *primary_base   = this->primary_regs;
	void      *secondary_base = this->secondary_regs;

	add_event("Entering nfc_3_2_start_auto_write", 1);

	/* Check for nonsense. */

	if ((start > 7) || (!count) || (count > 8)) {
		return_value = !0;
		goto exit;
	}

	/* Set state. */

	nfc_3_2_set_auto_loop_params(this, start, start + count - 1, start);
	nfc_3_2_set_auto_addresses(this, 0, start, column, page);

	/* Set up for ONE iteration at a time. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_ITER_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Set up the commands. */

	x = __raw_readl(primary_base + NFC_3_2_CMD_REG_OFF);

	x &= 0xffff0000;
	x |= NAND_CMD_SEQIN    << 0;
	x |= NAND_CMD_PAGEPROG << 8;

	__raw_writel(x, primary_base + NFC_3_2_CMD_REG_OFF);

	/* Clear the auto_prog_done bit. */

	raw_clr_mask_l(NFC_3_2_IPC_AUTO_PROG_DONE_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

exit:	/* Return. */

	add_event("Exiting nfc_3_2_start_auto_write", -1);

	return return_value;

}

/**
 * nfc_3_2_wait_for_auto_write() - Waits for auto write to be writey for the CPU.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 */
static int nfc_3_2_wait_for_auto_write(struct imx_nfc_data *this)
{
	unsigned int  first;
	unsigned int  last;
	unsigned int  index;
	unsigned int  chip;
	unsigned int  column;
	unsigned int  page;
	uint32_t      x;
	int           interrupt;
	int           transmitted;
	int           ready;
	int           return_value = 0;
	void          *primary_base   = this->primary_regs;
	void          *secondary_base = this->secondary_regs;

	add_event("Entering nfc_3_2_wait_for_auto_write", 1);

	/* Get state. */

	nfc_3_2_get_auto_loop_params(this, &first, &last, &index);
	nfc_3_2_get_auto_addresses(this, 0, &chip, &column, &page);

	/* This function should be called for every chip. */

	if ((index < first) || (index > last)) {
		return_value = !0;
		goto exit;
	}

	/* Reset to buffer 0. */

	raw_clr_mask_l(NFC_3_2_CONFIG1_RBA_MSK,
					primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Launch the operation. */

	nfc_3_2_add_state_events(this);

	add_event("Launching", 0);

	__raw_writel(NFC_3_2_LAUNCH_AUTO_PROG_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

	nfc_3_2_add_state_events(this);

	/* Wait for the NFC to transmit the page. */

	add_event("Spinning while the NFC transmits the page...", 0);

	do
		transmitted = !!raw_read_mask_l(NFC_3_2_IPC_AUTO_PROG_DONE_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);
	while (!transmitted);

	/*
	 * When control arrives here, the auto_prog_done bit is set. This
	 * indicates the NFC has finished transmitting the current page. The CPU
	 * is now free to write the next page into the NFC's memory. The Flash
	 * hardware is still busy programming the page into its storage array.
	 *
	 * Clear the auto_prog_done bit. This is analogous to acknowledging an
	 * interrupt.
	 */

	nfc_3_2_add_state_events(this);

	add_event("Acknowledging the page...", 0);

	raw_clr_mask_l(NFC_3_2_IPC_AUTO_PROG_DONE_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

	nfc_3_2_add_state_events(this);

	/*
	 * If this is *not* the last iteration, move to the next chip and return
	 * to the caller so he can put the next page in the NFC buffer.
	 */

	if (index < last) {

		add_event("Moving to the next chip...", 0);

		index++;

		nfc_3_2_set_auto_loop_params(this, first, last, index);
		nfc_3_2_set_auto_addresses(this, 0, index, column, page);

		goto exit;

	}

	/*
	 * If control arrives here, this is the last iteration, so it's time to
	 * close out the entire operation. We need to wait for the medium to be
	 * ready and then acknowledge the final interrupt.
	 *
	 * Because of the way the NFC hardware works, the code here requires a
	 * bit of explanation. The most important rule is:
	 *
	 *         During automatic operations, the NFC sets its
	 *         interrupt bit *whenever* it sees the ready/busy
	 *         signal transition from "Busy" to "Ready".
	 *
	 * Recall that the ready/busy signals from all the chips in the medium
	 * are "wire-anded." Thus, the NFC will only see that the medium is
	 * ready if *all* chips are ready.
	 *
	 * Because of variability in NAND Flash timing, the medium *may* have
	 * become ready during previous iterations, which means the interrupt
	 * bit *may* be set at this moment. This is a "left-over" interrupt, and
	 * can complicate our logic.
	 *
	 * The two bits of state that interest us here are the interrupt bit
	 * and the ready/busy bit. It boils down to the following truth table:
	 *
	 * | Interrupt  | Ready/Busy | Description
	 * +------------+------------+---------------
	 * |            |            | Busy medium and no left-over interrupt.
	 * |     0      |     0      | The final interrupt will arrive in the
	 * |            |            | future.
	 * +------------+------------+---------------
	 * |            |            | Ready medium and no left-over interrupt.
	 * |     0      |     1      | There will be no final interrupt. This
	 * |            |            | case should be impossible.
	 * +------------+------------+---------------
	 * |            |            | Busy medium and left-over interrupt.
	 * |     1      |     0      | The final interrupt will arrive in the
	 * |            |            | future. This is the hard case.
	 * +------------+------------+---------------
	 * |            |            | Ready medium and left-over interrupt.
	 * |     1      |     1      | The final interrupt has already
	 * |            |            | arrived. Acknowledge it and exit.
	 * +------------+------------+---------------
	 *
	 * Case #3 is a small problem. If we clear the interrupt, we may or may
	 * not have another interrupt following.
	 */

	/* Sample the IPC register. */

	x = __raw_readl(secondary_base + NFC_3_2_IPC_REG_OFF);

	interrupt = !!(x & NFC_3_2_IPC_INT_MSK);
	ready     = !!(x & NFC_3_2_IPC_RB_B_MSK);

	/* Check for the easy cases. */

	if (!interrupt && !ready) {
		add_event("Waiting for the final interrupt..." , 0);
		nfc_util_wait_for_the_nfc(this, true);
		goto exit;
	} else if (!interrupt && ready) {
		add_event("Done." , 0);
		goto exit;
	} else if (interrupt && ready) {
		add_event("Acknowledging the final interrupt..." , 0);
		nfc_util_wait_for_the_nfc(this, false);
		goto exit;

	}

	/*
	 * If control arrives here, we hit case #3. Begin by acknowledging the
	 * interrupt we have right now.
	 */

	add_event("Clearing the left-over interrupt..." , 0);
	nfc_util_wait_for_the_nfc(this, false);

	/*
	 * Check the ready/busy bit again. If the medium is still busy, then
	 * we're going to get one more interrupt.
	 */

	ready = !!raw_read_mask_l(NFC_3_2_IPC_RB_B_MSK,
					secondary_base + NFC_3_2_IPC_REG_OFF);

	if (!ready) {
		add_event("Waiting for the final interrupt..." , 0);
		nfc_util_wait_for_the_nfc(this, true);
	}

exit:	/* Return. */

	add_event("Exiting nfc_3_2_wait_for_auto_write", -1);

	return return_value;

}

/**
 * nfc_3_2_start_auto_erase() - Starts an automatic erase.
 *
 * This function returns 0 if everything went well.
 *
 * @this:   Per-device data.
 * @start:  The first physical chip number on which to operate.
 * @count:  The number of physical chips on which to operate.
 * @page:   The page address.
 */
static int nfc_3_2_start_auto_erase(struct imx_nfc_data *this,
				unsigned start, unsigned count, unsigned page)
{
	uint32_t  x;
	unsigned  i;
	int       return_value = 0;
	void      *primary_base = this->primary_regs;

	add_event("Entering nfc_3_2_start_auto_erase", 1);

	/* Check for nonsense. */

	if ((start > 7) || (!count) || (count > 8)) {
		return_value = !0;
		goto exit;
	}

	/* Set up the commands. */

	x = __raw_readl(primary_base + NFC_3_2_CMD_REG_OFF);

	x &= 0xffff0000;
	x |= NAND_CMD_ERASE1 << 0;
	x |= NAND_CMD_ERASE2 << 8;

	__raw_writel(x, primary_base + NFC_3_2_CMD_REG_OFF);

	/* Set the iterations. */

	x = __raw_readl(primary_base + NFC_3_2_CONFIG1_REG_OFF);

	x &= ~NFC_3_2_CONFIG1_ITER_MSK;
	x |= ((count - 1) << NFC_3_2_CONFIG1_ITER_POS) &
						NFC_3_2_CONFIG1_ITER_MSK;

	__raw_writel(x, primary_base + NFC_3_2_CONFIG1_REG_OFF);

	/* Loop over chips, setting up the address groups. */

	for (i = 0; i < count; i++)
		nfc_3_2_set_auto_addresses(this, i, start + i, ~0, page);

	/* Launch the operation. */

	add_event("Launching", 0);

	__raw_writel(NFC_3_2_LAUNCH_AUTO_ERASE_MSK,
					primary_base + NFC_3_2_LAUNCH_REG_OFF);

exit:	/* Return. */

	add_event("Exiting nfc_3_2_start_auto_erase", -1);

	return return_value;

}

/*
 * At this point, we've defined all the version-specific primitives. We're now
 * ready to construct the NFC HAL structures for every version.
 */

struct nfc_hal nfc_1_0_hal = {
		.major_version       = 1,
		.minor_version       = 0,
		.max_chip_count      = 1,
		.max_buffer_count    = 4,
		.spare_buf_stride    = 16,
		.has_secondary_regs  = 0,
		.can_be_symmetric    = 0,
	};

struct nfc_hal nfc_2_0_hal = {
		.major_version       = 2,
		.minor_version       = 0,
		.max_chip_count      = 1,
		.max_buffer_count    = 4,
		.spare_buf_stride    = 16,
		.has_secondary_regs  = false,
		.can_be_symmetric    = true,
		.init                = nfc_2_0_init,
		.set_geometry        = nfc_2_0_set_geometry,
		.exit                = nfc_2_x_exit,
		.mask_interrupt      = nfc_2_0_mask_interrupt,
		.unmask_interrupt    = nfc_2_0_unmask_interrupt,
		.clear_interrupt     = nfc_2_x_clear_interrupt,
		.is_interrupting     = nfc_2_x_is_interrupting,
		.is_ready            = 0, /* Ready/Busy not exposed. */
		.set_ecc             = nfc_2_0_set_ecc,
		.get_ecc_status      = nfc_2_0_get_ecc_status,
		.get_symmetric       = nfc_2_0_get_symmetric,
		.set_symmetric       = nfc_2_0_set_symmetric,
		.select_chip         = nfc_2_0_select_chip,
		.command_cycle       = nfc_2_x_command_cycle,
		.write_cycle         = nfc_2_x_write_cycle,
		.read_cycle          = nfc_2_0_read_cycle,
		.read_page           = nfc_2_0_read_page,
		.send_page           = nfc_2_0_send_page,
		.start_auto_read     = 0, /* Not supported. */
		.wait_for_auto_read  = 0, /* Not supported. */
		.resume_auto_read    = 0, /* Not supported. */
		.start_auto_write    = 0, /* Not supported. */
		.wait_for_auto_write = 0, /* Not supported. */
		.start_auto_erase    = 0, /* Not supported. */
	};

struct nfc_hal nfc_2_1_hal = {
		.major_version       = 2,
		.minor_version       = 1,
		.max_chip_count      = 4,
		.max_buffer_count    = 8,
		.spare_buf_stride    = 64,
		.has_secondary_regs  = 0,
		.can_be_symmetric    = !0,
	};

struct nfc_hal nfc_3_1_hal = {
		.major_version       = 3,
		.minor_version       = 1,
		.max_chip_count      = 4,
		.max_buffer_count    = 8,
		.spare_buf_stride    = 64,
		.has_secondary_regs  = !0,
		.can_be_symmetric    = !0,
	};

struct nfc_hal nfc_3_2_hal = {
		.major_version       = 3,
		.minor_version       = 2,
		.max_chip_count      = 8,
		.max_buffer_count    = 8,
		.spare_buf_stride    = 64,
		.has_secondary_regs  = true,
		.can_be_symmetric    = true,
		.init                = nfc_3_2_init,
		.set_geometry        = nfc_3_2_set_geometry,
		.exit                = nfc_3_2_exit,
		.set_closest_cycle   = nfc_3_2_set_closest_cycle,
		.mask_interrupt      = nfc_3_2_mask_interrupt,
		.unmask_interrupt    = nfc_3_2_unmask_interrupt,
		.clear_interrupt     = nfc_3_2_clear_interrupt,
		.is_interrupting     = nfc_3_2_is_interrupting,
		.is_ready            = nfc_3_2_is_ready,
		.set_force_ce        = nfc_3_2_set_force_ce,
		.set_ecc             = nfc_3_2_set_ecc,
		.get_ecc_status      = nfc_3_2_get_ecc_status,
		.get_symmetric       = nfc_3_2_get_symmetric,
		.set_symmetric       = nfc_3_2_set_symmetric,
		.select_chip         = nfc_3_2_select_chip,
		.command_cycle       = nfc_3_2_command_cycle,
		.write_cycle         = nfc_3_2_write_cycle,
		.read_cycle          = nfc_3_2_read_cycle,
		.read_page           = nfc_3_2_read_page,
		.send_page           = nfc_3_2_send_page,
		.start_auto_read     = nfc_3_2_start_auto_read,
		.wait_for_auto_read  = nfc_3_2_wait_for_auto_read,
		.resume_auto_read    = nfc_3_2_resume_auto_read,
		.start_auto_write    = nfc_3_2_start_auto_write,
		.wait_for_auto_write = nfc_3_2_wait_for_auto_write,
		.start_auto_erase    = nfc_3_2_start_auto_erase,
	};

/*
 * This array has a pointer to every NFC HAL structure. The probing process will
 * find the one that matches the version given by the platform.
 */

struct nfc_hal  *(nfc_hals[]) = {
	&nfc_1_0_hal,
	&nfc_2_0_hal,
	&nfc_2_1_hal,
	&nfc_3_1_hal,
	&nfc_3_2_hal,
};

/**
 * mal_init() - Initialize the Medium Abstraction Layer.
 *
 * @this:  Per-device data.
 */
static void mal_init(struct imx_nfc_data  *this)
{
	this->interrupt_override = DRIVER_CHOICE;
	this->auto_op_override   = DRIVER_CHOICE;
	this->inject_ecc_error   = 0;
}

/**
 * mal_set_physical_geometry() - Set up the physical medium geometry.
 *
 * This function retrieves the physical geometry information discovered by
 * nand_scan(), corrects it, and records it in the per-device data structure.
 *
 * @this:  Per-device data.
 */
static int mal_set_physical_geometry(struct imx_nfc_data  *this)
{
	struct mtd_info           *mtd  = &this->mtd;
	struct nand_chip          *nand = &this->nand;
	struct device             *dev  = this->dev;
	uint8_t                   manufacturer_id;
	uint8_t                   device_id;
	unsigned int              block_size_in_pages;
	unsigned int              chip_size_in_blocks;
	unsigned int              chip_size_in_pages;
	uint64_t                  medium_size_in_bytes;
	struct physical_geometry  *physical = &this->physical_geometry;

	/*
	 * Begin by transcribing exactly what the MTD code discovered. If there
	 * are any mistakes, we'll fix them in a moment.
	 */

	physical->chip_count     = nand->numchips;
	physical->chip_size      = nand->chipsize;
	physical->block_size     = mtd->erasesize;
	physical->page_data_size = mtd->writesize;
	physical->page_oob_size  = mtd->oobsize;

	/* Read some of the ID bytes from the first NAND Flash chip. */

	nand->select_chip(mtd, 0);

	nfc_util_send_cmd_and_addrs(this, NAND_CMD_READID, 0x00, -1);

	manufacturer_id = nand->read_byte(mtd);
	device_id       = nand->read_byte(mtd);

	/*
	 * Most manufacturers sell 4K page devices with 218 out-of-band bytes
	 * per page to accomodate ECC-8.
	 *
	 * Samsung and Hynix claim their parts have better reliability, so they
	 * only need ECC-4 and they have only 128 out-of-band bytes.
	 *
	 * The MTD code pays no attention to the manufacturer ID (something that
	 * eventually will have to change), so it believes that all 4K pages
	 * have 218 out-of-band bytes.
	 *
	 * We correct that mistake here.
	 */

	if (physical->page_data_size == 4096) {
		if ((manufacturer_id == NAND_MFR_SAMSUNG) ||
					(manufacturer_id == NAND_MFR_HYNIX)) {
			physical->page_oob_size = 128;
		}
	}

	/* Compute some interesting facts. */

	block_size_in_pages  =
		physical->block_size / physical->page_data_size;
	chip_size_in_pages   =
		physical->chip_size >> (fls(physical->page_data_size) - 1);
	chip_size_in_blocks  =
		physical->chip_size >> (fls(physical->block_size) - 1);
	medium_size_in_bytes =
		physical->chip_size * physical->chip_count;

	/* Report. */

	dev_dbg(dev, "-----------------\n");
	dev_dbg(dev, "Physical Geometry\n");
	dev_dbg(dev, "-----------------\n");
	dev_dbg(dev, "Chip Count             : %d\n", physical->chip_count);
	dev_dbg(dev, "Page Data Size in Bytes: %u (0x%x)\n",
			physical->page_data_size, physical->page_data_size);
	dev_dbg(dev, "Page OOB Size in Bytes : %u\n",
			physical->page_oob_size);
	dev_dbg(dev, "Block Size in Bytes    : %u (0x%x)\n",
			physical->block_size, physical->block_size);
	dev_dbg(dev, "Block Size in Pages    : %u (0x%x)\n",
			block_size_in_pages, block_size_in_pages);
	dev_dbg(dev, "Chip Size in Bytes     : %llu (0x%llx)\n",
			physical->chip_size, physical->chip_size);
	dev_dbg(dev, "Chip Size in Pages     : %u (0x%x)\n",
			chip_size_in_pages, chip_size_in_pages);
	dev_dbg(dev, "Chip Size in Blocks    : %u (0x%x)\n",
			chip_size_in_blocks, chip_size_in_blocks);
	dev_dbg(dev, "Medium Size in Bytes   : %llu (0x%llx)\n",
			medium_size_in_bytes, medium_size_in_bytes);

	/* Return success. */

	return 0;

}

/**
 * mal_set_nfc_geometry() - Set up the NFC geometry.
 *
 * This function calls the NFC HAL to select an NFC geometry that is compatible
 * with the medium's physical geometry.
 *
 * @this:  Per-device data.
 */
static int mal_set_nfc_geometry(struct imx_nfc_data  *this)
{
	struct device        *dev = this->dev;
	struct nfc_geometry  *nfc;

	/* Set the NFC geometry. */

	if (this->nfc->set_geometry(this))
		return !0;

	/* Get a pointer to the new NFC geometry information. */

	nfc = this->nfc_geometry;

	/* Report. */

	dev_dbg(dev, "------------\n");
	dev_dbg(dev, "NFC Geometry\n");
	dev_dbg(dev, "------------\n");
	dev_dbg(dev, "Page Data Size in Bytes: %u (0x%x)\n",
				nfc->page_data_size, nfc->page_data_size);
	dev_dbg(dev, "Page OOB Size in Bytes : %u\n", nfc->page_oob_size);
	dev_dbg(dev, "ECC Algorithm          : %s\n", nfc->ecc_algorithm);
	dev_dbg(dev, "ECC Strength           : %d\n", nfc->ecc_strength);
	dev_dbg(dev, "Buffer Count           : %u\n", nfc->buffer_count);
	dev_dbg(dev, "Spare Buffer Size      : %u\n", nfc->spare_buf_size);
	dev_dbg(dev, "Spare Buffer Spillover : %u\n", nfc->spare_buf_spill);
	dev_dbg(dev, "Auto Read Available    : %s\n",
				this->nfc->start_auto_read  ? "Yes" : "No");
	dev_dbg(dev, "Auto Write Available   : %s\n",
				this->nfc->start_auto_write ? "Yes" : "No");
	dev_dbg(dev, "Auto Erase Available   : %s\n",
				this->nfc->start_auto_erase ? "Yes" : "No");

	/* Return success. */

	return 0;

}

/**
 * mal_set_logical_geometry() - Set up the logical medium geometry.
 *
 * This function constructs the logical geometry that we will expose to MTD,
 * based on the physical and NFC geometries, and whether or not interleaving is
 * on.
 *
 * @this:  Per-device data.
 */
static int mal_set_logical_geometry(struct imx_nfc_data  *this)
{
	const uint32_t            max_medium_size_in_bytes = ~0;
	int                       we_are_interleaving;
	uint64_t                  physical_medium_size_in_bytes;
	unsigned int              usable_blocks;
	unsigned int              block_size_in_pages;
	unsigned int              chip_size_in_blocks;
	unsigned int              chip_size_in_pages;
	unsigned int              usable_medium_size_in_pages;
	unsigned int              usable_medium_size_in_blocks;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *nfc      =  this->nfc_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;
	struct device             *dev      =  this->dev;

	/* Figure out if we're interleaving. */

	we_are_interleaving = this->pdata->interleave;

	switch (imx_nfc_module_interleave_override) {

	case NEVER:
		we_are_interleaving = false;
		break;

	case DRIVER_CHOICE:
		break;

	case ALWAYS:
		we_are_interleaving = true;
		break;

	}

	/* Compute the physical size of the medium. */

	physical_medium_size_in_bytes =
				physical->chip_count * physical->chip_size;

	/* Compute the logical geometry. */

	if (!we_are_interleaving) {

		/*
		 * At this writing, MTD uses unsigned 32-bit variables to
		 * represent the size of the medium. If the physical medium is
		 * larger than that, the logical medium must be smaller. Here,
		 * we compute the total number of physical blocks in the medium
		 * that we can actually use.
		 */

		if (physical_medium_size_in_bytes <= max_medium_size_in_bytes) {
			usable_blocks =
				physical_medium_size_in_bytes >>
						(ffs(physical->block_size) - 1);
		} else {
			usable_blocks =
				max_medium_size_in_bytes / physical->block_size;
		}

		/* Set up the logical geometry.
		 *
		 * Notice that the usable medium size is not necessarily the
		 * same as the chip size multiplied by the number of physical
		 * chips. We can't afford to touch the physical chip size
		 * because the NAND Flash MTD code *requires* it to be a power
		 * of 2.
		 */

		logical->chip_count     = physical->chip_count;
		logical->chip_size      = physical->chip_size;
		logical->usable_size    = usable_blocks * physical->block_size;
		logical->block_size     = physical->block_size;
		logical->page_data_size = nfc->page_data_size;

		/* Use the MTD layout that best matches the NFC geometry. */

		logical->mtd_layout    = &nfc->mtd_layout;
		logical->page_oob_size = nfc->mtd_layout.eccbytes +
						nfc->mtd_layout.oobavail;

	} else {

		/*
		 * If control arrives here, we are interleaving. Specifically,
		 * we are "horizontally concatenating" the pages in all the
		 * physical chips.
		 *
		 * - A logical page will be the size of a physical page
		 *   multiplied by the number of physical chips.
		 *
		 * - A logical block will have the same number of pages as a
		 *   physical block but, since the logical page size is larger,
		 *   the logical block size is larger.
		 *
		 * - The entire medium will appear to be a single chip.
		 *
		 * At this writing, MTD uses unsigned 32-bit variables to
		 * represent the size of the medium. If the physical medium is
		 * larger than that, the logical medium must be smaller.
		 *
		 * The NAND Flash MTD code represents the size of a single chip
		 * as an unsigned 32-bit value. It also *requires* that the size
		 * of a chip be a power of two. Thus, the largest possible chip
		 * size is 2GiB.
		 *
		 * When interleaving, the entire medium appears to be one chip.
		 * Thus, when interleaving, the largest possible medium size is
		 * 2GiB.
		 */

		if (physical_medium_size_in_bytes <= max_medium_size_in_bytes) {
			logical->chip_size =
				0x1 << (fls(physical_medium_size_in_bytes) - 1);
		} else {
			logical->chip_size =
				0x1 << (fls(max_medium_size_in_bytes) - 1);
		}

		/*
		 * If control arrives here, we're interleaving. The logical
		 * geometry is very different from the physical geometry.
		 */

		logical->chip_count     = 1;
		logical->usable_size    = logical->chip_size;
		logical->block_size     =
				physical->block_size * physical->chip_count;
		logical->page_data_size =
				nfc->page_data_size * physical->chip_count;

		/*
		 * Since the logical geometry doesn't match the physical
		 * geometry, we can't use the MTD layout that matches the
		 * NFC geometry. We synthesize one here.
		 *
		 * Our "logical" OOB will be the concatenation of the first 5
		 * bytes of the "physical" OOB of every chip. This has some
		 * important properties:
		 *
		 * - This will make the block mark of every physical chip
		 *   visible (even for small page chips, which put their block
		 *   mark in the 5th OOB byte).
		 *
		 * - None of the NFC controllers put ECC in the first 5 OOB
		 *   bytes, so this layout exposes no ECC.
		 */

		logical->page_oob_size = 5 * physical->chip_count;

		synthetic_layout.eccbytes = 0;
		synthetic_layout.oobavail = 5 * physical->chip_count;
		synthetic_layout.oobfree[0].offset = 0;
		synthetic_layout.oobfree[0].length = synthetic_layout.oobavail;

		/* Install the synthetic layout. */

		logical->mtd_layout = &synthetic_layout;

	}

	/* Compute some interesting facts. */

	block_size_in_pages = logical->block_size / logical->page_data_size;
	chip_size_in_pages  = logical->chip_size / logical->page_data_size;
	chip_size_in_blocks = logical->chip_size / logical->block_size;
	usable_medium_size_in_pages =
				logical->usable_size / logical->page_data_size;
	usable_medium_size_in_blocks =
				logical->usable_size / logical->block_size;

	/* Report. */

	dev_dbg(dev, "----------------\n");
	dev_dbg(dev, "Logical Geometry\n");
	dev_dbg(dev, "----------------\n");
	dev_dbg(dev, "Chip Count             : %d\n", logical->chip_count);
	dev_dbg(dev, "Page Data Size in Bytes: %u (0x%x)\n",
		logical->page_data_size, logical->page_data_size);
	dev_dbg(dev, "Page OOB Size in Bytes : %u\n",
		logical->page_oob_size);
	dev_dbg(dev, "Block Size in Bytes    : %u (0x%x)\n",
		logical->block_size, logical->block_size);
	dev_dbg(dev, "Block Size in Pages    : %u (0x%x)\n",
		block_size_in_pages, block_size_in_pages);
	dev_dbg(dev, "Chip Size in Bytes     : %u (0x%x)\n",
		logical->chip_size, logical->chip_size);
	dev_dbg(dev, "Chip Size in Pages     : %u (0x%x)\n",
		chip_size_in_pages, chip_size_in_pages);
	dev_dbg(dev, "Chip Size in Blocks    : %u (0x%x)\n",
		chip_size_in_blocks, chip_size_in_blocks);
	dev_dbg(dev, "Physical Size in Bytes : %llu (0x%llx)\n",
		physical_medium_size_in_bytes, physical_medium_size_in_bytes);
	dev_dbg(dev, "Usable Size in Bytes   : %u (0x%x)\n",
		logical->usable_size, logical->usable_size);
	dev_dbg(dev, "Usable Size in Pages   : %u (0x%x)\n",
		usable_medium_size_in_pages, usable_medium_size_in_pages);
	dev_dbg(dev, "Usable Size in Blocks  : %u (0x%x)\n",
		usable_medium_size_in_blocks, usable_medium_size_in_blocks);

	/* Return success. */

	return 0;

}

/**
 * mal_set_mtd_geometry() - Set up the MTD geometry.
 *
 * This function adjusts the owning MTD data structures to match the logical
 * geometry we've chosen.
 *
 * @this:  Per-device data.
 */
static int mal_set_mtd_geometry(struct imx_nfc_data *this)
{
	struct logical_geometry  *logical = &this->logical_geometry;
	struct mtd_info          *mtd     = &this->mtd;
	struct nand_chip         *nand    = &this->nand;

	/* Configure the struct mtd_info. */

	mtd->size        = logical->usable_size;
	mtd->erasesize   = logical->block_size;
	mtd->writesize   = logical->page_data_size;
	mtd->ecclayout   = logical->mtd_layout;
	mtd->oobavail    = mtd->ecclayout->oobavail;
	mtd->oobsize     = mtd->ecclayout->oobavail + mtd->ecclayout->eccbytes;
	mtd->subpage_sft = 0; /* We don't support sub-page writing. */

	/* Configure the struct nand_chip. */

	nand->numchips         = logical->chip_count;
	nand->chipsize         = logical->chip_size;
	nand->page_shift       = ffs(logical->page_data_size) - 1;
	nand->pagemask         = (nand->chipsize >> nand->page_shift) - 1;
	nand->subpagesize      = mtd->writesize >> mtd->subpage_sft;
	nand->phys_erase_shift = ffs(logical->block_size) - 1;
	nand->bbt_erase_shift  = nand->phys_erase_shift;
	nand->chip_shift       = ffs(logical->chip_size) - 1;
	nand->oob_poi          = nand->buffers->databuf+logical->page_data_size;
	nand->ecc.layout       = logical->mtd_layout;

	/* Set up the pattern that describes block marks. */

	if (is_small_page_chip(this))
		nand->badblock_pattern = &small_page_block_mark_descriptor;
	else
		nand->badblock_pattern = &large_page_block_mark_descriptor;

	/* Return success. */

	return 0;
}

/**
 * mal_set_geometry() - Set up the medium geometry.
 *
 * @this:  Per-device data.
 */
static int mal_set_geometry(struct imx_nfc_data  *this)
{

	/* Set up the various layers of geometry, in this specific order. */

	if (mal_set_physical_geometry(this))
		return !0;

	if (mal_set_nfc_geometry(this))
		return !0;

	if (mal_set_logical_geometry(this))
		return !0;

	if (mal_set_mtd_geometry(this))
		return !0;

	/* Return success. */

	return 0;

}

/**
 * mal_reset() - Resets the given chip.
 *
 * This is the fully-generalized reset operation, including support for
 * interleaving. All reset operations funnel through here.
 *
 * @this:  Per-device data.
 * @chip:  The logical chip of interest.
 */
static void mal_reset(struct imx_nfc_data *this, unsigned chip)
{
	int                       we_are_interleaving;
	unsigned int              start;
	unsigned int              end;
	unsigned int              i;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	add_event("Entering mal_get_status", 1);

	/* Establish some important facts. */

	we_are_interleaving = logical->chip_count != physical->chip_count;

	/* Choose the loop bounds. */

	if (we_are_interleaving) {
		start = 0;
		end   = physical->chip_count;
	} else {
		start = chip;
		end   = start + 1;
	}

	/* Loop over physical chips. */

	add_event("Looping over physical chips...", 0);

	for (i = start; i < end; i++) {

		/* Select the current chip. */

		this->nfc->select_chip(this, i);

		/* Reset the current chip. */

		add_event("Resetting...", 0);

		nfc_util_send_cmd(this, NAND_CMD_RESET);
		nfc_util_wait_for_the_nfc(this, false);

	}

	add_event("Exiting mal_get_status", -1);

}

/**
 * mal_get_status() - Abstracted status retrieval.
 *
 * For media with a single chip, or concatenated chips, the HIL explicitly
 * addresses a single chip at a time and wants the status from that chip only.
 *
 * For interleaved media, we must combine the individual chip states. At this
 * writing, the NAND MTD system knows about the following bits in status
 * registers:
 *
 *    +------------------------+-------+---------+
 *    |                        |       | Combine |
 *    |         Macro          | Value |  With   |
 *    +------------------------+-------+---------+
 *    | NAND_STATUS_FAIL       |  0x01 |    OR   |
 *    | NAND_STATUS_FAIL_N1    |  0x02 |    OR   |
 *    | NAND_STATUS_TRUE_READY |  0x20 |   AND   |
 *    | NAND_STATUS_READY      |  0x40 |   AND   |
 *    | NAND_STATUS_WP         |  0x80 |   AND   |
 *    +------------------------+-------+---------+
 *
 * @this:  Per-device data.
 * @chip:  The logical chip of interest.
 */
static uint8_t mal_get_status(struct imx_nfc_data *this, unsigned chip)
{
	int                       we_are_interleaving;
	unsigned int              start;
	unsigned int              end;
	unsigned int              i;
	unsigned int              x;
	unsigned int              or_mask;
	unsigned int              and_mask;
	uint8_t                   status;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	add_event("Entering mal_get_status", 1);

	/* Establish some important facts. */

	we_are_interleaving = logical->chip_count != physical->chip_count;

	/* Compute the masks we need. */

	or_mask  = NAND_STATUS_FAIL | NAND_STATUS_FAIL_N1;
	and_mask = NAND_STATUS_TRUE_READY | NAND_STATUS_READY | NAND_STATUS_WP;

	/* Assume the chip is successful, ready and writeable. */

	status = and_mask & ~or_mask;

	/* Choose the loop bounds. */

	if (we_are_interleaving) {
		start = 0;
		end   = physical->chip_count;
	} else {
		start = chip;
		end   = start + 1;
	}

	/* Loop over physical chips. */

	add_event("Looping over physical chips...", 0);

	for (i = start; i < end; i++) {

		/* Select the current chip. */

		this->nfc->select_chip(this, i);

		/* Get the current chip's status. */

		add_event("Sending the command...", 0);

		nfc_util_send_cmd(this, NAND_CMD_STATUS);
		nfc_util_wait_for_the_nfc(this, false);

		add_event("Reading the status...", 0);

		x = this->nfc->read_cycle(this);

		/* Fold this chip's status into the combined status. */

		status |= (x & or_mask);
		status &= (x & and_mask) | or_mask;

	}

	add_event("Exiting mal_get_status", -1);

	return status;

}

/**
 * mal_read_a_page() - Abstracted page read.
 *
 * This function returns the ECC status for the entire read operation. A
 * positive return value indicates the number of errors that were corrected
 * (symbol errors for Reed-Solomon hardware engines, bit errors for BCH hardware
 * engines). A negative return value indicates that the ECC engine failed to
 * correct all errors and the data is corrupted. A zero return value indicates
 * there were no errors at all.
 *
 * @this:    Per-device data.
 * @use_ecc: Indicates if we're to use ECC.
 * @chip:    The logical chip of interest.
 * @page:    The logical page number to read.
 * @data:    A pointer to the destination data buffer. If this pointer is null,
 *           that indicates the caller doesn't want the data.
 * @oob:     A pointer to the destination OOB buffer. If this pointer is null,
 *           that indicates the caller doesn't want the OOB.
 */
static int mal_read_a_page(struct imx_nfc_data *this, int use_ecc,
		unsigned chip, unsigned page, uint8_t *data, uint8_t *oob)
{
	int                       we_are_interleaving;
	int                       use_automatic_op;
	unsigned int              start;
	unsigned int              end;
	unsigned int              current_chip;
	unsigned int              oob_bytes_to_copy;
	unsigned int              data_bytes_to_copy;
	int                       status;
	unsigned int              worst_case_ecc_status;
	int                       return_value = 0;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *nfc      = this->nfc_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	add_event("Entering mal_read_a_page", 1);

	/* Establish some important facts. */

	we_are_interleaving = logical->chip_count != physical->chip_count;
	use_automatic_op    = !!this->nfc->start_auto_read;

	/* Apply the automatic operation override, if any. */

	switch (this->auto_op_override) {

	case NEVER:
		use_automatic_op = false;
		break;

	case DRIVER_CHOICE:
		break;

	case ALWAYS:
		if (this->nfc->start_auto_read)
			use_automatic_op = true;
		break;

	}

	/* Set up ECC. */

	this->nfc->set_ecc(this, use_ecc);

	/* Check if we're interleaving and set up the loop iterations. */

	if (we_are_interleaving) {

		start = 0;
		end   = physical->chip_count;

		data_bytes_to_copy =
			this->logical_geometry.page_data_size /
					this->physical_geometry.chip_count;
		oob_bytes_to_copy =
			this->logical_geometry.page_oob_size /
					this->physical_geometry.chip_count;

	} else {

		start = chip;
		end   = start + 1;

		data_bytes_to_copy = this->logical_geometry.page_data_size;
		oob_bytes_to_copy  = this->logical_geometry.page_oob_size;

	}

	/* If we're using the automatic operation, start it now. */

	if (use_automatic_op) {
		add_event("Starting the automatic operation...", 0);
		this->nfc->start_auto_read(this, start, end - start, 0, page);
	}

	/* Loop over physical chips. */

	add_event("Looping over physical chips...", 0);

	for (current_chip = start; current_chip < end; current_chip++) {

		/* Check if we're using the automatic operation. */

		if (use_automatic_op) {

			add_event("Waiting...", 0);
			this->nfc->wait_for_auto_read(this);

		} else {

			/* Select the current chip. */

			this->nfc->select_chip(this, current_chip);

			/* Set up the chip. */

			add_event("Sending the command and addresses...", 0);

			nfc_util_send_cmd_and_addrs(this,
						NAND_CMD_READ0, 0, page);

			if (is_large_page_chip(this)) {
				add_event("Sending the final command...", 0);
				nfc_util_send_cmd(this, NAND_CMD_READSTART);
			}

			/* Wait till the page is ready. */

			add_event("Waiting for the page to arrive...", 0);

			nfc_util_wait_for_the_nfc(this, true);

			/* Read the page. */

			add_event("Reading the page...", 0);

			this->nfc->read_page(this);

		}

		/* Copy a page out of the NFC. */

		add_event("Copying from the NFC...", 0);

		if (oob) {
			nfc_util_copy_from_the_nfc(this,
				nfc->page_data_size, oob, oob_bytes_to_copy);
			oob += oob_bytes_to_copy;
		}

		if (data) {
			nfc_util_copy_from_the_nfc(this,
						0, data, data_bytes_to_copy);
			data += data_bytes_to_copy;
		}

		/*
		 * If we're using ECC, and we haven't already seen an ECC
		 * failure, continue to gather ECC status. Note that, if we
		 * *do* see an ECC failure, we continue to read because the
		 * client might want the data for forensic purposes.
		 */

		if (use_ecc && (return_value >= 0)) {

			add_event("Getting ECC status...", 0);

			status = this->nfc->get_ecc_status(this);

			if (status >= 0)
				return_value += status;
			else
				return_value = -1;

		}

		/* Check if we're using the automatic operation. */

		if (use_automatic_op) {

			/*
			 * If this is not the last iteration, resume the
			 * automatic operation.
			 */

			if (current_chip < (end - 1)) {
				add_event("Resuming...", 0);
				this->nfc->resume_auto_read(this);
			}

		}

	}

	/* Check if we're supposed to inject an ECC error. */

	if (use_ecc && this->inject_ecc_error) {

		/* Inject the appropriate error. */

		if (this->inject_ecc_error < 0) {

			add_event("Injecting an uncorrectable error...", 0);

			return_value = -1;

		} else {

			add_event("Injecting correctable errors...", 0);

			worst_case_ecc_status =
				physical->chip_count *
				nfc->buffer_count *
				nfc->ecc_strength;

			if (this->inject_ecc_error > worst_case_ecc_status)
				return_value = worst_case_ecc_status;
			else
				return_value = this->inject_ecc_error;

		}

		/* Stop injecting further errors. */

		this->inject_ecc_error = 0;

	}

	/* Return. */

	add_event("Exiting mal_read_a_page", -1);

	return return_value;

}

/**
 * mal_write_a_page() - Abstracted page write.
 *
 * This function returns zero if the operation succeeded, or -EIO if the
 * operation failed.
 *
 * @this:    Per-device data.
 * @use_ecc: Indicates if we're to use ECC.
 * @chip:    The logical chip of interest.
 * @page:    The logical page number to write.
 * @data:    A pointer to the source data buffer.
 * @oob:     A pointer to the source OOB buffer.
 */
static int mal_write_a_page(struct imx_nfc_data *this, int use_ecc,
	unsigned chip, unsigned page, const uint8_t *data, const uint8_t *oob)
{
	int                       we_are_interleaving;
	int                       use_automatic_op;
	unsigned int              start;
	unsigned int              end;
	unsigned int              current_chip;
	unsigned int              oob_bytes_to_copy;
	unsigned int              data_bytes_to_copy;
	int                       return_value = 0;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct nfc_geometry       *nfc      = this->nfc_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	add_event("Entering mal_write_a_page", 1);

	/* Establish some important facts. */

	we_are_interleaving = logical->chip_count != physical->chip_count;
	use_automatic_op    = !!this->nfc->start_auto_write;

	/* Apply the automatic operation override, if any. */

	switch (this->auto_op_override) {

	case NEVER:
		use_automatic_op = false;
		break;

	case DRIVER_CHOICE:
		break;

	case ALWAYS:
		if (this->nfc->start_auto_write)
			use_automatic_op = true;
		break;

	}

	/* Set up ECC. */

	this->nfc->set_ecc(this, use_ecc);

	/* Check if we're interleaving and set up the loop iterations. */

	if (we_are_interleaving) {

		start = 0;
		end   = physical->chip_count;

		data_bytes_to_copy =
			this->logical_geometry.page_data_size /
					this->physical_geometry.chip_count;
		oob_bytes_to_copy =
			this->logical_geometry.page_oob_size /
					this->physical_geometry.chip_count;

	} else {

		start = chip;
		end   = start + 1;

		data_bytes_to_copy = this->logical_geometry.page_data_size;
		oob_bytes_to_copy  = this->logical_geometry.page_oob_size;

	}

	/* If we're using the automatic operation, start the hardware now. */

	if (use_automatic_op) {
		add_event("Starting the automatic operation...", 0);
		this->nfc->start_auto_write(this, start, end - start, 0, page);
	}

	/* Loop over physical chips. */

	add_event("Looping over physical chips...", 0);

	for (current_chip = start; current_chip < end; current_chip++) {

		/* Copy a page into the NFC. */

		add_event("Copying to the NFC...", 0);

		nfc_util_copy_to_the_nfc(this, oob, nfc->page_data_size,
							oob_bytes_to_copy);
		oob += oob_bytes_to_copy;

		nfc_util_copy_to_the_nfc(this, data, 0, data_bytes_to_copy);

		data += data_bytes_to_copy;

		/* Check if we're using the automatic operation. */

		if (use_automatic_op) {

			/* Wait for the write operation to finish. */

			add_event("Waiting...", 0);

			this->nfc->wait_for_auto_write(this);

		} else {

			/* Select the current chip. */

			this->nfc->select_chip(this, current_chip);

			/* Set up the chip. */

			add_event("Sending the command and addresses...", 0);

			nfc_util_send_cmd_and_addrs(this,
						NAND_CMD_SEQIN, 0, page);

			/* Send the page. */

			add_event("Sending the page...", 0);

			this->nfc->send_page(this);

			/* Start programming the page. */

			add_event("Programming the page...", 0);

			nfc_util_send_cmd(this, NAND_CMD_PAGEPROG);

			/* Wait until the page is finished. */

			add_event("Waiting...", 0);

			nfc_util_wait_for_the_nfc(this, true);

		}

	}

	/* Get status. */

	add_event("Gathering status...", 0);

	if (mal_get_status(this, chip) & NAND_STATUS_FAIL) {
		add_event("Bad status", 0);
		return_value = -EIO;
	} else {
		add_event("Good status", 0);
	}

	/* Return. */

	add_event("Exiting mal_write_a_page", -1);

	return return_value;

}

/**
 * mal_erase_a_block() - Abstract block erase operation.
 *
 * Note that this function does *not* wait for the operation to finish. The
 * caller is expected to call waitfunc() at some later time.
 *
 * @this:  Per-device data.
 * @chip:  The logical chip of interest.
 * @page:  A logical page address that identifies the block to erase.
 */
static void mal_erase_a_block(struct imx_nfc_data *this,
						unsigned chip, unsigned page)
{
	int                       we_are_interleaving;
	int                       use_automatic_op;
	unsigned int              start;
	unsigned int              end;
	unsigned int              i;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	add_event("Entering mal_erase_a_block", 1);

	/* Establish some important facts. */

	we_are_interleaving = logical->chip_count != physical->chip_count;
	use_automatic_op    = !!this->nfc->start_auto_erase;

	/* Apply the automatic operation override, if any. */

	switch (this->auto_op_override) {

	case NEVER:
		use_automatic_op = false;
		break;

	case DRIVER_CHOICE:
		break;

	case ALWAYS:
		if (this->nfc->start_auto_erase)
			use_automatic_op = true;
		break;

	}

	/* Choose the loop bounds. */

	if (we_are_interleaving) {
		start = 0;
		end   = physical->chip_count;
	} else {
		start = chip;
		end   = start + 1;
	}

	/* Check if we're using the automatic operation. */

	if (use_automatic_op) {

		/*
		 * Start the operation. Note that we don't wait for it to
		 * finish because the HIL will call our waitfunc().
		 */

		add_event("Starting the automatic operation...", 0);

		this->nfc->start_auto_erase(this, start, end - start, page);

	} else {

		/* Loop over physical chips. */

		add_event("Looping over physical chips...", 0);

		for (i = start; i < end; i++) {

			/* Select the current chip. */

			this->nfc->select_chip(this, i);

			/* Set up the chip. */

			nfc_util_send_cmd_and_addrs(this,
						NAND_CMD_ERASE1, -1, page);

			/* Start the erase. */

			nfc_util_send_cmd(this, NAND_CMD_ERASE2);

			/*
			 * If this is the last time through the loop, break out
			 * now so we don't try to wait (the HIL will call our
			 * waitfunc() for the final wait).
			 */

			if (i >= (end - 1))
				break;

			/* Wait for the erase on the current chip to finish. */

			nfc_util_wait_for_the_nfc(this, true);

		}

	}

	add_event("Exiting mal_erase_a_block", -1);

}

/**
 * mal_is_block_bad() - Abstract bad block check.
 *
 * @this:    Per-device data.
 * @chip:    The logical chip of interest.
 * @page:    The logical page number to read.
 */
 #if 0

/* TODO: Finish this function and plug it in. */

static int mal_is_block_bad(struct imx_nfc_data *this,
						unsigned chip, unsigned page)
{
	int                       we_are_interleaving;
	unsigned int              start;
	unsigned int              end;
	unsigned int              i;
	uint8_t                   *p;
	int                       return_value = 0;
	struct nand_chip          *nand = &this->nand;
	struct physical_geometry  *physical = &this->physical_geometry;
	struct logical_geometry   *logical  = &this->logical_geometry;

	/* Figure out if we're interleaving. */

	we_are_interleaving = logical->chip_count != physical->chip_count;

	/*
	 * We're about to use the NAND Flash MTD layer's buffer, so invalidate
	 * the page cache.
	 */

	this->nand.pagebuf = -1;

	/*
	 * Read the OOB of the given page, using the NAND Flash MTD's buffer.
	 *
	 * Notice that ECC is off, which it *must* be when scanning block marks.
	 */

	mal_read_a_page(this, false,
		this->current_chip, this->page_address, 0, nand->oob_poi);

	/* Choose the loop bounds. */

	if (we_are_interleaving) {
		start = 0;
		end   = physical->chip_count;
	} else {
		start = chip;
		end   = start + 1;
	}

	/* Start scanning at the beginning of the OOB data. */

	p = nand->oob_poi;

	/* Loop over physical chips. */

	add_event("Looping over physical chips...", 0);

	for (i = start; i < end; i++, p += 5) {

		/* Examine the OOB for this chip. */

		if (p[nand->badblockpos] != 0xff) {
			return_value = !0;
			break;
		}

	}

	/* Return. */

	return return_value;

}
#endif

/**
 * mil_init() - Initializes the MTD Interface Layer.
 *
 * @this:    Per-device data.
 */
static void mil_init(struct imx_nfc_data *this)
{
	this->current_chip   = -1;    /* No chip is selected yet. */
	this->command_is_new = false; /* No command yet.          */
}

/**
 * mil_cmd_ctrl() - MTD Interface cmd_ctrl()
 *
 * @mtd:  A pointer to the owning MTD.
 * @dat:  The data signals to present to the chip.
 * @ctrl: The control signals to present to the chip.
 */
static void mil_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_dev_ready() - MTD Interface dev_ready()
 *
 * @mtd:   A pointer to the owning MTD.
 */
static int mil_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc dev_ready]\n");

	add_event("Entering mil_dev_ready", 1);

	if (this->nfc->is_ready(this)) {
		add_event("Exiting mil_dev_ready - Returning ready", -1);
		return !0;
	} else {
		add_event("Exiting mil_dev_ready - Returning busy", -1);
		return 0;
	}

}

/**
 * mil_select_chip() - MTD Interface select_chip()
 *
 * @mtd:   A pointer to the owning MTD.
 * @chip:  The chip number to select, or -1 to select no chip.
 */
static void mil_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc select_chip] chip: %d\n", chip);

	/* Figure out what kind of transition this is. */

	if ((this->current_chip < 0) && (chip >= 0)) {
		start_event_trace("Entering mil_select_chip");
		if (this->pdata->force_ce)
			this->nfc->set_force_ce(this, true);
		clk_enable(this->clock);
		add_event("Exiting mil_select_chip", -1);
	} else if ((this->current_chip >= 0) && (chip < 0)) {
		add_event("Entering mil_select_chip", 1);
		if (this->pdata->force_ce)
			this->nfc->set_force_ce(this, false);
		clk_disable(this->clock);
		stop_event_trace("Exiting mil_select_chip");
	} else {
		add_event("Entering mil_select_chip", 1);
		add_event("Exiting mil_select_chip", -1);
	}

	this->current_chip = chip;

}

/**
 * mil_cmdfunc() - MTD Interface cmdfunc()
 *
 * This function handles NAND Flash command codes from the HIL. Since only the
 * HIL calls this function (none of the reference implementations we use do), it
 * needs to handle very few command codes.
 *
 * @mtd:      A pointer to the owning MTD.
 * @command:  The command code.
 * @column:   The column address associated with this command code, or -1 if no
 *            column address applies.
 * @page:     The page address associated with this command code, or -1 if no
 *            page address applies.
 */
static void mil_cmdfunc(struct mtd_info *mtd,
					unsigned command, int column, int page)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc cmdfunc] command: 0x%02x, "
		"column: 0x%04x, page: 0x%06x\n", command, column, page);

	add_event("Entering mil_cmdfunc", 1);

	/* Record the command and the fact that it hasn't yet been sent. */

	this->command        = command;
	this->command_is_new = true;

	/*
	 * Process the command code.
	 *
	 * Note the default case to trap unrecognized codes. Thus, every command
	 * we support must have a case here, even if we don't have to do any
	 * pre-processing work. If the HIL changes and starts sending commands
	 * we haven't explicitly implemented, this will warn us.
	 */

	switch (command) {

	case NAND_CMD_READ0:
		add_event("NAND_CMD_READ0", 0);
		/*
		 * After calling this function to send the command and
		 * addresses, the HIL will call ecc.read_page() or
		 * ecc.read_page_raw() to collect the data.
		 *
		 * The column address from the HIL is always zero. The only
		 * information we need to keep from this call is the page
		 * address.
		 */
		this->page_address = page;
		break;

	case NAND_CMD_STATUS:
		add_event("NAND_CMD_STATUS", 0);
		/*
		 * After calling this function to send the command, the HIL
		 * will call read_byte() once to collect the status.
		 */
		break;

	case NAND_CMD_READID:
		add_event("NAND_CMD_READID", 0);
		/*
		 * After calling this function to send the command, the HIL
		 * will call read_byte() repeatedly to collect ID bytes.
		 */
		break;

	case NAND_CMD_RESET:
		add_event("NAND_CMD_RESET", 0);
		mal_reset(this, this->current_chip);
		break;

	default:
		dev_emerg(this->dev, "Unsupported NAND Flash command code: "
							"0x%02x\n", command);
		BUG();
		break;

	}

	add_event("Exiting mil_cmdfunc", -1);

}

/**
 * mil_waitfunc() - MTD Interface waifunc()
 *
 * This function blocks until the current chip is ready and then returns the
 * contents of the chip's status register. The HIL only calls this function
 * after starting an erase operation.
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 */
static int mil_waitfunc(struct mtd_info *mtd, struct nand_chip *nand)
{
	int                  status;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc waitfunc]\n");

	add_event("Entering mil_waitfunc", 1);

	/* Wait for the NFC to finish. */

	nfc_util_wait_for_the_nfc(this, true);

	/* Get the status. */

	status = mal_get_status(this, this->current_chip);

	add_event("Exiting mil_waitfunc", -1);

	return status;

}

/**
 * mil_read_byte() - MTD Interface read_byte().
 *
 * @mtd:  A pointer to the owning MTD.
 */
static uint8_t mil_read_byte(struct mtd_info *mtd)
{
	uint8_t              byte = 0;
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	add_event("Entering mil_read_byte", 1);

	/*
	 * The command sent by the HIL before it called this function determines
	 * how we get the byte we're going to return.
	 */

	switch (this->command) {

	case NAND_CMD_STATUS:
		add_event("NAND_CMD_STATUS", 0);
		byte = mal_get_status(this, this->current_chip);
		break;

	case NAND_CMD_READID:
		add_event("NAND_CMD_READID", 0);

		/*
		 * Check if the command is new. If so, then the HIL just
		 * recently called cmdfunc(), so the current chip isn't selected
		 * and the command hasn't been sent to the chip.
		 */

		if (this->command_is_new) {
			add_event("Sending the \"Read ID\" command...", 0);
			this->nfc->select_chip(this, this->current_chip);
			nfc_util_send_cmd_and_addrs(this,
							NAND_CMD_READID, 0, -1);
			this->command_is_new = false;
		}

		/* Read the ID byte. */

		add_event("Reading the ID byte...", 0);

		byte = this->nfc->read_cycle(this);

		break;

	default:
		dev_emerg(this->dev, "Unsupported NAND Flash command code: "
						"0x%02x\n", this->command);
		BUG();
		break;

	}

	DEBUG(MTD_DEBUG_LEVEL2,
			"[imx_nfc read_byte] Returning: 0x%02x\n", byte);

	add_event("Exiting mil_read_byte", -1);

	return byte;

}

/**
 * mil_read_word() - MTD Interface read_word().
 *
 * @mtd:  A pointer to the owning MTD.
 */
static uint16_t mil_read_word(struct mtd_info *mtd)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
	return 0;
}

/**
 * mil_read_buf() - MTD Interface read_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The destination buffer.
 * @len:  The number of bytes to read.
 */
static void mil_read_buf(struct mtd_info *mtd, uint8_t * buf, int len)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_write_buf() - MTD Interface write_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The source buffer.
 * @len:  The number of bytes to read.
 */
static void mil_write_buf(struct mtd_info *mtd, const uint8_t * buf, int len)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_verify_buf() - MTD Interface verify_buf().
 *
 * @mtd:  A pointer to the owning MTD.
 * @buf:  The destination buffer.
 * @len:  The number of bytes to read.
 */
static int mil_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
	return 0;
}

/**
 * mil_ecc_hwctl() - MTD Interface ecc.hwctl().
 *
 * @mtd:   A pointer to the owning MTD.
 * @mode:  The ECC mode.
 */
static void mil_ecc_hwctl(struct mtd_info *mtd, int mode)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_ecc_calculate() - MTD Interface ecc.calculate().
 *
 * @mtd:       A pointer to the owning MTD.
 * @dat:       A pointer to the source data.
 * @ecc_code:  A pointer to a buffer that will receive the resulting ECC.
 */
static int mil_ecc_calculate(struct mtd_info *mtd,
					const uint8_t *dat, uint8_t *ecc_code)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
	return 0;
}

/**
 * mil_ecc_correct() - MTD Interface ecc.correct().
 *
 * @mtd:       A pointer to the owning MTD.
 * @dat:       A pointer to the source data.
 * @read_ecc:  A pointer to the ECC that was read from the medium.
 * @calc_ecc:  A pointer to the ECC that was calculated for the source data.
 */
static int mil_ecc_correct(struct mtd_info *mtd,
			uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
	return 0;
}

/**
 * mil_ecc_read_page() - MTD Interface ecc.read_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the destination buffer.
 */
static int mil_ecc_read_page(struct mtd_info *mtd,
					struct nand_chip *nand, uint8_t *buf)
{
	int                  ecc_status;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc ecc_read_page]\n");

	add_event("Entering mil_ecc_read_page", 1);

	/* Read the page. */

	ecc_status =
		mal_read_a_page(this, true, this->current_chip,
					this->page_address, buf, nand->oob_poi);

	/* Propagate ECC information. */

	if (ecc_status < 0) {
		add_event("ECC Failure", 0);
		mtd->ecc_stats.failed++;
	} else if (ecc_status > 0) {
		add_event("ECC Corrections", 0);
		mtd->ecc_stats.corrected += ecc_status;
	}

	add_event("Exiting mil_ecc_read_page", -1);

	return 0;

}

/**
 * mil_ecc_read_page_raw() - MTD Interface ecc.read_page_raw().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the destination buffer.
 */
static int mil_ecc_read_page_raw(struct mtd_info *mtd,
					struct nand_chip *nand, uint8_t *buf)
{
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc ecc_read_page_raw]\n");

	add_event("Entering mil_ecc_read_page_raw", 1);

	mal_read_a_page(this, false, this->current_chip,
					this->page_address, buf, nand->oob_poi);

	add_event("Exiting mil_ecc_read_page_raw", -1);

	return 0;

}

/**
 * mil_ecc_write_page() - MTD Interface ecc.write_page().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the source buffer.
 */
static void mil_ecc_write_page(struct mtd_info *mtd,
				struct nand_chip *nand, const uint8_t *buf)
{
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_ecc_write_page_raw() - MTD Interface ecc.write_page_raw().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @buf:   A pointer to the source buffer.
 */
static void mil_ecc_write_page_raw(struct mtd_info *mtd,
				struct nand_chip *nand, const uint8_t *buf)
{
	struct imx_nfc_data  *this = nand->priv;
	unimplemented(this, __func__);
}

/**
 * mil_write_page() - MTD Interface ecc.write_page().
 *
 * @mtd:     A pointer to the owning MTD.
 * @nand:    A pointer to the owning NAND Flash MTD.
 * @buf:     A pointer to the source buffer.
 * @page:    The page number to write.
 * @cached:  Indicates cached programming (ignored).
 * @raw:     Indicates not to use ECC.
 */
static int mil_write_page(struct mtd_info *mtd,
		struct nand_chip *nand, const uint8_t *buf,
		int page, int cached, int raw)
{
	int                  return_value;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc write_page]\n");

	add_event("Entering mil_write_page", 1);

	return_value = mal_write_a_page(this, !raw,
				this->current_chip, page, buf, nand->oob_poi);

	add_event("Exiting mil_write_page", -1);

	return return_value;

}

/**
 * mil_ecc_read_oob() - MTD Interface read_oob().
 *
 * @mtd:     A pointer to the owning MTD.
 * @nand:    A pointer to the owning NAND Flash MTD.
 * @page:    The page number to read.
 * @sndcmd:  Indicates this function should send a command to the chip before
 *           reading the out-of-band bytes. This is only false for small page
 *           chips that support auto-increment.
 */
static int mil_ecc_read_oob(struct mtd_info *mtd, struct nand_chip *nand,
							int page, int sndcmd)
{
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc ecc_read_oob] "
		"page: 0x%06x, sndcmd: %s\n", page, sndcmd ? "Yes" : "No");

	add_event("Entering mil_ecc_read_oob", 1);

	mal_read_a_page(this, false,
				this->current_chip, page, 0, nand->oob_poi);

	add_event("Exiting mil_ecc_read_oob", -1);

	/*
	 * Return true, indicating that the next call to this function must send
	 * a command.
	 */

	return true;

}

/**
 * mil_ecc_write_oob() - MTD Interface write_oob().
 *
 * @mtd:   A pointer to the owning MTD.
 * @nand:  A pointer to the owning NAND Flash MTD.
 * @page:  The page number to write.
 */
static int mil_ecc_write_oob(struct mtd_info *mtd,
					struct nand_chip *nand, int page)
{
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc ecc_write_oob] page: 0x%06x\n", page);

	/*
	 * There are fundamental incompatibilities between the i.MX NFC and the
	 * NAND Flash MTD model that make it essentially impossible to write the
	 * out-of-band bytes.
	 */

	dev_emerg(this->dev, "This driver doesn't support writing the OOB\n");
	WARN_ON(1);

	/* Return status. */

	return -EIO;

}

/**
 * mil_erase_cmd() - MTD Interface erase_cmd().
 *
 * We set the erase_cmd pointer in struct nand_chip to point to this function.
 * Thus, the HIL will call here for all erase operations.
 *
 * Strictly speaking, since the erase_cmd pointer is marked "Internal," we
 * shouldn't be doing this. However, the only reason the HIL uses that pointer
 * is to install a different function for erasing conventional NAND Flash or AND
 * Flash. Since AND Flash is obsolete and we don't support it, this isn't
 * important.
 *
 * Furthermore, to cleanly implement interleaving (which is critical to speeding
 * up erase operations), we want to "hook into" the operation at the highest
 * semantic level possible. If we don't hook this function, then the only way
 * we'll know that an erase is happening is when the HIL calls cmdfunc() with
 * an erase command. Implementing interleaving at that level is roughly a
 * billion times less desirable.
 *
 * This function does *not* wait for the operation to finish. The HIL will call
 * waitfunc() later to wait for the operation to finish.
 *
 * @mtd:   A pointer to the owning MTD.
 * @page:  A page address that identifies the block to erase.
 */
static void mil_erase_cmd(struct mtd_info *mtd, int page)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc erase_cmd] page: 0x%06x\n", page);

	add_event("Entering mil_erase_cmd", 1);

	mal_erase_a_block(this, this->current_chip, page);

	add_event("Exiting mil_erase_cmd", -1);

}

/**
 * mil_block_bad() - MTD Interface block_bad().
 *
 * @mtd:      A pointer to the owning MTD.
 * @ofs:      The offset of the block of interest, from the start of the medium.
 * @getchip:  Indicates this function must acquire the MTD before using it.
 */
#if 0

/* TODO: Finish this function and plug it in. */

static int mil_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	unsigned int         chip;
	unsigned int         page;
	int                  return_value;
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc block_bad] page: 0x%06x\n", page);

	add_event("Entering mil_block_bad", 1);

	/* Compute the logical chip number that contains the given offset. */

	chip = (unsigned int) (ofs >> nand->chip_shift);

	/* Compute the logical page address within the logical chip. */

	page = ((unsigned int) (ofs >> nand->page_shift)) & nand->pagemask;

	/* Check if the block is bad. */

	return_value = mal_is_block_bad(this, chip, page);

	if (return_value)
		add_event("Bad block", 0);

	/* Return. */

	add_event("Exiting mil_block_bad", -1);

	return return_value;

}
#endif

/**
 * mil_scan_bbt() - MTD Interface scan_bbt().
 *
 * The HIL calls this function once, when it initializes the NAND Flash MTD.
 *
 * Nominally, the purpose of this function is to look for or create the bad
 * block table. In fact, since the HIL calls this function at the very end of
 * the initialization process started by nand_scan(), and the HIL doesn't have a
 * more formal mechanism, everyone "hooks" this function to continue the
 * initialization process.
 *
 * At this point, the physical NAND Flash chips have been identified and
 * counted, so we know the physical geometry. This enables us to make some
 * important configuration decisions.
 *
 * The return value of this function propogates directly back to this driver's
 * call to nand_scan(). Anything other than zero will cause this driver to
 * tear everything down and declare failure.
 *
 * @mtd:  A pointer to the owning MTD.
 */
static int mil_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip     *nand = mtd->priv;
	struct imx_nfc_data  *this = nand->priv;

	DEBUG(MTD_DEBUG_LEVEL2, "[imx_nfc scan_bbt] \n");

	add_event("Entering mil_scan_bbt", 1);

	/*
	 * We replace the erase_cmd() function that the MTD NAND Flash system
	 * has installed with our own. See mil_erase_cmd() for the reasons.
	 */

	nand->erase_cmd = mil_erase_cmd;

	/*
	 * Tell MTD users that the out-of-band area can't be written.
	 *
	 * This flag is not part of the standard kernel source tree. It comes
	 * from a patch that touches both MTD and JFFS2.
	 *
	 * The problem is that, without this patch, JFFS2 believes it can write
	 * the data area and the out-of-band area separately. This is wrong for
	 * two reasons:
	 *
	 *     1)  Our NFC distributes out-of-band bytes throughout the page,
	 *         intermingled with the data, and covered by the same ECC.
	 *         Thus, it's not possible to write the out-of-band bytes and
	 *         data bytes separately.
	 *
	 *     2)  Large page (MLC) Flash chips don't support partial page
	 *         writes. You must write the entire page at a time. Thus, even
	 *         if our NFC didn't force you to write out-of-band and data
	 *         bytes together, it would *still* be a bad idea to do
	 *         otherwise.
	 */

	mtd->flags &= ~MTD_OOB_WRITEABLE;

	/* Set up geometry. */

	mal_set_geometry(this);

	/* We use the reference implementation for bad block management. */

	add_event("Exiting mil_scan_bbt", -1);

	return nand_scan_bbt(mtd, nand->badblock_pattern);

}

/**
 * parse_bool_param() - Parses the value of a boolean parameter string.
 *
 * @s: The string to parse.
 */
static int parse_bool_param(const char *s)
{

	if (!strcmp(s, "1") || !strcmp(s, "on") ||
				!strcmp(s, "yes") ||	!strcmp(s, "true")) {
		return 1;
	} else if (!strcmp(s, "0") || !strcmp(s, "off") ||
				!strcmp(s, "no") || !strcmp(s, "false")) {
		return 0;
	} else {
		return -1;
	}

}

/**
 * set_module_enable() - Controls whether this driver is enabled.
 *
 * Note that this state can be controlled from the command line. Disabling this
 * driver is sometimes useful for debugging.
 *
 * @s:   The new value of the parameter.
 * @kp:  The owning kernel parameter.
 */
static int set_module_enable(const char *s, struct kernel_param *kp)
{

	switch (parse_bool_param(s)) {

	case 1:
		imx_nfc_module_enable = true;
		break;

	case 0:
		imx_nfc_module_enable = false;
		break;

	default:
		return -EINVAL;
		break;

	}

	return 0;

}

/**
 * get_module_enable() - Indicates whether this driver is enabled.
 *
 * @p:   A pointer to a (small) buffer that will receive the response.
 * @kp:  The owning kernel parameter.
 */
static int get_module_enable(char *p, struct kernel_param *kp)
{
	p[0] = imx_nfc_module_enable ? '1' : '0';
	p[1] = 0;
	return 1;
}

#ifdef EVENT_REPORTING

/**
 * set_module_report_events() - Controls whether this driver reports events.
 *
 * @s:   The new value of the parameter.
 * @kp:  The owning kernel parameter.
 */
static int set_module_report_events(const char *s, struct kernel_param *kp)
{

	switch (parse_bool_param(s)) {

	case 1:
		imx_nfc_module_report_events = true;
		break;

	case 0:
		imx_nfc_module_report_events = false;
		reset_event_trace();
		break;

	default:
		return -EINVAL;
		break;

	}

	return 0;

}

/**
 * get_module_report_events() - Indicates whether the driver reports events.
 *
 * @p:   A pointer to a (small) buffer that will receive the response.
 * @kp:  The owning kernel parameter.
 */
static int get_module_report_events(char *p, struct kernel_param *kp)
{
	p[0] = imx_nfc_module_report_events ? '1' : '0';
	p[1] = 0;
	return 1;
}

/**
 * set_module_dump_events() - Forces the driver to dump current events.
 *
 * @s:   The new value of the parameter.
 * @kp:  The owning kernel parameter.
 */
static int set_module_dump_events(const char *s, struct kernel_param *kp)
{
	dump_event_trace();
	return 0;
}

#endif /*EVENT_REPORTING*/

/**
 * set_module_interleave_override() - Controls the interleave override.
 *
 * @s:   The new value of the parameter.
 * @kp:  The owning kernel parameter.
 */
static int set_module_interleave_override(const char *s,
							struct kernel_param *kp)
{

	if      (!strcmp(s, "-1"))
		imx_nfc_module_interleave_override = NEVER;
	else if (!strcmp(s, "0"))
		imx_nfc_module_interleave_override = DRIVER_CHOICE;
	else if (!strcmp(s, "1"))
		imx_nfc_module_interleave_override = ALWAYS;
	else
		return -EINVAL;

	return 0;

}

/**
 * get_module_interleave_override() - Indicates the interleave override state.
 *
 * @p:   A pointer to a (small) buffer that will receive the response.
 * @kp:  The owning kernel parameter.
 */
static int get_module_interleave_override(char *p, struct kernel_param *kp)
{
	return sprintf(p, "%d", imx_nfc_module_interleave_override);
}

/**
 * set_force_bytewise_copy() - Controls forced bytewise copy from/to the NFC.
 *
 * @s:   The new value of the parameter.
 * @kp:  The owning kernel parameter.
 */
static int set_module_force_bytewise_copy(const char *s,
							struct kernel_param *kp)
{

	switch (parse_bool_param(s)) {

	case 1:
		imx_nfc_module_force_bytewise_copy = true;
		break;

	case 0:
		imx_nfc_module_force_bytewise_copy = false;
		break;

	default:
		return -EINVAL;
		break;

	}

	return 0;

}

/**
 * get_force_bytewise_copy() - Indicates whether bytewise copy is being forced.
 *
 * @p:   A pointer to a (small) buffer that will receive the response.
 * @kp:  The owning kernel parameter.
 */
static int get_module_force_bytewise_copy(char *p, struct kernel_param *kp)
{
	p[0] = imx_nfc_module_force_bytewise_copy ? '1' : '0';
	p[1] = 0;
	return 1;
}

/* Module attributes that appear in sysfs. */

module_param_call(enable, set_module_enable, get_module_enable, 0, 0444);
MODULE_PARM_DESC(enable, "enables/disables probing");

#ifdef EVENT_REPORTING
module_param_call(report_events,
		set_module_report_events, get_module_report_events, 0, 0644);
MODULE_PARM_DESC(report_events, "enables/disables event reporting");

module_param_call(dump_events, set_module_dump_events, 0, 0, 0644);
MODULE_PARM_DESC(dump_events, "forces current event dump");
#endif

module_param_call(interleave_override, set_module_interleave_override,
				get_module_interleave_override, 0, 0444);
MODULE_PARM_DESC(interleave_override, "overrides interleaving choice");

module_param_call(force_bytewise_copy, set_module_force_bytewise_copy,
				get_module_force_bytewise_copy, 0, 0644);
MODULE_PARM_DESC(force_bytewise_copy, "forces bytewise copy from/to NFC");

/**
 * show_device_platform_info() - Shows the device's platform information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_platform_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                           o = 0;
	unsigned int                  i;
	void                          *buffer_base;
	void                          *primary_base;
	void                          *secondary_base;
	unsigned int                  interrupt_number;
	struct resource               *r;
	struct imx_nfc_data           *this  = dev_get_drvdata(dev);
	struct platform_device        *pdev  = this->pdev;
	struct imx_nfc_platform_data  *pdata = this->pdata;
	struct mtd_partition          *partition;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					IMX_NFC_BUFFERS_ADDR_RES_NAME);

	buffer_base = (void *) r->start;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME);

	primary_base = (void *) r->start;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME);

	secondary_base = (void *) r->start;

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					IMX_NFC_INTERRUPT_RES_NAME);

	interrupt_number = r->start;

	o += sprintf(buf,
		"NFC Major Version       : %u\n"
		"NFC Minor Version       : %u\n"
		"Force CE                : %s\n"
		"Target Cycle in ns      : %u\n"
		"Clock Name              : %s\n"
		"Interleave              : %s\n"
		"Buffer Base             : 0x%p\n"
		"Primary Registers Base  : 0x%p\n"
		"Secondary Registers Base: 0x%p\n"
		"Interrupt Number        : %u\n"
		,
		pdata->nfc_major_version,
		pdata->nfc_minor_version,
		pdata->force_ce ? "Yes" : "No",
		pdata->target_cycle_in_ns,
		pdata->clock_name,
		pdata->interleave ? "Yes" : "No",
		buffer_base,
		primary_base,
		secondary_base,
		interrupt_number
		);

	#ifdef CONFIG_MTD_PARTITIONS

		o += sprintf(buf + o,
			"Partition Count         : %u\n"
			,
			pdata->partition_count
		);

		/* Loop over partitions. */

		for (i = 0; i < pdata->partition_count; i++) {

			partition = pdata->partitions + i;

			o += sprintf(buf+o, "  [%d]\n", i);
			o += sprintf(buf+o, "  Name  : %s\n", partition->name);

			switch (partition->offset) {

			case MTDPART_OFS_NXTBLK:
				o += sprintf(buf+o, "  Offset: "
							"MTDPART_OFS_NXTBLK\n");
				break;
			case MTDPART_OFS_APPEND:
				o += sprintf(buf+o, "  Offset: "
							"MTDPART_OFS_APPEND\n");
				break;
			default:
				o += sprintf(buf+o, "  Offset: %u (%u MiB)\n",
					partition->offset,
					partition->offset / (1024 * 1024));
				break;

			}

			if (partition->size == MTDPART_SIZ_FULL) {
				o += sprintf(buf+o, "  Size  : "
							"MTDPART_SIZ_FULL\n");
			} else {
				o += sprintf(buf+o, "  Size  : %u (%u MiB)\n",
					partition->size,
					partition->size / (1024 * 1024));
			}

		}

	#endif

	return o;

}

/**
 * show_device_physical_geometry() - Shows the physical Flash device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_physical_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data       *this     = dev_get_drvdata(dev);
	struct physical_geometry  *physical = &this->physical_geometry;

	return sprintf(buf,
		"Chip Count             : %u\n"
		"Chip Size in Bytes     : %llu\n"
		"Block Size in Bytes    : %u\n"
		"Page Data Size in Bytes: %u\n"
		"Page OOB Size in Bytes : %u\n"
		,
		physical->chip_count,
		physical->chip_size,
		physical->block_size,
		physical->page_data_size,
		physical->page_oob_size
	);

}

/**
 * show_device_nfc_info() - Shows the NFC-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long        parent_clock_rate_in_hz;
	unsigned long        clock_rate_in_hz;
	struct clk           *parent_clock;
	struct imx_nfc_data  *this = dev_get_drvdata(dev);
	struct nfc_hal       *nfc  = this->nfc;

	parent_clock            = clk_get_parent(this->clock);
	parent_clock_rate_in_hz = clk_get_rate(parent_clock);
	clock_rate_in_hz        = clk_get_rate(this->clock);

	return sprintf(buf,
		"Major Version           : %u\n"
		"Minor Version           : %u\n"
		"Max Chip Count          : %u\n"
		"Max Buffer Count        : %u\n"
		"Spare Buffer Stride     : %u\n"
		"Has Secondary Registers : %s\n"
		"Can Be Symmetric        : %s\n"
		"Exposes Ready/Busy      : %s\n"
		"Parent Clock Rate in Hz : %lu\n"
		"Clock Rate in Hz        : %lu\n"
		"Symmetric Clock         : %s\n"
		,
		nfc->major_version,
		nfc->minor_version,
		nfc->max_chip_count,
		nfc->max_buffer_count,
		nfc->spare_buf_stride,
		nfc->has_secondary_regs ? "Yes" : "No",
		nfc->can_be_symmetric ? "Yes" : "No",
		nfc->is_ready ? "Yes" : "No",
		parent_clock_rate_in_hz,
		clock_rate_in_hz,
		this->nfc->get_symmetric(this) ? "Yes" : "No"
	);

}

/**
 * show_device_nfc_geometry() - Shows the NFC view of the device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_nfc_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	return sprintf(buf,
		"Page Data Size in Bytes : %u\n"
		"Page OOB Size in Bytes  : %u\n"
		"ECC Algorithm           : %s\n"
		"ECC Strength            : %u\n"
		"Buffers in Use          : %u\n"
		"Spare Buffer Size in Use: %u\n"
		"Spare Buffer Spillover  : %u\n"
		,
		this->nfc_geometry->page_data_size,
		this->nfc_geometry->page_oob_size,
		this->nfc_geometry->ecc_algorithm,
		this->nfc_geometry->ecc_strength,
		this->nfc_geometry->buffer_count,
		this->nfc_geometry->spare_buf_size,
		this->nfc_geometry->spare_buf_spill
	);

}

/**
 * show_device_logical_geometry() - Shows the logical device geometry.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_logical_geometry(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data      *this    = dev_get_drvdata(dev);
	struct logical_geometry  *logical = &this->logical_geometry;

	return sprintf(buf,
		"Chip Count             : %u\n"
		"Chip Size in Bytes     : %u\n"
		"Usable Size in Bytes   : %u\n"
		"Block Size in Bytes    : %u\n"
		"Page Data Size in Bytes: %u\n"
		"Page OOB Size in Bytes : %u\n"
		,
		logical->chip_count,
		logical->chip_size,
		logical->usable_size,
		logical->block_size,
		logical->page_data_size,
		logical->page_oob_size
	);

}

/**
 * show_device_mtd_nand_info() - Shows the device's MTD NAND-specific info.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_nand_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct imx_nfc_data        *this = dev_get_drvdata(dev);
	struct nand_chip           *nand = &this->nand;

	o += sprintf(buf + o,
		"Options              : 0x%08x\n"
		"Chip Count           : %u\n"
		"Chip Size            : %lu\n"
		"Minimum Writable Size: %u\n"
		"Page Shift           : %u\n"
		"Page Mask            : 0x%x\n"
		"Block Shift          : %u\n"
		"BBT Block Shift      : %u\n"
		"Chip Shift           : %u\n"
		"Block Mark Offset    : %u\n"
		"Cached Page Number   : %d\n"
		,
		nand->options,
		nand->numchips,
		nand->chipsize,
		nand->subpagesize,
		nand->page_shift,
		nand->pagemask,
		nand->phys_erase_shift,
		nand->bbt_erase_shift,
		nand->chip_shift,
		nand->badblockpos,
		nand->pagebuf
	);

	o += sprintf(buf + o,
		"ECC Byte Count       : %u\n"
		,
		nand->ecc.layout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < nand->ecc.layout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= nand->ecc.layout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				nand->ecc.layout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes  : %u\n"
		,
		nand->ecc.layout->oobavail
	);

	j = 0;

	for (i = 0; j < nand->ecc.layout->oobavail; i++) {

		j += nand->ecc.layout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			nand->ecc.layout->oobfree[i].offset,
			nand->ecc.layout->oobfree[i].length
		);

	}

	return o;

}

/**
 * show_device_mtd_info() - Shows the device's MTD-specific information.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_mtd_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                        o = 0;
	unsigned int               i;
	unsigned int               j;
	static const unsigned int  columns = 8;
	struct imx_nfc_data        *this = dev_get_drvdata(dev);
	struct mtd_info            *mtd  = &this->mtd;

	o += sprintf(buf + o,
		"Name               : %s\n"
		"Type               : %u\n"
		"Flags              : 0x%08x\n"
		"Size in Bytes      : %u\n"
		"Erase Region Count : %d\n"
		"Erase Size in Bytes: %u\n"
		"Write Size in Bytes: %u\n"
		"OOB Size in Bytes  : %u\n"
		"Errors Corrected   : %u\n"
		"Failed Reads       : %u\n"
		"Bad Block Count    : %u\n"
		"BBT Block Count    : %u\n"
		,
		mtd->name,
		mtd->type,
		mtd->flags,
		mtd->size,
		mtd->numeraseregions,
		mtd->erasesize,
		mtd->writesize,
		mtd->oobsize,
		mtd->ecc_stats.corrected,
		mtd->ecc_stats.failed,
		mtd->ecc_stats.badblocks,
		mtd->ecc_stats.bbtblocks
	);

	o += sprintf(buf + o,
		"ECC Byte Count     : %u\n"
		,
		mtd->ecclayout->eccbytes
	);

	/* Loop over rows. */

	for (i = 0; (i * columns) < mtd->ecclayout->eccbytes; i++) {

		/* Loop over columns within rows. */

		for (j = 0; j < columns; j++) {

			if (((i * columns) + j) >= mtd->ecclayout->eccbytes)
				break;

			o += sprintf(buf + o, " %3u",
				mtd->ecclayout->eccpos[(i * columns) + j]);

		}

		o += sprintf(buf + o, "\n");

	}

	o += sprintf(buf + o,
		"OOB Available Bytes: %u\n"
		,
		mtd->ecclayout->oobavail
	);

	j = 0;

	for (i = 0; j < mtd->ecclayout->oobavail; i++) {

		j += mtd->ecclayout->oobfree[i].length;

		o += sprintf(buf + o,
			"  [%3u, %2u]\n"
			,
			mtd->ecclayout->oobfree[i].offset,
			mtd->ecclayout->oobfree[i].length
		);

	}

	return o;

}

/**
 * show_device_bbt_pages() - Shows the pages in which BBT's appear.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_bbt_pages(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int                  o = 0;
	unsigned int         i;
	struct imx_nfc_data  *this = dev_get_drvdata(dev);
	struct nand_chip     *nand = &this->nand;

	/* Loop over main BBT pages. */

	if (nand->bbt_td)
		for (i = 0; i < NAND_MAX_CHIPS; i++)
			o += sprintf(buf + o, "%d: 0x%08x\n",
						i, nand->bbt_td->pages[i]);

	/* Loop over mirror BBT pages. */

	if (nand->bbt_md)
		for (i = 0; i < NAND_MAX_CHIPS; i++)
			o += sprintf(buf + o, "%d: 0x%08x\n",
						i, nand->bbt_md->pages[i]);

	return o;

}

/**
 * show_device_cycle_in_ns() - Shows the device's cycle in ns.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_cycle_in_ns(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", get_cycle_in_ns(this));
}

/**
 * store_device_cycle_in_ns() - Sets the device's cycle in ns.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_cycle_in_ns(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int                  error;
	unsigned long        new_cycle_in_ns;
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the new cycle period. */

	if (strict_strtoul(buf, 0, &new_cycle_in_ns))
		return -EINVAL;

	/* Try to implement the new cycle period. */

	error = this->nfc->set_closest_cycle(this, new_cycle_in_ns);

	if (error)
		return -EINVAL;

	/* Return success. */

	return size;

}

/**
 * show_device_interrupt_override() - Shows the device's interrupt override.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_interrupt_override(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	switch (this->interrupt_override) {

	case NEVER:
		return sprintf(buf, "-1\n");
		break;

	case DRIVER_CHOICE:
		return sprintf(buf, "0\n");
		break;

	case ALWAYS:
		return sprintf(buf, "1\n");
		break;

	default:
		return sprintf(buf, "?\n");
		break;

	}

}

/**
 * store_device_interrupt_override() - Sets the device's interrupt override.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_interrupt_override(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	if      (!strcmp(buf, "-1"))
		this->interrupt_override = NEVER;
	else if (!strcmp(buf, "0"))
		this->interrupt_override = DRIVER_CHOICE;
	else if (!strcmp(buf, "1"))
		this->interrupt_override = ALWAYS;
	else
		return -EINVAL;

	return size;

}

/**
 * show_device_auto_op_override() - Shows the device's automatic op override.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_auto_op_override(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	switch (this->auto_op_override) {

	case NEVER:
		return sprintf(buf, "-1\n");
		break;

	case DRIVER_CHOICE:
		return sprintf(buf, "0\n");
		break;

	case ALWAYS:
		return sprintf(buf, "1\n");
		break;

	default:
		return sprintf(buf, "?\n");
		break;

	}

}

/**
 * store_device_auto_op_override() - Sets the device's automatic op override.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_auto_op_override(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	if      (!strcmp(buf, "-1"))
		this->auto_op_override = NEVER;
	else if (!strcmp(buf, "0"))
		this->auto_op_override = DRIVER_CHOICE;
	else if (!strcmp(buf, "1"))
		this->auto_op_override = ALWAYS;
	else
		return -EINVAL;

	return size;

}

/**
 * show_device_inject_ecc_error() - Shows the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer that will receive a representation of the attribute.
 */
static ssize_t show_device_inject_ecc_error(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", this->inject_ecc_error);

}

/**
 * store_device_inject_ecc_error() - Sets the device's error injection flag.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_inject_ecc_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long        new_inject_ecc_error;
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	/* Look for nonsense. */

	if (!size)
		return -EINVAL;

	/* Try to understand the new cycle period. */

	if (strict_strtol(buf, 0, &new_inject_ecc_error))
		return -EINVAL;

	/* Store the value. */

	this->inject_ecc_error = new_inject_ecc_error;

	/* Return success. */

	return size;

}

/**
 * store_device_invalidate_page_cache() - Invalidates the device's page cache.
 *
 * @dev:   The device of interest.
 * @attr:  The attribute of interest.
 * @buf:   A buffer containing a new attribute value.
 * @size:  The size of the buffer.
 */
static ssize_t store_device_invalidate_page_cache(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct imx_nfc_data  *this = dev_get_drvdata(dev);

	/* Invalidate the page cache. */

	this->nand.pagebuf = -1;

	/* Return success. */

	return size;

}

/* Device attributes that appear in sysfs. */

static DEVICE_ATTR(platform_info    , 0444, show_device_platform_info    , 0);
static DEVICE_ATTR(physical_geometry, 0444, show_device_physical_geometry, 0);
static DEVICE_ATTR(nfc_info         , 0444, show_device_nfc_info         , 0);
static DEVICE_ATTR(nfc_geometry     , 0444, show_device_nfc_geometry     , 0);
static DEVICE_ATTR(logical_geometry , 0444, show_device_logical_geometry , 0);
static DEVICE_ATTR(mtd_nand_info    , 0444, show_device_mtd_nand_info    , 0);
static DEVICE_ATTR(mtd_info         , 0444, show_device_mtd_info         , 0);
static DEVICE_ATTR(bbt_pages        , 0444, show_device_bbt_pages        , 0);

static DEVICE_ATTR(cycle_in_ns, 0644,
	show_device_cycle_in_ns, store_device_cycle_in_ns);

static DEVICE_ATTR(interrupt_override, 0644,
	show_device_interrupt_override, store_device_interrupt_override);

static DEVICE_ATTR(auto_op_override, 0644,
	show_device_auto_op_override, store_device_auto_op_override);

static DEVICE_ATTR(inject_ecc_error, 0644,
	show_device_inject_ecc_error, store_device_inject_ecc_error);

static DEVICE_ATTR(invalidate_page_cache, 0644,
	0, store_device_invalidate_page_cache);

static struct device_attribute *device_attributes[] = {
	&dev_attr_platform_info,
	&dev_attr_physical_geometry,
	&dev_attr_nfc_info,
	&dev_attr_nfc_geometry,
	&dev_attr_logical_geometry,
	&dev_attr_mtd_nand_info,
	&dev_attr_mtd_info,
	&dev_attr_bbt_pages,
	&dev_attr_cycle_in_ns,
	&dev_attr_interrupt_override,
	&dev_attr_auto_op_override,
	&dev_attr_inject_ecc_error,
	&dev_attr_invalidate_page_cache,
};

/**
 * validate_the_platform() - Validates information about the platform.
 *
 * Note that this function doesn't validate the NFC version. That's done when
 * the probing process attempts to configure for the specific hardware version.
 *
 * @pdev:	A pointer to the platform device.
 */
static int validate_the_platform(struct platform_device *pdev)
{
	struct device                 *dev   = &pdev->dev;
	struct imx_nfc_platform_data  *pdata = pdev->dev.platform_data;

	/* Validate the clock name. */

	if (!pdata->clock_name) {
		dev_err(dev, "No clock name\n");
		return -ENXIO;
	}

	/* Validate the partitions. */

	if ((pdata->partitions && (!pdata->partition_count)) ||
			(!pdata->partitions && (pdata->partition_count))) {
		dev_err(dev, "Bad partition data\n");
		return -ENXIO;
	}

	/* Return success */

	return 0;

}

/**
 * set_up_the_nfc_hal() - Sets up for the specific NFC hardware HAL.
 *
 * @this:  Per-device data.
 */
static int set_up_the_nfc_hal(struct imx_nfc_data *this)
{
	unsigned int                  i;
	struct nfc_hal                *p;
	struct imx_nfc_platform_data  *pdata = this->pdata;

	for (i = 0; i < ARRAY_SIZE(nfc_hals); i++) {

		p = nfc_hals[i];

		/*
		 * Restrict to 3.2 until others are fully implemented.
		 *
		 * TODO: Remove this.
		 */

		if ((p->major_version != 3) && (p->minor_version != 2))
			continue;

		if ((p->major_version == pdata->nfc_major_version) &&
			(p->minor_version == pdata->nfc_minor_version)) {
			this->nfc = p;
			return 0;
			break;
		}

	}

	dev_err(this->dev, "Unkown NFC version %d.%d\n",
			pdata->nfc_major_version, pdata->nfc_minor_version);

	return !0;

}

/**
 * acquire_resources() - Tries to acquire resources.
 *
 * @this:  Per-device data.
 */
static int acquire_resources(struct imx_nfc_data *this)
{

	int                           error  = 0;
	struct platform_device        *pdev  = this->pdev;
	struct device                 *dev   = this->dev;
	struct imx_nfc_platform_data  *pdata = this->pdata;
	struct resource               *r;

	/* Find the buffers and map them. */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						IMX_NFC_BUFFERS_ADDR_RES_NAME);

	if (!r) {
		dev_err(dev, "Can't get '%s'\n", IMX_NFC_BUFFERS_ADDR_RES_NAME);
		error = -ENXIO;
		goto exit_buffers;
	}

	this->buffers = ioremap(r->start, r->end - r->start + 1);

	if (!this->buffers) {
		dev_err(dev, "Can't remap buffers\n");
		error = -EIO;
		goto exit_buffers;
	}

	/* Find the primary registers and map them. */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME);

	if (!r) {
		dev_err(dev, "Can't get '%s'\n",
					IMX_NFC_PRIMARY_REGS_ADDR_RES_NAME);
		error = -ENXIO;
		goto exit_primary_registers;
	}

	this->primary_regs = ioremap(r->start, r->end - r->start + 1);

	if (!this->primary_regs) {
		dev_err(dev, "Can't remap the primary registers\n");
		error = -EIO;
		goto exit_primary_registers;
	}

	/* Check for secondary registers. */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME);

	if (r && !this->nfc->has_secondary_regs) {

		dev_err(dev, "Resource '%s' should not be present\n",
					IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME);
		error = -ENXIO;
		goto exit_secondary_registers;

	}

	if (this->nfc->has_secondary_regs) {

		if (!r) {
			dev_err(dev, "Can't get '%s'\n",
					IMX_NFC_SECONDARY_REGS_ADDR_RES_NAME);
			error = -ENXIO;
			goto exit_secondary_registers;
		}

		this->secondary_regs = ioremap(r->start, r->end - r->start + 1);

		if (!this->secondary_regs) {
			dev_err(dev,
				"Can't remap the secondary registers\n");
			error = -EIO;
			goto exit_secondary_registers;
		}

	}

	/* Find out what our interrupt is and try to own it. */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						IMX_NFC_INTERRUPT_RES_NAME);

	if (!r) {
		dev_err(dev, "Can't get '%s'\n", IMX_NFC_INTERRUPT_RES_NAME);
		error = -ENXIO;
		goto exit_irq;
	}

	this->interrupt = r->start;

	error = request_irq(this->interrupt,
				nfc_util_isr, 0, this->dev->bus_id, this);

	if (error) {
		dev_err(dev, "Can't own interrupt %d\n", this->interrupt);
		goto exit_irq;
	}

	/* Find out the name of our clock and try to own it. */

	this->clock = clk_get(dev, pdata->clock_name);

	if (this->clock == ERR_PTR(-ENOENT)) {
		dev_err(dev, "Can't get clock '%s'\n", pdata->clock_name);
		error = -ENXIO;
		goto exit_clock;
	}

	/* Return success. */

	return 0;

	/* Error return paths begin here. */

exit_clock:
	free_irq(this->interrupt, this);
exit_irq:
	if (this->secondary_regs)
		iounmap(this->secondary_regs);
exit_secondary_registers:
	iounmap(this->primary_regs);
exit_primary_registers:
	iounmap(this->buffers);
exit_buffers:
	return error;

}

/**
 * release_resources() - Releases resources.
 *
 * @this:  Per-device data.
 */
static void release_resources(struct imx_nfc_data *this)
{

	/* Release our clock. */

	clk_disable(this->clock);
	clk_put(this->clock);

	/* Release our interrupt. */

	free_irq(this->interrupt, this);

	/* Release mapped memory. */

	iounmap(this->buffers);
	iounmap(this->primary_regs);
	if (this->secondary_regs)
		iounmap(this->secondary_regs);

}

/**
 * register_with_mtd() - Registers this medium with MTD.
 *
 * @this:  Per-device data.
 */
static int register_with_mtd(struct imx_nfc_data *this)
{
	int                           error  = 0;
	struct mtd_info               *mtd   = &this->mtd;
	struct nand_chip              *nand  = &this->nand;
	struct device                 *dev   = this->dev;
	struct imx_nfc_platform_data  *pdata = this->pdata;

	/* Link the MTD structures together, along with our own data. */

	mtd->priv  = nand;
	nand->priv = this;

	/* Prepare the MTD structure. */

	mtd->owner = THIS_MODULE;

	/*
	 * Signal Control Functions
	 */

	nand->cmd_ctrl = mil_cmd_ctrl;

	/*
	 * Chip Control Functions
	 *
	 * Not all of our NFC hardware versions expose Ready/Busy signals. For
	 * versions that don't, the is_ready function pointer will be NULL. In
	 * those cases, we leave the dev_ready member unassigned, which will
	 * cause the HIL to use a reference implementation's algorithm to
	 * discover when the hardware is ready.
	 */

	nand->select_chip = mil_select_chip;
	nand->cmdfunc     = mil_cmdfunc;
	nand->waitfunc    = mil_waitfunc;

	if (this->nfc->is_ready)
		nand->dev_ready = mil_dev_ready;

	/*
	 * Low-level I/O Functions
	 */

	nand->read_byte  = mil_read_byte;
	nand->read_word  = mil_read_word;
	nand->read_buf   = mil_read_buf;
	nand->write_buf  = mil_write_buf;
	nand->verify_buf = mil_verify_buf;

	/*
	 * ECC Control Functions
	 */

	nand->ecc.hwctl     = mil_ecc_hwctl;
	nand->ecc.calculate = mil_ecc_calculate;
	nand->ecc.correct   = mil_ecc_correct;

	/*
	 * ECC-Aware I/O Functions
	 */

	nand->ecc.read_page      = mil_ecc_read_page;
	nand->ecc.read_page_raw  = mil_ecc_read_page_raw;
	nand->ecc.write_page     = mil_ecc_write_page;
	nand->ecc.write_page_raw = mil_ecc_write_page_raw;

	/*
	 * High-level I/O Functions
	 */

	nand->write_page    = mil_write_page;
	nand->ecc.read_oob  = mil_ecc_read_oob;
	nand->ecc.write_oob = mil_ecc_write_oob;

	/*
	 * Bad Block Marking Functions
	 *
	 * We want to use the reference block_bad and block_markbad
	 * implementations, so we don't assign those members.
	 */

	nand->scan_bbt = mil_scan_bbt;

	/*
	 * Error Recovery Functions
	 *
	 * We don't fill in the errstat function pointer because it's optional
	 * and we don't have a need for it.
	 */

	/*
	 * Device Attributes
	 *
	 * We don't fill in the chip_delay member because we don't have a need
	 * for it.
	 *
	 * We support only 8-bit Flash bus width.
	 */

	/*
	 * ECC Attributes
	 *
	 * Members that aren't set here are configured by a version-specific
	 * set_geometry() function.
	 */

	nand->ecc.mode    = NAND_ECC_HW;
	nand->ecc.size    = NFC_MAIN_BUF_SIZE;
	nand->ecc.prepad  = 0;
	nand->ecc.postpad = 0;

	/*
	 * Bad Block Management Attributes
	 *
	 * We don't fill in the following attributes:
	 *
	 *     badblockpos
	 *     bbt
	 *     badblock_pattern
	 *     bbt_erase_shift
	 *
	 * These attributes aren't hardware-specific, and the HIL makes fine
	 * choices without our help.
	 */

	nand->options |= NAND_USE_FLASH_BBT;
	nand->bbt_td   = &bbt_main_descriptor;
	nand->bbt_md   = &bbt_mirror_descriptor;

	/*
	 * Device Control
	 *
	 * We don't fill in the controller attribute. In principle, we could set
	 * up a structure to represent the controller. However, since it's
	 * vanishingly improbable that we'll have more than one medium behind
	 * the controller, it's not worth the effort. We let the HIL handle it.
	 */

	/*
	 * Memory-mapped I/O
	 *
	 * We don't fill in the following attributes:
	 *
	 *     IO_ADDR_R
	 *     IO_ADDR_W
	 *
	 * None of these are necessary because we don't have a memory-mapped
	 * implementation.
	 */

	/*
	 * Install a "fake" ECC layout.
	 *
	 * We'll be calling nand_scan() to do the final MTD setup. If we haven't
	 * already chosen an ECC layout, then nand_scan() will choose one based
	 * on the part geometry it discovers. Unfortunately, it doesn't make
	 * good choices. It would be best if we could install the correct ECC
	 * layout now, before we call nand_scan(). We can't do that because we
	 * don't know the medium geometry yet. Here, we install a "fake" ECC
	 * layout just to stop nand_scan() from trying to pick on for itself.
	 * Later, in imx_nfc_scan_bbt(), when we know the medium geometry, we'll
	 * install the correct choice.
	 *
	 * Of course, this tactic depends critically on nand_scan() not using
	 * the fake layout before we can install a good one. This is in fact the
	 * case.
	 */

	nand->ecc.layout = &nfc_geometry_512_16_RS_ECC1.mtd_layout;

	/*
	 * Ask the NAND Flash system to scan for chips.
	 *
	 * This will fill in reference implementations for all the members of
	 * the MTD structures that we didn't set, and will make the medium fully
	 * usable.
	 */

	error = nand_scan(mtd, this->nfc->max_chip_count);

	if (error) {
		dev_err(dev, "Chip scan failed\n");
		error = -ENXIO;
		goto exit_scan;
	}

	/* Register the MTD that represents the entire medium. */

	mtd->name = "NAND Flash Medium";

	add_mtd_device(mtd);

	/* Check if we're doing partitions and register MTD's accordingly. */

	#ifdef CONFIG_MTD_PARTITIONS

		/*
		 * Look for partition information. If we find some, install
		 * them. Otherwise, use the partitions handed to us by the
		 * platform.
		 */

		this->partition_count =
			parse_mtd_partitions(mtd, partition_source_types,
							&this->partitions, 0);

		if ((this->partition_count <= 0) && (pdata->partitions)) {
			this->partition_count = pdata->partition_count;
			this->partitions      = pdata->partitions;
		}

		if (this->partitions)
			add_mtd_partitions(mtd, this->partitions,
							this->partition_count);

	#endif

	/* Return success. */

	return 0;

	/* Error return paths begin here. */

exit_scan:
	return error;

}

/**
 * unregister_with_mtd() - Unregisters this medium with MTD.
 *
 * @this:  Per-device data.
 */
static void unregister_with_mtd(struct imx_nfc_data *this)
{

	/* Get MTD to let go. */

	nand_release(&this->mtd);

}

/**
 * manage_sysfs_files() - Creates/removes sysfs files for this device.
 *
 * @this:  Per-device data.
 */
static void manage_sysfs_files(struct imx_nfc_data *this, int create)
{
	int                      error;
	unsigned int             i;
	struct device_attribute  **attr;

	for (i = 0, attr = device_attributes;
			i < ARRAY_SIZE(device_attributes); i++, attr++) {

		if (create) {
			error = device_create_file(this->dev, *attr);
			if (error) {
				while (--attr >= device_attributes)
					device_remove_file(this->dev, *attr);
				return;
			}
		} else {
			device_remove_file(this->dev, *attr);
		}

	}

}

/**
 * imx_nfc_probe() - Probes for a device and, if possible, takes ownership.
 *
 * @pdev:  A pointer to the platform device.
 */
static int imx_nfc_probe(struct platform_device *pdev)
{
	int                           error  = 0;
	char                          *symmetric_clock;
	struct clk                    *parent_clock;
	unsigned long                 parent_clock_rate_in_hz;
	unsigned long                 parent_clock_rate_in_mhz;
	unsigned long                 nfc_clock_rate_in_hz;
	unsigned long                 nfc_clock_rate_in_mhz;
	unsigned long                 clock_divisor;
	unsigned long                 cycle_in_ns;
	struct device                 *dev   = &pdev->dev;
	struct imx_nfc_platform_data  *pdata = pdev->dev.platform_data;
	struct imx_nfc_data           *this  = 0;

	/* Say hello. */

	dev_info(dev, "Probing...\n");

	/* Check if we're enabled. */

	if (!imx_nfc_module_enable) {
		dev_info(dev, "Disabled\n");
		return -ENXIO;
	}

	/* Validate the platform device data. */

	error = validate_the_platform(pdev);

	if (error)
		goto exit_validate_platform;

	/* Allocate memory for the per-device data. */

	this = kzalloc(sizeof(*this), GFP_KERNEL);

	if (!this) {
		dev_err(dev, "Failed to allocate per-device memory\n");
		error = -ENOMEM;
		goto exit_allocate_this;
	}

	/* Link our per-device data to the owning device. */

	platform_set_drvdata(pdev, this);

	/* Fill in the convenience pointers in our per-device data. */

	this->pdev  = pdev;
	this->dev   = &pdev->dev;
	this->pdata = pdata;

	/* Initialize the interrupt service pathway. */

	init_completion(&this->done);

	/* Set up the NFC HAL. */

	error = set_up_the_nfc_hal(this);

	if (error)
		goto exit_set_up_nfc_hal;

	/* Attempt to acquire the resources we need. */

	error = acquire_resources(this);

	if (error)
		goto exit_acquire_resources;

	/* Initialize the NFC HAL. */

	if (this->nfc->init(this)) {
		error = -ENXIO;
		goto exit_nfc_init;
	}

	/* Tell the platform we're bringing this device up. */

	if (pdata->init)
		error = pdata->init();

	if (error)
		goto exit_platform_init;

	/* Report. */

	parent_clock             = clk_get_parent(this->clock);
	parent_clock_rate_in_hz  = clk_get_rate(parent_clock);
	parent_clock_rate_in_mhz = parent_clock_rate_in_hz / 1000000;
	nfc_clock_rate_in_hz     = clk_get_rate(this->clock);
	nfc_clock_rate_in_mhz    = nfc_clock_rate_in_hz / 1000000;

	clock_divisor   = parent_clock_rate_in_hz / nfc_clock_rate_in_hz;
	symmetric_clock = this->nfc->get_symmetric(this) ? "Yes" : "No";
	cycle_in_ns     = get_cycle_in_ns(this);

	dev_dbg(dev, "-------------\n");
	dev_dbg(dev, "Configuration\n");
	dev_dbg(dev, "-------------\n");
	dev_dbg(dev, "NFC Version      : %d.%d\n"  , this->nfc->major_version,
						     this->nfc->minor_version);
	dev_dbg(dev, "Buffers          : 0x%p\n"   , this->buffers);
	dev_dbg(dev, "Primary Regs     : 0x%p\n"   , this->primary_regs);
	dev_dbg(dev, "Secondary Regs   : 0x%p\n"   , this->secondary_regs);
	dev_dbg(dev, "Interrupt        : %u\n"     , this->interrupt);
	dev_dbg(dev, "Clock Name       : %s\n"     , pdata->clock_name);
	dev_dbg(dev, "Parent Clock Rate: %lu Hz (%lu MHz)\n",
						     parent_clock_rate_in_hz,
						     parent_clock_rate_in_mhz);
	dev_dbg(dev, "Clock Divisor    : %lu\n",     clock_divisor);
	dev_dbg(dev, "NFC Clock Rate   : %lu Hz (%lu MHz)\n",
						     nfc_clock_rate_in_hz,
						     nfc_clock_rate_in_mhz);
	dev_dbg(dev, "Symmetric Clock  : %s\n"     , symmetric_clock);
	dev_dbg(dev, "Actual Cycle     : %lu ns\n" , cycle_in_ns);
	dev_dbg(dev, "Target Cycle     : %u ns\n"  , pdata->target_cycle_in_ns);

	/* Initialize the Medium Abstraction Layer. */

	mal_init(this);

	/* Initialize the MTD Interface Layer. */

	mil_init(this);

	/* Register this medium with MTD. */

	error = register_with_mtd(this);

	if (error)
		goto exit_mtd_registration;

	/* Create sysfs entries for this device. */

	manage_sysfs_files(this, true);

	/* Return success. */

	return 0;

	/* Error return paths begin here. */

exit_mtd_registration:
	if (pdata->exit)
		pdata->exit();
exit_platform_init:
	this->nfc->exit(this);
exit_nfc_init:
exit_acquire_resources:
exit_set_up_nfc_hal:
	platform_set_drvdata(pdev, NULL);
	kfree(this);
exit_allocate_this:
exit_validate_platform:
	return error;

}

/**
 * imx_nfc_remove() - Dissociates this driver from the given device.
 *
 * @pdev:  A pointer to the device.
 */
static int __exit imx_nfc_remove(struct platform_device *pdev)
{
	struct imx_nfc_data  *this  = platform_get_drvdata(pdev);

	/* Remove sysfs entries for this device. */

	manage_sysfs_files(this, false);

	/* Unregister with the NAND Flash MTD system. */

	unregister_with_mtd(this);

	/* Tell the platform we're shutting down this device. */

	if (this->pdata->exit)
		this->pdata->exit();

	/* Shut down the NFC. */

	this->nfc->exit(this);

	/* Release our resources. */

	release_resources(this);

	/* Unlink our per-device data from the platform device. */

	platform_set_drvdata(pdev, NULL);

	/* Free our per-device data. */

	kfree(this);

	/* Return success. */

	return 0;
}

#ifdef CONFIG_PM

/**
 * suspend() - Puts the NFC in a low power state.
 *
 * Refer to Documentation/driver-model/driver.txt for more information.
 *
 * @pdev:  A pointer to the device.
 * @state: The new power state.
 */

static int imx_nfc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int                  error = 0;
	struct imx_nfc_data  *this = platform_get_drvdata(pdev);
	struct mtd_info      *mtd  = &this->mtd;
	struct device        *dev  = &this->pdev->dev;

	dev_dbg(dev, "Suspending...\n");

	/* Suspend MTD's use of this device. */

	error = mtd->suspend(mtd);

	/* Suspend the actual hardware. */

	clk_disable(this->clock);

	return error;

}

/**
 * resume() - Brings the NFC back from a low power state.
 *
 * Refer to Documentation/driver-model/driver.txt for more information.
 *
 * @pdev:  A pointer to the device.
 */
static int imx_nfc_resume(struct platform_device *pdev)
{
	struct imx_nfc_data  *this = platform_get_drvdata(pdev);
	struct mtd_info      *mtd  = &this->mtd;
	struct device        *dev  = &this->pdev->dev;

	dev_dbg(dev, "Resuming...\n");

	/* Resume MTD's use of this device. */

	mtd->resume(mtd);

	return 0;

}

#else

#define suspend  NULL
#define resume   NULL

#endif  /* CONFIG_PM */

/*
 * This structure represents this driver to the platform management system.
 */
static struct platform_driver imx_nfc_driver = {
	.driver = {
		.name = IMX_NFC_DRIVER_NAME,
	},
	.probe   = imx_nfc_probe,
	.remove  = __exit_p(imx_nfc_remove),
	.suspend = imx_nfc_suspend,
	.resume  = imx_nfc_resume,
};

/**
 * imx_nfc_init() - Initializes this module.
 */
static int __init imx_nfc_init(void)
{

	pr_info("i.MX NFC driver %s\n", DRIVER_VERSION);

	/* Register this driver with the platform management system. */

	if (platform_driver_register(&imx_nfc_driver) != 0) {
		pr_err("i.MX NFC driver registration failed\n");
		return -ENODEV;
	}

	return 0;

}

/**
 * imx_nfc_exit() - Deactivates this module.
 */
static void __exit imx_nfc_exit(void)
{
	pr_debug("i.MX NFC driver exiting...\n");
	platform_driver_unregister(&imx_nfc_driver);
}

module_init(imx_nfc_init);
module_exit(imx_nfc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX NAND Flash Controller Driver");
MODULE_LICENSE("GPL");
