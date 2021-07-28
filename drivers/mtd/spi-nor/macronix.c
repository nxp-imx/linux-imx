// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define SPINOR_OP_RD_CR2		0x71		/* Read configuration register 2 */
#define SPINOR_OP_WR_CR2		0x72		/* Write configuration register 2 */
#define SPINOR_REG_MXIC_CR2_MODE	0x00000000	/* For setting octal DTR mode */
#define SPINOR_REG_MXIC_OPI_DTR_EN	0x2		/* Enable Octal DTR */
#define SPINOR_REG_MXIC_SPI_EN		0x0		/* Enable SPI */
#define SPINOR_OP_OPI_DTR_RD		0xEE		/* OPI DTR first read opcode */

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt,
			    struct spi_nor_flash_parameter *params)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[BFPT_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
};

/**
 * spi_nor_macronix_octal_dtr_enable() - Enable octal DTR on Macronix flashes.
 * @nor:		pointer to a 'struct spi_nor'
 * @enable:		whether to enable Octal DTR or switch back to SPI
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_macronix_octal_dtr_enable(struct spi_nor *nor, bool enable)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf, i;
	int ret;

	/* Set/unset the octal and DTR enable bits. */
	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	if (enable) {
		buf[0] = SPINOR_REG_MXIC_OPI_DTR_EN;
	} else {
		/*
		 * The register is 1-byte wide, but 1-byte transactions are not
		 * allowed in 8D-8D-8D mode. Since there is no register at the
		 * next location, just initialize the value to 0 and let the
		 * transaction go on.
		 */
		buf[0] = SPINOR_REG_MXIC_SPI_EN;
		buf[1] = 0x0;
	}

	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WR_CR2, 1),
			   SPI_MEM_OP_ADDR(4, SPINOR_REG_MXIC_CR2_MODE, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_OUT(enable ? 1 : 2, buf, 1));

	if (!enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	/* Read flash ID to make sure the switch was successful. */
	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDID, 1),
			   SPI_MEM_OP_ADDR(enable ? 4 : 0, 0, 1),
			   SPI_MEM_OP_DUMMY(enable ? 4 : 0, 1),
			   SPI_MEM_OP_DATA_IN(SPI_NOR_MAX_ID_LEN, buf, 1));

	if (enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	if (enable) {
		for (i = 0; i < nor->info->id_len; i++)
			if (buf[i * 2] != nor->info->id[i])
				return -EINVAL;
	} else {
		if (memcmp(buf, nor->info->id, nor->info->id_len))
			return -EINVAL;
	}

	return 0;
}

static void octaflash_default_init(struct spi_nor *nor)
{
	nor->params->octal_dtr_enable = spi_nor_macronix_octal_dtr_enable;
}

static struct spi_nor_fixups octaflash_fixups = {
	.default_init = octaflash_default_init,
};

static void mx25uw51345g_post_sfdp_fixup(struct spi_nor *nor)
{
	nor->params->hwcaps.mask |= SNOR_HWCAPS_READ_8_8_8_DTR;
	spi_nor_set_read_settings(&nor->params->reads[SNOR_CMD_READ_8_8_8_DTR],
				  0, 20, SPINOR_OP_OPI_DTR_RD,
				  SNOR_PROTO_8_8_8_DTR);
}

static struct spi_nor_fixups mx25uw51345g_fixups = {
	.default_init = octaflash_default_init,
	.post_sfdp = mx25uw51345g_post_sfdp_fixup,
};

static const struct flash_info macronix_parts[] = {
	/* Macronix */
	{ "mx25l512e",   INFO(0xc22010, 0, 64 * 1024,   1, SECT_4K) },
	{ "mx25l2005a",  INFO(0xc22012, 0, 64 * 1024,   4, SECT_4K) },
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16, 0) },
	{ "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, SECT_4K) },
	{ "mx25l3255e",  INFO(0xc29e16, 0, 64 * 1024,  64, SECT_4K) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, SECT_4K) },
	{ "mx25u2033e",  INFO(0xc22532, 0, 64 * 1024,   4, SECT_4K) },
	{ "mx25u3235f",	 INFO(0xc22536, 0, 64 * 1024,  64,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25u4035",   INFO(0xc22533, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25u8035",   INFO(0xc22534, 0, 64 * 1024,  16, SECT_4K) },
	{ "mx25u6435f",  INFO(0xc22537, 0, 64 * 1024, 128, SECT_4K) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, SECT_4K) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },
	{ "mx25r1635f",  INFO(0xc22815, 0, 64 * 1024,  32,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25r3235f",  INFO(0xc22816, 0, 64 * 1024,  64,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25u12835f", INFO(0xc22538, 0, 64 * 1024, 256,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512,
			      SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ)
		.fixups = &mx25l25635_fixups },
	{ "mx25u25635f", INFO(0xc22539, 0, 64 * 1024, 512,
			      SECT_4K | SPI_NOR_4B_OPCODES) },
	{ "mx25u51245g", INFO(0xc2253a, 0, 64 * 1024, 1024,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ "mx25v8035f",  INFO(0xc22314, 0, 64 * 1024,  16,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512, 0) },
	{ "mx66l51235l", INFO(0xc2201a, 0, 64 * 1024, 1024,
			      SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			      SPI_NOR_4B_OPCODES) },
	{ "mx66u51235f", INFO(0xc2253a, 0, 64 * 1024, 1024,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ "mx66l1g45g",  INFO(0xc2201b, 0, 64 * 1024, 2048,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ) },
	{ "mx66l1g55g",  INFO(0xc2261b, 0, 64 * 1024, 2048,
			      SPI_NOR_QUAD_READ) },
	{ "mx66u2g45g",	 INFO(0xc2253c, 0, 64 * 1024, 4096,
			      SECT_4K | SPI_NOR_DUAL_READ |
			      SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ "mx66lm2g45g", INFO(0xc2853c, 0, 64 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66lm1g45g", INFO(0xc2853b, 0, 32 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66lw1g45g", INFO(0xc2863b, 0, 32 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25lm51245g", INFO(0xc2853a, 0, 16 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25lw51245g", INFO(0xc2863a, 0, 16 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25lm25645g", INFO(0xc28539, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25lw25645g", INFO(0xc28639, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66um2g45g", INFO(0xc2803c, 0, 64 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66uw2g345g", INFO(0xc2843c, 0, 64 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66uw2g345gx0", INFO(0xc2943c, 0, 64 * 1024, 4096,
				 SECT_4K | SPI_NOR_OCTAL_DTR_READ |
				 SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66um1g45g", INFO(0xc2803b, 0, 32 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66um1g45g40", INFO(0xc2808b, 0, 32 * 1024, 4096,
				SECT_4K | SPI_NOR_OCTAL_DTR_READ |
				SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx66uw1g45g", INFO(0xc2813b, 0, 32 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25um51245g", INFO(0xc2803a, 0, 16 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw51245g", INFO(0xc2813a, 0, 16 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw51345g", INFO(0xc2843a, 0, 16 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &mx25uw51345g_fixups },
	{ "mx25um25645g", INFO(0xc28039, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw25645g", INFO(0xc28139, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25um25345g", INFO(0xc28339, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw25345g", INFO(0xc28439, 0, 8 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw12845g", INFO(0xc28138, 0, 4 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw12a45g", INFO(0xc28938, 0, 4 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw12345g", INFO(0xc28438, 0, 4 * 1024, 4096,
			       SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			       SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw6445g", INFO(0xc28137, 0, 2 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
	{ "mx25uw6345g", INFO(0xc28437, 0, 2 * 1024, 4096,
			      SECT_4K | SPI_NOR_OCTAL_DTR_READ |
			      SPI_NOR_OCTAL_DTR_PP | SPI_NOR_4B_OPCODES)
		.fixups = &octaflash_fixups },
};

static void macronix_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
	nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode;
}

static const struct spi_nor_fixups macronix_fixups = {
	.default_init = macronix_default_init,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_parts,
	.nparts = ARRAY_SIZE(macronix_parts),
	.fixups = &macronix_fixups,
};
