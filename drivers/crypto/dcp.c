/*
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Based on geode-aes.c
 * Copyright (C) 2004-2006, Advanced Micro Devices, Inc.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysdev.h>
#include <linux/bitops.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include "dcp.h"
#include "dcp_bootstream_ioctl.h"

/* Following data only used by DCP bootstream interface */
struct dcpboot_dma_area {
	struct dcp_hw_packet hw_packet;
	uint16_t block[16];
};

struct dcp {
	struct device *dev;
	spinlock_t lock;
	struct mutex op_mutex[DCP_NUM_CHANNELS];
	struct completion op_wait[DCP_NUM_CHANNELS];
	int wait[DCP_NUM_CHANNELS];
	int dcp_vmi_irq;
	int dcp_irq;
	u32 dcp_regs_base;
	ulong clock_state;
	bool chan_in_use[DCP_NUM_CHANNELS];

	/* Following buffers used in hashing to meet 64-byte len alignment */
	char *buf1;
	char *buf2;
	dma_addr_t buf1_phys;
	dma_addr_t buf2_phys;
	struct dcp_hash_coherent_block *buf1_desc;
	struct dcp_hash_coherent_block *buf2_desc;
	struct dcp_hash_coherent_block *user_buf_desc;

	/* Following data only used by DCP bootstream interface */
	struct dcpboot_dma_area *dcpboot_dma_area;
	dma_addr_t dcpboot_dma_area_phys;
};

/* cipher flags */
#define DCP_ENC	0x0001
#define DCP_DEC	0x0002
#define DCP_ECB	0x0004
#define DCP_CBC	0x0008
#define DCP_CBC_INIT	0x0010
#define DCP_OTPKEY	0x0020

/* hash flags */
#define DCP_INIT	0x0001
#define DCP_UPDATE	0x0002
#define DCP_FINAL	0x0004

#define DCP_AES	0x1000
#define DCP_SHA1	0x2000
#define DCP_CRC32	0x3000
#define DCP_COPY	0x4000
#define DCP_FILL	0x5000
#define DCP_MODE_MASK	0xf000

/* clock defines */
#define CLOCK_ON	1
#define CLOCK_OFF	0

struct dcp_op {

	unsigned int flags;

	void *src;
	dma_addr_t src_phys;

	void *dst;
	dma_addr_t dst_phys;

	int len;

	/* the key contains the IV for block modes */
	union {
		struct {
			u8 key[2 * AES_KEYSIZE_128]
				__attribute__ ((__aligned__(32)));
			dma_addr_t key_phys;
			int keylen;
		} cipher;
		struct {
			u8 digest[SHA256_DIGEST_SIZE]
				__attribute__ ((__aligned__(32)));
			dma_addr_t digest_phys;
			int digestlen;
			int init;
		} hash;
	};

	union {
		struct crypto_blkcipher *blk;
		struct crypto_cipher *cip;
		struct crypto_hash *hash;
	} fallback;

	struct dcp_hw_packet pkt
		__attribute__ ((__aligned__(32)));
};

struct dcp_hash_coherent_block {
	struct dcp_hw_packet pkt[1]
		__attribute__ ((__aligned__(32)));
	u8 digest[SHA256_DIGEST_SIZE]
		__attribute__ ((__aligned__(32)));
	unsigned int len;
	dma_addr_t src_phys;
	void *src;
	void *dst;
	dma_addr_t my_phys;
	u32 hash_sel;
	struct dcp_hash_coherent_block *next;
};

struct dcp_hash_op {

	unsigned int flags;

	/* the key contains the IV for block modes */
	union {
		struct {
			u8 key[2 * AES_KEYSIZE_128]
				__attribute__ ((__aligned__(32)));
			dma_addr_t key_phys;
			int keylen;
		} cipher;
		struct {
			u8 digest[SHA256_DIGEST_SIZE]
				__attribute__ ((__aligned__(32)));
			dma_addr_t digest_phys;
			int digestlen;
			int init;
		} hash;
	};

	u32 length;
	struct dcp_hash_coherent_block *head_desc;
	struct dcp_hash_coherent_block *tail_desc;
};

/* only one */
static struct dcp *global_sdcp;

static void dcp_clock(struct dcp *sdcp,  ulong state, bool force)
{
	u32 chan;
	struct clk *clk = clk_get(sdcp->dev, "dcp_clk");

	/* unless force is true (used during suspend/resume), if any
	  * channel is running, then clk is already on, and must stay on */
	if (!force)
		for (chan = 0; chan < DCP_NUM_CHANNELS; chan++)
			if (sdcp->chan_in_use[chan])
				goto exit;

	if (state == CLOCK_OFF) {
		/* gate at clock source */
		if (!IS_ERR(clk))
			clk_disable(clk);
		/* gate at DCP */
		else
			__raw_writel(BM_DCP_CTRL_CLKGATE,
				sdcp->dcp_regs_base + HW_DCP_CTRL_SET);

		sdcp->clock_state = CLOCK_OFF;

	} else {
		/* ungate at clock source */
		if (!IS_ERR(clk))
			clk_enable(clk);
		/* ungate at DCP */
		else
			__raw_writel(BM_DCP_CTRL_CLKGATE,
				sdcp->dcp_regs_base + HW_DCP_CTRL_CLR);

		sdcp->clock_state = CLOCK_ON;
	}

exit:
	return;
}

static void dcp_perform_op(struct dcp_op *op)
{
	struct dcp *sdcp = global_sdcp;
	struct mutex *mutex;
	struct dcp_hw_packet *pkt;
	int chan;
	u32 pkt1, pkt2;
	unsigned long timeout;
	dma_addr_t pkt_phys;
	u32 stat;

	pkt1 = BM_DCP_PACKET1_DECR_SEMAPHORE | BM_DCP_PACKET1_INTERRUPT;

	switch (op->flags & DCP_MODE_MASK) {

	case DCP_AES:

		chan = CIPHER_CHAN;

		/* key is at the payload */
		pkt1 |= BM_DCP_PACKET1_ENABLE_CIPHER;
		if ((op->flags & DCP_OTPKEY) == 0)
			pkt1 |= BM_DCP_PACKET1_PAYLOAD_KEY;
		if (op->flags & DCP_ENC)
			pkt1 |= BM_DCP_PACKET1_CIPHER_ENCRYPT;
		if (op->flags & DCP_CBC_INIT)
			pkt1 |= BM_DCP_PACKET1_CIPHER_INIT;

		pkt2 = BF(0, DCP_PACKET2_CIPHER_CFG) |
		       BF(0, DCP_PACKET2_KEY_SELECT) |
		       BF(BV_DCP_PACKET2_CIPHER_SELECT__AES128,
		       DCP_PACKET2_CIPHER_SELECT);

		if (op->flags & DCP_ECB)
			pkt2 |= BF(BV_DCP_PACKET2_CIPHER_MODE__ECB,
				DCP_PACKET2_CIPHER_MODE);
		else if (op->flags & DCP_CBC)
			pkt2 |= BF(BV_DCP_PACKET2_CIPHER_MODE__CBC,
				DCP_PACKET2_CIPHER_MODE);

		break;

	case DCP_SHA1:

		chan = HASH_CHAN;

		pkt1 |= BM_DCP_PACKET1_ENABLE_HASH;
		if (op->flags & DCP_INIT)
			pkt1 |= BM_DCP_PACKET1_HASH_INIT;
		if (op->flags & DCP_FINAL) {
			pkt1 |= BM_DCP_PACKET1_HASH_TERM;
			BUG_ON(op->hash.digest == NULL);
		}

		pkt2 = BF(BV_DCP_PACKET2_HASH_SELECT__SHA1,
			DCP_PACKET2_HASH_SELECT);
		break;

	default:
		dev_err(sdcp->dev, "Unsupported mode\n");
		return;
	}

	mutex = &sdcp->op_mutex[chan];
	pkt = &op->pkt;

	pkt->pNext = 0;
	pkt->pkt1 = pkt1;
	pkt->pkt2 = pkt2;
	pkt->pSrc = (u32)op->src_phys;
	pkt->pDst = (u32)op->dst_phys;
	pkt->size = op->len;
	pkt->pPayload = chan == CIPHER_CHAN ?
		(u32)op->cipher.key_phys : (u32)op->hash.digest_phys;
	pkt->stat = 0;

	pkt_phys = dma_map_single(sdcp->dev, pkt, sizeof(*pkt),
			DMA_BIDIRECTIONAL);
	if (dma_mapping_error(sdcp->dev, pkt_phys)) {
		dev_err(sdcp->dev, "Unable to map packet descriptor\n");
		return;
	}

	/* submit the work */
	mutex_lock(mutex);
	dcp_clock(sdcp, CLOCK_ON, false);
	sdcp->chan_in_use[chan] = true;

	__raw_writel(-1, sdcp->dcp_regs_base + HW_DCP_CHnSTAT_CLR(chan));

	/* Load the work packet pointer and bump the channel semaphore */
	__raw_writel((u32)pkt_phys, sdcp->dcp_regs_base +
		HW_DCP_CHnCMDPTR(chan));

	/* XXX wake from interrupt instead of looping */
	timeout = jiffies + msecs_to_jiffies(1000);

	sdcp->wait[chan] = 0;
	__raw_writel(BF(1, DCP_CHnSEMA_INCREMENT), sdcp->dcp_regs_base
		+ HW_DCP_CHnSEMA(chan));
	while (time_before(jiffies, timeout) && sdcp->wait[chan] == 0)
		cpu_relax();

	if (!time_before(jiffies, timeout)) {
		dev_err(sdcp->dev, "Timeout while waiting STAT 0x%08x\n",
				__raw_readl(sdcp->dcp_regs_base + HW_DCP_STAT));
		goto out;
	}

	stat = __raw_readl(sdcp->dcp_regs_base + HW_DCP_CHnSTAT(chan));
	if ((stat & 0xff) != 0)
		dev_err(sdcp->dev, "Channel stat error 0x%02x\n",
				__raw_readl(sdcp->dcp_regs_base +
				HW_DCP_CHnSTAT(chan)) & 0xff);
out:
	sdcp->chan_in_use[chan] = false;
	dcp_clock(sdcp, CLOCK_OFF, false);
	mutex_unlock(mutex);
	dma_unmap_single(sdcp->dev, pkt_phys, sizeof(*pkt), DMA_TO_DEVICE);
}

static int dcp_aes_setkey_cip(struct crypto_tfm *tfm, const u8 *key,
		unsigned int len)
{
	struct dcp_op *op = crypto_tfm_ctx(tfm);
	unsigned int ret;

	op->cipher.keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(op->cipher.key, key, len);
		return 0;
	}

	if (len != AES_KEYSIZE_192 && len != AES_KEYSIZE_256) {
		/* not supported at all */
		tfm->crt_flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	/*
	 * The requested key size is not supported by HW, do a fallback
	 */
	op->fallback.blk->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	op->fallback.blk->base.crt_flags |= (tfm->crt_flags &
						CRYPTO_TFM_REQ_MASK);

	ret = crypto_cipher_setkey(op->fallback.cip, key, len);
	if (ret) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (op->fallback.blk->base.crt_flags &
					CRYPTO_TFM_RES_MASK);
	}
	return ret;
}

static void dcp_aes_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128)) {
		crypto_cipher_encrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->flags = DCP_AES | DCP_ENC | DCP_ECB;
	op->len = AES_KEYSIZE_128;

	/* map the data */
	op->src_phys = dma_map_single(sdcp->dev, (void *)in, AES_KEYSIZE_128,
					DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->src_phys)) {
		dev_err(sdcp->dev, "Unable to map source\n");
		return;
	}

	op->dst_phys = dma_map_single(sdcp->dev, out, AES_KEYSIZE_128,
					DMA_FROM_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
		dev_err(sdcp->dev, "Unable to map dest\n");
		goto err_unmap_src;
	}

	op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
					AES_KEYSIZE_128, DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
		dev_err(sdcp->dev, "Unable to map key\n");
		goto err_unmap_dst;
	}

	/* perform the operation */
	dcp_perform_op(op);

	dma_unmap_single(sdcp->dev, op->cipher.key_phys, AES_KEYSIZE_128,
			DMA_TO_DEVICE);
err_unmap_dst:
	dma_unmap_single(sdcp->dev, op->dst_phys, op->len, DMA_FROM_DEVICE);
err_unmap_src:
	dma_unmap_single(sdcp->dev, op->src_phys, op->len, DMA_TO_DEVICE);
}

static void dcp_aes_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128)) {
		crypto_cipher_decrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->flags = DCP_AES | DCP_DEC | DCP_ECB;
	op->len = AES_KEYSIZE_128;

	/* map the data */
	op->src_phys = dma_map_single(sdcp->dev, (void *)in, AES_KEYSIZE_128,
					DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->src_phys)) {
		dev_err(sdcp->dev, "Unable to map source\n");
		return;
	}

	op->dst_phys = dma_map_single(sdcp->dev, out, AES_KEYSIZE_128,
					DMA_FROM_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
		dev_err(sdcp->dev, "Unable to map dest\n");
		goto err_unmap_src;
	}

	op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
					AES_KEYSIZE_128, DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
		dev_err(sdcp->dev, "Unable to map key\n");
		goto err_unmap_dst;
	}

	/* perform the operation */
	dcp_perform_op(op);

	dma_unmap_single(sdcp->dev, op->cipher.key_phys, AES_KEYSIZE_128,
			DMA_TO_DEVICE);
err_unmap_dst:
	dma_unmap_single(sdcp->dev, op->dst_phys, op->len, DMA_FROM_DEVICE);
err_unmap_src:
	dma_unmap_single(sdcp->dev, op->src_phys, op->len, DMA_TO_DEVICE);
}

static int fallback_init_cip(struct crypto_tfm *tfm)
{
	const char *name = tfm->__crt_alg->cra_name;
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	op->fallback.cip = crypto_alloc_cipher(name, 0,
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(op->fallback.cip)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(op->fallback.cip);
	}

	return 0;
}

static void fallback_exit_cip(struct crypto_tfm *tfm)
{
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	crypto_free_cipher(op->fallback.cip);
	op->fallback.cip = NULL;
}

static struct crypto_alg dcp_aes_alg = {
	.cra_name		= "aes",
	.cra_driver_name	= "dcp-aes",
	.cra_priority		= 300,
	.cra_alignmask		= 15,
	.cra_flags		= CRYPTO_ALG_TYPE_CIPHER |
				  CRYPTO_ALG_NEED_FALLBACK,
	.cra_init		= fallback_init_cip,
	.cra_exit		= fallback_exit_cip,
	.cra_blocksize		= AES_KEYSIZE_128,
	.cra_ctxsize		= sizeof(struct dcp_op),
	.cra_module		= THIS_MODULE,
	.cra_list		= LIST_HEAD_INIT(dcp_aes_alg.cra_list),
	.cra_u			= {
		.cipher	= {
			.cia_min_keysize	= AES_MIN_KEY_SIZE,
			.cia_max_keysize	= AES_MAX_KEY_SIZE,
			.cia_setkey		= dcp_aes_setkey_cip,
			.cia_encrypt		= dcp_aes_encrypt,
			.cia_decrypt		= dcp_aes_decrypt
		}
	}
};

static int dcp_aes_setkey_blk(struct crypto_tfm *tfm, const u8 *key,
		unsigned int len)
{
	struct dcp_op *op = crypto_tfm_ctx(tfm);
	unsigned int ret;

	op->cipher.keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(op->cipher.key, key, len);
		return 0;
	}

	if (len != AES_KEYSIZE_192 && len != AES_KEYSIZE_256) {
		/* not supported at all */
		tfm->crt_flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	/*
	 * The requested key size is not supported by HW, do a fallback
	 */
	op->fallback.blk->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	op->fallback.blk->base.crt_flags |= (tfm->crt_flags &
						CRYPTO_TFM_REQ_MASK);

	ret = crypto_blkcipher_setkey(op->fallback.blk, key, len);
	if (ret) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (op->fallback.blk->base.crt_flags &
					CRYPTO_TFM_RES_MASK);
	}
	return ret;
}

static int fallback_blk_dec(struct blkcipher_desc *desc,
		struct scatterlist *dst, struct scatterlist *src,
		unsigned int nbytes)
{
	unsigned int ret;
	struct crypto_blkcipher *tfm;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);

	tfm = desc->tfm;
	desc->tfm = op->fallback.blk;

	ret = crypto_blkcipher_decrypt_iv(desc, dst, src, nbytes);

	desc->tfm = tfm;
	return ret;
}

static int fallback_blk_enc(struct blkcipher_desc *desc,
		struct scatterlist *dst, struct scatterlist *src,
		unsigned int nbytes)
{
	unsigned int ret;
	struct crypto_blkcipher *tfm;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);

	tfm = desc->tfm;
	desc->tfm = op->fallback.blk;

	ret = crypto_blkcipher_encrypt_iv(desc, dst, src, nbytes);

	desc->tfm = tfm;
	return ret;
}

static int fallback_init_blk(struct crypto_tfm *tfm)
{
	const char *name = tfm->__crt_alg->cra_name;
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	op->fallback.blk = crypto_alloc_blkcipher(name, 0,
			CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(op->fallback.blk)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(op->fallback.blk);
	}

	return 0;
}

static void fallback_exit_blk(struct crypto_tfm *tfm)
{
	struct dcp_op *op = crypto_tfm_ctx(tfm);

	crypto_free_blkcipher(op->fallback.blk);
	op->fallback.blk = NULL;
}

static int
dcp_aes_ecb_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err;

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128))
		return fallback_blk_dec(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);

	/* key needs to be mapped only once */
	op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
				AES_KEYSIZE_128, DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
		dev_err(sdcp->dev, "Unable to map key\n");
		return -ENOMEM;
	}

	err = blkcipher_walk_virt(desc, &walk);
	while (err == 0 && (nbytes = walk.nbytes) > 0) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->flags = DCP_AES | DCP_DEC |
				DCP_ECB;
		op->len = nbytes - (nbytes % AES_KEYSIZE_128);

		/* map the data */
		op->src_phys = dma_map_single(sdcp->dev, op->src, op->len,
						DMA_TO_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->src_phys)) {
			dev_err(sdcp->dev, "Unable to map source\n");
			err = -ENOMEM;
			break;
		}

		op->dst_phys = dma_map_single(sdcp->dev, op->dst, op->len,
						DMA_FROM_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
			dma_unmap_single(sdcp->dev, op->src_phys, op->len,
						DMA_TO_DEVICE);
			dev_err(sdcp->dev, "Unable to map dest\n");
			err = -ENOMEM;
			break;
		}

		/* perform! */
		dcp_perform_op(op);

		dma_unmap_single(sdcp->dev, op->dst_phys, op->len,
					DMA_FROM_DEVICE);
		dma_unmap_single(sdcp->dev, op->src_phys, op->len,
					DMA_TO_DEVICE);

		nbytes -= op->len;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	dma_unmap_single(sdcp->dev, op->cipher.key_phys, AES_KEYSIZE_128,
				DMA_TO_DEVICE);

	return err;
}

static int
dcp_aes_ecb_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret;

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128))
		return fallback_blk_enc(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);

	/* key needs to be mapped only once */
	op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
				AES_KEYSIZE_128, DMA_TO_DEVICE);
	if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
		dev_err(sdcp->dev, "Unable to map key\n");
		return -ENOMEM;
	}

	err = blkcipher_walk_virt(desc, &walk);

	err = 0;
	while (err == 0 && (nbytes = walk.nbytes) > 0) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->flags = DCP_AES | DCP_ENC |
			    DCP_ECB;
		op->len = nbytes - (nbytes % AES_KEYSIZE_128);

		/* map the data */
		op->src_phys = dma_map_single(sdcp->dev, op->src, op->len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->src_phys)) {
			dev_err(sdcp->dev, "Unable to map source\n");
			err = -ENOMEM;
			break;
		}

		op->dst_phys = dma_map_single(sdcp->dev, op->dst, op->len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
			dma_unmap_single(sdcp->dev, op->src_phys, op->len,
					DMA_TO_DEVICE);
			dev_err(sdcp->dev, "Unable to map dest\n");
			err = -ENOMEM;
			break;
		}

		/* perform! */
		dcp_perform_op(op);

		dma_unmap_single(sdcp->dev, op->dst_phys, op->len,
				DMA_FROM_DEVICE);
		dma_unmap_single(sdcp->dev, op->src_phys, op->len,
				DMA_TO_DEVICE);

		nbytes -= op->len;
		ret =  blkcipher_walk_done(desc, &walk, nbytes);
	}

	dma_unmap_single(sdcp->dev, op->cipher.key_phys, AES_KEYSIZE_128,
			DMA_TO_DEVICE);

	return err;
}


static struct crypto_alg dcp_aes_ecb_alg = {
	.cra_name		= "ecb(aes)",
	.cra_driver_name	= "dcp-ecb-aes",
	.cra_priority		= 400,
	.cra_alignmask		= 15,
	.cra_flags		= CRYPTO_ALG_TYPE_BLKCIPHER |
				  CRYPTO_ALG_NEED_FALLBACK,
	.cra_init		= fallback_init_blk,
	.cra_exit		= fallback_exit_blk,
	.cra_blocksize		= AES_KEYSIZE_128,
	.cra_ctxsize		= sizeof(struct dcp_op),
	.cra_type		= &crypto_blkcipher_type,
	.cra_module		= THIS_MODULE,
	.cra_list		= LIST_HEAD_INIT(dcp_aes_ecb_alg.cra_list),
	.cra_u			= {
		.blkcipher 	= {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= dcp_aes_setkey_blk,
			.encrypt	= dcp_aes_ecb_encrypt,
			.decrypt	= dcp_aes_ecb_decrypt
		}
	}
};

static int
dcp_aes_cbc_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, blockno;

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128))
		return fallback_blk_dec(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);

	blockno = 0;
	err = blkcipher_walk_virt(desc, &walk);
	while (err == 0 && (nbytes = walk.nbytes) > 0) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->flags = DCP_AES | DCP_DEC |
			    DCP_CBC;
		if (blockno == 0) {
			op->flags |= DCP_CBC_INIT;
			memcpy(op->cipher.key + AES_KEYSIZE_128, walk.iv,
				AES_KEYSIZE_128);
		}
		op->len = nbytes - (nbytes % AES_KEYSIZE_128);

		/* key (+iv) needs to be mapped only once */
		op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
			dev_err(sdcp->dev, "Unable to map key\n");
			err = -ENOMEM;
			break;
		}

		/* map the data */
		op->src_phys = dma_map_single(sdcp->dev, op->src, op->len,
					DMA_TO_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->src_phys)) {
			dma_unmap_single(sdcp->dev, op->cipher.key_phys,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
			dev_err(sdcp->dev, "Unable to map source\n");
			err = -ENOMEM;
			break;
		}

		op->dst_phys = dma_map_single(sdcp->dev, op->dst, op->len,
						DMA_FROM_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
			dma_unmap_single(sdcp->dev, op->cipher.key_phys,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
			dma_unmap_single(sdcp->dev, op->src_phys, op->len,
					DMA_TO_DEVICE);
			dev_err(sdcp->dev, "Unable to map dest\n");
			err = -ENOMEM;
			break;
		}

		/* perform! */
		dcp_perform_op(op);

		dma_unmap_single(sdcp->dev, op->cipher.key_phys,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
		dma_unmap_single(sdcp->dev, op->dst_phys, op->len,
					DMA_FROM_DEVICE);
		dma_unmap_single(sdcp->dev, op->src_phys, op->len,
					DMA_TO_DEVICE);

		nbytes -= op->len;
		err = blkcipher_walk_done(desc, &walk, nbytes);

		blockno++;
	}

	return err;
}

static int
dcp_aes_cbc_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret, blockno;

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128))
		return fallback_blk_enc(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);

	blockno = 0;

	err = blkcipher_walk_virt(desc, &walk);
	while (err == 0 && (nbytes = walk.nbytes) > 0) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->flags = DCP_AES | DCP_ENC |
			    DCP_CBC;
		if (blockno == 0) {
			op->flags |= DCP_CBC_INIT;
			memcpy(op->cipher.key + AES_KEYSIZE_128, walk.iv,
				AES_KEYSIZE_128);
		}
		op->len = nbytes - (nbytes % AES_KEYSIZE_128);

		/* key needs to be mapped only once */
		op->cipher.key_phys = dma_map_single(sdcp->dev, op->cipher.key,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(sdcp->dev, op->cipher.key_phys)) {
			dev_err(sdcp->dev, "Unable to map key\n");
			return -ENOMEM;
		}

		/* map the data */
		op->src_phys = dma_map_single(sdcp->dev, op->src, op->len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->src_phys)) {
			dma_unmap_single(sdcp->dev, op->cipher.key_phys,
				AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
			dev_err(sdcp->dev, "Unable to map source\n");
			err = -ENOMEM;
			break;
		}

		op->dst_phys = dma_map_single(sdcp->dev, op->dst, op->len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(sdcp->dev, op->dst_phys)) {
			dma_unmap_single(sdcp->dev, op->cipher.key_phys,
					AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
			dma_unmap_single(sdcp->dev, op->src_phys, op->len,
					DMA_TO_DEVICE);
			dev_err(sdcp->dev, "Unable to map dest\n");
			err = -ENOMEM;
			break;
		}

		/* perform! */
		dcp_perform_op(op);

		dma_unmap_single(sdcp->dev, op->cipher.key_phys,
				AES_KEYSIZE_128 * 2, DMA_BIDIRECTIONAL);
		dma_unmap_single(sdcp->dev, op->dst_phys, op->len,
				DMA_FROM_DEVICE);
		dma_unmap_single(sdcp->dev, op->src_phys, op->len,
				DMA_TO_DEVICE);

		nbytes -= op->len;
		ret =  blkcipher_walk_done(desc, &walk, nbytes);

		blockno++;
	}

	return err;
}

static struct crypto_alg dcp_aes_cbc_alg = {
	.cra_name		= "cbc(aes)",
	.cra_driver_name	= "dcp-cbc-aes",
	.cra_priority		= 400,
	.cra_alignmask		= 15,
	.cra_flags		= CRYPTO_ALG_TYPE_BLKCIPHER |
				  CRYPTO_ALG_NEED_FALLBACK,
	.cra_init		= fallback_init_blk,
	.cra_exit		= fallback_exit_blk,
	.cra_blocksize		= AES_KEYSIZE_128,
	.cra_ctxsize		= sizeof(struct dcp_op),
	.cra_type		= &crypto_blkcipher_type,
	.cra_module		= THIS_MODULE,
	.cra_list		= LIST_HEAD_INIT(dcp_aes_cbc_alg.cra_list),
	.cra_u			= {
		.blkcipher 	= {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= dcp_aes_setkey_blk,
			.encrypt	= dcp_aes_cbc_encrypt,
			.decrypt	= dcp_aes_cbc_decrypt,
			.ivsize		= AES_KEYSIZE_128,
		}
	}
};

static int dcp_perform_hash_op(
	struct dcp_hash_coherent_block *input,
	u32 num_desc, bool init, bool terminate)
{
	struct dcp *sdcp = global_sdcp;
	int chan;
	struct dcp_hw_packet *pkt;
	struct dcp_hash_coherent_block *hw;
	unsigned long timeout;
	u32 stat;
	int descno, mapped;

	chan = HASH_CHAN;

	hw = input;
	pkt = hw->pkt;

	for (descno = 0; descno < num_desc; descno++) {

		if (descno != 0) {

			/* set next ptr and CHAIN bit in last packet */
			pkt->pNext = hw->next->my_phys + offsetof(
				struct dcp_hash_coherent_block,
				pkt[0]);
			pkt->pkt1 |= BM_DCP_PACKET1_CHAIN;

			/* iterate to next descriptor */
			hw = hw->next;
			pkt = hw->pkt;
		}

		pkt->pkt1 = BM_DCP_PACKET1_DECR_SEMAPHORE |
					BM_DCP_PACKET1_ENABLE_HASH;

		if (init && descno == 0)
			pkt->pkt1 |= BM_DCP_PACKET1_HASH_INIT;

		pkt->pkt2 = BF(hw->hash_sel,
				DCP_PACKET2_HASH_SELECT);

		/* no need to flush buf1 or buf2, which are uncached */
		if (hw->src != sdcp->buf1 && hw->src != sdcp->buf2) {

			/* we have to flush the cache for the buffer */
			hw->src_phys = dma_map_single(sdcp->dev,
				hw->src, hw->len, DMA_TO_DEVICE);

			if (dma_mapping_error(sdcp->dev, hw->src_phys)) {
				dev_err(sdcp->dev, "Unable to map source\n");

				/* unmap any previous mapped buffers */
				for (mapped = 0, hw = input; mapped < descno;
					mapped++) {

					if (mapped != 0)
						hw = hw->next;
					if (hw->src != sdcp->buf1 &&
						hw->src != sdcp->buf2)
						dma_unmap_single(sdcp->dev,
							hw->src_phys, hw->len,
							DMA_TO_DEVICE);
				}

				return -EFAULT;
			}
		}

		pkt->pSrc = (u32)hw->src_phys;
		pkt->pDst = 0;
		pkt->size = hw->len;
		pkt->pPayload = 0;
		pkt->stat = 0;

		/* set HASH_TERM bit on last buf if terminate was set */
		if (terminate && (descno == (num_desc - 1))) {
			pkt->pkt1 |= BM_DCP_PACKET1_HASH_TERM;

			memset(input->digest, 0, sizeof(input->digest));

			/* set payload ptr to the 1st buffer's digest */
			pkt->pPayload = (u32)input->my_phys +
				offsetof(
				struct dcp_hash_coherent_block,
				digest);
		}
	}

	/* submit the work */

	__raw_writel(-1, sdcp->dcp_regs_base + HW_DCP_CHnSTAT_CLR(chan));

	mb();
	/* Load the 1st descriptor's physical address */
	__raw_writel((u32)input->my_phys +
		offsetof(struct dcp_hash_coherent_block,
		pkt[0]), sdcp->dcp_regs_base + HW_DCP_CHnCMDPTR(chan));

	/* XXX wake from interrupt instead of looping */
	timeout = jiffies + msecs_to_jiffies(1000);

	/* write num_desc into sema register */
	__raw_writel(BF(num_desc, DCP_CHnSEMA_INCREMENT),
		sdcp->dcp_regs_base + HW_DCP_CHnSEMA(chan));

	while (time_before(jiffies, timeout) &&
		((__raw_readl(sdcp->dcp_regs_base +
		HW_DCP_CHnSEMA(chan)) >> 16) & 0xff) != 0) {

		cpu_relax();
	}

	if (!time_before(jiffies, timeout)) {
		dev_err(sdcp->dev,
			"Timeout while waiting STAT 0x%08x\n",
			__raw_readl(sdcp->dcp_regs_base + HW_DCP_STAT));
	}

	stat = __raw_readl(sdcp->dcp_regs_base + HW_DCP_CHnSTAT(chan));
	if ((stat & 0xff) != 0)
		dev_err(sdcp->dev, "Channel stat error 0x%02x\n",
				__raw_readl(sdcp->dcp_regs_base +
				HW_DCP_CHnSTAT(chan)) & 0xff);

	/* unmap all src buffers */
	for (descno = 0, hw = input; descno < num_desc; descno++) {
		if (descno != 0)
			hw = hw->next;
		if (hw->src != sdcp->buf1 && hw->src != sdcp->buf2)
			dma_unmap_single(sdcp->dev, hw->src_phys, hw->len,
				DMA_TO_DEVICE);
	}

	return 0;

}

static int dcp_sha_init(struct shash_desc *desc)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_hash_op *op = shash_desc_ctx(desc);
	struct mutex *mutex = &sdcp->op_mutex[HASH_CHAN];

	mutex_lock(mutex);
	dcp_clock(sdcp, CLOCK_ON, false);
	sdcp->chan_in_use[HASH_CHAN] = true;

	op->length = 0;

	/* reset the lengths and the pointers of buffer descriptors */
	sdcp->buf1_desc->len = 0;
	sdcp->buf1_desc->src = sdcp->buf1;
	sdcp->buf2_desc->len = 0;
	sdcp->buf2_desc->src = sdcp->buf2;
	op->head_desc = sdcp->buf1_desc;
	op->tail_desc = sdcp->buf2_desc;

	return 0;
}

static int dcp_sha_update(struct shash_desc *desc, const u8 *data,
		      unsigned int length)
{
	struct dcp *sdcp = global_sdcp;
	struct dcp_hash_op *op = shash_desc_ctx(desc);
	struct dcp_hash_coherent_block *temp;
	u32 rem_bytes, bytes_borrowed, hash_sel;
	int ret = 0;

	if (strcmp(desc->tfm->base.__crt_alg->cra_name, "sha1") == 0)
		hash_sel = BV_DCP_PACKET2_HASH_SELECT__SHA1;
	else
		hash_sel = BV_DCP_PACKET2_HASH_SELECT__SHA256;

	sdcp->user_buf_desc->src = (void *)data;
	sdcp->user_buf_desc->len = length;

	op->tail_desc->len = 0;

	/* check if any pending data from previous updates */
	if (op->head_desc->len) {

			/* borrow from this buffer to make it 64 bytes */
			bytes_borrowed = min(64 - op->head_desc->len,
					sdcp->user_buf_desc->len);

			/* copy n bytes to head */
			memcpy(op->head_desc->src + op->head_desc->len,
				sdcp->user_buf_desc->src, bytes_borrowed);
			op->head_desc->len += bytes_borrowed;

			/* update current buffer's src and len */
			sdcp->user_buf_desc->src += bytes_borrowed;
			sdcp->user_buf_desc->len -= bytes_borrowed;
	}

	/* Is current buffer unaligned to 64 byte length?
	  * Each buffer's length must be a multiple of 64 bytes for DCP
	  */
	rem_bytes = sdcp->user_buf_desc->len % 64;

	/* if length is unaligned, copy remainder to tail */
	if (rem_bytes) {

		memcpy(op->tail_desc->src, (sdcp->user_buf_desc->src +
			sdcp->user_buf_desc->len - rem_bytes),
			rem_bytes);

		/* update length of current buffer */
		sdcp->user_buf_desc->len -= rem_bytes;

		op->tail_desc->len = rem_bytes;
	}

	/* do not send to DCP if length is < 64 */
	if ((op->head_desc->len + sdcp->user_buf_desc->len) >= 64) {

		/* set hash alg to be used (SHA1 or SHA256) */
		op->head_desc->hash_sel = hash_sel;
		sdcp->user_buf_desc->hash_sel = hash_sel;

		if (op->head_desc->len) {
			op->head_desc->next = sdcp->user_buf_desc;

			ret = dcp_perform_hash_op(op->head_desc,
				sdcp->user_buf_desc->len ? 2 : 1,
				op->length == 0, false);
		} else {
			ret = dcp_perform_hash_op(sdcp->user_buf_desc, 1,
				op->length == 0, false);
		}

		op->length += op->head_desc->len + sdcp->user_buf_desc->len;
		op->head_desc->len = 0;
	}

	/* if tail has bytes, make it the head for next time */
	if (op->tail_desc->len) {
		temp = op->head_desc;
		op->head_desc = op->tail_desc;
		op->tail_desc = temp;
	}

	/* hash_sel to be used by final function */
	op->head_desc->hash_sel = hash_sel;

	return ret;
}

static int dcp_sha_final(struct shash_desc *desc, u8 *out)
{
	struct dcp_hash_op *op = shash_desc_ctx(desc);
	const uint8_t *digest;
	struct dcp *sdcp = global_sdcp;
	u32 i, digest_len;
	struct mutex *mutex = &sdcp->op_mutex[HASH_CHAN];
	int ret = 0;

	/* Send the leftover bytes in head, which can be length 0,
	  * but DCP still produces hash result in payload ptr.
	  * Last data bytes need not be 64-byte multiple.
	  */
	ret = dcp_perform_hash_op(op->head_desc, 1, op->length == 0, true);

	op->length += op->head_desc->len;

	digest_len = (op->head_desc->hash_sel ==
		BV_DCP_PACKET2_HASH_SELECT__SHA1) ? SHA1_DIGEST_SIZE :
		SHA256_DIGEST_SIZE;

	/* hardware reverses the digest (for some unexplicable reason) */
	digest = op->head_desc->digest + digest_len;
	for (i = 0; i < digest_len; i++)
		*out++ = *--digest;

	sdcp->chan_in_use[HASH_CHAN] = false;
	dcp_clock(sdcp, CLOCK_OFF, false);
	mutex_unlock(mutex);

	return ret;
}

static struct shash_alg dcp_sha1_alg = {
	.init			=	dcp_sha_init,
	.update			=	dcp_sha_update,
	.final			=	dcp_sha_final,
	.descsize		=	sizeof(struct dcp_hash_op),
	.digestsize		=	SHA1_DIGEST_SIZE,
	.base			=	{
		.cra_name		=	"sha1",
		.cra_driver_name	=	"sha1-dcp",
		.cra_priority		=	300,
		.cra_blocksize		=	SHA1_BLOCK_SIZE,
		.cra_ctxsize		=
			sizeof(struct dcp_hash_op),
		.cra_module		=	THIS_MODULE,
	}
};

static struct shash_alg dcp_sha256_alg = {
	.init			=	dcp_sha_init,
	.update			=	dcp_sha_update,
	.final			=	dcp_sha_final,
	.descsize		=	sizeof(struct dcp_hash_op),
	.digestsize		=	SHA256_DIGEST_SIZE,
	.base			=	{
		.cra_name		=	"sha256",
		.cra_driver_name	=	"sha256-dcp",
		.cra_priority		=	300,
		.cra_blocksize		=	SHA256_BLOCK_SIZE,
		.cra_ctxsize		=
			sizeof(struct dcp_hash_op),
		.cra_module		=	THIS_MODULE,
	}
};

static irqreturn_t dcp_common_irq(int irq, void *context)
{
	struct dcp *sdcp = context;
	u32 msk;

	/* check */
	msk = __raw_readl(sdcp->dcp_regs_base + HW_DCP_STAT) &
		BF(0x0f, DCP_STAT_IRQ);
	if (msk == 0)
		return IRQ_NONE;

	/* clear this channel */
	__raw_writel(msk, sdcp->dcp_regs_base + HW_DCP_STAT_CLR);
	if (msk & BF(0x01, DCP_STAT_IRQ))
		sdcp->wait[0]++;
	if (msk & BF(0x02, DCP_STAT_IRQ))
		sdcp->wait[1]++;
	if (msk & BF(0x04, DCP_STAT_IRQ))
		sdcp->wait[2]++;
	if (msk & BF(0x08, DCP_STAT_IRQ))
		sdcp->wait[3]++;
	return IRQ_HANDLED;
}

static irqreturn_t dcp_vmi_irq(int irq, void *context)
{
	return dcp_common_irq(irq, context);
}

static irqreturn_t dcp_irq(int irq, void *context)
{
	return dcp_common_irq(irq, context);
}

/* DCP bootstream verification interface: uses OTP key for crypto */
static long dcp_bootstream_ioctl(struct file *file,
					 unsigned int cmd, unsigned long arg)
{
	struct dcp *sdcp = global_sdcp;
	struct dcpboot_dma_area *da = sdcp->dcpboot_dma_area;
	void __user *argp = (void __user *)arg;
	int chan = ROM_DCP_CHAN;
	unsigned long timeout;
	struct mutex *mutex;
	int retVal;

	/* be paranoid */
	if (sdcp == NULL)
		return -EBADF;

	if (cmd != DBS_ENC && cmd != DBS_DEC)
		return -EINVAL;

	/* copy to (aligned) block */
	if (copy_from_user(da->block, argp, 16))
		return -EFAULT;

	mutex = &sdcp->op_mutex[chan];
	mutex_lock(mutex);
	dcp_clock(sdcp, CLOCK_ON, false);
	sdcp->chan_in_use[chan] = true;

	__raw_writel(-1, sdcp->dcp_regs_base +
		HW_DCP_CHnSTAT_CLR(ROM_DCP_CHAN));
	__raw_writel(BF(ROM_DCP_CHAN_MASK, DCP_STAT_IRQ),
		sdcp->dcp_regs_base + HW_DCP_STAT_CLR);

	da->hw_packet.pNext = 0;
	da->hw_packet.pkt1 = BM_DCP_PACKET1_DECR_SEMAPHORE |
	    BM_DCP_PACKET1_ENABLE_CIPHER | BM_DCP_PACKET1_OTP_KEY |
	    BM_DCP_PACKET1_INTERRUPT |
	    (cmd == DBS_ENC ? BM_DCP_PACKET1_CIPHER_ENCRYPT : 0);
	da->hw_packet.pkt2 = BF(0, DCP_PACKET2_CIPHER_CFG) |
	    BF(0, DCP_PACKET2_KEY_SELECT) |
	    BF(BV_DCP_PACKET2_CIPHER_MODE__ECB, DCP_PACKET2_CIPHER_MODE) |
	    BF(BV_DCP_PACKET2_CIPHER_SELECT__AES128, DCP_PACKET2_CIPHER_SELECT);
	da->hw_packet.pSrc = sdcp->dcpboot_dma_area_phys +
	    offsetof(struct dcpboot_dma_area, block);
	da->hw_packet.pDst = da->hw_packet.pSrc;	/* in-place */
	da->hw_packet.size = 16;
	da->hw_packet.pPayload = 0;
	da->hw_packet.stat = 0;

	/* Load the work packet pointer and bump the channel semaphore */
	__raw_writel(sdcp->dcpboot_dma_area_phys +
		     offsetof(struct dcpboot_dma_area, hw_packet),
		     sdcp->dcp_regs_base + HW_DCP_CHnCMDPTR(ROM_DCP_CHAN));

	sdcp->wait[chan] = 0;
	__raw_writel(BF(1, DCP_CHnSEMA_INCREMENT),
		     sdcp->dcp_regs_base + HW_DCP_CHnSEMA(ROM_DCP_CHAN));

	timeout = jiffies + msecs_to_jiffies(100);

	while (time_before(jiffies, timeout) && sdcp->wait[chan] == 0)
		cpu_relax();

	if (!time_before(jiffies, timeout)) {
		dev_err(sdcp->dev,
			"Timeout while waiting for operation to complete\n");
		retVal = -ETIMEDOUT;
		goto exit;
	}

	if ((__raw_readl(sdcp->dcp_regs_base + HW_DCP_CHnSTAT(ROM_DCP_CHAN))
			& 0xff) != 0) {
		dev_err(sdcp->dev, "Channel stat error 0x%02x\n",
			__raw_readl(sdcp->dcp_regs_base +
				    HW_DCP_CHnSTAT(ROM_DCP_CHAN)) & 0xff);
		retVal = -EFAULT;
		goto exit;
	}

	if (copy_to_user(argp, da->block, 16)) {
		retVal =  -EFAULT;
		goto exit;
	}

	retVal = 0;

exit:
	sdcp->chan_in_use[chan] = false;
	dcp_clock(sdcp, CLOCK_OFF, false);
	mutex_unlock(mutex);
	return retVal;
}

static const struct file_operations dcp_bootstream_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl =	dcp_bootstream_ioctl,
};

static struct miscdevice dcp_bootstream_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "dcpboot",
	.fops = &dcp_bootstream_fops,
};

static int dcp_probe(struct platform_device *pdev)
{
	struct dcp *sdcp = NULL;
	struct resource *r;
	int i, ret;
	dma_addr_t hw_phys;

	if (global_sdcp != NULL) {
		dev_err(&pdev->dev, "Only one instance allowed\n");
		ret = -ENODEV;
		goto err;
	}

	/* allocate memory */
	sdcp = kzalloc(sizeof(*sdcp), GFP_KERNEL);
	if (sdcp == NULL) {
		dev_err(&pdev->dev, "Failed to allocate structure\n");
		ret = -ENOMEM;
		goto err;
	}

	sdcp->dev = &pdev->dev;
	spin_lock_init(&sdcp->lock);

	for (i = 0; i < DCP_NUM_CHANNELS; i++) {
		mutex_init(&sdcp->op_mutex[i]);
		init_completion(&sdcp->op_wait[i]);
		sdcp->chan_in_use[i] = false;
	}

	platform_set_drvdata(pdev, sdcp);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get IORESOURCE_MEM\n");
		ret = -ENXIO;
		goto err_kfree;
	}
	sdcp->dcp_regs_base = (u32) ioremap(r->start, r->end - r->start + 1);
	dcp_clock(sdcp, CLOCK_ON, true);

	/* Soft reset and remove the clock gate */
	__raw_writel(BM_DCP_CTRL_SFTRST, sdcp->dcp_regs_base + HW_DCP_CTRL_SET);

	/* At 24Mhz, it takes no more than 4 clocks (160 ns) Maximum for
	 * the part to reset, reading the register twice should
	 * be sufficient to get 4 clks delay.
	 */
	__raw_readl(sdcp->dcp_regs_base + HW_DCP_CTRL);
	__raw_readl(sdcp->dcp_regs_base + HW_DCP_CTRL);

	__raw_writel(BM_DCP_CTRL_SFTRST | BM_DCP_CTRL_CLKGATE,
		sdcp->dcp_regs_base + HW_DCP_CTRL_CLR);

	/* Initialize control registers */
	__raw_writel(DCP_CTRL_INIT, sdcp->dcp_regs_base + HW_DCP_CTRL);
	__raw_writel(DCP_CHANNELCTRL_INIT, sdcp->dcp_regs_base +
		HW_DCP_CHANNELCTRL);

	/* We do not enable context switching. Give the context
	 * buffer pointer an illegal address so if context switching is
	 * inadvertantly enabled, the dcp will return an error instead of
	 * trashing good memory. The dcp dma cannot access rom, so any rom
	 * address will do.
	 */
	__raw_writel(0xFFFF0000, sdcp->dcp_regs_base + HW_DCP_CONTEXT);

	for (i = 0; i < DCP_NUM_CHANNELS; i++)
		__raw_writel(-1, sdcp->dcp_regs_base + HW_DCP_CHnSTAT_CLR(i));
	__raw_writel(-1, sdcp->dcp_regs_base + HW_DCP_STAT_CLR);

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get IRQ resource (0)\n");
		ret = -EIO;
		goto err_gate_clk;
	}
	sdcp->dcp_vmi_irq = r->start;
	ret = request_irq(sdcp->dcp_vmi_irq, dcp_vmi_irq, 0, "dcp",
				sdcp);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't request_irq (0)\n");
		goto err_gate_clk;
	}

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!r) {
		dev_err(&pdev->dev, "can't get IRQ resource (1)\n");
		ret = -EIO;
		goto err_free_irq0;
	}
	sdcp->dcp_irq = r->start;
	ret = request_irq(sdcp->dcp_irq, dcp_irq, 0, "dcp", sdcp);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't request_irq (1)\n");
		goto err_free_irq0;
	}

	global_sdcp = sdcp;

	ret = crypto_register_alg(&dcp_aes_alg);
	if (ret != 0)  {
		dev_err(&pdev->dev, "Failed to register aes crypto\n");
		goto err_free_irq1;
	}

	ret = crypto_register_alg(&dcp_aes_ecb_alg);
	if (ret != 0)  {
		dev_err(&pdev->dev, "Failed to register aes ecb crypto\n");
		goto err_unregister_aes;
	}

	ret = crypto_register_alg(&dcp_aes_cbc_alg);
	if (ret != 0)  {
		dev_err(&pdev->dev, "Failed to register aes cbc crypto\n");
		goto err_unregister_aes_ecb;
	}

	/* Allocate the descriptor to be used for user buffer
	  * passed in by the "update" function from Crypto API
	  */
	sdcp->user_buf_desc = dma_alloc_coherent(sdcp->dev,
		sizeof(struct dcp_hash_coherent_block),  &hw_phys,
		GFP_KERNEL);
	if (sdcp->user_buf_desc == NULL) {
		printk(KERN_ERR "Error allocating coherent block\n");
		ret = -ENOMEM;
		goto err_unregister_aes_cbc;
	}

	sdcp->user_buf_desc->my_phys = hw_phys;

	/* Allocate 2 buffers (head & tail) & its descriptors to deal with
	  * buffer lengths that are not 64 byte aligned, except for the
	  * last one.
	  */
	sdcp->buf1 = dma_alloc_coherent(sdcp->dev,
		64, &sdcp->buf1_phys, GFP_KERNEL);
	if (sdcp->buf1 == NULL) {
		printk(KERN_ERR "Error allocating coherent block\n");
		ret = -ENOMEM;
		goto err_unregister_aes_cbc;
	}

	sdcp->buf2 = dma_alloc_coherent(sdcp->dev,
		64,  &sdcp->buf2_phys, GFP_KERNEL);
	if (sdcp->buf2 == NULL) {
		printk(KERN_ERR "Error allocating coherent block\n");
		ret = -ENOMEM;
		goto err_unregister_aes_cbc;
	}

	sdcp->buf1_desc = dma_alloc_coherent(sdcp->dev,
		sizeof(struct dcp_hash_coherent_block), &hw_phys,
		GFP_KERNEL);
	if (sdcp->buf1_desc == NULL) {
		printk(KERN_ERR "Error allocating coherent block\n");
		ret = -ENOMEM;
		goto err_unregister_aes_cbc;
	}

	sdcp->buf1_desc->my_phys = hw_phys;
	sdcp->buf1_desc->src = (void *)sdcp->buf1;
	sdcp->buf1_desc->src_phys = sdcp->buf1_phys;

	sdcp->buf2_desc = dma_alloc_coherent(sdcp->dev,
		sizeof(struct dcp_hash_coherent_block), &hw_phys,
		GFP_KERNEL);
	if (sdcp->buf2_desc == NULL) {
		printk(KERN_ERR "Error allocating coherent block\n");
		ret = -ENOMEM;
		goto err_unregister_aes_cbc;
	}

	sdcp->buf2_desc->my_phys = hw_phys;
	sdcp->buf2_desc->src = (void *)sdcp->buf2;
	sdcp->buf2_desc->src_phys = sdcp->buf2_phys;


	ret = crypto_register_shash(&dcp_sha1_alg);
	if (ret != 0)  {
		dev_err(&pdev->dev, "Failed to register sha1 hash\n");
		goto err_unregister_aes_cbc;
	}

	if (__raw_readl(sdcp->dcp_regs_base + HW_DCP_CAPABILITY1) &
		BF_DCP_CAPABILITY1_HASH_ALGORITHMS(
		BV_DCP_CAPABILITY1_HASH_ALGORITHMS__SHA256)) {

		ret = crypto_register_shash(&dcp_sha256_alg);
		if (ret != 0)  {
			dev_err(&pdev->dev, "Failed to register sha256 hash\n");
			goto err_unregister_sha1;
		}
	}

	/* register dcpboot interface to allow apps (such as kobs-ng) to
	 * verify files (such as the bootstream) using the OTP key for crypto */
	ret = misc_register(&dcp_bootstream_misc);
	if (ret != 0) {
		dev_err(&pdev->dev, "Unable to register misc device\n");
		goto err_unregister_sha1;
	}

	sdcp->dcpboot_dma_area = dma_alloc_coherent(&pdev->dev,
		sizeof(*sdcp->dcpboot_dma_area), &sdcp->dcpboot_dma_area_phys,
		GFP_KERNEL);
	if (sdcp->dcpboot_dma_area == NULL) {
		dev_err(&pdev->dev,
			"Unable to allocate DMAable memory \
			 for dcpboot interface\n");
		goto err_dereg;
	}

	dcp_clock(sdcp, CLOCK_OFF, false);
	dev_notice(&pdev->dev, "DCP crypto enabled.!\n");
	return 0;

err_dereg:
	misc_deregister(&dcp_bootstream_misc);
err_unregister_sha1:
	crypto_unregister_shash(&dcp_sha1_alg);
err_unregister_aes_cbc:
	crypto_unregister_alg(&dcp_aes_cbc_alg);
err_unregister_aes_ecb:
	crypto_unregister_alg(&dcp_aes_ecb_alg);
err_unregister_aes:
	crypto_unregister_alg(&dcp_aes_alg);
err_free_irq1:
	free_irq(sdcp->dcp_irq, sdcp);
err_free_irq0:
	free_irq(sdcp->dcp_vmi_irq, sdcp);
err_gate_clk:
	dcp_clock(sdcp, CLOCK_OFF, false);
err_kfree:
	kfree(sdcp);
err:

	return ret;
}

static int dcp_remove(struct platform_device *pdev)
{
	struct dcp *sdcp;

	sdcp = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	dcp_clock(sdcp, CLOCK_ON, false);

	free_irq(sdcp->dcp_irq, sdcp);
	free_irq(sdcp->dcp_vmi_irq, sdcp);

	/* if head and tail buffers were allocated, free them */
	if (sdcp->buf1) {
		dma_free_coherent(sdcp->dev, 64, sdcp->buf1, sdcp->buf1_phys);
		dma_free_coherent(sdcp->dev, 64, sdcp->buf2, sdcp->buf2_phys);

		dma_free_coherent(sdcp->dev,
				sizeof(struct dcp_hash_coherent_block),
				sdcp->buf1_desc, sdcp->buf1_desc->my_phys);

		dma_free_coherent(sdcp->dev,
				sizeof(struct dcp_hash_coherent_block),
				sdcp->buf2_desc, sdcp->buf2_desc->my_phys);

		dma_free_coherent(sdcp->dev,
			sizeof(struct dcp_hash_coherent_block),
			sdcp->user_buf_desc, sdcp->user_buf_desc->my_phys);
	}

	if (sdcp->dcpboot_dma_area) {
		dma_free_coherent(&pdev->dev, sizeof(*sdcp->dcpboot_dma_area),
			  sdcp->dcpboot_dma_area, sdcp->dcpboot_dma_area_phys);
		misc_deregister(&dcp_bootstream_misc);
	}


	crypto_unregister_shash(&dcp_sha1_alg);

	if (__raw_readl(sdcp->dcp_regs_base + HW_DCP_CAPABILITY1) &
		BF_DCP_CAPABILITY1_HASH_ALGORITHMS(
		BV_DCP_CAPABILITY1_HASH_ALGORITHMS__SHA256))
		crypto_unregister_shash(&dcp_sha256_alg);

	crypto_unregister_alg(&dcp_aes_cbc_alg);
	crypto_unregister_alg(&dcp_aes_ecb_alg);
	crypto_unregister_alg(&dcp_aes_alg);

	dcp_clock(sdcp, CLOCK_OFF, true);
	iounmap((void *) sdcp->dcp_regs_base);
	kfree(sdcp);
	global_sdcp = NULL;

	return 0;
}

static int dcp_suspend(struct platform_device *pdev,
		pm_message_t state)
{
#ifdef CONFIG_PM
	struct dcp *sdcp = platform_get_drvdata(pdev);

	if (sdcp->clock_state == CLOCK_ON) {
		dcp_clock(sdcp, CLOCK_OFF, true);
		/* indicate that clock needs to be turned on upon resume */
		sdcp->clock_state = CLOCK_ON;
	}
#endif
	return 0;
}

static int dcp_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	struct dcp *sdcp = platform_get_drvdata(pdev);

	/* if clock was on prior to suspend, turn it back on */
	if (sdcp->clock_state == CLOCK_ON)
		dcp_clock(sdcp, CLOCK_ON, true);
#endif
	return 0;
}

static struct platform_driver dcp_driver = {
	.probe		= dcp_probe,
	.remove		= dcp_remove,
	.suspend	= dcp_suspend,
	.resume		= dcp_resume,
	.driver		= {
		.name   = "dcp",
		.owner	= THIS_MODULE,
	},
};

static int __init
dcp_init(void)
{
	return platform_driver_register(&dcp_driver);
}

static void __exit
dcp_exit(void)
{
	platform_driver_unregister(&dcp_driver);
}

MODULE_AUTHOR("Pantelis Antoniou <pantelis@embeddedalley.com>");
MODULE_DESCRIPTION("DCP Crypto Driver");
MODULE_LICENSE("GPL");

module_init(dcp_init);
module_exit(dcp_exit);
