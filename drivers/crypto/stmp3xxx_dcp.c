/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/delay.h>

#include <asm/cacheflush.h>

#include "stmp3xxx_dcp.h"

struct stmp3xxx_dcp {
	struct device *dev;
	spinlock_t lock;
	struct mutex op_mutex[STMP3XXX_DCP_NUM_CHANNELS];
	struct completion op_wait[STMP3XXX_DCP_NUM_CHANNELS];
	int wait[STMP3XXX_DCP_NUM_CHANNELS];
	int dcp_vmi_irq;
	int dcp_irq;

	/* Following buffers used in hashing to meet 64-byte len alignment */
	char *buf1;
	char *buf2;
	dma_addr_t buf1_phys;
	dma_addr_t buf2_phys;
	struct stmp3xxx_dcp_hash_coherent_block *buf1_desc;
	struct stmp3xxx_dcp_hash_coherent_block *buf2_desc;
	struct stmp3xxx_dcp_hash_coherent_block *user_buf_desc;
};

/* cipher flags */
#define STMP3XXX_DCP_ENC	0x0001
#define STMP3XXX_DCP_DEC	0x0002
#define STMP3XXX_DCP_ECB	0x0004
#define STMP3XXX_DCP_CBC	0x0008
#define STMP3XXX_DCP_CBC_INIT	0x0010
#define STMP3XXX_DCP_OTPKEY	0x0020

/* hash flags */
#define STMP3XXX_DCP_INIT	0x0001
#define STMP3XXX_DCP_UPDATE	0x0002
#define STMP3XXX_DCP_FINAL	0x0004

#define STMP3XXX_DCP_AES	0x1000
#define STMP3XXX_DCP_SHA1	0x2000
#define STMP3XXX_DCP_CRC32	0x3000
#define STMP3XXX_DCP_COPY	0x4000
#define STMP3XXX_DCP_FILL	0x5000
#define STMP3XXX_DCP_MODE_MASK	0xf000

struct stmp3xxx_dcp_op {

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
			u8 digest[SHA1_DIGEST_SIZE]
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

	struct stmp3xxx_dcp_hw_packet pkt
		__attribute__ ((__aligned__(32)));
};

struct stmp3xxx_dcp_hash_coherent_block {
	struct stmp3xxx_dcp_hw_packet pkt[1]
		__attribute__ ((__aligned__(32)));
	u8 digest[SHA1_DIGEST_SIZE]
		__attribute__ ((__aligned__(32)));
	unsigned int len;
	dma_addr_t src_phys;
	void *src;
	void *dst;
	dma_addr_t my_phys;
	struct stmp3xxx_dcp_hash_coherent_block *next;
};

struct stmp3xxx_dcp_hash_op {

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
			u8 digest[SHA1_DIGEST_SIZE]
				__attribute__ ((__aligned__(32)));
			dma_addr_t digest_phys;
			int digestlen;
			int init;
		} hash;
	};

	u32 length;
	struct stmp3xxx_dcp_hash_coherent_block *head_desc;
	struct stmp3xxx_dcp_hash_coherent_block *tail_desc;
};

/* only one */
static struct stmp3xxx_dcp *global_sdcp;

static void dcp_perform_op(struct stmp3xxx_dcp_op *op)
{
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct mutex *mutex;
	struct stmp3xxx_dcp_hw_packet *pkt;
	int chan;
	u32 pkt1, pkt2;
	unsigned long timeout;
	dma_addr_t pkt_phys;
	u32 stat;

	pkt1 = BM_DCP_PACKET1_DECR_SEMAPHORE | BM_DCP_PACKET1_INTERRUPT;

	switch (op->flags & STMP3XXX_DCP_MODE_MASK) {

	case STMP3XXX_DCP_AES:

		chan = CIPHER_CHAN;

		/* key is at the payload */
		pkt1 |= BM_DCP_PACKET1_ENABLE_CIPHER;
		if ((op->flags & STMP3XXX_DCP_OTPKEY) == 0)
			pkt1 |= BM_DCP_PACKET1_PAYLOAD_KEY;
		if (op->flags & STMP3XXX_DCP_ENC)
			pkt1 |= BM_DCP_PACKET1_CIPHER_ENCRYPT;
		if (op->flags & STMP3XXX_DCP_CBC_INIT)
			pkt1 |= BM_DCP_PACKET1_CIPHER_INIT;

		pkt2 = BF(0, DCP_PACKET2_CIPHER_CFG) |
		       BF(0, DCP_PACKET2_KEY_SELECT) |
		       BF(BV_DCP_PACKET2_CIPHER_SELECT__AES128,
		       DCP_PACKET2_CIPHER_SELECT);

		if (op->flags & STMP3XXX_DCP_ECB)
			pkt2 |= BF(BV_DCP_PACKET2_CIPHER_MODE__ECB,
				DCP_PACKET2_CIPHER_MODE);
		else if (op->flags & STMP3XXX_DCP_CBC)
			pkt2 |= BF(BV_DCP_PACKET2_CIPHER_MODE__CBC,
				DCP_PACKET2_CIPHER_MODE);

		break;

	case STMP3XXX_DCP_SHA1:

		chan = HASH_CHAN;

		pkt1 |= BM_DCP_PACKET1_ENABLE_HASH;
		if (op->flags & STMP3XXX_DCP_INIT)
			pkt1 |= BM_DCP_PACKET1_HASH_INIT;
		if (op->flags & STMP3XXX_DCP_FINAL) {
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

	__raw_writel(-1, REGS_DCP_BASE + HW_DCP_CHnSTAT_CLR(chan));

	/* Load the work packet pointer and bump the channel semaphore */
	__raw_writel((u32)pkt_phys, REGS_DCP_BASE + HW_DCP_CHnCMDPTR(chan));

	/* XXX wake from interrupt instead of looping */
	timeout = jiffies + msecs_to_jiffies(1000);

	sdcp->wait[chan] = 0;
	__raw_writel(BF(1, DCP_CHnSEMA_INCREMENT), REGS_DCP_BASE + HW_DCP_CHnSEMA(chan));
	while (time_before(jiffies, timeout) && sdcp->wait[chan] == 0)
		cpu_relax();

	if (!time_before(jiffies, timeout)) {
		dev_err(sdcp->dev, "Timeout while waiting STAT 0x%08x\n",
				__raw_readl(REGS_DCP_BASE + HW_DCP_STAT));
		goto out;
	}

	stat = __raw_readl(REGS_DCP_BASE + HW_DCP_CHnSTAT(chan));
	if ((stat & 0xff) != 0)
		dev_err(sdcp->dev, "Channel stat error 0x%02x\n",
				__raw_readl(REGS_DCP_BASE + HW_DCP_CHnSTAT(chan)) & 0xff);
out:
	mutex_unlock(mutex);

	dma_unmap_single(sdcp->dev, pkt_phys, sizeof(*pkt), DMA_TO_DEVICE);
}

static int dcp_aes_setkey_cip(struct crypto_tfm *tfm, const u8 *key,
		unsigned int len)
{
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);
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
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128)) {
		crypto_cipher_encrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_ENC | STMP3XXX_DCP_ECB;
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
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->cipher.keylen != AES_KEYSIZE_128)) {
		crypto_cipher_decrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_DEC | STMP3XXX_DCP_ECB;
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
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

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
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

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
	.cra_ctxsize		= sizeof(struct stmp3xxx_dcp_op),
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
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);
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
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);

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
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);

	tfm = desc->tfm;
	desc->tfm = op->fallback.blk;

	ret = crypto_blkcipher_encrypt_iv(desc, dst, src, nbytes);

	desc->tfm = tfm;
	return ret;
}

static int fallback_init_blk(struct crypto_tfm *tfm)
{
	const char *name = tfm->__crt_alg->cra_name;
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

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
	struct stmp3xxx_dcp_op *op = crypto_tfm_ctx(tfm);

	crypto_free_blkcipher(op->fallback.blk);
	op->fallback.blk = NULL;
}

static int
dcp_aes_ecb_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
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
		op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_DEC |
				STMP3XXX_DCP_ECB;
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
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
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
		op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_ENC |
			    STMP3XXX_DCP_ECB;
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
	.cra_ctxsize		= sizeof(struct stmp3xxx_dcp_op),
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
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
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
		op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_DEC |
			    STMP3XXX_DCP_CBC;
		if (blockno == 0) {
			op->flags |= STMP3XXX_DCP_CBC_INIT;
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
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_op *op = crypto_blkcipher_ctx(desc->tfm);
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
		op->flags = STMP3XXX_DCP_AES | STMP3XXX_DCP_ENC |
			    STMP3XXX_DCP_CBC;
		if (blockno == 0) {
			op->flags |= STMP3XXX_DCP_CBC_INIT;
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
	.cra_ctxsize		= sizeof(struct stmp3xxx_dcp_op),
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
	struct stmp3xxx_dcp_hash_coherent_block *input,
	u32 num_desc, bool init, bool terminate)
{
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	int chan;
	struct stmp3xxx_dcp_hw_packet *pkt;
	struct stmp3xxx_dcp_hash_coherent_block *hw;
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
				struct stmp3xxx_dcp_hash_coherent_block,
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

		pkt->pkt2 = BF(BV_DCP_PACKET2_HASH_SELECT__SHA1,
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
				struct stmp3xxx_dcp_hash_coherent_block,
				digest);
		}
	}

	/* submit the work */

	__raw_writel(-1, REGS_DCP_BASE + HW_DCP_CHnSTAT_CLR(chan));

	mb();
	/* Load the 1st descriptor's physical address */
	__raw_writel((u32)input->my_phys +
		offsetof(struct stmp3xxx_dcp_hash_coherent_block,
		pkt[0]), REGS_DCP_BASE + HW_DCP_CHnCMDPTR(chan));

	/* XXX wake from interrupt instead of looping */
	timeout = jiffies + msecs_to_jiffies(1000);

	/* write num_desc into sema register */
	__raw_writel(BF(num_desc, DCP_CHnSEMA_INCREMENT),
		REGS_DCP_BASE + HW_DCP_CHnSEMA(chan));

	while (time_before(jiffies, timeout) &&
		((__raw_readl(REGS_DCP_BASE +
		HW_DCP_CHnSEMA(chan)) >> 16) & 0xff) != 0) {

		cpu_relax();
	}

	if (!time_before(jiffies, timeout)) {
		dev_err(sdcp->dev,
			"Timeout while waiting STAT 0x%08x\n",
			__raw_readl(REGS_DCP_BASE + HW_DCP_STAT));
	}

	stat = __raw_readl(REGS_DCP_BASE + HW_DCP_CHnSTAT(chan));
	if ((stat & 0xff) != 0)
		dev_err(sdcp->dev, "Channel stat error 0x%02x\n",
				__raw_readl(REGS_DCP_BASE +
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

static int dcp_sha1_init(struct shash_desc *desc)
{
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_hash_op *op = shash_desc_ctx(desc);
	struct mutex *mutex = &sdcp->op_mutex[HASH_CHAN];

	mutex_lock(mutex);

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

static int dcp_sha1_update(struct shash_desc *desc, const u8 *data,
		      unsigned int length)
{
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	struct stmp3xxx_dcp_hash_op *op = shash_desc_ctx(desc);
	struct stmp3xxx_dcp_hash_coherent_block *temp;
	u32 rem_bytes, bytes_borrowed;
	int ret = 0;

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

	return ret;
}

static int dcp_sha1_final(struct shash_desc *desc, u8 *out)
{
	struct stmp3xxx_dcp_hash_op *op = shash_desc_ctx(desc);
	const uint8_t *digest;
	struct stmp3xxx_dcp *sdcp = global_sdcp;
	u32 i;
	struct mutex *mutex = &sdcp->op_mutex[HASH_CHAN];
	int ret = 0;

	/* Send the leftover bytes in head, which can be length 0,
	  * but DCP still produces hash result in payload ptr.
	  * Last data bytes need not be 64-byte multiple.
	  */
	ret = dcp_perform_hash_op(op->head_desc, 1, op->length == 0, true);

	op->length += op->head_desc->len;

	/* hardware reverses the digest (for some unexplicable reason) */
	digest = op->head_desc->digest + SHA1_DIGEST_SIZE;
	for (i = 0; i < SHA1_DIGEST_SIZE; i++)
		*out++ = *--digest;

	mutex_unlock(mutex);

	return ret;
}

static struct shash_alg dcp_sha1_alg = {
	.init			=	dcp_sha1_init,
	.update			=	dcp_sha1_update,
	.final			=	dcp_sha1_final,
	.descsize		=	sizeof(struct stmp3xxx_dcp_hash_op),
	.digestsize		=	SHA1_DIGEST_SIZE,
	.base			=	{
		.cra_name		=	"sha1",
		.cra_driver_name	=	"sha1-dcp",
		.cra_priority		=	300,
		.cra_blocksize		=	SHA1_BLOCK_SIZE,
		.cra_ctxsize		=
			sizeof(struct stmp3xxx_dcp_hash_op),
		.cra_module		=	THIS_MODULE,
	}
};

static irqreturn_t dcp_common_irq(int irq, void *context)
{
	struct stmp3xxx_dcp *sdcp = context;
	u32 msk;

	/* check */
	msk = __raw_readl(REGS_DCP_BASE + HW_DCP_STAT) & BF(0x0f, DCP_STAT_IRQ);
	if (msk == 0)
		return IRQ_NONE;

	/* clear this channel */
	__raw_writel(msk, REGS_DCP_BASE + HW_DCP_STAT_CLR);
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

static int stmp3xxx_dcp_probe(struct platform_device *pdev)
{
	struct stmp3xxx_dcp *sdcp = NULL;
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

	for (i = 0; i < STMP3XXX_DCP_NUM_CHANNELS; i++) {
		mutex_init(&sdcp->op_mutex[i]);
		init_completion(&sdcp->op_wait[i]);
	}

	platform_set_drvdata(pdev, sdcp);

	/* Soft reset and remove the clock gate */
	__raw_writel(BM_DCP_CTRL_SFTRST, REGS_DCP_BASE + HW_DCP_CTRL_SET);

	/* At 24Mhz, it takes no more than 4 clocks (160 ns) Maximum for
	 * the part to reset, reading the register twice should
	 * be sufficient to get 4 clks delay.
	 */
	__raw_readl(REGS_DCP_BASE + HW_DCP_CTRL);
	__raw_readl(REGS_DCP_BASE + HW_DCP_CTRL);

	__raw_writel(BM_DCP_CTRL_SFTRST | BM_DCP_CTRL_CLKGATE,
		REGS_DCP_BASE + HW_DCP_CTRL_CLR);

	/* Initialize control registers */
	__raw_writel(STMP3XXX_DCP_CTRL_INIT, REGS_DCP_BASE + HW_DCP_CTRL);
	__raw_writel(STMP3XXX_DCP_CHANNELCTRL_INIT, REGS_DCP_BASE + HW_DCP_CHANNELCTRL);

	/* We do not enable context switching. Give the context
	 * buffer pointer an illegal address so if context switching is
	 * inadvertantly enabled, the dcp will return an error instead of
	 * trashing good memory. The dcp dma cannot access rom, so any rom
	 * address will do.
	 */
	__raw_writel(0xFFFF0000, REGS_DCP_BASE + HW_DCP_CONTEXT);

	for (i = 0; i < STMP3XXX_DCP_NUM_CHANNELS; i++)
		__raw_writel(-1, REGS_DCP_BASE + HW_DCP_CHnSTAT_CLR(i));
	__raw_writel(-1, REGS_DCP_BASE + HW_DCP_STAT_CLR);

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!r) {
		dev_err(&pdev->dev, "can't get IRQ resource (0)\n");
		ret = -EIO;
		goto err_kfree;
	}
	sdcp->dcp_vmi_irq = r->start;
	ret = request_irq(sdcp->dcp_vmi_irq, dcp_vmi_irq, 0, "stmp3xxx-dcp",
				sdcp);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't request_irq (0)\n");
		goto err_kfree;
	}

	r = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!r) {
		dev_err(&pdev->dev, "can't get IRQ resource (1)\n");
		ret = -EIO;
		goto err_free_irq0;
	}
	sdcp->dcp_irq = r->start;
	ret = request_irq(sdcp->dcp_irq, dcp_irq, 0, "stmp3xxx-dcp", sdcp);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't request_irq (1)\n");
		goto err_free_irq0;
	}

	global_sdcp = sdcp;

	ret = crypto_register_alg(&dcp_aes_alg);
	if (ret != 0)  {
		dev_err(&pdev->dev, "Failed to register aes crypto\n");
		goto err_kfree;
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
		sizeof(struct stmp3xxx_dcp_hash_coherent_block),  &hw_phys,
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
		sizeof(struct stmp3xxx_dcp_hash_coherent_block), &hw_phys,
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
		sizeof(struct stmp3xxx_dcp_hash_coherent_block), &hw_phys,
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

	dev_notice(&pdev->dev, "DCP crypto enabled.!\n");
	return 0;

	crypto_unregister_shash(&dcp_sha1_alg);
err_unregister_aes_cbc:
	crypto_unregister_alg(&dcp_aes_cbc_alg);
err_unregister_aes_ecb:
	crypto_unregister_alg(&dcp_aes_ecb_alg);
err_unregister_aes:
	crypto_unregister_alg(&dcp_aes_alg);
err_free_irq0:
	free_irq(sdcp->dcp_vmi_irq, sdcp);
err_kfree:
	kfree(sdcp);
err:

	return ret;
}

static int stmp3xxx_dcp_remove(struct platform_device *pdev)
{
	struct stmp3xxx_dcp *sdcp;

	sdcp = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);

	free_irq(sdcp->dcp_irq, sdcp);
	free_irq(sdcp->dcp_vmi_irq, sdcp);

	/* if head and tail buffers were allocated, free them */
	if (sdcp->buf1) {
		dma_free_coherent(sdcp->dev, 64, sdcp->buf1, sdcp->buf1_phys);
		dma_free_coherent(sdcp->dev, 64, sdcp->buf2, sdcp->buf2_phys);

		dma_free_coherent(sdcp->dev,
				sizeof(struct stmp3xxx_dcp_hash_coherent_block),
				sdcp->buf1_desc, sdcp->buf1_desc->my_phys);

		dma_free_coherent(sdcp->dev,
				sizeof(struct stmp3xxx_dcp_hash_coherent_block),
				sdcp->buf2_desc, sdcp->buf2_desc->my_phys);

		dma_free_coherent(sdcp->dev,
			sizeof(struct stmp3xxx_dcp_hash_coherent_block),
			sdcp->user_buf_desc, sdcp->user_buf_desc->my_phys);
	}

	crypto_unregister_shash(&dcp_sha1_alg);

	crypto_unregister_alg(&dcp_aes_cbc_alg);
	crypto_unregister_alg(&dcp_aes_ecb_alg);
	crypto_unregister_alg(&dcp_aes_alg);
	kfree(sdcp);
	global_sdcp = NULL;

	return 0;
}


#ifdef CONFIG_PM
static int stmp3xxx_dcp_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	return 0;
}

static int stmp3xxx_dcp_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define stmp3xxx_dcp_suspend	NULL
#define	stmp3xxx_dcp_resume	NULL
#endif

static struct platform_driver stmp3xxx_dcp_driver = {
	.probe		= stmp3xxx_dcp_probe,
	.remove		= stmp3xxx_dcp_remove,
	.suspend	= stmp3xxx_dcp_suspend,
	.resume		= stmp3xxx_dcp_resume,
	.driver		= {
		.name   = "stmp3xxx-dcp",
		.owner	= THIS_MODULE,
	},
};

static int __init
stmp3xxx_dcp_init(void)
{
	return platform_driver_register(&stmp3xxx_dcp_driver);
}

static void __exit
stmp3xxx_dcp_exit(void)
{
	platform_driver_unregister(&stmp3xxx_dcp_driver);
}

MODULE_AUTHOR("Pantelis Antoniou <pantelis@embeddedalley.com>");
MODULE_DESCRIPTION("STMP3XXX DCP Crypto Driver");
MODULE_LICENSE("GPL");

module_init(stmp3xxx_dcp_init);
module_exit(stmp3xxx_dcp_exit);
