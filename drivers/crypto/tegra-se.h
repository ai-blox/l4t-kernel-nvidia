/*
 * Driver for Tegra Security Engine
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _CRYPTO_TEGRA_SE_H
#define _CRYPTO_TEGRA_SE_H

#include <crypto/hash.h>
#include <crypto/sha.h>

#define PFX	"tegra-se: "

#define TEGRA_SE_CRA_PRIORITY	300
#define TEGRA_SE_COMPOSITE_PRIORITY 400
#define TEGRA_SE_CRYPTO_QUEUE_LENGTH 50
#define SE_MAX_SRC_SG_COUNT		50
#define SE_MAX_DST_SG_COUNT		50

#define TEGRA_SE_KEYSLOT_COUNT		16

/* SE register definitions */
#define SE_CONFIG_REG_OFFSET		0x014
#define SE_CONFIG_ENC_ALG_SHIFT		12
#define SE_CONFIG_DEC_ALG_SHIFT		8
#define ALG_AES_ENC		1
#define ALG_RNG			2
#define ALG_SHA			3
#define ALG_RSA			4
#define ALG_NOP			0
#define ALG_AES_DEC		1
#define SE_CONFIG_ENC_ALG(x)		(x << SE_CONFIG_ENC_ALG_SHIFT)
#define SE_CONFIG_DEC_ALG(x)		(x << SE_CONFIG_DEC_ALG_SHIFT)
#define SE_CONFIG_DST_SHIFT			2
#define DST_MEMORY		0
#define DST_HASHREG		1
#define DST_KEYTAB		2
#define DST_SRK			3
#define DST_RSAREG		4
#define SE_CONFIG_DST(x)			(x << SE_CONFIG_DST_SHIFT)
#define SE_CONFIG_ENC_MODE_SHIFT	24
#define SE_CONFIG_DEC_MODE_SHIFT	16
#define MODE_KEY128		0
#define MODE_KEY192		1
#define MODE_KEY256		2
#define MODE_SHA1		0
#define MODE_SHA224		4
#define MODE_SHA256		5
#define MODE_SHA384		6
#define MODE_SHA512		7
#define SE_CONFIG_ENC_MODE(x)		(x << SE_CONFIG_ENC_MODE_SHIFT)
#define SE_CONFIG_DEC_MODE(x)		(x << SE_CONFIG_DEC_MODE_SHIFT)

#define SE_KEYTABLE_REG_OFFSET		0x31c
#define SE_KEYTABLE_SLOT_SHIFT		4
#define SE_KEYTABLE_SLOT(x)			(x << SE_KEYTABLE_SLOT_SHIFT)
#define SE_KEYTABLE_QUAD_SHIFT		2
#define QUAD_KEYS_128	0
#define QUAD_KEYS_192	1
#define QUAD_KEYS_256	1
#define QUAD_ORG_IV		2
#define QUAD_UPDTD_IV	3
#define SE_KEYTABLE_QUAD(x)			(x << SE_KEYTABLE_QUAD_SHIFT)
#define SE_KEYTABLE_OP_TYPE_SHIFT	9
#define OP_READ			0
#define OP_WRITE		1
#define SE_KEYTABLE_OP_TYPE(x)		(x << SE_KEYTABLE_OP_TYPE_SHIFT)
#define SE_KEYTABLE_TABLE_SEL_SHIFT		8
#define TABLE_KEYIV		0
#define TABLE_SCHEDULE	1
#define SE_KEYTABLE_TABLE_SEL(x)	(x << SE_KEYTABLE_TABLE_SEL_SHIFT)
#define SE_KEYTABLE_PKT_SHIFT		0
#define SE_KEYTABLE_PKT(x)			(x << SE_KEYTABLE_PKT_SHIFT)

#define SE_CRYPTO_REG_OFFSET		0x304
#define SE_CRYPTO_HASH_SHIFT		0
#define HASH_DISABLE	0
#define HASH_ENABLE		1
#define SE_CRYPTO_HASH(x)			(x << SE_CRYPTO_HASH_SHIFT)
#define SE_CRYPTO_XOR_POS_SHIFT		1
#define XOR_BYPASS		0
#define XOR_TOP			2
#define XOR_BOTTOM		3
#define SE_CRYPTO_XOR_POS(x)		(x << SE_CRYPTO_XOR_POS_SHIFT)
#define SE_CRYPTO_INPUT_SEL_SHIFT	3
#define INPUT_AHB		0
#define INPUT_LFSR		1
#define INPUT_AESOUT	2
#define INPUT_LNR_CTR	3
#define SE_CRYPTO_INPUT_SEL(x)		(x << SE_CRYPTO_INPUT_SEL_SHIFT)
#define SE_CRYPTO_VCTRAM_SEL_SHIFT	5
#define VCTRAM_AHB		0
#define VCTRAM_AESOUT	2
#define VCTRAM_PREVAHB	3
#define SE_CRYPTO_VCTRAM_SEL(x)		(x << SE_CRYPTO_VCTRAM_SEL_SHIFT)
#define SE_CRYPTO_IV_SEL_SHIFT		7
#define IV_ORIGINAL		0
#define IV_UPDATED		1
#define SE_CRYPTO_IV_SEL(x)			(x << SE_CRYPTO_IV_SEL_SHIFT)
#define SE_CRYPTO_CORE_SEL_SHIFT	8
#define CORE_DECRYPT	0
#define CORE_ENCRYPT	1
#define SE_CRYPTO_CORE_SEL(x)		(x << SE_CRYPTO_CORE_SEL_SHIFT)
#define SE_CRYPTO_CTR_VAL_SHIFT		11
#define SE_CRYPTO_CTR_VAL(x)		(x << SE_CRYPTO_CTR_VAL_SHIFT)
#define SE_CRYPTO_KEY_INDEX_SHIFT	24
#define SE_CRYPTO_KEY_INDEX(x)		(x << SE_CRYPTO_KEY_INDEX_SHIFT)
#define SE_CRYPTO_CTR_CNTN_SHIFT	11
#define SE_CRYPTO_CTR_CNTN(x)		(x << SE_CRYPTO_CTR_CNTN_SHIFT)

#define SE_CRYPTO_CTR_REG_COUNT		4
#define SE_CRYPTO_CTR_REG_OFFSET	0x308

#define SE_OPERATION_REG_OFFSET		0x008
#define SE_OPERATION_SHIFT			0
#define OP_ABORT		0
#define OP_SRART		1
#define OP_RESTART		2
#define OP_CTX_SAVE		3
#define SE_OPERATION(x)				(x << SE_OPERATION_SHIFT)

#define SE_CONTEXT_SAVE_CONFIG_REG_OFFSET		0x070
#define SE_CONTEXT_SAVE_WORD_QUAD_SHIFT		0
#define KEYS_0_3		0
#define KEYS_4_7		1
#define ORIG_IV		2
#define UPD_IV		3
#define SE_CONTEXT_SAVE_WORD_QUAD(x)	(x << SE_CONTEXT_SAVE_WORD_QUAD_SHIFT)

#define SE_CONTEXT_SAVE_KEY_INDEX_SHIFT		8
#define SE_CONTEXT_SAVE_KEY_INDEX(x)	(x << SE_CONTEXT_SAVE_KEY_INDEX_SHIFT)


#define SE_CONTEXT_SAVE_SRC_SHIFT		30
#define STICKY_BITS		0
#define KEYTABLE		1
#define MEM		2
#define SRK		3
#define SE_CONTEXT_SAVE_SRC(x)		(x << SE_CONTEXT_SAVE_SRC_SHIFT)

#define SE_INT_ENABLE_REG_OFFSET	0x00c
#define SE_INT_STATUS_REG_OFFSET	0x010
#define INT_DISABLE		0
#define INT_ENABLE		1
#define INT_UNSET		0
#define INT_SET			1
#define SE_INT_OP_DONE_SHIFT		4
#define SE_INT_OP_DONE(x)			(x << SE_INT_OP_DONE_SHIFT)
#define SE_INT_ERROR_SHIFT		16
#define SE_INT_ERROR(x)			(x << SE_INT_ERROR_SHIFT)

#define SE_CRYPTO_KEYTABLE_DST_REG_OFFSET	0X330
#define SE_CRYPTO_KEYTABLE_DST_WORD_QUAD_SHIFT		0
#define SE_CRYPTO_KEYTABLE_DST_WORD_QUAD(x)		\
		(x << SE_CRYPTO_KEYTABLE_DST_WORD_QUAD_SHIFT)

#define SE_KEY_INDEX_SHIFT		8
#define SE_CRYPTO_KEYTABLE_DST_KEY_INDEX(x)	(x << SE_KEY_INDEX_SHIFT)

#define SE_IN_LL_ADDR_REG_OFFSET	0x018
#define SE_OUT_LL_ADDR_REG_OFFSET	0x024

#define SE_KEYTABLE_DATA0_REG_OFFSET	0x320
#define SE_KEYTABLE_REG_MAX_DATA		16

#define SE_BLOCK_COUNT_REG_OFFSET	0x318

#define SE_SPARE_0_REG_OFFSET		0x80c

#define SE_SHA_CONFIG_REG_OFFSET	0x200
#define SHA_DISABLE		0
#define SHA_ENABLE		1

#define SE_SHA_MSG_LENGTH_REG_OFFSET	0x204
#define SE_SHA_MSG_LEFT_REG_OFFSET		0x214


#define SE_HASH_RESULT_REG_COUNT	16
#define SE_HASH_RESULT_REG_OFFSET	0x030


#define TEGRA_SE_KEY_256_SIZE		32
#define TEGRA_SE_KEY_192_SIZE		24
#define TEGRA_SE_KEY_128_SIZE		16
#define TEGRA_SE_AES_BLOCK_SIZE		16
#define TEGRA_SE_AES_MIN_KEY_SIZE	16
#define TEGRA_SE_AES_MAX_KEY_SIZE	32
#define TEGRA_SE_AES_IV_SIZE		16
#define TEGRA_SE_RNG_IV_SIZE		16
#define TEGRA_SE_RNG_DT_SIZE		16
#define TEGRA_SE_RNG_KEY_SIZE		16
#define TEGRA_SE_RNG_SEED_SIZE		(TEGRA_SE_RNG_IV_SIZE + \
						TEGRA_SE_RNG_KEY_SIZE + \
						TEGRA_SE_RNG_DT_SIZE)
#define TEGRA_SE_AES_CMAC_DIGEST_SIZE	16
#define TEGRA_SE_RSA512_DIGEST_SIZE	64
#define TEGRA_SE_RSA1024_DIGEST_SIZE	128
#define TEGRA_SE_RSA1536_DIGEST_SIZE	192
#define TEGRA_SE_RSA2048_DIGEST_SIZE	256

#define SE_KEY_TABLE_ACCESS_REG_OFFSET	0x284
#define SE_KEY_READ_DISABLE_SHIFT		0

#define SE_CONTEXT_BUFER_SIZE	1072
#define SE_CONTEXT_SAVE_RANDOM_DATA_OFFSET	0
#define SE_CONTEXT_SAVE_RANDOM_DATA_SIZE	16
#define SE_CONTEXT_SAVE_STICKY_BITS_OFFSET	\
	(SE_CONTEXT_SAVE_RANDOM_DATA_OFFSET + SE_CONTEXT_SAVE_RANDOM_DATA_SIZE)
#define SE_CONTEXT_SAVE_STICKY_BITS_SIZE	16
#define SE_CONTEXT_SAVE_KEYS_OFFSET	(SE_CONTEXT_SAVE_STICKY_BITS_OFFSET + \
					SE_CONTEXT_SAVE_STICKY_BITS_SIZE)
#define SE_CONTEXT_SAVE_KEY_LENGTH	512
#define SE_CONTEXT_ORIGINAL_IV_OFFSET	(SE_CONTEXT_SAVE_KEYS_OFFSET + \
			SE_CONTEXT_SAVE_KEY_LENGTH)
#define SE_CONTEXT_ORIGINAL_IV_LENGTH	256

#define SE_CONTEXT_UPDATED_IV_OFFSET	(SE_CONTEXT_ORIGINAL_IV_OFFSET + \
			SE_CONTEXT_ORIGINAL_IV_LENGTH)

#define SE_CONTEXT_UPDATED_IV_LENGTH	256
#define SE_CONTEXT_KNOWN_PATTERN_OFFSET	(SE_CONTEXT_UPDATED_IV_OFFSET + \
			SE_CONTEXT_UPDATED_IV_LENGTH)
#define SE_CONTEXT_KNOWN_PATTERN_SIZE	16

#define TEGRA_SE_RSA_KEYSLOT_COUNT		2

#define SE_RSA_KEYTABLE_ADDR	0x420
#define SE_RSA_KEYTABLE_DATA	0x424
#define SE_RSA_OUTPUT 0x428

#define RSA_KEY_READ	0
#define RSA_KEY_WRITE	1
#define SE_RSA_KEY_OP_SHIFT	10
#define SE_RSA_KEY_OP(x)	(x << SE_RSA_KEY_OP_SHIFT)

#define RSA_KEY_INPUT_MODE_REG	0
#define RSA_KEY_INPUT_MODE_DMA	1
#define RSA_KEY_INPUT_MODE_SHIFT	8
#define RSA_KEY_INPUT_MODE(x)	(x << RSA_KEY_INPUT_MODE_SHIFT)


#define RSA_KEY_SLOT_ONE	0
#define RSA_KEY_SLOT_TW0	1
#define RSA_KEY_NUM_SHIFT	7
#define RSA_KEY_NUM(x)	(x << RSA_KEY_NUM_SHIFT)

#define RSA_KEY_TYPE_EXP	0
#define RSA_KEY_TYPE_MOD	1
#define RSA_KEY_TYPE_SHIFT	6
#define RSA_KEY_TYPE(x)	(x << RSA_KEY_TYPE_SHIFT)

#define SE_RSA_KEY_SIZE_REG_OFFSET	0x404
#define SE_RSA_EXP_SIZE_REG_OFFSET	0x408

#define RSA_KEY_SLOT_SHIFT	24
#define RSA_KEY_SLOT(x)	(x << RSA_KEY_SLOT_SHIFT)
#define SE_RSA_CONFIG	0x400

#define RSA_KEY_PKT_WORD_ADDR_SHIFT	0
#define RSA_KEY_PKT_WORD_ADDR(x)	(x << RSA_KEY_PKT_WORD_ADDR_SHIFT)

#define RSA_KEY_WORD_ADDR_SHIFT	0
#define RSA_KEY_WORD_ADDR(x)	(x << RSA_KEY_WORD_ADDR_SHIFT)

#define SE_RSA_KEYTABLE_PKT_SHIFT	0
#define SE_RSA_KEYTABLE_PKT(x)	(x << SE_RSA_KEYTABLE_PKT_SHIFT)

#endif /* _CRYPTO_TEGRA_SE_H */
