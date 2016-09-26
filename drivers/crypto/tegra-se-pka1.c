
/*
 * Cryptographic API.
 * drivers/crypto/tegra-se-pka1.c
 *
 * Support for Tegra Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2016, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation, and may be copied,
 * distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/tegra-soc.h>

#include "tegra-se-pka1.h"

#define DRIVER_NAME	"tegra-se-pka1"

#define TEGRA_SE_MUTEX_WDT_UNITS	0x600000
#define PKA1_TIMEOUT			2000	/*micro seconds*/

enum tegra_se_pka1_rsa_type {
	RSA_EXP_MOD,
	RSA_CRT_KEY_SETUP,
	RSA_CRT,
};

enum tegra_se_pka1_ecc_type {
	ECC_POINT_MUL,
	ECC_POINT_ADD,
	ECC_POINT_DOUBLE,
	ECC_POINT_VER,
	ECC_SHAMIR_TRICK,
};

enum tegra_se_pka1_precomp_vals {
	PRECOMP_RINV,
	PRECOMP_M,
	PRECOMP_R2,
};

/* Security Engine operation modes */
enum tegra_se_pka1_op_mode {
	SE_PKA1_OP_MODE_RSA512,
	SE_PKA1_OP_MODE_RSA768,
	SE_PKA1_OP_MODE_RSA1024,
	SE_PKA1_OP_MODE_RSA1536,
	SE_PKA1_OP_MODE_RSA2048,
	SE_PKA1_OP_MODE_RSA3072,
	SE_PKA1_OP_MODE_RSA4096,
	SE_PKA1_OP_MODE_ECC160,
	SE_PKA1_OP_MODE_ECC192,
	SE_PKA1_OP_MODE_ECC224,
	SE_PKA1_OP_MODE_ECC256,
	SE_PKA1_OP_MODE_ECC384,
	SE_PKA1_OP_MODE_ECC512,
	SE_PKA1_OP_MODE_ECC521,
};

struct tegra_se_chipdata {
	bool use_key_slot;
};

struct tegra_se_pka1_dev {
	struct device *dev;
	void __iomem *io_reg;
	struct clk *c;
	struct tegra_se_slot *slot_list;
	const struct tegra_se_chipdata *chipdata;
	struct tegra_se_pka1_request *pka1_req;
};

/* TODO: Planning to remove global pka1_dev once crypto framework
 * APIs are re-routed to this driver from a context
 */
static struct tegra_se_pka1_dev *pka1_dev;

struct tegra_se_pka1_request {
	struct tegra_se_pka1_dev *se_dev;
	struct tegra_se_slot *slot;
	u32 *message;
	u32 *result;
	u32 *exponent;
	u32 *modulus;
	u32 *m;
	u32 *r2;
	u32 *rinv;
	int op_mode;
	int size;
	int ecc_type;
	int rsa_type;
	u32 *curve_param_a;
	u32 *curve_param_b;
	u32 *order;
	u32 *base_pt_x;
	u32 *base_pt_y;
	u32 *res_pt_x;
	u32 *res_pt_y;
	u32 *key;
	bool pv_ok;
};

/* Security Engine key slot */
struct tegra_se_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	atomic_t available; /* Tells whether key slot is free to use */
};

static LIST_HEAD(key_slot);

static u32 pka1_op_size[] = {512, 768, 1024, 1536, 2048, 3072, 4096, 160, 192,
				224, 256, 384, 512, 640};
static inline u32 num_words(int mode)
{
	u32 words = 0;

	switch (mode) {
	case SE_PKA1_OP_MODE_ECC160:
	case SE_PKA1_OP_MODE_ECC192:
	case SE_PKA1_OP_MODE_ECC224:
	case SE_PKA1_OP_MODE_ECC256:
		words = pka1_op_size[SE_PKA1_OP_MODE_ECC256] / 32;
		break;
	case SE_PKA1_OP_MODE_RSA512:
	case SE_PKA1_OP_MODE_ECC384:
	case SE_PKA1_OP_MODE_ECC512:
		words = pka1_op_size[SE_PKA1_OP_MODE_RSA512] / 32;
		break;
	case SE_PKA1_OP_MODE_RSA768:
	case SE_PKA1_OP_MODE_RSA1024:
	case SE_PKA1_OP_MODE_ECC521:
		words = pka1_op_size[SE_PKA1_OP_MODE_RSA1024] / 32;
		break;
	case SE_PKA1_OP_MODE_RSA1536:
	case SE_PKA1_OP_MODE_RSA2048:
		words = pka1_op_size[SE_PKA1_OP_MODE_RSA2048] / 32;
		break;
	case SE_PKA1_OP_MODE_RSA3072:
	case SE_PKA1_OP_MODE_RSA4096:
		words = pka1_op_size[SE_PKA1_OP_MODE_RSA4096] / 32;
		break;
	default:
		dev_warn(pka1_dev->dev, "Invalid operation mode\n");
		break;
	}

	return words;
}

static inline void se_pka1_writel(struct tegra_se_pka1_dev *se_dev,
				  unsigned int val, unsigned int reg_offset)
{
	writel(val, se_dev->io_reg + reg_offset);
}

static inline unsigned int se_pka1_readl(struct tegra_se_pka1_dev *se_dev,
					 unsigned int reg_offset)
{
	return readl(se_dev->io_reg + reg_offset);
}

static void tegra_se_pka1_free_key_slot(struct tegra_se_slot *slot)
{
	if (!slot)
		return;
	atomic_set(&slot->available, 1);
}

static struct tegra_se_slot *tegra_se_pka1_alloc_key_slot(void)
{
	struct tegra_se_slot *slot;
	bool found = false;

	list_for_each_entry(slot, &key_slot, node) {
		if (atomic_read(&slot->available)) {
			atomic_set(&slot->available, 0);
			found = true;
			break;
		}
	}

	return found ? slot : NULL;
}

static int tegra_se_pka1_init_key_slot(struct tegra_se_pka1_dev *se_dev)
{
	int i;

	se_dev->slot_list = devm_kzalloc(se_dev->dev,
					 sizeof(struct tegra_se_slot) *
					 TEGRA_SE_PKA1_KEYSLOT_COUNT,
					 GFP_KERNEL);
	if (!se_dev->slot_list)
		return -ENOMEM;

	for (i = 0; i < TEGRA_SE_PKA1_KEYSLOT_COUNT; i++) {
		atomic_set(&se_dev->slot_list[i].available, 1);
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}

	return 0;
}

static u32 tegra_se_check_trng_op(struct tegra_se_pka1_dev *se_dev)
{
	u32 trng_val;
	u32 val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_TRNG_STATUS_OFFSET);

	trng_val = TEGRA_SE_PKA1_TRNG_STATUS_SECURE(TRNG_TRUE) |
			TEGRA_SE_PKA1_TRNG_STATUS_NONCE(TRNG_FALSE) |
			TEGRA_SE_PKA1_TRNG_STATUS_SEEDED(TRNG_TRUE) |
			TEGRA_SE_PKA1_TRNG_STATUS_LAST_RESEED(
						TRNG_LAST_RESEED_HOST);
	if ((val & trng_val) ||
	    (val & TEGRA_SE_PKA1_TRNG_STATUS_LAST_RESEED
					(TRNG_LAST_RESEED_RESEED)))
		return 0;

	return -EINVAL;
}

static u32 tegra_se_set_trng_op(struct tegra_se_pka1_dev *se_dev)
{
	u32 val, i = 0;

	se_pka1_writel(se_dev,
		      TEGRA_SE_PKA1_TRNG_SMODE_SECURE(PKA1_ENABLE) |
		      TEGRA_SE_PKA1_TRNG_SMODE_NONCE(PKA1_DISABLE),
		      TEGRA_SE_PKA1_TRNG_SMODE_OFFSET);
	se_pka1_writel(se_dev,
		      TEGRA_SE_PKA1_CTRL_CONTROL_AUTO_RESEED(PKA1_ENABLE),
		      TEGRA_SE_PKA1_CTRL_CONTROL_OFFSET);

	/* Poll seeded status */
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev,
				"Poll TRNG seeded status timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_TRNG_STATUS_OFFSET);
		i++;
	} while (val & TEGRA_SE_PKA1_TRNG_STATUS_SEEDED(TRNG_FALSE));

	return 0;
}

static void tegra_se_restart_pka1_mutex_wdt(struct tegra_se_pka1_dev *se_dev)
{
	se_pka1_writel(se_dev, TEGRA_SE_MUTEX_WDT_UNITS,
		      TEGRA_SE_PKA1_MUTEX_WATCHDOG_OFFSET);
}

static u32 tegra_se_acquire_pka1_mutex(struct tegra_se_pka1_dev *se_dev)
{
	u32 val, i = 0;

	/* Acquire pka mutex */
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev, "Acquire PKA Mutex timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_MUTEX_OFFSET);
		i++;
	} while (val != 0x01);

	/* One unit is 256 SE Cycles */
	tegra_se_restart_pka1_mutex_wdt(se_dev);
	se_pka1_writel(se_dev, TEGRA_SE_PKA1_MUTEX_TIMEOUT_ACTION,
		      TEGRA_SE_PKA1_MUTEX_TIMEOUT_ACTION_OFFSET);

	return 0;
}

static void tegra_se_release_pka1_mutex(struct tegra_se_pka1_dev *se_dev)
{
	se_pka1_writel(se_dev, 0x01, TEGRA_SE_PKA1_MUTEX_RELEASE_OFFSET);
}

static inline u32 pka1_bank_start(u32 bank)
{
	return PKA1_BANK_START_A + (bank * 0x400);
}

static inline u32 reg_bank_offset(u32 bank, u32 idx, u32 mode)
{
	return pka1_bank_start(bank) + ((idx * 4) * num_words(mode));
}

static void tegra_se_fill_pka1_opmem_addr(struct tegra_se_pka1_dev *se_dev,
					 struct tegra_se_pka1_request *req)
{
	u32 i;
	int len = req->size;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K, *QX, *QY;

	MOD = req->modulus;
	M = req->m;
	R2 = req->r2;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_PKA1_OP_MODE_RSA512:
	case SE_PKA1_OP_MODE_RSA768:
	case SE_PKA1_OP_MODE_RSA1024:
	case SE_PKA1_OP_MODE_RSA1536:
	case SE_PKA1_OP_MODE_RSA2048:
	case SE_PKA1_OP_MODE_RSA3072:
	case SE_PKA1_OP_MODE_RSA4096:
		EXP = req->exponent;
		MSG = req->message;

		for (i = 0; i < req->size / 4; i++) {
			se_pka1_writel(se_dev, *EXP++, reg_bank_offset(
				      TEGRA_SE_PKA1_RSA_EXP_BANK,
				      TEGRA_SE_PKA1_RSA_EXP_ID,
				      req->op_mode) + (i * 4));
			se_pka1_writel(se_dev, *MSG++, reg_bank_offset(
				      TEGRA_SE_PKA1_RSA_MSG_BANK,
				      TEGRA_SE_PKA1_RSA_MSG_ID,
				      req->op_mode) + (i * 4));
		}
		break;

	case SE_PKA1_OP_MODE_ECC160:
	case SE_PKA1_OP_MODE_ECC192:
	case SE_PKA1_OP_MODE_ECC224:
	case SE_PKA1_OP_MODE_ECC256:
	case SE_PKA1_OP_MODE_ECC384:
	case SE_PKA1_OP_MODE_ECC512:
	case SE_PKA1_OP_MODE_ECC521:
		A = req->curve_param_a;

		if (req->op_mode == SE_PKA1_OP_MODE_ECC521) {
			for (i = 0; i < req->size / 4; i++)
				se_pka1_writel(se_dev, *MOD++,
					      reg_bank_offset(
						TEGRA_SE_PKA1_MOD_BANK,
						TEGRA_SE_PKA1_MOD_ID,
						req->op_mode) + (i * 4));
		}

		for (i = 0; i < req->size / 4; i++)
			se_pka1_writel(se_dev, *A++, reg_bank_offset(
					TEGRA_SE_PKA1_ECC_A_BANK,
					TEGRA_SE_PKA1_ECC_A_ID,
					req->op_mode) + (i * 4));

		if (req->ecc_type != ECC_POINT_DOUBLE) {
			PX = req->base_pt_x;
			PY = req->base_pt_y;
			for (i = 0; i < req->size / 4; i++) {
				se_pka1_writel(se_dev, *PX++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_XP_BANK,
						TEGRA_SE_PKA1_ECC_XP_ID,
						req->op_mode) + (i * 4));

				se_pka1_writel(se_dev, *PY++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_YP_BANK,
						TEGRA_SE_PKA1_ECC_YP_ID,
						req->op_mode) + (i * 4));
			}
		}

		if (req->ecc_type == ECC_POINT_VER ||
		    req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, curve_param_b is parameter k
			 * and k should be of size CTRL_BASE_RADIX
			 */
			if (req->ecc_type == ECC_SHAMIR_TRICK)
				len = (num_words(req->op_mode)) * 4;

			B = req->curve_param_b;
			for (i = 0; i < len / 4; i++)
				se_pka1_writel(se_dev, *B++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_B_BANK,
						TEGRA_SE_PKA1_ECC_B_ID,
						req->op_mode) + (i * 4));
		}

		if (req->ecc_type == ECC_POINT_ADD ||
		    req->ecc_type == ECC_SHAMIR_TRICK ||
		    req->ecc_type == ECC_POINT_DOUBLE) {
			QX = req->res_pt_x;
			QY = req->res_pt_y;
			for (i = 0; i < req->size / 4; i++) {
				se_pka1_writel(se_dev, *QX++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_XQ_BANK,
						TEGRA_SE_PKA1_ECC_XQ_ID,
						req->op_mode) + (i * 4));

				se_pka1_writel(se_dev, *QY++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_YQ_BANK,
						TEGRA_SE_PKA1_ECC_YQ_ID,
						req->op_mode) + (i * 4));
			}
		}

		if (req->ecc_type == ECC_POINT_MUL ||
		    req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, key is parameter l
			 * and k for ECC_POINT_MUL and l for ECC_SHAMIR_TRICK
			 * should be of size CTRL_BASE_RADIX
			 */
			len = (num_words(req->op_mode)) * 4;
			K = req->key;
			for (i = 0; i < len / 4; i++)
				se_pka1_writel(se_dev, *K++, reg_bank_offset(
						TEGRA_SE_PKA1_ECC_K_BANK,
						TEGRA_SE_PKA1_ECC_K_ID,
						req->op_mode) + (i * 4));
		}
		break;
	}
}

static u32 pka1_ctrl_base(u32 mode)
{
	struct tegra_se_pka1_dev *se_dev = pka1_dev;
	u32 val, base_radix;

	val = num_words(mode) * 32;
	switch (val) {
	case PKA1_OP_SIZE_256:
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_256;
		break;
	case PKA1_OP_SIZE_512:
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_512;
		break;
	case PKA1_OP_SIZE_1024:
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_1024;
		break;
	case PKA1_OP_SIZE_2048:
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_2048;
		break;
	case PKA1_OP_SIZE_4096:
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_4096;
		break;
	default:
		dev_warn(se_dev->dev, "Invalid size: using PKA1_OP_SIZE_256\n");
		base_radix = TEGRA_SE_PKA1_CTRL_BASE_256;
		break;
	}

	return base_radix;
}

static void tegra_se_program_pka1_regs(struct tegra_se_pka1_dev *se_dev,
				      struct tegra_se_pka1_request *req)
{
	u32 val;

	se_pka1_writel(se_dev, 0, TEGRA_SE_PKA1_FLAGS_OFFSET);
	se_pka1_writel(se_dev, 0, TEGRA_SE_PKA1_FSTACK_PTR_OFFSET);

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_PKA1_OP_MODE_RSA512:
	case SE_PKA1_OP_MODE_RSA768:
	case SE_PKA1_OP_MODE_RSA1024:
	case SE_PKA1_OP_MODE_RSA1536:
	case SE_PKA1_OP_MODE_RSA2048:
	case SE_PKA1_OP_MODE_RSA3072:
	case SE_PKA1_OP_MODE_RSA4096:
		se_pka1_writel(se_dev, TEGRA_SE_PKA1_RSA_MOD_EXP_PRG_ENTRY_VAL,
			       TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
		se_pka1_writel(se_dev,
			       TEGRA_SE_PKA1_INT_ENABLE_IE_IRQ_EN(PKA1_ENABLE),
			       TEGRA_SE_PKA1_INT_ENABLE_OFFSET);
		val =
		TEGRA_SE_PKA1_CTRL_BASE_RADIX(pka1_ctrl_base(req->op_mode))
			| TEGRA_SE_PKA1_CTRL_PARTIAL_RADIX(req->size / 4);
		val |= TEGRA_SE_PKA1_CTRL_GO(TEGRA_SE_PKA1_CTRL_GO_START);
		se_pka1_writel(se_dev, val, TEGRA_SE_PKA1_CTRL_OFFSET);
		break;

	case SE_PKA1_OP_MODE_ECC160:
	case SE_PKA1_OP_MODE_ECC192:
	case SE_PKA1_OP_MODE_ECC224:
	case SE_PKA1_OP_MODE_ECC256:
	case SE_PKA1_OP_MODE_ECC384:
	case SE_PKA1_OP_MODE_ECC512:
	case SE_PKA1_OP_MODE_ECC521:
		if (req->ecc_type == ECC_POINT_MUL) {
			se_pka1_writel(
				se_dev,
				TEGRA_SE_PKA1_ECC_POINT_MUL_PRG_ENTRY_VAL,
				TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
			/*clear F0 for binding val*/
			se_pka1_writel(
				se_dev,
				TEGRA_SE_PKA1_FLAGS_FLAG_F0(PKA1_DISABLE),
				TEGRA_SE_PKA1_FLAGS_OFFSET);
		} else if (req->ecc_type == ECC_POINT_ADD) {
			se_pka1_writel(
				se_dev,
				TEGRA_SE_PKA1_ECC_POINT_ADD_PRG_ENTRY_VAL,
				TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			se_pka1_writel(
				se_dev,
				TEGRA_SE_PKA1_ECC_POINT_DOUBLE_PRG_ENTRY_VAL,
				TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_VER) {
			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_ECC_ECPV_PRG_ENTRY_VAL,
				       TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
		} else {
			se_pka1_writel(
				se_dev,
				TEGRA_SE_PKA1_ECC_SHAMIR_TRICK_PRG_ENTRY_VAL,
				TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
		}

		se_pka1_writel(se_dev,
			       TEGRA_SE_PKA1_INT_ENABLE_IE_IRQ_EN(PKA1_ENABLE),
			       TEGRA_SE_PKA1_INT_ENABLE_OFFSET);

		if (req->op_mode == SE_PKA1_OP_MODE_ECC521) {
			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_FLAGS_FLAG_F1(PKA1_ENABLE),
				       TEGRA_SE_PKA1_FLAGS_OFFSET);
		}

		se_pka1_writel(se_dev,
			       TEGRA_SE_PKA1_CTRL_BASE_RADIX
				(pka1_ctrl_base(req->op_mode)) |
			       TEGRA_SE_PKA1_CTRL_PARTIAL_RADIX
				(req->size / 4) |
			       TEGRA_SE_PKA1_CTRL_GO
				(TEGRA_SE_PKA1_CTRL_GO_START),
			       TEGRA_SE_PKA1_CTRL_OFFSET);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}
}

static int tegra_se_check_pka1_op_done(struct tegra_se_pka1_dev *se_dev)
{
	u32 val, i = 0;
	u32 abnormal_val;

	/* poll pka done status*/
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev, "PKA Done status timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_STATUS_OFFSET);
		i++;
	} while (!(val & TEGRA_SE_PKA1_STATUS_IRQ_STAT(PKA1_ENABLE)));

	val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_RETURN_CODE_OFFSET);

	abnormal_val = TEGRA_SE_PKA1_RETURN_CODE_STOP_REASON(
			TEGRA_SE_PKA1_RETURN_CODE_STOP_REASON_ABNORMAL);

	if (abnormal_val & val) {
		dev_err(se_dev->dev, "PKA Operation ended Abnormally\n");
		return -EINVAL;
	}
	/* Write Status Register to acknowledge interrupt */
	val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_STATUS_OFFSET);
	se_pka1_writel(se_dev, val, TEGRA_SE_PKA1_STATUS_OFFSET);

	return 0;
}

static void tegra_se_read_pka1_result(struct tegra_se_pka1_dev *se_dev,
				     struct tegra_se_pka1_request *req)
{
	u32 val, i;
	u32 *RES = req->result;
	u32 *QX = req->res_pt_x;
	u32 *QY = req->res_pt_y;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_PKA1_OP_MODE_RSA512:
	case SE_PKA1_OP_MODE_RSA768:
	case SE_PKA1_OP_MODE_RSA1024:
	case SE_PKA1_OP_MODE_RSA1536:
	case SE_PKA1_OP_MODE_RSA2048:
	case SE_PKA1_OP_MODE_RSA3072:
	case SE_PKA1_OP_MODE_RSA4096:
		for (i = 0; i < req->size / 4; i++) {
			val = se_pka1_readl(se_dev, reg_bank_offset(
					    TEGRA_SE_PKA1_RSA_RESULT_BANK,
					    TEGRA_SE_PKA1_RSA_RESULT_ID,
					    req->op_mode) + (i * 4));
			*RES = be32_to_cpu(val);
			RES++;
		}
		break;

	case SE_PKA1_OP_MODE_ECC160:
	case SE_PKA1_OP_MODE_ECC192:
	case SE_PKA1_OP_MODE_ECC224:
	case SE_PKA1_OP_MODE_ECC256:
	case SE_PKA1_OP_MODE_ECC384:
	case SE_PKA1_OP_MODE_ECC512:
	case SE_PKA1_OP_MODE_ECC521:
		if (req->ecc_type == ECC_POINT_VER) {
			val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_FLAGS_OFFSET);
			if (val & TEGRA_SE_PKA1_FLAGS_FLAG_ZERO(PKA1_ENABLE))
				req->pv_ok = true;
			else
				req->pv_ok = false;
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			for (i = 0; i < req->size / 4; i++) {
				val = se_pka1_readl(se_dev, reg_bank_offset(
						    TEGRA_SE_PKA1_ECC_XP_BANK,
						    TEGRA_SE_PKA1_ECC_XP_ID,
						    req->op_mode) + (i * 4));
				*QX = be32_to_cpu(val);
				QX++;
			}
			for (i = 0; i < req->size / 4; i++) {
				val = se_pka1_readl(se_dev, reg_bank_offset(
						    TEGRA_SE_PKA1_ECC_YP_BANK,
						    TEGRA_SE_PKA1_ECC_YP_ID,
						    req->op_mode) + (i * 4));
				*QY = be32_to_cpu(val);
				QY++;
			}
		} else {
			for (i = 0; i < req->size / 4; i++) {
				val = se_pka1_readl(se_dev, reg_bank_offset(
						    TEGRA_SE_PKA1_ECC_XQ_BANK,
						    TEGRA_SE_PKA1_ECC_XQ_ID,
						    req->op_mode) + (i * 4));
				*QX = be32_to_cpu(val);
				QX++;
			}
			for (i = 0; i < req->size / 4; i++) {
				val = se_pka1_readl(se_dev, reg_bank_offset(
						    TEGRA_SE_PKA1_ECC_YQ_BANK,
						    TEGRA_SE_PKA1_ECC_YQ_ID,
						    req->op_mode) + (i * 4));
				*QY = be32_to_cpu(val);
				QY++;
			}
		}
		break;
	}
}

enum tegra_se_pka1_keyslot_field {
	EXPONENT,
	MOD_RSA,
	M_RSA,
	R2_RSA,
	PARAM_A,
	PARAM_B,
	MOD_ECC,
	XP,
	YP,
	XQ,
	YQ,
	KEY,
	M_ECC,
	R2_ECC,
};

static void tegra_se_set_pka1_key(struct tegra_se_pka1_dev *se_dev,
				 enum tegra_se_pka1_op_mode mode,
				 struct tegra_se_pka1_request *req)
{
	u32 i;
	u32 slot_num = req->slot->slot_num;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K;

	MOD = req->modulus;
	M = req->m;
	R2 = req->r2;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (mode) {
	case SE_PKA1_OP_MODE_RSA512:
	case SE_PKA1_OP_MODE_RSA768:
	case SE_PKA1_OP_MODE_RSA1024:
	case SE_PKA1_OP_MODE_RSA1536:
	case SE_PKA1_OP_MODE_RSA2048:
	case SE_PKA1_OP_MODE_RSA3072:
	case SE_PKA1_OP_MODE_RSA4096:
		EXP = req->exponent;
		MSG = req->message;
		for (i = 0; i < req->size / 4; i++) {
			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(EXPONENT) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *EXP++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(MOD_RSA) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *MOD++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(M_RSA) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *M++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(R2_RSA) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
				 		(slot_num));
			se_pka1_writel(se_dev, *R2++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));
		}
		break;

	case SE_PKA1_OP_MODE_ECC160:
	case SE_PKA1_OP_MODE_ECC192:
	case SE_PKA1_OP_MODE_ECC224:
	case SE_PKA1_OP_MODE_ECC256:
	case SE_PKA1_OP_MODE_ECC384:
	case SE_PKA1_OP_MODE_ECC512:
	case SE_PKA1_OP_MODE_ECC521:
		A = req->curve_param_a;
		B = req->curve_param_b;
		PX = req->base_pt_x;
		PY = req->base_pt_y;
		K = req->key;
		for (i = 0; i < req->size / 4; i++) {
			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(PARAM_A) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *A++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(PARAM_B) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *B++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(MOD_ECC) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *MOD++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD(XP) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *PX++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD(YP) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
					(slot_num));
			se_pka1_writel(se_dev, *PY++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD(KEY) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *K++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(M_ECC) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *M++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_pka1_writel(se_dev,
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_FIELD
						(R2_ECC) |
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_WORD(i),
				       TEGRA_SE_PKA1_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_pka1_writel(se_dev, *R2++,
				       TEGRA_SE_PKA1_KEYSLOT_DATA_OFFSET
						(slot_num));
		}
		break;
	}
}

static int tegra_se_pka1_precomp(struct tegra_se_pka1_dev *se_dev,
				    struct tegra_se_pka1_request *req,
				    u32 op)
{
	int ret, i;
	u32 *MOD = req->modulus;
	u32 *RINV = req->rinv;
	u32 *M = req->m;
	u32 *R2 = req->r2;

	if (req->op_mode == SE_PKA1_OP_MODE_ECC521)
		return 0;

	se_pka1_writel(se_dev, 0, TEGRA_SE_PKA1_FLAGS_OFFSET);
	se_pka1_writel(se_dev, 0, TEGRA_SE_PKA1_FSTACK_PTR_OFFSET);

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size / 4; i++) {
			se_pka1_writel(se_dev, *MOD++, reg_bank_offset(
				       TEGRA_SE_PKA1_MOD_BANK,
				       TEGRA_SE_PKA1_MOD_ID,
				       req->op_mode) + (i * 4));
		}

		se_pka1_writel(se_dev, TEGRA_SE_PKA1_RSA_RINV_PRG_ENTRY_VAL,
			       TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
	} else if (op == PRECOMP_M) {
		se_pka1_writel(se_dev, TEGRA_SE_PKA1_RSA_M_PRG_ENTRY_VAL,
			       TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
	} else {
		se_pka1_writel(se_dev, TEGRA_SE_PKA1_RSA_R2_PRG_ENTRY_VAL,
			       TEGRA_SE_PKA1_PRG_ENTRY_OFFSET);
	}

	se_pka1_writel(se_dev, TEGRA_SE_PKA1_INT_ENABLE_IE_IRQ_EN(PKA1_ENABLE),
		       TEGRA_SE_PKA1_INT_ENABLE_OFFSET);
	se_pka1_writel(se_dev, TEGRA_SE_PKA1_CTRL_BASE_RADIX(
			pka1_ctrl_base(req->op_mode)) |
		       TEGRA_SE_PKA1_CTRL_PARTIAL_RADIX(req->size / 4) |
		       TEGRA_SE_PKA1_CTRL_GO(TEGRA_SE_PKA1_CTRL_GO_START),
		       TEGRA_SE_PKA1_CTRL_OFFSET);

	ret = tegra_se_check_pka1_op_done(se_dev);
	if (ret)
		return ret;

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size / 4; i++) {
			*RINV = se_pka1_readl(se_dev, reg_bank_offset(
					      TEGRA_SE_PKA1_RINV_BANK,
					      TEGRA_SE_PKA1_RINV_ID,
					      req->op_mode) + (i * 4));
			RINV++;
		}
	} else if (op == PRECOMP_M) {
		for (i = 0; i < req->size / 4; i++) {
			*M = se_pka1_readl(se_dev, reg_bank_offset(
					   TEGRA_SE_PKA1_M_BANK,
					   TEGRA_SE_PKA1_M_ID,
					   req->op_mode) + (i * 4));
			M++;
		}
	} else {
		for (i = 0; i < req->size / 4; i++) {
			*R2 = se_pka1_readl(se_dev, reg_bank_offset(
					    TEGRA_SE_PKA1_R2_BANK,
					    TEGRA_SE_PKA1_R2_ID,
					    req->op_mode) + (i * 4));
			R2++;
		}
	}

	return ret;
}

static int tegra_se_pka1_do(struct tegra_se_pka1_dev *se_dev,
			       struct tegra_se_pka1_request *req)
{
	int ret;
	u32 val;
	struct tegra_se_slot *pslot;

	if (se_dev->chipdata->use_key_slot) {
		if (!req->slot) {
			pslot = tegra_se_pka1_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				return -ENOMEM;
			}
			req->slot = pslot;
		}
		tegra_se_set_pka1_key(se_dev, req->op_mode, req);
		/* Set LOAD_KEY */
		val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_CTRL_CONTROL_OFFSET);
		val |= TEGRA_SE_PKA1_CTRL_CONTROL_LOAD_KEY(PKA1_ENABLE);
		se_pka1_writel(se_dev, val, TEGRA_SE_PKA1_CTRL_CONTROL_OFFSET);

		/*Write KEYSLOT Number */
		val = se_pka1_readl(se_dev, TEGRA_SE_PKA1_CTRL_CONTROL_OFFSET);
		val |= TEGRA_SE_PKA1_CTRL_CONTROL_KEYSLOT(req->slot->slot_num);
		se_pka1_writel(se_dev, val, TEGRA_SE_PKA1_CTRL_CONTROL_OFFSET);
	} else {
		tegra_se_fill_pka1_opmem_addr(se_dev, req);
	}

	tegra_se_program_pka1_regs(se_dev, req);

	ret = tegra_se_check_pka1_op_done(se_dev);
	if (ret)
		return ret;

	tegra_se_read_pka1_result(se_dev, req);

	if (se_dev->chipdata->use_key_slot)
		tegra_se_pka1_free_key_slot(req->slot);

	return ret;
}

static int tegra_se_pka1_init(struct tegra_se_pka1_request *req)
{
	struct tegra_se_pka1_dev *se_dev = pka1_dev;

	if (req->op_mode == SE_PKA1_OP_MODE_ECC521)
		return 0;

	req->rinv = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->rinv)
		return -ENOMEM;

	req->m = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->m) {
		devm_kfree(se_dev->dev, req->rinv);
		return -ENOMEM;
	}

	req->r2 = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->r2) {
		devm_kfree(se_dev->dev, req->m);
		devm_kfree(se_dev->dev, req->rinv);
		return -ENOMEM;
	}

	return 0;
}

static void tegra_se_pka1_exit(struct tegra_se_pka1_request *req)
{
	struct tegra_se_pka1_dev *se_dev = pka1_dev;

	if (req->op_mode == SE_PKA1_OP_MODE_ECC521)
		return;

	devm_kfree(se_dev->dev, req->r2);
	devm_kfree(se_dev->dev, req->m);
	devm_kfree(se_dev->dev, req->rinv);
}

int tegra_se_pka1_op(struct tegra_se_pka1_request *req)
{
	struct tegra_se_pka1_dev *se_dev = pka1_dev;
	int ret;

	clk_prepare_enable(se_dev->c);
	ret = tegra_se_acquire_pka1_mutex(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "PKA1 Mutex acquire failed\n");
		goto clk_dis;
	}

	ret = tegra_se_pka1_init(req);
	if (ret)
		goto rel_mutex;

	ret = tegra_se_check_trng_op(se_dev);
	if (ret)
		ret = tegra_se_set_trng_op(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "set_trng_op Failed\n");
		goto exit;
	}
	ret = tegra_se_pka1_precomp(se_dev, req, PRECOMP_RINV);
	if (ret) {
		dev_err(se_dev->dev,
			"RINV: tegra_se_pka1_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_pka1_precomp(se_dev, req, PRECOMP_M);
	if (ret) {
		dev_err(se_dev->dev,
			"M: tegra_se_pka1_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_pka1_precomp(se_dev, req, PRECOMP_R2);
	if (ret) {
		dev_err(se_dev->dev,
			"R2: tegra_se_pka1_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_pka1_do(se_dev, req);
exit:
	tegra_se_pka1_exit(req);
rel_mutex:
	tegra_se_release_pka1_mutex(se_dev);
clk_dis:
	clk_disable_unprepare(se_dev->c);

	return ret;
}
EXPORT_SYMBOL(tegra_se_pka1_op);

static struct tegra_se_chipdata tegra214_se_chipdata = {
	.use_key_slot = false,
};

static int tegra_se_pka1_probe(struct platform_device *pdev)
{
	struct tegra_se_pka1_dev *se_dev;
	struct resource *res;
	int err;

	se_dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_se_pka1_dev),
			      GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	se_dev->chipdata = of_device_get_match_data(&pdev->dev);

	platform_set_drvdata(pdev, se_dev);
	se_dev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	se_dev->io_reg = devm_ioremap_resource(se_dev->dev, res);
	if (IS_ERR(se_dev->io_reg))
		return PTR_ERR(se_dev->io_reg);

	se_dev->c = devm_clk_get(se_dev->dev, "se");
	if (IS_ERR(se_dev->c)) {
		dev_err(se_dev->dev, "se clk_get_sys failed: %ld\n",
			PTR_ERR(se_dev->c));
		return PTR_ERR(se_dev->c);
	}

	err = clk_prepare_enable(se_dev->c);
	if (err) {
		dev_err(se_dev->dev, "clk enable failed for se\n");
		return err;
	}

	pka1_dev = se_dev;

	err = tegra_se_pka1_init_key_slot(se_dev);
	if (err)
		dev_err(se_dev->dev, "tegra_se_pka1_init_key_slot failed\n");

	clk_disable_unprepare(se_dev->c);

	if (!err)
		dev_info(se_dev->dev, "%s: complete", __func__);

	return err;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_se_pka1_suspend(struct device *dev)
{
	struct tegra_se_pka1_dev *se_dev = dev_get_drvdata(dev);

	/* This is needed as ATF (ARM Trusted Firmware) needs SE clk in SC7
	 * cycle and ATF does not have access to BPMP to enable the clk by
	 * itself. So, currently this is handled in linux driver.
	 */
	clk_prepare_enable(se_dev->c);

	return 0;
}

static int tegra_se_pka1_resume(struct device *dev)
{
	struct tegra_se_pka1_dev *se_dev = dev_get_drvdata(dev);

	clk_disable_unprepare(se_dev->c);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops tegra_se_pka1_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_se_pka1_suspend, tegra_se_pka1_resume)
};

static const struct of_device_id tegra_se_pka1_of_match[] = {
	{
		.compatible = "nvidia,tegra-se-pka1",
		.data = &tegra214_se_chipdata,
	},
};
MODULE_DEVICE_TABLE(of, tegra_se_pka1_of_match);

static struct platform_driver tegra_se_pka1_driver = {
	.probe  = tegra_se_pka1_probe,
	.driver = {
		.name   = "tegra-se-pka1",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_pka1_of_match),
		.pm = &tegra_se_pka1_pm_ops,
	},
};
module_platform_driver(tegra_se_pka1_driver);

MODULE_DESCRIPTION("Tegra Elliptic PKA1 Crypto algorithm support");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se-pka1");

