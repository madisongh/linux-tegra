/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Sumit Sharma <sumsharma@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __TEGRA_FUSE_H
#define __TEGRA_FUSE_H

#include <linux/tegra-soc.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>

#define FUSE_FUSEBYPASS_0		0x24
#define FUSE_WRITE_ACCESS_SW_0		0x30

#define FUSE_SKU_INFO			0x10
#define FUSE_SKU_MSB_MASK		0xFF00
#define FUSE_SKU_MSB_SHIFT		8

#define FUSE_SKU_USB_CALIB_0		0xf0

#define FUSE_OPT_SUBREVISION		0x148
#define FUSE_OPT_SUBREVISION_MASK	0xF

#define FUSE_OPT_PRIV_SEC_DIS_0		0x164
#define FUSE_OPT_PRIV_SEC_EN_0		0x164
#define FUSE_GCPLEX_CONFIG_FUSE_0	0x1c8

#define FUSE_TSENSOR8_CALIB		0x180
#define FUSE_SPARE_REALIGNMENT_REG_0	0x1fc

#define FUSE_OPT_GPU_TPC0_DISABLE_0	0x20c
#define FUSE_OPT_GPU_TPC1_DISABLE_0	0x23c

#define FUSE_USB_CALIB_EXT_0		0x250

#define FUSE_OPT_ECC_EN			0x258

unsigned long long tegra_chip_uid(void);
int tegra_fuse_readl(unsigned long offset, u32 *val);

#if defined(CONFIG_TEGRA_FUSE)
bool tegra_spare_fuse(int bit);
u8 tegra_get_chip_id(void);
int tegra_get_sku_override(void);
u32 tegra_get_sku_id(void);
#endif

void tegra_fuse_writel(u32 val, unsigned long offset);

extern enum tegra_revision tegra_chip_get_revision(void);

extern int (*tegra_fuse_regulator_en)(int);
int tegra_soc_speedo_id(void);
void tegra_init_speedo_data(void);
int tegra_cpu_process_id(void);
int tegra_core_process_id(void);
int tegra_gpu_process_id(void);
int tegra_get_age(void);
static inline bool tegra_is_soc_automotive_speedo(void)
{
	return 0;
}

int tegra_package_id(void);
int tegra_cpu_speedo_id(void);
int tegra_cpu_speedo_mv(void);
int tegra_cpu_speedo_value(void);
int tegra_core_speedo_mv(void);
int tegra_core_speedo_min_mv(void);
int tegra_gpu_speedo_id(void);
int tegra_fuse_get_cpu_iddq(void);
int tegra_get_chip_personality(void);

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
int tegra_cpu_speedo_0_value(void);
int tegra_cpu_speedo_1_value(void);
int tegra_soc_speedo_0_value(void);
int tegra_soc_speedo_1_value(void);
int tegra_soc_speedo_2_value(void);
int tegra_fuse_get_soc_iddq(void);
int tegra_fuse_get_gpu_iddq(void);
int tegra_gpu_speedo_value(void);
#endif

int tegra_fuse_get_tsensor_calib(int index, u32 *calib);
int tegra_fuse_calib_base_get_cp(u32 *base_cp, s32 *shifted_cp);
int tegra_fuse_calib_base_get_ft(u32 *base_ft, s32 *shifted_ft);

int tegra_fuse_control_read(unsigned long offset, u32 *value);
void tegra_fuse_control_write(u32 value, unsigned long offset);
void tegra_pmc_fuse_disable_mirroring(void);
void tegra_pmc_fuse_enable_mirroring(void);
#endif /* TEGRA_FUSE_H */
