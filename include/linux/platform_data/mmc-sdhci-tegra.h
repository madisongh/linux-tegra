/*
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __PLATFORM_DATA_TEGRA_SDHCI_H
#define __PLATFORM_DATA_TEGRA_SDHCI_H

#include <linux/mmc/host.h>

/* uhs mask can be used to mask any of the UHS modes support */
#define MMC_UHS_MASK_DDR50	0x8
#define MMC_MASK_HS200		0x20
#define MMC_MASK_HS400		0x40

struct tegra_sdhci_platform_data {
	unsigned int max_clk_limit;
	unsigned int uhs_mask;
	unsigned int ddr_tap_delay;
	unsigned int ddr_trim_delay;
	bool en_strobe;
};

#endif
