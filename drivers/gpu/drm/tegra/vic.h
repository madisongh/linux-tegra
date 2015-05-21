/*
 * Copyright (C) 2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TEGRA_VIC_H
#define TEGRA_VIC_H

/* VIC methods */

#define VIC_SET_APPLICATION_ID			0x00000200
#define VIC_SET_FCE_UCODE_SIZE			0x0000071C
#define VIC_SET_FCE_UCODE_OFFSET		0x0000072C
#define VIC_SET_SURFACE0_SLOT0_LUMA_OFFSET	0x00000400
#define VIC_SET_SURFACE7_SLOT4_CHROMAV_OFFSET	0x000005dc
#define VIC_SET_CONFIG_STRUCT_OFFSET		0x00000720
#define VIC_SET_OUTPUT_SURFACE_CHROMAV_OFFSET	0x00000738

/* VIC registers */

#define NV_PVIC_MISC_PRI_VIC_CG			0x000016d0
#define CG_IDLE_CG_DLY_CNT(val)			((val & 0x3f) << 0)
#define CG_IDLE_CG_EN				(1 << 6)
#define CG_WAKEUP_DLY_CNT(val)			((val & 0xf) << 16)

/* Firmware offsets */

#define VIC_UCODE_FCE_HEADER_OFFSET		(6*4)
#define VIC_UCODE_FCE_DATA_OFFSET		(7*4)
#define FCE_UCODE_SIZE_OFFSET			(2*4)

#endif /* TEGRA_VIC_H */
