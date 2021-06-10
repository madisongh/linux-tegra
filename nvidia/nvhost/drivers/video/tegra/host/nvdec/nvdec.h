/*
 * drivers/video/tegra/host/nvdec/nvdec.h
 *
 * Tegra NVDEC Module Support
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __NVHOST_NVDEC_H__
#define __NVHOST_NVDEC_H__

#include <linux/types.h>
#include <linux/nvhost.h>

extern const struct file_operations tegra_nvdec_ctrl_ops;

int nvhost_nvdec_finalize_poweron(struct platform_device *dev);
int nvhost_nvdec_prepare_poweroff(struct platform_device *dev);
int nvhost_nvdec_t210_finalize_poweron(struct platform_device *dev);

/* Would have preferred a static inline here... but we're using this
 * in a place where a constant initializer is required */
#define NVHOST_ENCODE_NVDEC_VER(maj, min) \
	((((maj) & 0xff) << 8) | ((min) & 0xff))

static inline void decode_nvdec_ver(int version, u8 *maj, u8 *min)
{
	u32 uv32 = (u32)version;
	*maj = (u8)((uv32 >> 8) & 0xff);
	*min = (u8)(uv32 & 0xff);
}

struct nvdec_private {
	struct platform_device *pdev;
	atomic_t refcnt;
};

struct nvdec {
	bool valid;
	size_t size;

	struct {
		u32 bin_data_offset;
		u32 data_offset;
		u32 data_size;
		u32 code_offset;
		u32 size;
	} os;

	struct sg_table *pa;
	dma_addr_t phys;
	u32 *mapped;
};

struct nvdec_ucode_bin_header_v1 {
	u32 bin_magic;		/* 0x10de */
	u32 bin_ver;		/* cya, versioning of bin format (1) */
	u32 bin_size;		/* entire image size including this header */
	u32 os_bin_header_offset;
	u32 os_bin_data_offset;
	u32 os_bin_size;
};

struct nvdec_ucode_os_code_header_v1 {
	u32 offset;
	u32 size;
};

struct nvdec_ucode_os_header_v1 {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	struct nvdec_ucode_os_code_header_v1 *app_code;
	struct nvdec_ucode_os_code_header_v1 *app_data;
	u32 *os_ovl_offset;
	u32 *of_ovl_size;
};

struct nvdec_ucode_v1 {
	struct nvdec_ucode_bin_header_v1 *bin_header;
	struct nvdec_ucode_os_header_v1  *os_header;
	bool valid;
};

struct nvdec_bl_shared_data {
	uint ls_fw_start_addr;
	uint ls_fw_size;
	uint wpr_addr;
	uint wpr_size;
};

#endif
