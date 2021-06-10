/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __ACR_FLCNBL_H__
#define __ACR_FLCNBL_H__

#include <nvgpu/flcnif_cmn.h>

#ifndef __NVGPU_ACR_H__
#warning "acr_flcnbl.h not included from nvgpu_acr.h!" \
	"Include nvgpu_acr.h instead of acr_xxx.h to get access to ACR interfaces"
#endif

/*
 * Structure used by the boot-loader to load the rest of the code. This has
 * to be filled by NVGPU and copied into DMEM at offset provided in the
 * hsflcn_bl_desc.bl_desc_dmem_load_off.
 */
struct flcn_bl_dmem_desc {
	u32    reserved[4];        /*Should be the first element..*/
	u32    signature[4];        /*Should be the first element..*/
	u32    ctx_dma;
	u32    code_dma_base;
	u32    non_sec_code_off;
	u32    non_sec_code_size;
	u32    sec_code_off;
	u32    sec_code_size;
	u32    code_entry_point;
	u32    data_dma_base;
	u32    data_size;
	u32    code_dma_base1;
	u32    data_dma_base1;
};

struct flcn_bl_dmem_desc_v1 {
	u32    reserved[4];        /*Should be the first element..*/
	u32    signature[4];        /*Should be the first element..*/
	u32    ctx_dma;
	struct falc_u64 code_dma_base;
	u32    non_sec_code_off;
	u32    non_sec_code_size;
	u32    sec_code_off;
	u32    sec_code_size;
	u32    code_entry_point;
	struct falc_u64 data_dma_base;
	u32    data_size;
	u32 argc;
	u32 argv;
};

/*
 * The header used by NVGPU to figure out code and data sections of bootloader
 *
 * bl_code_off        - Offset of code section in the image
 * bl_code_size          - Size of code section in the image
 * bl_data_off        - Offset of data section in the image
 * bl_data_size          - Size of data section in the image
 */
struct flcn_bl_img_hdr {
	u32 bl_code_off;
	u32 bl_code_size;
	u32 bl_data_off;
	u32 bl_data_size;
};

/*
 * The descriptor used by NVGPU to figure out the requirements of bootloader
 *
 * bl_start_tag - Starting tag of bootloader
 * bl_desc_dmem_load_off - Dmem offset where _def_rm_flcn_bl_dmem_desc
 * to be loaded
 * bl_img_hdr - Description of the image
 */
struct hsflcn_bl_desc {
	u32 bl_start_tag;
	u32 bl_desc_dmem_load_off;
	struct flcn_bl_img_hdr bl_img_hdr;
};

/*
 * Legacy structure used by the current PMU/DPU bootloader.
 */
struct loader_config {
	u32 dma_idx;
	u32 code_dma_base;     /* upper 32-bits of 40-bit dma address */
	u32 code_size_total;
	u32 code_size_to_load;
	u32 code_entry_point;
	u32 data_dma_base;     /* upper 32-bits of 40-bit dma address */
	u32 data_size;         /* initialized data of the application  */
	u32 overlay_dma_base;  /* upper 32-bits of the 40-bit dma address */
	u32 argc;
	u32 argv;
	u16 code_dma_base1;    /* upper 7 bits of 47-bit dma address */
	u16 data_dma_base1;    /* upper 7 bits of 47-bit dma address */
	u16 overlay_dma_base1; /* upper 7 bits of the 47-bit dma address */
};

struct loader_config_v1 {
	u32 reserved;
	u32 dma_idx;
	struct falc_u64 code_dma_base;
	u32 code_size_total;
	u32 code_size_to_load;
	u32 code_entry_point;
	struct falc_u64 data_dma_base;
	u32 data_size;
	struct falc_u64 overlay_dma_base;
	u32 argc;
	u32 argv;
};

/*
 * Union of all supported structures used by bootloaders.
 */
union flcn_bl_generic_desc {
	struct flcn_bl_dmem_desc bl_dmem_desc;
	struct loader_config loader_cfg;
};

union flcn_bl_generic_desc_v1 {
	struct flcn_bl_dmem_desc_v1 bl_dmem_desc_v1;
	struct loader_config_v1 loader_cfg_v1;
};

#endif /* __ACR_FLCNBL_H__ */
