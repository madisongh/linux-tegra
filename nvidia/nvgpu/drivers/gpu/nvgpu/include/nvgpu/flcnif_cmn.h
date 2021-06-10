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

#ifndef __FLCNIFCMN_H__
#define __FLCNIFCMN_H__

#define PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED 0

struct falc_u64 {
	u32 lo;
	u32 hi;
};

struct falc_dma_addr {
	u32 dma_base;
	/*
	 * dma_base1 is 9-bit MSB for FB Base
	 * address for the transfer in FB after
	 * address using 49b FB address
	 */
	u16 dma_base1;
	u8 dma_offset;
};

struct pmu_mem_v0 {
	u32 dma_base;
	u8  dma_offset;
	u8  dma_idx;
};

struct pmu_mem_v1 {
	u32 dma_base;
	u8  dma_offset;
	u8  dma_idx;
	u16 fb_size;
};

struct pmu_mem_v2 {
	struct falc_dma_addr dma_addr;
	u8  dma_idx;
	u16 fb_size;
};

struct pmu_mem_desc_v0 {
	struct falc_u64 dma_addr;
	u16       dma_sizemax;
	u8        dma_idx;
};

struct pmu_dmem {
	u16 size;
	u32 offset;
};

struct flcn_mem_desc_v0 {
	struct falc_u64 address;
	u32 params;
};

#define nv_flcn_mem_desc flcn_mem_desc_v0

struct pmu_allocation_v0 {
	u8 pad[3];
	u8 fb_mem_use;
	struct {
		struct pmu_dmem dmem;
		struct pmu_mem_v0 fb;
	} alloc;
};

struct pmu_allocation_v1 {
	struct {
		struct pmu_dmem dmem;
		struct pmu_mem_v1 fb;
	} alloc;
};

struct pmu_allocation_v2 {
	struct {
		struct pmu_dmem dmem;
		struct pmu_mem_desc_v0 fb;
	} alloc;
};

struct pmu_allocation_v3 {
	struct {
		struct pmu_dmem dmem;
		struct flcn_mem_desc_v0 fb;
	} alloc;
};

#define nv_pmu_allocation pmu_allocation_v3

struct pmu_hdr {
	u8 unit_id;
	u8 size;
	u8 ctrl_flags;
	u8 seq_id;
};

#define PMU_MSG_HDR_SIZE	sizeof(struct pmu_hdr)
#define PMU_CMD_HDR_SIZE	sizeof(struct pmu_hdr)

#define nv_pmu_hdr pmu_hdr
typedef u8 flcn_status;

#define PMU_DMEM_ALLOC_ALIGNMENT	(4)
#define PMU_DMEM_ALIGNMENT		(4)

#define PMU_CMD_FLAGS_PMU_MASK		(0xF0)

#define PMU_CMD_FLAGS_STATUS		BIT(0)
#define PMU_CMD_FLAGS_INTR		BIT(1)
#define PMU_CMD_FLAGS_EVENT		BIT(2)
#define PMU_CMD_FLAGS_WATERMARK		BIT(3)

#define ALIGN_UP(v, gran)       (((v) + ((gran) - 1)) & ~((gran)-1))

#define NV_UNSIGNED_ROUNDED_DIV(a, b)    (((a) + ((b) / 2)) / (b))

#endif /* _FLCNIFCMN_H_*/
