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
#ifndef _GPMUIFPMU_H_
#define _GPMUIFPMU_H_

#include <nvgpu/flcnif_cmn.h>
#include "gpmuif_cmn.h"

/* Make sure size of this structure is a multiple of 4 bytes */
struct pmu_cmdline_args_v0 {
	u32 cpu_freq_hz;
	u32 falc_trace_size;
	u32 falc_trace_dma_base;
	u32 falc_trace_dma_idx;
	struct pmu_mem_v0 gc6_ctx;
};

struct pmu_cmdline_args_v1 {
	u32 cpu_freq_hz;
	u32 falc_trace_size;
	u32 falc_trace_dma_base;
	u32 falc_trace_dma_idx;
	u8 secure_mode;
	struct pmu_mem_v1 gc6_ctx;
};

struct pmu_cmdline_args_v2 {
	u32 cpu_freq_hz;
	u32 falc_trace_size;
	u32 falc_trace_dma_base;
	u32 falc_trace_dma_idx;
	u8 secure_mode;
	u8 raise_priv_sec;
	struct pmu_mem_v1 gc6_ctx;
};

struct pmu_cmdline_args_v3 {
	u32 reserved;
	u32 cpu_freq_hz;
	u32 falc_trace_size;
	u32 falc_trace_dma_base;
	u32 falc_trace_dma_idx;
	u8 secure_mode;
	u8 raise_priv_sec;
	struct pmu_mem_v1 gc6_ctx;
};

struct pmu_cmdline_args_v4 {
	u32 reserved;
	u32 cpu_freq_hz;
	u32 falc_trace_size;
	struct falc_dma_addr dma_addr;
	u32 falc_trace_dma_idx;
	u8 secure_mode;
	u8 raise_priv_sec;
	struct pmu_mem_desc_v0 gc6_ctx;
	u8 pad;
};

struct pmu_cmdline_args_v5 {
	u32 cpu_freq_hz;
	struct flcn_mem_desc_v0 trace_buf;
	u8 secure_mode;
	u8 raise_priv_sec;
	struct flcn_mem_desc_v0 gc6_ctx;
	struct flcn_mem_desc_v0 init_data_dma_info;
	u32 dummy;
};

/* GPU ID */
#define PMU_SHA1_GID_SIGNATURE		0xA7C66AD2
#define PMU_SHA1_GID_SIGNATURE_SIZE	4

#define PMU_SHA1_GID_SIZE	16

struct pmu_sha1_gid {
	bool valid;
	u8 gid[PMU_SHA1_GID_SIZE];
};

struct pmu_sha1_gid_data {
	u8 signature[PMU_SHA1_GID_SIGNATURE_SIZE];
	u8 gid[PMU_SHA1_GID_SIZE];
};

/* PMU INIT MSG */
enum {
	PMU_INIT_MSG_TYPE_PMU_INIT = 0,
};

struct pmu_init_msg_pmu_v0 {
	u8 msg_type;
	u8 pad;

	struct {
		u16 size;
		u16 offset;
		u8  index;
		u8  pad;
	} queue_info[PMU_QUEUE_COUNT];

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;
};

struct pmu_init_msg_pmu_v1 {
	u8 msg_type;
	u8 pad;
	u16  os_debug_entry_point;

	struct {
		u16 size;
		u16 offset;
		u8  index;
		u8  pad;
	} queue_info[PMU_QUEUE_COUNT];

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;
};
struct pmu_init_msg_pmu_v2 {
	u8 msg_type;
	u8 pad;
	u16  os_debug_entry_point;

	struct {
		u16 size;
		u16 offset;
		u8  index;
		u8  pad;
	} queue_info[PMU_QUEUE_COUNT];

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;
	u8 dummy[18];
};

#define PMU_QUEUE_COUNT_FOR_V4 5
#define PMU_QUEUE_COUNT_FOR_V3 3
#define PMU_QUEUE_HPQ_IDX_FOR_V3 0
#define PMU_QUEUE_LPQ_IDX_FOR_V3 1
#define PMU_QUEUE_MSG_IDX_FOR_V3 2
struct pmu_init_msg_pmu_v3 {
	u8 msg_type;
	u8  queue_index[PMU_QUEUE_COUNT_FOR_V3];
	u16 queue_size[PMU_QUEUE_COUNT_FOR_V3];
	u16 queue_offset;

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;

	u16  os_debug_entry_point;

	u8 dummy[18];
};

struct pmu_init_msg_pmu_v4 {
	u8 msg_type;
	u8  queue_index[PMU_QUEUE_COUNT_FOR_V4];
	u16 queue_size[PMU_QUEUE_COUNT_FOR_V4];
	u16 queue_offset;

	u16 sw_managed_area_offset;
	u16 sw_managed_area_size;

	u16  os_debug_entry_point;

	u8 dummy[18];
};

union pmu_init_msg_pmu {
	struct pmu_init_msg_pmu_v0 v0;
	struct pmu_init_msg_pmu_v1 v1;
	struct pmu_init_msg_pmu_v2 v2;
	struct pmu_init_msg_pmu_v3 v3;
	struct pmu_init_msg_pmu_v4 v4;
};

struct pmu_init_msg {
	union {
		u8 msg_type;
		struct pmu_init_msg_pmu_v1 pmu_init_v1;
		struct pmu_init_msg_pmu_v0 pmu_init_v0;
		struct pmu_init_msg_pmu_v2 pmu_init_v2;
		struct pmu_init_msg_pmu_v3 pmu_init_v3;
		struct pmu_init_msg_pmu_v4 pmu_init_v4;
	};
};

/* robust channel (RC) messages */
enum {
	PMU_RC_MSG_TYPE_UNHANDLED_CMD = 0,
};

struct pmu_rc_msg_unhandled_cmd {
	u8 msg_type;
	u8 unit_id;
};

struct pmu_rc_msg {
	u8 msg_type;
	struct pmu_rc_msg_unhandled_cmd unhandled_cmd;
};

#endif /* _GPMUIFPMU_H_*/
