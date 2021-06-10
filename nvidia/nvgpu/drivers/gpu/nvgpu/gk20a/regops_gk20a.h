/*
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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
#ifndef REGOPS_GK20A_H
#define REGOPS_GK20A_H

#include <uapi/linux/nvgpu.h>

struct regop_offset_range {
	u32 base:24;
	u32 count:8;
};

int exec_regops_gk20a(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_gpu_reg_op *ops,
		      u64 num_ops);

/* turn seriously unwieldy names -> something shorter */
#define REGOP(x) NVGPU_DBG_GPU_REG_OP_##x

static inline bool reg_op_is_gr_ctx(u8 type)
{
	return  type == REGOP(TYPE_GR_CTX) ||
		type == REGOP(TYPE_GR_CTX_TPC) ||
		type == REGOP(TYPE_GR_CTX_SM) ||
		type == REGOP(TYPE_GR_CTX_CROP) ||
		type == REGOP(TYPE_GR_CTX_ZROP) ||
		type == REGOP(TYPE_GR_CTX_QUAD);
}
static inline bool reg_op_is_read(u8 op)
{
	return  op == REGOP(READ_32) ||
		op == REGOP(READ_64) ;
}

bool is_bar0_global_offset_whitelisted_gk20a(struct gk20a *g, u32 offset);

void gk20a_init_regops(struct gpu_ops *gops);
#endif /* REGOPS_GK20A_H */
