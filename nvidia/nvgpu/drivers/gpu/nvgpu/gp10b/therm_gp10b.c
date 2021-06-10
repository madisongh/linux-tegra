/*
 * drivers/gpu/nvgpu/gm20b/therm_gk20a.c
 *
 * GP10B Therm
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#include <nvgpu/hw/gp10b/hw_therm_gp10b.h>

static int gp10b_init_therm_setup_hw(struct gk20a *g)
{
	u32 v;

	gk20a_dbg_fn("");

	/* program NV_THERM registers */
	gk20a_writel(g, therm_use_a_r(), therm_use_a_ext_therm_0_enable_f() |
					therm_use_a_ext_therm_1_enable_f()  |
					therm_use_a_ext_therm_2_enable_f());
	gk20a_writel(g, therm_evt_ext_therm_0_r(),
		therm_evt_ext_therm_0_slow_factor_f(0x2));
	gk20a_writel(g, therm_evt_ext_therm_1_r(),
		therm_evt_ext_therm_1_slow_factor_f(0x6));
	gk20a_writel(g, therm_evt_ext_therm_2_r(),
		therm_evt_ext_therm_2_slow_factor_f(0xe));

	gk20a_writel(g, therm_grad_stepping_table_r(0),
		therm_grad_stepping_table_slowdown_factor0_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by1p5_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by2_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by4_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()));

	gk20a_writel(g, therm_grad_stepping_table_r(1),
		therm_grad_stepping_table_slowdown_factor0_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()));

	v = gk20a_readl(g, therm_clk_timing_r(0));
	v |= therm_clk_timing_grad_slowdown_enabled_f();
	gk20a_writel(g, therm_clk_timing_r(0), v);

	v = gk20a_readl(g, therm_config2_r());
	v |= therm_config2_grad_enable_f(1);
	v |= therm_config2_slowdown_factor_extended_f(1);
	gk20a_writel(g, therm_config2_r(), v);

	gk20a_writel(g, therm_grad_stepping1_r(),
			therm_grad_stepping1_pdiv_duration_f(32));

	v = gk20a_readl(g, therm_grad_stepping0_r());
	v |= therm_grad_stepping0_feature_enable_f();
	gk20a_writel(g, therm_grad_stepping0_r(), v);

	return 0;
}

static int gp10b_elcg_init_idle_filters(struct gk20a *g)
{
	u32 gate_ctrl, idle_filter;
	u32 engine_id;
	u32 active_engine_id = 0;
	struct fifo_gk20a *f = &g->fifo;

	gk20a_dbg_fn("");

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(active_engine_id));

		if (tegra_platform_is_linsim()) {
			gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_delay_after_m(),
				therm_gate_ctrl_eng_delay_after_f(4));
		}

		/* 2 * (1 << 9) = 1024 clks */
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_exp_m(),
			therm_gate_ctrl_eng_idle_filt_exp_f(9));
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_mant_m(),
			therm_gate_ctrl_eng_idle_filt_mant_f(2));
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_delay_before_m(),
			therm_gate_ctrl_eng_delay_before_f(4));
		gk20a_writel(g, therm_gate_ctrl_r(active_engine_id), gate_ctrl);
	}

	/* default fecs_idle_filter to 0 */
	idle_filter = gk20a_readl(g, therm_fecs_idle_filter_r());
	idle_filter &= ~therm_fecs_idle_filter_value_m();
	gk20a_writel(g, therm_fecs_idle_filter_r(), idle_filter);
	/* default hubmmu_idle_filter to 0 */
	idle_filter = gk20a_readl(g, therm_hubmmu_idle_filter_r());
	idle_filter &= ~therm_hubmmu_idle_filter_value_m();
	gk20a_writel(g, therm_hubmmu_idle_filter_r(), idle_filter);

	gk20a_dbg_fn("done");
	return 0;
}

void gp10b_init_therm_ops(struct gpu_ops *gops)
{
	gops->therm.init_therm_setup_hw = gp10b_init_therm_setup_hw;
	gops->therm.elcg_init_idle_filters = gp10b_elcg_init_idle_filters;
}
