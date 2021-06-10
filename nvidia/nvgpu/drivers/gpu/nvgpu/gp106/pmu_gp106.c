/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>	/* for udelay */

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"

#include "gm20b/pmu_gm20b.h"
#include "gp10b/pmu_gp10b.h"
#include "gp106/pmu_gp106.h"
#include "gp106/acr_gp106.h"

#include "clk/clk_mclk.h"

#include "lpwr/lpwr.h"
#include "lpwr/rppg.h"

#include <nvgpu/hw/gp106/hw_psec_gp106.h>
#include <nvgpu/hw/gp106/hw_mc_gp106.h>
#include <nvgpu/hw/gp106/hw_pwr_gp106.h>

#define PMU_MEM_SCRUBBING_TIMEOUT_MAX 1000
#define PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT 10

static int gp106_pmu_enable_hw(struct pmu_gk20a *pmu, bool enable)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	gk20a_dbg_fn("");

	/*
	* From GP10X onwards, we are using PPWR_FALCON_ENGINE for reset. And as
	* it may come into same behaviour, reading NV_PPWR_FALCON_ENGINE again
	* after Reset.
	*/

	if (enable) {
		int retries = PMU_MEM_SCRUBBING_TIMEOUT_MAX /
				PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT;
		gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_false_f());
		gk20a_readl(g, pwr_falcon_engine_r());

		/* make sure ELPG is in a good state */
		if (g->ops.clock_gating.slcg_pmu_load_gating_prod)
			g->ops.clock_gating.slcg_pmu_load_gating_prod(g,
					g->slcg_enabled);
		if (g->ops.clock_gating.blcg_pmu_load_gating_prod)
			g->ops.clock_gating.blcg_pmu_load_gating_prod(g,
					g->blcg_enabled);

		/* wait for Scrubbing to complete */
		do {
			u32 w = gk20a_readl(g, pwr_falcon_dmactl_r()) &
				(pwr_falcon_dmactl_dmem_scrubbing_m() |
				 pwr_falcon_dmactl_imem_scrubbing_m());

			if (!w) {
				gk20a_dbg_fn("done");
				return 0;
			}
			udelay(PMU_MEM_SCRUBBING_TIMEOUT_DEFAULT);
		} while (--retries || !tegra_platform_is_silicon());

		/* If scrubbing timeout, keep PMU in reset state */
		gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
		gk20a_readl(g, pwr_falcon_engine_r());
		gk20a_err(dev_from_gk20a(g), "Falcon mem scrubbing timeout");
		return -ETIMEDOUT;
	} else {
		/* DISBALE */
		gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
		gk20a_readl(g, pwr_falcon_engine_r());
		return 0;
	}
}

static int pmu_enable(struct pmu_gk20a *pmu, bool enable)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 reg_reset;
	int err;

	gk20a_dbg_fn("");

	if (!enable) {
		reg_reset = gk20a_readl(g, pwr_falcon_engine_r());
		if (reg_reset !=
			pwr_falcon_engine_reset_true_f()) {

			pmu_enable_irq(pmu, false);
			gp106_pmu_enable_hw(pmu, false);
			udelay(10);
		}
	} else {
		gp106_pmu_enable_hw(pmu, true);
		/* TBD: post reset */

		/*idle the PMU and enable interrupts on the Falcon*/
		err = pmu_idle(pmu);
		if (err)
			return err;
		udelay(5);
		pmu_enable_irq(pmu, true);
	}

	gk20a_dbg_fn("done");
	return 0;
}

int gp106_pmu_reset(struct gk20a *g)
{
	struct pmu_gk20a *pmu = &g->pmu;
	int err = 0;

	gk20a_dbg_fn("");

	err = pmu_idle(pmu);
	if (err)
		return err;

	/* TBD: release pmu hw mutex */

	err = pmu_enable(pmu, false);
	if (err)
		return err;

	/* TBD: cancel all sequences */
	/* TBD: init all sequences and state tables */
	/* TBD: restore pre-init message handler */

	err = pmu_enable(pmu, true);
	if (err)
		return err;

	return err;
}

static int gp106_sec2_reset(struct gk20a *g)
{
	gk20a_dbg_fn("");
	//sec2 reset
	gk20a_writel(g, psec_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
	udelay(10);
	gk20a_writel(g, psec_falcon_engine_r(),
			pwr_falcon_engine_reset_false_f());

	gk20a_dbg_fn("done");
	return 0;
}

static int gp106_falcon_reset(struct gk20a *g)
{
	gk20a_dbg_fn("");

	gp106_pmu_reset(g);
	gp106_sec2_reset(g);

	gk20a_dbg_fn("done");
	return 0;
}

static bool gp106_is_pmu_supported(struct gk20a *g)
{
	return true;
}

static u32 gp106_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id)
{
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS)
		return PMU_PG_FEATURE_GR_RPPG_ENABLED;

	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS)
		return NVGPU_PMU_MS_FEATURE_MASK_ALL;

	return 0;
}

static u32 gp106_pmu_pg_engines_list(struct gk20a *g)
{
	return BIT(PMU_PG_ELPG_ENGINE_ID_GRAPHICS) |
			BIT(PMU_PG_ELPG_ENGINE_ID_MS);
}

static void pmu_handle_param_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	gk20a_dbg_fn("");

	if (status != 0) {
		gk20a_err(dev_from_gk20a(g), "PG PARAM cmd aborted");
		return;
	}

	gp106_dbg_pmu("PG PARAM is acknowledged from PMU %x",
			msg->msg.pg.msg_type);
}

static int gp106_pg_param_init(struct gk20a *g, u32 pg_engine_id)
{
	struct pmu_gk20a *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;
	u32 status;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {

		status = init_rppg(g);
		if (status != 0) {
			gk20a_err(dev_from_gk20a(g), "RPPG init Failed");
			return -1;
		}

		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_pg_cmd_gr_init_param);
		cmd.cmd.pg.gr_init_param.cmd_type =
				PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.gr_init_param.sub_cmd_id =
				PMU_PG_PARAM_CMD_GR_INIT_PARAM;
		cmd.cmd.pg.gr_init_param.featuremask =
				PMU_PG_FEATURE_GR_RPPG_ENABLED;

		gp106_dbg_pmu("cmd post GR PMU_PG_CMD_ID_PG_PARAM");
		gk20a_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_param_msg, pmu, &seq, ~0);
	} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
			sizeof(struct pmu_pg_cmd_ms_init_param);
		cmd.cmd.pg.ms_init_param.cmd_type =
			PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.ms_init_param.cmd_id =
			PMU_PG_PARAM_CMD_MS_INIT_PARAM;
		cmd.cmd.pg.ms_init_param.support_mask =
			NVGPU_PMU_MS_FEATURE_MASK_CLOCK_GATING |
			NVGPU_PMU_MS_FEATURE_MASK_SW_ASR |
			NVGPU_PMU_MS_FEATURE_MASK_RPPG |
			NVGPU_PMU_MS_FEATURE_MASK_FB_TRAINING;

		gp106_dbg_pmu("cmd post MS PMU_PG_CMD_ID_PG_PARAM");
		gk20a_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_param_msg, pmu, &seq, ~0);
	}

	return 0;
}

static void gp106_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data)
{
	struct pmu_gk20a *pmu = &g->pmu;
	struct pmu_pg_stats_v2 stats;

	pmu_copy_from_dmem(pmu,
		pmu->stat_dmem_offset[pg_engine_id],
		(u8 *)&stats, sizeof(struct pmu_pg_stats_v2), 0);

	pg_stat_data->ingating_time = stats.total_sleep_time_us;
	pg_stat_data->ungating_time = stats.total_non_sleep_time_us;
	pg_stat_data->gating_cnt = stats.entry_count;
	pg_stat_data->avg_entry_latency_us = stats.entry_latency_avg_us;
	pg_stat_data->avg_exit_latency_us = stats.exit_latency_avg_us;
}

static bool gp106_pmu_is_lpwr_feature_supported(struct gk20a *g, u32 feature_id)
{
	bool is_feature_supported = false;

	switch (feature_id) {
	case PMU_PG_LPWR_FEATURE_RPPG:
		is_feature_supported = nvgpu_lpwr_is_rppg_supported(g,
			nvgpu_clk_arb_get_current_pstate(g));
		break;
	case PMU_PG_LPWR_FEATURE_MSCG:
		is_feature_supported = nvgpu_lpwr_is_mscg_supported(g,
			nvgpu_clk_arb_get_current_pstate(g));
		break;
	default:
		is_feature_supported = false;
	}

	return is_feature_supported;
}

static bool gp106_is_lazy_bootstrap(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = true;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

static bool gp106_is_priv_load(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = true;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

static void gp106_pmu_load_multiple_falcons(struct gk20a *g, u32 falconidmask,
					 u32 flags)
{
	struct pmu_gk20a *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	gk20a_dbg_fn("");

	gp106_dbg_pmu("wprinit status = %x\n", g->ops.pmu.lspmuwprinitdone);
	if (g->ops.pmu.lspmuwprinitdone) {
		/* send message to load FECS falcon */
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_ACR;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
		  sizeof(struct pmu_acr_cmd_bootstrap_multiple_falcons);
		cmd.cmd.acr.boot_falcons.cmd_type =
		  PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS;
		cmd.cmd.acr.boot_falcons.flags = flags;
		cmd.cmd.acr.boot_falcons.falconidmask =
				falconidmask;
		cmd.cmd.acr.boot_falcons.usevamask = 0;
		cmd.cmd.acr.boot_falcons.wprvirtualbase.lo = 0;
		cmd.cmd.acr.boot_falcons.wprvirtualbase.hi = 0;

		gp106_dbg_pmu("PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS:%x\n",
				falconidmask);
		gk20a_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_fecs_boot_acr_msg, pmu, &seq, ~0);
	}

	gk20a_dbg_fn("done");
}

static int gp106_load_falcon_ucode(struct gk20a *g, u32 falconidmask)
{
	u32 flags = PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES;

	/* GM20B PMU supports loading FECS and GPCCS only */
	if (falconidmask == 0)
		return -EINVAL;
	if (falconidmask & ~((1 << LSF_FALCON_ID_FECS) |
			(1 << LSF_FALCON_ID_GPCCS)))
		return -EINVAL;
	g->ops.pmu.lsfloadedfalconid = 0;
	/* check whether pmu is ready to bootstrap lsf if not wait for it */
	if (!g->ops.pmu.lspmuwprinitdone) {
		pmu_wait_message_cond(&g->pmu,
				gk20a_get_gr_idle_timeout(g),
				&g->ops.pmu.lspmuwprinitdone, 1);
		/* check again if it still not ready indicate an error */
		if (!g->ops.pmu.lspmuwprinitdone) {
			gk20a_err(dev_from_gk20a(g),
				"PMU not ready to load LSF");
			return -ETIMEDOUT;
		}
	}
	/* load falcon(s) */
	gp106_pmu_load_multiple_falcons(g, falconidmask, flags);
	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&g->ops.pmu.lsfloadedfalconid, falconidmask);
	if (g->ops.pmu.lsfloadedfalconid != falconidmask)
		return -ETIMEDOUT;
	return 0;
}

void gp106_init_pmu_ops(struct gpu_ops *gops)
{
	gk20a_dbg_fn("");

	if (gops->privsecurity) {
		gp106_init_secure_pmu(gops);
		gops->pmu.init_wpr_region = gm20b_pmu_init_acr;
		gops->pmu.load_lsfalcon_ucode = gp106_load_falcon_ucode;
		gops->pmu.is_lazy_bootstrap = gp106_is_lazy_bootstrap;
		gops->pmu.is_priv_load = gp106_is_priv_load;
	} else {
		gk20a_init_pmu_ops(gops);
		gops->pmu.pmu_setup_hw_and_bootstrap =
			gm20b_init_nspmu_setup_hw1;
		gops->pmu.load_lsfalcon_ucode = NULL;
		gops->pmu.init_wpr_region = NULL;
	}
	gops->pmu.pmu_setup_elpg = NULL;
	gops->pmu.pmu_get_queue_head = pwr_pmu_queue_head_r;
	gops->pmu.pmu_get_queue_head_size = pwr_pmu_queue_head__size_1_v;
	gops->pmu.pmu_get_queue_tail = pwr_pmu_queue_tail_r;
	gops->pmu.pmu_get_queue_tail_size = pwr_pmu_queue_tail__size_1_v;
	gops->pmu.lspmuwprinitdone = 0;
	gops->pmu.fecsbootstrapdone = false;
	gops->pmu.write_dmatrfbase = gp10b_write_dmatrfbase;
	gops->pmu.pmu_elpg_statistics = gp106_pmu_elpg_statistics;
	gops->pmu.pmu_pg_init_param = gp106_pg_param_init;
	gops->pmu.pmu_pg_supported_engines_list = gp106_pmu_pg_engines_list;
	gops->pmu.pmu_pg_engines_feature_list = gp106_pmu_pg_feature_list;
	gops->pmu.pmu_is_lpwr_feature_supported =
		gp106_pmu_is_lpwr_feature_supported;
	gops->pmu.pmu_lpwr_enable_pg = nvgpu_lpwr_enable_pg;
	gops->pmu.pmu_lpwr_disable_pg = nvgpu_lpwr_disable_pg;
	gops->pmu.pmu_pg_param_post_init = nvgpu_lpwr_post_init;
	gops->pmu.send_lrf_tex_ltc_dram_overide_en_dis_cmd = NULL;
	gops->pmu.dump_secure_fuses = NULL;
	gops->pmu.reset = gp106_falcon_reset;
	gops->pmu.mclk_init = clk_mclkseq_init_mclk_gddr5;
	gops->pmu.is_pmu_supported = gp106_is_pmu_supported;

	gk20a_dbg_fn("done");
}
