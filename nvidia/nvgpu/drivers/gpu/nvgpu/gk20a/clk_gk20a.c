/*
 * GK20A Clocks
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/delay.h>	/* for mdelay */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>

#include "gk20a.h"

#include <nvgpu/hw/gk20a/hw_trim_gk20a.h>
#include <nvgpu/hw/gk20a/hw_timer_gk20a.h>

#define gk20a_dbg_clk(fmt, arg...) \
	gk20a_dbg(gpu_dbg_clk, fmt, ##arg)

/* from vbios PLL info table */
static struct pll_parms gpc_pll_params = {
	144000, 2064000,	/* freq */
	1000000, 2064000,	/* vco */
	12000, 38000,		/* u */
	1, 255,			/* M */
	8, 255,			/* N */
	1, 32,			/* PL */
	0, 0, 0, 0, 0,		/* NA mode parameters: not supported on GK20A */
	500,			/* Locking and ramping timeout */
	0,			/* NA mode lock delay: not supported on GK20A */
	2,			/* IDDQ mode exit delay */
};

#ifdef CONFIG_DEBUG_FS
static int clk_gk20a_debugfs_init(struct gk20a *g);
#endif

static u8 pl_to_div[] = {
/* PL:   0, 1, 2, 3, 4, 5, 6,  7,  8,  9, 10, 11, 12, 13, 14 */
/* p: */ 1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 12, 16, 20, 24, 32 };

/* Calculate and update M/N/PL as well as pll->freq
    ref_clk_f = clk_in_f / src_div = clk_in_f; (src_div = 1 on gk20a)
    u_f = ref_clk_f / M;
    PLL output = vco_f = u_f * N = ref_clk_f * N / M;
    gpc2clk = target clock frequency = vco_f / PL;
    gpcclk = gpc2clk / 2; */
static int clk_config_pll(struct clk_gk20a *clk, struct pll *pll,
	struct pll_parms *pll_params, u32 *target_freq, bool best_fit)
{
	u32 min_vco_f, max_vco_f;
	u32 best_M, best_N;
	u32 low_PL, high_PL, best_PL;
	u32 m, n, n2;
	u32 target_vco_f, vco_f;
	u32 ref_clk_f, target_clk_f, u_f;
	u32 delta, lwv, best_delta = ~0;
	unsigned int pl;

	BUG_ON(target_freq == NULL);

	gk20a_dbg_fn("request target freq %d MHz", *target_freq);

	ref_clk_f = pll->clk_in;
	target_clk_f = *target_freq;
	max_vco_f = pll_params->max_vco;
	min_vco_f = pll_params->min_vco;
	best_M = pll_params->max_M;
	best_N = pll_params->min_N;
	best_PL = pll_params->min_PL;

	target_vco_f = target_clk_f + target_clk_f / 50;
	if (max_vco_f < target_vco_f)
		max_vco_f = target_vco_f;

	high_PL = (max_vco_f + target_vco_f - 1) / target_vco_f;
	high_PL = min(high_PL, pll_params->max_PL);
	high_PL = max(high_PL, pll_params->min_PL);

	low_PL = min_vco_f / target_vco_f;
	low_PL = min(low_PL, pll_params->max_PL);
	low_PL = max(low_PL, pll_params->min_PL);

	/* Find Indices of high_PL and low_PL */
	for (pl = 0; pl < 14; pl++) {
		if (pl_to_div[pl] >= low_PL) {
			low_PL = pl;
			break;
		}
	}
	for (pl = 0; pl < 14; pl++) {
		if (pl_to_div[pl] >= high_PL) {
			high_PL = pl;
			break;
		}
	}
	gk20a_dbg_info("low_PL %d(div%d), high_PL %d(div%d)",
			low_PL, pl_to_div[low_PL], high_PL, pl_to_div[high_PL]);

	for (pl = low_PL; pl <= high_PL; pl++) {
		target_vco_f = target_clk_f * pl_to_div[pl];

		for (m = pll_params->min_M; m <= pll_params->max_M; m++) {
			u_f = ref_clk_f / m;

			if (u_f < pll_params->min_u)
				break;
			if (u_f > pll_params->max_u)
				continue;

			n = (target_vco_f * m) / ref_clk_f;
			n2 = ((target_vco_f * m) + (ref_clk_f - 1)) / ref_clk_f;

			if (n > pll_params->max_N)
				break;

			for (; n <= n2; n++) {
				if (n < pll_params->min_N)
					continue;
				if (n > pll_params->max_N)
					break;

				vco_f = ref_clk_f * n / m;

				if (vco_f >= min_vco_f && vco_f <= max_vco_f) {
					lwv = (vco_f + (pl_to_div[pl] / 2))
						/ pl_to_div[pl];
					delta = abs(lwv - target_clk_f);

					if (delta < best_delta) {
						best_delta = delta;
						best_M = m;
						best_N = n;
						best_PL = pl;

						if (best_delta == 0 ||
						    /* 0.45% for non best fit */
						    (!best_fit && (vco_f / best_delta > 218))) {
							goto found_match;
						}

						gk20a_dbg_info("delta %d @ M %d, N %d, PL %d",
							delta, m, n, pl);
					}
				}
			}
		}
	}

found_match:
	BUG_ON(best_delta == ~0U);

	if (best_fit && best_delta != 0)
		gk20a_dbg_clk("no best match for target @ %dMHz on gpc_pll",
			target_clk_f);

	pll->M = best_M;
	pll->N = best_N;
	pll->PL = best_PL;

	/* save current frequency */
	pll->freq = ref_clk_f * pll->N / (pll->M * pl_to_div[pll->PL]);

	*target_freq = pll->freq;

	gk20a_dbg_clk("actual target freq %d MHz, M %d, N %d, PL %d(div%d)",
		*target_freq, pll->M, pll->N, pll->PL, pl_to_div[pll->PL]);

	gk20a_dbg_fn("done");

	return 0;
}

static int clk_slide_gpc_pll(struct gk20a *g, u32 n)
{
	u32 data, coeff;
	u32 nold;
	int ramp_timeout = gpc_pll_params.lock_timeout;

	/* get old coefficients */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	nold = trim_sys_gpcpll_coeff_ndiv_v(coeff);

	/* do nothing if NDIV is same */
	if (n == nold)
		return 0;

	/* setup */
	data = gk20a_readl(g, trim_sys_gpcpll_cfg2_r());
	data = set_field(data, trim_sys_gpcpll_cfg2_pll_stepa_m(),
			trim_sys_gpcpll_cfg2_pll_stepa_f(0x2b));
	gk20a_writel(g, trim_sys_gpcpll_cfg2_r(), data);
	data = gk20a_readl(g, trim_sys_gpcpll_cfg3_r());
	data = set_field(data, trim_sys_gpcpll_cfg3_pll_stepb_m(),
			trim_sys_gpcpll_cfg3_pll_stepb_f(0xb));
	gk20a_writel(g, trim_sys_gpcpll_cfg3_r(), data);

	/* pll slowdown mode */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(),
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_yes_f());
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);

	/* new ndiv ready for ramp */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	coeff = set_field(coeff, trim_sys_gpcpll_coeff_ndiv_m(),
			trim_sys_gpcpll_coeff_ndiv_f(n));
	udelay(1);
	gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);

	/* dynamic ramp to new ndiv */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(),
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_yes_f());
	udelay(1);
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);

	do {
		udelay(1);
		ramp_timeout--;
		data = gk20a_readl(
			g, trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_r());
		if (trim_gpc_bcast_gpcpll_ndiv_slowdown_debug_pll_dynramp_done_synced_v(data))
			break;
	} while (ramp_timeout > 0);

	/* exit slowdown mode */
	data = gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_m(),
			trim_sys_gpcpll_ndiv_slowdown_slowdown_using_pll_no_f());
	data = set_field(data,
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_m(),
			trim_sys_gpcpll_ndiv_slowdown_en_dynramp_no_f());
	gk20a_writel(g, trim_sys_gpcpll_ndiv_slowdown_r(), data);
	gk20a_readl(g, trim_sys_gpcpll_ndiv_slowdown_r());

	if (ramp_timeout <= 0) {
		gk20a_err(dev_from_gk20a(g), "gpcpll dynamic ramp timeout");
		return -ETIMEDOUT;
	}
	return 0;
}

static int clk_program_gpc_pll(struct gk20a *g, struct clk_gk20a *clk,
			int allow_slide)
{
	u32 data, cfg, coeff, timeout;
	u32 m, n, pl;
	u32 nlo;

	gk20a_dbg_fn("");

	if (!tegra_platform_is_silicon())
		return 0;

	/* get old coefficients */
	coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	m = trim_sys_gpcpll_coeff_mdiv_v(coeff);
	n = trim_sys_gpcpll_coeff_ndiv_v(coeff);
	pl = trim_sys_gpcpll_coeff_pldiv_v(coeff);

	/* do NDIV slide if there is no change in M and PL */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (allow_slide && clk->gpc_pll.M == m && clk->gpc_pll.PL == pl
		&& trim_sys_gpcpll_cfg_enable_v(cfg)) {
		return clk_slide_gpc_pll(g, clk->gpc_pll.N);
	}

	/* slide down to NDIV_LO */
	nlo = DIV_ROUND_UP(m * gpc_pll_params.min_vco, clk->gpc_pll.clk_in);
	if (allow_slide && trim_sys_gpcpll_cfg_enable_v(cfg)) {
		int ret = clk_slide_gpc_pll(g, nlo);
		if (ret)
			return ret;
	}

	/* split FO-to-bypass jump in halfs by setting out divider 1:2 */
	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data, trim_sys_gpc2clk_out_vcodiv_m(),
		trim_sys_gpc2clk_out_vcodiv_f(2));
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);

	/* put PLL in bypass before programming it */
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_bypass_f());
	udelay(2);
	gk20a_writel(g, trim_sys_sel_vco_r(), data);

	/* get out from IDDQ */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (trim_sys_gpcpll_cfg_iddq_v(cfg)) {
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_iddq_m(),
				trim_sys_gpcpll_cfg_iddq_power_on_v());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
		gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		udelay(gpc_pll_params.iddq_exit_delay);
	}

	/* disable PLL before changing coefficients */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
			trim_sys_gpcpll_cfg_enable_no_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	gk20a_readl(g, trim_sys_gpcpll_cfg_r());

	/* change coefficients */
	nlo = DIV_ROUND_UP(clk->gpc_pll.M * gpc_pll_params.min_vco,
			clk->gpc_pll.clk_in);
	coeff = trim_sys_gpcpll_coeff_mdiv_f(clk->gpc_pll.M) |
		trim_sys_gpcpll_coeff_ndiv_f(allow_slide ?
					     nlo : clk->gpc_pll.N) |
		trim_sys_gpcpll_coeff_pldiv_f(clk->gpc_pll.PL);
	gk20a_writel(g, trim_sys_gpcpll_coeff_r(), coeff);

	/* enable PLL after changing coefficients */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
			trim_sys_gpcpll_cfg_enable_yes_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);

	/* lock pll */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (cfg & trim_sys_gpcpll_cfg_enb_lckdet_power_off_f()){
		cfg = set_field(cfg, trim_sys_gpcpll_cfg_enb_lckdet_m(),
			trim_sys_gpcpll_cfg_enb_lckdet_power_on_f());
		gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	}

	/* wait pll lock */
	timeout = gpc_pll_params.lock_timeout / 2 + 1;
	do {
		cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
		if (cfg & trim_sys_gpcpll_cfg_pll_lock_true_f())
			goto pll_locked;
		udelay(2);
	} while (--timeout > 0);

	/* PLL is messed up. What can we do here? */
	BUG();
	return -EBUSY;

pll_locked:
	/* put PLL back on vco */
	data = gk20a_readl(g, trim_sys_sel_vco_r());
	data = set_field(data, trim_sys_sel_vco_gpc2clk_out_m(),
		trim_sys_sel_vco_gpc2clk_out_vco_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), data);
	clk->gpc_pll.enabled = true;

	/* restore out divider 1:1 */
	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data, trim_sys_gpc2clk_out_vcodiv_m(),
		trim_sys_gpc2clk_out_vcodiv_by1_f());
	udelay(2);
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);

	/* slide up to target NDIV */
	return clk_slide_gpc_pll(g, clk->gpc_pll.N);
}

static int clk_disable_gpcpll(struct gk20a *g, int allow_slide)
{
	u32 cfg, coeff, m, nlo;
	struct clk_gk20a *clk = &g->clk;

	/* slide to VCO min */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	if (allow_slide && trim_sys_gpcpll_cfg_enable_v(cfg)) {
		coeff = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
		m = trim_sys_gpcpll_coeff_mdiv_v(coeff);
		nlo = DIV_ROUND_UP(m * gpc_pll_params.min_vco,
				   clk->gpc_pll.clk_in);
		clk_slide_gpc_pll(g, nlo);
	}

	/* put PLL in bypass before disabling it */
	cfg = gk20a_readl(g, trim_sys_sel_vco_r());
	cfg = set_field(cfg, trim_sys_sel_vco_gpc2clk_out_m(),
			trim_sys_sel_vco_gpc2clk_out_bypass_f());
	gk20a_writel(g, trim_sys_sel_vco_r(), cfg);

	/* disable PLL */
	cfg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	cfg = set_field(cfg, trim_sys_gpcpll_cfg_enable_m(),
			trim_sys_gpcpll_cfg_enable_no_f());
	gk20a_writel(g, trim_sys_gpcpll_cfg_r(), cfg);
	gk20a_readl(g, trim_sys_gpcpll_cfg_r());

	clk->gpc_pll.enabled = false;
	return 0;
}

static int gk20a_init_clk_reset_enable_hw(struct gk20a *g)
{
	gk20a_dbg_fn("");
	return 0;
}

static int gk20a_init_clk_setup_sw(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	static int initialized;
	struct clk *ref;
	unsigned long ref_rate;

	gk20a_dbg_fn("");

	if (clk->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	if (!gk20a_clk_get(g))
		return -EINVAL;

	ref = clk_get_parent(clk_get_parent(clk->tegra_clk));
	if (IS_ERR(ref)) {
		gk20a_err(dev_from_gk20a(g),
			"failed to get GPCPLL reference clock");
		return -EINVAL;
	}
	ref_rate = clk_get_rate(ref);

	clk->gpc_pll.id = GK20A_GPC_PLL;
	clk->gpc_pll.clk_in = ref_rate / KHZ;
	if (clk->gpc_pll.clk_in == 0) {
		gk20a_err(dev_from_gk20a(g),
			"GPCPLL reference clock is zero");
		return -EINVAL;
	}

	/* Decide initial frequency */
	if (!initialized) {
		initialized = 1;
		clk->gpc_pll.M = 1;
		clk->gpc_pll.N = DIV_ROUND_UP(gpc_pll_params.min_vco,
					clk->gpc_pll.clk_in);
		clk->gpc_pll.PL = 1;
		clk->gpc_pll.freq = clk->gpc_pll.clk_in * clk->gpc_pll.N;
		clk->gpc_pll.freq /= pl_to_div[clk->gpc_pll.PL];
	}

	nvgpu_mutex_init(&clk->clk_mutex);

	clk->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;
}

static int gk20a_init_clk_setup_hw(struct gk20a *g)
{
	u32 data;

	gk20a_dbg_fn("");

	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data,
			trim_sys_gpc2clk_out_sdiv14_m() |
			trim_sys_gpc2clk_out_vcodiv_m() |
			trim_sys_gpc2clk_out_bypdiv_m(),
			trim_sys_gpc2clk_out_sdiv14_indiv4_mode_f() |
			trim_sys_gpc2clk_out_vcodiv_by1_f() |
			trim_sys_gpc2clk_out_bypdiv_f(0));
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);

	return 0;
}

static int set_pll_target(struct gk20a *g, u32 freq, u32 old_freq)
{
	struct clk_gk20a *clk = &g->clk;

	if (freq > gpc_pll_params.max_freq)
		freq = gpc_pll_params.max_freq;
	else if (freq < gpc_pll_params.min_freq)
		freq = gpc_pll_params.min_freq;

	if (freq != old_freq) {
		/* gpc_pll.freq is changed to new value here */
		if (clk_config_pll(clk, &clk->gpc_pll, &gpc_pll_params,
				   &freq, true)) {
			gk20a_err(dev_from_gk20a(g),
				   "failed to set pll target for %d", freq);
			return -EINVAL;
		}
	}
	return 0;
}

static int set_pll_freq(struct gk20a *g, u32 freq, u32 old_freq)
{
	struct clk_gk20a *clk = &g->clk;
	int err = 0;

	gk20a_dbg_fn("curr freq: %dMHz, target freq %dMHz", old_freq, freq);

	if ((freq == old_freq) && clk->gpc_pll.enabled)
		return 0;

	/* change frequency only if power is on */
	if (g->clk.clk_hw_on) {
		err = clk_program_gpc_pll(g, clk, 1);
		if (err)
			err = clk_program_gpc_pll(g, clk, 0);
	}

	/* Just report error but not restore PLL since dvfs could already change
	    voltage even when it returns error. */
	if (err)
		gk20a_err(dev_from_gk20a(g),
			"failed to set pll to %d", freq);
	return err;
}

static int gk20a_clk_export_set_rate(void *data, unsigned long *rate)
{
	u32 old_freq;
	int ret = -ENODATA;
	struct gk20a *g = data;
	struct clk_gk20a *clk = &g->clk;

	if (rate) {
		nvgpu_mutex_acquire(&clk->clk_mutex);
		old_freq = clk->gpc_pll.freq;
		ret = set_pll_target(g, rate_gpu_to_gpc2clk(*rate), old_freq);
		if (!ret && clk->gpc_pll.enabled)
			ret = set_pll_freq(g, clk->gpc_pll.freq, old_freq);
		if (!ret)
			*rate = rate_gpc2clk_to_gpu(clk->gpc_pll.freq);
		nvgpu_mutex_release(&clk->clk_mutex);
	}
	return ret;
}

static int gk20a_clk_export_enable(void *data)
{
	int ret;
	struct gk20a *g = data;
	struct clk_gk20a *clk = &g->clk;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	ret = set_pll_freq(g, clk->gpc_pll.freq, clk->gpc_pll.freq);
	nvgpu_mutex_release(&clk->clk_mutex);
	return ret;
}

static void gk20a_clk_export_disable(void *data)
{
	struct gk20a *g = data;
	struct clk_gk20a *clk = &g->clk;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	if (g->clk.clk_hw_on)
		clk_disable_gpcpll(g, 1);
	nvgpu_mutex_release(&clk->clk_mutex);
}

static void gk20a_clk_export_init(void *data, unsigned long *rate, bool *state)
{
	struct gk20a *g = data;
	struct clk_gk20a *clk = &g->clk;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	if (state)
		*state = clk->gpc_pll.enabled;
	if (rate)
		*rate = rate_gpc2clk_to_gpu(clk->gpc_pll.freq);
	nvgpu_mutex_release(&clk->clk_mutex);
}

static struct tegra_clk_export_ops gk20a_clk_export_ops = {
	.init = gk20a_clk_export_init,
	.enable = gk20a_clk_export_enable,
	.disable = gk20a_clk_export_disable,
	.set_rate = gk20a_clk_export_set_rate,
};

static int gk20a_clk_register_export_ops(struct gk20a *g)
{
	int ret;
	struct clk *c;

	if (gk20a_clk_export_ops.data)
		return 0;

	gk20a_clk_export_ops.data = (void *)g;
	c = g->clk.tegra_clk;
	if (!c || !clk_get_parent(c))
		return -ENOSYS;

	ret = tegra_clk_register_export_ops(clk_get_parent(c),
					    &gk20a_clk_export_ops);

	return ret;
}

static void gk20a_clk_disable_slowboot(struct gk20a *g)
{
	u32 data;

	data = gk20a_readl(g, trim_sys_gpc2clk_out_r());
	data = set_field(data,
			trim_sys_gpc2clk_out_bypdiv_m(),
			trim_sys_gpc2clk_out_bypdiv_f(0));
	gk20a_writel(g, trim_sys_gpc2clk_out_r(), data);
}

static int gk20a_init_clk_support(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	u32 err;

	gk20a_dbg_fn("");

	clk->g = g;

	err = gk20a_init_clk_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_clk_setup_sw(g);
	if (err)
		return err;

	nvgpu_mutex_acquire(&clk->clk_mutex);
	clk->clk_hw_on = true;

	err = gk20a_init_clk_setup_hw(g);
	nvgpu_mutex_release(&clk->clk_mutex);
	if (err)
		return err;

	err = gk20a_clk_register_export_ops(g);
	if (err)
		return err;

	/* FIXME: this effectively prevents host level clock gating */
	err = clk_enable(g->clk.tegra_clk);
	if (err)
		return err;

	/* The prev call may not enable PLL if gbus is unbalanced - force it */
	nvgpu_mutex_acquire(&clk->clk_mutex);
	err = set_pll_freq(g, clk->gpc_pll.freq, clk->gpc_pll.freq);
	nvgpu_mutex_release(&clk->clk_mutex);
	if (err)
		return err;

#ifdef CONFIG_DEBUG_FS
	if (!clk->debugfs_set) {
		if (!clk_gk20a_debugfs_init(g))
			clk->debugfs_set = true;
	}
#endif
	return err;
}

static int gk20a_suspend_clk_support(struct gk20a *g)
{
	int ret;

	clk_disable(g->clk.tegra_clk);

	/* The prev call may not disable PLL if gbus is unbalanced - force it */
	nvgpu_mutex_acquire(&g->clk.clk_mutex);
	ret = clk_disable_gpcpll(g, 1);
	g->clk.clk_hw_on = false;
	nvgpu_mutex_release(&g->clk.clk_mutex);
	return ret;
}

void gk20a_init_clk_ops(struct gpu_ops *gops)
{
	gops->clk.disable_slowboot = gk20a_clk_disable_slowboot;
	gops->clk.init_clk_support = gk20a_init_clk_support;
	gops->clk.suspend_clk_support = gk20a_suspend_clk_support;
}

#ifdef CONFIG_DEBUG_FS

static int rate_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	*val = (u64)gk20a_clk_get_rate(g);
	return 0;
}
static int rate_set(void *data, u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	return gk20a_clk_set_rate(g, (u32)val);
}
DEFINE_SIMPLE_ATTRIBUTE(rate_fops, rate_get, rate_set, "%llu\n");

static int pll_reg_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	u32 reg, m, n, pl, f;

	nvgpu_mutex_acquire(&g->clk.clk_mutex);
	if (!g->clk.clk_hw_on) {
		seq_printf(s, "gk20a powered down - no access to registers\n");
		nvgpu_mutex_release(&g->clk.clk_mutex);
		return 0;
	}

	reg = gk20a_readl(g, trim_sys_gpcpll_cfg_r());
	seq_printf(s, "cfg  = 0x%x : %s : %s\n", reg,
		   trim_sys_gpcpll_cfg_enable_v(reg) ? "enabled" : "disabled",
		   trim_sys_gpcpll_cfg_pll_lock_v(reg) ? "locked" : "unlocked");

	reg = gk20a_readl(g, trim_sys_gpcpll_coeff_r());
	m = trim_sys_gpcpll_coeff_mdiv_v(reg);
	n = trim_sys_gpcpll_coeff_ndiv_v(reg);
	pl = trim_sys_gpcpll_coeff_pldiv_v(reg);
	f = g->clk.gpc_pll.clk_in * n / (m * pl_to_div[pl]);
	seq_printf(s, "coef = 0x%x : m = %u : n = %u : pl = %u", reg, m, n, pl);
	seq_printf(s, " : pll_f(gpu_f) = %u(%u) kHz\n", f, f/2);
	nvgpu_mutex_release(&g->clk.clk_mutex);
	return 0;
}

static int pll_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_reg_show, inode->i_private);
}

static const struct file_operations pll_reg_fops = {
	.open		= pll_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int monitor_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	struct clk_gk20a *clk = &g->clk;
	int err;

	u32 ncycle = 100; /* count GPCCLK for ncycle of clkin */
	u64 freq = clk->gpc_pll.clk_in;
	u32 count1, count2;

	err = gk20a_busy(g);
	if (err)
		return err;

	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_reset_asserted_f());
	gk20a_writel(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0),
		     trim_gpc_clk_cntr_ncgpcclk_cfg_enable_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_write_en_asserted_f() |
		     trim_gpc_clk_cntr_ncgpcclk_cfg_noofipclks_f(ncycle));
	/* start */

	/* It should take about 8us to finish 100 cycle of 12MHz.
	   But longer than 100us delay is required here. */
	gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cfg_r(0));
	udelay(2000);

	count1 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	udelay(100);
	count2 = gk20a_readl(g, trim_gpc_clk_cntr_ncgpcclk_cnt_r(0));
	freq *= trim_gpc_clk_cntr_ncgpcclk_cnt_value_v(count2);
	do_div(freq, ncycle);
	*val = freq;

	gk20a_idle(g);

	if (count1 != count2)
		return -EBUSY;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");

static int clk_gk20a_debugfs_init(struct gk20a *g)
{
	struct dentry *d;
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);

	d = debugfs_create_file(
		"rate", S_IRUGO|S_IWUSR, platform->debugfs, g, &rate_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"pll_reg", S_IRUGO, platform->debugfs, g, &pll_reg_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"monitor", S_IRUGO, platform->debugfs, g, &monitor_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	debugfs_remove_recursive(platform->debugfs);
	return -ENOMEM;
}

#endif /* CONFIG_DEBUG_FS */
