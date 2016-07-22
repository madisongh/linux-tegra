/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>

#include "clk.h"

#define SKIPPER_DIVISOR 256

static int calc_skipper_mul(unsigned long rate, unsigned long prate)
{
	int mul;

	rate = rate * 256;
	mul = rate / prate;
	mul = max(mul, 1);
	mul = min(mul, 256);

	return mul;
}

static int clk_skipper_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	struct tegra_clk_skipper *skipper = to_clk_skipper(hw);
	unsigned long flags = 0;
	int mul;
	u32 val;

	mul = calc_skipper_mul(rate, prate);

	if (skipper->lock)
		spin_lock_irqsave(skipper->lock, flags);

	val = readl_relaxed(skipper->reg);
	val &= (0xff << 8);
	val |= (mul - 1) << 8;
	writel_relaxed(val, skipper->reg);

	if (skipper->lock)
		spin_unlock_irqrestore(skipper->lock, flags);

	return 0;
}

static long clk_skipper_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	int mul;

	mul = calc_skipper_mul(rate, *prate);

	return (*prate * mul) / SKIPPER_DIVISOR;
}

static unsigned long clk_skipper_recalc_rate(struct clk_hw *hw,
					     unsigned long prate)
{
	struct tegra_clk_skipper *skipper = to_clk_skipper(hw);
	u32 val;
	int mul;

	val = readl_relaxed(skipper->reg);
	mul = ((val >> 8) & 0xff) + 1;

	return (prate * mul) / SKIPPER_DIVISOR;
}

static int clk_skipper_enable(struct clk_hw *hw)
{
	struct tegra_clk_skipper *skipper = to_clk_skipper(hw);
	unsigned long flags = 0;
	u32 val;

	if (skipper->lock)
		spin_lock_irqsave(skipper->lock, flags);

	val = readl_relaxed(skipper->reg);
	val |= BIT(31);
	writel_relaxed(val, skipper->reg);

	if (skipper->lock)
		spin_unlock_irqrestore(skipper->lock, flags);

	return 0;
}

static void clk_skipper_disable(struct clk_hw *hw)
{
	struct tegra_clk_skipper *skipper = to_clk_skipper(hw);
	unsigned long flags = 0;
	u32 val;

	if (skipper->lock)
		spin_lock_irqsave(skipper->lock, flags);

	val = readl_relaxed(skipper->reg);
	val &= ~BIT(31);
	writel_relaxed(val, skipper->reg);

	if (skipper->lock)
		spin_unlock_irqrestore(skipper->lock, flags);

	return;
}

static int clk_skipper_is_enabled(struct clk_hw *hw)
{
	struct tegra_clk_skipper *skipper = to_clk_skipper(hw);

	return !!(readl_relaxed(skipper->reg) & BIT(31));
}

const struct clk_ops tegra_clk_skipper_ops = {
	.recalc_rate = clk_skipper_recalc_rate,
	.set_rate = clk_skipper_set_rate,
	.round_rate = clk_skipper_round_rate,
	.enable = clk_skipper_enable,
	.disable = clk_skipper_disable,
	.is_enabled = clk_skipper_is_enabled,
};

struct clk *tegra_clk_register_skipper(const char *name,
		const char *parent_name, void __iomem *reg,
		unsigned long flags, spinlock_t *lock)
{
	struct tegra_clk_skipper *skipper;
	struct clk *clk;
	struct clk_init_data init;
	u32 val;

	skipper = kzalloc(sizeof(*skipper), GFP_KERNEL);
	if (!skipper) {
		pr_err("%s: could not allocate skipper clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &tegra_clk_skipper_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	skipper->reg = reg;
	skipper->lock = lock;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	skipper->hw.init = &init;

	clk = clk_register(NULL, &skipper->hw);
	if (IS_ERR(clk))
		kfree(skipper);

	val = readl_relaxed(skipper->reg);
	val |= (SKIPPER_DIVISOR - 1) << 8;
	val |= SKIPPER_DIVISOR - 1;
	writel(val, skipper->reg);

	return clk;
}
