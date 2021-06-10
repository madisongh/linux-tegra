/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp_stub.c
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* Define stub functions to allow compilation */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/clk/tegra.h>
#include <linux/of.h>
#include <soc/tegra/chip-id.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/iommu.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>

#include <linux/platform/tegra/latency_allowance.h>

#include "dc.h"
#include "board-panel.h"
#include "dc_priv.h"
#include <linux/platform_data/lp855x.h>
#include <soc/tegra/common.h>

const struct disp_client *tegra_la_disp_clients_info;
atomic_t sd_brightness = ATOMIC_INIT(255);
EXPORT_SYMBOL(sd_brightness);

int tegra_is_clk_enabled(struct clk *c)
{
	dump_stack();
	pr_info(" WARNING!!! OBSOLETE FUNCTION CALL!!! \
			DON'T USE %s FUNCTION \n", __func__);
	return 0;
}
EXPORT_SYMBOL(tegra_is_clk_enabled);

static int tegra_bl_notify(struct device *dev, int brightness)
{
	int cur_sd_brightness;
	struct lp855x *lp = NULL;
	u8 *bl_measured = NULL;
	u8 *bl_curve = NULL;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else if (of_device_is_compatible(dev->of_node,
				"ti,lp8550") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8551") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8552") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8553") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8554") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8555") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8556") ||
		of_device_is_compatible(dev->of_node,
				"ti,lp8557")) {
		lp = (struct lp855x *)dev_get_drvdata(dev);
		if (lp && lp->pdata) {
			bl_measured = lp->pdata->bl_measured;
			bl_curve = lp->pdata->bl_curve;
		}
	}

	if (bl_curve)
		brightness = bl_curve[brightness];

	cur_sd_brightness = atomic_read(&sd_brightness);
	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	if (bl_measured)
		brightness = bl_measured[brightness];

	return brightness;
}

static int tegra_t210ref_i2c_notifier_call(struct notifier_block *nb,
	unsigned long event, void *data)
{
	struct backlight_device_brightness_info *smartdim_info = data;
	struct device *dev = smartdim_info->dev;
	if (dev->of_node) {
		if (of_device_is_compatible(dev->of_node,
			"ti,lp8557")) {
			smartdim_info->brightness = tegra_bl_notify(dev,
				smartdim_info->brightness);
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block i2c_nb = {
	.notifier_call = tegra_t210ref_i2c_notifier_call,
};

int nvdisp_register_backlight_notifier(struct tegra_dc *dc)
{
	if (dc->out->sd_settings && !dc->out->sd_settings->bl_device &&
		dc->out->sd_settings->bl_device_name) {
		char *bl_device_name = dc->out->sd_settings->bl_device_name;
		struct backlight_device *bl_device =
			get_backlight_device_by_name(bl_device_name);
		if (bl_device)
			backlight_device_register_notifier(bl_device, &i2c_nb);
	}
	return 0;
}

static int disp_fb_linear_set(void)
{
	tegra_fb_linear_set(NULL);
	return 0;
}
arch_initcall(disp_fb_linear_set);

int tegra_dvfs_set_rate(struct clk *c, unsigned long rate)
{
	return 0;
}

int tegra_dvfs_use_alt_freqs_on_clk(struct clk *c, bool use_alt_freq)
{
	return 0;
}

int tegra_panel_gpio_get_dt(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int cnt = 0;
	char *label = NULL;
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	/*
	 * If gpios are already populated, just return.
	 */
	if (panel->panel_gpio_populated)
		return 0;

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
		goto fail;
	}

	panel->panel_gpio[TEGRA_GPIO_RESET] =
		of_get_named_gpio(node, "nvidia,panel-rst-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PANEL_EN] =
		of_get_named_gpio(node, "nvidia,panel-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BL_ENABLE] =
		of_get_named_gpio(node, "nvidia,panel-bl-en-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_PWM] =
		of_get_named_gpio(node, "nvidia,panel-bl-pwm-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_0] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-0-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_EN_1] =
		of_get_named_gpio(node, "nvidia,panel-bridge-en-1-gpio", 0);

	panel->panel_gpio[TEGRA_GPIO_BRIDGE_REFCLK_EN] =
		of_get_named_gpio(node,
			"nvidia,panel-bridge-refclk-en-gpio", 0);

	for (cnt = 0; cnt < TEGRA_N_GPIO_PANEL; cnt++) {
		if (gpio_is_valid(panel->panel_gpio[cnt])) {
			switch (cnt) {
			case TEGRA_GPIO_RESET:
				label = "tegra-panel-reset";
				break;
			case TEGRA_GPIO_PANEL_EN:
				label = "tegra-panel-en";
				break;
			case TEGRA_GPIO_BL_ENABLE:
				label = "tegra-panel-bl-enable";
				break;
			case TEGRA_GPIO_PWM:
				label = "tegra-panel-pwm";
				break;
			case TEGRA_GPIO_BRIDGE_EN_0:
				label = "tegra-panel-bridge-en-0";
				break;
			case TEGRA_GPIO_BRIDGE_EN_1:
				label = "tegra-panel-bridge-en-1";
				break;
			case TEGRA_GPIO_BRIDGE_REFCLK_EN:
				label = "tegra-panel-bridge-refclk-en";
				break;
			default:
				pr_err("tegra panel no gpio entry\n");
			}
			if (label) {
				gpio_request(panel->panel_gpio[cnt],
					label);
				label = NULL;
			}
		}
	}
	if (gpio_is_valid(panel->panel_gpio[TEGRA_GPIO_PWM]))
		gpio_free(panel->panel_gpio[TEGRA_GPIO_PWM]);
	panel->panel_gpio_populated = true;
fail:
	of_node_put(node);
	return err;
}

int tegra_panel_check_regulator_dt_support(const char *comp_str,
				struct tegra_panel_of *panel)
{
	int err = 0;
	struct device_node *node =
		of_find_compatible_node(NULL, NULL, comp_str);

	if (!node) {
		pr_info("%s panel dt support not available\n", comp_str);
		err = -ENOENT;
	}

	panel->en_vmm_vpp_i2c_config =
		of_property_read_bool(node, "nvidia,en-vmm-vpp-with-i2c-config");

	return err;
}

static void tegra_panel_register_mods_ops(struct tegra_dc_out *dc_out)
{
	BUG_ON(!dc_out);

	dc_out->enable = NULL;
	dc_out->postpoweron = NULL;
	dc_out->prepoweroff = NULL;
	dc_out->disable = NULL;
	dc_out->hotplug_init = NULL;
	dc_out->postsuspend = NULL;
	dc_out->hotplug_report = NULL;
}

static void tegra_panel_register_ops(struct tegra_dc_out *dc_out,
				struct tegra_panel_ops *p_ops)
{
	BUG_ON(!dc_out);

	dc_out->enable = p_ops->enable;
	dc_out->postpoweron = p_ops->postpoweron;
	dc_out->prepoweroff = p_ops->prepoweroff;
	dc_out->disable = p_ops->disable;
	dc_out->hotplug_init = p_ops->hotplug_init;
	dc_out->postsuspend = p_ops->postsuspend;
	dc_out->hotplug_report = p_ops->hotplug_report;
}

extern struct tegra_panel_ops panel_sim_ops;

/* Clone from board-panel.c */
struct device_node *tegra_primary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_primary;

	if (!pdata) {
		pr_err("platform data not available\n");
		return NULL;
	}

	dc_out = pdata->default_out;

	np_primary = of_find_node_by_path(pdata->dc_or_node_name);

	if (of_device_is_available(np_primary)) {
		/* DSI */
		/* SHARP 19x12 panel is being used */

		np_panel = of_get_child_by_name(np_primary,
			"panel-s-wuxga-8-0");
		if (of_device_is_available(np_panel) && dc_out)
			tegra_panel_register_ops(dc_out,
				&dsi_s_wuxga_8_0_ops);

		/* SHARP 2160x3840 panel is being used */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
					"panel-s-4kuhd-5-46");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
						&dsi_s_4kuhd_5_46_ops);
		}

		/* P2393 DSI2DP Bridge */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_primary,
				"panel-dsi-1080p-p2382");
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
				"panel-s-wqxga-10-1");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&dsi_s_wqxga_10_1_ops);
		}

		/* 62681 */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_primary,
				"panel-a-2820x720-10-1_8-6");

		/* DSI null panel with hotplug support */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
				"panel-null-dsi-hotplug");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&dsi_null_panel_ops);
		}

		/* HDMI */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_primary,
				"hdmi-display");
		/* DP */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_primary,
				"dp-display");
		/* eDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
				"panel-s-edp-uhdtv-15-6");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
				&edp_s_uhdtv_15_6_ops);
		}
		/* AUO 1080p EDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
				"panel-a-edp-1080p-14-0");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
				&edp_a_1080p_14_0_ops);
		}
		/* MODS - DSI to fakeDP - Bug 1734772*/
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_primary,
				"panel-s-wuxga-8-0-mods");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_mods_ops(dc_out);
		}
		/* simulation panel */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(
				np_primary, "panel-nvidia-sim");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out, &panel_sim_ops);
		}
	}

	if (!np_panel && !strcmp(pdata->dc_or_node_name, ""))
		pr_err("Could not find panel for primary node\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_secondary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_secondary;

	if (!pdata) {
		pr_err("platform data not available\n");
		return NULL;
	}

	dc_out = pdata->default_out;

	np_secondary = of_find_node_by_path(pdata->dc_or_node_name);
	if (of_device_is_available(np_secondary)) {
		/* HDMI */
		np_panel = of_get_child_by_name(np_secondary,
			"hdmi-display");
		/* DP */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_secondary,
				"dp-display");
		/* eDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_secondary,
				"panel-s-edp-uhdtv-15-6");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&edp_s_uhdtv_15_6_ops);
		}
		/* AUO 1080p EDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_secondary,
				"panel-a-edp-1080p-14-0");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
				&edp_a_1080p_14_0_ops);
		}
		/* DSI */
		/* SHARP 19x12 panel */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_secondary,
				"panel-s-wuxga-8-0");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&dsi_s_wuxga_8_0_ops);
		}
		/* SHARP 2160x3840 panel is being used */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_secondary,
					"panel-s-4kuhd-5-46");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
						&dsi_s_4kuhd_5_46_ops);
		}
		/* P2393 DSI2DP Bridge */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(
				np_secondary, "panel-dsi-1080p-p2382");
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_secondary,
				"panel-s-wqxga-10-1");

		/* 62681 */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(
				np_secondary, "panel-a-2820x720-10-1_8-6");

		/* DSI null panel with hotplug support */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_secondary,
				"panel-null-dsi-hotplug");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&dsi_null_panel_ops);
		}

		/* simulation panel */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(
				np_secondary, "panel-nvidia-sim");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out, &panel_sim_ops);
		}
	}

	if (!np_panel && !strcmp(pdata->dc_or_node_name, ""))
		pr_err("Could not find panel for secondary node\n");

	return of_device_is_available(np_panel) ? np_panel : NULL;
}

struct device_node *tegra_tertiary_panel_get_dt_node(
			struct tegra_dc_platform_data *pdata)
{
	struct device_node *np_panel = NULL;
	struct tegra_dc_out *dc_out = NULL;
	struct device_node *np_tertiary;

	if (!pdata) {
		pr_err("platform data not available\n");
		return NULL;
	}

	dc_out = pdata->default_out;

	np_tertiary = of_find_node_by_path(pdata->dc_or_node_name);
	if (of_device_is_available(np_tertiary)) {
		/* DP */
		np_panel = of_get_child_by_name(np_tertiary,
				"dp-display");
		/* eDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_tertiary,
				"panel-s-edp-uhdtv-15-6");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
					&edp_s_uhdtv_15_6_ops);
		}
		/* AUO 1080p EDP */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(np_tertiary,
				"panel-a-edp-1080p-14-0");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out,
				&edp_a_1080p_14_0_ops);
		}
		/* HDMI */
		if (!of_device_is_available(np_panel))
			np_panel = of_get_child_by_name(np_tertiary,
				"hdmi-display");
		/* simulation panel */
		if (!of_device_is_available(np_panel)) {
			np_panel = of_get_child_by_name(
				np_tertiary, "panel-nvidia-sim");
			if (of_device_is_available(np_panel) && dc_out)
				tegra_panel_register_ops(dc_out, &panel_sim_ops);
		}
	}

	if (!np_panel && !strcmp(pdata->dc_or_node_name, ""))
		pr_err("Could not find panel for tertiary node\n");

	return (of_device_is_available(np_panel) ? np_panel : NULL);
}
