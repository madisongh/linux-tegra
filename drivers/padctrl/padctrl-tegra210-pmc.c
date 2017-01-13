/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/padctrl/padctrl.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <dt-bindings/soc/tegra-io-pads.h>

#include <soc/tegra/pmc.h>

struct tegra210_pmc_padcontrl {
	struct padctrl_dev *pad_dev;
};

struct tegra210_pmc_pads_info {
	const char *name;
	int pad_id;
	int io_dpd_bit_pos;
	int io_dpd_reg_off;
	int io_volt_bit_pos;
	int io_volt_reg_off;
	bool dynamic_pad_voltage;
};

#define TEGRA_210_PAD_INFO(_name, _id, _dpd_off, _dpd_bit, _vbit) \
{								\
	.name = _name,						\
	.pad_id = TEGRA_IO_PAD_GROUP_##_id,			\
	.io_dpd_reg_off = _dpd_off,				\
	.io_dpd_bit_pos = _dpd_bit,				\
	.io_volt_bit_pos = _vbit,				\
	.io_volt_reg_off = 0,					\
}

static struct tegra210_pmc_pads_info tegra210_pads_info[] = {
	TEGRA_210_PAD_INFO("audio", AUDIO, 0, 17, 5),
	TEGRA_210_PAD_INFO("audio-hv", AUDIO_HV, 1, 29, 18),
	TEGRA_210_PAD_INFO("cam", CAM, 1, 4, 10),
	TEGRA_210_PAD_INFO("csia", CSIA, 0, 0, -1),
	TEGRA_210_PAD_INFO("csib", CSIB, 0, 1, -1),
	TEGRA_210_PAD_INFO("csic", CSIC, 1, 10, -1),
	TEGRA_210_PAD_INFO("csid", CSID, 1, 11, -1),
	TEGRA_210_PAD_INFO("csie", CSIE, 1, 12, -1),
	TEGRA_210_PAD_INFO("csif", CSIF, 1, 13, -1),
	TEGRA_210_PAD_INFO("dbg", DBG, 0, 25, 19),
	TEGRA_210_PAD_INFO("debug-nonao", DEBUG_NONAO, 0, 26, -1),
	TEGRA_210_PAD_INFO("dmic", DMIC, 1, 18, 20),
	TEGRA_210_PAD_INFO("dp", DP, 1, 19, -1),
	TEGRA_210_PAD_INFO("dsi", DSI, 0, 2, -1),
	TEGRA_210_PAD_INFO("dsib", DSIB, 1, 7, -1),
	TEGRA_210_PAD_INFO("dsic", DSIC, 1, 8, -1),
	TEGRA_210_PAD_INFO("dsid", DSID, 1, 9, -1),
	TEGRA_210_PAD_INFO("emmc", EMMC, 1, 3, -1),
	TEGRA_210_PAD_INFO("emmc2", EMMC2, 1, 5, -1),
	TEGRA_210_PAD_INFO("gpio", GPIO, 0, 27, 21),
	TEGRA_210_PAD_INFO("hdmi", HDMI, 0, 28, -1),
	TEGRA_210_PAD_INFO("hsic", HSIC, 0, 19, -1),
	TEGRA_210_PAD_INFO("lvds", LVDS, 1, 25, -1),
	TEGRA_210_PAD_INFO("mipi-bias", MIPI_BIAS, 0, 3, -1),
	TEGRA_210_PAD_INFO("pex-bias", PEX_BIAS, 0, 4, -1),
	TEGRA_210_PAD_INFO("pex-clk1", PEX_CLK1, 0, 5, -1),
	TEGRA_210_PAD_INFO("pex-clk2", PEX_CLK2, 0, 6, -1),
	TEGRA_210_PAD_INFO("pex-ctrl", PEX_CTRL, 1, 0, 11),
	TEGRA_210_PAD_INFO("sdmmc1", SDMMC1, 1, 1, 12),
	TEGRA_210_PAD_INFO("sdmmc3", SDMMC3, 1, 2, 13),
	TEGRA_210_PAD_INFO("spi", SPI, 1, 14, 22),
	TEGRA_210_PAD_INFO("spi-hv", SPI_HV, 1, 15, 23),
	TEGRA_210_PAD_INFO("uart", UART, 0, 14, 2),
	TEGRA_210_PAD_INFO("usb-bias", USB_BIAS, 0, 12, -1),
	TEGRA_210_PAD_INFO("usb0", USB0, 0, 9, -1),
	TEGRA_210_PAD_INFO("usb1", USB1, 0, 10, -1),
	TEGRA_210_PAD_INFO("usb2", USB2, 0, 11, -1),
	TEGRA_210_PAD_INFO("usb3", USB3, 0, 18, -1),
	TEGRA_210_PAD_INFO("sys", SYS, 0, -1, -1),
	TEGRA_210_PAD_INFO("bb", BB, 0, -1, -1),
	TEGRA_210_PAD_INFO("hv", HV, 0, -1, -1),
};

static struct tegra210_pmc_pads_info *tegra210_get_io_pad_info(int pad_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra210_pads_info); ++i) {
		if (tegra210_pads_info[i].pad_id == pad_id)
			break;
	}

	if (i == ARRAY_SIZE(tegra210_pads_info))
		return NULL;

	return &tegra210_pads_info[i];
}

static int tegra210_pmc_padctrl_set_voltage(struct padctrl_dev *pad_dev,
					    int pad_id, u32 voltage)
{
	struct tegra210_pmc_pads_info *pad;
	u32 curr_volt;
	int ret;

	if ((voltage != 1800000) && (voltage != 3300000)) {
		pr_err("Pad voltage %u is not valid\n", voltage);
		return -EINVAL;
	}

	pad = tegra210_get_io_pad_info(pad_id);
	if (!pad)
		return -EINVAL;

	if (pad->io_volt_bit_pos < 0)
		return -EINVAL;;

	if (!pad->dynamic_pad_voltage) {
		ret = tegra_pmc_io_pad_get_voltage(pad->name);
		if (!ret < 0)
			return ret;

		curr_volt = ret;
		if (voltage == curr_volt)
			return 0;

		pr_err("Pad %s: Dynamic pad voltage is not supported\n",
		       pad->name);

		return -EINVAL;
	}

	curr_volt = (voltage == 3300000U) ? voltage : 1800000U;
	ret = tegra_pmc_io_pad_set_voltage(pad->name, curr_volt);
	if (!ret)
		udelay(100);

	return ret;
}

static int tegra210_pmc_padctl_get_voltage(struct padctrl_dev *pad_dev,
					   int pad_id, u32 *voltage)
{
	struct tegra210_pmc_pads_info *pad;
	int ret;

	pad = tegra210_get_io_pad_info(pad_id);
	if (!pad)
		return -EINVAL;

	if (pad->io_volt_bit_pos < 0)
		return -EINVAL;;

	ret = tegra_pmc_io_pad_get_voltage(pad->name);
	if (ret < 0)
		return ret;

	*voltage = ret;

	return 0;
}

static int tegra210_pmc_padctrl_set_power(struct padctrl_dev *pad_dev,
					  int pad_id, u32 enable)
{
	struct tegra210_pmc_pads_info *pad;
	int ret;

	pad = tegra210_get_io_pad_info(pad_id);
	if (!pad)
		return -EINVAL;

	if (pad->io_dpd_bit_pos < 0)
		return -EINVAL;

	if (enable)
		ret = tegra_pmc_io_pad_low_power_disable(pad->name);
	else
		ret = tegra_pmc_io_pad_low_power_enable(pad->name);

	return ret;
}

static int tegra210_pmc_padctrl_power_enable(struct padctrl_dev *pad_dev,
					     int pad_id)
{
	return tegra210_pmc_padctrl_set_power(pad_dev, pad_id, 1);
}

static int tegra210_pmc_padctrl_power_disable(struct padctrl_dev *pad_dev,
					      int pad_id)
{
	return tegra210_pmc_padctrl_set_power(pad_dev, pad_id, 0);
}

static struct padctrl_ops tegra210_pmc_padctrl_ops = {
	.set_voltage = &tegra210_pmc_padctrl_set_voltage,
	.get_voltage = &tegra210_pmc_padctl_get_voltage,
	.power_enable = &tegra210_pmc_padctrl_power_enable,
	.power_disable = &tegra210_pmc_padctrl_power_disable,
};

static struct padctrl_desc tegra210_pmc_padctrl_desc = {
	.name = "tegra-pmc-padctrl",
	.ops = &tegra210_pmc_padctrl_ops,
};

static int tegra210_pmc_parse_io_pad_init(struct device_node *np,
		struct padctrl_dev *pad_dev)
{
	struct device_node *pad_np, *child;
	struct tegra210_pmc_pads_info *pad;
	u32 pval;
	int pad_id;
	const char *pad_name, *name;
	bool dpd_en, dpd_dis, pad_en, pad_dis, io_dpd_en, io_dpd_dis;
	bool dyn_pad_volt;
	int n_config;
	u32 *volt_configs, *iodpd_configs;
	int i, index, vcount, dpd_count, pindex;
	int ret;

	pad_np = of_get_child_by_name(np, "io-pad-defaults");
	if (!pad_np)
		return 0;

	/* Ignore the nodes if disabled */
	ret = of_device_is_available(pad_np);
	if (!ret)
		return 0;

	pr_info("PMC: configuring io pad defaults\n");
	n_config = of_get_child_count(pad_np);
	if (!n_config)
		return 0;
	n_config *= 2;

	volt_configs = kzalloc(n_config * sizeof(*volt_configs), GFP_KERNEL);
	if (!volt_configs)
		return -ENOMEM;

	iodpd_configs = kzalloc(n_config * sizeof(*iodpd_configs), GFP_KERNEL);
	if (!iodpd_configs) {
		kfree(volt_configs);
		return -ENOMEM;
	}

	vcount = 0;
	dpd_count = 0;
	for_each_available_child_of_node(pad_np, child) {
		name = of_get_property(child, "nvidia,pad-name", NULL);
		if (!name)
			name = child->name;

		for (i = 0; i < ARRAY_SIZE(tegra210_pads_info); ++i) {
			pad = &tegra210_pads_info[i];
			if (strcmp(name, pad->name))
				continue;

			ret = of_property_read_u32(child,
					"nvidia,io-pad-init-voltage", &pval);
			if (!ret) {
				volt_configs[vcount] = i;
				volt_configs[vcount + 1] = pval;
				vcount += 2;
			}

			pad->dynamic_pad_voltage = of_property_read_bool(child,
					"nvidia,enable-dynamic-pad-voltage");

			dpd_en = of_property_read_bool(child,
						"nvidia,deep-power-down-enable");
			dpd_dis = of_property_read_bool(child,
						"nvidia,deep-power-down-disable");
			pad_en = of_property_read_bool(child,
						"nvidia,io-pad-power-enable");
			pad_dis = of_property_read_bool(child,
						"nvidia,io-pad-power-disable");

			io_dpd_en = dpd_en | pad_dis;
			io_dpd_dis = dpd_dis | pad_en;

			if ((dpd_en && pad_en)	|| (dpd_dis && pad_dis) ||
					(io_dpd_en & io_dpd_dis)) {
				pr_err("PMC: Conflict on io-pad %s config\n",
					name);
				continue;
			}
			if (io_dpd_en || io_dpd_dis) {
				iodpd_configs[dpd_count] = i;
				iodpd_configs[dpd_count + 1] = !!io_dpd_dis;
				dpd_count += 2;
			}
		}
	}

	for (i = 0; i < vcount/2; ++i) {
		index = i * 2;
		if (!volt_configs[index + 1])
			continue;
		pindex = volt_configs[index];
		pad_id = tegra210_pads_info[volt_configs[index]].pad_id;
		pad_name = tegra210_pads_info[volt_configs[index]].name;

		dyn_pad_volt = tegra210_pads_info[pindex].dynamic_pad_voltage;
		tegra210_pads_info[pindex].dynamic_pad_voltage = true;
		ret = tegra210_pmc_padctrl_set_voltage(pad_dev,
				pad_id, volt_configs[index + 1]);
		if (ret < 0) {
			pr_warn("PMC: IO pad %s voltage config failed: %d\n",
				pad_name, ret);
			WARN_ON(1);
		} else {
			pr_info("PMC: IO pad %s voltage is %d\n",
				pad_name, volt_configs[index + 1]);
		}
		tegra210_pads_info[pindex].dynamic_pad_voltage = dyn_pad_volt;
	}

	for (i = 0; i < dpd_count / 2; ++i) {
		index = i * 2;
		pad_id = tegra210_pads_info[iodpd_configs[index]].pad_id;
		pad_name = tegra210_pads_info[iodpd_configs[index]].name;

		ret = tegra210_pmc_padctrl_set_power(pad_dev,
				pad_id, iodpd_configs[index + 1]);
		if (ret < 0) {
			pr_warn("PMC: IO pad %s power config failed: %d\n",
			     pad_name, ret);
			WARN_ON(1);
		} else {
			pr_info("PMC: IO pad %s power is %s\n",
				pad_name, (iodpd_configs[index + 1]) ?
				"enable" : "disable");
		}
	}

	kfree(volt_configs);
	kfree(iodpd_configs);
	return 0;
}

int tegra210_pmc_padctrl_init(struct device *dev, struct device_node *np)
{
	struct tegra210_pmc_padcontrl *pmc_padctrl;
	struct padctrl_config config = { };
	int ret;

	pmc_padctrl = kzalloc(sizeof(*pmc_padctrl), GFP_KERNEL);
	if (!pmc_padctrl) {
		pr_err("Mem allocation for pmc_padctrl failed\n");
		return -ENOMEM;
	}

	config.of_node = (dev && dev->of_node) ? dev->of_node : np;
	pmc_padctrl->pad_dev = padctrl_register(dev, &tegra210_pmc_padctrl_desc,
						&config);
	if (IS_ERR(pmc_padctrl->pad_dev)) {
		ret = PTR_ERR(pmc_padctrl->pad_dev);
		pr_err("T210 padctrl driver init failed: %d\n", ret);
		kfree(pmc_padctrl);
		return ret;
	}
	padctrl_set_drvdata(pmc_padctrl->pad_dev, pmc_padctrl);

	/* Clear all DPD */
	tegra_pmc_io_dpd_clear();

	tegra210_pmc_parse_io_pad_init(config.of_node,
				       pmc_padctrl->pad_dev);

	pr_info("T210 pmc padctrl driver initialized\n");

	return 0;
}
