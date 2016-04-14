/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2012-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/clk/tegra.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_VNDR_CLK_CTRL       0x100
#define SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT		16
#define SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK		0xFF
#define SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT		24
#define SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK		0x1F
#define SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK		0x2
#define SDHCI_VNDR_CLK_CTRL_SDMMC_CLK			0x1

#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_MISC_CTRL_ENABLE_SDR104		0x8
#define SDHCI_MISC_CTRL_ENABLE_SDR50		0x10
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_MISC_CTRL_ENABLE_DDR50		0x200

#define NVQUIRK_FORCE_SDHCI_SPEC_200	BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET	BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300	BIT(2)
#define NVQUIRK_DISABLE_SDR50		BIT(3)
#define NVQUIRK_DISABLE_SDR104		BIT(4)
#define NVQUIRK_DISABLE_DDR50		BIT(5)
/* ENAABLE FEEDBACK IO CLOCK */
#define NVQUIRK_EN_FEEDBACK_CLK		BIT(7)

/* Common quirks for Tegra 12x and later versions of sdmmc controllers */
#define TEGRA_SDHCI_QUIRKS (SDHCI_QUIRK_BROKEN_TIMEOUT_VAL | \
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK | \
		  SDHCI_QUIRK_SINGLE_POWER_WRITE | \
		  SDHCI_QUIRK_NO_HISPD_BIT | \
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC | \
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION | \
		  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC)

#define TEGRA_SDHCI_QUIRKS2 (SDHCI_QUIRK2_PRESET_VALUE_BROKEN | \
	SDHCI_QUIRK2_HOST_OFF_CARD_ON)

#define TEGRA_SDHCI_NVQUIRKS (NVQUIRK_EN_FEEDBACK_CLK)

/* max limit defines */
#define SDHCI_TEGRA_MAX_TAP_VALUES	0xFF
#define SDHCI_TEGRA_MAX_TRIM_VALUES	0x1F

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
	u32 nvquirks2;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	struct gpio_desc *power_gpio;
	const struct tegra_sdhci_platform_data *plat;
	bool	clk_enabled;
	/* ensure atomic set clock calls */
	struct mutex	set_clock_mutex;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	/* max ddr clk supported by the platform */
	struct reset_control *rstc;
};

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}

	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	switch (reg) {
	case SDHCI_TRANSFER_MODE:
		/*
		 * Postpone this write, we must do it together with a
		 * command write that is down below.
		 */
		pltfm_host->xfer_mode_shadow = val;
		return;
	case SDHCI_COMMAND:
		writel((val << 16) | pltfm_host->xfer_mode_shadow,
			host->ioaddr + SDHCI_TRANSFER_MODE);
		return;
	}

	writew(val, host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

	if (unlikely((soc_data->nvquirks & NVQUIRK_ENABLE_BLOCK_GAP_DET) &&
			(reg == SDHCI_INT_ENABLE))) {
		/* Erratum: Must enable block gap interrupt detection */
		u8 gap_ctrl = readb(host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
		if (val & SDHCI_INT_CARD_INT)
			gap_ctrl |= 0x8;
		else
			gap_ctrl &= ~0x8;
		writeb(gap_ctrl, host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
	}
}

static unsigned int tegra_sdhci_get_ro(struct sdhci_host *host)
{
	return mmc_gpio_get_ro(host->mmc);
}
static inline int sdhci_tegra_set_trim_delay(struct sdhci_host *sdhci,
	int trim_delay)
{
	u32 vendor_ctrl;

	if ((trim_delay > SDHCI_TEGRA_MAX_TRIM_VALUES) && (trim_delay < 0)) {
		dev_err(mmc_dev(sdhci->mmc), "Invalid trim value\n");
		return -1;
	}

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= ~(SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	vendor_ctrl |= (trim_delay << SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	return 0;
}

static inline int sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	int tap_delay)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u32 vendor_ctrl;
	u16 clk;
	bool card_clk_enabled;

	if ((tap_delay > SDHCI_TEGRA_MAX_TAP_VALUES) && (tap_delay < 0)){
		dev_err(mmc_dev(sdhci->mmc), "Invalid tap value\n");
		return -1;
	}

	clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL);
	card_clk_enabled = clk & SDHCI_CLOCK_CARD_EN;

	if (card_clk_enabled) {
		clk &= ~SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
	}

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= ~(SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
	udelay(1);
	sdhci_reset(sdhci, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

	if (card_clk_enabled) {
		clk |= SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
	}

	return 0;
}

static void tegra_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	u32 misc_ctrl, vendor_ctrl;

	sdhci_reset(host, mask);

	if (!(mask & SDHCI_RESET_ALL))
		return;

	vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE;
	/* Enable feedback/internal clock */
	if (soc_data->nvquirks & NVQUIRK_EN_FEEDBACK_CLK)
		vendor_ctrl &= ~SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK;
	else
		vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK;
	sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	misc_ctrl = sdhci_readw(host, SDHCI_TEGRA_VENDOR_MISC_CTRL);
	/* Erratum: Enable SDHCI spec v3.00 support */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDHCI_SPEC_300)
		misc_ctrl |= SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
	/* Don't advertise UHS modes which aren't supported yet */
	if (soc_data->nvquirks & NVQUIRK_DISABLE_SDR50)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_SDR50;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_DDR50)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_DDR50;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_SDR104)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_SDR104;
	sdhci_writew(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);

	/* Mask any bus speed modes if set in platform data */
	if (plat->uhs_mask & MMC_UHS_MASK_DDR50) {
		host->mmc->caps &= ~MMC_CAP_UHS_DDR50;
		host->mmc->caps &= ~MMC_CAP_1_8V_DDR;
	}

	if (plat->uhs_mask & MMC_MASK_HS200) {
		host->mmc->caps2 &= ~MMC_CAP2_HS200;
		host->mmc->caps2 &= ~MMC_CAP2_HS400;
	}

	if (plat->uhs_mask & MMC_MASK_HS400)
		host->mmc->caps2 &= ~MMC_CAP2_HS400;
}

static void tegra_sdhci_set_clock(struct sdhci_host *sdhci, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u8 vendor_ctrl;
	int ret = 0;

	mutex_lock(&tegra_host->set_clock_mutex);
	pr_debug("%s %s %u enabled=%u\n", __func__,
		mmc_hostname(sdhci->mmc), clock, tegra_host->clk_enabled);
	if (clock) {
		if (!tegra_host->clk_enabled) {
			ret = clk_prepare_enable(pltfm_host->clk);
			if (ret) {
				dev_err(mmc_dev(sdhci->mmc),
				"clock enable is failed, ret: %d\n", ret);
				mutex_unlock(&tegra_host->set_clock_mutex);
				return;
			}
			tegra_host->clk_enabled = true;
			vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
			vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
		}
		sdhci_set_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		sdhci_set_clock(sdhci, 0);
		vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
		vendor_ctrl &= ~0x1;
		sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
		clk_disable_unprepare(pltfm_host->clk);
		tegra_host->clk_enabled = false;
	}
	mutex_unlock(&tegra_host->set_clock_mutex);
}

static void tegra_sdhci_set_bus_width(struct sdhci_host *host, int bus_width)
{
	u32 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if ((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
	    (bus_width == MMC_BUS_WIDTH_8)) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int timing)
{
	u16 clk;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	/* Set the UHS signaling mode */
	sdhci_set_uhs_signaling(host, timing);

	/*
	 * Tegra SDMMC controllers support only a clock divisor of 2 in DDR
	 * mode. No other divisors are supported.
	 */
	if ((timing == MMC_TIMING_UHS_DDR50) ||
		(timing == MMC_TIMING_MMC_DDR52)) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

		/* Set ddr mode tap delay */
		sdhci_tegra_set_tap_delay(host, plat->ddr_tap_delay);

		/* Set the ddr mode trim delay if required */
		sdhci_tegra_set_trim_delay(host, plat->ddr_trim_delay);
	}
}

static const struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.set_clock  = tegra_sdhci_set_clock,
	.set_bus_width = tegra_sdhci_set_bus_width,
	.reset      = tegra_sdhci_reset,
	.set_uhs_signaling = tegra_sdhci_set_uhs_signaling,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.write_w    = tegra_sdhci_writew,
};

static const struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra20 = {
	.pdata = &sdhci_tegra20_pdata,
	.nvquirks = NVQUIRK_FORCE_SDHCI_SPEC_200 |
		    NVQUIRK_ENABLE_BLOCK_GAP_DET,
};

static const struct sdhci_pltfm_data sdhci_tegra30_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra30 = {
	.pdata = &sdhci_tegra30_pdata,
	.nvquirks = NVQUIRK_ENABLE_SDHCI_SPEC_300 |
		    NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct sdhci_ops tegra114_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_w     = tegra_sdhci_readw,
	.write_w    = tegra_sdhci_writew,
	.write_l    = tegra_sdhci_writel,
	.set_clock  = sdhci_set_clock,
	.set_bus_width = tegra_sdhci_set_bus_width,
	.reset      = tegra_sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
};

static const struct sdhci_pltfm_data sdhci_tegra114_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra114_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra114 = {
	.pdata = &sdhci_tegra114_pdata,
	.nvquirks = NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_DDR50 |
		    NVQUIRK_DISABLE_SDR104,
};
static const struct sdhci_pltfm_data sdhci_tegra186_pdata = {
	.quirks = TEGRA_SDHCI_QUIRKS,
	.quirks2 = TEGRA_SDHCI_QUIRKS2,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra186 = {
	.pdata = &sdhci_tegra186_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra186-sdhci", .data = &soc_data_tegra186 },
	{ .compatible = "nvidia,tegra124-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra114-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra30 },
	{ .compatible = "nvidia,tegra20-sdhci", .data = &soc_data_tegra20 },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tegra_dt_match);

static int sdhci_tegra_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tegra_sdhci_platform_data *plat;

	if (!np)
		return -EINVAL;

	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	of_property_read_u32(np, "max-clk-limit", &plat->max_clk_limit);
	of_property_read_u32(np, "uhs-mask", &plat->uhs_mask);
	of_property_read_u32(np, "nvidia,ddr-tap-delay", &plat->ddr_tap_delay);
	of_property_read_u32(np, "nvidia,ddr-trim-delay", &plat->ddr_trim_delay);

	tegra_host->plat = plat;
	return mmc_of_parse(host->mmc);
}

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tegra *tegra_host;
	const struct tegra_sdhci_platform_data *plat;
	struct clk *clk;
	int rc;

	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm_host = sdhci_priv(host);

	tegra_host = devm_kzalloc(&pdev->dev, sizeof(*tegra_host), GFP_KERNEL);
	if (!tegra_host) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra_host\n");
		rc = -ENOMEM;
		goto err_alloc_tegra_host;
	}
	tegra_host->soc_data = soc_data;
	pltfm_host->priv = tegra_host;
	rc = sdhci_tegra_parse_dt(&pdev->dev);
	if (rc)
		goto err_parse_dt;

	plat = tegra_host->plat;

	mutex_init(&tegra_host->set_clock_mutex);
	clk = devm_clk_get(&pdev->dev, "sdmmc");
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;

	/* Reset the sdhci controller to clear all previous status.*/

	tegra_host->rstc = devm_reset_control_get(&pdev->dev, "sdmmc");
	if (IS_ERR(tegra_host->rstc))
		pr_err("Reset for %s is failed\n", dev_name(&pdev->dev));
	else
		reset_control_reset(tegra_host->rstc);

	pltfm_host->priv = tegra_host;

	tegra_host->max_clk_limit = plat->max_clk_limit;

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	return 0;

err_add_host:
	clk_disable_unprepare(pltfm_host->clk);
err_clk_get:
err_parse_dt:
err_alloc_tegra_host:
	sdhci_pltfm_free(pdev);
	return rc;
}

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= SDHCI_PLTFM_PMOPS,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_pltfm_unregister,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
