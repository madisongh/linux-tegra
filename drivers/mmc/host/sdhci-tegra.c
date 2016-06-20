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
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/reset.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/clk/tegra.h>
#include <linux/tegra_prod.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/mmc-sdhci-tegra.h>
#include <linux/padctrl/padctrl.h>
#include <linux/mmc/cmdq_hci.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_VNDR_CLK_CTRL       0x100
#define SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT		16
#define SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK		0xFF
#define SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT		24
#define SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK		0x1F
#define SDHCI_VNDR_CLK_CTRL_SDR50_TUNING		0x20
#define SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK		0x2
#define SDHCI_VNDR_CLK_CTRL_SDMMC_CLK			0x1

#define SDHCI_VNDR_SYS_SW_CTRL				0x104
#define SDHCI_VNDR_SYS_SW_CTRL_WR_CRC_USE_TMCLK		0x40000000
#define SDHCI_VNDR_SYS_SW_CTRL_STROBE_SHIFT		31

#define SDHCI_VNDR_CAP_OVERRIDES_0			0x10c
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_SHIFT	8
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_MASK	0x3F

#define SDHCI_VNDR_DLLCAL_CFG				0x1b0
#define SDHCI_VNDR_DLLCAL_CFG_EN_CALIBRATE		0x80000000

#define SDHCI_VNDR_DLLCAL_CFG_STATUS			0x1bc
#define SDHCI_VNDR_DLLCAL_CFG_STATUS_DLL_ACTIVE		0x80000000

#define SDHCI_VNDR_MISC_CTRL		0x120

#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_MISC_CTRL_ENABLE_SDR104		0x8
#define SDHCI_MISC_CTRL_ENABLE_SDR50		0x10
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_MISC_CTRL_ENABLE_DDR50		0x200

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START	0x80000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_SLW_OVERRIDE	0x10000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT	0x10

#define SDMMC_AUTO_CAL_STATUS	0x1EC
#define SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE	0x80000000

#define NVQUIRK_FORCE_SDHCI_SPEC_200	BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET	BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300	BIT(2)
#define NVQUIRK_DISABLE_SDR50		BIT(3)
#define NVQUIRK_DISABLE_SDR104		BIT(4)
#define NVQUIRK_DISABLE_DDR50		BIT(5)
/* Do not enable auto calibration if the platform doesn't support */
#define NVQUIRK_DISABLE_AUTO_CALIBRATION	BIT(6)
/* ENAABLE FEEDBACK IO CLOCK */
#define NVQUIRK_EN_FEEDBACK_CLK		BIT(7)
/*Enable e_input before setting voltage*/
#define NVQUIRK2_SET_PAD_E_INPUT_VOL		BIT(8)
/* update PAD_E_INPUT_OR_E_PWRD bit */
#define NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD	BIT(9)
/* Set SDMEMCOMP VREF sel values based on IO voltage */
#define NVQUIRK_SET_SDMEMCOMP_VREF_SEL		BIT(10)
#define NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION	BIT(11)
/* Set Calibration Offsets */
#define NVQUIRK_SET_CALIBRATION_OFFSETS		BIT(12)
/* Set tap delay */
#define NVQUIRK_SET_TAP_DELAY			BIT(13)
/* Set trim delay */
#define NVQUIRK_SET_TRIM_DELAY			BIT(14)
/* Enable Frequency Tuning for SDR50 mode */
#define NVQUIRK_ENABLE_SDR50_TUNING		BIT(15)

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
#define SDHCI_TEGRA_MAX_DQS_TRIM_VALUES	0x3F
#define MAX_DIVISOR_VALUE		128
#define DEFAULT_SDHOST_FREQ		50000000
#define SDHOST_MIN_FREQ			6000000
#define SDMMC_EMC_MAX_FREQ		150000000
#define TEGRA_SDHCI_MAX_PLL_SOURCE 2

/* Interface voltages */
#define SDHOST_1V8_OCR_MASK	0x8
#define SDHOST_HIGH_VOLT_MIN	2700000
#define SDHOST_HIGH_VOLT_MAX	3600000
#define SDHOST_HIGH_VOLT_2V8	2800000
#define SDHOST_LOW_VOLT_MIN	1800000
#define SDHOST_LOW_VOLT_MAX	1800000
#define SDHOST_HIGH_VOLT_3V2	3200000
#define SDHOST_HIGH_VOLT_3V3	3300000
#define SDHOST_MAX_VOLT_SUPPORT	3000000

static unsigned int sdmmc_emc_clinet_id[] = {
	TEGRA_BWMGR_CLIENT_SDMMC1,
	TEGRA_BWMGR_CLIENT_SDMMC2,
	TEGRA_BWMGR_CLIENT_SDMMC3,
	TEGRA_BWMGR_CLIENT_SDMMC4
};

enum tegra_regulator_config_ops {
	CONFIG_REG_GET,
	CONFIG_REG_EN,
	CONFIG_REG_DIS,
	CONFIG_REG_SET_VOLT,
};

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
	u32 nvquirks2;
	const char *parent_clk_list[TEGRA_SDHCI_MAX_PLL_SOURCE];
};

struct sdhci_tegra_pll_parent {
	struct clk *pll;
	unsigned long pll_rate;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	struct gpio_desc *power_gpio;
	const struct tegra_sdhci_platform_data *plat;
	struct tegra_bwmgr_client *emc_clk;
	bool	clk_enabled;
	/* ensure atomic set clock calls */
	struct mutex	set_clock_mutex;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	/* max ddr clk supported by the platform */
	unsigned int ddr_clk_limit;
	struct reset_control *rstc;
	unsigned int tuned_tap_delay;
	unsigned int tuning_status;
	#define TUNING_STATUS_DONE	1
	struct sdhci_tegra_pll_parent pll_source[TEGRA_SDHCI_MAX_PLL_SOURCE];
	bool is_parent_pll_source_1;
	struct regulator *vdd_io_reg;
	struct regulator *vdd_slot_reg;
	unsigned int vddio_min_uv;
	unsigned int vddio_max_uv;
	bool check_pad_ctrl_setting;
	struct tegra_prod_list *prods;
	int vddio_prev;
	bool is_rail_enabled;
	bool set_1v8_status;
	struct padctrl *sdmmc_padctrl;
	bool calib_1v8_offsets_done;
};

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci,
	unsigned char signal_voltage);
static int tegra_sdhci_configure_regulators(struct sdhci_host *sdhci,
	u8 option, int min_uV, int max_uV);
static inline int sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	int tap_delay, int type);
static inline int sdhci_tegra_set_dqs_trim_delay(struct sdhci_host *sdhci,
	int dqs_trim_delay);

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

static bool tegra_sdhci_is_tuning_done(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	if (tegra_host->tuning_status == TUNING_STATUS_DONE) {
		dev_info(mmc_dev(sdhci->mmc),
			"Tuning already done, restoring the best tap value : %u\n",
				tegra_host->tuned_tap_delay);
		sdhci_tegra_set_tap_delay(sdhci, tegra_host->tuned_tap_delay,
				SET_TUNED_TAP);
		return true;
	}
	return false;
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
static inline int sdhci_tegra_set_dqs_trim_delay(struct sdhci_host *sdhci,
	int dqs_trim_delay)
{
	u32 vend_ovrds;

	if ((dqs_trim_delay > SDHCI_TEGRA_MAX_DQS_TRIM_VALUES) &&
		(dqs_trim_delay < 0)) {
		dev_err(mmc_dev(sdhci->mmc), "Invalid dqs trim value\n");
		return -1;
	}
	vend_ovrds = sdhci_readl(sdhci, SDHCI_VNDR_CAP_OVERRIDES_0);
	vend_ovrds &= ~(SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_MASK <<
			SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_SHIFT);
	vend_ovrds |= (dqs_trim_delay <<
			SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_SHIFT);
	sdhci_writel(sdhci, vend_ovrds, SDHCI_VNDR_CAP_OVERRIDES_0);

	return 0;
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
	int tap_delay, int type)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u16 clk;
	bool card_clk_enabled;
	int err;
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
	if (type & (SET_DDR_TAP | SET_DEFAULT_TAP)) {
		if (type == SET_DDR_TAP)
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
				"tap-delay-ddr", tegra_host->prods);
		else
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
				"tap-delay-default", tegra_host->prods);
		if (err < 0)
			dev_err(mmc_dev(sdhci->mmc),
				"%s: error %d in prod settings\n",
				__func__, err);
	}
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
	int err;

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
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50_TUNING)
		vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDR50_TUNING;
	sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	/* Set the tap delay value */
	if (soc_data->nvquirks & NVQUIRK_SET_TAP_DELAY) {
		if (!tegra_sdhci_is_tuning_done(host))
			sdhci_tegra_set_tap_delay(host, plat->tap_delay,
					SET_DEFAULT_TAP);
	}
	/* Set the trim delay value */
	if (soc_data->nvquirks & NVQUIRK_SET_TRIM_DELAY) {
		err = tegra_prod_set_by_name(&host->ioaddr,
			"trim-delay-default", tegra_host->prods);
		if (err < 0)
			dev_err(mmc_dev(host->mmc),
			"%s: error %d in prod settings\n",__func__, err);
	}

	misc_ctrl = sdhci_readl(host, SDHCI_VNDR_MISC_CTRL);
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

/*
* Calculation of nearest clock frequency for desired rate:
* Get the divisor value, div = p / d_rate
* 1. If it is nearer to ceil(p/d_rate) then increment the div value by 0.5 and
* nearest_rate, i.e. result = p / (div + 0.5) = (p << 1)/((div << 1) + 1).
* 2. If not, result = p / div
* As the nearest clk freq should be <= to desired_rate,
* 3. If result > desired_rate then increment the div by 0.5
* and do, (p << 1)/((div << 1) + 1)
* 4. Else return result
* Here, If condtions 1 & 3 are both satisfied then to keep track of div value,
* defined index variable.
*/
static unsigned long get_nearest_clock_freq(unsigned long pll_rate,
		unsigned long desired_rate)
{
	unsigned long result;
	int div;
	int index = 1;

	if (pll_rate <= desired_rate)
		return pll_rate;

	div = pll_rate / desired_rate;
	if (div > MAX_DIVISOR_VALUE) {
		div = MAX_DIVISOR_VALUE;
		result = pll_rate / div;
	} else {
		if ((pll_rate % desired_rate) >= (desired_rate / 2))
			result = (pll_rate << 1) / ((div << 1) + index++);
		else
			result = pll_rate / div;

		if (desired_rate < result) {
			/*
			* Trying to get lower clock freq than desired clock,
			* by increasing the divisor value by 0.5
			*/
			result = (pll_rate << 1) / ((div << 1) + index);
		}
	}

	return result;
}

static void tegra_sdhci_clock_set_parent(struct sdhci_host *host,
		unsigned long desired_rate)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct clk *parent_clk;
	unsigned long pll_source_1_freq;
	unsigned long pll_source_2_freq;
	struct sdhci_tegra_pll_parent *pll_source = tegra_host->pll_source;
	int rc;

#ifdef CONFIG_TEGRA_FPGA_PLATFORM
	return;
#endif

	/*
	 * Currently pll_p and pll_c are used as clock sources for SDMMC. If clk
	 * rate is missing for either of them, then no selection is needed and
	 * the default parent is used.
	 */
	if (!pll_source[0].pll_rate || !pll_source[1].pll_rate)
		return;

	pll_source_1_freq = get_nearest_clock_freq(pll_source[0].pll_rate,
			desired_rate);
	pll_source_2_freq = get_nearest_clock_freq(pll_source[1].pll_rate,
			desired_rate);

	/*
	 * For low freq requests, both the desired rates might be higher than
	 * the requested clock frequency. In such cases, select the parent
	 * with the lower frequency rate.
	 */
	if ((pll_source_1_freq > desired_rate)
		&& (pll_source_2_freq > desired_rate)) {
		if (pll_source_2_freq <= pll_source_1_freq) {
			desired_rate = pll_source_2_freq;
			pll_source_1_freq = 0;
			parent_clk = pll_source[1].pll;
		} else {
			desired_rate = pll_source_1_freq;
			pll_source_2_freq = 0;
			parent_clk = pll_source[0].pll;
		}
		rc = clk_set_rate(pltfm_host->clk, desired_rate);
		goto set_parent_clk;
	}

	if (pll_source_1_freq > pll_source_2_freq) {
		if (!tegra_host->is_parent_pll_source_1) {
			parent_clk = pll_source[0].pll;
			tegra_host->is_parent_pll_source_1 = true;
			clk_set_rate(pltfm_host->clk, DEFAULT_SDHOST_FREQ);
		} else
			return;
	} else if (tegra_host->is_parent_pll_source_1) {
		parent_clk = pll_source[1].pll;
		tegra_host->is_parent_pll_source_1 = false;
		clk_set_rate(pltfm_host->clk, DEFAULT_SDHOST_FREQ);
	} else
		return;

set_parent_clk:
	rc = clk_set_parent(pltfm_host->clk, parent_clk);
	if (rc)
		pr_err("%s: failed to set pll parent clock %d\n",
			mmc_hostname(host->mmc), rc);
}

static void tegra_sdhci_set_clk_rate(struct sdhci_host *sdhci,
	unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int clk_rate;

	clk_rate = clk_get_rate(pltfm_host->clk);

	if (clk_rate == clock)
		return;

	/*
	 * In ddr mode, tegra sdmmc controller clock frequency
	 * should be double the card clock frequency.
	 */
	if ((sdhci->mmc->ios.timing == MMC_TIMING_UHS_DDR50) ||
		(sdhci->mmc->ios.timing == MMC_TIMING_MMC_DDR52)) {
		if (tegra_host->ddr_clk_limit &&
				(tegra_host->ddr_clk_limit < clock))
			clk_rate = tegra_host->ddr_clk_limit * 2;
		else
			clk_rate = clock * 2;
	} else
		clk_rate = clock;

	if (tegra_host->max_clk_limit && (clk_rate > tegra_host->max_clk_limit))
		clk_rate = tegra_host->max_clk_limit;

	if (clk_rate < SDHOST_MIN_FREQ)
		clk_rate = SDHOST_MIN_FREQ;

	tegra_sdhci_clock_set_parent(sdhci, clk_rate);
	clk_set_rate(pltfm_host->clk, clk_rate);
	sdhci->max_clk = clk_get_rate(pltfm_host->clk);
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
			if (tegra_host->emc_clk)
				tegra_bwmgr_set_emc(tegra_host->emc_clk,
				SDMMC_EMC_MAX_FREQ, TEGRA_BWMGR_SET_EMC_SHARED_BW);
		}
		tegra_sdhci_set_clk_rate(sdhci, clock);
		sdhci_set_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		sdhci_set_clock(sdhci, 0);
		vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
		vendor_ctrl &= ~0x1;
		sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
		clk_disable_unprepare(pltfm_host->clk);
		tegra_host->clk_enabled = false;
		if (tegra_host->emc_clk)
			tegra_bwmgr_set_emc(tegra_host->emc_clk, 0,
				TEGRA_BWMGR_SET_EMC_SHARED_BW);
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
	bool set_1v8_calib_offsets = false;
	unsigned int dqs_trim_delay;
	int err;
	/* Set the UHS signaling mode */
	sdhci_set_uhs_signaling(host, timing);

	switch (timing) {
	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_UHS_SDR12:
	case MMC_TIMING_UHS_SDR25:
	case MMC_TIMING_UHS_SDR50:
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
	set_1v8_calib_offsets = true;

		break;
	}

	if (timing == MMC_TIMING_MMC_HS400) {
		dqs_trim_delay = plat->dqs_trim_delay;
		sdhci_tegra_set_dqs_trim_delay(host, dqs_trim_delay);
	}

	/*
	 * Tegra SDMMC controllers support only a clock divisor of 2 in DDR
	 * mode. No other divisors are supported.
	 * Set the best tap value based on timing.
	 * Set trim delay if required.
	 */
	if ((timing == MMC_TIMING_UHS_DDR50) ||
		(timing == MMC_TIMING_MMC_DDR52)) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		sdhci_tegra_set_tap_delay(host, plat->ddr_tap_delay,
				SET_DDR_TAP);
		err = tegra_prod_set_by_name(&host->ioaddr,
			"trim-delay-ddr", tegra_host->prods);
		if (err < 0)
			dev_err(mmc_dev(host->mmc),
				"%s: error %d in prod settings\n",
				__func__, err);
	} else if (((timing == MMC_TIMING_MMC_HS200) ||
		(timing == MMC_TIMING_UHS_SDR104) ||
		(timing == MMC_TIMING_MMC_HS400) ||
		(timing == MMC_TIMING_UHS_SDR50)) &&
		(tegra_host->tuning_status == TUNING_STATUS_DONE)) {
		sdhci_tegra_set_tap_delay(host, tegra_host->tuned_tap_delay,
				SET_TUNED_TAP);
	} else {
		sdhci_tegra_set_tap_delay(host, tegra_host->plat->tap_delay,
				SET_DEFAULT_TAP);
	}

	if (set_1v8_calib_offsets && !tegra_host->calib_1v8_offsets_done) {
		tegra_sdhci_do_calibration(host,
			host->mmc->ios.signal_voltage);
		tegra_host->calib_1v8_offsets_done = true;
	}
}

static void tegra_sdhci_en_strobe(struct sdhci_host *host)
{
	u32 vndr_ctrl;

	vndr_ctrl = sdhci_readl(host, SDHCI_VNDR_SYS_SW_CTRL);
	vndr_ctrl |= (1 <<
		SDHCI_VNDR_SYS_SW_CTRL_STROBE_SHIFT);
	sdhci_writel(host, vndr_ctrl, SDHCI_VNDR_SYS_SW_CTRL);
}

/* Execute DLL calibration once for MMC device if it is
 * enumerated in HS400 mode at 200MHz clock freq before
 * starting any data transfer.
 */
static void tegra_sdhci_post_init(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u32 dll_cfg;
	unsigned timeout = 5;

	if ((sdhci->mmc->card->ext_csd.strobe_support) &&
			(sdhci->mmc->caps2 & MMC_CAP2_EN_STROBE) &&
			tegra_host->plat->en_strobe)
		tegra_sdhci_en_strobe(sdhci);

	dll_cfg = sdhci_readl(sdhci, SDHCI_VNDR_DLLCAL_CFG);
	dll_cfg |= SDHCI_VNDR_DLLCAL_CFG_EN_CALIBRATE;
	sdhci_writel(sdhci, dll_cfg, SDHCI_VNDR_DLLCAL_CFG);

	mdelay(1);

	/* Wait until the dll calibration is done */
	do {
		if (!(sdhci_readl(sdhci, SDHCI_VNDR_DLLCAL_CFG_STATUS) &
			SDHCI_VNDR_DLLCAL_CFG_STATUS_DLL_ACTIVE))
			break;

		mdelay(1);
		timeout--;
	} while (timeout);

	if (!timeout) {
		dev_err(mmc_dev(sdhci->mmc), "DLL calibration is failed\n");
	}
}

static void tegra_sdhci_configure_e_input(struct sdhci_host *sdhci, bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int err;

	if (enable)
		err = tegra_prod_set_by_name(&sdhci->ioaddr,
			"en-pad-ein-pwrd", tegra_host->prods);
	else
		err = tegra_prod_set_by_name(&sdhci->ioaddr,
			"dis-pad-ein-pwrd", tegra_host->prods);
	if (err < 0)
		dev_err(mmc_dev(sdhci->mmc),
			"%s: error %d in prod settings\n",__func__, err);
	udelay(1);
	return;
}

static int tegra_sdhci_configure_regulators(struct sdhci_host *sdhci,
	u8 option, int min_uV, int max_uV)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	int rc = 0;

	switch (option) {
	case CONFIG_REG_GET:
		tegra_host->vdd_io_reg = regulator_get(mmc_dev(sdhci->mmc),
			"vqmmc");
		if (IS_ERR_OR_NULL(tegra_host->vdd_io_reg)) {
			dev_info(mmc_dev(sdhci->mmc), "%s regulator not found: %ld",
				"vqmmc", PTR_ERR(tegra_host->vdd_io_reg));
			tegra_host->vdd_io_reg = NULL;
		}
		tegra_host->vdd_slot_reg = regulator_get(mmc_dev(sdhci->mmc),
			"vmmc");
		if (IS_ERR_OR_NULL(tegra_host->vdd_slot_reg)) {
			dev_info(mmc_dev(sdhci->mmc), "%s regulator not found: %ld",
				"vmmc" , PTR_ERR(tegra_host->vdd_slot_reg));
			tegra_host->vdd_slot_reg = NULL;
		}
	break;
	case CONFIG_REG_EN:
		if (!tegra_host->is_rail_enabled) {
			if (soc_data->nvquirks2 & NVQUIRK2_SET_PAD_E_INPUT_VOL)
				tegra_sdhci_configure_e_input(sdhci, true);
			if (tegra_host->vdd_slot_reg)
				rc = regulator_enable(tegra_host->vdd_slot_reg);
			if (tegra_host->vdd_io_reg)
				rc = regulator_enable(tegra_host->vdd_io_reg);
			tegra_host->is_rail_enabled = true;
		}
	break;
	case CONFIG_REG_DIS:
		if (tegra_host->is_rail_enabled) {
			if (tegra_host->vdd_io_reg)
				rc = regulator_disable(tegra_host->vdd_io_reg);
			if (tegra_host->vdd_slot_reg)
				rc = regulator_disable(
				tegra_host->vdd_slot_reg);
			tegra_host->is_rail_enabled = false;
		}
	break;
	case CONFIG_REG_SET_VOLT:

		if (tegra_host->vdd_io_reg)
			rc = regulator_set_voltage(tegra_host->vdd_io_reg,
				min_uV, max_uV);
	break;
	default:
		pr_err("Invalid argument passed to reg config %d\n", option);
	}

	return rc;
}

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci,
	unsigned char signal_voltage)
{
	unsigned int val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int timeout = 10;
	u16 clk;
	bool card_clk_enabled;
	int err = 0;
	unsigned int timing = sdhci->mmc->ios.timing;

	if (tegra_host->plat->disable_auto_cal)
		return;

	/*
	 * Do not enable auto calibration if the platform doesn't
	 * support it.
	 */
	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CALIBRATION))
		return;

	clk = sdhci_readw(sdhci, SDHCI_CLOCK_CONTROL);
	card_clk_enabled = clk & SDHCI_CLOCK_CARD_EN;
	if (card_clk_enabled) {
		clk &= ~SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
	}

	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD)
		tegra_sdhci_configure_e_input(sdhci, true);
	if (soc_data->nvquirks & NVQUIRK_SET_SDMEMCOMP_VREF_SEL) {
		if (signal_voltage == MMC_SIGNAL_VOLTAGE_330)
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
			"comp-vref-3v3", tegra_host->prods);
		else if (signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
			"comp-vref-1v8", tegra_host->prods);
	} else
		err = tegra_prod_set_by_name(&sdhci->ioaddr,
			"comp-vref-default", tegra_host->prods);
	if (err < 0)
		dev_err(mmc_dev(sdhci->mmc),
			"%s: error %d in prod settings\n",__func__, err);

	/* Wait for 1us after e_input is enabled*/
	if (soc_data->nvquirks2 & NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION)
		udelay(1);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (tegra_host->plat->enable_autocal_slew_override)
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_SLW_OVERRIDE;

	if (tegra_host->plat->auto_cal_step) {
		val &= ~(0x7 <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT);
		val |= (tegra_host->plat->auto_cal_step <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT);
	}
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

	switch (signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_180:
		if (timing == MMC_TIMING_UHS_SDR104)
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
				"prod-sdr104-1v8", tegra_host->prods);
		else
			err = tegra_prod_set_by_name(&sdhci->ioaddr,
				"prod-default-1v8", tegra_host->prods);
		break;
	case MMC_SIGNAL_VOLTAGE_330:
		err = tegra_prod_set_by_name(&sdhci->ioaddr,
				"prod-default-3v3", tegra_host->prods);
		break;
	}
	if (err < 0)
		dev_err(mmc_dev(sdhci->mmc),
			"error %d in prod settings\n", err);

	/* Wait for 2us after auto calibration is enabled*/
	if (soc_data->nvquirks2 & NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION)
		udelay(2);

	/* Wait until the calibration is done */
	do {
		if (!(sdhci_readl(sdhci, SDMMC_AUTO_CAL_STATUS) &
			SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE))
			break;

		mdelay(1);
		timeout--;
	} while (timeout);

	if (!timeout)
		dev_err(mmc_dev(sdhci->mmc), "Auto calibration failed\n");

	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD)
		tegra_sdhci_configure_e_input(sdhci, false);

	if (card_clk_enabled) {
		clk |= SDHCI_CLOCK_CARD_EN;
		sdhci_writew(sdhci, clk, SDHCI_CLOCK_CONTROL);
	}

}

static void tegra_sdhci_set_padctrl(struct sdhci_host *sdhci, int voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	int ret;

	if (plat->pwrdet_support && tegra_host->sdmmc_padctrl) {
		ret = padctrl_set_voltage(tegra_host->sdmmc_padctrl, voltage);
		if (ret)
			dev_err(mmc_dev(sdhci->mmc),
				"padctrl set volt failed %d\n", ret);
	}
}

static int tegra_sdhci_signal_voltage_switch(struct sdhci_host *sdhci,
	unsigned int signal_voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int min_uV = tegra_host->vddio_min_uv;
	unsigned int max_uV = tegra_host->vddio_max_uv;
	unsigned int rc = 0;
	u16 ctrl;
	unsigned int clock;

	ctrl = sdhci_readw(sdhci, SDHCI_HOST_CONTROL2);
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		ctrl |= SDHCI_CTRL_VDD_180;
		min_uV = SDHOST_LOW_VOLT_MIN;
		max_uV = SDHOST_LOW_VOLT_MAX;
	} else if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		if (ctrl & SDHCI_CTRL_VDD_180)
			ctrl &= ~SDHCI_CTRL_VDD_180;
	}

	/* Check if the slot can support the required voltage */
	if (min_uV > tegra_host->vddio_max_uv)
		return 0;

	clock = sdhci->clock;
	sdhci_set_clock(sdhci, 0);

	/* Set/clear the 1.8V signalling */
	sdhci_writew(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	if (soc_data->nvquirks2 & NVQUIRK2_SET_PAD_E_INPUT_VOL)
		tegra_sdhci_configure_e_input(sdhci, true);

	/* Switch the I/O rail voltage */
	dev_info(mmc_dev(sdhci->mmc),
		"setting min_voltage: %u, max_voltage: %u\n", min_uV, max_uV);
	rc = tegra_sdhci_configure_regulators(sdhci, CONFIG_REG_SET_VOLT,
		min_uV, max_uV);
	if (rc && (signal_voltage == MMC_SIGNAL_VOLTAGE_180)) {
		dev_err(mmc_dev(sdhci->mmc),
			"setting 1.8V failed %d. Revert to 3.3V\n", rc);
		tegra_host->set_1v8_status = false;
		signal_voltage = MMC_SIGNAL_VOLTAGE_330;
		rc = tegra_sdhci_configure_regulators(sdhci,
			CONFIG_REG_SET_VOLT, tegra_host->vddio_min_uv,
			tegra_host->vddio_max_uv);
	} else if ((!rc) && (signal_voltage == MMC_SIGNAL_VOLTAGE_180))
		tegra_host->set_1v8_status = true;

	if (gpio_is_valid(tegra_host->power_gpio)) {
		if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
			gpio_set_value(tegra_host->power_gpio, 1);
		} else {
			gpio_set_value(tegra_host->power_gpio, 0);
			mdelay(1000);
		}
	}

	/* Wait for the voltage to stabilize */
	mdelay(10);

	sdhci_set_clock(sdhci, clock);

	/* Wait 1 msec for clock to stabilize */
	mdelay(1);

	tegra_host->check_pad_ctrl_setting = true;

	return rc;
}

static void tegra_sdhci_pre_voltage_switch(struct sdhci_host *sdhci,
	unsigned char signal_voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int min_uV;

	if (!tegra_host || !(tegra_host->vdd_io_reg))
		return;

	min_uV = tegra_host->vddio_min_uv;
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		min_uV = SDHOST_LOW_VOLT_MIN;

	tegra_host->vddio_prev = regulator_get_voltage(tegra_host->vdd_io_reg);
	/* set pwrdet sdmmc1 before set 3.3 V */
	if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		if ((tegra_host->vddio_prev < min_uV) &&
			(min_uV >= SDHOST_HIGH_VOLT_2V8)) {
			tegra_sdhci_set_padctrl(sdhci,
				SDHOST_HIGH_VOLT_3V3);
		}
	}
}

static void tegra_sdhci_post_voltage_switch(struct sdhci_host *sdhci,
	unsigned char signal_voltage)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int voltage;
	bool set;

	if (!tegra_host || !(tegra_host->vdd_io_reg))
		return;

	if (tegra_host->check_pad_ctrl_setting) {
		voltage = regulator_get_voltage(tegra_host->vdd_io_reg);
		set = (signal_voltage == MMC_SIGNAL_VOLTAGE_180) ? true : false;
		if (set && (voltage <= tegra_host->vddio_prev) &&
				tegra_host->set_1v8_status) {
			tegra_sdhci_set_padctrl(sdhci, SDHOST_LOW_VOLT_MAX);
		}
	}
	tegra_sdhci_do_calibration(sdhci, signal_voltage);
	return;
}

static int tegra_sdhci_suspend(struct sdhci_host *sdhci)
{
	int ret = 0;

#ifndef CONFIG_ARCH_TEGRA_18x_SOC
	/* FIXME:
	 * Device hang is seen for t186 platform or removable card when
	 * clock is disabled (getting track in bug# 200107947). Keeping
	 * clock enabled during suspend for now to avoid device hang.
	 */
	if (!(tegra_host->plat->is_sd_device))
		tegra_sdhci_set_clock(sdhci, 0);
#endif
	ret = tegra_sdhci_configure_regulators(sdhci, CONFIG_REG_DIS, 0, 0);
	return ret;
}

static int tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret = 0;
	int signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);
	if (tegra_host->vddio_max_uv < SDHOST_HIGH_VOLT_MIN)
		signal_voltage = MMC_SIGNAL_VOLTAGE_180;
	ret = tegra_sdhci_configure_regulators(sdhci, CONFIG_REG_EN, 0, 0);
	if (!ret) {
		tegra_sdhci_pre_voltage_switch(sdhci, signal_voltage);
		ret = tegra_sdhci_signal_voltage_switch(sdhci, signal_voltage);
		tegra_sdhci_post_voltage_switch(sdhci, signal_voltage);
	}

	return ret;
}

static void tegra_sdhci_post_resume(struct sdhci_host *sdhci)
{
	bool dll_calib_req = false;

	dll_calib_req = (sdhci->mmc->card &&
		(sdhci->mmc->card->type == MMC_TYPE_MMC) &&
		(sdhci->mmc->ios.timing == MMC_TIMING_MMC_HS400));
	if (dll_calib_req)
		tegra_sdhci_post_init(sdhci);
}

static int sdhci_tegra_get_pll_from_dt(struct platform_device *pdev,
		const char **parent_clk_list, int size)
{
	struct device_node *np = pdev->dev.of_node;
	const char *pll_str;
	int i, cnt;

	if (!np)
		return -EINVAL;

	if (!of_find_property(np, "pll_source", NULL))
		return -ENXIO;

	cnt = of_property_count_strings(np, "pll_source");
	if (!cnt)
		return -EINVAL;

	if (cnt > size) {
		dev_warn(&pdev->dev,
			"pll list provide in DT exceeds max supported\n");
		cnt = size;
	}

	for (i = 0; i < cnt; i++) {
		of_property_read_string_index(np, "pll_source", i, &pll_str);
		parent_clk_list[i] = pll_str;
	}
	return 0;
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
	.post_init	= tegra_sdhci_post_init,
	.switch_signal_voltage = tegra_sdhci_signal_voltage_switch,
	.switch_signal_voltage_exit = tegra_sdhci_post_voltage_switch,
	.switch_signal_voltage_enter = tegra_sdhci_pre_voltage_switch,
	.suspend                = tegra_sdhci_suspend,
	.resume                 = tegra_sdhci_resume,
	.platform_resume	= tegra_sdhci_post_resume,
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
	.quirks2 = TEGRA_SDHCI_QUIRKS2 |
		SDHCI_QUIRK2_USE_64BIT_ADDR |
		SDHCI_QUIRK2_DDR_FIXED_DIVISOR,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra186 = {
	.pdata = &sdhci_tegra186_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_SET_SDMEMCOMP_VREF_SEL |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD,
	.nvquirks2 = NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION |
		NVQUIRK2_SET_PAD_E_INPUT_VOL,
};

static const struct sdhci_pltfm_data sdhci_tegra210_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
	.ops  = &tegra114_sdhci_ops,
};

static const struct sdhci_tegra_soc_data soc_data_tegra210 = {
	.pdata = &sdhci_tegra210_pdata,
	.nvquirks = NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_DDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra186-sdhci", .data = &soc_data_tegra186 },
	{ .compatible = "nvidia,tegra210-sdhci", .data = &soc_data_tegra210 },
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
	int val;

	if (!np)
		return -EINVAL;

	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	of_property_read_u32(np, "max-clk-limit", &plat->max_clk_limit);
	of_property_read_u32(np, "uhs-mask", &plat->uhs_mask);
	of_property_read_u32(np, "dqs-trim-delay", &plat->dqs_trim_delay);
	of_property_read_u32(np, "nvidia,ddr-tap-delay", &plat->ddr_tap_delay);
	of_property_read_u32(np, "nvidia,ddr-trim-delay", &plat->ddr_trim_delay);
	plat->pwrdet_support = of_property_read_bool(np, "pwrdet-support");
	plat->instance = of_alias_get_id(np, "sdhci");
	of_property_read_u32(np, "tap-delay", &plat->tap_delay);
	of_property_read_u32(np, "trim-delay", &plat->trim_delay);
	of_property_read_u32(np, "max-clk-limit", &plat->max_clk_limit);
	plat->en_strobe =
		of_property_read_bool(np, "nvidia,enable-strobe-mode");

	if (!of_property_read_u32(np, "mmc-ocr-mask", &val)) {
		if (val == 0)
			plat->ocr_mask = MMC_OCR_1V8_MASK;
		else if (val == 1)
			plat->ocr_mask = MMC_OCR_2V8_MASK;
		else if (val == 2)
			plat->ocr_mask = MMC_OCR_3V2_MASK;
		else if (val == 3)
			plat->ocr_mask = MMC_OCR_3V3_MASK;
	}
#ifdef CONFIG_MMC_CQ_HCI
	plat->enable_hw_cq =
		of_property_read_bool(np, "nvidia,enable-hwcq");
#endif
	plat->en_periodic_cflush = of_property_read_bool(np,
			"nvidia,en-periodic-cflush");
	if (plat->en_periodic_cflush) {
		val = 0;
		of_property_read_u32(np, "nvidia,periodic-cflush-to", &val);
		host->mmc->flush_timeout = val;
		if (val == 0) {
			plat->en_periodic_cflush = false;
			dev_warn(dev, "Periodic cache flush feature disabled,"
				"since flush timeout value is zero.\n");
		}
	}

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
	const char *parent_clk_list[TEGRA_SDHCI_MAX_PLL_SOURCE];
	int rc, i;
	int signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	for (i = 0; i < ARRAY_SIZE(parent_clk_list); i++)
		parent_clk_list[i] = NULL;
	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm_host = sdhci_priv(host);

	/* FIXME: This is for until dma-mask binding is supported in DT.
	 *        Set coherent_dma_mask for each Tegra SKUs.
	 *        If dma_mask is NULL, set it to coherent_dma_mask. */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

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

	/* check if DT provide list possible pll parents */
	if (sdhci_tegra_get_pll_from_dt(pdev,
		&parent_clk_list[0], ARRAY_SIZE(parent_clk_list))) {
		parent_clk_list[0] = soc_data->parent_clk_list[0];
		parent_clk_list[1] = soc_data->parent_clk_list[1];
	}
	for (i = 0; i < ARRAY_SIZE(parent_clk_list); i++) {
		if (!parent_clk_list[i])
			continue;

		tegra_host->pll_source[i].pll = devm_clk_get(&pdev->dev,
				parent_clk_list[i]);
		if (IS_ERR(tegra_host->pll_source[i].pll)) {
			rc = PTR_ERR(tegra_host->pll_source[i].pll);
			dev_err(mmc_dev(host->mmc),
					"clk[%d] error in getting %s: %d\n",
					i, parent_clk_list[i], rc);
			goto err_power_req;
		}
		tegra_host->pll_source[i].pll_rate =
			clk_get_rate(tegra_host->pll_source[i].pll);

		dev_info(mmc_dev(host->mmc), "Parent select= %s rate=%ld\n",
				parent_clk_list[i],
				tegra_host->pll_source[i].pll_rate);
	}

	tegra_host->prods = tegra_prod_get(&pdev->dev, NULL);
	if (IS_ERR_OR_NULL(tegra_host->prods)) {
		dev_info(&pdev->dev, "Prod-setting not available\n");
		tegra_host->prods = NULL;
	}
	if (plat->pwrdet_support) {
		tegra_host->sdmmc_padctrl = devm_padctrl_get(&pdev->dev, "sdmmc");
		if (IS_ERR(tegra_host->sdmmc_padctrl)) {
			rc = PTR_ERR(tegra_host->sdmmc_padctrl);\
			      tegra_host->sdmmc_padctrl = NULL;
			dev_err(&pdev->dev, "pad control get failed, error:%d\n", rc);
		}
	}

	mutex_init(&tegra_host->set_clock_mutex);
	clk = devm_clk_get(&pdev->dev, "sdmmc");
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}

	tegra_host->emc_clk =
		tegra_bwmgr_register(sdmmc_emc_clinet_id[plat->instance]);

	if (IS_ERR_OR_NULL(tegra_host->emc_clk))
		dev_err(mmc_dev(host->mmc),
			"Client registration for eMC failed\n");
	else
		dev_info(mmc_dev(host->mmc),
			"Client registration for eMC Successful\n");

	pltfm_host->clk = clk;
	/* enable clocks first time */
	rc = clk_prepare_enable(pltfm_host->clk);
	if (rc != 0)
		goto err_clk_put;

	/* Reset the sdhci controller to clear all previous status.*/

	tegra_host->rstc = devm_reset_control_get(&pdev->dev, "sdmmc");
	if (IS_ERR(tegra_host->rstc))
		pr_err("Reset for %s is failed\n", dev_name(&pdev->dev));
	else
		reset_control_reset(tegra_host->rstc);

	pltfm_host->priv = tegra_host;

	tegra_host->max_clk_limit = plat->max_clk_limit;
	host->mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;

	if (plat->ocr_mask & SDHOST_1V8_OCR_MASK) {
		tegra_host->vddio_min_uv = SDHOST_LOW_VOLT_MIN;
		tegra_host->vddio_max_uv = SDHOST_LOW_VOLT_MAX;
	} else if (plat->ocr_mask & MMC_OCR_2V8_MASK) {
		tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_2V8;
		tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	} else if (plat->ocr_mask & MMC_OCR_3V2_MASK) {
		tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_3V2;
		tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	} else if (plat->ocr_mask & MMC_OCR_3V3_MASK) {
		tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_3V3;
		tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	} else {
		/*
		 * Set the minV and maxV to default
		 * voltage range of 2.7V - 3.6V
		 */
		tegra_host->vddio_min_uv = SDHOST_HIGH_VOLT_MIN;
		tegra_host->vddio_max_uv = SDHOST_HIGH_VOLT_MAX;
	}

	rc = tegra_sdhci_configure_regulators(host, CONFIG_REG_GET, 0, 0);
	if (!rc) {
		tegra_sdhci_pre_voltage_switch(host, MMC_SIGNAL_VOLTAGE_330);
		if (tegra_host->vddio_max_uv < SDHOST_HIGH_VOLT_MIN)
			signal_voltage = MMC_SIGNAL_VOLTAGE_180;
		rc = tegra_sdhci_signal_voltage_switch(host, signal_voltage);
		if (rc) {
			dev_err(&pdev->dev,
				" voltage switch failed in probe, err: %d\n"
				, rc);
		} else {
			rc = tegra_sdhci_configure_regulators(host,
				CONFIG_REG_EN, 0, 0);
			if (rc)
				dev_err(&pdev->dev,
					" voltage enable failed in probe, err: %d\n"
					, rc);
			tegra_sdhci_post_voltage_switch(host, signal_voltage);
		}
	}
	if (plat->en_strobe)
		host->mmc->caps2 |= MMC_CAP2_EN_STROBE;

	if (plat->en_periodic_cflush)
		host->mmc->caps2 |= MMC_CAP2_PERIODIC_CACHE_FLUSH;

#ifdef CONFIG_MMC_CQ_HCI
	if (plat->enable_hw_cq) {
		host->mmc->caps2 |= MMC_CAP2_HW_CQ;
		host->cq_host = cmdq_pltfm_init(pdev);
		if (IS_ERR(host->cq_host))
			pr_err("CMDQ: Error in cmdq_platfm_init function\n");
		else
			pr_err("CMDQ: cmdq_platfm_init successful\n");
	}
#endif

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	return 0;

err_add_host:
	clk_disable_unprepare(pltfm_host->clk);
err_clk_put:
	if (pltfm_host->clk)
		clk_put(pltfm_host->clk);
err_clk_get:
err_power_req:
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
