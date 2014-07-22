/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/slab.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/mmc/sd.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/tegra_pm_domains.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>
#include <linux/dma-mapping.h>
#include <mach/pinmux-defines.h>

#include <asm/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/reboot.h>

#include <mach/hardware.h>

#include <linux/platform_data/mmc-sdhci-tegra.h>

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

#define SDHCI_VNDR_MISC_CTRL		0x120
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104		0x8
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50		0x10
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50		0x200
#define SDHCI_VENDOR_MISC_CNTRL_INFINITE_ERASE_TIMEOUT	0x1
#define SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK_SHIFT	17

#define SDHCI_VNDR_CAP_OVERRIDES_0			0x10c
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_SHIFT	8
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_MASK	0x3F

#define SDHCI_VNDR_TUN_CTRL				0x1c0
/* Enable Re-tuning request only when CRC error is detected
 * in SDR50/SDR104/HS200 modes
 */
#define SDHCI_VNDR_TUN_CTRL_RETUNE_REQ_EN		0x8000000

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF
#define SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK	0x80000000

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START	0x80000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET	0x70
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET	0x62

#define SDMMC_AUTO_CAL_STATUS	0x1EC
#define SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE	0x80000000

#define SDHCI_TEGRA_MAX_TAP_VALUES	0xFF
#define SDHCI_TEGRA_MAX_TRIM_VALUES	0x1F
#define DEFAULT_SDHOST_FREQ	50000000

/* Erratum: Version register is invalid in HW */
#define NVQUIRK_FORCE_SDHCI_SPEC_200		BIT(0)
/* Erratum: Enable block gap interrupt detection */
#define NVQUIRK_ENABLE_BLOCK_GAP_DET		BIT(1)
/* Enable SDHOST v3.0 support */
#define NVQUIRK_ENABLE_SDHCI_SPEC_300		BIT(2)
/* Enable SDR50 mode */
#define NVQUIRK_DISABLE_SDR50			BIT(3)
/* Enable SDR104 mode */
#define NVQUIRK_DISABLE_SDR104			BIT(4)
/* Enable DDR50 mode */
#define NVQUIRK_DISABLE_DDR50			BIT(5)
/* Do not enable auto calibration if the platform doesn't support */
#define NVQUIRK_DISABLE_AUTO_CALIBRATION	BIT(6)
/* Set Calibration Offsets */
#define NVQUIRK_SET_CALIBRATION_OFFSETS		BIT(7)
/* Set Drive Strengths */
#define NVQUIRK_SET_DRIVE_STRENGTH		BIT(8)
/* Enable PADPIPE CLKEN */
#define NVQUIRK_ENABLE_PADPIPE_CLKEN		BIT(9)
/* DISABLE SPI_MODE CLKEN */
#define NVQUIRK_DISABLE_SPI_MODE_CLKEN		BIT(10)
/* Set tap delay */
#define NVQUIRK_SET_TAP_DELAY			BIT(11)
/* Set trim delay */
#define NVQUIRK_SET_TRIM_DELAY			BIT(12)
/* Enable Frequency Tuning for SDR50 mode */
#define NVQUIRK_ENABLE_SDR50_TUNING		BIT(13)
/* Enable Infinite Erase Timeout*/
#define NVQUIRK_INFINITE_ERASE_TIMEOUT		BIT(14)
/* No Calibration for sdmmc4 */
#define NVQUIRK_DISABLE_SDMMC4_CALIB		BIT(15)
/* ENAABLE FEEDBACK IO CLOCK */
#define NVQUIRK_EN_FEEDBACK_CLK			BIT(16)
/* Disable AUTO CMD23 */
#define NVQUIRK_DISABLE_AUTO_CMD23		BIT(17)
/* update PAD_E_INPUT_OR_E_PWRD bit */
#define NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD	BIT(18)
/* Shadow write xfer mode reg and write it alongwith CMD register */
#define NVQUIRK_SHADOW_XFER_MODE_REG		BIT(19)
/* Set Pipe stages value o zero */
#define NVQUIRK_SET_PIPE_STAGES_MASK_0		BIT(20)
/* Disable SDMMC3 external loopback */
#define NVQUIRK_DISABLE_EXTERNAL_LOOPBACK	BIT(23)
/* Disable Timer Based Re-tuning mode */
#define NVQUIRK_DISABLE_TIMER_BASED_TUNING	BIT(24)
/* Set SDMEMCOMP VREF sel values based on IO voltage */
#define NVQUIRK_SET_SDMEMCOMP_VREF_SEL		BIT(25)

/* Max number of clock parents for sdhci is fixed to 2 */
#define TEGRA_SDHCI_MAX_PLL_SOURCE 2

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
	const char *parent_clk_list[TEGRA_SDHCI_MAX_PLL_SOURCE];
};

struct sdhci_tegra_sd_stats {
	unsigned int data_crc_count;
	unsigned int cmd_crc_count;
	unsigned int data_to_count;
	unsigned int cmd_to_count;
};

struct sdhci_tegra_pll_parent {
	struct clk *pll;
	unsigned long pll_rate;
};

#ifdef CONFIG_DEBUG_FS
struct dbg_cfg_data {
	unsigned int		tap_val;
	unsigned int		trim_val;
	bool			clk_ungated;
};
#endif
struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	int power_gpio;
	bool	clk_enabled;
	/* ensure atomic set clock calls */
	struct mutex		set_clock_mutex;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	/* max ddr clk supported by the platform */
	unsigned int ddr_clk_limit;
	struct sdhci_tegra_sd_stats *sd_stat_head;
	struct notifier_block reboot_notify;
	struct sdhci_tegra_pll_parent pll_source[TEGRA_SDHCI_MAX_PLL_SOURCE];
	bool is_parent_pll_source_1;
	bool set_1v8_calib_offsets;
#ifdef CONFIG_DEBUG_FS
	/* Override debug config data */
	struct dbg_cfg_data dbg_cfg;
#endif
	struct pinctrl_dev *pinctrl;
	int drive_group_sel;
};

static int show_error_stats_dump(struct seq_file *s, void *data)
{
	struct sdhci_host *host = s->private;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct sdhci_tegra_sd_stats *head;

	seq_printf(s, "ErrorStatistics:\n");
	seq_printf(s, "DataCRC\tCmdCRC\tDataTimeout\tCmdTimeout\n");
	head = tegra_host->sd_stat_head;
	if (head != NULL)
		seq_printf(s, "%d\t%d\t%d\t%d\n", head->data_crc_count,
			head->cmd_crc_count, head->data_to_count,
			head->cmd_to_count);
	return 0;
}

static int sdhci_error_stats_dump(struct inode *inode, struct file *file)
{
	return single_open(file, show_error_stats_dump, inode->i_private);
}

static const struct file_operations sdhci_host_fops = {
	.open		= sdhci_error_stats_dump,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}
#endif
	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
#endif

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
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
#endif
}

static void tegra_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (soc_data->nvquirks & NVQUIRK_SHADOW_XFER_MODE_REG) {
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
			pltfm_host->xfer_mode_shadow = 0;
			return;
		}
	}

	writew(val, host->ioaddr + reg);
}

static unsigned int tegra_sdhci_get_ro(struct sdhci_host *sdhci)
{
	return mmc_gpio_get_ro(host->mmc);
}

static inline int sdhci_tegra_set_dqs_trim_delay(struct sdhci_host *sdhci,
	u8 dqs_trim_delay)
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
	sdhci_writel(host, vend_ovrds, SDHCI_VNDR_CAP_OVERRIDES_0);
}

static inline int sdhci_tegra_set_trim_delay(struct sdhci_host *sdhci,
	u8 trim_delay)
{
	u32 vendor_ctrl;

	if ((trim_delay > SDHCI_TEGRA_MAX_TRIM_VALUES) && (trim_delay < 0)) {
		dev_err(mmc_dev(sdhci->mmc), "Invalid trim value\n");
		return -1;

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= (SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	vendor_ctrl |= (trim_delay << SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	return 0;
}

static inline int sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	u8 tap_delay)
{
	u32 vendor_ctrl;

	if ((tap_delay > SDHCI_TEGRA_MAX_TAP_VALUES) && (tap_delay < 0)){
		dev_err(mmc_dev(sdhci->mmc), "Invalid tap value\n");
		return -1;
	}

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= (SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

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

	if (tegra_host->sd_stat_head != NULL) {
		tegra_host->sd_stat_head->data_crc_count = 0;
		tegra_host->sd_stat_head->cmd_crc_count = 0;
		tegra_host->sd_stat_head->data_to_count = 0;
		tegra_host->sd_stat_head->cmd_to_count = 0;
	}

	vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE;
	/* Enable feedback/internal clock */
	if (soc_data->nvquirks & NVQUIRK_EN_FEEDBACK_CLK)
		vendor_ctrl &= ~SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK;
	else
		vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_INTERNAL_CLK;
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50_TUNING)
		vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDR50_TUNING;
	sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	/* Set the tap delay value */
	if (soc_data->nvquirks & NVQUIRK_SET_TAP_DELAY)
		sdhci_tegra_set_tap_delay(host, plat->tap_delay);
	/* Set the trim delay value */
	if (soc_data->nvquirks & NVQUIRK_SET_TRIM_DELAY)
		sdhci_tegra_set_trim_delay(host, plat->trim_delay);

	misc_ctrl = sdhci_readl(host, SDHCI_VNDR_MISC_CTRL);
	/* Erratum: Enable SDHCI spec v3.00 support */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SD_3_0)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_ENABLE_SD_3_0;
	/* Don't advertise UHS modes which aren't supported yet */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR104)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104_SUPPORT;
	else
		misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104_SUPPORT;
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50_SUPPORT;
	else
		misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50_SUPPORT;
	if (soc_data->nvquirks & NVQUIRK_ENABLE_DDR50)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50_SUPPORT;
	else
		misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50_SUPPORT;
	if (soc_data->nvquirks & NVQUIRK_INFINITE_ERASE_TIMEOUT)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_INFINITE_ERASE_TIMEOUT;
	if (soc_data->nvquirks & NVQUIRK_SET_PIPE_STAGES_MASK_0)
		misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_PIPE_STAGES_MASK;
	/* External loopback is valid for sdmmc3 only */
	if (soc_data->nvquirks & NVQUIRK_DISABLE_EXTERNAL_LOOPBACK)
		misc_ctrl &= ~(1 << SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK_SHIFT);
	else
		misc_ctrl |= (1 << SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK_SHIFT);
	sdhci_writel(host, misc_ctrl, SDHCI_VNDR_MISC_CTRL);

	if (soc_data->nvquirks &
		NVQUIRK_DISABLE_TIMER_BASED_TUNING) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL);
		vendor_ctrl |= SDHCI_VNDR_TUN_CTRL_RETUNE_REQ_EN;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_TUN_CTRL);
	}
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

static int tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int timing)
{
	u16 clk;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	/* Set the UHS signaling mode */
	sdhci_set_uhs_signaling(host, timing);

	if (uhs == MMC_TIMING_MMC_HS400)
		sdhci_tegra_set_dqs_trim_delay(host, plat->dqs_trim_delay);       

	/*
	 * Tegra SDMMC controllers support only a clock divisor of 2 in DDR
	 * mode. No other divisors are supported.
	 */
	if (timing == MMC_TIMING_UHS_DDR50) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		
		/* Set ddr mode tap delay */
		sdhci_tegra_set_tap_delay(host, plat->ddr_tap_delay);

		/* Set the ddr mode trim delay if required */
		sdhci_tegra_set_trim_delay(host, plat->ddr_trim_delay);
	}

	return 0;
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
		return ;

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
		} else {
			desired_rate = pll_source_1_freq;
			pll_source_2_freq = 0;
		}
		rc = clk_set_rate(pltfm_host->clk, desired_rate);
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

	/*
	 * In ddr mode, tegra sdmmc controller clock frequency
	 * should be double the card clock frequency.
	 */
	if (sdhci->mmc->card &&	mmc_card_ddr_mode(sdhci->mmc->card))
		clk_rate = clock * 2;
	else
		clk_rate = clock;

	if (tegra_host->max_clk_limit && (clk_rate > tegra_host->max_clk_limit))
		clk_rate = tegra_host->max_clk_limit;

	tegra_sdhci_clock_set_parent(sdhci, clk_rate);
	clk_set_rate(pltfm_host->clk, clk_rate);
	sdhci->max_clk = clk_get_rate(pltfm_host->clk);

	/* FPGA supports 26MHz of clock for SDMMC. */
	if (tegra_platform_is_fpga())
		sdhci->max_clk = 26000000;
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
		tegra_sdhci_set_clk_rate(sdhci, clock);
		if (!tegra_host->clk_enabled) {
			ret = clk_prepare_enable(pltfm_host->clk);
			if (ret) {
				dev_err(mmc_dev(sdhci->mmc),
					"clock enable is failed, ret: %d\n", ret);
				return;
			}
			vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
			vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
			tegra_host->clk_enabled = true;
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

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci,
	unsigned char signal_voltage)
{
	unsigned int val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int timeout = 10;
	unsigned int calib_offsets = 0;


	/*
	 * Do not enable auto calibration if the platform doesn't
	 * support it.
	 */
	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CALIBRATION))
		return;

	val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
	val &= ~SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK;
	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD)
		val |= SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK;
	if (soc_data->nvquirks & NVQUIRK_SET_SDMEMCOMP_VREF_SEL) {
		if (signal_voltage == MMC_SIGNAL_VOLTAGE_330)
			val |= tegra_host->plat->compad_vref_3v3;
		else if (signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			val |= tegra_host->plat->compad_vref_1v8;
	} else {
		val |= 0x7;
	}
	sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_CALIBRATION_OFFSETS)) {
		if (signal_voltage == MMC_SIGNAL_VOLTAGE_330)
			calib_offsets = tegra_host->plat->calib_3v3_offsets;
		else if (signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			calib_offsets = tegra_host->plat->calib_1v8_offsets;
		if (calib_offsets) {
			/* Program Auto cal PD offset(bits 8:14) */
			val &= ~(0x7F <<
				SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
			val |= (((calib_offsets >> 8) & 0xFF) <<
				SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
			/* Program Auto cal PU offset(bits 0:6) */
			val &= ~0x7F;
			val |= (calib_offsets & 0xFF);
		}
	}
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

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

	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD) {
		val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
		val &= ~SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK;
		sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);
	}

	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_DRIVE_STRENGTH)) {
		unsigned int pulldown_code;
		unsigned int pullup_code;
		unsigned long pin_config;
		int err;

		/* Disable Auto calibration */
		val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
		val &= ~SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
		sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

		if (tegra_host->pinctrl && tegra_host->drive_group_sel >= 0) {
			/* Get the pull down codes from auto cal status reg */
			pulldown_code = (
				sdhci_readl(sdhci, SDMMC_AUTO_CAL_STATUS) >>
				SDMMC_AUTO_CAL_STATUS_PULLDOWN_OFFSET);
			pin_config = TEGRA_PINCONF_PACK(
					TEGRA_PINCONF_PARAM_DRIVE_DOWN_STRENGTH,
					pulldown_code);
			err = pinctrl_set_config_for_group_sel(tegra_host->pinctrl,
					tegra_host->drive_group_sel, pin_config);
			if (err)
				dev_err(mmc_dev(sdhci->mmc),
				"Failed to set pulldown codes %d err %d\n",
				pulldown_code, err);

			/* Calculate the pull up codes */
			pullup_code = pulldown_code + PULLUP_ADJUSTMENT_OFFSET;
			if (pullup_code >= TEGRA_MAX_PULL)
				pullup_code = TEGRA_MAX_PULL - 1;
			pin_config = TEGRA_PINCONF_PACK(
					TEGRA_PINCONF_PARAM_DRIVE_UP_STRENGTH,
					pullup_code);
			/* Set the pull up code in the pinmux reg */
			err = pinctrl_set_config_for_group_sel(tegra_host->pinctrl,
					tegra_host->drive_group_sel, pin_config);
			if (err)
				dev_err(mmc_dev(sdhci->mmc),
				"Failed to set pullup codes %d err %d\n",
				pullup_code, err);
		}
	}
}

static int sdhci_tegra_sd_error_stats(struct sdhci_host *host, u32 int_status)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct sdhci_tegra_sd_stats *head = tegra_host->sd_stat_head;

	if (int_status & SDHCI_INT_DATA_CRC)
		head->data_crc_count = head->data_crc_count + 1;
	if (int_status & SDHCI_INT_CRC)
		head->cmd_crc_count = head->cmd_crc_count + 1;
	if (int_status & SDHCI_INT_TIMEOUT)
		head->cmd_to_count = head->cmd_to_count + 1;
	if (int_status & SDHCI_INT_DATA_TIMEOUT)
		head->data_to_count = head->data_to_count + 1;
	return 0;
}

static int tegra_sdhci_suspend(struct sdhci_host *sdhci)
{
	tegra_sdhci_set_clock(sdhci, 0);
}

static int tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);

	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER)
		tegra_sdhci_do_calibration(sdhci, MMC_SIGNAL_VOLTAGE_330);
}

static void sdhci_tegra_error_stats_debugfs(struct sdhci_host *host)
{
	struct dentry *root;

	root = debugfs_create_dir(dev_name(mmc_dev(host->mmc)), NULL);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err_root;

	host->debugfs_root = root;

	if (!debugfs_create_file("error_stats", S_IRUSR, root, host,
				&sdhci_host_fops))
		goto err_node;
	return;
err_node:
	debugfs_remove_recursive(root);
	host->debugfs_root = NULL;
err_root:
	pr_err("%s: Failed to initialize debugfs functionality\n", __func__);
	return;
}

static void tegra_sdhci_post_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	/* Turn OFF the clocks if removable card is not present */
	if (!(sdhci->mmc->caps & MMC_CAP_NONREMOVABLE) &&
		(mmc_gpio_get_cd(sdhci->mmc) != 0) && tegra_host->clk_enabled)
		tegra_sdhci_set_clock(sdhci, 0);
}

static const struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.write_w    = tegra_sdhci_writew,
	.set_clock  = tegra_sdhci_set_clock,
	.set_bus_width = tegra_sdhci_set_bus_width,
	.set_uhs_signaling = tegra_sdhci_set_uhs_signaling,
	.reset      = tegra_sdhci_reset,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.suspend		= tegra_sdhci_suspend,
	.resume			= tegra_sdhci_resume,
	.platform_resume	= tegra_sdhci_post_resume,
	.platform_reset_exit	= tegra_sdhci_reset_exit,
	.switch_signal_voltage_exit = tegra_sdhci_do_calibration,
	.sd_error_stats		= sdhci_tegra_sd_error_stats,
};

static const struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_BROKEN_PRESET_VALUES,
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
	.quirks2 = SDHCI_QUIRK2_BROKEN_PRESET_VALUES,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra30 = {
	.pdata = &sdhci_tegra30_pdata,
	.nvquirks = NVQUIRK_ENABLE_SDHCI_SPEC_300 |
		    NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct sdhci_pltfm_data sdhci_tegra114_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_BROKEN_PRESET_VALUES,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra114 = {
	.pdata = &sdhci_tegra114_pdata,
	.nvquirks = NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_DDR50 |
		    NVQUIRK_DISABLE_SDR104 |
		    NVQUIRK_SHADOW_XFER_MODE_REG |
		    NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD,
};

static struct sdhci_pltfm_data sdhci_tegra210_pdata = {
	.quirks = TEGRA_SDHCI_QUIRKS,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
		   SDHCI_QUIRK2_NON_STD_VOLTAGE_SWITCHING |
		   SDHCI_QUIRK2_NO_CALC_MAX_DISCARD_TO |
		   SDHCI_QUIRK2_REG_ACCESS_REQ_HOST_CLK |
		   SDHCI_QUIRK2_SUPPORT_64BIT_DMA,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra210 = {
	.pdata = &sdhci_tegra21_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS |
		    NVQUIRK_SET_TRIM_DELAY |
		    NVQUIRK_ENABLE_DDR50 |
		    NVQUIRK_ENABLE_AUTO_CMD23 |
		    NVQUIRK_INFINITE_ERASE_TIMEOUT |
		    NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		    NVQUIRK_SET_CALIBRATION_OFFSETS |
		    NVQUIRK_DISABLE_TIMER_BASED_TUNING |
		    NVQUIRK_DISABLE_EXTERNAL_LOOPBACK,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra210-sdhci", .data = &soc_data_tegra210 },
	{ .compatible = "nvidia,tegra124-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra114-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra30 },
	{ .compatible = "nvidia,tegra20-sdhci", .data = &soc_data_tegra20 },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tegra_dt_match);

static int sdhci_tegra_parse_dt(struct device *dev) {
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tegra_sdhci_platform_data *plat;
	int val;
	struct property *prop;
	const __be32 *p;
	u32 u;
	int i = 0;

	if (!np)
		return NULL;

	tegra_host->power_gpio = of_get_named_gpio(np, "power-gpios", 0);
	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		dev_err(&pdev->dev, "Can't allocate platform data\n");
		return NULL;
	}

	if (of_find_property(np, "edp_support", NULL)) {
		plat->edp_support = true;
		of_property_for_each_u32(np, "edp_states", prop, p, u) {
			if (i == SD_EDP_NUM_STATES)
				break;
			plat->edp_states[i] = u;
			i++;
		}
		p = NULL;
		prop = NULL;
	}

	of_property_read_u32(np, "tap-delay", &plat->tap_delay);
	of_property_read_u32(np, "trim-delay", &plat->trim_delay);
	of_property_read_u32(np, "ddr-trim-delay", &plat->ddr_trim_delay);
	of_property_read_u32(np, "dqs-trim-delay", &plat->dqs_trim_delay);
	of_property_read_u32(np, "ddr-clk-limit", &plat->ddr_clk_limit);
	of_property_read_u32(np, "max-clk-limit", &plat->max_clk_limit);
	of_property_read_u32(np, "uhs_mask", &plat->uhs_mask);
	of_property_read_u32(np, "compad-vref-3v3", &plat->compad_vref_3v3);
	of_property_read_u32(np, "compad-vref-1v8", &plat->compad_vref_1v8);
	of_property_read_u32(np, "calib_3v3_offsets", &plat->calib_3v3_offsets);
	of_property_read_u32(np, "calib_1v8_offsets", &plat->calib_1v8_offsets);

	return mmc_of_parse(host->mmc);
}

static void tegra_sdhci_rail_off(struct sdhci_tegra *tegra_host)
{
	/*
	 * Fix me: Tegra sdhci regulators are no longer used. So, either
	 * move reboot notifier to sdhci driver or remove the notifier.
	 */
}

static int show_disableclkgating_value(void *data, u64 *value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		struct sdhci_tegra *tegra_host = pltfm_host->priv;
		if (tegra_host != NULL)
			*value = tegra_host->dbg_cfg.clk_ungated;
	}
	return 0;
}

static int set_disableclkgating_value(void *data, u64 value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		if (pltfm_host != NULL) {
			struct sdhci_tegra *tegra_host = pltfm_host->priv;
			if (tegra_host != NULL) {
				if (value) {
					host->mmc->ops->set_ios(host->mmc,
						&host->mmc->ios);
					tegra_host->dbg_cfg.clk_ungated = true;
				} else {
					tegra_host->dbg_cfg.clk_ungated = false;
				}
			}
		}
	}
	return 0;
}

static int set_trim_override_value(void *data, u64 value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		if (pltfm_host != NULL) {
			struct sdhci_tegra *tegra_host = pltfm_host->priv;
			if (tegra_host != NULL) {
				/* Make sure clock gating is disabled */
				if ((tegra_host->dbg_cfg.clk_ungated) &&
				(tegra_host->clk_enabled)) {
					sdhci_tegra_set_trim_delay(host, value);
					tegra_host->dbg_cfg.trim_val =
						value;
				} else {
					pr_info("%s: Disable clock gating before setting value\n",
						mmc_hostname(host->mmc));
				}
			}
		}
	}
	return 0;
}

static int show_trim_override_value(void *data, u64 *value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		if (pltfm_host != NULL) {
			struct sdhci_tegra *tegra_host = pltfm_host->priv;
			if (tegra_host != NULL)
				*value = tegra_host->dbg_cfg.trim_val;
		}
	}
	return 0;
}

static int show_tap_override_value(void *data, u64 *value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		if (pltfm_host != NULL) {
			struct sdhci_tegra *tegra_host = pltfm_host->priv;
			if (tegra_host != NULL)
				*value = tegra_host->dbg_cfg.tap_val;
		}
	}
	return 0;
}

static int set_tap_override_value(void *data, u64 value)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	if (host != NULL) {
		struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
		if (pltfm_host != NULL) {
			struct sdhci_tegra *tegra_host = pltfm_host->priv;
			if (tegra_host != NULL) {
				/* Make sure clock gating is disabled */
				if ((tegra_host->dbg_cfg.clk_ungated) &&
				(tegra_host->clk_enabled)) {
					sdhci_tegra_set_tap_delay(host, value);
					tegra_host->dbg_cfg.tap_val = value;
				} else {
					pr_info("%s: Disable clock gating before setting value\n",
						mmc_hostname(host->mmc));
				}
			}
		}
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(sdhci_polling_period_fops, show_polling_period,
		set_polling_period, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_active_load_high_threshold_fops,
		show_active_load_high_threshold,
		set_active_load_high_threshold, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_disable_clkgating_fops,
		show_disableclkgating_value,
		set_disableclkgating_value, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_override_trim_data_fops,
		show_trim_override_value,
		set_trim_override_value, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_override_tap_data_fops,
		show_tap_override_value,
		set_tap_override_value, "%llu\n");

static void sdhci_tegra_error_stats_debugfs(struct sdhci_host *host)
{
	struct dentry *root = host->debugfs_root;
	struct dentry *dfs_root;
	unsigned saved_line;

	if (!root) {
		root = debugfs_create_dir(dev_name(mmc_dev(host->mmc)), NULL);
		if (IS_ERR_OR_NULL(root)) {
			saved_line = __LINE__;
			goto err_root;
		}
		host->debugfs_root = root;
	}

	if (!debugfs_create_file("error_stats", S_IRUSR, root, host,
				&sdhci_host_fops)) {
		saved_line = __LINE__;
		goto err_node;
	}

	dfs_root = debugfs_create_dir("override_data", root);
	if (IS_ERR_OR_NULL(dfs_root)) {
		saved_line = __LINE__;
		goto err_node;
	}

	if (!debugfs_create_file("clk_gate_disabled", 0644,
				dfs_root, (void *)host,
				&sdhci_disable_clkgating_fops)) {
		saved_line = __LINE__;
		goto err_node;
	}

	if (!debugfs_create_file("tap_value", 0644,
				dfs_root, (void *)host,
				&sdhci_override_tap_data_fops)) {
		saved_line = __LINE__;
		goto err_node;
	}

	if (!debugfs_create_file("trim_value", 0644,
				dfs_root, (void *)host,
				&sdhci_override_trim_data_fops)) {
		saved_line = __LINE__;
		goto err_node;
	}

	return;
err_node:
	debugfs_remove_recursive(root);
	host->debugfs_root = NULL;
err_root:
	pr_err("%s %s: Failed to initialize debugfs functionality at line=%d\n", __func__,
		mmc_hostname(host->mmc), saved_line);
	return;
}

static int tegra_sdhci_reboot_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct sdhci_tegra *tegra_host =
		container_of(nb, struct sdhci_tegra, reboot_notify);

	switch (event) {
	case SYS_RESTART:
	case SYS_POWER_OFF:
		tegra_sdhci_rail_off(tegra_host);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static int tegra_sdhci_get_drive_strength(struct sdhci_host *sdhci,
		unsigned int max_dtr, int host_drv, int card_drv)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	return plat->default_drv_type;
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

static int sdhci_tegra_init_pinctrl_info(struct device *dev,
		struct sdhci_tegra *tegra_host,
		struct tegra_sdhci_platform_data *plat)
{
	struct device_node *np = dev->of_node;
	const char *drive_gname;

	if (!np)
		return 0;

	tegra_host->pinctrl = pinctrl_get_dev_from_of_property(np,
					"drive-pin-pinctrl");
	if (!tegra_host->pinctrl)
		 return -EINVAL;

	drive_gname = of_get_property(np, "drive-pin-name", 0);
	tegra_host->drive_group_sel = pinctrl_get_selector_from_group_name(
					tegra_host->pinctrl, drive_gname);
	return 0;
}

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tegra *tegra_host;
	struct tegra_sdhci_platform_data *plat;
	struct clk *clk;
	const char *parent_clk_list[TEGRA_SDHCI_MAX_PLL_SOURCE];
	int rc;

	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm_host = sdhci_priv(host);
	plat = pdev->dev.platform_data;

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

	tegra_host->plat = plat;

	sdhci_tegra_init_pinctrl_info(&pdev->dev, tegra_host, plat);

	tegra_host->sd_stat_head = devm_kzalloc(&pdev->dev,
		sizeof(struct sdhci_tegra_sd_stats), GFP_KERNEL);
	if (!tegra_host->sd_stat_head) {
		dev_err(mmc_dev(host->mmc), "failed to allocate sd_stat_head\n");
		rc = -ENOMEM;
		goto err_power_req;
	}
	tegra_host->soc_data = soc_data;
	pltfm_host->priv = tegra_host;

	rc = sdhci_tegra_parse_dt(&pdev->dev);
	if (rc)
		goto err_parse_dt;
	/* check if DT provide list possible pll parents */
	if (sdhci_tegra_get_pll_from_dt(pdev,
		parent_clk_list, ARRAY_SIZE(parent_clk_list))) {
		parent_clk_list[0] = soc_data->parent_clk_list[0];
		parent_clk_list[1] = soc_data->parent_clk_list[1];
	}

	for (i = 0; i < ARRAY_SIZE(parent_clk_list); i++) {
		if (!parent_clk_list[i])
			continue;
		tegra_host->pll_source[i].pll = clk_get_sys(NULL,
				parent_clk_list[i]);
		if (IS_ERR(tegra_host->pll_source[i].pll)) {
			rc = PTR_ERR(tegra_host->pll_source[i].pll);
			dev_err(mmc_dev(host->mmc),
					"clk error in getting %s: %d\n",
					parent_clk_list[i], rc);
		}
		tegra_host->pll_source[i].pll_rate =
			clk_get_rate(tegra_host->pll_source[i].pll);

		dev_info(mmc_dev(host->mmc), "Parent select= %s rate=%ld\n",
				parent_clk_list[i], tegra_host->pll_source[i].pll_rate);
	}

	if (gpio_is_valid(tegra_host->power_gpio)) {
		rc = gpio_request(tegra_host->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			goto err_power_req;
		}
		gpio_direction_output(tegra_host->power_gpio, 1);
	}

	pll_c = clk_get_sys(NULL, "pll_c");
	if (IS_ERR(pll_c)) {
		rc = PTR_ERR(pll_c);
		dev_err(mmc_dev(host->mmc),
			"clk error in getting pll_c: %d\n", rc);
	}

	pll_p = clk_get_sys(NULL, "pll_p");
	if (IS_ERR(pll_p)) {
		rc = PTR_ERR(pll_p);
		dev_err(mmc_dev(host->mmc),
			"clk error in getting pll_p: %d\n", rc);
	}

	pll_c_rate = clk_get_rate(pll_c);
	pll_p_rate = clk_get_rate(pll_p);


	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}

	if (clk_get_parent(pltfm_host->clk) == tegra_host->pll_source[0].pll)
		tegra_host->is_parent_pll_source_1 = true;

	rc = clk_prepare_enable(clk);
	if (rc != 0)
		goto err_clk_put;
	pltfm_host->clk = clk;
	pltfm_host->priv = tegra_host;
	tegra_host->clk_enabled = true;
	mutex_init(&tegra_host->set_clock_mutex);

	tegra_host->max_clk_limit = plat->max_clk_limit;

	host->mmc->pm_caps |= plat->pm_caps;
	host->mmc->pm_flags |= plat->pm_flags;

	/* disable access to boot partitions */
	host->mmc->caps2 |= MMC_CAP2_BOOTPART_NOACC;

	tegra_pd_add_device(&pdev->dev);
	host->mmc->caps2 |= MMC_CAP2_PACKED_CMD;
	rc = sdhci_add_host(host);
	sdhci_tegra_error_stats_debugfs(host);
	if (rc)
		goto err_add_host;

	/* Enable async suspend/resume to reduce LP0 latency */
	device_enable_async_suspend(&pdev->dev);

	if (plat->power_off_rail) {
		tegra_host->reboot_notify.notifier_call =
			tegra_sdhci_reboot_notify;
		register_reboot_notifier(&tegra_host->reboot_notify);
	}
#ifdef CONFIG_DEBUG_FS
	tegra_host->dbg_cfg.tap_val =
		plat->tap_delay;
	tegra_host->dbg_cfg.trim_val =
		plat->ddr_trim_delay;
	tegra_host->dbg_cfg.clk_ungated =
		plat->disable_clock_gate;
#endif
	return 0;

err_add_host:
	clk_disable_unprepare(pltfm_host->clk);
err_clk_put:
	clk_put(pltfm_host->clk);
err_clk_get:
	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_free(tegra_host->power_gpio);
err_power_req:
err_parse_dt:
err_alloc_tegra_host:
	sdhci_pltfm_free(pdev);
	return rc;
}

static int sdhci_tegra_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	sdhci_remove_host(host, dead);

	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_free(tegra_host->power_gpio);

	if (tegra_host->clk_enabled)
		clk_disable_unprepare(pltfm_host->clk);
	clk_put(pltfm_host->clk);

	if (plat->power_off_rail)
		unregister_reboot_notifier(&tegra_host->reboot_notify);

	sdhci_pltfm_free(pdev);

	return 0;
}

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
