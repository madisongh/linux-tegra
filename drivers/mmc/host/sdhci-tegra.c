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

#include <asm/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/reboot.h>

#include <mach/hardware.h>
#include <mach/pm_domains.h>

#include <linux/platform_data/mmc-sdhci-tegra.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
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
/* Shadow write xfer mode reg and write it alongwith CMD register */
#define NVQUIRK_SHADOW_XFER_MODE_REG	BIT(6)
#define NVQUIRK_DISABLE_AUTO_CALIBRATION	BIT(7)
#define NVQUIRK_SET_CALIBRATION_OFFSETS	BIT(8)

#define SDHCI_VENDOR_CLOCK_CNTRL       0x100
#define SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT		16
#define SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_MASK			0xFF
#define SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT		24
#define SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_MASK		0x1F
#define SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE		0x8
#define SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VENDOR_CLOCK_CNTRL_INPUT_IO_CLK			0x2
#define SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK			0x1

#define SDHCI_VENDOR_MISC_CNTRL		0x120
#define SDHCI_VENDOR_MISC_CNTRL_SDMMC_SPARE0_ENABLE_SD_3_0	0x20
#define SDHCI_VENDOR_MISC_CNTRL_INFINITE_ERASE_TIMEOUT	0x1

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF

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

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
};

struct sdhci_tegra_sd_stats {
	unsigned int data_crc_count;
	unsigned int cmd_crc_count;
	unsigned int data_to_count;
	unsigned int cmd_to_count;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	int power_gpio;
	bool	clk_enabled;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
	/* max ddr clk supported by the platform */
	unsigned int ddr_clk_limit;
	struct sdhci_tegra_sd_stats *sd_stat_head;
	struct notifier_block reboot_notify;
	bool is_parent_pllc;
};

static struct clk *pll_c;
static struct clk *pll_p;
static unsigned long pll_c_rate;
static unsigned long pll_p_rate;

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

static inline int sdhci_tegra_set_trim_delay(struct sdhci_host *sdhci,
	u8 trim_delay)
{
	u32 vendor_ctrl;

	if ((trim_delay > SDHCI_TEGRA_MAX_TRIM_VALUES) && (trim_delay < 0)) {
		dev_err(mmc_dev(sdhci->mmc), "Invalid trim value\n");
		return -1;

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= (SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_MASK <<
			SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT);
	vendor_ctrl |= (trim_delay << SDHCI_VENDOR_CLOCK_CNTRL_TRIM_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

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

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
	vendor_ctrl &= (SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_MASK <<
			SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VENDOR_CLOCK_CNTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);

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

	/* Set the tap delay value */
	tegra_sdhci_set_tap_delay(host, plat->tap_delay);
	/* Set the trim delay value */
	tegra_sdhci_set_trim_delay(host, plat->trim_delay);

	vendor_ctrl = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLOCK_CNTRL);
	vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_PADPIPE_CLKEN_OVERRIDE;
	vendor_ctrl &= ~SDHCI_VENDOR_CLOCK_CNTRL_SPI_MODE_CLKEN_OVERRIDE;
	vendor_ctrl &=  ~SDHCI_VENDOR_CLOCK_CNTRL_INPUT_IO_CLK;
	sdhci_writel(host, SDHCI_TEGRA_VENDOR_CLOCK_CNTRL);

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
	/* Enable infinite erase timeout */
	misc_ctrl |= SDHCI_VENDOR_MISC_CNTRL_INFINITE_ERASE_TIMEOUT;
	sdhci_writew(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);
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
	u32 vndr_ctrl;

	/* Set the UHS signaling mode */
	sdhci_set_uhs_signaling(host, timing);

	/*
	 * Tegra SDMMC controllers support only a clock divisor of 2 in DDR
	 * mode. No other divisors are supported.
	 */
	if (timing == MMC_TIMING_UHS_DDR50) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

		/* Set the ddr mode trim delay if required */
		if (plat->ddr_trim_delay != -1)
			sdhci_tegra_set_trim_value(host, plat->ddr_trim_delay);
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
	unsigned long pll_c_freq;
	unsigned long pll_p_freq;
	int rc;

	pll_c_freq = (pll_c_rate >= desired_rate) ?
		get_nearest_clock_freq(pll_c_rate, desired_rate) : pll_c_rate;
	pll_p_freq = (pll_p_rate >= desired_rate) ?
		get_nearest_clock_freq(pll_p_rate, desired_rate) : pll_p_rate;

	if (pll_c_freq > pll_p_freq) {
		if (!tegra_host->is_parent_pllc) {
			parent_clk = pll_c;
			tegra_host->is_parent_pllc = true;
			clk_set_rate(pltfm_host->clk, DEFAULT_SDHOST_FREQ);
		} else
			return;
	} else if (tegra_host->is_parent_pllc) {
		parent_clk = pll_p;
		tegra_host->is_parent_pllc = false;
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

	pr_debug("%s %s %u enabled=%u\n", __func__,
		mmc_hostname(sdhci->mmc), clock, tegra_host->clk_enabled);

	if (clock) {
		tegra_sdhci_set_clk_rate(sdhci, clock);
		if (!tegra_host->clk_enabled) {
			clk_prepare_enable(pltfm_host->clk);
			vendor_ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
			vendor_ctrl |= SDHCI_VENDOR_CLOCK_CNTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
			tegra_host->clk_enabled = true;
		}
		sdhci_set_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		sdhci_set_clock(sdhci, 0);
		vendor_ctrl = sdhci_readb(sdhci, SDHCI_VENDOR_CLOCK_CNTRL);
		vendor_ctrl &= ~0x1;
		sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VENDOR_CLOCK_CNTRL);
		clk_disable_unprepare(pltfm_host->clk);
		tegra_host->clk_enabled = false;
	}
}
static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci)
{
	unsigned int val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int timeout = 10;

	/*
	 * Do not enable auto calibration if the platform doesn't
	 * support it.
	 */
	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CALIBRATION))
		return;

	val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
	val &= ~SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK;
	val |= 0x7;
	sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_CALIBRATION_OFFSETS)) {
		/* Program Auto cal PD offset(bits 8:14) */
		val &= ~(0x7F <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		val |= (SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
		/* Program Auto cal PU offset(bits 0:6) */
		val &= ~0x7F;
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET;
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

	/* Disable Auto calibration */
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val &= ~SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);
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
		    NVQUIRK_SHADOW_XFER_MODE_REG,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra210-sdhci", .data = &soc_data_tegra114 },
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
	of_property_read_u32(np, "ddr-clk-limit", &plat->ddr_clk_limit);
	of_property_read_u32(np, "max-clk-limit", &plat->max_clk_limit);

	return mmc_of_parse(host->mmc);
}

static void tegra_sdhci_rail_off(struct sdhci_tegra *tegra_host)
{
	/*
	 * Fix me: Tegra sdhci regulators are no longer used. So, either
	 * move reboot notifier to sdhci driver or remove the notifier.
	 */
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

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tegra *tegra_host;
	struct tegra_sdhci_platform_data *plat;
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
	plat = pdev->dev.platform_data;

	tegra_host = devm_kzalloc(&pdev->dev, sizeof(*tegra_host), GFP_KERNEL);
	if (!tegra_host) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra_host\n");
		rc = -ENOMEM;
		goto err_alloc_tegra_host;
	}

	tegra_host->plat = plat;
	tegra_host->sd_stat_head =
		devm_kzalloc(&pdev->dev, sizeof(struct sdhci_tegra_sd_stats),
			     GFP_KERNEL);
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

	if (clk_get_parent(clk) == pll_c)
		tegra_host->is_parent_pllc = true;

	rc = clk_prepare_enable(clk);
	if (rc != 0)
		goto err_clk_put;
	pltfm_host->clk = clk;
	pltfm_host->priv = tegra_host;
	tegra_host->clk_enabled = true;
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
