/*
 * Copyright (C) 2010 Google, Inc.
 *
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/uaccess.h>
#include <linux/ktime.h>

#include <asm/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/reboot.h>

#include <linux/clk/tegra.h>

#include <mach/hardware.h>

#include <linux/platform_data/mmc-sdhci-tegra.h>
#include <linux/platform/tegra/common.h>

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
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104_SUPPORT	0x8
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50_SUPPORT	0x10
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50_SUPPORT	0x200
#define SDHCI_VNDR_MISC_CTRL_INFINITE_ERASE_TIMEOUT	0x1
#define SDHCI_VNDR_MISC_CTRL_PIPE_STAGES_MASK		0x180
#define SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK_SHIFT	17

#define SDHCI_VNDR_SYS_SW_CTRL				0x104
#define SDHCI_VNDR_SYS_SW_CTRL_WR_CRC_USE_TMCLK		0x40000000
#define SDHCI_VNDR_SYS_SW_CTRL_STROBE_SHIFT		31

#define SDHCI_VNDR_CAP_OVERRIDES_0			0x10c
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_SHIFT	8
#define SDHCI_VNDR_CAP_OVERRIDES_0_DQS_TRIM_MASK	0x3F

#define SDMMC_VNDR_IO_TRIM_CTRL_0	0x1AC
#define SDMMC_VNDR_IO_TRIM_CTRL_0_SEL_VREG_MASK	0x4

#define SDHCI_VNDR_DLLCAL_CFG				0x1b0
#define SDHCI_VNDR_DLLCAL_CFG_EN_CALIBRATE		0x80000000

#define SDHCI_VNDR_DLL_CTRL0_0				0x1b4
#define SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_SHIFT		7
#define SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_MASK		0x7F
#define SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_OFFSET		0x7C


#define SDHCI_VNDR_DLLCAL_CFG_STATUS			0x1bc
#define SDHCI_VNDR_DLLCAL_CFG_STATUS_DLL_ACTIVE		0x80000000

#define SDHCI_VNDR_TUN_CTRL0_0				0x1c0
/*MUL_M is defined in [12:6] bits*/
#define SDHCI_VNDR_TUN_CTRL0_0_MUL_M			0x1FC0
/* To Set value of [12:6] as 1 */
#define SDHCI_VNDR_TUN_CTRL0_0_MUL_M_VAL		0x40
#define SDHCI_VNDR_TUN_CTRL1_0				0x1c4
#define SDHCI_VNDR_TUN_STATUS0_0			0x1c8
/* Enable Re-tuning request only when CRC error is detected
 * in SDR50/SDR104/HS200 modes
 */
#define SDHCI_VNDR_TUN_CTRL_RETUNE_REQ_EN		0x8000000
#define SDHCI_VNDR_TUN_CTRL0_TUN_HW_TAP			0x20000
#define TUNING_WORD_SEL_MASK 				0x7
/*value 4 in 13 to 15 bits indicates 256 iterations*/
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_MASK	0x7
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT	13
/* Value 1 in NUM_TUNING_ITERATIONS indicates 64 iterations */
#define HW_TUNING_64_TRIES				1
/* Value 2 in NUM_TUNING_ITERATIONS indicates 128 iterations */
#define HW_TUNING_128_TRIES				2
/* Value 4 in NUM_TUNING_ITERATIONS indicates 256 iterations */
#define HW_TUNING_256_TRIES				4

#define SDHCI_VNDR_TUN_CTRL1_TUN_STEP_SIZE		0x77

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF
#define SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK	0x80000000

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START	0x80000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_SLW_OVERRIDE	0x10000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET	0x70
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PU_OFFSET	0x62
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT	0x10

#define SDMMC_AUTO_CAL_STATUS	0x1EC
#define SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE	0x80000000
#define SDMMC_AUTO_CAL_STATUS_PULLDOWN_OFFSET	24
#define PULLUP_ADJUSTMENT_OFFSET	20

#define SDMMC_VENDOR_ERR_INTR_STATUS_0	0x108

#define SDMMC_IO_SPARE_0	0x1F0
#define SPARE_OUT_3_OFFSET	19


/* Erratum: Version register is invalid in HW */
#define NVQUIRK_FORCE_SDHCI_SPEC_200		BIT(0)
/* Erratum: Enable block gap interrupt detection */
#define NVQUIRK_ENABLE_BLOCK_GAP_DET		BIT(1)
/* Enable SDHOST v3.0 support */
#define NVQUIRK_ENABLE_SDHCI_SPEC_300		BIT(2)
/* Enable SDR50 mode */
#define NVQUIRK_ENABLE_SDR50			BIT(3)
/* Enable SDR104 mode */
#define NVQUIRK_ENABLE_SDR104			BIT(4)
/* Enable DDR50 mode */
#define NVQUIRK_ENABLE_DDR50			BIT(5)
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
#define NVQUIRK_DISABLE_EXTERNAL_LOOPBACK	BIT(21)
/* Disable Timer Based Re-tuning mode */
#define NVQUIRK_DISABLE_TIMER_BASED_TUNING	BIT(22)
/* Set SDMEMCOMP VREF sel values based on IO voltage */
#define NVQUIRK_SET_SDMEMCOMP_VREF_SEL		BIT(23)
#define NVQUIRK_UPDATE_PAD_CNTRL_REG		BIT(24)
#define NVQUIRK_UPDATE_PIN_CNTRL_REG		BIT(25)
/* Use timeout clk for write crc status data timeout counter */
#define NVQUIRK_USE_TMCLK_WR_CRC_TIMEOUT	BIT(26)
/* Enable T210 specific SDMMC WAR - Tuning Step Size, Tuning Iterations*/
#define NVQUIRK_UPDATE_HW_TUNING_CONFG		BIT(27)
/* Enable Enhanced strobe mode support */
#define NVQUIRK_EN_STROBE_SUPPORT		BIT(28)
/*controller does not support cards if 1.8 V is not supported by cards*/
#define NVQUIRK_BROKEN_SD2_0_SUPPORT		BIT(29)
#define NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH	BIT(30)

#define NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION	BIT(0)

/* Common quirks for Tegra 12x and later versions of sdmmc controllers */
#define TEGRA_SDHCI_QUIRKS (SDHCI_QUIRK_BROKEN_TIMEOUT_VAL | \
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK | \
		  SDHCI_QUIRK_SINGLE_POWER_WRITE | \
		  SDHCI_QUIRK_NO_HISPD_BIT | \
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC | \
		  SDHCI_QUIRK_BROKEN_CARD_DETECTION | \
		  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC)

#define TEGRA_SDHCI_QUIRKS2 (SDHCI_QUIRK2_PRESET_VALUE_BROKEN | \
	SDHCI_QUIRK2_HOST_OFF_CARD_ON | \
	SDHCI_QUIRK2_SUPPORT_64BIT_DMA | \
	SDHCI_QUIRK2_USE_64BIT_ADDR | \
	SDHCI_QUIRK2_DISABLE_CARD_CLOCK_FIRST)

#define TEGRA_SDHCI_NVQUIRKS (NVQUIRK_ENABLE_PADPIPE_CLKEN | \
				NVQUIRK_DISABLE_SPI_MODE_CLKEN | \
				NVQUIRK_EN_FEEDBACK_CLK | \
				NVQUIRK_ENABLE_SDR50 | \
				NVQUIRK_ENABLE_DDR50 | \
				NVQUIRK_ENABLE_SDR104 | \
				NVQUIRK_SET_TAP_DELAY | \
				NVQUIRK_SET_TRIM_DELAY | \
				NVQUIRK_ENABLE_SDR50_TUNING | \
				NVQUIRK_SHADOW_XFER_MODE_REG)

/* max limit defines */
#define SDHCI_TEGRA_MAX_TAP_VALUES	0xFF
#define SDHCI_TEGRA_MAX_TRIM_VALUES	0x1F
#define SDHCI_TEGRA_MAX_DQS_TRIM_VALUES	0x3F
#define MAX_DIVISOR_VALUE		128
#define DEFAULT_SDHOST_FREQ		50000000
/* Max number of clock parents for sdhci is fixed to 2 */
#define TEGRA_SDHCI_MAX_PLL_SOURCE 2

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
	u32 nvquirks2;
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
	const struct tegra_sdhci_platform_data *plat;
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
	struct pinctrl *pinctrl_sdmmc;
	struct pinctrl_state *schmitt_enable[2];
	struct pinctrl_state *schmitt_disable[2];
	struct pinctrl_state *drv_code_strength;
	struct pinctrl_state *default_drv_code_strength;
	struct pinctrl_state *sdmmc_pad_ctrl[MMC_TIMINGS_MAX_MODES];
	int drive_group_sel;
	bool en_strobe;
	unsigned int tuned_tap_delay;
	struct padctrl *sdmmc_padctrl;
	ktime_t timestamp;
	bool limit_vddio_max_volt;
};

static unsigned long get_nearest_clock_freq(unsigned long pll_rate,
		unsigned long desired_rate);
static inline int sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	int tap_delay);
static inline int sdhci_tegra_set_trim_delay(struct sdhci_host *sdhci,
	int trim_delay);
static inline int sdhci_tegra_set_dqs_trim_delay(struct sdhci_host *sdhci,
	int dqs_trim_delay);
static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci,
	unsigned char signal_voltage);
static void tegra_sdhci_update_sdmmc_pinctrl_register(struct sdhci_host *sdhci,
		bool set);
static void tegra_sdhci_config_tap(struct sdhci_host *sdhci, u8 option);
static void vendor_trim_clear_sel_vreg(struct sdhci_host *host, bool enable);
static void sdhci_tegra_select_drive_strength(struct sdhci_host *host,
		unsigned int uhs);
static void tegra_sdhci_get_clock_freq_for_mode(struct sdhci_host *sdhci,
			unsigned int *clock);

static void tegra_sdhci_dumpregs(struct sdhci_host *sdhci)
{
	u32 tap_delay;
	u32 trim_delay;
	u32 reg, val;
	int i;

	/* print tuning windows */
	for (i = 0; i <= TUNING_WORD_SEL_MASK; i++) {
		reg = sdhci_readl(sdhci, SDHCI_VNDR_TUN_CTRL0_0);
		reg &= ~TUNING_WORD_SEL_MASK;
		reg |= i;
		sdhci_writel(sdhci, reg, SDHCI_VNDR_TUN_CTRL0_0);
		val = sdhci_readl(sdhci, SDHCI_VNDR_TUN_STATUS0_0);
		pr_info("%s: tuning_window[%d]: %#x\n",
		mmc_hostname(sdhci->mmc), i, val);
	}
	tap_delay = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	trim_delay = tap_delay;
	tap_delay >>= SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT;
	tap_delay &= SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK;
	trim_delay >>= SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT;
	trim_delay &= SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK;
	pr_info("sdhci: Tap value: %u | Trim value: %u\n", tap_delay,
			trim_delay);
	pr_info("sdhci: SDMMC Interrupt status: 0x%08x\n", sdhci_readl(sdhci,
				SDMMC_VENDOR_ERR_INTR_STATUS_0));
}

static int sdhci_tegra_get_max_tuning_loop_counter(struct sdhci_host *sdhci)
{
	u16 hw_tuning_iterations;
	u32 vendor_ctrl;

	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_SDR50)
		hw_tuning_iterations = HW_TUNING_256_TRIES;
	else if (sdhci->mmc->caps2 & MMC_CAP2_HS533)
		hw_tuning_iterations = HW_TUNING_64_TRIES;
	else
		hw_tuning_iterations = HW_TUNING_128_TRIES;

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_TUN_CTRL0_0);
	vendor_ctrl &=	~(SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_MASK <<
			SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT);
	vendor_ctrl |= (hw_tuning_iterations <<
			SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0_0);

	return 257;
}

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
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
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
	return mmc_gpio_get_ro(sdhci->mmc);
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
	vendor_ctrl &= (SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	vendor_ctrl |= (trim_delay << SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	return 0;
}

static inline int sdhci_tegra_set_tap_delay(struct sdhci_host *sdhci,
	int tap_delay)
{
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

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_TUN_CTRL0_0);
	vendor_ctrl &= ~SDHCI_VNDR_TUN_CTRL0_TUN_HW_TAP;
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0_0);

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= (SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK <<
			SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_TUN_CTRL0_0);
	vendor_ctrl |= SDHCI_VNDR_TUN_CTRL0_TUN_HW_TAP;
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0_0);
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
		vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK;
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
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDHCI_SPEC_300)
		misc_ctrl |= SDHCI_VNDR_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
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

	if (soc_data->nvquirks & NVQUIRK_UPDATE_PAD_CNTRL_REG) {
		misc_ctrl = sdhci_readl(host, SDMMC_IO_SPARE_0);
		misc_ctrl |= (1 << SPARE_OUT_3_OFFSET);
		sdhci_writel(host, misc_ctrl, SDMMC_IO_SPARE_0);
	}

	/* SEL_VREG should be 0 for all modes*/
	if (soc_data->nvquirks &
		NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH)
		vendor_trim_clear_sel_vreg(host, true);

	if (soc_data->nvquirks & NVQUIRK_UPDATE_HW_TUNING_CONFG) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL0_0);
		vendor_ctrl &= ~(SDHCI_VNDR_TUN_CTRL0_0_MUL_M);
		vendor_ctrl |= SDHCI_VNDR_TUN_CTRL0_0_MUL_M_VAL;
		vendor_ctrl |= SDHCI_VNDR_TUN_CTRL_RETUNE_REQ_EN;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0_0);

		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL1_0);
		vendor_ctrl &= ~(SDHCI_VNDR_TUN_CTRL1_TUN_STEP_SIZE);
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_TUN_CTRL1_0);
	}

	/* Use timeout clk data timeout counter for generating wr crc status */
	if (soc_data->nvquirks &
		NVQUIRK_USE_TMCLK_WR_CRC_TIMEOUT) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_SYS_SW_CTRL);
		vendor_ctrl |= SDHCI_VNDR_SYS_SW_CTRL_WR_CRC_USE_TMCLK;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_SYS_SW_CTRL);
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

void tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int timing)
{
	u16 clk;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;

	/* Set the UHS signaling mode */
	sdhci_set_uhs_signaling(host, timing);
	sdhci_tegra_select_drive_strength(host, timing);

	if (timing == MMC_TIMING_MMC_HS400)
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
}

static void vendor_trim_clear_sel_vreg(struct sdhci_host *host, bool enable)
{
	unsigned int misc_ctrl;
	unsigned int wait_usecs;

	misc_ctrl = sdhci_readl(host, SDMMC_VNDR_IO_TRIM_CTRL_0);
	if (enable) {
		misc_ctrl &= ~(SDMMC_VNDR_IO_TRIM_CTRL_0_SEL_VREG_MASK);
		wait_usecs = 3;
	} else {
		misc_ctrl |= (SDMMC_VNDR_IO_TRIM_CTRL_0_SEL_VREG_MASK);
		wait_usecs = 1;
	}
	sdhci_writel(host, misc_ctrl, SDMMC_VNDR_IO_TRIM_CTRL_0);
	udelay(wait_usecs);
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

static void tegra_sdhci_get_clock_freq_for_mode(struct sdhci_host *sdhci,
			unsigned int *clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	unsigned int ios_timing = sdhci->mmc->ios.timing;
	unsigned int index;

	if (!(plat->is_fix_clock_freq) || (ios_timing >= MMC_TIMINGS_MAX_MODES))
		return;

	/*
	* Index 0 is for ID mode and rest mapped with index being ios timings.
	* If the frequency for some particular mode is set as 0 then return
	* without updating the clock
	*/
	if (*clock <= 400000)
		index = 0;
	else
		index = ios_timing + 1;

	if (plat->fixed_clk_freq_table[index] != 0)
		*clock = plat->fixed_clk_freq_table[index];
	else
		pr_warn("%s: The fixed_clk_freq_table entry for ios timing %d is 0. So using clock rate as requested by card\n",
			mmc_hostname(sdhci->mmc), ios_timing);
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
	if (sdhci->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
		if (tegra_host->ddr_clk_limit &&
				(tegra_host->ddr_clk_limit < clock))
			clk_rate = tegra_host->ddr_clk_limit * 2;
		else
			clk_rate = clock * 2;
	} else
		clk_rate = clock;

	tegra_sdhci_get_clock_freq_for_mode(sdhci, &clk_rate);
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
	const struct tegra_sdhci_platform_data *plat = tegra_host->plat;
	int ret = 0;
	ktime_t cur_time;
	s64 period_time;

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
				mutex_unlock(&tegra_host->set_clock_mutex);
				return;
			}
			vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
			vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDMMC_CLK;
			sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
			tegra_host->clk_enabled = true;
			if (tegra_host->soc_data->nvquirks &
				NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH) {
				/* power up / active state */
				vendor_trim_clear_sel_vreg(sdhci, true);
			}
		}
		if (plat->en_periodic_calib &&
			sdhci->is_calibration_done) {
			cur_time = ktime_get();
			period_time = ktime_to_ms(ktime_sub(cur_time,
						tegra_host->timestamp));
			if (period_time >= SDHCI_PERIODIC_CALIB_TIMEOUT)
				tegra_sdhci_do_calibration(sdhci,
						sdhci->mmc->ios.signal_voltage);
		}
		sdhci_set_clock(sdhci, clock);
	} else if (!clock && tegra_host->clk_enabled) {
		sdhci_set_clock(sdhci, 0);
		if (tegra_host->soc_data->nvquirks &
			NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH){
			/* power down / idle state */
			vendor_trim_clear_sel_vreg(sdhci, false);
		}
		vendor_ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
		vendor_ctrl &= ~0x1;
		sdhci_writeb(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
		clk_disable_unprepare(pltfm_host->clk);
		tegra_host->clk_enabled = false;
	}
	mutex_unlock(&tegra_host->set_clock_mutex);
}

static void tegra_sdhci_en_strobe(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u32 vndr_ctrl;

	vndr_ctrl = sdhci_readl(host, SDHCI_VNDR_SYS_SW_CTRL);
	vndr_ctrl |= (1 <<
		SDHCI_VNDR_SYS_SW_CTRL_STROBE_SHIFT);
	sdhci_writel(host, vndr_ctrl, SDHCI_VNDR_SYS_SW_CTRL);
	tegra_host->en_strobe = true;
}

static void tegra_sdhci_post_init(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u32 dll_cfg;
	u32 dll_ctrl0;
	unsigned timeout = 5;

	if ((sdhci->mmc->card->ext_csd.strobe_support) &&
			(sdhci->mmc->caps2 & MMC_CAP2_EN_STROBE) &&
			tegra_host->plat->en_strobe)
		tegra_sdhci_en_strobe(sdhci);

	/* Program TX_DLY_CODE_OFFSET Value for HS533 mode*/
	if (mmc_card_hs533(sdhci->mmc->card)) {
		dll_ctrl0 = sdhci_readl(sdhci, SDHCI_VNDR_DLL_CTRL0_0);
		dll_ctrl0 &= ~(SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_MASK <<
			SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_SHIFT);
		dll_ctrl0 |= ((SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_OFFSET &
				SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_MASK) <<
			SDHCI_VNDR_DLL_CTRL0_0_TX_DLY_SHIFT);
		sdhci_writel(sdhci, dll_ctrl0, SDHCI_VNDR_DLL_CTRL0_0);
	}

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

static void tegra_sdhci_update_sdmmc_pinctrl_register(struct sdhci_host *sdhci,
	bool set)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct pinctrl_state *set_schmitt[2];
	int ret;
	int i;

	if (set) {
		set_schmitt[0] = tegra_host->schmitt_enable[0];
		set_schmitt[1] = tegra_host->schmitt_enable[1];

		if (!IS_ERR_OR_NULL(tegra_host->drv_code_strength)) {
			ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
				tegra_host->drv_code_strength);
			if (ret < 0)
				dev_warn(mmc_dev(sdhci->mmc),
				"setting drive code strength failed\n");
		}
	} else {
		set_schmitt[0] = tegra_host->schmitt_disable[0];
		set_schmitt[1] = tegra_host->schmitt_disable[1];

		if (!IS_ERR_OR_NULL(tegra_host->default_drv_code_strength)) {
			ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
				tegra_host->default_drv_code_strength);
			if (ret < 0)
				dev_warn(mmc_dev(sdhci->mmc),
				"setting default drive code strength failed\n");
		}
	}

	for (i = 0; i < 2; i++) {
		if (IS_ERR_OR_NULL(set_schmitt[i]))
			continue;
		ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
				set_schmitt[i]);
		if (ret < 0)
			dev_warn(mmc_dev(sdhci->mmc),
				"setting schmitt state failed\n");
	}
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
	bool set;

	if (tegra_host->plat->disable_auto_cal)
		return;
	set = (signal_voltage == MMC_SIGNAL_VOLTAGE_180) ? true : false;
	tegra_sdhci_update_sdmmc_pinctrl_register(sdhci, set);

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

	/* Wait for 1us after e_input is enabled*/
	if (soc_data->nvquirks2 & NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION)
		udelay(1);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (tegra_host->plat->enable_autocal_slew_override)
		val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_SLW_OVERRIDE;
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
	if (tegra_host->plat->auto_cal_step) {
		val &= ~(0x7 <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT);
		val |= (tegra_host->plat->auto_cal_step <<
			SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_STEP_OFFSET_SHIFT);
	}
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

	/* Wait for 1us after auto calibration is enabled*/
	if (soc_data->nvquirks2 & NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION)
		udelay(1);

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

	if (tegra_host->plat->en_periodic_calib) {
		tegra_host->timestamp = ktime_get();
		sdhci->timestamp = ktime_get();
		sdhci->is_calibration_done = true;
	}
}

static int tegra_sdhci_validate_sd2_0(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhci->mmc));
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	struct tegra_sdhci_platform_data *plat;

	plat = pdev->dev.platform_data;

	if ((soc_data->nvquirks & NVQUIRK_BROKEN_SD2_0_SUPPORT) &&
		(tegra_host->limit_vddio_max_volt)) {
		/* T210: Bug 1561291
		 * Design issue where a cap connected to IO node is stressed
		 * to 3.3v while it can only tolerate up to 1.8v.
		 */
		dev_err(mmc_dev(sdhci->mmc),
			"SD cards with out 1.8V is not supported\n");
		return -EPERM;
	} else {
		return 0;
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

	return 0;
}

static int tegra_sdhci_resume(struct sdhci_host *sdhci)
{
	/* Setting the min identification clock of freq 400KHz */
	tegra_sdhci_set_clock(sdhci, 400000);

	if (sdhci->mmc->pm_flags & MMC_PM_KEEP_POWER)
		tegra_sdhci_do_calibration(sdhci, MMC_SIGNAL_VOLTAGE_330);

	return 0;
}

static void tegra_sdhci_post_resume(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	bool dll_calib_req = false;

	dll_calib_req = (sdhci->mmc->card &&
		(sdhci->mmc->card->type == MMC_TYPE_MMC) &&
		(sdhci->mmc->ios.timing == MMC_TIMING_MMC_HS400));
	if (dll_calib_req)
		tegra_sdhci_post_init(sdhci);

	/* Turn OFF the clocks if removable card is not present */
	if (!(sdhci->mmc->caps & MMC_CAP_NONREMOVABLE) &&
		(mmc_gpio_get_cd(sdhci->mmc) != 0) && tegra_host->clk_enabled)
		tegra_sdhci_set_clock(sdhci, 0);
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
					host->ops->set_clock(host,
						host->mmc->ios.clock);
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

/*
 * Simulate the card remove and insert
 * set req to true to insert the card
 * set req to false to remove the card
 */
static int sdhci_tegra_carddetect(struct sdhci_host *sdhost, bool req)
{
	struct platform_device *pdev = to_platform_device(mmc_dev(sdhost->mmc));
	struct tegra_sdhci_platform_data *plat;
	bool card_present = false;
	int err = 0;

	plat = pdev->dev.platform_data;

	if (!(sdhost->mmc->caps & MMC_CAP_NONREMOVABLE))
		if (mmc_gpio_get_cd(sdhost->mmc) == 0)
			card_present = true;
	/*
	 *check if card is inserted physically before performing
	 *virtual remove or insertion
	 */
	if (!card_present) {
		err = -ENXIO;
		dev_err(mmc_dev(sdhost->mmc),
				"Card not inserted in slot\n");
		goto err_config;
	}

	/* Ignore the request if card already in requested state*/
	if (card_present == req) {
		dev_info(mmc_dev(sdhost->mmc),
				"Card already in requested state\n");
		goto err_config;
	}

	sdhost->mmc->trigger_card_event = true;
	mmc_detect_change(sdhost->mmc, msecs_to_jiffies(200));
err_config:
	return err;
};

static int get_card_insert(void *data, u64 *val)
{
	struct sdhci_host *sdhost = data;

	*val = (mmc_gpio_get_cd(sdhost->mmc) == 0);

	return 0;
}

static int set_card_insert(void *data, u64 val)
{
	struct sdhci_host *sdhost = data;
	int err = 0;

	if (val > 1) {
		err = -EINVAL;
		dev_err(mmc_dev(sdhost->mmc),
			"Usage error. Use 0 to remove, 1 to insert %d\n", err);
		goto err_detect;
	}

	if (sdhost->mmc->caps & MMC_CAP_NONREMOVABLE) {
		err = -EINVAL;
		dev_err(mmc_dev(sdhost->mmc),
		    "usage error, Supports only SDCARD hosts only %d\n", err);
		goto err_detect;
	}

	err = sdhci_tegra_carddetect(sdhost, val == 1);
err_detect:
	return err;
}

static ssize_t get_bus_timing(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct sdhci_host *host = file->private_data;
	unsigned int len = 0;
	char buf[16];

	static const char *const sdhci_tegra_timing[] = {
		[MMC_TIMING_LEGACY]	= "legacy",
		[MMC_TIMING_MMC_HS]	= "highspeed",
		[MMC_TIMING_SD_HS]	= "highspeed",
		[MMC_TIMING_UHS_SDR12]	= "SDR12",
		[MMC_TIMING_UHS_SDR25]	= "SDR25",
		[MMC_TIMING_UHS_SDR50]	= "SDR50",
		[MMC_TIMING_UHS_SDR104]	= "SDR104",
		[MMC_TIMING_UHS_DDR50]	= "DDR50",
		[MMC_TIMING_MMC_HS200]	= "HS200",
		[MMC_TIMING_MMC_HS400]	= "HS400",
	};

	len = snprintf(buf, sizeof(buf), "%s\n",
			sdhci_tegra_timing[host->mmc->ios.timing]);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t set_bus_timing(struct file *file,
				 const char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct sdhci_host *sdhost = file->private_data;
	char buf[16];
	int err = 0;
	u32 mask = 0;
	u32 timing_req = 0;

	/* Ignore the request if card is not yet removed*/
	if (mmc_gpio_get_cd(sdhost->mmc) == 0) {
		dev_err(mmc_dev(sdhost->mmc),
		    "Sdcard not removed. Set bus timing denied\n");
		err = -EINVAL;
		goto err_detect;
	}

	if (copy_from_user(buf, userbuf, min(count, sizeof(buf)))) {
		err = -EFAULT;
		goto err_detect;
	}

	buf[count-1] = '\0';

	/*prepare the temp mask to mask higher host timing modes wrt user
	 *requested one
	 */
	mask = ~(MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_DDR50
			    | MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25
			    | MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104);
	if (strcmp(buf, "highspeed") == 0) {
		timing_req = MMC_CAP_SD_HIGHSPEED;
		mask |= MMC_CAP_SD_HIGHSPEED;
	} else if (strcmp(buf, "SDR12") == 0) {
		timing_req = MMC_CAP_UHS_SDR12;
		mask |= (MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12);
	} else if (strcmp(buf, "SDR25") == 0) {
		timing_req = MMC_CAP_UHS_SDR25;
		mask |= (MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12
			| MMC_CAP_UHS_SDR25);
	} else if (strcmp(buf, "SDR50") == 0) {
		timing_req = MMC_CAP_UHS_SDR50;
		mask |= (MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12
			| MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50);
	} else if (strcmp(buf, "SDR104") == 0) {
		timing_req = MMC_CAP_UHS_SDR104;
		mask |= (MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12
			| MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50
			| MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_DDR50);
	} else if (strcmp(buf, "DDR50") == 0) {
		timing_req = MMC_CAP_UHS_DDR50;
		mask |= (MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12
			| MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50
			| MMC_CAP_UHS_DDR50);
	} else if (strcmp(buf, "legacy")) {
		err = -EINVAL;
		dev_err(mmc_dev(sdhost->mmc),
			"Invalid bus timing requested %d\n", err);
		goto err_detect;
	}

	/*Checks if user requested mode is supported by host*/
	if (timing_req && (!(sdhost->caps_timing_orig & timing_req))) {
		err = -EINVAL;
		dev_err(mmc_dev(sdhost->mmc),
			"Timing not supported by Host %d\n", err);
		goto err_detect;
	}

	/*
	 *Limit the capability of host upto user requested timing
	 */
	sdhost->mmc->caps |= sdhost->caps_timing_orig;
	sdhost->mmc->caps &= mask;

	dev_dbg(mmc_dev(sdhost->mmc),
		"Host Bus Timing limited to %s mode\n", buf);
	dev_dbg(mmc_dev(sdhost->mmc),
		"when sdcard is inserted next time, bus timing");
	dev_dbg(mmc_dev(sdhost->mmc),
		"gets selected based on card speed caps");
	return count;

err_detect:
	return err;
}


static const struct file_operations sdhci_host_bus_timing_fops = {
	.read		= get_bus_timing,
	.write		= set_bus_timing,
	.open		= simple_open,
	.owner		= THIS_MODULE,
	.llseek		= default_llseek,
};

DEFINE_SIMPLE_ATTRIBUTE(sdhci_tegra_card_insert_fops, get_card_insert, set_card_insert,
	"%llu\n");
static void sdhci_tegra_misc_debugfs(struct sdhci_host *host)
{
	struct dentry *root = host->debugfs_root;
	unsigned saved_line;

	/*backup original host timing capabilities as debugfs may override it later*/
	host->caps_timing_orig = host->mmc->caps &
				(MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_DDR50
				| MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25
				| MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104);

	if (!root) {
		root = debugfs_create_dir(dev_name(mmc_dev(host->mmc)), NULL);
		if (IS_ERR_OR_NULL(root)) {
			saved_line = __LINE__;
			goto err_root;
		}
		host->debugfs_root = root;
	}

	if (!debugfs_create_file("bus_timing", S_IRUSR | S_IWUSR, root, host,
				&sdhci_host_bus_timing_fops)) {
		saved_line = __LINE__;
		goto err_node;
	}

	if (!debugfs_create_file("card_insert", S_IRUSR | S_IWUSR, root, host,
			&sdhci_tegra_card_insert_fops)) {
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

static void tegra_sdhci_config_tap(struct sdhci_host *sdhci, u8 option)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	u32 tap_delay;

	switch (option) {
	case SAVE_TUNED_TAP:
		tap_delay = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
		tap_delay >>= SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT;
		tap_delay &= SDHCI_VNDR_CLK_CTRL_TAP_VALUE_MASK;
		tegra_host->tuned_tap_delay = tap_delay;
		break;
	case SET_DEFAULT_TAP:
		sdhci_tegra_set_tap_delay(sdhci, tegra_host->plat->tap_delay);
		break;
	case SET_TUNED_TAP:
		sdhci_tegra_set_tap_delay(sdhci, tegra_host->tuned_tap_delay);
		break;
	default:
		dev_err(mmc_dev(sdhci->mmc),
			"Invalid argument passed to tap config\n");
	}
}

static void sdhci_tegra_select_drive_strength(struct sdhci_host *host,
		unsigned int uhs)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret = 0;

	if (!IS_ERR_OR_NULL(tegra_host->pinctrl_sdmmc)) {
		if (!IS_ERR_OR_NULL(tegra_host->sdmmc_pad_ctrl[uhs])) {
			ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
					tegra_host->sdmmc_pad_ctrl[uhs]);
			if (ret < 0)
				dev_warn(mmc_dev(host->mmc),
					"setting pad strength for sdcard mode %d failed\n", uhs);

		} else {
			dev_dbg(mmc_dev(host->mmc),
				"No custom pad-ctrl strength settings present for sdcard %d mode\n", uhs);
		}
	}
}

/*
 * Set the max pio transfer limits to allow for dynamic switching between dma
 * and pio modes if the platform data indicates support for it. Option to set
 * different limits for different interfaces.
 */
static void tegra_sdhci_set_max_pio_transfer_limits(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	if (!tegra_host->plat->dynamic_dma_pio_switch || !sdhci->mmc->card)
		return;

	switch (sdhci->mmc->card->type) {
	case MMC_TYPE_MMC:
		sdhci->max_pio_size = 0;
		sdhci->max_pio_blocks = 0;
		break;
	case MMC_TYPE_SD:
		sdhci->max_pio_size = 0;
		sdhci->max_pio_blocks = 0;
		break;
	case MMC_TYPE_SDIO:
		sdhci->max_pio_size = 0;
		sdhci->max_pio_blocks = 0;
		break;
	default:
		dev_err(mmc_dev(sdhci->mmc),
			"Unknown device type. No max pio limits set\n");
	}
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
		const struct tegra_sdhci_platform_data *plat)
{
	struct device_node *np = dev->of_node;
	const char *drive_gname;
	int i = 0;
	int ret = 0;
	struct pinctrl_state *pctl_state;

	if (!np)
		return 0;

	if (plat->update_pinctrl_settings) {
		tegra_host->pinctrl_sdmmc = devm_pinctrl_get(dev);
		if (IS_ERR_OR_NULL(tegra_host->pinctrl_sdmmc)) {
			dev_err(dev, "Missing pinctrl info\n");
			return -EINVAL;
		}

		tegra_host->schmitt_enable[0] =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_schmitt_enable");
		if (IS_ERR_OR_NULL(tegra_host->schmitt_enable[0]))
			dev_dbg(dev, "Missing schmitt enable state\n");

		tegra_host->schmitt_enable[1] =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_clk_schmitt_enable");
		if (IS_ERR_OR_NULL(tegra_host->schmitt_enable[1]))
			dev_dbg(dev, "Missing clk schmitt enable state\n");

		tegra_host->schmitt_disable[0] =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_schmitt_disable");
		if (IS_ERR_OR_NULL(tegra_host->schmitt_disable[0]))
			dev_dbg(dev, "Missing schmitt disable state\n");

		tegra_host->schmitt_disable[1] =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_clk_schmitt_disable");
		if (IS_ERR_OR_NULL(tegra_host->schmitt_disable[1]))
			dev_dbg(dev, "Missing clk schmitt disable state\n");

		for (i = 0; i < 2; i++) {
			if (!IS_ERR_OR_NULL(tegra_host->schmitt_disable[i])) {
				ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
						tegra_host->schmitt_disable[i]);
				if (ret < 0)
					dev_warn(dev, "setting schmitt state failed\n");
			}
		}
		tegra_host->drv_code_strength =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_drv_code");
		if (IS_ERR_OR_NULL(tegra_host->drv_code_strength))
			dev_dbg(dev, "Missing sdmmc drive code state\n");

		tegra_host->default_drv_code_strength =
			pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
			"sdmmc_default_drv_code");
		if (IS_ERR_OR_NULL(tegra_host->default_drv_code_strength))
			dev_dbg(dev, "Missing sdmmc default drive code state\n");

		/* Apply the default_mode settings to all modes of SD/MMC
		   initially and then later update the pad strengths depending
		   upon the states specified if any */
		pctl_state = pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
						"default_mode");
		if (IS_ERR_OR_NULL(pctl_state)) {
			dev_dbg(dev, "Missing default mode pad control state\n");
		}
		else {
			for (i = 0; i < MMC_TIMINGS_MAX_MODES; i++)
				tegra_host->sdmmc_pad_ctrl[i] = pctl_state;
		}

		pctl_state = pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
						"uhs_sdr50_mode");
		if (IS_ERR_OR_NULL(pctl_state)) {
			dev_dbg(dev, "Missing sdr50 pad control state\n");
		}
		else {
			tegra_host->sdmmc_pad_ctrl[MMC_TIMING_UHS_SDR50] = pctl_state;
			tegra_host->sdmmc_pad_ctrl[MMC_TIMING_UHS_DDR50] = pctl_state;
		}

		pctl_state = pinctrl_lookup_state(tegra_host->pinctrl_sdmmc,
						"uhs_sdr104_mode");
		if (IS_ERR_OR_NULL(pctl_state)) {
			dev_dbg(dev, "Missing sdr104 pad control state\n");
		}
		else {
			tegra_host->sdmmc_pad_ctrl[MMC_TIMING_UHS_SDR104] = pctl_state;
		}

		/*Select the default state*/
		if (!IS_ERR_OR_NULL(tegra_host->sdmmc_pad_ctrl[MMC_TIMING_MMC_HS])) {
			ret = pinctrl_select_state(tegra_host->pinctrl_sdmmc,
					tegra_host->sdmmc_pad_ctrl[MMC_TIMING_MMC_HS]);
			if (ret < 0)
				dev_warn(dev, "setting default pad state failed\n");
		}
	}

	tegra_host->pinctrl = pinctrl_get_dev_from_of_property(np,
					"drive-pin-pinctrl");
	if (!tegra_host->pinctrl)
		 return -EINVAL;

	drive_gname = of_get_property(np, "drive-pin-name", NULL);
	tegra_host->drive_group_sel = pinctrl_get_selector_from_group_name(
					tegra_host->pinctrl, drive_gname);
	return 0;
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
	.post_init	= tegra_sdhci_post_init,
	.platform_resume	= tegra_sdhci_post_resume,
	.switch_signal_voltage_exit = tegra_sdhci_do_calibration,
	.sd_error_stats		= sdhci_tegra_sd_error_stats,
	.get_drive_strength	= tegra_sdhci_get_drive_strength,
	.dump_host_cust_regs	= tegra_sdhci_dumpregs,
	.get_max_tuning_loop_counter = sdhci_tegra_get_max_tuning_loop_counter,
	.config_tap_delay	= tegra_sdhci_config_tap,
	.validate_sd2_0		= tegra_sdhci_validate_sd2_0,
	.get_max_pio_transfer_limits = tegra_sdhci_set_max_pio_transfer_limits,
};

static const struct sdhci_pltfm_data sdhci_tegra124_pdata = {
	.quirks = TEGRA_SDHCI_QUIRKS,
	.quirks2 = TEGRA_SDHCI_QUIRKS2,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra124 = {
	.pdata = &sdhci_tegra124_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS |
		NVQUIRK_INFINITE_ERASE_TIMEOUT |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_SET_CALIBRATION_OFFSETS |
		NVQUIRK_SHADOW_XFER_MODE_REG |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD,
};

static const struct sdhci_pltfm_data sdhci_tegra210_pdata = {
	.quirks = TEGRA_SDHCI_QUIRKS,
	.quirks2 = TEGRA_SDHCI_QUIRKS2 |
		SDHCI_QUIRK2_NON_STD_TUN_CARD_CLOCK |
		SDHCI_QUIRK2_NON_STD_TUNING_LOOP_CNTR |
		SDHCI_QUIRK2_PERIODIC_CALIBRATION,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra210 = {
	.pdata = &sdhci_tegra210_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS |
		NVQUIRK_INFINITE_ERASE_TIMEOUT |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_SET_CALIBRATION_OFFSETS |
		NVQUIRK_SHADOW_XFER_MODE_REG |
		NVQUIRK_DISABLE_TIMER_BASED_TUNING |
		NVQUIRK_SET_SDMEMCOMP_VREF_SEL |
		NVQUIRK_DISABLE_EXTERNAL_LOOPBACK |
		NVQUIRK_UPDATE_PAD_CNTRL_REG |
		NVQUIRK_UPDATE_PIN_CNTRL_REG |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_UPDATE_HW_TUNING_CONFG |
		NVQUIRK_BROKEN_SD2_0_SUPPORT |
		NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH,
	.nvquirks2 = NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION,
};

static const struct sdhci_pltfm_data sdhci_tegra186_pdata = {
	.quirks = TEGRA_SDHCI_QUIRKS,
	.quirks2 = TEGRA_SDHCI_QUIRKS2 |
		SDHCI_QUIRK2_NON_STD_TUN_CARD_CLOCK |
		SDHCI_QUIRK2_NON_STD_TUNING_LOOP_CNTR |
		SDHCI_QUIRK2_PERIODIC_CALIBRATION,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra186 = {
	.pdata = &sdhci_tegra210_pdata,
	.nvquirks = TEGRA_SDHCI_NVQUIRKS |
		NVQUIRK_INFINITE_ERASE_TIMEOUT |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_SET_CALIBRATION_OFFSETS |
		NVQUIRK_SHADOW_XFER_MODE_REG |
		NVQUIRK_DISABLE_TIMER_BASED_TUNING |
		NVQUIRK_SET_SDMEMCOMP_VREF_SEL |
		NVQUIRK_DISABLE_EXTERNAL_LOOPBACK |
		NVQUIRK_UPDATE_PAD_CNTRL_REG |
		NVQUIRK_UPDATE_PIN_CNTRL_REG |
		NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		NVQUIRK_UPDATE_HW_TUNING_CONFG |
		NVQUIRK_BROKEN_SD2_0_SUPPORT |
		NVQUIRK_DYNAMIC_TRIM_SUPPLY_SWITCH,
	.nvquirks2 = NVQUIRK2_ADD_DELAY_AUTO_CALIBRATION,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra186-sdhci", .data = &soc_data_tegra186 },
	{ .compatible = "nvidia,tegra210-sdhci", .data = &soc_data_tegra210 },
	{ .compatible = "nvidia,tegra124-sdhci", .data = &soc_data_tegra124 },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tegra_dt_match);

static int sdhci_tegra_parse_dt(struct device *dev) {
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tegra_sdhci_platform_data *plat;

	if (!np)
		return -EINVAL;

	tegra_host->power_gpio = of_get_named_gpio(np, "power-gpios", 0);
	plat = devm_kzalloc(dev, sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		dev_err(dev, "Can't allocate platform data\n");
		return -EINVAL;
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
	of_property_read_u32(np, "default-drv-type", &plat->default_drv_type);
	plat->update_pinctrl_settings = of_property_read_bool(np,
		"nvidia,update-pinctrl-settings");
	plat->dll_calib_needed = of_property_read_bool(np, "nvidia,dll-calib-needed");
	plat->pwr_off_during_lp0 = of_property_read_bool(np,
		"pwr-off-during-lp0");
	of_property_read_u32(np, "auto-cal-step", &plat->auto_cal_step);
	plat->disable_auto_cal = of_property_read_bool(np,
		"nvidia,disable-auto-cal");
	plat->en_io_trim_volt = of_property_read_bool(np,
			"nvidia,en-io-trim-volt");
	plat->power_off_rail = of_property_read_bool(np,
		"power-off-rail");
	plat->is_emmc = of_property_read_bool(np, "nvidia,is-emmc");
	plat->is_sd_device = of_property_read_bool(np, "nvidia,sd-device");
	plat->enable_hs533_mode = of_property_read_bool(np, "nvidia,enable-hs533-mode");
	if (of_find_property(np, "fixed-clock-freq", NULL)) {
		plat->is_fix_clock_freq = true;
		of_property_read_u32_array(np,
					"fixed-clock-freq",
					(u32 *)&plat->fixed_clk_freq_table,
					MMC_TIMINGS_MAX_MODES);
	}
	plat->enable_autocal_slew_override = of_property_read_bool(np,
					"nvidia,auto-cal-slew-override");
	plat->enable_cq = of_property_read_bool(np, "nvidia,enable-cq");
	plat->en_periodic_calib = of_property_read_bool(np,
			"nvidia,en-periodic-calib");

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
	int rc;
	u8 i;
	u32 opt_subrevision;

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

	tegra_host->sd_stat_head = devm_kzalloc(&pdev->dev,
		sizeof(struct sdhci_tegra_sd_stats), GFP_KERNEL);
	if (!tegra_host->sd_stat_head) {
		dev_err(mmc_dev(host->mmc), "failed to allocate sd_stat_head\n");
		rc = -ENOMEM;
		goto err_power_req;
	}

	/* check if DT provide list possible pll parents */
	if (sdhci_tegra_get_pll_from_dt(pdev,
		&parent_clk_list[0], ARRAY_SIZE(parent_clk_list))) {
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
					"clk[%d] error in getting %s: %d\n",
					i, parent_clk_list[i], rc);
			goto err_power_req;
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


	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;
	if (clk_get_parent(pltfm_host->clk) == tegra_host->pll_source[0].pll)
		tegra_host->is_parent_pll_source_1 = true;
	rc = clk_prepare_enable(clk);
	if (rc != 0)
		goto err_clk_put;

	/* Reset the sdhci controller to clear all previous status.*/
	tegra_periph_reset_assert(pltfm_host->clk);
	udelay(2);
	tegra_periph_reset_deassert(pltfm_host->clk);

	if (plat->is_sd_device &&
		(tegra_get_chipid() == TEGRA_CHIPID_TEGRA21) &&
		(tegra_chip_get_revision() == TEGRA_REVISION_A01)) {
		opt_subrevision = tegra_get_fuse_opt_subrevision();
		if ((opt_subrevision == 0) || (opt_subrevision == 1))
			tegra_host->limit_vddio_max_volt = true;
	}

	pltfm_host->priv = tegra_host;
	tegra_host->clk_enabled = true;
	mutex_init(&tegra_host->set_clock_mutex);

	tegra_host->max_clk_limit = plat->max_clk_limit;

	sdhci_tegra_init_pinctrl_info(&pdev->dev, tegra_host, plat);

	host->mmc->pm_caps |= plat->pm_caps;
	host->mmc->pm_flags |= plat->pm_flags;

	tegra_pd_add_device(&pdev->dev);

	/* disable access to boot partitions */
	host->mmc->caps2 |= MMC_CAP2_BOOTPART_NOACC;
	host->mmc->caps2 |= MMC_CAP2_PACKED_CMD;
	host->mmc->caps2 |= MMC_CAP2_SINGLE_POWERON;
	if (soc_data->nvquirks & NVQUIRK_EN_STROBE_SUPPORT)
		host->mmc->caps2 |= MMC_CAP2_EN_STROBE;
	if (plat->pwr_off_during_lp0)
		host->mmc->caps2 |= MMC_CAP2_NO_SLEEP_CMD;
	if ((plat->enable_hs533_mode) && (host->mmc->caps2 & MMC_CAP2_HS400))
		host->mmc->caps2 |= MMC_CAP2_HS533;
	if (plat->enable_cq)
		host->mmc->caps2 |= MMC_CAP2_CQ;
	if (plat->en_periodic_calib)
		host->quirks2 |= SDHCI_QUIRK2_PERIODIC_CALIBRATION;

	rc = sdhci_add_host(host);
	sdhci_tegra_error_stats_debugfs(host);
	if (rc)
		goto err_add_host;

	sdhci_tegra_misc_debugfs(host);
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
	const struct tegra_sdhci_platform_data *plat;
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	plat = tegra_host->plat;
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

static void sdhci_tegra_shutdown(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	dev_dbg(&pdev->dev, " %s shutting down\n",
		mmc_hostname(host->mmc));
#ifdef CONFIG_MMC_RTPM
	pm_runtime_forbid(&pdev->dev);
#endif
}

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
	.shutdown	= sdhci_tegra_shutdown,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
