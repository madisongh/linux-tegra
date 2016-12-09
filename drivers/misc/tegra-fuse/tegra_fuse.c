/*
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2012-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Sumit Sharma <sumsharma@nvidia.com>
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/tegra-soc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/tegra-fuse.h>
#include <linux/platform/tegra/common.h>
#include <linux/of_address.h>

#include <asm/fixmap.h>
#ifdef CONFIG_ARM64
#include <asm/mmu.h>
#endif

#include <soc/tegra/gpufuse.h>

/* Tegra macros defining HIDREV MINORREV */
#define MINOR_QT                0
#define MINOR_FPGA              1
#define MINOR_ASIM_QT           2
#define MINOR_ASIM_LINSIM       3
#define MINOR_DSIM_ASIM_LINSIM  4
#define MINOR_UNIT_FPGA         5
#define MINOR_VDK               6

/* Tegra macros defining HIDREV PRE_SI_PLATFORM */
#define PRE_SI_QT		1
#define PRE_SI_FPGA		2
#define PRE_SI_UNIT_FPGA	3
#define PRE_SI_ASIM_QT		4
#define PRE_SI_ASIM_LINSIM	5
#define PRE_SI_DSIM_ASIM_LINSIM	6
#define PRE_SI_VDK		8

#define FUSE_BEGIN		0x100

struct tegra_id {
	enum tegra_chipid chipid;
	unsigned int major, minor, netlist, patch;
	enum tegra_revision revision;
	char *priv;
};

struct tegra_fuse {
	void __iomem *base;
	struct clk *clk;
};

static struct tegra_fuse fuse = {0};
static struct tegra_id tegra_id;
static int tegra_gpu_num_pixel_pipes;
static int tegra_gpu_num_alus_per_pixel_pipe;
static u32 tegra_chip_sku_id;
static u32 tegra_chip_id;
static u32 tegra_chip_bct_strapping;
enum tegra_revision tegra_revision;
static enum tegra_platform tegra_platform;
static bool cpu_is_asim;
static bool cpu_is_dsim;
static const char *tegra_platform_name[TEGRA_PLATFORM_MAX] = {
	[TEGRA_PLATFORM_SILICON] = "silicon",
	[TEGRA_PLATFORM_QT]      = "quickturn",
	[TEGRA_PLATFORM_LINSIM]  = "linsim",
	[TEGRA_PLATFORM_FPGA]    = "fpga",
	[TEGRA_PLATFORM_UNIT_FPGA] = "unit fpga",
	[TEGRA_PLATFORM_VDK] = "vdk",
};

int tegra_gpu_register_sets(void)
{
#ifdef CONFIG_ARCH_TEGRA_HAS_DUAL_3D
	u32 reg = tegra_read_clk_ctrl_reg(FUSE_GPU_INFO);
	if (reg & FUSE_GPU_INFO_MASK)
		return 1;
	else
		return 2;
#else
	return 1;
#endif
}

void tegra_gpu_get_info(struct gpu_info *info)
{
	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA11) {
		info->num_pixel_pipes = 4;
		info->num_alus_per_pixel_pipe = 3;
	} else {
		info->num_pixel_pipes = 1;
		info->num_alus_per_pixel_pipe = 1;
	}
}

static int get_gpu_num_pixel_pipes(char *val, const struct kernel_param *kp)
{
	struct gpu_info gpu_info;

	tegra_gpu_get_info(&gpu_info);
	tegra_gpu_num_pixel_pipes = gpu_info.num_pixel_pipes;
	return param_get_uint(val, kp);
}

static int get_gpu_num_alus_per_pixel_pipe(char *val,
						const struct kernel_param *kp)
{
	struct gpu_info gpu_info;

	tegra_gpu_get_info(&gpu_info);
	tegra_gpu_num_alus_per_pixel_pipe = gpu_info.num_alus_per_pixel_pipe;

	return param_get_uint(val, kp);
}

static struct kernel_param_ops tegra_gpu_num_pixel_pipes_ops = {
	.get = get_gpu_num_pixel_pipes,
};

static struct kernel_param_ops tegra_gpu_num_alus_per_pixel_pipe_ops = {
	.get = get_gpu_num_alus_per_pixel_pipe,
};

module_param_cb(tegra_gpu_num_pixel_pipes, &tegra_gpu_num_pixel_pipes_ops,
		&tegra_gpu_num_pixel_pipes, 0444);
module_param_cb(tegra_gpu_num_alus_per_pixel_pipe,
		&tegra_gpu_num_alus_per_pixel_pipe_ops,
		&tegra_gpu_num_alus_per_pixel_pipe, 0444);

struct chip_revision {
	enum tegra_chipid	chipid;
	unsigned int		major;
	unsigned int		minor;
	char			prime;
	enum tegra_revision	revision;
};

#define CHIP_REVISION(id, m, n, p, rev) {	\
	.chipid = TEGRA_CHIPID_##id,		\
	.major = m,				\
	.minor = n,				\
	.prime = p,				\
	.revision = TEGRA_REVISION_##rev }

static struct chip_revision tegra_chip_revisions[] = {
	CHIP_REVISION(TEGRA2,  1, 2, 0,   A02),
	CHIP_REVISION(TEGRA2,  1, 3, 0,   A03),
	CHIP_REVISION(TEGRA2,  1, 3, 'p', A03p),
	CHIP_REVISION(TEGRA2,  1, 4, 0,   A04),
	CHIP_REVISION(TEGRA2,  1, 4, 'p', A04p),
	CHIP_REVISION(TEGRA3,  1, 1, 0,   A01),
	CHIP_REVISION(TEGRA3,  1, 2, 0,   A02),
	CHIP_REVISION(TEGRA3,  1, 3, 0,   A03),
	CHIP_REVISION(TEGRA11, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA11, 1, 2, 0,   A02),
	CHIP_REVISION(TEGRA12, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA13, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA13, 1, 2, 0,   A02),
	CHIP_REVISION(TEGRA13, 1, 3, 0,   A03),
	CHIP_REVISION(TEGRA14, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA14, 1, 2, 0,   A02),
	CHIP_REVISION(TEGRA21, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA21, 1, 1, 'q', A01q),
	CHIP_REVISION(TEGRA21, 1, 2, 0,   A02),
	CHIP_REVISION(TEGRA18, 1, 1, 0,   A01),
	CHIP_REVISION(TEGRA18, 1, 2, 0,   A02),
	CHIP_REVISION(TEGRA18, 1, 2, 'p',   A02p),
};

static enum tegra_revision tegra_decode_revision(const struct tegra_id *id)
{
	enum tegra_revision revision = TEGRA_REVISION_UNKNOWN;
	int i;
	char prime;

	/* For pre-silicon the major is 0, for silicon it is >= 1 */
	if (id->major == 0) {
		if (id->minor == 1)
			revision = TEGRA_REVISION_A01;
		else if (id->minor == 2)
			revision = TEGRA_REVISION_QT;
		else if (id->minor == 3)
			revision = TEGRA_REVISION_SIM;
		return revision;
	}

	if (id->priv == NULL)
		prime = 0;
	else
		prime = *(id->priv);

	for (i = 0; i < ARRAY_SIZE(tegra_chip_revisions); i++) {
		if ((id->chipid != tegra_chip_revisions[i].chipid) ||
		    (id->minor != tegra_chip_revisions[i].minor) ||
		    (id->major != tegra_chip_revisions[i].major) ||
		    (prime != tegra_chip_revisions[i].prime))
			continue;

		revision = tegra_chip_revisions[i].revision;
		break;
	}

	return revision;
}

void tegra_set_tegraid(u32 chipid, u32 major, u32 minor,
	u32 pre_si_plat, u32 nlist, u32 patch, const char *priv)
{
	tegra_id.chipid  = (enum tegra_chipid) chipid;
	tegra_id.major   = major;
	tegra_id.minor   = minor;
	tegra_id.netlist = nlist;
	tegra_id.patch   = patch;
	tegra_id.priv    = (char *)priv;
	tegra_id.revision = tegra_decode_revision(&tegra_id);
	/* for backward compatibility */
	if (tegra_id.major == 0) {
		switch (tegra_id.minor) {
		case MINOR_QT:
			cpu_is_asim = false;
			tegra_platform = TEGRA_PLATFORM_QT;
			break;
		case MINOR_FPGA:
			cpu_is_asim = false;
			tegra_platform = TEGRA_PLATFORM_FPGA;
			break;
		case MINOR_ASIM_QT:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_QT;
			break;
		case MINOR_ASIM_LINSIM:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_LINSIM;
			break;
		case MINOR_DSIM_ASIM_LINSIM:
			cpu_is_asim = true;
			cpu_is_dsim = true;
			tegra_platform = TEGRA_PLATFORM_LINSIM;
			break;
		case MINOR_UNIT_FPGA:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_UNIT_FPGA;
			break;
		case MINOR_VDK:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_VDK;
			break;
		default:
			pr_err("%s: Invalid pre_si_platform MINOREV=%d\n",
				__func__, tegra_id.minor);
		}
	} else if (pre_si_plat > 0) {
	/* New way to detect pre-silicon platforms T194 onwards */
		switch (pre_si_plat) {
		case PRE_SI_QT:
			cpu_is_asim = false;
			tegra_platform = TEGRA_PLATFORM_QT;
			break;
		case PRE_SI_FPGA:
			cpu_is_asim = false;
			tegra_platform = TEGRA_PLATFORM_FPGA;
			break;
		case PRE_SI_UNIT_FPGA:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_UNIT_FPGA;
			break;
		case PRE_SI_ASIM_QT:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_QT;
			break;
		case PRE_SI_ASIM_LINSIM:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_LINSIM;
			break;
		case PRE_SI_DSIM_ASIM_LINSIM:
			cpu_is_asim = true;
			cpu_is_dsim = true;
			tegra_platform = TEGRA_PLATFORM_LINSIM;
			break;
		case PRE_SI_VDK:
			cpu_is_asim = true;
			tegra_platform = TEGRA_PLATFORM_VDK;
			break;
		default:
			pr_err("%s: Invalid pre_si_platform=%d\n",
				__func__, pre_si_plat);
		}
	} else {
		cpu_is_asim = false;
		tegra_platform = TEGRA_PLATFORM_SILICON;
	}
}

enum tegra_chipid tegra_get_chipid(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();

	return tegra_id.chipid;
}
EXPORT_SYMBOL(tegra_get_chipid);

enum tegra_revision tegra_chip_get_revision(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();

	return tegra_id.revision;
}
EXPORT_SYMBOL(tegra_chip_get_revision);

unsigned int tegra_get_minor_rev(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();

	return tegra_id.minor;
}

void tegra_get_netlist_revision(u32 *netlist, u32 *patchid)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();

	if (!tegra_platform_is_fpga()
	    && !tegra_platform_is_qt()
	    && !tegra_platform_is_linsim())
		BUG();
	*netlist = tegra_id.netlist;
	*patchid = tegra_id.patch & 0xF;
}

enum tegra_platform tegra_get_platform(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();
	return tegra_platform;
}
EXPORT_SYMBOL(tegra_get_platform);

bool tegra_cpu_is_asim(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();
	return cpu_is_asim;
}
EXPORT_SYMBOL(tegra_cpu_is_asim);

bool tegra_cpu_is_dsim(void)
{
	if (tegra_id.chipid == TEGRA_CHIPID_UNKNOWN)
		tegra_get_tegraid_from_hw();
	return cpu_is_dsim;
}

static const char *tegra_platform_ptr;
static int get_platform(char *val, const struct kernel_param *kp)
{
	tegra_platform_ptr = tegra_platform_name[tegra_platform];
	return param_get_charp(val, kp);
}
static struct kernel_param_ops tegra_platform_ops = {
	.get = get_platform,
};
module_param_cb(tegra_platform, &tegra_platform_ops, &tegra_platform_ptr, 0444);

static const char *tegra_cpu_ptr;
static int get_cpu_type(char *val, const struct kernel_param *kp)
{
	tegra_cpu_ptr = cpu_is_asim ? "asim" :
				tegra_platform_name[tegra_platform];
	return param_get_charp(val, kp);
}
static struct kernel_param_ops tegra_cpu_ops = {
	.get = get_cpu_type,
};
module_param_cb(tegra_cpu, &tegra_cpu_ops, &tegra_cpu_ptr, 0444);

static int get_chip_id(char *val, const struct kernel_param *kp)
{
	return param_get_uint(val, kp);
}

static int get_revision(char *val, const struct kernel_param *kp)
{
	return param_get_uint(val, kp);
}

static struct kernel_param_ops tegra_chip_id_ops = {
	.get = get_chip_id,
};

static struct kernel_param_ops tegra_revision_ops = {
	.get = get_revision,
};

module_param_cb(tegra_chip_id, &tegra_chip_id_ops, &tegra_id.chipid, 0444);
module_param_cb(tegra_chip_rev, &tegra_revision_ops, &tegra_id.revision, 0444);

static void tegra_set_sku_id(void)
{
	u32 reg;

	tegra_fuse_readl(FUSE_SKU_INFO, &reg);
	if (reg & FUSE_SKU_MSB_MASK)
		tegra_chip_sku_id = (reg >> FUSE_SKU_MSB_SHIFT);
	else
		tegra_chip_sku_id = reg & 0xFF;

}

u32 tegra_get_fuse_opt_subrevision(void)
{
	u8 ret = 0;
	u32 reg;

	tegra_fuse_readl(FUSE_OPT_SUBREVISION, &reg);

	ret = reg & FUSE_OPT_SUBREVISION_MASK;

	return ret;
}
EXPORT_SYMBOL(tegra_get_fuse_opt_subrevision);

u32 tegra_get_sku_id(void)
{
	return tegra_chip_sku_id;
}

u8 tegra_get_chip_id(void)
{
	return tegra_chip_id;
}
EXPORT_SYMBOL(tegra_get_chip_id);

u32 tegra_get_bct_strapping(void)
{
	return tegra_chip_bct_strapping;
}

static struct of_device_id tegra_fuse_of_match[] = {
	{
		.compatible = "nvidia,tegra186-efuse",
	}, {}
};
MODULE_DEVICE_TABLE(of, tegra_fuse_of_match);

static void tegra_fuse_early_init(void)
{
	struct device_node *np;

	np = of_find_matching_node(NULL, tegra_fuse_of_match);
	if (!np) {
		pr_err("fuse DT node missing\n");
		return;
	}

	fuse.base = of_iomap(np, 0);
	if (!fuse.base)
		pr_warn("fuse DT node missing\n");
}

int tegra_fuse_readl(unsigned long offset, u32 *val)
{
	if (!fuse.base)
		tegra_fuse_early_init();

	if (fuse.base) {
		*val = readl(fuse.base + FUSE_BEGIN + offset);
	} else {
		pr_err("fuse base not initialized\n");
		WARN_ON(1);
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_fuse_readl);

int tegra_fuse_control_read(unsigned long offset, u32 *value)
{
	if (!fuse.base)
		tegra_fuse_early_init();

	if (fuse.base) {
		*value = readl(fuse.base + offset);
	} else {
		pr_err("fuse base not initialized\n");
		WARN_ON(1);
		return -ENODEV;
	}

	return 0;
}

void tegra_fuse_writel(u32 val, unsigned long offset)
{
	if (fuse.base) {
		writel(val, fuse.base + FUSE_BEGIN + offset);
	} else {
		pr_err("fuse base not initialized\n");
		WARN_ON(1);
	}
}
EXPORT_SYMBOL(tegra_fuse_writel);

void tegra_fuse_control_write(u32 value, unsigned long offset)
{
	if (fuse.base) {
		writel(value, fuse.base + offset);
	} else {
		pr_err("fuse base not initialized\n");
		WARN_ON(1);
	}
}

static int tegra_fuse_probe(struct platform_device *pdev)
{
	struct resource *regs;
	int err;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fuse.base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(fuse.base))
		return PTR_ERR(fuse.base);

	fuse.clk = devm_clk_get(&pdev->dev, "fuse");
	if (IS_ERR(fuse.clk)) {
		pr_err("fuse clk get err:%ld\n", PTR_ERR(fuse.clk));
		return PTR_ERR(fuse.clk);
	}

	/* Keeping fuse clock always ON for now*/
	clk_prepare_enable(fuse.clk);

	/* Read SKU register and set the sku id structure */
	tegra_set_sku_id();

	err = of_platform_default_populate(pdev->dev.of_node, NULL, &pdev->dev);
	if (err < 0) {
		dev_err(&pdev->dev, "child node not available\n");
		return err;
	}

	return 0;
}

static struct platform_driver tegra_fuse_driver = {
	.probe   = tegra_fuse_probe,
	.driver  = {
		.name  = "tegra-fuse",
		.of_match_table = tegra_fuse_of_match,
	},
};

static int __init tegra_fuse_init_driver(void)
{
	return platform_driver_register(&tegra_fuse_driver);
}
subsys_initcall(tegra_fuse_init_driver);
