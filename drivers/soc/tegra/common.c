/*
 * Copyright (C) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/ioport.h>

#include <soc/tegra/common.h>

static const struct of_device_id tegra_machine_match[] = {
	{ .compatible = "nvidia,tegra20", },
	{ .compatible = "nvidia,tegra30", },
	{ .compatible = "nvidia,tegra114", },
	{ .compatible = "nvidia,tegra124", },
	{ .compatible = "nvidia,tegra132", },
	{ .compatible = "nvidia,tegra210", },
	{ }
};

phys_addr_t tegra_bootloader_fb_start;
phys_addr_t tegra_bootloader_fb_size;
phys_addr_t tegra_bootloader_fb2_start;
phys_addr_t tegra_bootloader_fb2_size;

bool soc_is_tegra(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/");
	if (!root)
		return false;

	return of_match_node(tegra_machine_match, root) != NULL;
}

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem: %08llx@%08llx\n",
			(u64)tegra_bootloader_fb_size, (u64)tegra_bootloader_fb_start);

	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_bootloader_fb2_arg(char *options)
{
	char *p = options;

	tegra_bootloader_fb2_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb2_start = memparse(p+1, &p);

	pr_info("Found tegra_fbmem2: %08llx@%08llx\n",
			(u64)tegra_bootloader_fb2_size,
			(u64)tegra_bootloader_fb2_start);

	return 0;
}
early_param("tegra_fbmem2", tegra_bootloader_fb2_arg);

void tegra_get_fb_resource(struct resource *fb_res)
{
	fb_res->start = (resource_size_t) tegra_bootloader_fb_start;
	fb_res->end = fb_res->start +
		(resource_size_t) tegra_bootloader_fb_size - 1;
}

void tegra_get_fb2_resource(struct resource *fb2_res)
{
	fb2_res->start = (resource_size_t) tegra_bootloader_fb2_start;
	fb2_res->end = fb2_res->start +
		(resource_size_t) tegra_bootloader_fb2_size - 1;
}

/* returns true if bl initialized the display */
bool tegra_is_bl_display_initialized(int instance)
{
	/* display initialized implies non-zero
	 * fb size is passed from bl to kernel
	 */
	switch (instance) {
		case 0:
			return tegra_bootloader_fb_start && tegra_bootloader_fb_size;
		case 1:
			return tegra_bootloader_fb2_start && tegra_bootloader_fb2_size;
		default:
			return false;
	}
}
