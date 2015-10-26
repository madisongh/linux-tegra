/*
 * Copyright (C) 2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/host1x.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/platform/tegra/mc.h>
#include <linux/reset.h>

#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
#include <linux/clk/tegra.h>
#include <linux/tegra-powergate.h>
#include <linux/platform/tegra/mc.h>
#else
#include <soc/tegra/pmc.h>
#endif

#include "drm.h"
#include "falcon.h"

#define NVDEC_MAX_CLK_RATE	716800000

#define MC_SECURITY_CARVEOUT1_BOM_LO		0xc0c
#define MC_SECURITY_CARVEOUT1_BOM_HI		0xc10
#define MC_SECURITY_CARVEOUT1_SIZE_128KB	0xc14
#define WPR_SIZE (128 * 1024)

struct nvdec_bl_shared_data {
	u32 ls_fw_start_addr;
	u32 ls_fw_size;
	u32 wpr_addr;
	u32 wpr_size;
};

struct nvdec_config {
	/* Firmware name */
	const char *ucode_name;
	const char *ucode_name_bl;
	const char *ucode_name_ls;
};

struct nvdec {
	struct falcon falcon_bl;
	struct falcon falcon_ls;
	bool booted;

	void __iomem *regs;
	struct tegra_drm_client client;
	struct host1x_channel *channel;
	struct iommu_domain *domain;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst;

	/* Platform configuration */
	const struct nvdec_config *config;

	/* for firewall - this determines if method 1 should be regarded
	 * as an address register */
	bool method_data_is_addr_reg;
};

static inline struct nvdec *to_nvdec(struct tegra_drm_client *client)
{
	return container_of(client, struct nvdec, client);
}

static int nvdec_runtime_resume(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
	err = tegra_unpowergate_partition(TEGRA_POWERGATE_NVDEC);
	if (err)
		return err;

	err = clk_prepare_enable(nvdec->clk);
	if (err) {
		tegra_powergate_partition(TEGRA_POWERGATE_NVDEC);
		return err;
	}

	return 0;
#else
	err = tegra_powergate_sequence_power_up(TEGRA_POWERGATE_NVDEC,
						nvdec->clk, nvdec->rst);
	if (err < 0)
		dev_err(dev, "failed to power up device\n");

	return err;
#endif
}

static int nvdec_runtime_suspend(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);

	clk_disable_unprepare(nvdec->clk);

#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
	tegra_powergate_partition(TEGRA_POWERGATE_NVDEC);
#else
	reset_control_assert(nvdec->rst);
	tegra_powergate_power_off(TEGRA_POWERGATE_NVDEC);
#endif

	nvdec->booted = false;

	return 0;
}

static int nvdec_wpr_setup(struct nvdec *nvdec)
{
	struct nvdec_bl_shared_data shared_data;
	struct mc_carveout_info inf;
	size_t fb_data_offset;
	int err;

	err = mc_get_carveout_info(&inf, NULL, MC_SECURITY_CARVEOUT1);
	if (err < 0)
		return err;

	/* Put the 40-bit addr formed by wpr_addr_hi and
	   wpr_addr_lo divided by 256 into 32-bit wpr_addr */
	shared_data.wpr_addr = inf.base >> 8;
	shared_data.wpr_size = inf.size; /* Already in bytes. */

	shared_data.ls_fw_start_addr = nvdec->falcon_ls.firmware.paddr >> 8;
	shared_data.ls_fw_size = nvdec->falcon_ls.firmware.size;

	fb_data_offset = nvdec->falcon_bl.firmware.bin_data.offset +
			 nvdec->falcon_bl.firmware.data.offset;

	memcpy(nvdec->falcon_bl.firmware.vaddr + fb_data_offset,
	       &shared_data, sizeof(shared_data));

	return 0;
}

static int nvdec_read_firmware(struct nvdec *nvdec)
{
	int err;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		err = falcon_read_firmware(&nvdec->falcon_bl,
					   nvdec->config->ucode_name_bl);
		if (err < 0)
			return err;

		err = falcon_read_firmware(&nvdec->falcon_ls,
					   nvdec->config->ucode_name_ls);
		if (err < 0)
			return err;

		err = nvdec_wpr_setup(nvdec);
		if (err < 0)
			return err;
	} else {
		err = falcon_read_firmware(&nvdec->falcon_bl,
					   nvdec->config->ucode_name);
		if (err < 0)
			return err;
	}

	return 0;
}

static int nvdec_boot(struct nvdec *nvdec)
{
	int err = 0;

	if (nvdec->booted)
		return 0;

	err = nvdec_read_firmware(nvdec);
	if (err < 0)
		return err;

	/* ensure that the engine is in sane state */
#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
	tegra_mc_flush(true);
	tegra_periph_reset_assert(nvdec->clk);
	usleep_range(10, 100);
	tegra_periph_reset_deassert(nvdec->clk);
	tegra_mc_flush_done(true);
#else
	reset_control_assert(nvdec->rst);
	usleep_range(10, 100);
	reset_control_deassert(nvdec->rst);
#endif

	err = falcon_boot(&nvdec->falcon_bl);
	if (err < 0)
		return err;

	err = falcon_wait_idle(&nvdec->falcon_bl);
	if (err < 0) {
		dev_err(nvdec->dev,
			"failed to set application ID and FCE base\n");
		return err;
	}

	nvdec->booted = true;

	return 0;
}

static void *nvdec_falcon_alloc(struct falcon *falcon, size_t size,
			       dma_addr_t *iova)
{
#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
	return dma_alloc_writecombine(falcon->dev, size, iova,
					   GFP_KERNEL | __GFP_NOWARN);
#else
	struct tegra_drm *tegra = falcon->data;

	return tegra_drm_alloc(tegra, size, iova);
#endif
}

static void nvdec_falcon_free(struct falcon *falcon, size_t size,
			    dma_addr_t iova, void *va)
{
#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
	dma_free_writecombine(falcon->dev, size, va, iova);
#else
	struct tegra_drm *tegra = falcon->data;

	return tegra_drm_free(tegra, size, va, iova);
#endif
}

static const struct falcon_ops nvdec_falcon_ops = {
	.alloc = nvdec_falcon_alloc,
	.free = nvdec_falcon_free
};

static int nvdec_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvdec *nvdec = to_nvdec(drm);
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, nvdec->dev);
		if (err < 0) {
			dev_err(nvdec->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		nvdec->domain = tegra->domain;
	}

	nvdec->falcon_bl.dev = nvdec->dev;
	nvdec->falcon_bl.regs = nvdec->regs;
	nvdec->falcon_bl.data = tegra;
	nvdec->falcon_bl.ops = &nvdec_falcon_ops;

	err = falcon_init(&nvdec->falcon_bl);
	if (err < 0)
		goto detach_device;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		nvdec->falcon_ls.dev = nvdec->dev;
		nvdec->falcon_ls.regs = nvdec->regs;
		nvdec->falcon_ls.data = tegra;
		nvdec->falcon_ls.ops = &nvdec_falcon_ops;

		err = falcon_init(&nvdec->falcon_ls);
		if (err < 0)
			goto exit_falcon_bl;

	}

	nvdec->channel = host1x_channel_request(client->dev);
	if (!nvdec->channel) {
		err = -ENOMEM;
		goto exit_falcon_ls;
	}

	client->syncpts[0] = host1x_syncpt_request(client->dev, 0);
	if (!client->syncpts[0]) {
		err = -ENOMEM;
		goto free_channel;
	}

	err = tegra_drm_register_client(tegra, drm);
	if (err < 0)
		goto free_syncpt;

	return 0;

free_syncpt:
	host1x_syncpt_free(client->syncpts[0]);
free_channel:
	host1x_channel_free(nvdec->channel);
exit_falcon_ls:
	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
		falcon_exit(&nvdec->falcon_ls);
exit_falcon_bl:
	falcon_exit(&nvdec->falcon_bl);
detach_device:
	if (tegra->domain)
		iommu_detach_device(tegra->domain, nvdec->dev);

	return err;
}

static int nvdec_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvdec *nvdec = to_nvdec(drm);
	int err;

	err = tegra_drm_unregister_client(tegra, drm);
	if (err < 0)
		return err;

	host1x_syncpt_free(client->syncpts[0]);
	host1x_channel_free(nvdec->channel);

	if (nvdec->booted) {
#ifdef CONFIG_DRM_TEGRA_DOWNSTREAM
		tegra_mc_flush(true);
		tegra_periph_reset_assert(nvdec->clk);
		usleep_range(10, 100);
		tegra_periph_reset_deassert(nvdec->clk);
		tegra_mc_flush_done(true);
#else
		reset_control_assert(nvdec->rst);
		usleep_range(10, 100);
		reset_control_deassert(nvdec->rst);
#endif
	}

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
		falcon_exit(&nvdec->falcon_ls);

	falcon_exit(&nvdec->falcon_bl);

	if (nvdec->domain) {
		iommu_detach_device(nvdec->domain, nvdec->dev);
		nvdec->domain = NULL;
	}

	return 0;
}

static const struct host1x_client_ops nvdec_client_ops = {
	.init = nvdec_init,
	.exit = nvdec_exit,
};

static int nvdec_open_channel(struct tegra_drm_client *client,
			    struct tegra_drm_context *context)
{
	struct nvdec *nvdec = to_nvdec(client);
	int err;

	err = pm_runtime_get_sync(nvdec->dev);
	if (err < 0)
		return err;

	/*
	 * Try to boot the Falcon microcontroller. Booting is deferred until
	 * here because the firmware might not yet be available during system
	 * boot, for example if it's on remote storage.
	 */
	err = nvdec_boot(nvdec);
	if (err < 0) {
		pm_runtime_put(nvdec->dev);
		return err;
	}

	context->channel = host1x_channel_get(nvdec->channel);
	if (!context->channel) {
		pm_runtime_put(nvdec->dev);
		return -ENOMEM;
	}

	return 0;
}

static void nvdec_close_channel(struct tegra_drm_context *context)
{
	struct nvdec *nvdec = to_nvdec(context->client);

	host1x_channel_put(context->channel);

	pm_runtime_put(nvdec->dev);
}

static const struct tegra_drm_client_ops nvdec_ops = {
	.open_channel = nvdec_open_channel,
	.close_channel = nvdec_close_channel,
	.submit = tegra_drm_submit,
};

static const struct nvdec_config nvdec_t210_config = {
	.ucode_name = "tegra21x/nvhost_nvdec020_ns.fw",
	.ucode_name_bl = "tegra21x/nvhost_nvdec_bl020_prod.fw",
	.ucode_name_ls = "tegra21x/nvhost_nvdec020_prod.fw",
};

static const struct of_device_id nvdec_match[] = {
	{ .compatible = "nvidia,tegra210-nvdec", .data = &nvdec_t210_config },
	{ },
};

static int nvdec_probe(struct platform_device *pdev)
{
	struct nvdec_config *nvdec_config = NULL;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	const struct of_device_id *match;
	struct nvdec *nvdec;
	int err;

	match = of_match_device(nvdec_match, dev);
	nvdec_config = (struct nvdec_config *)match->data;

	nvdec = devm_kzalloc(dev, sizeof(*nvdec), GFP_KERNEL);
	if (!nvdec)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "failed to get registers\n");
		return -ENXIO;
	}

	nvdec->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(nvdec->regs))
		return PTR_ERR(nvdec->regs);

	nvdec->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nvdec->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(nvdec->clk);
	}

	nvdec->rst = devm_reset_control_get(dev, "nvdec");
	if (IS_ERR(nvdec->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(nvdec->rst);
	}

	platform_set_drvdata(pdev, nvdec);

	INIT_LIST_HEAD(&nvdec->client.base.list);
	nvdec->client.base.ops = &nvdec_client_ops;
	nvdec->client.base.dev = dev;
	nvdec->client.base.class = HOST1X_CLASS_NVDEC;
	nvdec->client.base.syncpts = syncpts;
	nvdec->client.base.num_syncpts = 1;
	nvdec->dev = dev;
	nvdec->config = nvdec_config;

	INIT_LIST_HEAD(&nvdec->client.list);
	nvdec->client.ops = &nvdec_ops;

	err = host1x_client_register(&nvdec->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		platform_set_drvdata(pdev, NULL);
		return err;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		err = nvdec_runtime_resume(&pdev->dev);
		if (err < 0)
			goto unregister_client;
	}

	dev_info(&pdev->dev, "initialized");

	return 0;

unregister_client:
	host1x_client_unregister(&nvdec->client.base);

	return err;
}

static int nvdec_remove(struct platform_device *pdev)
{
	struct nvdec *nvdec = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&nvdec->client.base);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	if (pm_runtime_enabled(&pdev->dev))
		pm_runtime_disable(&pdev->dev);
	else
		nvdec_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops nvdec_pm_ops = {
	SET_RUNTIME_PM_OPS(nvdec_runtime_suspend, nvdec_runtime_resume, NULL)
};

struct platform_driver tegra_nvdec_driver = {
	.driver = {
		.name = "tegra-nvdec",
		.of_match_table = nvdec_match,
		.pm = &nvdec_pm_ops
	},
	.probe = nvdec_probe,
	.remove = nvdec_remove,
};
