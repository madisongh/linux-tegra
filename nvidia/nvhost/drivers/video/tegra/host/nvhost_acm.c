/*
 * drivers/video/tegra/host/nvhost_acm.c
 *
 * Tegra Graphics Host Automatic Clock Management
 *
 * Copyright (c) 2010-2017, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/clk/tegra.h>
#include <linux/reset.h>
#include <linux/tegra-powergate.h>
#include <soc/tegra/chip-id.h>
#include <trace/events/nvhost.h>
#include <linux/tegra_pm_domains.h>
#include <linux/nvhost_ioctl.h>
#include <linux/version.h>

#include <linux/platform/tegra/mc.h>
#if defined(CONFIG_TEGRA_BWMGR)
#include <linux/platform/tegra/emc_bwmgr.h>
#endif

#include "nvhost_acm.h"
#include "nvhost_vm.h"
#include "nvhost_scale.h"
#include "nvhost_channel.h"
#include "dev.h"
#include "bus_client.h"

#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT	(2 * HZ)
#define POWERGATE_DELAY 			10
#define MAX_DEVID_LENGTH			32

static void nvhost_module_load_regs(struct platform_device *pdev, bool prod);
static int nvhost_module_toggle_slcg(struct notifier_block *nb,
				     unsigned long action, void *data);

#ifdef CONFIG_PM
static int nvhost_module_prepare_suspend(struct device *dev);
static int nvhost_module_suspend(struct device *dev);
static int nvhost_module_resume(struct device *dev);
static int nvhost_module_runtime_suspend(struct device *dev);
static int nvhost_module_runtime_resume(struct device *dev);
static int nvhost_module_prepare_poweroff(struct device *dev);
static int nvhost_module_finalize_poweron(struct device *dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
static int nvhost_module_power_on(struct generic_pm_domain *domain);
static int nvhost_module_power_off(struct generic_pm_domain *domain);
#endif
#endif

struct nvhost_pm_domain {
	struct generic_pm_domain domain;
	struct list_head list;
	int powergate_id;
};

static struct nvhost_pm_domain *genpd_to_nvhost_pd(
	struct generic_pm_domain *genpd)
{
	return container_of(genpd, struct nvhost_pm_domain, domain);
}

static DEFINE_MUTEX(client_list_lock);

struct nvhost_module_client {
	struct list_head node;
	unsigned long constraint[NVHOST_MODULE_MAX_CLOCKS];
	unsigned long type[NVHOST_MODULE_MAX_CLOCKS];
	void *priv;
};

static bool nvhost_module_emc_clock(struct nvhost_clock *clock)
{
	return (clock->moduleid ==
		NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER) ||
	       (clock->moduleid == NVHOST_MODULE_ID_EMC_SHARED);
}

static bool nvhost_is_bwmgr_clk(struct nvhost_device_data *pdata, int index)
{

	return (nvhost_module_emc_clock(&pdata->clocks[index]) &&
		pdata->bwmgr_handle);
}

static int do_powergate_locked(int id)
{
	int ret;

	nvhost_dbg_fn("%d", id);
	if (id == -1 || !tegra_powergate_is_powered(id))
		return 0;

	ret = tegra_powergate_partition(id);
	if (ret && tegra_platform_is_sim()) {
		pr_err("%s: running on simulator, ignoring powergate failure\n",
		       __func__);
		ret = 0;
	}

	return ret;
}

static int do_unpowergate_locked(int id)
{
	int ret;

	if (id == -1)
		return 0;

	ret = tegra_unpowergate_partition(id);
	if (ret) {
		pr_err("%s: unpowergate failed: id = %d\n", __func__, id);
		if (tegra_platform_is_sim()) {
			pr_err("%s: running on simulator, ignoring failure\n",
			       __func__);
			ret = 0;
		}
	}

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
static void dump_clock_status(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	unsigned long rate = 0;

	int i;

	pr_info("\n%s: %s status:\n", __func__, dev_name(&dev->dev));

	for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
		if (!pdata->clocks[i].name)
			break;

#if defined(CONFIG_TEGRA_BWMGR)
		if (nvhost_is_bwmgr_clk(pdata, i))
			rate = tegra_bwmgr_get_emc_rate();
		else
#endif
			if (pdata->clk[i])
				rate = clk_get_rate(pdata->clk[i]);

		pr_info("%s: clock %s: rate = %lu\n",
			__func__, pdata->clocks[i].name,
			rate);
	}
}
#endif

static void do_module_reset_locked(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	int ret;
#endif

	if (pdata->reset) {
		pdata->reset(dev);
		return;
	}

	if (pdata->reset_control) {
		reset_control_reset(pdata->reset_control);
		return;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	/* assert module and mc client reset */
	if (pdata->clk[0] && pdata->clocks[0].reset) {
		ret = tegra_mc_flush(pdata->clocks[0].reset);
		if (ret) {
			dump_clock_status(nvhost_get_host(dev)->dev);
			dump_clock_status(dev);
		}
		tegra_periph_reset_assert(pdata->clk[0]);
	}

	if (pdata->clk[1] && pdata->clocks[1].reset) {
		ret = tegra_mc_flush(pdata->clocks[1].reset);
		if (ret) {
			dump_clock_status(nvhost_get_host(dev)->dev);
			dump_clock_status(dev);
		}
		tegra_periph_reset_assert(pdata->clk[1]);
	}

	udelay(POWERGATE_DELAY);

	/* deassert reset */
	if (pdata->clk[0] && pdata->clocks[0].reset) {
		tegra_periph_reset_deassert(pdata->clk[0]);
		tegra_mc_flush_done(pdata->clocks[0].reset);
	}

	if (pdata->clk[1] && pdata->clocks[1].reset) {
		tegra_periph_reset_deassert(pdata->clk[1]);
		tegra_mc_flush_done(pdata->clocks[1].reset);
	}
#endif
}

static unsigned long nvhost_emc_bw_to_freq_req(unsigned long rate)
{
	return tegra_emc_bw_to_freq_req((unsigned long)(rate));
}

void nvhost_module_reset(struct platform_device *dev, bool reboot)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	dev_dbg(&dev->dev,
		"%s: asserting %s module reset (id %d)\n",
		__func__, dev_name(&dev->dev),
		pdata->powergate_id);

	/* Ensure that the device state is sane (i.e. device specifics
	 * IRQs get disabled */
	if (reboot)
		if (pdata->prepare_poweroff)
			pdata->prepare_poweroff(dev);

	mutex_lock(&pdata->lock);
	do_module_reset_locked(dev);
	mutex_unlock(&pdata->lock);

	if (reboot) {
		/* Load clockgating registers */
		nvhost_module_load_regs(dev, pdata->engine_can_cg);

		/* initialize device vm */
		nvhost_vm_init_device(dev);

		/* ..and execute engine specific operations (i.e. boot) */
		if (pdata->finalize_poweron)
			pdata->finalize_poweron(dev);
	}

	dev_dbg(&dev->dev, "%s: module %s out of reset\n",
		__func__, dev_name(&dev->dev));
}
EXPORT_SYMBOL(nvhost_module_reset);

void nvhost_module_busy_noresume(struct platform_device *dev)
{
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		nvhost_module_busy_noresume(nvhost_get_parent(dev));

#ifdef CONFIG_PM
	pm_runtime_get_noresume(&dev->dev);
#endif
}

int nvhost_module_busy(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int ret = 0;

	/* Explicitly turn on the host1x clocks
	 * - This is needed as host1x driver sets ignore_children = true
	 * to cater the use case of display clock ON but host1x clock OFF
	 * in OS-Idle-Display-ON case
	 * - This was easily done in ACM as it only checked the ref count
	 * of host1x (or any device for that matter) to be zero before
	 * turning off its clock
	 * - However, runtime PM checks to see if *ANY* child of device is
	 * in ACTIVE state and if yes, it doesn't suspend the parent. As a
	 * result of this, display && host1x clocks remains ON during
	 * OS-Idle-Display-ON case
	 * - The code below fixes this use-case
	 */
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		ret = nvhost_module_busy(nvhost_get_parent(dev));

	if (ret)
		return ret;

#ifdef CONFIG_PM
	ret = pm_runtime_get_sync(&dev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&dev->dev);
		if (dev->dev.parent && (dev->dev.parent != &platform_bus))
			nvhost_module_idle(nvhost_get_parent(dev));
		nvhost_err(&dev->dev, "failed to power on, err %d", ret);
		return ret;
	}
#else
	if (!pdata->booted && pdata->finalize_poweron) {
		ret = nvhost_vm_init_device(dev);
		if (ret < 0) {
			nvhost_err(&dev->dev, "failed to init vm, err %d",
				   ret);
			return ret;
		}

		ret = pdata->finalize_poweron(dev);
		if (ret < 0) {
			nvhost_err(&dev->dev, "failed to power on, err %d",
				   ret);
			return ret;
		}
		pdata->booted = true;
	}
#endif

	if (pdata->busy)
		pdata->busy(dev);

	return 0;
}
EXPORT_SYMBOL(nvhost_module_busy);

inline void nvhost_module_idle(struct platform_device *dev)
{
	nvhost_module_idle_mult(dev, 1);
}
EXPORT_SYMBOL(nvhost_module_idle);

void nvhost_module_idle_mult(struct platform_device *dev, int refs)
{
	int original_refs = refs;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

#ifdef CONFIG_PM
	/* call idle callback only if the device is turned on. */
	if (atomic_read(&dev->dev.power.usage_count) == refs &&
	    pm_runtime_active(&dev->dev)) {
		if (pdata->idle)
			pdata->idle(dev);
	}

	while (refs--) {
		pm_runtime_mark_last_busy(&dev->dev);
		if (pdata->autosuspend_delay)
			pm_runtime_put_autosuspend(&dev->dev);
		else
			pm_runtime_put(&dev->dev);
	}
#else
	if (pdata->idle)
		pdata->idle(dev);
#endif

	/* Explicitly turn off the host1x clocks */
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		nvhost_module_idle_mult(nvhost_get_parent(dev), original_refs);
}

int nvhost_module_get_rate(struct platform_device *dev, unsigned long *rate,
		int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int err = 0;

	/* Need to enable client to get correct rate */
	err = nvhost_module_busy(dev);
	if (err)
		return err;

#if defined(CONFIG_TEGRA_BWMGR)
	if (nvhost_is_bwmgr_clk(pdata, index))
		*rate = tegra_bwmgr_get_emc_rate();
	else
#endif
		if (pdata->clk[index])
			*rate = clk_get_rate(pdata->clk[index]);
		else
			err = -EINVAL;

	nvhost_module_idle(dev);
	return err;
}

static int nvhost_module_update_rate(struct platform_device *dev, int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	unsigned long bw_constraint = 0, floor_rate = 0, pixelrate = 0;
	unsigned long rate = 0;
	struct nvhost_module_client *m;
	int ret = -EINVAL;

	if (!nvhost_is_bwmgr_clk(pdata, index) && !pdata->clk[index])
		return -EINVAL;

	/* don't update rate if scaling is disabled for this clock */
	if (pdata->clocks[index].disable_scaling)
		return 0;

	/* aggregate client constraints */
	list_for_each_entry(m, &pdata->client_list, node) {
		unsigned long constraint = m->constraint[index];
		unsigned long type = m->type[index];

		if (!constraint)
			continue;

		/* Note: We need to take max to avoid wrapping issues */
		if (type == NVHOST_BW)
			bw_constraint = max(bw_constraint,
				bw_constraint + (constraint / 1000));
		else if (type == NVHOST_PIXELRATE)
			pixelrate = max(pixelrate,
				pixelrate + constraint);
		else if (type == NVHOST_BW_KHZ)
			bw_constraint = max(bw_constraint,
				bw_constraint + constraint);
		else
			floor_rate = max(floor_rate, constraint);
	}

	/* use client specific aggregation if available */
	if (pdata->aggregate_constraints)
		rate = pdata->aggregate_constraints(dev, index, floor_rate,
						    pixelrate, bw_constraint);

	/* if frequency is not available, use default policy */
	if (!rate) {
		unsigned long bw_freq_khz =
			nvhost_emc_bw_to_freq_req(bw_constraint);
		bw_freq_khz = min(ULONG_MAX / 1000, bw_freq_khz);
		rate = max(floor_rate, bw_freq_khz * 1000);
		if (pdata->num_ppc)
			rate = max(rate, pixelrate / pdata->num_ppc);
	}

	/* take devfreq rate into account */
	rate = max(rate, pdata->clocks[index].devfreq_rate);

	/* if we still don't have any rate, use default */
	if (!rate)
		rate = pdata->clocks[index].default_rate;

	trace_nvhost_module_update_rate(dev->name, pdata->clocks[index].name,
					rate);

#if defined(CONFIG_TEGRA_BWMGR)
	if (nvhost_is_bwmgr_clk(pdata, index))
		ret = tegra_bwmgr_set_emc(pdata->bwmgr_handle, rate,
			pdata->clocks[index].bwmgr_request_type);
	else
#endif
		ret = clk_set_rate(pdata->clk[index], rate);

	return ret;

}

int nvhost_module_set_rate(struct platform_device *dev, void *priv,
		unsigned long constraint, int index, unsigned long type)
{
	struct nvhost_module_client *m;
	int ret = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s", dev->name);

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &pdata->client_list, node) {
		if (m->priv == priv) {
			m->constraint[index] = constraint;
			m->type[index] = type;
		}
	}

	ret = nvhost_module_update_rate(dev, index);
	mutex_unlock(&client_list_lock);
	return ret;
}

int nvhost_module_add_client(struct platform_device *dev, void *priv)
{
	struct nvhost_module_client *client;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s num_clks=%d priv=%p", dev->name,
		      pdata->num_clks, priv);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	mutex_lock(&client_list_lock);
	list_add_tail(&client->node, &pdata->client_list);
	mutex_unlock(&client_list_lock);

	return 0;
}

void nvhost_module_remove_client(struct platform_device *dev, void *priv)
{
	int i;
	struct nvhost_module_client *m;
	int found = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s priv=%p", dev->name, priv);

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &pdata->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			found = 1;
			break;
		}
	}
	if (found) {
		kfree(m);
		for (i = 0; i < pdata->num_clks; i++)
			nvhost_module_update_rate(dev, i);
	}
	mutex_unlock(&client_list_lock);
}

static ssize_t force_on_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int force_on = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr,
			     power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	ret = sscanf(buf, "%d", &force_on);
	if (ret != 1)
		return -EINVAL;

	/* should we force the power on or off? */
	if (force_on && !pdata->forced_on) {
		/* force on */
		ret = nvhost_module_busy(dev);
		if (!ret)
			pdata->forced_on = true;
	} else if (!force_on && pdata->forced_on) {
		/* force off */
		nvhost_module_idle(dev);
		pdata->forced_on = false;
	}

	return count;
}

static ssize_t force_on_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr,
			     power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", pdata->forced_on);

}

static ssize_t autosuspend_delay_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int autosuspend_delay = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	ret = sscanf(buf, "%d", &autosuspend_delay);
	if (ret == 1 && autosuspend_delay >= 0) {
		mutex_lock(&pdata->lock);
		pdata->autosuspend_delay = autosuspend_delay;
		mutex_unlock(&pdata->lock);

		pm_runtime_set_autosuspend_delay(&dev->dev,
				pdata->autosuspend_delay);
	} else {
		dev_err(&dev->dev, "Invalid autosuspend delay\n");
	}

	return count;
}

static ssize_t autosuspend_delay_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	mutex_lock(&pdata->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pdata->autosuspend_delay);
	mutex_unlock(&pdata->lock);

	return ret;
}

int nvhost_clk_get(struct platform_device *dev, char *name, struct clk **clk)
{
	int i;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	for (i = 0; i < pdata->num_clks; i++) {
		if (strcmp(pdata->clocks[i].name, name) == 0) {
			*clk = pdata->clk[i];
			return 0;
		}
	}
	return -EINVAL;
}

static int nvhost_module_set_parent(struct platform_device *dev,
			     struct nvhost_clock *clock, struct clk *clk)
{
	struct clk *parent_clk;
	char parent_name[MAX_DEVID_LENGTH];
	int err;

	/* set parent only if CCF is enabled. TCF handles this otherwise */
	if (!IS_ENABLED(CONFIG_COMMON_CLK))
		return 0;

	snprintf(parent_name, sizeof(parent_name), "%s_parent", clock->name);

	/* if parent is not available, assume that
	 * we do not need to change it.
	 */
	parent_clk = devm_clk_get(&dev->dev, parent_name);
	if (IS_ERR(parent_clk))
		return 0;

	err = clk_set_parent(clk, parent_clk);

	devm_clk_put(&dev->dev, parent_clk);

	return err;
}

int nvhost_module_init(struct platform_device *dev)
{
	int i = 0, err = 0;
	struct kobj_attribute *attr = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* initialize clocks to known state (=enabled) */
	pdata->num_clks = 0;
	INIT_LIST_HEAD(&pdata->client_list);

	if (nvhost_dev_is_virtual(dev)) {
		pm_runtime_enable(&dev->dev);

		/* If powergating is disabled, take a reference on the device
		 * without turning it on
		 */
		if (!pdata->can_powergate)
			nvhost_module_busy_noresume(dev);

		return err;
	}

#if defined(CONFIG_TEGRA_BWMGR)
	/* get bandwidth manager handle if needed */
	if (pdata->bwmgr_client_id)
		pdata->bwmgr_handle =
			tegra_bwmgr_register(pdata->bwmgr_client_id);
#endif

	while (i < NVHOST_MODULE_MAX_CLOCKS && pdata->clocks[i].name) {
		char devname[MAX_DEVID_LENGTH];
		long rate = pdata->clocks[i].default_rate;
		struct clk *c;

#if defined(CONFIG_TEGRA_BWMGR)
		if (nvhost_is_bwmgr_clk(pdata, i)) {
			tegra_bwmgr_set_emc(pdata->bwmgr_handle, 0,
				pdata->clocks[i].bwmgr_request_type);
			pdata->clk[pdata->num_clks++] = NULL;
			i++;
			continue;
		}
#endif

		snprintf(devname, MAX_DEVID_LENGTH,
			 (dev->id <= 0) ? "tegra_%s" : "tegra_%s.%d",
			 dev->name, dev->id);

		/* Get device managed clock if CCF is available, otherwise
		 * assume tegra specific clock framework */

		if (IS_ENABLED(CONFIG_COMMON_CLK))
			c = devm_clk_get(&dev->dev, pdata->clocks[i].name);
		else
			c = clk_get_sys(devname, pdata->clocks[i].name);

		if (IS_ERR(c)) {
			dev_err(&dev->dev, "clk_get_sys failed for i=%d %s:%s",
				i, devname, pdata->clocks[i].name);
			/* arguably we should fail init here instead... */
			i++;
			continue;
		}

		nvhost_dbg_fn("%s->clk[%d] -> %s:%s:%p",
			      dev->name, pdata->num_clks,
			      devname, pdata->clocks[i].name,
			      c);

		/* Update parent */
		err = nvhost_module_set_parent(dev, &pdata->clocks[i], c);
		if (err)
			dev_warn(&dev->dev, "failed to set parent clock %s (err=%d)",
				 pdata->clocks[i].name, err);

		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		clk_prepare_enable(c);

		pdata->clk[pdata->num_clks++] = c;
		i++;
	}
	pdata->num_clks = i;

	/* try to get reset control - if it fails, we use Tegra specific
	 * control as fall-back */
	pdata->reset_control = devm_reset_control_get(&dev->dev, NULL);
	if (IS_ERR(pdata->reset_control))
		pdata->reset_control = NULL;

	/* reset the module */
	mutex_lock(&pdata->lock);
	do_module_reset_locked(dev);
	mutex_unlock(&pdata->lock);

	/* disable the clocks */
	for (i = 0; i < pdata->num_clks; ++i) {
		if (pdata->clk[i])
			clk_disable_unprepare(pdata->clk[i]);
	}


	/* disable railgating if pm runtime is not available
	 * and for linsim platform */
	pdata->can_powergate = IS_ENABLED(CONFIG_PM) &&
		IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS) &&
		pdata->can_powergate && !tegra_platform_is_linsim() &&
		!tegra_platform_is_vdk();

	if (dev->dev.pm_domain) {
		struct nvhost_pm_domain *pd;
		pd = genpd_to_nvhost_pd(pd_to_genpd(dev->dev.pm_domain));
		pdata->powergate_id = pd->powergate_id;
	} else {
		pdata->powergate_id = -1;
	}

	/* needed to WAR MBIST issue */
	if (pdata->poweron_toggle_slcg) {
		pdata->toggle_slcg_notifier.notifier_call =
			&nvhost_module_toggle_slcg;
		if (pdata->powergate_id != -1)
			slcg_register_notifier(pdata->powergate_id,
				&pdata->toggle_slcg_notifier);
	}

	/* Ensure that the above or device specific MBIST WAR gets applied */
	if (pdata->poweron_toggle_slcg || pdata->slcg_notifier_enable) {
		do_powergate_locked(pdata->powergate_id);
		do_unpowergate_locked(pdata->powergate_id);
	}

	/* power gate units that we can power gate */
	if (pdata->can_powergate) {
		do_powergate_locked(pdata->powergate_id);
	} else {
		do_unpowergate_locked(pdata->powergate_id);
	}

	/* set pm runtime delays */
	if (pdata->autosuspend_delay) {
		pm_runtime_set_autosuspend_delay(&dev->dev,
			pdata->autosuspend_delay);
		pm_runtime_use_autosuspend(&dev->dev);
	}

	/* initialize no_poweroff_req_mutex */
	mutex_init(&pdata->no_poweroff_req_mutex);

	/* turn on pm runtime */
	pm_runtime_enable(&dev->dev);
	if (!pm_runtime_enabled(&dev->dev))
		nvhost_module_enable_clk(&dev->dev);

	/* If powergating is disabled, take a reference on the device without
	 * turning it on
	 */
	if (!pdata->can_powergate)
		nvhost_module_busy_noresume(dev);

	/* if genpd is not available, the domain is powered already.
	 * just ensure that we load the gating registers now */
	if (!(IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS) &&
	      IS_ENABLED(CONFIG_PM))) {
		nvhost_module_enable_clk(&dev->dev);
		nvhost_module_load_regs(dev, pdata->engine_can_cg);
		nvhost_module_disable_clk(&dev->dev);
	}

	/* Init the power sysfs attributes for this device */
	pdata->power_attrib = devm_kzalloc(&dev->dev,
		sizeof(struct nvhost_device_power_attr), GFP_KERNEL);
	if (!pdata->power_attrib) {
		dev_err(&dev->dev, "Unable to allocate sysfs attributes\n");
		return -ENOMEM;
	}
	pdata->power_attrib->ndev = dev;

	pdata->power_kobj = kobject_create_and_add("acm", &dev->dev.kobj);
	if (!pdata->power_kobj) {
		dev_err(&dev->dev, "Could not add dir 'power'\n");
		err = -EIO;
		goto fail_attrib_alloc;
	}

	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY];
	attr->attr.name = "autosuspend_delay";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = autosuspend_delay_show;
	attr->store = autosuspend_delay_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(pdata->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute autosuspend_delay\n");
		err = -EIO;
		goto fail_autosuspenddelay;
	}

	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON];
	attr->attr.name = "force_on";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = force_on_show;
	attr->store = force_on_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(pdata->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute force_on\n");
		err = -EIO;
		goto fail_forceon;
	}

	return 0;

fail_forceon:
	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY];
	sysfs_remove_file(pdata->power_kobj, &attr->attr);

fail_autosuspenddelay:
	kobject_put(pdata->power_kobj);

fail_attrib_alloc:
	return err;
}
EXPORT_SYMBOL(nvhost_module_init);

void nvhost_module_deinit(struct platform_device *dev)
{
	int i;
	struct kobj_attribute *attr = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	devfreq_suspend_device(pdata->power_manager);

	if (pm_runtime_enabled(&dev->dev)) {
		pm_runtime_disable(&dev->dev);
	} else {
		/* call device specific poweroff call */
		if (pdata->booted && pdata->prepare_poweroff)
			pdata->prepare_poweroff(dev);

		/* disable clock.. */
		nvhost_module_disable_clk(&dev->dev);

		/* ..and turn off the module */
		do_powergate_locked(pdata->powergate_id);
	}

#if defined(CONFIG_TEGRA_BWMGR)
	if (pdata->bwmgr_handle)
		tegra_bwmgr_unregister(pdata->bwmgr_handle);
#endif

	if (!IS_ENABLED(CONFIG_COMMON_CLK))
		for (i = 0; i < pdata->num_clks; i++) {
			if (pdata->clk[i])
				clk_put(pdata->clk[i]);
	}

	if (pdata->power_kobj) {
		for (i = 0; i < NVHOST_POWER_SYSFS_ATTRIB_MAX; i++) {
			attr = &pdata->power_attrib->power_attr[i];
			sysfs_remove_file(pdata->power_kobj, &attr->attr);
		}

		kobject_put(pdata->power_kobj);
	}
}
EXPORT_SYMBOL(nvhost_module_deinit);

/*
 * Suspend / resume control flow on different kernel versions:
 * 3.10:
 * Suspend:
 * - genpd's prepare calls runtime_resume
 * - System suspend calls runtime_suspend
 * Resume:
 * - System resume calls runtime_resume
 *
 * 3.18 / 4.4:
 * Suspend:
 * - genpd's prepare calls runtime_resume
 * - genpd's suspend_noirq calls runtime_suspend
 * Resume:
 * - genpd's complete calls runtime_resume
 */
const struct dev_pm_ops nvhost_module_pm_ops = {
	SET_RUNTIME_PM_OPS(nvhost_module_runtime_suspend,
			   nvhost_module_runtime_resume, NULL)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
	/* On 3.18 and 4.4, genpd will call our runtime pm ops. */
	SET_SYSTEM_SLEEP_PM_OPS(nvhost_module_runtime_suspend,
				nvhost_module_runtime_resume)
#else
	.prepare = nvhost_module_prepare_suspend,
	.suspend = nvhost_module_suspend,
	.resume = nvhost_module_resume,
#endif
};
EXPORT_SYMBOL(nvhost_module_pm_ops);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
static int pm_subdomain_attach(struct device_node *node)
{
	int ret;
	struct of_phandle_args master_phandle, child_phandle;

	child_phandle.np = node;
	child_phandle.args_count = 0;

	ret = of_parse_phandle_with_args(node, "power-domains",
					 "#power-domain-cells", 0,
					 &master_phandle);
	if (ret < 0)
		return ret;

	pr_info("Adding domain %s to PM domain %s\n",
		node->name, master_phandle.np->name);

	return of_genpd_add_subdomain(&master_phandle, &child_phandle);
}
#endif

static LIST_HEAD(pd_list);

static int _nvhost_init_domain(struct device_node *np)
{
#ifdef CONFIG_PM_GENERIC_DOMAINS
	struct nvhost_pm_domain *pd;
	struct list_head *head;
	bool is_off = false;

	list_for_each(head, &pd_list) {
		struct nvhost_pm_domain *list_pd;
		list_pd = list_entry(head, struct nvhost_pm_domain, list);
		/* Domain already registered somewhere else */
		if (list_pd->domain.name == np->name)
			return 0;
	}

	if (of_property_read_bool(np, "is_off"))
		is_off = true;

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	if (of_property_read_u32(np, "partition-id", &pd->powergate_id))
		pd->powergate_id = -1;

	pd->domain.name = (char *)np->name;

	pm_genpd_init(&pd->domain, NULL, is_off);

	pd->domain.power_off = nvhost_module_power_off;
	pd->domain.power_on = nvhost_module_power_on;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	pd->domain.flags = GENPD_FLAG_PM_UPSTREAM;
#endif

	of_genpd_add_provider_simple(np, &pd->domain);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	genpd_pm_subdomain_attach(&pd->domain);
#else
	pm_subdomain_attach(np);
#endif

	list_add(&pd->list, &pd_list);
#endif
	return 0;
}

int nvhost_domain_init(struct of_device_id *matches)
{
	struct device_node *np;
	int ret = 0;

	for_each_matching_node(np, matches) {
		ret = _nvhost_init_domain(np);
		if (ret)
			break;
	}
	return ret;

}
EXPORT_SYMBOL(nvhost_domain_init);

void nvhost_register_client_domain(struct generic_pm_domain *domain)
{
}
EXPORT_SYMBOL(nvhost_register_client_domain);

void nvhost_unregister_client_domain(struct generic_pm_domain *domain)
{
}
EXPORT_SYMBOL(nvhost_unregister_client_domain);

int nvhost_module_enable_clk(struct device *dev)
{
	int index = 0;
	struct nvhost_device_data *pdata;

	/* enable parent's clock if required */
	if (dev->parent && dev->parent != &platform_bus)
		nvhost_module_enable_clk(dev->parent);

	pdata = dev_get_drvdata(dev);
	if (!pdata)
		return -EINVAL;

	for (index = 0; index < pdata->num_clks; index++) {
		int err;

		if (!pdata->clk[index])
			continue;

		err = clk_prepare_enable(pdata->clk[index]);
		if (err) {
			dev_err(dev, "Cannot turn on clock %s",
				pdata->clocks[index].name);
			return -EINVAL;
		}
	}

	trace_nvhost_module_enable_clk(pdata->pdev->name,
					pdata->num_clks);

	return 0;
}

int nvhost_module_disable_clk(struct device *dev)
{
	int index = 0;
	struct nvhost_device_data *pdata;

	pdata = dev_get_drvdata(dev);
	if (!pdata)
		return -EINVAL;

	trace_nvhost_module_disable_clk(pdata->pdev->name,
					pdata->num_clks);

	for (index = 0; index < pdata->num_clks; index++)
		if (pdata->clk[index])
			clk_disable_unprepare(pdata->clk[index]);

	/* disable parent's clock if required */
	if (dev->parent && dev->parent != &platform_bus)
		nvhost_module_disable_clk(dev->parent);

	return 0;
}

static void nvhost_module_load_regs(struct platform_device *pdev, bool prod)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_gating_register *regs = pdata->engine_cg_regs;

	if (!regs)
		return;

	while (regs->addr) {
		if (prod)
			host1x_writel(pdev, regs->addr, regs->prod);
		else
			host1x_writel(pdev, regs->addr, regs->disable);
		regs++;
	}
}

#ifdef CONFIG_PM
static int nvhost_module_runtime_suspend(struct device *dev)
{
	int err;

	if (atomic_read(&dev->power.usage_count) == 1)
		return 0;

	dev_dbg(dev, "runtime suspending");

	err = nvhost_module_prepare_poweroff(dev);
	if (err)
		return err;

	err = nvhost_module_disable_clk(dev);
	if (err)
		return err;

	return 0;
}

static int nvhost_module_runtime_resume(struct device *dev)
{
	int err;

	dev_dbg(dev, "runtime resuming");

	err = nvhost_module_enable_clk(dev);
	if (err)
		return err;

	err = nvhost_module_finalize_poweron(dev);
	if (err) {
		nvhost_module_disable_clk(dev);
		return err;
	}

	return 0;
}

static bool nvhost_module_has_open_channels(struct platform_device *pdev)
{
	struct nvhost_master *host = nvhost_get_host(pdev);
	struct nvhost_channel *ch = NULL;
	int idx = 0;

	for (idx = 0; idx < nvhost_channel_nb_channels(host); idx++) {
		ch = host->chlist[idx];
		if (ch->dev == pdev)
			return true;
	}

	return false;
}

#ifdef CONFIG_PM_GENERIC_DOMAINS
static int nvhost_module_power_on(struct generic_pm_domain *domain)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(domain);

	trace_nvhost_module_power_on(pd->domain.name, pd->powergate_id);
	return do_unpowergate_locked(pd->powergate_id);
}

static int nvhost_module_power_off(struct generic_pm_domain *domain)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(domain);

	trace_nvhost_module_power_off(pd->domain.name, pd->powergate_id);
	return do_powergate_locked(pd->powergate_id);
}
#endif

static int nvhost_module_prepare_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	if (nvhost_module_has_open_channels(pdev))
		return -EBUSY;

	return 0;
}

static int nvhost_module_suspend(struct device *dev)
{
	int err;
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);

	if (!pdata->power_on)
		return 0;

	dev_dbg(dev, "suspending");

	err = nvhost_module_runtime_suspend(dev);
	if (err)
		return err;

	pdata->suspended = true;

	return 0;
}

static int nvhost_module_resume(struct device *dev)
{
	int err;
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);

	if (!pdata->suspended)
		return 0;

	dev_dbg(dev, "resuming");

	err = nvhost_module_runtime_resume(dev);
	if (err)
		return err;

	pdata->suspended = false;

	return 0;
}

static int nvhost_module_prepare_poweroff(struct device *dev)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;

	pdata = dev_get_drvdata(dev);
	if (!pdata)
		return -EINVAL;

	if (!pdata->power_on)
		return 0;

	host = nvhost_get_host(pdata->pdev);

	devfreq_suspend_device(pdata->power_manager);
	nvhost_scale_hw_deinit(to_platform_device(dev));

	/* disable module interrupt if support available */
	if (pdata->module_irq)
		nvhost_intr_disable_module_intr(&host->intr,
						pdata->module_irq);

#if defined(CONFIG_TEGRA_BWMGR)
	/* set EMC rate to zero */
	if (pdata->bwmgr_handle) {
		int i;

		for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
			if (nvhost_module_emc_clock(&pdata->clocks[i])) {
				tegra_bwmgr_set_emc(pdata->bwmgr_handle, 0,
					pdata->clocks[i].bwmgr_request_type);
				break;
			}
		}
	}
#endif

	if (pdata->prepare_poweroff)
		pdata->prepare_poweroff(to_platform_device(dev));

	pdata->power_on = false;

	return 0;
}

static int nvhost_module_finalize_poweron(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvhost_master *host;
	struct nvhost_device_data *pdata;
	int retry_count, ret = 0, i;

	pdata = dev_get_drvdata(dev);
	if (!pdata)
		return -EINVAL;

	host = nvhost_get_host(pdata->pdev);
	/* WAR to bug 1588951: Retry booting 3 times */

	for (retry_count = 0; retry_count < 3; retry_count++) {
		if (!pdata->poweron_toggle_slcg) {
			if (pdata->poweron_reset)
				nvhost_module_reset(pdev, false);

			/* Load clockgating registers */
			nvhost_module_load_regs(pdev, pdata->engine_can_cg);
		} else {
			/* If poweron_toggle_slcg is set, following is already
			 * executed once. Skip to avoid doing it twice. */
			if (retry_count > 0) {
				/* First, reset module */
				nvhost_module_reset(pdev, false);

				/* Disable SLCG, wait and re-enable it */
				nvhost_module_load_regs(pdev, false);
				udelay(1);
				if (pdata->engine_can_cg)
					nvhost_module_load_regs(pdev, true);
			}
		}

		/* initialize device vm */
		ret = nvhost_vm_init_device(pdev);
		if (ret)
			continue;

		if (pdata->finalize_poweron)
			ret = pdata->finalize_poweron(to_platform_device(dev));

		/* Exit loop if we pass module specific initialization */
		if (!ret)
			break;
	}

	/* Failed to start the device */
	if (ret)
		goto out;

	/* set default EMC rate to zero */
	if (pdata->bwmgr_handle) {
		for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
			if (nvhost_module_emc_clock(&pdata->clocks[i])) {
				nvhost_module_update_rate(pdev, i);
				break;
			}
		}
	}

	/* enable module interrupt if support available */
	if (pdata->module_irq)
		nvhost_intr_enable_module_intr(&host->intr,
						pdata->module_irq);

	nvhost_scale_hw_init(to_platform_device(dev));
	devfreq_resume_device(pdata->power_manager);

	pdata->power_on = true;

out:
	return ret;
}
#endif

static int nvhost_module_toggle_slcg(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct nvhost_device_data *pdata =
		container_of(nb, struct nvhost_device_data,
			     toggle_slcg_notifier);
	struct platform_device *pdev = pdata->pdev;

	/* First, reset the engine */
	do_module_reset_locked(pdev);

	/* Then, disable slcg, wait a while and re-enable it */
	nvhost_module_load_regs(pdev, false);
	udelay(1);
	if (pdata->engine_can_cg)
		nvhost_module_load_regs(pdev, true);

	return 0;
}

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	return nvhost_module_powered(dev);
}
EXPORT_SYMBOL(nvhost_module_powered_ext);

int nvhost_module_busy_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	return nvhost_module_busy(dev);
}
EXPORT_SYMBOL(nvhost_module_busy_ext);

void nvhost_module_idle_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	nvhost_module_idle(dev);
}
EXPORT_SYMBOL(nvhost_module_idle_ext);
