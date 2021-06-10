/*
 * Virtualized GPU
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>

#include <nvgpu/kmem.h>

#include "vgpu/vgpu.h"
#include "vgpu/fecs_trace_vgpu.h"
#include "vgpu/clk_vgpu.h"
#include "gk20a/debug_gk20a.h"
#include "gk20a/hal_gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "gk20a/tsg_gk20a.h"
#include "gk20a/gk20a_scale.h"
#include "gk20a/channel_gk20a.h"
#include "gm20b/hal_gm20b.h"

#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>

static inline int vgpu_comm_init(struct platform_device *pdev)
{
	size_t queue_sizes[] = { TEGRA_VGPU_QUEUE_SIZES };

	return tegra_gr_comm_init(pdev, TEGRA_GR_COMM_CTX_CLIENT, 3,
				queue_sizes, TEGRA_VGPU_QUEUE_CMD,
				ARRAY_SIZE(queue_sizes));
}

static inline void vgpu_comm_deinit(void)
{
	size_t queue_sizes[] = { TEGRA_VGPU_QUEUE_SIZES };

	tegra_gr_comm_deinit(TEGRA_GR_COMM_CTX_CLIENT, TEGRA_VGPU_QUEUE_CMD,
			ARRAY_SIZE(queue_sizes));
}

int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out)
{
	void *handle;
	size_t size = size_in;
	void *data = msg;
	int err;

	err = tegra_gr_comm_sendrecv(TEGRA_GR_COMM_CTX_CLIENT,
				tegra_gr_comm_get_server_vmid(),
				TEGRA_VGPU_QUEUE_CMD, &handle, &data, &size);
	if (!err) {
		WARN_ON(size < size_out);
		memcpy(msg, data, size_out);
		tegra_gr_comm_release(handle);
	}

	return err;
}

static u64 vgpu_connect(void)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_connect_params *p = &msg.params.connect;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_CONNECT;
	p->module = TEGRA_VGPU_MODULE_GPU;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? 0 : p->handle;
}

int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_attrib_params *p = &msg.params.attrib;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_GET_ATTRIBUTE;
	msg.handle = handle;
	p->attrib = attrib;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret)
		return -1;

	*value = p->value;
	return 0;
}

static void vgpu_handle_channel_event(struct gk20a *g,
			struct tegra_vgpu_channel_event_info *info)
{
	if (info->id >= g->fifo.num_channels ||
		info->event_id >= NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX) {
		gk20a_err(g->dev, "invalid channel event");
		return;
	}

	if (info->is_tsg) {
		struct tsg_gk20a *tsg = &g->fifo.tsg[info->id];

		gk20a_tsg_event_id_post_event(tsg, info->event_id);
	} else {
		struct channel_gk20a *ch = &g->fifo.channel[info->id];

		if (!gk20a_channel_get(ch)) {
			gk20a_err(g->dev, "invalid channel %d for event %d",
					(int)info->id, (int)info->event_id);
			return;
		}
		gk20a_channel_event_id_post_event(ch, info->event_id);
		gk20a_channel_put(ch);
	}
}



static int vgpu_intr_thread(void *dev_id)
{
	struct gk20a *g = dev_id;

	while (true) {
		struct tegra_vgpu_intr_msg *msg;
		u32 sender;
		void *handle;
		size_t size;
		int err;

		err = tegra_gr_comm_recv(TEGRA_GR_COMM_CTX_CLIENT,
					TEGRA_VGPU_QUEUE_INTR, &handle,
					(void **)&msg, &size, &sender);
		if (err == -ETIME)
			continue;
		if (WARN_ON(err))
			continue;

		if (msg->event == TEGRA_VGPU_EVENT_ABORT) {
			tegra_gr_comm_release(handle);
			break;
		}

		switch (msg->event) {
		case TEGRA_VGPU_EVENT_INTR:
			if (msg->unit == TEGRA_VGPU_INTR_GR)
				vgpu_gr_isr(g, &msg->info.gr_intr);
			else if (msg->unit == TEGRA_VGPU_NONSTALL_INTR_GR)
				vgpu_gr_nonstall_isr(g,
					&msg->info.gr_nonstall_intr);
			else if (msg->unit == TEGRA_VGPU_INTR_FIFO)
				vgpu_fifo_isr(g, &msg->info.fifo_intr);
			else if (msg->unit == TEGRA_VGPU_NONSTALL_INTR_FIFO)
				vgpu_fifo_nonstall_isr(g,
						&msg->info.fifo_nonstall_intr);
			else if (msg->unit == TEGRA_VGPU_NONSTALL_INTR_CE2)
				vgpu_ce2_nonstall_isr(g,
					&msg->info.ce2_nonstall_intr);
			break;
		case TEGRA_VGPU_EVENT_FECS_TRACE:
			vgpu_fecs_trace_data_update(g);
			break;
		case TEGRA_VGPU_EVENT_CHANNEL:
			vgpu_handle_channel_event(g, &msg->info.channel_event);
			break;
		case TEGRA_VGPU_EVENT_SM_ESR:
			vgpu_gr_handle_sm_esr_event(g, &msg->info.sm_esr);
			break;
		default:
			gk20a_err(g->dev, "unknown event %u", msg->event);
			break;
		}

		tegra_gr_comm_release(handle);
	}

	while (!kthread_should_stop())
		msleep(10);
	return 0;
}

static void vgpu_remove_support(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data_from_dev(g->dev);
	struct tegra_vgpu_intr_msg msg;
	int err;

	if (g->dbg_regops_tmp_buf)
		kfree(g->dbg_regops_tmp_buf);

	if (g->pmu.remove_support)
		g->pmu.remove_support(&g->pmu);

	if (g->gr.remove_support)
		g->gr.remove_support(&g->gr);

	if (g->fifo.remove_support)
		g->fifo.remove_support(&g->fifo);

	if (g->mm.remove_support)
		g->mm.remove_support(&g->mm);

	msg.event = TEGRA_VGPU_EVENT_ABORT;
	err = tegra_gr_comm_send(TEGRA_GR_COMM_CTX_CLIENT,
				TEGRA_GR_COMM_ID_SELF, TEGRA_VGPU_QUEUE_INTR,
				&msg, sizeof(msg));
	WARN_ON(err);
	kthread_stop(priv->intr_handler);

	/* free mappings to registers, etc*/

	if (g->bar1) {
		iounmap(g->bar1);
		g->bar1 = NULL;
	}
}

static void vgpu_init_vars(struct gk20a *g)
{
	nvgpu_mutex_init(&g->poweroff_lock);
	g->regs_saved = g->regs;
	g->bar1_saved = g->bar1;

	INIT_LIST_HEAD(&g->pending_sema_waits);
	nvgpu_raw_spinlock_init(&g->pending_sema_waits_lock);
}

static int vgpu_init_support(struct platform_device *pdev)
{
	struct resource *r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct gk20a *g = get_gk20a(&pdev->dev);
	int err = 0;

	if (!r) {
		dev_err(dev_from_gk20a(g), "faield to get gk20a bar1\n");
		err = -ENXIO;
		goto fail;
	}

	g->bar1 = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(g->bar1)) {
		dev_err(dev_from_gk20a(g), "failed to remap gk20a bar1\n");
		err = PTR_ERR(g->bar1);
		goto fail;
	}
	g->bar1_mem = r;

	nvgpu_mutex_init(&g->dbg_sessions_lock);
	nvgpu_mutex_init(&g->client_lock);

	INIT_LIST_HEAD(&g->profiler_objects);

	g->dbg_regops_tmp_buf = kzalloc(SZ_4K, GFP_KERNEL);
	if (!g->dbg_regops_tmp_buf) {
		dev_err(g->dev, "couldn't allocate regops tmp buf");
		return -ENOMEM;
	}
	g->dbg_regops_tmp_buf_ops =
		SZ_4K / sizeof(g->dbg_regops_tmp_buf[0]);

	g->remove_support = vgpu_remove_support;
	return 0;

 fail:
	vgpu_remove_support(g);
	return err;
}

int vgpu_pm_prepare_poweroff(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int ret = 0;

	gk20a_dbg_fn("");

	if (!g->power_on)
		return 0;

	ret = gk20a_channel_suspend(g);
	if (ret)
		return ret;

	g->power_on = false;

	return ret;
}

static void vgpu_detect_chip(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gpu->arch = priv->constants.arch;
	gpu->impl = priv->constants.impl;
	gpu->rev = priv->constants.rev;

	gk20a_dbg_info("arch: %x, impl: %x, rev: %x\n",
			g->gpu_characteristics.arch,
			g->gpu_characteristics.impl,
			g->gpu_characteristics.rev);
}

static int vgpu_init_gpu_characteristics(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int err;

	gk20a_dbg_fn("");

	err = gk20a_init_gpu_characteristics(g);
	if (err)
		return err;

	g->gpu_characteristics.max_freq = priv->constants.max_freq;
	g->gpu_characteristics.map_buffer_batch_limit = 0;

	/* features vgpu does not support */
	g->gpu_characteristics.flags &= ~NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS;
	g->gpu_characteristics.flags &= ~NVGPU_GPU_FLAGS_SUPPORT_MAP_COMPBITS;
	g->gpu_characteristics.flags &=
		~NVGPU_GPU_FLAGS_SUPPORT_RESCHEDULE_RUNLIST;

	return 0;
}

static int vgpu_read_ptimer(struct gk20a *g, u64 *value)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_read_ptimer_params *p = &msg.params.read_ptimer;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_READ_PTIMER;
	msg.handle = vgpu_get_handle(g);

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (!err)
		*value = p->time;
	else
		gk20a_err(dev_from_gk20a(g),
			"vgpu read ptimer failed, err=%d", err);

	return err;
}

int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_get_timestamps_zipper_params *p =
			&msg.params.get_timestamps_zipper;
	int err;
	u32 i;

	gk20a_dbg_fn("");

	if (count > TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_MAX_COUNT) {
		gk20a_err(dev_from_gk20a(g),
			"count %u overflow", count);
		return -EINVAL;
	}

	if (source_id != NVGPU_GPU_GET_CPU_TIME_CORRELATION_INFO_SRC_ID_TSC) {
		gk20a_err(dev_from_gk20a(g),
			"source_id %u not supported", source_id);
		return -EINVAL;
	}

	msg.cmd = TEGRA_VGPU_CMD_GET_TIMESTAMPS_ZIPPER;
	msg.handle = vgpu_get_handle(g);
	p->source_id = TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_SRC_ID_TSC;
	p->count = count;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		gk20a_err(dev_from_gk20a(g),
			"vgpu get timestamps zipper failed, err=%d", err);
		return err;
	}

	for (i = 0; i < count; i++) {
		samples[i].cpu_timestamp = p->samples[i].cpu_timestamp;
		samples[i].gpu_timestamp = p->samples[i].gpu_timestamp;
	}

	return err;
}

void vgpu_init_hal_common(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;

	vgpu_init_fifo_ops(gops);
	vgpu_init_gr_ops(gops);
	vgpu_init_ltc_ops(gops);
	vgpu_init_mm_ops(gops);
	vgpu_init_debug_ops(gops);
	vgpu_init_dbg_session_ops(gops);
	vgpu_init_fecs_trace_ops(gops);
	vgpu_init_tsg_ops(gops);
#if defined(CONFIG_GK20A_CYCLE_STATS)
	vgpu_init_css_ops(gops);
#endif
	gops->chip_init_gpu_characteristics = vgpu_init_gpu_characteristics;
	gops->read_ptimer = vgpu_read_ptimer;
	gops->bus.get_timestamps_zipper = vgpu_get_timestamps_zipper;
}

static int vgpu_init_hal(struct gk20a *g)
{
	u32 ver = g->gpu_characteristics.arch + g->gpu_characteristics.impl;
	int err;

	switch (ver) {
	case GK20A_GPUID_GK20A:
		gk20a_dbg_info("gk20a detected");
		err = vgpu_gk20a_init_hal(g);
		break;
	case GK20A_GPUID_GM20B:
		gk20a_dbg_info("gm20b detected");
		err = vgpu_gm20b_init_hal(g);
		break;
	case NVGPU_GPUID_GP10B:
		gk20a_dbg_info("gp10b detected");
		err = vgpu_gp10b_init_hal(g);
		break;
	default:
		gk20a_err(g->dev, "no support for %x", ver);
		err = -ENODEV;
		break;
	}

	return err;
}

int vgpu_pm_finalize_poweron(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int err;

	gk20a_dbg_fn("");

	if (g->power_on)
		return 0;

	g->power_on = true;

	vgpu_detect_chip(g);
	err = vgpu_init_hal(g);
	if (err)
		goto done;

	if (g->ops.ltc.init_fs_state)
		g->ops.ltc.init_fs_state(g);

	err = vgpu_init_mm_support(g);
	if (err) {
		gk20a_err(dev, "failed to init gk20a mm");
		goto done;
	}

	err = vgpu_init_fifo_support(g);
	if (err) {
		gk20a_err(dev, "failed to init gk20a fifo");
		goto done;
	}

	err = vgpu_init_gr_support(g);
	if (err) {
		gk20a_err(dev, "failed to init gk20a gr");
		goto done;
	}

	err = g->ops.chip_init_gpu_characteristics(g);
	if (err) {
		gk20a_err(dev, "failed to init gk20a gpu characteristics");
		goto done;
	}

	gk20a_ctxsw_trace_init(g);
	gk20a_sched_ctrl_init(g);
	gk20a_channel_resume(g);

	g->sw_ready = true;

done:
	return err;
}

static int vgpu_qos_notify(struct notifier_block *nb,
			  unsigned long n, void *data)
{
	struct gk20a_scale_profile *profile =
			container_of(nb, struct gk20a_scale_profile,
			qos_notify_block);
	u32 max_freq;
	int err;

	gk20a_dbg_fn("");

	max_freq = (u32)pm_qos_read_max_bound(PM_QOS_GPU_FREQ_BOUNDS);
	err = vgpu_clk_cap_rate(profile->dev, max_freq);
	if (err)
		gk20a_err(profile->dev, "%s failed, err=%d", __func__, err);

	return NOTIFY_OK; /* need notify call further */
}

static int vgpu_pm_qos_init(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;

	if (!profile)
		return -EINVAL;

	profile->dev = dev;
	profile->qos_notify_block.notifier_call = vgpu_qos_notify;
	pm_qos_add_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&profile->qos_notify_block);

	return 0;
}

static void vgpu_pm_qos_remove(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	pm_qos_remove_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&g->scale_profile->qos_notify_block);
	kfree(g->scale_profile);
	g->scale_profile = NULL;
}

static int vgpu_pm_init(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long *freqs;
	int num_freqs;
	int err = 0;

	gk20a_dbg_fn("");

	__pm_runtime_disable(dev, false);

	if (IS_ENABLED(CONFIG_GK20A_DEVFREQ))
		gk20a_scale_init(dev);

	/* set min/max frequency based on frequency table */
	err = vgpu_clk_get_freqs(dev, &freqs, &num_freqs);
	if (err)
		return err;

	if (num_freqs < 1)
		return -EINVAL;

	g->devfreq->min_freq = freqs[0];
	g->devfreq->max_freq = freqs[num_freqs - 1];

	err = vgpu_pm_qos_init(dev);
	if (err)
		return err;

	return err;
}

static int vgpu_get_constants(struct gk20a *g)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_constants_params *p = &msg.params.constants;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_GET_CONSTANTS;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;

	if (unlikely(err)) {
		gk20a_err(g->dev, "%s failed, err=%d", __func__, err);
		return err;
	}

	if (unlikely(p->gpc_count > TEGRA_VGPU_MAX_GPC_COUNT ||
		p->max_tpc_per_gpc_count > TEGRA_VGPU_MAX_TPC_COUNT_PER_GPC)) {
		gk20a_err(g->dev, "gpc_count %d max_tpc_per_gpc %d overflow",
			(int)p->gpc_count, (int)p->max_tpc_per_gpc_count);
		return -EINVAL;
	}

	priv->constants = *p;
	return 0;
}

int vgpu_probe(struct platform_device *pdev)
{
	struct gk20a *gk20a;
	int err;
	struct device *dev = &pdev->dev;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct vgpu_priv_data *priv;

	if (!platform) {
		dev_err(dev, "no platform data\n");
		return -ENODATA;
	}

	gk20a_dbg_fn("");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	gk20a = kzalloc(sizeof(struct gk20a), GFP_KERNEL);
	if (!gk20a) {
		dev_err(dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}

	platform->g = gk20a;
	platform->vgpu_priv = priv;
	gk20a->dev = dev;

	gk20a->is_fmodel = platform->is_fmodel;

	nvgpu_kmem_init(gk20a);

	err = gk20a_user_init(dev, INTERFACE_NAME, &nvgpu_class);
	if (err)
		return err;

	vgpu_init_support(pdev);

	vgpu_init_vars(gk20a);

	init_rwsem(&gk20a->busy_lock);

	nvgpu_spinlock_init(&gk20a->mc_enable_lock);

	/* Initialize the platform interface. */
	err = platform->probe(dev);
	if (err) {
		if (err == -EPROBE_DEFER)
			dev_info(dev, "platform probe failed");
		else
			dev_err(dev, "platform probe failed");
		return err;
	}

	if (platform->late_probe) {
		err = platform->late_probe(dev);
		if (err) {
			dev_err(dev, "late probe failed");
			return err;
		}
	}

	err = vgpu_comm_init(pdev);
	if (err) {
		dev_err(dev, "failed to init comm interface\n");
		return -ENOSYS;
	}

	priv->virt_handle = vgpu_connect();
	if (!priv->virt_handle) {
		dev_err(dev, "failed to connect to server node\n");
		vgpu_comm_deinit();
		return -ENOSYS;
	}

	err = vgpu_get_constants(gk20a);
	if (err) {
		vgpu_comm_deinit();
		return err;
	}

	err = vgpu_pm_init(dev);
	if (err) {
		dev_err(dev, "pm init failed");
		return err;
	}

	priv->intr_handler = kthread_run(vgpu_intr_thread, gk20a, "gk20a");
	if (IS_ERR(priv->intr_handler))
		return -ENOMEM;

	gk20a_debug_init(dev, "gpu.0");

	/* Set DMA parameters to allow larger sgt lists */
	dev->dma_parms = &gk20a->dma_parms;
	dma_set_max_seg_size(dev, UINT_MAX);

	gk20a->gr_idle_timeout_default =
			CONFIG_GK20A_DEFAULT_TIMEOUT;
	gk20a->timeouts_enabled = true;

	vgpu_create_sysfs(dev);
	gk20a_init_gr(gk20a);

	kref_init(&gk20a->refcount);

	return 0;
}

int vgpu_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gk20a *g = get_gk20a(dev);
	gk20a_dbg_fn("");

	vgpu_pm_qos_remove(dev);
	if (g->remove_support)
		g->remove_support(g);

	vgpu_comm_deinit();
	gk20a_sched_ctrl_cleanup(g);
	gk20a_user_deinit(dev, &nvgpu_class);
	vgpu_remove_sysfs(dev);
	gk20a_get_platform(dev)->g = NULL;
	gk20a_put(g);

	return 0;
}
