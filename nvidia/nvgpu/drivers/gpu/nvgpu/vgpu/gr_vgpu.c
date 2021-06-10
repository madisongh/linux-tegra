/*
 * Virtualized GPU Graphics
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

#include "vgpu/vgpu.h"
#include "gk20a/dbg_gpu_gk20a.h"

#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

static void vgpu_gr_detect_sm_arch(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	g->gpu_characteristics.sm_arch_sm_version =
			priv->constants.sm_arch_sm_version;
	g->gpu_characteristics.sm_arch_spa_version =
			priv->constants.sm_arch_spa_version;
	g->gpu_characteristics.sm_arch_warp_count =
			priv->constants.sm_arch_warp_count;
}

static int vgpu_gr_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_CTX;
	msg.handle = vgpu_get_handle(c->g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

static int vgpu_gr_commit_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c, bool patch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_GLOBAL_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

/* load saved fresh copy of gloden image into channel gr_ctx */
static int vgpu_gr_load_golden_ctx_image(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_LOAD_GR_GOLDEN_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

int vgpu_gr_init_ctx_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	g->gr.ctx_vars.golden_image_size = priv->constants.golden_ctx_size;
	g->gr.ctx_vars.zcull_ctxsw_image_size = priv->constants.zcull_ctx_size;
	g->gr.ctx_vars.pm_ctxsw_image_size = priv->constants.hwpm_ctx_size;
	if (!g->gr.ctx_vars.golden_image_size ||
		!g->gr.ctx_vars.zcull_ctxsw_image_size ||
		!g->gr.ctx_vars.pm_ctxsw_image_size)
		return -ENXIO;

	gr->ctx_vars.buffer_size = g->gr.ctx_vars.golden_image_size;
	g->gr.ctx_vars.priv_access_map_size = 512 * 1024;
	return 0;
}

static int vgpu_gr_alloc_global_ctx_buffers(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int attr_buffer_size;

	u32 cb_buffer_size = gr->bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v();

	u32 pagepool_buffer_size = g->ops.gr.pagepool_default_size(g) *
		gr_scc_pagepool_total_pages_byte_granularity_v();

	gk20a_dbg_fn("");

	attr_buffer_size = g->ops.gr.calc_global_ctx_buffer_size(g);

	gk20a_dbg_info("cb_buffer_size : %d", cb_buffer_size);
	gr->global_ctx_buffer[CIRCULAR].mem.size = cb_buffer_size;

	gk20a_dbg_info("pagepool_buffer_size : %d", pagepool_buffer_size);
	gr->global_ctx_buffer[PAGEPOOL].mem.size = pagepool_buffer_size;

	gk20a_dbg_info("attr_buffer_size : %d", attr_buffer_size);
	gr->global_ctx_buffer[ATTRIBUTE].mem.size = attr_buffer_size;

	gk20a_dbg_info("priv access map size : %d",
		gr->ctx_vars.priv_access_map_size);
	gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size =
		gr->ctx_vars.priv_access_map_size;

	return 0;
}

static int vgpu_gr_map_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	u64 *g_bfr_va = c->ch_ctx.global_ctx_buffer_va;
	u64 *g_bfr_size = c->ch_ctx.global_ctx_buffer_size;
	struct gr_gk20a *gr = &g->gr;
	u64 gpu_va;
	u32 i;
	int err;

	gk20a_dbg_fn("");

	/* FIXME: add VPR support */

	/* Circular Buffer */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[CIRCULAR].mem.size,
			gmmu_page_size_kernel);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[CIRCULAR_VA] = gpu_va;
	g_bfr_size[CIRCULAR_VA] = gr->global_ctx_buffer[CIRCULAR].mem.size;

	/* Attribute Buffer */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[ATTRIBUTE].mem.size,
			gmmu_page_size_kernel);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[ATTRIBUTE_VA] = gpu_va;
	g_bfr_size[ATTRIBUTE_VA] = gr->global_ctx_buffer[ATTRIBUTE].mem.size;

	/* Page Pool */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PAGEPOOL].mem.size,
			gmmu_page_size_kernel);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PAGEPOOL_VA] = gpu_va;
	g_bfr_size[PAGEPOOL_VA] = gr->global_ctx_buffer[PAGEPOOL].mem.size;

	/* Priv register Access Map */
	gpu_va = gk20a_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size,
			gmmu_page_size_kernel);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PRIV_ACCESS_MAP_VA] = gpu_va;
	g_bfr_size[PRIV_ACCESS_MAP_VA] =
		gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_MAP_GR_GLOBAL_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->cb_va = g_bfr_va[CIRCULAR_VA];
	p->attr_va = g_bfr_va[ATTRIBUTE_VA];
	p->page_pool_va = g_bfr_va[PAGEPOOL_VA];
	p->priv_access_map_va = g_bfr_va[PRIV_ACCESS_MAP_VA];
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		goto clean_up;

	c->ch_ctx.global_ctx_buffer_mapped = true;
	return 0;

 clean_up:
	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_va[i]) {
			gk20a_vm_free_va(ch_vm, g_bfr_va[i],
					g_bfr_size[i], gmmu_page_size_kernel);
			g_bfr_va[i] = 0;
		}
	}
	return -ENOMEM;
}

static void vgpu_gr_unmap_global_ctx_buffers(struct channel_gk20a *c)
{
	struct vm_gk20a *ch_vm = c->vm;
	u64 *g_bfr_va = c->ch_ctx.global_ctx_buffer_va;
	u64 *g_bfr_size = c->ch_ctx.global_ctx_buffer_size;
	u32 i;

	gk20a_dbg_fn("");

	if (c->ch_ctx.global_ctx_buffer_mapped) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_UNMAP_GR_GLOBAL_CTX;
		msg.handle = vgpu_get_handle(c->g);
		p->handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);
	}

	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_va[i]) {
			gk20a_vm_free_va(ch_vm, g_bfr_va[i], g_bfr_size[i],
					gmmu_page_size_kernel);
			g_bfr_va[i] = 0;
			g_bfr_size[i] = 0;
		}
	}
	c->ch_ctx.global_ctx_buffer_mapped = false;
}

int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
			struct gr_ctx_desc **__gr_ctx,
			struct vm_gk20a *vm,
			u32 class,
			u32 flags)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	struct gr_gk20a *gr = &g->gr;
	struct gr_ctx_desc *gr_ctx;
	int err;

	gk20a_dbg_fn("");

	if (gr->ctx_vars.buffer_size == 0)
		return 0;

	/* alloc channel gr ctx buffer */
	gr->ctx_vars.buffer_size = gr->ctx_vars.golden_image_size;
	gr->ctx_vars.buffer_total_size = gr->ctx_vars.golden_image_size;

	gr_ctx = kzalloc(sizeof(*gr_ctx), GFP_KERNEL);
	if (!gr_ctx)
		return -ENOMEM;

	gr_ctx->mem.size = gr->ctx_vars.buffer_total_size;
	gr_ctx->mem.gpu_va = gk20a_vm_alloc_va(vm,
						gr_ctx->mem.size,
						gmmu_page_size_kernel);

	if (!gr_ctx->mem.gpu_va) {
		kfree(gr_ctx);
		return -ENOMEM;
	}

	msg.cmd = TEGRA_VGPU_CMD_GR_CTX_ALLOC;
	msg.handle = vgpu_get_handle(g);
	p->as_handle = vm->handle;
	p->gr_ctx_va = gr_ctx->mem.gpu_va;
	p->class_num = class;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;

	if (unlikely(err)) {
		gk20a_err(dev_from_gk20a(g), "fail to alloc gr_ctx");
		gk20a_vm_free_va(vm, gr_ctx->mem.gpu_va,
				 gr_ctx->mem.size, gmmu_page_size_kernel);
		kfree(gr_ctx);
	} else {
		gr_ctx->virt_ctx = p->gr_ctx_handle;
		*__gr_ctx = gr_ctx;
	}

	return err;
}

void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			struct gr_ctx_desc *gr_ctx)
{
	gk20a_dbg_fn("");

	if (gr_ctx && gr_ctx->mem.gpu_va) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_GR_CTX_FREE;
		msg.handle = vgpu_get_handle(g);
		p->gr_ctx_handle = gr_ctx->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);

		gk20a_vm_free_va(vm, gr_ctx->mem.gpu_va, gr_ctx->mem.size,
				gmmu_page_size_kernel);
		kfree(gr_ctx);
	}
}

static void vgpu_gr_free_channel_gr_ctx(struct channel_gk20a *c)
{
	gk20a_dbg_fn("");

	c->g->ops.gr.free_gr_ctx(c->g, c->vm, c->ch_ctx.gr_ctx);
	c->ch_ctx.gr_ctx = NULL;
}

static int vgpu_gr_alloc_channel_patch_ctx(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct patch_desc *patch_ctx = &c->ch_ctx.patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	gk20a_dbg_fn("");

	patch_ctx->mem.size = 128 * sizeof(u32);
	patch_ctx->mem.gpu_va = gk20a_vm_alloc_va(ch_vm,
						patch_ctx->mem.size,
						gmmu_page_size_kernel);
	if (!patch_ctx->mem.gpu_va)
		return -ENOMEM;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_PATCH_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->patch_ctx_va = patch_ctx->mem.gpu_va;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		gk20a_vm_free_va(ch_vm, patch_ctx->mem.gpu_va,
				 patch_ctx->mem.size, gmmu_page_size_kernel);
		err = -ENOMEM;
	}

	return err;
}

static void vgpu_gr_free_channel_patch_ctx(struct channel_gk20a *c)
{
	struct patch_desc *patch_ctx = &c->ch_ctx.patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;

	gk20a_dbg_fn("");

	if (patch_ctx->mem.gpu_va) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_GR_PATCH_CTX;
		msg.handle = vgpu_get_handle(c->g);
		p->handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);

		gk20a_vm_free_va(ch_vm, patch_ctx->mem.gpu_va,
				 patch_ctx->mem.size, gmmu_page_size_kernel);
		patch_ctx->mem.gpu_va = 0;
	}
}

static void vgpu_gr_free_channel_pm_ctx(struct channel_gk20a *c)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_free_hwpm_ctx *p = &msg.params.free_hwpm_ctx;
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	struct pm_ctx_desc *pm_ctx = &ch_ctx->pm_ctx;
	int err;

	gk20a_dbg_fn("");

	/* check if hwpm was ever initialized. If not, nothing to do */
	if (pm_ctx->mem.gpu_va == 0)
		return;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_HWPM_CTX;
	msg.handle = vgpu_get_handle(c->g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	gk20a_vm_free_va(c->vm, pm_ctx->mem.gpu_va, pm_ctx->mem.size,
			gmmu_page_size_kernel);
	pm_ctx->mem.gpu_va = 0;
}

static void vgpu_gr_free_channel_ctx(struct channel_gk20a *c)
{
	gk20a_dbg_fn("");

	vgpu_gr_unmap_global_ctx_buffers(c);
	vgpu_gr_free_channel_patch_ctx(c);
	vgpu_gr_free_channel_pm_ctx(c);
	if (!gk20a_is_channel_marked_as_tsg(c))
		vgpu_gr_free_channel_gr_ctx(c);

	/* zcull_ctx, pm_ctx */

	memset(&c->ch_ctx, 0, sizeof(struct channel_ctx_gk20a));

	c->first_init = false;
}

static int vgpu_gr_ch_bind_gr_ctx(struct channel_gk20a *c)
{
	struct gr_ctx_desc *gr_ctx = c->ch_ctx.gr_ctx;
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_channel_bind_gr_ctx_params *p =
				&msg.params.ch_bind_gr_ctx;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTX;
	msg.handle = vgpu_get_handle(c->g);
	p->ch_handle = c->virt_ctx;
	p->gr_ctx_handle = gr_ctx->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

static int vgpu_gr_tsg_bind_gr_ctx(struct tsg_gk20a *tsg)
{
	struct gr_ctx_desc *gr_ctx = tsg->tsg_gr_ctx;
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_tsg_bind_gr_ctx_params *p =
					&msg.params.tsg_bind_gr_ctx;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_BIND_GR_CTX;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->gr_ctx_handle = gr_ctx->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

static int vgpu_gr_alloc_obj_ctx(struct channel_gk20a  *c,
				struct nvgpu_alloc_obj_ctx_args *args)
{
	struct gk20a *g = c->g;
	struct fifo_gk20a *f = &g->fifo;
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	struct tsg_gk20a *tsg = NULL;
	int err = 0;

	gk20a_dbg_fn("");

	/* an address space needs to have been bound at this point.*/
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(dev_from_gk20a(g),
			   "not bound to address space at time"
			   " of grctx allocation");
		return -EINVAL;
	}

	if (!g->ops.gr.is_valid_class(g, args->class_num)) {
		gk20a_err(dev_from_gk20a(g),
			   "invalid obj class 0x%x", args->class_num);
		err = -EINVAL;
		goto out;
	}
	c->obj_class = args->class_num;

	if (gk20a_is_channel_marked_as_tsg(c))
		tsg = &f->tsg[c->tsgid];

	if (!tsg) {
		/* allocate gr ctx buffer */
		if (!ch_ctx->gr_ctx) {
			err = g->ops.gr.alloc_gr_ctx(g, &c->ch_ctx.gr_ctx,
						c->vm,
						args->class_num,
						args->flags);
			if (!err)
				err = vgpu_gr_ch_bind_gr_ctx(c);
			if (err) {
				gk20a_err(dev_from_gk20a(g),
					"fail to allocate gr ctx buffer");
				goto out;
			}
		} else {
			/*TBD: needs to be more subtle about which is
			 * being allocated as some are allowed to be
			 * allocated along same channel */
			gk20a_err(dev_from_gk20a(g),
				"too many classes alloc'd on same channel");
			err = -EINVAL;
			goto out;
		}
	} else {
		if (!tsg->tsg_gr_ctx) {
			tsg->vm = c->vm;
			gk20a_vm_get(tsg->vm);
			err = g->ops.gr.alloc_gr_ctx(g, &tsg->tsg_gr_ctx,
						c->vm,
						args->class_num,
						args->flags);
			if (!err)
				err = vgpu_gr_tsg_bind_gr_ctx(tsg);
			if (err) {
				gk20a_err(dev_from_gk20a(g),
					"fail to allocate TSG gr ctx buffer, err=%d", err);
				gk20a_vm_put(tsg->vm);
				tsg->vm = NULL;
				goto out;
			}
		}

		ch_ctx->gr_ctx = tsg->tsg_gr_ctx;
		err = vgpu_gr_ch_bind_gr_ctx(c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to bind gr ctx buffer");
			goto out;
		}
	}

	/* commit gr ctx buffer */
	err = vgpu_gr_commit_inst(c, ch_ctx->gr_ctx->mem.gpu_va);
	if (err) {
		gk20a_err(dev_from_gk20a(g),
			"fail to commit gr ctx buffer");
		goto out;
	}

	/* allocate patch buffer */
	if (ch_ctx->patch_ctx.mem.pages == NULL) {
		err = vgpu_gr_alloc_channel_patch_ctx(g, c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to allocate patch buffer");
			goto out;
		}
	}

	/* map global buffer to channel gpu_va and commit */
	if (!ch_ctx->global_ctx_buffer_mapped) {
		err = vgpu_gr_map_global_ctx_buffers(g, c);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to map global ctx buffer");
			goto out;
		}
		gr_gk20a_elpg_protected_call(g,
				vgpu_gr_commit_global_ctx_buffers(g, c, true));
	}

	/* load golden image */
	if (!c->first_init) {
		err = gr_gk20a_elpg_protected_call(g,
				vgpu_gr_load_golden_ctx_image(g, c));
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"fail to load golden ctx image");
			goto out;
		}
		c->first_init = true;
	}

	gk20a_dbg_fn("done");
	return 0;
out:
	/* 1. gr_ctx, patch_ctx and global ctx buffer mapping
	   can be reused so no need to release them.
	   2. golden image load is a one time thing so if
	   they pass, no need to undo. */
	gk20a_err(dev_from_gk20a(g), "fail");
	return err;
}

static int vgpu_gr_init_gr_config(struct gk20a *g, struct gr_gk20a *gr)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 gpc_index;

	gk20a_dbg_fn("");

	gr->max_gpc_count = priv->constants.max_gpc_count;
	gr->gpc_count = priv->constants.gpc_count;
	gr->max_tpc_per_gpc_count = priv->constants.max_tpc_per_gpc_count;

	gr->max_tpc_count = gr->max_gpc_count * gr->max_tpc_per_gpc_count;

	gr->gpc_tpc_count = kzalloc(gr->gpc_count * sizeof(u32), GFP_KERNEL);
	if (!gr->gpc_tpc_count)
		goto cleanup;

	gr->gpc_tpc_mask = kzalloc(gr->gpc_count * sizeof(u32), GFP_KERNEL);
	if (!gr->gpc_tpc_mask)
		goto cleanup;

	gr->sm_to_cluster = kzalloc(gr->gpc_count * gr->max_tpc_per_gpc_count *
				sizeof(struct sm_info), GFP_KERNEL);
	if (!gr->sm_to_cluster)
		goto cleanup;

	gr->tpc_count = 0;
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		gr->gpc_tpc_count[gpc_index] =
			priv->constants.gpc_tpc_count[gpc_index];

		gr->tpc_count += gr->gpc_tpc_count[gpc_index];

		if (g->ops.gr.get_gpc_tpc_mask)
			gr->gpc_tpc_mask[gpc_index] =
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index);
	}

	g->ops.gr.bundle_cb_defaults(g);
	g->ops.gr.cb_size_default(g);
	g->ops.gr.calc_global_ctx_buffer_size(g);
	g->ops.gr.init_fs_state(g);
	return 0;
cleanup:
	gk20a_err(dev_from_gk20a(g), "%s: out of memory", __func__);

	kfree(gr->gpc_tpc_count);
	gr->gpc_tpc_count = NULL;

	kfree(gr->gpc_tpc_mask);
	gr->gpc_tpc_mask = NULL;

	return -ENOMEM;
}

static int vgpu_gr_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_bind_params *p = &msg.params.zcull_bind;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_ZCULL;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->zcull_va = zcull_va;
	p->mode = mode;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

static int vgpu_gr_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_info_params *p = &msg.params.zcull_info;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_GET_ZCULL_INFO;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	zcull_params->width_align_pixels = p->width_align_pixels;
	zcull_params->height_align_pixels = p->height_align_pixels;
	zcull_params->pixel_squares_by_aliquots = p->pixel_squares_by_aliquots;
	zcull_params->aliquot_total = p->aliquot_total;
	zcull_params->region_byte_multiplier = p->region_byte_multiplier;
	zcull_params->region_header_size = p->region_header_size;
	zcull_params->subregion_header_size = p->subregion_header_size;
	zcull_params->subregion_width_align_pixels =
		p->subregion_width_align_pixels;
	zcull_params->subregion_height_align_pixels =
		p->subregion_height_align_pixels;
	zcull_params->subregion_count = p->subregion_count;

	return 0;
}

static u32 vgpu_gr_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	return priv->constants.gpc_tpc_mask[gpc_index];
}

static u32 vgpu_gr_get_max_fbps_count(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.num_fbps;
}

static u32 vgpu_gr_get_fbp_en_mask(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.fbp_en_mask;
}

static u32 vgpu_gr_get_max_ltc_per_fbp(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.ltc_per_fbp;
}

static u32 vgpu_gr_get_max_lts_per_ltc(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.max_lts_per_ltc;
}

static u32 *vgpu_gr_rop_l2_en_mask(struct gk20a *g)
{
	/* no one use it yet */
	return NULL;
}

static int vgpu_gr_add_zbc(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *zbc_val)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_zbc_set_table_params *p = &msg.params.zbc_set_table;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_ZBC_SET_TABLE;
	msg.handle = vgpu_get_handle(g);

	p->type = zbc_val->type;
	p->format = zbc_val->format;
	switch (p->type) {
	case GK20A_ZBC_TYPE_COLOR:
		memcpy(p->color_ds, zbc_val->color_ds, sizeof(p->color_ds));
		memcpy(p->color_l2, zbc_val->color_l2, sizeof(p->color_l2));
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		p->depth = zbc_val->depth;
		break;
	default:
		return -EINVAL;
	}

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

static int vgpu_gr_query_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_zbc_query_table_params *p =
					&msg.params.zbc_query_table;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_ZBC_QUERY_TABLE;
	msg.handle = vgpu_get_handle(g);

	p->type = query_params->type;
	p->index_size = query_params->index_size;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	switch (query_params->type) {
	case GK20A_ZBC_TYPE_COLOR:
		memcpy(query_params->color_ds, p->color_ds,
				sizeof(query_params->color_ds));
		memcpy(query_params->color_l2, p->color_l2,
				sizeof(query_params->color_l2));
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		query_params->depth = p->depth;
		break;
	case GK20A_ZBC_TYPE_INVALID:
		query_params->index_size = p->index_size;
		break;
	default:
		return -EINVAL;
	}
	query_params->ref_cnt = p->ref_cnt;
	query_params->format = p->format;

	return 0;
}

static void vgpu_remove_gr_support(struct gr_gk20a *gr)
{
	gk20a_dbg_fn("");

	gk20a_comptag_allocator_destroy(&gr->comp_tags);

	kfree(gr->sm_error_states);
	gr->sm_error_states = NULL;

	kfree(gr->gpc_tpc_mask);
	gr->gpc_tpc_mask = NULL;

	kfree(gr->sm_to_cluster);
	gr->sm_to_cluster = NULL;

	kfree(gr->gpc_tpc_count);
	gr->gpc_tpc_count = NULL;
}

static int vgpu_gr_init_gr_setup_sw(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int err;

	gk20a_dbg_fn("");

	if (gr->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	gr->g = g;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	nvgpu_mutex_init(&g->gr.cs_lock);
#endif

	err = vgpu_gr_init_gr_config(g, gr);
	if (err)
		goto clean_up;

	err = g->ops.gr.init_ctx_state(g);
	if (err)
		goto clean_up;

	err = g->ops.ltc.init_comptags(g, gr);
	if (err)
		goto clean_up;

	err = vgpu_gr_alloc_global_ctx_buffers(g);
	if (err)
		goto clean_up;

	nvgpu_mutex_init(&gr->ctx_mutex);

	gr->sm_error_states = kzalloc(
			sizeof(struct nvgpu_dbg_gpu_sm_error_state_record) *
			gr->no_of_sm, GFP_KERNEL);
	if (!gr->sm_error_states) {
		err = -ENOMEM;
		goto clean_up;
	}

	gr->remove_support = vgpu_remove_gr_support;
	gr->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_err(dev_from_gk20a(g), "fail");
	vgpu_remove_gr_support(gr);
	return err;
}

int vgpu_init_gr_support(struct gk20a *g)
{
	gk20a_dbg_fn("");

	return vgpu_gr_init_gr_setup_sw(g);
}

int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = gk20a_channel_get(&f->channel[info->chid]);

	gk20a_dbg_fn("");
	if (!ch)
		return 0;

	if (info->type != TEGRA_VGPU_GR_INTR_NOTIFY &&
		info->type != TEGRA_VGPU_GR_INTR_SEMAPHORE)
		gk20a_err(dev_from_gk20a(g), "gr intr (%d) on ch %u",
			info->type, info->chid);

	switch (info->type) {
	case TEGRA_VGPU_GR_INTR_NOTIFY:
		wake_up(&ch->notifier_wq);
		break;
	case TEGRA_VGPU_GR_INTR_SEMAPHORE:
		wake_up_interruptible_all(&ch->semaphore_wq);
		break;
	case TEGRA_VGPU_GR_INTR_SEMAPHORE_TIMEOUT:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_SEMAPHORE_TIMEOUT);
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_NOTIFY:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ILLEGAL_NOTIFY);
	case TEGRA_VGPU_GR_INTR_ILLEGAL_METHOD:
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_CLASS:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FECS_ERROR:
		break;
	case TEGRA_VGPU_GR_INTR_CLASS_ERROR:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FIRMWARE_METHOD:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_EXCEPTION:
		gk20a_set_error_notifier(ch,
				NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_SM_EXCEPTION:
		gk20a_dbg_gpu_post_events(ch);
		break;
	default:
		WARN_ON(1);
		break;
	}

	gk20a_channel_put(ch);
	return 0;
}

int vgpu_gr_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_gr_nonstall_intr_info *info)
{
	gk20a_dbg_fn("");

	switch (info->type) {
	case TEGRA_VGPU_GR_NONSTALL_INTR_SEMAPHORE:
		gk20a_channel_semaphore_wakeup(g, true);
		break;
	default:
		WARN_ON(1);
		break;
	}

	return 0;
}

static int vgpu_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_sm_debug_mode *p = &msg.params.sm_debug_mode;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_SET_SM_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->sms = sms;
	p->enable = (u32)enable;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

static int vgpu_gr_update_smpc_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_set_ctxsw_mode *p = &msg.params.set_ctxsw_mode;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_SMPC_CTXSW_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;

	if (enable)
		p->mode = TEGRA_VGPU_CTXSW_MODE_CTXSW;
	else
		p->mode = TEGRA_VGPU_CTXSW_MODE_NO_CTXSW;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

static int vgpu_gr_update_hwpm_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, bool enable)
{
	struct channel_ctx_gk20a *ch_ctx = &ch->ch_ctx;
	struct pm_ctx_desc *pm_ctx = &ch_ctx->pm_ctx;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_set_ctxsw_mode *p = &msg.params.set_ctxsw_mode;
	int err;

	gk20a_dbg_fn("");

	if (enable) {
		p->mode = TEGRA_VGPU_CTXSW_MODE_CTXSW;

		/* Allocate buffer if necessary */
		if (pm_ctx->mem.gpu_va == 0) {
			pm_ctx->mem.gpu_va = gk20a_vm_alloc_va(ch->vm,
					g->gr.ctx_vars.pm_ctxsw_image_size,
					gmmu_page_size_kernel);

			if (!pm_ctx->mem.gpu_va)
				return -ENOMEM;
			pm_ctx->mem.size = g->gr.ctx_vars.pm_ctxsw_image_size;
		}
	} else
		p->mode = TEGRA_VGPU_CTXSW_MODE_NO_CTXSW;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_HWPM_CTXSW_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->gpu_va = pm_ctx->mem.gpu_va;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

static int vgpu_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id)
{
	struct gr_gk20a *gr = &g->gr;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_clear_sm_error_state *p =
			&msg.params.clear_sm_error_state;
	int err;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	msg.cmd = TEGRA_VGPU_CMD_CLEAR_SM_ERROR_STATE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->sm_id = sm_id;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	memset(&gr->sm_error_states[sm_id], 0, sizeof(*gr->sm_error_states));
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err ? err : msg.ret;


	return 0;
}

static int vgpu_gr_suspend_resume_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd, u32 cmd)
{
	struct dbg_session_channel_data *ch_data;
	struct tegra_vgpu_cmd_msg *msg;
	struct tegra_vgpu_suspend_resume_contexts *p;
	size_t size_out = offsetof(struct tegra_vgpu_cmd_msg,
			params.suspend_contexts.chids);
	size_t size_in;
	size_t n;
	int channel_fd = -1;
	int err = 0;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);

	n = 0;
	list_for_each_entry(ch_data, &dbg_s->ch_list, ch_entry)
		n++;

	size_in = size_out + n * sizeof(u16);

	msg = kmalloc(size_in, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->cmd = cmd;
	msg->handle = vgpu_get_handle(g);
	p = &msg->params.suspend_contexts;
	p->num_channels = n;
	n = 0;
	list_for_each_entry(ch_data, &dbg_s->ch_list, ch_entry) {
		p->chids[n++] = (u16)ch_data->chid;
	}

	err = vgpu_comm_sendrecv(msg, size_in, size_out);
	if (err || msg->ret) {
		err = -ENOMEM;
		goto fail;
	}

	if (p->resident_chid != (u16)~0) {
		list_for_each_entry(ch_data, &dbg_s->ch_list, ch_entry) {
			if (ch_data->chid == p->resident_chid) {
				channel_fd = ch_data->channel_fd;
				break;
			}
		}
	}

fail:
	nvgpu_mutex_release(&dbg_s->ch_list_lock);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	*ctx_resident_ch_fd = channel_fd;
	kfree(msg);

	return err;
}

static int vgpu_gr_suspend_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd)
{
	return vgpu_gr_suspend_resume_contexts(g, dbg_s,
			ctx_resident_ch_fd, TEGRA_VGPU_CMD_SUSPEND_CONTEXTS);
}

static int vgpu_gr_resume_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd)
{
	return vgpu_gr_suspend_resume_contexts(g, dbg_s,
			ctx_resident_ch_fd, TEGRA_VGPU_CMD_RESUME_CONTEXTS);
}

void vgpu_gr_handle_sm_esr_event(struct gk20a *g,
			struct tegra_vgpu_sm_esr_info *info)
{
	struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_states;

	if (info->sm_id >= g->gr.no_of_sm) {
		gk20a_err(g->dev, "invalid smd_id %d / %d",
			info->sm_id, g->gr.no_of_sm);
		return;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	sm_error_states = &g->gr.sm_error_states[info->sm_id];

	sm_error_states->hww_global_esr = info->hww_global_esr;
	sm_error_states->hww_warp_esr = info->hww_warp_esr;
	sm_error_states->hww_warp_esr_pc = info->hww_warp_esr_pc;
	sm_error_states->hww_global_esr_report_mask =
				info->hww_global_esr_report_mask;
	sm_error_states->hww_warp_esr_report_mask =
				info->hww_warp_esr_report_mask;

	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

void vgpu_init_gr_ops(struct gpu_ops *gops)
{
	gops->gr.detect_sm_arch = vgpu_gr_detect_sm_arch;
	gops->gr.free_channel_ctx = vgpu_gr_free_channel_ctx;
	gops->gr.alloc_obj_ctx = vgpu_gr_alloc_obj_ctx;
	gops->gr.alloc_gr_ctx = vgpu_gr_alloc_gr_ctx;
	gops->gr.free_gr_ctx = vgpu_gr_free_gr_ctx;
	gops->gr.bind_ctxsw_zcull = vgpu_gr_bind_ctxsw_zcull;
	gops->gr.get_zcull_info = vgpu_gr_get_zcull_info;
	gops->gr.get_gpc_tpc_mask = vgpu_gr_get_gpc_tpc_mask;
	gops->gr.get_max_fbps_count = vgpu_gr_get_max_fbps_count;
	gops->gr.get_fbp_en_mask = vgpu_gr_get_fbp_en_mask;
	gops->gr.get_max_ltc_per_fbp = vgpu_gr_get_max_ltc_per_fbp;
	gops->gr.get_max_lts_per_ltc = vgpu_gr_get_max_lts_per_ltc;
	gops->gr.get_rop_l2_en_mask = vgpu_gr_rop_l2_en_mask;
	gops->gr.zbc_set_table = vgpu_gr_add_zbc;
	gops->gr.zbc_query_table = vgpu_gr_query_zbc;
	gops->gr.init_ctx_state = vgpu_gr_init_ctx_state;
	gops->gr.set_sm_debug_mode = vgpu_gr_set_sm_debug_mode;
	gops->gr.update_smpc_ctxsw_mode = vgpu_gr_update_smpc_ctxsw_mode;
	gops->gr.update_hwpm_ctxsw_mode = vgpu_gr_update_hwpm_ctxsw_mode;
	gops->gr.clear_sm_error_state = vgpu_gr_clear_sm_error_state;
	gops->gr.suspend_contexts = vgpu_gr_suspend_contexts;
	gops->gr.resume_contexts = vgpu_gr_resume_contexts;
	gops->gr.dump_gr_regs = NULL;
	gops->gr.set_boosted_ctx = NULL;
	gops->gr.update_boosted_ctx = NULL;
}
