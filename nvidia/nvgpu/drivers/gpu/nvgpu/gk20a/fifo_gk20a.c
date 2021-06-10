/*
 * GK20A Graphics FIFO (gr host)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <trace/events/gk20a.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>
#include <linux/sort.h>

#include <nvgpu/timers.h>
#include <nvgpu/semaphore.h>

#include "gk20a.h"
#include "debug_gk20a.h"
#include "ctxsw_trace_gk20a.h"

#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

#define FECS_METHOD_WFI_RESTORE 0x80000
#define FECS_MAILBOX_0_ACK_RESTORE 0x4

static int gk20a_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					    u32 hw_chid, bool add,
					    bool wait_for_finish);
static u32 gk20a_fifo_engines_on_id(struct gk20a *g, u32 id, bool is_tsg);
static int gk20a_fifo_is_preempt_pending(struct gk20a *g);

#ifdef CONFIG_DEBUG_FS
static void __gk20a_fifo_profile_free(struct kref *ref);
#endif

u32 gk20a_fifo_get_engine_ids(struct gk20a *g,
		u32 engine_id[], u32 engine_id_sz,
		u32 engine_enum)
{
	struct fifo_gk20a *f = NULL;
	u32 instance_cnt = 0;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	struct fifo_engine_info_gk20a *info = NULL;

	if (g && engine_id_sz && (engine_enum < ENGINE_INVAL_GK20A)) {
		f = &g->fifo;
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			active_engine_id = f->active_engines_list[engine_id_idx];
			info = &f->engine_info[active_engine_id];

			if (info->engine_enum == engine_enum) {
				if (instance_cnt < engine_id_sz) {
					engine_id[instance_cnt] = active_engine_id;
					++instance_cnt;
				} else {
					gk20a_dbg_info("warning engine_id table sz is small %d",
							engine_id_sz);
				}
			}
		}
	}
	return instance_cnt;
}

struct fifo_engine_info_gk20a *gk20a_fifo_get_engine_info(struct gk20a *g, u32 engine_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *info = NULL;

	if (!g)
		return info;

	f = &g->fifo;

	if (engine_id < f->max_engines) {
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			if (engine_id == f->active_engines_list[engine_id_idx]) {
				info = &f->engine_info[engine_id];
				break;
			}
		}
	}

	if (!info)
		gk20a_err(g->dev, "engine_id is not in active list/invalid %d", engine_id);

	return info;
}

bool gk20a_fifo_is_valid_engine_id(struct gk20a *g, u32 engine_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	bool valid = false;

	if (!g)
		return valid;

	f = &g->fifo;

	if (engine_id < f->max_engines) {
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			if (engine_id == f->active_engines_list[engine_id_idx]) {
				valid = true;
				break;
			}
		}
	}

	if (!valid)
		gk20a_err(g->dev, "engine_id is not in active list/invalid %d", engine_id);

	return valid;
}

u32 gk20a_fifo_get_gr_engine_id(struct gk20a *g)
{
	u32 gr_engine_cnt = 0;
	u32 gr_engine_id = FIFO_INVAL_ENGINE_ID;

	/* Consider 1st available GR engine */
	gr_engine_cnt = gk20a_fifo_get_engine_ids(g, &gr_engine_id,
			1, ENGINE_GR_GK20A);

	if (!gr_engine_cnt) {
		gk20a_err(dev_from_gk20a(g), "No GR engine available on this device!\n");
	}

	return gr_engine_id;
}

u32 gk20a_fifo_get_all_ce_engine_reset_mask(struct gk20a *g)
{
	u32 reset_mask = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *engine_info;
	u32 active_engine_id = 0;

	if (!g)
		return reset_mask;

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = &f->engine_info[active_engine_id];
		engine_enum = engine_info->engine_enum;

		if ((engine_enum == ENGINE_GRCE_GK20A) ||
			(engine_enum == ENGINE_ASYNC_CE_GK20A))
				reset_mask |= engine_info->reset_mask;
	}

	return reset_mask;
}

u32 gk20a_fifo_get_fast_ce_runlist_id(struct gk20a *g)
{
	u32 ce_runlist_id = gk20a_fifo_get_gr_runlist_id(g);
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *engine_info;
	u32 active_engine_id = 0;

	if (!g)
		return ce_runlist_id;

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = &f->engine_info[active_engine_id];
		engine_enum = engine_info->engine_enum;

		/* selecet last available ASYNC_CE if available */
		if (engine_enum == ENGINE_ASYNC_CE_GK20A)
			ce_runlist_id = engine_info->runlist_id;
	}

	return ce_runlist_id;
}

u32 gk20a_fifo_get_gr_runlist_id(struct gk20a *g)
{
	u32 gr_engine_cnt = 0;
	u32 gr_engine_id = FIFO_INVAL_ENGINE_ID;
	struct fifo_engine_info_gk20a *engine_info;
	u32 gr_runlist_id = ~0;

	/* Consider 1st available GR engine */
	gr_engine_cnt = gk20a_fifo_get_engine_ids(g, &gr_engine_id,
			1, ENGINE_GR_GK20A);

	if (!gr_engine_cnt) {
		gk20a_err(dev_from_gk20a(g),
			"No GR engine available on this device!");
		goto end;
	}

	engine_info = gk20a_fifo_get_engine_info(g, gr_engine_id);

	if (engine_info) {
		gr_runlist_id = engine_info->runlist_id;
	} else {
		gk20a_err(g->dev,
			"gr_engine_id is not in active list/invalid %d", gr_engine_id);
	}

end:
	return gr_runlist_id;
}

bool gk20a_fifo_is_valid_runlist_id(struct gk20a *g, u32 runlist_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;

	if (!g)
		return false;

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = gk20a_fifo_get_engine_info(g, active_engine_id);
		if (engine_info && (engine_info->runlist_id == runlist_id)) {
			return true;
		}
	}

	return false;
}

/*
 * Link engine IDs to MMU IDs and vice versa.
 */

static inline u32 gk20a_engine_id_to_mmu_id(struct gk20a *g, u32 engine_id)
{
	u32 fault_id = FIFO_INVAL_ENGINE_ID;
	struct fifo_engine_info_gk20a *engine_info;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		fault_id = engine_info->fault_id;
	} else {
		gk20a_err(g->dev, "engine_id is not in active list/invalid %d", engine_id);
	}
	return fault_id;
}

static inline u32 gk20a_mmu_id_to_engine_id(struct gk20a *g, u32 fault_id)
{
	u32 engine_id;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_gk20a *f = &g->fifo;

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if (engine_info->fault_id == fault_id)
			break;
		active_engine_id = FIFO_INVAL_ENGINE_ID;
	}
	return active_engine_id;
}

int gk20a_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id)
{
	int ret = ENGINE_INVAL_GK20A;

	gk20a_dbg_info("engine type %d", engine_type);
	if (engine_type == top_device_info_type_enum_graphics_v())
		ret = ENGINE_GR_GK20A;
	else if ((engine_type >= top_device_info_type_enum_copy0_v()) &&
		(engine_type <= top_device_info_type_enum_copy2_v())) {
		/* Lets consider all the CE engine have separate runlist at this point
		 * We can identify the ENGINE_GRCE_GK20A type CE using runlist_id
		 * comparsion logic with GR runlist_id in init_engine_info() */
			ret = ENGINE_ASYNC_CE_GK20A;
		/* inst_id starts from CE0 to CE2 */
		if (inst_id)
			*inst_id = (engine_type - top_device_info_type_enum_copy0_v());
	}

	return ret;
}

int gk20a_fifo_init_engine_info(struct fifo_gk20a *f)
{
	struct gk20a *g = f->g;
	struct device *d = dev_from_gk20a(g);
	u32 i;
	u32 max_info_entries = top_device_info__size_1_v();
	u32 engine_enum = ENGINE_INVAL_GK20A;
	u32 engine_id = FIFO_INVAL_ENGINE_ID;
	u32 runlist_id = ~0;
	u32 pbdma_id = ~0;
	u32 intr_id = ~0;
	u32 reset_id = ~0;
	u32 inst_id  = 0;
	u32 pri_base = 0;
	u32 fault_id = 0;
	u32 gr_runlist_id = ~0;
	bool found_pbdma_for_runlist = false;

	gk20a_dbg_fn("");

	f->num_engines = 0;

	for (i = 0; i < max_info_entries; i++) {
		u32 table_entry = gk20a_readl(f->g, top_device_info_r(i));
		u32 entry = top_device_info_entry_v(table_entry);
		u32 runlist_bit;

		if (entry == top_device_info_entry_enum_v()) {
			if (top_device_info_engine_v(table_entry)) {
				engine_id =
					top_device_info_engine_enum_v(table_entry);
				gk20a_dbg_info("info: engine_id %d",
					top_device_info_engine_enum_v(table_entry));
			}


			if (top_device_info_runlist_v(table_entry)) {
				runlist_id =
					top_device_info_runlist_enum_v(table_entry);
				gk20a_dbg_info("gr info: runlist_id %d", runlist_id);

				runlist_bit = BIT(runlist_id);

				found_pbdma_for_runlist = false;
				for (pbdma_id = 0; pbdma_id < f->num_pbdma;
								pbdma_id++) {
					if (f->pbdma_map[pbdma_id] &
								runlist_bit) {
						gk20a_dbg_info(
						"gr info: pbdma_map[%d]=%d",
							pbdma_id,
							f->pbdma_map[pbdma_id]);
						found_pbdma_for_runlist = true;
					}
				}

				if (!found_pbdma_for_runlist) {
					gk20a_err(d, "busted pbdma map");
					return -EINVAL;
				}
			}

			if (top_device_info_intr_v(table_entry)) {
				intr_id =
					top_device_info_intr_enum_v(table_entry);
				gk20a_dbg_info("gr info: intr_id %d", intr_id);
			}

			if (top_device_info_reset_v(table_entry)) {
				reset_id =
					top_device_info_reset_enum_v(table_entry);
				gk20a_dbg_info("gr info: reset_id %d",
						reset_id);
			}
		} else if (entry == top_device_info_entry_engine_type_v()) {
			u32 engine_type =
				top_device_info_type_enum_v(table_entry);
			engine_enum =
				g->ops.fifo.engine_enum_from_type(g,
						engine_type, &inst_id);
		} else if (entry == top_device_info_entry_data_v()) {
			/* gk20a doesn't support device_info_data packet parsing */
			if (g->ops.fifo.device_info_data_parse)
				g->ops.fifo.device_info_data_parse(g,
					table_entry, &inst_id, &pri_base,
					&fault_id);
		}

		if (!top_device_info_chain_v(table_entry)) {
			if (engine_enum < ENGINE_INVAL_GK20A) {
				struct fifo_engine_info_gk20a *info =
					&g->fifo.engine_info[engine_id];

				info->intr_mask |= BIT(intr_id);
				info->reset_mask |= BIT(reset_id);
				info->runlist_id = runlist_id;
				info->pbdma_id = pbdma_id;
				info->inst_id  = inst_id;
				info->pri_base = pri_base;

				if (engine_enum == ENGINE_GR_GK20A)
					gr_runlist_id = runlist_id;

				/* GR and GR_COPY shares same runlist_id */
				if ((engine_enum == ENGINE_ASYNC_CE_GK20A) &&
					(gr_runlist_id == runlist_id))
						engine_enum = ENGINE_GRCE_GK20A;

				info->engine_enum = engine_enum;

				if (!fault_id && (engine_enum == ENGINE_GRCE_GK20A))
					fault_id = 0x1b;
				info->fault_id = fault_id;

				/* engine_id starts from 0 to NV_HOST_NUM_ENGINES */
				f->active_engines_list[f->num_engines] = engine_id;

				++f->num_engines;

				engine_enum = ENGINE_INVAL_GK20A;
			}
		}
	}

	return 0;
}

u32 gk20a_fifo_engine_interrupt_mask(struct gk20a *g)
{
	u32 eng_intr_mask = 0;
	unsigned int i;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 intr_mask;
		active_engine_id = g->fifo.active_engines_list[i];
		intr_mask = g->fifo.engine_info[active_engine_id].intr_mask;
		engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
		if (((engine_enum == ENGINE_GRCE_GK20A) ||
			(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
			(!g->ops.ce2.isr_stall || !g->ops.ce2.isr_nonstall))
				continue;

		eng_intr_mask |= intr_mask;
	}

	return eng_intr_mask;
}

void gk20a_fifo_delete_runlist(struct fifo_gk20a *f)
{
	u32 i;
	u32 runlist_id;
	struct fifo_runlist_info_gk20a *runlist;
	struct gk20a *g = NULL;

	if (!f || !f->runlist_info)
		return;

	g = f->g;

	for (runlist_id = 0; runlist_id < f->max_runlists; runlist_id++) {
		runlist = &f->runlist_info[runlist_id];
		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
			gk20a_gmmu_free(g, &runlist->mem[i]);
		}

		kfree(runlist->active_channels);
		runlist->active_channels = NULL;

		kfree(runlist->active_tsgs);
		runlist->active_tsgs = NULL;

		nvgpu_mutex_destroy(&runlist->mutex);

	}
	memset(f->runlist_info, 0, (sizeof(struct fifo_runlist_info_gk20a) *
		f->max_runlists));

	kfree(f->runlist_info);
	f->runlist_info = NULL;
	f->max_runlists = 0;
}

static void gk20a_remove_fifo_support(struct fifo_gk20a *f)
{
	struct gk20a *g = f->g;
	unsigned int i = 0;

	gk20a_dbg_fn("");

	nvgpu_channel_worker_deinit(g);
	/*
	 * Make sure all channels are closed before deleting them.
	 */
	for (; i < f->num_channels; i++) {
		struct channel_gk20a *c = f->channel + i;

		/*
		 * Could race but worst that happens is we get an error message
		 * from gk20a_free_channel() complaining about multiple closes.
		 */
		if (c->referenceable)
			__gk20a_channel_kill(c);
	}

	vfree(f->channel);
	vfree(f->tsg);
	if (g->ops.mm.is_bar1_supported(g))
		gk20a_gmmu_unmap_free(&g->mm.bar1.vm, &f->userd);
	else
		gk20a_gmmu_free(g, &f->userd);

	gk20a_fifo_delete_runlist(f);

	kfree(f->pbdma_map);
	f->pbdma_map = NULL;
	kfree(f->engine_info);
	f->engine_info = NULL;
	kfree(f->active_engines_list);
	f->active_engines_list = NULL;
#ifdef CONFIG_DEBUG_FS
	nvgpu_mutex_acquire(&f->profile.lock);
	if (f->profile.enabled) {
		f->profile.enabled = false;
		kref_put(&f->profile.ref, __gk20a_fifo_profile_free);
	}
	nvgpu_mutex_release(&f->profile.lock);
#endif
}

/* reads info from hardware and fills in pbmda exception info record */
static inline void get_exception_pbdma_info(
	struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_pbdma_exception_info_gk20a *e =
		&eng_info->pbdma_exception_info;

	u32 pbdma_status_r = e->status_r = gk20a_readl(g,
		   fifo_pbdma_status_r(eng_info->pbdma_id));
	e->id = fifo_pbdma_status_id_v(pbdma_status_r); /* vs. id_hw_v()? */
	e->id_is_chid = fifo_pbdma_status_id_type_v(pbdma_status_r) ==
		fifo_pbdma_status_id_type_chid_v();
	e->chan_status_v  = fifo_pbdma_status_chan_status_v(pbdma_status_r);
	e->next_id_is_chid =
		fifo_pbdma_status_next_id_type_v(pbdma_status_r) ==
		fifo_pbdma_status_next_id_type_chid_v();
	e->next_id = fifo_pbdma_status_next_id_v(pbdma_status_r);
	e->chsw_in_progress =
		fifo_pbdma_status_chsw_v(pbdma_status_r) ==
		fifo_pbdma_status_chsw_in_progress_v();
}

static void fifo_pbdma_exception_status(struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_pbdma_exception_info_gk20a *e;
	get_exception_pbdma_info(g, eng_info);
	e = &eng_info->pbdma_exception_info;

	gk20a_dbg_fn("pbdma_id %d, "
		      "id_type %s, id %d, chan_status %d, "
		      "next_id_type %s, next_id %d, "
		      "chsw_in_progress %d",
		      eng_info->pbdma_id,
		      e->id_is_chid ? "chid" : "tsgid", e->id, e->chan_status_v,
		      e->next_id_is_chid ? "chid" : "tsgid", e->next_id,
		      e->chsw_in_progress);
}

/* reads info from hardware and fills in pbmda exception info record */
static inline void get_exception_engine_info(
	struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_engine_exception_info_gk20a *e =
		&eng_info->engine_exception_info;
	u32 engine_status_r = e->status_r =
		gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	e->id = fifo_engine_status_id_v(engine_status_r); /* vs. id_hw_v()? */
	e->id_is_chid = fifo_engine_status_id_type_v(engine_status_r) ==
		fifo_engine_status_id_type_chid_v();
	e->ctx_status_v = fifo_engine_status_ctx_status_v(engine_status_r);
	e->faulted =
		fifo_engine_status_faulted_v(engine_status_r) ==
		fifo_engine_status_faulted_true_v();
	e->idle =
		fifo_engine_status_engine_v(engine_status_r) ==
		fifo_engine_status_engine_idle_v();
	e->ctxsw_in_progress =
		fifo_engine_status_ctxsw_v(engine_status_r) ==
		fifo_engine_status_ctxsw_in_progress_v();
}

static void fifo_engine_exception_status(struct gk20a *g,
			       struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_engine_exception_info_gk20a *e;
	get_exception_engine_info(g, eng_info);
	e = &eng_info->engine_exception_info;

	gk20a_dbg_fn("engine_id %d, id_type %s, id %d, ctx_status %d, "
		      "faulted %d, idle %d, ctxsw_in_progress %d, ",
		      eng_info->engine_id, e->id_is_chid ? "chid" : "tsgid",
		      e->id, e->ctx_status_v,
		      e->faulted, e->idle,  e->ctxsw_in_progress);
}

static int init_runlist(struct gk20a *g, struct fifo_gk20a *f)
{
	struct fifo_runlist_info_gk20a *runlist;
	struct device *d = dev_from_gk20a(g);
	unsigned int runlist_id;
	u32 i;
	size_t runlist_size;
	u32 active_engine_id, pbdma_id, engine_id;
	struct fifo_engine_info_gk20a *engine_info;

	gk20a_dbg_fn("");

	f->max_runlists = g->ops.fifo.eng_runlist_base_size();
	f->runlist_info = kzalloc(sizeof(struct fifo_runlist_info_gk20a) *
				  f->max_runlists, GFP_KERNEL);
	if (!f->runlist_info)
		goto clean_up_runlist;

	memset(f->runlist_info, 0, (sizeof(struct fifo_runlist_info_gk20a) *
		f->max_runlists));

	for (runlist_id = 0; runlist_id < f->max_runlists; runlist_id++) {
		runlist = &f->runlist_info[runlist_id];

		runlist->active_channels =
			kzalloc(DIV_ROUND_UP(f->num_channels, BITS_PER_BYTE),
				GFP_KERNEL);
		if (!runlist->active_channels)
			goto clean_up_runlist;

		runlist->active_tsgs =
			kzalloc(DIV_ROUND_UP(f->num_channels, BITS_PER_BYTE),
				GFP_KERNEL);
		if (!runlist->active_tsgs)
			goto clean_up_runlist;

		runlist_size  = f->runlist_entry_size * f->num_runlist_entries;
		gk20a_dbg_info("runlist_entries %d runlist size %zu\n",
					f->num_runlist_entries, runlist_size);

		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
			int err = gk20a_gmmu_alloc_sys(g, runlist_size,
					&runlist->mem[i]);
			if (err) {
				dev_err(d, "memory allocation failed\n");
				goto clean_up_runlist;
			}
		}
		nvgpu_mutex_init(&runlist->mutex);

		/* None of buffers is pinned if this value doesn't change.
		    Otherwise, one of them (cur_buffer) must have been pinned. */
		runlist->cur_buffer = MAX_RUNLIST_BUFFERS;

		for (pbdma_id = 0; pbdma_id < f->num_pbdma; pbdma_id++) {
			if (f->pbdma_map[pbdma_id] & BIT(runlist_id))
				runlist->pbdma_bitmask |= BIT(pbdma_id);
		}
		gk20a_dbg_info("runlist %d : pbdma bitmask %x",
				 runlist_id, runlist->pbdma_bitmask);

		for (engine_id = 0; engine_id < f->num_engines; ++engine_id) {
			active_engine_id = f->active_engines_list[engine_id];
			engine_info = &f->engine_info[active_engine_id];

			if (engine_info && engine_info->runlist_id == runlist_id)
				runlist->eng_bitmask |= BIT(engine_id);
		}
		gk20a_dbg_info("runlist %d : eng bitmask %x",
				 runlist_id, runlist->eng_bitmask);
	}


	gk20a_dbg_fn("done");
	return 0;

clean_up_runlist:
	gk20a_fifo_delete_runlist(f);
	gk20a_dbg_fn("fail");
	return -ENOMEM;
}

#define GRFIFO_TIMEOUT_CHECK_PERIOD_US 100000

int gk20a_init_fifo_reset_enable_hw(struct gk20a *g)
{
	u32 intr_stall;
	u32 mask;
	u32 timeout;
	unsigned int i;
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	gk20a_dbg_fn("");
	/* enable pmc pfifo */
	gk20a_reset(g, mc_enable_pfifo_enabled_f());

	if (g->ops.clock_gating.slcg_ce2_load_gating_prod)
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.slcg_fifo_load_gating_prod)
		g->ops.clock_gating.slcg_fifo_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.blcg_fifo_load_gating_prod)
		g->ops.clock_gating.blcg_fifo_load_gating_prod(g,
				g->blcg_enabled);

	/* enable pbdma */
	mask = 0;
	for (i = 0; i < host_num_pbdma; ++i)
		mask |= mc_enable_pb_sel_f(mc_enable_pb_0_enabled_v(), i);
	gk20a_writel(g, mc_enable_pb_r(), mask);

	/* enable pfifo interrupt */
	gk20a_writel(g, fifo_intr_0_r(), 0xFFFFFFFF);
	gk20a_writel(g, fifo_intr_en_0_r(), 0x7FFFFFFF);
	gk20a_writel(g, fifo_intr_en_1_r(), 0x80000000);

	/* enable pbdma interrupt */
	mask = 0;
	for (i = 0; i < host_num_pbdma; i++) {
		intr_stall = gk20a_readl(g, pbdma_intr_stall_r(i));
		intr_stall &= ~pbdma_intr_stall_lbreq_enabled_f();
		gk20a_writel(g, pbdma_intr_stall_r(i), intr_stall);
		gk20a_writel(g, pbdma_intr_0_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_en_0_r(i),
			~pbdma_intr_en_0_lbreq_enabled_f());
		gk20a_writel(g, pbdma_intr_1_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_en_1_r(i),
			~pbdma_intr_en_0_lbreq_enabled_f());
	}

	/* TBD: apply overrides */

	/* TBD: BLCG prod */

	/* reset runlist interrupts */
	gk20a_writel(g, fifo_intr_runlist_r(), ~0);

	/* TBD: do we need those? */
	timeout = gk20a_readl(g, fifo_fb_timeout_r());
	timeout = set_field(timeout, fifo_fb_timeout_period_m(),
			fifo_fb_timeout_period_max_f());
	gk20a_writel(g, fifo_fb_timeout_r(), timeout);

	/* write pbdma timeout value */
	for (i = 0; i < host_num_pbdma; i++) {
		timeout = gk20a_readl(g, pbdma_timeout_r(i));
		timeout = set_field(timeout, pbdma_timeout_period_m(),
				    pbdma_timeout_period_max_f());
		gk20a_writel(g, pbdma_timeout_r(i), timeout);
	}

	if (g->ops.fifo.apply_pb_timeout)
		g->ops.fifo.apply_pb_timeout(g);

	timeout = GRFIFO_TIMEOUT_CHECK_PERIOD_US;
	timeout = scale_ptimer(timeout,
		ptimer_scalingfactor10x(platform->ptimer_src_freq));
	timeout |= fifo_eng_timeout_detection_enabled_f();
	gk20a_writel(g, fifo_eng_timeout_r(), timeout);

	gk20a_dbg_fn("done");

	return 0;
}

static void gk20a_init_fifo_pbdma_intr_descs(struct fifo_gk20a *f)
{
	/* These are all errors which indicate something really wrong
	 * going on in the device. */
	f->intr.pbdma.device_fatal_0 =
		pbdma_intr_0_memreq_pending_f() |
		pbdma_intr_0_memack_timeout_pending_f() |
		pbdma_intr_0_memack_extra_pending_f() |
		pbdma_intr_0_memdat_timeout_pending_f() |
		pbdma_intr_0_memdat_extra_pending_f() |
		pbdma_intr_0_memflush_pending_f() |
		pbdma_intr_0_memop_pending_f() |
		pbdma_intr_0_lbconnect_pending_f() |
		pbdma_intr_0_lback_timeout_pending_f() |
		pbdma_intr_0_lback_extra_pending_f() |
		pbdma_intr_0_lbdat_timeout_pending_f() |
		pbdma_intr_0_lbdat_extra_pending_f() |
		pbdma_intr_0_xbarconnect_pending_f() |
		pbdma_intr_0_pri_pending_f();

	/* These are data parsing, framing errors or others which can be
	 * recovered from with intervention... or just resetting the
	 * channel. */
	f->intr.pbdma.channel_fatal_0 =
		pbdma_intr_0_gpfifo_pending_f() |
		pbdma_intr_0_gpptr_pending_f() |
		pbdma_intr_0_gpentry_pending_f() |
		pbdma_intr_0_gpcrc_pending_f() |
		pbdma_intr_0_pbptr_pending_f() |
		pbdma_intr_0_pbentry_pending_f() |
		pbdma_intr_0_pbcrc_pending_f() |
		pbdma_intr_0_method_pending_f() |
		pbdma_intr_0_methodcrc_pending_f() |
		pbdma_intr_0_pbseg_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents
	 * a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

static int gk20a_init_fifo_setup_sw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct device *d = dev_from_gk20a(g);
	unsigned int chid, i;
	int err = 0;

	gk20a_dbg_fn("");

	if (f->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	f->g = g;

	nvgpu_mutex_init(&f->intr.isr.mutex);
	nvgpu_mutex_init(&f->gr_reset_mutex);
	gk20a_init_fifo_pbdma_intr_descs(f); /* just filling in data/tables */

	f->num_channels = g->ops.fifo.get_num_fifos(g);
	f->runlist_entry_size =  g->ops.fifo.runlist_entry_size();
	f->num_runlist_entries = fifo_eng_runlist_length_max_v();
	f->num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);
	f->max_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	f->userd_entry_size = 1 << ram_userd_base_shift_v();

	f->channel = vzalloc(f->num_channels * sizeof(*f->channel));
	f->tsg = vzalloc(f->num_channels * sizeof(*f->tsg));
	f->pbdma_map = kzalloc(f->num_pbdma * sizeof(*f->pbdma_map),
				GFP_KERNEL);
	f->engine_info = kzalloc(f->max_engines * sizeof(*f->engine_info),
				GFP_KERNEL);
	f->active_engines_list = kzalloc(f->max_engines * sizeof(u32),
				GFP_KERNEL);

	if (!(f->channel && f->pbdma_map && f->engine_info &&
		f->active_engines_list)) {
		err = -ENOMEM;
		goto clean_up;
	}
	memset(f->active_engines_list, 0xff, (f->max_engines * sizeof(u32)));

	/* pbdma map needs to be in place before calling engine info init */
	for (i = 0; i < f->num_pbdma; ++i)
		f->pbdma_map[i] = gk20a_readl(g, fifo_pbdma_map_r(i));

	g->ops.fifo.init_engine_info(f);

	init_runlist(g, f);

	INIT_LIST_HEAD(&f->free_chs);
	nvgpu_mutex_init(&f->free_chs_mutex);

	if (g->ops.mm.is_bar1_supported(g))
		err = gk20a_gmmu_alloc_map_sys(&g->mm.bar1.vm,
				   f->userd_entry_size * f->num_channels,
				   &f->userd);

	else
		err = gk20a_gmmu_alloc_sys(g, f->userd_entry_size *
				f->num_channels, &f->userd);
	if (err) {
		dev_err(d, "userd memory allocation failed\n");
		goto clean_up;
	}
	gk20a_dbg(gpu_dbg_map, "userd gpu va = 0x%llx", f->userd.gpu_va);

	for (chid = 0; chid < f->num_channels; chid++) {
		f->channel[chid].userd_iova =
			g->ops.mm.get_iova_addr(g, f->userd.sgt->sgl, 0)
			+ chid * f->userd_entry_size;
		f->channel[chid].userd_gpu_va =
			f->userd.gpu_va + chid * f->userd_entry_size;
		gk20a_init_channel_support(g, chid);
		gk20a_init_tsg_support(g, chid);
	}
	nvgpu_mutex_init(&f->tsg_inuse_mutex);

	err = nvgpu_channel_worker_init(g);
	if (err)
		goto clean_up;
	f->remove_support = gk20a_remove_fifo_support;

	f->deferred_reset_pending = false;
	nvgpu_mutex_init(&f->deferred_reset_mutex);

	f->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_dbg_fn("fail");
	if (g->ops.mm.is_bar1_supported(g))
		gk20a_gmmu_unmap_free(&g->mm.bar1.vm, &f->userd);
	else
		gk20a_gmmu_free(g, &f->userd);

	vfree(f->channel);
	f->channel = NULL;
	vfree(f->tsg);
	f->tsg = NULL;
	kfree(f->pbdma_map);
	f->pbdma_map = NULL;
	kfree(f->engine_info);
	f->engine_info = NULL;
	kfree(f->active_engines_list);
	f->active_engines_list = NULL;

	return err;
}

static void gk20a_fifo_handle_runlist_event(struct gk20a *g)
{
	u32 runlist_event = gk20a_readl(g, fifo_intr_runlist_r());

	gk20a_dbg(gpu_dbg_intr, "runlist event %08x\n",
		  runlist_event);

	gk20a_writel(g, fifo_intr_runlist_r(), runlist_event);
}

int gk20a_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	gk20a_dbg_fn("");

	/* test write, read through bar1 @ userd region before
	 * turning on the snooping */
	{
		struct fifo_gk20a *f = &g->fifo;
		u32 v, v1 = 0x33, v2 = 0x55;

		u32 bar1_vaddr = f->userd.gpu_va;
		volatile u32 *cpu_vaddr = f->userd.cpu_va;

		gk20a_dbg_info("test bar1 @ vaddr 0x%x",
			   bar1_vaddr);

		v = gk20a_bar1_readl(g, bar1_vaddr);

		*cpu_vaddr = v1;
		smp_mb();

		if (v1 != gk20a_bar1_readl(g, bar1_vaddr)) {
			gk20a_err(dev_from_gk20a(g), "bar1 broken @ gk20a: CPU wrote 0x%x, \
				GPU read 0x%x", *cpu_vaddr, gk20a_bar1_readl(g, bar1_vaddr));
			return -EINVAL;
		}

		gk20a_bar1_writel(g, bar1_vaddr, v2);

		if (v2 != gk20a_bar1_readl(g, bar1_vaddr)) {
			gk20a_err(dev_from_gk20a(g), "bar1 broken @ gk20a: GPU wrote 0x%x, \
				CPU read 0x%x", gk20a_bar1_readl(g, bar1_vaddr), *cpu_vaddr);
			return -EINVAL;
		}

		/* is it visible to the cpu? */
		if (*cpu_vaddr != v2) {
			gk20a_err(dev_from_gk20a(g),
				"cpu didn't see bar1 write @ %p!",
				cpu_vaddr);
		}

		/* put it back */
		gk20a_bar1_writel(g, bar1_vaddr, v);
	}

	/*XXX all manner of flushes and caching worries, etc */

	/* set the base for the userd region now */
	gk20a_writel(g, fifo_bar1_base_r(),
			fifo_bar1_base_ptr_f(f->userd.gpu_va >> 12) |
			fifo_bar1_base_valid_true_f());

	gk20a_dbg_fn("done");

	return 0;
}

int gk20a_init_fifo_support(struct gk20a *g)
{
	u32 err;

	err = gk20a_init_fifo_setup_sw(g);
	if (err)
		return err;

	if (g->ops.fifo.init_fifo_setup_hw)
		err = g->ops.fifo.init_fifo_setup_hw(g);
	if (err)
		return err;

	return err;
}

/* return with a reference to the channel, caller must put it back */
static struct channel_gk20a *
channel_from_inst_ptr(struct fifo_gk20a *f, u64 inst_ptr)
{
	struct gk20a *g = f->g;
	unsigned int ci;
	if (unlikely(!f->channel))
		return NULL;
	for (ci = 0; ci < f->num_channels; ci++) {
		struct channel_gk20a *ch;
		u64 ch_inst_ptr;

		ch = gk20a_channel_get(&f->channel[ci]);
		/* only alive channels are searched */
		if (!ch)
			continue;

		ch_inst_ptr = gk20a_mm_inst_block_addr(g, &ch->inst_block);
		if (inst_ptr == ch_inst_ptr)
			return ch;

		gk20a_channel_put(ch);
	}
	return NULL;
}

/* fault info/descriptions.
 * tbd: move to setup
 *  */
static const char * const fault_type_descs[] = {
	 "pde", /*fifo_intr_mmu_fault_info_type_pde_v() == 0 */
	 "pde size",
	 "pte",
	 "va limit viol",
	 "unbound inst",
	 "priv viol",
	 "ro viol",
	 "wo viol",
	 "pitch mask",
	 "work creation",
	 "bad aperture",
	 "compression failure",
	 "bad kind",
	 "region viol",
	 "dual ptes",
	 "poisoned",
};
/* engine descriptions */
static const char * const engine_subid_descs[] = {
	"gpc",
	"hub",
};

static const char * const hub_client_descs[] = {
	"vip", "ce0", "ce1", "dniso", "fe", "fecs", "host", "host cpu",
	"host cpu nb", "iso", "mmu", "mspdec", "msppp", "msvld",
	"niso", "p2p", "pd", "perf", "pmu", "raster twod", "scc",
	"scc nb", "sec", "ssync", "gr copy", "ce2", "xv", "mmu nb",
	"msenc", "d falcon", "sked", "a falcon", "n/a",
};

static const char * const gpc_client_descs[] = {
	"l1 0", "t1 0", "pe 0",
	"l1 1", "t1 1", "pe 1",
	"l1 2", "t1 2", "pe 2",
	"l1 3", "t1 3", "pe 3",
	"rast", "gcc", "gpccs",
	"prop 0", "prop 1", "prop 2", "prop 3",
	"l1 4", "t1 4", "pe 4",
	"l1 5", "t1 5", "pe 5",
	"l1 6", "t1 6", "pe 6",
	"l1 7", "t1 7", "pe 7",
	"gpm",
	"ltp utlb 0", "ltp utlb 1", "ltp utlb 2", "ltp utlb 3",
	"rgg utlb",
};

/* reads info from hardware and fills in mmu fault info record */
static inline void get_exception_mmu_fault_info(
	struct gk20a *g, u32 engine_id,
	struct fifo_mmu_fault_info_gk20a *f)
{
	u32 fault_info_v;

	gk20a_dbg_fn("engine_id %d", engine_id);

	memset(f, 0, sizeof(*f));

	f->fault_info_v = fault_info_v = gk20a_readl(g,
	     fifo_intr_mmu_fault_info_r(engine_id));
	f->fault_type_v =
		fifo_intr_mmu_fault_info_type_v(fault_info_v);
	f->engine_subid_v =
		fifo_intr_mmu_fault_info_engine_subid_v(fault_info_v);
	f->client_v = fifo_intr_mmu_fault_info_client_v(fault_info_v);

	BUG_ON(f->fault_type_v >= ARRAY_SIZE(fault_type_descs));
	f->fault_type_desc =  fault_type_descs[f->fault_type_v];

	BUG_ON(f->engine_subid_v >= ARRAY_SIZE(engine_subid_descs));
	f->engine_subid_desc = engine_subid_descs[f->engine_subid_v];

	if (f->engine_subid_v ==
	    fifo_intr_mmu_fault_info_engine_subid_hub_v()) {

		BUG_ON(f->client_v >= ARRAY_SIZE(hub_client_descs));
		f->client_desc = hub_client_descs[f->client_v];
	} else if (f->engine_subid_v ==
		   fifo_intr_mmu_fault_info_engine_subid_gpc_v()) {
		BUG_ON(f->client_v >= ARRAY_SIZE(gpc_client_descs));
		f->client_desc = gpc_client_descs[f->client_v];
	} else {
		BUG_ON(1);
	}

	f->fault_hi_v = gk20a_readl(g, fifo_intr_mmu_fault_hi_r(engine_id));
	f->fault_lo_v = gk20a_readl(g, fifo_intr_mmu_fault_lo_r(engine_id));
	/* note:ignoring aperture on gk20a... */
	f->inst_ptr = fifo_intr_mmu_fault_inst_ptr_v(
		 gk20a_readl(g, fifo_intr_mmu_fault_inst_r(engine_id)));
	/* note: inst_ptr is a 40b phys addr.  */
	f->inst_ptr <<= fifo_intr_mmu_fault_inst_ptr_align_shift_v();
}

void gk20a_fifo_reset_engine(struct gk20a *g, u32 engine_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	u32 inst_id = 0;
	struct fifo_engine_info_gk20a *engine_info;

	gk20a_dbg_fn("");

	if (!g)
		return;

	f = &g->fifo;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		engine_enum = engine_info->engine_enum;
		inst_id = engine_info->inst_id;
	}

	if (engine_enum == ENGINE_INVAL_GK20A)
		gk20a_err(dev_from_gk20a(g), "unsupported engine_id %d", engine_id);

	if (engine_enum == ENGINE_GR_GK20A) {
		if (support_gk20a_pmu(g->dev) && g->elpg_enabled)
			gk20a_pmu_disable_elpg(g);
		/* resetting engine will alter read/write index.
		 * need to flush circular buffer before re-enabling FECS.
		 */
		if (g->ops.fecs_trace.reset)
			g->ops.fecs_trace.reset(g);
		/*HALT_PIPELINE method, halt GR engine*/
		if (gr_gk20a_halt_pipe(g))
			gk20a_err(dev_from_gk20a(g), "failed to HALT gr pipe");
		/* resetting engine using mc_enable_r() is not
		enough, we do full init sequence */
		gk20a_gr_reset(g);
		if (support_gk20a_pmu(g->dev) && g->elpg_enabled)
			gk20a_pmu_enable_elpg(g);
	}
	if ((engine_enum == ENGINE_GRCE_GK20A) ||
		(engine_enum == ENGINE_ASYNC_CE_GK20A)) {
			gk20a_reset(g, engine_info->reset_mask);
	}
}

static void gk20a_fifo_handle_chsw_fault(struct gk20a *g)
{
	u32 intr;

	intr = gk20a_readl(g, fifo_intr_chsw_error_r());
	gk20a_err(dev_from_gk20a(g), "chsw: %08x\n", intr);
	gk20a_fecs_dump_falcon_stats(g);
	gk20a_writel(g, fifo_intr_chsw_error_r(), intr);
}

static void gk20a_fifo_handle_dropped_mmu_fault(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	u32 fault_id = gk20a_readl(g, fifo_intr_mmu_fault_id_r());
	gk20a_err(dev, "dropped mmu fault (0x%08x)", fault_id);
}

bool gk20a_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid)
{
	return (engine_subid == fifo_intr_mmu_fault_info_engine_subid_gpc_v());
}

static bool gk20a_fifo_should_defer_engine_reset(struct gk20a *g, u32 engine_id,
		u32 engine_subid, bool fake_fault)
{
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_gk20a *fifo = NULL;
	struct fifo_engine_info_gk20a *engine_info;

	if (!g)
		return false;

	fifo = &g->fifo;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		engine_enum = engine_info->engine_enum;
	}

	if (engine_enum == ENGINE_INVAL_GK20A)
		return false;

	/* channel recovery is only deferred if an sm debugger
	   is attached and has MMU debug mode is enabled */
	if (!gk20a_gr_sm_debugger_attached(g) ||
	    !g->ops.mm.is_debug_mode_enabled(g))
		return false;

	/* if this fault is fake (due to RC recovery), don't defer recovery */
	if (fake_fault)
		return false;

	if (engine_enum != ENGINE_GR_GK20A)
		return false;

	return g->ops.fifo.is_fault_engine_subid_gpc(g, engine_subid);
}

/* caller must hold a channel reference */
static bool gk20a_fifo_set_ctx_mmu_error(struct gk20a *g,
		struct channel_gk20a *ch)
{
	bool verbose = true;
	if (!ch)
		return verbose;

	nvgpu_mutex_acquire(&ch->error_notifier_mutex);
	if (ch->error_notifier_ref) {
		u32 err = ch->error_notifier->info32;
		if (ch->error_notifier->status == 0xffff) {
			/* If error code is already set, this mmu fault
			 * was triggered as part of recovery from other
			 * error condition.
			 * Don't overwrite error flag. */
			/* Fifo timeout debug spew is controlled by user */
			if (err == NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT)
				verbose = ch->timeout_debug_dump;
		} else {
			gk20a_set_error_notifier_locked(ch,
				NVGPU_CHANNEL_FIFO_ERROR_MMU_ERR_FLT);
		}
	}
	nvgpu_mutex_release(&ch->error_notifier_mutex);

	/* mark channel as faulted */
	ch->has_timedout = true;
	wmb();
	/* unblock pending waits */
	wake_up(&ch->semaphore_wq);
	wake_up(&ch->notifier_wq);
	return verbose;
}

bool gk20a_fifo_set_ctx_mmu_error_ch(struct gk20a *g,
		struct channel_gk20a *ch)
{
	gk20a_err(dev_from_gk20a(g),
		"channel %d generated a mmu fault", ch->hw_chid);

	return gk20a_fifo_set_ctx_mmu_error(g, ch);
}

bool gk20a_fifo_set_ctx_mmu_error_tsg(struct gk20a *g,
		struct tsg_gk20a *tsg)
{
	bool ret = true;
	struct channel_gk20a *ch = NULL;

	gk20a_err(dev_from_gk20a(g),
		"TSG %d generated a mmu fault", tsg->tsgid);

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		if (gk20a_channel_get(ch)) {
			if (!gk20a_fifo_set_ctx_mmu_error(g, ch))
				ret = false;
			gk20a_channel_put(ch);
		}
	}
	up_read(&tsg->ch_list_lock);

	return ret;
}

void gk20a_fifo_abort_tsg(struct gk20a *g, u32 tsgid, bool preempt)
{
	struct tsg_gk20a *tsg = &g->fifo.tsg[tsgid];
	struct channel_gk20a *ch;

	gk20a_dbg_fn("");

	gk20a_disable_tsg(tsg);

	if (preempt)
		g->ops.fifo.preempt_tsg(g, tsgid);

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		if (gk20a_channel_get(ch)) {
			ch->has_timedout = true;
			gk20a_channel_abort_clean_up(ch);
			gk20a_channel_put(ch);
		}
	}
	up_read(&tsg->ch_list_lock);
}

int gk20a_fifo_deferred_reset(struct gk20a *g, struct channel_gk20a *ch)
{
	u32 engine_id, engines;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	gr_gk20a_disable_ctxsw(g);

	if (!g->fifo.deferred_reset_pending)
		goto clean_up;

	if (gk20a_is_channel_marked_as_tsg(ch))
		engines = gk20a_fifo_engines_on_id(g, ch->tsgid, true);
	else
		engines = gk20a_fifo_engines_on_id(g, ch->hw_chid, false);
	if (!engines)
		goto clean_up;

	/*
	 * If deferred reset is set for an engine, and channel is running
	 * on that engine, reset it
	 */
	for_each_set_bit(engine_id, &g->fifo.deferred_fault_engines, 32) {
		if (BIT(engine_id) & engines)
			gk20a_fifo_reset_engine(g, engine_id);
	}

	g->fifo.deferred_fault_engines = 0;
	g->fifo.deferred_reset_pending = false;

clean_up:
	gr_gk20a_enable_ctxsw(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return 0;
}

static bool gk20a_fifo_handle_mmu_fault(
	struct gk20a *g,
	u32 mmu_fault_engines, /* queried from HW if 0 */
	u32 hw_id, /* queried from HW if ~(u32)0 OR mmu_fault_engines == 0*/
	bool id_is_tsg)
{
	bool fake_fault;
	unsigned long fault_id;
	unsigned long engine_mmu_fault_id;
	bool verbose = true;
	u32 grfifo_ctl;

	gk20a_dbg_fn("");

	g->fifo.deferred_reset_pending = false;

	/* Disable power management */
	if (support_gk20a_pmu(g->dev) && g->elpg_enabled)
		gk20a_pmu_disable_elpg(g);
	if (g->ops.clock_gating.slcg_gr_load_gating_prod)
		g->ops.clock_gating.slcg_gr_load_gating_prod(g,
				false);
	if (g->ops.clock_gating.slcg_perf_load_gating_prod)
		g->ops.clock_gating.slcg_perf_load_gating_prod(g,
				false);
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
				false);

	gr_gk20a_init_cg_mode(g, ELCG_MODE, ELCG_RUN);

	/* Disable fifo access */
	grfifo_ctl = gk20a_readl(g, gr_gpfifo_ctl_r());
	grfifo_ctl &= ~gr_gpfifo_ctl_semaphore_access_f(1);
	grfifo_ctl &= ~gr_gpfifo_ctl_access_f(1);

	gk20a_writel(g, gr_gpfifo_ctl_r(),
		grfifo_ctl | gr_gpfifo_ctl_access_f(0) |
		gr_gpfifo_ctl_semaphore_access_f(0));

	if (mmu_fault_engines) {
		fault_id = mmu_fault_engines;
		fake_fault = true;
	} else {
		fault_id = gk20a_readl(g, fifo_intr_mmu_fault_id_r());
		fake_fault = false;
		gk20a_debug_dump(g->dev);
	}


	/* go through all faulted engines */
	for_each_set_bit(engine_mmu_fault_id, &fault_id, 32) {
		/* bits in fifo_intr_mmu_fault_id_r do not correspond 1:1 to
		 * engines. Convert engine_mmu_id to engine_id */
		u32 engine_id = gk20a_mmu_id_to_engine_id(g,
					engine_mmu_fault_id);
		struct fifo_mmu_fault_info_gk20a f;
		struct channel_gk20a *ch = NULL;
		struct tsg_gk20a *tsg = NULL;
		struct channel_gk20a *referenced_channel = NULL;
		/* read and parse engine status */
		u32 status = gk20a_readl(g, fifo_engine_status_r(engine_id));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);
		bool ctxsw = (ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_switch_v()
				|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_save_v()
				|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v());

		get_exception_mmu_fault_info(g, engine_mmu_fault_id, &f);
		trace_gk20a_mmu_fault(f.fault_hi_v,
				      f.fault_lo_v,
				      f.fault_info_v,
				      f.inst_ptr,
				      engine_id,
				      f.engine_subid_desc,
				      f.client_desc,
				      f.fault_type_desc);
		gk20a_err(dev_from_gk20a(g), "%s mmu fault on engine %d, "
			   "engine subid %d (%s), client %d (%s), "
			   "addr 0x%08x:0x%08x, type %d (%s), info 0x%08x,"
			   "inst_ptr 0x%llx\n",
			   fake_fault ? "fake" : "",
			   engine_id,
			   f.engine_subid_v, f.engine_subid_desc,
			   f.client_v, f.client_desc,
			   f.fault_hi_v, f.fault_lo_v,
			   f.fault_type_v, f.fault_type_desc,
			   f.fault_info_v, f.inst_ptr);

		if (ctxsw) {
			gk20a_fecs_dump_falcon_stats(g);
			gk20a_err(dev_from_gk20a(g), "gr_status_r : 0x%x",
					gk20a_readl(g, gr_status_r()));
		}

		/* get the channel/TSG */
		if (fake_fault) {
			/* use next_id if context load is failing */
			u32 id, type;

			if (hw_id == ~(u32)0) {
				id = (ctx_status ==
				      fifo_engine_status_ctx_status_ctxsw_load_v()) ?
					fifo_engine_status_next_id_v(status) :
					fifo_engine_status_id_v(status);
				type = (ctx_status ==
					fifo_engine_status_ctx_status_ctxsw_load_v()) ?
					fifo_engine_status_next_id_type_v(status) :
					fifo_engine_status_id_type_v(status);
			} else {
				id = hw_id;
				type = id_is_tsg ?
					fifo_engine_status_id_type_tsgid_v() :
					fifo_engine_status_id_type_chid_v();
			}

			if (type == fifo_engine_status_id_type_tsgid_v())
				tsg = &g->fifo.tsg[id];
			else if (type == fifo_engine_status_id_type_chid_v()) {
				ch = &g->fifo.channel[id];
				referenced_channel = gk20a_channel_get(ch);
			}
		} else {
			/* read channel based on instruction pointer */
			ch = channel_from_inst_ptr(&g->fifo, f.inst_ptr);
			referenced_channel = ch;
		}

		if (ch && gk20a_is_channel_marked_as_tsg(ch))
			tsg = &g->fifo.tsg[ch->tsgid];

		/* check if engine reset should be deferred */
		if ((ch || tsg) && gk20a_fifo_should_defer_engine_reset(g,
				engine_id, f.engine_subid_v, fake_fault)) {
			g->fifo.deferred_fault_engines |= BIT(engine_id);

			/* handled during channel free */
			g->fifo.deferred_reset_pending = true;
			gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
				   "sm debugger attached,"
				   " deferring channel recovery to channel free");
		} else if (engine_id != FIFO_INVAL_ENGINE_ID) {
			/* if lock is already taken, a reset is taking place
			so no need to repeat */
			if (nvgpu_mutex_tryacquire(&g->fifo.gr_reset_mutex)) {
				gk20a_fifo_reset_engine(g, engine_id);
				nvgpu_mutex_release(&g->fifo.gr_reset_mutex);
			}
		}

		if (ch)
			gk20a_ctxsw_trace_channel_reset(g, ch);
		else if (tsg)
			gk20a_ctxsw_trace_tsg_reset(g, tsg);

		/* disable the channel/TSG from hw and increment
		 * syncpoints */

		if (tsg) {
			if (g->fifo.deferred_reset_pending) {
				gk20a_disable_tsg(tsg);
			} else {
				verbose =
				       gk20a_fifo_set_ctx_mmu_error_tsg(g, tsg);
				gk20a_fifo_abort_tsg(g, tsg->tsgid, false);
			}

			/* put back the ref taken early above */
			if (referenced_channel)
				gk20a_channel_put(ch);
		} else if (ch) {
			if (referenced_channel) {
				if (g->fifo.deferred_reset_pending) {
					g->ops.fifo.disable_channel(ch);
				} else {
					verbose = gk20a_fifo_set_ctx_mmu_error_ch(g, ch);
					gk20a_channel_abort(ch, false);
				}
				gk20a_channel_put(ch);
			} else {
				gk20a_err(dev_from_gk20a(g),
						"mmu error in freed channel %d",
						ch->hw_chid);
			}
		} else if (f.inst_ptr ==
				gk20a_mm_inst_block_addr(g, &g->mm.bar1.inst_block)) {
			gk20a_err(dev_from_gk20a(g), "mmu fault from bar1");
		} else if (f.inst_ptr ==
				gk20a_mm_inst_block_addr(g, &g->mm.pmu.inst_block)) {
			gk20a_err(dev_from_gk20a(g), "mmu fault from pmu");
		} else
			gk20a_err(dev_from_gk20a(g), "couldn't locate channel for mmu fault");
	}

	/* clear interrupt */
	gk20a_writel(g, fifo_intr_mmu_fault_id_r(), fault_id);

	/* resume scheduler */
	gk20a_writel(g, fifo_error_sched_disable_r(),
		     gk20a_readl(g, fifo_error_sched_disable_r()));

	/* Re-enable fifo access */
	gk20a_writel(g, gr_gpfifo_ctl_r(),
		     gr_gpfifo_ctl_access_enabled_f() |
		     gr_gpfifo_ctl_semaphore_access_enabled_f());

	/* It is safe to enable ELPG again. */
	if (support_gk20a_pmu(g->dev) && g->elpg_enabled)
		gk20a_pmu_enable_elpg(g);
	return verbose;
}

static void gk20a_fifo_get_faulty_id_type(struct gk20a *g, int engine_id,
					  u32 *id, u32 *type)
{
	u32 status = gk20a_readl(g, fifo_engine_status_r(engine_id));
	u32 ctx_status = fifo_engine_status_ctx_status_v(status);

	/* use next_id if context load is failing */
	*id = (ctx_status ==
		fifo_engine_status_ctx_status_ctxsw_load_v()) ?
		fifo_engine_status_next_id_v(status) :
		fifo_engine_status_id_v(status);

	*type = (ctx_status ==
		fifo_engine_status_ctx_status_ctxsw_load_v()) ?
		fifo_engine_status_next_id_type_v(status) :
		fifo_engine_status_id_type_v(status);
}

static void gk20a_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	unsigned long engine_id;

	/* trigger faults for all bad engines */
	for_each_set_bit(engine_id, &engine_ids, 32) {
		u32 mmu_id;

		if (!gk20a_fifo_is_valid_engine_id(g, engine_id)) {
			WARN_ON(true);
			break;
		}

		mmu_id = gk20a_engine_id_to_mmu_id(g, engine_id);
		if (mmu_id != FIFO_INVAL_ENGINE_ID)
			gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_id),
				     fifo_trigger_mmu_fault_id_f(mmu_id) |
				     fifo_trigger_mmu_fault_enable_f(1));
	}

	/* Wait for MMU fault to trigger */
	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);
	do {
		if (gk20a_readl(g, fifo_intr_0_r()) &
				fifo_intr_0_mmu_fault_pending_f())
			break;

		usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired_msg(&timeout, "mmu fault timeout"));

	/* release mmu fault trigger */
	for_each_set_bit(engine_id, &engine_ids, 32)
		gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_id), 0);
}

static u32 gk20a_fifo_engines_on_id(struct gk20a *g, u32 id, bool is_tsg)
{
	unsigned int i;
	u32 engines = 0;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		u32 status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		u32 ctx_status =
			fifo_engine_status_ctx_status_v(status);
		u32 ctx_id = (ctx_status ==
			fifo_engine_status_ctx_status_ctxsw_load_v()) ?
			fifo_engine_status_next_id_v(status) :
			fifo_engine_status_id_v(status);
		u32 type = (ctx_status ==
			fifo_engine_status_ctx_status_ctxsw_load_v()) ?
			fifo_engine_status_next_id_type_v(status) :
			fifo_engine_status_id_type_v(status);
		bool busy = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();
		if (busy && ctx_id == id) {
			if ((is_tsg && type ==
					fifo_engine_status_id_type_tsgid_v()) ||
				    (!is_tsg && type ==
					fifo_engine_status_id_type_chid_v()))
				engines |= BIT(active_engine_id);
		}
	}

	return engines;
}

void gk20a_fifo_recover_ch(struct gk20a *g, u32 hw_chid, bool verbose)
{
	u32 engines;

	/* stop context switching to prevent engine assignments from
	   changing until channel is recovered */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	gr_gk20a_disable_ctxsw(g);

	engines = gk20a_fifo_engines_on_id(g, hw_chid, false);

	if (engines)
		gk20a_fifo_recover(g, engines, hw_chid, false, true, verbose);
	else {
		struct channel_gk20a *ch = &g->fifo.channel[hw_chid];

		if (gk20a_channel_get(ch)) {
			gk20a_channel_abort(ch, false);

			if (gk20a_fifo_set_ctx_mmu_error_ch(g, ch))
				gk20a_debug_dump(g->dev);

			gk20a_channel_put(ch);
		}
	}

	gr_gk20a_enable_ctxsw(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

void gk20a_fifo_recover_tsg(struct gk20a *g, u32 tsgid, bool verbose)
{
	u32 engines;

	/* stop context switching to prevent engine assignments from
	   changing until TSG is recovered */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	gr_gk20a_disable_ctxsw(g);

	engines = gk20a_fifo_engines_on_id(g, tsgid, true);

	if (engines)
		gk20a_fifo_recover(g, engines, tsgid, true, true, verbose);
	else {
		struct tsg_gk20a *tsg = &g->fifo.tsg[tsgid];

		if (gk20a_fifo_set_ctx_mmu_error_tsg(g, tsg))
			gk20a_debug_dump(g->dev);

		gk20a_fifo_abort_tsg(g, tsgid, false);
	}

	gr_gk20a_enable_ctxsw(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

void gk20a_fifo_recover(struct gk20a *g, u32 __engine_ids,
			u32 hw_id, bool id_is_tsg,
			bool id_is_known, bool verbose)
{
	unsigned long engine_id, i;
	unsigned long _engine_ids = __engine_ids;
	unsigned long engine_ids = 0;
	u32 val;
	u32 mmu_fault_engines = 0;
	u32 ref_type;
	u32 ref_id;
	u32 ref_id_is_tsg = false;

	if (verbose)
		gk20a_debug_dump(g->dev);

	if (g->ops.ltc.flush)
		g->ops.ltc.flush(g);

	if (id_is_known) {
		engine_ids = gk20a_fifo_engines_on_id(g, hw_id, id_is_tsg);
		ref_id = hw_id;
		ref_type = id_is_tsg ?
			fifo_engine_status_id_type_tsgid_v() :
			fifo_engine_status_id_type_chid_v();
		ref_id_is_tsg = id_is_tsg;
		/* atleast one engine will get passed during sched err*/
		engine_ids |= __engine_ids;
		for_each_set_bit(engine_id, &engine_ids, 32) {
			u32 mmu_id = gk20a_engine_id_to_mmu_id(g, engine_id);

			if (mmu_id != FIFO_INVAL_ENGINE_ID)
				mmu_fault_engines |= BIT(mmu_id);
		}
	} else {
		/* store faulted engines in advance */
		for_each_set_bit(engine_id, &_engine_ids, 32) {
			gk20a_fifo_get_faulty_id_type(g, engine_id, &ref_id,
						      &ref_type);
			if (ref_type == fifo_engine_status_id_type_tsgid_v())
				ref_id_is_tsg = true;
			else
				ref_id_is_tsg = false;
			/* Reset *all* engines that use the
			 * same channel as faulty engine */
			for (i = 0; i < g->fifo.num_engines; i++) {
				u32 active_engine_id = g->fifo.active_engines_list[i];
				u32 type;
				u32 id;

				gk20a_fifo_get_faulty_id_type(g, active_engine_id, &id, &type);
				if (ref_type == type && ref_id == id) {
					u32 mmu_id = gk20a_engine_id_to_mmu_id(g, active_engine_id);

					engine_ids |= BIT(active_engine_id);
					if (mmu_id != FIFO_INVAL_ENGINE_ID)
						mmu_fault_engines |= BIT(mmu_id);
				}
			}
		}
	}

	if (mmu_fault_engines) {
		/*
		 * sched error prevents recovery, and ctxsw error will retrigger
		 * every 100ms. Disable the sched error to allow recovery.
		 */
		val = gk20a_readl(g, fifo_intr_en_0_r());
		val &= ~(fifo_intr_en_0_sched_error_m() |
			fifo_intr_en_0_mmu_fault_m());
		gk20a_writel(g, fifo_intr_en_0_r(), val);
		gk20a_writel(g, fifo_intr_0_r(),
				fifo_intr_0_sched_error_reset_f());

		g->ops.fifo.trigger_mmu_fault(g, engine_ids);
		gk20a_fifo_handle_mmu_fault(g, mmu_fault_engines, ref_id,
				ref_id_is_tsg);

		val = gk20a_readl(g, fifo_intr_en_0_r());
		val |= fifo_intr_en_0_mmu_fault_f(1)
			| fifo_intr_en_0_sched_error_f(1);
		gk20a_writel(g, fifo_intr_en_0_r(), val);
	}
}

/* force reset channel and tsg (if it's part of one) */
int gk20a_fifo_force_reset_ch(struct channel_gk20a *ch,
				u32 err_code, bool verbose)
{
	struct tsg_gk20a *tsg = NULL;
	struct channel_gk20a *ch_tsg = NULL;
	struct gk20a *g = ch->g;

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		tsg = &g->fifo.tsg[ch->tsgid];

		down_read(&tsg->ch_list_lock);

		list_for_each_entry(ch_tsg, &tsg->ch_list, ch_entry) {
			if (gk20a_channel_get(ch_tsg)) {
				gk20a_set_error_notifier(ch_tsg, err_code);
				gk20a_channel_put(ch_tsg);
			}
		}

		up_read(&tsg->ch_list_lock);
		gk20a_fifo_recover_tsg(g, ch->tsgid, verbose);
	} else {
		gk20a_set_error_notifier(ch, err_code);
		gk20a_fifo_recover_ch(g, ch->hw_chid, verbose);
	}

	return 0;
}

u32 gk20a_fifo_get_failing_engine_data(struct gk20a *g,
			int *__id, bool *__is_tsg)
{
	u32 engine_id;
	int id = -1;
	bool is_tsg = false;
	u32 mailbox2;
	u32 active_engine_id = FIFO_INVAL_ENGINE_ID;

	for (engine_id = 0; engine_id < g->fifo.num_engines; engine_id++) {
		u32 status;
		u32 ctx_status;
		bool failing_engine;

		active_engine_id = g->fifo.active_engines_list[engine_id];
		status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		ctx_status = fifo_engine_status_ctx_status_v(status);

		/* we are interested in busy engines */
		failing_engine = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();

		/* ..that are doing context switch */
		failing_engine = failing_engine &&
			(ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_switch_v()
			|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_save_v()
			|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v());

		if (!failing_engine) {
		    active_engine_id = FIFO_INVAL_ENGINE_ID;
			continue;
		}

		if (ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v()) {
			id = fifo_engine_status_next_id_v(status);
			is_tsg = fifo_engine_status_next_id_type_v(status) !=
				fifo_engine_status_next_id_type_chid_v();
		} else if (ctx_status ==
			       fifo_engine_status_ctx_status_ctxsw_switch_v()) {
			mailbox2 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(2));
			if (mailbox2 & FECS_METHOD_WFI_RESTORE) {
				id = fifo_engine_status_next_id_v(status);
				is_tsg = fifo_engine_status_next_id_type_v(status) !=
					fifo_engine_status_next_id_type_chid_v();
			} else {
				id = fifo_engine_status_id_v(status);
				is_tsg = fifo_engine_status_id_type_v(status) !=
					fifo_engine_status_id_type_chid_v();
			}
		} else {
			id = fifo_engine_status_id_v(status);
			is_tsg = fifo_engine_status_id_type_v(status) !=
				fifo_engine_status_id_type_chid_v();
		}
		break;
	}

	*__id = id;
	*__is_tsg = is_tsg;

	return active_engine_id;
}

static bool gk20a_fifo_check_ch_ctxsw_timeout(struct channel_gk20a *ch,
		bool *verbose, u32 *ms)
{
	bool recover = false;
	bool progress = false;

	if (gk20a_channel_get(ch)) {
		recover = gk20a_channel_update_and_check_timeout(ch,
				GRFIFO_TIMEOUT_CHECK_PERIOD_US / 1000,
				&progress);
		*verbose = ch->timeout_debug_dump;
		*ms = ch->timeout_accumulated_ms;
		if (recover)
			gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT);

		gk20a_channel_put(ch);
	}
	return recover;
}

static bool gk20a_fifo_check_tsg_ctxsw_timeout(struct tsg_gk20a *tsg,
		bool *verbose, u32 *ms)
{
	struct channel_gk20a *ch;
	bool recover = false;
	bool progress = false;

	*verbose = false;
	*ms = GRFIFO_TIMEOUT_CHECK_PERIOD_US / 1000;

	down_read(&tsg->ch_list_lock);

	/* check if there was some progress on any of the TSG channels.
	 * fifo recovery is needed if at least one channel reached the
	 * maximum timeout without progress (update in gpfifo pointers).
	 */
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		if (gk20a_channel_get(ch)) {
			recover = gk20a_channel_update_and_check_timeout(ch,
					*ms, &progress);
			if (progress || recover)
				break;
			gk20a_channel_put(ch);
		}
	}

	/* if at least one channel in the TSG made some progress, reset
	 * accumulated timeout for all channels in the TSG. In particular,
	 * this resets timeout for channels that already completed their work
	 */
	if (progress) {
		gk20a_dbg_info("progress on tsg=%d ch=%d",
				tsg->tsgid, ch->hw_chid);
		gk20a_channel_put(ch);
		*ms = GRFIFO_TIMEOUT_CHECK_PERIOD_US / 1000;
		list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
			if (gk20a_channel_get(ch)) {
				ch->timeout_accumulated_ms = *ms;
				gk20a_channel_put(ch);
			}
		}
	}

	/* if one channel is presumed dead (no progress for too long), then
	 * fifo recovery is needed. we can't really figure out which channel
	 * caused the problem, so set timeout error notifier for all channels.
	 */
	if (recover) {
		gk20a_dbg_info("timeout on tsg=%d ch=%d",
				tsg->tsgid, ch->hw_chid);
		*ms = ch->timeout_accumulated_ms;
		gk20a_channel_put(ch);
		list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
			if (gk20a_channel_get(ch)) {
				gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT);
				*verbose |= ch->timeout_debug_dump;
				gk20a_channel_put(ch);
			}
		}
	}

	/* if we could not detect progress on any of the channel, but none
	 * of them has reached the timeout, there is nothing more to do:
	 * timeout_accumulated_ms has been updated for all of them.
	 */
	up_read(&tsg->ch_list_lock);
	return recover;
}

static bool gk20a_fifo_handle_sched_error(struct gk20a *g)
{
	u32 sched_error;
	u32 engine_id;
	int id = -1;
	bool is_tsg = false;
	bool ret = false;

	/* read the scheduler error register */
	sched_error = gk20a_readl(g, fifo_intr_sched_error_r());

	engine_id = gk20a_fifo_get_failing_engine_data(g, &id, &is_tsg);

	/* could not find the engine - should never happen */
	if (!gk20a_fifo_is_valid_engine_id(g, engine_id)) {
		gk20a_err(dev_from_gk20a(g), "fifo sched error : 0x%08x, failed to find engine\n",
			sched_error);
		ret = false;
		goto err;
	}

	if (fifo_intr_sched_error_code_f(sched_error) ==
			fifo_intr_sched_error_code_ctxsw_timeout_v()) {
		struct fifo_gk20a *f = &g->fifo;
		u32 ms = 0;
		bool verbose = false;

		if (is_tsg) {
			ret = gk20a_fifo_check_tsg_ctxsw_timeout(
					&f->tsg[id], &verbose, &ms);
		} else {
			ret = gk20a_fifo_check_ch_ctxsw_timeout(
					&f->channel[id], &verbose, &ms);
		}

		if (ret) {
			gk20a_err(dev_from_gk20a(g),
				"fifo sched ctxsw timeout error: "
				"engine=%u, %s=%d, ms=%u",
				engine_id, is_tsg ? "tsg" : "ch", id, ms);
			/*
			 * Cancel all channels' timeout since SCHED error might
			 * trigger multiple watchdogs at a time
			 */
			gk20a_channel_timeout_restart_all_channels(g);
			gk20a_fifo_recover(g, BIT(engine_id), id,
					is_tsg, true, verbose);
		} else {
			gk20a_dbg_info(
				"fifo is waiting for ctx switch for %d ms, "
				"%s=%d", ms, is_tsg ? "tsg" : "ch", id);
		}
	} else {
		gk20a_err(dev_from_gk20a(g),
			"fifo sched error : 0x%08x, engine=%u, %s=%d",
			sched_error, engine_id, is_tsg ? "tsg" : "ch", id);
	}

err:
	return ret;
}

static u32 fifo_error_isr(struct gk20a *g, u32 fifo_intr)
{
	bool print_channel_reset_log = false;
	struct device *dev = dev_from_gk20a(g);
	u32 handled = 0;

	gk20a_dbg_fn("fifo_intr=0x%08x", fifo_intr);

	if (fifo_intr & fifo_intr_0_pio_error_pending_f()) {
		/* pio mode is unused.  this shouldn't happen, ever. */
		/* should we clear it or just leave it pending? */
		gk20a_err(dev, "fifo pio error!\n");
		BUG_ON(1);
	}

	if (fifo_intr & fifo_intr_0_bind_error_pending_f()) {
		u32 bind_error = gk20a_readl(g, fifo_intr_bind_error_r());
		gk20a_err(dev, "fifo bind error: 0x%08x", bind_error);
		print_channel_reset_log = true;
		handled |= fifo_intr_0_bind_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_sched_error_pending_f()) {
		print_channel_reset_log = gk20a_fifo_handle_sched_error(g);
		handled |= fifo_intr_0_sched_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_chsw_error_pending_f()) {
		gk20a_fifo_handle_chsw_fault(g);
		handled |= fifo_intr_0_chsw_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_mmu_fault_pending_f()) {
		print_channel_reset_log =
			gk20a_fifo_handle_mmu_fault(g, 0, ~(u32)0, false);
		handled |= fifo_intr_0_mmu_fault_pending_f();
	}

	if (fifo_intr & fifo_intr_0_dropped_mmu_fault_pending_f()) {
		gk20a_fifo_handle_dropped_mmu_fault(g);
		handled |= fifo_intr_0_dropped_mmu_fault_pending_f();
	}

	print_channel_reset_log = !g->fifo.deferred_reset_pending
			&& print_channel_reset_log;

	if (print_channel_reset_log) {
		unsigned int engine_id;
		gk20a_err(dev_from_gk20a(g),
			   "channel reset initiated from %s; intr=0x%08x",
			   __func__, fifo_intr);
		for (engine_id = 0;
		     engine_id < g->fifo.num_engines;
		     engine_id++) {
				u32 active_engine_id = g->fifo.active_engines_list[engine_id];
				u32 engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
				gk20a_dbg_fn("enum:%d -> engine_id:%d", engine_enum,
					active_engine_id);
				fifo_pbdma_exception_status(g,
						&g->fifo.engine_info[active_engine_id]);
				fifo_engine_exception_status(g,
						&g->fifo.engine_info[active_engine_id]);
		}
	}

	return handled;
}

static inline void gk20a_fifo_reset_pbdma_header(struct gk20a *g, int pbdma_id)
{
	gk20a_writel(g, pbdma_pb_header_r(pbdma_id),
			pbdma_pb_header_first_true_f() |
			pbdma_pb_header_type_non_inc_f());
}

static inline void gk20a_fifo_reset_pbdma_method(struct gk20a *g, int pbdma_id,
						int pbdma_method_index)
{
	u32 pbdma_method_stride;
	u32 pbdma_method_reg;

	pbdma_method_stride = pbdma_method1_r(pbdma_id) -
				pbdma_method0_r(pbdma_id);

	pbdma_method_reg = pbdma_method0_r(pbdma_id) +
		(pbdma_method_index * pbdma_method_stride);

	gk20a_writel(g, pbdma_method_reg,
			pbdma_method0_valid_true_f() |
			pbdma_method0_first_true_f() |
			pbdma_method0_addr_f(
			     pbdma_udma_nop_r() >> 2));
}

static bool gk20a_fifo_is_sw_method_subch(struct gk20a *g, int pbdma_id,
						int pbdma_method_index)
{
	u32 pbdma_method_stride;
	u32 pbdma_method_reg, pbdma_method_subch;

	pbdma_method_stride = pbdma_method1_r(pbdma_id) -
				pbdma_method0_r(pbdma_id);

	pbdma_method_reg = pbdma_method0_r(pbdma_id) +
			(pbdma_method_index * pbdma_method_stride);

	pbdma_method_subch = pbdma_method0_subch_v(
			gk20a_readl(g, pbdma_method_reg));

	if (pbdma_method_subch == 5 || pbdma_method_subch == 6 ||
				       pbdma_method_subch == 7)
		return true;

	return false;
}

static u32 gk20a_fifo_handle_pbdma_intr(struct device *dev,
					struct gk20a *g,
					struct fifo_gk20a *f,
					u32 pbdma_id)
{
	u32 pbdma_intr_0 = gk20a_readl(g, pbdma_intr_0_r(pbdma_id));
	u32 pbdma_intr_1 = gk20a_readl(g, pbdma_intr_1_r(pbdma_id));
	u32 status = gk20a_readl(g, fifo_pbdma_status_r(pbdma_id));

	u32 handled = 0;
	u32 error_notifier = NVGPU_CHANNEL_PBDMA_ERROR;
	bool reset = false;
	int i;

	gk20a_dbg_fn("");

	gk20a_dbg(gpu_dbg_intr, "pbdma id intr pending %d %08x %08x", pbdma_id,
			pbdma_intr_0, pbdma_intr_1);
	if (pbdma_intr_0) {
		if ((f->intr.pbdma.device_fatal_0 |
		     f->intr.pbdma.channel_fatal_0 |
		     f->intr.pbdma.restartable_0) & pbdma_intr_0) {
			gk20a_err(dev_from_gk20a(g),
				"pbdma_intr_0(%d):0x%08x PBH: %08x SHADOW: %08x M0: %08x %08x %08x %08x",
				pbdma_id, pbdma_intr_0,
				gk20a_readl(g, pbdma_pb_header_r(pbdma_id)),
				gk20a_readl(g, pbdma_hdr_shadow_r(pbdma_id)),
				gk20a_readl(g, pbdma_method0_r(pbdma_id)),
				gk20a_readl(g, pbdma_method1_r(pbdma_id)),
				gk20a_readl(g, pbdma_method2_r(pbdma_id)),
				gk20a_readl(g, pbdma_method3_r(pbdma_id))
				);
			reset = true;
			handled |= ((f->intr.pbdma.device_fatal_0 |
				     f->intr.pbdma.channel_fatal_0 |
				     f->intr.pbdma.restartable_0) &
				    pbdma_intr_0);
		}

		if (pbdma_intr_0 & pbdma_intr_0_acquire_pending_f()) {
			u32 val = gk20a_readl(g, pbdma_acquire_r(pbdma_id));
			val &= ~pbdma_acquire_timeout_en_enable_f();
			gk20a_writel(g, pbdma_acquire_r(pbdma_id), val);
			if (g->timeouts_enabled) {
				reset = true;
				gk20a_err(dev_from_gk20a(g),
					"semaphore acquire timeout!");
			}
			handled |= pbdma_intr_0_acquire_pending_f();
		}

		if (pbdma_intr_0 & pbdma_intr_0_pbentry_pending_f()) {
			gk20a_fifo_reset_pbdma_header(g, pbdma_id);
			gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
			reset = true;
		}

		if (pbdma_intr_0 & pbdma_intr_0_method_pending_f()) {
			gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
			reset = true;
		}

		if (pbdma_intr_0 & pbdma_intr_0_pbcrc_pending_f()) {
			error_notifier =
				NVGPU_CHANNEL_PBDMA_PUSHBUFFER_CRC_MISMATCH;
			reset = true;
		}

		if (pbdma_intr_0 & pbdma_intr_0_device_pending_f()) {
			gk20a_fifo_reset_pbdma_header(g, pbdma_id);

			for (i = 0; i < 4; i++) {
				if (gk20a_fifo_is_sw_method_subch(g,
						pbdma_id, i))
					gk20a_fifo_reset_pbdma_method(g,
							pbdma_id, i);
			}
			reset = true;
		}

		gk20a_writel(g, pbdma_intr_0_r(pbdma_id), pbdma_intr_0);
	}

	/* all intrs in _intr_1 are "host copy engine" related,
	 * which gk20a doesn't have. for now just make them channel fatal. */
	if (pbdma_intr_1) {
		dev_err(dev, "channel hce error: pbdma_intr_1(%d): 0x%08x",
			pbdma_id, pbdma_intr_1);
		reset = true;
		gk20a_writel(g, pbdma_intr_1_r(pbdma_id), pbdma_intr_1);
	}

	if (reset) {
		/* Remove the channel from runlist */
		u32 id = fifo_pbdma_status_id_v(status);
		if (fifo_pbdma_status_id_type_v(status)
				== fifo_pbdma_status_id_type_chid_v()) {
			struct channel_gk20a *ch = &f->channel[id];

			if (gk20a_channel_get(ch)) {
				gk20a_set_error_notifier(ch, error_notifier);
				gk20a_fifo_recover_ch(g, id, true);
				gk20a_channel_put(ch);
			}
		} else if (fifo_pbdma_status_id_type_v(status)
				== fifo_pbdma_status_id_type_tsgid_v()) {
			struct tsg_gk20a *tsg = &f->tsg[id];
			struct channel_gk20a *ch = NULL;

			down_read(&tsg->ch_list_lock);
			list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
				if (gk20a_channel_get(ch)) {
					gk20a_set_error_notifier(ch,
						error_notifier);
					gk20a_channel_put(ch);
				}
			}
			up_read(&tsg->ch_list_lock);
			gk20a_fifo_recover_tsg(g, id, true);
		}
	}

	return handled;
}

static u32 fifo_pbdma_isr(struct gk20a *g, u32 fifo_intr)
{
	struct device *dev = dev_from_gk20a(g);
	struct fifo_gk20a *f = &g->fifo;
	u32 clear_intr = 0, i;
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);
	u32 pbdma_pending = gk20a_readl(g, fifo_intr_pbdma_id_r());

	for (i = 0; i < host_num_pbdma; i++) {
		if (fifo_intr_pbdma_id_status_v(pbdma_pending, i)) {
			gk20a_dbg(gpu_dbg_intr, "pbdma id %d intr pending", i);
			clear_intr |=
				gk20a_fifo_handle_pbdma_intr(dev, g, f, i);
		}
	}
	return fifo_intr_0_pbdma_intr_pending_f();
}

void gk20a_fifo_isr(struct gk20a *g)
{
	u32 error_intr_mask =
		fifo_intr_0_bind_error_pending_f() |
		fifo_intr_0_sched_error_pending_f() |
		fifo_intr_0_chsw_error_pending_f() |
		fifo_intr_0_fb_flush_timeout_pending_f() |
		fifo_intr_0_dropped_mmu_fault_pending_f() |
		fifo_intr_0_mmu_fault_pending_f() |
		fifo_intr_0_lb_error_pending_f() |
		fifo_intr_0_pio_error_pending_f();

	u32 fifo_intr = gk20a_readl(g, fifo_intr_0_r());
	u32 clear_intr = 0;

	if (g->fifo.sw_ready) {
		/* note we're not actually in an "isr", but rather
		 * in a threaded interrupt context... */
		nvgpu_mutex_acquire(&g->fifo.intr.isr.mutex);

		gk20a_dbg(gpu_dbg_intr, "fifo isr %08x\n", fifo_intr);

		/* handle runlist update */
		if (fifo_intr & fifo_intr_0_runlist_event_pending_f()) {
			gk20a_fifo_handle_runlist_event(g);
			clear_intr |= fifo_intr_0_runlist_event_pending_f();
		}
		if (fifo_intr & fifo_intr_0_pbdma_intr_pending_f())
			clear_intr |= fifo_pbdma_isr(g, fifo_intr);

		if (unlikely(fifo_intr & error_intr_mask))
			clear_intr = fifo_error_isr(g, fifo_intr);

		nvgpu_mutex_release(&g->fifo.intr.isr.mutex);
	}
	gk20a_writel(g, fifo_intr_0_r(), clear_intr);

	return;
}

int gk20a_fifo_nonstall_isr(struct gk20a *g)
{
	u32 fifo_intr = gk20a_readl(g, fifo_intr_0_r());
	u32 clear_intr = 0;

	gk20a_dbg(gpu_dbg_intr, "fifo nonstall isr %08x\n", fifo_intr);

	if (fifo_intr & fifo_intr_0_channel_intr_pending_f())
		clear_intr = fifo_intr_0_channel_intr_pending_f();

	gk20a_writel(g, fifo_intr_0_r(), clear_intr);

	return gk20a_nonstall_ops_wakeup_semaphore;
}

void gk20a_fifo_issue_preempt(struct gk20a *g, u32 id, bool is_tsg)
{
	if (is_tsg)
		gk20a_writel(g, fifo_preempt_r(),
			fifo_preempt_id_f(id) |
			fifo_preempt_type_tsg_f());
	else
		gk20a_writel(g, fifo_preempt_r(),
			fifo_preempt_chid_f(id) |
			fifo_preempt_type_channel_f());
}

static int gk20a_fifo_is_preempt_pending(struct gk20a *g)
{
	struct nvgpu_timeout timeout;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -EBUSY;

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
				NVGPU_TIMER_CPU_TIMER);
	do {
		if (!(gk20a_readl(g, fifo_preempt_r()) &
				fifo_preempt_pending_true_f())) {
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired_msg(&timeout, "preempt timeout"));

	return ret;
}

static int __locked_fifo_preempt(struct gk20a *g, u32 id, bool is_tsg)
{
	u32 ret = 0;

	gk20a_dbg_fn("%d", id);

	/* issue preempt */
	gk20a_fifo_issue_preempt(g, id, is_tsg);

	gk20a_dbg_fn("%d", id);
	/* wait for preempt */
	ret = gk20a_fifo_is_preempt_pending(g);

	gk20a_dbg_fn("%d", id);
	if (ret) {
		if (is_tsg) {
			struct tsg_gk20a *tsg = &g->fifo.tsg[id];
			struct channel_gk20a *ch = NULL;

			gk20a_err(dev_from_gk20a(g),
				"preempt TSG %d timeout\n", id);

			down_read(&tsg->ch_list_lock);
			list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
				if (!gk20a_channel_get(ch))
					continue;
				gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT);
				gk20a_channel_put(ch);
			}
			up_read(&tsg->ch_list_lock);
			gk20a_fifo_recover_tsg(g, id, true);
		} else {
			struct channel_gk20a *ch = &g->fifo.channel[id];

			gk20a_err(dev_from_gk20a(g),
				"preempt channel %d timeout\n", id);

			if (gk20a_channel_get(ch)) {
				gk20a_set_error_notifier(ch,
						NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT);
				gk20a_fifo_recover_ch(g, id, true);
				gk20a_channel_put(ch);
			}
		}
	}

	return ret;
}

int gk20a_fifo_preempt_channel(struct gk20a *g, u32 hw_chid)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 i;

	gk20a_dbg_fn("%d", hw_chid);

	/* we have no idea which runlist we are using. lock all */
	for (i = 0; i < g->fifo.max_runlists; i++)
		nvgpu_mutex_acquire(&f->runlist_info[i].mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, hw_chid, false);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	for (i = 0; i < g->fifo.max_runlists; i++)
		nvgpu_mutex_release(&f->runlist_info[i].mutex);

	return ret;
}

int gk20a_fifo_preempt_tsg(struct gk20a *g, u32 tsgid)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 i;

	gk20a_dbg_fn("%d", tsgid);

	/* we have no idea which runlist we are using. lock all */
	for (i = 0; i < g->fifo.max_runlists; i++)
		nvgpu_mutex_acquire(&f->runlist_info[i].mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, tsgid, true);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	for (i = 0; i < g->fifo.max_runlists; i++)
		nvgpu_mutex_release(&f->runlist_info[i].mutex);

	return ret;
}

int gk20a_fifo_preempt(struct gk20a *g, struct channel_gk20a *ch)
{
	int err;

	if (gk20a_is_channel_marked_as_tsg(ch))
		err = g->ops.fifo.preempt_tsg(ch->g, ch->tsgid);
	else
		err = g->ops.fifo.preempt_channel(ch->g, ch->hw_chid);

	return err;
}

int gk20a_fifo_enable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info)
{
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	u32 enable;

	gk20a_dbg_fn("");

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	enable = gk20a_readl(g, fifo_sched_disable_r());
	enable &= ~(fifo_sched_disable_true_v() << eng_info->runlist_id);
	gk20a_writel(g, fifo_sched_disable_r(), enable);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	gk20a_dbg_fn("done");
	return 0;
}

int gk20a_fifo_enable_all_engine_activity(struct gk20a *g)
{
	unsigned int i;
	int err = 0, ret = 0;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		err = gk20a_fifo_enable_engine_activity(g,
				&g->fifo.engine_info[active_engine_id]);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"failed to enable engine %d activity\n", active_engine_id);
			ret = err;
		}
	}

	return ret;
}

int gk20a_fifo_disable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info,
				bool wait_for_idle)
{
	u32 gr_stat, pbdma_stat, chan_stat, eng_stat, ctx_stat;
	u32 pbdma_chid = FIFO_INVAL_CHANNEL_ID;
	u32 engine_chid = FIFO_INVAL_CHANNEL_ID, disable;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	u32 err = 0;

	gk20a_dbg_fn("");

	gr_stat =
		gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	if (fifo_engine_status_engine_v(gr_stat) ==
	    fifo_engine_status_engine_busy_v() && !wait_for_idle)
		return -EBUSY;

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	disable = gk20a_readl(g, fifo_sched_disable_r());
	disable = set_field(disable,
			fifo_sched_disable_runlist_m(eng_info->runlist_id),
			fifo_sched_disable_runlist_f(fifo_sched_disable_true_v(),
				eng_info->runlist_id));
	gk20a_writel(g, fifo_sched_disable_r(), disable);

	/* chid from pbdma status */
	pbdma_stat = gk20a_readl(g, fifo_pbdma_status_r(eng_info->pbdma_id));
	chan_stat  = fifo_pbdma_status_chan_status_v(pbdma_stat);
	if (chan_stat == fifo_pbdma_status_chan_status_valid_v() ||
	    chan_stat == fifo_pbdma_status_chan_status_chsw_save_v())
		pbdma_chid = fifo_pbdma_status_id_v(pbdma_stat);
	else if (chan_stat == fifo_pbdma_status_chan_status_chsw_load_v() ||
		 chan_stat == fifo_pbdma_status_chan_status_chsw_switch_v())
		pbdma_chid = fifo_pbdma_status_next_id_v(pbdma_stat);

	if (pbdma_chid != FIFO_INVAL_CHANNEL_ID) {
		err = g->ops.fifo.preempt_channel(g, pbdma_chid);
		if (err)
			goto clean_up;
	}

	/* chid from engine status */
	eng_stat = gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	ctx_stat  = fifo_engine_status_ctx_status_v(eng_stat);
	if (ctx_stat == fifo_engine_status_ctx_status_valid_v() ||
	    ctx_stat == fifo_engine_status_ctx_status_ctxsw_save_v())
		engine_chid = fifo_engine_status_id_v(eng_stat);
	else if (ctx_stat == fifo_engine_status_ctx_status_ctxsw_load_v() ||
		 ctx_stat == fifo_engine_status_ctx_status_ctxsw_switch_v())
		engine_chid = fifo_engine_status_next_id_v(eng_stat);

	if (engine_chid != FIFO_INVAL_ENGINE_ID && engine_chid != pbdma_chid) {
		err = g->ops.fifo.preempt_channel(g, engine_chid);
		if (err)
			goto clean_up;
	}

clean_up:
	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	if (err) {
		gk20a_dbg_fn("failed");
		if (gk20a_fifo_enable_engine_activity(g, eng_info))
			gk20a_err(dev_from_gk20a(g),
				"failed to enable gr engine activity\n");
	} else {
		gk20a_dbg_fn("done");
	}
	return err;
}

int gk20a_fifo_disable_all_engine_activity(struct gk20a *g,
				bool wait_for_idle)
{
	unsigned int i;
	int err = 0, ret = 0;
	u32 active_engine_id;

	for (i = 0; i < g->fifo.num_engines; i++) {
		active_engine_id = g->fifo.active_engines_list[i];
		err = gk20a_fifo_disable_engine_activity(g,
				&g->fifo.engine_info[active_engine_id],
				wait_for_idle);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
				"failed to disable engine %d activity\n", active_engine_id);
			ret = err;
			break;
		}
	}

	if (err) {
		while (i-- != 0) {
			active_engine_id = g->fifo.active_engines_list[i];
			err = gk20a_fifo_enable_engine_activity(g,
						&g->fifo.engine_info[active_engine_id]);
			if (err)
				gk20a_err(dev_from_gk20a(g),
				 "failed to re-enable engine %d activity\n", active_engine_id);
		}
	}

	return ret;
}

static void gk20a_fifo_runlist_reset_engines(struct gk20a *g, u32 runlist_id)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 engines = 0;
	unsigned int i;

	for (i = 0; i < f->num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		u32 status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		bool engine_busy = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();

		if (engine_busy &&
		    (f->engine_info[active_engine_id].runlist_id == runlist_id))
			engines |= BIT(active_engine_id);
	}

	if (engines)
		gk20a_fifo_recover(g, engines, ~(u32)0, false, false, true);
}

static int gk20a_fifo_runlist_wait_pending(struct gk20a *g, u32 runlist_id)
{
	struct fifo_runlist_info_gk20a *runlist;
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -ETIMEDOUT;

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	runlist = &g->fifo.runlist_info[runlist_id];
	do {
		if ((gk20a_readl(g, fifo_eng_runlist_r(runlist_id)) &
				fifo_eng_runlist_pending_true_f()) == 0) {
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	return ret;
}

void gk20a_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
{

	u32 runlist_entry_0 = ram_rl_entry_id_f(tsg->tsgid) |
			ram_rl_entry_type_tsg_f() |
			ram_rl_entry_tsg_length_f(tsg->num_active_channels);

	if (tsg->timeslice_timeout)
		runlist_entry_0 |=
			ram_rl_entry_timeslice_scale_f(tsg->timeslice_scale) |
			ram_rl_entry_timeslice_timeout_f(tsg->timeslice_timeout);
	else
		runlist_entry_0 |=
			ram_rl_entry_timeslice_scale_3_f() |
			ram_rl_entry_timeslice_timeout_128_f();

	runlist[0] = runlist_entry_0;
	runlist[1] = 0;

}

void gk20a_get_ch_runlist_entry(struct channel_gk20a *ch, u32 *runlist)
{
	runlist[0] = ram_rl_entry_chid_f(ch->hw_chid);
	runlist[1] = 0;
}

/* recursively construct a runlist with interleaved bare channels and TSGs */
static u32 *gk20a_runlist_construct_locked(struct fifo_gk20a *f,
				struct fifo_runlist_info_gk20a *runlist,
				u32 cur_level,
				u32 *runlist_entry,
				bool interleave_enabled,
				bool prev_empty,
				u32 *entries_left)
{
	bool last_level = cur_level == NVGPU_RUNLIST_INTERLEAVE_LEVEL_HIGH;
	struct channel_gk20a *ch;
	bool skip_next = false;
	u32 chid, tsgid, count = 0;
	u32 runlist_entry_words = f->runlist_entry_size / sizeof(u32);

	gk20a_dbg_fn("");

	/* for each bare channel, CH, on this level, insert all higher-level
	   channels and TSGs before inserting CH. */
	for_each_set_bit(chid, runlist->active_channels, f->num_channels) {
		ch = &f->channel[chid];

		if (ch->interleave_level != cur_level)
			continue;

		if (gk20a_is_channel_marked_as_tsg(ch))
			continue;

		if (!last_level && !skip_next) {
			runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							false,
							entries_left);
			/* if interleaving is disabled, higher-level channels
			   and TSGs only need to be inserted once */
			if (!interleave_enabled)
				skip_next = true;
		}

		if (!(*entries_left))
			return NULL;

		gk20a_dbg_info("add channel %d to runlist", chid);
		f->g->ops.fifo.get_ch_runlist_entry(ch, runlist_entry);
		gk20a_dbg_info("run list count %d runlist [0] %x [1] %x\n",
				count, runlist_entry[0], runlist_entry[1]);
		runlist_entry += runlist_entry_words;
		count++;
		(*entries_left)--;
	}

	/* for each TSG, T, on this level, insert all higher-level channels
	   and TSGs before inserting T. */
	for_each_set_bit(tsgid, runlist->active_tsgs, f->num_channels) {
		struct tsg_gk20a *tsg = &f->tsg[tsgid];

		if (tsg->interleave_level != cur_level)
			continue;

		if (!last_level && !skip_next) {
			runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							false,
							entries_left);
			if (!interleave_enabled)
				skip_next = true;
		}

		if (!(*entries_left))
			return NULL;

		/* add TSG entry */
		gk20a_dbg_info("add TSG %d to runlist", tsg->tsgid);
		f->g->ops.fifo.get_tsg_runlist_entry(tsg, runlist_entry);
		gk20a_dbg_info("tsg runlist count %d runlist [0] %x [1] %x\n",
				count, runlist_entry[0], runlist_entry[1]);
		runlist_entry += runlist_entry_words;
		count++;
		(*entries_left)--;

		down_read(&tsg->ch_list_lock);
		/* add runnable channels bound to this TSG */
		list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
			if (!test_bit(ch->hw_chid,
				      runlist->active_channels))
				continue;

			if (!(*entries_left)) {
				up_read(&tsg->ch_list_lock);
				return NULL;
			}

			gk20a_dbg_info("add channel %d to runlist",
				ch->hw_chid);
			f->g->ops.fifo.get_ch_runlist_entry(ch, runlist_entry);
			gk20a_dbg_info(
				"run list count %d runlist [0] %x [1] %x\n",
				count, runlist_entry[0], runlist_entry[1]);
			count++;
			runlist_entry += runlist_entry_words;
			(*entries_left)--;
		}
		up_read(&tsg->ch_list_lock);
	}

	/* append entries from higher level if this level is empty */
	if (!count && !last_level)
		runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							true,
							entries_left);

	/*
	 * if previous and this level have entries, append
	 * entries from higher level.
	 *
	 * ex. dropping from MEDIUM to LOW, need to insert HIGH
	 */
	if (interleave_enabled && count && !prev_empty && !last_level)
		runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							false,
							entries_left);
	return runlist_entry;
}

int gk20a_fifo_set_runlist_interleave(struct gk20a *g,
				u32 id,
				bool is_tsg,
				u32 runlist_id,
				u32 new_level)
{
	gk20a_dbg_fn("");

	if (is_tsg)
		g->fifo.tsg[id].interleave_level = new_level;
	else
		g->fifo.channel[id].interleave_level = new_level;

	return 0;
}

int gk20a_fifo_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct gk20a *g = tsg->g;

	if (timeslice < g->min_timeslice_us ||
		timeslice > g->max_timeslice_us)
		return -EINVAL;

	gk20a_channel_get_timescale_from_timeslice(g, timeslice,
			&tsg->timeslice_timeout, &tsg->timeslice_scale);

	tsg->timeslice_us = timeslice;

	return g->ops.fifo.update_runlist(g, tsg->runlist_id, ~0, true, true);
}

static int gk20a_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					    u32 hw_chid, bool add,
					    bool wait_for_finish)
{
	int ret = 0;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 *runlist_entry_base = NULL;
	u64 runlist_iova;
	u32 old_buf, new_buf;
	struct channel_gk20a *ch = NULL;
	struct tsg_gk20a *tsg = NULL;
	u32 count = 0;
	u32 runlist_entry_words = f->runlist_entry_size / sizeof(u32);

	runlist = &f->runlist_info[runlist_id];

	/* valid channel, add/remove it from active list.
	   Otherwise, keep active list untouched for suspend/resume. */
	if (hw_chid != FIFO_INVAL_CHANNEL_ID) {
		ch = &f->channel[hw_chid];
		if (gk20a_is_channel_marked_as_tsg(ch))
			tsg = &f->tsg[ch->tsgid];

		if (add) {
			if (test_and_set_bit(hw_chid,
				runlist->active_channels) == 1)
				return 0;
			if (tsg && ++tsg->num_active_channels)
				set_bit(f->channel[hw_chid].tsgid,
					runlist->active_tsgs);
		} else {
			if (test_and_clear_bit(hw_chid,
				runlist->active_channels) == 0)
				return 0;
			if (tsg && --tsg->num_active_channels == 0)
				clear_bit(f->channel[hw_chid].tsgid,
					runlist->active_tsgs);
		}
	}

	old_buf = runlist->cur_buffer;
	new_buf = !runlist->cur_buffer;

	runlist_iova = g->ops.mm.get_iova_addr(
			g, runlist->mem[new_buf].sgt->sgl, 0);

	gk20a_dbg_info("runlist_id : %d, switch to new buffer 0x%16llx",
		runlist_id, (u64)runlist_iova);

	if (!runlist_iova) {
		ret = -EINVAL;
		goto clean_up;
	}

	runlist_entry_base = runlist->mem[new_buf].cpu_va;
	if (!runlist_entry_base) {
		ret = -ENOMEM;
		goto clean_up;
	}

	if (hw_chid != FIFO_INVAL_CHANNEL_ID || /* add/remove a valid channel */
	    add /* resume to add all channels back */) {
		u32 max_entries = f->num_runlist_entries;
		u32 *runlist_end;

		runlist_end = gk20a_runlist_construct_locked(f,
						runlist,
						0,
						runlist_entry_base,
						g->runlist_interleave,
						true,
						&max_entries);
		if (!runlist_end) {
			ret = -E2BIG;
			goto clean_up;
		}
		count = (runlist_end - runlist_entry_base) / runlist_entry_words;
		WARN_ON(count > f->num_runlist_entries);
	} else	/* suspend to remove all channels */
		count = 0;

	if (count != 0) {
		runlist->runlist_base_r =
			fifo_runlist_base_ptr_f(u64_lo32(runlist_iova >> 12)) |
			gk20a_aperture_mask(g, &runlist->mem[new_buf],
			  fifo_runlist_base_target_sys_mem_ncoh_f(),
			  fifo_runlist_base_target_vid_mem_f());
		gk20a_writel(g, fifo_runlist_base_r(), runlist->runlist_base_r);
	}

	runlist->runlist_r =
		fifo_runlist_engine_f(runlist_id) |
		fifo_eng_runlist_length_f(count);
	gk20a_writel(g, fifo_runlist_r(), runlist->runlist_r);

	if (wait_for_finish) {
		ret = gk20a_fifo_runlist_wait_pending(g, runlist_id);

		if (ret == -ETIMEDOUT) {
			gk20a_err(dev_from_gk20a(g),
				   "runlist update timeout");

			gk20a_fifo_runlist_reset_engines(g, runlist_id);

			/* engine reset needs the lock. drop it */
			/* wait until the runlist is active again */
			ret = gk20a_fifo_runlist_wait_pending(g, runlist_id);
			/* get the lock back. at this point everything should
			 * should be fine */

			if (ret)
				gk20a_err(dev_from_gk20a(g),
					   "runlist update failed: %d", ret);
		} else if (ret == -EINTR)
			gk20a_err(dev_from_gk20a(g),
				   "runlist update interrupted");
	}

	runlist->cur_buffer = new_buf;

clean_up:
	return ret;
}

int gk20a_fifo_update_runlist_ids(struct gk20a *g, u32 runlist_ids, u32 hw_chid,
				bool add, bool wait_for_finish)
{
	u32 ret = -EINVAL;
	u32 runlist_id = 0;
	u32 errcode;
	unsigned long ulong_runlist_ids = (unsigned long)runlist_ids;

	if (!g)
		goto end;

	ret = 0;
	for_each_set_bit(runlist_id, &ulong_runlist_ids, 32) {
		/* Capture the last failure error code */
		errcode = g->ops.fifo.update_runlist(g, runlist_id, hw_chid, add, wait_for_finish);
		if (errcode) {
			gk20a_err(dev_from_gk20a(g),
				"failed to update_runlist %d %d", runlist_id, errcode);
			ret = errcode;
		}
	}
end:
	return ret;
}

/* trigger host to expire current timeslice and reschedule runlist from front */
int gk20a_fifo_reschedule_runlist(struct gk20a *g, u32 runlist_id)
{
	struct fifo_runlist_info_gk20a *runlist;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	int ret = 0;

	runlist = &g->fifo.runlist_info[runlist_id];
	if (nvgpu_mutex_tryacquire(&runlist->mutex)) {
		mutex_ret = pmu_mutex_acquire(
			&g->pmu, PMU_MUTEX_ID_FIFO, &token);

		gk20a_writel(g, fifo_runlist_base_r(), runlist->runlist_base_r);
		gk20a_writel(g, fifo_runlist_r(), runlist->runlist_r);
		gk20a_fifo_runlist_wait_pending(g, runlist_id);

		if (!mutex_ret)
			pmu_mutex_release(
				&g->pmu, PMU_MUTEX_ID_FIFO, &token);
		nvgpu_mutex_release(&runlist->mutex);
	} else {
		/* someone else is writing fifo_runlist_r so not needed here */
		ret = -EBUSY;
	}
	return ret;
}

/* trigger host preempt of gr engine pending load context if that ctx is not for ch */
int gk20a_fifo_reschedule_preempt_next(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct fifo_runlist_info_gk20a *runlist;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	int ret = 0;
	u32 gr_eng_id = 0;

	runlist = &g->fifo.runlist_info[ch->runlist_id];
	if (1 != gk20a_fifo_get_engine_ids(
		g, &gr_eng_id, 1, ENGINE_GR_GK20A))
		return 0;
	if (!(runlist->eng_bitmask & (1 << gr_eng_id)))
		return 0;

	if (!nvgpu_mutex_tryacquire(&runlist->mutex))
		return -EBUSY; /* someone else is writing fifo_runlist_r so not needed here */
	mutex_ret = pmu_mutex_acquire(
		&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	do {
		u32 engstat, ctxstat, fecsstat0, fecsstat1;
		s32 preempt_id = -1;
		u32 preempt_type = 0;
		bool same_ctx;

		if (gk20a_readl(g, fifo_preempt_r()) &
			fifo_preempt_pending_true_f())
			break;

		fecsstat0 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
		engstat = gk20a_readl(g, fifo_engine_status_r(gr_eng_id));
		ctxstat = fifo_engine_status_ctx_status_v(engstat);
		if (ctxstat == fifo_engine_status_ctx_status_ctxsw_switch_v()) {
			/* host switching to new context, preempt next context if needed */
			preempt_id = fifo_engine_status_next_id_v(engstat);
			preempt_type = fifo_engine_status_next_id_type_v(engstat);
		} else {
			break;
		}
		if (gk20a_is_channel_marked_as_tsg(ch))
			same_ctx = (preempt_id == ch->tsgid && preempt_type);
		else
			same_ctx = (preempt_id == ch->hw_chid && !preempt_type);
		if (same_ctx)
			break;
		fecsstat1 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
		if (fecsstat0 != FECS_MAILBOX_0_ACK_RESTORE ||
			fecsstat1 != FECS_MAILBOX_0_ACK_RESTORE)
			break; /* preempt is useless if FECS acked save and started restore */

		gk20a_fifo_issue_preempt(g, preempt_id, preempt_type);

		trace_gk20a_reschedule_preempt_next(ch->hw_chid, fecsstat0, engstat,
			fecsstat1, gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0)),
			gk20a_readl(g, fifo_preempt_r()));

		gk20a_fifo_is_preempt_pending(g);

		trace_gk20a_reschedule_preempted_next(ch->hw_chid);

	} while (false);

	if (!mutex_ret)
		pmu_mutex_release(
			&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	nvgpu_mutex_release(&runlist->mutex);

	return ret;
}

/* add/remove a channel from runlist
   special cases below: runlist->active_channels will NOT be changed.
   (hw_chid == ~0 && !add) means remove all active channels from runlist.
   (hw_chid == ~0 &&  add) means restore all active channels on runlist. */
int gk20a_fifo_update_runlist(struct gk20a *g, u32 runlist_id, u32 hw_chid,
			      bool add, bool wait_for_finish)
{
	struct fifo_runlist_info_gk20a *runlist = NULL;
	struct fifo_gk20a *f = &g->fifo;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	u32 ret = 0;

	gk20a_dbg_fn("");

	runlist = &f->runlist_info[runlist_id];

	nvgpu_mutex_acquire(&runlist->mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = gk20a_fifo_update_runlist_locked(g, runlist_id, hw_chid, add,
					       wait_for_finish);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&runlist->mutex);
	return ret;
}

int gk20a_fifo_suspend(struct gk20a *g)
{
	gk20a_dbg_fn("");

	/* stop bar1 snooping */
	if (g->ops.mm.is_bar1_supported(g))
		gk20a_writel(g, fifo_bar1_base_r(),
			fifo_bar1_base_valid_false_f());

	/* disable fifo intr */
	gk20a_writel(g, fifo_intr_en_0_r(), 0);
	gk20a_writel(g, fifo_intr_en_1_r(), 0);

	gk20a_dbg_fn("done");
	return 0;
}

bool gk20a_fifo_mmu_fault_pending(struct gk20a *g)
{
	if (gk20a_readl(g, fifo_intr_0_r()) &
			fifo_intr_0_mmu_fault_pending_f())
		return true;
	else
		return false;
}

bool gk20a_fifo_is_engine_busy(struct gk20a *g)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		if (fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v())
			return true;
	}
	return false;
}

int gk20a_fifo_wait_engine_idle(struct gk20a *g)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -ETIMEDOUT;
	u32 i, host_num_engines;

	gk20a_dbg_fn("");

	host_num_engines =
		 nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	for (i = 0; i < host_num_engines; i++) {
		do {
			u32 status = gk20a_readl(g, fifo_engine_status_r(i));
			if (!fifo_engine_status_engine_v(status)) {
				ret = 0;
				break;
			}

			usleep_range(delay, delay * 2);
			delay = min_t(unsigned long,
					delay << 1, GR_IDLE_CHECK_MAX);
		} while (!nvgpu_timeout_expired(&timeout));

		if (ret) {
			gk20a_dbg_info("cannot idle engine %u", i);
			break;
		}
	}

	gk20a_dbg_fn("done");

	return ret;
}

static void gk20a_fifo_apply_pb_timeout(struct gk20a *g)
{
	u32 timeout;

	if (tegra_platform_is_silicon()) {
		timeout = gk20a_readl(g, fifo_pb_timeout_r());
		timeout &= ~fifo_pb_timeout_detection_enabled_f();
		gk20a_writel(g, fifo_pb_timeout_r(), timeout);
	}
}

static u32 gk20a_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

u32 gk20a_fifo_get_pbdma_signature(struct gk20a *g)
{
	return pbdma_signature_hw_valid_f() | pbdma_signature_sw_zero_f();
}

struct channel_gk20a *gk20a_fifo_channel_from_hw_chid(struct gk20a *g,
		u32 hw_chid)
{
	return g->fifo.channel + hw_chid;
}

#ifdef CONFIG_DEBUG_FS

/* Get the next element in the ring buffer of profile entries
 * and grab a reference to the structure
 */
struct fifo_profile_gk20a *gk20a_fifo_profile_acquire(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_profile_gk20a *profile;
	unsigned int index;

	/* If kref is zero, profiling is not enabled */
	if (!kref_get_unless_zero(&f->profile.ref))
		return NULL;
	index = atomic_inc_return(&f->profile.get);
	profile = &f->profile.data[index % FIFO_PROFILING_ENTRIES];

	return profile;
}

/* Free the reference to the structure. This allows deferred cleanups */
void gk20a_fifo_profile_release(struct gk20a *g,
					struct fifo_profile_gk20a *profile)
{
	kref_put(&g->fifo.profile.ref, __gk20a_fifo_profile_free);
}

static void *gk20a_fifo_sched_debugfs_seq_start(
		struct seq_file *s, loff_t *pos)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;

	if (*pos >= f->num_channels)
		return NULL;

	return &f->channel[*pos];
}

static void *gk20a_fifo_sched_debugfs_seq_next(
		struct seq_file *s, void *v, loff_t *pos)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;

	++(*pos);
	if (*pos >= f->num_channels)
		return NULL;

	return &f->channel[*pos];
}

static void gk20a_fifo_sched_debugfs_seq_stop(
		struct seq_file *s, void *v)
{
}

static int gk20a_fifo_sched_debugfs_seq_show(
		struct seq_file *s, void *v)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = v;
	struct tsg_gk20a *tsg = NULL;

	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_runlist_info_gk20a *runlist;
	u32 runlist_id;
	int ret = SEQ_SKIP;
	u32 engine_id;

	engine_id = gk20a_fifo_get_gr_engine_id(g);
	engine_info = (f->engine_info + engine_id);
	runlist_id = engine_info->runlist_id;
	runlist = &f->runlist_info[runlist_id];

	if (ch == f->channel) {
		seq_puts(s, "chid     tsgid    pid      timeslice  timeout  interleave graphics_preempt compute_preempt\n");
		seq_puts(s, "                            (usecs)   (msecs)\n");
		ret = 0;
	}

	if (!test_bit(ch->hw_chid, runlist->active_channels))
		return ret;

	if (gk20a_channel_get(ch)) {
		if (gk20a_is_channel_marked_as_tsg(ch))
			tsg = &f->tsg[ch->tsgid];

		seq_printf(s, "%-8d %-8d %-8d %-9d %-8d %-10d %-8d %-8d\n",
				ch->hw_chid,
				ch->tsgid,
				ch->tgid,
				tsg ? tsg->timeslice_us : ch->timeslice_us,
				ch->timeout_ms_max,
				tsg ? tsg->interleave_level : ch->interleave_level,
				ch->ch_ctx.gr_ctx ? ch->ch_ctx.gr_ctx->graphics_preempt_mode : U32_MAX,
				ch->ch_ctx.gr_ctx ? ch->ch_ctx.gr_ctx->compute_preempt_mode : U32_MAX);
		gk20a_channel_put(ch);
	}
	return 0;
}

static const struct seq_operations gk20a_fifo_sched_debugfs_seq_ops = {
	.start = gk20a_fifo_sched_debugfs_seq_start,
	.next = gk20a_fifo_sched_debugfs_seq_next,
	.stop = gk20a_fifo_sched_debugfs_seq_stop,
	.show = gk20a_fifo_sched_debugfs_seq_show
};

static int gk20a_fifo_sched_debugfs_open(struct inode *inode,
	struct file *file)
{
	int err;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	err = seq_open(file, &gk20a_fifo_sched_debugfs_seq_ops);
	if (err)
		return err;

	gk20a_dbg(gpu_dbg_info, "i_private=%p", inode->i_private);

	((struct seq_file *)file->private_data)->private = inode->i_private;
	return 0;
};

/*
 * The file operations structure contains our open function along with
 * set of the canned seq_ ops.
 */
static const struct file_operations gk20a_fifo_sched_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = gk20a_fifo_sched_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static void __gk20a_fifo_profile_free(struct kref *ref)
{
	struct fifo_gk20a *f = container_of(ref, struct fifo_gk20a,
						profile.ref);
	vfree(f->profile.data);
	vfree(f->profile.sorted);
}

static int gk20a_fifo_profile_enable(void *data, u64 val)
{
	struct gk20a *g = (struct gk20a *) data;
	struct fifo_gk20a *f = &g->fifo;


	nvgpu_mutex_acquire(&f->profile.lock);
	if (val == 0) {
		if (f->profile.enabled) {
			f->profile.enabled = false;
			kref_put(&f->profile.ref, __gk20a_fifo_profile_free);
		}
	} else {
		if (!f->profile.enabled) {
			/* not kref init as it can have a running condition if
			 * we enable/disable/enable while kickoff is happening
			 */
			if (!kref_get_unless_zero(&f->profile.ref)) {
				f->profile.data = vzalloc(
							FIFO_PROFILING_ENTRIES *
					sizeof(struct fifo_profile_gk20a));
				f->profile.sorted  = vzalloc(
							FIFO_PROFILING_ENTRIES *
							sizeof(u64));
				if (!(f->profile.data && f->profile.sorted)) {
					vfree(f->profile.data);
					vfree(f->profile.sorted);
					nvgpu_mutex_release(&f->profile.lock);
					return -ENOMEM;
				}
				kref_init(&f->profile.ref);
			}
			atomic_set(&f->profile.get, 0);
			f->profile.enabled = true;
		}
	}
	nvgpu_mutex_release(&f->profile.lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(
	gk20a_fifo_profile_enable_debugfs_fops,
	NULL,
	gk20a_fifo_profile_enable,
	"%llu\n"
);

static int __profile_cmp(const void *a, const void *b)
{
	return *((unsigned long long *) a) - *((unsigned long long *) b);
}

/*
 * This uses about 800b in the stack, but the function using it is not part
 * of a callstack where much memory is being used, so it is fine
 */
#define PERCENTILE_WIDTH	5
#define PERCENTILE_RANGES	(100/PERCENTILE_WIDTH)

static unsigned int __gk20a_fifo_create_stats(struct gk20a *g,
		u64 *percentiles, u32 index_end, u32 index_start)
{
	unsigned int nelem = 0;
	unsigned int index;
	struct fifo_profile_gk20a *profile;

	for (index = 0; index < FIFO_PROFILING_ENTRIES; index++) {
		profile = &g->fifo.profile.data[index];

		if (profile->timestamp[index_end] >
				profile->timestamp[index_start]) {
			/* This is a valid element */
			g->fifo.profile.sorted[nelem] =
						profile->timestamp[index_end] -
						profile->timestamp[index_start];
			nelem++;
		}
	}

	/* sort it */
	sort(g->fifo.profile.sorted, nelem, sizeof(unsigned long long),
		__profile_cmp, NULL);

	/* build ranges */
	for (index = 0; index < PERCENTILE_RANGES; index++)
		percentiles[index] =
			g->fifo.profile.sorted[(PERCENTILE_WIDTH * index *
						nelem)/100];
	return nelem;
}

static int gk20a_fifo_profile_stats(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	unsigned int get, nelem, index;
	/*
	 * 800B in the stack, but function is declared statically and only
	 * called from debugfs handler
	 */
	u64 percentiles_ioctl[PERCENTILE_RANGES];
	u64 percentiles_kickoff[PERCENTILE_RANGES];
	u64 percentiles_jobtracking[PERCENTILE_RANGES];
	u64 percentiles_append[PERCENTILE_RANGES];
	u64 percentiles_userd[PERCENTILE_RANGES];

	if (!kref_get_unless_zero(&g->fifo.profile.ref)) {
		seq_printf(s, "Profiling disabled\n");
		return 0;
	}

	get = atomic_read(&g->fifo.profile.get);

	__gk20a_fifo_create_stats(g, percentiles_ioctl,
		PROFILE_IOCTL_EXIT, PROFILE_IOCTL_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_kickoff,
		PROFILE_END, PROFILE_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_jobtracking,
		PROFILE_JOB_TRACKING, PROFILE_IOCTL_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_append,
		PROFILE_APPEND, PROFILE_JOB_TRACKING);
	nelem = __gk20a_fifo_create_stats(g, percentiles_userd,
		PROFILE_END, PROFILE_APPEND);

	seq_printf(s, "Number of kickoffs: %d\n", nelem);
	seq_printf(s, "Perc \t ioctl(ns) \t kickoff(ns) \t pbcopy(ns) \t jobtrack(ns) \t userd(ns)\n");

	for (index = 0; index < PERCENTILE_RANGES; index++)
		seq_printf(s, "[%2dpc]\t%8lld\t%8lld\t%8lld\t%8lld\t%8lld\n",
			PERCENTILE_WIDTH * (index+1),
			percentiles_ioctl[index],
			percentiles_kickoff[index],
			percentiles_append[index],
			percentiles_jobtracking[index],
			percentiles_userd[index]);

	kref_put(&g->fifo.profile.ref, __gk20a_fifo_profile_free);

	return 0;
}

static int gk20a_fifo_profile_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_fifo_profile_stats, inode->i_private);
}

static const struct file_operations gk20a_fifo_profile_stats_debugfs_fops = {
	.open		= gk20a_fifo_profile_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


void gk20a_fifo_debugfs_init(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a *g = get_gk20a(dev);

	struct dentry *gpu_root = platform->debugfs;
	struct dentry *fifo_root;
	struct dentry *profile_root;


	fifo_root = debugfs_create_dir("fifo", gpu_root);
	if (IS_ERR_OR_NULL(fifo_root))
		return;

	gk20a_dbg(gpu_dbg_info, "g=%p", g);

	debugfs_create_file("sched", 0600, fifo_root, g,
		&gk20a_fifo_sched_debugfs_fops);

	profile_root = debugfs_create_dir("profile", fifo_root);
	if (IS_ERR_OR_NULL(profile_root))
		return;

	nvgpu_mutex_init(&g->fifo.profile.lock);
	g->fifo.profile.enabled = false;
	atomic_set(&g->fifo.profile.get, 0);
	atomic_set(&g->fifo.profile.ref.refcount, 0);

	debugfs_create_file("enable", 0600, profile_root, g,
		&gk20a_fifo_profile_enable_debugfs_fops);

	debugfs_create_file("stats", 0600, profile_root, g,
		&gk20a_fifo_profile_stats_debugfs_fops);

}
#endif /* CONFIG_DEBUG_FS */

static const char * const ccsr_chan_status_str[] = {
	"idle",
	"pending",
	"pending_ctx_reload",
	"pending_acquire",
	"pending_acq_ctx_reload",
	"on_pbdma",
	"on_pbdma_and_eng",
	"on_eng",
	"on_eng_pending_acquire",
	"on_eng_pending",
	"on_pbdma_ctx_reload",
	"on_pbdma_and_eng_ctx_reload",
	"on_eng_ctx_reload",
	"on_eng_pending_ctx_reload",
	"on_eng_pending_acq_ctx_reload",
};

const char * const pbdma_chan_eng_ctx_status_str[] = {
	"invalid",
	"valid",
	"NA",
	"NA",
	"NA",
	"load",
	"save",
	"switch",
};

static const char * const not_found_str[] = {
	"NOT FOUND"
};

const char *gk20a_decode_ccsr_chan_status(u32 index)
{
	if (index >= ARRAY_SIZE(ccsr_chan_status_str))
		return not_found_str[0];
	else
		return ccsr_chan_status_str[index];
}

const char *gk20a_decode_pbdma_chan_eng_ctx_status(u32 index)
{
	if (index >= ARRAY_SIZE(pbdma_chan_eng_ctx_status_str))
		return not_found_str[0];
	else
		return pbdma_chan_eng_ctx_status_str[index];
}

void gk20a_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 hw_chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(hw_chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 syncpointa, syncpointb;
	u32 *inst_mem;
	struct channel_gk20a *c = g->fifo.channel + hw_chid;
	struct nvgpu_semaphore_int *hw_sema = NULL;

	if (c->hw_sema)
		hw_sema = c->hw_sema;

	if (!ch_state)
		return;

	inst_mem = &ch_state->inst_block[0];

	syncpointa = inst_mem[ram_fc_syncpointa_w()];
	syncpointb = inst_mem[ram_fc_syncpointb_w()];

	gk20a_debug_output(o, "%d-%s, pid %d, refs: %d: ", hw_chid,
			dev_name(g->dev),
			ch_state->pid,
			ch_state->refs);
	gk20a_debug_output(o, "channel status: %s in use %s %s\n",
			ccsr_channel_enable_v(channel) ? "" : "not",
			gk20a_decode_ccsr_chan_status(status),
			ccsr_channel_busy_v(channel) ? "busy" : "not busy");
	gk20a_debug_output(o, "RAMFC : TOP: %016llx PUT: %016llx GET: %016llx "
			"FETCH: %016llx\nHEADER: %08x COUNT: %08x\n"
			"SYNCPOINT %08x %08x SEMAPHORE %08x %08x %08x %08x\n",
		(u64)inst_mem[ram_fc_pb_top_level_get_w()] +
		((u64)inst_mem[ram_fc_pb_top_level_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_put_w()] +
		((u64)inst_mem[ram_fc_pb_put_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_get_w()] +
		((u64)inst_mem[ram_fc_pb_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_fetch_w()] +
		((u64)inst_mem[ram_fc_pb_fetch_hi_w()] << 32ULL),
		inst_mem[ram_fc_pb_header_w()],
		inst_mem[ram_fc_pb_count_w()],
		syncpointa,
		syncpointb,
		inst_mem[ram_fc_semaphorea_w()],
		inst_mem[ram_fc_semaphoreb_w()],
		inst_mem[ram_fc_semaphorec_w()],
		inst_mem[ram_fc_semaphored_w()]);
	if (hw_sema)
		gk20a_debug_output(o, "SEMA STATE: value: 0x%08x "
				   "next_val: 0x%08x addr: 0x%010llx\n",
				   readl(hw_sema->value),
				   atomic_read(&hw_sema->next_value),
				   nvgpu_hw_sema_addr(hw_sema));

#ifdef CONFIG_TEGRA_GK20A
	if ((pbdma_syncpointb_op_v(syncpointb) == pbdma_syncpointb_op_wait_v())
		&& (pbdma_syncpointb_wait_switch_v(syncpointb) ==
			pbdma_syncpointb_wait_switch_en_v()))
		gk20a_debug_output(o, "%s on syncpt %u (%s) val %u\n",
			(status == 3 || status == 8) ? "Waiting" : "Waited",
			pbdma_syncpointb_syncpt_index_v(syncpointb),
			nvhost_syncpt_get_name(g->host1x_dev,
				pbdma_syncpointb_syncpt_index_v(syncpointb)),
			pbdma_syncpointa_payload_v(syncpointa));
#endif

	gk20a_debug_output(o, "\n");
}

void gk20a_dump_pbdma_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_pbdma;

	host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	for (i = 0; i < host_num_pbdma; i++) {
		u32 status = gk20a_readl(g, fifo_pbdma_status_r(i));
		u32 chan_status = fifo_pbdma_status_chan_status_v(status);

		gk20a_debug_output(o, "%s pbdma %d: ", dev_name(g->dev), i);
		gk20a_debug_output(o,
				"id: %d (%s), next_id: %d (%s) chan status: %s\n",
				fifo_pbdma_status_id_v(status),
				fifo_pbdma_status_id_type_v(status) ?
					"tsg" : "channel",
				fifo_pbdma_status_next_id_v(status),
				fifo_pbdma_status_next_id_type_v(status) ?
					"tsg" : "channel",
			gk20a_decode_pbdma_chan_eng_ctx_status(chan_status));
		gk20a_debug_output(o, "PUT: %016llx GET: %016llx "
				"FETCH: %08x HEADER: %08x\n",
			(u64)gk20a_readl(g, pbdma_put_r(i)) +
			((u64)gk20a_readl(g, pbdma_put_hi_r(i)) << 32ULL),
			(u64)gk20a_readl(g, pbdma_get_r(i)) +
			((u64)gk20a_readl(g, pbdma_get_hi_r(i)) << 32ULL),
			gk20a_readl(g, pbdma_gp_fetch_r(i)),
			gk20a_readl(g, pbdma_pb_header_r(i)));
	}
	gk20a_debug_output(o, "\n");
}

void gk20a_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);

		gk20a_debug_output(o, "%s eng %d: ", dev_name(g->dev), i);
		gk20a_debug_output(o,
			"id: %d (%s), next_id: %d (%s), ctx status: %s ",
			fifo_engine_status_id_v(status),
			fifo_engine_status_id_type_v(status) ?
				"tsg" : "channel",
			fifo_engine_status_next_id_v(status),
			fifo_engine_status_next_id_type_v(status) ?
				"tsg" : "channel",
			gk20a_decode_pbdma_chan_eng_ctx_status(ctx_status));

		if (fifo_engine_status_faulted_v(status))
			gk20a_debug_output(o, "faulted ");
		if (fifo_engine_status_engine_v(status))
			gk20a_debug_output(o, "busy ");
		gk20a_debug_output(o, "\n");
	}
	gk20a_debug_output(o, "\n");
}

void gk20a_init_fifo(struct gpu_ops *gops)
{
	gk20a_init_channel(gops);
	gops->fifo.init_fifo_setup_hw = gk20a_init_fifo_setup_hw;
	gops->fifo.preempt_channel = gk20a_fifo_preempt_channel;
	gops->fifo.preempt_tsg = gk20a_fifo_preempt_tsg;
	gops->fifo.update_runlist = gk20a_fifo_update_runlist;
	gops->fifo.trigger_mmu_fault = gk20a_fifo_trigger_mmu_fault;
	gops->fifo.apply_pb_timeout = gk20a_fifo_apply_pb_timeout;
	gops->fifo.wait_engine_idle = gk20a_fifo_wait_engine_idle;
	gops->fifo.get_num_fifos = gk20a_fifo_get_num_fifos;
	gops->fifo.get_pbdma_signature = gk20a_fifo_get_pbdma_signature;
	gops->fifo.set_runlist_interleave = gk20a_fifo_set_runlist_interleave;
	gops->fifo.tsg_set_timeslice = gk20a_fifo_tsg_set_timeslice;
	gops->fifo.force_reset_ch = gk20a_fifo_force_reset_ch;
	gops->fifo.engine_enum_from_type = gk20a_fifo_engine_enum_from_type;
	/* gk20a doesn't support device_info_data packet parsing */
	gops->fifo.device_info_data_parse = NULL;
	gops->fifo.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v;
	gops->fifo.init_engine_info = gk20a_fifo_init_engine_info;
	gops->fifo.runlist_entry_size = ram_rl_entry_size_v;
	gops->fifo.get_tsg_runlist_entry = gk20a_get_tsg_runlist_entry;
	gops->fifo.get_ch_runlist_entry = gk20a_get_ch_runlist_entry;
	gops->fifo.is_fault_engine_subid_gpc = gk20a_is_fault_engine_subid_gpc;
	gops->fifo.dump_pbdma_status = gk20a_dump_pbdma_status;
	gops->fifo.dump_eng_status = gk20a_dump_eng_status;
	gops->fifo.dump_channel_status_ramfc = gk20a_dump_channel_status_ramfc;
}
