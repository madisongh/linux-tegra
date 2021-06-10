/*
 * drivers/video/tegra/host/gk20a/fifo_gk20a.h
 *
 * GK20A graphics fifo (gr host)
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __FIFO_GK20A_H__
#define __FIFO_GK20A_H__

#include "channel_gk20a.h"
#include "tsg_gk20a.h"
#include "debug_gk20a.h"

#define MAX_RUNLIST_BUFFERS	2

#define FIFO_INVAL_ENGINE_ID	((u32)~0)
#define FIFO_INVAL_CHANNEL_ID	((u32)~0)
#define FIFO_INVAL_TSG_ID	((u32)~0)

/*
 * Number of entries in the kickoff latency buffer, used to calculate
 * the profiling and histogram. This number is calculated to be statistically
 * significative on a histogram on a 5% step
 */
#ifdef CONFIG_DEBUG_FS
#define FIFO_PROFILING_ENTRIES	16384
#endif

/* generally corresponds to the "pbdma" engine */

struct fifo_runlist_info_gk20a {
	unsigned long *active_channels;
	unsigned long *active_tsgs;
	/* Each engine has its own SW and HW runlist buffer.*/
	struct mem_desc mem[MAX_RUNLIST_BUFFERS];
	u32  cur_buffer;
	u32  total_entries;
	u32  pbdma_bitmask;      /* pbdmas supported for this runlist*/
	u32  eng_bitmask;        /* engines using this runlist */
	u32  reset_eng_bitmask;  /* engines to be reset during recovery */
	u32  runlist_base_r;     /* cached runlist_base_r */
	u32  runlist_r;          /* cached runlist_r */
	bool stopped;
	bool support_tsg;
	struct nvgpu_mutex mutex; /* protect channel preempt and runlist update */
};

enum {
	ENGINE_GR_GK20A	    = 0,
	ENGINE_GRCE_GK20A    = 1,
	ENGINE_ASYNC_CE_GK20A  = 2,
	ENGINE_INVAL_GK20A
};

struct fifo_pbdma_exception_info_gk20a {
	u32 status_r; /* raw register value from hardware */
	u32 id, next_id;
	u32 chan_status_v; /* raw value from hardware */
	bool id_is_chid, next_id_is_chid;
	bool chsw_in_progress;
};

struct fifo_engine_exception_info_gk20a {
	u32 status_r; /* raw register value from hardware */
	u32 id, next_id;
	u32 ctx_status_v; /* raw value from hardware */
	bool id_is_chid, next_id_is_chid;
	bool faulted, idle, ctxsw_in_progress;
};

struct fifo_mmu_fault_info_gk20a {
	u32 fault_info_v;
	u32 fault_type_v;
	u32 engine_subid_v;
	u32 client_v;
	u32 fault_hi_v;
	u32 fault_lo_v;
	u64 inst_ptr;
	const char *fault_type_desc;
	const char *engine_subid_desc;
	const char *client_desc;
};

struct fifo_engine_info_gk20a {
	u32 engine_id;
	u32 runlist_id;
	u32 intr_mask;
	u32 reset_mask;
	u32 pbdma_id;
	u32 inst_id;
	u32 pri_base;
	u32 fault_id;
	u32 engine_enum;
	struct fifo_pbdma_exception_info_gk20a pbdma_exception_info;
	struct fifo_engine_exception_info_gk20a engine_exception_info;
	struct fifo_mmu_fault_info_gk20a mmu_fault_info;

};

enum {
	PROFILE_IOCTL_ENTRY = 0,
	PROFILE_ENTRY,
	PROFILE_JOB_TRACKING,
	PROFILE_APPEND,
	PROFILE_END,
	PROFILE_IOCTL_EXIT,
	PROFILE_MAX
};

struct fifo_profile_gk20a {
	u64 timestamp[PROFILE_MAX];
};

struct fifo_gk20a {
	struct gk20a *g;
	unsigned int num_channels;
	unsigned int runlist_entry_size;
	unsigned int num_runlist_entries;

	unsigned int num_pbdma;
	u32 *pbdma_map;

	struct fifo_engine_info_gk20a *engine_info;
	u32 max_engines;
	u32 num_engines;
	u32 *active_engines_list;

	struct fifo_runlist_info_gk20a *runlist_info;
	u32 max_runlists;
#ifdef CONFIG_DEBUG_FS
	struct {
		struct fifo_profile_gk20a *data;
		atomic_t get;
		bool enabled;
		u64 *sorted;
		struct kref ref;
		struct nvgpu_mutex lock;
	} profile;
#endif
	struct mem_desc userd;
	u32 userd_entry_size;

	unsigned int used_channels;
	struct channel_gk20a *channel;
	/* zero-kref'd channels here */
	struct list_head free_chs;
	struct nvgpu_mutex free_chs_mutex;
	struct nvgpu_mutex gr_reset_mutex;

	struct tsg_gk20a *tsg;
	struct nvgpu_mutex tsg_inuse_mutex;

	void (*remove_support)(struct fifo_gk20a *);
	bool sw_ready;
	struct {
		/* share info between isrs and non-isr code */
		struct {
			struct nvgpu_mutex mutex;
		} isr;
		struct {
			u32 device_fatal_0;
			u32 channel_fatal_0;
			u32 restartable_0;
		} pbdma;
		struct {

		} engine;


	} intr;

	unsigned long deferred_fault_engines;
	bool deferred_reset_pending;
	struct nvgpu_mutex deferred_reset_mutex;
};

static inline const char *gk20a_fifo_interleave_level_name(u32 interleave_level)
{
	switch (interleave_level) {
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW:
		return "LOW";

	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
		return "MEDIUM";

	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		return "HIGH";

	default:
		return "?";
	}
}

struct ch_state {
	int pid;
	int refs;
	u32 inst_block[0];
};

int gk20a_init_fifo_support(struct gk20a *g);

int gk20a_init_fifo_setup_hw(struct gk20a *g);

void gk20a_fifo_isr(struct gk20a *g);
int gk20a_fifo_nonstall_isr(struct gk20a *g);

int gk20a_fifo_preempt_channel(struct gk20a *g, u32 hw_chid);
int gk20a_fifo_preempt_tsg(struct gk20a *g, u32 tsgid);
int gk20a_fifo_preempt(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_fifo_preempt_next(struct gk20a *g, struct channel_gk20a *ch);

int gk20a_fifo_enable_engine_activity(struct gk20a *g,
			struct fifo_engine_info_gk20a *eng_info);
int gk20a_fifo_enable_all_engine_activity(struct gk20a *g);
int gk20a_fifo_disable_engine_activity(struct gk20a *g,
			struct fifo_engine_info_gk20a *eng_info,
			bool wait_for_idle);
int gk20a_fifo_disable_all_engine_activity(struct gk20a *g,
				bool wait_for_idle);
u32 gk20a_fifo_engines_on_ch(struct gk20a *g, u32 hw_chid);

int gk20a_fifo_reschedule_runlist(struct gk20a *g, u32 runlist_id);
int gk20a_fifo_reschedule_preempt_next(struct channel_gk20a *ch);

int gk20a_fifo_update_runlist(struct gk20a *g, u32 engine_id, u32 hw_chid,
			      bool add, bool wait_for_finish);

int gk20a_fifo_suspend(struct gk20a *g);

bool gk20a_fifo_mmu_fault_pending(struct gk20a *g);

void gk20a_fifo_recover(struct gk20a *g,
			u32 engine_ids, /* if zero, will be queried from HW */
			u32 hw_id, /* if ~0, will be queried from HW */
			bool hw_id_is_tsg, /* ignored if hw_id == ~0 */
			bool id_is_known, bool verbose);
void gk20a_fifo_recover_ch(struct gk20a *g, u32 hw_chid, bool verbose);
void gk20a_fifo_recover_tsg(struct gk20a *g, u32 tsgid, bool verbose);
int gk20a_fifo_force_reset_ch(struct channel_gk20a *ch,
				u32 err_code, bool verbose);
void gk20a_fifo_reset_engine(struct gk20a *g, u32 engine_id);
int gk20a_init_fifo_reset_enable_hw(struct gk20a *g);
void gk20a_init_fifo(struct gpu_ops *gops);

void fifo_gk20a_finish_mmu_fault_handling(struct gk20a *g,
		unsigned long fault_id);
int gk20a_fifo_wait_engine_idle(struct gk20a *g);
bool gk20a_fifo_is_engine_busy(struct gk20a *g);
u32 gk20a_fifo_engine_interrupt_mask(struct gk20a *g);
u32 gk20a_fifo_get_pbdma_signature(struct gk20a *g);
u32 gk20a_fifo_get_failing_engine_data(struct gk20a *g,
		int *__id, bool *__is_tsg);
bool gk20a_fifo_set_ctx_mmu_error_tsg(struct gk20a *g,
		struct tsg_gk20a *tsg);
void gk20a_fifo_abort_tsg(struct gk20a *g, u32 tsgid, bool preempt);
bool gk20a_fifo_set_ctx_mmu_error_ch(struct gk20a *g,
		struct channel_gk20a *ch);

struct channel_gk20a *gk20a_fifo_channel_from_hw_chid(struct gk20a *g,
		u32 hw_chid);

void gk20a_fifo_issue_preempt(struct gk20a *g, u32 id, bool is_tsg);
int gk20a_fifo_set_runlist_interleave(struct gk20a *g,
				u32 id,
				bool is_tsg,
				u32 runlist_id,
				u32 new_level);
int gk20a_fifo_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);


void gk20a_fifo_debugfs_init(struct device *dev);

const char *gk20a_fifo_interleave_level_name(u32 interleave_level);

int gk20a_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
		u32 *inst_id);

u32 gk20a_fifo_get_engine_ids(struct gk20a *g, u32 engine_id[], u32 engine_id_sz, u32 engine_enum);

void gk20a_fifo_delete_runlist(struct fifo_gk20a *f);

struct fifo_engine_info_gk20a *gk20a_fifo_get_engine_info(struct gk20a *g, u32 engine_id);

bool gk20a_fifo_is_valid_engine_id(struct gk20a *g, u32 engine_id);

u32 gk20a_fifo_get_gr_engine_id(struct gk20a *g);

int gk20a_fifo_deferred_reset(struct gk20a *g, struct channel_gk20a *ch);

u32 gk20a_fifo_get_all_ce_engine_reset_mask(struct gk20a *g);

u32 gk20a_fifo_get_fast_ce_runlist_id(struct gk20a *g);

u32 gk20a_fifo_get_gr_runlist_id(struct gk20a *g);

bool gk20a_fifo_is_valid_runlist_id(struct gk20a *g, u32 runlist_id);

int gk20a_fifo_update_runlist_ids(struct gk20a *g, u32 runlist_ids, u32 hw_chid,
		bool add, bool wait_for_finish);

int gk20a_fifo_init_engine_info(struct fifo_gk20a *f);

void gk20a_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist);
void gk20a_get_ch_runlist_entry(struct channel_gk20a *ch, u32 *runlist);

u32 gk20a_userd_gp_get(struct gk20a *g, struct channel_gk20a *c);
void gk20a_userd_gp_put(struct gk20a *g, struct channel_gk20a *c);
bool gk20a_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid);
#ifdef CONFIG_DEBUG_FS
struct fifo_profile_gk20a *gk20a_fifo_profile_acquire(struct gk20a *g);
void gk20a_fifo_profile_release(struct gk20a *g,
	struct fifo_profile_gk20a *profile);
#endif

void gk20a_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 hw_chid,
				     struct ch_state *ch_state);
void gk20a_dump_pbdma_status(struct gk20a *g,
				 struct gk20a_debug_output *o);
void gk20a_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o);
const char *gk20a_decode_ccsr_chan_status(u32 index);
const char *gk20a_decode_pbdma_chan_eng_ctx_status(u32 index);

#endif /*__GR_GK20A_H__*/
