/*
 * GK20A graphics channel
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CHANNEL_GK20A_H
#define CHANNEL_GK20A_H

#include <linux/log2.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/stacktrace.h>
#include <linux/wait.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/lock.h>
#include <nvgpu/timers.h>

struct gk20a;
struct gr_gk20a;
struct dbg_session_gk20a;
struct gk20a_fence;
struct fifo_profile_gk20a;

#include "channel_sync_gk20a.h"

#include "mm_gk20a.h"
#include "gr_gk20a.h"
#include "fence_gk20a.h"

extern const struct file_operations gk20a_event_id_ops;
extern const struct file_operations gk20a_channel_ops;

struct notification {
	struct {
		u32 nanoseconds[2];
	} timestamp;
	u32 info32;
	u16 info16;
	u16 status;
};

/* contexts associated with a channel */
struct channel_ctx_gk20a {
	struct gr_ctx_desc	*gr_ctx;
	struct patch_desc	patch_ctx;
	struct zcull_ctx_desc	zcull_ctx;
	struct pm_ctx_desc	pm_ctx;
	u64	global_ctx_buffer_va[NR_GLOBAL_CTX_BUF_VA];
	u64	global_ctx_buffer_size[NR_GLOBAL_CTX_BUF_VA];
	bool	global_ctx_buffer_mapped;
	struct ctx_header_desc ctx_header;
};

struct channel_gk20a_job {
	struct mapped_buffer_node **mapped_buffers;
	int num_mapped_buffers;
	struct gk20a_fence *pre_fence;
	struct gk20a_fence *post_fence;
	struct priv_cmd_entry *wait_cmd;
	struct priv_cmd_entry *incr_cmd;
	struct list_head list;
};

struct channel_gk20a_joblist {
	struct {
		bool enabled;
		unsigned int length;
		unsigned int put;
		unsigned int get;
		struct channel_gk20a_job *jobs;
		struct nvgpu_mutex read_lock;
	} pre_alloc;

	struct {
		struct list_head jobs;
		struct nvgpu_spinlock lock;
	} dynamic;

	/*
	 * Synchronize abort cleanup (when closing a channel) and job cleanup
	 * (asynchronously from worker) - protect from concurrent access when
	 * job resources are being freed.
	 */
	struct nvgpu_mutex cleanup_lock;
};

struct channel_gk20a_timeout {
	struct nvgpu_raw_spinlock lock;
	struct nvgpu_timeout timer;
	bool running;
	u32 gp_get;
};

struct gk20a_event_id_data {
	struct gk20a *g;

	int id; /* ch or tsg */
	bool is_tsg;
	u32 event_id;

	bool event_posted;

	wait_queue_head_t event_id_wq;
	struct nvgpu_mutex lock;
	struct list_head event_id_node;
};

/*
 * Track refcount actions, saving their stack traces. This number specifies how
 * many most recent actions are stored in a buffer. Set to 0 to disable. 128
 * should be enough to track moderately hard problems from the start.
 */
#define GK20A_CHANNEL_REFCOUNT_TRACKING 0
/* Stack depth for the saved actions. */
#define GK20A_CHANNEL_REFCOUNT_TRACKING_STACKLEN 8

/*
 * Because the puts and gets are not linked together explicitly (although they
 * should always come in pairs), it's not possible to tell which ref holder to
 * delete from the list when doing a put. So, just store some number of most
 * recent gets and puts in a ring buffer, to obtain a history.
 *
 * These are zeroed when a channel is closed, so a new one starts fresh.
 */

enum channel_gk20a_ref_action_type {
	channel_gk20a_ref_action_get,
	channel_gk20a_ref_action_put
};

struct channel_gk20a_ref_action {
	enum channel_gk20a_ref_action_type type;
	unsigned long jiffies;
	/*
	 * Many of these traces will be similar. Simpler to just capture
	 * duplicates than to have a separate database for the entries.
	 */
	struct stack_trace trace;
	unsigned long trace_entries[GK20A_CHANNEL_REFCOUNT_TRACKING_STACKLEN];
};

/* this is the priv element of struct nvhost_channel */
struct channel_gk20a {
	struct gk20a *g; /* set only when channel is active */

	struct list_head free_chs;

	struct nvgpu_spinlock ref_obtain_lock;
	bool referenceable;
	atomic_t ref_count;
	wait_queue_head_t ref_count_dec_wq;
#if GK20A_CHANNEL_REFCOUNT_TRACKING
	/*
	 * Ring buffer for most recent refcount gets and puts. Protected by
	 * ref_actions_lock when getting or putting refs (i.e., adding
	 * entries), and when reading entries.
	 */
	struct channel_gk20a_ref_action ref_actions[
		GK20A_CHANNEL_REFCOUNT_TRACKING];
	size_t ref_actions_put; /* index of next write */
	struct nvgpu_spinlock ref_actions_lock;
#endif

	struct nvgpu_semaphore_int *hw_sema;

	int hw_chid;
	bool wdt_enabled;
	atomic_t bound;
	bool first_init;
	bool vpr;
	bool deterministic;
	bool cde;
	pid_t pid;
	pid_t tgid;
	struct nvgpu_mutex ioctl_lock;

	int tsgid;
	struct list_head ch_entry; /* channel's entry in TSG */

	struct channel_gk20a_joblist joblist;
	struct nvgpu_allocator fence_allocator;

	struct vm_gk20a *vm;

	struct gpfifo_desc gpfifo;

	struct channel_ctx_gk20a ch_ctx;

	struct mem_desc inst_block;
	struct mem_desc_sub ramfc;

	u64 userd_iova;
	u64 userd_gpu_va;

	u32 obj_class;	/* we support only one obj per channel */

	struct priv_cmd_queue priv_cmd_q;

	wait_queue_head_t notifier_wq;
	wait_queue_head_t semaphore_wq;

	u32 timeout_accumulated_ms;
	u32 timeout_gpfifo_get;

	struct channel_gk20a_timeout timeout;
	/* for job cleanup handling in the background worker */
	struct list_head worker_item;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	struct {
	void *cyclestate_buffer;
	u32 cyclestate_buffer_size;
	struct dma_buf *cyclestate_buffer_handler;
	struct nvgpu_mutex cyclestate_buffer_mutex;
	} cyclestate;

	struct nvgpu_mutex cs_client_mutex;
	struct gk20a_cs_snapshot_client *cs_client;
#endif
	struct nvgpu_mutex dbg_s_lock;
	struct list_head dbg_s_list;

	struct list_head event_id_list;
	struct nvgpu_mutex event_id_list_lock;

	bool has_timedout;
	u32 timeout_ms_max;
	bool timeout_debug_dump;
	unsigned int timeslice_us;

	struct dma_buf *error_notifier_ref;
	struct nvgpu_notification *error_notifier;
	void *error_notifier_va;
	struct nvgpu_mutex error_notifier_mutex;

	struct nvgpu_mutex sync_lock;
	struct gk20a_channel_sync *sync;

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	u64 virt_ctx;
#endif

	/*
	 * Signal channel owner via a callback, if set, in job cleanup with
	 * schedule_work. Means that something finished on the channel (perhaps
	 * more than one job).
	 */
	void (*update_fn)(struct channel_gk20a *, void *);
	void *update_fn_data;
	struct nvgpu_spinlock update_fn_lock; /* make access to the two above atomic */
	struct work_struct update_fn_work;

	u32 interleave_level;

	u32 runlist_id;

	bool is_privileged_channel;
};

static inline bool gk20a_channel_as_bound(struct channel_gk20a *ch)
{
	return !!ch->vm;
}
int channel_gk20a_commit_va(struct channel_gk20a *c);
int gk20a_init_channel_support(struct gk20a *, u32 chid);

/* must be inside gk20a_busy()..gk20a_idle() */
void gk20a_channel_close(struct channel_gk20a *ch);
void __gk20a_channel_kill(struct channel_gk20a *ch);

bool gk20a_channel_update_and_check_timeout(struct channel_gk20a *ch,
		u32 timeout_delta_ms, bool *progress);
void gk20a_disable_channel(struct channel_gk20a *ch);
void gk20a_channel_abort(struct channel_gk20a *ch, bool channel_preempt);
void gk20a_channel_abort_clean_up(struct channel_gk20a *ch);
void gk20a_set_error_notifier(struct channel_gk20a *ch, __u32 error);
void gk20a_set_error_notifier_locked(struct channel_gk20a *ch, __u32 error);
void gk20a_channel_semaphore_wakeup(struct gk20a *g, bool post_events);
int gk20a_channel_alloc_priv_cmdbuf(struct channel_gk20a *c, u32 size,
			     struct priv_cmd_entry *entry);
int gk20a_free_priv_cmdbuf(struct channel_gk20a *c, struct priv_cmd_entry *e);

int gk20a_enable_channel_tsg(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_disable_channel_tsg(struct gk20a *g, struct channel_gk20a *ch);

int gk20a_channel_suspend(struct gk20a *g);
int gk20a_channel_resume(struct gk20a *g);

int nvgpu_channel_worker_init(struct gk20a *g);
void nvgpu_channel_worker_deinit(struct gk20a *g);

/* Channel file operations */
int gk20a_channel_open(struct inode *inode, struct file *filp);
int gk20a_channel_open_ioctl(struct gk20a *g,
		struct nvgpu_channel_open_args *args);
long gk20a_channel_ioctl(struct file *filp,
			 unsigned int cmd,
			 unsigned long arg);
int gk20a_channel_release(struct inode *inode, struct file *filp);
struct channel_gk20a *gk20a_get_channel_from_file(int fd);
void gk20a_channel_update(struct channel_gk20a *c);

void gk20a_init_channel(struct gpu_ops *gops);

/* returns ch if reference was obtained */
struct channel_gk20a *__must_check _gk20a_channel_get(struct channel_gk20a *ch,
						      const char *caller);
#define gk20a_channel_get(ch) _gk20a_channel_get(ch, __func__)


void _gk20a_channel_put(struct channel_gk20a *ch, const char *caller);
#define gk20a_channel_put(ch) _gk20a_channel_put(ch, __func__)

int gk20a_wait_channel_idle(struct channel_gk20a *ch);

/* runlist_id -1 is synonym for ENGINE_GR_GK20A runlist id */
struct channel_gk20a *gk20a_open_new_channel(struct gk20a *g,
		s32 runlist_id,
		bool is_privileged_channel);
struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data,
		int runlist_id,
		bool is_privileged_channel);
void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a);

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvgpu_gpfifo *gpfifo,
				struct nvgpu_submit_gpfifo_args *args,
				u32 num_entries,
				u32 flags,
				struct nvgpu_fence *fence,
				struct gk20a_fence **fence_out,
				bool force_need_sync_fence,
				struct fifo_profile_gk20a *profile);

int gk20a_alloc_channel_gpfifo(struct channel_gk20a *c,
			       struct nvgpu_alloc_gpfifo_ex_args *args);

void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a);
void channel_gk20a_disable(struct channel_gk20a *ch);
int channel_gk20a_alloc_inst(struct gk20a *g, struct channel_gk20a *ch);
void channel_gk20a_free_inst(struct gk20a *g, struct channel_gk20a *ch);
u32 channel_gk20a_pbdma_acquire_val(struct channel_gk20a *c);
int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries, u32 flags);
void channel_gk20a_enable(struct channel_gk20a *ch);
void gk20a_channel_timeout_restart_all_channels(struct gk20a *g);

bool channel_gk20a_is_prealloc_enabled(struct channel_gk20a *c);
void channel_gk20a_joblist_lock(struct channel_gk20a *c);
void channel_gk20a_joblist_unlock(struct channel_gk20a *c);
bool channel_gk20a_joblist_is_empty(struct channel_gk20a *c);

int gk20a_channel_get_timescale_from_timeslice(struct gk20a *g,
		int timeslice_period,
		int *__timeslice_timeout, int *__timeslice_scale);
int gk20a_channel_set_priority(struct channel_gk20a *ch, u32 priority);
int gk20a_channel_set_timeslice(struct channel_gk20a *ch, unsigned int timeslice);
int gk20a_channel_set_runlist_interleave(struct channel_gk20a *ch,
		u32 level);
void gk20a_channel_event_id_post_event(struct channel_gk20a *ch,
				       u32 event_id);

void gk20a_channel_setup_ramfc_for_privileged_channel(struct channel_gk20a *c);

#endif /* CHANNEL_GK20A_H */
