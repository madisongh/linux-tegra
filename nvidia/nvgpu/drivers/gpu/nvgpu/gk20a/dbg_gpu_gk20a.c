/*
 * Tegra GK20A GPU Debugger/Profiler Driver
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/nvhost.h>
#include <linux/dma-buf.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/kmem.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "regops_gk20a.h"

#include <nvgpu/hw/gk20a/hw_therm_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_perf_gk20a.h>

/*
 * API to get first channel from the list of all channels
 * bound to the debug session
 */
struct channel_gk20a *
nvgpu_dbg_gpu_get_session_channel(struct dbg_session_gk20a *dbg_s)
{
	struct dbg_session_channel_data *ch_data;
	struct channel_gk20a *ch;
	struct gk20a *g = dbg_s->g;

	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	if (list_empty(&dbg_s->ch_list)) {
		nvgpu_mutex_release(&dbg_s->ch_list_lock);
		return NULL;
	}

	ch_data = list_first_entry(&dbg_s->ch_list,
				   struct dbg_session_channel_data,
				   ch_entry);
	ch = g->fifo.channel + ch_data->chid;

	nvgpu_mutex_release(&dbg_s->ch_list_lock);

	return ch;
}

/* silly allocators - just increment id */
static atomic_t session_id = ATOMIC_INIT(0);
static atomic_t profiler_id = ATOMIC_INIT(0);
static int generate_id(atomic_t *id)
{
	return atomic_add_return(1, id);
}

static int alloc_session(struct dbg_session_gk20a **_dbg_s)
{
	struct dbg_session_gk20a *dbg_s;
	*_dbg_s = NULL;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	dbg_s = kzalloc(sizeof(*dbg_s), GFP_KERNEL);
	if (!dbg_s)
		return -ENOMEM;

	dbg_s->id = generate_id(&session_id);
	*_dbg_s = dbg_s;
	return 0;
}

static int alloc_profiler(struct dbg_profiler_object_data **_prof)
{
	struct dbg_profiler_object_data *prof;
	*_prof = NULL;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	prof = kzalloc(sizeof(*prof), GFP_KERNEL);
	if (!prof)
		return -ENOMEM;

	prof->prof_handle = generate_id(&profiler_id);
	*_prof = prof;
	return 0;
}

static int gk20a_dbg_gpu_do_dev_open(struct inode *inode,
		struct file *filp, bool is_profiler)
{
	struct dbg_session_gk20a *dbg_session;
	struct gk20a *g;

	struct device *dev;

	int err;

	if (!is_profiler)
		g = container_of(inode->i_cdev,
				 struct gk20a, dbg.cdev);
	else
		g = container_of(inode->i_cdev,
				 struct gk20a, prof.cdev);
	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	dev = g->dev;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "dbg session: %s", dev_name(dev));

	err  = alloc_session(&dbg_session);
	if (err)
		goto free_ref;

	filp->private_data = dbg_session;
	dbg_session->dev   = dev;
	dbg_session->g     = g;
	dbg_session->is_profiler = is_profiler;
	dbg_session->is_pg_disabled = false;
	dbg_session->is_timeout_disabled = false;

	init_waitqueue_head(&dbg_session->dbg_events.wait_queue);
	INIT_LIST_HEAD(&dbg_session->ch_list);
	nvgpu_mutex_init(&dbg_session->ch_list_lock);
	nvgpu_mutex_init(&dbg_session->ioctl_lock);
	dbg_session->dbg_events.events_enabled = false;
	dbg_session->dbg_events.num_pending_events = 0;

	return 0;
free_ref:
	gk20a_put(g);
	return err;
}

/* used in scenarios where the debugger session can take just the inter-session
 * lock for performance, but the profiler session must take the per-gpu lock
 * since it might not have an associated channel. */
static void gk20a_dbg_session_nvgpu_mutex_acquire(struct dbg_session_gk20a *dbg_s)
{
	struct channel_gk20a *ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);

	if (dbg_s->is_profiler || !ch)
		nvgpu_mutex_acquire(&dbg_s->g->dbg_sessions_lock);
	else
		nvgpu_mutex_acquire(&ch->dbg_s_lock);
}

static void gk20a_dbg_session_nvgpu_mutex_release(struct dbg_session_gk20a *dbg_s)
{
	struct channel_gk20a *ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);

	if (dbg_s->is_profiler || !ch)
		nvgpu_mutex_release(&dbg_s->g->dbg_sessions_lock);
	else
		nvgpu_mutex_release(&ch->dbg_s_lock);
}

static void gk20a_dbg_gpu_events_enable(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_nvgpu_mutex_acquire(dbg_s);

	dbg_s->dbg_events.events_enabled = true;
	dbg_s->dbg_events.num_pending_events = 0;

	gk20a_dbg_session_nvgpu_mutex_release(dbg_s);
}

static void gk20a_dbg_gpu_events_disable(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_nvgpu_mutex_acquire(dbg_s);

	dbg_s->dbg_events.events_enabled = false;
	dbg_s->dbg_events.num_pending_events = 0;

	gk20a_dbg_session_nvgpu_mutex_release(dbg_s);
}

static void gk20a_dbg_gpu_events_clear(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_nvgpu_mutex_acquire(dbg_s);

	if (dbg_s->dbg_events.events_enabled &&
			dbg_s->dbg_events.num_pending_events > 0)
		dbg_s->dbg_events.num_pending_events--;

	gk20a_dbg_session_nvgpu_mutex_release(dbg_s);
}

static int gk20a_dbg_gpu_events_ctrl(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_events_ctrl_args *args)
{
	int ret = 0;
	struct channel_gk20a *ch;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "dbg events ctrl cmd %d", args->cmd);

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch) {
		gk20a_err(dev_from_gk20a(dbg_s->g),
			   "no channel bound to dbg session\n");
		return -EINVAL;
	}

	switch (args->cmd) {
	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_ENABLE:
		gk20a_dbg_gpu_events_enable(dbg_s);
		break;

	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_DISABLE:
		gk20a_dbg_gpu_events_disable(dbg_s);
		break;

	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_CLEAR:
		gk20a_dbg_gpu_events_clear(dbg_s);
		break;

	default:
		gk20a_err(dev_from_gk20a(dbg_s->g),
			   "unrecognized dbg gpu events ctrl cmd: 0x%x",
			   args->cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

unsigned int gk20a_dbg_gpu_dev_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct dbg_session_gk20a *dbg_s = filep->private_data;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	poll_wait(filep, &dbg_s->dbg_events.wait_queue, wait);

	gk20a_dbg_session_nvgpu_mutex_acquire(dbg_s);

	if (dbg_s->dbg_events.events_enabled &&
			dbg_s->dbg_events.num_pending_events > 0) {
		gk20a_dbg(gpu_dbg_gpu_dbg, "found pending event on session id %d",
				dbg_s->id);
		gk20a_dbg(gpu_dbg_gpu_dbg, "%d events pending",
				dbg_s->dbg_events.num_pending_events);
		mask = (POLLPRI | POLLIN);
	}

	gk20a_dbg_session_nvgpu_mutex_release(dbg_s);

	return mask;
}

int gk20a_dbg_gpu_dev_open(struct inode *inode, struct file *filp)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");
	return gk20a_dbg_gpu_do_dev_open(inode, filp, false /* not profiler */);
}

int gk20a_prof_gpu_dev_open(struct inode *inode, struct file *filp)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");
	return gk20a_dbg_gpu_do_dev_open(inode, filp, true /* is profiler */);
}

void gk20a_dbg_gpu_post_events(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	list_for_each_entry(session_data, &ch->dbg_s_list, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->dbg_events.events_enabled) {
			gk20a_dbg(gpu_dbg_gpu_dbg, "posting event on session id %d",
					dbg_s->id);
			gk20a_dbg(gpu_dbg_gpu_dbg, "%d events pending",
					dbg_s->dbg_events.num_pending_events);

			dbg_s->dbg_events.num_pending_events++;

			wake_up_interruptible_all(&dbg_s->dbg_events.wait_queue);
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);
}

bool gk20a_dbg_gpu_broadcast_stop_trigger(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;
	bool broadcast = false;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	list_for_each_entry(session_data, &ch->dbg_s_list, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->broadcast_stop_trigger) {
			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn | gpu_dbg_intr,
					"stop trigger broadcast enabled");
			broadcast = true;
			break;
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);

	return broadcast;
}

int gk20a_dbg_gpu_clear_broadcast_stop_trigger(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	list_for_each_entry(session_data, &ch->dbg_s_list, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->broadcast_stop_trigger) {
			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn | gpu_dbg_intr,
					"stop trigger broadcast disabled");
			dbg_s->broadcast_stop_trigger = false;
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);

	return 0;
}

static int nvgpu_dbg_timeout_enable(struct dbg_session_gk20a *dbg_s,
			  int timeout_mode)
{
	struct gk20a *g = dbg_s->g;
	int err = 0;

	gk20a_dbg(gpu_dbg_gpu_dbg, "Timeouts mode requested : %d",
			timeout_mode);

	switch (timeout_mode) {
	case NVGPU_DBG_GPU_IOCTL_TIMEOUT_ENABLE:
		if (dbg_s->is_timeout_disabled &&
		    --g->dbg_timeout_disabled_refcount == 0) {
			g->timeouts_enabled = true;
		}
		dbg_s->is_timeout_disabled = false;
		break;

	case NVGPU_DBG_GPU_IOCTL_TIMEOUT_DISABLE:
		if ((dbg_s->is_timeout_disabled == false) &&
		    (g->dbg_timeout_disabled_refcount++ == 0)) {
			g->timeouts_enabled = false;
		}
		dbg_s->is_timeout_disabled = true;
		break;

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized dbg gpu timeout mode : 0x%x",
			   timeout_mode);
		err = -EINVAL;
		break;
	}

	gk20a_dbg(gpu_dbg_gpu_dbg, "Timeouts enabled : %s",
			g->timeouts_enabled ? "Yes" : "No");

	return err;
}

int dbg_unbind_single_channel_gk20a(struct dbg_session_gk20a *dbg_s,
			struct dbg_session_channel_data *ch_data)
{
	struct gk20a *g = dbg_s->g;
	int chid;
	struct dbg_session_data *session_data;
	struct dbg_profiler_object_data *prof_obj, *tmp_obj;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	chid = ch_data->chid;

	/* If there's a profiler ctx reservation record associated with this
	 * session/channel pair, release it.
	 */
	list_for_each_entry_safe(prof_obj, tmp_obj, &g->profiler_objects,
							prof_obj_entry) {
		if ((prof_obj->session_id == dbg_s->id) &&
			(prof_obj->ch->hw_chid == chid)) {
			if (prof_obj->has_reservation) {
				g->ops.dbg_session_ops.
				  release_profiler_reservation(dbg_s, prof_obj);
			}
			list_del(&prof_obj->prof_obj_entry);
			kfree(prof_obj);
		}
	}

	list_del_init(&ch_data->ch_entry);

	session_data = ch_data->session_data;
	list_del_init(&session_data->dbg_s_entry);
	kfree(session_data);

	fput(ch_data->ch_f);
	kfree(ch_data);

	return 0;
}

static int dbg_unbind_all_channels_gk20a(struct dbg_session_gk20a *dbg_s)
{
	struct dbg_session_channel_data *ch_data, *tmp;
	struct gk20a *g = dbg_s->g;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	list_for_each_entry_safe(ch_data, tmp, &dbg_s->ch_list, ch_entry)
		dbg_unbind_single_channel_gk20a(dbg_s, ch_data);
	nvgpu_mutex_release(&dbg_s->ch_list_lock);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return 0;
}

static int dbg_unbind_channel_gk20a(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_unbind_channel_args *args)
{
	struct dbg_session_channel_data *ch_data;
	struct gk20a *g = dbg_s->g;
	bool channel_found = false;
	struct channel_gk20a *ch;
	int err;

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s fd=%d",
		   dev_name(dbg_s->dev), args->channel_fd);

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch) {
		gk20a_dbg_fn("no channel found for fd");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	list_for_each_entry(ch_data, &dbg_s->ch_list, ch_entry) {
		if (ch->hw_chid == ch_data->chid) {
			channel_found = true;
			break;
		}
	}
	nvgpu_mutex_release(&dbg_s->ch_list_lock);

	if (!channel_found) {
		gk20a_dbg_fn("channel not bounded, fd=%d\n", args->channel_fd);
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	err = dbg_unbind_single_channel_gk20a(dbg_s, ch_data);
	nvgpu_mutex_release(&dbg_s->ch_list_lock);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err;
}

int gk20a_dbg_gpu_dev_release(struct inode *inode, struct file *filp)
{
	struct dbg_session_gk20a *dbg_s = filp->private_data;
	struct gk20a *g = dbg_s->g;
	struct dbg_profiler_object_data *prof_obj, *tmp_obj;

	gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "%s", dev_name(dbg_s->dev));

	/* unbind channels */
	dbg_unbind_all_channels_gk20a(dbg_s);

	/* Powergate/Timeout enable is called here as possibility of dbg_session
	 * which called powergate/timeout disable ioctl, to be killed without
	 * calling powergate/timeout enable ioctl
	 */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	g->ops.dbg_session_ops.dbg_set_powergate(dbg_s,
				NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE);
	nvgpu_dbg_timeout_enable(dbg_s, NVGPU_DBG_GPU_IOCTL_TIMEOUT_ENABLE);

	/* Per-context profiler objects were released when we called
	 * dbg_unbind_all_channels. We could still have global ones.
	 */
	list_for_each_entry_safe(prof_obj, tmp_obj, &g->profiler_objects,
							prof_obj_entry) {
		if (prof_obj->session_id == dbg_s->id) {
			if (prof_obj->has_reservation)
				g->ops.dbg_session_ops.
				  release_profiler_reservation(dbg_s, prof_obj);
			list_del(&prof_obj->prof_obj_entry);
			kfree(prof_obj);
		}
	}
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	kfree(dbg_s);
	gk20a_put(g);

	return 0;
}

static int dbg_bind_channel_gk20a(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_bind_channel_args *args)
{
	struct file *f;
	struct gk20a *g = dbg_s->g;
	struct channel_gk20a *ch;
	struct dbg_session_channel_data *ch_data;
	struct dbg_session_data *session_data;

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s fd=%d",
		   dev_name(dbg_s->dev), args->channel_fd);

	/* even though get_file_channel is doing this it releases it as well */
	/* by holding it here we'll keep it from disappearing while the
	 * debugger is in session */
	f = fget(args->channel_fd);
	if (!f)
		return -ENODEV;

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch) {
		gk20a_dbg_fn("no channel found for fd");
		fput(f);
		return -EINVAL;
	}

	gk20a_dbg_fn("%s hwchid=%d", dev_name(dbg_s->dev), ch->hw_chid);

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	ch_data = kzalloc(sizeof(*ch_data), GFP_KERNEL);
	if (!ch_data) {
		fput(f);
		return -ENOMEM;
	}
	ch_data->ch_f = f;
	ch_data->channel_fd = args->channel_fd;
	ch_data->chid = ch->hw_chid;
	INIT_LIST_HEAD(&ch_data->ch_entry);

	session_data = kzalloc(sizeof(*session_data), GFP_KERNEL);
	if (!session_data) {
		kfree(ch_data);
		fput(f);
		return -ENOMEM;
	}
	session_data->dbg_s = dbg_s;
	INIT_LIST_HEAD(&session_data->dbg_s_entry);
	ch_data->session_data = session_data;

	list_add(&session_data->dbg_s_entry, &ch->dbg_s_list);

	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	list_add_tail(&ch_data->ch_entry, &dbg_s->ch_list);
	nvgpu_mutex_release(&dbg_s->ch_list_lock);

	nvgpu_mutex_release(&ch->dbg_s_lock);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return 0;
}

static int nvgpu_ioctl_channel_reg_ops(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_exec_reg_ops_args *args);

static int nvgpu_ioctl_powergate_gk20a(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_powergate_args *args);

static int nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			      struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *args);

static int nvgpu_dbg_gpu_ioctl_hwpm_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			      struct nvgpu_dbg_gpu_hwpm_ctxsw_mode_args *args);

static int nvgpu_dbg_gpu_ioctl_suspend_resume_sm(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *args);

static int nvgpu_ioctl_allocate_profiler_object(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_profiler_obj_mgt_args *args);

static int nvgpu_ioctl_free_profiler_object(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_profiler_obj_mgt_args *args);

static int nvgpu_ioctl_profiler_reserve(struct dbg_session_gk20a *dbg_s,
			   struct nvgpu_dbg_gpu_profiler_reserve_args *args);

static int gk20a_perfbuf_map(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_map_args *args);

static int gk20a_perfbuf_unmap(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_unmap_args *args);

static int gk20a_dbg_pc_sampling(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_pc_sampling_args *args)
{
	struct channel_gk20a *ch;
	struct gk20a *g = dbg_s->g;

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch)
		return -EINVAL;

	gk20a_dbg_fn("");

	return g->ops.gr.update_pc_sampling ?
		g->ops.gr.update_pc_sampling(ch, args->enable) : -EINVAL;
}

static int nvgpu_dbg_gpu_ioctl_timeout(struct dbg_session_gk20a *dbg_s,
			 struct nvgpu_dbg_gpu_timeout_args *args)
{
	int err;
	struct gk20a *g = dbg_s->g;

	gk20a_dbg_fn("powergate mode = %d", args->enable);

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = nvgpu_dbg_timeout_enable(dbg_s, args->enable);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err;
}

static void nvgpu_dbg_gpu_ioctl_get_timeout(struct dbg_session_gk20a *dbg_s,
			 struct nvgpu_dbg_gpu_timeout_args *args)
{
	int status;
	struct gk20a *g = dbg_s->g;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	status = g->timeouts_enabled;
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	if (status)
		args->enable = NVGPU_DBG_GPU_IOCTL_TIMEOUT_ENABLE;
	else
		args->enable = NVGPU_DBG_GPU_IOCTL_TIMEOUT_DISABLE;
}

static int nvgpu_dbg_gpu_ioctl_set_next_stop_trigger_type(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_set_next_stop_trigger_type_args *args)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_nvgpu_mutex_acquire(dbg_s);

	dbg_s->broadcast_stop_trigger = (args->broadcast != 0);

	gk20a_dbg_session_nvgpu_mutex_release(dbg_s);

	return 0;
}

static int nvgpu_dbg_gpu_ioctl_read_single_sm_error_state(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_read_single_sm_error_state_args *args)
{
	struct gk20a *g = dbg_s->g;
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_state;
	u32 sm_id;
	int err = 0;

	sm_id = args->sm_id;
	if (sm_id >= gr->no_of_sm)
		return -EINVAL;

	sm_error_state = gr->sm_error_states + sm_id;

	if (args->sm_error_state_record_size > 0) {
		size_t write_size = sizeof(*sm_error_state);

		if (write_size > args->sm_error_state_record_size)
			write_size = args->sm_error_state_record_size;

		nvgpu_mutex_acquire(&g->dbg_sessions_lock);
		err = copy_to_user((void __user *)(uintptr_t)
						args->sm_error_state_record_mem,
				   sm_error_state,
				   write_size);
		nvgpu_mutex_release(&g->dbg_sessions_lock);
		if (err) {
			gk20a_err(dev_from_gk20a(g), "copy_to_user failed!\n");
			return err;
		}

		args->sm_error_state_record_size = write_size;
	}

	return 0;
}

static int nvgpu_dbg_gpu_ioctl_clear_single_sm_error_state(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_clear_single_sm_error_state_args *args)
{
	struct gk20a *g = dbg_s->g;
	struct gr_gk20a *gr = &g->gr;
	u32 sm_id;
	struct channel_gk20a *ch;
	int err = 0;

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch)
		return -EINVAL;

	sm_id = args->sm_id;

	if (sm_id >= gr->no_of_sm)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.clear_sm_error_state(g, ch, sm_id));

	gk20a_idle(g);

	return err;
}

static int nvgpu_dbg_gpu_ioctl_write_single_sm_error_state(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_write_single_sm_error_state_args *args)
{
	struct gk20a *g = dbg_s->g;
	struct gr_gk20a *gr = &g->gr;
	u32 sm_id;
	struct channel_gk20a *ch;
	struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_state;
	int err = 0;

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch)
		return -EINVAL;

	sm_id = args->sm_id;
	if (sm_id >= gr->no_of_sm)
		return -EINVAL;

	sm_error_state = kzalloc(sizeof(*sm_error_state), GFP_KERNEL);
	if (!sm_error_state)
		return -ENOMEM;

	if (args->sm_error_state_record_size > 0) {
		size_t read_size = sizeof(*sm_error_state);

		if (read_size > args->sm_error_state_record_size)
			read_size = args->sm_error_state_record_size;

		nvgpu_mutex_acquire(&g->dbg_sessions_lock);
		err = copy_from_user(sm_error_state,
			  (void __user *)(uintptr_t)
				args->sm_error_state_record_mem,
			  read_size);
		nvgpu_mutex_release(&g->dbg_sessions_lock);
		if (err) {
			err = -ENOMEM;
			goto err_free;
		}
	}

	err = gk20a_busy(g);
	if (err)
		goto err_free;

	err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.update_sm_error_state(g, ch,
					sm_id, sm_error_state));

	gk20a_idle(g);

err_free:
	kfree(sm_error_state);

	return err;
}

static int
nvgpu_dbg_gpu_ioctl_suspend_resume_contexts(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_suspend_resume_contexts_args *args)
{
	struct gk20a *g = dbg_s->g;
	int err = 0;
	int ctx_resident_ch_fd = -1;

	err = gk20a_busy(g);
	if (err)
		return err;

	switch (args->action) {
	case NVGPU_DBG_GPU_SUSPEND_ALL_CONTEXTS:
		err = g->ops.gr.suspend_contexts(g, dbg_s,
					&ctx_resident_ch_fd);
		break;

	case NVGPU_DBG_GPU_RESUME_ALL_CONTEXTS:
		err = g->ops.gr.resume_contexts(g, dbg_s,
					&ctx_resident_ch_fd);
		break;
	}

	if (ctx_resident_ch_fd < 0) {
		args->is_resident_context = 0;
	} else {
		args->is_resident_context = 1;
		args->resident_context_fd = ctx_resident_ch_fd;
	}

	gk20a_idle(g);

	return err;
}

static int nvgpu_dbg_gpu_ioctl_access_fb_memory(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_access_fb_memory_args *args)
{
	struct gk20a *g = dbg_s->g;
	struct dma_buf *dmabuf;
	void __user *user_buffer = (void __user *)(uintptr_t)args->buffer;
	void *buffer;
	u64 size, access_size, offset;
	u64 access_limit_size = SZ_4K;
	int err = 0;

	if ((args->offset & 3) || (!args->size) || (args->size & 3))
		return -EINVAL;

	dmabuf = dma_buf_get(args->dmabuf_fd);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	if ((args->offset > dmabuf->size) ||
	    (args->size > dmabuf->size) ||
	    (args->offset + args->size > dmabuf->size)) {
		err = -EINVAL;
		goto fail_dmabuf_put;
	}

	buffer = nvgpu_big_zalloc(g, access_limit_size);
	if (!buffer) {
		err = -ENOMEM;
		goto fail_dmabuf_put;
	}

	size = args->size;
	offset = 0;

	err = gk20a_busy(g);
	if (err)
		goto fail_free_buffer;

	while (size) {
		/* Max access size of access_limit_size in one loop */
		access_size = min(access_limit_size, size);

		if (args->cmd ==
		    NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_WRITE) {
			err = copy_from_user(buffer, user_buffer + offset,
					     access_size);
			if (err)
				goto fail_idle;
		}

		err = gk20a_vidbuf_access_memory(g, dmabuf, buffer,
					 args->offset + offset, access_size,
					 args->cmd);
		if (err)
			goto fail_idle;

		if (args->cmd ==
		    NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY_CMD_READ) {
			err = copy_to_user(user_buffer + offset,
					   buffer, access_size);
			if (err)
				goto fail_idle;
		}

		size -= access_size;
		offset += access_size;
	}

fail_idle:
	gk20a_idle(g);
fail_free_buffer:
	nvgpu_big_free(g, buffer);
fail_dmabuf_put:
	dma_buf_put(dmabuf);

	return err;
}

long gk20a_dbg_gpu_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct dbg_session_gk20a *dbg_s = filp->private_data;
	struct gk20a *g = dbg_s->g;
	u8 buf[NVGPU_DBG_GPU_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	if ((_IOC_TYPE(cmd) != NVGPU_DBG_GPU_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == 0) ||
	    (_IOC_NR(cmd) > NVGPU_DBG_GPU_IOCTL_LAST) ||
	    (_IOC_SIZE(cmd) > NVGPU_DBG_GPU_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->sw_ready) {
		err = gk20a_busy(g);
		if (err)
			return err;

		gk20a_idle(g);
	}

	/* protect from threaded user space calls */
	nvgpu_mutex_acquire(&dbg_s->ioctl_lock);

	switch (cmd) {
	case NVGPU_DBG_GPU_IOCTL_BIND_CHANNEL:
		err = dbg_bind_channel_gk20a(dbg_s,
			     (struct nvgpu_dbg_gpu_bind_channel_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_REG_OPS:
		err = nvgpu_ioctl_channel_reg_ops(dbg_s,
			   (struct nvgpu_dbg_gpu_exec_reg_ops_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_POWERGATE:
		err = nvgpu_ioctl_powergate_gk20a(dbg_s,
			   (struct nvgpu_dbg_gpu_powergate_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_EVENTS_CTRL:
		err = gk20a_dbg_gpu_events_ctrl(dbg_s,
			   (struct nvgpu_dbg_gpu_events_ctrl_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SMPC_CTXSW_MODE:
		err = nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(dbg_s,
			   (struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_HWPM_CTXSW_MODE:
		err = nvgpu_dbg_gpu_ioctl_hwpm_ctxsw_mode(dbg_s,
			   (struct nvgpu_dbg_gpu_hwpm_ctxsw_mode_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SUSPEND_RESUME_ALL_SMS:
		err = nvgpu_dbg_gpu_ioctl_suspend_resume_sm(dbg_s,
		       (struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PERFBUF_MAP:
		err = gk20a_perfbuf_map(dbg_s,
		       (struct nvgpu_dbg_gpu_perfbuf_map_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PERFBUF_UNMAP:
		err = gk20a_perfbuf_unmap(dbg_s,
		       (struct nvgpu_dbg_gpu_perfbuf_unmap_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PC_SAMPLING:
		err = gk20a_dbg_pc_sampling(dbg_s,
			   (struct nvgpu_dbg_gpu_pc_sampling_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SET_NEXT_STOP_TRIGGER_TYPE:
		err = nvgpu_dbg_gpu_ioctl_set_next_stop_trigger_type(dbg_s,
		       (struct nvgpu_dbg_gpu_set_next_stop_trigger_type_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_TIMEOUT:
		err = nvgpu_dbg_gpu_ioctl_timeout(dbg_s,
			   (struct nvgpu_dbg_gpu_timeout_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_GET_TIMEOUT:
		nvgpu_dbg_gpu_ioctl_get_timeout(dbg_s,
			   (struct nvgpu_dbg_gpu_timeout_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_READ_SINGLE_SM_ERROR_STATE:
		err = nvgpu_dbg_gpu_ioctl_read_single_sm_error_state(dbg_s,
		   (struct nvgpu_dbg_gpu_read_single_sm_error_state_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_CLEAR_SINGLE_SM_ERROR_STATE:
		err = nvgpu_dbg_gpu_ioctl_clear_single_sm_error_state(dbg_s,
		  (struct nvgpu_dbg_gpu_clear_single_sm_error_state_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_WRITE_SINGLE_SM_ERROR_STATE:
		err = nvgpu_dbg_gpu_ioctl_write_single_sm_error_state(dbg_s,
		  (struct nvgpu_dbg_gpu_write_single_sm_error_state_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_UNBIND_CHANNEL:
		err = dbg_unbind_channel_gk20a(dbg_s,
			     (struct nvgpu_dbg_gpu_unbind_channel_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SUSPEND_RESUME_CONTEXTS:
		err = nvgpu_dbg_gpu_ioctl_suspend_resume_contexts(dbg_s,
		      (struct nvgpu_dbg_gpu_suspend_resume_contexts_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_ACCESS_FB_MEMORY:
		err = nvgpu_dbg_gpu_ioctl_access_fb_memory(dbg_s,
			(struct nvgpu_dbg_gpu_access_fb_memory_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PROFILER_ALLOCATE:
		err = nvgpu_ioctl_allocate_profiler_object(dbg_s,
			(struct nvgpu_dbg_gpu_profiler_obj_mgt_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PROFILER_FREE:
		err = nvgpu_ioctl_free_profiler_object(dbg_s,
			(struct nvgpu_dbg_gpu_profiler_obj_mgt_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PROFILER_RESERVE:
		err = nvgpu_ioctl_profiler_reserve(dbg_s,
			   (struct nvgpu_dbg_gpu_profiler_reserve_args *)buf);
		break;

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized dbg gpu ioctl cmd: 0x%x",
			   cmd);
		err = -ENOTTY;
		break;
	}

	nvgpu_mutex_release(&dbg_s->ioctl_lock);

	gk20a_dbg(gpu_dbg_gpu_dbg, "ret=%d", err);

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg,
				   buf, _IOC_SIZE(cmd));

	return err;
}

/* In order to perform a context relative op the context has
 * to be created already... which would imply that the
 * context switch mechanism has already been put in place.
 * So by the time we perform such an opertation it should always
 * be possible to query for the appropriate context offsets, etc.
 *
 * But note: while the dbg_gpu bind requires the a channel fd,
 * it doesn't require an allocated gr/compute obj at that point...
 */
static bool gr_context_info_available(struct dbg_session_gk20a *dbg_s,
				      struct gr_gk20a *gr)
{
	int err;

	nvgpu_mutex_acquire(&gr->ctx_mutex);
	err = !gr->ctx_vars.golden_image_initialized;
	nvgpu_mutex_release(&gr->ctx_mutex);
	if (err)
		return false;
	return true;

}

static int nvgpu_ioctl_channel_reg_ops(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_exec_reg_ops_args *args)
{
	int err = 0, powergate_err = 0;
	bool is_pg_disabled = false;

	struct device *dev = dbg_s->dev;
	struct gk20a *g = dbg_s->g;
	struct channel_gk20a *ch;

	gk20a_dbg_fn("%d ops, max fragment %d", args->num_ops, g->dbg_regops_tmp_buf_ops);

	if (args->num_ops > g->gpu_characteristics.reg_ops_limit) {
		gk20a_err(dev, "regops limit exceeded");
		return -EINVAL;
	}

	if (args->num_ops == 0) {
		/* Nothing to do */
		return 0;
	}

	if (g->dbg_regops_tmp_buf_ops == 0 || !g->dbg_regops_tmp_buf) {
		gk20a_err(dev, "reg ops work buffer not allocated");
		return -ENODEV;
	}

	if (!dbg_s->id) {
		gk20a_err(dev, "can't call reg_ops on an unbound debugger session");
		return -EINVAL;
	}

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!dbg_s->is_profiler && !ch) {
		gk20a_err(dev, "bind a channel before regops for a debugging session");
		return -EINVAL;
	}

	/* be sure that ctx info is in place */
	if (!gk20a_gpu_is_virtual(dbg_s->dev) &&
		!gr_context_info_available(dbg_s, &g->gr)) {
		gk20a_err(dev, "gr context data not available\n");
		return -ENODEV;
	}

	/* since exec_reg_ops sends methods to the ucode, it must take the
	 * global gpu lock to protect against mixing methods from debug sessions
	 * on other channels */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	if (!dbg_s->is_pg_disabled && !gk20a_gpu_is_virtual(dbg_s->dev)) {
		/* In the virtual case, the server will handle
		 * disabling/enabling powergating when processing reg ops
		 */
		powergate_err = g->ops.dbg_session_ops.dbg_set_powergate(dbg_s,
					NVGPU_DBG_GPU_POWERGATE_MODE_DISABLE);
		is_pg_disabled = true;
	}

	if (!powergate_err) {
		u64 ops_offset = 0; /* index offset */

		while (ops_offset < args->num_ops && !err) {
			const u64 num_ops =
				min(args->num_ops - ops_offset,
				    (u64)(g->dbg_regops_tmp_buf_ops));
			const u64 fragment_size =
				num_ops * sizeof(g->dbg_regops_tmp_buf[0]);

			void __user *const fragment =
				(void __user *)(uintptr_t)
				(args->ops +
				 ops_offset * sizeof(g->dbg_regops_tmp_buf[0]));

			gk20a_dbg_fn("Regops fragment: start_op=%llu ops=%llu",
				     ops_offset, num_ops);

			gk20a_dbg_fn("Copying regops from userspace");

			if (copy_from_user(g->dbg_regops_tmp_buf,
					   fragment, fragment_size)) {
				dev_err(dev, "copy_from_user failed!");
				err = -EFAULT;
				break;
			}

			err = g->ops.dbg_session_ops.exec_reg_ops(
				dbg_s, g->dbg_regops_tmp_buf, num_ops);

			gk20a_dbg_fn("Copying result to userspace");

			if (copy_to_user(fragment, g->dbg_regops_tmp_buf,
					 fragment_size)) {
				dev_err(dev, "copy_to_user failed!");
				err = -EFAULT;
				break;
			}

			ops_offset += num_ops;
		}

		/* enable powergate, if previously disabled */
		if (is_pg_disabled) {
			powergate_err =
				g->ops.dbg_session_ops.dbg_set_powergate(dbg_s,
					NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE);
		}
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);

	if (!err && powergate_err)
		err = powergate_err;

	if (err)
		gk20a_err(dev, "dbg regops failed");

	return err;
}

static int dbg_set_powergate(struct dbg_session_gk20a *dbg_s, u32  powermode)
{
	int err = 0;
	struct gk20a *g = dbg_s->g;

	 /* This function must be called with g->dbg_sessions_lock held */

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %d",
		   dev_name(dbg_s->dev), powermode);

	switch (powermode) {
	case NVGPU_DBG_GPU_POWERGATE_MODE_DISABLE:
		/* save off current powergate, clk state.
		 * set gpu module's can_powergate = 0.
		 * set gpu module's clk to max.
		 * while *a* debug session is active there will be no power or
		 * clocking state changes allowed from mainline code (but they
		 * should be saved).
		 */
		/* Allow powergate disable if the current dbg_session doesn't
		 * call a powergate disable ioctl and the global
		 * powergating_disabled_refcount is zero
		 */

		if ((dbg_s->is_pg_disabled == false) &&
		    (g->dbg_powergating_disabled_refcount++ == 0)) {

			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "module busy");
			err = gk20a_busy(g);
			if (err)
				return err;

			/*do elpg disable before clock gating */
			gk20a_pmu_pg_global_enable(g, false);

			if (g->ops.clock_gating.slcg_gr_load_gating_prod)
				g->ops.clock_gating.slcg_gr_load_gating_prod(g,
						false);
			if (g->ops.clock_gating.slcg_perf_load_gating_prod)
				g->ops.clock_gating.slcg_perf_load_gating_prod(g,
						false);
			if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
				g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
						false);

			gr_gk20a_init_cg_mode(g, BLCG_MODE, BLCG_RUN);
			g->elcg_enabled = false;
			gr_gk20a_init_cg_mode(g, ELCG_MODE, ELCG_RUN);

		}

		dbg_s->is_pg_disabled = true;
		break;

	case NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE:
		/* restore (can) powergate, clk state */
		/* release pending exceptions to fault/be handled as usual */
		/*TBD: ordering of these? */

		/* Re-enabling powergate as no other sessions want
		 * powergate disabled and the current dbg-sessions had
		 * requested the powergate disable through ioctl
		*/
		if (dbg_s->is_pg_disabled &&
		    --g->dbg_powergating_disabled_refcount == 0) {

			g->elcg_enabled = true;
			gr_gk20a_init_cg_mode(g, ELCG_MODE, ELCG_AUTO);
			gr_gk20a_init_cg_mode(g, BLCG_MODE, BLCG_AUTO);

			if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
				g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
						g->slcg_enabled);
			if (g->ops.clock_gating.slcg_perf_load_gating_prod)
				g->ops.clock_gating.slcg_perf_load_gating_prod(g,
						g->slcg_enabled);
			if (g->ops.clock_gating.slcg_gr_load_gating_prod)
				g->ops.clock_gating.slcg_gr_load_gating_prod(g,
						g->slcg_enabled);

			gk20a_pmu_pg_global_enable(g, true);

			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "module idle");
			gk20a_idle(g);
		}

		dbg_s->is_pg_disabled = false;
		break;

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized dbg gpu powergate mode: 0x%x",
			   powermode);
		err = -ENOTTY;
		break;
	}

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %d done",
		   dev_name(dbg_s->dev), powermode);
	return err;
}

static int nvgpu_ioctl_powergate_gk20a(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_powergate_args *args)
{
	int err;
	struct gk20a *g = dbg_s->g;
	gk20a_dbg_fn("%s  powergate mode = %d",
		      dev_name(dbg_s->dev), args->mode);

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	err = g->ops.dbg_session_ops.dbg_set_powergate(dbg_s, args->mode);
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return  err;
}

static int nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			       struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *args)
{
	int err;
	struct gk20a *g = dbg_s->g;
	struct channel_gk20a *ch_gk20a;

	gk20a_dbg_fn("%s smpc ctxsw mode = %d",
		     dev_name(dbg_s->dev), args->mode);

	err = gk20a_busy(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to poweron");
		return err;
	}

	/* Take the global lock, since we'll be doing global regops */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	ch_gk20a = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch_gk20a) {
		gk20a_err(dev_from_gk20a(g),
			  "no bound channel for smpc ctxsw mode update\n");
		err = -EINVAL;
		goto clean_up;
	}

	err = g->ops.gr.update_smpc_ctxsw_mode(g, ch_gk20a,
				args->mode == NVGPU_DBG_GPU_SMPC_CTXSW_MODE_CTXSW);
	if (err) {
		gk20a_err(dev_from_gk20a(g),
			  "error (%d) during smpc ctxsw mode update\n", err);
		goto clean_up;
	}

	err = g->ops.regops.apply_smpc_war(dbg_s);
 clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	gk20a_idle(g);
	return  err;
}

static int nvgpu_dbg_gpu_ioctl_hwpm_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			       struct nvgpu_dbg_gpu_hwpm_ctxsw_mode_args *args)
{
	int err;
	struct gk20a *g = dbg_s->g;
	struct channel_gk20a *ch_gk20a;

	gk20a_dbg_fn("%s pm ctxsw mode = %d",
		     dev_name(dbg_s->dev), args->mode);

	/* Must have a valid reservation to enable/disable hwpm cxtsw.
	 * Just print an error message for now, but eventually this should
	 * return an error, at the point where all client sw has been
	 * cleaned up.
	 */
	if (!dbg_s->has_profiler_reservation) {
		gk20a_err(dev_from_gk20a(g),
			"session doesn't have a valid reservation");
	}

	err = gk20a_busy(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to poweron");
		return err;
	}

	/* Take the global lock, since we'll be doing global regops */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	ch_gk20a = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch_gk20a) {
		gk20a_err(dev_from_gk20a(g),
			  "no bound channel for pm ctxsw mode update\n");
		err = -EINVAL;
		goto clean_up;
	}

	err = g->ops.gr.update_hwpm_ctxsw_mode(g, ch_gk20a,
				args->mode == NVGPU_DBG_GPU_HWPM_CTXSW_MODE_CTXSW);
	if (err)
		gk20a_err(dev_from_gk20a(g),
			  "error (%d) during pm ctxsw mode update\n", err);

	/* gk20a would require a WAR to set the core PM_ENABLE bit, not
	 * added here with gk20a being deprecated
	 */
 clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	gk20a_idle(g);
	return  err;
}

static int nvgpu_dbg_gpu_ioctl_suspend_resume_sm(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *args)
{
	struct gk20a *g = dbg_s->g;
	struct channel_gk20a *ch;
	int err = 0, action = args->mode;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "action: %d", args->mode);

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	if (!ch)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to poweron");
		return err;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	/* Suspend GPU context switching */
	err = gr_gk20a_disable_ctxsw(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "unable to stop gr ctxsw");
		/* this should probably be ctx-fatal... */
		goto clean_up;
	}

	switch (action) {
	case NVGPU_DBG_GPU_SUSPEND_ALL_SMS:
		gr_gk20a_suspend_context(ch);
		break;

	case NVGPU_DBG_GPU_RESUME_ALL_SMS:
		gr_gk20a_resume_context(ch);
		break;
	}

	err = gr_gk20a_enable_ctxsw(g);
	if (err)
		gk20a_err(dev_from_gk20a(g), "unable to restart ctxsw!\n");

clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	gk20a_idle(g);

	return  err;
}

static int nvgpu_ioctl_allocate_profiler_object(
				struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_profiler_obj_mgt_args *args)
{
	int err = 0;
	struct gk20a *g = get_gk20a(dbg_s->dev);
	struct dbg_profiler_object_data *prof_obj;

	gk20a_dbg_fn("%s", dev_name(dbg_s->dev));

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	err = alloc_profiler(&prof_obj);
	if (err)
		goto clean_up;

	prof_obj->session_id = dbg_s->id;

	if (dbg_s->is_profiler)
		prof_obj->ch = NULL;
	else {
		prof_obj->ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
		if (prof_obj->ch == NULL) {
			gk20a_err(dev_from_gk20a(g),
				"bind a channel for dbg session");
			kfree(prof_obj);
			err = -EINVAL;
			goto clean_up;
		}
	}

	/* Return handle to client */
	args->profiler_handle = prof_obj->prof_handle;

	INIT_LIST_HEAD(&prof_obj->prof_obj_entry);

	list_add(&prof_obj->prof_obj_entry, &g->profiler_objects);
clean_up:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return  err;
}

static int nvgpu_ioctl_free_profiler_object(
				struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_profiler_obj_mgt_args *args)
{
	int err = 0;
	struct gk20a *g = get_gk20a(dbg_s->dev);
	struct dbg_profiler_object_data *prof_obj, *tmp_obj;
	bool obj_found = false;

	gk20a_dbg_fn("%s session_id = %d profiler_handle = %x",
		     dev_name(dbg_s->dev), dbg_s->id, args->profiler_handle);

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	/* Remove profiler object from the list, if a match is found */
	list_for_each_entry_safe(prof_obj, tmp_obj, &g->profiler_objects,
							prof_obj_entry) {
		if (prof_obj->prof_handle == args->profiler_handle) {
			if (prof_obj->session_id != dbg_s->id) {
				gk20a_err(dev_from_gk20a(g),
						"invalid handle %x",
						args->profiler_handle);
				err = -EINVAL;
				break;
			}
			if (prof_obj->has_reservation)
				g->ops.dbg_session_ops.
				  release_profiler_reservation(dbg_s, prof_obj);
			list_del(&prof_obj->prof_obj_entry);
			kfree(prof_obj);
			obj_found = true;
			break;
		}
	}
	if (!obj_found) {
		gk20a_err(dev_from_gk20a(g), "profiler %x not found",
							args->profiler_handle);
		err = -EINVAL;
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return  err;
}

static struct dbg_profiler_object_data *find_matching_prof_obj(
						struct dbg_session_gk20a *dbg_s,
						u32 profiler_handle)
{
	struct gk20a *g = dbg_s->g;
	struct dbg_profiler_object_data *prof_obj;

	list_for_each_entry(prof_obj, &g->profiler_objects, prof_obj_entry) {
		if (prof_obj->prof_handle == profiler_handle) {
			if (prof_obj->session_id != dbg_s->id) {
				gk20a_err(dev_from_gk20a(g),
						"invalid handle %x",
						profiler_handle);
				return NULL;
			}
			return prof_obj;
		}
	}
	return NULL;
}

static bool nvgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	if (g->profiler_reservation_count == 0) {
		g->global_profiler_reservation_held = true;
		g->profiler_reservation_count = 1;
		dbg_s->has_profiler_reservation = true;
		prof_obj->has_reservation = true;
		return true;
	}
	return false;
}

static bool nvgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	/* Assumes that we've already checked that no global reservation
	 * is in effect.
	 */
	g->profiler_reservation_count++;
	dbg_s->has_profiler_reservation = true;
	prof_obj->has_reservation = true;
	return true;
}

static void nvgpu_release_profiler_reservation(struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	g->profiler_reservation_count--;
	if (g->profiler_reservation_count < 0)
		gk20a_err(dev_from_gk20a(g), "Negative reservation count!");
	dbg_s->has_profiler_reservation = false;
	prof_obj->has_reservation = false;
	if (prof_obj->ch == NULL)
		g->global_profiler_reservation_held = false;
}

static int nvgpu_profiler_reserve_acquire(struct dbg_session_gk20a *dbg_s,
								u32 profiler_handle)
{
	struct gk20a *g = dbg_s->g;
	struct dbg_profiler_object_data *prof_obj, *my_prof_obj;
	int err = 0;

	gk20a_dbg_fn("%s profiler_handle = %x", dev_name(dbg_s->dev), profiler_handle);

	if (g->profiler_reservation_count < 0) {
		gk20a_err(dev_from_gk20a(g), "Negative reservation count!");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	/* Find matching object. */
	my_prof_obj = find_matching_prof_obj(dbg_s, profiler_handle);

	if (!my_prof_obj) {
		gk20a_err(dev_from_gk20a(g), "object not found");
		err = -EINVAL;
		goto exit;
	}

	/* If we already have the reservation, we're done */
	if (my_prof_obj->has_reservation) {
		err = 0;
		goto exit;
	}

	if (my_prof_obj->ch == NULL) {
		/* Global reservations are only allowed if there are no other
		 * global or per-context reservations currently held
		 */
		if (!g->ops.dbg_session_ops.check_and_set_global_reservation(
							dbg_s, my_prof_obj)) {
			gk20a_err(dev_from_gk20a(g),
				"global reserve: have existing reservation");
			err =  -EBUSY;
		}
	} else if (g->global_profiler_reservation_held) {
		/* If there's a global reservation,
		 * we can't take a per-context one.
		 */
		gk20a_err(dev_from_gk20a(g),
			"per-ctxt reserve: global reservation in effect");
		err = -EBUSY;
	} else if (gk20a_is_channel_marked_as_tsg(my_prof_obj->ch)) {
		/* TSG: check that another channel in the TSG
		 * doesn't already have the reservation
		 */
		int my_tsgid = my_prof_obj->ch->tsgid;

		list_for_each_entry(prof_obj, &g->profiler_objects,
							prof_obj_entry) {
			if (prof_obj->has_reservation &&
					(prof_obj->ch->tsgid == my_tsgid)) {
				gk20a_err(dev_from_gk20a(g),
				    "per-ctxt reserve (tsg): already reserved");
				err = -EBUSY;
				goto exit;
			}
		}

		if (!g->ops.dbg_session_ops.check_and_set_context_reservation(
							dbg_s, my_prof_obj)) {
			/* Another guest OS has the global reservation */
			gk20a_err(dev_from_gk20a(g),
				"per-ctxt reserve: global reservation in effect");
			err = -EBUSY;
		}
	} else {
		/* channel: check that some other profiler object doesn't
		 * already have the reservation.
		 */
		struct channel_gk20a *my_ch = my_prof_obj->ch;

		list_for_each_entry(prof_obj, &g->profiler_objects,
							prof_obj_entry) {
			if (prof_obj->has_reservation &&
						(prof_obj->ch == my_ch)) {
				gk20a_err(dev_from_gk20a(g),
				    "per-ctxt reserve (ch): already reserved");
				err = -EBUSY;
				goto exit;
			}
		}

		if (!g->ops.dbg_session_ops.check_and_set_context_reservation(
							dbg_s, my_prof_obj)) {
			/* Another guest OS has the global reservation */
			gk20a_err(dev_from_gk20a(g),
				"per-ctxt reserve: global reservation in effect");
			err = -EBUSY;
		}
	}
exit:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return err;
}

static int nvgpu_profiler_reserve_release(struct dbg_session_gk20a *dbg_s,
								u32 profiler_handle)
{
	struct gk20a *g = dbg_s->g;
	struct dbg_profiler_object_data *prof_obj;
	int err = 0;

	gk20a_dbg_fn("%s profiler_handle = %x", dev_name(dbg_s->dev), profiler_handle);

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	/* Find matching object. */
	prof_obj = find_matching_prof_obj(dbg_s, profiler_handle);

	if (!prof_obj) {
		gk20a_err(dev_from_gk20a(g), "object not found");
		err = -EINVAL;
		goto exit;
	}

	if (prof_obj->has_reservation)
		g->ops.dbg_session_ops.release_profiler_reservation(dbg_s, prof_obj);
	else {
		gk20a_err(dev_from_gk20a(g), "No reservation found");
		err = -EINVAL;
		goto exit;
	}
exit:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return err;
}

static int nvgpu_ioctl_profiler_reserve(struct dbg_session_gk20a *dbg_s,
			   struct nvgpu_dbg_gpu_profiler_reserve_args *args)
{
	if (args->acquire)
		return nvgpu_profiler_reserve_acquire(dbg_s, args->profiler_handle);

	return nvgpu_profiler_reserve_release(dbg_s, args->profiler_handle);
}

static int gk20a_perfbuf_map(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_map_args *args)
{
	struct gk20a *g = dbg_s->g;
	int err;
	u32 virt_size;
	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;

	if (!g->allow_all)
		return -EACCES;

	err = gk20a_vm_map_buffer(&g->mm.pmu.vm,
			args->dmabuf_fd,
			&args->offset,
			0,
			0,
			0,
			args->mapping_size,
			NULL);
	if (err)
		return err;

	/* perf output buffer may not cross a 4GB boundary - with a separate va
	 * smaller than that, it won't */
	virt_size = u64_lo32(args->mapping_size);
	virt_addr_lo = u64_lo32(args->offset);
	virt_addr_hi = u64_hi32(args->offset);
	/* but check anyway */
	if (args->offset + virt_size > SZ_4G) {
		err = -EINVAL;
		goto fail_unmap;
	}

	err = gk20a_busy(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to poweron");
		goto fail_unmap;
	}

	/* address and size are aligned to 32 bytes, the lowest bits read back
	 * as zeros */
	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), virt_size);

	/* this field is aligned to 4K */
	inst_pa_page = gk20a_mm_inst_block_addr(g, &g->mm.hwpm.inst_block) >> 12;

	/* A write to MEM_BLOCK triggers the block bind operation. MEM_BLOCK
	 * should be written last */
	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(inst_pa_page) |
			perf_pmasys_mem_block_valid_true_f() |
			perf_pmasys_mem_block_target_lfb_f());

	gk20a_idle(g);

	return 0;

fail_unmap:
	gk20a_vm_unmap_buffer(&g->mm.pmu.vm, args->offset, NULL);
	return err;
}

static int gk20a_perfbuf_unmap(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_unmap_args *args)
{
	struct gk20a *g = dbg_s->g;
	int err;

	if (!g->allow_all)
		return -EACCES;

	err = gk20a_busy(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to poweron");
		return err;
	}

	gk20a_writel(g, perf_pmasys_outbase_r(), 0);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(0));
	gk20a_writel(g, perf_pmasys_outsize_r(), 0);

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(0) |
			perf_pmasys_mem_block_valid_false_f() |
			perf_pmasys_mem_block_target_f(0));

	gk20a_idle(g);

	gk20a_vm_unmap_buffer(&g->mm.pmu.vm, args->offset, NULL);

	return 0;
}

void gk20a_init_dbg_session_ops(struct gpu_ops *gops)
{
	gops->dbg_session_ops.exec_reg_ops = exec_regops_gk20a;
	gops->dbg_session_ops.dbg_set_powergate = dbg_set_powergate;
	gops->dbg_session_ops.check_and_set_global_reservation =
					nvgpu_check_and_set_global_reservation;
	gops->dbg_session_ops.check_and_set_context_reservation =
					nvgpu_check_and_set_context_reservation;
	gops->dbg_session_ops.release_profiler_reservation =
					nvgpu_release_profiler_reservation;
};
