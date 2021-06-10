/*
 * dev.c: Device interface for tegradc ext.
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
 * Some code based on fbdev extensions written by:
 *	Erik Gilling <konkers@android.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/version.h>
#include <video/tegra_dc_ext.h>
#include <trace/events/display.h>

/* XXX ew */
#include "../dc.h"
#include "../dc_priv.h"
#include "../dc_config.h"
/* XXX ew 3 */
#include "tegra_dc_ext_priv.h"
/* XXX ew 4 */
#ifdef CONFIG_TEGRA_GRHOST_SYNC
#include "../drivers/staging/android/sync.h"
#endif

#include "../edid.h"

#define TEGRA_DC_TS_MAX_DELAY_US 1000000
#define TEGRA_DC_TS_SLACK_US 2000

/* Compatibility for kthread refactoring */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
#define kthread_init_work init_kthread_work
#define kthread_init_worker init_kthread_worker
#define kthread_queue_work queue_kthread_work
#define kthread_flush_worker flush_kthread_worker
#endif

#ifdef CONFIG_COMPAT
/* compat versions that happen to be the same size as the uapi version. */

struct tegra_dc_ext_lut32 {
	__u32 win_index;	/* window index to set lut for */
	__u32 flags;		/* Flag bitmask, see TEGRA_DC_EXT_LUT_FLAGS_* */
	__u32 start;		/* start index to update lut from */
	__u32 len;		/* number of valid lut entries */
	__u32 r;		/* array of 16-bit red values, 0 to reset */
	__u32 g;		/* array of 16-bit green values, 0 to reset */
	__u32 b;		/* array of 16-bit blue values, 0 to reset */
};

#define TEGRA_DC_EXT_SET_LUT32 \
		_IOW('D', 0x0A, struct tegra_dc_ext_lut32)

struct tegra_dc_ext_feature32 {
	__u32 length;
	__u32 entries;		/* pointer to array of 32-bit entries */
};

#define TEGRA_DC_EXT_GET_FEATURES32 \
		_IOW('D', 0x0B, struct tegra_dc_ext_feature32)

struct tegra_dc_ext_flip_2_32 {
	__u32 __user win;	/* struct tegra_dc_ext_flip_windowattr */
	__u8 win_num;
	__u8 reserved1;		/* unused - must be 0 */
	__u16 reserved2;	/* unused - must be 0 */
	__u32 post_syncpt_id;
	__u32 post_syncpt_val;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
};

#define TEGRA_DC_EXT_FLIP2_32 \
		_IOWR('D', 0x0E, struct tegra_dc_ext_flip_2_32)

#define TEGRA_DC_EXT_SET_PROPOSED_BW32 \
		_IOR('D', 0x13, struct tegra_dc_ext_flip_2_32)

#endif

dev_t tegra_dc_ext_devno;
struct class *tegra_dc_ext_class;
static int head_count;
static atomic_t dc_open_count;

struct tegra_dc_ext_flip_win {
	struct tegra_dc_ext_flip_windowattr_v2	attr;
	struct tegra_dc_dmabuf			*handle[TEGRA_DC_NUM_PLANES];
	dma_addr_t				phys_addr;
	dma_addr_t				phys_addr_u;
	dma_addr_t				phys_addr_v;
	dma_addr_t				phys_addr_cde;
	/* field 2 */
	dma_addr_t				phys_addr2;
	dma_addr_t				phys_addr_u2;
	dma_addr_t				phys_addr_v2;
	u32					syncpt_max;
#ifdef CONFIG_TEGRA_GRHOST_SYNC
	struct sync_fence			*pre_syncpt_fence;
#endif
	bool					user_csc_v2;
	struct tegra_dc_ext_csc_v2		csc_v2;
};

struct tegra_dc_ext_flip_data {
	struct tegra_dc_ext		*ext;
	struct kthread_work		work;
	struct tegra_dc_ext_flip_win	win[DC_N_WINDOWS];
	struct list_head		timestamp_node;
	int act_window_num;
	u16 dirty_rect[4];
	bool dirty_rect_valid;
	u8 flags;
	struct tegra_dc_hdr hdr_data;
	bool hdr_cache_dirty;
	bool imp_dirty;
	u64 imp_session_id;
	bool cmu_update_needed; /* this is needed for ENABLE or DISABLE */
	bool new_cmu_values; /* this is used only when ENABLE */
	struct tegra_dc_ext_cmu_v2 user_cmu_v2;
	bool output_colorspace_update_needed;
	bool output_range_update_needed;
	u32 output_colorspace;
	u8 limited_range_enable;
};

struct tegra_dc_ext_scanline_data {
	struct tegra_dc_ext		*ext;
	struct kthread_work		scanline_work;
	int				triggered_line;
	int				max_val;
};

static int tegra_dc_ext_set_vblank(struct tegra_dc_ext *ext, bool enable);
static void tegra_dc_ext_unpin_window(struct tegra_dc_ext_win *win);
static void tegra_dc_flip_trace(struct tegra_dc_ext_flip_data *data,
				display_syncpt_notifier trace_fn);

static inline s64 tegra_timespec_to_ns(const struct tegra_timespec *ts)
{
	return ((s64) ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

static inline int test_bit_u32(int bitnum, const u32 *data, int entries)
{
	int i;

	for (i = 0; i < entries; i++) {
		if (bitnum < 32) {
			if (1UL & (data[bitnum / 32] >> (bitnum & 31)))
				return 1;
		} else {
			bitnum -= 32;
			data++;
		}
	}

	return 0;
}

int tegra_dc_ext_get_num_outputs(void)
{
	/* TODO: decouple output count from head count */
	return head_count;
}

static int tegra_dc_ext_get_window(struct tegra_dc_ext_user *user,
				   unsigned int n)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc_ext_win *win;
	int ret = 0;

	if ((n >= tegra_dc_get_numof_dispwindows()) ||
		!(ext->dc->valid_windows & BIT(n)))
		return -EINVAL;

	win = &ext->win[n];

	mutex_lock(&win->lock);

	if (!win->user) {
		win->user = user;
		win->enabled = false;
	} else {
		if (win->user != user)
			ret = -EBUSY;
	}
	mutex_unlock(&win->lock);

	return ret;
}

static int tegra_dc_ext_put_window(struct tegra_dc_ext_user *user,
				   unsigned int n)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc_ext_win *win;
	int ret = 0;

	if ((n >= tegra_dc_get_numof_dispwindows()) ||
		!(ext->dc->valid_windows & BIT(n)))
		return -EINVAL;

	win = &ext->win[n];

	mutex_lock(&win->lock);

	if (win->user == user) {
		kthread_flush_worker(&win->flip_worker);
		win->user = NULL;
		win->enabled = false;
	} else {
		ret = -EACCES;
	}

	mutex_unlock(&win->lock);

	return ret;
}

static unsigned long tegra_dc_ext_get_winmask(struct tegra_dc_ext_user *user)
{
	return user->ext->dc->valid_windows;
}

static int tegra_dc_ext_set_winmask(struct tegra_dc_ext_user *user,
					unsigned long winmask)
{
	if (!user || !user->ext || !user->ext->dc)
		return -EINVAL;

	return tegra_dc_update_winmask(user->ext->dc, winmask);
}

int tegra_dc_ext_restore(struct tegra_dc_ext *ext)
{
	int nwins = tegra_dc_get_numof_dispwindows();
	struct tegra_dc_win *wins[nwins];
	int i, nr_win = 0;

	for_each_set_bit(i, &ext->dc->valid_windows,
			tegra_dc_get_numof_dispwindows())
		if (ext->win[i].enabled) {
			wins[nr_win] = tegra_dc_get_window(ext->dc, i);
			wins[nr_win++]->flags |= TEGRA_WIN_FLAG_ENABLED;
		}

	if (nr_win) {
		tegra_dc_update_windows(&wins[0], nr_win, NULL, true);
		tegra_dc_sync_windows(&wins[0], nr_win);
		tegra_dc_program_bandwidth(ext->dc, true);
	}

	return nr_win;
}

static void set_enable(struct tegra_dc_ext *ext, bool en)
{
	int i;

	/*
	 * Take all locks to make sure any flip requests or cursor moves are
	 * out of their critical sections
	 */
	for (i = 0; i < ext->dc->n_windows; i++)
		mutex_lock_nested(&ext->win[i].lock, i);
	mutex_lock(&ext->cursor.lock);

	ext->enabled = en;

	mutex_unlock(&ext->cursor.lock);
	for (i = ext->dc->n_windows - 1; i >= 0 ; i--)
		mutex_unlock(&ext->win[i].lock);
}

void tegra_dc_ext_enable(struct tegra_dc_ext *ext)
{
	/* Clear trigger line value */
	ext->scanline_trigger = -1;
	/* Update min val all the way to max. */
	nvhost_syncpt_set_min_eq_max_ext(ext->dc->ndev,
					ext->dc->vpulse3_syncpt);

	set_enable(ext, true);
}

int tegra_dc_ext_disable(struct tegra_dc_ext *ext)
{
	int i;
	unsigned long int windows = 0;

	set_enable(ext, false);

	/* Flush any scanline work */
	kthread_flush_worker(&ext->scanline_worker);

	/*
	 * Disable vblank requests
	 */
	tegra_dc_ext_set_vblank(ext, false);

	/*
	 * Flush the flip queue -- note that this must be called with dc->lock
	 * unlocked or else it will hang.
	 */
	for (i = 0; i < ext->dc->n_windows; i++) {
		struct tegra_dc_ext_win *win = &ext->win[i];

		kthread_flush_worker(&win->flip_worker);
	}

	/*
	 * Blank all windows owned by dcext driver, unpin buffers that were
	 * removed from screen, and advance syncpt.
	 */
	if (ext->dc->enabled) {
		for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
			if (ext->win[i].user)
				windows |= BIT(i);
		}

		tegra_dc_blank_wins(ext->dc, windows);
		if (!IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE)) {
			for_each_set_bit(i, &windows,
					tegra_dc_get_numof_dispwindows()) {
				tegra_dc_ext_unpin_window(&ext->win[i]);
			}
		}
	}

	return !!windows;
}

static int tegra_dc_ext_check_windowattr(struct tegra_dc_ext *ext,
						struct tegra_dc_win *win)
{
	u32 *p_data;
	struct tegra_dc *dc = ext->dc;

	p_data = tegra_dc_parse_feature(dc, win->idx, GET_WIN_FORMATS);
	/* Check if the window exists */
	if (!p_data) {
		dev_err(&dc->ndev->dev,
			"Window %d is not found.\n", win->idx);
		goto fail;
	}
	/* Check the window format */
	if (!test_bit_u32(tegra_dc_fmt(win->fmt), p_data, ENTRY_SIZE)) {
		dev_err(&dc->ndev->dev,
			"Color format of window %d is invalid: %u.\n",
			win->idx, tegra_dc_fmt(win->fmt));
		goto fail;
	}

	/* Check window size */
	p_data = tegra_dc_parse_feature(dc, win->idx, GET_WIN_SIZE);
	if (CHECK_SIZE(win->out_w, p_data[MIN_WIDTH], p_data[MAX_WIDTH]) ||
		CHECK_SIZE(win->out_h, p_data[MIN_HEIGHT], p_data[MAX_HEIGHT])) {
		dev_err(&dc->ndev->dev,
			"Size of window %d is invalid with %d wide %d high.\n",
			win->idx, win->out_w, win->out_h);
		goto fail;
	}

	if (win->flags & TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR) {
		if (win->flags & TEGRA_DC_EXT_FLIP_FLAG_TILED) {
			dev_err(&dc->ndev->dev, "Layout cannot be both "
				"blocklinear and tile for window %d.\n",
				win->idx);
			goto fail;
		}

		/* TODO: also check current window blocklinear support */
	}

	if ((win->flags & TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN) &&
		!tegra_dc_feature_has_scan_column(dc, win->idx)) {
		dev_err(&dc->ndev->dev,
			"Rotation not supported for window %d.\n", win->idx);
		goto fail;
	}

	/* our interface doesn't support both compression and interlace,
	 * even if the HW can do it. */
	if ((win->flags & TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED) &&
		(win->flags & TEGRA_DC_EXT_FLIP_FLAG_INTERLACE)) {
		dev_err(&dc->ndev->dev,
			"Compression and interlace can't both be on win %d.\n",
			win->idx);
		goto fail;
	}

	return 0;
fail:
	return -EINVAL;
}

static void tegra_dc_ext_set_windowattr_basic(struct tegra_dc_win *win,
		       const struct tegra_dc_ext_flip_windowattr_v2 *flip_win)
{
	win->flags = TEGRA_WIN_FLAG_ENABLED;
	if (flip_win->blend == TEGRA_DC_EXT_BLEND_PREMULT)
		win->flags |= TEGRA_WIN_FLAG_BLEND_PREMULT;
	else if (flip_win->blend == TEGRA_DC_EXT_BLEND_COVERAGE)
		win->flags |= TEGRA_WIN_FLAG_BLEND_COVERAGE;
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_TILED)
		win->flags |= TEGRA_WIN_FLAG_TILED;
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INVERT_H)
		win->flags |= TEGRA_WIN_FLAG_INVERT_H;
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INVERT_V)
		win->flags |= TEGRA_WIN_FLAG_INVERT_V;
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA)
		win->global_alpha = flip_win->global_alpha;
	else
		win->global_alpha = 255;
#if defined(CONFIG_TEGRA_DC_SCAN_COLUMN)
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN)
		win->flags |= TEGRA_WIN_FLAG_SCAN_COLUMN;
#endif
#if defined(CONFIG_TEGRA_DC_BLOCK_LINEAR)
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR) {
		win->flags |= TEGRA_WIN_FLAG_BLOCKLINEAR;
		win->block_height_log2 = flip_win->block_height_log2;
	}
#endif
#if defined(CONFIG_TEGRA_DC_INTERLACE)
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INTERLACE)
		win->flags |= TEGRA_WIN_FLAG_INTERLACE;
#endif

#if defined(CONFIG_TEGRA_DC_CDE)
	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED) {
		win->cde.zbc_color = flip_win->cde.zbc_color;
		win->cde.offset_x = flip_win->cde.offset_x;
		win->cde.offset_y = flip_win->cde.offset_y;
		win->cde.ctb_entry = 0x02;
	}
#endif

	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_LIMITED)
		win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_LIMITED;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_BYPASS)
		win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_BYPASS;
	else
		win->flags |= TEGRA_WIN_FLAG_INPUT_RANGE_FULL;

	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC601)
		win->flags |= TEGRA_WIN_FLAG_CS_REC601;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC709)
		win->flags |= TEGRA_WIN_FLAG_CS_REC709;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020)
		win->flags |= TEGRA_WIN_FLAG_CS_REC2020;
	else
		win->flags |= TEGRA_WIN_FLAG_CS_DEFAULT;

	if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_NONE)
		win->flags |= TEGRA_WIN_FLAG_DEGAMMA_NONE;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_SRGB)
		win->flags |= TEGRA_WIN_FLAG_DEGAMMA_SRGB;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_8_10)
		win->flags |= TEGRA_WIN_FLAG_DEGAMMA_YUV_8_10;
	else if (flip_win->flags & TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_12)
		win->flags |= TEGRA_WIN_FLAG_DEGAMMA_YUV_12;

	win->fmt = flip_win->pixformat;
	win->x.full = flip_win->x;
	win->y.full = flip_win->y;
	win->w.full = flip_win->w;
	win->h.full = flip_win->h;
	/* XXX verify that this doesn't go outside display's active region */
	win->out_x = flip_win->out_x;
	win->out_y = flip_win->out_y;
	win->out_w = flip_win->out_w;
	win->out_h = flip_win->out_h;
	win->z = flip_win->z;

	win->stride = flip_win->stride;
	win->stride_uv = flip_win->stride_uv;
}



static int tegra_dc_ext_set_windowattr(struct tegra_dc_ext *ext,
			       struct tegra_dc_win *win,
			       const struct tegra_dc_ext_flip_win *flip_win)
{
	int err = 0;
	struct tegra_dc_ext_win *ext_win = &ext->win[win->idx];
	s64 timestamp_ns;
	struct tegra_vrr *vrr = ext->dc->out->vrr;

	if (flip_win->handle[TEGRA_DC_Y] == NULL) {
		win->flags = 0;
		memset(ext_win->cur_handle, 0, sizeof(ext_win->cur_handle));
		return 0;
	}

	tegra_dc_ext_set_windowattr_basic(win, &flip_win->attr);

	memcpy(ext_win->cur_handle, flip_win->handle,
	       sizeof(ext_win->cur_handle));

	/* XXX verify that this won't read outside of the surface */
	win->phys_addr = flip_win->phys_addr + flip_win->attr.offset;

	win->phys_addr_u = flip_win->handle[TEGRA_DC_U] ?
		flip_win->phys_addr_u : flip_win->phys_addr;
	win->phys_addr_u += flip_win->attr.offset_u;

	win->phys_addr_v = flip_win->handle[TEGRA_DC_V] ?
		flip_win->phys_addr_v : flip_win->phys_addr;
	win->phys_addr_v += flip_win->attr.offset_v;

#if defined(CONFIG_TEGRA_DC_INTERLACE)
	if (ext->dc->mode.vmode == FB_VMODE_INTERLACED) {
		if (flip_win->attr.flags & TEGRA_DC_EXT_FLIP_FLAG_INTERLACE) {
			win->phys_addr2 = flip_win->phys_addr +
				flip_win->attr.offset2;

			win->phys_addr_u2 = flip_win->handle[TEGRA_DC_U] ?
				flip_win->phys_addr_u : flip_win->phys_addr;

			win->phys_addr_u2 += flip_win->attr.offset_u2;

			win->phys_addr_v2 = flip_win->handle[TEGRA_DC_V] ?
				flip_win->phys_addr_v : flip_win->phys_addr;
			win->phys_addr_v2 += flip_win->attr.offset_v2;
		} else {
			win->phys_addr2 = flip_win->phys_addr;

			win->phys_addr_u2 = flip_win->handle[TEGRA_DC_U] ?
				flip_win->phys_addr_u : flip_win->phys_addr;

			win->phys_addr_v2 = flip_win->handle[TEGRA_DC_V] ?
				flip_win->phys_addr_v : flip_win->phys_addr;
		}
	}
#endif

#if defined(CONFIG_TEGRA_DC_CDE)
	if (flip_win->attr.flags & TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED)
		win->cde.cde_addr =
			flip_win->phys_addr_cde + flip_win->attr.cde.offset;
	else
		win->cde.cde_addr = 0;
#endif

	err = tegra_dc_ext_check_windowattr(ext, win);
	if (err < 0)
		dev_err(&ext->dc->ndev->dev,
				"Window atrributes are invalid.\n");

#ifdef CONFIG_TEGRA_GRHOST_SYNC
	if (flip_win->pre_syncpt_fence) {
		sync_fence_wait(flip_win->pre_syncpt_fence, 5000);
		sync_fence_put(flip_win->pre_syncpt_fence);
	} else
#endif
	if ((s32)flip_win->attr.pre_syncpt_id >= 0) {
		nvhost_syncpt_wait_timeout_ext(ext->dc->ndev,
				flip_win->attr.pre_syncpt_id,
				flip_win->attr.pre_syncpt_val,
				msecs_to_jiffies(5000), NULL, NULL);
	}

	if (err < 0)
		return err;

	if (tegra_platform_is_silicon()) {
		timestamp_ns = tegra_timespec_to_ns(&flip_win->attr.timestamp);

		if (timestamp_ns) {
			/* XXX: Should timestamping be overridden by "no_vsync"
			 * flag */
			if (vrr && vrr->enable) {
				struct timespec tm;
				s64 now_ns = 0;
				s32 sleep_us = 0;
				ktime_get_ts(&tm);
				now_ns = timespec_to_ns(&tm);
				sleep_us = (s32)div_s64(timestamp_ns -
					now_ns, 1000ll);

				if (sleep_us > TEGRA_DC_TS_MAX_DELAY_US)
					sleep_us = TEGRA_DC_TS_MAX_DELAY_US;

				if (sleep_us > 0)
					usleep_range(sleep_us, sleep_us +
						TEGRA_DC_TS_SLACK_US);
			} else {
				tegra_dc_config_frame_end_intr(win->dc, true);
				err = wait_event_interruptible(
					win->dc->timestamp_wq,
					tegra_dc_is_within_n_vsync(win->dc,
						timestamp_ns));
				tegra_dc_config_frame_end_intr(win->dc, false);
			}
		}
	}

	return err;
}

static int tegra_dc_ext_should_show_background(
		struct tegra_dc_ext_flip_data *data,
		int win_num)
{
	struct tegra_dc *dc = data->ext->dc;
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	int i;

	if (!dc->yuv_bypass || yuv_flag != (FB_VMODE_Y420 | FB_VMODE_Y30))
		return false;

	for (i = 0; i < win_num; i++) {
		struct tegra_dc_ext_flip_win *flip_win = &data->win[i];
		int index = flip_win->attr.index;

		if (index < 0 || !test_bit(index, &dc->valid_windows))
			continue;

		if (flip_win->handle[TEGRA_DC_Y] == NULL)
			continue;

		/* Bypass expects only a single window, thus it is enough to
		 * inspect the first enabled window.
		 *
		 * Full screen input surface for YUV420 10-bit 4k has 2400x2160
		 * active area. dc->mode is already adjusted to this dimension.
		 */
		if (flip_win->attr.out_x > 0 ||
		    flip_win->attr.out_y > 0 ||
		    flip_win->attr.out_w != dc->mode.h_active ||
		    flip_win->attr.out_h != dc->mode.v_active)
			return true;
	}

	return false;
}

static int tegra_dc_ext_get_background(struct tegra_dc_ext *ext,
		struct tegra_dc_win *win)
{
	struct tegra_dc *dc = ext->dc;
	u32 active_width = dc->mode.h_active;
	u32 active_height = dc->mode.v_active;

	*win = *tegra_fb_get_blank_win(dc->fb);

	win->flags |= TEGRA_WIN_FLAG_ENABLED;
	win->fmt = TEGRA_WIN_FMT_B8G8R8A8;
	win->x.full = dfixed_const(0);
	win->y.full = dfixed_const(0);
	win->h.full = dfixed_const(1);
	win->w.full = dfixed_const(active_width);
	win->out_x = 0;
	win->out_y = 0;
	win->out_w = active_width;
	win->out_h = active_height;
	win->z = 0xff;

	return 0;
}

static int tegra_dc_ext_set_vblank(struct tegra_dc_ext *ext, bool enable)
{
	struct tegra_dc *dc;
	int ret = 0;

	if (ext->vblank_enabled == enable)
		return 0;

	dc = ext->dc;

	if (enable)
		ret = tegra_dc_vsync_enable(dc);
	else if (ext->vblank_enabled)
		tegra_dc_vsync_disable(dc);

	if (!ret) {
		ext->vblank_enabled = enable;
		return 0;
	}
	return 1;
}

static void tegra_dc_ext_setup_vpulse3(struct tegra_dc_ext *ext,
					unsigned int scanline_num)
{
	struct tegra_dc *dc = ext->dc;
	u32 old_state;

	tegra_dc_get(dc);
	mutex_lock(&dc->lock);
	old_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);

	/* write active version; this updates assembly as well */
	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
			DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, scanline_num, DC_DISP_V_PULSE3_POSITION_A);

	tegra_dc_writel(dc, old_state, DC_CMD_STATE_ACCESS);
	tegra_dc_readl(dc, DC_CMD_STATE_ACCESS); /* flush */
	mutex_unlock(&dc->lock);
	tegra_dc_put(dc);
}

static void tegra_dc_ext_incr_vpulse3(struct tegra_dc_ext *ext)
{
	struct tegra_dc *dc = ext->dc;
	unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;

	/* Request vpulse3 syncpt increment */
	tegra_dc_writel(dc, VPULSE3_COND | vpulse3_sync_id,
				DC_CMD_GENERAL_INCR_SYNCPT);
}

static void tegra_dc_ext_scanline_worker(struct kthread_work *work)
{
	struct tegra_dc_ext_scanline_data *data =
		container_of(work, struct tegra_dc_ext_scanline_data,
				scanline_work);
	struct tegra_dc_ext *ext = data->ext;
	struct tegra_dc *dc = ext->dc;
	unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;
	int min_val;

	/* Wait for frame end before programming new request */
	_tegra_dc_wait_for_frame_end(dc,
		div_s64(dc->frametime_ns, 1000000ll) * 2);

	if (data->triggered_line >= 0) {
		if (ext->scanline_trigger != data->triggered_line) {
			dev_dbg(&dc->ndev->dev, "vp3 sl#: %d\n",
					data->triggered_line);

			/* Setup vpulse3 trigger line */
			tegra_dc_ext_setup_vpulse3(ext, data->triggered_line);

			/* Save new scanline trigger state */
			ext->scanline_trigger = data->triggered_line;
		}

		/* Request vpulse3 increment */
		tegra_dc_ext_incr_vpulse3(ext);
	} else {
		/* Clear scanline trigger state */
		ext->scanline_trigger = -1;

		/* Read current min_val */
		min_val = nvhost_syncpt_read_minval(dc->ndev, vpulse3_sync_id);

		/* Udate min_val to expected max only if they don't match */
		if (min_val != data->max_val) {
			dev_dbg(&dc->ndev->dev, "vp3 mismatch; %d vs %d",
						min_val, data->max_val);

			nvhost_syncpt_set_minval(dc->ndev, vpulse3_sync_id,
						data->max_val);
		}
	}

	/* Free arg allocated in the ioctl call */
	kfree(data);
}

int tegra_dc_ext_vpulse3(struct tegra_dc_ext *ext,
				struct tegra_dc_ext_scanline_info *args)
{
	int ret = -EFAULT;
	struct tegra_dc *dc = ext->dc;
	struct tegra_dc_ext_scanline_data *data;
	unsigned int vpulse3_sync_id = dc->vpulse3_syncpt;
	unsigned int vpulse3_sync_max_val = 0;

	/* If display has been disconnected OR
	 * Vpulse3 syncpt is invalid OR
	 * scanline workqueue is not setup, return error
	 */
	if (!dc->enabled || !dc->connected ||
		vpulse3_sync_id == NVSYNCPT_INVALID) {
		dev_err(&dc->ndev->dev, "DC not setup\n");
		return -EACCES;
	}

	if (!ext->enabled) {
		dev_err(&dc->ndev->dev, "DC ext is disabled\n");
		return -ENXIO;
	}

	/* TODO: Support id != 0 */
	if (args->id) {
		dev_err(&dc->ndev->dev, "Invalid scanline id\n");
		return -EINVAL;
	}

	/* TODO: Support frame != 0 and raw syncpt */
	if ((args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE) &&
		(args->frame ||
		args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_RAW_SYNCPT)) {
		dev_err(&dc->ndev->dev, "Invalid args\n");
		return -EINVAL;
	}

	dev_dbg(&dc->ndev->dev, "vp3 id : %d\n", vpulse3_sync_id);

	/* Allocate arg for workqueue */
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (args->flags & TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE) {
		/* Increment max_val by 1 */
		vpulse3_sync_max_val =
		    nvhost_syncpt_incr_max_ext(dc->ndev, vpulse3_sync_id, 1);

		dev_dbg(&dc->ndev->dev, "vp3 max: %d\n", vpulse3_sync_max_val);

		/* Create a fencefd */
		ret = nvhost_syncpt_create_fence_single_ext(dc->ndev,
				vpulse3_sync_id, vpulse3_sync_max_val,
				"vpulse3-fence", &args->syncfd);

		if (ret) {
			dev_err(&dc->ndev->dev,
				"Failed creating vpulse3 fence err: %d\n", ret);

			kfree(data);
			return ret;
		}

		/* Pass trigger line value */
		data->triggered_line = args->triggered_line;
	} else {
		/* Clear trigger line value */
		data->triggered_line = -1;

		/* Pass current max_val */
		data->max_val = nvhost_syncpt_read_maxval(dc->ndev,
							vpulse3_sync_id);
	}

	/* Queue work to setup/increment/remove scanline trigger */
	data->ext = ext;
	kthread_init_work(&data->scanline_work, tegra_dc_ext_scanline_worker);
	kthread_queue_work(&ext->scanline_worker, &data->scanline_work);

	return 0;
}

int tegra_dc_ext_get_scanline(struct tegra_dc_ext *ext)
{
	return ext->scanline_trigger;
}

static void tegra_dc_ext_unpin_handles(struct tegra_dc_dmabuf *unpin_handles[],
				       int nr_unpin)
{
	int i;

	for (i = 0; i < nr_unpin; i++) {
		dma_buf_unmap_attachment(unpin_handles[i]->attach,
			unpin_handles[i]->sgt, DMA_TO_DEVICE);
		dma_buf_detach(unpin_handles[i]->buf,
			       unpin_handles[i]->attach);
		dma_buf_put(unpin_handles[i]->buf);
		kfree(unpin_handles[i]);
	}
}

static void tegra_dc_flip_trace(struct tegra_dc_ext_flip_data *data,
				display_syncpt_notifier trace_fn)
{
	struct tegra_dc *dc = data->ext->dc;
	struct tegra_dc_ext_flip_win *win;
	struct tegra_dc_ext_flip_windowattr_v2 *attr;
	int win_num;
	u64 timestamp;
	int i;

	timestamp = tegra_dc_get_tsc_time();

	for (i = 0; i < data->act_window_num; i++) {
		win = &data->win[i];
		attr = &win->attr;
		win_num = attr->index;

		if (win_num < 0 || !test_bit(win_num, &dc->valid_windows))
			continue;

		(*trace_fn)(dc->ctrl_num, win_num,
			win->syncpt_max, attr->buff_id, timestamp);
	}
}

static void tegra_dc_ext_flip_worker(struct kthread_work *work)
{
	struct tegra_dc_ext_flip_data *data =
		container_of(work, struct tegra_dc_ext_flip_data, work);
	int win_num = data->act_window_num;
	struct tegra_dc_ext *ext = data->ext;
	struct tegra_dc_win *wins[DC_N_WINDOWS];
	struct tegra_dc_win *blank_win;
	struct tegra_dc_dmabuf *unpin_handles[DC_N_WINDOWS *
					       TEGRA_DC_NUM_PLANES];
	struct tegra_dc_dmabuf *old_handle;
	struct tegra_dc *dc = ext->dc;
	int i, nr_unpin = 0, nr_win = 0;
	bool skip_flip = false;
	bool wait_for_vblank = false;
	bool show_background =
		tegra_dc_ext_should_show_background(data, win_num);

	blank_win = kzalloc(sizeof(*blank_win), GFP_KERNEL);
	if (!blank_win)
		dev_err(&ext->dc->ndev->dev, "Failed to allocate blank_win.\n");

	tegra_dc_scrncapt_disp_pause_lock(dc);

	BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
	for (i = 0; i < win_num; i++) {
		struct tegra_dc_ext_flip_win *flip_win = &data->win[i];
		int index = flip_win->attr.index;
		struct tegra_dc_win *win;
		struct tegra_dc_ext_win *ext_win;
		struct tegra_dc_ext_flip_data *temp = NULL;
		s64 head_timestamp = -1;
		int j = 0;
		u32 reg_val = 0;

		if (index < 0 || !test_bit(index, &dc->valid_windows))
			continue;

		win = tegra_dc_get_window(dc, index);
		if (!win)
			continue;
		ext_win = &ext->win[index];

		if (!(atomic_dec_and_test(&ext_win->nr_pending_flips)) &&
			(flip_win->attr.flags & TEGRA_DC_EXT_FLIP_FLAG_CURSOR))
			skip_flip = true;

		mutex_lock(&ext_win->queue_lock);
		list_for_each_entry(temp, &ext_win->timestamp_queue,
				timestamp_node) {
			if (!tegra_platform_is_silicon())
				continue;
			if (j == 0) {
				if (unlikely(temp != data)) {
					/* Frame doesn't contain timestamp in list */
					break;
				} else
					head_timestamp = tegra_timespec_to_ns(
						&flip_win->attr.timestamp);
			} else {
				s64 timestamp = tegra_timespec_to_ns(
					&temp->win[i].attr.timestamp);

				skip_flip = !tegra_dc_does_vsync_separate(dc,
						timestamp, head_timestamp);
				/* Look ahead only one flip */
				break;
			}
			j++;
		}
		if (head_timestamp >= 0)
			list_del(&data->timestamp_node);
		mutex_unlock(&ext_win->queue_lock);

		if (skip_flip)
			old_handle = flip_win->handle[TEGRA_DC_Y];
		else
			old_handle = ext_win->cur_handle[TEGRA_DC_Y];

		if (old_handle) {
			int j;
			for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
				if (skip_flip)
					old_handle = flip_win->handle[j];
				else
					old_handle = ext_win->cur_handle[j];

				if (!old_handle)
					continue;

				unpin_handles[nr_unpin++] = old_handle;
			}
		}

#ifdef CONFIG_TEGRA_CSC
		if ((data->win[i].attr.flags &
				TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC)
				&& !dc->yuv_bypass) {
			win->csc.yof = data->win[i].attr.csc.yof;
			win->csc.kyrgb = data->win[i].attr.csc.kyrgb;
			win->csc.kur = data->win[i].attr.csc.kur;
			win->csc.kug = data->win[i].attr.csc.kug;
			win->csc.kub = data->win[i].attr.csc.kub;
			win->csc.kvr = data->win[i].attr.csc.kvr;
			win->csc.kvg = data->win[i].attr.csc.kvg;
			win->csc.kvb = data->win[i].attr.csc.kvb;
			win->csc_dirty = true;
		}
#endif

#ifdef CONFIG_TEGRA_CSC_V2
		if ((data->win[i].attr.flags &
				TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC_V2)
				&& !ext->dc->yuv_bypass
				&& !win->force_user_csc) {
			win->csc.r2r = data->win[i].attr.csc2.r2r;
			win->csc.g2r = data->win[i].attr.csc2.g2r;
			win->csc.b2r = data->win[i].attr.csc2.b2r;
			win->csc.const2r = data->win[i].attr.csc2.const2r;
			win->csc.r2g = data->win[i].attr.csc2.r2g;
			win->csc.g2g = data->win[i].attr.csc2.g2g;
			win->csc.b2g = data->win[i].attr.csc2.b2g;
			win->csc.const2g = data->win[i].attr.csc2.const2g;
			win->csc.r2b = data->win[i].attr.csc2.r2b;
			win->csc.g2b = data->win[i].attr.csc2.g2b;
			win->csc.b2b = data->win[i].attr.csc2.b2b;
			win->csc.const2b = data->win[i].attr.csc2.const2b;
			win->csc_dirty = true;
		} else if (data->win[i].user_csc_v2 &&
			   !ext->dc->yuv_bypass &&
			   !win->force_user_csc) {
			win->csc.csc_enable = data->win[i].csc_v2.csc_enable;
			win->csc.r2r = data->win[i].csc_v2.r2r;
			win->csc.g2r = data->win[i].csc_v2.g2r;
			win->csc.b2r = data->win[i].csc_v2.b2r;
			win->csc.const2r = data->win[i].csc_v2.const2r;
			win->csc.r2g = data->win[i].csc_v2.r2g;
			win->csc.g2g = data->win[i].csc_v2.g2g;
			win->csc.b2g = data->win[i].csc_v2.b2g;
			win->csc.const2g = data->win[i].csc_v2.const2g;
			win->csc.r2b = data->win[i].csc_v2.r2b;
			win->csc.g2b = data->win[i].csc_v2.g2b;
			win->csc.b2b = data->win[i].csc_v2.b2b;
			win->csc.const2b = data->win[i].csc_v2.const2b;
			win->csc_dirty = true;
		}
#endif

		if (!skip_flip)
			tegra_dc_ext_set_windowattr(ext, win, &data->win[i]);

		if (dc->yuv_bypass) {
			reg_val = tegra_dc_readl(dc,
				DC_DISP_DISP_COLOR_CONTROL);
			reg_val &= ~CMU_ENABLE;
			tegra_dc_writel(dc, reg_val,
				DC_DISP_DISP_COLOR_CONTROL);
		}

#ifdef CONFIG_TEGRA_NVDISPLAY
		/* If we're transitioning between a bypass and a non-bypass
		 * mode, update the output CSC and chroma LPF during this flip.
		 */
		if (dc->yuv_bypass_dirty) {
			tegra_nvdisp_set_ocsc(dc, &dc->mode);
			tegra_nvdisp_set_chroma_lpf(dc);
		}
#endif

		if (dc->yuv_bypass || dc->yuv_bypass_dirty) {
			reg_val = GENERAL_UPDATE;
			tegra_dc_writel(dc, reg_val, DC_CMD_STATE_CONTROL);
			tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */

			reg_val = GENERAL_ACT_REQ;
			tegra_dc_writel(dc, reg_val, DC_CMD_STATE_CONTROL);
			tegra_dc_readl(dc, DC_CMD_STATE_CONTROL); /* flush */

			dc->yuv_bypass_dirty = false;
		}

		if (flip_win->attr.swap_interval && !no_vsync)
			wait_for_vblank = true;

		ext_win->enabled = !!(win->flags & TEGRA_WIN_FLAG_ENABLED);

		/* Hijack first disabled, scaling capable window to host
		 * the background pattern.
		 */
		if (blank_win && !ext_win->enabled && show_background &&
			tegra_dc_feature_has_scaling(ext->dc, win->idx)) {
			tegra_dc_ext_get_background(ext, blank_win);
			blank_win->idx = win->idx;
			wins[nr_win++] = blank_win;
			show_background = false;
		} else {
			wins[nr_win++] = win;
		}
	}

	/* YUV packing consumes only one window, thus there must have been
	 * free window which can host background pattern.
	 */
	BUG_ON(show_background);

	if (trace_sync_wt_ovr_syncpt_upd_enabled())
		tegra_dc_flip_trace(data, trace_sync_wt_ovr_syncpt_upd);

	if (dc->enabled && !skip_flip)
		tegra_dc_set_hdr(dc, &data->hdr_data, data->hdr_cache_dirty);

	if (dc->enabled && !skip_flip) {
		dc->blanked = false;
		if (dc->out_ops && dc->out_ops->vrr_enable)
				dc->out_ops->vrr_enable(dc,
					data->flags &
					TEGRA_DC_EXT_FLIP_HEAD_FLAG_VRR_MODE);

		if (data->imp_dirty) {
			dc->imp_dirty = true;
			tegra_dc_adjust_imp(dc, true);
		}

#ifdef CONFIG_TEGRA_NVDISPLAY
		/* call tegra_dc_update_postcomp */
		if (data->cmu_update_needed)
			tegra_nvdisp_update_per_flip_output_lut(dc,
				&data->user_cmu_v2, data->new_cmu_values);
#endif
		tegra_dc_update_windows(wins, nr_win,
			data->dirty_rect_valid ? data->dirty_rect : NULL,
			wait_for_vblank);
		/* TODO: implement swapinterval here */
		tegra_dc_sync_windows(wins, nr_win);

		if (trace_scanout_syncpt_upd_enabled())
			tegra_dc_flip_trace(data, trace_scanout_syncpt_upd);

		if (dc->out->vrr)
			trace_scanout_vrr_stats((data->win[win_num-1]).syncpt_max
							, dc->out->vrr->dcb);
		tegra_dc_program_bandwidth(dc, true);
		if (!tegra_dc_has_multiple_dc())
			tegra_dc_call_flip_callback();

		if (data->imp_dirty)
			tegra_dc_adjust_imp(dc, false);
	}

	if (data->imp_dirty)
		tegra_dc_release_common_channel(dc);

	tegra_dc_scrncapt_disp_pause_unlock(dc);

	if (!skip_flip) {
		for (i = 0; i < win_num; i++) {
			struct tegra_dc_ext_flip_win *flip_win = &data->win[i];
			int index = flip_win->attr.index;

			if (index < 0 ||
				!test_bit(index, &dc->valid_windows))
				continue;

			tegra_dc_incr_syncpt_min(dc, index,
					flip_win->syncpt_max);
		}
	}

	/* unpin and deref previous front buffers */
	tegra_dc_ext_unpin_handles(unpin_handles, nr_unpin);
#ifdef CONFIG_ANDROID
	/* now DC has submitted buffer for display, try to release fbmem */
	tegra_fb_release_fbmem(ext->dc->fb);
#endif
	kfree(data);
	kfree(blank_win);
}

static int lock_windows_for_flip(struct tegra_dc_ext_user *user,
			struct tegra_dc_ext_flip_windowattr_v2 *win_attr,
			int win_num)
{
	struct tegra_dc_ext *ext = user->ext;
	u8 idx_mask = 0;
	int i;

	BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
	for (i = 0; i < win_num; i++) {
		int index = win_attr[i].index;

		if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
			continue;

		idx_mask |= BIT(index);
	}

	for (i = 0; i < win_num; i++) {
		struct tegra_dc_ext_win *win;

		if (!(idx_mask & BIT(i)))
			continue;

		win = &ext->win[i];

		mutex_lock_nested(&win->lock, i);

		if (win->user != user)
			goto fail_unlock;
	}

	return 0;

fail_unlock:
	do {
		if (!(idx_mask & BIT(i)))
			continue;

		mutex_unlock(&ext->win[i].lock);
	} while (i--);

	return -EACCES;
}

static void unlock_windows_for_flip(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_flip_windowattr_v2 *win,
				int win_num)
{
	struct tegra_dc_ext *ext = user->ext;
	u8 idx_mask = 0;
	int i;

	BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
	for (i = 0; i < win_num; i++) {
		int index = win[i].index;

		if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
			continue;

		idx_mask |= BIT(index);
	}
	for (i = win_num - 1; i >= 0; i--) {
		if (!(idx_mask & BIT(i)))
			continue;

		mutex_unlock(&ext->win[i].lock);
	}
}

static int sanitize_flip_args(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_flip_windowattr_v2 *win,
				int win_num, __u16 **dirty_rect)
{
	int i, used_windows = 0;
	struct tegra_dc *dc = user->ext->dc;
#ifdef CONFIG_TEGRA_NVDISPLAY
	int ret = 0;
#endif

	if (win_num > tegra_dc_get_numof_dispwindows())
		return -EINVAL;

	for (i = 0; i < win_num; i++) {
		int index = win[i].index;
		fixed20_12 input_w, input_h;
		u32 w, h;

		if (index < 0)
			continue;

		if (index >= tegra_dc_get_numof_dispwindows() ||
			!test_bit(index, &dc->valid_windows))
			return -EINVAL;

		if (used_windows & BIT(index))
			return -EINVAL;

		used_windows |= BIT(index);

		/*
		 * Check that the window dimensions are non-zero. The input
		 * width and height are specified as 20.12 fixed-point numbers.
		 */
		input_w.full = win->w;
		input_h.full = win->h;
		w = dfixed_trunc(input_w);
		h = dfixed_trunc(input_h);
		if (win->buff_id != 0 &&
			(w == 0 || h == 0 ||
			win->out_w == 0 || win->out_h == 0)) {
			dev_err(&dc->ndev->dev,
			"%s: WIN %d invalid size:w=%u,h=%u,out_w=%u,out_h=%u\n",
				__func__, index, w, h, win->out_w, win->out_h);
			return -EINVAL;
		}

		/*
		 * Window output geometry including width/height + offset
		 * should not exceed hActive/vActive of current mode
		 */
		if ((win->out_w + win->out_x) > dc->mode.h_active ||
			(win->out_h + win->out_y) > dc->mode.v_active) {

			dev_err(&dc->ndev->dev,
			"Invalid out_w + out_x (%u) > hActive (%u)\n OR/AND out_h + out_y (%u) > vActive (%u)\n for WIN %d\n",
				win->out_w + win->out_x, dc->mode.h_active,
				win->out_h + win->out_y, dc->mode.v_active, i);
			return -EINVAL;
		}

#ifdef CONFIG_TEGRA_NVDISPLAY
		ret = tegra_nvdisp_verify_win_properties(dc, win);
		if (ret) {
			/* Error in window properties */
			return ret;
		}
#endif
	}

	if (!used_windows)
		return -EINVAL;

	if (*dirty_rect) {
		unsigned int xoff = (*dirty_rect)[0];
		unsigned int yoff = (*dirty_rect)[1];
		unsigned int width = (*dirty_rect)[2];
		unsigned int height = (*dirty_rect)[3];

		if ((!width && !height) ||
			dc->mode.vmode == FB_VMODE_INTERLACED ||
			!dc->out_ops ||
			!dc->out_ops->partial_update ||
			(!xoff && !yoff &&
			(width == dc->mode.h_active) &&
			(height == dc->mode.v_active))) {
			/* Partial update undesired, unsupported,
			 * or dirty_rect covers entire frame. */
			*dirty_rect = NULL;
		} else {
			if (!width || !height ||
				(xoff + width) > dc->mode.h_active ||
				(yoff + height) > dc->mode.v_active)
				return -EINVAL;

			/* Constraint 7: H/V_DISP_ACTIVE >= 16.
			 * Make sure the minimal size of dirty region is 16*16.
			 * If not, extend the dirty region. */
			if (width < 16) {
				width = (*dirty_rect)[2] = 16;
				if (xoff + width > dc->mode.h_active)
					(*dirty_rect)[0] = dc->mode.h_active -
						width;
			}
			if (height < 16) {
				height = (*dirty_rect)[3] = 16;
				if (yoff + height > dc->mode.v_active)
					(*dirty_rect)[1] = dc->mode.v_active -
						height;
			}
		}
	}

	return 0;
}

static int tegra_dc_ext_pin_windows(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_flip_windowattr_v2 *wins,
				int win_num,
				struct tegra_dc_ext_flip_win *flip_wins,
				bool *has_timestamp,
				bool syncpt_fd)
{
	int i, ret;
	struct tegra_dc *dc = user->ext->dc;

	for (i = 0; i < win_num; i++) {
		struct tegra_dc_ext_flip_win *flip_win = &flip_wins[i];
		int index = wins[i].index;

		memcpy(&flip_win->attr, &wins[i], sizeof(flip_win->attr));

		if (has_timestamp && tegra_timespec_to_ns(
			&flip_win->attr.timestamp)) {

			/* Set first frame timestamp to 0 after device boot-up
			   to prevent wait on 1st flip request */
			if (!dc->frame_end_timestamp)
				memset(&flip_win->attr.timestamp, 0,
					sizeof(flip_win->attr.timestamp));
			*has_timestamp = true;
		}

		if (index < 0 || !test_bit(index, &dc->valid_windows))
			continue;

		ret = tegra_dc_ext_pin_window(user, flip_win->attr.buff_id,
					      &flip_win->handle[TEGRA_DC_Y],
					      &flip_win->phys_addr);
		if (ret)
			return ret;

		if (flip_win->attr.buff_id_u) {
			ret = tegra_dc_ext_pin_window(user,
					      flip_win->attr.buff_id_u,
					      &flip_win->handle[TEGRA_DC_U],
					      &flip_win->phys_addr_u);
			if (ret)
				return ret;
		} else {
			flip_win->handle[TEGRA_DC_U] = NULL;
			flip_win->phys_addr_u = 0;
		}

		if (flip_win->attr.buff_id_v) {
			ret = tegra_dc_ext_pin_window(user,
					      flip_win->attr.buff_id_v,
					      &flip_win->handle[TEGRA_DC_V],
					      &flip_win->phys_addr_v);
			if (ret)
				return ret;
		} else {
			flip_win->handle[TEGRA_DC_V] = NULL;
			flip_win->phys_addr_v = 0;
		}

		if ((flip_win->attr.flags & TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED)) {
#if defined(CONFIG_TEGRA_DC_CDE)
			/* use buff_id of the main surface when cde is 0 */
			__u32 cde_buff_id = flip_win->attr.cde.buff_id;
			if (!cde_buff_id)
				cde_buff_id = flip_win->attr.buff_id;
			ret = tegra_dc_ext_pin_window(user,
					      cde_buff_id,
					      &flip_win->handle[TEGRA_DC_CDE],
					      &flip_win->phys_addr_cde);
			if (ret)
				return ret;
#else
			return -EINVAL; /* not supported */
#endif
		} else {
			flip_win->handle[TEGRA_DC_CDE] = NULL;
			flip_win->phys_addr_cde = 0;
		}

		if (syncpt_fd) {
			if (flip_win->attr.pre_syncpt_fd >= 0) {
#ifdef CONFIG_TEGRA_GRHOST_SYNC
				flip_win->pre_syncpt_fence = sync_fence_fdget(
					flip_win->attr.pre_syncpt_fd);
#else
				BUG();
#endif
			} else {
				flip_win->attr.pre_syncpt_id = NVSYNCPT_INVALID;
			}
		}
	}

	return 0;
}

static void tegra_dc_ext_unpin_window(struct tegra_dc_ext_win *win)
{
	struct tegra_dc_dmabuf *unpin_handles[TEGRA_DC_NUM_PLANES];
	int nr_unpin = 0;

	mutex_lock(&win->lock);
	if (win->cur_handle[TEGRA_DC_Y]) {
		int j;

		for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
			if (win->cur_handle[j])
				unpin_handles[nr_unpin++] = win->cur_handle[j];
		}
		memset(win->cur_handle, 0, sizeof(win->cur_handle));
	}
	mutex_unlock(&win->lock);

	tegra_dc_ext_unpin_handles(unpin_handles, nr_unpin);
}

static int tegra_dc_ext_configure_cmu_v2_user_data(
	struct tegra_dc_ext_flip_data *flip_kdata,
	struct tegra_dc_ext_flip_user_data *flip_udata)
{
	int ret = 0;
	size_t size;
	struct tegra_dc *dc;
	struct tegra_dc_ext_cmu_v2 *kcmu_v2;
	struct tegra_dc_ext_udata_cmu_v2 *udata_cmu_v2;
	struct tegra_dc_ext_cmu_v2 *ucmu_v2;

	if (!flip_kdata || !flip_udata)
		return -EINVAL;

	dc = flip_kdata->ext->dc;

	udata_cmu_v2 = &flip_udata->cmu_v2;
	ucmu_v2 = (struct tegra_dc_ext_cmu_v2 *)udata_cmu_v2->cmu_v2;

	kcmu_v2 = &flip_kdata->user_cmu_v2;
	size = sizeof(kcmu_v2->cmu_enable);
	/* copying only cmu_enable now */
	if (copy_from_user(kcmu_v2,
		(void __user *) (uintptr_t)ucmu_v2, size)) {
		return -EFAULT;
	}

	/* copying rest of the cmu_v2 struct after checking for update flag */
	if (kcmu_v2->cmu_enable) {
		if (flip_udata->flags & TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CMU_V2) {
			size = sizeof(*kcmu_v2) - size;
			if (copy_from_user(&kcmu_v2->lut_size,
				(void __user *) (uintptr_t)&ucmu_v2->lut_size,
				size)) {
				return -EFAULT;
			}
			flip_kdata->new_cmu_values = true;
		}
	}
	flip_kdata->cmu_update_needed = true;

	return ret;
}

static int tegra_dc_ext_configure_output_csc_user_data(
	struct tegra_dc_ext_flip_data *flip_kdata,
	struct tegra_dc_ext_flip_user_data *flip_udata)
{
	struct tegra_dc_ext_udata_output_csc *user_csc =
		&flip_udata->output_csc;

	if (flip_udata->flags & TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_CS) {
		switch (user_csc->output_colorspace) {
		case TEGRA_DC_EXT_FLIP_FLAG_CS_REC601:
		case TEGRA_DC_EXT_FLIP_FLAG_CS_REC709:
		case TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020:
			flip_kdata->output_colorspace_update_needed = true;
			flip_kdata->output_colorspace =
				user_csc->output_colorspace;
			break;
		default:
			dev_err(&flip_kdata->ext->dc->ndev->dev,
				"Invalid colorspace value\n");
			return -EINVAL;
		}
	}

	if (flip_udata->flags & TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_RANGE) {
		flip_kdata->output_range_update_needed = true;
		flip_kdata->limited_range_enable =
			user_csc->limited_range_enable;
	}

	return 0;
}

static int tegra_dc_ext_read_csc_v2_user_data(
	struct tegra_dc_ext_flip_data *flip_kdata,
	struct tegra_dc_ext_flip_user_data *flip_udata)
{
	int ret = 0;
	int i, j;
	struct tegra_dc_ext_csc_v2 *entry;
	struct tegra_dc_ext_udata_csc_v2 *ucsc_v2;

	if (!flip_kdata || !flip_udata)
		return -EINVAL;

	ucsc_v2 = &flip_udata->csc_v2;

	if (ucsc_v2->nr_elements <= 0) {
		dev_err(&flip_kdata->ext->dc->ndev->dev,
			"Invalid TEGRA_DC_EXT_FLIP_USER_DATA_CSC_V2 config\n");
		return -EINVAL;
	}

	entry = kcalloc((size_t)ucsc_v2->nr_elements, (size_t)sizeof(*entry),
			GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	if (copy_from_user(entry, (void __user *) (uintptr_t)ucsc_v2->array,
			   sizeof(*entry) * ucsc_v2->nr_elements)) {
		ret = -EFAULT;
		goto end;
	}

	for (i = 0; i < ucsc_v2->nr_elements; i++) {
		for (j = 0; j < flip_kdata->act_window_num; j++) {
			if (entry[i].win_index ==
				flip_kdata->win[j].attr.index) {
				break;
			}
		}

		if (j >= flip_kdata->act_window_num) {
			dev_err(&flip_kdata->ext->dc->ndev->dev,
				"win%d for csc_v2 not in flip wins\n",
					entry[i].win_index);
			ret = -EINVAL;
			goto end;
		}

		if (flip_kdata->win[j].attr.flags &
					TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC_V2) {
			dev_err(&flip_kdata->ext->dc->ndev->dev,
				"only one csc_v2/win%d allowed\n",
					entry[i].win_index);
			ret = -EINVAL;
			goto end;
		}

		/* copy into local tegra_dc_ext_flip_win */
		flip_kdata->win[j].csc_v2 = entry[i];
		flip_kdata->win[j].user_csc_v2 = true;
	}

end:
	kfree(entry);
	return ret;
}

static int tegra_dc_ext_read_user_data(struct tegra_dc_ext_flip_data *data,
			struct tegra_dc_ext_flip_user_data *flip_user_data,
			int nr_user_data)
{
	int i = 0, ret = 0;
	bool hdr_present = false;
#ifdef CONFIG_TEGRA_NVDISPLAY
	bool imp_tag_present = false;
#endif
	bool csc_v2_present = false;

	for (i = 0; i < nr_user_data; i++) {
		switch (flip_user_data[i].data_type) {
		case TEGRA_DC_EXT_FLIP_USER_DATA_HDR_DATA:
		{
			struct tegra_dc_hdr *hdr;
			struct tegra_dc_ext_hdr *info;

			if (hdr_present) {
				dev_err(&data->ext->dc->ndev->dev,
					"only one hdr_data/flip is allowed\n");
				return -EINVAL;
			}
			hdr_present = true;

			hdr = &data->hdr_data;
			info = &flip_user_data[i].hdr_info;
			if (flip_user_data[i].flags &
				TEGRA_DC_EXT_FLIP_FLAG_HDR_ENABLE)
				hdr->enabled = true;
			if (flip_user_data[i].flags &
				TEGRA_DC_EXT_FLIP_FLAG_HDR_DATA_UPDATED) {
				data->hdr_cache_dirty = true;
				hdr->eotf = info->eotf;
				hdr->static_metadata_id =
						info->static_metadata_id;
				memcpy(hdr->static_metadata,
					info->static_metadata,
					sizeof(hdr->static_metadata));
			}
			break;
		}
#ifdef CONFIG_TEGRA_NVDISPLAY
		case TEGRA_DC_EXT_FLIP_USER_DATA_IMP_TAG:
			if (imp_tag_present) {
				dev_err(&data->ext->dc->ndev->dev,
					"only one imp_tag/flip is allowed\n");
				return -EINVAL;
			}
			imp_tag_present = true;

			data->imp_session_id =
					flip_user_data[i].imp_tag.session_id;
			data->imp_dirty = true;
			break;
#endif
		case TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT:
			/* Already handled in the core FLIP ioctl. */
			break;

		case TEGRA_DC_EXT_FLIP_USER_DATA_CSC_V2:
			if (csc_v2_present) {
				dev_err(&data->ext->dc->ndev->dev,
					"only one csc_v2/flip is allowed\n");
				return -EINVAL;
			}
			csc_v2_present = true;

			ret = tegra_dc_ext_read_csc_v2_user_data(data,
							&flip_user_data[i]);
			if (ret)
				return ret;

			break;
		case TEGRA_DC_EXT_FLIP_USER_DATA_CMU_V2:
			ret = tegra_dc_ext_configure_cmu_v2_user_data(data,
				&flip_user_data[i]);
			if (ret)
				return ret;

			break;
		case TEGRA_DC_EXT_FLIP_USER_DATA_OUTPUT_CSC:
			ret = tegra_dc_ext_configure_output_csc_user_data(data,
				&flip_user_data[i]);
			if (ret)
				return ret;

			break;
		default:
			dev_err(&data->ext->dc->ndev->dev,
				"Invalid FLIP_USER_DATA_TYPE\n");
			return -EINVAL;
		}
	}
	return ret;
}

static int tegra_dc_ext_flip(struct tegra_dc_ext_user *user,
			     struct tegra_dc_ext_flip_windowattr_v2 *win,
			     int win_num,
			     __u32 *syncpt_id, __u32 *syncpt_val,
			     int *syncpt_fd, __u16 *dirty_rect, u8 flip_flags,
			     struct tegra_dc_ext_flip_user_data *flip_user_data,
			     int nr_user_data)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc_ext_flip_data *data;
	int work_index = -1;
	__u32 post_sync_val = 0, post_sync_id = NVSYNCPT_INVALID;
	int i, ret = 0;
	bool has_timestamp = false;

	/* If display has been disconnected return with error. */
	if (!ext->dc->connected)
		return -1;

	ret = sanitize_flip_args(user, win, win_num, &dirty_rect);
	if (ret)
		return ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	kthread_init_work(&data->work, &tegra_dc_ext_flip_worker);
	data->ext = ext;
	data->act_window_num = win_num;
	if (dirty_rect) {
		memcpy(data->dirty_rect, dirty_rect, sizeof(data->dirty_rect));
		data->dirty_rect_valid = true;
	}

	BUG_ON(win_num > tegra_dc_get_numof_dispwindows());

	ret = tegra_dc_ext_pin_windows(user, win, win_num,
				     data->win, &has_timestamp,
				     syncpt_fd != NULL);
	if (ret)
		goto fail_pin;

	ret = tegra_dc_ext_read_user_data(data, flip_user_data, nr_user_data);
	if (ret)
		goto fail_pin;

	/*
	 * If this flip needs to update the current IMP settings, reserve
	 * exclusive access to the COMMON channel. This call can potentially
	 * block.
	 */
	if (data->imp_dirty) {
		ret = tegra_dc_reserve_common_channel(ext->dc);
		if (ret) {
			dev_err(&ext->dc->ndev->dev,
			"%s: DC %d flip failed to reserve the COMMON channel\n",
				__func__, ext->dc->ctrl_num);
			goto fail_pin;
		}

		ret = tegra_dc_validate_imp_queue(ext->dc,
							data->imp_session_id);
		if (ret) {
			dev_err(&ext->dc->ndev->dev,
				"Couldn't find corresponding PROPOSE\n");
			goto fail_pin;
		}
	}

	ret = lock_windows_for_flip(user, win, win_num);
	if (ret)
		goto fail_pin;

	if (!ext->enabled) {
		ret = -ENXIO;
		goto unlock;
	}

	BUG_ON(win_num > tegra_dc_get_numof_dispwindows());
	for (i = 0; i < win_num; i++) {
		u32 syncpt_max;
		int index = win[i].index;
		struct tegra_dc_ext_win *ext_win;

		if (index < 0 || !test_bit(index, &ext->dc->valid_windows))
			continue;

		ext_win = &ext->win[index];

		syncpt_max = tegra_dc_incr_syncpt_max(ext->dc, index);

		data->win[i].syncpt_max = syncpt_max;

		/*
		 * Any of these windows' syncpoints should be equivalent for
		 * the client, so we just send back an arbitrary one of them
		 */
		post_sync_val = syncpt_max;
		post_sync_id = tegra_dc_get_syncpt_id(ext->dc, index);

		work_index = index;

		atomic_inc(&ext->win[work_index].nr_pending_flips);
	}
	if (work_index < 0) {
		ret = -EINVAL;
		goto unlock;
	}
#ifdef CONFIG_ANDROID
	work_index = ffs(ext->dc->valid_windows);
	if (!work_index) {
		dev_err(&ext->dc->ndev->dev, "no valid window\n");
		ret = -EINVAL;
		goto unlock;
	}
	work_index -= 1; /* window index starts from 0 */
#endif

	if (syncpt_fd) {
		if (post_sync_id != NVSYNCPT_INVALID) {
			ret = nvhost_syncpt_create_fence_single_ext(
					ext->dc->ndev, post_sync_id,
					post_sync_val + 1, "flip-fence",
					syncpt_fd);
			if (ret) {
				dev_err(&ext->dc->ndev->dev,
					"Failed creating fence err:%d\n", ret);
				goto unlock;
			}
		}
	} else {
		*syncpt_val = post_sync_val;
		*syncpt_id = post_sync_id;
	}

	if (trace_flip_rcvd_syncpt_upd_enabled())
		tegra_dc_flip_trace(data, trace_flip_rcvd_syncpt_upd);

	/* Avoid queueing timestamps on Android, to disable skipping flips */
#ifndef CONFIG_ANDROID
	if (has_timestamp) {
		mutex_lock(&ext->win[work_index].queue_lock);
		list_add_tail(&data->timestamp_node, &ext->win[work_index].timestamp_queue);
		mutex_unlock(&ext->win[work_index].queue_lock);
	}
#endif
	data->flags = flip_flags;

	kthread_queue_work(&ext->win[work_index].flip_worker, &data->work);

	unlock_windows_for_flip(user, win, win_num);

	return 0;

unlock:
	unlock_windows_for_flip(user, win, win_num);

fail_pin:

	for (i = 0; i < win_num; i++) {
		int j;
		for (j = 0; j < TEGRA_DC_NUM_PLANES; j++) {
			if (!data->win[i].handle[j])
				continue;

			dma_buf_unmap_attachment(data->win[i].handle[j]->attach,
				data->win[i].handle[j]->sgt, DMA_TO_DEVICE);
			dma_buf_detach(data->win[i].handle[j]->buf,
				data->win[i].handle[j]->attach);
			dma_buf_put(data->win[i].handle[j]->buf);
			kfree(data->win[i].handle[j]);
		}
	}

	/* Release the COMMON channel in case of failure. */
	if (data->imp_dirty)
		tegra_dc_release_common_channel(ext->dc);

	kfree(data);

	return ret;
}

#if defined(CONFIG_TEGRA_CSC)
static int tegra_dc_ext_set_csc(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_csc *new_csc)
{
	unsigned int index = new_csc->win_index;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_ext_win *ext_win;
	struct tegra_dc_csc *csc;
	struct tegra_dc_win *win = tegra_dc_get_window(dc, index);

	if (!win)
		return -EINVAL;

	ext_win = &user->ext->win[index];
	csc = &win->csc;

	mutex_lock(&ext_win->lock);

	if (ext_win->user != user) {
		mutex_unlock(&ext_win->lock);
		return -EACCES;
	}

	csc->yof =   new_csc->yof;
	csc->kyrgb = new_csc->kyrgb;
	csc->kur =   new_csc->kur;
	csc->kvr =   new_csc->kvr;
	csc->kug =   new_csc->kug;
	csc->kvg =   new_csc->kvg;
	csc->kub =   new_csc->kub;
	csc->kvb =   new_csc->kvb;

	tegra_dc_update_csc(dc, index);

	mutex_unlock(&ext_win->lock);

	return 0;
}
#endif

#if defined(CONFIG_TEGRA_CSC_V2)
static int tegra_dc_ext_set_csc(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_csc_v2 *new_csc)
{
	unsigned int index = new_csc->win_index;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_ext_win *ext_win;
	struct tegra_dc_csc_v2 *csc;
	struct tegra_dc_win *win = tegra_dc_get_window(dc, index);

	if (!win)
		return -EINVAL;

	ext_win = &user->ext->win[index];
	csc = &win->csc;

	mutex_lock(&ext_win->lock);

	if (ext_win->user != user) {
		mutex_unlock(&ext_win->lock);
		return -EACCES;
	}

	if (!win->force_user_csc) {
		csc->csc_enable = new_csc->csc_enable;
		csc->r2r = new_csc->r2r;
		csc->g2r = new_csc->g2r;
		csc->b2r = new_csc->b2r;
		csc->const2r = new_csc->const2r;
		csc->r2g = new_csc->r2g;
		csc->g2g = new_csc->g2g;
		csc->b2g = new_csc->b2g;
		csc->const2g = new_csc->const2g;
		csc->r2b = new_csc->r2b;
		csc->g2b = new_csc->g2b;
		csc->b2b = new_csc->b2b;
		csc->const2b = new_csc->const2b;
		tegra_nvdisp_update_csc(dc, index);
	}
	mutex_unlock(&ext_win->lock);

	return 0;
}
#endif

#if defined(CONFIG_TEGRA_LUT)
static int set_lut_channel(u16 __user *channel_from_user,
			   u8 *channel_to,
			   u32 start,
			   u32 len)
{
	int i;
	u16 lut16bpp[256];

	if (channel_from_user) {
		if (copy_from_user(lut16bpp, channel_from_user, len<<1))
			return 1;

		for (i = 0; i < len; i++)
			channel_to[start+i] = lut16bpp[i]>>8;
	} else {
		for (i = 0; i < len; i++)
			channel_to[start+i] = start+i;
	}

	return 0;
}
#elif defined(CONFIG_TEGRA_LUT_V2)
static int set_lut_channel(struct tegra_dc_ext_lut *new_lut,
			   u64 *channel_to)
{
	int i, j;
	u16 lut16bpp[256];
	u64 inlut = 0;
	u16 __user *channel_from_user;
	u32 start = new_lut->start;
	u32 len = new_lut->len;

	for (j = 0; j < 3; j++) {

		if (j == 0)
			channel_from_user = new_lut->r;
		else if (j == 1)
			channel_from_user = new_lut->g;
		else if (j == 2)
			channel_from_user = new_lut->b;

		if (channel_from_user) {
			if (copy_from_user(lut16bpp,
					channel_from_user, len<<1))
				return 1;

			for (i = 0; i < len; i++) {
				inlut = (u64)lut16bpp[i];
				/*16bit data in MSB format*/
				channel_to[start+i] |=
					((inlut & 0xFF00) << (j * 16));
			}
		} else {
			for (i = 0; i < len; i++) {
				inlut = (u64)(start+i);
				channel_to[start+i] |= (inlut << (j * 16));
			}
		}
	}
	return 0;
}
#endif

static int tegra_dc_ext_set_lut(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_lut *new_lut)
{
	int err;
	unsigned int index = new_lut->win_index;
	u32 start = new_lut->start;
	u32 len = new_lut->len;

	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_ext_win *ext_win;
	struct tegra_dc_lut *lut;
	struct tegra_dc_win *win = tegra_dc_get_window(dc, index);

	if (!win)
		return -EINVAL;

	if ((start >= 256) || (len > 256) || ((start + len) > 256))
		return -EINVAL;

	ext_win = &user->ext->win[index];
	lut = &win->lut;

	mutex_lock(&ext_win->lock);

	if (ext_win->user != user) {
		mutex_unlock(&ext_win->lock);
		return -EACCES;
	}
	tegra_dc_scrncapt_disp_pause_lock(dc);

#if defined(CONFIG_TEGRA_LUT)

	err = set_lut_channel(new_lut->r, lut->r, start, len) |
	      set_lut_channel(new_lut->g, lut->g, start, len) |
	      set_lut_channel(new_lut->b, lut->b, start, len);

#elif defined(CONFIG_TEGRA_LUT_V2)
	memset(lut->rgb, 0, sizeof(u64) * len);
	err = set_lut_channel(new_lut, lut->rgb);

#endif
	if (err) {
		tegra_dc_scrncapt_disp_pause_unlock(dc);
		mutex_unlock(&ext_win->lock);
		return -EFAULT;
	}

	tegra_dc_update_lut(dc, index,
			new_lut->flags & TEGRA_DC_EXT_LUT_FLAGS_FBOVERRIDE);

	tegra_dc_scrncapt_disp_pause_unlock(dc);
	mutex_unlock(&ext_win->lock);

	return 0;
}

#if defined(CONFIG_TEGRA_DC_CMU_V2)
static int tegra_dc_ext_get_cmu_v2(struct tegra_dc_ext_user *user,
			struct tegra_dc_ext_cmu_v2 *args, bool custom_value)
{
	int i;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_cmu *cmu = dc->pdata->cmu;
	struct tegra_dc_lut *cmu_lut = &dc->cmu;

	if (custom_value && dc->pdata->cmu) {
		cmu = dc->pdata->cmu;
		for (i = 0; i < TEGRA_DC_EXT_LUT_SIZE_1025; i++)
			args->rgb[i] = cmu->rgb[i];
	}

	else if (custom_value && !dc->pdata->cmu)
		return -EACCES;
	else {
		cmu_lut = &dc->cmu;
		for (i = 0; i < TEGRA_DC_EXT_LUT_SIZE_1025; i++)
			args->rgb[i] = cmu_lut->rgb[i];
	}

	args->cmu_enable = dc->pdata->cmu_enable;
	return 0;
}

static int tegra_dc_ext_set_cmu_v2(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_cmu_v2 *args)
{
	int i, lut_size;
	struct tegra_dc_lut *cmu;
	struct tegra_dc *dc = user->ext->dc;

	/* Directly copying the values to DC
	 * output lut whose address is already
	 * programmed to HW register.
	 */
	cmu = &dc->cmu;
	if (!cmu)
		return -ENOMEM;

	tegra_dc_scrncapt_disp_pause_lock(dc);
	dc->pdata->cmu_enable = args->cmu_enable;
	lut_size = args->lut_size;
	for (i = 0; i < lut_size; i++)
		cmu->rgb[i] = args->rgb[i];

	tegra_nvdisp_update_cmu(dc, cmu);
	tegra_dc_scrncapt_disp_pause_unlock(dc);

	return 0;
}

#elif defined(CONFIG_TEGRA_DC_CMU)
static int tegra_dc_ext_get_cmu(struct tegra_dc_ext_user *user,
			struct tegra_dc_ext_cmu *args, bool custom_value)
{
	int i;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_cmu *cmu;

	if (custom_value && dc->pdata->cmu)
		cmu = dc->pdata->cmu;
	else if (custom_value && !dc->pdata->cmu)
		return -EACCES;
	else
		cmu = &dc->cmu;

	args->cmu_enable = dc->pdata->cmu_enable;
	for (i = 0; i < 256; i++)
		args->lut1[i] = cmu->lut1[i];

	args->csc[0] = cmu->csc.krr;
	args->csc[1] = cmu->csc.kgr;
	args->csc[2] = cmu->csc.kbr;
	args->csc[3] = cmu->csc.krg;
	args->csc[4] = cmu->csc.kgg;
	args->csc[5] = cmu->csc.kbg;
	args->csc[6] = cmu->csc.krb;
	args->csc[7] = cmu->csc.kgb;
	args->csc[8] = cmu->csc.kbb;

	for (i = 0; i < 960; i++)
		args->lut2[i] = cmu->lut2[i];

	return 0;
}

static int tegra_dc_ext_set_cmu(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_cmu *args)
{
	int i;
	struct tegra_dc_cmu *cmu;
	struct tegra_dc *dc = user->ext->dc;

	cmu = kzalloc(sizeof(*cmu), GFP_KERNEL);
	if (!cmu)
		return -ENOMEM;

	dc->pdata->cmu_enable = args->cmu_enable;
	for (i = 0; i < 256; i++)
		cmu->lut1[i] = args->lut1[i];

	cmu->csc.krr = args->csc[0];
	cmu->csc.kgr = args->csc[1];
	cmu->csc.kbr = args->csc[2];
	cmu->csc.krg = args->csc[3];
	cmu->csc.kgg = args->csc[4];
	cmu->csc.kbg = args->csc[5];
	cmu->csc.krb = args->csc[6];
	cmu->csc.kgb = args->csc[7];
	cmu->csc.kbb = args->csc[8];

	for (i = 0; i < 960; i++)
		cmu->lut2[i] = args->lut2[i];

	tegra_dc_update_cmu(dc, cmu);

	kfree(cmu);
	return 0;
}

static int tegra_dc_ext_set_cmu_aligned(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_cmu *args)
{
	int i;
	struct tegra_dc_cmu *cmu;
	struct tegra_dc *dc = user->ext->dc;

	cmu = kzalloc(sizeof(*cmu), GFP_KERNEL);
	if (!cmu)
		return -ENOMEM;

	for (i = 0; i < 256; i++)
		cmu->lut1[i] = args->lut1[i];

	cmu->csc.krr = args->csc[0];
	cmu->csc.kgr = args->csc[1];
	cmu->csc.kbr = args->csc[2];
	cmu->csc.krg = args->csc[3];
	cmu->csc.kgg = args->csc[4];
	cmu->csc.kbg = args->csc[5];
	cmu->csc.krb = args->csc[6];
	cmu->csc.kgb = args->csc[7];
	cmu->csc.kbb = args->csc[8];

	for (i = 0; i < 960; i++)
		cmu->lut2[i] = args->lut2[i];

	tegra_dc_update_cmu_aligned(dc, cmu);

	kfree(cmu);
	return 0;
}
static int tegra_dc_ext_get_cmu_adbRGB(struct tegra_dc_ext_user *user,
			struct tegra_dc_ext_cmu *args)
{
	int i;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_cmu *cmu;

	if (dc->pdata->cmu_adbRGB)
		cmu = dc->pdata->cmu_adbRGB;
	else
		return -EACCES;

	args->cmu_enable = dc->pdata->cmu_enable;
	for (i = 0; i < 256; i++)
		args->lut1[i] = cmu->lut1[i];

	args->csc[0] = cmu->csc.krr;
	args->csc[1] = cmu->csc.kgr;
	args->csc[2] = cmu->csc.kbr;
	args->csc[3] = cmu->csc.krg;
	args->csc[4] = cmu->csc.kgg;
	args->csc[5] = cmu->csc.kbg;
	args->csc[6] = cmu->csc.krb;
	args->csc[7] = cmu->csc.kgb;
	args->csc[8] = cmu->csc.kbb;

	for (i = 0; i < 960; i++)
		args->lut2[i] = cmu->lut2[i];

	return 0;
}

#endif

#ifdef CONFIG_TEGRA_ISOMGR
static int tegra_dc_ext_negotiate_bw(struct tegra_dc_ext_user *user,
			struct tegra_dc_ext_flip_windowattr_v2 *wins,
			int win_num)
{
	int i;
	int ret;
	struct tegra_dc_win *dc_wins[DC_N_WINDOWS];
	struct tegra_dc *dc = user->ext->dc;

	/* If display has been disconnected return with error. */
	if (!dc->connected)
		return -1;

	for (i = 0; i < win_num; i++) {
		int idx = wins[i].index;

		if (wins[i].buff_id > 0) {
			tegra_dc_ext_set_windowattr_basic(&dc->tmp_wins[idx],
							  &wins[i]);
		} else {
			dc->tmp_wins[idx].flags = 0;
			dc->tmp_wins[idx].new_bandwidth = 0;
		}
		dc_wins[i] = &dc->tmp_wins[idx];
	}

	ret = tegra_dc_bandwidth_negotiate_bw(dc, dc_wins, win_num);

	return ret;
}
#endif

static u32 tegra_dc_ext_get_vblank_syncpt(struct tegra_dc_ext_user *user)
{
	struct tegra_dc *dc = user->ext->dc;

	return dc->vblank_syncpt;
}

static int tegra_dc_ext_get_status(struct tegra_dc_ext_user *user,
				   struct tegra_dc_ext_status *status)
{
	struct tegra_dc *dc = user->ext->dc;

	memset(status, 0, sizeof(*status));

	if (dc->enabled)
		status->flags |= TEGRA_DC_EXT_FLAGS_ENABLED;

	return 0;
}

static int tegra_dc_ext_get_feature(struct tegra_dc_ext_user *user,
				   struct tegra_dc_ext_feature *feature)
{
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_dc_feature *table = dc->feature;

	if (dc->enabled && feature->entries) {
		feature->length = table->num_entries;
		memcpy(feature->entries, table->entries, table->num_entries *
					sizeof(struct tegra_dc_feature_entry));
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static int dev_cpy_from_usr_compat(
			struct tegra_dc_ext_flip_windowattr_v2 *outptr,
			void *inptr, u32 usr_win_size, u32 win_num)
{
	int i = 0;
	u8 *srcptr;

	for (i = 0; i < win_num; i++) {
		srcptr  = (u8 *)inptr + (usr_win_size * i);

		if (copy_from_user(&outptr[i],
			compat_ptr((uintptr_t)srcptr), usr_win_size))
			return -EFAULT;
	}
	return 0;
}

static int dev_cpy_to_usr_compat(void *outptr, u32 usr_win_size,
		struct tegra_dc_ext_flip_windowattr_v2 *inptr, u32 win_num)
{
	int i = 0;
	u8 *dstptr;

	for (i = 0; i < win_num; i++) {
		dstptr  = (u8 *)outptr + (usr_win_size * i);

		if (copy_to_user(compat_ptr((uintptr_t)dstptr),
			&inptr[i], usr_win_size))
			return -EFAULT;
	}
	return 0;
}
#endif

static int dev_cpy_from_usr(struct tegra_dc_ext_flip_windowattr_v2 *outptr,
				void *inptr, u32 usr_win_size, u32 win_num)
{
	int i = 0;
	u8 *srcptr;

	for (i = 0; i < win_num; i++) {
		srcptr  = (u8 *)inptr + (usr_win_size * i);

		if (copy_from_user(&outptr[i],
			(void __user *) (uintptr_t)srcptr, usr_win_size))
			return -EFAULT;
	}
	return 0;
}

static int dev_cpy_to_usr(void *outptr, u32 usr_win_size,
		struct tegra_dc_ext_flip_windowattr_v2 *inptr, u32 win_num)
{
	int i = 0;
	u8 *dstptr;

	for (i = 0; i < win_num; i++) {
		dstptr  = (u8 *)outptr + (usr_win_size * i);

		if (copy_to_user((void __user *) (uintptr_t)dstptr,
			&inptr[i], usr_win_size))
			return -EFAULT;
	}
	return 0;
}

static int tegra_dc_get_cap_hdr_info(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_hdr_caps *hdr_cap_info)
{
	int ret = 0;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_edid *dc_edid = dc->edid;

	/* Currently only dc->edid has this info. In future,
	 * we have to provide info for non-edid interfaces
	 * in the device tree.
	 */
	if (dc_edid)
		ret = tegra_edid_get_ex_hdr_cap_info(dc_edid, hdr_cap_info);

	return ret;

}

static int tegra_dc_get_cap_quant_info(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_quant_caps *quant_cap_info)
{
	int ret = 0;
	struct tegra_dc *dc = user->ext->dc;
	struct tegra_edid *dc_edid = dc->edid;

	if (dc_edid)
		ret = tegra_edid_get_ex_quant_cap_info(dc_edid, quant_cap_info);

	return ret;
}

static int tegra_dc_get_cap_info(struct tegra_dc_ext_user *user,
				struct tegra_dc_ext_caps *cap_info,
				int nr_elements)
{
	int i, ret = 0;
	for (i = 0; i < nr_elements; i++) {

		switch (cap_info[i].data_type) {

		case TEGRA_DC_EXT_CAP_TYPE_HDR_SINK:
		{
			struct tegra_dc_ext_hdr_caps *hdr_cap_info;

			hdr_cap_info = kzalloc(sizeof(*hdr_cap_info),
				GFP_KERNEL);

			ret = tegra_dc_get_cap_hdr_info(user, hdr_cap_info);

			if (copy_to_user((void __user *)(uintptr_t)
				cap_info[i].data, hdr_cap_info,
				sizeof(*hdr_cap_info))) {
				kfree(hdr_cap_info);
				return -EFAULT;
			}
			kfree(hdr_cap_info);
			break;
		}
		case TEGRA_DC_EXT_CAP_TYPE_QUANT_SELECTABLE:
		{
			struct tegra_dc_ext_quant_caps *quant_cap_info;

			quant_cap_info = kzalloc(sizeof(*quant_cap_info),
				GFP_KERNEL);

			ret = tegra_dc_get_cap_quant_info(user, quant_cap_info);
			if (copy_to_user((void __user *)(uintptr_t)
				cap_info[i].data, quant_cap_info,
				sizeof(*quant_cap_info))) {
				kfree(quant_cap_info);
				return -EFAULT;
			}
			kfree(quant_cap_info);
			break;
		}
		default:
			return -EINVAL;
		}
	}

	return ret;
}

static int tegra_dc_copy_syncpts_from_user(struct tegra_dc *dc,
			struct tegra_dc_ext_flip_user_data *flip_user_data,
			int nr_user_data, u32 **syncpt_id, u32 **syncpt_val,
			int **syncpt_fd, int *syncpt_idx)
{
	struct tegra_dc_ext_flip_user_data *syncpt_user_data = NULL;
	int i;

	for (i = 0; i < nr_user_data; i++) {
		if (flip_user_data[i].data_type ==
				TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT) {
			/* Only one allowed. */
			if (syncpt_user_data) {
				dev_err(&dc->ndev->dev,
					"Only one syncpt user data allowed!\n");
					return -EINVAL;
			}

			syncpt_user_data = &flip_user_data[i];
			*syncpt_idx = i;
		}
	}

	if (syncpt_user_data) {
		struct tegra_dc_ext_syncpt *ext_syncpt;
		u16 syncpt_type;

		syncpt_type = syncpt_user_data->flags &
				TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_MASK;
		ext_syncpt = &syncpt_user_data->post_syncpt;

		switch (syncpt_type) {
		case TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_FD:
			*syncpt_fd = &ext_syncpt->syncpt_fd;
			break;
		case TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_RAW:
			*syncpt_id = &ext_syncpt->syncpt_id;
			*syncpt_val = &ext_syncpt->syncpt_val;
			break;
		default:
			dev_err(&dc->ndev->dev,
				"Unrecognized syncpt user data type!\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int tegra_dc_copy_syncpts_to_user(
			struct tegra_dc_ext_flip_user_data *flip_user_data,
			int syncpt_idx,
			u8 *base_addr)
{
	u32 offset = sizeof(*flip_user_data) * syncpt_idx;
	u8 *dstptr = base_addr + offset;

	if (copy_to_user((void __user *)(uintptr_t)dstptr,
			&flip_user_data[syncpt_idx], sizeof(*flip_user_data)))
		return -EFAULT;

	return 0;
}

static long tegra_dc_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	void __user *user_arg = (void __user *)arg;
	struct tegra_dc_ext_user *user = filp->private_data;
	int ret;

	switch (cmd) {
	case TEGRA_DC_EXT_SET_NVMAP_FD:
		return 0;

	case TEGRA_DC_EXT_GET_WINDOW:
		return tegra_dc_ext_get_window(user, arg);
	case TEGRA_DC_EXT_PUT_WINDOW:
		ret = tegra_dc_ext_put_window(user, arg);
		return ret;

	case TEGRA_DC_EXT_GET_WINMASK:
	{
		u32 winmask = tegra_dc_ext_get_winmask(user);

		if (copy_to_user(user_arg, &winmask, sizeof(winmask)))
			return -EFAULT;

		return 0;
	}
	case TEGRA_DC_EXT_SET_WINMASK:
		return tegra_dc_ext_set_winmask(user, arg);

	case TEGRA_DC_EXT_FLIP:
	{
		int ret;
		int win_num, i;
		struct tegra_dc_ext_flip args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;
		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = TEGRA_DC_EXT_FLIP_N_WINDOWS;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);
		if (!win)
			return -EFAULT;

		for (i = 0; i < win_num; i++)
			memcpy(&win[i], &args.win[i], usr_win_size);

		ret = tegra_dc_ext_flip(user, win,
			TEGRA_DC_EXT_FLIP_N_WINDOWS,
			&args.post_syncpt_id, &args.post_syncpt_val, NULL,
			NULL, 0, NULL, 0);

		for (i = 0; i < win_num; i++)
			memcpy(&args.win[i], &win[i], usr_win_size);

		if (copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}

#ifdef CONFIG_COMPAT
	case TEGRA_DC_EXT_FLIP2_32:
	{
		int ret;
		int win_num;
		struct tegra_dc_ext_flip_2_32 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;

		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (dev_cpy_from_usr_compat(win, (void *)(uintptr_t)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		ret = tegra_dc_ext_flip(user, win, win_num,
			&args.post_syncpt_id, &args.post_syncpt_val, NULL,
			args.dirty_rect, 0, NULL, 0);

		if (dev_cpy_to_usr_compat((void *)(uintptr_t)args.win,
				usr_win_size, win, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		if (copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}

#endif
	case TEGRA_DC_EXT_FLIP2:
	{
		int ret;
		int win_num;
		struct tegra_dc_ext_flip_2 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;

		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		ret = tegra_dc_ext_flip(user, win, win_num,
			&args.post_syncpt_id, &args.post_syncpt_val, NULL,
			args.dirty_rect, 0, NULL, 0);

		if (dev_cpy_to_usr((void *)args.win, usr_win_size,
					win, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		if (copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}

	case TEGRA_DC_EXT_FLIP3:
	{
		int ret;
		int win_num;
		struct tegra_dc_ext_flip_3 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;
		bool bypass;

		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		bypass = !!(args.flags & TEGRA_DC_EXT_FLIP_HEAD_FLAG_YUVBYPASS);

#ifndef CONFIG_TEGRA_NVDISPLAY
		if (!IS_RGB(user->ext->dc->mode.vmode) != bypass)
			return -EINVAL;
#endif

		if (bypass != user->ext->dc->yuv_bypass)
			user->ext->dc->yuv_bypass_dirty = true;
		user->ext->dc->yuv_bypass = bypass;
		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (args.flags &
				TEGRA_DC_EXT_FLIP_HEAD_FLAG_V2_ATTR)
			usr_win_size =
				sizeof(struct tegra_dc_ext_flip_windowattr_v2);

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		ret = tegra_dc_ext_flip(user, win, win_num,
			NULL, NULL, &args.post_syncpt_fd, args.dirty_rect,
			args.flags, NULL, 0);

		if (dev_cpy_to_usr((void *)args.win, usr_win_size,
					win, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		if (copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}
	case TEGRA_DC_EXT_FLIP4:
	{
		int ret;
		int win_num;
		int nr_user_data;
		struct tegra_dc_ext_flip_4 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;
		struct tegra_dc_ext_flip_user_data *flip_user_data;
		bool bypass;
		u32 *syncpt_id = NULL, *syncpt_val = NULL;
		int *syncpt_fd = NULL;
		int syncpt_idx = -1;

		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		bypass = !!(args.flags & TEGRA_DC_EXT_FLIP_HEAD_FLAG_YUVBYPASS);

#ifndef CONFIG_TEGRA_NVDISPLAY
		if (!IS_RGB(user->ext->dc->mode.vmode) != bypass)
			return -EINVAL;
#endif

		if (bypass != user->ext->dc->yuv_bypass)
			user->ext->dc->yuv_bypass_dirty = true;
		user->ext->dc->yuv_bypass = bypass;
		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (args.flags &
				TEGRA_DC_EXT_FLIP_HEAD_FLAG_V2_ATTR)
			usr_win_size =
				sizeof(struct tegra_dc_ext_flip_windowattr_v2);

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		nr_user_data = args.nr_elements;
		flip_user_data = kzalloc(sizeof(*flip_user_data)
					* nr_user_data, GFP_KERNEL);
		if (nr_user_data > 0) {
			if (copy_from_user(flip_user_data,
				(void __user *) (uintptr_t)args.data,
				sizeof(*flip_user_data) * nr_user_data)) {
				kfree(flip_user_data);
				kfree(win);
				return -EFAULT;
			}
		}

		/*
		 * Check if the client is explicitly requesting syncpts via flip
		 * user data. If so, populate the syncpt variables accordingly.
		 * Else, default to sync fds and use the original post_syncpt_fd
		 * fence.
		 */
		ret = tegra_dc_copy_syncpts_from_user(user->ext->dc,
			flip_user_data, nr_user_data, &syncpt_id, &syncpt_val,
			&syncpt_fd, &syncpt_idx);
		if (ret) {
			kfree(flip_user_data);
			kfree(win);
			return ret;
		}

		if (syncpt_idx == -1)
			syncpt_fd = &args.post_syncpt_fd;

		ret = tegra_dc_ext_flip(user, win, win_num,
			syncpt_id, syncpt_val, syncpt_fd, args.dirty_rect,
			args.flags, flip_user_data, nr_user_data);

		/*
		 * If the client requested post syncpt values via user data,
		 * copy them back.
		 */
		if (syncpt_idx > -1) {
			args.post_syncpt_fd = -1;
			ret = tegra_dc_copy_syncpts_to_user(flip_user_data,
					syncpt_idx, (u8 *)(void *)args.data);
			if (ret) {
				kfree(flip_user_data);
				kfree(win);
				return ret;
			}
		}

		kfree(flip_user_data);

		if (dev_cpy_to_usr((void *)args.win, usr_win_size,
					win, win_num) ||
			copy_to_user(user_arg, &args, sizeof(args))) {
			kfree(win);
			return -EFAULT;
		}

		kfree(win);
		return ret;
	}

	case TEGRA_DC_EXT_GET_IMP_USER_INFO:
	{
#ifdef CONFIG_TEGRA_NVDISPLAY
		struct tegra_dc_ext_imp_user_info *info;
		int ret = 0;

		info = kzalloc(sizeof(*info), GFP_KERNEL);
		if (!info)
			return -ENOMEM;

		if (copy_from_user(info, user_arg, sizeof(*info))) {
			kfree(info);
			return -EFAULT;
		}

		ret = tegra_nvdisp_get_imp_user_info(user->ext->dc, info);
		if (ret) {
			kfree(info);
			return ret;
		}

		if (copy_to_user(user_arg, info, sizeof(*info))) {
			kfree(info);
			return -EFAULT;
		}

		kfree(info);
		return ret;
#else
		return -EINVAL;
#endif
	}

#ifdef CONFIG_TEGRA_ISOMGR
#ifdef CONFIG_COMPAT
	case TEGRA_DC_EXT_SET_PROPOSED_BW32:
	{
		int ret;
		int win_num;
		struct tegra_dc_ext_flip_2_32 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;

		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (dev_cpy_from_usr_compat(win, (void *)(uintptr_t)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		ret = tegra_dc_ext_negotiate_bw(user, win, win_num);

		kfree(win);

		return ret;
	}
#endif
	case TEGRA_DC_EXT_SET_PROPOSED_BW:
	{
		int ret;
		int win_num;
		struct tegra_dc_ext_flip_2 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win;
		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			kfree(win);
			return -EFAULT;
		}

		ret = tegra_dc_ext_negotiate_bw(user, win, win_num);

		kfree(win);

		return ret;
	}
	case TEGRA_DC_EXT_SET_PROPOSED_BW_3:
	{
		int ret = 0;
		int win_num;
		int nr_user_data;
		struct tegra_dc_ext_flip_4 args;
		struct tegra_dc_ext_flip_windowattr_v2 *win = NULL;
		struct tegra_dc_ext_flip_user_data *flip_user_data = NULL;
		bool imp_proposed = false;

		/* Keeping window attribute size as version1 for old
		 *  legacy applications
		 */
		u32 usr_win_size = sizeof(struct tegra_dc_ext_flip_windowattr);

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		win_num = args.win_num;
		win = kzalloc(sizeof(*win) * win_num, GFP_KERNEL);
		if (!win)
			return -ENOMEM;

		if (dev_cpy_from_usr(win, (void *)args.win,
					usr_win_size, win_num)) {
			ret = -EFAULT;
			goto free_and_ret;
		}

		nr_user_data = args.nr_elements;
		flip_user_data = kzalloc(sizeof(*flip_user_data)
					* nr_user_data, GFP_KERNEL);
		if (!flip_user_data) {
			ret = -ENOMEM;
			goto free_and_ret;
		}

		if (nr_user_data > 0) {
			if (copy_from_user(flip_user_data,
				(void __user *) (uintptr_t)args.data,
				sizeof(*flip_user_data) * nr_user_data)) {
				ret = -EFAULT;
				goto free_and_ret;
			}

			if (flip_user_data[0].data_type ==
				TEGRA_DC_EXT_FLIP_USER_DATA_IMP_DATA) {
				ret = tegra_dc_handle_imp_propose(
						user->ext->dc, flip_user_data);
				if (ret)
					goto free_and_ret;

				imp_proposed = true;
			}
		}

		if (!imp_proposed)
			ret = tegra_dc_ext_negotiate_bw(user, win, win_num);

free_and_ret:
		kfree(flip_user_data);
		kfree(win);
		return ret;
	}
#else
/* if isomgr is not present, allow any request to pass. */
#ifdef CONFIG_COMPAT
	case TEGRA_DC_EXT_SET_PROPOSED_BW32:
		return 0;
#endif
	case TEGRA_DC_EXT_SET_PROPOSED_BW:
		return 0;
	case TEGRA_DC_EXT_SET_PROPOSED_BW_3:
		return 0;
#endif
	case TEGRA_DC_EXT_GET_CURSOR:
		return tegra_dc_ext_get_cursor(user);
	case TEGRA_DC_EXT_PUT_CURSOR:
		return tegra_dc_ext_put_cursor(user);
	case TEGRA_DC_EXT_SET_CURSOR_IMAGE:
	{
		struct tegra_dc_ext_cursor_image args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_cursor_image(user, &args);
	}
	case TEGRA_DC_EXT_SET_CURSOR:
	{
		struct tegra_dc_ext_cursor args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_cursor(user, &args);
	}
#ifdef CONFIG_TEGRA_CSC
	case TEGRA_DC_EXT_SET_CSC:
	{
		struct tegra_dc_ext_csc args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_csc(user, &args);
	}
#endif

#ifdef CONFIG_TEGRA_CSC_V2
	case TEGRA_DC_EXT_SET_CSC_V2:
	{
		struct tegra_dc_ext_csc_v2 args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_csc(user, &args);
	}
#endif
	case TEGRA_DC_EXT_GET_VBLANK_SYNCPT:
	{
		u32 syncpt = tegra_dc_ext_get_vblank_syncpt(user);

		if (copy_to_user(user_arg, &syncpt, sizeof(syncpt)))
			return -EFAULT;

		return 0;
	}

	case TEGRA_DC_EXT_GET_STATUS:
	{
		struct tegra_dc_ext_status args;
		int ret;

		ret = tegra_dc_ext_get_status(user, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

#ifdef CONFIG_COMPAT
	case TEGRA_DC_EXT_SET_LUT32:
	{
		struct tegra_dc_ext_lut32 args;
		struct tegra_dc_ext_lut tmp;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		/* translate 32-bit version to 64-bit */
		tmp.win_index = args.win_index;
		tmp.flags = args.flags;
		tmp.start = args.start;
		tmp.len = args.len;
		tmp.r = compat_ptr(args.r);
		tmp.g = compat_ptr(args.g);
		tmp.b = compat_ptr(args.b);

		return tegra_dc_ext_set_lut(user, &tmp);
	}
#endif
	case TEGRA_DC_EXT_SET_LUT:
	{
		struct tegra_dc_ext_lut args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_lut(user, &args);
	}

#ifdef CONFIG_COMPAT
	case TEGRA_DC_EXT_GET_FEATURES32:
	{
		struct tegra_dc_ext_feature32 args;
		struct tegra_dc_ext_feature tmp;
		int ret;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		/* convert 32-bit to 64-bit version */
		tmp.length = args.length;
		tmp.entries = compat_ptr(args.entries);

		ret = tegra_dc_ext_get_feature(user, &tmp);

		/* convert back to 32-bit version, tmp.entries not modified */
		args.length = tmp.length;

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}
#endif
	case TEGRA_DC_EXT_GET_FEATURES:
	{
		struct tegra_dc_ext_feature args;
		int ret;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		ret = tegra_dc_ext_get_feature(user, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

	case TEGRA_DC_EXT_CURSOR_CLIP:
	{
		int args;
		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_cursor_clip(user, &args);
	}

	case TEGRA_DC_EXT_GET_CMU:
	{
#ifdef CONFIG_TEGRA_DC_CMU
		struct tegra_dc_ext_cmu *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		tegra_dc_ext_get_cmu(user, args, 0);

		if (copy_to_user(user_arg, args, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		kfree(args);
		return 0;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_GET_CUSTOM_CMU:
	{
#ifdef CONFIG_TEGRA_DC_CMU
		struct tegra_dc_ext_cmu *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (tegra_dc_ext_get_cmu(user, args, 1)) {
			kfree(args);
			return -EACCES;
		}

		if (copy_to_user(user_arg, args, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		kfree(args);
		return 0;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_GET_CMU_ADBRGB:
	{
#ifdef CONFIG_TEGRA_DC_CMU
		struct tegra_dc_ext_cmu *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (tegra_dc_ext_get_cmu_adbRGB(user, args)) {
			kfree(args);
			return -EACCES;
		}

		if (copy_to_user(user_arg, args, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		kfree(args);
		return 0;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_SET_CMU:
	{
#ifdef CONFIG_TEGRA_DC_CMU
		int ret;
		struct tegra_dc_ext_cmu *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (copy_from_user(args, user_arg, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		ret = tegra_dc_ext_set_cmu(user, args);

		kfree(args);

		return ret;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_GET_CMU_V2:
	case TEGRA_DC_EXT_GET_CUSTOM_CMU_V2:
	{
#ifdef CONFIG_TEGRA_DC_CMU_V2
		struct tegra_dc_ext_cmu_v2 *args;
		bool custom_value = false;

		if (TEGRA_DC_EXT_GET_CUSTOM_CMU_V2 == cmd)
			custom_value = true;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (tegra_dc_ext_get_cmu_v2(user, args, custom_value)) {
			kfree(args);
			return -EACCES;
		}

		if (copy_to_user(user_arg, args, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		kfree(args);
		return 0;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_SET_CMU_V2:
	{
#ifdef CONFIG_TEGRA_DC_CMU_V2
		int ret;
		struct tegra_dc_ext_cmu_v2 *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (copy_from_user(args, user_arg, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		ret = tegra_dc_ext_set_cmu_v2(user, args);

		kfree(args);

		return ret;
#else
		return -EACCES;
#endif
	}

	case TEGRA_DC_EXT_SET_VBLANK:
	{
		struct tegra_dc_ext_set_vblank args;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		return tegra_dc_ext_set_vblank(user->ext, args.enable);
	}

	/* Update only modified elements in CSC and LUT2.
	 * Align writes to FRAME_END_INT */
	case TEGRA_DC_EXT_SET_CMU_ALIGNED:
	{
#ifdef CONFIG_TEGRA_DC_CMU
		int ret;
		struct tegra_dc_ext_cmu *args;

		args = kzalloc(sizeof(*args), GFP_KERNEL);
		if (!args)
			return -ENOMEM;

		if (copy_from_user(args, user_arg, sizeof(*args))) {
			kfree(args);
			return -EFAULT;
		}

		ret = tegra_dc_ext_set_cmu_aligned(user, args);

		kfree(args);

		return ret;
#else
		return -EACCES;
#endif
	}
	case TEGRA_DC_EXT_GET_CAP_INFO:
	{
		int ret = 0;
		int nr_elements = 0;
		struct tegra_dc_ext_get_cap_info args;
		struct tegra_dc_ext_caps *cap_info = NULL;


		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		nr_elements = args.nr_elements;

		if (nr_elements > 0) {
			cap_info = kzalloc(sizeof(*cap_info)
					* nr_elements, GFP_KERNEL);
			if (!cap_info)
				return -ENOMEM;
			if (copy_from_user(cap_info,
				(void __user *) (uintptr_t)args.data,
				sizeof(*cap_info) * nr_elements)) {
				kfree(cap_info);
				return -EFAULT;
			}
		}
		ret = tegra_dc_get_cap_info(user, cap_info, nr_elements);

		kfree(cap_info);

		return ret;
	}

	/* Screen Capture Support
	 */
	case TEGRA_DC_EXT_SCRNCAPT_GET_INFO:
	{
#ifdef CONFIG_TEGRA_DC_SCREEN_CAPTURE
		struct tegra_dc_ext_scrncapt_get_info  args;
		int  ret;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;
		ret = tegra_dc_scrncapt_get_info(user, &args);
		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;
		return ret;
#else
		return -EINVAL;
#endif
	}

	case TEGRA_DC_EXT_SCRNCAPT_DUP_FBUF:
	{
#ifdef CONFIG_TEGRA_DC_SCREEN_CAPTURE
		struct tegra_dc_ext_scrncapt_dup_fbuf  args;
		int  ret;

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;
		ret = tegra_dc_scrncapt_dup_fbuf(user, &args);
		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;
		return ret;
#else
		return -EINVAL;
#endif
	}

	case TEGRA_DC_EXT_GET_SCANLINE:
	{
		u32 scanln;

		dev_dbg(&user->ext->dc->ndev->dev, "GET SCANLN IOCTL\n");

		/* If display has been disconnected return with error. */
		if (!user->ext->dc->connected)
			return -EACCES;

		scanln = tegra_dc_get_v_count(user->ext->dc);

		if (copy_to_user(user_arg, &scanln, sizeof(scanln)))
			return -EFAULT;

		return 0;
	}

	case TEGRA_DC_EXT_SET_SCANLINE:
	{
		struct tegra_dc_ext_scanline_info args;

		dev_dbg(&user->ext->dc->ndev->dev, "SET SCANLN IOCTL\n");

		if (copy_from_user(&args, user_arg, sizeof(args)))
			return -EFAULT;

		ret = tegra_dc_ext_vpulse3(user->ext, &args);

		if (copy_to_user(user_arg, &args, sizeof(args)))
			return -EFAULT;

		return ret;
	}

	default:
		return -EINVAL;
	}
}

static int tegra_dc_open(struct inode *inode, struct file *filp)
{
	struct tegra_dc_ext_user *user;
	struct tegra_dc_ext *ext;
	int open_count;

	user = kzalloc(sizeof(*user), GFP_KERNEL);
	if (!user)
		return -ENOMEM;

	ext = container_of(inode->i_cdev, struct tegra_dc_ext, cdev);
	user->ext = ext;

	filp->private_data = user;

	open_count = atomic_inc_return(&dc_open_count);
	if (open_count < 1) {
		pr_err("%s: unbalanced dc open count=%d\n", __func__,
			open_count);
		return -EINVAL;
	}

	return 0;
}

static int tegra_dc_release(struct inode *inode, struct file *filp)
{
	struct tegra_dc_ext_user *user = filp->private_data;
	struct tegra_dc_ext *ext = user->ext;
	unsigned int i;
	unsigned long int windows = 0;
	int open_count;

	for (i = 0; i < tegra_dc_get_numof_dispwindows(); i++) {
		if (ext->win[i].user == user) {
			tegra_dc_ext_put_window(user, i);
			windows |= BIT(i);
		}
	}

	if (ext->dc->enabled) {
		tegra_dc_blank_wins(ext->dc, windows);
		for_each_set_bit(i, &windows,
				tegra_dc_get_numof_dispwindows()) {
			tegra_dc_ext_unpin_window(&ext->win[i]);
			tegra_dc_disable_window(ext->dc, i);
		}
	}

	if (ext->cursor.user == user)
		tegra_dc_ext_put_cursor(user);

	kfree(user);

	open_count = atomic_dec_return(&dc_open_count);
	if (open_count < 0) {
		pr_err("%s: unbalanced dc open count=%d\n", __func__,
			open_count);
		return -EINVAL;
	}

	if (open_count == 0)
		tegra_dc_reset_imp_state();

	return 0;
}

static int tegra_dc_ext_setup_windows(struct tegra_dc_ext *ext)
{
	int i, ret;
	struct sched_param sparm = { .sched_priority = 1 };

	for (i = 0; i < ext->dc->n_windows; i++) {
		struct tegra_dc_ext_win *win = &ext->win[i];
		char name[32];

		win->ext = ext;
		win->idx = i;

		snprintf(name, sizeof(name), "tegradc.%d/%c",
			 ext->dc->ndev->id, 'a' + i);
		kthread_init_worker(&win->flip_worker);
		win->flip_kthread = kthread_run(&kthread_worker_fn,
			&win->flip_worker, name);
		if (!win->flip_kthread) {
			ret = -ENOMEM;
			goto cleanup;
		}

		sched_setscheduler(win->flip_kthread, SCHED_FIFO, &sparm);

		mutex_init(&win->lock);
		mutex_init(&win->queue_lock);
		INIT_LIST_HEAD(&win->timestamp_queue);
	}

	return 0;

cleanup:
	while (i--) {
		struct tegra_dc_ext_win *win = &ext->win[i];
		kthread_stop(win->flip_kthread);
	}

	return ret;
}

static const struct file_operations tegra_dc_devops = {
	.owner =		THIS_MODULE,
	.open =			tegra_dc_open,
	.release =		tegra_dc_release,
	.unlocked_ioctl =	tegra_dc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =		tegra_dc_ioctl,
#endif
};

struct tegra_dc_ext *tegra_dc_ext_register(struct platform_device *ndev,
					   struct tegra_dc *dc)
{
	int ret;
	struct tegra_dc_ext *ext;
	dev_t devno;
	char name[16];
	struct sched_param sparm = { .sched_priority = 1 };

	ext = kzalloc(sizeof(*ext), GFP_KERNEL);
	if (!ext)
		return ERR_PTR(-ENOMEM);

	BUG_ON(!tegra_dc_ext_devno);
	devno = tegra_dc_ext_devno + head_count + 1;

	cdev_init(&ext->cdev, &tegra_dc_devops);
	ext->cdev.owner = THIS_MODULE;
	ret = cdev_add(&ext->cdev, devno, 1);
	if (ret) {
		dev_err(&ndev->dev, "Failed to create character device\n");
		goto cleanup_alloc;
	}

	ext->dev = device_create(tegra_dc_ext_class,
				 &ndev->dev,
				 devno,
				 NULL,
				 "tegra_dc_%d",
				 ndev->id);

	if (IS_ERR(ext->dev)) {
		ret = PTR_ERR(ext->dev);
		goto cleanup_cdev;
	}

	ext->dc = dc;

	ret = tegra_dc_ext_setup_windows(ext);
	if (ret)
		goto cleanup_device;

	mutex_init(&ext->cursor.lock);

	/* Setup scanline workqueues */
	ext->scanline_trigger = -1;
	snprintf(name, sizeof(name), "tegradc.%d/sl", dc->ndev->id);
	kthread_init_worker(&ext->scanline_worker);
	ext->scanline_task = kthread_run(kthread_worker_fn,
				&ext->scanline_worker, name);
	if (!ext->scanline_task) {
		ret = -ENOMEM;
		goto cleanup_device;
	}
	sched_setscheduler(ext->scanline_task, SCHED_FIFO, &sparm);

	head_count++;

	return ext;

cleanup_device:
	device_del(ext->dev);

cleanup_cdev:
	cdev_del(&ext->cdev);

cleanup_alloc:
	kfree(ext);

	return ERR_PTR(ret);
}

void tegra_dc_ext_unregister(struct tegra_dc_ext *ext)
{
	int i;

	for (i = 0; i < ext->dc->n_windows; i++) {
		struct tegra_dc_ext_win *win = &ext->win[i];

		kthread_flush_worker(&win->flip_worker);
		kthread_stop(win->flip_kthread);
	}

	/* Remove scanline work */
	kthread_flush_worker(&ext->scanline_worker);
	ext->scanline_task = NULL;
	/* Clear trigger line value */
	ext->scanline_trigger = -1;
	/* Update min val all the way to max. */
	nvhost_syncpt_set_min_eq_max_ext(ext->dc->ndev,
					ext->dc->vpulse3_syncpt);

	device_del(ext->dev);
	cdev_del(&ext->cdev);

	kfree(ext);

	head_count--;
}

int __init tegra_dc_ext_module_init(void)
{
	int ret, nheads = tegra_dc_get_numof_dispheads();

	if (nheads <= 0) {
		pr_err("%s: max heads:%d cannot be negative or zero\n",
			__func__, nheads);
		return -EINVAL;
	}

	tegra_dc_ext_class = class_create(THIS_MODULE, "tegra_dc_ext");
	if (!tegra_dc_ext_class) {
		printk(KERN_ERR "tegra_dc_ext: failed to create class\n");
		return -ENOMEM;
	}

	/* Reserve one character device per head, plus the control device */
	ret = alloc_chrdev_region(&tegra_dc_ext_devno,
				  0, nheads + 1,
				  "tegra_dc_ext");
	if (ret)
		goto cleanup_class;

	ret = tegra_dc_ext_control_init();
	if (ret)
		goto cleanup_region;

	tegra_dc_scrncapt_init();

	return 0;

cleanup_region:
	unregister_chrdev_region(tegra_dc_ext_devno, nheads);

cleanup_class:
	class_destroy(tegra_dc_ext_class);

	return ret;
}

void __exit tegra_dc_ext_module_exit(void)
{
	tegra_dc_scrncapt_exit();
	unregister_chrdev_region(tegra_dc_ext_devno,
			tegra_dc_get_numof_dispheads());
	class_destroy(tegra_dc_ext_class);
}
