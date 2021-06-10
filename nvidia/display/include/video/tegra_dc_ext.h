/*
 * tegra_dc_ext.h: tegra dc ext interface.
 *
 * Copyright (C) 2011-2019, NVIDIA Corporation. All rights reserved.
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

#ifndef __TEGRA_DC_EXT_H
#define __TEGRA_DC_EXT_H

#include <linux/types.h>
#include <linux/ioctl.h>
#if defined(__KERNEL__)
# include <linux/time.h>
#else
# include <time.h>
# include <unistd.h>
#endif

/* pixformat - color format */
#define TEGRA_DC_EXT_FMT_P1		0
#define TEGRA_DC_EXT_FMT_P2		1
#define TEGRA_DC_EXT_FMT_P4		2
#define TEGRA_DC_EXT_FMT_P8		3
#define TEGRA_DC_EXT_FMT_B4G4R4A4	4
#define TEGRA_DC_EXT_FMT_B5G5R5A	5
#define TEGRA_DC_EXT_FMT_B5G6R5		6
#define TEGRA_DC_EXT_FMT_AB5G5R5	7
#define TEGRA_DC_EXT_FMT_B8G8R8A8	12
#define TEGRA_DC_EXT_FMT_R8G8B8A8	13
#define TEGRA_DC_EXT_FMT_B6x2G6x2R6x2A8	14
#define TEGRA_DC_EXT_FMT_R6x2G6x2B6x2A8	15
#define TEGRA_DC_EXT_FMT_YCbCr422	16
#define TEGRA_DC_EXT_FMT_YUV422		17
#define TEGRA_DC_EXT_FMT_YCbCr420P	18
#define TEGRA_DC_EXT_FMT_YUV420P	19
#define TEGRA_DC_EXT_FMT_YCbCr422P	20
#define TEGRA_DC_EXT_FMT_YUV422P	21
#define TEGRA_DC_EXT_FMT_YCbCr422R	22
#define TEGRA_DC_EXT_FMT_YUV422R	23
#define TEGRA_DC_EXT_FMT_YCbCr422RA	24
#define TEGRA_DC_EXT_FMT_YUV422RA	25
#define TEGRA_DC_EXT_FMT_YCbCr444P	41
#define TEGRA_DC_EXT_FMT_YCrCb422RSP	46
#define TEGRA_DC_EXT_FMT_YCbCr422RSP	47
#define TEGRA_DC_EXT_FMT_YCrCb444SP	48
#define TEGRA_DC_EXT_FMT_YCbCr444SP	49
#define TEGRA_DC_EXT_FMT_YUV444P	52
#define TEGRA_DC_EXT_FMT_YCrCb420SP	42
#define TEGRA_DC_EXT_FMT_YCbCr420SP	43
#define TEGRA_DC_EXT_FMT_YCrCb422SP	44
#define TEGRA_DC_EXT_FMT_YCbCr422SP	45
#define TEGRA_DC_EXT_FMT_YVU420SP	53
#define TEGRA_DC_EXT_FMT_YUV420SP	54
#define TEGRA_DC_EXT_FMT_YVU422SP	55
#define TEGRA_DC_EXT_FMT_YUV422SP	56
#define TEGRA_DC_EXT_FMT_YVU444SP	59
#define TEGRA_DC_EXT_FMT_YUV444SP	60
/* color format type field is 8-bits */
#define TEGRA_DC_EXT_FMT_SHIFT		0
#define TEGRA_DC_EXT_FMT_MASK		(0xff << TEGRA_DC_EXT_FMT_SHIFT)

/* pixformat - byte order options ( w x y z ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_NOSWAP	(0 << 8) /* ( 3 2 1 0 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP2	(1 << 8) /* ( 2 3 0 1 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP4	(2 << 8) /* ( 0 1 2 3 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP4HW	(3 << 8) /* ( 1 0 3 2 ) */
/* the next two are not available on T30 or earlier */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP02	(4 << 8) /* ( 3 0 1 2 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAPLEFT	(5 << 8) /* ( 2 1 0 3 ) */
/* byte order field is 4-bits */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SHIFT	8
#define TEGRA_DC_EXT_FMT_BYTEORDER_MASK		\
		(0x0f << TEGRA_DC_EXT_FMT_BYTEORDER_SHIFT)

#define TEGRA_DC_EXT_BLEND_NONE		0
#define TEGRA_DC_EXT_BLEND_PREMULT	1
#define TEGRA_DC_EXT_BLEND_COVERAGE	2

#define TEGRA_DC_EXT_FLIP_FLAG_INVERT_H		(1 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_INVERT_V		(1 << 1)
#define TEGRA_DC_EXT_FLIP_FLAG_TILED		(1 << 2)
#define TEGRA_DC_EXT_FLIP_FLAG_CURSOR		(1 << 3)
#define TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA	(1 << 4)
#define TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR	(1 << 5)
#define TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN	(1 << 6)
#define TEGRA_DC_EXT_FLIP_FLAG_INTERLACE	(1 << 7)
#define TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED	(1 << 8)
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC	(1 << 9)
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC_V2	(1 << 10)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_MASK	(3 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_FULL	(0 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_LIMITED	(1 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_BYPASS	(2 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_MASK		(7 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_DEFAULT	(0 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC601	(1 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC709	(2 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020	(4 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_MASK	(15 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_DEFAULT	(0 << 16) /* driver selects */
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_NONE	(1 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_SRGB	(2 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_8_10	(4 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_12	(8 << 16)
/*End of window specific flip flags*/
/*Passthrough condition for running 4K HDMI*/
#define TEGRA_DC_EXT_FLIP_HEAD_FLAG_YUVBYPASS	(1 << 0)
#define TEGRA_DC_EXT_FLIP_HEAD_FLAG_VRR_MODE	(1 << 1)
/* Flag to notify attr v2 struct is being used */
#define TEGRA_DC_EXT_FLIP_HEAD_FLAG_V2_ATTR	(1 << 2)
/* Flag for HDR_DATA handling */
#define TEGRA_DC_EXT_FLIP_FLAG_HDR_ENABLE	(1 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_HDR_DATA_UPDATED (1 << 1)
/*
 * Following flag is used when TEGRA_DC_EXT_FLIP_USER_DATA_CMU_V2 is used.
 * FLAG_CMU_V2_UPDATE: this flag is valid only when
 *                     tegra_dc_ext_cmu_v2.cmu_enable = true.
 *                     If flag is present, driver will update LUT
 *                     values provided from userspace.
 *                     If flag is not present then driver will program LUT
 *                     from cached values.
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CMU_V2	(1 << 0)
/* FLAG_UPDATE_OCSC_CS: If flag is present, driver will update
 *                      output colorspace with values provided from userspace
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_CS	(1 << 0)
/* FLAG_UPDATE_OCSC_RANGE: If flag is present, driver will update output color
 *                      range with values provided from userspace
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_RANGE	(1 << 1)
/* Flags for post-syncpt handling */
/* Bits 1:0 are reserved for the post-syncpt type */
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_SHIFT	0
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_MASK \
			(0x3 << TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_SHIFT)
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_FD	(0 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_RAW	(1 << 0)

/* Flags for CEA861.3 defined eotfs in HDR Metadata form sink */
#define TEGRA_DC_EXT_CEA861_3_EOTF_SDR_LR	(1 << 0)
#define TEGRA_DC_EXT_CEA861_3_EOTF_HDR_LR	(1 << 1)
#define TEGRA_DC_EXT_CEA861_3_EOTF_SMPTE_2084	(1 << 2)

struct tegra_timespec {
	__s32	tv_sec; /* seconds */
	__s32	tv_nsec; /* nanoseconds */
};

/*
 * Keeping the old struct to maintain the app compatibility.
 *
 */
struct tegra_dc_ext_flip_windowattr {
	__s32	index;
	__u32	buff_id;
	__u32	blend;
	__u32	offset;
	__u32	offset_u;
	__u32	offset_v;
	__u32	stride;
	__u32	stride_uv;
	__u32	pixformat;
	/*
	 * x, y, w, h are fixed-point: 20 bits of integer (MSB) and 12 bits of
	 * fractional (LSB)
	 */
	__u32	x;
	__u32	y;
	__u32	w;
	__u32	h;
	__u32	out_x;
	__u32	out_y;
	__u32	out_w;
	__u32	out_h;
	__u32	z;
	__u32	swap_interval;
	struct tegra_timespec timestamp;
	union {
		struct {
			__u32 pre_syncpt_id;
			__u32 pre_syncpt_val;
		};
		__s32 pre_syncpt_fd;
	};
	/* These two are optional; if zero, U and V are taken from buff_id */
	__u32	buff_id_u;
	__u32	buff_id_v;
	__u32	flags;
	__u8	global_alpha; /* requires TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA */
	/* log2(blockheight) for blocklinear format */
	__u8	block_height_log2;
	__u8	pad1[2];
	union { /* fields for mutually exclusive options */
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_INTERLACE set */
			__u32	offset2;
			__u32	offset_u2;
			__u32	offset_v2;
			/* Leave some wiggle room for future expansion */
			__u32   pad2[1];
		};
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED set */
			__u32	buff_id; /* take from buff_id if zero */
			__u32	offset; /* added to base */
			__u16	offset_x;
			__u16	offset_y;
			__u32	zbc_color;
		} cde;
		struct { /* TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC */
			__u16 yof;	/* s.7.0 */
			__u16 kyrgb;	/*   2.8 */
			__u16 kur;	/* s.2.8 */
			__u16 kvr;	/* s.2.8 */
			__u16 kug;	/* s.1.8 */
			__u16 kvg;	/* s.1.8 */
			__u16 kub;	/* s.2.8 */
			__u16 kvb;	/* s.2.8 */
		} csc;
	};
};

/*
 * New struct added for CSC changes, CSC is using
 * 4X3 matrix instead of 3X3 one.
 * Existing struct is modified to add csc_v2 support
 * This is done to minimize the code change for struct change.
 */
struct tegra_dc_ext_flip_windowattr_v2 {
	__s32	index;
	__u32	buff_id;
	__u32	blend;
	__u32	offset;
	__u32	offset_u;
	__u32	offset_v;
	__u32	stride;
	__u32	stride_uv;
	__u32	pixformat;
	/*
	 * x, y, w, h are fixed-point: 20 bits of integer (MSB) and 12 bits of
	 * fractional (LSB)
	 */
	__u32	x;
	__u32	y;
	__u32	w;
	__u32	h;
	__u32	out_x;
	__u32	out_y;
	__u32	out_w;
	__u32	out_h;
	__u32	z;
	__u32	swap_interval;
	struct tegra_timespec timestamp;
	union {
		struct {
			__u32 pre_syncpt_id;
			__u32 pre_syncpt_val;
		};
		__s32 pre_syncpt_fd;
	};
	/* These two are optional; if zero, U and V are taken from buff_id */
	__u32	buff_id_u;
	__u32	buff_id_v;
	__u32	flags;
	__u8	global_alpha; /* requires TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA */
	/* log2(blockheight) for blocklinear format */
	__u8	block_height_log2;
	__u8	pad1[2];
	union { /* fields for mutually exclusive options */
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_INTERLACE set */
			__u32	offset2;
			__u32	offset_u2;
			__u32	offset_v2;
			/* Leave some wiggle room for future expansion */
			__u32   pad2[9];
		};
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED set */
			__u32	buff_id; /* take from buff_id if zero */
			__u32	offset; /* added to base */
			__u16	offset_x;
			__u16	offset_y;
			__u32	zbc_color;
			__u32   reserved[8];
		} cde;
		struct { /* TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC */
			__u16 yof;	/* s.7.0 */
			__u16 kyrgb;	/*   2.8 */
			__u16 kur;	/* s.2.8 */
			__u16 kvr;	/* s.2.8 */
			__u16 kug;	/* s.1.8 */
			__u16 kvg;	/* s.1.8 */
			__u16 kub;	/* s.2.8 */
			__u16 kvb;	/* s.2.8 */
			__u32   reserved[8];
		} csc;
		struct { /* TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC_V2 */
			__u32 r2r;	/* s.3.16 */
			__u32 g2r;	/* s.3.16 */
			__u32 b2r;	/* s.3.16 */
			__u32 const2r;	/* s.3.16 */
			__u32 r2g;	/* s.3.16 */
			__u32 g2g;	/* s.3.16 */
			__u32 b2g;	/* s.3.16 */
			__u32 const2g;	/* s.3.16 */
			__u32 r2b;	/* s.3.16 */
			__u32 g2b;	/* s.3.16 */
			__u32 b2b;	/* s.3.16 */
			__u32 const2b;	/* s.3.16 */
		} csc2;
	};
};

#define TEGRA_DC_EXT_FLIP_N_WINDOWS	3

struct tegra_dc_ext_flip {
	struct tegra_dc_ext_flip_windowattr win[TEGRA_DC_EXT_FLIP_N_WINDOWS];
	__u32 post_syncpt_id;
	__u32 post_syncpt_val;
};

/*
 * Variable win is the pointer to struct tegra_dc_ext_flip_windowattr.
 * Using the modified struct to avoid code conflict in user mode,
 * To avoid any issue for a precompiled application to use with kernel update,
 * kernel code will copy only sizeof(tegra_dc_ext_flip_windowattr)
 * for flip2 use case.
 *
 */

struct tegra_dc_ext_flip_2 {
	struct tegra_dc_ext_flip_windowattr __user *win;
	__u8 win_num;
	__u8 reserved1; /* unused - must be 0 */
	__u16 reserved2; /* unused - must be 0 */
	__u32 post_syncpt_id;
	__u32 post_syncpt_val;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
};

/*
 * Variable win is the pointer to struct tegra_dc_ext_flip_windowattr_v2.
 * Flags has set to use TEGRA_DC_EXT_FLIP_HEAD_FLAG_V2_ATTR,
 * then use struct tegra_dc_ext_flip_windowattr_v2. If flag is not set
 * to use V2_ATTR, then code will use old struct tegra_dc_ext_flip_windowattr
 *
*/
struct tegra_dc_ext_flip_3 {
	__u64 __user win;
	__u8 win_num;
	__u8 flags;
	__u16 reserved2; /* unused - must be 0 */
	__s32 post_syncpt_fd;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
};

enum tegra_dc_ext_flip_data_type {
	TEGRA_DC_EXT_FLIP_USER_DATA_NONE, /* dummy value - do not use */
	TEGRA_DC_EXT_FLIP_USER_DATA_HDR_DATA,
	TEGRA_DC_EXT_FLIP_USER_DATA_IMP_DATA, /* only valid during PROPOSE */
	TEGRA_DC_EXT_FLIP_USER_DATA_IMP_TAG, /* only valid during FLIP */
	TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT,
	TEGRA_DC_EXT_FLIP_USER_DATA_CSC_V2,
	TEGRA_DC_EXT_FLIP_USER_DATA_CMU_V2,
	TEGRA_DC_EXT_FLIP_USER_DATA_OUTPUT_CSC,
};

/*
 * Static Metadata for HDR
 * This lets us specify which HDR static metadata to specify in the infoframe.
 * Please see CEA 861.3 for more information.
 */
struct tegra_dc_ext_hdr {
	__u8 eotf;
	__u8 static_metadata_id;
	__u8 static_metadata[24];
} __attribute__((__packed__));

/*
 * The value at index i of each window-specific array corresponds to the
 * i-th window that's assigned to this head. For the actual id of the i-th
 * window, look in win_ids[i].
 */
#define TEGRA_DC_EXT_N_HEADS	3
#define TEGRA_DC_EXT_N_WINDOWS	6
/*
 * IMP info that's exported to userspace.
 */
struct tegra_dc_ext_imp_emc_dvfs_pair {
	__u32 freq;	/* core EMC frequency (KHz) */
	__u32 latency;	/* DVFS latency (ns) */
};

struct tegra_dc_ext_imp_user_info {
	__u32 num_windows; /* in */
	__u32 __user *win_ids; /* in */
	__u32 __user *in_widths; /* in */
	__u32 __user *out_widths; /* in */
	__u32 emc_dvfs_pairs_requested; /* in */
	__u32 mempool_size; /* out */
	__u32 __user *v_taps; /* out */
	__u32 emc_dvfs_pairs_returned; /* out */
	struct tegra_dc_ext_imp_emc_dvfs_pair __user *emc_dvfs_pairs; /* out */
};

struct tegra_dc_ext_imp_head_results {
	__u32	num_windows;
	__u8	cursor_active;
	__u32	win_ids[TEGRA_DC_EXT_N_WINDOWS];
	__u32	thread_group_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	metering_slots_value_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	thresh_lwm_dvfs_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	pipe_meter_value_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	pool_config_entries_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	metering_slots_value_cursor;
	__u32	thresh_lwm_dvfs_cursor;
	__u32	pipe_meter_value_cursor;
	__u32	pool_config_entries_cursor;
	__u8	head_active;
	__u64	reserved[4]; /* reserved - must be 0 */
};

struct tegra_dc_ext_imp_settings {
	struct tegra_dc_ext_imp_head_results imp_results[TEGRA_DC_EXT_N_HEADS];
	__u64 hubclk;
	__u32 window_slots_value;
	__u32 cursor_slots_value;
	__u64 required_total_bw_kbps;
	__u64 total_display_iso_bw_kbps;
	__u64 proposed_emc_hz;
	__u64 __user session_id_ptr; /* out - ptr to unsigned 64-bit val */
	__u64 reserved[4]; /* reserved - must be 0 */
};

/*
 * Variable settings is a pointer to tegra_dc_ext_imp_settings.
 * reserved is padding so that the total struct size is 26 bytes.
 */
struct tegra_dc_ext_imp_ptr {
	__u64 __user settings;
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

/*
 * Variable session_id is a unique per-head id that designates which IMP
 * settings actually correspond to this flip.
 * reserved is padding so that the total struct size is 26 bytes.
 */
struct tegra_dc_ext_imp_flip_tag {
	__u64 session_id;
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

/*
 * syncpt_id and syncpt_val are used for raw syncpts. syncpt_fd is used for the
 * fd variant.
 * reserved is padding so that the total struct size is 26 bytes.
 *
 * Users can explicitly request post-syncpts as part of the flip user data by
 * doing the following:
 * 1) Set the flip user data type to TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT.
 * 2) Set the flip user data flags to select the requested syncpt type. See the
 *    TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_* flags for the supported options.
 *
 * The kernel will fill in the actual struct fields based on the requested
 * syncpt type.
 *
 * There are a few caveats to this mechanism that users should be aware of:
 * 1) The original post-syncpt fence in whatever FLIP ioctl struct is being used
 *    will NOT be honored. This is a MUTUALLY EXCLUSIVE option.
 * 2) The requested post-syncpt type will be enforced for the window pre-syncpts
 *    as well, so it's up to the user to make sure these fields are specified in
 *    a consistent manner.
 * 3) Only one flip user data of this type is allowed. If more than one is
 *    specified, the FLIP ioctl will return failure.
 */
struct tegra_dc_ext_syncpt {
	union {
		struct {
			__u32 syncpt_id;
			__u32 syncpt_val;
		};
		__s32 syncpt_fd;
	};
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

struct tegra_dc_ext_udata_csc_v2 {
	__u64 __user array; /* pointer to an array of "tegra_dc_ext_csc_v2" */
	__u8 nr_elements;
	__u8 reserved[17];
} __attribute__((__packed__));

struct tegra_dc_ext_udata_cmu_v2 {
	__u64 __user cmu_v2; /* pointer to "tegra_dc_ext_cmu_v2" */
	__u8 reserved[18];
} __attribute__((__packed__));

struct tegra_dc_ext_udata_output_csc {
	__u32 output_colorspace;
	/* Valid values for output colorspace:
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC601
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC709
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020
	 */
	__u8 limited_range_enable;
	__u8 reserved[21];
} __attribute__((__packed__));

/* size of the this struct is 32 bytes */
struct tegra_dc_ext_flip_user_data {
	__u8 data_type;
	__u8 reserved0;
	__u16 flags;
	__u16 reserved1;
	union { /* data to be packed into 26 bytes */
		__u8 data8[26];
		__u16 data16[13];
		struct tegra_dc_ext_hdr hdr_info;
		struct tegra_dc_ext_imp_ptr imp_ptr;
		struct tegra_dc_ext_imp_flip_tag imp_tag;
		struct tegra_dc_ext_syncpt post_syncpt; /* out */
		struct tegra_dc_ext_udata_csc_v2 csc_v2;
		struct tegra_dc_ext_udata_cmu_v2 cmu_v2;
		struct tegra_dc_ext_udata_output_csc output_csc;
	};
} __attribute__((__packed__));

/*
 *tegra_dc_flip_4 : Incorporates a new pointer to an array of 32 bytes of data
 *to pass head specific info. The new nr_elements carries the number of such
 *elements. Everything else remains the same as in tegra_dc_ext_flip_3
 */
struct tegra_dc_ext_flip_4 {
	__u64 __user win;
	__u8 win_num;
	__u8 flags;
	__u16 reserved2; /* unused - must be 0 */
	__s32 post_syncpt_fd;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
	__u32 nr_elements; /* number of data entities pointed to by data */
	__u64 __user data; /* pointer to struct tegra_dc_ext_flip_user_data*/
};

/*
 * vblank control - enable or disable vblank events
 */

struct tegra_dc_ext_set_vblank {
	__u8	enable;
	__u8	reserved[3]; /* unused - must be 0 */
};

/* tegra_dc_ext_cap_type : Defines the diffrent types of capability info that
 * could be provided by the kernel to user space.
 */
enum tegra_dc_ext_cap_type {
	TEGRA_DC_EXT_CAP_TYPE_NONE, /* dummy value - do not use */
	TEGRA_DC_EXT_CAP_TYPE_HDR_SINK, /* struct tegra_dc_ext_hdr_caps */
	TEGRA_DC_EXT_CAP_TYPE_QUANT_SELECTABLE,
		/* struct tegra_dc_ext_quant_caps */
	TEGRA_DC_EXT_CAP_TYPE_MAX,
};

/*
 * tegra_dc_ext_hdr_caps : Incorporates target display's hdr capabilities.
 * nr_elements : Indicates the number of the following data. When set to 0,
 * the sink didn't provide the hdr static metadata in the edid.
 * eotf : Indiactes the eotf supported by the sink.
 * static_metadata_type : indicates which Static Metadata Descriptors are
 * supported.
 * desired_content_max_lum : Code Value indicating the Desired Content Max
 * Luminance Data
 * desired_content_max_frame_avg_lum : Code Value indicating the Desired
 * Content Max Frame-average Luminance.
 * desired_content_min_lum : Code Value indicating the Desired Content Min
 * Luminance.
 *
 * Note:  The last 3 data are optional to declare in the edid. When nr_elements
 * = 3, they are absent. When nr_elements = 4, desired_content_max_lum is
 * present; when nr_elements = 5, desired_content_max_lum and
 * desired_content_max_frame_avg_lum are present; and when it's 6, all 3 of
 * them are present. When nr_elements > 3, each of the 3 values which are
 * indicated to be present in the HDR Static Metadata Data Block may be set to
 * zero. This value indicates that the data for the relevant Desired Max
 * Content Luminance, Desired Content Max Frameaverage Luminance or Desired
 * Content Min Luminance is not indicated.
 */
struct tegra_dc_ext_hdr_caps {
	__u8 nr_elements;
	__u8 eotf;
	__u8 static_metadata_type;
	__u8 desired_content_max_lum;
	__u8 desired_content_max_frame_avg_lum;
	__u8 desired_content_min_lum;
};

/*
 * tegra_dc_ext_quant_caps : Incorporates target display's quantization
 * capabilities.
 * rgb_quant_selectable : indicates whether rgb quantization range is
 * selectable
 * yuv_quant_selectable : indicates whether yuv quantization range is
 * selectable
 */
struct tegra_dc_ext_quant_caps {
	__u8 rgb_quant_selectable;
	__u8 yuv_quant_selectable;
};

/*
 * tegra_dc_ext_caps : Incorporates target display capabilities.
 * data_type : Indicates the type of capability.
 * data = pointer to the actual data.
 */
struct tegra_dc_ext_caps {
	__u32 data_type;
	__u64 __user data;
};

/*
 * get display capabilities.
 * nr_elements : Inidicates the no of elements of "tegra_dc_ext_caps" type the
 * data pointer in pointing to.
 * data : pointer to tegra_dc_ext_caps.
 */
struct tegra_dc_ext_get_cap_info {
	__u32	nr_elements;
	__u64 __user data;
};

/*
 * Cursor image format:
 *
 * Tegra hardware supports two different cursor formats:
 *
 * (1) Two color cursor: foreground and background colors are
 *     specified by the client in RGB8.
 *
 *     The two-color image should be specified as two 1bpp bitmaps
 *     immediately following each other in memory.  Each pixel in the
 *     final cursor will be constructed from the bitmaps with the
 *     following logic:
 *
 *		bitmap1 bitmap0
 *		(mask)  (color)
 *		  1	   0	transparent
 *		  1	   1	inverted
 *		  0	   0	background color
 *		  0	   1	foreground color
 *
 *     This format is supported when TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES
 *     reports the TEGRA_DC_EXT_CAPABILITIES_CURSOR_TWO_COLOR bit.
 *
 * (2) RGBA cursor: in this case the image is four bytes per pixel,
 *     with alpha in the low eight bits.
 *
 *     The RGB components of the cursor image can be either
 *     premultipled by alpha:
 *
 *         cursor[r,g,b] + desktop[r,g,b] * (1 - cursor[a])
 *
 *     or not:
 *
 *         cursor[r,g,b] * cursor[a] + desktop[r,g,b] * (1 - cursor[a])
 *
 *     TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES will report one or more of
 *     TEGRA_DC_EXT_CURSOR_FLAGS_RGBA{,_NON}_PREMULT_ALPHA to indicate
 *     which are supported on the current hardware.
 *
 * Specify one of TEGRA_DC_EXT_CURSOR_FLAGS to indicate the format.
 *
 * Exactly one of the SIZE flags must be specified.
 */


#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32	((1 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64	((2 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_128x128	((3 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_256x256	((4 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE(x)		(((x) & 0x7) >> 0)

#define TEGRA_DC_EXT_CURSOR_FORMAT_2BIT_LEGACY			(0)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA	(1)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_PREMULT_ALPHA		(3)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_XOR			(4)

#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_2BIT_LEGACY \
	(TEGRA_DC_EXT_CURSOR_FORMAT_2BIT_LEGACY << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_NON_PREMULT_ALPHA	\
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_PREMULT_ALPHA \
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_PREMULT_ALPHA << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_XOR \
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_XOR << 16)

#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS(x)	(((x) >> 16) & 0x7)

#define TEGRA_DC_EXT_CURSOR_COLORFMT_LEGACY	(0)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_R8G8B8A8	(1)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_A1R5G5B5	(2)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_A8R8G8B8	(3)

#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_LEGACY \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_LEGACY << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_R8G8B8A8 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_R8G8B8A8 << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A1R5G5B5 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_A1R5G5B5 << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A8R8G8B8 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_A8R8G8B8 << 8)

#define TEGRA_DC_EXT_CURSOR_COLORFMT_FLAGS(x)	(((x) >> 8) & 0xf)

/* aliases for source-level backwards compatibility */
#define TEGRA_DC_EXT_CURSOR_FLAGS_RGBA_NORMAL \
	TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_NON_PREMULT_ALPHA
#define TEGRA_DC_EXT_CURSOR_FLAGS_2BIT_LEGACY \
	TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_2BIT_LEGACY

#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MIN (0x00)
#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MAX (0xff)
#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MSK (0xff)

enum CURSOR_COLOR_FORMAT {
	legacy,		/* 2bpp */
	r8g8b8a8,	/* normal */
	a1r5g5b5,
	a8r8g8b8,
};

struct tegra_dc_ext_cursor_image {
	struct {
		__u8	r;
		__u8	g;
		__u8	b;
	} foreground, background;
	__u32	buff_id;
	__u32	flags;
	__s16	x;
	__s16	y;
	__u32	alpha; /* was vis*/
	enum CURSOR_COLOR_FORMAT colorfmt; /* was mode */
};

/* Possible flags for struct nvdc_cursor's flags field */
#define TEGRA_DC_EXT_CURSOR_FLAGS_VISIBLE	(1 << 0)

struct tegra_dc_ext_cursor {
	__s16 x;
	__s16 y;
	__u32 flags;
};

/*
 * Color conversion is performed as follows:
 *
 * r = sat(kyrgb * sat(y + yof) + kur * u + kvr * v)
 * g = sat(kyrgb * sat(y + yof) + kug * u + kvg * v)
 * b = sat(kyrgb * sat(y + yof) + kub * u + kvb * v)
 *
 * Coefficients should be specified as fixed-point values; the exact format
 * varies for each coefficient.
 * The format for each coefficient is listed below with the syntax:
 * - A "s." prefix means that the coefficient has a sign bit (twos complement).
 * - The first number is the number of bits in the integer component (not
 *   including the optional sign bit).
 * - The second number is the number of bits in the fractional component.
 *
 * All three fields should be tightly packed, justified to the LSB of the
 * 16-bit value.  For example, the "s.2.8" value should be packed as:
 * (MSB) 5 bits of 0, 1 bit of sign, 2 bits of integer, 8 bits of frac (LSB)
 */
struct tegra_dc_ext_csc {
	__u32 win_index;
	__u16 yof;	/* s.7.0 */
	__u16 kyrgb;	/*   2.8 */
	__u16 kur;	/* s.2.8 */
	__u16 kvr;	/* s.2.8 */
	__u16 kug;	/* s.1.8 */
	__u16 kvg;	/* s.1.8 */
	__u16 kub;	/* s.2.8 */
	__u16 kvb;	/* s.2.8 */
};

/*
 * Coefficients should be specified as fixed-point values; the exact format
 * varies for each coefficient.
 * Each coefficient is a signed 19bit number with 3 integer bits and 16
 * fractional bits. Overall range is from -4.0 to 3.999
 * All three fields should be tightly packed in 32bit
 * For example, the "s.3.16" value should be packed as:
 * (MSB) 12 bits of 0, 1 bit of sign, 3 bits of integer, 16 bits of frac (LSB)
 */
struct tegra_dc_ext_csc_v2 {
	__u32 win_index;
	__u32 csc_enable;
	__u32 r2r;		/* s.3.16 */
	__u32 g2r;		/* s.3.16 */
	__u32 b2r;		/* s.3.16 */
	__u32 const2r;		/* s.3.16 */
	__u32 r2g;		/* s.3.16 */
	__u32 g2g;		/* s.3.16 */
	__u32 b2g;		/* s.3.16 */
	__u32 const2g;		/* s.3.16 */
	__u32 r2b;		/* s.3.16 */
	__u32 g2b;		/* s.3.16 */
	__u32 b2b;		/* s.3.16 */
	__u32 const2b;		/* s.3.16 */
};

struct tegra_dc_ext_cmu {
	__u16 cmu_enable;
	__u16 csc[9];
	__u16 lut1[256];
	__u16 lut2[960];
};

/*
 * Two types for LUT size 257 or 1025
 * Based on lut size the input width is different
 * Each component is in 16 bit format
 * For example With Unity LUT range with 1025 size
 * Each component (R,G,B) content is in 14bits
 * Index bits are in upper 10 bits
 * Black to White range from 0x6000 to 0x9FFF
 * LUT array is represented in 64 bit, each component is shifted
 * appropriately to represent in 64bit data.
 * For example, rgb[i] = (B << 32) | (G << 16) | (R << 0)
 * lut_ranges - input value range covered by lut,
 * it can be unity (0.0 ..1.0), xrbias(-0.75 .. +1.25),
 * xvycc(-1.5 to +2.5)
 * lut_mode - index or interpolate
 */
#define TEGRA_DC_EXT_LUT_SIZE_257	256
#define TEGRA_DC_EXT_LUT_SIZE_1025	1024

#define TEGRA_DC_EXT_OUTLUT_MODE_INDEX		0
#define TEGRA_DC_EXT_OUTLUT_MODE_INTERPOLATE	1

#define TEGRA_DC_EXT_OUTLUT_RANGE_UNITY		0
#define TEGRA_DC_EXT_OUTLUT_RANGE_XRBAIS	1
#define TEGRA_DC_EXT_OUTLUT_RANGE_XVYCC		2

struct tegra_dc_ext_cmu_v2 {
	__u16 cmu_enable;
	__u16 lut_size;
	__u16 lut_range; /* ignored in the driver */
	__u16 lut_mode; /* ignored in the driver */
	__u64 rgb[TEGRA_DC_EXT_LUT_SIZE_1025 + 1];
};

/*
 * RGB Lookup table
 *
 * In true-color and YUV modes this is used for post-CSC RGB->RGB lookup, i.e.
 * gamma-correction. In palette-indexed RGB modes, this table designates the
 * mode's color palette.
 *
 * To convert 8-bit per channel RGB values to 16-bit, duplicate the 8 bits
 * in low and high byte, e.g. r=r|(r<<8)
 *
 * To just update flags, set len to 0.
 *
 * Current Tegra DC hardware supports 8-bit per channel to 8-bit per channel,
 * and each hardware window (overlay) uses its own lookup table.
 *
 */
struct tegra_dc_ext_lut {
	__u32  win_index; /* window index to set lut for */
	__u32  flags;     /* Flag bitmask, see TEGRA_DC_EXT_LUT_FLAGS_* */
	__u32  start;     /* start index to update lut from */
	__u32  len;       /* number of valid lut entries */
	__u16 __user *r;         /* array of 16-bit red values, 0 to reset */
	__u16 __user *g;         /* array of 16-bit green values, 0 to reset */
	__u16 __user *b;         /* array of 16-bit blue values, 0 to reset */
};

/* tegra_dc_ext_lut.flags - override global fb device lookup table.
 * Default behaviour is double-lookup.
 */
#define TEGRA_DC_EXT_LUT_FLAGS_FBOVERRIDE 0x01

#define TEGRA_DC_EXT_FLAGS_ENABLED	1
struct tegra_dc_ext_status {
	__u32 flags;
	/* Leave some wiggle room for future expansion */
	__u32 pad[3];
};

struct tegra_dc_ext_feature {
	__u32 length;
	__u32 __user *entries;
};


/*
 * Tegra Display Screen Capture
 *
 * This feature will make a snap shot of display HW configurations and its
 * frame buffer contents, so reconstruction of full display screen would be
 * possible.
 *
 * A screen capture will be made in following sequence.
 *
 * 1. Pause display flip on all heads with the IOCTL
 *    TEGRA_DC_EXT_CONTROL_SCRNCAPT_PAUSE. The kernel driver may also set a
 *    timer for automatic resume. The default timer value for native Linux is
 *    0.5Sec and user app can override the default timer value. The timer
 *    value can vary depending on each OS and HyperVisor configuration.
 *    The display pause is an exclusive operation. Once a pause is called,
 *    no more pause call is allowed until resuming the pause in effective.
 *    For this purpose, the pause call will return a magic number. It should
 *    be saved and used by following screen capture calls until the resume.
 * 2. Collect configration information of a display head and all windows
 *    assigned to the head with the IOCTL TEGRA_DC_EXT_SCRNCAPT_GET_INFO.
 * 3. Duplicate the frame buffer of the latest flip of a window that is
 *    currently on display with the IOCTL TEGRA_DC_EXT_SCRNCAPT_DUP_FBUF.
 * 4. Repeat the step 3 for every window assigned to the head.
 * 5. Repeat the step 2 through step 4 for every head to capture.
 * 6. Resume display flip on all heads and clear the auto resume timer with
 *    the IOCTL TEGRA_DC_EXT_CONTROL_SCRNCAPT_RESUME. Pausing and resuming
 *    flips are intended for all display heads to make the internal logic
 *    simple and efficient. This means no individual head pause and no partial
 *    resumming are supported.
 */

/* To collect configuration information of one display head
 */
/* API data structure version and magic number */
#define TEGRA_DC_EXT_SCRNCAPT_VER_2(magic) (((magic) & ~0xff) | 2)
#define TEGRA_DC_EXT_SCRNCAPT_VER_V(magic) ((magic) & 0xff)
/* info_type HEAD: struct tegra_dc_ext_scrncapt_get_info_head */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_HEAD (0)
/* info_type WIN: struct tegra_dc_ext_scrncapt_get_info_win */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_WINS (1)
/* info_type CURSOR: struct tegra_dc_ext_cursor_image */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR (2)
/* info_type CURSOR_DATA: struct tegra_dc_ext_scrncapt_get_info_cursor_data */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR_DATA (3)
/* info_type CMU_V2: struct tegra_dc_ext_cmu_v2 */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CMU_V2 (4)
/* info_type CSC_V2: struct tegra_dc_ext_csc_v2 */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CSC_V2 (5)
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_FLAG_WINS(n)  (1u << (n))

struct tegra_dc_ext_scrncapt_get_info_head {
	__u8   sts_en;        /* returns display head enabled or not */
	__u32  hres;          /* returns display horizontal resolution */
	__u32  vres;          /* returns display vertical resolution */
	__u32  flag_val_wins; /* returns valid windows mask */
	__u32  reserved[12];
};

struct tegra_dc_ext_scrncapt_get_info_win {
	__u32 flag_wins; /* bitmask of TEGRA_DC_EXT_SCRNCAPT_GET_INFO_FLAG_WINS
			  * set bit to indicate the window to be captured */
	__u32 num_wins;    /* returns number of windows saved to 'wins' */
	__u64 __user wins; /* pointer to array of
			    * struct tegra_dc_ext_flip_windowattr_v2.
			    * should have enough entries to hold 'flag_wins'
			    * selection. */
	__u32 reserved[8];
};

struct tegra_dc_ext_scrncapt_get_info_cursor_data {
	__u32 size;       /* byte size of total space allocated to 'ptr' */
	__u32 len;        /* returns byte size of data copied into 'ptr' */
	__u64 __user ptr; /* pointer to buffer */
	__u32 reserved[4];
};

struct tegra_dc_ext_scrncapt_get_info_data {
	__u32 type; /* type of data, TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_xxx */
	__u64 __user ptr; /* pointer to its data structure */
	__u32 reserved[3];
};

struct tegra_dc_ext_scrncapt_get_info  {
	__u32 ver;      /* set to TEGRA_DC_EXT_SCRNCAPT_VER_2
			 * returns TEGRA_DC_EXT_SCRNCAPT_VER_2 */
	__u32 head;     /* head ID */
	__u32 num_data; /* number of entries in 'data' */
	__u64 __user data; /* pointer to the array of
			    * struct tegra_dc_ext_scrncapt_get_info_data */
	__u32 reserved[8];
};

/* To duplicate single window raw frame buffer that may consists with one or
 * more planes.
 *
 * RGB pixel formats use only the first plane.
 * YUV(YCbCr) pixel formats use one to 3 planes depending on its format.
 * Y plane holds Y or YUV(YCbCr) data depending on the pixel format.
 * U(Cb) plane holds U(Cb) or UV(CbCr) data depending on the pixel format.
 * V(Cr) plane holds V(Cr) data depending on the pixel format.
 * CDE plane is not used.
 *
 * The returned length of each plane indicates the data length of each plane
 * in the raw frame buffer, and 0 length indicates the plane is not used.
 * The returned offset of each plane indicates the offset to the plane within
 * the data buffer pointed by 'buffer'.
 */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_RGB  0 /* RGB plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_Y    0 /* Y plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_U    1 /* U or Cb plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_V    2 /* V or Cr plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_CDE  3 /* CDE plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM  4 /* number of planes */

struct tegra_dc_ext_scrncapt_dup_fbuf  {
	__u32 ver;      /* set to TEGRA_DC_EXT_SCRNCAPT_VER_2
			 * returns TEGRA_DC_EXT_SCRNCAPT_VER_2 */
	__u32 head;     /* head ID */
	__u32 win;      /* window ID */
	__u32 buffer_max;    /* allocated byte amount to 'buffer' */
	__u64 __user buffer; /* pointer to data buffer to store all planes */
	/* returns length of each plane, 0 for plane not available */
	__u32 plane_sizes[TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM];
	/* returns offset of each plane within the 'buffer' */
	__u32 plane_offsets[TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM];
	__u32 reserved[16];
};

/* Scanline sync ioctl */
#define TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE (1U << 0)
#define TEGRA_DC_EXT_SCANLINE_FLAG_DISABLE (0U << 0)
#define TEGRA_DC_EXT_SCANLINE_FLAG_RAW_SYNCPT (1U << 1)
#define TEGRA_DC_EXT_SCANLINE_FLAG_SYNCFD (0U << 1)
/* unused flags are reserved and must be 0 */

struct tegra_dc_ext_scanline_info {
	__u8 id; /* 0 = VPULSE3 */
	__u8 frame; /* a.k.a. swap_interval, a.k.a. min_present. typically 0 */
	__u16 flags; /* select between fd and id:val */
	__u32 triggered_line;
	union {
		struct {
			__u32 id;
			__u32 val;
		} raw_syncpt; /* used only is RAW_SYNCPT */

		__u32 syncfd;
	};
};

#define TEGRA_DC_EXT_SET_NVMAP_FD \
	_IOW('D', 0x00, __s32)

#define TEGRA_DC_EXT_GET_WINDOW \
	_IOW('D', 0x01, __u32)
#define TEGRA_DC_EXT_PUT_WINDOW \
	_IOW('D', 0x02, __u32)

#define TEGRA_DC_EXT_FLIP \
	_IOWR('D', 0x03, struct tegra_dc_ext_flip)

#define TEGRA_DC_EXT_GET_CURSOR \
	_IO('D', 0x04)
#define TEGRA_DC_EXT_PUT_CURSOR \
	_IO('D', 0x05)
#define TEGRA_DC_EXT_SET_CURSOR_IMAGE \
	_IOW('D', 0x06, struct tegra_dc_ext_cursor_image)
#define TEGRA_DC_EXT_SET_CURSOR \
	_IOW('D', 0x07, struct tegra_dc_ext_cursor)

#define TEGRA_DC_EXT_SET_CSC \
	_IOW('D', 0x08, struct tegra_dc_ext_csc)

#define TEGRA_DC_EXT_GET_STATUS \
	_IOR('D', 0x09, struct tegra_dc_ext_status)

/*
 * Returns the auto-incrementing vblank syncpoint for the head associated with
 * this device node
 */
#define TEGRA_DC_EXT_GET_VBLANK_SYNCPT \
	_IOR('D', 0x09, __u32)

#define TEGRA_DC_EXT_SET_LUT \
	_IOW('D', 0x0A, struct tegra_dc_ext_lut)

#define TEGRA_DC_EXT_GET_FEATURES \
	_IOW('D', 0x0B, struct tegra_dc_ext_feature)

#define TEGRA_DC_EXT_CURSOR_CLIP \
	_IOW('D', 0x0C, __s32)

#define TEGRA_DC_EXT_SET_CMU \
	_IOW('D', 0x0D, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_FLIP2 \
	_IOWR('D', 0x0E, struct tegra_dc_ext_flip_2)

#define TEGRA_DC_EXT_GET_CMU \
	_IOR('D', 0x0F, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_GET_CUSTOM_CMU \
	_IOR('D', 0x10, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_SET_PROPOSED_BW \
	_IOR('D', 0x13, struct tegra_dc_ext_flip_2)

#define TEGRA_DC_EXT_FLIP3 \
	_IOWR('D', 0x14, struct tegra_dc_ext_flip_3)

#define TEGRA_DC_EXT_SET_VBLANK \
	_IOW('D', 0x15, struct tegra_dc_ext_set_vblank)

#define TEGRA_DC_EXT_SET_CMU_ALIGNED \
	_IOW('D', 0x16, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_SET_CSC_V2 \
	_IOW('D', 0x17, struct tegra_dc_ext_csc_v2)

#define TEGRA_DC_EXT_SET_CMU_V2 \
	_IOW('D', 0x18, struct tegra_dc_ext_cmu_v2)

#define TEGRA_DC_EXT_GET_CMU_V2 \
	_IOR('D', 0x19, struct tegra_dc_ext_cmu_v2)

#define TEGRA_DC_EXT_GET_CUSTOM_CMU_V2 \
	_IOR('D', 0x1A, struct tegra_dc_ext_cmu_v2)

#define TEGRA_DC_EXT_SET_PROPOSED_BW_3 \
	_IOWR('D', 0x1B, struct tegra_dc_ext_flip_4)

#define TEGRA_DC_EXT_GET_CMU_ADBRGB\
	_IOR('D', 0x1C, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_FLIP4\
	_IOW('D', 0x1D, struct tegra_dc_ext_flip_4)

#define TEGRA_DC_EXT_GET_WINMASK \
	_IOR('D', 0x1E, __u32)

#define TEGRA_DC_EXT_SET_WINMASK \
	_IOW('D', 0x1F, __u32)

#define TEGRA_DC_EXT_GET_IMP_USER_INFO \
	_IOW('D', 0x20, struct tegra_dc_ext_imp_user_info)

#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO \
	_IOWR('D', 0x21, struct tegra_dc_ext_scrncapt_get_info)

#define TEGRA_DC_EXT_SCRNCAPT_DUP_FBUF \
	_IOWR('D', 0x22, struct tegra_dc_ext_scrncapt_dup_fbuf)

#define TEGRA_DC_EXT_GET_CAP_INFO\
	_IOW('D', 0x23, struct tegra_dc_ext_get_cap_info)

#define TEGRA_DC_EXT_GET_SCANLINE \
	_IOR('D', 0x24, __u32)

#define TEGRA_DC_EXT_SET_SCANLINE \
	_IOWR('D', 0x25, struct tegra_dc_ext_scanline_info)

enum tegra_dc_ext_control_output_type {
	TEGRA_DC_EXT_DSI,
	TEGRA_DC_EXT_LVDS,
	TEGRA_DC_EXT_VGA,
	TEGRA_DC_EXT_HDMI,
	TEGRA_DC_EXT_DVI,
	TEGRA_DC_EXT_DP,
	TEGRA_DC_EXT_EDP,
	TEGRA_DC_EXT_NULL,
	TEGRA_DC_EXT_HDSI, /*support DSI as external display*/
};

/*
 * Get the properties for a given output.
 *
 * handle (in): Which output to query
 * type (out): Describes the type of the output
 * connected (out): Non-zero iff the output is currently connected
 * associated_head (out): The head number that the output is currently
 *      bound to.  -1 iff the output is not associated with any head.
 * head_mask (out): Bitmask of which heads the output may be bound to (some
 *      outputs are permanently bound to a single head).
 */
struct tegra_dc_ext_control_output_properties {
	__u32 handle;
	enum tegra_dc_ext_control_output_type type;
	__u32 connected;
	__s32 associated_head;
	__u32 head_mask;
};

/*
 * This allows userspace to query the raw EDID data for the specified output
 * handle.
 *
 * Here, the size parameter is both an input and an output:
 * 1. Userspace passes in the size of the buffer allocated for data.
 * 2. If size is too small, the call fails with the error EFBIG; otherwise, the
 *    raw EDID data is written to the buffer pointed to by data.  In both
 *    cases, size will be filled in with the size of the data.
 */
struct tegra_dc_ext_control_output_edid {
	__u32 handle;
	__u32 size;
	void __user *data;
};

struct tegra_dc_ext_event {
	__u32	type;
	__u32	data_size;
	char	data[0];
};

/* Events types are bits in a mask */
#define TEGRA_DC_EXT_EVENT_HOTPLUG			(1 << 0)
struct tegra_dc_ext_control_event_hotplug {
	__u32 handle;
};

#define TEGRA_DC_EXT_EVENT_VBLANK			(1 << 1)
struct tegra_dc_ext_control_event_vblank {
	__u32 handle;
	__u32 reserved; /* unused */
	__u64 timestamp_ns;
};

#define TEGRA_DC_EXT_EVENT_BANDWIDTH_INC	(1 << 2)
#define TEGRA_DC_EXT_EVENT_BANDWIDTH_DEC	(1 << 3)
struct tegra_dc_ext_control_event_bandwidth {
	__u32 handle;
	__u32 total_bw;
	__u32 avail_bw;
	__u32 resvd_bw;
};

#define TEGRA_DC_EXT_EVENT_MODECHANGE      (1 << 4)
struct tegra_dc_ext_control_event_modechange {
	__u32 handle;
};

#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_MODE			(1 << 0)
#define TEGRA_DC_EXT_CAPABILITIES_BLOCKLINEAR			(1 << 1)

#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_TWO_COLOR		(1 << 2)
#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_RGBA_NON_PREMULT_ALPHA	(1 << 3)
#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_RGBA_PREMULT_ALPHA	(1 << 4)
#define TEGRA_DC_EXT_CAPABILITIES_NVDISPLAY			(1 << 5)

struct tegra_dc_ext_control_capabilities {
	__u32 caps;
	/* Leave some wiggle room for future expansion */
	__u32 pad[3];
};


/* Tegra Display Screen Capture
 * control IOCTL
 */
#define  TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC  (0x73636171)

struct tegra_dc_ext_control_scrncapt_pause {
	__u32  magic; /* call with TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC
		       * returns a magic value for the session */
	__u32  reserved;
	__u32  tm_resume_msec; /* auto resume timer value in mSec
				*   0: use disp driver default value
				*   -1: turn off auto resume timer
				*   others: valid timer value
				* returns the timer set value
				*   -1: auto resume timer turned off
				*   others: timer set value */
	__u32  num_heads; /* returns max number of DC heads */
	__u32  num_wins;  /* returns max number of windows */
	__u32  reserved2[3];
};

struct tegra_dc_ext_control_scrncapt_resume {
	__u32  magic; /* magic value returned from the pause call */
	__u32  reserved[7];
};


#define TEGRA_DC_EXT_CONTROL_GET_NUM_OUTPUTS \
	_IOR('C', 0x00, __u32)
#define TEGRA_DC_EXT_CONTROL_GET_OUTPUT_PROPERTIES \
	_IOWR('C', 0x01, struct tegra_dc_ext_control_output_properties)
#define TEGRA_DC_EXT_CONTROL_GET_OUTPUT_EDID \
	_IOWR('C', 0x02, struct tegra_dc_ext_control_output_edid)
#define TEGRA_DC_EXT_CONTROL_SET_EVENT_MASK \
	_IOW('C', 0x03, __u32)
#define TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES \
	_IOR('C', 0x04, struct tegra_dc_ext_control_capabilities)
#define TEGRA_DC_EXT_CONTROL_SCRNCAPT_PAUSE \
	_IOWR('C', 0x05, struct tegra_dc_ext_control_scrncapt_pause)
#define TEGRA_DC_EXT_CONTROL_SCRNCAPT_RESUME \
	_IOW('C', 0x06, struct tegra_dc_ext_control_scrncapt_resume)

#endif /* __TEGRA_DC_EXT_H */
