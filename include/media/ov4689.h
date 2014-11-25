/*
 * ov4689.h - ov4689 sensor driver
 *
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
 *
 * Contributors:
 *  Jerry Chang <jerchang@nvidia.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OV4689_H__
#define __OV4689_H__

#include <linux/ioctl.h>

#define OV4689_IOCTL_SET_MODE	_IOW('o', 1, struct ov4689_mode)
#define OV4689_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV4689_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV4689_IOCTL_SET_GAIN	_IOW('o', 4, __u16)
#define OV4689_IOCTL_GET_STATUS	_IOR('o', 5, __u8)
#define OV4689_IOCTL_SET_GROUP_HOLD	_IOW('o', 6, struct ov4689_grouphold)
#define OV4689_IOCTL_GET_SENSORDATA	_IOR('o', 7, struct ov4689_sensordata)
#define OV4689_IOCTL_SET_FLASH	_IOW('o', 8, struct ov4689_flash_control)
#define OV4689_IOCTL_SET_POWER          _IOW('o', 20, __u32)

struct ov4689_sensordata {
	__u32 fuse_id_size;
	__u8 fuse_id[16];
};

struct ov4689_mode {
	__u32 xres;
	__u32 yres;
	__u32 fps;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct ov4689_grouphold {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};

struct ov4689_flash_control {
	u8 enable;
	u8 edge_trig_en;
	u8 start_edge;
	u8 repeat;
	u16 delay_frm;
};

#ifdef __KERNEL__
struct ov4689_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vif;
};

struct ov4689_platform_data {
	struct ov4689_flash_control flash_cap;
	int (*power_on)(struct ov4689_power_rail *pw);
	int (*power_off)(struct ov4689_power_rail *pw);
	const char *mclk_name;
	unsigned int reset_gpio;
};
#endif /* __KERNEL__ */

#endif /* __OV4689_H__ */
