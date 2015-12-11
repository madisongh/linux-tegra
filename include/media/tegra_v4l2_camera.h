/*
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_CAMERA_H_
#define _TEGRA_CAMERA_H_

#include <linux/regulator/consumer.h>
#include <linux/i2c.h>

enum tegra_camera_port {
	TEGRA_CAMERA_PORT_CSI_A = 0,
	TEGRA_CAMERA_PORT_CSI_B,
	TEGRA_CAMERA_PORT_CSI_C,
	TEGRA_CAMERA_PORT_CSI_D,
	TEGRA_CAMERA_PORT_CSI_E,
	TEGRA_CAMERA_PORT_CSI_F,
	TEGRA_CAMERA_PORT_VIP,
};

#define TEGRA_CAMERA_NUM_CSI_PORT	6

enum camera_gang_mode {
	CAMERA_GANG_DISABLED = 0,
	CAMERA_GANG_L_R,
	CAMERA_GANG_R_L,
	CAMERA_GANG_T_B,
	CAMERA_GANG_B_T,
};

struct tegra_camera_platform_data {
	int			(*enable_camera)(struct platform_device *pdev);
	void			(*disable_camera)(struct platform_device *pdev);
	bool			flip_h;
	bool			flip_v;
	enum tegra_camera_port	port;
	int			lanes;		/* For CSI port only */
	bool			continuous_clk;	/* For CSI port only */
	int			gang_port;
	enum camera_gang_mode   gang_mode;
};

struct i2c_camera_ctrl {
	int	(*new_devices)(struct platform_device *pdev);
	void	(*remove_devices)(struct platform_device *pdev);
};
#endif /* _TEGRA_CAMERA_H_ */
