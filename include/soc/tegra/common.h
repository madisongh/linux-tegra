/*
 * Copyright (C) 2014 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_TEGRA_COMMON_H__
#define __SOC_TEGRA_COMMON_H__

bool soc_is_tegra(void);
int tegra_get_usb_port_owner_info(void);

#endif /* __SOC_TEGRA_COMMON_H__ */
