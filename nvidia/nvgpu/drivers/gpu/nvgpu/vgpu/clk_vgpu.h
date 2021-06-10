/*
 * Virtualized GPU Clock Interface
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _CLK_VIRT_H_
#define _CLK_VIRT_H_

unsigned long vgpu_clk_get_rate(struct device *dev);
long vgpu_clk_round_rate(struct device *dev, unsigned long rate);
int vgpu_clk_set_rate(struct device *dev, unsigned long rate);
int vgpu_clk_get_freqs(struct device *dev,
		unsigned long **freqs, int *num_freqs);
int vgpu_clk_cap_rate(struct device *dev, unsigned long rate);
#endif
