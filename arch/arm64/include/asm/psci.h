/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 ARM Limited
 */

#ifndef __ASM_PSCI_H
#define __ASM_PSCI_H

#define PSCI_POWER_STATE_TYPE_STANDBY			0
#define PSCI_POWER_STATE_TYPE_POWER_DOWN		1

struct psci_power_state {
	u16	id;
	u8	type;
	u8	affinity_level;
};

int psci_init(void);
u32 psci_power_state_pack(struct psci_power_state state);
struct psci_power_state to_psci_power_state(unsigned long arg);

#endif /* __ASM_PSCI_H */
