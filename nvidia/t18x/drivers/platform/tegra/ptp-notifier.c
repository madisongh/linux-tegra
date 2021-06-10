/*
 * drivers/platform/tegra/ptp-notifier.c
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/notifier.h>

static u64 (*get_systime)(void);
static DEFINE_SPINLOCK(ptp_notifier_lock);
static ATOMIC_NOTIFIER_HEAD(tegra_hwtime_chain_head);

/* Clients register for notification of hwtime change events */
int tegra_register_hwtime_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&tegra_hwtime_chain_head, nb);
}
EXPORT_SYMBOL(tegra_register_hwtime_notifier);

/* Clients unregister for notification of hwtime change events */
int tegra_unregister_hwtime_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&tegra_hwtime_chain_head, nb);
}
EXPORT_SYMBOL(tegra_unregister_hwtime_notifier);

/* Trigger notification of hwtime change to all registered clients */
int tegra_hwtime_notifier_call_chain(unsigned int val, void *v)
{
	int ret = atomic_notifier_call_chain(&tegra_hwtime_chain_head, val, v);

	return notifier_to_errno(ret);
}

void tegra_register_hwtime_source(u64 (*func)(void))
{
	unsigned long flags;

	spin_lock_irqsave(&ptp_notifier_lock, flags);
	get_systime = func;
	spin_unlock_irqrestore(&ptp_notifier_lock, flags);

	/* Notify HW time stamp update to registered clients.
	 * NULL callback parameter. We use a separate timestamp
	 * function to peek MAC time.
	 */
	tegra_hwtime_notifier_call_chain(0, NULL);
}
EXPORT_SYMBOL(tegra_register_hwtime_source);

void tegra_unregister_hwtime_source(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ptp_notifier_lock, flags);
	get_systime = NULL;
	spin_unlock_irqrestore(&ptp_notifier_lock, flags);
}
EXPORT_SYMBOL(tegra_unregister_hwtime_source);

u64 get_ptp_hwtime(void)
{
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&ptp_notifier_lock, flags);
	if (get_systime)
		ns = get_systime();
	else
		ns = ktime_to_ns(ktime_get_raw());
	spin_unlock_irqrestore(&ptp_notifier_lock, flags);

	return ns;
}
EXPORT_SYMBOL(get_ptp_hwtime);
