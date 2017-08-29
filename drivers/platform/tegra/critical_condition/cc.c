/*
 * drivers/platform/tegra/cc.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <soc/tegra/pmc.h>
#include <linux/workqueue.h>

#include "cc.h"

struct crtlcond_platform_data cc_pdata;

static void gamedata_ram_read_write(void)
{
	char *buf;
	size_t len = cc_pdata.read_write_bytes;
	size_t retlen = 0;

	retlen = gamedata_cvt_read(&len, &buf);
	if (!retlen) {
		pr_info("crtcl_cond: Data not available in GameData RAM\n");
		return;
	}
	pr_debug("crtcl_cond: writing Data into FRAM");
	qspi_write(START_ADDRESS0, len, &retlen, buf);
}

static int crtcl_cond_reboot_cb(struct notifier_block *nb,
			      unsigned long event, void *unused)
{
	if (!(cc_pdata.flags & CC_PAGE_WRITE)) {
		pr_info("crtcl_cond: No DATA in GameData RAM, skip write\n");
		return NOTIFY_DONE;
	}
	gamedata_ram_read_write();

	return NOTIFY_DONE;
}

static struct notifier_block crtcl_cond_reboot_notifier = {
	.notifier_call = crtcl_cond_reboot_cb,
};

static int game_data_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int game_data_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t game_data_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int len;

	len = gamedata_cvt_read(&count, &buf);
	if (!len) {
		pr_info("%s: crtcl_cond: no data available in gamedata ram\n",
					 __func__);
		return -EINVAL;
	}
	return len;
}

static ssize_t game_data_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	gamedata_cvt_write(buf, count);
	cc_pdata.flags |= CC_PAGE_WRITE;

	mod_timer(&cc_pdata.cc_timer,
			jiffies + cc_pdata.cc_timer_timeout_jiffies);

	return count;
}

static long game_data_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	return 0;
}

static const struct file_operations game_data_fops = {
	.owner		= THIS_MODULE,
	.write		= game_data_write,
	.read		= game_data_read,
	.unlocked_ioctl	= game_data_ioctl,
	.open		= game_data_open,
	.release	= game_data_release,
};

static struct miscdevice gd_miscdev = {
	.name   = "gamedata",
	.fops   = &game_data_fops,
};

void cc_periodic_save_cvt_data_work(struct work_struct *work)
{
	gamedata_ram_read_write();
}

static void cc_periodic_save_cvt_data_timer(unsigned long _data)
{
	if (!(cc_pdata.flags & CC_PAGE_WRITE)) {
		pr_info("crtcl_cond: No DATA in GameData RAM, skip write\n");
		return;
	}

	schedule_work(&cc_pdata.cc_work);
	mod_timer(&cc_pdata.cc_timer,
			jiffies + cc_pdata.cc_timer_timeout_jiffies);
}

static int crtcl_cond_probe(struct platform_device *pdev)
{
	int ret, reset_reason;

	pdev->dev.platform_data = &cc_pdata;
	ret = gamedata_cvt_probe(pdev);
	if (ret) {
		pr_info("%s: crtcl_cond: gamedata_cvt_probe failed\n",
						 __func__);
		return ret;
	}

	cc_pdata.flags &= ~CC_PAGE_WRITE;

	ret = register_reboot_notifier(&crtcl_cond_reboot_notifier);
	if (ret) {
		pr_info("%s: crtcl_cond: probe failed\n", __func__);
		return ret;
	}

	reset_reason = tegra_reset_reason_status();
	if (reset_reason == WDT_TIMEOUT) {
		pr_info("%s: Reset Reason is Watchdog Timeout\n", __func__);
		gamedata_ram_read_write();
	}

	INIT_WORK(&cc_pdata.cc_work, cc_periodic_save_cvt_data_work);

	setup_timer(&cc_pdata.cc_timer, cc_periodic_save_cvt_data_timer, 0);
	cc_pdata.cc_timer_timeout_jiffies = msecs_to_jiffies(cc_pdata.
						cc_timer_timeout * 1000);
	add_timer(&cc_pdata.cc_timer);

	ret = misc_register(&gd_miscdev);
	if (ret)
		pr_err("cannot register miscdev (err=%d)\n", ret);

	return ret;
}

static int crtcl_cond_remove(struct platform_device *pdev)
{
	misc_deregister(&gd_miscdev);
	del_timer(&cc_pdata.cc_timer);
	cancel_work_sync(&cc_pdata.cc_work);
	unregister_reboot_notifier(&crtcl_cond_reboot_notifier);

	return 0;
}

static const struct of_device_id of_crtcl_cond_match[] = {
	{
		.compatible = "nvidia,tegra186-crtcl_cond",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, of_crtcl_cond_match);

static struct platform_driver crtcl_cond_driver = {
	.probe		= crtcl_cond_probe,
	.remove		= crtcl_cond_remove,
	.driver		= {
		.name	= "crtlcond",
		.owner	= THIS_MODULE,
		.of_match_table = of_crtcl_cond_match,
	},
};

static int __init crtcl_cond_driver_init(void)
{
	return platform_driver_register(&crtcl_cond_driver);
}
late_initcall(crtcl_cond_driver_init);

static int __init crtlcond_init(struct reserved_mem *rmem)
{
	cc_pdata.cvt_mem_address = rmem->base;
	cc_pdata.cvt_mem_size = rmem->size;

	return 0;
}
RESERVEDMEM_OF_DECLARE(tegra_crtlcond, "nvidia,gamedata", crtlcond_init);

