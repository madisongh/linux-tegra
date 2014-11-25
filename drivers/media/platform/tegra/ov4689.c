/*
 * ov4682.c - ov4682 sensor driver
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All Rights Reserved.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <media/ov4689.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "nvc_utilities.h"

#include "ov4689_tables.h"

#define MAX_BUFFER_SIZE 32

/* Frame length: R0x380e~R0x380f */
#define OV4689_NUM_BYTES_FL (2)
/* Coarse time: R0x3500~R0x3502 */
#define OV4689_NUM_BYTES_CT (3)
/* Gain: R0x3508~R0x3509 */
#define OV4689_NUM_BYTES_GAIN (2)
/* Num of regs in override list */
#define OV4689_NUM_BYTES_OVERRIDES (OV4689_NUM_BYTES_FL + \
		OV4689_NUM_BYTES_CT + \
		OV4689_NUM_BYTES_GAIN)

#define OV4689_SUPPORT_SENSORID (1)
#define OV4689_SUPPORT_EEPROM (0)
#define OV4689_SUPPORT_DEBUGFS (1)

struct ov4689_info {
	struct miscdevice		miscdev_info;
	int				mode;
	struct ov4689_power_rail	power;
	struct ov4689_sensordata	sensor_data;
	struct i2c_client		*i2c_client;
	struct ov4689_platform_data	*pdata;
	struct clk			*mclk;
	struct regmap			*regmap;
#if OV4689_SUPPORT_DEBUGFS
	struct dentry			*debugdir;
#endif
	atomic_t			in_use;
#if OV4689_SUPPORT_EEPROM
	struct ov4689_eeprom_data eeprom[ov4689_EEPROM_NUM_BLOCKS];
	u8 eeprom_buf[ov4689_EEPROM_SIZE];
#endif
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void ov4689_get_frame_length_regs(struct ov4689_reg *regs,
				u32 frame_length)
{
	/* 2 registers for FL, i.e., 2-byte FL */
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov4689_get_coarse_time_regs(struct ov4689_reg *regs,
				u32 coarse_time)
{
	/* 3 registers for CT, i.e., 3-byte CT */
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov4689_get_gain_reg(struct ov4689_reg *regs,
				u16 gain)
{
	/* 2 register for gain, i.e., 2-byte gain */
	regs->addr = 0x3508;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = 0x3509;
	(regs + 1)->val = gain & 0xff;
}

static inline int ov4689_read_reg(struct ov4689_info *info,
				u16 addr, u8 *val)
{
	return regmap_read(info->regmap, addr, (unsigned int *) val);
}

static int ov4689_write_reg(struct ov4689_info *info, u16 addr, u8 val)
{
	int err;

	err = regmap_write(info->regmap, addr, val);

	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int ov4689_write_table(struct ov4689_info *info,
				const struct ov4689_reg table[],
				const struct ov4689_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct ov4689_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != OV4689_TABLE_END; next++) {
		if (next->addr == OV4689_TABLE_WAIT_MS) {
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = ov4689_write_reg(info, next->addr, val);
		if (err) {
			pr_err("%s:ov4689_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int ov4689_set_mode(struct ov4689_info *info, struct ov4689_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov4689_reg reg_list[OV4689_NUM_BYTES_OVERRIDES];

	if ((mode->xres == 2688 && mode->yres == 1520) ||
		(mode->xres == 2688 && mode->yres == 1504)) {
		sensor_mode = OV4689_MODE_2688x1520;
		pr_info("ov4689_set_mode 2688x1520\n");
	} else if (mode->xres == 1920 && mode->yres == 1080) {
		sensor_mode = OV4689_MODE_1920x1080;
		pr_info("ov4689_set_mode 1920x1080\n");
	} else {
		pr_err("There is no this resolution no support %dX%d!!",
			mode->xres, mode->yres);
		return 1;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain. */
	ov4689_get_frame_length_regs(reg_list, mode->frame_length);
	ov4689_get_coarse_time_regs(reg_list + OV4689_NUM_BYTES_FL,
				mode->coarse_time);
	ov4689_get_gain_reg(reg_list + OV4689_NUM_BYTES_FL +
				OV4689_NUM_BYTES_CT, mode->gain);

	err = ov4689_write_table(info, mode_table[sensor_mode],
				reg_list, OV4689_NUM_BYTES_OVERRIDES);

	info->mode = sensor_mode;

	pr_info("[ov4689]: stream on.\n");
	return 0;
}

static int ov4689_get_status(struct ov4689_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int ov4689_set_frame_length(struct ov4689_info *info, u32 frame_length,
				bool group_hold)
{
	struct ov4689_reg reg_list[OV4689_NUM_BYTES_FL];
	int i = 0;
	int ret;

	ov4689_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	for (i = 0; i < OV4689_NUM_BYTES_FL; i++) {
		ret = ov4689_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x10);
		if (ret)
			return ret;
		ret = ov4689_write_reg(info, 0x3208, 0xA0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ov4689_set_coarse_time(struct ov4689_info *info, u32 coarse_time,
				bool group_hold)
{
	int ret;

	struct ov4689_reg reg_list[OV4689_NUM_BYTES_CT];
	int i = 0;

	ov4689_get_coarse_time_regs(reg_list, coarse_time);

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	for (i = 0; i < OV4689_NUM_BYTES_CT; i++) {
		ret = ov4689_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x10);
		if (ret)
			return ret;
		ret = ov4689_write_reg(info, 0x3208, 0xA0);
		if (ret)
			return ret;
	}
	return 0;
}

static int ov4689_set_gain(struct ov4689_info *info, u16 gain,
				bool group_hold)
{
	int ret, i;
	struct ov4689_reg reg_list[OV4689_NUM_BYTES_GAIN];

	ov4689_get_gain_reg(reg_list, gain);

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	/* writing 1-byte gain */
	for (i = 0; i < OV4689_NUM_BYTES_GAIN; i++) {
		ret = ov4689_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ov4689_write_reg(info, 0x3208, 0x10);
		if (ret)
			return ret;
		ret = ov4689_write_reg(info, 0x3208, 0xA0);
		if (ret)
			return ret;
	}
	return 0;
}

static int ov4689_set_group_hold(struct ov4689_info *info,

				struct ov4689_grouphold *ae)
{
	int ret;
	int count = 0;
	bool group_hold_enabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		group_hold_enabled = true;

	if (group_hold_enabled) {
		ret = ov4689_write_reg(info, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ov4689_set_gain(info, ae->gain, false);
	if (ae->coarse_time_enable)
		ov4689_set_coarse_time(info, ae->coarse_time, false);
	if (ae->frame_length_enable)
		ov4689_set_frame_length(info, ae->frame_length, false);

	if (group_hold_enabled) {
		ret = ov4689_write_reg(info, 0x3208, 0x10);
		if (ret)
			return ret;
		ret = ov4689_write_reg(info, 0x3208, 0xA0);
		if (ret)
			return ret;
	}

	return 0;
}

#if OV4689_SUPPORT_SENSORID
static int ov4689_get_sensor_id(struct ov4689_info *info)
{
	int ret = 0;
	int i;
	u8 bak = 0;

	pr_info("%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	/* select bank 31 */
	ov4689_write_reg(info, 0x3d84, 31);
	for (i = 0; i < 8; i++) {
		ret |= ov4689_read_reg(info, 0x300A + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = 2;

	return ret;
}
#endif

static void ov4689_mclk_disable(struct ov4689_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ov4689_mclk_enable(struct ov4689_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_info(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

#if OV4689_SUPPORT_EEPROM
static int ov4689_eeprom_device_release(struct ov4689_info *info)
{
	int i;

	for (i = 0; i < ov4689_EEPROM_NUM_BLOCKS; i++) {
		if (info->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(info->eeprom[i].i2c_client);
			info->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int ov4689_eeprom_device_init(struct ov4689_info *info)
{
	char *dev_name = "eeprom_ov4689";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	for (i = 0; i < ov4689_EEPROM_NUM_BLOCKS; i++) {
		info->eeprom[i].adap = i2c_get_adapter(
				info->i2c_client->adapter->nr);
		memset(&info->eeprom[i].brd, 0, sizeof(info->eeprom[i].brd));
		strncpy(info->eeprom[i].brd.type, dev_name,
				sizeof(info->eeprom[i].brd.type));
		info->eeprom[i].brd.addr = ov4689_EEPROM_ADDRESS + i;
		info->eeprom[i].i2c_client = i2c_new_device(
				info->eeprom[i].adap, &info->eeprom[i].brd);

		info->eeprom[i].regmap = devm_regmap_init_i2c(
			info->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(info->eeprom[i].regmap)) {
			err = PTR_ERR(info->eeprom[i].regmap);
			ov4689_eeprom_device_release(info);
			return err;
		}
	}

	return 0;
}

static int ov4689_read_eeprom(struct ov4689_info *info,
				u8 reg, u16 length, u8 *buf)
{
	return regmap_raw_read(info->eeprom[0].regmap, reg, &buf[reg], length);
}

static int ov4689_write_eeprom(struct ov4689_info *info,
				u16 addr, u8 val)
{
	return regmap_write(info->eeprom[addr >> 8].regmap, addr & 0xFF, val);
}
#endif

static long OV4689_IOCTL(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct ov4689_info *info = file->private_data;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(OV4689_IOCTL_SET_POWER):
		if (!info->pdata)
			break;
		if (arg && info->pdata->power_on) {
			err = ov4689_mclk_enable(info);
			if (!err)
				err = info->pdata->power_on(&info->power);
			if (err < 0)
				ov4689_mclk_disable(info);
		}
		if (!arg && info->pdata->power_off) {
			info->pdata->power_off(&info->power);
			ov4689_mclk_disable(info);
		}
		break;
	case _IOC_NR(OV4689_IOCTL_SET_MODE):
	{
		struct ov4689_mode mode;
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct ov4689_mode))) {
			pr_err("%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return ov4689_set_mode(info, &mode);
	}
	case _IOC_NR(OV4689_IOCTL_SET_FRAME_LENGTH):
		return ov4689_set_frame_length(info, (u32)arg, true);
	case _IOC_NR(OV4689_IOCTL_SET_COARSE_TIME):
		return ov4689_set_coarse_time(info, (u32)arg, true);
	case _IOC_NR(OV4689_IOCTL_SET_GAIN):
		return ov4689_set_gain(info, (u16)arg, true);
	case _IOC_NR(OV4689_IOCTL_GET_STATUS):
	{
		u8 status;

		err = ov4689_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			pr_err("%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
#if OV4689_SUPPORT_SENSORID
	case _IOC_NR(OV4689_IOCTL_GET_SENSORDATA):
	{
		err = ov4689_get_sensor_id(info);

		if (err) {
			pr_err("%s:Failed to get fuse id info.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &info->sensor_data,
				sizeof(struct ov4689_sensordata))) {
			pr_info("%s:Failed to copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
#endif
	case _IOC_NR(OV4689_IOCTL_SET_GROUP_HOLD):
	{
		struct ov4689_grouphold grouphold;
		if (copy_from_user(&grouphold, (const void __user *)arg,
			sizeof(struct ov4689_grouphold))) {
			pr_info("%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return ov4689_set_group_hold(info, &grouphold);
	}
#if OV4689_SUPPORT_EEPROM
	/* there is actually one of them really in use
		NVC_IOCTL_GET_EEPROM_DATA
	or
		OV4689_IOCTL_GET_SENSORDATA (legacy?)
	*/
	case _IOC_NR(NVC_IOCTL_GET_EEPROM_DATA):
	{
		ov4689_read_eeprom(info,
			0,
			ov4689_EEPROM_SIZE,
			info->eeprom_buf);

		if (copy_to_user((void __user *)arg,
			info->eeprom_buf, ov4689_EEPROM_SIZE)) {
			dev_err(&info->i2c_client->dev,
				"%s:Failed to copy status to user\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}

	case _IOC_NR(NVC_IOCTL_SET_EEPROM_DATA):
	{
		int i;
		if (copy_from_user(info->eeprom_buf,
			(const void __user *)arg, ov4689_EEPROM_SIZE)) {
			dev_err(&info->i2c_client->dev,
					"%s:Failed to read from user buffer\n",
					__func__);
			return -EFAULT;
		}
		for (i = 0; i < ov4689_EEPROM_SIZE; i++) {
			ov4689_write_eeprom(info,
				i,
				info->eeprom_buf[i]);
			msleep(20);
		}
		return 0;
	}
#endif

	default:
		pr_err("%s:unknown cmd.\n", __func__);
		err = -EINVAL;
	}

	return err;
}

#if OV4689_SUPPORT_DEBUGFS
static int ov4689_debugfs_show(struct seq_file *s, void *unused)
{
	struct ov4689_info *dev = s->private;

	dev_dbg(&dev->i2c_client->dev, "%s: ++\n", __func__);

	return 0;
}

static ssize_t ov4689_debugfs_write(struct file *file, char const __user *buf,
				size_t count, loff_t *offset)
{
	struct ov4689_info *dev =
			((struct seq_file *)file->private_data)->private;
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret = 0;
	char buffer[MAX_BUFFER_SIZE];
	u32 address;
	u32 data;
	u8 readback;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	if (copy_from_user(&buffer, buf, sizeof(buffer)))
		goto debugfs_write_fail;

	if (sscanf(buf, "0x%x 0x%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buf, "0X%x 0X%x", &address, &data) == 2)
		goto set_attr;
	if (sscanf(buf, "%d %d", &address, &data) == 2)
		goto set_attr;

	if (sscanf(buf, "0x%x 0x%x", &address, &data) == 1)
		goto read;
	if (sscanf(buf, "0X%x 0X%x", &address, &data) == 1)
		goto read;
	if (sscanf(buf, "%d %d", &address, &data) == 1)
		goto read;

	dev_err(&i2c_client->dev, "SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	dev_info(&i2c_client->dev,
			"new address = %x, data = %x\n", address, data);
	ret |= ov4689_write_reg(dev, address, data);
read:
	ret |= ov4689_read_reg(dev, address, &readback);
	dev_dbg(&i2c_client->dev,
			"wrote to address 0x%x with value 0x%x\n",
			address, readback);

	if (ret)
		goto debugfs_write_fail;

	return count;

debugfs_write_fail:
	dev_err(&i2c_client->dev,
			"%s: test pattern write failed\n", __func__);
	return -EFAULT;
}

static int ov4689_debugfs_open(struct inode *inode, struct file *file)
{
	struct ov4689_info *dev = inode->i_private;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	return single_open(file, ov4689_debugfs_show, inode->i_private);
}

static const struct file_operations ov4689_debugfs_fops = {
	.open		= ov4689_debugfs_open,
	.read		= seq_read,
	.write		= ov4689_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void ov4689_remove_debugfs(struct ov4689_info *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(dev->debugdir);
	dev->debugdir = NULL;
}

static void ov4689_create_debugfs(struct ov4689_info *dev)
{
	struct dentry *ret;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s\n", __func__);

	dev->debugdir =
		debugfs_create_dir(dev->miscdev_info.this_device->kobj.name,
							NULL);
	if (!dev->debugdir)
		goto remove_debugfs;

	ret = debugfs_create_file("d",
				S_IWUSR | S_IRUGO,
				dev->debugdir, dev,
				&ov4689_debugfs_fops);
	if (!ret)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(&i2c_client->dev, "couldn't create debugfs\n");
	ov4689_remove_debugfs(dev);
}
#endif

static int ov4689_power_on(struct ov4689_power_rail *pw)
{
	struct ov4689_info *info = container_of(pw, struct ov4689_info, power);

	gpio_set_value(info->pdata->reset_gpio, 0);
	usleep_range(10, 20);
	gpio_set_value(info->pdata->reset_gpio, 1);
	usleep_range(1000, 1100);

	return 0;
}

static int ov4689_power_off(struct ov4689_power_rail *pw)
{
	struct ov4689_info *info = container_of(pw, struct ov4689_info, power);

	gpio_set_value(info->pdata->reset_gpio, 0);
	return 0;
}

static int ov4689_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ov4689_info *info;

	info = container_of(miscdev, struct ov4689_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	return 0;
}

static int ov4689_release(struct inode *inode, struct file *file)
{
	struct ov4689_info *info = file->private_data;

	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));

	return 0;
}

static int ov4689_power_put(struct ov4689_power_rail *pw)
{
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;

	return 0;
}

static int ov4689_regulator_get(struct ov4689_info *info,
	struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	reg = regulator_get(&info->i2c_client->dev, vreg_name);
	if (unlikely(IS_ERR(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
			__func__, vreg_name);

	*vreg = reg;
	return err;
}

static int ov4689_power_get(struct ov4689_info *info)
{
	struct ov4689_power_rail *pw = &info->power;
	int err = 0;

	err |= ov4689_regulator_get(info, &pw->avdd, "vana"); /* ananlog 2.7v */
	err |= ov4689_regulator_get(info, &pw->dvdd, "vdig"); /* digital 1.2v */
	err |= ov4689_regulator_get(info, &pw->iovdd, "vif"); /* IO 1.8v */

	return err;
}

static const struct file_operations ov4689_fileops = {
	.owner = THIS_MODULE,
	.open = ov4689_open,
	.unlocked_ioctl = OV4689_IOCTL,
#ifdef CONFIG_COMPAT
	.compat_ioctl = OV4689_IOCTL,
#endif
	.release = ov4689_release,
};

static struct miscdevice ov4689_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov4689",
	.fops = &ov4689_fileops,
};

static struct ov4689_platform_data *ov4689_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ov4689_platform_data *board_info_pdata;

	board_info_pdata = devm_kzalloc(&client->dev, sizeof(*board_info_pdata),
			GFP_KERNEL);
	if (!board_info_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	of_property_read_string(np, "clocks", &board_info_pdata->mclk_name);
	board_info_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);

	board_info_pdata->power_on = ov4689_power_on;
	board_info_pdata->power_off = ov4689_power_off;

	return board_info_pdata;
}

static int ov4689_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ov4689_info *info;
	int err;
	const char *mclk_name;

	pr_info("[ov4689]: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
			sizeof(struct ov4689_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s:Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(info->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(info->regmap));
		return -ENODEV;
	}

	if (client->dev.of_node)
		info->pdata = ov4689_parse_dt(client);
	else
		info->pdata = client->dev.platform_data;

	if (!info->pdata) {
		pr_err("[ov4689]:%s:Unable to get platform data\n", __func__);
		return -EFAULT;
	}

	gpio_request_one(info->pdata->reset_gpio, GPIOF_OUT_INIT_LOW,
			"reset_gpio");

	info->i2c_client = client;
	atomic_set(&info->in_use, 0);
	info->mode = -1;

	mclk_name = info->pdata->mclk_name ?
		    info->pdata->mclk_name : "default_mclk";
	info->mclk = devm_clk_get(&client->dev, mclk_name);
	if (IS_ERR(info->mclk)) {
		dev_err(&client->dev, "%s: unable to get clock %s\n",
			__func__, mclk_name);
		return PTR_ERR(info->mclk);
	}

	ov4689_power_get(info);

	memcpy(&info->miscdev_info,
		&ov4689_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		pr_err("%s:Unable to register misc device!\n", __func__);
		goto ov4689_probe_fail;
	}

	i2c_set_clientdata(client, info);

#if OV4689_SUPPORT_EEPROM
	/* eeprom interface */
	err = ov4689_eeprom_device_init(info);
	if (err) {
		dev_err(&client->dev,
			"Failed to allocate eeprom register map: %d\n", err);
		return err;
	}
#endif

#if OV4689_SUPPORT_DEBUGFS
	/* create debugfs interface */
	ov4689_create_debugfs(info);
#endif
	return 0;

ov4689_probe_fail:
	ov4689_power_put(&info->power);

	return err;
}

static int ov4689_remove(struct i2c_client *client)
{
	struct ov4689_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov4689_device);

	ov4689_power_put(&info->power);

#if OV4689_SUPPORT_DEBUGFS
	ov4689_remove_debugfs(info);
#endif

#if OV4689_SUPPORT_EEPROM
	ov4689_eeprom_device_release(info);
#endif
	return 0;
}

static const struct i2c_device_id ov4689_id[] = {
	{ "ov4689", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ov4689_id);

static struct i2c_driver ov4689_i2c_driver = {
	.driver = {
		.name = "ov4689",
		.owner = THIS_MODULE,
	},
	.probe = ov4689_probe,
	.remove = ov4689_remove,
	.id_table = ov4689_id,
};


static int __init
ov4689_init(void)
{
	pr_info("[ov4689] sensor driver loading\n");
	return i2c_add_driver(&ov4689_i2c_driver);
}

static void __exit
ov4689_exit(void)
{
	i2c_del_driver(&ov4689_i2c_driver);
}

module_init(ov4689_init);
module_exit(ov4689_exit);
