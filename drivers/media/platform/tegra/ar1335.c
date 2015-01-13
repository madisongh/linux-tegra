/*
 * ar1335.c - ar1335 sensor driver
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All Rights Reserved.
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
#include <media/ar1335.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "nvc_utilities.h"

struct ar1335_reg {
	u16 addr;
	u16 val;
};

struct ar1335_info {
	struct miscdevice		miscdev_info;
	int				mode;
	struct ar1335_power_rail	power;
	struct ar1335_sensordata	sensor_data;
	struct i2c_client		*i2c_client;
	struct ar1335_platform_data	*pdata;
	struct clk			*mclk;
	struct regmap			*regmap;
	struct dentry			*debugdir;
	atomic_t			in_use;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

#define AR1335_TABLE_WAIT_MS 0
#define AR1335_TABLE_END 1
#define AR1335_MAX_RETRIES 3
#define AR1335_WAIT_MS 100

#define MAX_BUFFER_SIZE 32
#define AR1335_FRAME_LENGTH_ADDR 0x300A
#define AR1335_COARSE_TIME_ADDR 0x3012
#define AR1335_GAIN_ADDR 0x305E

static struct ar1335_reg mode_4208x3120[] = {
	{0x3042, 0x1004},
	{0x30D2, 0x0120},
	{0x30D4, 0x0000},
	{0x3090, 0x0000},
	{0x30FC, 0x0060},
	{0x30FE, 0x0060},
	{0x31E0, 0x0781},
	{0x3180, 0x9434},
	{0x317C, 0xEFF4},
	{0x30EE, 0x4140},
	{0x3F2C, 0x4428},

	{0x3D00, 0x0446},
	{0x3D02, 0x4C66},
	{0x3D04, 0xFFFF},
	{0x3D06, 0xFFFF},
	{0x3D08, 0x5E40},
	{0x3D0A, 0x1146},
	{0x3D0C, 0x5D41},
	{0x3D0E, 0x1088},
	{0x3D10, 0x8342},
	{0x3D12, 0x00C0},
	{0x3D14, 0x5580},
	{0x3D16, 0x5B83},
	{0x3D18, 0x6084},
	{0x3D1A, 0x5A8D},
	{0x3D1C, 0x00C0},
	{0x3D1E, 0x8342},
	{0x3D20, 0x925A},
	{0x3D22, 0x8664},
	{0x3D24, 0x1030},
	{0x3D26, 0x801C},
	{0x3D28, 0x00A0},
	{0x3D2A, 0x56B0},
	{0x3D2C, 0x5788},
	{0x3D2E, 0x5150},
	{0x3D30, 0x824D},
	{0x3D32, 0x8D58},
	{0x3D34, 0x58D2},
	{0x3D36, 0x438A},
	{0x3D38, 0x4592},
	{0x3D3A, 0x458A},
	{0x3D3C, 0x439D},
	{0x3D3E, 0x51CA},
	{0x3D40, 0x5182},
	{0x3D42, 0x100C},
	{0x3D44, 0x9259},
	{0x3D46, 0x5982},
	{0x3D48, 0x5FF7},
	{0x3D4A, 0x6182},
	{0x3D4C, 0x6283},
	{0x3D4E, 0x4281},
	{0x3D50, 0x10C0},
	{0x3D52, 0x6498},
	{0x3D54, 0x4281},
	{0x3D56, 0x41FF},
	{0x3D58, 0xFFB8},
	{0x3D5A, 0x4081},
	{0x3D5C, 0x4080},
	{0x3D5E, 0x4180},
	{0x3D60, 0x4280},
	{0x3D62, 0x438D},
	{0x3D64, 0x44BA},
	{0x3D66, 0x4488},
	{0x3D68, 0x4380},
	{0x3D6A, 0x4241},
	{0x3D6C, 0x8140},
	{0x3D6E, 0x8240},
	{0x3D70, 0x8041},
	{0x3D72, 0x8042},
	{0x3D74, 0x8043},
	{0x3D76, 0x8D44},
	{0x3D78, 0xBA44},
	{0x3D7A, 0x875E},
	{0x3D7C, 0x4354},
	{0x3D7E, 0x4241},
	{0x3D80, 0x8140},
	{0x3D82, 0x8120},
	{0x3D84, 0x2881},
	{0x3D86, 0x6026},
	{0x3D88, 0x8055},
	{0x3D8A, 0x8070},
	{0x3D8C, 0x8040},
	{0x3D8E, 0x4C81},
	{0x3D90, 0x45C3},
	{0x3D92, 0x4581},
	{0x3D94, 0x4C40},
	{0x3D96, 0x8070},
	{0x3D98, 0x8040},
	{0x3D9A, 0x4C85},
	{0x3D9C, 0x6CA8},
	{0x3D9E, 0x6C8C},
	{0x3DA0, 0x000E},
	{0x3DA2, 0xBE44},
	{0x3DA4, 0x8844},
	{0x3DA6, 0xBC78},
	{0x3DA8, 0x0900},
	{0x3DAA, 0x8904},
	{0x3DAC, 0x8080},
	{0x3DAE, 0x0240},
	{0x3DB0, 0x8609},
	{0x3DB2, 0x008E},
	{0x3DB4, 0x0900},
	{0x3DB6, 0x8002},
	{0x3DB8, 0x4080},
	{0x3DBA, 0x0480},
	{0x3DBC, 0x887C},
	{0x3DBE, 0xAA86},
	{0x3DC0, 0x0900},
	{0x3DC2, 0x877A},
	{0x3DC4, 0x000E},
	{0x3DC6, 0xC379},
	{0x3DC8, 0x4C40},
	{0x3DCA, 0xBF70},
	{0x3DCC, 0x5E40},
	{0x3DCE, 0x114E},
	{0x3DD0, 0x5D41},
	{0x3DD2, 0x5383},
	{0x3DD4, 0x4200},
	{0x3DD6, 0xC055},
	{0x3DD8, 0xA400},
	{0x3DDA, 0xC083},
	{0x3DDC, 0x4288},
	{0x3DDE, 0x6083},
	{0x3DE0, 0x5B80},
	{0x3DE2, 0x5A64},
	{0x3DE4, 0x1030},
	{0x3DE6, 0x801C},
	{0x3DE8, 0x00A5},
	{0x3DEA, 0x5697},
	{0x3DEC, 0x57A5},
	{0x3DEE, 0x5180},
	{0x3DF0, 0x505A},
	{0x3DF2, 0x814D},
	{0x3DF4, 0x8358},
	{0x3DF6, 0x8058},
	{0x3DF8, 0xA943},
	{0x3DFA, 0x8345},
	{0x3DFC, 0xB045},
	{0x3DFE, 0x8343},
	{0x3E00, 0xA351},
	{0x3E02, 0xE251},
	{0x3E04, 0x8C59},
	{0x3E06, 0x8059},
	{0x3E08, 0x8A5F},
	{0x3E0A, 0xEC7C},
	{0x3E0C, 0xCC84},
	{0x3E0E, 0x6182},
	{0x3E10, 0x6283},
	{0x3E12, 0x4283},
	{0x3E14, 0x10CC},
	{0x3E16, 0x6496},
	{0x3E18, 0x4281},
	{0x3E1A, 0x41BB},
	{0x3E1C, 0x4082},
	{0x3E1E, 0x407E},
	{0x3E20, 0xCC41},
	{0x3E22, 0x8042},
	{0x3E24, 0x8043},
	{0x3E26, 0x8300},
	{0x3E28, 0xC088},
	{0x3E2A, 0x44BA},
	{0x3E2C, 0x4488},
	{0x3E2E, 0x00C8},
	{0x3E30, 0x8042},
	{0x3E32, 0x4181},
	{0x3E34, 0x4082},
	{0x3E36, 0x4080},
	{0x3E38, 0x4180},
	{0x3E3A, 0x4280},
	{0x3E3C, 0x4383},
	{0x3E3E, 0x00C0},
	{0x3E40, 0x8844},
	{0x3E42, 0xBA44},
	{0x3E44, 0x8800},
	{0x3E46, 0xC880},
	{0x3E48, 0x4241},
	{0x3E4A, 0x8240},
	{0x3E4C, 0x8140},
	{0x3E4E, 0x8041},
	{0x3E50, 0x8042},
	{0x3E52, 0x8043},
	{0x3E54, 0x8300},
	{0x3E56, 0xC088},
	{0x3E58, 0x44BA},
	{0x3E5A, 0x4488},
	{0x3E5C, 0x00C8},
	{0x3E5E, 0x8042},
	{0x3E60, 0x4181},
	{0x3E62, 0x4082},
	{0x3E64, 0x4080},
	{0x3E66, 0x4180},
	{0x3E68, 0x4280},
	{0x3E6A, 0x4383},
	{0x3E6C, 0x00C0},
	{0x3E6E, 0x8844},
	{0x3E70, 0xBA44},
	{0x3E72, 0x8800},
	{0x3E74, 0xC880},
	{0x3E76, 0x4241},
	{0x3E78, 0x8140},
	{0x3E7A, 0x9F5E},
	{0x3E7C, 0x8A54},
	{0x3E7E, 0x8620},
	{0x3E80, 0x2881},
	{0x3E82, 0x6026},
	{0x3E84, 0x8055},
	{0x3E86, 0x8070},
	{0x3E88, 0x0000},
	{0x3E8A, 0x0000},
	{0x3E8C, 0x0000},
	{0x3E8E, 0x0000},
	{0x3E90, 0x0000},
	{0x3E92, 0x0000},
	{0x3E94, 0x0000},
	{0x3E96, 0x0000},
	{0x3E98, 0x0000},
	{0x3E9A, 0x0000},
	{0x3E9C, 0x0000},
	{0x3E9E, 0x0000},
	{0x3EA0, 0x0000},
	{0x3EA2, 0x0000},
	{0x3EA4, 0x0000},
	{0x3EA6, 0x0000},
	{0x3EA8, 0x0000},
	{0x3EAA, 0x0000},
	{0x3EAC, 0x0000},
	{0x3EAE, 0x0000},
	{0x3EB0, 0x0000},
	{0x3EB2, 0x0000},
	{0x3EB4, 0x0000},

	{0x3EB6, 0x004D},
	{0x3EB8, 0x0F0C},
	{0x3EBC, 0xAA06},
	{0x3EC0, 0x1E02},
	{0x3EC2, 0x7700},
	{0x3EC4, 0x1308},
	{0x3EC6, 0xEA44},
	{0x3EC8, 0x0F0F},
	{0x3ECA, 0x0F4A},
	{0x3ECC, 0x0706},
	{0x3ECE, 0x443B},
	{0x3ED0, 0x12F0},
	{0x3ED2, 0x0039},
	{0x3ED4, 0x862F},
	{0x3ED6, 0x4080},
	{0x3ED8, 0x0523},
	{0x3EDA, 0xF896},
	{0x3EDC, 0x5096},
	{0x3EDE, 0x5005},
	{0x316A, 0x8200},
	{0x316E, 0x8200},
	{0x316C, 0x8200},
	{0x3EF0, 0x4D4D},
	{0x3EF2, 0x0101},
	{0x3EF6, 0x0307},
	{0x3EFA, 0x0F0F},
	{0x3EFC, 0x0F0F},
	{0x3EFE, 0x0F0F},

	{0x31B0, 0x005C},
	{0x31B2, 0x002D},
	{0x31B4, 0x2412},
	{0x31B6, 0x142A},
	{0x31B8, 0x2413},
	{0x31BA, 0x1C70},
	{0x31BC, 0x868B},
	{0x31AE, 0x0204},

	{0x0300, 0x0005},
	{0x0302, 0x0001},
	{0x0304, 0x0101},
	{0x0306, 0x2E2E},
	{0x0308, 0x000A},
	{0x030A, 0x0001},
	{0x0112, 0x0A0A},
	{0x3016, 0x0101},
	{AR1335_TABLE_WAIT_MS, AR1335_WAIT_MS},

	{0x0344, 0x0010},
	{0x0348, 0x107F},
	{0x0346, 0x0010},
	{0x034A, 0x0C3F},
	{0x034C, 0x1070},
	{0x034E, 0x0C30},
	{0x3040, 0x0041},
	{0x0112, 0x0A0A},
	{0x0112, 0x0A0A},
	{0x3172, 0x0206},
	{0x317A, 0x416E},
	{0x3F3C, 0x0003},
	{0x0342, 0x1240},
	{0x0340, 0x0C4E},
	{0x0202, 0x0C44},

	{0x31E0, 0x0781},
	{0x3F00, 0x004F},
	{0x3F02, 0x0125},
	{0x3F04, 0x0020},
	{0x3F06, 0x0040},
	{0x3F08, 0x0070},
	{0x3F0A, 0x0101},
	{0x3F0C, 0x0302},
	{0x3F1E, 0x0022},
	{0x3F1A, 0x01FF},
	{0x3F14, 0x0101},
	{0x3F44, 0x0707},
	{0x3F18, 0x011E},
	{0x3F12, 0x0303},
	{0x3F42, 0x1511},
	{0x3F16, 0x011E},
	{0x3F40, 0x1511},

	{0x3F3C, 0x0003},
	{0x301A, 0x021C},
	{AR1335_TABLE_END, 0x00}
};

enum {
	AR1335_MODE_4208X3120,
};

static struct ar1335_reg *mode_table[] = {
	[AR1335_MODE_4208X3120] = mode_4208x3120,
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void
ar1335_get_frame_length_regs(struct ar1335_reg *regs, u32 frame_length)
{
	regs->addr = AR1335_FRAME_LENGTH_ADDR;
	regs->val = frame_length & 0xffff;
}

static inline void
ar1335_get_coarse_time_regs(struct ar1335_reg *regs, u32 coarse_time)
{
	regs->addr = AR1335_COARSE_TIME_ADDR;
	regs->val = coarse_time & 0xffff;
}

static inline void
ar1335_get_gain_reg(struct ar1335_reg *regs, u16 gain)
{
	regs->addr = AR1335_GAIN_ADDR;
	regs->val = gain;
}

static inline int
ar1335_read_reg(struct ar1335_info *info, u16 addr, u16 *val)
{
	return regmap_read(info->regmap, addr, (unsigned int *) val);
}

static int
ar1335_write_reg(struct ar1335_info *info, u16 addr, u16 val)
{
	int err;

	err = regmap_write(info->regmap, addr, val);

	if (err)
		pr_err("%s:i2c write failed, %x = %x\n",
			__func__, addr, val);

	return err;
}

static int
ar1335_write_table(struct ar1335_info *info,
				 const struct ar1335_reg table[],
				 const struct ar1335_reg override_list[],
				 int num_override_regs)
{
	int err;
	const struct ar1335_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != AR1335_TABLE_END; next++) {
		if (next->addr == AR1335_TABLE_WAIT_MS) {
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

		err = ar1335_write_reg(info, next->addr, val);
		if (err) {
			pr_err("%s:ar1335_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int ar1335_get_flash_cap(struct ar1335_info *info)
{
	struct ar1335_flash_control *fctl;

	dev_dbg(&info->i2c_client->dev, "%s: %p\n", __func__, info->pdata);
	if (info->pdata) {
		fctl = &info->pdata->flash_cap;
		dev_dbg(&info->i2c_client->dev,
			"edg: %x, st: %x, rpt: %x, dl: %x\n",
			fctl->edge_trig_en,
			fctl->start_edge,
			fctl->repeat,
			fctl->delay_frm);

		if (fctl->enable)
			return 0;
	}
	return -ENODEV;
}

static inline int ar1335_set_flash_control(
	struct ar1335_info *info, struct ar1335_flash_control *fc)
{
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	return ar1335_write_reg(info, 0x0802, 0x01);
}

static int
ar1335_set_mode(struct ar1335_info *info, struct ar1335_mode *mode)
{
	int sensor_mode;
	int err;
	struct ar1335_reg reg_list[8];

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			 __func__, mode->xres, mode->yres, mode->frame_length,
			 mode->coarse_time, mode->gain);

	if (mode->xres == 4208 && mode->yres == 3120) {
		sensor_mode = AR1335_MODE_4208X3120;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
			 __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	ar1335_get_frame_length_regs(reg_list, mode->frame_length);
	ar1335_get_coarse_time_regs(reg_list + 1, mode->coarse_time);
	ar1335_get_gain_reg(reg_list + 2, mode->gain);

	err = ar1335_write_table(info,
				mode_table[sensor_mode],
				reg_list, 3);
	if (err)
		return err;
	info->mode = sensor_mode;
	pr_info("[AR1335]: stream on.\n");
	return 0;
}

static int
ar1335_get_status(struct ar1335_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int
ar1335_set_frame_length(struct ar1335_info *info, u32 frame_length,
						 bool group_hold)
{
	struct ar1335_reg reg_list[2];
	int i = 0;
	int ret;

	ar1335_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 1; i++) {
		ret = ar1335_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x0104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int
ar1335_set_coarse_time(struct ar1335_info *info, u32 coarse_time,
						 bool group_hold)
{
	int ret;

	struct ar1335_reg reg_list[2];
	int i = 0;

	ar1335_get_coarse_time_regs(reg_list, coarse_time);

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 1; i++) {
		ret = ar1335_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
ar1335_set_gain(struct ar1335_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct ar1335_reg reg_list;

	ar1335_get_gain_reg(&reg_list, gain);

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x104, 0x1);
		if (ret)
			return ret;
	}

	ret = ar1335_write_reg(info, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

	if (group_hold) {
		ret = ar1335_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
ar1335_set_group_hold(struct ar1335_info *info, struct ar1335_ae *ae)
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
		ret = ar1335_write_reg(info, 0x104, 0x1);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ar1335_set_gain(info, ae->gain, true);
	if (ae->coarse_time_enable)
		ar1335_set_coarse_time(info, ae->coarse_time, true);
	if (ae->frame_length_enable)
		ar1335_set_frame_length(info, ae->frame_length, true);

	if (group_hold_enabled) {
		ret = ar1335_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ar1335_get_sensor_id(struct ar1335_info *info)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	/* Note 1: If the sensor does not have power at this point
	Need to supply the power, e.g. by calling power on function */

	/*ret |= ar1335_write_reg(info, 0x3B02, 0x00);
	ret |= ar1335_write_reg(info, 0x3B00, 0x01);
	for (i = 0; i < 9; i++) {
		ret |= ar1335_read_reg(info, 0x3B24 + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = i;*/

	/* Note 2: Need to clean up any action carried out in Note 1 */

	return ret;
}

static void ar1335_mclk_disable(struct ar1335_info *info)
{
	dev_dbg(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int ar1335_mclk_enable(struct ar1335_info *info)
{
	int err;
	unsigned long mclk_init_rate = 24000000;

	dev_dbg(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static long
ar1335_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct ar1335_info *info = file->private_data;

	switch (cmd) {
	case AR1335_IOCTL_SET_POWER:
		if (!info->pdata)
			break;
		if (arg && info->pdata->power_on) {
			err = ar1335_mclk_enable(info);
			if (!err)
				err = info->pdata->power_on(&info->power);
			if (err < 0)
				ar1335_mclk_disable(info);
		}
		if (!arg && info->pdata->power_off) {
			info->pdata->power_off(&info->power);
			ar1335_mclk_disable(info);
		}
		break;
	case AR1335_IOCTL_SET_MODE:
	{
		struct ar1335_mode mode;
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct ar1335_mode))) {
			pr_err("%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return ar1335_set_mode(info, &mode);
	}
	case AR1335_IOCTL_SET_FRAME_LENGTH:
		return ar1335_set_frame_length(info, (u32)arg, true);
	case AR1335_IOCTL_SET_COARSE_TIME:
		return ar1335_set_coarse_time(info, (u32)arg, true);
	case AR1335_IOCTL_SET_GAIN:
		return ar1335_set_gain(info, (u16)arg, true);
	case AR1335_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ar1335_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			pr_err("%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case AR1335_IOCTL_GET_SENSORDATA:
	{
		err = ar1335_get_sensor_id(info);

		if (err) {
			pr_err("%s:Failed to get fuse id info.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &info->sensor_data,
				sizeof(struct ar1335_sensordata))) {
			pr_info("%s:Failed to copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case AR1335_IOCTL_SET_GROUP_HOLD:
	{
		struct ar1335_ae ae;
		if (copy_from_user(&ae, (const void __user *)arg,
			sizeof(struct ar1335_ae))) {
			pr_info("%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return ar1335_set_group_hold(info, &ae);
	}
	case AR1335_IOCTL_SET_FLASH_MODE:
	{
		struct ar1335_flash_control values;

		dev_dbg(&info->i2c_client->dev,
			"AR1335_IOCTL_SET_FLASH_MODE\n");
		if (copy_from_user(&values,
			(const void __user *)arg,
			sizeof(struct ar1335_flash_control))) {
			err = -EFAULT;
			break;
		}
		err = ar1335_set_flash_control(info, &values);
		break;
	}
	case AR1335_IOCTL_GET_FLASH_CAP:
		err = ar1335_get_flash_cap(info);
		break;
	default:
		pr_err("%s:unknown cmd.\n", __func__);
		err = -EINVAL;
	}

	return err;
}

static int ar1335_debugfs_show(struct seq_file *s, void *unused)
{
	struct ar1335_info *dev = s->private;

	dev_dbg(&dev->i2c_client->dev, "%s: ++\n", __func__);

	return 0;
}

static ssize_t ar1335_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset)
{
	struct ar1335_info *dev =
			((struct seq_file *)file->private_data)->private;
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret = 0;
	char buffer[MAX_BUFFER_SIZE];
	u32 address;
	u32 data;
	u16 readback;

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
	ret |= ar1335_write_reg(dev, address, data);
read:
	ret |= ar1335_read_reg(dev, address, &readback);
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

static int ar1335_debugfs_open(struct inode *inode, struct file *file)
{
	struct ar1335_info *dev = inode->i_private;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	return single_open(file, ar1335_debugfs_show, inode->i_private);
}

static const struct file_operations ar1335_debugfs_fops = {
	.open		= ar1335_debugfs_open,
	.read		= seq_read,
	.write		= ar1335_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void ar1335_remove_debugfs(struct ar1335_info *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(dev->debugdir);
	dev->debugdir = NULL;
}

static void ar1335_create_debugfs(struct ar1335_info *dev)
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
				&ar1335_debugfs_fops);
	if (!ret)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(&i2c_client->dev, "couldn't create debugfs\n");
	ar1335_remove_debugfs(dev);
}

static int ar1335_power_on(struct ar1335_power_rail *pw)
{
	int err;
	struct ar1335_info *info = container_of(pw, struct ar1335_info, power);

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	gpio_set_value(info->pdata->reset_gpio, 0);
	gpio_set_value(info->pdata->af_gpio, 1);
	gpio_set_value(info->pdata->cam1_gpio, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto ar1335_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto ar1335_iovdd_fail;

	usleep_range(1, 2);
	gpio_set_value(info->pdata->reset_gpio, 1);
	gpio_set_value(info->pdata->cam1_gpio, 1);

	usleep_range(300, 310);

	return 1;


ar1335_iovdd_fail:
	regulator_disable(pw->avdd);

ar1335_avdd_fail:
	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int ar1335_power_off(struct ar1335_power_rail *pw)
{
	struct ar1335_info *info = container_of(pw, struct ar1335_info, power);

	if (unlikely(WARN_ON(!pw || !pw->iovdd || !pw->avdd)))
		return -EFAULT;

	usleep_range(1, 2);
	gpio_set_value(info->pdata->cam1_gpio, 0);
	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static int
ar1335_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct ar1335_info *info;

	info = container_of(miscdev, struct ar1335_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	return 0;
}

static int
ar1335_release(struct inode *inode, struct file *file)
{
	struct ar1335_info *info = file->private_data;

	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}

static int ar1335_power_put(struct ar1335_power_rail *pw)
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

static int ar1335_regulator_get(struct ar1335_info *info,
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

static int ar1335_power_get(struct ar1335_info *info)
{
	struct ar1335_power_rail *pw = &info->power;
	int err = 0;

	err |= ar1335_regulator_get(info, &pw->avdd, "vana"); /* ananlog 2.7v */
	err |= ar1335_regulator_get(info, &pw->dvdd, "vdig_csi"); /* dig 1.2v */
	err |= ar1335_regulator_get(info, &pw->iovdd, "vif"); /* IO 1.8v */

	return err;
}

static const struct file_operations ar1335_fileops = {
	.owner = THIS_MODULE,
	.open = ar1335_open,
	.unlocked_ioctl = ar1335_ioctl,
	.release = ar1335_release,
};

static struct miscdevice ar1335_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ar1335",
	.fops = &ar1335_fileops,
};

static struct of_device_id ar1335_of_match[] = {
	{ .compatible = "nvidia,ar1335", },
	{ },
};

MODULE_DEVICE_TABLE(of, ar1335_of_match);

static struct ar1335_platform_data *ar1335_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct ar1335_platform_data *board_info_pdata;
	const struct of_device_id *match;

	match = of_match_device(ar1335_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_info_pdata = devm_kzalloc(&client->dev, sizeof(*board_info_pdata),
			GFP_KERNEL);
	if (!board_info_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	board_info_pdata->cam1_gpio = of_get_named_gpio(np, "cam1-gpios", 0);
	board_info_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	board_info_pdata->af_gpio = of_get_named_gpio(np, "af-gpios", 0);

	board_info_pdata->power_on = ar1335_power_on;
	board_info_pdata->power_off = ar1335_power_off;

	return board_info_pdata;
}

static int
ar1335_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ar1335_info *info;
	int err;
	const char *mclk_name;

	pr_err("[AR1335]: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
			sizeof(struct ar1335_info), GFP_KERNEL);
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
		info->pdata = ar1335_parse_dt(client);
	else
		info->pdata = client->dev.platform_data;

	if (!info->pdata) {
		pr_err("[AR1335]:%s:Unable to get platform data\n", __func__);
		return -EFAULT;
	}

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

	ar1335_power_get(info);

	memcpy(&info->miscdev_info,
		&ar1335_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		pr_err("%s:Unable to register misc device!\n", __func__);
		goto ar1335_probe_fail;
	}

	i2c_set_clientdata(client, info);
	/* create debugfs interface */
	ar1335_create_debugfs(info);
	pr_err("[AR1335]: end of probing sensor.\n");
	return 0;

ar1335_probe_fail:
	ar1335_power_put(&info->power);

	return err;
}

static int
ar1335_remove(struct i2c_client *client)
{
	struct ar1335_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ar1335_device);

	ar1335_power_put(&info->power);

	ar1335_remove_debugfs(info);
	return 0;
}

static const struct i2c_device_id ar1335_id[] = {
	{ "ar1335", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar1335_id);

static struct i2c_driver ar1335_i2c_driver = {
	.driver = {
		.name = "ar1335",
		.owner = THIS_MODULE,
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
	.id_table = ar1335_id,
};

static int __init ar1335_init(void)
{
	pr_info("[AR1335] sensor driver loading\n");
	return i2c_add_driver(&ar1335_i2c_driver);
}

static void __exit ar1335_exit(void)
{
	i2c_del_driver(&ar1335_i2c_driver);
}

module_init(ar1335_init);
module_exit(ar1335_exit);
