/*
 * imx185.c - imx185 sensor driver
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
#include <media/imx185.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <mach/io_dpd.h>

#include "nvc_utilities.h"

struct imx185_reg {
	u16 addr;
	u8 val;
};

struct imx185_info {
	struct miscdevice		miscdev_info;
	int				mode;
	struct imx185_power_rail	power;
	struct imx185_sensordata	sensor_data;
	struct i2c_client		*i2c_client;
	struct imx185_platform_data	*pdata;
	struct clk			*mclk;
	struct regmap			*regmap;
	struct mutex			imx185_camera_lock;
	struct dentry			*debugdir;
	atomic_t			in_use;
	u32				frame_length;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

#define IMX185_TABLE_WAIT_MS 0
#define IMX185_TABLE_END 1
#define IMX185_MAX_RETRIES 3
#define IMX185_WAIT_MS 3

#define MAX_BUFFER_SIZE 32
#define IMX185_FRAME_LENGTH_ADDR_MSB 0x3019
#define IMX185_FRAME_LENGTH_ADDR_LSB 0x3018
#define IMX185_COARSE_TIME_ADDR_MSB 0x3021
#define IMX185_COARSE_TIME_ADDR_LSB 0x3020
#define IMX185_GAIN_ADDR 0x3014
#define IMX185_REG_HOLD 0x3001

static struct imx185_reg mode_1920x1200[] = {
	/* standby */
	{0x3000, 0x01},
	{IMX185_TABLE_WAIT_MS, IMX185_WAIT_MS},

	{0x3002, 0x01}, /* master mode operation stop */
	{0x3005, 0x01}, /* AD conversion 12 bits */
	{0x3006, 0x00}, /* All-pixel scan mode */
	{0x3007, 0x00}, /* WUXGA mode 1920x1200 */
	{0x3009, 0x01}, /* frame rate 30fps */
	{0x300A, 0xF0}, /* Black level */
	{0x300B, 0x00},
	{0x300C, 0x00},
	{0x300F, 0x01},

	{0x3010, 0x39},
	{0x3012, 0x50},
	{0x3084, 0x00},
	{0x3065, 0x20},
	{0x3086, 0x01},
	{0x30CF, 0xD1},
	{0x30D0, 0x1B},
	{0x30D2, 0x5F},
	{0x30D3, 0x00},
	{0x3018, 0x28}, /* VMAX=1320 */
	{0x3019, 0x05},
	{0x301A, 0x00},
	{0x301B, 0x53}, /* HMAX=1875 */
	{0x301C, 0x07},
	{0x301D, 0x08},
	{0x301E, 0x02},
	{0x3048, 0x33},
	{0x3044, 0xE1}, /* MIPI output mode */
	{0x305C, 0x20},
	{0x305D, 0x00},
	{0x305E, 0x18},
	{0x305F, 0x00},
	{0x3063, 0x74}, /* INCLK */

	{0x311D, 0x0A},
	{0x3123, 0x0F},
	{0x3147, 0x87},
	{0x31E1, 0x9E},
	{0x31E2, 0x01},
	{0x31E5, 0x05},
	{0x31E6, 0x05},
	{0x31E7, 0x3A},
	{0x31E8, 0x3A},

	{0x3203, 0xC8},
	{0x3207, 0x54},
	{0x3213, 0x16},
	{0x3215, 0xF6},
	{0x321A, 0x14},
	{0x321B, 0x51},
	{0x3229, 0xE7},
	{0x322A, 0xF0},
	{0x322B, 0x10},
	{0x3231, 0xE7},
	{0x3232, 0xF0},
	{0x3233, 0x10},
	{0x323C, 0xE8},
	{0x323D, 0x70},
	{0x3243, 0x08},
	{0x3244, 0xE1},
	{0x3245, 0x10},
	{0x3247, 0xE7},
	{0x3248, 0x60},
	{0x3249, 0x1E},
	{0x324B, 0x00},
	{0x324C, 0x41},
	{0x3250, 0x30},
	{0x3251, 0x0A},
	{0x3252, 0xFF},
	{0x3253, 0xFF},
	{0x3254, 0xFF},
	{0x3255, 0x02},
	{0x3257, 0xF0},
	{0x325D, 0xEE},
	{0x325E, 0xA0},
	{0x3260, 0x03},
	{0x3261, 0x61},
	{0x3266, 0x30},
	{0x3267, 0x05},
	{0x3275, 0xE7},
	{0x3281, 0xEA},
	{0x3282, 0x70},
	{0x3285, 0xFF},
	{0x328A, 0xF0},
	{0x328D, 0xB6},
	{0x328E, 0x40},
	{0x3290, 0x42},
	{0x3291, 0x51},
	{0x3292, 0x1E},
	{0x3294, 0xC4},
	{0x3295, 0x20},
	{0x3297, 0x50},
	{0x3298, 0x31},
	{0x3299, 0x1F},
	{0x329B, 0xC0},
	{0x329C, 0x60},
	{0x329E, 0x4C},
	{0x329F, 0x71},
	{0x32A0, 0x1F},
	{0x32A2, 0xB6},
	{0x32A3, 0xC0},
	{0x32A4, 0x0B},
	{0x32A9, 0x24},
	{0x32AA, 0x41},
	{0x32B0, 0x25},
	{0x32B1, 0x51},
	{0x32B7, 0x1C},
	{0x32B8, 0xC1},
	{0x32B9, 0x12},
	{0x32BE, 0x1D},
	{0x32BF, 0xD1},
	{0x32C0, 0x12},
	{0x32C2, 0xA8},
	{0x32C3, 0xC0},
	{0x32C4, 0x0A},
	{0x32C5, 0x1E},
	{0x32C6, 0x21},
	{0x32C9, 0xB0},
	{0x32CA, 0x40},
	{0x32CC, 0x26},
	{0x32CD, 0xA1},
	{0x32D0, 0xB6},
	{0x32D1, 0xC0},
	{0x32D2, 0x0B},
	{0x32D4, 0xE2},
	{0x32D5, 0x40},
	{0x32D8, 0x4E},
	{0x32D9, 0xA1},
	{0x32EC, 0xF0},

	{0x332D, 0x20},
	{0x3303, 0x00}, /* REPETITION */
	{0x3305, 0x03}, /* MIPI 4 Lanes */
	{0x3314, 0x0E},
	{0x3315, 0x01},
	{0x3316, 0x04},
	{0x3317, 0x04},
	{0x3318, 0xC1}, /* V effective pixel=1217 */
	{0x3319, 0x04},
	{0x332C, 0x40},
	{0x332D, 0x20},
	{0x332E, 0x03},
	{0x333E, 0x0C}, /* CSI_DT_FMT RAM 12 */
	{0x333F, 0x0C},
	{0x3340, 0x03}, /* CSI Lane 4 */
	{0x3341, 0x20}, /* INCLK = 37.125MHz */
	{0x3342, 0x25},
	{0x334E, 0xB4},
	{0x334F, 0x01},
	{0x3343, 0x68},
	{0x3344, 0x20},
	{0x3345, 0x40},
	{0x3346, 0x28},
	{0x3347, 0x20},
	{0x3348, 0x18},
	{0x3349, 0x78},
	{0x334A, 0x28},
	{0x338C, 0x0C},
	{0x338D, 0x0C},
	{0x33D8, 0x03},

	/* Exit standby */
	{0x3000, 0x00},
	{IMX185_TABLE_WAIT_MS, 25},
	/* Master mode start */
	{0x3002, 0x00},
	{0x3049, 0x0A}, /* XVS VSYNC and XHS HSYNC */
	{IMX185_TABLE_WAIT_MS, 200},
	{IMX185_TABLE_END, 0x00}
};

static struct imx185_reg mode_1920x1080[] = {
	/* standby */
	{0x3000, 0x01},
	{IMX185_TABLE_WAIT_MS, IMX185_WAIT_MS},

	{0x3002, 0x01}, /* master mode operation stop */
	{0x3005, 0x01}, /* AD conversion 12 bits */
	{0x3006, 0x00}, /* All-pixel scan mode */
	{0x3007, 0x50}, /* 1080p-HD cropping mode 1920x1080 */
	{0x3009, 0x02}, /* frame rate 30fps */
#if 0
	/* IMX185 PG1 */
	{0x300A, 0x00}, /* Black level */
	{0x300E, 0x00},
	{0x3089, 0x00},
	{0x308C, 0x13},
#else
	{0x300A, 0xf0}, /* Black level */
#endif
	{0x300B, 0x00},
	{0x300C, 0x00},
	{0x300F, 0x01},

	{0x3010, 0x39},
	{0x3012, 0x50},
	{0x3084, 0x00},
	{0x3065, 0x20},
	{0x3086, 0x01},
	{0x30CF, 0xD1},
	{0x30D0, 0x1B},
	{0x30D2, 0x5F},
	{0x30D3, 0x00},
	{0x3018, 0x65}, /* VMAX=1125 */
	{0x3019, 0x04},
	{0x301A, 0x00},
	{0x301B, 0x98}, /* HMAX=2200 */
	{0x301C, 0x08},
	{0x301D, 0x08},
	{0x301E, 0x02},
	{0x3038, 0x08}, /* WINPV=8 */
	{0x3039, 0x00},
	{0x303a, 0x40}, /* WINWV=1088 */
	{0x303b, 0x04},
	{0x303c, 0x0c}, /* WINPH=12 */
	{0x303d, 0x00},
	{0x303e, 0x7c}, /* WINWH=1916 */
	{0x303f, 0x07},
	{0x3048, 0x33},
	{0x3044, 0xE1}, /* MIPI output mode */
	{0x305C, 0x20},
	{0x305D, 0x00},
	{0x305E, 0x18},
	{0x305F, 0x00},
	{0x3063, 0x74}, /* INCLK */

	{0x311D, 0x0A},
	{0x3123, 0x0F},
	{0x3147, 0x87},
	{0x31E1, 0x9E},
	{0x31E2, 0x01},
	{0x31E5, 0x05},
	{0x31E6, 0x05},
	{0x31E7, 0x3A},
	{0x31E8, 0x3A},

	{0x3203, 0xC8},
	{0x3207, 0x54},
	{0x3213, 0x16},
	{0x3215, 0xF6},
	{0x321A, 0x14},
	{0x321B, 0x51},
	{0x3229, 0xE7},
	{0x322A, 0xF0},
	{0x322B, 0x10},
	{0x3231, 0xE7},
	{0x3232, 0xF0},
	{0x3233, 0x10},
	{0x323C, 0xE8},
	{0x323D, 0x70},
	{0x3243, 0x08},
	{0x3244, 0xE1},
	{0x3245, 0x10},
	{0x3247, 0xE7},
	{0x3248, 0x60},
	{0x3249, 0x1E},
	{0x324B, 0x00},
	{0x324C, 0x41},
	{0x3250, 0x30},
	{0x3251, 0x0A},
	{0x3252, 0xFF},
	{0x3253, 0xFF},
	{0x3254, 0xFF},
	{0x3255, 0x02},
	{0x3257, 0xF0},
	{0x325D, 0xEE},
	{0x325E, 0xA0},
	{0x3260, 0x03},
	{0x3261, 0x61},
	{0x3266, 0x30},
	{0x3267, 0x05},
	{0x3275, 0xE7},
	{0x3281, 0xEA},
	{0x3282, 0x70},
	{0x3285, 0xFF},
	{0x328A, 0xF0},
	{0x328D, 0xB6},
	{0x328E, 0x40},
	{0x3290, 0x42},
	{0x3291, 0x51},
	{0x3292, 0x1E},
	{0x3294, 0xC4},
	{0x3295, 0x20},
	{0x3297, 0x50},
	{0x3298, 0x31},
	{0x3299, 0x1F},
	{0x329B, 0xC0},
	{0x329C, 0x60},
	{0x329E, 0x4C},
	{0x329F, 0x71},
	{0x32A0, 0x1F},
	{0x32A2, 0xB6},
	{0x32A3, 0xC0},
	{0x32A4, 0x0B},
	{0x32A9, 0x24},
	{0x32AA, 0x41},
	{0x32B0, 0x25},
	{0x32B1, 0x51},
	{0x32B7, 0x1C},
	{0x32B8, 0xC1},
	{0x32B9, 0x12},
	{0x32BE, 0x1D},
	{0x32BF, 0xD1},
	{0x32C0, 0x12},
	{0x32C2, 0xA8},
	{0x32C3, 0xC0},
	{0x32C4, 0x0A},
	{0x32C5, 0x1E},
	{0x32C6, 0x21},
	{0x32C9, 0xB0},
	{0x32CA, 0x40},
	{0x32CC, 0x26},
	{0x32CD, 0xA1},
	{0x32D0, 0xB6},
	{0x32D1, 0xC0},
	{0x32D2, 0x0B},
	{0x32D4, 0xE2},
	{0x32D5, 0x40},
	{0x32D8, 0x4E},
	{0x32D9, 0xA1},
	{0x32EC, 0xF0},

	{0x332D, 0x20},
	{0x334E, 0xB4}, /*INCK_FREQ */
	{0x334F, 0x01},
	{0x3303, 0x10}, /* REPETITION */
	{0x3305, 0x03}, /* MIPI 4 Lanes */
	{0x3314, 0x08},
	{0x3315, 0x01},
	{0x3316, 0x04},
	{0x3317, 0x04},
	{0x3318, 0x38}, /* V effective pixel=1080 */
	{0x3319, 0x04},
	{0x332C, 0x30},
	{0x332D, 0x20},
	{0x332E, 0x03},
	{0x333E, 0x0C}, /* CSI_DT_FMT RAM 10 */
	{0x333F, 0x0C},
	{0x3340, 0x03}, /* CSI 4Lane */
	{0x3341, 0x20}, /* INCLK = 37.125MHz */
	{0x3342, 0x25},
	/* Global Timing */
	{0x3343, 0x58}, /* TCLK_POST */
	{0x3344, 0x10}, /* THS_PREPARE */
	{0x3345, 0x30}, /* THS_ZERO_MIN */
	{0x3346, 0x18}, /* THS_TRAIL */
	{0x3347, 0x10}, /* TCLK_TRAIL_MIN */
	{0x3348, 0x10}, /* TCLK_PREPARE */
	{0x3349, 0x48}, /* TCLK_ZERO */
	{0x334A, 0x28}, /* TLP_X */
	{0x338C, 0x0C}, /* ADC 12bit */
	{0x338D, 0x0C},
	{0x33D8, 0x03},

	/* Exit standby */
	{0x3000, 0x00},
	{IMX185_TABLE_WAIT_MS, 25},
	/* Master mode start */
	{0x3002, 0x00},
	{0x3049, 0x0A}, /* XVS VSYNC and XHS HSYNC */
	{IMX185_TABLE_WAIT_MS, 210},
	{IMX185_TABLE_END, 0x00}
};

enum {
	IMX185_MODE_1920X1200,
	IMX185_MODE_1920X1080,
};

static struct imx185_reg *mode_table[] = {
	[IMX185_MODE_1920X1200] = mode_1920x1200,
	[IMX185_MODE_1920X1080] = mode_1920x1080,
};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000+500);
}

static inline void
imx185_get_frame_length_regs(struct imx185_reg *regs, u32 frame_length)
{
	/* vertical span setting */
	regs->addr = IMX185_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX185_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void
imx185_get_coarse_time_regs(struct imx185_reg *regs, u32 coarse_time)
{
	/* storage time adjustment */
	regs->addr = IMX185_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX185_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void
imx185_get_gain_reg(struct imx185_reg *regs, u16 gain)
{
	regs->addr = IMX185_GAIN_ADDR;
	regs->val = gain;
}

static int imx185_read_reg(struct imx185_info *info, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];
	struct i2c_client *client = info->i2c_client;
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2];
	return 0;
}

	static int
imx185_write_reg(struct imx185_info *info, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	struct i2c_client *client =	info->i2c_client;
	if (!client->adapter)
		return -ENODEV;

	pr_debug("imx185_write_reg: %x,%x\n", addr, val);

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("%s:i2c write failed, client->addr = 0x%x %x = %x\n",
			__func__, client->addr, addr, val);

	return err;
}
static int
imx185_write_table(struct imx185_info *info,
				 const struct imx185_reg table[],
				 const struct imx185_reg override_list[],
				 int num_override_regs)
{
	int err;
	const struct imx185_reg *next;
	int i;
	u16 val;

	for (next = table; next->addr != IMX185_TABLE_END; next++) {
		if (next->addr == IMX185_TABLE_WAIT_MS) {
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

		err = imx185_write_reg(info, next->addr, val);
		if (err) {
			pr_err("%s:imx185_write_table:%d", __func__, err);
			return err;
		}
	}
	return 0;
}

static int imx185_power_on(struct imx185_power_rail *pw)
{
	int err;
	struct imx185_info *info = container_of(pw, struct imx185_info, power);

	if (info->pdata->cam1_gpio > 0)
		gpio_set_value(info->pdata->cam1_gpio, 0);
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd);
	if (err)
		goto imx185_avdd_fail;

	err = regulator_enable(pw->iovdd);
	if (err)
		goto imx185_iovdd_fail;

	usleep_range(1, 2);
	if (info->pdata->cam1_gpio > 0)
		gpio_set_value(info->pdata->cam1_gpio, 1);

	usleep_range(300, 310);

	return 1;


imx185_iovdd_fail:
	regulator_disable(pw->avdd);

imx185_avdd_fail:
	gpio_set_value(info->pdata->af_gpio, 0);

	pr_err("%s failed.\n", __func__);
	return -ENODEV;
}

static int imx185_power_off(struct imx185_power_rail *pw)
{
	struct imx185_info *info = container_of(pw, struct imx185_info, power);

	gpio_set_value(info->pdata->cam1_gpio, 0);
	usleep_range(1, 2);

	regulator_disable(pw->iovdd);
	regulator_disable(pw->avdd);

	return 0;
}

static int imx185_get_flash_cap(struct imx185_info *info)
{
	return -ENODEV;
}

static inline int imx185_set_flash_control(
	struct imx185_info *info, struct imx185_flash_control *fc)
{
	return -ENODEV;
}

static int
imx185_set_mode(struct imx185_info *info, struct imx185_mode *mode)
{
	int sensor_mode;
	int err;
	struct imx185_reg reg_list[8];

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
			 __func__, mode->xres, mode->yres, mode->frame_length,
			 mode->coarse_time, mode->gain);

	if (mode->xres == 1920 && mode->yres == 1200) {
		sensor_mode = IMX185_MODE_1920X1200;
	} else if (mode->xres == 1920 && mode->yres == 1080) {
		sensor_mode = IMX185_MODE_1920X1080;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
			 __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	imx185_get_frame_length_regs(reg_list, mode->frame_length);
	imx185_get_coarse_time_regs(reg_list + 2,
		mode->frame_length - mode->coarse_time - 1);
	imx185_get_gain_reg(reg_list + 4, mode->gain);

	err = imx185_write_table(info,
				mode_table[sensor_mode],
				reg_list, 0);
	if (err)
		return err;
	info->mode = sensor_mode;
	info->frame_length = mode->frame_length;
	pr_info("[IMX185]: stream on.\n");

	return 0;
}

static int
imx185_get_status(struct imx185_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int
imx185_set_frame_length(struct imx185_info *info, u32 frame_length,
						 bool group_hold)
{
	struct imx185_reg reg_list[2];
	int i = 0;
	int ret;

	pr_debug("%s: framelength %d\n", __func__, frame_length);

	imx185_get_frame_length_regs(reg_list, frame_length);

	if (group_hold) {
		ret = imx185_write_reg(info, 0x0104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 2; i++) {
		ret = imx185_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx185_write_reg(info, 0x0104, 0x0);
		if (ret)
			return ret;
	}
	info->frame_length = frame_length;

	return 0;
}

static int
imx185_set_coarse_time(struct imx185_info *info, u32 coarse_time,
						 bool group_hold)
{
	int ret;

	struct imx185_reg reg_list[2];
	int i = 0;

	pr_debug("%s: coarsetime %d\n", __func__, coarse_time);

	imx185_get_coarse_time_regs(reg_list,
		info->frame_length - coarse_time - 1);

	if (group_hold) {
		ret = imx185_write_reg(info, 0x104, 0x01);
		if (ret)
			return ret;
	}

	for (i = 0; i < 2; i++) {
		ret = imx185_write_reg(info, reg_list[i].addr,
			 reg_list[i].val);
		if (ret)
			return ret;
	}

	if (group_hold) {
		ret = imx185_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx185_set_gain(struct imx185_info *info, u16 gain, bool group_hold)
{
	int ret;
	struct imx185_reg reg_list;

	pr_debug("%s: gain %d\n", __func__, gain);

	imx185_get_gain_reg(&reg_list, gain);

	if (group_hold) {
		ret = imx185_write_reg(info, 0x104, 0x1);
		if (ret)
			return ret;
	}

	ret = imx185_write_reg(info, reg_list.addr, reg_list.val);
	if (ret)
		return ret;

	if (group_hold) {
		ret = imx185_write_reg(info, 0x104, 0x0);
		if (ret)
			return ret;
	}
	return 0;
}

static int
imx185_set_group_hold(struct imx185_info *info, struct imx185_ae *ae)
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
		ret = imx185_write_reg(info, IMX185_REG_HOLD, 0x1);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		imx185_set_gain(info, ae->gain, false);
	if (ae->frame_length_enable)
		imx185_set_frame_length(info, ae->frame_length, false);
	if (ae->coarse_time_enable)
		imx185_set_coarse_time(info, ae->coarse_time, false);

	if (group_hold_enabled) {
		ret = imx185_write_reg(info, IMX185_REG_HOLD, 0x0);
		if (ret)
			return ret;
	}

	return 0;
}

static int imx185_get_sensor_id(struct imx185_info *info)
{
	int ret = 0;
	int i;

	pr_debug("%s\n", __func__);
	if (info->sensor_data.fuse_id_size)
		return 0;

	for (i = 0; i < 9; i++) {
		info->sensor_data.fuse_id[i] = 0xaa;
	}

	if (!ret)
		info->sensor_data.fuse_id_size = i;

	return ret;
}

static void imx185_mclk_disable(struct imx185_info *info)
{
	dev_err(&info->i2c_client->dev, "%s: disable MCLK\n", __func__);
	clk_disable_unprepare(info->mclk);
}

static int imx185_mclk_enable(struct imx185_info *info)
{
	int err;
	unsigned long mclk_init_rate = 37125000;

	dev_err(&info->i2c_client->dev, "%s: enable MCLK with %lu Hz\n",
		__func__, mclk_init_rate);

	err = clk_set_rate(info->mclk, mclk_init_rate);
	if (!err)
		err = clk_prepare_enable(info->mclk);
	return err;
}

static long
imx185_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct imx185_info *info = file->private_data;

	switch (cmd) {
	case IMX185_IOCTL_SET_POWER:
		if (!info->pdata)
			break;
		if (arg && info->pdata->power_on) {
			err = imx185_mclk_enable(info);
			if (!err)
				err = info->pdata->power_on(&info->power);
			if (err < 0)
				imx185_mclk_disable(info);
		}
		if (!arg) {
			info->pdata->power_off(&info->power);
			imx185_mclk_disable(info);
		}
		break;
	case IMX185_IOCTL_SET_MODE:
	{
		struct imx185_mode mode;
		if (copy_from_user(&mode, (const void __user *)arg,
			sizeof(struct imx185_mode))) {
			pr_err("%s:Failed to get mode from user.\n", __func__);
			return -EFAULT;
		}
		return imx185_set_mode(info, &mode);
	}
	case IMX185_IOCTL_SET_FRAME_LENGTH:
		return imx185_set_frame_length(info, (u32)arg, true);
	case IMX185_IOCTL_SET_COARSE_TIME:
		return imx185_set_coarse_time(info, (u32)arg, true);
	case IMX185_IOCTL_SET_GAIN:
		return imx185_set_gain(info, (u16)arg, true);
	case IMX185_IOCTL_GET_STATUS:
	{
		u8 status;

		err = imx185_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 1)) {
			pr_err("%s:Failed to copy status to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX185_IOCTL_GET_SENSORDATA:
	{
		err = imx185_get_sensor_id(info);

		if (err) {
			pr_err("%s:Failed to get fuse id info.\n", __func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &info->sensor_data,
				sizeof(struct imx185_sensordata))) {
			pr_info("%s:Failed to copy fuse id to user space\n",
				__func__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX185_IOCTL_SET_GROUP_HOLD:
	{
		struct imx185_ae ae;
		if (copy_from_user(&ae, (const void __user *)arg,
			sizeof(struct imx185_ae))) {
			pr_info("%s:fail group hold\n", __func__);
			return -EFAULT;
		}
		return imx185_set_group_hold(info, &ae);
	}
	case IMX185_IOCTL_SET_FLASH_MODE:
	{
		struct imx185_flash_control values;

		dev_dbg(&info->i2c_client->dev,
			"IMX185_IOCTL_SET_FLASH_MODE\n");
		if (copy_from_user(&values,
			(const void __user *)arg,
			sizeof(struct imx185_flash_control))) {
			err = -EFAULT;
			break;
		}
		err = imx185_set_flash_control(info, &values);
		break;
	}
	case IMX185_IOCTL_GET_FLASH_CAP:
		err = imx185_get_flash_cap(info);
		break;
	default:
		pr_err("%s:unknown cmd.\n", __func__);
		err = -EINVAL;
	}

	return err;
}

static int imx185_debugfs_show(struct seq_file *s, void *unused)
{
	struct imx185_info *dev = s->private;

	dev_dbg(&dev->i2c_client->dev, "%s: ++\n", __func__);

	mutex_lock(&dev->imx185_camera_lock);
	mutex_unlock(&dev->imx185_camera_lock);

	return 0;
}

static ssize_t imx185_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset)
{
	struct imx185_info *dev =
			((struct seq_file *)file->private_data)->private;
	struct i2c_client *i2c_client = dev->i2c_client;
	int ret = 0;
	char buffer[MAX_BUFFER_SIZE];
	u32 address;
	u32 data;
	u8 readback = 0;
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
	ret |= imx185_write_reg(dev, address, data);
read:
	ret |= imx185_read_reg(dev, address, &readback);
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

static int imx185_debugfs_open(struct inode *inode, struct file *file)
{
	struct imx185_info *dev = inode->i_private;
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	return single_open(file, imx185_debugfs_show, inode->i_private);
}

static const struct file_operations imx185_debugfs_fops = {
	.open		= imx185_debugfs_open,
	.read		= seq_read,
	.write		= imx185_debugfs_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void imx185_remove_debugfs(struct imx185_info *dev)
{
	struct i2c_client *i2c_client = dev->i2c_client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);

	debugfs_remove_recursive(dev->debugdir);
	dev->debugdir = NULL;
}

static void imx185_create_debugfs(struct imx185_info *dev)
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
				&imx185_debugfs_fops);
	if (!ret)
		goto remove_debugfs;

	return;
remove_debugfs:
	dev_err(&i2c_client->dev, "couldn't create debugfs\n");
	imx185_remove_debugfs(dev);
}


static int
imx185_open(struct inode *inode, struct file *file)
{
	struct miscdevice	*miscdev = file->private_data;
	struct imx185_info *info;

	info = container_of(miscdev, struct imx185_info, miscdev_info);
	/* check if the device is in use */
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("%s:BUSY!\n", __func__);
		return -EBUSY;
	}

	file->private_data = info;

	return 0;
}

static int
imx185_release(struct inode *inode, struct file *file)
{
	struct imx185_info *info = file->private_data;

	file->private_data = NULL;

	/* warn if device is already released */
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}

static int imx185_power_put(struct imx185_power_rail *pw)
{
	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	if (likely(pw->dvdd))
		regulator_put(pw->dvdd);

	if (likely(pw->ext_reg1))
		regulator_put(pw->ext_reg1);

	if (likely(pw->ext_reg2))
		regulator_put(pw->ext_reg2);

	pw->avdd = NULL;
	pw->iovdd = NULL;
	pw->dvdd = NULL;
	pw->ext_reg1 = NULL;
	pw->ext_reg2 = NULL;

	return 0;
}

static int imx185_regulator_get(struct imx185_info *info,
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

static int imx185_power_get(struct imx185_info *info)
{
	struct imx185_power_rail *pw = &info->power;
	int err = 0;

	err |= imx185_regulator_get(info, &pw->dvdd, "vdig"); /* digital 1.2v */
	err |= imx185_regulator_get(info, &pw->iovdd, "vif"); /* IO 1.8v */
	err |= imx185_regulator_get(info, &pw->avdd, "vana"); /* analog 2.7v */

	return err;
}

static const struct file_operations imx185_fileops = {
	.owner = THIS_MODULE,
	.open = imx185_open,
	.unlocked_ioctl = imx185_ioctl,
	.release = imx185_release,
};

static struct miscdevice imx185_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx185",
	.fops = &imx185_fileops,
};

static struct of_device_id imx185_of_match[] = {
	{ .compatible = "nvidia,imx185", },
	{ },
};

MODULE_DEVICE_TABLE(of, imx185_of_match);

static struct imx185_platform_data *imx185_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct imx185_platform_data *board_info_pdata;

	board_info_pdata = devm_kzalloc(&client->dev, sizeof(*board_info_pdata),
			GFP_KERNEL);
	if (!board_info_pdata) {
		dev_err(&client->dev, "Failed to allocate pdata\n");
		return NULL;
	}

	of_property_read_string(np, "clocks", &board_info_pdata->mclk_name);
	board_info_pdata->cam1_gpio = of_get_named_gpio(np, "cam1-gpios", 0);
	board_info_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	board_info_pdata->af_gpio = of_get_named_gpio(np, "af-gpios", 0);

	board_info_pdata->ext_reg = of_property_read_bool(np, "nvidia,ext_reg");

	board_info_pdata->power_on = imx185_power_on;
	board_info_pdata->power_off = imx185_power_off;

	return board_info_pdata;
}

static int
imx185_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct imx185_info *info;
	int err;
	const char *mclk_name;

	pr_info("[IMX185]: probing sensor.\n");

	info = devm_kzalloc(&client->dev,
			sizeof(struct imx185_info), GFP_KERNEL);
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

	if (client->dev.of_node) {
		info->pdata = imx185_parse_dt(client);
	} else {
		info->pdata = client->dev.platform_data;
	}

	if (!info->pdata) {
		pr_err("[IMX185]:%s:Unable to get platform data\n", __func__);
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

	imx185_power_get(info);

	memcpy(&info->miscdev_info,
		&imx185_device,
		sizeof(struct miscdevice));

	err = misc_register(&info->miscdev_info);
	if (err) {
		pr_err("%s:Unable to register misc device!\n", __func__);
		goto imx185_probe_fail;
	}

	i2c_set_clientdata(client, info);
	/* create debugfs interface */
	imx185_create_debugfs(info);
	pr_err("[IMX185]: end of probing sensor.\n");
	return 0;

imx185_probe_fail:
	imx185_power_put(&info->power);

	return err;
}

static int
imx185_remove(struct i2c_client *client)
{
	struct imx185_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx185_device);

	imx185_power_put(&info->power);

	imx185_remove_debugfs(info);
	return 0;
}

static const struct i2c_device_id imx185_id[] = {
	{ "imx185", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx185_id);

static struct i2c_driver imx185_i2c_driver = {
	.driver = {
		.name = "imx185",
		.owner = THIS_MODULE,
		.of_match_table = imx185_of_match,
	},
	.probe = imx185_probe,
	.remove = imx185_remove,
	.id_table = imx185_id,
};

static int __init imx185_init(void)
{
	pr_info("[IMX185] sensor driver loading\n");
	return i2c_add_driver(&imx185_i2c_driver);
}

static void __exit imx185_exit(void)
{
	i2c_del_driver(&imx185_i2c_driver);
}

module_init(imx185_init);
module_exit(imx185_exit);
