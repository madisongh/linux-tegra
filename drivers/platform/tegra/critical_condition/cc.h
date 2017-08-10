/*
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#define WDT_TIMEOUT				2
#define MAX_RECORD_CNT				2
#define START_ADDRESS0				0
#define CC_PAGE_WRITE				1

/*
 * This enum is used to update buffer status while writing
 * data to GameData RAM cvt, currently using 2-buffer
 * @GD_BUFFER		To select buffer among 2-buffer to write/read to/from
 *			GameData RAM cvt by toggling this element.
 * @GD_BUFFER_COMPLETE	Once data write to 'GameData RAM' cvt complete, this
 *			element will be set for that buffer.
 */
enum gd_buffer_status {
	GD_BUFFER = 1,
	GD_BUFFER_COMPLETE,
};

/*
 * structure to maintain gamedata cvt
 * @gd_ram_buffs	Contains array of buffers, currently 2 buffer will
 *			be used.
 * @phys_addr		Start of the physical cvt address
 * @size		Size of the pysical cvt address
 * @max_record_cnt	No. of buffer will be used.
 * @flags		flags to check buffer status, using
 *			member of 'enum gd_buffer_status'
 */
struct gamedata_cvt_context {
	struct gd_ram_buffer **gd_ram_buffs;
	phys_addr_t phys_addr;
	unsigned long size;
	unsigned int max_record_cnt;
	size_t record_size;
	unsigned int flags;
};

/*
 * structure to maintain per buffer, currently 2-buffer will be used
 * @paddr		physical address of one buffer.
 * @mem_size		buffer size for one buffer,
 * @vaddr		cpu-address mapped via ioremap for paddr
 * @game_data		data to be write/read to/from GameData RAM carveout
 * @data_size		data-size to be write/read to/from GameData RAM carveout
 */
struct gd_ram_buffer {
	phys_addr_t paddr;
	size_t mem_size;
	void *vaddr;
	char *game_data;
	size_t data_size;
};

/*
 * Critical Condition Carveout platform data, this will be fill
 * from *.dtb value
 * @mem_size		Total memory size for gamedata_cvt, currently 1MiB
 * @mem_address		physical memory address to contain gamedata_cvt
 * @read_write_bytes	data-size to be write/read to/from FRAM
 * @flags		flags to check write has been done till now
 */
struct crtlcond_platform_data {
	unsigned long   cvt_mem_size;
	unsigned long   cvt_mem_address;
	u32		read_write_bytes;
	int		flags;
};

struct gd_ram_buffer *gd_ram_new(phys_addr_t start, size_t size);
void gd_ram_free(struct gd_ram_buffer *grb);

int gd_ram_write(struct gd_ram_buffer *grb, const void *s,
	unsigned int count);

void gd_read_from_cvt(struct gd_ram_buffer *grb);
void gd_ram_free_data(struct gd_ram_buffer *grb);

int gamedata_cvt_write(const char *buf, size_t size);
ssize_t gamedata_cvt_read(size_t *count, char **buf);
int gamedata_cvt_probe(struct platform_device *pdev);

int qspi_write(loff_t to, size_t len,
		size_t *retlen, const u_char *buf);
