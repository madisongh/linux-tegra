/*
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#define pr_fmt(fmt) "cc_gamedata: " fmt

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/vmalloc.h>
#include <asm/page.h>

#include "cc.h"

static struct gamedata_cvt_context *gd_cvt_cxt;

void gd_read_from_cvt(struct gd_ram_buffer *gdrz)
{
	if (!gdrz->game_data)
		gdrz->game_data = kzalloc(gd_cvt_cxt->record_size, GFP_KERNEL);

	if (!gdrz->game_data)
		return;
	memcpy_fromio(gdrz->game_data, gdrz->vaddr, gd_cvt_cxt->record_size);
}

int notrace gd_ram_write(struct gd_ram_buffer *gdrz,
	const void *s, unsigned int count)
{
	int c = count;

	if (unlikely(c > gdrz->mem_size))
		c = gdrz->mem_size;

	memcpy_toio(gdrz->vaddr, s, count);
	if (!gdrz->game_data)
		gdrz->game_data = kmalloc(c, GFP_KERNEL);

	gdrz->data_size = c;
	return c;
}

void gd_ram_free_data(struct gd_ram_buffer *gdrz)
{
	kfree(gdrz->game_data);
	gdrz->game_data = NULL;
	gdrz->data_size = 0;
}

static void *gd_ram_iomap(phys_addr_t start, size_t size)
{
	void *va;

	if (!request_mem_region(start, size, "GameData RAM")) {
		pr_err("request mem region (0x%llx@0x%llx) failed\n",
			(unsigned long long)size, (unsigned long long)start);
		return NULL;
	}

	va = ioremap(start, size);

	return va;
}

static int gd_ram_buffer_map(phys_addr_t start, phys_addr_t size,
		struct gd_ram_buffer *gdrz)
{
	gdrz->paddr = start;
	gdrz->mem_size = size;

	gdrz->vaddr = gd_ram_iomap(start, size);

	if (!gdrz->vaddr) {
		pr_err("%s: Failed to map 0x%llx at 0x%llx\n", __func__,
			(unsigned long long)size, (unsigned long long)start);
		return -ENOMEM;
	}

	return 0;
}

void gd_ram_free(struct gd_ram_buffer *gdrz)
{
	if (!gdrz)
		return;

	if (gdrz->vaddr) {
		iounmap(gdrz->vaddr);
		release_mem_region(gdrz->paddr, gdrz->mem_size);
		gdrz->vaddr = NULL;
	}
	gd_ram_free_data(gdrz);
	kfree(gdrz);
}

struct gd_ram_buffer *gd_ram_new(phys_addr_t start, size_t size)
{
	struct gd_ram_buffer *gdrz;
	int ret = -ENOMEM;

	gdrz = kzalloc(sizeof(struct gd_ram_buffer), GFP_KERNEL);
	if (!gdrz) {
		pr_err("failed to allocate Gamedata RAM buffer\n");
		goto err;
	}

	ret = gd_ram_buffer_map(start, size, gdrz);
	if (ret)
		goto err;

	return gdrz;
err:
	gd_ram_free(gdrz);
	return ERR_PTR(ret);
}

ssize_t gamedata_cvt_read(size_t *count, char **buf)
{
	ssize_t size;
	struct gd_ram_buffer *gdrz = NULL;
	int buffer_cnt;

	/*
	 * mask the GD_BUFFER_COMPLETE and select write completed
	 * buffer by shifting mask value to shift left by 1.
	 */
	buffer_cnt = (gd_cvt_cxt->flags & GD_BUFFER_COMPLETE) >> 1;

	gdrz = gd_cvt_cxt->gd_ram_buffs[buffer_cnt];
	if (!gdrz)
		return -EINVAL;

	gd_read_from_cvt(gdrz);

	*buf = gdrz->game_data;

	return size;
}
EXPORT_SYMBOL_GPL(gamedata_cvt_read);

int gamedata_cvt_write(const char *buf, size_t size)
{
	struct gd_ram_buffer *gdrz;
	unsigned int buffer_cnt, buffer_mask;

	/*
	 * mask the GD_BUFFER and select next buffer by toggling
	 * current buffer
	 */
	buffer_mask = gd_cvt_cxt->flags & GD_BUFFER;
	buffer_cnt = buffer_mask ^ GD_BUFFER;
	gd_cvt_cxt->flags ^= GD_BUFFER;
	gdrz = gd_cvt_cxt->gd_ram_buffs[buffer_cnt];

	gd_ram_write(gdrz, buf, size);

	gd_cvt_cxt->flags ^= GD_BUFFER_COMPLETE;

	return 0;
}
EXPORT_SYMBOL_GPL(gamedata_cvt_write);

static int gamedata_cvt_init_buffs(struct device *dev,
			phys_addr_t *paddr, size_t total_mem_sz)
{
	int err = -ENOMEM;
	int i;

	gd_cvt_cxt->max_record_cnt = total_mem_sz / gd_cvt_cxt->record_size;
	if (!gd_cvt_cxt->max_record_cnt)
		return -ENOMEM;

	/* As of now, use only max_record_cnt = 2 */
	if (gd_cvt_cxt->max_record_cnt > MAX_RECORD_CNT)
		gd_cvt_cxt->max_record_cnt = MAX_RECORD_CNT;

	gd_cvt_cxt->gd_ram_buffs = devm_kzalloc(dev,
			sizeof(*gd_cvt_cxt->gd_ram_buffs) *
				 gd_cvt_cxt->max_record_cnt, GFP_KERNEL);
	if (!gd_cvt_cxt->gd_ram_buffs) {
		dev_err(dev, "failed to initialize a gdrz for dumps\n");
		goto fail_gdrz;
	}

	for (i = 0; i < gd_cvt_cxt->max_record_cnt; i++) {
		gd_cvt_cxt->gd_ram_buffs[i] = gd_ram_new(*paddr,
						 gd_cvt_cxt->record_size);
		if (IS_ERR(gd_cvt_cxt->gd_ram_buffs[i])) {
			err = PTR_ERR(gd_cvt_cxt->gd_ram_buffs[i]);
			dev_err(dev, "failed to request mem region"
					 "(0x%zx@0x%llx): %d\n", total_mem_sz,
					 (unsigned long long)*paddr, err);

			while (i > 0) {
				i--;
				gd_ram_free(gd_cvt_cxt->gd_ram_buffs[i]);
			}
			goto fail_gdrz;
		}
		*paddr += gd_cvt_cxt->record_size;
	}

	return 0;
fail_gdrz:
	gd_cvt_cxt->max_record_cnt = 0;
	return err;
}

int gamedata_cvt_parse_dt(struct platform_device *pdev,
		struct crtlcond_platform_data *pdata)
{
	struct device_node *of_node = pdev->dev.of_node;

	dev_dbg(&pdev->dev, "using Device Tree\n");

	of_property_read_u32(of_node, "nvidia,read-write-bytes",
			&pdata->read_write_bytes);
	of_property_read_u32(of_node, "nvidia,periodic-save-time",
			&pdata->cc_timer_timeout);

	return 0;
}

int gamedata_cvt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct crtlcond_platform_data *pdata = pdev->dev.platform_data;
	size_t total_mem_sz;
	phys_addr_t paddr;
	int err = -EINVAL;

	gd_cvt_cxt = devm_kzalloc(&pdev->dev, sizeof(*gd_cvt_cxt), GFP_KERNEL);
	if (!gd_cvt_cxt) {
		dev_err(dev, "failed to initialize a gd_cvt_cxt\n");
		goto fail_out;
	}

	if (dev->of_node) {
		err = gamedata_cvt_parse_dt(pdev, pdata);
		if (err < 0)
			goto fail_out;
	}

	if (!pdata->cvt_mem_size) {
		pr_err("The memory size must be non-zero, cvt_mem_size = %lu\n",
			 pdata->cvt_mem_size);
		goto fail_out;
	}

	gd_cvt_cxt->size = pdata->cvt_mem_size;
	gd_cvt_cxt->phys_addr = pdata->cvt_mem_address;

	paddr = gd_cvt_cxt->phys_addr;

	total_mem_sz = gd_cvt_cxt->size;
	gd_cvt_cxt->record_size = pdata->read_write_bytes;
	err = gamedata_cvt_init_buffs(dev, &paddr, total_mem_sz);
	if (err)
		goto fail_out;

	pr_info("attached 0x%lx@0x%llx\n",
		gd_cvt_cxt->size, (unsigned long long)gd_cvt_cxt->phys_addr);

	return 0;

fail_out:
	return err;
}
