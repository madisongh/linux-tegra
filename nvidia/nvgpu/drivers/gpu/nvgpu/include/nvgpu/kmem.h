/*
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __NVGPU_KMEM_H__
#define __NVGPU_KMEM_H__

/*
 * Incase this isn't defined already.
 */
#ifndef _THIS_IP_
#define _THIS_IP_  ({ __label__ __here; __here: (unsigned long)&&__here; })
#endif

struct gk20a;

/**
 * DOC: Kmem cache support
 *
 * In Linux there is support for the notion of a kmem_cache. It gives better
 * memory usage characteristics for lots of allocations of the same size. Think
 * structs that get allocated over and over. Normal kmalloc() type routines
 * typically round to the next power-of-2 since that's easy.
 *
 * But if we know the size ahead of time the packing for the allocations can be
 * much better. This is the benefit of a slab allocator. This type hides the
 * underlying kmem_cache (or absense thereof).
 */
struct nvgpu_kmem_cache;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
/*
 * Uncomment this if you want to enable stack traces in the memory profiling.
 * Since this is a fairly high overhead operation and is only necessary for
 * debugging actual bugs it's left here for developers to enable.
 */
/* #define __NVGPU_SAVE_KALLOC_STACK_TRACES */

/*
 * Defined per-OS.
 */
struct nvgpu_mem_alloc_tracker;
#endif


/**
 * nvgpu_kmem_cache_create - create an nvgpu kernel memory cache.
 *
 * @g		The GPU driver struct using this cache.
 * @size	Size of the object allocated by the cache.
 *
 * This cache can be used to allocate objects of size @size. Common usage would
 * be for a struct that gets allocated a lot. In that case @size should be
 * sizeof(struct my_struct).
 *
 * A given implementation of this need not do anything special. The allocation
 * routines can simply be passed on to nvgpu_kzalloc() if desired so packing
 * and alignment of the structs cannot be assumed.
 */
struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size);

/**
 * nvgpu_kmem_cache_destroy - destroy a cache created by
 *                            nvgpu_kmem_cache_create().
 *
 * @cache	The cache to destroy.
 */
void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache);

/**
 * nvgpu_kmem_cache_alloc - Allocate an object from the cache
 *
 * @cache	The cache to alloc from.
 */
void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache);

/**
 * nvgpu_kmem_cache_free - Free an object back to a cache
 *
 * @cache	The cache to return the object to.
 * @ptr		Pointer to the object to free.
 */
void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr);

/**
 * nvgpu_kmalloc - Allocate from the kernel's allocator.
 *
 * @g:		Current GPU.
 * @size:	Size of the allocation.
 *
 * Allocate a chunk of system memory from the kernel. Allocations larger than 1
 * page may fail even when there may appear to be enough memory.
 *
 * This function may sleep so cannot be used in IRQs.
 */
#define nvgpu_kmalloc(g, size)		__nvgpu_kmalloc(g, size, _THIS_IP_)

/**
 * nvgpu_kzalloc - Allocate from the kernel's allocator.
 *
 * @g:		Current GPU.
 * @size:	Size of the allocation.
 *
 * Identical to nvgpu_kalloc() except the memory will be zeroed before being
 * returned.
 */
#define nvgpu_kzalloc(g, size)		__nvgpu_kzalloc(g, size, _THIS_IP_)

/**
 * nvgpu_kcalloc - Allocate from the kernel's allocator.
 *
 * @g:		Current GPU.
 * @n:          Number of objects.
 * @size:	Size of each object.
 *
 * Identical to nvgpu_kalloc() except the size of the memory chunk returned is
 * @n * @size.
 */
#define nvgpu_kcalloc(g, n, size)	__nvgpu_kcalloc(g, n, size, _THIS_IP_)

/**
 * nvgpu_vmalloc - Allocate memory and return a map to it.
 *
 * @g:		Current GPU.
 * @size:	Size of the allocation.
 *
 * Allocate some memory and return a pointer to a virtual memory mapping of
 * that memory in the kernel's virtual address space. The underlying physical
 * memory is not guaranteed to be contiguous (and indeed likely isn't). This
 * allows for much larger allocations to be done without worrying about as much
 * about physical memory fragmentation.
 *
 * This function may sleep.
 */
#define nvgpu_vmalloc(g, size)		__nvgpu_vmalloc(g, size, _THIS_IP_)

/**
 * nvgpu_vzalloc - Allocate memory and return a map to it.
 *
 * @g:		Current GPU.
 * @size:	Size of the allocation.
 *
 * Identical to nvgpu_vmalloc() except this will return zero'ed memory.
 */
#define nvgpu_vzalloc(g, size)		__nvgpu_vzalloc(g, size, _THIS_IP_)

/**
 * nvgpu_kfree - Frees an alloc from nvgpu_kmalloc, nvgpu_kzalloc,
 *               nvgpu_kcalloc.
 *
 * @g:		Current GPU.
 * @addr:	Address of object to free.
 */
#define nvgpu_kfree(g, addr)		__nvgpu_kfree(g, addr)

/**
 * nvgpu_vfree - Frees an alloc from nvgpu_vmalloc, nvgpu_vzalloc.
 *
 * @g:		Current GPU.
 * @addr:	Address of object to free.
 */
#define nvgpu_vfree(g, addr)		__nvgpu_vfree(g, addr)

#define kmem_dbg(fmt, args...)			\
	gk20a_dbg(gpu_dbg_kmem, fmt, ##args)

/**
 * nvgpu_kmem_init - Initialize the kmem tracking stuff.
 *
 *@g: The driver to init.
 *
 * Returns non-zero on failure.
 */
int nvgpu_kmem_init(struct gk20a *g);

/**
 * nvgpu_kmem_fini - Finalize the kmem tracking code
 *
 * @g     - The GPU.
 * @flags - Flags that control operation of this finalization.
 *
 * Cleanup resources used by nvgpu_kmem. Available flags for cleanup are:
 *
 *   %NVGPU_KMEM_FINI_DO_NOTHING
 *   %NVGPU_KMEM_FINI_FORCE_CLEANUP
 *   %NVGPU_KMEM_FINI_DUMP_ALLOCS
 *   %NVGPU_KMEM_FINI_WARN
 *   %NVGPU_KMEM_FINI_BUG
 *
 * %NVGPU_KMEM_FINI_DO_NOTHING will be overridden by anything else specified.
 * Put another way don't just add %NVGPU_KMEM_FINI_DO_NOTHING and expect that
 * to suppress other flags from doing anything.
 */
void nvgpu_kmem_fini(struct gk20a *g, int flags);

/*
 * These will simply be ignored if CONFIG_NVGPU_TRACK_MEM_USAGE is not defined.
 */
#define NVGPU_KMEM_FINI_DO_NOTHING		0
#define NVGPU_KMEM_FINI_FORCE_CLEANUP		(1 << 0)
#define NVGPU_KMEM_FINI_DUMP_ALLOCS		(1 << 1)
#define NVGPU_KMEM_FINI_WARN			(1 << 2)
#define NVGPU_KMEM_FINI_BUG			(1 << 3)

/*
 * When there's other implementations make sure they are included instead of
 * Linux when not compiling on Linux!
 */
#include <nvgpu/kmem_linux.h>

static inline void *__nvgpu_big_alloc(struct gk20a *g, size_t size, bool clear)
{
	void *p;

	if (size > PAGE_SIZE) {
		if (clear)
			p = nvgpu_vzalloc(g, size);
		else
			p = nvgpu_vmalloc(g, size);
	} else {
		if (clear)
			p = nvgpu_kzalloc(g, size);
		else
			p = nvgpu_kmalloc(g, size);
	}

	return p;
}

/**
 * nvgpu_big_malloc - Pick virtual or physical alloc based on @size
 *
 * @g - The GPU.
 * @size - Size of the allocation.
 *
 * On some platforms (i.e Linux) it is possible to allocate memory directly
 * mapped into the kernel's address space (kmalloc) or allocate discontiguous
 * pages which are then mapped into a special kernel address range. Each type
 * of allocation has pros and cons. kmalloc() for instance lets you allocate
 * small buffers more space efficiently but vmalloc() allows you to successfully
 * allocate much larger buffers without worrying about fragmentation as much
 * (but will allocate in multiples of page size).
 *
 * This function aims to provide the right allocation for when buffers are of
 * variable size. In some cases the code doesn't know ahead of time if the
 * buffer is going to be big or small so this does the check for you and
 * provides the right type of memory allocation.
 *
 * Returns a pointer to a virtual address range that the kernel can access or
 * %NULL on failure.
 */
static inline void *nvgpu_big_malloc(struct gk20a *g, size_t size)
{
	return __nvgpu_big_alloc(g, size, false);
}

/**
 * nvgpu_big_malloc - Pick virtual or physical alloc based on @size
 *
 * @g - The GPU.
 * @size - Size of the allocation.
 *
 * Zeroed memory version of nvgpu_big_malloc().
 */
static inline void *nvgpu_big_zalloc(struct gk20a *g, size_t size)
{
	return __nvgpu_big_alloc(g, size, true);
}

/**
 * nvgpu_big_free - Free and alloc from nvgpu_big_zalloc() or
 *                  nvgpu_big_malloc().
 * @g - The GPU.
 * @p - A pointer allocated by nvgpu_big_zalloc() or nvgpu_big_malloc().
 */
static inline void nvgpu_big_free(struct gk20a *g, void *p)
{
	/*
	 * This will have to be fixed eventually. Allocs that use
	 * nvgpu_big_[mz]alloc() will need to remember the size of the alloc
	 * when freeing.
	 */
	if (is_vmalloc_addr(p))
		nvgpu_vfree(g, p);
	else
		nvgpu_kfree(g, p);
}

#endif /* __NVGPU_KMEM_H__ */
