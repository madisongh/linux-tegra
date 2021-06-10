/*
 * drivers/video/tegra/host/gk20a/pmu_gk20a.h
 *
 * GK20A PMU (aka. gPMU outside gk20a context)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __PMU_GK20A_H__
#define __PMU_GK20A_H__

#include <linux/version.h>
#include <nvgpu/flcnif_cmn.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

/* defined by pmu hw spec */
#define GK20A_PMU_VA_SIZE		(512 * 1024 * 1024)
#define GK20A_PMU_UCODE_SIZE_MAX	(256 * 1024)
#define GK20A_PMU_SEQ_BUF_SIZE		4096

#define ZBC_MASK(i)			(~(~(0) << ((i)+1)) & 0xfffe)

#define APP_VERSION_NC_3	21548462
#define APP_VERSION_NC_2	23782727
#define APP_VERSION_NC_1	20313802
#define APP_VERSION_NC_0	20360931
#define APP_VERSION_GM206	20652057
#define APP_VERSION_NV_GPU	21307569
#define APP_VERSION_NV_GPU_1	21308030
#define APP_VERSION_GM20B_5 20490253
#define APP_VERSION_GM20B_4 19008461
#define APP_VERSION_GM20B_3 18935575
#define APP_VERSION_GM20B_2 18694072
#define APP_VERSION_GM20B_1 18547257
#define APP_VERSION_GM20B 17615280
#define APP_VERSION_3 18357968
#define APP_VERSION_2 18542378
#define APP_VERSION_1 17997577 /*Obsolete this once 18357968 gets in*/
#define APP_VERSION_0 16856675

/*Fuse defines*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
#define FUSE_GCPLEX_CONFIG_FUSE_0           0x2C8
#endif
#define PMU_MODE_MISMATCH_STATUS_MAILBOX_R  6
#define PMU_MODE_MISMATCH_STATUS_VAL        0xDEADDEAD

enum {
	GK20A_PMU_DMAIDX_UCODE		= 0,
	GK20A_PMU_DMAIDX_VIRT		= 1,
	GK20A_PMU_DMAIDX_PHYS_VID	= 2,
	GK20A_PMU_DMAIDX_PHYS_SYS_COH	= 3,
	GK20A_PMU_DMAIDX_PHYS_SYS_NCOH	= 4,
	GK20A_PMU_DMAIDX_RSVD		= 5,
	GK20A_PMU_DMAIDX_PELPG		= 6,
	GK20A_PMU_DMAIDX_END		= 7
};

#define GK20A_PMU_TRACE_BUFSIZE     0x4000   /* 4K */
#define GK20A_PMU_DMEM_BLKSIZE2		8

#define GK20A_PMU_UCODE_NB_MAX_OVERLAY	    32
#define GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH  64

struct pmu_ucode_desc {
	u32 descriptor_size;
	u32 image_size;
	u32 tools_version;
	u32 app_version;
	char date[GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH];
	u32 bootloader_start_offset;
	u32 bootloader_size;
	u32 bootloader_imem_offset;
	u32 bootloader_entry_point;
	u32 app_start_offset;
	u32 app_size;
	u32 app_imem_offset;
	u32 app_imem_entry;
	u32 app_dmem_offset;
	u32 app_resident_code_offset;  /* Offset from appStartOffset */
	u32 app_resident_code_size;    /* Exact size of the resident code ( potentially contains CRC inside at the end ) */
	u32 app_resident_data_offset;  /* Offset from appStartOffset */
	u32 app_resident_data_size;    /* Exact size of the resident code ( potentially contains CRC inside at the end ) */
	u32 nb_overlays;
	struct {u32 start; u32 size;} load_ovl[GK20A_PMU_UCODE_NB_MAX_OVERLAY];
	u32 compressed;
};

struct pmu_ucode_desc_v1 {
	u32 descriptor_size;
	u32 image_size;
	u32 tools_version;
	u32 app_version;
	char date[GK20A_PMU_UCODE_NB_MAX_DATE_LENGTH];
	u32 bootloader_start_offset;
	u32 bootloader_size;
	u32 bootloader_imem_offset;
	u32 bootloader_entry_point;
	u32 app_start_offset;
	u32 app_size;
	u32 app_imem_offset;
	u32 app_imem_entry;
	u32 app_dmem_offset;
	u32 app_resident_code_offset;
	u32 app_resident_code_size;
	u32 app_resident_data_offset;
	u32 app_resident_data_size;
	u32 nb_imem_overlays;
	u32 nb_dmem_overlays;
	struct {u32 start; u32 size; } load_ovl[64];
	u32 compressed;
};

#define PMU_PGENG_GR_BUFFER_IDX_INIT	(0)
#define PMU_PGENG_GR_BUFFER_IDX_ZBC	(1)
#define PMU_PGENG_GR_BUFFER_IDX_FECS	(2)

struct pmu_gk20a;
struct pmu_queue;

struct pmu_queue {

	/* used by hw, for BIOS/SMI queue */
	u32 mutex_id;
	u32 mutex_lock;
	/* used by sw, for LPQ/HPQ queue */
	struct nvgpu_mutex mutex;

	/* current write position */
	u32 position;
	/* physical dmem offset where this queue begins */
	u32 offset;
	/* logical queue identifier */
	u32 id;
	/* physical queue index */
	u32 index;
	/* in bytes */
	u32 size;

	/* open-flag */
	u32 oflag;
	bool opened; /* opened implies locked */
};

struct pmu_mutex {
	u32 id;
	u32 index;
	u32 ref_cnt;
};

#define PMU_MAX_NUM_SEQUENCES		(256)
#define PMU_SEQ_BIT_SHIFT		(5)
#define PMU_SEQ_TBL_SIZE	\
		(PMU_MAX_NUM_SEQUENCES >> PMU_SEQ_BIT_SHIFT)

#define PMU_INVALID_SEQ_DESC		(~0)

enum
{
	PMU_SEQ_STATE_FREE = 0,
	PMU_SEQ_STATE_PENDING,
	PMU_SEQ_STATE_USED,
	PMU_SEQ_STATE_CANCELLED
};

struct pmu_payload {
	struct {
		void *buf;
		u32 offset;
		u32 size;
		u32 fb_size;
	} in, out;
};

struct pmu_surface {
	struct mem_desc vidmem_desc;
	struct mem_desc sysmem_desc;
	struct flcn_mem_desc_v0 params;
};

typedef void (*pmu_callback)(struct gk20a *, struct pmu_msg *, void *, u32,
	u32);

struct pmu_sequence {
	u8 id;
	u32 state;
	u32 desc;
	struct pmu_msg *msg;
	union {
		struct pmu_allocation_v0 in_v0;
		struct pmu_allocation_v1 in_v1;
		struct pmu_allocation_v2 in_v2;
		struct pmu_allocation_v3 in_v3;
	};
	struct mem_desc *in_mem;
	union {
		struct pmu_allocation_v0 out_v0;
		struct pmu_allocation_v1 out_v1;
		struct pmu_allocation_v2 out_v2;
		struct pmu_allocation_v3 out_v3;
	};
	struct mem_desc *out_mem;
	u8 *out_payload;
	pmu_callback callback;
	void* cb_params;
};

/*PG defines used by nvpgu-pmu*/
struct pmu_pg_stats_data {
	u32 gating_cnt;
	u32 ingating_time;
	u32 ungating_time;
	u32 avg_entry_latency_us;
	u32 avg_exit_latency_us;
};

#define PMU_PG_IDLE_THRESHOLD_SIM		1000
#define PMU_PG_POST_POWERUP_IDLE_THRESHOLD_SIM	4000000
/* TBD: QT or else ? */
#define PMU_PG_IDLE_THRESHOLD			15000
#define PMU_PG_POST_POWERUP_IDLE_THRESHOLD	1000000

#define PMU_PG_LPWR_FEATURE_RPPG 0x0
#define PMU_PG_LPWR_FEATURE_MSCG 0x1

/* state transition :
    OFF => [OFF_ON_PENDING optional] => ON_PENDING => ON => OFF
    ON => OFF is always synchronized */
#define PMU_ELPG_STAT_OFF		0   /* elpg is off */
#define PMU_ELPG_STAT_ON		1   /* elpg is on */
#define PMU_ELPG_STAT_ON_PENDING	2   /* elpg is off, ALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_OFF_PENDING	3   /* elpg is on, DISALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_OFF_ON_PENDING	4   /* elpg is off, caller has requested on, but ALLOW
					       cmd hasn't been sent due to ENABLE_ALLOW delay */

#define PG_REQUEST_TYPE_GLOBAL 0x0
#define PG_REQUEST_TYPE_PSTATE 0x1

#define PMU_MSCG_DISABLED 0
#define PMU_MSCG_ENABLED 1

/* Default Sampling Period of AELPG */
#define APCTRL_SAMPLING_PERIOD_PG_DEFAULT_US                    (1000000)

/* Default values of APCTRL parameters */
#define APCTRL_MINIMUM_IDLE_FILTER_DEFAULT_US                   (100)
#define APCTRL_MINIMUM_TARGET_SAVING_DEFAULT_US                 (10000)
#define APCTRL_POWER_BREAKEVEN_DEFAULT_US                       (2000)
#define APCTRL_CYCLES_PER_SAMPLE_MAX_DEFAULT                    (200)
/*PG defines used by nvpgu-pmu*/

/* Falcon Register index */
#define PMU_FALCON_REG_R0		(0)
#define PMU_FALCON_REG_R1		(1)
#define PMU_FALCON_REG_R2		(2)
#define PMU_FALCON_REG_R3		(3)
#define PMU_FALCON_REG_R4		(4)
#define PMU_FALCON_REG_R5		(5)
#define PMU_FALCON_REG_R6		(6)
#define PMU_FALCON_REG_R7		(7)
#define PMU_FALCON_REG_R8		(8)
#define PMU_FALCON_REG_R9		(9)
#define PMU_FALCON_REG_R10		(10)
#define PMU_FALCON_REG_R11		(11)
#define PMU_FALCON_REG_R12		(12)
#define PMU_FALCON_REG_R13		(13)
#define PMU_FALCON_REG_R14		(14)
#define PMU_FALCON_REG_R15		(15)
#define PMU_FALCON_REG_IV0		(16)
#define PMU_FALCON_REG_IV1		(17)
#define PMU_FALCON_REG_UNDEFINED	(18)
#define PMU_FALCON_REG_EV		(19)
#define PMU_FALCON_REG_SP		(20)
#define PMU_FALCON_REG_PC		(21)
#define PMU_FALCON_REG_IMB		(22)
#define PMU_FALCON_REG_DMB		(23)
#define PMU_FALCON_REG_CSW		(24)
#define PMU_FALCON_REG_CCR		(25)
#define PMU_FALCON_REG_SEC		(26)
#define PMU_FALCON_REG_CTX		(27)
#define PMU_FALCON_REG_EXCI		(28)
#define PMU_FALCON_REG_RSVD0		(29)
#define PMU_FALCON_REG_RSVD1		(30)
#define PMU_FALCON_REG_RSVD2		(31)
#define PMU_FALCON_REG_SIZE		(32)

/* Choices for pmu_state */
#define PMU_STATE_OFF			0 /* PMU is off */
#define PMU_STATE_STARTING		1 /* PMU is on, but not booted */
#define PMU_STATE_INIT_RECEIVED		2 /* PMU init message received */
#define PMU_STATE_ELPG_BOOTING		3 /* PMU is booting */
#define PMU_STATE_ELPG_BOOTED		4 /* ELPG is initialized */
#define PMU_STATE_LOADING_PG_BUF	5 /* Loading PG buf */
#define PMU_STATE_LOADING_ZBC		6 /* Loading ZBC buf */
#define PMU_STATE_STARTED		7 /* Fully unitialized */



struct pmu_gk20a {

	union {
		struct pmu_ucode_desc *desc;
		struct pmu_ucode_desc_v1 *desc_v1;
	};
	struct mem_desc ucode;

	struct mem_desc pg_buf;
	/* TBD: remove this if ZBC seq is fixed */
	struct mem_desc seq_buf;
	struct mem_desc trace_buf;
	struct mem_desc wpr_buf;
	bool buf_loaded;

	struct pmu_sha1_gid gid_info;

	struct pmu_queue queue[PMU_QUEUE_COUNT];

	struct pmu_sequence *seq;
	unsigned long pmu_seq_tbl[PMU_SEQ_TBL_SIZE];
	u32 next_seq_desc;

	struct pmu_mutex *mutex;
	u32 mutex_cnt;

	struct nvgpu_mutex pmu_copy_lock;
	struct nvgpu_mutex pmu_seq_lock;

	struct nvgpu_allocator dmem;

	u32 *ucode_image;
	bool pmu_ready;

	u32 zbc_save_done;

	u32 stat_dmem_offset[PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE];

	u32 elpg_stat;

	u32 mscg_stat;
	u32 mscg_transition_state;

	int pmu_state;

#define PMU_ELPG_ENABLE_ALLOW_DELAY_MSEC	1 /* msec */
	struct work_struct pg_init;
	struct nvgpu_mutex pg_mutex; /* protect pg-RPPG/MSCG enable/disable */
	struct nvgpu_mutex elpg_mutex; /* protect elpg enable/disable */
	int elpg_refcnt; /* disable -1, enable +1, <=0 elpg disabled, > 0 elpg enabled */

	union {
		struct pmu_perfmon_counter_v2 perfmon_counter_v2;
		struct pmu_perfmon_counter_v0 perfmon_counter_v0;
	};
	u32 perfmon_state_id[PMU_DOMAIN_GROUP_NUM];

	bool initialized;

	void (*remove_support)(struct pmu_gk20a *pmu);
	bool sw_ready;
	bool perfmon_ready;

	u32 sample_buffer;
	u32 load_shadow;
	u32 load_avg;

	struct nvgpu_mutex isr_mutex;
	bool isr_enabled;

	bool zbc_ready;
	union {
		struct pmu_cmdline_args_v0 args_v0;
		struct pmu_cmdline_args_v1 args_v1;
		struct pmu_cmdline_args_v2 args_v2;
		struct pmu_cmdline_args_v3 args_v3;
		struct pmu_cmdline_args_v4 args_v4;
		struct pmu_cmdline_args_v5 args_v5;
	};
	unsigned long perfmon_events_cnt;
	bool perfmon_sampling_enabled;
	u8 pmu_mode; /*Added for GM20b, and ACR*/
	u32 falcon_id;
	u32 aelpg_param[5];
	u32 override_done;

	const struct firmware *fw;
};

int gk20a_init_pmu_support(struct gk20a *g);
int gk20a_init_pmu_bind_fecs(struct gk20a *g);

void gk20a_pmu_isr(struct gk20a *g);

/* send a cmd to pmu */
int gk20a_pmu_cmd_post(struct gk20a *g, struct pmu_cmd *cmd, struct pmu_msg *msg,
		struct pmu_payload *payload, u32 queue_id,
		pmu_callback callback, void* cb_param,
		u32 *seq_desc, unsigned long timeout);

int gk20a_pmu_enable_elpg(struct gk20a *g);
int gk20a_pmu_disable_elpg(struct gk20a *g);
int gk20a_pmu_pg_global_enable(struct gk20a *g, u32 enable_pg);

u32 gk20a_pmu_pg_engines_list(struct gk20a *g);
u32 gk20a_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);

void gk20a_pmu_save_zbc(struct gk20a *g, u32 entries);

int gk20a_pmu_perfmon_enable(struct gk20a *g, bool enable);

int pmu_mutex_acquire(struct pmu_gk20a *pmu, u32 id, u32 *token);
int pmu_mutex_release(struct pmu_gk20a *pmu, u32 id, u32 *token);
int gk20a_pmu_destroy(struct gk20a *g);
int gk20a_pmu_load_norm(struct gk20a *g, u32 *load);
int gk20a_pmu_load_update(struct gk20a *g);
int gk20a_pmu_debugfs_init(struct device *dev);
void gk20a_pmu_reset_load_counters(struct gk20a *g);
void gk20a_pmu_get_load_counters(struct gk20a *g, u32 *busy_cycles,
		u32 *total_cycles);
void gk20a_init_pmu_ops(struct gpu_ops *gops);

void pmu_copy_to_dmem(struct pmu_gk20a *pmu,
		u32 dst, u8 *src, u32 size, u8 port);
void pmu_copy_from_dmem(struct pmu_gk20a *pmu,
		u32 src, u8 *dst, u32 size, u8 port);
int pmu_reset(struct pmu_gk20a *pmu);
int pmu_bootstrap(struct pmu_gk20a *pmu);
int gk20a_init_pmu(struct pmu_gk20a *pmu);
void pmu_dump_falcon_stats(struct pmu_gk20a *pmu);
void gk20a_remove_pmu_support(struct pmu_gk20a *pmu);
void pmu_setup_hw(struct work_struct *work);
void pmu_seq_init(struct pmu_gk20a *pmu);

int gk20a_init_pmu(struct pmu_gk20a *pmu);

int gk20a_pmu_ap_send_command(struct gk20a *g,
		union pmu_ap_cmd *p_ap_cmd, bool b_block);
int gk20a_aelpg_init(struct gk20a *g);
int gk20a_aelpg_init_and_enable(struct gk20a *g, u8 ctrl_id);
void pmu_enable_irq(struct pmu_gk20a *pmu, bool enable);
int pmu_wait_message_cond(struct pmu_gk20a *pmu, u32 timeout_ms,
				 u32 *var, u32 val);
void pmu_handle_fecs_boot_acr_msg(struct gk20a *g, struct pmu_msg *msg,
				void *param, u32 handle, u32 status);
void gk20a_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
int gk20a_pmu_reset(struct gk20a *g);
int pmu_idle(struct pmu_gk20a *pmu);
int pmu_enable_hw(struct pmu_gk20a *pmu, bool enable);

void gk20a_pmu_surface_free(struct gk20a *g, struct mem_desc *mem);
void gk20a_pmu_surface_describe(struct gk20a *g, struct mem_desc *mem,
		struct flcn_mem_desc_v0 *fb);
int gk20a_pmu_vidmem_surface_alloc(struct gk20a *g, struct mem_desc *mem,
		u32 size);
int gk20a_pmu_sysmem_surface_alloc(struct gk20a *g, struct mem_desc *mem,
		u32 size);

#endif /*__PMU_GK20A_H__*/
