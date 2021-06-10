/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
/*
 * Function naming determines intended use:
 *
 *     <x>_r(void) : Returns the offset for register <x>.
 *
 *     <x>_o(void) : Returns the offset for element <x>.
 *
 *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
 *
 *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
 *
 *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
 *         and masked to place it at field <y> of register <x>.  This value
 *         can be |'d with others to produce a full register value for
 *         register <x>.
 *
 *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
 *         value can be ~'d and then &'d to clear the value of field <y> for
 *         register <x>.
 *
 *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
 *         to place it at field <y> of register <x>.  This value can be |'d
 *         with others to produce a full register value for <x>.
 *
 *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
 *         <x> value 'r' after being shifted to place its LSB at bit 0.
 *         This value is suitable for direct comparison with other unshifted
 *         values appropriate for use in field <y> of register <x>.
 *
 *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
 *         field <y> of register <x>.  This value is suitable for direct
 *         comparison with unshifted values appropriate for use in field <y>
 *         of register <x>.
 */
#ifndef _hw_fb_gm206_h_
#define _hw_fb_gm206_h_

static inline u32 fb_fbhub_num_active_ltcs_r(void)
{
	return 0x00100800;
}
static inline u32 fb_mmu_ctrl_r(void)
{
	return 0x00100c80;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_f(u32 v)
{
	return (v & 0x1) << 0;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_128kb_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_ctrl_vm_pg_size_64kb_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_v(u32 r)
{
	return (r >> 15) & 0x1;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_ctrl_pri_fifo_space_v(u32 r)
{
	return (r >> 16) & 0xff;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_v(u32 r)
{
	return (r >> 11) & 0x1;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_true_f(void)
{
	return 0x800;
}
static inline u32 fb_mmu_ctrl_use_pdb_big_page_size_false_f(void)
{
	return 0x0;
}
static inline u32 fb_priv_mmu_phy_secure_r(void)
{
	return 0x00100ce4;
}
static inline u32 fb_mmu_invalidate_pdb_r(void)
{
	return 0x00100cb8;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_sys_mem_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_invalidate_pdb_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_invalidate_r(void)
{
	return 0x00100cbc;
}
static inline u32 fb_mmu_invalidate_all_va_true_f(void)
{
	return 0x1;
}
static inline u32 fb_mmu_invalidate_all_pdb_true_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_invalidate_trigger_s(void)
{
	return 1;
}
static inline u32 fb_mmu_invalidate_trigger_f(u32 v)
{
	return (v & 0x1) << 31;
}
static inline u32 fb_mmu_invalidate_trigger_m(void)
{
	return 0x1 << 31;
}
static inline u32 fb_mmu_invalidate_trigger_v(u32 r)
{
	return (r >> 31) & 0x1;
}
static inline u32 fb_mmu_invalidate_trigger_true_f(void)
{
	return 0x80000000;
}
static inline u32 fb_mmu_debug_wr_r(void)
{
	return 0x00100cc8;
}
static inline u32 fb_mmu_debug_wr_aperture_s(void)
{
	return 2;
}
static inline u32 fb_mmu_debug_wr_aperture_f(u32 v)
{
	return (v & 0x3) << 0;
}
static inline u32 fb_mmu_debug_wr_aperture_m(void)
{
	return 0x3 << 0;
}
static inline u32 fb_mmu_debug_wr_aperture_v(u32 r)
{
	return (r >> 0) & 0x3;
}
static inline u32 fb_mmu_debug_wr_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_coh_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(void)
{
	return 0x3;
}
static inline u32 fb_mmu_debug_wr_vol_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_wr_vol_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_debug_wr_vol_true_f(void)
{
	return 0x4;
}
static inline u32 fb_mmu_debug_wr_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_debug_wr_addr_alignment_v(void)
{
	return 0x0000000c;
}
static inline u32 fb_mmu_debug_rd_r(void)
{
	return 0x00100ccc;
}
static inline u32 fb_mmu_debug_rd_aperture_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_coh_f(void)
{
	return 0x2;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_ncoh_f(void)
{
	return 0x3;
}
static inline u32 fb_mmu_debug_rd_vol_false_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_debug_rd_addr_f(u32 v)
{
	return (v & 0xfffffff) << 4;
}
static inline u32 fb_mmu_debug_rd_addr_alignment_v(void)
{
	return 0x0000000c;
}
static inline u32 fb_mmu_debug_ctrl_r(void)
{
	return 0x00100cc4;
}
static inline u32 fb_mmu_debug_ctrl_debug_v(u32 r)
{
	return (r >> 16) & 0x1;
}
static inline u32 fb_mmu_debug_ctrl_debug_m(void)
{
	return 0x1 << 16;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_f(void)
{
	return 0x10000;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_f(void)
{
	return 0x0;
}
static inline u32 fb_mmu_vpr_info_r(void)
{
	return 0x00100cd0;
}
static inline u32 fb_mmu_vpr_info_index_f(u32 v)
{
	return (v & 0x3) << 0;
}
static inline u32 fb_mmu_vpr_info_index_v(u32 r)
{
	return (r >> 0) & 0x3;
}
static inline u32 fb_mmu_vpr_info_index_addr_lo_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_vpr_info_index_addr_hi_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_vpr_info_index_cya_lo_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_vpr_info_index_cya_hi_v(void)
{
	return 0x00000003;
}
static inline u32 fb_mmu_vpr_info_fetch_f(u32 v)
{
	return (v & 0x1) << 2;
}
static inline u32 fb_mmu_vpr_info_fetch_v(u32 r)
{
	return (r >> 2) & 0x1;
}
static inline u32 fb_mmu_vpr_info_fetch_false_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_vpr_info_fetch_true_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_wpr_info_r(void)
{
	return 0x00100cd4;
}
static inline u32 fb_mmu_wpr_info_index_f(u32 v)
{
	return (v & 0xf) << 0;
}
static inline u32 fb_mmu_wpr_info_index_allow_read_v(void)
{
	return 0x00000000;
}
static inline u32 fb_mmu_wpr_info_index_allow_write_v(void)
{
	return 0x00000001;
}
static inline u32 fb_mmu_wpr_info_index_wpr1_addr_lo_v(void)
{
	return 0x00000002;
}
static inline u32 fb_mmu_wpr_info_index_wpr1_addr_hi_v(void)
{
	return 0x00000003;
}
static inline u32 fb_mmu_wpr_info_index_wpr2_addr_lo_v(void)
{
	return 0x00000004;
}
static inline u32 fb_mmu_wpr_info_index_wpr2_addr_hi_v(void)
{
	return 0x00000005;
}
static inline u32 fb_niso_flush_sysmem_addr_r(void)
{
	return 0x00100c10;
}
#endif
