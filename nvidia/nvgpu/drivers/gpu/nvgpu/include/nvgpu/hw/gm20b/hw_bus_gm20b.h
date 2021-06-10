/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_bus_gm20b_h_
#define _hw_bus_gm20b_h_

static inline u32 bus_bar0_window_r(void)
{
	return 0x00001700;
}
static inline u32 bus_bar0_window_base_f(u32 v)
{
	return (v & 0xffffff) << 0;
}
static inline u32 bus_bar0_window_target_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 bus_bar0_window_target_sys_mem_coherent_f(void)
{
	return 0x2000000;
}
static inline u32 bus_bar0_window_target_sys_mem_noncoherent_f(void)
{
	return 0x3000000;
}
static inline u32 bus_bar0_window_target_bar0_window_base_shift_v(void)
{
	return 0x00000010;
}
static inline u32 bus_bar1_block_r(void)
{
	return 0x00001704;
}
static inline u32 bus_bar1_block_ptr_f(u32 v)
{
	return (v & 0xfffffff) << 0;
}
static inline u32 bus_bar1_block_target_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 bus_bar1_block_target_sys_mem_coh_f(void)
{
	return 0x20000000;
}
static inline u32 bus_bar1_block_target_sys_mem_ncoh_f(void)
{
	return 0x30000000;
}
static inline u32 bus_bar1_block_mode_virtual_f(void)
{
	return 0x80000000;
}
static inline u32 bus_bar2_block_r(void)
{
	return 0x00001714;
}
static inline u32 bus_bar2_block_ptr_f(u32 v)
{
	return (v & 0xfffffff) << 0;
}
static inline u32 bus_bar2_block_target_vid_mem_f(void)
{
	return 0x0;
}
static inline u32 bus_bar2_block_target_sys_mem_coh_f(void)
{
	return 0x20000000;
}
static inline u32 bus_bar2_block_target_sys_mem_ncoh_f(void)
{
	return 0x30000000;
}
static inline u32 bus_bar2_block_mode_virtual_f(void)
{
	return 0x80000000;
}
static inline u32 bus_bar1_block_ptr_shift_v(void)
{
	return 0x0000000c;
}
static inline u32 bus_bar2_block_ptr_shift_v(void)
{
	return 0x0000000c;
}
static inline u32 bus_bind_status_r(void)
{
	return 0x00001710;
}
static inline u32 bus_bind_status_bar1_pending_v(u32 r)
{
	return (r >> 0) & 0x1;
}
static inline u32 bus_bind_status_bar1_pending_empty_f(void)
{
	return 0x0;
}
static inline u32 bus_bind_status_bar1_pending_busy_f(void)
{
	return 0x1;
}
static inline u32 bus_bind_status_bar1_outstanding_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 bus_bind_status_bar1_outstanding_false_f(void)
{
	return 0x0;
}
static inline u32 bus_bind_status_bar1_outstanding_true_f(void)
{
	return 0x2;
}
static inline u32 bus_bind_status_bar2_pending_v(u32 r)
{
	return (r >> 2) & 0x1;
}
static inline u32 bus_bind_status_bar2_pending_empty_f(void)
{
	return 0x0;
}
static inline u32 bus_bind_status_bar2_pending_busy_f(void)
{
	return 0x4;
}
static inline u32 bus_bind_status_bar2_outstanding_v(u32 r)
{
	return (r >> 3) & 0x1;
}
static inline u32 bus_bind_status_bar2_outstanding_false_f(void)
{
	return 0x0;
}
static inline u32 bus_bind_status_bar2_outstanding_true_f(void)
{
	return 0x8;
}
static inline u32 bus_intr_0_r(void)
{
	return 0x00001100;
}
static inline u32 bus_intr_0_pri_squash_m(void)
{
	return 0x1 << 1;
}
static inline u32 bus_intr_0_pri_fecserr_m(void)
{
	return 0x1 << 2;
}
static inline u32 bus_intr_0_pri_timeout_m(void)
{
	return 0x1 << 3;
}
static inline u32 bus_intr_en_0_r(void)
{
	return 0x00001140;
}
static inline u32 bus_intr_en_0_pri_squash_m(void)
{
	return 0x1 << 1;
}
static inline u32 bus_intr_en_0_pri_fecserr_m(void)
{
	return 0x1 << 2;
}
static inline u32 bus_intr_en_0_pri_timeout_m(void)
{
	return 0x1 << 3;
}
#endif
