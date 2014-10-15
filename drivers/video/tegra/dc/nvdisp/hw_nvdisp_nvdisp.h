/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_nvdisp_nvdisp_h_
#define _hw_nvdisp_nvdisp_h_

static inline u32 nvdisp_cmd_state_ctrl_r(void)
{
	return 0x00000041;
}
static inline u32 nvdisp_cmd_state_ctrl_general_act_req_enable_f(void)
{
	return 0x1;
}
static inline u32 nvdisp_cmd_state_ctrl_a_act_req_enable_f(void)
{
	return 0x2;
}
static inline u32 nvdisp_cmd_state_ctrl_b_act_req_enable_f(void)
{
	return 0x4;
}
static inline u32 nvdisp_cmd_state_ctrl_c_act_req_enable_f(void)
{
	return 0x8;
}
static inline u32 nvdisp_cmd_state_ctrl_d_act_req_enable_f(void)
{
	return 0x10;
}
static inline u32 nvdisp_cmd_state_ctrl_e_act_req_enable_f(void)
{
	return 0x20;
}
static inline u32 nvdisp_cmd_state_ctrl_f_act_req_enable_f(void)
{
	return 0x40;
}
static inline u32 nvdisp_cmd_state_ctrl_cursor_act_req_enable_f(void)
{
	return 0x80;
}
static inline u32 nvdisp_cmd_state_ctrl_general_update_enable_f(void)
{
	return 0x100;
}
static inline u32 nvdisp_cmd_state_ctrl_win_a_update_enable_f(void)
{
	return 0x200;
}
static inline u32 nvdisp_cmd_state_ctrl_win_b_update_enable_f(void)
{
	return 0x400;
}
static inline u32 nvdisp_cmd_state_ctrl_win_c_update_enable_f(void)
{
	return 0x800;
}
static inline u32 nvdisp_cmd_state_ctrl_win_d_update_enable_f(void)
{
	return 0x1000;
}
static inline u32 nvdisp_cmd_state_ctrl_win_e_update_enable_f(void)
{
	return 0x2000;
}
static inline u32 nvdisp_cmd_state_ctrl_win_f_update_enable_f(void)
{
	return 0x4000;
}
static inline u32 nvdisp_cmd_state_ctrl_cursor_update_enable_f(void)
{
	return 0x8000;
}
static inline u32 nvdisp_cmd_state_ctrl_common_act_req_enable_f(void)
{
	return 0x10000;
}
static inline u32 nvdisp_cmd_state_ctrl_common_act_update_enable_f(void)
{
	return 0x20000;
}
static inline u32 nvdisp_cmd_state_ctrl_host_trig_enable_f(void)
{
	return 0x1000000;
}
static inline u32 nvdisp_cmd_state_ctrl_host_trig_secure_v(void)
{
	return 0x00000000;
}
static inline u32 nvdisp_cmd_state_ctrl_gen_act_req_enable_f(void)
{
	return 0x1;
}
static inline u32 nvdisp_cmd_state_ctrl_win_act_req_range_f(u32 v)
{
	return (v & 0x1) << 1;
}
static inline u32 nvdisp_cmd_state_ctrl_win_act_req_range_v(u32 r)
{
	return (r >> 1) & 0x1;
}
static inline u32 nvdisp_cmd_disp_win_hdr_r(void)
{
	return 0x00000042;
}
static inline u32 nvdisp_cmd_int_status_r(void)
{
	return 0x00000037;
}
static inline u32 nvdisp_cmd_int_status_frame_end_f(u32 v)
{
	return (v & 0x1) << 1;
}
static inline u32 nvdisp_cmd_int_status_v_blank_f(u32 v)
{
	return (v & 0x1) << 2;
}
static inline u32 nvdisp_cmd_int_status_region_crc_f(u32 v)
{
	return (v & 0x1) << 6;
}
static inline u32 nvdisp_cmd_int_status_msf_f(u32 v)
{
	return (v & 0x1) << 12;
}
static inline u32 nvdisp_cmd_int_status_uf_f(u32 v)
{
	return (v & 0x1) << 23;
}
static inline u32 nvdisp_cmd_int_status_sd3_f(u32 v)
{
	return (v & 0x1) << 24;
}
static inline u32 nvdisp_cmd_int_status_obuf_uf_f(u32 v)
{
	return (v & 0x1) << 26;
}
static inline u32 nvdisp_cmd_int_status_rbuf_uf_f(u32 v)
{
	return (v & 0x1) << 27;
}
static inline u32 nvdisp_cmd_int_status_bbuf_uf_f(u32 v)
{
	return (v & 0x1) << 28;
}
static inline u32 nvdisp_cmd_int_status_dsc_uf_f(u32 v)
{
	return (v & 0x1) << 29;
}
static inline u32 nvdisp_state_access_r(void)
{
	return 0x00000040;
}
#endif
