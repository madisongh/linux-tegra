/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/v4l2-chip-ident.h>
#include <media/camera_common.h>
#include <media/tegra_v4l2_camera.h>

#include "tc358840.h"

#define COLOR_BAR_MODE 0
#define ENABLE_HPD_PIN 0

#define TC358840_DEFAULT_MODE     0
#define TC358840_DEFAULT_WIDTH    1920
#define TC358840_DEFAULT_HEIGHT   1080
#define TC358840_DEFAULT_DATAFMT  V4L2_MBUS_FMT_UYVY8_2X8
#define TC358840_DEFAULT_CLK_FREQ 24000000

static const struct camera_common_frmfmt tc358840_frmfmt[] = {
	{{960,  1080}, 0, 0},
	{{1920, 2160}, 0, 0},
	{{1920, 1080}, 0, 0},
	{{3840, 2160}, 0, 0},
};

static struct tc358840_reg startup_seq1[] = {
	/* SOFTWARE RESET */
	{WR16, CONFCTL0, 0x80C4},
	{WR16, SYSCTL, 0x3F01},
	{WR16, SYSCTL, 0x0000},
	{WR16, CONFCTL1, 0x0008},
	/* CSI-TX0 TRANSITION TIMING */
	{WR32, TX0CSI_TX_CLKEN, 0x00000001},
	{WR32, TX0CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX0MIPI_PLL_CNF, 0x000090E5},
	{DELAY, 0},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX0LANE_ENABLE, 0x00000014},
	{WR32, TX0LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX0HSTX_TO_COUNT, 0x00000000},
	{WR32, TX0FUNC_ENABLE, 0x00000101},
	/* REMEND, TX0CSI_LPTX_MODE, 0x00000000 */
	{WR32, TX0CSI_TATO_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX0CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX0CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX0CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX0FUNC_MODE, 0x00000400},
	{WR32, TX0CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX0IND_TO_COUNT, 0x000000C8},
	/* REMEND, TX0INIT_INT_MSK, 0x00000014 */
	{WR32, TX0CSI_HSYNC_STOP_COUNT, 0x0000002A},
	/* REMND, TX0APF_VDELAYCNT, 0x00000466 */
	/* REMEND, TX0APF_VC_CONFIG, 0x00000001 */
	{WR32, TX0CSI_RX_STATE_INT_MASK, 0x00000000},
	/* REMEND, TX0CSI_RXTRG_INT_MASK, 0x00000000 */
	{WR32, TX0CSI_LPRX_THRESH_COUNT, 0x00000015},
	/* REMEND, TX0CSI_PRTO_INT_MASK, 0x00000000 */
	{WR32, TX0APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX0CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX0CSI_LPTX_INT_MASK, 0x00000000},
	/* REMEND, TX0DPHY_DLYCNTRL, 0x00000000 */
	{WR32, TX0LPTXTIMECNT, 0x00000004},
	{WR32, TX0TCLK_HEADERCNT, 0x00180203},
	{WR32, TX0TCLK_TRAILCNT, 0x00040005},
	{WR32, TX0THS_HEADERCNT, 0x000D0004},
	{WR32, TX0TWAKEUPCNT, 0x00003E80},
	{WR32, TX0TCLK_POSTCNT, 0x0000000A},
	{WR32, TX0THS_TRAILCNT, 0x00080006},
	{WR32, TX0HSTXVREGCNT, 0x00000020},
	{WR32, TX0HSTXVREGEN, 0x0000001F},
	{WR32, TX0BTA_COUNT, 0x00040003},
	{WR32, TX0DPHY_TX_ADJUST, 0x00000002},
	/* REMEND, TX0DPHY_CAP, 0x000002AA */
	{WR32, TX0CSITX_START, 0x00000001},
	/* CSI-TX1 TRANSITION TIMING */
	{WR32, TX1CSI_TX_CLKEN, 0x00000001},
	{WR32, TX1CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX1MIPI_PLL_CNF, 0x000090E5},
	{DELAY, 0},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX1LANE_ENABLE, 0x00000014},
	{WR32, TX1LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX1HSTX_TO_COUNT, 0x00000000},
	{WR32, TX1FUNC_ENABLE, 0x00000101},
	/* REMEND, TX1CSI_LPTX_MODE, 0x00000004 */
	{WR32, TX1CSI_TATO_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX1CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX1CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX1CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX1FUNC_MODE, 0x00000400},
	{WR32, TX1CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX1IND_TO_COUNT, 0x000000C8},
	/* REMEND, TX1INIT_INT_MSK, 0x00000015 */
	{WR32, TX1CSI_HSYNC_STOP_COUNT, 0x0000002A},
	/* REMND, TX1APF_VDELAYCNT, 0x00000466 */
	/* REMEND, TX1APF_VC_CONFIG, 0x00000001 */
	{WR32, TX1CSI_RX_STATE_INT_MASK, 0x00000000},
	/* REMEND, TX1CSI_RXTRG_INT_MASK, 0x00000000 */
	{WR32, TX1CSI_LPRX_THRESH_COUNT, 0x00000015},
	/* REMEND, TX1CSI_PRTO_INT_MASK, 0x00000000 */
	{WR32, TX1APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX1CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX1CSI_LPTX_INT_MASK, 0x00000000},
	/* REMEND, TX1DPHY_DLYCNTRL, 0x00000000 */
	{WR32, TX1LPTXTIMECNT, 0x00000004},
	{WR32, TX1TCLK_HEADERCNT, 0x00180203},
	{WR32, TX1TCLK_TRAILCNT, 0x00040005},
	{WR32, TX1THS_HEADERCNT, 0x000D0004},
	{WR32, TX1TWAKEUPCNT, 0x00003E80},
	{WR32, TX1TCLK_POSTCNT, 0x0000000A},
	{WR32, TX1THS_TRAILCNT, 0x00080006},
	{WR32, TX1HSTXVREGCNT, 0x00000020},
	{WR32, TX1HSTXVREGEN, 0x0000001F},
	{WR32, TX1BTA_COUNT, 0x00040003},
	{WR32, TX1DPHY_TX_ADJUST, 0x00000002},
	/* REMEND, TX1DPHY_CAP, 0x00000000 */
	{WR32, TX1CSITX_START, 0x00000001},
	/* SYNC SIGNAL POLARITY */
	{WR32, TX0MODE_CONFIG, 0x00000006},
	{WR32, TX1MODE_CONFIG, 0x00000006},
	/* SPLIT CONTROL, VPID */
	{WR16, STX0_CTL, 0x0000},
	/* REMND, STX0_VPID1, 0x3435 */
	/* REMND, STX0_VPID2, 0x3600 */
	/* REMND, STX0_VPID3, 0x0024 */
	/* REMND, {WR16, STX0_WC, 0x0F00},*/
	{WR16, STX0_FPX, 0x8000},
	/* REMND, STX0_LPX, 077F */
	{WR16, STX1_CTL, 0x0000},
	/* REMND, STX1_VPID1, 0x3435 */
	/* REMND, STX1_VPID2, 0x3600 */
	/* REMND, STX1_VPID3, 0x0024 */
	/* REMND{WR16, STX1_WC, 0x0F00}, */
	/* REMND, STX1_FPX, 0x0000 */
	/* REMND, STX1_LPX, 077F */
	/* FRAME COUNT CONTROL */
	/* REMND, STX0_MAXFCNT, 0x0000 */
	/* REMND, STX1_MAXFCNT, 0x0000 */
	/* HDMI PHY */
	{WR8, PHY_CTL, 0x03},
	{WR8, PHY_ENB, 0x3F},
	{WR8, EQ_BYPS, 0x07},
	{WR8, APLL_CTL, 0x31},
	{WR8, DDCIO_CTL, 0x01},
	/* REM, HDMI CLOCK */
	{WR16, SYS_FREQ0, 0x0FA0},
	{WR8, LOCKDET_FREQ0, 0x80},
	{WR16, LOCKDET_REF1_2, 0x061A},
	{WR8, NCO_F0_MOD, 0x02},
	{WR16, CSC_SCLK0, 0xFA0},
	/* REM, HDMI INTERRUPT MASK, CLEAR */
	{WR8, SYS_INT, 0xFF},
	{WR8, SYS_INTM, 0xFE},
	{WR8, MISC_INT, 0xFF},
	{WR8, MISC_INTM, 0xFD},
	/* REMND, PACKET_INTM, 00 */
	/* REMND, AUDIO_IMNTM, 00 */
	/* REMND, ABUF_INTM, 00 */
	/* INTERRUPT CONTROL (TOP LEVEL) */
	{WR16, INTSTATUS, 0x0F3F},
	{WR16, INTMASK, 0x0D3F},
	{WR8, CLK_INTM, 0},
	/* EDID */
	{WR8, EDID_MODE, 0x01},
	{WR16, EDID_LEN1_2, EDID_RAM_SIZE},
	{CMD_END, 0},
};

static struct tc358840_reg startup_seq2[] = {
	/* VIDEO COLOR FORMAT SETTING */
	{WR8, VOUT_FMT, 0x01},
	{WR8, VOUT_FIL, 0x14},
	{WR8, VOUT_SYNC0, 0x42},
	{WR8, VOUT_COLOR, 0x31},
	/* HDMI SYSTEM */
	{WR8, DDC_CTL, 0x02},
	{WR8, HPD_CTL, 0x10},
	/* HDMI AUDIO SETTING */
	{WR8, AUD_AUTO_MUTE, 0x00},
	{WR8, AUTO_CMD0, 0xF3},
	{WR8, AUTO_CMD1, 0x02},
	{WR8, AUTO_CMD2, 0x0C},
	{WR8, BUFINIT_START, 0x05},
	{WR8, FS_MUTE, 0x00},
	{WR8, SDO_MODE1, 0x02},
	{WR32, NCO_48F0A_D, 0x02752546},
	{WR32, NCO_44F0A_D, 0x0242070B},
	{WR8, AUD_MODE, 0x00},
	/* LET HDMI SOURCE START ACCESS */
	{WR8, INIT_END, 0x01},
	{CMD_END, 0},
};

__maybe_unused
static struct tc358840_reg color_bar_3840_2160_seq[] = {
	{WR16, CONFCTL0, 0x80C4},
	{WR16, SYSCTL, 0x3F01},
	{WR16, SYSCTL, 0x0000},
	{WR16, CONFCTL1, 0x0000},
	{WR16, CB_CTL, 0x0009},
	/* CSI-TX1 TRANSITION TIMING */
	{WR32, TX1CSI_TX_CLKEN, 0x00000001},
	{WR32, TX1CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX1MIPI_PLL_CNF, 0x0000907C},
	{DELAY, 0},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX1LANE_ENABLE, 0x00000014},
	{WR32, TX1LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX1HSTX_TO_COUNT, 0x00000000},
	{WR32, TX1FUNC_ENABLE, 0x00000101},
	{WR32, TX1CSI_TATO_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX1CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX1CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX1CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX1FUNC_MODE, 0x00000160},
	{WR32, TX1CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX1IND_TO_COUNT, 0x000000C8},
	{WR32, TX1CSI_HSYNC_STOP_COUNT, 0x0000002A},
	{WR32, TX1APF_VDELAYCNT, 0x00000455},
	{WR32, TX1CSI_RX_STATE_INT_MASK, 0x00000000},
	{WR32, TX1CSI_LPRX_THRESH_COUNT, 0x00000015},
	{WR32, TX1APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX1CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX1CSI_LPTX_INT_MASK, 0x00000000},
	{WR32, TX1LPTXTIMECNT, 0x00000004},
	{WR32, TX1TCLK_HEADERCNT, 0x00180203},
	{WR32, TX1TCLK_TRAILCNT, 0x00040005},
	{WR32, TX1THS_HEADERCNT, 0x000D0004},
	{WR32, TX1TWAKEUPCNT, 0x00003E80},
	{WR32, TX1TCLK_POSTCNT, 0x0000000A},
	{WR32, TX1THS_TRAILCNT, 0x00080006},
	{WR32, TX1HSTXVREGCNT, 0x00000020},
	{WR32, TX1HSTXVREGEN, 0x0000001F},
	{WR32, TX1BTA_COUNT, 0x00040003},
	{WR32, TX1DPHY_TX_ADJUST, 0x00000002},
	{WR32, TX1CSITX_START, 0x00000001},
	/* CSI-TX0 TRANSITION TIMING */
	{WR32, TX0CSI_TX_CLKEN, 0x00000001},
	{WR32, TX0CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX0MIPI_PLL_CNF, 0x0000907C},
	{DELAY, 0},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX0LANE_ENABLE, 0x00000014},
	{WR32, TX0LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX0HSTX_TO_COUNT, 0x00000000},
	{WR32, TX0FUNC_ENABLE, 0x00000101},
	{WR32, TX0CSI_TATO_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX0CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX0CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX0CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX0FUNC_MODE, 0x00000160},
	{WR32, TX0CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX0IND_TO_COUNT, 0x000000C8},
	{WR32, TX0CSI_HSYNC_STOP_COUNT, 0x0000002A},
	{WR32, TX0APF_VDELAYCNT, 0x00000455},
	{WR32, TX0CSI_RX_STATE_INT_MASK, 0x00000000},
	{WR32, TX0CSI_LPRX_THRESH_COUNT, 0x00000015},
	{WR32, TX0APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX0CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX0CSI_LPTX_INT_MASK, 0x00000000},
	{WR32, TX0LPTXTIMECNT, 0x00000004},
	{WR32, TX0TCLK_HEADERCNT, 0x00180203},
	{WR32, TX0TCLK_TRAILCNT, 0x00040005},
	{WR32, TX0THS_HEADERCNT, 0x000D0004},
	{WR32, TX0TWAKEUPCNT, 0x00003E80},
	{WR32, TX0TCLK_POSTCNT, 0x0000000A},
	{WR32, TX0THS_TRAILCNT, 0x00080006},
	{WR32, TX0HSTXVREGCNT, 0x00000020},
	{WR32, TX0HSTXVREGEN, 0x0000001F},
	{WR32, TX0BTA_COUNT, 0x00040003},
	{WR32, TX0DPHY_TX_ADJUST, 0x00000002},
	{WR32, TX0CSITX_START, 0x00000001},
	/* SPLIT CONTROL */
	{WR16, STX0_CTL, 0x0000},
	{WR16, STX0_FPX, 0x8000},
	{WR16, STX1_CTL, 0x0000},
	/* COLOR BAR SETTING */
	{WR8, VOUT_FMT, 0x01},
	{WR8, VOUT_FIL, 0x14},
	{WR8, VOUT_COLOR, 0x31},
	{WR16, CB_CTL, 0x000D},
	{WR16, CB_HSW, 0x0020},
	{WR16, CB_VSW, 0x000A},
	{WR16, CB_HTOTAL, 0x0FA0},
	{WR16, CB_VTOTAL, 0x08A7},
	{WR16, CB_HACT, 0x0F00},
	{WR16, CB_VACT, 0x0870},
	{WR16, CB_HSTART, 0x0070},
	{WR16, CB_VSTART, 0x0034},
	/* START VIDEO TX */
	{WR16, CONFCTL0, 0x8CF7},
	{CMD_END, 0},
};

__maybe_unused
static struct tc358840_reg color_bar_1920_1080_seq[] = {
	{WR16, CONFCTL0, 0x80C4},
	{WR16, SYSCTL, 0x3F01},
	{WR16, SYSCTL, 0x0000},
	{WR16, CONFCTL1, 0x0000},
	{WR16, CB_CTL, 0x0009},
	/* CSI-TX1 TRANSITION TIMING */
	{WR32, TX1CSI_TX_CLKEN, 0x00000001},
	{WR32, TX1CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX1MIPI_PLL_CNF, 0x0000907C},
	{DELAY, 0},
	{WR32, TX1MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX1LANE_ENABLE, 0x00000014},
	{WR32, TX1LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX1HSTX_TO_COUNT, 0x00000000},
	{WR32, TX1FUNC_ENABLE, 0x00000101},
	{WR32, TX1CSI_TATO_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX1CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX1CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX1CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX1CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX1FUNC_MODE, 0x00000160},
	{WR32, TX1CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX1IND_TO_COUNT, 0x000000C8},
	{WR32, TX1CSI_HSYNC_STOP_COUNT, 0x0000002A},
	{WR32, TX1APF_VDELAYCNT, 0x00000630},
	{WR32, TX1CSI_RX_STATE_INT_MASK, 0x00000000},
	{WR32, TX1CSI_LPRX_THRESH_COUNT, 0x00000015},
	{WR32, TX1APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX1CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX1CSI_LPTX_INT_MASK, 0x00000000},
	{WR32, TX1LPTXTIMECNT, 0x00000004},
	{WR32, TX1TCLK_HEADERCNT, 0x00180203},
	{WR32, TX1TCLK_TRAILCNT, 0x00040005},
	{WR32, TX1THS_HEADERCNT, 0x000D0004},
	{WR32, TX1TWAKEUPCNT, 0x00003E80},
	{WR32, TX1TCLK_POSTCNT, 0x0000000A},
	{WR32, TX1THS_TRAILCNT, 0x00080006},
	{WR32, TX1HSTXVREGCNT, 0x00000020},
	{WR32, TX1HSTXVREGEN, 0x0000001F},
	{WR32, TX1BTA_COUNT, 0x00040003},
	{WR32, TX1DPHY_TX_ADJUST, 0x00000002},
	{WR32, TX1CSITX_START, 0x00000001},
	/* CSI-TX0 TRANSITION TIMING */
	{WR32, TX0CSI_TX_CLKEN, 0x00000001},
	{WR32, TX0CSI_TX_CLKSEL, 0x00000001},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000001},
	{WR32, TX0MIPI_PLL_CNF, 0x0000907C},
	{DELAY, 0},
	{WR32, TX0MIPI_PLL_CONTROL, 0x00000003},
	{WR32, TX0LANE_ENABLE, 0x00000014},
	{WR32, TX0LINE_INIT_COUNT, 0x00000FA0},
	{WR32, TX0HSTX_TO_COUNT, 0x00000000},
	{WR32, TX0FUNC_ENABLE, 0x00000101},
	{WR32, TX0CSI_TATO_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_BTA_COUNT, 0x00005000},
	{WR32, TX0CSI_PRESP_LPR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_LPW_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSR_COUNT, 0x00010000},
	{WR32, TX0CSI_PRESP_HSW_COUNT, 0x00010000},
	{WR32, TX0CSI_PR_TO_COUNT, 0x00001000},
	{WR32, TX0CSI_LRX_H_TO_COUNT, 0x00010000},
	{WR32, TX0FUNC_MODE, 0x00000160},
	{WR32, TX0CSI_RX_VC_ENABLE, 0x00000001},
	{WR32, TX0IND_TO_COUNT, 0x000000C8},
	{WR32, TX0CSI_HSYNC_STOP_COUNT, 0x0000002A},
	{WR32, TX0APF_VDELAYCNT, 0x00000630},
	{WR32, TX0CSI_RX_STATE_INT_MASK, 0x00000000},
	{WR32, TX0CSI_LPRX_THRESH_COUNT, 0x00000015},
	{WR32, TX0APP_SIDE_ERR_INT_MASK, 0x00000000},
	{WR32, TX0CSI_RX_ERR_INT_MASK, 0x00000080},
	{WR32, TX0CSI_LPTX_INT_MASK, 0x00000000},
	{WR32, TX0LPTXTIMECNT, 0x00000004},
	{WR32, TX0TCLK_HEADERCNT, 0x00180203},
	{WR32, TX0TCLK_TRAILCNT, 0x00040005},
	{WR32, TX0THS_HEADERCNT, 0x000D0004},
	{WR32, TX0TWAKEUPCNT, 0x00003E80},
	{WR32, TX0TCLK_POSTCNT, 0x0000000A},
	{WR32, TX0THS_TRAILCNT, 0x00080006},
	{WR32, TX0HSTXVREGCNT, 0x00000020},
	{WR32, TX0HSTXVREGEN, 0x0000001F},
	{WR32, TX0BTA_COUNT, 0x00040003},
	{WR32, TX0DPHY_TX_ADJUST, 0x00000002},
	{WR32, TX0CSITX_START, 0x00000001},
	/* SPLIT CONTROL */
	{WR16, STX0_CTL, 0x0000},
	{WR16, STX0_FPX, 0x8000},
	{WR16, STX1_CTL, 0x0000},
	/* COLOR BAR SETTING */
	{WR8, VOUT_FMT, 0x01},
	{WR8, VOUT_FIL, 0x14},
	{WR8, VOUT_COLOR, 0x31},
	{WR16, CB_CTL, 0x000D},
	{WR16, CB_HSW, 0x0020},
	{WR16, CB_VSW, 0x000A},
	{WR16, CB_HTOTAL, 0x0820},
	{WR16, CB_VTOTAL, 0x046F},
	{WR16, CB_HACT, 0x0780},
	{WR16, CB_VACT, 0x0438},
	{WR16, CB_HSTART, 0x0070},
	{WR16, CB_VSTART, 0x0034},
	/* START VIDEO TX */
	{WR16, CONFCTL0, 0x8CF7},
	{DELAY, 0},
	{CMD_END, 0},
};

static u8 edid[] = {
0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x52, 0x62, 0x88, 0x88, 0x00, 0x88, 0x88, 0x88,
0x1C, 0x15, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xEC, 0x68,
0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80, 0x30, 0x20,
0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C,
0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
0x58, 0x2C, 0x45, 0x00, 0xC4, 0x8E, 0x21, 0x00,
0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
0x6F, 0x73, 0x68, 0x69, 0x62, 0x61, 0x2D, 0x55,
0x48, 0x32, 0x44, 0x0A, 0x00, 0x00, 0x00, 0xFD,
0x00, 0x17, 0x3D, 0x0F, 0x8C, 0x17, 0x00, 0x0A,
0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xAF,
0x02, 0x03, 0x1A, 0x74, 0x47, 0x32, 0x10, 0x32,
0x32, 0x32, 0x32, 0x32, 0x23, 0x09, 0x07, 0x01,
0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00,
0x10, 0x00, 0xEC, 0x68, 0x00, 0xA0, 0xF0, 0x70,
0x37, 0x80, 0x30, 0x20, 0x3A, 0x00, 0x00, 0x70,
0xF8, 0x00, 0x00, 0x1C, 0xEC, 0x68, 0x00, 0xA0,
0xF0, 0x70, 0x37, 0x80, 0x30, 0x20, 0x3A, 0x00,
0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C, 0xEC, 0x68,
0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80, 0x30, 0x20,
0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00, 0x00, 0x1C,
0xEC, 0x68, 0x00, 0xA0, 0xF0, 0x70, 0x37, 0x80,
0x30, 0x20, 0x3A, 0x00, 0x00, 0x70, 0xF8, 0x00,
0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4A,
};

static int tc358840_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops tc358840_ctrl_ops = {
	.s_ctrl		= tc358840_s_ctrl,
};

static const s64 gang_mode_ctrl_qmenu[] = {
	CAMERA_GANG_DISABLED,
	CAMERA_GANG_L_R,
	CAMERA_GANG_R_L,
	CAMERA_GANG_T_B,
	CAMERA_GANG_B_T,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &tc358840_ctrl_ops,
		.id = V4L2_CID_GANG_MODE,
		.name = "Gang mode",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 1,
		.max = 4,
		.menu_skip_mask = 0,
		.def = 1,
		.qmenu_int = gang_mode_ctrl_qmenu,
	},
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

/* TC358840XBG requires register address in big endian
 * and register value in little endian.
 * Regmap currently sends it all in big endian
 */
__maybe_unused
static int tc358840_read_reg8(struct tc358840 *priv,
				unsigned int addr, unsigned int *val)
{
	regmap_read(priv->regmap, addr, val);
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, *val);
	return *val;
}

__maybe_unused
static int tc358840_read_reg16(struct tc358840 *priv,
				unsigned int addr, unsigned int *val)
{
	regmap_raw_read(priv->regmap, addr, val, 2);
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, *val);
	return *val;
}

__maybe_unused
static int tc358840_read_reg32(struct tc358840 *priv,
				unsigned int addr, unsigned int *val)
{
	regmap_raw_read(priv->regmap, addr, val, 4);
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, *val);
	return *val;
}

static int tc358840_write_reg8(struct tc358840 *priv,
				unsigned int addr, unsigned int val)
{
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, val);
	return regmap_write(priv->regmap, addr, val);
}

static int tc358840_write_reg16(struct tc358840 *priv,
				unsigned int addr, unsigned int val)
{
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, val);
	return regmap_raw_write(priv->regmap, addr, &val, 2);
}

static int tc358840_write_reg32(struct tc358840 *priv,
				unsigned int addr, unsigned int val)
{
	dev_dbg(&priv->i2c_client->dev, "%#x = %#x\n", addr, val);
	return regmap_raw_write(priv->regmap, addr, &val, 4);
}

static int tc358840_write_table(
	struct tc358840 *priv,
	const struct tc358840_reg table[])
{
	int val, err = 0;
	const struct tc358840_reg *next;
	int (*reg_write)(struct tc358840 *priv,
			unsigned int addr, unsigned int val);

	for (next = table; next->cmd_type != CMD_END; next++) {
		switch (next->cmd_type) {
		case DELAY:
			msleep(1);
			continue;
		case WR8:
			reg_write = tc358840_write_reg8;
			break;
		case WR16:
			reg_write = tc358840_write_reg16;
			break;
		case WR32:
			reg_write = tc358840_write_reg32;
			break;
		default:
			pr_err("%s: cmd type not supported!\n", __func__);
			return -EINVAL;
		}

		val = next->val;
		err = reg_write(priv, next->addr, val);

		if (err)
			return err;
	}
	return 0;
}

static int program_edid_ram(struct tc358840 *priv, u8 *edid)
{
	int i, err = 0;
	unsigned int val;

	for (i = 0; i < EDID_RAM_SIZE; i++) {
		val = edid[i];
		err = tc358840_write_reg8(priv, EDID_RAM_START + i, val);
		if (err) {
			dev_info(&priv->i2c_client->dev,
				"%s: write error@%#x!\n", __func__,
				EDID_RAM_START + i);
			return err;
		}
	}
	return 0;
}

static void run_init_seq(struct tc358840 *priv)
{
	/* step1: program startup sequence 1/2*/
	tc358840_write_table(priv, startup_seq1);

	/* step2: program EDID ram regs */
	program_edid_ram(priv, edid);

	/* step3: program startup sequence 2/2*/
	tc358840_write_table(priv, startup_seq2);
}

static int sync_hdmi(struct tc358840 *priv)
{
	int retry, max_try = 30, ret = 0;
	unsigned int sys_status;

	/* step4: wait HDMI sync */
	for (retry = 0; retry < max_try; retry++) {
		/* need to wait bit 7 of SYS_STATUS reg */
		tc358840_read_reg8(priv, SYS_STATUS, &sys_status);
		if (sys_status & 0x80)
			break;

		msleep(100);
	}

	if (retry == max_try) {
		dev_err(&priv->i2c_client->dev,
			"%s: fail to wait HDMI sync!\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static void start_csi_tx(struct tc358840 *priv)
{
	/* step5: start video TX*/
	tc358840_write_reg16(priv, CONFCTL0, 0x8CF7);
	tc358840_write_reg16(priv, CONFCTL1, 0x0000);
}

__maybe_unused
static void set_csi_split(struct tc358840 *priv,
	enum splitter_type split)
{
	u16 fpx, ctl;

	priv->split.mode = split;
	tc358840_read_reg16(priv, STX0_FPX, &fpx);

	switch (split) {
	case LEFT_RIGHT:
		ctl = (0x0 << 8);
		fpx |= (0x0 << 14);
		fpx |= (0x1 << 15);
		priv->split.tx0.first_pixel = 0;
		priv->split.tx0.len = priv->input.hactive / 2;
		priv->split.tx1.first_pixel = (priv->input.hactive / 2) - 1;
		priv->split.tx1.len = priv->input.hactive / 2;
		break;
	case RIGHT_LEFT:
		ctl = (0x0 << 8);
		fpx |= (0x1 << 14);
		fpx |= (0x1 << 15);
		priv->split.tx1.first_pixel = 0;
		priv->split.tx1.len = priv->input.hactive / 2;
		priv->split.tx0.first_pixel = (priv->input.hactive / 2) - 1;
		priv->split.tx0.len = priv->input.hactive / 2;
		break;
	case MANUAL:
		ctl = (0x0 << 8);
		fpx |= (0x0 << 15);
		break;
	case DISABLE:
		ctl = (0x1 << 8);
		priv->split.tx0.first_pixel = 0;
		priv->split.tx0.len = priv->input.hactive;
		priv->split.tx1.first_pixel = 0;
		priv->split.tx1.len = priv->input.hactive;
		break;
	default:
		dev_err(&priv->i2c_client->dev,
			"%s: invalid csi split mode!\n", __func__);
		break;
	}

	tc358840_write_reg16(priv, STX0_CTL, ctl);
	tc358840_write_reg16(priv, STX1_CTL, ctl);
	tc358840_write_reg16(priv, STX0_FPX, fpx);
}

/*
 * offset: start from 0
 */
static void set_csi_window(struct tc358840 *priv,
	int index, int offset, int len)
{
	int fist_pixel, last_pixel;

	/* auto split by default, set to manual */
	set_csi_split(priv, MANUAL);

	fist_pixel = offset & 0x0FFF;
	last_pixel = (offset + len) & 0x0FFF;

	if (index == 0) {
		priv->split.tx0.first_pixel = fist_pixel;
		priv->split.tx0.len = last_pixel - fist_pixel + 1;
		tc358840_write_reg16(priv, STX0_FPX, fist_pixel);
		tc358840_write_reg16(priv, STX0_LPX, last_pixel);
	} else if (index == 1) {
		priv->split.tx1.first_pixel = fist_pixel;
		priv->split.tx1.len = last_pixel - fist_pixel + 1;
		tc358840_write_reg16(priv, STX1_FPX, fist_pixel);
		tc358840_write_reg16(priv, STX1_LPX, last_pixel);
	} else
		dev_err(&priv->i2c_client->dev,
			"%s: invalid csi index!\n", __func__);
}

__maybe_unused
static void stop_tx(struct tc358840 *priv)
{
	/* Disable audio and video TX */
	tc358840_write_reg16(priv, CONFCTL0, 0x8CD4);
}

__maybe_unused
static void tc358840_dbg_print(struct tc358840 *priv,
	u8 type, unsigned int addr, char *reg)
{
	unsigned int val;
	int (*reg_read)(struct tc358840 *priv,
			unsigned int addr, unsigned int *val);

	if (type == RD8)
		reg_read = tc358840_read_reg8;
	else if (type == RD16)
		reg_read = tc358840_read_reg16;
	else
		return;

	reg_read(priv, addr, &val);
	dev_info(&priv->i2c_client->dev, "reg %s = %#x\n", reg, val);
}

#if defined DEBUG
#define dbg_print(priv, type, reg) \
	tc358840_dbg_print(priv, type, reg, #reg)
#else
#define dbg_print(priv, type, reg)
#endif

static void tc358840_hdmiin_timing_chk(struct tc358840 *priv)
{
	dbg_print(priv, RD8, SYS_FREQ0);
	dbg_print(priv, RD8, SYS_FREQ1);
	dbg_print(priv, RD8, CSC_SCLK0);
	dbg_print(priv, RD8, CSC_SCLK1);

	/* HDMI-In video timing check*/
	/* PCLK */
	dbg_print(priv, RD8, PX_FREQ0);
	dbg_print(priv, RD8, PX_FREQ1);
	/* Horizontal */
	dbg_print(priv, RD8, H_SIZE0);
	dbg_print(priv, RD8, H_SIZE1);
	dbg_print(priv, RD8, DE_HPOS0);
	dbg_print(priv, RD8, DE_HPOS1);
	dbg_print(priv, RD8, DE_HWID0);
	dbg_print(priv, RD8, DE_HWID1);
	/* Vertical */
	dbg_print(priv, RD8, V_SIZE0);
	dbg_print(priv, RD8, V_SIZE1);
	dbg_print(priv, RD8, DE_POS_A0);
	dbg_print(priv, RD8, DE_POS_A1);
	dbg_print(priv, RD8, DE_POS_B0);
	dbg_print(priv, RD8, DE_POS_B1);
	dbg_print(priv, RD8, DE_VWID0);
	dbg_print(priv, RD8, DE_VWID1);
	/* Vsync, Hsync porarity */
	dbg_print(priv, RD8, CLK_STATUS);

	dbg_print(priv, RD8, SYS_STATUS);
	dbg_print(priv, RD8, VI_STATUS0);
	dbg_print(priv, RD8, VI_STATUS1);
	dbg_print(priv, RD8, VI_STATUS2);
	dbg_print(priv, RD8, VI_STATUS3);
}

static int tc358840_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct camera_common_data *s_data =
		container_of(sd, struct camera_common_data, subdev);
	struct tc358840 *priv = (struct tc358840 *)s_data->priv;
	const struct camera_common_colorfmt *fmt = s_data->colorfmt;
	int err = 0;

	dev_dbg(&priv->i2c_client->dev, "%s++\n", __func__);

	if (!enable) {
		/* TODO: stop the stream here */
		return 0;
	}

#if COLOR_BAR_MODE
	if (s_data->fmt_height == 1080)
		tc358840_write_table(priv, color_bar_1920_1080_seq);
	else
		tc358840_write_table(priv, color_bar_3840_2160_seq);
#endif

	switch (fmt->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		tc358840_write_reg16(priv, CONFCTL0, 0x8CF7);
		tc358840_write_reg8(priv, VOUT_FMT, 0x01);
		tc358840_write_reg8(priv, VOUT_COLOR, 0x31);
		break;
	case V4L2_MBUS_FMT_RGBA8888_4X8_LE:
		tc358840_write_reg16(priv, CONFCTL0, 0x8C37);
		tc358840_write_reg8(priv, VOUT_FMT, 0x02);
		tc358840_write_reg8(priv, VOUT_COLOR, 0x11);
		break;
	default:
		dev_err(&priv->i2c_client->dev,
			"%s: invalid pixel format %08x!\n",
			__func__, fmt->code);
		break;
	}

	return 0;
exit:
	dev_dbg(&priv->i2c_client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int tc358840_s_power(struct v4l2_subdev *sd, int on)
{
	struct camera_common_data *s_data =
		container_of(sd, struct camera_common_data, subdev);
	struct tc358840 *priv = (struct tc358840 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power %s\n",
			__func__, on ? "on" : "off");

	if (on) {
		if (pw->dvdd)
			regulator_enable(pw->dvdd);
		if (pw->iovdd)
			regulator_enable(pw->iovdd);
	} else {
		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
	}

	return 0;
}

static struct v4l2_subdev_video_ops tc358840_subdev_video_ops = {
	.s_stream	= tc358840_s_stream,
	.s_mbus_fmt	= camera_common_s_fmt,
	.g_mbus_fmt	= camera_common_g_fmt,
	.try_mbus_fmt	= camera_common_try_fmt,
	.enum_mbus_fmt	= camera_common_enum_fmt,
	.g_mbus_config	= camera_common_g_mbus_config,
};

static struct v4l2_subdev_core_ops tc358840_subdev_core_ops = {
	.g_chip_ident	= camera_common_g_chip_ident,
	.s_power	= tc358840_s_power,
};

static struct v4l2_subdev_ops tc358840_subdev_ops = {
	.core	= &tc358840_subdev_core_ops,
	.video	= &tc358840_subdev_video_ops,
};

static void tc358840_check_id(struct tc358840 *priv)
{
	unsigned int chip_rev_id;

	tc358840_read_reg16(priv, 0x0000, &chip_rev_id);
	dev_info(&priv->i2c_client->dev, "chip id %x, rev id %x\n",
		(chip_rev_id >> 8) & 0xFF, chip_rev_id & 0xFF);
}

static irqreturn_t hpd_irq_handler(int irq, void *data)
{
	struct tc358840 *priv = (struct tc358840 *)data;
	int hpd_state = gpio_get_value(priv->hpd_gpio);

	/* hotplug pin is low active */
	hpd_state = !hpd_state;

#if !COLOR_BAR_MODE
	if (hpd_state) {
		/*
		 * FIXME: Very eash to hit csi syncpt timeout
		 * if not redo the init sequence.
		 */
		dev_info(&priv->i2c_client->dev,
			"hot plug detected, doing init...\n");
		run_init_seq(priv);
	} else {
		dev_info(&priv->i2c_client->dev,
			"hot unplug detected, stoping tx...\n");
		stop_tx(priv);
	}
#endif

#ifdef CONFIG_SWITCH
	switch_set_state(&priv->hpd_switch, hpd_state);
#endif
	return IRQ_HANDLED;
}

static int tc358840_pwr_init(struct tc358840 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_power_rail *pw = &priv->power;
	int err;

	err = camera_common_regulator_get(client, &pw->iovdd, "vif");
	if (err < 0) {
		dev_err(&client->dev, "cannot get regulator vif %d\n", err);
		return -EINVAL;
	}

	err = camera_common_regulator_get(client, &pw->dvdd, "vdig");
	if (err < 0) {
		dev_err(&client->dev, "cannot get regulator vdig %d\n", err);
		return -EINVAL;
	}

	return 0;
}

void set_def_res(struct tc358840 *priv, int gang_mode)
{
	switch (gang_mode) {
	case CAMERA_GANG_L_R:
	case CAMERA_GANG_R_L:
		priv->s_data->def_width = priv->input.hactive;
		priv->s_data->def_height = priv->input.vactive;
		break;
	case CAMERA_GANG_T_B:
	case CAMERA_GANG_B_T:
		priv->s_data->def_width = priv->input.hactive / 2;
		priv->s_data->def_height = priv->input.vactive * 2;
		break;
	}
	dev_info(&priv->i2c_client->dev, "gang_mode %d: size %i x %i\n",
		gang_mode, priv->s_data->def_width, priv->s_data->def_height);
}

static irqreturn_t int_irq_handler(int irq, void *data)
{
	struct tc358840 *info = (struct tc358840_info *)data;
	struct v4l2_control gang_mode_control;
	u16 int_status, val2;
	u8 hdmi_int0, hdmi_int1, clk_int, sys_int, sys_status;
	int offset;

	/* FIXME: somehow below delays are needed to make irq working */
	tc358840_read_reg16(info, INTSTATUS, &int_status);
	msleep(1);
	tc358840_read_reg8(info, 0x8500, &hdmi_int0);
	msleep(1);
	tc358840_read_reg8(info, 0x8501, &hdmi_int1);
	msleep(1);
	tc358840_read_reg8(info, 0x8502, &sys_int);
	msleep(1);
	tc358840_read_reg8(info, 0x8503, &clk_int);
	msleep(1);

	if (int_status & INTSTATUS_HDMI_INT) {
		/*
		 * Need to check if bit1 of hdmi_int1 is set
		 * But somehow it's not while it shall be
		 * Thus check bit5 of clk_int directly
		 */
		if (clk_int & 0x20) {
			stop_tx(info);
			sync_hdmi(info);

			/* Update hdmi input resolution */
			info->input.changed = true;
			tc358840_read_reg16(info,
				DE_HWID0, &info->input.hactive);
			tc358840_read_reg16(info,
				DE_VWID0, &info->input.vactive);
			dev_info(&info->i2c_client->dev,
				"hdmi in hactive: %d, vactive: %d\n",
				info->input.hactive, info->input.vactive);
			tc358840_hdmiin_timing_chk(info);

			/* Use single CSI port for resolutions below 1080p */
			if (info->input.hactive < 1920)
				set_csi_split(info, DISABLE);
			else
				set_csi_split(info, LEFT_RIGHT);

			gang_mode_control.id = V4L2_CID_GANG_MODE;
			v4l2_g_ctrl(&info->ctrl_handler, &gang_mode_control);
			set_def_res(info, gang_mode_control.value);

			start_csi_tx(info);
		}

#if ENABLE_HPD_PIN
		/* Check bit 0 (DDC change) of SYS_INT */
		if (sys_int & 0x01) {
			tc358840_read_reg8(info, SYS_STATUS, &sys_status);
			if (sys_status & 0x01) {
				dev_info(&info->i2c_client->dev,
					"%s: hot plug detected, doing init...\n",
					__func__);
			} else {
				dev_info(&info->i2c_client->dev,
					"%s: hot unplug detected, stoping tx...\n",
					__func__);
				stop_tx(info);
			}
		}
#endif
	}

	tc358840_read_reg16(info, SYSINTSTATUS, &val2);

	/* clear the interrupt status */
	for (offset = 0x8502; offset <= 0x850f; offset++)
		tc358840_write_reg8(info, offset, 0xFF);

	tc358840_write_reg16(info, INTSTATUS, int_status);

	return IRQ_HANDLED;
}

static int toggle_reset(struct tc358840 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device_node *np = client->dev.of_node;
	int err;

	priv->rst_gpio = of_get_named_gpio(np, "rst", 0);
	if (!gpio_is_valid(priv->rst_gpio)) {
		dev_err(&client->dev, "Invalid reset gpio\n");
		return -EINVAL;
	}

	err = gpio_request(priv->rst_gpio, "hdmi-reset");
	if (err < 0)
		dev_err(&client->dev,
			"rst gpio request failed %d\n", err);

	/*
	 * FIXME: VDD33_HDMI to RESETN should >= 200ns, make it 1ms
	 */
	gpio_direction_output(priv->rst_gpio, 0);
	msleep(1);
	gpio_direction_output(priv->rst_gpio, 1);
	/*
	 * FIXME: core clock will be stable after 0.7 ~ 1ms after
	 * the de-assertion of the reset pin, thus make it 1ms
	 */
	msleep(1);

	return 0;
}

#if ENABLE_HPD_PIN
static int hpd_init(struct tc358840 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device_node *np = client->dev.of_node;
	int irq;
	int err;

	priv->hpd_gpio = of_get_named_gpio(np, "hpd", 0);
	if (!gpio_is_valid(priv->hpd_gpio)) {
		dev_err(&client->dev, "Invalid hotplug gpio\n");
		return -EINVAL;
	}

	irq = gpio_to_irq(priv->hpd_gpio);
	if (irq < 0) {
		dev_err(&client->dev,
			"hpd gpio to irq map failed\n");
		return -EINVAL;
	}

	err = gpio_request(priv->hpd_gpio, "hdmi-in-hpd");
	if (err < 0)
		dev_err(&client->dev,
			"hpd gpio request failed %d\n", err);
	gpio_direction_input(priv->hpd_gpio);

	err = request_threaded_irq(irq,
				NULL, hpd_irq_handler,
				(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT),
				"hdmi-in-hpd", priv);
	if (err) {
		dev_err(&client->dev,
			"request threaded irq failed: %d\n", err);
		goto fail;
	}


	return 0;
fail:
	gpio_free(priv->hpd_gpio);
	return err;
}
#endif

static int tc358840_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tc358840 *priv =
		container_of(ctrl->handler, struct tc358840, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_GANG_MODE:
		set_def_res(priv, ctrl->val);
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int tc358840_ctrls_init(struct tc358840 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static int irq_init(struct tc358840 *info)
{
	struct i2c_client *client = info->i2c_client;
	struct device_node *np = client->dev.of_node;
	int irq;
	int err;

	info->int_gpio = of_get_named_gpio(np, "int", 0);
	if (!gpio_is_valid(info->int_gpio)) {
		dev_err(&client->dev, "Invalid interrupt gpio\n");
		return -EINVAL;
	}

	irq = gpio_to_irq(info->int_gpio);
	if (irq < 0) {
		dev_err(&client->dev,
			"int gpio to irq map failed\n");
		return -EINVAL;
	}

	err = gpio_request(info->int_gpio, "hdmi-in-int");
	if (err < 0)
		dev_err(&client->dev,
			"interrupt gpio request failed %d\n", err);
	gpio_direction_input(info->int_gpio);

	err = request_threaded_irq(irq,
				NULL, int_irq_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
				"hdmi-in-int", info);
	if (err) {
		dev_err(&client->dev,
			"%s: request threaded irq failed: %d\n",
				__func__, err);
		goto fail;
	}

	info->chip_irq = irq;

	return 0;
fail:
	gpio_free(info->int_gpio);
	return err;
}

static int tc358840_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tegra_camera_platform_data *tc358840_camera_data;
	struct soc_camera_link *tc358840_camera_link;
	struct camera_common_data *common_data;
	struct v4l2_control gang_mode_control;
	struct soc_camera_subdev_desc *ssdd;
	struct tc358840 *priv;
	int err;

	pr_info("[TC358840]: probing v4l2 sensor.\n");

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			sizeof(struct tc358840) + sizeof(struct v4l2_ctrl *) *
			ARRAY_SIZE(ctrl_config_list),
			GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	client->dev.of_node = of_find_node_by_name(NULL, "tc358840");

	tc358840_camera_link = (struct soc_camera_link *)
		soc_camera_i2c_to_desc(client);
	tc358840_camera_data = (struct tegra_camera_platform_data *)
		tc358840_camera_link->priv;

	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= &tc358840_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  TC358840_DEFAULT_DATAFMT);
	common_data->priv		= (void *)priv;
	common_data->ident		= V4L2_IDENT_UNKNOWN;
	common_data->numfmts		= ARRAY_SIZE(tc358840_frmfmt);
	common_data->def_mode		= TC358840_DEFAULT_MODE;
	common_data->def_width		= TC358840_DEFAULT_WIDTH;
	common_data->def_height		= TC358840_DEFAULT_HEIGHT;
	common_data->def_clk_freq	= TC358840_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;

	v4l2_i2c_subdev_init(&common_data->subdev, client,
			     &tc358840_subdev_ops);

	err = tc358840_ctrls_init(priv);
	if (err)
		return err;

	gang_mode_control.id = V4L2_CID_GANG_MODE;
	gang_mode_control.value = (int)tc358840_camera_data->gang_mode;
	v4l2_s_ctrl(NULL, common_data->ctrl_handler, &gang_mode_control);

	/* Power init */
	tc358840_pwr_init(priv);
	tc358840_s_power(priv->subdev, 1);

	/* Reset the chip */
	toggle_reset(priv);

	/* Read the chip and revision id */
	tc358840_check_id(priv);

	/* reset resolution info */
	priv->input.hactive = 0;
	priv->input.vactive = 0;

#if ENABLE_HPD_PIN
	hpd_init(priv);
#endif

	irq_init(priv);

	run_init_seq(priv);

	/* register a switch device for hpd status */
#ifdef CONFIG_SWITCH
	priv->hpd_switch.name = "hdmi-in";
	err = switch_dev_register(&priv->hpd_switch);
#endif

	return 0;
}

static int tc358840_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd;
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct tc358840 *priv = (struct tc358840 *)s_data->priv;

	ssdd = soc_camera_i2c_to_desc(client);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	v4l2_ctrl_handler_free(&priv->ctrl_handler);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&priv->hpd_switch);
#endif
	kfree(s_data);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id tc358840_id[] = {
	{ "tc358840_v4l2", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tc358840_id);

static const struct of_device_id tc358840_of_match[] = {
	{ .compatible = "tosh,hdmi2csi" },
	{ },
};
MODULE_DEVICE_TABLE(of, tc358840_of_match);

#ifdef CONFIG_PM_SLEEP
static int tc358840_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct tc358840 *priv = (struct tc358840 *)s_data->priv;
	int hpd_irq;

	hpd_irq = gpio_to_irq(priv->hpd_gpio);
	enable_irq_wake(hpd_irq);
	return 0;
}

static int tc358840_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct tc358840 *priv = (struct tc358840 *)s_data->priv;
	int hpd_irq;

	hpd_irq = gpio_to_irq(priv->hpd_gpio);
	disable_irq_wake(hpd_irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(tc358840_pm_ops,
	tc358840_suspend, tc358840_resume);
static struct i2c_driver tc358840_i2c_driver = {
	.driver = {
		.name = "tc358840_v4l2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tc358840_of_match),
		.pm = &tc358840_pm_ops,
	},
	.probe = tc358840_probe,
	.remove = tc358840_remove,
	.id_table = tc358840_id,
};

module_i2c_driver(tc358840_i2c_driver);

MODULE_AUTHOR("Frank Shi <fshi@nvidia.com>");
MODULE_LICENSE("GPL v2");
