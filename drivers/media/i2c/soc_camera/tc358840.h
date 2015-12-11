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

enum cmd_type {
	WR8 = 0,
	WR16,
	WR32,
	RD8,
	RD16,
	DELAY,
	CMD_END,
};

/*
 * How CSI port 0/1 splits the HDMI input
 * DISABLE - disable CSI2 TX split
 * MANUAL - enable, manually configure the window
 * LEFT_RIGHT - enable auto split, left half by TX0; right half by TX1
 * RIGHT_LEFT - enable auto split, left half by TX1; right half by TX0
 */
enum splitter_type {
	LEFT_RIGHT = 0,
	RIGHT_LEFT,
	MANUAL,
	DISABLE,
};

struct tc358840_reg {
	unsigned char cmd_type;
	unsigned int addr;
	unsigned int val;
};

struct hdmi_mode {
	int hactive;
	int vactive;
	bool changed;
};

struct csi_tx_window {
	int first_pixel;
	int len;
};

struct split_mode {
	u8 mode;
	struct csi_tx_window tx0;
	struct csi_tx_window tx1;
};

struct tc358840 {
	struct v4l2_subdev		*subdev;
	struct i2c_client		*i2c_client;
	struct regmap			*regmap;

	struct camera_common_power_rail	power;
	int				num_ctrls;
	struct v4l2_ctrl_handler	ctrl_handler;

	struct camera_common_data	*s_data;

#ifdef CONFIG_SWITCH
	struct switch_dev		hpd_switch;
#endif

	int				hpd_gpio;
	int				rst_gpio;
	int				int_gpio;
	int				cam_1v8_gpio;
	int				cam_1v2_gpio;

	struct				hdmi_mode input;
	unsigned int			hpd_irq;
	unsigned int			chip_irq;
	struct				split_mode split;

	struct v4l2_ctrl		*ctrls[];
};


#define SYSCTL	0x0002
#define CONFCTL0	0x0004
#define CONFCTL1	0x0006
#define CONFCTL1	0x0006
#define INTSTATUS	0x0014
#define INTSTATUS_HDMI_INT	0x0200
#define INTMASK	0x0016
#define SYSINTSTATUS	0x001A

#define TX0CSI_TX_CLKEN	0x0108
#define TX0CSI_TX_CLKSEL	0x010C
#define TX0MODE_CONFIG	0x0110
#define TX0LANE_ENABLE	0x0118
#define TX0CSITX_START	0x011C
#define TX0LINE_INIT_COUNT	0x0120
#define TX0HSTX_TO_COUNT	0x0124
#define TX0FUNC_ENABLE	0x0128
#define TX0CSI_LPTX_MODE	0x012C
#define TX0CSI_TATO_COUNT	0x0130
#define TX0CSI_PRESP_BTA_COUNT	0x0134
#define TX0CSI_PRESP_LPR_COUNT	0x0138
#define TX0CSI_PRESP_LPW_COUNT	0x013C
#define TX0CSI_PRESP_HSR_COUNT	0x0140
#define TX0CSI_PRESP_HSW_COUNT	0x0144
#define TX0CSI_PR_TO_COUNT	0x0148
#define TX0CSI_LRX_H_TO_COUNT	0x014C
#define TX0FUNC_MODE	0x0150
#define TX0CSI_RX_VC_ENABLE	0x0154
#define TX0IND_TO_COUNT	0x0158
#define TX0INIT_INT_MSK	0x0164
#define TX0CSI_HSYNC_STOP_COUNT	0x0168
#define TX0APF_VDELAYCNT	0x0170
#define TX0APF_VC_CONFIG	0x0178
#define TX0CSI_RX_STATE_INT_MASK	0x01A4
#define TX0CSI_RXTRG_INT_MASK	0x01AC
#define TX0CSI_LPRX_THRESH_COUNT	0x01C0

#define TX0CSI_PRTO_INT_MASK	0x020C
#define TX0APP_SIDE_ERR_INT_MASK	0x0214
#define TX0CSI_RX_ERR_INT_MASK	0x021C
#define TX0CSI_LPTX_INT_MASK	0x0224
#define TX0DPHY_DLYCNTRL	0x0240
#define TX0LPTXTIMECNT	0x0254
#define TX0TCLK_HEADERCNT	0x0258
#define TX0TCLK_TRAILCNT	0x025C
#define TX0THS_HEADERCNT	0x0260
#define TX0TWAKEUPCNT	0x0264
#define TX0TCLK_POSTCNT	0x0268
#define TX0THS_TRAILCNT	0x026C
#define TX0HSTXVREGCNT	0x0270
#define TX0HSTXVREGEN	0x0274
#define TX0BTA_COUNT	0x0278
#define TX0DPHY_TX_ADJUST	0x027C
#define TX0DPHY_CAP	0x0288
#define TX0MIPI_PLL_CONTROL	0x02A0
#define TX0MIPI_PLL_CNF	0x02AC

#define TX1CSI_TX_CLKEN	0x0308
#define TX1CSI_TX_CLKSEL	0x030C
#define TX1LANE_ENABLE	0x0318
#define TX1MODE_CONFIG	0x0310
#define TX1CSITX_START	0x031C
#define TX1LINE_INIT_COUNT	0x0320
#define TX1HSTX_TO_COUNT	0x0324
#define TX1FUNC_ENABLE	0x0328
#define TX1CSI_LPTX_MODE	0x032C
#define TX1CSI_TATO_COUNT	0x0330
#define TX1CSI_PRESP_BTA_COUNT	0x0334
#define TX1CSI_PRESP_LPR_COUNT	0x0338
#define TX1CSI_PRESP_LPW_COUNT	0x033C
#define TX1CSI_PRESP_HSR_COUNT	0x0340
#define TX1CSI_PRESP_HSW_COUNT	0x0344
#define TX1CSI_PR_TO_COUNT	0x0348
#define TX1CSI_LRX_H_TO_COUNT	0x034C
#define TX1FUNC_MODE	0x0350
#define TX1CSI_RX_VC_ENABLE	0x0354
#define TX1IND_TO_COUNT	0x0358
#define TX1INIT_INT_MSK	0x0364
#define TX1CSI_HSYNC_STOP_COUNT	0x0368
#define TX1APF_VDELAYCNT	0x0370
#define TX1APF_VC_CONFIG	0x0378
#define TX1CSI_RX_STATE_INT_MASK	0x03A4
#define TX1CSI_RXTRG_INT_MASK	0x03AC
#define TX1CSI_LPRX_THRESH_COUNT	0x03C0

#define TX1CSI_PRTO_INT_MASK	0x040C
#define TX1APP_SIDE_ERR_INT_MASK	0x0414
#define TX1CSI_RX_ERR_INT_MASK	0x041C
#define TX1CSI_LPTX_INT_MASK	0x0424
#define TX1DPHY_DLYCNTRL	0x0440
#define TX1LPTXTIMECNT	0x0454
#define TX1TCLK_HEADERCNT	0x0458
#define TX1TCLK_TRAILCNT	0x045C
#define TX1THS_HEADERCNT	0x0460
#define TX1TWAKEUPCNT	0x0464
#define TX1TCLK_POSTCNT	0x0468
#define TX1THS_TRAILCNT	0x046C
#define TX1HSTXVREGCNT	0x0470
#define TX1HSTXVREGEN	0x0474
#define TX1BTA_COUNT	0x0478
#define TX1DPHY_TX_ADJUST	0x047C
#define TX1DPHY_CAP	0x0488
#define TX1MIPI_PLL_CONTROL	0x04A0
#define TX1MIPI_PLL_CNF	0x04AC
#define TX1MIPI_PLL_CONTROL	0x04A0

#define STX0_CTL	0x5000
#define STX0_VPID1	0x5002
#define STX0_VPID2	0x5004
#define STX0_VPID3	0x5006
#define STX0_WC	0x5008
#define STX0_FPX	0x500C
#define STX0_LPX	0x500E
#define STX0_MAXFCNT	0x0510
#define STX1_MAXFCNT	0x0514
#define STX1_CTL	0x5080
#define STX1_VPID1	0x5082
#define STX1_VPID2	0x5084
#define STX1_VPID3	0x5086
#define STX1_WC	0x5088
#define STX1_FPX	0x508C
#define STX1_LPX	0x508E

#define CB_CTL	0x7000
#define CB_HSW	0x7008
#define CB_VSW	0x700A
#define CB_HTOTAL	0x700C
#define CB_VTOTAL	0x700E
#define CB_HACT	0x7010
#define CB_VACT	0x7012
#define CB_HSTART	0x7014
#define CB_VSTART	0x7016

#define PX_FREQ0	0x8405
#define PX_FREQ1	0x8406
#define PHY_CTL	0x8410
#define PHY_ENB	0x8413
#define EQ_BYPS	0x8420
#define APLL_CTL	0x84F0
#define DDCIO_CTL	0x84F4

#define SYS_INT	0x8502
#define MISC_INT	0x850B
#define SYS_INTM	0x8512
#define CLK_INTM	0x8513
#define PACKET_INTM	0x8514
#define AUDIO_IMNTM	0x8515
#define ABUF_INTM	0x8516
#define MISC_INTM	0X851B
#define SYS_STATUS	0x8520
#define VI_STATUS0	0x8521
#define VI_STATUS1	0x8522
#define VI_STATUS2	0x8525
#define CLK_STATUS	0x8526
#define VI_STATUS3	0x8528
#define SYS_FREQ0	0x8540
#define SYS_FREQ1	0x8541
#define DDC_CTL	0x8543
#define HPD_CTL	0x8544
#define INIT_END	0x854A
#define HDCP_MODE	0x8560
#define DE_HPOS0	0x8580
#define DE_HPOS1	0x8581
#define DE_HWID0	0x8582
#define DE_HWID1	0x8583
#define DE_POS_A0	0x8584
#define DE_POS_A1	0x8585
#define DE_POS_B0	0x8586
#define DE_POS_B1	0x8587
#define DE_VWID0	0x858C
#define DE_VWID1	0x858D
#define H_SIZE0	0x858E
#define H_SIZE1	0x858F
#define V_SIZE0	0x8590
#define V_SIZE1	0x8591

#define EDID_MODE	0x85E0
#define EDID_LEN1_2	0x85E3

#define AUD_AUTO_MUTE	0x8600
#define AUTO_CMD0	0x8602
#define AUTO_CMD1	0x8603
#define AUTO_CMD2	0x8604
#define BUFINIT_START	0x8606
#define FS_MUTE	0x8607
#define LOCKDET_FREQ0	0x8630
#define LOCKDET_REF1_2	0x8631
#define NCO_F0_MOD	0x8670
#define SDO_MODE1	0x8652
#define NCO_48F0A_D	0x8671
#define NCO_44F0A_D	0x8675
#define AUD_MODE	0x8680

#define EDID_RAM_START	0x8C00

#define CSC_SCLK0	0x8A0C
#define CSC_SCLK1	0x8A0D
#define VOUT_FMT	0x8A00
#define VOUT_FIL	0x8A01
#define VOUT_SYNC0	0x8A02
#define VOUT_COLOR	0x8A08

#define EDID_RAM_SIZE	256
