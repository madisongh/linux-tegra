/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CHANNELS_H
#define INCLUDE_CAMRTC_CHANNELS_H

#include "camrtc-common.h"

/*
 * All the enums and the fields inside the structs described in this header
 * file supports only uintX_t types, where X can be 8,16,32,64.
 */
#define CAMRTC_TAG64(s0, s1, s2, s3, s4, s5, s6, s7) ( \
	((uint64_t)s0 << 0) | ((uint64_t)s1 << 8) | \
	((uint64_t)s2 << 16) | ((uint64_t)s3 << 24) | \
	((uint64_t)s4 << 32) | ((uint64_t)s5 << 40) | \
	((uint64_t)s6 << 48) | ((uint64_t)s7 << 56))

#define CAMRTC_TAG_IVC_SETUP CAMRTC_TAG64('I','V','C', '-', 'S','E','T','U')

#define CAMRTC_TAG_NV_TRACE CAMRTC_TAG64('N','V',' ','T','R','A','C','E')

struct camrtc_tlv {
	uint64_t tag;
	uint64_t len;
};

/* Multiple setup structures can follow each other. */
struct camrtc_tlv_ivc_setup {
	uint64_t tag;
	uint64_t len;
	uint64_t rx_iova;
	uint32_t rx_frame_size;
	uint32_t rx_nframes;
	uint64_t tx_iova;
	uint32_t tx_frame_size;
	uint32_t tx_nframes;
	uint32_t channel_group;
	uint32_t ivc_version;
	char ivc_service[32];
};

enum {
	/* 0 .. 127 indicate unknown commands */
	RTCPU_CH_ERR_NO_SERVICE = 128,
	RTCPU_CH_ERR_ALREADY,
	RTCPU_CH_ERR_UNKNOWN_TAG,
	RTCPU_CH_ERR_INVALID_IOVA,
	RTCPU_CH_ERR_INVALID_PARAM,
};

#endif /* INCLUDE_CAMRTC_CHANNELS_H */
