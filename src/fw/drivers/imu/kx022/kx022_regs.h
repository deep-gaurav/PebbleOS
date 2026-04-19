/* SPDX-FileCopyrightText: 2025 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#define KX022_MANUFACTURER_ID  0x00
#define KX022_WHO_AM_I         0x0F
#define KX022_WHO_AM_I_VALUE   0x14

#define KX022_XOUT_L           0x06
#define KX022_XOUT_H           0x07
#define KX022_YOUT_L           0x08
#define KX022_YOUT_H           0x09
#define KX022_ZOUT_L           0x0A
#define KX022_ZOUT_H           0x0B

#define KX022_INT_REL          0x11
#define KX022_INS1             0x12
#define KX022_INS2             0x13
#define KX022_INS3             0x14
#define KX022_STATUS_REG       0x15

#define KX022_CNTL1            0x18
#define KX022_CNTL1_PC1        (1 << 7)
#define KX022_CNTL1_RES        (1 << 6)
#define KX022_CNTL1_DRDYE      (1 << 5)
#define KX022_CNTL1_GSEL_2G    (0 << 3)
#define KX022_CNTL1_GSEL_4G    (1 << 3)
#define KX022_CNTL1_GSEL_8G    (2 << 3)
#define KX022_CNTL1_GSEL_16G   (3 << 3)
#define KX022_CNTL1_TDTE       (1 << 2)
#define KX022_CNTL1_WUFE       (1 << 1)
#define KX022_CNTL1_TILT       (1 << 0)

#define KX022_CNTL2            0x19
#define KX022_CNTL2_SRST       (1 << 7)

#define KX022_CNTL3            0x1A
#define KX022_CNTL3_OTP_12P5   (0 << 6)
#define KX022_CNTL3_OTP_25     (1 << 6)
#define KX022_CNTL3_OTP_50     (2 << 6)
#define KX022_CNTL3_OTP_100    (3 << 6)
#define KX022_CNTL3_OTP_200    (4 << 6)
#define KX022_CNTL3_OTP_400    (5 << 6)
#define KX022_CNTL3_OTP_800    (6 << 6)
#define KX022_CNTL3_OTP_1600   (7 << 6)
#define KX022_CNTL3_OTDT_50    (0 << 4)
#define KX022_CNTL3_OTDT_100   (1 << 4)
#define KX022_CNTL3_OTDT_200   (2 << 4)
#define KX022_CNTL3_OTDT_400   (3 << 4)
#define KX022_CNTL3_OTDT_800   (4 << 4)
#define KX022_CNTL3_OTDT_1600  (5 << 4)
#define KX022_CNTL3_OTDT_3200  (6 << 4)
#define KX022_CNTL3_OTDT_6400  (7 << 4)
#define KX022_CNTL3_OWUF_0P781 (0 << 0)
#define KX022_CNTL3_OWUF_1P56  (1 << 0)
#define KX022_CNTL3_OWUF_3P13  (2 << 0)
#define KX022_CNTL3_OWUF_6P25  (3 << 0)
#define KX022_CNTL3_OWUF_12P5  (4 << 0)
#define KX022_CNTL3_OWUF_25    (5 << 0)
#define KX022_CNTL3_OWUF_50    (6 << 0)
#define KX022_CNTL3_OWUF_100   (7 << 0)

#define KX022_ODCNTL           0x1B
#define KX022_ODCNTL_OSA_12P5  (0 << 0)
#define KX022_ODCNTL_OSA_25    (1 << 0)
#define KX022_ODCNTL_OSA_50    (2 << 0)
#define KX022_ODCNTL_OSA_100   (3 << 0)
#define KX022_ODCNTL_OSA_200   (4 << 0)
#define KX022_ODCNTL_OSA_400   (5 << 0)
#define KX022_ODCNTL_OSA_800   (6 << 0)
#define KX022_ODCNTL_OSA_1600  (7 << 0)
#define KX022_ODCNTL_LPRO      (1 << 3)

#define KX022_INC1             0x1C
#define KX022_INC2             0x1D
#define KX022_INC3             0x1E
#define KX022_INC4             0x1F
#define KX022_INC5             0x20
#define KX022_INC6             0x21

#define KX022_WUFC             0x23
#define KX022_TDTRC            0x24
#define KX022_TDTRC_NTD        (1 << 3)
#define KX022_TDTRC_PTD        (1 << 2)
#define KX022_TDTRC_NSD        (1 << 1)
#define KX022_TDTRC_PSD        (1 << 0)

#define KX022_TDTC             0x25
#define KX022_TTH              0x26
#define KX022_TTL              0x27

#define KX022_NAXP             0x28
#define KX022_NAXN             0x29
#define KX022_NAYP             0x2A
#define KX022_NAYN             0x2B
#define KX022_NAZP             0x2C
#define KX022_NAZN             0x2D

#define KX022_WUFTH            0x30
#define KX022_LP_CNTL          0x35
#define KX022_LP_CNTL_AVER_1X  (0 << 4)
#define KX022_LP_CNTL_AVER_2X  (1 << 4)
#define KX022_LP_CNTL_AVER_4X  (2 << 4)
#define KX022_LP_CNTL_AVER_8X  (3 << 4)
#define KX022_LP_CNTL_AVER_16X (4 << 4)
#define KX022_LP_CNTL_AVER_32X (5 << 4)
#define KX022_LP_CNTL_AVER_64X (6 << 4)
#define KX022_LP_CNTL_AVER_128X (7 << 4)

#define KX022_BUF_CLEAR        0x3E
#define KX022_BUF_READ         0x3F
