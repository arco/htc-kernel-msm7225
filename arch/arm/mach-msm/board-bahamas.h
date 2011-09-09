/* arch/arm/mach-msm/board-bahamas.h
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Tony Liu <tony_liu@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#ifndef __ARCH_ARM_MACH_MSM_BOARD_BAHAMAS_H
#define __ARCH_ARM_MACH_MSM_BOARD_BAHAMAS_H

#include <mach/board.h>

#define MSM_LINUX_BASE1			0x02E00000
#define MSM_LINUX_SIZE1			0x05200000
#define MSM_LINUX_BASE2_MONODIE		0x08000000
#define MSM_LINUX_BASE2_DUALDIE		0x20000000
#define MSM_LINUX_SIZE2			0x06E00000

#define MSM_EBI1_CS0_BASE		0x00000000
#define MSM_EBI1_CS0_SIZE		0x5500000

#define MSM_EBI1_CS1_BASE		0x20000000
#define MSM_EBI1_CS1_SIZE		0x2000000

#define MSM_PMEM_MDP_BASE		0x00000000
#define MSM_PMEM_MDP_SIZE		0x00800000

#define MSM_PMEM_ADSP_BASE_MONODIE	0x0F000000
#define MSM_PMEM_ADSP_BASE_DUALDIE	0x27000000
#define MSM_PMEM_ADSP_SIZE		0x00800000

#define MSM_PMEM_CAMERA_BASE_MONODIE	0x0F800000
#define MSM_PMEM_CAMERA_BASE_DUALDIE	0x27800000
#define MSM_PMEM_CAMERA_SIZE		0x00800000

#define MSM_LINUX_BASE			MSM_EBI1_CS0_BASE + 0x400000
#define MSM_LINUX_SIZE			0x5100000

#define MSM_FB_BASE			0x02D00000
#define MSM_FB_SIZE			0x9b000

#define MSM_RAM_CONSOLE_BASE		MSM_FB_BASE + MSM_FB_SIZE
#define MSM_RAM_CONSOLE_SIZE		128 * SZ_1K

#define BAHAMAS_POWER_KEY		(20)
#define BAHAMAS_GPIO_WIFI_EN		(102)
#define BAHAMAS_GPIO_SDMC_CD_N		(38)

#define BAHAMAS_MT9T013_CAM_PWDN	(91)
#define BAHAMAS_GPIO_CABLE_IN1		(18)
#define BAHAMAS_GPIO_UP_INT		(27)
#define BAHAMAS_GPIO_CABLE_IN2		(31)
#define BAHAMAS_GPIO_H2W_DATA		(86)
#define BAHAMAS_GPIO_H2W_CLK		(87)
#define BAHAMAS_GPIO_UART3_RX		(86)
#define BAHAMAS_GPIO_UART3_TX		(87)
#define BAHAMAS_GPIO_HEADSET_MIC	(17)
#define BAHAMAS_GPIO_35MM_HEADSET_DET	(112)
#define BAHAMAS_GPIO_AUD_EXTMIC_SEL	(113)
#define BAHAMAS_GPIO_WFM_ANT_SW		(121)

#define BAHAMAS_GPIO_UP_RESET_N     	(76)
#define BAHAMAS_GPIO_PS_HOLD        	(25)

/* BT */
#define BAHAMAS_GPIO_UART1_RTS		(43)
#define BAHAMAS_GPIO_UART1_CTS		(44)
#define BAHAMAS_GPIO_UART1_RX		(45)
#define BAHAMAS_GPIO_UART1_TX		(46)
#define BAHAMAS_GPIO_WB_SHUT_DOWN_N	(101)

#define BAHAMAS_GPIO_H2W_POWER		(513)
#define BAHAMAS_H2W_POWER_NAME		"gp2"

#define BAHAMAS_GPIO_COMPASS_RST_N	(93)
#define BAHAMAS_GPIO_COMPASS_INT_N	(37)

#define BAHAMAS_PROJECT_NAME		"bahamas"
#define BAHAMAS_LAYOUTS			{ \
		{ {  0, -1, 0}, {  1,  0, 0}, {0, 0,  1} },  \
		{ {  0, -1, 0}, { -1,  0, 0}, {0, 0,  1} },  \
		{ {  1,  0, 0}, {  0,  1, 0}, {0, 0,  1} },  \
		{ {  1,  0, 0}, {  0,  0, 1}, {0, 1,  0} }   \
					}

#define BAHAMAS_GPIO_GSENSOR_INT_N	(49)

#define BAHAMAS_MDDI_RSTz		(82)
#define BAHAMAS_V_VDDE2E_VDD2_GPIO	(0)

#define BAHAMAS_GPIO_VSYNC		(97)

void msm_init_irq(void);
void msm_init_gpio(void);
void config_bahamas_camera_on_gpios(void);
void config_bahamas_camera_off_gpios(void);

unsigned int bahamas_get_hwid(void);
unsigned int bahamas_get_skuid(void);
unsigned bahamas_engineerid(void);
int __init bahamas_init_panel(void);
int __init bahamas_init_keypad(void);
int bahamas_init_mmc(unsigned int sys_rev);
#endif
