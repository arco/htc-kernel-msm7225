/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/kernel.h>

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
};

enum msm_camera_flash {
	MSM_CAMERA_FLASH_NONE,
	MSM_CAMERA_FLASH_LED
};

struct msm_camera_sensor_info {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int mclk;
	const char *sensor_name;
	enum msm_camera_flash flash_type;
	int (*sensor_probe)(void *, void *);
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
};

//for mm-camera
struct msm_camera_platform_data {
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	uint8_t snum;
	struct msm_camera_sensor_info *sinfo;
	struct msm_camera_io_ext ioext;
};

struct msm_camera_device_platform_data {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
};

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_add_usb_devices(void (*phy_reset) (void), void (*phy_shutdown) (void));
void __init msm_change_usb_id(__u16 vendor_id, __u16 product_id);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(void);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);

#if 0
#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif
#endif
#endif
