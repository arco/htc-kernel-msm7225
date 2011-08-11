/* arch/arm/mach-msm7225/board-bahamas-keypad.c
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

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <mach/board_htc.h>

#include "board-bahamas.h"

static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_bahamas."

module_param_named(keycaps, keycaps, charp, 0);

static unsigned int bahamas_col_gpios[] = { 35, 34, 33 };
static unsigned int bahamas_row_gpios[] = { 42, 41, 40, 39 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(bahamas_row_gpios) + (row))

static const unsigned short bahamas_keymap[ARRAY_SIZE(bahamas_col_gpios) * ARRAY_SIZE(bahamas_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_REPLY,
	[KEYMAP_INDEX(0, 1)] = KEY_UP,
	[KEYMAP_INDEX(0, 2)] = KEY_RIGHT,
	[KEYMAP_INDEX(0, 3)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(1, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(1, 1)] = KEY_DOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_BACK,
	[KEYMAP_INDEX(1, 3)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_SEND,
	[KEYMAP_INDEX(2, 2)] = KEY_LEFT,
	[KEYMAP_INDEX(2, 3)] = KEY_MENU,
};

static struct gpio_event_matrix_info bahamas_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = bahamas_keymap,
	.output_gpios = bahamas_col_gpios,
	.input_gpios = bahamas_row_gpios,
	.noutputs = ARRAY_SIZE(bahamas_col_gpios),
	.ninputs = ARRAY_SIZE(bahamas_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		GPIOKPF_REMOVE_PHANTOM_KEYS |
		GPIOKPF_PRINT_UNMAPPED_KEYS /* |
		GPIOKPF_PRINT_MAPPED_KEYS */),
};

static struct gpio_event_direct_entry bahamas_keypad_nav_map[] = {
	{
		.gpio = BAHAMAS_POWER_KEY,
		.code = KEY_END,
	}
};

static struct gpio_event_input_info bahamas_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = bahamas_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(bahamas_keypad_nav_map)
};

static struct gpio_event_info *bahamas_keypad_info[] = {
	&bahamas_keypad_matrix_info.info,
	&bahamas_keypad_nav_info.info,
};

static struct gpio_event_platform_data bahamas_keypad_data = {
	.name = "bahamas-keypad",
	.info = bahamas_keypad_info,
	.info_count = ARRAY_SIZE(bahamas_keypad_info)
};

static struct platform_device bahamas_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &bahamas_keypad_data,
	},
};

static int bahamas_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data bahamas_reset_keys_pdata0 = {
	.keys_up = bahamas_reset_keys_up,
	.keys_down = {
		KEY_HOME,
		KEY_MENU,
		KEY_END,
		0
	},
};

static struct keyreset_platform_data bahamas_reset_keys_pdata1 = {
	.keys_up = bahamas_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

struct platform_device bahamas_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &bahamas_reset_keys_pdata1,
};

int __init bahamas_init_keypad(void)
{
	char *get_cid, *get_carrier, *get_keycaps;

	board_get_cid_tag(&get_cid);
	board_get_carrier_tag(&get_carrier);
	board_get_keycaps_tag(&get_keycaps);

	if(!system_rev)
		bahamas_reset_keys_device.dev.platform_data = &bahamas_reset_keys_pdata0;

	if (platform_device_register(&bahamas_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&bahamas_keypad_device);
}
