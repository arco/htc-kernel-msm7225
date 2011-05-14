/* arch/arm/mach-msm/board-buzz.c
 * Copyright (C) 2009 HTC Corporation.
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
#include <asm/gpio.h>
#include <mach/vreg.h>

#include "board-buzz.h"

static int opt_x_axis_threshold = 1, opt_y_axis_threshold = 1;

struct buzz_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
};

static bool nav_just_on;
static int nav_on_jiffies;

static unsigned int buzz_col_gpios[] = {
	BUZZ_GPIO_Q_KP_MKOUT0,
	BUZZ_GPIO_Q_KP_MKOUT1,
	BUZZ_GPIO_Q_KP_MKOUT2,
};

static unsigned int buzz_row_gpios[] = {
	BUZZ_GPIO_Q_KP_MKIN0_1,
	BUZZ_GPIO_Q_KP_MKIN1_1,
	BUZZ_GPIO_Q_KP_MKIN2_1,
};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(buzz_row_gpios) + (row))

static unsigned short buzz_keymap[ARRAY_SIZE(buzz_col_gpios) *
					ARRAY_SIZE(buzz_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = BTN_MOUSE,
	[KEYMAP_INDEX(0, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(0, 2)] = KEY_VOLUMEUP,

	[KEYMAP_INDEX(1, 0)] = KEY_RESERVED,/*KEY_SEARCH,*/
	[KEYMAP_INDEX(1, 1)] = KEY_RESERVED,/*KEY_BACK,*/
	[KEYMAP_INDEX(1, 2)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(2, 0)] = KEY_RESERVED,/*KEY_HOME,*/
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,/*KEY_MENU,*/
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
};

static struct gpio_event_matrix_info buzz_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = buzz_keymap,
	.output_gpios = buzz_col_gpios,
	.input_gpios = buzz_row_gpios,
	.noutputs = ARRAY_SIZE(buzz_col_gpios),
	.ninputs = ARRAY_SIZE(buzz_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		GPIOKPF_REMOVE_PHANTOM_KEYS |
		GPIOKPF_PRINT_UNMAPPED_KEYS /* |
		GPIOKPF_PRINT_MAPPED_KEYS */),
	.detect_phone_status = 1,
};

static struct gpio_event_direct_entry buzz_keypad_nav_map[] = {
	{
		.gpio = BUZZ_GPIO_POWER_KEY,
		.code = KEY_POWER,
		.wakeup = 1,
	},
};

static struct gpio_event_input_info buzz_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = buzz_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(buzz_keypad_nav_map)
};

uint16_t buzz_x_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct buzz_axis_info *ai =
			container_of(info, struct buzz_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_x_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_x_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_x_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

uint16_t buzz_y_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct buzz_axis_info *ai =
			container_of(info, struct buzz_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_y_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_y_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_y_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

static struct vreg *jog_vreg;

int buzz_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	if (on) {
		vreg_enable(jog_vreg);
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	} else
		vreg_disable(jog_vreg);

	return 0;
}

static uint32_t buzz_x_axis_gpios[] = {
	BUZZ_GPIO_BALL_LEFT, BUZZ_GPIO_BALL_RIGHT
};

static struct buzz_axis_info buzz_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(buzz_x_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(buzz_x_axis_gpios),
		.map = buzz_x_axis_map,
		.gpio = buzz_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */,
	}
};

static uint32_t buzz_y_axis_gpios[] = {
	BUZZ_GPIO_BALL_UP, BUZZ_GPIO_BALL_DOWN
};

static struct buzz_axis_info buzz_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(buzz_y_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(buzz_y_axis_gpios),
		.map = buzz_y_axis_map,
		.gpio = buzz_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  */
	}
};

static struct gpio_event_info *buzz_ball_input_info[] = {
	&buzz_keypad_matrix_info.info,
	&buzz_keypad_nav_info.info,
	&buzz_x_axis.info.info,
	&buzz_y_axis.info.info,
};

static struct gpio_event_info *buzz_oj_input_info[] = {
	&buzz_keypad_matrix_info.info,
	&buzz_keypad_nav_info.info,
};

static struct gpio_event_platform_data buzz_keypad_data = {
	.names = {
		"buzz-keypad",
		"buzz-nav",
		NULL,
	},
	.info = buzz_ball_input_info,
	.info_count = ARRAY_SIZE(buzz_ball_input_info),
	.power = buzz_nav_power,
};

static struct platform_device buzz_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &buzz_keypad_data,
	},
};

static int buzz_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0
};

static struct keyreset_platform_data buzz_reset_keys_pdata = {
	.keys_up = buzz_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device buzz_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &buzz_reset_keys_pdata,
};

int __init buzz_init_keypad(void)
{
	jog_vreg = vreg_get(&buzz_keypad_device.dev, "wlan");
	if (jog_vreg == NULL) {
		printk(KERN_WARNING "%s: can't initialize jogball power\n",
								__func__);
		return -ENOENT;
	}

	if (platform_device_register(&buzz_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	if (system_rev < 4) {
		buzz_keymap[KEYMAP_INDEX(0, 0)] = MATRIX_KEY(1, BTN_MOUSE);
	} else {
		buzz_oj_input_info[0]->oj_btn = true;
		buzz_keypad_data.info = buzz_oj_input_info;
		buzz_keypad_data.info_count = ARRAY_SIZE(buzz_oj_input_info);
	}

	return platform_device_register(&buzz_keypad_device);
}

