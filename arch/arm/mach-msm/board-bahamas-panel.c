/*
 * Copyright (C) 2008 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#ifdef CONFIG_HTC_PWRSINK
#include <mach/htc_pwrsink.h>
#endif

#include "board-bahamas.h"
#include "proc_comm.h"
#include "devices.h"

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

static struct led_trigger *eid_lcd_backlight;
static void eid_set_backlight(int on)
{
	B("%s: enter.\n", __func__);

	if (on)
		led_trigger_event(eid_lcd_backlight, LED_FULL);
	else
		led_trigger_event(eid_lcd_backlight, LED_OFF);
}

struct mddi_table {
	uint32_t reg;
	uint32_t value;
	uint32_t msec;
};

static struct mddi_table hitachi_init_tb[] = {
	/* switching (a)->(b) */
	{0x11, 0x001A, 0},
	{0x12, 0x2000, 0},
	{0x13, 0x0070, 0},
	{0x14, 0x24e9, 0},
	{0x15, 0x0070, 0},
	{0x10, 0x0710, 10},
	{0x11, 0x0110, 10},
	{0x11, 0x0312, 10},
	{0x11, 0x0712, 10},
	{0x11, 0x0F1A, 20},
	{0x11, 0x0F3A, 30},
	{0x01, 0x0528, 0},
	{0x02, 0x0100, 0},
	{0x03, 0x1130, 0},
	{0x07, 0x0000, 0},
	{0x08, 0x0808, 0},
	{0x0B, 0x2107, 0},
	{0x0C, 0x0000, 0},
	{0x0F, 0x1801, 10},
	/* Jay, for adding delay after vsync */
	{0x29, 0x64, 0},
	/* Gamma setting */
	{0x50, 0x0005, 0},
	{0x51, 0x0109, 0},
	{0x52, 0x0000, 0},
	{0x53, 0x0008, 0},
	{0x54, 0x0000, 0},
	{0x55, 0x0901, 0},
	{0x56, 0x0500, 0},
	{0x57, 0x0800, 0},
	{0x58, 0x0000, 0},
	{0x59, 0x0000, 0},

	{0x30, 0x0000, 0},
	{0x31, 0x013F, 0},
	{0x32, 0x0000, 0},
	{0x33, 0x0000, 0},
	{0x36, 0x00EF, 0},
	{0x37, 0x0000, 0},
	{0x38, 0x013F, 0},
	{0x39, 0x0000, 30}, /* wait 2 frames */
	{0x07, 0x0012, 30}, /* wait 2 frames */
	{0x07, 0x1013, 0},
	{0x20, 0x0000, 0},
	{0x21, 0x0000, 0},
};

static struct mddi_table hitachi_deinit_tb[] = {
	/* switching (b)->(a) */
	{0x07, 0x0012, 30}, /* wait 2 frames */
	{0x11, 0x0F1A, 10},
	{0x11, 0x0712, 10},
	{0x11, 0x0312, 10},
	{0x11, 0x0110, 10},
	{0x07, 0x0000,  0},
};

static struct mddi_table wintek_init_tb[] = {
	{0x0F, 0x0b01,  0},	/*set frame rate to 68.2Hz*/
	{0x11, 0x0019,  0},
	{0x12, 0x1101,  0},
	{0x13, 0x007F,  0},

	{0x14, 0x6b68,  0}, /* flicker */
	{0x10, 0x0500,  0},
	{0x15, 0x0040,  0}, /* flicker */

	{0x11, 0x011a, 10},
	{0x11, 0x031a, 10},
	{0x11, 0x071a, 10},
	{0x11, 0x0F1a, 20},
	{0x11, 0x0F3a, 30},

	{0x01, 0x0128,  0},
	{0x02, 0x0100,  0},
	{0x03, 0x1030,  0},
	{0x08, 0x0404,  0},
	{0x0B, 0x1100,  0},
	{0x0C, 0x0000,  0},
	{0x30, 0x0000,  0},

	{0x36, 0x00EF,  0},
	{0x37, 0x0000,  0},
	{0x38, 0x013F,  0},
	{0x39, 0x0000,  0},
	/* Jay, for adding delay after vsync */
	{0x29, 0x64, 0},
	/* gamma curve */
	{0x50, 0x0105,  0},
	{0x51, 0x0808,  0},
	{0x52, 0x0f0D,  0},
	{0x53, 0x0604,  0},
	{0x54, 0x0D0f,  0},
	{0x55, 0x0808,  0},
	{0x56, 0x0501,  0},
	{0x57, 0x0406,  0},
	{0x58, 0x0600,  0},
	{0x59, 0x0006,  0},

	{0x07, 0x1012, 40},
	{0x07, 0x1013,  0},

	{0x20, 0x0000,  0},
	{0x21, 0x0000,  0},
	{0x22, 0x0000,  0},
};

static struct mddi_table wintek_deinit_tb[] = {
	{0x07, 0x1012, 32},
	{0x07, 0x1000, 16},
	{0x10, 0x0001, 20},
};

static struct mddi_table samsung_init_tb[] = {
	/* Power on sequence */
	{0x0f, 0x0f01, 0},
	{0x0b, 0x8101, 0},	/*set frame rate to 69Hz*/
	/* Power setting sequence */
	{0x11, 0x0018, 0},
	{0x12, 0x3101, 0},
	{0x13, 0x0061, 0},
	{0x14, 0x547f, 0},
	{0x10, 0x0800, 10},
	{0x11, 0x0118, 10},
	{0x11, 0x0318, 10},
	{0x11, 0x0718, 10},
	{0x11, 0x0f18, 20},
	{0x11, 0x0f38, 30},
	/* Initialzing sequence */
	{0x01, 0x0128, 0},
	{0x02, 0x0100, 0},
	{0x03, 0x1030, 0},
	{0x07, 0x0000, 0},
	{0x08, 0x0808, 0},
	{0x0c, 0x0000, 0},
	{0x15, 0x0050, 0},
	/* Jay, for adding delay after vsync */
	{0x29, 0x64, 0},
	{0x30, 0x0000, 0},
	{0x34, 0x013f, 0},
	{0x35, 0x0000, 0},
	{0x36, 0x00ef, 0},
	{0x37, 0x0000, 0},
	{0x38, 0x013f, 0},
	{0x39, 0x0000, 0},
	{0x50, 0x0301, 0},
	{0x51, 0x040e, 0},
	{0x52, 0x0b05, 0},
	{0x53, 0x0600, 0},
	{0x54, 0x050b, 0},
	{0x55, 0x0e04, 0},
	{0x56, 0x0103, 0},
	{0x57, 0x0006, 0},
	{0x58, 0x0f1a, 0},
	{0x59, 0x001b, 0},
	{0x07, 0x0012, 40},
	{0x07, 0x1013, 0},
};

static struct mddi_table samsung_deinit_tb[] = {
	{0x07, 0x0012, 40},
	{0x07, 0x0000, 20},
} ;

#define GPIOSEL_VWAKEINT (1U << 0)
#define INTMASK_VWAKEOUT (1U << 0)

static void
eid_process_mddi_table(struct msm_mddi_client_data *client_data,
		struct mddi_table *table, ssize_t count)
{
	int i;
	uint32_t reg, value, msec;

	BUG_ON(!client_data);
	BUG_ON(!table);
	BUG_ON(!count);

	for (i = 0; i < count; i++) {
		reg = table[i].reg;
		value = table[i].value;
		msec = table[i].msec;

		client_data->remote_write(client_data, value, reg);
		if (msec)
			hr_msleep(msec);
	}
}

static struct vreg *vreg_lcm_2v85;
#if defined(CONFIG_MACH_BAHAMAS) || defined(CONFIG_MACH_CLICKC)
static struct vreg *vreg_lcm_2v6;
#endif

static void
eid_mddi_power_client(struct msm_mddi_client_data *cdata, int on)
{
	unsigned id, on_off;

	B("%s: enter.\n", __func__);
	if (on) {
		on_off = 0;
		id = PM_VREG_PDOWN_MDDI_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

		gpio_set_value(BAHAMAS_V_VDDE2E_VDD2_GPIO, 1);

		hr_msleep(3);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		if (machine_is_bahamas() || machine_is_clickc()) {
			hr_msleep(1);
			id = PM_VREG_PDOWN_RFRX2_ID;
			msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
			vreg_enable(vreg_lcm_2v85);
		}
		hr_msleep(3);
		gpio_set_value(BAHAMAS_MDDI_RSTz, 1);
		hr_msleep(10);
	} else {
		gpio_set_value(BAHAMAS_MDDI_RSTz, 0);
		hr_msleep(10);
		on_off = 1;
		if (machine_is_bahamas() || machine_is_clickc()) {
			vreg_disable(vreg_lcm_2v6);
			id = PM_VREG_PDOWN_RFRX2_ID;
			msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
			hr_msleep(1);
		}
		vreg_disable(vreg_lcm_2v85);
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		hr_msleep(5);

		gpio_set_value(BAHAMAS_V_VDDE2E_VDD2_GPIO, 0);
		hr_msleep(200);

		id = PM_VREG_PDOWN_MDDI_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
	}
}

enum {
	PANEL_HITACHI = 0,
	PANEL_WINTEK,
	PANEL_SAMSUNG,
};

#if defined(CONFIG_ARCH_MSM7225)
#define LCM_ID0 57
#define LCM_ID1 58
#else
#define LCM_ID0 _bad_id()
#define LCM_ID1 _bad_id()
#endif

static int
eid_panel_detect(void)
{
	int panel_id = -1 ;

	panel_id = ((gpio_get_value(LCM_ID1) << 1) | gpio_get_value(LCM_ID0));

	switch (panel_id) {
	case PANEL_HITACHI:
		break ;
	case PANEL_WINTEK:
		break ;
	case PANEL_SAMSUNG:
		break ;
	default:
		printk("%s: Invalid panel id !\n", __func__);
		break ;
	}

	return panel_id ;
}

static int
eid_mddi_client_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int panel_id;

	client_data->auto_hibernate(client_data, 0);
	panel_id = eid_panel_detect();

	switch (panel_id) {
	case PANEL_HITACHI:
		B("found hitachi mddi panel\n");
		eid_process_mddi_table(client_data,
				hitachi_init_tb,
				ARRAY_SIZE(hitachi_init_tb));
		break;
	case PANEL_WINTEK:
		B("found wintek mddi panel\n");
		eid_process_mddi_table(client_data,
				wintek_init_tb,
				ARRAY_SIZE(wintek_init_tb));
		break;
	case PANEL_SAMSUNG:
		B("found samsung mddi panel\n");
		eid_process_mddi_table(client_data,
				samsung_init_tb,
				ARRAY_SIZE(samsung_init_tb));
		break;
	default:
		B("unknown panel_id: %d\n", panel_id);
	};
	client_data->auto_hibernate(client_data, 1);

	return 0;
}

static int
eid_mddi_client_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	return 0;
}

static struct mddi_table hitachi_adjust_tb[] = {
	{0x36, 0x00EF, 0},
	{0x37, 0x0000, 0},
	{0x38, 0x013F, 0},
	{0x39, 0x0000, 0}, /* wait 2 frames */
	{0x20, 0x0000, 0},
	{0x21, 0x0000, 0},
};

static struct mddi_table wintek_adjust_tb[] = {
	{0x36, 0x00EF,  0},
	{0x37, 0x0000,  0},
	{0x38, 0x013F,  0},
	{0x39, 0x0000,  0},
	{0x20, 0x0000,  0},
	{0x21, 0x0000,  0},
	{0x22, 0x0000,  0},
};

static struct mddi_table samsung_adjust_tb[] = {
	{0x36, 0x00EF, 0},
	{0x37, 0x0000, 0},
	{0x38, 0x013F, 0},
	{0x39, 0x0000, 0},
	{0x20, 0x0000, 0},
	{0x21, 0x0000, 0},
};

static void
eid_samsung_adjust(struct msm_mddi_client_data *client_data)
{
	int panel_id;

	BUG_ON(!client_data);

	panel_id = eid_panel_detect();
	switch (panel_id) {
	case PANEL_HITACHI:
		B("hitachi panel\n");
		eid_process_mddi_table(client_data,
				hitachi_adjust_tb,
				ARRAY_SIZE(hitachi_adjust_tb));
		break;
	case PANEL_WINTEK:
		B("wintek panel\n");
		eid_process_mddi_table(client_data,
				wintek_adjust_tb,
				ARRAY_SIZE(wintek_adjust_tb));
		break;
	case PANEL_SAMSUNG:
		B("samsung panel\n");
		eid_process_mddi_table(client_data,
				samsung_adjust_tb,
				ARRAY_SIZE(samsung_adjust_tb));
		break;
	default:
		break;
	}
}

static int panel_inited = 0;
static int
eid_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		    struct msm_mddi_client_data *client_data)
{
	int panel_id;
	BUG_ON(!bridge_data);
	BUG_ON(!client_data);
	B("%s: enter.\n", __func__);

	client_data->auto_hibernate(client_data, 0);
	panel_id = eid_panel_detect();

	if (!panel_inited) {
		panel_inited = 1;
	} else {
		switch (panel_id) {
		case PANEL_HITACHI:
			B("found hitachi mddi panel\n");
			eid_process_mddi_table(client_data,
					hitachi_init_tb,
					ARRAY_SIZE(hitachi_init_tb));
			break;
		case PANEL_WINTEK:
			B("found wintek mddi panel\n");
			eid_process_mddi_table(client_data,
					wintek_init_tb,
					ARRAY_SIZE(wintek_init_tb));
			break;
		case PANEL_SAMSUNG:
			B("found samsung mddi panel\n");
			eid_process_mddi_table(client_data,
					samsung_init_tb,
					ARRAY_SIZE(samsung_init_tb));
			break;
		default:
			B("unknown panel_id: %d\n", panel_id);
		};
	}
	eid_set_backlight(1);
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
eid_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	int panel_id = eid_panel_detect();

	eid_set_backlight(0);
	client_data->auto_hibernate(client_data, 0);

	switch (panel_id) {
	case PANEL_HITACHI:
		eid_process_mddi_table(client_data, hitachi_deinit_tb,
				ARRAY_SIZE(hitachi_deinit_tb));
		break;
	case PANEL_WINTEK:
		eid_process_mddi_table(client_data, wintek_deinit_tb,
				ARRAY_SIZE(wintek_deinit_tb));
		break;
	case PANEL_SAMSUNG:
		eid_process_mddi_table(client_data, samsung_deinit_tb,
				ARRAY_SIZE(samsung_deinit_tb));
		break;
	default:
		B("unknown panel_id: %d\n", panel_id);
	}
	client_data->auto_hibernate(client_data, 1);

	return 0;
}

static void
eid_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B("%s: enter.\n", __func__);
	*mfr_name = 0x0101;
	*product_code = 0x0154;
}

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_mddi_bridge_platform_data samsung_client_data = {
	.init = eid_mddi_client_init,
	.uninit = eid_mddi_client_uninit,
	.bridge_type = SAMSUNG_D,
	.blank = eid_panel_blank,
	.unblank = eid_panel_unblank,
	.adjust = eid_samsung_adjust,
	.fb_data = {
		.xres = 240,
		.yres = 320,
		.width = 42,
		.height = 56,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data eid_pdata = {
	.clk_rate = 68571000,
	.power_client = eid_mddi_power_client,
	.fixup = eid_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0x0101 << 16 | 0x0154),
			.name = "mddi_c_0101_0154",
			.id = 0,
			.client_data = &samsung_client_data,
			.clk_rate = 0,
		},
	},
};

static struct msm_mdp_platform_data mdp_pdata = {
        .color_format = MSM_MDP_OUT_IF_FMT_RGB565,
};

int __init bahamas_init_panel(void)
{
	int rc;

	if (!(machine_is_bahamas() || machine_is_clickc() ||
				machine_is_memphis()))
		return 0;

	vreg_lcm_2v85 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);
	vreg_lcm_2v6 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &eid_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	led_trigger_register_simple("lcd-backlight-gate", &eid_lcd_backlight);
	if (IS_ERR(eid_lcd_backlight))
		printk(KERN_ERR
			"%s: backlight registration failed!\n", __func__);

	return 0;
}
