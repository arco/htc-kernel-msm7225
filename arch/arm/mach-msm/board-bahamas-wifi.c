/* arch/arm/mach-msm/board-bahamas-wifi.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
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

#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#include <linux/wifi_tiwlan.h>

extern int bahamas_wifi_set_carddetect(int val);
extern int bahamas_wifi_power(int on);
extern int bahamas_wifi_reset(int on);

#ifdef CONFIG_WIFI_MEM_PREALLOC
typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[WMPA_NUMBER_OF_SECTIONS] = {
	{ NULL, (WMPA_SECTION_SIZE_0 + WMPA_SECTION_HEADER) },
	{ NULL, (WMPA_SECTION_SIZE_1 + WMPA_SECTION_HEADER) },
	{ NULL, (WMPA_SECTION_SIZE_2 + WMPA_SECTION_HEADER) }
};

static void *bahamas_wifi_mem_prealloc(int section, unsigned long size)
{
	if ((section < 0) || (section >= WMPA_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init bahamas_init_wifi_mem (void)
{
	int i;

	for (i = 0; (i < WMPA_NUMBER_OF_SECTIONS); i++) {
		wifi_mem_array[i].mem_ptr = vmalloc(wifi_mem_array[i].size);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}
#endif

struct wifi_platform_data bahamas_wifi_control = {
	.set_power		= bahamas_wifi_power,
	.set_reset		= bahamas_wifi_reset,
	.set_carddetect		= bahamas_wifi_set_carddetect,
#ifdef CONFIG_WIFI_MEM_PREALLOC
	.mem_prealloc		= bahamas_wifi_mem_prealloc,
#else
	.mem_prealloc		= NULL,
#endif
};

static struct platform_device bahamas_wifi = {
	.name		= "msm_wifi",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
	.dev		= {
		.platform_data = &bahamas_wifi_control,
	},
};

static int __init bahamas_wifi_init(void)
{
	int rc;
	int ret;

	printk("%s: start\n", __func__);

#ifdef CONFIG_WIFI_MEM_PREALLOC
	rc = bahamas_init_wifi_mem();
	if (rc) {
		printk(KERN_CRIT "%s: WiFi memory init failure (%d)\n",
			__func__, rc);
	}
#endif

	ret = platform_device_register(&bahamas_wifi);
	return ret;
}


late_initcall(bahamas_wifi_init);
#endif
