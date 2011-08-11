/* arch/arm/mach-msm/htc_tp_cal.c
 *
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
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/setup.h>

/* configuration tags specific to TP */
#define ATAG_TP	0x41387898 /* TP */
#define MAX_CALI_SIZE	0x20U
#define ATAG_TP_DEBUG

static unsigned char tp_cal_ram[MAX_CALI_SIZE];

unsigned char *get_tp_cal_ram(void)
{
	return(tp_cal_ram);
}
EXPORT_SYMBOL(get_tp_cal_ram);

static int __init parse_tag_tp(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;

#ifdef ATAG_TP_DEBUG
	unsigned i;
	unsigned char *ptr;
#endif
	size = min((tag->hdr.size - 2) * sizeof(__u32), MAX_CALI_SIZE);

	printk(KERN_INFO "TP Data size = %d , 0x%x, size = %d\n",
			tag->hdr.size, tag->hdr.tag, size);

#ifdef ATAG_TP_DEBUG
	ptr = dptr;
	printk(KERN_INFO
	       "TP Data size = %d , 0x%x\n",
	       tag->hdr.size, tag->hdr.tag);
	for (i = 0; i < size; i++)
		printk(KERN_INFO "%02x ", *ptr++);
#endif
	memcpy((void *)tp_cal_ram, (void *)dptr, size);
	return 0;
}

__tagtable(ATAG_TP, parse_tag_tp);
