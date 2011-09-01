/* drivers/video/msm_fb/mddi_client_s6d0154.c
 *
 * Support for eid mddi client devices with Samsung S6D0154
 *
 * Copyright (C) 2007 HTC Incorporated
 * Author: Wade Wu (wade_wu@htc.com)
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_fb.h>
#include <mach/msm_panel.h>
#include <mach/msm_iomap.h>
#include <mach/debug_display.h>


#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

#define DEBUG_VSYNC_INT 0
#define VSYNC_COUNTER 31

#define VSYNC_CLEAR (MSM_GPIO1_BASE + 0x800 + 0x9c)
#define VSYNC_STATUS (MSM_GPIO1_BASE + 0x800 + 0xac)
#define VSYNC_EN (MSM_GPIO1_BASE + 0x800 + 0x8c)

static DECLARE_WAIT_QUEUE_HEAD(samsung_vsync_wait);

#define INTERVAL_ADJUSTING	300

struct panel_info {
	struct msm_mddi_client_data *client_data;
	struct platform_device pdev;
	struct msm_panel_data panel_data;
	struct work_struct adjust_panel_work;
	struct msmfb_callback *fb_callback;
	struct wake_lock idle_lock;
	atomic_t frame_counter;
	int samsung_got_int;
};

static struct clk *ebi1_clk;

static void samsung_dump_vsync(void)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
}

static void samsung_adjust_work(struct work_struct *work){
	struct panel_info * panel = container_of(work, struct panel_info, adjust_panel_work);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
		client_data->private_client_data;

	if(bridge_data->adjust)
		bridge_data->adjust(client_data);

	atomic_set(&panel->frame_counter, INTERVAL_ADJUSTING);
}

static void
samsung_request_vsync(struct msm_panel_data *panel_data,
		      struct msmfb_callback *callback)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->fb_callback = callback;
	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}
}

static void samsung_wait_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	if (panel->samsung_got_int) {
		panel->samsung_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}

	if (wait_event_timeout(samsung_vsync_wait, panel->samsung_got_int,
				HZ/2) == 0)
		PR_DISP_ERR("timeout waiting for VSYNC\n");

	panel->samsung_got_int = 0;
	/* interrupt clears when screen dma starts */
}


/* -------------------------------------------------------------------------- */

/* got called by msmfb_suspend */
static int samsung_suspend(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	wake_lock(&panel->idle_lock);
	ret = bridge_data->uninit(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret) {
		B(KERN_INFO "mddi samsung client: non zero return from "
		  "uninit\n");
		return ret;
	}
	client_data->suspend(client_data);
	return 0;
}

/* got called by msmfb_resume */
static int samsung_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	client_data->resume(client_data);

	wake_lock(&panel->idle_lock);
	ret = bridge_data->init(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret)
		return ret;
	return 0;
}

static int samsung_blank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->blank(bridge_data, client_data);
}

static int samsung_unblank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->unblank(bridge_data, client_data);
}

static irqreturn_t samsung_vsync_interrupt(int irq, void *data)
{
	struct panel_info *panel = data;

	panel->samsung_got_int = 1;

	if (atomic_dec_and_test(&panel->frame_counter)) {
		schedule_work(&panel->adjust_panel_work);
		return IRQ_HANDLED;
	}

	if (panel->fb_callback  && atomic_read(&panel->frame_counter) > 0) {
		panel->fb_callback->func(panel->fb_callback);
		panel->fb_callback = NULL;
	}
	wake_up(&samsung_vsync_wait);

	return IRQ_HANDLED;
}

static int setup_vsync(struct panel_info *panel, int init)
{
	int ret;
	int gpio = 97;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, samsung_vsync_interrupt, IRQF_TRIGGER_RISING,
			"vsync", panel);
	if (ret)
		goto err_request_irq_failed;

	PR_DISP_INFO("vsync on gpio %d now %d\n", gpio,
			gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel->client_data);
err_request_irq_failed:
err_get_irq_num_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_samsung_probe(struct platform_device *pdev)
{
	int ret, err = -EINVAL;
	struct panel_info *panel;
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	B(KERN_DEBUG "%s: enter\n", __func__);

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (!panel) {
		err = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(pdev, panel);
	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_samsung_setup_vsync failed\n");
		err = -EIO;
		goto err_panel;
	}

	panel->client_data = client_data;
	panel->panel_data.suspend = samsung_suspend;
	panel->panel_data.resume = samsung_resume;
	panel->panel_data.wait_vsync = samsung_wait_vsync;
	panel->panel_data.request_vsync = samsung_request_vsync;
	panel->panel_data.blank = samsung_blank;
	panel->panel_data.unblank = samsung_unblank;
	panel->panel_data.dump_vsync = samsung_dump_vsync;
	panel->panel_data.fb_data = &bridge_data->fb_data;
	panel->panel_data.caps = ~MSMFB_CAP_PARTIAL_UPDATES;
	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	panel->pdev.resource = client_data->fb_resource;
	panel->pdev.num_resources = 1;
	panel->pdev.dev.platform_data = &panel->panel_data;
	platform_device_register(&panel->pdev);
	wake_lock_init(&panel->idle_lock, WAKE_LOCK_IDLE, "eid_idle_lock");
	/*for debugging vsync issue*/
	ebi1_clk = clk_get(NULL, "ebi1_clk");

	INIT_WORK(&panel->adjust_panel_work, samsung_adjust_work);
	atomic_set(&panel->frame_counter, INTERVAL_ADJUSTING);

	return 0;

err_panel:
	kfree(panel);
err_out:
	return err;
}

static int mddi_samsung_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);

	setup_vsync(panel, 0);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_0101_0000 = {
	.probe = mddi_samsung_probe,
	.remove = mddi_samsung_remove,
	.driver = {.name = "mddi_c_0101_0154"},
};

static int __init mddi_client_samsung_init(void)
{
	return platform_driver_register(&mddi_client_0101_0000);
}

module_init(mddi_client_samsung_init);
