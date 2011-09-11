/* arch/arm/mach-msm/board-bahamas.c
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>

#include <mach/board.h>

#if defined(CONFIG_MT9T013_CLICK)
#include "../../../../drivers/media/video/msm/click/include/mach/camera.h"
#else
#include <mach/camera.h>
#endif

#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/msm_serial_debugger.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>

#include "board-bahamas.h"
#include "proc_comm.h"
#include "gpio_chip.h"

#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>

#include "devices.h"

#include <mach/h2w_v1.h>
#include <mach/audio_jack.h>
#include <mach/microp_i2c.h>
#include <mach/htc_battery.h>
#include <mach/msm_tssc.h>
#include <mach/htc_pwrsink.h>

#if defined(CONFIG_PERFLOCK)
#include <mach/perflock.h>
#endif

#include <mach/drv_callback.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/htc_usb.h>

static int bahamas_phy_init_seq[] = {0x40, 0x31, 0x1D, 0x0D, 0x1D, 0x10, -1};

#define HSUSB_API_INIT_PHY_PROC 2
#define HSUSB_API_PROG		0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1, 1)

static void bahamas_phy_reset(void)
{
	struct msm_rpc_endpoint *usb_ep;
	int rc;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	printk(KERN_INFO "msm_hsusb_phy_reset\n");

	usb_ep = msm_rpc_connect(HSUSB_API_PROG, HSUSB_API_VERS, 0);
	if (IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(usb_ep));
		return;
	}
	rc = msm_rpc_call(usb_ep, HSUSB_API_INIT_PHY_PROC,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

	msm_rpc_close(usb_ep);
}

enum {
	PANEL_HITACHI = 0,
	PANEL_WINTEK,
	PANEL_SAMSUNG,
};

#define LCM_ID0 57
#define LCM_ID1 58

static int panel_detect(void)
{
	int panel_id = -1 ;

	panel_id = ((gpio_get_value(LCM_ID1) << 1) | gpio_get_value(LCM_ID0));

	switch(panel_id) {
	case PANEL_HITACHI:
		break ;
	case PANEL_WINTEK:
		break ;
	case PANEL_SAMSUNG:
		break ;
	default:
		printk("%s: Invalid panel id !\n", __FUNCTION__ ) ;
		break ;
	}

	return panel_id ;
}

// Samsung panel
static struct microp_pin_config microp_pins_0[] = {
	{
		.name	= "green",
		.pin	= 3,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "amber",
		.pin	= 5,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "lcd-backlight",
		.pin	= 6,
		.config	= MICROP_PIN_CONFIG_PWM,
		.freq	= MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels	= { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 180, 207, 235, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config	= MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "adc",
		.pin	= 24,
		.config	= MICROP_PIN_CONFIG_ADC,
		.levels	= { 0, 0, 0, 6, 51, 319, 425, 497, 569, 638 },
	}
};

// Wintek panel
static struct microp_pin_config microp_pins_0_wint[] = {
	{
		.name	= "green",
		.pin	= 3,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "amber",
		.pin	= 5,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "lcd-backlight",
		.pin	= 6,
		.config	= MICROP_PIN_CONFIG_PWM,
		.freq	= MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels	= { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 176, 201, 226, 240 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config	= MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "adc",
		.pin	= 24,
		.config	= MICROP_PIN_CONFIG_ADC,
		.levels	= { 0, 0, 0, 6, 51, 319, 425, 497, 569, 638 },
	}
};

// XD after, Samsung panel
static struct microp_pin_config microp_pins_1[] = {
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(7, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(8, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name	= "microp-pullup",
		.pin	= 23,
		.config	= MICROP_PIN_CONFIG_PULL_UP1,
		.mask	= { 0x00, 0x00, 0x02 },
	},
	{
		.name	= "green",
		.pin	= 3,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "amber",
		.pin	= 5,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "lcd-backlight",
		.pin	= 6,
		.config	= MICROP_PIN_CONFIG_PWM,
		.freq	= MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels	= { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 180, 207, 235, 255 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config	= MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "microp_11pin_mic",
		.pin	= 16,
		.config	= MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 17,
		.config	= MICROP_PIN_CONFIG_INTR_ALL,
		.mask	= { 0x00, 0x00, 0x00 },
	},
	{
		.name	= "audio_remote_sensor",
		.pin	= 25,
		.adc_pin= 7,
		.config	= MICROP_PIN_CONFIG_ADC_READ,
	}
};

// XD after, Wintek panel
static struct microp_pin_config microp_pins_1_wint[] = {
	MICROP_PIN(0, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(1, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(2, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(4, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(7, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(8, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(9, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(11, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(12, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(13, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(14, MICROP_PIN_CONFIG_GPO),
	MICROP_PIN(15, MICROP_PIN_CONFIG_GPO),
	{	.name	= "microp-pullup",
		.pin	= 23,
		.config	= MICROP_PIN_CONFIG_PULL_UP1,
		.mask	= { 0x00, 0x00, 0x02 },
	},
	{
		.name	= "green",
		.pin	= 3,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "amber",
		.pin	= 5,
		.config	= MICROP_PIN_CONFIG_GPO_INV,
	},
	{
		.name	= "lcd-backlight",
		.pin	= 6,
		.config	= MICROP_PIN_CONFIG_PWM,
		.freq	= MICROP_PIN_PWM_FREQ_HZ_15600,
		.levels	= { 3, 31, 59, 87, 115, 143, 171, 199, 227, 255 },
		.dutys	= { 10, 16, 39, 74, 118, 154, 176, 201, 226, 240 },
	},
	{
		.name	= "button-backlight",
		.pin	= 10,
		.config	= MICROP_PIN_CONFIG_GPO,
	},
	{
		.name	= "microp_11pin_mic",
		.pin	= 16,
		.config	= MICROP_PIN_CONFIG_MIC,
		.init_value = 1,
	},
	{
		.name	= "microp_intrrupt",
		.pin	= 17,
		.config	= MICROP_PIN_CONFIG_INTR_ALL,
		.mask	= { 0x00, 0x00, 0x00 },
	},
	{
		.name	= "audio_remote_sensor",
		.pin	= 25,
		.adc_pin= 7,
		.config	= MICROP_PIN_CONFIG_ADC_READ,
	}
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver	= GUAGE_MODEM,
	.charger	= LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name	= "htc_battery",
	.id	= -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

#if defined(CONFIG_MT9T013_CLICK)
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013 = {
	.sensor_reset	= 118,
	.sensor_pwd	= BAHAMAS_MT9T013_CAM_PWDN,
	.sensor_name	= "mt9t013",
	.sensor_probe	= mt9t013_probe_init,
};

static struct msm_camera_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_bahamas_camera_on_gpios,
	.camera_gpio_off = config_bahamas_camera_off_gpios,
	.snum = 1,
	.sinfo = &msm_camera_sensor_mt9t013,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct platform_device msm_camera_device = {
	.name   = "msm_camera",
	.id     = -1,
	.dev    = {
		.platform_data = &msm_camera_device_data,
	},
};
#endif

static struct akm8973_platform_data compass_platform_data = {
	.layouts	= BAHAMAS_LAYOUTS,
	.project_name	= BAHAMAS_PROJECT_NAME,
	.reset		= BAHAMAS_GPIO_COMPASS_RST_N,
	.intr		= BAHAMAS_GPIO_COMPASS_INT_N,
};

static struct bma150_platform_data bahamas_g_sensor_pdata = {
	.intr = BAHAMAS_GPIO_GSENSOR_INT_N,
};

static struct microp_i2c_platform_data microp_data = {
        .num_pins = ARRAY_SIZE(microp_pins_1),
        .pin_config = microp_pins_1,
        .gpio_reset = BAHAMAS_GPIO_UP_RESET_N,
	.microp_enable_early_suspend = 1,
        .microp_enable_reset_button = 1,
};

static struct i2c_board_info i2c_microp_devices = {
	I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
	.platform_data = &microp_data,
	.irq = MSM_GPIO_TO_INT(BAHAMAS_GPIO_UP_INT),
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
		.platform_data = &msm_camera_device_data,
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BAHAMAS_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO(BMA150_G_SENSOR_NAME, 0x38),
		.platform_data = &bahamas_g_sensor_pdata,
		.irq = MSM_GPIO_TO_INT(BAHAMAS_GPIO_GSENSOR_INT_N),
	},
};

#if defined(CONFIG_USB_ANDROID)
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq = bahamas_phy_init_seq,
	.phy_reset = bahamas_phy_reset,
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#if defined(CONFIG_USB_ANDROID_RNDIS)
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "Google, Inc.",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct pwr_sink bahamas_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 100000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 125000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max	= 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static int bahamas_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void bahamas_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void bahamas_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int bahamas_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data bahamas_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(bahamas_pwrsink_table),
	.sinks		= bahamas_pwrsink_table,
	.suspend_late	= bahamas_pwrsink_suspend_late,
	.resume_early	= bahamas_pwrsink_resume_early,
	.suspend_early	= bahamas_pwrsink_suspend_early,
	.resume_late	= bahamas_pwrsink_resume_late,
};

static struct platform_device bahamas_pwr_sink = {
	.name	= "htc_pwrsink",
	.id	= -1,
	.dev	= {
		.platform_data = &bahamas_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting_monodie = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE_MONODIE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE_MONODIE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

static struct msm_pmem_setting pmem_setting_dualdie = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE_DUALDIE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE_DUALDIE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

/* Switch between UART3 and GPIO */
static uint32_t uart3_on_gpio_table[] = {
	/* RX */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_RX, 1, GPIO_INPUT, GPIO_NO_PULL, 0),
	/* TX */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_TX, 1, GPIO_OUTPUT, GPIO_NO_PULL, 0),
};

/* Set TX,RX to GPI */
static uint32_t uart3_off_gpio_table[] = {
	PCOM_GPIO_CFG(BAHAMAS_GPIO_H2W_DATA, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* RX, H2W DATA */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_H2W_CLK, 0,
		      GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* TX, H2W CLK */
};

static int bahamas_h2w_path = H2W_GPIO;

static void h2w_configure(int route)
{
	printk(KERN_INFO "H2W route = %d \n", route);
	switch (route) {
	case H2W_UART3:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_on_gpio_table+1, 0);
		bahamas_h2w_path = H2W_UART3;
		printk(KERN_INFO "H2W -> UART3\n");
		break;
	case H2W_GPIO:
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpio_table+0, 0);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX,
			      uart3_off_gpio_table+1, 0);
		bahamas_h2w_path = H2W_GPIO;
		printk(KERN_INFO "H2W -> GPIO\n");
		break;
	}
}

static void h2w_defconfig(void)
{
	h2w_configure(H2W_GPIO);
}

static void set_h2w_dat(int n)
{
	gpio_set_value(BAHAMAS_GPIO_H2W_DATA, n);
}

static void set_h2w_clk(int n)
{
	gpio_set_value(BAHAMAS_GPIO_H2W_CLK, n);
}

static void set_h2w_dat_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(BAHAMAS_GPIO_H2W_DATA);
	else
		gpio_configure(BAHAMAS_GPIO_H2W_DATA, GPIOF_DRIVE_OUTPUT);
}

static void set_h2w_clk_dir(int n)
{
	if (n == 0) /* input */
		gpio_direction_input(BAHAMAS_GPIO_H2W_CLK);
	else
		gpio_configure(BAHAMAS_GPIO_H2W_CLK, GPIOF_DRIVE_OUTPUT);
}

static int get_h2w_dat(void)
{
	return gpio_get_value(BAHAMAS_GPIO_H2W_DATA);
}

static int get_h2w_clk(void)
{
	return gpio_get_value(BAHAMAS_GPIO_H2W_CLK);
}

#if defined(CONFIG_HTC_HEADSET_V1)
static int set_h2w_path(const char *val, struct kernel_param *kp)
{
	int ret = -EINVAL;
	int enable;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (bahamas_h2w_path) {
	case H2W_GPIO:
		enable = 1;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	case H2W_UART3:
		enable = 0;
		cnf_driver_event("H2W_enable_irq", &enable);
		break;
	default:
		bahamas_h2w_path = -1;
		return -EINVAL;
	}

	h2w_configure(bahamas_h2w_path);
	return ret;
}

module_param_call(h2w_path, set_h2w_path, param_get_int,
		&bahamas_h2w_path, S_IWUSR | S_IRUGO);
#endif

static struct h2w_platform_data bahamas_h2w_data = {
	.h2w_power		= BAHAMAS_GPIO_H2W_POWER,
	.cable_in1		= BAHAMAS_GPIO_CABLE_IN1,
	.cable_in2		= BAHAMAS_GPIO_CABLE_IN2,
	.h2w_clk		= BAHAMAS_GPIO_H2W_CLK,
	.h2w_data		= BAHAMAS_GPIO_H2W_DATA,
	.headset_mic_35mm	= BAHAMAS_GPIO_HEADSET_MIC,
	.ext_mic_sel		= BAHAMAS_GPIO_AUD_EXTMIC_SEL,
	.wfm_ant_sw		= BAHAMAS_GPIO_WFM_ANT_SW,
	.debug_uart 		= H2W_UART3,
	.config 		= h2w_configure,
	.defconfig 		= h2w_defconfig,
	.set_dat		= set_h2w_dat,
	.set_clk		= set_h2w_clk,
	.set_dat_dir		= set_h2w_dat_dir,
	.set_clk_dir		= set_h2w_clk_dir,
	.get_dat		= get_h2w_dat,
	.get_clk		= get_h2w_clk,
};

static struct platform_device bahamas_h2w = {
	.name		= "h2w",
	.id		= -1,
	.dev		= {
		.platform_data = &bahamas_h2w_data,
	},
};

static struct audio_jack_platform_data bahamas_jack_data = {
	.gpio	= BAHAMAS_GPIO_35MM_HEADSET_DET,
};

static struct platform_device bahamas_audio_jack = {
	.name		= "audio-jack",
	.id		= -1,
	.dev		= {
		.platform_data = &bahamas_jack_data,
	},
};

static struct tssc_ts_platform_data tssc_ts_device_data = {
	.version	= 1,
	.x_min		= 0,
	.x_max		= 1023,
	.y_min		= 0,
	.y_max		= 1023,
	.cal_range_x	= 653,
	.cal_range_y	= 672,
	.cal_err	= 50,
	.screen_width	= 240,
	.screen_height	= 400,
	.cal_x		= { 15, 223, 15, 223, 119},
	.cal_y		= { 20, 20, 378, 378, 199},
	.abs_pressure_min = 0,
	.abs_pressure_max = 255,
};

static struct platform_device tssc_ts_device = {
	.name   = "tssc-manager",
	.id     = -1,
	.dev    = {
		.platform_data = &tssc_ts_device_data,
	},
};

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock	= 100000,
	.clock_strength	= GPIO_8MA,
	.data_strength	= GPIO_4MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct platform_device bahamas_rfkill = {
	.name = "bahamas_rfkill",
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
	&bahamas_h2w,
	&htc_battery_pdev,
	&tssc_ts_device,
#if defined(CONFIG_MT9T013_CLICK)
	&msm_camera_device,
#endif
	&bahamas_rfkill,
	&bahamas_audio_jack,
#if defined(CONFIG_HTC_PWRSINK)
	&bahamas_pwr_sink,
#endif
};

extern struct sys_timer msm_timer;

static void __init bahamas_init_irq(void)
{
	printk("bahamas_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint cpld_usb_h2w_sw;
static uint opt_disable_uart3;
static char *keycaps = "";

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);
module_param_named(keycaps, keycaps, charp, 0);

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void bahamas_reset(void)
{
	gpio_set_value(BAHAMAS_GPIO_PS_HOLD, 0);
}

static uint32_t gpio_table[] = {
	/* BLUETOOTH */
	#if defined(CONFIG_SERIAL_MSM_HS)
	PCOM_GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RTS */
	PCOM_GPIO_CFG(44, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA), /* CTS */
	PCOM_GPIO_CFG(45, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* RX */
	PCOM_GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* TX */
	#else
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_RTS */
	PCOM_GPIO_CFG(44, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_CTS */
	PCOM_GPIO_CFG(45, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_RX */
	PCOM_GPIO_CFG(46, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* BT_TX */
	#endif
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

void config_bahamas_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_bahamas_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static uint32_t bahamas_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(BAHAMAS_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void bahamas_config_serial_debug_gpios(void)
{
	config_gpio_table(bahamas_serial_debug_table,
			ARRAY_SIZE(bahamas_serial_debug_table));
}

static void __init config_gpios(void)
{
	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));
	bahamas_config_serial_debug_gpios();
	config_bahamas_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data bahamas_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
#if defined(CONFIG_TURBO_MODE)
	.wait_for_irq_khz = 176000,
#else
	.wait_for_irq_khz = 128000,
#endif
};

#if defined(CONFIG_PERFLOCK)
static unsigned bahamas_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data bahamas_perflock_data = {
	.perf_acpu_table = bahamas_perf_acpu_table,
	.table_size = ARRAY_SIZE(bahamas_perf_acpu_table),
};
#endif

#if defined(CONFIG_SERIAL_MSM_HS)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(BAHAMAS_GPIO_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

static struct vreg *vreg_h2w;
static int h2w_power_configure(struct gpio_chip *chip,
			       unsigned int gpio,
			       unsigned long flags)
{
	if ((flags & GPIOF_DRIVE_OUTPUT) && !vreg_h2w)
		vreg_h2w = vreg_get(0, BAHAMAS_H2W_POWER_NAME);

	if ((flags & GPIOF_OUTPUT_HIGH) && vreg_h2w) {
		vreg_enable(vreg_h2w);
		vreg_set_level(vreg_h2w, 3000);
	} else if ((flags & GPIOF_OUTPUT_LOW) && vreg_h2w)
		vreg_disable(vreg_h2w);

	return 0;
}

static int h2w_power_get_irq_num(struct gpio_chip *chip,
				 unsigned int gpio,
				 unsigned int *irqp,
				 unsigned long *irqnumflagsp)
{
	return -1;
}

static int h2w_power_read(struct gpio_chip *chip, unsigned n)
{
	return -1;
}

static int h2w_power_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	if (!vreg_h2w)
		return -1;

	if (on) {
		vreg_enable(vreg_h2w);
		vreg_set_level(vreg_h2w, 3000);
	} else
		vreg_disable(vreg_h2w);
	return 0;
}

static struct gpio_chip bahamas_h2w_gpio_chip = {
	.start = BAHAMAS_GPIO_H2W_POWER,
	.end = BAHAMAS_GPIO_H2W_POWER,
	.configure = h2w_power_configure,
	.get_irq_num = h2w_power_get_irq_num,
	.read = h2w_power_read,
	.write = h2w_power_write,
};

void bahamas_init_h2w_power_gpio(void)
{
	register_gpio_chip(&bahamas_h2w_gpio_chip);
}

static void __init bahamas_init(void)
{
	int rc;
	char *cid = NULL;

	printk("bahamas_init() revision = 0x%X\n", system_rev);
	printk(KERN_INFO "mfg_mode=%d\n", board_mfg_mode());
	msm_clock_init();
	board_get_cid_tag(&cid);

	/*
	* Setup common MSM GPIOS
	*/
	config_gpios();

	gpio_request(BAHAMAS_GPIO_CABLE_IN2, "gpio_cable_in2");
	gpio_request(BAHAMAS_GPIO_AUD_EXTMIC_SEL, "gpio_aud_extmic_sel");
	gpio_request(BAHAMAS_GPIO_WFM_ANT_SW, "gpio_wfm_ant_sw");

	msm_hw_reset_hook = bahamas_reset;

	msm_acpu_clock_init(&bahamas_clock_data);
#if defined(CONFIG_PERFLOCK)
	perflock_init(&bahamas_perflock_data);
#endif

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(BAHAMAS_GPIO_UART3_RX));
#endif

	msm_add_devices();

#if defined(CONFIG_SERIAL_MSM_HS)
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
#if defined(CONFIG_USB_FUNCTION)
	msm_register_usb_phy_init_seq(bahamas_phy_init_seq);
	msm_add_usb_devices(bahamas_phy_reset, NULL);
#endif

#if defined(CONFIG_USB_ANDROID)
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
#if defined(CONFIG_USB_ANDROID_RNDIS)
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif
	if (board_mcp_monodie())
		msm_add_mem_devices(&pmem_setting_monodie);
	else
		msm_add_mem_devices(&pmem_setting_dualdie);

	bahamas_init_h2w_power_gpio();

	rc = bahamas_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	if(system_rev < 3) {
		if (panel_detect() == PANEL_WINTEK) {
			microp_data.num_pins	= ARRAY_SIZE(microp_pins_0_wint);
			microp_data.pin_config	= microp_pins_0_wint;
		} else {
			microp_data.num_pins	= ARRAY_SIZE(microp_pins_0);
			microp_data.pin_config	= microp_pins_0;
		}
			i2c_microp_devices.irq = 0;
		} else if (panel_detect() == PANEL_WINTEK) {
			microp_data.num_pins	= ARRAY_SIZE(microp_pins_1_wint);
			microp_data.pin_config	= microp_pins_1_wint;
	}

	msm_device_i2c_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));

	if (system_rev >= 3)
		bahamas_h2w_data.flags |= _35MM_MIC_DET_L2H;

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	i2c_register_board_info(0 ,&i2c_microp_devices, 1);

	bahamas_init_keypad();
	bahamas_init_panel();

	msm_init_pmic_vibrator(3000);
}

static void __init bahamas_fixup(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
	if (board_mcp_monodie()) {
		mi->nr_banks=1;
		mi->bank[0].start = MSM_LINUX_BASE1;
		mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
		mi->bank[0].size = MSM_LINUX_SIZE1+MSM_LINUX_SIZE2;
	}
	else {
		mi->nr_banks=2;
		mi->bank[0].start = MSM_LINUX_BASE1;
		mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
		mi->bank[0].size = MSM_LINUX_SIZE1;
		mi->bank[1].start = MSM_LINUX_BASE2_DUALDIE;
		mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2_DUALDIE);
		mi->bank[1].size = MSM_LINUX_SIZE2;
	}
}

static void __init bahamas_map_io(void)
{
	printk("bahamas_init_map_io()\n");
	msm_map_common_io();
}

MACHINE_START(BAHAMAS, "bahamas")
#if defined(CONFIG_MSM_DEBUG_UART)
	.phys_io	= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x02E00100,
	.fixup		= bahamas_fixup,
	.map_io		= bahamas_map_io,
	.init_irq	= bahamas_init_irq,
	.init_machine	= bahamas_init,
	.timer		= &msm_timer,
MACHINE_END
