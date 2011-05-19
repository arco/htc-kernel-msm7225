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
#include <linux/switch.h>
#include <linux/atmel_qt602240.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>

#include <mach/board.h>
#include <mach/camera.h>

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
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>

#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/curcial_oj.h>
#include "board-buzz.h"
#include "proc_comm.h"
#include "gpio_chip.h"

#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
#include <mach/bcm_bt_lpm.h>
#endif

#include "devices.h"

#include <mach/atmega_microp.h>
#include <mach/htc_battery.h>

#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <mach/msm_flashlight.h>
#include <mach/msm_hsusb.h>
#include <mach/htc_usb.h>

void msm_init_irq(void);
void msm_init_gpio(void);
void config_buzz_camera_on_gpios(void);
void config_buzz_camera_off_gpios(void);
#ifdef CONFIG_MICROP_COMMON
void __init buzz_microp_init(void);
#endif
void config_buzz_proximity_gpios(int on);
static int buzz_phy_init_seq[] = {0x2C, 0x31, 0x20, 0x32, 0x1, 0x0D, 0x1, 0x10, -1};

#define HSUSB_API_INIT_PHY_PROC 2
#define HSUSB_API_PROG          0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1, 1)

static void buzz_phy_reset(void)
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

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= BUZZ_GPIO_35MM_HEADSET_DET,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_MICROP Driver */
static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.remote_int		= 1 << 5,
	.remote_irq		= MSM_uP_TO_INT(5),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x01,
	.adc_remote		= {0, 33, 38, 82, 95, 167},
};

static struct platform_device htc_headset_microp = {
	.name	= "HTC_HEADSET_MICROP",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_microp_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_microp,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};
static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
	{
	.name   = "oj",
	.category = MICROP_FUNCTION_OJ,
	.int_pin = 1 << 12,
	},
};

static struct microp_function_config microp_lightsensor = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 1, 3, 5, 15, 38, 150, 273, 298, 323, 0x3FF },
	.channel = 3,
	.int_pin = 1 << 9,
	.golden_adc = 0xCF,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor,
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config led_config[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "jogball-backlight",
		.type = LED_JOGBALL,
	},
	{
		.name = "button-backlight",
		.type = LED_PWM,
		.led_pin = 1 << 0,
		.init_value = 30,
		.fade_time = 5,
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct microp_led_config led_config_xc[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "jogball-backlight",
		.type = LED_JOGBALL,
	},
};

static struct microp_led_platform_data microp_leds_data_xc = {
	.num_leds	= ARRAY_SIZE(led_config_xc),
	.led_config	= led_config_xc,
};

static struct bma150_platform_data buzz_g_sensor_pdata = {
	.microp_new_cmd = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &buzz_g_sensor_pdata,
		},
	},
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct platform_device microp_devices_xc[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data_xc,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &buzz_g_sensor_pdata,
		},
	},
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BUZZ_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct microp_i2c_platform_data microp_data_xc = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices_xc),
	.microp_devices = microp_devices_xc,
	.gpio_reset = BUZZ_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct i2c_board_info i2c_microp_devices = {
	I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
	.platform_data = &microp_data,
	.irq = MSM_GPIO_TO_INT(BUZZ_GPIO_UP_INT),
};

static struct gpio_led buzz_led_list[] = {
	{
		.name = "button-backlight",
		.gpio = BUZZ_AP_KEY_LED_EN,
		.active_low = 0,
	},
};

static struct gpio_led_platform_data buzz_leds_data = {
	.num_leds = ARRAY_SIZE(buzz_led_list),
	.leds = buzz_led_list,
};

static struct platform_device buzz_leds = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &buzz_leds_data,
	},
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BUZZ_LAYOUTS,
	.project_name = BUZZ_PROJECT_NAME,
	.reset = BUZZ_GPIO_COMPASS_RST_N,
	.intr = BUZZ_GPIO_COMPASS_INT_N,
};

static int buzz_ts_atmel_power(int on)
{
	printk(KERN_INFO "%s():\n", __func__);
	if (on) {
		gpio_set_value(BUZZ_GPIO_TP_RST, 0);
		msleep(5);
		gpio_set_value(BUZZ_TP_3V_EN, 1);
		msleep(5);
		gpio_set_value(BUZZ_GPIO_TP_RST, 1);
		msleep(40);
	} else {
		gpio_set_value(BUZZ_TP_3V_EN, 0);
		msleep(2);
	}
	return 0;
}

struct atmel_i2c_platform_data buzz_ts_atmel_data[] = {
	{
		.version = 0x020,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 915,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = BUZZ_GPIO_TP_ATT_N,
		.power = buzz_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {7, 0, 5, 2, 0, 0, 10, 15, 0, 0},
		.config_T9 = {139, 0, 0, 16, 11, 0, 16, 40, 3, 3, 10, 10, 5, 15, 3, 10, 20, 0, 0, 0, 0, 0, 252, 252, 34, 40, 160, 50, 142, 73, 40, 8},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 30},
		.object_crc = {0x3D, 0xBB, 0x8E},
		.cable_config = {30, 30, 8, 16},
	},
	{
		.version = 0x016,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 915,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = BUZZ_GPIO_TP_ATT_N,
		.power = buzz_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {7, 0, 5, 2, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 16, 11, 0, 16, 40, 3, 3, 10, 10, 5, 15, 3, 10, 20, 0, 0, 0, 0, 0, 252, 252, 34, 40, 160, 50, 142, 73, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 16, 0, 1, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 30},
		.object_crc = {0x2B, 0xFB, 0x8F},
		.cable_config = {30, 30, 8, 16},
	},
	{
		.version = 0x015,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 915,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = BUZZ_GPIO_TP_ATT_N,
		.power = buzz_ts_atmel_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {50, 15, 25},
		.config_T8 = {8, 0, 20, 10, 0, 0, 10, 15},
		.config_T9 = {139, 0, 0, 16, 11, 0, 16, 40, 3, 3, 0, 2, 2, 64, 2, 10, 20, 0, 0, 0, 0, 0, 0, 0, 0, 8, 150, 57, 150, 83},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T19 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {13, 0, 0, 25, 0, 231, 255, 4, 16, 0, 1, 10, 15, 20, 255, 255, 4},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {0, 0, 248, 42, 88, 27, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 0, 4, 8, 60},
	},
};

static int buzz_syn_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		gpio_set_value(BUZZ_TP_3V_EN, 1);
		msleep(250);
	} else {
		gpio_set_value(BUZZ_TP_3V_EN, 0);
		udelay(50);
	}

	return 0;
}

static struct synaptics_i2c_rmi_platform_data buzz_ts_3k_data[] = {
	{
		.version = 0x0100,
		.flags = SYNAPTICS_FLIP_Y,
		.power = buzz_syn_ts_power,
		.abs_x_min = 0,
		.abs_x_max = 990,
		.abs_y_min = 0,
		.abs_y_max = 1300,
		.sensitivity_adjust = 0,
		.finger_support = 4,
	}
};

static void buzz_disable_usb_charger(void)
{
	printk(KERN_INFO "%s\n", __func__);
	htc_battery_charger_disable();
}

#ifdef CONFIG_USB_ANDROID
static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(BUZZ_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(BUZZ_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

void config_buzz_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table,
			ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(BUZZ_GPIO_USB_ID_PIN, 1);
	} else
		config_gpio_table(usb_ID_PIN_input_table,
			ARRAY_SIZE(usb_ID_PIN_input_table));
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= buzz_phy_init_seq,
	.phy_reset		= buzz_phy_reset,
	.usb_id_pin_gpio	= BUZZ_GPIO_USB_ID_PIN,
	.disable_usb_charger	= buzz_disable_usb_charger,
	.accessory_detect	= 1, /* detect by ID pin gpio */
	.config_usb_id_gpios	= config_buzz_usb_id_gpios,
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

#ifdef CONFIG_USB_ANDROID_RNDIS
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
	.product_id	= 0x0c8b,
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
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &buzz_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(BUZZ_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_3K_NAME, 0x20),
		.platform_data = &buzz_ts_3k_data,
		.irq = MSM_GPIO_TO_INT(BUZZ_GPIO_TP_ATT_N)
	},
};

static struct i2c_board_info i2c_sensor[] = {
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BUZZ_GPIO_COMPASS_INT_N),
	},
};

static struct pwr_sink buzz_pwrsink_table[] = {
	{
		.id     = PWRSINK_AUDIO,
		.ua_max = 100000,
	},
	{
		.id     = PWRSINK_BACKLIGHT,
		.ua_max = 125000,
	},
	{
		.id     = PWRSINK_LED_BUTTON,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_LED_KEYBOARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_GP_CLK,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_BLUETOOTH,
		.ua_max = 15000,
	},
	{
		.id     = PWRSINK_CAMERA,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_SDCARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_VIDEO,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id     = PWRSINK_SYSTEM_LOAD,
		.ua_max = 100000,
		.percent_util = 38,
	},
};

static int buzz_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void buzz_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void buzz_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int buzz_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data buzz_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(buzz_pwrsink_table),
	.sinks          = buzz_pwrsink_table,
	.suspend_late	= buzz_pwrsink_suspend_late,
	.resume_early	= buzz_pwrsink_resume_early,
	.suspend_early	= buzz_pwrsink_suspend_early,
	.resume_late	= buzz_pwrsink_resume_late,
};

static struct platform_device buzz_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &buzz_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
};

#ifdef CONFIG_MSM_CAMERA

static int camera_power_on_init(void)
{
	int rc=0;

//	printk(KERN_INFO "%s():\n", __func__);

	gpio_request(BUZZ_GPIO_VCM_PWDN, "cam_pwr_on");
	gpio_direction_output(BUZZ_GPIO_VCM_PWDN, 0);
	gpio_free(BUZZ_GPIO_VCM_PWDN);

	return rc;
}

static int flashlight_control(int mode)
{
	return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_buzz_camera_on_gpios,
	.camera_gpio_off = config_buzz_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = BUZZ_GPIO_CAM_RST_N,
	.vcm_pwd        = BUZZ_GPIO_VCM_PWDN,
	.camera_power_on = camera_power_on_init,
	.pdata          = &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED,
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif

static ssize_t buzz_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":16:350:32:58"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":68:350:56:58"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":175:350:56:58"
	   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":224:350:32:58"
	   "\n");
}

static struct kobj_attribute buzz_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &buzz_virtual_keys_show,
};

static struct kobj_attribute buzz_synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &buzz_virtual_keys_show,
};

static struct attribute *buzz_properties_attrs[] = {
	&buzz_synaptics_virtual_keys_attr.attr,
	&buzz_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group buzz_properties_attr_group = {
	.attrs = buzz_properties_attrs,
};

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct platform_device buzz_rfkill = {
	.name = "buzz_rfkill",
	.id = -1,
};

static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		config_buzz_proximity_gpios(1);
		gpio_direction_output(BUZZ_GPIO_PROXIMITY_EN, 1);
		gpio_direction_output(BUZZ_PS_2V85_EN, 1);
	} else {
		gpio_direction_output(BUZZ_PS_2V85_EN, 0);
		gpio_direction_output(BUZZ_GPIO_PROXIMITY_EN, 0);
		config_buzz_proximity_gpios(0);
	}
	return 0;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static unsigned int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.p_out = BUZZ_GPIO_PROXIMITY_INT,
	.p_en = BUZZ_GPIO_PROXIMITY_EN,
	.power = capella_cm3602_power,
	.irq = MSM_GPIO_TO_INT(BUZZ_GPIO_PROXIMITY_INT),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

/* End Proximity Sensor (Capella_CM3602)*/
#define CURCIAL_OJ_MOTION            39
static void curcial_oj_shutdown (int	enable)
{
	uint8_t cmd[3];
	static uint32_t oj_motion_on[] = {
		PCOM_GPIO_CFG(CURCIAL_OJ_MOTION, 0, GPIO_INPUT,
						GPIO_PULL_UP, GPIO_2MA),
	};
	static uint32_t oj_motion_off[] = {
		PCOM_GPIO_CFG(CURCIAL_OJ_MOTION, 0, GPIO_OUTPUT,
						GPIO_PULL_DOWN, GPIO_2MA),
	};

	memset(cmd, 0, sizeof(uint8_t)*3);

	cmd[2] = 0x80;
	if (enable) {
	/*keep O(L) enable by HW*/
		/*microp_i2c_write(0x91, cmd, 3);*/
		config_gpio_table(oj_motion_off, ARRAY_SIZE(oj_motion_off));
	} else {
		/*microp_i2c_write(0x90, cmd, 3);*/
		config_gpio_table(oj_motion_on, ARRAY_SIZE(oj_motion_on));
	}

}
static int curcial_oj_poweron(int on)
{
/*
	gpio_set_value(CURCIAL_OJ_POWER, on);

	if (gpio_get_value(CURCIAL_OJ_POWER) != on) {
		printk(KERN_ERR "%s:OJ:power status fail \n", __func__);
		return 0;
	}
	*/
		printk(KERN_ERR "%s:OJ:power status ok \n", __func__);
	return 1;
}
#define BUZZ_MICROP_VER	0x05
static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;


	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (0) {
		deltaX = (-1)*((int8_t) data[2]); /*X=2*/
		deltaY = (-1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (1)*((int8_t) data[1]);
		deltaY = (1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}

static struct curcial_oj_platform_data buzz_oj_data = {
	.oj_poweron = curcial_oj_poweron,
	.oj_shutdown = curcial_oj_shutdown,
	.oj_adjust_xy = curcial_oj_adjust_xy,
	.microp_version = BUZZ_MICROP_VER,
	.mdelay_time = 0,
	.normal_th = 10,
	.xy_ratio = 15,
	.interval = 0,
	.swap = true,
	.x = -1,
	.y = -1,
	.share_power = true,
	.debugflag = 0,
	.ap_code = true,
	.sht_tbl = {0, 1000, 1250, 1500, 1750, 2000, 3000},
	.pxsum_tbl = {0, 0, 90, 100, 110, 120, 130},
	.degree = 7,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 10, 10, 10, 10, 9, 9, 9, 9, 9,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq = MSM_uP_TO_INT(12),
	.device_id = 0x0D,
};

static struct platform_device buzz_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data	= &buzz_oj_data,
	}
};

static uint32_t fl_gpio_table[] = {
		PCOM_GPIO_CFG(BUZZ_GPIO_FL_TORCH, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(BUZZ_GPIO_FL_FLASH, 0,
					GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static void config_buzz_flashlight_gpios(void)
{
	config_gpio_table(fl_gpio_table, ARRAY_SIZE(fl_gpio_table));
}

static struct flashlight_platform_data buzz_flashlight_data = {
	.gpio_init = config_buzz_flashlight_gpios,
	.torch = BUZZ_GPIO_FL_TORCH,
	.flash = BUZZ_GPIO_FL_FLASH,
	.flash_duration_ms = 600,
	.chip_model = AAT1271,
};

static struct platform_device buzz_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data  = &buzz_flashlight_data,
	},
};

#if defined(CONFIG_SERIAL_MSM_HS) && defined(CONFIG_SERIAL_MSM_HS_PURE_ANDROID)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
};

static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = BUZZ_GPIO_BT_CHIP_WAKE,
	.gpio_host_wake = BUZZ_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device buzz_bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

#define ATAG_BDADDR 0x43294329  /* bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);
	printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddr);

	return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

#elif defined(CONFIG_SERIAL_MSM_HS)
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(BUZZ_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = BUZZ_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = BUZZ_GPIO_BT_HOST_WAKE,
};

/* for bcm */
static char bdaddress[20];
extern unsigned char *get_bt_bd_ram(void);

static void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
		cTemp[0], cTemp[1], cTemp[2], cTemp[3], cTemp[4], cTemp[5]);
	printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

static char bt_chip_id[10] = "bcm4329";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");
#endif

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	&buzz_bcm_bt_lpm_device,
#endif
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&htc_battery_pdev,
	&msm_camera_sensor_s5k4e1gx,
	&buzz_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&buzz_pwr_sink,
#endif
/* TODO:JOGALL */
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&capella_cm3602,
#endif
	&buzz_flashlight_device,
};

extern struct sys_timer msm_timer;

static void __init buzz_init_irq(void)
{
	printk("buzz_init_irq()\n");
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

static void buzz_reset(void)
{
	gpio_set_value(BUZZ_GPIO_PS_HOLD, 0);
}

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(BUZZ_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(BUZZ_GPIO_PROXIMITY_INT,
		0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
};

static struct i2c_board_info i2c_camera_devices[] = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 1), /*5M samsung bayer sensor driver*/
	},
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */

	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* MCLK */
	PCOM_GPIO_CFG(118, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /*CAM_RST*/
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

void config_buzz_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_buzz_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

void config_buzz_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

static uint32_t buzz_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(BUZZ_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(BUZZ_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void buzz_config_serial_debug_gpios(void)
{
	config_gpio_table(buzz_serial_debug_table,
			ARRAY_SIZE(buzz_serial_debug_table));
}


static void __init config_gpios(void)
{
	uint32_t config;
	buzz_config_serial_debug_gpios();
	config_buzz_camera_off_gpios();
	/* config display VSYNC gpio */
	config = PCOM_GPIO_CFG(BUZZ_MDDI_VSYNC, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
}

static struct msm_acpu_clock_platform_data buzz_clock_data = {
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

static unsigned buzz_perf_acpu_table[] = {
	245760000,
	480000000,
	528000000,
};

static struct perflock_platform_data buzz_perflock_data = {
	.perf_acpu_table = buzz_perf_acpu_table,
	.table_size = ARRAY_SIZE(buzz_perf_acpu_table),
};

static void __init buzz_init(void)
{
	int rc;
	struct kobject *properties_kobj;

	printk("buzz_init() revision=%d\n", system_rev);
	printk(KERN_INFO "mfg_mode=%d\n", board_mfg_mode());
	msm_clock_init();

#ifndef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	/* for bcm */
	bt_export_bd_address();
#endif

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	/* We need to set this pin to 0 only once on power-up; we will
	 * not actually enable the chip until we apply power to it via
	 * vreg.
	 */
	gpio_request(BUZZ_GPIO_LS_EN, "ls_en");
	gpio_direction_output(BUZZ_GPIO_LS_EN, 0);

	gpio_request(BUZZ_PS_2V85_EN, "ps_2v85_en");

	msm_hw_reset_hook = buzz_reset;

	msm_acpu_clock_init(&buzz_clock_data);
	perflock_init(&buzz_perflock_data);
	/* adjust GPIOs based on bootloader request */

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(BUZZ_GPIO_UART3_RX));
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#ifndef CONFIG_SERIAL_MSM_HS_PURE_ANDROID
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
#endif
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
#ifdef CONFIG_USB_FUNCTION
	msm_register_usb_phy_init_seq(buzz_phy_init_seq);
	msm_add_usb_id_pin_gpio(BUZZ_GPIO_USB_ID_PIN);
	msm_add_usb_devices(buzz_phy_reset, NULL);
#endif

#ifdef CONFIG_USB_ANDROID
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_buzz_usb_id_gpios(0);
	platform_device_register(&msm_device_hsusb);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#endif
	msm_add_mem_devices(&pmem_setting);

	#ifdef CONFIG_MICROP_COMMON
	buzz_microp_init();
#endif

	rc = buzz_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
					 &buzz_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	msm_device_i2c_init();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, i2c_sensor, ARRAY_SIZE(i2c_sensor));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	if (system_rev < 3) {
		i2c_microp_devices.platform_data = &microp_data_xc;
		platform_device_register(&buzz_leds);
	}
	if (system_rev >= 4) {
		platform_device_register(&buzz_oj);

	}
	i2c_register_board_info(0, &i2c_microp_devices, 1);

	/* probe camera driver */
	i2c_register_board_info(0, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));

	buzz_init_keypad();
	buzz_wifi_init();
	buzz_panel_init();

	msm_init_pmic_vibrator(3000);
}

static void __init buzz_fixup(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE1);
	mi->bank[0].size = MSM_LINUX_SIZE1;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].node = PHYS_TO_NID(MSM_LINUX_BASE2);
	mi->bank[1].size = MSM_LINUX_SIZE2;
}

static void __init buzz_map_io(void)
{
	printk("buzz_init_map_io()\n");
	msm_map_common_io();
}

MACHINE_START(BUZZ, "buzz")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x02E00100,
	.fixup          = buzz_fixup,
	.map_io         = buzz_map_io,
	.init_irq       = buzz_init_irq,
	.init_machine   = buzz_init,
	.timer          = &msm_timer,
MACHINE_END
