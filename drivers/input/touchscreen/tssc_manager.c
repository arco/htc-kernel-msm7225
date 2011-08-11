/* drivers/input/touchscreen/tssc_manager.c
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/msm_iomap.h>
#include <mach/msm_tssc.h>
#include <linux/earlysuspend.h>
#include <linux/rtc.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tssc_manager_early_suspend(struct early_suspend *h);
static void tssc_manager_late_resume(struct early_suspend *h);
#endif

static ssize_t tssc_calibration_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "%s():\n", __func__);
	return calibration_show(buf);
}

static ssize_t tssc_calibration_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	struct tssc_ts_platform_data *pdata;

	pdata = dev->platform_data;
	printk(KERN_DEBUG "%s():\n", __func__);

	calibration_store(buf, pdata);

	return count;
}

static ssize_t tssc_calibration_points_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return calibration_points_show(buf);
}

static ssize_t tssc_calibration_points_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	calibration_points_store(buf, dev->platform_data);

	return count;
}

static ssize_t tssc_calibration_screen_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return calibration_screen_show(buf);
}

static ssize_t tssc_calibration_screen_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	calibration_screen_store(buf);

	return count;
}

static ssize_t tssc_calibration_mfg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return calibration_mfg_show(buf);
}
static ssize_t tssc_calibration_mfg_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	calibration_mfg_store(buf);
	return count;
}
/* sys/class/input/input1/calibration */
static DEVICE_ATTR(calibration, 0666, tssc_calibration_show, tssc_calibration_store);

/* sys/class/input/input1/calibration_points */
static DEVICE_ATTR(calibration_points, 0666, tssc_calibration_points_show, tssc_calibration_points_store);

/* sys/class/input/input1/calibration_screen */
static DEVICE_ATTR(calibration_screen, 0666, tssc_calibration_screen_show, tssc_calibration_screen_store);
/* sys/class/input/input1/calibration_mfg */
static DEVICE_ATTR(calibration_mfg, 0666, tssc_calibration_mfg_show, tssc_calibration_mfg_store);



#define INT_TCHSCRN1         30
#define INT_TCHSCRN2         31
static unsigned int msm7225_irq_down = INT_TCHSCRN1;
static struct workqueue_struct *tssc_manager_wq;

struct tssc_manager_data {
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct work;
	struct work_struct polling_work;
	int x;
	int x16;
	int y;
	int y16;
	int z1;
	int z2;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	struct tssc_vkey *vkey;
	int vkey_num;
};

#define TOUCH_POLLING_NSEC 10000000
#define TOUCH_QUEUE_NUMBER 2

#define TOUCH_CHECK_BORDER_EN 1

#if TOUCH_CHECK_BORDER_EN
static unsigned short TOUCH_BORDER_X1  = 107;
static unsigned short TOUCH_BORDER_X10 =  5;
static unsigned short TOUCH_BORDER_X11 =  81;

static unsigned short TOUCH_BORDER_X2  = 942;
static unsigned short TOUCH_BORDER_X20 =  1020;
static unsigned short TOUCH_BORDER_X21 =  916;

static unsigned short TOUCH_BORDER_Y1  = 80;
static unsigned short TOUCH_BORDER_Y10 =  5;
static unsigned short TOUCH_BORDER_Y11 =  61;

static unsigned short TOUCH_BORDER_Y2  = 962;
static unsigned short TOUCH_BORDER_Y20 =  1020;
static unsigned short TOUCH_BORDER_Y21 =  943;

static unsigned char touch_check_left_border;
static unsigned char touch_check_right_border;
static unsigned char touch_check_upper_border;
static unsigned char touch_check_bottom_border;
#endif

static long touch_sample_count = 0;
static long touch_report_count = 0;
static int touch_queue_index = 0;

int touch_queue_x[TOUCH_QUEUE_NUMBER];
int touch_queue_y[TOUCH_QUEUE_NUMBER];
int touch_queue_p[TOUCH_QUEUE_NUMBER];
int touch_average_x;
int touch_average_y;
int touch_average_p;
long total_x = 0;
long total_y = 0;

#define NOISE_THRESHOLD 7
int touch_noise_index = TOUCH_QUEUE_NUMBER;

struct msm_tssc_ssbi *tssc_codec;
struct msm_tssc_software_register *tssc_reg;

enum {
	DEBUG_TP_OFF = 0,
	DEBUG_TP_ON = 1,
	STATISTIC_TP_ON = 2,
};
static int debug_tp;
module_param_named(debug, debug_tp, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define MAX_TOUCH_CHECK_UP 5
static int cnt_touch_check_up;
static int touch_check_up(int pressure)
{
	if (debug_tp & STATISTIC_TP_ON) {
		printk(KERN_DEBUG "touch_check_up(): tssc_reg->tssc_ctl.intr_flag2=0x%x\t", tssc_reg->tssc_ctl.intr_flag2);
		printk(KERN_DEBUG "tssc_ssbi_fsm_state =0x%x\t", tssc_reg->tssc_status.tssc_ssbi_fsm_state);
		printk(KERN_DEBUG "tssc_fsm_state =0x%x\t", tssc_reg->tssc_status.tssc_fsm_state);
		printk(KERN_DEBUG "busy =0x%x\t", tssc_reg->tssc_status.busy);
		printk(KERN_DEBUG "error_code =0x%x\n", tssc_reg->tssc_status.error_code);
		printk(KERN_DEBUG "adc_eoc_status =0x%x\t", tssc_reg->tssc_status.adc_eoc_status);
		printk(KERN_DEBUG "penirq_status =0x%x\t", tssc_reg->tssc_status.penirq_status);
		printk(KERN_DEBUG "operation =0x%x\t", tssc_reg->tssc_status.operation);
		printk(KERN_DEBUG "samples_collected =0x%x\t", tssc_reg->tssc_status.samples_collected);
		printk(KERN_DEBUG "error =0x%x\t", tssc_reg->tssc_status.error);
		printk(KERN_DEBUG "pressure =%d\t", pressure);
		printk(KERN_DEBUG "\n");
	}

	if ((1 == tssc_reg->tssc_status.penirq_status) &&
	    (0 == tssc_reg->tssc_status.busy || tssc_reg->tssc_ctl.intr_flag2 == 1) &&
	    (0 == tssc_reg->tssc_status.operation) &&
	    (0 == tssc_reg->tssc_status.tssc_fsm_state)) {
		cnt_touch_check_up++;
		return 1;
	} else {
		cnt_touch_check_up = 0;
		return 0;
	}
}

static int touch_check_noise(int x, int x16, int y, int y16)
{

	int X_temp = abs(x-x16);
	int Y_temp = abs(y-y16);

	struct timespec ts;
	struct rtc_time tm;

	if (x16 > 0 && y16 > 0) {
		if ((X_temp >= NOISE_THRESHOLD) || (Y_temp >= NOISE_THRESHOLD)) {
			if (debug_tp & DEBUG_TP_ON) {
				printk(KERN_DEBUG "touch_check_noise(): This is a noise point.\t");
				printk(KERN_DEBUG "x=%d y=%d x16=%d y16=%d\t", x, y, x16, y16);
			}

			if (debug_tp & DEBUG_TP_ON) {
				getnstimeofday(&ts);
				rtc_time_to_tm(ts.tv_sec, &tm);
			}

			if (debug_tp & DEBUG_TP_ON)
				pr_info("(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
					tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

			return 1;
		} else {
			return 0;
		}
	} else {
			return 0;
	}
}

/*
 * Add touch point data into the queue.
 */
static int touch_add_queue(int x, int x16, int y, int y16, int p)
{
	if (touch_check_noise(x, x16, y, y16))
		return 0;

	touch_queue_x[touch_queue_index] = x;
	touch_queue_y[touch_queue_index] = y;
	touch_queue_p[touch_queue_index] = p;

	touch_queue_index++;
	touch_queue_index %= TOUCH_QUEUE_NUMBER;

	return ++touch_sample_count;
}



/*
 * Report touch up input event.
 */
static void touch_input_up(struct input_dev *dev)
{
	struct timespec ts;
	struct rtc_time tm;

	if (debug_tp & STATISTIC_TP_ON) {
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
	}

	if (touch_report_count > 0) {
		input_report_abs(dev, ABS_PRESSURE, 0);
		input_report_abs(dev, ABS_TOOL_WIDTH, 0);
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, BTN_2, 0);
		input_sync(dev);
		touch_report_count = 0; /* reset report count */


		if (debug_tp & STATISTIC_TP_ON)
			printk(KERN_DEBUG "touch_input_up(): input_sync(): (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		if (debug_tp & STATISTIC_TP_ON)
			printk(KERN_DEBUG "touch_input_up(): input_sync():\n");
	}
}

static void touch_get_average(void)
{
	int i;
	long total_x = 0;
	long total_y = 0;
	long total_p = 0;

	for (i = 0; i < TOUCH_QUEUE_NUMBER; i++) {
		total_x += touch_queue_x[i];
		total_y += touch_queue_y[i];
		total_p += touch_queue_p[i];
	}

	touch_average_x = total_x / TOUCH_QUEUE_NUMBER;
	touch_average_y = total_y / TOUCH_QUEUE_NUMBER;
	touch_average_p = total_p / TOUCH_QUEUE_NUMBER;
}

#if TOUCH_CHECK_BORDER_EN
static ssize_t touch_border_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *ptr_data = (char *)buf;
	char *p;
	int cnt = 0;
	long para[4];
	long val;

	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "touch_border_store():\n");
	while ((p = strsep(&ptr_data, " "))) {
		if (!*p)
			break;

		if (cnt >= 4)
			break;

		val = simple_strtol(p, NULL, 10);
		para[cnt] = val;
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "touch_border_store(): para[%d]=%ld\n", cnt, para[cnt]);

		cnt++;
	}
	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "touch_border_store(): cnt=%d\n", cnt);

	if (cnt == 4) {

		if (para[0] == 0)
			touch_check_left_border = 0;
		else
			touch_check_left_border = 1;
		if (para[1] == 0)
			touch_check_right_border = 0;
		else
			touch_check_right_border = 1;
		if (para[2] == 0)
			touch_check_upper_border = 0;
		else
			touch_check_upper_border = 1;
		if (para[3] == 0)
			touch_check_bottom_border = 0;
		else
			touch_check_bottom_border = 1;

		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "touch_border_store(): %ld %ld %ld %ld\n", para[0], para[1], para[2], para[3]);

		TOUCH_BORDER_X1 = para[0];

		TOUCH_BORDER_X21 = para[1];
		TOUCH_BORDER_Y1 = para[2];
		TOUCH_BORDER_Y21 = para[3];
	}
	if (debug_tp & DEBUG_TP_ON) {
		printk(KERN_DEBUG "touch_border_store(): touch_check_left_border=%d\n", touch_check_left_border);
		printk(KERN_DEBUG "touch_border_store(): touch_check_right_border=%d\n", touch_check_right_border);
		printk(KERN_DEBUG "touch_border_store(): touch_check_upper_border=%d\n", touch_check_upper_border);
		printk(KERN_DEBUG "touch_border_store(): touch_check_bottom_border=%d\n", touch_check_bottom_border);

		printk(KERN_DEBUG "TOUCH_BORDER_X1 =%d\n", TOUCH_BORDER_X1);
		printk(KERN_DEBUG "TOUCH_BORDER_X10=%d\n", TOUCH_BORDER_X10);
		printk(KERN_DEBUG "TOUCH_BORDER_X11=%d\n", TOUCH_BORDER_X11);

		printk(KERN_DEBUG "TOUCH_BORDER_X2 =%d\n", TOUCH_BORDER_X2);
		printk(KERN_DEBUG "TOUCH_BORDER_X20=%d\n", TOUCH_BORDER_X20);
		printk(KERN_DEBUG "TOUCH_BORDER_X21=%d\n", TOUCH_BORDER_X21);

		printk(KERN_DEBUG "TOUCH_BORDER_Y1 =%d\n", TOUCH_BORDER_Y1);
		printk(KERN_DEBUG "TOUCH_BORDER_Y10=%d\n", TOUCH_BORDER_Y10);
		printk(KERN_DEBUG "TOUCH_BORDER_Y11=%d\n", TOUCH_BORDER_Y11);

		printk(KERN_DEBUG "TOUCH_BORDER_Y2 =%d\n", TOUCH_BORDER_Y2);
		printk(KERN_DEBUG "TOUCH_BORDER_Y20=%d\n", TOUCH_BORDER_Y20);
		printk(KERN_DEBUG "TOUCH_BORDER_Y21=%d\n", TOUCH_BORDER_Y21);
	}

	return count;
}


static ssize_t touch_border_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "%s():\n", __func__);
	return sprintf(buf, "%d,%d,%d,%d\n", TOUCH_BORDER_X1, TOUCH_BORDER_X2, TOUCH_BORDER_Y1, TOUCH_BORDER_Y2);
}
/* sys/class/input/input1/border */
static DEVICE_ATTR(border, 0666, touch_border_read, touch_border_store);

static void touch_check_border_report(struct input_dev *dev)
{
	input_report_abs(dev, ABS_PRESSURE, touch_average_p);
	input_report_abs(dev, ABS_TOOL_WIDTH, 1);
	input_report_key(dev, BTN_TOUCH, 1);
	input_report_key(dev, BTN_2, 0);
	input_sync(dev);

	touch_report_count++;
}

static void touch_check_border(struct input_dev *dev, int x, int y)
{
	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "touch_check_border(): x=%d y=%d\n", x , y);

	if (x > TOUCH_BORDER_X11 && x < TOUCH_BORDER_X1 && touch_check_left_border == 1) {
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "x is in left border\n");

		input_report_abs(dev, ABS_X, TOUCH_BORDER_X10);
		input_report_abs(dev, ABS_Y, y);
		touch_check_border_report(dev);

	}

	if (x > TOUCH_BORDER_X21 && x < TOUCH_BORDER_X2 && touch_check_right_border == 1) {
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "x is in right border\n");
		input_report_abs(dev, ABS_X, TOUCH_BORDER_X20);
		input_report_abs(dev, ABS_Y, y);
		touch_check_border_report(dev);

	}

	if (y > TOUCH_BORDER_Y11 && y < TOUCH_BORDER_Y1 && touch_check_upper_border == 1) {
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "y is in upper border\n");
		input_report_abs(dev, ABS_X, x);
		input_report_abs(dev, ABS_Y, TOUCH_BORDER_Y10);
		touch_check_border_report(dev);

	}
	if (y > TOUCH_BORDER_Y21 && y < TOUCH_BORDER_Y2 && touch_check_bottom_border == 1) {
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "y is in bottom border\n");
		input_report_abs(dev, ABS_X, x);
		input_report_abs(dev, ABS_Y, TOUCH_BORDER_Y20);
		touch_check_border_report(dev);

	}
}
#endif

/*
 * Process the current touch point in the queue.
 */
static void touch_process_queue(struct input_dev *dev, struct tssc_ts_platform_data *pdata)
{
	int x = 0, y = 0, loop_i;
	struct timespec ts;
	struct rtc_time tm;

	if (touch_sample_count <= 1) {
		if (touch_sample_count == 1) {
			if (debug_tp & STATISTIC_TP_ON)
				printk(KERN_DEBUG "touch_process_queue(): tssc_reg->tssc_ctl.intr_flag2=0x%x\t", tssc_reg->tssc_ctl.intr_flag2);
			if (debug_tp & DEBUG_TP_ON) {
				getnstimeofday(&ts);
				rtc_time_to_tm(ts.tv_sec, &tm);
			}

			if (debug_tp & DEBUG_TP_ON)
				pr_info("(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		}
		return;
	}

	if (touch_sample_count >= TOUCH_QUEUE_NUMBER)
		touch_get_average();
	calibration_translate(touch_average_x, touch_average_y, &x, &y, pdata);

	if (x >= 0 && y >= 0) {
		if (debug_tp & DEBUG_TP_ON)
			printk(KERN_DEBUG "touch_process_queue():\t");

		#if TOUCH_CHECK_BORDER_EN
		if (touch_sample_count == 2)
			touch_check_border(dev, x, y);
		#endif

		if (debug_tp & DEBUG_TP_ON) {
			printk(KERN_DEBUG "x=%d\t", x);
			printk(KERN_DEBUG "y=%d\t", y);
			printk(KERN_DEBUG "p=%d\n", touch_average_p);
		}
		if ((y < pdata->abs_report_max && pdata->vkey_num > 0) || pdata->vkey_num == 0) {

			input_report_abs(dev, ABS_X, x);
			input_report_abs(dev, ABS_Y, y);
			input_report_abs(dev, ABS_PRESSURE, touch_average_p);
			input_report_abs(dev, ABS_TOOL_WIDTH, 1);
			input_report_key(dev, BTN_TOUCH, 1);
			input_report_key(dev, BTN_2, 0);
			input_sync(dev);

			touch_report_count++;

		}


		if (pdata->vkey_num > 0 && y >= pdata->abs_vkey_y_min && y <= pdata->abs_vkey_y_max) {
			for (loop_i = 0; loop_i < pdata->vkey_num; loop_i++) {
				if (x >= pdata->vkey[loop_i].range_x_min && x <= pdata->vkey[loop_i].range_x_max) {
					input_report_key(dev, pdata->vkey[loop_i].keycode, 1);
					pdata->vkey[loop_i].status = 1;
					printk(KERN_INFO "vkey[%d], keycode=%d pressed\n",loop_i , pdata->vkey[loop_i].keycode);
					break;
				} else if(pdata->vkey[loop_i].status == 1) {
					input_report_key(dev, pdata->vkey[loop_i].keycode, 0);
					pdata->vkey[loop_i].status = 0;
					printk(KERN_INFO "vkey[%d], keycode=%d released\n",loop_i , pdata->vkey[loop_i].keycode);
				}
			}
		}

	}
}

/*
 * Process the final touch points in the queue.
 */
static void touch_finish_queue(struct input_dev *dev)
{
	touch_sample_count = 0;
	touch_queue_index = 0;


	touch_input_up(dev);
}

static void tssc_manager_work_func(struct work_struct *work)
{
	struct tssc_manager_data *ts;

	ts = container_of(work, struct tssc_manager_data, work);

	hrtimer_start(&ts->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
}

static int tssc_manager_polling_func(struct tssc_manager_data *ts)
{
	struct tssc_ts_platform_data *pdata;
	int pressure, loop_i;

	pdata = ts->input_dev->dev.platform_data;

	if (tssc_reg->tssc_ctl.data_flag == 0) { /* Check if TSSC data ready */
		if (tssc_reg->tssc_ctl.data_flag == 0) { /* Try again */
			ts->x = 0;
			ts->y = 0;
			ts->z1 = 0;
			ts->z2 = 0;
		}
	}

	if (tssc_reg->tssc_ctl.data_flag) { /* TSSC data is valid */
#ifdef ENABLE_TSSC_AVERAGE
		ts->x = tssc_reg->tssc_avg_12.samples_avg_1;
		ts->y = tssc_reg->tssc_avg_12.samples_avg_2;
		ts->z1 = tssc_reg->tssc_avg_34.samples_avg_3;
		ts->z2 = tssc_reg->tssc_avg_34.samples_avg_4;
#else
		ts->x = tssc_reg->tssc_sample_1_1.raw_sample_1;
		ts->x16 = tssc_reg->tssc_sample_1_8.raw_sample_16;
		ts->y = tssc_reg->tssc_sample_2_1.raw_sample_1;
		ts->y16 = tssc_reg->tssc_sample_2_8.raw_sample_16;
		ts->z1 = tssc_reg->tssc_sample_3_1.raw_sample_1;
		ts->z2 = tssc_reg->tssc_sample_4_1.raw_sample_1;
#endif
	}

	tssc_reg->tssc_ctl.intr_flag1 = 0; /* Clear INTR_FLAG1 for next point */
	tssc_reg->tssc_ctl.data_flag = 0; /* Clear DATA_FLAG for next point */

	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "tssc_manager_polling_func(): ts->z1=%d ts->z2=%d\n", ts->z1, ts->z2);
	if (ts->z1) {
		pressure = (ts->x * ((ts->z2 * 100 / ts->z1) - 100)) / 900;
		pressure = 270 - pressure;

		if (pressure <= 5)
		    pressure = 5; /* Report all touch points */
		else if (pressure > 255)
		    pressure = 255;
	} else {
		pressure = 0;
	}

	if (debug_tp & DEBUG_TP_ON)
		printk(KERN_DEBUG "%s(): ts->x=%d ts->y=%d\n", __func__,  ts->x, ts->y);


	if (ts->x != 0 && ts->y != 0 && ts->z1 != 0 && ts->z2 != 0 && pressure) {
		if (touch_add_queue(ts->x, ts->x16, ts->y, ts->y16, pressure)) {
			touch_process_queue(ts->input_dev, pdata);
		}
	}

	if (touch_check_up(pressure)) {
		if (cnt_touch_check_up >= MAX_TOUCH_CHECK_UP) {

			for (loop_i = 0; loop_i < ts->vkey_num; loop_i++) {
				if (ts->vkey_num > 0 && ts->vkey[loop_i].status == 1) {
					input_report_key(ts->input_dev, ts->vkey[loop_i].keycode, 0);
					ts->vkey[loop_i].status = 0;
					printk(KERN_INFO "vkey[%d], keycode=%d released\n",loop_i , ts->vkey[loop_i].keycode);
				}
			}

			touch_finish_queue(ts->input_dev);
			enable_irq(msm7225_irq_down);
			cnt_touch_check_up = 0;
			return 1;
		}
	}
	return 0;
}

static enum hrtimer_restart tssc_polling_timer_func(struct hrtimer *timer)
{
	struct tssc_manager_data *ts = container_of(timer, struct tssc_manager_data, timer);
	if (!tssc_manager_polling_func(ts)) {
		hrtimer_start(&ts->timer, ktime_set(0, TOUCH_POLLING_NSEC), HRTIMER_MODE_REL);
	}
	return HRTIMER_NORESTART;
}

static irqreturn_t tssc_manager_irq_down_handler(int irq, void *dev_id)
{
	struct tssc_manager_data *ts = dev_id;

	if (debug_tp & STATISTIC_TP_ON)
		printk(KERN_DEBUG "%s(): enter\n", __func__);

	disable_irq_nosync(msm7225_irq_down);
	queue_work(tssc_manager_wq, &ts->work);

	if (debug_tp & STATISTIC_TP_ON)
		printk(KERN_DEBUG "%s(): exit\n", __func__);

	return IRQ_HANDLED;
}

static void tssc_power_on(void)
{
	printk(KERN_DEBUG "tssc_power_on():\n");
	/* Set highest priority of SSBI port to TSSC. */


	tssc_codec->priorities.priority0 = 0x2;
	tssc_codec->priorities.priority1 = 0x1;
	tssc_codec->priorities.priority2 = 0x0;


	/* Enable TSSC. */

	tssc_reg->tssc_ctl.enable = 0x1;


	/* Reset TSSC. */
	tssc_reg->tssc_ctl.tssc_sw_reset = 0x1;


	/* Enable TSSC. */
	tssc_reg->tssc_ctl.enable = 0x1;


	/* Master mode. */
	tssc_reg->tssc_ctl.mode = 0x3;


	/* Enable the averaging function.*/
#ifdef ENABLE_TSSC_AVERAGE
	tssc_reg->tssc_ctl.en_average = 0x1;
#else
	tssc_reg->tssc_ctl.en_average = 0x0;
#endif


	/* Enable the debounce logic inside TSSC.*/
	tssc_reg->tssc_test_1.gate_debounce_en = 0x1;
	tssc_reg->tssc_ctl.debounce_en = 0x1;


	/* Debounce time = 400us.*/
	tssc_reg->tssc_ctl.debounce_time = 0x0;

	/* Clear data flag to ready for subsequent sample. */
	tssc_reg->tssc_ctl.data_flag = 0x0;

	/* 10-bit resolution. */
	tssc_reg->tssc_opn.resolution1 = 0x1;
	tssc_reg->tssc_opn.resolution2 = 0x1;
	tssc_reg->tssc_opn.resolution3 = 0x1;
	tssc_reg->tssc_opn.resolution4 = 0x1;


	/* Number of samples. */
#ifdef ENABLE_TSSC_AVERAGE
	tssc_reg->tssc_opn.num_sample1 = 0x3;	/* 16 samples */
	tssc_reg->tssc_opn.num_sample2 = 0x3;	/* 16 samples */
	tssc_reg->tssc_opn.num_sample3 = 0x1;	/* 4 samples */
	tssc_reg->tssc_opn.num_sample4 = 0x1;	/* 4 samples */
#else
	tssc_reg->tssc_opn.num_sample1 = 0x0;	/* 1 sample */
	tssc_reg->tssc_opn.num_sample2 = 0x0;	/* 1 sample */
	tssc_reg->tssc_opn.num_sample3 = 0x0;	/* 1 sample */
	tssc_reg->tssc_opn.num_sample4 = 0x0;	/* 1 sample */
#endif


	/* Place holder for the operations in the master mode.
	 Reference: Touchscreen Operation for MSM7200 and MSM7500, Table 4-1. */
	tssc_reg->tssc_opn.operation1 = 0x01; /* X, 4-wire */
	tssc_reg->tssc_opn.operation2 = 0x02; /* Y, 4-wire */
	tssc_reg->tssc_opn.operation3 = 0x03; /* Z1, 4-wire */
	tssc_reg->tssc_opn.operation4 = 0x04; /* Z2, 4-wire */


	/* Specifies the sampling interval in milliseconds units: 1ms. */
	tssc_reg->tssc_sampling_int.sampling_int = 0x1;

	touch_sample_count = 0;
	touch_report_count = 0;
	touch_queue_index = 0;
}

static void tssc_power_off(void)
{
	tssc_reg->tssc_ctl.enable = 0x0;
}

static int tssc_manager_input_init(struct platform_device *platform_dev)
{
	struct tssc_manager_data *ts;
	struct tssc_ts_platform_data *pdata;

	int ret = 0, ret_down = 0, loop_i = 0;

	pdata = platform_dev->dev.platform_data;

	cnt_touch_check_up = 0;

	printk(KERN_DEBUG "tssc_manager_input_init():\n");
	tssc_codec = (struct msm_tssc_ssbi *)ioremap(MSM_SSBI_PHYS, sizeof(struct msm_tssc_ssbi));
	tssc_reg = (struct msm_tssc_software_register *)ioremap(MSM_TSSC_PHYS + 0x100, sizeof(struct msm_tssc_ssbi));

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	if (ts == NULL) {
		printk(KERN_ERR "tssc_manager_input_init(): kzalloc()==NULL\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	tssc_manager_wq = create_singlethread_workqueue("tssc_manager_wq");
	if (!tssc_manager_wq)
		goto err_create_wq_failed;

	INIT_WORK(&ts->work, tssc_manager_work_func);

	tssc_power_on();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "tssc_manager_input_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = TSSC_MANAGER_NAME;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	printk(KERN_ERR "tssc_manager_input_init(): pdata->vkey_num=%d\n", pdata->vkey_num);
	if (pdata->vkey_num > 0) {
		ts->vkey = pdata->vkey;
		ts->vkey_num = pdata->vkey_num;
		for (loop_i = 0; loop_i < ts->vkey_num; loop_i++) {
			set_bit(ts->vkey[loop_i].keycode,
				ts->input_dev->keybit);
		}
	}

	ts->input_dev->dev.platform_data = pdata;

	/* Set input parameters boundary. */
	input_set_abs_params(ts->input_dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	/* Create device files. */
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_calibration);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init: Error to create calibration attribute\n");
		goto err_input_register_device_failed;
	}
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_calibration_points);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init: Error to create calibration_points attribute\n");
		goto err_input_register_device_failed;
	}
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_calibration_screen);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init: Error to create calibration_screen attribute\n");
		goto err_input_register_device_failed;
	}
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_calibration_mfg);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init: Error to create calibration_mfg attribute\n");
		goto err_input_register_device_failed;
	}

	#if TOUCH_CHECK_BORDER_EN
	ret = device_create_file(&ts->input_dev->dev, &dev_attr_border);
	if (ret) {
		printk(KERN_ERR "tssc_manager_input_init(): device_create_file(): %s: failed\n", __func__);
		goto err_input_register_device_failed;
	}
	#endif

	if (msm7225_irq_down) {
		ret_down = request_irq(msm7225_irq_down, tssc_manager_irq_down_handler,
			IRQF_TRIGGER_RISING, TSSC_MANAGER_NAME, ts);
	}

	if (ret_down == 0) {
		ts->use_irq = 1;
	} else {
		ts->use_irq = 0;
	}

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = tssc_polling_timer_func;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = tssc_manager_early_suspend;
	ts->early_suspend.resume = tssc_manager_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "tssc_manager_input_init: Start touchscreen %s in %s mode\n",
			ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	calibration_init(pdata);

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	destroy_workqueue(tssc_manager_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
	return ret;
}

static int tssc_manager_probe(struct platform_device *platform_dev)
{
	printk(KERN_INFO "tssc_manager_probe():\n");

	return tssc_manager_input_init(platform_dev);
}

static int tssc_manager_remove(struct platform_device *platform_dev)
{
	struct tssc_manager_data *ts = dev_get_drvdata(&platform_dev->dev);

	printk(KERN_INFO "tssc_manager_remove\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	if (ts->use_irq) {
		free_irq(msm7225_irq_down, ts);
	}

	hrtimer_cancel(&ts->timer);

	input_unregister_device(ts->input_dev);

	/* Remove device files. */
	device_remove_file(&ts->input_dev->dev, &dev_attr_calibration);
	device_remove_file(&ts->input_dev->dev, &dev_attr_calibration_points);
	device_remove_file(&ts->input_dev->dev, &dev_attr_calibration_screen);
	device_remove_file(&ts->input_dev->dev, &dev_attr_calibration_mfg);

	#if TOUCH_CHECK_BORDER_EN
	device_remove_file(&ts->input_dev->dev, &dev_attr_border);
	#endif

	kfree(ts);
	return 0;
}

static int tssc_manager_suspend(struct tssc_manager_data *ts, pm_message_t mesg)
{
	int ret;

	if (ts->use_irq)
		disable_irq(msm7225_irq_down);

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(msm7225_irq_down);

	tssc_power_off();

	return 0;
}

static int tssc_manager_resume(struct tssc_manager_data *ts)
{

	printk(KERN_DEBUG "tssc_manager_resume(): ts->use_irq=%d\n", ts->use_irq);

	tssc_power_on();

	if (ts->use_irq)
		enable_irq(msm7225_irq_down);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tssc_manager_early_suspend(struct early_suspend *h)
{
	struct tssc_manager_data *ts;
	ts = container_of(h, struct tssc_manager_data, early_suspend);
	tssc_manager_suspend(ts, PMSG_SUSPEND);
}

static void tssc_manager_late_resume(struct early_suspend *h)
{
	struct tssc_manager_data *ts;
	ts = container_of(h, struct tssc_manager_data, early_suspend);
	tssc_manager_resume(ts);
}
#endif

static struct platform_driver tssc_manager_driver = {
	.probe		= tssc_manager_probe,
	.remove		= tssc_manager_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= tssc_manager_suspend,
	.resume		= tssc_manager_resume,
#endif
	.driver = {
		.name	= TSSC_MANAGER_NAME,
	},
};

static int __devinit tssc_manager_init(void)
{
	int result = 0;

	printk(KERN_INFO "tssc_manager_init():\n");
	result = platform_driver_register(&tssc_manager_driver);
	if (result < 0)
		printk(KERN_ERR "tssc_manager_init: platform_driver_register failed\n");

	return result;
}

static void __exit tssc_manager_exit(void)
{
	if (tssc_manager_wq)
		destroy_workqueue(tssc_manager_wq);
	printk(KERN_DEBUG "tssc_manager_exit\n");
}

module_init(tssc_manager_init);
module_exit(tssc_manager_exit);

MODULE_DESCRIPTION("TSSC Manager Driver");
MODULE_LICENSE("GPL");
