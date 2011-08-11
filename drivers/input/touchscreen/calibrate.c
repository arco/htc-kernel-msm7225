/* drivers/input/touchscreen/calibrate.c
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
#include <mach/msm_tssc.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <asm/uaccess.h>

#define CALIBRATION_POINTS 5
#define CALIBRATION_COORDINATES (CALIBRATION_POINTS * 2)

/*
Flag to decide wheather current TP need to do adjustment or not.
0: No, not necessary to do adjustment.
1: Yes, should use calibration algorithm to do adjustment.
*/

static int adjustment_flag;

static int calibration_flag;
static int calibration_index;

/*
calibration_direction: Raw data vs. screen coordinate direction.
0: No swaped. X, Y same direction.
1: No swaped. X, same direction, Y reversed.
2: No swaped. X, reversed, Y same direction.
3: No swaped. X reversed, Y reversed.
4: X, Y swaped. X, Y same direction.
5: X, Y swaped. X, same direction, Y reversed.
6: X, Y swaped. X, reversed, Y same direction.
7: X, Y swaped. X, same direction, Y reversed.
*/

static int calibration_x_direction;
static int calibration_y_direction;
static int calibration_swapped;
static int calibration_x_min;
static int calibration_x_max;
static int calibration_y_min;
static int calibration_y_max;
static int saved_x_min;
static int saved_x_max;
static int saved_y_min;
static int saved_y_max;

static int calibration_x[CALIBRATION_POINTS];
static int calibration_y[CALIBRATION_POINTS];
static int saved_x[CALIBRATION_POINTS];
static int saved_y[CALIBRATION_POINTS];
static int screen_x[CALIBRATION_POINTS];
static int screen_y[CALIBRATION_POINTS];

/*
 * Get the touch input raw data and store into the coordinates buffer.
 */
void calibration_get_points(int x, int y)
{
	if (calibration_index >= CALIBRATION_POINTS)
		return;

	calibration_x[calibration_index] = x;
	calibration_y[calibration_index] = y;
}

/*
 * Translate the touch input raw data to the calibrated data.
 */
void calibration_translate(int x, int y, int *rx, int *ry, struct tssc_ts_platform_data *pdata)
{
	int cx, cy;

	if (calibration_flag > 0 && calibration_flag <= CALIBRATION_POINTS) { /* Under calibration */
		calibration_get_points(x, y);
		*rx = x;
		*ry = y;
	} else { /* Normal touch input */
		if (adjustment_flag) { /* After calibration */
			int kx = 0;
			int ky = 0;

			if (calibration_swapped) { /* X, Y swapped */
				/* Ratio translation. */
				if ((calibration_x_max > calibration_x_min) && (calibration_y_max > calibration_y_min)) {
					kx = pdata->x_min + ((x - calibration_y_min) * (pdata->x_max - pdata->x_min) / (calibration_y_max - calibration_y_min));
					ky = pdata->y_min + ((y - calibration_x_min) * (pdata->y_max - pdata->y_min) / (calibration_x_max - calibration_x_min));
				}

				/* X, Y direction */
				cx = (calibration_x_direction) ? (ky) : (pdata->x_max - ky);
				cy = (calibration_y_direction) ? (kx) : (pdata->y_max - kx);
			} else {
				/* Ratio translation. */
				if ((calibration_x_max > calibration_x_min) && (calibration_y_max > calibration_y_min)) {
					kx = pdata->x_min + ((x - calibration_x_min) * (pdata->x_max - pdata->x_min) / (calibration_x_max - calibration_x_min));
					ky = pdata->y_min + ((y - calibration_y_min) * (pdata->y_max - pdata->y_min) / (calibration_y_max - calibration_y_min));
				}

				/* X, Y direction */
				cx = (calibration_x_direction) ? (kx) : (pdata->x_max - kx);
				cy = (calibration_y_direction) ? (ky) : (pdata->y_max - ky);
			}
			if (cx < pdata->x_min)
				cx = pdata->x_min;
			else if (cx > pdata->x_max)
				cx = pdata->x_max;

			if (cy < pdata->y_min)
				cy = pdata->y_min;
			*rx = cx;
			*ry = cy;
		} else { /* No need to do calibration */
			*rx = y; /* Current device, x and y swapped */
			*ry = x;
		}
	}
}

/*
 * Show the calibration parameters.
 */
void calibration_show_parameters(int status)
{
	int i;
	for (i = 0; i < CALIBRATION_POINTS; i++)
		printk(KERN_INFO "touch_calibration: Point %d %d, %d.\n",
				i+1, calibration_x[i], calibration_y[i]);

	printk(KERN_INFO "touch_calibration: do calibration %s.\n",
			status ? "error" : "ok");
	printk(KERN_INFO "touch_calibration: x min %d, x max %d.\n",
			calibration_x_min, calibration_x_max);
	printk(KERN_INFO "touch_calibration: y min %d, y max %d.\n",
			calibration_y_min, calibration_y_max);
	printk(KERN_INFO "touch_calibration: %s x %s, y %s.\n",
		calibration_swapped ? "X Y swapped," : "",
		calibration_x_direction ? "reversed" : "same direction",
		calibration_y_direction ? "reversed" : "same direction");
}

/*
 * Calculate calibration variables.
 */
void calibration_calculate_varialbes(struct tssc_ts_platform_data *pdata)
{
	int xdir, xlen, ydir, ylen;
	int x_length, y_length;
	int x1, x2, y1, y2;
	int i;
	int result = 0;

	printk(KERN_DEBUG "%s():cal_range_x=%d cal_range_y=%d cal_err=%d\n", __func__, pdata->cal_range_x, pdata->cal_range_y, pdata->cal_err);
	saved_x_min = calibration_x_min;
	saved_x_max = calibration_x_max;
	saved_y_min = calibration_y_min;
	saved_y_max = calibration_y_max;

	if (calibration_index < CALIBRATION_POINTS - 1) {
		result = 1; /* Calibration points not enough */
	} else {
		if (calibration_x[0] > calibration_x[1]) {
			xdir = 0; /* Reversed */
			xlen = calibration_x[0] - calibration_x[1];
		} else {
			xdir = 1; /* Same */
			xlen = calibration_x[1] - calibration_x[0];
		}

		if (calibration_y[0] > calibration_y[1]) {
			ydir = 0; /* Reversed */
			ylen = calibration_y[0] - calibration_y[1];
		} else {
			ydir = 1; /* Same */
			ylen = calibration_y[1] - calibration_y[0];
		}

		if (xlen > ylen) {
			calibration_swapped = 0; /* No swapped */
			calibration_x_direction = xdir;
			x_length = xlen;

			if (calibration_y[0] > calibration_y[3]) {
				calibration_y_direction = 0; /* Reversed */
				y_length = calibration_y[0] - calibration_y[3];
			} else {
				calibration_y_direction = 1; /* Same */
				y_length = calibration_y[3] - calibration_y[0];
			}

			x1 = (calibration_x_direction) ? (calibration_x[0]) : (calibration_x[1]);
			x2 = (calibration_x_direction) ? (calibration_x[1]) : (calibration_x[0]);
			y1 = (calibration_y_direction) ? (calibration_y[0]) : (calibration_y[3]);
			y2 = (calibration_y_direction) ? (calibration_y[3]) : (calibration_y[0]);
		} else {
			calibration_swapped = 1; /* X, Y swapped */
			calibration_x_direction = ydir;
			x_length = ylen;

			if (calibration_x[0] > calibration_x[3]) {
				calibration_y_direction = 0; /* Reversed */
				y_length = calibration_x[0] - calibration_x[3];
			} else {
				calibration_y_direction = 1; /* Same */
				y_length = calibration_x[3] - calibration_x[0];
			}

			x1 = (calibration_x_direction) ? (calibration_y[0]) : (calibration_y[1]);
			x2 = (calibration_x_direction) ? (calibration_y[1]) : (calibration_y[0]);
			y1 = (calibration_y_direction) ? (calibration_x[0]) : (calibration_x[3]);
			y2 = (calibration_y_direction) ? (calibration_x[3]) : (calibration_x[0]);
		}

		xlen = (screen_x[1] - screen_x[0]);

		if (xlen) {
			calibration_x_min = x1 - ((x_length) * (screen_x[0]) / xlen);
			calibration_x_max = x2 + ((x_length) * (pdata->screen_width - screen_x[1]) / xlen);
		} else {
			result = 1;
		}

		printk(KERN_INFO "screen_y[2]=%d screen_y[0]=%d\n", screen_y[2], screen_y[0]);
		ylen = (screen_y[2] - screen_y[0]);
		printk(KERN_INFO "ylen=%d\n", ylen);

		if (ylen) {
			calibration_y_min = y1 - ((y_length) * (screen_y[0]) / ylen);
			calibration_y_max = y2 + ((y_length) * (pdata->screen_height - screen_y[2]) / ylen);
		} else {
			result = 1;
		}
	}

	for (i = 0; i < CALIBRATION_POINTS; i++) {
		printk(KERN_INFO "calibration_x[%d]=%d\t", i, calibration_x[i]);
		printk(KERN_INFO "calibration_y[%d]=%d\n", i, calibration_y[i]);
	}
	printk(KERN_INFO "abs(calibration_x[0] - calibration_x[3])=%d\n", abs(calibration_x[0] - calibration_x[3]));
	printk(KERN_INFO "abs(calibration_y[0] - calibration_y[1])=%d\n", abs(calibration_y[0] - calibration_y[1]));
	printk(KERN_INFO "abs(calibration_x[1] - calibration_x[4])=%d\n", abs(calibration_x[1] - calibration_x[4]));
	printk(KERN_INFO "abs(calibration_y[3] - calibration_y[4])=%d\n", abs(calibration_y[3] - calibration_y[4]));
	printk(KERN_INFO "abs(calibration_x[2] - ((calibration_x[0] + calibration_x[1])/2))=%d\n", abs(calibration_x[2] - ((calibration_x[0] + calibration_x[1])/2)));
	printk(KERN_INFO "abs(calibration_x[2] - ((calibration_x[3] + calibration_x[4])/2))=%d\n", abs(calibration_x[2] - ((calibration_x[3] + calibration_x[4])/2)));
	printk(KERN_INFO "abs(calibration_y[2] - ((calibration_y[0] + calibration_y[3])/2))=%d\n", abs(calibration_y[2] - ((calibration_y[0] + calibration_y[3])/2)));
	printk(KERN_INFO "abs(calibration_y[2] - ((calibration_y[1] + calibration_y[4])/2))=%d\n", abs(calibration_y[2] - ((calibration_y[1] + calibration_y[4])/2)));

	printk(KERN_INFO "abs(calibration_x[0] - calibration_x[1]))=%d\n", abs(calibration_x[0] - calibration_x[1]));
	printk(KERN_INFO "abs(calibration_x[3] - calibration_x[4]))=%d\n", abs(calibration_x[3] - calibration_x[4]));
	printk(KERN_INFO "abs(calibration_y[0] - calibration_y[3])=%d\n", abs(calibration_y[0] - calibration_y[3]));
	printk(KERN_INFO "abs(calibration_y[1] - calibration_y[4])=%d\n", abs(calibration_y[1] - calibration_y[4]));

	if (abs(calibration_x[0] - calibration_x[3]) < pdata->cal_err
	&& abs(calibration_y[0] - calibration_y[1]) < pdata->cal_err
	&& abs(calibration_x[1] - calibration_x[4]) < pdata->cal_err
	&& abs(calibration_y[3] - calibration_y[4]) < pdata->cal_err
	&& abs(calibration_x[2] - ((calibration_x[0] + calibration_x[1])/2)) < pdata->cal_err
	&& abs(calibration_x[2] - ((calibration_x[3] + calibration_x[4])/2)) < pdata->cal_err
	&& abs(calibration_y[2] - ((calibration_y[0] + calibration_y[3])/2)) < pdata->cal_err
	&& abs(calibration_y[2] - ((calibration_y[1] + calibration_y[4])/2)) < pdata->cal_err

	&& abs(abs(calibration_x[0] - calibration_x[1]) - pdata->cal_range_x) < pdata->cal_err*2
	&& abs(abs(calibration_x[3] - calibration_x[4]) - pdata->cal_range_x) < pdata->cal_err*2
	&& abs(abs(calibration_y[0] - calibration_y[3]) - pdata->cal_range_y) < pdata->cal_err*2
	&& abs(abs(calibration_y[1] - calibration_y[4]) - pdata->cal_range_y) < pdata->cal_err*2
	) {
		result = 0;
	} else {
		result = 1;
	}
	printk(KERN_INFO "calibration_calculate_varialbes(): result=%d\n", result);

	calibration_show_parameters(result);

	if (result == 1) {
		for (i = 0; i < CALIBRATION_POINTS; i++) {
			calibration_x[i] = saved_x[i];
			calibration_y[i] = saved_y[i];
		}

		calibration_x_min = saved_x_min;
		calibration_x_max = saved_x_max;
		calibration_y_min = saved_y_min;
		calibration_y_max = saved_y_max;

		calibration_flag = 9; /* Calibration error */
	} else {
		for (i = 0; i < CALIBRATION_POINTS; i++) {
			saved_x[i] = calibration_x[i];
			saved_y[i] = calibration_y[i];
		}
		calibration_flag = 8; /* Calibration ok */
	}

	adjustment_flag = 1; /* Calibration finished */
}

/*
 * Set the coordinates of calibration points from input data buffer.
 */
void calibration_set(int *inPtr)
{
	int i;
	int j = 0;
	for (i = 0; i < CALIBRATION_POINTS; i++) {
		calibration_x[i] = inPtr[j++];
		calibration_y[i] = inPtr[j++];
	}
}

/*
 * Initialize calibration parameters at boot time.
 */
void calibration_init(struct tssc_ts_platform_data *pdata)
{
	int i;
	calibration_x_min = pdata->x_min;
	calibration_x_max = pdata->x_max;
	calibration_y_min = pdata->y_min;
	calibration_y_max = pdata->y_max;

	saved_x_min = pdata->x_min;
	saved_x_max = pdata->x_max;
	saved_y_min = pdata->y_min;
	saved_y_max = pdata->y_max;

	for( i = 0; i < 5; i++) {
		screen_x[i] = pdata->cal_x[i];
		screen_y[i] = pdata->cal_y[i];
	}
}

/*
 * Display the flag of the calibration algorithm.
 * 	0: exit calibration mode.
 * 	1: enter calibration mode.
 *	2: calibration error.
 *	3: calibration ok.
 */
ssize_t calibration_show(char *buf)
{
	return sprintf(buf, "%d\n", calibration_flag);
}

/*
 * Store the flag of the calibration algorithm.
 */
void calibration_store(const char *buf, struct tssc_ts_platform_data *pdata)
{
	char *ptr_data = (char *)buf;
	unsigned long val = simple_strtoul(ptr_data, NULL, 10);

	printk(KERN_DEBUG "%s():\n", __func__);

	if (val == 0) { /* exit calibration mode */
		calibration_flag = 0;
		calibration_calculate_varialbes(pdata);
		calibration_index = 0;
		printk(KERN_DEBUG "touch_calibration: exit.\n");
	} else { /* calibration mode */
		calibration_flag = val;
		calibration_index = val - 1;

		if (val <= CALIBRATION_POINTS)
			printk(KERN_DEBUG "touch_calibration: point %d.\n",
					calibration_index + 1);
		else
			printk(KERN_ERR "touch_calibration: point error.\n");
	}
}

/*
 * Display the coordinates of the calibration touch panel input points.
 */
ssize_t calibration_points_show(char *buf)
{
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			saved_x[0], saved_y[0], saved_x[1], saved_y[1],
			saved_x[2], saved_y[2], saved_x[3], saved_y[3],
			saved_x[4], saved_y[4]);
}

/*
 * Store the coordinates of the calibration touch panel input points.
 */
void calibration_points_store(const char *buf, struct tssc_ts_platform_data *pdata)
{
	char *ptr_data = (char *)buf;
	char *p;
	int cnt = 0;
	int tmp[CALIBRATION_COORDINATES];
	unsigned long val;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
			break;

		if (cnt >= CALIBRATION_COORDINATES)
			break;

		val = simple_strtoul(p, NULL, 10);
		tmp[cnt++] = val;
	}

	if (cnt >= CALIBRATION_COORDINATES) {
		calibration_set(&tmp[0]);
		calibration_index = CALIBRATION_POINTS;
		calibration_calculate_varialbes(pdata);
		calibration_index = 0;
	}
}

/*
 * Display the coordinates of the calibration screen points.
 */
ssize_t calibration_screen_show(char *buf)
{
	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			screen_x[0], screen_y[0],
			screen_x[1], screen_y[1], screen_x[2], screen_y[2],
			screen_x[3], screen_y[3], screen_x[4], screen_y[4]);
}

/*
 * Store the coordinates of the calibration screen points.
 */
void calibration_screen_store(const char *buf)
{
	char *ptr_data = (char *)buf;
	char *p;
	int cnt = 0;
	int tmp[CALIBRATION_COORDINATES];
	unsigned long val;

	while ((p = strsep(&ptr_data, ","))) {
		if (!*p)
		    break;

		if (cnt >= CALIBRATION_COORDINATES)
		    break;

		val = simple_strtoul(p, NULL, 10);
		tmp[cnt++] = val;
	}

	if (cnt >= CALIBRATION_COORDINATES) {
		int i;
		int j = 0;

		for (i = 0; i < CALIBRATION_POINTS; i++) {
			screen_x[i] = tmp[j++];
			screen_y[i] = tmp[j++];
		}
	}
}

extern char *get_tp_cal_ram(void);
int tp_get_cal(unsigned char *data)
{
	unsigned char *cal, i;

	cal = get_tp_cal_ram();

	for (i = 0 ; i < 32; i++)
	printk("%2x ", *cal++);
	printk("\n");

	if (copy_to_user(data, cal, 32))
		return -EFAULT;
	return 0;
}

/*
 * Display the coordinates of the calibration screen points.
 */
ssize_t calibration_mfg_show(char *buf)
{
	unsigned char *cal, i;

	typedef struct _TP_CAL_IN_FLASH {
		unsigned short x1;
		unsigned short y1;
		unsigned short x2;
		unsigned short y2;
		unsigned short x3;
		unsigned short y3;
		unsigned short x4;
		unsigned short y4;
		unsigned short x5;
		unsigned short y5;
		unsigned long crc;
	} TP_CAL_IN_FLASH, *PTP_CAL_IN_FLASH;

	PTP_CAL_IN_FLASH pTpCal;
	pTpCal = (PTP_CAL_IN_FLASH) get_tp_cal_ram();

	printk("calibration_mfg_show():\n");
	cal = get_tp_cal_ram();

	for (i = 0; i < 32; i++)
	printk("%2x ", *cal++);
	printk("\n");

	printk("calibration_mfg_show(): %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lx\n",
		pTpCal->x1, pTpCal->y1, pTpCal->x2, pTpCal->y2,
		pTpCal->x3, pTpCal->y3, pTpCal->x4, pTpCal->y4,
		pTpCal->x5, pTpCal->y5, pTpCal->crc);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lx\n",
		pTpCal->x1, pTpCal->y1, pTpCal->x2, pTpCal->y2,
		pTpCal->x3, pTpCal->y3, pTpCal->x4, pTpCal->y4,
		pTpCal->x5, pTpCal->y5, pTpCal->crc);
}

/*
 * Store the coordinates of the calibration screen points.
 */
void calibration_mfg_store(const char *buf)
{
	printk("Writing to calibration_mfg is not supported\n");
}
