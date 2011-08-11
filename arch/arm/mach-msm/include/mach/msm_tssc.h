/* arch/arm/mach-msm/include/mach/msm_tssc.h
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

#ifndef __LINUX_MSM_TSSC_H
#define __LINUX_MSM_TSSC_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>

#define TSSC_MANAGER_NAME "tssc-manager"

struct tssc_vkey {
	int keycode;
	int status;
	int range_x_min;
	int range_x_max;
};

struct tssc_ts_platform_data {
	int version;
	int x_min;
	int x_max;
	int y_min;
	int y_max;
	int cal_range_x;
	int cal_range_y;
	int cal_err;
	int mfg_mode;
	int screen_width;
	int screen_height;
	int cal_x[5];
	int cal_y[5];
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_report_max;
	int abs_vkey_y_min;
	int abs_vkey_y_max;
	struct tssc_vkey *vkey;
	int vkey_num;
};

struct msm_tssc_ssbi_priorities {
	unsigned long priority0	: 3;
	unsigned long priority1	: 3;
	unsigned long priority2	: 3;
	unsigned long reserved	: 23;
};

struct msm_tssc_ssbi {
	volatile unsigned long ctl;
	volatile unsigned long reset;
	volatile unsigned long cmd;
	volatile unsigned long bypass;
	volatile unsigned long rd;
	volatile unsigned long status;
	volatile struct msm_tssc_ssbi_priorities priorities;
};

struct msm_tssc_ctl {
	unsigned long	enable		: 1;
	unsigned long	command_wr	: 1;
	unsigned long	tssc_sw_reset	: 1;
	unsigned long	mode		: 2;
	unsigned long	en_average	: 1;
	unsigned long	debounce_en	: 1;
	unsigned long	debounce_time	: 3;
	unsigned long	intr_flag1	: 1;
	unsigned long	data_flag	: 1;
	unsigned long	intr_flag2	: 1;
	unsigned long	reserved31_13	: 19;
};

struct msm_tssc_opn {
	unsigned long	resolution1	: 2;
	unsigned long	resolution2	: 2;
	unsigned long	resolution3	: 2;
	unsigned long	resolution4	: 2;
	unsigned long	num_sample1	: 2;
	unsigned long	num_sample2	: 2;
	unsigned long	num_sample3	: 2;
	unsigned long	num_sample4	: 2;
	unsigned long	operation1	: 4;
	unsigned long	operation2	: 4;
	unsigned long	operation3	: 4;
	unsigned long	operation4	: 4;
};

struct msm_tssc_sampling_int {
	unsigned long	sampling_int	: 5;
	unsigned long	reserved31_5	: 27;
};

struct msm_tssc_status {
	unsigned long	error			: 1;
	unsigned long	samples_collected	: 5;
	unsigned long	operation		: 3;
	unsigned long	penirq_status		: 1;
	unsigned long	adc_eoc_status		: 1;
	unsigned long	error_code		: 2;
	unsigned long	busy			: 1;
	unsigned long	tssc_fsm_state		: 4;
	unsigned long	tssc_ssbi_fsm_state	: 3;
	unsigned long	reserved31_21		: 11;
};

struct msm_tssc_avg_12 {
	unsigned long	samples_avg_1	: 16;
	unsigned long	samples_avg_2	: 16;
};

struct msm_tssc_avg_34 {
	unsigned long	samples_avg_3	: 16;
	unsigned long	samples_avg_4	: 16;
};

struct msm_tssc_sample_1 {
	unsigned long	raw_sample_1	: 16;
	unsigned long	raw_sample_2	: 16;
};

struct msm_tssc_sample_2 {
	unsigned long	raw_sample_3	: 16;
	unsigned long	raw_sample_4	: 16;
};

struct msm_tssc_sample_3 {
	unsigned long	raw_sample_5	: 16;
	unsigned long	raw_sample_6	: 16;
};

struct msm_tssc_sample_4 {
	unsigned long	raw_sample_7	: 16;
	unsigned long	raw_sample_8	: 16;
};

struct msm_tssc_sample_5 {
	unsigned long	raw_sample_9	: 16;
	unsigned long	raw_sample_10	: 16;
};

struct msm_tssc_sample_6 {
	unsigned long	raw_sample_11	: 16;
	unsigned long	raw_sample_12	: 16;
};

struct msm_tssc_sample_7 {
	unsigned long	raw_sample_13	: 16;
	unsigned long	raw_sample_14	: 16;
};

struct msm_tssc_sample_8 {
	unsigned long	raw_sample_15	: 16;
	unsigned long	raw_sample_16	: 16;
};

struct msm_tssc_test_1 {
	unsigned long	test_penirq_n		: 1;
	unsigned long	test_mode		: 1;
	unsigned long	gate_debounce_en	: 1;
	unsigned long	reserved_bits23_3	: 21;
	unsigned long	test_ssbi_rd_data	: 8;
};

struct msm_tssc_software_register {
	volatile struct msm_tssc_ctl tssc_ctl;
	volatile struct msm_tssc_opn tssc_opn;
	volatile struct msm_tssc_sampling_int tssc_sampling_int;
	volatile struct msm_tssc_status tssc_status;
	volatile struct msm_tssc_avg_12 tssc_avg_12;
	volatile struct msm_tssc_avg_34 tssc_avg_34;
	volatile struct msm_tssc_sample_1 tssc_sample_1_1;
	volatile struct msm_tssc_sample_2 tssc_sample_1_2;
	volatile struct msm_tssc_sample_3 tssc_sample_1_3;
	volatile struct msm_tssc_sample_4 tssc_sample_1_4;
	volatile struct msm_tssc_sample_5 tssc_sample_1_5;
	volatile struct msm_tssc_sample_6 tssc_sample_1_6;
	volatile struct msm_tssc_sample_7 tssc_sample_1_7;
	volatile struct msm_tssc_sample_8 tssc_sample_1_8;
	volatile struct msm_tssc_sample_1 tssc_sample_2_1;
	volatile struct msm_tssc_sample_2 tssc_sample_2_2;
	volatile struct msm_tssc_sample_3 tssc_sample_2_3;
	volatile struct msm_tssc_sample_4 tssc_sample_2_4;
	volatile struct msm_tssc_sample_5 tssc_sample_2_5;
	volatile struct msm_tssc_sample_6 tssc_sample_2_6;
	volatile struct msm_tssc_sample_7 tssc_sample_2_7;
	volatile struct msm_tssc_sample_8 tssc_sample_2_8;
	volatile struct msm_tssc_sample_1 tssc_sample_3_1;
	volatile struct msm_tssc_sample_2 tssc_sample_3_2;
	volatile struct msm_tssc_sample_3 tssc_sample_3_3;
	volatile struct msm_tssc_sample_4 tssc_sample_3_4;
	volatile struct msm_tssc_sample_5 tssc_sample_3_5;
	volatile struct msm_tssc_sample_6 tssc_sample_3_6;
	volatile struct msm_tssc_sample_7 tssc_sample_3_7;
	volatile struct msm_tssc_sample_8 tssc_sample_3_8;
	volatile struct msm_tssc_sample_1 tssc_sample_4_1;
	volatile struct msm_tssc_sample_2 tssc_sample_4_2;
	volatile struct msm_tssc_sample_3 tssc_sample_4_3;
	volatile struct msm_tssc_sample_4 tssc_sample_4_4;
	volatile struct msm_tssc_sample_5 tssc_sample_4_5;
	volatile struct msm_tssc_sample_6 tssc_sample_4_6;
	volatile struct msm_tssc_sample_7 tssc_sample_4_7;
	volatile struct msm_tssc_sample_8 tssc_sample_4_8;
	volatile struct msm_tssc_test_1 tssc_test_1;
	volatile unsigned long tssc_test_2;
};

// Calibration algorithm functions
ssize_t calibration_show(char *buf);
void calibration_store(const char *buf, struct tssc_ts_platform_data *pdata);
ssize_t calibration_points_show(char *buf);
void calibration_points_store(const char *buf, struct tssc_ts_platform_data *pdata);
ssize_t calibration_screen_show(char *buf);
void calibration_screen_store(const char *buf);
void calibration_init(struct tssc_ts_platform_data *pdata);
void calibration_check_mode(void);
void calibration_translate(int x, int y, int *rx, int *ry, struct tssc_ts_platform_data *pdata);
ssize_t calibration_mfg_show(char *buf);
void calibration_mfg_store(const char *buf);
#endif
