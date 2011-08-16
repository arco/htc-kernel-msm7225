/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include "include/media/msm_camera.h"
#include <mach/gpio.h>
#include "include/mach/camera.h"
#include <linux/wakelock.h>
#include "mt9t013.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define MT9T013_REG_MODEL_ID	     0x0000
#define MT9T013_MODEL_ID             0x2600
#define REG_GROUPED_PARAMETER_HOLD   0x0104
#define GROUPED_PARAMETER_HOLD       0x0100
#define GROUPED_PARAMETER_UPDATE     0x0000
#define REG_COARSE_INT_TIME          0x3012
#define REG_VT_PIX_CLK_DIV           0x0300
#define REG_VT_SYS_CLK_DIV           0x0302
#define REG_PRE_PLL_CLK_DIV          0x0304
#define REG_PLL_MULTIPLIER           0x0306
#define REG_OP_PIX_CLK_DIV           0x0308
#define REG_OP_SYS_CLK_DIV           0x030A
#define REG_SCALE_M                  0x0404
#define REG_FRAME_LENGTH_LINES       0x300A
#define REG_LINE_LENGTH_PCK          0x300C
#define REG_X_ADDR_START             0x3004
#define REG_Y_ADDR_START             0x3002
#define REG_X_ADDR_END               0x3008
#define REG_Y_ADDR_END               0x3006
#define REG_X_OUTPUT_SIZE            0x034C
#define REG_Y_OUTPUT_SIZE            0x034E
#define REG_FINE_INT_TIME            0x3014
#define REG_ROW_SPEED                0x3016
#define MT9T013_REG_RESET_REGISTER   0x301A
#define MT9T013_RESET_REGISTER_PWON  0x10CC
#define MT9T013_RESET_REGISTER_PWOFF 0x1008 /* 0x10C8 stop streaming*/
#define MT9T013_RESET_FAST_TRANSITION 0x0002
#define REG_READ_MODE                0x3040
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

static struct wake_lock mt9t013_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&mt9t013_wake_lock, WAKE_LOCK_IDLE, "mt9t013");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&mt9t013_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&mt9t013_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&mt9t013_wake_lock);
}


enum mt9t013_test_mode {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum mt9t013_resolution {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9t013_reg_update {
  /* Sensor egisters that need to be updated during initialization */
  REG_INIT,
  /* Sensor egisters that needs periodic I2C writes */
  UPDATE_PERIODIC,
  /* All the sensor Registers will be updated */
  UPDATE_ALL,
  /* Not valid update */
  UPDATE_INVALID
};

enum mt9t013_setting {
	RES_PREVIEW,
	RES_CAPTURE
};

/* actuator's Slave Address */
#define MT9T013_AF_I2C_ADDR   0x18

/*
* AF Total steps parameters
*/
#define MT9T013_TOTAL_STEPS_NEAR_TO_FAR    30 /* 28 */

/*
 * Time in milisecs for waiting for the sensor to reset.
 */
#define MT9T013_RESET_DELAY_MSECS   66

/* for 30 fps preview */
#define MT9T013_DEFAULT_CLOCK_RATE  24000000
#define MT9T013_DEFAULT_MAX_FPS     26


/* FIXME: Changes from here */
struct mt9t013_work {
	struct work_struct work;
};

static struct  mt9t013_work *mt9t013_sensorw;
static struct  i2c_client *mt9t013_client;

struct mt9t013_ctrl {
	struct  msm_camera_sensor_info *sensordata;

	enum sensor_mode sensormode;
	uint32_t fps_divider; 		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; 	/* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9t013_resolution prev_res;
	enum mt9t013_resolution pict_res;
	enum mt9t013_resolution curr_res;
	enum mt9t013_test_mode  set_test;

	unsigned short imgaddr;
};


static struct mt9t013_ctrl *mt9t013_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9t013_wait_queue);
DECLARE_MUTEX(mt9t013_sem);


/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct mt9t013_reg mt9t013_regs;	/* from mt9t013_reg.c */



/*=============================================================*/

static int mt9t013_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr  = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(mt9t013_client->adapter, msgs, 2) < 0) {
		CDBG("mt9t013_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t013_i2c_read_w(unsigned short saddr,
	unsigned short raddr, unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9t013_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CDBG("mt9t013_i2c_read failed!\n");

	return rc;
}

static int32_t mt9t013_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
	{
		.addr = saddr,
		.flags = 0,
		.len = length,
		.buf = txdata,
	},
	};

	if (i2c_transfer(mt9t013_client->adapter, msg, 1) < 0) {
		CDBG("mt9t013_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t013_i2c_write_b(unsigned short saddr,
	unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = wdata;
	rc = mt9t013_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		CDBG("i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9t013_i2c_write_w(unsigned short saddr,
	unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9t013_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9t013_i2c_write_w_table(
	struct mt9t013_i2c_reg_conf *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int32_t rc = -EFAULT;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9t013_i2c_write_w(mt9t013_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

static int32_t mt9t013_test(enum mt9t013_test_mode mo)
{
	int32_t rc = 0;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	if (mo == TEST_OFF)
		return 0;
	else {
		rc = mt9t013_i2c_write_w_table(mt9t013_regs.ttbl, 
									   mt9t013_regs.ttbl_size);
		if (rc < 0)
			return rc;
		
		rc = mt9t013_i2c_write_w(mt9t013_client->addr, 
								 REG_TEST_PATTERN_MODE, (uint16_t)mo);
		if (rc < 0)
			return rc;
	}

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_set_lc(void)
{
	int32_t rc;

/*	rc = mt9t013_i2c_write_w_table(mt9t013_regs.lctbl, ARRAY_SIZE(mt9t013_regs.lctbl)); */
	rc = mt9t013_i2c_write_w_table(mt9t013_regs.lctbl, mt9t013_regs.lctbl_size);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	uint8_t code_val_msb, code_val_lsb;
	code_val_msb = 0x01;
	code_val_lsb = af_step;

	/* Write the digital code for current to the actuator */
	rc =
		mt9t013_i2c_write_b(MT9T013_AF_I2C_ADDR>>1,
			code_val_msb, code_val_lsb);

	mt9t013_ctrl->curr_lens_pos = 0;
	mt9t013_ctrl->init_curr_lens_pos = 0;
	return rc;
}

static void mt9t013_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;   /*Q10 */
	uint32_t pclk_mult; /*Q10 */
	uint32_t d1;
	uint32_t d2;

	d1 =
		(uint32_t)(
		(mt9t013_regs.reg_pat[RES_PREVIEW].frame_length_lines *
		0x00000400) /
		mt9t013_regs.reg_pat[RES_CAPTURE].frame_length_lines);

	d2 =
		(uint32_t)(
		(mt9t013_regs.reg_pat[RES_PREVIEW].line_length_pck *
		0x00000400) /
		mt9t013_regs.reg_pat[RES_CAPTURE].line_length_pck);

	divider = (uint32_t) (d1 * d2) / 0x00000400;

	pclk_mult =
		(uint32_t) ((mt9t013_regs.reg_pat[RES_CAPTURE].pll_multiplier *
		0x00000400) /
		(mt9t013_regs.reg_pat[RES_PREVIEW].pll_multiplier));

	/* Verify PCLK settings and frame sizes. */
	*pfps =
		(uint16_t) (fps * divider * pclk_mult /
		0x00000400 / 0x00000400);
}

static uint16_t mt9t013_get_prev_lines_pf(void)
{
	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		return mt9t013_regs.reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9t013_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9t013_get_prev_pixels_pl(void)
{
	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		return mt9t013_regs.reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9t013_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9t013_get_pict_lines_pf(void)
{
	return mt9t013_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9t013_get_pict_pixels_pl(void)
{
	return mt9t013_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9t013_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9t013_ctrl->pict_res == QTR_SIZE) {
		snapshot_lines_per_frame =
		mt9t013_regs.reg_pat[RES_PREVIEW].frame_length_lines - 1;
	} else  {
		snapshot_lines_per_frame =
		mt9t013_regs.reg_pat[RES_CAPTURE].frame_length_lines - 1;
	}

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9t013_set_fps(struct fps_cfg	*fps)
{
	/* input is new fps in Q8 format */
	int32_t rc = 0;
	enum mt9t013_setting setting;

	mt9t013_ctrl->fps_divider = fps->fps_div;
	mt9t013_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

	CDBG("mt9t013_set_fps: fps_div is %d, f_mult is %d\n",
			fps->fps_div, fps->f_mult);

	if (mt9t013_ctrl->sensormode == SENSOR_PREVIEW_MODE)
		setting = RES_PREVIEW;
	else
		setting = RES_CAPTURE;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_FRAME_LENGTH_LINES,
			(uint16_t) (
			mt9t013_regs.reg_pat[setting].frame_length_lines *
			fps->fps_div / 0x00000400));

	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

#if 0 // andy: comment by andy
static int32_t mt9t013_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0x01FF;
	int32_t rc = 0;

	if (mt9t013_ctrl->sensormode ==
			SENSOR_PREVIEW_MODE) {
		mt9t013_ctrl->my_reg_gain = gain;
		mt9t013_ctrl->my_reg_line_count = (uint16_t) line;
	}

/*	if (gain > line_length_ratio) wilson change this*/
	if (gain > max_legal_gain)
		gain = max_legal_gain;

	if (mt9t013_ctrl->sensormode != SENSOR_SNAPSHOT_MODE)
		line = (uint32_t) (line * mt9t013_ctrl->fps_divider /
				   0x00000400);
	else
		line = (uint32_t) (line * mt9t013_ctrl->pict_fps_divider /
				   0x00000400);

	/*Set digital gain to 1 */
	gain |= 0x0200;

	/* There used to be PARAMETER_HOLD register write before and
	 * after REG_GLOBAL_GAIN & REG_COARSE_INIT_TIME. This causes
	 * aec oscillation. Hence removed. */ 
	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GLOBAL_GAIN, gain);
	if (rc < 0)
		return rc;
#if 0/*wilson for low light eposure */
	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line * line_length_ratio /
			line_length_ratio));
#endif
/*add by wilson for writing line to sensor*/
	rc = mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_COARSE_INT_TIME, line);
	if (rc < 0)
		return rc;
	return rc;
}

#endif

// andy: previous QCT code works.
static int32_t mt9t013_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0x01FF;
	uint32_t line_length_ratio = 0x00000400;
	enum mt9t013_setting setting;
	int32_t rc = 0;

	if (mt9t013_ctrl->sensormode ==
			SENSOR_PREVIEW_MODE) {
		mt9t013_ctrl->my_reg_gain = gain;
		mt9t013_ctrl->my_reg_line_count = (uint16_t) line;
	}

// andy: sync with mt9p012 code, it's more reasonable
	if (gain > max_legal_gain) //0x00000400)
		gain = max_legal_gain;

	/* Verify no overflow */
	if (mt9t013_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line =
			(uint32_t) (line * mt9t013_ctrl->fps_divider /
			0x00000400);

		setting = RES_PREVIEW;

	} else {
		line =
			(uint32_t) (line * mt9t013_ctrl->pict_fps_divider /
			0x00000400);

		setting = RES_CAPTURE;
	}

	/*Set digital gain to 1 */
	gain |= 0x0200;

	if ((mt9t013_regs.reg_pat[setting].frame_length_lines - 1) < line) {

		line_length_ratio =
		(uint32_t) (line * 0x00000400) /
		(mt9t013_regs.reg_pat[setting].frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

	/* There used to be PARAMETER_HOLD register write before and
	 * after REG_GLOBAL_GAIN & REG_COARSE_INIT_TIME. This causes
	 * aec oscillation. Hence removed. */

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GLOBAL_GAIN, gain);
	if (rc < 0)
		return rc;
/*CC090610*/
/*
	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line * 0x00000400 /
			line_length_ratio));
*/
	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line));
/*CC090610~*/
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	rc =
		mt9t013_write_exp_gain(gain, line);
	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			MT9T013_REG_RESET_REGISTER,
			0x10CC | 0x0002);

	mdelay(5);

	return rc;
}

static int32_t mt9t013_setting(enum mt9t013_reg_update rupdate,
	enum mt9t013_setting rt)
{
	int32_t rc = 0;

	switch (rupdate) {
	case UPDATE_PERIODIC: {

	if (rt == RES_PREVIEW ||
			rt == RES_CAPTURE) {
#if 0
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWOFF);
		if (rc < 0)
			return rc;
#endif

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_VT_PIX_CLK_DIV,
				mt9t013_regs.reg_pat[rt].vt_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_VT_SYS_CLK_DIV,
				mt9t013_regs.reg_pat[rt].vt_sys_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_PRE_PLL_CLK_DIV,
				mt9t013_regs.reg_pat[rt].pre_pll_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_PLL_MULTIPLIER,
				mt9t013_regs.reg_pat[rt].pll_multiplier);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_OP_PIX_CLK_DIV,
				mt9t013_regs.reg_pat[rt].op_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_OP_SYS_CLK_DIV,
				mt9t013_regs.reg_pat[rt].op_sys_clk_div);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_ROW_SPEED,
				mt9t013_regs.reg_pat[rt].row_speed);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_ADDR_START,
				mt9t013_regs.reg_pat[rt].x_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_ADDR_END,
				mt9t013_regs.reg_pat[rt].x_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_ADDR_START,
				mt9t013_regs.reg_pat[rt].y_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_ADDR_END,
				mt9t013_regs.reg_pat[rt].y_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_READ_MODE,
				mt9t013_regs.reg_pat[rt].read_mode);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_SCALE_M,
				mt9t013_regs.reg_pat[rt].scale_m);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_OUTPUT_SIZE,
				mt9t013_regs.reg_pat[rt].x_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_OUTPUT_SIZE,
				mt9t013_regs.reg_pat[rt].y_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_LINE_LENGTH_PCK,
				mt9t013_regs.reg_pat[rt].line_length_pck);
		if (rc < 0)
			return rc;

#if 0  /*Maverick 20090918*/
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_FRAME_LENGTH_LINES,
			(mt9t013_regs.reg_pat[rt].frame_length_lines *
			mt9t013_ctrl->fps_divider / 0x00000400));
		if (rc < 0)
			return rc;
#else
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_FRAME_LENGTH_LINES,
			(mt9t013_regs.reg_pat[rt].frame_length_lines));
		if (rc < 0)
			return rc;
#endif

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_COARSE_INT_TIME,
			mt9t013_regs.reg_pat[rt].coarse_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_FINE_INT_TIME,
			mt9t013_regs.reg_pat[rt].fine_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
		if (rc < 0)
			return rc;

		rc = mt9t013_test(mt9t013_ctrl->set_test);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
			MT9T013_REG_RESET_REGISTER,
			MT9T013_RESET_REGISTER_PWON|
			MT9T013_RESET_FAST_TRANSITION);
		if (rc < 0)
			return rc;

		mdelay(5);

		return rc;
	}
	}
		break;

	/*CAMSENSOR_REG_UPDATE_PERIODIC */
	case REG_INIT: {
	if (rt == RES_PREVIEW ||
			rt == RES_CAPTURE) {

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWOFF);
		if (rc < 0)
			/* MODE_SELECT, stop streaming */
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_VT_PIX_CLK_DIV,
				mt9t013_regs.reg_pat[rt].vt_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_VT_SYS_CLK_DIV,
				mt9t013_regs.reg_pat[rt].vt_sys_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_PRE_PLL_CLK_DIV,
				mt9t013_regs.reg_pat[rt].pre_pll_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_PLL_MULTIPLIER,
				mt9t013_regs.reg_pat[rt].pll_multiplier);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_OP_PIX_CLK_DIV,
				mt9t013_regs.reg_pat[rt].op_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_OP_SYS_CLK_DIV,
				mt9t013_regs.reg_pat[rt].op_sys_clk_div);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		/* additional power saving mode ok around 38.2MHz */
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				0x3084, 0x2409);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				0x3092, 0x0A49);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				0x3094, 0x4949);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				0x3096, 0x4949);
		if (rc < 0)
			return rc;

		/* Set preview or snapshot mode */
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_ROW_SPEED,
				mt9t013_regs.reg_pat[rt].row_speed);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_ADDR_START,
				mt9t013_regs.reg_pat[rt].x_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_ADDR_END,
				mt9t013_regs.reg_pat[rt].x_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_ADDR_START,
				mt9t013_regs.reg_pat[rt].y_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_ADDR_END,
				mt9t013_regs.reg_pat[rt].y_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_READ_MODE,
				mt9t013_regs.reg_pat[rt].read_mode);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_SCALE_M,
				mt9t013_regs.reg_pat[rt].scale_m);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_X_OUTPUT_SIZE,
				mt9t013_regs.reg_pat[rt].x_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_Y_OUTPUT_SIZE,
				mt9t013_regs.reg_pat[rt].y_output_size);
		if (rc < 0)
			return 0;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_LINE_LENGTH_PCK,
				mt9t013_regs.reg_pat[rt].line_length_pck);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_FRAME_LENGTH_LINES,
				mt9t013_regs.reg_pat[rt].frame_length_lines);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_COARSE_INT_TIME,
				mt9t013_regs.reg_pat[rt].coarse_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_FINE_INT_TIME,
				mt9t013_regs.reg_pat[rt].fine_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_UPDATE);
			if (rc < 0)
				return rc;

		/* load lens shading */
		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		/* most likely needs to be written only once. */
		rc = mt9t013_set_lc();
		if (rc < 0)
			return -EBUSY;

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_UPDATE);
		if (rc < 0)
			return rc;

		rc = mt9t013_test(mt9t013_ctrl->set_test);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(mt9t013_client->addr,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWON);
		if (rc < 0)
			/* MODE_SELECT, stop streaming */
			return rc;

		CDBG("!!! mt9t013 !!! PowerOn is done!\n");
		mdelay(5);
		return rc;
		}
		} /* case CAMSENSOR_REG_INIT: */
		break;

	/*CAMSENSOR_REG_INIT */
	default:
		rc = -EFAULT;
		break;
	} /* switch (rupdate) */

	return rc;
}

static int32_t mt9t013_video_config(enum sensor_mode mode,
	enum sensor_resolution res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
		rc = mt9t013_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

			CDBG("sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc = mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;
		break;

	default:
		return 0;
	} /* switch */

	mt9t013_ctrl->prev_res = res;
	mt9t013_ctrl->curr_res = res;
	mt9t013_ctrl->sensormode = mode;

	rc = mt9t013_write_exp_gain(mt9t013_ctrl->my_reg_gain,
			mt9t013_ctrl->my_reg_line_count);
	if (rc < 0)
		return rc;

	rc = mt9t013_i2c_write_w(mt9t013_client->addr,
		MT9T013_REG_RESET_REGISTER,
		MT9T013_RESET_REGISTER_PWON|MT9T013_RESET_FAST_TRANSITION);
	if (rc < 0)
		return rc;

	msleep(5);
	return rc;
}

static int32_t mt9t013_snapshot_config(enum sensor_mode mode)
{
	int32_t rc = 0;

	rc =
		mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t013_ctrl->curr_res = mt9t013_ctrl->pict_res;

	mt9t013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9t013_raw_snapshot_config(enum sensor_mode mode)
{
	int32_t rc = 0;

	rc = mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t013_ctrl->curr_res = mt9t013_ctrl->pict_res;

	mt9t013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9t013_power_down(void)
{
	int32_t rc = 0;

	rc =
		mt9t013_i2c_write_w(mt9t013_client->addr,
			MT9T013_REG_RESET_REGISTER,
			MT9T013_RESET_REGISTER_PWOFF);

	mdelay(5);

	return rc;
}

static int32_t mt9t013_move_focus(enum sensor_move_focus direction,
	int32_t num_steps)
{
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	int16_t break_steps[4];
	uint8_t code_val_msb, code_val_lsb;
	int16_t i;

	if (num_steps > MT9T013_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9T013_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0)
		return -EINVAL;

	if (direction == MOVE_NEAR)
		step_direction = 4;
	else if (direction == MOVE_FAR)
		step_direction = -4;
	else
		return -EINVAL;

	if (mt9t013_ctrl->curr_lens_pos < mt9t013_ctrl->init_curr_lens_pos)
		mt9t013_ctrl->curr_lens_pos = mt9t013_ctrl->init_curr_lens_pos;

	actual_step =
		(int16_t) (step_direction *
		(int16_t) num_steps);

	for (i = 0; i < 4; i++)
		break_steps[i] =
			actual_step / 4 * (i + 1) - actual_step / 4 * i;

	for (i = 0; i < 4; i++) {
		next_position =
		(int16_t)
		(mt9t013_ctrl->curr_lens_pos + break_steps[i]);

		if (next_position > 255)
			next_position = 255;
		else if (next_position < 0)
			next_position = 0;

		code_val_msb =
		((next_position >> 4) << 2) |
		((next_position << 4) >> 6);

		code_val_lsb =
		((next_position & 0x03) << 6);

		/* Writing the digital code for current to the actuator */
		if (mt9t013_i2c_write_b(MT9T013_AF_I2C_ADDR>>1,
				code_val_msb, code_val_lsb) < 0)
			return -EBUSY;

		/* Storing the current lens Position */
		mt9t013_ctrl->curr_lens_pos = next_position;

		if (i < 3)
			mdelay(1);
	} /* for */

	return 0;
}

static int mt9t013_sensor_init_done(struct msm_camera_sensor_info *data)
{
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9t013_probe_init_sensor(struct msm_camera_sensor_info *data)
{
	int rc;
	uint16_t chipid;

	rc = gpio_request(data->sensor_reset, "mt9t013");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else
		goto init_probe_done;

	msleep(20);

	/* RESET the sensor image part via I2C command */
	rc = mt9t013_i2c_write_w(mt9t013_client->addr,
		MT9T013_REG_RESET_REGISTER, 0x1009);
	if (rc < 0)
		goto init_probe_fail;

	msleep(MT9T013_RESET_DELAY_MSECS);

	/* 3. Read sensor Model ID: */
	rc = mt9t013_i2c_read_w(mt9t013_client->addr,
		MT9T013_REG_MODEL_ID, &chipid);

	if (rc < 0)
		goto init_probe_fail;

	CDBG("mt9t013 model_id = 0x%x\n", chipid);

	/* 4. Compare sensor ID to MT9T012VC ID: */
	if (chipid != MT9T013_MODEL_ID) {
		rc = -ENODEV;
		goto init_probe_fail;
	}

	rc = mt9t013_i2c_write_w(mt9t013_client->addr,
		0x3064, 0x0805);
	if (rc < 0)
		goto init_probe_fail;

	msleep(MT9T013_RESET_DELAY_MSECS);

  goto init_probe_done;

	/* sensor: output enable */
#if 0
	rc = mt9t013_i2c_write_w(mt9t013_client->addr,
		MT9T013_REG_RESET_REGISTER,
		MT9T013_RESET_REGISTER_PWON);

	/* if this fails, the sensor is not the MT9T013 */
	rc = mt9t013_set_default_focus(0);
#endif

init_probe_fail:
	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);
init_probe_done:
	return rc;
}

int mt9t013_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int32_t  rc;

	mt9t013_ctrl = kzalloc(sizeof(struct mt9t013_ctrl), GFP_KERNEL);
	if (!mt9t013_ctrl) {
		CDBG("mt9t013_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9t013_ctrl->fps_divider = 1 * 0x00000400;
	mt9t013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9t013_ctrl->set_test = TEST_OFF;
	mt9t013_ctrl->prev_res = QTR_SIZE;
	mt9t013_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9t013_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9T013_DEFAULT_CLOCK_RATE);
	mdelay(20);

	msm_camio_camif_pad_reg_reset();
	mdelay(20);

  rc = mt9t013_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		rc = mt9t013_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9t013_setting(REG_INIT, RES_CAPTURE);

	if (rc < 0)
		goto init_fail;
	else
		goto init_done;

init_fail:
	kfree(mt9t013_ctrl);
init_done:
	prevent_suspend();
	return rc;
}

static int mt9t013_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9t013_wait_queue);
	return 0;
}


static int32_t mt9t013_set_sensor_mode(enum sensor_mode mode,
            enum sensor_resolution res)
{
            int32_t rc = 0;
            rc =
                        mt9t013_i2c_write_w(mt9t013_client->addr,
                                    REG_GROUPED_PARAMETER_HOLD,
                                    GROUPED_PARAMETER_HOLD);
            if (rc < 0)
                        return rc;
            switch (mode) {
            case SENSOR_PREVIEW_MODE:
                        rc = mt9t013_video_config(mode, res);
                        break;

            case SENSOR_SNAPSHOT_MODE:
                        rc = mt9t013_snapshot_config(mode);
                        break;

            case SENSOR_RAW_SNAPSHOT_MODE:
                        rc = mt9t013_raw_snapshot_config(mode);
                        break;
            default:
                        rc = -EINVAL;
                        break;
            }
            rc =
                        mt9t013_i2c_write_w(mt9t013_client->addr,
                                    REG_GROUPED_PARAMETER_HOLD,
                                    GROUPED_PARAMETER_UPDATE);
            return rc;
}

int mt9t013_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	down(&mt9t013_sem);

  CDBG("mt9t013_sensor_config: cfgtype = %d\n",
	  cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
				mt9t013_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9t013_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9t013_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9t013_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9t013_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9t013_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9t013_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9t013_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9t013_set_pict_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9t013_set_sensor_mode(cdata.mode,
						cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9t013_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9t013_move_focus(
					cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9t013_set_default_focus(
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_EFFECT:
			rc = mt9t013_set_default_focus(
						cdata.cfg.effect);
			break;

		default:
			rc = -EFAULT;
			break;
		}

	up(&mt9t013_sem);

	return rc;
}

int mt9t013_sensor_release(void)
{
	int rc = -EBADF;

	down(&mt9t013_sem);

	mt9t013_power_down();

	gpio_direction_output(mt9t013_ctrl->sensordata->sensor_reset,
			0);
	gpio_free(mt9t013_ctrl->sensordata->sensor_reset);

	kfree(mt9t013_ctrl);

	up(&mt9t013_sem);
	CDBG("mt9t013_release completed!\n");
	
	allow_suspend();
	return rc;
}

/*CC090708*/
static const char *MT9T013Vendor = "micron";
static const char *MT9T013NAME = "mt9t013";
static const char *MT9T013Size = "3M";



static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", MT9T013Vendor, MT9T013NAME, MT9T013Size);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);


static struct kobject *android_mt9t013;

static int mt9t013_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "mt9t013:kobject creat and add\n");
	android_mt9t013 = NULL;
	android_mt9t013 = kobject_create_and_add("android_camera", NULL);
	if (android_mt9t013 == NULL) {
		printk(KERN_INFO "mt9t013_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "mt9t013:sysfs_create_file\n");
	ret = sysfs_create_file(android_mt9t013, &dev_attr_sensor.attr);
	if (ret) {
		printk(KERN_INFO "mt9t013_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_mt9t013);
	}
	return 0 ;
}
/*CC090708~*/

static int mt9t013_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	mt9t013_sensorw =
		kzalloc(sizeof(struct mt9t013_work), GFP_KERNEL);

	if (!mt9t013_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9t013_sensorw);
	mt9t013_init_client(client);
	mt9t013_client = client;
	mt9t013_client->addr = mt9t013_client->addr >> 1;
	mdelay(50);
	init_suspend();
    /*CC090708*/
	mt9t013_sysfs_init();
	return 0;

probe_failure:
	kfree(mt9t013_sensorw);
	mt9t013_sensorw = NULL;
	return rc;
}

static int __exit mt9t013_remove(struct i2c_client *client)
{
	struct mt9t013_work *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	//i2c_detach_client(client);
	mt9t013_client = NULL;
	kfree(sensorw);
	deinit_suspend();
	return 0;
}

static const struct i2c_device_id mt9t013_id[] = {
	{ "mt9t013", 0},
	{ }
};

static struct i2c_driver mt9t013_driver = {
	.id_table = mt9t013_id,
	.probe  = mt9t013_probe,
	.remove = __exit_p(mt9t013_remove),
	.driver = {
		.name = "mt9t013",
	},
};

static int32_t mt9t013_init(void)
{
	int32_t rc = 0;

	rc = i2c_add_driver(&mt9t013_driver);
	if (IS_ERR_VALUE(rc))
		goto init_failure;
	return rc;

init_failure:
	CDBG("mt9t013_init failed\n");
	return rc;
}

void mt9t013_exit(void)
{
	i2c_del_driver(&mt9t013_driver);
}

int mt9t013_probe_init(void *dev, void *ctrl)
{
	int rc = 0;
	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev;

	struct msm_sensor_ctrl *s = (struct msm_sensor_ctrl *)ctrl;
	rc = mt9t013_init();
	if (rc < 0)
		goto probe_done;

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9T013_DEFAULT_CLOCK_RATE);
	mdelay(20);

	rc = mt9t013_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9t013_sensor_open_init;
	s->s_release = mt9t013_sensor_release;
	s->s_config  = mt9t013_sensor_config;
	mt9t013_sensor_init_done(info);
	
probe_done:
	return rc;
}
