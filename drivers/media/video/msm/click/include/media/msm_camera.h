/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */
#ifndef __LINUX_MSM_CAMERA_H
#define __LINUX_MSM_CAMERA_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#endif
#include <linux/msm_adsp.h>

#undef CDBG
#ifdef __KERNEL__
#define CDBG(fmt, args...) /*printk(KERN_INFO "msm_camera: " fmt, ##args)*/  /*CC090518*/
#define CDBG_INFO(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)
#define CDBG_ERR(fmt, args...) printk(KERN_INFO "msm_camera error: " fmt, ##args)
#define CDBG_WARING(fmt, args...) printk(KERN_INFO "msm_camera waring: " fmt, ##args)
#else
#ifdef LOG_DEBUG
#include <utils/Log.h>
#define CDBG(fmt, args...) LOGI(fmt, ##args)
#else
#define CDBG(fmt, args...) fprintf(stderr, fmt, ##args)
#endif
#endif

#define MSM_CAM_IOCTL_MAGIC 'm'

#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmt *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_FRAMEE_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info *)

#define MSM_CAM_IOCTL_PICT_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PICT_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data *)

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, enum msm_camera_led_state *)

#define MSM_CAM_IOCTL_AF_CTRL \
	_IOR(MSM_CAM_IOCTL_MAGIC, 23, struct msm_ctrl_cmt *)
#define MSM_CAM_IOCTL_AF_CTRL_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmt *)

#define MAX_SENSOR_NUM  3
#define MAX_SENSOR_NAME 32

#define PP_SNAP  0x01
#define PP_RAW_SNAP ((0x01)<<1)
#define PP_PREV  ((0x01)<<2)

/*****************************************************
 *  enum
 *****************************************************/
enum msm_camera_update {
	MSM_CAM_CTRL_CMD_DONE,
	MSM_CAM_SENSOR_VFE_CMD,
};

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd {
	int timeout_ms;
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
};

struct msm_vfe_evt_msg {
	unsigned short type;	/* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len;	/* size in, number of bytes out */
	unsigned char *data;
};

enum msm_camera_resp {
	MSM_CAM_RESP_CTRL,
	MSM_CAM_RESP_STAT_EVT_MSG,
	MSM_CAM_RESP_V4L2,
	MSM_CAM_RESP_MAX
};

/* this one is used to send ctrl/status up to config thread */
struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	enum msm_camera_resp resptype;
	int timeout_ms;
	struct msm_ctrl_cmd ctrl_cmd;
	/* struct  vfe_event  stats_event; */
	struct msm_vfe_evt_msg stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

enum cfg_cmd_type {
	CMD_GENERAL,
	CMD_AXI_CFG_OUT1,
	CMD_AXI_CFG_SNAP_O1_AND_O2,
	CMD_AXI_CFG_OUT2,
	CMD_PICT_T_AXI_CFG,
	CMD_PICT_M_AXI_CFG,
	CMD_RAW_PICT_AXI_CFG,
	CMD_STATS_AXI_CFG,
	CMD_STATS_AF_AXI_CFG,
	CMD_FRAME_BUF_RELEASE,
	CMD_PREV_BUF_CFG,
	CMD_SNAP_BUF_RELEASE,
	CMD_SNAP_BUF_CFG,
	CMD_STATS_DISABLE,
	CMD_STATS_ENABLE,
	CMD_STATS_AF_ENABLE,
	CMD_STATS_BUF_RELEASE,
	CMD_STATS_AF_BUF_RELEASE,
	UPDATE_STATS_INVALID
};

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	enum cfg_cmd_type cmd_type;
	uint16_t length;
	void *value;
};

struct camera_enable_cmd {
	char *name;
	uint16_t length;
};

enum msm_pmem {
	MSM_PMEM_OUTPUT1,
	MSM_PMEM_OUTPUT2,
	MSM_PMEM_OUTPUT1_OUTPUT2,
	MSM_PMEM_THUMBNAIL,
	MSM_PMEM_MAINIMG,
	MSM_PMEM_RAW_MAINIMG,
	MSM_PMEM_AEC_AWB,
	MSM_PMEM_AF,
	MSM_PMEM_MAX
};

enum msm_camera_out_frame {
	FRAME_PREVIEW_OUTPUT1,
	FRAME_PREVIEW_OUTPUT2,
	FRAME_SNAPSHOT,
	FRAME_THUMBAIL,
	FRAME_RAW_SNAPSHOT,
	FRAME_MAX
};

struct msm_pmem_info {
	enum msm_pmem type;
	int fd;
	void *vaddr;
	uint32_t y_off;
	uint32_t cbcr_off;
	uint8_t active;
};

struct outputCfg {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

enum vfeoutput_mode {
	OUTPUT_1,
	OUTPUT_2,
	OUTPUT_1_AND_2,
	CAMIF_TO_AXI_VIA_OUTPUT_2,
	OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2,
	OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1,
	LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1
};

enum msm_frame_path {
	MSM_FRAME_PREV_1,
	MSM_FRAME_PREV_2,
	MSM_FRAME_ENC,
};

struct msm_frame {
	enum msm_frame_path path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
};

enum stat_type {
	STAT_AEAW,
	STAT_AF,
	STAT_MAX,
};

struct msm_stats_buf {
	enum stat_type type;
	unsigned long buffer;
	int fd;
};

enum msm_v4l2_ctrl {
	MSM_V4L2_VID_CAP_TYPE,
	MSM_V4L2_STREAM_ON,
	MSM_V4L2_STREAM_OFF,
	MSM_V4L2_SNAPSHOT,
	MSM_V4L2_QUERY_CTRL,
	MSM_V4L2_GET_CTRL,
	MSM_V4L2_SET_CTRL,
	MSM_V4L2_QUERY,
	MSM_V4L2_MAX
};

struct crop_info {
	void *info;
	int len;
};

struct msm_postproc {
	int ftnum;
	struct msm_frame fthumnail;
	int fmnum;
	struct msm_frame fmain;
};

struct msm_snapshot_pp_status {
	void *status;
};

enum sensor_cfg {
	CFG_SET_MODE,
	CFG_SET_EFFECT,
	CFG_START,
	CFG_PWR_UP,
	CFG_PWR_DOWN,
	CFG_WRITE_EXPOSURE_GAIN,
	CFG_SET_DEFAULT_FOCUS,
	CFG_MOVE_FOCUS,
	CFG_REGISTER_TO_REAL_GAIN,
	CFG_REAL_TO_REGISTER_GAIN,
	CFG_SET_FPS,
	CFG_SET_PICT_FPS,
	CFG_SET_BRIGHTNESS,
	CFG_SET_CONTRAST,
	CFG_SET_ZOOM,
	CFG_SET_EXPOSURE_MODE,
	CFG_SET_WB,
	CFG_SET_ANTIBANDING,
	CFG_SET_EXP_GAIN,
	CFG_SET_PICT_EXP_GAIN,
	CFG_SET_LENS_SHADING,
	CFG_GET_PICT_FPS,
	CFG_GET_PREV_L_PF,
	CFG_GET_PREV_P_PL,
	CFG_GET_PICT_L_PF,
	CFG_GET_PICT_P_PL,
	CFG_GET_PICT_MAX_EXP_LC,
	CFG_MAX
};

enum sensor_move_focus {
	MOVE_NEAR,
	MOVE_FAR
};

enum sensor_mode {
	SENSOR_PREVIEW_MODE,
	SENSOR_SNAPSHOT_MODE,
	SENSOR_RAW_SNAPSHOT_MODE
};

enum sensor_resolution {
	SENSOR_QTR_SIZE,
	SENSOR_FULL_SIZE,
	SENSOR_INVALID_SIZE,
};

enum camera_effect {
	CAMERA_EFFECT_MIN_MINUS_1,
	CAMERA_EFFECT_OFF = 1,  /* This list must match aeecamera.h */
	CAMERA_EFFECT_MONO,
	CAMERA_EFFECT_NEGATIVE,
	CAMERA_EFFECT_SOLARIZE,
	CAMERA_EFFECT_PASTEL,
	CAMERA_EFFECT_MOSAIC,
	CAMERA_EFFECT_RESIZE,
	CAMERA_EFFECT_SEPIA,
	CAMERA_EFFECT_POSTERIZE,
	CAMERA_EFFECT_WHITEBOARD,
	CAMERA_EFFECT_BLACKBOARD,
	CAMERA_EFFECT_AQUA,
	CAMERA_EFFECT_MAX_PLUS_1
};

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
};

struct focus_cfg {
	int32_t steps;
	enum sensor_move_focus dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

enum msm_camera_led_state {
	MSM_LED_OFF,
	MSM_LED_LOW,
	MSM_LED_HIGH
};

struct sensor_cfg_data {
	enum sensor_cfg  cfgtype;
	enum sensor_mode mode;
	enum sensor_resolution rs;

	union {
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg    exp_gain;
		struct focus_cfg       focus;
		struct fps_cfg	       fps;
	} cfg;
};

enum sensor_get_info {
	GET_NAME,
	GET_PREVIEW_LINE_PER_FRAME,
	GET_PREVIEW_PIXELS_PER_LINE,
	GET_SNAPSHOT_LINE_PER_FRAME,
	GET_SNAPSHOT_PIXELS_PER_LINE,
	GET_SNAPSHOT_FPS,
	GET_SNAPSHOT_MAX_EP_LINE_CNT,
};

struct msm_camsensor_info {
	char name[MAX_SENSOR_NAME];
	int8_t flash_enabled;
};
#endif /* __LINUX_MSM_CAMERA_H */
