/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#ifndef __ASM__ARCH_CAMERA_H
#define __ASM__ARCH_CAMERA_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "linux/types.h"

#include "board.h"
#include "../media/msm_camera.h"

// andy
//#undef CDBG
//#define CDBG(fmt, args...) printk(KERN_INFO "msm_camera: " fmt, ##args)

#define MSM_CAMERA_MSG				0
#define MSM_CAMERA_EVT				1
#define NUM_WB_EXP_NEUTRAL_REGION_LINES		4
#define NUM_WB_EXP_STAT_OUTPUT_BUFFERS		3
#define NUM_AUTOFOCUS_MULTI_WINDOW_GRIDS	16
#define NUM_AF_STAT_OUTPUT_BUFFERS		3


enum msm_queue {
	MSM_CAM_Q_IVALID,
	MSM_CAM_Q_CTRL,
	MSM_CAM_Q_VFE_EVT,
	MSM_CAM_Q_VFE_MSG,
	MSM_CAM_Q_V4L2_REQ,
	MSM_CAM_Q_MAX
};

enum vfe_resp_msg {
	VFE_EVENT,
	VFE_MSG_GENERAL,
	VFE_MSG_SNAPSHOT,
	VFE_MSG_OUTPUT1,
	VFE_MSG_OUTPUT2,
	VFE_MSG_STATS_AF,
	VFE_MSG_STATS_WE,
	VFE_MSG_INVALID
};

struct msm_vfe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
};

struct msm_vfe_resp {
	enum vfe_resp_msg type;
	struct msm_vfe_evt_msg evt_msg;
	struct msm_vfe_phy_info  phy;
	void    *extdata;
	int32_t extlen;
};

struct msm_vfe_callback {
	void (*vfe_resp)(struct msm_vfe_resp *,
		enum msm_queue, void *syncdata);
};

struct msm_camvfe_fn {
	int (*vfe_init)      (struct msm_vfe_callback *, struct platform_device *);
	int (*vfe_enable)    (struct camera_enable_cmd *);
	int (*vfe_config)    (struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)   (struct camera_enable_cmd *,
		struct platform_device *dev);
	void (*vfe_release)  (struct platform_device *);
};

struct msm_sensor_ctrl {
	int (*s_init)(struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
};

struct msm_sync {
	spinlock_t msg_event_queue_lock;
	struct list_head msg_event_queue;
	wait_queue_head_t msg_event_wait;

	spinlock_t prev_frame_q_lock;
	struct list_head prev_frame_q;
	wait_queue_head_t prev_frame_wait;

	spinlock_t pict_frame_q_lock;
	struct list_head pict_frame_q;
	wait_queue_head_t pict_frame_wait;

	spinlock_t ctrl_status_lock;
	struct list_head ctrl_status_queue;
	wait_queue_head_t ctrl_status_wait;

        spinlock_t get_pp_q_lock;
        struct list_head get_pp_q;
        wait_queue_head_t get_pp_wait;

	spinlock_t af_status_lock;
	struct msm_ctrl_cmd af_status;
	int af_flag;
	wait_queue_head_t af_status_wait;
	struct hlist_head frame;
	struct hlist_head stats;
};

struct msm_device {
	struct msm_camvfe_fn vfefn;
	struct device *device;
	struct cdev cdev;
	struct platform_device *pdev;

	struct mutex msm_lock;
	uint8_t opencnt;

	const char *apps_id;

	void *cropinfo;
	int  croplen;

	struct mutex pict_pp_lock;
	uint32_t pict_pp;

	int sidx;
	struct msm_sensor_ctrl sctrl;

	struct mutex msm_sem;
	struct msm_sync sync;
};

/* this structure is used in kernel */
struct msm_queue_cmd {
	struct list_head list;

	/* 1 - control command or control command status;
	 * 2 - adsp event;
	 * 3 - adsp message;
	 * 4 - v4l2 request;
	 */
	enum msm_queue type;
	void             *command;
};

struct register_address_value_pair {
	uint16_t register_address;
	uint16_t register_value;
};

struct msm_pmem_region {
	struct hlist_node list;
	enum msm_pmem type;
	void *vaddr;
	unsigned long paddr;
	unsigned long len;
	struct file *file;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;
	uint8_t  active;
};

struct axidata {
	uint32_t bufnum1;
	uint32_t bufnum2;
	struct msm_pmem_region *region;
};


int32_t mt9d112_probe_init(void *, void *);
void mt9d112_exit(void);
int32_t mt9t013_probe_init(void *, void *);
void mt9t013_exit(void);
int32_t mt9p012_probe_init(void *, void *);
void mt9p012_exit(void);
int32_t s5k4b2fx_init(void *pdata);
int32_t s5k3e2fx_probe_init(void *, void *);

int32_t flash_set_led_state(enum msm_camera_led_state led_state);

/* Below functions are added for V4L2 kernel APIs */
struct msm_driver {
	struct msm_device *vmsm;
	long (*init)(struct msm_device *);
	long (*ctrl)(struct msm_ctrl_cmd *,
		struct msm_device *);

	long (*reg_pmem)(struct msm_pmem_info *,
		struct msm_device *);

	long (*get_frame) (struct msm_frame *,
		struct msm_device *);

	long (*put_frame) (struct msm_frame *,
		struct msm_device *msm);

	long (*get_pict) (struct msm_ctrl_cmd *,
		struct msm_device *msm);

	unsigned int (*drv_poll) (struct file *, struct poll_table_struct *,
		struct msm_device *msm);
	int (*release)(struct file *, struct msm_device *);
};

unsigned int msm_poll(struct file *, struct poll_table_struct *);

long msm_register(struct msm_driver *,
	const char *);
long msm_unregister(struct msm_driver *,
	const char *);

void msm_camvfe_init(void);
int msm_camvfe_check(void *);
void msm_camvfe_fn_init(struct msm_camvfe_fn *);
int msm_camera_drv_start(struct platform_device *);
int msm_camera_drv_remove(struct platform_device *);

enum msm_camio_clk_type {
	CAMIO_VFE_MDC_CLK,
	CAMIO_MDC_CLK,
	CAMIO_VFE_CLK,
	CAMIO_VFE_AXI_CLK,

	CAMIO_MAX_CLK
};

enum msm_camio_clk_src_type {
	MSM_CAMIO_CLK_SRC_INTERNAL,
	MSM_CAMIO_CLK_SRC_EXTERNAL,
	MSM_CAMIO_CLK_SRC_MAX
};

enum msm_s_test_mode {
	S_TEST_OFF,
	S_TEST_1,
	S_TEST_2,
	S_TEST_3
};

enum msm_s_resolution {
	S_QTR_SIZE,
	S_FULL_SIZE,
	S_INVALID_SIZE
};

enum msm_s_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	S_REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	S_UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	S_UPDATE_ALL,
	/* Not valid update */
	S_UPDATE_INVALID
};

enum msm_s_setting {
	S_RES_PREVIEW,
	S_RES_CAPTURE
};

int msm_camio_enable(struct platform_device *dev);

int  msm_camio_clk_enable(enum msm_camio_clk_type clk);
int  msm_camio_clk_disable(enum msm_camio_clk_type clk);
int  msm_camio_clk_config(uint32_t freq);
void msm_camio_clk_rate_set(int rate);
void msm_camio_clk_axi_rate_set(int rate);

void msm_camio_camif_pad_reg_reset(void);
void msm_camio_camif_pad_reg_reset_2(void);

void msm_camio_vfe_blk_reset(void);

void msm_camio_clk_sel(enum msm_camio_clk_src_type);
void msm_camio_disable(struct platform_device *);
int msm_camio_probe_on(struct platform_device *);
int msm_camio_probe_off(struct platform_device *);
#endif
