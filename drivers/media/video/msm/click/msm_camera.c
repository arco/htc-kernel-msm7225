/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include "include/mach/board.h"

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include "include/media/msm_camera.h"
#include "include/mach/camera.h"
#include <mach/perflock.h>
static struct perf_lock camera_perf_lock;
struct msm_device **msm_camera;
//static struct msm_sync msm_sync;
static struct class *msm_class;
static dev_t msm_devno;

static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info, struct file *file, unsigned long paddr,
	unsigned long len, int fd)
{
	struct msm_pmem_region *region =
		kmalloc(sizeof(*region), GFP_KERNEL);

	if (!region){
		CDBG_ERR("msm_pmem_table_add:cannot allocate buffer region\n");
		return -ENOMEM;
	}
	INIT_HLIST_NODE(&region->list);

	region->type = info->type;
	region->vaddr = info->vaddr;
	region->paddr = paddr;
	region->len = len;
	region->file = file;
	region->y_off = info->y_off;
	region->cbcr_off = info->cbcr_off;
	region->fd = fd;
	region->active = info->active;

	hlist_add_head(&(region->list), ptype);

	return 0;
}

static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	enum msm_pmem type, struct msm_pmem_region *reg, uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node;

	uint8_t rc = 0;

	regptr = reg;

	hlist_for_each_entry(region, node, ptype, list) {

		if ((region->type == type) &&
				(region->active)) {
			*regptr = *region;

			rc += 1;
			if (rc >= maxcount)
				break;

			regptr++;
		}
	}

	return rc;
}

static unsigned long msm_pmem_frame_ptov_lookup(unsigned long pyaddr,
	unsigned long pcbcraddr, uint32_t *yoff, uint32_t *cbcroff, int *fd,
	struct msm_device	*msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;
	unsigned long rc = 0;

	hlist_for_each_entry(region, node,
		&msm->sync.frame, list) {

		if (pyaddr == (region->paddr + region->y_off) &&
		pcbcraddr == (region->paddr + region->cbcr_off) &&
		region->active) {

			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer */
			rc = (unsigned long)(region->vaddr);
			*yoff = region->y_off;
			*cbcroff = region->cbcr_off;
			*fd = region->fd;
			region->active = 0;

			return rc;
		}
	}

	return 0;
}

static unsigned long msm_pmem_stats_ptov_lookup(unsigned long addr, int *fd,
	struct msm_device *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;
	unsigned long rc = 0;

	hlist_for_each_entry(region, node,
		&msm->sync.stats, list) {

		if (addr == region->paddr &&
				region->active) {
			/* offset since we could pass vaddr inside a
			*  registered pmem buffer */
			rc = (unsigned long)(region->vaddr);
			*fd = region->fd;
			region->active = 0;
			return rc;
		}
	}

	return 0;
}

static void msm_pmem_frame_vtop_lookup(unsigned long buffer,
	uint32_t yoff, uint32_t cbcroff, int fd, unsigned long *phyaddr,
	struct msm_device *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;

	hlist_for_each_entry(region,
		node, &msm->sync.frame, list) {

		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->y_off == yoff) &&
				(region->cbcr_off == cbcroff) &&
				(region->fd == fd) &&
				(region->active == 0)) {

			*phyaddr = region->paddr;
			region->active = 1;

			return;
		}
	}

	*phyaddr = 0;
}

static void msm_pmem_stats_vtop_lookup(unsigned long buffer,
	int fd, unsigned long *phyaddr, struct msm_device *msm)
{
	struct msm_pmem_region *region;
	struct hlist_node *node;

	hlist_for_each_entry(region, node,
		&msm->sync.stats, list) {

		if (((unsigned long)(region->vaddr) == buffer) &&
				(region->fd == fd) &&
				region->active == 0) {

			*phyaddr = region->paddr;
			region->active = 1;

			return;
		}
	}

	*phyaddr = 0;
}

static long msm_pmem_table_del_proc(struct msm_pmem_info *pinfo,
	struct msm_device *msm)
{
	long rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node;
	struct hlist_node *n;

	mutex_lock(&msm->msm_sem);
	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
		hlist_for_each_entry_safe(region, node, n,
			&msm->sync.frame, list) {

			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		hlist_for_each_entry_safe(region, node, n,
			&msm->sync.stats, list) {

			if (pinfo->type == region->type &&
					pinfo->vaddr == region->vaddr &&
					pinfo->fd == region->fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	default:
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&msm->msm_sem);

	return rc;
}

static long msm_pmem_table_del(void __user *arg,
	struct msm_device *msm)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))){
		CDBG_ERR("msm_pmem_table_del:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&info,__LINE__);
		return -EFAULT;
	}
	return msm_pmem_table_del_proc(&info, msm);
}

static long msm_get_frame_proc(struct msm_frame *frame,
	struct msm_device	*msm)
{
	unsigned long flags;
	long rc = 0;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vfe_phy_info *pphy;

	spin_lock_irqsave(&msm->sync.prev_frame_q_lock, flags);

	if (!list_empty(&msm->sync.prev_frame_q)) {
		qcmd = list_first_entry(&msm->sync.prev_frame_q,
			struct msm_queue_cmd, list);
		list_del(&qcmd->list);
	}

	spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock, flags);

	if (!qcmd)
		return -EAGAIN;

	pphy = (struct msm_vfe_phy_info *)(qcmd->command);

	frame->buffer =
		msm_pmem_frame_ptov_lookup(pphy->y_phy,
			pphy->cbcr_phy, &(frame->y_off),
			&(frame->cbcr_off), &(frame->fd), msm);

	CDBG("get_fr_proc: y= 0x%x, cbcr= 0x%x, qcmd= 0x%x, virt_addr= 0x%x\n",
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);

	kfree(qcmd->command);
	kfree(qcmd);
	return rc;
}

static long msm_get_frame(void __user *arg,
	struct msm_device	*msm)
{
	long rc = 0;
	struct msm_frame frame;

	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame))) {
		CDBG_ERR("msm_get_frame:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&frame,__LINE__);
		return -EFAULT;
	}
	rc = msm_get_frame_proc(&frame, msm);
	if (rc < 0)
		return rc;
	if (msm->croplen) {
		if (frame.croplen != msm->croplen) {
		  CDBG("msm_get_frame: invalid frame croplen %d,"
                               "expecting %d\n",
                               frame.croplen,
                               msm->croplen);
			return -EINVAL;
		}

		if (copy_to_user((void *)frame.cropinfo,
				msm->cropinfo,
				msm->croplen)) {
			CDBG_ERR("msm_get_frame:copy 0x%p -> 0x%p fail LINE:%d \n",
				msm->cropinfo,frame.cropinfo,__LINE__);
			return -EFAULT;
	  }
	}


	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame))) {
		CDBG_ERR("msm_get_frame:copy 0x%p -> 0x%p fail LINE:%d\n",
			&frame,arg,__LINE__);
		rc = -EFAULT;
	}

	CDBG("Got frame!!!\n");

	return rc;
}

static long msm_enable_vfe(void __user *arg,
	struct msm_device *msm)
{
	long rc = 0;
	struct camera_enable_cmd *cfg;
	struct camera_enable_cmd cfg_t;

	if (copy_from_user(
				&cfg_t,
				arg,
				sizeof(struct camera_enable_cmd))){
		CDBG_ERR("msm_enable_vfe:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&cfg_t,__LINE__);
		return -EFAULT;
	}
	cfg = kmalloc(sizeof(struct camera_enable_cmd),
					GFP_KERNEL);
	if (!cfg){
		CDBG_ERR("msm_enable_vfe:cannot allocate buffer cfg\n");
		return -ENOMEM;
	}
	cfg->name = kmalloc(cfg_t.length, GFP_KERNEL);

	if (!(cfg->name)) {
		CDBG_ERR("msm_enable_vfe:cannot allocate buffer cfg->name\n");
		kfree(cfg);
		return -ENOMEM;
	}

	if (copy_from_user(cfg->name, (void *)cfg_t.name, cfg_t.length)){
		CDBG_ERR("msm_enable_vfe:copy 0x%p -> 0x%p fail LINE:%d\n",
			cfg_t.name,cfg->name,__LINE__);
		return -EFAULT;
	}
	if (msm->vfefn.vfe_enable)
		rc = msm->vfefn.vfe_enable(cfg);

	CDBG("msm_enable_vfe:returned rc = %ld\n", rc);

	 /*CC091023*/
	kfree(cfg->name);
	kfree(cfg);
	return rc;
}

static int msm_ctrl_stats_pending(struct msm_device *msm)
{
	unsigned long flags;
	int yes = 0;

	spin_lock_irqsave(&msm->sync.ctrl_status_lock,
		flags);

	yes = !list_empty(&msm->sync.ctrl_status_queue);

	spin_unlock_irqrestore(&msm->sync.ctrl_status_lock,
		flags);

	CDBG("msm_ctrl_stats_pending, yes = %d\n", yes);
	return yes;
}

static long msm_control(void __user *arg,
	struct msm_device *msm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;

	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_queue_cmd *qcmd = NULL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		CDBG_ERR("msm_control:copy 0x%p -> 0x%p fail\n",
			arg,&ctrlcmd_t);
		rc = -EFAULT;
		goto end;
	}
	
	ctrlcmd = kmalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		CDBG_ERR("msm_control: cannot allocate buffer ctrlcmd\n");
		rc = -ENOMEM;
		goto no_mem;//becker change end to no_mem 
	}

	ctrlcmd->value = kmalloc(ctrlcmd_t.length, GFP_ATOMIC);
	if (!ctrlcmd->value) {
		CDBG_ERR("msm_control: cannot allocate buffer ctrlcmd->value\n");
		rc = -ENOMEM;
		goto fail; //becker change no_mem to fail
	}

	if (copy_from_user(ctrlcmd->value,
				ctrlcmd_t.value,
				ctrlcmd_t.length)) {
		CDBG_ERR("msm_control:copy 0x%p -> 0x%p fail LINE:%d\n",
			ctrlcmd_t.value,ctrlcmd->value,__LINE__);
		rc = -EFAULT;
		goto fail;
	}

	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->length = ctrlcmd_t.length;

	/* wake up config thread */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd), GFP_ATOMIC);
	if (!qcmd) {
		CDBG_ERR("msm_control: cannot allocate buffer qcmd\n");
		rc = -ENOMEM;
		goto fail; //from end chang to fail(6360 version)
	}

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock, flags);
	qcmd->type = MSM_CAM_Q_CTRL;
	qcmd->command = ctrlcmd;
	list_add_tail(&qcmd->list, &msm->sync.msg_event_queue);
	/* wake up config thread */
	wake_up(&msm->sync.msg_event_wait);
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

	/* wait for config status */
	timeout = (int)ctrlcmd_t.timeout_ms;
	CDBG("msm_control, timeout = %d\n", timeout);
	if (timeout > 0) {
		rc = wait_event_timeout(msm->sync.ctrl_status_wait,
					msm_ctrl_stats_pending(msm),
					msecs_to_jiffies(timeout));

		CDBG("msm_control: rc = %ld\n", rc);

		if (rc == 0) {
			CDBG_WARING("msm_control: timed out\n");
			rc = -ETIMEDOUT;
			goto fail;
		}
	} else
		rc = wait_event_interruptible(msm->sync.ctrl_status_wait,
			msm_ctrl_stats_pending(msm));

	if (rc < 0) {
		CDBG_WARING("msm_control: waiting for event interrupt\n");
		rc = -EAGAIN;
		goto fail;
	}

	/* control command status is ready */
	spin_lock_irqsave(&msm->sync.ctrl_status_lock, flags);
	if (!list_empty(&msm->sync.ctrl_status_queue)) {
		qcmd = list_first_entry(&msm->sync.ctrl_status_queue,
				struct msm_queue_cmd, list);

		if (!qcmd) {
			spin_unlock_irqrestore(&msm->sync.ctrl_status_lock,
				flags);
			CDBG_ERR("msm_control:get message form msm_sync.ctrl_status_queue fail\n");
			rc = -EAGAIN;
			goto fail;
		}
		
		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.ctrl_status_lock, flags);

	if (!qcmd->command) {
		ctrlcmd_t.type = 0xFFFF;
		ctrlcmd_t.length = 0xFFFF;
		ctrlcmd_t.status = 0xFFFF;
	} else {

		CDBG("msm_control: length = %d\n",
			((struct msm_ctrl_cmd *)(qcmd->command))->length);
		ctrlcmd_t.type =
			((struct msm_ctrl_cmd *)(qcmd->command))->type;

		ctrlcmd_t.length =
			((struct msm_ctrl_cmd *)(qcmd->command))->length;

		ctrlcmd_t.status =
			((struct msm_ctrl_cmd *)(qcmd->command))->status;

		if (ctrlcmd_t.length > 0) {
			if (copy_to_user(ctrlcmd_t.value,
			((struct msm_ctrl_cmd *)(qcmd->command))->value,
			((struct msm_ctrl_cmd *)(qcmd->command))->length)) {
			CDBG_ERR("msm_control:copy 0x%p -> 0x%p fail LINE:%d\n",
				((struct msm_ctrl_cmd *)(qcmd->command))->value,ctrlcmd_t.value,
				__LINE__);
			CDBG_ERR("copy_to_user value failed!\n");
			rc = -EFAULT;
			goto end;
		}

			kfree(((struct msm_ctrl_cmd *)
				(qcmd->command))->value);
	}

		if (copy_to_user((void *)arg,
				&ctrlcmd_t,
				sizeof(struct msm_ctrl_cmd))) {
			CDBG_ERR("msm_control:copy 0x%p -> 0x%p fail LINE:%d\n",
				&ctrlcmd_t,arg,__LINE__);
			CDBG_ERR("copy_to_user ctrlcmd failed!\n");
			rc = -EFAULT;
			goto end;
		}
	}

	goto end;

fail:
	kfree(ctrlcmd->value);

no_mem:
	kfree(ctrlcmd);
	return rc; //becker protect kfree twice ctrlcmd = qcmd->command to avoid reboot

end:
	if (qcmd) {
		kfree(qcmd->command);
		kfree(qcmd);
	}

	CDBG("msm_control: end rc = %ld\n", rc);
	return rc;
}

static int msm_stats_pending(struct msm_device *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock,
		flags);

	if (!list_empty(&msm->sync.msg_event_queue)) {

		qcmd = list_first_entry(&msm->sync.msg_event_queue,
			struct msm_queue_cmd, list);

		if (qcmd) {

			if ((qcmd->type  == MSM_CAM_Q_CTRL)    ||
					(qcmd->type  == MSM_CAM_Q_VFE_EVT) ||
					(qcmd->type  == MSM_CAM_Q_VFE_MSG) ||
					(qcmd->type  == MSM_CAM_Q_V4L2_REQ)) {
				yes = 1;
			}
		}
	}
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

	CDBG("msm_stats_pending, tyes = %d\n", yes);
	return yes;
}

static long msm_get_stats(void __user *arg,
	struct msm_device *msm)
{
	unsigned long flags;
	int           timeout;
	long          rc = 0;

	struct msm_stats_event_ctrl se;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_ctrl_cmd  *ctrl = NULL;
	struct msm_vfe_resp  *data = NULL;
	struct msm_stats_buf stats;

	if (copy_from_user(&se, arg,
				sizeof(struct msm_stats_event_ctrl))){
		CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&se,__LINE__);
		return -EFAULT;
	}
	timeout = (int)se.timeout_ms;

	if (timeout > 0) {
		rc =
			wait_event_timeout(
				msm->sync.msg_event_wait,
				msm_stats_pending(msm),
				msecs_to_jiffies(timeout));

		if (rc == 0) {
			CDBG("msm_get_stats, timeout\n");
			return -ETIMEDOUT;
		}
	} else {
		rc = wait_event_interruptible(msm->sync.msg_event_wait,
			msm_stats_pending(msm));
	}

	if (rc < 0) {
		CDBG_WARING("msm_get_stats, rc = %ld\n", rc);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock, flags);
	if (!list_empty(&msm->sync.msg_event_queue)) {
		qcmd = list_first_entry(&msm->sync.msg_event_queue,
				struct msm_queue_cmd, list);

		if (!qcmd) {
			spin_unlock_irqrestore(
				&msm->sync.msg_event_queue_lock, flags);
			return -EAGAIN;
		}

		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock, flags);

	CDBG("=== received from DSP === %d\n", qcmd->type);

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		data = (struct msm_vfe_resp *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", se.stats_event.len);
		CDBG("msg_id = %d\n", se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(data->phy.sbuf_phy,
				&(stats.fd), msm);

			if (copy_to_user((void *)(se.stats_event.data),
				&stats, sizeof(struct msm_stats_buf))) {
				CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
					&stats,se.stats_event.data,__LINE__);
				rc = -EFAULT;
				goto failure;
			}
		} else if ((data->evt_msg.len > 0) &&
				(data->type == VFE_MSG_GENERAL)) {

			if (copy_to_user((void *)(se.stats_event.data),
					data->evt_msg.data,
					data->evt_msg.len)){
				CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
					data->evt_msg.data,se.stats_event.data,__LINE__);
				rc = -EFAULT;
			}
		} else if (data->type == VFE_MSG_OUTPUT1 ||
			data->type == VFE_MSG_OUTPUT2) {

			uint32_t pp_en;
			struct msm_postproc buf;
			mutex_lock(&msm->pict_pp_lock);
			pp_en = msm->pict_pp;
			mutex_unlock(&msm->pict_pp_lock);
			if (pp_en & PP_PREV) {
				CDBG
				("Started Preview post processing. pp_en=%d\n",
				pp_en);
				buf.fmain.buffer =
					msm_pmem_frame_ptov_lookup(
					data->phy.y_phy,
					data->phy.cbcr_phy, &buf.fmain.y_off,
					&buf.fmain.cbcr_off, &buf.fmain.fd,
					msm);
				if (buf.fmain.buffer) {
					if (copy_to_user(
						(void *)(se.stats_event.data),
						&(buf.fmain),
						sizeof(struct msm_frame)))
							rc = -EFAULT;
					} else
							rc = -EFAULT;
			} else {
				if (copy_to_user(
					(void *)(se.stats_event.data),
					data->extdata,
					data->extlen))
						rc = -EFAULT;
			}
		} else if (data->type == VFE_MSG_SNAPSHOT) {

			uint32_t pp_en;
			struct msm_postproc buf;
			struct msm_pmem_region region;
			spin_lock_irqsave(&(msm->sync.get_pp_q_lock), flags);
			pp_en = msm->pict_pp;
			spin_unlock_irqrestore(&(msm->sync.get_pp_q_lock), flags);


			if (pp_en & PP_SNAP) {
				buf.fmnum =
					msm_pmem_region_lookup(
						&msm->sync.frame,
						MSM_PMEM_MAINIMG,
						&region, 1);

				if (buf.fmnum == 1) {
					buf.fmain.buffer =
					(unsigned long)region.vaddr;

					buf.fmain.y_off  = region.y_off;
					buf.fmain.cbcr_off = region.cbcr_off;
					buf.fmain.fd = region.fd;
				} else {
					buf.fmnum =
						msm_pmem_region_lookup(
						&msm->sync.frame,
						MSM_PMEM_RAW_MAINIMG,
						&region, 1);

					if (buf.fmnum == 1) {
						buf.fmain.path =
							MSM_FRAME_PREV_2;
						buf.fmain.buffer =
						(unsigned long)region.vaddr;

						buf.fmain.fd = region.fd;
					}
				}

				if (copy_to_user((void *)(se.stats_event.data),
					&buf, sizeof(struct msm_postproc))) {
        				CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
						&buf,se.stats_event.data,__LINE__);
					rc = -EFAULT;
					goto failure;
				}
			}

			CDBG("SNAPSHOT copy_to_user!\n");
		}
		break;

	case MSM_CAM_Q_CTRL:{
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
						ctrl->value,
						ctrl->length)) {
				CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
						ctrl->value,se.ctrl_cmd.value,__LINE__);
				rc = -EFAULT;
				goto failure;
			}
		}

		se.resptype = MSM_CAM_RESP_CTRL;

		/* what to control */
		se.ctrl_cmd.type = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
	} /* MSM_CAM_Q_CTRL */
		break;

	case MSM_CAM_Q_V4L2_REQ: {
		/* control command from v4l2 client */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("msm_get_stats, qcmd->type = %d\n", qcmd->type);
		CDBG("length = %d\n", ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
					ctrl->value, ctrl->length)) {
				CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
						ctrl->value,se.ctrl_cmd.value,__LINE__);
				rc = -EFAULT;
				goto failure;
			}
		}

		/* 2 tells config thread this is v4l2 request */
		se.resptype = MSM_CAM_RESP_V4L2;

		/* what to control */
		se.ctrl_cmd.type   = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
	} /* MSM_CAM_Q_V4L2_REQ */
		break;

	default:
		rc = -EFAULT;
		goto failure;
	} /* switch qcmd->type */

	if (copy_to_user((void *)arg, &se, sizeof(se))){
		CDBG_ERR("msm_get_stats:copy 0x%p -> 0x%p fail LINE:%d\n",
						&se,arg,__LINE__);
		rc = -EFAULT;
	}
failure:
	if (qcmd) {

		if (qcmd->type == MSM_CAM_Q_VFE_MSG)
			kfree(((struct msm_vfe_resp *)
				(qcmd->command))->evt_msg.data);

		/*CC091023*/
		if (qcmd->type == MSM_CAM_Q_CTRL)
			kfree(((struct msm_ctrl_cmd *)
			(qcmd->command))->value);
		/*CC091023~*/

		kfree(qcmd->command);
		kfree(qcmd);
	}

	CDBG("msm_get_stats: end rc = %ld\n", rc);
	return rc;
}

static long msm_ctrl_cmd_done(void __user *arg,
	struct msm_device *msm)
{
	unsigned long flags;
	long rc = 0;

	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_queue_cmd *qcmd = NULL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))){
		CDBG_ERR("msm_ctrl_cmd_done:copy 0x%p -> 0x%p fail LINE:%d\n",
				arg,&ctrlcmd_t,__LINE__);
		return -EFAULT;
	}
	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		CDBG_ERR("msm_ctrl_cmd_done: kmalloc faile ctrlcmd\n");
		rc = -ENOMEM;
		goto end;
	}

	if (ctrlcmd_t.length > 0) {
		ctrlcmd->value = kmalloc(ctrlcmd_t.length, GFP_ATOMIC);
		if (!ctrlcmd->value) {
			CDBG_ERR("msm_ctrl_cmd_done: kmalloc faile ctrlcmd->value\n");
			rc = -ENOMEM;
			goto no_mem;
		}

		if (copy_from_user(ctrlcmd->value,
					(void *)ctrlcmd_t.value,
					ctrlcmd_t.length)) {
			CDBG_ERR("msm_ctrl_cmd_done:copy 0x%p -> 0x%p fail LINE:%d\n",
				ctrlcmd_t.value,ctrlcmd->value,__LINE__);
			rc = -EFAULT;
			goto fail;
		}
	} else
		ctrlcmd->value = NULL;

	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->length = ctrlcmd_t.length;
	ctrlcmd->status = ctrlcmd_t.status;

	qcmd = kmalloc(sizeof(*qcmd), GFP_ATOMIC);
	if (!qcmd) {
		CDBG_ERR("msm_ctrl_cmd_done: kmalloc faile qcmd\n");
		rc = -ENOMEM;
		goto fail;
	}

	qcmd->command = (void *)ctrlcmd;

	goto end;

fail:
	kfree(ctrlcmd->value);
no_mem:
	kfree(ctrlcmd);
end:
	CDBG("msm_ctrl_cmd_done: end rc = %ld\n", rc);
	if (rc == 0) {
		/* wake up control thread */
		spin_lock_irqsave(&msm->sync.ctrl_status_lock, flags);
		list_add_tail(&qcmd->list, &msm->sync.ctrl_status_queue);
		spin_unlock_irqrestore(&msm->sync.ctrl_status_lock, flags);
		wake_up(&msm->sync.ctrl_status_wait);
	}
	return rc;
}

static long msm_config_vfe(void __user *arg,
	struct msm_device	*msm)
{
	struct msm_vfe_cfg_cmd cfgcmd_t;
	struct msm_pmem_region region[8];
	struct axidata axi_data;
	long rc = 0;

	memset(&axi_data, 0, sizeof(axi_data));

	if (copy_from_user(&cfgcmd_t, arg, sizeof(cfgcmd_t))){
		CDBG_ERR("msm_config_vfe:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&cfgcmd_t,__LINE__);
		return -EFAULT;
	}
  	if (cfgcmd_t.cmd_type == CMD_STATS_ENABLE) {

		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.stats,
		MSM_PMEM_AEC_AWB, &region[0],
		NUM_WB_EXP_STAT_OUTPUT_BUFFERS);
		axi_data.region = &region[0];

	} else if (cfgcmd_t.cmd_type == CMD_STATS_AF_ENABLE) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.stats,
			MSM_PMEM_AF, &region[0],
			NUM_AF_STAT_OUTPUT_BUFFERS);
		axi_data.region = &region[0];
	}

	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(&cfgcmd_t, &(axi_data));

	return rc;
}

static long msm_frame_axi_cfg(struct msm_vfe_cfg_cmd *cfgcmd_t,
	struct msm_device *msm)
{
	long rc = 0;
	struct axidata axi_data;
	struct msm_pmem_region region[8];
	enum msm_pmem mtype;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd_t->cmd_type) {
	case CMD_AXI_CFG_OUT1:
		mtype = MSM_PMEM_OUTPUT1;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;

	case CMD_AXI_CFG_OUT2:
		mtype = MSM_PMEM_OUTPUT2;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;

	case CMD_AXI_CFG_SNAP_O1_AND_O2:
		mtype = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);

		mtype = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[axi_data.bufnum1], 8);
		break;

	case CMD_RAW_PICT_AXI_CFG:
		mtype = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&msm->sync.frame, mtype,
				&region[0], 8);
		break;
	default:
		break;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(cfgcmd_t, &axi_data);

	return rc;
}

static long msm_get_sensor_info(void __user *arg,
	struct msm_device *msm)
{
	long rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_platform_data *pdata;

	if (copy_from_user(&info,
				arg,
				sizeof(struct msm_camsensor_info))){
		CDBG_ERR("msm_get_sensor_info:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&info,__LINE__);
		return -EFAULT;
	}
	pdata = msm->pdev->dev.platform_data;
	CDBG("sensor_name %s\n", pdata->sinfo[msm->sidx].sensor_name);

	memcpy(&info.name[0],
		pdata->sinfo[msm->sidx].sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled =
		(pdata->sinfo[msm->sidx].flash_type != MSM_CAMERA_FLASH_NONE);

	/* copy back to user space */
	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info))) {
		
		CDBG_ERR("msm_get_sensor_info:copy 0x%p > 0x%p fail LINE:%d\n",
			&info,arg,__LINE__);
		rc = -EFAULT;
	}

	return rc;
}

static long msm_put_frame_buf_proc(struct msm_frame *pb,
	struct msm_device *msm)
{
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd_t;

	long rc = 0;

	msm_pmem_frame_vtop_lookup(pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd, &pphy, msm);

	CDBG("rel: vaddr = 0x%lx, paddr = 0x%lx\n",
		pb->buffer, pphy);

	if (pphy != 0) {

		cfgcmd_t.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd_t.value    = (void *)pb;

		if (msm->vfefn.vfe_config)
			rc =
				msm->vfefn.vfe_config(&cfgcmd_t, &pphy);
	} else {
		CDBG_ERR("msm_put_frame_buf_proc:pphy address is NULL\n");
		rc = -EFAULT;
	}
	return rc;
}

static long msm_put_frame_buffer(void __user *arg,
	struct msm_device *msm)
{
	struct msm_frame buf_t;

	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame))){
		CDBG_ERR("msm_put_frame_buffer:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&buf_t,__LINE__);
		return -EFAULT;
	}

	return msm_put_frame_buf_proc(&buf_t, msm);
}

static long msm_register_pmem_proc(struct msm_pmem_info *pinfo,
	struct msm_device *msm)
{
	unsigned long paddr, len, rc = 0;
	struct file   *file;
	unsigned long vstart;

	mutex_lock(&msm->msm_sem);

	CDBG("Here1 ==> reg: type = %d, paddr = 0x%lx, vaddr = 0x%lx\n",
		pinfo->type, paddr, (unsigned long)pinfo->vaddr);

	get_pmem_file(pinfo->fd, &paddr, &vstart, &len, &file);
	CDBG("Here2 ==> reg: type = %d, paddr = 0x%lx, vaddr = 0x%lx, len = %ld\n",
		pinfo->type, paddr, (unsigned long)pinfo->vaddr, len);
	switch (pinfo->type) {
	case MSM_PMEM_OUTPUT1:
	case MSM_PMEM_OUTPUT2:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
		rc = msm_pmem_table_add(&msm->sync.frame,
			pinfo, file, paddr, len, pinfo->fd);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		rc = msm_pmem_table_add(&msm->sync.stats,
			pinfo, file, paddr, len, pinfo->fd);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&msm->msm_sem);
	return rc;
}

static long msm_register_pmem(void __user *arg,
	struct msm_device *msm)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))){
		CDBG_ERR("msm_register_pmem:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&info,__LINE__);
		return -EFAULT;
	}	

	return msm_register_pmem_proc(&info, msm);
}

static long msm_stats_axi_cfg(struct msm_vfe_cfg_cmd *cfgcmd_t,
	struct msm_device *msm)
{
	long rc = 0;
	struct axidata axi_data;

	struct msm_pmem_region region[3];
	enum msm_pmem mtype = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

	if (cfgcmd_t->cmd_type == CMD_STATS_AXI_CFG)
		mtype = MSM_PMEM_AEC_AWB;
	else if (cfgcmd_t->cmd_type == CMD_STATS_AF_AXI_CFG)
		mtype = MSM_PMEM_AF;

	axi_data.bufnum1 =
		msm_pmem_region_lookup(&msm->sync.stats, mtype,
			&region[0], NUM_WB_EXP_STAT_OUTPUT_BUFFERS);

	axi_data.region = &region[0];

	/* send the AEC/AWB STATS configuration command to driver */
	if (msm->vfefn.vfe_config)
		rc = msm->vfefn.vfe_config(cfgcmd_t, &axi_data);

	return rc;
}

static long msm_put_stats_buffer(void __user *arg,
	struct msm_device *msm)
{
	long rc = 0;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd_t;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))){
		CDBG_ERR("msm put stats buffer:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&buf,__LINE__);
		return -EFAULT;
	}
	CDBG("msm_put_stats_buffer\n");
	msm_pmem_stats_vtop_lookup(buf.buffer,buf.fd, &pphy, msm);

	if (pphy != 0) {

		if (buf.type == STAT_AEAW)
			cfgcmd_t.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd_t.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else {
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd_t.value    = (void *)&buf;

		if (msm->vfefn.vfe_config)
			rc = msm->vfefn.vfe_config(&cfgcmd_t, &pphy);
	} else {
		CDBG_ERR("msm_put_stats_buffer:pphy address is NULL\n");
		rc = -EFAULT;
	}
put_done:
	return rc;
}

static long msm_axi_config(void __user *arg,
	struct msm_device *msm)
{
	long rc = 0;
	struct msm_vfe_cfg_cmd cfgcmd_t;

	if (copy_from_user(&cfgcmd_t, arg, sizeof(cfgcmd_t))){
		CDBG_ERR("msm_axi_config:copy 0x%p -> 0x%p fail LINE:%d\n",
			arg,&cfgcmd_t,__LINE__);
		return -EFAULT;
	}
	switch (cfgcmd_t.cmd_type) {
	case CMD_AXI_CFG_OUT1:
	case CMD_AXI_CFG_OUT2:
	case CMD_AXI_CFG_SNAP_O1_AND_O2:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(&cfgcmd_t, msm);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(&cfgcmd_t, msm);

	default:
	{
		CDBG_ERR("msm_axi_config:No such command type cfgcmd_t.cmd_type:%d\n",
			cfgcmd_t.cmd_type);
		rc = -EFAULT;
	}
	}

	return rc;
}

static int msm_camera_pict_pending(struct msm_device *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd *qcmd = NULL;
	CDBG("msm_camera_pict_pending\n");
	spin_lock_irqsave(&msm->sync.pict_frame_q_lock,
		flags);

	if (!list_empty(&msm->sync.pict_frame_q)) {

		qcmd =
			list_first_entry(&msm->sync.pict_frame_q,
				struct msm_queue_cmd, list);

		if (qcmd) {
			if (qcmd->type  == MSM_CAM_Q_VFE_MSG)
				yes = 1;
		}
	}
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);

	CDBG("msm_camera_pict_pending, yes = %d\n", yes);
	return yes;
}

static long msm_get_pict_proc(struct msm_ctrl_cmd *ctrl,
	struct msm_device *msm)
{
	unsigned long flags;
	long rc = 0;
	int tm;

	struct msm_queue_cmd *qcmd = NULL;

	tm = (int)ctrl->timeout_ms;

	if (tm > 0) {
		rc =
			wait_event_timeout(
				msm->sync.pict_frame_wait,
				msm_camera_pict_pending(msm),
				msecs_to_jiffies(tm));

		if (rc == 0) {
			CDBG_WARING("msm_get_pict_proc timeout\n");
			return -ETIMEDOUT;
		}
	} else{
		rc = wait_event_interruptible(
					msm->sync.pict_frame_wait,
					msm_camera_pict_pending(msm));
	}
	
	if (rc < 0) {
		CDBG_WARING("msm get pict proc:wait interrupt fail, rc = %ld\n", rc);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&msm->sync.pict_frame_q_lock, flags);
	if (!list_empty(&msm->sync.pict_frame_q)) {
		qcmd = list_first_entry(&msm->sync.pict_frame_q,
			struct msm_queue_cmd, list);
		list_del(&qcmd->list);
	}
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);

	if (qcmd->command != NULL) {
		ctrl->type =
		((struct msm_ctrl_cmd *)(qcmd->command))->type;

		ctrl->status =
		((struct msm_ctrl_cmd *)(qcmd->command))->status;

		kfree(qcmd->command);
	} else {
		ctrl->type = 0xFFFF;
		ctrl->status = 0xFFFF;
	}

	kfree(qcmd);

	return rc;
}

static long msm_get_pic(void __user *arg,
	struct msm_device *msm)
{
	struct msm_ctrl_cmd ctrlcmd_t;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))){
		CDBG_ERR("msm_get_pic:copy 0x%p ->  0x%p fail LINE:%d\n",
				arg,&ctrlcmd_t,__LINE__);
		return -EFAULT;
	}	
// andy add
	if (msm_get_pict_proc(&ctrlcmd_t, msm) < 0)
		return -EFAULT;
			
	CDBG("msm_camera->croplen:%d\n",msm->croplen);
	CDBG("msm_camera->cropinfo:0x%p\n",msm->cropinfo);
	if (msm->croplen) {
	if (ctrlcmd_t.length != msm->croplen) {
			CDBG_ERR("msm_get_pic:crop info wrong \n");
			CDBG_ERR("msm_get_pic:length:%d \n",ctrlcmd_t.length);
			CDBG_ERR("msm_get_pic:msm_camera->croplen:%d \n",
				msm->croplen);
			CDBG_ERR("msm_get_pic:value:0x%p \n",ctrlcmd_t.value);
			CDBG_ERR("msm_get_pic:crop_info:0x%p \n",msm->cropinfo );

			return -EINVAL;
		}
		if (copy_to_user(ctrlcmd_t.value,
				msm->cropinfo,
				msm->croplen)){
			CDBG_ERR("msm_get_pic:copy 0x%p ->  0x%p fail LINE:%d\n",
				msm->cropinfo,ctrlcmd_t.value,__LINE__);
			return -EINVAL;
		}
	}

	if (copy_to_user((void *)arg,
			&ctrlcmd_t,
			sizeof(struct msm_ctrl_cmd))){
		CDBG_ERR("msm_get_pic:copy 0x%p ->  0x%p fail LINE:%d\n",
			&ctrlcmd_t,arg,__LINE__);
		return -EFAULT;
	}
	CDBG("copy frame to user done\n");
	return 0;	
}

static long msm_set_crop(void __user *arg,
	struct msm_device *msm)
{
	struct crop_info crop;
	CDBG_INFO("set crop in kernel\n");
	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))){
		CDBG_ERR("msm_set_crop:copy 0x%p ->0x%p fail LINE:%d\n",
			arg,&crop,__LINE__);
		return -EFAULT;
	}
	if (!msm->croplen) {
		msm->cropinfo = kmalloc(crop.len, GFP_KERNEL);

		if (!msm->cropinfo){
			CDBG_ERR("msm_set_crop:kmallo fail 0x%p",msm->cropinfo);
			return -ENOMEM;
		}	
	} else if (msm->croplen < crop.len){
		CDBG_ERR("msm_set_crop: crop len ");
		return -EINVAL;
	}
	if (copy_from_user(msm->cropinfo,
				crop.info,
				crop.len)) {
		CDBG_ERR("msm_set_crop : copy 0x%p -> 0x%p fail LINE:%d\n",
			crop.info,msm->cropinfo,__LINE__);
		kfree(msm->cropinfo);
		return -EFAULT;
	}

	msm->croplen = crop.len;
	CDBG("msm_camera->cropinfo:0x%p\n",msm->cropinfo);
	return 0;

}

static long msm_pict_pp_done(void __user *arg,
	struct msm_device *msm)
{
	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_ctrl_cmd *ctrlcmd = NULL;
	struct msm_queue_cmd *qcmd = NULL;
	unsigned long flags;
	long rc = 0;

	uint32_t pp_en;
	spin_lock_irqsave(&(msm->sync.get_pp_q_lock), flags);
	pp_en = msm->pict_pp;
	spin_unlock_irqrestore(&(msm->sync.get_pp_q_lock), flags);

	if (!pp_en)
		return -EINVAL;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		CDBG_ERR("msm pict pp done: copy 0x%p ->0x%p fail LINE:%d\n",
			&ctrlcmd_t,arg,__LINE__);
		rc = -EFAULT;
		goto pp_done;
	}

	qcmd = kmalloc(sizeof(struct msm_queue_cmd),
						GFP_ATOMIC);
	if (!qcmd) {
		CDBG_ERR("msm pict pp done: qcmd kmalloc faile 0x%p\n",qcmd);
		rc = -ENOMEM;
		goto pp_fail;
	}

	ctrlcmd = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!ctrlcmd) {
		CDBG_ERR("msm pict pp done: ctrlcmd kmalloc faile 0x%p\n",ctrlcmd);
		rc = -ENOMEM;
		goto pp_done;
	}

	ctrlcmd->type = ctrlcmd_t.type;
	ctrlcmd->status = ctrlcmd_t.status; 

pp_done:
	qcmd->type = MSM_CAM_Q_VFE_MSG; 
	qcmd->command = ctrlcmd;

	spin_lock_irqsave(&msm->sync.pict_frame_q_lock, flags);
	list_add_tail(&qcmd->list, &msm->sync.pict_frame_q);
	spin_unlock_irqrestore(&msm->sync.pict_frame_q_lock, flags);
	wake_up(&msm->sync.pict_frame_wait);

pp_fail:
	return rc;
}

static int msm_af_status_pending(struct msm_device *msm)
{
	int rc;
	unsigned long flags;
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	rc = msm->sync.af_flag;
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	return rc;
}
static long msm_af_control(void __user *arg,
	struct msm_device *msm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;
	if (copy_from_user(&msm->sync.af_status,
			arg, sizeof(struct msm_ctrl_cmd))) {
		rc = -EFAULT;
		goto end;
	}
	timeout = (int)msm->sync.af_status.timeout_ms;
	if (timeout > 0) {
		rc = wait_event_timeout(msm->sync.af_status_wait,
			msm_af_status_pending(msm),
			msecs_to_jiffies(timeout));
		CDBG("msm_af_control: rc = %ld\n", rc);
		if (rc == 0) {
			CDBG("msm_af_control: timed out\n");
			rc = -ETIMEDOUT;
			goto end;
		}
	} else
		rc = wait_event_interruptible(msm->sync.af_status_wait,
			msm_af_status_pending(msm));
	if (rc < 0) {
		rc = -EAGAIN;
		goto end;
	}
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	if (msm->sync.af_flag < 0) {
		msm->sync.af_status.type = 0xFFFF;
		msm->sync.af_status.status = 0xFFFF;
	}
	msm->sync.af_flag = 0;
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	if (copy_to_user((void *)arg,
			&msm->sync.af_status,
			sizeof(struct msm_ctrl_cmd))) {
		CDBG("copy_to_user ctrlcmd failed!\n");
		rc = -EFAULT;
	}

end:
	CDBG("msm_control: end rc = %ld\n", rc);
	return rc;
}
static long msm_af_control_done(void __user *arg,
	struct msm_device *msm)
{
	unsigned long flags;
	long rc = 0;
	rc = copy_from_user(&msm->sync.af_status,
		arg, sizeof(struct msm_ctrl_cmd));
	spin_lock_irqsave(&msm->sync.af_status_lock, flags);
	msm->sync.af_flag = (rc == 0 ? 1 : -1);
	spin_unlock_irqrestore(&msm->sync.af_status_lock, flags);
	wake_up(&msm->sync.af_status_wait);
	return rc;
}
static long msm_ioctl(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct msm_device *pmsm = filep->private_data;
	unsigned long flags;

	CDBG("!!! msm_ioctl !!!, cmd = %d\n", _IOC_NR(cmd));
	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		return msm_get_sensor_info(argp, pmsm);

	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(argp, pmsm);

	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(argp, pmsm);

	case MSM_CAM_IOCTL_CTRL_COMMAND:
		/* Coming from control thread, may need to wait for
		* command status */
		return msm_control(argp, pmsm);

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		return msm_config_vfe(argp, pmsm);

	case MSM_CAM_IOCTL_GET_STATS:
		/* Coming from config thread wait
		* for vfe statistics and control requests */
		return msm_get_stats(argp, pmsm);

	case MSM_CAM_IOCTL_GETFRAME:
		/* Coming from frame thread to get frame
		* after SELECT is done */
		return msm_get_frame(argp, pmsm);

	case MSM_CAM_IOCTL_ENABLE_VFE:
		/* This request comes from control thread:
		* enable either QCAMTASK or VFETASK */
		return msm_enable_vfe(argp, pmsm);

	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		/* Config thread notifies the result of contrl command */
		return msm_ctrl_cmd_done(argp, pmsm);

	case MSM_CAM_IOCTL_VFE_APPS_RESET:
		msm_camio_vfe_blk_reset();
		return 0;

	case MSM_CAM_IOCTL_RELEASE_FRAMEE_BUFFER:
		return msm_put_frame_buffer(argp, pmsm);

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		return msm_put_stats_buffer(argp, pmsm);

	case MSM_CAM_IOCTL_AXI_CONFIG:
		return msm_axi_config(argp, pmsm);

	case MSM_CAM_IOCTL_GET_PICTURE:
		return msm_get_pic(argp, pmsm);

	case MSM_CAM_IOCTL_SET_CROP:
		return msm_set_crop(argp, pmsm);

	case MSM_CAM_IOCTL_PICT_PP: {
		uint32_t pp;
		if (copy_from_user(&pp, argp, sizeof(pp)))
			return -EFAULT;
		spin_lock_irqsave(&(pmsm->sync.get_pp_q_lock), flags);
		pmsm->pict_pp = pp;
		spin_unlock_irqrestore(&(pmsm->sync.get_pp_q_lock), flags);
		return 0;
	}

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		return msm_pict_pp_done(argp, pmsm);

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		return pmsm->sctrl.s_config(argp);

	case MSM_CAM_IOCTL_AF_CTRL:
		return msm_af_control(argp, pmsm);

	case MSM_CAM_IOCTL_AF_CTRL_DONE:
		return msm_af_control_done(argp, pmsm);

	default:
		break;
	}

	return -EINVAL;
}

static int msm_frame_pending(struct msm_device *msm)
{
	unsigned long flags;
	int yes = 0;

	struct msm_queue_cmd *qcmd = NULL;

	spin_lock_irqsave(&msm->sync.prev_frame_q_lock, flags);

	if (!list_empty(&msm->sync.prev_frame_q)) {

		qcmd = list_first_entry(&msm->sync.prev_frame_q,
			struct msm_queue_cmd, list);

		if (!qcmd)
			yes = 0;
		else {
			yes = 1;
			CDBG("msm_frame_pending: yes = %d\n",
				yes);
		}
	}

	spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock, flags);

	CDBG("msm_frame_pending, yes = %d\n", yes);
	return yes;
}
static int msm_release_proc(struct file *filep, struct msm_device *pmsm)
{
	struct msm_pmem_region *region;
	struct hlist_node *hnode;
	struct hlist_node *n;
	struct msm_queue_cmd *qcmd = NULL;
	unsigned long flags;

	mutex_lock(&pmsm->msm_lock);
	pmsm->opencnt -= 1;
	mutex_unlock(&pmsm->msm_lock);

	if (!pmsm->opencnt) {
		/* need to clean up
		 * system resource */
		if (pmsm->vfefn.vfe_release)
			pmsm->vfefn.vfe_release(pmsm->pdev);

		if (pmsm->croplen) {
			kfree(pmsm->cropinfo);
			pmsm->cropinfo = NULL;
			pmsm->croplen = 0;
		}

		hlist_for_each_entry_safe(region, hnode, n,
			&pmsm->sync.frame, list) {

			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		hlist_for_each_entry_safe(region, hnode, n,
			&pmsm->sync.stats, list) {

			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		while (msm_ctrl_stats_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.ctrl_status_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.ctrl_status_queue,
				struct msm_queue_cmd, list);
			spin_unlock_irqrestore(&pmsm->sync.ctrl_status_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				if (qcmd->type == MSM_CAM_Q_VFE_MSG)
					kfree(((struct msm_vfe_resp *)
						(qcmd->command))->evt_msg.data);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_stats_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.msg_event_queue_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.msg_event_queue,
				struct msm_queue_cmd, list);
			spin_unlock_irqrestore(&pmsm->sync.msg_event_queue_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_camera_pict_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.pict_frame_q_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.pict_frame_q,
				struct msm_queue_cmd, list);
			spin_unlock_irqrestore(&pmsm->sync.pict_frame_q_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		while (msm_frame_pending(pmsm)) {
			spin_lock_irqsave(&pmsm->sync.prev_frame_q_lock,
				flags);
			qcmd = list_first_entry(&pmsm->sync.prev_frame_q,
				struct msm_queue_cmd, list);
			spin_unlock_irqrestore(&pmsm->sync.prev_frame_q_lock,
				flags);

			if (qcmd) {
				list_del(&qcmd->list);
				kfree(qcmd->command);
				kfree(qcmd);
			}
		};

		pmsm->sctrl.s_release();

		CDBG("msm_release completed!\n");
	}
	if(is_perf_lock_active(&camera_perf_lock)){
		perf_unlock(&camera_perf_lock);
	}
	return 0;
}

static int msm_release(struct inode *node, struct file *filep)
{
	return msm_release_proc(filep, filep->private_data);
}

static ssize_t msm_read(struct file *filep, char __user *arg,
	size_t size, loff_t *loff)
{
	return 0;
}

static ssize_t msm_write(struct file *filep, const char __user *arg,
	size_t size, loff_t *loff)
{
	return 0;
}

unsigned int msm_poll(struct file *filep,
	struct poll_table_struct *pll_table)
{
	struct msm_device *pmsm = filep->private_data;
	struct msm_queue_cmd *qcmd = NULL;
	unsigned long flags;

	while (msm_camera_pict_pending(pmsm)) {
		spin_lock_irqsave(&pmsm->sync.pict_frame_q_lock,
			flags);
		qcmd = list_first_entry(&pmsm->sync.pict_frame_q,
			struct msm_queue_cmd, list);
		spin_unlock_irqrestore(&pmsm->sync.pict_frame_q_lock,
			flags);

		if (qcmd) {
			list_del(&qcmd->list);
			kfree(qcmd->command);
			kfree(qcmd);
		}
	};

	poll_wait(filep, &pmsm->sync.prev_frame_wait, pll_table);

	if (msm_frame_pending(pmsm))
		/* frame ready */
		return POLLIN | POLLRDNORM;

	return 0;
}

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
	 enum msm_queue qtype, void *syncdata)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_queue_cmd *qcmd_frame = NULL;
	struct msm_vfe_phy_info *fphy;
	uint32_t pp;

	unsigned long flags;
	struct msm_device *msm =
		(struct msm_device *)syncdata;

	if (!msm)
		return;
	qcmd = kmalloc(sizeof(struct msm_queue_cmd),
					GFP_ATOMIC);
	if (!qcmd) {
		CDBG_ERR("evt_msg: cannot allocate buffer\n");
		goto mem_fail1;
	}
	CDBG("msm_vfe_sync qtype:%d\n",qtype);
	if (qtype == MSM_CAM_Q_VFE_EVT) {
		CDBG("event from dsp\n");
		qcmd->type    = MSM_CAM_Q_VFE_EVT;
	} else if (qtype == MSM_CAM_Q_VFE_MSG) {

		qcmd->type = MSM_CAM_Q_VFE_MSG;

		if (vdata->type == VFE_MSG_OUTPUT1 ||
		    vdata->type == VFE_MSG_OUTPUT2) {

			mutex_lock(&msm->pict_pp_lock);
			pp = msm->pict_pp;
			mutex_unlock(&msm->pict_pp_lock);

			if (pp & PP_PREV)
				goto sync_done;

			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd),
					GFP_ATOMIC);
			if (!qcmd_frame){
				CDBG_ERR("msm_vfe_sync:kmalloc faile qcmd_frame 0x%p LINE:%d\n"
					,qcmd_frame,__LINE__);
				goto mem_fail2;
			}

			fphy = kmalloc(sizeof(struct msm_vfe_phy_info),
				GFP_ATOMIC);
			if (!fphy){
				CDBG_ERR("msm_vfe_sync:kmalloc  fphy faile 0%p \n",fphy);
				goto mem_fail3;
			}
			*fphy = vdata->phy;

			qcmd_frame->type    = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = fphy;

			CDBG("qcmd_frame= 0x%x phy_y= 0x%x, phy_cbcr= 0x%x\n",
				qcmd_frame, fphy->y_phy, fphy->cbcr_phy);

			spin_lock_irqsave(&msm->sync.prev_frame_q_lock,
				flags);

			list_add_tail(&qcmd_frame->list,
				&msm->sync.prev_frame_q);

			spin_unlock_irqrestore(&msm->sync.prev_frame_q_lock,
				flags);

			wake_up(&msm->sync.prev_frame_wait);
			CDBG("waked up frame thread\n");

		} else if (vdata->type == VFE_MSG_SNAPSHOT) {

			spin_lock_irqsave(&(msm->sync.get_pp_q_lock), flags);
			pp = msm->pict_pp;
			spin_unlock_irqrestore(&(msm->sync.get_pp_q_lock), flags);
			if ((pp & PP_SNAP) || (pp & PP_RAW_SNAP))
				goto sync_done;

			qcmd_frame =
				kmalloc(sizeof(struct msm_queue_cmd),
					GFP_ATOMIC);
			if (!qcmd_frame)
				goto mem_fail2;

			qcmd_frame->type    = MSM_CAM_Q_VFE_MSG;
			qcmd_frame->command = NULL;

			spin_lock_irqsave(&msm->sync.pict_frame_q_lock,
				flags);

			list_add_tail(&qcmd_frame->list,
				&msm->sync.pict_frame_q);

			spin_unlock_irqrestore(
				&msm->sync.pict_frame_q_lock, flags);
			wake_up(&msm->sync.pict_frame_wait);
		}
	}

sync_done:
	qcmd->command = (void *)vdata;
	CDBG("vdata->type = %d\n", vdata->type);

	spin_lock_irqsave(&msm->sync.msg_event_queue_lock,
		flags);
	list_add_tail(&qcmd->list, &msm->sync.msg_event_queue);
	spin_unlock_irqrestore(&msm->sync.msg_event_queue_lock,
		flags);
	wake_up(&msm->sync.msg_event_wait);
	CDBG("waked up config thread\n");

	return;

mem_fail3:
	kfree(qcmd_frame);

mem_fail2:
	kfree(qcmd);

mem_fail1:
	if (qtype == MSM_CAMERA_MSG &&
			vdata->evt_msg.len > 0)
		kfree(vdata->evt_msg.data);

	kfree(vdata);
	return;
}

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
};

static long msm_open_proc(struct msm_device *msm)
{
	long rc = 0;
	struct msm_camera_platform_data *pdata =
		msm->pdev->dev.platform_data;

	rc = msm_camvfe_check(msm);
	if (rc < 0)
		goto msm_open_proc_done;

	if (!pdata) {
		rc = -ENODEV;
		goto msm_open_proc_done;
	}
	mutex_lock(&msm->msm_lock);
	if (msm->opencnt > 5) {
		mutex_unlock(&msm->msm_lock);
		CDBG_ERR("msm open proc : open count more than 5\n");
		return -EFAULT;
	}
	msm->opencnt += 1;
	mutex_unlock(&msm->msm_lock);
	if (msm->opencnt == 1)  {
		msm_camvfe_fn_init(&msm->vfefn);

		if (msm->vfefn.vfe_init) {

			rc = msm->vfefn.vfe_init(&msm_vfe_s,
				msm->pdev);
			if (rc < 0) {
				CDBG("vfe_init failed at %ld\n", rc);
				msm->opencnt -= 1;
				goto msm_open_proc_done;
			}
			rc = msm->sctrl.s_init(&pdata->sinfo[msm->sidx]);
			if (rc < 0) {
				CDBG("sensor init at %ld\n", rc);
				msm->opencnt -= 1;
				goto msm_open_proc_done;
			}
		} else{
				rc = -ENODEV;
				CDBG_ERR("vfe_init pointer dosent exist \n");
				msm->opencnt -= 1;
				goto msm_open_proc_done;
		}

		mutex_lock(&msm->msm_sem);
		if (rc >= 0) {
			INIT_HLIST_HEAD(&msm->sync.frame);
			INIT_HLIST_HEAD(&msm->sync.stats);
		}
		mutex_unlock(&msm->msm_sem);

	} else if (msm->opencnt > 1)
		rc = 0;

msm_open_proc_done:
	return rc;
}

static int msm_open(struct inode *inode, struct file *filep)
{
	struct msm_device *pmsm;
	int rc = 0;

	rc = nonseekable_open(inode, filep);
	if (rc < 0){
		CDBG_ERR("msm_open : nonseekable open faile\n");
		goto cam_open_fail;
	}
	pmsm = container_of(inode->i_cdev,
		struct msm_device, cdev);

	if (!pmsm) {
		CDBG_ERR("msm_open : cast structure out faile\n");
		rc = -ENODEV;
		goto cam_open_fail;
	}


	rc = msm_open_proc(pmsm);
	if (rc < 0){
		CDBG_ERR("msm_open : msm open proc faile\n");
		goto cam_open_done;
	}
	filep->private_data = pmsm;
cam_open_fail:
cam_open_done:
	if(!is_perf_lock_active(&camera_perf_lock)){
		perf_lock(&camera_perf_lock);
	}
	CDBG("msm_open() open: rc = %d\n", rc);
	return rc;
}

static struct file_operations msm_fops = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl,
	.release = msm_release,
	.read = msm_read,
	.write = msm_write,
	.poll = msm_poll,
};

static long msm_setup_cdevs(struct msm_device *msm,
	dev_t devno)
{
	int rc = -ENODEV;
	struct device *class_dev;
	char name[20];

	sprintf(name, "%s%d", "control", MINOR(devno));
	class_dev = device_create(msm_class, NULL, devno, "%s", name);
	if (IS_ERR(class_dev))
		goto setup_fail_return;

	spin_lock_init(&msm->sync.msg_event_queue_lock);
	INIT_LIST_HEAD(&msm->sync.msg_event_queue);
	init_waitqueue_head(&msm->sync.msg_event_wait);

	spin_lock_init(&msm->sync.prev_frame_q_lock);
	INIT_LIST_HEAD(&msm->sync.prev_frame_q);
	init_waitqueue_head(&msm->sync.prev_frame_wait);

	spin_lock_init(&msm->sync.pict_frame_q_lock);
	INIT_LIST_HEAD(&msm->sync.pict_frame_q);
	init_waitqueue_head(&msm->sync.pict_frame_wait);

	spin_lock_init(&msm->sync.ctrl_status_lock);
	INIT_LIST_HEAD(&msm->sync.ctrl_status_queue);
	init_waitqueue_head(&msm->sync.ctrl_status_wait);
	spin_lock_init(&msm->sync.af_status_lock);
	msm->sync.af_flag = 0;
	init_waitqueue_head(&msm->sync.af_status_wait);

	spin_lock_init(&(msm->sync.get_pp_q_lock));
	INIT_LIST_HEAD(&(msm->sync.get_pp_q));
	init_waitqueue_head(&(msm->sync.get_pp_wait));

	mutex_init(&msm->msm_lock);
	mutex_init(&msm->pict_pp_lock);
	mutex_init(&msm->msm_sem);

	cdev_init(&msm->cdev, &msm_fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0)
		goto setup_cleanup_all;

	CDBG("msm_camera setup finishes!\n");
	return 0;

setup_cleanup_all:
	cdev_del(&msm->cdev);
setup_fail_return:
	return rc;
}

static long msm_control_proc(struct msm_ctrl_cmd *ctrlcmd,
	struct msm_device *vmsm)
{
	unsigned long flags;
	int timeout;
	long rc = 0;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_queue_cmd *rcmd = NULL;

	struct msm_ctrl_cmd *pctrlcmd;

	pctrlcmd = kmalloc(sizeof(struct msm_ctrl_cmd), GFP_ATOMIC);
	if (!pctrlcmd) {
		CDBG("msm_control: cannot allocate buffer ctrlcmd\n");
		rc = -ENOMEM;
		goto end;
	}

	memcpy(pctrlcmd, ctrlcmd, sizeof(struct msm_ctrl_cmd));

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kmalloc(sizeof(struct msm_queue_cmd), GFP_ATOMIC);
	if (!qcmd) {
		CDBG_ERR("msm_control: cannot allocate buffer\n");
		kfree(pctrlcmd);
		rc = -ENOMEM;
		goto end;
	}

	spin_lock_irqsave(&vmsm->sync.msg_event_queue_lock, flags);
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = pctrlcmd;
	list_add_tail(&qcmd->list, &vmsm->sync.msg_event_queue);
	wake_up(&vmsm->sync.msg_event_wait);
	spin_unlock_irqrestore(&vmsm->sync.msg_event_queue_lock, flags);

	/* wait for config status */
	timeout = ctrlcmd->timeout_ms;
	CDBG("msm_control, timeout = %d\n", timeout);
	if (timeout > 0) {
		rc =
			wait_event_timeout(
				vmsm->sync.ctrl_status_wait,
				msm_ctrl_stats_pending(vmsm),
				msecs_to_jiffies(timeout));

		CDBG("msm_control: rc = %ld\n", rc);

		if (rc == 0) {
			CDBG_WARING("msm_control: timed out\n");
			rc = -ETIMEDOUT;
			goto end;
		}
	} else {
		rc = wait_event_interruptible(
			vmsm->sync.ctrl_status_wait,
			msm_ctrl_stats_pending(vmsm));
	}
	if (rc < 0){
		CDBG_ERR("wait for interrupt error from ADSP\n");
		return -ERESTARTSYS;
	}
	/* control command status is ready */
	spin_lock_irqsave(&vmsm->sync.ctrl_status_lock, flags);
	if (!list_empty(&vmsm->sync.ctrl_status_queue)) {
		rcmd = list_first_entry(
			&vmsm->sync.ctrl_status_queue,
			struct msm_queue_cmd,
			list);

		if (!rcmd) {
			CDBG_ERR("get msg from queue faile\n");
			spin_unlock_irqrestore(&vmsm->sync.ctrl_status_lock,
				flags);
			rc = -EAGAIN;
			goto end;
		}

		list_del(&(rcmd->list));
	}
	spin_unlock_irqrestore(&vmsm->sync.ctrl_status_lock, flags);

	memcpy(ctrlcmd->value,
		((struct msm_ctrl_cmd *)(rcmd->command))->value,
		((struct msm_ctrl_cmd *)(rcmd->command))->length);

	if (((struct msm_ctrl_cmd *)(rcmd->command))->length > 0)
		kfree(((struct msm_ctrl_cmd *)
					 (rcmd->command))->value);
	kfree(rcmd);
end:
	kfree(ctrlcmd);
	CDBG("msm_control_proc: end rc = %ld\n", rc);
	return rc;
}

unsigned int msm_apps_poll(struct file *filep,
	struct poll_table_struct *pll_table, struct msm_device *pmsm)
{
	poll_wait(filep, &pmsm->sync.prev_frame_wait, pll_table);

	if (msm_frame_pending(pmsm))
		/* frame ready */
		return POLLIN | POLLRDNORM;

	return 0;
}

long msm_register(struct msm_driver *drv, const char *id)
{
	long rc = -ENODEV;

	if (drv->vmsm)
		return -EINVAL;

	if (msm_camera) {
		if (*msm_camera)
			/* @todo to support multiple sensors */
			drv->vmsm = *msm_camera;
		else
			return rc;
	}

	mutex_lock(&drv->vmsm->msm_sem);

	if (drv->vmsm->apps_id == NULL) {
		drv->vmsm->apps_id = id;

		drv->init = msm_open_proc;
		drv->ctrl = msm_control_proc;
		drv->reg_pmem = msm_register_pmem_proc;
		drv->get_frame = msm_get_frame_proc;
		drv->put_frame = msm_put_frame_buf_proc;
		drv->get_pict  = msm_get_pict_proc;
		drv->drv_poll  = msm_apps_poll;
		drv->release   = msm_release_proc;
		rc = 0;

	} else{
		CDBG_ERR("msm register faile msm_camera->apps_id:%s\n",
			drv->vmsm->apps_id);
		rc = -EFAULT;
	}
	mutex_unlock(&drv->vmsm->msm_sem);
	return rc;
}

long msm_unregister(struct msm_driver *drv,
	const char *id)
{
	long rc = -EFAULT;

	mutex_lock(&drv->vmsm->msm_sem);
	if (!strcmp(drv->vmsm->apps_id, id)) {
		drv->vmsm->apps_id = NULL;
		rc = 0;
	}
	mutex_unlock(&drv->vmsm->msm_sem);

	if (!rc)
		drv->vmsm = NULL;

	return rc;
}

int msm_camera_drv_start(struct platform_device *dev)
{
	struct msm_camera_platform_data *pdata;
	struct msm_camera_sensor_info *sinfo;
	struct msm_sensor_ctrl sctrl;
	struct msm_device *pmsm = NULL;
	int i, cnt = 0;
	int rc = -ENODEV;

	if (!dev)
		return rc;

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	msm_camera =
		kzalloc(sizeof(struct msm_device *) * pdata->snum,
			GFP_KERNEL);
	if (!msm_camera) {
		rc = -ENOMEM;
		goto start_done;
	}

	rc = alloc_chrdev_region(&msm_devno, 0,
		pdata->snum, "msm_camera");
	if (rc < 0)
		goto start_region_fail;

	msm_class = class_create(THIS_MODULE, "msm_camera");
	if (IS_ERR(msm_class))
		goto start_class_fail;

	rc = msm_camio_probe_on(dev);
	if (rc < 0)
		goto start_io_fail;

	for (i = 0; i < pdata->snum; i++) {

		if (sinfo->sensor_probe(sinfo, &sctrl) >= 0) {

			pmsm = *(msm_camera + cnt) =
				kzalloc(sizeof(struct msm_device),
					GFP_KERNEL);
			if (!pmsm)
				continue;

			pmsm->pdev = dev;
			pmsm->sctrl = sctrl;
			pmsm->sidx  = i;

			rc = msm_setup_cdevs(pmsm,
				MKDEV(MAJOR(msm_devno), cnt));

			if (rc >= 0) {
				cnt++;
				if (cnt > 0)
					break;
			} else {
				kfree(pmsm);
				*(msm_camera + cnt) = NULL;
				continue;
			}
		}

		sinfo++;
	}
	msm_camio_probe_off(dev);

	if (cnt > 0) {
		msm_camvfe_init();
		goto start_done;
	}

start_io_fail:
	class_destroy(msm_class);
start_class_fail:
	unregister_chrdev_region(msm_devno, 1);
start_region_fail:
	kfree(msm_camera);
	CDBG("FAIL: %s %s:%d\n", __FILE__, __func__, __LINE__);
start_done:
	perf_lock_init(&camera_perf_lock, PERF_LOCK_HIGH, "mm_camera");
	CDBG("DONE: %s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

int msm_camera_drv_remove(struct platform_device *dev)
{
	struct msm_camera_platform_data *pdata;
	struct msm_camera_sensor_info *sinfo;
	struct msm_device *pmsm = NULL;
	int i;

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	for (i = 0; i < pdata->snum; i++) {
		pmsm = *(msm_camera + i);
		if (pmsm) {
			cdev_del(&pmsm->cdev);
			kfree(pmsm);
			/* Not necessary; do it anyway */
			*(msm_camera + i) = NULL;
		}
	}

	pdata = dev->dev.platform_data;
	sinfo = pdata->sinfo;

	class_destroy(msm_class);
	unregister_chrdev_region(msm_devno, 1);

	kfree(msm_camera);

	return 0;
}
