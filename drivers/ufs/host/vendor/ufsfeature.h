/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Universal Flash Storage Feature Support
 * Copyright (C) 2017-2019, 2022 Samsung Electronics Co., Ltd.
 * Authors: Yongmyung Lee <ymhungry.lee@samsung.com>
 *          Jinyoung Choi <j-young.choi@samsung.com>
 *          Keoseong Park <keosung.park@samsung.com>
 */
#ifndef _QCOM_UFSFEATURE_H
#define _QCOM_UFSFEATURE_H

#include <linux/hashtable.h>
#include <linux/highmem.h>
#include <linux/kref.h>
#include <linux/proc_fs.h>
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi.h>

int ufshcd_query_descriptor_retry(struct ufs_hba *hba,
		enum query_opcode opcode, enum desc_idn idn, u8 index,
		u8 selector, u8 *desc_buf, int *buf_len);

static unsigned int ufs_id;
module_param(ufs_id, uint, 0444);
MODULE_PARM_DESC(ufs_id, "UFS ID value (hex), e.g., ufs_id=0x012C");

enum ufsf_kind { UFSF_HID, UFSF_TP, UFSF_FBO, UFSF_COUNT };
enum ufsf_state { UFSF_NEED_INIT, UFSF_PRESENT, UFSF_SUSPEND,
	UFSF_FAILED = -2, UFSF_RESET = -3 };

struct ufsf_bounce {
	u8 *buf;
	size_t len;
	dma_addr_t dma;
	bool mapped;
};

struct ufsf_feature;
struct ufsf_node {
	struct kobject kobj;
	struct ufsf_feature *f;
	enum ufsf_kind kind;
};

struct ufstp_inode {
	struct hlist_node entry;
	u64 ino;
};

struct ufsf_feature {
	struct list_head entry;
	struct kref ref;
	struct ufs_hba *hba;
	struct mutex lock;
	spinlock_t data_lock;
	struct delayed_work init_work;
	struct delayed_work hid_work;
	struct delayed_work fbo_work;
	struct ufsf_node *node[UFSF_COUNT];
	int state[UFSF_COUNT];
	bool dying;
	bool reset;
	bool system_suspending;
	unsigned int init_retries;
	unsigned int pm_users;
	bool ahit_pending;
	bool paused_hid;
	bool paused_fbo;
	bool paused_block;
	struct ufsf_bounce *bounce;
	unsigned int bounce_slots;
	u32 saved_ahit;
	bool auto_hibern8;
	bool block_suspend;
	bool debug[UFSF_COUNT];
	bool hid_trigger;
	bool hid_analyze;
	u32 hid_size;
	u32 hid_delay;
	bool tp_enable;
	u16 tp_version;
	u32 tp_alloc_units;
	u32 tp_max_alloc_units;
	u32 max_inodes;
	u32 inode_count;
	DECLARE_HASHTABLE(inodes, 10);
	u8 tp_wb[4096];
	size_t tp_wb_size;
	u8 tp_lun;
	bool fbo_trigger;
	bool fbo_optimize;
	u32 fbo_delay;
	u32 execute_threshold;
	u16 fbo_version;
	u32 rec_lba_range_size;
	u32 max_lba_range_size;
	u32 min_lba_range_size;
	u8 max_lba_range_count;
	u16 lba_range_alignment;
	u8 fbo_wb[4096];
	size_t fbo_wb_size;
	u8 fbo_lun;
};

static LIST_HEAD(ufsf_devices);
static DEFINE_MUTEX(ufsf_devices_lock);

/* The host owns the context until remove has drained its callbacks. */
static struct ufsf_feature *ufsf_find(struct ufs_hba *hba)
{
	struct ufsf_feature *f;

	list_for_each_entry_rcu(f, &ufsf_devices, entry)
		if (f->hba == hba)
			return f;
	return NULL;
}

static void ufsf_free(struct kref *ref)
{
	struct ufsf_feature *f = container_of(ref, struct ufsf_feature, ref);

	kfree(f);
}

static int ufsf_desc_selector(struct ufs_hba *hba)
{
	switch (hba->dev_info.wspecversion) {
	case 0x220:
	case 0x310:
		return 1;
	case 0x400:
	case 0x410:
		return 0;
	default:
		return -ENODEV;
	}
}

static bool ufsf_supported(struct ufs_hba *hba)
{
	return ufs_id == UFS_VENDOR_SAMSUNG &&
		hba->dev_info.wmanufacturerid == UFS_VENDOR_SAMSUNG &&
		ufsf_desc_selector(hba) >= 0;
}

static int ufsf_ready(struct ufsf_feature *f)
{
	if (READ_ONCE(f->dying) || READ_ONCE(f->hba->shutting_down) ||
	    !READ_ONCE(f->hba->is_powered) || !f->hba->ufs_device_wlun ||
	    !ufsf_supported(f->hba))
		return -ENODEV;
	if (READ_ONCE(f->reset) || READ_ONCE(f->system_suspending) ||
	    READ_ONCE(f->hba->ufshcd_state) != UFSHCD_STATE_OPERATIONAL ||
	    READ_ONCE(f->hba->eh_flags))
		return -EBUSY;
	return 0;
}

/* Sleeping callers hold f->lock and host_sem except during SCSI waits. */
static int ufsf_rpm_get(struct ufsf_feature *f)
{
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	ret = ufshcd_rpm_get_sync(f->hba);
	if (ret >= 0)
		ret = ufsf_ready(f);
	if (ret < 0)
		ufshcd_rpm_put(f->hba);
	return ret;
}

static int ufsf_query_attr(struct ufsf_feature *f, u8 idn, bool write, u32 *val)
{
	enum ufsf_kind kind = idn >= 0x35 && idn <= 0x39 ? UFSF_HID :
			      idn >= 0x31 && idn <= 0x33 ? UFSF_FBO : UFSF_TP;
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	ret = ufsf_rpm_get(f);
	if (ret < 0)
		return ret;
	ret = ufshcd_query_attr_retry(f->hba, write ? UPIU_QUERY_OPCODE_WRITE_ATTR :
			UPIU_QUERY_OPCODE_READ_ATTR, idn, 0, 0, val);
	if (READ_ONCE(f->debug[kind]))
		dev_info(f->hba->dev, "UFS feature attr 0x%02x %s: ret=%d value=%u\n",
			 idn, write ? "write" : "read", ret, ret ? 0 : *val);
	ufshcd_rpm_put(f->hba);
	return ret;
}

static int ufsf_query_flag(struct ufsf_feature *f, enum query_opcode op,
			  bool *val)
{
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	ret = ufsf_rpm_get(f);
	if (ret < 0)
		return ret;
	ret = ufshcd_query_flag_retry(f->hba, op, 0x86, 0, val);
	ufshcd_rpm_put(f->hba);
	return ret;
}

static int ufsf_read_desc(struct ufsf_feature *f, u8 idn, u8 selector,
			  u8 *buf, int len)
{
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	ret = ufsf_rpm_get(f);
	if (ret < 0)
		return ret;
	memset(buf, 0, len);
	ret = ufshcd_query_descriptor_retry(f->hba, UPIU_QUERY_OPCODE_READ_DESC,
					  idn, 0, selector, buf, &len);
	ufshcd_rpm_put(f->hba);
	return ret ? ret : len;
}

static int ufsf_hold(struct ufsf_feature *f)
{
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	if (f->pm_users) {
		f->pm_users++;
		return 0;
	}
	ret = ufsf_rpm_get(f);
	if (ret < 0)
		return ret;
	ufshcd_hold(f->hba);
	if (f->hba->ahit)
		f->saved_ahit = f->hba->ahit;
	ufshcd_auto_hibern8_update(f->hba, 0);
	f->pm_users = 1;
	return 0;
}

static void ufsf_release(struct ufsf_feature *f)
{
	if (!f->pm_users || --f->pm_users)
		return;
	if (!ufsf_ready(f)) {
		ufshcd_auto_hibern8_update(f->hba,
					  f->auto_hibern8 ? f->saved_ahit : 0);
		f->ahit_pending = false;
	} else {
		f->ahit_pending = true;
		if (!f->dying && !f->hba->shutting_down && f->hba->is_powered)
			queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
	}
	ufshcd_release(f->hba);
	ufshcd_rpm_put(f->hba);
}

/* Buffer IDs/modes are restricted to the maintenance protocol. */
static int ufsf_buffer(struct ufsf_feature *f, u8 lun, bool write,
		       u8 mode, u8 id, u8 *buf, size_t len)
{
	struct scsi_device *sdev;
	struct scsi_sense_hdr sshdr;
	const struct scsi_exec_args args = { .sshdr = &sshdr };
	u8 cdb[10] = { write ? WRITE_BUFFER : READ_BUFFER, mode, id };
	int ret = ufsf_ready(f);

	if (ret)
		return ret;
	if (lun >= 8 || !len || len > (mode == 1 && !write ? 32768 : 4096) ||
	    !((mode == 1 && id == 1) ||
	      (mode == 2 && id == (write ? 1 : 2))))
		return -EINVAL;
	sdev = scsi_device_lookup(f->hba->host, 0, 0, lun);
	if (!sdev)
		return -ENODEV;
	put_unaligned_be24(len, cdb + 6);
	ret = ufsf_rpm_get(f);
	if (ret >= 0) {
		/* SCSI error recovery must be able to acquire host_sem. */
		up(&f->hba->host_sem);
		ret = scsi_execute_cmd(sdev, cdb,
				write ? REQ_OP_DRV_OUT : REQ_OP_DRV_IN,
				buf, len, msecs_to_jiffies(30000), 0, &args);
		down(&f->hba->host_sem);
		if (!ret)
			ret = ufsf_ready(f);
		if (ret > 0)
			ret = -EIO;
		ufshcd_rpm_put(f->hba);
	}
	scsi_device_put(sdev);
	return ret;
}

static void ufsf_bounce_clear(struct ufsf_feature *f, int tag)
{
	struct ufsf_bounce *b = &f->bounce[tag];

	if (b->mapped)
		dma_unmap_single(f->hba->dev, b->dma, b->len, DMA_TO_DEVICE);
	kfree(b->buf);
	memset(b, 0, sizeof(*b));
}

static bool ufsf_bounce_set(struct ufsf_feature *f, struct ufshcd_lrb *lrbp,
			  u8 *buf, size_t len)
{
	struct ufsf_bounce *b = &f->bounce[lrbp->task_tag];

	if (len != scsi_bufflen(lrbp->cmd) || (len & 3) ||
	    lrbp->cmd->sc_data_direction != DMA_TO_DEVICE ||
	    scsi_cmd_to_rq(lrbp->cmd)->crypt_ctx)
		return false;
	b->buf = buf;
	b->len = len;
	return true;
}

static void ufsf_fill_prdt(void *data, struct ufs_hba *hba,
			   struct ufshcd_lrb *lrbp, unsigned int segments, int *err)
{
	struct ufsf_feature *f;
	struct ufsf_bounce *b;
	struct ufshcd_sg_entry *prd;

	rcu_read_lock();
	f = ufsf_find(hba);
	/* Acquire the bounce array and its initialized slot count. */
	if (!f || !lrbp->cmd || !smp_load_acquire(&f->bounce) ||
	    lrbp->task_tag >= f->bounce_slots)
		goto out;
	b = &f->bounce[lrbp->task_tag];
	if (!b->buf)
		goto out;
	if (*err)
		goto clear;
	b->dma = dma_map_single(hba->dev, b->buf, b->len, DMA_TO_DEVICE);
	if (dma_mapping_error(hba->dev, b->dma)) {
		scsi_dma_unmap(lrbp->cmd);
		*err = SCSI_MLQUEUE_HOST_BUSY;
		goto clear;
	}
	b->mapped = true;
	prd = lrbp->ucd_prdt_ptr;
	prd->addr = cpu_to_le64(b->dma);
	prd->size = cpu_to_le32(b->len - 1);
	prd->reserved = 0;
	lrbp->utr_descriptor_ptr->prd_table_length = cpu_to_le16(
		hba->quirks & UFSHCD_QUIRK_PRDT_BYTE_GRAN ?
		ufshcd_sg_entry_size(hba) : 1);
	goto out;
clear:
	ufsf_bounce_clear(f, lrbp->task_tag);
out:
	rcu_read_unlock();
}

static void ufsf_complete_command(void *data, struct ufs_hba *hba,
				  struct ufshcd_lrb *lrbp)
{
	struct ufsf_feature *f;

	rcu_read_lock();
	f = ufsf_find(hba);
	/* Acquire the bounce array and its initialized slot count. */
	if (f && lrbp->cmd && smp_load_acquire(&f->bounce) &&
	    lrbp->task_tag < f->bounce_slots)
		ufsf_bounce_clear(f, lrbp->task_tag);
	rcu_read_unlock();
}

#include "ufshid.h"
#include "ufstp.h"
#include "ufsfbo.h"

static ssize_t ufsf_attr_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	struct ufsf_node *node = container_of(kobj, struct ufsf_node, kobj);
	struct ufsf_feature *f = node->f;
	ssize_t ret;

	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	if (f->dying || f->hba->shutting_down || f->state[node->kind] != UFSF_PRESENT) {
		ret = -ENODEV;
		goto out;
	}
	switch (node->kind) {
	case UFSF_HID:
		ret = ufshid_show(f, attr->attr.name, buf);
		break;
	case UFSF_TP:
		ret = ufstp_show(f, attr->attr.name, buf);
		break;
	case UFSF_FBO:
		ret = ufsfbo_show(f, attr->attr.name, buf);
		break;
	default:
		ret = -EINVAL;
	}
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
	return ret;
}

static ssize_t ufsf_attr_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	struct ufsf_node *node = container_of(kobj, struct ufsf_node, kobj);
	struct ufsf_feature *f = node->f;
	ssize_t ret;

	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	ret = ufsf_ready(f);
	if (!ret && f->state[node->kind] != UFSF_PRESENT)
		ret = -ENODEV;
	if (ret)
		goto out;
	switch (node->kind) {
	case UFSF_HID:
		ret = ufshid_store(f, attr->attr.name, buf);
		break;
	case UFSF_TP:
		ret = ufstp_store(f, attr->attr.name, buf);
		break;
	case UFSF_FBO:
		ret = ufsfbo_store(f, attr->attr.name, buf);
		break;
	default:
		ret = -EINVAL;
	}
	if (!ret)
		ret = count;
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
	return ret;
}

#define UFSF_ATTR(prefix, name, mode) \
	static struct kobj_attribute prefix##_##name = \
		__ATTR(name, mode, ufsf_attr_show, ufsf_attr_store)
#define UFSF_RO(prefix, name) \
	static struct kobj_attribute prefix##_##name = \
		__ATTR(name, 0444, ufsf_attr_show, NULL)
#define UFSF_WO(prefix, name, mode) \
	static struct kobj_attribute prefix##_##name = \
		__ATTR(name, mode, NULL, ufsf_attr_store)
UFSF_RO(hid, progress_ratio);
UFSF_RO(hid, available_size);
UFSF_ATTR(hid, execute, 0644);
UFSF_ATTR(hid, trigger_interval, 0644);
UFSF_ATTR(hid, analyze, 0644);
UFSF_ATTR(hid, hid_size, 0644);
UFSF_RO(hid, hid_state);
UFSF_ATTR(hid, debug, 0644);
UFSF_ATTR(hid, debug_op, 0644);
UFSF_ATTR(hid, block_suspend, 0644);
UFSF_ATTR(hid, auto_hibern8_enable, 0644);
UFSF_ATTR(tp, tp_enable, 0644);
UFSF_RO(tp, available_tp_size);
UFSF_ATTR(tp, max_inodes, 0644);
UFSF_WO(tp, add_inode, 0200);
UFSF_WO(tp, del_inode, 0200);
UFSF_RO(fbo, version);
UFSF_RO(fbo, max_lba_range_size);
UFSF_RO(fbo, min_lba_range_size);
UFSF_RO(fbo, max_lba_range_count);
UFSF_RO(fbo, lba_range_alignment);
UFSF_RO(fbo, progress_state);
UFSF_RO(fbo, analyze);
UFSF_ATTR(fbo, trigger, 0644);
UFSF_ATTR(fbo, trigger_interval, 0644);
UFSF_ATTR(fbo, execute_threshold, 0644);
UFSF_WO(fbo, optimize, 0220);
UFSF_ATTR(fbo, debug, 0644);

static struct attribute *ufshid_attrs[] = {
	&hid_progress_ratio.attr, &hid_available_size.attr, &hid_execute.attr,
	&hid_trigger_interval.attr, &hid_analyze.attr, &hid_hid_size.attr,
	&hid_hid_state.attr, &hid_debug.attr, &hid_debug_op.attr,
	&hid_block_suspend.attr, &hid_auto_hibern8_enable.attr, NULL,
};
static struct attribute *ufstp_attrs[] = {
	&tp_tp_enable.attr, &tp_available_tp_size.attr, &tp_max_inodes.attr,
	&tp_add_inode.attr, &tp_del_inode.attr, NULL,
};
static struct attribute *ufsfbo_attrs[] = {
	&fbo_version.attr, &fbo_max_lba_range_size.attr,
	&fbo_min_lba_range_size.attr, &fbo_max_lba_range_count.attr,
	&fbo_lba_range_alignment.attr, &fbo_progress_state.attr,
	&fbo_analyze.attr, &fbo_trigger.attr, &fbo_trigger_interval.attr,
	&fbo_execute_threshold.attr, &fbo_optimize.attr, &fbo_debug.attr, NULL,
};
static const struct attribute_group ufsf_groups[] = {
	{ .attrs = ufshid_attrs }, { .attrs = ufstp_attrs },
	{ .attrs = ufsfbo_attrs },
};

static void ufsf_node_release(struct kobject *kobj)
{
	struct ufsf_node *node = container_of(kobj, struct ufsf_node, kobj);

	kref_put(&node->f->ref, ufsf_free);
	kfree(node);
}

static const struct kobj_type ufsf_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.release = ufsf_node_release,
};

static int ufsf_create_sysfs(struct ufsf_feature *f, enum ufsf_kind kind)
{
	static const char * const names[] = { "ufshid", "ufstp", "ufsfbo" };
	struct ufsf_node *node;
	int ret;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;
	node->f = f;
	node->kind = kind;
	kref_get(&f->ref);
	ret = kobject_init_and_add(&node->kobj, &ufsf_ktype,
				  &f->hba->dev->kobj, "%s", names[kind]);
	if (!ret)
		ret = sysfs_create_group(&node->kobj, &ufsf_groups[kind]);
	if (ret) {
		kobject_put(&node->kobj);
		return ret;
	}
	f->node[kind] = node;
	/* Publish the initialized feature node before command preparation. */
	smp_store_release(&f->state[kind], UFSF_PRESENT);
	dev_info(f->hba->dev, "UFS %s create sysfs finished\n", names[kind]);
	kobject_uevent(&node->kobj, KOBJ_ADD);
	return 0;
}

static void ufsf_device_check_work(struct work_struct *work)
{
	struct ufsf_feature *f = container_of(to_delayed_work(work),
					    struct ufsf_feature, init_work);
	u8 *desc;
	struct ufsf_bounce *bounce;
	int len, ret, i, selector;

	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	if (f->dying || f->hba->shutting_down || !f->hba->is_powered)
		goto out;
	if (f->system_suspending) {
		queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
		goto out;
	}
	if (f->hba->ufshcd_state != UFSHCD_STATE_OPERATIONAL ||
	    f->hba->eh_flags || !f->hba->dev_info.wmanufacturerid ||
	    !f->hba->ufs_device_wlun) {
		if (++f->init_retries < 100)
			queue_delayed_work(system_freezable_wq, &f->init_work, HZ / 10);
		goto out;
	}
	if (!ufsf_supported(f->hba))
		goto out;
	if (!f->bounce) {
		bounce = kcalloc(f->hba->nutrs, sizeof(*bounce), GFP_KERNEL);
		if (!bounce) {
			if (++f->init_retries < 100)
				queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
			goto out;
		}
		f->bounce_slots = f->hba->nutrs;
		/* Publish the array after its slot count is initialized. */
		smp_store_release(&f->bounce, bounce);
	}
	if (f->reset) {
		f->init_retries = 0;
		f->paused_hid = 0;
		f->paused_fbo = 0;
		ufshid_stop(f, false);
		ufsfbo_stop(f, false);
		if (f->block_suspend && !f->paused_block)
			ufsf_release(f);
		f->block_suspend = false;
		f->paused_block = false;
		f->reset = false;
	}
	if (f->ahit_pending && !f->pm_users) {
		ret = ufsf_rpm_get(f);
		if (ret >= 0) {
			ufshcd_auto_hibern8_update(f->hba,
				f->auto_hibern8 ? f->saved_ahit : 0);
			f->ahit_pending = false;
			ufshcd_rpm_put(f->hba);
		} else {
			if (++f->init_retries < 100)
				queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
			goto out;
		}
	}
	if (f->node[UFSF_HID] || f->node[UFSF_TP] || f->node[UFSF_FBO])
		goto out;
	selector = ufsf_desc_selector(f->hba);
	desc = kzalloc(255, GFP_KERNEL);
	if (!desc)
		goto out;
	len = ufsf_read_desc(f, selector ? QUERY_DESC_IDN_DEVICE : 0xf0,
			     selector, desc, selector ? 0x65 : 0xff);
	if (len < 0) {
		if (++f->init_retries < 10)
			queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
		goto free_desc;
	}
	dev_info(f->hba->dev, "UFS DEVICE SPEC is %x; One Driver Feature Version : (010200)\n",
		 f->hba->dev_info.wspecversion);
	if (len > 0xfd && desc[0] > 0xfd && (desc[0xfd] & BIT(2))) {
		f->tp_version = get_unaligned_be16(desc + 0xdf);
		f->tp_alloc_units = get_unaligned_be32(desc + 0xdb);
		dev_info(f->hba->dev, "Support TP Spec: %04x; TP Driver Version: 010202\n",
			 f->tp_version);
		if (!f->tp_alloc_units)
			f->state[UFSF_TP] = UFSF_FAILED;
	} else {
		f->state[UFSF_TP] = UFSF_FAILED;
	}
	len = ufsf_read_desc(f, QUERY_DESC_IDN_DEVICE, 0, desc, 255);
	/* Each extension is gated by the returned descriptor's actual length. */
	if (len > 0x51 && desc[0] > 0x51 && (desc[0x51] & BIT(5))) {
		dev_info(f->hba->dev, "bUFSExFeaturesSupport: HID support\n");
		f->state[UFSF_HID] = UFSF_NEED_INIT;
	} else {
		f->state[UFSF_HID] = UFSF_FAILED;
	}
	if (len > 0x50 && desc[0] > 0x50 && (desc[0x50] & BIT(1)))
		dev_info(f->hba->dev, "dExtendedUFSFeaturesSupport: FBO support\n");
	else
		f->state[UFSF_FBO] = UFSF_FAILED;
	if (f->state[UFSF_TP] == UFSF_NEED_INIT) {
		len = ufsf_read_desc(f, selector ? QUERY_DESC_IDN_GEOMETRY : 0xf7,
				     selector, desc, selector ? 0x5e : 0xff);
		if (len > 0xf6 && desc[0] > 0xf6)
			f->tp_max_alloc_units = get_unaligned_be32(desc + 0xf3);
		if (!f->tp_max_alloc_units)
			f->state[UFSF_TP] = UFSF_FAILED;
		else
			dev_info(f->hba->dev, "TP-GEOMETRY [0xF3] dTurboPocketSizeMaxNAllocUnits (%u)\n",
				 f->tp_max_alloc_units);
	}
	if (f->state[UFSF_FBO] == UFSF_NEED_INIT) {
		len = ufsf_read_desc(f, 0x0a, 0, desc, 18);
		if (len < 18 || desc[0] < 18) {
			f->state[UFSF_FBO] = UFSF_FAILED;
		} else {
			f->fbo_version = get_unaligned_be16(desc + 1);
			f->rec_lba_range_size = get_unaligned_be32(desc + 3);
			f->max_lba_range_size = get_unaligned_be32(desc + 7);
			f->min_lba_range_size = get_unaligned_be32(desc + 11);
			f->max_lba_range_count = desc[15];
			f->lba_range_alignment = get_unaligned_be16(desc + 16);
			dev_info(f->hba->dev, "Support FBO Spec: %04x; FBO Driver version (010200)\n",
				 f->fbo_version);
		}
	}
	if (f->hba->ahit)
		f->saved_ahit = f->hba->ahit;
	for (i = 0; i < UFSF_COUNT; i++) {
		if (f->state[i] != UFSF_NEED_INIT || f->node[i])
			continue;
		ret = ufsf_create_sysfs(f, i);
		if (ret)
			f->state[i] = UFSF_FAILED;
		else if (i == UFSF_TP) {
			ret = ufstp_create_procfs(f);
			if (ret)
				dev_warn(f->hba->dev, "ufstp procfs init failed (%d)\n", ret);
		}
	}
free_desc:
	kfree(desc);
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
}

static void ufsf_prepare_command(void *data, struct ufs_hba *hba,
				 struct request *rq, struct ufshcd_lrb *lrbp, int *err)
{
	struct ufsf_feature *f;

	if (*err || !lrbp || !lrbp->cmd)
		return;
	rcu_read_lock();
	f = ufsf_find(hba);
	/* Acquire the bounce array and its initialized slot count. */
	if (f && smp_load_acquire(&f->bounce) &&
	    lrbp->task_tag < f->bounce_slots)
		ufsf_bounce_clear(f, lrbp->task_tag);
	else
		goto out;
	if (!ufsf_ready(f)) {
		ufstp_prep_fn(f, rq, lrbp, err);
		if (!*err)
			ufsfbo_prep_fn(f, lrbp, err);
	}
out:
	rcu_read_unlock();
}

static void ufsf_quiesce(struct ufsf_feature *f)
{
	int i;

	mutex_lock(&f->lock);
	WRITE_ONCE(f->dying, true);
	mutex_unlock(&f->lock);
	cancel_delayed_work_sync(&f->init_work);
	cancel_delayed_work_sync(&f->hid_work);
	cancel_delayed_work_sync(&f->fbo_work);
	ufstp_remove_procfs(f);
	for (i = 0; i < UFSF_COUNT; i++) {
		if (!f->node[i])
			continue;
		sysfs_remove_group(&f->node[i]->kobj, &ufsf_groups[i]);
		kobject_del(&f->node[i]->kobj);
		kobject_put(&f->node[i]->kobj);
	}
	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	if (f->hid_trigger)
		ufsf_release(f);
	if (f->fbo_trigger || f->fbo_optimize)
		ufsf_release(f);
	if (f->block_suspend && !f->paused_block)
		ufsf_release(f);
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
}

static void ufsf_remove(struct ufsf_feature *f)
{
	struct ufstp_inode *inode;
	struct hlist_node *tmp;
	int i, bucket;

	mutex_lock(&ufsf_devices_lock);
	list_del_rcu(&f->entry);
	if (list_empty(&ufsf_devices)) {
		unregister_trace_android_vh_ufs_prepare_command(
					ufsf_prepare_command, NULL);
		unregister_trace_android_vh_ufs_fill_prdt(ufsf_fill_prdt, NULL);
		unregister_trace_android_vh_ufs_compl_command(
					ufsf_complete_command, NULL);
	}
	mutex_unlock(&ufsf_devices_lock);
	synchronize_rcu();
	for (i = 0; i < f->bounce_slots; i++)
		ufsf_bounce_clear(f, i);
	kfree(f->bounce);
	hash_for_each_safe(f->inodes, bucket, tmp, inode, entry) {
		hash_del(&inode->entry);
		kfree(inode);
	}
	kref_put(&f->ref, ufsf_free);
}

static void ufsf_set_init(struct ufs_hba *hba)
{
	struct ufsf_feature *f;
	int ret;

	if (ufs_id != UFS_VENDOR_SAMSUNG ||
	    (hba->dev_info.wmanufacturerid &&
	     hba->dev_info.wmanufacturerid != UFS_VENDOR_SAMSUNG) ||
	    (hba->dev_info.wspecversion && ufsf_desc_selector(hba) < 0))
		return;
	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return;
	kref_init(&f->ref);
	f->hba = hba;
	mutex_init(&f->lock);
	spin_lock_init(&f->data_lock);
	hash_init(f->inodes);
	f->hid_size = U32_MAX;
	f->hid_delay = 2000;
	f->fbo_delay = 500;
	f->execute_threshold = 2;
	f->max_inodes = 1024;
	f->auto_hibern8 = ufshcd_is_auto_hibern8_supported(hba);
	INIT_DELAYED_WORK(&f->init_work, ufsf_device_check_work);
	INIT_DELAYED_WORK(&f->hid_work, ufshid_trigger_work);
	INIT_DELAYED_WORK(&f->fbo_work, ufsfbo_trigger_work);
	mutex_lock(&ufsf_devices_lock);
	ret = 0;
	if (list_empty(&ufsf_devices)) {
		ret = register_trace_android_vh_ufs_prepare_command(ufsf_prepare_command, NULL);
		if (!ret) {
			ret = register_trace_android_vh_ufs_fill_prdt(ufsf_fill_prdt, NULL);
			if (ret)
				unregister_trace_android_vh_ufs_prepare_command(
					ufsf_prepare_command, NULL);
		}
		if (!ret) {
			ret = register_trace_android_vh_ufs_compl_command(
					ufsf_complete_command, NULL);
			if (ret) {
				unregister_trace_android_vh_ufs_prepare_command(
					ufsf_prepare_command, NULL);
				unregister_trace_android_vh_ufs_fill_prdt(ufsf_fill_prdt, NULL);
			}
		}
	}
	if (!ret)
		list_add_rcu(&f->entry, &ufsf_devices);
	mutex_unlock(&ufsf_devices_lock);
	if (ret) {
		dev_warn(hba->dev, "UFS feature hook registration failed (%d)\n", ret);
		kfree(f->bounce);
		kref_put(&f->ref, ufsf_free);
		return;
	}
}

static void ufsf_start(struct ufs_hba *hba)
{
	struct ufsf_feature *f;

	rcu_read_lock();
	f = ufsf_find(hba);
	if (f && (!hba->ufs_device_wlun || ufsf_supported(hba)))
		queue_delayed_work(system_freezable_wq, &f->init_work, 0);
	rcu_read_unlock();
}

static void ufsf_reset_host(struct ufs_hba *hba)
{
	struct ufsf_feature *f;
	unsigned long flags;

	rcu_read_lock();
	f = ufsf_find(hba);
	if (f && !READ_ONCE(f->dying)) {
		WRITE_ONCE(f->reset, true);
		WRITE_ONCE(f->init_retries, 0);
		WRITE_ONCE(f->tp_enable, false);
		spin_lock_irqsave(&f->data_lock, flags);
		f->tp_wb_size = 0;
		f->fbo_wb_size = 0;
		spin_unlock_irqrestore(&f->data_lock, flags);
		cancel_delayed_work(&f->hid_work);
		cancel_delayed_work(&f->fbo_work);
		queue_delayed_work(system_freezable_wq, &f->init_work, HZ);
	}
	rcu_read_unlock();
}

static void ufsf_suspend_prepare(struct ufs_hba *hba)
{
	struct ufsf_feature *f;

	rcu_read_lock();
	f = ufsf_find(hba);
	rcu_read_unlock();
	if (!f)
		return;
	mutex_lock(&f->lock);
	down(&hba->host_sem);
	f->paused_hid = f->hid_trigger;
	f->paused_fbo = f->fbo_trigger || f->fbo_optimize;
	if (f->paused_hid)
		ufsf_release(f);
	if (f->paused_fbo)
		ufsf_release(f);
	if (f->block_suspend) {
		ufsf_release(f);
		f->paused_block = true;
	}
	WRITE_ONCE(f->system_suspending, true);
	cancel_delayed_work(&f->hid_work);
	cancel_delayed_work(&f->fbo_work);
	up(&hba->host_sem);
	mutex_unlock(&f->lock);
}

static void ufsf_resume_complete(struct ufs_hba *hba)
{
	struct ufsf_feature *f;
	int ret;

	rcu_read_lock();
	f = ufsf_find(hba);
	rcu_read_unlock();
	if (!f)
		return;
	mutex_lock(&f->lock);
	down(&hba->host_sem);
	WRITE_ONCE(f->system_suspending, false);
	if (!ufsf_ready(f)) {
		if (f->paused_block) {
			ret = ufsf_hold(f);
			if (ret)
				f->block_suspend = false;
		}
		if (f->paused_hid) {
			ret = ufsf_hold(f);
			if (!ret)
				mod_delayed_work(system_freezable_wq, &f->hid_work,
					msecs_to_jiffies(f->hid_delay));
			else {
				f->hid_trigger = false;
				f->hid_analyze = false;
			}
		}
		if (f->paused_fbo) {
			ret = ufsf_hold(f);
			if (!ret)
				mod_delayed_work(system_freezable_wq, &f->fbo_work,
					msecs_to_jiffies(f->fbo_delay));
			else {
				f->fbo_trigger = false;
				f->fbo_optimize = false;
			}
		}
	} else {
		f->block_suspend = false;
		f->hid_trigger = false;
		f->hid_analyze = false;
		f->fbo_trigger = false;
		f->fbo_optimize = false;
	}
	f->paused_block = false;
	f->paused_hid = 0;
	f->paused_fbo = 0;
	up(&hba->host_sem);
	mutex_unlock(&f->lock);
}

static void ufsf_stop(struct ufs_hba *hba)
{
	struct ufsf_feature *f;

	rcu_read_lock();
	f = ufsf_find(hba);
	rcu_read_unlock();
	if (f)
		ufsf_quiesce(f);
}

static void ufsf_exit(struct ufs_hba *hba)
{
	struct ufsf_feature *f;

	rcu_read_lock();
	f = ufsf_find(hba);
	rcu_read_unlock();
	if (f)
		ufsf_remove(f);
}

#endif
