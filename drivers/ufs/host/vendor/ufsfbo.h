/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Universal Flash Storage File Based Optimization
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 * Author: Keoseong Park <keosung.park@samsung.com>
 */
#ifndef _QCOM_UFSFBO_H
#define _QCOM_UFSFBO_H

static void ufsfbo_prep_fn(struct ufsf_feature *f, struct ufshcd_lrb *lrbp, int *err)
{
	struct scsi_cmnd *cmd = lrbp->cmd;
	u8 *buf;
	size_t len;
	unsigned long flags;
	u32 n, i, j;

	/* Acquire the state published after feature initialization. */
	if (smp_load_acquire(&f->state[UFSF_FBO]) != UFSF_PRESENT ||
	    cmd->cmnd[0] != WRITE_BUFFER || cmd->cmnd[1] != 2 || cmd->cmnd[2] != 1)
		return;
	len = scsi_bufflen(cmd);
	if (lrbp->lun >= 8 || len < 20 || len > 4096)
		return;
	buf = kmalloc(len, GFP_ATOMIC);
	if (!buf) {
		*err = SCSI_MLQUEUE_HOST_BUSY;
		return;
	}
	if (sg_copy_to_buffer(scsi_sglist(cmd), scsi_sg_count(cmd), buf, len) != len)
		goto invalid;
	n = buf[5];
	if (buf[0] || buf[4] > 1 || !n || n > f->max_lba_range_count ||
	    12 + n * 8 > len)
		goto invalid;
	for (i = 0; i < n; i++) {
		u8 *entry = buf + 12 + i * 8;
		u32 start = get_unaligned_be32(entry);
		u32 count = get_unaligned_be24(entry + 4);

		if (!start || !count || count > f->max_lba_range_size ||
		    count < f->min_lba_range_size || start > U32_MAX - count + 1 ||
		    (f->lba_range_alignment && start % f->lba_range_alignment))
			goto invalid;
		for (j = 0; j < i; j++) {
			u8 *prev = buf + 12 + j * 8;
			u32 prev_start = get_unaligned_be32(prev);
			u32 prev_count = get_unaligned_be24(prev + 4);

			if ((u64)start + count > prev_start &&
			    (u64)prev_start + prev_count > start)
				goto invalid;
		}
		if (buf[4] == 1)
			entry[7] = lrbp->lun;
	}
	if (!ufsf_bounce_set(f, lrbp, buf, len))
		goto invalid;
	spin_lock_irqsave(&f->data_lock, flags);
	len = 12 + n * 8;
	memcpy(f->fbo_wb, buf, len);
	f->fbo_wb_size = len;
	f->fbo_lun = lrbp->lun;
	spin_unlock_irqrestore(&f->data_lock, flags);
	return;
invalid:
	kfree(buf);
}

static int ufsfbo_read_result(struct ufsf_feature *f, char *out)
{
	u8 *buf;
	u8 lun;
	size_t len;
	unsigned long flags;
	u32 state;
	int ret, i, pos = 0;

	ret = ufsf_query_attr(f, 0x33, false, &state);
	if (ret)
		return ret;
	if (state != 2)
		return -EINVAL;
	spin_lock_irqsave(&f->data_lock, flags);
	len = f->fbo_wb_size;
	lun = f->fbo_lun;
	spin_unlock_irqrestore(&f->data_lock, flags);
	if (!len)
		return -EINVAL;
	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = ufsf_buffer(f, lun, false, 2, 2, buf, len);
	if (!ret) {
		if (len < 12 || 12 + buf[5] * 8 > len) {
			ret = -EIO;
		} else if (out) {
			if (buf[6])
				pos += sysfs_emit_at(out, pos, "All Ranges Regression Level %2u\n",
						     buf[7]);
			pos += sysfs_emit_at(out, pos, "CAR %1u\n", buf[6]);
			for (i = 0; i < buf[5]; i++)
				pos += sysfs_emit_at(out, pos, "N %3d lvl %2u\n",
						     i + 1, buf[19 + i * 8]);
			ret = pos;
		}
	}
	kfree(buf);
	return ret;
}

static int ufsfbo_stop(struct ufsf_feature *f, bool disable)
{
	u32 op = 0;
	int ret = 0;

	if (!f->fbo_trigger && !f->fbo_optimize)
		return 0;
	if (disable && !ufsf_ready(f))
		ret = ufsf_query_attr(f, 0x31, true, &op);
	f->fbo_trigger = false;
	f->fbo_optimize = false;
	ufsf_release(f);
	return ret;
}

static int ufsfbo_start(struct ufsf_feature *f, bool optimize)
{
	u8 *buf, lun;
	size_t len;
	unsigned long flags;
	u32 op = optimize ? 2 : 1;
	u32 state;
	int ret;

	if (f->fbo_trigger || f->fbo_optimize)
		return -EBUSY;
	ret = ufsf_hold(f);
	if (ret)
		return ret;
	if (optimize) {
		ret = ufsf_query_attr(f, 0x33, false, &state);
		if (!ret && state != 2)
			ret = -EINVAL;
		if (ret)
			goto release;
	} else {
		buf = kmalloc(4096, GFP_KERNEL);
		if (!buf) {
			ret = -ENOMEM;
			goto release;
		}
		spin_lock_irqsave(&f->data_lock, flags);
		len = f->fbo_wb_size;
		lun = f->fbo_lun;
		memcpy(buf, f->fbo_wb, len);
		spin_unlock_irqrestore(&f->data_lock, flags);
		ret = ufsf_buffer(f, lun, true, 2, 1, buf, len);
		kfree(buf);
		if (ret)
			goto release;
	}
	ret = ufsf_query_attr(f, 0x32, true, &f->execute_threshold);
	if (!ret)
		ret = ufsf_query_attr(f, 0x31, true, &op);
	if (ret)
		goto release;
	f->fbo_trigger = !optimize;
	f->fbo_optimize = optimize;
	mod_delayed_work(system_freezable_wq, &f->fbo_work,
			 msecs_to_jiffies(f->fbo_delay));
	return 0;
release:
	ufsf_release(f);
	return ret;
}

static void ufsfbo_trigger_work(struct work_struct *work)
{
	struct ufsf_feature *f = container_of(to_delayed_work(work),
					    struct ufsf_feature, fbo_work);
	u32 state;
	int ret;

	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	if (f->reset || f->system_suspending || (!f->fbo_trigger && !f->fbo_optimize))
		goto out;
	if (ufsf_ready(f)) {
		ufsfbo_stop(f, false);
		goto out;
	}
	ret = ufsf_query_attr(f, 0x33, false, &state);
	if (ret || state == 0 || state == 3 || state == 0xff) {
		ufsfbo_stop(f, !ret && state != 3);
		goto out;
	}
	if (state == 2 && f->fbo_trigger) {
		/* Analysis remains available until an explicit optimize request. */
		f->fbo_trigger = false;
		ufsf_release(f);
		goto out;
	}
	queue_delayed_work(system_freezable_wq, &f->fbo_work,
			   msecs_to_jiffies(f->fbo_delay));
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
}

static ssize_t ufsfbo_show(struct ufsf_feature *f, const char *name, char *buf)
{
	u32 val;
	int ret;

	if (!strcmp(name, "version"))
		return sysfs_emit(buf, "FBO D/D version (010200)\n");
	if (!strcmp(name, "max_lba_range_size"))
		return sysfs_emit(buf, "%u\n", f->max_lba_range_size);
	if (!strcmp(name, "min_lba_range_size"))
		return sysfs_emit(buf, "%u\n", f->min_lba_range_size);
	if (!strcmp(name, "max_lba_range_count"))
		return sysfs_emit(buf, "%u\n", f->max_lba_range_count);
	if (!strcmp(name, "lba_range_alignment"))
		return sysfs_emit(buf, "%u\n", f->lba_range_alignment);
	if (!strcmp(name, "trigger"))
		return sysfs_emit(buf, "%u\n", f->fbo_trigger);
	if (!strcmp(name, "trigger_interval"))
		return sysfs_emit(buf, "%u\n", f->fbo_delay);
	if (!strcmp(name, "execute_threshold"))
		return sysfs_emit(buf, "%u\n", f->execute_threshold);
	if (!strcmp(name, "debug"))
		return sysfs_emit(buf, "%u\n", f->debug[UFSF_FBO]);
	if (!strcmp(name, "analyze"))
		return ufsfbo_read_result(f, buf);
	if (!strcmp(name, "progress_state")) {
		ret = ufsf_query_attr(f, 0x33, false, &val);
		if (ret)
			return ret;
		if (val < 3) {
			static const char * const states[] = {
				"IDLE", "ON_GOING", "COMPL_ANALYSIS",
			};

			return sysfs_emit(buf, "%s\n", states[val]);
		}
		return sysfs_emit(buf, "%s\n", val == 3 ?
				  "COMPL_OPTIMIZATION" : "STATE_ERROR");
	}
	return -EINVAL;
}

static int ufsfbo_store(struct ufsf_feature *f, const char *name, const char *buf)
{
	u32 val;
	bool enable;
	int ret;

	if (!strcmp(name, "trigger") || !strcmp(name, "debug")) {
		if (kstrtobool(buf, &enable))
			return -EINVAL;
		if (!strcmp(name, "debug")) {
			f->debug[UFSF_FBO] = enable;
			return 0;
		}
		return enable ? ufsfbo_start(f, false) : ufsfbo_stop(f, true);
	}
	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (!strcmp(name, "trigger_interval")) {
		if (val < 100 || val > 10000)
			return -EINVAL;
		f->fbo_delay = val;
		return 0;
	}
	if (!strcmp(name, "execute_threshold")) {
		if (val > 10)
			return -EINVAL;
		ret = ufsf_query_attr(f, 0x32, true, &val);
		if (!ret)
			f->execute_threshold = val;
		return ret;
	}
	if (!strcmp(name, "optimize"))
		return val == 1 ? ufsfbo_start(f, true) : -EINVAL;
	return -EINVAL;
}
#endif
