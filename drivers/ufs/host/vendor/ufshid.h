/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Universal Flash Storage Host Initiated Defrag
 * Copyright (C) 2019 Samsung Electronics Co., Ltd.
 * Authors: Yongmyung Lee <ymhungry.lee@samsung.com>
 *          Jinyoung Choi <j-young.choi@samsung.com>
 */
#ifndef _QCOM_UFSHID_H
#define _QCOM_UFSHID_H

static int ufshid_stop(struct ufsf_feature *f, bool disable)
{
	u32 op = 0;
	int ret = 0;

	if (!f->hid_trigger)
		return 0;
	if (disable && !ufsf_ready(f))
		ret = ufsf_query_attr(f, 0x35, true, &op);
	f->hid_trigger = false;
	f->hid_analyze = false;
	ufsf_release(f);
	return ret;
}

static int ufshid_start(struct ufsf_feature *f, u32 op)
{
	int ret;

	if (f->hid_trigger)
		return 0;
	ret = ufsf_hold(f);
	if (ret)
		return ret;
	ret = ufsf_query_attr(f, 0x37, true, &f->hid_size);
	if (!ret)
		ret = ufsf_query_attr(f, 0x35, true, &op);
	if (ret) {
		ufsf_release(f);
		return ret;
	}
	f->hid_trigger = true;
	f->hid_analyze = op == 1;
	mod_delayed_work(system_freezable_wq, &f->hid_work,
			 msecs_to_jiffies(f->hid_delay));
	return 0;
}

static void ufshid_trigger_work(struct work_struct *work)
{
	struct ufsf_feature *f = container_of(to_delayed_work(work),
					    struct ufsf_feature, hid_work);
	u32 progress, state;
	int ret;

	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	if (!f->hid_trigger || f->reset || f->system_suspending)
		goto out;
	if (ufsf_ready(f)) {
		ufshid_stop(f, false);
		goto out;
	}
	ret = ufsf_query_attr(f, 0x39, false, &state);
	if (ret || (state != 1 && state != 3)) {
		ret = ufsf_query_attr(f, 0x38, false, &progress);
		if (ret || !progress || progress == 100) {
			ufshid_stop(f, true);
			goto out;
		}
	}
	queue_delayed_work(system_freezable_wq, &f->hid_work,
			   msecs_to_jiffies(f->hid_delay));
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
}

static ssize_t ufshid_show(struct ufsf_feature *f, const char *name, char *buf)
{
	u32 val;
	u8 idn;
	int ret;

	if (!strcmp(name, "execute"))
		return sysfs_emit(buf, "%u\n", f->hid_trigger && !f->hid_analyze);
	if (!strcmp(name, "analyze"))
		return sysfs_emit(buf, "%u\n", f->hid_trigger && f->hid_analyze);
	if (!strcmp(name, "trigger_interval"))
		return sysfs_emit(buf, "%u\n", f->hid_delay);
	if (!strcmp(name, "hid_size"))
		return sysfs_emit(buf, "%u\n", f->hid_size);
	if (!strcmp(name, "hid_state")) {
		static const char * const states[] = {
			"Analysis Required", "Analysis in Progress", "Defrag Required",
			"Defrag in Progress", "Defrag Completed", "Defrag is Not Required",
		};

		ret = ufsf_query_attr(f, 0x39, false, &val);
		if (ret || val >= ARRAY_SIZE(states))
			return ret ?: -EINVAL;
		return sysfs_emit(buf, "%s\n", states[val]);
	}
	if (!strcmp(name, "debug"))
		return sysfs_emit(buf, "%u\n", f->debug[UFSF_HID]);
	if (!strcmp(name, "block_suspend"))
		return sysfs_emit(buf, "%u\n", f->block_suspend);
	if (!strcmp(name, "auto_hibern8_enable"))
		return sysfs_emit(buf, "%u\n", f->auto_hibern8);
	if (!strcmp(name, "progress_ratio"))
		idn = 0x38;
	else if (!strcmp(name, "available_size"))
		idn = 0x36;
	else if (!strcmp(name, "debug_op"))
		idn = 0x35;
	else
		return -EINVAL;
	ret = ufsf_query_attr(f, idn, false, &val);
	if (ret)
		return ret;
	if (idn == 0x36 && val == U32_MAX)
		return sysfs_emit(buf, "Analysis Required\n");
	return sysfs_emit(buf, "%u\n", val);
}

static int ufshid_store(struct ufsf_feature *f, const char *name, const char *buf)
{
	u32 val;
	bool enable;
	int ret;

	if (!strcmp(name, "execute") || !strcmp(name, "analyze")) {
		ret = kstrtobool(buf, &enable);
		if (ret)
			return -EINVAL;
		return enable ? ufshid_start(f, !strcmp(name, "analyze") ? 1 : 2) :
			ufshid_stop(f, true);
	}
	if (!strcmp(name, "debug") || !strcmp(name, "block_suspend") ||
	    !strcmp(name, "auto_hibern8_enable")) {
		ret = kstrtobool(buf, &enable);
		if (ret)
			return -EINVAL;
		if (!strcmp(name, "debug")) {
			f->debug[UFSF_HID] = enable;
		} else if (!strcmp(name, "block_suspend")) {
			if (enable != f->block_suspend) {
				if (enable) {
					ret = ufsf_hold(f);
					if (ret)
						return ret;
				} else {
					ufsf_release(f);
				}
				f->block_suspend = enable;
			}
		} else {
			if (!ufshcd_is_auto_hibern8_supported(f->hba))
				return -EOPNOTSUPP;
			ret = ufsf_rpm_get(f);
			if (ret >= 0) {
				f->auto_hibern8 = enable;
				if (!f->pm_users)
					ufshcd_auto_hibern8_update(f->hba,
						enable ? f->saved_ahit : 0);
				ufshcd_rpm_put(f->hba);
			}
			if (ret < 0)
				return ret;
		}
		return 0;
	}
	if (kstrtouint(buf, 0, &val))
		return -EINVAL;
	if (!strcmp(name, "trigger_interval")) {
		if (val < 100 || val > 10000)
			return -EINVAL;
		f->hid_delay = val;
		return 0;
	}
	if (!strcmp(name, "hid_size")) {
		ret = ufsf_query_attr(f, 0x37, true, &val);
		if (!ret)
			f->hid_size = val;
		return ret;
	}
	if (!strcmp(name, "debug_op")) {
		if (val > 2)
			return -EINVAL;
		return val ? ufshid_start(f, val) : ufshid_stop(f, true);
	}
	return -EINVAL;
}
#endif
