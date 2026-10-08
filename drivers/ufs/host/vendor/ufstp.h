/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _QCOM_UFSTP_H
#define _QCOM_UFSTP_H

struct ufstp_proc {
	struct proc_dir_entry *root;
	struct ufsf_feature *f;
};
static DEFINE_MUTEX(ufstp_proc_lock);
static struct ufstp_proc ufstp_proc;

static bool ufstp_inode_lookup(struct ufsf_feature *f, u64 ino)
{
	struct ufstp_inode *inode;

	hash_for_each_possible(f->inodes, inode, entry, ino)
		if (inode->ino == ino)
			return true;
	return false;
}

static ssize_t ufstp_show(struct ufsf_feature *f, const char *name, char *buf)
{
	u32 val;
	bool enabled;
	int ret;

	if (!strcmp(name, "max_inodes"))
		return sysfs_emit(buf, "%u\n", f->max_inodes);
	if (!strcmp(name, "tp_enable")) {
		ret = ufsf_query_flag(f, UPIU_QUERY_OPCODE_READ_FLAG, &enabled);
		if (ret)
			return ret;
		WRITE_ONCE(f->tp_enable, enabled);
		return sysfs_emit(buf, "%u\n", enabled);
	}
	if (!strcmp(name, "available_tp_size")) {
		ret = ufsf_query_attr(f, 0x99, false, &val);
		return ret ? ret : sysfs_emit(buf, "%u\n", val);
	}
	return -EINVAL;
}

static int ufstp_store(struct ufsf_feature *f, const char *name, const char *buf)
{
	struct ufstp_inode *inode;
	struct hlist_node *tmp;
	unsigned long flags;
	u64 ino;
	u32 val;
	bool enable;
	int ret = 0;

	if (!strcmp(name, "tp_enable")) {
		if (kstrtobool(buf, &enable))
			return -EINVAL;
		ret = ufsf_query_flag(f, enable ? UPIU_QUERY_OPCODE_SET_FLAG :
					   UPIU_QUERY_OPCODE_CLEAR_FLAG, NULL);
		if (!ret)
			WRITE_ONCE(f->tp_enable, enable);
		return ret;
	}
	if (!strcmp(name, "max_inodes")) {
		if (kstrtouint(buf, 0, &val) || val > 1024 || val < f->inode_count)
			return -EINVAL;
		f->max_inodes = val;
		return 0;
	}
	if (kstrtoull(buf, 0, &ino) || !ino)
		return -EINVAL;
	if (!strcmp(name, "add_inode")) {
		inode = kzalloc(sizeof(*inode), GFP_KERNEL);
		if (!inode)
			return -ENOMEM;
		inode->ino = ino;
		spin_lock_irqsave(&f->data_lock, flags);
		if (ufstp_inode_lookup(f, ino)) {
			ret = -EEXIST;
		} else if (f->inode_count >= f->max_inodes) {
			ret = -ENOSPC;
		} else {
			hash_add(f->inodes, &inode->entry, ino);
			f->inode_count++;
		}
		spin_unlock_irqrestore(&f->data_lock, flags);
		if (ret)
			kfree(inode);
		return ret;
	}
	if (!strcmp(name, "del_inode")) {
		ret = -ENOENT;
		spin_lock_irqsave(&f->data_lock, flags);
		hash_for_each_possible_safe(f->inodes, inode, tmp, entry, ino) {
			if (inode->ino != ino)
				continue;
			hash_del(&inode->entry);
			f->inode_count--;
			ret = 0;
			break;
		}
		spin_unlock_irqrestore(&f->data_lock, flags);
		if (!ret)
			kfree(inode);
		return ret;
	}
	return -EINVAL;
}

static void ufstp_prep_fn(struct ufsf_feature *f, struct request *rq,
			 struct ufshcd_lrb *lrbp, int *err)
{
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct folio *folio;
	struct address_space *mapping;
	struct bio *bio;
	unsigned long flags;
	u64 ino;

	/* Acquire the state published after feature initialization. */
	if (smp_load_acquire(&f->state[UFSF_TP]) != UFSF_PRESENT)
		return;
	if (cmd->cmnd[0] == WRITE_BUFFER && cmd->cmnd[1] == 1 && cmd->cmnd[2] == 1) {
		u8 *buf;
		size_t len = scsi_bufflen(cmd);
		u32 blocks = 0, n, i;

		if (lrbp->lun >= 8 || len < 4 || len > 4096)
			return;
		buf = kmalloc(len, GFP_ATOMIC);
		if (!buf) {
			*err = SCSI_MLQUEUE_HOST_BUSY;
			return;
		}
		if (sg_copy_to_buffer(scsi_sglist(cmd), scsi_sg_count(cmd), buf, len) != len)
			goto invalid;
		n = get_unaligned_be16(buf);
		if (!n || n > 128 || 4 + n * 8 > len)
			goto invalid;
		for (i = 0; i < n; i++) {
			u32 start = get_unaligned_be32(buf + 4 + i * 8);
			u32 count = get_unaligned_be32(buf + 8 + i * 8);

			if (!start || !count || count > 0x40000 - blocks)
				goto invalid;
			blocks += count;
		}
		buf[2] = 0;
		buf[3] = lrbp->lun;
		if (!ufsf_bounce_set(f, lrbp, buf, len))
			goto invalid;
		spin_lock_irqsave(&f->data_lock, flags);
		memcpy(f->tp_wb, buf, len);
		f->tp_wb_size = len;
		f->tp_lun = lrbp->lun;
		spin_unlock_irqrestore(&f->data_lock, flags);
		return;
invalid:
		kfree(buf);
		return;
	}
	if (!READ_ONCE(f->tp_enable) ||
	    (cmd->cmnd[0] != WRITE_10 && cmd->cmnd[0] != WRITE_16) || !rq)
		return;
	bio = rq->bio;
	if (!bio || !bio->bi_iter.bi_size || !bio->bi_vcnt)
		return;
	folio = page_folio(bio->bi_io_vec[0].bv_page);
	if (folio_test_anon(folio))
		return;
	mapping = folio_mapping(folio);
	if (!mapping || !mapping->host)
		return;
	ino = mapping->host->i_ino;
	spin_lock_irqsave(&f->data_lock, flags);
	if (ino && ufstp_inode_lookup(f, ino)) {
		bool write10 = cmd->cmnd[0] == WRITE_10;
		u8 *group = &cmd->cmnd[write10 ? 6 : 14];

		*group = (*group & (write10 ? ~0x1f : ~0x3f)) | 0x12;
	}
	spin_unlock_irqrestore(&f->data_lock, flags);
}

static int ufstp_inode_list_show(struct seq_file *m, void *data)
{
	struct ufsf_feature *f = m->private;
	struct ufstp_inode *inode;
	unsigned long flags;
	int bucket;

	spin_lock_irqsave(&f->data_lock, flags);
	hash_for_each(f->inodes, bucket, inode, entry)
		seq_printf(m, "%llu\n", inode->ino);
	spin_unlock_irqrestore(&f->data_lock, flags);
	return 0;
}

static int ufstp_inode_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, ufstp_inode_list_show, pde_data(inode));
}

struct ufstp_result {
	size_t len;
	u8 data[];
};

static int ufstp_retrieve_show(struct seq_file *m, void *data)
{
	struct ufstp_result *result = m->private;
	int i, bit;

	for (i = 0; i < result->len; i++) {
		for (bit = 7; bit >= 0; bit--)
			seq_putc(m, result->data[i] & BIT(bit) ? 'O' : 'X');
		seq_putc(m, (i % 4) == 3 ? '\n' : ' ');
	}
	if (result->len % 4)
		seq_putc(m, '\n');
	return 0;
}

static int ufstp_retrieve_open(struct inode *inode, struct file *file)
{
	struct ufsf_feature *f = pde_data(inode);
	struct ufstp_result *result;
	u8 *buf;
	unsigned long flags;
	size_t len;
	u32 blocks = 0;
	u8 lun;
	int ret, i;

	result = kzalloc(struct_size(result, data, 32768), GFP_KERNEL);
	if (!result)
		return -ENOMEM;
	buf = result->data;
	mutex_lock(&f->lock);
	down(&f->hba->host_sem);
	ret = ufsf_ready(f);
	if (ret || f->state[UFSF_TP] != UFSF_PRESENT) {
		ret = ret ?: -ENODEV;
		goto out;
	}
	spin_lock_irqsave(&f->data_lock, flags);
	len = f->tp_wb_size;
	lun = f->tp_lun;
	memcpy(buf, f->tp_wb, len);
	spin_unlock_irqrestore(&f->data_lock, flags);
	if (len < 4 || !get_unaligned_be16(buf)) {
		ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < get_unaligned_be16(buf); i++)
		blocks += get_unaligned_be32(buf + 8 + i * 8);
	ret = ufsf_buffer(f, lun, true, 1, 1, buf, len);
	if (ret)
		goto out;
	memset(buf, 0, 32768);
	/* One bit per logical block; eight blocks per response byte. */
	len = DIV_ROUND_UP(blocks, 8);
	ret = ufsf_buffer(f, lun, false, 1, 1, buf, len);
	if (!ret)
		result->len = len;
out:
	up(&f->hba->host_sem);
	mutex_unlock(&f->lock);
	if (!ret)
		ret = single_open(file, ufstp_retrieve_show, result);
	if (ret)
		kfree(result);
	return ret;
}

static int ufstp_retrieve_release(struct inode *inode, struct file *file)
{
	struct seq_file *m = file->private_data;
	struct ufstp_result *result = m->private;
	int ret = single_release(inode, file);

	kfree(result);
	return ret;
}

static const struct proc_ops ufstp_inode_ops = {
	.proc_open = ufstp_inode_list_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
static const struct proc_ops ufstp_retrieve_ops = {
	.proc_open = ufstp_retrieve_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = ufstp_retrieve_release,
};

static int ufstp_create_procfs(struct ufsf_feature *f)
{
	int ret = -EEXIST;

	mutex_lock(&ufstp_proc_lock);
	if (ufstp_proc.root)
		goto out;
	ufstp_proc.root = proc_mkdir("ufstp", NULL);
	if (!ufstp_proc.root) {
		ret = -ENOMEM;
		goto out;
	}
	if (!proc_create_data("retrieve_cell_type", 0444, ufstp_proc.root,
			      &ufstp_retrieve_ops, f) ||
	    !proc_create_data("inode_list", 0444, ufstp_proc.root, &ufstp_inode_ops, f)) {
		proc_remove(ufstp_proc.root);
		ufstp_proc.root = NULL;
		ret = -ENOMEM;
		goto out;
	}
	ufstp_proc.f = f;
	ret = 0;
out:
	mutex_unlock(&ufstp_proc_lock);
	return ret;
}

static void ufstp_remove_procfs(struct ufsf_feature *f)
{
	mutex_lock(&ufstp_proc_lock);
	if (ufstp_proc.f == f) {
		proc_remove(ufstp_proc.root);
		ufstp_proc.root = NULL;
		ufstp_proc.f = NULL;
	}
	mutex_unlock(&ufstp_proc_lock);
}
#endif
