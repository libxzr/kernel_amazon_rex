/*
 * idme.c
 *
 * Copyright 2013 Amazon Technologies, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License Version 2. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define IDME_OF_BOARD_ID	"/idme/board_id"
#define IDME_OF_USID	"usid"
#define IDME_SERIAL_LENGTH 32

#define PRODUCT_FEATURES_DIR "product_features"
#define PRODUCT_FEATURE_NAME_GPS "gps"
#define PRODUCT_FEATURE_NAME_WAN "wan"
#define MAC_SEC_KEY "mac_sec"
#define MAC_SEC_OWNER 1000

#define PRODUCT_FEATURE_STRING_GPS " "
#define PRODUCT_FEATURE_STRING_WAN " "

static int idme_proc_show(struct seq_file *seq, void *v)
{
	struct property *pp = (struct property *)seq->private;

	BUG_ON(!pp);

	seq_write(seq, pp->value, pp->length);

	return 0;
}

static int idme_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, idme_proc_show, PDE_DATA(inode));
}

static const struct file_operations idme_fops = {
	.owner = THIS_MODULE,
	.open = idme_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

bool board_has_wan(void)
{
	struct device_node *ap;
	int len;

	ap = of_find_node_by_path(IDME_OF_BOARD_ID);
	if (ap) {
		const char *boardid = of_get_property(ap, "value", &len);
		if (len >= 2) {
			if (boardid[0] == '0' && boardid[5] == '1')
				return true;
		}
	}

	return false;
}

EXPORT_SYMBOL(board_has_wan);

unsigned int idme_get_board_type(void)
{
	struct device_node *ap = NULL;
	char board_type[5] = { 0 };
	const char *board_id = NULL;
	unsigned int rs = 0;

	ap = of_find_node_by_path(IDME_OF_BOARD_ID);
	if (ap)
		board_id = (const char *)of_get_property(ap, "value", NULL);
	else
		pr_err("of_find_node_by_path failed\n");

	strlcpy(board_type, board_id, sizeof(board_type));
	if (unlikely(kstrtouint(board_type, 16, &rs)))
		pr_err("idme_get_board_type kstrtouint failed!\v");

	return rs;
}

EXPORT_SYMBOL(idme_get_board_type);

unsigned int idme_get_board_rev(void)
{
	struct device_node *ap = NULL;
	char board_rev[3] = { 0 };
	const char *board_id = NULL;
	unsigned int rs = 0;

	ap = of_find_node_by_path(IDME_OF_BOARD_ID);
	if (ap)
		board_id = (const char *)of_get_property(ap, "value", NULL);
	else
		pr_err("of_find_node_by_path failed\n");

	strlcpy(board_rev, (board_id + 7), sizeof(board_rev));

	if (unlikely(kstrtouint(board_rev, 16, &rs)))
		pr_err("idme_get_board_rev kstrtouint failed!\v");

	return rs;
}

EXPORT_SYMBOL(idme_get_board_rev);

int idme_hwid_value;
int idme_vcom_value;
#define OPTEE_DYNAMIC_IDME
#ifdef OPTEE_DYNAMIC_IDME
int idme_bootmode_value;
#endif
char idme_alscal1_value[32];
char idme_alscal2_value[32];
char idme_serial_value[IDME_SERIAL_LENGTH + 1];
char idme_board_id_value[16+1];
static int __init idme_init(void)
{
	struct proc_dir_entry *proc_idme = NULL;
	struct device_node *root = NULL, *child = NULL;
	struct property *pp_value = NULL;
	int perm = 0;
	/* static struct proc_dir_entry *proc_product_features_dir; */
	struct proc_dir_entry *child_pde = NULL;
	bool access_restrict = false;

	idme_vcom_value = S32_MIN;
	idme_hwid_value = 5;

	root = of_find_node_by_path("/idme");
	if (!root) {
		pr_err("%s:can't find idme node in DT\n",__func__);
		return -EINVAL;
	}

	/* Create the root IDME procfs node */
	proc_idme = proc_mkdir("idme", NULL);
	if (!proc_idme) {
		of_node_put(root);
		return -ENOMEM;
	}

	/* Populate each IDME field */
	for (child = NULL; (child = of_get_next_child(root, child));) {
		pp_value = of_find_property(child, "value", NULL);

		if (strcmp(child->name, MAC_SEC_KEY) == 0)
			access_restrict = true;
		else
			access_restrict = false;

		if (!pp_value)
			continue;

		if (of_property_read_u32(child, "permission", &perm))
			continue;

		/* These values aren't writable anyways */
		perm &= ~(S_IWUGO);

		if (access_restrict)
			perm = 0400;

		child_pde = proc_create_data(child->name, perm, proc_idme,
			&idme_fops, pp_value);

		if (child_pde && access_restrict) {
			kuid_t mac_uid;
			kgid_t mac_gid;
			mac_uid.val = MAC_SEC_OWNER;
			mac_gid.val = 0;
			proc_set_user(child_pde,mac_uid, mac_gid);
		}

		if (1) /* JUNO stack look for idme fields under /proc, so sets up sym links */ {
			char proc_path[64]="/proc/idme/";
			int ret;

			if (strlen(child->name) < 64 - strlen(proc_path)) {
				strcpy(proc_path+strlen(proc_path),child->name);

				if (!proc_symlink(child->name,NULL,proc_path))
					pr_err("%s: error in symlink /proc/%s to %s\n",__func__,child->name,proc_path);
				else
					pr_debug("%s: OK to symlink /proc/%s to %s\n",__func__,child->name,proc_path);

				/* Add for JUNO framework wifid */
				if (strncmp(child->name,"serial",6) == 0) {
					if (!proc_symlink(IDME_OF_USID,NULL,proc_path))
						pr_err("%s: error in symlink /proc/%s to %s\n",__func__,IDME_OF_USID,proc_path);
					else
						pr_debug("%s: OK to symlink /proc/%s to %s\n",__func__,IDME_OF_USID,proc_path);
				}
			}

			if ((strncmp(child->name,"vcom",4) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n",__func__, (char*) pp_value->value, pp_value->length);
				ret = kstrtoint((char*)pp_value->value, 10, &idme_vcom_value);
				if ( ret == 0)
					pr_info("%s: vcom = %d\n",__func__, idme_vcom_value);
				else
					pr_err("%s: can't read vcom err=%d\n",__func__,ret);
			}
			/* idme_hwid_value*/
			if ((strncmp(child->name,"hwid",4) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n",__func__, (char*) pp_value->value, pp_value->length);
				ret = kstrtoint((char*)pp_value->value, 10, &idme_hwid_value);
				if ( ret == 0)
					pr_info("%s: hwid = %d\n",__func__, idme_hwid_value);
				else
					pr_err("%s: can't read hwid err=%d\n",__func__,ret);
			}

#ifdef OPTEE_DYNAMIC_IDME
			if ((strncmp(child->name,"bootmode",8) == 0) && pp_value->value) {
				pr_err("%s: %s %d\n",__func__, (char*) pp_value->value, pp_value->length);
				ret = kstrtoint((char*)pp_value->value, 10, &idme_bootmode_value);
				if ( ret == 0)
					pr_info("%s: bootmode = %d\n",__func__, idme_bootmode_value);
				else
					pr_err("%s: can't read bootmode err=%d\n",__func__,ret);
			}
#endif
			if ((strncmp(child->name,"alscal1",7) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n",__func__, (char *) pp_value->value, pp_value->length);
				ret = strncpy(idme_alscal1_value, (char *) pp_value->value, 32);
				if ( ret != 0)
					pr_info("%s: alscal1 = %s\n",__func__, idme_alscal1_value);
				else
					pr_err("%s: can't read alscal1 err=%d\n",__func__,ret);
			}

			if ((strncmp(child->name,"alscal2",7) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n",__func__, (char *) pp_value->value, pp_value->length);
				ret = strncpy(idme_alscal2_value, (char *) pp_value->value, 32);
				if ( ret != 0 )
					pr_info("%s: alscal2 = %s\n",__func__, idme_alscal2_value);
				else
					pr_err("%s: can't read alscal2 err=%d\n",__func__,ret);
			}

			if ((strncmp(child->name, "serial", 6) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n", __func__, (char *) pp_value->value, pp_value->length);
				ret = strncpy(idme_serial_value, (char *) pp_value->value, IDME_SERIAL_LENGTH);
				idme_serial_value[IDME_SERIAL_LENGTH] = 0;
				if (ret != 0)
					pr_info("%s: read serial successfully\n", __func__);
				else
					pr_err("%s: can't read serial err=%d\n", __func__, ret);
			}

			if ((strncmp(child->name, "board_id", 8) == 0) && pp_value->value) {
				pr_debug("%s: %s %d\n", __func__, (char *) pp_value->value, pp_value->length);
				ret = strlcpy(idme_board_id_value, (char *) pp_value->value, sizeof(idme_board_id_value));
				if (ret > 0)
					pr_info("%s: read board_id successfully\n", __func__);
				else
					pr_err("%s: can't read board_id err=%d\n", __func__, ret);
			}
		}
	}

	of_node_put(child);
	of_node_put(root);

	return 0;
}
EXPORT_SYMBOL(idme_vcom_value);
EXPORT_SYMBOL(idme_hwid_value);
#ifdef OPTEE_DYNAMIC_IDME
EXPORT_SYMBOL(idme_bootmode_value);
#endif
EXPORT_SYMBOL(idme_alscal1_value);
EXPORT_SYMBOL(idme_alscal2_value);
EXPORT_SYMBOL(idme_serial_value);
EXPORT_SYMBOL(idme_board_id_value);

fs_initcall(idme_init);

