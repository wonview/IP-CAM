/*
 * Copyright (c) 2009 Atheros Communications Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>	/* for copy_from_user */

#include "ssv_cmd.h"


MODULE_AUTHOR("Atheros Communications");
MODULE_DESCRIPTION("Shared library for Atheros wireless LAN cards.");
MODULE_LICENSE("Dual BSD/GPL");

u32 ssvcabrio_modparam_mactxfixrate=9999;
EXPORT_SYMBOL(ssvcabrio_modparam_mactxfixrate);
u32 ssvcabrio_modparam_macampdu=9999;
EXPORT_SYMBOL(ssvcabrio_modparam_macampdu);


u32 ssvcabrio_maxtxbuf_id=9999;
EXPORT_SYMBOL(ssvcabrio_maxtxbuf_id);

u32 ssvcabrio_maxtxbuf_page=9999;
EXPORT_SYMBOL(ssvcabrio_maxtxbuf_page);

u32 ssvcabrio_maxrxbuf_id=9999;
EXPORT_SYMBOL(ssvcabrio_maxrxbuf_id);

u32 ssvcabrio_maxrxbuf_page=9999;
EXPORT_SYMBOL(ssvcabrio_maxrxbuf_page);

u32 ssvcabrio_wmm_edca1_aifsn=9999;
EXPORT_SYMBOL(ssvcabrio_wmm_edca1_aifsn);

u32 ssvcabrio_wmm_edca1_cwmin=9999;
EXPORT_SYMBOL(ssvcabrio_wmm_edca1_cwmin);


#if 0
#ifdef CONFIG_SSV_SDIO_BRIDGE
u32 ssv_devicetype = 0;
#else //CONFIG_SSV_CABRIO_WIFI
u32 ssv_devicetype = 1;
#endif
#endif
u32 ssv_devicetype = 1;
EXPORT_SYMBOL(ssv_devicetype);



/* for debug */
static struct dentry *debugfs;


static char *ssv6xxx_cmd_buf;
char *ssv6xxx_result_buf;








static int ssv6xxx_dbg_open(struct inode *inode, struct file *filp)
{
    filp->private_data = inode->i_private;
    return 0;
}


static ssize_t ssv6xxx_dbg_read(struct file *filp, char __user *buffer,
                size_t count, loff_t *ppos)
{
    int len;
    if (*ppos != 0)
        return 0;
    len = strlen(ssv6xxx_result_buf) + 1;
    if (len == 1)
        return 0;
    if (copy_to_user(buffer, ssv6xxx_result_buf, len))
        return -EFAULT;
    ssv6xxx_result_buf[0] = 0x00;
    return len;
}


static ssize_t ssv6xxx_dbg_write(struct file *filp, const char __user *buffer,
                size_t count, loff_t *ppos)
{
    if (*ppos != 0 || count > 255)
        return 0;
    
    if (copy_from_user(ssv6xxx_cmd_buf, buffer, count))
        return -EFAULT;
    ssv6xxx_cmd_buf[count-1] = 0x00;
    ssv_cmd_submit(ssv6xxx_cmd_buf);
    return count;
}


static struct file_operations ssv6xxx_dbg_fops = {
    .owner = THIS_MODULE,
    .open  = ssv6xxx_dbg_open,
    .read  = ssv6xxx_dbg_read,
    .write = ssv6xxx_dbg_write,
};


#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
extern int ssv6xxx_hci_init(void);
extern void ssv6xxx_hci_exit(void);
extern int ssv6xxx_init(void);
extern void ssv6xxx_exit(void);
extern int ssvcabrio_sdio_init(void);
extern void ssvcabrio_sdio_exit(void);
#endif
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
int ssvdevice_init(void)
#else
static int __init ssvdevice_init(void)
#endif
{
    ssv6xxx_cmd_buf = (char *)kzalloc(CLI_BUFFER_SIZE+CLI_RESULT_BUF_SIZE, GFP_KERNEL);
    if (!ssv6xxx_cmd_buf)
        return -ENOMEM;
    ssv6xxx_result_buf = ssv6xxx_cmd_buf+CLI_BUFFER_SIZE;
    ssv6xxx_cmd_buf[0] = 0x00;
    ssv6xxx_result_buf[0] = 0x00;

    debugfs = debugfs_create_dir("ssv",
						   NULL);
	if (!debugfs)
		return -ENOMEM;   

	debugfs_create_u32("ssv_devicetype", S_IRUGO|S_IWUGO, debugfs, &ssv_devicetype);
    debugfs_create_file("ssv_cmd", S_IRUGO|S_IWUGO, debugfs, NULL, &ssv6xxx_dbg_fops);

#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
    {
        int ret;
        ret = ssv6xxx_hci_init();
        if(!ret){
            ret = ssv6xxx_init();
        }if(!ret){
            ret = ssvcabrio_sdio_init();
        }
        return ret;
    }
#endif
    return 0;
}
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
void ssvdevice_exit(void)
#else
static void __exit ssvdevice_exit(void)
#endif
{
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
    ssv6xxx_exit();
    ssv6xxx_hci_exit();
    ssvcabrio_sdio_exit();
#endif
    debugfs_remove_recursive(debugfs);
    kfree(ssv6xxx_cmd_buf);
}
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
EXPORT_SYMBOL(ssvdevice_init);
EXPORT_SYMBOL(ssvdevice_exit);
#else
module_init(ssvdevice_init);
module_exit(ssvdevice_exit);
module_param_named(devicetype,ssv_devicetype, uint , S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(devicetype, "Enable sdio bridge Mode/Wifi Mode.");

module_param_named(mactxfixrate, ssvcabrio_modparam_mactxfixrate, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(mactxfixrate, "Set MacTx Fix Rate");

module_param_named(macampdu, ssvcabrio_modparam_macampdu, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(macampdu, "Set Mac Ampdu");

module_param_named(maxtxbuf_id, ssvcabrio_maxtxbuf_id, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(maxtxbuf_id, "Set Maximum ID Number of Tx buffers");

module_param_named(maxrxbuf_id, ssvcabrio_maxrxbuf_id, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(maxrxbuf_id, "Set Maximum ID Number of Rx buffers");

module_param_named(maxtxbuf_page, ssvcabrio_maxtxbuf_page, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(maxtxbuf_page, "Set Maximum Page Number of Tx buffers");

module_param_named(maxrxbuf_page, ssvcabrio_maxrxbuf_page, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(maxrxbuf_page, "Set Maximum Page Number of Rx buffers");

module_param_named(wmm_edca1_aifsn, ssvcabrio_wmm_edca1_aifsn, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(wmm_edca1_aifsn, "Set EDCA1 AIFSN");

module_param_named(wmm_edca1_cwmin, ssvcabrio_wmm_edca1_cwmin, uint, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(wmm_edca1_cwmin, "Set EDCA1 CWMIN");
#endif


