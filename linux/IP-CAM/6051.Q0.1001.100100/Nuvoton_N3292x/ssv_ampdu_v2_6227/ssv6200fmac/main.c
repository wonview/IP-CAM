/*
 * This file contains the major functions in WLAN
 * driver. It includes init, exit, open, close and main
 * thread etc..
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/hardirq.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/slab.h>
#include <net/cfg80211.h>

#include "host.h"
#include "decl.h"
#include "dev.h"
#include "cfg.h"
#include "debugfs.h"
#include "cmd.h"
#include "cabrio-if/cmd_def.h"
#include "cabrio-if/ssv_pktdef.h"
#include "cabrio-if/common.h"
//SSV -- #include "mesh.h"


#define DRIVER_RELEASE_VERSION "0.1"
const char cabrio_driver_version[] = "CABRIO_FMAC-" DRIVER_RELEASE_VERSION
#ifdef  DEBUG
    "-dbg"
#endif
    "";


/* Module parameters */
unsigned int cabrio_debug = 0xFFFFFFFF &
		                    ~(0
                              //| CABRIO_DBG_ENTER
                              //| CABRIO_DBG_LEAVE
                              //| CABRIO_DBG_MAIN
                              //| CABRIO_DBG_NET
                              //| CABRIO_DBG_MESH
                              //| CABRIO_DBG_WEXT
                              //| CABRIO_DBG_IOCTL
                              //| CABRIO_DBG_SCAN
                              //| CABRIO_DBG_ASSOC
                              //| CABRIO_DBG_JOIN
                              //| CABRIO_DBG_11D
                              //| CABRIO_DBG_DEBUGFS
                              //| CABRIO_DBG_ETHTOOL
                              //| CABRIO_DBG_HOST
                              //| CABRIO_DBG_CMD
                              | CABRIO_DBG_RX
                              | CABRIO_DBG_TX
                              //| CABRIO_DBG_USB
                              //| CABRIO_DBG_CS
                              //| CABRIO_DBG_FW
                              | CABRIO_DBG_THREAD
                              //| CABRIO_DBG_HEX
                              //| CABRIO_DBG_SDIO
                              //| CABRIO_DBG_SYSFS
                              //| CABRIO_DBG_SPI
                              //| CABRIO_DBG_CFG80211
                              //| 0xFFFFFFFF // all off
		                      );

EXPORT_SYMBOL_GPL(cabrio_debug);
module_param_named(cabrio_debug_param, cabrio_debug, int, 0644);
//SSV --
#if 0
unsigned int cabrio_disablemesh;
EXPORT_SYMBOL_GPL(cabrio_disablemesh);
module_param_named(cabrio_disablemesh, cabrio_disablemesh, int, 0644);
#endif

/*
 * This global structure is used to send the confirm_sleep command as
 * fast as possible down to the firmware.
 */
struct cmd_confirm_sleep confirm_sleep;


/*
 * the table to keep region code
 */
u16 cabrio_region_code_to_index[CABRIO_MAX_REGION_CODE] =
    { 0x10, 0x20, 0x30, 0x31, 0x32, 0x40 };

/*
 * FW rate table.  FW refers to rates by their index in this table, not by the
 * rate value itself.  Values of 0x00 are
 * reserved positions.
 */
static u8 fw_data_rates[MAX_RATES] =
    { 0x02, 0x04, 0x0B, 0x16, 0x00, 0x0C, 0x12,
      0x18, 0x24, 0x30, 0x48, 0x60, 0x6C, 0x00
};

/**
 *  cabrio_fw_index_to_data_rate - use index to get the data rate
 *
 *  @idx:    The index of data rate
 *  returns:    data rate or 0
 */
u32 cabrio_fw_index_to_data_rate(u8 idx)
{
    if (idx >= sizeof(fw_data_rates))
        idx = 0;
    return fw_data_rates[idx];
}

/**
 *  cabrio_data_rate_to_fw_index - use rate to get the index
 *
 *  @rate:    data rate
 *  returns:    index or 0
 */
u8 cabrio_data_rate_to_fw_index(u32 rate)
{
    u8 i;

    if (!rate)
        return 0;

    for (i = 0; i < sizeof(fw_data_rates); i++) {
        if (rate == fw_data_rates[i])
            return i;
    }
    return 0;
}

int cabrio_set_iface_type(struct cabrio_private *priv, enum nl80211_iftype type)
{
    int ret = 0;

    switch (type) {
    case NL80211_IFTYPE_MONITOR:
        ret = cabrio_set_monitor_mode(priv, 1);
        break;
    case NL80211_IFTYPE_STATION:
        if (priv->wdev->iftype == NL80211_IFTYPE_MONITOR)
            ret = cabrio_set_monitor_mode(priv, 0);
        #if ORIG
        if (!ret)
            ret = cabrio_set_snmp_mib(priv, SNMP_MIB_OID_BSS_TYPE, 1);
        #endif // 0
        break;
    #if 0
    case NL80211_IFTYPE_ADHOC:
        if (priv->wdev->iftype == NL80211_IFTYPE_MONITOR)
            ret = cabrio_set_monitor_mode(priv, 0);
        if (!ret)
            ret = cabrio_set_snmp_mib(priv, SNMP_MIB_OID_BSS_TYPE, 2);
        break;
    #endif // 0        
    default:
        ret = -ENOTSUPP;
    }
    return ret;
}

int cabrio_start_iface(struct cabrio_private *priv)
{
    // struct cmd_ds_802_11_mac_address cmd;
    //#define SET_MAC_CMD_SIZE (HOST_CMD_HDR_LEN + ETHER_ADDR_LEN + 2)
    //u8  dummy_buf[SET_MAC_CMD_SIZE]; /* padding 2 bytes */
    // HDR_HostCmd     *cmd = (HDR_HostCmd *)dummy_buf;
    int ret;

    cabrio_dbg_enter(CABRIO_DBG_NET);

    if (priv->power_restore) {
        ret = priv->power_restore(priv);
        if (ret)
            return ret;
    }
#if 0
    cmd->h_cmd = SSV_HOST_CMD_SET_STA_MAC;
    cmd->len = SET_MAC_CMD_SIZE;
    cmd->c_type = HOST_CMD;
    memcpy(&cmd->dat8[0], priv->current_addr, ETH_ALEN);

    ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_SET_STA_MAC, cmd);
    if (ret) {
        cabrio_dbg_net("set MAC address failed\n");
        goto err;
    }
#endif
    ret = cabrio_set_iface_type(priv, priv->wdev->iftype);
    if (ret) {
        cabrio_dbg_net("set iface type failed\n");
        goto err;
    }

    cabrio_update_channel(priv);

    priv->iface_running = true;
    cabrio_dbg_leave(CABRIO_DBG_NET);
    return 0;

err:
    if (priv->power_save)
        priv->power_save(priv);
    cabrio_dbg_leave(CABRIO_DBG_NET);
    return ret;
}

/**
 *  cabrio_dev_open - open the ethX interface
 *
 *  @dev:    A pointer to &net_device structure
 *  returns:    0 or -EBUSY if monitor mode active
 */
static int cabrio_dev_open(struct net_device *dev)
{
    struct cabrio_private *priv = dev->ml_priv;
    int ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_NET);
    if (!priv->iface_running) {
        ret = cabrio_start_iface(priv);
        if (ret)
            goto out;
    }

    spin_lock_irq(&priv->driver_lock);

    netif_carrier_off(dev);

    if (!priv->tx_pending_len)
        netif_wake_queue(dev);

    spin_unlock_irq(&priv->driver_lock);

out:
    cabrio_dbg_leave_args(CABRIO_DBG_NET, "ret %d", ret);
    return ret;
}

static bool cabrio_command_queue_empty(struct cabrio_private *priv)
{
    unsigned long flags;
    bool ret;
    spin_lock_irqsave(&priv->driver_lock, flags);
    ret = priv->cur_cmd == NULL && list_empty(&priv->cmdpendingq);
    spin_unlock_irqrestore(&priv->driver_lock, flags);
    return ret;
}

int cabrio_stop_iface(struct cabrio_private *priv)
{
    unsigned long flags;
    int ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    spin_lock_irqsave(&priv->driver_lock, flags);
    priv->iface_running = false;
    kfree_skb(priv->currenttxskb);
    priv->currenttxskb = NULL;
    priv->tx_pending_len = 0;
    spin_unlock_irqrestore(&priv->driver_lock, flags);

    cancel_work_sync(&priv->mcast_work);
    del_timer_sync(&priv->tx_lockup_timer);

    /* Disable command processing, and wait for all commands to complete */
    cabrio_dbg_main("waiting for commands to complete\n");
    wait_event(priv->waitq, cabrio_command_queue_empty(priv));
    cabrio_dbg_main("all commands completed\n");

    if (priv->power_save)
        ret = priv->power_save(priv);

    cabrio_dbg_leave(CABRIO_DBG_MAIN);
    return ret;
}

/**
 *  cabrio_eth_stop - close the ethX interface
 *
 *  @dev:    A pointer to &net_device structure
 *  returns:    0
 */
static int cabrio_eth_stop(struct net_device *dev)
{
    struct cabrio_private *priv = dev->ml_priv;

    cabrio_dbg_enter(CABRIO_DBG_NET);

    if (priv->connect_status == CABRIO_CONNECTED)
        cabrio_disconnect(priv, WLAN_REASON_DEAUTH_LEAVING);

    spin_lock_irq(&priv->driver_lock);
    netif_stop_queue(dev);
    spin_unlock_irq(&priv->driver_lock);

    cabrio_update_mcast(priv);
    cancel_delayed_work_sync(&priv->scan_work);
    if (priv->scan_req)
        cabrio_scan_done(priv);

    netif_carrier_off(priv->dev);

    if (!cabrio_iface_active(priv))
        cabrio_stop_iface(priv);

    cabrio_dbg_leave(CABRIO_DBG_NET);
    return 0;
}

void cabrio_host_to_card_done(struct cabrio_private *priv)
{
    unsigned long flags;

    cabrio_dbg_enter(CABRIO_DBG_THREAD);

    spin_lock_irqsave(&priv->driver_lock, flags);
    del_timer(&priv->tx_lockup_timer);

    priv->dnld_sent = DNLD_RES_RECEIVED;

    /* Wake main thread if commands are pending */
    if (!priv->cur_cmd || priv->tx_pending_len > 0) {
        if (!priv->wakeup_dev_required)
            wake_up(&priv->waitq);
    }

    /* We can wake the queues immediately if we aren't
       waiting for TX feedback */
    /* Wake up TX queue */
    if (!priv->currenttxskb) {
        if (priv->connect_status == CABRIO_CONNECTED)
        	netif_wake_queue(priv->dev);
    }
    spin_unlock_irqrestore(&priv->driver_lock, flags);
    cabrio_dbg_leave(CABRIO_DBG_THREAD);
}
EXPORT_SYMBOL_GPL(cabrio_host_to_card_done);

int cabrio_set_mac_address(struct net_device *dev, void *addr)
{
    int ret = 0;
    struct cabrio_private *priv = dev->ml_priv;
    struct sockaddr *phwaddr = addr;

    cabrio_dbg_enter(CABRIO_DBG_NET);

    /*
     * Can only set MAC address when all interfaces are down, to be written
     * to the hardware when one of them is brought up.
     */
    if (cabrio_iface_active(priv)) {
        return -EBUSY;
    }

    /* In case it was called from the mesh device */
    dev = priv->dev;

    memcpy(priv->current_addr, phwaddr->sa_data, ETH_ALEN);
    memcpy(dev->dev_addr, phwaddr->sa_data, ETH_ALEN);
    // SSV --
        #if 0    
    if (priv->mesh_dev)
        memcpy(priv->mesh_dev->dev_addr, phwaddr->sa_data, ETH_ALEN);
        #endif
    cabrio_dbg_leave_args(CABRIO_DBG_NET, "ret %d", ret);
    return ret;
}


static inline int mac_in_list(unsigned char *list, int list_len,
                              unsigned char *mac)
{
    while (list_len) {
        if (!memcmp(list, mac, ETH_ALEN))
            return 1;
        list += ETH_ALEN;
        list_len--;
    }
    return 0;
}

#if ORIG
static int cabrio_add_mcast_addrs(struct cmd_ds_mac_multicast_adr *cmd,
                                  struct net_device *dev, int nr_addrs)
{
    int i = nr_addrs;
    struct netdev_hw_addr *ha;
    int cnt;

    if ((dev->flags & (IFF_UP|IFF_MULTICAST)) != (IFF_UP|IFF_MULTICAST))
        return nr_addrs;

    netif_addr_lock_bh(dev);
    cnt = netdev_mc_count(dev);
    netdev_for_each_mc_addr(ha, dev) {
        if (mac_in_list(cmd->maclist, nr_addrs, ha->addr)) {
            cabrio_dbg_net("mcast address %s:%pM skipped\n", dev->name,
                    ha->addr);
            cnt--;
            continue;
        }

        if (i == CABRIO_MAX_MULTICAST_LIST_SIZE)
            break;
        memcpy(&cmd->maclist[6*i], ha->addr, ETH_ALEN);
        cabrio_dbg_net("mcast address %s:%pM added to filter\n", dev->name,
                ha->addr);
        i++;
        cnt--;
    }
    netif_addr_unlock_bh(dev);
    if (cnt)
        return -EOVERFLOW;

    return i;
}
#endif // ORIG


void cabrio_update_mcast(struct cabrio_private *priv)
{
    #if 1
    CABRIO_TODO(__func__);
    #else
    struct cmd_ds_mac_multicast_adr mcast_cmd;
    int dev_flags = 0;
    int nr_addrs;
    int old_mac_control = priv->mac_control;

    cabrio_dbg_enter(CABRIO_DBG_NET);

    if (netif_running(priv->dev))
        dev_flags |= priv->dev->flags;
        // SSV --
        #if 0
    if (priv->mesh_dev && netif_running(priv->mesh_dev))
        dev_flags |= priv->mesh_dev->flags;
        #endif
    if (dev_flags & IFF_PROMISC) {
        priv->mac_control |= CMD_ACT_MAC_PROMISCUOUS_ENABLE;
        priv->mac_control &= ~(CMD_ACT_MAC_ALL_MULTICAST_ENABLE |
                       CMD_ACT_MAC_MULTICAST_ENABLE);
        goto out_set_mac_control;
    } else if (dev_flags & IFF_ALLMULTI) {
    do_allmulti:
        priv->mac_control |= CMD_ACT_MAC_ALL_MULTICAST_ENABLE;
        priv->mac_control &= ~(CMD_ACT_MAC_PROMISCUOUS_ENABLE |
                       CMD_ACT_MAC_MULTICAST_ENABLE);
        goto out_set_mac_control;
    }

    /* Once for priv->dev, again for priv->mesh_dev if it exists */
    nr_addrs = cabrio_add_mcast_addrs(&mcast_cmd, priv->dev, 0);
    // SSV --
    #if 0
    if (nr_addrs >= 0 && priv->mesh_dev)
        nr_addrs = cabrio_add_mcast_addrs(&mcast_cmd, priv->mesh_dev, nr_addrs);
        #endif
    if (nr_addrs < 0)
        goto do_allmulti;

    if (nr_addrs) {
        int size = offsetof(struct cmd_ds_mac_multicast_adr,
                    maclist[6*nr_addrs]);

        mcast_cmd.action = cpu_to_le16(CMD_ACT_SET);
        mcast_cmd.hdr.size = cpu_to_le16(size);
        mcast_cmd.nr_of_adrs = cpu_to_le16(nr_addrs);

        cabrio_cmd_async(priv, CMD_MAC_MULTICAST_ADR, &mcast_cmd.hdr, size);

        priv->mac_control |= CMD_ACT_MAC_MULTICAST_ENABLE;
    } else
        priv->mac_control &= ~CMD_ACT_MAC_MULTICAST_ENABLE;

    priv->mac_control &= ~(CMD_ACT_MAC_PROMISCUOUS_ENABLE |
                   CMD_ACT_MAC_ALL_MULTICAST_ENABLE);
 out_set_mac_control:
    if (priv->mac_control != old_mac_control)
        cabrio_set_mac_control(priv);
#endif // TODO
    cabrio_dbg_leave(CABRIO_DBG_NET);
}

static void cabrio_set_mcast_worker(struct work_struct *work)
{
//    cabrio_dbg_net("cabrio_set_mcast_worker\n");

    struct cabrio_private *priv = container_of(work, struct cabrio_private, mcast_work);
    cabrio_update_mcast(priv);

}

void cabrio_set_multicast_list(struct net_device *dev)
{
    struct cabrio_private *priv = dev->ml_priv;

    cabrio_dbg_enter(CABRIO_DBG_NET);
    schedule_work(&priv->mcast_work);
    cabrio_dbg_enter(CABRIO_DBG_NET);
}

/**
 *  cabrio_thread - handles the major jobs in the Cabrio driver.
 *  It handles all events generated by firmware, RX data received
 *  from firmware and TX data sent from kernel.
 *
 *  @data:    A pointer to &cabrio_thread structure
 *  returns:    0
 */
static int cabrio_thread(void *data)
{
    struct net_device *dev = data;
    struct cabrio_private *priv = dev->ml_priv;
    wait_queue_t wait;
    int cmd_empty = (-1);

    cabrio_dbg_enter(CABRIO_DBG_THREAD);

    init_waitqueue_entry(&wait, current);

    for (;;) {
        int shouldsleep;
        u8 resp_idx;

        cabrio_dbg_thread("1: currenttxskb %p, dnld_sent %d\n",
                priv->currenttxskb, priv->dnld_sent);

        add_wait_queue(&priv->waitq, &wait);
        set_current_state(TASK_INTERRUPTIBLE);
        spin_lock_irq(&priv->driver_lock);

        if (kthread_should_stop())
            shouldsleep = 0;    /* Bye */
        else if (priv->surpriseremoved)
            shouldsleep = 1;    /* We need to wait until we're _told_ to die */
        else if (priv->psstate == PS_STATE_SLEEP)
            shouldsleep = 1;    /* Sleep mode. Nothing we can do till it wakes */
        else if (priv->cmd_timed_out)
            shouldsleep = 0;    /* Command timed out. Recover */
        else if (!priv->fw_ready)
            shouldsleep = 1;    /* Firmware not ready. We're waiting for it */
        else if (priv->tx_pending_len > 0)
            shouldsleep = 0;    /* We've a packet to send */
        else if (priv->resp_len[priv->resp_idx])
            shouldsleep = 0;    /* We have a command response */
        else if (priv->dnld_sent)
            shouldsleep = 1;    /* Something is en route to the device already */
        else if (priv->cur_cmd)
            shouldsleep = 1;    /* Can't send a command; one already running */
        else if (!(cmd_empty = list_empty(&priv->cmdpendingq)) &&
                    !(priv->wakeup_dev_required))
            shouldsleep = 0;    /* We have a command to send */
        else if (kfifo_len(&priv->event_fifo))
            shouldsleep = 0;    /* We have an event to process */
        else
            shouldsleep = 1;    /* No command */

        if (shouldsleep) {
            cabrio_dbg_thread("sleeping, connect_status %d, "
                "psmode %d, psstate %d, cur_cmd %zX, cmd_empty %d, wakeup_dev_required %d\n",
                priv->connect_status,
                priv->psmode, priv->psstate, (size_t)priv->cur_cmd, cmd_empty, priv->wakeup_dev_required);
            spin_unlock_irq(&priv->driver_lock);
            schedule();
        } else
            spin_unlock_irq(&priv->driver_lock);

        cabrio_dbg_thread("2: currenttxskb %p, dnld_send %d\n",
                   priv->currenttxskb, priv->dnld_sent);

        set_current_state(TASK_RUNNING);
        remove_wait_queue(&priv->waitq, &wait);

        cabrio_dbg_thread("3: currenttxskb %p, dnld_sent %d\n",
                   priv->currenttxskb, priv->dnld_sent);

        if (kthread_should_stop()) {
            cabrio_dbg_thread("break from main thread\n");
            break;
        }

        if (priv->surpriseremoved) {
            cabrio_dbg_thread("adapter removed; waiting to die...\n");
            continue;
        }

        cabrio_dbg_thread("4: currenttxskb %p, dnld_sent %d\n",
               priv->currenttxskb, priv->dnld_sent);

        /* Process any pending command response */
        spin_lock_irq(&priv->driver_lock);
        resp_idx = priv->resp_idx;
        if (priv->resp_len[resp_idx]) {
            spin_unlock_irq(&priv->driver_lock);
            cabrio_process_command_response(priv,
                priv->resp_buf[resp_idx],
                priv->resp_len[resp_idx]);
            spin_lock_irq(&priv->driver_lock);
            priv->resp_len[resp_idx] = 0;
        }
        spin_unlock_irq(&priv->driver_lock);

        /* Process hardware events, e.g. card removed, link lost */
        spin_lock_irq(&priv->driver_lock);
        while (kfifo_len(&priv->event_fifo)) {
        	static u8 event[SSV_EVENT_SIZE]; // Use static to prevent from taking large stack size.
        	HDR_HostEvent  *host_event = (HDR_HostEvent *)event;

        	kfifo_out(&priv->event_fifo, event, sizeof(HDR_HostEvent));
        	BUG_ON(host_event->len > SSV_EVENT_SIZE);
        	if (host_event->len > sizeof(HDR_HostEvent))
        		kfifo_out(&priv->event_fifo, &event[sizeof(HDR_HostEvent)],
        				  host_event->len - sizeof(HDR_HostEvent));
            spin_unlock_irq(&priv->driver_lock);
            cabrio_process_event(priv, event);
            spin_lock_irq(&priv->driver_lock);
        }
        spin_unlock_irq(&priv->driver_lock);

        if (priv->wakeup_dev_required) {
            cabrio_dbg_thread("Waking up device...\n");
            /* Wake up device */
            if (priv->exit_deep_sleep(priv))
                cabrio_dbg_thread("Wakeup device failed\n");
            continue;
        }

        /* command timeout stuff */
        if (priv->cmd_timed_out && priv->cur_cmd) {
            struct host_cmd_node *cmdnode = priv->cur_cmd;

            netdev_info(dev, "Timeout submitting command 0x%04x\n",
                    le16_to_cpu(cmdnode->cmdbuf->h_cmd));
            cabrio_complete_command(priv, cmdnode, -ETIMEDOUT);
            if (priv->reset_card)
                priv->reset_card(priv);
        }
        priv->cmd_timed_out = 0;

        if (!priv->fw_ready)
            continue;

        /* Check if we need to confirm Sleep Request received previously */
        if (priv->psstate == PS_STATE_PRE_SLEEP &&
            !priv->dnld_sent && !priv->cur_cmd) {
            if (priv->connect_status == CABRIO_CONNECTED) {
                cabrio_dbg_thread("pre-sleep, currenttxskb %p, "
                    "dnld_sent %d, cur_cmd %p\n",
                    priv->currenttxskb, priv->dnld_sent,
                    priv->cur_cmd);

                cabrio_ps_confirm_sleep(priv);
            } else {
                /* workaround for firmware sending
                 * deauth/linkloss event immediately
                 * after sleep request; remove this
                 * after firmware fixes it
                 */
                priv->psstate = PS_STATE_AWAKE;
                netdev_alert(dev,
                         "ignore PS_SleepConfirm in non-connected state\n");
            }
        }

        /* The PS state is changed during processing of Sleep Request
         * event above
         */
        if ((priv->psstate == PS_STATE_SLEEP) ||
            (priv->psstate == PS_STATE_PRE_SLEEP))
            continue;

        if (priv->is_deep_sleep)
            continue;

        /* Execute the next command */
        if (!priv->dnld_sent && !priv->cur_cmd)
            cabrio_execute_next_command(priv);

        spin_lock_irq(&priv->driver_lock);
        if (/*!priv->dnld_sent && */priv->tx_pending_len > 0) {
            int ret = priv->hw_host_to_card(priv, CABRIO_MS_DAT,
                            priv->tx_pending_buf,
                            priv->tx_pending_len);
            if (ret < 0) {
                cabrio_dbg_tx("host_to_card failed %d\n", ret);
                priv->dnld_sent = DNLD_RES_RECEIVED;
            } else {
                #if 0
                mod_timer(&priv->tx_lockup_timer,
                      jiffies + (HZ * 5));
                #endif
            }
            priv->tx_pending_len = 0;
            if (!priv->currenttxskb) {
                /* We can wake the queues immediately if we aren't
                   waiting for TX feedback */
                if ((priv->connect_status == CABRIO_CONNECTED) && (ret == 0))
                    netif_wake_queue(priv->dev);
                //SSV --
                #if 0
                if (priv->mesh_dev &&
                    netif_running(priv->mesh_dev))
                    netif_wake_queue(priv->mesh_dev);
                #endif
            }
        }
        spin_unlock_irq(&priv->driver_lock);
    }

    del_timer(&priv->command_timer);
    del_timer(&priv->tx_lockup_timer);
    del_timer(&priv->auto_deepsleep_timer);

    cabrio_dbg_leave(CABRIO_DBG_THREAD);
    return 0;
}

/**
 * cabrio_setup_firmware - gets the HW spec from the firmware and sets
 *        some basic parameters
 *
 *  @priv:    A pointer to &struct cabrio_private structure
 *  returns:    0 or -1
 */
static int cabrio_setup_firmware(struct cabrio_private *priv)
{
    int ret = -1;
    // s16 curlevel = 0, minlevel = 0, maxlevel = 0;

    cabrio_dbg_enter(CABRIO_DBG_FW);

    /* Read MAC address from firmware */
    // SSV -
    #if 1
    memset(priv->current_addr, 0xff, ETH_ALEN);
    ret = cabrio_update_hw_spec(priv);
    if (ret)
        goto done;
    #endif 
    // SSV -
    #if 0
    /* Read power levels if available */
    ret = cabrio_get_tx_power(priv, &curlevel, &minlevel, &maxlevel);
    if (ret == 0) {
        priv->txpower_cur = curlevel;
        priv->txpower_min = minlevel;
        priv->txpower_max = maxlevel;
    }
    #endif
    // SSV -
    /* Send cmd to FW to enable 11D function */
    //ret = cabrio_set_snmp_mib(priv, SNMP_MIB_OID_11D_ENABLE, 1);
    // SSV -
    #if 0
    cabrio_set_mac_control(priv);
    #else
    CABRIO_TODO(__func__);
    ret = 0;
    #endif
done:
    cabrio_dbg_leave_args(CABRIO_DBG_FW, "ret %d", ret);
    return ret;
}


int cabrio_suspend(struct cabrio_private *priv)
{
    int ret;

    cabrio_dbg_enter(CABRIO_DBG_FW);

    if (priv->is_deep_sleep) {
        ret = cabrio_set_deep_sleep(priv, 0);
        if (ret) {
            netdev_err(priv->dev,
                   "deep sleep cancellation failed: %d\n", ret);
            return ret;
        }
        priv->deep_sleep_required = 1;
    }

    ret = cabrio_set_host_sleep(priv, 1);

    netif_device_detach(priv->dev);
    //SSV --
    #if 0
    if (priv->mesh_dev)
        netif_device_detach(priv->mesh_dev);
        #endif
    cabrio_dbg_leave_args(CABRIO_DBG_FW, "ret %d", ret);
    return ret;
}
EXPORT_SYMBOL_GPL(cabrio_suspend);

int cabrio_resume(struct cabrio_private *priv)
{
    int ret;

    cabrio_dbg_enter(CABRIO_DBG_FW);

    ret = cabrio_set_host_sleep(priv, 0);

    netif_device_attach(priv->dev);
    //SSV --
    #if 0
    if (priv->mesh_dev)
        netif_device_attach(priv->mesh_dev);
    #endif
    if (priv->deep_sleep_required) {
        priv->deep_sleep_required = 0;
        ret = cabrio_set_deep_sleep(priv, 1);
        if (ret)
            netdev_err(priv->dev,
                   "deep sleep activation failed: %d\n", ret);
    }

    if (priv->setup_fw_on_resume)
        ret = cabrio_setup_firmware(priv);

    cabrio_dbg_leave_args(CABRIO_DBG_FW, "ret %d", ret);
    return ret;
}
EXPORT_SYMBOL_GPL(cabrio_resume);

/**
 * cabrio_cmd_timeout_handler - handles the timeout of command sending.
 * It will re-send the same command again.
 *
 * @data: &struct cabrio_private pointer
 */
static void cabrio_cmd_timeout_handler(unsigned long data)
{
    struct cabrio_private *priv = (struct cabrio_private *)data;
    unsigned long flags;

    cabrio_dbg_enter(CABRIO_DBG_CMD);
    spin_lock_irqsave(&priv->driver_lock, flags);

    if (!priv->cur_cmd)
        goto out;

    netdev_info(priv->dev, "command 0x%04x timed out\n",
            le16_to_cpu(priv->cur_cmd->cmdbuf->h_cmd));

    priv->cmd_timed_out = 1;

    /*
     * If the device didn't even acknowledge the command, reset the state
     * so that we don't block all future commands due to this one timeout.
     */
    if (priv->dnld_sent == DNLD_CMD_SENT)
        priv->dnld_sent = DNLD_RES_RECEIVED;

    wake_up(&priv->waitq);
out:
    spin_unlock_irqrestore(&priv->driver_lock, flags);
    cabrio_dbg_leave(CABRIO_DBG_CMD);
}

/**
 * cabrio_tx_lockup_handler - handles the timeout of the passing of TX frames
 * to the hardware. This is known to frequently happen with SD8686 when
 * waking up after a Wake-on-WLAN-triggered resume.
 *
 * @data: &struct cabrio_private pointer
 */
static void cabrio_tx_lockup_handler(unsigned long data)
{
    struct cabrio_private *priv = (struct cabrio_private *)data;
    unsigned long flags;

    cabrio_dbg_enter(CABRIO_DBG_TX);
    spin_lock_irqsave(&priv->driver_lock, flags);

    netdev_info(priv->dev, "TX lockup detected\n");
    if (priv->reset_card)
        priv->reset_card(priv);

    priv->dnld_sent = DNLD_RES_RECEIVED;
    wake_up_interruptible(&priv->waitq);

    spin_unlock_irqrestore(&priv->driver_lock, flags);
    cabrio_dbg_leave(CABRIO_DBG_TX);
}

/**
 * auto_deepsleep_timer_fn - put the device back to deep sleep mode when
 * timer expires and no activity (command, event, data etc.) is detected.
 * @data:    &struct cabrio_private pointer
 * returns:    N/A
 */
static void auto_deepsleep_timer_fn(unsigned long data)
{
    #if 1
    CABRIO_TODO(__func__);
    #else
    struct cabrio_private *priv = (struct cabrio_private *)data;

    cabrio_dbg_enter(CABRIO_DBG_CMD);

    if (priv->is_activity_detected) {
        priv->is_activity_detected = 0;
    } else {
        if (priv->is_auto_deep_sleep_enabled &&
            (!priv->wakeup_dev_required) &&
            (priv->connect_status != CABRIO_CONNECTED)) {
            struct cmd_header cmd;

            cabrio_dbg_main("Entering auto deep sleep mode...\n");
            memset(&cmd, 0, sizeof(cmd));
            cmd.size = cpu_to_le16(sizeof(cmd));
            cabrio_cmd_async(priv, CMD_802_11_DEEP_SLEEP, &cmd,
                    sizeof(cmd));
        }
    }
    mod_timer(&priv->auto_deepsleep_timer , jiffies +
                (priv->auto_deep_sleep_timeout * HZ)/1000);
    cabrio_dbg_leave(CABRIO_DBG_CMD);
    #endif // TODO
}

int cabrio_enter_auto_deep_sleep(struct cabrio_private *priv)
{
    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    priv->is_auto_deep_sleep_enabled = 1;
    if (priv->is_deep_sleep)
        priv->wakeup_dev_required = 1;
    mod_timer(&priv->auto_deepsleep_timer ,
            jiffies + (priv->auto_deep_sleep_timeout * HZ)/1000);

    cabrio_dbg_leave(CABRIO_DBG_SDIO);
    return 0;
}

int cabrio_exit_auto_deep_sleep(struct cabrio_private *priv)
{
    cabrio_dbg_enter(CABRIO_DBG_SDIO);

    priv->is_auto_deep_sleep_enabled = 0;
    priv->auto_deep_sleep_timeout = 0;
    del_timer(&priv->auto_deepsleep_timer);

    cabrio_dbg_leave(CABRIO_DBG_SDIO);
    return 0;
}

static int cabrio_init_adapter(struct cabrio_private *priv)
{
    int ret;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    memset(priv->current_addr, 0xff, ETH_ALEN);

    priv->connect_status = CABRIO_DISCONNECTED;
    priv->channel = DEFAULT_AD_HOC_CHANNEL;
    priv->mac_control = CMD_ACT_MAC_RX_ON | CMD_ACT_MAC_TX_ON;
    priv->radio_on = 1;
    priv->psmode = CABRIO802_11POWERMODECAM;
    priv->psstate = PS_STATE_FULL_POWER;
    priv->is_deep_sleep = 0;
    priv->is_auto_deep_sleep_enabled = 0;
    priv->deep_sleep_required = 0;
    priv->wakeup_dev_required = 0;
    init_waitqueue_head(&priv->ds_awake_q);
    init_waitqueue_head(&priv->scan_q);
    priv->authtype_auto = 1;
    priv->is_host_sleep_configured = 0;
    priv->is_host_sleep_activated = 0;
    init_waitqueue_head(&priv->host_sleep_q);
    mutex_init(&priv->lock);

    setup_timer(&priv->command_timer, cabrio_cmd_timeout_handler,
        (unsigned long)priv);
    setup_timer(&priv->tx_lockup_timer, cabrio_tx_lockup_handler,
        (unsigned long)priv);
    setup_timer(&priv->auto_deepsleep_timer, auto_deepsleep_timer_fn,
            (unsigned long)priv);

    INIT_LIST_HEAD(&priv->cmdfreeq);
    INIT_LIST_HEAD(&priv->cmdpendingq);

    spin_lock_init(&priv->driver_lock);

    /* Allocate the command buffers */
    if (cabrio_allocate_cmd_buffer(priv)) {
        pr_err("Out of memory allocating command buffers\n");
        ret = -ENOMEM;
        goto out;
    }
    priv->resp_idx = 0;
    priv->resp_len[0] = priv->resp_len[1] = 0;

    /* Create the event FIFO */
    ret = kfifo_alloc(&priv->event_fifo, SSV_EVENT_SIZE * SSV_EVENT_COUNT, GFP_KERNEL);
    if (ret) {
        pr_err("Out of memory allocating event FIFO buffer\n");
        goto out;
    }

out:
    cabrio_dbg_leave_args(CABRIO_DBG_MAIN, "ret %d", ret);

    return ret;
}

static void cabrio_free_adapter(struct cabrio_private *priv)
{
    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    cabrio_free_cmd_buffer(priv);
    kfifo_free(&priv->event_fifo);
    del_timer(&priv->command_timer);
    del_timer(&priv->tx_lockup_timer);
    del_timer(&priv->auto_deepsleep_timer);

    cabrio_dbg_leave(CABRIO_DBG_MAIN);
}

static const struct net_device_ops cabrio_netdev_ops = {
    .ndo_open         = cabrio_dev_open,
    .ndo_stop        = cabrio_eth_stop,
    .ndo_start_xmit        = cabrio_start_xmit,
    .ndo_set_mac_address    = cabrio_set_mac_address,
    .ndo_set_rx_mode    = cabrio_set_multicast_list,
    .ndo_change_mtu        = eth_change_mtu,
    .ndo_validate_addr    = eth_validate_addr,
};

/**
 * cabrio_add_card - adds the card. It will probe the
 * card, allocate the cabrio_priv and initialize the device.
 *
 * @card:   A pointer to card
 * @dmdev:  A pointer to &struct device
 * returns: A pointer to &struct cabrio_private structure
 */
struct cabrio_private *cabrio_add_card(void *card, struct device *dmdev)
{
    struct net_device *dev;
    struct wireless_dev *wdev;
    struct cabrio_private *priv = NULL;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    /* Allocate an Ethernet device and register it */
    wdev = cabrio_cfg_alloc(dmdev);
    if (IS_ERR(wdev)) {
        pr_err("cfg80211 init failed\n");
        goto done;
    }

    wdev->iftype = NL80211_IFTYPE_STATION;
    priv = wdev_priv(wdev);
    priv->wdev = wdev;

    if (cabrio_init_adapter(priv)) {
        pr_err("failed to initialize adapter structure\n");
        goto err_wdev;
    }

    dev = alloc_netdev(0, "wlan%d", ether_setup);
    if (!dev) {
        dev_err(dmdev, "no memory for network device instance\n");
        goto err_adapter;
    }

    dev->ieee80211_ptr = wdev;
    dev->ml_priv = priv;
    SET_NETDEV_DEV(dev, dmdev);
    wdev->netdev = dev;
    priv->dev = dev;

    dev->netdev_ops = &cabrio_netdev_ops;
    dev->watchdog_timeo = 5 * HZ;
    dev->ethtool_ops = &cabrio_ethtool_ops;
    dev->flags |= IFF_BROADCAST | IFF_MULTICAST;
    // dev->flags |= IFF_BROADCAST;
    // Make sure sockets has enough headroom for SSV socket header.
    dev->needed_headroom = ETHER_HDR_LEN + M0_HDR_LEN_MAX; 

    priv->card = card;

    strcpy(dev->name, "wlan%d");

    cabrio_dbg_thread("Starting main thread...\n");

    init_waitqueue_head(&priv->waitq);

    priv->main_thread = kthread_run(cabrio_thread, dev, "cabrio_main");
    if (IS_ERR(priv->main_thread)) {
        cabrio_dbg_thread("Error creating main thread.\n");
        goto err_ndev;
    }

    priv->work_thread = create_singlethread_workqueue("cabrio_worker");

    INIT_WORK(&priv->mcast_work, cabrio_set_mcast_worker);

    priv->wol_criteria = EHS_REMOVE_WAKEUP;
    priv->wol_gpio = 0xff;
    priv->wol_gap = 20;
    priv->ehs_remove_supported = true;

    goto done;

 err_ndev:
    free_netdev(dev);

 err_adapter:
    cabrio_free_adapter(priv);

 err_wdev:
    cabrio_cfg_free(priv);

    priv = NULL;

done:
    cabrio_dbg_leave_args(CABRIO_DBG_MAIN, "priv %p", priv);
    return priv;
}
EXPORT_SYMBOL_GPL(cabrio_add_card);


void cabrio_remove_card(struct cabrio_private *priv)
{
    struct net_device *dev = priv->dev;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    //SSV -- cabrio_remove_mesh(priv);
    cabrio_scan_deinit(priv);

    /* worker thread destruction blocks on the in-flight command which
     * should have been cleared already in cabrio_stop_card().
     */
    cabrio_dbg_main("destroying worker thread\n");
    destroy_workqueue(priv->work_thread);
    cabrio_dbg_main("done destroying worker thread\n");

    if (priv->psmode == CABRIO802_11POWERMODEMAX_PSP) {
        priv->psmode = CABRIO802_11POWERMODECAM;
        cabrio_set_ps_mode(priv, PS_MODE_ACTION_EXIT_PS, true);
    }

    if (priv->is_deep_sleep) {
        priv->is_deep_sleep = 0;
        wake_up_interruptible(&priv->ds_awake_q);
    }

    priv->is_host_sleep_configured = 0;
    priv->is_host_sleep_activated = 0;
    wake_up_interruptible(&priv->host_sleep_q);

    /* Stop the thread servicing the interrupts */
    priv->surpriseremoved = 1;
    kthread_stop(priv->main_thread);

    cabrio_free_adapter(priv);
    cabrio_cfg_free(priv);
    free_netdev(dev);

    cabrio_dbg_leave(CABRIO_DBG_MAIN);
}
EXPORT_SYMBOL_GPL(cabrio_remove_card);


int cabrio_rtap_supported(struct cabrio_private *priv)
{
   // SSV --
   #if 0
    if (CABRIO_FW_MAJOR_REV(priv->fwrelease) == CABRIO_FW_V5)
        return 1;

    /* newer firmware use a capability mask */
    return ((CABRIO_FW_MAJOR_REV(priv->fwrelease) >= CABRIO_FW_V10) /* SSV --  &&
        (priv->fwcapinfo & MESH_CAPINFO_ENABLE_MASK)*/);
  #endif
  return 0;
}


int cabrio_start_card(struct cabrio_private *priv)
{
    struct net_device *dev = priv->dev;
    int ret = -1;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    /* poke the firmware */
    ret = cabrio_setup_firmware(priv);
    if (ret)
        goto done;

        //SSV --
    #if 0
    if (!cabrio_disablemesh)
        cabrio_init_mesh(priv);
    else
    #endif
        pr_info("%s: mesh disabled\n", dev->name);

    if (cabrio_cfg_register(priv)) {
        pr_err("cannot register device\n");
        goto done;
    }
    //SSV --
    #if 0
    if (cabrio_mesh_activated(priv))
        cabrio_start_mesh(priv);
        #endif
    cabrio_debugfs_init_one(priv, dev);

    netdev_info(dev, "SSV Cabrio WLAN 802.11b/g/n adapter\n");

    ret = 0;

done:
    cabrio_dbg_leave_args(CABRIO_DBG_MAIN, "ret %d", ret);
    return ret;
}
EXPORT_SYMBOL_GPL(cabrio_start_card);


void cabrio_stop_card(struct cabrio_private *priv)
{
    struct net_device *dev;

    cabrio_dbg_enter(CABRIO_DBG_MAIN);

    if (!priv)
        goto out;
    dev = priv->dev;

    spin_lock_irq(&priv->driver_lock);
    if (!netif_queue_stopped(dev))
        netif_stop_queue(priv->dev);

    if (netif_carrier_ok(dev))
    	netif_carrier_off(dev);
    spin_unlock_irq(&priv->driver_lock);

    cabrio_debugfs_remove_one(priv);
    //SSV -- cabrio_deinit_mesh(priv);
    unregister_netdev(dev);

out:
    cabrio_dbg_leave(CABRIO_DBG_MAIN);
}
EXPORT_SYMBOL_GPL(cabrio_stop_card);


void cabrio_queue_event(struct cabrio_private *priv, void *event, u32 size)
{
    unsigned long flags;

    cabrio_dbg_enter(CABRIO_DBG_THREAD);
    spin_lock_irqsave(&priv->driver_lock, flags);

    if (priv->psstate == PS_STATE_SLEEP)
        priv->psstate = PS_STATE_AWAKE;

    if (size > SSV_EVENT_SIZE) {
    	spin_unlock_irqrestore(&priv->driver_lock, flags);
    	pr_err("Excessive event size %d received.\n", size);
    	return;
    }
    kfifo_in(&priv->event_fifo, (unsigned char *)event, size);

    wake_up(&priv->waitq);

    spin_unlock_irqrestore(&priv->driver_lock, flags);
    cabrio_dbg_leave(CABRIO_DBG_THREAD);
}
EXPORT_SYMBOL_GPL(cabrio_queue_event);

void cabrio_notify_command_response(struct cabrio_private *priv, u8 resp_idx)
{
    cabrio_dbg_enter(CABRIO_DBG_THREAD);

    if (priv->psstate == PS_STATE_SLEEP)
        priv->psstate = PS_STATE_AWAKE;

    /* Swap buffers by flipping the response index */
    BUG_ON(resp_idx > 1);
    priv->resp_idx = resp_idx;

    wake_up(&priv->waitq);

    cabrio_dbg_leave(CABRIO_DBG_THREAD);
}
EXPORT_SYMBOL_GPL(cabrio_notify_command_response);

/**
 *  cabrio_get_firmware - Retrieves two-stage firmware
 *
 *  @dev:         A pointer to &device structure
 *  @user_mainfw: User-defined main firmware file
 *  @card_model: Bus-specific card model ID used to filter firmware table
 *        elements
 *  @fw_table:    Table of firmware file names and device model numbers
 *        terminated by an entry with a NULL helper name
 *  @mainfw:    On success, the main firmware; caller must free
 *
 *  returns:        0 on success, non-zero on failure
 */
int cabrio_get_firmware(struct device *dev,
            char **user_mainfw, u32 card_model,
            const struct cabrio_fw_table *fw_table,
            const struct firmware **mainfw)
{
    const struct cabrio_fw_table *iter;
    int ret;

    //hBUG_ON(helper == NULL);
    BUG_ON(mainfw == NULL);

    /* Try user-specified firmware first */
    if (*user_mainfw) {
        ret = request_firmware(mainfw, *user_mainfw, dev);
        if (ret) {
            dev_err(dev, "couldn't find main firmware %s\n",
                *user_mainfw);
            goto fail;
        }
        if (*mainfw)
        	return 0;
    }

    /* Otherwise search for firmware to use.  If neither the helper or
     * the main firmware were specified by the user, then we need to
     * make sure that found helper & main are from the same entry in
     * fw_table.
     */
    iter = fw_table;
    while (iter->fwname != NULL) {
        if (iter->model != card_model)
            goto next;
                // SSV --
        if (*mainfw == NULL) {
            ret = request_firmware(mainfw, iter->fwname, dev);
            *user_mainfw = (char *)iter->fwname;
        }

        if (*mainfw)
            return 0;

  next:
        iter++;
    }
  dev_err(dev, "couldn't find main firmware for model %d\n", card_model);

  fail:
    /* Failed */
    if (*mainfw) {
        release_firmware(*mainfw);
        *mainfw = NULL;
    }

    return -ENOENT;
}
EXPORT_SYMBOL_GPL(cabrio_get_firmware);

static int __init cabrio_init_module(void)
{
    cabrio_dbg_enter(CABRIO_DBG_MAIN);
    memset(&confirm_sleep, 0, sizeof(confirm_sleep));
    confirm_sleep.hdr.command = cpu_to_le16(CMD_802_11_PS_MODE);
    confirm_sleep.hdr.size = cpu_to_le16(sizeof(confirm_sleep));
    confirm_sleep.action = cpu_to_le16(PS_MODE_ACTION_SLEEP_CONFIRMED);
    cabrio_debugfs_init();
    cabrio_dbg_leave(CABRIO_DBG_MAIN);
    return 0;
}

static void __exit cabrio_exit_module(void)
{
    cabrio_dbg_enter(CABRIO_DBG_MAIN);
    cabrio_debugfs_remove();
    cabrio_dbg_leave(CABRIO_DBG_MAIN);
}

module_init(cabrio_init_module);
module_exit(cabrio_exit_module);

MODULE_DESCRIPTION("Libertas WLAN Driver Library");
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_LICENSE("GPL");
