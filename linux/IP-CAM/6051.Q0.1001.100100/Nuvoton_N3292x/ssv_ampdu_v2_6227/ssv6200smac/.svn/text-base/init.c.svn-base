

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/nl80211.h>
#include <linux/kthread.h>

#include <ssv6200.h>
#include <hci/hctrl.h>
#include <ssv_version.h>
#include "dev_tbl.h"
#include "dev.h"
#include "lib.h"
#include "ssv_rc.h"
#include "ap.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#include "linux_2_6_35.h"
#endif

#ifdef CONFIG_SSV6XXX_DEBUGFS
#include "ssv6xxx_debugfs.h"
#endif

#ifdef MULTI_THREAD_ENCRYPT
#include <linux/cpu.h>
#include <linux/notifier.h>
#endif 

MODULE_AUTHOR(" South Silicon Valley Microelectronics Co., Ltd");
MODULE_DESCRIPTION("Support for SSV 802.11n wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("SSV 802.11n WLAN cards");
MODULE_LICENSE("Dual BSD/GPL");

#define LBYTESWAP(a)  ((((a) & 0x00ff00ff) << 8) | \
    (((a) & 0xff00ff00) >> 8))

#define LONGSWAP(a)   ((LBYTESWAP(a) << 16) | (LBYTESWAP(a) >> 16))

#define CHAN2G(_freq, _idx)  {      \
    .band = IEEE80211_BAND_2GHZ,    \
    .center_freq = (_freq),         \
    .hw_value = (_idx),             \
    .max_power = 20,                \
}


#define SHPCHECK(__hw_rate, __flags) \
    ((__flags & IEEE80211_RATE_SHORT_PREAMBLE) ? (__hw_rate +3 ) : 0)


#define RATE(_bitrate, _hw_rate, _flags) {      \
    .bitrate        = (_bitrate),               \
    .flags          = (_flags),                 \
    .hw_value       = (_hw_rate),               \
    .hw_value_short = SHPCHECK(_hw_rate,_flags) \
}


/**
* Note that this table maybe modified at run-time. We shall make a copy of 
* this table before using it.
*/
static const struct ieee80211_channel ssv6200_2ghz_chantable[] =
{
    CHAN2G(2412, 1 ), /* Channel 1 */
    CHAN2G(2417, 2 ), /* Channel 2 */
    CHAN2G(2422, 3 ), /* Channel 3 */
    CHAN2G(2427, 4 ), /* Channel 4 */
    CHAN2G(2432, 5 ), /* Channel 5 */
    CHAN2G(2437, 6 ), /* Channel 6 */
    CHAN2G(2442, 7 ), /* Channel 7 */
    CHAN2G(2447, 8 ), /* Channel 8 */
    CHAN2G(2452, 9 ), /* Channel 9 */
    CHAN2G(2457, 10), /* Channel 10 */
    CHAN2G(2462, 11), /* Channel 11 */
    CHAN2G(2467, 12), /* Channel 12 */
    CHAN2G(2472, 13), /* Channel 13 */
    CHAN2G(2484, 14), /* Channel 14 */
};



static struct ieee80211_rate ssv6200_legacy_rates[] =
{
    RATE(10,  0x00, 0),
    RATE(20,  0x01, IEEE80211_RATE_SHORT_PREAMBLE),
    RATE(55,  0x02, IEEE80211_RATE_SHORT_PREAMBLE),
    RATE(110, 0x03, IEEE80211_RATE_SHORT_PREAMBLE),
    RATE(60,  0x07, 0),
    RATE(90,  0x08, 0),
    RATE(120, 0x09, 0),
    RATE(180, 0x0a, 0),
    RATE(240, 0x0b, 0),
    RATE(360, 0x0c, 0),
    RATE(480, 0x0d, 0),
    RATE(540, 0x0e, 0),
};

#ifdef CONFIG_SSV_CABRIO_E
int ssv6xxx_do_iq_calib(struct ssv_hw *sh, struct ssv6xxx_iqk_cfg *p_cfg)
{

    struct sk_buff          *skb;
    struct cfg_host_cmd     *host_cmd;

    printk("# Do init_cali (iq)\n");

    // make command packet
    skb = ssv_skb_alloc(HOST_CMD_HDR_LEN + IQK_CFG_LEN + PHY_SETTING_SIZE + RF_SETTING_SIZE);

    if(skb == NULL)
    {
        printk("init ssv6xxx_do_iq_calib fail!!!\n");
        return 0;
    }

    if((PHY_SETTING_SIZE > MAX_PHY_SETTING_TABLE_SIZE) ||
        (RF_SETTING_SIZE > MAX_RF_SETTING_TABLE_SIZE))
    {
        printk("Please recheck RF or PHY table size!!!\n");
        BUG_ON(1);
        return 0;
    }

    skb->data_len = HOST_CMD_HDR_LEN + IQK_CFG_LEN + PHY_SETTING_SIZE + RF_SETTING_SIZE;
    skb->len      = skb->data_len;

    host_cmd = (struct cfg_host_cmd *)skb->data;

    host_cmd->c_type = HOST_CMD;
    host_cmd->h_cmd  = (u8)SSV6XXX_HOST_CMD_INIT_CALI;
    host_cmd->len    = skb->data_len;

    p_cfg->phy_tbl_size = PHY_SETTING_SIZE;
    p_cfg->rf_tbl_size = RF_SETTING_SIZE;

    memcpy(host_cmd->dat32, p_cfg, IQK_CFG_LEN);

    memcpy(host_cmd->dat8+IQK_CFG_LEN, phy_setting, PHY_SETTING_SIZE);
    memcpy(host_cmd->dat8+IQK_CFG_LEN+PHY_SETTING_SIZE, asic_rf_setting, RF_SETTING_SIZE);

    sh->hci.hci_ops->hci_send_cmd(skb);

    ssv_skb_free(skb);

    mdelay(50);

    return 0;
}
#endif

#define HT_CAP_RX_STBC_ONE_STREAM	0x1

static void ssv6xxx_set_80211_hw_capab(struct ssv_softc *sc)
{
    struct ieee80211_hw *hw=sc->hw;
    struct ssv_hw *sh=sc->sh;
    struct ieee80211_sta_ht_cap *ht_info;

    /* see mac80211.h */
    hw->flags = IEEE80211_HW_SIGNAL_DBM;
    //hw->flags |= IEEE80211_HW_2GHZ_SHORT_SLOT_INCAPABLE;
    //hw->flags |= IEEE80211_HW_2GHZ_SHORT_PREAMBLE_INCAPABLE;
#ifdef CONFIG_SSV_SUPPORT_ANDROID
    hw->flags |= IEEE80211_HW_SUPPORTS_PS;
    hw->flags |= IEEE80211_HW_PS_NULLFUNC_STACK;
#endif        
    /* Set rate control algorithm if enabled*/
    if (sh->cfg.hw_caps & SSV6200_HW_CAP_RC)
        hw->rate_control_algorithm = "ssv6xxx_rate_control";

    /* set HT capability if hardware suppports HT mode */
    ht_info = &sc->sbands[IEEE80211_BAND_2GHZ].ht_cap;

    ampdu_db_log("sh->cfg.hw_caps = 0x%x\n", sh->cfg.hw_caps);
    
    if (sh->cfg.hw_caps & SSV6200_HW_CAP_HT) {
        if (sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_RX) 
        {
            hw->flags |= IEEE80211_HW_AMPDU_AGGREGATION;
            ampdu_db_log("set IEEE80211_HW_AMPDU_AGGREGATION(0x%x)\n", ((hw->flags)&IEEE80211_HW_AMPDU_AGGREGATION));
        }
        
        /*  SM Power Save disabled */
        ht_info->cap = IEEE80211_HT_CAP_SM_PS; 
        
        if (sh->cfg.hw_caps & SSV6200_HW_CAP_GF)
            ht_info->cap |= IEEE80211_HT_CAP_GRN_FLD;

        if (sh->cfg.hw_caps & SSV6200_HW_CAP_STBC)
            ht_info->cap |= HT_CAP_RX_STBC_ONE_STREAM<<IEEE80211_HT_CAP_RX_STBC_SHIFT;
            
        if (sh->cfg.hw_caps & SSV6200_HT_CAP_SGI_20)
            ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
        ht_info->ampdu_factor = IEEE80211_HT_MAX_AMPDU_32K;
        ht_info->ampdu_density = IEEE80211_HT_MPDU_DENSITY_8;
        
        memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));
        ht_info->mcs.rx_mask[0] = 0xff;
        ht_info->mcs.tx_params |= IEEE80211_HT_MCS_TX_DEFINED;
	    ht_info->mcs.rx_highest = cpu_to_le16(SSV6200_RX_HIGHEST_RATE);
        ht_info->ht_supported = true;
    }

    hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    if (sh->cfg.hw_caps & SSV6200_HW_CAP_P2P) {
        hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_P2P_CLIENT);
        hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_P2P_GO);
    }
#endif

    if (sh->cfg.hw_caps & SSV6200_HW_CAP_AP)
        hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_AP);

    hw->queues = 4;
	hw->max_rates = 4;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)	
	hw->channel_change_time = 5000;
#endif	
	hw->max_listen_interval = 1;
	hw->max_rate_tries = HW_MAX_RATE_TRIES;
    hw->extra_tx_headroom = sh->cfg.txpb_offset + SSV_SKB_info_size;

    if (sh->cfg.hw_caps & SSV6200_HW_CAP_2GHZ) {
		hw->wiphy->bands[IEEE80211_BAND_2GHZ] =
			&sc->sbands[IEEE80211_BAND_2GHZ];
    }

    SET_IEEE80211_PERM_ADDR(hw, sh->cfg.maddr);
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    /* Set rate control algorithm if enabled*/
    if (sh->cfg.hw_caps & SSV6200_HW_CAP_AMPDU_TX)
        #ifdef PREFER_RX
        hw->max_rx_aggregation_subframes = 64; //SSV_AMPDU_TX_SUBFRAME_NUM;
        #else // PREFER_RX
        hw->max_rx_aggregation_subframes = 16; //SSV_AMPDU_RX_SUBFRAME_NUM;
        #endif // PREFER_RX
    else
        hw->max_rx_aggregation_subframes = 12; //SSV_AMPDU_RX_SUBFRAME_NUM;

    hw->max_tx_aggregation_subframes = 64; //SSV_AMPDU_TX_SUBFRAME_NUM;
#endif
    
    hw->sta_data_size = sizeof(struct ssv_sta_priv_data); /* drv_priv sizeof of struct ieee80211_sta */
    hw->vif_data_size = sizeof(struct ssv_vif_priv_data); /* drv_priv sizeof of struct ieee80211_vif */
}

#ifdef MULTI_THREAD_ENCRYPT
/*
char * CPU_STATUS[] =
{
    "Online",
    "Up prepare",
    "Up canceled",
    "Down prepare",
    "Down Failed",
    "Dead",
    "Dying",
    "Post dead",
    "Starting",
};

#define show_cpu_status(hotcpu,action)                                                                  \
({                                                                                                      \
        printk("cpu %d is %s, Frozen is %ld\n", hotcpu, CPU_STATUS[(action-2)], (action&0x00F0));       \
})
*/
extern struct list_head encrypt_task_head;

int ssv6xxx_cpu_callback(struct notifier_block *nfb,
                                  unsigned long action,
                                  void *hcpu)
{
    int hotcpu = (unsigned long)hcpu;
    struct ssv_encrypt_task_list *ta = NULL;

    //show_cpu_status(hotcpu, action);
    switch (action) {
    case CPU_UP_PREPARE:
    case CPU_UP_PREPARE_FROZEN:
    case CPU_DOWN_FAILED: {
            int cpu = 0;

            list_for_each_entry(ta, &encrypt_task_head, list)
            {
                if(cpu == hotcpu)
                    break;
                cpu++;
            }
            if(ta->encrypt_task->state & TASK_UNINTERRUPTIBLE)
            {
                kthread_bind(ta->encrypt_task, hotcpu);
            }
            printk("encrypt_task %p state is %ld\n", ta->encrypt_task, ta->encrypt_task->state);

            break;
        }
    case CPU_ONLINE:
    case CPU_ONLINE_FROZEN: {
            int cpu = 0;
            list_for_each_entry(ta, &encrypt_task_head, list)
            {
                if(cpu == hotcpu)
                {
                    ta->cpu_offline = 0;
                    if ( (ta->started == 0) && (cpu_online(cpu)) )
                    {
                        wake_up_process(ta->encrypt_task);
                        ta->started = 1;
                        printk("wake up encrypt_task %p state is %ld, cpu = %d\n", ta->encrypt_task, ta->encrypt_task->state, cpu);
                    }
                    break;
                }
                cpu++;
            }
            printk("encrypt_task %p state is %ld\n", ta->encrypt_task, ta->encrypt_task->state);
            break;
        }
#ifdef CONFIG_HOTPLUG_CPU
    case CPU_UP_CANCELED:
    case CPU_UP_CANCELED_FROZEN:
            //break;
    case CPU_DOWN_PREPARE: {
            int cpu = 0;
            list_for_each_entry(ta, &encrypt_task_head, list)
            {
                if(cpu == hotcpu)
                {
                    ta->cpu_offline = 1;
                    break;
                }
                cpu++;
            }

            printk("p = %p\n",ta->encrypt_task);
            break;
        }
    case CPU_DEAD:
    case CPU_DEAD_FROZEN: {
            break;
        }
#endif /* CONFIG_HOTPLUG_CPU */
    }
    return NOTIFY_OK;
}

static struct notifier_block cpu_nfb = {
    .notifier_call = ssv6xxx_cpu_callback
};

#endif



static int ssv6xxx_init_softc(struct ssv_softc *sc)
{
//    struct ssv_hw *sh;
    void *channels;
    int ret=0;
#ifdef MULTI_THREAD_ENCRYPT        
    unsigned int cpu;
#endif

    sc->sc_flags = SC_OP_INVALID;
    sc->if_type = NL80211_IFTYPE_UNSPECIFIED;
    mutex_init(&sc->mutex);
    mutex_init(&sc->mem_mutex);

    sc->config_wq= create_singlethread_workqueue("ssv6xxx_cong_wq");
    
    INIT_WORK(&sc->set_tim_work, ssv6200_set_tim_work);
    INIT_WORK(&sc->bcast_start_work, ssv6200_bcast_start_work);
    INIT_DELAYED_WORK(&sc->bcast_stop_work, ssv6200_bcast_stop_work);
    INIT_DELAYED_WORK(&sc->bcast_tx_work, ssv6200_bcast_tx_work);
    INIT_WORK(&sc->set_ampdu_rx_add_work, ssv6xxx_set_ampdu_rx_add_work);
    INIT_WORK(&sc->set_ampdu_rx_del_work, ssv6xxx_set_ampdu_rx_del_work);
    INIT_WORK(&sc->set_ps_work, ssv6200_set_ps_work);

#ifdef CONFIG_SSV_SUPPORT_ANDROID
    sc->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 20;
    sc->early_suspend.suspend = ssv6xxx_early_suspend;
    sc->early_suspend.resume = ssv6xxx_late_resume;
    register_early_suspend(&sc->early_suspend);
    sc->bStaPS = false;
    //wake_lock_init(&sc->wifi_alive_lock, WAKE_LOCK_SUSPEND, "wifi alive");
#endif    
	//use to send mcast frame.
//	init_timer(&sc->bcast_timeout);	
//	sc->bcast_timeout.data = (unsigned long)sc;
//	sc->bcast_timeout.function = ssv6200_bcast_timer;
	

    /* By default, we apply staion decion table. */
    sc->mac_deci_tbl = sta_deci_tbl;
    
    /**
        * Initialize tx: 
        * Associate WMM AC queue to each hardward tx queue.
        * For hardware queue, queue 0 has the lowest priority.
        */
    memset((void *)&sc->tx, 0, sizeof(struct ssv_tx));
    sc->tx.hw_txqid[WMM_AC_VO] = 3; sc->tx.ac_txqid[3] = WMM_AC_VO;
    sc->tx.hw_txqid[WMM_AC_VI] = 2; sc->tx.ac_txqid[2] = WMM_AC_VI;
    sc->tx.hw_txqid[WMM_AC_BE] = 1; sc->tx.ac_txqid[1] = WMM_AC_BE;
    sc->tx.hw_txqid[WMM_AC_BK] = 0; sc->tx.ac_txqid[0] = WMM_AC_BK;   

    // init ampdu tx queue ==============================
    INIT_LIST_HEAD(&sc->tx.ampdu_tx_que);
    // init spinlock ===================================
    spin_lock_init(&sc->tx.ampdu_tx_que_lock);

    /**
        * Initialize rx: 
        * Allocate a dedicate skb for receiving data from interface (SDIO).
        */
    memset((void *)&sc->rx, 0, sizeof(struct ssv_rx));
    spin_lock_init(&sc->rx.rxq_lock);
    skb_queue_head_init(&sc->rx.rxq_head);
    sc->rx.rx_buf = ssv_skb_alloc(MAX_FRAME_SIZE);
    if (sc->rx.rx_buf == NULL)
        return -ENOMEM;

	/* Initialize broadcast queue */
	memset(&sc->bcast_txq, 0, sizeof(struct ssv6xxx_bcast_txq));
	spin_lock_init(&sc->bcast_txq.txq_lock);
	skb_queue_head_init(&sc->bcast_txq.qhead);

	/* Initialize power saver spin lock */
	spin_lock_init(&sc->ps_state_lock);
	
    /* Initialize channels & rates */
	if (sc->sh->cfg.hw_caps & SSV6200_HW_CAP_2GHZ) {
        /**
                * Make a copy of the channel table before using it because it maybe
                * modified at run-time. Be aware that the duplicate one shall be freed
                * at driver deinit time.
                */
        channels = kmemdup(ssv6200_2ghz_chantable,
			sizeof(ssv6200_2ghz_chantable), GFP_KERNEL);
		if (!channels) {
            kfree(sc->rx.rx_buf);
		    return -ENOMEM;
        }
        
		sc->sbands[IEEE80211_BAND_2GHZ].channels = channels;
		sc->sbands[IEEE80211_BAND_2GHZ].band = IEEE80211_BAND_2GHZ;
		sc->sbands[IEEE80211_BAND_2GHZ].n_channels =
			ARRAY_SIZE(ssv6200_2ghz_chantable);
		sc->sbands[IEEE80211_BAND_2GHZ].bitrates = ssv6200_legacy_rates;
		sc->sbands[IEEE80211_BAND_2GHZ].n_bitrates =
			ARRAY_SIZE(ssv6200_legacy_rates);
	}

    /* Set ssv6200 hardware capabilities to mac80211 stack */
    ssv6xxx_set_80211_hw_capab(sc);

    /* register rate control algorithm */
    ret = ssv6xxx_rate_control_register();
    if (ret != 0) {
        printk("%s(): Failed to register rc algorithm.\n", __FUNCTION__);
    }

    /* !!!!!!!!!!!!!!!!!!!!!!!! NOTE: Temporarily solution !!!!!!!!!!!!!!!!!!!!!!!! */
    //sc->sc_flags |= SC_OP_FIXED_RATE;
    //sc->max_rate_idx = 22;
    
#ifdef MULTI_THREAD_ENCRYPT    
    //encrypt threads and related members

    skb_queue_head_init(&sc->preprocess_q);
    spin_lock_init(&sc->encrypt_st_lock);
    INIT_LIST_HEAD(&encrypt_task_head);
    INIT_LIST_HEAD(&sc->encrypt_st_head);
    INIT_LIST_HEAD(&sc->encrypt_st_empty);
    
    sc->encrypt_st_cnt = 0;
    sc->empty_encrypt_st_cnt = 0;
    
    for_each_cpu(cpu, cpu_present_mask) 
    {
        struct ssv_encrypt_task_list *ta = kzalloc(sizeof(*ta), GFP_KERNEL);
		memset(ta, 0, sizeof(*ta));
        ta->encrypt_task = kthread_create_on_node(ssv6xxx_encrypt_task, sc, cpu_to_node(cpu), "%d/ssv6xxx_encrypt_task", cpu);
        init_waitqueue_head(&ta->encrypt_wait_q);

        if (!IS_ERR(ta->encrypt_task))
        {
            printk("[MT-ENCRYPT]: create kthread %p for CPU %d, ret = %d\n", ta->encrypt_task, cpu, ret);
            kthread_bind(ta->encrypt_task, cpu);
            list_add_tail(&ta->list, &encrypt_task_head);
            if (cpu_online(cpu))
            {
            	wake_up_process(ta->encrypt_task);
                ta->started = 1;
            }
        }
        else
        {
            printk("[MT-ENCRYPT]: Fail to create kthread\n");
        }
    }
    register_cpu_notifier(&cpu_nfb); 
#endif //#ifdef MULTI_THREAD_ENCRYPT  

    // Wait queue for TX/RX
    init_waitqueue_head(&sc->tx_wait_q);
    sc->tx_wait_q_woken = 0;
    skb_queue_head_init(&sc->tx_skb_q);
    sc->tx_task = kthread_run(ssv6xxx_tx_task, sc, "ssv6xxx_tx_task");
    sc->tx_q_empty = false;
    skb_queue_head_init(&sc->tx_done_q);

    // Wait queue for TX/RX
    init_waitqueue_head(&sc->rx_wait_q);
    sc->rx_wait_q_woken = 0;
    skb_queue_head_init(&sc->rx_skb_q);
    sc->rx_task = kthread_run(ssv6xxx_rx_task, sc, "ssv6xxx_rx_task");
        
    return ret;
}


static int ssv6xxx_deinit_softc(struct ssv_softc *sc)
{
    void *channels;
	struct sk_buff* skb;
    u8 remain_size;
#ifdef MULTI_THREAD_ENCRYPT        
    struct ssv_encrypt_task_list *qtask = NULL;
    struct ssv_encrypt_st_list *qst = NULL;
    int counter = 0;
#endif
    
    printk("%s():\n", __FUNCTION__);
    if (sc->sh->cfg.hw_caps & SSV6200_HW_CAP_2GHZ) {
        channels = sc->sbands[IEEE80211_BAND_2GHZ].channels;
        kfree(channels);
    }
#ifdef CONFIG_SSV_SUPPORT_ANDROID    
    unregister_early_suspend(&sc->early_suspend);
    if(sc->bStaPS == true){
        wake_lock_destroy(&sc->wifi_alive_lock);
    }
#endif
    ssv_skb_free(sc->rx.rx_buf);
   
    sc->rx.rx_buf = NULL;
    ssv6xxx_rate_control_unregister();
//    del_timer_sync(&sc->bcast_timeout);    
    cancel_delayed_work_sync(&sc->bcast_tx_work);
    
	/* Release broadcast frames. */
	do{
		skb = ssv6200_bcast_dequeue(&sc->bcast_txq, &remain_size);
		if(skb)
            ssv6xxx_txbuf_free_skb(skb, (void*)sc);//dev_kfree_skb_any(skb);
		else
			break;
	}while(remain_size);

    if (sc->tx_task != NULL)
    {
        printk("Stopping TX task...\n");
        kthread_stop(sc->tx_task);
        printk("Stopped TX task.\n");        
    }
    if (sc->tx_task != NULL)
    {
        printk("Stopping RX task...\n");
        kthread_stop(sc->rx_task);
        printk("Stopped RX task.\n");    
    }

#ifdef MULTI_THREAD_ENCRYPT        
    unregister_cpu_notifier(&cpu_nfb); 
    if(!list_empty(&encrypt_task_head))
    {        
        for(qtask = list_entry((&encrypt_task_head)->next, typeof(*qtask), list);
                !list_empty(&encrypt_task_head);
                qtask = list_entry((&encrypt_task_head)->next, typeof(*qtask), list))
        {
            counter++;
            printk("Stopping encrypt task %d: ...\n", counter);
            kthread_stop(qtask->encrypt_task);
            printk("Stopped encrypt task %d: ...\n", counter);
            list_del(&qtask->list);
            kfree(qtask);
        }        
    }
    
    printk("[MT-ENCRYPT]: free encrypt_st_empty\n");
    counter = 0;
    if(!list_empty(&sc->encrypt_st_empty))
    {
         for(qst = list_entry((&sc->encrypt_st_empty)->next, typeof(*qst), list);
                !list_empty(&sc->encrypt_st_empty);
                qst = list_entry((&sc->encrypt_st_empty)->next, typeof(*qst), list))
        {
            printk("[MT-ENCRYPT]: empty st item %d\n", counter);
            list_del(&qst->list);
            if(qst->skb_ptr != NULL)
                printk("[MT-ENCRYPT]: empty st item has skb ptr!!\n");
            kfree(qst);
        }
    }
    
    printk("[MT-ENCRYPT]: free encrypt_st_head\n");
    counter = 0;
    if(!list_empty(&sc->encrypt_st_head))
    {
        qst = NULL;
        for(qst = list_entry((&sc->encrypt_st_head)->next, typeof(*qst), list);
                !list_empty(&sc->encrypt_st_head);
                qst = list_entry((&sc->encrypt_st_head)->next, typeof(*qst), list))
        {
            printk("[MT-ENCRYPT]: encrypt st item %d\n", counter);
            list_del(&qst->list);
            if(qst->skb_ptr != NULL)
                printk("[MT-ENCRYPT]: skb is still waiting to be encrypted!!\n");
            kfree(qst);
        }
    }
    
    printk("[MT-ENCRYPT]: end of de-init\n");
#endif //#ifdef MULTI_THREAD_ENCRYPT    
    
    destroy_workqueue(sc->config_wq);

    return 0;
}

static void ssv6xxx_hw_set_replay_ignore(struct ssv_hw *sh,u8 ignore)
{
    u32 temp;
    SMAC_REG_READ(sh,ADR_SCRT_SET,&temp);
    temp = temp & SCRT_RPLY_IGNORE_I_MSK;
    temp |= (ignore << SCRT_RPLY_IGNORE_SFT);
    SMAC_REG_WRITE(sh,ADR_SCRT_SET, temp);
}


int ssv6xxx_init_mac(struct ssv_hw *sh)
{
    struct ssv_softc *sc=sh->sc;
    int i, ret=0;
    u32 *ptr, id_len, regval;
	char    chip_id[24]="";
	u64     chip_tag=0;

    //CHIP TAG
    SMAC_REG_READ(sh, ADR_IC_TIME_TAG_1, &regval);
    chip_tag = ((u64)regval<<32);
    SMAC_REG_READ(sh, ADR_IC_TIME_TAG_0, &regval);
    chip_tag |= (regval);
    printk(KERN_INFO "CHIP TAG: %llx \n",chip_tag);
    //CHIP ID
    SMAC_REG_READ(sh, ADR_CHIP_ID_3, &regval);
    *((u32 *)&chip_id[0]) = (u32)LONGSWAP(regval);
    SMAC_REG_READ(sh, ADR_CHIP_ID_2, &regval);
    *((u32 *)&chip_id[4]) = (u32)LONGSWAP(regval);
    SMAC_REG_READ(sh, ADR_CHIP_ID_1, &regval);
    *((u32 *)&chip_id[8]) = (u32)LONGSWAP(regval);
    SMAC_REG_READ(sh, ADR_CHIP_ID_0, &regval);
    *((u32 *)&chip_id[12]) = (u32)LONGSWAP(regval);
    printk(KERN_INFO "CHIP ID: %s \n",chip_id);
    //Firmware version
    regval = sizeof(files_modified);
    regval = sizeof(files_otherversion);
    printk(KERN_INFO "Firmware version %d\n",ssv_root_version);

    if(sc->ps_status == PWRSV_ENABLE){
#ifdef CONFIG_SSV_SUPPORT_ANDROID    
        printk(KERN_INFO "%s: wifi Alive lock!,bStaPS=%d\n",__FUNCTION__,sc->bStaPS);
        if(sc->bStaPS == true){
            //wake_lock_init(&sc->wifi_alive_lock, WAKE_LOCK_SUSPEND, "wifi alive");
            wake_lock_timeout(&sc->wifi_alive_lock,msecs_to_jiffies(20000));
            printk(KERN_INFO "wifi Alive lock!\n");
        }
#endif
#ifdef CONFIG_SSV_HW_ENCRYPT_SW_DECRYPT        
        SMAC_REG_WRITE(sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_HWHCI<<4));
#else
        SMAC_REG_WRITE(sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_ENCRYPT_SEC<<4)|(M_ENG_HWHCI<<8));
#endif
        SMAC_REG_WRITE(sc->sh, ADR_RX_FLOW_MNG,  M_ENG_MACRX|(M_ENG_HWHCI<<4));
#if Enable_AMPDU_FW_Retry
        SMAC_REG_WRITE(sh, ADR_RX_FLOW_CTRL, M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8));
#else
        SMAC_REG_WRITE(sh, ADR_RX_FLOW_CTRL, M_ENG_MACRX|(M_ENG_HWHCI<<4));
#endif
        SMAC_REG_WRITE(sc->sh, ADR_MRX_FLT_TB0+6*4, (sc->mac_deci_tbl[6]));   //ACTION_DO_NOTHING, FRAME_ACCEPT 0x11F8  /* 0 001000 111111 00 1 */       //Beacon
        return ret;
    }
    
    /* reset ssv6200 mac */
    SMAC_REG_WRITE(sh, ADR_BRG_SW_RST, 1 << 1);  /* bug if reset ?? */
    SMAC_REG_WRITE(sh, ADR_BOOT, 1 << 0);        /*??????????????????*/

	//HCI-enable 4 bits
	//tx rx packet debug counter enable(bit28) 
	//RX to Host(bit2)
	//TX on_demand interrupt control between allocate size and transmit data mismatch
	//Fill tx queue info in rx pacet(Bit 25)
    SMAC_REG_WRITE(sh, ADR_CONTROL, 0x12000006); /*??????????????????*/

	/* Set wmm parameter to EDCA Q4
	(use to send mgmt frame/null data frame in STA mode and broadcast frame in AP mode) */
	SMAC_REG_WRITE(sc->sh, ADR_TXQ4_MTX_Q_AIFSN, 0xffff2101);

    /* Enable hardware timestamp for TSF */
    // 28 => time stamp write location
    SMAC_REG_WRITE(sh, ADR_RX_TIME_STAMP_CFG, ((28<<MRX_STP_OFST_SFT)|0x01));
    
    SMAC_REG_WRITE(sh, ADR_HCI_TX_RX_INFO_SIZE,
          ((u32)(sh->cfg.txpb_offset) << TX_PBOFFSET_SFT) |    /* packet buffer offset for tx */
          ((u32)(sh->tx_desc_len)  << TX_INFO_SIZE_SFT) |   /* tx_info_size send (bytes) (need word boundry, times of 4bytes) */
          ((u32)(sh->rx_desc_len)  << RX_INFO_SIZE_SFT) |   /* rx_info_size send (bytes) (need word boundry, times of 4bytes) */
          ((u32)(sh->rx_pinfo_pad) << RX_LAST_PHY_SIZE_SFT )    /* rx_last_phy_size send (bytes) */
    );
    SMAC_REG_WRITE(sh, ADR_GLBLE_SET,
          (0 << OP_MODE_SFT)  |                           /* STA mode by default */
          (0 << SNIFFER_MODE_SFT) |                           /* disable sniffer mode */
          (1 << DUP_FLT_SFT) |                           /* Enable duplicate detection */
          ((u32)(sh->cfg.rxpb_offset) << PB_OFFSET_SFT)         /* set rx packet buffer offset */
    );

    /* Setting MMU to 256 pages */
    SMAC_REG_READ(sh,ADR_MMU_CTRL, &regval);
	regval |= (0xff<<MMU_SHARE_MCU_SFT);
    SMAC_REG_WRITE(sh,ADR_MMU_CTRL, regval);


    /**
        * Tx/RX threshold setting for packet buffer resource. 
        */
    SMAC_REG_READ(sh, ADR_TRX_ID_THRESHOLD, &id_len);
    id_len = (id_len&0xffff0000 ) |
             (SSV6200_ID_TX_THRESHOLD<<TX_ID_THOLD_SFT)|
             (SSV6200_ID_RX_THRESHOLD<<RX_ID_THOLD_SFT);
    SMAC_REG_WRITE(sh, ADR_TRX_ID_THRESHOLD, id_len);    

    SMAC_REG_READ(sh, ADR_ID_LEN_THREADSHOLD1, &id_len);
    id_len = (id_len&0x0f )|
             (SSV6200_PAGE_TX_THRESHOLD<<ID_TX_LEN_THOLD_SFT)|
             (SSV6200_PAGE_RX_THRESHOLD<<ID_RX_LEN_THOLD_SFT);
    SMAC_REG_WRITE(sh, ADR_ID_LEN_THREADSHOLD1, id_len);


#ifdef CONFIG_SSV_CABRIO_MB_DEBUG
	//SRAM address 0
	SMAC_REG_READ(sh, ADR_MB_DBG_CFG3, &regval);
    regval |= (debug_buffer<<0);
    SMAC_REG_WRITE(sh, ADR_MB_DBG_CFG3, regval);

	SMAC_REG_READ(sh, ADR_MB_DBG_CFG2, &regval);
    regval |= (DEBUG_SIZE<<16);
    SMAC_REG_WRITE(sh, ADR_MB_DBG_CFG2, regval);

	//enable mailbox debug
    SMAC_REG_READ(sh, ADR_MB_DBG_CFG1, &regval);
    regval |= (1<<MB_DBG_EN_SFT);
    SMAC_REG_WRITE(sh, ADR_MB_DBG_CFG1, regval);
    SMAC_REG_READ(sh, ADR_MBOX_HALT_CFG, &regval);
    regval |= (1<<MB_ERR_AUTO_HALT_EN_SFT);
    SMAC_REG_WRITE(sh, ADR_MBOX_HALT_CFG, regval);
#endif

#ifdef SSV6200_ECO
    SMAC_REG_WRITE(sh, 0xcd010004, 0x1213);
    /**
        * Allocate a hardware packet buffer space. This buffer is for security
        * key caching and phy info space.
        */
    for(i=0;i<SSV_RC_MAX_STA;i++)
    {
        if(i==0)
        {
            sh->hw_buf_ptr[i] = ssv6xxx_pbuf_alloc(sc, sizeof(phy_info_tbl)+
                                                    sizeof(struct ssv6xxx_hw_sec));
	        if((sh->hw_buf_ptr[i]>>28) != 8)
	        {
		        //asic pbuf address start from 0x8xxxxxxxx
		        printk("opps allocate pbuf error\n");
		        WARN_ON(1);	
		        ret = 1;
		        goto exit;
	        }
        }
        else
        {
            sh->hw_buf_ptr[i] = ssv6xxx_pbuf_alloc(sc, sizeof(struct ssv6xxx_hw_sec));
            if((sh->hw_buf_ptr[i]>>28) != 8)
            {
                //asic pbuf address start from 0x8xxxxxxxx
                printk("opps allocate pbuf error\n");
                WARN_ON(1); 
                ret = 1;
                goto exit;
            }
        }
        printk("%s(): ssv6200 reserved space=0x%08x\n", 
            __FUNCTION__, sh->hw_buf_ptr[i]);
    }
#else // SSV6200_ECO
    /**
        * Allocate a hardware packet buffer space. This buffer is for security
        * key caching and phy info space.
        */
    sh->hw_buf_ptr = ssv6xxx_pbuf_alloc(sc, sizeof(phy_info_tbl)+
                                            sizeof(struct ssv6xxx_hw_sec));
	if((sh->hw_buf_ptr>>28) != 8)
	{
		//asic pbuf address start from 0x8xxxxxxxx
		printk("opps allocate pbuf error\n");
		WARN_ON(1);	
		ret = 1;
		goto exit;
	}
	
    printk("%s(): ssv6200 reserved space=0x%08x, size=%d\n", 
        __FUNCTION__, sh->hw_buf_ptr, (u32)(sizeof(phy_info_tbl)+sizeof(struct ssv6xxx_hw_sec)));

#endif // SSV6200_ECO
	//BUG_ON(SSV_HW_RESERVE_SIZE < (sizeof(struct ssv6xxx_hw_sec)+sizeof(phy_info_tbl)-PHY_INFO_TBL1_SIZE*4+4));


/**	
Part 1. SRAM
	**********************
	*				          * 
	*	1. Security key table *
	* 				          *
	* *********************
	*				          *
	*    	2. PHY index table     *
	* 				          *
	* *********************
	* 				          *
	*	3. PHY ll-length table * 
	*				          *
	* *********************	
=============================================	
Part 2. Register     
	**********************
	*				          * 
	*	PHY Infor Table         *
	* 				          *
	* *********************
*
*/

#ifdef SSV6200_ECO
    /**
        * Init ssv6200 hardware security table: clean the table.
        * And set PKT_ID for hardware security.
        */
    for(i=0;i<SSV_RC_MAX_STA;i++)
        sh->hw_sec_key[i] = sh->hw_buf_ptr[i];

    for(i=0;i<SSV_RC_MAX_STA;i++)
    {
		int x;
	    //==>Section 1. Write Sec table to SRAM
        for(x=0; x<sizeof(struct ssv6xxx_hw_sec); x+=4) {
            SMAC_REG_WRITE(sh, sh->hw_sec_key[i]+x, 0);
        }
    }

    SMAC_REG_READ(sh, ADR_SCRT_SET, &regval);
	regval &= SCRT_PKT_ID_I_MSK;
	regval |= ((sh->hw_sec_key[0] >> 16) << SCRT_PKT_ID_SFT);
	SMAC_REG_WRITE(sh, ADR_SCRT_SET, regval);


    /**
        * Set default ssv6200 phy infomation table.
        */
    sh->hw_pinfo = sh->hw_sec_key[0] + sizeof(struct ssv6xxx_hw_sec);
    for(i=0, ptr=phy_info_tbl; i<PHY_INFO_TBL1_SIZE; i++, ptr++) {
        SMAC_REG_WRITE(sh, ADR_INFO0+i*4, *ptr);
        SMAC_REG_CONFIRM(sh, ADR_INFO0+i*4, *ptr);
    }	
#else // SSV6200_ECO
    /**
        * Init ssv6200 hardware security table: clean the table.
        * And set PKT_ID for hardware security.
        */
    sh->hw_sec_key = sh->hw_buf_ptr;
	
	//==>Section 1. Write Sec table to SRAM
    for(i=0; i<sizeof(struct ssv6xxx_hw_sec); i+=4) {
        SMAC_REG_WRITE(sh, sh->hw_sec_key+i, 0);
    }
    SMAC_REG_READ(sh, ADR_SCRT_SET, &regval);
	regval &= SCRT_PKT_ID_I_MSK;
	regval |= ((sh->hw_sec_key >> 16) << SCRT_PKT_ID_SFT);
	SMAC_REG_WRITE(sh, ADR_SCRT_SET, regval);


    /**
        * Set default ssv6200 phy infomation table.
        */
    sh->hw_pinfo = sh->hw_sec_key + sizeof(struct ssv6xxx_hw_sec);
    for(i=0, ptr=phy_info_tbl; i<PHY_INFO_TBL1_SIZE; i++, ptr++) {
        SMAC_REG_WRITE(sh, ADR_INFO0+i*4, *ptr);
        SMAC_REG_CONFIRM(sh, ADR_INFO0+i*4, *ptr);
    }	
#endif // SSV6200_ECO
	
	//==>Section 2. Write PHY index table and PHY ll-length table to SRAM
	for(i=0; i<PHY_INFO_TBL2_SIZE; i++, ptr++) {
        SMAC_REG_WRITE(sh, sh->hw_pinfo+i*4, *ptr);
        SMAC_REG_CONFIRM(sh, sh->hw_pinfo+i*4, *ptr);
    }
    for(i=0; i<PHY_INFO_TBL3_SIZE; i++, ptr++) {
        SMAC_REG_WRITE(sh, sh->hw_pinfo+
        (PHY_INFO_TBL2_SIZE<<2)+i*4, *ptr);
        SMAC_REG_CONFIRM(sh, sh->hw_pinfo+
        (PHY_INFO_TBL2_SIZE<<2)+i*4, *ptr);
    }


    SMAC_REG_WRITE(sh, ADR_INFO_RATE_OFFSET, 0x00040000);
	
	//Set SRAM address to register
	SMAC_REG_WRITE(sh, ADR_INFO_IDX_ADDR, sh->hw_pinfo);
    SMAC_REG_WRITE(sh, ADR_INFO_LEN_ADDR, sh->hw_pinfo+(PHY_INFO_TBL2_SIZE)*4);

	printk("ADR_INFO_IDX_ADDR[%08x] ADR_INFO_LEN_ADDR[%08x]\n", sh->hw_pinfo, sh->hw_pinfo+(PHY_INFO_TBL2_SIZE)*4);    
    /**
        * Set ssv6200 mac address and set default BSSID. In hardware reset,
        * we the BSSID to 00:00:00:00:00:00.
        */
    SMAC_REG_WRITE(sh, ADR_STA_MAC_0, *((u32 *)&sh->cfg.maddr[0]));
    SMAC_REG_WRITE(sh, ADR_STA_MAC_1, *((u32 *)&sh->cfg.maddr[4]));
    SMAC_REG_WRITE(sh, ADR_BSSID_0,   *((u32 *)&sc->bssid[0]));
    SMAC_REG_WRITE(sh, ADR_BSSID_1,   *((u32 *)&sc->bssid[4]));

    /*
        * Set tx interrupt threshold for EACA0 ~ EACA3 queue & low threshold
        */
    // Freddie ToDo: Use resource low instead of id threshold as interrupt source to reduce unecessary
    //Inital soc interrupt Bit13-Bit16(EDCA 0-3) Bit28(Tx resource low)
    SMAC_REG_WRITE(sh, ADR_SDIO_MASK, 0xfffe1fff);

//#define SSV6200_ID_AC_BK_OUT_QUEUE          8
//#define SSV6200_ID_AC_BE_OUT_QUEUE          15
//#define SSV6200_ID_AC_VI_OUT_QUEUE          16
//#define SSV6200_ID_AC_VO_OUT_QUEUE          16
//#define SSV6200_ID_MANAGER_QUEUE            8


#ifdef CONFIG_SSV_TX_LOWTHRESHOLD
    //Setting Tx resource low ---ID[0x5] Page[0x15]
    SMAC_REG_WRITE(sh, ADR_TX_LIMIT_INTR, 0x80020045);
#else
    //Enable EDCA low threshold 
    SMAC_REG_WRITE(sh, ADR_MB_THRESHOLD6, 0x80000000);
    //Enable EDCA low threshold EDCA-1[8] EDCA-0[4] 
    SMAC_REG_WRITE(sh, ADR_MB_THRESHOLD8, 0x04020000);
    //Enable EDCA low threshold EDCA-3[8] EDCA-2[8] 
    SMAC_REG_WRITE(sh, ADR_MB_THRESHOLD9, 0x00000404);
#endif

    /**
        * Disable tx/rx ether trap table.
        */
    SMAC_REG_WRITE(sh, ADR_TX_ETHER_TYPE_0, 0x00000000);
    SMAC_REG_WRITE(sh, ADR_TX_ETHER_TYPE_1, 0x00000000);
    SMAC_REG_WRITE(sh, ADR_RX_ETHER_TYPE_0, 0x00000000);    
    SMAC_REG_WRITE(sh, ADR_RX_ETHER_TYPE_1, 0x00000000);
    

    /**
        * Set reason trap to discard frames.
        */
    SMAC_REG_WRITE(sh, ADR_REASON_TRAP0, 0x7FBC7F8F);
    SMAC_REG_WRITE(sh, ADR_REASON_TRAP1, 0x00000000);
#ifndef FW_WSID_WATCH_LIST
    SMAC_REG_WRITE(sh, ADR_TRAP_HW_ID, M_ENG_HWHCI); /* Trap to HCI */
#else
    SMAC_REG_WRITE(sh, ADR_TRAP_HW_ID, M_ENG_CPU); /* Trap to CPU */
#endif


    /**
        * Reset all wsid table entry to invalid.
        */
    SMAC_REG_WRITE(sh, ADR_WSID0, 0x00000000);
    SMAC_REG_WRITE(sh, ADR_WSID1, 0x00000000);
    

    /**
        * Reset multicast filter table to accept all.
        */


    /**
        * Set Tx/Rx processing flows.
        */
#ifdef CONFIG_SSV_HW_ENCRYPT_SW_DECRYPT        
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_HWHCI<<4));
#else
#ifdef HCI_RX_AGGR
    sc->rx_data_mcu = true;
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_ENCRYPT_SEC<<4)|(M_ENG_CPU<<8)|(M_ENG_HWHCI<<12));
#else    
    sc->rx_data_mcu = false;
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_DATA, M_ENG_MACRX|(M_ENG_ENCRYPT_SEC<<4)|(M_ENG_HWHCI<<8));
#endif
#endif
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_MNG,  M_ENG_MACRX|(M_ENG_HWHCI<<4));
    #if Enable_AMPDU_FW_Retry
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_CTRL, M_ENG_MACRX|(M_ENG_CPU<<4)|(M_ENG_HWHCI<<8));
    #else
    SMAC_REG_WRITE(sh, ADR_RX_FLOW_CTRL, M_ENG_MACRX|(M_ENG_HWHCI<<4));
    #endif

    ssv6xxx_hw_set_replay_ignore(sh, 1); //?? move to init_mac

#ifdef CONFIG_SSV_SUPPORT_BTCX
        /* Set BTCX Parameter*/
        SMAC_REG_WRITE(sh, ADR_BTCX0,COEXIST_EN_MSK|(WIRE_MODE_SZ<<WIRE_MODE_SFT)
                           |WIFI_TX_SW_POL_MSK | BT_SW_POL_MSK);    
        SMAC_REG_WRITE(sh, ADR_BTCX1,SSV6200_BT_PRI_SMP_TIME|(SSV6200_BT_STA_SMP_TIME<<BT_STA_SMP_TIME_SFT)
                           |(SSV6200_WLAN_REMAIN_TIME<<WLAN_REMAIN_TIME_SFT));
        SMAC_REG_WRITE(sh, ADR_SWITCH_CTL,BT_2WIRE_EN_MSK);
    
        /*WIFI_TX_SW_O*/
        //SMAC_REG_READ(sh, ADR_PAD7, &regval);
        //regval |= 1;
        SMAC_REG_WRITE(sh, ADR_PAD7, 1);
    
        /*WIFI_RX_SW_O*/
        //SMAC_REG_READ(sh, ADR_PAD8, &regval);
        //regval &= ~1;
        SMAC_REG_WRITE(sh, ADR_PAD8, 0);
    
        /*BT_SW_O*/
        //SMAC_REG_READ(sh, ADR_PAD9, &regval);
        //regval |= 1;
        SMAC_REG_WRITE(sh, ADR_PAD9, 1);
    
        /*WLAN_ACT (GPIO_1)*/
        //SMAC_REG_READ(sh, ADR_PAD25, &regval);
        //regval |= 1;
        SMAC_REG_WRITE(sh, ADR_PAD25, 1);
    
        /*BT_ACT (GPIO_2)*/
        //SMAC_REG_READ(sh, ADR_PAD27, &regval);
        //regval |= (1<<8);
        SMAC_REG_WRITE(sh, ADR_PAD27, 8);
    
        /*BT_PRI (GPIO_3)*/
        //SMAC_REG_READ(sh, ADR_PAD28, &regval);
        //regval |= (1<<8);
        SMAC_REG_WRITE(sh, ADR_PAD28, 8);
#endif    

    /*
        Please don't move 
        Be sure host driver can get packetid 0x80000000
    */
    ret = SMAC_LOAD_FW(sh);

exit:

    return ret;

}




void inline ssv6xxx_deinit_mac(struct ssv_softc *sc)
{
// ToDo: After deinit, device should be reset before init again. Why free pbuf?
#ifdef SSV6200_ECO
    int i;
    for(i=0;i<SSV_RC_MAX_STA;i++)
        ssv6xxx_pbuf_free(sc, sc->sh->hw_buf_ptr[i]);
#else // SSV6200_ECO
	ssv6xxx_pbuf_free(sc, sc->sh->hw_buf_ptr);
#endif // SSV6200_ECO
}


void inline ssv6xxx_deinit_hw(struct ssv_softc *sc)
{
    printk("%s(): \n", __FUNCTION__);
	ssv6xxx_deinit_mac(sc);
}

void inline ssv6xxx_restart_hw(struct ssv_softc *sc)
{
    printk("%s(): \n", __FUNCTION__);
    sc->restart_counter++;
    sc->force_triger_reset = true;
    HCI_STOP(sc->sh);
    ieee80211_restart_hw(sc->hw);
}

static int ssv6xxx_init_hw(struct ssv_hw *sh)
{
    int ret=0;

    /* ssv6200 hardware parameter settings. */
    //sh->pb_offset = 0x50;
    sh->tx_desc_len = SSV6XXX_TX_DESC_LEN;
    sh->rx_desc_len = SSV6XXX_RX_DESC_LEN;
    sh->rx_pinfo_pad = 0x04;
#if 0
    /* ssv6200 MAC initialization */
    ret = ssv6xxx_init_mac(sh);
    if (ret) {
 	    printk("ssv6xxx_init_mac(): init failed !\n");
        return ret;
    }
#endif


	//write rf table
    if (ret == 0) ret = SSV6XXX_SET_HW_TABLE(sh, ssv6200_rf_tbl);

    if (ret == 0) ret = SMAC_REG_WRITE(sh, 0xce000004, 0x00000000); /* ???? */
                                                                    /* Turn off phy before configuration */

	//write phy table
    if (ret == 0) ret = SSV6XXX_SET_HW_TABLE(sh, ssv6200_phy_tbl);

#ifdef CONFIG_SSV_CABRIO_E
    /* Cabrio E: GPIO setting */
    if (ret == 0) ret = SMAC_REG_WRITE(sh, ADR_PAD53, 0x21);        /* like debug-uart config ? */
    if (ret == 0) ret = SMAC_REG_WRITE(sh, ADR_PAD54, 0x3000);
    if (ret == 0) ret = SMAC_REG_WRITE(sh, ADR_PIN_SEL_0, 0x4000);

    /* TR switch: */
    if (ret == 0) ret = SMAC_REG_WRITE(sh, 0xc0000304, 0x01);
    if (ret == 0) ret = SMAC_REG_WRITE(sh, 0xc0000308, 0x01);
#endif // CONFIG_SSV_CABRIO_E

    //Switch clock to PLL output of RF
    //MAC and MCU clock selection :   00 : OSC clock   01 : RTC clock   10 : synthesis 80MHz clock   11 : synthesis 40MHz clock
    if (ret == 0) ret = SMAC_REG_WRITE(sh, ADR_CLOCK_SELECTION, 0x3);

    /**
        * The ordering of PHY/RF default register setting, IQ Calibration and 
        * channel calibration is (from bernie's suggestion)
        *
        *   1. channel calibration (to lock on a channel)
        *   2. IQ calibration
        *   3. set default PHY registers
        *   4. set default RF registers
        *   5. channel calibration ...................
        */
    if ((ret=ssv6xxx_set_channel(sh->sc, sh->cfg.def_chan)))
        return ret;

    /* Move iq_calib to ssv6200_ops.start, after fw loaded. */
    /*
    if ((ret=ssv6xxx_do_iq_calib(sh)))
        return ret;
    */

    
    //disable g/h mode
    //REG_WRITE(sh,0xce000004,0x07f); // B only
    //REG_WRITE(sh,0xce000004,0x06f); // G/N only
    if (ret == 0) ret = SMAC_REG_WRITE(sh, 0xce000004, 0x17f); // B/G/N only 
   

//SMAC_REG_WRITE(sh,0xce00002E,0x8155002C); //20 byte --> RTS
//SMAC_REG_WRITE(sh,0xce0000AC,0x800F000D); //14 byte --> ACK


    return ret;

}


static int ssv6xxx_read_hw_info(struct ssv_softc *sc)
{
    extern struct ssv6xxx_cfg ssv_cfg;
    struct ssv_hw *sh;
    
    /**
        * Allocate ssv6200 hardware data structure and set ssv6200 
        * hardware capabilities 
        */
    sh = kzalloc(sizeof(struct ssv_hw), GFP_KERNEL);
    if (sh == NULL)
        return -ENOMEM;
    memset((void *)sh, 0, sizeof(struct ssv_hw));
    sc->sh = sh;
    sh->sc = sc;
    sh->priv = sc->dev->platform_data;

    /**
        * Read ssv6200 configuration from external module, 
        * such as flash/eeprom...,etc.
        */
    memcpy(&sh->cfg, &ssv_cfg, sizeof(ssv_cfg));
    //printk("****hw_caps=%x\n", sh->cfg.hw_caps);

    /* Default configuration if no user configuration specified */
    if (sh->cfg.hw_caps == 0) {
        u8 sta_mac[] = { 0x20, 0x11, 0x22, 0x33, 0x44, 0x55 };
        sh->cfg.drv_type = 1;
        sh->cfg.def_chan = 7;
        sh->cfg.rxpb_offset = 80;
        sh->cfg.txpb_offset = 80;
        sh->cfg.hw_caps = SSV6200_HW_CAP_HT |
                          SSV6200_HW_CAP_2GHZ |
                          SSV6200_HW_CAP_SECURITY |
                          SSV6200_HW_CAP_RC |
                          SSV6200_HW_CAP_B |
                          SSV6200_HW_CAP_G |
                          SSV6200_HW_CAP_P2P|
                          SSV6200_HT_CAP_SGI_20|
                          //SSV6200_HW_CAP_AMPDU_RX|
                          //SSV6200_HW_CAP_AMPDU_TX|
                          SSV6200_HW_CAP_AP;
        memcpy(sh->cfg.maddr, sta_mac, 6);
    }

    /* HCI register parameters */
    sh->hci.dev = sc->dev;
    sh->hci.hci_ops = NULL;
    sh->hci.hci_rx_cb = ssv6200_rx;
    sh->hci.rx_cb_args = (void *)sc;
    sh->hci.hci_tx_cb= ssv6xxx_tx_cb;
    sh->hci.tx_cb_args = (void *)sc;
#ifdef RATE_CONTROL_REALTIME_UPDATA
    sh->hci.hci_skb_update_cb = ssv6xxx_tx_rate_update;
    sh->hci.skb_update_args = (void *)sc;
#else
    sh->hci.hci_skb_update_cb = NULL;
    sh->hci.skb_update_args = NULL;
#endif
    sh->hci.hci_tx_flow_ctrl_cb = ssv6200_tx_flow_control;
    sh->hci.tx_fctrl_cb_args = (void *)sc;

    sh->hci.hci_tx_q_empty_cb = ssv6xxx_tx_q_empty_cb;
    sh->hci.tx_q_empty_args = (void *)sc;
    
    sh->hci.if_ops = sh->priv->ops;

    sh->hci.hci_tx_buf_free_cb = ssv6xxx_txbuf_free_skb;
    sh->hci.tx_buf_free_args = (void *)sc;

    
    return 0;
}



static int ssv6xxx_init_device(struct ssv_softc *sc, const char *name)
{
    struct ieee80211_hw *hw = sc->hw;
    struct ssv_hw *sh;
    int error = 0;

    BUG_ON(!sc->dev->platform_data);
        
    if ((error=ssv6xxx_read_hw_info(sc)) != 0) {
        return error;
    }
    sh = sc->sh;    

    if (sh->cfg.hw_caps == 0)
        return -1;
    
    /* Initialize software control structure */
    if ((error=ssv6xxx_init_softc(sc)) != 0) {
        kfree(sh);
        return error;
    }
    
    /* HCI driver initialization */
    ssv6xxx_hci_register(&sh->hci);    

    /* Initialize ssv6200 hardware */
    if ((error=ssv6xxx_init_hw(sc->sh)) != 0) {
        ssv6xxx_deinit_softc(sc);
        return error;
    }

    /* Register to mac80211 stack */
    if ((error=ieee80211_register_hw(hw)) != 0) {
        ssv6xxx_hci_deregister();
        ssv6xxx_deinit_softc(sc);
        return error;
    }

#ifdef CONFIG_SSV6XXX_DEBUGFS
    ssv6xxx_init_debugfs(sc, name);
#endif // CONFIG_SSV6200_DEBUGFS
    return 0;
}


static void ssv6xxx_deinit_device(struct ssv_softc *sc)
{
    printk("%s(): \n", __FUNCTION__);
#ifdef CONFIG_SSV6XXX_DEBUGFS
    ssv6xxx_deinit_debugfs(sc);
#endif // CONFIG_SSV6200_DEBUGFS
    ssv6xxx_rf_disable(sc->sh);
    ieee80211_unregister_hw(sc->hw);
    ssv6xxx_deinit_hw(sc);
    ssv6xxx_deinit_softc(sc);
    ssv6xxx_hci_deregister();

    kfree(sc->sh);
}


extern struct ieee80211_ops ssv6200_ops;


int ssv6xxx_dev_probe(struct platform_device *pdev)
{
#ifdef CONFIG_SSV6200_CLI_ENABLE
    extern struct ssv_softc *ssv_dbg_sc;
#endif
    struct ssv_softc *softc;
    struct ieee80211_hw *hw;
    int ret;

    if (!pdev->dev.platform_data) {
        dev_err(&pdev->dev, "no platform data specified!\n");
        return -EINVAL;
    }
    
    printk("%s(): ssv6200 device found !\n", __FUNCTION__);
    hw = ieee80211_alloc_hw(sizeof(struct ssv_softc), &ssv6200_ops);
    if (hw == NULL) {
        dev_err(&pdev->dev, "No memory for ieee80211_hw\n");
        return -ENOMEM;
    }

    SET_IEEE80211_DEV(hw, &pdev->dev);
    dev_set_drvdata(&pdev->dev, hw);

    memset((void *)hw->priv, 0, sizeof(struct ssv_softc));
    softc = hw->priv;
    softc->hw = hw;
    softc->dev = &pdev->dev;
    
    ret = ssv6xxx_init_device(softc, pdev->name);
    if (ret) {
        dev_err(&pdev->dev, "Failed to initialize device\n");
        ieee80211_free_hw(hw);
        return ret;
    }

#ifdef CONFIG_SSV6200_CLI_ENABLE
    ssv_dbg_sc = softc;
#endif

    wiphy_info(hw->wiphy, "%s\n", "SSV6200 of South Silicon Valley");
    
    return 0;
}

EXPORT_SYMBOL(ssv6xxx_dev_probe);



int ssv6xxx_dev_remove(struct platform_device *pdev)
{
    struct ieee80211_hw *hw=dev_get_drvdata(&pdev->dev);
    struct ssv_softc *softc=hw->priv;
    printk("ssv6xxx_dev_remove(): pdev=%p, hw=%p\n", pdev, hw);    
    
    ssv6xxx_deinit_device(softc);


    printk("ieee80211_free_hw(): \n");
    ieee80211_free_hw(hw);
    pr_info("ssv6200: Driver unloaded\n");
    return 0;
}

EXPORT_SYMBOL(ssv6xxx_dev_remove);



static const struct platform_device_id ssv6xxx_id_table[] = {
    {
        .name = "ssv6200",
        .driver_data = 0x00,
    },
    {}, /* Terminating Entry */
};
MODULE_DEVICE_TABLE(platform, ssv6xxx_id_table);


static struct platform_driver ssv6xxx_driver =
{
    .probe          = ssv6xxx_dev_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    .remove         = __devexit_p(ssv6xxx_dev_remove),
#else
    .remove         = ssv6xxx_dev_remove,
#endif
    .id_table       = ssv6xxx_id_table,
    .driver     = {
        .name       = "SSV WLAN driver",
        .owner      = THIS_MODULE,
    }
};






#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
int ssv6xxx_init(void)
#else
static int __init ssv6xxx_init(void)
#endif
{
    /* for debugging interface */
    extern void *ssv_dbg_phy_table;
    extern u32 ssv_dbg_phy_len;
    extern void *ssv_dbg_rf_table;
    extern u32 ssv_dbg_rf_len;
    ssv_dbg_phy_table = (void *)ssv6200_phy_tbl;
    ssv_dbg_phy_len = sizeof(ssv6200_phy_tbl)/sizeof(struct ssv6xxx_dev_table);
    ssv_dbg_rf_table = (void *)ssv6200_rf_tbl;
    ssv_dbg_rf_len = sizeof(ssv6200_rf_tbl)/sizeof(struct ssv6xxx_dev_table);

    return platform_driver_register(&ssv6xxx_driver);
}
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
void ssv6xxx_exit(void)
#else
static void __exit ssv6xxx_exit(void)
#endif
{
    platform_driver_unregister(&ssv6xxx_driver);
}
#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
EXPORT_SYMBOL(ssv6xxx_init);
EXPORT_SYMBOL(ssv6xxx_exit);
#else
module_init(ssv6xxx_init);
module_exit(ssv6xxx_exit);
#endif


