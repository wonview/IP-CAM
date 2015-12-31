#ifndef _DEV_H_
#define _DEV_H_

#include <linux/version.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#ifdef CONFIG_SSV_SUPPORT_ANDROID
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#endif
#include <net/mac80211.h>

#include "ampdu.h"
#include "ssv_rc_common.h"
#include "drv_comm.h"
#include "sec.h"

#include <linux/kthread.h>


#define SSV6200_RX_BA_MAX_SESSIONS			1



#define SSV6200_OPMODE_STA				0
#define SSV6200_OPMODE_AP				1
#define SSV6200_OPMODE_IBSS				2
#define SSV6200_OPMODE_WDS				3



/* The maximal frames size */
//#define MAX_FRAME_SIZE  4096

#define HW_MAX_RATE_TRIES   7

/**
* Define the number of entries of mac decision table.
*
* @ MAC_DECITBL1_SIZE: The decision filtering table size
* @ MAC_DECITBL2_SIZE: The decision mask table size
*/
#define MAC_DECITBL1_SIZE               16
#define MAC_DECITBL2_SIZE               9

#ifndef USE_GENERIC_DECI_TBL
extern u16 ap_deci_tbl[];
extern u16 sta_deci_tbl[];
#else
extern u16 generic_deci_tbl[];
#define ap_deci_tbl     generic_deci_tbl
#define sta_deci_tbl    generic_deci_tbl
#endif // USE_GENERIC_DECI_TBL

/**
 *
 */
#define HT_SIGNAL_EXT				    6		//For 2.4G
#define HT_SIFS_TIME        		    10
#define BITS_PER_BYTE           8
#define HT_RC_2_STREAMS(_rc)    ((((_rc) & 0x78) >> 3) + 1)
#define ACK_LEN				    (14)	//include fcs
#define RTS_LEN					(20)	//include fcs
#define CTS_LEN					(14)	//include fcs
#define L_STF                   8
#define L_LTF                   8
#define L_SIG                   4
#define HT_SIG                  8
#define HT_STF                  4
#define HT_LTF(_ns)             (4 * (_ns))
#define SYMBOL_TIME(_ns)        ((_ns) << 2) /* ns * 4 us */
#define SYMBOL_TIME_HALFGI(_ns) (((_ns) * 18 + 4) / 5)  /* ns * 3.6 us */

#define CCK_SIFS_TIME        10
#define CCK_PREAMBLE_BITS   144
#define CCK_PLCP_BITS        48

#define OFDM_SIFS_TIME        16
#define OFDM_PREAMBLE_TIME    20
#define OFDM_PLCP_BITS        22
#define OFDM_SYMBOL_TIME      4

//#define WLAN_RC_PHY_CCK         0
//#define WLAN_RC_PHY_OFDM        1




/* These must match mac80211 skb queue mapping numbers */
#define WMM_AC_VO       0
#define WMM_AC_VI       1
#define WMM_AC_BE       2
#define WMM_AC_BK       3
#define WMM_NUM_AC      4

/* The maximal number of support TID */
#define WMM_TID_NUM     8


/**
*
*/
#define TXQ_EDCA_0      0x01
#define TXQ_EDCA_1      0x02
#define TXQ_EDCA_2      0x04
#define TXQ_EDCA_3      0x08
#define TXQ_MGMT        0x10



#define IS_SSV_HT(dsc)             ((dsc)->rate_idx >= 15)
#define IS_SSV_SHORT_GI(dsc)       ((dsc)->rate_idx>=23 && (dsc)->rate_idx<=30)
#define IS_SSV_HT_GF(dsc)          ((dsc)->rate_idx >= 31)
#define IS_SSV_SHORT_PRE(dsc)      ((dsc)->rate_idx>=4 && (dsc)->rate_idx<=14)


/**
* Macros for ssv6200 register read/write access on Linux platform.
* @ SMAC_REG_WRITE() : write 4-byte value into hardware register.
* @ SMAC_REG_READ()  : read 4-byte value from hardware register.
* @ SMAC_REG_SET_BITS: set the specified bits to a value.
* @ SMAC_REG_CONFIRM:
* 
*/
#define SMAC_REG_WRITE(_s, _r, _v)                  \
        (_s)->hci.hci_ops->hci_write_word(_r,_v)
#define SMAC_REG_READ(_s, _r, _v)                   \
        (_s)->hci.hci_ops->hci_read_word(_r, _v)
#define SMAC_LOAD_FW(_s)                            \
        (_s)->hci.hci_ops->hci_load_fw()
#define SMAC_REG_CONFIRM(_s, _r, _v)                \
{                                                   \
    u32 _regval;                                    \
    SMAC_REG_READ(_s, _r, &_regval);                \
    if (_regval != (_v)) {                          \
        printk("[0x%08x]: 0x%08x!=0x%08x\n",        \
        (_r), (_v), _regval);                       \
        return -1;                                  \
    }                                               \
}    
#define SMAC_REG_SET_BITS(_sh, _reg, _set, _clr)    \
({                                                  \
    int ret;                                        \
    u32 _regval;                                    \
    ret = SMAC_REG_READ(_sh, _reg, &_regval);       \
    _regval &= ~(_clr);                             \
    _regval |= (_set);                              \
    if (ret == 0)                                   \
        ret = SMAC_REG_WRITE(_sh, _reg, _regval);   \
    ret;                                            \
})



#define HCI_START(_sh) \
    (_sh)->hci.hci_ops->hci_start()
#define HCI_STOP(_sh) \
    (_sh)->hci.hci_ops->hci_stop()
#define HCI_SEND(_sh, _sk, _q) \
    (_sh)->hci.hci_ops->hci_tx(_sk, _q, 0)
#define HCI_SEND_FLAG(_sh, _sk, _q, _flag) \
    (_sh)->hci.hci_ops->hci_tx((_sk), (_q), (_flag))
#define HCI_PAUSE(_sh, _mk) \
    (_sh)->hci.hci_ops->hci_tx_pause(_mk)
#define HCI_RESUME(_sh, _mk) \
    (_sh)->hci.hci_ops->hci_tx_resume(_mk)
#define HCI_TXQ_FLUSH(_sh, _mk) \
    (_sh)->hci.hci_ops->hci_txq_flush(_mk)
#define HCI_TXQ_FLUSH_BY_STA(_sh, _aid) \
		(_sh)->hci.hci_ops->hci_txq_flush_by_sta(_aid)    
#define HCI_TXQ_EMPTY(_sh, _txqid) \
		(_sh)->hci.hci_ops->hci_txq_empty(_txqid)
#define HCI_WAKEUP_PMU(_sh) \
    (_sh)->hci.hci_ops->hci_pmu_wakeup()
#define HCI_SEND_CMD(_sh, _sk) \
        (_sh)->hci.hci_ops->hci_send_cmd(_sk)

#define SSV6XXX_SET_HW_TABLE(sh_, tbl_)                                 \
({                                                                      \
    int ret = 0;                                                        \
    u32 i=0;                                                            \
    for(; i<sizeof(tbl_)/sizeof(struct ssv6xxx_dev_table); i++) {       \
        ret = SMAC_REG_WRITE(sh_, tbl_[i].address, tbl_[i].data);       \
        if (ret) break;                                                 \
    }                                                                   \
    ret;                                                                \
})



struct ssv_softc;


#define SSV6200_HT_TX_STREAMS			    1
#define SSV6200_HT_RX_STREAMS			    1
#define SSV6200_RX_HIGHEST_RATE             72


enum PWRSV_STATUS{
    PWRSV_DISABLE,
    PWRSV_ENABLE,
    PWRSV_PREPARE,
};
/**
* struct ssv_hw - the structure for ssv6200 hardware information.
*
* This structure is shared between ssv6200 hw/sw mac & HCI/SDIO
* drivers. hw/sw mac registers this structure to HCI/SDIO.
*/
struct ssv_hw {
    struct ssv_softc *sc; /* back point to ssv_softc */
    struct ssvcabrio_platform_data *priv;
    struct ssv6xxx_hci_info hci;

    
//    u32 hw_caps;
//    u8 mac_addr[6];
//    int default_channel;

    /* parameter settings for ssv6200 mac */
//    u32 pb_offset;
    u32 tx_desc_len; /* include tx_phy_info length = 0 */
    u32 rx_desc_len; /* include rx_phy_info length  */ 
    u32 rx_pinfo_pad; /* after the payload */
    


    /**
        * on chip init, a packet buffer is allocated for both
        * security and phy info table space.
        */
#ifdef SSV6200_ECO
	//Set key base on VIF. 
    u32 hw_buf_ptr[SSV_RC_MAX_STA];
    u32 hw_sec_key[SSV_RC_MAX_STA];
#else
    u32 hw_buf_ptr;
    u32 hw_sec_key;
#endif
    u32 hw_pinfo; 

    /**
        * ssv6200 hardware configuration from external module,
        * such as flash/eeprom...,etc.
        */
    struct ssv6xxx_cfg cfg;
};


/**
* struct ssv_tx - tx queue for outgoing frames through interface.
* Each AC queue uniquely associates with a hardware tx queue.
*/
struct ssv_tx {
    u16 seq_no;
    int hw_txqid[WMM_NUM_AC];
    int ac_txqid[WMM_NUM_AC];
    u32 flow_ctrl_status; /* bit wise */

    /* statistical counters: */
    u32 tx_pkt[SSV_HW_TXQ_NUM];
    u32 tx_frag[SSV_HW_TXQ_NUM];

    struct list_head ampdu_tx_que;
    spinlock_t ampdu_tx_que_lock;
    u16 ampdu_tx_group_id;
};



/**
* struct ssv_rx - rx queue for queuing incoming frames from 
*                         interface (SDIO).
* The queue is processed in the background by a work job which
* is scheduled upon frames received.
*/
struct ssv_rx {
    
    /**
         * Hold an empty packet buffer to store 
         * incoming data from interface (SDIO). 
         * move to HCI, do ned to remove ?????????????????????????????????????????
         */
    struct sk_buff *rx_buf;
    
    spinlock_t rxq_lock;
    struct sk_buff_head rxq_head;
    u32 rxq_count;

    /* statistical counters: */
    u32 num_pkts;
    u32 rx_frag;
};


#ifdef MULTI_THREAD_ENCRYPT
/*Encryption for multi-threading*/
struct ssv_encrypt_task_list {
    struct task_struct* encrypt_task;
    wait_queue_head_t   encrypt_wait_q;
    volatile int started;
    volatile int cpu_offline;
    struct list_head list;
};

struct ssv_encrypt_st_list {
    u8 status;
    struct sk_buff *skb_ptr;
    struct list_head list;
};
#endif

/* Macros for struct ssv_sta_info */
#define SSV6XXX_GET_STA_INFO(_sc, _s) \
    &(_sc)->sta_info[((struct ssv_sta_priv_data *)((_s)->drv_priv))->sta_idx]



/**
* Constatnt value defined for s_flag which indicates a station's 
* current capabilities.
*
* @ STA_FLAG_PS
* @ STA_FLAG_QOS
* @ STA_FLAG_AMPDU
*
*/
#define STA_FLAG_VALID                  0x00001
#define STA_FLAG_QOS                    0x00002
#define STA_FLAG_AMPDU                  0x00004
#define STA_FLAG_ENCRYPT                0x00008


/**
* struct ssv_sta_info - structure to hold station info.
*/
struct ssv_sta_info {
    u16 aid;
    u16 s_flags;

    int hw_wsid;    /* -1: only software */
    
    struct ieee80211_sta *sta;
    struct ieee80211_vif *vif;

    bool sleeping;
    bool tim_set;
    //#ifdef USE_LOCAL_CRYPTO
    #if 0
    struct ssv_crypto_ops *crypt;
    void *crypt_priv;
    u32  KeySelect;
    bool ampdu_ccmp_encrypt;
    #endif // USE_LOCAL_CRYPTO
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    struct dentry *debugfs_dir;
	#endif // CONFIG_SSV6XXX_DEBUGFS
};


/**
* struct ssv_vif_info - structure to hold vif info.
*/
struct ssv_vif_info {
    struct ieee80211_vif   *vif;
    enum                    nl80211_iftype if_type;
    u8                      bssid[6];
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    struct dentry          *debugfs_dir;
    #endif // CONFIG_SSV6XXX_DEBUGFS
};


/* struct ssv_sta_priv_data - private data structure for ieee80211_*/
struct ssv_sta_priv_data {
    int sta_idx;
    int rc_idx;
    int rx_data_rate;
//    struct ssv6xxx_sta_rc_info sta_rc_info;
    u32 ampdu_mib_total_BA_counter;
    AMPDU_TID ampdu_tid[WMM_TID_NUM];
    bool has_hw_encrypt;
    bool need_sw_encrypt;
    bool has_hw_decrypt;
    u8   group_key_idx;
    #ifdef USE_LOCAL_CRYPTO
    // Local crypto setting for unicast frames.
    struct ssv_crypto_ops   *crypt;
    void                    *crypt_priv;
    u32                      KeySelect;
    bool                     ampdu_ccmp_encrypt;
    #endif // USE_LOCAL_CRYPTO
};

/* struct ssv_sta_priv_data - private data structure for ieee80211_*/
struct ssv_vif_priv_data {
    int         vif_idx;
    bool        is_security_valid; // For VIF-wide security.
    bool        has_hw_encrypt;
    bool        need_sw_encrypt;
    bool        has_hw_decrypt;
    bool        force_sw_encrypt; // Freddie ToDo: temporary solution for AP mode.
    u8          group_key_idx;

    #ifdef USE_LOCAL_CRYPTO
    // Local crypto setting for broadcast frame.
    struct ssv_crypto_ops   *crypt;
    void                    *crypt_priv;
    u32                      KeySelect;
    bool                     ampdu_ccmp_encrypt;
    #endif // USE_LOCAL_CRYPTO
};

/**
* Constant values defined for sc_flags which indicates the 
* current status of WiFi driver.
*
* @ SC_OP_INVALID
* @ SC_OP_HW_RESET
*
*/
#define SC_OP_INVALID               0x00000001
#define SC_OP_HW_RESET              0x00000002
#define SC_OP_OFFCHAN               0x00000004
#define SC_OP_FIXED_RATE            0x00000008
#define SC_OP_SHORT_PREAMBLE        0x00000010



struct ssv6xxx_beacon_info {
	u32 pubf_addr;
	u16 len;
	u8 tim_offset;
	u8 tim_cnt;	
};


#define SSV6200_MAX_BCAST_QUEUE_LEN 16
struct ssv6xxx_bcast_txq {	
    spinlock_t txq_lock;
    struct sk_buff_head qhead;       
    int cur_qsize;         
};


// #define DEBUG_AMPDU_FLUSH

#ifdef DEBUG_AMPDU_FLUSH
typedef struct AMPDU_TID_st AMPDU_TID;
#define MAX_TID     (24)
#endif // DEBUG_AMPDU_FLUSH

/**
* struct ssv_softc - hold the whole wifi driver data structure.
*
*/
struct ssv_softc {
    struct ieee80211_hw *hw;
    struct device *dev;

    //Force reset
    u32 restart_counter;
    bool force_triger_reset;
    bool rx_data_mcu;    
    struct ieee80211_supported_band sbands[IEEE80211_NUM_BANDS];
    struct ieee80211_channel *cur_channel;

    /* for sc configuration */
    struct mutex mutex;
    

    /* hardware configuration: */
    struct ssv_hw *sh;

    struct ssv_tx tx;
    struct ssv_rx rx;

    //struct ieee80211_vif *vif_ptr[SSV_NUM_VIF];
    struct ssv_vif_info vif_info[SSV_NUM_VIF];
    struct ieee80211_vif *vif;
    enum nl80211_iftype if_type;
    u8 bssid[6];
    u32 sc_flags;

    /* rate control algorithm */
    void *rc;

    /**
     * For fixed rate, max_rate_idx is the fixed rate to use.
     * For auto rate, max_rate_idx is the maximal auto rate.
     */
    int max_rate_idx;


    /*  security algorithm  the variable just use only for wep mode now. 
     *  we just keep the algorithm of pairwise key or group key.
     */
    u8 algorithm;
    u8 group_key_idx;
    
    bool bSecurity_wapi;

    //Rate control timer
    struct workqueue_struct *rc_sample_workqueue;
    struct sk_buff_head rc_report_queue;
    struct work_struct rc_sample_work;
    struct ssv_sta_info sta_info[SSV_NUM_STA];
    #ifdef DEBUG_AMPDU_FLUSH
    struct AMPDU_TID_st *tid[MAX_TID];
    #endif // DEBUG_AMPDU_FLUSH
    u16 rc_sample_sechedule;

    
    /* ssv6200 mac decision table */
    u16 *mac_deci_tbl;

	/* hanlde mac80211 interface work queue*/
	struct workqueue_struct *config_wq;

	/* for asic pbuf allocate/release*/
	struct mutex mem_mutex;

	/* beacon related function*/
	struct work_struct set_tim_work;
	
	bool enable_beacon;	
	u8 beacon_interval;
	u8 beacon_dtim_cnt;			//Maxium DTIM counter number, not DTIM

	//DRV_BCN_BCN0 bit 0
	//DRV_BCN_BCN1 bit 1
	u8 beacon_usage;		
	//double beacon buffer
	struct ssv6xxx_beacon_info beacon_info[2];
    //cache latest beacon
    struct sk_buff *beacon_buf; 

	/* broadcast related */	
	struct work_struct	bcast_start_work;
	struct delayed_work	bcast_stop_work;
    struct delayed_work	bcast_tx_work;

	bool aid0_bit_set;
	u8 hw_mng_used;
	u32 sta_asleep_mask;
	struct ssv6xxx_bcast_txq bcast_txq;
    int bcast_interval;
    
	
	/* Protect power save state */
	spinlock_t ps_state_lock; 			
	
	/* station mode related*/
	u8 sta_count;
	u8 hw_wsid_bit;

    #if 0
	// CCMP encryption in SSV6xxx driver.
    struct work_struct ampdu_tx_encry_work;
    bool ampdu_encry_work_scheduled;
    bool ampdu_ccmp_encrypt;
    // Sync key to HW encrption engine when mixing software and hardware encryption in one session.
    struct work_struct sync_hwkey_work;
    bool sync_hwkey_write;
    struct ssv_sta_info *key_sync_sta_info;

    AMPDU_REKEY_PAUSE_STATE  ampdu_rekey_pause;
    #endif // 0
	/* ampdu rx related*/
	int rx_ba_session_count;
    struct ieee80211_sta *rx_ba_sta;
    u8  rx_ba_bitmap;
    u8 	ba_ra_addr[ETH_ALEN];
    u16 ba_tid;
    u16 ba_ssn;
    struct work_struct set_ampdu_rx_add_work;
	struct work_struct set_ampdu_rx_del_work;


    bool isAssoc;
    u16 channel_center_freq;		//The channel we use in station mode    

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    bool bScanning;
#endif
    /*power saving*/
    int ps_status;
    u16 ps_aid;
    bool bStaPS;
    struct work_struct set_ps_work;    
#ifdef CONFIG_SSV_SUPPORT_ANDROID
    struct wake_lock wifi_alive_lock;    
    struct early_suspend early_suspend;
#endif

    // TX handling thread
    u16                 tx_wait_q_woken;
    wait_queue_head_t   tx_wait_q;
    struct sk_buff_head tx_skb_q;
    struct task_struct *tx_task;
    bool                tx_q_empty; // HCI queue empty
    struct sk_buff_head tx_done_q;

    // RX handling thread
    u16                 rx_wait_q_woken;
    wait_queue_head_t   rx_wait_q;
    struct sk_buff_head rx_skb_q;
    struct task_struct *rx_task;

#ifdef MULTI_THREAD_ENCRYPT    
    // Encryption threads
    struct sk_buff_head preprocess_q;
    spinlock_t encrypt_st_lock;    
    struct list_head encrypt_st_head;
    struct list_head encrypt_st_empty;
    volatile u32 encrypt_st_cnt;
    volatile u32 encrypted_cnt;
    volatile u32 empty_encrypt_st_cnt;
#endif    
    
    /**
        * MAC debug counters:
        */
    bool dbg_rx_frame;
    bool dbg_tx_frame;

#ifdef CONFIG_SSV6XXX_DEBUGFS
    /*
     * DebugFS
     */
    struct dentry *debugfs_dir;
    // struct dentry *vif_debugfs_dir;
#endif // CONFIG_SSV6XXX_DEBUGFS

    struct ssv6xxx_hw_sec sramKey;

};


typedef enum {
    SSV6XXX_IQK_CFG_XTAL_26M = 0,
    SSV6XXX_IQK_CFG_XTAL_40M,
    SSV6XXX_IQK_CFG_XTAL_24M,
    SSV6XXX_IQK_CFG_XTAL_MAX,
} ssv6xxx_iqk_cfg_xtal;

typedef enum {
    SSV6XXX_IQK_CFG_PA_DEF = 0,
    SSV6XXX_IQK_CFG_PA_LI_MPB,
    SSV6XXX_IQK_CFG_PA_LI_EVB,
    SSV6XXX_IQK_CFG_PA_HP,
} ssv6xxx_iqk_cfg_pa;

typedef enum {
    SSV6XXX_IQK_CMD_INIT_CALI = 0,
    SSV6XXX_IQK_CMD_RTBL_LOAD,
    SSV6XXX_IQK_CMD_RTBL_LOAD_DEF,
    SSV6XXX_IQK_CMD_RTBL_RESET,
    SSV6XXX_IQK_CMD_RTBL_SET,
    SSV6XXX_IQK_CMD_RTBL_EXPORT,
    SSV6XXX_IQK_CMD_TK_EVM,
    SSV6XXX_IQK_CMD_TK_TONE,
    SSV6XXX_IQK_CMD_TK_CHCH,
    SSV6XXX_IQK_CMD_TK_RXCNT,
} ssv6xxx_iqk_cmd_sel;

#define SSV6XXX_IQK_TEMPERATURE 0x00000004
#define SSV6XXX_IQK_RXDC        0x00000008
#define SSV6XXX_IQK_RXRC        0x00000010
#define SSV6XXX_IQK_TXDC        0x00000020
#define SSV6XXX_IQK_TXIQ        0x00000040
#define SSV6XXX_IQK_RXIQ        0x00000080
#define SSV6XXX_IQK_TSSI        0x00000100
#define SSV6XXX_IQK_PAPD        0x00000200


//typedef struct cfg_host_event HDR_HostEvent;

void ssv6xxx_txbuf_free_skb(struct sk_buff *skb , void *args);


void ssv6200_rx_process(struct work_struct *work);
#if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
int ssv6200_rx(struct sk_buff_head *rx_skb_q, void *args);
#else			
int ssv6200_rx(struct sk_buff *rx_skb, void *args);
#endif
void ssv6xxx_tx_cb(struct sk_buff_head *skb_head, void *args);
#ifdef RATE_CONTROL_REALTIME_UPDATA
void ssv6xxx_tx_rate_update(struct sk_buff *skb, void *args);
#endif
int ssv6200_tx_flow_control(void *dev, int hw_txqid, bool fc_en, int debug);
void ssv6xxx_tx_q_empty_cb (u32 txq_no, void *);
int ssv6xxx_rf_disable(struct ssv_hw *sh);
int ssv6xxx_rf_enable(struct ssv_hw *sh);
int ssv6xxx_set_channel(struct ssv_softc *sc, int ch);

int ssv6xxx_tx_task (void *data);
int ssv6xxx_rx_task (void *data);

//u32 ssv6xxx_pbuf_alloc(struct ssv_softc *sc, int size);
u32 ssv6xxx_pbuf_alloc(struct ssv_softc *sc, int size, int type);
bool ssv6xxx_pbuf_free(struct ssv_softc *sc, u32 pbuf_addr);

void ssv6xxx_add_txinfo(struct ssv_softc *sc, struct sk_buff *skb);
void ssv6xxx_ps_callback_func(unsigned long data);
void ssv6200_set_ps_work(struct work_struct *work);
void ssv6xxx_enable_ps(struct ssv_softc *sc);
void ssv6xxx_disable_ps(struct ssv_softc *sc);
void ssv6xxx_skb_encryt(struct sk_buff *mpdu,struct ssv_softc *sc);
void ssv6200_sync_hw_key_sequence(struct ssv_softc *sc, struct ssv_sta_info* sta_info, bool bWrite);

#ifdef CONFIG_SSV_SUPPORT_ANDROID
void ssv6xxx_early_suspend(struct early_suspend *h);
void ssv6xxx_late_resume(struct early_suspend *h);
#endif

#ifdef MULTI_THREAD_ENCRYPT
void ssv6xxx_skb_get_cryptops(struct sk_buff *mpdu, struct ssv_crypto_ops **crypt_ptr, void **crypt_priv_ptr);
int ssv6xxx_skb_pre_encryt(struct sk_buff *mpdu, struct ssv_softc *sc);
int ssv6xxx_encrypt_task (void *data);
#endif

#endif /* _DEV_H_ */

