#ifndef _AMPDU_H_
#define _AMPDU_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#include "linux_2_6_35.h"
#endif

// switch
#define Enable_ampdu_debug_log   (0)
#define Enable_AMPDU_Live_Time   (0) 
#define Enable_HW_AUTO_CRC_32    (1)
#define Enable_AMPDU_Rx          (1)
#define Enable_AMPDU_Tx          (1)
#define Enable_AMPDU_FW_Retry    (1)
#define Enable_AMPDU_delay_work  (1)
#define USE_FLUSH_RETRY
#define USE_AMPDU_TX_STATUS_ARRAY
#define SSV_AMPDU_FLOW_CONTROL
#define AMPDU_CHECK_SKB_SEQNO

// AMPDU-TX parameter
#define SSV_AMPDU_aggr_num_max           MAX_AGGR_NUM
#define SSV_AMPDU_seq_num_max            (4096)
#define SSV_AMPDU_aggr_size_max          ((SSV6200_PAGE_TX_THRESHOLD/2) << HW_MMU_PAGE_SHIFT)
#define SSV_AMPDU_retry_counter_max      (3)
#define SSV_AMPDU_tx_group_id_max        (64)

#define SSV_AMPDU_MAX_SSN                (4096)
#define SSV_AMPDU_BA_WINDOW_SIZE         (64)
#define SSV_AMPDU_WINDOW_SIZE            (64)
#define SSV_AMPDU_MAX_SIZE               ((SSV6200_PAGE_TX_THRESHOLD/2 - 1) << HW_MMU_PAGE_SHIFT)

#define SSV_AMPDU_FLOW_CONTROL_UPPER_BOUND  (64)
#define SSV_AMPDU_FLOW_CONTROL_LOWER_BOUND  (48)

// unit : ms
#define SSV_AMPDU_timer_period            (50)
#define SSV_AMPDU_TX_TIME_THRESHOLD       (50)
#define SSV_AMPDU_MPDU_LIVE_TIME          (SSV_AMPDU_retry_counter_max*8)
#define SSV_AMPDU_BA_TIME                 (50)

#define SSV_ILLEGAL_SN                   (0xffff)

// constant
#define AMPDU_BUFFER_SIZE            (32*1024)
#define AMPDU_SIGNATURE	             (0x4E)
#define AMPDU_DELIMITER_LEN          (4)
#define AMPDU_FCS_LEN                (4)
#define AMPDU_RESERVED_LEN           (3)
#define AMPDU_TX_NAV_MCS_567         (48) 
#define SSV_SEQ_NUM_SHIFT            (4)
#define SSV_RETRY_BIT_SHIFT          (11)
#define IEEE80211_SEQ_SEQ_SHIFT      (4)
#define IEEE80211_AMPDU_BA_LEN       (34)
//MCS 1
#define SSV6200_AMPDU_TRIGGER_INDEX  0

// flag
#define SSV_SN_STATUS_Release     (0xaa)
#define SSV_SN_STATUS_Retry       (0xbb)
#define SSV_SN_STATUS_Wait_BA     (0xcc)
#define SSV_SN_STATUS_Discard     (0xdd)
#define AMPDU_HCI_SEND_TAIL_WITH_FLOWCTRL    (0)
#define AMPDU_HCI_SEND_HEAD_WITH_FLOWCTRL    (1)
#define AMPDU_HCI_SEND_TAIL_WITHOUT_FLOWCTRL (2)
#define AMPDU_HCI_SEND_HEAD_WITHOUT_FLOWCTRL (3)

// mask
#define SSV_BAR_CTRL_ACK_POLICY_NORMAL     (0x0000)
#define SSV_BAR_CTRL_CBMTID_COMPRESSED_BA  (0x0004)
#define SSV_BAR_CTRL_TID_INFO_SHIFT	       (12)

// state
#define AMPDU_STATE_START        BIT(0)
#define AMPDU_STATE_OPERATION    BIT(1)
#define AMPDU_STATE_STOP         BIT(2)

typedef enum{
    AMPDU_REKEY_PAUSE_STOP=0,
    AMPDU_REKEY_PAUSE_START,
    AMPDU_REKEY_PAUSE_ONGOING,
    AMPDU_REKEY_PAUSE_DEFER,
    AMPDU_REKEY_PAUSE_HWKEY_SYNC,
}AMPDU_REKEY_PAUSE_STATE;

// macro
#define SSV_a_minus_b_in_c(a, b, c)            (((a)>=(b))?((a)-(b)):((c)-(b)+(a)))
#define SSV_AMPDU_SN_a_minus_b(a, b)           (SSV_a_minus_b_in_c((a), (b), SSV_AMPDU_seq_num_max))
#define AMPDU_HCI_SEND(_sh, _sk, _q, _flag)    (_sh)->hci.hci_ops->hci_tx((_sk), (_q), (_flag))
#define AMPDU_HCI_Q_EMPTY(_sh, _q)             (_sh)->hci.hci_ops->hci_txq_empty((_q))

struct SKB_info_st
{
    struct ieee80211_sta   *sta;
    u16                     mpdu_retry_counter;
    u16                     mpdu_group_id;
    unsigned long           aggr_timestamp;
    u16                     ampdu_tx_status;
    u16                     ampdu_tx_final_retry_count;
};

typedef struct SKB_info_st      SKB_info;
typedef struct SKB_info_st     *p_SKB_info;


// Header to keep tracking status of early aggregated AMPDU.
struct ampdu_hdr_st
{
    u32                     first_sn;
    struct sk_buff_head     mpdu_q;         // Aggreated MPDU queue. To keep track them during aggregation before send to HCI Q.
    u32                     max_size; // Maximum aggregated size allowed in this AMPDU according to the rate of the first MPDU.
    u32                     size;     // Current aggregated size.
    struct ieee80211_sta   *sta;
};


// unit : Bytes
#define SSV_SKB_info_size (sizeof(struct SKB_info_st))

enum AMPDU_TX_STATUS_E {
	AMPDU_ST_NON_AMPDU,    // Not an AMPDU frame or not being processed by AMPDU engine yet.
	AMPDU_ST_AGGREGATED,   // Aggregated into an AMPDU. No BA received.
	AMPDU_ST_RETRY,        // Need retry. (Missed acknowledgment)
	AMPDU_ST_DROPPED,      // Dropped MPDU. (After retry)
	AMPDU_ST_DONE,         // TX done. (Acknowledged)
};


typedef struct AMPDU_TID_st
{
    struct list_head                list;
    volatile unsigned long          timestamp;

    u32                             tidno;
    u16                             ac;
    struct ieee80211_sta           *sta;
    u16                             ssv_baw_size;
    u8                              agg_num_max;
    u8                              state;

#ifdef AMPDU_CHECK_SKB_SEQNO
    u32                             last_seqno;
#endif // AMPDU_CHECK_SKB_SEQNO
    struct sk_buff_head             ampdu_skb_tx_queue;
    spinlock_t                      ampdu_skb_tx_queue_lock;
    struct sk_buff_head             retry_queue;
    struct sk_buff_head             release_queue;

    // Aggregated MPDU skb. Combined with ssv_baw_head, aggregated MPDU can be easily accessed.
    struct sk_buff                 *aggr_pkts[SSV_AMPDU_BA_WINDOW_SIZE];
    volatile u32                    aggr_pkt_num;
    volatile u16                    ssv_baw_head;   //first un-ack sn
    spinlock_t                      pkt_array_lock;
    #ifdef ENABLE_AGGREGATE_IN_TIME
    struct sk_buff                 *cur_ampdu_pkt;  //
    struct sk_buff_head             early_aggr_ampdu_q;
    u32                             early_aggr_skb_num;
    #endif // ENABLE_AGGREGATE_IN_TIME

    // Freddie ToDo: CCMP encryption in ssv6xxx driver.
    struct sk_buff_head ampdu_skb_wait_encry_queue;

    u32                             ampdu_mib_reset;
    u32                             ampdu_mib_ampdu_counter;
    u32                             ampdu_mib_retry_counter;
    u32                             ampdu_mib_aggr_retry_counter;
    u32                             ampdu_mib_pure_retry_counter;
    u32                             ampdu_mib_small_pure_retry_counter;
    u32                             ampdu_mib_bar_counter;
    u32                             ampdu_mib_discard_counter;
    u32                             ampdu_mib_total_BA_counter;
    u32                             ampdu_mib_BA_counter;
    u32	                            ampdu_mib_dist[SSV_AMPDU_aggr_num_max + 1];

#ifdef CONFIG_SSV6XXX_DEBUGFS
    struct dentry                  *debugfs_dir;
    struct sk_buff_head             ba_q;
#endif // CONFIG_SSV6XXX_DEBUGFS
} AMPDU_TID, *p_AMPDU_TID;


typedef struct AMPDU_DELIMITER_st
{
    u16         reserved:4;     //0-3
    u16         length:12;      //4-15
    u8          crc;
    u8          signature;
} AMPDU_DELIMITER, *p_AMPDU_DELIMITER;


typedef struct AMPDU_BLOCKACK_st
{
	u16			frame_control; 	
	u16			duration; 		
	u8			ra_addr[ETH_ALEN];
	u8			ta_addr[ETH_ALEN];
    
    u16         BA_ack_ploicy:1;
    u16         multi_tid:1;
    u16         compress_bitmap:1;
    u16         reserved:9;
    u16         tid_info:4;
    
    u16         BA_fragment_sn:4;
    u16         BA_ssn:12;
    u32         BA_sn_bit_map[2];
} AMPDU_BLOCKACK, *p_AMPDU_BLOCKACK;


struct ssv_bar {
	unsigned short frame_control;
	unsigned short duration;
	unsigned char ra[6];
	unsigned char ta[6];
	unsigned short control;
	unsigned short start_seq_num;
} __packed;


#if Enable_ampdu_debug_log
    #define ampdu_db_log(format, args...) printk("~~~ampdu [%s:%d] "format, __FUNCTION__, __LINE__, ##args)
    #define ampdu_db_log_simple(format, args...) printk(format, ##args)
#else
    #define ampdu_db_log(...) do {} while (0)
    #define ampdu_db_log_simple(...) do {} while (0)
#endif

#if Enable_AMPDU_delay_work
void ssv6200_ampdu_delayed_work_callback_func(struct work_struct *work);
#else
void ssv6200_ampdu_timer_callback_func(unsigned long data);
#endif
void ssv6200_ampdu_init(struct ieee80211_hw *hw);
void ssv6200_ampdu_deinit(struct ieee80211_hw *hw);
void ssv6200_ampdu_release_skb(struct sk_buff *skb, struct ieee80211_hw *hw);
void ssv6200_ampdu_tx_start(u16 tid, struct ieee80211_sta *sta, struct ieee80211_hw *hw, u16 *ssn);
void ssv6200_ampdu_tx_operation(u16 tid, struct ieee80211_sta *sta, struct ieee80211_hw *hw, u8 buffer_size);
void ssv6200_ampdu_tx_stop(u16 tid, struct ieee80211_sta *sta, struct ieee80211_hw *hw);
bool ssv6200_ampdu_tx_handler(struct ieee80211_hw *hw, struct sk_buff *skb);
u32 ssv6xxx_ampdu_flush(struct ieee80211_hw *hw);
void ssv6200_ampdu_timeout_tx (struct ieee80211_hw *hw);
struct cfg_host_event;
void ssv6200_ampdu_no_BA_handler(struct ieee80211_hw *hw, struct sk_buff *skb);
void ssv6200_ampdu_BA_handler(struct ieee80211_hw *hw, struct sk_buff *skb);
void ssv6200_ampdu_tx_update_state(void *priv, struct ieee80211_sta *sta, struct sk_buff *skb);
void ssv6200_ampdu_tx_add_sta(struct ieee80211_hw *hw, struct ieee80211_sta *sta);
void ssv6xxx_ampdu_postprocess_BA (struct ieee80211_hw *hw);


extern void ssv6xxx_set_ampdu_rx_add_work(struct work_struct *work);
extern void ssv6xxx_set_ampdu_rx_del_work(struct work_struct *work);

void ssv6xxx_mib_reset (struct ieee80211_hw *hw);
ssize_t ssv6xxx_mib_dump (struct ieee80211_hw *hw, char *mib_str, ssize_t length);

#if 0
void tx_work(struct work_struct *work);
void retry_work(struct work_struct *work);
#endif // 0

// Freddie ToDo: CCMP encryption in ssv6xxx driver.
void encry_work(struct work_struct *work);
void sync_hw_key_work(struct work_struct *work);

#endif /* _AMPDU_H_ */



