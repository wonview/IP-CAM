#ifndef _SSV_HCI_H_
#define _SSV_HCI_H_



/**
* The number of SSV6200 hardware TX queue. The
* higher queue value has the higher priority. 
* (BK) (BE) (VI) (VO) (MNG)
*/
#define SSV_HW_TXQ_NUM              5

/**
* The size of the each hardware tx queue.
*/
#define SSV_HW_TXQ_MAX_SIZE         512
#define SSV_HW_TXQ_RESUME_THRES     ((SSV_HW_TXQ_MAX_SIZE >> 2) *3)



/**
* Define flags for enqueue API 
*/
#define HCI_FLAGS_ENQUEUE_HEAD	   0x00000001
#define HCI_FLAGS_NO_FLOWCTRL	   0x00000002



/**
* struct ssv_hw_txq - ssv6200 hardware tx queue.
* The outgoing frames are finally queued here and wait for
* tx thread to send to hardware through interface (SDIO).
*/
struct ssv_hw_txq {
    u32 txq_no;
    
    //spinlock_t txq_lock;
    struct sk_buff_head qhead;
    int max_qsize;
    int resum_thres;
//    int cur_qsize;
    bool paused;

    /* statistic counters: */
    u32 tx_pkt;
    u32 tx_flags;
};



/**
* struct ssv_hci_ops - the interface between ssv hci and upper driver.
*
*/
struct ssv6xxx_hci_ops {

//    int (*hci_irq_enable)(void);
//    int (*hci_irq_disable)(void);

    int (*hci_start)(void);
    int (*hci_stop)(void);

    int (*hci_read_word)(u32 addr, u32 *regval);
    int (*hci_write_word)(u32 addr, u32 regval);
    int (*hci_load_fw)(void);

    
    /**
        * This function is assigned by HCI driver at initial time and is called 
        * from the drivers above the HCI layer if upper layer has tx frames
        * to send. The return value of this function maybe one of:
        * @   0: sccess
        * @   1: after accepting the current frame, the queue is full
        * @ -1: failed
        */
    int (*hci_tx)(struct sk_buff *, int, u32);

#if 0
    /**
        * This function is assigned by the drivers above the HCI layer and 
        * is called from HCI driver once it receives frames from interface
        * (SDIO).
        */
    int (*hci_rx)(struct sk_buff *);
#endif

    int (*hci_tx_pause)(u32 txq_mask);

    /**
        * If HCI queue is full, HCI will prevent upper layer from transmitting 
        * frames. This function is used by HCI to signal upper layer to resume
        * frame transmission.
        */
    int (*hci_tx_resume)(u32 txq_mask);

    /**
        * This function is used by upper layer to discard the specified txq 
        * frames. If the parameter is NULL, all txq in HCI will be discarded.
        */
    int (*hci_txq_flush)(u32 txq_mask);


    /**
        * Called from upper layer to flush tx frames which are dedicated to
        * a explicitly specify station AID. This function is normally used on
        * AP mode.
        */
    int (*hci_txq_flush_by_sta)(int aid);

   
    /**
        * Function provided for query of queue status by upper layer. The
        * parameter maybe one of
        * @ NULL :        indicate all queues
        * @ non-NULL: indicate the specify queue
        */
    bool (*hci_txq_empty)(int txqid);
 
    int (*hci_pmu_wakeup)(void);
 
    int (*hci_send_cmd)(struct sk_buff *);

#ifdef CONFIG_SSV6XXX_DEBUGFS
    bool (*hci_init_debugfs)(struct dentry *dev_deugfs_dir);
    void (*hci_deinit_debugfs)(void);
#endif // CONFIG_SSV6XXX_DEBUGFS
};



/**
* struct ssv6xxx_hci_info - ssv6xxx hci registration interface.
*
* This structure shall be allocated from registrar and register to
* ssv6xxx hci.
* @ dev
* @ if_ops : sdio/spi operation
* @ hci_ops : hci operation
* @ hci_rx_cb
*/
struct ssv6xxx_hci_info {

    struct device *dev;
    struct ssv6xxx_hwif_ops *if_ops;
    struct ssv6xxx_hci_ops *hci_ops;
    
    /* Rx callback function */

    #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
    int (*hci_rx_cb)(struct sk_buff_head *, void *);
    #else
    int (*hci_rx_cb)(struct sk_buff *, void *);
    #endif // USE_THREAD_RX
    void *rx_cb_args;

    /* Tx callback function */
    void (*hci_tx_cb)(struct sk_buff_head *, void *);
    void *tx_cb_args;
    
    /* Flow control callback function */
    int (*hci_tx_flow_ctrl_cb)(void *, int, bool, int debug);
    void *tx_fctrl_cb_args;

    /* Tx buffer function */
    void (*hci_tx_buf_free_cb)(struct sk_buff *, void *);
    void *tx_buf_free_args;

    /* Rate control update */
    void (*hci_skb_update_cb)(struct sk_buff *, void *);
    void *skb_update_args;

    /* HCI queue empty */
    void (*hci_tx_q_empty_cb)(u32 txq_no, void *);
    void *tx_q_empty_args;
};





int ssv6xxx_hci_deregister(void);
int ssv6xxx_hci_register(struct ssv6xxx_hci_info *);


#endif /* _SSV_HCI_H_ */


