#ifndef _HCTRL_H_
#define _HCTRL_H_


/* The maximal frames size */
#define MAX_FRAME_SIZE              8200//4608//2304


#define SSV6XXX_INT_RX              0x00000001  //1<<0

/*Workaround solution: We use this interrupt bit for Queue 4(MNG) */
#define SSV6XXX_INT_TX              0x00000002  //1<<1
#define SSV6XXX_INT_SOC             0x00000004  //1<<2
#define SSV6XXX_INT_LOW_EDCA_0      0x00000008  //1<<3
#define SSV6XXX_INT_LOW_EDCA_1      0x00000010  //1<<4
#define SSV6XXX_INT_LOW_EDCA_2      0x00000020  //1<<5
#define SSV6XXX_INT_LOW_EDCA_3      0x00000040  //1<<6
#define SSV6XXX_INT_RESOURCE_LOW    0x00000080  //1<<7


#define IFDEV(_ct)                      ((_ct)->shi->dev)
#define IFOPS(_ct)                      ((_ct)->shi->if_ops)
#define HCI_REG_READ(_ct, _adr, _val)   IFOPS(_ct)->readreg(IFDEV(_ct), _adr, _val)
#define HCI_REG_WRITE(_ct,_adr, _val)   IFOPS(_ct)->writereg(IFDEV(_ct), _adr, _val)
#define HCI_REG_SET_BITS(_ct, _reg, _set, _clr)     \
{                                                   \
    u32 _regval;                                    \
    if(HCI_REG_READ(_ct, _reg, &_regval));          \
    _regval &= ~(_clr);                             \
    _regval |= (_set);                              \
    if(HCI_REG_WRITE(_ct, _reg, _regval));          \
}




/**
* sdio irq interfaces for HCI:
*
* @ HCI_IRQ_REQUEST()
*/
#define HCI_IRQ_REQUEST(ct, hdle)       IFOPS(ct)->irq_request(IFDEV(ct), hdle, ct)
#define HCI_IRQ_ENABLE(ct)              IFOPS(ct)->irq_enable(IFDEV(ct))
#define HCI_IRQ_DISABLE(ct)             IFOPS(ct)->irq_disable(IFDEV(ct), false)
#define HCI_IRQ_STATUS(ct, sts)         IFOPS(ct)->irq_getstatus(IFDEV(ct), sts)
#define HCI_IRQ_SET_MASK(ct, mk)        IFOPS(ct)->irq_setmask(IFDEV(ct), mk)
#define IF_SEND(_ct, _bf, _len, _qid)   IFOPS(_ct)->write(IFDEV(_ct), _bf, _len, _qid);
#define IF_RECV(ct, bf, len)            IFOPS(ct)->read(IFDEV(ct), bf, len)
#define HCI_IRQ_TRIGGER(ct)             IFOPS(ct)->irq_trigger(IFDEV(ct))
#define HCI_PMU_WAKEUP(ct)      		IFOPS(ct)->pmu_wakeup(IFDEV(ct))
#define HCI_LOAD_FW(ct)                 IFOPS(ct)->load_fw(IFDEV(ct));









struct ssv6xxx_hci_ctrl {

    struct ssv6xxx_hci_info *shi;

    /* SDIO interrupt status */
    spinlock_t int_lock;
    u32 int_status;
    
    /* bit3(VO) bit4(VI) bit5(BE) bit6(BK) r/w this field need to use "int_lock"*/
    u32 int_mask;

    /* pause/resume */
    struct mutex txq_mask_lock;
    u32 txq_mask;
    
    /* hardware tx queue: 0 has the lowest priority */
    struct ssv_hw_txq hw_txq[SSV_HW_TXQ_NUM];

    /* ASIC int mask need to be protected by it */
    struct mutex hci_mutex;
    
    bool hci_start;
    
    /**
        * Always hold an empty skbuff so that the incoming data
        * could be receive.
        */
    struct sk_buff *rx_buf;
    u32 rx_pkt;

    struct workqueue_struct *hci_work_queue;
    struct work_struct hci_rx_work;    
#ifdef CONFIG_SSV_TX_POLLING
    struct work_struct hci_tx_work;
#else
    struct work_struct hci_tx_work[SSV_HW_TXQ_NUM];
#endif


    /**
        * HCI debug statistical counters:
        */
    u32 read_rs0_info_fail;
    u32 read_rs1_info_fail;
    u32 rx_work_running;
    u32 isr_running;
    u32 xmit_running;

    u32 isr_summary_eable;
    u32 isr_routine_time;    
    u32 isr_tx_time;
    u32 isr_rx_time;
    u32 isr_idle_time;
    u32 isr_rx_idle_time;
    /* neither tx or tx->should tx be happen*/
    u32 isr_miss_cnt;

    unsigned long prev_isr_jiffes;
    unsigned long prev_rx_isr_jiffes;

#ifdef CONFIG_SSV6XXX_DEBUGFS
	struct dentry *debugfs_dir;
	u32       isr_mib_enable;
	u32       isr_mib_reset;
	long long isr_total_time;
	long long isr_tx_io_time;
	long long isr_rx_io_time;
	u32       isr_rx_io_count;
	u32       isr_tx_io_count;
	long long isr_rx_proc_time;
#ifdef CONFIG_IRQ_DEBUG_COUNT
    bool irq_enable;
    u32 irq_count;
    u32 invalid_irq_count;
    u32 tx_irq_count;
    u32 real_tx_irq_count;
    u32 rx_irq_count;
    u32 irq_rx_pkt_count;
    u32 irq_tx_pkt_count;
#endif // CONFIG_IRQ_DEBUG_COUNT
#endif // CONFIG_SSV6XXX_DEBUGFS
};



struct ssv6xxx_hci_txq_info {
	u32 tx_use_page:8;
    u32 tx_use_id:6;
    u32 txq0_size:4;
	u32 txq1_size:4;
	u32 txq2_size:5;
	u32 txq3_size:5;
};


struct ssv6xxx_hci_txq_info2 {
	u32 tx_use_page:9;
    u32 tx_use_id:8;
	u32 txq4_size:4;
    u32 rsvd:11;
};




#endif /* _HCTRL_H */

