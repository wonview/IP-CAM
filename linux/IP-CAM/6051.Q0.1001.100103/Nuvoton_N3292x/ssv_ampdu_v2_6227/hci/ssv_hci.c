#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include <ssv6200.h>
#include "hctrl.h"




MODULE_AUTHOR(" South Silicon Valley Microelectronics Co., Ltd");
MODULE_DESCRIPTION("HCI driver for SSV 802.11n wireless LAN cards.");
MODULE_SUPPORTED_DEVICE("SSV 802.11n WLAN cards");
MODULE_LICENSE("Dual BSD/GPL");



static struct ssv6xxx_hci_ctrl *ctrl_hci=NULL;



/* ?????????????????????????????????????????????????????????????????? */
struct sk_buff *ssv_skb_alloc(s32 len)
{
    struct sk_buff *skb;

    skb = __dev_alloc_skb(len + 128 , GFP_KERNEL);
    if (skb != NULL) {
        skb_put(skb,0x20);
        skb_pull(skb,0x20);
    }
    return skb;
}


void ssv_skb_free(struct sk_buff *skb)
{
    dev_kfree_skb_any(skb);
}



static int ssv6xxx_hci_irq_enable(void)
{
    /* enable interrupt */
    ctrl_hci->shi->if_ops->irq_setmask(
        ctrl_hci->shi->dev,
        ~(ctrl_hci->int_mask));
    ctrl_hci->shi->if_ops->irq_enable(
        ctrl_hci->shi->dev);
    return 0;
}



static int ssv6xxx_hci_irq_disable(void)
{

    /* disable interrupt */
    ctrl_hci->shi->if_ops->irq_setmask(
        ctrl_hci->shi->dev, 0xffffffff);    
    ctrl_hci->shi->if_ops->irq_disable(ctrl_hci->shi->dev, false);
    return 0;
}
#ifndef HCI_TX_POLLING
static void ssv6xxx_hci_irq_register(u32 irq_mask)
{
    unsigned long flags;
    u32 regval;
    
    mutex_lock(&ctrl_hci->hci_mutex);

    spin_lock_irqsave(&ctrl_hci->int_lock, flags);  
    ctrl_hci->int_mask |= irq_mask;
    regval = ~ctrl_hci->int_mask;
    spin_unlock_irqrestore(&ctrl_hci->int_lock, flags);
    
    HCI_IRQ_SET_MASK(ctrl_hci, regval);
    mutex_unlock(&ctrl_hci->hci_mutex);
}
#endif
static inline u32 ssv6xxx_hci_get_int_bitno(int txqid)
{
    /*Workaround solution: We use this interrupt bit(bit 1) for Queue 4(MNG) 
        other interrupt bit you can reference */
    //#define SSV6XXX_INT_TX              0x00000002  //1<<1
    
    if(txqid ==  SSV_HW_TXQ_NUM-1)
        return 1;
    else
        return txqid+3;            
}


static int ssv6xxx_hci_start(void)
{
    ssv6xxx_hci_irq_enable();
    ctrl_hci->hci_start = true;
#ifndef HCI_TX_POLLING
    HCI_IRQ_TRIGGER(ctrl_hci);
#endif
    return 0;
}


static int ssv6xxx_hci_stop(void)
{
    ssv6xxx_hci_irq_disable();
    ctrl_hci->hci_start = false;
    return 0;
}


static int ssv6xxx_hci_read_word(u32 addr, u32 *regval)
{
    int ret = HCI_REG_READ(ctrl_hci, addr, regval);
    return ret;
}


static int ssv6xxx_hci_write_word(u32 addr, u32 regval)
{
    return HCI_REG_WRITE(ctrl_hci, addr, regval);
}

static int ssv6xxx_hci_load_fw(void)
{
    return HCI_LOAD_FW(ctrl_hci);
}

static int ssv6xxx_hci_pmu_wakeup(void)
{
    HCI_PMU_WAKEUP(ctrl_hci);
    return 0;
}

static int ssv6xxx_hci_send_cmd(struct sk_buff *skb)
{
    int ret;
    ret = IF_SEND(ctrl_hci, (void *)skb->data, skb->len, 0);
    if (ret < 0) {
        printk("ssv6xxx_hci_send_cmd fail......\n");
    }
    return ret;
}

static int ssv6xxx_hci_enqueue(struct sk_buff *skb, int txqid, u32 tx_flags)
{
    struct ssv_hw_txq *hw_txq;
#ifndef HCI_TX_POLLING
    unsigned long flags;
    u32 status;
#endif
    BUG_ON(txqid >= SSV_HW_TXQ_NUM || txqid < 0);
    if (txqid >= SSV_HW_TXQ_NUM || txqid < 0)
        return -1;

    hw_txq = &ctrl_hci->hw_txq[txqid];

    hw_txq->tx_flags = tx_flags;

    if (tx_flags & HCI_FLAGS_ENQUEUE_HEAD)
        skb_queue_head(&hw_txq->qhead, skb);
    else
        skb_queue_tail(&hw_txq->qhead, skb);

    //Flow control check
    //spin_lock_irqsave(&hw_txq->txq_lock, flags);
    if (!(tx_flags & HCI_FLAGS_NO_FLOWCTRL)) {
        if (skb_queue_len(&hw_txq->qhead) >= hw_txq->max_qsize) {
            /* start tx flow control */
            ctrl_hci->shi->hci_tx_flow_ctrl_cb(
                ctrl_hci->shi->tx_fctrl_cb_args,
                hw_txq->txq_no,
                true,2000
            );
        }
    }
    //spin_unlock_irqrestore(&hw_txq->txq_lock, flags);
#ifndef HCI_TX_POLLING
    spin_lock_irqsave(&ctrl_hci->int_lock, flags);
    status = (ctrl_hci->int_mask | ctrl_hci->int_status);

#ifdef CONFIG_SSV_TX_LOWTHRESHOLD
    /* Read ctrl_hci->int_mask to see if need to enable interrupt. */
    if ((status&(SSV6XXX_INT_RESOURCE_LOW))== 0)
        queue_work(ctrl_hci->hci_work_queue,&ctrl_hci->hci_tx_work[txqid]);
#else
    {
        u32 bitno;
        bitno = ssv6xxx_hci_get_int_bitno(txqid);
        /* Read ctrl_hci->int_mask to see if need to enable interrupt. */
        if ((status&(1<<bitno))== 0)
            queue_work(ctrl_hci->hci_work_queue,&ctrl_hci->hci_tx_work[txqid]);
    }
#endif

    spin_unlock_irqrestore(&ctrl_hci->int_lock, flags);
#else
    //printk("eq=%p\n",skb);
    queue_work(ctrl_hci->hci_work_queue,&ctrl_hci->hci_tx_work[txqid]);
#endif
    return 0;
    
}


static bool ssv6xxx_hci_is_txq_empty(int txqid)
{
    struct ssv_hw_txq *hw_txq;
    BUG_ON(txqid >= SSV_HW_TXQ_NUM);
    if (txqid >= SSV_HW_TXQ_NUM)
        return false;
    
    hw_txq = &ctrl_hci->hw_txq[txqid];
    if (skb_queue_len(&hw_txq->qhead) <= 0)
        return true;
    return false;
}


static int ssv6xxx_hci_txq_flush(u32 txq_mask)
{
    struct ssv_hw_txq *hw_txq;
    struct sk_buff *skb = NULL;
    int txqid;

    for(txqid=0; txqid<SSV_HW_TXQ_NUM; txqid++) {
        if ((txq_mask & (1<<txqid)) != 0)
            continue;
        hw_txq = &ctrl_hci->hw_txq[txqid];
        while((skb = skb_dequeue(&hw_txq->qhead))) {
            ctrl_hci->shi->hci_tx_buf_free_cb (skb,
                        ctrl_hci->shi->tx_buf_free_args);
        }
    }
   
    return 0;
}


static int ssv6xxx_hci_txq_flush_by_sta(int aid)
{

    return 0;
}



static int ssv6xxx_hci_txq_pause(u32 txq_mask)
{
    struct ssv_hw_txq *hw_txq;    
    int txqid;

    mutex_lock(&ctrl_hci->txq_mask_lock);
    ctrl_hci->txq_mask |= (txq_mask & 0x1F);
    for(txqid=0; txqid<SSV_HW_TXQ_NUM; txqid++) {
        if ((ctrl_hci->txq_mask&(1<<txqid)) == 0)
            continue;
        hw_txq = &ctrl_hci->hw_txq[txqid];
        hw_txq->paused = true;
    }

    /* halt hardware tx queue */
    HCI_REG_SET_BITS(ctrl_hci, ADR_MTX_MISC_EN,
        (ctrl_hci->txq_mask<<16), (0x1F<<16));
    mutex_unlock(&ctrl_hci->txq_mask_lock);
    
    //printk("%s(): ctrl_hci->txq_mas=0x%x\n", __FUNCTION__, ctrl_hci->txq_mask);    
    return 0;
}



static int ssv6xxx_hci_txq_resume(u32 txq_mask)
{
    struct ssv_hw_txq *hw_txq;
    int txqid;

    mutex_lock(&ctrl_hci->txq_mask_lock);
    ctrl_hci->txq_mask &= ~(txq_mask&0x1F);
    for(txqid=0; txqid<SSV_HW_TXQ_NUM; txqid++) {
        if ((ctrl_hci->txq_mask&(1<<txqid)) != 0)
            continue;
        hw_txq = &ctrl_hci->hw_txq[txqid];
        hw_txq->paused = false;
    }    
    
    /* resume hardware tx queue */
    HCI_REG_SET_BITS(ctrl_hci, ADR_MTX_MISC_EN,
        (ctrl_hci->txq_mask<<16), (0x1F<<16));
    
    mutex_unlock(&ctrl_hci->txq_mask_lock);
    return 0;
}



/**
* int ssv6xxx_hci_xmit() - send the specified number of frames for a specified 
*                                       tx queue to SDIO.
*
* @ struct ssv_hw_txq *hw_txq: the output queue to send.
* @ int max_count: the maximal number of frames to send.
*/
static int ssv6xxx_hci_xmit(struct ssv_hw_txq *hw_txq, int max_count)
{
    struct sk_buff_head tx_cb_list;
    struct sk_buff *skb;
    int tx_count, ret;
    //unsigned long flags;
    
    ctrl_hci->xmit_running = 1;
    skb_queue_head_init(&tx_cb_list);
    
    for(tx_count=0; tx_count<max_count; tx_count++) {
        if (ctrl_hci->hci_start == false){
            printk("ssv6xxx_hci_xmit - hci_start = false\n");
            goto xmit_out;
        }
        skb = skb_dequeue(&hw_txq->qhead);
        if (!skb){
            printk("ssv6xxx_hci_xmit - queue empty \n");
            goto xmit_out;
	    }
#if 1
        //Rate control update rate
        if(ctrl_hci->shi->hci_skb_update_cb != NULL)
        {
            ctrl_hci->shi->hci_skb_update_cb(skb,ctrl_hci->shi->skb_update_args);
        }
#endif   
        /**
                * send to ssv6xxx SoC through SDIO/SPI interface. If fail to send 
                * the frame, try again next tx round.
                */
        ret = IF_SEND(ctrl_hci, (void *)skb->data, skb->len, hw_txq->txq_no);
        if (ret < 0) {
            printk(KERN_ALERT "ssv6xxx_hci_xmit fail......\n");
            skb_queue_head(&hw_txq->qhead, skb);
            break;
        }
        skb_queue_tail(&tx_cb_list, skb);
        hw_txq->tx_pkt ++;
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if(ctrl_hci->irq_enable)
		ctrl_hci->irq_tx_pkt_count++;
#endif 
        
        /**
                * Notify upper layer of stopping flow control if the flow
                * control has been enabled by upper layer.
                */

        if (!(hw_txq->tx_flags & HCI_FLAGS_NO_FLOWCTRL)) {
            //spin_lock_irqsave(&hw_txq->txq_lock, flags);  
            if (skb_queue_len(&hw_txq->qhead) < hw_txq->resum_thres) {
                ctrl_hci->shi->hci_tx_flow_ctrl_cb(
                    ctrl_hci->shi->tx_fctrl_cb_args,
                    hw_txq->txq_no, 
                    false,2000
                );
            }
            //spin_unlock_irqrestore(&hw_txq->txq_lock, flags);
        }
    }
    
xmit_out:

    /* Report frames tx status to mac80211: for rate control */
    if (ctrl_hci->shi->hci_tx_cb) {
        ctrl_hci->shi->hci_tx_cb (&tx_cb_list,
            ctrl_hci->shi->tx_cb_args);
    }        
    ctrl_hci->xmit_running = 0;
    return tx_count;
}




#if 0
static int ssv6xxx_hci_tx_handler_poll(void *dev, int max_tx_cnt)
{
    struct ssv6xxx_hci_txq_info txq_info;
    struct ssv_hw_txq *hw_txq;
    struct sk_buff *skb;
    u32 free_tx_page=0, free_tx_id=0, max_tx_frame[SSV_HW_TXQ_NUM];
    int txqid, tx_count, page_count, max_count, total_count=0;
    unsigned long flags;
    

    HCI_REG_READ(ctrl_hci, ADR_TX_ID_ALL_INFO, (u32 *)&txq_info);
    BUG_ON(SSV6200_PAGE_TX_THRESHOLD < txq_info.tx_use_page);
    BUG_ON(SSV6200_ID_TX_THRESHOLD < txq_info.tx_use_id);

    free_tx_page = SSV6200_PAGE_TX_THRESHOLD - txq_info.tx_use_page;
    free_tx_id   = SSV6200_ID_TX_THRESHOLD - txq_info.tx_use_id;
    max_tx_frame[4] = 100;
    max_tx_frame[3] = SSV6200_ID_AC_BK_OUT_QUEUE - txq_info.txq0_size;
    max_tx_frame[2] = SSV6200_ID_AC_BE_OUT_QUEUE - txq_info.txq1_size;
    max_tx_frame[1] = SSV6200_ID_AC_VI_OUT_QUEUE - txq_info.txq2_size;
    max_tx_frame[0] = SSV6200_ID_AC_VO_OUT_QUEUE - txq_info.txq3_size;

    for(txqid=SSV_HW_TXQ_NUM-1; txqid>=0; txqid--) {
        if (ctrl_hci->hci_start == false)
            break;
            hw_txq = &ctrl_hci->hw_txq[txqid];            
        max_count = 0;

            /**
               * Check to see if there is enough hardware resource for the
               * outgoing frame. If out of resource for the frame, stop frame
               * transmission and move to the next queue.
                      */
        skb_queue_walk(&hw_txq->qhead, skb) {
            //24 mean TX descriptor length
            page_count = (skb->len + SSV6200_ALLOC_RSVD - 24);
            if (page_count & HW_MMU_PAGE_MASK)
                page_count = (page_count >> HW_MMU_PAGE_SHIFT) + 1;
            else
                page_count = page_count >> HW_MMU_PAGE_SHIFT;
            if (max_tx_frame[txqid] <= 0)
                break;
            if (free_tx_page < page_count)
                break;
            if (free_tx_id <= 0)
                goto if_tx_out;
            free_tx_page -= page_count;
            free_tx_id --;
            max_count ++;
            total_count ++;
            max_tx_frame[txqid] --;
        }
        if (max_count > 0) {
            tx_count = ssv6xxx_hci_xmit(hw_txq, max_count);
            BUG_ON(tx_count != max_count);
        }
    }

if_tx_out:

    return total_count;
    
}

        
                
#endif

static int ssv6xxx_hci_tx_handler(void *dev, int max_count)
{
    struct ssv6xxx_hci_txq_info txq_info;
    struct ssv6xxx_hci_txq_info2 txq_info2; 
    struct ssv_hw_txq *hw_txq=dev;
    struct sk_buff *skb, *tmp_skb;
    u32 free_tx_page=0, free_tx_id=0;
    int max_tx_frame[SSV_HW_TXQ_NUM];
    int ret, count, tx_count, page_count;
    u32 retry_cnt=0;

XMIT_RTY:
    if(hw_txq->txq_no == 4){
        
        ret = HCI_REG_READ(ctrl_hci, ADR_TX_ID_ALL_INFO2, (u32 *)&txq_info2);
        if (ret < 0) {
            ctrl_hci->read_rs1_info_fail++;
            return 0;
        }
        
        BUG_ON(SSV6200_PAGE_TX_THRESHOLD < txq_info2.tx_use_page);
        BUG_ON(SSV6200_ID_TX_THRESHOLD < txq_info2.tx_use_id);
        free_tx_page = SSV6200_PAGE_TX_THRESHOLD - txq_info2.tx_use_page;
        free_tx_id   = SSV6200_ID_TX_THRESHOLD - txq_info2.tx_use_id;

        //    max_tx_frame[4] = SSV6200_ID_MANAGER_QUEUE-(txq_info.tx_use_id-txq_info.txq0_size-txq_info.txq1_size-txq_info.txq2_size-txq_info.txq3_size);
        max_tx_frame[4] = SSV6200_ID_MANAGER_QUEUE - txq_info2.txq4_size;        
    }
    else{
        
        ret = HCI_REG_READ(ctrl_hci, ADR_TX_ID_ALL_INFO, (u32 *)&txq_info);
        if (ret < 0) {
            ctrl_hci->read_rs0_info_fail++;
            return 0;
        }


        BUG_ON(SSV6200_PAGE_TX_THRESHOLD < txq_info.tx_use_page);
        BUG_ON(SSV6200_ID_TX_THRESHOLD < txq_info.tx_use_id);
        free_tx_page = SSV6200_PAGE_TX_THRESHOLD - txq_info.tx_use_page;
        free_tx_id   = SSV6200_ID_TX_THRESHOLD - txq_info.tx_use_id;        

        max_tx_frame[0] = SSV6200_ID_AC_BK_OUT_QUEUE - txq_info.txq0_size;
        max_tx_frame[1] = SSV6200_ID_AC_BE_OUT_QUEUE - txq_info.txq1_size;
        max_tx_frame[2] = SSV6200_ID_AC_VI_OUT_QUEUE - txq_info.txq2_size;
        max_tx_frame[3] = SSV6200_ID_AC_VO_OUT_QUEUE - txq_info.txq3_size;

        BUG_ON(max_tx_frame[3] < 0);
        BUG_ON(max_tx_frame[2] < 0);
        BUG_ON(max_tx_frame[1] < 0);
        BUG_ON(max_tx_frame[0] < 0);
    }                                

    tx_count = 0;

    /**
        * Check to see if there are enough hardware resource for the outgoing frames.
        * If out of resource for the frame, stop frame transmission. The final frame
        * transmission count shall meet the following conditions:
        *
        *    [1] enough page size for the frame
        *    [2] enough packet ID for the frame
        *    [3] more queue space to queue the outgoing frame
        *    [4] min(user_limit, resource_limie)
        */
    skb_queue_walk_safe(&hw_txq->qhead, skb, tmp_skb) {
        //Page offset 80  HCI reserve 16 * 3 ( 80 + 48)
        page_count = (skb->len + 128);
        if (page_count & HW_MMU_PAGE_MASK)
            page_count = (page_count >> HW_MMU_PAGE_SHIFT) + 1;
        else page_count = page_count >> HW_MMU_PAGE_SHIFT;
        
        if (page_count > (SSV6200_PAGE_TX_THRESHOLD / 2))
            printk(KERN_ERR "Asking page %d(%d) exceeds resource limit %d.\n",
                   page_count, skb->len, (SSV6200_PAGE_TX_THRESHOLD / 2));

        //if (max_tx_frame[hw_txq->txq_no] <= 0)
        //    break;
        if (free_tx_page < page_count)
            break;
        if (free_tx_id <= 0)
            break;
        if (tx_count >= max_count)
            break;
        free_tx_page -= page_count;
        free_tx_id --;
        tx_count ++;
        max_tx_frame[hw_txq->txq_no] --;
    }
    if (tx_count > 0) {
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if(ctrl_hci->irq_enable)
			ctrl_hci->real_tx_irq_count++;
#endif
        count = ssv6xxx_hci_xmit(hw_txq, tx_count);
        
        if(count != tx_count)
            printk("ssv6xxx_hci_xmit count[%d] tx_count[%d]\n", count, tx_count);

        WARN_ON(count != tx_count);
    }else{
        if((retry_cnt>0)&&(skb_queue_len(&hw_txq->qhead))){
            //printk("rty%d,%d,%d,%d,%d\n",retry_cnt,max_tx_frame[hw_txq->txq_no],free_tx_page,txq_info.tx_use_page,skb_queue_len(&hw_txq->qhead));
            retry_cnt--;
            //msleep(10);
            udelay(100);
            //yield();
            goto XMIT_RTY;
        }else{
            //printk("f,%d,%d,%d,%d\n",max_tx_frame[hw_txq->txq_no],free_tx_page,txq_info.tx_use_page,skb_queue_len(&hw_txq->qhead));
            //printk("fail\n");
        }
    }
    // Check if queue is empty, call empty callback so the AMPDU TX can send its buffered frames down.
#ifndef HCI_TX_POLLING
    if (   (ctrl_hci->shi->hci_tx_q_empty_cb != NULL) 
        && (skb_queue_len(&hw_txq->qhead) == 0))
    {
        ctrl_hci->shi->hci_tx_q_empty_cb(hw_txq->txq_no, ctrl_hci->shi->tx_q_empty_args);
    }
#endif    
    return tx_count;
}


void ssv6xxx_hci_tx_work(struct work_struct *work)
{
#ifdef CONFIG_SSV_TX_LOWTHRESHOLD

    // enable interrupt bit
    ssv6xxx_hci_irq_register(SSV6XXX_INT_RESOURCE_LOW);
    // Trigger TX interrupt from host
    //HCI_IRQ_TRIGGER(ctrl_hci);

#else
    int txqid;
#ifdef HCI_TX_POLLING
    struct ssv_hw_txq *hw_txq;
#endif
    for(txqid=SSV_HW_TXQ_NUM-1; txqid>=0; txqid--) {
#ifdef HCI_TX_POLLING
        /* check which worker wake up this work queue. */
        if (&ctrl_hci->hci_tx_work[txqid] != work)
            continue;

        hw_txq = &ctrl_hci->hw_txq[txqid];
        ssv6xxx_hci_tx_handler(hw_txq, 999);
        if(skb_queue_len(&hw_txq->qhead))
            queue_work(ctrl_hci->hci_work_queue,&ctrl_hci->hci_tx_work[txqid]);
#else 
       u32 bitno;
        
        /* check which worker wake up this work queue. */
        if (&ctrl_hci->hci_tx_work[txqid] != work)
            continue;

        /* enable interrupt bit */
        bitno = ssv6xxx_hci_get_int_bitno(txqid);         
        ssv6xxx_hci_irq_register(1<<(bitno));              
#endif
        break;
    }
#endif
}


static void ssv6xxx_hci_rx_work(struct work_struct *work)
{
    #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
    struct sk_buff_head rx_list;
    #endif // USE_THREAD_RX
    struct sk_buff *rx_mpdu;
    int rx_cnt, ret;
    size_t dlen;
    u32 status,rxq_len=0;
#ifdef CONFIG_SSV6XXX_DEBUGFS
    struct timespec     rx_io_start_time, rx_io_end_time, rx_io_diff_time;
    struct timespec     rx_proc_start_time, rx_proc_end_time, rx_proc_diff_time;
#endif // CONFIG_SSV6XXX_DEBUGFS

    ctrl_hci->rx_work_running = 1;
    #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
    skb_queue_head_init(&rx_list);
    #endif // USE_THREAD_RX
    status = SSV6XXX_INT_RX;
    for (rx_cnt = 0; (status & SSV6XXX_INT_RX) && (rx_cnt < 32/*999999*/); rx_cnt++) {
#ifdef CONFIG_SSV6XXX_DEBUGFS
        if (ctrl_hci->isr_mib_enable)
            getnstimeofday(&rx_io_start_time);
#endif // CONFIG_SSV6XXX_DEBUGFS

        ret = IF_RECV(ctrl_hci, ctrl_hci->rx_buf->data, &dlen);

#ifdef CONFIG_SSV6XXX_DEBUGFS
        if (ctrl_hci->isr_mib_enable)
            getnstimeofday(&rx_io_end_time);
#endif // CONFIG_SSV6XXX_DEBUGFS

        if (ret < 0 || dlen<=0) {
            printk("%s(): IF_RECV() retruns %d (dlen=%d)\n", __FUNCTION__,
                ret, (int)dlen);
            if (ret != -84 || dlen>MAX_FRAME_SIZE)
                break;
        }

        rx_mpdu = ctrl_hci->rx_buf;
        ctrl_hci->rx_buf = ssv_skb_alloc(MAX_FRAME_SIZE);
        if (ctrl_hci->rx_buf == NULL) {
            printk(KERN_ERR "RX buffer allocation failure!\n");
            ctrl_hci->rx_buf = rx_mpdu;
            break;
        }
        ctrl_hci->rx_pkt ++;
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if(ctrl_hci->irq_enable)
			ctrl_hci->irq_rx_pkt_count ++;
#endif
        if(dlen < MAX_FRAME_SIZE){
            skb_put(rx_mpdu, dlen);
        }else{
            printk("RX PKT > MAX_FRAME_SIZE len=%d!!\n",dlen);
            skb_put(rx_mpdu, MAX_FRAME_SIZE-1);
        }
            
#ifdef CONFIG_SSV6XXX_DEBUGFS
        if (ctrl_hci->isr_mib_enable)
            getnstimeofday(&rx_proc_start_time);
#endif // CONFIG_SSV6XXX_DEBUGFS
        #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
        __skb_queue_tail(&rx_list, rx_mpdu);
        #else
        rxq_len = ctrl_hci->shi->hci_rx_cb(rx_mpdu, ctrl_hci->shi->rx_cb_args);
        if(rxq_len > 2500){
            msleep(20);
            printk("!s = %d\n",rxq_len);
        }
        #endif // USE_THREAD_RX
        HCI_IRQ_STATUS(ctrl_hci, &status);
#ifdef CONFIG_SSV6XXX_DEBUGFS
        if (ctrl_hci->isr_mib_enable)
        {
            getnstimeofday(&rx_proc_end_time);
            ctrl_hci->isr_rx_io_count++;

            rx_io_diff_time = timespec_sub(rx_io_end_time, rx_io_start_time);
            ctrl_hci->isr_rx_io_time += timespec_to_ns(&rx_io_diff_time);

            rx_proc_diff_time = timespec_sub(rx_proc_end_time, rx_proc_start_time);
            ctrl_hci->isr_rx_proc_time += timespec_to_ns(&rx_proc_diff_time);
        }
#endif // CONFIG_SSV6XXX_DEBUGFS
    }
    #if !defined(USE_THREAD_RX) || defined(USE_BATCH_RX)
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    if (ctrl_hci->isr_mib_enable)
        getnstimeofday(&rx_proc_start_time);
    #endif // CONFIG_SSV6XXX_DEBUGFS
    ctrl_hci->shi->hci_rx_cb(&rx_list, ctrl_hci->shi->rx_cb_args);
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    if (ctrl_hci->isr_mib_enable)
    {
        getnstimeofday(&rx_proc_end_time);

        rx_proc_diff_time = timespec_sub(rx_proc_end_time, rx_proc_start_time);
        ctrl_hci->isr_rx_proc_time += timespec_to_ns(&rx_proc_diff_time);
    }
    #endif // CONFIG_SSV6XXX_DEBUGFS
    #endif // USE_THREAD_RX

    ctrl_hci->rx_work_running = 0;
}

#ifdef CONFIG_SSV6XXX_DEBUGFS
static void ssv6xxx_isr_mib_reset (void)
{
    ctrl_hci->isr_mib_reset = 0;
    ctrl_hci->isr_total_time = 0;
    ctrl_hci->isr_rx_io_time = 0;
    ctrl_hci->isr_tx_io_time = 0;
    ctrl_hci->isr_rx_io_count = 0;
    ctrl_hci->isr_tx_io_count = 0;
    ctrl_hci->isr_rx_proc_time =0;
}


bool ssv6xxx_hci_init_debugfs(struct dentry *dev_deugfs_dir)
{
    ctrl_hci->debugfs_dir = debugfs_create_dir("hci", dev_deugfs_dir);

    if (ctrl_hci->debugfs_dir == NULL)
    {
        dev_err(ctrl_hci->shi->dev, "Failed to create HCI debugfs directory.\n");
        return false;
    }

    debugfs_create_u32("hci_isr_mib_enable", 00644, ctrl_hci->debugfs_dir, &ctrl_hci->isr_mib_enable);
    debugfs_create_u32("hci_isr_mib_reset", 00666, ctrl_hci->debugfs_dir, &ctrl_hci->isr_mib_reset);
    debugfs_create_u64("isr_total_time", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_total_time);
    debugfs_create_u64("tx_io_time", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_tx_io_time);
    debugfs_create_u64("rx_io_time", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_rx_io_time);
    debugfs_create_u32("tx_io_count", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_tx_io_count);
    debugfs_create_u32("rx_io_count", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_rx_io_count);
    debugfs_create_u64("rx_proc_time", 00444, ctrl_hci->debugfs_dir, &ctrl_hci->isr_rx_proc_time);
    return true;
}

void ssv6xxx_hci_deinit_debugfs(void)
{
    if (ctrl_hci->debugfs_dir == NULL)
        return;

    //debugfs_remove_recursive(ctrl_hci->debugfs_dir);
    ctrl_hci->debugfs_dir = NULL;
}
#endif // CONFIG_SSV6XXX_DEBUGFS
//--------------------------------------------------------------
//                              ssv6xxx_hci_isr
//--------------------------------------------------------------
irqreturn_t ssv6xxx_hci_isr(int irq, void *args)
{
    struct ssv6xxx_hci_ctrl *hctl=args;
    struct ssv_hw_txq *hw_txq;
    u32 status, regval;
    unsigned long flags;
    int q_num;
    int ret = IRQ_HANDLED;    
	//==============================================================>
	//DEBUG CODE
	bool dbg_isr_miss = true;
    if(ctrl_hci->isr_summary_eable
        && ctrl_hci->prev_isr_jiffes){

        if(ctrl_hci->isr_idle_time){                
            ctrl_hci->isr_idle_time += (jiffies - ctrl_hci->prev_isr_jiffes);
            ctrl_hci->isr_idle_time = ctrl_hci->isr_idle_time >>1;
        }
        else{
            ctrl_hci->isr_idle_time += (jiffies - ctrl_hci->prev_isr_jiffes);
        }
    }
	//<==============================================================
    
    BUG_ON(!args);
    do {
#ifdef CONFIG_SSV6XXX_DEBUGFS
        struct timespec  start_time, end_time, diff_time;
        struct timespec  tx_io_start_time, tx_io_end_time, tx_io_diff_time;

        if (hctl->isr_mib_reset)
            ssv6xxx_isr_mib_reset();

        if (hctl->isr_mib_enable)
            getnstimeofday(&start_time);
#endif // CONFIG_SSV6XXX_DEBUGFS
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if(ctrl_hci->irq_enable)
        	ctrl_hci->irq_count++;
#endif
        mutex_lock(&hctl->hci_mutex);
        ret = HCI_IRQ_STATUS(hctl, &status);
        spin_lock_irqsave(&hctl->int_lock, flags);
        status &= hctl->int_mask;
        if (ret<0 || status==0) {            
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if(ctrl_hci->irq_enable)
			ctrl_hci->invalid_irq_count++;
#endif
            //printk("get irq status[%d] status[0x%08x]\n", ret, status);
            //WARN_ON(1);                      
            spin_unlock_irqrestore(&hctl->int_lock, flags);
            mutex_unlock(&hctl->hci_mutex);
            ret = IRQ_NONE;
            break;
        }
        //else
            //printk("get irq status[%d] status[0x%08x]\n", ret, status);
        
        spin_unlock_irqrestore(&hctl->int_lock, flags);
        mutex_unlock(&hctl->hci_mutex);
        
        ctrl_hci->isr_running = 1;

        /* handle Rx interrupt */
        if (status & SSV6XXX_INT_RX){
			//==============================================================>
			//DEBUG CODE
            u32 before = jiffies;
#ifdef CONFIG_IRQ_DEBUG_COUNT	
			if(ctrl_hci->irq_enable)
            	ctrl_hci->rx_irq_count++;
#endif
            if(ctrl_hci->isr_summary_eable
                && ctrl_hci->prev_rx_isr_jiffes){


               if(ctrl_hci->isr_rx_idle_time){ 
                    ctrl_hci->isr_rx_idle_time += (jiffies - ctrl_hci->prev_rx_isr_jiffes);
                    ctrl_hci->isr_rx_idle_time = ctrl_hci->isr_rx_idle_time >>1;
               }
               else{
                    //first time
                    ctrl_hci->isr_rx_idle_time += (jiffies - ctrl_hci->prev_rx_isr_jiffes);
               }
               
            }
			//<==============================================================

            
            ssv6xxx_hci_rx_work(&hctl->hci_rx_work);
			
			//==============================================================>
			//DEBUG CODE
			
            dbg_isr_miss = false;
            if(ctrl_hci->isr_summary_eable){

				//caculate how many time spend in handling rx frames
                if(ctrl_hci->isr_rx_time){
                    ctrl_hci->isr_rx_time += (jiffies-before);
                    ctrl_hci->isr_rx_time = ctrl_hci->isr_rx_time >>1;
                }
                else{
                    ctrl_hci->isr_rx_time += (jiffies-before);
                }
                                
				//log timestamp for calculating no interrupt space.
                ctrl_hci->prev_rx_isr_jiffes = jiffies;
                
            }
			//<=============================================================				
				
        }
       
        /* handle TX interrupt */
#ifdef CONFIG_IRQ_DEBUG_COUNT
		if ((!(status & SSV6XXX_INT_RX))&& ctrl_hci->irq_enable)
			ctrl_hci->tx_irq_count++;
#endif

#ifdef CONFIG_SSV_TX_LOWTHRESHOLD
        if (status & (SSV6XXX_INT_RESOURCE_LOW ))
        {
            u32 enableFlag=0;
            for(q_num = SSV_HW_TXQ_NUM-1; q_num>=0; q_num--) {
                //==============================================================>
                //DEBUG CODE
                u32 before = jiffies;
                //<==============================================================
    
                hw_txq = &hctl->hw_txq[q_num];

    
                dbg_isr_miss = false;
    
                #ifdef CONFIG_SSV6XXX_DEBUGFS
                if (ctrl_hci->isr_mib_enable)
                    getnstimeofday(&tx_io_start_time);
                #endif // CONFIG_SSV6XXX_DEBUGFS
    
                /* xmit tx frames */
                ssv6xxx_hci_tx_handler(hw_txq, 999);
 
                #ifdef CONFIG_SSV6XXX_DEBUGFS
                if (ctrl_hci->isr_mib_enable)
                {
                    getnstimeofday(&tx_io_end_time);
    
                    tx_io_diff_time = timespec_sub(tx_io_end_time, tx_io_start_time);
                    ctrl_hci->isr_tx_io_time += timespec_to_ns(&tx_io_diff_time);
                }
                #endif // CONFIG_SSV6XXX_DEBUGFS
    
                //==============================================================>
                //DEBUG CODE
                if (skb_queue_len(&hw_txq->qhead) > 0)
                    enableFlag = 1;
    
                if(ctrl_hci->isr_summary_eable){
                    //caculate how many time spend in handling tx frames
                    if(ctrl_hci->isr_tx_time){
                        ctrl_hci->isr_tx_time += (jiffies-before);
                        ctrl_hci->isr_tx_time = ctrl_hci->isr_tx_time >>1;
                    }
                    else{
                        ctrl_hci->isr_tx_time += (jiffies-before);
                    }
                }
            }

            mutex_lock(&hctl->hci_mutex);
            spin_lock_irqsave(&hctl->int_lock, flags);
            /*check if need to disable interrupt */
            if(enableFlag)
            {
                hctl->int_mask &= ~SSV6XXX_INT_TX;
                regval = ~hctl->int_mask;
                spin_unlock_irqrestore(&hctl->int_lock, flags);

                HCI_IRQ_SET_MASK(hctl, regval);
            }
            else
            {
                hctl->int_mask &= ~(SSV6XXX_INT_RESOURCE_LOW | SSV6XXX_INT_TX);
                regval = ~hctl->int_mask;
                spin_unlock_irqrestore(&hctl->int_lock, flags);

                HCI_IRQ_SET_MASK(hctl, regval);
            }
            mutex_unlock(&hctl->hci_mutex);
        }
#else
        for(q_num = SSV_HW_TXQ_NUM-1; q_num>=0; q_num--) {
            int bitno;
            //==============================================================>
			//DEBUG CODE
            u32 before = jiffies;
      		//<==============================================================

            hw_txq = &hctl->hw_txq[q_num];
            bitno = ssv6xxx_hci_get_int_bitno(hw_txq->txq_no);

            if ((status & (1<<bitno)) == 0)
                continue;

            dbg_isr_miss = false;

            #ifdef CONFIG_SSV6XXX_DEBUGFS
            if (ctrl_hci->isr_mib_enable)
                getnstimeofday(&tx_io_start_time);
            #endif // CONFIG_SSV6XXX_DEBUGFS

            /* xmit tx frames */
            ssv6xxx_hci_tx_handler(hw_txq, 999);

            mutex_lock(&hctl->hci_mutex);
            spin_lock_irqsave(&hctl->int_lock, flags);

            /*check if need to disable interrupt */
            if (skb_queue_len(&hw_txq->qhead) <= 0) {
                hctl->int_mask &= ~(1<<bitno);
                regval = ~hctl->int_mask;
                spin_unlock_irqrestore(&hctl->int_lock, flags);

                HCI_IRQ_SET_MASK(hctl, regval);
            }
            else spin_unlock_irqrestore(&hctl->int_lock, flags);

            mutex_unlock(&hctl->hci_mutex);

            #ifdef CONFIG_SSV6XXX_DEBUGFS
            if (ctrl_hci->isr_mib_enable)
            {
                getnstimeofday(&tx_io_end_time);

                tx_io_diff_time = timespec_sub(tx_io_end_time, tx_io_start_time);
                ctrl_hci->isr_tx_io_time += timespec_to_ns(&tx_io_diff_time);
            }
            #endif // CONFIG_SSV6XXX_DEBUGFS

            //==============================================================>
            //DEBUG CODE

            if(ctrl_hci->isr_summary_eable){
                //caculate how many time spend in handling tx frames
                if(ctrl_hci->isr_tx_time){
                    ctrl_hci->isr_tx_time += (jiffies-before);
                    ctrl_hci->isr_tx_time = ctrl_hci->isr_tx_time >>1;
                }
                else{
                    ctrl_hci->isr_tx_time += (jiffies-before);
                }
            }
        }
#endif

        ctrl_hci->isr_running = 0;
#ifdef CONFIG_SSV6XXX_DEBUGFS
        if (ctrl_hci->isr_mib_enable)
        {
            getnstimeofday(&end_time);

            diff_time = timespec_sub(end_time, start_time);
            ctrl_hci->isr_total_time += timespec_to_ns(&diff_time);
        }
#endif // CONFIG_SSV6XXX_DEBUGFS
    }while(0);

    //==============================================================>
    //DEBUG CODE
    if(ctrl_hci->isr_summary_eable ){
        if(dbg_isr_miss)
			ctrl_hci->isr_miss_cnt++;

    	ctrl_hci->prev_isr_jiffes = jiffies;
	}
    //<==============================================================

    
    return ret;
}

static struct ssv6xxx_hci_ops hci_ops = 
{
    .hci_start            = ssv6xxx_hci_start,
    .hci_stop             = ssv6xxx_hci_stop,
    .hci_read_word        = ssv6xxx_hci_read_word,
    .hci_write_word       = ssv6xxx_hci_write_word,
    .hci_tx               = ssv6xxx_hci_enqueue,
    .hci_tx_pause         = ssv6xxx_hci_txq_pause,
    .hci_tx_resume        = ssv6xxx_hci_txq_resume,
    .hci_txq_flush        = ssv6xxx_hci_txq_flush,
    .hci_txq_flush_by_sta = ssv6xxx_hci_txq_flush_by_sta,
    .hci_txq_empty        = ssv6xxx_hci_is_txq_empty,
    .hci_load_fw          = ssv6xxx_hci_load_fw,
    .hci_pmu_wakeup       = ssv6xxx_hci_pmu_wakeup,
    .hci_send_cmd         = ssv6xxx_hci_send_cmd,
    .hci_init_debugfs     = ssv6xxx_hci_init_debugfs,
    .hci_deinit_debugfs   = ssv6xxx_hci_deinit_debugfs,
};




int ssv6xxx_hci_deregister(void)
{
    u32 regval;

    /**
        * Wait here until there is no frame on the hardware. Before
        * call this function, the RF shall be turned off to make sure
        * no more incoming frames. This function also disable interrupt
        * once no more frames.
        */
    printk("%s(): \n", __FUNCTION__);


    if (ctrl_hci->shi == NULL)
        return -1;
    
    regval = 1;
//    sido module may release before.
//    /* check mcu/hci/fragment/security/mrx/mic */
//    while(!regval) {
//        if(ctrl_hci->shi->if_ops->readreg(
//        ctrl_hci->shi->dev, ADR_RD_FFOUT_CNT1, 
//        &regval));
//        mdelay(10);
//    };
    ssv6xxx_hci_irq_disable();
    flush_workqueue(ctrl_hci->hci_work_queue);
    destroy_workqueue(ctrl_hci->hci_work_queue);
    ctrl_hci->shi = NULL;
      
    return 0;
}
EXPORT_SYMBOL(ssv6xxx_hci_deregister);



int ssv6xxx_hci_register(struct ssv6xxx_hci_info *shi)
{
    int i;
    
    if (shi == NULL || ctrl_hci->shi)
        return -1;

    /* HCI & hw/sw mac interface binding */
    shi->hci_ops = &hci_ops;
    ctrl_hci->shi = shi;
    
    ctrl_hci->txq_mask = 0;
    mutex_init(&ctrl_hci->txq_mask_lock);
    mutex_init(&ctrl_hci->hci_mutex);

    spin_lock_init(&ctrl_hci->int_lock);
#ifdef CONFIG_IRQ_DEBUG_COUNT
	ctrl_hci->irq_enable = false;
	ctrl_hci->irq_count = 0;
	ctrl_hci->invalid_irq_count = 0;
	ctrl_hci->tx_irq_count = 0;
	ctrl_hci->real_tx_irq_count = 0;
	ctrl_hci->rx_irq_count = 0;
	ctrl_hci->irq_rx_pkt_count = 0;
	ctrl_hci->irq_tx_pkt_count = 0;
#endif	
    /* TX queue initialization */
    for(i=0; i<SSV_HW_TXQ_NUM; i++) {
        memset(&ctrl_hci->hw_txq[i], 0, sizeof(struct ssv_hw_txq));
        skb_queue_head_init(&ctrl_hci->hw_txq[i].qhead);
        //spin_lock_init(&ctrl_hci->hw_txq[i].txq_lock);
        ctrl_hci->hw_txq[i].txq_no = (u32)i;
        ctrl_hci->hw_txq[i].max_qsize = SSV_HW_TXQ_MAX_SIZE;
        ctrl_hci->hw_txq[i].resum_thres = SSV_HW_TXQ_RESUME_THRES;
    }

    ctrl_hci->hci_work_queue = create_singlethread_workqueue("ssv6xxx_hci_wq");
    for(i=0; i<SSV_HW_TXQ_NUM; i++)
        INIT_WORK(&ctrl_hci->hci_tx_work[i], ssv6xxx_hci_tx_work);

    INIT_WORK(&ctrl_hci->hci_rx_work, ssv6xxx_hci_rx_work);

#ifdef CONFIG_SSV_TX_LOWTHRESHOLD
    /* set irq mask & register irq handler */
    ctrl_hci->int_mask = SSV6XXX_INT_RX|SSV6XXX_INT_RESOURCE_LOW;
#else
    /* set irq mask & register irq handler */
    ctrl_hci->int_mask = SSV6XXX_INT_RX|SSV6XXX_INT_TX|SSV6XXX_INT_LOW_EDCA_0|
        SSV6XXX_INT_LOW_EDCA_1|SSV6XXX_INT_LOW_EDCA_2|SSV6XXX_INT_LOW_EDCA_3;
#endif
    ctrl_hci->int_status= 0;
    HCI_IRQ_SET_MASK(ctrl_hci, 0xFFFFFFFF);
    ssv6xxx_hci_irq_disable();
    HCI_IRQ_REQUEST(ctrl_hci, ssv6xxx_hci_isr);
    #ifdef CONFIG_SSV6XXX_DEBUGFS
    ctrl_hci->debugfs_dir = NULL;
    ctrl_hci->isr_mib_enable = false;
    ctrl_hci->isr_mib_reset = 0;
    ctrl_hci->isr_total_time = 0;
    ctrl_hci->isr_rx_io_time = 0;
    ctrl_hci->isr_tx_io_time = 0;
    ctrl_hci->isr_rx_io_count = 0;
    ctrl_hci->isr_tx_io_count = 0;
    ctrl_hci->isr_rx_proc_time =0;
    #endif // CONFIG_SSV6XXX_DEBUGFS
    return 0;
}
EXPORT_SYMBOL(ssv6xxx_hci_register);











#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
int ssv6xxx_hci_init(void)
#else
static int __init ssv6xxx_hci_init(void)
#endif
{
#ifdef CONFIG_SSV6200_CLI_ENABLE
    extern struct ssv6xxx_hci_ctrl *ssv_dbg_ctrl_hci;
#endif
    ctrl_hci = kzalloc(sizeof(*ctrl_hci), GFP_KERNEL);
    if (ctrl_hci==NULL)
        return -ENOMEM;
    
    memset((void *)ctrl_hci, 0, sizeof(*ctrl_hci));
    ctrl_hci->rx_buf = ssv_skb_alloc(MAX_FRAME_SIZE);
    if (ctrl_hci->rx_buf == NULL) {
        kfree(ctrl_hci);
        return -ENOMEM;
    }    
#ifdef CONFIG_SSV6200_CLI_ENABLE
    ssv_dbg_ctrl_hci = ctrl_hci;
#endif
    return 0;
}

#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
void ssv6xxx_hci_exit(void)
#else
static void __exit ssv6xxx_hci_exit(void)
#endif
{
#ifdef CONFIG_SSV6200_CLI_ENABLE
    extern struct ssv6xxx_hci_ctrl *ssv_dbg_ctrl_hci;
#endif
    kfree(ctrl_hci);    
    ctrl_hci = NULL;
#ifdef CONFIG_SSV6200_CLI_ENABLE
    ssv_dbg_ctrl_hci = NULL;
#endif   
}


#if (defined(CONFIG_SSV_SUPPORT_ANDROID)||defined(CONFIG_SSV_BUILD_AS_ONE_KO))
EXPORT_SYMBOL(ssv6xxx_hci_init);
EXPORT_SYMBOL(ssv6xxx_hci_exit);
#else
module_init(ssv6xxx_hci_init);
module_exit(ssv6xxx_hci_exit);
#endif




