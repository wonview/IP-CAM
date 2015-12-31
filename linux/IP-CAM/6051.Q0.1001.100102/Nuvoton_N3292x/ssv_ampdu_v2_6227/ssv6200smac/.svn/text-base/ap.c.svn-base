#include <ssv6200.h>

#include <linux/nl80211.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/sched.h>

#include <net/mac80211.h>

#include <ssv6200.h>
#include "lib.h"
#include "dev.h"
#include "ap.h"
#include "ssv_rc_common.h"
#include "ssv_rc.h"

int ssv6200_bcast_queue_len(struct ssv6xxx_bcast_txq *bcast_txq);



//-------------------------------------
//Beacon related function
#define IS_EQUAL(a, b)                  ( (a) == (b) )
#define SET_BIT(v, b)					( (v) |= (0x01<<b) )
#define CLEAR_BIT(v, b)			    	( (v) &= ~(0x01<<b) )
#define IS_BIT_SET(v, b)				( (v) & (0x01<<(b) ) )

#define PBUF_BASE_ADDR	            0x80000000
#define PBUF_ADDR_SHIFT	            16

#define PBUF_MapPkttoID(_PKT)		(((u32)_PKT&0x0FFF0000)>>PBUF_ADDR_SHIFT)	
#define PBUF_MapIDtoPkt(_ID)		(PBUF_BASE_ADDR|((_ID)<<PBUF_ADDR_SHIFT))



#define SSV6xxx_BEACON_MAX_ALLOCATE_CNT	   10


//=====>ADR_MTX_BCN_EN_MISC
#define MTX_BCN_PKTID_CH_LOCK_SHIFT MTX_BCN_PKTID_CH_LOCK_SFT 	 															//bit 0

#define MTX_BCN_CFG_VLD_SHIFT MTX_BCN_CFG_VLD_SFT 																			// 1
#define MTX_BCN_CFG_VLD_MASK  MTX_BCN_CFG_VLD_MSK 																			//0x00000007  //just see 3 bits

#define AUTO_BCN_ONGOING_MASK  MTX_AUTO_BCN_ONGOING_MSK 																	//0x00000008
#define AUTO_BCN_ONGOING_SHIFT MTX_AUTO_BCN_ONGOING_SFT 																	// 3



#define MTX_BCN_TIMER_EN_SHIFT				MTX_BCN_TIMER_EN_SFT															//	0
#define MTX_TSF_TIMER_EN_SHIFT				MTX_TSF_TIMER_EN_SFT															//	5
#define MTX_HALT_MNG_UNTIL_DTIM_SHIFT 		MTX_HALT_MNG_UNTIL_DTIM_SFT														//	6
#define MTX_BCN_ENABLE_MASK					(MTX_BCN_TIMER_EN_I_MSK&MTX_TSF_TIMER_EN_I_MSK&MTX_HALT_MNG_UNTIL_DTIM_I_MSK)  	//0xffffff9e





//=====>


//=====>ADR_MTX_BCN_PRD
#define MTX_BCN_PERIOD_SHIFT 	MTX_BCN_PERIOD_SFT		// 0			//bit0~7
#define MTX_DTIM_NUM_SHIFT 		MTX_DTIM_NUM_SFT		// 24			//bit 24~bit31
//=====>



//=====>ADR_MTX_BCN_CFG0/ADR_MTX_BCN_CFG1
#define MTX_DTIM_OFST0			MTX_DTIM_OFST0_SFT		// 16
//=====>






enum ssv6xxx_beacon_type{
	SSV6xxx_BEACON_0,
	SSV6xxx_BEACON_1,
};

static const u32 ssv6xxx_beacon_adr[] =
{
	ADR_MTX_BCN_CFG0,
    ADR_MTX_BCN_CFG1,
};




void ssv6xxx_beacon_reg_lock(struct ssv_softc *sc, bool block)
{
	u32 val;
	
	val = block<<MTX_BCN_PKTID_CH_LOCK_SHIFT;
#ifdef BEACON_DEBUG	    
	printk("ssv6xxx_beacon_reg_lock   val[0x:%08x]\n ", val);
#endif	
	SMAC_REG_WRITE(sc->sh, ADR_MTX_BCN_MISC, val);
}


void ssv6xxx_beacon_set_info(struct ssv_softc *sc, u8 beacon_interval, u8 dtim_cnt)
{
	u32 val;

	//if default is 0 set to our default
	if(beacon_interval==0)
		beacon_interval = 100;


#ifdef BEACON_DEBUG	
	printk("[A] BSS_CHANGED_BEACON_INT beacon_int[%d] dtim_cnt[%d]\n",beacon_interval, (dtim_cnt));
#endif
	val = (beacon_interval<<MTX_BCN_PERIOD_SHIFT)| (dtim_cnt<<MTX_DTIM_NUM_SHIFT);
	SMAC_REG_WRITE(sc->sh, ADR_MTX_BCN_PRD, val);
}



//0-->success others-->fail
bool ssv6xxx_beacon_enable(struct ssv_softc *sc, bool bEnable)
{
	u32 regval=0;
	int ret;

	//If there is no beacon set to register, beacon could not be turn on.
	if(bEnable && !sc->beacon_usage)
	{
		printk("[A] ssv6xxx_beacon_enable bEnable[%d] sc->beacon_usage[%d]\n",bEnable ,sc->beacon_usage);
		return -1;
	}

	if(bEnable == sc->enable_beacon)
	{
		printk("[A] ssv6xxx_beacon_enable bEnable[%d] and sc->enable_beacon[%d] are the same. no need to execute.\n",bEnable ,sc->enable_beacon);
		//return -1;
	}
	
	SMAC_REG_READ(sc->sh, ADR_MTX_BCN_EN_MISC, &regval);
#ifdef BEACON_DEBUG		
	printk("[A] ssv6xxx_beacon_enable read misc reg val [%08x]\n", regval);
#endif
	regval&= MTX_BCN_ENABLE_MASK;
#ifdef BEACON_DEBUG		
	printk("[A] ssv6xxx_beacon_enable read misc reg val [%08x]\n", regval);
#endif

	regval|=(bEnable<<MTX_BCN_TIMER_EN_SHIFT)|
		(bEnable<<MTX_TSF_TIMER_EN_SHIFT) |
		(bEnable<<MTX_HALT_MNG_UNTIL_DTIM_SHIFT);

	ret = SMAC_REG_WRITE(sc->sh, ADR_MTX_BCN_EN_MISC, regval);
#ifdef BEACON_DEBUG	
	printk("[A] ssv6xxx_beacon_enable read misc reg val [%08x]\n", regval);
#endif

    sc->enable_beacon = bEnable;

	return ret;
		
}









int ssv6xxx_beacon_fill_content(struct ssv_softc *sc, u32 regaddr, u8 *beacon, int size)
{
	u32 i, val;
	u32 *ptr = (u32*)beacon;
	size = size/4;

	for(i=0; i<size; i++)
	{
		val = (u32)(*(ptr+i));
#ifdef BEACON_DEBUG	
		printk("[%08x] ", val );
#endif			
		SMAC_REG_WRITE(sc->sh, regaddr+i*4, val);
	}
#ifdef BEACON_DEBUG	    
		printk("\n");
#endif	
    return 0;
}





void ssv6xxx_beacon_fill_tx_desc(struct ssv_softc *sc, struct sk_buff* beacon_skb)
{
	struct ieee80211_tx_info *tx_info = IEEE80211_SKB_CB(beacon_skb);
	struct ssv6200_tx_desc *tx_desc;
	u16 pb_offset = sc->sh->cfg.txpb_offset;
	struct ssv_rate_info ssv_rate;

	
	/* Insert description space */	
	skb_push(beacon_skb, pb_offset);
	
	tx_desc = (struct ssv6200_tx_desc *)beacon_skb->data;	
	memset(tx_desc,0, pb_offset);

	ssv6xxx_rc_hw_rate_idx(sc, tx_info, &ssv_rate);
	
	//length
    tx_desc->len            = beacon_skb->len-pb_offset;
	tx_desc->c_type         = M2_TXREQ;
    tx_desc->f80211         = 1;
	tx_desc->ack_policy     = 1;//no ack;	
    tx_desc->hdr_offset 	= pb_offset;					
    tx_desc->hdr_len 		= 24;									
    tx_desc->payload_offset = tx_desc->hdr_offset + tx_desc->hdr_len;
	tx_desc->crate_idx      = ssv_rate.crate_hw_idx;
	tx_desc->drate_idx      = ssv_rate.drate_hw_idx;

	/* Insert 4 bytes for FCS*/
    skb_put(beacon_skb, 4);	
}










// 00: bcn cfg0 / cfg1 not be config 01: cfg0 is valid 10: cfg1 is valid 11: error (valid cfg be write) 
inline enum ssv6xxx_beacon_type ssv6xxx_beacon_get_valid_reg(struct ssv_softc *sc)
{
	u32 regval =0;
	SMAC_REG_READ(sc->sh, ADR_MTX_BCN_MISC, &regval);

	
	regval &= MTX_BCN_CFG_VLD_MASK;
	regval = regval >>MTX_BCN_CFG_VLD_SHIFT;



	//get MTX_BCN_CFG_VLD
	
	if(regval==0x2 || regval == 0x0)//bcn 0 is availabke to use.
		return SSV6xxx_BEACON_0;
	else if(regval==0x1)//bcn 1 is availabke to use.
		return SSV6xxx_BEACON_1;
	else
		printk("=============>ERROR!!drv_bcn_reg_available\n");//ASSERT(FALSE);// 11 error happened need check with ASIC.

		
	return SSV6xxx_BEACON_0;
}



//Assume Beacon come from one sk_buf
//Beacon
bool ssv6xxx_beacon_set(struct ssv_softc *sc, struct sk_buff *beacon_skb, int dtim_offset)
{
	u32 reg_tx_beacon_adr = ADR_MTX_BCN_CFG0;
	enum ssv6xxx_beacon_type avl_bcn_type = SSV6xxx_BEACON_0;
	bool ret = true;
	int val;

	//Lock HW BCN module.
	//SET_MTX_BCN_PKTID_CH_LOCK(1);
	ssv6xxx_beacon_reg_lock(sc, 1);

	
	//if(beacon_skb->len != beacon_skb->data_len)
	//{
	//	ret = false;
	//	printk("=============>ERROR!! Beacon store in more than one sk_buff: beacon_skb->len[%d] beacon_skb->data_len[%d]....Need to implement\n", 
	//		beacon_skb->len, beacon_skb->data_len);
	//	goto out;
	//}

	//1.Decide which register can be used to set
	avl_bcn_type = ssv6xxx_beacon_get_valid_reg(sc);
	if(avl_bcn_type == SSV6xxx_BEACON_1)
		reg_tx_beacon_adr = ADR_MTX_BCN_CFG1;

#ifdef BEACON_DEBUG		
	printk("[A] ssv6xxx_beacon_set avl_bcn_type[%d]\n", avl_bcn_type);
#endif	
	//2.Get Pbuf from ASIC
	do{
		
		if(IS_BIT_SET(sc->beacon_usage, avl_bcn_type))
		{
#ifdef BEACON_DEBUG	
			printk("[A] beacon has already been set old len[%d] new len[%d]\n", sc->beacon_info[avl_bcn_type].len, beacon_skb->len);
#endif
			if (sc->beacon_info[avl_bcn_type].len >= beacon_skb->len)
			{
				break;
			}
			else
			{
				//old beacon too small, need to free
				if(false == ssv6xxx_pbuf_free(sc, sc->beacon_info[avl_bcn_type].pubf_addr))
				{
#ifdef BEACON_DEBUG	
					printk("=============>ERROR!!Intend to allcoate beacon from ASIC fail.\n");
#endif
					ret = false;
					goto out;
				}				
				CLEAR_BIT(sc->beacon_usage, avl_bcn_type);				
			}			
		}


		//Allocate new one
		//sc->beacon_info[avl_bcn_type].pubf_addr = ssv6xxx_pbuf_alloc(sc, beacon_skb->len);
        sc->beacon_info[avl_bcn_type].pubf_addr = ssv6xxx_pbuf_alloc(sc, beacon_skb->len, TX_BUF);
		sc->beacon_info[avl_bcn_type].len = beacon_skb->len;

		//if can't allocate beacon, just leave.
		if(sc->beacon_info[avl_bcn_type].pubf_addr == 0)
		{
			ret = false;
			goto out;
		}
						
		//Indicate reg is stored packet buf.
		SET_BIT(sc->beacon_usage, avl_bcn_type);
#ifdef BEACON_DEBUG	
		printk("[A] beacon type[%d] usage[%d] allocate new beacon addr[%08x] \n", avl_bcn_type, sc->beacon_usage, sc->beacon_info[avl_bcn_type].pubf_addr);
#endif
	}while(0);
	
	
	//3. Write Beacon content.
	ssv6xxx_beacon_fill_content(sc, sc->beacon_info[avl_bcn_type].pubf_addr, beacon_skb->data, beacon_skb->len);



	//4. Assign to register let tx know. Beacon is updated.	
	val = (PBUF_MapPkttoID(sc->beacon_info[avl_bcn_type].pubf_addr))|(dtim_offset<<MTX_DTIM_OFST0);	
	SMAC_REG_WRITE(sc->sh, reg_tx_beacon_adr,  val);
#ifdef BEACON_DEBUG	
	printk("[A] update to register reg_tx_beacon_adr[%08x] val[%08x]\n", reg_tx_beacon_adr, val);
#endif
out:
	ssv6xxx_beacon_reg_lock(sc, 0);



	return ret;
}



inline bool ssv6xxx_auto_bcn_ongoing(struct ssv_softc *sc)
{
	u32 regval;

	SMAC_REG_READ(sc->sh, ADR_MTX_BCN_MISC, &regval);
	
	return ((AUTO_BCN_ONGOING_MASK&regval)>>AUTO_BCN_ONGOING_SHIFT);
}



//need to stop beacon firstly.
void ssv6xxx_beacon_release(struct ssv_softc *sc)
{
//#ifdef BEACON_DEBUG		
	printk("[A] ssv6xxx_beacon_release Enter\n");
//#endif
	do{
		if(ssv6xxx_auto_bcn_ongoing(sc))
			ssv6xxx_beacon_enable(sc, false);
		else
			break;
					
	}while(1);

			
	if(IS_BIT_SET(sc->beacon_usage, SSV6xxx_BEACON_0))
	{
		ssv6xxx_pbuf_free(sc, sc->beacon_info[SSV6xxx_BEACON_0].pubf_addr);
		CLEAR_BIT(sc->beacon_usage, SSV6xxx_BEACON_0);	
	}
	//else
	//	printk("=============>ERROR!! release ");


	if(IS_BIT_SET(sc->beacon_usage, SSV6xxx_BEACON_1))
	{	
		ssv6xxx_pbuf_free(sc, sc->beacon_info[SSV6xxx_BEACON_1].pubf_addr);
		CLEAR_BIT(sc->beacon_usage, SSV6xxx_BEACON_1);
	}

	sc->enable_beacon = false;

    if(sc->beacon_buf){    
        dev_kfree_skb_any(sc->beacon_buf);
        sc->beacon_buf = NULL;
    }
#ifdef BEACON_DEBUG	    
	printk("[A] ssv6xxx_beacon_release leave\n");
#endif	
}




void ssv6xxx_beacon_change(struct ssv_softc *sc, struct ieee80211_hw *hw, struct ieee80211_vif *vif, bool aid0_bit_set)
{
	//struct ssv_hw *sh = sc->sh;
	struct sk_buff *skb;
    struct sk_buff *old_skb = NULL;
	u16 tim_offset, tim_length;
    
	
//	printk("[A] ssv6xxx_beacon_change\n");

    if(sc == NULL || hw == NULL || vif == NULL ){
        printk("[Error]........ssv6xxx_beacon_change input error\n");
		return;
    }

    do{

    	skb = ieee80211_beacon_get_tim(hw, vif,
    			&tim_offset, &tim_length);

        if(skb == NULL){
            printk("[Error]........skb is NULL\n");
            break;
        }

    	if (tim_offset && tim_length >= 6) {
    	/* Ignore DTIM count from mac80211:
    		 * firmware handles DTIM internally.
    		 */
    		skb->data[tim_offset + 2] = 0;

    		/* Set/reset aid0 bit */
    		if (aid0_bit_set)
    			skb->data[tim_offset + 4] |= 1;
    		else
    			skb->data[tim_offset + 4] &= ~1;
    	}

        if(sc->beacon_buf)
        {
            struct ssv6200_tx_desc *tx_desc = (struct ssv6200_tx_desc *)sc->beacon_buf->data;                            
            /* 
                    * Compare if content is the same
                    */
            if(memcmp(sc->beacon_buf->data+tx_desc->hdr_offset, skb->data, skb->len) == 0){                
                /* no need set new beacon to register*/
                old_skb = skb;
                break;
            }
            else{                
                 old_skb = sc->beacon_buf;
                 sc->beacon_buf = skb;
            }        
        }
        else{
            sc->beacon_buf = skb;
        }
        
    //for debug 
    //	{
    //		int i;
    //		u8 *ptr = &skb->data[tim_offset];
    //		printk("=================DTIM===================\n");	
    //		for(i=0;i<tim_length;i++)
    //			printk("%08x ", *(ptr+i));
    //		printk("\n=======================================\n");
    //	}
    //	

    		

    	//ASIC need to know the position of DTIM count. therefore add 2 bytes more.
    	tim_offset+=2;
#ifdef BEACON_DEBUG	    	
    	printk("[A] beacon len [%d] tim_offset[%d]\n", skb->len, tim_offset);
#endif
    	ssv6xxx_beacon_fill_tx_desc(sc, skb);
#ifdef BEACON_DEBUG	
    	printk("[A] beacon len [%d] tim_offset[%d]\n", skb->len, tim_offset);
#endif
    	
    	
    	if(ssv6xxx_beacon_set(sc, skb, tim_offset))
    	{

    		u8 dtim_cnt = vif->bss_conf.dtim_period-1;
    		if(sc->beacon_dtim_cnt != dtim_cnt)
    		{
    			sc->beacon_dtim_cnt = dtim_cnt;
#ifdef BEACON_DEBUG	
                printk("[A] beacon_dtim_cnt [%d]\n", sc->beacon_dtim_cnt);
#endif
                ssv6xxx_beacon_set_info(sc, sc->beacon_interval, 
                                                        sc->beacon_dtim_cnt);
    		}
    	}


        

    }while(0);
    
    if(old_skb)
	    dev_kfree_skb_any(old_skb);

}





void ssv6200_set_tim_work(struct work_struct *work)
{
    struct ssv_softc *sc = 
            container_of(work, struct ssv_softc, set_tim_work);
#ifdef BROADCAST_DEBUG
	printk("%s() enter\n", __FUNCTION__);
#endif    
	ssv6xxx_beacon_change(sc, sc->hw, sc->vif, sc->aid0_bit_set);
#ifdef BROADCAST_DEBUG
    printk("%s() leave\n", __FUNCTION__);
#endif    
}


int ssv6200_bcast_queue_len(struct ssv6xxx_bcast_txq *bcast_txq)
{
	u32 len;
    unsigned long flags;

    spin_lock_irqsave(&bcast_txq->txq_lock, flags);
    len = bcast_txq->cur_qsize;
    spin_unlock_irqrestore(&bcast_txq->txq_lock, flags);

    return len;
}



struct sk_buff* ssv6200_bcast_dequeue(struct ssv6xxx_bcast_txq *bcast_txq, u8 *remain_len)
{
    struct sk_buff *skb = NULL;
    unsigned long flags;
	
    spin_lock_irqsave(&bcast_txq->txq_lock, flags);
    if(bcast_txq->cur_qsize){
        bcast_txq->cur_qsize --;
        if(remain_len)
            *remain_len = bcast_txq->cur_qsize;    
        skb = __skb_dequeue(&bcast_txq->qhead);
    }
    spin_unlock_irqrestore(&bcast_txq->txq_lock, flags);  

    return skb;
}


int ssv6200_bcast_enqueue(struct ssv_softc *sc, struct ssv6xxx_bcast_txq *bcast_txq, 
                                                        struct sk_buff *skb)
{
    unsigned long flags;

    spin_lock_irqsave(&bcast_txq->txq_lock, flags);    
        
	/* Release oldest frame. */
    if (bcast_txq->cur_qsize >= 
                    SSV6200_MAX_BCAST_QUEUE_LEN){
        struct sk_buff *old_skb;
            
		old_skb = __skb_dequeue(&bcast_txq->qhead);
        bcast_txq->cur_qsize --;
        //dev_kfree_skb_any(old_skb);
        ssv6xxx_txbuf_free_skb(old_skb, (void*)sc);
        
        printk("[B] ssv6200_bcast_enqueue - remove oldest queue\n");
    }
	
    
    __skb_queue_tail(&bcast_txq->qhead, skb);
    bcast_txq->cur_qsize ++;
    
    spin_unlock_irqrestore(&bcast_txq->txq_lock, flags);
    
    return bcast_txq->cur_qsize;    
}

void ssv6200_bcast_flush(struct ssv_softc *sc, struct ssv6xxx_bcast_txq *bcast_txq)
{
 
    struct sk_buff *skb;
    unsigned long flags;
 
    spin_lock_irqsave(&bcast_txq->txq_lock, flags);
    while(bcast_txq->cur_qsize > 0) {
        skb = __skb_dequeue(&bcast_txq->qhead);
        bcast_txq->cur_qsize --;
        //dev_kfree_skb_any(skb);
        ssv6xxx_txbuf_free_skb(skb, (void*)sc);
    }
    spin_unlock_irqrestore(&bcast_txq->txq_lock, flags);    
}



static int queue_block_cnt = 0;

/* If buffer any mcast frame, send it. */
//void ssv6200_bcast_timer(unsigned long arg)	
void ssv6200_bcast_tx_work(struct work_struct *work)
{
    struct ssv_softc *sc =
            container_of(work, struct ssv_softc, bcast_tx_work.work);
//	struct ssv_softc *sc = (struct ssv_softc *)arg;


#if 1
    struct sk_buff* skb;
    int i;
    u8 remain_size;
#endif    
    unsigned long flags;
    bool needtimer = true;
    long tmo = sc->bcast_interval;							//Trigger after DTIM

    spin_lock_irqsave(&sc->ps_state_lock, flags);

    do{
#ifdef  BCAST_DEBUG
        printk("[B] bcast_timer: hw_mng_used[%d] HCI_TXQ_EMPTY[%d].....................\n", sc->hw_mng_used, HCI_TXQ_EMPTY(sc->sh, 4));
#endif
        //HCI_PAUSE(sc->sh, (TXQ_MGMT));
        
        /* if there is any frame in low layer, stop sending at this time */
        if(sc->hw_mng_used != 0 || 
            false == HCI_TXQ_EMPTY(sc->sh, 4)){
#ifdef  BCAST_DEBUG            
            printk("HW queue still have frames insdide. skip this one hw_mng_used[%d] bEmptyTXQ4[%d]\n", 
                sc->hw_mng_used, HCI_TXQ_EMPTY(sc->sh, 4));
#endif
            queue_block_cnt++;
            /* does hw queue have problem??? flush bcast queue to prevent tx_work drop in an inifate loop*/
            if(queue_block_cnt>5){
                queue_block_cnt = 0;
                ssv6200_bcast_flush(sc, &sc->bcast_txq);
                needtimer = false;
            }
            
            break;
        }

        queue_block_cnt = 0;
       
        for(i=0;i<SSV6200_ID_MANAGER_QUEUE;i++){
		        
            skb = ssv6200_bcast_dequeue(&sc->bcast_txq, &remain_size);            
            if(!skb){
                needtimer = false;
                break;
            }

            if( (0 != remain_size) &&
                (SSV6200_ID_MANAGER_QUEUE-1) != i ){
                /* tell station there are more broadcast frame sending... */
                struct ieee80211_hdr *hdr;
                struct ssv6200_tx_desc *tx_desc = (struct ssv6200_tx_desc *)skb->data;                
                hdr = (struct ieee80211_hdr *) ((u8*)tx_desc+tx_desc->hdr_offset);                      
                hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
            }

#ifdef  BCAST_DEBUG
            printk("[B] bcast_timer:tx remain_size[%d] i[%d]\n", remain_size, i);
#endif

            if(HCI_SEND(sc->sh, skb, 4)<0){
                printk("bcast_timer send fail!!!!!!! \n");
                //dev_kfree_skb_any(skb);
                ssv6xxx_txbuf_free_skb(skb, (void*)sc);
                BUG_ON(1);                
            }

        }     
    }while(0);
        
    if(needtimer){
#ifdef  BCAST_DEBUG        
        printk("[B] bcast_timer:need more timer to tx bcast frame time[%d]\n", sc->bcast_interval);
#endif
        queue_delayed_work(sc->config_wq,
				   &sc->bcast_tx_work,
				   tmo);
     
        //mod_timer(&sc->bcast_timeout, jiffies + tmo);
    }
    else{
#ifdef  BCAST_DEBUG        
       printk("[B] bcast_timer: ssv6200_bcast_stop\n");
#endif       
       ssv6200_bcast_stop(sc);
    }

    spin_unlock_irqrestore(&sc->ps_state_lock, flags);

#ifdef  BCAST_DEBUG
    printk("[B] bcast_timer: leave.....................\n");
#endif
}






/**
  *1. Update DTIM    
  *2. Send Broadcast frame after DTIM 
  */
void ssv6200_bcast_start_work(struct work_struct *work)
{
	struct ssv_softc *sc =
		container_of(work, struct ssv_softc, bcast_start_work);

#ifdef  BCAST_DEBUG		
    printk("[B] ssv6200_bcast_start_work==\n");
#endif
    
	/* Every DTIM launch timer to send b-frames*/
    sc->bcast_interval = (sc->beacon_dtim_cnt+1) *
			(sc->beacon_interval + 20) * HZ / 1000;							//Trigger after DTIM;
    
	if (!sc->aid0_bit_set) {
		sc->aid0_bit_set = true;

        /* 1. Update DTIM  */
		ssv6xxx_beacon_change(sc, sc->hw, 
                        sc->vif, sc->aid0_bit_set);

        /* 2. Send Broadcast frame after DTIM  */
        //mod_timer(&sc->bcast_timeout, jiffies + sc->bcast_interval);
        queue_delayed_work(sc->config_wq,
				   &sc->bcast_tx_work,
				   sc->bcast_interval);

#ifdef  BCAST_DEBUG
        printk("[B] bcast_start_work: Modify timer to DTIM[%d]==\n", (sc->beacon_dtim_cnt+1)*(sc->beacon_interval + 20));
#endif        
	}
    
    
}

/**
  *1. Update DTIM.
  *2. Remove timer to send beacon.
  */  
void ssv6200_bcast_stop_work(struct work_struct *work)
{
	struct ssv_softc *sc =
		container_of(work, struct ssv_softc, bcast_stop_work.work);

#ifdef  BCAST_DEBUG
    printk("[B] ssv6200_bcast_stop_work\n");
#endif

    /* expired every 10ms*/
    sc->bcast_interval = HZ / 10;

	if (sc->aid0_bit_set) {
        if(0== ssv6200_bcast_queue_len(&sc->bcast_txq)){

            /* 1. Remove timer to send beacon. */
//    		del_timer_sync(&sc->bcast_timeout);
            cancel_delayed_work_sync(&sc->bcast_tx_work);
            
    		sc->aid0_bit_set = false;               

            /* 2. Update DTIM. */
    		ssv6xxx_beacon_change(sc, sc->hw, 
                            sc->vif, sc->aid0_bit_set);
#ifdef  BCAST_DEBUG
            printk("remove group bit in DTIM\n");
#endif            
        }
        else{
#ifdef  BCAST_DEBUG            
            printk("bcast_stop_work: bcast queue still have data. just modify timer to 10ms\n");
#endif            
            //mod_timer(&sc->bcast_timeout, jiffies + sc->bcast_interval);
            queue_delayed_work(sc->config_wq,
				   &sc->bcast_tx_work,
				   sc->bcast_interval);
        }
	}
}





void ssv6200_bcast_stop(struct ssv_softc *sc)
{
    //cancel_work_sync(&sc->bcast_start_work);
    queue_delayed_work(sc->config_wq,
						   &sc->bcast_stop_work, sc->beacon_interval*HZ/1024);
}



void ssv6200_bcast_start(struct ssv_softc *sc)
{
    //cancel_delayed_work_sync(&sc->bcast_stop_work);
    queue_work(sc->config_wq,
						   &sc->bcast_start_work);
}




//Beacon related end
//----------------------------------------------------------------------------







