#ifndef _SSV6200_H_
#define _SSV6200_H_

#include <linux/device.h>
#include <linux/interrupt.h>
#include <net/mac80211.h>

// Include defines from config.mak to feed eclipse defines from ccflags-y
#ifdef ECLIPSE
#include <ssv_mod_conf.h>
#endif // ECLIPSE

#include <ssv6200_reg.h>
#include <ssv6200_aux.h>

#include <hwif/hwif.h>
#include <hci/ssv_hci.h>

#include "ssv6200_common.h"



//Reserve 5 pages for KEY & PIH info & rate control
/* tx page size could not more than 255
  * tx id could not more than 63
  *  TX_ID_ALL_INFO (TX_PAGE_USE_7_0 only 8bits and TX_ID_USE_5_0 only 6bits)
  */
#ifdef SSV6200_ECO
//Security use 8 ids.
#define SSV6200_ID_TX_THRESHOLD 			60//64      
#define SSV6200_ID_RX_THRESHOLD 			60//64 

//Reserve 4 for KEY & PIH info and 7*3 for security
#define SSV6200_PAGE_TX_THRESHOLD 		   115	//128 
#define SSV6200_PAGE_RX_THRESHOLD 		   116	//128 
#else
#define SSV6200_ID_TX_THRESHOLD             63//64      
#define SSV6200_ID_RX_THRESHOLD             63//64 

//Reserve 4 for KEY & PIH info , total num 256. 1 is 256 KB
#ifdef PREFER_RX
#define SSV6200_PAGE_TX_THRESHOLD          (126-24)  //128 
#define SSV6200_PAGE_RX_THRESHOLD          (126+24)  //128 
#else // PREFER_RX
#define SSV6200_PAGE_TX_THRESHOLD          126  //128
#define SSV6200_PAGE_RX_THRESHOLD          126  //128
#endif // PREFER_RX
#endif // SSV6200_ECO



#define SSV6200_ID_AC_RESERVED              1


#define SSV6200_ID_AC_BK_OUT_QUEUE          8
#define SSV6200_ID_AC_BE_OUT_QUEUE          15
#define SSV6200_ID_AC_VI_OUT_QUEUE          16
#define SSV6200_ID_AC_VO_OUT_QUEUE          16
#define SSV6200_ID_MANAGER_QUEUE            8

#define	HW_MMU_PAGE_SHIFT			0x8 //256 bytes
#define	HW_MMU_PAGE_MASK			0xff

//TX_PKT_RSVD(3) * unit(16)
#define SSV6200_DESC_OFFSET 			0x50
#define SSV6200_TX_PKT_RSVD_SETTING 	0x3
#define SSV6200_TX_PKT_RSVD 	 		SSV6200_TX_PKT_RSVD_SETTING*16
#define SSV6200_ALLOC_RSVD				SSV6200_DESC_OFFSET+SSV6200_TX_PKT_RSVD

#define SSV6200_BT_PRI_SMP_TIME     0
#define SSV6200_BT_STA_SMP_TIME     (SSV6200_BT_PRI_SMP_TIME+0)
#define SSV6200_WLAN_REMAIN_TIME    0
#define BT_2WIRE_EN_MSK                        0x00000400

struct txResourceControl {
    u32 txUsePage:8;
    u32 txUseID:6;
    u32 edca0:4;
    u32 edca1:4;
    u32 edca2:5;
    u32 edca3:5;
};

typedef enum __PBuf_Type_E {
    NOTYPE_BUF  = 0,
    TX_BUF      = 1,
    RX_BUF      = 2    
} PBuf_Type_E;


#include <ssv_cfg.h>


#endif /* _SSV6200_H_ */

