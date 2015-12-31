#ifndef __CABRIO_API_TYPES_H__
#define __CABRIO_API_TYPES_H__

#include "types.h"

#undef STRUCT_PACKED

#ifdef __GNUC__
#define STRUCT_PACKED __attribute__ ((packed))
#else
#define STRUCT_PACKED
#endif

#ifdef _MSC_VER
#pragma pack(push, 1)
#endif /* _MSC_VER */

/**
 * Define the error numbers of wifi host api:
 *
 * @ SSV_SUCCESS:
 * @ SSV_FAILED:
 */
 
typedef enum{
	SSV_SUCCESS                      =     0,
	SSV_FAILED                       =    -1,
	SSV_INVA_PARAM                   =    -2,
	SSV_NO_MEM                       =    -3,	
	SSV_QUEUE_FULL					 =	  -4,
	SSV_WRONG_HW_MODE_CMD			 =	  -5,

}ssv_result;


#define AP_PS_FRAME 			BIT(0)		//This frame was buffered by AP.


/**
 * struct cfg_host_txreq0 - Host frame transmission Request  Header
 *
 * TX-REQ0 uses 4-byte header to carry host message to wifi-controller.
 * The first two-byte is the length indicating the whole message length (
 * including 2-byte header length).
 */
 struct cfg_host_txreq0 {	
    u32               len:16;
    u32               c_type:3;
    u32               f80211:1;
    u32               qos:1;
    u32               ht:1;
    u32               use_4addr:1;
    u32               RSVD0:4;			//AP mode use one bit to know if this packet is buffered frame(Power saving)
    u32               security:1;
    u32               more_data:1;
    u32               sub_type:2;
    u32               extra_info:1;
 } STRUCT_PACKED;

 
#if 0
/**
 *  struct cfg_host_txreq1 - Host frame transmission Request  Header
 */
struct cfg_host_txreq1 {
    u32               len:16;
    u32               c_type:3;
    u32               f80211:1;
    u32               qos:1;
    u32               ht:1;
    u32               use_4addr:1;
    u32               RSVD0:4;
	u32               security:1;
    u32               more_data:1;
    u32               sub_type:2;
    u32               extra_info:1;
    
    u32               f_cmd;
} STRUCT_PACKED; 


/**
 *  struct cfg_host_txreq2 - Host frame transmission Request  Header
 */
struct cfg_host_txreq2 {
#pragma message("===================================================")
#pragma message("     cfg_host_txreq2 not implement yet")
#pragma message("===================================================")
    u32               len:16;
    u32               c_type:3;
    u32               f80211:1;
    u32               qos:1;
    u32               ht:1;
    u32               use_4addr:1;
    u32               RSVD0:4;
	u32               security:1;
    u32               more_data:1;
    u32               sub_type:2;
    u32               extra_info:1;

    u32               f_cmd;
    u32               AAA;
}; //__attribute__((packed))

 /**
 *  struct cfg_host_txreq - Host frame transmission Request Parameters
 */
struct cfg_host_txreq {
	struct cfg_host_txreq0 txreq0;
        u16           qos;
        u32           ht;
        u8            addr4[ETHER_ADDR_LEN];
	u8            priority;	
}; //__attribute__((packed));


#endif

enum ssv_data_priority {
	ssv_data_priority_0,
	ssv_data_priority_1,
	ssv_data_priority_2,
	ssv_data_priority_3,
	ssv_data_priority_4,
	ssv_data_priority_5,
	ssv_data_priority_6,
	ssv_data_priority_7,				
};

struct cfg_host_rxpkt {   
	/* The definition of WORD_1: */
	u32             len:16;
	u32             c_type:3;
	u32             f80211:1;
	u32             qos:1;          /* 0: without qos control field, 1: with qos control field */
	u32             ht:1;           /* 0: without ht control field, 1: with ht control field */
	u32             use_4addr:1;
	u32             l3cs_err:1;
	u32             l4cs_err:1;
	u32             align2:1;
	u32             RSVD_0:2;
	u32             psm:1;
	u32             stype_b5b4:2;
	u32             extra_info:1; 

};

typedef enum{
	SSV_DATA_ACPT		= 0,	//Accept
	SSV_DATA_CONT		= 1,	//Pass data
	SSV_DATA_QUEUED		= 2,	//Data Queued 
}ssv_data_result;

#ifdef _MSC_VER
#pragma pack(pop)
#endif /* _MSC_VER */
#endif // __CABRIO_API_TYPES_H__

