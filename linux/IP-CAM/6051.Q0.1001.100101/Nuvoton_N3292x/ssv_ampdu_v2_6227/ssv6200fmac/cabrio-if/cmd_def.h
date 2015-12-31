#ifndef _CMD_DEF_H_
#define _CMD_DEF_H_
#include "common.h"

/************************************************************************************************************************************************/
/*                                                                Extra Info                                                                    */
/************************************************************************************************************************************************/

enum ssv_tx_extra_type
{
    SSV_TX_FRAME_TIME,
	SSV_TX_DO_RTS_CTS,
	SSV_TX_ACK_POLICY,
	SSV_TX_TX_BRUST,
	SSV_TX_TX_REPORT,
	SSV_TX_RTS_CTS_NAV,
	SSV_TX_CRATE_IDX,
	SSV_TX_DRATE_IDX,
	SSV_TX_DL_LENGTH_IDX,

	SSV_SET_INIT_BEACON,
	SSV_SET_BEACON,
};


#define SSV_EXTRA_TOTAL_LEN_SIZE sizeof(u16)

struct cfg_bcn_info {
    u16 bcn_len;
    u16 tim_cnt_oft;		//include mac header
    //u8  hdr_len;
	
};


//SSV_SET_BEACON
struct cfg_bcn_param {
	u8 			 dtim_cnt;	//DTIM=4-->dtim_cnt=3;
	u16          bcn_itv;  	//beacon interval		
};


//SSV_SET_INIT_BEACON
struct cfg_set_init_bcn {
	struct cfg_bcn_param  param;
	struct cfg_bcn_info   bcn_info;    
};


/************************************************************************************************************************************************/
/*                                                                Host Command                                                                    */
/************************************************************************************************************************************************/


/**
 *  struct cfg_host_cmd - Host Command Header Format description
 * 
 */
typedef struct cfg_host_cmd {
    u32             len:16;
    u32             c_type:3;
    u32             RSVD0:5;//It will be used as command index eg.  STA-WSID[0]-->RSVD0=0, STA-WSID[1]-->RSVD0=1
    u32             h_cmd:8;//------------------------->ssv_host_cmd/command id
    u32				cmd_seq_no;
    union { /*lint -save -e157 */
    u32             dummy; // Put a u32 dummy to make MSVC and GCC treat HDR_HostCmd as the same size.
    u8              dat8[0];
    u16             dat16[0];
    u32             dat32[0];
    }; /*lint -restore */
} HDR_HostCmd;
// Use 100 instead of 0 to get header size to avoid lint from reporting null pointer access.
#define HOST_CMD_HDR_LEN        ((size_t)(((HDR_HostCmd *)100)->dat8)-100U)

typedef struct cmd_seq_ctrl_st{
	u32 cmd_seq_no;
}CMD_SEQ_CTRL;

//=========================================
//                Public Command Data
//=========================================

//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_CMD_SCAN

#define IEEE80211_MAX_SSID_LEN		32
#define IEEE80211_MAX_SUPP_RATES    32

/**
 *  struct cfg_80211_ssid - SSID description, the max of the length of 32-byte.
 *
 *  @ ssid: the SSID
 *  @ ssid_len: length of the SSID
 */
PACK( struct cfg_80211_ssid {
    u8              ssid[MAX_SSID_LEN];
    u8              ssid_len;
} )


PACK( struct ssv_ieee80211_bss {
	struct cfg_80211_ssid ssid;
	u16 capab_info;
	u16 parameter_set_count;

	ETHER_ADDR	bssid;
	
	u8 dtim_period;
	u8 wmm_used;
	u8 uapsd_supported;
	u8 channel_id;

	//RSN
	u16 rsn_version;
	u32	rsn_group_cipher;
	u32	rsn_pairwise_cipher[2];

	u32	last_probe_resp;

	u8 supp_rates[IEEE80211_MAX_SUPP_RATES];
	u8 supp_rates_len;

	/*
	 * During association, we save an ERP value from a probe response so
	 * that we can feed ERP info to the driver when handling the
	 * association completes. these fields probably won't be up-to-date
	 * otherwise, you probably don't want to use them.
	 */
	u8 has_erp_value;
	u8 erp_value;
} )



//=========================================
//                Public Command Data
//=========================================

/**
 *  struct cfg_scan_request  - scan request description
 *  
 *  @ is_active: do active scan or passive scan
 *  @ n_ssids: the number of ssid to scan
 *  @ dwell_time: Time to wait for receiving beacon or probe response in each channel. (in 10ms).
 *  @ ssids: array of ssids to scan
 *  @ channel_mask: channel bit mask indicates which channels to scan
 *  @ ie_len: length of ie in octects
 *  @ ie_offset: optional information element(s) to add into probe request 
 *                    frame. 
 */
struct cfg_scan_request {
    u8              RSVD[2];
    bool            is_active;
    u8              n_ssids;
    u8              dwell_time;
    u16             channel_mask; /* channel 0 ~ channel 15 */

    /* keep last */
    struct cfg_80211_ssid ssids[0];
} ;//__attribute__ ((packed));



//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_CMD_JOIN

/**
 *  struct cfg_join_request - join request description
 */
struct cfg_join_request {  
    ssv_sec_type             sec_type;
    int                      wep_keyidx;
    u8                       password[MAX_PASSWD_LEN+1];
    u8                       auth_alg;
    u8                       request_supplicant_bypass;
    struct ssv_ieee80211_bss bss;
    u32                      assoc_ie_len;
    u8                       assoc_ie[0];
} ;//__attribute__ ((packed));





//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_CMD_LEAVE

/**
 *  struct cfg_leave_request - leave request description
 */
struct cfg_leave_request {
    //ETHER_ADDR      bssid;

    u16             info_len;
    u16             reason;
} ;//__attribute__ ((packed));


//Security entry
struct security_group_entry
{
    u8            index;
    u8            key[32];
    u64            tx_pn;
    u64            rx_pn;
};


struct security_pair_entry
{
    u8            index;
    u8             pair_key_idx;     //0-3
    u8             group_key_idx;    //0-3
    u8            reserve;
    u8          key[32];
    u64            tx_pn;
    u64            rx_pn;
};





//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_CMD_SET_EVT_CB


struct cfg_set_cb_request {
    void *cb;
};



//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_CMD_SET_DATA_CB




//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_WMM_PARAM

struct cfg_set_wmm_param {
    u32                aifsn:4;    //0~15
    u32                acm:1;
    u32                resv:3;    
    u32                cwmin:4;//(1, 3, 7, 15, 31, 63, 127, 255, 511, 1023) millionsecond
    u32                cwmax:4;//(1, 3, 7, 15, 31, 63, 127, 255, 511, 1023)
    u32                txop:16;    //65536  milliseconds

    //u32                cw:4;    
    u32                backoffvalue:16;
    u32                enable_backoffvalue:1;
    u32                RESV:15;
};





//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_TX_PACKET_CNT
//dat->u32



//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_TX_QUEUE_HALT
//dat->u16


//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_INIT_BCN_PARAM



//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_STA


//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_MIB
enum cfg_mib_type
{
    CFG_MIB_RTS,
    CFG_MIB_FRAG,
};

struct cfg_set_mib{
    u32 mib_type;
    u32 mib_value;
};


//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_STA
//******************************************************
//				=======IMPORTANT=======
//Info flag is driect mapping to hw reg. DO NOT CHANGE IT.
//******************************************************
//HT_MODE(2)+OP_MODE(2)+QoS(1)+valid(1)
//
#define SET_STA_INFO_VALID(s, v)       (s)->info = (((s)->info & ~(1<<0)) | ((v)<<0))
#define SET_STA_INFO_QOS_EN(s, v)      (s)->info = (((s)->info & ~(1<<1)) | ((v)<<1))
#define SET_STA_INFO_OP_MODE(s, v)     (s)->info = (((s)->info & ~(3<<2)) | ((v)<<2))
#define SET_STA_INFO_HT_MODE(s, v)     (s)->info = (((s)->info & ~(3<<4)) | ((v)<<4))

#define IS_STA_INFO_VALID(s)               !!((s)->info & (1<<0))

//#define GET_STA_INFO_OPER(s)       ((s)->info & (3))//bit 0 1




enum cfg_sta_oper
{
    CFG_STA_DEL,
    CFG_STA_ADD,
    CFG_STA_DEL_ALL,
    CFG_STA_MODIFY,
};


//-------------------------------------
enum cfg_qos_type
{
    CFG_QOS_NONE,
    CFG_QOS_ENABLE,
};


enum cfg_op_mode_type
{
    CFG_OP_MODE_STA,
    CFG_OP_MODE_AP,
    CFG_OP_MODE_AD_HOC,
    CFG_OP_MODE_WDS,
};


enum cfg_ht_type
{
    CFG_HT_NONE,
    CFG_HT_MF,
    CFG_HT_GF,
};




struct cfg_wsid_info {
    u32 info;
    struct ETHER_ADDR_st addr;
};



struct cfg_set_sta {
    enum cfg_sta_oper sta_oper;
    struct cfg_wsid_info wsid_info;//info set to reg
};


//-------------------------------------------------------------------------------------------------------------------------------------------------
//SSV_HOST_CMD_SET_AP_CFG
struct cfg_set_ap_cfg {
	struct cfg_80211_ssid ssid;
	ssv_sec_type sec_type;
    u8 password[MAX_PASSWD_LEN+1];    
};


#ifdef THROUGHPUT_TEST

struct mcu_result
{
    bool done;
    u32 tx_speed;
    u32 rx_speed;
    u32 rx_snr;
    u32 rx_rssi;
    
};
#endif

enum wpa_host_cipher {
	CIPHER_HOST_NONE,
	CIPHER_HOST_WEP40,
	CIPHER_HOST_TKIP,
	CIPHER_HOST_CCMP,
	CIPHER_HOST_WEP104
};

//Security
struct securityEntry {
	u8 cipher;
	u8 wpaUnicast;
	u8 keyLen;
	u8 keyIndex;
    u8 key[32];
	u8 bssid[6];
};



typedef enum{
//===========================================================================    
    //Public command        
    SSV_HOST_CMD_START                  = 0                                                     ,
    SSV_HOST_CMD_SCAN                   = SSV_HOST_CMD_START                                    ,   //Station
    SSV_HOST_CMD_JOIN                                                                           ,   //Station
    SSV_HOST_CMD_LEAVE                                                                          ,   //Station

    SSV_HOST_CMD_SET_HW_MODE                                                                    ,
    SSV_HOST_CMD_SET_CONFIG                                                                     ,
    SSV_HOST_CMD_GET_CONFIG                                                                     ,
    SSV_HOST_CMD_SET_REG                                                                        ,
    SSV_HOST_CMD_GET_REG                                                                        ,
	SSV_HOST_CMD_SET_STA_MAC                                                                    ,
    SSV_HOST_CMD_SET_BSSID                                                                      ,
    SSV_HOST_CMD_SET_DECITBL                                                                    ,
    SSV_HOST_CMD_SET_WSIDTBL                                                                    ,
    SSV_HOST_CMD_SET_TX_ETHTRAP                                                                 ,
    SSV_HOST_CMD_SET_RX_ETHTRAP                                                                 ,
    SSV_HOST_CMD_SET_FCMD_TXDATA                                                                ,
    SSV_HOST_CMD_SET_FCMD_TXMGMT                                                                ,
    SSV_HOST_CMD_SET_FCMD_TXCTRL                                                                ,
    SSV_HOST_CMD_SET_FCMD_RXDATA                                                                ,    
    SSV_HOST_CMD_SET_FCMD_RXMGMT                                                                ,    
    SSV_HOST_CMD_SET_FCMD_RXCTRL                                                                ,
    //SSV_HOST_CMD_SET_TXF_ID                                                                  ,
    SSV_HOST_CMD_SET_SIFS                                                                       ,
    SSV_HOST_CMD_SET_DIFS                                                                       ,
    SSV_HOST_CMD_SET_EIFS                                                                       ,
    //Security
    SSV_HOST_CMD_SET_PAIR_SECURITY                                                              ,
    SSV_HOST_CMD_SET_GROUP_SECURITY                                                             ,
    SSV_HOST_CMD_SET_PAIR_ENTRY                                                                 ,
    SSV_HOST_CMD_SET_GROUP_ENTRY                                                                ,
    SSV_HOST_CMD_SET_SECURITY_ENTRY                                                             ,
    SSV_HOST_CMD_SET_TX_INFO_SECURITY                                                           ,
    SSV_HOST_CMD_SET_WMM_PARAM                                                                  ,
    SSV_HOST_CMD_SET_WMM_RANDOM                                                                 ,
    SSV_HOST_CMD_SET_TX_PACKET_CNT                                                              ,
    SSV_HOST_CMD_SET_TX_QUEUE_HALT                                                              ,
    SSV_HOST_CMD_SET_TXOP_SUB_FRM_TIME                                                          ,
    SSV_HOST_CMD_SET_OPMODE                                                                     ,
    SSV_HOST_CMD_SET_NAV                                                                        ,
    SSV_HOST_CMD_SET_RX2HOST                                                                    ,
    SSV_HOST_CMD_SET_RXSNIFFER									                                ,
    SSV_HOST_CMD_STRIP_OFF                                                                      ,
    SSV_HOST_CMD_SET_TRAP_MASK                                                                  ,
    SSV_HOST_CMD_SET_GLOBAL_SEQCTRL                                                             ,
    SSV_HOST_CMD_SET_AUTO_SEQNO                                                                 ,
    SSV_HOST_CMD_SET_RX_NULL_DATA_TRAP															,
    SSV_HOST_CMD_SET_M2_RX2HOST																,
    SSV_HOST_CMD_SET_RX_INFO_SIZE															,
    SSV_HOST_CMD_SET_ERP_PROTECT                                                                ,
    SSV_HOST_CMD_SET_PBUF_OFFSET                                                                ,
    SSV_HOST_CMD_SET_DUP_FLT                                                                    ,
    SSV_HOST_CMD_SET_HT_MODE                                                                    ,
    SSV_HOST_CMD_SET_PROMIS_MODE                                                                ,
    SSV_HOST_CMD_SET_QOS_CAP                                                                    ,
    SSV_HOST_CMD_SET_GMFLT                                                                      ,
    SSV_HOST_CMD_SET_MGMT_TXQID                                                                 ,
    SSV_HOST_CMD_SET_NONQOS_TXQID                                                               ,
    SSV_HOST_CMD_SET_MAC_MODE                                                                   ,

    SSV_HOST_CMD_SET_STOP_BCN                                                                  ,
    SSV_HOST_CMD_SET_STA                                                                        ,
    SSV_HOST_CMD_SET_FRAGTHRES																	,
    SSV_HOST_CMD_GET_STA_MAC                                                                    ,
    SSV_HOST_CMD_GET_BSSID                                                                      ,
    SSV_HOST_CMD_GET_DECI_TBL                                                                   ,
    SSV_HOST_CMD_GET_WSID_TBL                                                                   ,
    SSV_HOST_CMD_GET_ETHER_TRAP                                                                 ,
    SSV_HOST_CMD_GET_FCMDS                                                                      ,
    //SSV_HOST_CMD_GET_TX_ETH_TRAP                                                                ,
    //SSV_HOST_CMD_GET_RX_ETH_TRAP                                                                ,
    //SSV_HOST_CMD_GET_FCMD_TX_DATA                                                               ,
    //SSV_HOST_CMD_GET_FCMD_TX_MGMT                                                               ,
    //SSV_HOST_CMD_GET_FCMD_TX_CTRL                                                               ,
    //SSV_HOST_CMD_GET_FCMD_RX_DATA                                                               ,
    //SSV_HOST_CMD_GET_FCMD_RX_MGMT                                                               ,
    //SSV_HOST_CMD_GET_FCMD_RX_CTRL                                                               ,    
   
	SSV_HOST_CMD_SET_MIB																		,
//	SSV_HOST_CMD_INIT_PHY_TABLE																,
	SSV_HOST_CMD_SET_PHY_INFO_TBL                                                               ,
	SSV_HOST_CMD_GET_PHY_INFO_TBL                                                               ,
	
	SSV_HOST_CMD_SET_RX_CHECK	                                                                ,
	SSV_HOST_CMD_SHOW_ALL																		,
	
	SSV_HOST_CMD_LOG                                                                            ,
	SSV_HOST_CMD_CAL											   								,
	SSV_HOST_CMD_SET_BCN_TIMER_EN                                                               ,
    SSV_HOST_CMD_GET_CHANNEL                                                                    ,
#ifdef THROUGHPUT_TEST
	SSV_HOST_CMD_HCI2SDIO																		,
	SSV_HOST_CMD_MIC2SDIO																		,
	SSV_HOST_CMD_SEC2SDIO										 								,

	SSV_HOST_CMD_THROUGHTPUT																	,
	SSV_HOST_CMD_GET_RESULT                                          ,
#endif
	/*
    SSV_HOST_CMD_GET_SIFS                                                                       ,
    SSV_HOST_CMD_GET_DIFS                                                                       ,
    SSV_HOST_CMD_GET_EIFS                                                                       ,
    */




#if defined(BEACON_DBG) && (BEACON_DBG == 1)
	SSV_HOST_CMD_BEACON_RELEASE_TEST															,
	
#endif

    SSV_HOST_SOC_CMD_MAXID                                                                      ,  
    
//===========================================================================    

    SSV_HOST_CMD_SET_EVT_CB                    = SSV_HOST_SOC_CMD_MAXID,
    SSV_HOST_CMD_SET_DATA_CB                                                                    ,
    SSV_HOST_CMD_UNSET_EVT_CB                                                                   ,
    SSV_HOST_CMD_UNSET_DATA_CB                                                                  ,

    SSV_HOST_CMD_SET_BSS_PARAM                                                                  ,
    SSV_HOST_CMD_SET_ADDR4                                                                      ,
    SSV_HOST_CMD_SET_HT                                                                         ,
    SSV_HOST_CMD_SET_SECURITY                                                                   ,
    SSV_HOST_CMD_SET_QOS                                                                        ,
	SSV_HOST_CMD_SET_AP_CFG																		,

    SSV_HOST_CMD_MAXID                                                                          ,
}ssv_host_cmd_id;


//-------------------------------------------------------------------------------------------------------------------------------------------


/************************************************************************************************************************************************/
/*                                                                Host Event                                                                        */
/************************************************************************************************************************************************/


/**
 *  struct cfg_host_event - Host Event Header Format description
 * 
 */
typedef struct cfg_host_event {
    u32             len:16;
    u32             c_type:3;
    u32             RSVD0:5;
    u32             h_event:8;//------------------>ssv_host_evt
    u32				evt_seq_no;
    u8              dat[0];
    
} HDR_HostEvent;



//=========================================
//                Public Event Data
//=========================================

#if 0
struct resp_evt_result {
    union{
        bool is_sucessful;  //Scan, SetHwMode
        s32  status_code;   //Join    
   }u;
   s32 aid;
   
} ;//__attribute__ ((packed));
#endif

#define  CMD_SUCESS 0
#define CMD_PASS_FRAME 1

struct resp_evt_result {
	#ifdef USE_CMD_RESP
	u8		cmd;
	u8		result;
	u32     cmd_seq_no;
	#endif // USE_CMD_RESP
	union {     
        struct {
   		 u8                   		result_code;//SUCCESS,PASS_FRAME
		 struct ssv_ieee80211_bss 	bss_info;
         u16                  		dat_size;
		 u8                   		dat[0];//for saving probe resp
        } scan;		
        struct {
		 s32 			status_code;
		 s32			aid;
        } join;
        struct {
         s16			reason_code;
        } leave;
        u8 dat[0];
    } u;
};

#define RESP_EVT_HEADER_SIZE		((size_t)(&((struct resp_evt_result *)100)->u.dat[0]) - 100U)
#define CMD_RESPONSE_BASE_SIZE		(sizeof(HDR_HostEvent) + RESP_EVT_HEADER_SIZE)
#define SCAN_RESP_EVT_HEADER_SIZE	((size_t)(&((struct resp_evt_result *)100)->u.scan.dat[0]) - 100U)


//=========================================
//                Private Event Data
//=========================================



//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_EVENT_PS_POLL
struct cfg_ps_poll_info {
    //This field must be in the beginning. 
    ETHER_ADDR      SAAddr;

} ;






//-------------------------------------------------------------------------------------------------------------------------------------------------
//HOST_EVENT_NULL_DATA
//
//Bit info stores in Flags of cfg_null_data_info
#define HOST_EVT_NULL_DATA_PM   BIT(0)
#define HOST_EVT_NULL_DATA_QOS  BIT(1)



struct cfg_null_data_info {
    //This field must be in the beginning. 
    ETHER_ADDR    SAAddr;

    u32            Flags:28;
    u32            Priority:4;
} ;


typedef enum _CmdResult_E {
	CMD_OK,				// Command executed successfully. Check corresponding returned values for status.
	CMD_INVALID,		// Invalid command.
	CMD_STATE_ERROR,    // Not executable in current firmware state.
	CMD_TIMEOUT,        // Peer does not response in time. Command expired.
	CMD_BUSY,			// Too busy to accept new command.
	CMD_NOMEM,          // Out of resource to execute this command
} CmdResult_E;


typedef enum{
//===========================================================================    
    //Public event
#ifdef USE_CMD_RESP
    SOC_EVT_CMD_RESP                        , // Response of a host command.
    SOC_EVT_SCAN_RESULT                     , // Scan result from probe response or beacon
    SOC_EVT_DEAUTH                          , // Deauthentication received but not for leave command
#else
    SOC_EVT_HW_MODE_RESP					,
    SOC_EVT_SCAN_RESULT						,
    SOC_EVT_JOIN_RESULT						,
    SOC_EVT_LEAVE_RESULT					,
    SOC_EVT_GET_REG_RESP					,
    SOC_EVT_GET_STA_MAC_RESP				,
    SOC_EVT_GET_BSSID_RESP					,
    SOC_EVT_GET_DECI_TABLE_RESP				,
    SOC_EVT_GET_WSID_TABLE_RESP				,
    //SOC_EVT_GET_TX_ETH_TRAP_RESP			,
    //SOC_EVT_GET_RX_ETH_TRAP_RESP			,
    SOC_EVT_GET_ETHER_TRAP_RESP             ,
    //SOC_EVT_GET_FCMD_TX_DATA_RESP			,
    //SOC_EVT_GET_FCMD_TX_MGMT_RESP			,
    //SOC_EVT_GET_FCMD_TX_CTRL_RESP			,
    //SOC_EVT_GET_FCMD_RX_DATA_RESP			,
    //SOC_EVT_GET_FCMD_RX_MGMT_RESP			,
    //SOC_EVT_GET_FCMD_RX_CTRL_RESP			,
    SOC_EVT_GET_FCMDS_RESP                  ,
    SOC_EVT_GET_PHY_INFO_TBL_RESP           ,
    SOC_EVT_GET_SIFS_RESP					,
    SOC_EVT_GET_DIFS_RESP					,
    SOC_EVT_GET_EIFS_RESP					,
#ifdef THROUGHPUT_TEST	
    SOC_EVT_GET_RESULT_RESP			        ,
#endif	
#endif // USE_CMD_RESP
	SOC_EVT_LOG  							,			// ssv log module soc event
	SOC_EVT_ACK								,
	//SOC_EVT_SDIO_TEST_RX_RESULT,
	SOC_EVT_TX_ALL_DONE						,
	
    SOC_EVT_USER_END						,
    
//===========================================================================    
    //Private    event
    SOC_EVT_PRIVE_CMD_START					= SOC_EVT_USER_END,
    SOC_EVT_CONFIG_HW_RESP					= SOC_EVT_PRIVE_CMD_START,    
    SOC_EVT_SET_BSS_PARAM_RESP				,
    SOC_EVT_PS_POLL							,
    SOC_EVT_NULL_DATA						,
    SOC_EVT_REG_RESULT						,
    
	SOC_EVT_DTIM_EXPIRED					,


//    SOC_EVT_GET_REG_RESP					,
    SOC_EVT_MAXID							,
} ssv_soc_event;

struct MsgEvent_st;
//typedef struct MsgEvent_st MsgEvent;

struct MsgEvent_st *HostEventAlloc(ssv_soc_event hEvtID, u32 size);

struct MsgEvent_st *HostCmdRespEventAlloc(const struct cfg_host_cmd *host_cmd, CmdResult_E cmd_result, u32 resp_size, void **p_resp_data);

/**
 * Define Macros for host event manipulation: 
 *
 * @ HOST_EVENT_ALLOC():    Allocate a host event structure from the system.
 * @ HOST_EVENT_SET_LEN(): Set the host event length. The length excludes 
 *                                             the event header length.
 * @ HOST_EVENT_SEND():      Send the event to the host.
 */
#define HOST_EVENT_SET_LEN(ev, l)                       \
{                                                       \
    ((HDR_HostEvent *)((ev)->MsgData))->len =          \
    (l) + sizeof(HDR_HostEvent);                        \
}
#define HOST_EVENT_ALLOC_RET(ev, evid, l, ret)          \
{                                                       \
    (ev) = HostEventAlloc(evid, l);                     \
    ASSERT_RET(ev, ret);                                \
}
#define HOST_EVENT_ALLOC(ev, evid, l)                   \
{                                                       \
    (ev) = HostEventAlloc(evid, l);                     \
    ASSERT_RET(ev, EMPTY);                              \
}
#define HOST_EVENT_DATA_PTR(ev)                         \
    ((HDR_HostEvent *)((ev)->MsgData))->dat
#define HOST_EVENT_ASSIGN_EVT_NO(ev,no)			\
{														\
	 (((HDR_HostEvent *)((ev)->MsgData))->evt_seq_no)=no;	\
}

#define HOST_EVENT_ASSIGN_EVT(ev, evt_id)                   \
    do {                                                   \
	    ((HDR_HostEvent *)((ev)->MsgData))->h_event = evt_id;\
    } while (0)

#ifdef USE_CMD_RESP
#define HOST_EVENT_SEND(ev)                             \
{                                                       \
    extern u32 sg_soc_evt_seq_no;                       \
	sg_soc_evt_seq_no++;                                \
	HOST_EVENT_ASSIGN_EVT_NO(MsgEv,sg_soc_evt_seq_no);  \
    msg_evt_post(MBOX_CMD_ENGINE, (ev));                \
}
#else
#define HOST_EVENT_SEND(ev)                             \
{                                                       \
    msg_evt_post(MBOX_CMD_ENGINE, (ev));                \
}
#endif // USE_CMD_RESP


#define CMD_RESP_ALLOC(evt_msg, host_cmd, cmd_result, resp_size, p_resp_data) \
    do { \
        evt_msg = host_cmd_resp_alloc(host_cmd, cmd_result, resp_size, p_resp_data); \
        if (evt_msg == NULL) \
            return; \
    } while (0)

#define CMD_RESP_ALLOC_RET(evt_msg, host_cmd, cmd_result, resp_size, p_resp_data, fail_ret) \
    do { \
        evt_msg = host_cmd_resp_alloc(host_cmd, cmd_result, resp_size, p_resp_data); \
        if (evt_msg == NULL) \
            return fail_ret; \
    } while (0)

#endif//_CMD_DEF_H_

