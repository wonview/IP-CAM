#ifndef _SSV6200_COMMON_H_
#define _SSV6200_COMMON_H_

/*
    Reference with firmware
*/

/* Hardware Offload Engine ID */
#define M_ENG_CPU                       0x00
#define M_ENG_HWHCI                     0x01
//#define M_ENG_FRAG                    0x02
#define M_ENG_EMPTY                     0x02
#define M_ENG_ENCRYPT                   0x03
#define M_ENG_MACRX                     0x04  
#define M_ENG_MIC                       0x05
#define M_ENG_TX_EDCA0                  0x06
#define M_ENG_TX_EDCA1                  0x07
#define M_ENG_TX_EDCA2                  0x08
#define M_ENG_TX_EDCA3                  0x09
#define M_ENG_TX_MNG                    0x0A
#define M_ENG_ENCRYPT_SEC               0x0B
#define M_ENG_MIC_SEC                   0x0C
#define M_ENG_RESERVED_1                0x0D
#define M_ENG_RESERVED_2                0x0E
#define M_ENG_TRASH_CAN                 0x0F
#define M_ENG_MAX                      (M_ENG_TRASH_CAN+1)


/* Software Engine ID: */
#define M_CPU_HWENG                     0x00
#define M_CPU_TXL34CS                   0x01
#define M_CPU_RXL34CS                   0x02
#define M_CPU_DEFRAG                    0x03
#define M_CPU_EDCATX                    0x04
#define M_CPU_RXDATA                    0x05
#define M_CPU_RXMGMT                    0x06
#define M_CPU_RXCTRL                    0x07
#define M_CPU_FRAG                      0x08



/**
 *  The flag definition for c_type (command type) field of PKTInfo:
 *
 *      @ M0_TXREQ:         
 *      @ M1_TXREQ
 *      @ M2_TXREQ
 *      @ M0_RXEVENT
 *      @ M1_RXEVENT
 *      @ HOST_CMD
 *      @ HOST_EVENT
 *
 */
#define M0_TXREQ                            0
#define M1_TXREQ                            1
#define M2_TXREQ                            2
#define M0_RXEVENT                          3
#define M2_RXEVENT                          4
#define HOST_CMD                            5
#define HOST_EVENT                          6
#define TEST_CMD                            7

#define SSV6XXX_RX_DESC_LEN                     \
        (sizeof(struct ssv6200_rx_desc) +       \
         sizeof(struct ssv6200_rxphy_info))
#define SSV6XXX_TX_DESC_LEN                     \
        (sizeof(struct ssv6200_tx_desc) + 0)    \


#define SSV62XX_TX_MAX_RATES    3
struct fw_rc_retry_params {
    u32 count:4;
    u32 drate:6;
    u32 crate:6;
    u32 rts_cts_nav:16;
    u32 frame_consume_time:10;
    u32 dl_length:12;
    u32 RSVD:10;
} __attribute__((packed));

/**
* struct ssv6200_tx_desc - ssv6200 tx frame descriptor.
* This descriptor is shared with ssv6200 hardware and driver.
*/
struct ssv6200_tx_desc
{
    /* The definition of WORD_1: */
    u32             len:16;
    u32             c_type:3;
    u32             f80211:1;
    u32             qos:1;          /* 0: without qos control field, 1: with qos control field */
    u32             ht:1;           /* 0: without ht control field, 1: with ht control field */
    u32             use_4addr:1;
    u32             RSVD_0:3;//used for rate control report event.
    u32             bc_que:1;
    u32             security:1;
    u32             more_data:1;
    u32             stype_b5b4:2;
    u32             extra_info:1;   /* 0: don't trap to cpu after parsing, 1: trap to cpu after parsing */

    /* The definition of WORD_2: */
    u32             fCmd;

    /* The definition of WORD_3: */
    u32             hdr_offset:8;
    u32             frag:1;
    u32             unicast:1;
    u32             hdr_len:6;
    u32             tx_report:1;
    u32             tx_burst:1;     /* 0: normal, 1: burst tx */
    u32             ack_policy:2;   /* See Table 8-6, IEEE 802.11 Spec. 2012 */
    u32             aggregation:1;
    u32             RSVD_1:3;//Used for AMPDU retry counter
    u32             do_rts_cts:2;   /* 0: no RTS/CTS, 1: need RTS/CTS */
                                    /* 2: CTS protection, 3: RSVD */
    u32             reason:6;

    /* The definition of WORD_4: */
    u32             payload_offset:8;
    u32             next_frag_pid:7;
    u32             RSVD_2:1;
    u32             fCmdIdx:3;
    u32             wsid:4;
    u32             txq_idx:3;
    u32             TxF_ID:6;

    /* The definition of WORD_5: */
    u32             rts_cts_nav:16;
    u32             frame_consume_time:10;  //32 units
    u32             crate_idx:6;

    /* The definition of WORD_6: */
    u32             drate_idx:6;
    u32             dl_length:12;
    u32             RSVD_3:14;
    /* The definition of WORD_7~14: */
    u32             RESERVED[8];
    /* The definition of WORD_15~20: */
    struct fw_rc_retry_params rc_params[SSV62XX_TX_MAX_RATES];
};

#define HCI_AGGR_MAX 8
typedef struct HCI_aggr_Info_st 
{
    //u32 num;
    u16 len[HCI_AGGR_MAX];
}HCI_aggr_Info, *PHCI_aggr_Info;

/**
* struct ssv6200_rx_desc - ssv6200 rx frame descriptor.
* This descriptor is shared with ssv6200 hardware and driver.
*/
struct ssv6200_rx_desc
{
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

    /* The definition of WORD_2: */
    u32             edca0_used:4;
    u32             edca1_used:5;
    u32             edca2_used:5;
    u32             edca3_used:5;
    u32             mng_used:4;
    u32             tx_page_used:9;

    /* The definition of WORD_3: */
    u32             hdr_offset:8;
    u32             frag:1;
    u32             unicast:1;
    u32             hdr_len:6;
    u32             RxResult:8;
    u32             wildcard_bssid:1;
    u32             RSVD_1:1;
    u32             reason:6;

    /* The definition of WORD_4: */
    u32             payload_offset:8;
    u32             tx_id_used:8;
    u32             fCmdIdx:3;
    u32             wsid:4;
    u32             RSVD_3:3;
    u32             rate_idx:6;

    HCI_aggr_Info   hci_agr_info;

};



struct ssv6200_rxphy_info {
    /* WORD 1: */
    u32             len:16;
    u32             rsvd0:16;

    /* WORD 2: */
    u32             mode:3;
    u32             ch_bw:3;
    u32             preamble:1;
    u32             ht_short_gi:1;
    u32             rate:7;
    u32             rsvd1:1;
    u32             smoothing:1;
    u32             no_sounding:1;
    u32             aggregate:1;
    u32             stbc:2;
    u32             fec:1;
    u32             n_ess:2;
    u32             rsvd2:8;

    /* WORD 3: */
    u32             l_length:12;
    u32             l_rate:3;
    u32             rsvd3:17;

    /* WORD 4: */
    u32             rsvd4;

    /* WORD 5: G, N mode only */
    u32             rpci:8;     /* RSSI */
    u32             snr:8;
    u32             service:16;

};




struct ssv6200_rxphy_info_padding {

/* WORD 1: for B, G, N mode */
u32             rpci:8;     /* RSSI */
u32             snr:8;
u32             RSVD:16;
};



struct ssv6200_txphy_info {
    u32             rsvd[7];

};


/**
 *  struct cfg_host_cmd - Host Command Header Format description
 * 
 */
typedef struct cfg_host_cmd {
    u32             len:16;
    u32             c_type:3;
    u32             RSVD0:5;//It will be used as command index eg.  STA-WSID[0]-->RSVD0=0, STA-WSID[1]-->RSVD0=1
    u32             h_cmd:8;//------------------------->ssv_host_cmd/command id
    u32             cmd_seq_no;
    union { /*lint -save -e157 */
    u32             dummy; // Put a u32 dummy to make MSVC and GCC treat HDR_HostCmd as the same size.
    u8              dat8[0];
    u16             dat16[0];
    u32             dat32[0];
    }; /*lint -restore */
} HDR_HostCmd;
// Use 100 instead of 0 to get header size to avoid lint from reporting null pointer access.
#define HOST_CMD_HDR_LEN        ((size_t)(((HDR_HostCmd *)100)->dat8)-100U)


typedef enum{
//===========================================================================    
    //Public command        
    SSV6XXX_HOST_CMD_START                  = 0                                                     ,
    //SSV6XXX_HOST_CMD_SET_REG                                                                        ,
    //SSV6XXX_HOST_CMD_GET_REG                                                                        ,
    SSV6XXX_HOST_CMD_LOG                                                                            ,
    SSV6XXX_HOST_CMD_PS                                                                             ,
    SSV6XXX_HOST_CMD_INIT_CALI                                                                      ,
#ifdef FW_WSID_WATCH_LIST
    SSV6XXX_HOST_CMD_WSID_OP                                                                        ,
#endif

    SSV6XXX_HOST_SOC_CMD_MAXID                                                                      ,  
    
//===========================================================================    
}ssv6xxx_host_cmd_id;

/* The maximal number of hardware offload STAs */
#define SSV_NUM_HW_STA  2

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
    u32             evt_seq_no;
    u8              dat[0];
    
} HDR_HostEvent;

typedef enum{
//===========================================================================    
    //Public event
#ifdef USE_CMD_RESP
    SOC_EVT_CMD_RESP                        , // Response of a host command.
    SOC_EVT_SCAN_RESULT                     , // Scan result from probe response or beacon
    SOC_EVT_DEAUTH                          , // Deauthentication received but not for leave command
#else
    SOC_EVT_GET_REG_RESP                    ,
#endif // USE_CMD_RESP
    SOC_EVT_NO_BA                           ,
    SOC_EVT_RC_MPDU_REPORT                  ,
    SOC_EVT_RC_AMPDU_REPORT                 ,
    SOC_EVT_LOG                             ,           // ssv log module soc event
    SOC_EVT_USER_END                        ,
    
//===========================================================================    
    //Private    event
    SOC_EVT_MAXID                           ,
} ssv6xxx_soc_event;



typedef enum{
//===========================================================================    
    SSV6XXX_RC_COUNTER_CLEAR                = 1                                                     ,
    SSV6XXX_RC_REPORT                                                                            ,
    
//===========================================================================    
}ssv6xxx_host_rate_control_event;

typedef enum __PBuf_Type_E {
    NOTYPE_BUF  = 0,
    TX_BUF      = 1,
    RX_BUF      = 2    
} PBuf_Type_E;


#define MAX_AGGR_NUM   (24)


struct ssv62xx_tx_rate {
    s8 data_rate;
    u8 count;
} __attribute__((packed));

struct ampdu_ba_notify_data {
    //u16 retry_count;
    u8  wsid;
    struct ssv62xx_tx_rate tried_rates[SSV62XX_TX_MAX_RATES];
    u16 seq_no[MAX_AGGR_NUM];    
} __attribute__((packed));

struct firmware_rate_control_report_data{
    u8 wsid;
    struct ssv62xx_tx_rate rates[SSV62XX_TX_MAX_RATES];
    u16 ampdu_len;
    u16 ampdu_ack_len;
    int ack_signal;
    /* 15 bytes free */
} __attribute__((packed));

#define RC_RETRY_PARAM_OFFSET  ((sizeof(struct fw_rc_retry_params))*SSV62XX_TX_MAX_RATES)
#define SSV_RC_RATE_MAX                     39

#ifdef FW_WSID_WATCH_LIST
enum SSV6XXXX_WSID_OPS
{
    SSV6XXXX_WSID_OPS_ADD,
    SSV6XXXX_WSID_OPS_DEL,
    SSV6XXXX_WSID_OPS_RESETALL,
    SSV6XXXX_WSID_OPS_MAX
};

struct ssv6xxx_wsid_params
{
    u8 cmd;
    u8 wsid_idx;
    u8 target_wsid[6];
}; 
#endif //#ifdef FW_WSID_WATCH_LIST

struct ssv6xxx_iqk_cfg {
    u32 cfg_xtal:8;
    u32 cfg_pa:8;
    u32 cfg_tssi_trgt:8;
    u32 cfg_tssi_div:8;
    u32 cfg_def_tx_scale_11b:8;
    u32 cfg_def_tx_scale_11b_p0d5:8;
    u32 cfg_def_tx_scale_11g:8;
    u32 cfg_def_tx_scale_11g_p0d5:8;
    u32 cfg_papd_tx_scale_11b:8;
    u32 cfg_papd_tx_scale_11b_p0d5:8;
    u32 cfg_papd_tx_scale_11g:8;
    u32 cfg_papd_tx_scale_11g_p0d5:8;
    u32 cmd_sel;
    union {
        u32 fx_sel;
        u32 argv;
    };
    u32 phy_tbl_size;
    u32 rf_tbl_size;
};
#define PHY_SETTING_SIZE sizeof(phy_setting)

#ifdef CONFIG_SSV_CABRIO_E
#define IQK_CFG_LEN         (sizeof(struct ssv6xxx_iqk_cfg))
#define RF_SETTING_SIZE     (sizeof(asic_rf_setting))
#endif

/*
    If change defallt value .please recompiler firmware image.
*/
#define MAX_PHY_SETTING_TABLE_SIZE    1920
#define MAX_RF_SETTING_TABLE_SIZE    512

#endif /* _SSV6200_H_ */
