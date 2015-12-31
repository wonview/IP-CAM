#ifndef _SSV_CFG_H_
#define _SSV_CFG_H_



/**
* SSV6200 Hardware Capabilities:
*
* @ SSV6200_HW_CAP_HT: hardware supports HT capability.
* @ SSV6200_HW_CAP_LDPC:
* @ SSV6200_HW_CAP_2GHZ:
* @ SSV6200_HW_CAP_5GHZ:
* @ SSV6200_HW_CAP_DFS:
* @ SSV6200_HW_CAP_SECUR:
*/
#define SSV6200_HW_CAP_HT                   0x00000001
#define SSV6200_HW_CAP_GF                   0x00000002
#define SSV6200_HW_CAP_2GHZ                 0x00000004
#define SSV6200_HW_CAP_5GHZ                 0x00000008
#define SSV6200_HW_CAP_DFS                  0x00000010
#define SSV6200_HW_CAP_SECURITY             0x00000020
#define SSV6200_HW_CAP_RC                   0x00000040
#define SSV6200_HT_CAP_SGI_20               0x00000080
#define SSV6200_HT_CAP_SGI_40               0x00000100
#define SSV6200_HW_CAP_LDPC                 0x00000200
#define SSV6200_HW_CAP_STBC                 0x00000400
#define SSV6200_HW_CAP_B                    0x00000800
#define SSV6200_HW_CAP_G                    0x00001000
#define SSV6200_HW_CAP_AP                   0x00002000
#define SSV6200_HW_CAP_P2P                  0x00004000
#define SSV6200_HW_CAP_DUP_DETECT           0x00008000
#define SSV6200_HW_CAP_MC_FILTER            0x00010000
#define SSV6200_HW_CAP_L34_OFFLOAD          0x00020000
#define SSV6200_HW_CAP_AMPDU_RX             0x00040000
#define SSV6200_HW_CAP_AMPDU_TX             0x00080000




struct ssv6xxx_cfg {
    /**
        * ssv6200 hardware capabilities sets.
        */
    u32 hw_caps;
    
    u8  drv_type;

    /**
        * The default channel once the wifi system is up.
        */
    u8  def_chan;

    /**
        * The ssv6200 hardware configuration value. These vcalues set
        * the tx/rx packet buffer offset.
        */
    u8  txpb_offset;
    u8  rxpb_offset;

    /**
        * The mac address of Wifi STA .
        */
    u8 maddr[2][6];
    u32     n_maddr;

    //E-fuse configuration
    u8 r_calbration_result;
    u8 sar_result;
    u8 crystal_frequecy_offse;
    u8 dc_calbration_result;
    u16 iq_calbration_result;
    u8 tx_power_index_1;
    u8 tx_power_index_2;
};




#endif /* _SSV_CFG_H_ */

