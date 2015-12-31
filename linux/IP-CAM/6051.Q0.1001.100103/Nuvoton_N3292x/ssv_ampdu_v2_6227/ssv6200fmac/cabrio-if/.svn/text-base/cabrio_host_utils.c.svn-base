#include <linux/ieee80211.h>

#include "common.h"
#include "cmd_def.h"
#include "ssv_pktdef.h"
#include "cabrio_host_utils.h"

u8* ssv_host_tx_req_get_qos_ptr(struct cfg_host_txreq0 *req0)
{
    u16 nOffset = 0;
    switch (req0->c_type)
    {
        case SSV_TX_REQ0:
            nOffset += sizeof(struct cfg_host_txreq0);
            break;
//         case SSV_TX_REQ1:
//             nOffset += sizeof(struct cfg_host_txreq1);
//             break;
//         case SSV_TX_REQ2:
//             nOffset += sizeof(struct cfg_host_txreq2);
//             break;
        default:
            //ASSERT(FALSE);
            //break;
            return NULL;
    }


    if (req0->use_4addr)
        nOffset+=ETHER_ADDR_LEN;

    if (!req0->qos)    
        return NULL;
        
    if (req0->ht)
        nOffset+=IEEE80211_HT_CTL_LEN;

    return (u8*)(((u8*)req0)+nOffset);
}


u8* ssv_host_tx_req_get_data_ptr(struct cfg_host_txreq0 *req0)
{
    u16 nOffset = 0;
    switch (req0->c_type)
    {
        case SSV_TX_REQ0:
            nOffset += sizeof(struct cfg_host_txreq0);
            break;
//         case SSV_TX_REQ1:
//             nOffset += sizeof(struct cfg_host_txreq1);
//             break;
//         case SSV_TX_REQ2:
//             nOffset += sizeof(struct cfg_host_txreq2);
//             break;
        default:
            //ASSERT(FALSE);
            //break;
            return NULL;
    }

    
    if (req0->use_4addr)
        nOffset+=ETHER_ADDR_LEN;
    
    if (req0->qos)
        nOffset+=IEEE80211_QOS_CTL_LEN;

    if (req0->ht)
        nOffset+=IEEE80211_HT_CTL_LEN;

        
    return (u8*)(((u8*)req0)+nOffset);
}


u16 ssv_host_rx_get_raw_data_offset (struct cfg_host_rxpkt *rx_pkt_hdr)
{
    u16 offset=0;

    do {
        switch (rx_pkt_hdr->c_type)
        {
            case M0_RXEVENT:
                offset = RX_M0_HDR_LEN;
                break;
            case M2_RXEVENT:
                // ASSERT(FALSE);
                offset = M2_HDR_LEN;
                break;

            default:
                break;
        }

        //mac80211 no need to put extra header.
        if(rx_pkt_hdr->f80211)
            break;
                
        /*|(AL)|MAC|QOS|HT|*/
        if(rx_pkt_hdr->ht == 1)
            offset += IEEE80211_HT_CTL_LEN;

        if(rx_pkt_hdr->qos == 1)
            offset +=  IEEE80211_QOS_CTL_LEN;                

        if(rx_pkt_hdr->use_4addr == 1)
            offset +=  ETHER_ADDR_LEN;
    
        if(rx_pkt_hdr->align2 == 1)
            offset +=  2;

    } while (0);

    return offset;
}

