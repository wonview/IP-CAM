/*
 * This file contains the handling of command
 * responses as well as events generated by firmware.
 */

#include <linux/hardirq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <asm/unaligned.h>
#include <net/cfg80211.h>
#include <net/lib80211.h>

#include "cfg.h"
#include "cmd.h"
#include "cabrio-if/cabrio-if.h"

#define CABRIO_SCAN_RSSI_TO_MBM(rssi) \
	((-(int)rssi + 3)*100)

static void _scan_result_handler (struct cabrio_private *priv, struct resp_evt_result *resp, u32 resp_size)
{
    struct ssv_ieee80211_bss *ssv_bss = &resp->u.scan.bss_info;

    cabrio_dbg_enter_args(CABRIO_DBG_SCAN, ": %d - %d - %zd - %zd", ssv_bss->channel_id, resp->u.scan.dat_size, SCAN_RESP_EVT_HEADER_SIZE, sizeof(struct ssv_ieee80211_bss));
    /* No channel, no luck */

    if (ssv_bss->channel_id != (-1)) {
        struct HDR80211_Mgmt_st  *mgmt_frame = (struct HDR80211_Mgmt_st *)resp->u.scan.dat;
        struct wiphy   *wiphy = priv->wdev->wiphy;
        int             freq = ieee80211_channel_to_frequency(ssv_bss->channel_id,
                                                  IEEE80211_BAND_2GHZ);
        struct ieee80211_channel *channel = ieee80211_get_channel(wiphy, freq);
        const u8       *bssid;
        const u8       *ie;
        int 			ielen;
        const u8       *timestamp;
        int             rssi;
        u16             intvl;
        u16             capa;
        __le16          fc;

        DECLARE_SSID_BUF(ssid_buf);

        bssid = mgmt_frame->bssid.addr;
        /* RSSI */
        rssi = -20; // TODO: Get RSSI from PHY information in the frame.
        // Information needed in common fields of beacon and probe response.
        fc = *(__le16 *)&mgmt_frame->fc;
        if (ieee80211_is_probe_resp(fc)) {
        	intvl = mgmt_frame->u.probe_resp.beacon_int;
        	capa = mgmt_frame->u.probe_resp.capab_info;
        	timestamp = (u8 *)&mgmt_frame->u.probe_resp.timestamp;
        	ie = mgmt_frame->u.probe_resp.variable;
        } else if (ieee80211_is_beacon(fc)) {
        	intvl = mgmt_frame->u.beacon.beacon_int;
        	capa = mgmt_frame->u.beacon.capab_info;
        	timestamp = (u8 *)&mgmt_frame->u.beacon.timestamp;
        	ie = mgmt_frame->u.probe_resp.variable;
        } else {
        	cabrio_dbg_scan("Frame from scan response is in unknown subtype %d.\n", fc & IEEE80211_FCTL_STYPE);
        	return;
        }
        ielen = resp->u.scan.dat_size - (u32)(ie - (u8 *)mgmt_frame);
        BUG_ON((size_t)(ie + ielen) > (size_t)(resp + resp_size));

        cabrio_dbg_scan("scan: %pM, capa %04x, chan %2d, %s, %d dBm\n",
                 bssid, capa, ssv_bss->channel_id,
                 print_ssid(ssid_buf, ssv_bss->ssid.ssid, ssv_bss->ssid.ssid_len),
                 CABRIO_SCAN_RSSI_TO_MBM(rssi)/100);

        if (channel &&
            !(channel->flags & IEEE80211_CHAN_DISABLED))
            cfg80211_inform_bss(wiphy, channel,
                bssid, get_unaligned_le64(timestamp),
                capa, intvl, ie, ielen,
                CABRIO_SCAN_RSSI_TO_MBM(rssi),
                GFP_KERNEL);
    } else
        cabrio_dbg_scan("scan response: missing BSS channel IE\n");

    cabrio_dbg_leave(CABRIO_DBG_SCAN);
} // end of - _scan_result_handler -


#if 0
    const u8 *bssid;
    const u8 *ie;
    int left;
    int ielen;
    int rssi;
    u16 intvl;
    u16 capa;
    int chan_no = -1;
    const u8 *ssid = NULL;
    u8 ssid_len = 0;
    DECLARE_SSID_BUF(ssid_buf);
    
    int len = get_unaligned_le16(pos);
    pos += 2;
    
    /* BSSID */
    bssid = pos;
    pos += ETH_ALEN;
    /* RSSI */
    rssi = *pos++;
    /* Packet time stamp */
    pos += 8;
    /* Beacon interval */
    intvl = get_unaligned_le16(pos);
    pos += 2;
    /* Capabilities */
    capa = get_unaligned_le16(pos);
    pos += 2;
    
    /* To find out the channel, we must parse the IEs */
    ie = pos;
    /*
     * 6+1+8+2+2: size of BSSID, RSSI, time stamp, beacon
     * interval, capabilities
     */
    ielen = left = len - (6 + 1 + 8 + 2 + 2);
    while (left >= 2) {
        u8 id, elen;
        id = *pos++;
        elen = *pos++;
        left -= 2;
        if (elen > left || elen == 0) {
            cabrio_dbg_scan("scan response: invalid IE fmt\n");
            goto done;
        }
    
        if (id == WLAN_EID_DS_PARAMS)
            chan_no = *pos;
        if (id == WLAN_EID_SSID) {
            ssid = pos;
            ssid_len = elen;
        }
        left -= elen;
        pos += elen;
    }
    
    /* No channel, no luck */
    if (chan_no != -1) {
        struct wiphy *wiphy = priv->wdev->wiphy;
        int freq = ieee80211_channel_to_frequency(chan_no,
                        IEEE80211_BAND_2GHZ);
        struct ieee80211_channel *channel =
            ieee80211_get_channel(wiphy, freq);
    
        cabrio_dbg_scan("scan: %pM, capa %04x, chan %2d, %s, "
                 "%d dBm\n",
                 bssid, capa, chan_no,
                 print_ssid(ssid_buf, ssid, ssid_len),
                 CABRIO_SCAN_RSSI_TO_MBM(rssi)/100);
    
        if (channel &&
            !(channel->flags & IEEE80211_CHAN_DISABLED))
            cfg80211_inform_bss(wiphy, channel,
                bssid, get_unaligned_le64(tsfdesc),
                capa, intvl, ie, ielen,
                CABRIO_SCAN_RSSI_TO_MBM(rssi),
                GFP_KERNEL);
    } else
        cabrio_dbg_scan("scan response: missing BSS channel IE\n");
    
    tsfdesc += 8;

    struct wiphy *wiphy = priv->wdev->wiphy;
    int freq = ieee80211_channel_to_frequency(chan_no,
                    IEEE80211_BAND_2GHZ);
    struct ieee80211_channel *channel =
        ieee80211_get_channel(wiphy, freq);

    cabrio_dbg_scan("scan: %pM, capa %04x, chan %2d, %s, "
             "%d dBm\n",
             bssid, capa, chan_no,
             print_ssid(ssid_buf, ssid, ssid_len),
             CABRIO_SCAN_RSSI_TO_MBM(rssi)/100);

    if (channel &&
        !(channel->flags & IEEE80211_CHAN_DISABLED))
        cfg80211_inform_bss(wiphy, channel,
            bssid, get_unaligned_le64(tsfdesc),
            capa, intvl, ie, ielen,
            CABRIO_SCAN_RSSI_TO_MBM(rssi),
            GFP_KERNEL);
} else
    cabrio_dbg_scan("scan response: missing BSS channel IE\n");
#endif

#if 0
static void _join_result_handler (struct cabrio_private *priv, struct resp_evt_result *resp)
{
    #if 1
    CABRIO_TODO(__func__);
    #else
    struct resp_evt_result *join_res = (struct resp_evt_result *)data;
    if (join_res->result.status_code != 0) 
    {
        LOG_PRINTF("Join failure!!\n");
		return;
    }
    
    LOG_PRINTF("Join success!! AID=%d\n", join_res->data.aid);
	ssv_wifi_apply_security();
    //sim_set_link_status(LINK_UP);
   
#if 0
			netif_add(&wlan, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
			netif_set_default(&wlan);
			netif_set_up(&wlan);
			dhcp_start(&wlan);
		   for(i=0;i<6;i++)
		   {
			 wlan.hwaddr[ i ] = MAC_ADDR[i];
			
				
		   }
			while (wlan.ip_addr.addr==0) {
				sys_msleep(DHCP_FINE_TIMER_MSECS);
				dhcp_fine_tmr();
				mscnt += DHCP_FINE_TIMER_MSECS;
			 if (mscnt >= DHCP_COARSE_TIMER_SECS*1000) {
				dhcp_coarse_tmr();
				mscnt = 0;
			 }
		  }
#endif
    #endif // TODO
} // end of - _join_result_handler -


static void _leave_result_handler (struct cabrio_private *priv, struct resp_evt_result *resp)
{
    //struct resp_evt_result *leave_res = (struct resp_evt_result *)data;
	// LOG_PRINTF("Deauth from AP (status=%d) !!\n", leave_res->result.status_code);
    cabrio_mac_event_disconnected(priv);
    // sim_set_link_status(LINK_DOWN);
}
#endif

#if 0
static void _handle_cmd_resp (struct cabrio_private *priv, struct resp_evt_result *resp, u32 resp_len)
{
	u32 data_len;
	if (resp->result != CMD_OK)
	{
		cabrio_dbg_host("Command %d is not OK with code %d.\n", resp->cmd, resp->result);
		return;
    }
    switch (resp->cmd)
        {
        case SSV_HOST_CMD_SCAN:
        	cabrio_dbg_host("Scan done.\n");
            break;
        case SSV_HOST_CMD_JOIN:
            _join_result_handler(priv, resp);
            break;
        case SSV_HOST_CMD_LEAVE:
            _leave_result_handler(priv, resp);
            break;
        }

    data_len = resp_len - RESP_EVT_HEADER_SIZE;
    switch (resp->cmd)
        {
        case SSV_HOST_CMD_GET_REG:
            //_get_soc_reg_response(resp->cmd, resp->u.dat, data_len);
            break;
        case SSV_HOST_CMD_GET_STA_MAC:
            //_soc_evt_get_sta_mac(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_BSSID:
            //_soc_evt_get_bssid(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_ETHER_TRAP:
            //_soc_evt_get_ether_trap_tbl(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_FCMDS:
            //_soc_evt_get_fcmds(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_PHY_INFO_TBL:
            //_soc_evt_get_phy_info(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_DECI_TBL:
            //_soc_evt_get_decision_tbl(resp->cmd, data, data_len);
            break;
        case SSV_HOST_CMD_GET_WSID_TBL:
            //_soc_evt_get_wsid_tbl(resp->cmd, data, data_len);
            break;
        }
} // end of - _handle_cmd_resp -
#endif

#if 0
static void _get_soc_reg_response(struct cabrio_private *priv, void *data, s32 len)
{
    #if 0
    LOG_PRINTF("%s(): HOST_EVENT=%d: len=%d\n", __FUNCTION__, eid, len);
    memcpy((void *)g_soc_cmd_rx_buffer, (void *)data, len);
    g_soc_cmd_rx_ready = 1;
    #endif
}
#endif

static void _soc_evt_handler_ssv_log(struct cabrio_private *priv, void *data, s32 len)
{
	// LOG_host_exec_soc_evt((log_soc_evt_st *)data);
	return;
}


/**
 * cabrio_mac_event_disconnected - handles disconnect event. It
 * reports disconnect to upper layer, clean tx/rx packets,
 * reset link state etc.
 *
 * @priv:	A pointer to struct cabrio_private structure
 *
 * returns:	n/a
 */
void cabrio_mac_event_disconnected(struct cabrio_private *priv)
{
	if (priv->connect_status != CABRIO_CONNECTED)
		return;

	cabrio_dbg_enter(CABRIO_DBG_ASSOC);

	/*
	 * Cisco AP sends EAP failure and de-auth in less than 0.5 ms.
	 * It causes problem in the Supplicant
	 */
	msleep_interruptible(1000);

	if (priv->wdev->iftype == NL80211_IFTYPE_STATION)
		cabrio_send_disconnect_notification(priv);

	/* report disconnect to upper layer */
	netif_stop_queue(priv->dev);
	netif_carrier_off(priv->dev);

	/* Free Tx and Rx packets */
	kfree_skb(priv->currenttxskb);
	priv->currenttxskb = NULL;
	priv->tx_pending_len = 0;

	priv->connect_status = CABRIO_DISCONNECTED;

    // SSV TODO -- 
    #if 0
	if (priv->psstate != PS_STATE_FULL_POWER) {
		/* make firmware to exit PS mode */
		cabrio_dbg_cmd("disconnected, so exit PS mode\n");
		cabrio_set_ps_mode(priv, PS_MODE_ACTION_EXIT_PS, false);
	}
    #endif
	cabrio_dbg_leave(CABRIO_DBG_ASSOC);
}

int cabrio_process_command_response(struct cabrio_private *priv, u8 *data, u32 len)
{
	uint16_t respcmd, curcmd;
	int ret = 0;
	unsigned long flags;
	uint16_t result;
    HDR_HostEvent *p_host_evt = (HDR_HostEvent *)data;
    struct resp_evt_result *resp;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

    
	mutex_lock(&priv->lock);
	spin_lock_irqsave(&priv->driver_lock, flags);

	if (!priv->cur_cmd) {
		cabrio_dbg_host("CMD_RESP: cur_cmd is NULL\n");
		ret = -1;
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		goto done;
	}

	resp = (struct resp_evt_result *)&p_host_evt->dat[0];
	curcmd = priv->cur_cmd->cmdbuf->h_cmd;
	respcmd = resp->cmd;
    result = (uint16_t)resp->result;
    #if 1
	cabrio_dbg_cmd("CMD_RESP: response 0x%04x, seq %d, size %d\n",
		           respcmd, le16_to_cpu(resp->cmd_seq_no), len);
	//cabrio_dbg_hex(CABRIO_DBG_CMD, "CMD_RESP", (void *) resp, len);

	if (resp->cmd_seq_no != priv->cur_cmd->cmdbuf->cmd_seq_no) {
		netdev_info(priv->dev,
			    "Received CMD_RESP with invalid sequence %d (expected %d)\n",
			    le16_to_cpu(resp->cmd_seq_no),
			    le16_to_cpu(priv->cur_cmd->cmdbuf->cmd_seq_no));
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}
    #endif
	if (respcmd != curcmd /* SSV -- respcmd != CMD_RET(curcmd) &&
	    respcmd != CMD_RET_802_11_ASSOCIATE && curcmd != CMD_802_11_ASSOCIATE */) {
		netdev_info(priv->dev, "Invalid command response %x to command %x!\n",
			    respcmd, curcmd);
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}
    #if 0 
	if (resp->result == cpu_to_le16(0x0004)) {
		/* 0x0004 means -EAGAIN. Drop the response, let it time out
		   and be resubmitted */
		netdev_info(priv->dev,
			    "Firmware returns DEFER to command %x. Will let it time out...\n",
			    le16_to_cpu(resp->command));
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}
    #endif
    
	/* Now we got response from FW, cancel the command timer */
	del_timer(&priv->command_timer);
	priv->cmd_timed_out = 0;
    // SSV --
    #if 0 
	if (respcmd == CMD_RET(CMD_802_11_PS_MODE)) {
		struct cmd_ds_802_11_ps_mode *psmode = (void *) &resp[1];
		u16 action = le16_to_cpu(psmode->action);

		cabrio_dbg_host(
		       "CMD_RESP: PS_MODE cmd reply result 0x%x, action 0x%x\n",
		       result, action);

		if (result) {
			cabrio_dbg_host("CMD_RESP: PS command failed with 0x%x\n",
				    result);
			/*
			 * We should not re-try enter-ps command in
			 * ad-hoc mode. It takes place in
			 * cabrio_execute_next_command().
			 */
			if (priv->wdev->iftype == NL80211_IFTYPE_MONITOR &&
			    action == PS_MODE_ACTION_ENTER_PS)
				priv->psmode = CABRIO802_11POWERMODECAM;
		} else if (action == PS_MODE_ACTION_ENTER_PS) {
			priv->needtowakeup = 0;
			priv->psstate = PS_STATE_AWAKE;

			cabrio_dbg_host("CMD_RESP: ENTER_PS command response\n");
			if (priv->connect_status != CABRIO_CONNECTED) {
				/*
				 * When Deauth Event received before Enter_PS command
				 * response, We need to wake up the firmware.
				 */
				cabrio_dbg_host(
				       "disconnected, invoking cabrio_ps_wakeup\n");

				spin_unlock_irqrestore(&priv->driver_lock, flags);
				mutex_unlock(&priv->lock);
				cabrio_set_ps_mode(priv, PS_MODE_ACTION_EXIT_PS,
						false);
				mutex_lock(&priv->lock);
				spin_lock_irqsave(&priv->driver_lock, flags);
			}
		} else if (action == PS_MODE_ACTION_EXIT_PS) {
			priv->needtowakeup = 0;
			priv->psstate = PS_STATE_FULL_POWER;
			cabrio_dbg_host("CMD_RESP: EXIT_PS command response\n");
		} else {
			cabrio_dbg_host("CMD_RESP: PS action 0x%X\n", action);
		}
        /*
        SSV TODO
        Process private command response
        */
		__cabrio_complete_command(priv, priv->cur_cmd, result);
		spin_unlock_irqrestore(&priv->driver_lock, flags);

		ret = 0;
		goto done;
	}

	/* If the command is not successful, cleanup and return failure */
	if ((result != 0 || !(respcmd & 0x8000))) {
		cabrio_dbg_host("CMD_RESP: error 0x%04x in command reply 0x%04x\n",
		       result, respcmd);
		/*
		 * Handling errors here
		 */
		switch (respcmd) {
		case CMD_RET(CMD_GET_HW_SPEC):
		case CMD_RET(CMD_802_11_RESET):
			cabrio_dbg_host("CMD_RESP: reset failed\n");
			break;

		}
		__cabrio_complete_command(priv, priv->cur_cmd, result);
		spin_unlock_irqrestore(&priv->driver_lock, flags);

		ret = -1;
		goto done;
	}
    #endif

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	if (priv->cur_cmd && priv->cur_cmd->callback) {
		ret = priv->cur_cmd->callback(priv, priv->cur_cmd->callback_arg,
                                      p_host_evt);
	}

	spin_lock_irqsave(&priv->driver_lock, flags);

	if (priv->cur_cmd) {
		/* Clean up and Put current command back to cmdfreeq */
		__cabrio_complete_command(priv, priv->cur_cmd, 0/*result*/);
	}
	spin_unlock_irqrestore(&priv->driver_lock, flags);

done:
	mutex_unlock(&priv->lock);
	cabrio_dbg_leave_args(CABRIO_DBG_HOST, "ret %d", ret);
	return ret;
}

#if ORIG
int cabrio_process_command_response_orig(struct cabrio_private *priv, u8 *data, u32 len)
{
	uint16_t respcmd, curcmd;
	struct cmd_header *resp;
	int ret = 0;
	unsigned long flags;
	uint16_t result;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	mutex_lock(&priv->lock);
	spin_lock_irqsave(&priv->driver_lock, flags);

	if (!priv->cur_cmd) {
		cabrio_dbg_host("CMD_RESP: cur_cmd is NULL\n");
		ret = -1;
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		goto done;
	}

	resp = (void *)data;
	curcmd = le16_to_cpu(priv->cur_cmd->cmdbuf->command);
	respcmd = le16_to_cpu(resp->command);
	result = le16_to_cpu(resp->result);

	cabrio_dbg_cmd("CMD_RESP: response 0x%04x, seq %d, size %d\n",
		     respcmd, le16_to_cpu(resp->seqnum), len);
	cabrio_dbg_hex(CABRIO_DBG_CMD, "CMD_RESP", (void *) resp, len);

	if (resp->seqnum != priv->cur_cmd->cmdbuf->seqnum) {
		netdev_info(priv->dev,
			    "Received CMD_RESP with invalid sequence %d (expected %d)\n",
			    le16_to_cpu(resp->seqnum),
			    le16_to_cpu(priv->cur_cmd->cmdbuf->seqnum));
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}
	if (respcmd != CMD_RET(curcmd) &&
	    respcmd != CMD_RET_802_11_ASSOCIATE && curcmd != CMD_802_11_ASSOCIATE) {
		netdev_info(priv->dev, "Invalid CMD_RESP %x to command %x!\n",
			    respcmd, curcmd);
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}

	if (resp->result == cpu_to_le16(0x0004)) {
		/* 0x0004 means -EAGAIN. Drop the response, let it time out
		   and be resubmitted */
		netdev_info(priv->dev,
			    "Firmware returns DEFER to command %x. Will let it time out...\n",
			    le16_to_cpu(resp->command));
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}

	/* Now we got response from FW, cancel the command timer */
	del_timer(&priv->command_timer);
	priv->cmd_timed_out = 0;

	if (respcmd == CMD_RET(CMD_802_11_PS_MODE)) {
		struct cmd_ds_802_11_ps_mode *psmode = (void *) &resp[1];
		u16 action = le16_to_cpu(psmode->action);

		cabrio_dbg_host(
		       "CMD_RESP: PS_MODE cmd reply result 0x%x, action 0x%x\n",
		       result, action);

		if (result) {
			cabrio_dbg_host("CMD_RESP: PS command failed with 0x%x\n",
				    result);
			/*
			 * We should not re-try enter-ps command in
			 * ad-hoc mode. It takes place in
			 * cabrio_execute_next_command().
			 */
			if (priv->wdev->iftype == NL80211_IFTYPE_MONITOR &&
			    action == PS_MODE_ACTION_ENTER_PS)
				priv->psmode = CABRIO802_11POWERMODECAM;
		} else if (action == PS_MODE_ACTION_ENTER_PS) {
			priv->needtowakeup = 0;
			priv->psstate = PS_STATE_AWAKE;

			cabrio_dbg_host("CMD_RESP: ENTER_PS command response\n");
			if (priv->connect_status != CABRIO_CONNECTED) {
				/*
				 * When Deauth Event received before Enter_PS command
				 * response, We need to wake up the firmware.
				 */
				cabrio_dbg_host(
				       "disconnected, invoking cabrio_ps_wakeup\n");

				spin_unlock_irqrestore(&priv->driver_lock, flags);
				mutex_unlock(&priv->lock);
				cabrio_set_ps_mode(priv, PS_MODE_ACTION_EXIT_PS,
						false);
				mutex_lock(&priv->lock);
				spin_lock_irqsave(&priv->driver_lock, flags);
			}
		} else if (action == PS_MODE_ACTION_EXIT_PS) {
			priv->needtowakeup = 0;
			priv->psstate = PS_STATE_FULL_POWER;
			cabrio_dbg_host("CMD_RESP: EXIT_PS command response\n");
		} else {
			cabrio_dbg_host("CMD_RESP: PS action 0x%X\n", action);
		}

		__cabrio_complete_command(priv, priv->cur_cmd, result);
		spin_unlock_irqrestore(&priv->driver_lock, flags);

		ret = 0;
		goto done;
	}

	/* If the command is not successful, cleanup and return failure */
	if ((result != 0 || !(respcmd & 0x8000))) {
		cabrio_dbg_host("CMD_RESP: error 0x%04x in command reply 0x%04x\n",
		       result, respcmd);
		/*
		 * Handling errors here
		 */
		switch (respcmd) {
		case CMD_RET(CMD_GET_HW_SPEC):
		case CMD_RET(CMD_802_11_RESET):
			cabrio_dbg_host("CMD_RESP: reset failed\n");
			break;

		}
		__cabrio_complete_command(priv, priv->cur_cmd, result);
		spin_unlock_irqrestore(&priv->driver_lock, flags);

		ret = -1;
		goto done;
	}

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	if (priv->cur_cmd && priv->cur_cmd->callback) {
		ret = priv->cur_cmd->callback(priv, priv->cur_cmd->callback_arg,
				resp);
	}

	spin_lock_irqsave(&priv->driver_lock, flags);

	if (priv->cur_cmd) {
		/* Clean up and Put current command back to cmdfreeq */
		__cabrio_complete_command(priv, priv->cur_cmd, result);
	}
	spin_unlock_irqrestore(&priv->driver_lock, flags);

done:
	mutex_unlock(&priv->lock);
	cabrio_dbg_leave_args(CABRIO_DBG_HOST, "ret %d", ret);
	return ret;
}
#endif // ORIG


int cabrio_process_event(struct cabrio_private *priv, void *event)
{
	int ret = 0;
	HDR_HostEvent *p_host_evt = (HDR_HostEvent *)event;
    u32 event_payload_size = p_host_evt->len - sizeof(HDR_HostEvent);

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	switch (p_host_evt->h_event) {
        case SOC_EVT_LOG:
            cabrio_dbg_cmd("EVENT: log\n");
            _soc_evt_handler_ssv_log(priv, p_host_evt->dat,
                                     event_payload_size);
            break;
		#ifdef USE_CMD_RESP
        case SOC_EVT_SCAN_RESULT:
            cabrio_dbg_cmd("EVENT: scan result\n");
            _scan_result_handler(priv, (struct resp_evt_result *)p_host_evt->dat,
            		             event_payload_size);
            break;
		#else // USE_CMD_RESP
        case SOC_EVT_JOIN_RESULT:
            cabrio_dbg_cmd("EVENT: join result\n");
            _join_result_handler(priv, p_host_evt->dat);
            break;
        case SOC_EVT_LEAVE_RESULT:
            cabrio_dbg_cmd("EVENT: leave result\n");
            _leave_result_handler(priv, p_host_evt->dat);
            break;
        case SOC_EVT_HW_MODE_RESP:
            cabrio_dbg_cmd("EVENT: TX done\n");
            break;
        case SOC_EVT_GET_REG_RESP:
            cabrio_dbg_cmd("EVENT: Get DIFS\n");
            _get_soc_reg_response(priv, p_host_evt->dat,
                                  event_payload_size);
            break;
        case SOC_EVT_GET_STA_MAC_RESP:
            cabrio_dbg_cmd("EVENT: Get STA MAC\n");
            // _soc_evt_get_sta_mac(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_BSSID_RESP:
            cabrio_dbg_cmd("EVENT: Get DIFS\n");
            // _soc_evt_get_bssid(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_DECI_TABLE_RESP:
            cabrio_dbg_cmd("EVENT: Get DECI table\n");
            // _soc_evt_get_decision_tbl(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_WSID_TABLE_RESP:
            cabrio_dbg_cmd("EVENT: Get WSID table\n");
            // _soc_evt_get_wsid_tbl(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_ETHER_TRAP_RESP:
            cabrio_dbg_cmd("EVENT: Get ether trap\n");
            //_soc_evt_get_bssid(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_FCMDS_RESP:
            cabrio_dbg_cmd("EVENT: Get FCMDS\n");
            // _soc_evt_get_fcmds(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_PHY_INFO_TBL_RESP:
            cabrio_dbg_cmd("EVENT: Get PHY info table\n");
            // _soc_evt_get_phy_info(priv, p_host_evt->dat);
            break;
        case SOC_EVT_GET_SIFS_RESP:
            cabrio_dbg_cmd("EVENT: Get SIFS\n");
            break;
        case SOC_EVT_GET_DIFS_RESP:
            cabrio_dbg_cmd("EVENT: Get DIFS\n");
            break;
        case SOC_EVT_GET_EIFS_RESP:
            cabrio_dbg_cmd("EVENT: Get EIFS\n");
            break;
        #endif  //
        case SOC_EVT_DEAUTH:
        	cabrio_mac_event_disconnected(priv);
        	break;
        case SOC_EVT_ACK:
            cabrio_dbg_cmd("EVENT: ack\n");
            break;
        case SOC_EVT_TX_ALL_DONE:
            cabrio_dbg_cmd("EVENT: TX done\n");
            break;
        case SOC_EVT_CONFIG_HW_RESP:
            cabrio_dbg_cmd("EVENT: Config HW\n");
            break;
        case SOC_EVT_SET_BSS_PARAM_RESP:
            cabrio_dbg_cmd("EVENT: Set BSS param\n");
            break;
        case SOC_EVT_PS_POLL:
            cabrio_dbg_cmd("EVENT: TX done\n");
            break;
        case SOC_EVT_NULL_DATA:
            cabrio_dbg_cmd("EVENT: NULL data\n");
            break;
        case SOC_EVT_REG_RESULT:
            cabrio_dbg_cmd("EVENT: REG result\n");
            break;
    	default:
	    	netdev_alert(priv->dev, "EVENT: unknown event id %d\n", p_host_evt->h_event);
		    break;
	}

	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}


#if 0
int cabrio_process_event_orig(struct cabrio_private *priv, u32 event)
{
	int ret = 0;
	struct cmd_header cmd;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	switch (event) {
	case MACREG_INT_CODE_LINK_SENSED:
		cabrio_dbg_cmd("EVENT: link sensed\n");
		break;

	case MACREG_INT_CODE_DEAUTHENTICATED:
		cabrio_dbg_cmd("EVENT: deauthenticated\n");
		cabrio_mac_event_disconnected(priv);
		break;

	case MACREG_INT_CODE_DISASSOCIATED:
		cabrio_dbg_cmd("EVENT: disassociated\n");
		cabrio_mac_event_disconnected(priv);
		break;

	case MACREG_INT_CODE_LINK_LOST_NO_SCAN:
		cabrio_dbg_cmd("EVENT: link lost\n");
		cabrio_mac_event_disconnected(priv);
		break;

	case MACREG_INT_CODE_PS_SLEEP:
		cabrio_dbg_cmd("EVENT: ps sleep\n");

		/* handle unexpected PS SLEEP event */
		if (priv->psstate == PS_STATE_FULL_POWER) {
			cabrio_dbg_cmd(
			       "EVENT: in FULL POWER mode, ignoreing PS_SLEEP\n");
			break;
		}
		priv->psstate = PS_STATE_PRE_SLEEP;

		cabrio_ps_confirm_sleep(priv);

		break;

	case MACREG_INT_CODE_HOST_AWAKE:
		cabrio_dbg_cmd("EVENT: host awake\n");
		if (priv->reset_deep_sleep_wakeup)
			priv->reset_deep_sleep_wakeup(priv);
		priv->is_deep_sleep = 0;
		cabrio_cmd_async(priv, CMD_802_11_WAKEUP_CONFIRM, &cmd,
				sizeof(cmd));
		priv->is_host_sleep_activated = 0;
		wake_up_interruptible(&priv->host_sleep_q);
		break;

	case MACREG_INT_CODE_DEEP_SLEEP_AWAKE:
		if (priv->reset_deep_sleep_wakeup)
			priv->reset_deep_sleep_wakeup(priv);
		cabrio_dbg_cmd("EVENT: ds awake\n");
		priv->is_deep_sleep = 0;
		priv->wakeup_dev_required = 0;
		wake_up_interruptible(&priv->ds_awake_q);
		break;

	case MACREG_INT_CODE_PS_AWAKE:
		cabrio_dbg_cmd("EVENT: ps awake\n");
		/* handle unexpected PS AWAKE event */
		if (priv->psstate == PS_STATE_FULL_POWER) {
			cabrio_dbg_cmd(
			       "EVENT: In FULL POWER mode - ignore PS AWAKE\n");
			break;
		}

		priv->psstate = PS_STATE_AWAKE;

		if (priv->needtowakeup) {
			/*
			 * wait for the command processing to finish
			 * before resuming sending
			 * priv->needtowakeup will be set to FALSE
			 * in cabrio_ps_wakeup()
			 */
			cabrio_dbg_cmd("waking up ...\n");
			cabrio_set_ps_mode(priv, PS_MODE_ACTION_EXIT_PS, false);
		}
		break;

	case MACREG_INT_CODE_MIC_ERR_UNICAST:
		cabrio_dbg_cmd("EVENT: UNICAST MIC ERROR\n");
		cabrio_send_mic_failureevent(priv, event);
		break;

	case MACREG_INT_CODE_MIC_ERR_MULTICAST:
		cabrio_dbg_cmd("EVENT: MULTICAST MIC ERROR\n");
		cabrio_send_mic_failureevent(priv, event);
		break;

	case MACREG_INT_CODE_MIB_CHANGED:
		cabrio_dbg_cmd("EVENT: MIB CHANGED\n");
		break;
	case MACREG_INT_CODE_INIT_DONE:
		cabrio_dbg_cmd("EVENT: INIT DONE\n");
		break;
	case MACREG_INT_CODE_ADHOC_BCN_LOST:
		cabrio_dbg_cmd("EVENT: ADHOC beacon lost\n");
		break;
	case MACREG_INT_CODE_RSSI_LOW:
		netdev_alert(priv->dev, "EVENT: rssi low\n");
		break;
	case MACREG_INT_CODE_SNR_LOW:
		netdev_alert(priv->dev, "EVENT: snr low\n");
		break;
	case MACREG_INT_CODE_MAX_FAIL:
		netdev_alert(priv->dev, "EVENT: max fail\n");
		break;
	case MACREG_INT_CODE_RSSI_HIGH:
		netdev_alert(priv->dev, "EVENT: rssi high\n");
		break;
	case MACREG_INT_CODE_SNR_HIGH:
		netdev_alert(priv->dev, "EVENT: snr high\n");
		break;

	case MACREG_INT_CODE_MESH_AUTO_STARTED:
		/* Ignore spurious autostart events */
		netdev_info(priv->dev, "EVENT: MESH_AUTO_STARTED (ignoring)\n");
		break;

	default:
		netdev_alert(priv->dev, "EVENT: unknown event id %d\n", event);
		break;
	}

	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}
#endif

