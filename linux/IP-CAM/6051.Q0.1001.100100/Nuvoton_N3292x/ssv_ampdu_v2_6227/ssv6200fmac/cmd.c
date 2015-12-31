/*
 * This file contains the handling of command.
 * It prepares command and sends it to firmware when it is ready.
 */

#include <linux/hardirq.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/if_arp.h>
#include <linux/export.h>

#include "decl.h"
#include "cfg.h"
#include "cmd.h"
#include "defs.h"
#include "cabrio-if/cmd_def.h"
#include "cabrio-if/ssv_pktdef.h"
#include "cabrio-if/common.h"

#define CAL_NF(nf)		((s32)(-(s32)(nf)))
#define CAL_RSSI(snr, nf)	((s32)((s32)(snr) + CAL_NF(nf)))

/**
 * cabrio_cmd_copyback - Simple callback that copies response back into command
 *
 * @priv:	A pointer to &struct cabrio_private structure
 * @extra:	A pointer to the original command structure for which
 *		'resp' is a response
 * @resp:	A pointer to the command response
 *
 * returns:	0 on success, error on failure
 */
int cabrio_cmd_copyback(struct cabrio_private *priv, unsigned long extra,
                        HDR_HostEvent *event)
{
	HDR_HostEvent *event_buf = (void *)extra;
	uint16_t copy_len;

	copy_len = min(le16_to_cpu(event_buf->len), le16_to_cpu(event->len));
	pr_info("cmd rseponse data of %d/%d. %08X %08X -- %02X %02X %02X %02X\n", le16_to_cpu(event->len), copy_len,
			((u32 *)event)[0], ((u32 *)event)[1],
			((u8 *)event)[event_buf->len - 4], ((u8 *)event)[event_buf->len - 3],
			((u8 *)event)[event_buf->len - 2], ((u8 *)event)[event_buf->len - 1]);
	memcpy(event_buf, event, copy_len);
	return 0;
}
EXPORT_SYMBOL_GPL(cabrio_cmd_copyback);

/**
 *  cabrio_cmd_async_callback - Simple callback that ignores the result.
 *  Use this if you just want to send a command to the hardware, but don't
 *  care for the result.
 *
 *  @priv:	ignored
 *  @extra:	ignored
 *  @resp:	ignored
 *
 *  returns:	0 for success
 */
static int cabrio_cmd_async_callback(struct cabrio_private *priv, unsigned long extra,
		                             HDR_HostEvent *event)
{
	return 0;
}


#if ORIG
/**
 *  is_command_allowed_in_ps - tests if a command is allowed in Power Save mode
 *
 *  @cmd:	the command ID
 *
 *  returns:	1 if allowed, 0 if not allowed
 */
static u8 is_command_allowed_in_ps(u16 cmd)
{
	switch (cmd) {
	case CMD_802_11_RSSI:
		return 1;
	case CMD_802_11_HOST_SLEEP_CFG:
		return 1;
	default:
		break;
	}
	return 0;
}
#endif


/**
 *  cabrio_update_hw_spec - Updates the hardware details like MAC address
 *  and regulatory region
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_update_hw_spec(struct cabrio_private *priv)
{
	int 			ret = 0;
    u32             cmd_size =   HOST_CMD_HDR_LEN
                               + sizeof(ETHER_ADDR);
    u32             resp_size =   CMD_RESPONSE_BASE_SIZE
    		                    + sizeof(ETHER_ADDR);
    u32				buf_size = resp_size + cmd_size;
    u8             *buf = kzalloc(buf_size, GFP_KERNEL);
    HDR_HostCmd    *cmd = (HDR_HostCmd *)buf;
    HDR_HostEvent  *event = (HDR_HostEvent *)(buf + cmd_size);
    struct resp_evt_result *resp = (struct resp_evt_result *)&event->dat[0];
    ETHER_ADDR     *mac_addr = (ETHER_ADDR *)&resp->u.dat[0];

    cabrio_dbg_enter(CABRIO_DBG_CMD);

    #if 1
    if (cmd == NULL)
        return (-1);

    memset(cmd, 0, cmd_size);
    cmd->c_type = HOST_CMD;
    cmd->h_cmd = SSV_HOST_CMD_GET_STA_MAC;
    cmd->len = cmd_size;

    event->len = resp_size;

    ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_GET_STA_MAC, cmd, event);
    kfree(cmd);

    if (ret) {
        cabrio_dbg_cmd("Failed to get MAC address from device.\n");
        goto out;
    } else {
    	if (resp->result == CMD_OK)
    		cabrio_dbg_cmd("%08X %08X %08X %08X MAC address: %02X:%02X:%02X:%02X:%02X:%02X.\n",
    					   ((u32 *)resp)[0], ((u32 *)resp)[1], ((u32 *)resp)[2], ((u32 *)resp)[3],
    				       mac_addr->addr[0], mac_addr->addr[1],
    				       mac_addr->addr[2], mac_addr->addr[3],
    				       mac_addr->addr[4], mac_addr->addr[5]);
    }

    if (priv->current_addr[0] == 0xff)
        memmove(priv->current_addr, mac_addr, ETH_ALEN);

    if (!priv->copied_hwaddr) {
        memcpy(priv->dev->dev_addr, priv->current_addr, ETH_ALEN);
        priv->copied_hwaddr = 1;
    }
    #else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	memcpy(cmd.permanentaddr, priv->current_addr, ETH_ALEN);
	ret = cabrio_cmd_with_response(priv, CMD_GET_HW_SPEC, &cmd);
	if (ret)
		goto out;

	priv->fwcapinfo = le32_to_cpu(cmd.fwcapinfo);

	/* The firmware release is in an interesting format: the patch
	 * level is in the most significant nibble ... so fix that: */
	priv->fwrelease = le32_to_cpu(cmd.fwrelease);
	priv->fwrelease = (priv->fwrelease << 8) |
		(priv->fwrelease >> 24 & 0xff);

	/* Some firmware capabilities:
	 * CF card    firmware 5.0.16p0:   cap 0x00000303
	 * USB dongle firmware 5.110.17p2: cap 0x00000303
	 */
	netdev_info(priv->dev, "%pM, fw %u.%u.%up%u, cap 0x%08x\n",
		cmd.permanentaddr,
		priv->fwrelease >> 24 & 0xff,
		priv->fwrelease >> 16 & 0xff,
		priv->fwrelease >>  8 & 0xff,
		priv->fwrelease       & 0xff,
		priv->fwcapinfo);
	cabrio_dbg_cmd("GET_HW_SPEC: hardware interface 0x%x, hardware spec 0x%04x\n",
		    cmd.hwifversion, cmd.version);

	/* Clamp region code to 8-bit since FW spec indicates that it should
	 * only ever be 8-bit, even though the field size is 16-bit.  Some firmware
	 * returns non-zero high 8 bits here.
	 *
	 * Firmware version 4.0.102 used in CF8381 has region code shifted.  We
	 * need to check for this problem and handle it properly.
	 */
	// SSV ??
	#if 0
	if (CABRIO_FW_MAJOR_REV(priv->fwrelease) == CABRIO_FW_V4)
		priv->regioncode = (le16_to_cpu(cmd.regioncode) >> 8) & 0xFF;
	else
    #endif    
		priv->regioncode = le16_to_cpu(cmd.regioncode) & 0xFF;

	for (i = 0; i < CABRIO_MAX_REGION_CODE; i++) {
		/* use the region code to search for the index */
		if (priv->regioncode == cabrio_region_code_to_index[i])
			break;
	}

	/* if it's unidentified region code, use the default (USA) */
	if (i >= CABRIO_MAX_REGION_CODE) {
		priv->regioncode = 0x10;
		netdev_info(priv->dev,
			    "unidentified region code; using the default (USA)\n");
	}

	if (priv->current_addr[0] == 0xff)
		memmove(priv->current_addr, cmd.permanentaddr, ETH_ALEN);

	if (!priv->copied_hwaddr) {
		memcpy(priv->dev->dev_addr, priv->current_addr, ETH_ALEN);
		if (priv->mesh_dev)
			memcpy(priv->mesh_dev->dev_addr,
				priv->current_addr, ETH_ALEN);
		priv->copied_hwaddr = 1;
	}
#endif // TODO
out:
	cabrio_dbg_leave(CABRIO_DBG_CMD);
	return ret;
}


#if ORIG
static int cabrio_ret_host_sleep_cfg(struct cabrio_private *priv, unsigned long dummy,
			HDR_HostCmd *resp)
{
	cabrio_dbg_enter(CABRIO_DBG_CMD);
	if (priv->is_host_sleep_activated) {
		priv->is_host_sleep_configured = 0;
		if (priv->psstate == PS_STATE_FULL_POWER) {
			priv->is_host_sleep_activated = 0;
			wake_up_interruptible(&priv->host_sleep_q);
		}
	} else {
		priv->is_host_sleep_configured = 1;
	}
	cabrio_dbg_leave(CABRIO_DBG_CMD);
	return 0;
}


int cabrio_host_sleep_cfg(struct cabrio_private *priv, uint32_t criteria,
		struct wol_config *p_wol_config)
{
	// struct cmd_ds_host_sleep cmd_config;
	int ret = 0;

    #if 1
    CABRIO_TODO(__func__);
    #else
	/*
	 * Certain firmware versions do not support EHS_REMOVE_WAKEUP command
	 * and the card will return a failure.  Since we need to be
	 * able to reset the mask, in those cases we set a 0 mask instead.
	 */
	if (criteria == EHS_REMOVE_WAKEUP && !priv->ehs_remove_supported)
		criteria = 0;

	cmd_config.hdr.size = cpu_to_le16(sizeof(cmd_config));
	cmd_config.criteria = cpu_to_le32(criteria);
	cmd_config.gpio = priv->wol_gpio;
	cmd_config.gap = priv->wol_gap;

	if (p_wol_config != NULL)
		memcpy((uint8_t *)&cmd_config.wol_conf, (uint8_t *)p_wol_config,
				sizeof(struct wol_config));
	else
		cmd_config.wol_conf.action = CMD_ACT_ACTION_NONE;

	ret = __cabrio_cmd(priv, CMD_802_11_HOST_SLEEP_CFG, &cmd_config.hdr,
                       le16_to_cpu(cmd_config.hdr.size),
                       cabrio_ret_host_sleep_cfg, 0);
	if (!ret) {
		if (p_wol_config)
			memcpy((uint8_t *) p_wol_config,
					(uint8_t *)&cmd_config.wol_conf,
					sizeof(struct wol_config));
	} else {
		netdev_info(priv->dev, "HOST_SLEEP_CFG failed %d\n", ret);
	}
    #endif // TODO
	return ret;
}
EXPORT_SYMBOL_GPL(cabrio_host_sleep_cfg);
#endif // ORIG


/**
 *  cabrio_set_ps_mode - Sets the Power Save mode
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @cmd_action: The Power Save operation (PS_MODE_ACTION_ENTER_PS or
 *                         PS_MODE_ACTION_EXIT_PS)
 *  @block:	Whether to block on a response or not
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_set_ps_mode(struct cabrio_private *priv, u16 cmd_action, bool block)
{
	//struct cmd_ds_802_11_ps_mode cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(cmd_action);

	if (cmd_action == PS_MODE_ACTION_ENTER_PS) {
		cabrio_dbg_cmd("PS_MODE: action ENTER_PS\n");
		cmd.multipledtim = cpu_to_le16(1);  /* Default DTIM multiple */
	} else if (cmd_action == PS_MODE_ACTION_EXIT_PS) {
		cabrio_dbg_cmd("PS_MODE: action EXIT_PS\n");
	} else {
		/* We don't handle CONFIRM_SLEEP here because it needs to
		 * be fastpathed to the firmware.
		 */
		cabrio_dbg_cmd("PS_MODE: unknown action 0x%X\n", cmd_action);
		ret = -EOPNOTSUPP;
		goto out;
	}

	if (block)
		ret = cabrio_cmd_with_response(priv, CMD_802_11_PS_MODE, &cmd);
	else
		cabrio_cmd_async(priv, CMD_802_11_PS_MODE, &cmd.hdr, sizeof (cmd));

out:
    #endif // TODO
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}


int cabrio_cmd_802_11_sleep_params(struct cabrio_private *priv, uint16_t cmd_action,
				struct sleep_params *sp)
{
	//struct cmd_ds_802_11_sleep_params cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

    #if 1
    CABRIO_TODO(__func__);
    #else
	if (cmd_action == CMD_ACT_GET) {
		memset(&cmd, 0, sizeof(cmd));
	} else {
		cmd.error = cpu_to_le16(sp->sp_error);
		cmd.offset = cpu_to_le16(sp->sp_offset);
		cmd.stabletime = cpu_to_le16(sp->sp_stabletime);
		cmd.calcontrol = sp->sp_calcontrol;
		cmd.externalsleepclk = sp->sp_extsleepclk;
		cmd.reserved = cpu_to_le16(sp->sp_reserved);
	}
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(cmd_action);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_SLEEP_PARAMS, &cmd);

	if (!ret) {
		cabrio_dbg_cmd("error 0x%x, offset 0x%x, stabletime 0x%x, "
			    "calcontrol 0x%x extsleepclk 0x%x\n",
			    le16_to_cpu(cmd.error), le16_to_cpu(cmd.offset),
			    le16_to_cpu(cmd.stabletime), cmd.calcontrol,
			    cmd.externalsleepclk);

		sp->sp_error = le16_to_cpu(cmd.error);
		sp->sp_offset = le16_to_cpu(cmd.offset);
		sp->sp_stabletime = le16_to_cpu(cmd.stabletime);
		sp->sp_calcontrol = cmd.calcontrol;
		sp->sp_extsleepclk = cmd.externalsleepclk;
		sp->sp_reserved = le16_to_cpu(cmd.reserved);
	}
    #endif
    
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return 0;
}

#if ORIG
static int cabrio_wait_for_ds_awake(struct cabrio_private *priv)
{
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else    
	if (priv->is_deep_sleep) {
		if (!wait_event_interruptible_timeout(priv->ds_awake_q,
					!priv->is_deep_sleep, (10 * HZ))) {
			netdev_err(priv->dev, "ds_awake_q: timer expired\n");
			ret = -1;
		}
	}
    #endif // TODO
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}
#endif //ORIG


int cabrio_set_deep_sleep(struct cabrio_private *priv, int deep_sleep)
{
	int ret =  0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	if (deep_sleep) {
		if (priv->is_deep_sleep != 1) {
			cabrio_dbg_cmd("deep sleep: sleep\n");
			BUG_ON(!priv->enter_deep_sleep);
			ret = priv->enter_deep_sleep(priv);
			if (!ret) {
				netif_stop_queue(priv->dev);
				netif_carrier_off(priv->dev);
			}
		} else {
			netdev_err(priv->dev, "deep sleep: already enabled\n");
		}
	} else {
		if (priv->is_deep_sleep) {
			cabrio_dbg_cmd("deep sleep: wakeup\n");
			BUG_ON(!priv->exit_deep_sleep);
			ret = priv->exit_deep_sleep(priv);
			if (!ret) {
				ret = cabrio_wait_for_ds_awake(priv);
				if (ret)
					netdev_err(priv->dev,
						   "deep sleep: wakeup failed\n");
			}
		}
	}
    #endif // TODO
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}


#if ORIG
static int cabrio_ret_host_sleep_activate(struct cabrio_private *priv,
		unsigned long dummy,
		HDR_HostCmd *cmd)
{
	cabrio_dbg_enter(CABRIO_DBG_FW);
	priv->is_host_sleep_activated = 1;
	wake_up_interruptible(&priv->host_sleep_q);
	cabrio_dbg_leave(CABRIO_DBG_FW);
	return 0;
}
#endif // ORIG


int cabrio_set_host_sleep(struct cabrio_private *priv, int host_sleep)
{
	//HDR_HostCmd cmd;
	int ret = 0;
	//uint32_t criteria = EHS_REMOVE_WAKEUP;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	if (host_sleep) {
		if (priv->is_host_sleep_activated != 1) {
			memset(&cmd, 0, sizeof(cmd));
			ret = cabrio_host_sleep_cfg(priv, priv->wol_criteria,
					(struct wol_config *)NULL);
			if (ret) {
				netdev_info(priv->dev,
					    "Host sleep configuration failed: %d\n",
					    ret);
				return ret;
			}
			if (priv->psstate == PS_STATE_FULL_POWER) {
				ret = __cabrio_cmd(priv,
						CMD_802_11_HOST_SLEEP_ACTIVATE,
						&cmd,
						sizeof(cmd),
						cabrio_ret_host_sleep_activate, 0);
				if (ret)
					netdev_info(priv->dev,
						    "HOST_SLEEP_ACTIVATE failed: %d\n",
						    ret);
			}

			if (!wait_event_interruptible_timeout(
						priv->host_sleep_q,
						priv->is_host_sleep_activated,
						(10 * HZ))) {
				netdev_err(priv->dev,
					   "host_sleep_q: timer expired\n");
				ret = -1;
			}
		} else {
			netdev_err(priv->dev, "host sleep: already enabled\n");
		}
	} else {
		if (priv->is_host_sleep_activated)
			ret = cabrio_host_sleep_cfg(priv, criteria,
					(struct wol_config *)NULL);
	}
    #endif // TODO
	return ret;
}


#if ORIG
/**
 *  cabrio_set_snmp_mib - Set an SNMP MIB value
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @oid:	The OID to set in the firmware
 *  @val:	Value to set the OID to
 *
 *  returns: 	   	0 on success, error on failure
 */
int cabrio_set_snmp_mib(struct cabrio_private *priv, u32 oid, u16 val)
{
	//struct cmd_ds_802_11_snmp_mib cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	memset(&cmd, 0, sizeof (cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.oid = cpu_to_le16((u16) oid);

	switch (oid) {
	case SNMP_MIB_OID_BSS_TYPE:
		cmd.bufsize = cpu_to_le16(sizeof(u8));
		cmd.value[0] = val;
		break;
	case SNMP_MIB_OID_11D_ENABLE:
	case SNMP_MIB_OID_FRAG_THRESHOLD:
	case SNMP_MIB_OID_RTS_THRESHOLD:
	case SNMP_MIB_OID_SHORT_RETRY_LIMIT:
	case SNMP_MIB_OID_LONG_RETRY_LIMIT:
		cmd.bufsize = cpu_to_le16(sizeof(u16));
		*((__le16 *)(&cmd.value)) = cpu_to_le16(val);
		break;
	default:
		cabrio_dbg_cmd("SNMP_CMD: (set) unhandled OID 0x%x\n", oid);
		ret = -EINVAL;
		goto out;
	}

	cabrio_dbg_cmd("SNMP_CMD: (set) oid 0x%x, oid size 0x%x, value 0x%x\n",
		    le16_to_cpu(cmd.oid), le16_to_cpu(cmd.bufsize), val);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_SNMP_MIB, &cmd);

out:
    #endif // TODO
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_get_snmp_mib - Get an SNMP MIB value
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @oid:	The OID to retrieve from the firmware
 *  @out_val:	Location for the returned value
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_get_snmp_mib(struct cabrio_private *priv, u32 oid, u16 *out_val)
{
	//struct cmd_ds_802_11_snmp_mib cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

    #if 1
    CABRIO_TODO(__func__);
    #else
	memset(&cmd, 0, sizeof (cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_GET);
	cmd.oid = cpu_to_le16(oid);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_SNMP_MIB, &cmd);
	if (ret)
		goto out;

	switch (le16_to_cpu(cmd.bufsize)) {
	case sizeof(u8):
		*out_val = cmd.value[0];
		break;
	case sizeof(u16):
		*out_val = le16_to_cpu(*((__le16 *)(&cmd.value)));
		break;
	default:
		cabrio_dbg_cmd("SNMP_CMD: (get) unhandled OID 0x%x size %d\n",
		            oid, le16_to_cpu(cmd.bufsize));
		break;
	}

out:
    #endif // TODO
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}
#endif // ORIG

/**
 *  cabrio_get_tx_power - Get the min, max, and current TX power
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @curlevel:	Current power level in dBm
 *  @minlevel:	Minimum supported power level in dBm (optional)
 *  @maxlevel:	Maximum supported power level in dBm (optional)
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_get_tx_power(struct cabrio_private *priv, s16 *curlevel, s16 *minlevel,
		     s16 *maxlevel)
{
	//struct cmd_ds_802_11_rf_tx_power cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_GET);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_RF_TX_POWER, &cmd);
	if (ret == 0) {
		*curlevel = le16_to_cpu(cmd.curlevel);
		if (minlevel)
			*minlevel = cmd.minlevel;
		if (maxlevel)
			*maxlevel = cmd.maxlevel;
	}
    #endif // TODO
	cabrio_dbg_leave(CABRIO_DBG_CMD);
	return ret;
}

/**
 *  cabrio_set_tx_power - Set the TX power
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @dbm:	The desired power level in dBm
 *
 *  returns: 	   	0 on success, error on failure
 */
int cabrio_set_tx_power(struct cabrio_private *priv, s16 dbm)
{
	//struct cmd_ds_802_11_rf_tx_power cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.curlevel = cpu_to_le16(dbm);

	cabrio_dbg_cmd("SET_RF_TX_POWER: %d dBm\n", dbm);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_RF_TX_POWER, &cmd);
    #endif // TODO
	cabrio_dbg_leave(CABRIO_DBG_CMD);
	return ret;
}

/**
 *  cabrio_set_monitor_mode - Enable or disable monitor mode
 *  (only implemented on OLPC usb8388 FW)
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @enable:	1 to enable monitor mode, 0 to disable
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_set_monitor_mode(struct cabrio_private *priv, int enable)
{
	//struct cmd_ds_802_11_monitor_mode cmd;
	int ret = -1;

    cabrio_dbg_enter(CABRIO_DBG_CMD);
#if 1
    CABRIO_TODO(__func__);
#else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	if (enable)
		cmd.mode = cpu_to_le16(0x1);

	cabrio_dbg_cmd("SET_MONITOR_MODE: %d\n", enable);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_MONITOR_MODE, &cmd);
	if (ret == 0) {
		priv->dev->type = enable ? ARPHRD_IEEE80211_RADIOTAP :
						ARPHRD_ETHER;
	}
    #endif
	cabrio_dbg_leave(CABRIO_DBG_CMD);
	return ret;
}

/**
 *  cabrio_get_channel - Get the radio channel
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	The channel on success, error on failure
 */
static int cabrio_get_channel(struct cabrio_private *priv)
{
	//struct cmd_ds_802_11_rf_channel cmd;
	int                          ret = -1;
	HDR_HostCmd                  cmd;
	u8			                 event_buf[sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result)];
	HDR_HostEvent               *event = (HDR_HostEvent *)event_buf;
	struct resp_evt_result      *resp = (struct resp_evt_result *)event->dat;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

    #if 1
	memset(&cmd, 0, sizeof(cmd));
    cmd.c_type = HOST_CMD;
    cmd.h_cmd = SSV_HOST_CMD_GET_CHANNEL;
    cmd.len = HOST_CMD_HDR_LEN;

    event->len = sizeof(event_buf);

	ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_GET_CHANNEL, &cmd, event);
	if (ret)
		goto out;

	if (resp->result != CMD_OK) {
		cabrio_dbg_cmd("Get channel failed with error %d\n", resp->result);
		goto out;
	}

	ret = resp->u.dat[0];
    #else
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_OPT_802_11_RF_CHANNEL_GET);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_RF_CHANNEL, &cmd);
	if (ret)
		goto out;

	ret = le16_to_cpu(cmd.channel);
    #endif // TODO
    
	cabrio_dbg_cmd("current radio channel is %d\n", ret);
out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

int cabrio_update_channel(struct cabrio_private *priv)
{
	int ret;

	/* the channel in f/w could be out of sync; get the current channel */
	cabrio_dbg_enter(CABRIO_DBG_ASSOC);

	ret = cabrio_get_channel(priv);
	if (ret > 0) {
		priv->channel = ret;
		ret = 0;
	}
	cabrio_dbg_leave_args(CABRIO_DBG_ASSOC, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_set_channel - Set the radio channel
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *  @channel:	The desired channel, or 0 to clear a locked channel
 *
 *  returns:	0 on success, error on failure
 */
int cabrio_set_channel(struct cabrio_private *priv, u8 channel)
{
	HDR_HostCmd     cmd;
	u8			    event_buf[sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result)];
	HDR_HostEvent  *event = (HDR_HostEvent *)event_buf;
	struct resp_evt_result *resp = (struct resp_evt_result *)event->dat;
#ifdef DEBUG
	u8 old_channel = priv->channel;
#endif
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	memset(&cmd, 0, sizeof(cmd));
    cmd.c_type = HOST_CMD;
    cmd.h_cmd = SSV_HOST_CMD_CAL;
    cmd.len = HOST_CMD_HDR_LEN + sizeof(u32);
    cmd.dat32[0] = (u32)channel;

    event->len = sizeof(event_buf);

	ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_CAL, &cmd, event);
	if (ret)
		goto out;

	if (resp->result != CMD_OK) {
		cabrio_dbg_cmd("Channel switch failed with error %d\n", resp->result);
		goto out;
	}
	priv->channel = (uint8_t)channel;
#ifdef DEBUG
	cabrio_dbg_cmd("channel switch from %d to %d\n", old_channel, priv->channel);
#endif
out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

#if 0
int cabrio_set_channel_orig(struct cabrio_private *priv, u8 channel)
{
	struct cmd_ds_802_11_rf_channel cmd;
#ifdef DEBUG
	u8 old_channel = priv->channel;
#endif
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_OPT_802_11_RF_CHANNEL_SET);
	cmd.channel = cpu_to_le16(channel);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_RF_CHANNEL, &cmd);
	if (ret)
		goto out;

	priv->channel = (uint8_t) le16_to_cpu(cmd.channel);
	cabrio_dbg_cmd("channel switch from %d to %d\n", old_channel,
		priv->channel);

out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}
#endif // ORIG
/**
 * cabrio_get_rssi - Get current RSSI and noise floor
 *
 * @priv:	A pointer to &struct cabrio_private structure
 * @rssi:	On successful return, signal level in mBm
 * @nf:		On successful return, Noise floor
 *
 * returns:	The channel on success, error on failure
 */
int cabrio_get_rssi(struct cabrio_private *priv, s8 *rssi, s8 *nf)
{
	struct cmd_ds_802_11_rssi cmd;
	int ret = -1;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	BUG_ON(rssi == NULL);
	BUG_ON(nf == NULL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	/* Average SNR over last 8 beacons */
	cmd.n_or_snr = cpu_to_le16(8);

    #if 1
    CABRIO_TODO(__func__);
    #else
	ret = cabrio_cmd_with_response(priv, CMD_802_11_RSSI, &cmd);
	if (ret == 0) {
		*nf = CAL_NF(le16_to_cpu(cmd.nf));
		*rssi = CAL_RSSI(le16_to_cpu(cmd.n_or_snr), le16_to_cpu(cmd.nf));
	}
    #endif
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_set_11d_domain_info - Send regulatory and 802.11d domain information
 *  to the firmware
 *
 *  @priv:	pointer to &struct cabrio_private
 *  @request:	cfg80211 regulatory request structure
 *  @bands:	the device's supported bands and channels
 *
 *  returns:	0 on success, error code on failure
*/
int cabrio_set_11d_domain_info(struct cabrio_private *priv,
			    struct regulatory_request *request,
			    struct ieee80211_supported_band **bands)
{
	struct cmd_ds_802_11d_domain_info cmd;
	struct cabrio_ie_domain_param_set *domain = &cmd.domain;
	struct ieee80211_country_ie_triplet *t;
	enum ieee80211_band band;
	struct ieee80211_channel *ch;
	u8 num_triplet = 0;
	u8 num_parsed_chan = 0;
	u8 first_channel = 0, next_chan = 0, max_pwr = 0;
	u8 i, flag = 0;
	size_t triplet_size;
	int ret = -1;

	cabrio_dbg_enter(CABRIO_DBG_11D);

	memset(&cmd, 0, sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);

	cabrio_dbg_11d("Setting country code '%c%c'\n",
		    request->alpha2[0], request->alpha2[1]);

	domain->header.type = cpu_to_le16(SSS_TYPE_DOMAIN);

	/* Set country code */
	domain->country_code[0] = request->alpha2[0];
	domain->country_code[1] = request->alpha2[1];
	domain->country_code[2] = ' ';

	/* Now set up the channel triplets; firmware is somewhat picky here
	 * and doesn't validate channel numbers and spans; hence it would
	 * interpret a triplet of (36, 4, 20) as channels 36, 37, 38, 39.  Since
	 * the last 3 aren't valid channels, the driver is responsible for
	 * splitting that up into 4 triplet pairs of (36, 1, 20) + (40, 1, 20)
	 * etc.
	 */
	for (band = 0;
	     (band < IEEE80211_NUM_BANDS) && (num_triplet < MAX_11D_TRIPLETS);
	     band++) {

		if (!bands[band])
			continue;

		for (i = 0;
		     (i < bands[band]->n_channels) && (num_triplet < MAX_11D_TRIPLETS);
		     i++) {
			ch = &bands[band]->channels[i];
			if (ch->flags & IEEE80211_CHAN_DISABLED)
				continue;

			if (!flag) {
				flag = 1;
				next_chan = first_channel = (u32) ch->hw_value;
				max_pwr = ch->max_power;
				num_parsed_chan = 1;
				continue;
			}

			if ((ch->hw_value == next_chan + 1) &&
					(ch->max_power == max_pwr)) {
				/* Consolidate adjacent channels */
				next_chan++;
				num_parsed_chan++;
			} else {
				/* Add this triplet */
				cabrio_dbg_11d("11D triplet (%d, %d, %d)\n",
					first_channel, num_parsed_chan,
					max_pwr);
				t = &domain->triplet[num_triplet];
				t->chans.first_channel = first_channel;
				t->chans.num_channels = num_parsed_chan;
				t->chans.max_power = max_pwr;
				num_triplet++;
				flag = 0;
			}
		}

		if (flag) {
			/* Add last triplet */
			cabrio_dbg_11d("11D triplet (%d, %d, %d)\n", first_channel,
				num_parsed_chan, max_pwr);
			t = &domain->triplet[num_triplet];
			t->chans.first_channel = first_channel;
			t->chans.num_channels = num_parsed_chan;
			t->chans.max_power = max_pwr;
			num_triplet++;
		}
	}

	cabrio_dbg_11d("# triplets %d\n", num_triplet);

	/* Set command header sizes */
	triplet_size = num_triplet * sizeof(struct ieee80211_country_ie_triplet);
	domain->header.len = cpu_to_le16(sizeof(domain->country_code) +
					triplet_size);

	cabrio_dbg_hex(CABRIO_DBG_11D, "802.11D domain param set",
			(u8 *) &cmd.domain.country_code,
			le16_to_cpu(domain->header.len));

	cmd.hdr.size = cpu_to_le16(sizeof(cmd.hdr) +
				   sizeof(cmd.action) +
				   sizeof(cmd.domain.header) +
				   sizeof(cmd.domain.country_code) +
				   triplet_size);

    #if 1
    CABRIO_TODO(__func__);
    ret = 0;
    #else
	ret = cabrio_cmd_with_response(priv, CMD_802_11D_DOMAIN_INFO, &cmd);
    #endif
	cabrio_dbg_leave_args(CABRIO_DBG_11D, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_get_reg - Read a MAC, Baseband, or RF register
 *
 *  @priv:	pointer to &struct cabrio_private
 *  @reg:	register command, one of CMD_MAC_REG_ACCESS,
 *		CMD_BBP_REG_ACCESS, or CMD_RF_REG_ACCESS
 *  @offset:	byte offset of the register to get
 *  @value:	on success, the value of the register at 'offset'
 *
 *  returns:	0 on success, error code on failure
*/
int cabrio_get_reg(struct cabrio_private *priv, u16 reg, u16 offset, u32 *value)
{
	struct cmd_ds_reg_access cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	BUG_ON(value == NULL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_GET);
	cmd.offset = cpu_to_le16(offset);

	if (reg != CMD_MAC_REG_ACCESS &&
	    reg != CMD_BBP_REG_ACCESS &&
	    reg != CMD_RF_REG_ACCESS) {
		ret = -EINVAL;
		goto out;
	}

    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
	ret = cabrio_cmd_with_response(priv, reg, &cmd);
    #endif
	if (!ret) {
		if (reg == CMD_BBP_REG_ACCESS || reg == CMD_RF_REG_ACCESS)
			*value = cmd.value.bbp_rf;
		else if (reg == CMD_MAC_REG_ACCESS)
			*value = le32_to_cpu(cmd.value.mac);
	}

out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_set_reg - Write a MAC, Baseband, or RF register
 *
 *  @priv:	pointer to &struct cabrio_private
 *  @reg:	register command, one of CMD_MAC_REG_ACCESS,
 *		CMD_BBP_REG_ACCESS, or CMD_RF_REG_ACCESS
 *  @offset:	byte offset of the register to set
 *  @value:	the value to write to the register at 'offset'
 *
 *  returns:	0 on success, error code on failure
*/
int cabrio_set_reg(struct cabrio_private *priv, u16 reg, u16 offset, u32 value)
{
	struct cmd_ds_reg_access cmd;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.offset = cpu_to_le16(offset);

	if (reg == CMD_BBP_REG_ACCESS || reg == CMD_RF_REG_ACCESS)
		cmd.value.bbp_rf = (u8) (value & 0xFF);
	else if (reg == CMD_MAC_REG_ACCESS)
		cmd.value.mac = cpu_to_le32(value);
	else {
		ret = -EINVAL;
		goto out;
	}
    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
    ret = cabrio_cmd_with_response(priv, reg, &cmd);
    #endif
out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

static void cabrio_queue_cmd(struct cabrio_private *priv,
			                 struct host_cmd_node *cmdnode)
{
	unsigned long flags;
	int addtail = 1;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	if (!cmdnode) {
		cabrio_dbg_host("QUEUE_CMD: cmdnode is NULL\n");
		goto done;
	}
	if (!cmdnode->cmdbuf->len) {
		cabrio_dbg_host("DNLD_CMD: cmd size is zero\n");
		goto done;
	}
	cmdnode->result = 0;

    #if 0
	/* Exit_PS command needs to be queued in the header always. */
	if (le16_to_cpu(cmdnode->cmdbuf->command) == CMD_802_11_PS_MODE) {
		struct cmd_ds_802_11_ps_mode *psm = (void *) &cmdnode->cmdbuf;

		if (psm->action == cpu_to_le16(PS_MODE_ACTION_EXIT_PS)) {
			if (priv->psstate != PS_STATE_FULL_POWER)
				addtail = 0;
		}
	}

	if (le16_to_cpu(cmdnode->cmdbuf->command) == CMD_802_11_WAKEUP_CONFIRM)
		addtail = 0;
    #endif
	spin_lock_irqsave(&priv->driver_lock, flags);

	if (addtail)
		list_add_tail(&cmdnode->list, &priv->cmdpendingq);
	else
		list_add(&cmdnode->list, &priv->cmdpendingq);

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	cabrio_dbg_host("QUEUE_CMD: inserted command 0x%04x into cmdpendingq\n",
                    le16_to_cpu(cmdnode->cmdbuf->h_cmd));

done:
	cabrio_dbg_leave(CABRIO_DBG_HOST);
}

static void cabrio_submit_command(struct cabrio_private *priv,
			       struct host_cmd_node *cmdnode)
{
	unsigned long flags;
	HDR_HostCmd *cmd;
	uint16_t cmdsize;
	uint16_t command;
	int timeo = 3 * HZ;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	cmd = cmdnode->cmdbuf;

	spin_lock_irqsave(&priv->driver_lock, flags);
	priv->seqnum++;
	cmd->cmd_seq_no = cpu_to_le16(priv->seqnum);
	priv->cur_cmd = cmdnode;
	spin_unlock_irqrestore(&priv->driver_lock, flags);

	cmdsize = le16_to_cpu(cmd->len);
	command = le16_to_cpu(cmd->h_cmd);

	/* These commands take longer */
	if (command == SSV_HOST_CMD_SCAN || command == SSV_HOST_CMD_JOIN)
		timeo = 5 * HZ;

	cabrio_dbg_cmd("DNLD_CMD: command 0x%04x, seq %d, size %d\n",
                   command, le16_to_cpu(cmd->cmd_seq_no), cmdsize);
	cabrio_dbg_hex(CABRIO_DBG_CMD, "DNLD_CMD", (void *) cmdnode->cmdbuf, cmdsize);

	ret = priv->hw_host_to_card(priv, CABRIO_MS_CMD, (u8 *) cmd, cmdsize);

	if (ret < 0) {
		netdev_info(priv->dev, "DNLD_CMD: hw_host_to_card failed: %d\n",
			    ret);
		/* Let the timer kick in and retry, and potentially reset
		   the whole thing if the condition persists */
		timeo = HZ/4;
	}
    #if 0
	if (command == CMD_802_11_DEEP_SLEEP) {
		if (priv->is_auto_deep_sleep_enabled) {
			priv->wakeup_dev_required = 1;
			priv->dnld_sent = 0;
		}
		priv->is_deep_sleep = 1;
		cabrio_complete_command(priv, cmdnode, 0);
	} else {
	#endif 

	#if 0
    // TODO: cabrio_complete_command should be called at receiption of response message.
	cabrio_complete_command(priv, cmdnode, 0);
	#else
	{
		/* Setup the timer after transmit command */
		mod_timer(&priv->command_timer, jiffies + timeo);
	}
	#endif
	cabrio_dbg_leave(CABRIO_DBG_HOST);
}

#if 0
static void cabrio_queue_cmd_orig(struct cabrio_private *priv,
			  struct host_cmd_node *cmdnode)
{
	unsigned long flags;
	int addtail = 1;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	if (!cmdnode) {
		cabrio_dbg_host("QUEUE_CMD: cmdnode is NULL\n");
		goto done;
	}
	if (!cmdnode->cmdbuf->size) {
		cabrio_dbg_host("DNLD_CMD: cmd size is zero\n");
		goto done;
	}
	cmdnode->result = 0;

	/* Exit_PS command needs to be queued in the header always. */
	if (le16_to_cpu(cmdnode->cmdbuf->command) == CMD_802_11_PS_MODE) {
		struct cmd_ds_802_11_ps_mode *psm = (void *) &cmdnode->cmdbuf;

		if (psm->action == cpu_to_le16(PS_MODE_ACTION_EXIT_PS)) {
			if (priv->psstate != PS_STATE_FULL_POWER)
				addtail = 0;
		}
	}

	if (le16_to_cpu(cmdnode->cmdbuf->command) == CMD_802_11_WAKEUP_CONFIRM)
		addtail = 0;

	spin_lock_irqsave(&priv->driver_lock, flags);

	if (addtail)
		list_add_tail(&cmdnode->list, &priv->cmdpendingq);
	else
		list_add(&cmdnode->list, &priv->cmdpendingq);

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	cabrio_dbg_host("QUEUE_CMD: inserted command 0x%04x into cmdpendingq\n",
		     le16_to_cpu(cmdnode->cmdbuf->command));

done:
	cabrio_dbg_leave(CABRIO_DBG_HOST);
}

static void cabrio_submit_command_orig(struct cabrio_private *priv,
			       struct host_cmd_node *cmdnode)
{
	unsigned long flags;
	HDR_HostCmd *cmd;
	uint16_t cmdsize;
	uint16_t command;
	int timeo = 3 * HZ;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	cmd = cmdnode->cmdbuf;

	spin_lock_irqsave(&priv->driver_lock, flags);
	priv->seqnum++;
	cmd->seqnum = cpu_to_le16(priv->seqnum);
	priv->cur_cmd = cmdnode;
	spin_unlock_irqrestore(&priv->driver_lock, flags);

	cmdsize = le16_to_cpu(cmd->size);
	command = le16_to_cpu(cmd->command);

	/* These commands take longer */
	if (command == CMD_802_11_SCAN || command == CMD_802_11_ASSOCIATE)
		timeo = 5 * HZ;

	cabrio_dbg_cmd("DNLD_CMD: command 0x%04x, seq %d, size %d\n",
		     command, le16_to_cpu(cmd->seqnum), cmdsize);
	cabrio_dbg_hex(CABRIO_DBG_CMD, "DNLD_CMD", (void *) cmdnode->cmdbuf, cmdsize);

	ret = priv->hw_host_to_card(priv, CABRIO_MS_CMD, (u8 *) cmd, cmdsize);

	if (ret) {
		netdev_info(priv->dev, "DNLD_CMD: hw_host_to_card failed: %d\n",
			    ret);
		/* Let the timer kick in and retry, and potentially reset
		   the whole thing if the condition persists */
		timeo = HZ/4;
	}

	if (command == CMD_802_11_DEEP_SLEEP) {
		if (priv->is_auto_deep_sleep_enabled) {
			priv->wakeup_dev_required = 1;
			priv->dnld_sent = 0;
		}
		priv->is_deep_sleep = 1;
		cabrio_complete_command(priv, cmdnode, 0);
	} else {
		/* Setup the timer after transmit command */
		mod_timer(&priv->command_timer, jiffies + timeo);
	}

	cabrio_dbg_leave(CABRIO_DBG_HOST);
}
#endif

/*
 *  This function inserts command node to cmdfreeq
 *  after cleans it. Requires priv->driver_lock held.
 */
static void __cabrio_cleanup_and_insert_cmd(struct cabrio_private *priv,
					 struct host_cmd_node *cmdnode)
{
	cabrio_dbg_enter(CABRIO_DBG_HOST);

	if (!cmdnode)
		goto out;

	cmdnode->callback = NULL;
	cmdnode->callback_arg = 0;

	memset(cmdnode->cmdbuf, 0, CABRIO_CMD_BUFFER_SIZE);

	list_add_tail(&cmdnode->list, &priv->cmdfreeq);
 out:
	cabrio_dbg_leave(CABRIO_DBG_HOST);
}

static void cabrio_cleanup_and_insert_cmd(struct cabrio_private *priv,
	struct host_cmd_node *ptempcmd)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->driver_lock, flags);
	__cabrio_cleanup_and_insert_cmd(priv, ptempcmd);
	spin_unlock_irqrestore(&priv->driver_lock, flags);
}

void __cabrio_complete_command(struct cabrio_private *priv, struct host_cmd_node *cmd,
			    int result)
{
	/*
	 * Normally, commands are removed from cmdpendingq before being
	 * submitted. However, we can arrive here on alternative codepaths
	 * where the command is still pending. Make sure the command really
	 * isn't part of a list at this point.
	 */
	list_del_init(&cmd->list);

	cmd->result = result;
	cmd->cmdwaitqwoken = 1;
	wake_up(&cmd->cmdwait_q);

	if (!cmd->callback || cmd->callback == cabrio_cmd_async_callback)
	{
		// cabrio_dbg_cmd("Clean cmd.\n");
		__cabrio_cleanup_and_insert_cmd(priv, cmd);
	}
	priv->cur_cmd = NULL;
	wake_up(&priv->waitq);
}

void cabrio_complete_command(struct cabrio_private *priv, struct host_cmd_node *cmd,
			  int result)
{
	unsigned long flags;
	spin_lock_irqsave(&priv->driver_lock, flags);
	__cabrio_complete_command(priv, cmd, result);
	spin_unlock_irqrestore(&priv->driver_lock, flags);
}

int cabrio_set_radio(struct cabrio_private *priv, u8 preamble, u8 radio_on)
{
	struct cmd_ds_802_11_radio_control cmd;
	int ret = -EINVAL;

	cabrio_dbg_enter(CABRIO_DBG_CMD);

	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);

	/* Only v8 and below support setting the preamble */
	if (priv->fwrelease < 0x09000000) {
		switch (preamble) {
		case RADIO_PREAMBLE_SHORT:
		case RADIO_PREAMBLE_AUTO:
		case RADIO_PREAMBLE_LONG:
			cmd.control = cpu_to_le16(preamble);
			break;
		default:
			goto out;
		}
	}

	if (radio_on)
		cmd.control |= cpu_to_le16(0x1);
	else {
		cmd.control &= cpu_to_le16(~0x1);
		priv->txpower_cur = 0;
	}

	cabrio_dbg_cmd("RADIO_CONTROL: radio %s, preamble %d\n",
		    radio_on ? "ON" : "OFF", preamble);

	priv->radio_on = radio_on;

    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
	ret = cabrio_cmd_with_response(priv, CMD_802_11_RADIO_CONTROL, &cmd);
    #endif
out:
	cabrio_dbg_leave_args(CABRIO_DBG_CMD, "ret %d", ret);
	return ret;
}

void cabrio_set_mac_control(struct cabrio_private *priv)
{
	//struct cmd_ds_mac_control cmd;

	cabrio_dbg_enter(CABRIO_DBG_CMD);
    #if 1
    CABRIO_TODO(__func__);
    #else
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(priv->mac_control);
	cmd.reserved = 0;

	cabrio_cmd_async(priv, CMD_MAC_CONTROL, &cmd.hdr, sizeof(cmd));
    #endif // TODO
	cabrio_dbg_leave(CABRIO_DBG_CMD);
}

/**
 *  cabrio_allocate_cmd_buffer - allocates the command buffer and links
 *  it to command free queue
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	0 for success or -1 on error
 */
int cabrio_allocate_cmd_buffer(struct cabrio_private *priv)
{
	int ret = 0;
	u32 bufsize;
	u32 i;
	struct host_cmd_node *cmdarray;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	/* Allocate and initialize the command array */
	bufsize = sizeof(struct host_cmd_node) * CABRIO_NUM_CMD_BUFFERS;
	if (!(cmdarray = kzalloc(bufsize, GFP_KERNEL))) {
		cabrio_dbg_host("ALLOC_CMD_BUF: tempcmd_array is NULL\n");
		ret = -1;
		goto done;
	}
	priv->cmd_array = cmdarray;

	/* Allocate and initialize each command buffer in the command array */
	for (i = 0; i < CABRIO_NUM_CMD_BUFFERS; i++) {
		cmdarray[i].cmdbuf = kzalloc(CABRIO_CMD_BUFFER_SIZE, GFP_KERNEL);
		if (!cmdarray[i].cmdbuf) {
			cabrio_dbg_host("ALLOC_CMD_BUF: ptempvirtualaddr is NULL\n");
			ret = -1;
			goto done;
		}
	}

	for (i = 0; i < CABRIO_NUM_CMD_BUFFERS; i++) {
		init_waitqueue_head(&cmdarray[i].cmdwait_q);
		cabrio_cleanup_and_insert_cmd(priv, &cmdarray[i]);
	}
	ret = 0;

done:
	cabrio_dbg_leave_args(CABRIO_DBG_HOST, "ret %d", ret);
	return ret;
}

/**
 *  cabrio_free_cmd_buffer - free the command buffer
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	0 for success
 */
int cabrio_free_cmd_buffer(struct cabrio_private *priv)
{
	struct host_cmd_node *cmdarray;
	unsigned int i;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	/* need to check if cmd array is allocated or not */
	if (priv->cmd_array == NULL) {
		cabrio_dbg_host("FREE_CMD_BUF: cmd_array is NULL\n");
		goto done;
	}

	cmdarray = priv->cmd_array;

	/* Release shared memory buffers */
	for (i = 0; i < CABRIO_NUM_CMD_BUFFERS; i++) {
		if (cmdarray[i].cmdbuf) {
			kfree(cmdarray[i].cmdbuf);
			cmdarray[i].cmdbuf = NULL;
		}
	}

	/* Release host_cmd_node */
	if (priv->cmd_array) {
		kfree(priv->cmd_array);
		priv->cmd_array = NULL;
	}

done:
	cabrio_dbg_leave(CABRIO_DBG_HOST);
	return 0;
}

/**
 *  cabrio_get_free_cmd_node - gets a free command node if available in
 *  command free queue
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	A pointer to &host_cmd_node structure on success
 *		or %NULL on error
 */
static struct host_cmd_node *cabrio_get_free_cmd_node(struct cabrio_private *priv)
{
	struct host_cmd_node *tempnode;
	unsigned long flags;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	if (!priv)
		return NULL;

	spin_lock_irqsave(&priv->driver_lock, flags);

	if (!list_empty(&priv->cmdfreeq)) {
		tempnode = list_first_entry(&priv->cmdfreeq,
					    struct host_cmd_node, list);
		list_del_init(&tempnode->list);
	} else {
		cabrio_dbg_host("GET_CMD_NODE: host_cmd_node is not available\n");
		tempnode = NULL;
	}

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	cabrio_dbg_leave(CABRIO_DBG_HOST);
	return tempnode;
}

/**
 *  cabrio_execute_next_command - execute next command in command
 *  pending queue. Will put firmware back to PS mode if applicable.
 *
 *  @priv:	A pointer to &struct cabrio_private structure
 *
 *  returns:	0 on success or -1 on error
 */
int cabrio_execute_next_command(struct cabrio_private *priv)
{
	struct host_cmd_node *cmdnode = NULL;
	HDR_HostCmd *cmd;
	unsigned long flags;
	int ret = 0;

	/* Debug group is CABRIO_DBG_THREAD and not CABRIO_DBG_HOST, because the
	 * only caller to us is cabrio_thread() and we get even when a
	 * data packet is received */
	cabrio_dbg_enter(CABRIO_DBG_THREAD);

	spin_lock_irqsave(&priv->driver_lock, flags);

	if (priv->cur_cmd) {
		netdev_alert(priv->dev,
			     "EXEC_NEXT_CMD: already processing command!\n");
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		ret = -1;
		goto done;
	}

	if (!list_empty(&priv->cmdpendingq)) {
		cmdnode = list_first_entry(&priv->cmdpendingq,
					   struct host_cmd_node, list);
	}

	spin_unlock_irqrestore(&priv->driver_lock, flags);

	if (cmdnode) {
		cmd = cmdnode->cmdbuf;
        // CABRIO TODO
        #if 0
		if (is_command_allowed_in_ps(le16_to_cpu(cmd->command))) {
			if ((priv->psstate == PS_STATE_SLEEP) ||
			    (priv->psstate == PS_STATE_PRE_SLEEP)) {
				cabrio_dbg_host(
				       "EXEC_NEXT_CMD: cannot send cmd 0x%04x in psstate %d\n",
				       le16_to_cpu(cmd->command),
				       priv->psstate);
				ret = -1;
				goto done;
			}
			cabrio_dbg_host("EXEC_NEXT_CMD: OK to send command "
				     "0x%04x in psstate %d\n",
				     le16_to_cpu(cmd->command), priv->psstate);
		} else if (priv->psstate != PS_STATE_FULL_POWER) {
			/*
			 * 1. Non-PS command:
			 * Queue it. set needtowakeup to TRUE if current state
			 * is SLEEP, otherwise call send EXIT_PS.
			 * 2. PS command but not EXIT_PS:
			 * Ignore it.
			 * 3. PS command EXIT_PS:
			 * Set needtowakeup to TRUE if current state is SLEEP,
			 * otherwise send this command down to firmware
			 * immediately.
			 */
			if (cmd->command != cpu_to_le16(CMD_802_11_PS_MODE)) {
				/*  Prepare to send Exit PS,
				 *  this non PS command will be sent later */
				if ((priv->psstate == PS_STATE_SLEEP)
				    || (priv->psstate == PS_STATE_PRE_SLEEP)
				    ) {
					/* w/ new scheme, it will not reach here.
					   since it is blocked in main_thread. */
					priv->needtowakeup = 1;
				} else {
					cabrio_set_ps_mode(priv,
							PS_MODE_ACTION_EXIT_PS,
							false);
				}

				ret = 0;
				goto done;
			} else {
				/*
				 * PS command. Ignore it if it is not Exit_PS.
				 * otherwise send it down immediately.
				 */
				struct cmd_ds_802_11_ps_mode *psm = (void *)&cmd[1];

				cabrio_dbg_host(
				       "EXEC_NEXT_CMD: PS cmd, action 0x%02x\n",
				       psm->action);
				if (psm->action !=
				    cpu_to_le16(PS_MODE_ACTION_EXIT_PS)) {
					cabrio_dbg_host(
					       "EXEC_NEXT_CMD: ignore ENTER_PS cmd\n");
					cabrio_complete_command(priv, cmdnode, 0);

					ret = 0;
					goto done;
				}

				if ((priv->psstate == PS_STATE_SLEEP) ||
				    (priv->psstate == PS_STATE_PRE_SLEEP)) {
					cabrio_dbg_host(
					       "EXEC_NEXT_CMD: ignore EXIT_PS cmd in sleep\n");
					cabrio_complete_command(priv, cmdnode, 0);
					priv->needtowakeup = 1;

					ret = 0;
					goto done;
				}

				cabrio_dbg_host(
				       "EXEC_NEXT_CMD: sending EXIT_PS\n");
			}
		}
        #endif
		spin_lock_irqsave(&priv->driver_lock, flags);
		list_del_init(&cmdnode->list);
		spin_unlock_irqrestore(&priv->driver_lock, flags);
		cabrio_dbg_host("EXEC_NEXT_CMD: sending command 0x%04x\n",
                        le16_to_cpu(cmd->h_cmd));
		cabrio_submit_command(priv, cmdnode);
	} else {
		/*
		 * check if in power save mode, if yes, put the device back
		 * to PS mode
		 */
#ifdef TODO
		/*
		 * This was the old code for libertas+wext. Someone that
		 * understands this beast should re-code it in a sane way.
		 *
		 * I actually don't understand why this is related to WPA
		 * and to connection status, shouldn't powering should be
		 * independ of such things?
		 */
		if ((priv->psmode != CABRIO802_11POWERMODECAM) &&
		    (priv->psstate == PS_STATE_FULL_POWER) &&
		    ((priv->connect_status == CABRIO_CONNECTED) ||
		    cabrio_mesh_connected(priv))) {
			if (priv->secinfo.WPAenabled ||
			    priv->secinfo.WPA2enabled) {
				/* check for valid WPA group keys */
				if (priv->wpa_mcast_key.len ||
				    priv->wpa_unicast_key.len) {
					cabrio_dbg_host(
					       "EXEC_NEXT_CMD: WPA enabled and GTK_SET"
					       " go back to PS_SLEEP");
					cabrio_set_ps_mode(priv,
							PS_MODE_ACTION_ENTER_PS,
							false);
				}
			} else {
				cabrio_dbg_host(
				       "EXEC_NEXT_CMD: cmdpendingq empty, "
				       "go back to PS_SLEEP");
				cabrio_set_ps_mode(priv, PS_MODE_ACTION_ENTER_PS,
						false);
			}
		}
#endif
	}

	ret = 0;
done:
	cabrio_dbg_leave(CABRIO_DBG_THREAD);
	return ret;
}

static void cabrio_send_confirmsleep(struct cabrio_private *priv)
{
	unsigned long flags;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_HOST);
	cabrio_dbg_hex(CABRIO_DBG_HOST, "sleep confirm", (u8 *) &confirm_sleep,
		sizeof(confirm_sleep));

	ret = priv->hw_host_to_card(priv, CABRIO_MS_CMD, (u8 *) &confirm_sleep,
		sizeof(confirm_sleep));
	if (ret < 0) {
		netdev_alert(priv->dev, "confirm_sleep failed\n");
		goto out;
	}

	spin_lock_irqsave(&priv->driver_lock, flags);

	/* We don't get a response on the sleep-confirmation */
	priv->dnld_sent = DNLD_RES_RECEIVED;

	if (priv->is_host_sleep_configured) {
		priv->is_host_sleep_activated = 1;
		wake_up_interruptible(&priv->host_sleep_q);
	}

	/* If nothing to do, go back to sleep (?) */
	if (!kfifo_len(&priv->event_fifo) && !priv->resp_len[priv->resp_idx])
		priv->psstate = PS_STATE_SLEEP;

	spin_unlock_irqrestore(&priv->driver_lock, flags);

out:
	cabrio_dbg_leave(CABRIO_DBG_HOST);
}

/**
 * cabrio_ps_confirm_sleep - checks condition and prepares to
 * send sleep confirm command to firmware if ok
 *
 * @priv:	A pointer to &struct cabrio_private structure
 *
 * returns:	n/a
 */
void cabrio_ps_confirm_sleep(struct cabrio_private *priv)
{
	unsigned long flags =0;
	int allowed = 1;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	spin_lock_irqsave(&priv->driver_lock, flags);
	if (priv->dnld_sent) {
		allowed = 0;
		cabrio_dbg_host("dnld_sent was set\n");
	}

	/* In-progress command? */
	if (priv->cur_cmd) {
		allowed = 0;
		cabrio_dbg_host("cur_cmd was set\n");
	}

	/* Pending events or command responses? */
	if (kfifo_len(&priv->event_fifo) || priv->resp_len[priv->resp_idx]) {
		allowed = 0;
		cabrio_dbg_host("pending events or command responses\n");
	}
	spin_unlock_irqrestore(&priv->driver_lock, flags);

	if (allowed) {
		cabrio_dbg_host("sending cabrio_ps_confirm_sleep\n");
		cabrio_send_confirmsleep(priv);
	} else {
		cabrio_dbg_host("sleep confirm has been delayed\n");
	}

	cabrio_dbg_leave(CABRIO_DBG_HOST);
}


/**
 * cabrio_set_tpc_cfg - Configures the transmission power control functionality
 *
 * @priv:	A pointer to &struct cabrio_private structure
 * @enable:	Transmission power control enable
 * @p0:		Power level when link quality is good (dBm).
 * @p1:		Power level when link quality is fair (dBm).
 * @p2:		Power level when link quality is poor (dBm).
 * @usesnr:	Use Signal to Noise Ratio in TPC
 *
 * returns:	0 on success
 */
int cabrio_set_tpc_cfg(struct cabrio_private *priv, int enable, int8_t p0, int8_t p1,
		int8_t p2, int usesnr)
{
	struct cmd_ds_802_11_tpc_cfg cmd;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.enable = !!enable;
	cmd.usesnr = !!usesnr;
	cmd.P0 = p0;
	cmd.P1 = p1;
	cmd.P2 = p2;

    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
	ret = cabrio_cmd_with_response(priv, CMD_802_11_TPC_CFG, &cmd);
    #endif
	return ret;
}

/**
 * cabrio_set_power_adapt_cfg - Configures the power adaptation settings
 *
 * @priv:	A pointer to &struct cabrio_private structure
 * @enable:	Power adaptation enable
 * @p0:		Power level for 1, 2, 5.5 and 11 Mbps (dBm).
 * @p1:		Power level for 6, 9, 12, 18, 22, 24 and 36 Mbps (dBm).
 * @p2:		Power level for 48 and 54 Mbps (dBm).
 *
 * returns:	0 on Success
 */

int cabrio_set_power_adapt_cfg(struct cabrio_private *priv, int enable, int8_t p0,
		int8_t p1, int8_t p2)
{
	struct cmd_ds_802_11_pa_cfg cmd;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.enable = !!enable;
	cmd.P0 = p0;
	cmd.P1 = p1;
	cmd.P2 = p2;
    #if 1
    CABRIO_TODO(__func__);
    ret = -1;
    #else
	ret = cabrio_cmd_with_response(priv, CMD_802_11_PA_CFG , &cmd);
    #endif
	return ret;
}


struct host_cmd_node *__cabrio_cmd_async(struct cabrio_private *priv,
                                         uint16_t command, const u8 *in_cmd, int in_cmd_size,
                                         int (*callback)(struct cabrio_private *, 
                                                         unsigned long, HDR_HostEvent *),
                                         unsigned long callback_arg)
{
	struct host_cmd_node *cmdnode;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	if (priv->surpriseremoved) {
		cabrio_dbg_host("PREP_CMD: card removed\n");
		cmdnode = ERR_PTR(-ENOENT);
		goto done;
	}

	/* No commands are allowed in Deep Sleep until we toggle the GPIO
	 * to wake up the card and it has signaled that it's ready.
	 */
	if (!priv->is_auto_deep_sleep_enabled) {
		if (priv->is_deep_sleep) {
			cabrio_dbg_cmd("command not allowed in deep sleep\n");
			cmdnode = ERR_PTR(-EBUSY);
			goto done;
		}
	}

	cmdnode = cabrio_get_free_cmd_node(priv);
	if (cmdnode == NULL) {
		cabrio_dbg_host("PREP_CMD: cmdnode is NULL\n");

		/* Wake up main thread to execute next command */
		wake_up(&priv->waitq);
		cmdnode = ERR_PTR(-ENOBUFS);
		goto done;
	}

	cmdnode->callback = callback;
	cmdnode->callback_arg = callback_arg;

	/* Copy the incoming command to the buffer */
	//memcpy(&cmdnode->cmdbuf->dat8[0], in_cmd, in_cmd_size);
	BUG_ON(in_cmd_size > CABRIO_CMD_BUFFER_SIZE);

	memcpy(cmdnode->cmdbuf, in_cmd, in_cmd_size);

	/* Set command, clean result, move to buffer */
	cmdnode->cmdbuf->h_cmd  = (u8)command;
	cmdnode->cmdbuf->len    = cpu_to_le16(in_cmd_size);
    cmdnode->cmdbuf->c_type = HOST_CMD;
	// cmdnode->cmdbuf->result  = 0;
    if (in_cmd_size > 7)
	cabrio_dbg_host("PREP_CMD: command 0x%04x -- %02X %02X %02X %02X %02X %02X %02X %02X -- %02X %02X %02X %02X %02X %02X %02X %02X \n",
			        command,
			        ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 8], ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 7],
			        ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 6], ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 5],
			        ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 4], ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 3],
			        ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 2], ((u8 *)cmdnode->cmdbuf)[in_cmd_size - 1],
			        in_cmd[in_cmd_size - 8], in_cmd[in_cmd_size - 7],
			        in_cmd[in_cmd_size - 6], in_cmd[in_cmd_size - 5],
		            in_cmd[in_cmd_size - 4], in_cmd[in_cmd_size - 3],
		            in_cmd[in_cmd_size - 2], in_cmd[in_cmd_size - 1]);

	cmdnode->cmdwaitqwoken = 0;
	cabrio_queue_cmd(priv, cmdnode);
	wake_up(&priv->waitq);

 done:
	cabrio_dbg_leave_args(CABRIO_DBG_HOST, "ret %p", cmdnode);
	return cmdnode;
}

void cabrio_cmd_async(struct cabrio_private *priv, uint16_t command,
	HDR_HostCmd *in_cmd, int in_cmd_size)
{
	cabrio_dbg_enter(CABRIO_DBG_CMD);
	__cabrio_cmd_async(priv, command, (const u8 *)in_cmd, in_cmd_size,
		cabrio_cmd_async_callback, 0);
	cabrio_dbg_leave(CABRIO_DBG_CMD);
}
EXPORT_SYMBOL_GPL(cabrio_cmd_async);


int __cabrio_cmd(struct cabrio_private *priv, uint16_t command,
	             const u8 *in_cmd, int in_cmd_size,
	             int (*callback)(struct cabrio_private *, unsigned long, HDR_HostEvent *),
	             unsigned long callback_arg)
{
	struct host_cmd_node *cmdnode;
	unsigned long flags;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_HOST);

	cmdnode = __cabrio_cmd_async(priv, command, in_cmd, in_cmd_size,
				  callback, callback_arg);
	if (IS_ERR(cmdnode)) {
		ret = PTR_ERR(cmdnode);
		goto done;
	}

	might_sleep();

	/*
	 * Be careful with signals here. A signal may be received as the system
	 * goes into suspend or resume. We do not want this to interrupt the
	 * command, so we perform an uninterruptible sleep.
	 */
	wait_event(cmdnode->cmdwait_q, cmdnode->cmdwaitqwoken);

	spin_lock_irqsave(&priv->driver_lock, flags);
	ret = cmdnode->result;
	if (cmdnode->result)
		netdev_info(priv->dev, "PREP_CMD: command 0x%04x failed: %d\n",
			    command, ret);

	__cabrio_cleanup_and_insert_cmd(priv, cmdnode);
	spin_unlock_irqrestore(&priv->driver_lock, flags);

done:
	cabrio_dbg_leave_args(CABRIO_DBG_HOST, "ret %d", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(__cabrio_cmd);

