/*
 * Implement cfg80211 ("iw") support.
 *
 * Copyright (C) 2009 M&N Solutions GmbH, 61191 Rosbach, Germany
 * Holger Schurig <hs4233@mail.mn-solutions.de>
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/hardirq.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/ieee80211.h>
#include <net/cfg80211.h>
#include <net/lib80211.h>
#include <asm/unaligned.h>

#include "decl.h"
#include "cfg.h"
#include "cmd.h"
#include "cabrio-if/cmd_def.h"
#include "cabrio-if/ssv_pktdef.h"
#include "cabrio-if/common.h"
// SSV -- #include "mesh.h"


#define CHAN2G(_channel, _freq, _flags) {        \
	.band             = IEEE80211_BAND_2GHZ, \
	.center_freq      = (_freq),             \
	.hw_value         = (_channel),          \
	.flags            = (_flags),            \
	.max_antenna_gain = 0,                   \
	.max_power        = 30,                  \
}

static struct ieee80211_channel cabrio_2ghz_channels[] = {
	CHAN2G(1,  2412, 0),
	CHAN2G(2,  2417, 0),
	CHAN2G(3,  2422, 0),
	CHAN2G(4,  2427, 0),
	CHAN2G(5,  2432, 0),
	CHAN2G(6,  2437, 0),
	CHAN2G(7,  2442, 0),
	CHAN2G(8,  2447, 0),
	CHAN2G(9,  2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

#define RATETAB_ENT(_rate, _hw_value, _flags) { \
	.bitrate  = (_rate),                    \
	.hw_value = (_hw_value),                \
	.flags    = (_flags),                   \
}


/* Table 6 in section 3.2.1.1 */
static struct ieee80211_rate cabrio_rates[] = {
	RATETAB_ENT(10,  0,  0),
	RATETAB_ENT(20,  1,  0),
	RATETAB_ENT(55,  2,  0),
	RATETAB_ENT(110, 3,  0),
	RATETAB_ENT(60,  9,  0),
	RATETAB_ENT(90,  6,  0),
	RATETAB_ENT(120, 7,  0),
	RATETAB_ENT(180, 8,  0),
	RATETAB_ENT(240, 9,  0),
	RATETAB_ENT(360, 10, 0),
	RATETAB_ENT(480, 11, 0),
	RATETAB_ENT(540, 12, 0),
};

static struct ieee80211_supported_band cabrio_band_2ghz = {
	.channels = cabrio_2ghz_channels,
	.n_channels = ARRAY_SIZE(cabrio_2ghz_channels),
	.bitrates = cabrio_rates,
	.n_bitrates = ARRAY_SIZE(cabrio_rates),
};


static const u32 cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
};

/* Time to stay on the channel */
#define CABRIO_DWELL_PASSIVE 100
#define CABRIO_DWELL_ACTIVE  40


/***************************************************************************
 * Misc utility functions
 *
 * Convert IE's auth type to Cabrio's. They are very similar to IEs, they have the
 * same structure: type, length, data*. The only difference: for IEs, the
 * type and length are u8, but for Cabrio's they're __le16.
 */

#if ORIG
/*
 * Convert NL80211's auth_type to the one from Cabrio.
 */
static u8 cabrio_auth_to_authtype(enum nl80211_auth_type auth_type)
{
	int ret = -ENOTSUPP;

	switch (auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
	case NL80211_AUTHTYPE_SHARED_KEY:
		ret = auth_type;
		break;
	case NL80211_AUTHTYPE_AUTOMATIC:
		ret = NL80211_AUTHTYPE_OPEN_SYSTEM;
		break;
	case NL80211_AUTHTYPE_NETWORK_EAP:
		ret = 0x80;
		break;
	default:
		/* silence compiler */
		break;
	}
	return ret;
}


/*
 * Various firmware commands need the list of supported rates, but with
 * the high-bit set for basic rates
 */
static int cabrio_add_rates(u8 *rates)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(cabrio_rates); i++) {
		u8 rate = cabrio_rates[i].bitrate / 5;
		if (rate == 0x02 || rate == 0x04 ||
		    rate == 0x0b || rate == 0x16)
			rate |= 0x80;
		rates[i] = rate;
	}
	return ARRAY_SIZE(cabrio_rates);
}


/*
 * Add ssid SSS
 */
#define CABRIO_MAX_SSID_SSS_SIZE			\
	(sizeof(struct cabrio_ie_header)		\
	 + IEEE80211_MAX_SSID_LEN)

static int cabrio_add_ssid_sss(u8 *sss, const u8 *ssid, int ssid_len)
{
	struct cabrio_ie_ssid_param_set *ssid_sss = (void *)sss;

	/*
	 * SSS-ID SSID  00 00
	 * length       06 00
	 * ssid         4d 4e 54 45 53 54
	 */
	ssid_sss->header.type = cpu_to_le16(SSS_TYPE_SSID);
	ssid_sss->header.len = cpu_to_le16(ssid_len);
	memcpy(ssid_sss->ssid, ssid, ssid_len);
	return sizeof(ssid_sss->header) + ssid_len;
}


/*
 * Add channel list SSS
 *
 * Actual channel data comes from priv->wdev->wiphy->channels.
 */
#define CABRIO_MAX_CHANNEL_LIST_SSS_SIZE					\
	(sizeof(struct cabrio_ie_header)					\
	 + (CABRIO_SCAN_BEFORE_NAP * sizeof(struct chanscanparamset)))

static int cabrio_add_channel_list_sss(struct cabrio_private *priv, u8 *sss,
				    int last_channel, int active_scan)
{
	int chanscanparamsize = sizeof(struct chanscanparamset) *
		(last_channel - priv->scan_channel);

	struct cabrio_ie_header *header = (void *) sss;

	/*
	 * SSS-ID CHANLIST  01 01
	 * length           0e 00
	 * channel          00 01 00 00 00 64 00
	 *   radio type     00
	 *   channel           01
	 *   scan type            00
	 *   min scan time           00 00
	 *   max scan time                 64 00
	 * channel 2        00 02 00 00 00 64 00
	 *
	 */

	header->type = cpu_to_le16(SSS_TYPE_CHANLIST);
	header->len  = cpu_to_le16(chanscanparamsize);
	sss += sizeof(struct cabrio_ie_header);

	/* cabrio_dbg_scan("scan: channels %d to %d\n", priv->scan_channel,
		     last_channel); */
	memset(sss, 0, chanscanparamsize);

	while (priv->scan_channel < last_channel) {
		struct chanscanparamset *param = (void *) sss;

		param->radiotype = CMD_SCAN_RADIO_TYPE_BG;
		param->channumber =
			priv->scan_req->channels[priv->scan_channel]->hw_value;
		if (active_scan) {
			param->maxscantime = cpu_to_le16(CABRIO_DWELL_ACTIVE);
		} else {
			param->chanscanmode.passivescan = 1;
			param->maxscantime = cpu_to_le16(CABRIO_DWELL_PASSIVE);
		}
		sss += sizeof(struct chanscanparamset);
		priv->scan_channel++;
	}
	return sizeof(struct cabrio_ie_header) + chanscanparamsize;
}


/*
 * Add rates SSS
 *
 * The rates are in cabrio_bg_rates[], but for the 802.11b
 * rates the high bit is set. We add this SSS only because
 * there's a firmware which otherwise doesn't report all
 * APs in range.
 */
#define CABRIO_MAX_RATES_SSS_SIZE			\
	(sizeof(struct cabrio_ie_header)		\
	 + (ARRAY_SIZE(cabrio_rates)))

/* Adds a SSS with all rates the hardware supports */
static int cabrio_add_supported_rates_sss(u8 *sss)
{
	size_t i;
	struct cabrio_ie_rates_param_set *rate_sss = (void *)sss;

	/*
	 * SSS-ID RATES  01 00
	 * length        0e 00
	 * rates         82 84 8b 96 0c 12 18 24 30 48 60 6c
	 */
	rate_sss->header.type = cpu_to_le16(SSS_TYPE_RATES);
	sss += sizeof(rate_sss->header);
	i = cabrio_add_rates(sss);
	sss += i;
	rate_sss->header.len = cpu_to_le16(i);
	return sizeof(rate_sss->header) + i;
}

/* Add common rates from a SSS and return the new end of the SSS */
static u8 *
add_ie_rates(u8 *sss, const u8 *ie, int *nrates)
{
	int hw, ap, ap_max = ie[1];
	u8 hw_rate;

	/* Advance past IE header */
	ie += 2;

	cabrio_dbg_hex(CABRIO_DBG_ASSOC, "AP IE Rates", (u8 *) ie, ap_max);

	for (hw = 0; hw < ARRAY_SIZE(cabrio_rates); hw++) {
		hw_rate = cabrio_rates[hw].bitrate / 5;
		for (ap = 0; ap < ap_max; ap++) {
			if (hw_rate == (ie[ap] & 0x7f)) {
				*sss++ = ie[ap];
				*nrates = *nrates + 1;
			}
		}
	}
	return sss;
}

/*
 * Adds a SSS with all rates the hardware *and* BSS supports.
 */
static int cabrio_add_common_rates_sss(u8 *sss, struct cfg80211_bss *bss)
{
	struct cabrio_ie_rates_param_set *rate_sss = (void *)sss;
	const u8 *rates_eid, *ext_rates_eid;
	int n = 0;

	rates_eid = ieee80211_bss_get_ie(bss, WLAN_EID_SUPP_RATES);
	ext_rates_eid = ieee80211_bss_get_ie(bss, WLAN_EID_EXT_SUPP_RATES);

	/*
	 * 01 00                   SSS_TYPE_RATES
	 * 04 00                   len
	 * 82 84 8b 96             rates
	 */
	rate_sss->header.type = cpu_to_le16(SSS_TYPE_RATES);
	sss += sizeof(rate_sss->header);

	/* Add basic rates */
	if (rates_eid) {
		sss = add_ie_rates(sss, rates_eid, &n);

		/* Add extended rates, if any */
		if (ext_rates_eid)
			sss = add_ie_rates(sss, ext_rates_eid, &n);
	} else {
		cabrio_dbg_assoc("assoc: bss had no basic rate IE\n");
		/* Fallback: add basic 802.11b rates */
		*sss++ = 0x82;
		*sss++ = 0x84;
		*sss++ = 0x8b;
		*sss++ = 0x96;
		n = 4;
	}

	rate_sss->header.len = cpu_to_le16(n);
	return sizeof(rate_sss->header) + n;
}


/*
 * Add auth type SSS.
 *
 * This is only needed for newer firmware.
 */
#define CABRIO_MAX_AUTH_TYPE_SSS_SIZE \
	sizeof(struct cabrio_ie_auth_type)

static int cabrio_add_auth_type_sss(u8 *sss, enum nl80211_auth_type auth_type)
{
	struct cabrio_ie_auth_type *auth = (void *) sss;

	/*
	 * 1f 01  SSS_TYPE_AUTH_TYPE
	 * 01 00  len
	 * 01     auth type
	 */
	auth->header.type = cpu_to_le16(SSS_TYPE_AUTH_TYPE);
	auth->header.len = cpu_to_le16(sizeof(*auth)-sizeof(auth->header));
	auth->auth = cpu_to_le16(cabrio_auth_to_authtype(auth_type));
	return sizeof(*auth);
}


/*
 * Add channel (phy ds) SSS
 */
#define CABRIO_MAX_CHANNEL_SSS_SIZE \
	sizeof(struct cabrio_ie_header)

static int cabrio_add_channel_sss(u8 *sss, u8 channel)
{
	struct cabrio_ie_ds_param_set *ds = (void *) sss;

	/*
	 * 03 00  SSS_TYPE_PHY_DS
	 * 01 00  len
	 * 06     channel
	 */
	ds->header.type = cpu_to_le16(SSS_TYPE_PHY_DS);
	ds->header.len = cpu_to_le16(sizeof(*ds)-sizeof(ds->header));
	ds->channel = channel;
	return sizeof(*ds);
}


/*
 * Add (empty) CF param SSS of the form:
 */
#define CABRIO_MAX_CF_PARAM_SSS_SIZE		\
	sizeof(struct cabrio_ie_header)

static int cabrio_add_cf_param_sss(u8 *sss)
{
	struct cabrio_ie_cf_param_set *cf = (void *)sss;

	/*
	 * 04 00  SSS_TYPE_CF
	 * 06 00  len
	 * 00     cfpcnt
	 * 00     cfpperiod
	 * 00 00  cfpmaxduration
	 * 00 00  cfpdurationremaining
	 */
	cf->header.type = cpu_to_le16(SSS_TYPE_CF);
	cf->header.len = cpu_to_le16(sizeof(*cf)-sizeof(cf->header));
	return sizeof(*cf);
}

/*
 * Add WPA SSS
 */
#define CABRIO_MAX_WPA_SSS_SIZE			\
	(sizeof(struct cabrio_ie_header)		\
	 + 128 /* TODO: I guessed the size */)

static int cabrio_add_wpa_sss(u8 *sss, const u8 *ie, u8 ie_len)
{
	size_t sss_len;

	/*
	 * We need just convert an IE to an SSS. IEs use u8 for the header,
	 *   u8      type
	 *   u8      len
	 *   u8[]    data
	 * but SSSs use __le16 instead:
	 *   __le16  type
	 *   __le16  len
	 *   u8[]    data
	 */
	*sss++ = *ie++;
	*sss++ = 0;
	sss_len = *sss++ = *ie++;
	*sss++ = 0;
	while (sss_len--)
		*sss++ = *ie++;
	/* the SSS is two bytes larger than the IE */
	return ie_len + 2;
}
#endif // ORIG


/*
 * Set Channel
 */

static int cabrio_cfg_set_channel(struct wiphy *wiphy,
	struct net_device *netdev,
	struct ieee80211_channel *channel,
	enum nl80211_channel_type channel_type)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	int ret = -ENOTSUPP;

	cabrio_dbg_enter_args(CABRIO_DBG_CFG80211, "iface %s freq %d, type %d",
			   netdev_name(netdev), channel->center_freq, channel_type);

	if (channel_type != NL80211_CHAN_NO_HT)
		goto out;
	// SSV --
	#if 0
	if (netdev == priv->mesh_dev)
		ret = cabrio_mesh_set_channel(priv, channel->hw_value);
	else
	#endif // CONFIG_LIBERTAS_MESH
	ret = cabrio_set_channel(priv, channel->hw_value);
 out:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}


/*
 * Scanning
 */

/*
 * When scanning, the firmware doesn't send a nul packet with the power-safe
 * bit to the AP. So we cannot stay away from our current channel too long,
 * otherwise we loose data. So take a "nap" while scanning every other
 * while.
 */
//#define CABRIO_SCAN_BEFORE_NAP 4
#define CABRIO_SCAN_BEFORE_NAP 14


static int cabrio_ret_scan(struct cabrio_private *priv, unsigned long dummy,
	                       HDR_HostEvent *p_host_event)
{
    int 		ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_SCAN);

    cabrio_dbg_scan("Scan: FW finished one scan.\n");
    
// done:
    cabrio_dbg_leave_args(CABRIO_DBG_SCAN, "ret %d", ret);
    return ret;
}

#if ORIG
/*
 * When the firmware reports back a scan-result, it gives us an "u8 rssi",
 * which isn't really an RSSI, as it becomes larger when moving away from
 * the AP. Anyway, we need to convert that into mBm.
*/
#define CABRIO_SCAN_RSSI_TO_MBM(rssi) \
	((-(int)rssi + 3)*100)

static int cabrio_ret_scan_orig(struct cabrio_private *priv, unsigned long dummy,
	struct cmd_header *resp)
{
	struct cmd_ds_802_11_scan_rsp *scanresp = (void *)resp;
	int bsssize;
	const u8 *pos;
	const u8 *tsfdesc;
	int tsfsize;
	int i;
	int ret = -EILSEQ;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	bsssize = get_unaligned_le16(&scanresp->bssdescriptsize);

	cabrio_dbg_scan("scan response: %d BSSs (%d bytes); resp size %d bytes\n",
			scanresp->nr_sets, bsssize, le16_to_cpu(resp->size));

	if (scanresp->nr_sets == 0) {
		ret = 0;
		goto done;
	}

	/*
	 * The general layout of the scan response is described in chapter
	 * 5.7.1. Basically we have a common part, then any number of BSS
	 * descriptor sections. Finally we have section with the same number
	 * of TSFs.
	 *
	 * cmd_ds_802_11_scan_rsp
	 *   cmd_header
	 *   pos_size
	 *   nr_sets
	 *   bssdesc 1
	 *     bssid
	 *     rssi
	 *     timestamp
	 *     intvl
	 *     capa
	 *     IEs
	 *   bssdesc 2
	 *   bssdesc n
	 *   MrvlIEtypes_TsfFimestamp_t
	 *     TSF for BSS 1
	 *     TSF for BSS 2
	 *     TSF for BSS n
	 */

	pos = scanresp->bssdesc_and_sssbuffer;

	cabrio_dbg_hex(CABRIO_DBG_SCAN, "SCAN_RSP", scanresp->bssdesc_and_sssbuffer,
			scanresp->bssdescriptsize);

	tsfdesc = pos + bsssize;
	tsfsize = 4 + 8 * scanresp->nr_sets;
	cabrio_dbg_hex(CABRIO_DBG_SCAN, "SCAN_TSF", (u8 *) tsfdesc, tsfsize);

	/* Validity check: we expect a Marvell-Local SSS */
	i = get_unaligned_le16(tsfdesc);
	tsfdesc += 2;
	if (i != SSS_TYPE_TSFTIMESTAMP) {
		cabrio_dbg_scan("scan response: invalid TSF Timestamp %d\n", i);
		goto done;
	}

	/*
	 * Validity check: the SSS holds TSF values with 8 bytes each, so
	 * the size in the SSS must match the nr_sets value
	 */
	i = get_unaligned_le16(tsfdesc);
	tsfdesc += 2;
	if (i / 8 != scanresp->nr_sets) {
		cabrio_dbg_scan("scan response: invalid number of TSF timestamp "
			     "sets (expected %d got %d)\n", scanresp->nr_sets,
			     i / 8);
		goto done;
	}

	for (i = 0; i < scanresp->nr_sets; i++) {
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
	}
	ret = 0;

done:
	cabrio_dbg_leave_args(CABRIO_DBG_SCAN, "ret %d", ret);
	return ret;
}
#endif // ORIG

/*
 * Assumes priv->scan_req is initialized and valid
 * Assumes priv->scan_channel is initialized
 */
static void cabrio_scan_worker(struct work_struct *work)
{
    struct cabrio_private *priv =
        container_of(work, struct cabrio_private, scan_work.work);
    struct cfg_scan_request *scan_cmd;
    HDR_HostCmd *cmd;
    int last_channel;
    int start_channel;
    int running, carrier;
    u32 num_ssids = priv->scan_req->n_ssids;
    size_t cmd_size =   HOST_CMD_HDR_LEN 
                      + sizeof(struct cfg_scan_request)
                      + sizeof(struct cfg_80211_ssid)*num_ssids;
    u32 i;

    cabrio_dbg_enter(CABRIO_DBG_SCAN);
    
    cmd = (HDR_HostCmd *)kzalloc(cmd_size, GFP_KERNEL);
    if (cmd == NULL)
        goto out_no_scan_cmd;

    cmd->c_type = HOST_CMD;
    cmd->h_cmd = SSV_HOST_CMD_SCAN;
    cmd->len = cmd_size;
    scan_cmd = (struct cfg_scan_request *)&cmd->dat8[0];
        
    /* stop network while we're away from our main channel */
    running = !netif_queue_stopped(priv->dev);
    carrier = netif_carrier_ok(priv->dev);
    if (running)
        netif_stop_queue(priv->dev);
    if (carrier)
        netif_carrier_off(priv->dev);

    /* rates */
    /* channel dwell time */
    
    /* SSIDs */
    for (i = 0, scan_cmd->n_ssids = 0; i < priv->scan_req->n_ssids; i++) {
        if (priv->scan_req->ssids[0].ssid_len == 0)
            continue;
        memcpy(scan_cmd->ssids[scan_cmd->n_ssids++].ssid, 
               priv->scan_req->ssids[i].ssid, 
               priv->scan_req->ssids[i].ssid_len);
    }
    
    if (priv->connect_status == CABRIO_CONNECTED) {
    	scan_cmd->dwell_time = 1; // 10ms;
        scan_cmd->is_active = 1;
    } else {
    	scan_cmd->dwell_time = 0; // Use default channel dwell time;
        scan_cmd->is_active = (num_ssids > 0);
    }

    /* Set scan channels */
    start_channel = priv->scan_channel;
    last_channel = priv->scan_channel + CABRIO_SCAN_BEFORE_NAP;
    if (last_channel > priv->scan_req->n_channels)
        last_channel = priv->scan_req->n_channels;


    for (i = priv->scan_channel, scan_cmd->channel_mask = 0; i < last_channel; i++) {
        scan_cmd->channel_mask |= (1 << priv->scan_req->channels[i]->hw_value);
        priv->scan_channel++;
    }

    /* More channels to scan */
    if (priv->scan_channel < priv->scan_req->n_channels) {
        cancel_delayed_work(&priv->scan_work);
        if (netif_running(priv->dev))
            queue_delayed_work(priv->work_thread, &priv->scan_work,
            				   (  (priv->connect_status == CABRIO_CONNECTED)
            				    ? msecs_to_jiffies(800)
            				    : msecs_to_jiffies(300)));
    }

    /* Send cmd */

    cabrio_dbg_scan("Scan %d - %d : %04X -- %08X %08X\n",
    		        start_channel, last_channel, scan_cmd->channel_mask,
    		        ((u32 *)scan_cmd)[0], ((u32 *)scan_cmd)[1]);

    //cabrio_dbg_hex(CABRIO_DBG_SCAN, "SCAN_CMD", (void *)scan_cmd, cmd_size);
    __cabrio_cmd(priv, SSV_HOST_CMD_SCAN, (const u8 *)cmd, cmd_size, cabrio_ret_scan, 0);
    
    if (priv->scan_channel >= priv->scan_req->n_channels) {
       /* Mark scan done */
       cancel_delayed_work(&priv->scan_work);
       cabrio_scan_done(priv);
    }

    /* Restart network */
    if (carrier)
        netif_carrier_on(priv->dev);
    if (running && !priv->tx_pending_len)
        netif_wake_queue(priv->dev);
    
    kfree(cmd);
    
    /* Wake up anything waiting on scan completion */
    if (priv->scan_req == NULL) {
        cabrio_dbg_scan("scan: waking up waiters\n");
        wake_up_all(&priv->scan_q);
    }
    
out_no_scan_cmd:
    cabrio_dbg_leave(CABRIO_DBG_SCAN);
} // end of - cabrio_scan_worker -


/*
 * Our scan command contains a SSS, consting of a SSID SSS, a channel list
 * SSS and a rates SSS. Determine the maximum size of them:
 */
#if ORIG
#define CABRIO_SCAN_MAX_CMD_SIZE			\
	(sizeof(struct cmd_ds_802_11_scan)	\
	 + CABRIO_MAX_SSID_SSS_SIZE		\
	 + CABRIO_MAX_CHANNEL_LIST_SSS_SIZE	\
	 + CABRIO_MAX_RATES_SSS_SIZE)

static void cabrio_scan_worker_orig(struct work_struct *work)
{
	struct cabrio_private *priv =
		container_of(work, struct cabrio_private, scan_work.work);
	struct cmd_ds_802_11_scan *scan_cmd;
	u8 *sss; /* pointer into our current, growing SSS storage area */
	int last_channel;
	int running, carrier;

	cabrio_dbg_enter(CABRIO_DBG_SCAN);

	scan_cmd = kzalloc(CABRIO_SCAN_MAX_CMD_SIZE, GFP_KERNEL);
	if (scan_cmd == NULL)
		goto out_no_scan_cmd;

	/* prepare fixed part of scan command */
	scan_cmd->bsstype = CMD_BSS_TYPE_ANY;

	/* stop network while we're away from our main channel */
	running = !netif_queue_stopped(priv->dev);
	carrier = netif_carrier_ok(priv->dev);
	if (running)
		netif_stop_queue(priv->dev);
	if (carrier)
		netif_carrier_off(priv->dev);

	/* prepare fixed part of scan command */
	sss = scan_cmd->sssbuffer;

	/* add SSID SSS */
	if (priv->scan_req->n_ssids && priv->scan_req->ssids[0].ssid_len > 0)
		sss += cabrio_add_ssid_sss(sss,
					priv->scan_req->ssids[0].ssid,
					priv->scan_req->ssids[0].ssid_len);

	/* add channel SSSs */
	last_channel = priv->scan_channel + CABRIO_SCAN_BEFORE_NAP;
	if (last_channel > priv->scan_req->n_channels)
		last_channel = priv->scan_req->n_channels;
	sss += cabrio_add_channel_list_sss(priv, sss, last_channel,
		priv->scan_req->n_ssids);

	/* add rates SSS */
	sss += cabrio_add_supported_rates_sss(sss);

	if (priv->scan_channel < priv->scan_req->n_channels) {
		cancel_delayed_work(&priv->scan_work);
		if (netif_running(priv->dev))
			queue_delayed_work(priv->work_thread, &priv->scan_work,
				msecs_to_jiffies(300));
	}

	/* This is the final data we are about to send */
	scan_cmd->hdr.size = cpu_to_le16(sss - (u8 *)scan_cmd);
	cabrio_dbg_hex(CABRIO_DBG_SCAN, "SCAN_CMD", (void *)scan_cmd,
		    sizeof(*scan_cmd));
	cabrio_dbg_hex(CABRIO_DBG_SCAN, "SCAN_SSS", scan_cmd->sssbuffer,
		    sss - scan_cmd->sssbuffer);

	__cabrio_cmd(priv, CMD_802_11_SCAN, &scan_cmd->hdr,
		le16_to_cpu(scan_cmd->hdr.size),
		cabrio_ret_scan, 0);

	if (priv->scan_channel >= priv->scan_req->n_channels) {
		/* Mark scan done */
		cancel_delayed_work(&priv->scan_work);
		cabrio_scan_done(priv);
	}

	/* Restart network */
	if (carrier)
		netif_carrier_on(priv->dev);
	if (running && !priv->tx_pending_len)
		netif_wake_queue(priv->dev);

	kfree(scan_cmd);

	/* Wake up anything waiting on scan completion */
	if (priv->scan_req == NULL) {
		cabrio_dbg_scan("scan: waking up waiters\n");
		wake_up_all(&priv->scan_q);
	}

 out_no_scan_cmd:
	cabrio_dbg_leave(CABRIO_DBG_SCAN);
}
#endif // ORIG

static void _internal_start_scan(struct cabrio_private *priv, bool internal,
	struct cfg80211_scan_request *request)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	cabrio_dbg_scan("scan: ssids %d, channels %d, ie_len %zd\n",
		request->n_ssids, request->n_channels, request->ie_len);

	priv->scan_channel = 0;
	priv->scan_req = request;
	priv->internal_scan = internal;

	queue_delayed_work(priv->work_thread, &priv->scan_work,
		msecs_to_jiffies(50));

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
}

/*
 * Clean up priv->scan_req.  Should be used to handle the allocation details.
 */
void cabrio_scan_done(struct cabrio_private *priv)
{
	WARN_ON(!priv->scan_req);

	if (priv->internal_scan)
		kfree(priv->scan_req);
	else
		cfg80211_scan_done(priv->scan_req, false);

	priv->scan_req = NULL;
}

static int cabrio_cfg_scan(struct wiphy *wiphy,
	struct net_device *dev,
	struct cfg80211_scan_request *request)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (priv->scan_req || delayed_work_pending(&priv->scan_work)) {
		/* old scan request not yet processed */
		ret = -EAGAIN;
		goto out;
	}

	_internal_start_scan(priv, false, request);

	if (priv->surpriseremoved)
		ret = -EIO;

 out:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}




/*
 * Events
 */

void cabrio_send_disconnect_notification(struct cabrio_private *priv)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	cfg80211_disconnected(priv->dev,
		0,
		NULL, 0,
		GFP_KERNEL);
	priv->is_securied_connected = 0;
	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
}

void cabrio_send_mic_failureevent(struct cabrio_private *priv, u32 event)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	cfg80211_michael_mic_failure(priv->dev,
		priv->assoc_bss,
		event == MACREG_INT_CODE_MIC_ERR_MULTICAST ?
			NL80211_KEYTYPE_GROUP :
			NL80211_KEYTYPE_PAIRWISE,
		-1,
		NULL,
		GFP_KERNEL);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
}




/*
 * Connect/disconnect
 */


#if ORIG
/*
 * This removes all WEP keys
 */
static int cabrio_remove_wep_keys(struct cabrio_private *priv)
{
	int ret = 0;
#if 1
    CABRIO_TODO(__func__);
#else
	struct cmd_ds_802_11_set_wep cmd;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.keyindex = cpu_to_le16(priv->wep_tx_key);
	cmd.action = cpu_to_le16(CMD_ACT_REMOVE);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_SET_WEP, &cmd);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
#endif // TODO
	return ret;
}
#endif // ORIG


/*
 * Set WEP keys
 */
static int cabrio_set_wep_keys(struct cabrio_private *priv)
{
	int ret = 0;
#if 1
    CABRIO_TODO(__func__);
#else

	struct cmd_ds_802_11_set_wep cmd;
	int i;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);


	/*
	 * command         13 00
	 * size            50 00
	 * sequence        xx xx
	 * result          00 00
	 * action          02 00     ACT_ADD
	 * transmit key    00 00
	 * type for key 1  01        WEP40
	 * type for key 2  00
	 * type for key 3  00
	 * type for key 4  00
	 * key 1           39 39 39 39 39 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 * key 2           00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 * key 3           00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 * key 4           00 00 00 00 00 00 00 00
	 */

    // Cabrio TODO
	if (priv->wep_key_len[0] || priv->wep_key_len[1] ||
	    priv->wep_key_len[2] || priv->wep_key_len[3]) {
		/* Only set wep keys if we have at least one of them */
		memset(&cmd, 0, sizeof(cmd));
		cmd.hdr.size = cpu_to_le16(sizeof(cmd));
		cmd.keyindex = cpu_to_le16(priv->wep_tx_key);
		cmd.action = cpu_to_le16(CMD_ACT_ADD);

		for (i = 0; i < 4; i++) {
			switch (priv->wep_key_len[i]) {
			case WLAN_KEY_LEN_WEP40:
				cmd.keytype[i] = CMD_TYPE_WEP_40_BIT;
				break;
			case WLAN_KEY_LEN_WEP104:
				cmd.keytype[i] = CMD_TYPE_WEP_104_BIT;
				break;
			default:
				cmd.keytype[i] = 0;
				break;
			}
			memcpy(cmd.keymaterial[i], priv->wep_key[i],
			       priv->wep_key_len[i]);
		}

		ret = cabrio_cmd_with_response(priv, CMD_802_11_SET_WEP, &cmd);
	} else {
		/* Otherwise remove all wep keys */
		ret = cabrio_remove_wep_keys(priv);
	}

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
#endif  // TODO
	return ret;
}


#if ORIG
/*
 * Enable/Disable RSN status
 */
static int cabrio_enable_rsn(struct cabrio_private *priv, int enable)
{
        int ret = 0;
#if 1
        CABRIO_TODO(__func__);
#else
	struct cmd_ds_802_11_enable_rsn cmd;
	int ret;

	cabrio_dbg_enter_args(CABRIO_DBG_CFG80211, "%d", enable);

	/*
	 * cmd       2f 00
	 * size      0c 00
	 * sequence  xx xx
	 * result    00 00
	 * action    01 00    ACT_SET
	 * enable    01 00
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.enable = cpu_to_le16(enable);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_ENABLE_RSN, &cmd);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
#endif // TODO
	return ret;
}
#endif // ORIG

/*
 * Set WPA/WPA key material
 */

/*
 * like "struct cmd_ds_802_11_key_material", but with cmd_header. Once we
 * get rid of WEXT, this should go into host.h
 */
static int prepare_host_cmd (size_t cmd_extra_size,
		                     HDR_HostCmd **cmd, void **cmd_extra,
		                     HDR_HostEvent **event, struct resp_evt_result **resp)
{
    s32                      cmd_size = HOST_CMD_HDR_LEN + cmd_extra_size;
    s32                      event_size = sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result);
    u8                      *buf;

    buf = (u8 *)kzalloc(cmd_size + event_size, GFP_KERNEL);
    if (buf == NULL)
        return -ENOMEM;

    *cmd = (HDR_HostCmd *)buf;
    (*cmd)->c_type = HOST_CMD;
    (*cmd)->len = cmd_size;
    if (cmd_extra_size && (cmd_extra != NULL))
    	*cmd_extra = (void *)(buf + HOST_CMD_HDR_LEN);

    *event = (HDR_HostEvent *)(buf + cmd_size);
   	*resp = (struct resp_evt_result *)(buf + cmd_size + sizeof(HDR_HostEvent));

    return 0;
} // end of - prepare_host_cmd -

static int cleanup_host_cmd (HDR_HostCmd *cmd)
{
	if (cmd)
	    kfree(cmd);
	return 0;
} // end of - cleanup_host_cmd -


static int cabrio_set_key_material(struct cabrio_private *priv,
				int key_type,
				int key_idx,
				u8 *key, u16 key_len)
{
	int                      ret = 0;
    HDR_HostCmd             *cmd = NULL;
    HDR_HostEvent           *event;
    struct resp_evt_result  *resp;
	struct securityEntry    *sec_entry;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	ret = prepare_host_cmd(sizeof(struct securityEntry), &cmd, (void **)&sec_entry, &event, &resp);
	if (ret)
	    goto done;
	switch (key_type) {
		case KEY_TYPE_ID_TKIP:
			sec_entry->cipher = CIPHER_HOST_TKIP;
			break;
		case KEY_TYPE_ID_AES:
			sec_entry->cipher = CIPHER_HOST_CCMP;
			break;
		default:
			pr_err("");
			ret = -EINVAL;
			goto done;
	}

	sec_entry->wpaUnicast = (key_idx == 0);
	sec_entry->keyLen = key_len;
	sec_entry->keyIndex = key_idx;
	memcpy(sec_entry->key, key, key_len);
    #if 0
	cabrio_dbg_assoc("%s: type:%d idx:%08X uni:%d\n", __func__, key_type, key_idx, sec_entry->wpaUnicast);

	{
	u8 *d = cmd->dat8;
	cabrio_dbg_assoc("%02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X %02X %02X\n",
			         d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
			         d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]);
	d = (u8 *)sec_entry;
	cabrio_dbg_assoc("%02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X %02X %02X  %02X %02X %02X %02X\n",
			         d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
			         d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]);
	}
    #endif // 0
	ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_SET_SECURITY_ENTRY, cmd, event);
	if (ret)
	    goto done;

	if (resp->result != CMD_OK)
		ret = -EFAULT;

done:
	cleanup_host_cmd(cmd);
	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
	return ret;
} // end of - cabrio_set_key_material -

#if 0
struct cmd_key_material {
	struct cmd_header hdr;

	__le16 action;
	struct MrvlIEtype_keyParamSet param;
} __packed;


static int cabrio_set_key_material(struct cabrio_private *priv,
				int key_type,
				int key_info,
				u8 *key, u16 key_len)
{
    int ret = 0;
#if 1
    CABRIO_TODO(__func__);
    if (key_len > sizeof(priv->wpa_key)) {
        wiphy_err(priv->wdev->wiphy, "Key length is too long.\n");
        return (-1);
    }
    memcpy(priv->wpa_key, key, key_len);
#else
	struct cmd_key_material cmd;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	/*
	 * Example for WPA (TKIP):
	 *
	 * cmd       5e 00
	 * size      34 00
	 * sequence  xx xx
	 * result    00 00
	 * action    01 00
	 * SSS type  00 01    key param
	 * length    00 26
	 * key type  01 00    TKIP
	 * key info  06 00    UNICAST | ENABLED
	 * key len   20 00
	 * key       32 bytes
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	cmd.action = cpu_to_le16(CMD_ACT_SET);
	cmd.param.type = cpu_to_le16(SSS_TYPE_KEY_MATERIAL);
	cmd.param.length = cpu_to_le16(sizeof(cmd.param) - 4);
	cmd.param.keytypeid = cpu_to_le16(key_type);
	cmd.param.keyinfo = cpu_to_le16(key_info);
	cmd.param.keylen = cpu_to_le16(key_len);
	if (key && key_len)
		memcpy(cmd.param.key, key, key_len);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_KEY_MATERIAL, &cmd);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
#endif // TODO
	return ret;
}
#endif // 0

#if ORIG
/*
 * Sets the auth type (open, shared, etc) in the firmware. That
 * we use CMD_802_11_AUTHENTICATE is misleading, this firmware
 * command doesn't send an authentication frame at all, it just
 * stores the auth_type.
 */
static int cabrio_set_authtype(struct cabrio_private *priv,
			    struct cfg80211_connect_params *sme)
{
    int ret = 0;
#if 1
    CABRIO_TODO(__func__);
#else
	struct cmd_ds_802_11_authenticate cmd;
	int ret;

	cabrio_dbg_enter_args(CABRIO_DBG_CFG80211, "%d", sme->auth_type);

	/*
	 * cmd        11 00
	 * size       19 00
	 * sequence   xx xx
	 * result     00 00
	 * BSS id     00 13 19 80 da 30
	 * auth type  00
	 * reserved   00 00 00 00 00 00 00 00 00 00
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	if (sme->bssid)
		memcpy(cmd.bssid, sme->bssid, ETH_ALEN);
	/* convert auth_type */
	ret = cabrio_auth_to_authtype(sme->auth_type);
	if (ret < 0)
		goto done;

	cmd.authtype = ret;
	ret = cabrio_cmd_with_response(priv, CMD_802_11_AUTHENTICATE, &cmd);

done:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
#endif // TODO
	return ret;
}
#endif // ORIG


static void _ie_2_ssv_bss (struct ieee802_11_elems *elems, struct ssv_ieee80211_bss *ssv_bss)
{
	// SSID
    #if 0
	if (elems->ssid && elems->ssid_len>0) {
		memcpy(ssv_bss->ssid.ssid, elems->ssid, elems->ssid_len);
		ssv_bss->ssid.ssid_len = elems->ssid_len;
	}
   #endif
	// DSSS parameter: channel
    #if 0
	if (elems->ds_params && elems->ds_params_len == 1)
		ssv_bss->channel_id = elems->ds_params[0];
    #endif
	/* save the ERP value so that it is available at association time */
    if (elems->erp_info && elems->erp_info_len >= 1) {
		ssv_bss->erp_value = elems->erp_info[0];
		ssv_bss->has_erp_value = 1;
	}
	if (elems->tim) {
		ssv_bss->dtim_period = elems->tim->dtim_period;
	}
	/* If the beacon had no TIM IE, or it was invalid, use 1 */
	if (!ssv_bss->dtim_period)
		ssv_bss->dtim_period = 1;
	if (elems->wmm_param && elems->wmm_param_len>0) {
		ssv_bss->wmm_used = 1;
		ssv_bss->uapsd_supported = (elems->wmm_param[6]) >> 7;
		ssv_bss->parameter_set_count = (elems->wmm_param[6]) & 0x07;
	}
} // end of - _ie_2_ssv_bss -


// ieee802_11_parse_elems_crc is a utility function in lib80211 for mac80211.
// Not declared in public header files, so we put its declaration here.
extern u32 ieee802_11_parse_elems_crc(u8 *start, size_t len,
			       struct ieee802_11_elems *elems,
			       u64 filter, u32 crc);
/*
 * Create association request
 */
static int cabrio_associate(struct cabrio_private *priv,
                            struct cfg80211_bss *bss,
                            struct cfg80211_connect_params *sme)
{
    // struct cfg_80211_ssid   *ssid;
    s32                      size = HOST_CMD_HDR_LEN + sizeof(struct cfg_join_request);
    s32                      event_size = sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result);
    u8                      *buf;
    HDR_HostCmd             *cmd;
    HDR_HostEvent           *event;
    struct cfg_join_request *join_req;
    struct resp_evt_result  *join_resp;
    int                      ret = 0, status = WLAN_STATUS_AUTH_TIMEOUT;
    struct ieee802_11_elems  elems;
    struct ssv_ieee80211_bss *ssv_bss;

    cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	// Include IE in cfg80211_connect_params.
    if (sme)
    	size += sme->ie_len;

    buf = kzalloc(size + event_size, GFP_KERNEL);
    if (!buf) {
        ret = -ENOMEM;
        goto done;
    }

    cmd          = (HDR_HostCmd *)buf;
    event        = (HDR_HostEvent *)(buf + size);
    join_req     = (struct cfg_join_request *)cmd->dat8;
    join_resp    = (struct resp_evt_result  *)event->dat;
    ssv_bss      = &join_req->bss;

    cmd->c_type  = HOST_CMD;
    cmd->h_cmd   = SSV_HOST_CMD_JOIN;
    cmd->len     = size;
    
    event->len   = event_size;

    /* Fill in join request */
    memset(join_req, 0, sizeof(struct cfg_join_request));
    // BSSID
    memcpy(ssv_bss->bssid.addr, bss->bssid, ETH_ALEN);
    // SSID
    ssv_bss->ssid.ssid_len = sme->ssid_len;
    memcpy(ssv_bss->ssid.ssid, sme->ssid, sme->ssid_len);
    // Get BSS parameters from IE
    if ((bss->information_elements != NULL) && (bss->len_information_elements > 0)) {
    	ieee802_11_parse_elems_crc(bss->information_elements, bss->len_information_elements, &elems, 0, 0);
    	_ie_2_ssv_bss(&elems, ssv_bss);
    }

    // for WPA/WPA2 PSK.
    memcpy(join_req->password, sme->key, sme->key_len);
    // Authentication type
    join_req->sec_type = SSV_SEC_NONE;
    join_req->auth_alg = WPA_AUTH_ALG_OPEN;
    join_req->request_supplicant_bypass = 1; // Linux uses its own WPA supplicant.
    join_req->assoc_ie_len = sme->ie_len;
    if (sme->ie_len > 0)
    	memcpy(join_req->assoc_ie, sme->ie, sme->ie_len);

    switch (sme->crypto.cipher_group) {
    	case WLAN_CIPHER_SUITE_WEP40:
            join_req->sec_type = SSV_SEC_WEP_40;
	    case WLAN_CIPHER_SUITE_WEP104:
            join_req->auth_alg = WPA_AUTH_ALG_SHARED;
            if (join_req->sec_type == SSV_SEC_NONE)
                join_req->sec_type = join_req->sec_type == SSV_SEC_WEP_104;
    		/* Set WEP keys and WEP mode */
	    	cabrio_set_wep_keys(priv);
            // WEP key index.
            join_req->wep_keyidx = sme->key_idx;
	    	break;
    	case WLAN_CIPHER_SUITE_TKIP:
            join_req->sec_type = SSV_SEC_WPA_PSK;
            break;
    	case WLAN_CIPHER_SUITE_CCMP:
            join_req->sec_type = SSV_SEC_WPA2_PSK;
            break;
        default:
            break;
    }

    #if 0
    // EID??
    /* add SSID SSS */
        ssid_eid = ieee80211_bss_get_ie(bss, WLAN_EID_SSID);
        if (ssid_eid)
            pos += cabrio_add_ssid_sss(pos, ssid_eid + 2, ssid_eid[1]);
        else
            cabrio_dbg_assoc("no SSID\n");
        // 
        /* add DS param SSS */
        if (bss->channel)
            pos += cabrio_add_channel_sss(pos, bss->channel->hw_value);
        else
            cabrio_dbg_assoc("no channel\n");
    
        /* add (empty) CF param SSS */
        pos += cabrio_add_cf_param_sss(pos);
    
        /* add rates SSS */
        tmp = pos + 4; /* skip Marvell IE header */
        pos += cabrio_add_common_rates_sss(pos, bss);
        cabrio_dbg_hex(CABRIO_DBG_ASSOC, "Common Rates", tmp, pos - tmp);
    
        /* add auth type SSS */
        if (CABRIO_FW_MAJOR_REV(priv->fwrelease) >= 9)
            pos += cabrio_add_auth_type_sss(pos, sme->auth_type);
    
        /* add WPA/WPA2 SSS */
        if (sme->ie && sme->ie_len)
            pos += cabrio_add_wpa_sss(pos, sme->ie, sme->ie_len);
    
        len = (sizeof(*cmd) - sizeof(cmd->iebuf)) +
            (u16)(pos - (u8 *) &cmd->iebuf);
        cmd->hdr.size = cpu_to_le16(len);
    
        cabrio_dbg_hex(CABRIO_DBG_ASSOC, "ASSOC_CMD", (u8 *) cmd,
                le16_to_cpu(cmd->hdr.size));
    #endif // 0

    /* store for later use */
    memcpy(priv->assoc_bss, bss->bssid, ETH_ALEN);

    cabrio_dbg_assoc("Join:\n");
    cabrio_dbg_assoc("\tBSSID: %02X %02X %02X %02X %02X %02X\n",
    		   join_req->bss.bssid.addr[0], join_req->bss.bssid.addr[1],
    		   join_req->bss.bssid.addr[2], join_req->bss.bssid.addr[3],
    		   join_req->bss.bssid.addr[4], join_req->bss.bssid.addr[5]);

    cabrio_dbg_assoc("\tsec_type: %d\n", join_req->sec_type);
    cabrio_dbg_assoc("\twep_keyidx: %d\n", join_req->wep_keyidx);
    cabrio_dbg_assoc("\tpassword: \"%s\"\n", join_req->password);
    cabrio_dbg_assoc("\tauth_alg: %d\n", join_req->auth_alg);
    cabrio_dbg_assoc("\tSSID: \"%s\"\n", join_req->bss.ssid.ssid);

    {
   	const char password[] = "secret00";
    memcpy(join_req->password, password, sizeof(password));
    }
    
    priv->in_association = true;

    ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_JOIN, cmd, event);
    if (ret)
        goto asso_done;
    /* generate connect message to cfg80211 */
    
    status = le16_to_cpu(join_resp->u.join.status_code);
    
    /* Older FW versions map the IEEE 802.11 Status Code in the association
     * response to the following values returned in resp->statuscode:
     *
     *    IEEE Status Code                Marvell Status Code
     *    0                       ->      0x0000 ASSOC_RESULT_SUCCESS
     *    13                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
     *    14                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
     *    15                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
     *    16                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
     *    others                  ->      0x0003 ASSOC_RESULT_REFUSED
     *
     * Other response codes:
     *    0x0001 -> ASSOC_RESULT_INVALID_PARAMETERS (unused)
     *    0x0002 -> ASSOC_RESULT_TIMEOUT (internal timer expired waiting for
     *                                    association response from the AP)
     */
    //CABRIO_TODO("Map assocication response result to Linux status code");
    switch (status) {
        case 0:
            break;
        case 1:
            cabrio_dbg_assoc("invalid association parameters\n");
            status = WLAN_STATUS_CAPS_UNSUPPORTED;
            break;
        case 2:
            cabrio_dbg_assoc("timer expired while waiting for AP\n");
            status = WLAN_STATUS_AUTH_TIMEOUT;
            break;
        case 3:
            cabrio_dbg_assoc("association refused by AP\n");
            status = WLAN_STATUS_ASSOC_DENIED_UNSPEC;
            break;
        case 4:
            cabrio_dbg_assoc("authentication refused by AP\n");
            status = WLAN_STATUS_UNKNOWN_AUTH_TRANSACTION;
            break;
        default:
            cabrio_dbg_assoc("association failure %d\n", status);
            /* v5 OLPC firmware does return the AP status code if
             * it's not one of the values above.  Let that through.
             */
            break;
    }

    cabrio_dbg_assoc("status %d, statuscode 0x%04x, capability 0x%04x, "
                     "aid 0x%04x\n", status, join_resp->u.join.status_code,
                     (-1), join_resp->u.join.aid);
    //cabrio_dbg_hex(CABRIO_DBG_CFG80211, "Join Resp:", join_resp, sizeof(struct resp_evt_result));
    
    #if 0
    cabrio_dbg_assoc("status %d, statuscode 0x%04x, capability 0x%04x, "
                     "aid 0x%04x\n", status, le16_to_cpu(resp->statuscode),
                     le16_to_cpu(resp->capability), le16_to_cpu(resp->aid));

    resp_ie_len = le16_to_cpu(resp->hdr.size)
        - sizeof(resp->hdr)
        - 6;
    #endif
asso_done:
    cfg80211_connect_result(priv->dev,
    		                bss->bssid, //priv->assoc_bss,
                            sme->ie, sme->ie_len,
                            //resp->iebuf, resp_ie_len,
                            NULL, 0,
                            status,
                            GFP_KERNEL);
    
    if (status == 0) {
        /* TODO: get rid of priv->connect_status */
        priv->connect_status = CABRIO_CONNECTED;
        netif_carrier_on(priv->dev);
        if (!priv->tx_pending_len)
            netif_tx_wake_all_queues(priv->dev);
    }

done:
	// Flush out held RX packets during association.
#if 1
	mutex_lock(&priv->lock);
	if (priv->held_rx_skb != NULL)
		cabrio_dbg_assoc("Flush held RX packets during association.\n");
	while (priv->held_rx_skb != NULL) {
		struct sk_buff *skb = priv->held_rx_skb;
		priv->held_rx_skb = skb->next;
		if (in_interrupt())
			netif_rx(skb);
		else
			netif_rx_ni(skb);
	}
	priv->held_rx_skb_head = NULL;
	priv->in_association = false;
	mutex_unlock(&priv->lock);
#endif
    cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
    if (buf != NULL)
    	kfree(buf);
    return ret;
} // end of - cabrio_associate -


#if 0
#define CABRIO_ASSOC_MAX_CMD_SIZE                     \
	(sizeof(struct cmd_ds_802_11_associate)    \
	 - 512 /* cmd_ds_802_11_associate.iebuf */ \
	 + CABRIO_MAX_SSID_SSS_SIZE                   \
	 + CABRIO_MAX_CHANNEL_SSS_SIZE                \
	 + CABRIO_MAX_CF_PARAM_SSS_SIZE               \
	 + CABRIO_MAX_AUTH_TYPE_SSS_SIZE              \
	 + CABRIO_MAX_WPA_SSS_SIZE)

static int cabrio_associate_orig(struct cabrio_private *priv,
		struct cfg80211_bss *bss,
		struct cfg80211_connect_params *sme)
{
	struct cmd_ds_802_11_associate_response *resp;
	struct cmd_ds_802_11_associate *cmd = kzalloc(CABRIO_ASSOC_MAX_CMD_SIZE,
						      GFP_KERNEL);
	const u8 *ssid_eid;
	size_t len, resp_ie_len;
	int status;
	int ret;
	u8 *pos = &(cmd->iebuf[0]);
	u8 *tmp;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (!cmd) {
		ret = -ENOMEM;
		goto done;
	}

	/*
	 * cmd              50 00
	 * length           34 00
	 * sequence         xx xx
	 * result           00 00
	 * BSS id           00 13 19 80 da 30
	 * capabilities     11 00
	 * listen interval  0a 00
	 * beacon interval  00 00
	 * DTIM period      00
	 * SSSs             xx   (up to 512 bytes)
	 */
	cmd->hdr.command = cpu_to_le16(CMD_802_11_ASSOCIATE);

	/* Fill in static fields */
	memcpy(cmd->bssid, bss->bssid, ETH_ALEN);
	cmd->listeninterval = cpu_to_le16(CABRIO_DEFAULT_LISTEN_INTERVAL);
	cmd->capability = cpu_to_le16(bss->capability);

	/* add SSID SSS */
	ssid_eid = ieee80211_bss_get_ie(bss, WLAN_EID_SSID);
	if (ssid_eid)
		pos += cabrio_add_ssid_sss(pos, ssid_eid + 2, ssid_eid[1]);
	else
		cabrio_dbg_assoc("no SSID\n");

	/* add DS param SSS */
	if (bss->channel)
		pos += cabrio_add_channel_sss(pos, bss->channel->hw_value);
	else
		cabrio_dbg_assoc("no channel\n");

	/* add (empty) CF param SSS */
	pos += cabrio_add_cf_param_sss(pos);

	/* add rates SSS */
	tmp = pos + 4; /* skip Marvell IE header */
	pos += cabrio_add_common_rates_sss(pos, bss);
	cabrio_dbg_hex(CABRIO_DBG_ASSOC, "Common Rates", tmp, pos - tmp);

	/* add auth type SSS */
	if (CABRIO_FW_MAJOR_REV(priv->fwrelease) >= 9)
		pos += cabrio_add_auth_type_sss(pos, sme->auth_type);

	/* add WPA/WPA2 SSS */
	if (sme->ie && sme->ie_len)
		pos += cabrio_add_wpa_sss(pos, sme->ie, sme->ie_len);

	len = (sizeof(*cmd) - sizeof(cmd->iebuf)) +
		(u16)(pos - (u8 *) &cmd->iebuf);
	cmd->hdr.size = cpu_to_le16(len);

	cabrio_dbg_hex(CABRIO_DBG_ASSOC, "ASSOC_CMD", (u8 *) cmd,
			le16_to_cpu(cmd->hdr.size));

	/* store for later use */
	memcpy(priv->assoc_bss, bss->bssid, ETH_ALEN);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_ASSOCIATE, cmd);
	if (ret)
		goto done;

	/* generate connect message to cfg80211 */

	resp = (void *) cmd; /* recast for easier field access */
	status = le16_to_cpu(resp->statuscode);

	/* Older FW versions map the IEEE 802.11 Status Code in the association
	 * response to the following values returned in resp->statuscode:
	 *
	 *    IEEE Status Code                Marvell Status Code
	 *    0                       ->      0x0000 ASSOC_RESULT_SUCCESS
	 *    13                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
	 *    14                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
	 *    15                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
	 *    16                      ->      0x0004 ASSOC_RESULT_AUTH_REFUSED
	 *    others                  ->      0x0003 ASSOC_RESULT_REFUSED
	 *
	 * Other response codes:
	 *    0x0001 -> ASSOC_RESULT_INVALID_PARAMETERS (unused)
	 *    0x0002 -> ASSOC_RESULT_TIMEOUT (internal timer expired waiting for
	 *                                    association response from the AP)
	 */
	if (CABRIO_FW_MAJOR_REV(priv->fwrelease) <= 8) {
		switch (status) {
		case 0:
			break;
		case 1:
			cabrio_dbg_assoc("invalid association parameters\n");
			status = WLAN_STATUS_CAPS_UNSUPPORTED;
			break;
		case 2:
			cabrio_dbg_assoc("timer expired while waiting for AP\n");
			status = WLAN_STATUS_AUTH_TIMEOUT;
			break;
		case 3:
			cabrio_dbg_assoc("association refused by AP\n");
			status = WLAN_STATUS_ASSOC_DENIED_UNSPEC;
			break;
		case 4:
			cabrio_dbg_assoc("authentication refused by AP\n");
			status = WLAN_STATUS_UNKNOWN_AUTH_TRANSACTION;
			break;
		default:
			cabrio_dbg_assoc("association failure %d\n", status);
			/* v5 OLPC firmware does return the AP status code if
			 * it's not one of the values above.  Let that through.
			 */
			break;
		}
	}

	cabrio_dbg_assoc("status %d, statuscode 0x%04x, capability 0x%04x, "
		      "aid 0x%04x\n", status, le16_to_cpu(resp->statuscode),
		      le16_to_cpu(resp->capability), le16_to_cpu(resp->aid));

	resp_ie_len = le16_to_cpu(resp->hdr.size)
		- sizeof(resp->hdr)
		- 6;
	cfg80211_connect_result(priv->dev,
				priv->assoc_bss,
				sme->ie, sme->ie_len,
				resp->iebuf, resp_ie_len,
				status,
				GFP_KERNEL);

	if (status == 0) {
		/* TODO: get rid of priv->connect_status */
		priv->connect_status = CABRIO_CONNECTED;
		netif_carrier_on(priv->dev);
		if (!priv->tx_pending_len)
			netif_tx_wake_all_queues(priv->dev);
	}

done:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}
#endif // 0

static struct cfg80211_scan_request *
_new_connect_scan_req(struct wiphy *wiphy, struct cfg80211_connect_params *sme)
{
	struct cfg80211_scan_request *creq = NULL;
	int i, n_channels = 0;
	enum ieee80211_band band;

	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		if (wiphy->bands[band])
			n_channels += wiphy->bands[band]->n_channels;
	}

	creq = kzalloc(sizeof(*creq) + sizeof(struct cfg80211_ssid) +
		       n_channels * sizeof(void *),
		       GFP_ATOMIC);
	if (!creq)
		return NULL;

	/* SSIDs come after channels */
	creq->ssids = (void *)&creq->channels[n_channels];
	creq->n_channels = n_channels;
	creq->n_ssids = 1;

	/* Scan all available channels */
	i = 0;
	for (band = 0; band < IEEE80211_NUM_BANDS; band++) {
		int j;

		if (!wiphy->bands[band])
			continue;

		for (j = 0; j < wiphy->bands[band]->n_channels; j++) {
			/* ignore disabled channels */
			if (wiphy->bands[band]->channels[j].flags &
						IEEE80211_CHAN_DISABLED)
				continue;

			creq->channels[i] = &wiphy->bands[band]->channels[j];
			i++;
		}
	}
	if (i) {
		/* Set real number of channels specified in creq->channels[] */
		creq->n_channels = i;

		/* Scan for the SSID we're going to connect to */
		memcpy(creq->ssids[0].ssid, sme->ssid, sme->ssid_len);
		creq->ssids[0].ssid_len = sme->ssid_len;
	} else {
		/* No channels found... */
		kfree(creq);
		creq = NULL;
	}

	return creq;
}


#if 0
static int cabrio_cfg_connect_orig(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *sme)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	struct cfg80211_bss *bss = NULL;
	int ret = 0;
	u8 preamble = RADIO_PREAMBLE_SHORT;
	// SSV --
	#if 0
	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (!sme->bssid) {
		struct cfg80211_scan_request *creq;

		/*
		 * Scan for the requested network after waiting for existing
		 * scans to finish.
		 */
		cabrio_dbg_assoc("assoc: waiting for existing scans\n");
		wait_event_interruptible_timeout(priv->scan_q,
						 (priv->scan_req == NULL),
						 (15 * HZ));

		creq = _new_connect_scan_req(wiphy, sme);
		if (!creq) {
			ret = -EINVAL;
			goto done;
		}

		cabrio_dbg_assoc("assoc: scanning for compatible AP\n");
		_internal_start_scan(priv, true, creq);

		cabrio_dbg_assoc("assoc: waiting for scan to complete\n");
		wait_event_interruptible_timeout(priv->scan_q,
						 (priv->scan_req == NULL),
						 (15 * HZ));
		cabrio_dbg_assoc("assoc: scanning competed\n");
	}

	/* Find the BSS we want using available scan results */
	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid,
		sme->ssid, sme->ssid_len,
		WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);
	if (!bss) {
		wiphy_err(wiphy, "assoc: bss %pM not in scan results\n",
			  sme->bssid);
		ret = -ENOENT;
		goto done;
	}
	cabrio_dbg_assoc("trying %pM\n", bss->bssid);
	cabrio_dbg_assoc("cipher 0x%x, key index %d, key len %d\n",
		      sme->crypto.cipher_group,
		      sme->key_idx, sme->key_len);

	/* As this is a new connection, clear locally stored WEP keys */
	priv->wep_tx_key = 0;
	memset(priv->wep_key, 0, sizeof(priv->wep_key));
	memset(priv->wep_key_len, 0, sizeof(priv->wep_key_len));

	/* set/remove WEP keys */
	switch (sme->crypto.cipher_group) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		/* Store provided WEP keys in priv-> */
		priv->wep_tx_key = sme->key_idx;
		priv->wep_key_len[sme->key_idx] = sme->key_len;
		memcpy(priv->wep_key[sme->key_idx], sme->key, sme->key_len);
		/* Set WEP keys and WEP mode */
		cabrio_set_wep_keys(priv);
		priv->mac_control |= CMD_ACT_MAC_WEP_ENABLE;
		cabrio_set_mac_control(priv);
		/* No RSN mode for WEP */
		cabrio_enable_rsn(priv, 0);
		break;
	case 0: /* there's no WLAN_CIPHER_SUITE_NONE definition */
		/*
		 * If we don't have no WEP, no WPA and no WPA2,
		 * we remove all keys like in the WPA/WPA2 setup,
		 * we just don't set RSN.
		 *
		 * Therefore: fall-through
		 */
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
		/* Remove WEP keys and WEP mode */
		cabrio_remove_wep_keys(priv);
		priv->mac_control &= ~CMD_ACT_MAC_WEP_ENABLE;
		cabrio_set_mac_control(priv);

		/* clear the WPA/WPA2 keys */
		cabrio_set_key_material(priv,
			KEY_TYPE_ID_WEP, /* doesn't matter */
			KEY_INFO_WPA_UNICAST,
			NULL, 0);
		cabrio_set_key_material(priv,
			KEY_TYPE_ID_WEP, /* doesn't matter */
			KEY_INFO_WPA_MCAST,
			NULL, 0);
		/* RSN mode for WPA/WPA2 */
		cabrio_enable_rsn(priv, sme->crypto.cipher_group != 0);
		break;
	default:
		wiphy_err(wiphy, "unsupported cipher group 0x%x\n",
			  sme->crypto.cipher_group);
		ret = -ENOTSUPP;
		goto done;
	}

	cabrio_set_authtype(priv, sme);
	cabrio_set_radio(priv, preamble, 1);

	/* Do the actual association */
	ret = cabrio_associate(priv, bss, sme);

 done:
	if (bss)
		cfg80211_put_bss(bss);
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}
#endif // 0

static int cabrio_cfg_connect(struct wiphy *wiphy, struct net_device *dev,
                              struct cfg80211_connect_params *sme)
{
    //struct cfg_80211_ssid   *ssid;
    //s32                      size, i;
    //struct cfg_join_request *JoinReq;
    //const char *sec_name;

    struct cabrio_private *priv = wiphy_priv(wiphy);
    struct cfg80211_bss *bss = NULL;
    int ret = 0;

    cabrio_dbg_enter(CABRIO_DBG_CFG80211);
    
    if (!sme->bssid) {
        struct cfg80211_scan_request *creq;
    
        /*
         * Scan for the requested network after waiting for existing
         * scans to finish.
         */
        cabrio_dbg_assoc("assoc: waiting for existing scans\n");
        wait_event_interruptible_timeout(priv->scan_q,
                                         (priv->scan_req == NULL),
                                         (15 * HZ));
    
        creq = _new_connect_scan_req(wiphy, sme);
        if (!creq) {
            ret = -EINVAL;
            goto done;
        }
    
        cabrio_dbg_assoc("assoc: scanning for compatible AP\n");
        _internal_start_scan(priv, true, creq);
    
        cabrio_dbg_assoc("assoc: waiting for scan to complete\n");
        wait_event_interruptible_timeout(priv->scan_q,
                                         (priv->scan_req == NULL),
                                         (15 * HZ));
        cabrio_dbg_assoc("assoc: scanning competed\n");
    }
    
    /* Find the BSS we want using available scan results */
    bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid,
                           sme->ssid, sme->ssid_len,
                           WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);
    if (!bss) {
        wiphy_err(wiphy, "assoc: bss %pM not in scan results\n",
                  sme->bssid);
        ret = -ENOENT;
        goto done;
    }

    cabrio_dbg_assoc("trying %pM\n", bss->bssid);
    cabrio_dbg_assoc("cipher 0x%x, key index %d, key len %d\n",
                     sme->crypto.cipher_group,
                     sme->key_idx, sme->key_len);
    /* Switch channel to BSS's */
    cabrio_set_channel(priv, bss->channel->hw_value);

    /* As this is a new connection, clear locally stored WEP keys */
    priv->wep_tx_key = 0;
    memset(priv->wep_key, 0, sizeof(priv->wep_key));
    memset(priv->wep_key_len, 0, sizeof(priv->wep_key_len));
    
    /* set/remove WEP keys */
    switch (sme->crypto.cipher_group) {
        case WLAN_CIPHER_SUITE_WEP40:
        case WLAN_CIPHER_SUITE_WEP104:
            /* Store provided WEP keys in priv-> */
            priv->wep_tx_key = sme->key_idx;
            priv->wep_key_len[sme->key_idx] = sme->key_len;
            memcpy(priv->wep_key[sme->key_idx], sme->key, sme->key_len);
           break;
       case WLAN_CIPHER_SUITE_TKIP:
       case WLAN_CIPHER_SUITE_CCMP:
       case 0: /* there's no WLAN_CIPHER_SUITE_NONE definition */
           break;
       default:
           wiphy_err(wiphy, "unsupported cipher group 0x%x\n",
                 sme->crypto.cipher_group);
           ret = -ENOTSUPP;
           goto done;
       }
    
    /* Do the actual association */
    ret = cabrio_associate(priv, bss, sme);
    
done:
    if (bss)
        cfg80211_put_bss(bss);
    cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
    return ret;
}


#if 0
static int cabrio_cfg_connect_orig(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_connect_params *sme)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	struct cfg80211_bss *bss = NULL;
	int ret = 0;
	u8 preamble = RADIO_PREAMBLE_SHORT;
	// SSV --
	#if 0
	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (!sme->bssid) {
		struct cfg80211_scan_request *creq;

		/*
		 * Scan for the requested network after waiting for existing
		 * scans to finish.
		 */
		cabrio_dbg_assoc("assoc: waiting for existing scans\n");
		wait_event_interruptible_timeout(priv->scan_q,
						 (priv->scan_req == NULL),
						 (15 * HZ));

		creq = _new_connect_scan_req(wiphy, sme);
		if (!creq) {
			ret = -EINVAL;
			goto done;
		}

		cabrio_dbg_assoc("assoc: scanning for compatible AP\n");
		_internal_start_scan(priv, true, creq);

		cabrio_dbg_assoc("assoc: waiting for scan to complete\n");
		wait_event_interruptible_timeout(priv->scan_q,
						 (priv->scan_req == NULL),
						 (15 * HZ));
		cabrio_dbg_assoc("assoc: scanning competed\n");
	}

	/* Find the BSS we want using available scan results */
	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid,
		sme->ssid, sme->ssid_len,
		WLAN_CAPABILITY_ESS, WLAN_CAPABILITY_ESS);
	if (!bss) {
		wiphy_err(wiphy, "assoc: bss %pM not in scan results\n",
			  sme->bssid);
		ret = -ENOENT;
		goto done;
	}
	cabrio_dbg_assoc("trying %pM\n", bss->bssid);
	cabrio_dbg_assoc("cipher 0x%x, key index %d, key len %d\n",
		      sme->crypto.cipher_group,
		      sme->key_idx, sme->key_len);

	/* As this is a new connection, clear locally stored WEP keys */
	priv->wep_tx_key = 0;
	memset(priv->wep_key, 0, sizeof(priv->wep_key));
	memset(priv->wep_key_len, 0, sizeof(priv->wep_key_len));

	/* set/remove WEP keys */
	switch (sme->crypto.cipher_group) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		/* Store provided WEP keys in priv-> */
		priv->wep_tx_key = sme->key_idx;
		priv->wep_key_len[sme->key_idx] = sme->key_len;
		memcpy(priv->wep_key[sme->key_idx], sme->key, sme->key_len);
		/* Set WEP keys and WEP mode */
		cabrio_set_wep_keys(priv);
		priv->mac_control |= CMD_ACT_MAC_WEP_ENABLE;
		cabrio_set_mac_control(priv);
		/* No RSN mode for WEP */
		cabrio_enable_rsn(priv, 0);
		break;
	case 0: /* there's no WLAN_CIPHER_SUITE_NONE definition */
		/*
		 * If we don't have no WEP, no WPA and no WPA2,
		 * we remove all keys like in the WPA/WPA2 setup,
		 * we just don't set RSN.
		 *
		 * Therefore: fall-through
		 */
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
		/* Remove WEP keys and WEP mode */
		cabrio_remove_wep_keys(priv);
		priv->mac_control &= ~CMD_ACT_MAC_WEP_ENABLE;
		cabrio_set_mac_control(priv);

		/* clear the WPA/WPA2 keys */
		cabrio_set_key_material(priv,
			KEY_TYPE_ID_WEP, /* doesn't matter */
			KEY_INFO_WPA_UNICAST,
			NULL, 0);
		cabrio_set_key_material(priv,
			KEY_TYPE_ID_WEP, /* doesn't matter */
			KEY_INFO_WPA_MCAST,
			NULL, 0);
		/* RSN mode for WPA/WPA2 */
		cabrio_enable_rsn(priv, sme->crypto.cipher_group != 0);
		break;
	default:
		wiphy_err(wiphy, "unsupported cipher group 0x%x\n",
			  sme->crypto.cipher_group);
		ret = -ENOTSUPP;
		goto done;
	}

	cabrio_set_authtype(priv, sme);
	cabrio_set_radio(priv, preamble, 1);

	/* Do the actual association */
	ret = cabrio_associate(priv, bss, sme);

 done:
	if (bss)
		cfg80211_put_bss(bss);
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}
#endif // 0

int cabrio_disconnect(struct cabrio_private *priv, u16 reason)
{
    u32                          size =   HOST_CMD_HDR_LEN 
                                        + sizeof(struct cfg_leave_request);
    u32                          event_size = sizeof(HDR_HostEvent) + sizeof(struct resp_evt_result);
    u8                          *buf = kzalloc(size + event_size, GFP_KERNEL);
    HDR_HostCmd                 *cmd = (HDR_HostCmd *)buf;
    struct cfg_leave_request    *leave_req = (struct cfg_leave_request *)&cmd->dat8[0];
    HDR_HostEvent               *event = (HDR_HostEvent *)(buf + size);
    struct resp_evt_result      *leave_resp = (struct resp_evt_result  *)event->dat;
    int                          ret = -ENOMEM;

    if (buf == NULL)
    	goto disconnect;

	memset(cmd, 0, size);
    cmd->c_type = HOST_CMD;
    cmd->h_cmd = SSV_HOST_CMD_LEAVE;
    cmd->len = size;
    leave_req->reason = reason;

    event->len = event_size;
	//const u8 *ssid_eid;
    //const char *sec_name;

	ret = cabrio_cmd_with_response(priv, SSV_HOST_CMD_LEAVE, cmd, event);

	if (ret)
		goto disconnect;

    if (leave_resp->result == CMD_OK) {
    	cabrio_dbg_cfg80211("Leave with response  %d.\n", leave_resp->u.leave.reason_code);
    } else {
    	cabrio_dbg_cfg80211("Leave with error %d.\n", leave_resp->result);
    }
    kfree(cmd);

disconnect:
    cfg80211_disconnected(priv->dev,
			reason,
			NULL, 0,
			GFP_KERNEL);
    priv->is_securied_connected = 0;
    priv->connect_status = CABRIO_DISCONNECTED;

	return ret;
}


#if ORIG
int cabrio_disconnect_orig(struct cabrio_private *priv, u16 reason)
{
	struct cmd_ds_802_11_deauthenticate cmd;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	/* Mildly ugly to use a locally store my own BSSID ... */
	memcpy(cmd.macaddr, &priv->assoc_bss, ETH_ALEN);
	cmd.reasoncode = cpu_to_le16(reason);

	ret = cabrio_cmd_with_response(priv, CMD_802_11_DEAUTHENTICATE, &cmd);
	if (ret)
		return ret;

	cfg80211_disconnected(priv->dev,
			reason,
			NULL, 0,
			GFP_KERNEL);
	priv->connect_status = CABRIO_DISCONNECTED;

	return 0;
}
#endif
static int cabrio_cfg_disconnect(struct wiphy *wiphy, struct net_device *dev,
                                 u16 reason_code)
{
#if 0
	CABRIO_TODO(__FUNCTION__);
	return 0;
#else
	int ret;
	struct cabrio_private *priv = wiphy_priv(wiphy);
	// SSV --
	#if 0
	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter_args(CABRIO_DBG_CFG80211, "reason_code %d", reason_code);

	/* store for cabrio_cfg_ret_disconnect() */
	priv->disassoc_reason = reason_code;

	ret = cabrio_disconnect(priv, reason_code);

	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
#endif
}

static int cabrio_cfg_set_default_key(struct wiphy *wiphy,
				   struct net_device *netdev,
				   u8 key_index, bool unicast,
				   bool multicast)
{
#if 1
	CABRIO_TODO(__FUNCTION__);
#else
	struct cabrio_private *priv = wiphy_priv(wiphy);
	// SSV --
	#if 0
	if (netdev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (key_index != priv->wep_tx_key) {
		cabrio_dbg_assoc("set_default_key: to %d\n", key_index);
		priv->wep_tx_key = key_index;
		cabrio_set_wep_keys(priv);
	}
#endif
	return 0;
}


static int cabrio_cfg_add_key(struct wiphy *wiphy, struct net_device *netdev,
			   u8 idx, bool pairwise, const u8 *mac_addr,
			   struct key_params *params)
{
#if 0
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);
	CABRIO_TODO(__FUNCTION__);
	return 0;
#else
	struct cabrio_private *priv = wiphy_priv(wiphy);
	u16 key_type;
	int ret = 0;
	// SSV -
	#if 0
	if (netdev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	cabrio_dbg_assoc("add_key: cipher 0x%x, mac_addr %pM\n",
		      params->cipher, mac_addr);
	cabrio_dbg_assoc("add_key: key index %d, key len %d\n",
		      idx, params->key_len);
	if (params->key_len)
		cabrio_dbg_hex(CABRIO_DBG_CFG80211, "KEY",
			    params->key, params->key_len);

	cabrio_dbg_assoc("add_key: seq len %d\n", params->seq_len);
	if (params->seq_len)
		cabrio_dbg_hex(CABRIO_DBG_CFG80211, "SEQ",
			    params->seq, params->seq_len);

	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		/* actually compare if something has changed ... */
		if ((priv->wep_key_len[idx] != params->key_len) ||
			memcmp(priv->wep_key[idx],
			       params->key, params->key_len) != 0) {
			priv->wep_key_len[idx] = params->key_len;
			memcpy(priv->wep_key[idx],
			       params->key, params->key_len);
			cabrio_set_wep_keys(priv);
		}
		priv->is_securied_connected = true;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
		key_type = (params->cipher == WLAN_CIPHER_SUITE_TKIP)
			? KEY_TYPE_ID_TKIP
			: KEY_TYPE_ID_AES;
		cabrio_set_key_material(priv,
				     key_type,
				     idx,
				     params->key, params->key_len);
		priv->is_securied_connected = true;
		break;
	default:
		wiphy_err(wiphy, "unhandled cipher 0x%x\n", params->cipher);
		ret = -ENOTSUPP;
		break;
	}

	return ret;
#endif
}


static int cabrio_cfg_del_key(struct wiphy *wiphy, struct net_device *netdev,
			   u8 key_index, bool pairwise, const u8 *mac_addr)
{

#if 1
	CABRIO_TODO(__FUNCTION__);
#else
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	cabrio_dbg_assoc("del_key: key_idx %d, mac_addr %pM\n",
		      key_index, mac_addr);

#ifdef TODO
	struct cabrio_private *priv = wiphy_priv(wiphy);
	/*
	 * I think can keep this a NO-OP, because:

	 * - we clear all keys whenever we do cabrio_cfg_connect() anyway
	 * - neither "iw" nor "wpa_supplicant" won't call this during
	 *   an ongoing connection
	 * - TODO: but I have to check if this is still true when
	 *   I set the AP to periodic re-keying
	 * - we've not kzallec() something when we've added a key at
	 *   cabrio_cfg_connect() or cabrio_cfg_add_key().
	 *
	 * This causes cabrio_cfg_del_key() only called at disconnect time,
	 * where we'd just waste time deleting a key that is not going
	 * to be used anyway.
	 */
	if (key_index < 3 && priv->wep_key_len[key_index]) {
		priv->wep_key_len[key_index] = 0;
		cabrio_set_wep_keys(priv);
	}
#endif

#endif
	return 0;

}


/*
 * Get station
 */

static int cabrio_cfg_get_station(struct wiphy *wiphy, struct net_device *dev,
                                  u8 *mac, struct station_info *sinfo)
{
#if 1
	CABRIO_TODO(__FUNCTION__);
	return 0;
#else
	struct cabrio_private *priv = wiphy_priv(wiphy);
    s8 signal, noise;
    int ret;
    size_t i;

    cabrio_dbg_enter(CABRIO_DBG_CFG80211);

    sinfo->filled |= STATION_INFO_TX_BYTES |
                     STATION_INFO_TX_PACKETS |
                     STATION_INFO_RX_BYTES |
                     STATION_INFO_RX_PACKETS;
	sinfo->tx_bytes = priv->dev->stats.tx_bytes;
	sinfo->tx_packets = priv->dev->stats.tx_packets;
	sinfo->rx_bytes = priv->dev->stats.rx_bytes;
	sinfo->rx_packets = priv->dev->stats.rx_packets;

	/* Get current RSSI */
	ret = cabrio_get_rssi(priv, &signal, &noise);
	if (ret == 0) {
		sinfo->signal = signal;
		sinfo->filled |= STATION_INFO_SIGNAL;
	}

	/* Convert priv->cur_rate from hw_value to NL80211 value */
	for (i = 0; i < ARRAY_SIZE(cabrio_rates); i++) {
		if (priv->cur_rate == cabrio_rates[i].hw_value) {
			sinfo->txrate.legacy = cabrio_rates[i].bitrate;
			sinfo->filled |= STATION_INFO_TX_BITRATE;
			break;
		}
	}
	return 0;
#endif
}




/*
 * "Site survey", here just current channel and noise level
 */

static int cabrio_get_survey(struct wiphy *wiphy, struct net_device *dev,
	int idx, struct survey_info *survey)
{
#if 1
	CABRIO_TODO(__FUNCTION__);
	return 0;
#else
	struct cabrio_private *priv = wiphy_priv(wiphy);
	s8 signal, noise;
	int ret;

	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;

	if (idx != 0)
		ret = -ENOENT;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	survey->channel = ieee80211_get_channel(wiphy,
		ieee80211_channel_to_frequency(priv->channel,
					       IEEE80211_BAND_2GHZ));

	ret = cabrio_get_rssi(priv, &signal, &noise);
	if (ret == 0) {
		survey->filled = SURVEY_INFO_NOISE_DBM;
		survey->noise = noise;
	}

	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
#endif
}




/*
 * Change interface
 */

static int cabrio_change_intf(struct wiphy *wiphy, struct net_device *dev,
	enum nl80211_iftype type, u32 *flags,
	       struct vif_params *params)
{
#if 1
	CABRIO_TODO(__FUNCTION__);
	return 0;
#else
	struct cabrio_private *priv = wiphy_priv(wiphy);
	int ret = 0;
	// SSV --
	#if 0
	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	switch (type) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_ADHOC:
		break;
	default:
		return -EOPNOTSUPP;
	}

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (priv->iface_running)
		ret = cabrio_set_iface_type(priv, type);

	if (!ret)
		priv->wdev->iftype = type;

	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
#endif
}

// Cabrio does not support IBSS
#if 0
/*
 * IBSS (Ad-Hoc)
 */

/*
 * The firmware needs the following bits masked out of the beacon-derived
 * capability field when associating/joining to a BSS:
 *  9 (QoS), 11 (APSD), 12 (unused), 14 (unused), 15 (unused)
 */
#define CAPINFO_MASK (~(0xda00))


static void cabrio_join_post(struct cabrio_private *priv,
			  struct cfg80211_ibss_params *params,
			  u8 *bssid, u16 capability)
{
	u8 fake_ie[2 + IEEE80211_MAX_SSID_LEN + /* ssid */
		   2 + 4 +                      /* basic rates */
		   2 + 1 +                      /* DS parameter */
		   2 + 2 +                      /* atim */
		   2 + 8];                      /* extended rates */
	u8 *fake = fake_ie;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	/*
	 * For cfg80211_inform_bss, we'll need a fake IE, as we can't get
	 * the real IE from the firmware. So we fabricate a fake IE based on
	 * what the firmware actually sends (sniffed with wireshark).
	 */
	/* Fake SSID IE */
	*fake++ = WLAN_EID_SSID;
	*fake++ = params->ssid_len;
	memcpy(fake, params->ssid, params->ssid_len);
	fake += params->ssid_len;
	/* Fake supported basic rates IE */
	*fake++ = WLAN_EID_SUPP_RATES;
	*fake++ = 4;
	*fake++ = 0x82;
	*fake++ = 0x84;
	*fake++ = 0x8b;
	*fake++ = 0x96;
	/* Fake DS channel IE */
	*fake++ = WLAN_EID_DS_PARAMS;
	*fake++ = 1;
	*fake++ = params->channel->hw_value;
	/* Fake IBSS params IE */
	*fake++ = WLAN_EID_IBSS_PARAMS;
	*fake++ = 2;
	*fake++ = 0; /* ATIM=0 */
	*fake++ = 0;
	/* Fake extended rates IE, TODO: don't add this for 802.11b only,
	 * but I don't know how this could be checked */
	*fake++ = WLAN_EID_EXT_SUPP_RATES;
	*fake++ = 8;
	*fake++ = 0x0c;
	*fake++ = 0x12;
	*fake++ = 0x18;
	*fake++ = 0x24;
	*fake++ = 0x30;
	*fake++ = 0x48;
	*fake++ = 0x60;
	*fake++ = 0x6c;
	cabrio_dbg_hex(CABRIO_DBG_CFG80211, "IE", fake_ie, fake - fake_ie);

	cfg80211_inform_bss(priv->wdev->wiphy,
			    params->channel,
			    bssid,
			    0,
			    capability,
			    params->beacon_interval,
			    fake_ie, fake - fake_ie,
			    0, GFP_KERNEL);

	memcpy(priv->wdev->ssid, params->ssid, params->ssid_len);
	priv->wdev->ssid_len = params->ssid_len;

	cfg80211_ibss_joined(priv->dev, bssid, GFP_KERNEL);

	/* TODO: consider doing this at MACREG_INT_CODE_LINK_SENSED time */
	priv->connect_status = CABRIO_CONNECTED;
	netif_carrier_on(priv->dev);
	if (!priv->tx_pending_len)
		netif_wake_queue(priv->dev);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
}

static int cabrio_ibss_join_existing(struct cabrio_private *priv,
	struct cfg80211_ibss_params *params,
	struct cfg80211_bss *bss)
{
	const u8 *rates_eid = ieee80211_bss_get_ie(bss, WLAN_EID_SUPP_RATES);
	struct cmd_ds_802_11_ad_hoc_join cmd;
	u8 preamble = RADIO_PREAMBLE_SHORT;
	int ret = 0;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	/* TODO: set preamble based on scan result */
	ret = cabrio_set_radio(priv, preamble, 1);
	if (ret)
		goto out;

	/*
	 * Example CMD_802_11_AD_HOC_JOIN command:
	 *
	 * command         2c 00         CMD_802_11_AD_HOC_JOIN
	 * size            65 00
	 * sequence        xx xx
	 * result          00 00
	 * bssid           02 27 27 97 2f 96
	 * ssid            49 42 53 53 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 * type            02            CMD_BSS_TYPE_IBSS
	 * beacon period   64 00
	 * dtim period     00
	 * timestamp       00 00 00 00 00 00 00 00
	 * localtime       00 00 00 00 00 00 00 00
	 * IE DS           03
	 * IE DS len       01
	 * IE DS channel   01
	 * reserveed       00 00 00 00
	 * IE IBSS         06
	 * IE IBSS len     02
	 * IE IBSS atim    00 00
	 * reserved        00 00 00 00
	 * capability      02 00
	 * rates           82 84 8b 96 0c 12 18 24 30 48 60 6c 00
	 * fail timeout    ff 00
	 * probe delay     00 00
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));

	memcpy(cmd.bss.bssid, bss->bssid, ETH_ALEN);
	memcpy(cmd.bss.ssid, params->ssid, params->ssid_len);
	cmd.bss.type = CMD_BSS_TYPE_IBSS;
	cmd.bss.beaconperiod = cpu_to_le16(params->beacon_interval);
	cmd.bss.ds.header.id = WLAN_EID_DS_PARAMS;
	cmd.bss.ds.header.len = 1;
	cmd.bss.ds.channel = params->channel->hw_value;
	cmd.bss.ibss.header.id = WLAN_EID_IBSS_PARAMS;
	cmd.bss.ibss.header.len = 2;
	cmd.bss.ibss.atimwindow = 0;
	cmd.bss.capability = cpu_to_le16(bss->capability & CAPINFO_MASK);

	/* set rates to the intersection of our rates and the rates in the
	   bss */
	if (!rates_eid) {
		cabrio_add_rates(cmd.bss.rates);
	} else {
		int hw, i;
		u8 rates_max = rates_eid[1];
		u8 *rates = cmd.bss.rates;
		for (hw = 0; hw < ARRAY_SIZE(cabrio_rates); hw++) {
			u8 hw_rate = cabrio_rates[hw].bitrate / 5;
			for (i = 0; i < rates_max; i++) {
				if (hw_rate == (rates_eid[i+2] & 0x7f)) {
					u8 rate = rates_eid[i+2];
					if (rate == 0x02 || rate == 0x04 ||
					    rate == 0x0b || rate == 0x16)
						rate |= 0x80;
					*rates++ = rate;
				}
			}
		}
	}

	/* Only v8 and below support setting this */
	if (CABRIO_FW_MAJOR_REV(priv->fwrelease) <= 8) {
		cmd.failtimeout = cpu_to_le16(CABRIO_ASSOCIATION_TIME_OUT);
		cmd.probedelay = cpu_to_le16(CMD_SCAN_PROBE_DELAY_TIME);
	}
	ret = cabrio_cmd_with_response(priv, CMD_802_11_AD_HOC_JOIN, &cmd);
	if (ret)
		goto out;

	/*
	 * This is a sample response to CMD_802_11_AD_HOC_JOIN:
	 *
	 * response        2c 80
	 * size            09 00
	 * sequence        xx xx
	 * result          00 00
	 * reserved        00
	 */
	cabrio_join_post(priv, params, bss->bssid, bss->capability);

 out:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}



static int cabrio_ibss_start_new(struct cabrio_private *priv,
	struct cfg80211_ibss_params *params)
{
	struct cmd_ds_802_11_ad_hoc_start cmd;
	struct cmd_ds_802_11_ad_hoc_result *resp =
		(struct cmd_ds_802_11_ad_hoc_result *) &cmd;
	u8 preamble = RADIO_PREAMBLE_SHORT;
	int ret = 0;
	u16 capability;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	ret = cabrio_set_radio(priv, preamble, 1);
	if (ret)
		goto out;

	/*
	 * Example CMD_802_11_AD_HOC_START command:
	 *
	 * command         2b 00         CMD_802_11_AD_HOC_START
	 * size            b1 00
	 * sequence        xx xx
	 * result          00 00
	 * ssid            54 45 53 54 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 *                 00 00 00 00 00 00 00 00
	 * bss type        02
	 * beacon period   64 00
	 * dtim period     00
	 * IE IBSS         06
	 * IE IBSS len     02
	 * IE IBSS atim    00 00
	 * reserved        00 00 00 00
	 * IE DS           03
	 * IE DS len       01
	 * IE DS channel   01
	 * reserved        00 00 00 00
	 * probe delay     00 00
	 * capability      02 00
	 * rates           82 84 8b 96   (basic rates with have bit 7 set)
	 *                 0c 12 18 24 30 48 60 6c
	 * padding         100 bytes
	 */
	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	memcpy(cmd.ssid, params->ssid, params->ssid_len);
	cmd.bsstype = CMD_BSS_TYPE_IBSS;
	cmd.beaconperiod = cpu_to_le16(params->beacon_interval);
	cmd.ibss.header.id = WLAN_EID_IBSS_PARAMS;
	cmd.ibss.header.len = 2;
	cmd.ibss.atimwindow = 0;
	cmd.ds.header.id = WLAN_EID_DS_PARAMS;
	cmd.ds.header.len = 1;
	cmd.ds.channel = params->channel->hw_value;
	/* Only v8 and below support setting probe delay */
	if (CABRIO_FW_MAJOR_REV(priv->fwrelease) <= 8)
		cmd.probedelay = cpu_to_le16(CMD_SCAN_PROBE_DELAY_TIME);
	/* TODO: mix in WLAN_CAPABILITY_PRIVACY */
	capability = WLAN_CAPABILITY_IBSS;
	cmd.capability = cpu_to_le16(capability);
	cabrio_add_rates(cmd.rates);


	ret = cabrio_cmd_with_response(priv, CMD_802_11_AD_HOC_START, &cmd);
	if (ret)
		goto out;

	/*
	 * This is a sample response to CMD_802_11_AD_HOC_JOIN:
	 *
	 * response        2b 80
	 * size            14 00
	 * sequence        xx xx
	 * result          00 00
	 * reserved        00
	 * bssid           02 2b 7b 0f 86 0e
	 */
	cabrio_join_post(priv, params, resp->bssid, capability);

 out:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}


static int cabrio_join_ibss(struct wiphy *wiphy, struct net_device *dev,
		struct cfg80211_ibss_params *params)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	int ret = 0;
	struct cfg80211_bss *bss;
	DECLARE_SSID_BUF(ssid_buf);
	// SSV --
	#if 0
	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;
	#endif
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (!params->channel) {
		ret = -ENOTSUPP;
		goto out;
	}

	ret = cabrio_set_channel(priv, params->channel->hw_value);
	if (ret)
		goto out;

	/* Search if someone is beaconing. This assumes that the
	 * bss list is populated already */
	bss = cfg80211_get_bss(wiphy, params->channel, params->bssid,
		params->ssid, params->ssid_len,
		WLAN_CAPABILITY_IBSS, WLAN_CAPABILITY_IBSS);

	if (bss) {
		ret = cabrio_ibss_join_existing(priv, params, bss);
		cfg80211_put_bss(bss);
	} else
		ret = cabrio_ibss_start_new(priv, params);


 out:
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}


static int cabrio_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	struct cmd_ds_802_11_ad_hoc_stop cmd;
	int ret = 0;

	if (dev == priv->mesh_dev)
		return -EOPNOTSUPP;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.size = cpu_to_le16(sizeof(cmd));
	ret = cabrio_cmd_with_response(priv, CMD_802_11_AD_HOC_STOP, &cmd);

	/* TODO: consider doing this at MACREG_INT_CODE_ADHOC_BCN_LOST time */
	cabrio_mac_event_disconnected(priv);

	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}
#endif // Cabrio does not support IBSS


static int	cabrio_set_pmksa(struct wiphy *wiphy, struct net_device *netdev,
		             struct cfg80211_pmksa *pmksa)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);
	CABRIO_TODO(__func__);
	return 0;
}


int	cabrio_del_pmksa(struct wiphy *wiphy, struct net_device *netdev,
		             struct cfg80211_pmksa *pmksa)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);
	CABRIO_TODO(__func__);
	return 0;
}


int	cabrio_flush_pmksa(struct wiphy *wiphy, struct net_device *netdev)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);
	CABRIO_TODO(__func__);
	return 0;
}


/*
 * Initialization
 */

static struct cfg80211_ops cabrio_cfg80211_ops = {
	.set_channel = cabrio_cfg_set_channel,
	.scan = cabrio_cfg_scan,
	.connect = cabrio_cfg_connect,
	.disconnect = cabrio_cfg_disconnect,
	.add_key = cabrio_cfg_add_key,
	.del_key = cabrio_cfg_del_key,
	.set_default_key = cabrio_cfg_set_default_key,
	.get_station = cabrio_cfg_get_station,
	.dump_survey = cabrio_get_survey,
	.change_virtual_intf = cabrio_change_intf,
	.set_pmksa = cabrio_set_pmksa,
	.del_pmksa = cabrio_del_pmksa,
	.flush_pmksa = cabrio_flush_pmksa,
	// SSV -- .join_ibss = cabrio_join_ibss,
	// SSV -- .leave_ibss = cabrio_leave_ibss,
};


/*
 * At this time cabrio_private *priv doesn't even exist, so we just allocate
 * memory and don't initialize the wiphy further. This is postponed until we
 * can talk to the firmware and happens at registration time in
 * cabrio_cfg_wiphy_register().
 */
struct wireless_dev *cabrio_cfg_alloc(struct device *dev)
{
	int ret = 0;
	struct wireless_dev *wdev;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	wdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (!wdev) {
		dev_err(dev, "cannot allocate wireless device\n");
		return ERR_PTR(-ENOMEM);
	}

	wdev->wiphy = wiphy_new(&cabrio_cfg80211_ops, sizeof(struct cabrio_private));
	if (!wdev->wiphy) {
		dev_err(dev, "cannot allocate wiphy\n");
		ret = -ENOMEM;
		goto err_wiphy_new;
	}

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
	return wdev;

 err_wiphy_new:
	kfree(wdev);
	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ERR_PTR(ret);
}


static void cabrio_cfg_set_regulatory_hint(struct cabrio_private *priv)
{
	struct region_code_mapping {
		const char *cn;
		int code;
	};

	/* Section 5.17.2 */
	static const struct region_code_mapping regmap[] = {
		{"US ", 0x10}, /* US FCC */
		{"CA ", 0x20}, /* Canada */
		{"EU ", 0x30}, /* ETSI   */
		{"ES ", 0x31}, /* Spain  */
		{"FR ", 0x32}, /* France */
		{"JP ", 0x40}, /* Japan  */
	};
	size_t i;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	for (i = 0; i < ARRAY_SIZE(regmap); i++)
		if (regmap[i].code == priv->regioncode) {
			regulatory_hint(priv->wdev->wiphy, regmap[i].cn);
			break;
		}

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
}


/*
 * This function get's called after cabrio_setup_firmware() determined the
 * firmware capabities. So we can setup the wiphy according to our
 * hardware/firmware.
 */
int cabrio_cfg_register(struct cabrio_private *priv)
{
	struct wireless_dev *wdev = priv->wdev;
	int ret;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	wdev->wiphy->max_scan_ssids = 1;
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	wdev->wiphy->interface_modes =
			  BIT(NL80211_IFTYPE_STATION) 
			// SSV - | BIT(NL80211_IFTYPE_ADHOC)
			;
	// SSV --
    #if 0
	if (cabrio_rtap_supported(priv))
		wdev->wiphy->interface_modes |= BIT(NL80211_IFTYPE_MONITOR);
	if (cabrio_mesh_activated(priv))
		wdev->wiphy->interface_modes |= BIT(NL80211_IFTYPE_MESH_POINT);
	#endif
	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &cabrio_band_2ghz;

	/*
	 * We could check priv->fwcapinfo && FW_CAPINFO_WPA, but I have
	 * never seen a firmware without WPA
	 */
	wdev->wiphy->cipher_suites = cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites);
	wdev->wiphy->reg_notifier = cabrio_reg_notifier;

	ret = wiphy_register(wdev->wiphy);
	if (ret < 0)
		pr_err("cannot register wiphy device\n");

	priv->wiphy_registered = true;

	ret = register_netdev(priv->dev);
	if (ret)
		pr_err("cannot register network device\n");

	INIT_DELAYED_WORK(&priv->scan_work, cabrio_scan_worker);

	cabrio_cfg_set_regulatory_hint(priv);

	cabrio_dbg_leave_args(CABRIO_DBG_CFG80211, "ret %d", ret);
	return ret;
}

int cabrio_reg_notifier(struct wiphy *wiphy,
		struct regulatory_request *request)
{
	struct cabrio_private *priv = wiphy_priv(wiphy);
	int ret;

	cabrio_dbg_enter_args(CABRIO_DBG_CFG80211, "cfg80211 regulatory domain "
			"callback for domain %c%c\n", request->alpha2[0],
			request->alpha2[1]);

	ret = cabrio_set_11d_domain_info(priv, request, wiphy->bands);

	cabrio_dbg_leave(CABRIO_DBG_CFG80211);
	return ret;
}

void cabrio_scan_deinit(struct cabrio_private *priv)
{
	cabrio_dbg_enter(CABRIO_DBG_CFG80211);
	cancel_delayed_work_sync(&priv->scan_work);
}


void cabrio_cfg_free(struct cabrio_private *priv)
{
	struct wireless_dev *wdev = priv->wdev;

	cabrio_dbg_enter(CABRIO_DBG_CFG80211);

	if (!wdev)
		return;

	if (priv->wiphy_registered)
		wiphy_unregister(wdev->wiphy);

	if (wdev->wiphy)
		wiphy_free(wdev->wiphy);

	kfree(wdev);
}
