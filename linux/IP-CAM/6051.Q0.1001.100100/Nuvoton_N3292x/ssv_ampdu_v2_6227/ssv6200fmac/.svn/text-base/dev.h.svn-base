/*
 * This file contains definitions and data structures specific
 * to South Silicon Valley Cabrio 802.11 NIC. It contains the 
 * Device Information structure struct cabrio_private..
 */
#ifndef _CABRIO_DEV_H_
#define _CABRIO_DEV_H_

#include "defs.h"
#include "host.h"

#include <linux/kfifo.h>

/* sleep_params */
struct sleep_params {
	uint16_t sp_error;
	uint16_t sp_offset;
	uint16_t sp_stabletime;
	uint8_t  sp_calcontrol;
	uint8_t  sp_extsleepclk;
	uint16_t sp_reserved;
};

/* Mesh statistics */
struct cabrio_mesh_stats {
	u32	fwd_bcast_cnt;		/* Fwd: Broadcast counter */
	u32	fwd_unicast_cnt;	/* Fwd: Unicast counter */
	u32	fwd_drop_ttl;		/* Fwd: TTL zero */
	u32	fwd_drop_rbt;		/* Fwd: Recently Broadcasted */
	u32	fwd_drop_noroute; 	/* Fwd: No route to Destination */
	u32	fwd_drop_nobuf;		/* Fwd: Run out of internal buffers */
	u32	drop_blind;		/* Rx:  Dropped by blinding table */
	u32	tx_failed_cnt;		/* Tx:  Failed transmissions */
};

/* Private structure for the MV device */
struct cabrio_private {

	/* Basic networking */
	struct net_device *dev;
	u32 connect_status;
	struct work_struct mcast_work;
	u32 nr_of_multicastmacaddr;
	u8 multicastlist[CABRIO_MAX_MULTICAST_LIST_SIZE][ETH_ALEN];

	/* CFG80211 */
	struct wireless_dev *wdev;
	bool wiphy_registered;
	struct cfg80211_scan_request *scan_req;
	u8 assoc_bss[ETH_ALEN];
	u8 disassoc_reason;

#if 0
	/* Mesh */
	struct net_device *mesh_dev; /* Virtual device */
//SSV -- #ifdef CONFIG_LIBERTAS_MESH
	struct cabrio_mesh_stats mstats;
	uint16_t mesh_tlv;
	u8 mesh_ssid[IEEE80211_MAX_SSID_LEN + 1];
	u8 mesh_ssid_len;
#endif

	/* Debugfs */
	struct dentry *debugfs_dir;
	struct dentry *debugfs_debug;
	struct dentry *debugfs_files[6];
	struct dentry *events_dir;
	struct dentry *debugfs_events_files[6];
	struct dentry *regs_dir;
	struct dentry *debugfs_regs_files[6];

	/* Hardware debugging */
	u32 mac_offset;
	u32 bbp_offset;
	u32 rf_offset;

	/* Power management */
	u16 psmode;
	u32 psstate;
	u8 needtowakeup;

	/* Deep sleep */
	int is_deep_sleep;
	int deep_sleep_required;
	int is_auto_deep_sleep_enabled;
	int wakeup_dev_required;
	int is_activity_detected;
	int auto_deep_sleep_timeout; /* in ms */
	wait_queue_head_t ds_awake_q;
	struct timer_list auto_deepsleep_timer;

	/* Host sleep*/
	int is_host_sleep_configured;
	int is_host_sleep_activated;
	wait_queue_head_t host_sleep_q;

	/* Hardware access */
	void *card;
	bool iface_running;
	u8 fw_ready;
	u8 surpriseremoved;
	u8 setup_fw_on_resume;
	int (*hw_host_to_card) (struct cabrio_private *priv, u8 type, u8 *payload, u16 nb);
	void (*reset_card) (struct cabrio_private *priv);
	int (*power_save) (struct cabrio_private *priv);
	int (*power_restore) (struct cabrio_private *priv);
	int (*enter_deep_sleep) (struct cabrio_private *priv);
	int (*exit_deep_sleep) (struct cabrio_private *priv);
	int (*reset_deep_sleep_wakeup) (struct cabrio_private *priv);

    /* Hardware options */
    bool    wmm; 
    u16     qos_ctrl;
    bool    ht;
    u32     ht_ctrl;
    
	/* Adapter info (from EEPROM) */
	u32 fwrelease;
	u32 fwcapinfo;
	u16 regioncode;
	u8 current_addr[ETH_ALEN];
	u8 copied_hwaddr;

	/* Command download */
	u8 dnld_sent;
	/* bit0 1/0=data_sent/data_tx_done,
	   bit1 1/0=cmd_sent/cmd_tx_done,
	   all other bits reserved 0 */
	u16 seqnum;
	struct host_cmd_node *cmd_array;
	struct host_cmd_node *cur_cmd;
	struct list_head cmdfreeq;    /* free command buffers */
	struct list_head cmdpendingq; /* pending command buffers */
	struct timer_list command_timer;
	int cmd_timed_out;

	/* Command responses sent from the hardware to the driver */
	u8 resp_idx;
	u8 resp_buf[2][CABRIO_UPLD_SIZE];
	u32 resp_len[2];

	/* RX */
	bool in_association;
	struct sk_buff *held_rx_skb;
	struct sk_buff *held_rx_skb_head;

	/* Events sent from hardware to driver */
	struct kfifo event_fifo;

	/* thread to service interrupts */
	struct task_struct *main_thread;
	wait_queue_head_t waitq;
	struct workqueue_struct *work_thread;

	/* Encryption stuff */
	u8 is_securied_connected;
	u8 authtype_auto;
	u8 wep_tx_key;
	u8 wep_key[4][WLAN_KEY_LEN_WEP104];
	u8 wep_key_len[4];
    u8 wpa_key[WLAN_KEY_LEN_TKIP]; // TKIP has longest key.
    u8 wpa_key_len; 

	/* Wake On LAN */
	uint32_t wol_criteria;
	uint8_t wol_gpio;
	uint8_t wol_gap;
	bool ehs_remove_supported;

	/* Transmitting */
	int tx_pending_len;		/* -1 while building packet */
	u8 tx_pending_buf[CABRIO_UPLD_SIZE];
	/* protected by hard_start_xmit serialization */
	u8 txretrycount;
	struct sk_buff *currenttxskb;
	struct timer_list tx_lockup_timer;

	/* Locks */
	struct mutex lock;
	spinlock_t driver_lock;

	/* NIC/link operation characteristics */
	u16 mac_control;
	u8 radio_on;
	u8 cur_rate;
	u8 channel;
	s16 txpower_cur;
	s16 txpower_min;
	s16 txpower_max;

	/* Scanning */
	struct delayed_work scan_work;
	int scan_channel;
	/* Queue of things waiting for scan completion */
	wait_queue_head_t scan_q;
	/* Whether the scan was initiated internally and not by cfg80211 */
	bool internal_scan;
};

extern struct cmd_confirm_sleep confirm_sleep;

/* Check if there is an interface active. */
static inline int cabrio_iface_active(struct cabrio_private *priv)
{
	int r;

	r = netif_running(priv->dev);
#if 0
	if (priv->mesh_dev)
		r |= netif_running(priv->mesh_dev);
#endif // 0
	return r;
}

#define SSV_EVENT_SIZE  (768)		// Probe response might be large.
#define SSV_EVENT_COUNT (12)

#endif
