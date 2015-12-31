
/*
 *  This file contains declaration referring to
 *  functions defined in other source files
 */

#ifndef _SSV_DECL_H_
#define _SSV_DECL_H_

#include <linux/netdevice.h>
#include <linux/firmware.h>
#include <linux/nl80211.h>

/* Should be terminated by a NULL entry */
struct cabrio_fw_table {
	int model;
	const char *fwname;
};

struct cabrio_private;
struct sk_buff;
struct net_device;
struct cmd_ds_command;


/* ethtool.c */
extern const struct ethtool_ops cabrio_ethtool_ops;


/* tx.c */
void cabrio_send_tx_feedback(struct cabrio_private *priv, u32 try_count);
netdev_tx_t cabrio_start_xmit(struct sk_buff *skb,
                              struct net_device *dev);

/* rx.c */
int cabrio_process_rxed_packet(struct cabrio_private *priv, struct sk_buff *);


/* main.c */
struct cabrio_private *cabrio_add_card(void *card, struct device *dmdev);
void cabrio_remove_card(struct cabrio_private *priv);
int cabrio_start_card(struct cabrio_private *priv);
void cabrio_stop_card(struct cabrio_private *priv);
void cabrio_host_to_card_done(struct cabrio_private *priv);

int cabrio_start_iface(struct cabrio_private *priv);
int cabrio_stop_iface(struct cabrio_private *priv);
int cabrio_set_iface_type(struct cabrio_private *priv, enum nl80211_iftype type);

int cabrio_rtap_supported(struct cabrio_private *priv);

int cabrio_set_mac_address(struct net_device *dev, void *addr);
void cabrio_set_multicast_list(struct net_device *dev);
void cabrio_update_mcast(struct cabrio_private *priv);

int cabrio_suspend(struct cabrio_private *priv);
int cabrio_resume(struct cabrio_private *priv);

void cabrio_queue_event(struct cabrio_private *priv, void *event, u32 size);
void cabrio_notify_command_response(struct cabrio_private *priv, u8 resp_idx);

int cabrio_enter_auto_deep_sleep(struct cabrio_private *priv);
int cabrio_exit_auto_deep_sleep(struct cabrio_private *priv);

u32 cabrio_fw_index_to_data_rate(u8 index);
u8 cabrio_data_rate_to_fw_index(u32 rate);

int cabrio_get_firmware(struct device *dev,
			char **user_mainfw, u32 card_model,
			const struct cabrio_fw_table *fw_table,
			const struct firmware **mainfw);

#endif // _SSV_DECL_H_
