#ifndef __CABRIO_CFG80211_H__
#define __CABRIO_CFG80211_H__

struct device;
struct cabrio_private;
struct regulatory_request;
struct wiphy;

struct wireless_dev *cabrio_cfg_alloc(struct device *dev);
int cabrio_cfg_register(struct cabrio_private *priv);
void cabrio_cfg_free(struct cabrio_private *priv);

int cabrio_reg_notifier(struct wiphy *wiphy,
		struct regulatory_request *request);

void cabrio_send_disconnect_notification(struct cabrio_private *priv);
void cabrio_send_mic_failureevent(struct cabrio_private *priv, u32 event);

void cabrio_scan_done(struct cabrio_private *priv);
void cabrio_scan_deinit(struct cabrio_private *priv);
int cabrio_disconnect(struct cabrio_private *priv, u16 reason);

#endif
