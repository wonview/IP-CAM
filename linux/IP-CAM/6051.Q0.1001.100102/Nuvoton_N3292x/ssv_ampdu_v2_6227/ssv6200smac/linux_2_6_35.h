#ifndef _LINUX_2_6_35_H_
#define _LINUX_2_6_35_H_

#include <net/mac80211.h>


/* Add some code lack in LINUX kernel 2.6.35*/


/**
 * enum ieee80211_ac_numbers - AC numbers as used in mac80211
 * @IEEE80211_AC_VO: voice
 * @IEEE80211_AC_VI: video
 * @IEEE80211_AC_BE: best effort
 * @IEEE80211_AC_BK: background
 */
enum ieee80211_ac_numbers {
	IEEE80211_AC_VO		= 0,
	IEEE80211_AC_VI		= 1,
	IEEE80211_AC_BE		= 2,
	IEEE80211_AC_BK		= 3,
};
#define IEEE80211_NUM_ACS	4


#define wiphy_info(wiphy, format, args...)			\
	dev_info(&(wiphy)->dev, format, ##args)


#define IEEE80211_CONF_OFFCHANNEL   (1<<30)
#define FIF_PROBE_REQ               (1<<8)
#define BSS_CHANGED_SSID            (1<<15)



#endif /* _LINUX_2_6_35_H_ */

