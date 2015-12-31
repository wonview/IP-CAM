/*
 * This header file contains global constant/enum definitions,
 * global variable declaration.
 */
#ifndef _CABRIO_DEFS_H_
#define _CABRIO_DEFS_H_

#include <linux/spinlock.h>

#ifdef CONFIG_CABRIO_DEBUG
#define DEBUG
#define PROC_DEBUG
#endif

#ifndef DRV_NAME
#define DRV_NAME "cabrio"
#endif


#define CABRIO_DBG_ENTER    0x00000001
#define CABRIO_DBG_LEAVE    0x00000002
#define CABRIO_DBG_MAIN     0x00000004
#define CABRIO_DBG_NET      0x00000008
#define CABRIO_DBG_MESH     0x00000010
#define CABRIO_DBG_WEXT     0x00000020
#define CABRIO_DBG_IOCTL    0x00000040
#define CABRIO_DBG_SCAN     0x00000080
#define CABRIO_DBG_ASSOC	0x00000100
#define CABRIO_DBG_JOIN     0x00000200
#define CABRIO_DBG_11D      0x00000400
#define CABRIO_DBG_DEBUGFS  0x00000800
#define CABRIO_DBG_ETHTOOL  0x00001000
#define CABRIO_DBG_HOST     0x00002000
#define CABRIO_DBG_CMD      0x00004000
#define CABRIO_DBG_RX       0x00008000
#define CABRIO_DBG_TX       0x00010000
#define CABRIO_DBG_USB      0x00020000
#define CABRIO_DBG_CS       0x00040000
#define CABRIO_DBG_FW       0x00080000
#define CABRIO_DBG_THREAD	0x00100000
#define CABRIO_DBG_HEX      0x00200000
#define CABRIO_DBG_SDIO     0x00400000
#define CABRIO_DBG_SYSFS    0x00800000
#define CABRIO_DBG_SPI      0x01000000
#define CABRIO_DBG_CFG80211 0x02000000

extern unsigned int cabrio_debug;

#ifdef DEBUG
#define CABRIO_DBG_LL(grp, grpnam, fmt, args...) \
do { if ((cabrio_debug & (grp)) == (grp)) \
  printk(KERN_DEBUG DRV_NAME grpnam "%s: " fmt, \
         in_interrupt() ? " (INT)" : "", ## args); } while (0)
#else
#define CABRIO_DBG_LL(grp, grpnam, fmt, args...) do {} while (0)
#endif

#define CABRIO_TODO(args...) \
  do { \
    printk(KERN_DEBUG DRV_NAME " TODO %s \"%s\": \n" , \
           in_interrupt() ? " (INT)" : "", ## args); \
  } while (0)

#define cabrio_dbg_enter(grp) \
  CABRIO_DBG_LL(grp | CABRIO_DBG_ENTER, " enter", "%s()\n", __func__);
#define cabrio_dbg_enter_args(grp, fmt, args...) \
  CABRIO_DBG_LL(grp | CABRIO_DBG_ENTER, " enter", "%s(" fmt ")\n", __func__, ## args);
#define cabrio_dbg_leave(grp) \
  CABRIO_DBG_LL(grp | CABRIO_DBG_LEAVE, " leave", "%s()\n", __func__);
#define cabrio_dbg_leave_args(grp, fmt, args...) \
  CABRIO_DBG_LL(grp | CABRIO_DBG_LEAVE, " leave", "%s(), " fmt "\n", \
  __func__, ##args);
#define cabrio_dbg_main(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_MAIN, " main", fmt, ##args)
#define cabrio_dbg_net(fmt, args...)       CABRIO_DBG_LL(CABRIO_DBG_NET, " net", fmt, ##args)
#define cabrio_dbg_mesh(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_MESH, " mesh", fmt, ##args)
#define cabrio_dbg_wext(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_WEXT, " wext", fmt, ##args)
#define cabrio_dbg_ioctl(fmt, args...)     CABRIO_DBG_LL(CABRIO_DBG_IOCTL, " ioctl", fmt, ##args)
#define cabrio_dbg_scan(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_SCAN, " scan", fmt, ##args)
#define cabrio_dbg_assoc(fmt, args...)     CABRIO_DBG_LL(CABRIO_DBG_ASSOC, " assoc", fmt, ##args)
#define cabrio_dbg_join(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_JOIN, " join", fmt, ##args)
#define cabrio_dbg_11d(fmt, args...)       CABRIO_DBG_LL(CABRIO_DBG_11D, " 11d", fmt, ##args)
#define cabrio_dbg_debugfs(fmt, args...)   CABRIO_DBG_LL(CABRIO_DBG_DEBUGFS, " debugfs", fmt, ##args)
#define cabrio_dbg_ethtool(fmt, args...)   CABRIO_DBG_LL(CABRIO_DBG_ETHTOOL, " ethtool", fmt, ##args)
#define cabrio_dbg_host(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_HOST, " host", fmt, ##args)
#define cabrio_dbg_cmd(fmt, args...)       CABRIO_DBG_LL(CABRIO_DBG_CMD, " cmd", fmt, ##args)
#define cabrio_dbg_rx(fmt, args...)        CABRIO_DBG_LL(CABRIO_DBG_RX, " rx", fmt, ##args)
#define cabrio_dbg_tx(fmt, args...)        CABRIO_DBG_LL(CABRIO_DBG_TX, " tx", fmt, ##args)
#define cabrio_dbg_fw(fmt, args...)        CABRIO_DBG_LL(CABRIO_DBG_FW, " fw", fmt, ##args)
#define cabrio_dbg_usb(fmt, args...)       CABRIO_DBG_LL(CABRIO_DBG_USB, " usb", fmt, ##args)
#define cabrio_dbg_usbd(dev, fmt, args...) CABRIO_DBG_LL(CABRIO_DBG_USB, " usbd", "%s:" fmt, dev_name(dev), ##args)
#define cabrio_dbg_cs(fmt, args...)        CABRIO_DBG_LL(CABRIO_DBG_CS, " cs", fmt, ##args)
#define cabrio_dbg_thread(fmt, args...)    CABRIO_DBG_LL(CABRIO_DBG_THREAD, " thread", fmt, ##args)
#define cabrio_dbg_sdio(fmt, args...)      CABRIO_DBG_LL(CABRIO_DBG_SDIO, " sdio", fmt, ##args)
#define cabrio_dbg_sysfs(fmt, args...)     CABRIO_DBG_LL(CABRIO_DBG_SYSFS, " sysfs", fmt, ##args)
#define cabrio_dbg_spi(fmt, args...)       CABRIO_DBG_LL(CABRIO_DBG_SPI, " spi", fmt, ##args)
#define cabrio_dbg_cfg80211(fmt, args...)  CABRIO_DBG_LL(CABRIO_DBG_CFG80211, " cfg80211", fmt, ##args)

#ifdef DEBUG
static inline void cabrio_dbg_hex(unsigned int grp, const char *prompt, u8 *buf, int len)
{
	int i = 0;

	if (len &&
	    (cabrio_debug & CABRIO_DBG_HEX) &&
	    (cabrio_debug & grp))
	{
		for (i = 1; i <= len; i++) {
			if ((i & 0xf) == 1) {
				if (i != 1)
					printk("\n");
				printk(DRV_NAME " %s: ", prompt);
			}
			printk("%02x ", (u8) * buf);
			buf++;
		}
		printk("\n");
	}
}
#else
#define cabrio_dbg_hex(grp,prompt,buf,len)	do {} while (0)
#endif



/* Buffer Constants */

/*	The size of SQ memory PPA, DPA are 8 DWORDs, that keep the physical
 *	addresses of TxPD buffers. Station has only 8 TxPD available, Whereas
 *	driver has more local TxPDs. Each TxPD on the host memory is associated
 *	with a Tx control node. The driver maintains 8 RxPD descriptors for
 *	station firmware to store Rx packet information.
 *
 *	Current version of MAC has a 32x6 multicast address buffer.
 *
 *	802.11b can have up to  14 channels, the driver keeps the
 *	BSSID(MAC address) of each APs or Ad hoc stations it has sensed.
 */

#define CABRIO_MAX_MULTICAST_LIST_SIZE  32
//#define CABRIO_NUM_CMD_BUFFERS          10
#define CABRIO_NUM_CMD_BUFFERS          100
#define CABRIO_CMD_BUFFER_SIZE          (2 * 1024)
#define CABRIO_MAX_CHANNEL_SIZE         14
#define CABRIO_ASSOCIATION_TIME_OUT     255
#define CABRIO_SNAP_HEADER_LEN          8

#define	CABRIO_UPLD_SIZE			2312
#define DEV_NAME_LEN			32

/* Wake criteria for HOST_SLEEP_CFG command */
#define EHS_WAKE_ON_BROADCAST_DATA	0x0001
#define EHS_WAKE_ON_UNICAST_DATA	0x0002
#define EHS_WAKE_ON_MAC_EVENT		0x0004
#define EHS_WAKE_ON_MULTICAST_DATA	0x0008
#define EHS_REMOVE_WAKEUP		0xFFFFFFFF
/* Wake rules for Host_Sleep_CFG command */
#define WOL_RULE_NET_TYPE_INFRA_OR_IBSS	0x00
#define WOL_RULE_NET_TYPE_MESH		0x10
#define WOL_RULE_ADDR_TYPE_BCAST	0x01
#define WOL_RULE_ADDR_TYPE_MCAST	0x08
#define WOL_RULE_ADDR_TYPE_UCAST	0x02
#define WOL_RULE_OP_AND			0x01
#define WOL_RULE_OP_OR			0x02
#define WOL_RULE_OP_INVALID		0xFF
#define WOL_RESULT_VALID_CMD		0
#define WOL_RESULT_NOSPC_ERR		1
#define WOL_RESULT_EEXIST_ERR		2

/* Misc constants */
/* This section defines 802.11 specific contants */

#define CABRIO_MAX_BSS_DESCRIPTS		16
#define CABRIO_MAX_REGION_CODE			6

#define CABRIO_DEFAULT_LISTEN_INTERVAL		10

#define	CABRIO_CHANNELS_PER_SCAN		4
#define	CABRIO_MAX_CHANNELS_PER_SCAN		14

#define CABRIO_MIN_BEACON_INTERVAL		20
#define CABRIO_MAX_BEACON_INTERVAL		1000
#define CABRIO_BEACON_INTERVAL			100


/* INT status Bit Definition */
#define CABRIO_TX_DNLD_RDY		0x0001
#define CABRIO_RX_UPLD_RDY		0x0002
#define CABRIO_CMD_DNLD_RDY		0x0004
#define CABRIO_CMD_UPLD_RDY		0x0008
#define CABRIO_CARDEVENT		0x0010

/* Automatic TX control default levels */
#define POW_ADAPT_DEFAULT_P0 13
#define POW_ADAPT_DEFAULT_P1 15
#define POW_ADAPT_DEFAULT_P2 18
#define TPC_DEFAULT_P0 5
#define TPC_DEFAULT_P1 10
#define TPC_DEFAULT_P2 13

/* TxPD status */

/*
 *	Station firmware use TxPD status field to report final Tx transmit
 *	result, Bit masks are used to present combined situations.
 */

#define CABRIO_TxPD_POWER_MGMT_NULL_PACKET 0x01
#define CABRIO_TxPD_POWER_MGMT_LAST_PACKET 0x08

/* FW definition from SSV Cabrio v1 */
#define CABRIO_FW_V1					(0x01)
/* FW major revision definition */
#define CABRIO_FW_MAJOR_REV(x)				((x)>>24)

/* RxPD status */

#define CABRIO_RXPD_STATUS_OK                0x0001

/* RxPD status - Received packet types */

/* RSSI-related defines */
/*
 *	RSSI constants are used to implement 802.11 RSSI threshold
 *	indication. if the Rx packet signal got too weak for 5 consecutive
 *	times, miniport driver (driver) will report this event to wrapper
 */

#define CABRIO_NF_DEFAULT_SCAN_VALUE		(-96)

/* RTS/FRAG related defines */
#define CABRIO_RTS_MIN_VALUE		0
#define CABRIO_RTS_MAX_VALUE		2347
#define CABRIO_FRAG_MIN_VALUE		256
#define CABRIO_FRAG_MAX_VALUE		2346

/* This is for firmware specific length */
/*
#define EXTRA_LEN	36

#define CABRIO_ETH_TX_PACKET_BUFFER_SIZE \
	(ETH_FRAME_LEN + sizeof(struct txpd) + EXTRA_LEN)

#define CABRIO_ETH_RX_PACKET_BUFFER_SIZE \
	(ETH_FRAME_LEN + sizeof(struct rxpd) \
	 + CABRIO_SNAP_HEADER_LEN + EXTRA_LEN)
*/

#define	CMD_F_HOSTCMD		(1 << 0)
#define FW_CAPINFO_WPA  	(1 << 0)
#define FW_CAPINFO_PS  		(1 << 1)
#define FW_CAPINFO_FIRMWARE_UPGRADE	(1 << 13)
#define FW_CAPINFO_BOOT2_UPGRADE	(1<<14)
#define FW_CAPINFO_PERSISTENT_CONFIG	(1<<15)

#define KEY_LEN_WPA_AES			16
#define KEY_LEN_WPA_TKIP		32
#define KEY_LEN_WEP_104			13
#define KEY_LEN_WEP_40			5

#define RF_ANTENNA_1		0x1
#define RF_ANTENNA_2		0x2
#define RF_ANTENNA_AUTO		0xFFFF

#define	BAND_B			(0x01)
#define	BAND_G			(0x02)
#define ALL_802_11_BANDS	(BAND_B | BAND_G)

#define MAX_RATES			14

#define	MAX_LEDS			8

/* Global Variable Declaration */
extern const char cabrio_driver_version[];
extern u16 cabrio_region_code_to_index[CABRIO_MAX_REGION_CODE];


/* ENUM definition */
/* SNRNF_TYPE */
enum SNRNF_TYPE {
	TYPE_BEACON = 0,
	TYPE_RXPD,
	MAX_TYPE_B
};

/* SNRNF_DATA */
enum SNRNF_DATA {
	TYPE_NOAVG = 0,
	TYPE_AVG,
	MAX_TYPE_AVG
};

/* CABRIO_802_11_POWER_MODE */
enum CABRIO_802_11_POWER_MODE {
	CABRIO802_11POWERMODECAM,
	CABRIO802_11POWERMODEMAX_PSP,
	CABRIO802_11POWERMODEFAST_PSP,
	/* not a real mode, defined as an upper bound */
	CABRIO802_11POWERMODEMAX
};

/* PS_STATE */
enum PS_STATE {
	PS_STATE_FULL_POWER,
	PS_STATE_AWAKE,
	PS_STATE_PRE_SLEEP,
	PS_STATE_SLEEP
};

/* DNLD_STATE */
enum DNLD_STATE {
	DNLD_RES_RECEIVED,
	DNLD_DATA_SENT,
	DNLD_CMD_SENT,
	DNLD_BOOTCMD_SENT,
};

/* CABRIO_MEDIA_STATE */
enum CABRIO_MEDIA_STATE {
	CABRIO_CONNECTED,
	CABRIO_DISCONNECTED
};

/* CABRIO_802_11_PRIVACY_FILTER */
enum CABRIO_802_11_PRIVACY_FILTER {
	CABRIO802_11PRIVFILTERACCEPTALL,
	CABRIO802_11PRIVFILTER8021XWEP
};

/* mv_ms_type */
enum cabrio_ms_type {
	CABRIO_MS_DAT = 0,
	CABRIO_MS_CMD = 1,
	CABRIO_MS_TXDONE = 2,
	CABRIO_MS_EVENT
};

/* KEY_TYPE_ID */
enum KEY_TYPE_ID {
	KEY_TYPE_ID_WEP = 0,
	KEY_TYPE_ID_TKIP,
	KEY_TYPE_ID_AES
};

/* KEY_INFO_WPA (applies to both TKIP and AES/CCMP) */
enum KEY_INFO_WPA {
	KEY_INFO_WPA_MCAST = 0x01,
	KEY_INFO_WPA_UNICAST = 0x02,
	KEY_INFO_WPA_ENABLED = 0x04
};

/* Default values for fwt commands. */
#define FWT_DEFAULT_METRIC 0
#define FWT_DEFAULT_DIR 1
/* Default Rate, 11Mbps */
#define FWT_DEFAULT_RATE 3
#define FWT_DEFAULT_SSN 0xffffffff
#define FWT_DEFAULT_DSN 0
#define FWT_DEFAULT_HOPCOUNT 0
#define FWT_DEFAULT_TTL 0
#define FWT_DEFAULT_EXPIRATION 0
#define FWT_DEFAULT_SLEEPMODE 0
#define FWT_DEFAULT_SNR 0

#define ORIG    (0)

#endif
