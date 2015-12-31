#ifndef _COMMON_H_
#define _COMMON_H_

#include "config.h"
#include "types.h"

#ifndef EAPOL_ETHER_TYPPE
#define EAPOL_ETHER_TYPPE	0x888E
#endif

#define WPA_AUTH_ALG_OPEN BIT(0)
#define WPA_AUTH_ALG_SHARED BIT(1)

typedef enum{
	SSV_SEC_NONE,
	SSV_SEC_WEP_40,			//5		ASCII
	SSV_SEC_WEP_104,		//13	ASCII
	SSV_SEC_WPA_PSK,		//8~63	ASCII
	SSV_SEC_WPA2_PSK,		//8~63	ASCII
	SSV_SEC_WPS,
}ssv_sec_type;

#define MAX_SSID_LEN 32
#define MAX_PASSWD_LEN 63

#define MAX_WEP_PASSWD_LEN (13+1)

//------------------------------------------------

/**
 *  struct cfg_sta_info - STA structure description
 *
 */
struct cfg_sta_info {
    ETHER_ADDR      addr;
    //u32             bit_rates; /* The first eight rates are the basic rate set */

    //u8              listen_interval;
    
    //u8              key_id;
    //u8              key[16];


} ;//__attribute__ ((packed));




/**
 *  struct cfg_bss_info - BSS/IBSS structure description
 *
 */
struct cfg_bss_info {
    ETHER_ADDR          bssid;

};// __attribute__ ((packed));



#ifndef BIT
#define BIT(x) (1 << (x))
#endif

#define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))


#endif /* _COMMON_H_ */

