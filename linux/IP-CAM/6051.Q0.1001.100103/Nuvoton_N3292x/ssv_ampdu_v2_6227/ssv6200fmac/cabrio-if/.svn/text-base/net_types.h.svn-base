#ifndef __NET_TYPE_H__
#define __NET_TYPE_H__

#define	ETHER_ADDR_LEN	6
#define	LLC_HEADER_LEN	6
#define	ETHER_TYPE_LEN	2
#define	ETHER_HDR_LEN	14
#define	ETHER_MAC_LEN	12		//802.3 DA+SA

struct ETHER_ADDR_st
{
    u8      addr[ETHER_ADDR_LEN];
};

typedef struct ETHER_ADDR_st             ETHER_ADDR;

#define IS_WILDCARD_ADDR(m)             (!memcmp(&WILDCARD_ADDR, (m), ETHER_ADDR_LEN))
#define IS_EQUAL_MACADDR(m1, m2)        (!memcmp((m1), (m2), ETHER_ADDR_LEN))

#define ETH_ADDR_FORMAT                 "%02X:%02X:%02X:%02X:%02X:%02X"
#define ETH_ADDR(a)                     ((ETHER_ADDR *)(a))->addr[0], ((ETHER_ADDR *)(a))->addr[1], \
                                        ((ETHER_ADDR *)(a))->addr[2], ((ETHER_ADDR *)(a))->addr[3], \
                                        ((ETHER_ADDR *)(a))->addr[4], ((ETHER_ADDR *)(a))->addr[5]
#define KEY_32_FORMAT                   "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X"
#define KEY_16_FORMAT                   "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X"
#define KEY_8_FORMAT                    "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X"
#define KEY_32_VAL(m)                   (m)[0],(m)[1],(m)[2],(m)[3],(m)[4],(m)[5],(m)[6],(m)[7],(m)[8],(m)[9],(m)[10],(m)[11],(m)[12],(m)[13],(m)[14],(m)[15],(m)[16],(m)[18],(m)[18],(m)[19],(m)[20],(m)[21],(m)[22],(m)[23],(m)[24],(m)[25],(m)[26],(m)[27],(m)[28],(m)[29],(m)[30],(m)[31]
#define KEY_16_VAL(m)                   (m)[0],(m)[1],(m)[2],(m)[3],(m)[4],(m)[5],(m)[6],(m)[7],(m)[8],(m)[9],(m)[10],(m)[11],(m)[12],(m)[13],(m)[14],(m)[15]
#define KEY_8_VAL(m)                    (m)[0],(m)[1],(m)[2],(m)[3],(m)[4],(m)[5],(m)[6],(m)[7]

#endif // __NET_TYPE_H__

