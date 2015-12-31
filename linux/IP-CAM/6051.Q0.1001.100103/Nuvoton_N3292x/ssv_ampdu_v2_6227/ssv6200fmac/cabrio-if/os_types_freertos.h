#ifndef __OS_TYPES_FREERTOS_H__
#define __OS_TYPES_FREERTOS_H__

#define ASSERT(x) \
{ \
    extern void stop_and_halt (void); \
    if (!(x)) \
    { \
        printf("Assert!! file: %s, function: %s, line: %d\n\t" #x, __FILE__, \
        	__FUNCTION__, __LINE__); \
        stop_and_halt(); \
    } \
}

#define EMPTY

#define ASSERT_RET(x, ret) \
{ \
    extern void stop_and_halt (void); \
    if (!(x)) \
    { \
        printf("Assert!! file: %s, function: %s, line: %d\n\t" #x, __FILE__, \
        	__FUNCTION__, __LINE__); \
        stop_and_halt(); \
        return ret; \
    } \
}

#define ASSERT_PKT(x, pkt) \
{ \
    extern void stop_and_dump_and_halt (const void *data, u32 size); \
    if (!(x)) \
    { \
        printf("Assert!! file: %s, function: %s, line: %d\n\t" #x, __FILE__, \
        	__FUNCTION__, __LINE__); \
        stop_and_dump_and_halt((const void *)pkt, 64); \
    } \
}


#undef assert
#define assert(x)                       ASSERT(x)

#define _LE16(x)                        (u16)(x)
#define IS_EQUAL(a, b)                  ( (a) == (b) )
#define SET_BIT(v, b)	                ( (v) |= (0x01<<b) )
#define CLEAR_BIT(v, b)                 ( (v) &= ~(0x01<<b) )
#define IS_BIT_SET(v, b)                ( (v) & (0x01<<(b) ) )

#ifndef BIT
#define BIT(nr)                         (1UL << (nr))
#endif

#endif // __OS_TYPES_FREERTOS_H__
