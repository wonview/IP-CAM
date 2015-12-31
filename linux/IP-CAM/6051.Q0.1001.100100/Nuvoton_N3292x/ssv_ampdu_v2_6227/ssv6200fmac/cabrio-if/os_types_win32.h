#ifndef __OS_TYPES_WIN32_H__
#define __OS_TYPES_WIN32_H__

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

typedef char                        s8;
typedef unsigned char               u8;
typedef short                       s16;
typedef unsigned short              u16;
typedef int                         s32;
typedef unsigned int                u32;
typedef long long                   u64;
//typedef u8                          bool;

#ifndef bool
#define bool                        u8
#endif

typedef unsigned int                size_t;


#if !defined(__cplusplus)
#define inline __inline
#endif

#define __le16 u16
#define __le32 u32
#define __le64 u64
#define __u8 u8

#define __be16 u16
#define __be32 u32
#define __be64 u64


#define le16 u16
#define le32 u32
#define le64 u64

#define be16 u16
#define be32 u32
#define be64 u64

#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )

#endif // __OS_TYPES_WIN32_H__
