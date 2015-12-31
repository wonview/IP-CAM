#ifndef _TYPES_H_
#define _TYPES_H_

#include "os_types.h"

#ifndef NULL
#define NULL                        (void *)0
#endif

#ifndef true
#define true                        1
#endif

#ifndef false
#define false                       0
#endif

#ifndef TRUE
#define TRUE                        1
#endif

#ifndef FALSE
#define FALSE                       0
#endif

#ifndef SIZE_1KB
#define		SIZE_1KB	(1024)
#endif

#ifndef SIZE_1MB
#define		SIZE_1MB	(1024 * SIZE_1KB)
#endif

#ifndef OFFSETOF
#define OFFSETOF(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#include "net_types.h"

#endif /* _TYPES_H_ */

