#ifndef __OS_TYPES_H__
#define __OS_TYPES_H__

#ifdef __GNUC__
#include "os_types_gnu.h"
#endif

#ifdef WIN32
#include "os_types_win32.h"
#endif
 
#endif // __OS_TYPES_H__
