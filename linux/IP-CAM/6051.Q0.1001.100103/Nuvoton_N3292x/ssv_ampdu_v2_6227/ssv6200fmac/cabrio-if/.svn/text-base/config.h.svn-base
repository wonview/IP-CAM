#ifndef _CONFIG_H_
#define _CONFIG_H_

// Common configuration should be applied to for both host simuator and device
// FW calculate checksum to check correctness of downloaded image.
#define ENABLE_FW_SELF_CHECK            1
#define FW_BLOCK_SIZE                   (1024)
#define FW_CHECKSUM_INIT                (0x12345678)
// Use signle command response event instead of specific response event for each command.
#define USE_CMD_RESP                    1

// ------------------------- config -------------------------------
#ifndef linux
#if (defined _WIN32) || (defined __SSV_UNIX_SIM__)
#define CONFIG_SIM_PLATFORM         1
#include <sim_config.h>
#else
#define CONFIG_SIM_PLATFORM			0
#include <soc_config.h>
#endif
#endif // linux
#endif	/* _CONFIG_H_ */
