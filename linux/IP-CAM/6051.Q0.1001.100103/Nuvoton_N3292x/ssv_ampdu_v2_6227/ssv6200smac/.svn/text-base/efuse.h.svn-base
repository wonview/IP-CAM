#ifndef _SSV_EFUSE_H_
#define _SSV_EFUSE_H_

#include "dev.h"

struct efuse_map {
    u8 offset;
    u8 byte_cnts;
    u16 value;
};

enum efuse_data_item {
   EFUSE_R_CALIBRAION_RESULT = 1,
   EFUSE_SAR_RESULT,
   EFUSE_MAC,
   EFUSE_CRYSTAL_FREQUECY_OFFSET,
   EFUSE_DC_CALIBRAION_RESULT,
   EFUSE_IQ_CALIBRAION_RESULT,
   EFUSE_TX_POWER_INDEX_1,
   EFUSE_TX_POWER_INDEX_2
};

void efuse_read_all_map(struct ssv_hw *sh);

#endif

