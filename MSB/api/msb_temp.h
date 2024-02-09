
#ifndef MSB_TEMP_H
#define MSB_TEMP_H

#include "api/msb_common.h"

typedef struct msb_temp {
    const short id;
} msb_temp_t;

void monitor_temp(msb_temp_t* msb);

#endif
