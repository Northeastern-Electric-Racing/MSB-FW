
#ifndef MSB_KNUCKLE_H
#define MSB_KNUCKLE_H

#include "api/msb_common.h"

typedef struct msb_knuckle {
    const short id;
} msb_knuckle_t;

void monitor_imu(msb_knuckle_t* msb);

#endif
