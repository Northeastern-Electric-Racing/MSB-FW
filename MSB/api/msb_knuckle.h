
#ifndef MSB_KNUCKLE_H
#define MSB_KNUCKLE_H

#include "msb_common.h"

typedef struct msb_knuckle {
    short id;
} msb_knuckle_t;

typedef struct msb_data {
    int acceleration;
    int x_angle;
    int y_angle;
    int z_angle;
} knuckle_data_t;

int measure_imu(knuckle_data_t* out);

#endif
