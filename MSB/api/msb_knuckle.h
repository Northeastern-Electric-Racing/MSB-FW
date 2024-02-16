
#ifndef MSB_KNUCKLE_H
#define MSB_KNUCKLE_H

#include "api/msb_common.h"

typedef struct msb_knuckle {
    const short id;
} msb_knuckle_t;

typedef struct knuckle_data_t {
    int accerlation;
    int x_angle;
    int y_angle;
    int z_angle;
} imu_data_t;

int monitor_imu(msb_knuckle_t* msb);

#endif
