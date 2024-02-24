
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "msb_common.h"

typedef struct msb_central {
    short id;
} msb_central_t;

typedef struct central_data {
    const int strain;
    const int potentiometer;
    const int tof;
} central_data_t;
 

int measure_strain_gauge(central_data_t* out);
int measure_potentiometer(central_data_t* out);
int measure_tof(central_data_t* out);

#endif
