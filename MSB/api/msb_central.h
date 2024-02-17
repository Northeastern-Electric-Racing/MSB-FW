
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "msb_common.h"

typedef struct msb_central {
    const short id;
} msb_central_t;

typedef struct central_data
{
    const int strain;
    const int potentiometer;
    const int tof;
} central_data_t;
 

void monitor_strain_gauge(msb_central_t* msb);
void monitor_potentiometer(msb_central_t* msb);
void monitor_tof(msb_central_t* msb);

#endif
