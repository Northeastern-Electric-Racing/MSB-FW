
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "api/msb_common.h"

typedef struct msb_central {
    const short id;
} msb_central_t;

void monitor_strain_gauge(msb_central_t* msb);
void monitor_potentiometer(msb_central_t* msb);
void monitor_tof(msb_central_t* msb);

#endif
