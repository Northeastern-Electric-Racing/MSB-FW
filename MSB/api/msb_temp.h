
#ifndef MSB_TEMP_H
#define MSB_TEMP_H

#include "api/msb_common.h"

typedef struct msb_temp {
    const short id;
} msb_temp_t;

typedef struct temp_data
{
    int data;
    char unit;
} temp_data_t;


int monitor_temp(temp_data_t* output);

#endif
