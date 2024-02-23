#ifndef READ_MSB_DATA
#define READ_MSB_DATA

#include "msb_temp.h"
#include "msb_knuckle.h"
#include "msb_central.h"

void monitor_temp_msb(void* arg);
void monitor_knuckle_msb(void* arg);
void monitor_central_msb(void* arg);

#endif