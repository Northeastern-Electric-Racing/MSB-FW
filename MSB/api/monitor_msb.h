#ifndef READ_MSB_DATA
#define READ_MSB_DATA

#include "msb_temp.h"
#include "msb_knuckle.h"
#include "msb_central.h"

void monitor_temp_msb(msb_temp_t* msb, temp_data_t* out);
void monitor_knuckle_msb(msb_knuckle_t* msb, knuckle_data_t* out);
void monitor_central_msb(msb_central_t* msb, central_data_t* msb_central_data);

#endif