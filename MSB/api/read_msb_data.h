#ifndef READ_MSB_DATA
#define READ_MSB_DATA

#include "msb_temp.h"
#include "msb_knuckle.h"
#include "msb_central.h"

void read_temp_msb(temp_data_t* msb_temp_data);
void read_imu_msb(imu_data_t* msb_imu_data);
void read_central_msb(central_data_t* msb_central_data);

#endif