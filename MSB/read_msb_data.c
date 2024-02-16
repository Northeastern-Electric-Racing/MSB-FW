#include "msb_temp.h"
#include "msb_knuckle.h"
#include "cmsis_os.h"
#include <stdio.h>

void read_temp_msg(void *);
void read_imu_msg(void *);

msb_temp_t* msb_temp;
msb_knuckle_t* msb_knuckle;

temp_data_t* msb_temp_data;
imu_data_t* msb_imu_data;

void read_temp_msg(temp_data_t* msb_temp_data) {
    char status[100];
    for (;;) {
        int ret = monitor_temp(msb_temp_data);
        if (ret) {
            return; //handle error 
        }
        sprintf(status, "{\"id\": %d, \"data\": %d, \"unit\": \"%c\"}", msb_temp->id, msb_temp_data->data, msb_temp_data->unit);
        push_can_queue(status);
        osDelay(500);
    }
}

void read_imu_msg() {
    char status[100];
    for (;;) {
        int ret = monitor_imu(msb_imu_data);
        if (ret) {
            return; // handle error
        }
        sprintf(status, "{\"id\": %d, \"accelertion\": %d, \"x_angle\": %d, \"y-angle\": %d, \"z-angle\": %d}", 
        msb_knuckle->id, msb_imu_data->accerlation, msb_imu_data->x_angle, msb_imu_data->y_angle, msb_imu_data->z_angle);
        push_can_queue(status);
        osDelay(500);
    }
}


