#include "cmsis_os.h"
#include <stdio.h>

#include "read_msb_data.h"

#define BOARD_ID 1;

void read_temp_msb(temp_data_t *);
void read_imu_msb(imu_data_t *);
void read_central_msb(central_data_t *);

msb_temp_t* msb_temp;
msb_knuckle_t* msb_knuckle;
msb_central_t* msb_central;

void read_temp_msb(temp_data_t* msb_temp_data) {

    msb_temp->id = BOARD_ID;

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

void read_imu_msb(imu_data_t* msb_imu_data) {

    msb_knuckle->id = BOARD_ID;

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

void read_central_msb(central_data_t* msb_central_data) {

    msb_central->id = BOARD_ID;

    char status[100];
    for (;;) {
        int potentiometer_ret = monitor_potentiometer(msb_central_data);
        int strain_ret = monitor_strain_gauge(msb_central_data);
        int tof_ret = monitor_tof(msb_central_data);
        if (potentiometer_ret) {
            return; //handle error
        }
        if (strain_ret) {
            return; //handle error
        }
        if (tof_ret) {
            return; //handle error
        }
        sprintf(status, "{\"id\": %d, \"potentiometer\": %d, \"strain\": %d, \"ToF\": %d}",
        msb_central->id, msb_central_data->potentiometer, msb_central_data->strain, msb_central_data->tof);
        push_can_queue(status);
        osDelay(500);
    }
}


