#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>

#include "api/monitor_msb.h"

void monitor_temp_msb(void* arg) {

    msb_temp_t* msb = (msb_temp_t*) arg;

    temp_data_t* out = malloc(sizeof(temp_data_t));

    char status[100];
    for (;;) {
        int ret = measure_temp(out);
        if (ret) {
            return; //handle error 
        }
        sprintf(status, "{\"id\": %d, \"data\": %d, \"unit\": \"%c\"}", msb->id, out->data, out->unit);
        push_can_queue(status);
        osDelay(500);
    }
}

void monitor_knuckle_msb(void* arg) {

    msb_knuckle_t* msb = (msb_knuckle_t*) arg;

    knuckle_data_t* out = malloc(sizeof(knuckle_data_t));

    char status[100];
    for (;;) {
        int ret = measure_imu(out);
        if (ret) {
            return; // handle error
        }
        sprintf(status, "{\"id\": %d, \"acceleration\": %d, \"x_angle\": %d, \"y-angle\": %d, \"z-angle\": %d}", 
            msb->id, out->acceleration, out->x_angle, out->y_angle, out->z_angle);
        push_can_queue(status);
        osDelay(500);
    }
}

void monitor_central_msb(void* arg) {

    msb_central_t* msb = (msb_central_t*) arg;

    central_data_t* out = malloc(sizeof(msb_central_t));

    char status[100];
    for (;;) {
        int potentiometer_ret = measure_potentiometer(out);
        int strain_ret = measure_strain_gauge(out);
        int tof_ret = measure_tof(out);
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
            msb->id, out->potentiometer, out->strain, out->tof);
        push_can_queue(status);
        osDelay(500);
    }
}


