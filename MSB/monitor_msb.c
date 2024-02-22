#include "cmsis_os.h"
#include <stdio.h>

#include "api/monitor_msb.h"

void monitor_temp_msb(msb_temp_t* msb, temp_data_t* out) {

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

void monitor_knuckle_msb(msb_knuckle_t* msb, knuckle_data_t* out) {

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

void monitor_central_msb(msb_central_t* msb, central_data_t* out) {

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


