#ifndef CONTROL_MSB
#define CONTROL_MSB

#include "cmsis_os.h"

void vLedController(void *pv_params);
extern osThreadId_t led_controller_handle;
extern const osThreadAttr_t led_controller_attributes;

#endif