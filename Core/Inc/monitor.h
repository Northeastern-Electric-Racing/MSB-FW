#ifndef READ_MSB_DATA
#define READ_MSB_DATA

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/* Defining Temperature Monitor Task */
void vTempMonitor(void* pv_params);
extern osThreadId_t temp_monitor_handle;
extern const osThreadAttr_t temp_monitor_attributes;

/* Task for Monitoring the IMU */
void vIMUMonitor(void* pv_params);
extern osThreadId_t imu_monitor_handle;
extern const osThreadAttr_t imu_monitor_attributes;

#endif