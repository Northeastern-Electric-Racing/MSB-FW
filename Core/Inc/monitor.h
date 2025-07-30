#ifndef READ_MSB_DATA
#define READ_MSB_DATA

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "msb_conf.h"

#ifdef CAN_ENABLE // TODO change to wheel sensor flag  
void record_wheel_angle(uint8_t *data);
#endif

#ifdef SENSOR_TEMP
/* Defining Temperature Monitor Task */
void vTempMonitor(void *pv_params);
extern osThreadId_t temp_monitor_handle;
extern const osThreadAttr_t temp_monitor_attributes;
#endif

#ifdef SENSOR_IMU
/* Task for Monitoring the IMU */
void vIMUMonitor(void *pv_params);
extern osThreadId_t imu_monitor_handle;
extern const osThreadAttr_t imu_monitor_attributes;
#endif

#ifdef SENSOR_TOF
void vTOFMonitor(void *pv_params);
extern osThreadId_t tof_monitor_handle;
extern const osThreadAttr_t tof_monitor_attributes;
#endif

#ifdef SENSOR_SHOCKPOT
void vShockpotMonitor(void *pv_params);
extern osThreadId_t shockpot_monitor_handle;
extern const osThreadAttr_t shockpot_monitor_attributes;
#endif

#ifdef SENSOR_STRAIN
void vStrainMonitor(void *pv_params);
extern osThreadId_t strain_monitor_handle;
extern const osThreadAttr_t strain_monitor_attributes;
#endif

#endif