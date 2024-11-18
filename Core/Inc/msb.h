
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "cmsis_os.h"
#include "lsm6dso.h"
#include "sht30.h"
#include "stm32f405xx.h"
#include "vl6180x_api.h"
#include "vl6180x_platform.h"
#include "msb_conf.h"

typedef enum {
	DEVICE_FRONT_LEFT,
	DEVICE_FRONT_RIGHT,
	DEVICE_BACK_RIGHT,
	DEVICE_BACK_LEFT,
} device_loc_t;

int8_t msb_init();

#ifdef SENSOR_TEMP
int8_t central_temp_measure(uint16_t *temp, uint16_t *humidity);
#endif

#ifdef SENSOR_IMU
int8_t accel_read(LSM6DSO_Axes_t* accel);

int8_t gyro_read(LSM6DSO_Axes_t* gyro);
#endif

#ifdef SENSOR_TOF
int8_t distance_read(int32_t *range_mm);
#endif

int8_t debug1_write(bool status);

int8_t debug2_write(bool status);

int8_t vcc5_en_write(bool status);

#ifdef SENSOR_SHOCKPOT
void shockpot_read(uint32_t shockpot_sense);
#endif

#ifdef SENSOR_STRAIN
void strain1_read(uint32_t strain1);
void strain2_read(uint32_t strain2);
#endif

#endif
