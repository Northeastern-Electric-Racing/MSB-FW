
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "cmsis_os.h"
#include "lsm6dso.h"
#include "sht30.h"
#include "stm32f405xx.h"
#include "vl6180x_api.h"
#include "msb_conf.h"

typedef enum {
	DEVICE_FRONT_LEFT,
	DEVICE_FRONT_RIGHT,
	DEVICE_BACK_RIGHT,
	DEVICE_BACK_LEFT,
} device_loc_t;

int8_t init_msb();

#ifdef SENSOR_TEMP
int8_t measure_central_temp(uint16_t *temp, uint16_t *humidity);
#endif

#ifdef SENSOR_IMU
int8_t read_accel(uint16_t accel[3]);

int8_t read_gyro(uint16_t gyro[3]);
#endif

#ifdef SENSOR_TOF
int8_t read_distance(int32_t *range_mm);
#endif

int8_t write_debug1(bool status);

int8_t write_debug2(bool status);

int8_t write_vcc5_en(bool status);

#ifdef SENSOR_SHOCKPOT
void read_shockpot(uint32_t shockpot_sense);
#endif

#ifdef SENSOR_STRAIN
void read_strain1(uint32_t strain1);
void read_strain2(uint32_t strain2);
#endif

#endif
