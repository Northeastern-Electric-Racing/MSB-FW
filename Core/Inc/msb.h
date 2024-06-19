
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "cmsis_os.h"
#include "lsm6dso.h"
#include "sht30.h"
#include "stm32f405xx.h"
#include "vl6180x_api.h"

typedef enum {
	DEVICE_FRONT_LEFT,
	DEVICE_FRONT_RIGHT,
	DEVICE_BACK_RIGHT,
	DEVICE_BACK_LEFT,
} device_loc_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	sht30_t *temp_sensor;
	lsm6dso_t *imu;
	VL6180xDev_t tof;
	ADC_HandleTypeDef *adc1;
	uint32_t adc1_buf[3];
	GPIO_TypeDef *debug_led1_gpio;
	uint16_t *debug_led1_pin;
	GPIO_TypeDef *debug_led2_gpio;
	uint16_t *debug_led2_pin;
	device_loc_t *device_loc;
	osMutexId_t *i2c_mutex;
} msb_t;

msb_t *init_msb(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *adc1,
		GPIO_TypeDef *debug_led1_gpio, uint16_t *debug_led1_pin,
		GPIO_TypeDef *debug_led2_gpio, uint16_t *debug_led2_pin,
		device_loc_t *device_loc);

int8_t measure_central_temp(msb_t *msb, uint16_t *temp, uint16_t *humidity);

int8_t read_accel(msb_t *msb, uint16_t accel[3]);

int8_t read_gyro(msb_t *msb, uint16_t gyro[3]);

int8_t read_distance(msb_t *msb, int32_t *range_mm);

int8_t write_debug1(msb_t *msb, bool status);

int8_t write_debug2(msb_t *msb, bool status);

void read_shockpot(msb_t *msb, uint32_t shockpot_sense);
void read_strain1(msb_t *msb, uint32_t strain1);
void read_strain2(msb_t *msb, uint32_t strain2);

#endif
