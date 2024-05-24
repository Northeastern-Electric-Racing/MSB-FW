#include "msb.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include "lsm6dso.h"

static osMutexAttr_t msb_i2c_mutex_attr;

msb_t *init_msb(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *adc1, GPIO_TypeDef *debug_led1_gpio,
				uint16_t *debug_led1_pin, GPIO_TypeDef *debug_led2_gpio, uint16_t *debug_led2_pin,
				device_loc_t *device_loc)
{
	assert(hi2c);
	assert(adc1);
	assert(debug_led1_gpio);
	assert(debug_led2_gpio);
	assert(device_loc);

	msb_t *msb = malloc(sizeof(msb_t));
	assert(msb);

	msb->hi2c = hi2c;
	msb->adc1 = adc1;
	msb->debug_led1_gpio = debug_led1_gpio;
	msb->debug_led1_pin = debug_led1_pin;
	msb->debug_led2_gpio = debug_led2_gpio;
	msb->debug_led2_pin = debug_led2_pin;
	msb->device_loc = device_loc;

	/* Initialize the Onboard Temperature Sensor */
	msb->temp_sensor = malloc(sizeof(sht30_t));
	assert(msb->temp_sensor);
	msb->temp_sensor->i2c_handle = hi2c;
	assert(!sht30_init(msb->temp_sensor)); /* This is always connected */

	/* Initialize the IMU */
	msb->imu = malloc(sizeof(lsm6dso_t));
	assert(msb->imu);
	assert(!lsm6dso_init(msb->imu, msb->hi2c)); /* This is always connected */

	assert(!HAL_ADC_Start_DMA(msb->adc1, msb->adc1_buf, sizeof(msb->adc1_buf) / sizeof(uint32_t)));

	/* Create Mutexes */
	msb->i2c_mutex = osMutexNew(&msb_i2c_mutex_attr);
	assert(msb->i2c_mutex);

	return msb;
}

/// @brief Measure the temperature and humidity of central MSB SHT30
/// @param out
/// @return error code
int8_t measure_central_temp(msb_t *msb, uint16_t *temp, uint16_t *humidity)
{
	if (!msb)
		return -1;

	osStatus_t mut_stat = osMutexAcquire(msb->i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = sht30_get_temp_humid(msb->temp_sensor);
	if (hal_stat)
		return hal_stat;

	*temp = msb->temp_sensor->temp;
	*humidity = msb->temp_sensor->humidity;

	osMutexRelease(msb->i2c_mutex);

	return 0;
}

void read_adc1(msb_t *msb, uint32_t adc1_buf[3])
{
	memcpy(adc1_buf, msb->adc1_buf, sizeof(msb->adc1_buf));
}

void read_shockpot(msb_t *msb, uint32_t shockpot_sense)
{
	memcpy((uint32_t *)shockpot_sense, msb->adc1_buf, sizeof(shockpot_sense));
}

void read_strain1(msb_t *msb, uint32_t strain1)
{
	memcpy((uint32_t *)strain1, msb->adc1_buf + 1, sizeof(strain1));
}

void read_strain2(msb_t *msb, uint32_t strain2)
{
	memcpy((uint32_t *)strain2, msb->adc1_buf + 2, sizeof(strain2));
}

int8_t read_accel(msb_t *msb, uint16_t accel[3])
{
	if (!msb)
		return -1;

	osStatus_t mut_stat = osMutexAcquire(msb->i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = lsm6dso_read_accel(msb->imu);
	if (hal_stat)
		return hal_stat;

	memcpy(accel, msb->imu->accel_data, 3);

	osMutexRelease(msb->i2c_mutex);
	return 0;
}

int8_t read_gyro(msb_t *msb, uint16_t gyro[3])
{
	if (!msb)
		return -1;

	osStatus_t mut_stat = osMutexAcquire(msb->i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = lsm6dso_read_gyro(msb->imu);
	if (hal_stat)
		return hal_stat;

	memcpy(gyro, msb->imu->gyro_data, 3);

	osMutexRelease(msb->i2c_mutex);
	return 0;
}

int8_t write_debug1(msb_t *msb, bool status)
{
	if (!msb)
		return -1;

	HAL_GPIO_WritePin(msb->debug_led1_gpio, *msb->debug_led1_pin, status);
	return 0;
}

int8_t write_debug2(msb_t *msb, bool status)
{
	if (!msb)
		return -1;

	HAL_GPIO_WritePin(msb->debug_led2_gpio, *msb->debug_led2_pin, status);
	return 0;
}