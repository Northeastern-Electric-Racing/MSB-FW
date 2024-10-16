#include "msb.h"
#include "main.h"
#include "lsm6dso.h"
#include <assert.h>
#include <serial_monitor.h>
#include <stdlib.h>
#include <string.h>

static osMutexAttr_t msb_i2c_mutex_attr;

extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern device_loc_t device_loc;

osMutexId_t i2c_mutex;

#ifdef SENSOR_TEMP
sht30_t temp_sensor;
#endif

#ifdef SENSOR_IMU
lsm6dso_t imu;
#endif

#ifdef SENSOR_TOF
VL6180xDev_t tof;
#endif

#if defined SENSOR_STRAIN || defined SENSOR_SHOCKPOT
uint32_t adc1_buf[3];
#endif

int8_t msb_init()
{
#ifdef SENSOR_TEMP
	/* Initialize the Onboard Temperature Sensor */
	temp_sensor = (sht30_t){
		.i2c_handle = &hi2c3,
	};
	assert(!sht30_init(&temp_sensor)); /* This is always connected */
#endif

#ifdef SENSOR_IMU
	/* Initialize the IMU */
	imu = (lsm6dso_t){

	};
	assert(!lsm6dso_init(&imu, &hi2c3)); /* This is always connected */
#endif

#ifdef SENSOR_TOF
	/* Initialize the ToF sensor */
	tof = malloc(sizeof(VL6180xDev_t));
	assert(tof);
	assert(!VL6180x_WaitDeviceBooted(tof));
	assert(!VL6180x_InitData(tof));
	assert(!VL6180x_Prepare(tof));
#endif

#if defined SENSOR_SHOCKPOT || defined SENSOR_STRAIN
	assert(!HAL_ADC_Start_DMA(&hadc1, adc1_buf,
				  sizeof(adc1_buf) / sizeof(uint32_t)));
#endif

	/* Create Mutexes */
	i2c_mutex = osMutexNew(&msb_i2c_mutex_attr);
	assert(i2c_mutex);

	return 0;
}

#ifdef SENSOR_TEMP
/// @brief Measure the temperature and humidity of central MSB SHT30
/// @param out
/// @return error code
int8_t central_temp_measure(uint16_t *temp, uint16_t *humidity)
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = sht30_get_temp_humid(&temp_sensor);
	if (hal_stat)
		return hal_stat;

	*temp = temp_sensor.temp;
	*humidity = temp_sensor.humidity;

	osMutexRelease(i2c_mutex);

	return 0;
}
#endif

#if defined SENSOR_SHOCKPOT || defined SENSOR_STRAIN
void adc1_read(uint32_t result_buf[3])
{
	memcpy(result_buf, adc1_buf, sizeof(adc1_buf));
}
#endif

#ifdef SENSOR_SHOCKPOT
void shockpot_read(uint32_t shockpot_sense)
{
	memcpy((uint32_t *)shockpot_sense, adc1_buf, sizeof(shockpot_sense));
}
#endif

#ifdef SENSOR_STRAIN
void strain1_read(uint32_t strain1)
{
	memcpy((uint32_t *)strain1, adc1_buf + 1, sizeof(strain1));
}

void strain2_read(uint32_t strain2)
{
	memcpy((uint32_t *)strain2, adc1_buf + 2, sizeof(strain2));
}
#endif

#ifdef SENSOR_IMU
int8_t accel_read(uint16_t accel[3])
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = lsm6dso_read_accel(&imu);
	if (hal_stat)
		return hal_stat;

	memcpy(accel, imu.accel_data, 3);

	osMutexRelease(i2c_mutex);
	return 0;
}

int8_t gyro_read(uint16_t gyro[3])
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	HAL_StatusTypeDef hal_stat = lsm6dso_read_gyro(&imu);
	if (hal_stat)
		return hal_stat;

	memcpy(gyro, imu.gyro_data, 3);

	osMutexRelease(i2c_mutex);
	return 0;
}
#endif

#ifdef SENSOR_TOF
VL6180x_RangeData_t *range;
int8_t distance_read(int32_t *range_mm)
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	VL6180x_RangePollMeasurement(tof, range);
	if (range->errorStatus) {
		serial_print(
			"Error in range %f",
			VL6180x_RangeGetStatusErrString(range->errorStatus));
		return range->errorStatus;
	}

	memcpy(range_mm, &range->range_mm, sizeof(range->range_mm));

	osMutexRelease(i2c_mutex);
	return 0;
}
#endif

int8_t debug1_write(bool status)
{
	HAL_GPIO_WritePin(Debug_LED_1_GPIO_Port, Debug_LED_1_Pin, status);
	return 0;
}

int8_t debug2_write(bool status)
{
	HAL_GPIO_WritePin(Debug_LED_2_GPIO_Port, Debug_LED_2_Pin, status);
	return 0;
}

int8_t vcc5_en_write(bool status)
{
	HAL_GPIO_WritePin(VCC5_En_GPIO_Port, VCC5_En_Pin, status);
	return 0;
}