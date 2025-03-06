#include "msb.h"
#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include "main.h"
#include "sht30.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static osMutexAttr_t msb_i2c_mutex_attr;

extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern device_loc_t device_loc;

osMutexId_t i2c_mutex;

// reads imu reg

int32_t lsm6dso_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
			 uint16_t len)
{
	return HAL_I2C_Mem_Read(&hi2c3, LSM6DSO_I2C_ADD_L, reg,
				I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
int32_t lsm6dso_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
			  uint16_t len)
{
	return HAL_I2C_Mem_Write(&hi2c3, LSM6DSO_I2C_ADD_L, reg,
				 I2C_MEMADD_SIZE_8BIT, data, len,
				 HAL_MAX_DELAY);
}
#ifdef SENSOR_TEMP
sht30_t temp_sensor;
#endif

#ifdef SENSOR_IMU
LSM6DSO_Object_t imu;
#endif

#ifdef SENSOR_TOF
VL6180xDev_t tof;
#endif

#if defined SENSOR_STRAIN || defined SENSOR_SHOCKPOT
uint32_t adc1_buf[3];
#endif

static inline uint8_t sht30_i2c_write(uint8_t *data, uint8_t dev_address,
				       uint8_t length)
{
	return HAL_I2C_Master_Transmit(&hi2c3, dev_address,
		data, length, HAL_MAX_DELAY);
}

static inline uint8_t sht30_i2c_read(uint8_t *data, uint8_t dev_address,
				     uint16_t reg, uint8_t length)
{
	return HAL_I2C_Mem_Read(&hi2c3, dev_address, reg, I2C_MEMADD_SIZE_8BIT,
				data, length, HAL_MAX_DELAY);
}

int8_t msb_init()
{
#ifdef SENSOR_TEMP
	/* Initialize the Onboard Temperature Sensor */
	sht30_t temp_sensor;
	assert(!sht30_init(&temp_sensor, (Write_ptr) sht30_i2c_write, (Read_ptr) sht30_i2c_read,
			   (SHT30_I2C_ADDR))); /* This is always connected */
#endif

#ifdef SENSOR_IMU
	/* Initialize the IMU */
	assert(!LSM6DSO_Init(&imu)); /* This is always connected */

	/* Setup IMU Accelerometer */
	LSM6DSO_ACC_Enable(&imu);

	/* Setup IMU Gyroscope */
	LSM6DSO_GYRO_Enable(&imu);

	LSM6DSO_FIFO_Set_Mode(&imu, 0);
	LSM6DSO_ACC_Disable_Inactivity_Detection(&imu);
#endif

#ifdef SENSOR_TOF
	/* Initialize the ToF sensor */
	struct MyDev_t tof_get = {
		.i2c_bus_num = 0x29 << 1,
		.i2c_handle = &hi2c3,
	};
	tof = &tof_get;
	assert(tof);
	osDelay(1);
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
int8_t central_temp_measure(float *temp, float *humidity)
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	uint8_t status = sht30_get_temp_humid(&temp_sensor);
	if (status)
		return status;

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

#ifdef SENSOR_TOF
VL6180x_RangeData_t *range;
int8_t distance_read(int32_t *range_mm)
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;

	VL6180x_RangePollMeasurement(tof, range);
	if (range->errorStatus) {
		printf("Error in range %s\r\n",
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

#ifdef SENSOR_IMU
int32_t imu_data_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
		     lsm6dso_md_t *imu_md_temp, lsm6dso_data_t *imu_data_temp)
{
	osStatus_t mut_stat = osMutexAcquire(i2c_mutex, osWaitForever);
	if (mut_stat)
		return mut_stat;
	HAL_StatusTypeDef hal_stat =
		lsm6dso_data_get(ctx, aux_ctx, imu_md_temp, imu_data_temp);
	osMutexRelease(i2c_mutex);
	if (hal_stat)
		return hal_stat;
	return 0;
}
#endif