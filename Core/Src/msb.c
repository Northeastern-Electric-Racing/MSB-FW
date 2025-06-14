#include "msb.h"
#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include "main.h"
#include "motion_fx.h"
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
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mFXState[STATE_SIZE];
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
	assert(!LSM6DSO_Init(&imu)); /* This is always connected */

	/* Setup IMU Accelerometer - default 104Hz */
	LSM6DSO_ACC_Enable(&imu);
	/* Setup IMU Gyroscope */
	LSM6DSO_GYRO_Enable(&imu);

	LSM6DSO_ACC_Set_Filter_Mode(&imu, 0, 4);

	LSM6DSO_FIFO_Set_Mode(&imu, 0);
	LSM6DSO_ACC_Disable_Inactivity_Detection(&imu);

	/* Initialize Motion FX*/
	motion_fx_init();
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
void shockpot_read(uint32_t *shockpot_sense)
{
	memcpy(shockpot_sense, adc1_buf, sizeof(shockpot_sense));
}
#endif

#ifdef SENSOR_STRAIN
void strain1_read(uint32_t *strain1)
{
	memcpy(strain1, adc1_buf + 1, sizeof(strain1));
}

void strain2_read(uint32_t *strain2)
{
	memcpy(strain2, adc1_buf + 2, sizeof(strain2));
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

void motion_fx_init(void)
{
	if (STATE_SIZE < MotionFX_GetStateSize()) {
		printf("Not enough memory allocated for MotionFX!!");
		return;
	}

	MotionFX_initialize((MFXState_t *)mFXState);

	MotionFX_getKnobs(mFXState, ipKnobs);

	ipKnobs->acc_orientation[0] = 's';
	ipKnobs->acc_orientation[1] = 'e';
	ipKnobs->acc_orientation[2] = 'u';

	ipKnobs->gyro_orientation[0] = 's';
	ipKnobs->gyro_orientation[1] = 'e';
	ipKnobs->gyro_orientation[2] = 'u';

	ipKnobs->mag_orientation[0] = 'n';
	ipKnobs->mag_orientation[1] = 'e';
	ipKnobs->mag_orientation[2] = 'u';

	ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
	ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
	ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;

	ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
	ipKnobs->LMode = 1;
	ipKnobs->modx = DECIMATION;

	MotionFX_setKnobs(mFXState, ipKnobs);

	MotionFX_enable_6X(mFXState, MFX_ENGINE_ENABLE);
}

void process_motion_fx(MFX_input_t *data_in, MFX_output_t *data_out,
		       float delta_time)
{
	MotionFX_propagate(mFXState, data_out, data_in, &delta_time);

	MotionFX_update(mFXState, data_out, data_in, &delta_time, NULL);
}
#endif