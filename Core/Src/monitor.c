#include "c_utils.h"
#include "can.h"
#include "can_handler.h"
#include "cmsis_os.h"
#include "msb.h"
#include "msb_conf.h"

#include "stm32f405xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "monitor.h"

extern device_loc_t device_loc;

uint16_t convert_can(uint16_t original_value, device_loc_t mode)
{
	switch (mode) {
	case DEVICE_FRONT_LEFT:
		return original_value;
	case DEVICE_FRONT_RIGHT:
		return original_value + 0x20;
	case DEVICE_BACK_LEFT:
		return original_value + 0x40;
	case DEVICE_BACK_RIGHT:
		return original_value + 0x60;
	default:
		return original_value;
	}
}

#ifdef SENSOR_TEMP
osThreadId_t temp_monitor_handle;
const osThreadAttr_t temp_monitor_attributes = {
	.name = "TempMonitor",
	.stack_size = TEMP_MONITOR_STACK_SIZE,
	.priority = (osPriority_t)osPriorityHigh1,
};

void vTempMonitor(void *pv_params)
{
	can_msg_t temp_sensor_msg = { .id = convert_can(CANID_TEMP_SENSOR,
							device_loc),
				      .len = 4,
				      .data = { 0 } };

	struct __attribute__((__packed__)) {
		uint16_t temp;
		uint16_t humidity;
	} temp_sensor_data;

	uint16_t temp_dat = 0;
	uint16_t humidity_dat = 0;

	for (;;) {
		if (central_temp_measure(&temp_dat, &humidity_dat)) {
			printf("Failed to get temp\r\n");
		}

		temp_sensor_data.temp = temp_dat;
		temp_sensor_data.humidity = humidity_dat;

#ifdef LOG_VERBOSE
		printf("Board Temperature:\t%d\r\n", temp_sensor_data.temp);
		printf("Board Humidity:\t%d\r\n", temp_sensor_data.humidity);
#endif

		endian_swap(&temp_sensor_data.temp,
			    sizeof(temp_sensor_data.temp));
		endian_swap(&temp_sensor_data.humidity,
			    sizeof(temp_sensor_data.humidity));

		memcpy(temp_sensor_msg.data, &temp_sensor_data,
		       temp_sensor_msg.len);
		/* Send CAN message */
		if (queue_can_msg(temp_sensor_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		/* Yield to other tasks */
		osDelay(DELAY_TEMP_SENSOR_REFRESH);
	}
}
#endif

#ifdef SENSOR_IMU
osThreadId_t imu_monitor_handle;
const osThreadAttr_t imu_monitor_attributes = {
	.name = "IMUMonitor",
	.stack_size = IMU_MONITOR_STACK_SIZE,
	.priority = (osPriority_t)osPriorityHigh,
};

void vIMUMonitor(void *pv_params)
{
	// const uint8_t num_samples = 10;
	can_msg_t imu_accel_msg = { .id = convert_can(CANID_IMU_ACCEL,
						      device_loc),
				    .len = 6,
				    .data = { 0 } };
	can_msg_t imu_gyro_msg = { .id = convert_can(CANID_IMU_GYRO,
						     device_loc),
				   .len = 6,
				   .data = { 0 } };

	struct __attribute__((__packed__)) {
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
	} accel_data;

	struct __attribute__((__packed__)) {
		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;
	} gyro_data;

	struct __attribute__((__packed__)) {
		float_t temp;
	} temperature_data;

	stmdev_ctx_t ctx;
	stmdev_ctx_t aux_ctx;
	// int16_t temperature_data_temp;

	lsm6dso_md_t imu_md_temp;
	lsm6dso_data_t imu_data_temp;

	#ifdef MOTION_FX
	can_msg_t imu_orientation_msg = { .id = convert_can(CANID_IMU_ORIENTATION,
									device_loc), 
									  .len = 6,
									  .data = { 0 } };

	MFX_input_t mFXInput;
	float roll, pitch, yaw;

	struct __attribute__((__packed__)) {
		int16_t roll;
		int16_t pitch;
		int16_t yaw;
	} orientation_data;
	#endif

	/* Add parameters for formatting data */
	imu_md_temp.ui.gy.fs = LSM6DSO_500dps;
	imu_md_temp.ui.gy.odr = LSM6DSO_GY_UI_52Hz_LP;
	imu_md_temp.ui.xl.fs = LSM6DSO_XL_UI_2g;
	imu_md_temp.ui.xl.odr = LSM6DSO_XL_UI_52Hz_LP;

	for (;;) {
		/* Take measurement */
		if (imu_data_get(&ctx, &aux_ctx, &imu_md_temp,
				 &imu_data_temp)) {
			printf("Failed to get IMU data \r\n");
		}

		/* Run values through LPF of sample size  */
		accel_data.accel_x = imu_data_temp.ui.xl.mg[0];
		accel_data.accel_y = imu_data_temp.ui.xl.mg[1];
		accel_data.accel_z = imu_data_temp.ui.xl.mg[2];
		gyro_data.gyro_x = imu_data_temp.ui.gy.mdps[0];
		gyro_data.gyro_y = imu_data_temp.ui.gy.mdps[1];
		gyro_data.gyro_z = imu_data_temp.ui.gy.mdps[2];
		temperature_data.temp = imu_data_temp.ui.heat.deg_c;

		#ifdef MOTION_FX

			// Acc (Convert mg to g)
			mFXInput.acc[0] = imu_data_temp.ui.xl.mg[0] / 1000.0f;
			mFXInput.acc[1] = imu_data_temp.ui.xl.mg[1] / 1000.0f;
			mFXInput.acc[2] = imu_data_temp.ui.xl.mg[2] / 1000.0f;

			// Gyro (Convert mdps to dps)
			mFXInput.gyro[0] = imu_data_temp.ui.gy.mdps[0] * 0.001f;
			mFXInput.gyro[1] = imu_data_temp.ui.gy.mdps[1] * 0.001f;
			mFXInput.gyro[2] = imu_data_temp.ui.gy.mdps[2] * 0.001f;

			// Magnetometer
			mFXInput.mag[0] = 0.0f;
			mFXInput.mag[1] = 0.0f;
			mFXInput.mag[2] = 0.0f;

			process_motion_fx(&mFXInput, &roll, &pitch, &yaw);

			orientation_data.roll = (int16_t)roll;
			orientation_data.pitch = (int16_t)pitch;
			orientation_data.yaw = (int16_t)yaw;

		#endif

#ifdef LOG_VERBOSE
		printf("IMU Accel x: %d y: %d z: %d \r\n", accel_data.accel_x,
		       accel_data.accel_y, accel_data.accel_z);
		printf("IMU Gyro x: %d y: %d z: %d \r\n", gyro_data.gyro_x,
		       gyro_data.gyro_y, gyro_data.gyro_z);
		printf("IMU Orientation Yaw: %d Pitch: %d Roll: %d \r\n", 
       		   orientation_data.yaw, orientation_data.pitch, orientation_data.roll);
		printf("IMU Temp: %3.2f °C \r\n", temperature_data.temp);
#endif

		/* convert to big endian */
		endian_swap(&accel_data.accel_x, sizeof(accel_data.accel_x));
		endian_swap(&accel_data.accel_y, sizeof(accel_data.accel_y));
		endian_swap(&accel_data.accel_z, sizeof(accel_data.accel_z));
		endian_swap(&gyro_data.gyro_x, sizeof(gyro_data.gyro_x));
		endian_swap(&gyro_data.gyro_y, sizeof(gyro_data.gyro_y));
		endian_swap(&gyro_data.gyro_z, sizeof(gyro_data.gyro_z));
		endian_swap(&orientation_data.roll, sizeof(orientation_data.roll));
		endian_swap(&orientation_data.pitch, sizeof(orientation_data.pitch));
		endian_swap(&orientation_data.yaw, sizeof(orientation_data.yaw));

		/* Send CAN message */
		memcpy(imu_accel_msg.data, &accel_data, imu_accel_msg.len);
		if (queue_can_msg(imu_accel_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		memcpy(imu_gyro_msg.data, &gyro_data, imu_gyro_msg.len);
		if (queue_can_msg(imu_gyro_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		memcpy(imu_orientation_msg.data, &orientation_data, imu_orientation_msg.len);
		if (queue_can_msg(imu_orientation_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		/* Yield to other tasks */
		osDelay(DELAY_IMU_REFRESH);
	}
}
#endif

#ifdef SENSOR_TOF
osThreadId_t tof_monitor_handle;
const osThreadAttr_t tof_monitor_attributes = {
	.name = "TOFMonitor",
	.stack_size = TOF_MONITOR_STACK_SIZE,
	.priority = (osPriority_t)osPriorityHigh,
};

void vTOFMonitor(void *pv_params)
{
	can_msg_t range_msg = { .id = convert_can(CANID_TOF, device_loc),
				.len = 4,
				.data = { 0 } };

	int32_t range;

	for (;;) {
		if (distance_read(&range)) {
			printf("failed to read distance!\r\n");
			continue;
		}

#ifdef LOG_VERBOSE
		printf("Range is: %ld\r\n", range);
#endif

		endian_swap(&range, sizeof(range));

		memcpy(range_msg.data, &range, range_msg.len);
		/* Send CAN message */
		if (queue_can_msg(range_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		osDelay(DELAY_TOF_REFRESH);
	}
}
#endif

#ifdef SENSOR_SHOCKPOT
osThreadId_t shockpot_monitor_handle;
const osThreadAttr_t shockpot_monitor_attributes = {
	.name = "ShockpotMonitor",
	.stack_size = SHOCKPOT_MONITOR_STACK_SIZE,
	.priority = (osPriority_t)osPriorityHigh1,
};

void vShockpotMonitor(void *pv_params)
{
	can_msg_t shockpot_msg = { .id = convert_can(CANID_SHOCK_SENSE,
						     device_loc),
				   .len = 4,
				   .data = { 0 } };

	uint32_t shock_value = 0;

	for (;;) {
		shockpot_read(shock_value);

#ifdef LOG_VERBOSE
		printf("Shock value:\t%ld\r\n", shock_value);
#endif

		endian_swap(&shock_value, sizeof(shock_value));

		memcpy(shockpot_msg.data, &shock_value, shockpot_msg.len);
		/* Send CAN message */
		if (queue_can_msg(shockpot_msg)) {
			printf("Failed to send CAN message\r\n");
		}

		/* Yield to other tasks */
		osDelay(DELAY_SHOCKPOT_REFRESH);
	}
}
#endif

#ifdef SENSOR_STRAIN
osThreadId_t strain_monitor_handle;
const osThreadAttr_t strain_monitor_attributes = {
	.name = "StrainMonitor",
	.stack_size = STRAIN_MONITOR_STACK_SIZE,
	.priority = (osPriority_t)osPriorityHigh1,
};

void vStrainMonitor(void *pv_params)
{
	can_msg_t strain_msg = { .id = convert_can(CANID_STRAIN_SENSE,
						   device_loc),
				 .len = 8,
				 .data = { 0 } };

	struct __attribute__((__packed__)) {
		uint32_t strain1;
		uint32_t strain2;
	} strain_data;

	uint32_t strain1_dat = 0;
	uint32_t strain2_dat = 0;
	for (;;) {
		strain1_read(strain1_dat);
		strain2_read(strain2_dat);

#ifdef LOG_VERBOSE
		printf("Strain 1: %ld  2: %ld \r\n", strain1_dat, strain2_dat);
#endif

		strain_data.strain1 = strain1_dat;
		strain_data.strain2 = strain2_dat;

		endian_swap(&strain_data.strain1, sizeof(strain_data.strain1));
		endian_swap(&strain_data.strain2, sizeof(strain_data.strain2));

		memcpy(strain_msg.data, &strain_data, strain_msg.len);
		/* Send CAN message */
		if (queue_can_msg(strain_msg)) {
			printf("Failed to send CAN message");
		}

		/* Yield to other tasks */
		osDelay(DELAY_SHOCKPOT_REFRESH);
	}
}
#endif