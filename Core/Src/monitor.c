#include "c_utils.h"
#include "can.h"
#include "can_handler.h"
#include "cmsis_os.h"
#include "msb.h"
#include "msb_conf.h"
#include "serial_monitor.h"

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
	.stack_size = 32 * 8,
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
			printf("Failed to get temp");
		}

		temp_sensor_data.temp = temp_dat;
		temp_sensor_data.humidity = humidity_dat;

#ifdef LOG_VERBOSE
		serial_print("Board Temperature:\t%d\r\n",
			     temp_sensor_data.temp);
		serial_print("Board Humidity:\t%d\r\n",
			     temp_sensor_data.humidity);
#endif

		endian_swap(&temp_sensor_data.temp,
			    sizeof(temp_sensor_data.temp));
		endian_swap(&temp_sensor_data.humidity,
			    sizeof(temp_sensor_data.humidity));

		memcpy(temp_sensor_msg.data, &temp_sensor_data,
		       temp_sensor_msg.len);
		/* Send CAN message */
		if (queue_can_msg(temp_sensor_msg)) {
			serial_print("Failed to send CAN message");
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
	.stack_size = 32 * 8,
	.priority = (osPriority_t)osPriorityHigh,
};

void vIMUMonitor(void *pv_params)
{
	const uint8_t num_samples = 10;
	can_msg_t imu_accel_msg = { .id = convert_can(CANID_IMU_ACCEL,
						      device_loc),
				    .len = 6,
				    .data = { 0 } };
	can_msg_t imu_gyro_msg = { .id = convert_can(CANID_IMU_GYRO,
						     device_loc),
				   .len = 6,
				   .data = { 0 } };

	struct __attribute__((__packed__)) {
		uint16_t accel_x;
		uint16_t accel_y;
		uint16_t accel_z;
	} accel_data;

	struct __attribute__((__packed__)) {
		uint16_t gyro_x;
		uint16_t gyro_y;
		uint16_t gyro_z;
	} gyro_data;

	uint16_t accel_data_temp[3] = { 0 };
	uint16_t gyro_data_temp[3] = { 0 };

	for (;;) {
		/* Take measurement */

		if (accel_read(accel_data_temp)) {
			serial_print("Failed to get IMU acceleration");
		}

		if (gyro_read(gyro_data_temp)) {
			serial_print("Failed to get IMU gyroscope");
		}

		/* Run values through LPF of sample size  */
		accel_data.accel_x =
			(accel_data.accel_x + accel_data_temp[0]) / num_samples;
		accel_data.accel_y =
			(accel_data.accel_y + accel_data_temp[1]) / num_samples;
		accel_data.accel_z =
			(accel_data.accel_z + accel_data_temp[2]) / num_samples;
		gyro_data.gyro_x =
			(gyro_data.gyro_x + gyro_data_temp[0]) / num_samples;
		gyro_data.gyro_y =
			(gyro_data.gyro_y + gyro_data_temp[1]) / num_samples;
		gyro_data.gyro_z =
			(gyro_data.gyro_z + gyro_data_temp[2]) / num_samples;

#ifdef LOG_VERBOSE
		serial_print("IMU Accel x: %d y: %d z: %d \r\n",
			     accel_data.accel_x, accel_data.accel_y,
			     accel_data.accel_z);
		serial_print("IMU Gyro x: %d y: %d z: %d \r\n",
			     gyro_data.gyro_x, gyro_data.gyro_y,
			     gyro_data.gyro_z);
#endif

		/* convert to big endian */
		endian_swap(&accel_data.accel_x, sizeof(accel_data.accel_x));
		endian_swap(&accel_data.accel_y, sizeof(accel_data.accel_y));
		endian_swap(&accel_data.accel_z, sizeof(accel_data.accel_z));
		endian_swap(&gyro_data.gyro_x, sizeof(gyro_data.gyro_x));
		endian_swap(&gyro_data.gyro_y, sizeof(gyro_data.gyro_y));
		endian_swap(&gyro_data.gyro_z, sizeof(gyro_data.gyro_z));

		/* Send CAN message */
		memcpy(imu_accel_msg.data, &accel_data, imu_accel_msg.len);
		if (queue_can_msg(imu_accel_msg)) {
			serial_print("Failed to send CAN message");
		}

		memcpy(imu_gyro_msg.data, &gyro_data, imu_gyro_msg.len);
		if (queue_can_msg(imu_gyro_msg)) {
			serial_print("Failed to send CAN message");
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
	.stack_size = 32 * 8,
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
			serial_print("failed to read distance!");
			continue;
		}

#ifdef LOG_VERBOSE
		serial_print("Range is: %d", range);
#endif

		endian_swap(&range, sizeof(range));

		memcpy(range_msg.data, &range, range_msg.len);
		/* Send CAN message */
		if (queue_can_msg(range_msg)) {
			serial_print("Failed to send CAN message");
		}

		osDelay(DELAY_TOF_REFRESH);
	}
}
#endif

#ifdef SENSOR_SHOCKPOT
osThreadId_t shockpot_monitor_handle;
const osThreadAttr_t shockpot_monitor_attributes = {
	.name = "ShockpotMonitor",
	.stack_size = 32 * 8,
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
		serial_print("Shock value:\t%d\r\n", shock_value);
#endif

		endian_swap(&shock_value, sizeof(shock_value));

		memcpy(shockpot_msg.data, &shock_value, shockpot_msg.len);
		/* Send CAN message */
		if (queue_can_msg(shockpot_msg)) {
			serial_print("Failed to send CAN message");
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
	.stack_size = 32 * 8,
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
		serial_print("Strain 1: %d  2: %d \r\n", strain1_dat,
			     strain2_dat);
#endif

		strain_data.strain1 = strain1_dat;
		strain_data.strain2 = strain2_dat;

		endian_swap(&strain_data.strain1, sizeof(strain_data.strain1));
		endian_swap(&strain_data.strain2, sizeof(strain_data.strain2));

		memcpy(strain_msg.data, &strain_data, strain_msg.len);
		/* Send CAN message */
		if (queue_can_msg(strain_msg)) {
			serial_print("Failed to send CAN message");
		}

		/* Yield to other tasks */
		osDelay(DELAY_SHOCKPOT_REFRESH);
	}
}
#endif