#include "msb_conf.h"
#include "msb.h"
#include "c_utils.h"
#include "can.h"
#include "can_handler.h"
#include "cmsis_os.h"
#include "serial_monitor.h"

#include "stm32f405xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "monitor.h"

osThreadId_t temp_monitor_handle;
const osThreadAttr_t temp_monitor_attributes = {
	.name		= "TempMonitor",
	.stack_size = 32 * 8,
	.priority	= (osPriority_t)osPriorityHigh1,
};

void vTempMonitor(void *pv_params)
{
    can_msg_t temp_sensor_msg = {.id = CANID_TEMP_SENSOR, .len = 4, .data = {0}};

    msb_t *msb = (msb_t *)pv_params;

    struct __attribute__((__packed__))
    {
        uint16_t temp;
        uint16_t humidity;
    } temp_sensor_data;

    for (;;)
    {   
        uint16_t temp_dat;
        uint16_t humidity_dat;
        if (measure_central_temp(msb, &temp_dat, &humidity_dat))
        {
            printf("Failed to get temp");
        }

        temp_sensor_data.temp = temp_dat;
        temp_sensor_data.humidity = humidity_dat;

#ifdef LOG_VERBOSE
        serial_print("Board Temperature:\t%d\r\n", temp_sensor_data.temp);
        serial_print("Board Humidity:\t%d\r\n", temp_sensor_data.humidity);
#endif

        endian_swap(&temp_sensor_data.temp, sizeof(temp_sensor_data.temp));
        endian_swap(&temp_sensor_data.humidity, sizeof(temp_sensor_data.humidity));

        memcpy(temp_sensor_msg.data, &temp_sensor_data, temp_sensor_msg.len);
        /* Send CAN message */
        if (queue_can_msg(temp_sensor_msg))
        {
            serial_print("Failed to send CAN message");
        }

        /* Yield to other tasks */
        osDelay(DELAY_TEMP_SENSOR_REFRESH);
    }
}

osThreadId_t imu_monitor_handle;
const osThreadAttr_t imu_monitor_attributes = {
	.name		= "IMUMonitor",
	.stack_size = 32 * 8,
	.priority	= (osPriority_t)osPriorityHigh,
};

void vIMUMonitor(void *pv_params)
{
    const uint8_t num_samples = 10;
    can_msg_t imu_accel_msg = {.id = CANID_IMU_ACCEL, .len = 6, .data = {0}};
    can_msg_t imu_gyro_msg = {.id = CANID_IMU_GYRO, .len = 6, .data = {0}};

    msb_t *msb = (msb_t *)pv_params;

    struct __attribute__((__packed__))
    {
        uint16_t accel_x;
        uint16_t accel_y;
        uint16_t accel_z;
    } accel_data;

    struct __attribute__((__packed__))
    {
        uint16_t gyro_x;
        uint16_t gyro_y;
        uint16_t gyro_z;
    } gyro_data;

    for (;;)
    {
        /* Take measurement */
        uint16_t accel_data_temp[3] = {0};
        uint16_t gyro_data_temp[3] = {0};
        if (read_accel(msb, accel_data_temp))
        {
            serial_print("Failed to get IMU acceleration");
        }

        if (read_gyro(msb, gyro_data_temp))
        {
            serial_print("Failed to get IMU gyroscope");
        }

        /* Run values through LPF of sample size  */
        accel_data.accel_x = (accel_data.accel_x + accel_data_temp[0]) / num_samples;
        accel_data.accel_y = (accel_data.accel_y + accel_data_temp[1]) / num_samples;
        accel_data.accel_z = (accel_data.accel_z + accel_data_temp[2]) / num_samples;
        gyro_data.gyro_x = (gyro_data.gyro_x + gyro_data_temp[0]) / num_samples;
        gyro_data.gyro_y = (gyro_data.gyro_y + gyro_data_temp[1]) / num_samples;
        gyro_data.gyro_z = (gyro_data.gyro_z + gyro_data_temp[2]) / num_samples;

        /* convert to big endian */
        endian_swap(&accel_data.accel_x, sizeof(accel_data.accel_x));
        endian_swap(&accel_data.accel_y, sizeof(accel_data.accel_y));
        endian_swap(&accel_data.accel_z, sizeof(accel_data.accel_z));
        endian_swap(&gyro_data.gyro_x, sizeof(gyro_data.gyro_x));
        endian_swap(&gyro_data.gyro_y, sizeof(gyro_data.gyro_y));
        endian_swap(&gyro_data.gyro_z, sizeof(gyro_data.gyro_z));

        /* Send CAN message */
        memcpy(imu_accel_msg.data, &accel_data, imu_accel_msg.len);
        if (queue_can_msg(imu_accel_msg))
        {
            serial_print("Failed to send CAN message");
        }

        memcpy(imu_gyro_msg.data, &gyro_data, imu_gyro_msg.len);
        if (queue_can_msg(imu_gyro_msg))
        {
            serial_print("Failed to send CAN message");
        }

        /* Yield to other tasks */
        osDelay(DELAY_IMU_REFRESH);
    }
}