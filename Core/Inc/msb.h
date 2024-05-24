
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "lsm6dso.h"
#include "sht30.h"
#include "stm32f405xx.h"
#include "cmsis_os.h"

typedef enum
{
    DEVICE_FRONT_LEFT,
    DEVICE_FRONT_RIGHT,
    DEVICE_BACK_RIGHT,
    DEVICE_BACK_LEFT,
} device_loc_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    sht30_t *temp_sensor;
    lsm6dso_t *imu;
    ADC_HandleTypeDef *adc1;
    GPIO_TypeDef *debug_led1_gpio;
    uint16_t *debug_led1_pin;
    GPIO_TypeDef *debug_led2_gpio;
    uint16_t *debug_led2_pin;
    device_loc_t *device_loc;
    osMutexId_t *adc_mutex;
    osMutexId_t *i2c_mutex;
} msb_t;

typedef struct
{
    int8_t channel_0;
    int8_t channel_1;
    int8_t channel_2;
    int8_t channel_3;
} adc_channels;

msb_t *init_msb(I2C_HandleTypeDef *hi2c, ADC_HandleTypeDef *adc1, GPIO_TypeDef *debug_led1_gpio, 
uint16_t *debug_led1_pin, GPIO_TypeDef *debug_led2_gpio, uint16_t *debug_led2_pin,
                device_loc_t *device_loc);

int8_t measure_central_temp(msb_t *msb, uint16_t *temp, uint16_t *humidity);

int8_t read_accel(msb_t *msb, uint16_t accel[3]);

int8_t read_gyro(msb_t *msb, uint16_t gyro[3]);

int8_t write_debug1(msb_t* msb, bool status);

int8_t write_debug2(msb_t* msb, bool status);


#endif
