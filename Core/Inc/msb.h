
#ifndef MSB_CENTRAL_H
#define MSB_CENTRAL_H

#include "cmsis_os.h"
#include "lsm6dso.h"
#include "motion_fx.h"
#include "sht30.h"
#include "stm32f405xx.h"
#include "vl6180x_api.h"
#include "vl6180x_platform.h"
#include "msb_conf.h"

typedef enum {
	DEVICE_FRONT_LEFT,
	DEVICE_FRONT_RIGHT,
	DEVICE_BACK_RIGHT,
	DEVICE_BACK_LEFT,
} device_loc_t;

int8_t msb_init();

#ifdef SENSOR_TEMP
int8_t central_temp_measure(uint16_t *temp, uint16_t *humidity);
#endif

#ifdef SENSOR_TOF
int8_t distance_read(int32_t *range_mm);
#endif

int8_t debug1_write(bool status);

int8_t debug2_write(bool status);

int8_t vcc5_en_write(bool status);

int32_t imu_data_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
		     lsm6dso_md_t *imu_md_temp, lsm6dso_data_t *imu_data_temp);

#ifdef SENSOR_SHOCKPOT
void shockpot_read(uint32_t shockpot_sense);
#endif

#ifdef SENSOR_STRAIN
void strain1_read(uint32_t strain1);
void strain2_read(uint32_t strain2);
#endif

#ifdef MOTION_FX

#define GBIAS_ACC_TH_SC (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC (2.0f*0.002f)
#define GBIAS_MAG_TH_SC 0.0f

#define DECIMATION 1U

#define STATE_SIZE (size_t)(2432)

void motion_fx_init(void);
void process_motion_fx(MFX_input_t *data_in, MFX_output_t *data_out, float delta_time);

#endif

#endif
