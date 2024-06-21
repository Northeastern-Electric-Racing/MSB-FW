#include "controller.h"
#include "msb.h"
#include "msb_conf.h"

extern device_loc_t device_loc;

osThreadId_t led_controller_handle;
const osThreadAttr_t led_controller_attributes = {
	.name = "LedController",
	.stack_size = 32 * 8,
	.priority = (osPriority_t)osPriorityBelowNormal1,
};

void vLedController(void *pv_params)
{
	switch (device_loc) {
	case DEVICE_FRONT_LEFT:
		debug1_write(true);
		debug2_write(true);
		break;
	case DEVICE_FRONT_RIGHT:
		debug1_write(true);
		debug2_write(false);
		break;
	case DEVICE_BACK_LEFT:
		debug1_write(false);
		debug2_write(true);
		break;
	case DEVICE_BACK_RIGHT:
		debug1_write(false);
		debug2_write(false);
		break;
	}

	for (;;) {
		osDelay(DELAY_DEBUG_LED_REFRESH);
	}
}