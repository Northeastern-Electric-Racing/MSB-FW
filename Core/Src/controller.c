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
		write_debug1(true);
		write_debug2(true);
		break;
	case DEVICE_FRONT_RIGHT:
		write_debug1(true);
		write_debug2(false);
		break;
	case DEVICE_BACK_LEFT:
		write_debug1(false);
		write_debug2(true);
		break;
	case DEVICE_BACK_RIGHT:
		write_debug1(false);
		write_debug2(false);
		break;
	}

	for (;;) {
		osDelay(DELAY_DEBUG_LED_REFRESH);
	}
}