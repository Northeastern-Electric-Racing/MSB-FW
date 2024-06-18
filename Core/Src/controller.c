#include "controller.h"
#include "msb.h"
#include "msb_conf.h"

osThreadId_t led_controller_handle;
const osThreadAttr_t led_controller_attributes = {
	.name = "LedController",
	.stack_size = 32 * 8,
	.priority = (osPriority_t)osPriorityBelowNormal1,
};

void vLedController(void* pv_params)
{

	msb_t* msb = (msb_t*)pv_params;

	switch (*msb->device_loc) {
	case DEVICE_FRONT_LEFT:
		write_debug1(msb, true);
		write_debug2(msb, true);
		break;
	case DEVICE_FRONT_RIGHT:
		write_debug1(msb, true);
		write_debug2(msb, false);
		break;
	case DEVICE_BACK_LEFT:
		write_debug1(msb, false);
		write_debug2(msb, true);
		break;
	case DEVICE_BACK_RIGHT:
		write_debug1(msb, false);
		write_debug2(msb, false);
		break;
	}

	for (;;) {

		osDelay(DELAY_DEBUG_LED_REFRESH);
	}
}