#include "controller.h"
#include "msb.h"
#include "msb_conf.h"

extern device_loc_t device_loc;

osThreadId_t led_controller_handle;
const osThreadAttr_t led_controller_attributes = {
	.name = "LedController",
	.stack_size = LED_CONTROLLER_STACK_SIZE,
	.priority = (osPriority_t)osPriorityBelowNormal1,
};

void vLedController(void *pv_params)
{
	uint8_t i = 0;
	for (;;) {
		if (i % 8 == 0) {
			// occassionally oposing blink
			switch (device_loc) {
			case DEVICE_FRONT_LEFT:
				debug1_write(false);
				debug2_write(false);
				break;
			case DEVICE_FRONT_RIGHT:
				debug1_write(false);
				debug2_write(true);
				break;
			case DEVICE_BACK_LEFT:
				debug1_write(true);
				debug2_write(false);
				break;
			case DEVICE_BACK_RIGHT:
				debug1_write(true);
				debug2_write(true);
				break;
			}
		} else {
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
		}
		i++;
		osDelay(DELAY_DEBUG_LED_REFRESH);
	}
}