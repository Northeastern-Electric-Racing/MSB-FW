/**
 * @file can_handler.c
 * @author Hamza Iqbal and Nick DePatie
 * @brief Source file for CAN handler
 * @version 0.1
 * @date 2023-09-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "can_handler.h"
#include "can.h"
#include "msb_conf.h"
#include "serial_monitor.h"

#include "stdio.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define CAN_MSG_QUEUE_SIZE 25 /* messages */
static osMessageQueueId_t can_outbound_queue;

can_t *can1;

void init_can1(CAN_HandleTypeDef *hcan)
{
	assert(hcan);

	can1 = malloc(sizeof(can_t));
	assert(can1);

	can1->hcan = hcan;

	assert(!can_init(can1));

	can_outbound_queue =
		osMessageQueueNew(CAN_MSG_QUEUE_SIZE, sizeof(can_msg_t), NULL);
}

osThreadId_t can_dispatch_handle;
const osThreadAttr_t can_dispatch_attributes = {
	.name = "CanDispatch",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime5,
};

void vCanDispatch(void *pv_params)
{
	can_msg_t msg_from_queue;
	HAL_StatusTypeDef msg_status;

	for (;;) {
		/* Send CAN message */
		if (osOK == osMessageQueueGet(can_outbound_queue,
					      &msg_from_queue, NULL,
					      osWaitForever)) {
			msg_status = can_send_msg(can1, &msg_from_queue);
			if (msg_status == HAL_ERROR) {
				serial_print("Failed to send CAN message");
			} else if (msg_status == HAL_BUSY) {
				serial_print("Outbound mailbox full!");
			}
#ifdef LOG_VERBOSE
			else {
				printf("Message sent: %lX\r\n",
				       msg_from_queue.id);
			}
#endif
		}

		osDelay(DELAY_CAN_DISPATCH);
	}
}

int8_t queue_can_msg(can_msg_t msg)
{
	if (!can_outbound_queue)
		return -1;

	osMessageQueuePut(can_outbound_queue, &msg, 0U, 0U);
	return 0;
}
