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
#include "msb.h"
#include "monitor.h"

#include "stdio.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define CAN_MSG_QUEUE_SIZE 25 /* messages */
static osMessageQueueId_t can_outbound_queue;
static osMessageQueueId_t can_inbound_queue;

static uint16_t id_list[4] = {
	0x000, CANID_WHEEL_ANGLE, 0x000, 0x002
}; // id_list[0] is reserved for the IMU Zero CAN ID, which is added in can1_init().

extern CAN_HandleTypeDef hcan1;
extern device_loc_t device_loc;

can_t *can1;

void can1_init()
{
#ifdef CAN_ENABLE
	can1 = malloc(sizeof(can_t));
	assert(can1);

	can1->hcan = &hcan1;
	assert(!can_init(can1));

	/* Add the correct IMU Zero CAN ID to the filter depending on the location of the MSB. */
	switch (device_loc) {
	case DEVICE_FRONT_LEFT:
		id_list[0] = CANID_IMUZERO_FRONTLEFT;
		break;
	case DEVICE_FRONT_RIGHT:
		id_list[0] = CANID_IMUZERO_FRONTRIGHT;
		break;
	case DEVICE_BACK_LEFT:
		id_list[0] = CANID_IMUZERO_BACKLEFT;
		break;
	case DEVICE_BACK_RIGHT:
		id_list[0] = CANID_IMUZERO_BACKRIGHT;
		break;
	}

	assert(!can_add_filter_standard(can1, id_list));
#endif

	can_outbound_queue =
		osMessageQueueNew(CAN_MSG_QUEUE_SIZE, sizeof(can_msg_t), NULL);
	can_inbound_queue =
		osMessageQueueNew(CAN_MSG_QUEUE_SIZE, sizeof(can_msg_t), NULL);
}

/* Callback to be called when we get a CAN message */
void can1_callback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	can_msg_t new_msg;

	/* Read in CAN message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
				 new_msg.data) != HAL_OK) {
		printf("Failed to receive CAN message.\n");
		return;
	}

	new_msg.len = rx_header.DLC;

	if (rx_header.IDE == CAN_ID_EXT) {
		// If the message has an extended CAN ID, save the message accordingly.
		new_msg.id = rx_header.ExtId;
		new_msg.id_is_extended = true;
	} else {
		// If the message has a standard CAN ID, save the message accordingly.
		new_msg.id = rx_header.StdId;
		new_msg.id_is_extended = false;
	}

	osMessageQueuePut(can_inbound_queue, &new_msg, 0U, 0U);
}

osThreadId_t can_dispatch_handle;
const osThreadAttr_t can_dispatch_attributes = {
	.name = "CanDispatch",
	.stack_size = CAN_DISPATCH_STACK_SIZE,
	.priority = (osPriority_t)osPriorityRealtime5,
};

void vCanDispatch(void *pv_params)
{
	can_msg_t msg_from_queue;
#ifdef CAN_ENABLE
	HAL_StatusTypeDef msg_status;
#endif

	for (;;) {
		/* Send CAN message */
		if (osOK == osMessageQueueGet(can_outbound_queue,
					      &msg_from_queue, NULL,
					      osWaitForever)) {
#ifdef CAN_ENABLE
			msg_status = can_send_msg(can1, &msg_from_queue);
			if (msg_status != HAL_OK) {
				printf("Failed to send CAN message");
			} else if (msg_status == HAL_BUSY) {
				printf("Outbound mailbox full!");
			}
#endif
#ifdef LOG_VERBOSE
			printf("Message sent att: %lX\r\n", msg_from_queue.id);
#endif
		}

		//osDelay(DELAY_CAN_DISPATCH);
	}
}

int8_t queue_can_msg(can_msg_t msg)
{
	if (!can_outbound_queue)
		return -1;

	osMessageQueuePut(can_outbound_queue, &msg, 0U, 0U);
	return 0;
}

osThreadId_t can_receive_thread;
const osThreadAttr_t can_receive_attributes = {
	.name = "CanProcessing",
	.stack_size = 128 * 8,
	.priority = (osPriority_t)osPriorityRealtime,
};

void vCanReceive(void *pv_params)
{
	can_msg_t msg;

	for (;;) {
		while (osOK ==
		       osMessageQueueGet(can_inbound_queue, &msg, 0U, 0U)) {
			switch (msg.id) {
			case CANID_IMUZERO_BACKLEFT:
			case CANID_IMUZERO_BACKRIGHT:
			case CANID_IMUZERO_FRONTLEFT:
			case CANID_IMUZERO_FRONTRIGHT:
				imu_zero(msg.data[0], msg.data[1], msg.data[2]);
				break;
			case CANID_WHEEL_ANGLE:
				break;
			default:
				record_wheel_angle(msg.data);
				break;
			}
		}
	}
}
