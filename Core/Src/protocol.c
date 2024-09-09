/*
 * protocol.c
 *
 *  Created on: Sep 8, 2024
 *      Author: dinhnamuet
 */

#include "protocol.h"
#include <string.h>

static struct mesh_frame last_handle_msg[3] = { 0 };

HAL_StatusTypeDef lora_mesh_push_msg(struct lora_dev *sx1278, command_t cmd,u8 addr, u8 data) {
	struct mesh_frame frame;
	frame.command		= cmd;
	frame.source_addr	= NODE_ADDRESS;
	frame.desc_addr		= addr;
	frame.data			= data;
	return lora_transmit(sx1278, (u8 *)&frame, sizeof(frame), TX_TIME_OUT);
}

static HAL_StatusTypeDef handle_cmd(struct lora_dev *sx1278, struct mesh_frame frame) {
	switch(frame.command) {
		case WRITE_DATA:
			last_handle_msg[WRITE_DATA] = frame;
			frame.command		= RESPONSE_DATA;
			frame.desc_addr		= frame.source_addr;
			frame.source_addr	= NODE_ADDRESS;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, frame.data);
			break;
		case QUERY_DATA:
			last_handle_msg[QUERY_DATA] = frame;
			frame.command		= RESPONSE_DATA;
			frame.desc_addr		= frame.source_addr;
			frame.source_addr	= NODE_ADDRESS;
			frame.data			= (u8)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
			break;
		case RESPONSE_DATA:
			last_handle_msg[RESPONSE_DATA] = frame;
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
	return lora_transmit(sx1278, (u8 *)&frame, sizeof(frame), TX_TIME_OUT);;
}

HAL_StatusTypeDef lora_handle_msg(struct lora_dev *sx1278, u8 *message, u8 size) {
	struct mesh_frame frame;
	memcpy(&frame, message, size);
	if (frame.desc_addr != NODE_ADDRESS)
		return lora_transmit(sx1278, (u8 *)&frame, sizeof(frame), TX_TIME_OUT);
	else {
		for (int i = 0; i < 3; i ++) {
			if (!memcmp(&frame, &last_handle_msg[i], sizeof(frame)))
				return HAL_ERROR;
		}
		return handle_cmd(sx1278, frame);
	}
}
