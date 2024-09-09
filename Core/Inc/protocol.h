/*
 * protocol.h
 *
 *  Created on: Sep 8, 2024
 *      Author: dinhnamuet
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_
#include "sx1278.h"

#define NODE_ADDRESS	0x1234
#define TX_TIME_OUT		1000

typedef enum {
	QUERY_DATA,
	WRITE_DATA,
	RESPONSE_DATA
} command_t;

#pragma pack(1)
struct mesh_frame {
	u32 source_addr;
	u32 desc_addr;
	command_t command;
	u8 data;
};
#pragma pack()

HAL_StatusTypeDef lora_mesh_push_msg(struct lora_dev *sx1278, command_t cmd, u8 addr, u8 data);
HAL_StatusTypeDef lora_handle_msg(struct lora_dev *sx1278, u8 *message, u8 size);

#endif /* INC_PROTOCOL_H_ */
