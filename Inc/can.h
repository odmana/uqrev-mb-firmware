/*
 * can.h
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#ifndef CAN_H_
#define CAN_H_

#include "stdint.h"

#define CAN_MB_BASE		0x300

typedef union _group_64 {
	float data_fp[2];
	uint8_t data_u8[8];
	int8_t data_8[8];
	uint16_t data_u16[4];
	int16_t data_16[4];
	uint32_t data_u32[2];
	int32_t data_32[2];
} group_64;

typedef struct _can_variables {
	uint32_t	 		address;
	uint32_t			length;
	group_64			data;
} can_variables;

void can_send(can_variables* can_variables);

#endif /* CAN_H_ */
