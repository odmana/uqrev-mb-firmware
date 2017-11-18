/*
 * can.h
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#ifndef CAN_H_
#define CAN_H_

#include "stdint.h";

typedef union _group_64 {
	float data_fp[2];
	unsigned char data_u8[8];
	char data_8[8];
	unsigned int data_u16[4];
	int data_16[4];
	unsigned long data_u32[2];
	long data_32[2];
} group_64;

typedef struct _can_variables {
	uint32_t	 		address;
	uint32_t			length;
	group_64			data;
} can_variables;

void can_send(can_variables* can_variables);

#endif /* CAN_H_ */
