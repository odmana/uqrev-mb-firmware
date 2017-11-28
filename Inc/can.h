/*
 * can.h
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#ifndef CAN_H_
#define CAN_H_

#include "stdint.h"
#include "main.h"

#ifdef MB_1
#define CAN_MB_BASE		0x5E1
#endif
#ifdef MB_2
#define CAN_MB_BASE		0x5E2
#endif
#ifdef MB_3
#define CAN_MB_BASE		0x5E3
#endif
#ifdef MB_4
#define CAN_MB_BASE		0x5E4
#endif
#ifdef MB_5
#define CAN_MB_BASE		0x5E5
#endif
#ifdef MB_6
#define CAN_MB_BASE		0x5E6
#endif

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
