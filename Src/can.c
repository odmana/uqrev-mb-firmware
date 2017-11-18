/*
 * can.c
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#include "can.h"
#include "debug.h"
#include "stm32f3xx_hal.h"

extern CAN_HandleTypeDef hcan;

void can_send(can_variables* can) {
	CanTxMsgTypeDef TxMsg;
	HAL_StatusTypeDef status;

	hcan.pTxMsg = &TxMsg;
	hcan.pTxMsg->StdId = can->address;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->DLC = can->length;
	hcan.pTxMsg->Data[0] = can->data.data_8[0];
	hcan.pTxMsg->Data[1] = can->data.data_8[1];
	hcan.pTxMsg->Data[2] = can->data.data_8[2];
	hcan.pTxMsg->Data[3] = can->data.data_8[3];
	hcan.pTxMsg->Data[4] = can->data.data_8[4];
	hcan.pTxMsg->Data[5] = can->data.data_8[5];
	hcan.pTxMsg->Data[6] = can->data.data_8[6];
	hcan.pTxMsg->Data[7] = can->data.data_8[7];

	status = HAL_CAN_Transmit(&hcan, 100);
	if (status != HAL_OK) {
		/* Transmition Error */
		// Error_Handler();
		debug_log("CAN transmit error");
	} else if (HAL_CAN_GetState(&hcan) != HAL_CAN_STATE_READY) {
		// Error_Handler();
		debug_log("CAN ready error");
	} else {
		debug_log("CAN packet sent");
	}
}
