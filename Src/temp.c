/*
 * temp.c
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#include "temp.h"
#include "debug.h"
#include "stm32f3xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

uint32_t adc_data_full[18];
uint32_t adc1_data[9];
uint32_t adc2_data[7];
uint32_t adc3_data[2];

int adc1_data_ready = 0;
int adc2_data_ready = 0;
int adc3_data_ready = 0;

void temp_init() {
	// Start ADC 1
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_data, 9) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);
	if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);

	// Start ADC 2
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc2_data, 7) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);
	if (HAL_ADC_Start_IT(&hadc2) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);

	// Start ADC 3
	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc3_data, 2) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);
	if (HAL_ADC_Start_IT(&hadc3) != HAL_OK)
		Error_Handler();
	HAL_Delay(1);
}

void temp_process() {
	if (adc1_data_ready && adc2_data_ready && adc3_data_ready) {
		debug_log("ADC1 read: %d \t ADC2 read: %d \t ADC3 read: %d \t",
				adc1_data_ready, adc2_data_ready, adc3_data_ready);

		// 1. Convert temps to C
		// 2. Send temps over can
	} else {
		debug_log("Temp processing called before all ADC data was read [1: %d, 2: %d, 3: %d]",
				adc1_data_ready, adc2_data_ready, adc3_data_ready);
	}

	// Reset flags
	adc1_data_ready = 0;
	adc2_data_ready = 0;
	adc3_data_ready = 0;

	// Restart ADC reads
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		adc_data_full[2] = adc3_data[0];
		adc_data_full[4] = adc3_data[1];
		adc_data_full[6] = adc3_data[2];
		adc_data_full[8] = adc3_data[3];
		adc_data_full[10] = adc3_data[4];
		adc_data_full[12] = adc3_data[5];
		adc_data_full[14] = adc3_data[6];
		adc_data_full[16] = adc3_data[7];
		adc_data_full[17] = adc3_data[8];

		adc1_data_ready++;
	} else if (hadc->Instance == ADC2) {
		adc_data_full[3] = adc3_data[0];
		adc_data_full[5] = adc3_data[1];
		adc_data_full[7] = adc3_data[2];
		adc_data_full[9] = adc3_data[3];
		adc_data_full[11] = adc3_data[4];
		adc_data_full[13] = adc3_data[5];
		adc_data_full[15] = adc3_data[6];

		adc2_data_ready++;
	} else if (hadc->Instance == ADC3) {
		adc_data_full[0] = adc3_data[0];
		adc_data_full[1] = adc3_data[1];

		adc3_data_ready++;
	}
}
