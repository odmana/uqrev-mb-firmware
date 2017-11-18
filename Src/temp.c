/*
 * temp.c
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#include "temp.h"
#include "main.h"
#include "debug.h"
#include "can.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdlib.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

uint32_t adc_data_full[TEMP_SENSOR_COUNT];
uint32_t adc1_data[9];
uint32_t adc2_data[7];
uint32_t adc3_data[2];
float temps[TEMP_SENSOR_COUNT];

uint8_t adc1_data_ready = 0;
uint8_t adc2_data_ready = 0;
uint8_t adc3_data_ready = 0;

uint8_t mux_levels[TEMP_MUX_LEVELS] = { 6, 0, 7, 3 };
uint8_t current_mux_level = 0;

void log_status();
float calculate_temp(uint32_t adc_val);
void change_mux_level(uint8_t level);


void temp_process() {
	can_variables can;

	if (adc1_data_ready && adc2_data_ready && adc3_data_ready) {
		for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
			// 1. Convert temps to C
			temps[i] = calculate_temp(adc_data_full[i]);

			// 2. Send over-temps over can
			if (temps[i] >= TEMP_WARN_THRESHOLD) {
				//		can.address = 0x300;
				//		can.length = 8;
				//		can.data.data_fp[1] = 420.69f;
				//		can.data.data_u16[0] = 0xCAFE;
				//		can_send(&can);
			}
		}

		log_status();

	} else {
		debug_log("Temp processing called before all ADC data was read [1: %d, 2: %d, 3: %d]",
				adc1_data_ready, adc2_data_ready, adc3_data_ready);
	}

	// 3. Change MUX level
	current_mux_level = ++current_mux_level % TEMP_MUX_LEVELS;
	change_mux_level(mux_levels[current_mux_level]);

	// Reset flags
	adc1_data_ready = 0;
	adc2_data_ready = 0;
	adc3_data_ready = 0;

	// Restart ADC reads
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
}


float calculate_temp(uint32_t adc_val) {
	double temp_cal;
	double res = (float)(adc_val * TEMP_FIXED_RESISTOR) / (float)(ADC_MAX - adc_val);
	double log_res = log(res);

	//	The commented are for testing purpose - leave here
	//	temp_cal = A;
	//	temp_cal = B*log(res);
	//	temp_cal = C*pow(abs(log(res)), 3);

	//	temp_cal = 1.0 / temp_cal;

	temp_cal = 1 / (TEMP_A + TEMP_B*log_res + TEMP_C*log_res*log_res*log_res ) - TEMP_ZERO;

	return temp_cal;
}


void change_mux_level(uint8_t level) {
	uint8_t a = (level >> 0) & 0x01;
	uint8_t b = (level >> 1) & 0x01;
	uint8_t c = (level >> 2) & 0x01;

	HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, a);
	HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, b);
	HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, c);
}


void log_status() {
//	debug_log("ADC1 read: %d \t ADC2 read: %d \t ADC3 read: %d \t",
//			adc1_data_ready, adc2_data_ready, adc3_data_ready);

	debug_log_n("[Level %d]\t", current_mux_level);
	for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
//		debug_log_n("%d: %d, ", i, adc_data_full[i]);
		debug_log_n("%d: %d, ", i, (int32_t)(temps[i] * 100));
	}
	debug_log_n("\n");
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {					// Cell
		adc_data_full[1] = adc1_data[0];			// 1+
		adc_data_full[3] = adc1_data[1];			// 3+
		adc_data_full[5] = adc1_data[2];			// 5+
		adc_data_full[7] = adc1_data[3];			// 7+
		adc_data_full[9] = adc1_data[4];			// 9+
		adc_data_full[11] = adc1_data[5];			// 11+
		adc_data_full[13] = adc1_data[6];			// 13+
		adc_data_full[15] = adc1_data[7];			// 15+
		adc_data_full[16] = adc1_data[8];			// 16+

		adc1_data_ready++;
	} else if (hadc->Instance == ADC2) {
		adc_data_full[2] = adc2_data[0];			// 2+
		adc_data_full[4] = adc2_data[1];			// 4+
		adc_data_full[6] = adc2_data[2];			// 6+
		adc_data_full[8] = adc2_data[3];			// 8+
		adc_data_full[10] = adc2_data[4]; 			// 10+
		adc_data_full[12] = adc2_data[5];			// 12+
		adc_data_full[14] = adc2_data[6];			// 14+

		adc2_data_ready++;
	} else if (hadc->Instance == ADC3) {
		adc_data_full[17] = adc3_data[0];			// 0-
		adc_data_full[0] = adc3_data[1];			// 0+

		adc3_data_ready++;
	}
}


void temp_init() {
	// Set initial MUX level
	change_mux_level(mux_levels[current_mux_level]);

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
