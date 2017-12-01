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


/* Offesets to fix stuff */
#ifdef MB_1
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFFFF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 7151, 0, 7151, 0, 7151, 0, 7151, 7151, 7151, 7151, 7151, 0 };
float scale_m = 1;
float scale_b = 0;
#endif
#ifdef MB_2
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFFFFFF,
		0xFFFFFFFF,
		0xFFFFFBFF,
		0xFFFFFBFF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float scale_m = 1.5;
float scale_b = 6;
#endif
#ifdef MB_3
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFFFFEF,
		0xFFFFFDEF,
		0xFFFFFDEF,
		0xFFFFFFEF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float scale_m = 1.7437;
float scale_b = 4.5713;
#endif
#ifdef MB_4
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFF7FFD,
		0xFFFF7FFD,
		0xFFFFFFFF,
		0xFFFFFFFF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float scale_m = 1.3;
float scale_b = 10;
#endif
#ifdef MB_5
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFF7FEF,
		0xFFFF76EF,
		0xFFFF7FEF,
		0xFFFF7FEF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float scale_m = 1.3;
float scale_b = 13;
#endif
#ifdef MB_6
uint32_t const CHANNEL_ENABLE[4] = {
		0xFFFFFFBF,
		0xFFFFFFBF,
		0xFFFFFFBF,
		0xFFFFFFBF
};
uint16_t const P_ON_RES[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float scale_m = 0.8767;
float scale_b = 22.336;
#endif
/* End offsets */


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;

uint32_t adc_data_full[TEMP_SENSOR_COUNT];
uint32_t adc1_data[9];
uint32_t adc2_data[7];
uint32_t adc3_data[2];
int32_t temps[TEMP_MUX_LEVELS][TEMP_SENSOR_COUNT][TEMP_AVG_LENGTH];

uint8_t adc1_data_ready = 0;
uint8_t adc2_data_ready = 0;
uint8_t adc3_data_ready = 0;

uint8_t mux_levels[TEMP_MUX_LEVELS] = { 6, 0, 7, 3 };
uint8_t current_mux_level = 0;
uint8_t current_avg_level = 0;

void log_status();
float calculate_temp(uint32_t adc_val, uint16_t offset);
void change_mux_level(uint8_t level);


void temp_process() {
	can_variables can;
	uint8_t max_level, min_level, max_cell, min_cell;
	int32_t max_temp = 0, min_temp = 100;

	if (adc1_data_ready && adc2_data_ready && adc3_data_ready) {
		// 1. Convert temps to C
		for (int i = 0; i < TEMP_SENSOR_COUNT; i++) {
			if (CHANNEL_ENABLE[current_mux_level] & (1 << i))
				temps[current_mux_level][i][current_avg_level] = (int32_t)(calculate_temp(adc_data_full[i], P_ON_RES[i]) * 100);
		}

		// 2. Find and send temp data over CAN
		for (int l = 0; l < TEMP_MUX_LEVELS; l++) {
			for (int c = 0; c < TEMP_SENSOR_COUNT; c++) {
				int32_t avg_temp = 0;
				for (int a = 0; a < TEMP_AVG_LENGTH; a++) {
					avg_temp += temps[l][c][a];
				}
				avg_temp /= TEMP_AVG_LENGTH;

				// Set max details
				if (avg_temp > max_temp) {
					max_level = l;
					max_cell = c;
					max_temp = avg_temp;
				}
				// Set min details
				if (avg_temp < min_temp) {
					min_level = l;
					min_cell = c;
					min_temp = avg_temp;
				}
			}
		}

		can.address = CAN_MB_BASE;
		can.length = 8;
		can.data.data_u8[0] = max_cell;
		can.data.data_u8[1] = max_level;
		can.data.data_16[1] = max_temp;
		can.data.data_fp[1] = max_temp;
//		can.data.data_u8[4] = min_cell;
//		can.data.data_u8[5] = min_level;
//		can.data.data_16[3] = min_temp;
		can_send(&can);


		log_status();

	} else {
		debug_log("Temp processing called before all ADC data was read [1: %d, 2: %d, 3: %d]",
				adc1_data_ready, adc2_data_ready, adc3_data_ready);
	}

	// 3. Change MUX level and avg filter level
	current_mux_level = ++current_mux_level % TEMP_MUX_LEVELS;
	change_mux_level(mux_levels[current_mux_level]);
	current_avg_level = ++current_avg_level % TEMP_AVG_LENGTH;

	// Reset flags
	adc1_data_ready = 0;
	adc2_data_ready = 0;
	adc3_data_ready = 0;

	// Restart ADC reads
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);
}


float calculate_temp(uint32_t adc_val, uint16_t offset) {
	double temp_cal;
	double res = (float)(adc_val * TEMP_FIXED_RESISTOR) / (float)(ADC_MAX - adc_val);
	res = res - offset;
	double log_res = log(res);

	//	The commented are for testing purpose - leave here
	//	temp_cal = A;
	//	temp_cal = B*log(res);
	//	temp_cal = C*pow(abs(log(res)), 3);

	//	temp_cal = 1.0 / temp_cal;

	temp_cal = 1 / (TEMP_A + TEMP_B*log_res + TEMP_C*log_res*log_res*log_res ) - TEMP_ZERO;

	/* ****** */
	temp_cal = (scale_m * temp_cal) + scale_b;

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
		debug_log_n("%d: %d, ", i, temps[current_mux_level][i][current_avg_level]);
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

	// Start timer
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	static uint8_t first_flag = 1;
//
//	// timer is triggered when it is started
//	if (first_flag) {
//		first_flag = 0;
//		return;
//	}
//
//	temp_process();

}
