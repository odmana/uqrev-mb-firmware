/*
 * temp.h
 *
 *  Created on: 17Nov.,2017
 *      Author: uqracing
 */

#ifndef TEMP_H_
#define TEMP_H_

#include "stdint.h"

#define TEMP_SENSOR_COUNT 				18
#define TEMP_MUX_LEVELS 				4
#define TEMP_AVG_LENGTH 				5

// Calculator: https://edwardmallon.files.wordpress.com/2017/04/ntc-steinhart_and_hart_calculator.xls
// 2017 Sensor: NCU18XH103E6SRB
#define TEMP_A 							0.000870152266910
#define TEMP_B 							0.000254553787827
#define TEMP_C 							0.000000178336340
#define TEMP_FIXED_RESISTOR 			10000
#define TEMP_ZERO			 			273.15
#define TEMP_CLAMP						10000
#define ADC_MAX 						4096

void temp_init();
void temp_process();

#endif /* TEMP_H_ */
