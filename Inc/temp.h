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
#define TEMP_WARN_THRESHOLD 			65
#define TEMP_MUX_LEVELS 				4

// Calculator: https://edwardmallon.files.wordpress.com/2017/04/ntc-steinhart_and_hart_calculator.xls
// 2017 Sensor: NCU18XH103E6SRB
#define TEMP_A 							0.000790586013875
#define TEMP_B 							0.000268970072998
#define TEMP_C 							0.000000110229964
#define TEMP_FIXED_RESISTOR 			10000
#define TEMP_ZERO			 			273.15
#define ADC_MAX 						4096

void temp_init();
void temp_process();

#endif /* TEMP_H_ */
