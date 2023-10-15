/*
 * LineSensor.h
 *
 *  Created on: Apr 19, 2023
 *      Author: tutui
 */

#ifndef INC_LINESENSOR_H_
#define INC_LINESENSOR_H_


#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "LineChase.h"
#include "switch.h"
#include "main.h"
#include "LED.h"

#define LINESENSOR_ADC_NUM 12
#define SIDE_LINESENSOR_ADC_NUM 2

float sensor_coefficient[LINESENSOR_ADC_NUM];
float offset_values[LINESENSOR_ADC_NUM];

float side_sensor_coefficient[SIDE_LINESENSOR_ADC_NUM];
float side_offset_values[SIDE_LINESENSOR_ADC_NUM];

int16_t sensor[LINESENSOR_ADC_NUM];
int16_t side_sensorR;
int16_t side_sensorL;

void initADC(void);
void storeAnalogSensorBuffer(void);
void updateAnalogSensor(void);
void sensorCalibration();

#endif /* INC_LINESENSOR_H_ */
