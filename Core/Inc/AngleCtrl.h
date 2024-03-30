/*
 * AngleCtrl.h
 *
 *  Created on: Nov 19, 2023
 *      Author: tutui
 */

#ifndef INC_ANGLECTRL_H_
#define INC_ANGLECTRL_H_

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "Motor.h"
#include "LED.h"
#include "sideSensor.h"

void calculateAngleControlFlip(void);

float getAngleControlTerm(void);

void startAngleControl(void);
void stopAngleControl(void);

#endif /* INC_ANGLECTRL_H_ */
