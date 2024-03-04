/*
 * Purepursuit.h
 *
 *  Created on: 2024/03/03
 *      Author: tutui
 */

#ifndef INC_PUREPURSUIT_H_
#define INC_PUREPURSUIT_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "LineSensor.h"
#include "LED.h"
#include "VelocityCtrl.h"
#include "LineChase.h"
#include "AngleCtrl.h"
#include "Encoder.h"
#include "IMU.h"
#include "Logger.h"
#include "sideSensor.h"

void CreateXYcoordinates();
float CurrentXcoordinates(void);
float CurrentYcoordinates(void);
void PurepursuitCalculation();
void setLookaheadpoints_X(float);
void setLookaheadpoints_Y(float);

#endif /* INC_PUREPURSUIT_H_ */
