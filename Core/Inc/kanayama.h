/*
 * kanayama.h
 *
 *  Created on: 2024/03/20
 *      Author: tutui
 */

#ifndef INC_KANAYAMA_H_
#define INC_KANAYAMA_H_

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
void updateTargetpoint();
float ErrorXcoordinates(void);
float ErrorYcoordinates(void);
float ErrorTheta(void);

float getTargetpoint_X();
float getTargetpoint_Y();
float getTargetpoint_Theta();

#endif /* INC_KANAYAMA_H_ */
