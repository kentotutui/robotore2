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
void Error_XY(const float, const float, const float);
void Velocity_Angularvelocity(void);

float getTotal_length();
float getTargetpoint_X();
float getTargetpoint_Y();
float getTargetpoint_Theta();
float getOutput_velocity();
float getOutput_angularvelocity();

#endif /* INC_KANAYAMA_H_ */
