/*
 * sideSensor.h
 *
 *  Created on: 2023/07/22
 *      Author: tutui
 */

#ifndef INC_SIDESENSOR_H_
#define INC_SIDESENSOR_H_

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

void updateSideSensorStatus();
bool getSideSensorStatusL();
bool getSideSensorStatusR();
bool getgoalStatus();

void setRunMode(uint16_t);
bool isTargetDistance(float);

void running();
void runningFlip();
void runningInit();

void saveLog();
void startLogging();
void stopLogging();

void setVelocityRange(float, float);

#endif /* INC_SIDESENSOR_H_ */
