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
bool isCrossLine();
bool isContinuousCurvature();
bool isTargetDistance(float);

void running();
void runningFlip();
void runningInit();

void saveLog();
void startLogging();
void stopLogging();
void startVelocityUpdate();
void stopVelocityUpdate();

void createVelocityTable();
float radius2Velocity(float);
void addDecelerationDistanceMergin(float *, int16_t);
void addAccelerationDistanceMergin(float *, int16_t);
void decelerateProcessing(const float, const float *);
void accelerateProcessing(const float, const float *);
void updateTargetVelocity();
void correctionTotalDistanceFromCrossLine();
void correctionTotalDistanceFromSideLine();

void setVelocityRange(float, float);
void setAccDec(float, float);
void setStraightRadius(float);

#endif /* INC_SIDESENSOR_H_ */
