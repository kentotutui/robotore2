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
#include "kanayama.h"

void updateSideSensorStatus();
bool getSideSensorStatusL();
bool getSideSensorStatusR();
bool getgoalStatus();
bool getTargetUpdateflag();

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
void startTargetUpdate();
void stopTargetUpdate();
void startVelocityUpdate();
void stopVelocityUpdate();

void CreateVelocityTable();
float radius2Velocity(float);
void addDecelerationDistanceMergin(float *, int16_t);
void addAccelerationDistanceMergin(float *, int16_t);
void decelerateProcessing(const float, const uint16_t *);
void accelerateProcessing(const float, const uint16_t *);
void updateTargetVelocity();
void correctionTotalDistanceFromCrossLine();
void correctionTotalDistanceFromSideLine();
void CreateAcceleration(const uint16_t *);
void CreateXYcoordinates();

void setVelocityRange(float, float);
void setAccDec(float, float);
void setStraightRadius(float);
float getRunMode();
float getControl_Mode();
float getMaxvelocity();
float getMinvelocity();

#endif /* INC_SIDESENSOR_H_ */
