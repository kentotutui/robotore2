/*
 * Running.h
 *
 *  Created on: 2023/11/17
 *      Author: tutui
 */

#ifndef INC_RUNNING_H_
#define INC_RUNNING_H_

#include "stm32f4xx_hal.h"
#include "sideSensor.h"
#include "LineChase.h"
#include "Encoder.h"
#include "stdbool.h"
//#include "Logger.h"
#include "IMU.h"
#include "stdlib.h"
#include "LED.h"
#include "stdbool.h"

bool getgoalStatus();

bool isCrossLine();
bool isContinuousCurvature();

void setRunMode(uint16_t);
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
void decelerateProcessing(const float, const float *);
void accelerateProcessing(const float, const float *);
void addDecelerationDistanceMergin(float *, int16_t);
void addAccelerationDistanceMergin(float *, int16_t);
void shiftVelocityTable(float *, int16_t);
void updateTargetVelocity();
float getVelocityTableValue(uint16_t);

void correctionTotalDistanceFromCrossLine();
void correctionTotalDistanceFromSideLine();

void setVelocityRange(float, float);
void setAccDec(float, float);
void setStraightRadius(float);



#endif /* INC_RUNNING_H_ */
