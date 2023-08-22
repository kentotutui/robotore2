/*
 * LineChase.h
 *
 *  Created on: Jun 28, 2023
 *      Author: tutui
 */

#ifndef INC_LINECHASE_H_
#define INC_LINECHASE_H_

#define SENSOR_ALL_DARK 20

#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "LineSensor.h"
#include "main.h"
#include "VelocityCtrl.h"

void calculateLineFollowingTermFlip(void);
void lineTraceFlip(void);

float getLineFollowingTerm(void);

void startLineTrace();
void stopLineTrace();

void checkCourseOut(void);
bool getCouseOutFlag(void);

void debugmotor(float, float);

#endif /* INC_LINECHASE_H_ */
