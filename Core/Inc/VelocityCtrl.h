/*
 * VelocityCtrl.h
 *
 *  Created on: Aug 19, 2023
 *      Author: tutui
 */

#ifndef INC_VELOCITYCTRL_H_
#define INC_VELOCITYCTRL_H_

#include "stm32f4xx_hal.h"
#include "Encoder.h"
#include "Motor.h"
#include "sideSensor.h"

void calculateVelocityControlFlip(void);
float getVelocityControlTerm(void);

float getCurrentVelocity(void);
float getTargetVelocity(void);
float getTargetAcceleration(void);
float getPID(void);
float setvariablespeed(void);
void setTargetVelocity(float);
void setTargetAcceleration(float);

void startVelocityControl(void);
void stopVelocityControl(void);

void setClearFlagOfVelocityControlI();

#endif /* INC_VELOCITYCTRL_H_ */
