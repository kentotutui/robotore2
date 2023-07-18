/*
 * motor.h
 *
 *  Created on: 2023/06/28
 *      Author: tutui
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "stm32f4xx_hal.h"

#define MAX_COUNTER_PERIOD 1679
#define MIN_COUNTER_PERIOD -1679

void initMotor(void);
void motorCtrlFlip(void);
void droneMotorCtrlFlip(void);
void setMotor(int16_t, int16_t);
void setDroneMotor(int16_t, int16_t);


#endif /* INC_MOTOR_H_ */
