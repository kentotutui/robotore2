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

#endif /* INC_KANAYAMA_H_ */
