/*
 * Purepursuit.h
 *
 *  Created on: 2024/03/03
 *      Author: tutui
 */

#ifndef INC_PUREPURSUIT_H_
#define INC_PUREPURSUIT_H_

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

void CreateXYcoordinates();
void PurepursuitCalculation();

#endif /* INC_PUREPURSUIT_H_ */
