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
#include "VelocityCtrl.h"
#include "LineChase.h"

void updateSideSensorStatus();
bool getSideSensorStatusL();
bool getSideSensorStatusR();
bool getgoalStatus();

void running();

void setVelocityRange(float, float);

#endif /* INC_SIDESENSOR_H_ */
