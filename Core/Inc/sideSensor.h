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

void updateSideSensorStatus();
bool getSideSensorStatusL();
bool getSideSensorStatusR();

void running();

#endif /* INC_SIDESENSOR_H_ */
