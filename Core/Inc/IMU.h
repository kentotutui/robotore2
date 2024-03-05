/*
 * IMU.h
 *
 *  Created on: Oct 29, 2023
 *      Author: tutui
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "MPU6500.h"
#include "math.h"

#define R_IMU 1 //0.03 Lowpath filter constant. The smaller it is, the more effective/


uint8_t initGyro();
void updateIMUValue();
float getOmega();
float getTheta10mm();
float getaddTheta30mm();
void clearTheta10mm();
void clearaddTheta30mm();
void IMU_average();

#endif /* INC_IMU_H_ */
