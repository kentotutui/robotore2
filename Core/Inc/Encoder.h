/*
 * Encoder.h
 *
 *  Created on: May 9, 2023
 *      Author: tutui
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"

void initEncoder(void);
void updateEncoderCnt(void);
void getEncoderCnt(int16_t *, int16_t *);

float getDistance10mm(void);
void clearDistance10mm(void);
float getDistance30mm(void);
void clearDistance30mm(void);
float getCrossLineIgnoreDistance(void);
float getSideLineIgnoreDistance(void);
float getTotalDistance();
float getGoalJudgeDistance();
float getSideLineJudgeDistance();
float getspeedcount();
void clearspeedcount(void);

void setTotalDistance(float);

void clearCrossLineIgnoreDistance(void);
void clearSideLineIgnoreDistance(void);
void resetEncoderCnt(void);
void clearTotalDistance();
void clearGoalJudgeDistance();
void clearSideLineJudgeDistance();


#endif /* INC_ENCODER_H_ */
