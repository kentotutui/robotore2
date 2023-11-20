/*
 * AngleCtrl.c
 *
 *  Created on: Nov 19, 2023
 *      Author: tutui
 */

#include "AngleCtrl.h"

#define PI 3.1415926535
#define DELTA_T 0.001

static uint16_t Angle_control_enable_flag;
static uint16_t i_clear_flag;
static float Angle_diff;

static float Angle_control_term;
static float variable_Angle = 0;

void calculateAngleControlFlip(void){
	float p, d;
	static float i;

	float kp = 6000, ki = 60000, kd = 0.0;

	float diff = 0.;
	static float pre_diff = 0.;
	float current_Angle = getTheta10mm();

	if(Angle_control_enable_flag == 1){
		if(i_clear_flag == 1){
			i = 0;
			i_clear_flag = 0;
		}

		diff = setvariableAngle() - current_Angle;

		Angle_diff = diff;

		p = kp * diff; //P制御
		i += ki * diff * DELTA_T; //I制御
		d = kd * (diff - pre_diff) / DELTA_T; //D制御

		Angle_control_term = p + i + d;

		//setMotor(-Angle_control_term, Angle_control_term);

		pre_diff = diff;
	}
}

float getAngleControlTerm(void)
{
	return Angle_control_term;
}

float setvariableAngle(void){
	return variable_Angle;
}

void startAngleControl(void)
{
	Angle_control_enable_flag = 1;
	i_clear_flag = 1;
}

void stopAngleControl(void)
{
	Angle_control_enable_flag = 0;
}
