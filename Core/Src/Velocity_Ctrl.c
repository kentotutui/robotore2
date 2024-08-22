/*
 * Velocity_Ctrl.c
 *
 *  Created on: Aug 19, 2023
 *      Author: tutui
 */

#include "VelocityCtrl.h"

#define WHEEL_RADIUS 11.25 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 2048
#define REDUCTION_RATIO 0.33333
#define VELOCITY_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[m/s per cnt]
#define DELTA_T 0.001

static uint8_t velocity_control_enable_flag;
static uint8_t i_clear_flag;

static float velocity_control_term;
static float target_velocity;
static float target_acceleration;
static float variable_speed;

float pid_plus;

float mon_p,mon_i,mon_d = 0;

float mon_current_velocity, mon_diff;

void calculateVelocityControlFlip(void)
{
	float p, d;
	static float i;

	float kp = 2400, ki = 30000, kd = 0.0;

	float diff = 0.;
	static float pre_diff = 0.;
	float current_velocity = getCurrentVelocity();

	if(velocity_control_enable_flag == 1){
		if(i_clear_flag == 1){
			i = 0;
			i_clear_flag = 0;
		}

		diff = setvariablespeed() - current_velocity;
		//mon_diff = diff;
		p = kp * diff; //P制御
		i += ki * diff * DELTA_T; //I制御
		//d = kd * (diff - pre_diff) / DELTA_T; //D制御

		pid_plus = p + i;

		mon_p = p;
		mon_i = i;
		//mon_d = d;

		//if(i >= 1000) i = 1000;
		//if(i <= -1000) i = -1000;

		//velocity_control_term = p + i + d;//速度制御のPIDゲイン調整の時はこのコメント文をはずす

		if(getControl_Mode() == 1){
		    velocity_control_term = p + i + d;
		}
		else if(getControl_Mode() == 2){
			velocity_control_term = p + i + d;
		}
		else if(getControl_Mode() == 5){
			//velocity_control_term = (p + i + d) + target_acceleration;//速度PID ＋ フィードフォワード制御 2自由度制御
			velocity_control_term = p + i + d;
		}

		//setMotor(velocity_control_term, velocity_control_term);

		pre_diff = diff;

	}

}

float getVelocityControlTerm(void)
{
	return velocity_control_term;
}

void setTargetVelocity(float velocity)
{
	target_velocity = velocity;
}

void setTargetAcceleration(float acceleration)
{
	target_acceleration = acceleration;
}

float setvariablespeed(void)
{
	if(getspeedcount() >= target_velocity){
		variable_speed = target_velocity;
	}
	else if(getspeedcount() < target_velocity){
		variable_speed = getspeedcount();
	}

	return variable_speed;
}

float getCurrentVelocity(void)
{
	int16_t enc_l = 0, enc_r = 0;
	getEncoderCnt(&enc_l, &enc_r);
	float enc_cnt = (enc_l + enc_r) / 2;

	float current_velocity = VELOCITY_PER_CNT * enc_cnt;
	mon_current_velocity = current_velocity;

	return current_velocity;
}

float getTargetVelocity()
{
	return target_velocity;
}

float getTargetAcceleration()
{
	return target_acceleration;
}

float getPID()
{
	return pid_plus;
}

void startVelocityControl(void)
{
	velocity_control_enable_flag = 1;
	i_clear_flag = 1;
}

void stopVelocityControl(void)
{
	velocity_control_enable_flag = 0;
}

void setClearFlagOfVelocityControlI(void)
{
	i_clear_flag = 1;
}
