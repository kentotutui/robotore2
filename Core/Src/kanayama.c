/*
 * kanayama.c
 *
 *  Created on: 2024/03/20
 *      Author: tutui
 */

#include "kanayama.h"
#include "math.h"//M_PI

static int16_t X_table[6000];
static int16_t Y_table[6000];
static int16_t Theta_table[6000];

uint16_t targetpoint_table_idx;
uint16_t debug_table_idx;
float ref_XYdistance;

static float Total_length_of_course;

static float target_X_coordinate;
static float target_Y_coordinate;
static float target_Theta;

static float now_error_x;
static float now_error_y;
static float now_error_theta;

static float Output_velocity;
static float Output_angularvelocity;

void CreateXYcoordinates()
{
	const float *p_distance, *p_theta;
	p_distance = getDistanceArrayPointer();
	p_theta = getThetaArrayPointer();

	float temp_distance, temp_theta;
	float x = 0, y = 0, th = 0;
	uint16_t log_size = getDistanceLogSize();

	for(uint16_t i = 0; i < log_size; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;

		x = x + temp_distance * cos(th + temp_theta/2);
		y = y + temp_distance * sin(th + temp_theta/2);
		th = th + temp_theta;

		X_table[i] = x * 10;
		Y_table[i] = y * 10;
		Theta_table[i] = th * 1000;

		Total_length_of_course = temp_distance + Total_length_of_course;
	}
}

float CurrentXcoordinates(void)
{
	static float pre_x;
	static float pre_th;
	float x = 0, th = 0;

	float now_distance = getDistance10mm();
	float now_X_theta = getTheta10mm();

	if(now_X_theta == 0) now_X_theta = 0.00001;

	x = pre_x + now_distance * cosf(pre_th + now_X_theta/2);
	th = pre_th + now_X_theta;

	pre_x = x;
	pre_th = th;

	return x;
}

float CurrentYcoordinates(void)
{
	static float pre_y;
	static float pre_th;
	float y = 0, th = 0;

	float now_distance = getDistance10mm();
	float now_Y_theta = getTheta10mm();

	if(now_Y_theta == 0) now_Y_theta = 0.00001;

	y = pre_y + now_distance * sinf(pre_th + now_Y_theta/2);
	th = pre_th + now_Y_theta;

	pre_y = y;
	pre_th = th;

	return y;
}

void updateTargetpoint()
{
	static float mon_X_table, mon_Y_table, mon_Theta_table;

	if(getTargetUpdateflag() == true){
		/*if(getTotalDistance() >= ref_XYdistance){
			ref_XYdistance += getDistanceLog(targetpoint_table_idx);
			targetpoint_table_idx++;
			clearDistance30mm();
		}else if(getDistance30mm() >= 30){
			ref_XYdistance += getDistanceLog(targetpoint_table_idx);
			targetpoint_table_idx++;
			clearDistance30mm();
		}*/
		if(getVLT_Distance10mm() >= 10){
			targetpoint_table_idx++;
			clearVLT_Distance10mm();
		}
		/*if(targetpoint_table_idx >= getDistanceLogSize()){
			targetpoint_table_idx = getDistanceLogSize() - 1;
			mon_Theta_table = Theta_table[targetpoint_table_idx];
			target_X_coordinate = -400;
			target_Y_coordinate = 0;
			target_Theta = mon_Theta_table / 1000;
		}
		else
		{
			mon_X_table = X_table[targetpoint_table_idx];
			mon_Y_table = Y_table[targetpoint_table_idx];
			mon_Theta_table = Theta_table[targetpoint_table_idx];

			target_X_coordinate = mon_X_table / 10;
			target_Y_coordinate = mon_Y_table / 10;
			target_Theta = mon_Theta_table / 1000;
		}*/
		mon_X_table = X_table[targetpoint_table_idx];
		mon_Y_table = Y_table[targetpoint_table_idx];
		mon_Theta_table = Theta_table[targetpoint_table_idx];

		target_X_coordinate = mon_X_table / 10;
		target_Y_coordinate = mon_Y_table / 10;
		target_Theta = mon_Theta_table / 1000;
	}
}

void Error_XY_Debug(const float now_X, const float now_Y, const float now_Theta)
{
	float X_e;
	float Y_e;
	float Theta_e;
	float sin_theta = sinf(now_Theta);
	float cos_theta = cosf(now_Theta);

	X_e = (target_X_coordinate - now_X) * cos_theta + (target_Y_coordinate - now_Y) * sin_theta;
	Y_e = -(target_X_coordinate - now_X) * sin_theta + (target_Y_coordinate - now_Y) * cos_theta;
	Theta_e = target_Theta - now_Theta;

	now_error_x = X_e;
	now_error_y = Y_e;
	now_error_theta = Theta_e;

	saveDebug(X_e);
	saveDebug(Y_e);
	saveDebug(Theta_e);
}

void Velocity_Angularvelocity(void)
{
	float kx = 0.0005, ky = 0.001, kt = 0.0002;//Kanayama Control Methodゲイン値調整 とりあえずは全部0でいいかも

	float Target_velocity = getTargetVelocity();
	float Target_angularvelocity = now_error_theta;

	Output_velocity = Target_velocity * cosf(now_error_theta) + kx * now_error_x;//車速計算(m/s)
	Output_angularvelocity = Target_angularvelocity + Target_velocity * (ky * now_error_y + kt * sinf(now_error_theta));//車体の角速度計算(rad/s)
}

float getTotal_length()
{
	return Total_length_of_course;
}

float getTargetpoint_X()
{
	return target_X_coordinate;
}

float getTargetpoint_Y()
{
	return target_Y_coordinate;
}

float getTargetpoint_Theta()
{
	return target_Theta;
}

float getOutput_velocity()
{
	return Output_velocity;
}

float getOutput_angularvelocity()
{
	return Output_angularvelocity;
}
