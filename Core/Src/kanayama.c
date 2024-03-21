/*
 * kanayama.c
 *
 *  Created on: 2024/03/20
 *      Author: tutui
 */

#include "kanayama.h"
#include "math.h"//M_PI

static float X_table[2000];
static float Y_table[2000];
static float Theta_table[2000];

uint16_t targetpoint_table_idx;
float ref_XYdistance;

static float target_X_coordinate;
static float target_Y_coordinate;
static float target_Theta;

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

		X_table[i] = x;
		Y_table[i] = y;
		Theta_table[i] = th;

		//saveDebug(X_table[i]);//X_tableに値が入っているか確認済み
	    //saveDebug(Y_table[i]);

	}
}

float CurrentXcoordinates(void)
{
	static float pre_x;
	static float pre_th;
	float x = 0, th = 0;

	float now_distance = getDistance10mm();
	float now_theta = getTheta10mm();

	if(now_theta == 0) now_theta = 0.00001;

	x = pre_x + now_distance * cos(pre_th + now_theta/2);
	th = pre_th + now_theta;

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
	float now_theta = getTheta10mm();

	if(now_theta == 0) now_theta = 0.00001;

	y = pre_y + now_distance * sin(pre_th + now_theta/2);
	th = pre_th + now_theta;

	pre_y = y;
	pre_th = th;

	return y;
}

void updateTargetpoint()
{
	if(getTargetUpdateflag() == true){
		if(getTotalDistance() >= ref_XYdistance){
			ref_XYdistance += getDistanceLog(targetpoint_table_idx);
			targetpoint_table_idx++;
			clearDistance30mm();
		}else if(getDistance30mm() >= 30){
			ref_XYdistance += getDistanceLog(targetpoint_table_idx);
			targetpoint_table_idx++;
			clearDistance30mm();
		}
		if(targetpoint_table_idx >= getDistanceLogSize()){
			targetpoint_table_idx = getDistanceLogSize() - 1;
		}
		target_X_coordinate = X_table[targetpoint_table_idx];
		target_Y_coordinate = Y_table[targetpoint_table_idx];
		target_Theta = Theta_table[targetpoint_table_idx];
	}
}

float ErrorXcoordinates(void)
{
}

float ErrorYcoodinates(void)
{
}

float ErrorTheta(void)
{
}

float getTargetpoint_X()
{
	return target_X_coordinate;
}

float getTargetpoint_Y()
{
	return target_Y_coordinate;
}

float getTargetpoint_Theta(){
	return target_Theta;
}
