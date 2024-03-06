/*
 * Purepursuit.c
 *
 *  Created on: 2024/03/03
 *      Author: tutui
 */

#include "Purepursuit.h"
#include "math.h"

static float X_table[2000];
static float Y_table[2000];

#define PI 3.1415926535

uint16_t lookaheadpoint_table_idx = 0;
float ref_XYdistance;

static float target_X_coordinate;
static float target_Y_coordinate;

bool lookaheadpoint_update_flag;

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

		//saveDebug(X_table[i]);//X_tableに値が入っていることを確認
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

void updateLookaheadpoints(){
	if(getDebugflag() == true){
		if(getTotalDistance() >= ref_XYdistance){
			ref_XYdistance += getDistanceLog(lookaheadpoint_table_idx);
			lookaheadpoint_table_idx++;
		}
		if(lookaheadpoint_table_idx >= getDistanceLogSize()){
			lookaheadpoint_table_idx = getDistanceLogSize() - 1;
		}

		target_X_coordinate = X_table[lookaheadpoint_table_idx];
		target_Y_coordinate = Y_table[lookaheadpoint_table_idx];

	}
}

float PurepursuitCalculation(void)
{
	static float ang_atan2;
	static float ang_diff;

	float now_theta = getaddTheta30mm();

	if(now_theta == 0) now_theta = 0.00001;

	ang_atan2 = atan2((target_Y_coordinate - CurrentYcoordinates()) , (target_X_coordinate - CurrentXcoordinates()));

	if(ang_atan2 <= -PI/2 && now_theta >= PI/2){
		ang_atan2 = ang_atan2 + 2*PI;
	}

	if(ang_atan2 >= PI/2 && now_theta <= -PI/2){
		ang_atan2 = ang_atan2 - 2*PI;
	}

	ang_diff = ang_atan2 - now_theta;//目標点と走行中の点の差分角度を計算する(rad)

	return ang_diff;
}

float getLookaheadpoints_X()
{
	return target_X_coordinate;
}

float getLookaheadpoints_Y()
{
	return target_Y_coordinate;
}
