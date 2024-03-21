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
