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
