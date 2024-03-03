/*
 * Purepursuit.c
 *
 *  Created on: 2024/03/03
 *      Author: tutui
 */

#include "Purepursuit.h"

static float X_table[2000];
static float Y_table[2000];

uint16_t lookaheadpoint_table_idx;
float ref_points;

static bool lookaheadpoint_update_flag;

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

		saveDebug(x);
	    saveDebug(y);
	}
}

void updateLookaheadpoints(){
	if(lookaheadpoint_update_flag == true){
		if(getTotalDistance() >= ref_points){

		}
	}
}

void PurepursuitCalculation()
{

}
