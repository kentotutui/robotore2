/*
 * LineChase.c
 *
 *  Created on: Jun 28, 2023
 *      Author: tutui
 */


#include "LineChase.h"

#define DELTA_T 0.001

static int8_t line_trace_enable_flag;
static uint8_t i_clear_flag;
static float line_following_term;
static bool dark_flag;

static float motor_l_Deb;
static float motor_r_Deb;

/*static float p_Deb;
static float d_Deb;
static float i_Deb;*/

static float pre_diff;

float mon_velo_term;

void calculateLineFollowingTermFlip(void){
	float p, d;
	static float i;

	float kp = 1.0, ki = 0, kd = 0.00;   //kp = 0.7, kd = 0.01  //kp = 2.8, kd = 0.05
	float diff = 0.;

	if(line_trace_enable_flag == 1){
		if(i_clear_flag == 1){
			i = 0;
			i_clear_flag = 0;
		}

		//diff = ( ( sensor[0] * 5.2 + sensor[1] * 4.4 + sensor[2] * 3.6 + sensor[3] * 2.8 + sensor[4] * 2.0 + sensor[5] * 1.4 ) / 6 ) - ( ( sensor[6] * 1.4 + sensor[7] * 2.0 + sensor[8] * 2.8 + sensor[9] * 3.6 + sensor[10] * 4.4 + sensor[11] * 5.2 ) / 6 );
		diff = ( ( sensor[0] * 3.2 + sensor[1] * 2.8 + sensor[2] * 2.4 + sensor[3] * 2.0 + sensor[4] * 1.6 + sensor[5] * 1.4 ) / 6 ) - ( ( sensor[6] * 1.4 + sensor[7] * 1.6 + sensor[8] * 2.0 + sensor[9] * 2.4 + sensor[10] * 2.8 + sensor[11] * 3.2 ) / 6 );

		p = kp * diff; //P制御
		i += ki * diff * DELTA_T; //I制御
		d = kd * (diff - pre_diff) / DELTA_T; //D制御

		line_following_term = p + i + d;

		//p_Deb = p;
		//d_Deb = d;
		//i_Deb = i;

		pre_diff = diff;
	}
}

void lineTraceFlip(void)
{
	if(line_trace_enable_flag == 1){

		float velocity_control_term = getVelocityControlTerm();

		/*
		float limit = MAX_COUNTER_PERIOD * 0.9;

		if(velocity_control_term >= limit) velocity_control_term = limit;
		else if(velocity_control_term <= -limit) velocity_control_term = -limit;

		float exceeded = 0;
		if(velocity_control_term + line_following_term >= MAX_COUNTER_PERIOD){
			exceeded = (velocity_control_term + line_following_term) - MAX_COUNTER_PERIOD;
		}
		else if(velocity_control_term - line_following_term <= -MAX_COUNTER_PERIOD){
			exceeded = -MAX_COUNTER_PERIOD - (velocity_control_term - line_following_term);
		}

		velocity_control_term -= exceeded;
		line_following_term += exceeded;
		*/

		float motor_l = velocity_control_term + line_following_term;
		float motor_r = velocity_control_term - line_following_term;

		/*
		float motor_l = line_following_term;
		float motor_r = -line_following_term;
		*/

		/*
		float motor_l = velocity_control_term ;
		float motor_r = velocity_control_term ;
		*/

		mon_velo_term = velocity_control_term;

		//motor_l_Deb = motor_l;
		//motor_r_Deb = motor_r;

		setMotor(motor_l, motor_r);

		//setMotor(500, 500);
	}
	else
	{
		setMotor(0, 0);
	}
}

void startLineTrace()
{
	line_trace_enable_flag = 1;
	i_clear_flag = 1;
}

void stopLineTrace()
{
	line_trace_enable_flag = 0;
	line_following_term = 0;
	//setMotor(0, 0);
}

void checkCourseOut(void)
{
	uint16_t all_sensor;
	static uint16_t dark_cnt;

	all_sensor = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6] + sensor[7] + sensor[8] + sensor[9] + sensor[10] + sensor[11]) / 12;
	if(all_sensor > 900){
		dark_cnt++;
	}
	else dark_cnt = 0;

	if(dark_cnt >= SENSOR_ALL_DARK) dark_flag = true;
	else dark_flag = false;

}

void debugmotor(float mon_deb_l, float mon_deb_r)
{
	motor_l_Deb = mon_deb_l;
	motor_r_Deb = mon_deb_r;
}

bool getCouseOutFlag()
{
	return dark_flag;
}

