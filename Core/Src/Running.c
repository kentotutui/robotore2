/*
 * Running.c
 *
 *  Created on: 2023/11/17
 *      Author: tutui
 */

#include "Running.h"

static float velocity_table[6000];

uint8_t mon_is_crossline;
uint16_t velocity_table_idx;
uint16_t mode;

float ref_distance;

static uint8_t start_goal_line_cnt;
static bool logging_flag;
static bool velocity_update_flag;
uint16_t cnt_log;

static bool cross_line_ignore_flag;
//static bool side_line_ignore_flag;
static bool goal_flag = false;
static bool goal_judge_flag = false;
static bool side_line_judge_flag = false;
static bool continuous_cnt_reset_flag = false;
static bool continuous_curve_flag = false;
static bool running_flag = false;

static uint16_t cross_line_idx;
static uint16_t side_line_idx;

static uint16_t correction_check_cnt_cross, correction_check_cnt_side;
static uint16_t continuous_curve_check_cnt;

static float min_velocity, max_velocity;
static float acceleration, deceleration;
static float straight_radius;

float mon_diff_theta;

void running(void)
{
	uint16_t pattern = 0;
	startLineTrace();
	startVelocityControl();
	//setTargetVelocity(min_velocity);

	while(goal_flag == false){
		switch(pattern){

				  case 0:
					  if(getSideSensorStatusR() == true){
						  start_goal_line_cnt++;
						  clearGoalJudgeDistance();
						  pattern = 5;
					  }
					  break;

				  case 5:
					  if(getSideSensorStatusR() == false) pattern = 10;

				  case 10:
					  if(getSideSensorStatusL() == true){ //Leght side line detect
						  goal_judge_flag = false;
						  clearGoalJudgeDistance();
					  }

					  if(goal_judge_flag == false && getSideSensorStatusR() == true &&  getGoalJudgeDistance() >= 70){
						  goal_judge_flag = true;
						  clearGoalJudgeDistance();
					  }

					  else if(goal_judge_flag == true && getGoalJudgeDistance() >= 70){
						  start_goal_line_cnt++;
						  goal_judge_flag = false;
						  clearGoalJudgeDistance();
					  }

					  if(start_goal_line_cnt >= 2){
						  pattern = 20;
					  }

					  break;

				  case 20:

					  goal_flag = true;

					  break;
		}

		if(getCouseOutFlag() == true)
		{
		    pattern = 20;
	    }
	}
}

bool getgoalStatus()
{
	return goal_flag;
}
