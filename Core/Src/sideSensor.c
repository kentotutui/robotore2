/*
 * sideSensor.c
 *
 *  Created on: 2023/07/22
 *      Author: tutui
 */

#include "sideSensor.h"

uint16_t mode;

static uint16_t goal_timer;
static bool side_sensor_l, side_sensor_r;
static bool goal_flag = false;
static bool goal_judge_flag = false;
static bool run_flag = false;
static uint8_t start_goal_line_cnt;
static bool logging_flag;

static float min_velocity, max_velocity;

uint16_t tsutsui = 0;

//white <= 1700 black >= 1700

void updateSideSensorStatus(){
	goal_timer++;

	if(goal_timer >= 10000){
		goal_timer = 10000;
	}

	if(side_sensorL <= 500){
		side_sensor_l = true;
	}
	else{
		side_sensor_l = false;
	}

	if(side_sensorR <= 500){
		side_sensor_r = true;
	}
	else{
		side_sensor_r = false;
	}
}

void setRunMode(uint16_t num){
	mode = num;
}

bool isTargetDistance(float target){
	bool ret = false;
	if(getDistance10mm() >= target){
		ret = true;
	}
	return ret;
}

void running(void)
{
	uint16_t pattern = 0;
	runningInit();
	startLineTrace();
	startVelocityControl();
	//setTargetVelocity(min_velocity);

	while(goal_flag == false){
		switch(pattern){

				  case 0:
					  if(getSideSensorStatusR() == true){
						  start_goal_line_cnt++;

						  if(mode == 1) startLogging();

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
						  stopLogging();
						  pattern = 20;
					  }

					  break;

				  case 20:

					  setTargetVelocity(-0.01);
					  HAL_Delay(20);
					  setTargetVelocity(0);
					  HAL_Delay(500);

					  goal_flag = true;

					  break;
		}

		if(getCouseOutFlag() == true)
		{
			stopLogging();
		    pattern = 20;
	    }
	}
	//HAL_Delay(2000);
	//goal_flag = false;
}

void runningFlip()
{
	if(run_flag == true){
		setLED('G');
		if(isTargetDistance(10) == true){
			tsutsui++;
			setLED2('B');
			saveLog();

			clearDistance10mm();
			clearTheta10mm();
		}
	}
}

void runningInit()
{
	if(mode == 1){
		setLED('W');
		ereaseLog();
	}
	start_goal_line_cnt = 0;
	goal_judge_flag = false;
	run_flag = true;
}

void saveLog(){
	if(logging_flag == true){
		saveDistance(getDistance10mm());
		saveTheta(getTheta10mm());
	}
}

void startLogging(){
	clearDistance10mm();
	clearTheta10mm();
	clearTotalDistance();
	logging_flag = true;
}

void stopLogging()
{
	logging_flag = false;
}

bool getgoalStatus()
{
	return goal_flag;
}

void setVelocityRange(float min_vel, float max_vel)
{
	min_velocity = min_vel;
	max_velocity = max_vel;
}

//↓sidesensorjob
bool getSideSensorStatusL()
{
	return side_sensor_l;
}

bool getSideSensorStatusR()
{
	return side_sensor_r;
}
