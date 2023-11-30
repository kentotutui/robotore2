/*
 * sideSensor.c
 *
 *  Created on: 2023/07/22
 *      Author: tutui
 */

#include "sideSensor.h"

static float velocity_table[6000];

uint16_t velocity_table_idx;
uint16_t mode;

float ref_distance;

static uint16_t goal_timer;
static bool side_sensor_l, side_sensor_r;
static bool goal_flag = false;
static bool goal_judge_flag = false;
static bool run_flag = false;
static uint8_t start_goal_line_cnt;
static bool logging_flag;
static bool velocity_update_flag;

static float min_velocity, max_velocity;
static float acceleration, deceleration;
static float straight_radius;

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
	setTargetVelocity(min_velocity);

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
		updateTargetVelocity();

		if(isTargetDistance(10) == true){
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
	else
	{
		loadDistance();
		loadTheta();
		loadCross();
		loadSide();
		createVelocityTable();

		ereaseDebugLog();
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
	else if(velocity_update_flag == true){
		saveDebug(getTargetVelocity());
		saveDebug(getCurrentVelocity());
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

void startVelocityUpdate(){
	clearDistance10mm();
	clearTotalDistance();
	velocity_table_idx = 0;
	ref_distance = 0;
	velocity_update_flag = true;
}

void stopVelocityUpdate()
{
	velocity_update_flag = false;
}

void createVelocityTable(){
	const float *p_distance, *p_theta;
	p_distance = getDistanceArrayPointer();
	p_theta = getThetaArrayPointer();
	float temp_distance, temp_theta;

	uint16_t log_size = getDistanceLogSize();

	//uint16_t crossline_idx = 0;
	float total_distance = 0;
	for(uint16_t i = 0; i < log_size; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;
		float radius = fabs(temp_distance / temp_theta);
		if(radius >= straight_radius) radius = straight_radius;
		velocity_table[i] = radius2Velocity(radius);

		//Forced maximum speed on the crossline
		total_distance += temp_distance;
		/*
		float crossline_distance = getCrossLog(crossline_idx);
		if(crossline_distance + 60 >= total_distance && total_distance >= crossline_distance - 60){
			 velocity_table[i] = max_velocity;
		}

		if(total_distance >= crossline_distance + 60){
			crossline_idx++;
		}
		*/
	}
	for(uint16_t i = log_size; i < 6000; i++){
		velocity_table[i] = 1.5;
	}


	addDecelerationDistanceMergin(velocity_table, 8); //10
	addAccelerationDistanceMergin(velocity_table, 15); //12
	//shiftVelocityTable(velocity_table, 1);

	velocity_table[0] = min_velocity;

	decelerateProcessing(deceleration, p_distance);
	accelerateProcessing(acceleration, p_distance);

}

float radius2Velocity(float radius){
	float velocity;

	if(mode == 2){
		velocity = radius * ((max_velocity - min_velocity) / straight_radius) + min_velocity;
	}
	else if(mode == 3){
		velocity = 1e-3 * radius * radius * ((max_velocity - min_velocity) / straight_radius) + min_velocity;
		/*
		if(radius < 400) velocity = min_velocity;
		else if(radius < 500) velocity = 1.5;
		else if(radius < 650) velocity = 1.8;
		else if(radius < 1500) velocity = 2.0;
		else if(radius < 2000) velocity = 2.5;
		else velocity = max_velocity;
		*/
	}

	return velocity;
}

void addDecelerationDistanceMergin(float *table, int16_t mergin_size)
{
	//setRGB('R');
	uint16_t idx = mergin_size;
	float pre_target_velocity = table[idx];

	while(idx <= 6000 - 1){
		if(pre_target_velocity > table[idx]){
			float low_velocity = table[idx];
			for(uint16_t i = idx - mergin_size; i < idx; i++){
				table[i] = low_velocity;
			}
			pre_target_velocity = table[idx];
		}

		pre_target_velocity = table[idx];

		idx++;
	}
	//setRGB('r');

}

void addAccelerationDistanceMergin(float *table, int16_t mergin_size)
{
	uint16_t idx = 0;
	float pre_target_velocity = table[idx];

	while(idx <= 6000 - 1 - mergin_size){
		if(pre_target_velocity < table[idx]){
			float low_velocity = pre_target_velocity;
			for(uint16_t i = idx; i < idx + mergin_size; i++){
				table[i] = low_velocity;
			}
			idx += mergin_size;
			pre_target_velocity = table[idx];
		}

		pre_target_velocity = table[idx];

		idx++;
	}
	//setRGB('r');

}

void decelerateProcessing(const float am, const float *p_distance){
	uint16_t log_size = getDistanceLogSize();
	for(uint16_t i = log_size - 1; i >= 1; i--){
		float v_diff = velocity_table[i-1] - velocity_table[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table[i-1] = velocity_table[i] + am * p_distance[i]*1e-3;
			}
		}
	}
}

void accelerateProcessing(const float am, const float *p_distance){
	uint16_t log_size = getDistanceLogSize();
	for(uint16_t i = 0; i <= log_size - 1; i++){
		float v_diff = velocity_table[i+1] - velocity_table[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table[i+1] = velocity_table[i] + am * p_distance[i]*1e-3;
			}
		}
	}
}

void updateTargetVelocity(){
	static float pre_target_velocity;

	if(velocity_update_flag == true){
		if(getTotalDistance() >= ref_distance){
			ref_distance += getDistanceLog(velocity_table_idx);
			velocity_table_idx++;
		}
		if(velocity_table_idx >= getDistanceLogSize()){
			velocity_table_idx = getDistanceLogSize() - 1;
		}

		setTargetVelocity(velocity_table[velocity_table_idx]);


		/*
		if(getCurrentVelocity() > velocity_table[velocity_table_idx]+0.5){
			setClearFlagOfVelocityControlI();
		}
		*/
		/*
		float diff = pre_target_velocity - velocity_table[velocity_table_idx];
		if(diff > 0.01){
			setClearFlagOfVelocityControlI();
		}
		*/

		if(pre_target_velocity > velocity_table[velocity_table_idx]){
			setClearFlagOfVelocityControlI();
		}

		pre_target_velocity = velocity_table[velocity_table_idx];
	}
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

void setAccDec(float acc, float dec)
{
	acceleration = acc;
	deceleration = dec;
}

void setStraightRadius(float radius)
{
	straight_radius = radius;
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
