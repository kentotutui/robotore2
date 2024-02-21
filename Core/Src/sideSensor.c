/*
 * sideSensor.c
 *
 *  Created on: 2023/07/22
 *      Author: tutui
 */

#include "sideSensor.h"

static float velocity_table[5500];
static int16_t acceleration_table[5500];

//↓モータ特性
#define WHEEL_RADIUS 0.01125 //[m]
#define AIRCRAFT_MASS 0.139 //[kg]
#define TORQUE_CONSTANT 0.00352 //[Nm/A]
#define RWSISTANCE_BETWEEN_TERMINALS 2.7 //[Ω]
#define PI 3.1415926535
#define REDUCTION_RATIO 3
#define MAX_CounterPeriod 1679

#define Power_supply_voltage 12.0 //[V] 仮定電源電圧

uint16_t velocity_table_idx;
uint16_t mode;

float ref_distance;

static uint8_t start_goal_line_cnt;
static uint16_t cross_line_idx;
static uint16_t side_line_idx;
static uint16_t correction_check_cnt_cross, correction_check_cnt_side;

static bool cross_line_ignore_flag;
static bool side_line_judge_flag = false;
static bool side_sensor_l, side_sensor_r;
static bool goal_flag = false;
static bool goal_judge_flag = false;
static bool continuous_cnt_reset_flag = false;
static bool continuous_curve_flag = false;
static bool run_flag = false;
static bool logging_flag;
static bool velocity_update_flag;

static float min_velocity, max_velocity;
static float acceleration, deceleration;
static float straight_radius;
static int16_t V_motor;

void updateSideSensorStatus(){

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

bool isCrossLine()
{
	static uint16_t cnt = 0;
	float sensor_edge_val_l = sensor[0];
	float sensor_edge_val_r = sensor[11];
	static bool flag = false;

	if(sensor_edge_val_l < 700 && sensor_edge_val_r < 700){
		cnt++;
	}
	else{
		cnt = 0;
	}

	if(cnt >= 3){
		//setLED2('Y');
		flag = true;
	}
	else{
		//setLED2('N');
		flag = false;
	}

	return flag;
}

bool isContinuousCurvature()//連続曲率判定
{
	static float pre_theta;
	static float continuous_cnt;
	bool continuous_flag = false;
	float diff_theta = fabs(pre_theta - getTheta10mm());

	if(continuous_cnt_reset_flag == true){
		continuous_cnt_reset_flag = false;
		continuous_cnt = 0;
	}

	//if(diff_theta <= 0.005) continuous_cnt++;
	//if(diff_theta <= 0.010) continuous_cnt++;
	if(diff_theta <= 0.020) continuous_cnt++;
	else continuous_cnt = 0;

	if(continuous_cnt >= 40) continuous_flag = true;

	if(continuous_cnt >= 1000) continuous_cnt = 1000;

	pre_theta = getTheta10mm();

	return continuous_flag;
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
						  else startVelocityUpdate();

						  clearGoalJudgeDistance();
						  clearSideLineJudgeDistance();
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
						  stopVelocityUpdate();
						  pattern = 20;
					  }

					  break;

				  case 20:

					  setTargetVelocity(1.0);
					  HAL_Delay(100);
					  //setTargetVelocity(-0.01);
					  //HAL_Delay(20);
					  setTargetVelocity(0);
					  HAL_Delay(500);

					  goal_flag = true;

					  break;
		}

		if(getCouseOutFlag() == true)
		{
			stopLogging();
			stopVelocityUpdate();
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
			saveLog();

			if(isContinuousCurvature() == true){
				//continuous_curve_check_cnt = 0;
				continuous_curve_flag = true;
			}

			clearDistance10mm();
			clearTheta10mm();
		}

		//--- Cross Line Process ---//
		if(isCrossLine() == true && cross_line_ignore_flag == false){ //Cross line detect
			cross_line_ignore_flag = true;
			continuous_curve_flag = true;

			clearCrossLineIgnoreDistance();
			clearSideLineIgnoreDistance();

			if(mode == 1){
				correction_check_cnt_cross = 0;
				saveCross(getTotalDistance());
			}
			else{
				correction_check_cnt_cross = 0;
				correctionTotalDistanceFromCrossLine();
				//saveDebug(getTotalDistance());
			}
		}
		else if(cross_line_ignore_flag == true && getCrossLineIgnoreDistance() >= 50){ //50
			cross_line_ignore_flag = false;
		}

		//--- Side marker Process---//
		if(getSideSensorStatusR() == true){ //Right side line detect
			side_line_judge_flag = false;
			clearSideLineJudgeDistance();
		}
		if(side_line_judge_flag == false && getSideSensorStatusL() == true && getSideLineJudgeDistance() >= 60){
			side_line_judge_flag = true;
			clearSideLineJudgeDistance();
		}
		else if(side_line_judge_flag == true && getSideLineJudgeDistance() >= 60){ //Detect side line
			clearSideLineJudgeDistance();
			side_line_judge_flag= false;

			if(continuous_curve_flag == true){
				continuous_curve_flag = false;
				continuous_cnt_reset_flag = true;

				if(mode == 1){
					correction_check_cnt_side = 0;
					saveSide(getTotalDistance());
				}
				else{
					correctionTotalDistanceFromSideLine();
					//saveDebug(getTotalDistance());
				}
			}
		}

		// Debug LED //
		correction_check_cnt_cross++;
		correction_check_cnt_side++;
		if(correction_check_cnt_cross >= 10000) correction_check_cnt_cross = 10000;
	    if(correction_check_cnt_side >= 10000) correction_check_cnt_side = 10000;

	    if(correction_check_cnt_side <= 150) setLED2('G');
	    else setLED2('N');

	    if(correction_check_cnt_side <= 150) setLED('B');
	    else setLED('G');
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

	clearCrossLineIgnoreDistance();
	clearSideLineIgnoreDistance();

	start_goal_line_cnt = 0;
	cross_line_ignore_flag = false;
	side_line_judge_flag = false;
	goal_judge_flag = false;
	continuous_cnt_reset_flag = true;
	continuous_curve_flag = false;
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
		//saveDebug(getPID());
		//saveDebug(getTargetAcceleration());
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

	cross_line_idx = 0;
	side_line_idx = 0;
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

	uint16_t crossline_idx = 0;
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

		float crossline_distance = getCrossLog(crossline_idx);
		if(crossline_distance + 60 >= total_distance && total_distance >= crossline_distance - 60){
			 velocity_table[i] = max_velocity;
		}

		if(total_distance >= crossline_distance + 60){
			crossline_idx++;
		}

	}
	for(uint16_t i = log_size; i < 6000; i++){
		velocity_table[i] = 3.0;
		//velocity_table[i] = 2.0;
	}


	addDecelerationDistanceMergin(velocity_table, 13); //8
	addAccelerationDistanceMergin(velocity_table, 5); //15
	//shiftVelocityTable(velocity_table, 1);

	velocity_table[0] = min_velocity;

	decelerateProcessing(deceleration, p_distance);
	accelerateProcessing(acceleration, p_distance);

	CreateAcceleration(p_distance);

}

float radius2Velocity(float radius){
	float velocity;

	if(mode == 2){
		velocity = radius * ((max_velocity - min_velocity) / straight_radius) + min_velocity;
	}
	else if(mode == 3){
		velocity = 1e-3 * radius * radius * ((max_velocity - min_velocity) / straight_radius) + min_velocity;
	}

	return velocity;
}

//*table：速度テーブル（一定距離ごとの目標速度が格納された配列）
//mergin_size：マージンする距離（10mmごとに速度テーブルを生成している場合，10を引数にすると100mmマージンされます）
void addDecelerationDistanceMergin(float *table, int16_t mergin_size)
{
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
		setTargetAcceleration(acceleration_table[velocity_table_idx]);

		if(pre_target_velocity > velocity_table[velocity_table_idx]){
			setClearFlagOfVelocityControlI();
		}

		pre_target_velocity = velocity_table[velocity_table_idx];
	}
}

void correctionTotalDistanceFromCrossLine()//クロスでの距離補正
{
	while(cross_line_idx <= getCrossLogSize()){
		float temp_crossline_distance = getCrossLog(cross_line_idx);
		float diff = fabs(temp_crossline_distance - getTotalDistance());
		if(diff <= 250){
			correction_check_cnt_cross = 0;
			setTotalDistance(temp_crossline_distance);
			cross_line_idx++;
			break;
		}
		cross_line_idx++;

		if(cross_line_idx >= getCrossLogSize()){
			cross_line_idx = getCrossLogSize() - 1;
			break;
		}
	}
}

void correctionTotalDistanceFromSideLine()//連続曲率後の距離補正
{
	while(side_line_idx <= getSideLogSize()){
		float temp_sideline_distance = getSideLog(side_line_idx);
		float diff = fabs(temp_sideline_distance - getTotalDistance());
		//if(diff <= 700){
		if(diff <= 250){
			correction_check_cnt_side = 0;
			setTotalDistance(temp_sideline_distance);
			side_line_idx++;
			break;
		}
		side_line_idx++;

		if(side_line_idx >= getSideLogSize()){
			side_line_idx = getSideLogSize() - 1;
			break;
		}
	}
}

void CreateAcceleration(const float *p_distance)//フィードフォワード制御計算
{
	uint16_t log_size = getDistanceLogSize();
    for(uint16_t i = 0; i <= log_size - 1; i++){
		float v_diff = velocity_table[i+1] - velocity_table[i];//目標速度ー今の速度  Δv [m/s]

		float t = p_distance[i]*1e-3 / velocity_table[i];//時間を求める mm*1e-3 → m Δt [s]
		float a = v_diff / t;//加速度計算 [m/s^2]

		float n = (60*velocity_table[i]*REDUCTION_RATIO) / (2*PI*WHEEL_RADIUS);//回転数 [rpm]
		float K_e = ((2*PI)/60) * TORQUE_CONSTANT;//逆起電力定数 [V/rpm]
		float E = K_e * n;//逆起電力 [V]
		float T_t = (AIRCRAFT_MASS*WHEEL_RADIUS*a) / (2*REDUCTION_RATIO);//タイヤにかかるトルク [Nm]
		float I = T_t / TORQUE_CONSTANT;//電流 [A]
		float V_mot = I * RWSISTANCE_BETWEEN_TERMINALS + E;//モータの出力に追加したい電圧
		float Duty = V_mot / Power_supply_voltage;//Duty比
		float Duty_motor = Duty * MAX_CounterPeriod;

		if(Duty_motor >= MAX_CounterPeriod) Duty_motor = MAX_CounterPeriod;

		V_motor = Duty_motor;

		acceleration_table[i] = V_motor;
    }
}

void CreateXYcoordinates()
{
	const float *p_distance, *p_theta;
	p_distance = getDistanceArrayPointer();
	p_theta = getThetaArrayPointer();

	float temp_distance, temp_theta, x, y, th;
	uint16_t log_size = getDistanceLogSize();

	for(uint16_t i = 0; i < log_size; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;

		x = x + temp_distance * cos(th + temp_theta/2);
		y = y + temp_distance * sin(th + temp_theta/2);
		th = th + temp_theta;
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
