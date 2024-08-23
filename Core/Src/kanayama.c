/*
 * kanayama.c
 *
 *  Created on: 2024/03/20
 *      Author: tutui
 */

#include "kanayama.h"
#include "math.h"//M_PI

static int16_t X_table[1800];
static int16_t Y_table[1800];
//static int16_t Theta_table[1800];

static uint16_t EuclideanDistance_table[1800];

static int16_t SC_X_table[1800];
static int16_t SC_Y_table[1800];
static int16_t SC_Theta_table[1800];

uint8_t Variable_Window_Moving_Average = 10;// 可変窓移動平均の調整関数ログを見ながら値を変えていく　モードで変更できるようにする
uint8_t Distance_threshold = 1;// ユークリッド距離と比較して座標を間引くための数値
uint16_t targetpoint_table_idx;
uint16_t euclideandistance_idx = 0;
uint16_t debug_table_idx;

uint16_t SC_X_tablesize = 0;

float ref_XYdistance;

static float Total_length_of_course;

static float target_X_coordinate;
static float target_Y_coordinate;
static float target_Theta;

static float now_error_x;
static float now_error_y;
static float now_error_theta;

static float Output_velocity;
static float Output_angularvelocity;

static bool check_flag;

void CreateXYcoordinates()
{
	const float *p_distance, *p_theta;
	p_distance = getDistanceArrayPointer();
	p_theta = getThetaArrayPointer();
	float temp_distance, temp_theta;
	float deltaX = 0, deltaY = 0;
	float prev_x = 0, prev_y = 0, prev_atan2 = 0;
	float prev_x2 = 0, prev_y2 = 0;
	float atan2th = 0;
	float EuclideanDistance = 0;
	float EuclideanDistance_count = 0;
	float delta_ang = 0;
	int16_t X_tablesize = 0;

	float x = 0, y = 0, th = 0;
	uint16_t log_size = getDistanceLogSize();

	for(uint16_t i = 0; i < log_size; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;

		x = x + temp_distance * cos(th + temp_theta/2);//distanceとthetaからx座標を計算
		y = y + temp_distance * sin(th + temp_theta/2);//distanceとthetaからy座標を計算
		th = th + temp_theta;//累積角度を計算

		/*deltaX = x - prev_x;
		deltaY = y - prev_y;
		atan2th = atan2(deltaY, deltaX);//座標から角度を計算

		if(i > 0){
			prev_atan2 = Theta_table[i-1] / 1000;
			delta_ang = atan2th - prev_atan2;

			if(delta_ang > M_PI){
				while(delta_ang > M_PI){
				    atan2th -= 2 * M_PI;
				    delta_ang = atan2th - prev_atan2;
				}
			}
			else if(delta_ang < -M_PI){
				while(delta_ang < -M_PI){
					atan2th += 2 * M_PI;
					delta_ang = atan2th - prev_atan2;
				}
			}
		}*/

		//EuclideanDistance = sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));//ユークリッド距離の計算

		X_table[i] = x;//int16で保存するために値を加工
		Y_table[i] = y;//int16で保存するために値を加工
		//Theta_table[i] = atan2th * 1000;//int16で保存するために値を加工
		//EuclideanDistance_table[i] = EuclideanDistance * 100;

		if(i != 0){
			X_tablesize++;
		}
	}

	for(uint16_t i = 1; i < X_tablesize; i++){
		int windowSize;
		int remaining_points = X_tablesize - i + 1;
		float mon_theta_table;

		windowSize = (i < remaining_points) ? i : remaining_points;
		windowSize = (windowSize < Variable_Window_Moving_Average) ? windowSize : Variable_Window_Moving_Average;//将来的には，ラインから?mmズレるみたいな選択ができるようにしたい

		float temp_x, temp_y;

		prev_x = temp_x;
		prev_y = temp_y;

		 if (i <= windowSize) {
			float sum_x = 0, sum_y = 0;
			for (int j = 0; j < i; j++){
				sum_x += X_table[j];
				sum_y += Y_table[j];
			}
			temp_x = sum_x / i;
			temp_y = sum_y / i;
		}
		else
		{
			float sum_x = 0, sum_y = 0;
			for (int j = i - windowSize; j < i; j++){
				sum_x += X_table[j];
				sum_y += Y_table[j];
			}
			temp_x = sum_x / windowSize;
			temp_y = sum_y / windowSize;
		}

		EuclideanDistance = sqrt((temp_x - prev_x) * (temp_x - prev_x) + (temp_y - prev_y) * (temp_y - prev_y));//ユークリッド距離の計算
		EuclideanDistance_count += EuclideanDistance;

		if(EuclideanDistance_count > Distance_threshold){

			prev_x2 = SC_X_table[i-1];
			prev_y2 = SC_Y_table[i-1];

			SC_X_table[i] = temp_x;//int16で保存するために値を加工
			SC_Y_table[i] = temp_y;//int16で保存するために値を加工

			Total_length_of_course += EuclideanDistance_count;
			EuclideanDistance_table[i] = EuclideanDistance_count * 100;

			deltaX = temp_x - prev_x2;
			deltaY = temp_y - prev_y2;
			atan2th = atan2(deltaY, deltaX);//座標から角度を計算

			prev_atan2 = SC_Theta_table[i-1] / 1000;
			delta_ang = atan2th - prev_atan2;

			if(delta_ang > M_PI){
				while(delta_ang > M_PI){
					atan2th -= 2 * M_PI;
					delta_ang = atan2th - prev_atan2;
				}
			}
			else if(delta_ang < -M_PI){
				while(delta_ang < -M_PI){
					atan2th += 2 * M_PI;
					delta_ang = atan2th - prev_atan2;
				}
			}

			SC_Theta_table[i] = atan2th * 1000;//int16で保存するために値を加工
			mon_theta_table = SC_Theta_table[i];
			//saveDebug(SC_X_table[i]);//目標のx座標
			//saveDebug(SC_Y_table[i]);//目標のy座標
			//saveDebug(EuclideanDistance_table[i] / 100);
			//saveDebug(mon_theta_table / 1000);
			EuclideanDistance_count = 0;
		}

		/*if(i > 0){
			EuclideanDistance = sqrt((temp_x - prev_x) * (temp_x - prev_x) + (temp_y - prev_y) * (temp_y - prev_y));//ユークリッド距離の計算
			EuclideanDistance_count += EuclideanDistance;

			if(EuclideanDistance_count > Distance_threshold){

				prev_x2 = SC_X_table[i-1];
				prev_y2 = SC_Y_table[i-1];

				SC_X_table[i] = temp_x;//int16で保存するために値を加工
				SC_Y_table[i] = temp_y;//int16で保存するために値を加工

				Total_length_of_course += EuclideanDistance_count;
				EuclideanDistance_table[i] = EuclideanDistance_count * 100;

				deltaX = temp_x - prev_x2;
				deltaY = temp_y - prev_y2;
				atan2th = atan2(deltaY, deltaX);//座標から角度を計算

				prev_atan2 = SC_Theta_table[i-1] / 1000;
				delta_ang = atan2th - prev_atan2;

				if(delta_ang > M_PI){
					while(delta_ang > M_PI){
						atan2th -= 2 * M_PI;
						delta_ang = atan2th - prev_atan2;
					}
				}
				else if(delta_ang < -M_PI){
					while(delta_ang < -M_PI){
						atan2th += 2 * M_PI;
						delta_ang = atan2th - prev_atan2;
					}
				}

				SC_Theta_table[i] = atan2th * 1000;//int16で保存するために値を加工
				mon_theta_table = SC_Theta_table[i];
				saveDebug(SC_X_table[i]);//目標のx座標
				saveDebug(SC_Y_table[i]);//目標のy座標
				saveDebug(EuclideanDistance_table[i] / 100);
				saveDebug(mon_theta_table / 1000);
				EuclideanDistance_count = 0;
			}
		}*/

		if(i != 0){
			SC_X_tablesize++;
		}
	}

	Total_length_of_course = Total_length_of_course;
}

float CurrentXcoordinates(void)
{
	static float pre_x;
	static float pre_th;
	float x = 0, th = 0;

	float now_distance = getDistance10mm();
	float now_X_theta = getTheta10mm();

	if(now_X_theta == 0) now_X_theta = 0.00001;

	x = pre_x + now_distance * cosf(pre_th + now_X_theta/2);
	th = pre_th + now_X_theta;

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
	float now_Y_theta = getTheta10mm();

	if(now_Y_theta == 0) now_Y_theta = 0.00001;

	y = pre_y + now_distance * sinf(pre_th + now_Y_theta/2);
	th = pre_th + now_Y_theta;

	pre_y = y;
	pre_th = th;

	return y;
}

void updateTargetpoint()
{
	static float mon_X_table, mon_Y_table, mon_Theta_table;

	if(getTargetUpdateflag() == true){
		if(getVLT_Distance10mm() >= EuclideanDistance_table[euclideandistance_idx] / 100){
			euclideandistance_idx++;
			targetpoint_table_idx++;
			clearVLT_Distance10mm();
		}
		if(targetpoint_table_idx >= getSC_X_tablesize() ){
			targetpoint_table_idx = getSC_X_tablesize() - 1;
			mon_Y_table = SC_Y_table[targetpoint_table_idx];
			mon_Theta_table = SC_Theta_table[targetpoint_table_idx];

			target_X_coordinate = -350;
			target_Y_coordinate = mon_Y_table;
			target_Theta = mon_Theta_table / 1000;
		}
		else
		{
			mon_X_table = SC_X_table[targetpoint_table_idx];
			mon_Y_table = SC_Y_table[targetpoint_table_idx];
			mon_Theta_table = SC_Theta_table[targetpoint_table_idx];

			target_X_coordinate = mon_X_table;//1nt16の値を元に戻す
			target_Y_coordinate = mon_Y_table;//1nt16の値を元に戻す
			target_Theta = mon_Theta_table / 1000;//1nt16の値を元に戻す
		}

		/*mon_X_table = X_table[targetpoint_table_idx];
		mon_Y_table = Y_table[targetpoint_table_idx];
		mon_Theta_table = Theta_table[targetpoint_table_idx];*/

		/*target_X_coordinate = mon_X_table;
		target_Y_coordinate = mon_Y_table;
		target_Theta = mon_Theta_table / 1000;*/
	}
}

void Error_XY(const float now_X, const float now_Y, const float now_Theta)
{
	float X_e;
	float Y_e;
	float Theta_e;
	float sin_theta = sinf(now_Theta);
	float cos_theta = cosf(now_Theta);

	X_e = (target_X_coordinate - now_X) * cos_theta + (target_Y_coordinate - now_Y) * sin_theta;
	Y_e = -(target_X_coordinate - now_X) * sin_theta + (target_Y_coordinate - now_Y) * cos_theta;
	Theta_e = target_Theta - now_Theta;

	now_error_x = X_e;
	now_error_y = Y_e;
	now_error_theta = Theta_e;

	saveDebug(X_e);
	saveDebug(Y_e);
	saveDebug(Theta_e);
}

void Velocity_Angularvelocity(void)//Kanayama Control Methodの計算関数 1msで回している
{
	float kx = 0.0001, ky = 0.0003, kt = 0.0003;//Kanayama Control Methodゲイン値調整 全て0でも走る
	//float max_angularvelocity = 17.2 / (180/M_PI);//max角速度制限　式　制限角度(deg)/(180/π)
	//float min_angularvelocity = - (17.2 / (180/M_PI));//min角速度制限

	float Target_velocity = getTargetVelocity();
	float Target_angularvelocity = now_error_theta;

	Output_velocity = Target_velocity * cosf(now_error_theta) + kx * now_error_x;//車速計算(m/s)
	Output_angularvelocity = Target_angularvelocity + Target_velocity * (ky * now_error_y + kt * sinf(now_error_theta));//車体の角速度計算(rad/s)

	/*if(Output_angularvelocity >= max_angularvelocity)
	{
		Output_angularvelocity = max_angularvelocity;
	}
	else if(Output_angularvelocity <= min_angularvelocity)
	{
		Output_angularvelocity = min_angularvelocity;
	}*/
}

void checkAngularvelocity(void)
{
	static float checkangularvelocity = 45 / (180/M_PI);

	if(checkangularvelocity <= now_error_theta){
		check_flag = true;
	}
	else check_flag = false;
}

float getTotal_length()
{
	return Total_length_of_course;
}

float getTargetpoint_X()
{
	return target_X_coordinate;
}

float getTargetpoint_Y()
{
	return target_Y_coordinate;
}

float getTargetpoint_Theta()
{
	return target_Theta;
}

float getOutput_velocity()
{
	return Output_velocity;
}

float getOutput_angularvelocity()
{
	return Output_angularvelocity;
}

uint16_t getSC_X_tablesize(){
	return SC_X_tablesize;
}

const uint16_t *getEuclideanDistanceArrayPointer(){
	return EuclideanDistance_table;//100倍の値が入っているため /100して使う
}

const int16_t *getSC_Theta_tableArrayPointer(){
	return SC_Theta_table;//1000倍の値が入っているため /1000して使う
}

uint16_t getEuclideanDistance_table(uint16_t idx){
	return EuclideanDistance_table[idx];
}

bool getcheckAngularvelocityFlag()
{
	return check_flag;
}
