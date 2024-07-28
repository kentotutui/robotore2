/*
 * LineSensor.c (sidesensor)
 *
 *  Created on: Apr 19, 2023
 *      Author: tutui
 */

#include "LineSensor.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
//DMA_HandleTypeDef hdma_adc2;

float max_values[LINESENSOR_ADC_NUM];
float min_values[LINESENSOR_ADC_NUM] = {1000};

float side_max_values[SIDE_LINESENSOR_ADC_NUM];
float side_min_values[SIDE_LINESENSOR_ADC_NUM] = {1000};

static uint16_t adc_value[LINESENSOR_ADC_NUM];
static uint16_t side_adc_value[SIDE_LINESENSOR_ADC_NUM];

static int16_t sensor0_buffer[10];
static int16_t sensor1_buffer[10];
static int16_t sensor2_buffer[10];
static int16_t sensor3_buffer[10];
static int16_t sensor4_buffer[10];
static int16_t sensor5_buffer[10];
static int16_t sensor6_buffer[10];
static int16_t sensor7_buffer[10];
static int16_t sensor8_buffer[10];
static int16_t sensor9_buffer[10];
static int16_t sensor10_buffer[10];
static int16_t sensor11_buffer[10];

static int16_t side_sensorR_buffer[10];
static int16_t side_sensorL_buffer[10];

static uint8_t L_index = 1;

void initADC()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) side_adc_value, SIDE_LINESENSOR_ADC_NUM);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) adc_value, LINESENSOR_ADC_NUM);

	loadSensor();
	const float *p_sensor;
	p_sensor = getSensorArrayPointer();

	for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){
		sensor_coefficient[i] = p_sensor[i*2] - p_sensor[i*2 + 1];
		offset_values[i] = p_sensor[i*2 + 1];
	}
	for(uint16_t i = LINESENSOR_ADC_NUM; i < LINESENSOR_ADC_NUM + SIDE_LINESENSOR_ADC_NUM; i++){
		side_sensor_coefficient[i - LINESENSOR_ADC_NUM] = p_sensor[i*2] - p_sensor[i*2 + 1];
		side_offset_values[i - LINESENSOR_ADC_NUM] = p_sensor[i*2 + 1];
	}
}

void storeAnalogSensorBuffer(void)
{
	/*
	sensor0_buffer[index] = adc_value[1];
	sensor1_buffer[index] = adc_value[0];
	sensor2_buffer[index] = adc_value[2];
	sensor3_buffer[index] = adc_value[3];
	sensor4_buffer[index] = adc_value[4];
	sensor5_buffer[index] = adc_value[5];
	sensor6_buffer[index] = adc_value[6];
	sensor7_buffer[index] = adc_value[7];
	sensor8_buffer[index] = adc_value[8];
	sensor9_buffer[index] = adc_value[9];
	sensor10_buffer[index] = adc_value[10];
	sensor11_buffer[index] = adc_value[11];

	side_sensorR_buffer[index] = side_adc_value[1];
	side_sensorL_buffer[index] = side_adc_value[0];*/

	sensor1_buffer[L_index] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
	sensor0_buffer[L_index] = ((adc_value[0] - offset_values[0]) / sensor_coefficient[0]) * 1000;
	sensor2_buffer[L_index] = ((adc_value[2] - offset_values[2]) / sensor_coefficient[2]) * 1000;
	sensor3_buffer[L_index] = ((adc_value[3] - offset_values[3]) / sensor_coefficient[3]) * 1000;
	sensor4_buffer[L_index] = ((adc_value[4] - offset_values[4]) / sensor_coefficient[4]) * 1000;
	sensor5_buffer[L_index] = ((adc_value[5] - offset_values[5]) / sensor_coefficient[5]) * 1000;
	sensor6_buffer[L_index] = ((adc_value[6] - offset_values[6]) / sensor_coefficient[6]) * 1000;
	sensor7_buffer[L_index] = ((adc_value[7] - offset_values[7]) / sensor_coefficient[7]) * 1000;
	sensor8_buffer[L_index] = ((adc_value[8] - offset_values[8]) / sensor_coefficient[8]) * 1000;
	sensor9_buffer[L_index] = ((adc_value[9] - offset_values[9]) / sensor_coefficient[9]) * 1000;
	sensor10_buffer[L_index] = ((adc_value[10] - offset_values[10]) / sensor_coefficient[10]) * 1000;
	sensor11_buffer[L_index] = ((adc_value[11] - offset_values[11]) / sensor_coefficient[11]) * 1000;

//	sensor[0] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[1] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[2] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[3] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[4] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[5] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[6] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[7] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[8] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[9] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[10] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
//	sensor[11] = ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;



	side_sensorR_buffer[L_index] = ((side_adc_value[1] - side_offset_values[1]) / side_sensor_coefficient[1]) * 1000;
	side_sensorL_buffer[L_index] = ((side_adc_value[0] - side_offset_values[0]) / side_sensor_coefficient[0]) * 1000;

	L_index++;
}

void updateAnalogSensor(void) {
	/*
	sensor[0] = ( sensor0_buffer[0] + sensor0_buffer[1] + sensor0_buffer[2] + sensor0_buffer[3] + sensor0_buffer[4] + sensor0_buffer[5] + sensor0_buffer[6] + sensor0_buffer[7] + sensor0_buffer[8] + sensor0_buffer[9] ) / index;
	sensor[1] = ( sensor1_buffer[0] + sensor1_buffer[1] + sensor1_buffer[2] + sensor1_buffer[3] + sensor1_buffer[4] + sensor1_buffer[5] + sensor1_buffer[6] + sensor1_buffer[7] + sensor1_buffer[8] + sensor1_buffer[9] ) / index;
	sensor[2] = ( sensor2_buffer[0] + sensor2_buffer[1] + sensor2_buffer[2] + sensor2_buffer[3] + sensor2_buffer[4] + sensor2_buffer[5] + sensor2_buffer[6] + sensor2_buffer[7] + sensor2_buffer[8] + sensor2_buffer[9] ) / index;
	sensor[3] = ( sensor3_buffer[0] + sensor3_buffer[1] + sensor3_buffer[2] + sensor3_buffer[3] + sensor3_buffer[4] + sensor3_buffer[5] + sensor3_buffer[6] + sensor3_buffer[7] + sensor3_buffer[8] + sensor3_buffer[9] ) / index;
	sensor[4] = ( sensor4_buffer[0] + sensor4_buffer[1] + sensor4_buffer[2] + sensor4_buffer[3] + sensor4_buffer[4] + sensor4_buffer[5] + sensor4_buffer[6] + sensor4_buffer[7] + sensor4_buffer[8] + sensor4_buffer[9] ) / index;
	sensor[5] = ( sensor5_buffer[0] + sensor5_buffer[1] + sensor5_buffer[2] + sensor5_buffer[3] + sensor5_buffer[4] + sensor5_buffer[5] + sensor5_buffer[6] + sensor5_buffer[7] + sensor5_buffer[8] + sensor5_buffer[9] ) / index;
	sensor[6] = ( sensor6_buffer[0] + sensor6_buffer[1] + sensor6_buffer[2] + sensor6_buffer[3] + sensor6_buffer[4] + sensor6_buffer[5] + sensor6_buffer[6] + sensor6_buffer[7] + sensor6_buffer[8] + sensor6_buffer[9] ) / index;
	sensor[7] = ( sensor7_buffer[0] + sensor7_buffer[1] + sensor7_buffer[2] + sensor7_buffer[3] + sensor7_buffer[4] + sensor7_buffer[5] + sensor7_buffer[6] + sensor7_buffer[7] + sensor7_buffer[8] + sensor7_buffer[9] ) / index;
	sensor[8] = ( sensor8_buffer[0] + sensor8_buffer[1] + sensor8_buffer[2] + sensor8_buffer[3] + sensor8_buffer[4] + sensor8_buffer[5] + sensor8_buffer[6] + sensor8_buffer[7] + sensor8_buffer[8] + sensor8_buffer[9] ) / index;
	sensor[9] = ( sensor9_buffer[0] + sensor9_buffer[1] + sensor9_buffer[2] + sensor9_buffer[3] + sensor9_buffer[4] + sensor9_buffer[5] + sensor9_buffer[6] + sensor9_buffer[7] + sensor9_buffer[8] + sensor9_buffer[9] ) / index;
	sensor[10] = ( sensor10_buffer[0] + sensor10_buffer[1] + sensor10_buffer[2] + sensor10_buffer[3] + sensor10_buffer[4] + sensor10_buffer[5] + sensor10_buffer[6] + sensor10_buffer[7] + sensor10_buffer[8] + sensor10_buffer[9] ) / index;
	sensor[11] = ( sensor11_buffer[0] + sensor11_buffer[1] + sensor11_buffer[2] + sensor11_buffer[3] + sensor11_buffer[4] + sensor11_buffer[5] + sensor11_buffer[6] + sensor11_buffer[7] + sensor11_buffer[8] + sensor11_buffer[9] ) / index;
*/
	sensor[0] =  ((adc_value[0] - offset_values[0]) / sensor_coefficient[0]) * 1000;
	sensor[1] =  ((adc_value[1] - offset_values[1]) / sensor_coefficient[1]) * 1000;
	sensor[2] =  ((adc_value[2] - offset_values[2]) / sensor_coefficient[2]) * 1000;
	sensor[3] =  ((adc_value[3] - offset_values[3]) / sensor_coefficient[3]) * 1000;
	sensor[4] =  ((adc_value[4] - offset_values[4]) / sensor_coefficient[4]) * 1000;
	sensor[5] =  ((adc_value[5] - offset_values[5]) / sensor_coefficient[5]) * 1000;
	sensor[6] =  ((adc_value[6] - offset_values[6]) / sensor_coefficient[6]) * 1000;
	sensor[7] =  ((adc_value[7] - offset_values[7]) / sensor_coefficient[7]) * 1000;
	sensor[8] =  ((adc_value[8] - offset_values[8]) / sensor_coefficient[8]) * 1000;
	sensor[9] =  ((adc_value[9] - offset_values[9]) / sensor_coefficient[9]) * 1000;
	sensor[10] = ((adc_value[10] - offset_values[10]) / sensor_coefficient[10]) * 1000;
	sensor[11] = ((adc_value[11] - offset_values[11]) / sensor_coefficient[11]) * 1000;

	side_sensorR = ( side_sensorR_buffer[0] + side_sensorR_buffer[1] + side_sensorR_buffer[2] + side_sensorR_buffer[3] + side_sensorR_buffer[4] + side_sensorR_buffer[5] + side_sensorR_buffer[6] + side_sensorR_buffer[7] + side_sensorR_buffer[8] + side_sensorR_buffer[9] ) / 10;
	side_sensorL = ( side_sensorL_buffer[0] + side_sensorL_buffer[1] + side_sensorL_buffer[2] + side_sensorL_buffer[3] + side_sensorL_buffer[4] + side_sensorL_buffer[5] + side_sensorL_buffer[6] + side_sensorL_buffer[7] + side_sensorL_buffer[8] + side_sensorL_buffer[9] ) / 10;
	for(int j=0; j<=11; j++){
		if(sensor[j] >= 1000) sensor[j] = 1000;
		if(sensor[j] <= 0) sensor[j] = 0;
	}
    L_index = 0;

}

void sensorCalibration()//センサキャリブレーションはノムさんに修正してもらいました
{
	float max_values_buffer[LINESENSOR_ADC_NUM]={0};
	float min_values_buffer[LINESENSOR_ADC_NUM]={1000};
	float side_max_values_buffer[SIDE_LINESENSOR_ADC_NUM];
    float side_min_values_buffer[SIDE_LINESENSOR_ADC_NUM];

	for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){
		max_values[i] = 00;
		min_values[i] = 1500;
		max_values_buffer[i] = 0;
		min_values_buffer[i] = 1500;
	}

	for(uint16_t i = 0; i < SIDE_LINESENSOR_ADC_NUM; i++){
		side_max_values[i] = 00;
		side_min_values[i] = 1500;
	}

	while(getSwitchStatus('L') == 1){                       //sw2

		ereaseSensorLog();
		setLED2('X');

		for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){

			max_values_buffer[i] = adc_value[i];
			min_values_buffer[i] = adc_value[i];

			if(max_values_buffer[i] > max_values[i]){
				max_values[i] = max_values_buffer[i];
			}
			if((min_values_buffer[i] < min_values[i]) ){
				min_values[i] = min_values_buffer[i];
			}
		}

		for(uint16_t i = 0; i < SIDE_LINESENSOR_ADC_NUM; i++){
			side_max_values_buffer[i] = side_adc_value[i];
			side_min_values_buffer[i] = side_adc_value[i];

			if(side_max_values_buffer[i] > side_max_values[i]){
				side_max_values[i] = side_adc_value[i];
			}
			else if(side_min_values_buffer[i] < side_min_values[i]){
				side_min_values[i] = side_adc_value[i];
			}
		}

		for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){
			sensor_coefficient[i] = max_values[i] - min_values[i];
		}
		for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){
			offset_values[i] = min_values[i];
		}

		for(uint16_t i = 0; i < SIDE_LINESENSOR_ADC_NUM; i++){
			side_sensor_coefficient[i] = side_max_values[i] - side_min_values[i];
		}
		for(uint16_t i = 0; i < SIDE_LINESENSOR_ADC_NUM; i++){
			side_offset_values[i] = side_min_values[i];
		}

		for(uint16_t i = 0; i < LINESENSOR_ADC_NUM; i++){
			saveSensor(max_values[i]);
			saveSensor(min_values[i]);
		}
		for(uint16_t i = 0; i < SIDE_LINESENSOR_ADC_NUM; i++){
			saveSensor(side_max_values[i]);
			saveSensor(side_min_values[i]);
		}
	}
}
