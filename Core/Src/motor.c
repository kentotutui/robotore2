/*
 * motor.c
 *
 *  Created on: 2023/06/28
 *      Author: tutui
 */

#include "motor.h"

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim1;

static int16_t motor_l, motor_r;
static int16_t suction_motor;
int16_t rotation_l = 0;
int16_t rotation_r = 0;
int16_t mon_rev_l, mon_rev_r;

void initMotor(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //PWM start
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); //PWM start

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_Delay(100);
}

void motorCtrlFlip(void)
{
	int16_t motor_pwm_l, motor_pwm_r;

	if(motor_l >= 0){
		motor_pwm_l = motor_l;
		// motor1
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor_pwm_l);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	else{
		motor_pwm_l = motor_l * (-1);
		// motor1
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor_pwm_l);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}

	if(motor_r >= 0){
		motor_pwm_r = motor_r;
		// motor2
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor_pwm_r);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else{
		motor_pwm_r = motor_r * (-1);
		//motor2
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor_pwm_r);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	}
	mon_rev_l = motor_pwm_l;
	mon_rev_r = motor_pwm_r;
}

void suctionmotorCtrlFlip(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, suction_motor);
}

void setMotor(int16_t l, int16_t r)
{
	if(l >= MAX_COUNTER_PERIOD) l = MAX_COUNTER_PERIOD;
	else if(l <= MIN_COUNTER_PERIOD) l = MIN_COUNTER_PERIOD;

	if(r >= MAX_COUNTER_PERIOD) r = MAX_COUNTER_PERIOD;
	else if(r <= MIN_COUNTER_PERIOD) r = MIN_COUNTER_PERIOD;

	motor_l = l;
	motor_r = r;
}

void setsuctionMotor(int16_t suction)
{
	suction_motor = abs(suction);

	if(suction >= SUCTION_MOTOR_PERIOD) suction = SUCTION_MOTOR_PERIOD;
}
