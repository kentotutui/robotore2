/*
 * switch.c
 *
 *  Created on: 2023/04/13
 *      Author: tutui
 */

#include "switch.h"

uint16_t getSwitchStatus(uint8_t position)
{

	uint16_t ret = 0;

	if(position == 'R' && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == 0){                      //sw3
		ret = 1;
	}
	else if (position == 'L' && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) == 0){                 //sw2
		ret = 1;
	}

	return ret;

}
