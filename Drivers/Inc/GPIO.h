/*
 * GPIO.h
 *
 *  Created on: 1 May 2023
 *      Author: efebasol
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f446xx.h"

typedef struct
{
	uint32_t pinNumber;
	uint32_t Mode;
	uint32_t Otype;
	uint32_t Speed;
	uint32_t PuPd;
	uint32_t Alternate;

}GPIO_InıtTypeDef_t;






#endif /* INC_GPIO_H_ */
