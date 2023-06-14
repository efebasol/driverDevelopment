/*
 * SPI.h
 *
 *  Created on: 7 Haz 2023
 *      Author: efebasol
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f446xx.h"

typedef struct
{
	uint32_t Mode;
	uint32_t CPHA;
	uint32_t CPOL;
	uint32_t BaudRate;
	uint32_t SSM_Cmd;
	uint32_t DFF_Format;
	uint32_t BusConfig;
}SPI_InitTypeDef_t;

typedef struct
{
	SPI_TypeDef_t *Instance;
	SPI_InitTypeDef_t Init;
}SPI_HandleTypeDef_t;

#endif /* INC_SPI_H_ */
