/*
 * SPI.c
 *
 *  Created on: 7 Haz 2023
 *      Author: efebasol
 */

#include "SPI.h"

/*
 * @brief  SPI_Init, Configures the SPI Peripherals
 * @param  SPI_Handle = User config structure
 * @retval Void
 */

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint32_t tempValue = 0;
	tempValue = SPI_Handle->Instance->CR1;

	tempValue |= (SPI_Handle->Init.Mode) | (SPI_Handle->Init.CPHA) | (SPI_Handle->Init.CPOL) | (SPI_Handle->Init.BaudRate) \
			  |  (SPI_Handle->Init.SSM_Cmd) | (SPI_Handle->Init.DFF_Format) | (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.FrameFormat);

	SPI_Handle->Instance->CR1 = tempValue;
}

/*
 * @brief  SPI_PerhiparelCMD, enable or disable SPI Peripherals
 * @param  SPI_Handle = User config structure
 * @param  stateOfSPI = enable or disable
 * @retval Void
 */

void SPI_PerhiparelCMD(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t stateOfSPI)
{
	if ( stateOfSPI == ENABLE )
	{
		SPI_Handle->Instance->CR1 |=  (0x1U << SPI_CR1_SPE);
	}
	else
	{
		SPI_Handle->Instance->CR1 &= ~(0x1U << SPI_CR1_SPE);
	}
}
