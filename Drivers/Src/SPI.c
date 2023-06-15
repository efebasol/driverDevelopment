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

	tempValue |= (SPI_Handle->Init.Mode) | (SPI_Handle->Init.CPHA) | (SPI_Handle->Init.CPOL) | (SPI_Handle->Init.BaudRate) | (SPI_Handle->Init.SSM_Cmd) | (SPI_Handle->Init.DFF_Format) | (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.FrameFormat);

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

/*
 * @brief  SPI_TransmitData, Transmit data to slave
 * @param  SPI_Handle = User config structure
 * @param  pData = Address of data to send
 * @param  sizeOfData = Length of your data in bytes
 * @retval Void
 */

void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData)
{
	if ( SPI_Handle->Init.DFF_Format == SPI_DFF_16BITS )
	{
		while ( sizeOfData > 0 )
		{
			if ( SPI_GetFlagStatus(SPI_Handle, SPI_TxE_Flag) )
			{
				SPI_Handle->Instance->DR = *( (uint16_t*)(pData) );
				pData += sizeof(uint16_t);
				sizeOfData -= 2;
			}
		}
	}
	else
	{
		while ( sizeOfData > 0 )
		{
			if ( SPI_GetFlagStatus(SPI_Handle, SPI_TxE_Flag) )
			{
				SPI_Handle->Instance->DR = *pData;
				pData += sizeof(uint8_t);
				sizeOfData--;
			}
		}
	}
	while ( SPI_GetFlagStatus(SPI_Handle, SPI_Busy_Flag) );
}

/*
 * @brief  SPI_GetFlagStatus, Return the flag of SR register
 * @param  SPI_Handle = User config structure
 * @param  SPI_Flag = flag name of SR register
 * @retval SPI_FlagStatus_t
 */

SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag)
{
	return ( SPI_Handle->Instance->SR & SPI_Flag ) ? SPI_FLAG_SET : SPI_FLAG_RESET;
}
