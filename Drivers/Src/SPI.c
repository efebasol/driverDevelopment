/*
 * SPI.c
 *
 *  Created on: 7 Haz 2023
 *      Author: efebasol
 */

#include "SPI.h"

static void SPI_CloseISR_TX(SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->CR2 &= ~(0x1U << SPI_CR2_TXEIE);
	SPI_Handle->TxDataSize = 0;
	SPI_Handle->pTxDataAddr = NULL;
	SPI_Handle->busStateTX = SPI_BUS_FREE;
}

static void SPI_TransmitHelper_16Bits(SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->DR = *( (uint16_t*)(SPI_Handle->pTxDataAddr) );
	SPI_Handle->pTxDataAddr += sizeof(uint16_t);
	SPI_Handle->TxDataSize -= 2;

	if (SPI_Handle->TxDataSize == 0)
	{
		SPI_CloseISR_TX(SPI_Handle);
	}
}

static void SPI_TransmitHelper_8Bits(SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->DR = *( (uint8_t*)(SPI_Handle->pTxDataAddr) );
	SPI_Handle->pTxDataAddr += sizeof(uint8_t);
	SPI_Handle->TxDataSize--;

	if (SPI_Handle->TxDataSize == 0)
	{
		SPI_CloseISR_TX(SPI_Handle);
	}
}


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
 * @brief  SPI_ReciveData, Receive data from slave
 * @param  SPI_Handle = User config structure
 * @param  pBuffer = Address of data to store the values that I get
 * @param  sizeOfData = Length of your data in bytes
 * @retval Void
 */

void SPI_ReciveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData)
{
	if ( SPI_Handle->Init.DFF_Format == SPI_DFF_16BITS )
	{
		while ( sizeOfData > 0 )
		{
			if ( SPI_GetFlagStatus(SPI_Handle, SPI_RxNE_Flag) )
			{
				*( (uint16_t*)(pBuffer) ) = SPI_Handle->Instance->DR;
				pBuffer += sizeof(uint16_t);
				sizeOfData -= 2;
			}
		}
	}
	else
	{
		while ( sizeOfData > 0 )
		{
			if ( SPI_GetFlagStatus(SPI_Handle, SPI_RxNE_Flag) )
			{
				*(pBuffer) = *( ( __IO uint8_t*)(&SPI_Handle->Instance->DR) );
				pBuffer += sizeof(uint8_t);
				sizeOfData--;
			}
		}
	}
}

/*
 * @brief  SPI_GetFlagStatus, Return the flag of SR register
 * @param  SPI_Handle = User config structure
 * @param  SPI_Flag = flag name of SR register
 * @retval SPI_FlagStatus_t
 */

void SPI_TransmitData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData)
{
	SPI_BusStatus_t SPI_BusState = SPI_Handle->busStateTX;

	if ( SPI_BusState != SPI_BUS_BUSY_TX )
	{
		SPI_Handle->pTxDataAddr = (uint8_t*)pData;
		SPI_Handle->TxDataSize = (uint16_t)sizeOfData;
		SPI_Handle->busStateTX = SPI_BUS_BUSY_TX;

		if ( (SPI_Handle->Instance->CR1) & (0x1U << SPI_CR1_DFF) )
		{
			SPI_Handle->TxISRFunction = SPI_TransmitHelper_16Bits;
		}
		else
		{
			SPI_Handle->TxISRFunction = SPI_TransmitHelper_8Bits;
		}

		SPI_Handle->Instance->CR2 |= (0x1U << SPI_CR2_TXEIE);
		SPI_Handle->busStateTX = SPI_BUS_FREE;
	}
}

void SPI_InterruptHandler(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint8_t InterruptSource = 0, InterruptFlag = 0;

	InterruptSource = SPI_Handle->Instance->CR2 & (0x1U << SPI_CR2_TXEIE);
	InterruptFlag = SPI_Handle->Instance->SR & (0x1U << SPI_SR_TxE);

	if ( (InterruptSource != 0) && (InterruptFlag != 0) )
	{
		SPI_Handle->TxISRFunction(SPI_Handle);
	}
}

SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag)
{
	return ( SPI_Handle->Instance->SR & SPI_Flag ) ? SPI_FLAG_SET : SPI_FLAG_RESET;
}
