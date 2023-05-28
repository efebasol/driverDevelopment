/*
 * GPIO.c
 *
 *  Created on: 1 May 2023
 *      Author: efebasol
 */

#include "GPIO.h"

/*
 * @brief  GPIO_Init, Configures the port and pin
 * @param  GPIOx = GPIO Port Base Address
 * @param  GPIO_ InitTypeDef_t = User Config Structures
 * @retval Void
 */

void GPIO_Inıt ( GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct )
{
	uint32_t positon, fakePosition = 0, lastPosition = 0;
	uint32_t tempValue;

	for ( positon = 0; positon < 16; positon++ )
	{
		fakePosition = ( 0x1 << positon );
		lastPosition = ( uint32_t )( (GPIO_ConfigStruct->pinNumber ) & (fakePosition) );

		if ( fakePosition == lastPosition )
		{
			/* MODE CONFIG */

			tempValue = GPIOx->MODER;
			tempValue &= ~( 0x3U << ( positon * 2 ) );
			tempValue |=  ( GPIO_ConfigStruct->Mode << ( positon * 2) );
			GPIOx->MODER = tempValue;

			if ( GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT || GPIO_ConfigStruct->Mode == GPIO_MODE_AF )
			{
				/* Output Type CONFIG */

				tempValue = GPIOx->OTYPER;
				tempValue &= ~( 0x1U << positon );
				tempValue |=  ( GPIO_ConfigStruct->Otype << positon );
				GPIOx->OTYPER = tempValue;

				/* Output Speed CONFIG */

				tempValue = GPIOx->OSPEEDER;
				tempValue &= ~( 0x3U << ( positon * 2 ) );
				tempValue |=  ( GPIO_ConfigStruct->Speed << positon );
				GPIOx->OSPEEDER = tempValue;

			}

			/* Push Pull CONFIG */

			tempValue = GPIOx->PUPDR;
			tempValue &= ~( 0x3U << ( positon * 2 ) );
			tempValue |=  ( GPIO_ConfigStruct->PuPd << ( positon * 2 ) );
			GPIOx->PUPDR = tempValue;

		}
	}
}

/*
 * @brief  GPTO Write Pin, makes pin High or Low
 * @param  GPIOx = GPIO Port Base Address
 * @param  pinNumber = GPIO Pin Numbers 0 - 15
 * @param  pinState = GPIO_Pin_Set OR GPIO_Pin_Reset
 * @retval Void
 */

void GPIO_WritePin ( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber, GPIO_PinState_t pinState )
{
	if ( pinState == GPIO_Pin_Set)
	{
		GPIOx->BSRR = pinNumber;
	}
	else
	{
		GPIOx->BSRR = ( pinNumber << 16U );
	}
}

/*
 * @brief  GPIO Read Pin, reads the pin of GPIOx Port
 * @param  GPIOx = GPIO Port Base Address
 * @param  pinNumber = GPIO Pin Numbers 0 - 15
 * @retval GPIO_PinState_t
 */

GPIO_PinState_t GPIO_ReadPin ( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber )
{
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;

	if ( (GPIOx->IDR & pinNumber) != GPIO_Pin_Reset)
	{
		bitStatus = GPIO_Pin_Set;
	}

	return bitStatus;
}

/*
 * @brief  GPIO_LockPin, locks the pin of GPIOx Port
 * @param  GPIOx = GPIO Port Base Address
 * @param  pinNumber = GPIO Pin Numbers 0 - 15
 * @retval Void
 */

void GPIO_LockPin ( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber )
{
	uint32_t tempValue = ( 0x01U << 16U ) | pinNumber;

	GPIOx->LCKR = tempValue;		/* LCKR[16] = '1'	LCKR[15:0] = DATA */
	GPIOx->LCKR = pinNumber;		/* LCKR[16] = '0'	LCKR[15:0] = DATA */
	GPIOx->LCKR = tempValue;		/* LCKR[16] = '1'	LCKR[15:0] = DATA */
	tempValue = GPIOx->LCKR;		/* Read Lock Register 				  */
}

/*
 * @brief  GPIO_TogglePin, toggle the pin of GPIOx Port
 * @param  GPIOx = GPIO Port Base Address
 * @param  pinNumber = GPIO Pin Numbers 0 - 15
 * @retval Void
 */

void GPIO_TogglePin ( GPIO_TypeDef_t *GPIOx, uint16_t pinNumber )
{
	uint32_t tempODRRegister = GPIOx->ODR;

	GPIOx->BSRR = ( (tempODRRegister & pinNumber) << 16U ) | (~tempODRRegister & pinNumber);
}













