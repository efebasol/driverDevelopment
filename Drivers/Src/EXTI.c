/*
 * EXTI.c
 *
 *  Created on: 1 Haz 2023
 *      Author: efebasol
 */

#include "EXTI.h"

/*
 * @brief  GPIO_LineConfig, Configures the port and pin for SYSCFG
 * @param  EXTI_InitStruct = User Config structure
 * @retval Void
 */

void EXTI_Init( EXTI_InitTypeDef_t *EXTI_InitStruct )
{
	uint32_t tempValue = 0;

	tempValue = (uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	EXTI->EMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	if ( EXTI_InitStruct->EXTI_LineCmd != DISABLE )
	{
		tempValue += EXTI_InitStruct->EXTI_Mode;

		*( (__IO uint32_t*)(tempValue) ) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

		if ( EXTI_InitStruct->TriggerSelection == EXTI_Trigger_RF )
		{
			EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
			EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}
		else
		{
			tempValue += EXTI_InitStruct->TriggerSelection;
			*( (__IO uint32_t*)(tempValue) ) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}

	}
	else
	{
		tempValue = (uint32_t)EXTI_BASE_ADDR;
		tempValue += EXTI_InitStruct->EXTI_Mode;

		*( (__IO uint32_t*)(tempValue) ) &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	}
}

/*
 * @brief  EXTI_Init for valid GPIO port and Line number
 * @param  PortSource = Port Value A - I @def_group PORT Values
 * @param  EXTI LineSource = Pin Numbers & Line Numbers @def_group EXTI Line Values
 * @retval Void
 */

void EXTI_LineConfig ( uint8_t PortSource, uint8_t EXTI_LineSource )
{
	uint32_t tempValue;

	tempValue = SYSCFG->EXTICR[EXTI_LineSource >> 2U];
	tempValue &= ~( 0xFU << ( (EXTI_LineSource & 0x3U) * 4) );
	tempValue = ( PortSource << ( (EXTI_LineSource & 0x3U) * 4) );

	SYSCFG->EXTICR[EXTI_LineSource >> 2U] = tempValue;
}
