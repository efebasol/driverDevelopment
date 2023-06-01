/*
 * EXTI.c
 *
 *  Created on: 1 Haz 2023
 *      Author: efebasol
 */

#include "EXTI.h"

/*
 * @brief  GPIO_LineConfig, Configures the port and pin for SYSCFG
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
