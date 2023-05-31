 /*
 * EXTI.h
 *
 *  Created on: 1 Haz 2023
 *      Author: efebasol
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f446xx.h"

#define EXTI_PortSource_GPIOA					( (uint8_t)(0x0) )
#define EXTI_PortSource_GPIOB					( (uint8_t)(0x1) )
#define EXTI_PortSource_GPIOC					( (uint8_t)(0x2) )
#define EXTI_PortSource_GPIOD					( (uint8_t)(0x3) )
#define EXTI_PortSource_GPIOE					( (uint8_t)(0x4) )
#define EXTI_PortSource_GPIOF					( (uint8_t)(0x5) )
#define EXTI_PortSource_GPIOG					( (uint8_t)(0x6) )
#define EXTI_PortSource_GPIOH					( (uint8_t)(0x7) )

void EXTI_LineConfig ( uint8_t PortSOurce, uint8_t EXTI_LineSource );

#endif /* INC_EXTI_H_ */
