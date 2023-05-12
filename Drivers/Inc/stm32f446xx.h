/*
 * stm32f446xx.h
 *
 *  Created on: 28 Nis 2023
 *      Author: efebasol
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __IO volatile
#define SET_BIT(REG, BIT)					( (REG) |= (BIT) )
#define CLEAR_BIT(REG, BIT)					( (REG) &= ~(BIT) )
#define READ_BIT(REG, BIT)					( (REG) & (BIT) )
#define UNUSED(x)							(void)x

/*
 * Memory Base Address
 */

#define FLASH_BASE_ADDR 					(0x08000000UL) 						/* Flash Base Address (up to 512 KB)	*/
#define SRAM1_BASE_ADDR						(0x20000000UL) 						/* Sram1 Base Address (112 KB)			*/
#define SRAM2_BASE_ADDR						(0x2001C000UL) 						/* Sram2 Base Address (16 KB)			*/

/*
 * Peripheral Base Address
 */

#define PERIPH_BASE_ADDR 					(0x40000000UL) 						/* Base address of all peripherals	*/

#define APB1_BASE_ADDR						PERIPH_BASE_ADDR					/* APB1 Base domain address			*/
#define APB2_BASE_ADDR						(PERIPH_BASE_ADDR + 0x00010000UL)	/* APB2 Base domain address			*/
#define AHB1_BASE_ADDR						(PERIPH_BASE_ADDR + 0x00020000UL) 	/* AHB1 Base domain address			*/
#define AHB2_BASE_ADDR						(PERIPH_BASE_ADDR + 0x10000000UL) 	/* AHB2 Base domain address			*/

/*
 * APB1 Peripheral Base Address
 */

#define TIM2_BASE_ADDR						(APB1_BASE_ADDR + 0x0000UL)		/* Timer2 Base address		*/
#define TIM3_BASE_ADDR						(APB1_BASE_ADDR + 0x0400UL)		/* Timer3 Base address		*/
#define TIM4_BASE_ADDR						(APB1_BASE_ADDR + 0x0800UL)		/* Timer4 Base address		*/

#define SPI2_BASE_ADDR 						(APB1_BASE_ADDR + 0x3800UL) 	/* SPI2 Base address		*/
#define SPI3_BASE_ADDR 						(APB1_BASE_ADDR + 0x3C00UL) 	/* SPI3 Base address		*/

#define USART2_BASE_ADDR 					(APB1_BASE_ADDR + 0x4400UL)		/* USART2 Base address		*/
#define USART3_BASE_ADDR 					(APB1_BASE_ADDR + 0x4800UL)		/* USART3 Base address 		*/

#define UART4_BASE_ADDR 					(APB1_BASE_ADDR + 0x4C00UL)		/* UART4 Base address		*/
#define UART5_BASE_ADDR 					(APB1_BASE_ADDR + 0x5000UL)		/* UART5 Base address		*/

#define I2C1_BASE_ADDR						(APB1_BASE_ADDR + 0x5400UL)		/* I2C1 Base address		*/
#define I2C2_BASE_ADDR						(APB1_BASE_ADDR + 0x5800UL)		/* I2C2 Base address		*/
#define I2C3_BASE_ADDR						(APB1_BASE_ADDR + 0x5C00UL)		/* I2C3 Base address		*/

/*
 * APB2 Peripheral Base Address
 */

#define TIM1_BASE_ADDR 						(APB2_BASE_ADDR + 0x0000UL)		/* TIM1 Base address	*/
#define TIM8_BASE_ADDR 						(APB2_BASE_ADDR + 0x0400UL)		/* TIM8 Base address	*/

#define USART1_BASE_ADDR 					(APB2_BASE_ADDR + 0x1000UL)		/* USART1 Base address	*/
#define USART6_BASE_ADDR 					(APB2_BASE_ADDR + 0x1400UL)		/* USART6 Base address	*/

#define SPI1_BASE_ADDR 						(APB2_BASE_ADDR + 0x3400UL)		/* SPI1 Base address	*/
#define SPI4_BASE_ADDR 						(APB2_BASE_ADDR + 0x3400UL)		/* SPI4 Base address	*/

#define SYSCFG_BASE_ADDR 					(APB2_BASE_ADDR + 0x3800UL)		/* SYSCFG Base address	*/
#define EXTI_BASE_ADDR 						(APB2_BASE_ADDR + 0x3C00UL)		/* EXTI Base address	*/

/*
 * AHB1 Peripheral Base Address
 */

#define GPIOA_BASE_ADDR						(AHB1_BASE_ADDR + 0x0000UL)  	/* GPIOA Base address	*/
#define GPIOB_BASE_ADDR						(AHB1_BASE_ADDR + 0x0400UL)  	/* GPIOB Base address	*/
#define GPIOC_BASE_ADDR						(AHB1_BASE_ADDR + 0x0800UL)  	/* GPIOC Base address	*/
#define GPIOD_BASE_ADDR						(AHB1_BASE_ADDR + 0x0C00UL)  	/* GPIOD Base address	*/
#define GPIOE_BASE_ADDR						(AHB1_BASE_ADDR + 0x1000UL)  	/* GPIOE Base address	*/
#define GPIOF_BASE_ADDR						(AHB1_BASE_ADDR + 0x1400UL)  	/* GPIOF Base address	*/
#define GPIOG_BASE_ADDR						(AHB1_BASE_ADDR + 0x1800UL)  	/* GPIOG Base address	*/
#define GPIOH_BASE_ADDR						(AHB1_BASE_ADDR + 0x1C00UL)  	/* GPIOH Base address	*/

#define RCC_BASE_ADDR						(AHB1_BASE_ADDR + 0x3800UL)		/* RCC Base Address 	*/

/*
 * Peripheral Structure Definitions
 */

typedef struct
{
	__IO uint32_t MODER;					/*!< GPIO port mode register address 						offset = 0x0000 	*/
	__IO uint32_t OTYPER;					/*!< GPIO port output type register address 				offset = 0x0004		*/
	__IO uint32_t OSPEEDER;					/*!< GPIO port output speed register address 				offset = 0x0008		*/
	__IO uint32_t PUPDR;					/*!< GPIO port pull-up/pull-down register address 			offset = 0x000C		*/
	__IO uint32_t IDR;						/*!< GPIO port input data register address 					offset = 0x0010		*/
	__IO uint32_t ODR;						/*!< GPIO port output data register	address 				offset = 0x0014		*/
	__IO uint32_t BSRR;						/*!< GPIO port bit set/reset register address 				offset = 0x0018		*/
	__IO uint32_t LCKR;						/*!< GPIO port configuration lock register address 			offset = 0x001C		*/
	__IO uint32_t AFR[2];					/*!< GPIO alternate function low/high register address 		offset = 0x0020-24	*/
}GPIO_TypeDef_t;


#define GPIOA 								( (GPIO_TypeDef_t *)(GPIOA_BASE_ADDR) )
#define GPIOB 								( (GPIO_TypeDef_t *)(GPIOB_BASE_ADDR) )
#define GPIOC 								( (GPIO_TypeDef_t *)(GPIOC_BASE_ADDR) )
#define GPIOD 								( (GPIO_TypeDef_t *)(GPIOD_BASE_ADDR) )
#define GPIOE 								( (GPIO_TypeDef_t *)(GPIOE_BASE_ADDR) )
#define GPIOF 								( (GPIO_TypeDef_t *)(GPIOF_BASE_ADDR) )
#define GPIOG 								( (GPIO_TypeDef_t *)(GPIOG_BASE_ADDR) )
#define GPIOH 								( (GPIO_TypeDef_t *)(GPIOH_BASE_ADDR) )

/*
 * Reset and Clock Control Structure Definitions
 */

typedef struct
{
	__IO uint32_t CR;           			/*!< RCC clock control register                                   Address offset: 0x00 */
	__IO uint32_t PLLCFGR;    				/*!< RCC PLL configuration register                               Address offset: 0x04 */
	__IO uint32_t CFGR;         			/*!< RCC clock configuration register                             Address offset: 0x08 */
	__IO uint32_t CIR; 			            /*!< RCC clock interrupt register                                 Address offset: 0x0C */
	__IO uint32_t AHB1RSTR;      			/*!< RCC AHB1 peripheral reset register                           Address offset: 0x10 */
	__IO uint32_t AHB2RSTR;      			/*!< RCC AHB2 peripheral reset register                           Address offset: 0x14 */
	__IO uint32_t AHB3RSTR;  			    /*!< RCC AHB3 peripheral reset register                           Address offset: 0x18 */
	__IO uint32_t RESERVED0;   				/*!< Reserved 0x1C                                                                    */
	__IO uint32_t APB1RSTR;    				/*!< RCC APB1 peripheral reset register                           Address offset: 0x20 */
	__IO uint32_t APB2RSTR;      			/*!< RCC APB2 peripheral reset register                           Address offset: 0x24 */
	__IO uint32_t RESERVED1[2];  			/*!< Reserved 0x28-0x2C                                                               */
	__IO uint32_t AHB1ENR;       			/*!< RCC AHB1 peripheral clock register                           Address offset: 0x30 */
	__IO uint32_t AHB2ENR;       			/*!< RCC AHB2 peripheral clock register                           Address offset: 0x34 */
	__IO uint32_t AHB3ENR;       			/*!< RCC AHB3 peripheral clock register                           Address offset: 0x38 */
	__IO uint32_t RESERVED2;     			/*!< Reserved 0x3C                                                                    */
	__IO uint32_t APB1ENR;       			/*!< RCC APB1 peripheral clock enable register                    Address offset: 0x40 */
	__IO uint32_t APB2ENR;       			/*!< RCC APB2 peripheral clock enable register                    Address offset: 0x44 */
	__IO uint32_t RESERVED3[2]; 			/*!< Reserved 0x48-0x4C                                                               */
	__IO uint32_t AHB1LPENR;     			/*!< RCC AHB1 peripheral clock enable in low power mode register  Address offset: 0x50 */
	__IO uint32_t AHB2LPENR;     			/*!< RCC AHB2 peripheral clock enable in low power mode register  Address offset: 0x54 */
	__IO uint32_t AHB3LPENR;     			/*!< RCC AHB3 peripheral clock enable in low power mode register  Address offset: 0x58 */
	__IO uint32_t RESERVED4;     			/*!< Reserved 0x5C                                                                    */
	__IO uint32_t APB1LPENR;     			/*!< RCC APB1 peripheral clock enable in low power mode register  Address offset: 0x60 */
	__IO uint32_t APB2LPENR;     			/*!< RCC APB2 peripheral clock enable in low power mode register  Address offset: 0x64 */
	__IO uint32_t RESERVED5[2];  			/*!< Reserved 0x68-0x6C                                                               */
	__IO uint32_t BDCR;          			/*!< RCC Backup domain control register                           Address offset: 0x70 */
	__IO uint32_t CSR;           			/*!< RCC clock control & status register                          Address offset: 0x74 */
	__IO uint32_t RESERVED6[2];  			/*!< Reserved 0x78-0x7C                                                               */
	__IO uint32_t SSCGR;         			/*!< RCC spread spectrum clock generation register                Address offset: 0x80 */
	__IO uint32_t PLLI2SCFGR;    			/*!< RCC PLLI2S configuration register                            Address offset: 0x84 */
	__IO uint32_t PLLSAICFGR;    			/*!< RCC PLLSAI configuration register                            Address offset: 0x88 */
	__IO uint32_t DCKCFGR;       			/*!< RCC Dedicated Clocks configuration register                  Address offset: 0x8C */
	__IO uint32_t CKGATENR;      			/*!< RCC Clocks Gated ENable Register                             Address offset: 0x90 */
	__IO uint32_t DCKCFGR2;      			/*!< RCC Dedicated Clocks configuration register 2                Address offset: 0x94 */
}RCC_TypeDef_t;

#define RCC 								( (RCC_TypeDef_t *) (RCC_BASE_ADDR) ) 			/* RCC Base Address */

/*
 * Bit definition for RCC_AHB1ENR register
 */

#define RCC_AHB1ENR_GPIOAEN_Pos				(0U)											/* RCC AHBIENR register GPIOAEN Bit Position */
#define RCC_AHB1ENR_GPIOAEN_Mask			(0x01 << RCC_AHB1ENR_GPIOAEN_Pos)				/* RCC AHB1ENR register GPIOAEN Bit Mask 			offset = 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN					RCC_AHB1ENR_GPIOAEN_Mask						/* RCC AHB1ENR register GPIOAEN Macro*/
#define RCC_AHB1ENR_GPIOBEN_Pos				(1U)											/* RCC AHBIENR register GPIOBEN Bit Position */
#define RCC_AHB1ENR_GPIOBEN_Mask			(0x01 << RCC_AHB1ENR_GPIOBEN_Pos)				/* RCC AHB1ENR register GPIOBEN Bit Mask 			offset = 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN					RCC_AHB1ENR_GPIOBEN_Mask						/* RCC AHBIENR register GPIOBEN Macro */
#define RCC_AHB1ENR_GPIOCEN_Pos				(2U)											/* RCC AHBIENR register GPIOCEN Bit Position */
#define RCC_AHB1ENR_GPIOCEN_Mask			(0x01 << RCC_AHB1ENR_GPIOCEN_Pos)				/* RCC AHB1ENR register GPIOCEN Bit Mask 			offset = 0x00000003 */
#define RCC_AHB1ENR_GPIOCEN					RCC_AHB1ENR_GPIOCEN_Mask						/* RCC AHBIENR register GPIOCEN Macro */
#define RCC_AHB1ENR_GPIODEN_Pos				(3U)											/* RCC AHBIENR register GPIODEN Bit Position */
#define RCC_AHB1ENR_GPIODEN_Mask			(0x01 << RCC_AHB1ENR_GPIODEN_Pos)				/* RCC AHB1ENR register GPIODEN Bit Mask 			offset = 0x00000003 */
#define RCC_AHB1ENR_GPIODEN					RCC_AHB1ENR_GPIODEN_Mask						/* RCC AHBIENR register GPIODEN Macro */









#include "RCC.h"
#include "GPIO.h"

#endif /* INC_STM32F446XX_H_ */
