/*
 * stm32f446xx.h
 *
 *  Created on: 28 Nis 2023
 *      Author: efebasol
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <string.h>

/*
 * Microprocessor Defines
 */

#define NVIC_ISER0							( (uint32_t*)(0xE000E100) )

#define __IO volatile
#define SET_BIT(REG, BIT)					( (REG) |= (BIT) )
#define CLEAR_BIT(REG, BIT)					( (REG) &= ~(BIT) )
#define READ_BIT(REG, BIT)					( (REG) & (BIT) )
#define UNUSED(x)							(void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE
}FunctionalState_t;

/*
 * IRQ Numbers of MCU == Vector Table
 */

typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	EXTI4_IRQNumber = 10,
	EXTI9_5_IRQNumber = 23,
	EXTI15_10_IRQNumber = 40
}IRQNumber_TypeDef_t;

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

#define TIM2_BASE_ADDR						(APB1_BASE_ADDR + 0x0000UL)
#define TIM3_BASE_ADDR          		    (APB1_BASE_ADDR + 0x0400UL)
#define TIM4_BASE_ADDR 			            (APB1_BASE_ADDR + 0x0800UL)
#define TIM5_BASE_ADDR 			            (APB1_BASE_ADDR + 0x0C00UL)
#define TIM6_BASE_ADDR 			            (APB1_BASE_ADDR + 0x1000UL)
#define TIM7_BASE_ADDR  		            (APB1_BASE_ADDR + 0x1400UL)
#define TIM12_BASE_ADDR        		        (APB1_BASE_ADDR + 0x1800UL)
#define TIM13_BASE_ADDR    			        (APB1_BASE_ADDR + 0x1C00UL)
#define TIM14_BASE_ADDR     		        (APB1_BASE_ADDR + 0x2000UL)
#define RTC_BASE_ADDR      			        (APB1_BASE_ADDR + 0x2800UL)
#define WWDG_BASE_ADDR             			(APB1_BASE_ADDR + 0x2C00UL)
#define IWDG_BASE_ADDR             			(APB1_BASE_ADDR + 0x3000UL)
#define SPI2_BASE_ADDR             			(APB1_BASE_ADDR + 0x3800UL)
#define SPI3_BASE_ADDR             			(APB1_BASE_ADDR + 0x3C00UL)
#define SPDIFRX_BASE_ADDR          			(APB1_BASE_ADDR + 0x4000UL)
#define USART2_BASE_ADDR          			(APB1_BASE_ADDR + 0x4400UL)
#define USART3_BASE_ADDR           			(APB1_BASE_ADDR + 0x4800UL)
#define UART4_BASE_ADDR            			(APB1_BASE_ADDR + 0x4C00UL)
#define UART5_BASE_ADDR            			(APB1_BASE_ADDR + 0x5000UL)
#define I2C1_BASE_ADDR             			(APB1_BASE_ADDR + 0x5400UL)
#define I2C2_BASE_ADDR             			(APB1_BASE_ADDR + 0x5800UL)
#define I2C3_BASE_ADDR             			(APB1_BASE_ADDR + 0x5C00UL)
#define FMPI2C1_BASE_ADDR          			(APB1_BASE_ADDR + 0x6000UL)
#define CAN1_BASE_ADDR             			(APB1_BASE_ADDR + 0x6400UL)
#define CAN2_BASE_ADDR             			(APB1_BASE_ADDR + 0x6800UL)
#define CEC_BASE_ADDR              			(APB1_BASE_ADDR + 0x6C00UL)
#define PWR_BASE_ADDR              			(APB1_BASE_ADDR + 0x7000UL)
#define DAC_BASE_ADDR              			(APB1_BASE_ADDR + 0x7400UL)

/*
 * APB2 Peripheral Base Address
 */

#define TIM1_BASE_ADDR 						(APB2_BASE_ADDR + 0x0000UL)		/* TIM1 Base address	*/
#define TIM8_BASE_ADDR 						(APB2_BASE_ADDR + 0x0400UL)		/* TIM8 Base address	*/
#define USART1_BASE_ADDR 					(APB2_BASE_ADDR + 0x1000UL)		/* USART1 Base address	*/
#define USART6_BASE_ADDR 					(APB2_BASE_ADDR + 0x1400UL)		/* USART6 Base address	*/
#define ADC1_BASE_ADDR            			(APB2_BASE_ADDR + 0x2000UL)
#define ADC2_BASE_ADDR     					(APB2_BASE_ADDR + 0x2100UL)
#define ADC3_BASE_ADDR             			(APB2_BASE_ADDR + 0x2200UL)
#define ADC123_COMMON_BASE_ADDR    			(APB2_BASE_ADDR + 0x2300UL)
#define ADC_BASE_ADDR          				(ADC123_COMMON_BASE)
#define SDIO_BASE_ADDR         				(APB2_BASE_ADDR + 0x2C00UL)
#define SPI1_BASE_ADDR 						(APB2_BASE_ADDR + 0x3400UL)		/* SPI1 Base address	*/
#define SPI4_BASE_ADDR 						(APB2_BASE_ADDR + 0x3400UL)		/* SPI4 Base address	*/
#define SYSCFG_BASE_ADDR 					(APB2_BASE_ADDR + 0x3800UL)		/* Interrupt Register SYSCFG Base address	*/
#define EXTI_BASE_ADDR 						(APB2_BASE_ADDR + 0x3C00UL)		/* Interrupt Register EXTI Base address	*/
#define TIM9_BASE_ADDR						(APB2_BASE_ADDR + 0x4000UL)
#define TIM10_BASE_ADDR						(APB2_BASE_ADDR + 0x4400UL)
#define TIM11_BASE_ADDR						(APB2_BASE_ADDR + 0x4800UL)
#define SAI1_BASE_ADDR            			(APB2_BASE_ADDR + 0x5800UL)
#define SAI1_Block_A_BASE_ADDR     			(SAI1_BASE_ADDR + 0x004UL)
#define SAI1_Block_B_BASE_ADDR     			(SAI1_BASE_ADDR + 0x024UL)
#define SAI2_BASE_ADDR            			(APB2_BASE_ADDR + 0x5C00UL)
#define SAI2_Block_A_BASE_ADDR     			(SAI2_BASE_ADDR + 0x004UL)
#define SAI2_Block_B_BASE_ADDR			    (SAI2_BASE_ADDR + 0x024UL)

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
#define CRC_BASE_ADDR              			(AHB1_BASE_ADDR + 0x3000UL)
#define RCC_BASE_ADDR              			(AHB1_BASE_ADDR + 0x3800UL)
#define FLASH_R_BASE_ADDR    			    (AHB1_BASE_ADDR + 0x3C00UL)
#define DMA1_BASE_ADDR             			(AHB1_BASE_ADDR + 0x6000UL)
#define DMA1_Stream0_BASE_ADDR     			(DMA1_BASE_ADDR + 0x010UL)
#define DMA1_Stream1_BASE_ADDR     			(DMA1_BASE_ADDR + 0x028UL)
#define DMA1_Stream2_BASE_ADDR     			(DMA1_BASE_ADDR + 0x040UL)
#define DMA1_Stream3_BASE_ADDR     			(DMA1_BASE_ADDR + 0x058UL)
#define DMA1_Stream4_BASE_ADDR     			(DMA1_BASE_ADDR + 0x070UL)
#define DMA1_Stream5_BASE_ADDR     			(DMA1_BASE_ADDR + 0x088UL)
#define DMA1_Stream6_BASE_ADDR     			(DMA1_BASE_ADDR + 0x0A0UL)
#define DMA1_Stream7_BASE_ADDR     			(DMA1_BASE_ADDR + 0x0B8UL)
#define DMA2_BASE_ADDR             			(AHB1_BASE_ADDR + 0x6400UL)
#define DMA2_Stream0_BASE_ADDR     			(DMA1_BASE_ADDR + 0x010UL)
#define DMA2_Stream1_BASE_ADDR     			(DMA1_BASE_ADDR + 0x028UL)
#define DMA2_Stream2_BASE_ADDR     			(DMA1_BASE_ADDR + 0x040UL)
#define DMA2_Stream3_BASE_ADDR     			(DMA1_BASE_ADDR + 0x058UL)
#define DMA2_Stream4_BASE_ADDR    			(DMA1_BASE_ADDR + 0x070UL)
#define DMA2_Stream5_BASE_ADDR    			(DMA1_BASE_ADDR + 0x088UL)
#define DMA2_Stream6_BASE_ADDR     			(DMA1_BASE_ADDR + 0x0A0UL)
#define DMA2_Stream7_BASE_ADDR     			(DMA1_BASE_ADDR + 0x0B8UL)

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

/*
 * Bit definition for RCC_APB2ENR register
 */

#define RCC_APB2ENR_SYSCFG_POS				(14UL)											/* RCC APB2 register SYSCFGEN Bit Position */
#define RCC_APB2ENR_SYSCFG_Mask				(0x1UL << RCC_APB2ENR_SYSCFG_POS)				/* RCC APB2 register SYSCFGEN Bit Mask */
#define RCC_APB2ENR_SYSCFGEN				RCC_APB2ENR_SYSCFG_Mask							/* RCC APB2 register SYSCFGEN Macro */
#define RCC_APB2ENR_SPI1EN_Pos				(12UL)											/* RCC APB2 register SPI1EN Bit Position */
#define RCC_APB2ENR_SPI1EN_Mask				(0x1U << RCC_APB2ENR_SPI1EN_Pos)				/* RCC APB2 register SPI1EN Bit Mask */
#define RCC_APB2ENR_SPI1EN					RCC_APB2ENR_SPI1EN_Mask							/* RCC APB2 register SPI1EN Macro */
#define RCC_APB2ENR_SPI4EN_Pos				(13UL)											/* RCC APB2 register SPI4EN Bİt Position */
#define RCC_APB2ENR_SPI4EN_Mask				(0x1U << RCC_APB2ENR_SPI4EN_Pos)				/* RCC APB2 register SPI4EN Bit Mask */
#define RCC_APB2ENR_SPI4EN					RCC_APB2ENR_SPI4EN_Mask							/* RCC APB2 register SPI4EN Macro */

/*
 * Bit definition for RCC_APB1ENR register
 */

#define RCC_APB1_SPI2_Pos					(14UL)											/* RCC APB2 register SPI2EN Bit Position */
#define RCC_APB1_SPI2_Mask					(0x1U << RCC_APB1_SPI2_Pos)						/* RCC APB2 register SPI2EN Bit Mask */
#define RCC_APB1_SPI2EN						RCC_APB1_SPI2_Mask								/* RCC APB2 register SPI2EN Macro */
#define RCC_APB1_SPI3_Pos					(15UL)											/* RCC APB2 register SPI3EN Bit Position */
#define RCC_APB1_SPI3_Mask					(0x1U << RCC_APB1_SPI3_Pos)						/* RCC APB2 register SPI3EN Bit Mask */
#define RCC_APB1_SPI3EN						RCC_APB1_SPI3_Mask								/* RCC APB2 register SPI2EN Macro */

/*
 *	SYSCFG Register Structure Definitions
 */

typedef struct
{
	__IO uint32_t MEMRMP;					/*!< SYSCFG memory remap register 						Address offset: 0x00			*/
	__IO uint32_t PMC;						/*!< SYSCFG peripheral mode configuration register		Address offset: 0x04			*/
	__IO uint32_t EXTICR[4];				/*!< SYSCFG external interrupt configuration register 	Address offset: 0x08-0C-10-14	*/
	__IO uint32_t CMPCR;					/*!< Compensation cell control register					Address offset: 0x20 			*/
	__IO uint32_t CFGR;						/*!< SYSCFG configuration register						Address offset: 0x2C			*/
}SYSCFG_TypeDef_t;

#define SYSCFG								( (SYSCFG_TypeDef_t *)(SYSCFG_BASE_ADDR) )

/*
 *	EXTI Register Structure Definitions
 */

typedef struct
{
	__IO uint32_t IMR;						/*!< Interrupt mask register 							Address offset: 0x00 */
	__IO uint32_t EMR;						/*!< Event mask register								Address offset: 0x04 */
	__IO uint32_t RTSR;						/*!< Rising trigger selection register					Address offset: 0x08 */
	__IO uint32_t FTSR;						/*!< Falling trigger selection register					Address offset: 0x0C */
	__IO uint32_t SWIER;					/*!< Software interrupt event register					Address offset: 0x10 */
	__IO uint32_t PR;						/*!< Pending register									Address offset: 0x14 */
}EXTI_TypeDef_t;

#define EXTI								( (EXTI_TypeDef_t *)(EXTI_BASE_ADDR) )

/*
 * SPI Register Structure Definitions
 */

typedef struct
{
	__IO uint32_t CR1;						/*!< SPI control register			!!not used in I2S mode		Address offset: 0x00 */
	__IO uint32_t CR2;						/*!< SPI control register										Address offset: 0x04 */
	__IO uint32_t SR;						/*!< SPI status register 										Address offset: 0x08 */
	__IO uint32_t DR;						/*!< SPI data register 											Address offset: 0x0C */
	__IO uint32_t CRCPR;					/*!< SPI CRC polynomial register	!!not used in I2S mode		Address offset: 0x10 */
	__IO uint32_t RXCRCR;					/*!< SPI RX CRC register			!!not used in I2S mode		Address offset: 0x14 */
	__IO uint32_t TXCRCR;					/*!< SPI TX CRC register			!!not used in I2S mode		Address offset: 0x18 */
	__IO uint32_t I2SCFGR;					/*!< SPI_I2S configuration register    							Address offset: 0x1C */
	__IO uint32_t I2SPR;					/*!< SPI_I2S prescaler register									Address offset: 0x20 */
}SPI_TypeDef_t;

#define SPI1								( (SPI_TypeDef_t)(SPI1_BASE_ADDR) )
#define SPI2								( (SPI_TypeDef_t)(SPI2_BASE_ADDR) )
#define SPI3								( (SPI_TypeDef_t)(SPI3_BASE_ADDR) )
#define SPI4								( (SPI_TypeDef_t)(SPI4_BASE_ADDR) )

#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"

#endif /* INC_STM32F446XX_H_ */
