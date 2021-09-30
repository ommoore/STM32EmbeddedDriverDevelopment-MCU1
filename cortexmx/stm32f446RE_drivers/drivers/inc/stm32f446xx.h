/*
 * stm32f446xx.h
 *
 *  Created on: Jan 20, 2021
 *      Author: Oliver Moore
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>


#define __vo volatile
#define __weak __attribute__((weak))

/* General Macros */
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/**************************** Processor Specific Details ****************************/

/* ARM Cortex Mx Processor NVIC ISERx Register Addresses */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

/* ARM Cortex Mx Processor NVIC ICERx Register Addresses */
#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)

/* ARM Cortex Mx Processor NVIC IPR Priority Register Address */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)

/* ARM Cortex Mx Processor Number of Priority Bits implemented in Priority Register */
#define NO_PR_BITS_IMPLEMENTED 	4


/* Base addresses of Flash and SRAM memories */
#define FLASH_BASEADDR			0x08000000U			/* 512kB */
#define SRAM1_BASEADDR			0x20000000U			/* 112kB */
#define SRAM2_BASEADDR			0x2001C000U			/* 16kB */
#define SYSMEM_BASEADDR			0x1FFF0000U			/* 30kB */
#define OTP_BASEADDR			0x1FFF7800U			/* 512B */
#define ROM						SYSMEM_BASEADDR		/* alias */
#define SRAM 					SRAM1_BASEADDR		/* alias */

/* AHBx and APBx Bus Peripheral base addresses */
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U

/* Base addresses of peripherals which are hanging on AHB1 bus */
#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/* Base addresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 bus */
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)


/*********************** Peripheral Register Definition Structures ***********************/

/* Peripheral register definition structure for GPIO */
typedef struct {
	__vo uint32_t MODER;   		/* GPIO Port Mode			  					Address offset: 0x00  	*/
	__vo uint32_t OTYPER;  		/* GPIO Port Output Type 						Address offset: 0x04	*/
	__vo uint32_t OSPEEDR;  	/* GPIO Port Output Speed 						Address offset: 0x08	*/
	__vo uint32_t PUPDR;   		/* GPIO Port Pull-Up/Pull-Down					Address offset: 0x0C	*/
	__vo uint32_t IDR;   		/* GPIO Port Input Data 						Address offset: 0x10	*/
	__vo uint32_t ODR;   		/* GPIO Port Output Data 						Address offset: 0x14	*/
	__vo uint32_t BSRR;   		/* GPIO Port Bit Set/Reset						Address offset: 0x18	*/
	__vo uint32_t LCKR;   		/* GPIO Port Configuration Lock 				Address offset: 0x1C	*/
	__vo uint32_t AFR[2];   	/* GPIO Alternate Functions
						       	   AFR[0]: Low, AFR[1]: High					Address offset: 0x20,24 */
} GPIO_RegDef_t;


/* Peripheral register definition structure for RCC */
typedef struct {
	__vo uint32_t CR;   		/* RCC Clock Control  							Address offset: 0x00  	*/
	__vo uint32_t PLLCFGR;  	/* RCC PLL Configuration						Address offset: 0x04	*/
	__vo uint32_t CFGR;  		/* RCC Clock Configuration						Address offset: 0x08	*/
	__vo uint32_t CIR;   		/* RCC Clock Interrupt							Address offset: 0x0C	*/
	__vo uint32_t AHB1RSTR; 	/* RCC AHB1 Peripheral Reset					Address offset: 0x10	*/
	__vo uint32_t AHB2RSTR; 	/* RCC AHB2 Peripheral Reset					Address offset: 0x14	*/
	__vo uint32_t AHB3RSTR; 	/* RCC AHB3 Peripheral Reset					Address offset: 0x18	*/
	uint32_t	  RESERVED0;	/* 												Reserved, 0x1C			*/
	__vo uint32_t APB1RSTR; 	/* RCC APB1 Peripheral Reset					Address offset: 0x20	*/
	__vo uint32_t APB2RSTR; 	/* RCC APB2 Peripheral Reset					Address offset: 0x24	*/
	uint32_t	  RESERVED1[2];	/* 												Reserved, 0x28,0x2C		*/
	__vo uint32_t AHB1ENR; 		/* RCC AHB1 Peripheral Clock Enable				Address offset: 0x30	*/
	__vo uint32_t AHB2ENR; 		/* RCC AHB2 Peripheral Clock Enable				Address offset: 0x34	*/
	__vo uint32_t AHB3ENR; 		/* RCC AHB3 Peripheral Clock Enable				Address offset: 0x38	*/
	uint32_t	  RESERVED2;	/* 												Reserved, 0x3C			*/
	__vo uint32_t APB1ENR; 		/* RCC APB1 Peripheral Clock Enable				Address offset: 0x40	*/
	__vo uint32_t APB2ENR; 		/* RCC APB2 Peripheral Clock Enable				Address offset: 0x44	*/
	uint32_t	  RESERVED3[2];	/* 												Reserved, 0x48,0x4C		*/
	__vo uint32_t AHB1LPENR; 	/* RCC AHB1 Peripheral Clock Enable Low Power	Address offset: 0x50	*/
	__vo uint32_t AHB2LPENR; 	/* RCC AHB2 Peripheral Clock Enable	Low Power	Address offset: 0x54	*/
	__vo uint32_t AHB3LPENR; 	/* RCC AHB3 Peripheral Clock Enable	Low Power	Address offset: 0x58	*/
	uint32_t	  RESERVED4;	/* 												Reserved, 0x5C			*/
	__vo uint32_t APB1LPENR; 	/* RCC APB1 Peripheral Clock Enable	Low Power	Address offset: 0x60	*/
	__vo uint32_t APB2LPENR; 	/* RCC APB2 Peripheral Clock Enable	Low Power	Address offset: 0x64	*/
	uint32_t	  RESERVED5[2];	/* 												Reserved, 0x68,0x6C		*/
	__vo uint32_t BDCR; 		/* RCC Backup Domain Control					Address offset: 0x70	*/
	__vo uint32_t CSR; 			/* RCC Clock Control and Status					Address offset: 0x74	*/
	uint32_t	  RESERVED6[2];	/* 												Reserved, 0x78,0x7C		*/
	__vo uint32_t SSCGR; 		/* RCC Spread Spectrum Clock Generation			Address offset: 0x80	*/
	__vo uint32_t PLLI2SCFGR; 	/* RCC PLLI2S Configuration						Address offset: 0x84	*/
	__vo uint32_t PLLSAICFGR; 	/* RCC PLL Configuration						Address offset: 0x88	*/
	__vo uint32_t DCKCFGR; 		/* RCC Dedicated Clock Configuration			Address offset: 0x8C	*/
	__vo uint32_t CKGATENR; 	/* RCC Clocks Gated Enable						Address offset: 0x90	*/
	__vo uint32_t DCKCFGR2; 	/* RCC Dedicated Clocks Configuration 2			Address offset: 0x94	*/
} RCC_RegDef_t;


/* Peripheral register definition structure for EXTI */
typedef struct {
	__vo uint32_t IMR;   		/* Interrupt Mask Register	  					Address offset: 0x00  	*/
	__vo uint32_t EMR;  		/* Event Mask Register							Address offset: 0x04	*/
	__vo uint32_t RTSR;  		/* Rising Trigger Selection Register			Address offset: 0x08	*/
	__vo uint32_t FTSR;   		/* Falling Trigger Selection Register			Address offset: 0x0C	*/
	__vo uint32_t SWIER;   		/* Software Interrupt Event Register			Address offset: 0x10	*/
	__vo uint32_t PR;   		/* Pending Register								Address offset: 0x14	*/
} EXTI_RegDef_t;


/* Peripheral register definition structure for SPI */
typedef struct {
	__vo uint32_t CR1;			/* SPI Control Register 1 (unused in I2S mode)	Address offset: 0x00	*/
	__vo uint32_t CR2;			/* SPI Control Register 2						Address offset: 0x04	*/
	__vo uint32_t SR;			/* SPI Status Register							Address offset: 0x08	*/
	__vo uint32_t DR;			/* SPI Data Register							Address offset: 0x0C	*/
	__vo uint32_t CRCPR;		/* SPI CRC Polynomial Register (unused in I2S)  Address offset: 0x10	*/
	__vo uint32_t RXCRCR;		/* SPI RX CRC Register (unused in I2S)			Address offset: 0x14	*/
	__vo uint32_t TXCRCR;		/* SPI TX CRC Register (unused in I2s) 			Address offset: 0x18	*/
	__vo uint32_t I2SCFGR;		/* SPI I2S Configuration Register				Address offset: 0x1C	*/
	__vo uint32_t I2SPR;		/* SPI I2S Prescalar Register					Address offset: 0x20	*/
} SPI_RegDef_t;


/* Peripheral register definition structure for SYSCFG */
typedef struct {
	__vo uint32_t MEMRMP;   	/* SYSCFG Memory Remap Register 				Address offset: 0x00  	*/
	__vo uint32_t PMC;  		/* SYSCFG Peripheral Mode Config Register		Address offset: 0x04	*/
	__vo uint32_t EXTICR[4];  	/* SYSCFG Ext. Interrupt Config Register 1-4    Address offset: 0x08,0x0C,0x10,0x14 */
	uint32_t RESERVED1[2];   	/* 												Reserved, 0x18,0x1C		*/
	__vo uint32_t CMPCR;   		/* Compensation Cell Control Register			Address offset: 0x20	*/
	uint32_t RESERVED2[2];   	/* 												Reserved, 0x24,0x28		*/
	__vo uint32_t CFGR;   		/* SYSCFG Configuration Register				Address offset: 0x2C	*/
} SYSCFG_RegDef_t;

/* Peripheral register definition structure for I2C */
typedef struct {
	__vo uint32_t CR1;   		/* I2C Control Register 1		 				Address offset: 0x00  	*/
	__vo uint32_t CR2;  		/* I2C Control Register 2						Address offset: 0x04	*/
	__vo uint32_t OAR1;  		/* I2C Own Address Register 1				    Address offset: 0x08	*/
	__vo uint32_t OAR2;   		/* I2C Own Address Register 2					Address offset: 0x0C	*/
	__vo uint32_t DR;   		/* I2C Data Register							Address offset: 0x10	*/
	__vo uint32_t SR1;   		/* I2C Status Register 1						Address offset: 0x14	*/
	__vo uint32_t SR2;   		/* I2C Status Register 2						Address offset: 0x18	*/
	__vo uint32_t CCR;   		/* I2C Clock Control Register					Address offset: 0x1C	*/
	__vo uint32_t TRISE;   		/* I2C TRISE Register							Address offset: 0x20	*/
	__vo uint32_t FLTR;   		/* I2C FLTR Register							Address offset: 0x24	*/
} I2C_RegDef_t;

/* Peripheral register definition structure for USART */
typedef struct {
	__vo uint32_t SR;   		/* USART Status Register	 					Address offset: 0x00  	*/
	__vo uint32_t DR;  			/* USART Data Register							Address offset: 0x04	*/
	__vo uint32_t BRR;  		/* USART Baud Rate Register				  		Address offset: 0x08	*/
	__vo uint32_t CR1;   		/* USART Control Register 1						Address offset: 0x0C	*/
	__vo uint32_t CR2;   		/* USART Control Register 2						Address offset: 0x10	*/
	__vo uint32_t CR3;   		/* USART Control Register 3						Address offset: 0x14	*/
	__vo uint32_t GTPR;   		/* USART Guard Time and Prescaler Register		Address offset: 0x18	*/
} USART_RegDef_t;

/* Peripheral definitions (Peripheral base addresses type cast to xxx_RegDef_t) */
#define GPIOA 		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1		((USART_RegDef_t*) USART1_BASEADDR)
#define USART2		((USART_RegDef_t*) USART2_BASEADDR)
#define USART3		((USART_RegDef_t*) USART3_BASEADDR)
#define UART4		((USART_RegDef_t*) UART4_BASEADDR)
#define UART5		((USART_RegDef_t*) UART5_BASEADDR)
#define USART6		((USART_RegDef_t*) USART6_BASEADDR)

/********************* PERIPHERAL CLOCK ENABLE MACROS *********************/

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 <<  7))

/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 <<  4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 <<  5))

/* Clock Enable Macro for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/**** PERIPHERAL CLOCK DISABLE MACROS ****/

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 <<  7))

/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 <<  4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 <<  5))

/* Clock Disable Macro for SYSCFG peripheral */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


/* Macros to reset GPIOx Peripherals */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : \
								  (x == GPIOB) ? 1 : \
								  (x == GPIOC) ? 2 : \
								  (x == GPIOD) ? 3 : \
								  (x == GPIOE) ? 4 : \
								  (x == GPIOF) ? 5 : \
								  (x == GPIOG) ? 6 : \
								  (x == GPIOG) ? 7 : 0)

/* Macros to reset SPIx Peripherals */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12));  (RCC->APB2RSTR &= ~(1 << 0)); } while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 13));  (RCC->APB2RSTR &= ~(1 << 0)); } while(0)

/* Macros to reset I2Cx Peripherals */
#define I2C1_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 21));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define I2C2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 22));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define I2C3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 23));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)

/* Macros to reset USARTx Peripherals */
#define USART1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 <<  4));  (RCC->APB2RSTR &= ~(1 << 0)); } while(0)
#define USART2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 17));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define USART3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 18));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define UART4_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 19));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define UART5_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 20));  (RCC->APB1RSTR &= ~(1 << 0)); } while(0)
#define USART6_REG_RESET()	do{ (RCC->APB2RSTR |= (1 <<  5));  (RCC->APB2RSTR &= ~(1 << 0)); } while(0)


/* IRQ (Interrupt Request) Numbers of STM32F446xx MCU */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2			  	8
#define IRQ_NO_EXTI3			  	9
#define IRQ_NO_EXTI4			  	10
#define IRQ_NO_EXTI9_5			  	23
#define IRQ_NO_EXTI15_10		  	40

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI3					51
#define IRQ_NO_SPI4					84

#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73

#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_USART6				71


/* NVIC IRQ Priority Level Macros */
#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15
#define NVIC_IRQ_PRI16				16
#define NVIC_IRQ_PRI17				17
#define NVIC_IRQ_PRI18				18
#define NVIC_IRQ_PRI19				19
#define NVIC_IRQ_PRI20				20
#define NVIC_IRQ_PRI21				21
#define NVIC_IRQ_PRI22				22
#define NVIC_IRQ_PRI23				23
#define NVIC_IRQ_PRI24				24
#define NVIC_IRQ_PRI25				25
#define NVIC_IRQ_PRI26				26
#define NVIC_IRQ_PRI27				27
#define NVIC_IRQ_PRI28				28
#define NVIC_IRQ_PRI29				29
#define NVIC_IRQ_PRI30				30
#define NVIC_IRQ_PRI31				31
#define NVIC_IRQ_PRI32				32
#define NVIC_IRQ_PRI33				33
#define NVIC_IRQ_PRI34				34
#define NVIC_IRQ_PRI35				35
#define NVIC_IRQ_PRI36				36
#define NVIC_IRQ_PRI37				37
#define NVIC_IRQ_PRI38				38
#define NVIC_IRQ_PRI39				39
#define NVIC_IRQ_PRI40				40
#define NVIC_IRQ_PRI41				41
#define NVIC_IRQ_PRI42				42
#define NVIC_IRQ_PRI43				43
#define NVIC_IRQ_PRI44				44
#define NVIC_IRQ_PRI45				45
#define NVIC_IRQ_PRI46				46
#define NVIC_IRQ_PRI47				47
#define NVIC_IRQ_PRI48				48
#define NVIC_IRQ_PRI49				49
#define NVIC_IRQ_PRI50				50
#define NVIC_IRQ_PRI51				51
#define NVIC_IRQ_PRI52				52
#define NVIC_IRQ_PRI53				53
#define NVIC_IRQ_PRI54				54
#define NVIC_IRQ_PRI55				55
#define NVIC_IRQ_PRI56				56
#define NVIC_IRQ_PRI57				57
#define NVIC_IRQ_PRI58				58
#define NVIC_IRQ_PRI59				59
#define NVIC_IRQ_PRI60				60
#define NVIC_IRQ_PRI61				61
#define NVIC_IRQ_PRI62				62
#define NVIC_IRQ_PRI63				63
#define NVIC_IRQ_PRI64				64
#define NVIC_IRQ_PRI65				65
#define NVIC_IRQ_PRI66				66
#define NVIC_IRQ_PRI67				67
#define NVIC_IRQ_PRI68				68
#define NVIC_IRQ_PRI69				69
#define NVIC_IRQ_PRI70				70
#define NVIC_IRQ_PRI71				71
#define NVIC_IRQ_PRI72				72
#define NVIC_IRQ_PRI73				73
#define NVIC_IRQ_PRI74				74
#define NVIC_IRQ_PRI75				75
#define NVIC_IRQ_PRI76				76
#define NVIC_IRQ_PRI77				77
#define NVIC_IRQ_PRI78				78
#define NVIC_IRQ_PRI79				79
#define NVIC_IRQ_PRI80				80
#define NVIC_IRQ_PRI81				81
#define NVIC_IRQ_PRI82				82
#define NVIC_IRQ_PRI83				83
#define NVIC_IRQ_PRI84				84
#define NVIC_IRQ_PRI85				85
#define NVIC_IRQ_PRI86				86
#define NVIC_IRQ_PRI87				87
#define NVIC_IRQ_PRI88				88
#define NVIC_IRQ_PRI89				89
#define NVIC_IRQ_PRI90				90
#define NVIC_IRQ_PRI91				91
#define NVIC_IRQ_PRI92				92
#define NVIC_IRQ_PRI93				93
#define NVIC_IRQ_PRI94				94
#define NVIC_IRQ_PRI95				95
#define NVIC_IRQ_PRI96				96
#define NVIC_IRQ_PRI97				97
#define NVIC_IRQ_PRI98				98
#define NVIC_IRQ_PRI99				99
#define NVIC_IRQ_PRI100				100
#define NVIC_IRQ_PRI101				101
#define NVIC_IRQ_PRI102				102
#define NVIC_IRQ_PRI103				103


/********************* Bit Position Definitions of SPI Peripheral *********************/
/* Bit Position Definitions of SPI_CR1 */
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

/* Bit Position Definitions of SPI_CR2 */
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

/* Bit Position Definitions of SPI_SR */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/********************* Bit Position Definitions of I2C Peripheral *********************/
/* Bit Position Definitions of I2C_CR1 */
#define I2C_CR1_PE					0
#define I2C_CR1_SMBUS				1
#define I2C_CR1_SMBTYPE				3
#define I2C_CR1_ENARP				4
#define I2C_CR1_ENPEC				5
#define I2C_CR1_ENGC				6
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_POS					11
#define I2C_CR1_PEC					12
#define I2C_CR1_ALERT				13
#define I2C_CR1_SWRST				15

/* Bit Position Definitions of I2C_CR2 */
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10
#define I2C_CR2_DMAEN				11
#define I2C_CR2_LAST				12

/* Bit Position Definitions of I2C_OAR1 */
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

/* Bit Position Definitions of I2C_OAR2 */
#define I2C_OAR2_ENDUAL				0
#define I2C_OAR2_ADD71				1

/* Bit Position Definitions of I2C_DR */
#define I2C_DR_70					0

/* Bit Position Definitions of I2C_SR1 */
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_PECERR				12
#define I2C_SR1_TIMEOUT				14
#define I2C_SR1_SMBALERT			15

/* Bit Position Definitions of I2C_SR2 */
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_SMBDEFAULT			5
#define I2C_SR2_SMBHOST				6
#define I2C_SR2_DUALF				7
#define I2C_SR2_PEC70				8

/* Bit Position Definitions of I2C_CCR */
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

/* Bit Position Definitions of I2C_TRISE */
#define I2C_TRISE_TRISE5_0			0

/* Bit Position Definitions of I2C_FLTR */
#define I2C_FLTR_DNF3_0				0
#define I2C_ANOFF					4

/********************* Bit Position Definitions of USART Peripheral *********************/
/* Bit Position Definitions of USART_SR */
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9

/* Bit position definitions USART_CR1 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/* Bit position definitions USART_CR2 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN   				11
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/* Bit position definitions USART_CR3 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"

#endif /* INC_STM32F446XX_H_ */
