/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 *    GPIO Functional Summary:
 *
 *  . Input Floating
 *  . Input Pull-up
 *  . Input Pull-down
 *  . Analog
 *  . Output open-drain with pull-up or pull-down
 *  . Output push-pull with pull-up or pull-down
 *  . Alternate function push-pull with pull-up or pull-down
 *  . Alternate function open-drain with pull-up or pull-down
 *
 *	A GPIO Pin's 16 possible alternate functionalities:
 *	AF0 (system)
 *	AF1 (TIM1/TIM2)
 *	AF2 (TIM3..5)
 *	AF3 (TIM8..11, CEC)
 *	AF4 (I2C1..4, CEC)
 *	AF5 (SPI1/2/3/4)
 *	AF6 (SPI2/3/4, SAI1)
 *	AF7 (SPI2/3, USART1..3, UART5, SPDIF-IN)
 *	AF8 (SPI2/3, USART1..3, UART5, SPDIF-IN)
 *	AF9 (CAN1/2, TIM12..14, QUADSPI)
 *	AF10 (SAI2, QUADSPI, OTG_HS, OTG_FS)
 *	AF11
 *	AF12 (FMC, SDIO, OTG_HS)
 *	AF13 (DCMI)
 *	AF14
 *	AF15 (EVENTOUT)
 *
 *	Exercise: List out all the 16 possible alternate functionalities supported byb GPIO port 'A' pin 8 (GPIOA.8)
 *
 *	PA8:   (From Datasheet: Alternate Function Mapping)
 *	AF0		MCO1
 *	AF1		TIM1_CH1
 *	AF2		-
 *	AF3		-
 *	AF4		I2C3_SCL
 *	AF5		-
 *	AF6		-
 *	AF7		USART1_CK
 *	AF8		-
 *	AF9		-
 *	AF10	OTG_FS_SOF
 *	AF11	-
 *	AF12	-
 *	AF13	-
 *	AF14	-
 *	AF15	EVENTOUT
 *
 * 	MCU Specific Device Header File:
 * 	. Base Addresses of various memories present in the MCU such as Flash, SRAM1, SRAM2, ROM, etc
 * 	. The base addresses of various bus domains such as AHBx domain, APBx domain
 * 	. Base addresses of various peripherals present in different bus domains of the MCU
 * 	. Clock management macros (ie. cloak enable and disable macros)
 * 	. IRQ definitions
 * 	. Peripheral Register definition structures
 * 	. Peripheral register bit definitions
 * 	. Other useful microcontroller configuration macros
 *
 *
 *			 --- (Device Header) ---
 *	   		v						v
 *	   (Application)		   (Driver Files)
 *
 *
 *
 *	Different bus domains of the stm32f4x MCU:
 *
 *								   PERIPH_BASE
 *								        v
 *		   -------------------------------------------------------------
 *	       v                 v                     v                   v
 *	APB1PERIPH_BASE   APRB2PERIPH_BASE 		AHB1PERIPH_BASE 	AHB2PERIPH_BASE
 *
 *	. Different peripherals are hanging on different busses.
 *	. AHB bus is used for those peripherals which need high speed data communication (ex. Camera interfaces, GPIOs)
 *  . APB bus is used for those peripherals for which low speed communication would suffice.
 *
 *
 *	AHB1 Peripherals:
 *	GPIOA GPIOB GPIOC GPIOD GPIOE GPIOF GPIOG GPIOH
 *
 *	APB1 Peripherals:
 *	I2C1 I2C2 I2C3 SPI2 SPI3 USART2 USART3 UART4 UART5
 *
 *	APB2 Peripherals:
 *	SPI1 USART1 USART6 EXTI SYSCFG
 *
 *
 *

 */

#include <stdint.h>

int main(void)
{

	for(;;);
}