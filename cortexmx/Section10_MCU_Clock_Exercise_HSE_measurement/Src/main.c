/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
 *  Exercise HSE Measurement (Nucleo):
 *
 *	1. Enable the HSEBYP bit (RCC_CR)
 *	(bypass the oscillator with an external clock)
 *	2. Enable the HSE clock using the HSEON bit (RCC_CR)
 *	3. Switch the systemclock to HSE
 *	4. Do MCO1 settings to measure it
 */


#include <stdint.h>

#define RCC_BASE_ADDR		0x40023800UL
#define RCC_CFGR_REG_OFFSET	0x08UL
#define RCC_CR_REG_OFFSET	0x00UL
#define RCC_CFGR_REG_ADDR	(RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)
#define RCC_CR_REG_ADDR		(RCC_BASE_ADDR + RCC_CR_REG_OFFSET)

#define GPIOA_BASE_ADDR		0x40020000UL

int main(void)
{
	 uint32_t *pRccCrReg = (uint32_t*) RCC_CR_REG_ADDR;
	 uint32_t *pRccCfgrReg = (uint32_t*) RCC_CFGR_REG_ADDR;

	 //1. Enable the HSEBYP bit (RCC_CR) (bypass the oscillator with an external clock)
	 pRccCrReg |= (1 << 18);

	 //2. Enable the HSE clock using the HSEON bit (RCC_CR)
	 pRccCrReg |= (1 << 16);

	 //3. Switch the systemclock to HSE
	 *pRccCfgrReg |= (1 << 0);

	 /************** Do MCO1 settings to measure it ****************/

	//1. Configure the RCC_CFGR MCO1 bit fields to select HSE as clock source
	*pRccCfgrReg |= (1 << 22);

	//Configure MCO1 prescaler to divide by 4. Bits 26:24 we want to write 110.
	*pRccCfgrReg |= (1 << 26);
	*pRccCfgrReg |= (1 << 25);


	//2. Configure PA8 to AF0 mode to behave as MCO1 signal
	//a. Enable the peripheral clock for GPIOA peripheral
	uint32_t *pRccAhb1Enr = (uint32_t*)(RCC_BASE_ADDR + 0x30);
	*pRccAhb1Enr |= (1 << 0); //enable peripheral clock

	//b. Configure the mode of GPIOA pin 8 as alternate function mode
	uint32_t *pGPIOAModeReg = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
	*pGPIOAModeReg &= ~(0x3 << 16); //clear  0x3 = 0b11
	*pGPIOAModeReg |= (0x2 << 16); //set 0x2 = 0b10 -> 10 means alternate function. bits 16:17 is for pin8

	//c. Configure the alternate function register to set PA8 to mode 0
	uint32_t *pGPIOAAltFunHighReg = (uint32_t*)(GPIOA_BASE_ADDR + 0x24); //AFRH is for pins 8-15, AFRL is for pins 0-7
	*pGPIOAAltFunHighReg &= ~(0xF << 0); //clear 0xF = 0b1111  -> 0000: AF0

	for(;;);
}