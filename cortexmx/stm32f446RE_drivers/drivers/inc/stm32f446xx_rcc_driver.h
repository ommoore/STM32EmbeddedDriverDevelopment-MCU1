/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Mar 10, 2021
 *      Author: Oliver
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
