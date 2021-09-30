/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jan 24, 2021
 *      Author: Oliver
 */

#include "stm32f446xx_gpio_driver.h"


/* Peripheral Clock Setup */
/* ******************************************************************************
 * @fn					-  GPIO_PClockControl
 *
 * @brief				-  This function enables or disables the peripheral clock
 * 						   for the given GPIO port.
 *
 * @param[in]			-  Base address of the GPIO peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State) {

	//Enable peripheral clock for specified GPIO Port A..H
	if(State == ENABLE) {

		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}

	//Disable peripheral clock for specified GPIO Port A..H
	} else {

		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}


/* Initialize and Deinitialize */
/* ******************************************************************************
 * @fn					-  GPIO_Init
 *
 * @brief				-  Initializes the GPIO peripheral
 *
 * @param[in]			-  Base address of the GPIO handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t temp = 0;

	//Peripheral Clock Enable
	GPIO_PClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

		//non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	} else {

		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT) {
			//1. Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RT) {
			//1. Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT) {
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;	//setting

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		//configure alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); //setting
	}
}

/* ******************************************************************************
 * @fn					-  GPIO_DeInit
 *
 * @brief				-  De-initializes the GPIO peripheral
 *
 * @param[in]			-  Base address of the GPIO peripheral
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	if(pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if(pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if(pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/* Data Read and Write */
/* ******************************************************************************
 * @fn					-  GPIO_ReadFromInputPin
 *
 * @brief				-  Read from input GPIO pin
 *
 * @param[in]			-  Base address of the GPIO peripheral
 * @param[in]			-  GPIOx pin number
 *
 * @return				-  Byte: 0 or 1
 *
 * @note				-  None
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	uint8_t value;

	//only care about the LSB, so use right shift and masking to extract the correct bit
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/* ******************************************************************************
 * @fn					-  GPIO_ReadFromInputPort
 *
 * @brief				-  Read from input GPIO port
 *
 * @param[in]			-  Base address of the GPIO peripheral
 *
 * @return				-  Word (16-bit) read from GPIOx port
 *
 * @note				-  None
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {

	uint16_t value;

	value = (uint16_t) pGPIOx->IDR;

	return value;
}

/* ******************************************************************************
 * @fn					-  GPIO_WriteToOutputPin
 *
 * @brief				-  Write to output GPIO pin
 *
 * @param[in]			-  Base address of the GPIO peripheral
 * @param[in]			-  GPIOx pin number
 * @param[in]			-  SET or RESET macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

	if(Value == GPIO_PIN_SET) {
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/* ******************************************************************************
 * @fn					-  GPIO_WriteToOutputPort
 *
 * @brief				-  Write to output GPIO port
 *
 * @param[in]			-  Base address of the GPIO peripheral
 * @param[in]			-  16-bit value
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

	pGPIOx->ODR = Value;
}

/* ******************************************************************************
 * @fn					-  GPIO_ToggleOutputPin
 *
 * @brief				-  Toggle output GPIO pin
 *
 * @param[in]			-  Base address of the GPIO peripheral
 * @param[in]			-  GPIOx pin number
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);
}


/* IRQ Configuration, Priority and ISR Handling */
/* ******************************************************************************
 * @fn					-  GPIO_IRQInterruptConfig
 *
 * @brief				-  Configure interrupt request (IRQx)
 *
 * @param[in]			-  IRQ number
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {

	if(State == ENABLE) {

		if(IRQNumber <= 31) { //IRQ 0 to 31
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) { //IRQ 32 to 63
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber < 96) { //IRQ 64 to 95
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		} else if(IRQNumber >= 96 && IRQNumber < 128) { //IRQ 96 to 127 (stm32f446xx only supports up to IRQ96)
			//program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}

	} else {

		if(IRQNumber <= 31) { //0 to 31
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber < 64) { //IRQ 32 to 63
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if(IRQNumber >= 64 && IRQNumber < 96) { //IRQ 64 to 95
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		} else if(IRQNumber >= 96 && IRQNumber < 128) { //IRQ 96 to 127 (stm32f446xx only supports up to IRQ96)
			//program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
	}
}

/* ******************************************************************************
 * @fn					-  GPIO_IRQPriorityConfig
 *
 * @brief				-  Configure IRQ priority
 *
 * @param[in]			-  IRQ number
 * @param[in]			-  IRQ priority
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	//1. First calculate for the correct IPR register
	//IPRx registers are 32 bits wide, split into four sections of 1 Byte (8-bits): | IPRn || IRQ4n+3_PRI | IRQ4n+2_PRI | IRQ4n+1_PRI | IRQ4n_PRI |
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	//Within each 8-bit (byte) IRQ4n_PRI field, only the upper 4 bits are implemented (4:7), while the lower 4 are not (0:3).
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ******************************************************************************
 * @fn					-  GPIO_IRQHandling
 *
 * @brief				-  Configure IRQ handler for GPIOx PINy
 *
 * @param[in]			-  GPIOx pin number
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber) {

	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)) {
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
