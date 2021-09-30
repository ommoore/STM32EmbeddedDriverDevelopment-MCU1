/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jan 24, 2021
 *      Author: Oliver
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/* Configuration Structure for a GPIO Pin */
typedef struct {
	uint8_t GPIO_PinNumber;				/* possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;				/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;		/* possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;				/* possible values from @GPIO_PIN_OPTYPE */
	uint8_t GPIO_PinAltFunMode;			/* possible values from @GPIO_PIN_AFM */
} GPIO_PinConfig_t;


/* Handle Structure for a GPIO Pin */
typedef struct {
	GPIO_RegDef_t *pGPIOx; 				/* Holds the base address of the GPIO port belonging to the pin */
	GPIO_PinConfig_t GPIO_PinConfig; 	/* Holds GPIO pin configuration settings */
} GPIO_Handle_t;


/* GPIO Pin Numbers: @GPIO_PIN_NUMBERS */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15


/* GPIO Pin Modes: @GPIO_PIN_MODES */
	//non-interrupt modes
#define GPIO_MODE_IN 		0			/* Input */
#define GPIO_MODE_OUT 		1			/* Output */
#define GPIO_MODE_ALTFN 	2			/* Alternate Function */
#define GPIO_MODE_ANALOG	3			/* Analog Input */
	//interrupt modes
#define GPIO_MODE_IN_FT		4			/* Input Falling Edge Trigger */
#define GPIO_MODE_IN_RT		5			/* Input Rising Edge Trigger */
#define GPIO_MODE_IN_RFT	6			/* Input Rising or Falling Edge Trigger */


/* GPIO Pin Output Speeds: @GPIO_PIN_SPEED */
#define GPIO_SPEED_LOW	 	0			/* Refer to the product datasheets for the values of OSPEEDRy bits versus VDD range and external load.*/
#define GPIO_SPEED_MEDIUM 	1			/* Refer to the product datasheets for the values of OSPEEDRy bits versus VDD range and external load.*/
#define GPIO_SPEED_FAST	 	2			/* Refer to the product datasheets for the values of OSPEEDRy bits versus VDD range and external load.*/
#define GPIO_SPEED_HIGH	 	3			/* Refer to the product datasheets for the values of OSPEEDRy bits versus VDD range and external load.*/


/* GPIO Pin Pull-up and Pull-down Config: @GPIO_PIN_PUPD */
#define GPIO_NO_PUPD		0			/* No Pull-up or Pull-down */
#define GPIO_PIN_PU			0			/* Pull-Up */
#define GPIO_PIN_PD			0			/* Pull-down */


/* GPIO Pin Output Types: @GPIO_PIN_OPTYPE */
#define GPIO_OP_TYPE_PP		0			/* Output Push-Pull (reset-state) */
#define GPIO_OP_TYPE_OD		1			/* Output Open-Drain */

/* GPIO Pin Alternate Function Modes: @GPIO_PIN_AFM */




/*
 * GPIO Driver API requirements should be:
 * . GPIO Initialization
 * . Enable/Disable GPIO port clock
 * . Read from a GPIO pin
 * . Write to a GPIO pin
 * . Configure alternate functionality
 * . Interrupt Handling
 */

/***********************************************************************************
 * 							APIs supported by this driver
 * 			For more info about the APIs, check the function definitions
 **********************************************************************************/

/* Peripheral Clock Setup */
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t State);

/* Initialize and Deinitialize */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data Read and Write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Configuration, Priority and ISR Handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
