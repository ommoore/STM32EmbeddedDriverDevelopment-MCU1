/*
 * 002LedButton.c
 *
 *  Created on: Feb 2, 2021
 *      Author: Oliver
 */

/*
 * GPIO Pin Interrupt Configuration
 *
 * 1. Pin must be in input configuration
 * 2. Configure the edge trigger (RT,FT,RFT)
 * 3. Enable interrupt delivery from peripheral to the processor (on peripheral side)
 * 4. Identify the IRQ number on which the processor accepts the interrupt from that pin
 * 5. Configure the IRQ priority for the identified IRQ number (processor side)
 * 6. Enable interrupt reception on that IRQ number (processor side)
 * 7. Implement IRQ handler
 *
 *
 * 			STM32F4x GPIO Pins interrupt delivery to the Processor
 * 			MCU Peripheral Side							Processor Side
 * GPIO port is decided by
 * SYSCFG_EXTICR register config
 *
 *	  (GPIO)				    (EXTI)		          (NVIC)
 * 	GPIOx_PIN0		-->			EXTI0		-->			6
 * 	GPIOx_PIN1		-->			EXTI1		-->			7
 * 	GPIOx_PIN2		-->			EXTI2		-->			8
 * 	GPIOx_PIN3		-->			EXTI3		-->			9			<-->  Processor Core
 * 	GPIOx_PIN4		-->			EXTI4		-->			10
 * 	GPIOx_PIN5_9	-->			EXTI5_9		-->			23
 * 	GPIOx_PIN10_15	-->			EXTI10_15	-->			40
 * 					EXTI block does Edge Detection		Enable/Disable of IRQ's are
 * 					(FT,RT), Enable/Disable of			configured in NVIC registers
 * 					interrupt delivery to the
 * 					processor.
 *
 *	1. Implement the ISR function
 *	2. Store the addr. of your ISR at the vector addr. location corresponding to the IRQ number for which you have written the ISR`
 *
 *	Exercise:
 *	Configure user push button (PC13) and toggle the LED (PB5) whenever interrupt is triggered by the button press.
 *	Interrupt should be triggered during the falling edge of the button press.
 *
 *
 */

#include <string.h>
#include "stm32f446xx.h"

#define BTN_PRESSED 0

void delay(void) {

	//this will introduce ~200ms delay when the system clock is 16MHz
	for(uint32_t i = 0; i < 250000; i++);
}


int main(void) {

	GPIO_Handle_t GPIOLed, GPIOBtn;

	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	// RED_LED_1 = PB5
	GPIOLed.pGPIOx = GPIOB;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOLed);

	// User Push Button = PC13
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, GPIO_PIN_RESET);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void) {

	delay(); //200ms wait until button de-bouncing  is over
	GPIO_IRQHandling(GPIO_PIN_NO_13);	//clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);
}
