/*
 * 001LedToggle.c
 *
 *  Created on: Jan 29, 2021
 *      Author: Oliver
 */

/*
 * 	Exercise:
 *	Write a program to toggle the on board LED with some delay.
 *	Case 1: Use push pull configuration for the output pin
 *	Case 2: Use open drain configuration for the output pin
 */

#include "stm32f446xx.h"


void delay(void) {
	for(uint32_t i = 0; i < 500000; i++);

}


int main(void) {
	GPIO_Handle_t GPIOLed;

	// RED1 = PB5
	GPIOLed.pGPIOx = GPIOB;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOLed);

	while(1) {
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);
		delay();

	}
	return 0;
}
