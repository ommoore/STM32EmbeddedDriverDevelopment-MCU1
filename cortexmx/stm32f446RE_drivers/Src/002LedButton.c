/*
 * 002LedButton.c
 *
 *  Created on: Feb 2, 2021
 *      Author: Oliver
 */


#include "stm32f446xx.h"

#define BTN_PRESSED 0

void delay(void) {

	for(uint32_t i = 0; i < 300000; i++);
}


int main(void) {

	GPIO_Handle_t GPIOLed;
	GPIO_Handle_t GPIOBtn;

	// RED1 = PB5
	GPIOLed.pGPIOx = GPIOB;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOB, ENABLE);

	GPIO_Init(&GPIOLed);

	// User Push Button = PC13
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);


	while(1) {

		if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);
		}
	}

	return 0;
}
