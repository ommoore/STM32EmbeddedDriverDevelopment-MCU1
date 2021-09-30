/*
 * 014uart_case.c
 *
 *  Created on: Mar 20, 2021
 *      Author: Oliver
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

USART_Handle_t USART1Handle;

//reply from arduino will be stored here
char rx_buf[1024];

//This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;

extern void initialise_monitor_handles();

void delay(void) {

	//this will introduce ~200ms delay when the system clock is 16MHz
	for(uint32_t i = 0; i < 250000; i++);
}

void USART1_GPIOInit(void) {

	GPIO_Handle_t USARTPins;

	/* Note : Internal pull-up resistors are used */

	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//Using USART2 peripheral requires solder bridge changes to Nucleo-F446RE board
	//to allow PA2 and PA3 to be used instead of Tx/Rx jumper (CN3) on the ST-link debug board

	//USART1 peripheral does not require any physical changes

	//USART1 TX (PA9)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USARTPins);

	//USART1 RX (PA10)
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USARTPins);
}

void USART1_Init(void) {

	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&USART1Handle);
}

void GPIO_ButtonInit() {

	// User Push Button = PC13
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void) {


	uint32_t cnt = 0;

	initialise_monitor_handles();

	//Push Button Init
	GPIO_ButtonInit();

	//USART pin inits
	USART1_GPIOInit();

	//USART peripheral configuration
	USART1_Init();

	//Enable USART Interrupts
	USART_IRQInterruptConfig(IRQ_NO_USART1, ENABLE);

	//Enable the I2C peripheral
	USART_PeripheralControl(USART1, ENABLE);

    printf("Application is running\n");

	while(1) {
		//wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//Next message index -- make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First enable the reception in interrupt mode
		while(USART_ReceiveDataIT(&USART1Handle, rx_buf, strlen(msg[cnt])) != USART_READY);

		//Send the message indexed by cnt in blocking (polling) mode
		USART_SendData(&USART1Handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

		printf("Transmitted : %s\n", msg[cnt]);

		//Wait until all bytes are received from the Arduino
		//When all bytes are received, rxCmplt will be SET in application callback
		while(rxCmplt != SET);

		//Ensure that last byte should be null otherwise %s fails while printing
		rx_buf[strlen(msg[cnt]) + 1] = '\0';

    	//Print received message from Arduino
    	printf("Received    : %s\n", rx_buf);

    	//Clear the flag
    	rxCmplt = RESET;

    	//move on to the next message indexed in msg[]
    	cnt++;
	}

	return 0;
}

void USART1_IRQHandler(void)
{
	USART_IRQHandling(&USART1Handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
   if(AppEv == USART_EVENT_RX_CMPLT) {
	   rxCmplt = SET;

   } else if(AppEv == USART_EVENT_TX_CMPLT) {
	   ;
   }
}
