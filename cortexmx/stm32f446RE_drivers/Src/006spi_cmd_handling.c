/*
 * 004spi_tx_testing.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Oliver
 */

/*
 * Exercise:
 * SPI Master(STM) and SPI Slave(Arduino) command & response based communication.
 *
 * When the button on the master is pressed, master sends a command to the slave and slave responds
 * as per the command implementation.
 *
 * 1. Use SPI Full Duplex Mode
 * 2. ST Board will be in SPI Master Mode and Arduino will be configured for SPI slave mode
 * 3. Use DFF = 0
 * 4. Use Hardware Slave Management (SSM = 0)
 * 5. SCLK speed = 2MHz, fclk = 16MHz
 *
 *	Slave ACK (0xF5)
 *	Slave NACK (0xA5)
 *
 *	Command Formats:
 *
 *	1) CMD_LED_CTRL   	<pin no(1)>  <value(1)>
 *	2) CMD_SENSOR_READ 	<analog pin number(1)>
 *	3) CMD_LED_READ		<pin no(1)>
 *	4) CMD_PRINT		<len(2)> <message(len)>
 *	5) CMD_ID_READ
 *
 *	connect 1 LED with 470ohm resistor to pin 9 of arduino
 *
 *	Using Semihosting:
 *	1. Linker argument settings
 *	-specs=rdimon.specs -lc -lrdimon
 *
 *	2. Debug configuration of your application
 *	monitor arm semihosting enable
 *
 *	3. in main.c use below codes
 *	extern void initialise_monitor_handles();
 *	initialise_monitor_handles();
 *
 */

#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>

//semihosting
extern void initialise_monitor_handles();

//Command Codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_OFF					0
#define LED_ON					1

#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

//Arduino LED
#define LED_PIN					9



void delay(void) {

	//this will introduce ~200ms delay when the system clock is 16MHz
	for(uint32_t i = 0; i < 250000; i++);
}

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void) {

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; 			//generates sclk of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;							//hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {

	if(ackbyte == (uint8_t)0xF5) {

		//ack
		return 1;
	}

	//nack
	return 0;
}


int main(void) {

	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	//semihosting
	initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	//Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI Init done.\n");

	//Enabling SSOE will enable NSS Output. The NSS pin is automatically managed by hardware.
	//When SPE=1, NSS will be pulled to low. When SPE=0, NSS will be pulled to high.
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1) {

		//wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);
		printf("SPI Communication Opened\n");

	//1. CMD_LED_CTRL   	<pin no(1)>  <value(1)>
		uint8_t command_code = COMMAND_LED_CTRL;
		uint8_t ack_byte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &command_code, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if(SPI_VerifyResponse(ack_byte)) {

			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2, args, 2);
			//dummy read
			SPI_ReceiveData(SPI2, args, 2);

			printf("COMMAND_LED_CTRL executed\n");
		}

	//2. CMD_SENSOR_READ 	<analog pin number(1)>
		//wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues
		delay();

		//send command
		command_code = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &command_code, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if(SPI_VerifyResponse(ack_byte)) {

			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);

			printf("COMMAND_SENSOR_READ executed %d\n", analog_read);
		}

	//3. CMD_LED_READ		<pin no(1)>
		//wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues
		delay();

		//send command
		command_code = COMMAND_LED_READ;
		SPI_SendData(SPI2, &command_code, 1);

		//
		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ack_byte,1);

		if(SPI_VerifyResponse(ack_byte)) {

			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can ready with the data
			delay();

			//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);

			printf("COMMAND_READ_LED %d\n", led_status);
		}

	//4. CMD_PRINT			<len(2)> <message(len)>
		//wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues
		delay();

		//send command
		command_code = COMMAND_PRINT;
		SPI_SendData(SPI2, &command_code, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		uint8_t message[] = "Hello ! How are you ??";

		if(SPI_VerifyResponse(ack_byte)) {

			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2, args, 1); //sending length

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				SPI_SendData(SPI2,&message[i],1);
				SPI_ReceiveData(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed\n");
		}

	//5. CMD_ID_READ
		//wait until button is pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button debouncing related issues
		delay();

		//send command
		command_code = COMMAND_ID_READ;
		SPI_SendData(SPI2, &command_code, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bits (1 byte or 2 bytes depending on 8 or 16 bit comms) to fetch the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		uint8_t id[11];
		uint32_t i = 0;

		if(SPI_VerifyResponse(ack_byte)) {

			//read 10 bytes id from the slave
			for(i = 0; i < 10; i++) {

				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s\n", id);
		}

		//confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BUSY));

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}
