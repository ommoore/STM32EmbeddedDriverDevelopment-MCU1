/*
 * 004spi_tx_testing.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Oliver
 */

/*
 * Exercise:
 * SPI Master(STM) and SPI Slave(Arduino) communication
 * When the button on the master is pressed, master should send string of data to the Arduino slave connected. The data received by
 * the Arduino will be displayed on the Arduino serial port.
 *
 * 1. Use SPI Full duplex mode
 * 2. ST board will be in SPI master mode and Arduino will be configured for SPI slave mode
 * 3. Use DFF = 0
 * 4. Use Hardware slave management (SSM = 0)
 * 5. SCLK speed = 2MHz, fclk = 16MHz
 *
 * In this exercise, master is not going to receive anything for the slave. So you may not configure the MISO pin.
 *
 * Note: slave does not know how many bytes of data master is going to send. So master first sends the number of bytes info which slave
 * is going to receive next.
 *
 *
 *	Things you need:
 *	Arduino Board
 *	STM32 Board
 *	Logic Level converter
 *	Breadboard and jumper wires
 *
 *	STM32 Vdd = 3V 		Arduino Vcc = 5V
 *
 *	Power Arduino board and download SPI Slave Sketch to Arduino
 *	Sketch name: 001SPISlaveRxString.ino
 *
 *
 *	STM MCU feature:
 *	with SSM = 0, and SPE = 1, NSS output will be forced to 0 (provided SSOE bit = 1 as well)
 *	with SSM = 0, and SPE = 0, NSS output will be forced to 1 (provided SSOE bit = 1 as well)
 */

#include "stm32f446xx.h"
#include <string.h>

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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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

int main(void) {

	char user_data[] = "Hello world";

	GPIO_ButtonInit();

	//Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI2 peripheral parameters
	SPI2_Inits();

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

		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BUSY));

		//disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
