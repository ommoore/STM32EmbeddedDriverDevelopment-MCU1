/*
 * 004spi_tx_testing.c
 *
 *  Created on: Feb 9, 2021
 *      Author: Oliver
 */

/*
 * Exercise:
 * Test the SPI_SendData API to send the string "Hello world" and use the below configurations:
 * SPI-2 Master Mode
 * SCLK = max possible
 * DFF = 0 and DFF = 1
 *
 *	Options (some not all):
 *	SPI2_SCK - PA9 AF5, PB13 AF5
 *	SPI2_NSS - PB4 AF7, PB9 AF5, PB12 AF5
 *	SPI2_MISO - PB14 AF5
 *	SPI2_MOSI - PB15 AF5
 *
 *	Chosen:
 *	PB12 AF5 - SPI2_NSS
 *	PB13 AF5 - SPI2_SCK
 *	PB14 AF5 - SPI2_MISO
 *	PB15 AF5 - SPI2_MOSI
 */

#include "stm32f446xx.h"
#include <string.h>

void SPI2_GPIOInits(void) {

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; 			//generates sclk of 8MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;							//software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);


}

int main(void) {

	char user_data[] = "Hello world";

	//Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//Force NSS signal high (internally) to avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_FLAG_BUSY));

	//disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
