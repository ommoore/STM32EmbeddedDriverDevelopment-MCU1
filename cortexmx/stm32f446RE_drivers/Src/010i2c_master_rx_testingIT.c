/*
 * 010i2c_master_rx_testingIT.c
 *
 *  Created on: Mar 11, 2021
 *      Author: Oliver
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

extern void initialise_monitor_handles();

I2C_Handle_t I2C1Handle;

//Flag variable
uint8_t rxComplete = RESET;

#define MY_ADDR 	0x61
#define SLAVE_ADDR 	0x68 //changing this will trigger ACK failure (AF) error

//Rcv buffer
uint8_t rcv_buf[32];

void delay(void) {

	//this will introduce ~200ms delay when the system clock is 16MHz
	for(uint32_t i = 0; i < 250000; i++);
}

/*
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 *
 * ALT function mode : 4
 */

void I2C1_GPIOInits(void) {

	GPIO_Handle_t I2CPins;

	/* Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void) {

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_Speed_SM;

	I2C_Init(&I2C1Handle);
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

	uint8_t commandcode, len;

	initialise_monitor_handles();
	printf("Application is running\n");

	//Push Button Init
	GPIO_ButtonInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
	//no need to implement priority function

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ACK bit is set to 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1) {
		//wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

		rxComplete = RESET;

		//wait until RX completes
		while(rxComplete != SET);

		rcv_buf[len + 1] = '\0';
		printf("Data : %s", rcv_buf);

		rxComplete = RESET;
	}

	return 0;
}

void I2C1_EV_IRQHandler(void) {

	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {

	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	if(AppEv == I2C_EV_TX_CMPLT) {
		printf("Tx is completed\n");

	} else if(AppEv == I2C_EV_RX_CMPLT) {
		printf("Rx is completed\n");
		rxComplete = SET;

	} else if(AppEv == I2C_EV_STOP) {
		printf("STOP condition initiated\n");

	} else if(AppEv == I2C_ERROR_BERR) {
		printf("Error : Bus Error\n");

	} else if(AppEv == I2C_ERROR_ARLO) {
		printf("Error : Bus Arbitration Lost\n");

	} else if(AppEv == I2C_ERROR_AF) {
		printf("Error : ACK failure\n");

		//in master, ACK failure happens when slave fails to send ACK for the byte sent by master.
		I2C_CloseTransmission(pI2CHandle);

		//generate the STOP condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		//hang in infinite loop
		while(1);

	} else if(AppEv == I2C_ERROR_OVR) {
		printf("Error : Overrun/Underrun\n");

	} else if(AppEv == I2C_ERROR_TIMEOUT) {
		printf("Error : TIMEOUT\n");

	} else if(AppEv == I2C_EV_DATA_REQ) {
		printf("Data Requested\n");

	} else if(AppEv == I2C_EV_DATA_RCV) {
		printf("Data Received\n");

	}
}
