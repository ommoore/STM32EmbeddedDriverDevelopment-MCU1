/*
 * 012i2c_slave_tx_string2.c
 *
 *  Created on: Mar 11, 2021
 *      Author: Oliver
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"

I2C_Handle_t I2C1Handle;

#define SLAVE_ADDR 		0x68 //changing this will trigger ACK failure (AF) error
#define MY_ADDR			SLAVE_ADDR

uint32_t data_len = 0;

//tx buffer
uint8_t tx_buf[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123";
//rcv buffer
uint8_t rcv_buf[32];

uint8_t commandCode;

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

	//printf("Application is running\n");
	data_len = strlen((char*)tx_buf);

	//Push Button Init
	GPIO_ButtonInit();

	//I2C pin inits
	I2C1_GPIOInits();

	//I2C peripheral configuration
	I2C1_Inits();

	//Enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ACK bit is set to 1 after PE=1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);
	//no need to implement priority function

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	while(1);
	return 0;
}

void I2C1_EV_IRQHandler(void) {

	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void) {

	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	static uint8_t cnt = 0;
	static uint8_t w_ptr = 0;

	if(AppEv == I2C_EV_TX_CMPLT) {

	} else if(AppEv == I2C_EV_RX_CMPLT) {

	} else if(AppEv == I2C_EV_STOP) {
		//This will happen during end slave reception
		//slave concludes end of Rx
		cnt = 0;

	} else if(AppEv == I2C_ERROR_BERR) {

	} else if(AppEv == I2C_ERROR_ARLO) {

	} else if(AppEv == I2C_ERROR_AF) {
		//This will occur during slave transmission to master .
		//slave should understand master needs no more data
		//slave concludes end of Tx


		//if the current active code is 0x52 then don't invalidate
		if(!(commandCode == 0x52)) {
			commandCode = 0xFF;
		}

		//reset the cnt variable because transmission is completed
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= (data_len))
		{
			w_ptr = 0;
			commandCode = 0xFF;
		}

	} else if(AppEv == I2C_ERROR_OVR) {

	} else if(AppEv == I2C_ERROR_TIMEOUT) {

	} else if(AppEv == I2C_EV_DATA_REQ) {
		//Master is requesting data. Send data.
		if(commandCode == 0x51) {
			//sending 4 bytes of length information
			I2C_SlaveSendData(I2C1, ((data_len >> ((cnt % 4) * 8)) & 0xFF));
			cnt++;
		} else if (commandCode == 0x52) {
			//sending tx_buf contents indexed by w_ptr variable
			I2C_SlaveSendData(I2C1, tx_buf[w_ptr++]);
		}

	} else if(AppEv == I2C_EV_DATA_RCV) {
		//Master has sent command code, read it
		commandCode = I2C_SlaveReceiveData(I2C1);

	}
}
