/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Feb 19, 2021
 *      Author: Oliver
 */

/*
 * What is I2C?
 *
 * I2C is a protocol to achieve serial data communication between integrated circuits (ICs) which are very close to each other. It is a more serious
 * protocol than I2C because companies have come forward to design a specification for it.
 *
 * I2C protocol details (how data should be sent/received, how hand shaking should happen between sender and receiver, error handling) are more complex
 * than I2C.
 *
 * There is no dedicated spec for I2C but TI and Motorola have their own specs. I2C is based on a dedicated spec.
 *
 * I2C is multi-master capable, whereas I2C has no guidelines to achieve this, but depends on MCU designers. STM I2C peripherals can be used in multi-
 * master configurations but arbitration should be handled by software code.
 *
 * I2C hardware automatically ACKs every byte received. I2C does not support any automatic ACKing.
 *
 * I2C needs only 2 pins for communication while I2C may need 4 pins and even more if multiple slaves are involved.
 *
 * I2C master talks to slaves based on slave addresses, whereas I2C uses dedicated pins to select the slave.
 *
 * I2C is half duplex, while I2C is full duplex.
 *
 * For I2C, the max speed is 4MHz in ultra speed plus. For some STM microcontrollers, the max speed is just 400kHz. For I2C max speed, its Fpclk/2.
 * That means if the peripheral clock is 20MHz, then speed can be 10MHz.
 *
 * Slaves Control over Serial Clock:
 * In I2C, slave can make master wait by holding the clock down if its busy, thanks to clock stretching feature of I2C. But in I2C, slave has no control
 * over the clock, programmers may use their own tricks to overcome this situation.
 *
 * Data Rate:
 * Data Rate (number of bits transferred from sender to receiver in 1 sec) is much less compared to I2C.
 *
 * For example, in STM32F4x, if you have a peripheral clock of 40MHz, then with I2C you can only achieve data rate of 400Kbps compared with 20Mbps for I2C.
 *
 * SDA and SCL Signals:
 * Both SDA and SCL are bidirectional lines connected to a positive supply voltage via pullup resistors. When the bus is free, both lines are held high.
 * The output stages of devices connected to the bus must have an open-drain or open-collector configuration.
 * The bus capacitance limits the number of interfaces connected to the bus.
 *
 * I2C Modes:
 * Standard Mode		Up to 100Kbits/sec		Supported by STM32F4x
 * Fast Mode			Up to 400Kbits/sec		Supported by STM32F4x
 * Fast Mode+			Up to 1Mbits/sec		Supported by Some STM32F4x MCUs
 * High Speed Mode		Up to 3.4Mbits/sec		(not supported by F4x)
 *
 *	Start & Stop Conditions:
 *	All transactions begin with a Start(S) and are terminated by a STOP(P)
 *	A High to Low transition on the SDA line while SCL is High defines a Start condition
 *	A Low to High transition on the SDA line while SCL is High defines a Stop condition
 *
 *	The bus is considered to be busy after the START condition.
 *	The bus is considered to be free again a certain time after the STOP condition.
 *	When the bus is free another master (if present) may claim the bus.
 *	The bus stays busy if a repeated START is generated instead of a STOP condition.
 *	Most of the MCUs I2C peripherals support both master and slave mode. You need not to configure the mode because when the peripheral generates the
 *	start condition it automatically becomes the master and when it generates the stop condition it goes back to slave mode.
 *
 *	When Master gives ACK to slave, it means its ready to accept additional bytes. When Master sends NACK to slave, it indicates to slave to stop transmitting data.
 *
 *	Repeated Start Condition: (Sr) Start Again without Stop
 *	A master would do this to prevent another master from taking over the bus if it hasn't finished all communication.
 *	It is essentially a START, STOP, START without giving the chance for another master to take over in between those START conditions.
 *
 *	I2C1, I2C2, I2C3 are all on APB1 bus.
 *
 *	Required APIs:
 *	I2C Initialization
 *	I2C Master TX
 *	I2C Master RX
 *	I2C Slave TX
 *	I2C Slave RX
 *	I2C Error Interrupt Handling
 *	I2C Event Interrupt Handling
 *
 *	I2Cx Peripheral:
 *	I2C_SCLSpeed
 *	I2C_DeviceAddress
 *	I2C_ACKControl
 *	I2C_FMDutyCycle
 *
 *	Steps for I2C Init (Generic)
 *	1. Configure the Mode (standard or fast)
 *	2. Configure the speed of the serial clock (SCL)
 *	3. Configure the device address (applicable when device is slave)
 *	4. Enable the Acking
 *	5. Configure the rise time for I2C pins
 *
 *
 *	I2C Clock Stretching:
 *
 *	. Clock stretching simply means holding the clock to 0 or ground level.
 *	. The moment clock is held low, then the whole I2C interface pauses until clock is given up to its normal operation level.
 *	. I2C Devices, either master or slave, use clock stretching to slow down the communication by stretching SCL to low,
 *	  which prevents the clock to rise high again and the i2c communication stops for awhile.
 *		. There are situations where an I2C slave is not able to cooperate with the clock speed given by the master and needs to slow down a little.
 *		. If slave needs time, then it takes the advantage of clock stretching, and by holding clock at low, it pauses I2C operation.
 *
 *
 *
 *										I2C Interrupt Requests
 * 			Interrupt Event									Event Flag		Enable Control bit
 * Start bit sent (master)										SB				ITEVFEN
 * Address bit sent (master) or Address matched (slave)			ADDR			 ITEVFEN
 * 10-bit header sent (master)									ADD10			 ITEVFEN
 * Stop received (slave)										STOPF			 ITEVFEN
 * Data byte transfer finished									BTF				 ITEVFEN
 * Receive buffer not empty										RxNE			ITEVFEN and ITBUFEN
 * Transmit buffer empty										TxE				 ITEVFEN and ITBUFEN
 * Bus error													BERR			ITERREN
 * Arbitration loss (master)									ARLO			 ITERREN
 * Acknowledge failure											AF				 ITERREN
 * Overrun/Underrun												OVR				 ITERREN
 * PEC error													PECERR			 ITERREN
 * Timeout/Tlow error											TIMEOUT			 ITERREN
 * SMBusAlert													SMBALERT		 ITERREN
 *
 *		 				NVIC IRQn
 * I2Cx 	Event (I2Cx_EV_IRQ_LINE)	Error (I2Cx_ER_IRQ_LINE)
 * I2C1				31							32
 * I2C2				33							34
 * I2C3				72							73
 *
 * Bus Error:
 * 	This error happens when the interface detects an SDA rising or falling edge while SCL is high,
 * 	occurring in a non-valid position during a byte transfer.
 *
 * Arbitration Loss Error
 * 	This error can happen when the interface loses the arbitration of the bus to another master.
 *
 * ACK Failure Error
 * 	This error happens when no ACK is returned for the byte sent.
 *
 * Overrun Error
 * 	Happens during reception, when a new byte is received and the data register has not been
 * 	read yet and the new received byte is lost.
 *
 * Underrun Error
 * 	Happens when in transmission when a new byte should be sent and the data register has
 * 	not been written yet and the same byte is sent twice.
 *
 * PEC Error
 * 	Happens when there is CRC mismatch, if you have enabled the CRC feature
 *
 * Time-Out Error
 * 	Happens when master or slave stretches the clock, by holding it low more than the
 * 	recommended amount of time.
 *
 * BTF Flag in Tx and preventing underrun
 * 	During Txing of a data byte, if TXE=1, then that means data register is empty.
 * 	And if the firmware has not written any byte to data register before shift register becomes
 * 	empty (previous byte transmission), then BTF flag will be set and clock will be stretched
 * 	to prevent the underrun.
 *
 * BTF Flag in Rx and preventing overrun
 * 	If RXNE=1, then it means new data is waiting in the data register, and if the firmware
 * 	has not read the data byte yet before shift register is filled with another new data,
 * 	then also the BTF flag will be set and clock will be stretched to prevent the overrun.
 *
 * 	__Common Problems in I2C and Debugging Tips:__
 *
 * 	Problem   1: SDA and SCL line not held HIGH Voltage after I2C pin initialization
 * 	Reason    1: Not activating the pullup resistors if you are using the internal pullup resistor of an I/O line
 * 	Debug Tip 1: Worth checking the configuration register of an I/O line to see whether the pullups are really activated
 * 	           or not, best way is to dump the register contents.
 *
 * 	Problem   2: ACK failure
 * 	Reason    1: Generating the address with wrong slave address.
 * 	Debug Tip 1: Verify the slave address appearing on the SDA line by using a logic analyzer
 * 	Reason    2: Not enabling the ACKing feature in the I2C control register
 * 	Debug Tip 1: Cross check the I2C Control register ACK enable field
 *
 * 	Problem   3: Master is not producing the clock
 * 	Debug Tip 1: First check whether the I2C peripheral clock is enabled and set to at least 2MHz to produce
 * 				 standard mode I2C serial clock frequency
 *  Debug Tip 2: Check whether GPIOs which you used for SCL and SDA functionality are configured properly
 *   			 for alternate functionality.
 */

#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/******************* Private Helper Functions *******************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); 	// the LSB is R/nW bit which must be set to 0 for WRITE
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;  	// the LSB is R/nW bit which must be set to 1 for READ
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {

	uint32_t dummy_read;

	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				//disable ACKing
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		} else {
			//clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	} else {
		//device is in slave mode
		//clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle) {
	//Data transmission
	if(pI2CHandle->TxLen > 0) {
		//1. Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle) {
	//Data reception
	if(pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1) {
		if(pI2CHandle->RxLen == 2) {
			//clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		//read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0) {
		//close the I2C data reception and notify the application

		//1. Generate the stop condition
		if(pI2CHandle->SR == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close the I2C Rx
		I2C_CloseReception(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/* Peripheral Clock Setup */
/* ******************************************************************************
 * @fn					-  I2C_PClockControl
 *
 * @brief				-  This function enables or disables the peripheral clock
 * 						   for the given I2C peripheral.
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_PClockControl(I2C_RegDef_t *pI2Cx, uint8_t State) {

	//Enable peripheral clock for specified I2C peripheral 1..3
	if(State == ENABLE) {

		if(pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}

	//Disable peripheral clock for specified I2C peripheral 1..3
	} else {

		if(pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if(pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if(pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}


/* Initialization and Deinitialization */
/* ******************************************************************************
 * @fn					-  I2C_Init
 *
 * @brief				-  Initializes the I2C peripheral
 *
 * @param[in]			-  Base address of the I2C handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle) {

	uint32_t tempreg = 0;

	//enable the clock for the i2c peripheral
	I2C_PClockControl(pI2CHandle->pI2Cx, ENABLE);

	//1. ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//2. configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); //mask all bits except the first 5 bits

	//3. program the device's own address (helpful if device is slave)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM) {
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //50% duty cycle
		tempreg |= (ccr_value & 0xFFF); //mask all bits except the first 12 bits
	} else {
		//fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //DUTY = 0
		} else {
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //DUTY = 1
		}
		tempreg |= (ccr_value & 0xFFF); //mask all bits except the first 12 bits
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//5. TRISE
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM) {
		//configure the trise in std mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		//configure the trise in fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/* ******************************************************************************
 * @fn					-  I2C_DeInit
 *
 * @brief				-  De-initializes the I2C peripheral
 *
 * @param[in]			-  Base address of the I2C peripheral
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

	if(pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/* Data Send and Receive */
/* ******************************************************************************
 * @fn					-  I2C_MasterSendData
 *
 * @brief				-  Send data over I2C bus from master
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 * @param[in]			-  Pointer to Tx buffer
 * @param[in]			-  Length of data to be sent
 * @param[in]			-  Address of slave device
 * @param[in]			-  I2C_SR or I2C_NO_SR macros (Repeated Start bits)
 *
 * @return				-  None
 *
 * @note				-  Polling based (blocking)
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//	 Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddress);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared, SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(Len > 0) {
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When Len becomes zero, wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//   when BTF=1, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE)); //wait until TXE is set
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)); //wait until BTF is set

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//	 Note: generating STOP, automatically clears the BTF.
	if(SR == I2C_DISABLE_SR) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/* ******************************************************************************
 * @fn					-  I2C_MasterReceiveData
 *
 * @brief				-  Receive data over I2C bus from slave
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 * @param[in]			-  Address of slave device
 * @param[in]			-  I2C_SR or I2C_NO_SR macros (Repeated Start bits)
 *
 * @return				-  None
 *
 * @note				-  Polling based (blocking)
 *
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in SR1
	//	 Note: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddress);

	//4. Wait until address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//Procedure to read only 1 byte from slave
	if(Len == 1) {
		//Disable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear the Addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE=1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//Generate STOP condition
		if(SR == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	//Procedure to read data from slave when Len > 1
	if(Len > 1) {
		//Clear the Addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--) {

			//Wait until RXNE=1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

			if(i == 2) { //If last 2 bytes are remaining
				//Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate STOP condition
				if(SR == I2C_DISABLE_SR) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			//Read the data from data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//Increment the buffer address
			pRxBuffer++;
		}
	}

	//Re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/* ******************************************************************************
 * @fn					-  I2C_MasterSendDataIT
 *
 * @brief				-  Send data over I2C bus from master (IT)
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 * @param[in]			-  Address of slave device
 * @param[in]			-  I2C_SR or I2C_NO_SR macros (Repeated Start bits)
 *
 * @return				-  Busy state
 *
 * @note				-  Interrupt based (non-blocking)
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR) {

	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddress;
		pI2CHandle->SR = SR;

	//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

	//Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

	//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/* ******************************************************************************
 * @fn					-  I2C_MasterReceiveDataIT
 *
 * @brief				-  Receive data over I2C bus from slave (IT)
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 * @param[in]			-  Address of slave device
 * @param[in]			-  I2C_SR or I2C_NO_SR macros (Repeated Start bits)
 *
 * @return				-  Busy state
 *
 * @note				-  Interrupt based (non-blocking)
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddress, uint8_t SR) {
	//try uint8_t Len argument

	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;	//RxSize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddress;
		pI2CHandle->SR = SR;

	//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

	//Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

	//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/* ******************************************************************************
 * @fn					-  I2C_CloseTransmission
 *
 * @brief				-  End I2C transmission
 *
 * @param[in]			-  Pointer to I2C handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_CloseTransmission(I2C_Handle_t *pI2CHandle) {

	//Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/* ******************************************************************************
 * @fn					-  I2C_CloseReception
 *
 * @brief				-  End I2C reception
 *
 * @param[in]			-  Pointer to I2C handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_CloseReception(I2C_Handle_t *pI2CHandle) {

	//Disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVTEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/* ******************************************************************************
 * @fn					-  I2C_SlaveSendData
 *
 * @brief				-  Send data over I2C bus from slave
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  Data byte to be sent
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {

	pI2Cx->DR = data;
}

/* ******************************************************************************
 * @fn					-  I2C_SlaveReceiveData
 *
 * @brief				-  Receive data over I2C bus from master
 *
 * @param[in]			-  Base address of the I2C peripheral
 *
 * @return				-  Data byte received
 *
 * @note				-  None
 *
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {

	return (uint8_t)pI2Cx->DR;
}

/* IRQ Configuration, Priority and ISR Handling */
/* ******************************************************************************
 * @fn					-  I2C_IRQInterruptConfig
 *
 * @brief				-  Configure interrupt request (IRQx)
 *
 * @param[in]			-  IRQ number
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {

	if(State == ENABLE) {

		if(IRQNumber <= 31) { //IRQ 0 to 31
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64) { //IRQ 32 to 63
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96) { //IRQ 64 to 95
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		} else if(IRQNumber >= 96 && IRQNumber < 128) { //IRQ 96 to 127 (stm32f446xx only supports up to IRQ96)
			//program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));
		}

	} else {

		if(IRQNumber <= 31) { //0 to 31
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64) { //IRQ 32 to 63
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if(IRQNumber >= 64 && IRQNumber < 96) { //IRQ 64 to 95
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		} else if(IRQNumber >= 96 && IRQNumber < 128) { //IRQ 96 to 127 (stm32f446xx only supports up to IRQ96)
			//program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));
		}
	}
}

/* ******************************************************************************
 * @fn					-  I2C_IRQPriorityConfig
 *
 * @brief				-  Configure IRQ priority
 *
 * @param[in]			-  IRQ number
 * @param[in]			-  IRQ priority
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	//1. First calculate for the correct IPR register
	//IPRx registers are 32 bits wide, split into four sections of 1 Byte (8-bits): | IPRn || IRQ4n+3_PRI | IRQ4n+2_PRI | IRQ4n+1_PRI | IRQ4n_PRI |
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	//Within each 8-bit (byte) IRQ4n_PRI field, only the upper 4 bits are implemented (4:7), while the lower 4 are not (0:3).
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ******************************************************************************
 * @fn					-  I2C_EV_IRQHandling
 *
 * @brief				-  Configure I2C Event IRQ Handling
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {

	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//1. Handle for interrupt generated by SB event
	//	Note: SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3) {
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because SB is always 0
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handle for interrupt generated by ADDR event
	// 	Note: When master mode:	address is sent
	//		  When slave mode: address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3) {
		//The interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle for interrupt generated by BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3) {
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				//BTF,TXE = 1

				if(pI2CHandle->TxLen == 0) {
					//Len = 0

					//1. generate the STOP condition
					if(pI2CHandle->SR == I2C_DISABLE_SR) {
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. reset all of the member elements of the handle structure
					I2C_CloseTransmission(pI2CHandle);

					//3. notify the application about completed transmission
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			//Do nothing because I2C is Busy in Rx
			;
		}
	}

	//4. Handle for interrupt generated by STOPF event
	//	Note: Stop detection flag is applicable only in slave mode. For master,
	//		  this flag will never be set.
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3) {
		//STOPF flag is set
		//Clear the STOPF ( i.e. 1. Read from SR1 | 2. Write to CR1 )
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3) {
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//device is in master mode
			//TXE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else {
			//slave
			//make sure slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3) {
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//device is in master mode
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else {
			//slave
			//make sure the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/* ******************************************************************************
 * @fn					-  I2C_ER_IRQHandling
 *
 * @brief				-  Configure I2C Error IRQ Handling
 *
 * @param[in]			-  Pointer to base address of the I2C Handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {

	uint32_t temp1, temp2;

	//Know the status of ITERREN control bit in CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2) {
		//Bus error (BERR)
		//Clear bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//notify application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}
	/***********************Check for Arbitration Lost error************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2) {
		//Arbitration lost error (ARLO)
		//Clear ARLO flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//notify application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}
	/***********************Check for ACK Failure error******************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2) {
		//ACK failure error (AF)
		//Clear AF flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//notify application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}
	/***********************Check for Overrun/Underrun error*************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2) {
		//Overrun/Underrun error (OVR)
		//Clear OVR flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//notify application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	/***********************Check for Time Out error**********************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2) {
		//Time Out error (TIMEOUT)
		//Clear TIMEOUT flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//notify application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/* Other Peripheral Control APIs */
/* ******************************************************************************
 * @fn					-  I2C_PeripheralControl
 *
 * @brief				-  Enable or disable I2Cx peripheral
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t State) {

	if(State == ENABLE) {
 		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
 		//pI2CBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
 	} else {
 		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/* ******************************************************************************
 * @fn					-  I2C_GetFlagStatus
 *
 * @brief				-  Retrieve flag status of I2C peripheral
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  Flag name
 *
 * @return				-  Byte containing flag status info
 *
 * @note				-  None
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {

	if(pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/* ******************************************************************************
 * @fn					-  I2C_ManageAcking
 *
 * @brief				-  Enable or Disable ACKing
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  I2C_ACK_ENABLE or I2C_ACK_DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t State) {

	if(State == I2C_ACK_ENABLE) {
		//enable the ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		//disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/* ******************************************************************************
 * @fn					-  I2C_GenerateStopCondition
 *
 * @brief				-  Generate STOP condition for I2C bus
 *
 * @param[in]			-  Base address of the I2C peripheral
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/* ******************************************************************************
 * @fn					-  I2C_SlaveEnableDisableCallbackEvents
 *
 * @brief				-  Enable or Disable Callback Events
 *
 * @param[in]			-  Base address of the I2C peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t State) {

	if(State == ENABLE) {
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	} else {
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

/* Application Callback */
/* ******************************************************************************
 * @fn					-  I2C_ApplicationEventCallback
 *
 * @brief				-  Application Event Callback (weak implementation)
 *
 * @param[in]			-  Pointer to I2C handle
 * @param[in]			-  Application event byte
 *
 * @return				-  None
 *
 * @note				-  Weak implementation
 *
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv) {

	//This is a weak implementation -- the application may override this function.
}
