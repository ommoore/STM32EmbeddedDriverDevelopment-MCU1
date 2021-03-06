/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Feb 5, 2021
 *      Author: Oliver
 */

/*	Notes:
 *
 * 				Serial Peripheral Interface
 *
 * 	The SPI bus allows the communication between one master device and one or more slave devices:
 *
 * 				SCLK -----------------> SCLK
 * 	Master		MOSI -----------------> MOSI		Slave
 * 				MISO <----------------- MISO	(Sensor, EEPROM, SDCARD,
 * 				gpio1-----------------> šš		 Display, Bluetooth, etc)
 *
 *	Serial Clock, Master Out Slave In, Master In Slave Out, Slave Select.
 *
 *	1. Four I/O pins are dedicated to SPI communication with external devices.
 *	2. MISO: Master In / Slave Out data. In the general case, this pin is used to transmit data
 *	   in slave mode and receive data in master mode.
 *	3. MOSI: Master Out / Slave In data. In general case, this pin is used to transmit data
 *	   in master mode, and receive data in slave mode.
 *	4. SCK: Serial Clock output pin for SPI master and input pin for SPI slaves.
 *	5. NSS: Slave select pin. Depending on the SPI and NSS settings, this pin can be
 *	   used to select an individual slave device for communication.
 *
 *
 *	SPI advantages:  higher speed than I2C, simpler
 *		disadvantages: low distance (10ft), more pins used.
 *
 *	RS-485 (4000ft max distance)
 *
 *	SPI/I2C communicate over TTL signaling in range of 0-5V.
 *	CAN/RS-485 communicate over differential signaling (>10V).
 *	This is a reason why SPI and I2C are short distance communication interfaces.
 *
 *
 *	Protocol	Type					Max Distance (ft)				Max Speed (bps)			Typical Usage
 *	USB3.0		Dual Simplex Serial		9ft (up to 49 with 5 hubs)   	5G						Mass storage, video
 *	USB2.0		Half Duplex Serial		16ft (98ft with 5 hubs)			1.5M,12M,480M			Keyboard, Mouse, Drive, speakers, printer, camera
 *	Ethernet	Serial					1600ft							10G						Network Communications
 *	I2C			Synchronous Serial		18ft							3.4M (high-speed mode)	Microcontroller communications
 *	RS-232		Asynchronous Serial		50-100ft						20k						Modem, Mouse, Instrumentation
 *	RS-485		Asynchronous Serial		4000ft							10M						Data Acquisition, control systems SPI
 *	SPI			Synchronous Serial		10ft							fPCLK/2					Sensors, EEPROM, Flash, Display
 *
 *
 *	To use slave select, Master must use a GPIO pin and pull 'ss' to ground for slave to activate MOSI,MISO pins and take them out of High-Z state.
 *
 *	Minimal SPI Bus configuration:
 *	The SPI bus allows the communication between one master device and one or more slave devices. In some applications SPI bus may consist of just two wires -
 *	one for the clock signal and the other for synchronous data transfer. Other signals can be added depending on the data exchange between SPI nodes and their
 *	slave select signal management.
 *
 *	SPI Hardware - Behind the Scenes:
 *	SPI protocol uses Shift Registers on both Master and Slave side for communication.
 *	MOSI connects the LSB of Master's shift register with MSB of Slave's shift register.
 *	MISO connects the LSB of Slave's shift register with the MSB of Master's shift register.
 *	Every clock cycle, both shift registers will right-shift by 1 bit. This means the  LSB of Master will be pushed to MSB of Slave, and
 *	LSB of Slave will be pushed to MSB of Master simultaneously.
 *
 *						(After 1 Clock Cycle)
 *	  ->[B0,A7,A6,A5,A4,A3,A2,A1] -MOSI-> [A0,B7,B6,B5,B4,B3,B2,B1] -
 *	 |			Master			   					Slave			 |
 *    -----------------------------MISO------------------------------
 *
 *
 *   Customizing SPI Bus: Bus Configurations
 *   The SPI allows the MCU to communicate using different configurations, depending on the device targeted and the app requirements.
 *
 *   Full-Duplex Communication:
 *   In this config, the shift registers of the master and slave are linked using two unidirectional lines between the MOSI and the MISO pins.
 *   During SPI communication, data is shifted synchronously on the SCK clock edges provided by the master. The master transmits the data to be sent
 *   to the slave via the MOSI line and receives data from the slave via the MISO line. By default, the SPI is configured for Full-Duplex Communication.
 *   (Up to 16-bit shift registers on both sides).
 *
 *   Half-Duplex Communication:
 *   In this configuration, one single cross connection line is used to link the shift registers of the master and slave together. During this communication,
 *   the data is synchronously shifted between the shift registers on the SCK clock edge in the transfer direction selected reciprocally by both master and slave.
 *   MOSI of master has to be connected to MISO of slave here. NSS optional if only one slave. Master would be set up as transmit mode, slave as receiver mode, or vice versa.
 *   (Up to 8-bit shift registers on both sides)
 *
 *	 Simplex Communication:
 *	 Simplex single master, single slave application (master in transmit-only/slave in receive-only mode)
 *	 Transmit-only, receive-only mode:
 *	 The configuration settings are the same as for full-duplex. The application has to ignore the information captured on the unused input pin.
 *	 This pin can be used as a standard GPIO.
 *
 *		Slave Select (NSS) Pin Management
 *	When a device is in slave mode:
 *	In slave mode, the NSS works as a standard 'chip select' input and lets the slave communicate with the master.
 *
 *	When a device is a master:
 *	In master mode, NSS can be used either as output or input. As an input, it can prevent multi-master bus collision, and as an output it can
 *	drive a slave select signal of a single slave.
 *
 *	2 Types of Slave Management:
 *		Hardware or software slave management can be set using the SSM bit in the SPIx_CR1 register.
 *
 *	Hardware Slave Management
 *		Hardware NSS management (SSM=0): NSS pin must be pulled low to activate slave to communicate with master. NSS pin must be in output mode.
 *		The NSS pin is managed by the hardware.
 *
 *	Software Slave Management
 *		Software NSS management (SSM=1): in this configuration, slave select information is driven internally by the SSI bit value in register
 *		SPIx_CR1. The external NSS pin is free for other application uses.
 *
 *
 *
 *	Points to Remember:
 *	Scenario 1: 1 Master 1 Slave
 *		1) You dont need to use NSS pin of master and slave if you use software slave management.
 *		2) If you dont want to use software slave management, then you can connect NSS of master to NSS of slave.
 *
 *	Scenario2: 1 master and multiple slaves
 *		1) You cannot use software slave management here.
 *		2) You cannot use NSS pin of the master to connect NSS pin of any of the slaves.
 *		3) Master has to use some of its GPIO pins to control the different NSS pins of the slaves.
 *
 *
 *
 *	SPI Communication Format
 *	SCLK Phase (CPHA), SCLK Polarity (CPOL), Data Frame Format (DFF)
 *
 *	. During SPI communication, receive and transmit operations are performed simultaneously.
 *	. The serial clock (SCK) synchronizes the shifting and sampling of the information on the data lines.
 *	. The communication format depends on the clock phase, the clock polarity and the data frame format.
 *	  To be able to communicate together, the master and slaves devices must follow the same communication format.
 *
 *	CPOL(Clock Polarity)
 *	> The CPOL (clock polarity) bit controls the idle state value of the clock when no data is being transferred.
 *	> If CPOL is reset, the SCLK pin has a low-level idle state. If CPOL is set, the SCLK pin has a high-level idle state.
 *
 *	CPHA(Clock Phase)
 *	> CPHA controls at which clock edge of the SCLK(1st or 2nd) the data should be sampled by the slave.
 *	> The combination of CPOL (clock polarity) and CPHA (clock phase) bits selects the data capture clock edge.
 *
 *	4 Different SPI modes:
 *	SPI mode 0: CPOL = 0, CPHA = 0
 *	SPI mode 1: CPOL = 0, CPHA = 1
 *	SPI mode 2: CPOL = 1, CPHA = 0
 *	SPI mode 3: CPOL = 1, CPHA = 1
 *
 *	If CPHA = 1, Data will be sampled on the Trailing Edge of the clock
 *	If CPHA = 0, Data will be sampled on the Leading Edge of the clock
 *
 *	Slave samples on MOSI and Master samples on MISO
 *
 *	What is the maximum SCLK speed of SPIx peripheral which can be achieved on a given microcontroller?
 *	> First you have to know the speed of the APBx bus on which the SPI peripheral is connected.
 *
 *	By default, the clock signal is sourced from HSI signal (16MHZ) (other choices are HSE or PLL signals) and the APB1 bus prescalar is set to 1. The APB1 bus can support
 *	a maximum clock speed of 42MHz. Then we have: f_pclk = 16MHz -> SPI2/SPI3 (on APB1 bus) -> min SPI prescalar = 2 -> SCLK = 8MHZ
 *
 *	Note: SPI1 peripheral is hanging on APB2 bus (84MHz max speed -- 2x APB1 bus!), while SPI2/3 are on APB1 bus.
 *
 *	So, if we use the internal RC oscillator of 16MHz as our system clock, then SPI1/SPI2/SPI3 peripherals can produce the serial clock of 8MHz maximum.
 *
 *	SPI Driver API Requirements:
 *		. SPI initialization / peripheral clock control
 *		. SPI Tx
 *		. SPI Rx
 *		. SPI Interrupt Config & Handling
 *		. Other SPI Management APIs
 *
 *	SPIx Peripheral - Configurable Items for User Application:
 *	SPI_DeviceMode, SPI_BusConfig, SPI_DFF, SPI_CPHA, SPI_CPOL, SPI_SSM, SPI_Speed
 */

#include "stm32f446xx_spi_driver.h"

//static keyword used to make helper functions private
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/******************* Private Helper Functions *******************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

		//16bit DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;

	} else {

		//8bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen) {

		//If TxLen is zero, close the SPI transmission and inform application that Tx is finished
		//this prevents interrupts from setting the TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

		//16bit DFF
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	} else {

		//8bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen) {

		//Rx is complete, turn off the RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	//1. clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/* Peripheral Clock Setup */
/* ******************************************************************************
 * @fn					-  SPI_PClockControl
 *
 * @brief				-  This function enables or disables the peripheral clock
 * 						   for the given SPI peripheral.
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t State) {

	//Enable peripheral clock for specified SPI peripheral 1..4
	if(State == ENABLE) {

		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}

	//Disable peripheral clock for specified SPI peripheral 1..4
	} else {

		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/* Initialization and Deinitialization */
/* ******************************************************************************
 * @fn					-  SPI_Init
 *
 * @brief				-  Initializes the SPI peripheral
 *
 * @param[in]			-  Base address of the SPI handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {

	//Peripheral Clock Enable
	SPI_PClockControl(pSPIHandle->pSPIx, ENABLE);

	//1.a. Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	//1.b. Configure the device mode as Master (slave is default)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure the SPI Serial Clock Speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF (Data Frame Format)
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL (Clock Polarity)
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA (Clock Phase)
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/* ******************************************************************************
 * @fn					-  SPI_DeInit
 *
 * @brief				-  De-initializes the SPI peripheral
 *
 * @param[in]			-  Base address of the SPI peripheral
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	if(pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if(pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/* Data Send and Receive */
/* ******************************************************************************
 * @fn					-  SPI_SendData
 *
 * @brief				-  Send data over SPI bus
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  Pointer to Tx buffer
 * @param[in]			-  Length of data to be sent
 *
 * @return				-  None
 *
 * @note				-  This is a blocking call (polling)
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

	while(Len > 0) {

		//1. Wait (polling) until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_FLAG_TXE) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

			//16bit DFF
			//1. Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		} else {

			//8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/* ******************************************************************************
 * @fn					-  SPI_ReceiveData
 *
 * @brief				-  Receive data over SPI bus
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be received
 *
 * @return				-  None
 *
 * @note				-  This is a blocking call (polling)
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {

	while(Len > 0) {

		//1. Wait (polling) until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_FLAG_RXNE) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {

			//16bit DFF
			//1. Load the data from DR into Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;

		} else {

			//8bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/* ******************************************************************************
 * @fn					-  SPI_SendDataIT
 *
 * @brief				-  Receive data over SPI bus using interrupts
 *
 * @param[in]			-  Pointer to SPI handle
 * @param[in]			-  Pointer to Tx buffer
 * @param[in]			-  Length of data to be received
 *
 * @return				-  Status of Tx
 *
 * @note				-  This is a non-blocking call (interrupt)
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t status = pSPIHandle->TxState;

	if(status != SPI_BUSY_IN_TX) {

		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so no other code can take over
		//   the same peripheral until transmission is over.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return status;
}

/* ******************************************************************************
 * @fn					-  SPI_ReceiveDataIT
 *
 * @brief				-  Receive data over SPI bus using interrupts
 *
 * @param[in]			-  Pointer to SPI handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be received
 *
 * @return				-  None
 *
 * @note				-  This is a non-blocking call (interrupt)
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {

	uint8_t status = pSPIHandle->RxState;

	if(status != SPI_BUSY_IN_RX) {

		//1. Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in retrieval so no other code can take over
		//   the same peripheral until retrieval is over.
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return status;
}


/* IRQ Configuration, Priority and ISR Handling */
/* ******************************************************************************
 * @fn					-  SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {

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
 * @fn					-  SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	//1. First calculate for the correct IPR register
	//IPRx registers are 32 bits wide, split into four sections of 1 Byte (8-bits): | IPRn || IRQ4n+3_PRI | IRQ4n+2_PRI | IRQ4n+1_PRI | IRQ4n_PRI |
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	//Within each 8-bit (byte) IRQ4n_PRI field, only the upper 4 bits are implemented (4:7), while the lower 4 are not (0:3).
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}


/* ******************************************************************************
 * @fn					-  SPI_IRQHandling
 *
 * @brief				-  Configure IRQ handler for SPIx
 *
 * @param[in]			-  Pointer to SPI handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle) {

/*
 *	SPI Interrupts:
 *	During SPI communication, an interrupt can be generated by the following events:
 *	. Transmit Tx buffer ready to be loaded						(TXE Event Flag)  	(TXEIE  Enable Control Bit)
 *	. Data received in Rx buffer								(RXNE Event Flag) 	(RXNEIE Enable Control Bit)
 *	. Master mode fault (must avoid when using single-master)	(MODF Event Flag) 	(ERRIE  Enable Control Bit)
 *	. Overrun error												(OVR Event Flag)  	(ERRIE  Enable Control Bit)
 *	. CRC error													(CRCERR Event Flag)	(ERRIE  Enable Control Bit)
 *	. TI frame format error										(FRE Event Flag)  	(ERRIE  Enable Control Bit)
 *
 *	Interrupts can be enabled and disabled separately.
 *
 *	SPI connects directly to NVIC without EXTI line or other intermediate step.
 *
 *	Exercise 1:
 *	Complete SPI IRQ number definition macros in MCU specific header file
 *
 *	Exercise 2:
 *	Complete the SPI IRQ Configuration APIs Implementation (reuse code from GPIO driver)
 */
	uint8_t temp1, temp2;

	//check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2) {
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2) {
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

/* Other Peripheral Control APIs */
/* ******************************************************************************
 * @fn					-  SPI_PeripheralControl
 *
 * @brief				-  Enable or disable SPIx peripheral
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t State) {

	if(State == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/* ******************************************************************************
 * @fn					-  SPI_SSIConfig
 *
 * @brief				-  SSI (Internal Slave Select)
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  Forces a value onto NSS pin and the IO value of the
 * 						   NSS pin is ignored.
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t State) {

	if(State == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/* ******************************************************************************
 * @fn					-  SPI_SSOEConfig
 *
 * @brief				-  SSOE (Slave Select Output Enable)
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  0: SS output is disabled in master mode and cell can work
 * 					          in multi-master configuration.
 * 					       1: SS output is enabled in master mode and when cell is
 * 					          enabled. Cell cannot work in multi-master environment.
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t State) {

	if(State == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/* ******************************************************************************
 * @fn					-  SPI_GetFlagStatus
 *
 * @brief				-  Retrieve flag status of SPI peripheral
 *
 * @param[in]			-  Base address of the SPI peripheral
 * @param[in]			-  Flag name
 *
 * @return				-  Byte containing flag status info
 *
 * @note				-  None
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName) {

	if(pSPIx->SR & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/* ******************************************************************************
 * @fn					-  SPI_ClearOVRFlag
 *
 * @brief				-  Clear Overrun Flag
 *
 * @param[in]			-  Base address of the SPI peripheral
 *
 * @return				-  None
 *
 * @note				-  Pg864 of reference manual
 *
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/* ******************************************************************************
 * @fn					-  SPI_CloseTransmission
 *
 * @brief				-  End SPI transmission
 *
 * @param[in]			-  Pointer to SPI handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {

	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/* ******************************************************************************
 * @fn					-  SPI_CloseReception
 *
 * @brief				-  End SPI reception
 *
 * @param[in]			-  Pointer to SPI handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/* Application Callback */
/* ******************************************************************************
 * @fn					-  SPI_ApplicationEventCallback
 *
 * @brief				-  Application Event Callback (weak implementation)
 *
 * @param[in]			-  Pointer to SPI handle
 * @param[in]			-  Application event byte
 *
 * @return				-  None
 *
 * @note				-  Weak implementation
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {

	//This is a weak implementation -- the application may override this function.
}
