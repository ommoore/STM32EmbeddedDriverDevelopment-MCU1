/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Mar 12, 2021
 *      Author: Oliver
 */

/*
 *	UART (Universal Asynchronous Receiver Transmitter)
 *	and
 *	USART (Universal Synchronous/Asynchronous Receiver Transmitter)
 *	are pieces of hardware that converts parallel data into serial data.
 *
 *	You can use USART module in both synchronous and asynchronous modes.
 *
 *	There is no specific port for USART communication. They are commonly used in conjunction
 *	with protocols like RS-232, RS-434, USB etc.
 *
 *	In Synchronous transmission, the clock is sent separately from the data stream and
 *	no start/stop bits are used.
 *
 *	USART Hardware Components:
 *	Baud rate generator, TX and RX shift registers, Transmit/Receive control blocks and buffers,
 *	First-in First-out (FIFO) buffer memory
 *
 *	USART is just a piece of hardware in the MCU which transmits/receives data bits either in
 *	Synchronous or Asynchronous mode.
 *
 *	If it is Asynchronous mode, then the clock will not be sent alongside the data, instead we use
 *	synchronization bits like start and stop with the useful data.
 *
 *	__Understanding UART Pins__
 *	UART1:  TX, RX, RTS, CTS
 *
 *	The USART RX engine continuously samples the RX line to detect the start bit of a frame.
 *	When the start bit of the frame is detected, the frame reception kicks in on the RX line.
 *
 *	When transmission is not occurring, the TX line will be held high in its idle state.
 *
 *	RTS (Request to Send and CTS) (Clear to Send - Active Low pin)
 *
 *	When hardware flow control is used, the UART module will send the data out of TX line
 *	only when the CTS is pulled to low.
 *
 *	UART module uses the RTS line to request data from the other device.
 *
 *	__UART Frame Formats__
 *	9-bit word, 1 stop bit
 *	<Start Bit> <Data Frame Bit 0:8> <Stop Bit>  (Bit 8 can also be used as an optional Parity Bit)
 *	When parity is used, the MSB of the data will be replaced by parity bit
 *
 *	8-bit word, 1 stop bit
 *	<Start Bit> <Data Frame Bit 0:7> <Stop Bit>
 *
 *	__Baud Rate__
 *	Baud rate is how fast the data is sent over the serial line in units of bits-per-second (bps).
 *	If you invert the baud rate, you can find out how long it takes to transmit a single bit. This value
 *	determines how long the transmitter holds a serial line high or low.
 *
 *	Baud rate = 9600bps
 *	1 bit duration = 1/9600 = 104us
 *
 *	Both transmitting and receiving devices should operate at the same rate.
 *	Common values: 2400, 4800, 9600, 19200, 38400, 57600, 115200
 *
 *	The baud rates are usually dependent upon the peripheral clock frequency of the UART peripheral.
 *
 *	__Synchronization Bits__
 *	Start bit marks the beginning of a frame (idle data line going from high to low)
 *	Stop bits mark the the end of a frame (back to idle state low to high)
 *	There is always 1 start bit, but stop bits are configurable (0.5, 1, 1.5, 2 stop bits)
 *
 *	__UART Parity__
 *	Adding a Parity bit is a simple method of error detection. Parity is simply the number of one's appearing
 *	in the binary form of a number.
 *	55(decimal) -> 0b00110111  Parity=5 (odd number of one's -> set parity bit to 1)   Parity is either even or odd.
 *	Even = 0, Odd = 1
 *
 *	__UART Peripheral Clock__
 *	USART1 and USART6 are on APB2 (84MHz max but 16MHz if run from HSI or HSE).
 *	USART2, USART3, UART4, UART5 are on APB1 (42MHz max but 16MHz if run from HSI or HSE).
 *
 *	__Steps for Data Transmission__
 *	. Program the M bit in USART_CR1 to define the word length
 *	. Program the number of stop bits in USART2_CR2 register.
 *	. Select the desired baud rate using the USART_BRR register (check table to see baud rate choices given different peripheral clock values).
 *	. Set the TE bit in USART_CR1 to enable the transmit block.
 *	. Enable the USART by writing the UE bit in USART_CR1.
 *	. Now if TXE flag is set, then write the data byte to send, in the USART_DR register.
 *	. After writing the last data into the USART_DR register, wait until TC=1
 *
 *	__Steps for Data Reception__
 *	. Program the M bit in USART_CR1 to define the word length.
 *	. Program the number of stop bits in USART_CR2 register.
 *	. Select the desired baud rate using the USART_BRR register.
 *	. Enable the USART by writing the UE bit in USART_CR1.
 *	. Set the RE bit in the USART_CR1 register, which enables the receiver block of the USART peripheral.
 *	. When a character is received, wait until the RXNE bit is set and read the data byte from the data register.
 *	. The RXNE bit must be cleared by reading the data register, before the end of the reception of the next character to avoid an overrun error.
 *
 *	__USART: Oversampling__
 *	. The receiver implements different user-configurable over-sampling techniques (except in synchronous mode) for data recovery by
 *	  discriminating between valid incoming data and noise.
 *	. The over-sampling method can be selected by programming the OVER8 bit in the USART_CR1 register and can be either 16 or 8 times the baud rate clock.
 *	. Configurable oversampling method by 16 or by 8 to give flexibility between speed and clock tolerance.
 *
 *	__Sampled Values vs Noise__
 *	Sampled Value		NE Status		Received bit value
 *		000					0					0
 *		001					1					0
 *		010					1					0
 *		011					1					1
 *		100					1					0
 *		101					1					1
 *		110					1					1
 *		111					0					1
 *
 *	__Noise Error__
 *	When noise is detected in a frame:
 *	. The NF flag bit is set by hardware when noise is detected on a received frame.
 *	. The invalid data is transferred from the Shift register to the USART_DR register.
 *	. The application may consider or discard the frame based on application logic.
 *
 *	__Selecting the proper over-sampling method__
 *	The receiver implements different user-configurable over-sampling techniques (except in synchronous mode) for
 *	data recovery by discriminating between valid incoming data and noise.
 *	. Over-sampling by 8 (OVER8 = 1) -> max baud rate = F_pclk / 8
 *		- max receiver tolerance to clock deviation reduced
 *	. Over-sampling by 16 (OVER8 = 0) -> max baud rate = F_pclk / 16
 *		- max receiver tolerance to clock deviation increased
 *
 *	__Baud Rate Calculation__
 *	Tx/Rx baud = (F_pclk / (8 * USARTDIV))  if OVER8 = 1
 *	Tx/Rx baud = (F_pclk / (16 * USARTDIV))  if OVER8 = 0
 *
 *	General Formula:
 *	Tx/Rx baud = (F_pclk / (8 * (2 - OVER8) * USARTDIV))
 *
 *	Example:
 *	F_pclk = 16MHz (USART peripheral clock)
 *	Tx/Rx baud = 9600bps (Desired baud rate)
 *	OVER8 = 0 (Oversampling by 16)
 *
 *	USARTDIV = 16M / (8 * 2 * 9600) = 104.1875
 *
 *	The baud rate for Rx and Tx are both set to the same value as programmed in the Mantissa and Fraction values of USARTDIV
 *
 *	Now this value we have to convert into hex and then program the USART_BRR register to achieve desired baud rate.
 *	USART uses a fractional baud rate generator - with a 12-bit mantissa and 4-bit fraction
 *
 *				USART_BRR
 *	Mantissa.Fraction = 104.1875
 *	Mantissa = 104	 	DIV_Mantissa[15:4]
 *	Fraction = 1875  	DIV_Fraction[3:0]
 *
 *	USARTDIV = 104.1875 (For baudrate 9600bps with F_pclk = 16MHz and OVER8 = 0)
 *	Div_Fraction = 0.1875 * 16 = 3
 *	Div_Mantissa = 104 = 0x68
 *	USARTDIV = 0x683  (Program this value into USART_BRR register to generate baudrate of 9600bps)
 *
 *	CP2102 USB to UART converter chip exists on ST development boards to convert USB signal of PC to/from MCU UART signal.
 *	Can also use a USB to UART converter cable.
 */

#include "stm32f446xx_usart_driver.h"


/******************* Private Helper Functions *******************/


/* Peripheral Clock Setup */
/* ******************************************************************************
 * @fn					-  USART_PClockControl
 *
 * @brief				-  This function enables or disables the peripheral clock
 * 						   for the given USART peripheral.
 *
 * @param[in]			-  Base address of the USART peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_PClockControl(USART_RegDef_t *pUSARTx, uint8_t State) {

	//Enable peripheral clock for specified USART peripheral 1..3
	if(State == ENABLE) {

		if(pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_EN();
		}

	//Disable peripheral clock for specified USART peripheral 1..6
	} else {

		if(pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if(pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if(pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if(pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if(pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if(pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}


/* Initialization and Deinitialization */
/* ******************************************************************************
 * @fn					-  USART_Init
 *
 * @brief				-  Initializes the USART peripheral
 *
 * @param[in]			-  Base address of the USART handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle) {

	uint32_t tempreg = 0;

/* ************************************** Configuration of CR1 ************************************** */

	//Enable the clock for the given USART peripheral
	USART_PClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		//Enable the receiver bit field
		tempreg |= (1 << USART_CR1_RE);

	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		//Enable the transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	} else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		//Enable both the receiver and transmitter bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	//Configure the word length configuration
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	//Configure parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		//Enable parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Enable EVEN parity
		//Not required because EVEN parity is selected by default when parity control is enabled.

	} else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
		//Enable parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Enable ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/* ************************************** Configuration of CR2 ************************************** */
	tempreg = 0;

	//Configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/* ************************************** Configuration of CR3 ************************************** */
	tempreg = 0;

	//Configure of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		//Enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);

	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		//Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		//Enable both CTS and RTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	//Program the CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg;

/* ************************************** Configuration of BRR ************************************** */
	//Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

/* ******************************************************************************
 * @fn					-  USART_DeInit
 *
 * @brief				-  De-initializes the USART peripheral
 *
 * @param[in]			-  Base address of the USART peripheral
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx) {

	if(pUSARTx == USART1) {
		USART1_REG_RESET();
	} else if(pUSARTx == USART2) {
		USART2_REG_RESET();
	} else if(pUSARTx == USART3) {
		USART3_REG_RESET();
	} else if(pUSARTx == UART4) {
		UART4_REG_RESET();
	} else if(pUSARTx == UART5) {
		UART5_REG_RESET();
	} else if(pUSARTx == USART6) {
		USART6_REG_RESET();
	}
}

/* Data Send and Receive */
/* ******************************************************************************
 * @fn					-  USART_SendData
 *
 * @brief				-  Send data over USART bus
 *
 * @param[in]			-  Base address of the USART Handle
 * @param[in]			-  Pointer to Tx buffer
 * @param[in]			-  Length of data to be sent
 *
 * @return				-  None
 *
 * @note				-  Polling based (blocking)
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint16_t *pdata;

	//Loop until Len number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++) {
		//Wait until TXE flag is set in SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		//Check the USART_WordLength for 9 or 8 bit data frame configuration
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {

			//9 Bit Data Transfer
			//If 9 bits, load the DR with 2 bytes, masking the bits other than the first 9 of 16 bits (0x1FF)
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			//Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity is used in this transfer, so 9 bits of user data will be sent
				//Increment the buffer address twice
				pTxBuffer++;
				pTxBuffer++;
			} else {
				//Parity bit is used in this transfer, so 8 bits of user data will be sent
				//The 9th bit will be replaced with parity bit by the hardware
				//Increment the buffer address
				pTxBuffer++;
			}

		} else {

			//8 Bit Data Transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait until TC flag is set in SR
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

/* ******************************************************************************
 * @fn					-  USART_ReceiveData
 *
 * @brief				-  Receive data over USART bus
 *
 * @param[in]			-  Base address of the USART Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 *
 * @return				-  None
 *
 * @note				-  Polling based (blocking)
 *
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {

	//Loop until Len number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++) {
		//Wait until RXNE flag is set in SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength for 9 or 8 bit data frame configuration
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {

			//9 Bit Data Transfer

			//Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity is used in this transfer, so 9 bits of user data will be received

				//Read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

				//Increment the buffer address twice
				pRxBuffer++;
				pRxBuffer++;

			} else {
				//Parity bit is used in this transfer, so 8 bits of user data will be received
				//The 9th bit will be replaced with parity bit by the hardware
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

				//Increment the buffer address
				pRxBuffer++;
			}

		} else {

			//8 Bit Data Transfer
			//Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				//No parity is used in this transfer, so 8 bits of user data will be received

				//Read only first 9 bits so mask the DR with 0x01FF
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);


			} else {
				//Parity bit is used in this transfer, so 7 bits of user data will be received
				//The 8th bit will be replaced with parity bit by the hardware
				//Read only 7 bits, masking DR with 0x7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);

			}

			//Increment the buffer address
			pRxBuffer++;
		}
	}
}

/* ******************************************************************************
 * @fn					-  USART_SendDataIT
 *
 * @brief				-  Send data over USART bus (IT)
 *
 * @param[in]			-  Pointer to base address of the USART Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 *
 * @return				-  Busy state
 *
 * @note				-  Interrupt based (non-blocking)
 *
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/* ******************************************************************************
 * @fn					-  USART_ReceiveDataIT
 *
 * @brief				-  Receive data over USART bus (IT)
 *
 * @param[in]			-  Pointer to base address of the USART Handle
 * @param[in]			-  Pointer to Rx buffer
 * @param[in]			-  Length of data to be read
 *
 * @return				-  Busy state
 *
 * @note				-  Interrupt based (non-blocking)
 *
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {

	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


/* IRQ Configuration, Priority and ISR Handling */
/* ******************************************************************************
 * @fn					-  USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State) {

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
 * @fn					-  USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

	//1. First calculate for the correct IPR register
	//IPRx registers are 32 bits wide, split into four sections of 1 Byte (8-bits): | IPRn || IRQ4n+3_PRI | IRQ4n+2_PRI | IRQ4n+1_PRI | IRQ4n_PRI |
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	//Within each 8-bit (byte) IRQ4n_PRI field, only the upper 4 bits are implemented (4:7), while the lower 4 are not (0:3).
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ******************************************************************************
 * @fn					-  USART_IRQHandling
 *
 * @brief				-  Configure USART IRQ Handling
 *
 * @param[in]			-  Pointer to base address of the USART Handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {

	uint32_t temp1, temp2;
	//uint32_t temp3;

	uint16_t *pdata;

/* ************************ Check for TC flag ******************************************* */

	//Check state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	//Check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2) {

		//This interrupt is due to TC
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {

			//Check the TxLen. If it is zero, then close data transmission
			if(! pUSARTHandle->TxLen) {

				//Clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Clear the TCIE control bit
				//pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application callback with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/* ************************ Check for TXE flag ******************************************* */

	//Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2) {

		//This interrupt is because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {

			//Keep sending data until TxLen reaches zero
			if(pUSARTHandle->TxLen > 0) {

				//Check the USART_WordLength item for 9 bit or 8 bit in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {

					//9 Bit Data Transfer

					//If 9 bits, load DR with 2 bytes, masking the bits other than the first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

					//Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {

						//9 Bit Data Transfer
						//No parity is used in this transfer, so 9 bits of user data will be sent
						//Increment the buffer address twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;

					} else {
						//Parity bit is used in this transfer, so 8 bits of user data will be sent
						//The 9th bit will be replaced with parity bit by the hardware

						//Increment the buffer address
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}

				} else {

					//8 Bit Data Transfer

					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);

					//Increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}
			}

			if (pUSARTHandle->TxLen == 0) {
				//TxLen is zero
				//Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/* ************************ Check for RXNE flag ******************************************* */
	//Check the state of RXNE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);

	//Check the state of RXNEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2) {

		//This interrupt is because of RXNE
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {

			//Keep receiving data until RxLen reaches zero
			if(pUSARTHandle->RxLen > 0) {

				//Check the USART_WordLength item for 9 bit or 8 bit in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {

					//9 Bit Data Transfer

					//Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {

						//No parity is used in this transfer, so 9 bits of user data will be sent
						//Read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

						//Increment the buffer address twice
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;

					} else {
						//Parity bit is used in this transfer, so 8 bits of user data will be received
						//The 9th bit will be replaced with parity bit by the hardware
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

						//Increment the buffer address
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}

				} else {

					//8 Bit Data Transfer

					//Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
						//No parity is used in this transfer, so 8 bits of user data will be received
						//Read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

					} else {
						//Parity bit is used in this transfer, so 7 bits of user data will be received
						//The 8th bit will be replaced with parity bit by the hardware
						//Read only 7 bits, mask DR with 0x7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
					}

					//Increment the buffer address
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}
			}

			//If Len = 0
			if (!pUSARTHandle->RxLen) {
				//RxLen is zero
				//Clear the RXNEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

/* ***************************** Check for CTS flag ******************************************* */
//Note: CTS feature is not applicable for UART4 and UART5

	//Check status of CTS bit in SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Check status of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	//Check status of CTSIE bit in CR3 (This bit is not available for UART4 and UART5)
	//temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2) {
		//Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//This interrupt is because of CTS
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

/* ************************ Check for IDLE detection flag *************************************** */

	//Check status of IDLE flag bit in SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Check status of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2) {
		//Clear the IDLE flag in SR. Refer to RM to understand clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		//This interrupt is because of IDLE
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

/* ************************ Check for Overrun detection flag ************************************ */

	//Check status of ORE flag bit in SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Check status of RXNEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1 && temp2) {
		//Don't need to clear the ORE flag here, instead give an API for the application to clear the ORE flag

		//This interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

/* ********************************* Check for Error flag *************************************** */
//Noise Flag, Overrun Error and Framing Error in multibuffer communication
//We don't discuss multibuffer communication in this course. Please refer to RM
//The below code will get executed only if multibuffer mode is used.

	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if(temp2) {
		temp1 = pUSARTHandle->pUSARTx->SR;

		if(temp1 & (1 << USART_SR_FE)) {
			//This bit is set by hardware when a de-synchronization, excessive noise or a break character
			//is detected. It is cleared by a software sequence (a read to USART_SR followed by a read to USART_DR)
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NF)) {
			//This bit is set by hardware when noise is detected on a received frame. It is cleared by a software
			//sequence (a read to USART_SR followed by a read to USART_DR)
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NF);
		}

		if(temp1 & (1 << USART_SR_ORE)) {
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}

/* Other Peripheral Control APIs */
/* ******************************************************************************
 * @fn					-  USART_PeripheralControl
 *
 * @brief				-  Enable or disable USARTx peripheral
 *
 * @param[in]			-  Base address of the USART peripheral
 * @param[in]			-  ENABLE or DISABLE macros
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t State) {

	if(State == ENABLE) {
		pUSARTx->CR1 |= (1 << 13);

	} else {
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/* ******************************************************************************
 * @fn					-  USART_GetFlagStatus
 *
 * @brief				-  Retrieve flag status of USART peripheral
 *
 * @param[in]			-  Base address of the USART peripheral
 * @param[in]			-  Flag name
 *
 * @return				-  Byte containing flag status info
 *
 * @note				-  None
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName) {

	if(pUSARTx->SR & FlagName) {
		return SET;
	}

	return RESET;
}

/* ******************************************************************************
 * @fn					-  USART_ClearFlag
 *
 * @brief				-  Clear flag of USART peripheral
 *
 * @param[in]			-  Base address of the USART peripheral
 * @param[in]			-  Flag name
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName) {

	pUSARTx->SR &= ~(FlagName);
}

/* ******************************************************************************
 * @fn					-  USART_SetBaudRate
 *
 * @brief				-  Sets Baud Rate (bits per second) of USART communication
 *
 * @param[in]			-  Base address of the USART peripheral
 * @param[in]			-  Baud rate
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//Variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock into the variable PCLKx
	if((pUSARTx == USART1) || (pUSARTx == USART6)) {
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for Over8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//Over8 = 1, over-sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		//Over8 = 0, over-sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Determine the Mantissa (Integer) part
	M_part = usartdiv / 100;

	//Store the Mantissa part
	tempreg = (M_part << 4);

	//Determine the Fractional part
	F_part = (usartdiv - (100 * M_part));

	//Store the Fractional part
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		//Over8 = 1, over-sampling by 8
		tempreg |= (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	} else {
		//Over8 = 0, over-sampling by 16
		tempreg |= (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	//Copy the value of tempreg into BRR
	pUSARTx->BRR = (uint16_t)tempreg;
}

/* Application Callback */
/* ******************************************************************************
 * @fn					-  USART_ApplicationEventCallback
 *
 * @brief				-  Application Event Callback (weak implementation)
 *
 * @param[in]			-  Pointer to USART handle
 * @param[in]			-  Application event byte
 *
 * @return				-  None
 *
 * @note				-  Weak implementation
 *
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv) {

	//This is a weak implementation -- the application may override this function.
}

