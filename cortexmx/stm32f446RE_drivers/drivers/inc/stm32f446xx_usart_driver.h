/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Mar 12, 2021
 *      Author: Oliver
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/* Configuration Structure for USARTx Peripheral */
typedef struct {
	uint8_t  USART_Mode;					/* possible values from @USART_Mode */
	uint32_t USART_Baud;					/* possible values from @USART_Baud */
	uint8_t  USART_NoOfStopBits;			/* possible values from @USART_NoOfStopBits */
	uint8_t  USART_WordLength;				/* possible values from @USART_WordLength */
	uint8_t  USART_ParityControl;			/* possible values from @USART_ParityControl */
	uint8_t  USART_HWFlowControl;			/* possible values from @USART_HWFlowControl */
} USART_Config_t;

/* Handle Structure for USARTx Peripheral */
typedef struct {
	USART_RegDef_t *pUSARTx; 				/* Holds the base address of the USARTx peripheral */
	USART_Config_t USART_Config; 			/* Holds USARTx peripheral configuration settings */
	uint8_t  *pTxBuffer;					/* To store the application Tx buffer address */
	uint8_t  *pRxBuffer;					/* To store the application Rx buffer address */
	uint32_t TxLen;							/* To store the Tx length */
	uint32_t RxLen;							/* To store the Rx length */
	uint8_t  TxBusyState;					/* possible values from @USART_ApplicationStates */
	uint8_t  RxBusyState;					/* possible values from @USART_ApplicationStates */
} USART_Handle_t;


/* @USART_Mode */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/* @USART_Baud */
#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			2400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200 		19200
#define USART_STD_BAUD_38400 		38400
#define USART_STD_BAUD_57600 		57600
#define USART_STD_BAUD_115200 		115200
#define USART_STD_BAUD_230400 		230400
#define USART_STD_BAUD_460800 		460800
#define USART_STD_BAUD_921600 		921600
#define USART_STD_BAUD_2M 			2000000
#define USART_STD_BAUD_3M 			3000000


/* @USART_NoOfStopBits */
#define USART_STOPBITS_1     		0
#define USART_STOPBITS_0_5   		1
#define USART_STOPBITS_2     		2
#define USART_STOPBITS_1_5   		3

/* @USART_WordLength */
#define USART_WORDLEN_8BITS 		0
#define USART_WORDLEN_9BITS 		1

/* @USART_ParityControl */
#define USART_PARITY_EN_ODD   		2
#define USART_PARITY_EN_EVEN  		1
#define USART_PARITY_DISABLE   		0

/* Hardware Flow Control : @USART_HWFlowControl */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/* USART Related Status Flag Definitions */
#define USART_FLAG_PE				(1 << USART_SR_PE)
#define USART_FLAG_FE				(1 << USART_SR_FE)
#define USART_FLAG_NF				(1 << USART_SR_NF)
#define USART_FLAG_ORE				(1 << USART_SR_ORE)
#define USART_FLAG_IDLE				(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE				(1 << USART_SR_RXNE)
#define USART_FLAG_TC				(1 << USART_SR_TC)
#define USART_FLAG_TXE				(1 << USART_SR_TXE)
#define USART_FLAG_LBD				(1 << USART_SR_LBD)
#define USART_FLAG_CTS				(1 << USART_SR_CTS)

/* USART Application States : @USART_ApplicationStates */
#define USART_READY					0
#define USART_BUSY_IN_RX			1
#define USART_BUSY_IN_TX			2

/* USART Application Events Macro */
#define USART_EVENT_TX_CMPLT   		0
#define	USART_EVENT_RX_CMPLT   		1
#define	USART_EVENT_IDLE      		2
#define	USART_EVENT_CTS       		3
#define	USART_EVENT_PE        		4
#define	USART_ERR_FE     			5
#define	USART_ERR_NF    	 		6
#define	USART_ERR_ORE    			7

/***********************************************************************************
 * 							APIs supported by this driver
 * 			For more info about the APIs, check the function definitions
 **********************************************************************************/

/* Peripheral Clock Setup */
void USART_PClockControl(USART_RegDef_t *pUSARTx, uint8_t State);

/* Initialization and Deinitialization */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/* Data Send and Receive */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration, Priority and ISR Handling */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/* Other Peripheral Control APIs */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t State);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/* Application Callback */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
