/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Feb 5, 2021
 *      Author: Oliver
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/* Configuration Structure for SPIx Peripheral */
typedef struct {
	uint8_t SPI_DeviceMode;		/* possible values from @SPI_DEVICEMODE */
	uint8_t SPI_BusConfig;		/* possible values from @SPI_BUSCONFIG */
	uint8_t SPI_SclkSpeed;		/* possible values from @SPI_SCLKSPEED */
	uint8_t SPI_DFF;			/* possible values from @SPI_DFF */
	uint8_t SPI_CPOL;			/* possible values from @SPI_CPOL */
	uint8_t SPI_CPHA;			/* possible values from @SPI_CPHA */
	uint8_t SPI_SSM;			/* possible values from @SPI_SSM */
} SPI_Config_t;

/* Handle Structure for SPIx Peripheral */
typedef struct {
	SPI_RegDef_t *pSPIx;		/* This holds the base address of SPIx (x:0,1,2) peripheral */
	SPI_Config_t SPIConfig;		/* This contains the SPI peripheral configuration options */
	uint8_t	*pTxBuffer;			/* To store the application Tx buffer address */
	uint8_t	*pRxBuffer;			/* To store the application Rx buffer address */
	uint32_t TxLen;				/* To store the Tx length */
	uint32_t RxLen;				/* To store the Rx length */
	uint8_t	TxState;			/* possible values from @SPI_ApplicationStates */
	uint8_t	RxState;			/* possible values from @SPI_ApplicationStates */
} SPI_Handle_t;

/* SPI Application States: @SPI_ApplicationStates */
#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2

/* Possible SPI Application Events */
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4

/* SPI Device Modes: @SPI_DEVICEMODE */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/* SPI Bus Configurations: @SPI_BUSCONFIG */
#define SPI_BUS_CONFIG_FD					1		//full duplex
#define SPI_BUS_CONFIG_HD					2		//half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3		//simplex receive only

/* SPI Serial Clock Speed: @SPI_SCLKSPEED */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/* SPI Data Frame Format: @SPI_DFF */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/* SPI Clock Polarity: @SPI_CPOL */
#define SPI_CPOL_LOW						0		//Low clock position when idle
#define SPI_CPOL_HIGH						1		//High clock position when idle

/* SPI Clock Phase: @SPI_CPHA */
#define SPI_CPHA_LOW						0		//Data capture begins on first clock edge
#define SPI_CPHA_HIGH						1		//Data capture begins on second clock edge

/* SPI Software Slave Management: @SPI_SSM */
	//When SSM bit is set, NSS Pin Input is replaced with value of SSI bit.
#define SPI_SSM_DI							0		//Software slave management disabled
#define SPI_SSM_EN							1		//Software slave management enabled

/* SPI Related Status Flag Definitions */
#define SPI_FLAG_RXNE						(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE						(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE						(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR						(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR						(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF						(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR						(1 << SPI_SR_OVR)
#define SPI_FLAG_BUSY						(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE						(1 << SPI_SR_FRE)

/***********************************************************************************
 * 							APIs supported by this driver
 * 			For more info about the APIs, check the function definitions
 **********************************************************************************/

/* Peripheral Clock Setup */
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t State);

/* Initialization and Deinitialization */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration, Priority and ISR Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t State);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* Other Peripheral Control APIs */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t State);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t State);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* Application Callback */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
