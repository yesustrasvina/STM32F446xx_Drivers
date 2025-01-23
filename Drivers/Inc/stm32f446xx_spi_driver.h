/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Nov 11, 2024
 *      Author: Yesus
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for SPI
 * */
typedef struct
{
	uint8_t SPI_DeviceMode; /*Master, slave @SPI_DeviceMode*/
	uint8_t SPI_BusConfig;  /*Full duplex, Half duplex, Simplex @SPI_BusConfig*/
	uint8_t SPI_SclkSpeed;  /*Speed selection @SPI_SclkSpeed*/
	uint8_t SPI_DFF;        /*Data size 8 or 16 bits @SPI_DFF*/
	uint8_t SPI_CPOL;       /*@SPI_CPOL*/
	uint8_t SPI_CPHA;       /*SPI_CPHA*/
	uint8_t SPI_SSM;        /**Software Slave Management,  Hardware or Software @SPI_SSM*/
}SPI_Config_t;

/*
 * Handler structure for SPI
 * */
typedef struct
{
	SPI_Reg_Def_t *pSPIx;      /*Holds the base address of the SPI*/
	SPI_Config_t  SPI_Config;  /*Holds SPI configuration settings*/
	uint8_t 	  *pTxBuffer;  /*Stores TX buffer address*/
	uint32_t  	  TxLen;       /*Stores TX length*/
	uint8_t       TxState;     /*To know the current TX state*/
	uint8_t		  RxLen;	   /*Stores the RX length*/
	uint8_t 	  RxState;	   /*To know the current RX state*/
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * */
#define SPI_DEVICE_MODE_SLAVE  0
#define SPI_DEVICE_MODE_MASTER 1

/*
 * @SPI_BusConfig
 * */
#define SPI_BUS_CONFIG_FD        1 /*FULL DUPLEX*/
#define SPI_BUS_CONFIG_HD        2 /*HALF DUPLEX*/
#define SPI_BUS_CONFIG_S_RXONLY  3 /*SIMPLEX Receive only*/

/*
 * @SPI_SclkSpeed
 * Baud rate control
 */
#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7

/*
 * @SPI_DFF
 * */
#define SPI_DFF_8_BITS   0
#define SPI_DFF_16_BITS  1

/*
 * @SPI_CPOL
 * */
#define SPI_CPOL_LOW     0
#define SPI_CPOL_HIGH    1

/*
 * @SPI_CPHA
 * */
#define SPI_CPHA_FIRST   0
#define SPI_CPHA_SECOND  1

/*
 * @SPI_SSM
 * */
#define SPI_SSM_HW       0
#define SPI_SSM_SW       1

/*
 * Possible SPI Application States
 * */
#define SPI_READY       0
#define SPI_BUSY_IN_RX  1
#define SPI_BUSY_IN_TX  2

/*Other useful Macros*/

/*REGISTER BITS POSITIONS*/
/*CR1 Register*/
#define CPHA_BIT 		 0
#define CPOL_BIT         1
#define MSTR_BIT         2
#define BR_BIT           3
#define SPE_BIT          6
#define LSBFIRST_BIT     7
#define SSI_BIT          8
#define SSM_BIT          9
#define RXONLY_BIT       10
#define DFF_BIT          11
#define CRXNEXT_BIT      12
#define CRCEN_BIT        13
#define BIDIOE_BIT       14
#define BIDIMODE_BIT     15

/*CR2 Register*/
#define SSOE_BIT         2
#define ERRIE_BIT        5
#define RXNEIE_BIT       6
#define TXEIE_BIT		 7

/*STATUS REGISTER*/
#define RXNE_BIT        0
#define TXE_BIT         1
#define OVR_BIT         6

/*Flags status*/
#define SPI_TXE_FLAG    (1 << TXE_BIT)
#define SPI_RXNE_FLAG   (1 << RXNE_BIT)

/******APIs supported  for this driver******/
/*Peripheral clock setup*/
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi);

/*Init and De-init*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Reg_Def_t *pSPIx);

/*Data send and receive*/
/*Funciones bloqueantes*/
void SPI_SendData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

/*IRQ configuration and ISR handling*/
void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, NVIC_Priority IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle); /*When an interrupts occurs the user application can call this function*/

/*Other SPI APIs*/
uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName);
void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi);
void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx , uint8_t EnorDi);
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx , uint8_t EnorDi);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
