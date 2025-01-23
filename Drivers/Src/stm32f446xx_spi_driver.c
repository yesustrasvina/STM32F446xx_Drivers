/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Yesus
 */

#include "stm32f446xx_spi_driver.h"

void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx ==  SPI1)
		{
			SPI1_PERI_CLOCK_ENABLE();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERI_CLOCK_ENABLE();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERI_CLOCK_ENABLE();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PERI_CLOCK_ENABLE();
		}
	}
	else
	{
		if(pSPIx ==  SPI1)
		{
			SPI1_PERI_CLOCK_DISABLE();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERI_CLOCK_DISABLE();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERI_CLOCK_DISABLE();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PERI_CLOCK_DISABLE();
		}
	}
}

void SPI_Init (SPI_Handle_t *pSPIHandle)
{
	/*Peripheral clock enable*/
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configure CR1
	uint32_t tempreg = 0;
	//1.- Device Mode (MASTER/SLAVE)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << MSTR_BIT);
	//2-. Bus configuration (FD,HD, SIMPLEX)
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI MODE must be 0 for FULL DUPLEX (2 line unidirectional)
		tempreg &= ~(1 << BIDIMODE_BIT);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI MODE must be 1 for HALF DUPLEX (1 line bidirectional)
		tempreg |= (1 << BIDIMODE_BIT);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		//BIDI MODE must be 0 for SIMPLEX RX ONLY
		tempreg &= ~(1 << BIDIMODE_BIT);
		//BIT RX ONLY must be 1 (enables simplex RX ONLY)
		tempreg |= (1 << RXONLY_BIT);
	}
	//3.-SPEED CONFIGURATION (Baud rate control)
	tempreg &= ~(7 << BR_BIT); //Clear first
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << BR_BIT);

	//4.- Data frame format (Data Size 8 bits or 16 bits)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << DFF_BIT);

	//5.- Clock polarity CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << CPOL_BIT);

	//6.- Clock phase CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << CPHA_BIT);

	//7.- Software slave management
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SSM_BIT);

	//CR1 CONFIGURATION
	pSPIHandle->pSPIx->SPI_CR1 |= tempreg;
}

void SPI_SendData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while((int8_t)Len > 0)
	{
		//Wait for TXE to be 1 (TX BUFFER EMPTY)
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == DISABLE)
		{

		}
		if((pSPIx->SPI_CR1>>11 & 1)  == SPI_DFF_8_BITS)
		{
			//8 bit data size
			//Write 8 bits of Data in Data Register
			*((__vo uint8_t*)&pSPIx->SPI_DR) = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
		else if((pSPIx->SPI_CR1>>11 & 1)  == SPI_DFF_16_BITS)
		{
			//16 bit data size
			//Write 16 bits of Data in Data Register
			*((__vo uint16_t*)&pSPIx->SPI_DR) = *((uint16_t*)pTxBuffer);
			Len-=2;
			pTxBuffer+=2;
		}
	}
}

void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	//Wait for RXNE to be 1 (RX buffer no empty)
	while((int8_t)Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == DISABLE)
		{

		}
		if((pSPIx->SPI_CR1>>11 & 1)  == SPI_DFF_8_BITS)
		{
			//8 bit data size
			//Read 8 bits of Data in Data Register
			*pTxBuffer = *((__vo uint8_t*)&pSPIx->SPI_DR);
			Len--;
			pTxBuffer++;
		}
		else if((pSPIx->SPI_CR1>>11 & 1)  == SPI_DFF_16_BITS)
		{
			//16 bit data size
			//Read 16 bits of Data in Data Register
			 *((uint16_t*)pTxBuffer) = *((__vo uint16_t*)&pSPIx->SPI_DR);
			Len-=2;
			pTxBuffer+=2;
		}
	}
}

/********************************************************************
 * @fn                    - SPI_GetFlagStatus
 *
 * @brief                 - Obtains the state of a flag
 *
 * @param SPI_Reg_Def_t   - Base address of the SPI peripheral
 * @param FlagName        - Flag to set
 *
 * @return                - none
 */

uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return ENABLE;
	}
	return DISABLE;
}

void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SSI_BIT);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SSI_BIT);
	}
}

void SPI_PeripheralControl(SPI_Reg_Def_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPE_BIT);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPE_BIT);
	}
}
void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SSOE_BIT);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << SSOE_BIT);
	}
}

void SPI_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnorDi)
{
	 if(EnorDi == ENABLE)
	 {
		 if(IRQNumber <= 31)
		 {
			 *NVIC_ISER0 |= (1 << IRQNumber);
		 }
		 else if(IRQNumber > 31 && IRQNumber <= 63)
		 {
			 *NVIC_ISER1 |= (1 << IRQNumber % 32);
		 }
		 else if(IRQNumber > 63 && IRQNumber <= 95)
		 {
			 *NVIC_ISER2 |= (1 << IRQNumber % 64);
		 }
	 }
	 else
	 {
		 if(IRQNumber <= 31)
		 {
			 *NVIC_ICER0 = (1 << IRQNumber);
		 }
		 else if(IRQNumber > 31 && IRQNumber <= 63)
		 {
			 *NVIC_ICER1 = (1 << IRQNumber % 32);
		 }
		 else if(IRQNumber > 63 && IRQNumber <= 95)
		 {
			 *NVIC_ICER2 = (1 << IRQNumber % 64);
		 }
	 }
}

void SPI_IRQPriorityConfig(IRQn_Type IRQNumber, NVIC_Priority IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_value = (iprx_section * 8) + (4);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	//FIRST CHECK TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << TXE_BIT);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << TXEIE_BIT);

	if(temp1 && temp2)
	{
		//Handle TXE
	}

	//CHECK RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << RXNE_BIT);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << RXNEIE_BIT);

	if(temp1 && temp2)
	{
		//Handle RXNE
	}

	//CHECK OVR FLAG
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 << OVR_BIT);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << ERRIE_BIT);

	if(temp1 && temp2)
	{
		//Handle OVR FLAG
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle.TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1.- Save the  Tx  buffer address and len information in some global variables
	pSPIHandle.pTxBuffer = pTxBuffer;
	pSPIHandle.TxLen = Len;
	//2.-Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
	pSPIHandle.TxState = SPI_BUSY_IN_TX;
	//3.- Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle.pSPIx->SPI_CR2 |= (ENABLE << TXEIE_BIT);
	}
	//4.- Data transmission will be handled by the ISR code
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle.RxState;

		if(state != SPI_BUSY_IN_RX)
		{
		//1.- Save the  Tx  buffer address and len information in some global variables
		pSPIHandle.pTxBuffer = pTxBuffer;
		pSPIHandle.TxLen = Len;
		//2.-Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle.RxState = SPI_BUSY_IN_RX;
		//3.- Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle.pSPIx->SPI_CR2 |= (ENABLE << RXNEIE_BIT);
		}
		//4.- Data transmission will be handled by the ISR code
		return state;
}
