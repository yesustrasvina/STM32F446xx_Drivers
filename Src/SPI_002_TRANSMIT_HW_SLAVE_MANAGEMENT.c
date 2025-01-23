/*
 * SPI_001_TRANSMIT_HW_SLAVE_MANAGEMENT.c
 *
 *  Created on: Nov 18, 2024
 *      Author: Yesus
 */

#include <stdint.h>
#include <string.h> //For memset
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

void GPIOxInit(void);
void SPI2_Init(void);
void Delay(void);

/*Default clock: 16 MHZ HSI CLOCK*/
/*SPI2 PINS*/
//PB15 MOSI
//PB14 MISO
//PB13 SCK
//PB12 NSS

/*BUTTON*/
//PC13
/*LED*/
//PA5

uint8_t flag = 0;

int main(void)
{
	char data_to_send[] = "YESUS";

	GPIOxInit();
	GPIO_IRQInterruptConfig(EXTI15_10_IRQn, ENABLE);
	GPIO_IRQPriorityConfig(EXTI15_10_IRQn, NVIC_IRQ_PRI5);
	SPI2_Init();
	SPI_SSOEConfig(SPI2, ENABLE); /*SSM = 0, SSOE = 1*/


	while(1)
	{
		if(flag == 1)
		{
			Delay(); //Debounce
			SPI_PeripheralControl(SPI2, ENABLE); /*ENABLE SPI*/
			SPI_SendData(SPI2, (uint8_t*)data_to_send, strlen(data_to_send));
			SPI_PeripheralControl(SPI2, DISABLE); /*DISABLE SPI*/
			flag = 0;
		}
	}
	return 0;
}


void GPIOxInit(void)
{
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Handle_t LED;

	memset (&LED,0,sizeof(LED)); //Initialize all in 0

	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&LED);

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Handle_t BUTTON;

	memset (&BUTTON,0,sizeof(BUTTON)); //Initialize all in 0

	BUTTON.pGPIOx = GPIOC;
	BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PD;
	BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&BUTTON);

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t SPI2Pins;

	memset (&SPI2Pins,0,sizeof(SPI2Pins)); //Initialize all in 0

	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUTPUT_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MOSI
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI2Pins);

	//SCK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI2Pins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV4;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_HW;

	SPI_Init(&SPI2Handle);
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_13);
	flag = 1;
 }

void Delay(void)
{
	for(uint32_t i = 0; i <= 60000; i++);
}
