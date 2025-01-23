/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Oct 21, 2024
 *      Author: Yesus
 */
#include "stm32f446xx_gpio_driver.h"

/*Peripheral clock setup*/

/*******************************************
 * @fn                -GPIO_PeriClockControl
 *
 * @brief             -This functions enables or disables peripheral clock for the giving GPIO port
 *
 * @pGPIOx            -Base address of the GPIO peripheral
 * @EnorDi            -Enable or disable MACRO
 *
 * @return            -None
 * */

void GPIO_PeriClockControl(GPIO_Reg_Def_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_ENABLE();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLOCK_DISABLE();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLOCK_DISABLE();
		}
	}

}

/*Init and De-init*/
/*******************************************************
 * @fn                     -GPIO_init
 *
 * @brief                  -Initializes the GPIO pin
 *
 * @param pGPIO_Handle     -Pointer to the GPIO handle structure
 *
 * @return                 -None
 * */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1.- Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//temp = (config << position)
		//temp = (PinMode << PinNumber)
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Example (Output 1) << (2 * (PinNumber 2)) = 4 so (1 << 4)
		pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
		pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;
	}
	else
	{
		temp = 0;
		//EXTI
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			temp = (GPIO_MODE_INPUT << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
			pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;
			//1.- Configure the FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2.- Clear RTSR
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			temp = (GPIO_MODE_INPUT << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
			pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;
			//1.- Configure the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2.- Clear FTSR
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT_RT)
		{
			temp = (GPIO_MODE_INPUT << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
			pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;
			//1.- Configure the FTSR
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//2.- Configure the RTSR
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t tmp1 = 0,tmp2 = 0;
		//Calculate the register to write PinNumber/ElementsInRegister
		tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		//Calculate position
		tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERI_CLOCK_ENABLE();
		//Select input for EXTI LINE
		SYSCFG->SYSCFG_EXTICR[tmp1] |= (portcode << (tmp2 * 4));

		// Enable the EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;
	//2.- Configure the speed of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR &= (0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR |= temp;

	temp = 0;
	//3.- Configure the PUPD of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clear first
	pGPIOHandle->pGPIOx->GPIOx_PUPDR |= temp;

	temp = 0;
	//4.- Configure the OTYPE of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OTYPER |= temp;

	temp = 0;
	//5.- Configure the ALTF of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
	{
		uint8_t temp1= 0 ,temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; //Register Low or High
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; //Position
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/***************************************************
 * @fn               - GPIO_DeInit
 *
 * @brief            - De-initalizes the GPIO pin
 *
 * @param GPIOx      - Base address of the GPIO peripheral
 *
 * @return           - None
 * */
void GPIO_DeInit(GPIO_Reg_Def_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->GPIOx_IDR >> PinNumber) & 0x01);

	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_Reg_Def_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->GPIOx_IDR);

	return value;
}

void GPIO_WriteToOutputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber, uint8_t Value)
{
	if(Value == ENABLE)
	{
		pGPIOx->GPIOx_BSRR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->GPIOx_BSRR |= (1 << (PinNumber + 16));
	}
}

void GPIO_WriteToOutputPort(GPIO_Reg_Def_t *pGPIOx, uint16_t Value)
{
	pGPIOx->GPIOx_ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_Reg_Def_t *pGPIOx, GPIO_Pin PinNumber)
{
	pGPIOx->GPIOx_ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(IRQn_Type IRQNumber, uint8_t EnorDi)
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
void GPIO_IRQPriorityConfig(IRQn_Type IRQNumber, NVIC_Priority IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_value = (iprx_section * 8) + (4);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value);
}
void GPIO_IRQHandling(GPIO_Pin PinNumber)
{
	if(EXTI->EXTI_PR & (1 << PinNumber)) //Flag set to 1?
	{
		//Clear
		EXTI->EXTI_PR = (1 << PinNumber);
	}
}
